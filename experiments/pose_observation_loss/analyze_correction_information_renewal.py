#!/usr/bin/env python3
"""Reconstruct observation and anchor sharing across every late correction."""

import argparse
from collections import defaultdict
import csv
from itertools import product
import json
import math
from pathlib import Path
import statistics

from analyze_commanded_recovery_arc import flag
from analyze_correction_centre_transitions import (
    OFFSETS,
    KINDS,
    checked_poses,
    opportunity_kind,
)
from analyze_epoch_position_components import PARTS, Replay, file_digest, transported
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import KEY
from analyze_speed_probe_shared_information import ar_covariance, model
from analyze_terminal_arc_live import COHORTS, SIMULATOR
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/correction-centre-transitions"
PROTOCOL = ROOT / "results/correction-information-renewal/protocol.md"
PINNED = {
    "accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "events.csv": "819b026c74884681e8df9fb282998a19ade0d12f82e8d1ffa3ad1a3b5ce1df6f",
    "snapshots.csv": "9d25ee780dcb7695efee4c586509f8fe9c65edac9bb00b0cc5e4d16f2b8b5fcf",
    "sources.json": "e3dbe3ea3861f520035449371d2740bb12d5304df4dc09101589d2dc31fd71c5",
}
RAW_SHA = "ccd59c809aa90313f8f96af8857dca180ce8b3b92600b57fb14757a0edca4da4"
SHARED_SHA = "2607bc45964dda48ee5492405089b937b5435738f179d59b9842c2628019db33"
DT, EPS = 0.02, 1e-12
INPUTS = ["actual", "preview"]
ESTIMATORS = [
    "all",
    "recovery_mean",
    "raw_fallback",
    "three_epoch",
    "motion_filter",
    "unavailable",
]
ACCURACY = [
    "both_inside",
    "inside_to_outside",
    "outside_to_inside",
    "both_outside",
    "unavailable",
]
INFO_FIELDS = [
    "estimator",
    "current_anchor_id",
    "observation_count",
    "anchor_count",
    "anchor_signature",
    "rho",
    "noise_variance",
    "anchor_variance",
    "joint_variance",
    "error_x",
    "error_y",
    "error_norm",
]


def information(groups, truth, odom, rho):
    assert groups and abs(sum(g["weight"] for g in groups) - 1) < EPS
    noise, anchors, scales, anchor_info = {}, defaultdict(float), defaultdict(float), {}
    vectors = {p: [0.0, 0.0] for p in PARTS}
    details = []
    for index, g in enumerate(groups, 1):
        samples, weight = list(g["samples"]), g["weight"]
        assert samples and weight > 0
        first = samples[0]
        assert all(
            s.anchor_id == first.anchor_id
            and s.anchor_sigma == first.anchor_sigma
            and s.anchor == first.anchor
            for s in samples
        )
        assert first.anchor_id != 0 or first.anchor_sigma == 0
        details.append(
            dict(
                group_index=index,
                epoch_step=g["epoch"],
                group_weight=weight,
                sample_count=len(samples),
                sample_steps="+".join(str(s.step) for s in samples),
                sample_sigmas="+".join(str(s.sigma) for s in samples),
                anchor_id=first.anchor_id,
                anchor_sigma=first.anchor_sigma,
                anchor_x=first.anchor[0],
                anchor_y=first.anchor[1],
            )
        )
        for s in samples:
            w = weight / len(samples)
            assert s.step not in noise
            noise[s.step] = w * s.sigma
            anchor_info.setdefault(s.anchor_id, (s.anchor_sigma, s.anchor))
            assert anchor_info[s.anchor_id] == (s.anchor_sigma, s.anchor)
            anchors[s.anchor_id] += w
            scales[s.anchor_id] += w * s.anchor_sigma
            delta, _ = transported(s, odom, g["transport"])
            terms = [
                s.noise,
                s.anchor,
                tuple(s.bias[j] - s.anchor[j] for j in range(2)),
                tuple(s.truth[j] + delta[j] - truth[j] for j in range(2)),
            ]
            for p, term in zip(PARTS, terms):
                for j in range(2):
                    vectors[p][j] += w * term[j]
    vn = ar_covariance(noise, noise, rho)
    va = sum(v * v for v in scales.values())
    return dict(
        noise=noise,
        anchors=dict(anchors),
        scales=dict(scales),
        anchor_info=anchor_info,
        vectors=vectors,
        noise_variance=vn,
        anchor_variance=va,
        rho=rho,
        details=details,
    )


def check_snapshot(state, before, row, snapshot, poses, rho):
    kind = snapshot["input_kind"]
    estimator, groups = state.view(kind)
    truth = tuple(float(before[a]) for a in ["x", "y"])
    result = information(groups, truth, state.odom, rho)
    assert estimator == snapshot["estimator"]
    maximum = 0.0
    for j, axis in enumerate(["x", "y"]):
        assert abs(float(snapshot["truth_" + axis]) - truth[j]) < 1e-8
        assert abs(float(snapshot["pose_" + axis]) - poses[kind][j]) < 1e-8
        for p in PARTS:
            error = abs(result["vectors"][p][j] - float(snapshot[p + "_" + axis]))
            assert error < 1e-8, (snapshot["snapshot_id"], p, axis, error)
            maximum = max(maximum, error)
        assert (
            abs(
                sum(result["vectors"][p][j] for p in PARTS)
                - float(snapshot["error_" + axis])
            )
            < 1e-8
        )
    assert (
        snapshot["truth_time_s"] == before["time_s"]
        and snapshot["time_s"] == row["time_s"]
    )
    assert len(result["noise"]) == int(snapshot["used_sample_count"])
    assert len(groups) == int(snapshot["source_group_count"])
    for p in ["noise", "anchor"]:
        assert (
            abs(
                result[p + "_variance"]
                - float(snapshot["model_" + p + "_sigma_per_axis"]) ** 2
            )
            < EPS
        )
    assert (
        abs(
            result["noise_variance"]
            + result["anchor_variance"]
            - float(snapshot["model_noise_anchor_sigma_per_axis"]) ** 2
        )
        < EPS
    )
    result.update(current_anchor_id=state.current.anchor_id, estimator=estimator)
    return result, maximum


class Track:
    def __init__(self, case, rho, snapshots, events):
        self.case = {k: case[k] for k in KEY}
        self.rho, self.state, self.before, self.steps = rho, Replay(), None, 0
        self.targets = defaultdict(list)
        for s in snapshots:
            assert flag(s["available"]) == (float(s["requested_time_s"]) <= 180)
            if flag(s["available"]):
                self.targets[round(float(s["time_s"]) / DT)].append(s)
        self.events = {round(float(e["event_time_s"]) / DT): e for e in events}
        self.infos, self.seen_events = {}, set()
        self.max_pose_error = self.max_component_error = 0.0

    def process(self, row):
        if float(row["time_s"]) == 0:
            assert self.before is None
            self.before = row
            return
        step = round(float(row["time_s"]) / DT)
        assert self.before is not None and step == self.steps + 1
        assert abs(float(row["noise_rho"]) - self.rho) < EPS
        assert abs(float(row["measured_speed"]) - float(self.before["v"])) < 1e-9
        self.state.advance(self.before, row)
        self.state.check_counts(row)
        if step >= 7500:
            kind = opportunity_kind(row)
            assert (kind is not None) == (step in self.events)
            if kind is not None:
                assert kind == self.events[step]["event_kind"]
                self.seen_events.add(step)
        if step in self.targets:
            poses, error = checked_poses(self.state, row)
            self.max_pose_error = max(self.max_pose_error, error)
            for s in self.targets[step]:
                result, error = check_snapshot(
                    self.state, self.before, row, s, poses, self.rho
                )
                self.max_component_error = max(self.max_component_error, error)
                assert s["snapshot_id"] not in self.infos
                self.infos[s["snapshot_id"]] = result
        self.before, self.steps = row, step

    def validate(self):
        assert self.steps == 9000 and self.seen_events == set(self.events)
        assert set(self.infos) == {
            s["snapshot_id"] for ss in self.targets.values() for s in ss
        }
        return dict(
            **self.case,
            replayed_intervals=self.steps,
            opportunities=len(self.events),
            available_snapshot_references=len(self.infos),
            physical_snapshot_times=len(self.targets),
            max_pose_error=self.max_pose_error,
            max_component_error=self.max_component_error,
        )


def accuracy(a, b):
    if b is None:
        return "unavailable"
    return {
        (True, True): "both_inside",
        (True, False): "inside_to_outside",
        (False, True): "outside_to_inside",
        (False, False): "both_outside",
    }[a <= 0.1, b <= 0.1]


def transition(event, a, b, ia, ib, event_info, intervening):
    event_anchor = event_info["current_anchor_id"]
    replaced = event_anchor != ia["current_anchor_id"]
    assert replaced == (flag(event["received"]) or flag(event["precision_activation"]))
    if replaced:
        assert event_anchor == round(float(event["event_time_s"]) / DT)
    common = dict(
        event_id=event["event_id"],
        **{k: event[k] for k in KEY},
        event_kind=event["event_kind"],
        event_time_s=event["event_time_s"],
        phase=b["phase"],
        input_kind=b["input_kind"],
        pre_snapshot_id=a["snapshot_id"],
        post_snapshot_id=b["snapshot_id"],
        available=ib is not None,
        event_anchor_id=event_anchor,
        event_anchor_replaced=replaced,
    )
    fields = [
        "pre_estimator",
        "post_estimator",
        "pre_noise_variance",
        "post_noise_variance",
        "pre_anchor_variance",
        "post_anchor_variance",
        "pre_variance",
        "post_variance",
        "shared_samples",
        "shared_anchors",
        "post_shared_anchor_weight",
        "post_event_new_anchor_weight",
        "post_other_anchor_weight",
        "post_event_anchor_weight",
        "intervening_anchor_changes",
        "pre_error",
        "post_error",
        "error_delta_x",
        "error_delta_y",
        "error_change_norm",
        "pose_jump",
        "truth_displacement",
    ]
    fields += list(model(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    values = dict.fromkeys(fields, "")
    if ib is not None:
        shared = set(ia["anchors"]) & set(ib["anchors"])
        for anchor in shared:
            assert ia["anchor_info"][anchor] == ib["anchor_info"][anchor]
        cn = ar_covariance(ia["noise"], ib["noise"], ia["rho"])
        assert ia["rho"] == ib["rho"]
        ca = sum(ia["scales"][s] * ib["scales"][s] for s in shared)
        values.update(
            model(
                ia["noise_variance"],
                ia["anchor_variance"],
                ib["noise_variance"],
                ib["anchor_variance"],
                cn,
                ca,
            )
        )
        w_shared = sum(ib["anchors"][s] for s in shared)
        w_event = ib["anchors"].get(event_anchor, 0.0)
        w_new = w_event if event_anchor not in ia["anchors"] else 0.0
        other = sum(
            w
            for anchor, w in ib["anchors"].items()
            if anchor not in shared and anchor != event_anchor
        )
        assert abs(w_shared + w_new + other - 1) < EPS
        dx, dy = [
            float(b["error_" + axis]) - float(a["error_" + axis]) for axis in ["x", "y"]
        ]
        for axis, delta in zip(["x", "y"], [dx, dy]):
            assert (
                abs(
                    float(b["pose_" + axis])
                    - float(a["pose_" + axis])
                    - (float(b["truth_" + axis]) - float(a["truth_" + axis]))
                    - delta
                )
                < 1e-8
            )
        values.update(
            pre_estimator=ia["estimator"],
            post_estimator=ib["estimator"],
            pre_noise_variance=ia["noise_variance"],
            post_noise_variance=ib["noise_variance"],
            pre_anchor_variance=ia["anchor_variance"],
            post_anchor_variance=ib["anchor_variance"],
            pre_variance=ia["noise_variance"] + ia["anchor_variance"],
            post_variance=ib["noise_variance"] + ib["anchor_variance"],
            shared_samples=len(set(ia["noise"]) & set(ib["noise"])),
            shared_anchors=len(shared),
            post_shared_anchor_weight=w_shared,
            post_event_new_anchor_weight=w_new,
            post_other_anchor_weight=other,
            post_event_anchor_weight=w_event,
            intervening_anchor_changes=intervening,
            pre_error=float(a["error_norm"]),
            post_error=float(b["error_norm"]),
            error_delta_x=dx,
            error_delta_y=dy,
            error_change_norm=math.hypot(dx, dy),
            pose_jump=math.dist(
                [float(a["pose_" + s]) for s in ["x", "y"]],
                [float(b["pose_" + s]) for s in ["x", "y"]],
            ),
            truth_displacement=math.dist(
                [float(a["truth_" + s]) for s in ["x", "y"]],
                [float(b["truth_" + s]) for s in ["x", "y"]],
            ),
        )
    delta = values["post_error"] - values["pre_error"] if ib is not None else None
    return dict(
        **common,
        **values,
        accuracy_transition=accuracy(
            float(a["error_norm"]), float(b["error_norm"]) if ib is not None else None
        ),
        error_change=(
            "unavailable"
            if delta is None
            else (
                "within_tolerance"
                if abs(delta) <= 1e-8
                else "better" if delta < 0 else "worse"
            )
        ),
    )


def summarize(rows):
    groups = defaultdict(list)
    for r in rows:
        method = r["post_estimator"] if r["available"] else "unavailable"
        for co, kind, estimator in product(
            ["all", r["cohort"]], ["all", r["event_kind"]], ["all", method]
        ):
            groups[
                co, r["variant"], kind, r["phase"], r["input_kind"], estimator
            ].append(r)
    result = []
    for co, arm, kind, phase, inp, estimator in product(
        ["all", *COHORTS],
        ["epoch", "curved"],
        ["all", *KINDS],
        list(OFFSETS)[1:],
        INPUTS,
        ESTIMATORS,
    ):
        rr = groups[co, arm, kind, phase, inp, estimator]
        matched = [r for r in rr if r["available"]]
        item = dict(
            cohort=co,
            variant=arm,
            event_kind=kind,
            phase=phase,
            input_kind=inp,
            post_estimator=estimator,
            records=len(rr),
            matched=len(matched),
            unavailable=len(rr) - len(matched),
            cases=len({tuple(r[k] for k in KEY) for r in rr}),
            opportunities=len({r["event_id"] for r in rr}),
            event_anchor_replaced=sum(r["event_anchor_replaced"] for r in rr),
            shared_anchor_pairs=sum(r["shared_anchors"] > 0 for r in matched),
            shared_observation_pairs=sum(r["shared_samples"] > 0 for r in matched),
            intervening_anchor_pairs=sum(
                r["intervening_anchor_changes"] > 0 for r in matched
            ),
        )
        for label in ACCURACY:
            item[label] = sum(r["accuracy_transition"] == label for r in rr)
        assert sum(item[label] for label in ACCURACY) == len(rr)
        for label in ["better", "worse", "within_tolerance"]:
            item[label] = sum(r["error_change"] == label for r in rr)
        for name, field in [
            ("pooled_pre_sigma", "pre_variance"),
            ("pooled_post_sigma", "post_variance"),
            ("pooled_change_sigma", "difference_variance_joint"),
            ("pooled_independent_sigma", "independent_variance_joint"),
        ]:
            item[name] = (
                math.sqrt(statistics.fmean(r[field] for r in matched))
                if matched
                else ""
            )
        for field in [
            "post_shared_anchor_weight",
            "post_event_new_anchor_weight",
            "post_other_anchor_weight",
            "post_event_anchor_weight",
            "covariance_noise",
            "covariance_anchor",
            "correlation_joint",
        ]:
            item["mean_" + field] = (
                statistics.fmean(r[field] for r in matched) if matched else ""
            )
        for name, field in [
            ("pre_error_rms", "pre_error"),
            ("post_error_rms", "post_error"),
            ("error_change_rms", "error_change_norm"),
        ]:
            item[name] = (
                math.sqrt(statistics.fmean(r[field] ** 2 for r in matched))
                if matched
                else ""
            )
        result.append(item)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--raw-root", type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for n, h in PINNED.items():
        assert digest(PRIOR / n) == h, n
    prior = json.loads((PRIOR / "sources.json").read_text())
    helpers = dict(prior["analysis"]["helpers_sha256"])
    helpers["analyze_correction_centre_transitions.py"] = prior["analysis"][
        "script_sha256"
    ]
    helpers["analyze_speed_probe_shared_information.py"] = SHARED_SHA
    for n, h in helpers.items():
        assert digest(ROOT / n) == h, n
    assert digest(SIMULATOR) == prior["analysis"]["simulator_sha256"]
    assert digest(ROOT / "inputs/obstacles.csv") == prior["map_sha256"]
    assert digest(ROOT / "results/joint-admission-geometry/sources.json") == RAW_SHA
    raw = json.loads(
        (ROOT / "results/joint-admission-geometry/sources.json").read_text()
    )
    accounting, events, snapshots = [
        read_rows(PRIOR / n) for n in ["accounting.csv", "events.csv", "snapshots.csv"]
    ]
    assert (len(accounting), len(events), len(snapshots)) == (648, 2583, 25830)
    expected, case_events = defaultdict(list), defaultdict(list)
    for s in snapshots:
        expected[tuple(s[k] for k in KEY)].append(s)
    for e in events:
        case_events[tuple(e[k] for k in KEY)].append(e)
    infos, reconstruction, cohorts = {}, [], {}
    for cohort in COHORTS:
        pinned = raw["cohorts"][cohort]
        run = (
            args.raw_root / cohort if args.raw_root else Path(pinned["path"])
        ).resolve()
        provenance = json.loads((run / "provenance.json").read_text())
        for field in [
            "variants",
            "seeds",
            "noise_correlation_time_s",
            "simulator_sha256",
            "inputs_sha256",
            "inputs_by_variant_sha256",
        ]:
            assert provenance[field] == pinned["provenance"][field], field
        tau = provenance["noise_correlation_time_s"]
        rho = math.exp(-DT / tau) if tau else 0.0
        files = defaultdict(list)
        for case in accounting:
            if case["cohort"] == cohort and flag(case["selected"]):
                files[case["variant"], case["mode"], case["seed"]].append(case)
        hashes, intervals = {}, 0
        for (variant, mode, seed), cases in sorted(files.items()):
            name = f"{variant}-{mode}-{seed}/trajectories.csv"
            path = run / name
            hashes[name] = file_digest(path)
            assert hashes[name] == pinned["raw_sha256"][name], name
            selected = {(c["scenario"], c["planner"]): c for c in cases}
            tracks = {}
            with path.open() as f:
                reader = csv.DictReader(f)
                assert len(reader.fieldnames) == 82
                for row in reader:
                    assert None not in row and None not in row.values()
                    key = row["scenario"], row["planner"]
                    if key not in selected:
                        continue
                    if float(row["time_s"]) == 0:
                        assert key not in tracks
                        case = selected[key]
                        casekey = tuple(case[k] for k in KEY)
                        tracks[key] = Track(
                            case, rho, expected[casekey], case_events[casekey]
                        )
                    tracks[key].process(row)
            assert set(tracks) == set(selected)
            for case in cases:
                track = tracks[case["scenario"], case["planner"]]
                stats = track.validate()
                reconstruction.append(stats)
                intervals += stats["replayed_intervals"]
                assert not (set(infos) & set(track.infos))
                infos.update(track.infos)
        assert (
            intervals == pinned["selected_intervals"] and hashes == pinned["raw_sha256"]
        )
        cohorts[cohort] = dict(
            path=str(run),
            raw_sha256=hashes,
            replayed_intervals=intervals,
            provenance_sha256=file_digest(run / "provenance.json"),
        )
        print(json.dumps(dict(cohort=cohort, replayed_intervals=intervals)), flush=True)
    assert (
        len(reconstruction) == 368
        and sum(r["replayed_intervals"] for r in reconstruction) == 3312000
    )
    assert set(infos) == {s["snapshot_id"] for s in snapshots if flag(s["available"])}
    information_rows, weights = [], []
    for s in snapshots:
        info = infos.get(s["snapshot_id"])
        values = dict.fromkeys(INFO_FIELDS, "")
        if info is not None:
            values.update(
                estimator=info["estimator"],
                current_anchor_id=info["current_anchor_id"],
                observation_count=len(info["noise"]),
                anchor_count=len(info["anchors"]),
                anchor_signature=json.dumps(
                    sorted(info["anchors"].items()), separators=(",", ":")
                ),
                rho=info["rho"],
                noise_variance=info["noise_variance"],
                anchor_variance=info["anchor_variance"],
                joint_variance=info["noise_variance"] + info["anchor_variance"],
                **{k: float(s[k]) for k in ["error_x", "error_y", "error_norm"]},
            )
            weights.extend(
                dict(snapshot_id=s["snapshot_id"], **g) for g in info["details"]
            )
        information_rows.append(
            dict(
                **{
                    k: s[k]
                    for k in [
                        "snapshot_id",
                        "event_id",
                        *KEY,
                        "event_kind",
                        "event_time_s",
                        "phase",
                        "input_kind",
                        "requested_time_s",
                        "available",
                    ]
                },
                **values,
            )
        )
    sl = {(s["event_id"], s["phase"], s["input_kind"]): s for s in snapshots}
    transitions = []
    for e, phase, inp in product(events, list(OFFSETS)[1:], INPUTS):
        a, b = sl[e["event_id"], "pre", inp], sl[e["event_id"], phase, inp]
        immediate = sl[e["event_id"], "immediate", inp]
        changes = sum(
            float(e["event_time_s"])
            < float(q["event_time_s"])
            <= float(b["requested_time_s"])
            and (flag(q["received"]) or flag(q["precision_activation"]))
            for q in case_events[tuple(e[k] for k in KEY)]
        )
        r = transition(
            e,
            a,
            b,
            infos[a["snapshot_id"]],
            infos.get(b["snapshot_id"]),
            infos[immediate["snapshot_id"]],
            changes,
        )
        transitions.append(dict(transition_id=len(transitions) + 1, **r))
    assert len(transitions) == 20664
    summary = summarize(transitions)
    audit = dict(
        accounted_cases=len(accounting),
        timeout_cases=len(reconstruction),
        opportunities=len(events),
        source_snapshots=len(snapshots),
        available_snapshots=len(infos),
        groups=len(weights),
        requested_transitions=len(transitions),
        matched_transitions=sum(r["available"] for r in transitions),
        summary_rows=len(summary),
        replayed_intervals=3312000,
        max_pose_error=max(r["max_pose_error"] for r in reconstruction),
        max_component_error=max(r["max_component_error"] for r in reconstruction),
    )
    source = dict(
        input_sha256=PINNED,
        raw_source_sha256=RAW_SHA,
        map_sha256=prior["map_sha256"],
        cohorts=cohorts,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
            simulator_sha256=digest(SIMULATOR),
        ),
        schema=dict(
            dt_s=DT,
            offset_steps=OFFSETS,
            inputs=INPUTS,
            post_estimators=ESTIMATORS,
            accuracy_categories=ACCURACY,
            variance_guard_m2=EPS,
        ),
        inherited_validation=dict(
            source_sha256=PINNED["sources.json"],
            observation_and_physics=True,
            new_rollout=False,
        ),
        audit=audit,
    )
    args.output.mkdir(parents=True)
    for n, rr in [
        ("accounting", accounting),
        ("events", events),
        ("snapshot-information", information_rows),
        ("group-weights", weights),
        ("transitions", transitions),
        ("summary", summary),
        ("reconstruction", reconstruction),
    ]:
        save_rows(args.output / (n + ".csv"), rr)
    for n in ["accounting", "events"]:
        assert digest(args.output / (n + ".csv")) == PINNED[n + ".csv"]
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
