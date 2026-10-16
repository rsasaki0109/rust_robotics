#!/usr/bin/env python3
"""Replay signed position-error components on all audited first-chord poses."""

import argparse
from collections import defaultdict
import csv
from itertools import product
import json
import math
from pathlib import Path
import statistics

from analyze_commanded_recovery_arc import flag
from analyze_correction_centre_transitions import checked_poses
from analyze_epoch_position_components import PARTS, Replay, decompose, file_digest
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import KEY
from analyze_speed_probe_envelopes import decision
from analyze_speed_probe_coverage_spans import load_pose
from analyze_speed_probe_starts import ESTIMATES, KINDS
from analyze_terminal_arc_live import COHORTS, SIMULATOR
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PROTOCOL = ROOT / "results/speed-probe-error-components/protocol.md"
PINNED = {
    "speed-probe-coverage-spans/accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "speed-probe-coverage-spans/events.csv": "bc7a40a8da19e573d443c09fec79d4cb5fa1023de74b0bfbcfad2d4d40b07274",
    "speed-probe-coverage-spans/chains.csv": "2bdacbd6e17706455901c601a57d4041f8ca2b112d7fac40e88b75753f22ea61",
    "speed-probe-coverage-spans/members.csv": "fe6db79ea0af46dbc0953a8fabb7c27ac58b8c31b2a7766747efb154d9a3fda9",
    "speed-probe-coverage-spans/spans.csv": "8e61c2b73174facbe26cd6ec2278fe09cedd94fff7e94856a66ce28dc5e54f6d",
    "speed-probe-coverage-spans/loss-memberships.csv": "3c28f5a36e41279c3dceae50983868b54fb8ed5652617aa4ec527008e98ead07",
    "speed-probe-coverage-spans/sources.json": "19bf2b76e72b4e05549902e8414909c1a6e497c58c6118c7ddcc87082a395cd9",
    "speed-probe-envelopes/poses.csv": "966d626f68fe514c7b0e64902a8b894201a624353e2afe78808890080108a052",
    "speed-probe-starts/poses.csv": "f75df1bd327982a37344466534b9ff511aa10cfc512e865e6d7674bbafda5796",
    "joint-admission-geometry/sources.json": "ccd59c809aa90313f8f96af8857dca180ce8b3b92600b57fb14757a0edca4da4",
}
POPULATIONS = ["all_states", "raw_zero_loss", "command_only_zero_loss"]
CONDITIONS = ["all", "central_clear_truth_blocks", "envelope_clear_truth_blocks"]
ESTIMATORS = [
    "raw_observation",
    "recovery_mean",
    "raw_fallback",
    "three_epoch",
    "motion_filter",
]
DT, EPS = 0.02, 1e-8


def input_groups(state, kind):
    if kind == "raw":
        sample = state.current
        return "raw_observation", [
            dict(samples=[sample], epoch=sample.anchor_id, weight=1.0, transport=False)
        ]
    return state.view("actual" if kind == "recovery" else kind)


def moments(rows, part):
    if not rows:
        return {
            f"{part}_{k}": ""
            for k in ["rms", "mean_x", "mean_y", "mean_norm", "centered_rms"]
        }
    x, y = [statistics.fmean(r[part + "_" + a] for r in rows) for a in ["x", "y"]]
    square = statistics.fmean(r[part + "_x"] ** 2 + r[part + "_y"] ** 2 for r in rows)
    centered = statistics.fmean(
        (r[part + "_x"] - x) ** 2 + (r[part + "_y"] - y) ** 2 for r in rows
    )
    assert math.isclose(square, x * x + y * y + centered, rel_tol=1e-11, abs_tol=1e-14)
    return {
        f"{part}_rms": math.sqrt(square),
        f"{part}_mean_x": x,
        f"{part}_mean_y": y,
        f"{part}_mean_norm": math.hypot(x, y),
        f"{part}_centered_rms": math.sqrt(centered),
    }


def metrics(rows):
    result = {}
    for part in ["error", *PARTS]:
        result.update(moments(rows, part))
    for part in PARTS:
        result[part + "_mean_projection"] = (
            statistics.fmean(r[part + "_projection"] for r in rows) if rows else ""
        )
    for label in [*PARTS, "none"]:
        result[label + "_largest"] = sum(r["largest_projection"] == label for r in rows)
    assert sum(result[p + "_largest"] for p in [*PARTS, "none"]) == len(rows)
    for estimator in ESTIMATORS:
        result["estimator_" + estimator] = sum(
            r["estimator"] == estimator for r in rows
        )
    assert sum(result["estimator_" + e] for e in ESTIMATORS) == len(rows)
    return result


class ComponentReplay:
    def __init__(self, case, rho, targets, diagnostics, source_poses, members):
        self.case = {k: case[k] for k in KEY}
        self.rho, self.targets, self.diag = rho, targets, diagnostics
        self.source, self.members = source_poses, members
        self.state, self.before = Replay(), None
        self.steps, self.seen = 0, set()
        self.records, self.groups = [], []
        self.max_pose_error = self.max_norm_error = 0.0

    def process(self, row):
        if float(row["time_s"]) == 0:
            assert self.before is None
            self.before = row
            return
        before = self.before
        step = round(float(row["time_s"]) / DT)
        assert before is not None and step == self.steps + 1
        assert abs(float(row["noise_rho"]) - self.rho) < 1e-12
        assert abs(float(row["measured_speed"]) - float(before["v"])) < 1e-9
        self.state.advance(before, row)
        self.state.check_counts(row)
        if step in self.targets:
            poses, maximum = checked_poses(self.state, row)
            self.max_pose_error = max(self.max_pose_error, maximum)
            for pid in self.targets[step]:
                self.capture(pid, before, row, poses)
        self.before, self.steps = row, step

    def capture(self, pid, before, row, poses):
        assert pid not in self.seen
        self.seen.add(pid)
        p, saved = self.diag[pid], self.source[pid]
        truth_pose = self.source[saved["truth_pose_id"]]
        kind = p["pose_kind"]
        estimator, groups = input_groups(self.state, kind)
        truth = tuple(float(before[a]) for a in ["x", "y"])
        record, details = decompose(groups, truth, self.state.odom, self.rho)
        x, y = record.pop("reconstructed_x"), record.pop("reconstructed_y")
        if kind == "raw":
            yaw = self.state.current.yaw
            assert (
                abs(
                    x
                    - (
                        float(before["x"])
                        + float(row["noise_x"])
                        + float(row["bias_x"])
                    )
                )
                < EPS
            )
            assert (
                abs(
                    y
                    - (
                        float(before["y"])
                        + float(row["noise_y"])
                        + float(row["bias_y"])
                    )
                )
                < EPS
            )
        else:
            expected = poses["actual" if kind == "recovery" else kind]
            assert math.dist((x, y), expected[:2]) < EPS
            yaw = expected[2]
        for axis, value in zip(["x", "y", "yaw"], [x, y, yaw]):
            delta = value - float(saved[axis])
            if axis == "yaw":
                delta = math.atan2(math.sin(delta), math.cos(delta))
            assert abs(delta) < EPS, (pid, axis, delta)
            self.max_pose_error = max(self.max_pose_error, abs(delta))
        for a, value in zip(["x", "y"], truth):
            assert abs(value - float(truth_pose[a])) < EPS
        assert before["time_s"] == saved["truth_time_s"]
        assert row["time_s"] == p["time_s"]
        norm_error = abs(record["error_norm"] - float(p["position_error"]))
        assert norm_error < EPS
        self.max_norm_error = max(self.max_norm_error, norm_error)
        assert all(self.case[k] == p[k] for k in KEY)
        m = self.members[p["state_id"]]
        fixed = decision(load_pose(p), 0.1, 4)
        output = dict(
            pose_id=pid,
            state_id=p["state_id"],
            source_interval_id=p["source_interval_id"],
            chain_id=m["chain_id"],
            chain_index=m["chain_index"],
            **self.case,
            source_event_ids=p["source_event_ids"],
            source_event_kinds=p["source_event_kinds"],
            time_s=row["time_s"],
            truth_time_s=before["time_s"],
            pose_kind=kind,
            estimator=estimator,
            pose_x=x,
            pose_y=y,
            pose_yaw=yaw,
            truth_x=truth[0],
            truth_y=truth[1],
            central_clear=flag(p["central_clear"]),
            truth_clear=flag(p["truth_clear"]),
            central_clear_truth_blocks=flag(p["central_clear"])
            and not flag(p["truth_clear"]),
            envelope_clear_truth_blocks=fixed["clear_truth_blocks"],
            **record,
        )
        self.records.append(output)
        self.groups.extend(dict(pose_id=pid, **d) for d in details)

    def validate(self):
        assert self.steps == 9000
        expected = {pid for ids in self.targets.values() for pid in ids}
        assert self.seen == expected
        return dict(
            **self.case,
            replayed_intervals=self.steps,
            audited_states=len(self.targets),
            reconstructed_poses=len(self.seen),
            max_pose_error=self.max_pose_error if self.seen else "",
            max_norm_error=self.max_norm_error if self.seen else "",
        )


def summarize(components, losses, events):
    lookup = {r["pose_id"]: r for r in components}
    groups = defaultdict(list)

    def add(p, population, ids):
        kinds = {"all", *[events[e]["event_kind"] for e in ids]}
        for co, kind in product(["all", p["cohort"]], kinds):
            key = co, p["variant"], kind, p["pose_kind"], population
            groups[key].append((p, ids))

    for p in components:
        add(p, "all_states", p["source_event_ids"].split("+"))
    for loss in losses:
        if float(loss["radius_m"]) == 0 and int(loss["yaw_bound_deg"]) == 0:
            p = lookup[loss["pose_id"]]
            add(p, "raw_zero_loss", [loss["event_id"]])
            if flag(loss["command_only_zero_loss"]):
                add(p, "command_only_zero_loss", [loss["event_id"]])
    result = []
    for co, arm, kind, pose, population in product(
        ["all", *COHORTS], ["epoch", "curved"], ["all", *KINDS], ESTIMATES, POPULATIONS
    ):
        full = groups[co, arm, kind, pose, population]
        for condition in CONDITIONS:
            selected = (
                full
                if condition == "all"
                else [(p, ids) for p, ids in full if p[condition]]
            )
            rr = [p for p, _ in selected]
            ids = {e for _, event_ids in selected for e in event_ids}
            result.append(
                dict(
                    cohort=co,
                    variant=arm,
                    event_kind=kind,
                    pose_kind=pose,
                    population=population,
                    condition=condition,
                    records=len(rr),
                    states=len({r["state_id"] for r in rr}),
                    cases=len({tuple(r[k] for k in KEY) for r in rr}),
                    opportunities=sum(
                        kind == "all" or events[e]["event_kind"] == kind for e in ids
                    ),
                    **metrics(rr),
                )
            )
    return result


def span_components(spans, members, components):
    lookup = {r["pose_id"]: r for r in components}
    chains = defaultdict(list)
    for m in members:
        chains[m["chain_id"]].append(m)
    output = []
    for s in spans:
        if float(s["radius_m"]) != 0.1 or int(s["yaw_bound_deg"]) != 4:
            continue
        a, b = int(s["first_chain_index"]), int(s["last_chain_index"])
        mm = chains[s["chain_id"]][a : b + 1]
        assert len(mm) == int(s["states"])
        rr = [lookup[m[s["pose_kind"] + "_pose_id"]] for m in mm]
        assert (
            rr[0]["pose_id"] == s["first_pose_id"]
            and rr[-1]["pose_id"] == s["last_pose_id"]
        )
        output.append(
            dict(
                **s,
                first_estimator=rr[0]["estimator"],
                last_estimator=rr[-1]["estimator"],
                sample_count_min=min(r["used_sample_count"] for r in rr),
                sample_count_max=max(r["used_sample_count"] for r in rr),
                **metrics(rr),
            )
        )
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for name, sha in PINNED.items():
        assert digest(ROOT / "results" / name) == sha, name
    prior = json.loads(
        (ROOT / "results/speed-probe-coverage-spans/sources.json").read_text()
    )
    raw = json.loads(
        (ROOT / "results/joint-admission-geometry/sources.json").read_text()
    )
    helpers = dict(prior["analysis"]["helpers_sha256"])
    helpers["analyze_speed_probe_coverage_spans.py"] = prior["analysis"][
        "script_sha256"
    ]
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    assert digest(SIMULATOR) == prior["analysis"]["simulator_sha256"]
    assert digest(ROOT / "inputs/obstacles.csv") == prior["map_sha256"]
    prefix = ROOT / "results/speed-probe-coverage-spans"
    accounting, event_rows, member_rows, spans, losses = (
        read_rows(prefix / n)
        for n in [
            "accounting.csv",
            "events.csv",
            "members.csv",
            "spans.csv",
            "loss-memberships.csv",
        ]
    )
    diagnostics = {
        r["pose_id"]: r
        for r in read_rows(ROOT / "results/speed-probe-envelopes/poses.csv")
    }
    source_poses = {
        r["pose_id"]: r
        for r in read_rows(ROOT / "results/speed-probe-starts/poses.csv")
    }
    members = {r["state_id"]: r for r in member_rows}
    assert (
        len(accounting),
        len(event_rows),
        len(members),
        len(diagnostics),
        len(spans),
        len(losses),
    ) == (648, 2583, 6610, 19830, 52031, 19800)
    targets = defaultdict(lambda: defaultdict(list))
    for p in diagnostics.values():
        targets[tuple(p[k] for k in KEY)][round(float(p["time_s"]) / DT)].append(
            p["pose_id"]
        )
    assert len(targets) == 100 and all(
        len(ids) == 3 for tt in targets.values() for ids in tt.values()
    )
    components, groups, reconstruction = [], [], []
    sources = dict(input_sha256=PINNED, map_sha256=prior["map_sha256"], cohorts={})
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
            with path.open() as stream:
                reader = csv.DictReader(stream)
                assert len(reader.fieldnames) == 82
                for row in reader:
                    assert None not in row and None not in row.values()
                    key = row["scenario"], row["planner"]
                    if key not in selected:
                        continue
                    if float(row["time_s"]) == 0:
                        assert key not in tracks
                        case = selected[key]
                        tracks[key] = ComponentReplay(
                            case,
                            rho,
                            targets[tuple(case[k] for k in KEY)],
                            diagnostics,
                            source_poses,
                            members,
                        )
                    tracks[key].process(row)
            assert set(tracks) == set(selected)
            for case in cases:
                track = tracks[case["scenario"], case["planner"]]
                stats = track.validate()
                intervals += stats["replayed_intervals"]
                reconstruction.append(stats)
                components.extend(track.records)
                groups.extend(track.groups)
        assert intervals == pinned["selected_intervals"]
        assert hashes == pinned["raw_sha256"]
        sources["cohorts"][cohort] = dict(
            path=str(run),
            raw_sha256=hashes,
            replayed_intervals=intervals,
            provenance_sha256=file_digest(run / "provenance.json"),
        )
        print(f"{cohort}: {intervals} intervals replayed", flush=True)
    components.sort(key=lambda r: int(r["pose_id"]))
    groups.sort(key=lambda r: (int(r["pose_id"]), r["group_index"]))
    assert len(components) == 19830 and {r["pose_id"] for r in components} == set(
        diagnostics
    )
    assert (
        len(reconstruction) == 368
        and sum(r["replayed_intervals"] for r in reconstruction) == 3312000
    )
    events = {r["event_id"]: r for r in event_rows}
    summary = summarize(components, losses, events)
    sc = span_components(spans, member_rows, components)
    audit = dict(
        accounted_cases=648,
        timeout_cases=368,
        replayed_intervals=3312000,
        opportunities=2583,
        audited_cases=100,
        audited_states=6610,
        components=len(components),
        groups=len(groups),
        raw_zero_loss_pairs=550,
        command_only_zero_loss_pairs=509,
        inherited_loss_memberships=len(losses),
        summary_rows=len(summary),
        span_components=len(sc),
        max_pose_error=max(
            r["max_pose_error"] for r in reconstruction if r["max_pose_error"] != ""
        ),
        max_norm_error=max(
            r["max_norm_error"] for r in reconstruction if r["max_norm_error"] != ""
        ),
    )
    sources.update(
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
            simulator_sha256=digest(SIMULATOR),
        ),
        schema=dict(
            parts=PARTS,
            conditions=CONDITIONS,
            populations=POPULATIONS,
            span_radius_m=0.1,
            span_yaw_bound_deg=4,
            truth="preceding physical CSV row",
            moments="state-weighted vector RMS, mean-vector norm and centered RMS",
        ),
        inherited_validation=dict(
            physics_and_observations=True,
            new_rollout=False,
            source_sha256=PINNED["joint-admission-geometry/sources.json"],
        ),
        audit=audit,
    )
    args.output.mkdir(parents=True)
    for name, rr in [
        ("accounting", accounting),
        ("events", event_rows),
        ("loss-memberships", losses),
        ("reconstruction", reconstruction),
        ("components", components),
        ("groups", groups),
        ("summary", summary),
        ("span-components", sc),
    ]:
        save_rows(args.output / f"{name}.csv", rr)
    for n in ["accounting.csv", "events.csv", "loss-memberships.csv"]:
        assert digest(args.output / n) == PINNED["speed-probe-coverage-spans/" + n]
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
