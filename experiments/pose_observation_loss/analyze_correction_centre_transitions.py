#!/usr/bin/env python3
"""Replay every late correction opportunity, matched centre errors and actions."""

import argparse
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
import statistics

from analyze_commanded_recovery_arc import flag, identity
from analyze_epoch_divergence import REQUIREMENTS, release_inputs
from analyze_epoch_position_components import PARTS, Replay, decompose, file_digest
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import DT, EPS, KEY
from analyze_terminal_arc_live import COHORTS, SIMULATOR
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PROTOCOL = ROOT / "results/correction-centre-transitions/protocol.md"
PINNED = {
    "epoch-position-components/accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "epoch-position-components/positions.csv": "67fe28537f1e84d47ee8711b11f3df0cc52f8b770e694da0055d6265604f321e",
    "epoch-position-components/sources.json": "01e6b94b68c892621047829585df1db46c6471e7a3dfdd819c40e7fa6fe136db",
    "joint-admission-geometry/sources.json": "ccd59c809aa90313f8f96af8857dca180ce8b3b92600b57fb14757a0edca4da4",
}
OFFSETS = dict(pre=-1, immediate=0, after_0p18=9, after_0p5=25, after_1p5=75)
KINDS = [
    "received",
    "missed",
    "precision_only",
    "received_and_precision",
    "missed_and_precision",
]
CHANGES = ["better", "worse", "within_tolerance", "unavailable"]
HORIZON = 75
MODEL_FIELDS = [
    "model_noise_sigma_per_axis",
    "model_anchor_sigma_per_axis",
    "model_noise_anchor_sigma_per_axis",
]


def opportunity_kind(row):
    scheduled, received, precision = (
        flag(row[k])
        for k in ["correction", "correction_received", "precision_activation"]
    )
    assert not received or scheduled
    if not scheduled:
        return "precision_only" if precision else None
    name = "received" if received else "missed"
    return name + "_and_precision" if precision else name


def gate_context(before, row):
    route, requirements = release_inputs(before, row)
    return dict(
        route=route,
        active_ready=bool(requirements) and all(requirements.values()),
        joint_base_usable=route == "joint"
        and all(requirements[k] for k in ["stationary", "fresh", "samples"]),
        **{f"ready_{k}": requirements.get(k, "") for k in REQUIREMENTS},
        resumed=flag(row["resumed"]),
        guard_released=flag(row["guard_released"]),
        guard_active=flag(row["guard_active"]),
        safety_paused=flag(row["safety_paused"]),
        positive_target=float(row["target_speed"]) > 0,
        target_speed=float(row["target_speed"]),
        measured_speed=float(row["measured_speed"]),
        recovery_evidence=float(row["recovery_evidence"]),
        guard_release_evidence=float(row["guard_release_evidence"]),
        recovery_sample_count=int(row["recovery_sample_count"]),
        motion_filter_sample_count=int(row["motion_filter_sample_count"]),
        epoch_fusion_group_count=int(row["epoch_fusion_group_count"]),
        fusion_available=flag(row["epoch_fusion_available"]),
        observation_age=float(row["observation_age"]),
    )


def checked_poses(state, row):
    poses, maximum = {}, 0.0
    for kind, prefix in [
        ("actual", "recovery"),
        ("preview", "preview_pose"),
        ("motion", "motion_filter_pose"),
    ]:
        pose = state.pose(kind)
        for axis, label in enumerate(["x", "y"]):
            error = abs(pose[axis] - float(row[prefix + "_" + label]))
            assert error < 1e-8, (kind, label, error)
            maximum = max(maximum, error)
        heading = float(
            row["recovery_yaw"] if kind == "actual" else row["preview_pose_yaw"]
        )
        error = abs(
            math.atan2(math.sin(pose[2] - heading), math.cos(pose[2] - heading))
        )
        assert error < 1e-8, (kind, "yaw", error)
        maximum = max(maximum, error)
        poses[kind] = pose
    return poses, maximum


def input_snapshot(state, before, row, kind, rho, pose):
    estimator, groups = state.view(kind)
    truth = tuple(float(before[a]) for a in ["x", "y", "yaw"])
    components, _ = decompose(groups, truth[:2], state.odom, rho)
    x, y = components.pop("reconstructed_x"), components.pop("reconstructed_y")
    assert abs(x - pose[0]) < 1e-10 and abs(y - pose[1]) < 1e-10
    assert abs(components["error_norm"] - math.dist((x, y), truth[:2])) < 1e-10
    return dict(
        time_s=row["time_s"],
        truth_time_s=before["time_s"],
        estimator=estimator,
        pose_x=x,
        pose_y=y,
        pose_yaw=pose[2],
        truth_x=truth[0],
        truth_y=truth[1],
        truth_yaw=truth[2],
        yaw_error_deg=abs(
            math.degrees(
                math.atan2(math.sin(pose[2] - truth[2]), math.cos(pose[2] - truth[2]))
            )
        ),
        **components,
    )


class ActionWindow:
    """Exactly (event,event+1.5], with censoring and no fabricated completion."""

    def __init__(self, event_step):
        self.event_step = event_step
        self.intervals = self.resumes = self.releases = self.positive = 0
        self.scheduled = self.precision = 0
        self.travel = 0.0
        self.first_resume = self.first_release = self.first_positive = ""
        self.ready = dict(joint=0, independent=0)
        self.runs = dict(joint=0, independent=0)
        self.maxima = dict(joint=0, independent=0)

    def observe(self, step, context, travel, scheduled=False, precision=False):
        offset = step - self.event_step
        if not 0 < offset <= HORIZON:
            return
        assert offset == self.intervals + 1
        self.intervals += 1
        self.travel += travel
        self.scheduled += scheduled
        self.precision += precision
        for route in self.ready:
            ready = context["route"] == route and context["active_ready"]
            self.ready[route] += ready
            self.runs[route] = self.runs[route] + 1 if ready else 0
            self.maxima[route] = max(self.maxima[route], self.runs[route])
        for key, counter, first in [
            ("resumed", "resumes", "first_resume"),
            ("guard_released", "releases", "first_release"),
            ("positive_target", "positive", "first_positive"),
        ]:
            if context[key]:
                setattr(self, counter, getattr(self, counter) + 1)
                if getattr(self, first) == "":
                    setattr(self, first, round(offset * DT, 6))

    def record(self):
        return dict(
            post_intervals=self.intervals,
            post_window_complete=self.intervals == HORIZON,
            post_window_censored=self.intervals < HORIZON,
            post_resumes=self.resumes,
            post_guard_releases=self.releases,
            post_positive_target_intervals=self.positive,
            post_physical_travel=self.travel,
            first_resume_delay_s=self.first_resume,
            first_guard_release_delay_s=self.first_release,
            first_positive_target_delay_s=self.first_positive,
            intervening_scheduled=self.scheduled,
            intervening_precision=self.precision,
            **{
                f"{route}_ready_s": round(self.ready[route] * DT, 6)
                for route in self.ready
            },
            **{
                f"{route}_max_ready_s": round(self.maxima[route] * DT, 6)
                for route in self.ready
            },
        )


def pair_record(event, phase, kind, pre, post):
    available = post["available"]
    record = dict(
        **{
            k: event[k]
            for k in KEY + ["event_id", "event_kind", "event_time_s", "event_route"]
        },
        phase=phase,
        input_kind=kind,
        pre_snapshot_id=pre["snapshot_id"],
        post_snapshot_id=post["snapshot_id"],
        available=available,
        estimator_transition=pre["estimator"]
        + "->"
        + (post["estimator"] if available else "unavailable"),
        pre_error_norm=pre["error_norm"],
        post_error_norm=post["error_norm"] if available else "",
        error_norm_change=post["error_norm"] - pre["error_norm"] if available else "",
    )
    if available:
        change = record["error_norm_change"]
        status = (
            "better"
            if change < -EPS
            else "worse" if change > EPS else "within_tolerance"
        )
    else:
        status = "unavailable"
    record.update(change=status, **{k: status == k for k in CHANGES})
    for prefix, fields in [
        ("pose", ["x", "y"]),
        ("truth", ["x", "y"]),
        ("error", ["x", "y"]),
    ]:
        delta = (
            [post[f"{prefix}_{a}"] - pre[f"{prefix}_{a}"] for a in fields]
            if available
            else ["", ""]
        )
        record.update({f"delta_{prefix}_{a}": value for a, value in zip(fields, delta)})
        record[f"{prefix}_shift_norm"] = math.hypot(*delta) if available else ""
    for part in PARTS:
        for label, snapshot in [("pre", pre), ("post", post)]:
            record[f"{label}_{part}_norm"] = (
                snapshot[part + "_norm"] if snapshot["available"] else ""
            )
        for axis in ["x", "y"]:
            record[f"delta_{part}_{axis}"] = (
                post[f"{part}_{axis}"] - pre[f"{part}_{axis}"] if available else ""
            )
    for field in ["yaw_error_deg", "used_sample_count", *MODEL_FIELDS]:
        record["pre_" + field] = pre[field]
        record["post_" + field] = post[field] if available else ""
    for field in [
        "route",
        "active_ready",
        "joint_base_usable",
        "resumed",
        "guard_released",
        "positive_target",
    ]:
        record["pre_" + field] = pre[field]
        record["post_" + field] = post[field] if available else ""
    if available:
        for axis in ["x", "y"]:
            assert (
                abs(
                    record[f"delta_pose_{axis}"]
                    - record[f"delta_truth_{axis}"]
                    - record[f"delta_error_{axis}"]
                )
                < EPS
            )
            assert (
                abs(
                    sum(record[f"delta_{p}_{axis}"] for p in PARTS)
                    - record[f"delta_error_{axis}"]
                )
                < EPS
            )
    return record


class TrackReplay:
    def __init__(self, case, rho, expected):
        self.case = {k: case[k] for k in KEY}
        self.rho, self.expected = rho, expected
        self.state, self.before, self.last_truth = Replay(), None, None
        self.steps, self.seen = 0, set()
        self.events, self.active, self.targets, self.cache = [], [], set(), {}
        self.max_reconstruction_error = self.max_previous_error = 0.0

    def capture(self, truth, row):
        step = round(float(row["time_s"]) / DT)
        if step in self.cache:
            return self.cache[step]
        assert truth is not None and self.state.current.step == step
        poses, error = checked_poses(self.state, row)
        self.max_reconstruction_error = max(self.max_reconstruction_error, error)
        common = gate_context(truth, row)
        snapshots = {
            kind: dict(
                **input_snapshot(self.state, truth, row, kind, self.rho, poses[kind]),
                **common,
            )
            for kind in ["actual", "preview"]
        }
        self.cache[step] = snapshots
        return snapshots

    def process(self, row):
        if float(row["time_s"]) == 0:
            assert self.before is None
            self.before = row
            return
        before = self.before
        step = round(float(row["time_s"]) / DT)
        assert step == self.steps + 1
        assert abs(float(row["noise_rho"]) - self.rho) < 1e-12
        assert abs(float(row["measured_speed"]) - float(before["v"])) < 1e-9
        category = opportunity_kind(row) if 7500 <= step <= 9000 else None
        if category:
            self.capture(self.last_truth, before)
            self.targets.update(
                step + offset for offset in OFFSETS.values() if offset >= 0
            )
        self.state.advance(before, row)
        self.state.check_counts(row)
        self.active = [e for e in self.active if step <= e["step"] + HORIZON]
        if category or self.active or step in self.targets:
            context = gate_context(before, row)
            if category:
                event = dict(
                    step=step,
                    event_kind=category,
                    event_time_s=row["time_s"],
                    scheduled=flag(row["correction"]),
                    received=flag(row["correction_received"]),
                    precision_activation=flag(row["precision_activation"]),
                    event_route=context["route"],
                    immediate_resumed=context["resumed"],
                    immediate_guard_released=context["guard_released"],
                    immediate_positive_target=context["positive_target"],
                    immediate_physical_travel=math.dist(
                        tuple(float(before[a]) for a in ["x", "y"]),
                        tuple(float(row[a]) for a in ["x", "y"]),
                    ),
                    window=ActionWindow(step),
                )
                self.events.append(event)
                self.active.append(event)
            travel = math.dist(
                tuple(float(before[a]) for a in ["x", "y"]),
                tuple(float(row[a]) for a in ["x", "y"]),
            )
            for event in self.active:
                event["window"].observe(
                    step,
                    context,
                    travel,
                    flag(row["correction"]),
                    flag(row["precision_activation"]),
                )
            if step in self.targets:
                self.capture(before, row)
        if (row["time_s"], "actual") in self.expected:
            poses, error = checked_poses(self.state, row)
            self.max_reconstruction_error = max(self.max_reconstruction_error, error)
            for kind in ["actual", "preview"]:
                key = row["time_s"], kind
                expected = self.expected[key]
                error = abs(
                    math.dist(
                        poses[kind][:2], tuple(float(before[a]) for a in ["x", "y"])
                    )
                    - float(expected["error_norm"])
                )
                assert error < 1e-8, (self.case, key, error)
                self.max_previous_error = max(self.max_previous_error, error)
                self.seen.add(key)
        self.last_truth, self.before, self.steps = before, row, step

    def validate(self):
        assert self.steps == 9000 and float(self.before["time_s"]) == 180
        assert self.seen == set(self.expected)
        assert {e["step"] for e in self.events if e["scheduled"]} == set(
            range(7500, 9001, 250)
        )
        return dict(
            **self.case,
            intervals=self.steps,
            original_checkpoints=len(self.seen) // 2,
            opportunities=len(self.events),
            sampled_times=len(self.cache),
            scheduled_opportunities=sum(e["scheduled"] for e in self.events),
            precision_opportunities=sum(e["precision_activation"] for e in self.events),
            max_reconstruction_error=self.max_reconstruction_error,
            max_previous_norm_error=self.max_previous_error,
        )

    def export(self, events, snapshots, pairs):
        for raw_event in self.events:
            event = dict(
                event_id=len(events) + 1,
                **self.case,
                **{
                    k: raw_event[k]
                    for k in [
                        "event_kind",
                        "event_time_s",
                        "scheduled",
                        "received",
                        "precision_activation",
                        "event_route",
                        "immediate_resumed",
                        "immediate_guard_released",
                        "immediate_positive_target",
                        "immediate_physical_travel",
                    ]
                },
                **raw_event["window"].record(),
            )
            events.append(event)
            sample_rows = {}
            template = self.cache[raw_event["step"]]
            for phase, offset in OFFSETS.items():
                target = raw_event["step"] + offset
                available = target <= self.steps
                assert not available or target in self.cache
                for kind in ["actual", "preview"]:
                    snapshot = dict(
                        snapshot_id=len(snapshots) + 1,
                        event_id=event["event_id"],
                        **self.case,
                        event_kind=event["event_kind"],
                        event_time_s=event["event_time_s"],
                        phase=phase,
                        input_kind=kind,
                        requested_time_s=f"{target * DT:.6f}",
                        available=available,
                        **{
                            k: self.cache[target][kind][k] if available else ""
                            for k in template[kind]
                        },
                    )
                    snapshots.append(snapshot)
                    sample_rows[phase, kind] = snapshot
            for phase in list(OFFSETS)[1:]:
                for kind in ["actual", "preview"]:
                    pairs.append(
                        pair_record(
                            event,
                            phase,
                            kind,
                            sample_rows["pre", kind],
                            sample_rows[phase, kind],
                        )
                    )


def rms(rows, field):
    return math.sqrt(statistics.fmean(r[field] ** 2 for r in rows)) if rows else ""


def summarize_pairs(pairs):
    groups = defaultdict(list)
    dimensions = [
        "cohort",
        "variant",
        "event_kind",
        "phase",
        "input_kind",
        "route_group",
        "transition_group",
    ]
    for cohort in ["all", *COHORTS]:
        for variant in ["epoch", "curved"]:
            for category in ["all", *KINDS]:
                for phase in list(OFFSETS)[1:]:
                    for kind in ["actual", "preview"]:
                        groups[cohort, variant, category, phase, kind, "all", "all"]
    for row in pairs:
        for cohort in ["all", row["cohort"]]:
            for category in ["all", row["event_kind"]]:
                for route in ["all", row["event_route"]]:
                    for transition in ["all", row["estimator_transition"]]:
                        groups[
                            cohort,
                            row["variant"],
                            category,
                            row["phase"],
                            row["input_kind"],
                            route,
                            transition,
                        ].append(row)
    result = []
    for key, rows in sorted(groups.items()):
        available = [r for r in rows if r["available"]]
        record = dict(
            **dict(zip(dimensions, key)),
            events=len(rows),
            paired_available=len(available),
            cases=len({tuple(r[k] for k in KEY) for r in rows}),
            **{k: sum(r[k] for r in rows) for k in CHANGES},
            pre_error_rms=rms(available, "pre_error_norm"),
            post_error_rms=rms(available, "post_error_norm"),
            mean_error_norm_change=(
                statistics.fmean(r["error_norm_change"] for r in available)
                if available
                else ""
            ),
            mean_pose_shift=(
                statistics.fmean(r["pose_shift_norm"] for r in available)
                if available
                else ""
            ),
            mean_truth_shift=(
                statistics.fmean(r["truth_shift_norm"] for r in available)
                if available
                else ""
            ),
            **{
                f"{side}_{part}_rms": rms(available, f"{side}_{part}_norm")
                for side in ["pre", "post"]
                for part in PARTS
            },
        )
        assert sum(record[k] for k in CHANGES) == len(rows)
        assert record["paired_available"] + record["unavailable"] == len(rows)
        result.append(record)
    return result


def summarize_events(events):
    groups = defaultdict(list)
    dimensions = ["cohort", "variant", "event_kind", "route_group"]
    for cohort in ["all", *COHORTS]:
        for variant in ["epoch", "curved"]:
            for category in ["all", *KINDS]:
                groups[cohort, variant, category, "all"]
    for row in events:
        for cohort in ["all", row["cohort"]]:
            for category in ["all", row["event_kind"]]:
                for route in ["all", row["event_route"]]:
                    groups[cohort, row["variant"], category, route].append(row)
    result = []
    for key, rows in sorted(groups.items()):
        complete = [r for r in rows if r["post_window_complete"]]
        censored = [r for r in rows if r["post_window_censored"]]
        record = dict(
            **dict(zip(dimensions, key)),
            events=len(rows),
            cases=len({tuple(r[k] for k in KEY) for r in rows}),
            complete_windows=len(complete),
            censored_windows=len(censored),
            **{
                field + "_events": sum(r[field] for r in rows)
                for field in [
                    "scheduled",
                    "received",
                    "precision_activation",
                    "immediate_resumed",
                    "immediate_guard_released",
                    "immediate_positive_target",
                ]
            },
            **{
                f"{name}_{label}_events": sum(r[field] > 0 for r in selected)
                for name, selected in [
                    ("complete", complete),
                    ("censored_observed", censored),
                ]
                for field, label in [
                    ("post_resumes", "resume"),
                    ("post_guard_releases", "guard_release"),
                    ("post_positive_target_intervals", "positive_target"),
                ]
            },
            complete_motion_events=sum(
                r["post_physical_travel"] > EPS for r in complete
            ),
            complete_physical_travel=sum(r["post_physical_travel"] for r in complete),
            intervening_scheduled=sum(r["intervening_scheduled"] for r in rows),
            intervening_precision=sum(r["intervening_precision"] for r in rows),
        )
        assert len(complete) + len(censored) == len(rows)
        result.append(record)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    if args.output.exists():
        parser.error("Output directory must be new")
    for name, expected in PINNED.items():
        assert digest(ROOT / "results" / name) == expected, name
    model_source = json.loads(
        (ROOT / "results/epoch-position-components/sources.json").read_text()
    )
    raw = json.loads(
        (ROOT / "results/joint-admission-geometry/sources.json").read_text()
    )
    helpers = dict(model_source["analysis"]["helpers_sha256"])
    helpers["analyze_epoch_position_components.py"] = model_source["analysis"][
        "script_sha256"
    ]
    assert "analyze_epoch_divergence.py" in helpers
    for name, expected in helpers.items():
        assert digest(ROOT / name) == expected, name
    assert digest(SIMULATOR) == model_source["analysis"]["simulator_sha256"]
    component_path = ROOT / "results/commanded-arc-components/sources.json"
    assert digest(component_path) == model_source["input_sha256"]["sources.json"]
    map_sha = json.loads(component_path.read_text())["map_sha256"]
    assert digest(ROOT / "inputs/obstacles.csv") == map_sha
    accounting = read_rows(ROOT / "results/epoch-position-components/accounting.csv")
    positions = read_rows(ROOT / "results/epoch-position-components/positions.csv")
    assert (
        len(accounting) == 648 and sum(flag(r["selected"]) for r in accounting) == 368
    )
    assert len(positions) == 4240
    expected = defaultdict(dict)
    for row in positions:
        expected[identity(row)[:-1]][row["time_s"], row["input_kind"]] = row
    assert len(expected) == 368 and sum(len(rows) for rows in expected.values()) == 4240
    events, snapshots, pairs, reconstruction = [], [], [], []
    sources = dict(input_sha256=PINNED, map_sha256=map_sha, cohorts={})
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
            selected = {(r["scenario"], r["planner"]): r for r in cases}
            tracks = {}
            with path.open() as stream:
                reader = csv.DictReader(stream)
                assert len(reader.fieldnames) == 82
                for row in reader:
                    assert None not in row and None not in row.values(), "CSV columns"
                    key = row["scenario"], row["planner"]
                    if key not in selected:
                        continue
                    if float(row["time_s"]) == 0:
                        assert key not in tracks
                        case = selected[key]
                        tracks[key] = TrackReplay(
                            case, rho, expected[tuple(case[k] for k in KEY)]
                        )
                    tracks[key].process(row)
            assert set(tracks) == set(selected)
            for case in cases:
                track = tracks[case["scenario"], case["planner"]]
                stats = track.validate()
                reconstruction.append(stats)
                intervals += stats["intervals"]
                track.export(events, snapshots, pairs)
        assert intervals == pinned["selected_intervals"]
        assert hashes == model_source["cohorts"][cohort]["raw_sha256"]
        sources["cohorts"][cohort] = dict(
            path=str(run),
            provenance=provenance,
            raw_sha256=hashes,
            reconstructed_intervals=intervals,
        )
        print(
            f"{cohort}: {intervals} observation-window intervals replayed", flush=True
        )
    assert len(reconstruction) == 368
    assert sum(r["original_checkpoints"] for r in reconstruction) == 2120
    assert sum(e["scheduled"] for e in events) == 368 * 7
    assert len(snapshots) == len(events) * len(OFFSETS) * 2
    assert len(pairs) == len(events) * (len(OFFSETS) - 1) * 2
    pair_summary, event_summary = summarize_pairs(pairs), summarize_events(events)
    sources["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        protocol_sha256=digest(PROTOCOL),
        simulator_sha256=digest(SIMULATOR),
        helpers_sha256=dict(sorted(helpers.items())),
    )
    sources["sampling"] = dict(
        offset_steps=OFFSETS,
        dt=DT,
        post_window_intervals=HORIZON,
        snapshot_truth="preceding physical CSV row",
    )
    sources["inherited_validation"] = dict(
        physics_contact_timers=True,
        new_physics_validation=False,
        source_sha256=PINNED["joint-admission-geometry/sources.json"],
    )
    sources["audit"] = dict(
        cases=len(reconstruction),
        replayed_intervals=sum(r["intervals"] for r in reconstruction),
        original_checkpoints=sum(r["original_checkpoints"] for r in reconstruction),
        opportunities=len(events),
        scheduled_opportunities=sum(e["scheduled"] for e in events),
        event_kind_counts={
            kind: sum(e["event_kind"] == kind for e in events) for kind in KINDS
        },
        snapshots=len(snapshots),
        available_snapshots=sum(r["available"] for r in snapshots),
        paired_rows=len(pairs),
        complete_action_windows=sum(e["post_window_complete"] for e in events),
        censored_action_windows=sum(e["post_window_censored"] for e in events),
        max_reconstruction_error=max(
            r["max_reconstruction_error"] for r in reconstruction
        ),
        max_previous_norm_error=max(
            r["max_previous_norm_error"] for r in reconstruction
        ),
        intervening_scheduled=sum(e["intervening_scheduled"] for e in events),
        intervening_precision=sum(e["intervening_precision"] for e in events),
        pair_summary_rows=len(pair_summary),
        event_summary_rows=len(event_summary),
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", accounting),
        ("reconstruction", reconstruction),
        ("events", events),
        ("snapshots", snapshots),
        ("pairs", pairs),
        ("pair-summary", pair_summary),
        ("event-summary", event_summary),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(sources["audit"], indent=2))


if __name__ == "__main__":
    main()
