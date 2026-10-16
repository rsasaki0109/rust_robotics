#!/usr/bin/env python3
"""Compare adjacent recorded gates, nominal command zeros and position changes."""

import argparse
from collections import defaultdict
import csv
from itertools import product
import json
import math
from pathlib import Path
import statistics

from analyze_commanded_recovery_arc import flag
from analyze_correction_centre_transitions import gate_context
from analyze_correction_gate_sequences import check_value, failure_mask
from analyze_epoch_divergence import REQUIREMENTS
from analyze_epoch_position_components import file_digest
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import DT, KEY
from analyze_terminal_arc_live import COHORTS, SIMULATOR
from summarize_stopping_preview import save_rows
from verify_joint_recovery import release_margin
from verify_stopping_preview import (
    exact_speed_distance,
    preview_components,
    segment_distance,
)

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/correction-gate-sequences"
PROTOCOL = ROOT / "results/correction-gate-boundaries/protocol.md"
EPS = 2e-8
PINNED = {
    "accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "events.csv": "55d8d645ac5ad71c58595a5b2a75ab16dd5d59b9ce07f40f1b810a8f5501dc7c",
    "intervals.csv": "b7a4e711bf66114e2e3b932b64a958cc9693a704e92630b4956800ac1a44d166",
    "sources.json": "302d0d71095c66b09075233d92dcc342d2e5c2d559d12fcc85daca1160cd806a",
}
KINDS = [
    "received",
    "missed",
    "precision_only",
    "received_and_precision",
    "missed_and_precision",
]
LABELS = [
    "joint_retained",
    "joint_lost",
    "joint_acquired",
    "joint_not_ready",
    "joint_route_exit",
    "joint_route_entry",
    "other",
]
FOCUS = [
    "joint_retained",
    "joint_lost",
    "joint_acquired",
    "joint_route_exit",
    "joint_route_entry",
]
DISTANCE_GAPS = [
    "recovery_obstacle_gap",
    "recovery_wall_gap",
    "stop_gap",
    "terminal_gap",
]
ZERO_REASONS = [
    "positive",
    "uncapped_zero",
    "speed_cap_zero",
    "both_zero",
    "rounded_zero_other",
]
DEPTHS = ["no_geometric_failure", "within_1cm", "beyond_1cm"]


def close(a, b):
    assert a == b or abs(a - b) < EPS, (a, b)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def label_pair(a, b):
    aj, bj = a["route"] == "joint", b["route"] == "joint"
    ar, br = flag(a["active_ready"]), flag(b["active_ready"])
    if aj and bj:
        return (
            "joint_retained"
            if ar and br
            else "joint_lost" if ar else "joint_acquired" if br else "joint_not_ready"
        )
    return "joint_route_exit" if aj else "joint_route_entry" if bj else "other"


def nominal_command(free, cap, uncapped, nominal):
    assert free >= 0 and cap >= 0 and uncapped >= 0 and nominal >= 0
    reconstructed = min(1.0, max(0.0, math.sqrt(0.0225 + free) - 0.15))
    close(cap, reconstructed)
    close(nominal, min(uncapped, cap))
    reason = (
        "positive"
        if nominal > 0
        else (
            "both_zero"
            if uncapped == 0 and cap == 0
            else (
                "uncapped_zero"
                if uncapped == 0
                else "speed_cap_zero" if cap == 0 else "rounded_zero_other"
            )
        )
    )
    return reason, max(abs(cap - reconstructed), abs(nominal - min(uncapped, cap)))


def signed_state(before, row, context):
    truth = [float(before[a]) for a in ["x", "y", "yaw"]]
    state = dict(time_s=row["time_s"], truth_time_s=before["time_s"])
    for kind in ["truth", "raw", "recovery", "preview"]:
        pose = (
            truth
            if kind == "truth"
            else [
                (
                    truth[i] + float(row["noise_" + a]) + float(row["bias_" + a])
                    if kind == "raw"
                    else float(
                        row[("preview_pose_" if kind == "preview" else "recovery_") + a]
                    )
                )
                for i, a in enumerate(["x", "y", "yaw"])
            ]
        )
        state.update({kind + "_" + a: v for a, v in zip(["x", "y", "yaw"], pose)})
    for part, axis in product(["noise", "bias"], ["x", "y"]):
        state[part + "_" + axis] = float(row[part + "_" + axis])
    margin = (
        0.6
        + 2 * math.sqrt(2) * float(row["effective_position_sigma"])
        + 0.02 * float(row["observation_age"])
    )
    release = release_margin(row)
    x, y, yaw = (state["recovery_" + a] for a in ["x", "y", "yaw"])
    end = x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw)
    state.update(
        stationary_gap=0.02 - abs(float(row["measured_speed"])),
        fresh_gap=1.5 - float(row["observation_age"]),
        samples_gap=int(row["recovery_sample_count"]) - 10,
        recovery_obstacle_gap=float(row["recovery_clearance"]) - margin,
        recovery_wall_gap=min(x, y, *end, 50 - x, 50 - y, 50 - end[0], 50 - end[1])
        - margin,
        stop_gap=float(row["stop_preview_clearance_lower_bound"]) - release,
        terminal_gap=float(row["terminal_recovery_clearance"]) - 0.05 - release,
        recovery_margin=margin,
        release_margin=release,
        speed_margin=float(row["speed_margin"]),
        nominal_target_speed=float(row["nominal_target_speed"]),
        actual_target_speed=float(row["target_speed"]),
        uncapped_target_speed=float(row["uncapped_target_speed"]),
        speed_cap=float(row["speed_cap"]),
        speed_free_distance=float(row["speed_free_distance"]),
        nominal_steering=float(row["nominal_steering"]),
        stop_latched=flag(row["stop_latched"]),
    )
    checks = dict(
        stationary=state["stationary_gap"] >= 0,
        fresh=state["fresh_gap"] >= 0,
        samples=state["samples_gap"] >= 0,
        recovery_geometry=state["recovery_obstacle_gap"] > 0
        and state["recovery_wall_gap"] >= 0,
        positive_command=state["nominal_target_speed"] > 0,
        stop=state["stop_gap"] > 0,
        terminal=state["terminal_gap"] > 0,
    )
    for k, v in checks.items():
        if context["ready_" + k] != "":
            assert v == context["ready_" + k], (k, v, context)
    if context["route"] == "joint":
        expected = (
            0
            if state["stop_latched"]
            else min(
                0.4,
                max(0.1, 0.5 * float(row["sensed_goal_distance"])),
                0.2 if flag(row["restart_limited"]) else 1,
            )
        )
        close(state["uncapped_target_speed"], expected)
    reason, error = nominal_command(
        *(
            state[k]
            for k in [
                "speed_free_distance",
                "speed_cap",
                "uncapped_target_speed",
                "nominal_target_speed",
            ]
        )
    )
    state["command_zero_reason"] = reason
    assert all(math.isfinite(v) for v in state.values() if isinstance(v, (int, float)))
    return state, error


def geometric_depth(state):
    failed = [
        state[k]
        for k in DISTANCE_GAPS
        if state[k] < 0 or (k != "recovery_wall_gap" and state[k] == 0)
    ]
    return (
        "no_geometric_failure"
        if not failed
        else "within_1cm" if min(failed) >= -0.01 else "beyond_1cm"
    )


def change_record(transition, a, b, old_current):
    out = {
        k: transition[k]
        for k in [
            "transition_id",
            "event_id",
            "phase",
            "end_offset_steps",
            "label",
            "pre_state_id",
            "post_state_id",
        ]
    }
    out.update(
        failed_mask=(
            int(old_current["failed_mask"])
            if transition["label"] == "joint_lost"
            else ""
        ),
        failed_requirements=(
            old_current["failed_requirements"]
            if transition["label"] == "joint_lost"
            else ""
        ),
        geometric_depth=(
            geometric_depth(b) if transition["label"] == "joint_lost" else ""
        ),
        command_zero_reason=b["command_zero_reason"],
        previous_joint_evidence=(
            float(old_current["timer_before_s"])
            if old_current["route"] == "joint"
            else ""
        ),
    )
    for axis in ["x", "y"]:
        out["delta_truth_" + axis] = b["truth_" + axis] - a["truth_" + axis]
    out["truth_shift_m"] = math.hypot(out["delta_truth_x"], out["delta_truth_y"])
    for kind in ["raw", "recovery", "preview"]:
        for side, state in [("pre", a), ("post", b)]:
            out[f"{side}_{kind}_error_m"] = math.dist(
                [state[kind + "_x"], state[kind + "_y"]],
                [state["truth_x"], state["truth_y"]],
            )
        for axis in ["x", "y"]:
            delta = b[kind + "_" + axis] - a[kind + "_" + axis]
            error_delta = (b[kind + "_" + axis] - b["truth_" + axis]) - (
                a[kind + "_" + axis] - a["truth_" + axis]
            )
            close(delta - out["delta_truth_" + axis], error_delta)
            out[f"delta_{kind}_{axis}"] = delta
            out[f"delta_{kind}_error_{axis}"] = error_delta
        out[kind + "_shift_m"] = math.hypot(
            out[f"delta_{kind}_x"], out[f"delta_{kind}_y"]
        )
        out[kind + "_yaw_change_deg"] = math.degrees(
            wrap(b[kind + "_yaw"] - a[kind + "_yaw"])
        )
        out[kind + "_error_change_m"] = (
            out[f"post_{kind}_error_m"] - out[f"pre_{kind}_error_m"]
        )
    for part in ["noise", "bias"]:
        for axis in ["x", "y"]:
            out[f"delta_{part}_{axis}"] = b[part + "_" + axis] - a[part + "_" + axis]
        out[part + "_shift_m"] = math.hypot(
            out[f"delta_{part}_x"], out[f"delta_{part}_y"]
        )
    for axis in ["x", "y"]:
        close(
            out["delta_raw_error_" + axis],
            out["delta_noise_" + axis] + out["delta_bias_" + axis],
        )
    for k in [
        *DISTANCE_GAPS,
        "nominal_target_speed",
        "uncapped_target_speed",
        "speed_cap",
        "speed_free_distance",
        "recovery_margin",
        "release_margin",
    ]:
        out["delta_" + k] = b[k] - a[k]
    return out


def geometry_record(state, before, row, obstacles):
    raw_free = exact_speed_distance(before, row, obstacles)
    stop, terminal, n = preview_components(before, row, obstacles)
    x, y, yaw = (float(row["recovery_" + a]) for a in ["x", "y", "yaw"])
    end = x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw)
    recovery = min(segment_distance(p, (x, y), end) for p in obstacles)
    logged_recovery = float(row["recovery_clearance"])
    logged_terminal = float(row["terminal_recovery_clearance"])
    lower = float(row["stop_preview_clearance_lower_bound"])
    close(raw_free, state["speed_free_distance"])
    close(recovery, logged_recovery)
    close(terminal, logged_terminal)
    assert lower <= stop + EPS, (lower, stop)
    margin = state["release_margin"]
    boundary = abs(lower - margin) <= EPS or abs(stop - margin) <= EPS
    if not boundary:
        assert (lower > margin) == (stop > margin)
    return dict(
        state_id=state["state_id"],
        source_interval_id=state["source_interval_id"],
        speed_free_logged=state["speed_free_distance"],
        speed_free_exact=raw_free,
        speed_free_error=abs(raw_free - state["speed_free_distance"]),
        recovery_obstacle_logged=logged_recovery,
        recovery_obstacle_exact=recovery,
        recovery_obstacle_error=abs(recovery - logged_recovery),
        stop_lower_bound=lower,
        stop_exact=stop,
        stop_lower_slack=stop - lower,
        stop_threshold_boundary=boundary,
        terminal_logged=logged_terminal,
        terminal_exact=terminal,
        terminal_error=abs(terminal - logged_terminal),
        preview_steps=n,
    )


def summarize(events, transitions, changes):
    by_event = {e["event_id"]: e for e in events}
    groups, change_groups, patterns = (
        defaultdict(list),
        defaultdict(list),
        defaultdict(list),
    )
    for row in transitions:
        e = by_event[row["event_id"]]
        for cohort, kind, route in product(
            ["all", e["cohort"]], ["all", e["event_kind"]], ["all", e["event_route"]]
        ):
            groups[cohort, e["variant"], kind, route, row["phase"]].append(row)
    for c in changes:
        e = by_event[c["event_id"]]
        for cohort, kind in product(["all", e["cohort"]], ["all", e["event_kind"]]):
            change_groups[cohort, e["variant"], kind, c["phase"], c["label"]].append(c)
            if c["label"] == "joint_lost":
                for part in ["all", e["partition"]]:
                    patterns[
                        cohort,
                        e["variant"],
                        kind,
                        c["phase"],
                        part,
                        c["failed_mask"],
                        c["geometric_depth"],
                        c["command_zero_reason"],
                    ].append(c)
    summary = []
    for cohort, variant, kind, route, phase in product(
        ["all", *COHORTS],
        ["epoch", "curved"],
        ["all", *KINDS],
        ["all", "joint", "independent", "inactive"],
        ["immediate", "post"],
    ):
        ee = [
            e
            for e in events
            if e["variant"] == variant
            and (cohort == "all" or e["cohort"] == cohort)
            and (kind == "all" or e["event_kind"] == kind)
            and (route == "all" or e["event_route"] == route)
        ]
        rows = groups[cohort, variant, kind, route, phase]
        counts = {k: sum(r["label"] == k for r in rows) for k in LABELS}
        assert sum(counts.values()) == len(rows)
        summary.append(
            dict(
                cohort=cohort,
                variant=variant,
                event_kind=kind,
                event_route=route,
                phase=phase,
                source_events=len(ee),
                censored_source_events=sum(flag(e["post_window_censored"]) for e in ee),
                observed_events=len({r["event_id"] for r in rows}),
                cases=len(
                    {tuple(by_event[r["event_id"]][k] for k in KEY) for r in rows}
                ),
                adjacent_pairs=len(rows),
                **counts,
            )
        )
    matched = []
    for cohort, variant, kind, phase, label in product(
        ["all", *COHORTS],
        ["epoch", "curved"],
        ["all", *KINDS],
        ["immediate", "post"],
        FOCUS,
    ):
        rows = change_groups[cohort, variant, kind, phase, label]
        r = dict(
            cohort=cohort,
            variant=variant,
            event_kind=kind,
            phase=phase,
            label=label,
            pairs=len(rows),
            events=len({r["event_id"] for r in rows}),
            cases=len({tuple(by_event[r["event_id"]][k] for k in KEY) for r in rows}),
        )
        for field in [
            "truth_shift_m",
            "raw_shift_m",
            "recovery_shift_m",
            "preview_shift_m",
            "noise_shift_m",
            "bias_shift_m",
        ]:
            r["mean_" + field] = (
                statistics.fmean(a[field] for a in rows) if rows else ""
            )
        for part in ["raw", "recovery", "preview"]:
            for side in ["pre", "post"]:
                r[f"{side}_{part}_error_rms_m"] = (
                    math.sqrt(
                        statistics.fmean(a[f"{side}_{part}_error_m"] ** 2 for a in rows)
                    )
                    if rows
                    else ""
                )
        matched.append(r)
    loss = []
    fields = [
        "cohort",
        "variant",
        "event_kind",
        "phase",
        "event_partition",
        "failed_mask",
        "geometric_depth",
        "command_zero_reason",
    ]
    for key, rows in sorted(patterns.items()):
        loss.append(
            dict(
                **dict(zip(fields, key)),
                failed_requirements=rows[0]["failed_requirements"],
                pairs=len(rows),
                events=len({r["event_id"] for r in rows}),
                cases=len(
                    {tuple(by_event[r["event_id"]][k] for k in KEY) for r in rows}
                ),
                mean_prior_evidence_s=statistics.fmean(
                    r["previous_joint_evidence"] for r in rows
                ),
                max_prior_evidence_s=max(r["previous_joint_evidence"] for r in rows),
            )
        )
    return summary, matched, loss


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for name, sha in PINNED.items():
        assert digest(PRIOR / name) == sha, name
    prior = json.loads((PRIOR / "sources.json").read_text())
    helpers = dict(prior["analysis"]["helpers_sha256"])
    helpers["analyze_correction_gate_sequences.py"] = prior["analysis"]["script_sha256"]
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    assert digest(SIMULATOR) == prior["analysis"]["simulator_sha256"]
    # The preceding source pins the map through the centre-transition source.
    centre = ROOT / "results/correction-centre-transitions/sources.json"
    assert digest(centre) == prior["input_sha256"]["sources.json"]
    map_sha = json.loads(centre.read_text())["map_sha256"]
    assert digest(ROOT / "inputs/obstacles.csv") == map_sha
    maps = defaultdict(list)
    for r in read_rows(ROOT / "inputs/obstacles.csv"):
        maps[r["scenario"]].append((float(r["x"]), float(r["y"])))
    accounting, events, old_intervals = (
        read_rows(PRIOR / n) for n in ["accounting.csv", "events.csv", "intervals.csv"]
    )
    assert (
        len(accounting) == 648 and len(events) == 2583 and len(old_intervals) == 171291
    )
    E = {e["event_id"]: e for e in events}
    point_ids, refs, per_event = {}, defaultdict(list), defaultdict(list)
    for r in old_intervals:
        e = E[r["event_id"]]
        case = tuple(e[k] for k in KEY)
        key = case, round(float(r["time_s"]) / DT)
        if key not in point_ids:
            point_ids[key] = len(point_ids) + 1
        refs[key].append(r)
        per_event[r["event_id"]].append(r)
    assert len(point_ids) == prior["audit"]["checked_unique_decisions"]
    transitions, focus_ids = [], set()
    transitions_by_event = defaultdict(list)
    for e in events:
        rr = per_event[e["event_id"]]
        assert [int(r["offset_steps"]) for r in rr] == list(
            range(-1, 76 if flag(e["post_window_complete"]) else 1)
        )
        case = tuple(e[k] for k in KEY)
        for a, b in zip(rr, rr[1:]):
            assert int(b["offset_steps"]) - int(a["offset_steps"]) == 1
            pre = point_ids[case, round(float(a["time_s"]) / DT)]
            post = point_ids[case, round(float(b["time_s"]) / DT)]
            focused = any(
                r["route"] == "joint" and flag(r["active_ready"]) for r in [a, b]
            )
            t = dict(
                transition_id=len(transitions) + 1,
                event_id=e["event_id"],
                phase="immediate" if int(b["offset_steps"]) == 0 else "post",
                end_offset_steps=int(b["offset_steps"]),
                pre_interval_id=a["interval_id"],
                post_interval_id=b["interval_id"],
                pre_state_id=pre,
                post_state_id=post,
                label=label_pair(a, b),
                detailed=focused,
            )
            transitions.append(t)
            transitions_by_event[e["event_id"]].append(t)
            if focused:
                focus_ids.update([pre, post])
    states = [None] * len(point_ids)
    raw_focus = {}
    sources = dict(input_sha256=PINNED, map_sha256=map_sha, cohorts={})
    scanned = matched = 0
    max_command_error = 0
    by_case = defaultdict(list)
    for case, step in point_ids:
        by_case[case].append(step)
    for cohort in COHORTS:
        pinned = prior["cohorts"][cohort]
        run = (
            args.raw_root / cohort if args.raw_root else Path(pinned["path"])
        ).resolve()
        provenance = json.loads((run / "provenance.json").read_text())
        assert provenance == pinned["provenance"]
        files, hashes = defaultdict(list), {}
        for case in by_case:
            if case[0] == cohort:
                files[case[1:4]].append(case)
        for (variant, mode, seed), cases in sorted(files.items()):
            name = f"{variant}-{mode}-{seed}/trajectories.csv"
            hashes[name] = file_digest(run / name)
            assert hashes[name] == pinned["raw_sha256"][name]
            selected = {(c[4], c[5]): c for c in cases}
            before = {}
            steps = {}
            with (run / name).open() as stream:
                reader = csv.DictReader(stream)
                assert len(reader.fieldnames) == 82
                for row in reader:
                    assert None not in row and None not in row.values()
                    case = selected.get((row["scenario"], row["planner"]))
                    if case is None:
                        continue
                    step = round(float(row["time_s"]) / DT)
                    if step == 0:
                        assert case not in before
                        before[case], steps[case] = row, 0
                        continue
                    previous = before[case]
                    assert step == steps[case] + 1
                    close(float(row["measured_speed"]), float(previous["v"]))
                    scanned += 1
                    key = case, step
                    if key in point_ids:
                        context = gate_context(previous, row)
                        mask, failed = failure_mask(context)
                        for r in refs[key]:
                            for field, value in context.items():
                                check_value(value, r[field])
                            assert (
                                int(r["failed_mask"]) == mask
                                and r["failed_requirements"] == failed
                            )
                            matched += 1
                        state, error = signed_state(previous, row, context)
                        sid = point_ids[key]
                        states[sid - 1] = dict(
                            state_id=sid,
                            source_interval_id=refs[key][0]["interval_id"],
                            **state,
                        )
                        max_command_error = max(max_command_error, error)
                        if sid in focus_ids:
                            raw_focus[sid] = (previous, row, case[4])
                    before[case], steps[case] = row, step
            assert all(steps[c] == 9000 for c in cases)
        assert hashes == pinned["raw_sha256"]
        sources["cohorts"][cohort] = dict(
            path=str(run), provenance=provenance, raw_sha256=hashes
        )
        print(f"{cohort}: signed inputs and nominal commands matched", flush=True)
    assert (
        scanned == 3312000
        and matched == len(old_intervals)
        and all(s is not None for s in states)
    )
    assert set(raw_focus) == focus_ids
    geometry = []
    for sid in sorted(focus_ids):
        before, row, scenario = raw_focus[sid]
        geometry.append(geometry_record(states[sid - 1], before, row, maps[scenario]))
    print(
        f"{len(geometry)} distinct speed/recovery/stopping/terminal geometries audited",
        flush=True,
    )
    old_by_id = {r["interval_id"]: r for r in old_intervals}
    changes = [
        change_record(
            t,
            states[t["pre_state_id"] - 1],
            states[t["post_state_id"] - 1],
            old_by_id[t["post_interval_id"]],
        )
        for t in transitions
        if t["detailed"]
    ]
    for e in events:
        rr = transitions_by_event[e["event_id"]]
        for phase, label in product(["immediate", "post"], LABELS):
            e[f"{phase}_{label}_pairs"] = sum(
                t["phase"] == phase and t["label"] == label for t in rr
            )
    summary, matched_summary, patterns = summarize(events, transitions, changes)
    sources["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        protocol_sha256=digest(PROTOCOL),
        helpers_sha256=dict(sorted(helpers.items())),
        simulator_sha256=digest(SIMULATOR),
    )
    sources["schema"] = dict(
        labels=LABELS,
        zero_reasons=ZERO_REASONS,
        geometric_depths=DEPTHS,
        geometry_tolerance_m=EPS,
        near_failure_m=0.01,
        truth="preceding physical CSV row",
    )
    sources["inherited_validation"] = dict(
        source_sha256=PINNED["sources.json"],
        observation_reconstruction=True,
        physics=True,
        new_rollout=False,
    )
    sources["audit"] = dict(
        accounted_cases=len(accounting),
        cases=len(by_case),
        opportunities=len(events),
        scanned_selected_intervals=scanned,
        matched_interval_references=matched,
        states=len(states),
        transitions=len(transitions),
        detailed_changes=len(changes),
        geometry_states=len(geometry),
        max_command_error=max_command_error,
        max_speed_free_error=max(g["speed_free_error"] for g in geometry),
        max_recovery_geometry_error=max(g["recovery_obstacle_error"] for g in geometry),
        max_terminal_geometry_error=max(g["terminal_error"] for g in geometry),
        stop_threshold_boundaries=sum(g["stop_threshold_boundary"] for g in geometry),
        summary_rows=len(summary),
        change_summary_rows=len(matched_summary),
        loss_pattern_rows=len(patterns),
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", accounting),
        ("events", events),
        ("states", states),
        ("transitions", transitions),
        ("changes", changes),
        ("geometry", geometry),
        ("summary", summary),
        ("change-summary", matched_summary),
        ("loss-patterns", patterns),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(sources["audit"], indent=2))


if __name__ == "__main__":
    main()
