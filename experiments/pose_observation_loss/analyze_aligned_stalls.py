#!/usr/bin/env python3
"""Diagnose all new aligned timeouts and four known lost holds on fixed tracks."""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path

from analyze_entry_cycles import physical_intervals, wait_seconds
from analyze_filter_stalls import GOALS, category, conditions, hold_budget, pending
from analyze_recovery_stalls import (
    MODES,
    center_clearance,
    command_mode,
    digest,
    forward_clearance,
)
from summarize_stopping_preview import save_rows
from verify_joint_recovery import (
    check_guard_track,
    check_joint_track,
    release_clearance,
)
from verify_motion_filter import check_motion_filter_track
from verify_stopping_preview import (
    exact_release_preview,
    preview_components,
    segment_distance,
)

ROOT = Path(__file__).resolve().parent
KEY = ["cohort", "mode", "seed", "scenario", "planner"]
DT = 0.02
STATES = ["other_blocked", "both_blocked", "stop_only", "terminal_only", "ready"]
REQUIREMENTS = [
    "stationary",
    "fresh",
    "samples",
    "recovery_geometry",
    "positive_command",
    "stop",
    "terminal",
]


def release_requirements(before, row):
    """Use the aligned release metric, not the combined entry metric."""
    if pending(before, row):
        route = "joint"
        ready = conditions(before, row)
        del ready["guard_preview"]
    elif before["guard_active"] == "true":
        route = "independent"
        ready = dict(stationary=abs(float(before["v"])) <= 0.02)
    else:
        return None, {}
    margin = float(row["guard_release_margin"])
    ready.update(
        stop=float(row["stop_preview_clearance_lower_bound"]) > margin,
        terminal=float(row["terminal_recovery_clearance"]) > margin + 0.05,
    )
    return route, ready


def gate_state(ready):
    if not all(
        value for key, value in ready.items() if key not in ["stop", "terminal"]
    ):
        return "other_blocked"
    if ready["stop"] and ready["terminal"]:
        return "ready"
    if ready["stop"]:
        return "terminal_only"
    return "stop_only" if ready["terminal"] else "both_blocked"


def tail_gates(pairs):
    """Partition eligible intervals; consecutive runs are clipped to this window."""
    counts = {route: Counter() for route in ["independent", "joint"]}
    runs = {route: dict(actual=0, without_terminal=0) for route in counts}
    maxima = {route: dict(actual=0, without_terminal=0) for route in counts}
    previous = None
    for before, row in pairs:
        route, ready = release_requirements(before, row)
        if route != previous:
            for item in runs.values():
                item.update(actual=0, without_terminal=0)
        previous = route
        if route is None:
            continue
        state = gate_state(ready)
        counts[route][state] += 1
        counts[route].update("fail_" + key for key, value in ready.items() if not value)
        for kind, passed in [
            ("actual", all(ready.values())),
            ("without_terminal", all(v for k, v in ready.items() if k != "terminal")),
        ]:
            runs[route][kind] = runs[route][kind] + 1 if passed else 0
            maxima[route][kind] = max(maxima[route][kind], runs[route][kind])
    return counts, maxima


def checkpoint(case, before, row, obstacles, reasons):
    stop, terminal, steps = preview_components(before, row, obstacles)
    exact_release = exact_release_preview(stop, terminal, row)
    release_margin = float(row["guard_release_margin"])
    assert abs(terminal - float(row["terminal_recovery_clearance"])) < 2e-8
    for exact, logged, margin in [
        (stop, float(row["stop_preview_clearance_lower_bound"]), release_margin),
        (exact_release, release_clearance(row), release_margin),
        (
            min(stop, terminal - 0.1),
            float(row["preview_clearance_lower_bound"]),
            float(row["preview_margin"]),
        ),
    ]:
        assert logged <= exact + 2e-8
        if abs(exact - margin) > 2e-8 and abs(logged - margin) > 2e-8:
            assert (exact > margin) == (logged > margin)
    rx, ry, yaw = (float(row[f"recovery_{axis}"]) for axis in ["x", "y", "yaw"])
    end = rx + 0.6 * math.cos(yaw), ry + 0.6 * math.sin(yaw)
    measured_line = min(segment_distance(p, (rx, ry), end) for p in obstacles)
    assert abs(measured_line - float(row["recovery_clearance"])) < 2e-8
    recovery_margin = (
        0.6
        + 2 * math.sqrt(2) * float(row["effective_position_sigma"])
        + 0.02 * float(row["observation_age"])
    )
    assert conditions(before, row)["recovery_geometry"] == (
        forward_clearance(rx, ry, yaw, obstacles) > recovery_margin
    )
    truth = dict(
        row, **{f"preview_pose_{axis}": before[axis] for axis in ["x", "y", "yaw"]}
    )
    true_stop, true_terminal, _ = preview_components(before, truth, obstacles)
    route, ready = release_requirements(before, row)
    return dict(
        **{k: case[k] for k in KEY},
        time_s=row["time_s"],
        reasons="+".join(reasons),
        route=route or "none",
        gate_state=gate_state(ready) if ready else "none",
        stop_exact=stop,
        terminal_exact=terminal,
        release_exact=exact_release,
        release_margin=release_margin,
        preview_steps=steps,
        measured_recovery_line_with_bounds=forward_clearance(rx, ry, yaw, obstacles),
        measured_recovery_margin=recovery_margin,
        true_pose_stop_exact=true_stop,
        true_pose_terminal_exact=true_terminal,
        true_pose_release_exact=exact_release_preview(true_stop, true_terminal, row),
        true_pose_release_passes=exact_release_preview(true_stop, true_terminal, row)
        > release_margin,
    )


def analyze(case, track, obstacles):
    assert case["status"] == "timeout" and float(track[-1]["time_s"]) == 180
    check_guard_track(track)
    check_joint_track(track)
    check_motion_filter_track(track, True)
    travel = physical_intervals(track)
    for row in track[1:]:
        assert row["aligned_terminal_release"] == "true"
        expected = min(
            float(row["stop_preview_clearance_lower_bound"]),
            float(row["terminal_recovery_clearance"]) - 0.05,
        )
        assert abs(expected - release_clearance(row)) < 2e-9
    pairs = [
        (before, row)
        for before, row in zip(track, track[1:])
        if float(row["time_s"]) > 150 + 1e-8
    ]
    assert len(pairs) == 1500
    counts, maxima = tail_gates(pairs)
    modes = Counter(command_mode(row) for _, row in pairs)
    samples, selected_states = [], set()
    for before, row in pairs:
        reasons = []
        route, ready = release_requirements(before, row)
        state = gate_state(ready) if ready else "none"
        if route and (route, state) not in selected_states:
            selected_states.add((route, state))
            reasons.append("first_" + route + "_" + state)
        if (
            abs(float(before["v"])) <= 0.02
            and abs(float(row["observation_age"]) - 0.5) < 1e-8
        ):
            reasons.append("fresh_stationary")
        reasons.extend(
            field for field in ["guard_released", "resumed"] if row[field] == "true"
        )
        if float(row["time_s"]) == 180:
            reasons.append("final")
        if reasons:
            samples.append(checkpoint(case, before, row, obstacles, reasons))
    final = track[-1]
    assert (
        abs(
            math.hypot(
                float(final["x"]) - GOALS[case["scenario"]][0],
                float(final["y"]) - GOALS[case["scenario"]][1],
            )
            - float(case["goal_distance"])
        )
        < 2e-9
    )
    center = center_clearance(float(final["x"]), float(final["y"]), obstacles)
    forward = forward_clearance(
        *(float(final[axis]) for axis in ["x", "y", "yaw"]), obstacles
    )
    sigma = float(final["effective_position_sigma"])
    minimum_recovery = 0.6 + 2 * math.sqrt(2) * sigma
    first = float(case["first_goal_entry_s"])
    tail_travel = travel[-1] - travel[-1501]
    record = dict(
        **{k: case[k] for k in KEY},
        lost_hold=case["lost_hold"],
        classification=category(first, tail_travel),
        first_goal_entry_s=first,
        maximum_hold_budget_s=hold_budget(first),
        final_hold_time_s=case["hold_time_s"],
        final_goal_distance=case["goal_distance"],
        final_speed=case["final_speed"],
        full_wait_s=wait_seconds(track, 180),
        tail_travel=tail_travel,
        tail_wait_s=round(
            sum(modes[k] for k in ["both", "loss_only", "guard_only"]) * DT, 6
        ),
        final_guard_active=final["guard_active"],
        final_safety_paused=final["safety_paused"],
        final_true_center_clearance=center,
        final_true_forward_clearance=forward,
        final_minimum_stop_release_margin=0.55 + 2 * math.sqrt(2) * sigma,
        final_minimum_recovery_margin=minimum_recovery,
        final_center_below_fresh_recovery_margin=center <= minimum_recovery,
        final_forward_below_fresh_recovery_margin=forward <= minimum_recovery,
        **{f"tail_{k}_s": round(modes[k] * DT, 6) for k in MODES},
    )
    gate_rows = []
    for route in counts:
        gate_rows.append(
            dict(
                **{k: case[k] for k in KEY},
                route=route,
                **{k + "_s": round(counts[route][k] * DT, 6) for k in STATES},
                **{
                    k + "_fail_s": round(counts[route]["fail_" + k] * DT, 6)
                    for k in REQUIREMENTS
                },
                max_actual_ready_s=round(maxima[route]["actual"] * DT, 6),
                max_without_terminal_ready_s=round(
                    maxima[route]["without_terminal"] * DT, 6
                ),
            )
        )
    return record, samples, gate_rows, len(track) - 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ["known-white", "white", "correlated"]:
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    snapshot = ROOT / "results/aligned-release"
    pinned = list(csv.DictReader((snapshot / "all-results.csv").open()))
    sources = json.loads((snapshot / "sources.json").read_text())
    lookup = {tuple(r[k] for k in ["variant"] + KEY): r for r in pinned}
    selected = []
    for r in pinned:
        if r["variant"] != "aligned" or r["status"] != "timeout":
            continue
        lost = lookup[("entryline", *(r[k] for k in KEY))]["held"] == "true"
        if lost or r["cohort"] in ["white-validation", "correlated-validation"]:
            selected.append(dict(r, lost_hold=lost))
    assert len(selected) == 97 and sum(r["lost_hold"] for r in selected) == 4
    records, checkpoints, gate_rows, lost_pairs, evidence = [], [], [], [], {}
    intervals = 0
    for cohort, run in [
        ("known-white", args.known_white),
        ("white-validation", args.white),
        ("correlated-validation", args.correlated),
    ]:
        provenance = json.loads((run / "provenance.json").read_text())
        assert provenance == sources[cohort]["provenance"]
        rows = list(csv.DictReader((run / "all-results.csv").open()))
        assert rows == [
            {k: v for k, v in r.items() if k != "cohort"}
            for r in pinned
            if r["cohort"] == cohort
        ]
        inputs = run / "planning-inputs/fine-padding"
        assert {p.name: digest(p) for p in inputs.glob("*.csv")} == provenance[
            "inputs_by_variant_sha256"
        ]["aligned"]
        maps = {}
        for row in csv.DictReader((inputs / "obstacles.csv").open()):
            maps.setdefault(row["scenario"], []).append(
                (float(row["x"]), float(row["y"]))
            )
        hashes = {}
        subset = [r for r in selected if r["cohort"] == cohort]
        for mode, seed in sorted({(r["mode"], r["seed"]) for r in subset}):
            cases = [r for r in subset if (r["mode"], r["seed"]) == (mode, seed)]
            tracks = {}
            for variant in ["aligned", "entryline"]:
                wanted = {
                    (r["scenario"], r["planner"])
                    for r in cases
                    if variant == "aligned" or r["lost_hold"]
                }
                if not wanted:
                    continue
                path = run / f"{variant}-{mode}-{seed}/trajectories.csv"
                hashes[str(path.relative_to(run))] = digest(path)
                tracks[variant] = {key: [] for key in wanted}
                with path.open() as stream:
                    for row in csv.DictReader(stream):
                        key = row["scenario"], row["planner"]
                        if key in wanted:
                            tracks[variant][key].append(row)
            for case in cases:
                track = tracks["aligned"].pop((case["scenario"], case["planner"]))
                record, checked, gates, count = analyze(
                    case, track, maps[case["scenario"]]
                )
                records.append(record)
                checkpoints.extend(checked)
                gate_rows.extend(gates)
                intervals += count
                if case["lost_hold"]:
                    ref = tracks["entryline"].pop((case["scenario"], case["planner"]))
                    check_guard_track(ref)
                    check_joint_track(ref)
                    physical_intervals(ref)
                    intervals += len(ref) - 1
                    reference = lookup[("entryline", *(case[k] for k in KEY))]
                    horizon = float(reference["time_s"])
                    assert float(ref[-1]["time_s"]) == horizon
                    lost_pairs.append(
                        dict(
                            record,
                            reference_completion_s=horizon,
                            reference_matched_wait_s=wait_seconds(ref, horizon),
                            aligned_matched_wait_s=wait_seconds(track, horizon),
                        )
                    )
        evidence[cohort] = dict(
            provenance=provenance,
            raw_sha256=hashes,
            summary_sha256=digest(run / "all-results.csv"),
        )
        print(f"Checked {cohort}", flush=True)
    assert len(records) == 97 and len(lost_pairs) == 4
    summary = []
    gate_summary = []
    for cohort in ["known-white", "white-validation", "correlated-validation"]:
        for route in ["independent", "joint"]:
            group = [
                r for r in gate_rows if r["cohort"] == cohort and r["route"] == route
            ]
            threshold = 0.5 if route == "independent" else 1
            gate_summary.append(
                dict(
                    cohort=cohort,
                    route=route,
                    selected_cases=len(group),
                    **{
                        k: round(sum(r[k] for r in group), 6)
                        for k in [
                            *(s + "_s" for s in STATES),
                            *(s + "_fail_s" for s in REQUIREMENTS),
                        ]
                    },
                    cases_with_actual_window=sum(
                        r["max_actual_ready_s"] >= threshold - 1e-8 for r in group
                    ),
                    cases_with_without_terminal_window=sum(
                        r["max_without_terminal_ready_s"] >= threshold - 1e-8
                        for r in group
                    ),
                )
            )
    for cohort in ["white-validation", "correlated-validation"]:
        counts = Counter(r["classification"] for r in records if r["cohort"] == cohort)
        summary.extend(
            dict(cohort=cohort, classification=k, cases=v)
            for k, v in sorted(counts.items())
        )
    evidence["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        selection_sha256=digest(snapshot / "all-results.csv"),
        helper_sha256={
            name: digest(ROOT / name)
            for name in [
                "analyze_entry_cycles.py",
                "analyze_filter_stalls.py",
                "analyze_recovery_stalls.py",
                "verify_joint_recovery.py",
                "verify_motion_filter.py",
                "verify_stopping_preview.py",
                "summarize_stopping_preview.py",
            ]
        },
        physical_intervals_checked=intervals,
        geometry_checkpoints=len(checkpoints),
        selection="all 93 new aligned timeouts and four known lost holds, plus their reference tracks",
        tail_start_exclusive_s=150,
        tail_end_inclusive_s=180,
        note="Readiness runs clipped to tail window. True-pose forecasts retain logged command and margins; no policy replay.",
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, data in [
        ("timeout-cases.csv", records),
        ("tail-gates.csv", gate_rows),
        ("gate-summary.csv", gate_summary),
        ("checkpoints.csv", checkpoints),
        ("lost-pairs.csv", lost_pairs),
        ("summary.csv", summary),
    ]:
        save_rows(args.output / name, data)
    (args.output / "sources.json").write_text(json.dumps(evidence, indent=2) + "\n")
    for row in summary:
        print(row)
    print(
        f"Checked {intervals} physical intervals and {len(checkpoints)} geometry checkpoints"
    )


if __name__ == "__main__":
    main()
