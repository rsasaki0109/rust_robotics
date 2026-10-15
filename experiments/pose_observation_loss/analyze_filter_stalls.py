#!/usr/bin/env python3
"""Diagnose frozen motion-filter timeouts and four lost holds; no policy changes."""

import argparse
from collections import Counter
import csv
import json
import math
import os
from pathlib import Path
import tempfile

from analyze_entry_cycles import physical_intervals, wait_seconds
from analyze_recovery_stalls import (
    MODES,
    command_mode,
    digest,
    forward_clearance,
    center_clearance,
)
from summarize_stopping_preview import save_rows
from verify_joint_recovery import check_guard_track, check_joint_track
from verify_motion_filter import check_motion_filter_track
from verify_stopping_preview import exact_preview, exact_speed_distance

ROOT = Path(__file__).resolve().parent
KEY = ["cohort", "mode", "seed", "scenario", "planner"]
GOALS = {"open_50x50": (48, 48), "dense_50x50": (48, 48), "maze_50x50": (35, 45)}
DT = 0.02


def hold_budget(first_entry):
    # The simulator credits DT at the first qualifying endpoint, so include it.
    return max(0.0, 180 - first_entry + DT) if first_entry >= 0 else 0.0


def category(first_entry, tail_travel):
    if first_entry >= 0:
        return (
            "late_entry"
            if hold_budget(first_entry) < 10 - 1e-8
            else "earlier_entry_without_hold"
        )
    return "no_entry_stationary" if tail_travel < 0.1 else "no_entry_moving"


def conditions(before, row):
    margin = (
        0.6
        + 2 * math.sqrt(2) * float(row["effective_position_sigma"])
        + 0.02 * float(row["observation_age"])
    )
    x, y, yaw = (float(row[f"recovery_{k}"]) for k in ["x", "y", "yaw"])
    bounds = all(
        margin <= p <= 50 - margin
        for p in [x, y, x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw)]
    )
    return dict(
        stationary=abs(float(before["v"])) <= 0.02,
        fresh=float(row["observation_age"]) <= 1.5,
        samples=int(row["recovery_sample_count"]) >= 10,
        recovery_geometry=float(row["recovery_clearance"]) > margin and bounds,
        positive_command=float(row["nominal_target_speed"]) > 0,
        guard_preview=float(row["preview_clearance_lower_bound"])
        > float(row["guard_release_margin"]),
    )


def pending(before, row):
    return before["safety_paused"] == "true" or (
        row["correction"] == "true" and row["correction_received"] == "false"
    )


def filtered_speed_distance(before, row, obstacles):
    # exact_speed_distance normally reconstructs the raw observation. Substitute
    # only its pose, retaining the actual planned steering and original margin.
    pose = dict(
        before, **{axis: row[f"preview_pose_{axis}"] for axis in ["x", "y", "yaw"]}
    )
    errors = {
        f"{kind}_{axis}": "0"
        for kind in ["noise", "bias"]
        for axis in ["x", "y", "yaw"]
    }
    return exact_speed_distance(pose, dict(row, **errors), obstacles)


def command_spans(track):
    spans = []
    for row in track[1:]:
        mode = command_mode(row)
        if spans and spans[-1]["state"] == mode:
            spans[-1]["duration_s"] = round(spans[-1]["duration_s"] + DT, 6)
        else:
            spans.append(
                dict(
                    start_s=round(float(row["time_s"]) - DT, 6),
                    duration_s=DT,
                    state=mode,
                )
            )
    return spans


def checkpoint(case, before, row, obstacles, reasons):
    exact, _ = exact_preview(before, row, obstacles)
    for margin in [float(row["preview_margin"]), float(row["guard_release_margin"])]:
        lower = float(row["preview_clearance_lower_bound"])
        if abs(exact - margin) > 2e-8 and abs(lower - margin) > 2e-8:
            assert (exact > margin) == (lower > margin)
    raw_free = exact_speed_distance(before, row, obstacles)
    assert abs(raw_free - float(row["speed_free_distance"])) < 1e-8
    filtered_free = filtered_speed_distance(before, row, obstacles)
    target = min(
        float(row["uncapped_target_speed"]),
        max(0, min(1, math.sqrt(0.0225 + filtered_free) - 0.15)),
    )
    counter = dict(row, nominal_target_speed=str(target))
    counter_preview, _ = exact_preview(before, counter, obstacles)
    truth = dict(
        counter, **{f"preview_pose_{axis}": before[axis] for axis in ["x", "y", "yaw"]}
    )
    true_preview, _ = exact_preview(before, truth, obstacles)
    ready = conditions(before, row)
    counter_ready = dict(
        ready,
        positive_command=target > 0,
        guard_preview=counter_preview > float(row["guard_release_margin"]),
    )
    rx, ry, yaw = (float(row[f"recovery_{axis}"]) for axis in ["x", "y", "yaw"])
    recovery_line = forward_clearance(rx, ry, yaw, obstacles)
    margin = (
        0.6
        + 2 * math.sqrt(2) * float(row["effective_position_sigma"])
        + 0.02 * float(row["observation_age"])
    )
    assert ready["recovery_geometry"] == (recovery_line > margin)
    is_pending = pending(before, row)
    return dict(
        **{k: case[k] for k in KEY},
        time_s=row["time_s"],
        reasons="+".join(reasons),
        loss_pending=is_pending,
        guard_active=row["guard_active"],
        raw_speed_free=raw_free,
        filtered_speed_free=filtered_free,
        raw_nominal_target=row["nominal_target_speed"],
        counter_nominal_target=target,
        actual_preview_exact=exact,
        counter_preview_exact=counter_preview,
        counter_true_preview_exact=true_preview,
        counter_true_radius_clear=true_preview > 0.5,
        recovery_line_exact_with_bounds=recovery_line,
        recovery_margin=margin,
        raw_instant_joint_ready=is_pending and all(ready.values()),
        counter_instant_joint_ready=is_pending and all(counter_ready.values()),
        counter_removes_zero_speed=is_pending and raw_free == 0 and target > 0,
        **{f"ready_{k}": v for k, v in ready.items()},
    )


def analyze(case, track, obstacles):
    assert case["status"] == "timeout" and float(track[-1]["time_s"]) == 180
    check_motion_filter_track(track, True)
    check_joint_track(track)
    check_guard_track(track)
    travel = physical_intervals(track)
    tail = [(a, b) for a, b in zip(track, track[1:]) if float(b["time_s"]) > 150 + 1e-8]
    assert len(tail) == 1500
    modes, failures = Counter(), Counter()
    ready_run = max_ready_run = 0
    samples, zero_selected = [], False
    for before, row in tail:
        modes[command_mode(row)] += 1
        if pending(before, row):
            ready = conditions(before, row)
            failures.update(k for k, value in ready.items() if not value)
            ready_run = ready_run + 1 if all(ready.values()) else 0
            max_ready_run = max(max_ready_run, ready_run)
        else:
            ready_run = 0
        reasons = []
        stationary = abs(float(before["v"])) <= 0.02
        if stationary and abs(float(row["observation_age"]) - 0.5) < 1e-8:
            reasons.append("fresh_stationary")
        if (
            not zero_selected
            and stationary
            and float(row["speed_free_distance"]) == 0
            and (row["guard_active"] == "true" or pending(before, row))
        ):
            reasons.append("first_waiting_zero_speed")
            zero_selected = True
        for field in ["guard_released", "resumed"]:
            if row[field] == "true":
                reasons.append(field)
        if float(row["time_s"]) == 180:
            reasons.append("final")
        if reasons:
            samples.append(checkpoint(case, before, row, obstacles, reasons))
    first = float(case["first_goal_entry_s"])
    tail_travel = travel[-1] - travel[-1501]
    final = track[-1]
    sigma = float(final["effective_position_sigma"])
    final_center = center_clearance(float(final["x"]), float(final["y"]), obstacles)
    minimum_guard_release = 0.55 + 2 * math.sqrt(2) * sigma
    minimum_recovery = 0.6 + 2 * math.sqrt(2) * sigma
    goal = GOALS[case["scenario"]]
    assert (
        abs(
            math.hypot(float(final["x"]) - goal[0], float(final["y"]) - goal[1])
            - float(case["goal_distance"])
        )
        < 2e-9
    )
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
        tail_pending_max_ready_run_s=round(max_ready_run * DT, 6),
        final_stop_latched=final["stop_latched"],
        final_guard_active=final["guard_active"],
        final_safety_paused=final["safety_paused"],
        final_true_center_clearance=final_center,
        final_effective_position_sigma=sigma,
        final_minimum_guard_release_margin=minimum_guard_release,
        final_minimum_recovery_margin=minimum_recovery,
        final_true_center_below_minimum_guard_release=final_center
        <= minimum_guard_release,
        final_true_center_below_minimum_recovery=final_center <= minimum_recovery,
        **{f"tail_{k}_s": round(modes[k] * DT, 6) for k in MODES},
        **{
            f"tail_pending_{k}_fail_s": round(failures[k] * DT, 6)
            for k in conditions(track[-2], final)
        },
    )
    return record, samples, len(track) - 1


def plot(output, lost, spans):
    with tempfile.TemporaryDirectory(prefix="filter-stall-plot-") as cache:
        os.environ.setdefault("MPLCONFIGDIR", cache)
        os.environ.setdefault("XDG_CACHE_HOME", cache)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Patch

        plt.rcParams["svg.fonttype"] = "none"
        plt.rcParams["svg.hashsalt"] = "filter-stalls"
        colors = dict(
            zip(
                MODES,
                [
                    "#6d28d9",
                    "#ea580c",
                    "#2563eb",
                    "#64748b",
                    "#dc2626",
                    "#16a34a",
                    "#cbd5e1",
                ],
            )
        )
        fig, ax = plt.subplots(figsize=(14, 5.5), layout="constrained")
        labels = []
        for i, case in enumerate(lost):
            for span in spans:
                if all(span[k] == case[k] for k in KEY):
                    ax.broken_barh(
                        [(span["start_s"], span["duration_s"])],
                        (i - 0.3, 0.6),
                        facecolors=colors[span["state"]],
                    )
            ax.plot(
                float(case["joint_held_time_s"]), i, "|", color="black", markersize=22
            )
            if case["first_goal_entry_s"] >= 0:
                ax.plot(
                    case["first_goal_entry_s"], i, "D", color="#f59e0b", markersize=6
                )
            labels.append(
                f'{case["cohort"]}\n{case["mode"]} {case["seed"]} {case["scenario"].replace("_50x50", "")} {case["planner"]}'
            )
        ax.set_yticks(range(len(lost)), labels, fontsize=9)
        ax.invert_yaxis()
        ax.set(
            xlim=(0, 180),
            xlabel="Elapsed seconds; black tick: joint completion; gold diamond: first true goal entry",
            title="Four lost holds: filtered command timelines",
        )
        ax.legend(
            handles=[Patch(color=colors[k], label=k.replace("_", " ")) for k in MODES],
            loc="upper center",
            bbox_to_anchor=(0.5, -0.16),
            ncol=4,
            fontsize=8,
        )
        svg = output / "filter-stalls.svg"
        fig.savefig(svg, metadata={"Date": None})
        svg.write_text(
            "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(output / "filter-stalls.png", dpi=160)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ["reference", "known-white", "white", "correlated"]:
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    snapshot = ROOT / "results/motion-filter"
    pinned = list(csv.DictReader((snapshot / "all-results.csv").open()))
    pinned_sources = json.loads((snapshot / "sources.json").read_text())
    lookup = {tuple(r[k] for k in ["variant"] + KEY): r for r in pinned}
    selected = []
    for r in pinned:
        if r["variant"] != "filtered" or r["held"] == "true":
            continue
        ref = lookup[("joint", *(r[k] for k in KEY))]
        lost = ref["held"] == "true"
        if lost or r["cohort"] == "correlated-validation":
            selected.append(dict(r, lost_hold=lost))
    assert len(selected) == 33 and sum(r["lost_hold"] for r in selected) == 4
    records, checkpoints, lost, spans, evidence = [], [], [], [], {}
    interval_count = 0
    for cohort, run in [
        ("reference", args.reference),
        ("known-white", args.known_white),
        ("white-validation", args.white),
        ("correlated-validation", args.correlated),
    ]:
        provenance = json.loads((run / "provenance.json").read_text())
        for field in [
            "variants",
            "seeds",
            "noise_correlation_time_s",
            "simulator_sha256",
            "inputs_by_variant_sha256",
        ]:
            assert provenance[field] == pinned_sources[cohort]["provenance"][field]
        rows = list(csv.DictReader((run / "all-results.csv").open()))
        assert rows == [
            {k: v for k, v in r.items() if k != "cohort"}
            for r in pinned
            if r["cohort"] == cohort
        ]
        inputs = run / "planning-inputs/fine-padding"
        assert {p.name: digest(p) for p in inputs.glob("*.csv")} == provenance[
            "inputs_by_variant_sha256"
        ]["filtered"]
        maps = {}
        for r in csv.DictReader((inputs / "obstacles.csv").open()):
            maps.setdefault(r["scenario"], []).append((float(r["x"]), float(r["y"])))
        cache, hashes = {}, {}
        for case in (r for r in selected if r["cohort"] == cohort):
            tracks = {}
            for variant in (
                ["filtered", "joint"] if case["lost_hold"] else ["filtered"]
            ):
                path = run / f'{variant}-{case["mode"]}-{case["seed"]}/trajectories.csv'
                if path not in cache:
                    hashes[str(path.relative_to(run))] = digest(path)
                    wanted = {
                        (r["scenario"], r["planner"])
                        for r in selected
                        if r["cohort"] == cohort
                        and (r["mode"], r["seed"]) == (case["mode"], case["seed"])
                        and (variant == "filtered" or r["lost_hold"])
                    }
                    cache[path] = {key: [] for key in wanted}
                    with path.open() as stream:
                        for row in csv.DictReader(stream):
                            key = row["scenario"], row["planner"]
                            if key in wanted:
                                cache[path][key].append(row)
                tracks[variant] = cache[path].pop((case["scenario"], case["planner"]))
            record, checked, count = analyze(
                case, tracks["filtered"], maps[case["scenario"]]
            )
            records.append(record)
            checkpoints.extend(checked)
            interval_count += count
            if case["lost_hold"]:
                baseline = tracks["joint"]
                check_motion_filter_track(baseline, False)
                check_guard_track(baseline)
                check_joint_track(baseline)
                physical_intervals(baseline)
                interval_count += len(baseline) - 1
                ref = lookup[("joint", *(case[k] for k in KEY))]
                horizon = float(ref["time_s"])
                assert float(baseline[-1]["time_s"]) == horizon
                lost.append(
                    dict(
                        record,
                        joint_held_time_s=ref["time_s"],
                        joint_matched_wait_s=wait_seconds(baseline, horizon),
                        filtered_matched_wait_s=wait_seconds(
                            tracks["filtered"], horizon
                        ),
                    )
                )
                spans.extend(
                    dict(**{k: case[k] for k in KEY}, **span)
                    for span in command_spans(tracks["filtered"])
                )
        evidence[cohort] = dict(
            provenance=provenance,
            raw_sha256=hashes,
            summary_sha256=digest(run / "all-results.csv"),
        )
    classification = Counter(
        r["classification"] for r in records if r["cohort"] == "correlated-validation"
    )
    summary = [
        dict(cohort="correlated-validation", classification=k, cases=v)
        for k, v in sorted(classification.items())
    ]
    evidence["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        selection_sha256=digest(snapshot / "all-results.csv"),
        helper_sha256={
            name: digest(ROOT / name)
            for name in [
                "analyze_entry_cycles.py",
                "analyze_recovery_stalls.py",
                "verify_joint_recovery.py",
                "verify_motion_filter.py",
                "verify_stopping_preview.py",
                "summarize_stopping_preview.py",
            ]
        },
        physical_intervals_checked=interval_count,
        geometry_checkpoints=len(checkpoints),
        selection="four lost holds plus all thirty new correlated filtered timeouts (one overlap)",
        tail_start_exclusive_s=150,
        tail_end_inclusive_s=180,
        near_stationary_max_travel_exclusive=0.1,
        hold_budget_includes_first_endpoint_dt=DT,
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, rows in [
        ("timeout-cases.csv", records),
        ("lost-pairs.csv", lost),
        ("checkpoints.csv", checkpoints),
        ("timeline.csv", spans),
        ("summary.csv", summary),
    ]:
        save_rows(args.output / name, rows)
    (args.output / "sources.json").write_text(json.dumps(evidence, indent=2) + "\n")
    if args.plot:
        plot(args.output, lost, spans)
    for row in summary:
        print(row)
    print(
        f"Checked {len(records)} candidate timeouts, {len(lost)} lost pairs, {interval_count} physical intervals and {len(checkpoints)} geometry checkpoints"
    )


if __name__ == "__main__":
    main()
