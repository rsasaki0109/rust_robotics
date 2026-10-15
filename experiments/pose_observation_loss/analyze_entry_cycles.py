#!/usr/bin/env python3
"""Diagnose the ten new-seed holds lost by the recorded entry-buffer arm."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import tempfile

from analyze_recovery_stalls import (
    MODES,
    center_clearance,
    command_mode,
    digest,
    forward_clearance,
)
from summarize_stopping_preview import save_rows
from verify_joint_recovery import check_guard_track, check_joint_track
from verify_stopping_preview import exact_preview

ROOT = Path(__file__).resolve().parent
KEY = ["cohort", "mode", "seed", "scenario", "planner"]
DT = 0.02


def physical_intervals(track):
    """Check interval alignment and bicycle integration; return cumulative travel."""
    travel = [0.0]
    for before, row in zip(track, track[1:]):
        x, y, yaw, v = (float(before[k]) for k in ["x", "y", "yaw", "v"])
        a, steer = float(row["acceleration"]), float(row["steering"])
        assert abs(float(row["time_s"]) - float(before["time_s"]) - DT) < 1e-8
        assert abs(float(row["x"]) - x - v * math.cos(yaw) * DT) < 2e-9
        assert abs(float(row["y"]) - y - v * math.sin(yaw) * DT) < 2e-9
        assert abs(float(row["v"]) - v - a * DT) < 2e-9
        error = yaw + v / 0.4 * math.tan(steer) * DT - float(row["yaw"])
        assert abs(math.atan2(math.sin(error), math.cos(error))) < 3e-9
        travel.append(travel[-1] + math.hypot(float(row["x"]) - x, float(row["y"]) - y))
    return travel


def wait_seconds(track, end):
    return round(
        DT
        * sum(
            row["safety_paused"] == "true" or row["guard_active"] == "true"
            for row in track[1:]
            if float(row["time_s"]) <= end + 1e-8
        ),
        6,
    )


def release_bouts(track, travel):
    """Pair a release with its next guard entry; preserve interrupted/zero cases."""
    release = None
    interrupted = False
    bouts = []
    for i, row in enumerate(track[1:], 1):
        if row["safety_paused"] == "true":
            interrupted = True
        if row["guard_triggered"] == "true" and release is not None:
            start = track[release]
            lag = float(row["time_s"]) - float(start["time_s"])
            distance = travel[i - 1] - travel[release - 1]
            positive = float(start["target_speed"]) > 0
            bouts.append(
                dict(
                    release_index=release,
                    entry_index=i,
                    release_time_s=start["time_s"],
                    entry_time_s=row["time_s"],
                    lag_s=round(lag, 6),
                    travel_before_entry=distance,
                    release_positive_target=positive,
                    loss_interrupted=interrupted,
                    short_reentry=positive
                    and not interrupted
                    and lag <= 1 + 1e-8
                    and distance < 0.1,
                )
            )
            release = None
        if row["guard_released"] == "true":
            release = i
            interrupted = False
    return bouts


def analyze(case, reference, tracks, obstacles):
    key = {k: case[k] for k in KEY}
    track, baseline = tracks["entry"], tracks["joint"]
    travels = {}
    for variant, rows in tracks.items():
        check_guard_track(rows)
        check_joint_track(rows)
        travels[variant] = physical_intervals(rows)
    assert case["status"] == "timeout" and float(track[-1]["time_s"]) == 180
    assert float(baseline[-1]["time_s"]) == float(reference["time_s"])
    horizon = float(reference["time_s"])
    matched_index = round(horizon / DT)
    assert abs(float(track[matched_index]["time_s"]) - horizon) < 1e-8
    events = []
    for i, row in enumerate(track[1:], 1):
        if not any(
            row[k] == "true" for k in ["guard_triggered", "guard_released", "resumed"]
        ):
            continue
        before = track[i - 1]
        exact, steps = exact_preview(before, row, obstacles)
        base = (
            0.5
            + 2 * math.sqrt(2) * float(row["effective_position_sigma"])
            + 0.02 * float(row["observation_age"])
        )
        entry, release = float(row["preview_margin"]), float(
            row["guard_release_margin"]
        )
        extra = 0.1 * min(abs(float(before["v"])) / 0.4, 1)
        assert abs(entry - base - extra) < 3e-9
        assert abs(release - base - 0.05) < 3e-9
        for threshold in [base, entry, release]:
            lower = float(row["preview_clearance_lower_bound"])
            if abs(exact - threshold) > 2e-8 and abs(lower - threshold) > 2e-8:
                assert (lower > threshold) == (exact > threshold)
        triggered = row["guard_triggered"] == "true"
        if triggered:
            assert exact <= entry + 2e-8
        if row["guard_released"] == "true":
            assert exact > release - 2e-8
        events.append(
            dict(
                **key,
                time_s=row["time_s"],
                guard_triggered=triggered,
                guard_released=row["guard_released"],
                resumed=row["resumed"],
                speed_before=before["v"],
                exact_clearance=exact,
                preview_steps=steps,
                base_entry_margin=base,
                entry_margin=entry,
                release_margin=release,
                entry_extra=extra,
                reserve_only_entry=triggered and exact > base + 2e-8,
                threshold_ambiguous=triggered and abs(exact - base) <= 2e-8,
                target_speed=row["target_speed"],
                safety_paused=row["safety_paused"],
            )
        )
    by_time = {r["time_s"]: r for r in events}
    bouts = release_bouts(track, travels["entry"])
    for b in bouts:
        b.pop("release_index")
        b.pop("entry_index")
        b.update(key)
        b["reserve_only_entry"] = by_time[b["entry_time_s"]]["reserve_only_entry"]
    tail = [(a, b) for a, b in zip(track, track[1:]) if float(b["time_s"]) > 150 + 1e-8]
    assert len(tail) == 1500
    counts = {mode: 0 for mode in MODES}
    failures = {
        k: 0
        for k in [
            "stationary",
            "fresh",
            "samples",
            "recovery_geometry",
            "positive_command",
            "guard_preview",
        ]
    }
    pending_steps = ready_steps = ready_run = max_ready_run = 0
    for before, row in tail:
        counts[command_mode(row)] += 1
        pending = before["safety_paused"] == "true" or (
            row["correction"] == "true" and row["correction_received"] == "false"
        )
        if not pending:
            ready_run = 0
            continue
        pending_steps += 1
        recovery_margin = (
            0.6
            + 2 * math.sqrt(2) * float(row["effective_position_sigma"])
            + 0.02 * float(row["observation_age"])
        )
        x, y, yaw = (float(row[f"recovery_{k}"]) for k in ["x", "y", "yaw"])
        bounds = all(
            recovery_margin <= p <= 50 - recovery_margin
            for p in [x, y, x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw)]
        )
        conditions = dict(
            stationary=abs(float(before["v"])) <= 0.02,
            fresh=float(row["observation_age"]) <= 1.5,
            samples=int(row["recovery_sample_count"]) >= 10,
            recovery_geometry=float(row["recovery_clearance"]) > recovery_margin
            and bounds,
            positive_command=float(row["nominal_target_speed"]) > 0,
            guard_preview=float(row["preview_clearance_lower_bound"])
            > float(row["guard_release_margin"]),
        )
        for name, passed in conditions.items():
            failures[name] += not passed
        ready = all(conditions.values())
        ready_steps += ready
        ready_run = ready_run + 1 if ready else 0
        max_ready_run = max(max_ready_run, ready_run)
    final = track[-1]
    margin = 0.6 + 2 * math.sqrt(2) * float(final["effective_position_sigma"])
    final_center = center_clearance(float(final["x"]), float(final["y"]), obstacles)
    final_preview, _ = exact_preview(track[-2], final, obstacles)
    recovery_line = forward_clearance(
        *(float(final[f"recovery_{axis}"]) for axis in ["x", "y", "yaw"]),
        obstacles,
    )
    recovery_margin = margin + 0.02 * float(final["observation_age"])
    stationary = abs(float(track[-2]["v"])) <= 0.02
    recovery_ready = (
        stationary
        and float(final["observation_age"]) <= 1.5
        and int(final["recovery_sample_count"]) >= 10
        and recovery_line > recovery_margin
    )
    guard_ready = stationary and final_preview > float(final["guard_release_margin"])
    record = dict(
        **key,
        reference_hold_time_s=reference["time_s"],
        matched_horizon_s=horizon,
        joint_matched_wait_s=wait_seconds(baseline, horizon),
        entry_matched_wait_s=wait_seconds(track, horizon),
        joint_matched_travel=travels["joint"][-1],
        entry_matched_travel=travels["entry"][matched_index],
        entry_full_wait_s=wait_seconds(track, 180),
        entry_guard_entries=sum(row["guard_triggered"] == "true" for row in track[1:]),
        reserve_only_entries=sum(r["reserve_only_entry"] for r in events),
        threshold_ambiguous_entries=sum(r["threshold_ambiguous"] for r in events),
        completed_release_bouts=len(bouts),
        short_reentries=sum(b["short_reentry"] for b in bouts),
        short_reserve_only_reentries=sum(
            b["short_reentry"] and b["reserve_only_entry"] for b in bouts
        ),
        first_goal_entry_s=case["first_goal_entry_s"],
        final_goal_distance=case["goal_distance"],
        final_hold_time_s=case["hold_time_s"],
        final_stop_latched=final["stop_latched"],
        tail_travel=travels["entry"][-1] - travels["entry"][-1501],
        tail_wait_s=round(
            sum(counts[k] for k in ["both", "loss_only", "guard_only"]) * DT, 6
        ),
        final_center_clearance=final_center,
        fresh_recovery_margin=margin,
        final_center_below_fresh_recovery_margin=final_center <= margin,
        final_safety_paused=final["safety_paused"],
        final_guard_active=final["guard_active"],
        final_recovery_line_with_bounds=recovery_line,
        final_recovery_margin=recovery_margin,
        final_observation_age=final["observation_age"],
        final_recovery_sample_count=final["recovery_sample_count"],
        final_preview_exact=final_preview,
        final_guard_release_margin=final["guard_release_margin"],
        final_instant_recovery_ready=recovery_ready,
        final_instant_guard_ready=guard_ready,
        final_instant_joint_ready=recovery_ready
        and guard_ready
        and float(final["nominal_target_speed"]) > 0,
        tail_loss_pending_s=round(pending_steps * DT, 6),
        tail_pending_joint_ready_s=round(ready_steps * DT, 6),
        tail_pending_max_ready_run_s=round(max_ready_run * DT, 6),
        **{
            f"tail_pending_{name}_fail_s": round(count * DT, 6)
            for name, count in failures.items()
        },
        **{f"tail_{mode}_s": round(count * DT, 6) for mode, count in counts.items()},
    )
    return record, events, bouts, sum(len(r) - 1 for r in tracks.values())


def plot(output, cases):
    with tempfile.TemporaryDirectory(prefix="entry-cycle-plot-") as cache:
        os.environ.setdefault("MPLCONFIGDIR", cache)
        os.environ.setdefault("XDG_CACHE_HOME", cache)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.rcParams["svg.fonttype"] = "none"
        plt.rcParams["svg.hashsalt"] = "entry-cycles"
        fig, (left, right) = plt.subplots(1, 2, figsize=(14, 7), layout="constrained")
        y = list(range(len(cases)))
        for offset, field, label, color in [
            (-0.18, "joint_matched_wait_s", "Joint", "#94a3b8"),
            (0.18, "entry_matched_wait_s", "Entry reserve", "#2563eb"),
        ]:
            left.barh(
                [i + offset for i in y],
                [r[field] for r in cases],
                height=0.35,
                label=label,
                color=color,
            )
        labels = [
            f'{"W" if r["cohort"] == "white-validation" else "C"} {r["mode"]} {r["seed"]} {r["scenario"].replace("_50x50", "")} {r["planner"]}'
            for r in cases
        ]
        left.set_yticks(y, labels, fontsize=9)
        left.set(
            xlabel="Wait seconds until joint's successful termination",
            title="Same time horizon within each pair",
        )
        left.legend(fontsize=9)
        start = [0.0] * len(cases)
        for mode, color, label in [
            ("both", "#6d28d9", "Loss + guard"),
            ("loss_only", "#ea580c", "Loss only"),
            ("guard_only", "#2563eb", "Guard only"),
            ("goal_stop", "#64748b", "Goal stop"),
            ("speed_zero", "#dc2626", "Speed cap zero"),
            ("speed_limited", "#16a34a", "Speed limited"),
            ("ordinary", "#cbd5e1", "Ordinary"),
        ]:
            values = [r[f"tail_{mode}_s"] for r in cases]
            if any(values):
                right.barh(y, values, left=start, label=label, color=color)
                start = [a + b for a, b in zip(start, values)]
        right.set_yticks(y, [])
        right.set(
            xlim=(0, 30),
            xlabel="Aggregated command seconds (not a timeline)",
            title="Entry reserve: final 30 s",
        )
        right.legend(loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=3, fontsize=8)
        left.invert_yaxis()
        right.invert_yaxis()
        fig.suptitle(
            "Ten lost holds from the entry-buffer study; W: white, C: 1 s correlation"
        )
        svg = output / "entry-cycles.svg"
        fig.savefig(svg, metadata={"Date": None})
        svg.write_text(
            "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(output / "entry-cycles.png", dpi=160)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--white", type=Path, required=True)
    parser.add_argument("--correlated", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    snapshot = ROOT / "results/entry-buffer"
    pinned = list(csv.DictReader((snapshot / "all-results.csv").open()))
    sources = json.loads((snapshot / "sources.json").read_text())
    records, events, bouts = [], [], []
    evidence, intervals = {}, 0
    for cohort, run, tau in [
        ("white-validation", args.white, 0),
        ("correlated-validation", args.correlated, 1),
    ]:
        provenance = json.loads((run / "provenance.json").read_text())
        assert (
            provenance["variants"] == ["joint", "entry"]
            and provenance["noise_correlation_time_s"] == tau
        )
        assert (
            provenance["simulator_sha256"]
            == sources[cohort]["provenance"]["simulator_sha256"]
        )
        rows = list(csv.DictReader((run / "all-results.csv").open()))
        assert rows == [
            {k: v for k, v in r.items() if k != "cohort"}
            for r in pinned
            if r["cohort"] == cohort
        ]
        inputs = run / "planning-inputs/fine-padding"
        hashes = {p.name: digest(p) for p in inputs.glob("*.csv")}
        assert (
            hashes
            == provenance["inputs_by_variant_sha256"]["joint"]
            == provenance["inputs_by_variant_sha256"]["entry"]
        )
        maps = {}
        for r in csv.DictReader((inputs / "obstacles.csv").open()):
            maps.setdefault(r["scenario"], []).append((float(r["x"]), float(r["y"])))
        lookup = {
            tuple(r[k] for k in ["variant", "mode", "seed", "scenario", "planner"]): r
            for r in rows
        }
        selected = []
        for r in rows:
            if r["variant"] != "entry":
                continue
            ref = lookup[
                ("joint", *(r[k] for k in ["mode", "seed", "scenario", "planner"]))
            ]
            if ref["held"] == "true" and r["held"] == "false":
                selected.append((r, ref))
        raw_hashes, cache = {}, {}
        for case, ref in selected:
            tracks = {}
            for variant in ["joint", "entry"]:
                path = run / f'{variant}-{case["mode"]}-{case["seed"]}/trajectories.csv'
                if path not in cache:
                    raw_hashes[str(path.relative_to(run))] = digest(path)
                    wanted = {
                        (r["scenario"], r["planner"])
                        for r, _ in selected
                        if (r["mode"], r["seed"]) == (case["mode"], case["seed"])
                    }
                    cache[path] = {key: [] for key in wanted}
                    with path.open() as f:
                        for row in csv.DictReader(f):
                            key = row["scenario"], row["planner"]
                            if key in wanted:
                                cache[path][key].append(row)
                tracks[variant] = cache[path][case["scenario"], case["planner"]]
            record, checked, paired, count = analyze(
                dict(cohort=cohort, **case), ref, tracks, maps[case["scenario"]]
            )
            records.append(record)
            events.extend(checked)
            bouts.extend(paired)
            intervals += count
        evidence[cohort] = dict(
            provenance=provenance,
            raw_sha256=raw_hashes,
            summary_sha256=digest(run / "all-results.csv"),
        )
    assert len(records) == 10 and len({tuple(r[k] for k in KEY) for r in records}) == 10
    evidence["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        helper_sha256={
            name: digest(ROOT / name)
            for name in [
                "analyze_recovery_stalls.py",
                "summarize_stopping_preview.py",
                "verify_joint_recovery.py",
                "verify_stopping_preview.py",
            ]
        },
        selection_snapshot_sha256=digest(snapshot / "all-results.csv"),
        physical_intervals_checked=intervals,
        exact_geometry_events=len(events),
        terminal_geometry_checkpoints=len(records),
        short_reentry_max_lag_s=1,
        short_reentry_max_travel_exclusive=0.1,
        tail_start_exclusive_s=150,
        tail_end_inclusive_s=180,
        matched_horizon="joint successful termination",
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, rows in [
        ("lost-holds.csv", records),
        ("guard-events.csv", events),
        ("release-bouts.csv", bouts),
    ]:
        save_rows(args.output / name, rows)
    (args.output / "sources.json").write_text(json.dumps(evidence, indent=2) + "\n")
    if args.plot:
        plot(args.output, records)
    for r in records:
        print(r)
    print(
        f"Checked {intervals} physical intervals, {len(events)} exact geometry events, {len(bouts)} release bouts"
    )


if __name__ == "__main__":
    main()
