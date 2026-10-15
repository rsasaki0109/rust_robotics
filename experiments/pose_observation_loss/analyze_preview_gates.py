#!/usr/bin/env python3
"""Audit terminal-preview release gates on recorded new-seed trajectories."""

import argparse
import csv
import json
from pathlib import Path

from analyze_entry_cycles import physical_intervals, release_bouts, wait_seconds
from analyze_filter_stalls import conditions, pending
from analyze_recovery_stalls import digest
from summarize_stopping_preview import save_rows
from verify_joint_recovery import check_guard_track, check_joint_track
from verify_motion_filter import check_motion_filter_track
from verify_stopping_preview import preview_components

ROOT = Path(__file__).resolve().parent
KEY = ["cohort", "mode", "seed", "scenario", "planner"]
DT = 0.02
STATES = ["other_blocked", "both_blocked", "stop_only", "terminal_only", "ready"]


def gate(before, row):
    """Use pre-command guard/loss state, including the actual release interval."""
    if pending(before, row):
        route = "joint"
        other = conditions(before, row)
        del other["guard_preview"]
        other_ready = all(other.values())
    elif before["guard_active"] == "true":
        route = "independent"
        other_ready = abs(float(before["v"])) <= 0.02
    else:
        return None
    margin = float(row["guard_release_margin"])
    stop = float(row["stop_preview_clearance_lower_bound"]) > margin
    terminal = float(row["terminal_recovery_clearance"]) > margin + 0.1
    state = (
        "other_blocked"
        if not other_ready
        else (
            "ready"
            if stop and terminal
            else (
                "terminal_only" if stop else "stop_only" if terminal else "both_blocked"
            )
        )
    )
    return route, state


def episodes(track):
    """Maximal contiguous intervals on one release route; no simulated replay."""
    result = []
    active = None
    old_run = new_run = 0

    def close():
        if active is not None:
            result.append(active)

    for before, row in zip(track, track[1:]):
        value = gate(before, row)
        if value is None:
            close()
            active = None
            old_run = new_run = 0
            continue
        route, state = value
        if active is None or active["route"] != route:
            close()
            active = dict(
                route=route,
                start_s=round(float(row["time_s"]) - DT, 6),
                end_s=0,
                intervals=0,
                max_old_ready_s=0,
                max_new_ready_s=0,
                actual_releases=0,
                **{k + "_s": 0 for k in STATES},
            )
            old_run = new_run = 0
        active["end_s"] = float(row["time_s"])
        active["intervals"] += 1
        active[state + "_s"] = round(active[state + "_s"] + DT, 6)
        old_run = old_run + 1 if state in ["ready", "terminal_only"] else 0
        new_run = new_run + 1 if state == "ready" else 0
        active["max_old_ready_s"] = round(
            max(active["max_old_ready_s"], old_run * DT), 6
        )
        active["max_new_ready_s"] = round(
            max(active["max_new_ready_s"], new_run * DT), 6
        )
        active["actual_releases"] += int(
            row["resumed" if route == "joint" else "guard_released"] == "true"
        )
    close()
    for item in result:
        threshold = 1 if item["route"] == "joint" else 0.5
        item["old_window_reached"] = item["max_old_ready_s"] >= threshold - 1e-8
        item["new_window_reached"] = item["max_new_ready_s"] >= threshold - 1e-8
        assert item["new_window_reached"] == (item["actual_releases"] == 1)
        assert item["actual_releases"] <= 1
        assert abs(sum(item[k + "_s"] for k in STATES) - item["intervals"] * DT) < 1e-7
    return result


def checkpoint(case, before, row, obstacles):
    stop, terminal, steps = preview_components(before, row, obstacles)
    assert abs(terminal - float(row["terminal_recovery_clearance"])) < 1e-8
    for margin in [float(row["preview_margin"]), float(row["guard_release_margin"])]:
        logged = float(row["stop_preview_clearance_lower_bound"])
        if abs(stop - margin) > 2e-8 and abs(logged - margin) > 2e-8:
            assert (stop > margin) == (logged > margin)
    return dict(
        **{k: case[k] for k in KEY},
        time_s=row["time_s"],
        stop_exact=stop,
        terminal_exact=terminal,
        preview_steps=steps,
        entry_margin=row["preview_margin"],
        release_margin=row["guard_release_margin"],
    )


def short_record(case, track, bout):
    start, end = (track[bout[k]] for k in ["release_index", "entry_index"])
    result = dict(**{k: case[k] for k in KEY}, **bout)
    for name, row in [("release", start), ("entry", end)]:
        margin = float(row["preview_margin"])
        result.update(
            {
                name
                + "_stop_entry_slack": float(row["stop_preview_clearance_lower_bound"])
                - margin,
                name
                + "_terminal_entry_slack": float(row["terminal_recovery_clearance"])
                - margin
                - 0.1,
            }
        )
    result["terminal_clearance_change"] = float(
        end["terminal_recovery_clearance"]
    ) - float(start["terminal_recovery_clearance"])
    result["margin_change"] = float(end["preview_margin"]) - float(
        start["preview_margin"]
    )
    result["terminal_only_entry"] = result["entry_stop_entry_slack"] > 0
    assert result["release_terminal_entry_slack"] > 0.05 - 2e-8
    assert (
        min(result["entry_stop_entry_slack"], result["entry_terminal_entry_slack"])
        <= 2e-8
    )
    assert (
        abs(
            result["entry_terminal_entry_slack"]
            - result["release_terminal_entry_slack"]
            - result["terminal_clearance_change"]
            + result["margin_change"]
        )
        < 1e-8
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--white", type=Path, required=True)
    parser.add_argument("--correlated", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    snapshot = ROOT / "results/recovery-preview"
    pinned = list(csv.DictReader((snapshot / "all-results.csv").open()))
    sources = json.loads((snapshot / "sources.json").read_text())
    lookup = {tuple(r[k] for k in ["variant"] + KEY): r for r in pinned}
    records, gates, shorts, checks, lost_pairs = [], [], [], [], []
    evidence, intervals = {}, 0
    for cohort, run in [
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
        ]["recoverable"]
        maps = {}
        for row in csv.DictReader((inputs / "obstacles.csv").open()):
            maps.setdefault(row["scenario"], []).append(
                (float(row["x"]), float(row["y"]))
            )
        selected = [
            r for r in pinned if r["cohort"] == cohort and r["variant"] == "recoverable"
        ]
        hashes = {}
        for mode, seed in sorted({(r["mode"], r["seed"]) for r in selected}):
            cases = [r for r in selected if (r["mode"], r["seed"]) == (mode, seed)]
            tracks = {}
            for variant in ["recoverable", "filtered"]:
                wanted = {
                    (r["scenario"], r["planner"])
                    for r in cases
                    if variant == "recoverable"
                    or (
                        r["held"] != "true"
                        and lookup[(variant, *(r[k] for k in KEY))]["held"] == "true"
                    )
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
                key = {k: case[k] for k in KEY}
                track = tracks["recoverable"].pop((case["scenario"], case["planner"]))
                assert float(track[-1]["time_s"]) == float(case["time_s"])
                check_guard_track(track)
                check_joint_track(track)
                check_motion_filter_track(track, True)
                travel = physical_intervals(track)
                intervals += len(track) - 1
                assert all(r["recovery_preview_active"] == "true" for r in track[1:])
                for row in track[1:]:
                    assert (
                        abs(
                            float(row["preview_clearance_lower_bound"])
                            - min(
                                float(row["stop_preview_clearance_lower_bound"]),
                                float(row["terminal_recovery_clearance"]) - 0.1,
                            )
                        )
                        < 2e-9
                    )
                case_gates = episodes(track)
                gates.extend(dict(**key, **g) for g in case_gates)
                bouts = [b for b in release_bouts(track, travel) if b["short_reentry"]]
                shorts.extend(short_record(case, track, b) for b in bouts)
                indices = {
                    b[field]
                    for b in bouts
                    for field in ["release_index", "entry_index"]
                }
                checks.extend(
                    checkpoint(case, track[i - 1], track[i], maps[case["scenario"]])
                    for i in sorted(indices)
                )
                baseline = lookup[("filtered", *(case[k] for k in KEY))]
                lost = baseline["held"] == "true" and case["held"] != "true"
                entries = [r for r in track[1:] if r["guard_triggered"] == "true"]
                records.append(
                    dict(
                        **key,
                        status=case["status"],
                        lost_hold=lost,
                        guard_entries=len(entries),
                        terminal_only_entries=sum(
                            float(r["stop_preview_clearance_lower_bound"])
                            > float(r["preview_margin"])
                            for r in entries
                        ),
                        short_reentries=len(bouts),
                        wait_s=wait_seconds(track, float(case["time_s"])),
                        release_route_s=round(
                            sum(g["intervals"] for g in case_gates) * DT, 6
                        ),
                        terminal_only_block_s=round(
                            sum(g["terminal_only_s"] for g in case_gates), 6
                        ),
                    )
                )
                if lost:
                    ref = tracks["filtered"].pop((case["scenario"], case["planner"]))
                    check_guard_track(ref)
                    check_joint_track(ref)
                    physical_intervals(ref)
                    intervals += len(ref) - 1
                    horizon = float(baseline["time_s"])
                    assert float(ref[-1]["time_s"]) == horizon
                    lost_pairs.append(
                        dict(
                            **key,
                            filtered_completion_s=horizon,
                            filtered_matched_wait_s=wait_seconds(ref, horizon),
                            recoverable_matched_wait_s=wait_seconds(track, horizon),
                            final_goal_distance=case["goal_distance"],
                            first_goal_entry_s=case["first_goal_entry_s"],
                        )
                    )
        evidence[cohort] = dict(
            provenance=provenance,
            raw_sha256=hashes,
            summary_sha256=digest(run / "all-results.csv"),
        )
        print(f"Checked {cohort}", flush=True)
    assert len(records) == 162 and len(lost_pairs) == 34 and len(shorts) == 237
    assert sum(r["terminal_only_entries"] for r in records) == 633
    summary = []
    for cohort in ["white-validation", "correlated-validation"]:
        for route in ["independent", "joint"]:
            group = [g for g in gates if g["cohort"] == cohort and g["route"] == route]
            summary.append(
                dict(
                    cohort=cohort,
                    route=route,
                    episodes=len(group),
                    old_window_reached=sum(g["old_window_reached"] for g in group),
                    new_window_reached=sum(g["new_window_reached"] for g in group),
                    old_window_without_new=sum(
                        g["old_window_reached"] and not g["new_window_reached"]
                        for g in group
                    ),
                    **{
                        k + "_s": round(sum(g[k + "_s"] for g in group), 6)
                        for k in STATES
                    },
                )
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
        geometry_checkpoints=len(checks),
        selection="all 162 new-seed recoverable tracks and filtered counterparts for 34 lost holds",
        terminal_extra=0.1,
        note="Readiness on fixed recorded inputs; removing a gate changes later inputs and is not replayed.",
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, data in [
        ("cases.csv", records),
        ("episodes.csv", gates),
        ("short-reentries.csv", shorts),
        ("checkpoints.csv", checks),
        ("lost-pairs.csv", lost_pairs),
        ("summary.csv", summary),
    ]:
        save_rows(args.output / name, data)
    (args.output / "sources.json").write_text(json.dumps(evidence, indent=2) + "\n")
    for row in summary:
        print(row)
    print(
        f"Checked {intervals} physical intervals and {len(checks)} geometry checkpoints"
    )


if __name__ == "__main__":
    main()
