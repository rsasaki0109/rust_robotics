#!/usr/bin/env python3
"""Audit forward arcs at 81 pinned aligned-release stalls; no controller replay."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import tempfile

from analyze_entry_cycles import physical_intervals
from analyze_recovery_arcs import (
    ANGLES,
    GRID_ERROR_BOUND,
    chord_check,
    inspect_pose,
    status,
)
from analyze_recovery_stalls import digest
from summarize_stopping_preview import save_rows
from verify_joint_recovery import check_guard_track, check_joint_track
from verify_motion_filter import check_motion_filter_track

ROOT = Path(__file__).resolve().parent
KEY = ["cohort", "mode", "seed", "scenario", "planner"]
KINDS = ["truth", "filtered"]


def select_cases(rows):
    return [
        r
        for r in rows
        if r["cohort"] in ["white-validation", "correlated-validation"]
        and r["classification"] == "no_entry_stationary"
    ]


def selected_pairs(track):
    """Both poses describe the same pre-command instant, not the row's endpoint."""
    for before, row in zip(track, track[1:]):
        t = float(row["time_s"])
        if t <= 150:
            continue
        fresh = (
            abs(t % 5 - 0.5) < 1e-8
            and abs(float(row["observation_age"]) - 0.5) < 1e-8
            and abs(float(before["v"])) <= 0.02
        )
        if t == 180 or fresh:
            assert abs(t - float(before["time_s"]) - 0.02) < 1e-8
            poses = [
                ("truth", tuple(float(before[k]) for k in ["x", "y", "yaw"])),
                (
                    "filtered",
                    tuple(float(row[f"preview_pose_{k}"]) for k in ["x", "y", "yaw"]),
                ),
            ]
            yield before, row, "final" if t == 180 else "fresh_stationary", poses


def paired_row(records, truth_grid=None):
    assert {r["pose_kind"] for r in records} == set(KINDS) and len(records) == 2
    truth, filtered = (
        next(r for r in records if r["pose_kind"] == kind) for kind in KINDS
    )
    result = {k: truth[k] for k in [*KEY, "time_s", "decision_time_s", "reason"]}
    result.update(
        position_error=math.hypot(
            truth["x"] - filtered["x"], truth["y"] - filtered["y"]
        ),
        yaw_error_rad=abs(
            math.atan2(
                math.sin(truth["yaw"] - filtered["yaw"]),
                math.cos(truth["yaw"] - filtered["yaw"]),
            )
        ),
        motion_filter_sample_count=truth["motion_filter_sample_count"],
    )
    for suffix in ["zero_age", "current_age"]:
        for kind, row in [("truth", truth), ("filtered", filtered)]:
            result[f"{kind}_{suffix}_status"] = row[f"recovery_{suffix}_status"]
            result[f"{kind}_{suffix}_best_slack"] = (
                row["grid_max_clearance"] - row[f"recovery_{suffix}_margin"]
            )
        result[f"filtered_pass_truth_blocked_{suffix}"] = (
            filtered[f"recovery_{suffix}_status"] == "grid_pass"
            and truth[f"recovery_{suffix}_status"] == "all_fixed_steering_blocked"
        )
    if truth_grid is not None:
        angle = filtered["best_steering_deg"]
        actual = truth_grid[angle]
        result.update(
            filtered_best_steering_deg=angle, true_clearance_at_filtered_best=actual
        )
        for suffix in ["zero_age", "current_age"]:
            result[f"filtered_selected_passes_{suffix}"] = (
                filtered["best_clearance"]
                > filtered[f"recovery_{suffix}_margin"] + 1e-9
            )
            result[f"true_selected_passes_{suffix}"] = (
                actual > truth[f"recovery_{suffix}_margin"] + 1e-9
            )
    return result


def summarize(records):
    rows = []
    for reason in ["final", "fresh_stationary"]:
        for kind in KINDS:
            group = [
                r for r in records if r["reason"] == reason and r["pose_kind"] == kind
            ]
            if not group:
                continue
            for age in ["zero_age", "current_age"]:
                rows.append(
                    dict(
                        reason=reason,
                        pose_kind=kind,
                        margin_age=age,
                        checkpoints=len(group),
                        start_blocked=sum(
                            r["center_clearance"] <= r[f"recovery_{age}_margin"]
                            for r in group
                        ),
                        straight_passes=sum(
                            r["straight_clearance"] > r[f"recovery_{age}_margin"] + 1e-9
                            for r in group
                        ),
                        physical_grid_passes=sum(
                            r["physical_status"] == "grid_pass" for r in group
                        ),
                        buffer_grid_passes=sum(
                            r["buffer_status"] == "grid_pass" for r in group
                        ),
                        **{
                            state: sum(
                                r[f"recovery_{age}_status"] == state for r in group
                            )
                            for state in [
                                "grid_pass",
                                "all_fixed_steering_blocked",
                                "unresolved",
                            ]
                        },
                    )
                )
    return rows


def plot(output, pairs):
    with tempfile.TemporaryDirectory(prefix="aligned-arc-plot-") as cache:
        os.environ.setdefault("MPLCONFIGDIR", cache)
        os.environ.setdefault("XDG_CACHE_HOME", cache)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.rcParams.update({"svg.fonttype": "none", "svg.hashsalt": "aligned-arcs"})
        fig, axes = plt.subplots(
            1, 2, figsize=(11, 5), sharex=True, sharey=True, layout="constrained"
        )
        for ax, reason, title in zip(
            axes,
            ["final", "fresh_stationary"],
            ["Final command", "Fresh stationary checkpoints"],
        ):
            for cohort, color, label in [
                ("white-validation", "#2563eb", "White noise"),
                ("correlated-validation", "#d97706", "1 s correlation"),
            ]:
                group = [
                    r for r in pairs if r["reason"] == reason and r["cohort"] == cohort
                ]
                ax.scatter(
                    [r["truth_zero_age_best_slack"] for r in group],
                    [r["filtered_zero_age_best_slack"] for r in group],
                    color=color,
                    label=label,
                    s=18,
                    alpha=0.65,
                )
            ax.axhline(0, color="#475569", linewidth=1)
            ax.axvline(0, color="#475569", linewidth=1)
            ax.set(
                title=title,
                xlabel="True-pose best grid clearance minus age-0 margin",
                ylabel="Filtered-pose best grid clearance minus age-0 margin",
            )
        axes[0].legend(fontsize=8)
        fig.suptitle(
            "Forward 0.6 arcs at 81 frozen timeouts\nOffline geometry; positive slack is a grid witness, not a recovery result"
        )
        svg = output / "aligned-arcs.svg"
        fig.savefig(svg, metadata={"Date": None})
        svg.write_text(
            "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(output / "aligned-arcs.png", dpi=160)
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
    selection = ROOT / "results/aligned-stalls/timeout-cases.csv"
    pinned_sources = json.loads(
        (ROOT / "results/aligned-stalls/sources.json").read_text()
    )
    cases = select_cases(list(csv.DictReader(selection.open())))
    assert len(cases) == 81
    records, curves, audits, pairs, evidence = [], [], [], [], {}
    intervals = 0
    for cohort, run in [
        ("white-validation", args.white),
        ("correlated-validation", args.correlated),
    ]:
        source = pinned_sources[cohort]
        provenance = json.loads((run / "provenance.json").read_text())
        assert provenance == source["provenance"]
        assert digest(run / "all-results.csv") == source["summary_sha256"]
        inputs = run / "planning-inputs/fine-padding"
        assert {p.name: digest(p) for p in inputs.glob("*.csv")} == provenance[
            "inputs_by_variant_sha256"
        ]["aligned"]
        maps = {}
        for row in csv.DictReader((inputs / "obstacles.csv").open()):
            maps.setdefault(row["scenario"], []).append(
                (float(row["x"]), float(row["y"]))
            )
        subset = [r for r in cases if r["cohort"] == cohort]
        hashes = {}
        for mode, seed in sorted({(r["mode"], r["seed"]) for r in subset}):
            selected = [r for r in subset if (r["mode"], r["seed"]) == (mode, seed)]
            path = run / f"aligned-{mode}-{seed}/trajectories.csv"
            relative = str(path.relative_to(run))
            hashes[relative] = digest(path)
            assert hashes[relative] == source["raw_sha256"][relative]
            tracks = {(r["scenario"], r["planner"]): [] for r in selected}
            with path.open() as stream:
                for row in csv.DictReader(stream):
                    key = row["scenario"], row["planner"]
                    if key in tracks:
                        tracks[key].append(row)
            for case in selected:
                track = tracks.pop((case["scenario"], case["planner"]))
                assert float(track[-1]["time_s"]) == 180
                check_motion_filter_track(track, True)
                check_guard_track(track)
                check_joint_track(track)
                travel = physical_intervals(track)
                intervals += len(track) - 1
                assert (
                    abs(travel[-1] - travel[-1501] - float(case["tail_travel"])) < 1e-9
                )
                assert travel[-1] - travel[-1501] < 0.1
                for before, row, reason, poses in selected_pairs(track):
                    identity = dict(
                        **{k: case[k] for k in KEY},
                        time_s=row["time_s"],
                        decision_time_s=before["time_s"],
                        reason=reason,
                    )
                    group = []
                    truth_grid = None
                    for kind, pose in poses:
                        record, grid, checked = inspect_pose(
                            identity, pose, row, kind, maps[case["scenario"]]
                        )
                        record.update(
                            motion_filter_sample_count=int(
                                row["motion_filter_sample_count"]
                            ),
                            precommand_speed=before["v"],
                            physical_status=status(
                                record["grid_max_clearance"],
                                record["continuous_clearance_upper"],
                                0.5,
                            ),
                            buffer_status=status(
                                record["grid_max_clearance"],
                                record["continuous_clearance_upper"],
                                0.6,
                            ),
                        )
                        records.append(record)
                        group.append(record)
                        curves.extend(grid)
                        for audit in checked:
                            audit["role"] = "original"
                        audits.extend(checked)
                        if kind == "truth":
                            truth_grid = {
                                r["steering_deg"]: r["clearance"] for r in grid
                            }
                    pair = paired_row(group, truth_grid)
                    pairs.append(pair)
                    angle = math.radians(pair["filtered_best_steering_deg"])
                    exact = pair["true_clearance_at_filtered_best"]
                    error, bound = chord_check(
                        dict(poses)["truth"], angle, maps[case["scenario"]], exact
                    )
                    audits.append(
                        dict(
                            **identity,
                            pose_kind="truth",
                            steering_deg=pair["filtered_best_steering_deg"],
                            exact_clearance=exact,
                            chord_error=error,
                            sagitta_bound=bound,
                            role="filtered_best_on_truth",
                        )
                    )
        evidence[cohort] = dict(provenance=provenance, raw_sha256=hashes)
        print(f"Checked {cohort}", flush=True)
    assert sum(r["reason"] == "final" for r in pairs) == 81
    summary = summarize(records)
    evidence["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        selection_sha256=digest(selection),
        pinned_sources_sha256=digest(ROOT / "results/aligned-stalls/sources.json"),
        helpers_sha256={
            name: digest(ROOT / name)
            for name in [
                "analyze_recovery_arcs.py",
                "analyze_recovery_stalls.py",
                "analyze_entry_cycles.py",
                "verify_joint_recovery.py",
                "verify_motion_filter.py",
                "verify_stopping_preview.py",
                "summarize_stopping_preview.py",
            ]
        },
        physical_intervals_checked=intervals,
        geometry_audits=len(audits),
        poses=len(records),
        grid_arcs=len(curves),
        forward_length=0.6,
        wheelbase=0.4,
        steering_grid_degrees=ANGLES,
        continuous_grid_error_bound=GRID_ERROR_BOUND,
        chord_intervals=120,
        selection="81 new no-entry aligned timeouts with tail travel <0.1",
        note="Both poses at pre-command time. Spatial constant-steering arcs; no dynamics, timers or controller replay.",
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, data in [
        ("checkpoints.csv", records),
        ("steering-grid.csv", curves),
        ("geometry-audit.csv", audits),
        ("paired-poses.csv", pairs),
        ("summary.csv", summary),
    ]:
        save_rows(args.output / name, data)
    (args.output / "sources.json").write_text(json.dumps(evidence, indent=2) + "\n")
    if args.plot:
        plot(args.output, pairs)
    for row in summary:
        print(row)
    print(
        f"Checked {len(records)} poses, {len(curves)} arcs and {len(audits)} chord audits"
    )


if __name__ == "__main__":
    main()
