#!/usr/bin/env python3
"""Aggregate and independently audit the fixed live terminal-arc comparison."""

import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
import statistics

from analyze_epoch_preview import (
    CollisionAudit,
    compare_disabled,
    diagnostics,
    digest,
    paired,
    read_rows,
)
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
SIMULATOR = (
    ROOT.parents[1] / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
)
COHORTS = ["known-white", "known-correlated", "new-white", "new-correlated"]
VARIANTS = ["epoch", "curved"]
KEY = ["mode", "seed", "scenario", "planner"]
PRIOR_SHA = "ceea11fd0eaaa4b992140caf2e8a9a62233ef4b38202ec0c33b6970e41362c28"


def load_tracks(path):
    groups = defaultdict(list)
    for row in read_rows(path):
        assert None not in row and None not in row.values(), "CSV column count"
        groups[row["scenario"], row["planner"]].append(row)
    assert len(groups) == 9
    return groups


def shared_prefix(reference, candidate):
    """Verify equal physical/sensed inputs through the first different command."""
    inputs = [
        "time_s",
        "measured_speed",
        "nominal_target_speed",
        "nominal_steering",
        "preview_pose_x",
        "preview_pose_y",
        "preview_pose_yaw",
        "preview_margin",
        "guard_release_margin",
        "correction",
        "correction_received",
        "precision_active",
        "effective_position_sigma",
        "effective_yaw_sigma_rad",
        "loss_uniform",
        "observation_age",
        "speed_free_distance",
        "speed_cap",
        "speed_margin",
    ] + [
        f"{kind}_{axis}"
        for kind in ["noise", "bias", "noise_innovation"]
        for axis in ["x", "y", "yaw"]
    ]
    count = 0
    for a0, a, b0, b in zip(reference, reference[1:], candidate, candidate[1:]):
        assert all(a0[k] == b0[k] for k in ["x", "y", "yaw", "v"]), "physical prefix"
        assert all(a[k] == b[k] for k in inputs), "sensed/nominal prefix"
        count += 1
        if any(a[k] != b[k] for k in ["target_speed", "steering", "acceleration"]):
            return count
    assert len(reference) == len(candidate), "unchanged command length"
    return count


def summarize(cases, pairs):
    result = []
    for cohort in [*COHORTS, "known", "new", "all"]:

        def select(row):
            return (
                cohort == "all"
                or row["cohort"] == cohort
                or row["cohort"].startswith(cohort + "-")
            )

        for mode in ["all", "none", "random", "blackout"]:
            current_pairs = [
                r for r in pairs if select(r) and (mode == "all" or r["mode"] == mode)
            ]
            held_pairs = [r for r in current_pairs if r["both_held"]]
            for variant in VARIANTS:
                rows = [
                    r
                    for r in cases
                    if select(r)
                    and r["variant"] == variant
                    and (mode == "all" or r["mode"] == mode)
                ]
                if not rows:
                    continue
                result.append(
                    dict(
                        cohort=cohort,
                        mode=mode,
                        variant=variant,
                        cases=len(rows),
                        held=sum(r["held"] == "true" for r in rows),
                        timeout=sum(r["status"] == "timeout" for r in rows),
                        collision=sum(r["collision"] == "true" for r in rows),
                        out_of_bounds=sum(r["status"] == "out_of_bounds" for r in rows),
                        mean_wait_s=statistics.mean(r["wait_time_s"] for r in rows),
                        mean_stationary_wait_s=statistics.mean(
                            float(r["stationary_wait_time_s"]) for r in rows
                        ),
                        short_reentries=sum(int(r["short_reentries"]) for r in rows),
                        mean_goal_distance=statistics.mean(
                            float(r["goal_distance"]) for r in rows
                        ),
                        gained_holds=sum(r["gained_hold"] for r in current_pairs),
                        lost_holds=sum(r["lost_hold"] for r in current_pairs),
                        changed_command_pairs=sum(
                            bool(r["first_command_change_s"]) for r in current_pairs
                        ),
                        mean_matched_wait_delta_s=statistics.mean(
                            r["matched_wait_difference_s"] for r in current_pairs
                        ),
                        both_held_pairs=len(held_pairs),
                        mean_both_held_time_delta_s=(
                            statistics.mean(
                                r["held_time_difference_s"] for r in held_pairs
                            )
                            if held_pairs
                            else ""
                        ),
                    )
                )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    if args.output.exists():
        parser.error("Output directory must be new")
    prior_path = ROOT / "results/epoch-preview/sources.json"
    assert digest(prior_path) == PRIOR_SHA
    prior = json.loads(prior_path.read_text())
    expected_inputs = {
        p.name: digest(p)
        for p in (ROOT / "results/planning-clearance/fixtures/fine-padding").glob(
            "*.csv"
        )
    }
    maps = defaultdict(list)
    for row in read_rows(ROOT / "inputs/obstacles.csv"):
        maps[row["scenario"]].append((float(row["x"]), float(row["y"])))
    assert digest(ROOT / "inputs/obstacles.csv") == expected_inputs["obstacles.csv"]
    audits = {name: CollisionAudit(points) for name, points in maps.items()}
    cases, pairs, verification = [], [], []
    sources = {"prior_sources_sha256": PRIOR_SHA, "cohorts": {}}
    for cohort in COHORTS:
        run = args.raw_root.resolve() / cohort
        provenance = json.loads((run / "provenance.json").read_text())
        seeds = [269, 270, 271] if cohort.startswith("known-") else [274, 275, 276]
        assert provenance["variants"] == VARIANTS
        assert provenance["seeds"] == seeds
        assert provenance["noise_correlation_time_s"] == (
            1 if "correlated" in cohort else 0
        )
        assert provenance["simulator_sha256"] == digest(SIMULATOR)
        assert provenance["inputs_sha256"] == expected_inputs
        assert all(
            provenance["inputs_by_variant_sha256"][v] == expected_inputs
            for v in VARIANTS
        )
        results = read_rows(run / "all-results.csv")
        events = {
            tuple(r[k] for k in ["variant", *KEY]): r
            for r in read_rows(run / "control-events.csv")
        }
        preview_audits = read_rows(run / "preview-audit.csv")
        assert len(results) == len(events) == 162
        assert preview_audits and all(
            float(r["terminal_chord_error"])
            <= float(r["terminal_sagitta_bound"]) + 1e-9
            for r in preview_audits
        )
        raw_hashes, old_hashes = {}, {}
        intervals = regress_rows = fusion_intervals = prefix_intervals = 0
        for mode in ["none", "random", "blackout"]:
            for seed in seeds:
                groups = []
                for variant in VARIANTS:
                    trial = run / f"{variant}-{mode}-{seed}"
                    raw_hashes[f"{trial.name}/trajectories.csv"] = digest(
                        trial / "trajectories.csv"
                    )
                    if cohort.startswith("known-") and variant == "epoch":
                        old_cohort = ("known-" if seed <= 270 else "new-") + (
                            "correlated" if "correlated" in cohort else "white"
                        )
                        old = prior[old_cohort]
                        reference = Path(old["path"]) / trial.name
                        assert (
                            old["provenance"]["simulator_sha256"]
                            == "4e42e38a0f28730ab41f4c7d021b6824fea920057bfb0cc2a7baae3903d1153b"
                        )
                        assert old["provenance"]["inputs_sha256"] == expected_inputs
                        expected_hash = old["raw_sha256"][
                            f"{trial.name}/trajectories.csv"
                        ]
                        assert digest(reference / "trajectories.csv") == expected_hash
                        old_hashes[str(reference / "trajectories.csv")] = expected_hash
                        assert (reference / "results.csv").read_bytes() == (
                            trial / "results.csv"
                        ).read_bytes()
                        regress_rows += compare_disabled(
                            trial / "trajectories.csv",
                            reference / "trajectories.csv",
                            {"terminal_steering_preview"},
                        )
                    groups.append(load_tracks(trial / "trajectories.csv"))
                for key in sorted(groups[0]):
                    current, prefixes = [], []
                    for variant, tracks in zip(VARIANTS, groups):
                        track = tracks[key]
                        row = next(
                            r
                            for r in results
                            if r["variant"] == variant
                            and r["mode"] == mode
                            and int(r["seed"]) == seed
                            and (r["scenario"], r["planner"]) == key
                        )
                        diagnostic, prefix = diagnostics(track)
                        contacts, boundaries = audits[key[0]].check(track)
                        assert bool(contacts) == (row["collision"] == "true")
                        assert bool(boundaries) == (row["status"] == "out_of_bounds")
                        assert track[0]["terminal_steering_preview"] == "false"
                        assert all(
                            (r["terminal_steering_preview"] == "true")
                            == (variant == "curved")
                            for r in track[1:]
                        )
                        event = events[tuple(row[k] for k in ["variant", *KEY])]
                        assert (
                            abs(
                                diagnostic["wait_time_s"]
                                - float(event["total_wait_time_s"])
                            )
                            < 1e-8
                        )
                        record = dict(
                            cohort=cohort,
                            **row,
                            **diagnostic,
                            swept_contact_intervals=contacts,
                            out_of_bounds_intervals=boundaries,
                            stationary_wait_time_s=event["stationary_wait_time_s"],
                            short_reentries=event["short_reentries"],
                        )
                        cases.append(record)
                        current.append(record)
                        prefixes.append(prefix)
                        intervals += len(track) - 1
                        fusion_intervals += len(track) - 1
                    prefix_intervals += shared_prefix(groups[0][key], groups[1][key])
                    pairs.append(paired(*current, prefixes, [g[key] for g in groups]))
                del groups
        sources["cohorts"][cohort] = dict(
            path=str(run),
            provenance=provenance,
            raw_sha256=raw_hashes,
            prior_baseline_sha256=old_hashes,
            artifacts_sha256={
                name: digest(run / name)
                for name in [
                    "provenance.json",
                    "manifest.json",
                    "all-results.csv",
                    "control-events.csv",
                    "preview-audit.csv",
                ]
            },
        )
        verification.append(
            dict(
                cohort=cohort,
                cases=len(results),
                physical_intervals=intervals,
                fusion_reconstruction_intervals=fusion_intervals,
                shared_prefix_intervals=prefix_intervals,
                exact_preview_audits=len(preview_audits),
                terminal_chord_audits=len(preview_audits),
                disabled_reference_cases=81 if cohort.startswith("known-") else 0,
                disabled_reference_rows=regress_rows,
            )
        )
        print(
            f"{cohort}: {intervals} intervals, {regress_rows} baseline rows", flush=True
        )
    assert len(cases) == 648 and len(pairs) == 324
    sources["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        simulator_sha256=digest(SIMULATOR),
        protocol_sha256=digest(ROOT / "results/terminal-arc-live/protocol.md"),
        helpers_sha256={
            name: digest(ROOT / name)
            for name in [
                "run.py",
                "verify.py",
                "verify_motion_filter.py",
                "verify_joint_recovery.py",
                "verify_stopping_preview.py",
                "analyze_epoch_preview.py",
                "analyze_entry_cycles.py",
                "analyze_recovery_arcs.py",
                "analyze_terminal_steering_preview.py",
                "summarize_stopping_preview.py",
            ]
        },
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("cases", cases),
        ("pairs", pairs),
        ("summary", summarize(cases, pairs)),
        ("verification", verification),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")


if __name__ == "__main__":
    main()
