#!/usr/bin/env python3
"""Summarize frozen live epoch-preview runs and audit disabled-mode regression."""

import argparse
import csv
import hashlib
from itertools import zip_longest
import json
import math
from pathlib import Path
import statistics

from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
SIMULATOR = (
    ROOT.parents[1] / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
)
SIMULATOR_SHA = "4e42e38a0f28730ab41f4c7d021b6824fea920057bfb0cc2a7baae3903d1153b"
INITIAL_SIMULATOR_SHA = (
    "f46fabf0fa526c252636574e4f0a56add10eebfc06515ccfae246775f12b78f1"
)
PROTOTYPE_SHA = "1d730cb485768c564ae8a79600a3121cef5609eda95bd3435b18c0ef3e89b3b9"
COHORTS = ["known-white", "known-correlated", "new-white", "new-correlated"]
VARIANTS = ["aligned", "epoch"]
KEY = ["mode", "seed", "scenario", "planner"]
NEW_COLUMNS = {
    "stationary_epoch_fusion",
    "epoch_fusion_available",
    "epoch_fusion_group_count",
    "motion_filter_pose_x",
    "motion_filter_pose_y",
    "measured_speed",
}


def digest(path):
    hasher = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def compare_disabled(current, reference, added_columns=NEW_COLUMNS):
    """Compare every old field, including initial rows and trajectory lengths."""
    count = 0
    with current.open() as new, reference.open() as old:
        actual, expected = csv.DictReader(new), csv.DictReader(old)
        assert set(actual.fieldnames) - set(expected.fieldnames) == added_columns
        assert set(expected.fieldnames) <= set(actual.fieldnames)
        for row, prior in zip_longest(actual, expected):
            assert row is not None and prior is not None, "disabled trajectory length"
            assert all(row[k] == v for k, v in prior.items()), (
                current.name,
                count,
                "disabled field regression",
            )
            count += 1
    return count


def diagnostics(track):
    prefix = [0]
    available = []
    base_squared = fused_squared = 0.0
    better = worse = episodes = 0
    was_available = False
    for before, row in zip(track, track[1:]):
        prefix.append(
            prefix[-1]
            + (row["guard_active"] == "true" or row["safety_paused"] == "true")
        )
        active = row["epoch_fusion_available"] == "true"
        episodes += active and not was_available
        was_available = active
        if not active:
            assert all(
                row[f"preview_pose_{a}"] == row[f"motion_filter_pose_{a}"]
                for a in ["x", "y"]
            )
            continue
        available.append(row)
        base = math.hypot(
            *(
                float(row[f"motion_filter_pose_{a}"]) - float(before[a])
                for a in ["x", "y"]
            )
        )
        fused = math.hypot(
            *(float(row[f"preview_pose_{a}"]) - float(before[a]) for a in ["x", "y"])
        )
        base_squared += base * base
        fused_squared += fused * fused
        better += fused < base - 1e-12
        worse += fused > base + 1e-12
    result = dict(
        intervals=len(track) - 1,
        wait_time_s=round(prefix[-1] * 0.02, 6),
        fusion_available_steps=len(available),
        fusion_available_s=round(len(available) * 0.02, 6),
        fusion_episodes=episodes,
        fusion_first_s=available[0]["time_s"] if available else "",
        fusion_last_s=available[-1]["time_s"] if available else "",
        available_guard_entries=sum(r["guard_triggered"] == "true" for r in available),
        available_guard_releases=sum(r["guard_released"] == "true" for r in available),
        available_resumes=sum(r["resumed"] == "true" for r in available),
        base_position_squared_sum=base_squared,
        fused_position_squared_sum=fused_squared,
        base_position_rms=math.sqrt(base_squared / len(available)) if available else "",
        fused_position_rms=(
            math.sqrt(fused_squared / len(available)) if available else ""
        ),
        position_better_steps=better,
        position_worse_steps=worse,
        final_stationary=abs(float(track[-1]["v"])) <= 0.02,
    )
    return result, prefix


def paired(reference, candidate, prefixes, tracks):
    horizon = min(float(r["time_s"]) for r in [reference, candidate])
    end = round(horizon / 0.02)
    assert abs(end * 0.02 - horizon) < 1e-8
    change = next(
        (
            b["time_s"]
            for a, b in zip(tracks[0], tracks[1])
            if any(a[k] != b[k] for k in ["target_speed", "steering", "acceleration"])
        ),
        "",
    )
    both_held = reference["held"] == candidate["held"] == "true"
    return dict(
        **{k: candidate[k] for k in ["cohort", *KEY]},
        reference_status=reference["status"],
        candidate_status=candidate["status"],
        gained_hold=reference["held"] != "true" and candidate["held"] == "true",
        lost_hold=reference["held"] == "true" and candidate["held"] != "true",
        reference_time_s=reference["time_s"],
        candidate_time_s=candidate["time_s"],
        both_held=both_held,
        held_time_difference_s=(
            float(candidate["time_s"]) - float(reference["time_s"]) if both_held else ""
        ),
        common_horizon_s=horizon,
        reference_matched_wait_s=round(prefixes[0][end] * 0.02, 6),
        candidate_matched_wait_s=round(prefixes[1][end] * 0.02, 6),
        matched_wait_difference_s=round(
            (prefixes[1][end] - prefixes[0][end]) * 0.02, 6
        ),
        first_command_change_s=change,
        fusion_available_steps=candidate["fusion_available_steps"],
    )


def summarize(cases, pairs):
    result = []
    for cohort in [*COHORTS, "known", "new", "all"]:

        def selected(row):
            return (
                cohort == "all"
                or row["cohort"] == cohort
                or row["cohort"].startswith(cohort + "-")
            )

        for mode in ["all", "none", "random", "blackout"]:
            matching = [
                r for r in pairs if selected(r) and (mode == "all" or r["mode"] == mode)
            ]
            for variant in VARIANTS:
                group = [
                    r
                    for r in cases
                    if selected(r)
                    and r["variant"] == variant
                    and (mode == "all" or r["mode"] == mode)
                ]
                if not group:
                    continue
                available = sum(r["fusion_available_steps"] for r in group)
                held_pairs = [r for r in matching if r["both_held"]]
                result.append(
                    dict(
                        cohort=cohort,
                        mode=mode,
                        variant=variant,
                        cases=len(group),
                        held=sum(r["held"] == "true" for r in group),
                        collision=sum(r["collision"] == "true" for r in group),
                        timeout=sum(r["status"] == "timeout" for r in group),
                        out_of_bounds=sum(
                            r["status"] == "out_of_bounds" for r in group
                        ),
                        mean_wait_s=statistics.mean(r["wait_time_s"] for r in group),
                        mean_stationary_wait_s=statistics.mean(
                            float(r["stationary_wait_time_s"]) for r in group
                        ),
                        short_reentries=sum(int(r["short_reentries"]) for r in group),
                        stationary_timeouts=sum(
                            r["status"] == "timeout" and r["final_stationary"]
                            for r in group
                        ),
                        cases_with_fusion=sum(
                            r["fusion_available_steps"] > 0 for r in group
                        ),
                        fusion_available_steps=available,
                        fusion_episodes=sum(r["fusion_episodes"] for r in group),
                        base_position_rms=(
                            math.sqrt(
                                sum(r["base_position_squared_sum"] for r in group)
                                / available
                            )
                            if available
                            else ""
                        ),
                        fused_position_rms=(
                            math.sqrt(
                                sum(r["fused_position_squared_sum"] for r in group)
                                / available
                            )
                            if available
                            else ""
                        ),
                        position_better_steps=sum(
                            r["position_better_steps"] for r in group
                        ),
                        position_worse_steps=sum(
                            r["position_worse_steps"] for r in group
                        ),
                        paired_gained_holds=sum(r["gained_hold"] for r in matching),
                        paired_lost_holds=sum(r["lost_hold"] for r in matching),
                        mean_matched_wait_difference_s=statistics.mean(
                            r["matched_wait_difference_s"] for r in matching
                        ),
                        both_held_cases=len(held_pairs),
                        mean_both_held_time_difference_s=(
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
    for cohort in COHORTS:
        parser.add_argument(f"--{cohort}", type=Path, required=True)
    parser.add_argument("--reference-white", type=Path, required=True)
    parser.add_argument("--reference-correlated", type=Path, required=True)
    parser.add_argument("--initial-batch", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    assert __debug__, "Run without Python -O"
    assert digest(SIMULATOR) == SIMULATOR_SHA
    assert digest(ROOT / "analyze_stationary_epoch_fusion.py") == PROTOTYPE_SHA
    old_sources = json.loads(
        (ROOT / "results/epoch-selection-holdout/sources.json").read_text()
    )
    cases, pairs, verification, sources = [], [], [], {}
    for cohort in COHORTS:
        run = getattr(args, cohort.replace("-", "_")).resolve()
        provenance = json.loads((run / "provenance.json").read_text())
        expected_seeds = (
            [268, 269, 270] if cohort.startswith("known-") else [271, 272, 273]
        )
        assert provenance["variants"] == VARIANTS
        assert provenance["seeds"] == expected_seeds
        assert provenance["noise_correlation_time_s"] == (
            1 if "correlated" in cohort else 0
        )
        assert provenance["simulator_sha256"] == SIMULATOR_SHA
        assert (
            provenance["inputs_by_variant_sha256"]["aligned"]
            == provenance["inputs_by_variant_sha256"]["epoch"]
        )
        expected_inputs = {
            p.name: digest(p)
            for p in (ROOT / "results/planning-clearance/fixtures/fine-padding").glob(
                "*.csv"
            )
        }
        assert provenance["inputs_sha256"] == expected_inputs
        results = read_rows(run / "all-results.csv")
        assert len(results) == 162
        event_rows = read_rows(run / "control-events.csv")
        assert len(event_rows) == 162
        events = {tuple(r[k] for k in ["variant", *KEY]): r for r in event_rows}
        raw_hashes = {}
        intervals = fusion_intervals = reference_rows = initial_rows = 0
        initial = args.initial_batch.resolve() / cohort
        initial_provenance = json.loads((initial / "provenance.json").read_text())
        assert initial_provenance["simulator_sha256"] == INITIAL_SIMULATOR_SHA
        for field in ["variants", "seeds", "noise_correlation_time_s", "inputs_sha256"]:
            assert initial_provenance[field] == provenance[field]
        assert (run / "all-results.csv").read_bytes() == (
            initial / "all-results.csv"
        ).read_bytes()
        initial_hashes = {}
        old = None
        if cohort.startswith("known-"):
            label = "correlated" if "correlated" in cohort else "white"
            old = getattr(args, f"reference_{label}").resolve()
            old_source = old_sources[f"{label}-validation"]
            assert digest(old / "provenance.json") == old_source["provenance_sha256"]
            assert (
                provenance["inputs_sha256"] == old_source["provenance"]["inputs_sha256"]
            )
        for mode in ["none", "random", "blackout"]:
            for seed in expected_seeds:
                groups = []
                for variant in VARIANTS:
                    trial = run / f"{variant}-{mode}-{seed}"
                    raw_hashes[f"{trial.name}/trajectories.csv"] = digest(
                        trial / "trajectories.csv"
                    )
                    initial_hashes[f"{trial.name}/trajectories.csv"] = digest(
                        initial / trial.name / "trajectories.csv"
                    )
                    initial_rows += compare_disabled(
                        trial / "trajectories.csv",
                        initial / trial.name / "trajectories.csv",
                        {"measured_speed"},
                    )
                    if old is not None and variant == "aligned":
                        assert (
                            digest(old / trial.name / "trajectories.csv")
                            == old_source["raw_sha256"][
                                f"{trial.name}/trajectories.csv"
                            ]
                        )
                        assert (trial / "results.csv").read_bytes() == (
                            old / trial.name / "results.csv"
                        ).read_bytes()
                        reference_rows += compare_disabled(
                            trial / "trajectories.csv",
                            old / trial.name / "trajectories.csv",
                        )
                    tracks = {}
                    for row in read_rows(trial / "trajectories.csv"):
                        assert None not in row, "CSV column count"
                        tracks.setdefault((row["scenario"], row["planner"]), []).append(
                            row
                        )
                    assert len(tracks) == 9
                    groups.append(tracks)
                for key in sorted(groups[0]):
                    current, prefixes = [], []
                    for variant, tracks in zip(VARIANTS, groups):
                        row = next(
                            r
                            for r in results
                            if r["variant"] == variant
                            and r["mode"] == mode
                            and int(r["seed"]) == seed
                            and (r["scenario"], r["planner"]) == key
                        )
                        diagnostic, prefix = diagnostics(tracks[key])
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
                            stationary_wait_time_s=event["stationary_wait_time_s"],
                            short_reentries=event["short_reentries"],
                        )
                        cases.append(record)
                        current.append(record)
                        prefixes.append(prefix)
                        intervals += diagnostic["intervals"]
                        if variant == "epoch":
                            fusion_intervals += diagnostic["intervals"]
                    pairs.append(paired(*current, prefixes, [g[key] for g in groups]))
                del groups
        sources[cohort] = dict(
            path=str(run),
            provenance=provenance,
            raw_sha256=raw_hashes,
            artifact_sha256={
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
        if old is not None:
            sources[cohort]["disabled_reference"] = dict(path=str(old), **old_source)
        sources[cohort]["initial_batch"] = dict(
            path=str(initial), provenance=initial_provenance, raw_sha256=initial_hashes
        )
        verification.append(
            dict(
                cohort=cohort,
                cases=len(results),
                physical_intervals=intervals,
                fusion_reconstruction_intervals=fusion_intervals,
                exact_preview_audits=len(read_rows(run / "preview-audit.csv")),
                disabled_reference_cases=81 if old is not None else 0,
                disabled_reference_rows=reference_rows,
                initial_batch_cases=len(results),
                initial_batch_rows=initial_rows,
            )
        )
        print(
            f"{cohort}: {len(results)} cases, {intervals} intervals, {reference_rows} reference rows",
            flush=True,
        )
    summary = summarize(cases, pairs)
    sources["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        helpers_sha256={
            name: digest(ROOT / name)
            for name in [
                "run.py",
                "verify.py",
                "verify_motion_filter.py",
                "verify_stopping_preview.py",
                "verify_joint_recovery.py",
                "verify_restart_consistency.py",
                "analyze_stationary_epoch_fusion.py",
                "summarize_stopping_preview.py",
            ]
        },
        protocol_sha256=digest(ROOT / "results/epoch-preview/protocol.md"),
        simulator_sha256=digest(SIMULATOR),
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, rows in [
        ("cases", cases),
        ("pairs", pairs),
        ("summary", summary),
        ("verification", verification),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")


if __name__ == "__main__":
    main()
