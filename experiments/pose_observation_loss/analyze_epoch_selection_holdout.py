#!/usr/bin/env python3
"""Validate frozen position/arc replay rules on new aligned-control observations."""

import argparse
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
import statistics

from analyze_epoch_spread_selection import evaluate, summarize
from analyze_filter_position_components import Window
from analyze_recovery_arcs import ANGLES, arc_clearance
from analyze_recovery_stalls import digest
from analyze_robust_arc_selection import KEY, accepts, choose
from analyze_stationary_epoch_fusion import StationaryEpochFusion
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
SEEDS = [268, 269, 270]


def checkpoint_reason(before, row, final):
    if final:
        return "final"
    time = float(row["time_s"])
    fresh = (
        150.5 - 1e-8 <= time <= 175.5 + 1e-8
        and abs(time % 5 - 0.5) < 1e-8
        and abs(float(row["observation_age"]) - 0.5) < 1e-8
        and abs(float(before["v"])) <= 0.02
    )
    return "fresh_stationary" if fresh else None


def case_outcome(case):
    if case["collision"] == "true":
        return "collision"
    if case["held"] == "true":
        return "held"
    return "timeout" if case["status"] == "timeout" else "other"


def reference_decision(pose, truth, margin, obstacles):
    # Independent full-grid scans validate the reused selector's pruned scores.
    values = {a: arc_clearance(pose, math.radians(a), obstacles) for a in ANGLES}
    angle = choose(values)
    actual = arc_clearance(truth, math.radians(angle), obstacles)
    accepted = accepts(values[angle], margin)
    return dict(
        steering_deg=angle,
        estimated_clearance=values[angle],
        true_clearance=actual,
        target_margin=margin,
        accepted=str(accepted),
        accepted_true_pass=str(accepted and accepts(actual, margin)),
    )


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def outcome_summaries(decisions):
    rows = []
    for outcome in ["all", *sorted({r["outcome"] for r in decisions})]:
        group = [r for r in decisions if outcome == "all" or r["outcome"] == outcome]
        rows.extend(dict(outcome=outcome, **r) for r in summarize(group))
    return rows


def position_summaries(estimates):
    rows = []
    for cohort in ["white-validation", "correlated-validation"]:
        for reason in ["final", "fresh_stationary"]:
            for outcome in ["all", *sorted({r["outcome"] for r in estimates})]:
                for subset in ["all", "available", "fallback"]:
                    group = [
                        r
                        for r in estimates
                        if r["cohort"] == cohort
                        and r["reason"] == reason
                        and (outcome == "all" or r["outcome"] == outcome)
                        and (
                            subset == "all"
                            or (r["available"] == "True") == (subset == "available")
                        )
                    ]
                    if not group:
                        continue
                    row = dict(
                        cohort=cohort,
                        reason=reason,
                        outcome=outcome,
                        subset=subset,
                        checkpoints=len(group),
                        available=sum(r["available"] == "True" for r in group),
                        stationary=sum(r["stationary"] for r in group),
                        precision_active=sum(r["precision_active"] for r in group),
                        sigma_min=min(float(r["sigma"]) for r in group),
                        sigma_max=max(float(r["sigma"]) for r in group),
                    )
                    for name in ["baseline", "candidate"]:
                        errors = [r[f"{name}_error"] for r in group]
                        row.update(
                            {
                                f"{name}_mean": statistics.mean(errors),
                                f"{name}_median": statistics.median(errors),
                                f"{name}_rms": math.sqrt(
                                    statistics.mean(e * e for e in errors)
                                ),
                            }
                        )
                    rows.append(row)
    return rows


def analyze_track(case, track, obstacles):
    base = {k: case[k] for k in KEY if k != "time_s"}
    outcome = case_outcome(case)
    baseline_filter, fusion = Window(), StationaryEpochFusion()
    estimates, epochs, decisions, spreads, scores, audits = [], [], [], [], [], []
    for i, (before, row) in enumerate(zip(track, track[1:]), 1):
        assert abs(float(row["time_s"]) - float(before["time_s"]) - 0.02) < 1e-8
        baseline_filter.advance(before, row)
        measured = tuple(
            float(before[a]) + float(row[f"noise_{a}"]) + float(row[f"bias_{a}"])
            for a in ["x", "y", "yaw"]
        )
        fusion.observe(
            float(row["time_s"]),
            measured,
            float(before["v"]),
            float(row["steering"]),
            row["correction_received"] == "true",
            row["precision_activation"] == "true",
            float(row["observation_age"]),
        )
        reason = checkpoint_reason(before, row, i == len(track) - 1)
        if reason is None:
            continue
        identity = dict(
            **base,
            time_s=row["time_s"],
            decision_time_s=before["time_s"],
            reason=reason,
        )
        metadata = dict(
            outcome=outcome,
            stationary=abs(float(before["v"])) <= 0.02,
            precision_active=row["precision_active"] == "true",
            sigma=row["effective_position_sigma"],
        )
        baseline = tuple(float(row[f"preview_pose_{a}"]) for a in ["x", "y", "yaw"])
        truth = tuple(float(before[a]) for a in ["x", "y", "yaw"])
        position = fusion.position()
        available = position is not None
        candidate = (*position, baseline[2]) if available else baseline
        estimate = dict(
            **identity,
            **metadata,
            available=str(available),
            retained_epochs=len(fusion.groups),
            observation_age=row["observation_age"],
            baseline_x=baseline[0],
            baseline_y=baseline[1],
            candidate_x=candidate[0],
            candidate_y=candidate[1],
            heading=baseline[2],
            truth_x=truth[0],
            truth_y=truth[1],
            baseline_error=math.dist(baseline[:2], truth[:2]),
            candidate_error=math.dist(candidate[:2], truth[:2]),
        )
        means = fusion.means()
        margin = (
            0.6
            + 2 * math.sqrt(2) * float(row["effective_position_sigma"])
            + 0.02 * float(row["observation_age"])
        )
        prior = dict(
            baseline=reference_decision(baseline, truth, margin, obstacles),
            fused=reference_decision(candidate, truth, margin, obstacles),
            truth_yaw=truth[2],
        )
        d, s, g, a = evaluate(estimate, means, prior, obstacles)
        estimates.append(estimate)
        epochs.extend(
            dict(**identity, **metadata, available=available, **m) for m in means
        )
        for data in [d, [s], g, a]:
            for r in data:
                r.update(metadata)
        decisions.extend(d)
        spreads.append(s)
        scores.extend(g)
        audits.extend(a)
    assert sum(r["reason"] == "final" for r in estimates) == 1
    return (
        estimates,
        epochs,
        decisions,
        spreads,
        scores,
        audits,
        baseline_filter.max_error,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--white", required=True, type=Path)
    parser.add_argument("--correlated", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="New directory")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    frozen_sources = {
        name: json.loads((ROOT / f"results/{name}/sources.json").read_text())
        for name in [
            "epoch-spread-selection",
            "stationary-epoch-fusion",
            "filter-position-components",
        ]
    }
    for name, filename in [
        ("epoch-spread-selection", "analyze_epoch_spread_selection.py"),
        ("stationary-epoch-fusion", "analyze_stationary_epoch_fusion.py"),
        ("filter-position-components", "analyze_filter_position_components.py"),
    ]:
        assert (
            digest(ROOT / filename) == frozen_sources[name]["script_sha256"]
        ), "frozen rule changed"
    for filename, expected in frozen_sources["epoch-spread-selection"][
        "helpers_sha256"
    ].items():
        assert digest(ROOT / filename) == expected, filename
    aligned = json.loads((ROOT / "results/aligned-arcs/sources.json").read_text())[
        "white-validation"
    ]["provenance"]
    simulator = (
        ROOT.parents[1]
        / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
    )
    assert digest(simulator) == aligned["simulator_sha256"]
    maps = defaultdict(list)
    obstacles_path = (
        ROOT / "results/planning-clearance/fixtures/fine-padding/obstacles.csv"
    )
    for r in read_rows(obstacles_path):
        maps[r["scenario"]].append((float(r["x"]), float(r["y"])))
    cases, estimates, epochs, decisions, spreads, scores, audits = (
        [],
        [],
        [],
        [],
        [],
        [],
        [],
    )
    sources, steps, max_error = {}, 0, 0.0
    for cohort, run, tau in [
        ("white-validation", args.white, 0),
        ("correlated-validation", args.correlated, 1),
    ]:
        provenance = json.loads((run / "provenance.json").read_text())
        assert provenance["seeds"] == SEEDS and provenance["variants"] == [
            "entryline",
            "aligned",
        ]
        assert provenance["noise_correlation_time_s"] == tau
        assert provenance["simulator_sha256"] == aligned["simulator_sha256"]
        assert (
            provenance["inputs_by_variant_sha256"]["aligned"]
            == aligned["inputs_by_variant_sha256"]["aligned"]
        )
        generated = read_rows(run / "all-results.csv")
        selected = [
            dict(cohort=cohort, **r) for r in generated if r["variant"] == "aligned"
        ]
        assert len(generated) == 162 and len(selected) == 81
        assert {int(r["seed"]) for r in selected} == set(SEEDS)
        hashes = {}
        for mode, seed in sorted({(r["mode"], r["seed"]) for r in selected}):
            group = [r for r in selected if (r["mode"], r["seed"]) == (mode, seed)]
            assert len(group) == 9
            relative = f"aligned-{mode}-{seed}/trajectories.csv"
            path = run / relative
            hashes[relative] = digest(path)
            tracks = {(r["scenario"], r["planner"]): [] for r in group}
            with path.open() as stream:
                for r in csv.DictReader(stream):
                    tracks[(r["scenario"], r["planner"])].append(r)
            for case in group:
                track = tracks.pop((case["scenario"], case["planner"]))
                assert float(track[-1]["time_s"]) == float(case["time_s"])
                e, ep, d, s, g, a, error = analyze_track(
                    case, track, maps[case["scenario"]]
                )
                cases.append(
                    dict(
                        outcome=case_outcome(case), selected_checkpoints=len(e), **case
                    )
                )
                estimates.extend(e)
                epochs.extend(ep)
                decisions.extend(d)
                spreads.extend(s)
                scores.extend(g)
                audits.extend(a)
                steps += len(track) - 1
                max_error = max(max_error, error)
            print(f"Validated {cohort} {mode} {seed}: 9 cases", flush=True)
        sources[cohort] = dict(
            provenance=provenance,
            raw_sha256=hashes,
            summary_sha256=digest(run / "all-results.csv"),
            provenance_sha256=digest(run / "provenance.json"),
        )
    assert len(cases) == 162 and sum(r["reason"] == "final" for r in estimates) == 162
    assert len({tuple(r[k] for k in KEY) for r in estimates}) == len(estimates)
    assert len(decisions) == 4 * len(estimates) and len(scores) == 71 * len(estimates)
    assert len(audits) == 8 * len(estimates) + 2 * sum(
        r["available"] == "True" for r in estimates
    )
    summary = outcome_summaries(decisions)
    positions = position_summaries(estimates)
    helpers = [
        "analyze_epoch_spread_selection.py",
        "analyze_stationary_epoch_fusion.py",
        "analyze_filter_position_components.py",
        "analyze_recovery_arcs.py",
        "analyze_recovery_stalls.py",
        "analyze_robust_arc_selection.py",
        "summarize_stopping_preview.py",
        "verify_stopping_preview.py",
    ]
    sources["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        helpers_sha256={n: digest(ROOT / n) for n in helpers},
        frozen_sources_sha256={
            n: digest(ROOT / f"results/{n}/sources.json") for n in frozen_sources
        },
        protocol_sha256=digest(ROOT / "results/epoch-selection-holdout/protocol.md"),
        simulator_sha256=digest(simulator),
        fixture_sha256={
            p.name: digest(p)
            for p in (ROOT / "results/planning-clearance/fixtures/fine-padding").glob(
                "*.csv"
            )
        },
        cases=len(cases),
        checkpoints=len(estimates),
        epoch_rows=len(epochs),
        score_rows=len(scores),
        geometry_audits=len(audits),
        filter_steps_checked=steps,
        max_logged_filter_error=max_error,
        note="New seeds with unchanged generating control; frozen rules only in observation replay, not candidate control/hold validation.",
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, data in [
        ("cases.csv", cases),
        ("estimates.csv", estimates),
        ("epoch-means.csv", epochs),
        ("decisions.csv", decisions),
        ("spread.csv", spreads),
        ("candidate-scores.csv", scores),
        ("geometry-audit.csv", audits),
        ("summary.csv", summary),
        ("position-summary.csv", positions),
    ]:
        save_rows(args.output / name, data)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(
        f"Validated {len(cases)} cases, {len(estimates)} checkpoints and {len(audits)} geometry audits"
    )


if __name__ == "__main__":
    main()
