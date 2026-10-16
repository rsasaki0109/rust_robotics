#!/usr/bin/env python3
"""Compare fixed arc selectors using tracked three-epoch positional spread."""

import argparse
from collections import defaultdict
import csv
import json
import math
from pathlib import Path

from analyze_recovery_arcs import ANGLES, arc_clearance, chord_check
from analyze_recovery_stalls import digest
from analyze_robust_arc_selection import (
    KEY,
    YAW_GRID_ERROR,
    YAW_OFFSETS,
    accepts,
    candidate_scores,
    choose,
    envelope_covers,
    nearby_obstacles,
)
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
RULES = ["baseline", "fused_center", "epoch_min", "spread_yaw_bound"]


def score_angles(pose, positions, available, obstacles, angles=ANGLES):
    """No truth input: all scores use fixed estimated positions and heading."""
    radius = max(math.dist(pose[:2], p) for p in positions) if available else 0.0
    nearby = nearby_obstacles(pose, obstacles)
    heading_scores = candidate_scores(pose, obstacles, angles) if available else {}
    epoch_nearby = (
        [nearby_obstacles((*p, pose[2]), obstacles) for p in positions]
        if available
        else []
    )
    scores = {}
    for angle in angles:
        central = arc_clearance(pose, math.radians(angle), nearby)
        if available:
            epoch_values = [
                arc_clearance((*p, pose[2]), math.radians(angle), obs)
                for p, obs in zip(positions, epoch_nearby)
            ]
            worst = min(range(len(positions)), key=lambda i: (epoch_values[i], i))
            heading = heading_scores[angle]
            scores[angle] = dict(
                fused_center=central,
                epoch_min=epoch_values[worst],
                spread_yaw_bound=heading["yaw_lower"] - radius,
                worst_epoch_index=worst,
                worst_yaw_deg=heading["worst_yaw_deg"],
            )
        else:
            scores[angle] = dict(
                fused_center=central,
                epoch_min=central,
                spread_yaw_bound=central,
                worst_epoch_index=-1,
                worst_yaw_deg=0,
            )
    return radius, scores


def summarize(rows):
    result = []
    for cohort in ["white-validation", "correlated-validation"]:
        for reason in ["final", "fresh_stationary"]:
            for subset in ["all", "available", "fallback"]:
                for rule in RULES:
                    group = [
                        r
                        for r in rows
                        if r["cohort"] == cohort
                        and r["reason"] == reason
                        and r["rule"] == rule
                        and (
                            subset == "all" or r["available"] == (subset == "available")
                        )
                    ]
                    result.append(
                        dict(
                            cohort=cohort,
                            reason=reason,
                            subset=subset,
                            rule=rule,
                            checkpoints=len(group),
                            **{
                                k: sum(r[k] for r in group)
                                for k in [
                                    "available",
                                    "envelope_covers",
                                    "accepted",
                                    "accepted_true_pass",
                                    "mismatch",
                                    "abstained",
                                    "angle_changed",
                                    "gained_true_pass",
                                    "lost_true_pass",
                                    "accepted_covered",
                                    "mismatch_covered",
                                ]
                            },
                        )
                    )
    return result


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def audit(identity, rule, kind, pose, angle, obstacles):
    clearance = arc_clearance(pose, math.radians(angle), obstacles)
    error, bound = chord_check(pose, math.radians(angle), obstacles, clearance)
    return clearance, dict(
        **identity,
        rule=rule,
        pose_kind=kind,
        steering_deg=angle,
        x=pose[0],
        y=pose[1],
        yaw=pose[2],
        exact_clearance=clearance,
        chord_error=error,
        sagitta_bound=bound,
    )


def evaluate(estimate, epochs, prior, obstacles):
    identity = {k: estimate[k] for k in [*KEY, "decision_time_s", "reason"]}
    baseline = (
        float(estimate["baseline_x"]),
        float(estimate["baseline_y"]),
        float(estimate["heading"]),
    )
    center = float(estimate["candidate_x"]), float(estimate["candidate_y"]), baseline[2]
    truth = (
        float(estimate["truth_x"]),
        float(estimate["truth_y"]),
        float(prior["truth_yaw"]),
    )
    available = estimate["available"] == "True"
    positions = [(float(r["mean_x"]), float(r["mean_y"])) for r in epochs]
    if available:
        assert len(positions) == 3
        assert all(
            abs(sum(p[i] for p in positions) / 3 - center[i]) < 1e-12 for i in range(2)
        )
    else:
        assert baseline == center
    radius, scores = score_angles(center, positions, available, obstacles)
    covered, error, yaw_error = envelope_covers(center, truth, radius)
    covered = available and covered
    margin = float(prior["fused"]["target_margin"])
    assert (
        abs(
            margin
            - (
                0.6
                + 2 * math.sqrt(2) * float(estimate["sigma"])
                + 0.02 * float(estimate["observation_age"])
            )
        )
        < 1e-12
    )
    selected = {r: choose({a: s[r] for a, s in scores.items()}) for r in RULES[1:]}
    selected["baseline"] = int(prior["baseline"]["steering_deg"])
    assert selected["fused_center"] == int(prior["fused"]["steering_deg"])
    decisions, audits = [], []
    for rule in RULES:
        angle = selected[rule]
        score = (
            float(prior["baseline"]["estimated_clearance"])
            if rule == "baseline"
            else scores[angle][rule]
        )
        origins = [("estimated", baseline if rule == "baseline" else center)]
        if available and rule == "epoch_min":
            origins = [(f"epoch_{i}", (*p, center[2])) for i, p in enumerate(positions)]
        if available and rule == "spread_yaw_bound":
            offset = scores[angle]["worst_yaw_deg"]
            origins = [
                ("worst_heading", (*center[:2], center[2] + math.radians(offset)))
            ]
            full = [
                arc_clearance(
                    (*center[:2], center[2] + math.radians(o)),
                    math.radians(angle),
                    obstacles,
                )
                for o in YAW_OFFSETS
            ]
            assert abs(min(full) - YAW_GRID_ERROR - radius - score) < 1e-9
        clearances = []
        for kind, pose in origins:
            distance, checked = audit(identity, rule, kind, pose, angle, obstacles)
            clearances.append(distance)
            audits.append(checked)
        expected = min(clearances) - (
            YAW_GRID_ERROR + radius if available and rule == "spread_yaw_bound" else 0
        )
        assert abs(score - expected) < 1e-9
        actual, checked = audit(identity, rule, "truth", truth, angle, obstacles)
        audits.append(checked)
        accepted, true_pass = accepts(score, margin), accepts(actual, margin)
        if rule in ["baseline", "fused_center"]:
            saved = prior["baseline" if rule == "baseline" else "fused"]
            assert abs(score - float(saved["estimated_clearance"])) < 1e-9
            assert abs(actual - float(saved["true_clearance"])) < 1e-9
            assert accepted == (saved["accepted"] == "True")
            assert (accepted and true_pass) == (saved["accepted_true_pass"] == "True")
        if covered and rule == "spread_yaw_bound":
            assert actual >= score - 2e-8
            assert not accepted or true_pass
        decisions.append(
            dict(
                **identity,
                rule=rule,
                available=available,
                radius=radius,
                envelope_covers=covered,
                steering_deg=angle,
                score=score,
                target_margin=margin,
                true_clearance=actual,
                accepted=accepted,
                accepted_true_pass=accepted and true_pass,
                mismatch=accepted and not true_pass,
                abstained=not accepted,
                accepted_covered=accepted and covered,
                mismatch_covered=accepted and not true_pass and covered,
            )
        )
    fused = decisions[1]
    for row in decisions:
        row.update(
            angle_changed=row["steering_deg"] != fused["steering_deg"],
            gained_true_pass=row["accepted_true_pass"]
            and not fused["accepted_true_pass"],
            lost_true_pass=not row["accepted_true_pass"]
            and fused["accepted_true_pass"],
        )
        if not available:
            assert row["steering_deg"] == fused["steering_deg"]
            assert abs(row["score"] - fused["score"]) < 1e-9
    spread = dict(
        **identity,
        available=available,
        radius=radius,
        position_error=error,
        yaw_error_rad=yaw_error,
        position_covered=available and error <= radius + 1e-12,
        heading_covered=available and yaw_error <= math.radians(4) + 1e-12,
        envelope_covers=covered,
    )
    grid = [
        dict(**identity, available=available, steering_deg=a, radius=radius, **s)
        for a, s in scores.items()
    ]
    return decisions, spread, grid, audits


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="New directory")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    source = ROOT / "results/stationary-epoch-fusion"
    provenance = json.loads((source / "sources.json").read_text())
    for name, expected in provenance["inputs_sha256"].items():
        assert digest(ROOT / name) == expected, name
    estimates = read_rows(source / "estimates.csv")
    assert len(estimates) == 497
    epochs = defaultdict(list)
    for r in read_rows(source / "epoch-means.csv"):
        epochs[tuple(r[k] for k in KEY)].append(r)
    previous = {
        (tuple(r[k] for k in KEY), r["rule"]): r
        for r in read_rows(source / "decisions.csv")
    }
    truths = {
        tuple(r[k] for k in KEY): r
        for r in read_rows(ROOT / "results/aligned-arcs/checkpoints.csv")
        if r["pose_kind"] == "truth"
    }
    obstacles_path = (
        ROOT / "results/planning-clearance/fixtures/fine-padding/obstacles.csv"
    )
    maps = defaultdict(list)
    for r in read_rows(obstacles_path):
        maps[r["scenario"]].append((float(r["x"]), float(r["y"])))
    decisions, spreads, scores, audits = [], [], [], []
    for estimate in estimates:
        key = tuple(estimate[k] for k in KEY)
        prior = dict(
            baseline=previous[(key, "baseline")],
            fused=previous[(key, "three_epoch")],
            truth_yaw=truths[key]["yaw"],
        )
        d, s, g, a = evaluate(estimate, epochs[key], prior, maps[estimate["scenario"]])
        decisions.extend(d)
        spreads.append(s)
        scores.extend(g)
        audits.extend(a)
    assert len(decisions) == 1988 and len(scores) == 35287 and len(audits) == 4754
    assert sum(r["available"] for r in spreads) == 389
    summary = summarize(decisions)
    inputs = [
        source / n
        for n in ["estimates.csv", "epoch-means.csv", "decisions.csv", "sources.json"]
    ] + [ROOT / "results/aligned-arcs/checkpoints.csv", obstacles_path]
    sources = dict(
        inputs_sha256={str(p.relative_to(ROOT)): digest(p) for p in inputs},
        script_sha256=digest(Path(__file__)),
        helpers_sha256={
            name: digest(ROOT / name)
            for name in [
                "analyze_recovery_arcs.py",
                "analyze_recovery_stalls.py",
                "analyze_robust_arc_selection.py",
                "summarize_stopping_preview.py",
                "verify_stopping_preview.py",
            ]
        },
        protocol_sha256=digest(ROOT / "results/epoch-spread-selection/protocol.md"),
        checkpoints=len(spreads),
        available=389,
        score_rows=len(scores),
        geometry_audits=len(audits),
        note="Spread/heading envelope is assumed, not a confidence guarantee; fixed failed-run poses, no new dynamics or hold result.",
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, data in [
        ("decisions.csv", decisions),
        ("spread.csv", spreads),
        ("candidate-scores.csv", scores),
        ("geometry-audit.csv", audits),
        ("summary.csv", summary),
    ]:
        save_rows(args.output / name, data)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    for r in summary:
        if r["subset"] == "all":
            print(r)


if __name__ == "__main__":
    main()
