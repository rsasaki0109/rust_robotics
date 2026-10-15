#!/usr/bin/env python3
"""Compare fixed uncertainty-aware arc selectors on 497 saved pose pairs."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import tempfile

from analyze_recovery_arcs import ANGLES, LENGTH, arc_clearance, chord_check
from analyze_recovery_stalls import digest
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
KEY = ["cohort", "mode", "seed", "scenario", "planner", "time_s"]
YAW_OFFSETS = list(range(-4, 5))
YAW_LIMIT = math.radians(4)
YAW_GRID_ERROR = 2 * LENGTH * math.sin(math.radians(0.5) / 2)
RULES = ["nominal", "yaw_min", "pose_bound"]


def nearby_obstacles(pose, obstacles):
    x, y, _ = pose
    center = min(x, y, 50 - x, 50 - y, *(math.dist((x, y), p) for p in obstacles))
    return [p for p in obstacles if math.dist((x, y), p) <= center + LENGTH + 1e-9]


def choose(values):
    maximum = max(values.values())
    return min(
        (a for a, score in values.items() if score >= maximum - 1e-10),
        key=lambda a: (abs(a), a),
    )


def candidate_scores(pose, obstacles, angles=ANGLES):
    """Only estimated pose and map enter selection; truth is absent."""
    nearby = nearby_obstacles(pose, obstacles)
    scores = {}
    for angle in angles:
        values = {
            offset: arc_clearance(
                (pose[0], pose[1], pose[2] + math.radians(offset)),
                math.radians(angle),
                nearby,
            )
            for offset in YAW_OFFSETS
        }
        worst = min(values, key=lambda offset: (values[offset], abs(offset), offset))
        scores[angle] = dict(
            central=values[0],
            minimum_sampled_yaw=values[worst],
            yaw_lower=values[worst] - YAW_GRID_ERROR,
            worst_yaw_deg=worst,
        )
    return scores


def accepts(score, margin):
    return score > margin + 1e-9


def envelope_covers(estimated, truth, position_radius):
    position = math.dist(estimated[:2], truth[:2])
    yaw = abs(
        math.atan2(math.sin(truth[2] - estimated[2]), math.cos(truth[2] - estimated[2]))
    )
    return (
        position <= position_radius + 1e-12 and yaw <= YAW_LIMIT + 1e-12,
        position,
        yaw,
    )


def evaluate(identity, filtered, truth, previous_pair, obstacles):
    pose = tuple(float(filtered[k]) for k in ["x", "y", "yaw"])
    true_pose = tuple(float(truth[k]) for k in ["x", "y", "yaw"])
    sigma, age = float(filtered["effective_position_sigma"]), float(
        filtered["observation_age"]
    )
    assert sigma == 0.1
    radius = 2 * math.sqrt(2) * sigma + 0.02 * age
    margin = 0.6 + radius
    assert abs(margin - float(filtered["recovery_current_age_margin"])) < 1e-12
    scores = candidate_scores(pose, obstacles)
    nominal = choose({a: s["central"] for a, s in scores.items()})
    robust = choose({a: s["yaw_lower"] for a, s in scores.items()})
    assert (
        nominal
        == int(filtered["best_steering_deg"])
        == int(previous_pair["filtered_best_steering_deg"])
    )
    assert abs(scores[nominal]["central"] - float(filtered["best_clearance"])) < 1e-8
    covered, position_error, yaw_error = envelope_covers(pose, true_pose, radius)
    assert abs(position_error - float(previous_pair["position_error"])) < 1e-12
    assert abs(yaw_error - float(previous_pair["yaw_error_rad"])) < 1e-12
    actual, audits = {}, []
    for angle in sorted({nominal, robust}):
        for offset in YAW_OFFSETS:
            shifted = pose[0], pose[1], pose[2] + math.radians(offset)
            full = arc_clearance(shifted, math.radians(angle), obstacles)
            if offset == 0:
                assert abs(full - scores[angle]["central"]) < 1e-9
            assert full >= scores[angle]["minimum_sampled_yaw"] - 1e-9
            if offset == scores[angle]["worst_yaw_deg"]:
                assert abs(full - scores[angle]["minimum_sampled_yaw"]) < 1e-9
                error, bound = chord_check(
                    shifted, math.radians(angle), obstacles, full
                )
                audits.append(
                    dict(
                        **identity,
                        role="worst_sampled_heading",
                        steering_deg=angle,
                        yaw_offset_deg=offset,
                        exact_clearance=full,
                        chord_error=error,
                        sagitta_bound=bound,
                    )
                )
        actual[angle] = arc_clearance(true_pose, math.radians(angle), obstacles)
        error, bound = chord_check(
            true_pose, math.radians(angle), obstacles, actual[angle]
        )
        audits.append(
            dict(
                **identity,
                role="true_pose",
                steering_deg=angle,
                yaw_offset_deg=0,
                exact_clearance=actual[angle],
                chord_error=error,
                sagitta_bound=bound,
            )
        )
        if covered:
            assert actual[angle] >= scores[angle]["yaw_lower"] - radius - 2e-8
    assert (
        abs(actual[nominal] - float(previous_pair["true_clearance_at_filtered_best"]))
        < 1e-8
    )
    decisions = []
    for rule in RULES:
        angle = nominal if rule == "nominal" else robust
        score = (
            scores[angle]["central"]
            if rule == "nominal"
            else scores[angle]["yaw_lower"] - (radius if rule == "pose_bound" else 0)
        )
        accepted = accepts(score, margin)
        true_pass = accepts(actual[angle], margin)
        decisions.append(
            dict(
                **identity,
                rule=rule,
                steering_deg=angle,
                nominal_steering_deg=nominal,
                score=score,
                target_margin=margin,
                accepted=accepted,
                true_clearance=actual[angle],
                true_margin_pass=true_pass,
                accepted_true_pass=accepted and true_pass,
                mismatch=accepted and not true_pass,
                abstained_with_true_grid_witness=not accepted
                and truth["recovery_current_age_status"] == "grid_pass",
                envelope_covers=covered,
                position_error=position_error,
                yaw_error_rad=yaw_error,
                position_envelope=radius,
                pose_clearance_lower=scores[angle]["yaw_lower"] - radius,
                steering_changed=angle != nominal,
            )
        )
    assert decisions[0]["accepted"] == (
        previous_pair["filtered_selected_passes_current_age"] == "True"
    )
    assert decisions[0]["true_margin_pass"] == (
        previous_pair["true_selected_passes_current_age"] == "True"
    )
    if covered and decisions[-1]["accepted"]:
        assert decisions[-1]["true_margin_pass"]
    grids = [
        dict(
            **identity,
            steering_deg=a,
            **s,
            position_envelope=radius,
            pose_clearance_lower=s["yaw_lower"] - radius,
        )
        for a, s in scores.items()
    ]
    return decisions, grids, audits


def summarize(decisions):
    result = []
    for reason in ["final", "fresh_stationary"]:
        baseline = {
            tuple(r[k] for k in KEY): r
            for r in decisions
            if r["reason"] == reason and r["rule"] == "nominal"
        }
        for rule in RULES:
            group = [
                r for r in decisions if r["reason"] == reason and r["rule"] == rule
            ]
            row = dict(reason=reason, rule=rule, checkpoints=len(group))
            row.update(
                {
                    k: sum(r[k] for r in group)
                    for k in [
                        "accepted",
                        "accepted_true_pass",
                        "mismatch",
                        "abstained_with_true_grid_witness",
                        "envelope_covers",
                        "steering_changed",
                    ]
                }
            )
            row["abstained"] = len(group) - row["accepted"]
            row["accepted_covered"] = sum(
                r["accepted"] and r["envelope_covers"] for r in group
            )
            row["mismatch_covered"] = sum(
                r["mismatch"] and r["envelope_covers"] for r in group
            )
            row["gained_accepted_true_pass"] = sum(
                r["accepted_true_pass"]
                and not baseline[tuple(r[k] for k in KEY)]["accepted_true_pass"]
                for r in group
            )
            row["lost_accepted_true_pass"] = sum(
                not r["accepted_true_pass"]
                and baseline[tuple(r[k] for k in KEY)]["accepted_true_pass"]
                for r in group
            )
            result.append(row)
    return result


def plot(output, summary):
    with tempfile.TemporaryDirectory(prefix="robust-arc-plot-") as cache:
        os.environ.setdefault("MPLCONFIGDIR", cache)
        os.environ.setdefault("XDG_CACHE_HOME", cache)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.rcParams.update(
            {"svg.fonttype": "none", "svg.hashsalt": "robust-arc-selection"}
        )
        fig, axes = plt.subplots(1, 2, figsize=(10, 5), layout="constrained")
        for ax, reason, title in zip(
            axes,
            ["final", "fresh_stationary"],
            ["Final checkpoints (81)", "Fresh stationary checkpoints (416)"],
        ):
            rows = [
                next(r for r in summary if r["reason"] == reason and r["rule"] == rule)
                for rule in RULES
            ]
            bottom = [0] * 3
            for field, color, label in [
                ("accepted_true_pass", "#16a34a", "Accepted; true margin passes"),
                ("mismatch", "#f97316", "Accepted; true margin fails"),
                ("abstained", "#94a3b8", "Abstained"),
            ]:
                values = [r[field] for r in rows]
                ax.bar(range(3), values, bottom=bottom, color=color, label=label)
                for i, value in enumerate(values):
                    if 0 < value < 0.04 * rows[i]["checkpoints"]:
                        ax.annotate(
                            str(value),
                            xy=(i, bottom[i] + value / 2),
                            xytext=(-20, 20),
                            textcoords="offset points",
                            ha="center",
                            fontsize=9,
                            arrowprops=dict(arrowstyle="-", color="black"),
                        )
                    elif value:
                        ax.text(
                            i,
                            bottom[i] + value / 2,
                            str(value),
                            ha="center",
                            va="center",
                            fontsize=9,
                        )
                bottom = [a + b for a, b in zip(bottom, values)]
            ax.set_xticks(range(3), ["Nominal", "Yaw min", "Pose bound"])
            ax.set(title=title, ylabel="Recorded checkpoints")
        axes[-1].legend(loc="upper center", bbox_to_anchor=(0.5, -0.1), fontsize=8)
        fig.suptitle(
            "Arc selection on fixed failed-run poses\nSame full recovery margin; heading envelope +/-4 degrees"
        )
        svg = output / "robust-arc-selection.svg"
        fig.savefig(svg, metadata={"Date": None})
        svg.write_text(
            "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(output / "robust-arc-selection.png", dpi=160)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    source = ROOT / "results/aligned-arcs"
    records = list(csv.DictReader((source / "checkpoints.csv").open()))
    paired = list(csv.DictReader((source / "paired-poses.csv").open()))
    assert len(records) == 994 and len(paired) == 497
    lookup = {(tuple(r[k] for k in KEY), r["pose_kind"]): r for r in records}
    provenance = json.loads((source / "sources.json").read_text())
    obstacles_path = (
        ROOT / "results/planning-clearance/fixtures/fine-padding/obstacles.csv"
    )
    for cohort in ["white-validation", "correlated-validation"]:
        assert (
            digest(obstacles_path)
            == provenance[cohort]["provenance"]["inputs_by_variant_sha256"]["aligned"][
                "obstacles.csv"
            ]
        )
    maps = {}
    for row in csv.DictReader(obstacles_path.open()):
        maps.setdefault(row["scenario"], []).append((float(row["x"]), float(row["y"])))
    decisions, scores, audits = [], [], []
    for pair in paired:
        key = tuple(pair[k] for k in KEY)
        identity = {k: pair[k] for k in [*KEY, "decision_time_s", "reason"]}
        result, grid, checked = evaluate(
            identity,
            lookup[(key, "filtered")],
            lookup[(key, "truth")],
            pair,
            maps[pair["scenario"]],
        )
        decisions.extend(result)
        scores.extend(grid)
        audits.extend(checked)
    summary = summarize(decisions)
    evidence = dict(
        inputs_sha256={
            name: digest(source / name)
            for name in ["checkpoints.csv", "paired-poses.csv", "sources.json"]
        },
        obstacles_sha256=digest(obstacles_path),
        script_sha256=digest(Path(__file__)),
        helpers_sha256={
            name: digest(ROOT / name)
            for name in [
                "analyze_recovery_arcs.py",
                "analyze_recovery_stalls.py",
                "summarize_stopping_preview.py",
                "verify_stopping_preview.py",
            ]
        },
        protocol_sha256=digest(ROOT / "results/robust-arc-selection/protocol.md"),
        heading_offsets_degrees=YAW_OFFSETS,
        heading_error_limit_degrees=4,
        heading_grid_error_bound=YAW_GRID_ERROR,
        checkpoints=len(paired),
        grid_rows=len(scores),
        geometry_audits=len(audits),
        note="Truth used only for evaluation and envelope coverage; no controller or probability guarantee.",
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, data in [
        ("decisions.csv", decisions),
        ("candidate-scores.csv", scores),
        ("geometry-audit.csv", audits),
        ("summary.csv", summary),
    ]:
        save_rows(args.output / name, data)
    (args.output / "sources.json").write_text(json.dumps(evidence, indent=2) + "\n")
    if args.plot:
        plot(args.output, summary)
    for row in summary:
        print(row)
    print(
        f"Checked {len(paired)} checkpoints, {len(scores)} candidate scores and {len(audits)} chord audits"
    )


if __name__ == "__main__":
    main()
