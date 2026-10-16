#!/usr/bin/env python3
"""Separate position/heading effects for all pinned recovery-path decisions."""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics

from analyze_arc_pose_components import hybrid_poses, signed_losses
from analyze_commanded_recovery_arc import (
    META,
    arc_components,
    classify,
    flag,
    identity,
)
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import EPS, KEY, subsets
from analyze_recovery_arcs import arc_clearance, chord_check
from analyze_terminal_arc_live import SIMULATOR
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/commanded-recovery-arc"
PROTOCOL = ROOT / "results/commanded-arc-components/protocol.md"
PINNED = {
    "accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "geometry.csv": "87f015ae26a102d3da20f8270567b3ff6bb29925d446bc0b84feef2e4806084a",
    "decisions.csv": "5e01cbf0a031e37b0e6bec9c15e699f196e05e41dda5e65712d5b0ca9a745e73",
    "transitions.csv": "a00da7d3c247365fd31c2574247f0f2e68049b0495cb5621b4836f6774340388",
    "sources.json": "af392595563b6021f32dd6a92cd835bb8ca9ebb96e038a76659a93f86ce08ba1",
}
CATEGORIES = ["position_only", "heading_only", "either_alone", "joint_required"]
GROUPS = ["all", "optimistic", "conservative", "optimistic_arc_gain"]


def correction_category(passed):
    """Which single coordinate substitution reaches the full-truth decision?"""
    target = passed["TT"]
    if passed["FF"] == target:
        return "same_decision"
    position, heading = passed["TF"] == target, passed["FT"] == target
    if position and heading:
        return "either_alone"
    if position:
        return "position_only"
    return "heading_only" if heading else "joint_required"


def measured_geometry(scenario, pose, steering, obstacles, cache):
    key = scenario, pose, steering
    if key not in cache:
        obstacle, wall = arc_components(pose, steering, obstacles)
        clearance = min(obstacle, wall)
        assert abs(clearance - arc_clearance(pose, steering, obstacles)) < EPS
        error, bound = chord_check(pose, steering, obstacles, clearance)
        cache[key] = dict(
            audit_id=len(cache) + 1,
            scenario=scenario,
            pose_x=pose[0],
            pose_y=pose[1],
            pose_yaw=pose[2],
            steering=steering,
            obstacle=obstacle,
            wall=wall,
            clearance=clearance,
            chord_error=error,
            sagitta_bound=bound,
        )
    return cache[key]


def component_record(previous, paired, estimate, truth, obstacles, cache):
    steering = (
        0.0 if previous["rule"] == "straight" else float(previous["nominal_steering"])
    )
    values = {
        name: measured_geometry(previous["scenario"], pose, steering, obstacles, cache)
        for name, pose in hybrid_poses(estimate, truth).items()
    }
    margin = float(previous["margin"])
    checks = {k: classify(v["obstacle"], v["wall"], margin) for k, v in values.items()}
    passed = {k: v["clear"] for k, v in checks.items()}
    distances = {k: v["clearance"] for k, v in values.items()}
    position, heading = signed_losses(distances)
    optimistic = passed["FF"] and not passed["TT"]
    conservative = not passed["FF"] and passed["TT"]
    for key, value in [
        ("estimate_clear", passed["FF"]),
        ("truth_clear", passed["TT"]),
        ("optimistic", optimistic),
        ("conservative", conservative),
        ("accepted_physical_violation", passed["FF"] and distances["TT"] <= 0.5),
    ]:
        assert value == flag(previous[key]), key
    assert abs(checks["FF"]["slack"] - float(previous["estimate_slack"])) < EPS
    assert abs(checks["TT"]["slack"] - float(previous["truth_slack"])) < EPS
    assert abs(distances["TT"] - float(previous["truth_physical_clearance"])) < EPS
    yaw_difference = estimate[2] - truth[2]
    return dict(
        **{k: previous[k] for k in META + ["input_kind", "rule"]},
        estimate_clear=passed["FF"],
        truth_clear=passed["TT"],
        optimistic=optimistic,
        conservative=conservative,
        estimated_arc_gain=flag(paired["estimate_gain"]),
        optimistic_arc_gain=flag(paired["estimated_gain_truth_blocked"]),
        category=correction_category(passed),
        **{f"{k}_audit_id": v["audit_id"] for k, v in values.items()},
        **{f"{k}_distance": v for k, v in distances.items()},
        **{f"{k}_clear": v for k, v in passed.items()},
        distance_error=distances["FF"] - distances["TT"],
        position_loss=position,
        heading_loss=heading,
        position_error=math.dist(estimate[:2], truth[:2]),
        yaw_error_deg=abs(
            math.degrees(math.atan2(math.sin(yaw_difference), math.cos(yaw_difference)))
        ),
        truth_start_below_margin=float(previous["truth_center_clearance"])
        < margin - EPS,
        ambiguous=any(v["ambiguous"] for v in checks.values()),
    )


def paired_effects(rows, previous):
    by_rule = {r["rule"]: r for r in rows}
    assert len(rows) == 2 and set(by_rule) == {"straight", "commanded"}
    old, new = (by_rule[k] for k in ["straight", "commanded"])
    assert all(old[k] == new[k] for k in META + ["input_kind"])
    estimated_gain = new["FF_distance"] - old["FF_distance"]
    truth_gain = new["TT_distance"] - old["TT_distance"]
    error_delta = new["distance_error"] - old["distance_error"]
    position_delta = new["position_loss"] - old["position_loss"]
    heading_delta = new["heading_loss"] - old["heading_loss"]
    assert abs(estimated_gain - truth_gain - error_delta) < 1e-10
    assert abs(position_delta + heading_delta - error_delta) < 1e-10
    return dict(
        previous,
        straight_category=old["category"],
        commanded_category=new["category"],
        estimated_path_delta=estimated_gain,
        truth_path_delta=truth_gain,
        straight_distance_error=old["distance_error"],
        commanded_distance_error=new["distance_error"],
        path_error_delta=error_delta,
        position_loss_delta=position_delta,
        heading_loss_delta=heading_delta,
    )


def summarize(records):
    groups = defaultdict(list)
    for row in records:
        for cohort in ["all", row["cohort"]]:
            for subset in subsets(row):
                groups[
                    cohort, row["variant"], subset, row["input_kind"], row["rule"]
                ].append(row)
    summaries = []
    dimensions = ["cohort", "variant", "subset", "input_kind", "rule"]
    for key, full in sorted(groups.items()):
        for group in GROUPS:
            rows = full if group == "all" else [r for r in full if r[group]]
            record = dict(
                **dict(zip(dimensions, key)),
                group=group,
                checkpoints=len(rows),
                cases=len({tuple(r[k] for k in KEY) for r in rows}),
                **{
                    k: sum(r[k] for r in rows)
                    for k in [
                        "estimate_clear",
                        "truth_clear",
                        "optimistic",
                        "conservative",
                        "truth_start_below_margin",
                        "ambiguous",
                    ]
                },
                **{
                    k: sum(r["category"] == k for r in rows)
                    for k in [*CATEGORIES, "same_decision"]
                },
                position_larger_loss=sum(
                    r["position_loss"] > r["heading_loss"] + 1e-10 for r in rows
                ),
                negative_position_loss=sum(r["position_loss"] < -1e-10 for r in rows),
                negative_heading_loss=sum(r["heading_loss"] < -1e-10 for r in rows),
            )
            assert sum(record[k] for k in [*CATEGORIES, "same_decision"]) == len(rows)
            for field in [
                "distance_error",
                "position_loss",
                "heading_loss",
                "position_error",
                "yaw_error_deg",
            ]:
                values = [r[field] for r in rows]
                record[f"mean_{field}"] = statistics.fmean(values) if values else ""
                record[f"median_{field}"] = statistics.median(values) if values else ""
            summaries.append(record)
    return summaries


def pose(row):
    return tuple(float(row[f"pose_{a}"]) for a in ["x", "y", "yaw"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    if args.output.exists():
        parser.error("Output directory must be new")
    for name, expected in PINNED.items():
        assert digest(PRIOR / name) == expected, name
    prior = json.loads((PRIOR / "sources.json").read_text())
    assert (
        digest(ROOT / "analyze_commanded_recovery_arc.py")
        == prior["analysis"]["script_sha256"]
    )
    assert digest(SIMULATOR) == prior["analysis"]["simulator_sha256"]
    for name, expected in prior["analysis"]["helpers_sha256"].items():
        assert digest(ROOT / name) == expected, name
    map_path = ROOT / "inputs/obstacles.csv"
    assert digest(map_path) == prior["map_sha256"]
    maps = defaultdict(list)
    for row in read_rows(map_path):
        maps[row["scenario"]].append((float(row["x"]), float(row["y"])))
    accounting = read_rows(PRIOR / "accounting.csv")
    assert (
        len(accounting) == 648 and sum(flag(r["selected"]) for r in accounting) == 368
    )
    geometry = defaultdict(dict)
    for row in read_rows(PRIOR / "geometry.csv"):
        geometry[identity(row)][row["pose_kind"]] = row
    assert len(geometry) == 2120 and all(len(g) == 6 for g in geometry.values())
    paired = {
        (identity(r), r["input_kind"]): r for r in read_rows(PRIOR / "transitions.csv")
    }
    previous_decisions = read_rows(PRIOR / "decisions.csv")
    assert len(previous_decisions) == 8480 and len(paired) == 4240
    selected = {tuple(r[k] for k in KEY) for r in accounting if flag(r["selected"])}
    assert selected == {tuple(r[k] for k in KEY) for r in previous_decisions}
    cache, components, grouped = {}, [], defaultdict(list)
    for previous in previous_decisions:
        key = identity(previous), previous["input_kind"]
        poses = geometry[identity(previous)]
        estimated_row = poses[
            "recovery" if previous["input_kind"] == "actual" else "preview"
        ]
        row = component_record(
            previous,
            paired[key],
            pose(estimated_row),
            pose(poses["truth"]),
            maps[previous["scenario"]],
            cache,
        )
        if previous["input_kind"] == "actual":
            tf = poses["truth_xy_recovery_yaw"]
            distance = tf[
                (
                    "recovery_physical_clearance"
                    if previous["rule"] == "straight"
                    else "arc_clearance"
                )
            ]
            assert abs(row["TF_distance"] - float(distance)) < EPS
        components.append(row)
        grouped[key].append(row)
    effects = [paired_effects(rows, paired[key]) for key, rows in grouped.items()]
    helpers = set(prior["analysis"]["helpers_sha256"]) | {
        "analyze_commanded_recovery_arc.py",
        "analyze_arc_pose_components.py",
        "analyze_robust_arc_selection.py",
    }
    sources = dict(
        input_sha256=PINNED,
        map_sha256=digest(map_path),
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            simulator_sha256=digest(SIMULATOR),
            helpers_sha256={name: digest(ROOT / name) for name in sorted(helpers)},
        ),
        inherited_validation=dict(prior["inherited_validation"], raw_reloaded=False),
        audit=dict(
            checkpoints=len(geometry),
            component_rows=len(components),
            path_pairs=len(effects),
            unique_geometry_audits=len(cache),
            max_chord_error=max(r["chord_error"] for r in cache.values()),
            max_sagitta_bound=max(r["sagitta_bound"] for r in cache.values()),
            ambiguous_component_rows=sum(r["ambiguous"] for r in components),
        ),
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", accounting),
        ("components", components),
        ("geometry-audit", list(cache.values())),
        ("path-effects", effects),
        ("summary", summarize(components)),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(sources["audit"], indent=2))


if __name__ == "__main__":
    main()
