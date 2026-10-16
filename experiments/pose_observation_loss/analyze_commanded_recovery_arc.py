#!/usr/bin/env python3
"""Compare the recovery line and recorded-steering arc on pinned timeout poses."""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import EPS, KEY, subsets
from analyze_recovery_arcs import (
    LENGTH,
    WHEELBASE,
    arc_clearance,
    arc_point,
    chord_check,
)
from analyze_terminal_arc_live import SIMULATOR
from summarize_stopping_preview import save_rows
from verify_stopping_preview import segment_distance

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/joint-admission-geometry"
PROTOCOL = ROOT / "results/commanded-recovery-arc/protocol.md"
PINNED = {
    "accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "checkpoints.csv": "aa9207ab84acfacfb9f0a5aa3b4cd7eaf13a50100b92deb71e017539834d98f7",
    "geometry.csv": "1a3b1ba7e07654c09dcb1354f92f29e1f2ea39adb31c72a4c4164a70d638f08b",
    "decisions.csv": "77882af557b9bf528acbf9a00e39b17c71e6295ff1324cf4b8b3133c46dd55e6",
    "sources.json": "ccd59c809aa90313f8f96af8857dca180ce8b3b92600b57fb14757a0edca4da4",
}
POSES = {
    "sensed",
    "recovery",
    "preview",
    "truth_xy_sensed_yaw",
    "truth_xy_recovery_yaw",
    "truth",
}
META = KEY + [
    "time_s",
    "reasons",
    "joint_pending",
    "base_usable",
    "recovery_sample_count",
    "position_sigma",
    "observation_age",
    "margin",
    "zero_age_margin",
    "measured_speed",
    "nominal_steering",
    "nominal_target_speed",
    "uncapped_target_speed",
    "fusion_available",
    "goal_command_zero",
    "speed_geometry_zero",
    "full_raw_speed_distance",
    "truth_center_clearance",
    "truth_center_below_floor",
]
DECISION_FLAGS = [
    "estimate_clear",
    "truth_clear",
    "optimistic",
    "conservative",
    "estimate_zero_age_clear",
    "truth_zero_age_clear",
    "heading_evaluable",
    "heading_changes_truth",
    "accepted_physical_violation",
    "ambiguous",
]
TRANSITION_FLAGS = [
    "estimate_gain",
    "estimate_loss",
    "truth_gain",
    "truth_loss",
    "estimated_gain_truth_clear",
    "estimated_gain_truth_blocked",
    "estimated_gain_truth_gain",
    "estimated_loss_truth_loss",
    "estimated_gain_physical_violation",
    "ambiguous",
]


def flag(value):
    if isinstance(value, bool):
        return value
    assert value in {"True", "False", "true", "false"}, value
    return value.lower() == "true"


def arc_components(pose, steering, obstacles):
    """Exact point-obstacle and wall minima, kept separate for wall equality."""
    assert all(math.isfinite(v) for v in (*pose, steering))
    assert abs(steering) <= math.radians(35) + 1e-9
    x, y, yaw = pose
    start, end = (x, y), arc_point(pose, steering, LENGTH)
    curvature = math.tan(steering) / WHEELBASE
    extrema = [start, end]
    if curvature:
        sign = 1 if curvature > 0 else -1
        for heading in [0, math.pi / 2, math.pi, 3 * math.pi / 2]:
            progress = ((heading - yaw) * sign) % math.tau
            if progress <= abs(curvature) * LENGTH:
                extrema.append(arc_point(pose, steering, progress / abs(curvature)))
    wall = min(min(px, py, 50 - px, 50 - py) for px, py in extrema)
    obstacle = math.inf
    cosine, sine = math.cos(yaw), math.sin(yaw)
    for point in obstacles:
        if curvature == 0:
            distance = segment_distance(point, start, end)
        else:
            dx, dy = point[0] - x, point[1] - y
            forward, left = dx * cosine + dy * sine, -dx * sine + dy * cosine
            progress = math.atan2(curvature * forward, 1 - curvature * left) / curvature
            if 0 <= progress <= LENGTH:
                norm = math.hypot(curvature * forward, 1 - curvature * left)
                distance = abs(curvature * (forward**2 + left**2) - 2 * left) / (
                    norm + 1
                )
            else:
                distance = min(math.dist(point, start), math.dist(point, end))
        obstacle = min(obstacle, distance)
    return obstacle, wall


def classify(obstacle, wall, margin):
    return dict(
        clear=obstacle > margin and wall >= margin,
        slack=min(obstacle, wall) - margin,
        ambiguous=min(abs(obstacle - margin), abs(wall - margin)) <= EPS,
    )


def inspect_pose(row, obstacles, cache):
    pose = tuple(float(row[f"pose_{a}"]) for a in ["x", "y", "yaw"])
    steering = float(row["nominal_steering"])
    values = []
    for angle in [0.0, steering]:
        key = row["scenario"], pose, angle
        if key not in cache:
            obstacle, wall = arc_components(pose, angle, obstacles)
            exact = min(obstacle, wall)
            assert abs(exact - arc_clearance(pose, angle, obstacles)) < EPS
            error, bound = chord_check(pose, angle, obstacles, exact)
            cache[key] = obstacle, wall, error, bound
        values.append(cache[key])
    straight, commanded = values
    assert abs(straight[0] - float(row["recovery_obstacle"])) < EPS
    assert abs(straight[1] - float(row["recovery_wall"])) < EPS
    margin, floor = float(row["margin"]), float(row["zero_age_margin"])
    old, new = (classify(*v[:2], margin) for v in values)
    old_floor, new_floor = (classify(*v[:2], floor) for v in values)
    assert old["clear"] == flag(row["recovery_clear"])
    assert old_floor["clear"] == flag(row["recovery_zero_age_clear"])
    return dict(
        row,
        arc_obstacle=commanded[0],
        arc_wall=commanded[1],
        arc_clearance=min(commanded[:2]),
        arc_delta=min(commanded[:2]) - min(straight[:2]),
        arc_slack=new["slack"],
        arc_clear=new["clear"],
        arc_zero_age_clear=new_floor["clear"],
        arc_ambiguous=new["ambiguous"],
        arc_zero_age_ambiguous=new_floor["ambiguous"],
        arc_chord_error=commanded[2],
        arc_sagitta_bound=commanded[3],
        straight_chord_error=straight[2],
        straight_sagitta_bound=straight[3],
    )


def path_values(row, rule):
    prefix = "recovery" if rule == "straight" else "arc"
    physical = min(
        float(row["recovery_obstacle" if rule == "straight" else "arc_obstacle"]),
        float(row["recovery_wall" if rule == "straight" else "arc_wall"]),
    )
    return dict(
        clear=flag(row[prefix + "_clear"]),
        zero_age_clear=flag(row[prefix + "_zero_age_clear"]),
        slack=float(row[prefix + "_slack"]),
        physical=physical,
        ambiguous=flag(row[prefix + "_ambiguous"]),
    )


def decisions_at(common, rows):
    by_pose = {r["pose_kind"]: r for r in rows}
    assert len(rows) == len(POSES) and set(by_pose) == POSES
    records = []
    for rule in ["straight", "commanded"]:
        truth = path_values(by_pose["truth"], rule)
        intermediate = path_values(by_pose["truth_xy_recovery_yaw"], rule)
        for input_kind, pose in [("actual", "recovery"), ("preview", "preview")]:
            estimate = path_values(by_pose[pose], rule)
            records.append(
                dict(
                    **{k: common[k] for k in META},
                    rule=rule,
                    input_kind=input_kind,
                    estimate_clear=estimate["clear"],
                    truth_clear=truth["clear"],
                    estimate_zero_age_clear=estimate["zero_age_clear"],
                    truth_zero_age_clear=truth["zero_age_clear"],
                    estimate_slack=estimate["slack"],
                    truth_slack=truth["slack"],
                    optimistic=estimate["clear"] and not truth["clear"],
                    conservative=not estimate["clear"] and truth["clear"],
                    heading_evaluable=input_kind == "actual",
                    heading_changes_truth=(
                        intermediate["clear"] != truth["clear"]
                        if input_kind == "actual"
                        else False
                    ),
                    truth_physical_clearance=truth["physical"],
                    accepted_physical_violation=estimate["clear"]
                    and truth["physical"] <= 0.5,
                    ambiguous=estimate["ambiguous"] or truth["ambiguous"],
                )
            )
    return records


def transitions_at(decisions):
    assert len(decisions) == 4
    records = []
    for input_kind in ["actual", "preview"]:
        by_rule = {r["rule"]: r for r in decisions if r["input_kind"] == input_kind}
        assert set(by_rule) == {"straight", "commanded"}
        old, new = (by_rule[k] for k in ["straight", "commanded"])
        gain = new["estimate_clear"] and not old["estimate_clear"]
        loss = old["estimate_clear"] and not new["estimate_clear"]
        truth_gain = new["truth_clear"] and not old["truth_clear"]
        truth_loss = old["truth_clear"] and not new["truth_clear"]
        records.append(
            dict(
                **{k: old[k] for k in META},
                input_kind=input_kind,
                straight_estimate_clear=old["estimate_clear"],
                commanded_estimate_clear=new["estimate_clear"],
                straight_truth_clear=old["truth_clear"],
                commanded_truth_clear=new["truth_clear"],
                estimate_gain=gain,
                estimate_loss=loss,
                truth_gain=truth_gain,
                truth_loss=truth_loss,
                estimated_gain_truth_clear=gain and new["truth_clear"],
                estimated_gain_truth_blocked=gain and not new["truth_clear"],
                estimated_gain_truth_gain=gain and truth_gain,
                estimated_loss_truth_loss=loss and truth_loss,
                estimated_gain_physical_violation=gain
                and new["accepted_physical_violation"],
                ambiguous=old["ambiguous"] or new["ambiguous"],
            )
        )
    return records


def summarize(records, paired=False):
    groups = defaultdict(list)
    dimensions = ["cohort", "variant", "subset", "input_kind"]
    if not paired:
        dimensions.append("rule")
    for row in records:
        for cohort in ["all", row["cohort"]]:
            for subset in subsets(row):
                key = [cohort, row["variant"], subset, row["input_kind"]]
                if not paired:
                    key.append(row["rule"])
                groups[tuple(key)].append(row)
    result = []
    for key, rows in sorted(groups.items()):
        result.append(
            dict(
                **dict(zip(dimensions, key)),
                checkpoints=len(rows),
                cases=len({tuple(r[k] for k in KEY) for r in rows}),
                **{
                    k: sum(r[k] for r in rows)
                    for k in (TRANSITION_FLAGS if paired else DECISION_FLAGS)
                },
            )
        )
    return result


def identity(row):
    return tuple(row[k] for k in KEY + ["time_s"])


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
    assert digest(SIMULATOR) == prior["analysis"]["simulator_sha256"]
    assert (
        digest(ROOT / "analyze_joint_admission_geometry.py")
        == prior["analysis"]["script_sha256"]
    )
    for name, expected in prior["analysis"]["helpers_sha256"].items():
        assert digest(ROOT / name) == expected, name
    map_path = ROOT / "inputs/obstacles.csv"
    for cohort in prior["cohorts"].values():
        assert (
            digest(map_path) == cohort["provenance"]["inputs_sha256"]["obstacles.csv"]
        )
    maps = defaultdict(list)
    for row in read_rows(map_path):
        maps[row["scenario"]].append((float(row["x"]), float(row["y"])))
    accounting = read_rows(PRIOR / "accounting.csv")
    checkpoints = read_rows(PRIOR / "checkpoints.csv")
    previous_geometry = read_rows(PRIOR / "geometry.csv")
    assert (
        len(accounting) == 648 and sum(flag(r["selected"]) for r in accounting) == 368
    )
    assert len(checkpoints) == 2120 and len(previous_geometry) == 12720
    selected = {tuple(r[k] for k in KEY) for r in accounting if flag(r["selected"])}
    assert selected == {tuple(r[k] for k in KEY) for r in checkpoints}
    old_decisions = {
        (identity(r), r["input_kind"]): r
        for r in read_rows(PRIOR / "decisions.csv")
        if r["gate"] == "recovery"
    }
    cache, geometry, grouped = {}, [], defaultdict(list)
    for row in previous_geometry:
        audited = inspect_pose(row, maps[row["scenario"]], cache)
        geometry.append(audited)
        grouped[identity(row)].append(audited)
    decisions, transitions = [], []
    assert len(grouped) == len(checkpoints)
    for row in checkpoints:
        poses = grouped[identity(row)]
        assert all(all(r[k] == row[k] for k in META) for r in poses)
        items = decisions_at(row, poses)
        for item in items:
            if item["rule"] == "straight":
                previous = old_decisions[identity(item), item["input_kind"]]
                for key in [
                    "estimate_clear",
                    "truth_clear",
                    "optimistic",
                    "conservative",
                    "heading_evaluable",
                    "heading_changes_truth",
                    "accepted_physical_violation",
                    "ambiguous",
                ]:
                    assert item[key] == flag(previous[key]), key
                for key in [
                    "estimate_slack",
                    "truth_slack",
                    "truth_physical_clearance",
                ]:
                    assert abs(item[key] - float(previous[key])) < EPS, key
        decisions.extend(items)
        transitions.extend(transitions_at(items))
    sources = dict(
        input_sha256=PINNED,
        map_sha256=digest(map_path),
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            simulator_sha256=digest(SIMULATOR),
            helpers_sha256={
                name: digest(ROOT / name)
                for name in sorted(
                    set(prior["analysis"]["helpers_sha256"])
                    | {"analyze_joint_admission_geometry.py"}
                )
            },
        ),
        inherited_validation=dict(
            cases=sum(c["selected_cases"] for c in prior["cohorts"].values()),
            intervals=sum(c["selected_intervals"] for c in prior["cohorts"].values()),
            raw_reloaded=False,
        ),
        audit=dict(
            pose_rows=len(geometry),
            checkpoints=len(checkpoints),
            unique_geometry_audits=len(cache),
            max_chord_error=max(v[2] for v in cache.values()),
            max_sagitta_bound=max(v[3] for v in cache.values()),
        ),
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", accounting),
        ("geometry", geometry),
        ("decisions", decisions),
        ("transitions", transitions),
        ("summary", summarize(decisions)),
        ("transition-summary", summarize(transitions, paired=True)),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(sources["audit"], indent=2))


if __name__ == "__main__":
    main()
