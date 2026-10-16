#!/usr/bin/env python3
"""Audit recovery-line and speed admission geometry in every pinned timeout."""

import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path

from analyze_epoch_divergence import release_inputs
from analyze_epoch_preview import digest, read_rows
from analyze_recovery_arcs import arc_point
from analyze_terminal_arc_live import COHORTS, SIMULATOR
from summarize_stopping_preview import save_rows
from verify_stopping_preview import exact_speed_distance, segment_distance

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/terminal-arc-live"
KEY = ["cohort", "variant", "mode", "seed", "scenario", "planner"]
DT = 0.02
EPS = 2e-8
PINNED = {
    "cases.csv": "5dfb2eea39b95d8acd93843359fb71bc80944aeac25f81f6e643bdc6018d7b95",
    "pairs.csv": "0a2015799e7ebd4e062c192752cccdbbc1d602bc76bc64ae79f02e3496336a1c",
    "sources.json": "9c53f197f3d26d1c672526b6f7bf82083f0971ee4c6857d5335d408284cdfb54",
}
MASKS = dict(
    actual=set(),
    without_recovery={"recovery_geometry"},
    without_speed={"positive_command"},
    without_raw_gates={"recovery_geometry", "positive_command"},
    without_preview={"stop", "terminal"},
    base_only={"recovery_geometry", "positive_command", "stop", "terminal"},
)


def usable(ready):
    return all(ready[k] for k in ["stationary", "fresh", "samples"])


def tail_readiness(track):
    start = max(1, len(track) - 1500)
    counts, runs, maxima = Counter(), Counter(), Counter()
    choices = defaultdict(list)
    choices[len(track) - 1].append("endpoint")
    epochs = set()
    travel = 0.0
    for i in range(start, len(track)):
        before, row = track[i - 1], track[i]
        travel += math.hypot(
            float(row["x"]) - float(before["x"]), float(row["y"]) - float(before["y"])
        )
        counts["wait"] += (
            row["guard_active"] == "true" or row["safety_paused"] == "true"
        )
        counts["resumes"] += row["resumed"] == "true"
        route, ready = release_inputs(before, row)
        if route != "joint":
            runs.clear()
            continue
        counts["eligible"] += 1
        for name, passed in ready.items():
            counts["fail_" + name] += not passed
        if usable(ready):
            counts["base_usable"] += 1
            epoch = round(float(row["time_s"]) - float(row["observation_age"]), 6)
            if epoch not in epochs:
                choices[i].append("first_usable_epoch")
                epochs.add(epoch)
            if abs(float(row["observation_age"]) - 0.5) < 1e-8:
                choices[i].append("age_0p5")
        for name, omitted in MASKS.items():
            passed = all(value for key, value in ready.items() if key not in omitted)
            runs[name] = runs[name] + 1 if passed else 0
            maxima[name] = max(maxima[name], runs[name])
            counts["ready_" + name] += passed
    record = dict(
        window_start_s=track[start - 1]["time_s"],
        window_end_s=track[-1]["time_s"],
        travel=travel,
        wait_s=round(counts["wait"] * DT, 6),
        resumes=counts["resumes"],
        eligible_s=round(counts["eligible"] * DT, 6),
        base_usable_s=round(counts["base_usable"] * DT, 6),
        usable_epochs=len(epochs),
        **{
            name + "_fail_s": round(counts["fail_" + name] * DT, 6)
            for name in [
                "stationary",
                "fresh",
                "samples",
                "recovery_geometry",
                "positive_command",
                "stop",
                "terminal",
            ]
        },
        **{name + "_ready_s": round(counts["ready_" + name] * DT, 6) for name in MASKS},
        **{name + "_max_ready_s": round(maxima[name] * DT, 6) for name in MASKS},
    )
    return record, choices


def geometry(pose, steering, margin, obstacles):
    start = pose[:2]
    end = pose[0] + 0.6 * math.cos(pose[2]), pose[1] + 0.6 * math.sin(pose[2])
    first = arc_point(pose, steering, 0.05)

    def wall(point):
        return min(point[0], point[1], 50 - point[0], 50 - point[1])

    center_obstacle = min((math.dist(start, p) for p in obstacles), default=math.inf)
    line_obstacle = min(
        (segment_distance(p, start, end) for p in obstacles), default=math.inf
    )
    line_wall = min(wall(start), wall(end))
    first_clearance = min(
        wall(start),
        wall(first),
        *(segment_distance(p, start, first) for p in obstacles),
    )
    return dict(
        center_clearance=min(center_obstacle, wall(start)),
        recovery_obstacle=line_obstacle,
        recovery_wall=line_wall,
        recovery_slack=min(line_obstacle - margin, line_wall - margin),
        recovery_clear=line_obstacle > margin and line_wall >= margin,
        recovery_ambiguous=min(abs(line_obstacle - margin), abs(line_wall - margin))
        <= EPS,
        speed_first_clearance=first_clearance,
        speed_slack=first_clearance - margin,
        speed_geometry_clear=first_clearance > margin,
        speed_ambiguous=abs(first_clearance - margin) <= EPS,
        recovery_physical_clearance=min(line_obstacle, line_wall),
    )


def checkpoint(before, row, obstacles):
    sensed = tuple(
        float(before[a]) + float(row[f"noise_{a}"]) + float(row[f"bias_{a}"])
        for a in ["x", "y", "yaw"]
    )
    recovery = tuple(float(row[f"recovery_{a}"]) for a in ["x", "y", "yaw"])
    if int(row["recovery_sample_count"]) == 0:
        assert math.dist(recovery[:2], sensed[:2]) < EPS
        assert (
            abs(
                math.atan2(
                    math.sin(recovery[2] - sensed[2]), math.cos(recovery[2] - sensed[2])
                )
            )
            < EPS
        )
    preview = tuple(float(row[f"preview_pose_{a}"]) for a in ["x", "y", "yaw"])
    truth = tuple(float(before[a]) for a in ["x", "y", "yaw"])
    poses = dict(
        sensed=sensed,
        recovery=recovery,
        preview=preview,
        truth_xy_sensed_yaw=(*truth[:2], sensed[2]),
        truth_xy_recovery_yaw=(*truth[:2], recovery[2]),
        truth=truth,
    )
    sigma, age = float(row["effective_position_sigma"]), float(row["observation_age"])
    floor = 0.6 + 2 * math.sqrt(2) * sigma
    margin = floor + 0.02 * age
    assert abs(margin - float(row["speed_margin"])) < EPS
    steering = float(row["nominal_steering"])
    values, cache = {}, {}
    for name, pose in poses.items():
        if pose not in cache:
            cache[pose] = geometry(pose, steering, margin, obstacles)
        values[name] = dict(
            pose_kind=name,
            pose_x=pose[0],
            pose_y=pose[1],
            pose_yaw=pose[2],
            position_error=math.dist(pose[:2], truth[:2]),
            yaw_error_deg=abs(
                math.degrees(
                    math.atan2(
                        math.sin(pose[2] - truth[2]), math.cos(pose[2] - truth[2])
                    )
                )
            ),
            **cache[pose],
        )
        values[name]["recovery_zero_age_clear"] = (
            values[name]["recovery_obstacle"] > floor
            and values[name]["recovery_wall"] >= floor
        )
        values[name]["speed_zero_age_clear"] = (
            values[name]["speed_first_clearance"] > floor
        )
    full_speed = exact_speed_distance(before, row, obstacles)
    assert abs(full_speed - float(row["speed_free_distance"])) < 1e-8
    if not values["sensed"]["speed_ambiguous"]:
        assert (full_speed > 0) == values["sensed"]["speed_geometry_clear"]
    assert (
        abs(values["recovery"]["recovery_obstacle"] - float(row["recovery_clearance"]))
        < EPS
    )
    route, ready = release_inputs(before, row)
    if route == "joint":
        if not values["recovery"]["recovery_ambiguous"]:
            assert values["recovery"]["recovery_clear"] == ready["recovery_geometry"]
        assert ready["positive_command"] == (float(row["nominal_target_speed"]) > 0)
    if not values["sensed"]["speed_ambiguous"]:
        assert (
            float(row["uncapped_target_speed"]) > 0
            and values["sensed"]["speed_geometry_clear"]
        ) == (float(row["nominal_target_speed"]) > 0)
    c = values["truth"]["center_clearance"]
    record = dict(
        joint_pending=route == "joint",
        base_usable=route == "joint" and usable(ready),
        recovery_sample_count=row["recovery_sample_count"],
        recovery_averaged=int(row["recovery_sample_count"]) > 0,
        position_sigma=sigma,
        observation_age=age,
        margin=margin,
        zero_age_margin=floor,
        measured_speed=row["measured_speed"],
        nominal_steering=steering,
        nominal_target_speed=row["nominal_target_speed"],
        uncapped_target_speed=row["uncapped_target_speed"],
        fusion_available=row["epoch_fusion_available"],
        full_raw_speed_distance=full_speed,
        goal_command_zero=float(row["uncapped_target_speed"]) == 0,
        speed_geometry_zero=full_speed == 0,
        truth_center_clearance=c,
        truth_center_below_floor=c < floor - EPS,
        truth_center_below_current_margin=c < margin - EPS,
        truth_center_floor_ambiguous=abs(c - floor) <= EPS,
        truth_recovery_at_floor_clear=values["truth"]["recovery_zero_age_clear"],
        truth_speed_at_floor_clear=values["truth"]["speed_zero_age_clear"],
    )
    return record, list(values.values())


def decision_records(common, geometry_rows):
    by_pose = {r["pose_kind"]: r for r in geometry_rows}
    records = []
    for gate, actual, intermediate in [
        ("recovery", "recovery", "truth_xy_recovery_yaw"),
        ("speed", "sensed", "truth_xy_sensed_yaw"),
    ]:

        def clear(value):
            return (
                value["recovery_clear"]
                if gate == "recovery"
                else value["speed_geometry_clear"] and not common["goal_command_zero"]
            )

        truth = clear(by_pose["truth"])
        for input_kind, pose in [("actual", actual), ("preview", "preview")]:
            item = by_pose[pose]
            estimated = clear(item)
            physical = by_pose["truth"][
                (
                    "recovery_physical_clearance"
                    if gate == "recovery"
                    else "speed_first_clearance"
                )
            ]
            records.append(
                dict(
                    common,
                    gate=gate,
                    input_kind=input_kind,
                    estimate_clear=estimated,
                    truth_clear=truth,
                    estimate_slack=item[
                        "recovery_slack" if gate == "recovery" else "speed_slack"
                    ],
                    truth_slack=by_pose["truth"][
                        "recovery_slack" if gate == "recovery" else "speed_slack"
                    ],
                    optimistic=estimated and not truth,
                    conservative=not estimated and truth,
                    heading_evaluable=input_kind == "actual",
                    heading_changes_truth=(
                        (clear(by_pose[intermediate]) != truth)
                        if input_kind == "actual"
                        else False
                    ),
                    truth_physical_clearance=physical,
                    accepted_physical_violation=estimated and physical <= 0.5,
                    ambiguous=item[gate + "_ambiguous"]
                    or by_pose["truth"][gate + "_ambiguous"],
                )
            )
    return records


def subsets(row):
    return ["all"] + [
        name
        for name in ["first_usable_epoch", "age_0p5", "endpoint"]
        if name in row["reasons"].split("+")
    ]


def summarize(decisions):
    groups = defaultdict(list)
    for row in decisions:
        for cohort in ["all", row["cohort"]]:
            for subset in subsets(row):
                groups[
                    cohort, row["variant"], subset, row["gate"], row["input_kind"]
                ].append(row)
    result = []
    for (cohort, variant, subset, gate, input_kind), rows in sorted(groups.items()):
        result.append(
            dict(
                cohort=cohort,
                variant=variant,
                subset=subset,
                gate=gate,
                input_kind=input_kind,
                checkpoints=len(rows),
                cases=len({tuple(r[k] for k in KEY) for r in rows}),
                **{
                    k: sum(r[k] for r in rows)
                    for k in [
                        "estimate_clear",
                        "truth_clear",
                        "optimistic",
                        "conservative",
                        "ambiguous",
                        "heading_evaluable",
                        "heading_changes_truth",
                        "accepted_physical_violation",
                        "truth_center_below_floor",
                        "truth_center_below_current_margin",
                        "goal_command_zero",
                    ]
                },
            )
        )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    if args.output.exists():
        parser.error("Output directory must be new")
    for name, expected in PINNED.items():
        assert digest(PRIOR / name) == expected
    prior = json.loads((PRIOR / "sources.json").read_text())
    assert digest(SIMULATOR) == prior["analysis"]["simulator_sha256"]
    for name, expected in prior["analysis"]["helpers_sha256"].items():
        assert digest(ROOT / name) == expected, name
    cases = read_rows(PRIOR / "cases.csv")
    selected = [r for r in cases if r["status"] == "timeout"]
    assert len(cases) == 648 and len(selected) == 368
    accounting = [dict(r, selected=r["status"] == "timeout") for r in cases]
    maps = defaultdict(list)
    for r in read_rows(ROOT / "inputs/obstacles.csv"):
        maps[r["scenario"]].append((float(r["x"]), float(r["y"])))
    readiness, checkpoints, pose_geometry, decisions = [], [], [], []
    sources = dict(prior_sha256=PINNED, cohorts={})
    for cohort in COHORTS:
        pinned = prior["cohorts"][cohort]
        run = (
            args.raw_root / cohort if args.raw_root else Path(pinned["path"])
        ).resolve()
        provenance = json.loads((run / "provenance.json").read_text())
        for field in [
            "variants",
            "seeds",
            "noise_correlation_time_s",
            "simulator_sha256",
            "inputs_sha256",
            "inputs_by_variant_sha256",
        ]:
            assert provenance[field] == pinned["provenance"][field]
        assert (
            digest(ROOT / "inputs/obstacles.csv")
            == provenance["inputs_sha256"]["obstacles.csv"]
        )
        groups = defaultdict(list)
        for case in selected:
            if case["cohort"] == cohort:
                groups[case["variant"], case["mode"], case["seed"]].append(case)
        hashes = {}
        intervals = 0
        for (variant, mode, seed), group in sorted(groups.items()):
            name = f"{variant}-{mode}-{seed}/trajectories.csv"
            path = run / name
            hashes[name] = digest(path)
            assert hashes[name] == pinned["raw_sha256"][name]
            keys = {(r["scenario"], r["planner"]) for r in group}
            tracks = defaultdict(list)
            with path.open() as stream:
                for row in csv.DictReader(stream):
                    key = row["scenario"], row["planner"]
                    if key in keys:
                        tracks[key].append(row)
            for case in group:
                identity = {k: case[k] for k in KEY}
                track = tracks[case["scenario"], case["planner"]]
                assert float(track[-1]["time_s"]) == 180
                intervals += len(track) - 1
                record, choices = tail_readiness(track)
                readiness.append(
                    dict(
                        **identity,
                        first_goal_entry_s=case["first_goal_entry_s"],
                        final_goal_distance=case["goal_distance"],
                        **record,
                    )
                )
                for index, reasons in sorted(choices.items()):
                    row, before = track[index], track[index - 1]
                    record, geometry_rows = checkpoint(
                        before, row, maps[case["scenario"]]
                    )
                    common = dict(
                        **identity,
                        time_s=row["time_s"],
                        reasons="+".join(reasons),
                        **record,
                    )
                    checkpoints.append(common)
                    pose_geometry.extend(dict(common, **r) for r in geometry_rows)
                    decisions.extend(decision_records(common, geometry_rows))
            del tracks
        sources["cohorts"][cohort] = dict(
            path=str(run),
            provenance=provenance,
            raw_sha256=hashes,
            selected_cases=sum(len(g) for g in groups.values()),
            selected_intervals=intervals,
        )
        print(
            f"{cohort}: {sum(len(g) for g in groups.values())} timeouts, {intervals} pinned intervals",
            flush=True,
        )
    sources["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        simulator_sha256=digest(SIMULATOR),
        protocol_sha256=digest(ROOT / "results/joint-admission-geometry/protocol.md"),
        helpers_sha256={
            name: digest(ROOT / name)
            for name in [
                "analyze_epoch_divergence.py",
                "analyze_epoch_preview.py",
                "analyze_filter_stalls.py",
                "analyze_recovery_stalls.py",
                "analyze_recovery_arcs.py",
                "analyze_terminal_arc_live.py",
                "verify_stopping_preview.py",
                "verify_joint_recovery.py",
                "summarize_stopping_preview.py",
            ]
        },
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", accounting),
        ("readiness", readiness),
        ("checkpoints", checkpoints),
        ("geometry", pose_geometry),
        ("decisions", decisions),
        ("summary", summarize(decisions)),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")


if __name__ == "__main__":
    main()
