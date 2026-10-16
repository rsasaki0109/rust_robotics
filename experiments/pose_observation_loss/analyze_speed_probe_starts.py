#!/usr/bin/env python3
"""Separate speed-probe start and first-chord blocks at fixed recorded inputs."""

import argparse
from collections import defaultdict
import csv
from itertools import product
import json
import math
from pathlib import Path

from analyze_commanded_recovery_arc import flag
from analyze_correction_gate_boundaries import FOCUS, KINDS
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import KEY
from analyze_terminal_arc_live import COHORTS, SIMULATOR
from summarize_stopping_preview import save_rows
from verify_stopping_preview import segment_distance

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/correction-gate-boundaries"
PROTOCOL = ROOT / "results/speed-probe-starts/protocol.md"
EPS = 2e-8
LENGTH = 0.05
POSES = ["raw", "recovery", "preview", "truth"]
ESTIMATES = POSES[:-1]
CATEGORIES = [
    "start_obstacle",
    "start_wall",
    "start_both",
    "first_chord_obstacle",
    "first_chord_wall",
    "first_chord_both",
    "positive_first_chord",
]
RELATIONS = [
    "both_clear",
    "estimate_clear_truth_blocks",
    "estimate_blocks_truth_clear",
    "both_block",
]
MOVES = ["held_clear", "became_blocked", "became_clear", "held_blocked"]
SUBSETS = ["all", "raw_zero_loss", "command_only_zero_loss"]
PINNED = {
    "accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "events.csv": "6e9f6377ed20fe308cf5e7de06fe13c06d841b6041005c6432e872cbb0f08599",
    "states.csv": "28164ead18d19a9d10035b06055d4d47b2d818fc715772919b58ac31187117f7",
    "transitions.csv": "2b5f7eead9dc621cb337d787a3a5f8c7dbcde1e8b80f3dbeb480ac1e5bd8f26d",
    "changes.csv": "8df3f04aa88cef0b2801915acaee664b2988eb3d5746bb4177be77d7e7a96070",
    "geometry.csv": "909750623c861f06baf9e38cf40f773cfbe1ba462c710ac84bbb711114d25254",
    "sources.json": "35dfa2a6e5c50c4790ea1e654a8135dc4366c546a0d466645e77f6b688164d81",
}


def first_endpoint(pose, steering):
    x, y, yaw = pose
    curvature = math.tan(steering) / 0.4
    if abs(curvature) < 1e-9:
        return x + LENGTH * math.cos(yaw), y + LENGTH * math.sin(yaw)
    half = curvature * LENGTH / 2
    chord = LENGTH * math.sin(half) / half
    return x + chord * math.cos(yaw + half), y + chord * math.sin(yaw + half)


def classification(start_obstacle, start_wall, chord_obstacle, chord_wall):
    if start_obstacle or start_wall:
        return (
            "start_both"
            if start_obstacle and start_wall
            else "start_obstacle" if start_obstacle else "start_wall"
        )
    if chord_obstacle or chord_wall:
        return (
            "first_chord_both"
            if chord_obstacle and chord_wall
            else "first_chord_obstacle" if chord_obstacle else "first_chord_wall"
        )
    return "positive_first_chord"


def truth_relation(estimate_clear, truth_clear):
    return (
        "both_clear"
        if estimate_clear and truth_clear
        else (
            "estimate_clear_truth_blocks"
            if estimate_clear
            else "estimate_blocks_truth_clear" if truth_clear else "both_block"
        )
    )


def query(pose, steering, margin, obstacles):
    assert margin >= 0 and all(math.isfinite(v) for v in [*pose, steering, margin])
    start, end = tuple(pose[:2]), first_endpoint(pose, steering)
    start_distance = chord_distance = math.inf
    start_id = chord_id = ""
    start_witness = chord_witness = ("", "")
    for i, point in enumerate(obstacles, 1):
        distance = math.dist(start, point)
        if distance < start_distance:
            start_distance, start_id, start_witness = distance, i, point
        distance = segment_distance(point, start, end)
        if distance < chord_distance:
            chord_distance, chord_id, chord_witness = distance, i, point
    wall_start = min(start[0], start[1], 50 - start[0], 50 - start[1])
    wall_chord = min(wall_start, end[0], end[1], 50 - end[0], 50 - end[1])
    assert chord_distance <= start_distance + EPS and wall_chord <= wall_start
    gaps = dict(
        start_obstacle_gap=start_distance - margin,
        start_wall_gap=wall_start - margin,
        first_chord_obstacle_gap=chord_distance - margin,
        first_chord_wall_gap=wall_chord - margin,
    )
    blocked = [gaps[k] <= 0 for k in gaps]
    category = classification(*blocked)
    return dict(
        x=pose[0],
        y=pose[1],
        yaw=pose[2],
        steering=steering,
        margin=margin,
        first_end_x=end[0],
        first_end_y=end[1],
        start_obstacle_id=start_id,
        start_obstacle_x=start_witness[0],
        start_obstacle_y=start_witness[1],
        first_chord_obstacle_id=chord_id,
        first_chord_obstacle_x=chord_witness[0],
        first_chord_obstacle_y=chord_witness[1],
        **gaps,
        category=category,
        first_chord_clear=category == "positive_first_chord",
        near_boundary=any(abs(v) <= EPS for v in gaps.values()),
    )


def check_source_free(record, distance):
    assert 0 <= distance <= 2
    assert abs(distance / 0.05 - round(distance / 0.05)) < EPS
    assert record["first_chord_clear"] == (distance >= 0.05), (record, distance)


def source_subsets(change):
    zero = (
        change["phase"] == "post"
        and change["label"] == "joint_lost"
        and change["command_zero_reason"] == "speed_cap_zero"
    )
    return zero, zero and int(change["failed_mask"]) == 16


def pair_record(change, kind, pre, post, pre_truth, post_truth, raw_post):
    zero, command_only = source_subsets(change)
    a, b = pre["first_chord_clear"], post["first_chord_clear"]
    return dict(
        transition_id=change["transition_id"],
        event_id=change["event_id"],
        phase=change["phase"],
        source_label=change["label"],
        end_offset_steps=change["end_offset_steps"],
        pose_kind=kind,
        pre_state_id=change["pre_state_id"],
        post_state_id=change["post_state_id"],
        pre_pose_id=pre["pose_id"],
        post_pose_id=post["pose_id"],
        pre_truth_pose_id=pre_truth["pose_id"],
        post_truth_pose_id=post_truth["pose_id"],
        source_failed_mask=change["failed_mask"],
        source_command_reason=change["command_zero_reason"],
        raw_zero_loss=zero,
        command_only_zero_loss=command_only,
        pre_category=pre["category"],
        post_category=post["category"],
        pre_clear=a,
        post_clear=b,
        change=(
            "held_clear"
            if a and b
            else "became_blocked" if a else "became_clear" if b else "held_blocked"
        ),
        pre_truth_clear=pre_truth["first_chord_clear"],
        post_truth_clear=post_truth["first_chord_clear"],
        post_truth_relation=truth_relation(b, post_truth["first_chord_clear"]),
        post_raw_category=raw_post["category"],
        new_clear_from_raw=not raw_post["first_chord_clear"] and b,
    )


def summarize(poses, pairs, events):
    E = {e["event_id"]: e for e in events}
    sg, pg = defaultdict(list), defaultdict(list)
    for r in poses:
        for cohort, kind in product(
            ["all", r["cohort"]], ["all", *r["source_event_kinds"].split("+")]
        ):
            sg[cohort, r["variant"], kind, r["pose_kind"]].append(r)
    for r in pairs:
        e = E[r["event_id"]]
        subsets = [
            "all",
            *(["raw_zero_loss"] if r["raw_zero_loss"] else []),
            *(["command_only_zero_loss"] if r["command_only_zero_loss"] else []),
        ]
        for cohort, kind, label, subset in product(
            ["all", e["cohort"]],
            ["all", e["event_kind"]],
            ["all", r["source_label"]],
            subsets,
        ):
            pg[
                cohort, e["variant"], kind, r["phase"], label, r["pose_kind"], subset
            ].append(r)
    state_summary = []
    for cohort, variant, kind, pose in product(
        ["all", *COHORTS], ["epoch", "curved"], ["all", *KINDS], POSES
    ):
        rows = sg[cohort, variant, kind, pose]
        counts = {k: sum(r["category"] == k for r in rows) for k in CATEGORIES}
        assert sum(counts.values()) == len(rows)
        state_summary.append(
            dict(
                cohort=cohort,
                variant=variant,
                event_kind=kind,
                pose_kind=pose,
                states=len(rows),
                cases=len({tuple(r[k] for k in KEY) for r in rows}),
                **counts,
                near_boundary=sum(r["near_boundary"] for r in rows),
                **{k: sum(r["truth_relation"] == k for r in rows) for k in RELATIONS},
            )
        )
    pair_summary = []
    dimensions = [
        "cohort",
        "variant",
        "event_kind",
        "phase",
        "source_label",
        "pose_kind",
        "subset",
    ]
    for key in product(
        ["all", *COHORTS],
        ["epoch", "curved"],
        ["all", *KINDS],
        ["immediate", "post"],
        ["all", *FOCUS],
        ESTIMATES,
        SUBSETS,
    ):
        rows = pg[key]
        categories = {k: sum(r["post_category"] == k for r in rows) for k in CATEGORIES}
        moves = {k: sum(r["change"] == k for r in rows) for k in MOVES}
        relations = {
            k: sum(r["post_truth_relation"] == k for r in rows) for k in RELATIONS
        }
        assert (
            sum(categories.values())
            == sum(moves.values())
            == sum(relations.values())
            == len(rows)
        )
        pair_summary.append(
            dict(
                **dict(zip(dimensions, key)),
                pairs=len(rows),
                opportunities=len({r["event_id"] for r in rows}),
                cases=len({tuple(E[r["event_id"]][k] for k in KEY) for r in rows}),
                **categories,
                **moves,
                **relations,
                new_clear_from_raw=sum(r["new_clear_from_raw"] for r in rows),
                new_clear_truth_blocks=sum(
                    r["new_clear_from_raw"] and not r["post_truth_clear"] for r in rows
                ),
                new_clear_truth_clear=sum(
                    r["new_clear_from_raw"] and r["post_truth_clear"] for r in rows
                ),
            )
        )
    return state_summary, pair_summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for name, sha in PINNED.items():
        assert digest(PRIOR / name) == sha, name
    prior = json.loads((PRIOR / "sources.json").read_text())
    helpers = dict(prior["analysis"]["helpers_sha256"])
    helpers["analyze_correction_gate_boundaries.py"] = prior["analysis"][
        "script_sha256"
    ]
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    assert digest(SIMULATOR) == prior["analysis"]["simulator_sha256"]
    assert digest(ROOT / "inputs/obstacles.csv") == prior["map_sha256"]
    maps = defaultdict(list)
    for r in read_rows(ROOT / "inputs/obstacles.csv"):
        maps[r["scenario"]].append((float(r["x"]), float(r["y"])))
    accounting, events, changes, geometry = (
        read_rows(PRIOR / n)
        for n in ["accounting.csv", "events.csv", "changes.csv", "geometry.csv"]
    )
    assert (
        len(accounting) == 648
        and len(events) == 2583
        and len(changes) == 6187
        and len(geometry) == 6610
    )
    E = {e["event_id"]: e for e in events}
    G = {g["state_id"]: g for g in geometry}
    cases, state_events, event_states, expected_pairs = (
        {},
        defaultdict(set),
        defaultdict(set),
        defaultdict(list),
    )
    for c in changes:
        e = E[c["event_id"]]
        case = tuple(e[k] for k in KEY)
        for sid in [c["pre_state_id"], c["post_state_id"]]:
            assert sid not in cases or cases[sid] == case
            cases[sid] = case
            state_events[sid].add(c["event_id"])
            event_states[c["event_id"]].add(sid)
        expected_pairs[c["transition_id"]] = c
    assert set(cases) == set(G)
    scanned_transitions = 0
    matched = set()
    with (PRIOR / "transitions.csv").open() as stream:
        for t in csv.DictReader(stream):
            scanned_transitions += 1
            if flag(t["detailed"]):
                c = expected_pairs[t["transition_id"]]
                for k in [
                    "transition_id",
                    "event_id",
                    "phase",
                    "end_offset_steps",
                    "label",
                    "pre_state_id",
                    "post_state_id",
                ]:
                    assert c[k] == t[k]
                matched.add(t["transition_id"])
    assert scanned_transitions == 168708 and matched == set(expected_pairs)
    states = {}
    scanned_states = 0
    with (PRIOR / "states.csv").open() as stream:
        for s in csv.DictReader(stream):
            scanned_states += 1
            if s["state_id"] in G:
                assert s["source_interval_id"] == G[s["state_id"]]["source_interval_id"]
                assert (
                    abs(
                        float(s["speed_free_distance"])
                        - float(G[s["state_id"]]["speed_free_exact"])
                    )
                    < EPS
                )
                states[s["state_id"]] = s
    assert scanned_states == 171082 and set(states) == set(G)
    poses = []
    lookup = {}
    event_pairs = defaultdict(list)
    for sid in sorted(states, key=int):
        s = states[sid]
        case = dict(zip(KEY, cases[sid]))
        records = {}
        source_kinds = sorted({E[eid]["event_kind"] for eid in state_events[sid]})
        for kind in POSES:
            pose = tuple(float(s[kind + "_" + a]) for a in ["x", "y", "yaw"])
            r = dict(
                pose_id=len(poses) + len(records) + 1,
                state_id=sid,
                source_interval_id=s["source_interval_id"],
                **case,
                source_event_kinds="+".join(source_kinds),
                time_s=s["time_s"],
                truth_time_s=s["truth_time_s"],
                pose_kind=kind,
                **query(
                    pose,
                    float(s["nominal_steering"]),
                    float(s["speed_margin"]),
                    maps[case["scenario"]],
                ),
            )
            records[kind] = r
        check_source_free(records["raw"], float(s["speed_free_distance"]))
        for kind in POSES:
            r = records[kind]
            r["truth_pose_id"] = records["truth"]["pose_id"]
            r["truth_relation"] = truth_relation(
                r["first_chord_clear"], records["truth"]["first_chord_clear"]
            )
            r["source_raw_free_distance"] = (
                float(s["speed_free_distance"]) if kind == "raw" else ""
            )
            poses.append(r)
            lookup[sid, kind] = r
    pairs = []
    start_blocks = []
    for c in changes:
        pre, post = c["pre_state_id"], c["post_state_id"]
        zero, command_only = source_subsets(c)
        if zero:
            assert (
                float(states[post]["speed_free_distance"]) == 0
                and not lookup[post, "raw"]["first_chord_clear"]
            )
            e = E[c["event_id"]]
            block = dict(
                transition_id=c["transition_id"],
                event_id=c["event_id"],
                **{k: e[k] for k in KEY},
                post_state_id=post,
                time_s=states[post]["time_s"],
                command_only_zero_loss=command_only,
                source_failed_mask=c["failed_mask"],
            )
            for kind in POSES:
                r = lookup[post, kind]
                block[kind + "_pose_id"] = r["pose_id"]
                block[kind + "_category"] = r["category"]
                block[kind + "_clear"] = r["first_chord_clear"]
            for field in [
                "start_obstacle_gap",
                "start_wall_gap",
                "first_chord_obstacle_gap",
                "first_chord_wall_gap",
                "near_boundary",
            ]:
                block["raw_" + field] = lookup[post, "raw"][field]
            start_blocks.append(block)
        for kind in ESTIMATES:
            r = pair_record(
                c,
                kind,
                lookup[pre, kind],
                lookup[post, kind],
                lookup[pre, "truth"],
                lookup[post, "truth"],
                lookup[post, "raw"],
            )
            pairs.append(r)
            event_pairs[c["event_id"]].append(r)
    assert (
        len(start_blocks) == 550
        and sum(r["command_only_zero_loss"] for r in start_blocks) == 509
    )
    assert len(poses) == 6610 * 4 and len(pairs) == 6187 * 3
    for e in events:
        e["audited_states"] = len(event_states[e["event_id"]])
        e["detailed_source_pairs"] = len(event_pairs[e["event_id"]]) // 3
        e["raw_zero_loss_pairs"] = sum(
            r["raw_zero_loss"]
            for r in event_pairs[e["event_id"]]
            if r["pose_kind"] == "raw"
        )
    state_summary, pair_summary = summarize(poses, pairs, events)
    sources = dict(
        input_sha256=PINNED,
        map_sha256=prior["map_sha256"],
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
            simulator_sha256=digest(SIMULATOR),
        ),
        schema=dict(
            pose_kinds=POSES,
            categories=CATEGORIES,
            truth_relations=RELATIONS,
            subsets=SUBSETS,
            chord_length_m=LENGTH,
            boundary_tolerance_m=EPS,
        ),
        inherited_validation=dict(
            source_sha256=PINNED["sources.json"],
            raw_full_probe=True,
            observation_and_physics=True,
            new_rollout=False,
        ),
    )
    sources["audit"] = dict(
        accounted_cases=len(accounting),
        source_timeout_cases=sum(flag(r["selected"]) for r in accounting),
        opportunities=len(events),
        scanned_source_states=scanned_states,
        scanned_source_transitions=scanned_transitions,
        selected_states=len(states),
        audited_cases=len(set(cases.values())),
        audited_opportunities=sum(e["detailed_source_pairs"] > 0 for e in events),
        selected_source_pairs=len(changes),
        poses=len(poses),
        pairs=len(pairs),
        raw_source_matches=len(states),
        raw_zero_loss_records=len(start_blocks),
        raw_zero_loss_post_states=len({r["post_state_id"] for r in start_blocks}),
        command_only_zero_loss_records=sum(
            r["command_only_zero_loss"] for r in start_blocks
        ),
        near_boundary_poses=sum(r["near_boundary"] for r in poses),
        state_summary_rows=len(state_summary),
        pair_summary_rows=len(pair_summary),
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", accounting),
        ("events", events),
        ("poses", poses),
        ("pairs", pairs),
        ("state-summary", state_summary),
        ("pair-summary", pair_summary),
        ("start-blocks", start_blocks),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(sources["audit"], indent=2))


if __name__ == "__main__":
    main()
