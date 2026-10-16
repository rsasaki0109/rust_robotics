#!/usr/bin/env python3
"""Compare straight and commanded-steering terminal geometry at frozen poses."""

import argparse
from collections import defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path

from analyze_recovery_arcs import arc_clearance, chord_check
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/epoch-divergence"
PROTOCOL = ROOT / "results/terminal-steering-preview/protocol.md"
SIMULATOR = (
    ROOT.parents[1] / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
)
KEY = ["cohort", "mode", "seed", "scenario", "planner"]
IDENTITY = KEY + [
    "variant",
    "outcome",
    "time_s",
    "reasons",
    "paired_inputs_equal",
    "fusion_available",
]
POSES = {"logged", "original", "true_xy", "truth"}
EPS = 2e-8
PINNED = {
    "geometry.csv": "68854029b7c63dc36da8431761ce1de3abb4541788ae7f60fde600bee2c2c35f",
    "forks.csv": "705256e42a7521a6cd75ed04ab9b2ef9097e27c8fa864f7d72584f814778e1d3",
    "accounting.csv": "0a23e6d8f0bf3a19edaafc7404adae3c7443d9ad92b351e0c59fa1058b7f8be1",
    "sources.json": "f0e83e6947554a5edebdcc5ce32181f4daf37d4b9ed42fa4d57548f99e24ebed",
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_rows(path):
    with path.open() as source:
        return list(csv.DictReader(source))


def stopping_endpoint(pose, speed, target, steering):
    """Rebuild the published Euler forecast; only its terminal pose is needed."""
    assert all(math.isfinite(v) for v in (*pose, speed, target, steering))
    assert abs(steering) <= math.radians(35) + 1e-9
    x, y, yaw = pose
    for step in range(200):
        acceleration = target - speed if step < 10 else max(-1, min(0, -4 * speed))
        x += speed * math.cos(yaw) * 0.02
        y += speed * math.sin(yaw) * 0.02
        yaw += speed / 0.4 * math.tan(steering) * 0.02
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        speed += acceleration * 0.02
        if step >= 10 and abs(speed) <= 0.001:
            return (x, y, yaw), step + 1
    raise AssertionError("Forecast did not stop within 200 steps")


def slacks(stop, terminal, entry_margin, release_margin):
    return {
        "entry": min(stop, terminal - 0.1) - entry_margin,
        "release": min(stop, terminal - 0.05) - release_margin,
    }


def inspect_checkpoint(row, obstacles):
    pose = tuple(float(row[f"pose_{a}"]) for a in ["x", "y", "yaw"])
    steering = float(row["nominal_steering"])
    endpoint, steps = stopping_endpoint(
        pose, float(row["measured_speed"]), float(row["nominal_target_speed"]), steering
    )
    assert steps == int(row["preview_steps"])
    straight = arc_clearance(endpoint, 0, obstacles)
    assert abs(straight - float(row["terminal_exact"])) < EPS
    nominal = arc_clearance(endpoint, steering, obstacles)
    stop = float(row["stop_exact"])
    margins = [float(row[k]) for k in ["entry_margin", "release_margin"]]
    old = slacks(stop, straight, *margins)
    new = slacks(stop, nominal, *margins)
    for gate, slack in old.items():
        assert abs(slack - float(row[f"{gate}_slack"])) < EPS
    straight_error, straight_bound = chord_check(endpoint, 0, obstacles, straight)
    nominal_error, nominal_bound = chord_check(endpoint, steering, obstacles, nominal)
    return dict(
        row,
        stop_pose_x=endpoint[0],
        stop_pose_y=endpoint[1],
        stop_pose_yaw=endpoint[2],
        straight_terminal=straight,
        nominal_terminal=nominal,
        terminal_delta=nominal - straight,
        nominal_entry_slack=new["entry"],
        nominal_release_slack=new["release"],
        nominal_entry_clear=new["entry"] > 0,
        nominal_release_clear=new["release"] > 0,
        nominal_entry_ambiguous=abs(new["entry"]) <= EPS,
        nominal_release_ambiguous=abs(new["release"]) <= EPS,
        straight_chord_error=straight_error,
        straight_sagitta=straight_bound,
        nominal_chord_error=nominal_error,
        nominal_sagitta=nominal_bound,
    )


def decision_records(rows):
    """Use identical nominal controls for estimate and evaluation-only true poses."""
    poses = {r["pose_kind"]: r for r in rows}
    assert len(rows) == 4 and set(poses) == POSES
    estimate, truth, true_xy = (poses[k] for k in ["logged", "truth", "true_xy"])
    shared = IDENTITY + [
        "measured_speed",
        "nominal_target_speed",
        "nominal_steering",
        "entry_margin",
        "release_margin",
        "preview_steps",
    ]
    assert all(all(r[k] == estimate[k] for k in shared) for r in rows)
    result = []
    for rule in ["straight", "nominal"]:
        prefix = "" if rule == "straight" else "nominal_"
        terminal = f"{rule}_terminal"
        for gate in ["entry", "release"]:
            slack = f"{prefix}{gate}_slack"
            a, b, c = (float(r[slack]) for r in [estimate, truth, true_xy])
            physical = min(float(truth["stop_exact"]), float(truth[terminal]))
            result.append(
                dict(
                    **{k: estimate[k] for k in IDENTITY},
                    rule=rule,
                    gate=gate,
                    measured_speed=estimate["measured_speed"],
                    nominal_target_speed=estimate["nominal_target_speed"],
                    nominal_steering=estimate["nominal_steering"],
                    estimate_stop=estimate["stop_exact"],
                    truth_stop=truth["stop_exact"],
                    estimate_terminal=estimate[terminal],
                    truth_terminal=truth[terminal],
                    estimate_slack=a,
                    truth_slack=b,
                    true_xy_slack=c,
                    estimate_clear=a > 0,
                    truth_clear=b > 0,
                    optimistic=a > 0 and b <= 0,
                    conservative=a <= 0 and b > 0,
                    heading_changes_truth=(b > 0) != (c > 0),
                    truth_physical_clearance=physical,
                    accepted_physical_violation=a > 0 and physical <= 0.5,
                    ambiguous=abs(a) <= EPS or abs(b) <= EPS or abs(c) <= EPS,
                )
            )
    return result


def subsets(row):
    reasons = set(row["reasons"].split("+"))
    result = ["all"]
    if "first_decisive" in reasons:
        result.append("decisive")
    if reasons & {"common_endpoint", "own_endpoint"}:
        result.append("endpoints")
    if "first_fresh_stationary_in_final_window" in reasons:
        result.append("fresh_tail")
    return result


def summarize(decisions):
    groups = defaultdict(list)
    for row in decisions:
        for cohort in ["all", row["cohort"]]:
            for subset in subsets(row):
                groups[cohort, row["variant"], subset, row["rule"], row["gate"]].append(
                    row
                )
    results = []
    for (cohort, variant, subset, rule, gate), rows in sorted(groups.items()):
        results.append(
            dict(
                cohort=cohort,
                variant=variant,
                subset=subset,
                rule=rule,
                gate=gate,
                checkpoints=len(rows),
                **{
                    k: sum(r[k] for r in rows)
                    for k in [
                        "estimate_clear",
                        "truth_clear",
                        "optimistic",
                        "conservative",
                        "heading_changes_truth",
                        "accepted_physical_violation",
                        "ambiguous",
                    ]
                },
                agreement=sum(r["estimate_clear"] == r["truth_clear"] for r in rows),
                min_accepted_truth_physical_clearance=min(
                    (
                        r["truth_physical_clearance"]
                        for r in rows
                        if r["estimate_clear"]
                    ),
                    default="",
                ),
            )
        )
    return results


def summarize_transitions(decisions):
    paired = defaultdict(dict)
    for row in decisions:
        key = tuple(row[k] for k in KEY + ["variant", "time_s", "gate"])
        assert row["rule"] not in paired[key]
        paired[key][row["rule"]] = row
    groups = defaultdict(list)
    for pair in paired.values():
        assert set(pair) == {"straight", "nominal"}
        old, new = pair["straight"], pair["nominal"]
        assert all(old[k] == new[k] for k in IDENTITY)
        gained = not old["estimate_clear"] and new["estimate_clear"]
        transition = dict(
            estimate_gained_pass=gained,
            estimate_lost_pass=old["estimate_clear"] and not new["estimate_clear"],
            truth_gained_pass=not old["truth_clear"] and new["truth_clear"],
            truth_lost_pass=old["truth_clear"] and not new["truth_clear"],
            gained_true_clear=gained and new["truth_clear"],
            gained_true_block=gained and not new["truth_clear"],
        )
        for cohort in ["all", new["cohort"]]:
            for subset in subsets(new):
                groups[cohort, new["variant"], subset, new["gate"]].append(transition)
    result = []
    for (cohort, variant, subset, gate), rows in sorted(groups.items()):
        result.append(
            dict(
                cohort=cohort,
                variant=variant,
                subset=subset,
                gate=gate,
                checkpoints=len(rows),
                **{k: sum(r[k] for r in rows) for k in rows[0]},
            )
        )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O; assertions are required")
    if args.output.exists():
        parser.error("Output directory must be new")
    for name, expected in PINNED.items():
        assert digest(PRIOR / name) == expected, name
    source = json.loads((PRIOR / "sources.json").read_text())
    assert digest(SIMULATOR) == source["analysis"]["simulator_sha256"]
    obstacle_path = ROOT / "inputs/obstacles.csv"
    map_hash = digest(obstacle_path)
    for cohort in source["cohorts"].values():
        assert map_hash == cohort["provenance"]["inputs_sha256"]["obstacles.csv"]
    maps = defaultdict(list)
    for row in load_rows(obstacle_path):
        maps[row["scenario"]].append((float(row["x"]), float(row["y"])))
    original = load_rows(PRIOR / "geometry.csv")
    accounting = load_rows(PRIOR / "accounting.csv")
    forks = load_rows(PRIOR / "forks.csv")
    assert len(original) == 4464 and len(accounting) == 324 and len(forks) == 121
    fork_by_key = {tuple(r[k] for k in KEY): r for r in forks}
    assert len(fork_by_key) == 121
    groups = defaultdict(list)
    audited = []
    for row in original:
        result = inspect_checkpoint(row, maps[row["scenario"]])
        audited.append(result)
        groups[tuple(row[k] for k in KEY + ["variant", "time_s"])].append(result)
    assert len(groups) == 1116
    decisions = []
    decisive = defaultdict(set)
    for key, rows in groups.items():
        fork = fork_by_key[key[:5]]
        assert rows[0]["outcome"] == fork["outcome"]
        if "first_decisive" in rows[0]["reasons"].split("+"):
            assert rows[0]["time_s"] == fork["first_decisive_s"]
            decisive[key[5]].add(key[:5])
        decisions.extend(decision_records(rows))
    assert set(decisive) == {"aligned", "epoch"}
    assert all(len(keys) == 121 for keys in decisive.values())
    assert len(decisions) == 4464
    summary = summarize(decisions)
    transitions = summarize_transitions(decisions)
    provenance = dict(
        source_main="5da777553d4971b0df9af7e26e4718857825b6f3",
        prior_sha256=PINNED,
        obstacles_sha256=map_hash,
        simulator_sha256=digest(SIMULATOR),
        script_sha256=digest(Path(__file__)),
        protocol_sha256=digest(PROTOCOL),
        helpers_sha256={
            name: digest(ROOT / name)
            for name in [
                "analyze_recovery_arcs.py",
                "verify_stopping_preview.py",
                "summarize_stopping_preview.py",
            ]
        },
        accounting_pairs=len(accounting),
        selected_pairs=len(forks),
        checkpoints=len(groups),
        pose_rows=len(audited),
        terminal_chord_checks=2 * len(audited),
        max_straight_error=max(r["straight_chord_error"] for r in audited),
        max_nominal_error=max(r["nominal_chord_error"] for r in audited),
        ambiguous_decisions=sum(r["ambiguous"] for r in decisions),
        ambiguous_pose_gates=sum(
            abs(float(r[f"{gate}_slack"])) <= EPS
            or abs(r[f"nominal_{gate}_slack"]) <= EPS
            for r in audited
            for gate in ["entry", "release"]
        ),
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", accounting),
        ("geometry", audited),
        ("decisions", decisions),
        ("summary", summary),
        ("transitions", transitions),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(json.dumps(provenance, indent=2))


if __name__ == "__main__":
    main()
