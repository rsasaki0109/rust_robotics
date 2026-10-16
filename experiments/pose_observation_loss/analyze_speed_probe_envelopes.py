#!/usr/bin/env python3
"""Certify fixed first-chord geometry over position and heading product sets."""

import argparse
from collections import defaultdict
from itertools import product
import json
import math
from pathlib import Path

from analyze_commanded_recovery_arc import flag
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import KEY
from analyze_speed_probe_starts import EPS, ESTIMATES, KINDS
from analyze_terminal_arc_live import COHORTS, SIMULATOR
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/speed-probe-starts"
PROTOCOL = ROOT / "results/speed-probe-envelopes/protocol.md"
RADII = [0.0, 0.05, 0.10, 0.15, 0.20, 0.30]
YAWS = [0, 4]
POPULATIONS = ["all_states", "raw_zero_loss", "command_only_zero_loss"]
STATUSES = ["clear", "blocked", "unresolved"]
GAPS = [
    "start_obstacle_gap",
    "start_wall_gap",
    "first_chord_obstacle_gap",
    "first_chord_wall_gap",
]
PINNED = {
    "accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "events.csv": "bc7a40a8da19e573d443c09fec79d4cb5fa1023de74b0bfbcfad2d4d40b07274",
    "poses.csv": "f75df1bd327982a37344466534b9ff511aa10cfc512e865e6d7674bbafda5796",
    "pairs.csv": "15408caa093f53379fa3313ec3e087789399484eacef8159f7efdb91f06a1778",
    "start-blocks.csv": "4bb3e078803d9061e033437f90ec27844a32c3c41d296656422e4cd22985b176",
    "sources.json": "11db3d5bc8ac2ae1725d7c2b3c49e35e4fba3d8a15f0a0a4a169842a9ba6cbc3",
}


def wrapped_error(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def heading_displacement(chord_length, yaw_bound):
    assert chord_length >= 0 and 0 <= yaw_bound <= math.pi
    return 2 * chord_length * math.sin(yaw_bound / 2)


def budgets(gaps, displacement):
    """Radius budgets from individual 1-Lipschitz start/chord constraints."""
    assert displacement >= 0 and all(math.isfinite(g) for g in gaps)
    so, sw, co, cw = gaps
    return min(so, sw, co - displacement, cw - displacement), max(
        -so, -sw, -co - displacement, -cw - displacement
    )


def certificate(clear_budget, block_budget, radius):
    assert radius >= 0 and all(
        math.isfinite(v) for v in [clear_budget, block_budget, radius]
    )
    clear = clear_budget - radius > EPS
    blocked = block_budget - radius >= EPS
    assert not (clear and blocked)
    return "clear" if clear else "blocked" if blocked else "unresolved"


def pose_record(pose, truth, event_ids):
    assert pose["state_id"] == truth["state_id"]
    assert pose["truth_pose_id"] == truth["pose_id"]
    assert all(pose[k] == truth[k] for k in [*KEY, "steering", "margin"])
    xy = tuple(float(pose[k]) for k in ["x", "y"])
    end = tuple(float(pose[k]) for k in ["first_end_x", "first_end_y"])
    true_xy = tuple(float(truth[k]) for k in ["x", "y"])
    length = math.dist(xy, end)
    assert 0 < length <= 0.05 + EPS
    error = math.dist(xy, true_xy)
    yaw_error = wrapped_error(float(pose["yaw"]), float(truth["yaw"]))
    gaps = [float(pose[k]) for k in GAPS]
    true_gap = min(float(truth[k]) for k in GAPS)
    assert (true_gap > 0) == flag(truth["first_chord_clear"])
    assert (min(gaps) > 0) == flag(pose["first_chord_clear"])
    actual_displacement = heading_displacement(length, yaw_error)
    cb, bb = budgets(gaps, actual_displacement)
    lower, upper = cb - error, error - bb
    assert lower - EPS <= true_gap <= upper + EPS, (pose["pose_id"], true_gap)
    oracle = certificate(cb, bb, error)
    assert oracle == "unresolved" or (oracle == "clear") == (true_gap > 0)
    record = dict(
        pose_id=pose["pose_id"],
        state_id=pose["state_id"],
        source_interval_id=pose["source_interval_id"],
        truth_pose_id=truth["pose_id"],
        **{k: pose[k] for k in KEY},
        source_event_ids="+".join(sorted(event_ids, key=int)),
        source_event_kinds=pose["source_event_kinds"],
        time_s=pose["time_s"],
        pose_kind=pose["pose_kind"],
        **dict(zip(GAPS, gaps)),
        chord_length=length,
        central_clear=min(gaps) > 0,
        truth_clear=true_gap > 0,
        estimate_gap=min(gaps),
        truth_gap=true_gap,
        gap_overstatement=min(gaps) - true_gap,
        truth_shortfall=max(0.0, -true_gap),
        position_error=error,
        heading_error_rad=yaw_error,
        heading_error_deg=math.degrees(yaw_error),
        oracle_heading_displacement=actual_displacement,
        oracle_lower_gap=lower,
        oracle_upper_gap=upper,
        oracle_status=oracle,
    )
    for yaw in YAWS:
        e = heading_displacement(length, math.radians(yaw))
        cb, bb = budgets(gaps, e)
        record[f"heading_displacement_{yaw}deg"] = e
        record[f"clear_budget_{yaw}deg"] = cb
        record[f"block_budget_{yaw}deg"] = bb
    return record


def decision(row, radius, yaw):
    status = certificate(
        row[f"clear_budget_{yaw}deg"], row[f"block_budget_{yaw}deg"], radius
    )
    position_covered = row["position_error"] <= radius
    heading_covered = row["heading_error_rad"] <= math.radians(yaw)
    covered = position_covered and heading_covered
    clear_conflict = status == "clear" and not row["truth_clear"]
    block_conflict = status == "blocked" and row["truth_clear"]
    assert not (covered and (clear_conflict or block_conflict)), row["pose_id"]
    return dict(
        radius_m=radius,
        yaw_bound_deg=yaw,
        status=status,
        position_covered=position_covered,
        heading_covered=heading_covered,
        covered=covered,
        clear_truth_blocks=clear_conflict,
        blocked_truth_clear=block_conflict,
    )


def mean(rows, field):
    return sum(r[field] for r in rows) / len(rows) if rows else ""


def population_metrics(rows):
    conflicts = [r for r in rows if r["central_clear"] and not r["truth_clear"]]
    return dict(
        central_clear=sum(r["central_clear"] for r in rows),
        truth_clear=sum(r["truth_clear"] for r in rows),
        central_clear_truth_blocks=len(conflicts),
        central_blocks_truth_clear=sum(
            not r["central_clear"] and r["truth_clear"] for r in rows
        ),
        mean_position_error=mean(rows, "position_error"),
        rms_position_error=(
            math.sqrt(sum(r["position_error"] ** 2 for r in rows) / len(rows))
            if rows
            else ""
        ),
        mean_heading_error_deg=mean(rows, "heading_error_deg"),
        mean_truth_shortfall=mean(rows, "truth_shortfall"),
        max_truth_shortfall=max((r["truth_shortfall"] for r in rows), default=""),
        conflict_mean_position_error=mean(conflicts, "position_error"),
        conflict_mean_heading_error_deg=mean(conflicts, "heading_error_deg"),
        conflict_mean_truth_shortfall=mean(conflicts, "truth_shortfall"),
        conflict_max_truth_shortfall=max(
            (r["truth_shortfall"] for r in conflicts), default=""
        ),
        **{f"oracle_{s}": sum(r["oracle_status"] == s for r in rows) for s in STATUSES},
    )


def summarize(poses, blocks, events):
    lookup = {r["pose_id"]: r for r in poses}
    groups = defaultdict(list)
    opportunities = defaultdict(set)

    def add(row, population, event_ids):
        kinds = {"all", *[events[eid]["event_kind"] for eid in event_ids]}
        for cohort, kind in product(["all", row["cohort"]], sorted(kinds)):
            key = cohort, row["variant"], kind, row["pose_kind"], population
            groups[key].append(row)
            opportunities[key].update(
                eid
                for eid in event_ids
                if kind == "all" or events[eid]["event_kind"] == kind
            )

    for row in poses:
        add(row, "all_states", row["source_event_ids"].split("+"))
    for block in blocks:
        for pose in ESTIMATES:
            row = lookup[block[pose + "_pose_id"]]
            add(row, "raw_zero_loss", [block["event_id"]])
            if flag(block["command_only_zero_loss"]):
                add(row, "command_only_zero_loss", [block["event_id"]])
    summary = []
    for cohort, variant, kind, pose, population in product(
        ["all", *COHORTS],
        ["epoch", "curved"],
        ["all", *KINDS],
        ESTIMATES,
        POPULATIONS,
    ):
        key = cohort, variant, kind, pose, population
        rows = groups[key]
        metrics = population_metrics(rows)
        for radius, yaw in product(RADII, YAWS):
            decisions = [decision(r, radius, yaw) for r in rows]
            counts = {s: sum(d["status"] == s for d in decisions) for s in STATUSES}
            assert sum(counts.values()) == len(rows)
            summary.append(
                dict(
                    cohort=cohort,
                    variant=variant,
                    event_kind=kind,
                    pose_kind=pose,
                    population=population,
                    radius_m=radius,
                    yaw_bound_deg=yaw,
                    records=len(rows),
                    states=len({r["state_id"] for r in rows}),
                    cases=len({tuple(r[k] for k in KEY) for r in rows}),
                    opportunities=len(opportunities[key]),
                    **counts,
                    **{
                        k: sum(d[k] for d in decisions)
                        for k in [
                            "position_covered",
                            "heading_covered",
                            "covered",
                            "clear_truth_blocks",
                            "blocked_truth_clear",
                        ]
                    },
                    **{
                        f"{s}_covered": sum(
                            d["covered"] and d["status"] == s for d in decisions
                        )
                        for s in STATUSES
                    },
                    **metrics,
                )
            )
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for name, sha in PINNED.items():
        assert digest(PRIOR / name) == sha, name
    prior = json.loads((PRIOR / "sources.json").read_text())
    helpers = dict(prior["analysis"]["helpers_sha256"])
    helpers["analyze_speed_probe_starts.py"] = prior["analysis"]["script_sha256"]
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    assert digest(SIMULATOR) == prior["analysis"]["simulator_sha256"]
    assert digest(ROOT / "inputs/obstacles.csv") == prior["map_sha256"]
    accounting, event_rows, source_poses, pairs, blocks = (
        read_rows(PRIOR / n)
        for n in [
            "accounting.csv",
            "events.csv",
            "poses.csv",
            "pairs.csv",
            "start-blocks.csv",
        ]
    )
    assert len(accounting) == 648 and len(event_rows) == 2583
    assert len(source_poses) == 26440 and len(pairs) == 18561 and len(blocks) == 550
    events = {e["event_id"]: e for e in event_rows}
    source = {r["pose_id"]: r for r in source_poses}
    assert len(source) == len(source_poses)
    state_events = defaultdict(set)
    pair_views = defaultdict(list)
    for pair in pairs:
        pair_views[pair["transition_id"]].append(pair)
        for phase in ["pre", "post"]:
            row = source[pair[phase + "_pose_id"]]
            assert row["state_id"] == pair[phase + "_state_id"]
            assert row["pose_kind"] == pair["pose_kind"]
            assert row["truth_pose_id"] == pair[phase + "_truth_pose_id"]
            assert all(row[k] == events[pair["event_id"]][k] for k in KEY)
            state_events[row["state_id"]].add(pair["event_id"])
    assert len(pair_views) == 6187 and len(state_events) == 6610
    for views in pair_views.values():
        assert sorted(r["pose_kind"] for r in views) == sorted(ESTIMATES)
    block_ids = set()
    for b in blocks:
        assert b["transition_id"] not in block_ids
        block_ids.add(b["transition_id"])
        for p in pair_views[b["transition_id"]]:
            assert flag(p["raw_zero_loss"])
            assert all(
                p[k] == b[k]
                for k in [
                    "event_id",
                    "post_state_id",
                    "command_only_zero_loss",
                    "source_failed_mask",
                ]
            )
            assert p["post_pose_id"] == b[p["pose_kind"] + "_pose_id"]
    assert block_ids == {p["transition_id"] for p in pairs if flag(p["raw_zero_loss"])}
    assert len({b["post_state_id"] for b in blocks}) == 550
    assert sum(flag(b["command_only_zero_loss"]) for b in blocks) == 509
    poses = []
    for p in source_poses:
        if p["pose_kind"] in ESTIMATES:
            ids = state_events[p["state_id"]]
            assert p["source_event_kinds"] == "+".join(
                sorted({events[eid]["event_kind"] for eid in ids})
            )
            poses.append(pose_record(p, source[p["truth_pose_id"]], ids))
    assert len(poses) == 19830
    # Check every covered certificate, including groups not shown in a figure.
    for p, radius, yaw in product(poses, RADII, YAWS):
        decision(p, radius, yaw)
    lookup = {r["pose_id"]: r for r in poses}
    losses = []
    for b, pose, radius, yaw in product(blocks, ESTIMATES, RADII, YAWS):
        p = lookup[b[pose + "_pose_id"]]
        losses.append(
            dict(
                transition_id=b["transition_id"],
                event_id=b["event_id"],
                pose_id=p["pose_id"],
                state_id=p["state_id"],
                pose_kind=pose,
                command_only_zero_loss=flag(b["command_only_zero_loss"]),
                central_clear=p["central_clear"],
                truth_clear=p["truth_clear"],
                **decision(p, radius, yaw),
            )
        )
    summary = summarize(poses, blocks, events)
    audit = dict(
        accounted_cases=len(accounting),
        source_timeout_cases=sum(flag(r["selected"]) for r in accounting),
        opportunities=len(events),
        audited_states=len(state_events),
        audited_cases=len({tuple(r[k] for k in KEY) for r in poses}),
        audited_opportunities=len(
            {eid for ids in state_events.values() for eid in ids}
        ),
        estimated_poses=len(poses),
        source_pairs=len(pair_views),
        raw_zero_loss_pairs=len(blocks),
        command_only_zero_loss_pairs=509,
        checked_pose_settings=len(poses) * len(RADII) * len(YAWS),
        checked_oracle_intervals=len(poses),
        loss_decisions=len(losses),
        summary_rows=len(summary),
    )
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
            radii_m=RADII,
            yaw_bounds_deg=YAWS,
            populations=POPULATIONS,
            statuses=STATUSES,
            certificate_guard_m=EPS,
            coverage="closed disk and closed heading interval; no coverage tolerance",
        ),
        inherited_validation=dict(
            source_sha256=PINNED["sources.json"],
            observation_and_physics=True,
            raw_full_probe=True,
            new_rollout=False,
        ),
        audit=audit,
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", accounting),
        ("events", event_rows),
        ("poses", poses),
        ("loss-decisions", losses),
        ("summary", summary),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    for name in ["accounting.csv", "events.csv"]:
        assert digest(args.output / name) == PINNED[name]
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
