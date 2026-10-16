#!/usr/bin/env python3
"""Diagnose all command-changing pairs in the pinned live terminal-arc study."""

import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path

from analyze_epoch_divergence import (
    first_fork,
    release_inputs,
    selected_checkpoints,
    shared_inputs,
    tail_summary,
)
from analyze_epoch_preview import digest, read_rows
from analyze_filter_stalls import GOALS
from analyze_recovery_arcs import arc_clearance, chord_check
from analyze_terminal_arc_live import COHORTS, SIMULATOR
from analyze_terminal_steering_preview import slacks, stopping_endpoint
from summarize_stopping_preview import save_rows
from verify_stopping_preview import preview_components

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/terminal-arc-live"
KEY = ["cohort", "mode", "seed", "scenario", "planner"]
VARIANTS = ["epoch", "curved"]
DT = 0.02
EPS = 2e-8
PINNED = {
    "cases.csv": "5dfb2eea39b95d8acd93843359fb71bc80944aeac25f81f6e643bdc6018d7b95",
    "pairs.csv": "0a2015799e7ebd4e062c192752cccdbbc1d602bc76bc64ae79f02e3496336a1c",
    "sources.json": "9c53f197f3d26d1c672526b6f7bf82083f0971ee4c6857d5335d408284cdfb54",
}


def same_inputs(a0, a, b0, b):
    return shared_inputs(a0, a, b0, b) and all(
        a[k] == b[k] for k in ["preview_pose_x", "preview_pose_y"]
    )


def locate_fork(reference, candidate, published_time):
    indices, old = first_fork(reference, candidate, published_time)
    assert all(
        same_inputs(reference[i - 1], reference[i], candidate[i - 1], candidate[i])
        for i in range(1, indices["action"] + 1)
    )
    # The tested timing helper predates these two arms. Translate its labels;
    # its estimator-error fields do not describe this geometry-only comparison.
    actor = {"aligned": "epoch", "epoch": "curved"}[old["first_actor"]]
    record = {
        k: v for k, v in old.items() if k.startswith("first_") and k != "first_actor"
    }
    record.update(
        kind=old["kind"],
        first_actor=actor,
        permitting_arm=(
            VARIANTS[1 - VARIANTS.index(actor)] if old["kind"] == "entry" else actor
        ),
        gate="entry" if old["kind"] == "entry" else "release",
        winning_ready_run_s=old["winning_ready_run_s"],
        origin_to_action_s=old["origin_to_action_s"],
        decisive_to_action_s=old["decisive_to_action_s"],
        history_only_release=old["history_only_release"],
        current_release_geometry_differs=old["current_release_geometry_differs"],
        fusion_available_at_action=old["fusion_available_at_action"],
        shared_prefix_steps=indices["action"],
    )
    for legacy, variant in [("aligned", "epoch"), ("epoch", "curved")]:
        for kind in ["guard", "joint"]:
            record[f"{variant}_prior_{kind}_evidence"] = old[
                f"{legacy}_prior_{kind}_evidence"
            ]
    if record["kind"] != "entry":
        expected = 1.0 if record["kind"] == "joint_resume" else 0.5
        assert abs(record["winning_ready_run_s"] - expected) < 1e-8
    return indices, record


def window_summary(track, start, end):
    """Physical states [start,end]; commands/events are the following intervals."""
    assert 0 <= start <= end < len(track)
    goal = GOALS[track[0]["scenario"]]
    distances = [
        math.hypot(float(r["x"]) - goal[0], float(r["y"]) - goal[1])
        for r in track[start : end + 1]
    ]
    rows = track[start + 1 : end + 1]
    return dict(
        start_s=track[start]["time_s"],
        end_s=track[end]["time_s"],
        duration_s=round((end - start) * DT, 6),
        travel=sum(
            math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))
            for a, b in zip(track[start:end], rows)
        ),
        wait_s=round(
            sum(
                r["guard_active"] == "true" or r["safety_paused"] == "true"
                for r in rows
            )
            * DT,
            6,
        ),
        positive_command_s=round(
            sum(float(r["target_speed"]) > 0 for r in rows) * DT, 6
        ),
        moving_s=round(sum(abs(float(r["v"])) > 0.02 for r in rows) * DT, 6),
        guard_entries=sum(r["guard_triggered"] == "true" for r in rows),
        guard_releases=sum(r["guard_released"] == "true" for r in rows),
        resumes=sum(r["resumed"] == "true" for r in rows),
        initial_goal_distance=distances[0],
        final_goal_distance=distances[-1],
        minimum_goal_distance=min(distances),
        goal_progress=distances[0] - distances[-1],
        maximum_speed=max(abs(float(r["v"])) for r in track[start : end + 1]),
    )


def checkpoint_choices(tracks, indices):
    choices = selected_checkpoints(tracks, indices)
    common = min(len(t) for t in tracks) - 1
    for track, chosen in zip(tracks, choices):
        if indices.get("entry") is not None:
            chosen[indices["entry"]].append("first_entry_difference")
        first_release = next(
            (
                i
                for i in range(indices["action"], len(track))
                if float(track[i]["target_speed"]) > 0
                and (
                    track[i]["guard_released"] == "true"
                    or track[i]["resumed"] == "true"
                )
            ),
            None,
        )
        if first_release is not None:
            chosen[first_release].append("first_positive_release_after_fork")
        for duration in [5, 30]:
            chosen[min(indices["action"] - 1 + round(duration / DT), common)].append(
                f"postfork_{duration}s"
            )
    return choices


def exact_geometry(before, row, obstacles, cache):
    logged = tuple(float(row[f"preview_pose_{a}"]) for a in ["x", "y", "yaw"])
    views = dict(
        logged=logged,
        true_xy=(float(before["x"]), float(before["y"]), logged[2]),
        truth=tuple(float(before[a]) for a in ["x", "y", "yaw"]),
    )
    result = []
    speed, target, steering = (
        float(row[k])
        for k in ["measured_speed", "nominal_target_speed", "nominal_steering"]
    )
    entry_margin, release_margin = (
        float(row[k]) for k in ["preview_margin", "guard_release_margin"]
    )
    for pose_kind, pose in views.items():
        key = (*pose, speed, target, steering, entry_margin, release_margin)
        if key not in cache:
            probe = dict(
                row,
                terminal_steering_preview="false",
                **{
                    f"preview_pose_{a}": str(v) for a, v in zip(["x", "y", "yaw"], pose)
                },
            )
            stop, straight, steps = preview_components(before, probe, obstacles)
            endpoint, independent_steps = stopping_endpoint(
                pose, speed, target, steering
            )
            assert steps == independent_steps
            assert abs(straight - arc_clearance(endpoint, 0, obstacles)) < EPS
            curve = arc_clearance(endpoint, steering, obstacles)
            values = []
            for rule, terminal, angle in [
                ("straight", straight, 0),
                ("curved", curve, steering),
            ]:
                scores = slacks(stop, terminal, entry_margin, release_margin)
                error, bound = chord_check(endpoint, angle, obstacles, terminal)
                values.append(
                    dict(
                        rule=rule,
                        stop_exact=stop,
                        terminal_exact=terminal,
                        entry_slack=scores["entry"],
                        release_slack=scores["release"],
                        entry_clear=scores["entry"] > 0,
                        release_clear=scores["release"] > 0,
                        entry_ambiguous=abs(scores["entry"]) <= EPS,
                        release_ambiguous=abs(scores["release"]) <= EPS,
                        physical_clearance=min(stop, terminal),
                        preview_steps=steps,
                        terminal_chord_error=error,
                        terminal_sagitta_bound=bound,
                    )
                )
            cache[key] = values
        for value in cache[key]:
            item = dict(
                pose_kind=pose_kind,
                pose_x=pose[0],
                pose_y=pose[1],
                pose_yaw=pose[2],
                position_error=math.hypot(
                    pose[0] - float(before["x"]), pose[1] - float(before["y"])
                ),
                yaw_error_deg=abs(
                    math.degrees(
                        math.atan2(
                            math.sin(pose[2] - float(before["yaw"])),
                            math.cos(pose[2] - float(before["yaw"])),
                        )
                    )
                ),
                **value,
            )
            if pose_kind == "logged" and (
                (row["terminal_steering_preview"] == "true")
                == (item["rule"] == "curved")
            ):
                assert (
                    float(row["stop_preview_clearance_lower_bound"])
                    <= item["stop_exact"] + EPS
                )
                assert (
                    abs(
                        float(row["terminal_recovery_clearance"])
                        - item["terminal_exact"]
                    )
                    < EPS
                )
                for gate, field, margin in [
                    ("entry", "preview_clearance_lower_bound", entry_margin),
                    (
                        "release",
                        "release_preview_clearance_lower_bound",
                        release_margin,
                    ),
                ]:
                    actual = float(row[field]) - margin
                    assert actual <= item[f"{gate}_slack"] + EPS
                    if min(abs(actual), abs(item[f"{gate}_slack"])) > EPS:
                        assert (actual > 0) == item[f"{gate}_clear"]
            result.append(item)
    return result


def decisive_classification(fork, geometry):
    gate = fork["gate"]
    permission = fork["permitting_arm"]
    denial = VARIANTS[1 - VARIANTS.index(permission)]

    def at(variant, pose):
        rule = "curved" if variant == "curved" else "straight"
        return next(
            r
            for r in geometry
            if r["variant"] == variant and r["pose_kind"] == pose and r["rule"] == rule
        )

    estimated_permit, estimated_deny = at(permission, "logged"), at(denial, "logged")
    truth_permit, truth_deny = at(permission, "truth"), at(denial, "truth")
    assert estimated_permit[f"{gate}_clear"] and not estimated_deny[f"{gate}_clear"]
    p, d = truth_permit[f"{gate}_clear"], truth_deny[f"{gate}_clear"]
    record = dict(
        permit_truth_clear=p,
        deny_truth_clear=d,
        truth_relation=(
            "both_clear"
            if p and d
            else "permit_only" if p else "deny_only" if d else "both_blocked"
        ),
        optimistic_permission=not p,
        conservative_denial=d,
        permit_physical_clearance=truth_permit["physical_clearance"],
        heading_changes_permit=at(permission, "true_xy")[f"{gate}_clear"] != p,
        heading_changes_deny=at(denial, "true_xy")[f"{gate}_clear"] != d,
        decisive_ambiguous=any(
            at(v, pose)[f"{gate}_ambiguous"]
            for v in VARIANTS
            for pose in ["logged", "true_xy", "truth"]
        ),
    )
    margin = float(truth_permit[f"{gate}_margin"])
    record["permit_truth_stop_slack"] = truth_permit["stop_exact"] - margin
    record["permit_truth_terminal_slack"] = (
        truth_permit["terminal_exact"] - (0.1 if gate == "entry" else 0.05) - margin
    )
    record["permit_failure_component"] = (
        "+".join(
            component
            for component in ["stop", "terminal"]
            if record[f"permit_truth_{component}_slack"] <= 0
        )
        or "clear"
    )
    for variant in VARIANTS:
        for pose in ["logged", "truth"]:
            item = at(variant, pose)
            record[f"{variant}_{pose}_slack"] = item[f"{gate}_slack"]
    return record


def summarize(forks):
    result = []
    for cohort in [*COHORTS, "all"]:
        for outcome in [
            "all",
            "held/held",
            "timeout/timeout",
            "held/timeout",
            "timeout/held",
        ]:
            rows = [
                r
                for r in forks
                if (cohort == "all" or r["cohort"] == cohort)
                and (outcome == "all" or r["outcome"] == outcome)
            ]
            if not rows:
                continue
            result.append(
                dict(
                    cohort=cohort,
                    outcome=outcome,
                    pairs=len(rows),
                    **{
                        f"{actor}_{kind}": sum(
                            r["first_actor"] == actor and r["kind"] == kind
                            for r in rows
                        )
                        for actor in VARIANTS
                        for kind in ["entry", "independent_release", "joint_resume"]
                    },
                    **{
                        k: sum(r[k] for r in rows)
                        for k in [
                            "optimistic_permission",
                            "conservative_denial",
                            "heading_changes_permit",
                            "heading_changes_deny",
                            "decisive_ambiguous",
                            "history_only_release",
                        ]
                    },
                    **{
                        f"truth_{kind}": sum(r["truth_relation"] == kind for r in rows)
                        for kind in [
                            "both_clear",
                            "permit_only",
                            "deny_only",
                            "both_blocked",
                        ]
                    },
                    permit_physical_violation=sum(
                        r["permit_physical_clearance"] <= 0.5 for r in rows
                    ),
                    **{
                        f"permit_failure_{kind}": sum(
                            r["permit_failure_component"] == kind for r in rows
                        )
                        for kind in ["clear", "stop", "terminal", "stop+terminal"]
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
    cases = {
        tuple(r[k] for k in [*KEY, "variant"]): r
        for r in read_rows(PRIOR / "cases.csv")
    }
    pairs = read_rows(PRIOR / "pairs.csv")
    selected = [r for r in pairs if r["first_command_change_s"]]
    assert len(pairs) == 324 and len(selected) == 211
    accounting = [dict(r, selected=bool(r["first_command_change_s"])) for r in pairs]
    maps = defaultdict(list)
    for row in read_rows(ROOT / "inputs/obstacles.csv"):
        maps[row["scenario"]].append((float(row["x"]), float(row["y"])))
    sources = dict(prior_sha256=PINNED, cohorts={})
    forks, checkpoints, windows, tails, requirements = [], [], [], [], []
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
        for pair in selected:
            if pair["cohort"] == cohort:
                groups[pair["mode"], pair["seed"]].append(pair)
        hashes = {}
        intervals = unique_audits = prefix_steps = 0
        for (mode, seed), group in sorted(groups.items()):
            keys = {(r["scenario"], r["planner"]) for r in group}
            tracks = []
            for variant in VARIANTS:
                name = f"{variant}-{mode}-{seed}/trajectories.csv"
                path = run / name
                hashes[name] = digest(path)
                assert hashes[name] == pinned["raw_sha256"][name]
                data = defaultdict(list)
                with path.open() as stream:
                    for row in csv.DictReader(stream):
                        key = row["scenario"], row["planner"]
                        if key in keys:
                            data[key].append(row)
                assert set(data) == keys
                tracks.append(data)
            for pair in group:
                identity = {k: pair[k] for k in KEY}
                key = pair["scenario"], pair["planner"]
                current = [t[key] for t in tracks]
                indices, fork = locate_fork(*current, pair["first_command_change_s"])
                outcome = pair["reference_status"] + "/" + pair["candidate_status"]
                fork = dict(**identity, outcome=outcome, **fork)
                prefix_steps += indices["action"]
                choices = checkpoint_choices(current, indices)
                cache, decisive_rows = {}, []
                for arm, (variant, track) in enumerate(zip(VARIANTS, current)):
                    case = cases[tuple(pair[k] for k in KEY) + (variant,)]
                    intervals += len(track) - 1
                    for duration in [5, 30]:
                        start = indices["action"] - 1
                        end = min(
                            start + round(duration / DT),
                            min(len(t) for t in current) - 1,
                        )
                        windows.append(
                            dict(
                                **identity,
                                variant=variant,
                                outcome=outcome,
                                requested_duration_s=duration,
                                **window_summary(track, start, end),
                            )
                        )
                    for scope, end in [
                        ("own", float(track[-1]["time_s"])),
                        ("common", min(float(t[-1]["time_s"]) for t in current)),
                    ]:
                        summary, gates = tail_summary(track, end)
                        tail_id = dict(
                            **identity,
                            variant=variant,
                            outcome=outcome,
                            scope=scope,
                            status=case["status"],
                        )
                        tails.append(
                            dict(
                                **tail_id,
                                first_goal_entry_s=case["first_goal_entry_s"],
                                **summary,
                            )
                        )
                        requirements.extend(dict(**tail_id, **g) for g in gates)
                    for index, reasons in sorted(choices[arm].items()):
                        before, row = track[index - 1], track[index]
                        same = index < len(current[1 - arm]) and same_inputs(
                            before,
                            row,
                            current[1 - arm][index - 1],
                            current[1 - arm][index],
                        )
                        for value in exact_geometry(before, row, maps[key[0]], cache):
                            result = dict(
                                **identity,
                                variant=variant,
                                outcome=outcome,
                                time_s=row["time_s"],
                                reasons="+".join(reasons),
                                paired_inputs_equal=same,
                                measured_speed=row["measured_speed"],
                                nominal_target_speed=row["nominal_target_speed"],
                                nominal_steering=row["nominal_steering"],
                                entry_margin=row["preview_margin"],
                                release_margin=row["guard_release_margin"],
                                fusion_available=row["epoch_fusion_available"],
                                **value,
                            )
                            checkpoints.append(result)
                            if index == indices["decisive"]:
                                decisive_rows.append(result)
                unique_audits += 2 * len(cache)
                fork.update(decisive_classification(fork, decisive_rows))
                forks.append(fork)
            del tracks
        sources["cohorts"][cohort] = dict(
            path=str(run),
            provenance=provenance,
            raw_sha256=hashes,
            selected_pairs=sum(len(g) for g in groups.values()),
            selected_intervals=intervals,
            shared_prefix_steps=prefix_steps,
            unique_terminal_chord_audits=unique_audits,
        )
        print(
            f"{cohort}: {sum(len(g) for g in groups.values())} pairs, {intervals} pinned intervals",
            flush=True,
        )
    sources["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        simulator_sha256=digest(SIMULATOR),
        protocol_sha256=digest(ROOT / "results/terminal-arc-divergence/protocol.md"),
        helpers_sha256={
            name: digest(ROOT / name)
            for name in [
                "analyze_epoch_divergence.py",
                "analyze_epoch_preview.py",
                "analyze_filter_stalls.py",
                "analyze_recovery_stalls.py",
                "analyze_recovery_arcs.py",
                "analyze_terminal_arc_live.py",
                "analyze_terminal_steering_preview.py",
                "verify_stopping_preview.py",
                "verify_joint_recovery.py",
                "summarize_stopping_preview.py",
            ]
        },
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", accounting),
        ("forks", forks),
        ("geometry", checkpoints),
        ("windows", windows),
        ("tails", tails),
        ("requirements", requirements),
        ("summary", summarize(forks)),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")


if __name__ == "__main__":
    main()
