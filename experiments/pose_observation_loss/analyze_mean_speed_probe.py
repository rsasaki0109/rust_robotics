#!/usr/bin/env python3
"""Audit every case in the fixed closed-loop mean speed-probe pilot."""

import argparse
from collections import defaultdict, deque
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess

import build_mean_speed_probe as build
from analyze_epoch_preview import CollisionAudit
from analyze_reset_mean_probe import probe
from verify_joint_recovery import check_guard_track, check_joint_track
from verify_motion_filter import check_motion_filter_track

DT = 0.02
GOALS = {
    "open_50x50": (48.0, 48.0),
    "dense_50x50": (48.0, 48.0),
    "maze_50x50": (35.0, 45.0),
}
CASE_META = ("tau", "arm", "mode", "seed", "scenario", "planner")
CASE_FIELDS = (
    *CASE_META,
    "status",
    "held",
    "collision",
    "boundary",
    "time_s",
    "goal_distance",
    "final_speed",
    "paused_time_s",
    "pause_count",
    "resume_count",
    "decisions",
    "mean_used_steps",
    "geometry_samples",
    "contact_segments",
    "boundary_segments",
)
VERIFY_FIELDS = (
    *CASE_META,
    "physical_steps",
    "sidecar_decisions",
    "geometry_samples",
    "mean_used_steps",
    "contact_segments",
    "boundary_segments",
    "native_hold_s",
)
SAMPLE_FIELDS = (
    *CASE_META,
    "step",
    "used",
    "native_distance",
    "python_distance",
    "python_first_unsafe_step",
    "python_wall_ids",
    "python_obstacle_ids",
)
PAIR_FIELDS = (
    "tau",
    "mode",
    "seed",
    "scenario",
    "planner",
    "raw_status",
    "mean_status",
    "raw_held",
    "mean_held",
    "raw_collision",
    "mean_collision",
    "raw_boundary",
    "mean_boundary",
    "raw_goal_distance",
    "mean_goal_distance",
    "raw_time_s",
    "mean_time_s",
    "outcome",
    "first_difference_step",
    "common_steps",
    "raw_common_paused_s",
    "mean_common_paused_s",
    "mean_used_steps",
)
SUMMARY_FIELDS = (
    "noise_selector",
    "mode_selector",
    "pairs",
    "raw_held",
    "mean_held",
    "gained_hold",
    "lost_hold",
    "both_held",
    "neither_held",
    "raw_contacts",
    "mean_contacts",
    "raw_boundaries",
    "mean_boundaries",
    "raw_timeouts",
    "mean_timeouts",
    "changed_pairs",
    "mean_used_steps",
    "raw_common_paused_s",
    "mean_common_paused_s",
    "raw_mean_goal_distance",
    "mean_mean_goal_distance",
)


def read(path, fields=None):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        names = reader.fieldnames
        build.require(
            names and len(set(names)) == len(names), f"Invalid CSV header: {path}"
        )
        if fields is not None:
            build.require(
                tuple(names) == tuple(fields), f"Unexpected CSV fields: {path}"
            )
        rows = list(reader)
    build.require(
        all(None not in r and None not in r.values() for r in rows),
        f"Malformed CSV: {path}",
    )
    return rows


def write(path, rows, fields):
    with Path(path).open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def truth(value):
    build.require(value in ("true", "false"), "Expected a native boolean")
    return value == "true"


def finite(value):
    result = float(value)
    build.require(math.isfinite(result), "Nonfinite pilot evidence")
    return result


def close(a, b, tolerance=1e-12):
    build.require(
        abs(finite(a) - finite(b)) <= tolerance, f"Numeric mismatch: {a} != {b}"
    )


def ordered_mean(samples, index):
    # Python 3.12 sum uses a different algorithm from the archived Rust fold.
    total = 0.0
    for sample in samples:
        total += sample[index]
    return total / len(samples)


def geometry_sample_steps(sides):
    count = len(sides)
    selected = {1, count, *range(250, count + 1, 250)}
    previous = False
    for i, row in enumerate(sides, 1):
        current = truth(row["used"])
        if current != previous:
            selected.update((i - 1, i))
        previous = current
    return selected - {0}


def audit_track(meta, track, sides, outcome, obstacles, samples):
    build.require(
        len(track) >= 2 and len(sides) == len(track) - 1,
        "Incomplete sidecar/trajectory pair",
    )
    build.require(set(sides[0]) == set(build.SIDECAR), "Sidecar schema differs")
    key = meta["scenario"], meta["planner"]
    build.require(
        all((r["scenario"], r["planner"]) == key for r in (*track, *sides)),
        "Mixed native cases",
    )
    build.require(
        finite(track[0]["time_s"]) == 0 and not truth(track[0]["safety_paused"]),
        "Invalid initial checkpoint",
    )
    window = deque(maxlen=25)
    selected = geometry_sample_steps(sides)
    spatial = CollisionAudit([(x, y) for _, x, y in obstacles])
    goal = GOALS[meta["scenario"]]
    previous_state = tuple(finite(track[0][name]) for name in ("x", "y", "yaw", "v"))
    contacts = boundaries = mean_uses = pauses = resumes = 0
    hold = paused_time = rms = 0.0
    last_observation = 0.0
    min_clearance = finite(track[0]["step_min_obstacle_distance"])
    cached_segment = cached_contact = None
    for step, (before, row, side) in enumerate(zip(track, track[1:], sides), 1):
        build.require(int(side["step"]) == step, "Sidecar step order differs")
        close(side["time_s"], step * DT)
        close(row["time_s"], step * DT)
        fields = {
            k: finite(side[k])
            for k in build.SIDECAR
            if k
            not in (
                "scenario",
                "planner",
                "enabled",
                "used",
                "paused",
                "correction",
                "received",
                "precision_activation",
            )
        }
        velocity, age = fields["measured_speed"], fields["observation_age"]
        expected_rho = (
            math.exp(-DT / finite(meta["tau"])) if finite(meta["tau"]) > 0 else 0.0
        )
        close(row["noise_rho"], expected_rho, 5.1e-13)
        build.require(
            all(
                truth(row[name])
                for name in (
                    "terminal_steering_preview",
                    "stationary_epoch_fusion",
                    "aligned_terminal_release",
                    "terminal_entry_only",
                    "recovery_preview_active",
                )
            ),
            "Curved controller configuration differs",
        )
        build.require(
            velocity == previous_state[3],
            "Measured speed is not the physical predecessor",
        )
        close(side["measured_speed"], row["measured_speed"], 0)
        close(age, row["observation_age"], 5.1e-7)
        for name, legacy in (
            ("correction", "correction"),
            ("received", "correction_received"),
            ("precision_activation", "precision_activation"),
        ):
            build.require(side[name] == row[legacy], "Observation metadata differs")
        if truth(side["received"]) or truth(side["precision_activation"]):
            last_observation = fields["time_s"]
        build.require(
            age == fields["time_s"] - last_observation,
            "Observation age differs from received corrections",
        )
        if meta["mode"] == "none":
            available = True
        elif meta["mode"] == "blackout":
            available = not 45 <= fields["time_s"] < 75
        else:
            uniform = finite(row["loss_uniform"])
            build.require(
                abs(uniform - 0.3) > 5.1e-13,
                "Logged availability threshold is ambiguous",
            )
            available = uniform >= 0.3
        build.require(
            truth(row["observation_available"]) == available,
            "Availability mode differs",
        )
        missing = truth(side["correction"]) and not truth(side["received"])
        pending = truth(before["safety_paused"]) or missing
        build.require(
            truth(side["paused"]) == pending, "Pre-command pause state differs"
        )
        reset = (
            truth(side["received"])
            or truth(side["precision_activation"])
            or missing
            or abs(velocity) > 0.02
            or age > 1.5
        )
        if reset:
            window.clear()
        raw = tuple(fields["raw_" + axis] for axis in ("x", "y", "yaw"))
        for i, axis in enumerate(("x", "y", "yaw")):
            reconstructed = (previous_state[i] + finite(row["noise_" + axis])) + finite(
                row["bias_" + axis]
            )
            close(raw[i], reconstructed, 1.1e-9)
        if pending and abs(velocity) <= 0.02 and age <= 1.5:
            window.append(raw)
        build.require(
            fields["count"] == len(window) == int(row["recovery_sample_count"]),
            "Recovery window membership differs",
        )
        if window:
            mean = tuple(ordered_mean(window, i) for i in (0, 1))
            sine = cosine = 0.0
            for item in window:
                sine += math.sin(item[2])
                cosine += math.cos(item[2])
            mean_yaw = math.atan2(sine, cosine)
        else:
            mean, mean_yaw = raw[:2], raw[2]
        for axis, expected in zip(("x", "y"), mean):
            close(fields["mean_" + axis], expected)
            close(fields["mean_" + axis], row["recovery_" + axis], 5.1e-10)
        close(fields["mean_yaw"], mean_yaw)
        enabled = meta["arm"] == "mean_probe"
        used = (
            enabled
            and pending
            and abs(velocity) <= 0.02
            and age <= 1.5
            and len(window) >= 10
        )
        build.require(
            truth(side["enabled"]) == enabled and truth(side["used"]) == used,
            "Mean selection eligibility differs",
        )
        expected_pose = mean if used else raw[:2]
        for axis, expected in zip(("x", "y"), expected_pose):
            close(fields["probe_" + axis], expected)
            build.require(
                fields["probe_" + axis] == fields[("mean_" if used else "raw_") + axis],
                "Selected position differs from the actual logged selector input",
            )
        build.require(
            fields["probe_yaw"] == fields["raw_yaw"], "Speed probe heading changed"
        )
        close(fields["steering"], row["nominal_steering"], 5.1e-10)
        close(fields["margin"], row["speed_margin"], 5.1e-10)
        distance = fields["free_distance"]
        build.require(
            0 <= distance <= 2
            and abs(distance / 0.05 - round(distance / 0.05)) < 1e-12,
            "Invalid native probe distance",
        )
        cap = min(1.0, max(0.0, math.sqrt(0.15**2 + distance) - 0.15))
        close(fields["cap"], cap)
        close(fields["nominal_target"], min(fields["uncapped_target"], cap))
        for side_name, legacy in (
            ("free_distance", "speed_free_distance"),
            ("cap", "speed_cap"),
            ("nominal_target", "nominal_target_speed"),
            ("uncapped_target", "uncapped_target_speed"),
        ):
            close(fields[side_name], row[legacy], 5.1e-10)
        if step in selected:
            geometry = probe(
                fields["probe_x"],
                fields["probe_y"],
                fields["raw_yaw"],
                fields["steering"],
                fields["margin"],
                obstacles,
            )
            samples.append(
                dict(
                    **meta,
                    step=step,
                    used=side["used"],
                    native_distance=distance,
                    python_distance=geometry["free_distance"],
                    python_first_unsafe_step=geometry["first_unsafe_step"],
                    python_wall_ids=geometry["wall_ids"],
                    python_obstacle_ids=geometry["obstacle_ids"],
                )
            )
            build.require(
                geometry["free_distance"] == distance,
                f"Sampled geometry differs: {meta}, step={step}",
            )
        mean_uses += used
        state = tuple(fields["true_" + name] for name in ("x", "y", "yaw", "v"))
        for name, actual in zip(("x", "y", "yaw", "v"), state):
            close(actual, row[name], 5.1e-10)
        brake = (
            truth(row["stop_latched"])
            or truth(row["safety_paused"])
            or truth(row["guard_active"])
            or truth(row["speed_braking"])
        )
        acceleration = (
            max(-1.0, min(0.0, -4 * velocity))
            if brake
            else fields["applied_target"] - velocity
        )
        close(fields["applied_acceleration"], acceleration)
        for name, legacy in (
            ("applied_acceleration", "acceleration"),
            ("applied_target", "target_speed"),
            ("applied_steering", "steering"),
        ):
            close(fields[name], row[legacy], 5.1e-10)
        close(state[3], velocity + acceleration * DT)
        close(state[0], previous_state[0] + velocity * math.cos(previous_state[2]) * DT)
        close(state[1], previous_state[1] + velocity * math.sin(previous_state[2]) * DT)
        yaw = (
            previous_state[2]
            + velocity / 0.4 * math.tan(fields["applied_steering"]) * DT
        )
        error = state[2] - yaw
        build.require(
            abs(math.atan2(math.sin(error), math.cos(error))) <= 1e-12,
            "Yaw dynamics differ",
        )
        segment = previous_state[:2], state[:2]
        contact = (
            cached_contact if segment == cached_segment else spatial.contact(*segment)
        )
        cached_segment, cached_contact = segment, contact
        build.require(
            contact == (finite(row["step_min_obstacle_distance"]) <= 0.5),
            "Swept contact disagrees with native result",
        )
        contacts += contact
        boundary = any(not 0 <= coordinate <= 50 for coordinate in state[:2])
        boundaries += boundary
        if contact or boundary:
            build.require(
                step == len(sides), "Native run continued after contact/boundary exit"
            )
        true_goal = math.sqrt((state[0] - goal[0]) ** 2 + (state[1] - goal[1]) ** 2)
        hold = (
            hold + DT
            if truth(row["stop_latched"]) and true_goal < 0.25 and abs(state[3]) <= 0.02
            else 0.0
        )
        close(row["hold_time_s"], hold, 1e-7)
        paused_time += DT if truth(row["safety_paused"]) else 0
        pauses += missing and not truth(before["safety_paused"])
        resumes += truth(row["resumed"])
        rms += (
            finite(before["tracking_error"]) ** 2 + finite(row["tracking_error"]) ** 2
        ) * 0.01
        min_clearance = min(min_clearance, finite(row["step_min_obstacle_distance"]))
        previous_state = state
    exact = [dict(row) for row in track]
    for before, row in zip(exact, exact[1:]):
        before["v"] = row["measured_speed"]
    check_joint_track(exact)
    check_guard_track(exact)
    check_motion_filter_track(track, True, True)
    status = (
        "collision"
        if contacts
        else (
            "out_of_bounds"
            if boundaries
            else "held" if hold + 1e-9 >= 10 else "timeout"
        )
    )
    if status == "timeout":
        build.require(len(sides) == 9000, "Timeout did not reach the fixed horizon")
    build.require(
        outcome["status"] == status and truth(outcome["held"]) == (status == "held"),
        "Native endpoint status/hold differs",
    )
    build.require(
        truth(outcome["collision"]) == bool(contacts), "Native contact flag differs"
    )
    close(outcome["time_s"], len(sides) * DT, 5.1e-7)
    close(outcome["goal_distance"], true_goal, 1e-8)
    close(outcome["final_speed"], previous_state[3], 1e-8)
    close(outcome["paused_time_s"], paused_time, 1e-7)
    close(outcome["rms_tracking_error"], math.sqrt(rms / (len(sides) * DT)), 1e-7)
    close(outcome["min_obstacle_distance"], min_clearance, 1e-7)
    build.require(
        int(outcome["pause_count"]) == pauses
        and int(outcome["resume_count"]) == resumes,
        "Pause/resume counts differ",
    )
    case = dict(
        **meta,
        **{
            k: outcome[k]
            for k in (
                "status",
                "held",
                "collision",
                "time_s",
                "goal_distance",
                "final_speed",
                "paused_time_s",
                "pause_count",
                "resume_count",
            )
        },
        boundary=str(bool(boundaries)).lower(),
        decisions=len(sides),
        mean_used_steps=mean_uses,
        geometry_samples=len(selected),
        contact_segments=contacts,
        boundary_segments=boundaries,
    )
    checked = dict(
        **meta,
        physical_steps=len(sides),
        sidecar_decisions=len(sides),
        geometry_samples=len(selected),
        mean_used_steps=mean_uses,
        contact_segments=contacts,
        boundary_segments=boundaries,
        native_hold_s=hold,
    )
    return case, checked


def pair(meta, raw, candidate, raw_sides, mean_sides, raw_case, mean_case):
    common = min(len(raw), len(candidate))
    changed = next((i for i in range(common) if raw[i] != candidate[i]), None)
    if changed is None:
        build.require(
            len(raw) == len(candidate),
            "A common trajectory prefix ended without an earlier decision change",
        )
    else:
        build.require(
            changed > 0 and raw[changed - 1] == candidate[changed - 1],
            "Initial state/common prefix differs",
        )
        source, target = raw_sides[changed - 1], mean_sides[changed - 1]
        for name in (
            "raw_x",
            "raw_y",
            "raw_yaw",
            "measured_speed",
            "observation_age",
            "count",
            "mean_x",
            "mean_y",
            "steering",
            "margin",
            "uncapped_target",
        ):
            build.require(
                source[name] == target[name], f"First-divergence input differs: {name}"
            )
        build.require(
            truth(target["used"]), "First changed row has no mean-position use"
        )
    for left, right in zip(raw[1:], candidate[1:]):
        for name in (
            "loss_uniform",
            "observation_available",
            "noise_rho",
            "noise_innovation_x",
            "noise_innovation_y",
            "noise_innovation_yaw",
        ):
            build.require(
                left[name] == right[name],
                f"Paired latent observation stream differs: {name}",
            )
    if meta["mode"] == "none":
        build.require(
            raw == candidate and not mean_case["mean_used_steps"],
            "No-loss control differs",
        )
    held_raw, held_mean = truth(raw_case["held"]), truth(mean_case["held"])
    outcome = (
        "both_held"
        if held_raw and held_mean
        else "gained_hold" if held_mean else "lost_hold" if held_raw else "neither_held"
    )
    return dict(
        **{k: meta[k] for k in ("tau", "mode", "seed", "scenario", "planner")},
        raw_status=raw_case["status"],
        mean_status=mean_case["status"],
        raw_held=raw_case["held"],
        mean_held=mean_case["held"],
        raw_collision=raw_case["collision"],
        mean_collision=mean_case["collision"],
        raw_boundary=raw_case["boundary"],
        mean_boundary=mean_case["boundary"],
        raw_goal_distance=raw_case["goal_distance"],
        mean_goal_distance=mean_case["goal_distance"],
        raw_time_s=raw_case["time_s"],
        mean_time_s=mean_case["time_s"],
        outcome=outcome,
        first_difference_step=changed if changed is not None else "",
        common_steps=common - 1,
        raw_common_paused_s=sum(truth(r["safety_paused"]) for r in raw[1:common]) * DT,
        mean_common_paused_s=sum(truth(r["safety_paused"]) for r in candidate[1:common])
        * DT,
        mean_used_steps=mean_case["mean_used_steps"],
    )


def summarize(pairs):
    result = []
    for noise in ("all", "white", "correlated"):
        for mode in ("all", "none", "random", "blackout"):
            chosen = [
                p
                for p in pairs
                if (
                    noise == "all" or finite(p["tau"]) == (0 if noise == "white" else 1)
                )
                and (mode == "all" or p["mode"] == mode)
            ]
            cell = dict(noise_selector=noise, mode_selector=mode, pairs=len(chosen))
            for arm in ("raw", "mean"):
                cell[arm + "_held"] = sum(truth(p[arm + "_held"]) for p in chosen)
                cell[arm + "_contacts"] = sum(
                    truth(p[arm + "_collision"]) for p in chosen
                )
                cell[arm + "_boundaries"] = sum(
                    truth(p[arm + "_boundary"]) for p in chosen
                )
                cell[arm + "_timeouts"] = sum(
                    p[arm + "_status"] == "timeout" for p in chosen
                )
                cell[arm + "_common_paused_s"] = sum(
                    p[arm + "_common_paused_s"] for p in chosen
                )
                cell[arm + "_mean_goal_distance"] = (
                    sum(finite(p[arm + "_goal_distance"]) for p in chosen) / len(chosen)
                    if chosen
                    else ""
                )
            for outcome in ("gained_hold", "lost_hold", "both_held", "neither_held"):
                cell[outcome] = sum(p["outcome"] == outcome for p in chosen)
            cell["changed_pairs"] = sum(
                p["first_difference_step"] != "" for p in chosen
            )
            cell["mean_used_steps"] = sum(p["mean_used_steps"] for p in chosen)
            result.append(cell)
    return result


def check_provenance(raw, pins):
    provenance = json.loads((raw / "provenance.json").read_text())
    build.require(
        provenance["method"]
        == "fixed 108-case mean-position speed-probe pilot; original recovery window; no retuning",
        "Run method differs",
    )
    build.require(
        provenance["input_sha256"] == pins["input_sha256"]
        and provenance["cases"] == pins["expected_cases"],
        "Run selection/pins differ",
    )
    build.require(
        provenance["working_tree_clean"] is True,
        "Pilot was generated from an uncommitted implementation",
    )
    for name, key in (
        ("run_mean_speed_probe.py", "runner_sha256"),
        ("build_mean_speed_probe.py", "generator_sha256"),
    ):
        build.require(
            build.sha(build.ROOT / name) == provenance[key],
            f"Generation code differs: {name}",
        )
        committed = subprocess.check_output(
            [
                "git",
                "show",
                provenance["source_commit"]
                + ":experiments/pose_observation_loss/"
                + name,
            ],
            cwd=build.REPO,
        )
        build.require(
            hashlib.sha256(committed).hexdigest() == provenance[key],
            "Recorded implementation commit differs",
        )
    build.require(
        provenance["protocol_sha256"] == build.sha(build.STUDY / "protocol.md")
        and provenance["freeze_sha256"] == build.sha(build.STUDY / "freeze-pins.json"),
        "Protocol/freeze differs",
    )
    receipt = json.loads((raw / "build/build-sources.json").read_text())
    build.require(
        build.sha(raw / "build/build-sources.json")
        == provenance["build_sources_sha256"],
        "Build receipt differs",
    )
    build.require(
        (raw / "build/mean-speed-probe.rs").read_text()
        == build.generate(build.BASE.read_text()),
        "Generated source differs from the exact archived-source edits",
    )
    for path, key in (
        ("mean-speed-probe.rs", "generated_source_sha256"),
        ("candidate.diff", "diff_sha256"),
        ("mean-speed-probe", "candidate_binary_sha256"),
    ):
        build.require(
            build.sha(raw / "build" / path) == receipt[key],
            f"Built artifact differs: {path}",
        )
    build.require(
        receipt["generator_sha256"] == provenance["generator_sha256"]
        and receipt["input_sha256"] == pins["input_sha256"],
        "Build provenance differs",
    )
    build.require(
        receipt["candidate_binary_sha256"] == provenance["candidate_binary_sha256"]
        and receipt["generated_source_sha256"] == provenance["generated_source_sha256"],
        "Run/build executable differs",
    )
    build.require(
        build.sha(raw / "all-results.csv") == provenance["all_results_sha256"],
        "Aggregate native results differ",
    )
    expected = [
        (tau, mode, arm, pins["seed"])
        for tau in pins["noise_correlation_times"]
        for mode in pins["modes"]
        for arm in pins["arms"]
    ]
    build.require(
        [(t["tau"], t["mode"], t["arm"], t["seed"]) for t in provenance["trials"]]
        == expected,
        "Native trial grid differs",
    )
    for trial in provenance["trials"]:
        relative = f"tau{trial['tau']:g}/{trial['arm']}-{trial['mode']}-{trial['seed']}"
        build.require(trial["directory"] == relative, "Trial path/identity differs")
        for name, digest in trial["sha256"].items():
            build.require(
                build.sha(raw / relative / name) == digest,
                f"Raw file changed: {relative}/{name}",
            )
        expected_args = build.arguments(
            build.FIXTURES,
            raw / relative,
            trial["mode"],
            trial["seed"],
            trial["tau"],
            trial["arm"] == "mean_probe",
        )
        # Relocating the packet changes only the two recorded path arguments.
        build.require(
            trial["command"][3:] == expected_args[2:], "Native parameters differ"
        )
    reference = raw / f"tau0/raw_probe-none-{pins['seed']}"
    build.require(
        [r["name"] for r in provenance["regressions"]] == ["original", "omitted"],
        "Missing default/original regression",
    )
    for record in provenance["regressions"]:
        build.require(record["cases"] == 9, "Regression case count differs")
        for name in ("trajectories.csv", "results.csv"):
            build.require(
                build.sha(raw / "regression" / record["name"] / name)
                == record["sha256"][name]
                == build.sha(reference / name),
                "Default/original byte regression differs",
            )
    return provenance


def grouped(stream):
    reader = csv.DictReader(stream)
    build.require(
        reader.fieldnames and len(reader.fieldnames) == len(set(reader.fieldnames)),
        "Invalid native CSV header",
    )
    for key, rows in itertools.groupby(reader, lambda r: (r["scenario"], r["planner"])):
        group = list(rows)
        build.require(
            all(None not in r and None not in r.values() for r in group),
            "Malformed native row",
        )
        yield key, group


def audit(raw, output):
    pins = build.preflight()
    raw = Path(raw).resolve()
    output = build.fresh(output)
    provenance = check_provenance(raw, pins)
    output.mkdir(parents=True)
    obstacles = defaultdict(list)
    for i, row in enumerate(read(build.FIXTURES / "obstacles.csv"), 1):
        obstacles[row["scenario"]].append((i, finite(row["x"]), finite(row["y"])))
    paths = {(r["scenario"], r["planner"]) for r in read(build.FIXTURES / "paths.csv")}
    build.require(len(paths) == 9, "Fixture grid differs")
    all_cases, all_pairs, checks, samples = [], [], [], []
    native_rows = read(raw / "all-results.csv")
    native_index = {
        (
            finite(r["tau"]),
            r["arm"],
            r["mode"],
            int(r["seed"]),
            r["scenario"],
            r["planner"],
        ): r
        for r in native_rows
    }
    build.require(
        len(native_index) == len(native_rows) == pins["expected_cases"],
        "Duplicate/missing native case results",
    )
    try:
        for tau in pins["noise_correlation_times"]:
            for mode in pins["modes"]:
                directories = [
                    raw / f"tau{tau:g}/{arm}-{mode}-{pins['seed']}"
                    for arm in pins["arms"]
                ]
                with (directories[0] / "trajectories.csv").open() as rt, (
                    directories[1] / "trajectories.csv"
                ).open() as mt, (directories[0] / "speed-probes.csv").open() as rs, (
                    directories[1] / "speed-probes.csv"
                ).open() as ms:
                    seen = []
                    for groups in itertools.zip_longest(
                        grouped(rt), grouped(mt), grouped(rs), grouped(ms)
                    ):
                        build.require(
                            all(g is not None for g in groups)
                            and len({g[0] for g in groups}) == 1,
                            "Paired native case grouping differs",
                        )
                        key = groups[0][0]
                        seen.append(key)
                        data = [g[1] for g in groups]
                        cases = []
                        for i, arm in enumerate(pins["arms"]):
                            meta = dict(
                                tau=tau,
                                arm=arm,
                                mode=mode,
                                seed=pins["seed"],
                                scenario=key[0],
                                planner=key[1],
                            )
                            old = native_index[(tau, arm, mode, pins["seed"], *key)]
                            native_outcomes = {
                                (r["scenario"], r["planner"]): r
                                for r in read(directories[i] / "results.csv")
                            }
                            build.require(
                                all(
                                    old[k] == native_outcomes[key][k]
                                    for k in native_outcomes[key]
                                ),
                                "Native aggregate/result row differs",
                            )
                            case, checked = audit_track(
                                meta,
                                data[i],
                                data[i + 2],
                                old,
                                obstacles[key[0]],
                                samples,
                            )
                            cases.append(case)
                            all_cases.append(case)
                            checks.append(checked)
                        meta = dict(
                            tau=tau,
                            mode=mode,
                            seed=pins["seed"],
                            scenario=key[0],
                            planner=key[1],
                        )
                        all_pairs.append(
                            pair(meta, data[0], data[1], data[2], data[3], *cases)
                        )
                        print(
                            f"Audited tau={tau:g} mode={mode} {key[0]}/{key[1]}",
                            flush=True,
                        )
                    build.require(
                        len(seen) == len(set(seen)) == 9 and set(seen) == paths,
                        "Incomplete paired case grid",
                    )
        build.require(
            len(all_cases) == pins["expected_cases"]
            and len(all_pairs) == pins["expected_pairs"],
            "Audited population differs",
        )
        tables = [
            ("cases.csv", all_cases, CASE_FIELDS),
            ("pairs.csv", all_pairs, PAIR_FIELDS),
            ("summary.csv", summarize(all_pairs), SUMMARY_FIELDS),
            ("verification.csv", checks, VERIFY_FIELDS),
        ]
        for name, rows, fields in tables:
            write(output / name, rows, fields)
        sources = dict(
            method="complete full-precision sidecar/window, dynamics/contact/hold and inherited gate/filter audit; predeclared sampled probe geometry",
            analyzer_sha256=build.sha(Path(__file__)),
            protocol_sha256=build.sha(build.STUDY / "protocol.md"),
            freeze_sha256=build.sha(build.STUDY / "freeze-pins.json"),
            input_sha256=pins["input_sha256"],
            raw_provenance_sha256=build.sha(raw / "provenance.json"),
            implementation_commit=provenance["source_commit"],
            audit=dict(
                cases=len(all_cases),
                pairs=len(all_pairs),
                physical_steps=sum(r["physical_steps"] for r in checks),
                sidecar_decisions=sum(r["sidecar_decisions"] for r in checks),
                geometry_samples=len(samples),
                regression_cases=18,
            ),
            output_sha256={name: build.sha(output / name) for name, _, _ in tables},
        )
    except (ValueError, KeyError, AssertionError) as error:
        (output / "failure.txt").write_text(str(error) + "\n")
        raise
    finally:
        write(output / "probe-samples.csv", samples, SAMPLE_FIELDS)
    sources["output_sha256"]["probe-samples.csv"] = build.sha(
        output / "probe-samples.csv"
    )
    (output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(
        f"Verified all {len(all_cases)} native cases and {len(samples)} sampled probes: {output}",
        flush=True,
    )
    return sources


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not __debug__:
        parser.error("Run without Python optimization")
    try:
        audit(args.raw, args.output)
    except (
        OSError,
        ValueError,
        KeyError,
        AssertionError,
        subprocess.CalledProcessError,
    ) as error:
        parser.exit(1, f"Mean speed-probe audit failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
