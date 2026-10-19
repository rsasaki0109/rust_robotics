#!/usr/bin/env python3
"""Audit all physical histories, one-shot handoffs and release follow-up windows."""

import argparse
from collections import defaultdict
from contextlib import ExitStack
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import types

import analyze_mean_guard_preview as previous
import analyze_mean_speed_probe as pilot
import build_release_mean_handoff as build
import native_release_mean_handoff as native
import run_release_mean_handoff as runner

need = build.require
HORIZONS = (1, 2, 5, 25, 50)
HANDOFF_COUNTS = (
    "handoff_available_steps",
    "handoff_invalidated_steps",
    "handoff_used_steps",
    "handoff_seeded_steps",
    "handoff_changed_steps",
    "zero_target_handoff_steps",
)
FOLLOWUP_COUNTS = (
    "guard_triggers",
    "guard_releases",
    "release_records",
    "lag2_entries",
    *("entries_within_" + str(h) for h in HORIZONS),
    *("followup_at_least_" + str(h) for h in HORIZONS),
)
EXTRA_COUNTS = (*HANDOFF_COUNTS, *FOLLOWUP_COUNTS)
CASE_FIELDS = (*previous.CASE_FIELDS, *EXTRA_COUNTS)
VERIFY_FIELDS = (*previous.VERIFY_FIELDS, *EXTRA_COUNTS)
PAIR_EXTRA_FIELDS = (
    "candidate_guard_used_steps",
    *(arm + "_" + k for arm in ("reference", "candidate") for k in EXTRA_COUNTS),
)
EVENT_FIELDS = (
    "case_id",
    *pilot.CASE_META,
    "status",
    "resume_step",
    "resume_time_s",
    "next_available",
    "next_valid",
    "next_used",
    "followup_steps",
    "end_reason",
    "first_entry_step",
    "first_entry_lag",
    *("entry_within_" + str(h) for h in HORIZONS),
    *("observed_" + str(h) for h in HORIZONS),
)


def case_id(meta):
    return previous.case_id(meta)


def sample_steps(track, guards, handoffs):
    steps = previous.guard_sample_steps(guards) | pilot.geometry_sample_steps(handoffs)
    for i, row in enumerate(track[1:], 1):
        if pilot.truth(row["resumed"]):
            steps.update(range(i, min(len(track) - 1, i + 50) + 1))
    return steps


def validate_handoff(meta, track, sides, guards, handoffs):
    need(
        len(track) == len(sides) + 1 == len(guards) + 1 == len(handoffs) + 1,
        "Incomplete handoff case",
    )
    projected_track = [dict(row) for row in track]
    projected_guards = [dict(row) for row in guards]
    counts = dict.fromkeys(HANDOFF_COUNTS, 0)
    snapshot = None
    for step, (row, side, guard, handoff) in enumerate(
        zip(track[1:], sides, guards, handoffs), 1
    ):
        need(tuple(handoff) == build.HANDOFF_FIELDS, "Handoff schema differs")
        need(
            (handoff["scenario"], handoff["planner"])
            == (meta["scenario"], meta["planner"])
            and int(handoff["step"]) == step,
            "Handoff row join differs",
        )
        need(handoff["time_s"] == side["time_s"], "Handoff time differs")
        enabled = meta["arm"] == "one_step_handoff"
        need(pilot.truth(handoff["enabled"]) == enabled, "Handoff arm differs")
        need(
            pilot.truth(guard["enabled"]),
            "Both original guard selectors must be enabled",
        )
        for name, source in (
            ("correction", "correction"),
            ("received", "received"),
            ("precision_activation", "precision_activation"),
            ("observation_age", "observation_age"),
            ("count", "count"),
        ):
            need(handoff[name] == side[source], "Handoff event/window input differs")
        need(
            handoff["joint_pending"] == guard["joint_pending"],
            "Handoff joint input differs",
        )
        available = snapshot is not None
        valid = (
            not any(
                pilot.truth(handoff[k])
                for k in (
                    "joint_pending",
                    "correction",
                    "received",
                    "precision_activation",
                )
            )
            and abs(pilot.finite(side["measured_speed"])) <= 0.02
            and pilot.finite(side["observation_age"]) <= 1.5
            and int(side["count"]) >= 10
        )
        used = enabled and available and valid
        seeded = enabled and pilot.truth(row["resumed"])
        for name, expected in (
            ("available", available),
            ("valid", valid),
            ("used", used),
            ("seeded", seeded),
        ):
            need(pilot.truth(handoff[name]) == expected, f"Handoff {name} differs")
        carry = snapshot or dict.fromkeys(("x", "y", "yaw", "speed", "steering"), 0.0)
        for name, expected in carry.items():
            need(
                pilot.finite(handoff["carry_" + name]) == expected,
                "Handoff snapshot differs from previous resume",
            )
        for name in ("x", "y", "yaw", "v"):
            original = (
                pilot.finite(side["mean_" + name])
                if pilot.truth(guard["used"]) and name in ("x", "y")
                else pilot.finite(guard["base_" + name])
            )
            need(
                pilot.finite(handoff["before_" + name]) == original,
                "Before-handoff pose differs",
            )
            selected = original
            if used and name in ("x", "y"):
                selected = carry[name] + carry["speed"] * pilot.DT * (
                    math.cos(carry["yaw"]) if name == "x" else math.sin(carry["yaw"])
                )
            pilot.close(handoff["selected_" + name], selected)
            need(
                handoff["selected_" + name] == guard["selected_" + name],
                "Final guard/handoff pose differs",
            )
            if name in ("yaw", "v") or not used:
                need(
                    handoff["selected_" + name] == handoff["before_" + name],
                    "Handoff changed a fixed field",
                )
            if name in ("x", "y"):
                projected_track[step]["preview_pose_" + name] = f"{original:.9f}"
                projected_guards[step - 1]["selected_" + name] = handoff[
                    "before_" + name
                ]
            if name != "v":
                need(
                    previous.terminal.admission.rounded_matches(
                        row["preview_pose_" + name],
                        pilot.finite(handoff["selected_" + name]),
                        9,
                    ),
                    "Actual handoff pose rounding differs",
                )
        counts["handoff_available_steps"] += available
        counts["handoff_invalidated_steps"] += available and not valid
        counts["handoff_used_steps"] += used
        counts["handoff_seeded_steps"] += seeded
        counts["handoff_changed_steps"] += used and any(
            handoff["before_" + k] != handoff["selected_" + k] for k in ("x", "y")
        )
        counts["zero_target_handoff_steps"] += (
            used and pilot.finite(side["nominal_target"]) == 0
        )
        snapshot = (
            dict(
                x=pilot.finite(side["mean_x"]),
                y=pilot.finite(side["mean_y"]),
                yaw=pilot.finite(side["mean_yaw"]),
                speed=pilot.finite(side["measured_speed"]),
                steering=pilot.finite(side["applied_steering"]),
            )
            if seeded
            else None
        )
    counts.update(
        previous.validate_guard(
            dict(meta, arm="mean_preview"), projected_track, sides, projected_guards
        )
    )
    counts["native_geometry_queries"] = len(sample_steps(track, guards, handoffs))
    need(
        counts["handoff_used_steps"] + counts["handoff_invalidated_steps"]
        == counts["handoff_available_steps"],
        "Snapshot consumption is not conserved",
    )
    if meta["mode"] == "none":
        need(
            not counts["handoff_used_steps"] and not counts["guard_used_steps"],
            "No-loss selector use differs",
        )
    return counts


def release_followups(meta, track, guards, handoffs, status):
    events = []
    for i, row in enumerate(track[1:], 1):
        if not pilot.truth(row["resumed"]):
            continue
        need(
            not pilot.truth(row["guard_active"])
            and not pilot.truth(row["safety_paused"]),
            "Joint resume did not clear both brakes",
        )
        end, reason = len(track) - 1, "packet_endpoint"
        for j in range(i + 1, len(track)):
            if pilot.truth(guards[j - 1]["joint_pending"]):
                end, reason = j - 1, "next_joint_pause"
                break
        first = next(
            (
                j
                for j in range(i + 1, end + 1)
                if pilot.truth(track[j]["guard_triggered"])
            ),
            None,
        )
        lag = first - i if first is not None else None
        following = handoffs[i] if i < len(handoffs) else None
        event = dict(
            case_id=case_id(meta),
            **meta,
            status=status,
            resume_step=i,
            resume_time_s=row["time_s"],
            next_available=following["available"] if following else "",
            next_valid=following["valid"] if following else "",
            next_used=following["used"] if following else "",
            followup_steps=end - i,
            end_reason=reason,
            first_entry_step=first if first is not None else "",
            first_entry_lag=lag if lag is not None else "",
        )
        for h in HORIZONS:
            event["entry_within_" + str(h)] = lag is not None and lag <= h
            event["observed_" + str(h)] = end - i >= h
        events.append(event)
    return events


def audit_track(
    meta, track, sides, guards, handoffs, outcome, obstacles, samples, events
):
    counts = validate_handoff(meta, track, sides, guards, handoffs)
    namespace = dict(previous.audit_track.__globals__, validate_guard=lambda *_: counts)
    original = types.FunctionType(previous.audit_track.__code__, namespace)
    local_samples = []
    case, check = original(
        dict(meta, arm="mean_preview"),
        track,
        sides,
        guards,
        outcome,
        obstacles,
        local_samples,
    )
    for sample in local_samples:
        sample["arm"] = meta["arm"]
    samples.extend(local_samples)
    releases = release_followups(meta, track, guards, handoffs, outcome["status"])
    need(
        len(releases) == int(outcome["resume_count"]),
        "Release follow-up population differs",
    )
    events.extend(releases)
    extras = dict(
        guard_triggers=sum(pilot.truth(r["guard_triggered"]) for r in track[1:]),
        guard_releases=sum(pilot.truth(r["guard_released"]) for r in track[1:]),
        release_records=len(releases),
        lag2_entries=sum(e["first_entry_lag"] == 2 for e in releases),
    )
    for h in HORIZONS:
        extras["entries_within_" + str(h)] = sum(
            e["entry_within_" + str(h)] for e in releases
        )
        extras["followup_at_least_" + str(h)] = sum(
            e["observed_" + str(h)] for e in releases
        )
    for result in (case, check):
        result.update(arm=meta["arm"], case_id=case_id(meta), **extras)
    return case, check


def history_row(sequence, ident, meta, step, row, side, guard, handoff):
    result = previous.history_row(sequence, ident, meta, step, row, side, guard)
    result.update(
        dict(
            zip(
                native.EXTRA_FIELDS,
                (
                    handoff["enabled"],
                    handoff["available"],
                    handoff["valid"],
                    handoff["used"],
                    handoff["seeded"],
                    row["resumed"],
                    handoff["before_x"],
                    handoff["before_y"],
                    handoff["before_yaw"],
                ),
            )
        )
    )
    need(
        tuple(result) == native.HISTORY_FIELDS, "Handoff native history schema differs"
    )
    return result


def renamed_key(key):
    return previous.renamed_key(key)


def renamed(row):
    return previous.renamed(row)


def paired_core(
    meta, raw, candidate, raw_sides, mean_sides, handoffs, raw_case, mean_case
):
    common = min(len(raw), len(candidate))
    changed = next((i for i in range(common) if raw[i] != candidate[i]), None)
    if changed is None:
        need(
            len(raw) == len(candidate),
            "A common trajectory prefix ended without an earlier decision change",
        )
    else:
        need(
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
            need(
                source[name] == target[name], f"First-divergence input differs: {name}"
            )
        need(
            pilot.truth(handoffs[changed - 1]["used"]),
            "First changed row has no handoff use",
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
            need(
                left[name] == right[name],
                f"Paired latent observation stream differs: {name}",
            )
    if meta["mode"] == "none":
        need(
            raw == candidate and not mean_case["mean_used_steps"],
            "No-loss control differs",
        )
    held_raw, held_mean = pilot.truth(raw_case["held"]), pilot.truth(mean_case["held"])
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
        raw_common_paused_s=sum(pilot.truth(r["safety_paused"]) for r in raw[1:common])
        * pilot.DT,
        mean_common_paused_s=sum(
            pilot.truth(r["safety_paused"]) for r in candidate[1:common]
        )
        * pilot.DT,
        mean_used_steps=mean_case["mean_used_steps"],
    )


def pair(
    meta,
    left,
    right,
    left_sides,
    right_sides,
    left_guards,
    right_guards,
    left_handoffs,
    right_handoffs,
    left_case,
    right_case,
):
    old_cases = [
        dict(c, mean_used_steps=c["speed_used_steps"]) for c in (left_case, right_case)
    ]
    result = paired_core(
        meta, left, right, left_sides, right_sides, right_handoffs, *old_cases
    )
    changed = result["first_difference_step"]
    if changed != "":
        for key in ("before_x", "before_y", "before_yaw", "before_v"):
            need(
                left_handoffs[changed - 1][key] == right_handoffs[changed - 1][key],
                "Pre-handoff first-divergence input differs",
            )
    if meta["mode"] == "none":
        need(
            left_sides == right_sides
            and not left_case["guard_used_steps"]
            and not right_case["guard_used_steps"]
            and not right_case["handoff_used_steps"],
            "No-loss handoff control differs",
        )
    result["candidate_guard_used_steps"] = right_case["guard_used_steps"]
    for prefix, case in (("reference", left_case), ("candidate", right_case)):
        result.update({prefix + "_" + field: case[field] for field in EXTRA_COUNTS})
    return result


def check_provenance(raw, pins):
    provenance = json.loads((raw / "provenance.json").read_text())
    need(
        provenance["method"] == runner.METHOD
        and provenance["input_sha256"] == pins["input_sha256"]
        and provenance["cases"] == pins["expected_cases"]
        and provenance["working_tree_clean"] is True,
        "Run selection/pins differ",
    )
    need(
        set(provenance["script_sha256"]) == set(build.SCRIPTS),
        "Run script coverage differs",
    )
    for name, digest in provenance["script_sha256"].items():
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
        need(
            hashlib.sha256(committed).hexdigest()
            == digest
            == build.sha(build.ROOT / name),
            f"Implementation differs: {name}",
        )
    for name, key in (
        ("protocol.md", "protocol_sha256"),
        ("freeze-pins.json", "freeze_sha256"),
    ):
        need(build.sha(build.STUDY / name) == provenance[key], "Run protocol differs")
    receipt = json.loads((raw / "build/build-sources.json").read_text())
    need(
        build.sha(raw / "build/build-sources.json")
        == provenance["build_sources_sha256"],
        "Build receipt changed",
    )
    need(
        receipt["input_sha256"] == pins["input_sha256"]
        and receipt["generator_sha256"]
        == provenance["script_sha256"]["build_mean_guard_preview.py"],
        "Build inputs differ",
    )
    need(
        receipt["native_tests"] == provenance["native_tests"] == 35,
        "Native test coverage differs",
    )
    for filename, key in (
        ("release-mean-handoff.rs", "generated_source_sha256"),
        ("candidate.diff", "diff_sha256"),
        ("release-mean-handoff", "candidate_binary_sha256"),
        ("native-tests.log", "native_tests_sha256"),
        ("legacy/build-sources.json", "legacy_receipt_sha256"),
        ("legacy/mean-guard-preview", "legacy_binary_sha256"),
        ("legacy/original/pose_observation_loss", "original_binary_sha256"),
    ):
        need(
            build.sha(raw / "build" / filename) == receipt[key],
            f"Build artifact changed: {filename}",
        )
    need(
        (raw / "build/release-mean-handoff.rs").read_text()
        == build.generate(build.BASE.read_text()),
        "Generated live source differs",
    )
    old_receipt = json.loads((raw / "build/legacy/build-sources.json").read_text())
    need(
        old_receipt["libraries"] == receipt["libraries"]
        and (raw / "build/legacy/mean-guard-preview.rs").read_text()
        == build.legacy.generate(build.BASE.read_text()),
        "Legacy build inputs differ",
    )
    for key in (
        "candidate_binary_sha256",
        "legacy_binary_sha256",
        "original_binary_sha256",
        "generated_source_sha256",
    ):
        need(receipt[key] == provenance[key], "Run executable differs")
    need(
        old_receipt["candidate_binary_sha256"] == receipt["legacy_binary_sha256"]
        and old_receipt["original_binary_sha256"] == receipt["original_binary_sha256"],
        "Legacy executable differs",
    )
    need(
        build.sha(raw / "all-results.csv") == provenance["all_results_sha256"],
        "Aggregate results changed",
    )
    expected = [
        (tau, mode, arm, pins["seed"])
        for tau in pins["noise_correlation_times"]
        for mode in pins["modes"]
        for arm in pins["arms"]
    ]
    need(
        [(t["tau"], t["mode"], t["arm"], t["seed"]) for t in provenance["trials"]]
        == expected,
        "Live grid differs",
    )
    for trial in provenance["trials"]:
        relative = f"tau{trial['tau']:g}/{trial['arm']}-{trial['mode']}-{trial['seed']}"
        need(
            trial["directory"] == relative
            and set(trial["sha256"]) == set(runner.FILES),
            "Live trial path/files differ",
        )
        for name, digest in trial["sha256"].items():
            need(
                build.sha(raw / relative / name) == digest,
                f"Raw live file changed: {relative}/{name}",
            )
        args = build.arguments(
            build.FIXTURES,
            raw / relative,
            trial["mode"],
            trial["seed"],
            trial["tau"],
            trial["arm"] == "one_step_handoff",
        )
        need(
            len(trial["command"]) == 32
            and trial["command"][3:] == args[2:]
            and Path(trial["command"][0]).name == "release-mean-handoff",
            "Live command differs",
        )
    specs = runner.regression_plan(pins)
    need(
        len(provenance["regressions"]) == len(specs)
        and sum(r["cases"] for r in provenance["regressions"])
        == pins["expected_regression_cases"],
        "Regression grid differs",
    )
    for record, spec in zip(provenance["regressions"], specs):
        need(
            all(record[k] == v for k, v in spec.items())
            and record["cases"] == 9
            and record["directory"] == "regression/" + spec["name"],
            "Regression identity differs",
        )
        directory = raw / record["directory"]
        kind, mode, tau = spec["kind"], spec["mode"], spec["tau"]
        expected_files = {
            name
            for name in runner.FILES
            if (kind == "omitted" or name != "handoff-inputs.csv")
            and (
                kind != "original"
                or name in ("trajectories.csv", "results.csv", "simulation.log")
            )
        }
        need(
            set(record["sha256"]) == expected_files, "Regression file coverage differs"
        )
        for name, digest in record["sha256"].items():
            need(build.sha(directory / name) == digest, "Regression raw hash differs")
        compared = runner.BYTE_FILES[:2] if kind == "original" else runner.BYTE_FILES
        need(
            record["compared"] == list(compared),
            "Regression comparison coverage differs",
        )
        reference = raw / f"tau{tau:g}/mean_preview-{mode}-{pins['seed']}"
        for name in compared:
            need(
                build.sha(reference / name) == record["sha256"][name],
                "Regression byte comparison differs",
            )
        if kind == "omitted":
            args = build.arguments(build.FIXTURES, directory, mode, pins["seed"], tau)
        elif kind == "legacy":
            args = build.legacy.arguments(
                build.FIXTURES, directory, mode, pins["seed"], tau, True
            )
        else:
            args = build.legacy.legacy.arguments(
                build.FIXTURES, directory, mode, pins["seed"], tau
            )
        binary = {
            "omitted": "release-mean-handoff",
            "legacy": "mean-guard-preview",
            "original": "pose_observation_loss",
        }[kind]
        need(
            record["command"][3:] == args[2:]
            and len(record["command"]) == len(args) + 1
            and Path(record["command"][0]).name == binary,
            "Regression command differs",
        )
    for tau in pins["noise_correlation_times"]:
        paths = [raw / f"tau{tau:g}/{arm}-none-{pins['seed']}" for arm in pins["arms"]]
        for filename in ("trajectories.csv", "speed-probes.csv"):
            need(
                build.sha(paths[0] / filename) == build.sha(paths[1] / filename),
                "No-loss byte control differs",
            )
    return provenance


def audit(raw, output):
    pins = build.preflight()
    raw, output = Path(raw).resolve(), build.fresh(output)
    need(not output.is_relative_to(raw), "Output must be outside retained raw inputs")
    provenance = check_provenance(raw, pins)
    output.mkdir(parents=True)
    cases, pairs, checks, samples, events = [], [], [], [], []
    try:
        work = output / "native"
        compiler = native.compile_native(work)
        obstacles = defaultdict(list)
        for i, row in enumerate(pilot.read(build.FIXTURES / "obstacles.csv"), 1):
            obstacles[row["scenario"]].append(
                (i, pilot.finite(row["x"]), pilot.finite(row["y"]))
            )
        paths = {
            (r["scenario"], r["planner"])
            for r in pilot.read(build.FIXTURES / "paths.csv")
        }
        need(len(paths) == 9, "Fixture grid differs")
        native_rows = pilot.read(raw / "all-results.csv")
        index = {
            (
                pilot.finite(r["tau"]),
                r["arm"],
                r["mode"],
                int(r["seed"]),
                r["scenario"],
                r["planner"],
            ): r
            for r in native_rows
        }
        need(
            len(index) == len(native_rows) == pins["expected_cases"],
            "Native result grid differs",
        )
        sequence = selected = 0
        with (work / "history.tsv").open("x", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=native.HISTORY_FIELDS,
                delimiter="\t",
                lineterminator="\n",
            )
            writer.writeheader()
            for tau in pins["noise_correlation_times"]:
                for mode in pins["modes"]:
                    directories = [
                        raw / f"tau{tau:g}/{arm}-{mode}-{pins['seed']}"
                        for arm in pins["arms"]
                    ]
                    results = [
                        {
                            (r["scenario"], r["planner"]): r
                            for r in pilot.read(d / "results.csv")
                        }
                        for d in directories
                    ]
                    with ExitStack() as stack:
                        streams = [
                            stack.enter_context((d / name).open())
                            for name in (
                                "trajectories.csv",
                                "speed-probes.csv",
                                "guard-preview-inputs.csv",
                                "handoff-inputs.csv",
                            )
                            for d in directories
                        ]
                        seen = []
                        for groups in itertools.zip_longest(
                            *(pilot.grouped(s) for s in streams)
                        ):
                            need(
                                all(g is not None for g in groups)
                                and len({g[0] for g in groups}) == 1,
                                "Paired native grouping differs",
                            )
                            key = groups[0][0]
                            seen.append(key)
                            data = [g[1] for g in groups]
                            local_cases = []
                            for i, arm in enumerate(pins["arms"]):
                                meta = dict(
                                    tau=tau,
                                    arm=arm,
                                    mode=mode,
                                    seed=pins["seed"],
                                    scenario=key[0],
                                    planner=key[1],
                                )
                                outcome = index[tau, arm, mode, pins["seed"], *key]
                                need(
                                    key in results[i]
                                    and all(
                                        outcome[k] == v
                                        for k, v in results[i][key].items()
                                    ),
                                    "Native aggregate/result join differs",
                                )
                                case, check = audit_track(
                                    meta,
                                    data[i],
                                    data[i + 2],
                                    data[i + 4],
                                    data[i + 6],
                                    outcome,
                                    obstacles[key[0]],
                                    samples,
                                    events,
                                )
                                cases.append(case)
                                checks.append(check)
                                local_cases.append(case)
                                selected_steps = sample_steps(
                                    data[i], data[i + 4], data[i + 6]
                                )
                                for step, (row, side, guard, handoff) in enumerate(
                                    zip(
                                        data[i][1:],
                                        data[i + 2],
                                        data[i + 4],
                                        data[i + 6],
                                    ),
                                    1,
                                ):
                                    sequence += 1
                                    ident = 0
                                    if step in selected_steps:
                                        selected += 1
                                        ident = selected
                                    writer.writerow(
                                        history_row(
                                            sequence,
                                            ident,
                                            meta,
                                            step,
                                            row,
                                            side,
                                            guard,
                                            handoff,
                                        )
                                    )
                            pairs.append(
                                pair(
                                    dict(
                                        tau=tau,
                                        mode=mode,
                                        seed=pins["seed"],
                                        scenario=key[0],
                                        planner=key[1],
                                    ),
                                    *data,
                                    *local_cases,
                                )
                            )
                            print(
                                f"Audited tau={tau:g} mode={mode} {key[0]}/{key[1]}",
                                flush=True,
                            )
                        need(
                            len(seen) == len(set(seen)) == 9
                            and set(seen) == paths
                            and all(set(r) == paths for r in results),
                            "Incomplete paired case grid",
                        )
        need(
            len(cases) == pins["expected_cases"]
            and len(pairs) == pins["expected_pairs"],
            "Audited population differs",
        )
        evaluation = native.evaluate(work, work / "history.tsv")
        histories = pilot.read(work / "history-checks.csv", native.CHECK_FIELDS)
        need(
            [r["case_id"] for r in histories] == [r["case_id"] for r in cases],
            "Native case order differs",
        )
        for history, case in zip(histories, cases):
            need(
                history["scenario"] == case["scenario"]
                and int(history["decisions"]) == case["decisions"]
                and int(history["guard_uses"]) == case["guard_used_steps"]
                and int(history["geometry_queries"]) == case["native_geometry_queries"],
                "Native coverage differs",
            )
        trace = pilot.read(work / "trace.csv", native.TRACE_FIELDS)
        need(
            len(trace) == selected and sum(c["decisions"] for c in cases) == sequence,
            "Native query/history population differs",
        )
        summary = pilot.summarize(pairs)
        for cell in summary:
            chosen = [
                p
                for p in pairs
                if (
                    cell["noise_selector"] == "all"
                    or float(p["tau"])
                    == (0 if cell["noise_selector"] == "white" else 1)
                )
                and (
                    cell["mode_selector"] == "all" or p["mode"] == cell["mode_selector"]
                )
            ]
            for field in PAIR_EXTRA_FIELDS:
                cell[field] = sum(p[field] for p in chosen)
        tables = (
            ("cases.csv", cases, CASE_FIELDS),
            ("release-followups.csv", events, EVENT_FIELDS),
            ("verification.csv", checks, VERIFY_FIELDS),
            (
                "pairs.csv",
                [renamed(p) for p in pairs],
                (*map(renamed_key, pilot.PAIR_FIELDS), *PAIR_EXTRA_FIELDS),
            ),
            (
                "summary.csv",
                [renamed(c) for c in summary],
                (*map(renamed_key, pilot.SUMMARY_FIELDS), *PAIR_EXTRA_FIELDS),
            ),
            ("probe-samples.csv", samples, pilot.SAMPLE_FIELDS),
            ("history-checks.csv", histories, native.CHECK_FIELDS),
            ("native-queries.csv", trace, native.TRACE_FIELDS),
        )
        for name, rows, fields in tables:
            pilot.write(output / name, rows, fields)
        sources = dict(
            method="complete physical/contact/hold/speed-window audit; full independent sensor-estimator and one-shot replay; fixed native geometry and complete release follow-up",
            implementation_commit=provenance["source_commit"],
            raw_provenance_sha256=build.sha(raw / "provenance.json"),
            protocol_sha256=build.sha(build.STUDY / "protocol.md"),
            freeze_sha256=build.sha(build.STUDY / "freeze-pins.json"),
            input_sha256=pins["input_sha256"],
            script_sha256=provenance["script_sha256"],
            native=dict(compiler=compiler, evaluation=evaluation),
            audit=dict(
                cases=len(cases),
                pairs=len(pairs),
                physical_steps=sequence,
                estimator_decisions=sequence,
                native_geometry_queries=selected,
                guard_uses=sum(c["guard_used_steps"] for c in cases),
                handoff_uses=sum(c["handoff_used_steps"] for c in cases),
                release_followups=len(events),
                speed_geometry_samples=len(samples),
                regression_cases=pins["expected_regression_cases"],
            ),
            output_sha256={name: build.sha(output / name) for name, _, _ in tables},
        )
        (output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    except (
        OSError,
        ValueError,
        KeyError,
        AssertionError,
        subprocess.CalledProcessError,
    ) as error:
        (output / "failure.txt").write_text(str(error) + "\n")
        raise
    print(
        f"Verified {len(cases)} cases, {sequence} estimator decisions and {selected} original native guard queries",
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
        parser.exit(1, f"Release mean handoff audit failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
