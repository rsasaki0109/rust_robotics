#!/usr/bin/env python3
"""Audit every live guard-preview case, estimator history and sampled native query."""

import argparse
from collections import defaultdict
from contextlib import ExitStack
import csv
import hashlib
import itertools
import json
from pathlib import Path
import subprocess
import types

import analyze_mean_speed_probe as pilot
import analyze_terminal_preview as terminal
import build_mean_guard_preview as build
import native_mean_guard_preview as native
import run_mean_guard_preview as runner

need = build.require
GUARD_COUNTS = (
    "guard_used_steps",
    "changed_position_steps",
    "positive_guard_uses",
    "zero_guard_uses",
    "native_geometry_queries",
)
CASE_FIELDS = (
    "case_id",
    *("speed_used_steps" if k == "mean_used_steps" else k for k in pilot.CASE_FIELDS),
    *GUARD_COUNTS,
)
VERIFY_FIELDS = (
    "case_id",
    *("speed_used_steps" if k == "mean_used_steps" else k for k in pilot.VERIFY_FIELDS),
    *GUARD_COUNTS,
)


def renamed_key(key):
    if key == "mean_used_steps":
        return "candidate_speed_used_steps"
    if key.startswith("raw_"):
        return "reference_" + key[4:]
    if key.startswith("mean_"):
        return "candidate_" + key[5:]
    return key


def renamed(row):
    return {renamed_key(k): v for k, v in row.items()}


def guard_sample_steps(guards):
    return pilot.geometry_sample_steps(guards) | {
        i for i, row in enumerate(guards, 1) if pilot.truth(row["used"])
    }


def validate_guard(meta, track, sides, guards):
    need(len(guards) == len(sides) == len(track) - 1, "Incomplete guard input log")
    counts = dict.fromkeys(GUARD_COUNTS, 0)
    for step, (before, row, side, guard) in enumerate(
        zip(track, track[1:], sides, guards), 1
    ):
        need(
            tuple(guard) == build.GUARD_FIELDS and len(row) == 82,
            "Guard/trajectory schema differs",
        )
        need(
            (guard["scenario"], guard["planner"])
            == (meta["scenario"], meta["planner"]),
            "Guard case join differs",
        )
        need(int(guard["step"]) == int(side["step"]) == step, "Guard step join differs")
        pilot.close(guard["time_s"], side["time_s"], 0)
        pilot.close(guard["time_s"], row["time_s"], 1e-12)
        enabled, used = pilot.truth(guard["enabled"]), pilot.truth(guard["used"])
        pending, speed_used = pilot.truth(guard["joint_pending"]), pilot.truth(
            guard["speed_used"]
        )
        need(enabled == (meta["arm"] == "mean_preview"), "Guard arm differs")
        need(pending == pilot.truth(side["paused"]), "Guard pending join differs")
        need(
            speed_used == pilot.truth(side["used"])
            and used == (enabled and pending and speed_used),
            "Guard eligibility differs",
        )
        for name in ("x", "y", "yaw", "v"):
            base, selected = pilot.finite(guard["base_" + name]), pilot.finite(
                guard["selected_" + name]
            )
            expected = (
                pilot.finite(side["mean_" + name])
                if used and name in ("x", "y")
                else base
            )
            need(selected == expected, f"Guard selected {name} differs")
            if name == "v":
                need(
                    base == pilot.finite(side["measured_speed"]), "Guard speed changed"
                )
            else:
                need(
                    terminal.admission.rounded_matches(
                        row["preview_pose_" + name], selected, 9
                    ),
                    "Guard actual pose rounding differs",
                )
                need(
                    terminal.admission.rounded_matches(
                        guard["base_" + name + "_9"], base, 9
                    ),
                    "Guard baseline rounding differs",
                )
        counts["guard_used_steps"] += used
        counts["changed_position_steps"] += any(
            pilot.finite(guard["base_" + n]) != pilot.finite(guard["selected_" + n])
            for n in ("x", "y")
        )
        if used:
            target = pilot.finite(side["nominal_target"])
            need(target >= 0, "Negative guard target")
            counts["positive_guard_uses"] += target > 0
            counts["zero_guard_uses"] += target == 0
    counts["native_geometry_queries"] = len(guard_sample_steps(guards))
    return counts


def audit_track(meta, track, sides, guards, outcome, obstacles, samples):
    counts = validate_guard(meta, track, sides, guards)
    original_estimator_check = pilot.check_motion_filter_track

    def check_original_estimator(actual, *args):
        # Only this estimator verifier receives baseline x/y. Physics and gates
        # retain the actual native rows passed to the enclosing audit function.
        need(actual is track, "Estimator audit received a different trajectory")
        baseline = [dict(row) for row in actual]
        for row, guard in zip(baseline[1:], guards):
            row["preview_pose_x"] = guard["base_x_9"]
            row["preview_pose_y"] = guard["base_y_9"]
        return original_estimator_check(baseline, *args)

    namespace = dict(
        pilot.audit_track.__globals__,
        check_motion_filter_track=check_original_estimator,
    )
    independent = types.FunctionType(pilot.audit_track.__code__, namespace)
    old_samples = []
    case, check = independent(
        dict(meta, arm="mean_probe"), track, sides, outcome, obstacles, old_samples
    )
    for row in old_samples:
        row["arm"] = meta["arm"]
    samples.extend(old_samples)
    for row in (case, check):
        row["arm"] = meta["arm"]
        row["speed_used_steps"] = row.pop("mean_used_steps")
        row["case_id"] = case_id(meta)
        row.update(counts)
    return case, check


def case_id(meta):
    return f"tau{float(meta['tau']):g}/{meta['arm']}/{meta['mode']}/{meta['seed']}/{meta['scenario']}/{meta['planner']}"


def history_row(sequence, ident, meta, step, row, side, guard):
    result = terminal.history_row(
        sequence,
        ident,
        dict(meta, case_id=case_id(meta)),
        step,
        row,
        side,
        ("0.1", "0.02", "false"),
    )
    for axis in ("x", "y", "yaw"):
        result["preview_" + axis + "_9"] = guard["base_" + axis + "_9"]
    result.update(
        dict(
            zip(
                native.EXTRA_FIELDS,
                (
                    guard["enabled"],
                    guard["used"],
                    guard["joint_pending"],
                    guard["speed_used"],
                    guard["base_x"],
                    guard["base_y"],
                    guard["base_yaw"],
                    guard["selected_x"],
                    guard["selected_y"],
                    guard["selected_yaw"],
                    row["preview_pose_x"],
                    row["preview_pose_y"],
                    row["preview_pose_yaw"],
                    row["stop_preview_clearance_lower_bound"],
                    row["terminal_recovery_clearance"],
                    row["release_preview_clearance_lower_bound"],
                    row["guard_release_margin"],
                    side["count"],
                    side["paused"],
                ),
            )
        )
    )
    need(tuple(result) == native.HISTORY_FIELDS, "Native history schema differs")
    return result


def pair(
    meta,
    left,
    right,
    left_sides,
    right_sides,
    left_guards,
    right_guards,
    left_case,
    right_case,
):
    old_cases = [
        dict(c, mean_used_steps=c["speed_used_steps"]) for c in (left_case, right_case)
    ]
    result = pilot.pair(meta, left, right, left_sides, right_sides, *old_cases)
    changed = result["first_difference_step"]
    if changed != "":
        need(
            pilot.truth(right_guards[changed - 1]["used"]),
            "First changed row has no guard-preview use",
        )
    if meta["mode"] == "none":
        need(
            left_sides == right_sides
            and not left_case["guard_used_steps"]
            and not right_case["guard_used_steps"],
            "No-loss speed sidecar/guard control differs",
        )
    result["candidate_guard_used_steps"] = right_case["guard_used_steps"]
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
        receipt["native_tests"] == provenance["native_tests"] == 27,
        "Native test coverage differs",
    )
    for filename, key in (
        ("mean-guard-preview.rs", "generated_source_sha256"),
        ("candidate.diff", "diff_sha256"),
        ("mean-guard-preview", "candidate_binary_sha256"),
        ("native-tests.log", "native_tests_sha256"),
        ("legacy/build-sources.json", "legacy_receipt_sha256"),
        ("legacy/mean-speed-probe", "legacy_binary_sha256"),
        ("original/pose_observation_loss", "original_binary_sha256"),
    ):
        need(
            build.sha(raw / "build" / filename) == receipt[key],
            f"Build artifact changed: {filename}",
        )
    need(
        (raw / "build/mean-guard-preview.rs").read_text()
        == build.generate(build.BASE.read_text()),
        "Generated live source differs",
    )
    old_receipt = json.loads((raw / "build/legacy/build-sources.json").read_text())
    need(
        old_receipt["libraries"] == receipt["libraries"]
        and (raw / "build/legacy/mean-speed-probe.rs").read_text()
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
            trial["arm"] == "mean_preview",
        )
        need(
            len(trial["command"]) == 31
            and trial["command"][3:] == args[2:]
            and Path(trial["command"][0]).name == "mean-guard-preview",
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
        expected_files = set(runner.FILES) - (
            {"guard-preview-inputs.csv"}
            if kind == "legacy"
            else (
                {"speed-probes.csv", "guard-preview-inputs.csv"}
                if kind == "original"
                else set()
            )
        )
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
        reference = raw / f"tau{tau:g}/fused_preview-{mode}-{pins['seed']}"
        for name in compared:
            need(
                build.sha(reference / name) == record["sha256"][name],
                "Regression byte comparison differs",
            )
        args = (
            build.arguments(build.FIXTURES, directory, mode, pins["seed"], tau)
            if kind == "omitted"
            else build.legacy.arguments(
                build.FIXTURES,
                directory,
                mode,
                pins["seed"],
                tau,
                True if kind == "legacy" else None,
            )
        )
        binary = {
            "omitted": "mean-guard-preview",
            "legacy": "mean-speed-probe",
            "original": "pose_observation_loss",
        }[kind]
        need(
            record["command"][3:] == args[2:]
            and len(record["command"]) == len(args) + 1
            and Path(record["command"][0]).name == binary,
            "Regression command differs",
        )
    return provenance


def audit(raw, output):
    pins = build.preflight()
    raw, output = Path(raw).resolve(), build.fresh(output)
    provenance = check_provenance(raw, pins)
    output.mkdir(parents=True)
    cases, pairs, checks, samples = [], [], [], []
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
                                    outcome,
                                    obstacles[key[0]],
                                    samples,
                                )
                                cases.append(case)
                                checks.append(check)
                                local_cases.append(case)
                                sample_steps = guard_sample_steps(data[i + 4])
                                for step, (row, side, guard) in enumerate(
                                    zip(data[i][1:], data[i + 2], data[i + 4]), 1
                                ):
                                    sequence += 1
                                    ident = 0
                                    if step in sample_steps:
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
            cell["candidate_guard_used_steps"] = sum(
                p["candidate_guard_used_steps"] for p in chosen
            )
        tables = (
            ("cases.csv", cases, CASE_FIELDS),
            ("verification.csv", checks, VERIFY_FIELDS),
            (
                "pairs.csv",
                [renamed(p) for p in pairs],
                (*map(renamed_key, pilot.PAIR_FIELDS), "candidate_guard_used_steps"),
            ),
            (
                "summary.csv",
                [renamed(c) for c in summary],
                (*map(renamed_key, pilot.SUMMARY_FIELDS), "candidate_guard_used_steps"),
            ),
            ("probe-samples.csv", samples, pilot.SAMPLE_FIELDS),
            ("history-checks.csv", histories, native.CHECK_FIELDS),
        )
        for name, rows, fields in tables:
            pilot.write(output / name, rows, fields)
        sources = dict(
            method="complete native dynamics/contact/hold and original speed-window audit; full independent baseline-estimator histories and fixed native guard-query sampling",
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
        parser.exit(1, f"Mean guard-preview audit failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
