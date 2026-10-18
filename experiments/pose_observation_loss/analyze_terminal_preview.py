#!/usr/bin/env python3
"""Replay all frozen mean-arm preview histories and four static terminal queries."""

import argparse
from collections import Counter, defaultdict
import csv
import itertools
import json
import math
from pathlib import Path
import shutil
import subprocess

import analyze_mean_probe_admission as admission
import analyze_mean_speed_probe as pilot
import analyze_probe_geometry as previous
import build_mean_speed_probe as build
import native_terminal_preview as native

STUDY = build.ROOT / "results/terminal-preview-input-replay"
META = admission.META
MASKS = previous.MASKS
RECORD_FIELDS = (
    "id",
    *META,
    "step",
    "time_s",
    "logged_recovery",
    "logged_stop",
    "logged_terminal",
    "logged_release",
    "logged_margin",
    "logged_release_margin",
    "mean_x",
    "mean_y",
    "mean_yaw",
)
DETAIL_FIELDS = (
    "stop_pass",
    "terminal_pass",
    "release_pass",
    "recorded_release_pass",
    "parent_release_pass",
    "parent_variant",
    "shift_vs_recorded",
    "yaw_vs_recorded",
    "stop_gap",
    "terminal_gap",
    "release_gap",
)
ROW_FIELDS = (
    "id",
    *META,
    "step",
    "time_s",
    "original_mask",
    *(f for f in native.TRACE_FIELDS if f not in ("id", "scenario")),
    *DETAIL_FIELDS,
)
REP_FIELDS = ("id", *META, "step", "time_s", "original_mask", "roles", "previous_plot")
COUNT_FIELDS = (
    "selected_decisions",
    "release_pass",
    "release_fail",
    "stop_fail",
    "terminal_fail",
    "both_fail",
    "from_recorded_fail_to_pass",
    "from_recorded_pass_to_fail",
    "from_parent_fail_to_pass",
    "from_parent_pass_to_fail",
)
VALUE_FIELDS = (
    "shift_vs_recorded",
    "abs_yaw_vs_recorded_deg",
    "motion_length",
    "terminal_gap",
    "release_gap",
)
STAT_FIELDS = tuple(
    prefix + "_" + field for field in VALUE_FIELDS for prefix in ("min", "max", "mean")
)
CASE_FIELDS = (*META, "variant", *COUNT_FIELDS, *STAT_FIELDS)
SUMMARY_FIELDS = (
    "noise_selector",
    "mode_selector",
    "original_mask",
    "variant",
    "cases",
    "selected_cases",
    *COUNT_FIELDS,
    *STAT_FIELDS,
)
need = build.require


def preflight():
    pins = json.loads((STUDY / "freeze-pins.json").read_text())
    need(
        build.sha(STUDY / "protocol.md") == pins["protocol_sha256"],
        "Terminal protocol changed",
    )
    for name, digest in pins["input_sha256"].items():
        need(
            build.sha(build.REPO / name) == digest,
            f"Changed terminal replay input: {name}",
        )
    previous.preflight()
    return pins


def write_packet(path, rows, fields):
    with Path(path).open("x", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=fields, delimiter="\t", lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)


def configuration(trial):
    command = trial["command"]
    need(
        len(command) == 30 and trial["arm"] == "mean_probe" and command[29] == "true",
        "Different mean-arm command",
    )
    expected = {
        17: "true",
        18: "false",
        19: "true",
        21: "false",
        22: "false",
        23: "true",
        24: "true",
        26: "true",
        27: "true",
        28: "true",
    }
    need(
        all(command[i] == value for i, value in expected.items()),
        "Unsupported preview configuration",
    )
    return command[4], command[7], command[21]


def history_row(sequence, ident, meta, step, row, side, config):
    need(int(side["step"]) == step, "Terminal history step order differs")
    pilot.close(side["time_s"], row["time_s"])
    need(
        side["correction"] == row["correction"]
        and side["received"] == row["correction_received"]
        and side["precision_activation"] == row["precision_activation"],
        "Sensor event join differs",
    )
    need(
        pilot.finite(side["measured_speed"]) == pilot.finite(row["measured_speed"]),
        "Round-trip speed join differs",
    )
    need(
        admission.rounded_matches(
            row["nominal_target_speed"], side["nominal_target"], 9
        )
        and admission.rounded_matches(row["nominal_steering"], side["steering"], 9),
        "Nominal command join differs",
    )
    return dict(
        zip(
            native.HISTORY_FIELDS,
            (
                sequence,
                ident,
                meta["case_id"],
                meta["scenario"],
                step,
                side["time_s"],
                side["raw_x"],
                side["raw_y"],
                side["raw_yaw"],
                side["measured_speed"],
                side["observation_age"],
                side["correction"],
                side["received"],
                side["precision_activation"],
                side["applied_steering"],
                side["nominal_target"],
                side["steering"],
                side["margin"],
                side["mean_x"],
                side["mean_y"],
                side["mean_yaw"],
                row["precision_active"],
                row["motion_filter_sample_count"],
                row["epoch_fusion_available"],
                row["epoch_fusion_group_count"],
                row["motion_filter_pose_x"],
                row["motion_filter_pose_y"],
                row["preview_pose_x"],
                row["preview_pose_y"],
                row["preview_pose_yaw"],
                *config,
            ),
        )
    )


def extract(raw, provenance, work, old_cases):
    index = {admission.identity(r): r for r in old_cases if r["arm"] == "mean_probe"}
    cases, records, sequence = [], [], 0
    packet = work / "history.tsv"
    with packet.open("x", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=native.HISTORY_FIELDS,
            delimiter="\t",
            lineterminator="\n",
        )
        writer.writeheader()
        for trial in provenance["trials"]:
            if trial["arm"] != "mean_probe":
                continue
            config = configuration(trial)
            directory = raw / trial["directory"]
            with (directory / "trajectories.csv").open(newline="") as tracks, (
                directory / "speed-probes.csv"
            ).open(newline="") as sides:
                seen = set()
                for groups in itertools.zip_longest(
                    pilot.grouped(tracks), pilot.grouped(sides)
                ):
                    need(
                        all(g is not None for g in groups)
                        and groups[0][0] == groups[1][0],
                        "Terminal case grouping differs",
                    )
                    key, track, side_rows = groups[0][0], groups[0][1], groups[1][1]
                    need(key not in seen, "Repeated terminal history case")
                    seen.add(key)
                    lookup = dict(
                        tau=trial["tau"],
                        arm=trial["arm"],
                        mode=trial["mode"],
                        seed=trial["seed"],
                        scenario=key[0],
                        planner=key[1],
                    )
                    old = index[admission.identity(lookup)]
                    meta = {name: old[name] for name in META}
                    cases.append(meta)
                    need(
                        len(track) == len(side_rows) + 1 == int(old["decisions"]) + 1,
                        "Terminal track length differs",
                    )
                    for step, (row, side) in enumerate(zip(track[1:], side_rows), 1):
                        sequence += 1
                        selected = (
                            pilot.truth(side["used"])
                            and pilot.finite(side["nominal_target"]) > 0
                        )
                        ident = str(len(records) + 1) if selected else "0"
                        writer.writerow(
                            history_row(sequence, ident, meta, step, row, side, config)
                        )
                        if selected:
                            need(
                                pilot.truth(side["paused"])
                                and abs(pilot.finite(side["measured_speed"])) <= 0.02
                                and pilot.finite(side["observation_age"]) <= 1.5
                                and int(side["count"]) >= 10,
                                "Selected terminal decision is ineligible",
                            )
                            records.append(
                                dict(
                                    id=ident,
                                    **meta,
                                    step=side["step"],
                                    time_s=side["time_s"],
                                    logged_recovery=row["recovery_clearance"],
                                    logged_stop=row[
                                        "stop_preview_clearance_lower_bound"
                                    ],
                                    logged_terminal=row["terminal_recovery_clearance"],
                                    logged_release=row[
                                        "release_preview_clearance_lower_bound"
                                    ],
                                    logged_margin=side["margin"],
                                    logged_release_margin=row["guard_release_margin"],
                                    mean_x=side["mean_x"],
                                    mean_y=side["mean_y"],
                                    mean_yaw=side["mean_yaw"],
                                )
                            )
            need(len(seen) == 9, "Incomplete terminal history trial")
            print(f"Prepared complete sensor history: {trial['directory']}", flush=True)
    need(
        len(cases) == len(index)
        and {r["case_id"] for r in cases} == {r["case_id"] for r in index.values()},
        "Terminal history case set differs",
    )
    pilot.write(work / "metadata.csv", records, RECORD_FIELDS)
    return packet, records, cases, sequence


def classify(trace):
    stop, terminal, release, margin = (
        pilot.finite(trace[name])
        for name in (
            "stop_clearance",
            "terminal_clearance",
            "release_clearance",
            "release_margin",
        )
    )
    adjusted = terminal - 0.05
    need(release == min(stop, adjusted), "Native aligned release differs")
    return dict(
        stop_pass=stop > margin,
        terminal_pass=adjusted > margin,
        release_pass=release > margin,
        stop_gap=stop - margin,
        terminal_gap=adjusted - margin,
        release_gap=release - margin,
    )


def diagnose(record, block):
    need(
        len(block) == len(native.VARIANTS)
        and [t["variant"] for t in block] == list(native.VARIANTS),
        "Missing/reordered terminal variant",
    )
    need(
        all(
            t["id"] == record["id"] and t["scenario"] == record["scenario"]
            for t in block
        ),
        "Terminal identity join differs",
    )
    actual = block[0]
    for field, logged in (
        ("stop_9", "logged_stop"),
        ("terminal_9", "logged_terminal"),
        ("release_9", "logged_release"),
        ("release_margin_9", "logged_release_margin"),
    ):
        need(
            actual[field] == record[logged],
            f"Recorded terminal query differs: {field} at {record['id']}",
        )
    states = [classify(t) for t in block]
    previous_release = admission.release_gate(
        record["logged_release"], pilot.finite(actual["release_margin"])
    )
    need(
        previous_release is not None and previous_release == states[0]["release_pass"],
        "Previously definite release differs",
    )
    recovery = admission.release_gate(
        record["logged_recovery"], pilot.finite(record["logged_margin"])
    )
    need(recovery is not None, "Frozen recovery interval is unresolved")
    mask = (
        ";".join(
            name
            for name, ok in (("clearance", recovery), ("release", previous_release))
            if not ok
        )
        or "ready"
    )
    result = []
    for i, (trace, state) in enumerate(zip(block, states)):
        for field in ("steering", "speed", "preview_margin", "release_margin"):
            need(
                trace[field] == actual[field],
                f"Fixed terminal query input changed: {field}",
            )
        need(
            pilot.finite(trace["target"])
            == (0 if i == 3 else pilot.finite(actual["target"]))
            and pilot.finite(actual["target"]) > 0,
            "Terminal target variant differs",
        )
        if i in (0, 3):
            need(
                all(trace[f] == actual[f] for f in ("x", "y", "yaw")),
                "Zero-target pose changed",
            )
        else:
            need(
                all(
                    pilot.finite(trace[f]) == pilot.finite(record["mean_" + f])
                    for f in ("x", "y")
                ),
                "Mean-position variant differs",
            )
            need(
                pilot.finite(trace["yaw"])
                == pilot.finite(actual["yaw"] if i == 1 else record["mean_yaw"]),
                "Mean-heading variant differs",
            )
        parent = 1 if i == 2 else 0
        delta = pilot.finite(trace["yaw"]) - pilot.finite(actual["yaw"])
        detail = dict(
            state,
            recorded_release_pass=states[0]["release_pass"],
            parent_release_pass=states[parent]["release_pass"],
            parent_variant=block[parent]["variant"],
            shift_vs_recorded=math.hypot(
                pilot.finite(trace["x"]) - pilot.finite(actual["x"]),
                pilot.finite(trace["y"]) - pilot.finite(actual["y"]),
            ),
            yaw_vs_recorded=math.atan2(math.sin(delta), math.cos(delta)),
        )
        result.append(
            dict(
                id=record["id"],
                **{name: record[name] for name in META},
                step=record["step"],
                time_s=record["time_s"],
                original_mask=mask,
                **{
                    key: value
                    for key, value in trace.items()
                    if key not in ("id", "scenario")
                },
                **{
                    name: str(value).lower() if type(value) is bool else value
                    for name, value in detail.items()
                },
            )
        )
    return result


def stats(rows):
    counts = Counter({name: 0 for name in COUNT_FIELDS})
    values = defaultdict(list)
    for row in rows:
        passed, stop, terminal = (
            pilot.truth(row[k]) for k in ("release_pass", "stop_pass", "terminal_pass")
        )
        counts["selected_decisions"] += 1
        counts["release_pass" if passed else "release_fail"] += 1
        counts["stop_fail"] += not stop
        counts["terminal_fail"] += not terminal
        counts["both_fail"] += not stop and not terminal
        for ref in ("recorded", "parent"):
            before = pilot.truth(row[ref + "_release_pass"])
            counts["from_" + ref + "_fail_to_pass"] += not before and passed
            counts["from_" + ref + "_pass_to_fail"] += before and not passed
        for name in VALUE_FIELDS:
            values[name].append(
                abs(math.degrees(pilot.finite(row["yaw_vs_recorded"])))
                if name == "abs_yaw_vs_recorded_deg"
                else pilot.finite(row[name])
            )
    result = dict(counts)
    for name in VALUE_FIELDS:
        selected = values[name]
        for prefix, value in zip(
            ("min", "max", "mean"),
            (
                (min(selected), max(selected), math.fsum(selected) / len(selected))
                if selected
                else ("", "", "")
            ),
        ):
            result[prefix + "_" + name] = value
    return result


def summarize(cases, rows):
    by_case = defaultdict(list)
    for row in rows:
        by_case[row["case_id"], row["variant"]].append(row)
    case_rows = [
        dict(meta, variant=variant, **stats(by_case[meta["case_id"], variant]))
        for meta in cases
        for variant in native.VARIANTS
    ]
    summary = []
    for noise in ("all", "white", "correlated"):
        for mode in ("all", "none", "random", "blackout"):
            selected_cases = [
                c
                for c in cases
                if (noise == "all" or (float(c["tau"]) == 0) == (noise == "white"))
                and (mode == "all" or c["mode"] == mode)
            ]
            ids = {c["case_id"] for c in selected_cases}
            for mask in MASKS:
                for variant in native.VARIANTS:
                    selected = [
                        r
                        for r in rows
                        if r["case_id"] in ids
                        and r["original_mask"] == mask
                        and r["variant"] == variant
                    ]
                    summary.append(
                        dict(
                            noise_selector=noise,
                            mode_selector=mode,
                            original_mask=mask,
                            variant=variant,
                            cases=len(ids),
                            selected_cases=len({r["case_id"] for r in selected}),
                            **stats(selected),
                        )
                    )
    return case_rows, summary


def check_history(checks, cases, records, old_cases):
    need(
        [r["case_id"] for r in checks] == [r["case_id"] for r in cases],
        "Native history case order differs",
    )
    selected = Counter(r["case_id"] for r in records)
    old = {r["case_id"]: r for r in old_cases if r["arm"] == "mean_probe"}
    for check, meta in zip(checks, cases):
        need(
            check["scenario"] == meta["scenario"]
            and int(check["decisions"]) == int(old[meta["case_id"]]["decisions"])
            and int(check["selected_decisions"]) == selected[meta["case_id"]],
            "Native history case coverage differs",
        )


def check_baseline(cases, rows, old_cases):
    actual = Counter(
        (r["case_id"], r["original_mask"]) for r in rows if r["variant"] == "recorded"
    )
    old = {r["case_id"]: r for r in old_cases}
    need(set(old) == {r["case_id"] for r in cases}, "Previous geometry cases differ")
    for meta in cases:
        case = old[meta["case_id"]]
        for mask in MASKS:
            need(
                actual[meta["case_id"], mask] == int(case[mask.replace(";", "_and_")]),
                "Previous per-case terminal mask differs",
            )


def representatives(records, rows, old):
    index = {r["id"]: r for r in records}
    masks = {r["id"]: r["original_mask"] for r in rows if r["variant"] == "recorded"}
    need(len(old) == len({r["id"] for r in old}), "Duplicate previous representative")
    result = []
    for previous_row in old:
        row = index[previous_row["id"]]
        need(
            all(row[k] == previous_row[k] for k in ("id", *META, "step", "time_s"))
            and masks[row["id"]] == previous_row["mask"],
            "Previous representative identity/role population differs",
        )
        result.append(
            dict(
                id=row["id"],
                **{k: row[k] for k in META},
                step=row["step"],
                time_s=row["time_s"],
                original_mask=masks[row["id"]],
                roles=previous_row["roles"],
                previous_plot=previous_row["plot"],
            )
        )
    return result


def collect(raw, output):
    pins = preflight()
    raw, output = Path(raw).resolve(), build.fresh(output)
    need(
        build.sha(raw / "provenance.json") == pins["raw_provenance_sha256"],
        "Different terminal pilot packet",
    )
    provenance = pilot.check_provenance(raw, build.preflight())
    need(
        not subprocess.check_output(
            ["git", "status", "--porcelain=v1"], cwd=build.REPO, text=True
        ).strip(),
        "Commit terminal replay before extracting new preview results",
    )
    implementation = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=build.REPO, text=True
    ).strip()
    output.mkdir(parents=True)
    try:
        work = output / "native"
        compiler = native.compile_native(work)
        old_cases = pilot.read(admission.STUDY / "case-gates.csv")
        packet, records, cases, history_count = extract(
            raw, provenance, work, old_cases
        )
        need(
            len(cases) == pins["expected_cases"]
            and history_count == pins["expected_history_decisions"]
            and len(records) == pins["expected_selected_decisions"],
            "Terminal replay population differs",
        )
        evaluation = native.evaluate(work, packet)
        checks = pilot.read(work / "history-checks.csv", native.CHECK_FIELDS)
        check_history(checks, cases, records, old_cases)
        trace = pilot.read(work / "trace.csv", native.TRACE_FIELDS)
        need(
            len(trace) == pins["expected_native_queries"],
            "Terminal query count differs",
        )
        rows = [
            r
            for i, record in enumerate(records)
            for r in diagnose(record, trace[i * 4 : i * 4 + 4])
        ]
        old_geometry = pilot.read(previous.STUDY / "case-summary.csv")
        check_baseline(cases, rows, old_geometry)
        actual = [r for r in rows if r["variant"] == "recorded"]
        need(
            sum(r["original_mask"] == "ready" for r in actual) == pins["expected_ready"]
            and sum(not pilot.truth(r["release_pass"]) for r in actual)
            == pins["expected_release_failures"],
            "Frozen terminal readiness/release counts differ",
        )
        pilot.write(work / "decisions.csv", rows, ROW_FIELDS)
        reps = representatives(
            records, rows, pilot.read(previous.STUDY / "representatives.csv")
        )
        need(
            len(reps) == pins["expected_representatives"],
            "Compact terminal population differs",
        )
        ids = {r["id"] for r in reps}
        compact_trace = [t for t in trace if t["id"] in ids]
        compact_rows = [r for r in rows if r["id"] in ids]
        case_rows, summary = summarize(cases, rows)
        tables = (
            ("case-summary.csv", case_rows, CASE_FIELDS),
            ("summary.csv", summary, SUMMARY_FIELDS),
            ("representatives.csv", reps, REP_FIELDS),
            ("representative-native.csv", compact_trace, native.TRACE_FIELDS),
            ("representative-comparisons.csv", compact_rows, ROW_FIELDS),
            ("history-checks.csv", checks, native.CHECK_FIELDS),
        )
        for name, data, fields in tables:
            pilot.write(output / name, data, fields)
        write_packet(
            output / "representative-inputs.tsv",
            [{k: t[k] for k in native.DIRECT_FIELDS} for t in compact_trace],
            native.DIRECT_FIELDS,
        )
        shutil.copyfile(work / "terminal-preview.rs", output / "terminal-preview.rs")
        receipt = dict(
            compiler=compiler,
            evaluation=evaluation,
            history_decisions=history_count,
            selected_decisions=len(records),
            static_queries=len(trace),
            variants=list(native.VARIANTS),
            extra_sha256={
                name: build.sha(work / name)
                for name in ("metadata.csv", "decisions.csv")
            },
        )
        (output / "native-sources.json").write_text(
            json.dumps(receipt, indent=2) + "\n"
        )
        names = [name for name, _, _ in tables] + [
            "representative-inputs.tsv",
            "terminal-preview.rs",
            "native-sources.json",
        ]
        sources = dict(
            method="original native sensor-filter histories and four fixed static terminal queries on the frozen pilot",
            implementation_commit=implementation,
            working_tree_clean=True,
            protocol_sha256=build.sha(STUDY / "protocol.md"),
            freeze_sha256=build.sha(STUDY / "freeze-pins.json"),
            raw_provenance_sha256=pins["raw_provenance_sha256"],
            input_sha256=pins["input_sha256"],
            script_sha256={
                name: build.sha(build.ROOT / name)
                for name in (
                    "analyze_terminal_preview.py",
                    "native_terminal_preview.py",
                    "test_terminal_preview.py",
                )
            },
            audit=dict(
                cases=len(cases),
                history_decisions=history_count,
                selected_decisions=len(records),
                static_queries=len(trace),
                representatives=len(reps),
                compact_queries=len(compact_trace),
                ready=pins["expected_ready"],
                release_failures=pins["expected_release_failures"],
                fresh_simulator_steps=0,
            ),
            output_sha256={name: build.sha(output / name) for name in names},
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
        f"Verified {history_count} history decisions, {len(records)} selected decisions and {len(trace)} static queries",
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
        collect(args.raw, args.output)
    except (
        OSError,
        ValueError,
        KeyError,
        AssertionError,
        subprocess.CalledProcessError,
    ) as error:
        parser.exit(1, f"Terminal preview replay failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
