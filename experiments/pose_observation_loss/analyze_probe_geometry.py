#!/usr/bin/env python3
"""Explain fixed mean-probe decisions with native geometric witnesses."""

import argparse
from collections import Counter, defaultdict
import csv
from decimal import Decimal, localcontext
import itertools
import json
import math
from pathlib import Path
import shutil
import subprocess

import analyze_mean_probe_admission as admission
import analyze_mean_speed_probe as pilot
import build_mean_speed_probe as build
import native_probe_geometry as native

STUDY = build.ROOT / "results/mean-speed-probe-geometry-witnesses"
MASKS = ("ready", "clearance", "clearance;release", "release")
META = admission.META
RECORD_FIELDS = (
    "id",
    *META,
    "step",
    "time_s",
    "logged_distance",
    "logged_recovery",
    "logged_release_margin",
    "logged_release",
    "logged_stop",
    "logged_terminal",
    "guard_x",
    "guard_y",
    "guard_yaw",
)
DETAIL_FIELDS = (
    "mask",
    "recovery_gap",
    "stop_gate",
    "terminal_gate",
    "release_gate",
    "release_component_failures",
    "release_component_unknowns",
    "limiting_component",
    "actual_vs_mean",
    "mean_vs_straight",
    "mean_zero",
    "straight_zero",
    "witnesses",
    "interior_witnesses",
    "endpoint_witnesses",
    "tied_witnesses",
)
COUNT_FIELDS = (
    "selected_decisions",
    "ready",
    "clearance",
    "clearance_and_release",
    "release",
    "stop_pass",
    "stop_fail",
    "stop_unresolved",
    "terminal_pass",
    "terminal_fail",
    "terminal_unresolved",
    "combined_unresolved",
    "stop_limited",
    "terminal_limited",
    "limiter_unresolved",
    "both_components_fail",
    "actual_gt_mean",
    "actual_eq_mean",
    "actual_lt_mean",
    "mean_gt_straight",
    "mean_eq_straight",
    "mean_lt_straight",
    "mean_zero",
    "straight_zero",
    "witnesses",
    "interior_witnesses",
    "endpoint_witnesses",
    "tied_witness_decisions",
)
CASE_FIELDS = (*META, *COUNT_FIELDS, "distinct_witness_obstacles")
SUMMARY_FIELDS = (
    "noise_selector",
    "mode_selector",
    "mask",
    "cases",
    "selected_cases",
    *COUNT_FIELDS,
    "distinct_witness_obstacles",
    "min_recovery_gap",
    "max_recovery_gap",
    "max_abs_heading_delta_deg",
    "min_actual_distance",
    "max_actual_distance",
    "min_mean_distance",
    "max_mean_distance",
    "min_straight_distance",
    "max_straight_distance",
)
REP_FIELDS = (*RECORD_FIELDS, *DETAIL_FIELDS, "roles", "plot")
need = build.require


def preflight():
    pins = json.loads((STUDY / "freeze-pins.json").read_text())
    need(
        build.sha(STUDY / "protocol.md") == pins["protocol_sha256"],
        "Geometry protocol changed",
    )
    for name, digest in pins["input_sha256"].items():
        need(build.sha(build.REPO / name) == digest, f"Changed geometry input: {name}")
    admission.preflight()
    return pins


def rounding_box(text):
    """Outward binary enclosure of the closed nine-decimal rounding box."""
    pilot.finite(text)
    with localcontext() as context:
        context.prec = 80
        center, half = Decimal(text), Decimal("0.0000000005")
        lower = math.nextafter(float(center - half), -math.inf)
        upper = math.nextafter(float(center + half), math.inf)
    need(math.isfinite(lower) and math.isfinite(upper), "Nonfinite rounding enclosure")
    return lower, upper


def interval_gate(lower, upper, margin):
    lower, upper, margin = map(pilot.finite, (lower, upper, margin))
    need(lower <= upper, "Reversed clearance enclosure")
    return "pass" if lower > margin else "fail" if upper <= margin else "unresolved"


def compare(a, b):
    a, b = pilot.finite(a), pilot.finite(b)
    return "gt" if a > b else "lt" if a < b else "eq"


def write_queries(path, rows):
    with Path(path).open("x", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=native.INPUT_FIELDS, delimiter="\t", lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)


def read_queries(path):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream, delimiter="\t")
        need(
            tuple(reader.fieldnames) == native.INPUT_FIELDS,
            "Native input schema differs",
        )
        rows = list(reader)
    need(
        all(None not in r and None not in r.values() for r in rows),
        "Malformed native inputs",
    )
    return rows


def extract(raw, provenance, old_cases):
    index = {admission.identity(r): r for r in old_cases if r["arm"] == "mean_probe"}
    queries, records, cases = [], [], []
    for trial in provenance["trials"]:
        if trial["arm"] != "mean_probe":
            continue
        directory = raw / trial["directory"]
        with (directory / "trajectories.csv").open(newline="") as tracks, (
            directory / "speed-probes.csv"
        ).open(newline="") as sides:
            seen = set()
            for groups in itertools.zip_longest(
                pilot.grouped(tracks), pilot.grouped(sides)
            ):
                need(
                    all(g is not None for g in groups) and groups[0][0] == groups[1][0],
                    "Geometry case grouping differs",
                )
                key, track, side_rows = groups[0][0], groups[0][1], groups[1][1]
                need(key not in seen, "Duplicate geometry case")
                seen.add(key)
                meta = dict(
                    tau=trial["tau"],
                    arm=trial["arm"],
                    mode=trial["mode"],
                    seed=trial["seed"],
                    scenario=key[0],
                    planner=key[1],
                )
                old = index[admission.identity(meta)]
                meta = {name: old[name] for name in META}
                cases.append(meta)
                need(
                    len(track) == len(side_rows) + 1 == int(old["decisions"]) + 1,
                    "Geometry track length differs",
                )
                for step, (row, side) in enumerate(zip(track[1:], side_rows), 1):
                    need(int(side["step"]) == step, "Geometry step order differs")
                    pilot.close(row["time_s"], side["time_s"])
                    if (
                        not pilot.truth(side["used"])
                        or pilot.finite(side["nominal_target"]) <= 0
                    ):
                        continue
                    need(
                        pilot.truth(side["paused"])
                        and pilot.finite(side["observation_age"]) <= 1.5
                        and abs(pilot.finite(side["measured_speed"])) <= 0.02
                        and int(side["count"]) >= 10,
                        "Selected decision is ineligible",
                    )
                    need(
                        side["probe_x"] == side["mean_x"]
                        and side["probe_y"] == side["mean_y"]
                        and side["probe_yaw"] == side["raw_yaw"],
                        "Recorded probe input differs",
                    )
                    stop = rounding_box(row["stop_preview_clearance_lower_bound"])
                    terminal = rounding_box(row["terminal_recovery_clearance"])
                    ident = str(len(records) + 1)
                    queries.append(
                        dict(
                            zip(
                                native.INPUT_FIELDS,
                                (
                                    ident,
                                    key[0],
                                    side["mean_x"],
                                    side["mean_y"],
                                    side["mean_yaw"],
                                    side["raw_yaw"],
                                    side["steering"],
                                    side["margin"],
                                    side["observation_age"],
                                    row["precision_active"],
                                    trial["command"][4],
                                    trial["command"][7],
                                    trial["command"][21],
                                    *(format(v, ".17e") for v in (*stop, *terminal)),
                                    "false",
                                ),
                            )
                        )
                    )
                    records.append(
                        dict(
                            id=ident,
                            **meta,
                            step=side["step"],
                            time_s=side["time_s"],
                            logged_distance=side["free_distance"],
                            logged_recovery=row["recovery_clearance"],
                            logged_release_margin=row["guard_release_margin"],
                            logged_release=row["release_preview_clearance_lower_bound"],
                            logged_stop=row["stop_preview_clearance_lower_bound"],
                            logged_terminal=row["terminal_recovery_clearance"],
                            guard_x=row["preview_pose_x"],
                            guard_y=row["preview_pose_y"],
                            guard_yaw=row["preview_pose_yaw"],
                        )
                    )
        need(len(seen) == 9, "Incomplete mean-probe trial")
        print(f"Selected fixed decisions: {trial['directory']}", flush=True)
    need(
        len(cases) == len(index)
        and {r["case_id"] for r in cases} == {r["case_id"] for r in index.values()},
        "Mean case coverage differs",
    )
    return queries, records, cases


def diagnose(record, query, trace, witnesses, obstacle_index):
    ident = record["id"]
    need(query["id"] == trace["id"] == ident, "Native geometry query order differs")
    need(
        pilot.finite(trace["actual_distance"])
        == pilot.finite(record["logged_distance"])
        > 0,
        "Recorded speed-probe distance differs",
    )
    need(
        trace["recovery_clearance_9"] == record["logged_recovery"],
        "Recorded recovery line differs",
    )
    need(
        trace["release_margin_9"] == record["logged_release_margin"],
        "Recorded release margin differs",
    )
    margin, release_margin = pilot.finite(query["margin"]), pilot.finite(
        trace["release_margin"]
    )
    need(
        pilot.finite(trace["margin"]) == margin
        and pilot.truth(trace["recovery_bounds"]),
        "Native margin/bounds differ",
    )
    need(
        pilot.finite(trace["start_clearance"]) > margin
        and pilot.finite(trace["start_wall_clearance"]) > margin,
        "Positive probe has an unsafe initial point",
    )
    ids = trace["nearest_obstacle_ids"].split(";")
    need(
        [w["obstacle_id"] for w in witnesses] == ids and len(set(ids)) == len(ids),
        "Nearest witness coverage differs",
    )
    clearance = pilot.finite(trace["recovery_clearance"])
    for w in witnesses:
        need(w["id"] == ident, "Witness query differs")
        need(
            (
                record["scenario"],
                pilot.finite(w["obstacle_x"]),
                pilot.finite(w["obstacle_y"]),
            )
            == obstacle_index[int(w["obstacle_id"])],
            "Witness fixture row differs",
        )
        need(
            pilot.finite(w["distance"]) == clearance and 0 <= pilot.finite(w["t"]) <= 1,
            "Witness distance/projection differs",
        )
    old_release = admission.release_gate(record["logged_release"], release_margin)
    need(old_release is not None, "Previously definite release became unresolved")
    component = {
        name: interval_gate(trace[lo], trace[hi], release_margin)
        for name, lo, hi in (
            ("stop", "stop_lo", "stop_hi"),
            ("terminal", "terminal_release_lo", "terminal_release_hi"),
            ("release", "release_lo", "release_hi"),
        )
    }
    need(
        component["release"] == "unresolved"
        or (component["release"] == "pass") == old_release,
        "Combined release classification differs",
    )
    logged_lo, logged_hi = rounding_box(record["logged_release"])
    need(
        max(logged_lo, pilot.finite(trace["release_lo"]))
        <= min(logged_hi, pilot.finite(trace["release_hi"])),
        "Combined release intervals do not intersect",
    )
    clearance_ok = clearance > margin
    failed = [
        name
        for name, ok in (("clearance", clearance_ok), ("release", old_release))
        if not ok
    ]
    mask = ";".join(failed) or "ready"
    limiting = (
        "stop"
        if pilot.finite(trace["stop_hi"]) < pilot.finite(trace["terminal_release_lo"])
        else (
            "terminal"
            if pilot.finite(trace["terminal_release_hi"])
            < pilot.finite(trace["stop_lo"])
            else "unresolved"
        )
    )
    interior = sum(0 < pilot.finite(w["t"]) < 1 for w in witnesses)
    return dict(
        record,
        mask=mask,
        recovery_gap=clearance - margin,
        **{name + "_gate": value for name, value in component.items()},
        release_component_failures=";".join(
            name for name in ("stop", "terminal") if component[name] == "fail"
        ),
        release_component_unknowns=";".join(
            name for name in ("stop", "terminal") if component[name] == "unresolved"
        ),
        limiting_component=limiting,
        actual_vs_mean=compare(trace["actual_distance"], trace["mean_yaw_distance"]),
        mean_vs_straight=compare(
            trace["mean_yaw_distance"], trace["straight_distance"]
        ),
        mean_zero=str(pilot.finite(trace["mean_yaw_distance"]) == 0).lower(),
        straight_zero=str(pilot.finite(trace["straight_distance"]) == 0).lower(),
        witnesses=len(witnesses),
        interior_witnesses=interior,
        endpoint_witnesses=len(witnesses) - interior,
        tied_witnesses=str(len(witnesses) > 1).lower(),
    )


def counts(rows):
    result = Counter({name: 0 for name in COUNT_FIELDS})
    for r in rows:
        result["selected_decisions"] += 1
        result[r["mask"].replace(";", "_and_")] += 1
        for name in ("stop", "terminal"):
            result[name + "_" + r[name + "_gate"]] += 1
        result["both_components_fail"] += (
            r["release_component_failures"] == "stop;terminal"
        )
        result["combined_unresolved"] += r["release_gate"] == "unresolved"
        limiter = r["limiting_component"]
        result[
            limiter + "_limited" if limiter != "unresolved" else "limiter_unresolved"
        ] += 1
        result["actual_" + r["actual_vs_mean"] + "_mean"] += 1
        result["mean_" + r["mean_vs_straight"] + "_straight"] += 1
        for name in ("mean_zero", "straight_zero"):
            result[name] += pilot.truth(r[name])
        for name in ("witnesses", "interior_witnesses", "endpoint_witnesses"):
            result[name] += int(r[name])
        result["tied_witness_decisions"] += pilot.truth(r["tied_witnesses"])
    return dict(result)


def representatives(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[row["case_id"], row["mask"]].append(row)
    roles = defaultdict(list)
    for group in groups.values():
        roles[group[0]["id"]].append("first")
        worst = min(group, key=lambda r: r["recovery_gap"])
        roles[worst["id"]].append("minimum_recovery_gap")
    plots = {}
    for row in rows:
        if row["mask"] != "ready":
            plots.setdefault(row["mask"], row["id"])
    return [
        dict(
            r,
            roles=";".join(roles[r["id"]]),
            plot=str(r["id"] in plots.values()).lower(),
        )
        for r in rows
        if r["id"] in roles
    ]


def summaries(cases, rows, traces, witnesses):
    by_case = defaultdict(list)
    for row in rows:
        by_case[row["case_id"]].append(row)

    def unique(selected):
        return len({w["obstacle_id"] for r in selected for w in witnesses[r["id"]]})

    case_rows = [
        dict(
            meta,
            **counts(by_case[meta["case_id"]]),
            distinct_witness_obstacles=unique(by_case[meta["case_id"]]),
        )
        for meta in cases
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
            case_ids = {c["case_id"] for c in selected_cases}
            for mask in MASKS:
                selected = [
                    r for r in rows if r["case_id"] in case_ids and r["mask"] == mask
                ]
                extrema = {}
                for name, values in (
                    ("recovery_gap", [r["recovery_gap"] for r in selected]),
                    *(
                        (name, [pilot.finite(traces[r["id"]][field]) for r in selected])
                        for name, field in (
                            ("actual_distance", "actual_distance"),
                            ("mean_distance", "mean_yaw_distance"),
                            ("straight_distance", "straight_distance"),
                        )
                    ),
                ):
                    extrema["min_" + name], extrema["max_" + name] = (
                        (min(values), max(values)) if values else ("", "")
                    )
                summary.append(
                    dict(
                        noise_selector=noise,
                        mode_selector=mode,
                        mask=mask,
                        cases=len(case_ids),
                        selected_cases=len({r["case_id"] for r in selected}),
                        **counts(selected),
                        distinct_witness_obstacles=unique(selected),
                        **extrema,
                        max_abs_heading_delta_deg=max(
                            (
                                abs(
                                    math.degrees(
                                        pilot.finite(traces[r["id"]]["heading_delta"])
                                    )
                                )
                                for r in selected
                            ),
                            default="",
                        ),
                    )
                )
    return case_rows, summary


def check_previous(cases, rows, old_cases, patterns):
    actual = Counter((r["case_id"], r["mask"]) for r in rows)
    expected = Counter()
    for p in patterns:
        if (
            p["arm"] == "mean_probe"
            and p["phase"] == "mean_eligible"
            and "nominal" not in p["failed_gates"].split(";")
        ):
            need(p["unresolved_gates"] == "", "Previous eligible gate is unresolved")
            mask = p["failed_gates"] or "ready"
            need(mask in MASKS, "Unexpected previous positive mask")
            expected[p["case_id"], mask] += int(p["steps"])
    need(actual == expected, "Previous per-case gate masks differ")
    totals = Counter(r["case_id"] for r in rows)
    old_index = {r["case_id"]: r for r in old_cases if r["arm"] == "mean_probe"}
    need(
        {r["case_id"] for r in cases} == set(old_index),
        "Previous mean case set differs",
    )
    for meta in cases:
        need(
            totals[meta["case_id"]]
            == int(old_index[meta["case_id"]]["eligible_nominal_positive"]),
            "Previous case selected count differs",
        )


def collect(raw, output):
    pins = preflight()
    raw, output = Path(raw).resolve(), build.fresh(output)
    need(
        build.sha(raw / "provenance.json") == pins["raw_provenance_sha256"],
        "Different pilot packet",
    )
    provenance = pilot.check_provenance(raw, build.preflight())
    need(
        not subprocess.check_output(
            ["git", "status", "--porcelain=v1"], cwd=build.REPO, text=True
        ).strip(),
        "Commit the geometry diagnostic before extracting witnesses",
    )
    implementation = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=build.REPO, text=True
    ).strip()
    output.mkdir(parents=True)
    try:
        old_cases = pilot.read(admission.STUDY / "case-gates.csv")
        patterns = pilot.read(admission.STUDY / "gate-patterns.csv")
        work = output / "native"
        compiler = native.compile_native(work)
        queries, records, cases = extract(raw, provenance, old_cases)
        need(
            len(cases) == pins["expected_cases"]
            and len(queries) == len(records) == pins["expected_selected_decisions"],
            "Geometry population differs",
        )
        write_queries(work / "queries.tsv", queries)
        pilot.write(work / "selected.csv", records, RECORD_FIELDS)
        main_receipt = native.evaluate(work, work / "queries.tsv")
        trace_rows = pilot.read(work / "trace.csv", native.TRACE_FIELDS)
        witness_rows = pilot.read(work / "witnesses.csv", native.WITNESS_FIELDS)
        witnesses = defaultdict(list)
        for witness in witness_rows:
            witnesses[witness["id"]].append(witness)
        obstacle_rows = pilot.read(build.FIXTURES / "obstacles.csv")
        obstacle_index = {
            i: (r["scenario"], pilot.finite(r["x"]), pilot.finite(r["y"]))
            for i, r in enumerate(obstacle_rows, 1)
        }
        need(
            len(trace_rows) == len(records)
            and set(witnesses) == {r["id"] for r in records},
            "Native trace/witness coverage differs",
        )
        rows = [
            diagnose(record, query, trace, witnesses[record["id"]], obstacle_index)
            for record, query, trace in zip(records, queries, trace_rows)
        ]
        check_previous(cases, rows, old_cases, patterns)
        pilot.write(work / "decisions.csv", rows, (*RECORD_FIELDS, *DETAIL_FIELDS))
        totals = counts(rows)
        need(
            totals["ready"] == pins["expected_ready"]
            and totals["selected_decisions"] - totals["ready"]
            == pins["expected_blocked"],
            "Frozen readiness counts differ",
        )
        traces = {r["id"]: r for r in trace_rows}
        reps = representatives(rows)
        rep_ids = {r["id"] for r in reps}
        plot_ids = {r["id"] for r in reps if pilot.truth(r["plot"])}
        need(len(plot_ids) == 3, "Expected three failure-mask plot controls")
        plot_queries = [dict(q, draw="true") for q in queries if q["id"] in plot_ids]
        write_queries(work / "plot-queries.tsv", plot_queries)
        plot_receipt = native.evaluate(work, work / "plot-queries.tsv", "plot-")
        need(
            pilot.read(work / "plot-trace.csv", native.TRACE_FIELDS)
            == [t for t in trace_rows if t["id"] in plot_ids],
            "Repeated plot geometry differs",
        )
        need(
            pilot.read(work / "plot-witnesses.csv", native.WITNESS_FIELDS)
            == [w for w in witness_rows if w["id"] in plot_ids],
            "Repeated plot witnesses differ",
        )
        need(
            len(pilot.read(work / "plot-paths.csv", native.PATH_FIELDS)) == 369,
            "Plot path coverage differs",
        )
        case_rows, summary = summaries(cases, rows, traces, witnesses)
        tables = (
            ("case-summary.csv", case_rows, CASE_FIELDS),
            ("mask-summary.csv", summary, SUMMARY_FIELDS),
            ("representatives.csv", reps, REP_FIELDS),
            (
                "representative-native.csv",
                [r for r in trace_rows if r["id"] in rep_ids],
                native.TRACE_FIELDS,
            ),
            (
                "representative-witnesses.csv",
                [w for w in witness_rows if w["id"] in rep_ids],
                native.WITNESS_FIELDS,
            ),
        )
        for name, data, fields in tables:
            pilot.write(output / name, data, fields)
        write_queries(
            output / "representative-inputs.tsv",
            [q for q in queries if q["id"] in rep_ids],
        )
        for name, src in (
            ("probe-geometry.rs", "probe-geometry.rs"),
            ("plot-paths.csv", "plot-paths.csv"),
        ):
            shutil.copyfile(work / src, output / name)
        native_sources = dict(
            compiler=compiler,
            main=main_receipt,
            plot=plot_receipt,
            native_queries=len(rows),
            probe_evaluations=3 * len(rows),
            repeated_plot_queries=3,
            repeated_plot_evaluations=9,
            extra_sha256={
                name: build.sha(work / name)
                for name in ("selected.csv", "decisions.csv", "paths.csv")
            },
        )
        (output / "native-sources.json").write_text(
            json.dumps(native_sources, indent=2) + "\n"
        )
        output_names = [name for name, _, _ in tables] + [
            "representative-inputs.tsv",
            "probe-geometry.rs",
            "plot-paths.csv",
            "native-sources.json",
        ]
        sources = dict(
            method="all selected fixed mean-arm decisions; original native geometric queries; conservative release enclosures",
            implementation_commit=implementation,
            working_tree_clean=True,
            protocol_sha256=build.sha(STUDY / "protocol.md"),
            freeze_sha256=build.sha(STUDY / "freeze-pins.json"),
            input_sha256=pins["input_sha256"],
            raw_provenance_sha256=pins["raw_provenance_sha256"],
            script_sha256={
                name: build.sha(build.ROOT / name)
                for name in (
                    "analyze_probe_geometry.py",
                    "native_probe_geometry.py",
                    "plot_probe_geometry.py",
                    "test_probe_geometry.py",
                )
            },
            audit=dict(
                cases=len(cases),
                selected_decisions=len(rows),
                blocked=pins["expected_blocked"],
                ready=pins["expected_ready"],
                witnesses=len(witness_rows),
                representatives=len(reps),
                plotted_decisions=3,
                unresolved_stop=totals["stop_unresolved"],
                unresolved_terminal=totals["terminal_unresolved"],
                fresh_simulator_steps=0,
            ),
            output_sha256={name: build.sha(output / name) for name in output_names},
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
        f"Verified {len(cases)} cases, {len(rows)} decisions, {len(witness_rows)} witnesses and {len(reps)} representatives",
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
        parser.exit(1, f"Probe geometry diagnostic failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
