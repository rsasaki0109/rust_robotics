#!/usr/bin/env python3
"""Annotate frozen stopping/terminal bounds without changing geometry or control."""

import argparse
from collections import defaultdict
import csv
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys

import analyze_release_pose_accuracy as previous
import analyze_mean_speed_probe as pilot
import build_release_mean_handoff as build

STUDY = build.ROOT / "results/post-release-clearance-components"
SCRIPTS = ("analyze_clearance_components.py", "test_clearance_components.py")
VARIANTS, SCOPES, META = previous.VARIANTS, previous.SCOPES, previous.META
RADIUS, ENTRY_RESERVE, RELEASE_RESERVE, TIE = 0.5, 0.1, 0.05, 1e-12
MECHANISMS = ("stop_only", "terminal_only", "both", "clear")
BINDINGS = ("stop", "terminal", "tie")
LAYERS = (
    "radius_level",
    "margin_only",
    "reserve_only",
    "either_sufficient",
    "jointly_required",
    "clear",
)
GAPS = (
    "stop_entry_gap_m",
    "terminal_entry_gap_m",
    "entry_gap_m",
    "stop_release_gap_m",
    "terminal_release_gap_m",
    "release_gap_m",
    "raw_radius_gap_m",
    "raw_margin_gap_m",
    "entry_radius_gap_m",
    "allowance_m",
)
COUNTS = (
    "rows",
    "eligible_rows",
    "ready_rows",
    "actual_trigger_rows",
    "zero_target_rows",
    "entry_rows",
    "ready_entry_rows",
    "release_clear_rows",
    *("entry_" + k for k in MECHANISMS),
    *("binding_" + k for k in BINDINGS),
    *("layer_" + k for k in LAYERS),
    *("release_block_" + k for k in MECHANISMS),
)
STATS = (
    "mean_entry_gap_m",
    "min_entry_gap_m",
    "max_entry_gap_m",
    "mean_stop_entry_gap_m",
    "mean_terminal_entry_gap_m",
    "mean_release_gap_m",
    "mean_allowance_m",
)
METRICS = (*COUNTS, *STATS)
CASE_FIELDS = (*META, "status", "variant", "resumes", *METRICS)
PAIR_FIELDS = (
    *previous.previous.PAIR_KEYS,
    "variant",
    *(
        p + "_" + k
        for p in ("reference", "candidate")
        for k in ("status", "resumes", *METRICS)
    ),
)
SUMMARY_FIELDS = (
    "noise_selector",
    "mode_selector",
    "arm",
    "outcome_selector",
    "scope",
    "variant",
    "cases",
    "resumes",
    *METRICS,
)
VERIFY_FIELDS = (
    *META,
    "resumes",
    "points",
    "upstream_rows",
    "native_identity_matches",
    "margin_matches",
    "group_matches",
)
SHARED = (
    "point_id",
    "transition_id",
    *META,
    "status",
    "resume_step",
    "step",
    "lag",
    "eligible",
    "entry_ready",
    "actual_trigger",
    "selected_source",
    "recovery_source",
    "count",
    "observation_age",
    "true_x",
    "true_y",
    "true_yaw",
    "true_v",
    "speed",
    "target",
    "steering",
    "recovery_clearance",
    "recovery_clearance_9",
    "recovery_bounds",
    "recovery_margin",
    "preview_margin",
    "preview_margin_9",
    "release_margin",
    "release_margin_9",
    "effective_position_sigma_9",
)
ANNOTATION_INPUTS = (
    "query_id",
    "point_id",
    "transition_id",
    *META,
    "status",
    "resume_step",
    "step",
    "lag",
    "variant",
    "eligible",
    "entry_ready",
    "actual_trigger",
    "speed",
    "target",
    "observation_age",
    "effective_position_sigma_9",
    "selected_source",
    "count",
    "stop",
    "terminal",
    "entry",
    "release",
    "preview_margin",
    "release_margin",
)
CLASS_FIELDS = (
    "entry_mechanism",
    "binding",
    "layer",
    "release_block",
    "would_enter",
    "release_clear",
    "raw_radius_check",
    "raw_margin_check",
    "reserved_radius_check",
    *GAPS,
)
ANNOTATION_FIELDS = (*ANNOTATION_INPUTS, *CLASS_FIELDS)
RESUME_FIELDS = (
    "transition_id",
    *META,
    "status",
    "resume_step",
    "followup_steps",
    "first_actual_trigger_lag",
    "first_truth_entry_lag",
    *(
        "truth_" + k
        for k in (
            "entry_rows",
            "entry_stop_only",
            "entry_terminal_only",
            "entry_both",
            "layer_radius_level",
            "layer_margin_only",
            "layer_reserve_only",
            "layer_either_sufficient",
            "layer_jointly_required",
        )
    ),
    "lag2_actual_trigger",
    *(
        "lag2_" + p + "_" + k
        for p in ("recorded", "truth")
        for k in (
            "entry_mechanism",
            "layer",
            "binding",
            "entry_gap_m",
            "stop_entry_gap_m",
            "terminal_entry_gap_m",
        )
    ),
)
BOOL_FIELDS = {
    "eligible",
    "entry_ready",
    "actual_trigger",
    "recovery_bounds",
    *previous.DISAGREEMENTS,
}
TEXT_FIELDS = {
    "case_id",
    "arm",
    "mode",
    "scenario",
    "planner",
    "status",
    "variant",
    "selected_source",
    "recovery_source",
    "comparison",
}
NUMERIC_FIELDS = set(previous.SAMPLE_FIELDS) - BOOL_FIELDS - TEXT_FIELDS
need = build.require


def preflight():
    need(sys.flags.optimize == 0, "Python-O is unsupported")
    pins = json.loads((STUDY / "freeze-pins.json").read_text())
    need(
        build.sha(STUDY / "protocol.md") == pins["protocol_sha256"],
        "Component protocol changed",
    )
    for name, digest in pins["input_sha256"].items():
        need(build.sha(build.REPO / name) == digest, f"Changed component input: {name}")
    need(
        tuple(pins["variants"]) == VARIANTS
        and tuple(pins["scopes"]) == SCOPES
        and (
            pins["radius"],
            pins["entry_reserve"],
            pins["release_reserve"],
            pins["tie_epsilon_m"],
        )
        == (RADIUS, ENTRY_RESERVE, RELEASE_RESERVE, TIE),
        "Component design differs",
    )
    return pins


def mechanism(first, second):
    return (
        "both"
        if first and second
        else "stop_only" if first else "terminal_only" if second else "clear"
    )


def classify(stop, terminal, margin, release_margin):
    need(
        all(math.isfinite(v) for v in (stop, terminal, margin, release_margin)),
        "Nonfinite component value",
    )
    need(
        margin >= RADIUS and release_margin == margin + RELEASE_RESERVE,
        "Component margins differ",
    )
    terminal_entry = terminal - ENTRY_RESERVE
    terminal_release = terminal - RELEASE_RESERVE
    entry = min(stop, terminal_entry)
    release = min(stop, terminal_release)
    raw = min(stop, terminal)
    enters = entry <= margin
    radius_check, margin_check, reserve_check = (
        raw <= RADIUS,
        raw <= margin,
        entry <= RADIUS,
    )
    if not enters:
        layer = "clear"
    elif radius_check:
        layer = "radius_level"
    elif margin_check and reserve_check:
        layer = "either_sufficient"
    elif margin_check:
        layer = "margin_only"
    elif reserve_check:
        layer = "reserve_only"
    else:
        layer = "jointly_required"
    binding = (
        "tie"
        if abs(stop - terminal_entry) <= TIE
        else "stop" if stop < terminal_entry else "terminal"
    )
    return dict(
        entry_mechanism=mechanism(stop <= margin, terminal_entry <= margin),
        binding=binding,
        layer=layer,
        release_block=mechanism(
            stop <= release_margin, terminal_release <= release_margin
        ),
        would_enter=enters,
        release_clear=release > release_margin,
        raw_radius_check=radius_check,
        raw_margin_check=margin_check,
        reserved_radius_check=reserve_check,
        stop_entry_gap_m=stop - margin,
        terminal_entry_gap_m=terminal_entry - margin,
        entry_gap_m=entry - margin,
        stop_release_gap_m=stop - release_margin,
        terminal_release_gap_m=terminal_release - release_margin,
        release_gap_m=release - release_margin,
        raw_radius_gap_m=raw - RADIUS,
        raw_margin_gap_m=raw - margin,
        entry_radius_gap_m=entry - RADIUS,
        allowance_m=margin - RADIUS,
    )


def annotate(row):
    need(tuple(row) == previous.SAMPLE_FIELDS, "Component input schema differs")
    values = {k: pilot.finite(row[k]) for k in NUMERIC_FIELDS}
    for k in BOOL_FIELDS:
        pilot.truth(row[k])
    margin = (
        RADIUS
        + 0
        + 2 * math.sqrt(2) * values["effective_position_sigma_9"]
        + 0.02 * values["observation_age"]
    )
    need(
        margin == values["preview_margin"], "Original ordered component margin differs"
    )
    stop, terminal = values["stop"], values["terminal"]
    need(
        values["entry"] == min(stop, terminal - ENTRY_RESERVE)
        and values["release"] == min(stop, terminal - RELEASE_RESERVE),
        "Original native scalar identity differs",
    )
    result = {k: row[k] for k in ANNOTATION_INPUTS}
    for k in ("eligible", "entry_ready", "actual_trigger"):
        result[k] = pilot.truth(row[k])
    result["lag"] = int(row["lag"])
    result.update(
        classify(stop, terminal, values["preview_margin"], values["release_margin"])
    )
    need(
        result["would_enter"] == (result["entry_mechanism"] != "clear")
        and result["release_clear"] == (result["release_block"] == "clear"),
        "Component predicate conservation differs",
    )
    return result


def read_annotations(path, catalogue, events):
    cases = {c["case_id"]: c for c in catalogue}
    expected = {int(e["transition_id"]): e for e in events}
    need(
        len(cases) == len(catalogue) and len(expected) == len(events),
        "Duplicate component catalogue/event",
    )
    checks = {
        ident: dict(
            **{k: c[k] for k in META},
            resumes=int(c["resume_count"]),
            points=0,
            upstream_rows=0,
            native_identity_matches=0,
            margin_matches=0,
            group_matches=0,
        )
        for ident, c in cases.items()
    }
    annotations = []
    point_id = query_id = current_tid = previous_lag = 0
    seen = set()
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        need(
            tuple(reader.fieldnames or ()) == previous.SAMPLE_FIELDS,
            "Component CSV header differs",
        )
        for key, rows in itertools.groupby(reader, lambda r: r["point_id"]):
            group = list(rows)
            point_id += 1
            need(
                key == str(point_id) and tuple(r["variant"] for r in group) == VARIANTS,
                "Component point/variant sequence differs",
            )
            first = group[0]
            tid, lag = int(first["transition_id"]), int(first["lag"])
            need(
                0 <= lag <= 50 and tid in expected and first["case_id"] in cases,
                "Unknown component window",
            )
            event, case = expected[tid], cases[first["case_id"]]
            need(
                all(first[k] == case[k] for k in META)
                and first["status"] == case["status"],
                "Component case metadata differs",
            )
            need(
                first["case_id"] == event["case_id"]
                and first["status"] == event["status"]
                and first["resume_step"] == event["resume_step"]
                and int(first["step"]) == int(event["resume_step"]) + lag,
                "Component event join differs",
            )
            if lag == 0:
                if current_tid:
                    need(
                        previous_lag == int(expected[current_tid]["followup_steps"]),
                        "Truncated component window",
                    )
                need(
                    tid == current_tid + 1 and tid not in seen,
                    "Repeated/reordered component anchor",
                )
                current_tid = tid
                seen.add(tid)
            else:
                need(
                    tid == current_tid and lag == previous_lag + 1,
                    "Component lag sequence differs",
                )
            previous_lag = lag
            need(lag <= int(event["followup_steps"]), "Extra component following point")
            for row in group:
                query_id += 1
                need(
                    None not in row
                    and None not in row.values()
                    and row["query_id"] == str(query_id),
                    "Malformed/reordered component query",
                )
                need(
                    all(row[k] == first[k] for k in SHARED),
                    "Mixed same-point component inputs",
                )
                annotations.append(annotate(row))
            check = checks[first["case_id"]]
            check["points"] += 1
            for k in (
                "upstream_rows",
                "native_identity_matches",
                "margin_matches",
                "group_matches",
            ):
                check[k] += len(group)
    if current_tid:
        need(
            previous_lag == int(expected[current_tid]["followup_steps"]),
            "Truncated final component window",
        )
    need(
        seen == set(expected)
        and point_id == sum(int(e["followup_steps"]) + 1 for e in events)
        and query_id == point_id * len(VARIANTS),
        "Incomplete component population",
    )
    for ident, case in cases.items():
        need(
            sum(e["case_id"] == ident for e in events) == int(case["resume_count"]),
            "Component resume census differs",
        )
    return annotations, list(checks.values())


def metrics(rows):
    result = dict(
        rows=len(rows),
        eligible_rows=sum(r["eligible"] for r in rows),
        ready_rows=sum(r["entry_ready"] for r in rows),
        actual_trigger_rows=sum(r["actual_trigger"] for r in rows),
        zero_target_rows=sum(pilot.finite(r["target"]) == 0 for r in rows),
        entry_rows=sum(r["would_enter"] for r in rows),
        ready_entry_rows=sum(r["entry_ready"] and r["would_enter"] for r in rows),
        release_clear_rows=sum(r["release_clear"] for r in rows),
    )
    for prefix, field, labels in (
        ("entry_", "entry_mechanism", MECHANISMS),
        ("binding_", "binding", BINDINGS),
        ("layer_", "layer", LAYERS),
        ("release_block_", "release_block", MECHANISMS),
    ):
        result.update({prefix + k: sum(r[field] == k for r in rows) for k in labels})
    result.update(dict.fromkeys(STATS, ""))
    if rows:
        for key, field in (
            ("mean_entry_gap_m", "entry_gap_m"),
            ("mean_stop_entry_gap_m", "stop_entry_gap_m"),
            ("mean_terminal_entry_gap_m", "terminal_entry_gap_m"),
            ("mean_release_gap_m", "release_gap_m"),
            ("mean_allowance_m", "allowance_m"),
        ):
            result[key] = math.fsum(r[field] for r in rows) / len(rows)
        result.update(
            min_entry_gap_m=min(r["entry_gap_m"] for r in rows),
            max_entry_gap_m=max(r["entry_gap_m"] for r in rows),
        )
    return result


def tables(catalogue, annotations):
    index = defaultdict(list)
    for r in annotations:
        index[r["case_id"], r["variant"]].append(r)
    cases, pairs, summary = [], [], []
    paired = defaultdict(dict)
    for c in catalogue:
        for variant in VARIANTS:
            row = dict(
                **{k: c[k] for k in META},
                status=c["status"],
                variant=variant,
                resumes=int(c["resume_count"]),
                **metrics(index[c["case_id"], variant]),
            )
            cases.append(row)
            key = (*[c[k] for k in previous.previous.PAIR_KEYS], variant)
            need(c["arm"] not in paired[key], "Duplicate component case arm")
            paired[key][c["arm"]] = row
    for key, arms in paired.items():
        need(
            set(arms) == {"mean_preview", "one_step_handoff"},
            "Incomplete component pair",
        )
        row = dict(zip((*previous.previous.PAIR_KEYS, "variant"), key))
        for prefix, arm in (
            ("reference", "mean_preview"),
            ("candidate", "one_step_handoff"),
        ):
            row.update(
                {
                    prefix + "_" + k: arms[arm][k]
                    for k in ("status", "resumes", *METRICS)
                }
            )
        pairs.append(row)
    for noise, mode, arm, outcome, scope, variant in itertools.product(
        ("all", "white", "correlated"),
        ("all", "none", "random", "blackout"),
        ("mean_preview", "one_step_handoff"),
        ("all", "held", "timeout"),
        SCOPES,
        VARIANTS,
    ):
        selected = [
            c
            for c in catalogue
            if c["arm"] == arm
            and (noise == "all" or (float(c["tau"]) == 0) == (noise == "white"))
            and (mode == "all" or c["mode"] == mode)
            and (outcome == "all" or c["status"] == outcome)
        ]
        rows = [
            r
            for c in selected
            for r in index[c["case_id"], variant]
            if previous.in_scope(r, scope)
        ]
        summary.append(
            dict(
                noise_selector=noise,
                mode_selector=mode,
                arm=arm,
                outcome_selector=outcome,
                scope=scope,
                variant=variant,
                cases=len(selected),
                resumes=sum(int(c["resume_count"]) for c in selected),
                **metrics(rows),
            )
        )
    return cases, pairs, summary


def resume_rows(events, annotations):
    by = defaultdict(list)
    for r in annotations:
        by[int(r["transition_id"])].append(r)
    result = []
    for e in events:
        tid = int(e["transition_id"])
        rows = by[tid]
        truth = [r for r in rows if r["variant"] == "truth_pose_eval" and r["lag"] > 0]
        first = next((r["lag"] for r in truth if r["would_enter"]), "")
        stats = metrics(truth)
        record = dict(
            transition_id=tid,
            **{k: e[k] for k in META},
            status=e["status"],
            resume_step=e["resume_step"],
            followup_steps=e["followup_steps"],
            first_actual_trigger_lag=e["first_trigger_lag"],
            first_truth_entry_lag=first,
        )
        for k in (
            "entry_rows",
            "entry_stop_only",
            "entry_terminal_only",
            "entry_both",
            "layer_radius_level",
            "layer_margin_only",
            "layer_reserve_only",
            "layer_either_sufficient",
            "layer_jointly_required",
        ):
            record["truth_" + k] = stats[k]
        lag2 = {r["variant"]: r for r in rows if r["lag"] == 2}
        record["lag2_actual_trigger"] = (
            lag2["recorded"]["actual_trigger"] if lag2 else ""
        )
        for prefix, variant in (("recorded", "recorded"), ("truth", "truth_pose_eval")):
            for k in (
                "entry_mechanism",
                "layer",
                "binding",
                "entry_gap_m",
                "stop_entry_gap_m",
                "terminal_entry_gap_m",
            ):
                record["lag2_" + prefix + "_" + k] = lag2[variant][k] if lag2 else ""
        result.append(record)
    return result


def evaluate(input_directory, catalogue, events, output):
    annotations, checks = read_annotations(
        input_directory / "samples.csv", catalogue, events
    )
    cases, pairs, summary = tables(catalogue, annotations)
    resumes = resume_rows(events, annotations)
    output.mkdir(parents=True)
    for name, rows, fields in (
        ("cases.csv", cases, CASE_FIELDS),
        ("pairs.csv", pairs, PAIR_FIELDS),
        ("summary.csv", summary, SUMMARY_FIELDS),
        ("verification.csv", checks, VERIFY_FIELDS),
        ("resumes.csv", resumes, RESUME_FIELDS),
        ("annotations.csv", annotations, ANNOTATION_FIELDS),
    ):
        previous.write(output / name, rows, fields)
    return dict(
        cases=len(catalogue),
        case_variant_rows=len(cases),
        pairs=len(pairs) // len(VARIANTS),
        pair_variant_rows=len(pairs),
        resumes=len(resumes),
        points=sum(c["points"] for c in checks),
        rows=len(annotations),
        summary_cells=len(summary),
        native_identity_matches=sum(c["native_identity_matches"] for c in checks),
        margin_matches=sum(c["margin_matches"] for c in checks),
        group_matches=sum(c["group_matches"] for c in checks),
    )


def collect(input_one, input_two, output):
    pins = preflight()
    input_one, input_two = Path(input_one).resolve(), Path(input_two).resolve()
    output = build.fresh(output)
    need(
        input_one != input_two
        and not any(output.is_relative_to(p) for p in (input_one, input_two)),
        "Use distinct retained inputs and external component output",
    )
    need(
        not subprocess.check_output(
            ["git", "status", "--porcelain=v1"], cwd=build.REPO, text=True
        ).strip(),
        "Commit component implementation before annotation",
    )
    old = build.REPO / pins["previous_study"]
    proof = json.loads((old / "repeat-verification.json").read_text())
    for directory in (input_one, input_two):
        previous.previous.verify_manifest(directory, proof["sha256"])
    live = build.REPO / pins["live_study"]
    catalogue = pilot.read(live / "cases.csv")
    events = pilot.read(old / "resumes.csv")
    try:
        counts = evaluate(input_one, catalogue, events, output)
        need(
            (
                counts["cases"],
                counts["pairs"],
                counts["resumes"],
                counts["points"],
                counts["rows"],
            )
            == (
                pins["expected_cases"],
                pins["expected_pairs"],
                pins["expected_resumes"],
                pins["expected_points"],
                pins["expected_rows"],
            ),
            "Component denominator differs",
        )
        upstream = json.loads((input_one / "sources.json").read_text())
        sources = dict(
            method="Scalar classification of the same frozen native bounds; no new simulation/native geometry or physical audit",
            implementation_commit=subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=build.REPO, text=True
            ).strip(),
            script_sha256={name: build.sha(build.ROOT / name) for name in SCRIPTS},
            protocol_sha256=pins["protocol_sha256"],
            freeze_sha256=build.sha(STUDY / "freeze-pins.json"),
            input_sha256=pins["input_sha256"],
            parent_manifest_sha256=build.sha(old / "repeat-verification.json"),
            parent_outputs=proof["sha256"],
            upstream_raw_provenance_sha256=upstream["raw_provenance_sha256"],
            upstream_implementation_commit=upstream["implementation_commit"],
            counts=counts,
            output_sha256={
                p.name: build.sha(p) for p in sorted(output.iterdir()) if p.is_file()
            },
        )
        (output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    except Exception as exc:
        if output.exists():
            (output / "failure.log").write_text(repr(exc) + "\n")
        raise
    print(json.dumps(counts), flush=True)
    return sources


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("input-one", "input-two", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    args = parser.parse_args()
    collect(args.input_one, args.input_two, args.output)


if __name__ == "__main__":
    main()
