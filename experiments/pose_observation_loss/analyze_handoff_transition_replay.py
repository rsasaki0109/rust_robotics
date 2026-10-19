#!/usr/bin/env python3
"""Diagnose all post-handoff windows without changing recorded commands or state."""

import argparse
from collections import defaultdict
from contextlib import ExitStack
import csv
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys

import analyze_mean_speed_probe as pilot
import analyze_post_release_transition as previous
import analyze_release_mean_handoff as live
import build_release_mean_handoff as build
import native_handoff_transition_replay as native

STUDY = build.ROOT / "results/post-handoff-transition-replay"
SCRIPTS = (
    "analyze_handoff_transition_replay.py",
    "native_handoff_transition_replay.py",
    "test_handoff_transition_replay.py",
)
META = ("case_id", *pilot.CASE_META)
REASONS = (
    "joint_pending",
    "correction",
    "received",
    "precision_activation",
    "moving",
    "stale",
    "insufficient_samples",
)
ALTERNATIVES = dict(diagnostic="carried_diagnostic_xy", gated="gated_carried_xy")
FLIPS = previous.FLIP_FIELDS
COUNTS = (
    "following",
    "valid_following",
    "invalid_following",
    "zero_target_following",
    "after_joint_pause_following",
    "entry_ready_following",
    "actual_triggers",
    *(
        prefix + "_" + field
        for prefix in ALTERNATIVES
        for field in (*FLIPS, "ready_entry_avoided", "ready_entry_added")
    ),
)
CASE_FIELDS = (*META, "status", "decisions", "resumes", *COUNTS)
FOLLOW_FIELDS = (
    "transition_id",
    *META,
    "status",
    "resume_step",
    "step",
    "lag",
    "time_s",
    "valid_now",
    "valid_prefix",
    "invalid_reasons",
    "first_invalid_lag",
    "after_joint_pause",
    "entry_ready",
    "actual_trigger",
    "actual_guard_active",
    "actual_handoff_used",
    "recovery_count",
    "observation_age",
    "measured_speed",
    "nominal_target",
    "nominal_steering",
    "recorded_entry",
    "recorded_release",
    "preview_margin",
    "release_margin",
    "carried_x",
    "carried_y",
    "carried_yaw",
    "position_delta_m",
    *COUNTS[7:],
)
EVENT_FIELDS = (
    "transition_id",
    *META,
    "status",
    "resume_step",
    "resume_time_s",
    "followup_steps",
    "endpoint_truncated",
    "first_invalid_lag",
    "first_invalid_reasons",
    "first_joint_lag",
    "first_trigger_lag",
    "first_trigger_before_joint_lag",
    *("observed_" + str(h) for h in (1, 2, 5, 25, 50)),
    *("valid_through_" + str(h) for h in (1, 2, 5, 25, 50)),
)
QUERY_FIELDS = (
    "transition_id",
    *META,
    "status",
    "resume_step",
    "step",
    *(f for f in native.TRACE_FIELDS if f not in ("transition_id", "scenario")),
    "would_enter",
    "release_clear",
)
VERIFY_FIELDS = (
    *META,
    "decisions",
    "resumes",
    "anchor_queries",
    "following_queries",
    "recorded_geometry_matches",
    "transport_matches",
    "actual_handoff_matches",
)
PAIR_KEYS = ("tau", "mode", "seed", "scenario", "planner")
PAIR_FIELDS = (
    *PAIR_KEYS,
    *(
        p + "_" + f
        for p in ("reference", "candidate")
        for f in ("status", "resumes", *COUNTS)
    ),
)
SUMMARY_FIELDS = (
    "noise_selector",
    "mode_selector",
    "arm",
    "outcome_selector",
    "scope",
    "cases",
    "resumes",
    *COUNTS,
)
need = build.require


def preflight():
    need(sys.flags.optimize == 0, "Python-O is unsupported")
    pins = json.loads((STUDY / "freeze-pins.json").read_text())
    need(
        build.sha(STUDY / "protocol.md") == pins["protocol_sha256"],
        "Replay protocol changed",
    )
    for name, digest in pins["input_sha256"].items():
        need(build.sha(build.REPO / name) == digest, f"Changed replay input: {name}")
    need(
        tuple(pins["variants"]) == native.VARIANTS and pins["horizon"] == 50,
        "Frozen replay variants/horizon differ",
    )
    return pins


def verify_manifest(root, records):
    previous.verify_manifest(root, records)


def invalid_reasons(item):
    side, guard = item["side"], item["guard"]
    flags = {
        k: pilot.truth(guard[k] if k == "joint_pending" else side[k])
        for k in REASONS[:4]
    }
    flags.update(
        moving=abs(pilot.finite(side["measured_speed"])) > 0.02,
        stale=pilot.finite(side["observation_age"]) > 1.5,
        insufficient_samples=int(side["count"]) < 10,
    )
    return [key for key in REASONS if flags[key]]


def transported(carry, predecessor):
    distance = pilot.finite(predecessor["measured_speed"]) * pilot.DT
    turn = distance / 0.4 * math.tan(pilot.finite(predecessor["applied_steering"]))
    x, y, yaw = carry
    return (
        x + distance * math.cos(yaw),
        y + distance * math.sin(yaw),
        math.atan2(math.sin(yaw + turn), math.cos(yaw + turn)),
    )


def select_windows(case, track, sides, guards, handoffs, horizon):
    need(
        len(track)
        == len(sides) + 1
        == len(guards) + 1
        == len(handoffs) + 1
        == int(case["decisions"]) + 1,
        "Incomplete window case",
    )
    for step, (row, side, guard, handoff) in enumerate(
        zip(track[1:], sides, guards, handoffs), 1
    ):
        need(
            len(row) == 82
            and tuple(side) == build.legacy.legacy.SIDECAR
            and tuple(guard) == build.legacy.GUARD_FIELDS
            and tuple(handoff) == build.HANDOFF_FIELDS,
            "Window schema differs",
        )
        for record in (row, side, guard, handoff):
            need(
                (record["scenario"], record["planner"])
                == (case["scenario"], case["planner"]),
                "Mixed window case",
            )
        need(
            all(int(s["step"]) == step for s in (side, guard, handoff)),
            "Window step sequence differs",
        )
        need(
            len({s["time_s"] for s in (side, guard, handoff)}) == 1,
            "Window sidecar times differ",
        )
        need(
            abs(pilot.finite(row["time_s"]) - pilot.finite(side["time_s"])) <= 5e-7,
            "Window trajectory time differs",
        )
    windows = []
    for step, row in enumerate(track[1:], 1):
        if not pilot.truth(row["resumed"]):
            continue
        need(
            not pilot.truth(row["guard_active"])
            and not pilot.truth(row["safety_paused"])
            and pilot.truth(guards[step - 1]["joint_pending"]),
            "Joint resume does not clear brakes",
        )
        points = []
        for following in range(step, min(len(track) - 1, step + horizon) + 1):
            points.append(
                dict(
                    step=following,
                    row=track[following],
                    previous_row=track[following - 1],
                    side=sides[following - 1],
                    guard=guards[following - 1],
                    handoff=handoffs[following - 1],
                )
            )
        windows.append(dict(case=case, resume_step=step, points=points))
    need(len(windows) == int(case["resume_count"]), "Resume census differs")
    return windows


def collect_windows(raw, cases, pins):
    expected = {c["case_id"]: c for c in cases}
    need(
        len(expected) == len(cases) == pins["expected_cases"], "Case catalogue differs"
    )
    seen, windows, decisions = set(), [], 0
    for tau, mode, arm in itertools.product(
        pins["noise_correlation_times"], pins["modes"], pins["arms"]
    ):
        directory = raw / f"tau{tau:g}/{arm}-{mode}-{pins['seed']}"
        with ExitStack() as stack:
            streams = [
                stack.enter_context((directory / name).open(newline=""))
                for name in (
                    "trajectories.csv",
                    "speed-probes.csv",
                    "guard-preview-inputs.csv",
                    "handoff-inputs.csv",
                )
            ]
            local = set()
            for groups in itertools.zip_longest(*(pilot.grouped(s) for s in streams)):
                need(
                    all(g is not None for g in groups)
                    and len({g[0] for g in groups}) == 1,
                    "Window grouping differs",
                )
                key = groups[0][0]
                meta = dict(
                    tau=tau,
                    mode=mode,
                    arm=arm,
                    seed=pins["seed"],
                    scenario=key[0],
                    planner=key[1],
                )
                ident = live.case_id(meta)
                need(
                    ident in expected and ident not in seen and key not in local,
                    "Unexpected/repeated window case",
                )
                seen.add(ident)
                local.add(key)
                case = expected[ident]
                need(live.case_id(case) == ident, "Case metadata differs")
                windows.extend(
                    select_windows(case, *(g[1] for g in groups), pins["horizon"])
                )
                decisions += int(case["decisions"])
        print(f"Selected all windows tau={tau:g} mode={mode} arm={arm}", flush=True)
    need(
        seen == set(expected)
        and decisions == pins["expected_decisions"]
        and len(windows) == pins["expected_resumes"],
        "Incomplete window population",
    )
    for i, window in enumerate(windows, 1):
        window["transition_id"] = i
    return windows


def bind_baselines(audit, expected_count):
    rows = pilot.read(audit / "native-queries.csv", live.native.TRACE_FIELDS)
    need(len(rows) == expected_count, "Retained native query count differs")
    result = {}
    for i, row in enumerate(rows, 1):
        key = (row["case_id"], int(row["step"]))
        need(
            int(row["id"]) == i and key not in result,
            "Retained native query sequence differs",
        )
        result[key] = row
    return result


def write_packet(work, windows):
    sequence = 0
    packet = work / "queries.tsv"
    with packet.open("x", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=native.INPUT_FIELDS, delimiter="\t", lineterminator="\n"
        )
        writer.writeheader()
        for window in windows:
            for lag, item in enumerate(window["points"]):
                sequence += 1
                item["query_id"] = sequence
                record = previous.census_input(
                    dict(item, global_id=sequence), window["case"]["scenario"]
                )
                record.update(
                    transition_id=window["transition_id"],
                    lag=lag,
                    applied_steering=item["side"]["applied_steering"],
                    joint_pending=item["guard"]["joint_pending"],
                    **{
                        k: item["side"][k]
                        for k in (
                            "correction",
                            "received",
                            "precision_activation",
                            "count",
                        )
                    },
                )
                writer.writerow(record)
    return packet, sequence


def validate_recorded(item, geometry, baseline, case):
    row, side, guard, handoff = (item[k] for k in ("row", "side", "guard", "handoff"))
    need(
        baseline["case_id"] == case["case_id"]
        and baseline["scenario"] == case["scenario"]
        and int(baseline["step"]) == item["step"],
        "Retained native join differs",
    )
    for key, original in (
        ("x", "selected_x"),
        ("y", "selected_y"),
        ("yaw", "selected_yaw"),
        ("speed", "selected_v"),
    ):
        need(
            pilot.finite(geometry[key]) == pilot.finite(guard[original]),
            "Recorded selected pose differs",
        )
    for key, original in (("target", "nominal_target"), ("steering", "steering")):
        need(
            pilot.finite(geometry[key]) == pilot.finite(side[original]),
            "Recorded nominal input differs",
        )
    for key in (
        "x",
        "y",
        "yaw",
        "target",
        "steering",
        "preview_margin",
        "release_margin",
        "stop",
        "terminal",
        "release",
    ):
        need(geometry[key] == baseline[key], f"Recorded native geometry differs: {key}")
    need(
        baseline["used"] == guard["used"]
        and baseline["handoff_used"] == handoff["used"],
        "Retained selector flags differ",
    )
    for key, column in (
        ("stop_9", "stop_preview_clearance_lower_bound"),
        ("terminal_9", "terminal_recovery_clearance"),
        ("entry_9", "preview_clearance_lower_bound"),
        ("release_9", "release_preview_clearance_lower_bound"),
        ("preview_margin_9", "preview_margin"),
        ("release_margin_9", "guard_release_margin"),
    ):
        need(
            geometry[key] == row[column],
            f"Original trajectory geometry differs: {column}",
        )
    ready = not pilot.truth(item["previous_row"]["guard_active"]) and not pilot.truth(
        guard["joint_pending"]
    )
    if ready:
        need(
            previous.threshold(geometry)["would_enter"]
            == pilot.truth(row["guard_triggered"]),
            "Original inactive guard entry differs",
        )
    return ready


def near(actual, expected, message):
    need(abs(pilot.finite(actual) - expected) <= 3e-12, message)


def consume_window(window, geometries, baselines):
    case, points = window["case"], window["points"]
    meta = {k: case[k] for k in META}
    tid = window["transition_id"]
    anchor = points[0]["side"]
    carry = tuple(pilot.finite(anchor["mean_" + k]) for k in ("x", "y", "yaw"))
    valid_prefix, first_invalid, first_reasons, first_joint = True, "", "", ""
    queries, followups = [], []
    checks = dict.fromkeys(
        (
            "anchor_queries",
            "following_queries",
            "recorded_geometry_matches",
            "transport_matches",
            "actual_handoff_matches",
        ),
        0,
    )
    for lag, item in enumerate(points):
        reasons = invalid_reasons(item)
        valid_now = not reasons
        if lag:
            carry = transported(carry, points[lag - 1]["side"])
            valid_prefix = valid_prefix and valid_now
            if not valid_prefix and first_invalid == "":
                first_invalid, first_reasons = lag, "|".join(reasons)
            if pilot.truth(item["guard"]["joint_pending"]) and first_joint == "":
                first_joint = lag
        variants = {}
        for variant in (("recorded",) if lag == 0 else native.VARIANTS):
            geometry = next(geometries, None)
            need(
                geometry is not None
                and geometry["id"] == str(item["query_id"])
                and geometry["transition_id"] == str(tid)
                and geometry["lag"] == str(lag)
                and geometry["variant"] == variant
                and geometry["scenario"] == case["scenario"],
                "Native replay sequence/join differs",
            )
            need(
                pilot.truth(geometry["valid_now"]) == valid_now
                and pilot.truth(geometry["valid_prefix"]) == valid_prefix,
                "Native validity differs",
            )
            for k, expected in zip(("x", "y", "yaw"), carry):
                near(
                    geometry["carry_" + k],
                    expected,
                    "Sequential native transport differs",
                )
            for k, original in (("yaw", "selected_yaw"), ("speed", "selected_v")):
                need(
                    pilot.finite(geometry[k]) == pilot.finite(item["guard"][original]),
                    "Variant changed yaw/speed",
                )
            for k, original in (("target", "nominal_target"), ("steering", "steering")):
                need(
                    pilot.finite(geometry[k]) == pilot.finite(item["side"][original]),
                    "Variant changed nominal command",
                )
            use_carry = variant == "carried_diagnostic_xy" or (
                variant == "gated_carried_xy" and valid_prefix
            )
            if variant != "recorded":
                for k in ("x", "y"):
                    expected = (
                        geometry["carry_" + k] if use_carry else variants["recorded"][k]
                    )
                    need(geometry[k] == expected, "Variant selected position differs")
                if not use_carry:
                    for k in previous.old_native.TRACE_FIELDS[1:]:
                        need(
                            geometry[k] == variants["recorded"][k],
                            "Invalid gated geometry differs from recorded",
                        )
                for k in (
                    "recovery_margin",
                    "preview_margin",
                    "release_margin",
                    "effective_position_sigma_9",
                ):
                    need(
                        geometry[k] == variants["recorded"][k],
                        "Variant changed margin/precision",
                    )
            variants[variant] = geometry
            queries.append(
                dict(
                    transition_id=tid,
                    **meta,
                    status=case["status"],
                    resume_step=window["resume_step"],
                    step=item["step"],
                    **{
                        k: v
                        for k, v in geometry.items()
                        if k not in ("transition_id", "scenario")
                    },
                    **previous.threshold(geometry),
                )
            )
        recorded = variants["recorded"]
        key = (case["case_id"], item["step"])
        need(key in baselines, "Missing retained native baseline")
        ready = validate_recorded(item, recorded, baselines[key], case)
        checks["recorded_geometry_matches"] += 1
        checks["anchor_queries" if not lag else "following_queries"] += 1
        if not lag:
            continue
        checks["transport_matches"] += 1
        if lag == 1 and case["arm"] == "one_step_handoff" and valid_prefix:
            need(
                pilot.truth(item["handoff"]["used"]),
                "Valid candidate lag1 lacks actual handoff",
            )
            for k in ("x", "y"):
                need(
                    variants["carried_diagnostic_xy"][k] == recorded[k],
                    "Lag1 transport differs from actual handoff",
                )
            checks["actual_handoff_matches"] += 1
        flips = {}
        for prefix, variant in ALTERNATIVES.items():
            values = previous.flips(recorded, variants[variant])
            flips.update({prefix + "_" + k: v for k, v in values.items()})
            flips[prefix + "_ready_entry_avoided"] = ready and values["entry_avoided"]
            flips[prefix + "_ready_entry_added"] = ready and values["entry_added"]
        side, row = item["side"], item["row"]
        followups.append(
            dict(
                transition_id=tid,
                **meta,
                status=case["status"],
                resume_step=window["resume_step"],
                step=item["step"],
                lag=lag,
                time_s=row["time_s"],
                valid_now=valid_now,
                valid_prefix=valid_prefix,
                invalid_reasons="|".join(reasons),
                first_invalid_lag=first_invalid,
                after_joint_pause=first_joint != "",
                entry_ready=ready,
                actual_trigger=pilot.truth(row["guard_triggered"]),
                actual_guard_active=pilot.truth(row["guard_active"]),
                actual_handoff_used=pilot.truth(item["handoff"]["used"]),
                recovery_count=side["count"],
                observation_age=side["observation_age"],
                measured_speed=side["measured_speed"],
                nominal_target=side["nominal_target"],
                nominal_steering=side["steering"],
                recorded_entry=recorded["entry"],
                recorded_release=recorded["release"],
                preview_margin=recorded["preview_margin"],
                release_margin=recorded["release_margin"],
                carried_x=recorded["carry_x"],
                carried_y=recorded["carry_y"],
                carried_yaw=recorded["carry_yaw"],
                position_delta_m=math.hypot(
                    pilot.finite(recorded["carry_x"]) - pilot.finite(recorded["x"]),
                    pilot.finite(recorded["carry_y"]) - pilot.finite(recorded["y"]),
                ),
                **flips,
            )
        )
    triggers = [f["lag"] for f in followups if f["actual_trigger"]]
    before_joint = [lag for lag in triggers if first_joint == "" or lag < first_joint]
    event = dict(
        transition_id=tid,
        **meta,
        status=case["status"],
        resume_step=window["resume_step"],
        resume_time_s=points[0]["row"]["time_s"],
        followup_steps=len(followups),
        endpoint_truncated=len(followups) < 50,
        first_invalid_lag=first_invalid,
        first_invalid_reasons=first_reasons,
        first_joint_lag=first_joint,
        first_trigger_lag=triggers[0] if triggers else "",
        first_trigger_before_joint_lag=before_joint[0] if before_joint else "",
    )
    for h in (1, 2, 5, 25, 50):
        event["observed_" + str(h)] = len(followups) >= h
        event["valid_through_" + str(h)] = len(followups) >= h and (
            first_invalid == "" or first_invalid > h
        )
    return event, followups, queries, checks


def counts(rows):
    result = dict(
        following=len(rows),
        valid_following=sum(r["valid_prefix"] for r in rows),
        invalid_following=sum(not r["valid_prefix"] for r in rows),
        zero_target_following=sum(pilot.finite(r["nominal_target"]) == 0 for r in rows),
        after_joint_pause_following=sum(r["after_joint_pause"] for r in rows),
        entry_ready_following=sum(r["entry_ready"] for r in rows),
        actual_triggers=sum(r["actual_trigger"] for r in rows),
    )
    result.update({k: sum(r[k] for r in rows) for k in COUNTS[7:]})
    return result


def tables(catalogue, events, followups, checks):
    event_index, follow_index = defaultdict(list), defaultdict(list)
    for e in events:
        event_index[e["case_id"]].append(e)
    for f in followups:
        follow_index[f["case_id"]].append(f)
    cases, verification, paired = [], [], defaultdict(dict)
    for original in catalogue:
        ident = original["case_id"]
        result = dict(
            **{k: original[k] for k in META},
            status=original["status"],
            decisions=int(original["decisions"]),
            resumes=len(event_index[ident]),
            **counts(follow_index[ident]),
        )
        need(
            result["resumes"] == int(original["resume_count"]),
            "Exported resume conservation differs",
        )
        cases.append(result)
        verification.append(
            dict(
                **{k: result[k] for k in META},
                decisions=result["decisions"],
                resumes=result["resumes"],
                **checks[ident],
            )
        )
        key = tuple(original[k] for k in PAIR_KEYS)
        need(original["arm"] not in paired[key], "Duplicate case arm")
        paired[key][original["arm"]] = result
    pairs = []
    for key, arms in paired.items():
        need(
            set(arms) == {"mean_preview", "one_step_handoff"}, "Incomplete paired arms"
        )
        pair = dict(zip(PAIR_KEYS, key))
        for prefix, arm in (
            ("reference", "mean_preview"),
            ("candidate", "one_step_handoff"),
        ):
            pair.update(
                {prefix + "_" + k: arms[arm][k] for k in ("status", "resumes", *COUNTS)}
            )
        pairs.append(pair)
    summary = []
    scopes = {
        "lag1": lambda lag: lag == 1,
        "lag2": lambda lag: lag == 2,
        "within5": lambda lag: lag <= 5,
        "within25": lambda lag: lag <= 25,
        "within50": lambda lag: lag <= 50,
    }
    for noise, mode, arm, outcome, scope in itertools.product(
        ("all", "white", "correlated"),
        ("all", "none", "random", "blackout"),
        ("mean_preview", "one_step_handoff"),
        ("all", "held", "timeout"),
        scopes,
    ):
        selected = [
            c
            for c in cases
            if (noise == "all" or (float(c["tau"]) == 0) == (noise == "white"))
            and (mode == "all" or c["mode"] == mode)
            and c["arm"] == arm
            and (outcome == "all" or c["status"] == outcome)
        ]
        rows = [
            r
            for c in selected
            for r in follow_index[c["case_id"]]
            if scopes[scope](r["lag"])
        ]
        summary.append(
            dict(
                noise_selector=noise,
                mode_selector=mode,
                arm=arm,
                outcome_selector=outcome,
                scope=scope,
                cases=len(selected),
                resumes=sum(c["resumes"] for c in selected),
                **counts(rows),
            )
        )
    return cases, pairs, summary, verification


def write(path, rows, fields):
    pilot.write(
        path,
        [
            {k: (str(v).lower() if isinstance(v, bool) else v) for k, v in row.items()}
            for row in rows
        ],
        fields,
    )


def replay(windows, catalogue, baselines, output, map_path):
    output.mkdir(parents=True)
    work = output / "native"
    compiler = native.compile_native(work)
    packet, input_count = write_packet(work, windows)
    evaluation = native.evaluate(work, packet, map_path)
    trace = iter(pilot.read(work / "trace.csv", native.TRACE_FIELDS))
    events, followups, queries = [], [], []
    checks = {
        c["case_id"]: dict.fromkeys(
            (
                "anchor_queries",
                "following_queries",
                "recorded_geometry_matches",
                "transport_matches",
                "actual_handoff_matches",
            ),
            0,
        )
        for c in catalogue
    }
    for window in windows:
        event, local, expanded, check = consume_window(window, trace, baselines)
        events.append(event)
        followups.extend(local)
        queries.extend(expanded)
        for k, v in check.items():
            checks[window["case"]["case_id"]][k] += v
    need(next(trace, None) is None, "Extra native trace rows")
    need(
        len(queries) == len(events) + 3 * len(followups)
        and input_count == len(events) + len(followups),
        "Native query population differs",
    )
    cases, pairs, summary, verification = tables(catalogue, events, followups, checks)
    for name, rows, fields in (
        ("cases.csv", cases, CASE_FIELDS),
        ("pairs.csv", pairs, PAIR_FIELDS),
        ("summary.csv", summary, SUMMARY_FIELDS),
        ("verification.csv", verification, VERIFY_FIELDS),
        ("transitions.csv", events, EVENT_FIELDS),
        ("followups.csv", followups, FOLLOW_FIELDS),
        ("queries.csv", queries, QUERY_FIELDS),
    ):
        write(output / name, rows, fields)
    return (
        dict(
            cases=len(cases),
            pairs=len(pairs),
            resumes=len(events),
            window_inputs=input_count,
            following=len(followups),
            native_queries=len(queries),
            summary_cells=len(summary),
            recorded_geometry_matches=sum(
                c["recorded_geometry_matches"] for c in verification
            ),
            transport_matches=sum(c["transport_matches"] for c in verification),
            actual_handoff_matches=sum(
                c["actual_handoff_matches"] for c in verification
            ),
        ),
        compiler,
        evaluation,
    )


def collect(raw, audit_one, audit_two, output):
    pins = preflight()
    raw, audit_one, audit_two = (Path(p).resolve() for p in (raw, audit_one, audit_two))
    output = build.fresh(output)
    need(
        not any(output.is_relative_to(p) for p in (raw, audit_one, audit_two)),
        "Output must be outside retained inputs",
    )
    need(audit_one != audit_two, "Bind two distinct prior audits")
    need(
        not subprocess.check_output(
            ["git", "status", "--porcelain=v1"], cwd=build.REPO, text=True
        ).strip(),
        "Commit replay implementation before querying",
    )
    old = build.REPO / pins["previous_study"]
    raw_manifest = json.loads((old / "raw-file-manifest.json").read_text())
    repeat = json.loads((old / "repeat-verification.json").read_text())
    need(
        build.sha(old / "raw-file-manifest.json") == pins["raw_manifest_sha256"]
        and build.sha(old / "repeat-verification.json")
        == pins["audit_manifest_sha256"],
        "Retained evidence manifest differs",
    )
    verify_manifest(raw, raw_manifest["sha256"])
    for directory in (audit_one, audit_two):
        verify_manifest(directory, repeat["sha256"])
    need(
        build.sha(raw / "provenance.json") == repeat["raw_provenance_sha256"],
        "Raw provenance differs",
    )
    catalogue = pilot.read(old / "cases.csv")
    windows = collect_windows(raw, catalogue, pins)
    baselines = bind_baselines(audit_one, pins["expected_native_queries"])
    try:
        result, compiler, evaluation = replay(
            windows, catalogue, baselines, output, build.FIXTURES / "obstacles.csv"
        )
        need(
            result["cases"] == pins["expected_cases"]
            and result["pairs"] == pins["expected_pairs"]
            and result["resumes"] == pins["expected_resumes"],
            "Replay denominator differs",
        )
        sources = dict(
            method="Fixed recorded inputs; sequential original native transport and geometry; hash-bound prior full physical/estimator audits, not rerun here",
            implementation_commit=subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=build.REPO, text=True
            ).strip(),
            script_sha256={name: build.sha(build.ROOT / name) for name in SCRIPTS},
            protocol_sha256=pins["protocol_sha256"],
            freeze_sha256=build.sha(STUDY / "freeze-pins.json"),
            input_sha256=pins["input_sha256"],
            raw_provenance_sha256=repeat["raw_provenance_sha256"],
            raw_manifest_sha256=pins["raw_manifest_sha256"],
            prior_audit_manifest_sha256=pins["audit_manifest_sha256"],
            prior_audit_sha256=repeat["sha256"],
            prior_decisions_bound=pins["expected_decisions"],
            prior_native_queries_bound=pins["expected_native_queries"],
            compiler=compiler,
            evaluation=evaluation,
            counts=result,
            output_sha256={
                str(p.relative_to(output)): build.sha(p)
                for p in sorted(output.rglob("*"))
                if p.is_file()
            },
        )
        (output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    except Exception as exc:
        if output.exists():
            (output / "failure.log").write_text(repr(exc) + "\n")
        raise
    print(json.dumps(result), flush=True)
    return sources


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("raw", "audit-one", "audit-two", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    args = parser.parse_args()
    collect(args.raw, args.audit_one, args.audit_two, args.output)


if __name__ == "__main__":
    main()
