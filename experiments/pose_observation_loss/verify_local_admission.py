#!/usr/bin/env python3
"""Independently verify local admission masks, timer runs and native updates."""

import argparse
from collections import Counter, defaultdict
import csv
from decimal import Decimal as D, localcontext
from fractions import Fraction
from itertools import product
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import verify_point_history as previous
import verify_return_admission as admission
import verify_command_sign as domain

core = previous.core
ROOT, REPO, RESULTS = core.ROOT, core.REPO, core.RESULTS
BASE = RESULTS / "fusion-return-point-history"
STUDY = RESULTS / "fusion-return-local-admission"
BASE_MAIN = "b47c3939135776bb698989181cb1e7c599ab2285"
BASE_SHA = "41891a6cf0b16bd70edffa8e06579ccedec73a8e35e2802ce39666adbf30ff7d"
PROOF_SHA = "fe4e5d14009b8310aed1bdda012ed5d0d58113c7a70ca2ec18e8b8f334b5bd64"
META, KEY, GROUP = previous.META, previous.KEY, previous.GROUP
PHASES, BRANCHES, CATEGORIES = previous.PHASES, previous.BRANCHES, previous.CATEGORIES
GATES = admission.GATES
CODE = previous.CODE
HELPERS = {
    **previous.HELPERS,
    "verify_point_history.py": "081021cc95ae156243006b4a4e33aab5427f4562ab24c95428d099501f78cc2e",
    "verify_command_sign.py": "89f6a968726d59d3451fc0815f19814769e4171319640e5a10b7863941e8183f",
    "verify_return_admission.py": "963e35957b525d25b36991a18c1361603087302b9d7e6cd58f766dbefc77c4cc",
}

STATES = ("pass", "fail", "unknown", "not_applicable")
CLASSES = (
    "joint_pass",
    "joint_fail",
    "joint_unknown",
    "guard_pass",
    "guard_fail",
    "guard_unknown",
    "not_applicable",
    "missing",
)
KINDS = (
    "audited",
    "first_clear_audited",
    *("first_clear_" + s for s in STATES),
    "first_clear_nominal_positive",
    "first_clear_nominal_zero",
    "has_raw_clear_nominal_zero",
    "has_post_reset",
    "has_post_known_pass",
)
REFS = tuple(
    f"{p}_{b}_local_{k}_refs" for p, b in product(PHASES, BRANCHES) for k in KINDS
)
HAS = tuple(
    "has_available_local_" + k
    for k in ("raw_clear_nominal_zero", "first_clear_not_ready", "post_reset")
)
INPUTS = tuple(
    dict.fromkeys(
        (
            *admission.BOOLS,
            *admission.FLOATS,
            *domain.SPEEDS,
            *domain.FLAGS,
            "recovery_sample_count",
        )
    )
)
BEFORE = (
    "recovery_evidence",
    "guard_release_evidence",
    "safety_paused",
    "guard_active",
)
FIRST = (
    "branch",
    "nominal_positive",
    "certainty",
    "failed_gates",
    "unknown_gates",
    "timer_before_ticks",
    "timer_after_ticks",
    "timer_action",
    "ready_from_timer",
)
FIELDS = (
    *(f"local_{side}_{c}_slots" for side in ("pre", "post") for c in CLASSES),
    *(
        "local_" + k + "_slots"
        for k in (
            "raw_clear_nominal_zero",
            "raw_clear_not_ready",
            "raw_clear_ready",
            "mean_clear_not_ready",
        )
    ),
    "local_has_post_reset",
    "local_has_post_known_pass",
    *(
        f"local_{b}_{k}"
        for b in BRANCHES
        for k in (
            "max_known_pass_ticks",
            "post_max_known_pass_ticks",
            "max_before_ticks",
            "max_after_ticks",
            "post_max_after_ticks",
        )
    ),
    *("local_first_clear_" + k for k in FIRST),
)
SLOT_FIELDS = (
    *("local_input_" + k for k in INPUTS),
    *("local_prior_" + k for k in BEFORE),
    "local_age_exact_s",
    "local_age_exact_hex",
    "local_last_observation_step",
    "local_branch",
    "local_joint_pending",
    *(f"local_{stage}_gate_{g}" for stage in ("prior", "source") for g in GATES),
    *(
        f"local_{q}_gap_{b}"
        for q in ("clearance", "bounds", "nominal", "release")
        for b in ("lo", "hi")
    ),
    "local_lattice_index",
    "local_speed_cap_bits",
    "local_source_nominal_positive",
    "local_source_zero_reason",
    "local_prior_certainty",
    "local_source_certainty",
    "local_failed_gates",
    "local_unknown_gates",
    *(
        "local_" + k
        for k in (
            "timer_before_ticks",
            "timer_after_ticks",
            "threshold_ticks",
            "ready_from_timer",
            "timer_action",
        )
    ),
    "local_original_reference_joined",
)


def contract(source):
    ready = source[
        source.index("            let recovery_bounds =") : source.index(
            "            if loss_aware && safety_paused && !joint_recovery {"
        )
    ]
    start = source.index("            let mut guard_triggered =")
    guard = core.block(source[start:], "else if guard_active && !joint_pending {")
    joint = core.block(source[start:], "if joint_pending {")
    assert "&& (!averaged_recovery || recovery_sample_count >= 10)" in ready
    return dict(
        window_and_margin=previous.contract(source),
        command=domain.contract(source),
        admission_margin=admission.contract(),
        ready_sha256=hashlib.sha256(ready.encode()).hexdigest(),
        guard_timer_sha256=hashlib.sha256(guard.encode()).hexdigest(),
        joint_timer_sha256=hashlib.sha256(joint.encode()).hexdigest(),
    )


def check_context(old, snapshot, trace=None):
    core.check(old["logged_speed_margin"], snapshot["raw"]["speed_margin"])
    core.check(old["window_count"], len(snapshot["members"]))
    core.check(old["window_count"], int(snapshot["raw"]["recovery_sample_count"]))
    core.check(old["last_clear_step"], snapshot["clear_step"])
    core.check(old["last_clear_reasons"], snapshot["reasons"])
    for k in previous.RAW:
        core.check(old["input_" + k], snapshot["point"][k])
    if trace:
        core.check(trace["sample_count"], len(snapshot["members"]))
        for axis in ("x", "y"):
            assert (
                float(old[f"mean_{axis}_lo"])
                <= float(trace["mean_" + axis])
                <= float(old[f"mean_{axis}_hi"])
            )
            assert (
                float(old[f"raw_{axis}_lo"])
                <= snapshot["point"]["sensed_" + axis + "_center"]
                <= float(old[f"raw_{axis}_hi"])
            )


def decode(raw, n, last_observation_step):
    values = {k: core.truth(raw[k], raw=True) for k in admission.BOOLS}
    values.update({k: float(raw[k]) for k in admission.FLOATS})
    assert all(math.isfinite(values[k]) for k in admission.FLOATS)
    count = int(raw["recovery_sample_count"])
    assert str(count) == raw["recovery_sample_count"] and 0 <= count <= 25
    age = float(n) * float.fromhex("0x1.47ae147ae147bp-6") - float(
        last_observation_step
    ) * float.fromhex("0x1.47ae147ae147bp-6")
    assert format(age, ".6f") == raw["observation_age"]
    values.update(
        recovery_sample_count=count,
        age_exact_s=age,
        age_exact_hex=age.hex(),
        last_observation_step=last_observation_step,
    )
    assert values["stationary_epoch_fusion"] and values["speed_margin"] > 0
    return values


def local_slot(old, e, snapshot, lookup):
    check_context(old, snapshot)
    raw = snapshot["raw"]
    n = int(old["source_step"])
    values = decode(raw, n, snapshot["last_observation_step"])
    before = {
        k: (
            core.truth(snapshot["before_raw"][k], raw=True)
            if k in ("safety_paused", "guard_active")
            else float(snapshot["before_raw"][k])
        )
        for k in BEFORE
    }
    phase = (
        "historical_context" if old["context_phase"] == "before_return" else "available"
    )
    decision = admission.reconstruct_decision(e, n, before, values, phase)
    sign = domain.inspect(raw, lookup)
    original_tests = {g: decision["gate_" + g] for g in GATES}
    tests = dict(original_tests)
    if decision["branch"] == "joint":
        tests["nominal"] = "pass" if sign["source_nominal_positive"] else "fail"
    failed = [g for g in GATES if tests[g] == "fail"]
    unknown = [g for g in GATES if tests[g] == "unknown"]
    certainty = (
        "not_applicable"
        if decision["branch"] not in admission.BRANCHES
        else "fail" if failed else "unknown" if unknown else "pass"
    )
    if certainty in ("pass", "fail"):
        assert decision["ready_from_timer"] == (certainty == "pass")
    result = {"local_input_" + k: raw[k] for k in INPUTS}
    result.update({"local_prior_" + k: snapshot["before_raw"][k] for k in BEFORE})
    result.update(
        local_age_exact_s=values["age_exact_s"],
        local_age_exact_hex=values["age_exact_hex"],
        local_last_observation_step=snapshot["last_observation_step"],
        local_branch=decision["branch"],
        local_joint_pending=decision["joint_pending"],
        local_lattice_index=sign["lattice_index"],
        local_speed_cap_bits=sign["speed_cap_bits"],
        local_source_nominal_positive=sign["source_nominal_positive"],
        local_source_zero_reason=sign["source_zero_reason"],
        local_prior_certainty=decision["certainty"],
        local_source_certainty=certainty,
        local_failed_gates=";".join(failed),
        local_unknown_gates=";".join(unknown),
        local_original_reference_joined=old["context_phase"] != "before_return",
    )
    for g in GATES:
        result["local_prior_gate_" + g] = original_tests[g]
        result["local_source_gate_" + g] = tests[g]
    for q, b in product(("clearance", "bounds", "nominal", "release"), ("lo", "hi")):
        result[f"local_{q}_gap_{b}"] = decision[f"{q}_gap_{b}"]
    for k in (
        "timer_before_ticks",
        "timer_after_ticks",
        "threshold_ticks",
        "ready_from_timer",
        "timer_action",
    ):
        result["local_" + k] = decision[k]
    assert set(result) == set(SLOT_FIELDS)
    return result


def join_original(row, old):
    if not row["local_original_reference_joined"]:
        return
    reference = old[row["event_id"], core.tick(row["source_time_s"])]
    core.check(reference["case_id"], row["case_id"])
    core.check(reference["phase"], "available")
    for g in GATES:
        field = "source_nominal_gate" if g == "nominal" else "gate_" + g
        core.check(reference[field], row["local_source_gate_" + g])
    for field, original in (
        ("source_certainty", "refined_certainty"),
        ("failed_gates", "refined_failed_gates"),
        ("unknown_gates", "refined_unknown_gates"),
        ("branch", "branch"),
        ("source_zero_reason", "source_zero_reason"),
        ("timer_action", "timer_action"),
        ("source_nominal_positive", "source_nominal_positive"),
        ("ready_from_timer", "ready_from_timer"),
        ("timer_before_ticks", "timer_before_ticks"),
        ("timer_after_ticks", "timer_after_ticks"),
    ):
        core.check(reference[original], row["local_" + field])


def ready_runs(d, slots):
    ordered = sorted(slots, key=lambda s: int(s["offset_steps"]))
    assert [int(s["offset_steps"]) for s in ordered] == list(range(-25, 26))
    rows = []
    i = 0
    while i < len(ordered):
        first = ordered[i]
        if not first["local_audited"] or first["local_source_certainty"] != "pass":
            i += 1
            continue
        j = i
        while j + 1 < len(ordered):
            nxt = ordered[j + 1]
            if (
                ordered[j]["local_timer_action"] == "release"
                or not nxt["local_audited"]
                or nxt["local_source_certainty"] != "pass"
                or nxt["local_branch"] != first["local_branch"]
            ):
                break
            j += 1
        last = ordered[j]
        length = j - i + 1
        previous_row = ordered[i - 1] if i else None
        next_row = ordered[j + 1] if j + 1 < len(ordered) else None
        rows.append(
            dict(
                **{k: first[k] for k in (*META, "event_id", "reference_time_s")},
                run=len(rows) + 1,
                branch=first["local_branch"],
                start_offset_steps=int(first["offset_steps"]),
                end_offset_steps=int(last["offset_steps"]),
                start_time_s=float(first["source_time_s"]),
                end_time_s=float(last["source_time_s"]),
                checkpoint_count=length,
                checkpoint_span_s=(length - 1) / 50,
                evidence_added_s=length / 50,
                timer_before_ticks=first["local_timer_before_ticks"],
                timer_after_ticks=last["local_timer_after_ticks"],
                threshold_ticks=first["local_threshold_ticks"],
                observed_release=any(
                    s["local_timer_action"] == "release" for s in ordered[i : j + 1]
                ),
                touches_before_return=any(
                    s["context_phase"] == "before_return" for s in ordered[i : j + 1]
                ),
                left_context_limited=previous_row is None
                or not previous_row["local_audited"],
                right_context_limited=next_row is None or not next_row["local_audited"],
                predecessor_certainty=(
                    previous_row["local_source_certainty"]
                    if previous_row is not None and previous_row["local_audited"]
                    else None
                ),
                successor_certainty=(
                    next_row["local_source_certainty"]
                    if next_row is not None and next_row["local_audited"]
                    else None
                ),
            )
        )
        i = j + 1
    assert sum(row["checkpoint_count"] for row in rows) == sum(
        s["local_audited"] and s["local_source_certainty"] == "pass" for s in ordered
    )
    return rows


def metrics(d, slots, runs):
    result = dict.fromkeys(FIELDS)
    observed = [s for s in slots if s["local_audited"]]
    for side in ("pre", "post"):
        for kind in CLASSES:
            count = 0
            for s in slots:
                if (int(s["offset_steps"]) < 0) != (side == "pre") or int(
                    s["offset_steps"]
                ) == 0:
                    continue
                state = (
                    "missing"
                    if not s["local_audited"]
                    else (
                        "not_applicable"
                        if s["local_branch"] == "not_applicable"
                        else s["local_branch"] + "_" + s["local_source_certainty"]
                    )
                )
                count += state == kind
            result[f"local_{side}_{kind}_slots"] = count
    result["local_raw_clear_nominal_zero_slots"] = sum(
        s["raw_state"] == "clear" and not s["local_source_nominal_positive"]
        for s in observed
    )
    result["local_raw_clear_not_ready_slots"] = sum(
        s["raw_state"] == "clear" and s["local_ready_from_timer"] is False
        for s in observed
    )
    result["local_raw_clear_ready_slots"] = sum(
        s["raw_state"] == "clear" and s["local_ready_from_timer"] is True
        for s in observed
    )
    result["local_mean_clear_not_ready_slots"] = sum(
        s["mean_state"] == "clear" and s["local_ready_from_timer"] is False
        for s in observed
    )
    post = [s for s in observed if int(s["offset_steps"]) > 0]
    result["local_has_post_reset"] = any(
        s["local_timer_action"] == "reset" for s in post
    )
    result["local_has_post_known_pass"] = any(
        s["local_source_certainty"] == "pass" for s in post
    )
    for branch in BRANCHES:
        rr = [s for s in observed if s["local_branch"] == branch]
        pp = [s for s in post if s["local_branch"] == branch]
        result[f"local_{branch}_max_known_pass_ticks"] = max(
            (r["checkpoint_count"] for r in runs if r["branch"] == branch), default=0
        )
        post_lengths = []
        for r in runs:
            if r["branch"] == branch:
                post_lengths.append(
                    sum(
                        1
                        for offset in range(
                            r["start_offset_steps"], r["end_offset_steps"] + 1
                        )
                        if offset > 0
                    )
                )
        result[f"local_{branch}_post_max_known_pass_ticks"] = max(
            post_lengths, default=0
        )
        result[f"local_{branch}_max_before_ticks"] = max(
            (s["local_timer_before_ticks"] for s in rr), default=None
        )
        result[f"local_{branch}_max_after_ticks"] = max(
            (s["local_timer_after_ticks"] for s in rr), default=None
        )
        result[f"local_{branch}_post_max_after_ticks"] = max(
            (s["local_timer_after_ticks"] for s in pp), default=None
        )
    offset = d["history_raw_first_later_clear_offset_steps"]
    if offset != "":
        s = next(s for s in slots if int(s["offset_steps"]) == int(offset))
        assert s["local_audited"] and s["raw_state"] == "clear"
        for k in FIRST:
            result["local_first_clear_" + k] = (
                s["local_source_" + k]
                if k in ("nominal_positive", "certainty")
                else s["local_" + k]
            )
    return result


def raw_inputs(cases, planned, raw, roots, work):
    files = defaultdict(dict)
    queries, query_rows = previous.context_queries(
        [
            {
                **s,
                "source_step": int(s["source_step"]),
                "offset_steps": int(s["offset_steps"]),
                "source_time_s": float(s["source_time_s"]),
            }
            for s in planned
        ]
    )
    for c in cases:
        files[c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"][
            c["scenario"], c["planner"]
        ] = c
    assert set(files) == {(c, n) for c, s in raw.items() for n in s["raw_sha256"]}
    selected_cases = {c for c, n in queries}
    snapshots = {}
    audit = Counter()
    work.mkdir(parents=True)
    with (work / "native-replay-inputs.tsv").open("w") as wire:
        wire.write(
            "case_id\tstep\tpaused\tcorrection\treceived\tprecision\tspeed\tage\tx\ty\tyaw\tlogged_count\trow_ids\n"
        )
        for i, ((cohort, name), by_identity) in enumerate(sorted(files.items()), 1):
            path = Path(roots.get(cohort, raw[cohort]["path"])) / name
            assert core.sha(path) == raw[cohort]["raw_sha256"][name]
            audit["raw_bytes"] += path.stat().st_size
            last = defaultdict(lambda: -1)
            before = {}
            replays = {}
            with path.open() as f:
                header = next(f).rstrip("\r\n").split(",")
                assert len(header) == len(set(header))
                pos = {
                    k: header.index(k)
                    for k in (
                        "scenario",
                        "planner",
                        "time_s",
                        "effective_position_sigma",
                        "effective_yaw_sigma_rad",
                    )
                }
                for line in f:
                    assert '"' not in line
                    cells = line.rstrip("\r\n").split(",")
                    assert len(cells) == len(header)
                    identity = cells[pos["scenario"]], cells[pos["planner"]]
                    assert identity in by_identity
                    c = by_identity[identity]
                    n = core.tick(cells[pos["time_s"]])
                    assert n == last[identity] + 1
                    last[identity] = n
                    if not core.truth(c["selected_timeout"]):
                        continue
                    audit["timeout_grid_rows"] += 1
                    if n == 0:
                        vv = [
                            D(cells[pos[k]])
                            for k in (
                                "effective_position_sigma",
                                "effective_yaw_sigma_rad",
                            )
                        ]
                        assert all(v.is_finite() and v >= 0 for v in vv) and any(
                            v > 0 for v in vv
                        )
                        audit["noisy_branch_histories"] += 1
                    if c["case_id"] not in selected_cases:
                        continue
                    row = dict(zip(header, cells))
                    if n == 0:
                        replays[identity] = previous.previous.previous.previous.Replay(
                            row
                        )
                        audit["replayed_histories"] += 1
                    else:
                        replay = replays[identity]
                        point, members, age, paused = replay.advance(
                            n, before[identity], row
                        )
                        audit["replayed_steps"] += 1
                        row_ids = queries.get((c["case_id"], n), [])
                        values = [
                            c["case_id"],
                            str(n),
                            str(paused).lower(),
                            row["correction"],
                            row["correction_received"],
                            row["precision_activation"],
                            format(float(row["measured_speed"]), ".17e"),
                            format(age, ".17e"),
                            *(
                                format(point["sensed_" + a + "_center"], ".17e")
                                for a in ("x", "y", "yaw")
                            ),
                            row["recovery_sample_count"],
                            ";".join(map(str, row_ids)),
                        ]
                        wire.write("\t".join(values) + "\n")
                        if row_ids:
                            snapshots[c["case_id"], n] = dict(
                                point=point,
                                members=list(members),
                                raw=dict(row),
                                before_raw=dict(before[identity]),
                                last_observation_step=replay.observation_step,
                                clear_step=replay.clear_step,
                                reasons=replay.reasons,
                                pushes=len(replay.admissions),
                            )
                    before[identity] = row
            assert set(last) == set(by_identity)
            for identity, c in by_identity.items():
                assert last[identity] == (
                    9000
                    if core.truth(c["selected_timeout"])
                    else core.tick(c["time_s"])
                )
            audit["raw_files"] += 1
            if i % 18 == 0:
                print(
                    f"Independent local admission contexts read: {i}/108 files",
                    flush=True,
                )
    assert snapshots.keys() == queries.keys()
    audit["unique_context_points"] = len(snapshots)
    return snapshots, query_rows, dict(audit)


def native_queries(mapping, contexts):
    result = []
    for q in mapping:
        snap = contexts[q["row_id"]]
        raw, before = snap["raw"], snap["before_raw"]
        n = core.tick(q["source_time_s"])
        age = decode(raw, n, snap["last_observation_step"])["age_exact_s"]
        result.append(
            dict(
                row_id=q["row_id"],
                event_id=q["event_id"],
                reference_time_s=q["reference_time_s"],
                offset_steps=q["offset_steps"],
                source_step=n,
                prior_paused=before["safety_paused"],
                prior_guard=before["guard_active"],
                correction=raw["correction"],
                received=raw["correction_received"],
                guard_triggered=raw["guard_triggered"],
                before_joint_ticks=admission.counter(
                    float(before["recovery_evidence"]), 50
                ),
                before_guard_ticks=admission.counter(
                    float(before["guard_release_evidence"]), 25
                ),
                age_exact=format(age, ".17e"),
                measured_speed=raw["measured_speed"],
                sample_count=raw["recovery_sample_count"],
                recovery_x=raw["recovery_x"],
                recovery_y=raw["recovery_y"],
                recovery_yaw=raw["recovery_yaw"],
                margin=raw["speed_margin"],
                clearance=raw["recovery_clearance"],
                nominal=raw["nominal_target_speed"],
                release_clearance=raw["release_preview_clearance_lower_bound"],
                release_margin=raw["guard_release_margin"],
            )
        )
    return result


def native_admission(work, rustc, queries):
    source = (REPO / next(iter(CODE))).read_text()
    types = (REPO / "crates/rust_robotics_core/src/types.rs").read_text()
    start = source.index("            let mut guard_triggered =")
    joint = core.block(source[start:], "if joint_pending {")
    guard = core.block(source[start:], "else if guard_active && !joint_pending {")
    ready = source[
        source.index("            let recovery_bounds =") : source.index(
            "            if loss_aware && safety_paused && !joint_recovery {"
        )
    ]
    preview_start = source.index("            let preview_end = Point2D::new(")
    preview = source[
        preview_start : source.index(
            "            // This uses estimated pose", preview_start
        )
    ]
    joint_expression = joint.split("let ready = ", 1)[1].split(";", 1)[0]
    guard_expression = guard.split("let ready = ", 1)[1].split(";", 1)[0]
    harness = (
        "#![allow(dead_code,unused_variables,unused_assignments)]\n#[derive(Clone,Copy)]\n"
        + core.block(types, "pub struct Point2D")
    )
    harness += (
        "\nimpl Point2D {\n" + core.block(types, "pub fn new(x: f64, y: f64)") + "\n}\n"
    )
    harness += (
        next(line for line in source.splitlines() if line.startswith("const DT:"))
        + "\n"
    )
    body = r"""
#[derive(Clone,Copy)]struct Sensed{v:f64}
#[derive(Clone,Copy)]struct Input{
 step:usize,prior_paused:bool,prior_guard:bool,correction:bool,received:bool,triggered:bool,joint_ticks:usize,guard_ticks:usize,
 age:f64,speed:f64,count:usize,x:f64,y:f64,yaw:f64,margin:f64,clearance:f64,nominal:f64,release:f64,release_margin:f64,
}
struct Out{gates:[bool;7],recovery_ready:bool,ready:bool,branch:&'static str,joint:f64,guard:f64,paused:bool,active:bool,resumed:bool,released:bool}
fn ticks(n:usize)->f64{let mut value=0.;for _ in 0..n{value+=DT;}value}
fn evaluate(input:Input)->Out{
 let sensed=Sensed{v:input.speed};let observation_age=input.age;
 let recovery_x=input.x;let recovery_y=input.y;let recovery_yaw=input.yaw;
 let recovery_margin=input.margin;let recovery_clearance=input.clearance;let recovery_sample_count=input.count;
 let averaged_recovery=true;let consistent_restart=false;let consistency_ready=false;
 let nominal_target_speed=input.nominal;let release_preview_clearance_lower_bound=input.release;let guard_release_margin=input.release_margin;
 let mut safety_paused=input.prior_paused || (input.correction && !input.received);
 let joint_pending=safety_paused;
 let branch=if joint_pending{"joint"}else if input.prior_guard{"guard"}else{"not_applicable"};
 let mut guard_active=input.prior_guard || input.triggered;
 let mut guard_release_evidence=if input.triggered{0.}else{ticks(input.guard_ticks)};
 let mut recovery_evidence=if joint_pending && !input.prior_paused{0.}else{ticks(input.joint_ticks)};
 let mut guard_released=false;let mut resume_count=0;let mut recovery_limit_until=0.;let observation_time=input.step as f64*DT;
 __PREVIEW__
 __READY__
 let center_ready=if joint_pending{__JOINT_EXPRESSION__}else if input.prior_guard{__GUARD_EXPRESSION__}else{false};
 let gates=[observation_age<=1.5,sensed.v.abs()<=0.02,recovery_sample_count>=10,recovery_clearance>recovery_margin,recovery_bounds,nominal_target_speed>0.,release_preview_clearance_lower_bound>guard_release_margin];
 if branch=="guard" {__GUARD_BODY__}
 __JOINT__
 Out{gates,recovery_ready,ready:center_ready,branch,joint:recovery_evidence,guard:guard_release_evidence,paused:safety_paused,active:guard_active,resumed,released:guard_released}
}
fn main(){
 let args:Vec<_>=std::env::args().collect();
 println!("row_id,gate_age,gate_stationary,gate_samples,gate_clearance,gate_bounds,gate_nominal,gate_release,recovery_ready,ready,branch,recovery_evidence,guard_release_evidence,safety_paused,guard_active,resumed,guard_released");
 for line in std::fs::read_to_string(&args[1]).unwrap().lines().skip(1){
  let f:Vec<_>=line.split('\t').collect();
  let input=Input{step:f[4].parse().unwrap(),prior_paused:f[5].parse().unwrap(),prior_guard:f[6].parse().unwrap(),correction:f[7].parse().unwrap(),received:f[8].parse().unwrap(),triggered:f[9].parse().unwrap(),joint_ticks:f[10].parse().unwrap(),guard_ticks:f[11].parse().unwrap(),
   age:f[12].parse().unwrap(),speed:f[13].parse().unwrap(),count:f[14].parse().unwrap(),x:f[15].parse().unwrap(),y:f[16].parse().unwrap(),yaw:f[17].parse().unwrap(),margin:f[18].parse().unwrap(),clearance:f[19].parse().unwrap(),nominal:f[20].parse().unwrap(),release:f[21].parse().unwrap(),release_margin:f[22].parse().unwrap()};
  let o=evaluate(input);
  println!("{},{},{},{},{},{},{},{},{},{},{},{:.17e},{:.17e},{},{},{},{}",f[0],o.gates[0],o.gates[1],o.gates[2],o.gates[3],o.gates[4],o.gates[5],o.gates[6],o.recovery_ready,o.ready,o.branch,o.joint,o.guard,o.paused,o.active,o.resumed,o.released);
 }
}
#[cfg(test)]mod tests{
 use super::*;
 fn good()->Input{Input{step:10,prior_paused:true,prior_guard:true,correction:false,received:false,triggered:false,joint_ticks:0,guard_ticks:0,age:0.2,speed:0.,count:10,x:10.,y:10.,yaw:0.,margin:1.,clearance:2.,nominal:0.4,release:2.,release_margin:1.05}}
 #[test]fn joint_threshold(){let mut i=good();i.joint_ticks=49;let o=evaluate(i);assert!(o.resumed && o.released && !o.paused && !o.active);assert_eq!(o.joint,0.);}
 #[test]fn joint_accumulates_before_threshold(){let mut i=good();i.joint_ticks=48;let o=evaluate(i);assert!(!o.resumed && o.paused && o.ready);assert_eq!(o.joint,ticks(49));}
 #[test]fn guard_threshold(){let mut i=good();i.prior_paused=false;i.guard_ticks=24;let o=evaluate(i);assert!(o.released && !o.active && !o.resumed);assert_eq!(o.guard,0.);}
 #[test]fn guard_accumulates_before_threshold(){let mut i=good();i.prior_paused=false;i.guard_ticks=23;let o=evaluate(i);assert!(o.active && !o.released && o.ready);assert_eq!(o.guard,ticks(24));}
 #[test]fn failure_and_new_pause_reset(){let mut i=good();i.nominal=0.;i.joint_ticks=8;let o=evaluate(i);assert!(!o.ready);assert_eq!(o.joint,0.);i=good();i.prior_paused=false;i.correction=true;i.count=1;i.joint_ticks=49;let o=evaluate(i);assert!(!o.ready && o.paused);assert_eq!(o.joint,0.);}
 #[test]fn inclusive_bounds(){let mut i=good();i.x=1.;i.y=1.;let o=evaluate(i);assert!(o.gates[4]);i.x=48.4;assert!(evaluate(i).gates[4]);i.x=48.5;assert!(!evaluate(i).gates[4]);}
 #[test]fn strict_clearances(){let mut i=good();i.clearance=i.margin;assert!(!evaluate(i).gates[3]);i=good();i.release=i.release_margin;assert!(!evaluate(i).gates[6]);}
 #[test]fn inclusive_age_speed_and_sample_thresholds(){let mut i=good();i.age=1.5;i.speed=0.02;assert!(evaluate(i).recovery_ready);i.count=9;assert!(!evaluate(i).recovery_ready);i=good();i.age=f64::from_bits(1.5_f64.to_bits()+1);assert!(!evaluate(i).gates[0]);i=good();i.speed=f64::from_bits(0.02_f64.to_bits()+1);assert!(!evaluate(i).gates[1]);}
}
"""
    replacements = {
        "__PREVIEW__": preview,
        "__READY__": ready,
        "__JOINT_EXPRESSION__": joint_expression,
        "__GUARD_EXPRESSION__": guard_expression,
        "__GUARD_BODY__": guard[guard.index("{") + 1 : -1],
        "__JOINT__": joint,
    }
    for marker, text in replacements.items():
        body = body.replace(marker, text)
    harness += body
    work.mkdir(parents=True)
    script = work / "admission.rs"
    script.write_text(harness)
    for flags, name in ((["--test"], "admission-tests"), ([], "admission-trace")):
        subprocess.run(
            [
                str(rustc),
                "--edition=2021",
                "-O",
                *flags,
                str(script),
                "-o",
                str(work / name),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
    tests = subprocess.run(
        [str(work / "admission-tests")], check=True, capture_output=True, text=True
    ).stdout
    assert "8 passed; 0 failed" in tests
    (work / "admission-tests.txt").write_text(tests)
    query = work / "native-admission-inputs.tsv"
    with query.open("w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=queries[0], delimiter="\t", lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(queries)
    text = subprocess.run(
        [str(work / "admission-trace"), str(query)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    trace = work / "native-admission.csv"
    trace.write_text(text)
    rows = core.read(trace)
    assert [int(row["row_id"]) for row in rows] == list(range(1, len(queries) + 1))
    return {int(row["row_id"]): row for row in rows}, dict(
        harness_sha256=core.sha(script),
        inputs_sha256=core.sha(query),
        trace_sha256=core.sha(trace),
        native_tests=8,
        traced_context_queries=len(rows),
        method="unchanged original recovery bounds/ready and joint/guard timer fragments; exact clock/speed, rounded logged scalars and source DT counter reconstruction; observed guard-entry flag is an input, no closed-loop replay",
    )


def check_native(row, trace, raw):
    assert trace["branch"] == row["local_branch"]
    for gate in GATES:
        state = row["local_source_gate_" + gate]
        if state in ("pass", "fail"):
            assert core.truth(trace["gate_" + gate], raw=True) == (state == "pass"), (
                gate,
                state,
                trace["gate_" + gate],
            )
    matches = True
    if row["local_ready_from_timer"] is not None:
        matches &= core.truth(trace["ready"], raw=True) == row["local_ready_from_timer"]
    for field in ("recovery_evidence", "guard_release_evidence"):
        matches &= format(float(trace[field]), ".6f") == raw[field]
    for field in ("safety_paused", "guard_active", "resumed", "guard_released"):
        matches &= core.truth(trace[field], raw=True) == core.truth(
            raw[field], raw=True
        )
    if row["local_source_certainty"] != "unknown":
        assert matches, (row["event_id"], row["source_time_s"], trace, raw)
    return int(row["local_source_certainty"] == "unknown" and not matches)


def anchor(e, dd, ne, nr):
    parent = int(e["parent_rp_records"])
    measures = dict(
        census_case_weight=Fraction(1, ne),
        received_case_weight=(
            Fraction(1, nr) if core.truth(e["received"]) else Fraction()
        ),
        rp_case_mass=Fraction(int(e["rp_records"]), parent) if parent else Fraction(),
        rp_different_preview_case_mass=(
            Fraction(int(e["rp_different_preview_records"]), parent)
            if parent
            else Fraction()
        ),
    )
    for k, v in measures.items():
        core.check(e[k], v)
    out = dict(e, **measures, **dict.fromkeys((*REFS, *HAS)))
    if not core.truth(e["returned"] or "False"):
        assert not dd
        return out
    assert all(all(d[k] == e[k] for k in META) for d in dd)
    for p, b in product(PHASES, BRANCHES):
        selected = [
            d for d in dd if d["local_audited"] and d["phase"] == p and d["branch"] == b
        ]
        counts = {k: 0 for k in KINDS}
        counts["audited"] = len(selected)
        first = [d for d in selected if d["local_first_clear_certainty"] is not None]
        counts["first_clear_audited"] = len(first)
        for s in STATES:
            counts["first_clear_" + s] = sum(
                d["local_first_clear_certainty"] == s for d in first
            )
        counts["first_clear_nominal_positive"] = sum(
            d["local_first_clear_nominal_positive"] for d in first
        )
        counts["first_clear_nominal_zero"] = sum(
            not d["local_first_clear_nominal_positive"] for d in first
        )
        counts["has_raw_clear_nominal_zero"] = sum(
            bool(d["local_raw_clear_nominal_zero_slots"]) for d in selected
        )
        counts["has_post_reset"] = sum(d["local_has_post_reset"] for d in selected)
        counts["has_post_known_pass"] = sum(
            d["local_has_post_known_pass"] for d in selected
        )
        out.update({f"{p}_{b}_local_{k}_refs": value for k, value in counts.items()})
    available = [d for d in dd if d["phase"] == "available" and d["local_audited"]]
    out["has_available_local_raw_clear_nominal_zero"] = any(
        d["local_raw_clear_nominal_zero_slots"] for d in available
    )
    out["has_available_local_first_clear_not_ready"] = any(
        d["local_first_clear_ready_from_timer"] is False for d in available
    )
    out["has_available_local_post_reset"] = any(
        d["local_has_post_reset"] for d in available
    )
    return out


def closure(rows):
    checks = 0
    for key, r in rows.items():
        for i, whole, parts in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", CATEGORIES),
        ):
            if key[i] == whole:
                for f in (
                    *REFS,
                    *(h + s for h in HAS for s in ("_events", "_case_mass")),
                ):
                    assert r[f] == sum(
                        rows[key[:i] + (p,) + key[i + 1 :]][f] for p in parts
                    )
                    checks += 1
        for p, b in product(PHASES, BRANCHES):
            prefix = f"{p}_{b}_local_"
            assert r[prefix + "first_clear_audited_refs"] == sum(
                r[prefix + "first_clear_" + s + "_refs"] for s in STATES
            )
            assert (
                r[prefix + "first_clear_audited_refs"]
                == r[prefix + "first_clear_nominal_positive_refs"]
                + r[prefix + "first_clear_nominal_zero_refs"]
            )
    return checks


def compare(path, rows, keys):
    expected = {tuple(str(r[k]) for k in keys): r for r in rows}
    actual = core.index(core.read(path), keys)
    assert len(expected) == len(rows) and actual.keys() == expected.keys()
    for key, row in actual.items():
        assert set(row) == set(expected[key])
        for field, value in expected[key].items():
            if isinstance(value, (D, float)):
                assert math.isclose(
                    float(row[field]), float(value), rel_tol=1e-10, abs_tol=1e-9
                ), (key, field, row[field], value)
                core.comparisons += 1
            else:
                core.check(row[field], value)


def summaries(old, anchors):
    result = {}
    for r in old:
        key = tuple(r[k] for k in GROUP)
        p, n, arm, mode, cat = key
        aa = [
            a
            for a in anchors
            if (a["partition"], a["noise"], a["variant"]) == (p, n, arm)
            and (mode == "all_modes" or a["mode"] == mode)
            and (cat == "all" or a["anchor_category"] == cat)
        ]
        rr = [a for a in aa if core.truth(a["returned"] or "False")]
        mass = lambda rows: sum((a["census_case_weight"] for a in rows), Fraction())
        out = dict(r)
        for field in REFS:
            out[field] = sum(a[field] or 0 for a in aa)
        for h in HAS:
            selected = [a for a in aa if a[h]]
            out.update(
                {
                    h + "_events": len(selected),
                    h + "_case_mass": mass(selected),
                    h + "_fraction": Fraction(len(selected), len(rr)) if rr else None,
                    h
                    + "_case_mass_fraction": mass(selected) / mass(rr) if rr else None,
                }
            )
        core.check(r["census_case_mass"], mass(aa))
        assert key not in result
        result[key] = out
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for n in ("results", "figures", "work", "output"):
        p.add_argument("--" + n, type=Path, required=True)
    p.add_argument("--rustc", type=Path, default=shutil.which("rustc"))
    p.add_argument("--raw-roots", type=Path)
    args = p.parse_args()
    if not __debug__ or args.work.exists() or args.output.exists() or not args.rustc:
        p.error("Use new work/output paths, provide rustc and run without -O")
    prior = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        core.sha(BASE / "sources.json") == BASE_SHA
        and core.sha(BASE / "independent-verification.json") == PROOF_SHA
    )
    freeze = json.loads((STUDY / "freeze-pins.json").read_text())
    assert freeze == dict(
        base_main="b47c3939135776bb698989181cb1e7c599ab2285",
        sources_sha256=BASE_SHA,
        proof_sha256=PROOF_SHA,
        verifier_helper_sha256=HELPERS["verify_point_history.py"],
        command_verifier_sha256=HELPERS["verify_command_sign.py"],
        admission_verifier_sha256=HELPERS["verify_return_admission.py"],
    )
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        **{
            "fusion-return-point-history/" + n: h
            for n, h in prior["output_sha256"].items()
        },
        "fusion-return-point-history/sources.json": BASE_SHA,
        "fusion-return-point-history/independent-verification.json": PROOF_SHA,
        "fusion-return-point-history/protocol.md": prior["analysis"]["protocol_sha256"],
        "fusion-return-point-history/freeze-pins.json": prior["analysis"][
            "freeze_sha256"
        ],
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_point_history.py": prior["analysis"]["script_sha256"],
    }
    assert (
        source["analysis"]["verifier_helpers_sha256"] == HELPERS
        and source["analysis"]["code_sha256"] == CODE
    )
    assert (
        source["raw"] == prior["raw"]
        and source["map_sha256"] == prior["map_sha256"]
        and source["inherited_point_history_proof_sha256"] == PROOF_SHA
        and source["inherited_validation"] == prior["inherited_validation"]
    )
    assert source["schema"] == dict(
        half_unit=5e-10,
        elemental_allowance=1e-12,
        gates=list(GATES),
        states=list(STATES),
        classes=list(CLASSES),
        phases=list(PHASES),
        branches=list(BRANCHES),
        retrospective=True,
        inherited="all prior fields/point gaps/outcomes/weights; same 25 histories/slots; nominal source domain independent of timers; no new rollout or follow-up",
    )
    for n, h in source["input_sha256"].items():
        assert core.sha(RESULTS / n) == h, n
    for n, h in {**source["analysis"]["helpers_sha256"], **HELPERS}.items():
        assert core.sha(ROOT / n) == h, n
    for n, h in CODE.items():
        assert core.sha(REPO / n) == h, n
    assert source["analysis"]["source_contract"] == contract(
        (REPO / next(iter(core.CODE))).read_text()
    )
    for f, path in (
        ("script_sha256", ROOT / "analyze_local_admission.py"),
        ("protocol_sha256", STUDY / "protocol.md"),
        ("freeze_sha256", STUDY / "freeze-pins.json"),
    ):
        assert source["analysis"][f] == core.sha(path)
    assert set(source["output_sha256"]) == {
        "local-decisions.csv",
        "local-slots.csv",
        "ready-runs.csv",
        "anchor-local.csv",
        "local-summary.csv",
    }
    for n, h in source["output_sha256"].items():
        assert core.sha(args.results / n) == h, n
    compiler = subprocess.run(
        [str(args.rustc), "--version"], check=True, capture_output=True, text=True
    ).stdout.strip()
    assert compiler == proof["native"]["rustc"]
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    for cohort, spec in source["raw"].items():
        path = Path(roots.get(cohort, spec["path"])) / "provenance.json"
        assert core.sha(path) == spec["provenance_sha256"]
        record = json.loads(path.read_text())
        assert (
            record["rustc"] == compiler
            and record["inputs_sha256"]["obstacles.csv"] == source["map_sha256"]
        )
        assert set(record["inputs_by_variant_sha256"]) == {"epoch", "curved"} and all(
            v["obstacles.csv"] == source["map_sha256"]
            for v in record["inputs_by_variant_sha256"].values()
        )
    cases, original = core.original_inputs()
    events = core.read(BASE / "anchor-history.csv")
    assert {e["event_id"] for e in events} == original.keys()
    for e in events:
        for k in (
            *META,
            "source_event_id",
            "event_kind",
            "event_time_s",
            "scheduled",
            "received",
            "precision_activation",
            "horizon",
            "parent_rp_records",
            "rp_records",
            "rp_different_preview_records",
        ):
            core.check(e[k], original[e["event_id"]][k])
    original_checks = core.comparisons
    old = core.read(BASE / "history-decisions.csv")
    selected = [d for d in old if d["prior_certainty"] == "unknown"]
    assert len(selected) == 25 and all(
        d["phase"] == "available" and d["branch"] == "joint" for d in selected
    )
    planned = core.read(BASE / "history-slots.csv")
    assert len(planned) == 1275
    snapshots, query_rows, audit = raw_inputs(
        cases, planned, source["raw"], roots, args.work
    )
    queries = [
        dict(
            case_id=q["case_id"], event_id=q["event_id"], time_s=str(q["source_time_s"])
        )
        for q in query_rows
    ]
    traces, native_proof = previous.previous.previous.previous.native(
        args.work, args.rustc, queries
    )
    assert native_proof["replayed_steps"] == audit["replayed_steps"]
    native_proof["traced_context_queries"] = native_proof.pop("traced_references")
    context_map = args.work / "native-contexts.csv"
    with context_map.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=query_rows[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(query_rows)
    native_proof["context_map_sha256"] = core.sha(context_map)
    contexts = {
        q["row_id"]: snapshots[q["case_id"], core.tick(q["source_time_s"])]
        for q in query_rows
    }
    lattice, command_proof = domain.native(args.work / "command-domain", args.rustc)
    native_proof["command"] = command_proof
    lookup = {row["speed_free_distance"]: row for row in lattice}
    gate_queries = native_queries(query_rows, contexts)
    gate_traces, gate_proof = native_admission(
        args.work / "admission", args.rustc, gate_queries
    )
    native_proof["admission"] = gate_proof
    query_ids = {
        (q["event_id"], q["reference_time_s"], q["offset_steps"]): q["row_id"]
        for q in query_rows
    }
    original_decisions = {(d["event_id"], core.tick(d["time_s"])): d for d in old}
    slots = []
    by_reference = defaultdict(list)
    unknown_center_mismatches = 0
    for s in planned:
        audited = s["slot_status"] == "observed"
        row = dict(s, local_audited=audited, **dict.fromkeys(SLOT_FIELDS))
        if audited:
            context_id = query_ids[
                s["event_id"], s["reference_time_s"], int(s["offset_steps"])
            ]
            snap = contexts[context_id]
            check_context(s, snap, traces[context_id])
            row.update(
                local_slot(
                    s, {e["event_id"]: e for e in events}[s["event_id"]], snap, lookup
                )
            )
            join_original(row, original_decisions)
            unknown_center_mismatches += check_native(
                row, gate_traces[context_id], snap["raw"]
            )
        slots.append(row)
        by_reference[s["event_id"], s["reference_time_s"]].append(row)
    gate_proof["unknown_center_mismatches"] = unknown_center_mismatches
    selected_map = {(d["event_id"], d["time_s"]): d for d in selected}
    decisions = []
    runs = []
    for d in old:
        audited = (d["event_id"], d["time_s"]) in selected_map
        out = dict(d, local_audited=audited, **dict.fromkeys(FIELDS))
        if audited:
            rr = ready_runs(d, by_reference[d["event_id"], d["time_s"]])
            out.update(metrics(d, by_reference[d["event_id"], d["time_s"]], rr))
            runs.extend(rr)
        decisions.append(out)
    db = defaultdict(list)
    for d in decisions:
        db[d["event_id"]].append(d)
    ne = Counter(e["case_id"] for e in original.values())
    nr = Counter(e["case_id"] for e in original.values() if core.truth(e["received"]))
    anchors = [
        anchor(e, db[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]]) for e in events
    ]
    summary = summaries(core.read(BASE / "history-summary.csv"), anchors)
    for name, rows, keys in (
        ("local-decisions", decisions, ("event_id", "time_s")),
        ("local-slots", slots, ("event_id", "reference_time_s", "offset_steps")),
        ("ready-runs", runs, ("event_id", "reference_time_s", "run")),
        ("anchor-local", anchors, ("event_id",)),
        ("local-summary", list(summary.values()), GROUP),
    ):
        compare(args.results / (name + ".csv"), rows, keys)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(core.truth(e["returned"] or "False") for e in events),
        decision_references=len(decisions),
        selected_references=len(selected),
        inherited_zero_probes=1096,
        planned_slots=len(slots),
        context_queries=sum(s["slot_status"] == "observed" for s in slots),
        missing_slots=sum(s["slot_status"] != "observed" for s in slots),
        known_pass_runs=len(runs),
        original_reference_joins=sum(
            s["local_original_reference_joined"] is True for s in slots
        ),
        summary_rows=len(summary),
        new_additive_closure_comparisons=closure(summary),
    )
    assert audit == source["audit"], (audit, source["audit"])
    assert (
        sum(a["census_case_weight"] for a in anchors)
        == sum(a["received_case_weight"] for a in anchors)
        == 570
    )
    assert (
        sum(a["rp_case_mass"] for a in anchors) == 157
        and sum(int(a["rp_records"]) for a in anchors) == 10995
    )
    plots = json.loads((args.figures / "plot-sources.json").read_text())
    assert plots["script_sha256"] == core.sha(ROOT / "plot_local_admission.py")
    assert plots["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in (
            "sources.json",
            "anchor-local.csv",
            "local-summary.csv",
            "local-slots.csv",
        )
    }
    assert set(plots["output_sha256"]) == {
        "local-admission.svg",
        "plot-values.csv",
        "plot-return-values.csv",
        "plot-local.csv",
    }
    for n, h in plots["output_sha256"].items():
        assert core.sha(args.figures / n) == h
    compare(
        args.figures / "plot-values.csv",
        [r for r in summary.values() if r["mode"] != "all_modes"],
        GROUP,
    )
    compare(
        args.figures / "plot-return-values.csv",
        [a for a in anchors if core.truth(a["returned"] or "False")],
        ("event_id",),
    )
    compare(
        args.figures / "plot-local.csv",
        slots,
        ("event_id", "reference_time_s", "offset_steps"),
    )
    evidence = dict(
        **audit,
        comparisons=core.comparisons - original_checks,
        original_join_comparisons=original_checks,
        plotted_summary_rows=252,
        plotted_history_slots=len(slots),
        plotted_return_rows=137,
        decimal_precision=60,
        native=native_proof,
        inherited_point_history_proof_sha256=PROOF_SHA,
        script_sha256=core.sha(Path(__file__)),
        sources_sha256=core.sha(args.results / "sources.json"),
        plot_sources_sha256=core.sha(args.figures / "plot-sources.json"),
        method="independent Decimal source-gate refinement and timer inference, original joins, known-pass segmentation and accounting; pinned window/command/admission helpers with native cap/ready/timer traces; same historical context, no new rollout or follow-up",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
