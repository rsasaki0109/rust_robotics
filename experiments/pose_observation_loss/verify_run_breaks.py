#!/usr/bin/env python3
"""Independently verify segment endings and distinct recorded reset edges."""

import argparse
from collections import Counter, defaultdict
import csv
from fractions import Fraction
import json
import math
from pathlib import Path

import verify_local_admission as inherited

core = inherited.core
ROOT, REPO, RESULTS = core.ROOT, core.REPO, core.RESULTS
BASE = RESULTS / "fusion-return-local-admission"
STUDY = RESULTS / "fusion-return-run-breaks"
BASE_MAIN = "9d4ca904a64d1ebee693aa3bb79e8764deb0f7f7"
BASE_SHA = "855f9efd9a43a20251c58d3db043c8947c920c54d82aec1ae42cb9b1777a175c"
PROOF_SHA = "884fb4bca980ec2baf35fd7df28d6a51a5280ec01a9ff671a38f46bc4ef62770"
META = (
    "case_id",
    "family_id",
    "cohort",
    "variant",
    "mode",
    "seed",
    "scenario",
    "planner",
    "partition",
    "accounting_partition",
    "noise",
)
GROUP = ("partition", "noise", "variant", "mode", "category_selector")
GATES = ("age", "stationary", "samples", "clearance", "bounds", "nominal", "release")
CATEGORIES = (
    "ttl_ended",
    "age_ended",
    "returned_censored",
    "no_return",
    "unobserved_ttl_limit",
    "no_observed_loss",
)
KINDS = ("nominal_only", "nominal_with_other", "other_only")
STATUSES = (
    "gate_failure",
    "right_context_limit",
    "missing_context",
    "unknown_boundary",
    "branch_change",
    "recorded_release",
)
METRICS = (
    "audited_references",
    "runs",
    "gate_failure_ends",
    *(k + "_ends" for k in KINDS),
    "post_gate_failure_ends",
    *("post_" + k + "_ends" for k in KINDS),
    *(k + "_ends" for k in STATUSES[1:]),
    "lost_ticks",
    "post_lost_ticks",
)
HAS = tuple("has_available_break_post_" + k for k in KINDS)
REF_BASE = (
    *META,
    "event_id",
    "reference_time_s",
    "phase",
    "branch",
    "raw_distance_direction",
    "probe_origin",
    "timer_action",
    "local_joint_max_known_pass_ticks",
    "local_joint_post_max_known_pass_ticks",
    "history_raw_post_missing_slots",
)
ANCHOR_INPUT = (
    *META,
    "event_id",
    "anchor_category",
    "received",
    "returned",
    "parent_rp_records",
    "rp_records",
    "rp_different_preview_records",
    "census_case_weight",
    "received_case_weight",
    "rp_case_mass",
    "rp_different_preview_case_mass",
)
SUMMARY_INPUT = (
    *GROUP,
    "planned_cases",
    "timeout_cases",
    "held_cases",
    "census_cases",
    "returned_cases",
    "census_events",
    "census_case_mass",
    "received_case_mass",
    "rp_records",
    "rp_different_preview_records",
    "rp_case_mass",
    "rp_different_preview_case_mass",
    "returned_events",
    "returned_case_mass",
)
WEIGHT_FIELDS = (
    "census_case_weight",
    "received_case_weight",
    "rp_case_mass",
    "rp_different_preview_case_mass",
)
ANCHOR_BASE = (*ANCHOR_INPUT, *("exact_" + f for f in WEIGHT_FIELDS))
SUMMARY_BASE = (
    *SUMMARY_INPUT,
    *(
        "exact_" + f
        for f in (
            "census_case_mass",
            "received_case_mass",
            "rp_case_mass",
            "rp_different_preview_case_mass",
            "returned_case_mass",
        )
    ),
)
EXTRA = (
    "end_status",
    "failure_kind",
    "last_source_step",
    "native_last_row_id",
    "next_offset_steps",
    "next_source_step",
    "next_time_s",
    "next_side",
    "next_slot_status",
    "next_context_phase",
    "next_branch",
    "next_certainty",
    "next_failed_gates",
    "next_unknown_gates",
    *("next_gate_" + g for g in GATES),
    "next_nominal_positive",
    "next_zero_reason",
    "next_timer_before_ticks",
    "next_timer_after_ticks",
    "next_timer_action",
    "next_ready_from_timer",
    "native_next_row_id",
    "lost_ticks",
    "lost_evidence_s",
    "carried_ticks",
    "segment_ticks",
    "decomposition",
)
EDGE_FIELDS = (
    *META,
    "edge_id",
    "source_step",
    "source_time_s",
    "event_ids",
    "reference_keys",
    "member_native_rows",
    "multiplicity",
    "branch",
    "failed_gates",
    "unknown_gates",
    *("gate_" + g for g in GATES),
    "nominal_positive",
    "zero_reason",
    "failure_kind",
    "failed_gate_count",
    "timer_before_ticks",
    "timer_after_ticks",
    "lost_ticks",
    "lost_evidence_s",
)
HELPERS = {
    **inherited.HELPERS,
    "verify_local_admission.py": "0ba30545e184af3b41802e69b597826af34fec816573ff302226c3ae71c7a7da",
}


def check(actual, expected):
    if isinstance(expected, Fraction):
        core.comparisons += 1
        assert actual == str(expected), (actual, expected)
    elif isinstance(expected, float):
        core.comparisons += 1
        assert math.isclose(float(actual), expected, rel_tol=1e-10, abs_tol=1e-12)
    else:
        core.check(actual, expected)


def inherited_number(actual, expected):
    # Preserve old decimal strings; derive new exact weights from integer counts.
    if isinstance(expected, Fraction):
        core.comparisons += 1
        assert math.isclose(
            float(Fraction(actual)), float(expected), rel_tol=1e-10, abs_tol=2e-12
        )
    else:
        check(actual, expected)


def read(path):
    with path.open(newline="") as f:
        reader = csv.DictReader(f, delimiter="\t" if path.suffix == ".tsv" else ",")
        assert len(reader.fieldnames) == len(set(reader.fieldnames))
        rows = list(reader)
    assert all(None not in r and None not in r.values() for r in rows)
    return rows


def counter(value, limit):
    n = int(round(float(value) / 0.02))
    assert 0 <= n < limit and math.isclose(
        float(value), n * 0.02, rel_tol=0, abs_tol=5e-7
    )
    return n


def native_slot(s, q, trace):
    assert s["local_audited"] == "True"
    for f in ("event_id", "reference_time_s", "offset_steps", "source_step"):
        check(q[f], s[f])
    check(trace["row_id"], q["row_id"])
    expected = dict(
        prior_paused=s["local_prior_safety_paused"],
        prior_guard=s["local_prior_guard_active"],
        correction=s["local_input_correction"],
        received=s["local_input_correction_received"],
        guard_triggered=s["local_input_guard_triggered"],
        before_joint_ticks=counter(s["local_prior_recovery_evidence"], 50),
        before_guard_ticks=counter(s["local_prior_guard_release_evidence"], 25),
        measured_speed=s["local_input_measured_speed"],
        sample_count=s["local_input_recovery_sample_count"],
        recovery_x=s["local_input_recovery_x"],
        recovery_y=s["local_input_recovery_y"],
        recovery_yaw=s["local_input_recovery_yaw"],
        margin=s["local_input_speed_margin"],
        clearance=s["local_input_recovery_clearance"],
        nominal=s["local_input_nominal_target_speed"],
        release_clearance=s["local_input_release_preview_clearance_lower_bound"],
        release_margin=s["local_input_guard_release_margin"],
    )
    for f, value in expected.items():
        check(q[f], value)
    assert float(q["age_exact"]) == float.fromhex(s["local_age_exact_hex"])
    check(trace["branch"], s["local_branch"])
    for g in GATES:
        predicate = s["local_source_gate_" + g]
        if predicate in ("pass", "fail"):
            assert core.truth(trace["gate_" + g], raw=True) == (predicate == "pass")
    if s["local_source_certainty"] == "unknown":
        return
    if s["local_ready_from_timer"]:
        assert core.truth(trace["ready"], raw=True) == core.truth(
            s["local_ready_from_timer"]
        )
    for f in ("recovery_evidence", "guard_release_evidence"):
        check(s["local_input_" + f], format(float(trace[f]), ".6f"))
    for f in ("safety_paused", "guard_active", "resumed", "guard_released"):
        check(trace[f], s["local_input_" + f])


def native_links(slots, mapping, queries, traces):
    observed = {
        (s["event_id"], s["reference_time_s"], s["offset_steps"]): s
        for s in slots
        if s["local_audited"] == "True"
    }
    by_query = core.index(queries, ("row_id",))
    by_trace = core.index(traces, ("row_id",))
    assert len(mapping) == len(observed) == len(by_query) == len(by_trace)
    result = {}
    for row in mapping:
        k = row["event_id"], row["reference_time_s"], row["offset_steps"]
        assert k not in result
        s = observed[k]
        check(row["case_id"], s["case_id"])
        check(row["source_time_s"], s["source_time_s"])
        native_slot(s, by_query[row["row_id"],], by_trace[row["row_id"],])
        result[k] = int(row["row_id"])
    assert set(result) == set(observed)
    return result, {k[0]: v for k, v in by_trace.items()}


def scan(slots):
    ordered = sorted(slots, key=lambda s: int(s["offset_steps"]))
    assert [int(s["offset_steps"]) for s in ordered] == list(range(-25, 26))
    result = []
    position = 0
    while position < len(ordered):
        start = ordered[position]
        if (
            start["local_audited"] != "True"
            or start["local_source_certainty"] != "pass"
        ):
            position += 1
            continue
        stop = position + 1
        while stop < len(ordered):
            candidate, previous = ordered[stop], ordered[stop - 1]
            if (
                previous["local_timer_action"] == "release"
                or candidate["local_audited"] != "True"
                or candidate["local_source_certainty"] != "pass"
                or candidate["local_branch"] != start["local_branch"]
            ):
                break
            stop += 1
        result.append(ordered[position:stop])
        position = stop
    return result


def reconstruct_original(rows, slots, number):
    db = {int(s["offset_steps"]): s for s in slots}
    a, b = int(rows[0]["offset_steps"]), int(rows[-1]["offset_steps"])
    left, right = db.get(a - 1), db.get(b + 1)
    result = {f: rows[0][f] for f in (*META, "event_id", "reference_time_s")}
    result.update(
        run=number,
        branch=rows[0]["local_branch"],
        start_offset_steps=a,
        end_offset_steps=b,
        start_time_s=int(rows[0]["source_step"]) / 50,
        end_time_s=int(rows[-1]["source_step"]) / 50,
        checkpoint_count=b - a + 1,
        checkpoint_span_s=(b - a) / 50,
        evidence_added_s=(b - a + 1) / 50,
        timer_before_ticks=int(rows[0]["local_timer_before_ticks"]),
        timer_after_ticks=int(rows[-1]["local_timer_after_ticks"]),
        threshold_ticks=50 if rows[0]["local_branch"] == "joint" else 25,
        observed_release=rows[-1]["local_timer_action"] == "release",
        touches_before_return=any(r["context_phase"] == "before_return" for r in rows),
        left_context_limited=left is None or left["local_audited"] != "True",
        right_context_limited=right is None or right["local_audited"] != "True",
        predecessor_certainty=(
            left["local_source_certainty"]
            if left and left["local_audited"] == "True"
            else None
        ),
        successor_certainty=(
            right["local_source_certainty"]
            if right and right["local_audited"] == "True"
            else None
        ),
    )
    return result


def ending(run, rows, slots, links):
    length, carry = len(rows), int(rows[0]["local_timer_before_ticks"])
    threshold = 50 if rows[0]["local_branch"] == "joint" else 25
    released = rows[-1]["local_timer_action"] == "release"
    for i, s in enumerate(rows):
        check(s["source_step"], int(rows[0]["source_step"]) + i)
        check(s["offset_steps"], int(rows[0]["offset_steps"]) + i)
        check(s["local_branch"], rows[0]["local_branch"])
        check(s["local_source_certainty"], "pass")
        check(s["local_timer_before_ticks"], carry + i)
        check(s["local_threshold_ticks"], threshold)
        action = "release" if released and i == length - 1 else "accumulate"
        check(s["local_timer_action"], action)
        check(s["local_timer_after_ticks"], 0 if action == "release" else carry + i + 1)
        assert (
            carry + i + 1 <= threshold
            if action == "release"
            else carry + i + 1 < threshold
        )
    check(run["checkpoint_count"], length)
    check(run["observed_release"], released)
    if released:
        assert carry + length == threshold
    last = rows[-1]
    b = int(last["offset_steps"])
    neighbor = {int(s["offset_steps"]): s for s in slots}.get(b + 1)
    result = dict(run, **dict.fromkeys(EXTRA))
    result.update(
        last_source_step=int(last["source_step"]),
        native_last_row_id=links[
            last["event_id"], last["reference_time_s"], last["offset_steps"]
        ],
        carried_ticks=carry,
        segment_ticks=length,
        decomposition="released_at_threshold" if released else "carried_plus_segment",
    )
    if neighbor:
        offset = b + 1
        assert int(neighbor["source_step"]) - int(last["source_step"]) == 1
        result.update(
            next_offset_steps=offset,
            next_source_step=int(neighbor["source_step"]),
            next_time_s=neighbor["source_time_s"],
            next_side=("pre", "reference", "post")[(offset >= 0) + (offset > 0)],
            next_slot_status=neighbor["slot_status"],
            next_context_phase=neighbor["context_phase"],
        )
        if neighbor["local_audited"] == "True":
            result.update(
                next_branch=neighbor["local_branch"],
                next_certainty=neighbor["local_source_certainty"],
                next_failed_gates=neighbor["local_failed_gates"],
                next_unknown_gates=neighbor["local_unknown_gates"],
                next_nominal_positive=neighbor["local_source_nominal_positive"],
                next_zero_reason=neighbor["local_source_zero_reason"],
                next_timer_before_ticks=neighbor["local_timer_before_ticks"],
                next_timer_after_ticks=neighbor["local_timer_after_ticks"],
                next_timer_action=neighbor["local_timer_action"],
                next_ready_from_timer=neighbor["local_ready_from_timer"],
                native_next_row_id=links[
                    neighbor["event_id"],
                    neighbor["reference_time_s"],
                    neighbor["offset_steps"],
                ],
            )
            result.update(
                {"next_gate_" + g: neighbor["local_source_gate_" + g] for g in GATES}
            )
    observed = neighbor is not None and neighbor["local_audited"] == "True"
    if released:
        result["end_status"] = "recorded_release"
    elif neighbor is None or not observed:
        result["end_status"] = (
            "right_context_limit" if neighbor is None else "missing_context"
        )
    elif neighbor["local_branch"] != last["local_branch"]:
        result["end_status"] = "branch_change"
    elif neighbor["local_source_certainty"] == "unknown":
        result["end_status"] = "unknown_boundary"
    else:
        assert neighbor["local_source_certainty"] == "fail"
        result["end_status"] = "gate_failure"
        failed = {g for g in GATES if neighbor["local_source_gate_" + g] == "fail"}
        assert failed and neighbor["local_failed_gates"] == ";".join(
            g for g in GATES if g in failed
        )
        check(neighbor["local_timer_before_ticks"], carry + length)
        assert carry + length > 0
        check(neighbor["local_timer_after_ticks"], 0)
        check(neighbor["local_timer_action"], "reset")
        check(neighbor["local_ready_from_timer"], False)
        result["failure_kind"] = (
            "other_only"
            if "nominal" not in failed
            else "nominal_only" if len(failed) == 1 else "nominal_with_other"
        )
        result["lost_ticks"] = carry + length
        result["lost_evidence_s"] = (carry + length) * 0.02
    return result


def physical_edges(ends, slot_map, traces):
    failures = [r for r in ends if r["end_status"] == "gate_failure"]
    identities = sorted({(r["case_id"], int(r["next_source_step"])) for r in failures})
    result = []
    for case, step in identities:
        members = [
            r
            for r in failures
            if r["case_id"] == case and int(r["next_source_step"]) == step
        ]
        keys = [
            (r["event_id"], r["reference_time_s"], str(r["next_offset_steps"]))
            for r in members
        ]
        assert len(set((r["event_id"], r["reference_time_s"]) for r in members)) == len(
            members
        )
        first = slot_map[keys[0]]
        source_fields = [
            f
            for f in first
            if f.startswith("local_") and f != "local_original_reference_joined"
        ]
        first_native = traces[str(members[0]["native_next_row_id"])]
        for r, kk in zip(members, keys):
            s = slot_map[kk]
            for f in source_fields:
                check(s[f], first[f])
            trace = traces[str(r["native_next_row_id"])]
            assert {k: v for k, v in trace.items() if k != "row_id"} == {
                k: v for k, v in first_native.items() if k != "row_id"
            }
            assert (
                r["lost_ticks"] == members[0]["lost_ticks"]
                and r["failure_kind"] == members[0]["failure_kind"]
            )
        fields = {f: first[f] for f in META}
        events = list(dict.fromkeys(r["event_id"] for r in members))
        fields.update(
            edge_id=case + "@" + str(step),
            source_step=step,
            source_time_s=first["source_time_s"],
            event_ids=";".join(events),
            reference_keys=";".join(
                r["event_id"] + "@" + r["reference_time_s"] for r in members
            ),
            member_native_rows=";".join(str(r["native_next_row_id"]) for r in members),
            multiplicity=len(members),
            branch=first["local_branch"],
            failed_gates=first["local_failed_gates"],
            unknown_gates=first["local_unknown_gates"],
            nominal_positive=first["local_source_nominal_positive"],
            zero_reason=first["local_source_zero_reason"],
            failure_kind=members[0]["failure_kind"],
            failed_gate_count=len(first["local_failed_gates"].split(";")),
            timer_before_ticks=int(first["local_timer_before_ticks"]),
            timer_after_ticks=int(first["local_timer_after_ticks"]),
            lost_ticks=members[0]["lost_ticks"],
            lost_evidence_s=members[0]["lost_evidence_s"],
        )
        for g in GATES:
            fields["gate_" + g] = first["local_source_gate_" + g]
        result.append({f: fields[f] for f in EDGE_FIELDS})
    assert sum(r["multiplicity"] for r in result) == len(failures)
    return result


def reference(d, ends):
    result = {f: d["time_s"] if f == "reference_time_s" else d[f] for f in REF_BASE}
    counts = Counter(r["end_status"] for r in ends)
    failures = [r for r in ends if r["end_status"] == "gate_failure"]
    post = [r for r in failures if int(r["next_offset_steps"]) > 0]
    result.update({m: 0 for m in METRICS})
    result["audited_references"], result["runs"] = 1, len(ends)
    for status in STATUSES:
        result[status + "_ends"] = counts[status]
    for kind in KINDS:
        result[kind + "_ends"] = len([r for r in failures if r["failure_kind"] == kind])
        result["post_" + kind + "_ends"] = len(
            [r for r in post if r["failure_kind"] == kind]
        )
    result["post_gate_failure_ends"] = len(post)
    result["lost_ticks"] = sum(int(r["lost_ticks"]) for r in failures)
    result["post_lost_ticks"] = sum(int(r["lost_ticks"]) for r in post)
    result["max_lost_ticks"] = max([r["lost_ticks"] for r in failures], default=None)
    result["post_max_lost_ticks"] = max([r["lost_ticks"] for r in post], default=None)
    return result


def anchors(events, refs, canonical=None):
    original = list(canonical.values()) if canonical is not None else events
    ne = Counter(e["case_id"] for e in original)
    nr = Counter(e["case_id"] for e in original if core.truth(e["received"]))
    result = []
    for e in events:
        parent = int(e["parent_rp_records"])
        expected = dict(
            census_case_weight=Fraction(1, ne[e["case_id"]]),
            received_case_weight=(
                Fraction(1, nr[e["case_id"]])
                if core.truth(e["received"])
                else Fraction()
            ),
            rp_case_mass=(
                Fraction(int(e["rp_records"]), parent) if parent else Fraction()
            ),
            rp_different_preview_case_mass=(
                Fraction(int(e["rp_different_preview_records"]), parent)
                if parent
                else Fraction()
            ),
        )
        for f, value in expected.items():
            inherited_number(e[f], value)
        rr = [r for r in refs if r["event_id"] == e["event_id"]]
        row = {f: e[f] for f in ANCHOR_INPUT}
        row.update({"exact_" + f: value for f, value in expected.items()})
        row.update(dict.fromkeys((*METRICS, *HAS)))
        if core.truth(e["returned"] or "False"):
            assert all(
                all(r[k] == e[k] for k in META) and r["phase"] == "available"
                for r in rr
            )
            row.update({m: sum(r[m] for r in rr) for m in METRICS})
            for kind, flag in zip(KINDS, HAS):
                row[flag] = any(r["post_" + kind + "_ends"] != 0 for r in rr)
        else:
            assert not rr
        result.append(row)
    for f, total in (
        ("census_case_weight", 570),
        ("received_case_weight", 570),
        ("rp_case_mass", 157),
    ):
        if canonical is not None:
            assert sum(e["exact_" + f] for e in result) == total
    return result


def summaries(old, events):
    result = []
    for oldrow in old:
        p, n, arm, mode, cat = (oldrow[k] for k in GROUP)
        included = [
            e
            for e in events
            if (e["partition"], e["noise"], e["variant"]) == (p, n, arm)
            and (mode == "all_modes" or e["mode"] == mode)
            and (cat == "all" or e["anchor_category"] == cat)
        ]
        returned = [e for e in included if core.truth(e["returned"] or "False")]
        mass = lambda aa, field="census_case_weight": sum(
            (e["exact_" + field] for e in aa), Fraction()
        )
        expected = dict(
            census_events=len(included),
            returned_events=len(returned),
            census_case_mass=mass(included),
            received_case_mass=mass(included, "received_case_weight"),
            rp_records=sum(int(e["rp_records"]) for e in included),
            rp_different_preview_records=sum(
                int(e["rp_different_preview_records"]) for e in included
            ),
            rp_case_mass=mass(included, "rp_case_mass"),
            rp_different_preview_case_mass=mass(
                included, "rp_different_preview_case_mass"
            ),
            returned_case_mass=mass(returned),
        )
        for f, value in expected.items():
            inherited_number(oldrow[f], value)
        row = {f: oldrow[f] for f in SUMMARY_INPUT}
        row.update(
            {
                "exact_" + f: value
                for f, value in expected.items()
                if isinstance(value, Fraction)
            }
        )
        for metric in METRICS:
            row[metric] = sum((e[metric] or 0) for e in included)
        for h in HAS:
            picked = [e for e in included if e[h]]
            row[h + "_events"] = len(picked)
            row[h + "_case_mass"] = mass(picked)
            row[h + "_fraction"] = (
                Fraction(len(picked), len(returned)) if len(returned) else None
            )
            row[h + "_case_mass_fraction"] = (
                mass(picked) / mass(returned) if len(returned) else None
            )
        result.append(row)
    return result


def closure(rows):
    index = {tuple(r[k] for k in GROUP): r for r in rows}
    comparisons = 0
    for kk, row in index.items():
        assert sum(row[k + "_ends"] for k in KINDS) == row["gate_failure_ends"]
        assert (
            sum(row["post_" + k + "_ends"] for k in KINDS)
            == row["post_gate_failure_ends"]
        )
        assert sum(row[k + "_ends"] for k in STATUSES) == row["runs"]
        for dimension, labels in ((3, ("none", "random", "blackout")), (4, CATEGORIES)):
            if kk[dimension] != ("all_modes" if dimension == 3 else "all"):
                continue
            children = [
                index[kk[:dimension] + (label,) + kk[dimension + 1 :]]
                for label in labels
            ]
            for field in (
                *METRICS,
                *(h + s for h in HAS for s in ("_events", "_case_mass")),
            ):
                assert sum(child[field] for child in children) == row[field]
                comparisons += 1
    return comparisons


def compare(path, expected, fields):
    rows = read(path)
    actual = core.index(rows, fields)
    wanted = {tuple(str(r[k]) for k in fields): r for r in expected}
    assert len(wanted) == len(expected) and actual.keys() == wanted.keys()
    assert [tuple(r[k] for k in fields) for r in rows] == list(wanted)
    for key_, row in actual.items():
        assert set(row) == set(wanted[key_])
        for f, value in wanted[key_].items():
            if isinstance(value, float):
                assert math.isclose(float(row[f]), value, abs_tol=1e-12, rel_tol=1e-10)
                core.comparisons += 1
            else:
                check(row[f], value)


def provenance(results):
    prior = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    source = json.loads((results / "sources.json").read_text())
    freeze = json.loads((STUDY / "freeze-pins.json").read_text())
    assert freeze == dict(
        base_main=BASE_MAIN,
        sources_sha256=BASE_SHA,
        proof_sha256=PROOF_SHA,
        verifier_helper_sha256=HELPERS["verify_local_admission.py"],
    )
    assert (
        core.sha(BASE / "sources.json") == BASE_SHA
        and core.sha(BASE / "independent-verification.json") == PROOF_SHA
    )
    assert proof["sources_sha256"] == BASE_SHA
    expected_pins = {
        **prior["input_sha256"],
        **{
            "fusion-return-local-admission/" + p.name: core.sha(p)
            for p in BASE.iterdir()
        },
    }
    assert source["input_sha256"] == expected_pins
    for n, h in expected_pins.items():
        assert core.sha(RESULTS / n) == h
    expected_helpers = {
        **prior["analysis"]["helpers_sha256"],
        "analyze_local_admission.py": prior["analysis"]["script_sha256"],
    }
    assert source["analysis"]["helpers_sha256"] == expected_helpers
    assert source["analysis"]["verifier_helpers_sha256"] == HELPERS
    for n, h in {**expected_helpers, **HELPERS}.items():
        assert core.sha(ROOT / n) == h
    assert source["analysis"]["code_sha256"] == inherited.CODE
    for n, h in inherited.CODE.items():
        assert core.sha(REPO / n) == h
    for field, path in (
        ("script_sha256", ROOT / "analyze_run_breaks.py"),
        ("protocol_sha256", STUDY / "protocol.md"),
        ("freeze_sha256", STUDY / "freeze-pins.json"),
    ):
        assert source["analysis"][field] == core.sha(path)
    for n, h in prior["output_sha256"].items():
        assert core.sha(BASE / n) == h
    for field, name in (
        ("protocol_sha256", "protocol.md"),
        ("freeze_sha256", "freeze-pins.json"),
    ):
        assert core.sha(BASE / name) == prior["analysis"][field]
    plots = json.loads((BASE / "plot-sources.json").read_text())
    assert core.sha(BASE / "plot-sources.json") == proof["plot_sources_sha256"]
    assert core.sha(ROOT / "plot_local_admission.py") == plots["script_sha256"]
    for n, h in {**plots["input_sha256"], **plots["output_sha256"]}.items():
        assert core.sha(BASE / n) == h
    native = proof["native"]
    for n, h in (
        ("native-checkpoints.tsv", native["checkpoints_sha256"]),
        ("native-contexts.csv", native["context_map_sha256"]),
        ("native-window.csv", native["trace_sha256"]),
        ("native-lattice.csv", native["command"]["native_lattice_sha256"]),
        ("native-admission-inputs.tsv", native["admission"]["inputs_sha256"]),
        ("native-admission.csv", native["admission"]["trace_sha256"]),
    ):
        assert core.sha(BASE / n) == h
    assert source["inherited_native"] == native
    assert source["inherited_local_admission_proof_sha256"] == PROOF_SHA
    assert source["schema"] == dict(
        statuses=list(STATUSES),
        failure_kinds=list(KINDS),
        source_gate_order=list(GATES),
        retrospective=True,
        inherited_only=True,
        physical_key=["case_id", "next_source_step"],
        weighting="original preselection anchor weights; no segment or physical-edge weights",
    )
    assert set(source["output_sha256"]) == {
        n + ".csv"
        for n in (
            "run-ends",
            "reset-edges",
            "reference-breaks",
            "anchor-breaks",
            "break-summary",
        )
    }
    for n, h in source["output_sha256"].items():
        assert core.sha(results / n) == h
    return source, proof


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for n in ("results", "figures", "output"):
        p.add_argument("--" + n, type=Path, required=True)
    args = p.parse_args()
    if not __debug__ or args.output.exists():
        p.error("Use an unused evidence filename without -O")
    source, priorproof = provenance(args.results)
    cases, canonical = core.original_inputs()
    events = read(BASE / "anchor-local.csv")
    assert set(canonical) == {e["event_id"] for e in events}
    for e in events:
        for f in (
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
            check(e[f], canonical[e["event_id"]][f])
    original_checks = core.comparisons
    selected = [
        d for d in read(BASE / "local-decisions.csv") if core.truth(d["local_audited"])
    ]
    slots = read(BASE / "local-slots.csv")
    oldruns = read(BASE / "ready-runs.csv")
    links, traces = native_links(
        slots,
        read(BASE / "native-contexts.csv"),
        read(BASE / "native-admission-inputs.tsv"),
        read(BASE / "native-admission.csv"),
    )
    old_by = core.index(oldruns, ("event_id", "reference_time_s", "run"))
    ends, refs = [], []
    for d in selected:
        assert d["phase"] == "available" and d["branch"] == "joint"
        ss = [
            s
            for s in slots
            if (s["event_id"], s["reference_time_s"]) == (d["event_id"], d["time_s"])
        ]
        pieces = scan(ss)
        new = []
        for number, rows in enumerate(pieces, 1):
            old = old_by[d["event_id"], d["time_s"], str(number)]
            expected = reconstruct_original(rows, ss, number)
            assert set(expected) == set(old)
            for f, value in expected.items():
                check(old[f], value)
            new.append(ending(old, rows, ss, links))
        refs.append(reference(d, new))
        ends.extend(new)
    assert len(ends) == len(oldruns) == 191
    assert [(r["event_id"], r["reference_time_s"], r["run"]) for r in ends] == [
        (r["event_id"], r["reference_time_s"], r["run"]) for r in oldruns
    ]
    edges = physical_edges(
        ends,
        {(s["event_id"], s["reference_time_s"], s["offset_steps"]): s for s in slots},
        traces,
    )
    anchors_out = anchors(events, refs, canonical)
    summary = summaries(read(BASE / "local-summary.csv"), anchors_out)
    assert (
        len(cases),
        len({c["family_id"] for c in cases}),
        len(events),
        len(refs),
        len(slots),
        len(links),
        len(summary),
    ) == (972, 324, 3997, 25, 1275, 1215, 336)
    assert sum(int(e["rp_records"]) for e in anchors_out) == 10995
    for name, rows, fields in (
        ("run-ends", ends, ("event_id", "reference_time_s", "run")),
        ("reset-edges", edges, ("edge_id",)),
        ("reference-breaks", refs, ("event_id", "reference_time_s")),
        ("anchor-breaks", anchors_out, ("event_id",)),
        ("break-summary", summary, GROUP),
    ):
        compare(args.results / (name + ".csv"), rows, fields)
    audit = dict(
        planned_cases=972,
        planned_families=324,
        timeout_cases=570,
        census_events=len(events),
        returned_anchors=sum(core.truth(e["returned"] or "False") for e in events),
        inherited_decision_references=3751,
        inherited_zero_probes=1096,
        selected_references=len(refs),
        planned_slots=len(slots),
        included_slots=len(links),
        unique_context_points=247,
        missing_slots=60,
        before_return_slots=82,
        inherited_runs=len(ends),
        end_statuses=dict(Counter(r["end_status"] for r in ends)),
        failure_kinds=dict(
            Counter(r["failure_kind"] for r in ends if r["failure_kind"])
        ),
        unique_reset_edges=len(edges),
        reset_multiplicity=sum(r["multiplicity"] for r in edges),
        inherited_native_links_checked=len(links),
        summary_rows=len(summary),
        additive_closure_comparisons=closure(summary),
        fresh_raw_reads=0,
        fresh_native_runs=0,
    )
    assert audit == source["audit"]
    plots = json.loads((args.figures / "plot-sources.json").read_text())
    assert plots["script_sha256"] == core.sha(ROOT / "plot_run_breaks.py")
    assert plots["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in (
            "sources.json",
            "run-ends.csv",
            "reset-edges.csv",
            "reference-breaks.csv",
            "anchor-breaks.csv",
            "break-summary.csv",
        )
    }
    assert set(plots["output_sha256"]) == {
        "run-breaks.svg",
        "plot-run-ends.csv",
        "plot-reset-edges.csv",
        "plot-values.csv",
        "plot-return-values.csv",
    }
    for n, h in plots["output_sha256"].items():
        assert core.sha(args.figures / n) == h
    for name, rows, fields in (
        ("plot-run-ends.csv", ends, ("event_id", "reference_time_s", "run")),
        ("plot-reset-edges.csv", edges, ("edge_id",)),
        ("plot-values.csv", [r for r in summary if r["mode"] != "all_modes"], GROUP),
        (
            "plot-return-values.csv",
            [e for e in anchors_out if core.truth(e["returned"] or "False")],
            ("event_id",),
        ),
    ):
        compare(args.figures / name, rows, fields)
    evidence = dict(
        **audit,
        comparisons=core.comparisons - original_checks,
        original_join_comparisons=original_checks,
        plotted_summary_rows=252,
        plotted_return_rows=137,
        script_sha256=core.sha(Path(__file__)),
        sources_sha256=core.sha(args.results / "sources.json"),
        plot_sources_sha256=core.sha(args.figures / "plot-sources.json"),
        inherited_local_admission_proof_sha256=PROOF_SHA,
        inherited_native_tests=priorproof["native"]["native_tests"]
        + priorproof["native"]["command"]["native_tests"]
        + priorproof["native"]["admission"]["native_tests"],
        method="independent segment/end traversal, carry/reset checks, physical edge agreement and original census accounting; inherited native queries/traces rechecked without new raw/native execution"
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
