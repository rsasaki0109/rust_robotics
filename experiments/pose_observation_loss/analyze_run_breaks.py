#!/usr/bin/env python3
"""Audit endings of the fixed known-pass segments using verified artifacts."""

import argparse
from collections import Counter, defaultdict
import csv
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent.parent
RESULTS = ROOT / "results"
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


def sha(path):
    with path.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(path):
    with path.open(newline="") as f:
        reader = csv.DictReader(f, delimiter="\t" if path.suffix == ".tsv" else ",")
        assert len(reader.fieldnames) == len(set(reader.fieldnames))
        rows = list(reader)
    assert all(None not in r and None not in r.values() for r in rows)
    return rows


def write(path, rows, fields):
    assert len(fields) == len(set(fields)) and all(set(r) == set(fields) for r in rows)
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def truth(value, raw=False):
    assert value in (("true", "false") if raw else ("True", "False")), value
    return value == ("true" if raw else "True")


def check(actual, expected):
    if isinstance(expected, float):
        assert math.isclose(float(actual), expected, rel_tol=1e-10, abs_tol=1e-12)
    else:
        assert actual == ("" if expected is None else str(expected)), (actual, expected)


def inherited_number(actual, expected):
    # Preserve old decimal strings; derive new exact weights from integer counts.
    if isinstance(expected, Fraction):
        assert math.isclose(
            float(Fraction(actual)), float(expected), rel_tol=1e-10, abs_tol=2e-12
        )
    else:
        check(actual, expected)


def key(s):
    return s["event_id"], s["reference_time_s"]


def ticks(value, limit):
    n = round(float(value) * 50)
    assert 0 <= n < limit and abs(float(value) - n / 50) <= 5e-7
    return n


def native_slot(s, q, trace):
    assert s["local_audited"] == "True"
    for f in ("event_id", "reference_time_s", "offset_steps", "source_step"):
        assert q[f] == s[f]
    assert q["row_id"] == trace["row_id"]
    fields = {
        "prior_paused": "local_prior_safety_paused",
        "prior_guard": "local_prior_guard_active",
        "correction": "local_input_correction",
        "received": "local_input_correction_received",
        "guard_triggered": "local_input_guard_triggered",
        "measured_speed": "local_input_measured_speed",
        "sample_count": "local_input_recovery_sample_count",
        "recovery_x": "local_input_recovery_x",
        "recovery_y": "local_input_recovery_y",
        "recovery_yaw": "local_input_recovery_yaw",
        "margin": "local_input_speed_margin",
        "clearance": "local_input_recovery_clearance",
        "nominal": "local_input_nominal_target_speed",
        "release_clearance": "local_input_release_preview_clearance_lower_bound",
        "release_margin": "local_input_guard_release_margin",
    }
    assert all(q[a] == s[b] for a, b in fields.items())
    assert float(q["age_exact"]).hex() == s["local_age_exact_hex"]
    for qf, sf, limit in (
        ("before_joint_ticks", "recovery_evidence", 50),
        ("before_guard_ticks", "guard_release_evidence", 25),
    ):
        assert q[qf] == str(ticks(s["local_prior_" + sf], limit))
    assert trace["branch"] == s["local_branch"]
    for g in GATES:
        state = s["local_source_gate_" + g]
        if state in ("pass", "fail"):
            assert truth(trace["gate_" + g], True) == (state == "pass")
    if s["local_source_certainty"] != "unknown":
        if s["local_ready_from_timer"]:
            assert truth(trace["ready"], True) == truth(s["local_ready_from_timer"])
        for f in ("recovery_evidence", "guard_release_evidence"):
            assert format(float(trace[f]), ".6f") == s["local_input_" + f]
        for f in ("safety_paused", "guard_active", "resumed", "guard_released"):
            assert trace[f] == s["local_input_" + f]


def native_links(slots, mapping, queries, traces):
    qmap = {q["row_id"]: q for q in queries}
    tmap = {t["row_id"]: t for t in traces}
    assert len(qmap) == len(queries) == len(tmap) == len(traces) == len(mapping)
    assert set(qmap) == set(tmap) == {m["row_id"] for m in mapping}
    smap = {
        (s["event_id"], s["reference_time_s"], s["offset_steps"]): s
        for s in slots
        if s["local_audited"] == "True"
    }
    links = {}
    for m in mapping:
        k = m["event_id"], m["reference_time_s"], m["offset_steps"]
        assert k not in links and m["case_id"] == smap[k]["case_id"]
        assert m["source_time_s"] == smap[k]["source_time_s"]
        native_slot(smap[k], qmap[m["row_id"]], tmap[m["row_id"]])
        links[k] = int(m["row_id"])
    assert set(links) == set(smap)
    return links, tmap


def segments(slots):
    assert [int(s["offset_steps"]) for s in slots] == list(range(-25, 26))
    result = []
    for s in slots:
        if s["local_audited"] != "True" or s["local_source_certainty"] != "pass":
            continue
        if (
            result
            and int(s["offset_steps"]) == int(result[-1][-1]["offset_steps"]) + 1
            and s["local_branch"] == result[-1][-1]["local_branch"]
            and result[-1][-1]["local_timer_action"] != "release"
        ):
            result[-1].append(s)
        else:
            result.append([s])
    return result


def original_run(rows, slots, number):
    first, last = rows[0], rows[-1]
    a, b = int(first["offset_steps"]), int(last["offset_steps"])
    by = {int(s["offset_steps"]): s for s in slots}
    neighbor = lambda off: (
        by[off]["local_source_certainty"]
        if off in by and by[off]["local_audited"] == "True"
        else None
    )
    return dict(
        **{f: first[f] for f in (*META, "event_id", "reference_time_s")},
        run=number,
        branch=first["local_branch"],
        start_offset_steps=a,
        end_offset_steps=b,
        start_time_s=float(first["source_time_s"]),
        end_time_s=float(last["source_time_s"]),
        checkpoint_count=len(rows),
        checkpoint_span_s=(len(rows) - 1) / 50,
        evidence_added_s=len(rows) / 50,
        timer_before_ticks=int(first["local_timer_before_ticks"]),
        timer_after_ticks=int(last["local_timer_after_ticks"]),
        threshold_ticks=int(first["local_threshold_ticks"]),
        observed_release=last["local_timer_action"] == "release",
        touches_before_return=any(s["context_phase"] == "before_return" for s in rows),
        left_context_limited=a == -25 or by[a - 1]["local_audited"] != "True",
        right_context_limited=b == 25 or by[b + 1]["local_audited"] != "True",
        predecessor_certainty=neighbor(a - 1),
        successor_certainty=neighbor(b + 1),
    )


def ending(run, rows, slots, links):
    first, last = rows[0], rows[-1]
    out = dict(run, **dict.fromkeys(EXTRA))
    carry, length = int(run["timer_before_ticks"]), len(rows)
    threshold = int(run["threshold_ticks"])
    assert length == int(run["checkpoint_count"])
    prior_after = carry
    for i, s in enumerate(rows):
        assert int(s["source_step"]) == int(first["source_step"]) + i
        assert int(s["offset_steps"]) == int(first["offset_steps"]) + i
        assert s["local_branch"] == first["local_branch"]
        assert s["local_source_certainty"] == "pass"
        assert int(s["local_timer_before_ticks"]) == prior_after
        if s["local_timer_action"] == "release":
            assert i == length - 1 and prior_after + 1 == threshold
            assert int(s["local_timer_after_ticks"]) == 0
        else:
            assert s["local_timer_action"] == "accumulate"
            assert int(s["local_timer_after_ticks"]) == prior_after + 1 < threshold
        prior_after = int(s["local_timer_after_ticks"])
    released = truth(run["observed_release"])
    assert carry + length == (threshold if released else prior_after)
    out.update(
        last_source_step=int(last["source_step"]),
        native_last_row_id=links[(*key(last), last["offset_steps"])],
        carried_ticks=carry,
        segment_ticks=length,
        decomposition="released_at_threshold" if released else "carried_plus_segment",
    )
    nxt = next(
        (s for s in slots if int(s["offset_steps"]) == int(last["offset_steps"]) + 1),
        None,
    )
    if nxt is not None:
        offset = int(nxt["offset_steps"])
        out.update(
            next_offset_steps=offset,
            next_source_step=int(nxt["source_step"]),
            next_time_s=nxt["source_time_s"],
            next_side="pre" if offset < 0 else "reference" if offset == 0 else "post",
            next_slot_status=nxt["slot_status"],
            next_context_phase=nxt["context_phase"],
        )
        assert int(nxt["source_step"]) == int(last["source_step"]) + 1
        if nxt["local_audited"] == "True":
            for a, b in (
                ("branch", "branch"),
                ("certainty", "source_certainty"),
                ("failed_gates", "failed_gates"),
                ("unknown_gates", "unknown_gates"),
                ("nominal_positive", "source_nominal_positive"),
                ("zero_reason", "source_zero_reason"),
                ("timer_before_ticks", "timer_before_ticks"),
                ("timer_after_ticks", "timer_after_ticks"),
                ("timer_action", "timer_action"),
                ("ready_from_timer", "ready_from_timer"),
            ):
                out["next_" + a] = nxt["local_" + b]
            for g in GATES:
                out["next_gate_" + g] = nxt["local_source_gate_" + g]
            out["native_next_row_id"] = links[(*key(nxt), nxt["offset_steps"])]
    if released:
        status = "recorded_release"
    elif nxt is None:
        status = "right_context_limit"
    elif nxt["local_audited"] != "True":
        status = "missing_context"
    elif nxt["local_branch"] != last["local_branch"]:
        status = "branch_change"
    elif nxt["local_source_certainty"] == "unknown":
        status = "unknown_boundary"
    else:
        assert nxt["local_source_certainty"] == "fail"
        status = "gate_failure"
    out["end_status"] = status
    if status == "gate_failure":
        failed = nxt["local_failed_gates"].split(";")
        assert failed and failed == [
            g for g in GATES if nxt["local_source_gate_" + g] == "fail"
        ]
        assert int(nxt["local_timer_before_ticks"]) == prior_after > 0
        assert (
            nxt["local_timer_after_ticks"] == "0"
            and nxt["local_timer_action"] == "reset"
            and nxt["local_ready_from_timer"] == "False"
        )
        out.update(
            failure_kind=(
                "nominal_only"
                if failed == ["nominal"]
                else "nominal_with_other" if "nominal" in failed else "other_only"
            ),
            lost_ticks=prior_after,
            lost_evidence_s=prior_after / 50,
        )
        assert out["lost_ticks"] == carry + length
    return out


def physical_edges(ends, slot_map, traces):
    grouped = defaultdict(list)
    for r in ends:
        if r["end_status"] == "gate_failure":
            grouped[r["case_id"], int(r["next_source_step"])].append(r)
    output = []
    for (case, step), members in sorted(grouped.items()):
        slots = [slot_map[(*key(r), str(r["next_offset_steps"]))] for r in members]
        first, s = members[0], slots[0]
        signature = lambda x: {
            k: v
            for k, v in x.items()
            if k.startswith("local_") and k != "local_original_reference_joined"
        }
        native = lambda r: {
            k: v
            for k, v in traces[str(r["native_next_row_id"])].items()
            if k != "row_id"
        }
        assert all(signature(x) == signature(s) for x in slots)
        assert all(native(r) == native(first) for r in members)
        assert all(
            (r["failure_kind"], r["lost_ticks"])
            == (first["failure_kind"], first["lost_ticks"])
            for r in members
        )
        assert len({key(r) for r in members}) == len(members)
        out = {k: s[k] for k in META}
        out.update(
            edge_id=f"{case}@{step}",
            source_step=step,
            source_time_s=s["source_time_s"],
            event_ids=";".join(dict.fromkeys(r["event_id"] for r in members)),
            reference_keys=";".join(
                r["event_id"] + "@" + r["reference_time_s"] for r in members
            ),
            member_native_rows=";".join(str(r["native_next_row_id"]) for r in members),
            multiplicity=len(members),
            branch=s["local_branch"],
            failed_gates=s["local_failed_gates"],
            unknown_gates=s["local_unknown_gates"],
            nominal_positive=s["local_source_nominal_positive"],
            zero_reason=s["local_source_zero_reason"],
            failure_kind=first["failure_kind"],
            failed_gate_count=len(s["local_failed_gates"].split(";")),
            timer_before_ticks=int(s["local_timer_before_ticks"]),
            timer_after_ticks=int(s["local_timer_after_ticks"]),
            lost_ticks=first["lost_ticks"],
            lost_evidence_s=first["lost_evidence_s"],
        )
        out.update({"gate_" + g: s["local_source_gate_" + g] for g in GATES})
        assert set(out) == set(EDGE_FIELDS)
        output.append(out)
    assert sum(r["multiplicity"] for r in output) == sum(
        r["end_status"] == "gate_failure" for r in ends
    )
    return output


def reference(d, ends):
    out = {k: d["time_s"] if k == "reference_time_s" else d[k] for k in REF_BASE}
    out.update({m: 0 for m in METRICS})
    failures = [r for r in ends if r["end_status"] == "gate_failure"]
    post = [r for r in failures if r["next_side"] == "post"]
    out.update(
        audited_references=1,
        runs=len(ends),
        gate_failure_ends=len(failures),
        post_gate_failure_ends=len(post),
        lost_ticks=sum(r["lost_ticks"] for r in failures),
        post_lost_ticks=sum(r["lost_ticks"] for r in post),
        max_lost_ticks=max((r["lost_ticks"] for r in failures), default=None),
        post_max_lost_ticks=max((r["lost_ticks"] for r in post), default=None),
    )
    for k in KINDS:
        out[k + "_ends"] = sum(r["failure_kind"] == k for r in failures)
        out["post_" + k + "_ends"] = sum(r["failure_kind"] == k for r in post)
    for k in STATUSES[1:]:
        out[k + "_ends"] = sum(r["end_status"] == k for r in ends)
    assert out["runs"] == sum(out[k + "_ends"] for k in STATUSES)
    return out


def anchors(events, refs):
    by = defaultdict(list)
    for r in refs:
        by[r["event_id"]].append(r)
    ne, nr = Counter(e["case_id"] for e in events), Counter(
        e["case_id"] for e in events if truth(e["received"])
    )
    result = []
    for e in events:
        out = {k: e[k] for k in ANCHOR_INPUT}
        for f, expected in (
            ("census_case_weight", Fraction(1, ne[e["case_id"]])),
            (
                "received_case_weight",
                Fraction(1, nr[e["case_id"]]) if truth(e["received"]) else Fraction(),
            ),
            (
                "rp_case_mass",
                (
                    Fraction(int(e["rp_records"]), int(e["parent_rp_records"]))
                    if int(e["parent_rp_records"])
                    else Fraction()
                ),
            ),
            (
                "rp_different_preview_case_mass",
                (
                    Fraction(
                        int(e["rp_different_preview_records"]),
                        int(e["parent_rp_records"]),
                    )
                    if int(e["parent_rp_records"])
                    else Fraction()
                ),
            ),
        ):
            inherited_number(e[f], expected)
            out["exact_" + f] = expected
        out.update(dict.fromkeys((*METRICS, *HAS)))
        dd = by[e["event_id"]]
        if truth(e["returned"] or "False"):
            assert all(
                all(d[k] == e[k] for k in META) and d["phase"] == "available"
                for d in dd
            )
            out.update({m: sum(d[m] for d in dd) for m in METRICS})
            out.update(
                {
                    h: any(d["post_" + k + "_ends"] > 0 for d in dd)
                    for h, k in zip(HAS, KINDS)
                }
            )
        else:
            assert not dd
        result.append(out)
    return result


def summaries(old, events):
    output = []
    mass = lambda rows, f="census_case_weight": sum(
        (a["exact_" + f] for a in rows), Fraction()
    )
    for r in old:
        selected = [
            a
            for a in events
            if all(a[f] == r[f] for f in GROUP[:3])
            and (r["mode"] == "all_modes" or a["mode"] == r["mode"])
            and (
                r["category_selector"] == "all"
                or a["anchor_category"] == r["category_selector"]
            )
        ]
        returned = [a for a in selected if truth(a["returned"] or "False")]
        exact = {}
        for f, expected in (
            ("census_events", len(selected)),
            ("returned_events", len(returned)),
            ("census_case_mass", mass(selected)),
            ("received_case_mass", mass(selected, "received_case_weight")),
            ("rp_case_mass", mass(selected, "rp_case_mass")),
            (
                "rp_different_preview_case_mass",
                mass(selected, "rp_different_preview_case_mass"),
            ),
            ("returned_case_mass", mass(returned)),
            ("rp_records", sum(int(a["rp_records"]) for a in selected)),
            (
                "rp_different_preview_records",
                sum(int(a["rp_different_preview_records"]) for a in selected),
            ),
        ):
            inherited_number(r[f], expected)
            if isinstance(expected, Fraction):
                exact["exact_" + f] = expected
        out = {k: r[k] for k in SUMMARY_INPUT}
        out.update(exact)
        out.update({m: sum(a[m] or 0 for a in selected) for m in METRICS})
        for h in HAS:
            picked = [a for a in selected if a[h]]
            out.update(
                {
                    h + "_events": len(picked),
                    h + "_case_mass": mass(picked),
                    h
                    + "_fraction": (
                        Fraction(len(picked), len(returned)) if returned else None
                    ),
                    h
                    + "_case_mass_fraction": (
                        mass(picked) / mass(returned) if returned else None
                    ),
                }
            )
        output.append(out)
    return output


def closure(rows):
    by = {tuple(r[k] for k in GROUP): r for r in rows}
    checks = 0
    for key_, row in by.items():
        assert row["gate_failure_ends"] == sum(row[k + "_ends"] for k in KINDS)
        assert row["post_gate_failure_ends"] == sum(
            row["post_" + k + "_ends"] for k in KINDS
        )
        assert row["runs"] == sum(row[k + "_ends"] for k in STATUSES)
        for i, whole, parts in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", CATEGORIES),
        ):
            if key_[i] == whole:
                for f in (
                    *METRICS,
                    *(h + s for h in HAS for s in ("_events", "_case_mass")),
                ):
                    assert row[f] == sum(
                        by[key_[:i] + (p,) + key_[i + 1 :]][f] for p in parts
                    )
                    checks += 1
    return checks


def baseline():
    freeze = json.loads((STUDY / "freeze-pins.json").read_text())
    assert freeze == dict(
        base_main=BASE_MAIN,
        sources_sha256=BASE_SHA,
        proof_sha256=PROOF_SHA,
        verifier_helper_sha256=sha(ROOT / "verify_local_admission.py"),
    )
    assert (
        sha(BASE / "sources.json") == BASE_SHA
        and sha(BASE / "independent-verification.json") == PROOF_SHA
    )
    source = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    assert proof["sources_sha256"] == BASE_SHA
    pins = {
        **source["input_sha256"],
        **{"fusion-return-local-admission/" + p.name: sha(p) for p in BASE.iterdir()},
    }
    for n, h in pins.items():
        assert sha(RESULTS / n) == h, n
    for n, h in source["output_sha256"].items():
        assert sha(BASE / n) == h
    for n, h in source["analysis"]["code_sha256"].items():
        assert sha(REPO / n) == h
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_local_admission.py": source["analysis"]["script_sha256"],
    }
    vhelpers = {
        **source["analysis"]["verifier_helpers_sha256"],
        "verify_local_admission.py": freeze["verifier_helper_sha256"],
    }
    for n, h in {**helpers, **vhelpers}.items():
        assert sha(ROOT / n) == h
    for field, name in (
        ("protocol_sha256", "protocol.md"),
        ("freeze_sha256", "freeze-pins.json"),
    ):
        assert sha(BASE / name) == source["analysis"][field]
    plots = json.loads((BASE / "plot-sources.json").read_text())
    assert sha(BASE / "plot-sources.json") == proof["plot_sources_sha256"]
    assert sha(ROOT / "plot_local_admission.py") == plots["script_sha256"]
    for n, h in {**plots["input_sha256"], **plots["output_sha256"]}.items():
        assert sha(BASE / n) == h
    native = proof["native"]
    for name, h in (
        ("native-checkpoints.tsv", native["checkpoints_sha256"]),
        ("native-contexts.csv", native["context_map_sha256"]),
        ("native-window.csv", native["trace_sha256"]),
        ("native-lattice.csv", native["command"]["native_lattice_sha256"]),
        ("native-admission-inputs.tsv", native["admission"]["inputs_sha256"]),
        ("native-admission.csv", native["admission"]["trace_sha256"]),
    ):
        assert sha(BASE / name) == h
    assert (
        native["replayed_steps"] == 36000
        and native["admission"]["traced_context_queries"] == 1215
    )
    return source, proof, pins, helpers, vhelpers


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    if not __debug__ or args.output.exists():
        p.error("Use a new output directory without -O")
    oldsource, proof, pins, helpers, vhelpers = baseline()
    slots = read(BASE / "local-slots.csv")
    oldruns = read(BASE / "ready-runs.csv")
    selected = [
        d for d in read(BASE / "local-decisions.csv") if truth(d["local_audited"])
    ]
    assert len(slots) == 1275 and len(oldruns) == 191 and len(selected) == 25
    links, traces = native_links(
        slots,
        read(BASE / "native-contexts.csv"),
        read(BASE / "native-admission-inputs.tsv"),
        read(BASE / "native-admission.csv"),
    )
    grouped = defaultdict(list)
    oldgroup = defaultdict(list)
    for s in slots:
        grouped[key(s)].append(s)
    for r in oldruns:
        oldgroup[key(r)].append(r)
    ends, refs = [], []
    for d in selected:
        kk = d["event_id"], d["time_s"]
        rr = segments(grouped[kk])
        assert len(rr) == len(oldgroup[kk])
        erows = []
        for i, (rows, old) in enumerate(zip(rr, oldgroup[kk]), 1):
            expected = original_run(rows, grouped[kk], i)
            assert set(expected) == set(old)
            for f, value in expected.items():
                check(old[f], value)
            erows.append(ending(old, rows, grouped[kk], links))
        refs.append(reference(d, erows))
        ends.extend(erows)
    assert [(r["event_id"], r["reference_time_s"], r["run"]) for r in ends] == [
        (r["event_id"], r["reference_time_s"], r["run"]) for r in oldruns
    ]
    edges = physical_edges(
        ends, {(*key(s), s["offset_steps"]): s for s in slots}, traces
    )
    events = anchors(read(BASE / "anchor-local.csv"), refs)
    assert (
        sum(e["exact_census_case_weight"] for e in events)
        == sum(e["exact_received_case_weight"] for e in events)
        == 570
    )
    assert sum(e["exact_rp_case_mass"] for e in events) == 157
    assert sum(int(e["rp_records"]) for e in events) == 10995
    summary = summaries(read(BASE / "local-summary.csv"), events)
    assert len(events) == 3997 and len(summary) == 336 and len(links) == 1215
    audit = dict(
        planned_cases=972,
        planned_families=324,
        timeout_cases=570,
        census_events=len(events),
        returned_anchors=sum(truth(e["returned"] or "False") for e in events),
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
    args.output.mkdir(parents=True)
    for name, rows, fields in (
        ("run-ends", ends, (*oldruns[0], *EXTRA)),
        ("reset-edges", edges, EDGE_FIELDS),
        (
            "reference-breaks",
            refs,
            (*REF_BASE, *METRICS, "max_lost_ticks", "post_max_lost_ticks"),
        ),
        ("anchor-breaks", events, (*ANCHOR_BASE, *METRICS, *HAS)),
        (
            "break-summary",
            summary,
            (
                *SUMMARY_BASE,
                *METRICS,
                *(
                    h + s
                    for h in HAS
                    for s in (
                        "_events",
                        "_case_mass",
                        "_fraction",
                        "_case_mass_fraction",
                    )
                ),
            ),
        ),
    ):
        write(args.output / (name + ".csv"), rows, fields)
    source = dict(
        input_sha256=pins,
        analysis=dict(
            script_sha256=sha(Path(__file__)),
            protocol_sha256=sha(STUDY / "protocol.md"),
            freeze_sha256=sha(STUDY / "freeze-pins.json"),
            helpers_sha256=helpers,
            verifier_helpers_sha256=vhelpers,
            code_sha256=oldsource["analysis"]["code_sha256"],
        ),
        schema=dict(
            statuses=STATUSES,
            failure_kinds=KINDS,
            source_gate_order=GATES,
            retrospective=True,
            inherited_only=True,
            physical_key=["case_id", "next_source_step"],
            weighting="original preselection anchor weights; no segment or physical-edge weights",
        ),
        inherited_local_admission_proof_sha256=PROOF_SHA,
        inherited_native=proof["native"],
        audit=audit,
        output_sha256={f.name: sha(f) for f in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
