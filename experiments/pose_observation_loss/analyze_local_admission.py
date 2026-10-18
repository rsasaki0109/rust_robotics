#!/usr/bin/env python3
"""Audit original admission gates and actual timers over fixed local contexts."""

import argparse
from collections import Counter, defaultdict
from fractions import Fraction
from itertools import product
import csv
import hashlib
import json
import math
from pathlib import Path

import analyze_point_history as prior
import analyze_command_sign as command
import analyze_return_admission as admission

ROOT, REPO, RESULTS = prior.ROOT, prior.REPO, prior.RESULTS
BASE = RESULTS / "fusion-return-point-history"
STUDY = RESULTS / "fusion-return-local-admission"
BASE_MAIN = "b47c3939135776bb698989181cb1e7c599ab2285"
BASE_SHA = "41891a6cf0b16bd70edffa8e06579ccedec73a8e35e2802ce39666adbf30ff7d"
PROOF_SHA = "fe4e5d14009b8310aed1bdda012ed5d0d58113c7a70ca2ec18e8b8f334b5bd64"
META, KEY, GROUP = prior.META, prior.KEY, prior.GROUP
PHASES, BRANCHES, CATEGORIES = prior.PHASES, prior.BRANCHES, prior.CATEGORIES
GATES = admission.GATES
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
            *command.SPEEDS,
            *command.FLAGS,
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
sha, read, write, flag, step = (
    prior.sha,
    prior.read,
    prior.write,
    prior.flag,
    prior.step,
)
window = prior.window
probe = prior.prior.prior.probe


def contract(source):
    ready = source[
        source.index("            let recovery_bounds =") : source.index(
            "            if loss_aware && safety_paused && !joint_recovery {"
        )
    ]
    start = source.index("            let mut guard_triggered =")
    guard = command.extract(source[start:], "else if guard_active && !joint_pending {")
    joint = command.extract(source[start:], "if joint_pending {")
    for s in (
        "recovery_evidence + DT",
        "recovery_evidence + 1e-9 >= 1.0",
        "guard_release_evidence = 0.0;",
        "guard_release_evidence + 1e-9 >= 0.5",
    ):
        assert s in joint + guard
    assert "&& (!averaged_recovery || recovery_sample_count >= 10)" in ready
    return dict(
        window_and_margin=prior.contract(source),
        command=command.source_contract(source),
        admission_margin=admission.source_contract(),
        ready_sha256=hashlib.sha256(ready.encode()).hexdigest(),
        guard_timer_sha256=hashlib.sha256(guard.encode()).hexdigest(),
        joint_timer_sha256=hashlib.sha256(joint.encode()).hexdigest(),
    )


def check_context(old, snap):
    raw = snap["raw"]
    assert raw["speed_margin"] == old["logged_speed_margin"]
    assert raw["recovery_sample_count"] == str(len(snap["members"]))
    assert old["window_count"] == str(len(snap["members"]))
    assert snap["fields"]["window_last_clear_step"] == int(old["last_clear_step"])
    assert snap["fields"]["window_last_clear_reasons"] == old["last_clear_reasons"]
    for k in prior.RAW:
        assert snap["point"][k] == old["input_" + k]


def local_slot(old, e, snap, table):
    check_context(old, snap)
    raw, before_raw = snap["raw"], snap["before_raw"]
    n = int(old["source_step"])
    s = admission.decoded(raw, n, snap["last_observation_step"])
    before = {
        k: (
            flag(before_raw[k], raw=True)
            if k in ("safety_paused", "guard_active")
            else float(before_raw[k])
        )
        for k in BEFORE
    }
    phase = (
        "historical_context" if old["context_phase"] == "before_return" else "available"
    )
    decision = admission.decision(e, n, before, s, phase)
    domain = command.command(raw, table)
    gates = {g: decision["gate_" + g] for g in GATES}
    if decision["branch"] == "joint":
        gates["nominal"] = "pass" if domain["source_nominal_positive"] else "fail"
    failed = ";".join(g for g in GATES if gates[g] == "fail")
    unknown = ";".join(g for g in GATES if gates[g] == "unknown")
    certainty = (
        "not_applicable"
        if decision["branch"] == "not_applicable"
        else "fail" if failed else "unknown" if unknown else "pass"
    )
    if certainty in ("pass", "fail"):
        assert decision["ready_from_timer"] == (certainty == "pass")
    out = {"local_input_" + k: raw[k] for k in INPUTS}
    out.update({"local_prior_" + k: before_raw[k] for k in BEFORE})
    out.update(
        local_age_exact_s=s["age_exact_s"],
        local_age_exact_hex=s["age_exact_hex"],
        local_last_observation_step=snap["last_observation_step"],
        local_branch=decision["branch"],
        local_joint_pending=decision["joint_pending"],
        local_lattice_index=domain["lattice_index"],
        local_speed_cap_bits=domain["speed_cap_bits"],
        local_source_nominal_positive=domain["source_nominal_positive"],
        local_source_zero_reason=domain["source_zero_reason"],
        local_prior_certainty=decision["certainty"],
        local_source_certainty=certainty,
        local_failed_gates=failed,
        local_unknown_gates=unknown,
        local_original_reference_joined=old["context_phase"] != "before_return",
    )
    for g in GATES:
        out["local_prior_gate_" + g] = decision["gate_" + g]
        out["local_source_gate_" + g] = gates[g]
    for q, b in product(("clearance", "bounds", "nominal", "release"), ("lo", "hi")):
        out[f"local_{q}_gap_{b}"] = decision[f"{q}_gap_{b}"]
    for k in (
        "timer_before_ticks",
        "timer_after_ticks",
        "threshold_ticks",
        "ready_from_timer",
        "timer_action",
    ):
        out["local_" + k] = decision[k]
    assert set(out) == set(SLOT_FIELDS)
    return out


def join_original(row, old):
    if not row["local_original_reference_joined"]:
        return
    key = (row["event_id"], int(row["source_step"]))
    d = old[key]
    assert d["case_id"] == row["case_id"] and d["phase"] == "available"
    for g in GATES:
        field = "source_nominal_gate" if g == "nominal" else "gate_" + g
        assert row["local_source_gate_" + g] == d[field], (key, g)
    for a, b in (
        ("source_certainty", "refined_certainty"),
        ("failed_gates", "refined_failed_gates"),
        ("unknown_gates", "refined_unknown_gates"),
        ("branch", "branch"),
        ("source_zero_reason", "source_zero_reason"),
        ("timer_action", "timer_action"),
    ):
        assert row["local_" + a] == d[b], (key, a)
    for k in ("source_nominal_positive", "ready_from_timer"):
        assert row["local_" + k] == (flag(d[k]) if d[k] else None)
    for k in ("timer_before_ticks", "timer_after_ticks"):
        assert row["local_" + k] == (int(d[k]) if d[k] else None)


def ready_runs(d, slots):
    runs = []
    grouped = []
    for s in slots:
        ready = s["local_audited"] and s["local_source_certainty"] == "pass"
        if not ready:
            continue
        if (
            grouped
            and int(s["offset_steps"]) == int(grouped[-1][-1]["offset_steps"]) + 1
            and s["local_branch"] == grouped[-1][-1]["local_branch"]
            and grouped[-1][-1]["local_timer_action"] != "release"
        ):
            grouped[-1].append(s)
        else:
            grouped.append([s])
    db = {int(s["offset_steps"]): s for s in slots}
    for i, rows in enumerate(grouped, 1):
        first, last = rows[0], rows[-1]
        a, b = int(first["offset_steps"]), int(last["offset_steps"])
        neighbor = lambda off: (
            db[off]["local_source_certainty"]
            if off in db and db[off]["local_audited"]
            else None
        )
        runs.append(
            dict(
                **{k: first[k] for k in (*META, "event_id", "reference_time_s")},
                run=i,
                branch=first["local_branch"],
                start_offset_steps=a,
                end_offset_steps=b,
                start_time_s=float(first["source_time_s"]),
                end_time_s=float(last["source_time_s"]),
                checkpoint_count=len(rows),
                checkpoint_span_s=(len(rows) - 1) / 50,
                evidence_added_s=len(rows) / 50,
                timer_before_ticks=first["local_timer_before_ticks"],
                timer_after_ticks=last["local_timer_after_ticks"],
                threshold_ticks=first["local_threshold_ticks"],
                observed_release=any(
                    s["local_timer_action"] == "release" for s in rows
                ),
                touches_before_return=any(
                    s["context_phase"] == "before_return" for s in rows
                ),
                left_context_limited=a == -25 or not db[a - 1]["local_audited"],
                right_context_limited=b == 25 or not db[b + 1]["local_audited"],
                predecessor_certainty=neighbor(a - 1),
                successor_certainty=neighbor(b + 1),
            )
        )
    assert sum(r["checkpoint_count"] for r in runs) == sum(
        s["local_audited"] and s["local_source_certainty"] == "pass" for s in slots
    )
    return runs


def metrics(d, slots, runs):
    out = dict.fromkeys(FIELDS)
    observed = [s for s in slots if s["local_audited"]]
    for side, rows in (
        ("pre", [s for s in slots if int(s["offset_steps"]) < 0]),
        ("post", [s for s in slots if int(s["offset_steps"]) > 0]),
    ):
        counts = Counter(
            (
                "missing"
                if not s["local_audited"]
                else (
                    "not_applicable"
                    if s["local_branch"] == "not_applicable"
                    else s["local_branch"] + "_" + s["local_source_certainty"]
                )
            )
            for s in rows
        )
        out.update({f"local_{side}_{k}_slots": counts[k] for k in CLASSES})
    for kind in (
        "raw_clear_nominal_zero",
        "raw_clear_not_ready",
        "raw_clear_ready",
        "mean_clear_not_ready",
    ):
        count = 0
        for s in observed:
            clear = (
                s["mean_state" if kind.startswith("mean") else "raw_state"] == "clear"
            )
            condition = (
                not s["local_source_nominal_positive"]
                if kind.endswith("nominal_zero")
                else s["local_ready_from_timer"]
                is (True if kind == "raw_clear_ready" else False)
            )
            count += clear and condition
        out["local_" + kind + "_slots"] = count
    post = [s for s in observed if int(s["offset_steps"]) > 0]
    out["local_has_post_reset"] = any(s["local_timer_action"] == "reset" for s in post)
    out["local_has_post_known_pass"] = any(
        s["local_source_certainty"] == "pass" for s in post
    )
    for branch in BRANCHES:
        rr = [s for s in observed if s["local_branch"] == branch]
        pp = [s for s in post if s["local_branch"] == branch]
        out[f"local_{branch}_max_known_pass_ticks"] = max(
            (r["checkpoint_count"] for r in runs if r["branch"] == branch), default=0
        )
        out[f"local_{branch}_post_max_known_pass_ticks"] = max(
            (
                max(0, r["end_offset_steps"] - max(1, r["start_offset_steps"]) + 1)
                for r in runs
                if r["branch"] == branch
            ),
            default=0,
        )
        out[f"local_{branch}_max_before_ticks"] = max(
            (s["local_timer_before_ticks"] for s in rr), default=None
        )
        out[f"local_{branch}_max_after_ticks"] = max(
            (s["local_timer_after_ticks"] for s in rr), default=None
        )
        out[f"local_{branch}_post_max_after_ticks"] = max(
            (s["local_timer_after_ticks"] for s in pp), default=None
        )
    offset = d["history_raw_first_later_clear_offset_steps"]
    if offset != "":
        chosen = next(s for s in slots if int(s["offset_steps"]) == int(offset))
        assert chosen["local_audited"] and chosen["raw_state"] == "clear"
        for k in FIRST:
            field = (
                "local_source_" + k
                if k in ("nominal_positive", "certainty")
                else "local_" + k
            )
            out["local_first_clear_" + k] = chosen[field]
    return out


def collect(cases, planned, raw, roots):
    files = defaultdict(dict)
    wanted = {
        (s["case_id"], int(s["source_step"]))
        for s in planned
        if s["slot_status"] == "observed"
    }
    selected_cases = {c for c, n in wanted}
    for c in cases:
        files[c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"][
            c["scenario"], c["planner"]
        ] = c
    assert set(files) == {(c, n) for c, s in raw.items() for n in s["raw_sha256"]}
    snapshots = {}
    audit = Counter()
    for i, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert sha(path) == raw[cohort]["raw_sha256"][name]
        audit["raw_bytes"] += path.stat().st_size
        last = {}
        before = {}
        windows = {}
        with path.open(newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
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
            for cells in reader:
                assert len(cells) == len(header)
                identity = cells[pos["scenario"]], cells[pos["planner"]]
                assert identity in members
                c = members[identity]
                n = step(cells[pos["time_s"]])
                assert n == last.get(identity, -1) + 1
                last[identity] = n
                if not flag(c["selected_timeout"]):
                    continue
                audit["timeout_grid_rows"] += 1
                if n == 0:
                    probe.noisy_branch({k: cells[j] for k, j in pos.items()})
                    audit["noisy_branch_histories"] += 1
                if c["case_id"] not in selected_cases:
                    continue
                row = dict(zip(header, cells))
                if n == 0:
                    windows[identity] = window.Window(row)
                    audit["replayed_histories"] += 1
                else:
                    w = windows[identity]
                    point = w.update(n, before[identity], row)
                    audit["replayed_steps"] += 1
                    if (c["case_id"], n) in wanted:
                        fields, _ = window.reconstruct(w, point, row)
                        snapshots[c["case_id"], n] = dict(
                            point=point,
                            members=list(w.members),
                            fields=fields,
                            raw=row,
                            before_raw=before[identity],
                            last_observation_step=w.last_observation,
                        )
                before[identity] = row
        assert set(last) == set(members)
        for identity, c in members.items():
            assert last[identity] == (
                9000 if flag(c["selected_timeout"]) else step(c["time_s"])
            )
        audit["raw_files"] += 1
        if i % 18 == 0:
            print(f"Local admission contexts read: {i}/108 files", flush=True)
    assert snapshots.keys() == wanted
    audit["unique_context_points"] = len(snapshots)
    return snapshots, dict(audit)


def anchor(e, dd, ne, nr):
    out = dict(
        e, **prior.prior.prior.weights(e, ne, nr), **dict.fromkeys((*REFS, *HAS))
    )
    if not flag(e["returned"] or "False"):
        assert not dd
        return out
    assert all(all(d[k] == e[k] for k in META) for d in dd)
    counts = Counter()
    for d in dd:
        if not d["local_audited"]:
            continue
        kinds = ["audited"]
        if d["local_first_clear_certainty"] is not None:
            kinds.extend(
                (
                    "first_clear_audited",
                    "first_clear_" + d["local_first_clear_certainty"],
                    (
                        "first_clear_nominal_positive"
                        if d["local_first_clear_nominal_positive"]
                        else "first_clear_nominal_zero"
                    ),
                )
            )
        if d["local_raw_clear_nominal_zero_slots"]:
            kinds.append("has_raw_clear_nominal_zero")
        if d["local_has_post_reset"]:
            kinds.append("has_post_reset")
        if d["local_has_post_known_pass"]:
            kinds.append("has_post_known_pass")
        for k in kinds:
            counts[f"{d['phase']}_{d['branch']}_local_{k}_refs"] += 1
    out.update({k: counts[k] for k in REFS})
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
    db = {tuple(r[k] for k in GROUP): r for r in rows}
    assert len(db) == len(rows)
    checks = 0
    for key, row in db.items():
        for i, whole, parts in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", CATEGORIES),
        ):
            if key[i] == whole:
                for field in (
                    *REFS,
                    *(h + s for h in HAS for s in ("_events", "_case_mass")),
                ):
                    assert row[field] == sum(
                        db[key[:i] + (p,) + key[i + 1 :]][field] for p in parts
                    )
                    checks += 1
        for p, b in product(PHASES, BRANCHES):
            pre = f"{p}_{b}_local_"
            assert row[pre + "first_clear_audited_refs"] == sum(
                row[pre + "first_clear_" + s + "_refs"] for s in STATES
            )
            assert (
                row[pre + "first_clear_audited_refs"]
                == row[pre + "first_clear_nominal_positive_refs"]
                + row[pre + "first_clear_nominal_zero_refs"]
            )
    return checks


def summaries(old, anchors):
    rows = []
    for fixed in old:
        row = dict(fixed)
        p, n, arm, mode, category = (row[k] for k in GROUP)
        aa = [
            a
            for a in anchors
            if (a["partition"], a["noise"], a["variant"]) == (p, n, arm)
            and (mode == "all_modes" or a["mode"] == mode)
            and (category == "all" or a["anchor_category"] == category)
        ]
        returned = [a for a in aa if flag(a["returned"] or "False")]
        mass = lambda rr: sum((a["census_case_weight"] for a in rr), Fraction())
        row.update({k: sum(a[k] or 0 for a in aa) for k in REFS})
        for name in HAS:
            selected = [a for a in aa if a[name]]
            row.update(
                {
                    name + "_events": len(selected),
                    name + "_case_mass": mass(selected),
                    name
                    + "_fraction": (
                        Fraction(len(selected), len(returned)) if returned else None
                    ),
                    name
                    + "_case_mass_fraction": (
                        mass(selected) / mass(returned) if returned else None
                    ),
                }
            )
        assert math.isclose(
            float(fixed["census_case_mass"]), float(mass(aa)), abs_tol=2e-12
        )
        rows.append(row)
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--raw-roots", type=Path)
    args = p.parse_args()
    if not __debug__ or args.output.exists():
        p.error("Use a new output directory without -O")
    freeze = json.loads((STUDY / "freeze-pins.json").read_text())
    assert freeze["sources_sha256"] == sha(BASE / "sources.json") == BASE_SHA
    assert (
        freeze["proof_sha256"]
        == sha(BASE / "independent-verification.json")
        == PROOF_SHA
    )
    source = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    pins = {
        **source["input_sha256"],
        **{
            "fusion-return-point-history/" + n: h
            for n, h in source["output_sha256"].items()
        },
        "fusion-return-point-history/sources.json": BASE_SHA,
        "fusion-return-point-history/independent-verification.json": PROOF_SHA,
        "fusion-return-point-history/protocol.md": source["analysis"][
            "protocol_sha256"
        ],
        "fusion-return-point-history/freeze-pins.json": source["analysis"][
            "freeze_sha256"
        ],
    }
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_point_history.py": source["analysis"]["script_sha256"],
    }
    vhelpers = {
        **source["analysis"]["verifier_helpers_sha256"],
        "verify_point_history.py": freeze["verifier_helper_sha256"],
    }
    vhelpers.update(
        {
            "verify_command_sign.py": freeze["command_verifier_sha256"],
            "verify_return_admission.py": freeze["admission_verifier_sha256"],
        }
    )
    for n, h in pins.items():
        assert sha(RESULTS / n) == h, n
    for n, h in {**helpers, **vhelpers}.items():
        assert sha(ROOT / n) == h, n
    for n, h in source["analysis"]["code_sha256"].items():
        assert sha(REPO / n) == h, n
    source_contract = contract(
        (REPO / next(iter(source["analysis"]["code_sha256"]))).read_text()
    )
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    probe.provenance(source["raw"], roots, proof["native"]["rustc"])
    cases = read(RESULTS / "census-rp-selection/case-selection.csv")
    old = read(BASE / "history-decisions.csv")
    selected = [d for d in old if d["prior_certainty"] == "unknown"]
    assert len(selected) == 25 and all(
        d["phase"] == "available" and d["branch"] == "joint" for d in selected
    )
    events = read(BASE / "anchor-history.csv")
    event_map = {e["event_id"]: e for e in events}
    assert len(event_map) == len(events)
    planned = read(BASE / "history-slots.csv")
    assert len(planned) == 1275
    snapshots, audit = collect(cases, planned, source["raw"], roots)
    original = {(d["event_id"], step(d["time_s"])): d for d in old}
    table = command.lattice()
    slots = []
    by_reference = defaultdict(list)
    for s in planned:
        audited = s["slot_status"] == "observed"
        row = dict(s, local_audited=audited, **dict.fromkeys(SLOT_FIELDS))
        if audited:
            row.update(
                local_slot(
                    s,
                    event_map[s["event_id"]],
                    snapshots[s["case_id"], int(s["source_step"])],
                    table,
                )
            )
            join_original(row, original)
        slots.append(row)
        by_reference[s["event_id"], s["reference_time_s"]].append(row)
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
    ne = Counter(e["case_id"] for e in events)
    nr = Counter(e["case_id"] for e in events if flag(e["received"]))
    db = defaultdict(list)
    for d in decisions:
        db[d["event_id"]].append(d)
    anchors = [
        anchor(e, db[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]]) for e in events
    ]
    summary = summaries(read(BASE / "history-summary.csv"), anchors)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(flag(e["returned"] or "False") for e in events),
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
    assert (
        len(cases),
        len(ne),
        len(anchors),
        len(decisions),
        len(summary),
        audit["replayed_histories"],
        audit["replayed_steps"],
    ) == (972, 570, 3997, 3751, 336, 4, 36000)
    assert (
        sum(a["census_case_weight"] for a in anchors)
        == sum(a["received_case_weight"] for a in anchors)
        == 570
    )
    assert (
        sum(a["rp_case_mass"] for a in anchors) == 157
        and sum(int(a["rp_records"]) for a in anchors) == 10995
    )
    args.output.mkdir(parents=True)
    for name, rows in (
        ("local-decisions", decisions),
        ("ready-runs", runs),
        ("local-slots", slots),
        ("anchor-local", anchors),
        ("local-summary", summary),
    ):
        write(args.output / (name + ".csv"), rows)
    out = dict(
        input_sha256=pins,
        raw=source["raw"],
        map_sha256=source["map_sha256"],
        analysis=dict(
            script_sha256=sha(Path(__file__)),
            protocol_sha256=sha(STUDY / "protocol.md"),
            freeze_sha256=sha(STUDY / "freeze-pins.json"),
            helpers_sha256=helpers,
            verifier_helpers_sha256=vhelpers,
            code_sha256=source["analysis"]["code_sha256"],
            source_contract=source_contract,
        ),
        schema=dict(
            half_unit=admission.HALF,
            elemental_allowance=admission.PAD,
            gates=GATES,
            states=STATES,
            classes=CLASSES,
            phases=PHASES,
            branches=BRANCHES,
            retrospective=True,
            inherited="all prior fields/point gaps/outcomes/weights; same 25 histories/slots; nominal source domain independent of timers; no new rollout or follow-up",
        ),
        inherited_point_history_proof_sha256=PROOF_SHA,
        inherited_validation=source["inherited_validation"],
        audit=audit,
        output_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
