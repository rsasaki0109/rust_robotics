#!/usr/bin/env python3
"""Audit original return admission with conservative rounded-input intervals."""

import argparse
from collections import Counter, defaultdict
import csv
from fractions import Fraction
import hashlib
from itertools import product
import json
import math
from pathlib import Path
import re

import analyze_return_controls as previous

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
REPO = ROOT.parents[1]
BASE = RESULTS / "fusion-return-controls"
BASE_SHA = "d4805319372227496c5b6fece67644ee1d16b04e16cbf9fd846281929508ddf5"
PROOF_SHA = "61283776151e7bad7af977ff293b0c787b5e8069d806e66fe68af29f7bb38bcb"
PROTOCOL = RESULTS / "fusion-return-admission/protocol.md"
HELPERS = {
    "verify_epoch_retention.py": "417a3e93cc4680e77722fe2c95e24f206d532399bd4e2efc8d9951a24530e9f1"
}
META, KEY = previous.META, previous.KEY
digest, flag, step, indexed, read_rows, write = (
    previous.digest,
    previous.flag,
    previous.step,
    previous.indexed,
    previous.read_rows,
    previous.write,
)
CATEGORIES = previous.CATEGORIES
GROUP = ("partition", "noise", "variant", "mode", "category_selector")
GATES = ("age", "stationary", "samples", "clearance", "bounds", "nominal", "release")
BRANCH_GATES = {"joint": GATES, "guard": ("stationary", "release")}
STATES = ("pass", "fail", "unknown")
PHASES = ("available", "next_loss")
HALF, PAD = 5e-10, 1e-12
BOOLS = (
    "correction",
    "correction_received",
    "precision_activation",
    "safety_paused",
    "guard_active",
    "resumed",
    "guard_triggered",
    "guard_released",
    "stop_latched",
    "epoch_fusion_available",
    "stationary_epoch_fusion",
)
FLOATS = (
    "measured_speed",
    "observation_age",
    "recovery_evidence",
    "guard_release_evidence",
    "speed_margin",
    "recovery_clearance",
    "recovery_x",
    "recovery_y",
    "recovery_yaw",
    "nominal_target_speed",
    "target_speed",
    "release_preview_clearance_lower_bound",
    "guard_release_margin",
)
BASE_FIELDS = (
    *META,
    "event_id",
    "source_event_id",
    "event_kind",
    "event_time_s",
    "scheduled",
    "received",
    "precision_activation",
    "horizon",
    "preview_loss",
    "loss_operation",
    "loss_time_s",
    "returned",
    "return_time_s",
    "return_wait_s",
    "censor_wait_lower_bound_s",
    "returned_spell_span_s",
    "ended",
    "end_time_s",
    "end_operation",
    "censored_return",
    "no_return",
    "ttl_limit_without_next_step",
    "parent_rp_records",
    "rp_records",
    "rp_different_preview_records",
)
REFS = (
    "decision_references",
    "run_count",
    *(phase + "_references" for phase in PHASES),
    *(
        phase + "_" + name + "_references"
        for phase in PHASES
        for name in (
            "joint",
            "guard",
            "ready",
            "certainty_pass",
            "certainty_fail",
            "certainty_unknown",
            "accumulate",
            "reset",
            "stay_zero",
            "release",
        )
    ),
    *(
        "available_" + branch + "_" + gate + "_" + state + "_references"
        for branch, gates in BRANCH_GATES.items()
        for gate in gates
        for state in STATES
    ),
)
NEW = (
    *REFS,
    "return_branch",
    "return_failed_gates",
    "return_unknown_gates",
    "next_loss_failed_gates",
    "initial_counter_ticks",
    "max_available_counter_ticks",
    "has_ready_tick",
    "has_timer_reset",
    "has_unknown_readiness",
)
RUN_KEY = (
    "branch",
    "phase",
    "certainty",
    "ready_from_timer",
    "failed_gates",
    "unknown_gates",
    "timer_action",
)


def source_contract():
    source = (
        REPO / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
    ).read_text()
    assert (
        "let observation_time = step as f64 * DT;" in source
        and "const DT: f64 = 0.02;" in source
    )
    a = re.search(r"let recovery_margin = (.*?);", source, re.S).group(1)
    b = re.search(
        r"let speed_margin = if anticipatory_speed \{(.*?)\} else", source, re.S
    ).group(1)
    clean = lambda s: re.sub(r"\s+", "", s)
    assert clean(a) == clean(b)
    return hashlib.sha256(clean(a).encode()).hexdigest()


def interval(x):
    return x - HALF - PAD, x + HALF + PAD


def difference(a, b):
    return a[0] - b[1] - PAD, a[1] - b[0] + PAD


def compare_interval(ab, inclusive=False):
    lo, hi = ab
    assert math.isfinite(lo) and math.isfinite(hi) and lo <= hi
    if lo >= 0 if inclusive else lo > 0:
        return "pass"
    if hi < 0 if inclusive else hi <= 0:
        return "fail"
    return "unknown"


def condition_intervals(s):
    margin = interval(s["speed_margin"])
    x, y, yaw = s["recovery_x"], s["recovery_y"], s["recovery_yaw"]
    coords = [interval(x), interval(y)]
    error = (HALF + PAD) * 1.6 + PAD
    coords += [
        (x + 0.6 * math.cos(yaw) - error, x + 0.6 * math.cos(yaw) + error),
        (y + 0.6 * math.sin(yaw) - error, y + 0.6 * math.sin(yaw) + error),
    ]
    gaps = [difference(c, margin) for c in coords]
    gaps += [(50 - margin[1] - c[1] - PAD, 50 - margin[0] - c[0] + PAD) for c in coords]
    bounds = min(g[0] for g in gaps), min(g[1] for g in gaps)
    nominal = (
        (0.0, 0.0)
        if s["stop_latched"]
        else (
            max(0.0, interval(s["nominal_target_speed"])[0]),
            interval(s["nominal_target_speed"])[1],
        )
    )
    if s["stop_latched"]:
        assert s["nominal_target_speed"] == 0
    return dict(
        clearance=difference(interval(s["recovery_clearance"]), margin),
        bounds=bounds,
        nominal=nominal,
        release=difference(
            interval(s["release_preview_clearance_lower_bound"]),
            interval(s["guard_release_margin"]),
        ),
    )


def ticks(value):
    n = round(value * 50)
    assert 0 <= n <= 49 and abs(value - n / 50) <= 5e-7
    return n


def transition(before, current, branch):
    joint_before, guard_before = ticks(before["recovery_evidence"]), ticks(
        before["guard_release_evidence"]
    )
    joint_after, guard_after = ticks(current["recovery_evidence"]), ticks(
        current["guard_release_evidence"]
    )
    assert guard_before < 25 and guard_after < 25
    if branch == "not_applicable":
        return dict(
            timer_before_ticks=None,
            timer_after_ticks=None,
            threshold_ticks=None,
            ready_from_timer=None,
            timer_action="not_applicable",
        )
    joint = branch == "joint"
    prior, after, limit = (
        (joint_before, joint_after, 50) if joint else (guard_before, guard_after, 25)
    )
    if joint and not before["safety_paused"]:
        prior = 0  # Starting a new loss pause resets this counter before admission.
    released = current["resumed"] if joint else current["guard_released"]
    if released:
        assert prior + 1 == limit and after == 0
        ready, action = True, "release"
    elif after:
        assert after == prior + 1 and after < limit
        ready, action = True, "accumulate"
    else:
        ready, action = False, "reset" if prior else "stay_zero"
    if joint:
        assert guard_after == 0 and current["safety_paused"] == (not released)
        assert current["guard_released"] == (
            released and (before["guard_active"] or current["guard_triggered"])
        )
        assert current["guard_active"] == (
            not released and (before["guard_active"] or current["guard_triggered"])
        )
    else:
        assert not current["resumed"] and not current["safety_paused"]
        assert (
            current["guard_active"] == (not released) and not current["guard_triggered"]
        )
        assert joint_after == joint_before
    return dict(
        timer_before_ticks=prior,
        timer_after_ticks=after,
        threshold_ticks=limit,
        ready_from_timer=ready,
        timer_action=action,
    )


def decision(e, n, before, s, phase):
    pending = before["safety_paused"] or (
        s["correction"] and not s["correction_received"]
    )
    branch = (
        "joint" if pending else "guard" if before["guard_active"] else "not_applicable"
    )
    envelopes = condition_intervals(s)
    gates = dict.fromkeys(GATES, "not_applicable")
    if branch != "not_applicable":
        gates["stationary"] = "pass" if abs(s["measured_speed"]) <= 0.02 else "fail"
        gates["release"] = compare_interval(envelopes["release"])
    if branch == "joint":
        gates.update(
            age="pass" if s["age_exact_s"] <= 1.5 else "fail",
            samples="pass" if s["recovery_sample_count"] >= 10 else "fail",
            **{
                k: compare_interval(envelopes[k], inclusive=k == "bounds")
                for k in ("clearance", "bounds", "nominal")
            },
        )
    failed = ";".join(k for k in GATES if gates[k] == "fail")
    unknown = ";".join(k for k in GATES if gates[k] == "unknown")
    certainty = (
        "not_applicable"
        if branch == "not_applicable"
        else "fail" if failed else "unknown" if unknown else "pass"
    )
    timer = transition(before, s, branch)
    if certainty in ("pass", "fail"):
        assert timer["ready_from_timer"] == (certainty == "pass"), (
            e["event_id"],
            n,
            gates,
            timer,
        )
    common = {k: e[k] for k in (*META, "event_id", "source_event_id")}
    return dict(
        **common,
        time_s=n / 50,
        offset_from_return_ms=(n - step(e["return_time_s"])) * 20,
        phase=phase,
        prior_safety_paused=before["safety_paused"],
        prior_guard_active=before["guard_active"],
        joint_pending=pending,
        branch=branch,
        **s,
        **{"gate_" + k: v for k, v in gates.items()},
        **{
            name + "_gap_" + side: v
            for name, pair in envelopes.items()
            for side, v in zip(("lo", "hi"), pair)
        },
        certainty=certainty,
        failed_gates=failed,
        unknown_gates=unknown,
        **timer,
    )


def decoded(row, n, last):
    out = {k: flag(row[k], raw=True) for k in BOOLS}
    for k in FLOATS:
        out[k] = float(row[k])
        assert math.isfinite(out[k])
    out["recovery_sample_count"] = int(row["recovery_sample_count"])
    assert (
        str(out["recovery_sample_count"]) == row["recovery_sample_count"]
        and 0 <= out["recovery_sample_count"] <= 25
    )
    age = n * 0.02 - last * 0.02
    assert format(age, ".6f") == row["observation_age"]
    out.update(age_exact_s=age, age_exact_hex=age.hex(), last_observation_step=last)
    assert out["stationary_epoch_fusion"] and out["speed_margin"] > 0
    return out


def collect(cases, events, raw, roots):
    files, wanted, at = defaultdict(dict), defaultdict(set), {}
    for c in cases:
        files[
            (c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv")
        ][(c["scenario"], c["planner"])] = c
    for e in events:
        at[e["case_id"], step(e["event_time_s"])] = e
        w = previous.bounds(e)
        if w:
            wanted[e["case_id"]].update(range(w[0], w[1] + 1))
    assert len(at) == len(events)
    assert set(files) == {(p, n) for p, s in raw.items() for n in s["raw_sha256"]}
    points, audit = defaultdict(dict), Counter()
    for i, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert digest(path) == raw[cohort]["raw_sha256"][name]
        audit["raw_bytes"] += path.stat().st_size
        last_step, observations = {}, defaultdict(int)
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
                    "recovery_sample_count",
                    *BOOLS,
                    *FLOATS,
                )
            }
            for cells in reader:
                assert len(cells) == len(header)
                identity = cells[pos["scenario"]], cells[pos["planner"]]
                assert identity in members
                c = members[identity]
                cid = c["case_id"]
                n = step(cells[pos["time_s"]])
                assert n == last_step.get(identity, -1) + 1
                last_step[identity] = n
                if not flag(c["selected_timeout"]):
                    continue
                audit["timeout_grid_rows"] += 1
                if n == 0:
                    continue
                received = flag(cells[pos["correction_received"]], raw=True)
                precision = flag(cells[pos["precision_activation"]], raw=True)
                if received or precision:
                    observations[cid] = n
                age = n * 0.02 - observations[cid] * 0.02
                assert format(age, ".6f") == cells[pos["observation_age"]], (
                    cid,
                    n,
                    age,
                    cells[pos["observation_age"]],
                )
                audit["source_clock_decisions"] += 1
                if (cid, n) in at:
                    e = at[cid, n]
                    assert (
                        received,
                        precision,
                        flag(cells[pos["correction"]], raw=True),
                    ) == (
                        flag(e["received"]),
                        flag(e["precision_activation"]),
                        flag(e["scheduled"]),
                    )
                    assert (
                        "recovery_mean"
                        if int(cells[pos["recovery_sample_count"]])
                        else "raw_fallback"
                    ) == e["immediate_actual"]
                    assert flag(cells[pos["epoch_fusion_available"]], raw=True) == (
                        e["immediate_preview"] == "three_epoch"
                    )
                    audit["census_joins"] += 1
                if n in wanted[cid]:
                    points[cid][n] = decoded(
                        {k: cells[v] for k, v in pos.items()}, n, observations[cid]
                    )
        assert set(last_step) == set(members)
        for identity, c in members.items():
            assert last_step[identity] == (
                9000 if flag(c["selected_timeout"]) else step(c["time_s"])
            )
        audit["raw_files"] += 1
        if i % 12 == 0:
            print(f"Original admission inputs read: {i}/108 files", flush=True)
    assert all(set(points[cid]) == ns for cid, ns in wanted.items())
    return points, dict(audit)


def prior_controls(events, points):
    indexed(events, ("event_id",))
    by_event = {e["event_id"]: e for e in events}
    checks = 0
    for s in read_rows(BASE / "control-snapshots.csv"):
        p = points[s["case_id"]][step(s["time_s"])]
        for k in (*BOOLS, *FLOATS, "recovery_sample_count"):
            if k not in s:
                continue
            value = p[k]
            assert (
                flag(s[k]) == value if isinstance(value, bool) else float(s[k]) == value
            )
            checks += 1
    for r in read_rows(BASE / "control-runs.csv"):
        assert all(r[k] == by_event[r["event_id"]][k] for k in META)
        for n in range(step(r["start_time_s"]), step(r["end_time_s"]) + 1):
            s = points[r["case_id"]][n]
            expected = dict(
                pose_branch=(
                    "recovery_mean" if s["recovery_sample_count"] else "raw_fallback"
                ),
                preview_branch=(
                    "three_epoch" if s["epoch_fusion_available"] else "motion_filter"
                ),
                safety_paused=s["safety_paused"],
                guard_active=s["guard_active"],
                resumed=s["resumed"],
                target_positive=s["target_speed"] > 0,
            )
            for k, v in expected.items():
                assert r[k] == str(v)
                checks += 1
    return checks


def build(e, points, ne, nr):
    out = {
        k: None if e[k] == "" else flag(e[k]) if e[k] in ("True", "False") else e[k]
        for k in BASE_FIELDS
    }
    out["anchor_category"] = e["anchor_category"]
    parent = int(e["parent_rp_records"])
    for k in ("parent_rp_records", "rp_records", "rp_different_preview_records"):
        out[k] = int(e[k])
    out.update(
        census_case_weight=Fraction(1, ne),
        received_case_weight=Fraction(1, nr) if flag(e["received"]) else Fraction(),
        rp_case_mass=Fraction(out["rp_records"], parent) if parent else Fraction(),
        rp_different_preview_case_mass=(
            Fraction(out["rp_different_preview_records"], parent)
            if parent
            else Fraction()
        ),
    )
    out.update(dict.fromkeys(NEW))
    if not flag(e["returned"] or "False"):
        return out, [], []
    start = step(e["return_time_s"])
    end = step(e["end_time_s"]) if flag(e["ended"]) else 9000
    decisions = [
        decision(
            e,
            n,
            points[n - 1],
            points[n],
            "next_loss" if flag(e["ended"]) and n == end else "available",
        )
        for n in range(start, end + 1)
    ]
    assert all(r["branch"] in BRANCH_GATES for r in decisions)
    encoded = []
    for r in decisions:
        key = tuple(r[k] for k in RUN_KEY)
        n = step(r["time_s"])
        if encoded and encoded[-1][2] == key:
            encoded[-1] = encoded[-1][0], n, key
        else:
            encoded.append((n, n, key))
    common = {k: e[k] for k in (*META, "event_id", "source_event_id")}
    runs = [
        dict(
            **common,
            run=i,
            start_time_s=lo / 50,
            end_time_s=hi / 50,
            checkpoint_count=hi - lo + 1,
            **dict(zip(RUN_KEY, key)),
        )
        for i, (lo, hi, key) in enumerate(encoded, 1)
    ]
    out.update({k: 0 for k in REFS})
    out.update(decision_references=len(decisions), run_count=len(runs))
    for r in decisions:
        phase, branch = r["phase"], r["branch"]
        out[phase + "_references"] += 1
        for name in (branch, "certainty_" + r["certainty"], r["timer_action"]):
            out[phase + "_" + name + "_references"] += 1
        out[phase + "_ready_references"] += r["ready_from_timer"]
        if phase == "available":
            for gate in BRANCH_GATES[branch]:
                out[
                    "available_"
                    + branch
                    + "_"
                    + gate
                    + "_"
                    + r["gate_" + gate]
                    + "_references"
                ] += 1
    available = [r for r in decisions if r["phase"] == "available"]
    out.update(
        return_branch=decisions[0]["branch"],
        return_failed_gates=decisions[0]["failed_gates"],
        return_unknown_gates=decisions[0]["unknown_gates"],
        next_loss_failed_gates=(
            decisions[-1]["failed_gates"] if flag(e["ended"]) else None
        ),
        initial_counter_ticks=decisions[0]["timer_before_ticks"],
        max_available_counter_ticks=max(r["timer_after_ticks"] for r in available),
        has_ready_tick=any(r["ready_from_timer"] for r in available),
        has_timer_reset=any(r["timer_action"] == "reset" for r in available),
        has_unknown_readiness=any(r["certainty"] == "unknown" for r in available),
    )
    return out, decisions, runs


def summaries(cases, anchors):
    rows = []
    for p, n, a, m, cat in product(
        ("known", "new", "fresh"),
        ("white", "correlated"),
        ("epoch", "curved"),
        ("none", "random", "blackout", "all_modes"),
        ("all", *CATEGORIES),
    ):
        accept = lambda r: (r["partition"], r["noise"], r["variant"]) == (p, n, a) and (
            m == "all_modes" or r["mode"] == m
        )
        cc = [c for c in cases if accept(c)]
        rr = [
            r
            for r in anchors
            if accept(r) and (cat == "all" or r["anchor_category"] == cat)
        ]
        mass = lambda rs, k="census_case_weight": sum((r[k] for r in rs), Fraction())
        returned = [r for r in rr if r["returned"]]
        out = dict(zip(GROUP, (p, n, a, m, cat)))
        out.update(
            planned_cases=len(cc),
            timeout_cases=sum(flag(c["selected_timeout"]) for c in cc),
            held_cases=sum(flag(c["held"]) for c in cc),
            census_cases=len({r["case_id"] for r in rr}),
            returned_cases=len({r["case_id"] for r in returned}),
            census_events=len(rr),
            census_case_mass=mass(rr),
            received_case_mass=mass(rr, "received_case_weight"),
            rp_records=sum(r["rp_records"] for r in rr),
            rp_different_preview_records=sum(
                r["rp_different_preview_records"] for r in rr
            ),
            rp_case_mass=mass(rr, "rp_case_mass"),
            rp_different_preview_case_mass=mass(rr, "rp_different_preview_case_mass"),
            max_available_counter_ticks=max(
                (r["max_available_counter_ticks"] for r in returned), default=None
            ),
        )
        for name in (
            "preview_loss",
            "returned",
            "no_return",
            "has_ready_tick",
            "has_timer_reset",
            "has_unknown_readiness",
        ):
            selected = [r for r in rr if r[name]]
            out[name + "_events"], out[name + "_case_mass"] = len(selected), mass(
                selected
            )
            if name.startswith("has_"):
                out[name + "_fraction"] = (
                    Fraction(len(selected), len(returned)) if returned else None
                )
                out[name + "_case_mass_fraction"] = (
                    mass(selected) / mass(returned) if returned else None
                )
        for k in REFS:
            out[k] = sum(r[k] or 0 for r in rr)
        rows.append(out)
    return rows


def closure(rows):
    lookup = indexed(rows, GROUP)
    checks = 0
    additive = [
        k
        for k in rows[0]
        if k in REFS or k.endswith(("_events", "_records", "_case_mass"))
    ]
    for r in rows:
        for field, whole, parts in (
            ("mode", "all_modes", ("none", "random", "blackout")),
            ("category_selector", "all", CATEGORIES),
        ):
            if r[field] != whole:
                continue
            children = [
                lookup[tuple(v if k == field else r[k] for k in GROUP)] for v in parts
            ]
            for k in additive:
                assert sum(c[k] for c in children) == r[k], (field, k)
                checks += 1
        assert r["decision_references"] == sum(r[p + "_references"] for p in PHASES)
        for phase in PHASES:
            assert (
                sum(r[phase + "_certainty_" + s + "_references"] for s in STATES)
                == r[phase + "_references"]
            )
            assert (
                sum(r[phase + "_" + b + "_references"] for b in BRANCH_GATES)
                == r[phase + "_references"]
            )
        for b, gates in BRANCH_GATES.items():
            for g in gates:
                assert (
                    sum(
                        r["available_" + b + "_" + g + "_" + s + "_references"]
                        for s in STATES
                    )
                    == r["available_" + b + "_references"]
                )
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--raw-roots", type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    assert (
        digest(BASE / "sources.json") == BASE_SHA
        and digest(BASE / "independent-verification.json") == PROOF_SHA
    )
    prior = json.loads((BASE / "sources.json").read_text())
    pins = {
        **prior["input_sha256"],
        "fusion-return-controls/sources.json": BASE_SHA,
        "fusion-return-controls/independent-verification.json": PROOF_SHA,
        "fusion-return-controls/protocol.md": prior["analysis"]["protocol_sha256"],
        **{"fusion-return-controls/" + k: h for k, h in prior["output_sha256"].items()},
    }
    helpers = {
        **prior["analysis"]["helpers_sha256"],
        "analyze_return_controls.py": prior["analysis"]["script_sha256"],
    }
    for name, h in pins.items():
        assert digest(RESULTS / name) == h, name
    for name, h in {**helpers, **HELPERS}.items():
        assert digest(ROOT / name) == h, name
    for name, h in prior["analysis"]["code_sha256"].items():
        assert digest(REPO / name) == h, name
    contract = source_contract()
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(prior["raw"])
    cases = read_rows(RESULTS / "census-rp-selection/case-selection.csv")
    events = read_rows(BASE / "anchor-controls.csv")
    indexed(cases, KEY)
    indexed(events, ("event_id",))
    points, audit = collect(cases, events, prior["raw"], roots)
    audit["preceding_control_comparisons"] = prior_controls(events, points)
    ne = Counter(e["case_id"] for e in events)
    nr = Counter(e["case_id"] for e in events if flag(e["received"]))
    anchors, decisions, runs = [], [], []
    for e in events:
        a, dd, rr = build(e, points[e["case_id"]], ne[e["case_id"]], nr[e["case_id"]])
        anchors.append(a)
        decisions.extend(dd)
        runs.extend(rr)
    summary = summaries(cases, anchors)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(bool(r["returned"]) for r in anchors),
        decision_references=len(decisions),
        available_references=sum(r["phase"] == "available" for r in decisions),
        next_loss_references=sum(r["phase"] == "next_loss" for r in decisions),
        admission_runs=len(runs),
        summary_rows=len(summary),
        additive_closure_comparisons=closure(summary),
    )
    assert (
        audit["raw_files"],
        audit["source_clock_decisions"],
        audit["timeout_grid_rows"],
        audit["census_joins"],
    ) == (108, 5130000, 5130570, 3997)
    assert (
        len(cases),
        audit["planned_families"],
        len(anchors),
        audit["returned_anchors"],
        len(decisions),
        audit["available_references"],
        len(summary),
    ) == (972, 324, 3997, 137, 3751, 3662, 336)
    assert (
        sum(r["census_case_weight"] for r in anchors)
        == sum(r["received_case_weight"] for r in anchors)
        == 570
    )
    assert (
        sum(r["rp_records"] for r in anchors) == 10995
        and sum(r["rp_case_mass"] for r in anchors) == 157
    )
    args.output.mkdir(parents=True)
    for name, rows in (
        ("anchor-admission", anchors),
        ("admission-decisions", decisions),
        ("admission-runs", runs),
        ("admission-summary", summary),
    ):
        write(args.output / (name + ".csv"), rows)
    source = dict(
        input_sha256=pins,
        raw=prior["raw"],
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=helpers,
            verifier_helpers_sha256=HELPERS,
            code_sha256=prior["analysis"]["code_sha256"],
            shared_margin_expression_sha256=contract,
        ),
        schema=dict(
            half_nine_decimal_unit=HALF,
            numerical_guard=PAD,
            gates=GATES,
            branch_gates=BRANCH_GATES,
            states=STATES,
            clock="binary64 step*0.02 minus last received/precision step*0.02; six-place age checked; exact hex saved",
            timer="observed branch recurrence with carried counters; conditions retain uncertainty independently",
            retrospective=True,
        ),
        inherited_validation=prior["inherited_validation"],
        inherited_control_proof_sha256=PROOF_SHA,
        audit=audit,
        output_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
