#!/usr/bin/env python3
"""Verify original admission with Decimal geometry, timer enumeration and SQL."""

import argparse
from collections import Counter, defaultdict
import decimal
from decimal import Decimal as D
from fractions import Fraction
import hashlib
from itertools import product
import json
import math
from pathlib import Path
import sqlite3

import verify_epoch_retention as core

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
BASE_SHA = "d4805319372227496c5b6fece67644ee1d16b04e16cbf9fd846281929508ddf5"
PROOF_SHA = "61283776151e7bad7af977ff293b0c787b5e8069d806e66fe68af29f7bb38bcb"
HELPERS = {
    "verify_epoch_retention.py": "417a3e93cc4680e77722fe2c95e24f206d532399bd4e2efc8d9951a24530e9f1"
}
META, KEY = core.META, core.KEY
CATEGORIES = (
    "ttl_ended",
    "age_ended",
    "returned_censored",
    "no_return",
    "unobserved_ttl_limit",
    "no_observed_loss",
)
GROUP = ("partition", "noise", "variant", "mode", "category_selector")
GATES = ("age", "stationary", "samples", "clearance", "bounds", "nominal", "release")
BRANCHES = {"joint": GATES, "guard": ("stationary", "release")}
STATES = ("pass", "fail", "unknown")
PHASES = ("available", "next_loss")
BOOLS = "correction correction_received precision_activation safety_paused guard_active resumed guard_triggered guard_released stop_latched epoch_fusion_available stationary_epoch_fusion".split()
FLOATS = "measured_speed observation_age recovery_evidence guard_release_evidence speed_margin recovery_clearance recovery_x recovery_y recovery_yaw nominal_target_speed target_speed release_preview_clearance_lower_bound guard_release_margin".split()
BASE_FIELDS = (
    *META,
    *"event_id source_event_id event_kind event_time_s scheduled received precision_activation horizon preview_loss loss_operation loss_time_s returned return_time_s return_wait_s censor_wait_lower_bound_s returned_spell_span_s ended end_time_s end_operation censored_return no_return ttl_limit_without_next_step parent_rp_records rp_records rp_different_preview_records".split(),
)
REFS = (
    "decision_references",
    "run_count",
    *(p + "_references" for p in PHASES),
    *(
        p + "_" + s + "_references"
        for p in PHASES
        for s in (
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
        "available_" + b + "_" + g + "_" + s + "_references"
        for b, gg in BRANCHES.items()
        for g in gg
        for s in STATES
    ),
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
H, P = D("5e-10"), D("1e-12")
PI = D("3.141592653589793238462643383279502884197169399375105820974944592307816406286")


def contract():
    source = (core.REPO / next(iter(core.CODE))).read_text()
    assert (
        "const DT: f64 = 0.02;" in source
        and "let observation_time = step as f64 * DT;" in source
    )
    a = source.split("let recovery_margin = ", 1)[1].split(";", 1)[0]
    b = source.split("let speed_margin = if anticipatory_speed {", 1)[1].split(
        "} else", 1
    )[0]
    a, b = "".join(a.split()), "".join(b.split())
    assert a == b
    return hashlib.sha256(a.encode()).hexdigest()


def sincos(x):
    with decimal.localcontext() as ctx:
        ctx.prec = 65
        x = x % (2 * PI)
        if x > PI:
            x -= 2 * PI
        if x < -PI:
            x += 2 * PI
        sine, cosine = x, D(1)
        st, ct = x, D(1)
        for k in range(1, 100):
            st = -st * x * x / D((2 * k) * (2 * k + 1))
            ct = -ct * x * x / D((2 * k - 1) * (2 * k))
            sine += st
            cosine += ct
            if max(abs(st), abs(ct)) < D("1e-60"):
                return +sine, +cosine
    raise AssertionError("Taylor series failed to converge")


def geometry(s):
    with decimal.localcontext() as ctx:
        ctx.prec = 60
        q = lambda k: D(format(s[k], ".9f"))
        bounded = lambda x: (x - H - P, x + H + P)
        subtract = lambda x, y: (x[0] - y[1] - P, x[1] - y[0] + P)
        margin = bounded(q("speed_margin"))
        x, y, yaw = q("recovery_x"), q("recovery_y"), q("recovery_yaw")
        sn, cs = sincos(yaw)
        error = (H + P) * D("1.6") + P
        coords = [
            bounded(x),
            bounded(y),
            (x + D(".6") * cs - error, x + D(".6") * cs + error),
            (y + D(".6") * sn - error, y + D(".6") * sn + error),
        ]
        gaps = [subtract(c, margin) for c in coords] + [
            (50 - margin[1] - c[1] - P, 50 - margin[0] - c[0] + P) for c in coords
        ]
        nominal = (
            (D(0), D(0))
            if s["stop_latched"]
            else (
                max(D(0), bounded(q("nominal_target_speed"))[0]),
                bounded(q("nominal_target_speed"))[1],
            )
        )
        if s["stop_latched"]:
            assert s["nominal_target_speed"] == 0
        return dict(
            clearance=subtract(bounded(q("recovery_clearance")), margin),
            bounds=(min(g[0] for g in gaps), min(g[1] for g in gaps)),
            nominal=nominal,
            release=subtract(
                bounded(q("release_preview_clearance_lower_bound")),
                bounded(q("guard_release_margin")),
            ),
        )


def classify_range(pair, inclusive=False):
    lo, hi = pair
    assert lo <= hi
    if inclusive:
        return "pass" if lo >= 0 else "fail" if hi < 0 else "unknown"
    return "pass" if lo > 0 else "fail" if hi <= 0 else "unknown"


def counter(value, limit):
    n = int(round(value / 0.02))
    assert 0 <= n < limit and abs(value - 0.02 * n) <= 5e-7
    return n


def infer_timer(before, s, branch):
    jb, ja = counter(before["recovery_evidence"], 50), counter(
        s["recovery_evidence"], 50
    )
    gb, ga = counter(before["guard_release_evidence"], 25), counter(
        s["guard_release_evidence"], 25
    )
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
        (jb if before["safety_paused"] else 0, ja, 50) if joint else (gb, ga, 25)
    )
    release = s["resumed"] if joint else s["guard_released"]
    matches = []
    for ready in (False, True):
        count = prior + 1 if ready else 0
        released = count >= limit
        if (0 if released else count) == after and released == release:
            matches.append(ready)
    assert len(matches) == 1, (branch, prior, after, release)
    ready = matches[0]
    if joint:
        active = before["guard_active"] or s["guard_triggered"]
        assert ga == 0 and s["safety_paused"] == (not release)
        assert s["guard_released"] == (release and active) and s["guard_active"] == (
            active and not release
        )
    else:
        assert (
            ja == jb
            and not s["resumed"]
            and not s["safety_paused"]
            and not s["guard_triggered"]
        )
        assert s["guard_active"] == (not release)
    action = (
        "release"
        if release
        else "accumulate" if ready else "reset" if prior > 0 else "stay_zero"
    )
    return dict(
        timer_before_ticks=prior,
        timer_after_ticks=after,
        threshold_ticks=limit,
        ready_from_timer=ready,
        timer_action=action,
    )


def reconstruct_decision(e, n, before, s, phase):
    pending = before["safety_paused"] or (
        s["correction"] and not s["correction_received"]
    )
    branch = (
        "joint" if pending else "guard" if before["guard_active"] else "not_applicable"
    )
    env = geometry(s)
    tests = dict.fromkeys(GATES, "not_applicable")
    if branch in BRANCHES:
        tests["stationary"] = "fail" if abs(s["measured_speed"]) > 0.02 else "pass"
        tests["release"] = classify_range(env["release"])
    if branch == "joint":
        tests["samples"] = "fail" if s["recovery_sample_count"] < 10 else "pass"
        tests["age"] = "fail" if s["age_exact_s"] > 1.5 else "pass"
        for k in ("clearance", "bounds", "nominal"):
            tests[k] = classify_range(env[k], k == "bounds")
    failed = ";".join(g for g in GATES if tests[g] == "fail")
    unknown = ";".join(g for g in GATES if tests[g] == "unknown")
    state = (
        "not_applicable"
        if branch not in BRANCHES
        else "fail" if failed else "unknown" if unknown else "pass"
    )
    timer = infer_timer(before, s, branch)
    assert state not in ("pass", "fail") or timer["ready_from_timer"] == (
        state == "pass"
    )
    return dict(
        **{k: e[k] for k in (*META, "event_id", "source_event_id")},
        time_s=n / 50,
        offset_from_return_ms=(n - core.tick(e["return_time_s"])) * 20,
        phase=phase,
        prior_safety_paused=before["safety_paused"],
        prior_guard_active=before["guard_active"],
        joint_pending=pending,
        branch=branch,
        **s,
        **{"gate_" + k: v for k, v in tests.items()},
        **{
            k + "_gap_" + side: float(x)
            for k, pair in env.items()
            for side, x in zip(("lo", "hi"), pair)
        },
        certainty=state,
        failed_gates=failed,
        unknown_gates=unknown,
        **timer,
    )


def raw_inputs(cases, fixed, original, raw, roots):
    files, needed, at = defaultdict(dict), defaultdict(set), {}
    for c in cases:
        files[c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"][
            c["scenario"], c["planner"]
        ] = c
    for eid, e in original.items():
        at[e["case_id"], core.tick(e["event_time_s"])] = e
    for e in fixed.values():
        if core.truth(e["returned"] or "False"):
            a = core.tick(e["return_time_s"])
            b = core.tick(e["end_time_s"]) if core.truth(e["ended"]) else 9000
            needed[e["case_id"]].update(range(a - 1, b + 1))
    assert set(files) == {(p, n) for p, s in raw.items() for n in s["raw_sha256"]}
    points, audit = defaultdict(dict), Counter()
    for ordinal, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert core.sha(path) == raw[cohort]["raw_sha256"][name]
        audit["raw_bytes"] += path.stat().st_size
        grid = {k: -1 for k in members}
        last = defaultdict(int)
        with path.open() as f:
            header = f.readline().rstrip("\n").split(",")
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
            for line in f:
                assert '"' not in line
                cells = line.rstrip("\n").split(",")
                assert len(cells) == len(header)
                identity = cells[pos["scenario"]], cells[pos["planner"]]
                assert identity in members
                c = members[identity]
                cid = c["case_id"]
                n = core.tick(cells[pos["time_s"]])
                assert n == grid[identity] + 1
                grid[identity] = n
                if not core.truth(c["selected_timeout"]):
                    continue
                audit["timeout_grid_rows"] += 1
                if n == 0:
                    continue
                received, precision = (
                    core.truth(cells[pos[k]], True)
                    for k in ("correction_received", "precision_activation")
                )
                if received or precision:
                    last[cid] = n
                age = float(n) * float.fromhex("0x1.47ae147ae147bp-6") - float(
                    last[cid]
                ) * float.fromhex("0x1.47ae147ae147bp-6")
                assert format(age, ".6f") == cells[pos["observation_age"]]
                audit["source_clock_decisions"] += 1
                if (cid, n) in at:
                    e = at[cid, n]
                    assert (
                        received,
                        precision,
                        core.truth(cells[pos["correction"]], True),
                    ) == tuple(
                        core.truth(e[k])
                        for k in ("received", "precision_activation", "scheduled")
                    )
                    assert (
                        "recovery_mean"
                        if int(cells[pos["recovery_sample_count"]]) > 0
                        else "raw_fallback"
                    ) == e["immediate_actual"]
                    assert core.truth(cells[pos["epoch_fusion_available"]], True) == (
                        e["immediate_preview"] == "three_epoch"
                    )
                    audit["census_joins"] += 1
                if n not in needed[cid]:
                    continue
                s = {k: core.truth(cells[pos[k]], True) for k in BOOLS}
                s.update({k: float(cells[pos[k]]) for k in FLOATS})
                assert all(math.isfinite(s[k]) for k in FLOATS)
                count = int(cells[pos["recovery_sample_count"]])
                assert (
                    str(count) == cells[pos["recovery_sample_count"]]
                    and 0 <= count <= 25
                )
                assert s["stationary_epoch_fusion"] and s["speed_margin"] > 0
                s.update(
                    recovery_sample_count=count,
                    age_exact_s=age,
                    age_exact_hex=age.hex(),
                    last_observation_step=last[cid],
                )
                points[cid][n] = s
        for key, c in members.items():
            assert grid[key] == (
                9000 if core.truth(c["selected_timeout"]) else core.tick(c["time_s"])
            )
        audit["raw_files"] += 1
        if ordinal % 12 == 0:
            print(f"Independent admission inputs: {ordinal}/108 files", flush=True)
    assert all(set(points[cid]) == nn for cid, nn in needed.items())
    return points, dict(audit)


def old_controls(points, fixed):
    count = 0
    for s in core.read(RESULTS / "fusion-return-controls/control-snapshots.csv"):
        p = points[s["case_id"]][core.tick(s["time_s"])]
        for k in p.keys() & s.keys():
            core.check(s[k], p[k])
            count += 1
    for r in core.read(RESULTS / "fusion-return-controls/control-runs.csv"):
        assert all(r[k] == fixed[r["event_id"]][k] for k in META)
        for n in range(core.tick(r["start_time_s"]), core.tick(r["end_time_s"]) + 1):
            s = points[r["case_id"]][n]
            wanted = dict(
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
            for k, v in wanted.items():
                core.check(r[k], v)
                count += 1
    return count


def maximal_runs(db, decisions, fixed):
    db.execute(
        "CREATE TABLE raw(eid TEXT,n INT,branch TEXT,phase TEXT,certainty TEXT,ready_from_timer INT,failed_gates TEXT,unknown_gates TEXT,timer_action TEXT,PRIMARY KEY(eid,n))"
    )
    for (eid, n), r in decisions.items():
        db.execute(
            "INSERT INTO raw VALUES(?,?,?,?,?,?,?,?,?)",
            (
                eid,
                n,
                *(
                    (
                        int(r[k])
                        if k == "ready_from_timer" and r[k] is not None
                        else -1 if k == "ready_from_timer" else r[k]
                    )
                    for k in RUN_KEY
                ),
            ),
        )
    equal = " AND ".join(f"{k} IS LAG({k}) OVER w" for k in RUN_KEY)
    rows = db.execute(
        f"""WITH marked AS (SELECT *,CASE WHEN {equal} THEN 0 ELSE 1 END changed FROM raw WINDOW w AS(PARTITION BY eid ORDER BY n)),
      numbered AS(SELECT *,SUM(changed) OVER(PARTITION BY eid ORDER BY n) ordinal FROM marked)
      SELECT eid,ordinal,MIN(n),MAX(n),COUNT(*),{','.join(RUN_KEY)} FROM numbered GROUP BY eid,ordinal ORDER BY CAST(eid AS INT),ordinal"""
    ).fetchall()
    result = {}
    for eid, i, lo, hi, count, *vals in rows:
        assert count == hi - lo + 1
        data = dict(zip(RUN_KEY, vals))
        data["ready_from_timer"] = (
            None if data["ready_from_timer"] == -1 else bool(data["ready_from_timer"])
        )
        result[eid, str(i)] = dict(
            **{k: fixed[eid][k] for k in (*META, "event_id", "source_event_id")},
            run=i,
            start_time_s=lo / 50,
            end_time_s=hi / 50,
            checkpoint_count=count,
            **data,
        )
    return result


def anchor(e, dd, runs, ne, nr):
    out = {
        k: None if e[k] == "" else e[k] == "True" if e[k] in ("True", "False") else e[k]
        for k in BASE_FIELDS
    }
    out["anchor_category"] = e["anchor_category"]
    for k in ("parent_rp_records", "rp_records", "rp_different_preview_records"):
        out[k] = int(e[k])
    parent = out["parent_rp_records"]
    out.update(
        census_case_weight=Fraction(1, ne),
        received_case_weight=(
            Fraction(1, nr) if core.truth(e["received"]) else Fraction()
        ),
        rp_case_mass=Fraction(out["rp_records"], parent) if parent else Fraction(),
        rp_different_preview_case_mass=(
            Fraction(out["rp_different_preview_records"], parent)
            if parent
            else Fraction()
        ),
    )
    out.update(
        dict.fromkeys(
            (
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
        )
    )
    if not out["returned"]:
        assert not dd
        return out
    sums = Counter()
    for r in dd:
        phase, b = r["phase"], r["branch"]
        assert b in BRANCHES
        sums[phase + "_references"] += 1
        for x in (b, "certainty_" + r["certainty"], r["timer_action"]):
            sums[phase + "_" + x + "_references"] += 1
        sums[phase + "_ready_references"] += r["ready_from_timer"]
        if phase == "available":
            for g in BRANCHES[b]:
                sums[
                    "available_" + b + "_" + g + "_" + r["gate_" + g] + "_references"
                ] += 1
    out.update({k: sums[k] for k in REFS})
    available = [r for r in dd if r["phase"] == "available"]
    out.update(
        decision_references=len(dd),
        run_count=len(runs),
        return_branch=dd[0]["branch"],
        return_failed_gates=dd[0]["failed_gates"],
        return_unknown_gates=dd[0]["unknown_gates"],
        next_loss_failed_gates=dd[-1]["failed_gates"] if out["ended"] else None,
        initial_counter_ticks=dd[0]["timer_before_ticks"],
        max_available_counter_ticks=max(r["timer_after_ticks"] for r in available),
        has_ready_tick=any(r["ready_from_timer"] for r in available),
        has_timer_reset=any(r["timer_action"] == "reset" for r in available),
        has_unknown_readiness=any(r["certainty"] == "unknown" for r in available),
    )
    return out


def summaries(cases, anchors):
    buckets = defaultdict(list)
    for r in anchors.values():
        for m, c in product((r["mode"], "all_modes"), (r["anchor_category"], "all")):
            buckets[r["partition"], r["noise"], r["variant"], m, c].append(r)
    output = {}
    for key in product(
        ("known", "new", "fresh"),
        ("white", "correlated"),
        ("epoch", "curved"),
        ("none", "random", "blackout", "all_modes"),
        ("all", *CATEGORIES),
    ):
        p, n, a, m, _ = key
        cc = [
            c
            for c in cases
            if (c["partition"], c["noise"], c["variant"]) == (p, n, a)
            and (m == "all_modes" or c["mode"] == m)
        ]
        rr = buckets[key]
        returned = [r for r in rr if r["returned"]]
        mass = lambda rows, k="census_case_weight": sum(
            (r[k] for r in rows), Fraction()
        )
        out = dict(zip(GROUP, key))
        out.update(
            planned_cases=len(cc),
            timeout_cases=sum(core.truth(c["selected_timeout"]) for c in cc),
            held_cases=sum(core.truth(c["held"]) for c in cc),
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
        for name in REFS:
            out[name] = sum(r[name] or 0 for r in rr)
        output[key] = out
    return output


def closures(rows):
    checks = 0
    for key, r in rows.items():
        for i, whole, pieces in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", CATEGORIES),
        ):
            if key[i] == whole:
                children = [rows[key[:i] + (p,) + key[i + 1 :]] for p in pieces]
                for name in r:
                    if name in REFS or name.endswith(
                        ("_events", "_records", "_case_mass")
                    ):
                        assert sum(c[name] for c in children) == r[name], (key, name)
                        checks += 1
        assert r["decision_references"] == sum(r[p + "_references"] for p in PHASES)
        for p in PHASES:
            assert (
                sum(r[p + "_certainty_" + s + "_references"] for s in STATES)
                == r[p + "_references"]
            )
            assert (
                sum(r[p + "_" + b + "_references"] for b in BRANCHES)
                == r[p + "_references"]
            )
        for b, gg in BRANCHES.items():
            for g in gg:
                assert (
                    sum(
                        r["available_" + b + "_" + g + "_" + s + "_references"]
                        for s in STATES
                    )
                    == r["available_" + b + "_references"]
                )
    return checks


def compare(path, expected, keys):
    rows = core.index(core.read(path), keys)
    assert rows.keys() == expected.keys(), (path, len(rows), len(expected))
    for key, r in rows.items():
        core.fields(r, expected[key])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("results", "figures", "work", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--raw-roots", type=Path)
    args = parser.parse_args()
    if not __debug__ or args.work.exists() or args.output.exists():
        parser.error("Run without -O, a new work directory and unused output filename")
    base = RESULTS / "fusion-return-controls"
    assert (
        core.sha(base / "sources.json") == BASE_SHA
        and core.sha(base / "independent-verification.json") == PROOF_SHA
    )
    prior = json.loads((base / "sources.json").read_text())
    source = json.loads((args.results / "sources.json").read_text())
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        "fusion-return-controls/sources.json": BASE_SHA,
        "fusion-return-controls/independent-verification.json": PROOF_SHA,
        "fusion-return-controls/protocol.md": prior["analysis"]["protocol_sha256"],
        **{"fusion-return-controls/" + k: h for k, h in prior["output_sha256"].items()},
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_return_controls.py": prior["analysis"]["script_sha256"],
    }
    assert (
        source["analysis"]["verifier_helpers_sha256"] == HELPERS
        and source["analysis"]["code_sha256"] == core.CODE
    )
    assert (
        source["raw"] == prior["raw"]
        and source["inherited_validation"] == prior["inherited_validation"]
        and source["inherited_control_proof_sha256"] == PROOF_SHA
    )
    for name, h in source["input_sha256"].items():
        assert core.sha(RESULTS / name) == h, name
    for name, h in {**source["analysis"]["helpers_sha256"], **HELPERS}.items():
        assert core.sha(ROOT / name) == h, name
    for name, h in core.CODE.items():
        assert core.sha(core.REPO / name) == h, name
    assert source["analysis"]["shared_margin_expression_sha256"] == contract()
    assert source["analysis"]["script_sha256"] == core.sha(
        ROOT / "analyze_return_admission.py"
    )
    assert source["analysis"]["protocol_sha256"] == core.sha(
        RESULTS / "fusion-return-admission/protocol.md"
    )
    assert set(source["output_sha256"]) == {
        "anchor-admission.csv",
        "admission-decisions.csv",
        "admission-runs.csv",
        "admission-summary.csv",
    }
    for name, h in source["output_sha256"].items():
        assert core.sha(args.results / name) == h, name
    cases, original = core.original_inputs()
    fixed = {r["event_id"]: r for r in core.read(base / "anchor-controls.csv")}
    assert fixed.keys() == original.keys()
    for eid, e in fixed.items():
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
            core.check(e[k], original[eid][k])
    original_checks = core.comparisons
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    points, audit = raw_inputs(cases, fixed, original, source["raw"], roots)
    audit["preceding_control_comparisons"] = old_controls(points, fixed)
    preceding_checks = core.comparisons - original_checks
    decisions = {}
    for eid, e in fixed.items():
        if not core.truth(e["returned"] or "False"):
            continue
        start = core.tick(e["return_time_s"])
        end = core.tick(e["end_time_s"]) if core.truth(e["ended"]) else 9000
        for n in range(start, end + 1):
            decisions[eid, str(n)] = reconstruct_decision(
                e,
                n,
                points[e["case_id"]][n - 1],
                points[e["case_id"]][n],
                "next_loss" if n == end and core.truth(e["ended"]) else "available",
            )
    args.work.mkdir(parents=True)
    db = sqlite3.connect(args.work / "admission.sqlite")
    runs = maximal_runs(db, decisions, fixed)
    ne = Counter(e["case_id"] for e in original.values())
    nr = Counter(e["case_id"] for e in original.values() if core.truth(e["received"]))
    by_event, by_run = defaultdict(list), defaultdict(list)
    for (eid, _), r in decisions.items():
        by_event[eid].append(r)
    for (eid, _), r in runs.items():
        by_run[eid].append(r)
    anchors = {
        (eid,): anchor(
            e, by_event[eid], by_run[eid], ne[e["case_id"]], nr[e["case_id"]]
        )
        for eid, e in fixed.items()
    }
    summary = summaries(cases, anchors)
    closure_checks = closures(summary)
    decision_rows = {(eid, str(r["time_s"])): r for (eid, _), r in decisions.items()}
    for name, expected, keys in (
        ("anchor-admission", anchors, ("event_id",)),
        ("admission-decisions", decision_rows, ("event_id", "time_s")),
        ("admission-runs", runs, ("event_id", "run")),
        ("admission-summary", summary, GROUP),
    ):
        compare(args.results / (name + ".csv"), expected, keys)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(bool(r["returned"]) for r in anchors.values()),
        decision_references=len(decisions),
        available_references=sum(r["phase"] == "available" for r in decisions.values()),
        next_loss_references=sum(r["phase"] == "next_loss" for r in decisions.values()),
        admission_runs=len(runs),
        summary_rows=len(summary),
        additive_closure_comparisons=closure_checks,
    )
    assert audit == source["audit"], (audit, source["audit"])
    assert (
        sum(r["census_case_weight"] for r in anchors.values())
        == sum(r["received_case_weight"] for r in anchors.values())
        == 570
    )
    assert (
        sum(r["rp_case_mass"] for r in anchors.values()) == 157
        and sum(r["rp_records"] for r in anchors.values()) == 10995
    )
    plots = json.loads((args.figures / "plot-sources.json").read_text())
    assert plots["script_sha256"] == core.sha(ROOT / "plot_return_admission.py")
    assert plots["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in ("sources.json", "anchor-admission.csv", "admission-summary.csv")
    }
    assert set(plots["output_sha256"]) == {
        "admission-conditions.svg",
        "plot-values.csv",
        "plot-return-values.csv",
    }
    for n, h in plots["output_sha256"].items():
        assert core.sha(args.figures / n) == h, n
    plotted = {k: r for k, r in summary.items() if r["mode"] != "all_modes"}
    returned = {k: r for k, r in anchors.items() if r["returned"]}
    assert len(plotted) == 252 and len(returned) == 137
    compare(args.figures / "plot-values.csv", plotted, GROUP)
    compare(args.figures / "plot-return-values.csv", returned, ("event_id",))
    evidence = dict(
        **audit,
        comparisons=core.comparisons - original_checks - preceding_checks,
        original_join_comparisons=original_checks,
        plotted_summary_rows=252,
        plotted_return_rows=137,
        sqlite_version=sqlite3.sqlite_version,
        decimal_precision=60,
        independent_trigonometry="Decimal Taylor series to 1e-60",
        inherited_control_proof_sha256=PROOF_SHA,
        script_sha256=core.sha(Path(__file__)),
        sources_sha256=core.sha(args.results / "sources.json"),
        plot_sources_sha256=core.sha(args.figures / "plot-sources.json"),
        method="independent scalar raw reader, Decimal rounded-input intervals/Taylor trigonometry, ready-state counter enumeration and SQL maximal runs; no analyzer imports",
    )
    db.commit()
    db.close()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
