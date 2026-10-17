#!/usr/bin/env python3
"""Verify command domains with native Rust caps and independent Decimal inputs."""

import argparse
from collections import Counter, defaultdict
from decimal import Decimal
from fractions import Fraction
from itertools import product
import json
from pathlib import Path
import shutil
import struct
import subprocess

import verify_epoch_retention as core

ROOT, RESULTS = core.ROOT, core.RESULTS
BASE = RESULTS / "fusion-return-admission"
BASE_SHA = "f83beeea16ed0eb61846be0c3ce58299ec4ef8df29f37ea8dc50261938bfc056"
PROOF_SHA = "27857ce67c5446350510faa25badc2edbd6188b62e84c102ada889d07f76c018"
HELPERS = {
    "verify_epoch_retention.py": "417a3e93cc4680e77722fe2c95e24f206d532399bd4e2efc8d9951a24530e9f1"
}
META, KEY = core.META, core.KEY
GROUP = ("partition", "noise", "variant", "mode", "category_selector")
GATES = "age stationary samples clearance bounds nominal release".split()
STATES = ("pass", "fail", "unknown")
REASONS = ("positive", "cap_zero", "latch_zero", "both_zero")
PHASES, BRANCHES = ("available", "next_loss"), ("joint", "guard")
CATEGORIES = (
    "ttl_ended",
    "age_ended",
    "returned_censored",
    "no_return",
    "unobserved_ttl_limit",
    "no_observed_loss",
)
SPEEDS = "speed_free_distance speed_cap uncapped_target_speed nominal_target_speed target_speed".split()
FLAGS = "stop_latched restart_limited safety_paused guard_active speed_braking stationary_epoch_fusion correction correction_received".split()
REFS = (
    "decision_references",
    *(
        f"{p}_{b}_{x}_references"
        for p, b in product(PHASES, BRANCHES)
        for x in (
            "all",
            *("prior_" + s for s in STATES),
            *("refined_" + s for s in STATES),
            "prior_nominal_unknown",
            *REASONS,
            "reset",
            "reset_with_nominal_zero",
            "ready",
        )
    ),
)
HAS = ("has_prior_unknown", "has_refined_unknown", "has_nominal_zero")
ANCHOR_FIELDS = (
    *META,
    *"event_id source_event_id event_kind event_time_s scheduled received precision_activation horizon preview_loss loss_operation loss_time_s returned return_time_s return_wait_s censor_wait_lower_bound_s returned_spell_span_s ended end_time_s end_operation censored_return no_return ttl_limit_without_next_step parent_rp_records rp_records rp_different_preview_records anchor_category".split(),
)


def contract(source):
    import hashlib
    import re

    probe = core.block(source, "fn speed_free_distance(")
    assert re.findall(r"return\s+(.*?);", probe) == ["0.0", "(step - 1) as f64 * 0.05"]
    assert "for step in 1..=40" in probe and probe.rstrip().endswith("2.0\n}")
    for text in (
        "assert!(!stationary_epoch_fusion || aligned_terminal_release);",
        "assert!(!aligned_terminal_release || terminal_entry_only);",
        "assert!(!terminal_entry_only || recovery_preview);",
        "assert!(!recovery_preview || motion_filter);",
        "assert!(!motion_filter || (joint_recovery && !motion_entry_buffer && !recovery_guard_buffer));",
        "assert!(!joint_recovery || anticipatory_speed);",
        "let joint_pending = joint_recovery && safety_paused;",
        ".clamp(0.1, if restart_limited { 0.2 } else { TARGET_SPEED })",
        "target_speed = target_speed.min(0.4);",
        "if loss_aware && safety_paused && !joint_pending",
    ):
        assert text in source
    chain = source[
        source.index(
            "let mut target_speed = if stop_hold && stop_latched"
        ) : source.index("let nominal_steering = steering;")
    ]
    assert (
        chain.index("let uncapped_target_speed")
        < chain.index("let speed_cap")
        < chain.index("target_speed = target_speed.min(speed_cap);")
        < chain.index("let nominal_target_speed")
    )
    assert source.index(
        "if safety_paused {\n                    target_speed = 0.0;",
        source.index(chain),
    ) > source.index(chain) + len(chain)
    return dict(
        probe_sha256=hashlib.sha256(probe.encode()).hexdigest(),
        command_chain_sha256=hashlib.sha256(chain.encode()).hexdigest(),
        minimum_positive_nominal=0.1,
    )


def native(work, rustc):
    source = (core.REPO / next(iter(core.CODE))).read_text()
    contract(source)
    target = next(
        line for line in source.splitlines() if line.startswith("const TARGET_SPEED:")
    )
    cap = (
        source.split("let speed_cap = if anticipatory_speed {", 1)[1]
        .split("} else", 1)[0]
        .strip()
    )
    start = source.index("let mut target_speed = if stop_hold && stop_latched")
    uncapped = source[start : source.index("let mut steering = controller", start)]
    prelude = target + "\nfn cap(speed_free_distance: f64) -> f64 {\n" + cap + "\n}\n"
    prelude += (
        """fn uncapped(stop_hold: bool, stop_latched: bool, terminal_slowdown: bool,
restart_limited: bool, sensed_goal_distance: f64, safety_paused: bool,
observation_time: f64, recovery_limit_until: f64) -> f64 {
let joint_recovery = true;
let loss_aware = true;
""" + uncapped + "\ntarget_speed\n}\n"
    )
    harness = prelude + r"""
fn main() {
    println!("lattice_index,free_distance_bits,speed_free_distance,speed_cap_bits,speed_cap,cap_positive");
    for k in 0..=40 {
        let free = k as f64 * 0.05;
        let value = cap(free);
        println!("{k},{:016x},{free:.9},{:016x},{value:.9},{}", free.to_bits(), value.to_bits(), value > 0.0);
    }
}
#[cfg(test)] mod tests {
    use super::*;
    #[test] fn source_zero_is_exact() { assert_eq!(cap(0.0).to_bits(), 0); }
    #[test] fn first_positive_is_above_uncapped_floor() {
        assert!(cap(0.05) > 0.1);
        assert!(cap(0.05) < 0.12);
    }
    #[test] fn complete_lattice_is_monotone_and_clamps() {
        let mut previous = 0.0;
        for k in 1..=40 {
            let c = cap(k as f64 * 0.05);
            assert!(c >= previous && c > 0.1 && c <= 1.0);
            previous = c;
        }
        assert_eq!(cap(2.0), 1.0);
    }
    #[test] fn source_uncapped_domain_includes_pause_and_stop() {
        for hold in [false, true] { for stop in [false, true] {
            if stop && !hold { continue; }
            for slow in [false, true] { for restart in [false, true] {
                for paused in [false, true] { for limit in [0.0, 200.0] {
                    for distance in [0.0, 0.19, 0.2, 1.0, 3.0, 100.0] {
                        let u = uncapped(hold, stop, slow, restart, distance, paused, 100.0, limit);
                        if stop { assert_eq!(u, 0.0); }
                        else { assert!(u >= 0.1 && u <= 1.0); }
                        if paused { assert!(u <= 0.4); }
                    }
                } }
            } }
        } }
    }
}
"""
    work.mkdir(parents=True)
    path = work / "command-domain.rs"
    path.write_text(harness)
    test = work / "command-domain-tests"
    exe = work / "command-domain"
    for extra, output in ((["--test"], test), ([], exe)):
        subprocess.run(
            [str(rustc), "--edition=2021", "-O", *extra, str(path), "-o", str(output)],
            check=True,
            capture_output=True,
            text=True,
        )
    tests = subprocess.run(
        [str(test)], check=True, capture_output=True, text=True
    ).stdout
    assert "4 passed; 0 failed" in tests
    (work / "native-tests.txt").write_text(tests)
    values = subprocess.run(
        [str(exe)], check=True, capture_output=True, text=True
    ).stdout
    table_path = work / "native-lattice.csv"
    table_path.write_text(values)
    table = core.read(table_path)
    assert len(table) == 41
    for i, row in enumerate(table):
        assert row["lattice_index"] == str(i)
        row["lattice_index"] = i
        row["cap_positive"] = core.truth(row["cap_positive"], raw=True)
    return table, dict(
        rustc=subprocess.run(
            [str(rustc), "--version"], check=True, capture_output=True, text=True
        ).stdout.strip(),
        harness_sha256=core.sha(path),
        native_lattice_sha256=core.sha(table_path),
        native_tests=4,
        method="unchanged source cap expression and pre-steering command block; stdlib rustc -O, assertions enabled",
    )


def units(text):
    value = Decimal(text)
    assert value.is_finite() and value >= 0 and not text.startswith("-")
    assert value.as_tuple().exponent == -9 and format(value, ".9f") == text
    return int(value * 1000000000)


def inspect(row, lookup):
    assert row["speed_free_distance"] in lookup
    cap = lookup[row["speed_free_distance"]]
    values = [units(row[k]) for k in SPEEDS]
    free, logged_cap, u, n, final = values
    assert free == cap["lattice_index"] * 50000000
    assert row["speed_cap"] == cap["speed_cap"]
    flags = {k: core.truth(row[k], raw=True) for k in FLAGS}
    assert flags["stationary_epoch_fusion"]
    assert u == 0 if flags["stop_latched"] else 100000000 <= u <= 1000000000
    assert n == min(u, logged_cap)
    assert final == (0 if flags["safety_paused"] or flags["guard_active"] else n)
    measured = float(row["measured_speed"])
    assert Decimal(row["measured_speed"]).is_finite()
    exact_cap = struct.unpack(">d", bytes.fromhex(cap["speed_cap_bits"]))[0]
    assert flags["speed_braking"] == (measured > exact_cap + 1e-9)
    zero_latch, zero_cap = flags["stop_latched"], not cap["cap_positive"]
    reason = {
        (False, False): "positive",
        (False, True): "cap_zero",
        (True, False): "latch_zero",
        (True, True): "both_zero",
    }[zero_latch, zero_cap]
    positive = not (zero_latch or zero_cap)
    assert n >= 100000000 if positive else n == 0
    return dict(
        **{k: row[k] for k in SPEEDS},
        **flags,
        measured_speed=measured,
        lattice_index=cap["lattice_index"],
        speed_cap_bits=cap["speed_cap_bits"],
        source_nominal_positive=positive,
        source_zero_reason=reason,
    )


def raw_inputs(cases, old, raw, roots, table):
    wanted = {(r["case_id"], core.tick(r["time_s"])) for r in old}
    files = defaultdict(dict)
    for c in cases:
        files[c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"][
            c["scenario"], c["planner"]
        ] = c
    assert set(files) == {(p, n) for p, s in raw.items() for n in s["raw_sha256"]}
    lookup = {r["speed_free_distance"]: r for r in table}
    points, raw_points, counts, audit, groups = {}, {}, Counter(), Counter(), {}
    for i, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert core.sha(path) == raw[cohort]["raw_sha256"][name]
        audit["raw_bytes"] += path.stat().st_size
        seen, paused = defaultdict(lambda: -1), defaultdict(bool)
        with path.open() as f:
            header = next(f).rstrip("\r\n").split(",")
            assert len(header) == len(set(header))
            pos = {
                k: header.index(k)
                for k in (
                    "scenario",
                    "planner",
                    "time_s",
                    "measured_speed",
                    *SPEEDS,
                    *FLAGS,
                )
            }
            for line in f:
                assert '"' not in line
                cells = line.rstrip("\r\n").split(",")
                assert len(cells) == len(header)
                identity = cells[pos["scenario"]], cells[pos["planner"]]
                assert identity in members
                c = members[identity]
                step = core.tick(cells[pos["time_s"]])
                assert step == seen[identity] + 1
                seen[identity] = step
                if not core.truth(c["selected_timeout"]):
                    continue
                audit["timeout_grid_rows"] += 1
                if step == 0:
                    continue
                p = inspect({k: cells[j] for k, j in pos.items()}, lookup)
                pending = paused[identity] or (
                    p["correction"] and not p["correction_received"]
                )
                paused[identity] = p["safety_paused"]
                group = c["cohort"], c["variant"], c["mode"], c["seed"]
                groups[group] = c["partition"], c["noise"]
                counts[
                    (
                        *group,
                        "joint" if pending else "not_joint",
                        p["source_zero_reason"],
                    )
                ] += 1
                audit["command_decisions"] += 1
                key = c["case_id"], step
                if key in wanted:
                    p["joint_pending"] = pending
                    points[key], raw_points[key] = p, dict(zip(header, cells))
        assert set(seen) == set(members)
        for identity, c in members.items():
            assert seen[identity] == (
                9000 if core.truth(c["selected_timeout"]) else core.tick(c["time_s"])
            )
        audit["raw_files"] += 1
        if i % 12 == 0:
            print(f"Independent native-domain raw checks: {i}/108 files", flush=True)
    assert points.keys() == wanted
    rows = []
    for group in sorted(groups):
        for branch, reason in product(("joint", "not_joint"), REASONS):
            rows.append(
                dict(
                    zip(
                        (
                            "cohort",
                            "variant",
                            "mode",
                            "seed",
                            "partition",
                            "noise",
                            "pause_branch",
                            "source_zero_reason",
                            "decisions",
                        ),
                        (
                            *group,
                            *groups[group],
                            branch,
                            reason,
                            counts[(*group, branch, reason)],
                        ),
                    )
                )
            )
    return points, raw_points, rows, dict(audit)


def reconstruct(old, point, raw):
    for k in old.keys() & raw.keys():
        if k in META:
            assert old[k] == raw[k]
        elif raw[k] in ("true", "false"):
            assert core.truth(old[k]) == core.truth(raw[k], raw=True), k
        else:
            assert float(old[k]) == float(raw[k]), k
    assert core.truth(old["joint_pending"]) == point["joint_pending"]
    selected = {k: old["gate_" + k] for k in GATES}
    nominal = (
        "not_applicable"
        if old["branch"] == "guard"
        else "pass" if point["source_nominal_positive"] else "fail"
    )
    assert selected["nominal"] in ("unknown", nominal)
    selected["nominal"] = nominal
    failed = ";".join(k for k in GATES if selected[k] == "fail")
    unknown = ";".join(k for k in GATES if selected[k] == "unknown")
    result = "fail" if failed else "unknown" if unknown else "pass"
    # This comparison follows inference. No timer field enters inspect().
    if result in ("pass", "fail"):
        assert (result == "pass") == core.truth(old["ready_from_timer"])
    return dict(
        **{k: old[k] for k in (*META, "event_id", "source_event_id")},
        time_s=float(old["time_s"]),
        offset_from_return_ms=int(old["offset_from_return_ms"]),
        phase=old["phase"],
        branch=old["branch"],
        **point,
        prior_nominal_gate=old["gate_nominal"],
        source_nominal_gate=nominal,
        **{"gate_" + g: selected[g] for g in GATES if g != "nominal"},
        prior_certainty=old["certainty"],
        prior_failed_gates=old["failed_gates"],
        prior_unknown_gates=old["unknown_gates"],
        refined_certainty=result,
        refined_failed_gates=failed,
        refined_unknown_gates=unknown,
        timer_before_ticks=int(old["timer_before_ticks"]),
        timer_after_ticks=int(old["timer_after_ticks"]),
        threshold_ticks=int(old["threshold_ticks"]),
        ready_from_timer=core.truth(old["ready_from_timer"]),
        timer_action=old["timer_action"],
    )


def anchor(e, dd, ne, nr):
    a = {
        k: (
            None
            if e[k] == ""
            else core.truth(e[k]) if e[k] in ("True", "False") else e[k]
        )
        for k in ANCHOR_FIELDS
    }
    parent = int(e["parent_rp_records"])
    for k in ("parent_rp_records", "rp_records", "rp_different_preview_records"):
        a[k] = int(e[k])
    a.update(
        census_case_weight=Fraction(1, ne),
        received_case_weight=Fraction(1, nr) if a["received"] else Fraction(),
        rp_case_mass=Fraction(a["rp_records"], parent) if parent else Fraction(),
        rp_different_preview_case_mass=(
            Fraction(a["rp_different_preview_records"], parent)
            if parent
            else Fraction()
        ),
    )
    for k in (
        "census_case_weight",
        "received_case_weight",
        "rp_case_mass",
        "rp_different_preview_case_mass",
    ):
        core.check(e[k], a[k])
    a.update(dict.fromkeys((*REFS, *HAS)))
    if not a["returned"]:
        assert not dd
        return a
    start, end = core.tick(e["return_time_s"]), (
        core.tick(e["end_time_s"]) if a["ended"] else 9000
    )
    assert [core.tick(d["time_s"]) for d in dd] == list(range(start, end + 1))
    a["decision_references"] = len(dd)
    for p, b in product(PHASES, BRANCHES):
        rr = [d for d in dd if (d["phase"], d["branch"]) == (p, b)]
        prefix = f"{p}_{b}_"
        values = dict(
            all=len(rr),
            prior_nominal_unknown=sum(d["prior_nominal_gate"] == "unknown" for d in rr),
            reset=sum(d["timer_action"] == "reset" for d in rr),
            reset_with_nominal_zero=sum(
                d["timer_action"] == "reset" and not d["source_nominal_positive"]
                for d in rr
            ),
            ready=sum(d["ready_from_timer"] for d in rr),
        )
        values.update(
            {"prior_" + s: sum(d["prior_certainty"] == s for d in rr) for s in STATES}
        )
        values.update(
            {
                "refined_" + s: sum(d["refined_certainty"] == s for d in rr)
                for s in STATES
            }
        )
        values.update(
            {
                reason: sum(d["source_zero_reason"] == reason for d in rr)
                for reason in REASONS
            }
        )
        a.update({prefix + k + "_references": n for k, n in values.items()})
    rr = [d for d in dd if d["phase"] == "available"]
    a.update(
        has_prior_unknown=any(d["prior_certainty"] == "unknown" for d in rr),
        has_refined_unknown=any(d["refined_certainty"] == "unknown" for d in rr),
        has_nominal_zero=any(not d["source_nominal_positive"] for d in rr),
    )
    return a


def summaries(cases, anchors):
    rows = {}
    for key in product(
        ("known", "new", "fresh"),
        ("white", "correlated"),
        ("epoch", "curved"),
        ("none", "random", "blackout", "all_modes"),
        ("all", *CATEGORIES),
    ):
        p, n, arm, m, category = key
        cc = [
            c
            for c in cases
            if (c["partition"], c["noise"], c["variant"]) == (p, n, arm)
            and (m == "all_modes" or c["mode"] == m)
        ]
        aa = [
            a
            for a in anchors
            if (a["partition"], a["noise"], a["variant"]) == (p, n, arm)
            and (m == "all_modes" or a["mode"] == m)
            and (category == "all" or a["anchor_category"] == category)
        ]
        returned = [a for a in aa if a["returned"]]
        mass = lambda rr, field="census_case_weight": sum(
            (a[field] for a in rr), Fraction()
        )
        r = dict(zip(GROUP, key))
        r.update(
            planned_cases=len(cc),
            timeout_cases=sum(core.truth(c["selected_timeout"]) for c in cc),
            held_cases=sum(core.truth(c["held"]) for c in cc),
            census_cases=len({a["case_id"] for a in aa}),
            returned_cases=len({a["case_id"] for a in returned}),
            census_events=len(aa),
            census_case_mass=mass(aa),
            received_case_mass=mass(aa, "received_case_weight"),
            rp_records=sum(a["rp_records"] for a in aa),
            rp_different_preview_records=sum(
                a["rp_different_preview_records"] for a in aa
            ),
            rp_case_mass=mass(aa, "rp_case_mass"),
            rp_different_preview_case_mass=mass(aa, "rp_different_preview_case_mass"),
            returned_events=len(returned),
            returned_case_mass=mass(returned),
        )
        for name in HAS:
            chosen = [a for a in aa if a[name]]
            r.update(
                {
                    name + "_events": len(chosen),
                    name + "_case_mass": mass(chosen),
                    name
                    + "_fraction": (
                        Fraction(len(chosen), len(returned)) if returned else None
                    ),
                    name
                    + "_case_mass_fraction": (
                        mass(chosen) / mass(returned) if returned else None
                    ),
                }
            )
        r.update({k: sum(a[k] or 0 for a in aa) for k in REFS})
        rows[key] = r
    return rows


def closures(rows):
    checks = 0
    for key, r in rows.items():
        for i, whole, parts in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", CATEGORIES),
        ):
            if key[i] == whole:
                for field in r:
                    if field in REFS or field.endswith(
                        ("_events", "_records", "_case_mass")
                    ):
                        assert r[field] == sum(
                            rows[key[:i] + (p,) + key[i + 1 :]][field] for p in parts
                        )
                        checks += 1
        assert r["decision_references"] == sum(
            r[f"{p}_{b}_all_references"] for p, b in product(PHASES, BRANCHES)
        )
        for p, b in product(PHASES, BRANCHES):
            for names in (
                REASONS,
                tuple("prior_" + s for s in STATES),
                tuple("refined_" + s for s in STATES),
            ):
                assert (
                    sum(r[f"{p}_{b}_{x}_references"] for x in names)
                    == r[f"{p}_{b}_all_references"]
                )
    return checks


def compare(path, expected, keys):
    got = core.index(core.read(path), keys)
    assert got.keys() == expected.keys(), (path, len(got), len(expected))
    for key, row in got.items():
        core.fields(row, expected[key])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("results", "figures", "work", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--rustc", type=Path, default=shutil.which("rustc"))
    parser.add_argument("--raw-roots", type=Path)
    args = parser.parse_args()
    if not __debug__ or args.work.exists() or args.output.exists() or not args.rustc:
        parser.error(
            "Run without -O, provide rustc, and use a new work directory/evidence filename"
        )
    assert (
        core.sha(BASE / "sources.json") == BASE_SHA
        and core.sha(BASE / "independent-verification.json") == PROOF_SHA
    )
    prior = json.loads((BASE / "sources.json").read_text())
    source = json.loads((args.results / "sources.json").read_text())
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        "fusion-return-admission/sources.json": BASE_SHA,
        "fusion-return-admission/independent-verification.json": PROOF_SHA,
        "fusion-return-admission/protocol.md": prior["analysis"]["protocol_sha256"],
        **{
            "fusion-return-admission/" + k: h for k, h in prior["output_sha256"].items()
        },
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_return_admission.py": prior["analysis"]["script_sha256"],
    }
    assert (
        source["analysis"]["verifier_helpers_sha256"] == HELPERS
        and source["analysis"]["code_sha256"] == core.CODE
    )
    assert (
        source["raw"] == prior["raw"]
        and source["inherited_validation"] == prior["inherited_validation"]
        and source["inherited_admission_proof_sha256"] == PROOF_SHA
    )
    assert source["schema"] == dict(
        retrospective=True,
        source_nominal_sign="finite cap lattice and positive uncapped domain; timer-independent",
        inherited_gate_states="six nonnominal interval gates and observed timer transitions from pinned admission proof",
        phases=list(PHASES),
        branches=list(BRANCHES),
        zero_reasons=list(REASONS),
        minimum_positive_nominal=0.1,
    )
    for name, h in source["input_sha256"].items():
        assert core.sha(RESULTS / name) == h, name
    for name, h in {**source["analysis"]["helpers_sha256"], **HELPERS}.items():
        assert core.sha(ROOT / name) == h, name
    for name, h in core.CODE.items():
        assert core.sha(core.REPO / name) == h, name
    assert source["analysis"]["source_contract"] == contract(
        (core.REPO / next(iter(core.CODE))).read_text()
    )
    assert source["analysis"]["script_sha256"] == core.sha(
        ROOT / "analyze_command_sign.py"
    )
    assert source["analysis"]["protocol_sha256"] == core.sha(
        RESULTS / "fusion-return-command-sign/protocol.md"
    )
    assert set(source["output_sha256"]) == {
        n + ".csv"
        for n in (
            "command-lattice",
            "timeout-command-counts",
            "command-decisions",
            "anchor-command-sign",
            "command-summary",
        )
    }
    for name, h in source["output_sha256"].items():
        assert core.sha(args.results / name) == h
    table, native_proof = native(args.work, args.rustc)
    compare(
        args.results / "command-lattice.csv",
        {(str(r["lattice_index"]),): r for r in table},
        ("lattice_index",),
    )
    cases, original = core.original_inputs()
    events = core.read(BASE / "anchor-admission.csv")
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
    old = core.read(BASE / "admission-decisions.csv")
    core.index(old, ("event_id", "time_s"))
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    points, raw, counts, audit = raw_inputs(cases, old, source["raw"], roots, table)
    decisions = [
        reconstruct(
            d,
            points[d["case_id"], core.tick(d["time_s"])],
            raw[d["case_id"], core.tick(d["time_s"])],
        )
        for d in old
    ]
    by_event = defaultdict(list)
    for d in decisions:
        by_event[d["event_id"]].append(d)
    ne = Counter(e["case_id"] for e in original.values())
    nr = Counter(e["case_id"] for e in original.values() if core.truth(e["received"]))
    anchors = [
        anchor(e, by_event[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]])
        for e in events
    ]
    summary = summaries(cases, anchors)
    for name, rows, keys in (
        (
            "timeout-command-counts",
            counts,
            ("cohort", "variant", "mode", "seed", "pause_branch", "source_zero_reason"),
        ),
        ("command-decisions", decisions, ("event_id", "time_s")),
        ("anchor-command-sign", anchors, ("event_id",)),
        ("command-summary", list(summary.values()), GROUP),
    ):
        compare(
            args.results / (name + ".csv"),
            {tuple(str(r[k]) for k in keys): r for r in rows},
            keys,
        )
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(bool(a["returned"]) for a in anchors),
        decision_references=len(decisions),
        available_references=sum(d["phase"] == "available" for d in decisions),
        next_loss_references=sum(d["phase"] == "next_loss" for d in decisions),
        unique_audited_points=len(points),
        lattice_rows=len(table),
        full_grid_count_rows=len(counts),
        summary_rows=len(summary),
        additive_closure_comparisons=closures(summary),
    )
    assert audit == source["audit"], (audit, source["audit"])
    assert sum(r["decisions"] for r in counts) == 5130000
    assert (
        sum(a["census_case_weight"] for a in anchors)
        == sum(a["received_case_weight"] for a in anchors)
        == 570
    )
    assert (
        sum(a["rp_case_mass"] for a in anchors) == 157
        and sum(a["rp_records"] for a in anchors) == 10995
    )
    plot = json.loads((args.figures / "plot-sources.json").read_text())
    assert plot["script_sha256"] == core.sha(ROOT / "plot_command_sign.py")
    assert plot["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in ("sources.json", "anchor-command-sign.csv", "command-summary.csv")
    }
    assert set(plot["output_sha256"]) == {
        "command-sign.svg",
        "plot-values.csv",
        "plot-return-values.csv",
    }
    for name, h in plot["output_sha256"].items():
        assert core.sha(args.figures / name) == h
    compare(
        args.figures / "plot-values.csv",
        {k: r for k, r in summary.items() if r["mode"] != "all_modes"},
        GROUP,
    )
    compare(
        args.figures / "plot-return-values.csv",
        {(r["event_id"],): r for r in anchors if r["returned"]},
        ("event_id",),
    )
    evidence = dict(
        **audit,
        comparisons=core.comparisons - original_checks,
        original_join_comparisons=original_checks,
        plotted_summary_rows=252,
        plotted_return_rows=137,
        native=native_proof,
        inherited_admission_proof_sha256=PROOF_SHA,
        script_sha256=core.sha(Path(__file__)),
        sources_sha256=core.sha(args.results / "sources.json"),
        plot_sources_sha256=core.sha(args.figures / "plot-sources.json"),
        method="independent Decimal/nanounit raw parsing, native source cap lattice and command floor, original census joins, source-sign inference before timer consistency; no analyzer imports",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
