#!/usr/bin/env python3
"""Refine nominal signs using the pinned source's finite speed-cap domain."""

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
import struct

import analyze_return_admission as previous

ROOT, RESULTS, REPO = previous.ROOT, previous.RESULTS, previous.REPO
BASE = RESULTS / "fusion-return-admission"
BASE_SHA = "f83beeea16ed0eb61846be0c3ce58299ec4ef8df29f37ea8dc50261938bfc056"
PROOF_SHA = "27857ce67c5446350510faa25badc2edbd6188b62e84c102ada889d07f76c018"
PROTOCOL = RESULTS / "fusion-return-command-sign/protocol.md"
HELPERS = previous.HELPERS
META, KEY, GROUP = previous.META, previous.KEY, previous.GROUP
GATES, STATES, CATEGORIES = previous.GATES, previous.STATES, previous.CATEGORIES
PHASES, BRANCHES = ("available", "next_loss"), ("joint", "guard")
REASONS = ("positive", "cap_zero", "latch_zero", "both_zero")
SPEEDS = (
    "speed_free_distance",
    "speed_cap",
    "uncapped_target_speed",
    "nominal_target_speed",
    "target_speed",
)
FLAGS = (
    "stop_latched",
    "restart_limited",
    "safety_paused",
    "guard_active",
    "speed_braking",
    "stationary_epoch_fusion",
    "correction",
    "correction_received",
)
REFS = (
    "decision_references",
    *(
        f"{p}_{b}_{x}_references"
        for p, b in product(PHASES, BRANCHES)
        for x in (
            "all",
            *(f"prior_{s}" for s in STATES),
            *(f"refined_{s}" for s in STATES),
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
MASS_FIELDS = (
    "census_case_weight",
    "received_case_weight",
    "rp_case_mass",
    "rp_different_preview_case_mass",
)


def sha(path):
    with path.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def write(path, rows):
    previous.write(path, rows)


def extract(source, marker):
    start = source.index(marker)
    opening = source.index("{", start)
    depth = 0
    for i in range(opening, len(source)):
        depth += (source[i] == "{") - (source[i] == "}")
        if depth == 0:
            return source[start : i + 1]
    raise AssertionError("unclosed source block")


def source_contract(source):
    probe = extract(source, "fn speed_free_distance(")
    text = "".join(probe.split())
    assert re.findall(r"return([^;]+);", text) == ["0.0", "(step-1)asf64*0.05"]
    assert "forstepin1..=40{" in text and text.endswith("2.0}")
    links = (
        ("stationary_epoch_fusion", "aligned_terminal_release"),
        ("aligned_terminal_release", "terminal_entry_only"),
        ("terminal_entry_only", "recovery_preview"),
        ("recovery_preview", "motion_filter"),
    )
    for a, b in links:
        assert f"assert!(!{a} || {b});" in source
    assert (
        "assert!(!motion_filter || (joint_recovery && !motion_entry_buffer && !recovery_guard_buffer));"
        in source
    )
    assert "assert!(!joint_recovery || anticipatory_speed);" in source
    start = source.index("let mut target_speed = if stop_hold && stop_latched")
    end = source.index("let nominal_steering = steering;", start)
    chain = source[start:end]
    assert "0.0\n            } else if terminal_slowdown" in chain
    assert ".clamp(0.1, if restart_limited { 0.2 } else { TARGET_SPEED })" in chain
    assert "let joint_pending = joint_recovery && safety_paused;" in chain
    assert "if loss_aware && safety_paused && !joint_pending" in chain
    assert "target_speed = target_speed.min(0.4);" in chain
    markers = (
        "let uncapped_target_speed = target_speed;",
        "let speed_free_distance =",
        "let speed_cap =",
        "target_speed = target_speed.min(speed_cap);",
        "let nominal_target_speed = target_speed;",
    )
    assert [chain.index(m) for m in markers] == sorted(chain.index(m) for m in markers)
    assert (
        source.index("if safety_paused {\n                    target_speed = 0.0;", end)
        > end
    )
    assert (
        source.index("if guard_active {\n                    target_speed = 0.0;", end)
        > end
    )
    return {
        "probe_sha256": hashlib.sha256(probe.encode()).hexdigest(),
        "command_chain_sha256": hashlib.sha256(chain.encode()).hexdigest(),
        "minimum_positive_nominal": 0.1,
    }


def nano(value):
    assert re.fullmatch(r"(?:0|[1-9][0-9]*)\.[0-9]{9}", value), value
    whole, digits = value.split(".")
    return int(whole) * 10**9 + int(digits)


def lattice():
    out = []
    for k in range(41):
        free = k * 0.05
        cap = min(1.0, max(0.0, -0.15 + math.sqrt(0.15 * 0.15 + free)))
        out.append(
            dict(
                lattice_index=k,
                free_distance_bits=struct.pack(">d", free).hex(),
                speed_free_distance=format(free, ".9f"),
                speed_cap_bits=struct.pack(">d", cap).hex(),
                speed_cap=format(cap, ".9f"),
                cap_positive=cap > 0,
            )
        )
    assert not out[0]["cap_positive"] and all(r["cap_positive"] for r in out[1:])
    assert float(out[1]["speed_cap"]) > 0.1
    return out


def command(row, table):
    units = {k: nano(row[k]) for k in SPEEDS}
    assert units["speed_free_distance"] % 50000000 == 0
    k = units["speed_free_distance"] // 50000000
    assert 0 <= k <= 40
    item = table[k]
    assert row["speed_free_distance"] == item["speed_free_distance"]
    assert row["speed_cap"] == item["speed_cap"]
    flags = {k: previous.flag(row[k], raw=True) for k in FLAGS}
    assert flags["stationary_epoch_fusion"]
    stop = flags["stop_latched"]
    assert (
        units["uncapped_target_speed"] == 0
        if stop
        else 100000000 <= units["uncapped_target_speed"] <= 1000000000
    )
    assert units["nominal_target_speed"] == min(
        units["uncapped_target_speed"], units["speed_cap"]
    )
    assert units["target_speed"] == (
        0
        if flags["safety_paused"] or flags["guard_active"]
        else units["nominal_target_speed"]
    )
    measured = float(row["measured_speed"])
    assert math.isfinite(measured)
    cap = struct.unpack(">d", bytes.fromhex(item["speed_cap_bits"]))[0]
    assert flags["speed_braking"] == (measured > cap + 1e-9)
    positive = not stop and k > 0
    assert (
        units["nominal_target_speed"] >= 100000000
        if positive
        else units["nominal_target_speed"] == 0
    )
    reason = (
        "positive"
        if positive
        else "both_zero" if stop and k == 0 else "latch_zero" if stop else "cap_zero"
    )
    return dict(
        **{k: row[k] for k in SPEEDS},
        **flags,
        measured_speed=measured,
        lattice_index=k,
        speed_cap_bits=item["speed_cap_bits"],
        source_nominal_positive=positive,
        source_zero_reason=reason,
    )


def collect(cases, decisions, raw, roots, table):
    files, wanted = defaultdict(dict), set()
    for c in cases:
        files[c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"][
            c["scenario"], c["planner"]
        ] = c
    for d in decisions:
        wanted.add((d["case_id"], previous.step(d["time_s"])))
    assert set(files) == {
        (cohort, name) for cohort, s in raw.items() for name in s["raw_sha256"]
    }
    points, counts, audit = {}, Counter(), Counter()
    groups = {}
    for i, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert sha(path) == raw[cohort]["raw_sha256"][name]
        audit["raw_bytes"] += path.stat().st_size
        last, paused = {}, defaultdict(bool)
        with path.open(newline="") as f:
            rr = csv.reader(f)
            header = next(rr)
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
            for cells in rr:
                assert len(cells) == len(header)
                identity = cells[pos["scenario"]], cells[pos["planner"]]
                assert identity in members
                c = members[identity]
                n = previous.step(cells[pos["time_s"]])
                assert n == last.get(identity, -1) + 1
                last[identity] = n
                if not previous.flag(c["selected_timeout"]):
                    continue
                audit["timeout_grid_rows"] += 1
                if n == 0:
                    continue
                p = command({k: cells[j] for k, j in pos.items()}, table)
                pending = paused[identity] or (
                    p["correction"] and not p["correction_received"]
                )
                paused[identity] = p["safety_paused"]
                group = c["cohort"], c["variant"], c["mode"], c["seed"]
                groups[group] = tuple(c[k] for k in ("partition", "noise"))
                counts[
                    (
                        *group,
                        "joint" if pending else "not_joint",
                        p["source_zero_reason"],
                    )
                ] += 1
                audit["command_decisions"] += 1
                if (c["case_id"], n) in wanted:
                    p["joint_pending"] = pending
                    points[c["case_id"], n] = p
        assert set(last) == set(members)
        for identity, c in members.items():
            assert last[identity] == (
                9000
                if previous.flag(c["selected_timeout"])
                else previous.step(c["time_s"])
            )
        audit["raw_files"] += 1
        if i % 12 == 0:
            print(f"Source-domain commands checked: {i}/108 raw files", flush=True)
    assert set(points) == wanted
    rows = []
    for group, (part, noise) in sorted(groups.items()):
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
                            part,
                            noise,
                            branch,
                            reason,
                            counts[(*group, branch, reason)],
                        ),
                    )
                )
            )
    return points, rows, dict(audit)


def refine(old, point):
    for k in ("nominal_target_speed", "target_speed", "measured_speed"):
        assert float(old[k]) == float(point[k])
    for k in (
        "stop_latched",
        "safety_paused",
        "guard_active",
        "stationary_epoch_fusion",
        "correction",
        "correction_received",
        "joint_pending",
    ):
        assert previous.flag(old[k]) == point[k]
    gates = {g: old["gate_" + g] for g in GATES}
    prior = gates["nominal"]
    nominal = "pass" if point["source_nominal_positive"] else "fail"
    gates["nominal"] = nominal if old["branch"] == "joint" else "not_applicable"
    assert prior in ("unknown", gates["nominal"])
    failed = ";".join(g for g in GATES if gates[g] == "fail")
    unknown = ";".join(g for g in GATES if gates[g] == "unknown")
    certainty = "fail" if failed else "unknown" if unknown else "pass"
    if certainty != "unknown":
        assert (certainty == "pass") == previous.flag(old["ready_from_timer"])
    return dict(
        **{k: old[k] for k in (*META, "event_id", "source_event_id")},
        time_s=float(old["time_s"]),
        offset_from_return_ms=int(old["offset_from_return_ms"]),
        phase=old["phase"],
        branch=old["branch"],
        **point,
        prior_nominal_gate=prior,
        source_nominal_gate=gates["nominal"],
        **{"gate_" + g: gates[g] for g in GATES if g != "nominal"},
        prior_certainty=old["certainty"],
        prior_failed_gates=old["failed_gates"],
        prior_unknown_gates=old["unknown_gates"],
        refined_certainty=certainty,
        refined_failed_gates=failed,
        refined_unknown_gates=unknown,
        timer_before_ticks=int(old["timer_before_ticks"]),
        timer_after_ticks=int(old["timer_after_ticks"]),
        threshold_ticks=int(old["threshold_ticks"]),
        ready_from_timer=previous.flag(old["ready_from_timer"]),
        timer_action=old["timer_action"],
    )


def anchor(e, decisions, ne, nr):
    out = {
        k: (
            None
            if e[k] == ""
            else previous.flag(e[k]) if e[k] in ("True", "False") else e[k]
        )
        for k in ANCHOR_FIELDS
    }
    for k in ("parent_rp_records", "rp_records", "rp_different_preview_records"):
        out[k] = int(e[k])
    parent = out["parent_rp_records"]
    out.update(
        census_case_weight=Fraction(1, ne),
        received_case_weight=Fraction(1, nr) if out["received"] else Fraction(),
        rp_case_mass=Fraction(out["rp_records"], parent) if parent else Fraction(),
        rp_different_preview_case_mass=(
            Fraction(out["rp_different_preview_records"], parent)
            if parent
            else Fraction()
        ),
    )
    for k in MASS_FIELDS:
        assert math.isclose(float(e[k]), float(out[k]), abs_tol=2e-15)
    out.update(dict.fromkeys((*REFS, *HAS)))
    if not out["returned"]:
        assert not decisions
        return out
    start = previous.step(e["return_time_s"])
    end = previous.step(e["end_time_s"]) if out["ended"] else 9000
    assert [previous.step(r["time_s"]) for r in decisions] == list(
        range(start, end + 1)
    )
    counts = Counter()
    for d in decisions:
        assert all(d[k] == e[k] for k in META)
        p, b = d["phase"], d["branch"]
        names = [
            "all",
            "prior_" + d["prior_certainty"],
            "refined_" + d["refined_certainty"],
            d["source_zero_reason"],
        ]
        if d["prior_nominal_gate"] == "unknown":
            names.append("prior_nominal_unknown")
        if d["timer_action"] == "reset":
            names.append("reset")
            if not d["source_nominal_positive"]:
                names.append("reset_with_nominal_zero")
        if d["ready_from_timer"]:
            names.append("ready")
        for x in names:
            counts[f"{p}_{b}_{x}_references"] += 1
    out.update({k: counts[k] for k in REFS})
    out["decision_references"] = len(decisions)
    available = [d for d in decisions if d["phase"] == "available"]
    out.update(
        has_prior_unknown=any(d["prior_certainty"] == "unknown" for d in available),
        has_refined_unknown=any(d["refined_certainty"] == "unknown" for d in available),
        has_nominal_zero=any(not d["source_nominal_positive"] for d in available),
    )
    return out


def summaries(cases, anchors):
    buckets = defaultdict(list)
    for a in anchors:
        for m, c in product((a["mode"], "all_modes"), (a["anchor_category"], "all")):
            buckets[a["partition"], a["noise"], a["variant"], m, c].append(a)
    rows = []
    for key in product(
        ("known", "new", "fresh"),
        ("white", "correlated"),
        ("epoch", "curved"),
        ("none", "random", "blackout", "all_modes"),
        ("all", *CATEGORIES),
    ):
        p, n, arm, m, _ = key
        cc = [
            c
            for c in cases
            if (c["partition"], c["noise"], c["variant"]) == (p, n, arm)
            and (m == "all_modes" or c["mode"] == m)
        ]
        aa = buckets[key]
        returned = [a for a in aa if a["returned"]]
        mass = lambda rr, k="census_case_weight": sum((a[k] for a in rr), Fraction())
        r = dict(zip(GROUP, key))
        r.update(
            planned_cases=len(cc),
            timeout_cases=sum(previous.flag(c["selected_timeout"]) for c in cc),
            held_cases=sum(previous.flag(c["held"]) for c in cc),
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
            selected = [a for a in aa if a[name]]
            r[name + "_events"], r[name + "_case_mass"] = len(selected), mass(selected)
            r[name + "_fraction"] = (
                Fraction(len(selected), len(returned)) if returned else None
            )
            r[name + "_case_mass_fraction"] = (
                mass(selected) / mass(returned) if returned else None
            )
        r.update({k: sum(a[k] or 0 for a in aa) for k in REFS})
        rows.append(r)
    return rows


def closures(rows):
    lookup = {tuple(r[k] for k in GROUP): r for r in rows}
    assert len(lookup) == len(rows)
    checks = 0
    for key, r in lookup.items():
        for i, whole, pieces in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", CATEGORIES),
        ):
            if key[i] == whole:
                children = [lookup[key[:i] + (p,) + key[i + 1 :]] for p in pieces]
                for k in r:
                    if k in REFS or k.endswith(("_events", "_records", "_case_mass")):
                        assert sum(c[k] for c in children) == r[k], (key, k)
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--raw-roots", type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Use a new output directory and run without -O")
    assert (
        sha(BASE / "sources.json") == BASE_SHA
        and sha(BASE / "independent-verification.json") == PROOF_SHA
    )
    prior = json.loads((BASE / "sources.json").read_text())
    pins = {
        **prior["input_sha256"],
        "fusion-return-admission/sources.json": BASE_SHA,
        "fusion-return-admission/independent-verification.json": PROOF_SHA,
        "fusion-return-admission/protocol.md": prior["analysis"]["protocol_sha256"],
        **{
            "fusion-return-admission/" + k: h for k, h in prior["output_sha256"].items()
        },
    }
    helpers = {
        **prior["analysis"]["helpers_sha256"],
        "analyze_return_admission.py": prior["analysis"]["script_sha256"],
    }
    for name, h in pins.items():
        assert sha(RESULTS / name) == h, name
    for name, h in {**helpers, **HELPERS}.items():
        assert sha(ROOT / name) == h, name
    for name, h in prior["analysis"]["code_sha256"].items():
        assert sha(REPO / name) == h, name
    contract = source_contract(
        (
            REPO / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
        ).read_text()
    )
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(prior["raw"])
    cases = read(RESULTS / "census-rp-selection/case-selection.csv")
    events = read(BASE / "anchor-admission.csv")
    old = read(BASE / "admission-decisions.csv")
    previous.indexed(cases, KEY)
    previous.indexed(events, ("event_id",))
    previous.indexed(old, ("event_id", "time_s"))
    table = lattice()
    points, counts, audit = collect(cases, old, prior["raw"], roots, table)
    decisions = [
        refine(d, points[d["case_id"], previous.step(d["time_s"])]) for d in old
    ]
    by_event = defaultdict(list)
    for d in decisions:
        by_event[d["event_id"]].append(d)
    ne = Counter(e["case_id"] for e in events)
    nr = Counter(e["case_id"] for e in events if previous.flag(e["received"]))
    anchors = [
        anchor(e, by_event[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]])
        for e in events
    ]
    summary = summaries(cases, anchors)
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
    assert (
        audit["raw_files"],
        audit["command_decisions"],
        audit["timeout_grid_rows"],
    ) == (108, 5130000, 5130570)
    assert (
        len(cases),
        audit["planned_families"],
        len(ne),
        len(anchors),
        audit["returned_anchors"],
        len(decisions),
        audit["available_references"],
        len(summary),
    ) == (972, 324, 570, 3997, 137, 3751, 3662, 336)
    assert sum(r["decisions"] for r in counts) == 5130000
    assert (
        sum(a["census_case_weight"] for a in anchors)
        == sum(a["received_case_weight"] for a in anchors)
        == 570
    )
    assert (
        sum(a["rp_records"] for a in anchors) == 10995
        and sum(a["rp_case_mass"] for a in anchors) == 157
    )
    args.output.mkdir(parents=True)
    for name, rr in (
        ("command-lattice", table),
        ("timeout-command-counts", counts),
        ("command-decisions", decisions),
        ("anchor-command-sign", anchors),
        ("command-summary", summary),
    ):
        write(args.output / (name + ".csv"), rr)
    source = dict(
        input_sha256=pins,
        raw=prior["raw"],
        analysis=dict(
            script_sha256=sha(Path(__file__)),
            protocol_sha256=sha(PROTOCOL),
            helpers_sha256=helpers,
            verifier_helpers_sha256=HELPERS,
            code_sha256=prior["analysis"]["code_sha256"],
            source_contract=contract,
        ),
        schema=dict(
            retrospective=True,
            source_nominal_sign="finite cap lattice and positive uncapped domain; timer-independent",
            inherited_gate_states="six nonnominal interval gates and observed timer transitions from pinned admission proof",
            phases=PHASES,
            branches=BRANCHES,
            zero_reasons=REASONS,
            minimum_positive_nominal=0.1,
        ),
        inherited_admission_proof_sha256=PROOF_SHA,
        inherited_validation=prior["inherited_validation"],
        audit=audit,
        output_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
