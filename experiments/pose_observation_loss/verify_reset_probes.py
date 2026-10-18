#!/usr/bin/env python3
"""Independently verify fixed reset-to-probe/map/witness joins."""

import argparse
from collections import Counter
import csv
from decimal import Decimal as D
import json
import math
from pathlib import Path

import verify_run_breaks as inherited

core = inherited.core
ROOT, REPO, RESULTS = core.ROOT, core.REPO, core.RESULTS
BASE = RESULTS / "fusion-return-run-breaks"
STUDY = RESULTS / "fusion-return-reset-probes"
DECISIONS = RESULTS / "fusion-return-local-admission/local-decisions.csv"
WITNESSES = RESULTS / "fusion-return-point-clearance/point-witnesses.csv"
MAP = RESULTS / "fusion-return-probe-origin/obstacle-map.csv"
BASE_MAIN = "f2dab0a36f92a52df9b3811588aa97b509dd25af"
BASE_SHA = "83d3c06df66e4e331c05486bfe7df0bdb4197d8159f7c1820d767d287f4debaa"
PROOF_SHA = "bb50761c35e3b49125861af5c93d339a43a95b337f24cd75737f4b86c50f0fdc"
HELPERS = {
    **inherited.HELPERS,
    "verify_run_breaks.py": "35a370af36b285902270b8f9a5eb5acfbe49ca03d1e21b43a379192bf90ba373",
}
META, GATES = inherited.META, inherited.GATES
ORIGINS = ("initial_unsafe", "first_chord_by_exclusion", "ambiguous_zero")
EXTRA = (
    "stage",
    "selected_blocker_ids",
    "origin_gap_lo",
    "origin_gap_hi",
    "origin_failure_mask",
    "blocker_rows",
)
BLOCK_EXTRA = ("edge_id", "multiplicity", "probe_origin", "map_x", "map_y")
SUMMARY = (
    "case_selector",
    "origin_selector",
    "physical_resets",
    "overlapping_endings",
    "blocker_rows",
    "distinct_obstacle_ids",
    "distinct_coordinate_groups",
    "obstacle_ids",
    "max_lost_ticks",
)
read = inherited.read


def step(time):
    value = D(time) * 50
    assert value.is_finite() and value == value.to_integral_value()
    return int(value)


def stage(d):
    origin = d["probe_origin"]
    assert origin in ORIGINS
    out = dict.fromkeys(EXTRA)
    out.update(
        stage="unresolved",
        selected_blocker_ids="",
        origin_failure_mask="",
        blocker_rows=0,
    )
    if origin == "ambiguous_zero":
        assert d["initial_test_state"] == "unknown"
        return out
    if origin == "initial_unsafe":
        assert d["initial_test_state"] == "unsafe"
        prefix, mask = "", d["initial_failure_mask"]
        out["stage"] = "initial"
    else:
        assert d["initial_test_state"] == "safe"
        assert (
            core.truth(d["first_chord_audited"])
            and d["direct_first_test_state"] == "unsafe"
        )
        prefix, mask = "first_", d["direct_first_failure_mask"]
        out["stage"] = "first_chord"
    ids = d[prefix + "definite_obstacle_ids"]
    assert ids and not d[prefix + "definite_wall_ids"] and mask == "obstacle_only"
    assert len(ids.split(";")) == len(set(ids.split(";")))
    for field in ("original_blocker_ids", "fixed_obstacle_ids"):
        core.check(d[field], ids)
    assert core.truth(d["point_clearance_audited"])
    bounds = [
        tuple(D(d[prefix + kind + "_gap_" + side]) for side in ("lo", "hi"))
        for kind in ("boundary", "obstacle")
    ]
    assert all(a.is_finite() and b.is_finite() and a <= b for a, b in bounds)
    lo, hi = min(b[0] for b in bounds), min(b[1] for b in bounds)
    assert lo <= hi <= 0
    out.update(
        selected_blocker_ids=ids,
        origin_gap_lo=float(lo),
        origin_gap_hi=float(hi),
        origin_failure_mask=mask,
        blocker_rows=len(ids.split(";")),
    )
    return out


def decision_index(rows):
    identities = [(d["case_id"], d["event_id"], step(d["time_s"])) for d in rows]
    assert len(identities) == len(set(identities))
    return rows


def join(edge, lookup):
    events = edge["event_ids"].split(";")
    assert len(events) == 1
    n = int(edge["source_step"])
    assert step(edge["source_time_s"]) == n
    matches = [
        d
        for d in lookup
        if d["case_id"] == edge["case_id"]
        and d["event_id"] == events[0]
        and step(d["time_s"]) == n
    ]
    assert len(matches) == 1
    d = matches[0]
    for field in META:
        core.check(edge[field], d[field])
    core.check(d["phase"], "available")
    core.check(edge["branch"], "joint")
    core.check(d["branch"], "joint")
    for f, expected in (
        ("failed_gates", "refined_failed_gates"),
        ("unknown_gates", "refined_unknown_gates"),
        ("nominal_positive", "source_nominal_positive"),
        ("zero_reason", "source_zero_reason"),
        ("timer_before_ticks", "timer_before_ticks"),
        ("timer_after_ticks", "timer_after_ticks"),
    ):
        core.check(edge[f], d[expected])
    for g in GATES:
        core.check(
            edge["gate_" + g],
            d["source_nominal_gate"] if g == "nominal" else d["gate_" + g],
        )
    core.check(d["source_nominal_positive"], False)
    core.check(d["source_zero_reason"], "cap_zero")
    core.check(d["lattice_index"], 0)
    core.check(d["stop_latched"], False)
    assert all(
        D(d[f]) == 0
        for f in ("speed_free_distance", "speed_cap", "nominal_target_speed")
    )
    core.check(edge["lost_ticks"], int(d["timer_before_ticks"]))
    assert int(edge["lost_ticks"]) > 0
    core.check(d["timer_after_ticks"], 0)
    core.check(d["timer_action"], "reset")
    core.check(d["ready_from_timer"], False)
    result = dict(edge)
    result.update(("decision_" + k, value) for k, value in d.items())
    result.update(stage(d))
    return result


def point_state(lo, hi):
    a, b = D(lo), D(hi)
    assert a.is_finite() and b.is_finite() and a <= b
    if b <= 0:
        return "blocked"
    return "clear" if a > 0 else "unknown"


def blocker_rows(probes, witnesses, obstacles):
    all_keys = [
        (w["case_id"], w["event_id"], step(w["time_s"]), w["obstacle_id"])
        for w in witnesses
    ]
    assert len(set(all_keys)) == len(all_keys)
    omap = {str(i): row for i, row in enumerate(obstacles, 1)}
    result = []
    for p in probes:
        selected = [
            w
            for w in witnesses
            if w["case_id"] == p["case_id"]
            and w["event_id"] == p["decision_event_id"]
            and step(w["time_s"]) == int(p["source_step"])
        ]
        expected_ids = (
            p["selected_blocker_ids"].split(";") if p["selected_blocker_ids"] else []
        )
        assert [w["obstacle_id"] for w in selected] == expected_ids
        assert len(selected) == p["blocker_rows"]
        for i, w in enumerate(selected, 1):
            for f in META:
                core.check(w[f], p[f])
            core.check(w["phase"], p["decision_phase"])
            core.check(w["branch"], p["decision_branch"])
            core.check(w["obstacle_order"], i)
            row = omap[w["obstacle_id"]]
            core.check(row["scenario"], w["scenario"])
            for axis in ("x", "y"):
                assert D.from_float(float(row[axis])) == D.from_float(
                    float(w["obstacle_" + axis])
                )
            for kind in ("raw", "mean"):
                core.check(
                    w[kind + "_current_state"],
                    point_state(
                        w[kind + "_current_gap_lo"], w[kind + "_current_gap_hi"]
                    ),
                )
            result.append(
                dict(
                    w,
                    edge_id=p["edge_id"],
                    multiplicity=p["multiplicity"],
                    probe_origin=p["decision_probe_origin"],
                    map_x=row["x"],
                    map_y=row["y"],
                )
            )
    return result


def summaries(probes, blockers):
    output = []
    cases = sorted(set(p["case_id"] for p in probes))
    for case in ["all_cases"] + cases:
        for origin in ["all_origins", *ORIGINS]:
            rows = [
                p
                for p in probes
                if (case == "all_cases" or p["case_id"] == case)
                and (origin == "all_origins" or p["decision_probe_origin"] == origin)
            ]
            selected = [
                w for w in blockers if any(w["edge_id"] == p["edge_id"] for p in rows)
            ]
            ids = sorted(set(w["obstacle_id"] for w in selected), key=int)
            locations = {
                (
                    w["scenario"],
                    D.from_float(float(w["map_x"])),
                    D.from_float(float(w["map_y"])),
                )
                for w in selected
            }
            output.append(
                dict(
                    case_selector=case,
                    origin_selector=origin,
                    physical_resets=len(rows),
                    overlapping_endings=sum(int(p["multiplicity"]) for p in rows),
                    blocker_rows=len(selected),
                    distinct_obstacle_ids=len(ids),
                    distinct_coordinate_groups=len(locations),
                    obstacle_ids=";".join(ids),
                    max_lost_ticks=max(
                        [int(p["lost_ticks"]) for p in rows], default=None
                    ),
                )
            )
    return output


def closure(rows):
    index = {(r["case_selector"], r["origin_selector"]): r for r in rows}
    checks = 0
    for key, row in index.items():
        partitions = []
        if key[0] == "all_cases":
            partitions.append(
                [r for (c, o), r in index.items() if c != "all_cases" and o == key[1]]
            )
        if key[1] == "all_origins":
            partitions.append([index[key[0], o] for o in ORIGINS])
        for children in partitions:
            for field in ("physical_resets", "overlapping_endings", "blocker_rows"):
                assert row[field] == sum(c[field] for c in children)
                checks += 1
    return checks


def compare(path, expected, keys):
    actual = read(path)
    assert [tuple(str(r[k]) for k in keys) for r in expected] == [
        tuple(r[k] for k in keys) for r in actual
    ]
    for a, b in zip(actual, expected):
        assert set(a) == set(b)
        for f, value in b.items():
            if isinstance(value, float):
                assert math.isclose(float(a[f]), value, abs_tol=1e-12, rel_tol=1e-10)
                core.comparisons += 1
            else:
                core.check(a[f], value)


def provenance(results):
    parent, _ = inherited.provenance(BASE)
    source = json.loads((results / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    freeze = json.loads((STUDY / "freeze-pins.json").read_text())
    assert (
        freeze["base_main"] == BASE_MAIN
        and freeze["sources_sha256"] == core.sha(BASE / "sources.json") == BASE_SHA
    )
    assert (
        freeze["proof_sha256"]
        == core.sha(BASE / "independent-verification.json")
        == PROOF_SHA
    )
    assert (
        proof["sources_sha256"] == BASE_SHA
        and proof["script_sha256"]
        == freeze["verifier_helper_sha256"]
        == HELPERS["verify_run_breaks.py"]
    )
    for f, path in (
        ("decision_sha256", DECISIONS),
        ("witness_sha256", WITNESSES),
        ("map_sha256", MAP),
    ):
        assert (
            freeze[f]
            == core.sha(path)
            == parent["input_sha256"][str(path.relative_to(RESULTS))]
        )
    pins = {
        **parent["input_sha256"],
        **{"fusion-return-run-breaks/" + p.name: core.sha(p) for p in BASE.iterdir()},
    }
    assert source["input_sha256"] == pins
    for name, h in pins.items():
        assert core.sha(RESULTS / name) == h
    helpers = {
        **parent["analysis"]["helpers_sha256"],
        "analyze_run_breaks.py": parent["analysis"]["script_sha256"],
    }
    assert (
        source["analysis"]["helpers_sha256"] == helpers
        and source["analysis"]["verifier_helpers_sha256"] == HELPERS
    )
    for name, h in {**helpers, **HELPERS}.items():
        assert core.sha(ROOT / name) == h
    assert source["analysis"]["code_sha256"] == parent["analysis"]["code_sha256"]
    for name, h in source["analysis"]["code_sha256"].items():
        assert core.sha(REPO / name) == h
    for f, path in (
        ("script_sha256", ROOT / "analyze_reset_probes.py"),
        ("protocol_sha256", STUDY / "protocol.md"),
        ("freeze_sha256", STUDY / "freeze-pins.json"),
    ):
        assert source["analysis"][f] == core.sha(path)
    parentplot = json.loads((BASE / "plot-sources.json").read_text())
    assert core.sha(BASE / "plot-sources.json") == proof[
        "plot_sources_sha256"
    ] and parentplot["script_sha256"] == core.sha(ROOT / "plot_run_breaks.py")
    for name, h in {
        **parentplot["input_sha256"],
        **parentplot["output_sha256"],
    }.items():
        assert core.sha(BASE / name) == h
    assert source["schema"] == dict(
        origins=list(ORIGINS),
        retrospective=True,
        inherited_geometry=True,
        selection="19 preceding exact case/source-step resets; 185 overlapping endings; no new decisions or weights",
    )
    assert source["inherited_run_breaks_proof_sha256"] == PROOF_SHA
    assert source["inherited_geometry_proofs"] == {
        n: core.sha(RESULTS / n / "independent-verification.json")
        for n in (
            "fusion-return-probe-origin",
            "fusion-return-first-chord",
            "fusion-return-point-clearance",
        )
    }
    assert set(source["output_sha256"]) == {
        n + ".csv" for n in ("reset-probes", "reset-blockers", "probe-summary")
    }
    for name, h in source["output_sha256"].items():
        assert core.sha(results / name) == h
    return source


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("results", "figures", "output"):
        p.add_argument("--" + name, type=Path, required=True)
    args = p.parse_args()
    if not __debug__ or args.output.exists():
        p.error("Use an unused evidence filename without -O")
    source = provenance(args.results)
    cases, canonical = core.original_inputs()
    anchors = read(BASE / "anchor-breaks.csv")
    assert set(canonical) == {e["event_id"] for e in anchors}
    for e in anchors:
        for f in (
            *META,
            "received",
            "parent_rp_records",
            "rp_records",
            "rp_different_preview_records",
        ):
            core.check(e[f], canonical[e["event_id"]][f])
    original_checks = core.comparisons
    edges, decisions, witnesses, obstacles = [
        read(path) for path in (BASE / "reset-edges.csv", DECISIONS, WITNESSES, MAP)
    ]
    probes = [join(edge, decision_index(decisions)) for edge in edges]
    blockers = blocker_rows(probes, witnesses, obstacles)
    summary = summaries(probes, blockers)
    assert len(cases) == 972 and len(anchors) == 3997 and len(decisions) == 3751
    assert len(probes) == 19 and len(obstacles) == 1340 and len(summary) == 20
    for name, rows, keys in (
        ("reset-probes", probes, ("edge_id",)),
        ("reset-blockers", blockers, ("edge_id", "obstacle_id")),
        ("probe-summary", summary, ("case_selector", "origin_selector")),
    ):
        compare(args.results / (name + ".csv"), rows, keys)
    audit = dict(
        physical_resets=len(probes),
        overlapping_endings=sum(int(p["multiplicity"]) for p in probes),
        joined_decisions=len(probes),
        blocker_rows=len(blockers),
        origins=dict(Counter(p["decision_probe_origin"] for p in probes)),
        obstacle_map_rows=len(obstacles),
        summary_rows=len(summary),
        additive_closure_comparisons=closure(summary),
        fresh_raw_reads=0,
        fresh_geometry_evaluations=0,
        fresh_native_runs=0,
    )
    assert audit == source["audit"] and audit["overlapping_endings"] == 185
    plots = json.loads((args.figures / "plot-sources.json").read_text())
    assert plots["script_sha256"] == core.sha(ROOT / "plot_reset_probes.py")
    assert plots["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in (
            "sources.json",
            "reset-probes.csv",
            "reset-blockers.csv",
            "probe-summary.csv",
        )
    }
    assert set(plots["output_sha256"]) == {
        "reset-probes.svg",
        "plot-reset-probes.csv",
        "plot-reset-blockers.csv",
        "plot-values.csv",
    }
    for name, h in plots["output_sha256"].items():
        assert core.sha(args.figures / name) == h
    for name, rows, keys in (
        ("plot-reset-probes.csv", probes, ("edge_id",)),
        ("plot-reset-blockers.csv", blockers, ("edge_id", "obstacle_id")),
        ("plot-values.csv", summary, ("case_selector", "origin_selector")),
    ):
        compare(args.figures / name, rows, keys)
    evidence = dict(
        **audit,
        comparisons=core.comparisons - original_checks,
        original_join_comparisons=original_checks,
        script_sha256=core.sha(Path(__file__)),
        sources_sha256=core.sha(args.results / "sources.json"),
        plot_sources_sha256=core.sha(args.figures / "plot-sources.json"),
        inherited_run_breaks_proof_sha256=PROOF_SHA,
        inherited_geometry_proofs=source["inherited_geometry_proofs"],
        method="independent exact edge/decision/witness/map joins, Decimal interval-state and stage checks, original census identities and additive descriptive counts; no new geometry or native execution"
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
