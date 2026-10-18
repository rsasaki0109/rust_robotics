#!/usr/bin/env python3
"""Join inherited probe geometry and map witnesses to fixed reset edges."""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path

import analyze_run_breaks as inherited

ROOT, REPO, RESULTS = inherited.ROOT, inherited.REPO, inherited.RESULTS
BASE = RESULTS / "fusion-return-run-breaks"
STUDY = RESULTS / "fusion-return-reset-probes"
DECISIONS = RESULTS / "fusion-return-local-admission/local-decisions.csv"
WITNESSES = RESULTS / "fusion-return-point-clearance/point-witnesses.csv"
MAP = RESULTS / "fusion-return-probe-origin/obstacle-map.csv"
BASE_MAIN = "f2dab0a36f92a52df9b3811588aa97b509dd25af"
BASE_SHA = "83d3c06df66e4e331c05486bfe7df0bdb4197d8159f7c1820d767d287f4debaa"
PROOF_SHA = "bb50761c35e3b49125861af5c93d339a43a95b337f24cd75737f4b86c50f0fdc"
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
sha, read = inherited.sha, inherited.read


def write(path, rows, fields):
    assert len(fields) == len(set(fields)) and all(set(r) == set(fields) for r in rows)
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def step(time):
    n = round(float(time) * 50)
    assert math.isclose(float(time), n / 50, rel_tol=0, abs_tol=1e-10)
    return n


def stage(d):
    origin = d["probe_origin"]
    assert origin in ORIGINS
    if origin == "ambiguous_zero":
        assert d["initial_test_state"] == "unknown"
        return dict(
            stage="unresolved",
            selected_blocker_ids="",
            origin_gap_lo=None,
            origin_gap_hi=None,
            origin_failure_mask="",
            blocker_rows=0,
        )
    first = origin == "first_chord_by_exclusion"
    if first:
        assert (
            d["initial_test_state"] == "safe"
            and d["first_chord_audited"] == "True"
            and d["direct_first_test_state"] == "unsafe"
        )
    else:
        assert d["initial_test_state"] == "unsafe"
    prefix = "first_" if first else ""
    ids = d[prefix + "definite_obstacle_ids"]
    assert ids and not d[prefix + "definite_wall_ids"]
    numbers = ids.split(";")
    assert len(numbers) == len(set(numbers))
    assert (
        ids == d["original_blocker_ids"] == d["fixed_obstacle_ids"]
        and d["point_clearance_audited"] == "True"
    )
    lo = min(float(d[prefix + name + "_gap_lo"]) for name in ("boundary", "obstacle"))
    hi = min(float(d[prefix + name + "_gap_hi"]) for name in ("boundary", "obstacle"))
    assert math.isfinite(lo) and math.isfinite(hi) and lo <= hi <= 0
    mask = d["direct_first_failure_mask"] if first else d["initial_failure_mask"]
    assert mask == "obstacle_only"
    return dict(
        stage="first_chord" if first else "initial",
        selected_blocker_ids=ids,
        origin_gap_lo=lo,
        origin_gap_hi=hi,
        origin_failure_mask=mask,
        blocker_rows=len(numbers),
    )


def decision_index(rows):
    result = {(d["case_id"], d["event_id"], step(d["time_s"])): d for d in rows}
    assert len(result) == len(rows)
    return result


def join(edge, lookup):
    events = edge["event_ids"].split(";")
    assert len(events) == 1
    d = lookup[edge["case_id"], events[0], int(edge["source_step"])]
    assert step(edge["source_time_s"]) == int(edge["source_step"])
    assert all(edge[k] == d[k] for k in META)
    assert d["phase"] == "available" and edge["branch"] == d["branch"] == "joint"
    assert (
        edge["failed_gates"] == d["refined_failed_gates"]
        and edge["unknown_gates"] == d["refined_unknown_gates"]
    )
    for g in GATES:
        assert (
            edge["gate_" + g]
            == d["source_nominal_gate" if g == "nominal" else "gate_" + g]
        )
    assert edge["nominal_positive"] == d["source_nominal_positive"] == "False"
    assert edge["zero_reason"] == d["source_zero_reason"] == "cap_zero"
    assert d["lattice_index"] == "0" and d["stop_latched"] == "False"
    assert all(
        float(d[k]) == 0
        for k in ("speed_free_distance", "speed_cap", "nominal_target_speed")
    )
    assert (
        edge["timer_before_ticks"] == d["timer_before_ticks"] == edge["lost_ticks"]
        and int(edge["lost_ticks"]) > 0
    )
    assert (
        edge["timer_after_ticks"] == d["timer_after_ticks"] == "0"
        and d["timer_action"] == "reset"
        and d["ready_from_timer"] == "False"
    )
    return dict(edge, **{"decision_" + k: value for k, value in d.items()}, **stage(d))


def point_state(lo, hi):
    a, b = float(lo), float(hi)
    assert math.isfinite(a) and math.isfinite(b) and a <= b
    return "clear" if a > 0 else "blocked" if b <= 0 else "unknown"


def blocker_rows(probes, witnesses, obstacles):
    by = {}
    for w in witnesses:
        key = w["case_id"], w["event_id"], step(w["time_s"])
        by.setdefault(key, []).append(w)
    result = []
    for p in probes:
        event = p["decision_event_id"]
        current = by.get((p["case_id"], event, int(p["source_step"])), [])
        ids = p["selected_blocker_ids"].split(";") if p["selected_blocker_ids"] else []
        assert [w["obstacle_id"] for w in current] == ids
        for number, w in enumerate(current, 1):
            assert all(w[k] == p[k] for k in META)
            assert (
                w["phase"] == p["decision_phase"]
                and w["branch"] == p["decision_branch"]
            )
            assert w["obstacle_order"] == str(number)
            i = int(w["obstacle_id"])
            assert str(i) == w["obstacle_id"] and 1 <= i <= len(obstacles)
            obstacle = obstacles[i - 1]
            assert obstacle["scenario"] == w["scenario"]
            assert all(
                float(obstacle[a]) == float(w["obstacle_" + a]) for a in ("x", "y")
            )
            for kind in ("raw", "mean"):
                assert w[kind + "_current_state"] == point_state(
                    w[kind + "_current_gap_lo"], w[kind + "_current_gap_hi"]
                )
            result.append(
                dict(
                    w,
                    edge_id=p["edge_id"],
                    multiplicity=p["multiplicity"],
                    probe_origin=p["decision_probe_origin"],
                    map_x=obstacle["x"],
                    map_y=obstacle["y"],
                )
            )
        assert len(current) == p["blocker_rows"]
    return result


def summaries(probes, blockers):
    result = []
    for case in ("all_cases", *sorted({p["case_id"] for p in probes})):
        for origin in ("all_origins", *ORIGINS):
            selected = [
                p
                for p in probes
                if case in ("all_cases", p["case_id"])
                and origin in ("all_origins", p["decision_probe_origin"])
            ]
            keys = {p["edge_id"] for p in selected}
            records = [b for b in blockers if b["edge_id"] in keys]
            ids = sorted({int(b["obstacle_id"]) for b in records})
            coords = {
                (b["scenario"], float(b["map_x"]), float(b["map_y"])) for b in records
            }
            result.append(
                dict(
                    case_selector=case,
                    origin_selector=origin,
                    physical_resets=len(selected),
                    overlapping_endings=sum(int(p["multiplicity"]) for p in selected),
                    blocker_rows=len(records),
                    distinct_obstacle_ids=len(ids),
                    distinct_coordinate_groups=len(coords),
                    obstacle_ids=";".join(map(str, ids)),
                    max_lost_ticks=max(
                        (int(p["lost_ticks"]) for p in selected), default=None
                    ),
                )
            )
    return result


def closure(rows):
    by = {(r["case_selector"], r["origin_selector"]): r for r in rows}
    cases = [c for c, o in by if o == "all_origins" and c != "all_cases"]
    checks = 0
    for (c, o), row in by.items():
        for parts in (
            ([by[k, o] for k in cases] if c == "all_cases" else []),
            ([by[c, k] for k in ORIGINS] if o == "all_origins" else []),
        ):
            if parts:
                for f in ("physical_resets", "overlapping_endings", "blocker_rows"):
                    assert row[f] == sum(r[f] for r in parts)
                    checks += 1
    return checks


def baseline():
    inherited.baseline()
    freeze = json.loads((STUDY / "freeze-pins.json").read_text())
    assert (
        freeze["base_main"] == BASE_MAIN
        and freeze["sources_sha256"] == sha(BASE / "sources.json") == BASE_SHA
    )
    assert (
        freeze["proof_sha256"]
        == sha(BASE / "independent-verification.json")
        == PROOF_SHA
    )
    source, proof = (
        json.loads((BASE / n).read_text())
        for n in ("sources.json", "independent-verification.json")
    )
    assert proof["sources_sha256"] == BASE_SHA and proof["script_sha256"] == freeze[
        "verifier_helper_sha256"
    ] == sha(ROOT / "verify_run_breaks.py")
    for field, path in (
        ("script_sha256", ROOT / "analyze_run_breaks.py"),
        ("protocol_sha256", BASE / "protocol.md"),
        ("freeze_sha256", BASE / "freeze-pins.json"),
    ):
        assert source["analysis"][field] == sha(path)
    for n, h in source["output_sha256"].items():
        assert sha(BASE / n) == h
    for f, path in (
        ("decision_sha256", DECISIONS),
        ("witness_sha256", WITNESSES),
        ("map_sha256", MAP),
    ):
        assert (
            freeze[f]
            == sha(path)
            == source["input_sha256"][str(path.relative_to(RESULTS))]
        )
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_run_breaks.py": source["analysis"]["script_sha256"],
    }
    vhelpers = {
        **source["analysis"]["verifier_helpers_sha256"],
        "verify_run_breaks.py": freeze["verifier_helper_sha256"],
    }
    for n, h in {**helpers, **vhelpers}.items():
        assert sha(ROOT / n) == h
    plots = json.loads((BASE / "plot-sources.json").read_text())
    assert sha(BASE / "plot-sources.json") == proof["plot_sources_sha256"] and plots[
        "script_sha256"
    ] == sha(ROOT / "plot_run_breaks.py")
    for n, h in {**plots["input_sha256"], **plots["output_sha256"]}.items():
        assert sha(BASE / n) == h
    pins = {
        **source["input_sha256"],
        **{"fusion-return-run-breaks/" + p.name: sha(p) for p in BASE.iterdir()},
    }
    return source, pins, helpers, vhelpers


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    if not __debug__ or args.output.exists():
        p.error("Use a fresh output directory without -O")
    parent, pins, helpers, vhelpers = baseline()
    edges, decisions, witnesses, obstacles = (
        read(path) for path in (BASE / "reset-edges.csv", DECISIONS, WITNESSES, MAP)
    )
    lookup = decision_index(decisions)
    probes = [join(e, lookup) for e in edges]
    blockers = blocker_rows(probes, witnesses, obstacles)
    summary = summaries(probes, blockers)
    assert (
        len(probes) == 19
        and len(decisions) == 3751
        and len(obstacles) == 1340
        and len(summary) == 20
    )
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
    assert audit["overlapping_endings"] == 185
    args.output.mkdir(parents=True)
    for name, rows, fields in (
        (
            "reset-probes",
            probes,
            (*edges[0], *("decision_" + k for k in decisions[0]), *EXTRA),
        ),
        ("reset-blockers", blockers, (*witnesses[0], *BLOCK_EXTRA)),
        ("probe-summary", summary, SUMMARY),
    ):
        write(args.output / (name + ".csv"), rows, fields)
    data = dict(
        input_sha256=pins,
        analysis=dict(
            script_sha256=sha(Path(__file__)),
            protocol_sha256=sha(STUDY / "protocol.md"),
            freeze_sha256=sha(STUDY / "freeze-pins.json"),
            helpers_sha256=helpers,
            verifier_helpers_sha256=vhelpers,
            code_sha256=parent["analysis"]["code_sha256"],
        ),
        schema=dict(
            origins=ORIGINS,
            retrospective=True,
            inherited_geometry=True,
            selection="19 preceding exact case/source-step resets; 185 overlapping endings; no new decisions or weights",
        ),
        inherited_run_breaks_proof_sha256=PROOF_SHA,
        inherited_geometry_proofs={
            n: sha(RESULTS / n / "independent-verification.json")
            for n in (
                "fusion-return-probe-origin",
                "fusion-return-first-chord",
                "fusion-return-point-clearance",
            )
        },
        audit=audit,
        output_sha256={f.name: sha(f) for f in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
