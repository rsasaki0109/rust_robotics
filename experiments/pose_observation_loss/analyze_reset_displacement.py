#!/usr/bin/env python3
"""Compare the raw and mean position estimates at the 19 frozen reset edges."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path

import analyze_reset_probes as inherited

ROOT, REPO, RESULTS = inherited.ROOT, inherited.REPO, inherited.RESULTS
BASE = RESULTS / "fusion-return-reset-probes"
STUDY = RESULTS / "fusion-return-reset-displacement"
BASE_MAIN = "810409fa265762d614261016c85f8cd398365ab6"
BASE_SHA = "43e4109d5b9adbbc30acefe0a9583feb7724825c49e9448eb0d7d493e21a0858"
PROOF_SHA = "ba1acd798c92798c224de7759cbab28344b717d989bfbcfc5fa8018b2bd64122"
P = 1e-12
ORIGINS, META = inherited.ORIGINS, inherited.META
QUANTITIES = (
    "delta_x",
    "delta_y",
    "raw_offset_x",
    "raw_offset_y",
    "displacement_norm",
    "distance_gain",
    "gap_gain",
    "linear_term",
    "quadratic_term",
    "squared_distance_change",
    "squared_prediction",
    "identity_residual",
    "margin_cancellation_residual",
)
EXTRA = tuple(f"{q}_{s}" for q in QUANTITIES for s in ("center", "lo", "hi")) + (
    "distance_gain_state",
    "linear_state",
    "point_pair",
    "raw_blocked_mean_clear",
    "common_margin",
)
COUNTS = (
    "physical_resets",
    "overlapping_endings",
    "gain_increase",
    "gain_decrease",
    "gain_unknown",
    "linear_outward",
    "linear_inward",
    "linear_unknown",
    "raw_blocked_mean_clear",
)
EXTREMA = tuple(
    f"{op}_{q}_{s}"
    for q in ("distance_gain", "displacement_norm")
    for op in ("min", "max")
    for s in ("lo", "hi")
)
SUMMARY = ("case_selector", "origin_selector", *COUNTS, *EXTREMA)
SCHEMA = dict(
    origins=ORIGINS,
    retrospective=True,
    elemental_padding=P,
    quantities=QUANTITIES,
    selection="19 preceding exact reset edges and their original blocker witnesses; 185 overlapping endings; no new trials or weights",
    interpretation="same-step spatial estimate comparison; not temporal robot motion",
)
read, sha, write = inherited.read, inherited.sha, inherited.write


def box(center, lo, hi):
    values = tuple(float(v) for v in (center, lo, hi))
    c, a, b = values
    assert all(math.isfinite(v) for v in values) and a <= c <= b
    return values


def load(row, prefix):
    return box(*(row[prefix + "_" + s] for s in ("center", "lo", "hi")))


def constant(value):
    return box(value, value, value)


def add(a, b):
    return box(a[0] + b[0], a[1] + b[1] - P, a[2] + b[2] + P)


def sub(a, b):
    return box(a[0] - b[0], a[1] - b[2] - P, a[2] - b[1] + P)


def mul(a, b):
    products = [x * y for x in a[1:] for y in b[1:]]
    return box(a[0] * b[0], min(products) - P, max(products) + P)


def square(a):
    low = 0 if a[1] <= 0 <= a[2] else min(a[1] ** 2, a[2] ** 2)
    return box(a[0] ** 2, max(0, low - P), max(a[1] ** 2, a[2] ** 2) + P)


def positive_sum(a, b):
    c, lo, hi = add(a, b)
    return box(c, max(0, lo), hi)


def sqrt(a):
    assert a[1] >= 0
    return box(math.sqrt(a[0]), max(0, math.sqrt(a[1]) - P), math.sqrt(a[2]) + P)


def sign(a, positive, negative):
    return positive if a[1] > 0 else negative if a[2] < 0 else "unknown"


def compute(p, w):
    raw = [load(p, "decision_point_raw_current_" + axis) for axis in ("x", "y")]
    mean = [load(p, "decision_point_mean_current_" + axis) for axis in ("x", "y")]
    obstacle = [constant(w["map_" + axis]) for axis in ("x", "y")]
    for axis in ("x", "y"):
        assert float(w["map_" + axis]) == float(w["obstacle_" + axis])
    margin = box(*(p["decision_point_current_margin" + s] for s in ("", "_lo", "_hi")))
    assert margin[1] >= 0
    distances, gaps = {}, {}
    for kind, position in (("raw", raw), ("mean", mean)):
        d = distances[kind] = load(w, kind + "_current_distance")
        g = gaps[kind] = load(w, kind + "_current_gap")
        assert d[1] >= 0
        calculated = math.sqrt(
            sum((a[0] - b[0]) ** 2 for a, b in zip(position, obstacle))
        )
        assert math.isclose(calculated, d[0], abs_tol=1e-12, rel_tol=1e-10)
        assert abs(d[0] - g[0] - margin[0]) <= 1e-10
        assert w[kind + "_current_state"] == inherited.point_state(g[1], g[2])
    delta = [sub(m, r) for m, r in zip(mean, raw)]
    offset = [sub(r, o) for r, o in zip(raw, obstacle)]
    quadratic = positive_sum(*(square(v) for v in delta))
    linear = mul(constant(2), add(*(mul(u, v) for u, v in zip(offset, delta))))
    observed = sub(square(distances["mean"]), square(distances["raw"]))
    prediction = add(linear, quadratic)
    distance_gain = sub(distances["mean"], distances["raw"])
    gap_gain = sub(gaps["mean"], gaps["raw"])
    quantities = dict(
        delta_x=delta[0],
        delta_y=delta[1],
        raw_offset_x=offset[0],
        raw_offset_y=offset[1],
        displacement_norm=sqrt(quadratic),
        distance_gain=distance_gain,
        gap_gain=gap_gain,
        linear_term=linear,
        quadratic_term=quadratic,
        squared_distance_change=observed,
        squared_prediction=prediction,
        identity_residual=sub(observed, prediction),
        margin_cancellation_residual=sub(gap_gain, distance_gain),
    )
    for name in ("identity_residual", "margin_cancellation_residual"):
        c, lo, hi = quantities[name]
        assert abs(c) <= 1e-10 and lo <= 0 <= hi
    result = {
        f"{name}_{s}": value
        for name in QUANTITIES
        for s, value in zip(("center", "lo", "hi"), quantities[name])
    }
    raw_state, mean_state = (w[k + "_current_state"] for k in ("raw", "mean"))
    result.update(
        distance_gain_state=sign(distance_gain, "increase", "decrease"),
        linear_state=sign(linear, "outward", "inward"),
        point_pair=raw_state + "/" + mean_state,
        raw_blocked_mean_clear=raw_state == "blocked" and mean_state == "clear",
        common_margin=p["decision_point_current_margin"],
    )
    return result


def join(probes, witnesses):
    lookup = {w["edge_id"]: w for w in witnesses}
    assert len(lookup) == len(witnesses)
    assert len({p["edge_id"] for p in probes}) == len(probes)
    assert set(lookup) == {p["edge_id"] for p in probes}
    result = []
    for p in probes:
        w = lookup[p["edge_id"]]
        assert all(w[k] == p[k] for k in (*META, "edge_id", "multiplicity"))
        assert w["event_id"] == p["decision_event_id"] == p["event_ids"]
        assert (
            inherited.step(w["time_s"])
            == int(p["source_step"])
            == inherited.step(p["source_time_s"])
        )
        assert w["phase"] == p["decision_phase"] and w["branch"] == p["decision_branch"]
        assert w["probe_origin"] == p["decision_probe_origin"]
        assert w["obstacle_id"] == p["selected_blocker_ids"]
        assert p["blocker_rows"] == w["obstacle_order"] == "1"
        result.append(
            dict(p, **{"witness_" + k: v for k, v in w.items()}, **compute(p, w))
        )
    return result


def summaries(rows):
    output = []
    for case in ("all_cases", *sorted({r["case_id"] for r in rows})):
        for origin in ("all_origins", *ORIGINS):
            selected = [
                r
                for r in rows
                if case in ("all_cases", r["case_id"])
                and origin in ("all_origins", r["decision_probe_origin"])
            ]
            counts = dict(
                physical_resets=len(selected),
                overlapping_endings=sum(int(r["multiplicity"]) for r in selected),
                raw_blocked_mean_clear=sum(
                    r["raw_blocked_mean_clear"] for r in selected
                ),
            )
            for prefix, field, states in (
                ("gain_", "distance_gain_state", ("increase", "decrease", "unknown")),
                ("linear_", "linear_state", ("outward", "inward", "unknown")),
            ):
                counts.update(
                    {prefix + s: sum(r[field] == s for r in selected) for s in states}
                )
            extrema = {
                f"{name}_{q}_{s}": op((r[q + "_" + s] for r in selected), default=None)
                for q in ("distance_gain", "displacement_norm")
                for name, op in (("min", min), ("max", max))
                for s in ("lo", "hi")
            }
            output.append(
                dict(case_selector=case, origin_selector=origin, **counts, **extrema)
            )
    return output


def closure(rows):
    by = {(r["case_selector"], r["origin_selector"]): r for r in rows}
    checks = 0
    for (case, origin), r in by.items():
        partitions = []
        if case == "all_cases":
            partitions.append(
                [v for (c, o), v in by.items() if c != case and o == origin]
            )
        if origin == "all_origins":
            partitions.append([by[case, o] for o in ORIGINS])
        for parts in partitions:
            for f in COUNTS:
                assert r[f] == sum(p[f] for p in parts)
                checks += 1
    return checks


def baseline():
    inherited.baseline()
    freeze = json.loads((STUDY / "freeze-pins.json").read_text())
    assert freeze["base_main"] == BASE_MAIN
    assert freeze["sources_sha256"] == sha(BASE / "sources.json") == BASE_SHA
    assert (
        freeze["proof_sha256"]
        == sha(BASE / "independent-verification.json")
        == PROOF_SHA
    )
    parent = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    assert proof["sources_sha256"] == BASE_SHA
    assert (
        proof["script_sha256"]
        == freeze["verifier_helper_sha256"]
        == sha(ROOT / "verify_reset_probes.py")
    )
    for f, path in (
        ("script_sha256", ROOT / "analyze_reset_probes.py"),
        ("protocol_sha256", BASE / "protocol.md"),
        ("freeze_sha256", BASE / "freeze-pins.json"),
    ):
        assert parent["analysis"][f] == sha(path)
    helpers = {
        **parent["analysis"]["helpers_sha256"],
        "analyze_reset_probes.py": parent["analysis"]["script_sha256"],
    }
    vhelpers = {
        **parent["analysis"]["verifier_helpers_sha256"],
        "verify_reset_probes.py": freeze["verifier_helper_sha256"],
    }
    for name, h in {**helpers, **vhelpers}.items():
        assert sha(ROOT / name) == h
    for name, h in parent["analysis"]["code_sha256"].items():
        assert sha(REPO / name) == h
    for name, h in {**parent["input_sha256"]}.items():
        assert sha(RESULTS / name) == h
    for name, h in parent["output_sha256"].items():
        assert sha(BASE / name) == h
    plots = json.loads((BASE / "plot-sources.json").read_text())
    assert sha(BASE / "plot-sources.json") == proof["plot_sources_sha256"]
    assert plots["script_sha256"] == sha(ROOT / "plot_reset_probes.py")
    for name, h in {**plots["input_sha256"], **plots["output_sha256"]}.items():
        assert sha(BASE / name) == h
    pins = {
        **parent["input_sha256"],
        **{"fusion-return-reset-probes/" + p.name: sha(p) for p in BASE.iterdir()},
    }
    return parent, pins, helpers, vhelpers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Use a fresh output directory without -O")
    parent, pins, helpers, vhelpers = baseline()
    probes, witnesses = [
        read(BASE / n) for n in ("reset-probes.csv", "reset-blockers.csv")
    ]
    rows = join(probes, witnesses)
    summary = summaries(rows)
    assert len(rows) == len(witnesses) == 19 and len(summary) == 20
    audit = dict(
        physical_resets=len(rows),
        overlapping_endings=sum(int(r["multiplicity"]) for r in rows),
        blocker_rows=len(witnesses),
        origins=dict(Counter(r["decision_probe_origin"] for r in rows)),
        distance_gain_states=dict(Counter(r["distance_gain_state"] for r in rows)),
        linear_states=dict(Counter(r["linear_state"] for r in rows)),
        point_pairs=dict(Counter(r["point_pair"] for r in rows)),
        summary_rows=len(summary),
        additive_closure_comparisons=closure(summary),
        fresh_displacement_pairs=len(rows),
        fresh_raw_reads=0,
        fresh_native_runs=0,
    )
    assert audit["overlapping_endings"] == 185
    args.output.mkdir(parents=True)
    write(
        args.output / "displacements.csv",
        rows,
        (*probes[0], *("witness_" + f for f in witnesses[0]), *EXTRA),
    )
    write(args.output / "displacement-summary.csv", summary, SUMMARY)
    source = dict(
        input_sha256=pins,
        analysis=dict(
            script_sha256=sha(Path(__file__)),
            protocol_sha256=sha(STUDY / "protocol.md"),
            freeze_sha256=sha(STUDY / "freeze-pins.json"),
            helpers_sha256=helpers,
            verifier_helpers_sha256=vhelpers,
            code_sha256=parent["analysis"]["code_sha256"],
        ),
        schema=SCHEMA,
        inherited_reset_probes_proof_sha256=PROOF_SHA,
        inherited_geometry_proofs=parent["inherited_geometry_proofs"],
        audit=audit,
        output_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
