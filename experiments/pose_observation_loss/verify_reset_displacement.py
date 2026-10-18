#!/usr/bin/env python3
"""Independently verify the 19 spatial comparisons with 60-digit Decimal."""

import argparse
from collections import Counter
from decimal import Decimal as D, localcontext
import json
import math
from pathlib import Path

import verify_reset_probes as inherited

core = inherited.core
ROOT, REPO, RESULTS = core.ROOT, core.REPO, core.RESULTS
BASE = RESULTS / "fusion-return-reset-probes"
STUDY = RESULTS / "fusion-return-reset-displacement"
BASE_MAIN = "810409fa265762d614261016c85f8cd398365ab6"
BASE_SHA = "43e4109d5b9adbbc30acefe0a9583feb7724825c49e9448eb0d7d493e21a0858"
PROOF_SHA = "ba1acd798c92798c224de7759cbab28344b717d989bfbcfc5fa8018b2bd64122"
HELPERS = {
    **inherited.HELPERS,
    "verify_reset_probes.py": "bc868ba6f8449373e89d3f5fcde9b204a659b3d1353deb244e72fba02e62d332",
}
META = inherited.META
ORIGINS = ("initial_unsafe", "first_chord_by_exclusion", "ambiguous_zero")
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
P = D("1e-12")
read = inherited.read


def scalar(text):
    value = D.from_float(float(text))
    assert value.is_finite()
    return value


class Interval:
    def __init__(self, center, low, high):
        self.c, self.a, self.b = center, low, high
        assert all(v.is_finite() for v in (center, low, high))
        assert low <= center <= high

    @classmethod
    def from_row(cls, row, field):
        return cls(*(scalar(row[field + "_" + s]) for s in ("center", "lo", "hi")))

    @classmethod
    def exact(cls, text):
        v = scalar(text)
        return cls(v, v, v)

    def __add__(self, other):
        return Interval(self.c + other.c, self.a + other.a - P, self.b + other.b + P)

    def __sub__(self, other):
        return Interval(self.c - other.c, self.a - other.b - P, self.b - other.a + P)

    def __mul__(self, other):
        ends = [self.a * other.a, self.a * other.b, self.b * other.a, self.b * other.b]
        return Interval(self.c * other.c, min(ends) - P, max(ends) + P)

    def squared(self):
        lower = D(0) if self.a <= 0 <= self.b else min(self.a * self.a, self.b * self.b)
        return Interval(
            self.c * self.c,
            max(D(0), lower - P),
            max(self.a * self.a, self.b * self.b) + P,
        )

    def nonnegative(self):
        return Interval(self.c, max(D(0), self.a), self.b)

    def root(self):
        assert self.a >= 0
        return Interval(self.c.sqrt(), max(D(0), self.a.sqrt() - P), self.b.sqrt() + P)

    def direction(self, positive, negative):
        if self.b < 0:
            return negative
        return positive if self.a > 0 else "unknown"


def close(a, b):
    assert abs(a - b) <= max(D("1e-12"), D("1e-10") * max(abs(a), abs(b)))


def compute(probe, witness):
    with localcontext() as context:
        context.prec = 60
        raw_x = Interval.from_row(probe, "decision_point_raw_current_x")
        raw_y = Interval.from_row(probe, "decision_point_raw_current_y")
        mean_x = Interval.from_row(probe, "decision_point_mean_current_x")
        mean_y = Interval.from_row(probe, "decision_point_mean_current_y")
        ox, oy = (Interval.exact(witness["map_" + axis]) for axis in ("x", "y"))
        assert ox.c == scalar(witness["obstacle_x"]) and oy.c == scalar(
            witness["obstacle_y"]
        )
        margin = Interval(
            *(
                scalar(probe["decision_point_current_margin" + s])
                for s in ("", "_lo", "_hi")
            )
        )
        assert margin.a >= 0
        rd = Interval.from_row(witness, "raw_current_distance")
        md = Interval.from_row(witness, "mean_current_distance")
        rg = Interval.from_row(witness, "raw_current_gap")
        mg = Interval.from_row(witness, "mean_current_gap")
        for x, y, distance, gap, kind in (
            (raw_x, raw_y, rd, rg, "raw"),
            (mean_x, mean_y, md, mg, "mean"),
        ):
            assert distance.a >= 0
            calculated = ((x.c - ox.c) ** 2 + (y.c - oy.c) ** 2).sqrt()
            close(calculated, distance.c)
            assert abs(distance.c - gap.c - margin.c) <= D("1e-10")
            assert witness[kind + "_current_state"] == inherited.point_state(
                str(gap.a), str(gap.b)
            )
        vx, vy, ux, uy = mean_x - raw_x, mean_y - raw_y, raw_x - ox, raw_y - oy
        linear = Interval.exact(2) * (ux * vx + uy * vy)
        quadratic = (vx.squared() + vy.squared()).nonnegative()
        change = md.squared() - rd.squared()
        predicted = linear + quadratic
        gain, gap_gain = md - rd, mg - rg
        values = dict(
            delta_x=vx,
            delta_y=vy,
            raw_offset_x=ux,
            raw_offset_y=uy,
            displacement_norm=quadratic.root(),
            distance_gain=gain,
            gap_gain=gap_gain,
            linear_term=linear,
            quadratic_term=quadratic,
            squared_distance_change=change,
            squared_prediction=predicted,
            identity_residual=change - predicted,
            margin_cancellation_residual=gap_gain - gain,
        )
        for name in ("identity_residual", "margin_cancellation_residual"):
            assert (
                abs(values[name].c) <= D("1e-10")
                and values[name].a <= 0 <= values[name].b
            )
        output = {}
        for name in QUANTITIES:
            b = values[name]
            output.update({name + "_center": b.c, name + "_lo": b.a, name + "_hi": b.b})
        rs, ms = witness["raw_current_state"], witness["mean_current_state"]
        output.update(
            distance_gain_state=gain.direction("increase", "decrease"),
            linear_state=linear.direction("outward", "inward"),
            point_pair=rs + "/" + ms,
            raw_blocked_mean_clear=(rs == "blocked" and ms == "clear"),
            common_margin=probe["decision_point_current_margin"],
        )
        return output


def join(probes, witnesses):
    pids, wids = [p["edge_id"] for p in probes], [w["edge_id"] for w in witnesses]
    assert len(set(pids)) == len(pids) and len(set(wids)) == len(wids)
    assert set(pids) == set(wids)
    output = []
    for p in probes:
        matches = [w for w in witnesses if w["edge_id"] == p["edge_id"]]
        assert len(matches) == 1
        w = matches[0]
        for f in (*META, "edge_id", "multiplicity"):
            core.check(w[f], p[f])
        core.check(w["event_id"], p["decision_event_id"])
        core.check(w["event_id"], p["event_ids"])
        assert (
            inherited.step(w["time_s"])
            == int(p["source_step"])
            == inherited.step(p["source_time_s"])
        )
        core.check(w["phase"], p["decision_phase"])
        core.check(w["branch"], p["decision_branch"])
        core.check(w["probe_origin"], p["decision_probe_origin"])
        core.check(w["obstacle_id"], p["selected_blocker_ids"])
        core.check(p["blocker_rows"], 1)
        core.check(w["obstacle_order"], 1)
        row = p.copy()
        row.update(("witness_" + f, text) for f, text in w.items())
        row.update(compute(p, w))
        output.append(row)
    return output


def summaries(rows):
    output = []
    for case in ["all_cases"] + sorted(set(r["case_id"] for r in rows)):
        for origin in ["all_origins", *ORIGINS]:
            selected = [
                r
                for r in rows
                if (case == "all_cases" or r["case_id"] == case)
                and (origin == "all_origins" or r["decision_probe_origin"] == origin)
            ]
            out = dict(
                case_selector=case,
                origin_selector=origin,
                physical_resets=len(selected),
                overlapping_endings=sum(int(r["multiplicity"]) for r in selected),
                raw_blocked_mean_clear=sum(
                    1 for r in selected if r["raw_blocked_mean_clear"]
                ),
            )
            for state in ("increase", "decrease", "unknown"):
                out["gain_" + state] = len(
                    [r for r in selected if r["distance_gain_state"] == state]
                )
            for state in ("outward", "inward", "unknown"):
                out["linear_" + state] = len(
                    [r for r in selected if r["linear_state"] == state]
                )
            for quantity in ("distance_gain", "displacement_norm"):
                for side in ("lo", "hi"):
                    bounds = sorted(r[quantity + "_" + side] for r in selected)
                    out["min_" + quantity + "_" + side] = bounds[0] if bounds else None
                    out["max_" + quantity + "_" + side] = bounds[-1] if bounds else None
            output.append(out)
    return output


def closure(rows):
    checks = 0
    for row in rows:
        c, o = row["case_selector"], row["origin_selector"]
        partitions = []
        if c == "all_cases":
            partitions.append(
                [
                    r
                    for r in rows
                    if r["case_selector"] != c and r["origin_selector"] == o
                ]
            )
        if o == "all_origins":
            partitions.append(
                [
                    r
                    for r in rows
                    if r["case_selector"] == c and r["origin_selector"] in ORIGINS
                ]
            )
        for group in partitions:
            for f in COUNTS:
                assert row[f] == sum(r[f] for r in group)
                checks += 1
    return checks


def compare(path, expected, keys):
    actual = read(path)
    assert [tuple(a[k] for k in keys) for a in actual] == [
        tuple(str(b[k]) for k in keys) for b in expected
    ]
    for a, b in zip(actual, expected):
        assert set(a) == set(b)
        for f, value in b.items():
            if isinstance(value, D):
                observed = float(a[f])
                assert math.isfinite(observed) and math.isclose(
                    observed, float(value), abs_tol=1e-12, rel_tol=1e-10
                )
                core.comparisons += 1
            else:
                core.check(a[f], value)
        if "delta_x_center" in a:
            boxes = {q: Interval.from_row(a, q) for q in QUANTITIES}
            assert (
                boxes["distance_gain"].direction("increase", "decrease")
                == a["distance_gain_state"]
            )
            assert (
                boxes["linear_term"].direction("outward", "inward") == a["linear_state"]
            )
            for f in ("displacement_norm", "quadratic_term"):
                assert boxes[f].a >= 0
            for f in ("identity_residual", "margin_cancellation_residual"):
                assert abs(boxes[f].c) <= D("1e-10") and boxes[f].a <= 0 <= boxes[f].b


def provenance(results):
    parent = inherited.provenance(BASE)
    source = json.loads((results / "sources.json").read_text())
    freeze = json.loads((STUDY / "freeze-pins.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    assert freeze["base_main"] == BASE_MAIN
    assert freeze["sources_sha256"] == core.sha(BASE / "sources.json") == BASE_SHA
    assert (
        freeze["proof_sha256"]
        == core.sha(BASE / "independent-verification.json")
        == PROOF_SHA
    )
    assert proof["sources_sha256"] == BASE_SHA
    assert (
        proof["script_sha256"]
        == freeze["verifier_helper_sha256"]
        == HELPERS["verify_reset_probes.py"]
    )
    pins = {
        **parent["input_sha256"],
        **{"fusion-return-reset-probes/" + p.name: core.sha(p) for p in BASE.iterdir()},
    }
    assert source["input_sha256"] == pins
    for name, h in pins.items():
        assert core.sha(RESULTS / name) == h
    helpers = {
        **parent["analysis"]["helpers_sha256"],
        "analyze_reset_probes.py": parent["analysis"]["script_sha256"],
    }
    assert source["analysis"]["helpers_sha256"] == helpers
    assert source["analysis"]["verifier_helpers_sha256"] == HELPERS
    for name, h in {**helpers, **HELPERS}.items():
        assert core.sha(ROOT / name) == h
    assert source["analysis"]["code_sha256"] == parent["analysis"]["code_sha256"]
    for name, h in source["analysis"]["code_sha256"].items():
        assert core.sha(REPO / name) == h
    for f, path in (
        ("script_sha256", ROOT / "analyze_reset_displacement.py"),
        ("protocol_sha256", STUDY / "protocol.md"),
        ("freeze_sha256", STUDY / "freeze-pins.json"),
    ):
        assert source["analysis"][f] == core.sha(path)
    plots = json.loads((BASE / "plot-sources.json").read_text())
    assert core.sha(BASE / "plot-sources.json") == proof["plot_sources_sha256"]
    assert plots["script_sha256"] == core.sha(ROOT / "plot_reset_probes.py")
    for name, h in {**plots["input_sha256"], **plots["output_sha256"]}.items():
        assert core.sha(BASE / name) == h
    assert source["schema"] == dict(
        origins=list(ORIGINS),
        retrospective=True,
        elemental_padding=1e-12,
        quantities=list(QUANTITIES),
        selection="19 preceding exact reset edges and their original blocker witnesses; 185 overlapping endings; no new trials or weights",
        interpretation="same-step spatial estimate comparison; not temporal robot motion",
    )
    assert source["inherited_reset_probes_proof_sha256"] == PROOF_SHA
    assert source["inherited_geometry_proofs"] == parent["inherited_geometry_proofs"]
    assert set(source["output_sha256"]) == {
        "displacements.csv",
        "displacement-summary.csv",
    }
    for name, h in source["output_sha256"].items():
        assert core.sha(results / name) == h
    return source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("results", "figures", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Use an unused evidence filename without -O")
    source = provenance(args.results)
    cases, canonical = core.original_inputs()
    anchors = read(RESULTS / "fusion-return-run-breaks/anchor-breaks.csv")
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
    probes, witnesses = [
        read(BASE / n) for n in ("reset-probes.csv", "reset-blockers.csv")
    ]
    rows = join(probes, witnesses)
    summary = summaries(rows)
    assert (
        len(cases) == 972
        and len(anchors) == 3997
        and len(rows) == len(witnesses) == 19
        and len(summary) == 20
    )
    for name, expected, keys in (
        ("displacements.csv", rows, ("edge_id",)),
        ("displacement-summary.csv", summary, ("case_selector", "origin_selector")),
    ):
        compare(args.results / name, expected, keys)
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
    assert audit == source["audit"] and audit["overlapping_endings"] == 185
    plots = json.loads((args.figures / "plot-sources.json").read_text())
    assert plots["script_sha256"] == core.sha(ROOT / "plot_reset_displacement.py")
    assert plots["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in ("sources.json", "displacements.csv", "displacement-summary.csv")
    }
    assert set(plots["output_sha256"]) == {
        "reset-displacement.svg",
        "plot-displacements.csv",
        "plot-values.csv",
    }
    for name, h in plots["output_sha256"].items():
        assert core.sha(args.figures / name) == h
    compare(args.figures / "plot-displacements.csv", rows, ("edge_id",))
    compare(
        args.figures / "plot-values.csv", summary, ("case_selector", "origin_selector")
    )
    evidence = dict(
        **audit,
        comparisons=core.comparisons - original_checks,
        original_join_comparisons=original_checks,
        script_sha256=core.sha(Path(__file__)),
        sources_sha256=core.sha(args.results / "sources.json"),
        plot_sources_sha256=core.sha(args.figures / "plot-sources.json"),
        inherited_reset_probes_proof_sha256=PROOF_SHA,
        inherited_geometry_proofs=source["inherited_geometry_proofs"],
        method="independent exact joins; 60-digit Decimal spatial interval arithmetic, sign/identity/common-margin checks, extrema and additive count closures; original census identities; no new raw/native dataset execution",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
