#!/usr/bin/env python3
"""Independently verify radial/transverse projection and finite-distance identities."""

import argparse
from collections import Counter
from decimal import Decimal as D, localcontext
import json
from pathlib import Path

import verify_reset_displacement as inherited

core, Interval = inherited.core, inherited.Interval
ROOT, REPO, RESULTS = core.ROOT, core.REPO, core.RESULTS
BASE = RESULTS / "fusion-return-reset-displacement"
STUDY = RESULTS / "fusion-return-reset-projection"
BASE_MAIN = "1362b968aadf61b4173a28664836b975fac42dd1"
BASE_SHA = "5c41eca356ae027aa9fe062a7c60fcf9bc8949099951686f9589be20f1e752c1"
PROOF_SHA = "0672f7f6616aab12dd07fe9341cb5f7a535a2fec814114c4e2f88020ac7eb326"
HELPERS = {
    **inherited.HELPERS,
    "verify_reset_displacement.py": "0a41847816812846a80b024f4a1dc3c24a5806f361f07e06c83d868e99ba2c57",
}
ORIGINS = ("initial_unsafe", "first_chord_by_exclusion", "ambiguous_zero")
STATUSES = ("complete", "unresolved_raw_distance", "unresolved_remainder_denominator")
QUANTITIES = (
    "radial_component",
    "transverse_component",
    "transverse_abs",
    "projection_norm_squared",
    "projection_norm_residual",
    "radial_endpoint",
    "reconstructed_distance",
    "reconstruction_residual",
    "distance_gain_beyond_radial",
    "rational_denominator",
    "rational_remainder",
    "remainder_residual",
)
COUNTS = (
    "physical_resets",
    "overlapping_endings",
    *STATUSES,
    "radial_outward",
    "radial_inward",
    "radial_unknown",
    "transverse_positive",
    "transverse_negative",
    "transverse_unknown",
    "remainder_positive",
    "remainder_negative",
    "remainder_unknown",
)
EXTREMA_QUANTITIES = (
    "radial_component",
    "transverse_abs",
    "distance_gain_beyond_radial",
)
P = D("1e-12")
read = inherited.read


def divide(a, b):
    assert not b.a <= 0 <= b.b
    extremes = [a.a / b.a, a.a / b.b, a.b / b.a, a.b / b.b]
    return Interval(a.c / b.c, min(extremes) - P, max(extremes) + P)


def absolute(b):
    lower = D(0) if b.a <= 0 <= b.b else min(abs(b.a), abs(b.b))
    return Interval(abs(b.c), max(D(0), lower - P), max(abs(b.a), abs(b.b)) + P)


def compute(row):
    with localcontext() as context:
        context.prec = 60
        ux, uy = (Interval.from_row(row, "raw_offset_" + axis) for axis in ("x", "y"))
        vx, vy = (Interval.from_row(row, "delta_" + axis) for axis in ("x", "y"))
        raw = Interval.from_row(row, "witness_raw_current_distance")
        mean = Interval.from_row(row, "witness_mean_current_distance")
        norm2 = Interval.from_row(row, "quadratic_term")
        gain = Interval.from_row(row, "distance_gain")
        assert raw.a >= 0 and mean.a >= 0 and norm2.a >= 0
        quantities = dict.fromkeys(QUANTITIES)
        status, radial, transverse, remainder = (
            "unresolved_raw_distance",
            "unknown",
            "unknown",
            "unknown",
        )
        if raw.a > 0:
            along = divide(ux * vx + uy * vy, raw)
            across = divide(ux * vy - uy * vx, raw)
            projection2 = (along.squared() + across.squared()).nonnegative()
            endpoint = raw + along
            reconstructed = (endpoint.squared() + across.squared()).nonnegative().root()
            excess = gain - along
            denominator = mean + endpoint
            quantities.update(
                radial_component=along,
                transverse_component=across,
                transverse_abs=absolute(across),
                projection_norm_squared=projection2,
                projection_norm_residual=norm2 - projection2,
                radial_endpoint=endpoint,
                reconstructed_distance=reconstructed,
                reconstruction_residual=mean - reconstructed,
                distance_gain_beyond_radial=excess,
                rational_denominator=denominator,
            )
            status = "unresolved_remainder_denominator"
            radial = along.direction("outward", "inward")
            transverse = across.direction("positive", "negative")
            remainder = excess.direction("positive", "negative")
            if denominator.a > 0:
                rational = divide(across.squared(), denominator)
                quantities.update(
                    rational_remainder=rational, remainder_residual=excess - rational
                )
                status = "complete"
        for name in (
            "projection_norm_residual",
            "reconstruction_residual",
            "remainder_residual",
        ):
            b = quantities[name]
            if b is not None:
                assert abs(b.c) <= D("1e-10") and b.a <= 0 <= b.b
        output = dict(
            projection_status=status,
            radial_state=radial,
            transverse_state=transverse,
            remainder_state=remainder,
        )
        for name in QUANTITIES:
            b = quantities[name]
            for side, value in zip(
                ("center", "lo", "hi"),
                (None, None, None) if b is None else (b.c, b.a, b.b),
            ):
                output[name + "_" + side] = value
        return output


def project(rows):
    identities = [r["edge_id"] for r in rows]
    assert len(set(identities)) == len(identities)
    output = []
    for row in rows:
        projected = row.copy()
        projected.update(compute(row))
        output.append(projected)
    return output


def summaries(rows):
    output = []
    cases = ["all_cases"] + sorted(set(r["case_id"] for r in rows))
    for case in cases:
        for origin in ["all_origins", *ORIGINS]:
            subset = [
                r
                for r in rows
                if (case == "all_cases" or r["case_id"] == case)
                and (origin == "all_origins" or r["decision_probe_origin"] == origin)
            ]
            row = dict(
                case_selector=case,
                origin_selector=origin,
                physical_resets=len(subset),
                overlapping_endings=sum(int(r["multiplicity"]) for r in subset),
            )
            for status in STATUSES:
                row[status] = len(
                    [r for r in subset if r["projection_status"] == status]
                )
            for category, options in (
                ("radial", ("outward", "inward", "unknown")),
                ("transverse", ("positive", "negative", "unknown")),
                ("remainder", ("positive", "negative", "unknown")),
            ):
                for choice in options:
                    row[category + "_" + choice] = len(
                        [r for r in subset if r[category + "_state"] == choice]
                    )
            for q in EXTREMA_QUANTITIES:
                for side in ("lo", "hi"):
                    available = sorted(
                        r[q + "_" + side]
                        for r in subset
                        if r[q + "_" + side] is not None
                    )
                    row["min_" + q + "_" + side] = available[0] if available else None
                    row["max_" + q + "_" + side] = available[-1] if available else None
            output.append(row)
    return output


def closure(rows):
    checks = 0
    for row in rows:
        assert row["physical_resets"] == sum(row[s] for s in STATUSES)
        for category, options in (
            ("radial", ("outward", "inward", "unknown")),
            ("transverse", ("positive", "negative", "unknown")),
            ("remainder", ("positive", "negative", "unknown")),
        ):
            assert row["physical_resets"] == sum(
                row[category + "_" + s] for s in options
            )
        c, o = row["case_selector"], row["origin_selector"]
        groups = []
        if c == "all_cases":
            groups.append(
                [
                    r
                    for r in rows
                    if r["case_selector"] != c and r["origin_selector"] == o
                ]
            )
        if o == "all_origins":
            groups.append(
                [
                    r
                    for r in rows
                    if r["case_selector"] == c and r["origin_selector"] in ORIGINS
                ]
            )
        for group in groups:
            for f in COUNTS:
                assert row[f] == sum(r[f] for r in group)
                checks += 1
    return checks


def compare(path, expected, keys):
    inherited.compare(path, expected, keys)
    for row in read(path):
        if "projection_status" not in row:
            continue
        boxes = {}
        for name in QUANTITIES:
            fields = [row[name + "_" + s] for s in ("center", "lo", "hi")]
            assert all(f != "" for f in fields) or all(f == "" for f in fields)
            boxes[name] = Interval.from_row(row, name) if fields[0] else None
        if row["projection_status"] == "unresolved_raw_distance":
            assert all(b is None for b in boxes.values())
            assert all(
                row[k + "_state"] == "unknown"
                for k in ("radial", "transverse", "remainder")
            )
        else:
            for key, name, positive, negative in (
                ("radial", "radial_component", "outward", "inward"),
                ("transverse", "transverse_component", "positive", "negative"),
                ("remainder", "distance_gain_beyond_radial", "positive", "negative"),
            ):
                assert boxes[name].direction(positive, negative) == row[key + "_state"]
            for name in (
                "projection_norm_squared",
                "transverse_abs",
                "reconstructed_distance",
            ):
                assert boxes[name].a >= 0
            for name in (
                "projection_norm_residual",
                "reconstruction_residual",
                "remainder_residual",
            ):
                b = boxes[name]
                if b is not None:
                    assert abs(b.c) <= D("1e-10") and b.a <= 0 <= b.b
            if row["projection_status"] == "complete":
                assert (
                    all(b is not None for b in boxes.values())
                    and boxes["rational_denominator"].a > 0
                )
            else:
                assert row["projection_status"] == "unresolved_remainder_denominator"
                assert boxes["rational_denominator"].a <= 0
                assert {name for name, b in boxes.items() if b is None} == {
                    "rational_remainder",
                    "remainder_residual",
                }


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
        == HELPERS["verify_reset_displacement.py"]
    )
    assert len(list(BASE.iterdir())) == 11
    pins = {
        **parent["input_sha256"],
        **{
            "fusion-return-reset-displacement/" + p.name: core.sha(p)
            for p in BASE.iterdir()
        },
    }
    assert source["input_sha256"] == pins
    for name, digest in pins.items():
        assert core.sha(RESULTS / name) == digest
    helpers = {
        **parent["analysis"]["helpers_sha256"],
        "analyze_reset_displacement.py": parent["analysis"]["script_sha256"],
    }
    assert (
        source["analysis"]["helpers_sha256"] == helpers
        and source["analysis"]["verifier_helpers_sha256"] == HELPERS
    )
    for name, digest in {**helpers, **HELPERS}.items():
        assert core.sha(ROOT / name) == digest
    assert source["analysis"]["code_sha256"] == parent["analysis"]["code_sha256"]
    for name, digest in source["analysis"]["code_sha256"].items():
        assert core.sha(REPO / name) == digest
    for field, path in (
        ("script_sha256", ROOT / "analyze_reset_projection.py"),
        ("protocol_sha256", STUDY / "protocol.md"),
        ("freeze_sha256", STUDY / "freeze-pins.json"),
    ):
        assert source["analysis"][field] == core.sha(path)
    plots = json.loads((BASE / "plot-sources.json").read_text())
    assert core.sha(BASE / "plot-sources.json") == proof["plot_sources_sha256"]
    assert plots["script_sha256"] == core.sha(ROOT / "plot_reset_displacement.py")
    for name, digest in {**plots["input_sha256"], **plots["output_sha256"]}.items():
        assert core.sha(BASE / name) == digest
    assert source["schema"] == dict(
        origins=list(ORIGINS),
        statuses=list(STATUSES),
        quantities=list(QUANTITIES),
        retrospective=True,
        elemental_padding=1e-12,
        selection="same 19 reset/blocker pairs and 185 overlapping endings; no new trials or weights",
        interpretation="same-step spatial projections in recorded x/y convention; not temporal motion",
    )
    assert source["inherited_reset_displacement_proof_sha256"] == PROOF_SHA
    assert source["inherited_geometry_proofs"] == parent["inherited_geometry_proofs"]
    assert set(source["output_sha256"]) == {"projections.csv", "projection-summary.csv"}
    for name, digest in source["output_sha256"].items():
        assert core.sha(results / name) == digest
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
    assert set(canonical) == {r["event_id"] for r in anchors}
    for r in anchors:
        for f in (
            *inherited.META,
            "received",
            "parent_rp_records",
            "rp_records",
            "rp_different_preview_records",
        ):
            core.check(r[f], canonical[r["event_id"]][f])
    original_checks = core.comparisons
    previous = read(BASE / "displacements.csv")
    rows = project(previous)
    summary = summaries(rows)
    assert (
        len(cases) == 972
        and len(anchors) == 3997
        and len(rows) == 19
        and len(summary) == 20
    )
    compare(args.results / "projections.csv", rows, ("edge_id",))
    compare(
        args.results / "projection-summary.csv",
        summary,
        ("case_selector", "origin_selector"),
    )
    audit = dict(
        physical_resets=len(rows),
        overlapping_endings=sum(int(r["multiplicity"]) for r in rows),
        origins=dict(Counter(r["decision_probe_origin"] for r in rows)),
        projection_statuses=dict(Counter(r["projection_status"] for r in rows)),
        radial_states=dict(Counter(r["radial_state"] for r in rows)),
        transverse_states=dict(Counter(r["transverse_state"] for r in rows)),
        remainder_states=dict(Counter(r["remainder_state"] for r in rows)),
        summary_rows=len(summary),
        additive_closure_comparisons=closure(summary),
        fresh_projection_pairs=len(rows),
        fresh_raw_reads=0,
        fresh_native_runs=0,
    )
    assert audit == source["audit"] and audit["overlapping_endings"] == 185
    plot = json.loads((args.figures / "plot-sources.json").read_text())
    assert plot["script_sha256"] == core.sha(ROOT / "plot_reset_projection.py")
    assert plot["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in ("sources.json", "projections.csv", "projection-summary.csv")
    }
    assert set(plot["output_sha256"]) == {
        "reset-projection.svg",
        "plot-projections.csv",
        "plot-values.csv",
    }
    for name, digest in plot["output_sha256"].items():
        assert core.sha(args.figures / name) == digest
    compare(args.figures / "plot-projections.csv", rows, ("edge_id",))
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
        inherited_reset_displacement_proof_sha256=PROOF_SHA,
        inherited_geometry_proofs=source["inherited_geometry_proofs"],
        method="independent 60-digit Decimal projections, guarded interval division, norm/distance/remainder identities, strict signs, extrema/count closures and original census identities; no new raw/native dataset execution",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
