#!/usr/bin/env python3
"""Project the same 19 spatial estimate differences onto radial/transverse axes."""

import argparse
from collections import Counter
import json
from pathlib import Path

import analyze_reset_displacement as inherited

ROOT, REPO, RESULTS = inherited.ROOT, inherited.REPO, inherited.RESULTS
BASE = RESULTS / "fusion-return-reset-displacement"
STUDY = RESULTS / "fusion-return-reset-projection"
BASE_MAIN = "1362b968aadf61b4173a28664836b975fac42dd1"
BASE_SHA = "5c41eca356ae027aa9fe062a7c60fcf9bc8949099951686f9589be20f1e752c1"
PROOF_SHA = "0672f7f6616aab12dd07fe9341cb5f7a535a2fec814114c4e2f88020ac7eb326"
P, ORIGINS = inherited.P, inherited.ORIGINS
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
EXTRA = tuple(f"{q}_{s}" for q in QUANTITIES for s in ("center", "lo", "hi")) + (
    "projection_status",
    "radial_state",
    "transverse_state",
    "remainder_state",
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
EXTREMA = tuple(
    f"{op}_{q}_{s}"
    for q in EXTREMA_QUANTITIES
    for op in ("min", "max")
    for s in ("lo", "hi")
)
SUMMARY = ("case_selector", "origin_selector", *COUNTS, *EXTREMA)
SCHEMA = dict(
    origins=ORIGINS,
    statuses=STATUSES,
    quantities=QUANTITIES,
    retrospective=True,
    elemental_padding=P,
    selection="same 19 reset/blocker pairs and 185 overlapping endings; no new trials or weights",
    interpretation="same-step spatial projections in recorded x/y convention; not temporal motion",
)
read, write, sha = inherited.read, inherited.write, inherited.sha
box, load = inherited.box, inherited.load
add, sub, mul = inherited.add, inherited.sub, inherited.mul
square, sqrt, positive_sum = inherited.square, inherited.sqrt, inherited.positive_sum


def divide(numerator, denominator):
    assert denominator[2] < 0 or denominator[1] > 0
    ends = [a / b for a in numerator[1:] for b in denominator[1:]]
    return box(numerator[0] / denominator[0], min(ends) - P, max(ends) + P)


def absolute(value):
    c, lo, hi = value
    lower = 0 if lo <= 0 <= hi else min(abs(lo), abs(hi))
    return box(abs(c), max(0, lower - P), max(abs(lo), abs(hi)) + P)


def compute(row):
    ux, uy, vx, vy, r, d, q, g = [
        load(row, f)
        for f in (
            "raw_offset_x",
            "raw_offset_y",
            "delta_x",
            "delta_y",
            "witness_raw_current_distance",
            "witness_mean_current_distance",
            "quadratic_term",
            "distance_gain",
        )
    ]
    assert r[1] >= 0 and d[1] >= 0 and q[1] >= 0
    values = dict.fromkeys(QUANTITIES)
    output = dict(
        projection_status="unresolved_raw_distance",
        radial_state="unknown",
        transverse_state="unknown",
        remainder_state="unknown",
    )
    if r[1] > 0:
        a = divide(add(mul(ux, vx), mul(uy, vy)), r)
        b = divide(sub(mul(ux, vy), mul(uy, vx)), r)
        projected = positive_sum(square(a), square(b))
        endpoint = add(r, a)
        reconstructed = sqrt(positive_sum(square(endpoint), square(b)))
        beyond = sub(g, a)
        denominator = add(d, endpoint)
        values.update(
            radial_component=a,
            transverse_component=b,
            transverse_abs=absolute(b),
            projection_norm_squared=projected,
            projection_norm_residual=sub(q, projected),
            radial_endpoint=endpoint,
            reconstructed_distance=reconstructed,
            reconstruction_residual=sub(d, reconstructed),
            distance_gain_beyond_radial=beyond,
            rational_denominator=denominator,
        )
        output.update(
            projection_status="unresolved_remainder_denominator",
            radial_state=inherited.sign(a, "outward", "inward"),
            transverse_state=inherited.sign(b, "positive", "negative"),
            remainder_state=inherited.sign(beyond, "positive", "negative"),
        )
        if denominator[1] > 0:
            rational = divide(square(b), denominator)
            values.update(
                rational_remainder=rational, remainder_residual=sub(beyond, rational)
            )
            output["projection_status"] = "complete"
    for f in (
        "projection_norm_residual",
        "reconstruction_residual",
        "remainder_residual",
    ):
        if values[f] is not None:
            c, lo, hi = values[f]
            assert abs(c) <= 1e-10 and lo <= 0 <= hi
    for q in QUANTITIES:
        for s, value in zip(("center", "lo", "hi"), values[q] or (None, None, None)):
            output[q + "_" + s] = value
    return output


def project(rows):
    assert len({r["edge_id"] for r in rows}) == len(rows)
    return [dict(r, **compute(r)) for r in rows]


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
            )
            counts.update(
                {
                    s: sum(r["projection_status"] == s for r in selected)
                    for s in STATUSES
                }
            )
            for kind, states in (
                ("radial", ("outward", "inward", "unknown")),
                ("transverse", ("positive", "negative", "unknown")),
                ("remainder", ("positive", "negative", "unknown")),
            ):
                counts.update(
                    {
                        kind + "_" + s: sum(r[kind + "_state"] == s for r in selected)
                        for s in states
                    }
                )
            extrema = {
                f"{op}_{q}_{s}": fn(
                    (r[q + "_" + s] for r in selected if r[q + "_" + s] is not None),
                    default=None,
                )
                for q in EXTREMA_QUANTITIES
                for op, fn in (("min", min), ("max", max))
                for s in ("lo", "hi")
            }
            output.append(
                dict(case_selector=case, origin_selector=origin, **counts, **extrema)
            )
    return output


def closure(rows):
    by = {(r["case_selector"], r["origin_selector"]): r for r in rows}
    checks = 0
    for (c, o), row in by.items():
        for fields in (
            STATUSES,
            ("radial_outward", "radial_inward", "radial_unknown"),
            ("transverse_positive", "transverse_negative", "transverse_unknown"),
            ("remainder_positive", "remainder_negative", "remainder_unknown"),
        ):
            assert sum(row[f] for f in fields) == row["physical_resets"]
        partitions = []
        if c == "all_cases":
            partitions.append(
                [v for (case, origin), v in by.items() if case != c and origin == o]
            )
        if o == "all_origins":
            partitions.append([by[c, origin] for origin in ORIGINS])
        for parts in partitions:
            for f in COUNTS:
                assert row[f] == sum(p[f] for p in parts)
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
        == sha(ROOT / "verify_reset_displacement.py")
    )
    for field, path in (
        ("script_sha256", ROOT / "analyze_reset_displacement.py"),
        ("protocol_sha256", BASE / "protocol.md"),
        ("freeze_sha256", BASE / "freeze-pins.json"),
    ):
        assert parent["analysis"][field] == sha(path)
    helpers = {
        **parent["analysis"]["helpers_sha256"],
        "analyze_reset_displacement.py": parent["analysis"]["script_sha256"],
    }
    vhelpers = {
        **parent["analysis"]["verifier_helpers_sha256"],
        "verify_reset_displacement.py": freeze["verifier_helper_sha256"],
    }
    for name, digest in {**helpers, **vhelpers}.items():
        assert sha(ROOT / name) == digest
    for name, digest in parent["analysis"]["code_sha256"].items():
        assert sha(REPO / name) == digest
    for name, digest in parent["input_sha256"].items():
        assert sha(RESULTS / name) == digest
    for name, digest in parent["output_sha256"].items():
        assert sha(BASE / name) == digest
    plot = json.loads((BASE / "plot-sources.json").read_text())
    assert sha(BASE / "plot-sources.json") == proof["plot_sources_sha256"]
    assert plot["script_sha256"] == sha(ROOT / "plot_reset_displacement.py")
    for name, digest in {**plot["input_sha256"], **plot["output_sha256"]}.items():
        assert sha(BASE / name) == digest
    assert len(list(BASE.iterdir())) == 11
    pins = {
        **parent["input_sha256"],
        **{
            "fusion-return-reset-displacement/" + p.name: sha(p) for p in BASE.iterdir()
        },
    }
    return parent, pins, helpers, vhelpers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Use a fresh output directory without -O")
    parent, pins, helpers, vhelpers = baseline()
    previous = read(BASE / "displacements.csv")
    rows = project(previous)
    summary = summaries(rows)
    assert len(rows) == 19 and len(summary) == 20
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
    assert audit["overlapping_endings"] == 185
    args.output.mkdir(parents=True)
    write(args.output / "projections.csv", rows, (*previous[0], *EXTRA))
    write(args.output / "projection-summary.csv", summary, SUMMARY)
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
        inherited_reset_displacement_proof_sha256=PROOF_SHA,
        inherited_geometry_proofs=parent["inherited_geometry_proofs"],
        audit=audit,
        output_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
