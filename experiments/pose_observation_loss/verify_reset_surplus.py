#!/usr/bin/env python3
"""Independently verify guarded deficit/radial fractions with Decimal."""

import argparse
from collections import Counter
from decimal import Decimal as D, localcontext
import json
from pathlib import Path

import verify_reset_margin as inherited
from verify_reset_projection import divide

core, Interval = inherited.core, inherited.Interval
ROOT, REPO, RESULTS = core.ROOT, core.REPO, core.RESULTS
BASE = RESULTS / "fusion-return-reset-margin"
STUDY = RESULTS / "fusion-return-reset-surplus"
BASE_MAIN = "e96f9067522d81ad1998fd7e2c7b07433d4580d2"
BASE_SHA = "98754f83cc35426f46abd5af33fa37f0f492ce9d7a79007058a9d1fe85c8560b"
PROOF_SHA = "b65366c13bb48c3d4843233a45ebb593b15ad0562da5c3d013ec1f5233e8c898"
HELPERS = {
    **inherited.HELPERS,
    "verify_reset_margin.py": "65343ae1f87d9f22318bd846a6eba01356e1c8dc7ecad5f4cd82d5630c56eaa3",
}
META = inherited.META
ORIGINS = ("initial_unsafe", "first_chord_by_exclusion", "ambiguous_zero")
STATUSES = (
    "complete",
    "not_applicable_category",
    "unresolved_margin",
    "unresolved_radial_denominator",
)
RELATIONS = ("below_one", "above_one", "unknown", "not_applicable")
STATES = ("positive", "negative", "unknown", "not_applicable")
QUANTITIES = (
    "raw_deficit",
    "radial_surplus",
    "required_fraction",
    "surplus_fraction",
    "gap_fraction",
    "fraction_partition_residual",
    "fraction_identity_residual",
    "surplus_identity_residual",
)
COUNTS = (
    "physical_resets",
    "overlapping_endings",
    "computed_overlapping_endings",
    *("status_" + s for s in STATUSES),
    *("required_" + s for s in RELATIONS),
    *("surplus_" + s for s in STATES),
)
EXTREMA_QUANTITIES = ("required_fraction", "surplus_fraction", "radial_surplus")
read = inherited.read


def relation(b):
    if b.a > 1:
        return "above_one"
    return "below_one" if b.b < 1 else "unknown"


def sign(b):
    if b.b < 0:
        return "negative"
    return "positive" if b.a > 0 else "unknown"


def compute(row):
    with localcontext() as context:
        context.prec = 60
        assert (
            row["margin_status"] in inherited.STATUSES
            and row["point_gap_category"] in inherited.CATEGORIES
        )
        boxes = dict.fromkeys(QUANTITIES)
        status, needed_relation, surplus_state = (
            "unresolved_margin",
            "unknown",
            "unknown",
        )
        if row["margin_status"] == "complete":
            raw = Interval.from_row(row, "witness_raw_current_gap")
            mean = Interval.from_row(row, "witness_mean_current_gap")
            gap = Interval.from_row(row, "radial_gap")
            radial = Interval.from_row(row, "radial_component")
            raw_state, mean_state, gap_state = (
                row[f]
                for f in (
                    "witness_raw_current_state",
                    "witness_mean_current_state",
                    "radial_gap_state",
                )
            )
            assert (raw_state, mean_state, gap_state) == (
                inherited.state(raw),
                inherited.state(mean),
                inherited.state(gap),
            )
            assert (
                inherited.category(raw_state, mean_state, gap_state)
                == row["point_gap_category"]
            )
            if row["point_gap_category"] != "blocked_to_clear_radial_clear":
                status, needed_relation, surplus_state = (
                    "not_applicable_category",
                    "not_applicable",
                    "not_applicable",
                )
            elif radial.a <= 0:
                status = "unresolved_radial_denominator"
            else:
                deficit = Interval.exact(0) - raw
                surplus = radial - deficit
                needed, remaining = divide(deficit, radial), divide(surplus, radial)
                gap_ratio = divide(gap, radial)
                boxes.update(
                    raw_deficit=deficit,
                    radial_surplus=surplus,
                    required_fraction=needed,
                    surplus_fraction=remaining,
                    gap_fraction=gap_ratio,
                    fraction_partition_residual=needed + remaining - Interval.exact(1),
                    fraction_identity_residual=remaining - gap_ratio,
                    surplus_identity_residual=surplus - gap,
                )
                status, needed_relation, surplus_state = (
                    "complete",
                    relation(needed),
                    sign(remaining),
                )
        else:
            assert row["point_gap_category"] == "unresolved_projection"
        for q in (
            "fraction_partition_residual",
            "fraction_identity_residual",
            "surplus_identity_residual",
        ):
            b = boxes[q]
            if b is not None:
                assert abs(b.c) <= D("1e-10") and b.a <= 0 <= b.b
        out = dict(
            surplus_status=status,
            required_fraction_relation=needed_relation,
            surplus_fraction_state=surplus_state,
        )
        for q in QUANTITIES:
            b = boxes[q]
            out.update(
                {
                    q + "_" + s: value
                    for s, value in zip(
                        ("center", "lo", "hi"),
                        (b.c, b.a, b.b) if b is not None else (None, None, None),
                    )
                }
            )
        return out


def calculate(rows):
    identities = [r["edge_id"] for r in rows]
    assert len(identities) == len(set(identities))
    out = []
    for row in rows:
        result = row.copy()
        result.update(compute(row))
        out.append(result)
    return out


def summaries(rows):
    out = []
    for case in ["all_cases"] + sorted(set(r["case_id"] for r in rows)):
        for origin in ["all_origins", *ORIGINS]:
            records = [
                r
                for r in rows
                if (case == "all_cases" or r["case_id"] == case)
                and (origin == "all_origins" or r["decision_probe_origin"] == origin)
            ]
            result = dict(
                case_selector=case,
                origin_selector=origin,
                physical_resets=len(records),
                overlapping_endings=sum(int(r["multiplicity"]) for r in records),
                computed_overlapping_endings=sum(
                    int(r["multiplicity"])
                    for r in records
                    if r["surplus_status"] == "complete"
                ),
            )
            for f, prefix, choices in (
                ("surplus_status", "status_", STATUSES),
                ("required_fraction_relation", "required_", RELATIONS),
                ("surplus_fraction_state", "surplus_", STATES),
            ):
                for choice in choices:
                    result[prefix + choice] = len(
                        [r for r in records if r[f] == choice]
                    )
            for q in EXTREMA_QUANTITIES:
                for side in ("lo", "hi"):
                    values = sorted(
                        r[q + "_" + side]
                        for r in records
                        if r[q + "_" + side] is not None
                    )
                    result["min_" + q + "_" + side] = values[0] if values else None
                    result["max_" + q + "_" + side] = values[-1] if values else None
            out.append(result)
    return out


def closure(rows):
    checks = 0
    for row in rows:
        for prefix, choices in (
            ("status_", STATUSES),
            ("required_", RELATIONS),
            ("surplus_", STATES),
        ):
            assert sum(row[prefix + s] for s in choices) == row["physical_resets"]
        case, origin = row["case_selector"], row["origin_selector"]
        groups = []
        if case == "all_cases":
            groups.append(
                [
                    r
                    for r in rows
                    if r["case_selector"] != case and r["origin_selector"] == origin
                ]
            )
        if origin == "all_origins":
            groups.append(
                [
                    r
                    for r in rows
                    if r["case_selector"] == case and r["origin_selector"] in ORIGINS
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
        if "surplus_status" not in row:
            continue
        boxes = {}
        for q in QUANTITIES:
            fields = [row[q + "_" + s] for s in ("center", "lo", "hi")]
            assert all(f == "" for f in fields) or all(f != "" for f in fields)
            boxes[q] = Interval.from_row(row, q) if fields[0] else None
        status = row["surplus_status"]
        if status == "complete":
            assert (
                row["margin_status"] == "complete"
                and row["point_gap_category"] == "blocked_to_clear_radial_clear"
            )
            assert Interval.from_row(row, "radial_component").a > 0 and all(
                b is not None for b in boxes.values()
            )
            assert (
                relation(boxes["required_fraction"])
                == row["required_fraction_relation"]
            )
            assert sign(boxes["surplus_fraction"]) == row["surplus_fraction_state"]
            for q in (
                "fraction_partition_residual",
                "fraction_identity_residual",
                "surplus_identity_residual",
            ):
                b = boxes[q]
                assert abs(b.c) <= D("1e-10") and b.a <= 0 <= b.b
        else:
            assert status in STATUSES and all(b is None for b in boxes.values())
            if status == "not_applicable_category":
                assert (
                    row["margin_status"] == "complete"
                    and row["point_gap_category"] != "blocked_to_clear_radial_clear"
                )
                assert (
                    row["required_fraction_relation"]
                    == row["surplus_fraction_state"]
                    == "not_applicable"
                )
            else:
                assert (
                    row["required_fraction_relation"]
                    == row["surplus_fraction_state"]
                    == "unknown"
                )
                if status == "unresolved_margin":
                    assert row["margin_status"] != "complete"
                else:
                    assert (
                        row["margin_status"] == "complete"
                        and row["point_gap_category"] == "blocked_to_clear_radial_clear"
                    )
                    assert Interval.from_row(row, "radial_component").a <= 0


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
        == HELPERS["verify_reset_margin.py"]
    )
    assert len(list(BASE.iterdir())) == 11
    pins = {
        **parent["input_sha256"],
        **{"fusion-return-reset-margin/" + p.name: core.sha(p) for p in BASE.iterdir()},
    }
    assert source["input_sha256"] == pins
    for name, digest in pins.items():
        assert core.sha(RESULTS / name) == digest
    helpers = {
        **parent["analysis"]["helpers_sha256"],
        "analyze_reset_margin.py": parent["analysis"]["script_sha256"],
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
        ("script_sha256", ROOT / "analyze_reset_surplus.py"),
        ("protocol_sha256", STUDY / "protocol.md"),
        ("freeze_sha256", STUDY / "freeze-pins.json"),
    ):
        assert source["analysis"][field] == core.sha(path)
    plots = json.loads((BASE / "plot-sources.json").read_text())
    assert core.sha(BASE / "plot-sources.json") == proof["plot_sources_sha256"]
    assert plots["script_sha256"] == core.sha(ROOT / "plot_reset_margin.py")
    for name, digest in {**plots["input_sha256"], **plots["output_sha256"]}.items():
        assert core.sha(BASE / name) == digest
    assert source["schema"] == dict(
        origins=list(ORIGINS),
        statuses=list(STATUSES),
        relations=list(RELATIONS),
        states=list(STATES),
        quantities=list(QUANTITIES),
        retrospective=True,
        elemental_padding=1e-12,
        eligibility="preceding blocked_to_clear_radial_clear only; all 19 rows retained",
        interpretation="same-step algebraic fractions; not a controller parameter, trajectory or recovery probability",
    )
    assert source["inherited_reset_margin_proof_sha256"] == PROOF_SHA
    assert source["inherited_geometry_proofs"] == parent["inherited_geometry_proofs"]
    assert set(source["output_sha256"]) == {"surplus.csv", "surplus-summary.csv"}
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
    for row in anchors:
        for f in (
            *META,
            "received",
            "parent_rp_records",
            "rp_records",
            "rp_different_preview_records",
        ):
            core.check(row[f], canonical[row["event_id"]][f])
    original_checks = core.comparisons
    rows = calculate(read(BASE / "margins.csv"))
    summary = summaries(rows)
    assert (
        len(cases) == 972
        and len(anchors) == 3997
        and len(rows) == 19
        and len(summary) == 20
    )
    compare(args.results / "surplus.csv", rows, ("edge_id",))
    compare(
        args.results / "surplus-summary.csv",
        summary,
        ("case_selector", "origin_selector"),
    )
    audit = dict(
        physical_resets=len(rows),
        overlapping_endings=sum(int(r["multiplicity"]) for r in rows),
        inspected_pairs=len(rows),
        computed_overlapping_endings=sum(
            int(r["multiplicity"]) for r in rows if r["surplus_status"] == "complete"
        ),
        origins=dict(Counter(r["decision_probe_origin"] for r in rows)),
        surplus_statuses=dict(Counter(r["surplus_status"] for r in rows)),
        required_fraction_relations=dict(
            Counter(r["required_fraction_relation"] for r in rows)
        ),
        surplus_fraction_states=dict(
            Counter(r["surplus_fraction_state"] for r in rows)
        ),
        summary_rows=len(summary),
        additive_closure_comparisons=closure(summary),
        fresh_ratio_pairs=sum(r["surplus_status"] == "complete" for r in rows),
        fresh_raw_reads=0,
        fresh_native_runs=0,
    )
    assert audit == source["audit"] and audit["overlapping_endings"] == 185
    plot = json.loads((args.figures / "plot-sources.json").read_text())
    assert plot["script_sha256"] == core.sha(ROOT / "plot_reset_surplus.py")
    assert plot["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in ("sources.json", "surplus.csv", "surplus-summary.csv")
    }
    assert set(plot["output_sha256"]) == {
        "reset-surplus.svg",
        "plot-surplus.csv",
        "plot-values.csv",
    }
    for name, digest in plot["output_sha256"].items():
        assert core.sha(args.figures / name) == digest
    compare(args.figures / "plot-surplus.csv", rows, ("edge_id",))
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
        inherited_reset_margin_proof_sha256=PROOF_SHA,
        inherited_geometry_proofs=source["inherited_geometry_proofs"],
        method="independent 60-digit Decimal eligibility/guarded fractions, strict relations/signs, identity intervals, extrema/count closures and original census joins; no new raw/native dataset execution",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
