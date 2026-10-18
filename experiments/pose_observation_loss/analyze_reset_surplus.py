#!/usr/bin/env python3
"""Measure the frozen radial contribution relative to the original point deficit."""

import argparse
from collections import Counter
import json
from pathlib import Path

import analyze_reset_margin as inherited
from analyze_reset_projection import divide

ROOT, REPO, RESULTS = inherited.ROOT, inherited.REPO, inherited.RESULTS
BASE = RESULTS / "fusion-return-reset-margin"
STUDY = RESULTS / "fusion-return-reset-surplus"
BASE_MAIN = "e96f9067522d81ad1998fd7e2c7b07433d4580d2"
BASE_SHA = "98754f83cc35426f46abd5af33fa37f0f492ce9d7a79007058a9d1fe85c8560b"
PROOF_SHA = "b65366c13bb48c3d4843233a45ebb593b15ad0562da5c3d013ec1f5233e8c898"
ORIGINS, P = inherited.ORIGINS, inherited.P
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
EXTRA = tuple(q + "_" + s for q in QUANTITIES for s in ("center", "lo", "hi")) + (
    "surplus_status",
    "required_fraction_relation",
    "surplus_fraction_state",
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
    relations=RELATIONS,
    states=STATES,
    quantities=QUANTITIES,
    retrospective=True,
    elemental_padding=P,
    eligibility="preceding blocked_to_clear_radial_clear only; all 19 rows retained",
    interpretation="same-step algebraic fractions; not a controller parameter, trajectory or recovery probability",
)
read, write, sha = inherited.read, inherited.write, inherited.sha
box, load, add, sub = inherited.box, inherited.load, inherited.add, inherited.sub


def relation(value):
    return "below_one" if value[2] < 1 else "above_one" if value[1] > 1 else "unknown"


def sign(value):
    return "positive" if value[1] > 0 else "negative" if value[2] < 0 else "unknown"


def compute(row):
    assert (
        row["margin_status"] in inherited.STATUSES
        and row["point_gap_category"] in inherited.CATEGORIES
    )
    values = dict.fromkeys(QUANTITIES)
    out = dict(
        surplus_status="unresolved_margin",
        required_fraction_relation="unknown",
        surplus_fraction_state="unknown",
    )
    if row["margin_status"] == "complete":
        raw, mean, gap, radial = (
            load(row, name)
            for name in (
                "witness_raw_current_gap",
                "witness_mean_current_gap",
                "radial_gap",
                "radial_component",
            )
        )
        rs, ms, gs = (
            row[f]
            for f in (
                "witness_raw_current_state",
                "witness_mean_current_state",
                "radial_gap_state",
            )
        )
        assert (rs, ms, gs) == tuple(inherited.state(b) for b in (raw, mean, gap))
        assert row["point_gap_category"] == inherited.category(rs, ms, gs)
        if row["point_gap_category"] != "blocked_to_clear_radial_clear":
            out.update(
                surplus_status="not_applicable_category",
                required_fraction_relation="not_applicable",
                surplus_fraction_state="not_applicable",
            )
        elif radial[1] <= 0:
            out["surplus_status"] = "unresolved_radial_denominator"
        else:
            deficit = sub(box(0, 0, 0), raw)
            surplus = sub(radial, deficit)
            needed, remaining, from_gap = (
                divide(deficit, radial),
                divide(surplus, radial),
                divide(gap, radial),
            )
            values.update(
                raw_deficit=deficit,
                radial_surplus=surplus,
                required_fraction=needed,
                surplus_fraction=remaining,
                gap_fraction=from_gap,
                fraction_partition_residual=sub(add(needed, remaining), box(1, 1, 1)),
                fraction_identity_residual=sub(remaining, from_gap),
                surplus_identity_residual=sub(surplus, gap),
            )
            out.update(
                surplus_status="complete",
                required_fraction_relation=relation(needed),
                surplus_fraction_state=sign(remaining),
            )
    else:
        assert row["point_gap_category"] == "unresolved_projection"
    for name in (
        "fraction_partition_residual",
        "fraction_identity_residual",
        "surplus_identity_residual",
    ):
        if values[name] is not None:
            c, lo, hi = values[name]
            assert abs(c) <= 1e-10 and lo <= 0 <= hi
    for q in QUANTITIES:
        out.update(
            {
                q + "_" + s: value
                for s, value in zip(
                    ("center", "lo", "hi"), values[q] or (None, None, None)
                )
            }
        )
    return out


def calculate(rows):
    assert len({r["edge_id"] for r in rows}) == len(rows)
    return [dict(row, **compute(row)) for row in rows]


def summaries(rows):
    result = []
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
                computed_overlapping_endings=sum(
                    int(r["multiplicity"])
                    for r in selected
                    if r["surplus_status"] == "complete"
                ),
            )
            for field, prefix, choices in (
                ("surplus_status", "status_", STATUSES),
                ("required_fraction_relation", "required_", RELATIONS),
                ("surplus_fraction_state", "surplus_", STATES),
            ):
                counts.update(
                    {prefix + c: sum(r[field] == c for r in selected) for c in choices}
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
            result.append(
                dict(case_selector=case, origin_selector=origin, **counts, **extrema)
            )
    return result


def closure(rows):
    by = {(r["case_selector"], r["origin_selector"]): r for r in rows}
    checks = 0
    for (c, o), row in by.items():
        for prefix, choices in (
            ("status_", STATUSES),
            ("required_", RELATIONS),
            ("surplus_", STATES),
        ):
            assert sum(row[prefix + s] for s in choices) == row["physical_resets"]
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
        == sha(ROOT / "verify_reset_margin.py")
    )
    for field, path in (
        ("script_sha256", ROOT / "analyze_reset_margin.py"),
        ("protocol_sha256", BASE / "protocol.md"),
        ("freeze_sha256", BASE / "freeze-pins.json"),
    ):
        assert parent["analysis"][field] == sha(path)
    helpers = {
        **parent["analysis"]["helpers_sha256"],
        "analyze_reset_margin.py": parent["analysis"]["script_sha256"],
    }
    vhelpers = {
        **parent["analysis"]["verifier_helpers_sha256"],
        "verify_reset_margin.py": freeze["verifier_helper_sha256"],
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
    assert plot["script_sha256"] == sha(ROOT / "plot_reset_margin.py")
    for name, digest in {**plot["input_sha256"], **plot["output_sha256"]}.items():
        assert sha(BASE / name) == digest
    assert len(list(BASE.iterdir())) == 11
    pins = {
        **parent["input_sha256"],
        **{"fusion-return-reset-margin/" + p.name: sha(p) for p in BASE.iterdir()},
    }
    return parent, pins, helpers, vhelpers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Use a fresh output directory without -O")
    parent, pins, helpers, vhelpers = baseline()
    previous = read(BASE / "margins.csv")
    rows = calculate(previous)
    summary = summaries(rows)
    assert len(rows) == 19 and len(summary) == 20
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
    assert audit["overlapping_endings"] == 185
    args.output.mkdir(parents=True)
    write(args.output / "surplus.csv", rows, (*previous[0], *EXTRA))
    write(args.output / "surplus-summary.csv", summary, SUMMARY)
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
        inherited_reset_margin_proof_sha256=PROOF_SHA,
        inherited_geometry_proofs=parent["inherited_geometry_proofs"],
        audit=audit,
        output_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
