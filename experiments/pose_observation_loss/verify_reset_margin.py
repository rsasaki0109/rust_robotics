#!/usr/bin/env python3
"""Independently verify the frozen point-gap decomposition with Decimal."""

import argparse
from collections import Counter
from decimal import Decimal as D, localcontext
import json
from pathlib import Path

import verify_reset_projection as inherited

core, Interval = inherited.core, inherited.Interval
ROOT, REPO, RESULTS = core.ROOT, core.REPO, core.RESULTS
BASE = RESULTS / "fusion-return-reset-projection"
STUDY = RESULTS / "fusion-return-reset-margin"
BASE_MAIN = "f12dee7faea7a8a35d2262ab702be03db9c9434d"
BASE_SHA = "563b6549e56ae4ce085d861913ac956922b7617123088220bcd4335222ef7a23"
PROOF_SHA = "ded0de2bce633fd9ba6c041c9355a032ee55d018cfe75c0a76a2f8f2ebbd534d"
HELPERS = {
    **inherited.HELPERS,
    "verify_reset_projection.py": "49ee6cad7896deb8a41bb96c4494fbf67fafae2bbba967a0d8b139394ea5b853",
}
META = inherited.inherited.META
ORIGINS = ("initial_unsafe", "first_chord_by_exclusion", "ambiguous_zero")
STATUSES = ("complete", "unresolved_projection")
STATES = ("clear", "blocked", "unknown")
CATEGORIES = (
    "unresolved_projection",
    "already_clear",
    "blocked_to_clear_radial_clear",
    "blocked_to_clear_radial_blocked",
    "blocked_to_clear_radial_unknown",
    "other_point_states",
)
QUANTITIES = (
    "radial_gap",
    "rebuilt_mean_gap",
    "gap_identity_residual",
    "radial_gap_from_endpoint",
    "radial_gap_residual",
    "mean_gap_from_distance",
    "distance_gap_residual",
    "remainder_gap",
    "remainder_gap_residual",
)
COUNTS = (
    "physical_resets",
    "overlapping_endings",
    *("status_" + s for s in STATUSES),
    *(kind + "_" + s for kind in ("radial", "rebuilt") for s in STATES),
    *("category_" + c for c in CATEGORIES),
)
EXTREMA_QUANTITIES = ("radial_gap", "rebuilt_mean_gap", "remainder_gap")
read = inherited.read


def state(b):
    if b.b <= 0:
        return "blocked"
    return "clear" if b.a > 0 else "unknown"


def category(raw, mean, radial):
    assert all(s in STATES for s in (raw, mean, radial))
    if (raw, mean) == ("clear", "clear"):
        return "already_clear"
    if (raw, mean) == ("blocked", "clear"):
        return {
            "clear": "blocked_to_clear_radial_clear",
            "blocked": "blocked_to_clear_radial_blocked",
            "unknown": "blocked_to_clear_radial_unknown",
        }[radial]
    return "other_point_states"


def compute(row):
    with localcontext() as context:
        context.prec = 60
        assert row["projection_status"] in inherited.STATUSES
        raw = Interval.from_row(row, "witness_raw_current_gap")
        mean = Interval.from_row(row, "witness_mean_current_gap")
        rs, ms = row["witness_raw_current_state"], row["witness_mean_current_state"]
        assert (rs, ms) == (state(raw), state(mean))
        assert row["common_margin"] == row["decision_point_current_margin"]
        scalar = inherited.inherited.scalar
        margin = Interval(
            scalar(row["common_margin"]),
            scalar(row["decision_point_current_margin_lo"]),
            scalar(row["decision_point_current_margin_hi"]),
        )
        assert margin.a >= 0
        boxes = dict.fromkeys(QUANTITIES)
        status, radial_state, rebuilt_state, label = (
            "unresolved_projection",
            "unknown",
            "unknown",
            "unresolved_projection",
        )
        if row["projection_status"] == "complete":
            a = Interval.from_row(row, "radial_component")
            e = Interval.from_row(row, "distance_gain_beyond_radial")
            endpoint = Interval.from_row(row, "radial_endpoint")
            distance = Interval.from_row(row, "reconstructed_distance")
            partial = raw + a
            rebuilt = partial + e
            endpoint_gap = endpoint - margin
            distance_gap = distance - margin
            remaining = mean - partial
            boxes.update(
                radial_gap=partial,
                rebuilt_mean_gap=rebuilt,
                gap_identity_residual=mean - rebuilt,
                radial_gap_from_endpoint=endpoint_gap,
                radial_gap_residual=partial - endpoint_gap,
                mean_gap_from_distance=distance_gap,
                distance_gap_residual=mean - distance_gap,
                remainder_gap=remaining,
                remainder_gap_residual=remaining - e,
            )
            status, radial_state, rebuilt_state = (
                "complete",
                state(partial),
                state(rebuilt),
            )
            label = category(rs, ms, radial_state)
        for name in (
            "gap_identity_residual",
            "radial_gap_residual",
            "distance_gap_residual",
            "remainder_gap_residual",
        ):
            b = boxes[name]
            if b is not None:
                assert abs(b.c) <= D("1e-10") and b.a <= 0 <= b.b
        result = dict(
            margin_status=status,
            radial_gap_state=radial_state,
            rebuilt_mean_gap_state=rebuilt_state,
            point_gap_category=label,
        )
        for name in QUANTITIES:
            b = boxes[name]
            values = (b.c, b.a, b.b) if b is not None else (None, None, None)
            result.update(
                {
                    name + "_" + s: value
                    for s, value in zip(("center", "lo", "hi"), values)
                }
            )
        return result


def decompose(rows):
    ids = [r["edge_id"] for r in rows]
    assert len(set(ids)) == len(ids)
    result = []
    for row in rows:
        derived = row.copy()
        derived.update(compute(row))
        result.append(derived)
    return result


def summaries(rows):
    result = []
    for case in ["all_cases"] + sorted(set(r["case_id"] for r in rows)):
        for origin in ["all_origins", *ORIGINS]:
            records = [
                r
                for r in rows
                if (case == "all_cases" or r["case_id"] == case)
                and (origin == "all_origins" or r["decision_probe_origin"] == origin)
            ]
            out = dict(
                case_selector=case,
                origin_selector=origin,
                physical_resets=len(records),
                overlapping_endings=sum(int(r["multiplicity"]) for r in records),
            )
            for s in STATUSES:
                out["status_" + s] = len(
                    [r for r in records if r["margin_status"] == s]
                )
            for kind, field in (
                ("radial", "radial_gap_state"),
                ("rebuilt", "rebuilt_mean_gap_state"),
            ):
                for s in STATES:
                    out[kind + "_" + s] = len([r for r in records if r[field] == s])
            for c in CATEGORIES:
                out["category_" + c] = len(
                    [r for r in records if r["point_gap_category"] == c]
                )
            for q in EXTREMA_QUANTITIES:
                for side in ("lo", "hi"):
                    available = sorted(
                        r[q + "_" + side]
                        for r in records
                        if r[q + "_" + side] is not None
                    )
                    out["min_" + q + "_" + side] = available[0] if available else None
                    out["max_" + q + "_" + side] = available[-1] if available else None
            result.append(out)
    return result


def closure(rows):
    checks = 0
    for row in rows:
        for group in (
            ["status_" + s for s in STATUSES],
            ["category_" + c for c in CATEGORIES],
            ["radial_" + s for s in STATES],
            ["rebuilt_" + s for s in STATES],
        ):
            assert sum(row[f] for f in group) == row["physical_resets"]
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
        if "margin_status" not in row:
            continue
        boxes = {}
        for name in QUANTITIES:
            fields = [row[name + "_" + s] for s in ("center", "lo", "hi")]
            assert all(f != "" for f in fields) or all(f == "" for f in fields)
            boxes[name] = Interval.from_row(row, name) if fields[0] else None
        if row["margin_status"] == "unresolved_projection":
            assert row["projection_status"] != "complete" and all(
                b is None for b in boxes.values()
            )
            assert row["radial_gap_state"] == row["rebuilt_mean_gap_state"] == "unknown"
            assert row["point_gap_category"] == "unresolved_projection"
        else:
            assert row["margin_status"] == row["projection_status"] == "complete"
            assert all(b is not None for b in boxes.values())
            assert state(boxes["radial_gap"]) == row["radial_gap_state"]
            assert state(boxes["rebuilt_mean_gap"]) == row["rebuilt_mean_gap_state"]
            assert (
                category(
                    row["witness_raw_current_state"],
                    row["witness_mean_current_state"],
                    row["radial_gap_state"],
                )
                == row["point_gap_category"]
            )
            for name in (
                "gap_identity_residual",
                "radial_gap_residual",
                "distance_gap_residual",
                "remainder_gap_residual",
            ):
                b = boxes[name]
                assert abs(b.c) <= D("1e-10") and b.a <= 0 <= b.b


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
        == HELPERS["verify_reset_projection.py"]
    )
    assert len(list(BASE.iterdir())) == 11
    pins = {
        **parent["input_sha256"],
        **{
            "fusion-return-reset-projection/" + p.name: core.sha(p)
            for p in BASE.iterdir()
        },
    }
    assert source["input_sha256"] == pins
    for name, digest in pins.items():
        assert core.sha(RESULTS / name) == digest
    helpers = {
        **parent["analysis"]["helpers_sha256"],
        "analyze_reset_projection.py": parent["analysis"]["script_sha256"],
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
        ("script_sha256", ROOT / "analyze_reset_margin.py"),
        ("protocol_sha256", STUDY / "protocol.md"),
        ("freeze_sha256", STUDY / "freeze-pins.json"),
    ):
        assert source["analysis"][field] == core.sha(path)
    plots = json.loads((BASE / "plot-sources.json").read_text())
    assert core.sha(BASE / "plot-sources.json") == proof["plot_sources_sha256"]
    assert plots["script_sha256"] == core.sha(ROOT / "plot_reset_projection.py")
    for name, digest in {**plots["input_sha256"], **plots["output_sha256"]}.items():
        assert core.sha(BASE / name) == digest
    assert source["schema"] == dict(
        origins=list(ORIGINS),
        statuses=list(STATUSES),
        categories=list(CATEGORIES),
        quantities=list(QUANTITIES),
        retrospective=True,
        elemental_padding=1e-12,
        selection="same 19 reset/blocker pairs and 185 overlapping endings; no new trials or weights",
        interpretation="same-step point-gap identity categories; not motion or policy recovery",
    )
    assert source["inherited_reset_projection_proof_sha256"] == PROOF_SHA
    assert source["inherited_geometry_proofs"] == parent["inherited_geometry_proofs"]
    assert set(source["output_sha256"]) == {"margins.csv", "margin-summary.csv"}
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
    rows = decompose(read(BASE / "projections.csv"))
    summary = summaries(rows)
    assert (
        len(cases) == 972
        and len(anchors) == 3997
        and len(rows) == 19
        and len(summary) == 20
    )
    compare(args.results / "margins.csv", rows, ("edge_id",))
    compare(
        args.results / "margin-summary.csv",
        summary,
        ("case_selector", "origin_selector"),
    )
    audit = dict(
        physical_resets=len(rows),
        overlapping_endings=sum(int(r["multiplicity"]) for r in rows),
        origins=dict(Counter(r["decision_probe_origin"] for r in rows)),
        margin_statuses=dict(Counter(r["margin_status"] for r in rows)),
        radial_gap_states=dict(Counter(r["radial_gap_state"] for r in rows)),
        rebuilt_mean_gap_states=dict(
            Counter(r["rebuilt_mean_gap_state"] for r in rows)
        ),
        point_gap_categories=dict(Counter(r["point_gap_category"] for r in rows)),
        summary_rows=len(summary),
        additive_closure_comparisons=closure(summary),
        fresh_margin_pairs=len(rows),
        fresh_raw_reads=0,
        fresh_native_runs=0,
    )
    assert audit == source["audit"] and audit["overlapping_endings"] == 185
    plot = json.loads((args.figures / "plot-sources.json").read_text())
    assert plot["script_sha256"] == core.sha(ROOT / "plot_reset_margin.py")
    assert plot["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in ("sources.json", "margins.csv", "margin-summary.csv")
    }
    assert set(plot["output_sha256"]) == {
        "reset-margin.svg",
        "plot-margins.csv",
        "plot-values.csv",
    }
    for name, digest in plot["output_sha256"].items():
        assert core.sha(args.figures / name) == digest
    compare(args.figures / "plot-margins.csv", rows, ("edge_id",))
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
        inherited_reset_projection_proof_sha256=PROOF_SHA,
        inherited_geometry_proofs=source["inherited_geometry_proofs"],
        method="independent 60-digit Decimal point-gap identities, strict point states/categories, interval extrema/count closures and original census joins; no new raw/native dataset execution",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
