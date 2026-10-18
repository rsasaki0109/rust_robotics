#!/usr/bin/env python3
"""Decompose the original point gaps at the same 19 reset/blocker pairs."""

import argparse
from collections import Counter
import json
from pathlib import Path

import analyze_reset_projection as inherited

ROOT, REPO, RESULTS = inherited.ROOT, inherited.REPO, inherited.RESULTS
BASE = RESULTS / "fusion-return-reset-projection"
STUDY = RESULTS / "fusion-return-reset-margin"
BASE_MAIN = "f12dee7faea7a8a35d2262ab702be03db9c9434d"
BASE_SHA = "563b6549e56ae4ce085d861913ac956922b7617123088220bcd4335222ef7a23"
PROOF_SHA = "ded0de2bce633fd9ba6c041c9355a032ee55d018cfe75c0a76a2f8f2ebbd534d"
ORIGINS, P = inherited.ORIGINS, inherited.P
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
CATEGORIES = (
    "unresolved_projection",
    "already_clear",
    "blocked_to_clear_radial_clear",
    "blocked_to_clear_radial_blocked",
    "blocked_to_clear_radial_unknown",
    "other_point_states",
)
STATUSES = ("complete", "unresolved_projection")
STATES = ("clear", "blocked", "unknown")
EXTRA = tuple(q + "_" + s for q in QUANTITIES for s in ("center", "lo", "hi")) + (
    "margin_status",
    "radial_gap_state",
    "rebuilt_mean_gap_state",
    "point_gap_category",
)
COUNTS = (
    "physical_resets",
    "overlapping_endings",
    *("status_" + s for s in STATUSES),
    *(kind + "_" + s for kind in ("radial", "rebuilt") for s in STATES),
    *("category_" + c for c in CATEGORIES),
)
EXTREMA_QUANTITIES = ("radial_gap", "rebuilt_mean_gap", "remainder_gap")
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
    categories=CATEGORIES,
    quantities=QUANTITIES,
    retrospective=True,
    elemental_padding=P,
    selection="same 19 reset/blocker pairs and 185 overlapping endings; no new trials or weights",
    interpretation="same-step point-gap identity categories; not motion or policy recovery",
)
read, write, sha = inherited.read, inherited.write, inherited.sha
box, load, add, sub = inherited.box, inherited.load, inherited.add, inherited.sub


def state(gap):
    return "clear" if gap[1] > 0 else "blocked" if gap[2] <= 0 else "unknown"


def category(raw, mean, radial):
    assert raw in STATES and mean in STATES and radial in STATES
    if raw == mean == "clear":
        return "already_clear"
    if raw == "blocked" and mean == "clear":
        return "blocked_to_clear_radial_" + radial
    return "other_point_states"


def compute(row):
    assert row["projection_status"] in inherited.STATUSES
    raw, mean = (load(row, "witness_" + k + "_current_gap") for k in ("raw", "mean"))
    raw_state, mean_state = (
        row["witness_" + k + "_current_state"] for k in ("raw", "mean")
    )
    assert raw_state == state(raw) and mean_state == state(mean)
    assert row["common_margin"] == row["decision_point_current_margin"]
    margin = box(
        row["common_margin"],
        row["decision_point_current_margin_lo"],
        row["decision_point_current_margin_hi"],
    )
    assert margin[1] >= 0
    values = dict.fromkeys(QUANTITIES)
    result = dict(
        margin_status="unresolved_projection",
        radial_gap_state="unknown",
        rebuilt_mean_gap_state="unknown",
        point_gap_category="unresolved_projection",
    )
    if row["projection_status"] == "complete":
        radial, extra, endpoint, distance = (
            load(row, q)
            for q in (
                "radial_component",
                "distance_gain_beyond_radial",
                "radial_endpoint",
                "reconstructed_distance",
            )
        )
        radial_gap = add(raw, radial)
        rebuilt = add(radial_gap, extra)
        from_endpoint, from_distance = sub(endpoint, margin), sub(distance, margin)
        remainder = sub(mean, radial_gap)
        values.update(
            radial_gap=radial_gap,
            rebuilt_mean_gap=rebuilt,
            gap_identity_residual=sub(mean, rebuilt),
            radial_gap_from_endpoint=from_endpoint,
            radial_gap_residual=sub(radial_gap, from_endpoint),
            mean_gap_from_distance=from_distance,
            distance_gap_residual=sub(mean, from_distance),
            remainder_gap=remainder,
            remainder_gap_residual=sub(remainder, extra),
        )
        result.update(
            margin_status="complete",
            radial_gap_state=state(radial_gap),
            rebuilt_mean_gap_state=state(rebuilt),
            point_gap_category=category(raw_state, mean_state, state(radial_gap)),
        )
    for name in (
        "gap_identity_residual",
        "radial_gap_residual",
        "distance_gap_residual",
        "remainder_gap_residual",
    ):
        if values[name] is not None:
            c, lo, hi = values[name]
            assert abs(c) <= 1e-10 and lo <= 0 <= hi
    for q in QUANTITIES:
        result.update(
            {
                q + "_" + s: value
                for s, value in zip(
                    ("center", "lo", "hi"), values[q] or (None, None, None)
                )
            }
        )
    return result


def decompose(rows):
    assert len({r["edge_id"] for r in rows}) == len(rows)
    return [dict(row, **compute(row)) for row in rows]


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
                    "status_" + s: sum(r["margin_status"] == s for r in selected)
                    for s in STATUSES
                }
            )
            for key, prefix in (
                ("radial_gap_state", "radial_"),
                ("rebuilt_mean_gap_state", "rebuilt_"),
            ):
                counts.update(
                    {prefix + s: sum(r[key] == s for r in selected) for s in STATES}
                )
            counts.update(
                {
                    "category_" + c: sum(r["point_gap_category"] == c for r in selected)
                    for c in CATEGORIES
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
            tuple("status_" + s for s in STATUSES),
            tuple("category_" + s for s in CATEGORIES),
            tuple("radial_" + s for s in STATES),
            tuple("rebuilt_" + s for s in STATES),
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
        == sha(ROOT / "verify_reset_projection.py")
    )
    for field, path in (
        ("script_sha256", ROOT / "analyze_reset_projection.py"),
        ("protocol_sha256", BASE / "protocol.md"),
        ("freeze_sha256", BASE / "freeze-pins.json"),
    ):
        assert parent["analysis"][field] == sha(path)
    helpers = {
        **parent["analysis"]["helpers_sha256"],
        "analyze_reset_projection.py": parent["analysis"]["script_sha256"],
    }
    vhelpers = {
        **parent["analysis"]["verifier_helpers_sha256"],
        "verify_reset_projection.py": freeze["verifier_helper_sha256"],
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
    assert plot["script_sha256"] == sha(ROOT / "plot_reset_projection.py")
    for name, digest in {**plot["input_sha256"], **plot["output_sha256"]}.items():
        assert sha(BASE / name) == digest
    assert len(list(BASE.iterdir())) == 11
    pins = {
        **parent["input_sha256"],
        **{"fusion-return-reset-projection/" + p.name: sha(p) for p in BASE.iterdir()},
    }
    return parent, pins, helpers, vhelpers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Use a fresh output directory without -O")
    parent, pins, helpers, vhelpers = baseline()
    previous = read(BASE / "projections.csv")
    rows = decompose(previous)
    summary = summaries(rows)
    assert len(rows) == 19 and len(summary) == 20
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
    assert audit["overlapping_endings"] == 185
    args.output.mkdir(parents=True)
    write(args.output / "margins.csv", rows, (*previous[0], *EXTRA))
    write(args.output / "margin-summary.csv", summary, SUMMARY)
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
        inherited_reset_projection_proof_sha256=PROOF_SHA,
        inherited_geometry_proofs=parent["inherited_geometry_proofs"],
        audit=audit,
        output_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
