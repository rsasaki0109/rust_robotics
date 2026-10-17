#!/usr/bin/env python3
"""Delete each selected case/family from fixed same-event ranking populations."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import product
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_case_window_ranks import compare
import analyze_event_score_ties as prior
from summarize_stopping_preview import save_rows

ROOT, RESULTS = prior.ROOT, prior.RESULTS
previous, parent, window = prior.previous, prior.parent, prior.window
RANK, SCOPES = prior.RANK, prior.SCOPES
BASE = RESULTS / "event-score-ties"
BASE_SHA = "b1b180955af97033109e3147f83864291083856b68c426ef4e7ae5d1b0f8f1b2"
PROTOCOL = RESULTS / "event-unit-influence/protocol.md"
SELECT = ["partition", "noise", "variant", "stratum", "population", "planner_selection"]
KINDS = ["case", "family"]
METRICS = [
    "auc",
    "native_auc",
    "tick_unequal_concordance",
    "native_unequal_concordance",
]
MASS = [
    *prior.FIELDS,
    "native_concordant_pair_mass",
    "native_tied_pair_mass",
    "native_discordant_pair_mass",
    "native_numerator",
    "tick_tied_native_concordant_mass",
    "tick_tied_native_discordant_mass",
]
COUNTS = [
    "pair_count",
    "events_with_pairs",
    "cases_with_pairs",
    "tick_unequal_pair_count",
    "native_unequal_pair_count",
    "tick_unequal_events",
    "tick_unequal_cases",
    "native_unequal_events",
    "native_unequal_cases",
    *(c + "_pair_count" for c in prior.CLASSES),
]
ADDITIVE = [*MASS, *COUNTS]


def sum_parts(parts):
    return {
        f: sum((p[f] for p in parts), Fraction() if f in MASS else 0) for f in ADDITIVE
    }


def subtract(total, removed):
    result = {f: total[f] - removed[f] for f in ADDITIVE}
    assert all(v >= 0 for v in result.values()), "deletion must be a subset"
    return result


def decorate(values, full_den, event_den):
    r = dict(values)
    con, tied, dis, den = (
        r[f]
        for f in [
            "concordant_pair_mass",
            "tied_pair_mass",
            "discordant_pair_mass",
            "denominator",
        ]
    )
    nc, nt, nd = (
        r["native_" + f + "_pair_mass"] for f in ["concordant", "tied", "discordant"]
    )
    assert con + tied + dis == nc + nt + nd == den
    assert r["numerator"] == con + tied / 2
    assert r["native_numerator"] == nc + nt / 2
    auc, native = (
        (r["numerator"] / den, r["native_numerator"] / den) if den else (None, None)
    )
    r.update(
        auc=auc,
        native_auc=native,
        native_auc_delta=native - auc if den else None,
        full_pair_share=den / full_den if full_den else None,
        event_pair_share=den / event_den if event_den else None,
        numerator_full_contribution=r["numerator"] / full_den if full_den else None,
        tick_tied_share=tied / den if den else None,
        native_tied_share=nt / den if den else None,
        resolved_tick_tie_share=(
            (
                r["tick_tied_native_concordant_mass"]
                + r["tick_tied_native_discordant_mass"]
            )
            / den
            if den
            else None
        ),
        tick_unequal_concordance=con / (con + dis) if con + dis else None,
        native_unequal_concordance=nc / (nc + nd) if nc + nd else None,
    )
    return r


def unit_audits(rows, lookup, metadata):
    result, by_case = [], defaultdict(list)
    for r in rows:
        by_case[lookup[previous.identity(r)]["case_id"]].append(r)
    for kind in KINDS:
        units = defaultdict(list)
        for cid, rr in by_case.items():
            fid = lookup[previous.identity(rr[0])]["family_id"]
            units[cid if kind == "case" else fid].extend(rr)
        for uid, rr in sorted(units.items()):
            cases = sorted({lookup[previous.identity(r)]["case_id"] for r in rr})
            families = sorted({lookup[previous.identity(r)]["family_id"] for r in rr})
            result.append(
                dict(
                    **metadata,
                    unit_kind=kind,
                    unit_id=uid,
                    case_ids="+".join(cases),
                    family_ids="+".join(families),
                    records=len(rr),
                    cases=len(cases),
                    events=len({(previous.identity(r), r["_event_id"]) for r in rr}),
                    case_mass=sum(
                        (lookup[previous.identity(r)]["case_row_weight"] for r in rr),
                        Fraction(),
                    )
                )
            )
    return result, by_case


def deletion_row(baseline, removed, remaining, unit, selected):
    den, tick_den = (
        baseline["denominator"],
        baseline["concordant_pair_mass"] + baseline["discordant_pair_mass"],
    )
    native_den = (
        baseline["native_concordant_pair_mass"]
        + baseline["native_discordant_pair_mass"]
    )
    rtick = removed["concordant_pair_mass"] + removed["discordant_pair_mass"]
    rnative = (
        removed["native_concordant_pair_mass"] + removed["native_discordant_pair_mass"]
    )
    r = dict(
        unit_kind=unit["unit_kind"],
        unit_id=unit["unit_id"],
        removed_records=unit["records"],
        removed_cases=unit["cases"],
        removed_events=unit["events"],
        removed_case_mass=unit["case_mass"],
        removed_pair_count=removed["pair_count"],
        removed_pair_share=removed["denominator"] / den if den else None,
        removed_tick_unequal_pair_count=removed["tick_unequal_pair_count"],
        removed_native_unequal_pair_count=removed["native_unequal_pair_count"],
        removed_tick_unequal_share=rtick / tick_den if tick_den else None,
        removed_native_unequal_share=rnative / native_den if native_den else None,
        records=selected["records"] - unit["records"],
        cases=selected["cases"] - unit["cases"],
        events=selected["events"] - unit["events"],
    )
    for prefix in ["", "native_"]:
        for cls in ["concordant", "tied", "discordant"]:
            r["removed_" + prefix + cls + "_mass"] = removed[
                prefix + cls + "_pair_mass"
            ]
    for f in ["numerator", "denominator", "native_numerator", *COUNTS[:9]]:
        r[f] = remaining[f]
    for metric in METRICS:
        after, before = remaining[metric], baseline[metric]
        r[metric] = after
        r["delta_" + metric] = (
            after - before if after is not None and before is not None else None
        )
    return r


def summarize(baseline, deletions, selected):
    r = dict(
        records=selected["records"],
        cases=selected["cases"],
        events=selected["events"],
        baseline_pair_mass=baseline["denominator"],
        baseline_pair_count=baseline["pair_count"],
        baseline_tick_unequal_mass=baseline["concordant_pair_mass"]
        + baseline["discordant_pair_mass"],
        baseline_native_unequal_mass=baseline["native_concordant_pair_mass"]
        + baseline["native_discordant_pair_mass"],
        baseline_tick_unequal_pair_count=baseline["tick_unequal_pair_count"],
        baseline_native_unequal_pair_count=baseline["native_unequal_pair_count"],
        units=len(deletions),
        pair_supported_units=sum(d["removed_pair_count"] > 0 for d in deletions),
        tick_unequal_supported_units=sum(
            d["removed_tick_unequal_pair_count"] > 0 for d in deletions
        ),
        native_unequal_supported_units=sum(
            d["removed_native_unequal_pair_count"] > 0 for d in deletions
        ),
    )
    for field in [
        "removed_pair_share",
        "removed_tick_unequal_share",
        "removed_native_unequal_share",
    ]:
        values = [d[field] for d in deletions if d[field] is not None]
        r["max_" + field] = max(values) if values else None
    for metric in METRICS:
        base = baseline[metric]
        values = [d[metric] for d in deletions if d[metric] is not None]
        deltas = [
            d["delta_" + metric] for d in deletions if d["delta_" + metric] is not None
        ]
        r["baseline_" + metric] = base
        r.update(
            {
                metric + "_defined_deletions": len(values),
                metric + "_undefined_deletions": len(deletions) - len(values),
                metric + "_min": min(values) if values else None,
                metric + "_max": max(values) if values else None,
                metric + "_max_abs_delta": max(map(abs, deltas)) if deltas else None,
                metric + "_below_half": sum(v < Fraction(1, 2) for v in values),
                metric + "_at_half": sum(v == Fraction(1, 2) for v in values),
                metric + "_above_half": sum(v > Fraction(1, 2) for v in values),
                metric
                + "_strict_side_flips": (
                    sum(
                        (base < Fraction(1, 2) < v or v < Fraction(1, 2) < base)
                        for v in values
                    )
                    if base is not None
                    else 0
                ),
            }
        )
    return r


def influence(rows, lookup, target, score, label, units=None, cache=None):
    metadata = {} if units is None else None
    if units is None:
        units, _ = unit_audits(rows, lookup, metadata)
    by_case = defaultdict(list)
    for r in rows:
        by_case[lookup[previous.identity(r)]["case_id"]].append(r)
    case_scopes = {}
    cache = {} if cache is None else cache
    for cid, rr in sorted(by_case.items()):
        weights = [lookup[previous.identity(r)]["case_row_weight"] for r in rr]
        case_scopes[cid] = prior.decomposition(
            rr, weights, lookup, target, score, label, cache
        )[0]
    totals = [
        sum_parts([values[i] for values in case_scopes.values()])
        for i in range(len(SCOPES))
    ]
    weights = [lookup[previous.identity(r)]["case_row_weight"] for r in rows]
    full = previous.influence.pair_masses(
        *previous.influence.weighted_histogram(rows, weights, target, score, label)
    )
    event_den = totals[0]["denominator"]
    selected = dict(
        records=len(rows),
        cases=len(by_case),
        events=len({(previous.identity(r), r["_event_id"]) for r in rows}),
    )
    result = []
    for i, total in enumerate(totals):
        baseline = decorate(total, full["denominator"], event_den)
        pieces = []
        for unit in units:
            ids = unit["case_ids"].split("+")
            assert ids and all(cid in case_scopes for cid in ids)
            removed = sum_parts([case_scopes[cid][i] for cid in ids])
            remaining = decorate(
                subtract(total, removed), full["denominator"], event_den
            )
            pieces.append(deletion_row(baseline, removed, remaining, unit, selected))
        summary = {
            kind: summarize(
                baseline, [r for r in pieces if r["unit_kind"] == kind], selected
            )
            for kind in KINDS
        }
        result.append((baseline, pieces, summary))
    return result


def aggregate(rows, lookup, eligible, references):
    before = {tuple(r[k] for k in [*RANK, "scope"]): r for r in references}
    assert len(before) == len(references)
    parents = defaultdict(list)
    for r in rows:
        if r["view"] == previous.VIEW:
            parents[r["partition"], r["noise"], r["variant"]].append(r)
    audit, deletions, summaries, baselines = [], [], [], []
    cache = {}
    for part, noise, arm, mode, pop in product(
        previous.SEEDS,
        previous.NOISES,
        previous.ARMS,
        previous.STRATA,
        parent.POPULATIONS,
    ):
        metadata = dict(
            partition=part,
            noise=noise,
            variant=arm,
            stratum=mode,
            population=pop,
            planner_selection="all",
        )
        rr = [
            r
            for r in parents[part, noise, arm]
            if (mode == "all" or r["stratum"] == mode)
            and (
                pop == "source"
                or lookup[previous.identity(r)]["family_id"] in eligible[mode]
            )
        ]
        units, _ = unit_audits(rr, lookup, metadata)
        audit.extend(previous.historical.exported(u) for u in units)
        for target, score, label in product(
            previous.TARGETS, previous.SCORES, previous.historical.LABELS
        ):
            key = dict(**metadata, target=target, score=score, label=label)
            for scope, (baseline, pieces, summary) in zip(
                SCOPES, influence(rr, lookup, target, score, label, units, cache)
            ):
                record = previous.historical.exported(
                    dict(**key, scope=scope, **baseline)
                )
                assert set(record) == set(
                    before[tuple(record[k] for k in [*RANK, "scope"])]
                )
                baselines.append(record)
                deletions.extend(
                    previous.historical.exported(dict(**key, scope=scope, **d))
                    for d in pieces
                )
                summaries.extend(
                    previous.historical.exported(
                        dict(**key, scope=scope, unit_kind=kind, **summary[kind])
                    )
                    for kind in KINDS
                )
    checks = compare(baselines, references, [*RANK, "scope"])
    return audit, deletions, summaries, checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    assert digest(BASE / "sources.json") == BASE_SHA
    source = json.loads((BASE / "sources.json").read_text())
    pinned = {
        **source["input_sha256"],
        "event-score-ties/sources.json": BASE_SHA,
        **{"event-score-ties/" + k: v for k, v in source["output_sha256"].items()},
    }
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_event_score_ties.py": source["analysis"]["script_sha256"],
    }
    for name, sha in pinned.items():
        assert digest(RESULTS / name) == sha, name
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    temporal = prior.BASE
    events = {r["event_id"]: r for r in read_rows(temporal / "events.csv")}
    assert len(events) == 3997
    rows, reconstruction = window.load_rows(events)
    families, cases, lookup = window.audit_units(
        read_rows(temporal / "accounting.csv"), rows
    )
    rows, mapping = prior.prior.prior.attach_events(rows, lookup, events)
    rows, times = prior.prior.attach_times(rows, lookup)
    rows, scores = prior.attach_scores(
        rows,
        lookup,
        read_rows(RESULTS / "contrast-risk-transfer/scores.csv"),
        read_rows(RESULTS / "fresh-seed-cutoff-transfer/scores.csv"),
    )
    comparisons = compare(
        scores, read_rows(BASE / "score-audit.csv"), ["case_id", "source_id"]
    )
    for actual, name, keys in [
        (cases, "cases", ["case_id"]),
        (families, "families", ["family_id"]),
        (mapping, "source-event-map", ["case_id", "source_id"]),
        (times, "source-time-audit", ["case_id", "source_id"]),
    ]:
        comparisons += compare(actual, read_rows(temporal / (name + ".csv")), keys)
    _, eligible = parent.family_support(families, rows, lookup)
    audit, deletions, summaries, checks = aggregate(
        rows, lookup, eligible, read_rows(BASE / "precision-summary.csv")
    )
    assert (len(audit), len(deletions), len(summaries)) == (782, 56304, 17280)
    args.output.mkdir(parents=True)
    for name, data in [
        ("unit-audit", audit),
        ("delete-one", deletions),
        ("influence-summary", summaries),
    ]:
        save_rows(args.output / (name + ".csv"), data)
    counts = dict(
        source_pairs=len(rows),
        source_states=len(scores),
        represented_cases=157,
        accounted_cases=len(cases),
        planned_families=len(families),
        census_events=len(events),
        unit_audit_rows=len(audit),
        case_unit_audits=sum(r["unit_kind"] == "case" for r in audit),
        family_unit_audits=sum(r["unit_kind"] == "family" for r in audit),
        delete_one_rows=len(deletions),
        summary_rows=len(summaries),
        prior_field_comparisons=comparisons + checks,
        reconstruction_rows=len(reconstruction),
    )
    result = dict(
        input_sha256=pinned,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        schema=dict(
            scopes=SCOPES,
            deletion_kinds=KINDS,
            metrics=METRICS,
            selection="fixed original source/common_family populations; selected-state units only",
            weighting="original 1/n_case, no renormalization or eligibility recomputation",
            ranges="exhaustive single-unit deletion sensitivity, not confidence intervals",
            retrospective=True,
        ),
        inherited_validation=source["inherited_validation"],
        audit=counts,
        output_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(counts, indent=2), flush=True)


if __name__ == "__main__":
    main()
