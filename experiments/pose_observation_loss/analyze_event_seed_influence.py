#!/usr/bin/env python3
"""Retain or delete each of three planned seeds in fixed event/time populations."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import product
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_case_window_ranks import compare
import analyze_event_unit_influence as prior
from summarize_stopping_preview import save_rows

ROOT, RESULTS = prior.ROOT, prior.RESULTS
previous, parent, window = prior.previous, prior.parent, prior.window
ties = prior.prior
RANK, SCOPES = prior.RANK, prior.SCOPES
SELECT, METRICS, COUNTS = prior.SELECT, prior.METRICS, prior.COUNTS
BASE = RESULTS / "event-unit-influence"
BASE_SHA = "48d23a8c0deca5eaa9755644fecbd5960cafafb03e504cc96c54760f4e90b71f"
PROTOCOL = RESULTS / "event-seed-influence/protocol.md"
SEED_FIELDS = ["numerator", "denominator", "native_numerator", *COUNTS[:9], *METRICS]


def planned_seeds(partition, seeds=None):
    assert partition in previous.SEEDS, "unknown partition"
    seeds = tuple(previous.SEEDS[partition] if seeds is None else seeds)
    assert (
        len(seeds) == 3 and len(set(seeds)) == 3
    ), "three distinct planned seeds required"
    assert all(
        isinstance(s, str) and s.isascii() and s.isdecimal() and s == str(int(s))
        for s in seeds
    ), "canonical seed strings required"
    assert set(seeds) == set(previous.SEEDS[partition]), "unknown planned seed"
    return seeds


def seed_audits(rows, lookup, partition, metadata=None):
    metadata = {} if metadata is None else metadata
    seeds = planned_seeds(partition)
    assert all(r["partition"] == partition and r["seed"] in seeds for r in rows)
    units = []
    for seed in seeds:
        rr = [r for r in rows if r["seed"] == seed]
        cases = sorted({lookup[previous.identity(r)]["case_id"] for r in rr})
        families = sorted({lookup[previous.identity(r)]["family_id"] for r in rr})
        units.append(
            dict(
                **metadata,
                unit_kind="seed",
                unit_id=seed,
                case_ids="+".join(cases),
                family_ids="+".join(families),
                records=len(rr),
                cases=len(cases),
                events=len({(previous.identity(r), r["_event_id"]) for r in rr}),
                case_mass=sum(
                    (lookup[previous.identity(r)]["case_row_weight"] for r in rr),
                    Fraction(),
                ),
            )
        )
    return units


def summarize(baseline, pieces, selected):
    r = prior.summarize(baseline, pieces, selected)
    assert len(pieces) == 3
    r["represented_units"] = sum(p["removed_records"] > 0 for p in pieces)
    for metric in METRICS:
        values = [
            p["seed_" + metric] for p in pieces if p["seed_" + metric] is not None
        ]
        r[metric + "_seed_only_defined"] = len(values)
        r[metric + "_seed_only_undefined"] = 3 - len(values)
        r[metric + "_seed_only_min"] = min(values, default=None)
        r[metric + "_seed_only_max"] = max(values, default=None)
    return r


def influence(rows, lookup, partition, target, score, label, units=None, cache=None):
    seeds = planned_seeds(partition)
    if units is None:
        units = seed_audits(rows, lookup, partition)
    assert [u["unit_id"] for u in units] == list(seeds)
    by_case = defaultdict(list)
    for r in rows:
        assert r["partition"] == partition and r["seed"] in seeds
        by_case[lookup[previous.identity(r)]["case_id"]].append(r)
    case_scopes = {}
    cache = {} if cache is None else cache
    for cid, rr in sorted(by_case.items()):
        weights = [lookup[previous.identity(r)]["case_row_weight"] for r in rr]
        case_scopes[cid] = ties.decomposition(
            rr, weights, lookup, target, score, label, cache
        )[0]
    grouped_ids = []
    for unit in units:
        ids = unit["case_ids"].split("+") if unit["case_ids"] else []
        assert all(
            cid in by_case and by_case[cid][0]["seed"] == unit["unit_id"] for cid in ids
        )
        grouped_ids.extend(ids)
    assert len(grouped_ids) == len(set(grouped_ids)) and set(grouped_ids) == set(
        by_case
    )
    totals = [
        prior.sum_parts([v[i] for v in case_scopes.values()])
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
        baseline = prior.decorate(total, full["denominator"], event_den)
        pieces, seed_parts = [], []
        for unit in units:
            ids = unit["case_ids"].split("+") if unit["case_ids"] else []
            removed = prior.sum_parts([case_scopes[cid][i] for cid in ids])
            seed_parts.append(removed)
            remaining = prior.decorate(
                prior.subtract(total, removed), full["denominator"], event_den
            )
            kept = prior.decorate(removed, full["denominator"], event_den)
            row = prior.deletion_row(baseline, removed, remaining, unit, selected)
            row.update({"seed_" + f: kept[f] for f in SEED_FIELDS})
            pieces.append(row)
        assert prior.sum_parts(seed_parts) == total
        result.append((baseline, pieces, summarize(baseline, pieces, selected)))
    return result


def aggregate(rows, lookup, eligible, references):
    before = {tuple(r[k] for k in [*RANK, "scope"]): r for r in references}
    assert len(before) == len(references)
    parents = defaultdict(list)
    for r in rows:
        if r["view"] == previous.VIEW:
            parents[r["partition"], r["noise"], r["variant"]].append(r)
    audit, effects, summaries, baselines = [], [], [], []
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
        units = seed_audits(rr, lookup, part, metadata)
        audit.extend(previous.historical.exported(u) for u in units)
        for target, score, label in product(
            previous.TARGETS, previous.SCORES, previous.historical.LABELS
        ):
            key = dict(**metadata, target=target, score=score, label=label)
            for scope, (baseline, pieces, summary) in zip(
                SCOPES, influence(rr, lookup, part, target, score, label, units, cache)
            ):
                record = previous.historical.exported(
                    dict(**key, scope=scope, **baseline)
                )
                assert set(record) == set(
                    before[tuple(record[k] for k in [*RANK, "scope"])]
                )
                baselines.append(record)
                effects.extend(
                    previous.historical.exported(dict(**key, scope=scope, **d))
                    for d in pieces
                )
                summaries.append(
                    previous.historical.exported(
                        dict(**key, scope=scope, unit_kind="seed", **summary)
                    )
                )
    checks = compare(baselines, references, [*RANK, "scope"])
    return audit, effects, summaries, checks


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
        "event-unit-influence/sources.json": BASE_SHA,
        **{"event-unit-influence/" + k: v for k, v in source["output_sha256"].items()},
    }
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_event_unit_influence.py": source["analysis"]["script_sha256"],
    }
    for name, sha in pinned.items():
        assert digest(RESULTS / name) == sha, name
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    temporal = ties.BASE
    events = {r["event_id"]: r for r in read_rows(temporal / "events.csv")}
    assert len(events) == 3997
    rows, reconstruction = window.load_rows(events)
    families, cases, lookup = window.audit_units(
        read_rows(temporal / "accounting.csv"), rows
    )
    rows, mapping = ties.prior.prior.attach_events(rows, lookup, events)
    rows, times = ties.prior.attach_times(rows, lookup)
    rows, scores = ties.attach_scores(
        rows,
        lookup,
        read_rows(RESULTS / "contrast-risk-transfer/scores.csv"),
        read_rows(RESULTS / "fresh-seed-cutoff-transfer/scores.csv"),
    )
    comparisons = compare(
        scores,
        read_rows(ties.RESULTS / "event-score-ties/score-audit.csv"),
        ["case_id", "source_id"],
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
        rows,
        lookup,
        eligible,
        read_rows(ties.RESULTS / "event-score-ties/precision-summary.csv"),
    )
    assert (len(audit), len(deletions), len(summaries)) == (360, 25920, 8640)
    args.output.mkdir(parents=True)
    for name, data in [
        ("seed-audit", audit),
        ("seed-effects", deletions),
        ("sensitivity-summary", summaries),
    ]:
        save_rows(args.output / (name + ".csv"), data)
    counts = dict(
        source_pairs=len(rows),
        source_states=len(scores),
        represented_cases=157,
        accounted_cases=len(cases),
        planned_families=len(families),
        census_events=len(events),
        seed_audit_rows=len(audit),
        represented_seed_audits=sum(r["records"] > 0 for r in audit),
        seed_effect_rows=len(deletions),
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
            deletion_kinds=["seed"],
            planned_seeds=previous.SEEDS,
            seed_fields="seed_ prefix means retain only that seed; unprefixed means deletion",
            metrics=METRICS,
            selection="fixed original source/common_family populations; all three planned seeds including empty selections",
            weighting="original 1/n_case, no renormalization or eligibility recomputation",
            ranges="exhaustive seed deletion and seed-only sensitivity, not predictive validation or confidence intervals",
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
