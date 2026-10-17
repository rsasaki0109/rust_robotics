#!/usr/bin/env python3
"""Scenario deletion/retention with all nine planned seed/scenario cells."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import product
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_case_window_ranks import compare
import analyze_event_seed_influence as prior
from summarize_stopping_preview import save_rows

ROOT, RESULTS = prior.ROOT, prior.RESULTS
previous, parent, window, ties = prior.previous, prior.parent, prior.window, prior.ties
unit = prior.prior
RANK, SCOPES, SELECT, METRICS = prior.RANK, prior.SCOPES, prior.SELECT, prior.METRICS
COUNTS = prior.COUNTS
BASE = RESULTS / "event-seed-influence"
BASE_SHA = "fd8943676129642ab3edd66476faa2579f677aca29fe71a1576439159aac4176"
PROTOCOL = RESULTS / "event-scenario-influence/protocol.md"
SCENARIOS = ("dense_50x50", "maze_50x50", "open_50x50")
SUBSET_FIELDS = prior.SEED_FIELDS
CELL_MASS = [
    prefix + cls + "_pair_mass"
    for prefix in ["", "native_"]
    for cls in ["concordant", "tied", "discordant"]
]
CELL_FIELDS = [*CELL_MASS, *SUBSET_FIELDS]


def planned_scenarios(values=None):
    values = SCENARIOS if values is None else tuple(values)
    assert (
        len(values) == 3 and len(set(values)) == 3
    ), "three distinct planned scenarios required"
    assert all(
        isinstance(v, str) and v and v == v.strip() for v in values
    ), "canonical scenario strings required"
    assert set(values) == set(SCENARIOS), "unknown planned scenario"
    return SCENARIOS


def make_unit(rows, lookup, metadata, scenario, seed=None):
    cids = sorted({lookup[previous.identity(r)]["case_id"] for r in rows})
    fids = sorted({lookup[previous.identity(r)]["family_id"] for r in rows})
    result = dict(
        **metadata,
        unit_kind="scenario",
        unit_id=scenario,
        case_ids="+".join(cids),
        family_ids="+".join(fids),
        records=len(rows),
        cases=len(cids),
        events=len({(previous.identity(r), r["_event_id"]) for r in rows}),
        case_mass=sum(
            (lookup[previous.identity(r)]["case_row_weight"] for r in rows), Fraction()
        )
    )
    if seed is not None:
        result["seed"] = seed
    return result


def scenario_audits(rows, lookup, partition, metadata=None):
    seeds, scenarios = prior.planned_seeds(partition), planned_scenarios()
    metadata = {} if metadata is None else metadata
    assert all(
        r["partition"] == partition
        and r["seed"] in seeds
        and r["scenario"] in scenarios
        for r in rows
    )
    units, cells = [], []
    for scenario in scenarios:
        rr = [r for r in rows if r["scenario"] == scenario]
        units.append(make_unit(rr, lookup, metadata, scenario))
        cells.extend(
            make_unit(
                [r for r in rr if r["seed"] == seed], lookup, metadata, scenario, seed
            )
            for seed in seeds
        )
    return units, cells


def summarize(baseline, pieces, selected):
    result = unit.summarize(baseline, pieces, selected)
    assert len(pieces) == 3
    result["represented_units"] = sum(p["removed_records"] > 0 for p in pieces)
    for metric in METRICS:
        values = [
            p["scenario_" + metric]
            for p in pieces
            if p["scenario_" + metric] is not None
        ]
        result[metric + "_scenario_only_defined"] = len(values)
        result[metric + "_scenario_only_undefined"] = 3 - len(values)
        result[metric + "_scenario_only_min"] = min(values, default=None)
        result[metric + "_scenario_only_max"] = max(values, default=None)
    return result


def influence(rows, lookup, partition, target, score, label, audits=None, cache=None):
    seeds, scenarios = prior.planned_seeds(partition), planned_scenarios()
    units, cells = (
        scenario_audits(rows, lookup, partition) if audits is None else audits
    )
    assert [u["unit_id"] for u in units] == list(scenarios)
    assert [(c["unit_id"], c["seed"]) for c in cells] == list(product(scenarios, seeds))
    by_case = defaultdict(list)
    for r in rows:
        assert (
            r["partition"] == partition
            and r["seed"] in seeds
            and r["scenario"] in scenarios
        )
        by_case[lookup[previous.identity(r)]["case_id"]].append(r)
    cache = {} if cache is None else cache
    case_scopes = {
        cid: ties.decomposition(
            rr,
            [lookup[previous.identity(r)]["case_row_weight"] for r in rr],
            lookup,
            target,
            score,
            label,
            cache,
        )[0]
        for cid, rr in sorted(by_case.items())
    }
    cell_scopes, seen = {}, []
    for c in cells:
        ids = c["case_ids"].split("+") if c["case_ids"] else []
        assert all(
            cid in by_case
            and by_case[cid][0]["seed"] == c["seed"]
            and by_case[cid][0]["scenario"] == c["unit_id"]
            for cid in ids
        )
        seen.extend(ids)
        cell_scopes[c["unit_id"], c["seed"]] = [
            unit.sum_parts([case_scopes[cid][i] for cid in ids])
            for i in range(len(SCOPES))
        ]
    assert len(seen) == len(set(seen)) and set(seen) == set(by_case)
    totals = [
        unit.sum_parts([s[i] for s in case_scopes.values()]) for i in range(len(SCOPES))
    ]
    weights = [lookup[previous.identity(r)]["case_row_weight"] for r in rows]
    full = previous.influence.pair_masses(
        *previous.influence.weighted_histogram(rows, weights, target, score, label)
    )
    selected = dict(
        records=len(rows),
        cases=len(by_case),
        events=len({(previous.identity(r), r["_event_id"]) for r in rows}),
    )
    result = []
    for i, total in enumerate(totals):
        baseline = unit.decorate(total, full["denominator"], totals[0]["denominator"])
        pieces, contributions, removed_parts = [], [], []
        for u in units:
            removed = unit.sum_parts(
                [cell_scopes[u["unit_id"], seed][i] for seed in seeds]
            )
            removed_parts.append(removed)
            kept = unit.decorate(removed, full["denominator"], totals[0]["denominator"])
            remaining = unit.decorate(
                unit.subtract(total, removed),
                full["denominator"],
                totals[0]["denominator"],
            )
            row = unit.deletion_row(baseline, removed, remaining, u, selected)
            row.update({"scenario_" + f: kept[f] for f in SUBSET_FIELDS})
            pieces.append(row)
        for c in cells:
            value = unit.decorate(
                cell_scopes[c["unit_id"], c["seed"]][i],
                full["denominator"],
                totals[0]["denominator"],
            )
            contributions.append(
                dict(
                    unit_kind="scenario",
                    unit_id=c["unit_id"],
                    seed=c["seed"],
                    **{f: c[f] for f in ["records", "cases", "events", "case_mass"]},
                    **{f: value[f] for f in CELL_FIELDS}
                )
            )
        assert unit.sum_parts(removed_parts) == total
        assert unit.sum_parts([s[i] for s in cell_scopes.values()]) == total
        result.append(
            (baseline, pieces, summarize(baseline, pieces, selected), contributions)
        )
    return result


def aggregate(rows, lookup, eligible, references):
    before = {tuple(r[k] for k in [*RANK, "scope"]): r for r in references}
    assert len(before) == len(references)
    parents = defaultdict(list)
    for r in rows:
        if r["view"] == previous.VIEW:
            parents[r["partition"], r["noise"], r["variant"]].append(r)
    audit, effects, summaries, contributions, baselines = [], [], [], [], []
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
        audits = scenario_audits(rr, lookup, part, metadata)
        audit.extend(previous.historical.exported(c) for c in audits[1])
        for target, score, label in product(
            previous.TARGETS, previous.SCORES, previous.historical.LABELS
        ):
            key = dict(**metadata, target=target, score=score, label=label)
            for scope, (baseline, pieces, summary, cells) in zip(
                SCOPES, influence(rr, lookup, part, target, score, label, audits, cache)
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
                        dict(**key, scope=scope, unit_kind="scenario", **summary)
                    )
                )
                contributions.extend(
                    previous.historical.exported(dict(**key, scope=scope, **d))
                    for d in cells
                )
    return (
        audit,
        effects,
        summaries,
        contributions,
        compare(baselines, references, [*RANK, "scope"]),
    )


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
        "event-seed-influence/sources.json": BASE_SHA,
        **{"event-seed-influence/" + k: v for k, v in source["output_sha256"].items()},
    }
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_event_seed_influence.py": source["analysis"]["script_sha256"],
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
    audit, deletions, summaries, contributions, checks = aggregate(
        rows,
        lookup,
        eligible,
        read_rows(ties.RESULTS / "event-score-ties/precision-summary.csv"),
    )
    assert (len(audit), len(deletions), len(summaries), len(contributions)) == (
        1080,
        25920,
        8640,
        77760,
    )
    args.output.mkdir(parents=True)
    for name, data in [
        ("scenario-audit", audit),
        ("scenario-effects", deletions),
        ("sensitivity-summary", summaries),
        ("seed-scenario-pairs", contributions),
    ]:
        save_rows(args.output / (name + ".csv"), data)
    counts = dict(
        source_pairs=len(rows),
        source_states=len(scores),
        represented_cases=157,
        accounted_cases=len(cases),
        planned_families=len(families),
        census_events=len(events),
        scenario_audit_rows=len(audit),
        represented_seed_scenario_audits=sum(r["records"] > 0 for r in audit),
        scenario_effect_rows=len(deletions),
        seed_scenario_pair_rows=len(contributions),
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
            deletion_kinds=["scenario"],
            planned_scenarios=SCENARIOS,
            planned_seeds=previous.SEEDS,
            scenario_fields="scenario_ prefix means scenario alone; unprefixed effect fields mean deletion",
            metrics=METRICS,
            selection="fixed original source/common_family populations; all three scenarios and nine seed/scenario cells including empty selections",
            weighting="original 1/n_case, no renormalization or eligibility recomputation",
            ranges="exhaustive scenario deletion, scenario-only and seed/scenario sensitivity, not predictive validation or confidence intervals",
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
