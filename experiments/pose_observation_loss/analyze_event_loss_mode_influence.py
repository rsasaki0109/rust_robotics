#!/usr/bin/env python3
"""Loss-mode deletion/retention and focused full-case subsets."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import product
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_case_window_ranks import compare
import analyze_event_planner_influence as prior
from summarize_stopping_preview import save_rows

ROOT, RESULTS = prior.ROOT, prior.RESULTS
previous, parent, window, ties, unit = (
    prior.previous,
    prior.parent,
    prior.window,
    prior.ties,
    prior.unit,
)
RANK, SCOPES, SELECT, METRICS, COUNTS = (
    prior.RANK,
    prior.SCOPES,
    prior.SELECT,
    prior.METRICS,
    prior.COUNTS,
)
SUBSET_FIELDS, CELL_FIELDS, CELL_MASS = (
    prior.SUBSET_FIELDS,
    prior.CELL_FIELDS,
    prior.CELL_MASS,
)
SCENARIOS = prior.SCENARIOS
PLANNERS = ("A*", "JPS", "Theta*")
LOSS_MODES = ("none", "random", "blackout")
BASE = RESULTS / "event-planner-influence"
BASE_SHA = "8a18934fc97d3bb4e3c36e305274cc8d4001ee277ed1d90a0e240dcd92d1d85d"
PROTOCOL = RESULTS / "event-loss-mode-influence/protocol.md"


def planned_modes(values=None):
    values = LOSS_MODES if values is None else tuple(values)
    assert (
        len(values) == 3 and len(set(values)) == 3
    ), "three distinct planned loss modes required"
    assert all(
        isinstance(v, str) and v and v == v.strip() for v in values
    ), "canonical loss-mode strings required"
    assert set(values) == set(LOSS_MODES), "unknown planned loss mode"
    return LOSS_MODES


focused = prior.focused


def make_unit(rows, lookup, metadata, mode, seed=None, scenario=None, planner=None):
    cids = sorted({lookup[previous.identity(r)]["case_id"] for r in rows})
    fids = sorted({lookup[previous.identity(r)]["family_id"] for r in rows})
    result = dict(
        **metadata,
        unit_kind="loss_mode",
        unit_id=mode,
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
        assert (
            len(cids) <= 1
        ), "a complete mode/seed/scenario/planner leaf has at most one full case"
        result.update(seed=seed, scenario=scenario, planner=planner)
    return result


def mode_audits(rows, lookup, partition, metadata=None):
    modes, seeds, scenarios, planners = (
        planned_modes(),
        prior.prior.prior.planned_seeds(partition),
        prior.prior.planned_scenarios(),
        prior.planned_planners(),
    )
    metadata = {} if metadata is None else metadata
    assert all(
        r["partition"] == partition
        and r["mode"] in modes
        and r["seed"] in seeds
        and r["scenario"] in scenarios
        and r["planner"] in planners
        for r in rows
    )
    units, cells = [], []
    for mode in modes:
        rr = [r for r in rows if r["mode"] == mode]
        units.append(make_unit(rr, lookup, metadata, mode))
        cells.extend(
            make_unit(
                [
                    r
                    for r in rr
                    if r["seed"] == seed
                    and r["scenario"] == scenario
                    and r["planner"] == planner
                ],
                lookup,
                metadata,
                mode,
                seed,
                scenario,
                planner,
            )
            for scenario, seed, planner in product(scenarios, seeds, planners)
        )
    return units, cells


def summarize(baseline, pieces, selected):
    result = unit.summarize(baseline, pieces, selected)
    assert len(pieces) == 3
    result["represented_units"] = sum(p["removed_records"] > 0 for p in pieces)
    for metric in METRICS:
        values = [
            p["mode_" + metric] for p in pieces if p["mode_" + metric] is not None
        ]
        result[metric + "_mode_only_defined"] = len(values)
        result[metric + "_mode_only_undefined"] = 3 - len(values)
        result[metric + "_mode_only_min"] = min(values, default=None)
        result[metric + "_mode_only_max"] = max(values, default=None)
    return result


def collapse_cells(cells):
    seeds = sorted({c["seed"] for c in cells})
    assert len(seeds) == 3 and len(cells) == 81
    result = []
    for planner, scenario, seed in product(PLANNERS, SCENARIOS, seeds):
        rr = [
            c
            for c in cells
            if c["planner"] == planner
            and c["scenario"] == scenario
            and c["seed"] == seed
        ]
        assert len(rr) == 3 and {r["unit_id"] for r in rr} == set(LOSS_MODES)
        values = {
            f: sum((r[f] for r in rr), Fraction())
            for f in [
                "records",
                "cases",
                "events",
                "case_mass",
                *(f for f in CELL_FIELDS if f not in METRICS),
            ]
        }
        den = values["denominator"]
        c, d, nc, nd = [
            values[f]
            for f in [
                "concordant_pair_mass",
                "discordant_pair_mass",
                "native_concordant_pair_mass",
                "native_discordant_pair_mass",
            ]
        ]
        values.update(
            auc=values["numerator"] / den if den else None,
            native_auc=values["native_numerator"] / den if den else None,
            tick_unequal_concordance=c / (c + d) if c + d else None,
            native_unequal_concordance=nc / (nc + nd) if nc + nd else None,
        )
        result.append(
            dict(
                unit_kind="planner",
                unit_id=planner,
                seed=seed,
                scenario=scenario,
                **values
            )
        )
    return result


def influence(
    rows,
    lookup,
    partition,
    target,
    score,
    label,
    audits=None,
    cache=None,
    include_cells=True,
):
    modes, seeds, scenarios, planners = (
        planned_modes(),
        prior.prior.prior.planned_seeds(partition),
        prior.prior.planned_scenarios(),
        prior.planned_planners(),
    )
    units, cells = mode_audits(rows, lookup, partition) if audits is None else audits
    assert [u["unit_id"] for u in units] == list(modes)
    assert [
        (c["unit_id"], c["scenario"], c["seed"], c["planner"]) for c in cells
    ] == list(product(modes, scenarios, seeds, planners))
    by_case = defaultdict(list)
    for r in rows:
        assert (
            r["partition"] == partition
            and r["mode"] in modes
            and r["seed"] in seeds
            and r["scenario"] in scenarios
            and r["planner"] in planners
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

    def ids_for(u, leaf=False):
        ids = u["case_ids"].split("+") if u["case_ids"] else []
        assert not leaf or len(ids) <= 1
        assert all(
            cid in by_case
            and by_case[cid][0]["mode"] == u["unit_id"]
            and (
                not leaf
                or (
                    by_case[cid][0]["seed"] == u["seed"]
                    and by_case[cid][0]["scenario"] == u["scenario"]
                    and by_case[cid][0]["planner"] == u["planner"]
                )
            )
            for cid in ids
        )
        return ids

    for sets in [[ids_for(u) for u in units], [ids_for(c, True) for c in cells]]:
        flat = [cid for ids in sets for cid in ids]
        assert len(flat) == len(set(flat)) and set(flat) == set(by_case)
    totals = [
        unit.sum_parts([values[i] for values in case_scopes.values()])
        for i in range(len(SCOPES))
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
        pieces, removed_parts, contributions, leaf_parts = [], [], [], []
        for u in units:
            removed = unit.sum_parts([case_scopes[cid][i] for cid in ids_for(u)])
            removed_parts.append(removed)
            kept = unit.decorate(removed, full["denominator"], totals[0]["denominator"])
            remaining = unit.decorate(
                unit.subtract(total, removed),
                full["denominator"],
                totals[0]["denominator"],
            )
            row = unit.deletion_row(baseline, removed, remaining, u, selected)
            row.update({"mode_" + f: kept[f] for f in SUBSET_FIELDS})
            pieces.append(row)
        if include_cells:
            for c in cells:
                value = unit.decorate(
                    unit.sum_parts([case_scopes[cid][i] for cid in ids_for(c, True)]),
                    full["denominator"],
                    totals[0]["denominator"],
                )
                leaf_parts.append(value)
                contributions.append(
                    dict(
                        unit_kind="loss_mode",
                        unit_id=c["unit_id"],
                        seed=c["seed"],
                        scenario=c["scenario"],
                        planner=c["planner"],
                        case_id=c["case_ids"],
                        **{
                            f: c[f] for f in ["records", "cases", "events", "case_mass"]
                        },
                        **{f: value[f] for f in CELL_FIELDS}
                    )
                )
            assert unit.sum_parts(leaf_parts) == total
        assert unit.sum_parts(removed_parts) == total
        result.append(
            (baseline, pieces, summarize(baseline, pieces, selected), contributions)
        )
    return result


def aggregate(rows, lookup, eligible, references, cell_references):
    before = {tuple(r[k] for k in [*RANK, "scope"]): r for r in references}
    assert len(before) == len(references)
    parents = defaultdict(list)
    for r in rows:
        if r["view"] == previous.VIEW:
            parents[r["partition"], r["noise"], r["variant"]].append(r)
    audit, effects, summaries, contributions, baselines, collapsed = (
        [],
        [],
        [],
        [],
        [],
        [],
    )
    cache = {}
    for part, noise, arm, stratum, pop in product(
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
            stratum=stratum,
            population=pop,
            planner_selection="all",
        )
        rr = [
            r
            for r in parents[part, noise, arm]
            if (stratum == "all" or r["stratum"] == stratum)
            and (
                pop == "source"
                or lookup[previous.identity(r)]["family_id"] in eligible[stratum]
            )
        ]
        audits = mode_audits(rr, lookup, part, metadata)
        audit.extend(previous.historical.exported(c) for c in audits[1])
        for target, score, label in product(
            previous.TARGETS, previous.SCORES, previous.historical.LABELS
        ):
            key = dict(**metadata, target=target, score=score, label=label)
            for scope, (baseline, pieces, summary, cells) in zip(
                SCOPES,
                influence(
                    rr, lookup, part, target, score, label, audits, cache, focused(key)
                ),
            ):
                record = previous.historical.exported(
                    dict(**key, scope=scope, **baseline)
                )
                assert set(record) == set(
                    before[tuple(record[k] for k in [*RANK, "scope"])]
                )
                baselines.append(record)
                effects.extend(
                    previous.historical.exported(dict(**key, scope=scope, **p))
                    for p in pieces
                )
                summaries.append(
                    previous.historical.exported(
                        dict(**key, scope=scope, unit_kind="loss_mode", **summary)
                    )
                )
                contributions.extend(
                    previous.historical.exported(dict(**key, scope=scope, **c))
                    for c in cells
                )
                if cells:
                    collapsed.extend(
                        previous.historical.exported(dict(**key, scope=scope, **c))
                        for c in collapse_cells(cells)
                    )
    cell_keys = [*RANK, "scope", "unit_kind", "unit_id", "seed", "scenario"]
    assert len(collapsed) == len(cell_references) == 20736 and all(
        focused(r) for r in cell_references
    )
    return (
        audit,
        effects,
        summaries,
        contributions,
        compare(baselines, references, [*RANK, "scope"]),
        compare(collapsed, cell_references, cell_keys),
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
        "event-planner-influence/sources.json": BASE_SHA,
        **{
            "event-planner-influence/" + k: v
            for k, v in source["output_sha256"].items()
        },
    }
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_event_planner_influence.py": source["analysis"]["script_sha256"],
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
    audit, deletions, summaries, contributions, checks, cell_checks = aggregate(
        rows,
        lookup,
        eligible,
        read_rows(ties.RESULTS / "event-score-ties/precision-summary.csv"),
        read_rows(BASE / "planner-cells.csv"),
    )
    assert (len(audit), len(deletions), len(summaries), len(contributions)) == (
        9720,
        25920,
        8640,
        62208,
    )
    args.output.mkdir(parents=True)
    for name, data in [
        ("mode-audit", audit),
        ("mode-effects", deletions),
        ("sensitivity-summary", summaries),
        ("mode-cells", contributions),
    ]:
        save_rows(args.output / (name + ".csv"), data)
    counts = dict(
        source_pairs=len(rows),
        source_states=len(scores),
        represented_cases=157,
        accounted_cases=len(cases),
        planned_families=len(families),
        census_events=len(events),
        mode_audit_rows=len(audit),
        represented_leaf_audits=sum(r["records"] > 0 for r in audit),
        mode_effect_rows=len(deletions),
        mode_cell_rows=len(contributions),
        summary_rows=len(summaries),
        prior_field_comparisons=comparisons + checks,
        prior_planner_field_comparisons=cell_checks,
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
            deletion_kinds=["loss_mode"],
            planned_loss_modes=LOSS_MODES,
            single_case_leaves=True,
            planned_planners=PLANNERS,
            planned_scenarios=SCENARIOS,
            planned_seeds=previous.SEEDS,
            mode_fields="mode_ prefix means loss mode alone; unprefixed effect fields mean deletion",
            focused_cells="residual_sigma, Filter/Fusion, E/P, both targets and all scopes",
            metrics=METRICS,
            selection="fixed original source/common_family populations; all three loss modes and 81 complete case audit cells, including empty selections",
            weighting="original 1/n_case, no renormalization or eligibility recomputation",
            ranges="exhaustive loss-mode deletion, mode-only and fixed focused full-case sensitivity, not predictive validation or confidence intervals",
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
