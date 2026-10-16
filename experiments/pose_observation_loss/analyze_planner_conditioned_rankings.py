#!/usr/bin/env python3
"""Describe planner-specific rankings on source and common-family support."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import product
import json
from pathlib import Path
import shutil

from analyze_epoch_preview import digest, read_rows
import analyze_planner_window_sensitivity as window
from summarize_stopping_preview import save_rows

previous = window.previous
ROOT, RESULTS = window.ROOT, window.RESULTS
PROTOCOL = RESULTS / "planner-conditioned-rankings/protocol.md"
PLANNERS = ["A*", "JPS", "Theta*"]
NAMES = {"A*": "astar", "JPS": "jps", "Theta*": "theta"}
POPULATIONS = ["source", "common_family"]
PINNED = {
    **window.PINNED,
    "planner-window-sensitivity/sources.json": "d9ae96688fc1e176606b544835f3b1a9dd3cf7accfcfb71976361f39d2a98550",
    "planner-window-sensitivity/cases.csv": "3acb34f55fe03e8a09ea20e8a349516c67e51a69a91e5d862855766e385b714a",
    "planner-window-sensitivity/families.csv": "abefa6fb5ae76a151cb1dfdb435a2fb71c0f203bf608b093c11d61ddcd9d21f5",
    "planner-window-sensitivity/component-summary.csv": "a39c40edefc2235819a317d495b7a21350cfbd6e2ba4a87d0923be96b385b321",
    "planner-window-sensitivity/ranking-summary.csv": "6777f9d7971c30a311577ab86837560e7ad6f36a4045fca71e06b41764873720",
}
COPIES = {
    **previous.COPIES,
    "cases.csv": "planner-window-sensitivity/cases.csv",
    "families.csv": "planner-window-sensitivity/families.csv",
}
GROUP = [*previous.GROUP, "population", "planner_selection"]
RANK = [*GROUP, "score", "label"]


def support(rows, weights, lookup):
    return dict(
        **previous.support(rows, weights),
        families=len({lookup[previous.identity(r)]["family_id"] for r in rows}),
    )


def family_support(families, rows, lookup):
    by_family = defaultdict(list)
    for r in rows:
        if r["view"] == previous.VIEW:
            by_family[lookup[previous.identity(r)]["family_id"]].append(r)
    audits, eligible = [], defaultdict(set)
    for family, mode in product(families, previous.STRATA):
        assert set(family["planned_planners"].split("+")) == set(PLANNERS)
        rr = [
            r
            for r in by_family[family["family_id"]]
            if mode == "all" or r["stratum"] == mode
        ]
        planners = {r["planner"] for r in rr}
        common = planners == set(PLANNERS)
        if common:
            eligible[mode].add(family["family_id"])
        record = dict(
            **{
                k: family[k]
                for k in [
                    "family_id",
                    *window.FAMILY,
                    "partition",
                    "accounting_partition",
                    "noise",
                    "planned_planners",
                ]
            },
            stratum=mode,
            source_records=len(rr),
            represented_planners="+".join(sorted(planners)),
            missing_source_planners="+".join(sorted(set(PLANNERS) - planners)),
            common_family=common,
        )
        for planner in PLANNERS:
            selected = [r for r in rr if r["planner"] == planner]
            record[NAMES[planner] + "_records"] = len(selected)
            record[NAMES[planner] + "_case_mass"] = sum(
                (lookup[previous.identity(r)]["case_row_weight"] for r in selected),
                Fraction(),
            )
        audits.append(previous.historical.exported(record))
    return audits, eligible


def planner_decomposition(rows, weights, target, score, label, seeds):
    """Ordered planner blocks, with exact inherited mass and seed intersections."""
    assert len(rows) == len(weights) and all(w > 0 for w in weights)
    assert all(r["planner"] in PLANNERS and r["seed"] in seeds for r in rows)
    histogram = previous.influence.weighted_histogram
    pair_masses = previous.influence.pair_masses
    groups = {}
    for planner in PLANNERS:
        selected = [(r, w) for r, w in zip(rows, weights) if r["planner"] == planner]
        rr, ww = [r for r, w in selected], [w for r, w in selected]
        groups[planner] = dict(
            hist=histogram(rr, ww, target, score, label),
            positive_rows=sum(r[target]["outside"][label] for r in rr),
            negative_rows=sum(not r[target]["outside"][label] for r in rr),
            positive_cases=len(
                {previous.identity(r) for r in rr if r[target]["outside"][label]}
            ),
            negative_cases=len(
                {previous.identity(r) for r in rr if not r[target]["outside"][label]}
            ),
            seeds={
                seed: histogram(
                    [r for r, w in selected if r["seed"] == seed],
                    [w for r, w in selected if r["seed"] == seed],
                    target,
                    score,
                    label,
                )
                for seed in seeds
            },
        )
    full, _ = previous.decompose(rows, weights, target, score, label, seeds)
    blocks = []
    for positive, negative in product(PLANNERS, repeat=2):
        p, n = groups[positive], groups[negative]
        mass = pair_masses(n["hist"][0], p["hist"][1])
        same_seed = [pair_masses(n["seeds"][s][0], p["seeds"][s][1]) for s in seeds]
        num = sum((b["numerator"] for b in same_seed), Fraction())
        den = sum((b["denominator"] for b in same_seed), Fraction())
        blocks.append(
            dict(
                positive_planner=positive,
                negative_planner=negative,
                same_planner=positive == negative,
                **{k: p[k] for k in ["positive_rows", "positive_cases"]},
                **{k: n[k] for k in ["negative_rows", "negative_cases"]},
                positive_case_mass=sum(p["hist"][1].values(), Fraction()),
                negative_case_mass=sum(n["hist"][0].values(), Fraction()),
                **mass,
                same_seed_numerator=num,
                same_seed_denominator=den,
                same_seed_auc=num / den if den else None,
                same_seed_pair_share=(
                    den / mass["denominator"] if mass["denominator"] else None
                ),
                denominator_share=(
                    mass["denominator"] / full["denominator"]
                    if full["denominator"]
                    else None
                ),
                numerator_contribution=(
                    mass["numerator"] / full["denominator"]
                    if full["denominator"]
                    else None
                ),
            )
        )
    for field in previous.PAIR_FIELDS:
        assert sum((b[field] for b in blocks), Fraction()) == full[field]
    for field in ["same_seed_numerator", "same_seed_denominator"]:
        assert sum((b[field] for b in blocks), Fraction()) == full[field]
    for name, prefix in [("same_planner", ""), ("same_seed_planner", "same_seed_")]:
        num = sum(
            (b[prefix + "numerator"] for b in blocks if b["same_planner"]),
            Fraction(),
        )
        den = sum(
            (b[prefix + "denominator"] for b in blocks if b["same_planner"]),
            Fraction(),
        )
        full[name + "_numerator"] = num
        full[name + "_denominator"] = den
        full[name + "_auc"] = num / den if den else None
        full[name + "_pair_share"] = (
            den / full["denominator"] if full["denominator"] else None
        )
    return full, blocks


def summaries(rows, families, lookup):
    audit, eligible = family_support(families, rows, lookup)
    parents = defaultdict(list)
    for r in rows:
        if r["view"] == previous.VIEW:
            parents[r["partition"], r["noise"], r["variant"]].append(r)
    components, ranks, blocks, seed_support = [], [], [], []
    for partition, noise, variant in product(
        previous.SEEDS, previous.NOISES, previous.ARMS
    ):
        parent = parents[partition, noise, variant]
        weights = [lookup[previous.identity(r)]["case_row_weight"] for r in parent]
        parent_mass = sum(weights, Fraction())
        assert parent_mass == len({previous.identity(r) for r in parent})
        for mode, population in product(previous.STRATA, POPULATIONS):
            selected = [
                (r, w)
                for r, w in zip(parent, weights)
                if (mode == "all" or r["stratum"] == mode)
                and (
                    population == "source"
                    or lookup[previous.identity(r)]["family_id"] in eligible[mode]
                )
            ]
            for target in previous.TARGETS:
                fulls, pieces = {}, {}
                for score, label in product(
                    previous.SCORES, previous.historical.LABELS
                ):
                    fulls[score, label], pieces[score, label] = planner_decomposition(
                        [r for r, w in selected],
                        [w for r, w in selected],
                        target,
                        score,
                        label,
                        previous.SEEDS[partition],
                    )
                    for piece in pieces[score, label]:
                        blocks.append(
                            previous.historical.exported(
                                dict(
                                    partition=partition,
                                    noise=noise,
                                    variant=variant,
                                    stratum=mode,
                                    target=target,
                                    population=population,
                                    score=score,
                                    label=label,
                                    **piece,
                                )
                            )
                        )
                for planner in ["all", *PLANNERS]:
                    subset = [
                        (r, w)
                        for r, w in selected
                        if planner == "all" or r["planner"] == planner
                    ]
                    rr, ww = [r for r, w in subset], [w for r, w in subset]
                    common = dict(
                        partition=partition,
                        noise=noise,
                        variant=variant,
                        stratum=mode,
                        target=target,
                        population=population,
                        planner_selection=planner,
                        **support(rr, ww, lookup),
                        group_case_mass=parent_mass,
                        case_mass_share=(
                            sum(ww, Fraction()) / parent_mass if parent_mass else None
                        ),
                    )
                    components.append(
                        previous.historical.exported(
                            dict(
                                **common,
                                **previous.historical.component_metrics(rr, ww, target),
                            )
                        )
                    )
                    for seed in previous.SEEDS[partition]:
                        ss = [(r, w) for r, w in subset if r["seed"] == seed]
                        sr, sw = [r for r, w in ss], [w for r, w in ss]
                        record = dict(
                            **{k: common[k] for k in GROUP},
                            seed=seed,
                            **support(sr, sw, lookup),
                            group_case_mass=parent_mass,
                            stratum_case_mass=sum(ww, Fraction()),
                        )
                        for label in previous.historical.LABELS:
                            outside = [
                                (r, w) for r, w in ss if r[target]["outside"][label]
                            ]
                            mass = sum((w for r, w in outside), Fraction())
                            record[label + "_outside_rows"] = len(outside)
                            record[label + "_outside_cases"] = len(
                                {previous.identity(r) for r, w in outside}
                            )
                            record[label + "_outside_case_mass"] = mass
                            record[label + "_outside_fraction"] = (
                                mass / sum(sw, Fraction()) if sw else None
                            )
                        seed_support.append(previous.historical.exported(record))
                    for score, label in product(
                        previous.SCORES, previous.historical.LABELS
                    ):
                        metrics = (
                            fulls[score, label]
                            if planner == "all"
                            else previous.decompose(
                                rr, ww, target, score, label, previous.SEEDS[partition]
                            )[0]
                        )
                        if planner != "all":
                            diagonal = next(
                                p
                                for p in pieces[score, label]
                                if p["positive_planner"]
                                == p["negative_planner"]
                                == planner
                            )
                            for field in [
                                *previous.PAIR_FIELDS,
                                "auc",
                                "same_seed_numerator",
                                "same_seed_denominator",
                                "same_seed_auc",
                            ]:
                                assert metrics[field] == diagonal[field]
                            metrics.update(
                                {
                                    name + "_" + suffix: metrics[prefix + suffix]
                                    for name, prefix in [
                                        ("same_planner", ""),
                                        ("same_seed_planner", "same_seed_"),
                                    ]
                                    for suffix in ["numerator", "denominator", "auc"]
                                }
                            )
                            metrics["same_planner_pair_share"] = (
                                Fraction(1) if metrics["denominator"] else None
                            )
                            metrics["same_seed_planner_pair_share"] = metrics[
                                "same_seed_pair_share"
                            ]
                        ranks.append(
                            previous.historical.exported(
                                dict(**common, score=score, label=label, **metrics)
                            )
                        )
    return audit, components, ranks, blocks, seed_support


def check_previous(components, ranks):
    """All source/mixed case-reference fields must retain their saved values."""
    mapping = {
        "analysis_units": "cases",
        "analysis_mass": "case_mass",
        "parent_units": "group_case_mass",
        "parent_mass": "group_case_mass",
        "mass_share": "case_mass_share",
    }
    count = 0
    for filename, values, key in [
        ("component-summary.csv", components, previous.GROUP),
        ("ranking-summary.csv", ranks, previous.RANK),
    ]:
        lookup = {
            tuple(r[k] for k in key): r
            for r in values
            if r["population"] == "source" and r["planner_selection"] == "all"
        }
        old = [
            r
            for r in read_rows(RESULTS / "planner-window-sensitivity" / filename)
            if r["weighting"] == "case"
        ]
        assert len(lookup) == len(old)
        for before in old:
            after = lookup[tuple(before[k] for k in key)]
            for field, expected in before.items():
                if field == "weighting":
                    value = "case"
                elif field in ["auc_difference", "same_seed_auc_difference"]:
                    value = 0.0 if after[field.replace("_difference", "")] != "" else ""
                else:
                    value = after[
                        mapping.get(
                            field, field.replace("_analysis_mass", "_case_mass")
                        )
                    ]
                if isinstance(value, (bool, str)):
                    assert str(value) == expected, (field, value, expected)
                else:
                    previous.close(value, expected)
                count += 1
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for name, sha in PINNED.items():
        assert digest(RESULTS / name) == sha, name
    source = json.loads(
        (RESULTS / "planner-window-sensitivity/sources.json").read_text()
    )
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_planner_window_sensitivity.py": source["analysis"]["script_sha256"],
    }
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    events_list = read_rows(RESULTS / previous.COPIES["events.csv"])
    events = {r["event_id"]: r for r in events_list}
    assert len(events) == len(events_list) == 3997
    accounting = read_rows(RESULTS / previous.COPIES["accounting.csv"])
    rows, reconstruction = window.load_rows(events)
    families, cases, lookup = window.audit_units(accounting, rows)
    assert len(rows) == 32985 and len(cases) == 972 and len(families) == 324
    data = summaries(rows, families, lookup)
    assert tuple(map(len, data)) == (1620, 960, 8640, 19440, 2880)
    comparisons = check_previous(data[1], data[2])
    args.output.mkdir(parents=True)
    for name, values in zip(
        [
            "family-support",
            "component-summary",
            "ranking-summary",
            "planner-blocks",
            "seed-support",
        ],
        data,
    ):
        save_rows(args.output / (name + ".csv"), values)
    for name, original in COPIES.items():
        shutil.copyfile(RESULTS / original, args.output / name)
        assert digest(args.output / name) == PINNED[original]
    audit = dict(
        source_pairs=len(rows),
        source_states=len(rows) // 3,
        source_cases=sum(c["parent_records"] > 0 for c in cases),
        accounted_cases=len(cases),
        planned_families=len(families),
        family_support_rows=len(data[0]),
        component_rows=len(data[1]),
        ranking_rows=len(data[2]),
        planner_blocks=len(data[3]),
        seed_support_rows=len(data[4]),
        prior_field_comparisons=comparisons,
        reconstruction=reconstruction,
    )
    sources = dict(
        input_sha256=PINNED,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        schema=dict(
            populations=POPULATIONS,
            planners=PLANNERS,
            family_key=window.FAMILY,
            view=previous.VIEW,
            event_kind="all",
            budget="all_available",
            targets=previous.TARGETS,
            scores=previous.SCORES,
            labels=previous.historical.LABELS,
            strata=previous.STRATA,
            seeds=previous.SEEDS,
            weighting="original 1/n_case before stratum/planner/family selection",
            common_family="all three planned planners represented within this stratum; label/score blind",
            retrospective=True,
        ),
        inherited_validation=source["inherited_validation"],
        audit=audit,
        output_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
