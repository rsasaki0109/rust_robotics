#!/usr/bin/env python3
"""Partition fixed ranking comparisons by case, exact window, family and seed."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import product
import json
from pathlib import Path
import shutil

from analyze_epoch_preview import digest, read_rows
import analyze_planner_conditioned_rankings as parent
from summarize_stopping_preview import save_rows

previous, window = parent.previous, parent.window
ROOT, RESULTS = parent.ROOT, parent.RESULTS
PROTOCOL = RESULTS / "case-window-rank-decomposition/protocol.md"
BINS = [
    "same_case",
    "other_case_same_window",
    "other_window_same_family",
    "other_family_same_seed",
    "other_seed",
]
GROUP, RANK = parent.GROUP, parent.RANK
FIELDS = previous.PAIR_FIELDS
PINNED = {
    **parent.PINNED,
    "planner-conditioned-rankings/sources.json": "bc786d78bcd51cdcc8566527d1dc6c99f2bb473ffc28b2c1047ec87d78ce83cc",
    "planner-conditioned-rankings/ranking-summary.csv": "b32ceacb26b8c1ddb24667757156664cc5400a124861b18f0553cd7e949a5c58",
    "planner-conditioned-rankings/family-support.csv": "5d057898518bbe79ccf38075251c15ff686c508e8fa06f029dddf07c874bcdb2",
}
COPIES = {
    **parent.COPIES,
    "family-support.csv": "planner-conditioned-rankings/family-support.csv",
}


def category(positive, negative):
    if positive["case_id"] == negative["case_id"]:
        return BINS[0]
    if positive["window_id"] == negative["window_id"]:
        assert positive["family_id"] == negative["family_id"]
        return BINS[1]
    if positive["family_id"] == negative["family_id"]:
        assert positive["seed"] == negative["seed"]
        return BINS[2]
    return BINS[3] if positive["seed"] == negative["seed"] else BINS[4]


def decompose(rows, weights, lookup, target, score, label, parent_keys=None):
    assert len(rows) == len(weights) and all(w > 0 for w in weights)
    keys = sorted(
        {previous.identity(r) for r in rows} if parent_keys is None else parent_keys
    )
    assert len(keys) == len(set(keys))
    selected = defaultdict(list)
    for r, w in zip(rows, weights):
        key = previous.identity(r)
        assert key in keys and lookup[key]["case_row_weight"] == w
        selected[key].append((r, w))
    cases = {}
    histogram, pairs = (
        previous.influence.weighted_histogram,
        previous.influence.pair_masses,
    )
    for key in keys:
        rr, ww = [r for r, w in selected[key]], [w for r, w in selected[key]]
        hist = histogram(rr, ww, target, score, label)
        outside = sum(r[target]["outside"][label] for r in rr)
        cases[key] = dict(
            hist=hist,
            records=len(rr),
            case_mass=sum(ww, Fraction()),
            outside_rows=outside,
            inside_rows=len(rr) - outside,
            outside_case_mass=sum(hist[1].values(), Fraction()),
            inside_case_mass=sum(hist[0].values(), Fraction()),
            **pairs(*hist),
        )
    bins = {
        name: dict(
            bin=name,
            candidate_case_pairs=0,
            supported_case_pairs=0,
            **{f: Fraction() for f in FIELDS},
        )
        for name in BINS
    }
    checks = {
        name + "_" + f: Fraction()
        for name in ["same_seed", "same_planner", "same_seed_planner"]
        for f in ["numerator", "denominator"]
    }
    active = [key for key in keys if cases[key]["records"]]
    for pos, neg in product(active, repeat=2):
        p, n = lookup[pos], lookup[neg]
        assert p["window_id"] and n["window_id"]
        mass = pairs(cases[neg]["hist"][0], cases[pos]["hist"][1])
        bin_row = bins[category(p, n)]
        bin_row["candidate_case_pairs"] += 1
        bin_row["supported_case_pairs"] += mass["denominator"] > 0
        for field in FIELDS:
            bin_row[field] += mass[field]
        for name, condition in [
            ("same_seed", p["seed"] == n["seed"]),
            ("same_planner", p["planner"] == n["planner"]),
            (
                "same_seed_planner",
                (p["seed"], p["planner"]) == (n["seed"], n["planner"]),
            ),
        ]:
            if condition:
                for field in ["numerator", "denominator"]:
                    checks[name + "_" + field] += mass[field]
    full = pairs(*histogram(rows, weights, target, score, label))
    for field in FIELDS:
        assert sum((b[field] for b in bins.values()), Fraction()) == full[field]
    assert sum(b["candidate_case_pairs"] for b in bins.values()) == len(active) ** 2
    extra = {}
    for name, names in [
        ("same_case", BINS[:1]),
        ("same_window", BINS[:2]),
        ("same_family", BINS[:3]),
        ("between_case", BINS[1:]),
    ]:
        num = sum((bins[b]["numerator"] for b in names), Fraction())
        den = sum((bins[b]["denominator"] for b in names), Fraction())
        extra.update(
            {
                name + "_numerator": num,
                name + "_denominator": den,
                name + "_auc": num / den if den else None,
                name
                + "_pair_share": (
                    den / full["denominator"] if full["denominator"] else None
                ),
            }
        )
    for field in ["numerator", "denominator"]:
        assert (
            sum((bins[b][field] for b in BINS[:4]), Fraction())
            == checks["same_seed_" + field]
        )
    for name, id_field in [
        ("cases", "case_id"),
        ("windows", "window_id"),
        ("families", "family_id"),
    ]:
        by_id = defaultdict(lambda: [Fraction(), Fraction()])
        for key in active:
            for i, label_field in enumerate(["inside_case_mass", "outside_case_mass"]):
                by_id[lookup[key][id_field]][i] += cases[key][label_field]
        extra[name + "_with_both_classes"] = sum(
            a > 0 and b > 0 for a, b in by_id.values()
        )
    for b in bins.values():
        b["auc"] = b["numerator"] / b["denominator"] if b["denominator"] else None
        b["denominator_share"] = (
            b["denominator"] / full["denominator"] if full["denominator"] else None
        )
        b["numerator_contribution"] = (
            b["numerator"] / full["denominator"] if full["denominator"] else None
        )
    return extra, list(bins.values()), cases, {**full, **checks}


def case_strata(rows, lookup, eligible):
    by_case = defaultdict(list)
    for r in rows:
        if r["view"] == previous.VIEW:
            by_case[previous.identity(r)].append(r)
    audit = []
    for key in sorted(lookup):
        info = lookup[key]
        assert len(by_case[key]) == info["parent_records"]
        for mode in previous.STRATA:
            rr = [r for r in by_case[key] if mode == "all" or r["stratum"] == mode]
            weight = info["case_row_weight"]
            record = dict(
                **{
                    k: info[k]
                    for k in [
                        "case_id",
                        "family_id",
                        *window.KEY,
                        "partition",
                        "accounting_partition",
                        "noise",
                        "parent_records",
                        "case_row_weight",
                    ]
                },
                stratum=mode,
                records=len(rr),
                case_mass=weight * len(rr) if rr else Fraction(),
                common_family_eligible=info["family_id"] in eligible[mode],
            )
            for target, label in product(previous.TARGETS, previous.historical.LABELS):
                outside = sum(r[target]["outside"][label] for r in rr)
                prefix = target + "_" + label
                record[prefix + "_outside_rows"] = outside
                record[prefix + "_inside_rows"] = len(rr) - outside
                record[prefix + "_outside_case_mass"] = (
                    outside * weight if rr else Fraction()
                )
                record[prefix + "_inside_case_mass"] = (
                    (len(rr) - outside) * weight if rr else Fraction()
                )
            audit.append(previous.historical.exported(record))
    return audit


def summaries(rows, families, lookup):
    eligibility_audit, eligible = parent.family_support(families, rows, lookup)
    parents = defaultdict(list)
    for r in rows:
        if r["view"] == previous.VIEW:
            parents[r["partition"], r["noise"], r["variant"]].append(r)
    ranks, all_bins, case_ranks = [], [], []
    for partition, noise, arm in product(
        previous.SEEDS, previous.NOISES, previous.ARMS
    ):
        original = parents[partition, noise, arm]
        parent_keys = {previous.identity(r) for r in original}
        parent_mass = Fraction(len(parent_keys))
        for mode, population, target in product(
            previous.STRATA, parent.POPULATIONS, previous.TARGETS
        ):
            rr = [
                r
                for r in original
                if (mode == "all" or r["stratum"] == mode)
                and (
                    population == "source"
                    or lookup[previous.identity(r)]["family_id"] in eligible[mode]
                )
            ]
            ww = [lookup[previous.identity(r)]["case_row_weight"] for r in rr]
            common = dict(
                partition=partition,
                noise=noise,
                variant=arm,
                stratum=mode,
                target=target,
                population=population,
                planner_selection="all",
                **parent.support(rr, ww, lookup),
                group_case_mass=parent_mass,
                case_mass_share=(
                    sum(ww, Fraction()) / parent_mass if parent_mass else None
                ),
            )
            for score, label in product(previous.SCORES, previous.historical.LABELS):
                extra, bins, cases, cross = decompose(
                    rr, ww, lookup, target, score, label, parent_keys
                )
                reference, _ = parent.planner_decomposition(
                    rr, ww, target, score, label, previous.SEEDS[partition]
                )
                for field, value in cross.items():
                    assert value == reference[field], (field, value, reference[field])
                ranks.append(
                    previous.historical.exported(
                        dict(**common, score=score, label=label, **reference, **extra)
                    )
                )
                for bin_row in bins:
                    all_bins.append(
                        previous.historical.exported(
                            dict(
                                **{k: common[k] for k in GROUP},
                                score=score,
                                label=label,
                                **bin_row,
                            )
                        )
                    )
                if population == "source":
                    for key, c in cases.items():
                        info = lookup[key]
                        record = dict(
                            **{
                                k: info[k]
                                for k in [
                                    "case_id",
                                    "family_id",
                                    "window_id",
                                    *window.KEY,
                                    "partition",
                                    "noise",
                                    "parent_records",
                                    "case_row_weight",
                                ]
                            },
                            stratum=mode,
                            target=target,
                            score=score,
                            label=label,
                            common_family_eligible=info["family_id"] in eligible[mode],
                            **{k: v for k, v in c.items() if k != "hist"},
                        )
                        case_ranks.append(previous.historical.exported(record))
    return (
        case_strata(rows, lookup, eligible),
        ranks,
        all_bins,
        case_ranks,
        eligibility_audit,
    )


def compare(values, before, keys):
    lookup = {tuple(r[k] for k in keys): r for r in values}
    assert len(lookup) == len(values)
    count = 0
    for old in before:
        after = lookup[tuple(old[k] for k in keys)]
        for field, expected in old.items():
            value = after[field]
            if isinstance(value, (str, bool)):
                assert str(value) == expected, (field, value, expected)
            else:
                previous.close(value, expected)
            count += 1
    return count


def check_previous(ranks, eligibility_audit):
    old = [
        r
        for r in read_rows(RESULTS / "planner-conditioned-rankings/ranking-summary.csv")
        if r["planner_selection"] == "all"
    ]
    assert len(old) == len(ranks)
    count = compare(ranks, old, RANK)
    count += compare(
        eligibility_audit,
        read_rows(RESULTS / COPIES["family-support.csv"]),
        ["family_id", "stratum"],
    )
    latest = {
        tuple(r[k] for k in [*previous.GROUP[1:], "score", "label"]): r
        for r in ranks
        if r["partition"] == "fresh" and r["population"] == "source"
    }
    legacy = [
        r
        for r in read_rows(RESULTS / "fresh-seed-case-influence/ranking-influence.csv")
        if r["scope"] == "all"
    ]
    for old in legacy:
        r = latest[tuple(old[k] for k in [*previous.GROUP[1:], "score", "label"])]
        for name, field in product(
            ["same_case", "between_case"], ["numerator", "denominator"]
        ):
            previous.close(r[name + "_" + field], old[name + "_" + field])
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
        (RESULTS / "planner-conditioned-rankings/sources.json").read_text()
    )
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_planner_conditioned_rankings.py": source["analysis"]["script_sha256"],
    }
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    events_list = read_rows(RESULTS / previous.COPIES["events.csv"])
    events = {r["event_id"]: r for r in events_list}
    assert len(events) == len(events_list) == 3997
    rows, reconstruction = window.load_rows(events)
    families, cases, lookup = window.audit_units(
        read_rows(RESULTS / previous.COPIES["accounting.csv"]), rows
    )
    assert (len(rows), len(families), len(cases)) == (32985, 324, 972)
    data = summaries(rows, families, lookup)
    assert tuple(map(len, data)) == (4860, 2160, 10800, 14130, 1620)
    comparisons = check_previous(data[1], data[4])
    args.output.mkdir(parents=True)
    for name, values in zip(
        ["case-strata", "ranking-summary", "hierarchy-bins", "case-rankings"], data[:4]
    ):
        save_rows(args.output / (name + ".csv"), values)
    for name, original in COPIES.items():
        shutil.copyfile(RESULTS / original, args.output / name)
        assert digest(args.output / name) == PINNED[original]
    audit = dict(
        source_pairs=len(rows),
        source_states=len(rows) // 3,
        source_cases=157,
        source_opportunities=len(
            {
                e
                for r in rows
                if r["view"] == previous.VIEW
                for e in r["source_event_ids"].split("+")
            }
        ),
        accounted_cases=len(cases),
        planned_families=len(families),
        case_stratum_rows=len(data[0]),
        ranking_rows=len(data[1]),
        bin_rows=len(data[2]),
        case_ranking_rows=len(data[3]),
        prior_field_comparisons=comparisons,
        reconstruction_rows=len(reconstruction),
        max_vector_closure_m=max(
            a[t + "_vector_closure_m"] for a in reconstruction for t in previous.TARGETS
        ),
        max_residual_component_difference_m=max(
            a["residual_component_difference_m"] for a in reconstruction
        ),
    )
    sources = dict(
        input_sha256=PINNED,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        schema=dict(
            bins=BINS,
            populations=parent.POPULATIONS,
            family_key=window.FAMILY,
            view=previous.VIEW,
            targets=previous.TARGETS,
            scores=previous.SCORES,
            labels=previous.historical.LABELS,
            strata=previous.STRATA,
            seeds=previous.SEEDS,
            weighting="original 1/n_case before stratum/population selection",
            window="family-scoped exact complete saved diagnostic window",
            pair_counts="descriptive ordered case pairs, not independent observations",
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
