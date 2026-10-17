#!/usr/bin/env python3
"""Separate same-event, other-event/same-case and other-case ranking pairs."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path
import shutil

from analyze_epoch_preview import digest, read_rows
import analyze_case_window_ranks as prior
from summarize_stopping_preview import save_rows

parent, previous, window = prior.parent, prior.previous, prior.window
ROOT, RESULTS = prior.ROOT, prior.RESULTS
PROTOCOL = RESULTS / "event-conditioned-rankings/protocol.md"
GROUP, RANK, FIELDS = prior.GROUP, prior.RANK, prior.FIELDS
BINS = ["same_event", "other_event_same_case", "other_case"]
PINNED = {
    **prior.PINNED,
    "case-window-rank-decomposition/sources.json": "0797d936f0c97f62cf986871179699467f4e3ce884d69a309d41ec12e470fb85",
    "case-window-rank-decomposition/ranking-summary.csv": "3752b7b38a8cea3c8ce0edf7b5b7f4665fbc88d03f928b0b3fe73af7f4d6d351",
    "case-window-rank-decomposition/case-strata.csv": "e486370bf6af70dc247bcd2335409d71f166518556d6bdf2e2525745f3a6e994",
    "case-window-rank-decomposition/hierarchy-bins.csv": "db09f9adb51d97a6c928c16d489bf0b5720cb55362bb619bed58c92c6709690c",
}
COPIES = {
    **prior.COPIES,
    "case-strata.csv": "case-window-rank-decomposition/case-strata.csv",
}
EVENT_META = [
    "event_id",
    "source_event_id",
    "event_kind",
    "event_time_s",
    "truth_time_s",
    "scheduled",
    "received",
    "precision_activation",
]
CASE_META = [
    "case_id",
    "family_id",
    *window.KEY,
    "partition",
    "accounting_partition",
    "noise",
]


def event_id(row, events):
    ids = row["source_event_ids"].split("+")
    assert len(ids) == 1 and ids[0], "exactly one saved source-event tag required"
    assert ids[0] in events, "unknown source-event tag"
    event = events[ids[0]]
    assert event["event_id"] == ids[0]
    assert previous.identity(row) == previous.identity(event), "source-event case join"
    partition = "new" if event["partition"] == "existing_new" else event["partition"]
    assert (row["partition"], row["noise"]) == (
        partition,
        event["noise"],
    ), "source-event reporting group"
    return ids[0]


def attach_events(rows, lookup, events):
    attached, mapping = [], []
    seen = set()
    for row in rows:
        if row["view"] != previous.VIEW:
            attached.append(row)
            continue
        eid = event_id(row, events)
        info, event = lookup[previous.identity(row)], events[eid]
        key = info["case_id"], row["source_id"]
        assert key not in seen, "duplicate RP source mapping"
        seen.add(key)
        time = row["_window"]["time_s"]
        assert all(
            math.isfinite(float(v))
            for v in [time, event["event_time_s"], event["truth_time_s"]]
        )
        attached.append(dict(row, _event_id=eid))
        mapping.append(
            previous.historical.exported(
                dict(
                    **{k: info[k] for k in CASE_META},
                    source_id=row["source_id"],
                    event_id=eid,
                    stratum=row["stratum"],
                    time_s=time,
                    truth_time_s=row["_window"]["truth_time_s"],
                    event_time_s=float(event["event_time_s"]),
                    event_truth_time_s=float(event["truth_time_s"]),
                    event_kind=event["event_kind"],
                    event_age_s=time - float(event["event_time_s"]),
                    case_row_weight=info["case_row_weight"],
                )
            )
        )
    return attached, mapping


def event_tables(rows, lookup, events, eligible):
    by_event = defaultdict(list)
    for r in rows:
        if r["view"] == previous.VIEW:
            by_event[r["_event_id"]].append(r)
    coverage, support, ranks = [], [], []
    for eid in sorted(events, key=int):
        event = events[eid]
        info = lookup[previous.identity(event)]
        rr = by_event[eid]
        weight = info["case_row_weight"]
        metadata = dict(
            **{k: info[k] for k in CASE_META},
            **{k: event[k] for k in EVENT_META},
            event_partition=event["partition"],
            parent_case_records=info["parent_records"],
            case_row_weight=weight,
        )
        times = [r["_window"]["time_s"] for r in rr]
        coverage.append(
            previous.historical.exported(
                dict(
                    **metadata,
                    records=len(rr),
                    strata="+".join(sorted({r["stratum"] for r in rr})),
                    min_time_s=min(times) if times else None,
                    max_time_s=max(times) if times else None,
                    min_event_age_s=(
                        min(times) - float(event["event_time_s"]) if times else None
                    ),
                    max_event_age_s=(
                        max(times) - float(event["event_time_s"]) if times else None
                    ),
                )
            )
        )
        if not rr:
            continue
        for mode in previous.STRATA:
            ss = [r for r in rr if mode == "all" or r["stratum"] == mode]
            common = dict(
                **metadata,
                stratum=mode,
                parent_event_records=len(rr),
                records=len(ss),
                case_mass=len(ss) * weight,
                common_family_eligible=info["family_id"] in eligible[mode],
            )
            record = dict(common)
            for target, label in product(previous.TARGETS, previous.historical.LABELS):
                outside = sum(r[target]["outside"][label] for r in ss)
                prefix = target + "_" + label
                record[prefix + "_outside_rows"] = outside
                record[prefix + "_inside_rows"] = len(ss) - outside
                record[prefix + "_outside_case_mass"] = outside * weight
                record[prefix + "_inside_case_mass"] = (len(ss) - outside) * weight
            support.append(previous.historical.exported(record))
            if not ss:
                continue
            for target, score, label in product(
                previous.TARGETS, previous.SCORES, previous.historical.LABELS
            ):
                hist = previous.influence.weighted_histogram(
                    ss, [weight] * len(ss), target, score, label
                )
                outside = sum(r[target]["outside"][label] for r in ss)
                ranks.append(
                    previous.historical.exported(
                        dict(
                            **common,
                            target=target,
                            score=score,
                            label=label,
                            outside_rows=outside,
                            inside_rows=len(ss) - outside,
                            outside_case_mass=sum(hist[1].values(), Fraction()),
                            inside_case_mass=sum(hist[0].values(), Fraction()),
                            **previous.influence.pair_masses(*hist),
                        )
                    )
                )
    return coverage, support, ranks


def event_decomposition(rows, weights, lookup, target, score, label):
    assert len(rows) == len(weights) and all(w > 0 for w in weights)
    selected, by_case = defaultdict(list), defaultdict(list)
    for r, w in zip(rows, weights):
        key = previous.identity(r)
        assert lookup[key]["case_row_weight"] == w, "preserve original case weight"
        selected[key, r["_event_id"]].append((r, w))
    histogram, pairs = (
        previous.influence.weighted_histogram,
        previous.influence.pair_masses,
    )
    hist = {}
    for key, subset in sorted(selected.items()):
        hist[key] = histogram(
            [r for r, w in subset], [w for r, w in subset], target, score, label
        )
        by_case[key[0]].append(key)
    bins = {name: dict(bin=name, **{f: Fraction() for f in FIELDS}) for name in BINS}
    same_case = {f: Fraction() for f in FIELDS}
    for case_key, keys in by_case.items():
        subset = [(r, w) for key in keys for r, w in selected[key]]
        result = pairs(
            *histogram(
                [r for r, w in subset], [w for r, w in subset], target, score, label
            )
        )
        for field in FIELDS:
            same_case[field] += result[field]
        for pos, neg in product(keys, repeat=2):
            mass = pairs(hist[neg][0], hist[pos][1])
            for field in FIELDS:
                bins["same_event" if pos == neg else "other_event_same_case"][
                    field
                ] += mass[field]
    full = pairs(*histogram(rows, weights, target, score, label))
    for field in FIELDS:
        assert (
            bins["same_event"][field] + bins["other_event_same_case"][field]
            == same_case[field]
        )
        bins["other_case"][field] = full[field] - same_case[field]
        assert bins["other_case"][field] >= 0
        assert sum((b[field] for b in bins.values()), Fraction()) == full[field]
    extra = dict(
        events=len(hist),
        events_with_both_classes=sum(bool(h[0]) and bool(h[1]) for h in hist.values()),
        cases_with_event_support=len(
            {key[0] for key, h in hist.items() if h[0] and h[1]}
        ),
    )
    for name, b in bins.items():
        b["auc"] = b["numerator"] / b["denominator"] if b["denominator"] else None
        b["denominator_share"] = (
            b["denominator"] / full["denominator"] if full["denominator"] else None
        )
        b["numerator_contribution"] = (
            b["numerator"] / full["denominator"] if full["denominator"] else None
        )
        b["within_case_share"] = (
            b["denominator"] / same_case["denominator"]
            if name != "other_case" and same_case["denominator"]
            else None
        )
        if name != "other_case":
            for field in ["numerator", "denominator", "auc"]:
                extra[name + "_" + field] = b[field]
            extra[name + "_pair_share"] = b["denominator_share"]
            extra[name + "_within_case_share"] = b["within_case_share"]
    return extra, list(bins.values()), same_case


def summaries(rows, families, lookup, events):
    family_audit, eligible = parent.family_support(families, rows, lookup)
    coverage, support, event_ranks = event_tables(rows, lookup, events, eligible)
    parents = defaultdict(list)
    for r in rows:
        if r["view"] == previous.VIEW:
            parents[r["partition"], r["noise"], r["variant"]].append(r)
    ranks, bins = [], []
    for partition, noise, arm in product(
        previous.SEEDS, previous.NOISES, previous.ARMS
    ):
        original = parents[partition, noise, arm]
        parent_mass = Fraction(len({previous.identity(r) for r in original}))
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
                old_extra, old_bins, _, cross = prior.decompose(
                    rr, ww, lookup, target, score, label
                )
                reference, _ = parent.planner_decomposition(
                    rr, ww, target, score, label, previous.SEEDS[partition]
                )
                for field, value in cross.items():
                    assert reference[field] == value
                extra, pieces, same_case = event_decomposition(
                    rr, ww, lookup, target, score, label
                )
                for field in FIELDS:
                    assert same_case[field] == old_bins[0][field]
                ranks.append(
                    previous.historical.exported(
                        dict(
                            **common,
                            score=score,
                            label=label,
                            **reference,
                            **old_extra,
                            **extra,
                        )
                    )
                )
                for piece in pieces:
                    bins.append(
                        previous.historical.exported(
                            dict(
                                **{k: common[k] for k in GROUP},
                                score=score,
                                label=label,
                                **piece,
                            )
                        )
                    )
    return (
        coverage,
        support,
        event_ranks,
        ranks,
        bins,
        family_audit,
        prior.case_strata(rows, lookup, eligible),
    )


def check_previous(data):
    ranks, bins, family_audit, case_audit = data[3:]
    count = prior.compare(
        ranks,
        read_rows(RESULTS / "case-window-rank-decomposition/ranking-summary.csv"),
        RANK,
    )
    count += prior.compare(
        family_audit,
        read_rows(RESULTS / COPIES["family-support.csv"]),
        ["family_id", "stratum"],
    )
    count += prior.compare(
        case_audit,
        read_rows(RESULTS / COPIES["case-strata.csv"]),
        ["case_id", "stratum"],
    )
    old = defaultdict(list)
    for r in read_rows(RESULTS / "case-window-rank-decomposition/hierarchy-bins.csv"):
        old[tuple(r[k] for k in RANK)].append(r)
    new = defaultdict(list)
    for r in bins:
        new[tuple(r[k] for k in RANK)].append(r)
    assert set(old) == set(new)
    for key, values in new.items():
        for same in [True, False]:
            for field in FIELDS:
                actual = math.fsum(
                    float(r[field])
                    for r in values
                    if (r["bin"] != "other_case") == same
                )
                expected = math.fsum(
                    float(r[field])
                    for r in old[key]
                    if (r["bin"] == "same_case") == same
                )
                previous.close(actual, expected)
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
        (RESULTS / "case-window-rank-decomposition/sources.json").read_text()
    )
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_case_window_ranks.py": source["analysis"]["script_sha256"],
    }
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    census = read_rows(RESULTS / previous.COPIES["events.csv"])
    events = {r["event_id"]: r for r in census}
    assert len(events) == len(census) == 3997
    rows, reconstruction = window.load_rows(events)
    families, cases, lookup = window.audit_units(
        read_rows(RESULTS / previous.COPIES["accounting.csv"]), rows
    )
    rows, mapping = attach_events(rows, lookup, events)
    assert (len(rows), len(mapping), len(cases), len(families)) == (
        32985,
        10995,
        972,
        324,
    )
    data = summaries(rows, families, lookup, events)
    assert tuple(map(len, data[:5])) == (3997, 1375, 9900, 2160, 6480)
    comparisons = check_previous(data)
    args.output.mkdir(parents=True)
    save_rows(args.output / "source-event-map.csv", mapping)
    for name, values in zip(
        [
            "event-coverage",
            "event-support",
            "event-rankings",
            "ranking-summary",
            "event-pair-bins",
        ],
        data,
    ):
        save_rows(args.output / (name + ".csv"), values)
    for name, original in COPIES.items():
        shutil.copyfile(RESULTS / original, args.output / name)
        assert digest(args.output / name) == PINNED[original]
    audit = dict(
        source_pairs=len(rows),
        source_states=len(mapping),
        source_cases=157,
        source_events=sum(r["records"] > 0 for r in data[0]),
        census_events=len(events),
        accounted_cases=len(cases),
        planned_families=len(families),
        event_coverage_rows=len(data[0]),
        event_support_rows=len(data[1]),
        event_ranking_rows=len(data[2]),
        ranking_rows=len(data[3]),
        bin_rows=len(data[4]),
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
            view=previous.VIEW,
            targets=previous.TARGETS,
            scores=previous.SCORES,
            labels=previous.historical.LABELS,
            strata=previous.STRATA,
            seeds=previous.SEEDS,
            weighting="original 1/n_case before stratum/population selection",
            event_key=[*window.KEY, "event_id"],
            event_semantics="single saved diagnostic tag, not complete estimator information ancestry",
            empty_event_strata="explicit in support; no fabricated ranking rows",
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
