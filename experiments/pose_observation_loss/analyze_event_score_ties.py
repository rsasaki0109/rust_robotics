#!/usr/bin/env python3
"""Distinguish equal saved scores from ties created by the existing tick grid."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
import analyze_event_temporal_rankings as prior
from summarize_stopping_preview import save_rows

ROOT, RESULTS = prior.ROOT, prior.RESULTS
previous, parent, window = prior.previous, prior.parent, prior.window
GROUP, RANK, FIELDS = prior.GROUP, prior.RANK, prior.FIELDS
BASE = RESULTS / "event-temporal-rankings"
BASE_SHA = "19d64587f8fb0c5893ecf21312ed38a7f9261ab3898c9c6fc7fd1c565d0f6e80"
PROTOCOL = RESULTS / "event-score-ties/protocol.md"
BINS = prior.BINS
SCOPES = ["same_event", "le_500ms", "le_100ms", "le_20ms"]
CLASSES = [
    "tick_concordant",
    "tick_discordant",
    "tick_tied_native_concordant",
    "tick_tied_native_equal",
    "tick_tied_native_discordant",
]


def attach_scores(rows, lookup, historical, latest):
    sources = {}
    for origin, source in [("historical", historical), ("latest", latest)]:
        selected = [
            r
            for r in source
            if r["view"] == previous.VIEW
            and (origin == "latest" or r["source_family"] == "simultaneous")
        ]
        indexed = {(previous.identity(r), r["source_id"]): r for r in selected}
        assert len(indexed) == len(selected), "duplicate saved score key"
        sources[origin] = indexed
    attached, audit, consumed = [], [], set()
    for r in rows:
        if r["view"] != previous.VIEW:
            attached.append(r)
            continue
        origin = "latest" if r["partition"] == "fresh" else "historical"
        key = previous.identity(r), r["source_id"]
        assert (origin, key) not in consumed, "duplicate score mapping"
        consumed.add((origin, key))
        assert key in sources[origin], "missing saved score"
        s = sources[origin][key]
        for field in [
            *window.KEY,
            "source_id",
            "partition",
            "noise",
            "view",
            "source_event_ids",
        ]:
            assert str(r[field]) == s[field], ("score metadata", field)
        for field in [
            "a_id",
            "b_id",
            "state_id",
            "a_estimator",
            "b_estimator",
            "anchor_mode",
            "stratum",
        ]:
            if field in r and field in s:
                assert str(r[field]) == s[field], ("score metadata", field)
        item = dict(
            case_id=lookup[key[0]]["case_id"],
            source_id=r["source_id"],
            event_id=r["_event_id"],
            score_source=origin,
        )
        saved = {}
        for score in previous.SCORES:
            raw, tick = s[score + "_score_m"], int(s[score + "_tick"])
            value = float(raw)
            assert math.isfinite(value), "finite saved score required"
            assert tick == int(r[score + "_tick"]), "saved tick join"
            assert round(value / 1e-8) == tick, "saved score/tick rounding"
            saved[score] = value
            item.update(
                {
                    score + "_score_m": raw,
                    score + "_score_hex": value.hex(),
                    score + "_tick": tick,
                }
            )
        attached.append(dict(r, _saved_scores=saved))
        audit.append(item)
    assert len(consumed) == sum(
        len(v) for v in sources.values()
    ), "unused RP saved score"
    return attached, audit


def order_class(ptick, ntick, positive, negative):
    assert math.isfinite(positive) and math.isfinite(negative)
    if ptick > ntick:
        assert positive > negative, "tick/native order must be monotone"
        return 0
    if ptick < ntick:
        assert positive < negative, "tick/native order must be monotone"
        return 1
    return 2 if positive > negative else 4 if positive < negative else 3


def event_counts(rows, target, score, label):
    assert len({(previous.identity(r), r["_event_id"]) for r in rows}) <= 1
    counts = [[0] * len(CLASSES) for _ in BINS]
    positive = [r for r in rows if r[target]["outside"][label]]
    negative = [r for r in rows if not r[target]["outside"][label]]
    for p, n in product(positive, negative):
        bin_id = prior.time_bin(abs(p["_time_us"] - n["_time_us"]))
        category = order_class(
            p[score + "_tick"],
            n[score + "_tick"],
            p["_saved_scores"][score],
            n["_saved_scores"][score],
        )
        counts[bin_id][category] += 1
    assert sum(map(sum, counts)) == len(positive) * len(negative)
    return counts


def decomposition(rows, weights, lookup, target, score, label, cache=None):
    assert len(rows) == len(weights) and all(w > 0 for w in weights)
    groups, cache = defaultdict(list), {} if cache is None else cache
    for r, w in zip(rows, weights):
        key = previous.identity(r)
        assert lookup[key]["case_row_weight"] == w, "preserve original case weight"
        groups[key, r["_event_id"]].append(r)
    bins = [
        dict(
            mass=[Fraction() for _ in CLASSES],
            counts=[0] * len(CLASSES),
            events=set(),
            cases=set(),
            tick_events=set(),
            tick_cases=set(),
            native_events=set(),
            native_cases=set(),
        )
        for _ in BINS
    ]
    for key, rr in sorted(groups.items()):
        w = lookup[key[0]]["case_row_weight"]
        ck = (
            key,
            w,
            target,
            score,
            label,
            tuple(
                (
                    r["source_id"],
                    r["_time_us"],
                    r[score + "_tick"],
                    r["_saved_scores"][score],
                    r[target]["outside"][label],
                )
                for r in rr
            ),
        )
        if ck not in cache:
            counts = event_counts(rr, target, score, label)
            mass = [[c * w * w for c in b] for b in counts]
            reference = previous.influence.pair_masses(
                *previous.influence.weighted_histogram(
                    rr, [w] * len(rr), target, score, label
                )
            )
            assert (
                sum((b[0] for b in mass), Fraction())
                == reference["concordant_pair_mass"]
            )
            assert (
                sum((sum(b[2:], Fraction()) for b in mass), Fraction())
                == reference["tied_pair_mass"]
            )
            assert (
                sum((b[1] for b in mass), Fraction())
                == reference["discordant_pair_mass"]
            )
            cache[ck] = counts, mass
        counts, mass = cache[ck]
        for b, c, m in zip(bins, counts, mass):
            b["mass"] = [x + y for x, y in zip(b["mass"], m)]
            b["counts"] = [x + y for x, y in zip(b["counts"], c)]
            for prefix, exists in [
                ("", sum(c)),
                ("tick_", c[0] + c[1]),
                ("native_", c[0] + c[1] + c[2] + c[4]),
            ]:
                if exists:
                    b[prefix + "events"].add(key)
                    b[prefix + "cases"].add(key[0])
    full = previous.influence.pair_masses(
        *previous.influence.weighted_histogram(rows, weights, target, score, label)
    )
    event_den = sum((sum(b["mass"], Fraction()) for b in bins), Fraction())

    def finish(selected):
        mass = [
            sum((b["mass"][i] for b in selected), Fraction())
            for i in range(len(CLASSES))
        ]
        counts = [sum(b["counts"][i] for b in selected) for i in range(len(CLASSES))]
        con, dis, resolved_con, native_tied, resolved_dis = mass
        tied = resolved_con + native_tied + resolved_dis
        den = con + tied + dis
        num = con + tied / 2
        native_con, native_dis = con + resolved_con, dis + resolved_dis
        native_num = native_con + native_tied / 2
        auc, native_auc = (num / den, native_num / den) if den else (None, None)
        delta = (native_auc - auc) if den else None
        assert delta == ((resolved_con - resolved_dis) / (2 * den) if den else None)
        result = dict(
            concordant_pair_mass=con,
            tied_pair_mass=tied,
            discordant_pair_mass=dis,
            numerator=num,
            denominator=den,
            auc=auc,
            pair_count=sum(counts),
            events_with_pairs=len(set().union(*(b["events"] for b in selected))),
            cases_with_pairs=len(set().union(*(b["cases"] for b in selected))),
            full_pair_share=den / full["denominator"] if full["denominator"] else None,
            event_pair_share=den / event_den if event_den else None,
            numerator_full_contribution=(
                num / full["denominator"] if full["denominator"] else None
            ),
            native_concordant_pair_mass=native_con,
            native_tied_pair_mass=native_tied,
            native_discordant_pair_mass=native_dis,
            native_numerator=native_num,
            native_auc=native_auc,
            native_auc_delta=delta,
            tick_tied_native_concordant_mass=resolved_con,
            tick_tied_native_discordant_mass=resolved_dis,
            tick_tied_share=tied / den if den else None,
            native_tied_share=native_tied / den if den else None,
            resolved_tick_tie_share=(
                (resolved_con + resolved_dis) / den if den else None
            ),
            tick_unequal_concordance=con / (con + dis) if con + dis else None,
            native_unequal_concordance=(
                native_con / (native_con + native_dis)
                if native_con + native_dis
                else None
            ),
            tick_unequal_pair_count=counts[0] + counts[1],
            native_unequal_pair_count=counts[0] + counts[1] + counts[2] + counts[4],
        )
        result.update(
            {name + "_pair_count": count for name, count in zip(CLASSES, counts)}
        )
        for prefix in ["tick", "native"]:
            for unit in ["events", "cases"]:
                result[prefix + "_unequal_" + unit] = len(
                    set().union(*(b[prefix + "_" + unit] for b in selected))
                )
        return result

    scopes = [finish(bins), finish(bins[:3]), finish(bins[:2]), finish(bins[:1])]
    pieces = [finish([b]) for b in bins]
    for field in FIELDS:
        assert sum((b[field] for b in pieces), Fraction()) == scopes[0][field]
    return scopes, pieces


def summaries(rows, lookup, rankings, old_bins, eligible):
    parents, cache = defaultdict(list), {}
    for r in rows:
        if r["view"] == previous.VIEW:
            parents[r["partition"], r["noise"], r["variant"]].append(r)
    old_index = {tuple(r[k] for k in [*RANK, "bin"]): r for r in old_bins}
    assert len(old_index) == len(old_bins)
    summary, bins, comparisons = [], [], 0
    for reference in rankings:
        mode, population = reference["stratum"], reference["population"]
        rr = [
            r
            for r in parents[
                reference["partition"], reference["noise"], reference["variant"]
            ]
            if (mode == "all" or r["stratum"] == mode)
            and (
                population == "source"
                or lookup[previous.identity(r)]["family_id"] in eligible[mode]
            )
        ]
        ww = [lookup[previous.identity(r)]["case_row_weight"] for r in rr]
        scopes, pieces = decomposition(
            rr,
            ww,
            lookup,
            reference["target"],
            reference["score"],
            reference["label"],
            cache,
        )
        key = tuple(reference[k] for k in RANK)
        metadata = {k: reference[k] for k in RANK}
        for name, values in zip(SCOPES, scopes):
            if name == "same_event":
                expected = {
                    f: reference[name + "_" + f]
                    for f in ["numerator", "denominator", "auc"]
                }
                expected.update(
                    pair_count=reference["same_event_pair_count"],
                    events_with_pairs=reference["events_with_both_classes"],
                    cases_with_pairs=reference["cases_with_event_support"],
                    full_pair_share=reference["same_event_pair_share"],
                )
            else:
                expected = {
                    f: reference[name + "_" + f]
                    for f in [
                        *FIELDS,
                        "auc",
                        "pair_count",
                        "events_with_pairs",
                        "cases_with_pairs",
                        "full_pair_share",
                        "event_pair_share",
                        "numerator_full_contribution",
                    ]
                }
            for field, value in expected.items():
                previous.close(
                    float(values[field]) if values[field] is not None else "", value
                )
                comparisons += 1
            summary.append(
                previous.historical.exported(dict(**metadata, scope=name, **values))
            )
        for name, values in zip(BINS, pieces):
            before = old_index[*key, name]
            for field in [
                *FIELDS,
                "auc",
                "pair_count",
                "events_with_pairs",
                "cases_with_pairs",
                "full_pair_share",
                "event_pair_share",
                "numerator_full_contribution",
            ]:
                previous.close(
                    float(values[field]) if values[field] is not None else "",
                    before[field],
                )
                comparisons += 1
            bins.append(
                previous.historical.exported(dict(**metadata, bin=name, **values))
            )
    return summary, bins, comparisons


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    assert digest(BASE / "sources.json") == BASE_SHA
    baseline = json.loads((BASE / "sources.json").read_text())
    pinned = {
        **baseline["input_sha256"],
        "event-temporal-rankings/sources.json": BASE_SHA,
        **{
            "event-temporal-rankings/" + k: v
            for k, v in baseline["output_sha256"].items()
        },
    }
    for name, sha in pinned.items():
        assert digest(RESULTS / name) == sha, name
    helpers = {
        **baseline["analysis"]["helpers_sha256"],
        "analyze_event_temporal_rankings.py": baseline["analysis"]["script_sha256"],
    }
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    events = {r["event_id"]: r for r in read_rows(BASE / "events.csv")}
    assert len(events) == 3997
    rows, reconstruction = window.load_rows(events)
    families, cases, lookup = window.audit_units(
        read_rows(BASE / "accounting.csv"), rows
    )
    rows, mapping = prior.prior.attach_events(rows, lookup, events)
    rows, times = prior.attach_times(rows, lookup)
    historical = read_rows(RESULTS / "contrast-risk-transfer/scores.csv")
    latest = read_rows(RESULTS / "fresh-seed-cutoff-transfer/scores.csv")
    rows, scores = attach_scores(rows, lookup, historical, latest)
    _, eligible = parent.family_support(families, rows, lookup)
    comparisons = 0
    for actual, name, keys in [
        (cases, "cases", ["case_id"]),
        (families, "families", ["family_id"]),
        (mapping, "source-event-map", ["case_id", "source_id"]),
        (times, "source-time-audit", ["case_id", "source_id"]),
    ]:
        before = read_rows(BASE / (name + ".csv"))
        assert len(actual) == len(before)
        comparisons += prior.prior.prior.compare(actual, before, keys)
    summary, bins, checks = summaries(
        rows,
        lookup,
        read_rows(BASE / "ranking-summary.csv"),
        read_rows(BASE / "temporal-pair-bins.csv"),
        eligible,
    )
    assert (len(rows), len(scores), len(summary), len(bins)) == (
        32985,
        10995,
        8640,
        8640,
    )
    args.output.mkdir(parents=True)
    for name, data in [
        ("score-audit", scores),
        ("precision-summary", summary),
        ("precision-bins", bins),
    ]:
        save_rows(args.output / (name + ".csv"), data)
    audit = dict(
        source_pairs=len(rows),
        source_states=len(scores),
        represented_cases=157,
        accounted_cases=len(cases),
        planned_families=len(families),
        census_events=len(events),
        scope_rows=len(summary),
        bin_rows=len(bins),
        saved_score_values=len(scores) * 3,
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
            score_classes=CLASSES,
            bins=BINS,
            scopes=SCOPES,
            saved_score_order="strict finite binary64 saved score_m, no extra equality tolerance",
            original_score_order="unchanged 1e-8 m integer ticks",
            weighting="original 1/n_case before stratum/population selection",
            conditional_concordance="C/(C+D) on unequal scores; no case renormalization",
            retrospective=True,
        ),
        inherited_validation=baseline["inherited_validation"],
        audit=audit,
        output_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
