#!/usr/bin/env python3
"""Condition saved source-event rankings on frozen state-time differences."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path
import shutil

from analyze_epoch_preview import digest, read_rows
import analyze_event_conditioned_rankings as prior
from summarize_stopping_preview import save_rows

ROOT, RESULTS = prior.ROOT, prior.RESULTS
previous, parent, window = prior.previous, prior.parent, prior.window
GROUP, RANK, FIELDS = prior.GROUP, prior.RANK, prior.FIELDS
BASE = RESULTS / "event-conditioned-rankings"
PROTOCOL = RESULTS / "event-temporal-rankings/protocol.md"
BASE_SHA = "1e0656c67622cd7f673fd26e75af7bc84774d21491334c7aac94358c94611349"
LIMITS = [20_000, 100_000, 500_000]
BINS = ["le_20ms", "20_to_100ms", "100_to_500ms", "gt_500ms"]
SCOPES = ["le_20ms", "le_100ms", "le_500ms"]


def microseconds(value):
    time = float(value)
    assert math.isfinite(time), "finite saved time required"
    tick = round(time * 1_000_000)
    assert abs(tick / 1_000_000 - time) <= 1e-7, "saved time is off microsecond grid"
    return tick


def time_bin(gap):
    assert isinstance(gap, int) and gap >= 0
    return sum(gap > bound for bound in LIMITS)


def attach_times(rows, lookup):
    attached, audit, seen = [], [], set()
    for r in rows:
        if r["view"] != previous.VIEW:
            attached.append(r)
            continue
        case_id = lookup[previous.identity(r)]["case_id"]
        key = case_id, r["source_id"]
        assert key not in seen, "duplicate saved time mapping"
        seen.add(key)
        time = r["_window"]["time_s"]
        tick = microseconds(time)
        attached.append(dict(r, _time_us=tick))
        audit.append(
            dict(
                case_id=case_id,
                source_id=r["source_id"],
                time_s=time,
                time_us=tick,
                reconstruction_error_s=tick / 1_000_000 - time,
            )
        )
    return attached, audit


def event_counts(rows, target, score, label):
    """Integer pair counts for one full-case/event group, without case weights."""
    assert len({(previous.identity(r), r["_event_id"]) for r in rows}) <= 1
    result = [dict(concordant=0, tied=0, discordant=0, pair_count=0) for _ in BINS]
    positive = [r for r in rows if r[target]["outside"][label]]
    negative = [r for r in rows if not r[target]["outside"][label]]
    for p, n in product(positive, negative):
        b = result[time_bin(abs(p["_time_us"] - n["_time_us"]))]
        delta = int(p[score + "_tick"]) - int(n[score + "_tick"])
        b["concordant" if delta > 0 else "tied" if delta == 0 else "discordant"] += 1
        b["pair_count"] += 1
    assert sum(b["pair_count"] for b in result) == len(positive) * len(negative)
    return result


def count_masses(counts, weight):
    result = {
        name + "_pair_mass": counts[name] * weight * weight
        for name in ["concordant", "tied", "discordant"]
    }
    result["numerator"] = result["concordant_pair_mass"] + result["tied_pair_mass"] / 2
    result["denominator"] = counts["pair_count"] * weight * weight
    return result


def temporal_decomposition(rows, weights, lookup, target, score, label, cache=None):
    assert len(rows) == len(weights) and all(w > 0 for w in weights)
    cache = {} if cache is None else cache
    groups = defaultdict(list)
    for r, w in zip(rows, weights):
        key = previous.identity(r)
        assert lookup[key]["case_row_weight"] == w, "preserve original case weight"
        groups[key, r["_event_id"]].append(r)
    bins = [
        dict(
            bin=name,
            **{f: Fraction() for f in FIELDS},
            pair_count=0,
            _events=set(),
            _cases=set(),
        )
        for name in BINS
    ]
    same_event = {f: Fraction() for f in FIELDS}
    histogram, masses = (
        previous.influence.weighted_histogram,
        previous.influence.pair_masses,
    )
    for key, rr in sorted(groups.items()):
        weight = lookup[key[0]]["case_row_weight"]
        ck = (
            key,
            weight,
            target,
            score,
            label,
            tuple(
                (
                    r["source_id"],
                    r["_time_us"],
                    r[score + "_tick"],
                    r[target]["outside"][label],
                )
                for r in rr
            ),
        )
        if ck not in cache:
            counts = event_counts(rr, target, score, label)
            pieces = [count_masses(c, weight) for c in counts]
            reference = masses(*histogram(rr, [weight] * len(rr), target, score, label))
            for field in FIELDS:
                assert sum((p[field] for p in pieces), Fraction()) == reference[field]
            cache[ck] = counts, pieces, reference
        counts, pieces, reference = cache[ck]
        for field in FIELDS:
            same_event[field] += reference[field]
        for b, c, piece in zip(bins, counts, pieces):
            for field in FIELDS:
                b[field] += piece[field]
            b["pair_count"] += c["pair_count"]
            if c["pair_count"]:
                b["_events"].add(key)
                b["_cases"].add(key[0])
    for field in FIELDS:
        assert sum((b[field] for b in bins), Fraction()) == same_event[field]
    full = masses(*histogram(rows, weights, target, score, label))

    def finish(pieces):
        result = {f: sum((b[f] for b in pieces), Fraction()) for f in FIELDS}
        den = result["denominator"]
        result.update(
            auc=result["numerator"] / den if den else None,
            pair_count=sum(b["pair_count"] for b in pieces),
            events_with_pairs=len(set().union(*(b["_events"] for b in pieces))),
            cases_with_pairs=len(set().union(*(b["_cases"] for b in pieces))),
            full_pair_share=den / full["denominator"] if full["denominator"] else None,
            event_pair_share=(
                den / same_event["denominator"] if same_event["denominator"] else None
            ),
            numerator_full_contribution=(
                result["numerator"] / full["denominator"]
                if full["denominator"]
                else None
            ),
        )
        return result

    extra = {"same_event_pair_count": sum(b["pair_count"] for b in bins)}
    for i, scope in enumerate(SCOPES, 1):
        extra.update({scope + "_" + f: value for f, value in finish(bins[:i]).items()})
    return extra, [dict(bin=b["bin"], **finish([b])) for b in bins], same_event


def event_time_support(rows, lookup, eligible):
    groups = defaultdict(list)
    for r in rows:
        if r["view"] == previous.VIEW:
            groups[previous.identity(r), r["_event_id"]].append(r)
    support = []
    for (key, eid), parent_rows in sorted(groups.items()):
        info, weight = lookup[key], lookup[key]["case_row_weight"]
        for mode in previous.STRATA:
            rr = [r for r in parent_rows if mode == "all" or r["stratum"] == mode]
            if not rr:
                continue
            for target, label in product(previous.TARGETS, previous.historical.LABELS):
                counts = event_counts(rr, target, "contrast", label)
                outside = sum(r[target]["outside"][label] for r in rr)
                item = dict(
                    **{k: info[k] for k in prior.CASE_META},
                    event_id=eid,
                    stratum=mode,
                    target=target,
                    label=label,
                    common_family_eligible=info["family_id"] in eligible[mode],
                    records=len(rr),
                    outside_rows=outside,
                    inside_rows=len(rr) - outside,
                    outside_case_mass=outside * weight,
                    inside_case_mass=(len(rr) - outside) * weight,
                    min_time_us=min(r["_time_us"] for r in rr),
                    max_time_us=max(r["_time_us"] for r in rr),
                )
                for name, c in zip(BINS, counts):
                    item[name + "_pair_count"] = c["pair_count"]
                    item[name + "_denominator"] = c["pair_count"] * weight * weight
                support.append(previous.historical.exported(item))
    return support


def summaries(rows, lookup, ranks, eligible):
    by_group = defaultdict(list)
    for r in rows:
        if r["view"] == previous.VIEW:
            by_group[r["partition"], r["noise"], r["variant"]].append(r)
    expanded, bins, cache = [], [], {}
    for reference in ranks:
        mode, population = reference["stratum"], reference["population"]
        rr = [
            r
            for r in by_group[
                reference["partition"], reference["noise"], reference["variant"]
            ]
            if (mode == "all" or r["stratum"] == mode)
            and (
                population == "source"
                or lookup[previous.identity(r)]["family_id"] in eligible[mode]
            )
        ]
        ww = [lookup[previous.identity(r)]["case_row_weight"] for r in rr]
        extra, pieces, total = temporal_decomposition(
            rr,
            ww,
            lookup,
            reference["target"],
            reference["score"],
            reference["label"],
            cache,
        )
        for field in ["numerator", "denominator"]:
            previous.close(float(total[field]), reference["same_event_" + field])
        expanded.append(previous.historical.exported(dict(**reference, **extra)))
        for b in pieces:
            bins.append(
                previous.historical.exported(
                    dict(**{k: reference[k] for k in RANK}, **b)
                )
            )
    return expanded, bins, event_time_support(rows, lookup, eligible)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    assert digest(BASE / "sources.json") == BASE_SHA
    source = json.loads((BASE / "sources.json").read_text())
    pinned = {
        **prior.PINNED,
        "event-conditioned-rankings/sources.json": BASE_SHA,
        **{
            "event-conditioned-rankings/" + k: v
            for k, v in source["output_sha256"].items()
        },
    }
    for name, sha in pinned.items():
        assert digest(RESULTS / name) == sha, name
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_event_conditioned_rankings.py": source["analysis"]["script_sha256"],
    }
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    census = read_rows(BASE / "events.csv")
    events = {r["event_id"]: r for r in census}
    assert len(events) == len(census) == 3997
    rows, reconstruction = window.load_rows(events)
    families, cases, lookup = window.audit_units(
        read_rows(BASE / "accounting.csv"), rows
    )
    rows, mapping = prior.attach_events(rows, lookup, events)
    rows, times = attach_times(rows, lookup)
    data = prior.summaries(rows, families, lookup, events)
    comparisons = 0
    for name, actual, keys in [
        ("source-event-map", mapping, ["case_id", "source_id"]),
        ("event-coverage", data[0], ["event_id"]),
        ("event-support", data[1], ["event_id", "stratum"]),
        (
            "event-rankings",
            data[2],
            ["event_id", "stratum", "target", "score", "label"],
        ),
        ("ranking-summary", data[3], RANK),
        ("event-pair-bins", data[4], [*RANK, "bin"]),
        ("family-support", data[5], ["family_id", "stratum"]),
        ("case-strata", data[6], ["case_id", "stratum"]),
    ]:
        before = read_rows(BASE / (name + ".csv"))
        assert len(actual) == len(before)
        comparisons += prior.prior.compare(actual, before, keys)
    _, eligible = parent.family_support(families, rows, lookup)
    ranks, bins, support = summaries(rows, lookup, data[3], eligible)
    assert (len(rows), len(times), len(cases), len(families)) == (
        32985,
        10995,
        972,
        324,
    )
    assert (len(ranks), len(bins), len(support)) == (2160, 8640, 3300)
    args.output.mkdir(parents=True)
    for name in source["output_sha256"]:
        if name != "ranking-summary.csv":
            shutil.copyfile(BASE / name, args.output / name)
            assert digest(args.output / name) == source["output_sha256"][name]
    for name, values in [
        ("ranking-summary", ranks),
        ("temporal-pair-bins", bins),
        ("event-time-support", support),
        ("source-time-audit", times),
    ]:
        save_rows(args.output / (name + ".csv"), values)
    audit = dict(
        source_pairs=len(rows),
        source_states=len(times),
        source_cases=157,
        source_events=275,
        census_events=len(events),
        accounted_cases=len(cases),
        planned_families=len(families),
        ranking_rows=len(ranks),
        bin_rows=len(bins),
        event_time_support_rows=len(support),
        time_audit_rows=len(times),
        prior_field_comparisons=comparisons,
        reconstruction_rows=len(reconstruction),
        max_abs_time_reconstruction_error_s=max(
            abs(r["reconstruction_error_s"]) for r in times
        ),
    )
    result = dict(
        input_sha256=pinned,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        schema=dict(
            bins=BINS,
            cumulative_scopes=SCOPES,
            upper_bounds_us=LIMITS,
            time_rounding="nearest integer microsecond, ties to even; error <=1e-7 s",
            time_difference="absolute saved state-time gap within full case and saved event_id",
            weighting="original 1/n_case before stratum/population selection",
            event_key=[*window.KEY, "event_id"],
            populations=parent.POPULATIONS,
            view=previous.VIEW,
            targets=previous.TARGETS,
            scores=previous.SCORES,
            labels=previous.historical.LABELS,
            strata=previous.STRATA,
            seeds=previous.SEEDS,
            retrospective=True,
        ),
        inherited_validation=source["inherited_validation"],
        audit=audit,
        output_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
