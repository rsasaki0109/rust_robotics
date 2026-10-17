#!/usr/bin/env python3
"""Explain class-pair availability using frozen case/event/time support counts."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
BASE = RESULTS / "event-loss-mode-influence"
BASE_SHA = "9ade09e0728e4fd3f6a7a0bca821c05b90917d2ea3adf7963ee8ea8f15d98849"
DATA = RESULTS / "event-temporal-rankings"
PROTOCOL = RESULTS / "event-support-attrition/protocol.md"
KEY = ("cohort", "variant", "mode", "seed", "scenario", "planner")
META = ("case_id", "family_id", *KEY, "partition", "accounting_partition", "noise")
GROUP = ("partition", "noise", "variant", "stratum", "population", "target", "label")
STRATA = (
    "all",
    "recovery_single",
    "filter_single",
    "fusion_three",
    "recovery_fallback_single",
)
TARGETS = ("midpoint", "residual")
LABELS = ("observed", "modeled", "unmodeled")
MODES = ("none", "random", "blackout")
POPULATIONS = ("source", "common_family")
SCOPES = ("same_event", "le_500ms", "le_100ms", "le_20ms")
TIME_BINS = ("le_20ms", "20_to_100ms", "100_to_500ms", "gt_500ms")
REASONS = (
    "no_parent_rp",
    "no_stratum_rp",
    "family_ineligible",
    "no_outside",
    "no_inside",
    "event_separated",
    "gap_excluded",
    "supported",
)
SEEDS = {
    "known": ("269", "270", "271"),
    "new": ("274", "275", "276"),
    "fresh": ("277", "278", "279"),
}
SCENARIOS = ("dense_50x50", "maze_50x50", "open_50x50")
PLANNERS = ("A*", "JPS", "Theta*")


def integer(value):
    assert not isinstance(value, bool)
    n = int(value)
    assert n >= 0 and (
        isinstance(value, int) or str(n) == value
    ), "nonnegative canonical integer required"
    return n


def boolean(value):
    assert value in ("True", "False"), "canonical saved boolean required"
    return value == "True"


def close(actual, expected):
    assert actual != "" and math.isclose(
        float(actual), float(expected), rel_tol=1e-10, abs_tol=2e-12
    ), (actual, expected)


def index(rows, keys):
    result = {tuple(r[k] for k in keys): r for r in rows}
    assert len(result) == len(rows), "duplicate input identity"
    return result


def reason(
    parent,
    stratum_records,
    eligible,
    population,
    outside,
    inside,
    event_pairs,
    scope_pairs,
):
    assert population in POPULATIONS
    if not parent:
        return "no_parent_rp"
    if not stratum_records:
        return "no_stratum_rp"
    if population == "common_family" and not eligible:
        return "family_ineligible"
    if not outside:
        return "no_outside"
    if not inside:
        return "no_inside"
    if not event_pairs:
        return "event_separated"
    return "supported" if scope_pairs else "gap_excluded"


def event_result(raw, parent, population):
    assert parent > 0 and population in POPULATIONS
    assert (
        raw["stratum"] in STRATA and raw["target"] in TARGETS and raw["label"] in LABELS
    )
    count = integer(raw["records"])
    outside, inside = integer(raw["outside_rows"]), integer(raw["inside_rows"])
    assert outside + inside == count <= parent
    bins = [integer(raw[b + "_pair_count"]) for b in TIME_BINS]
    assert sum(bins) == outside * inside, "event class/bin partition does not close"
    for b, n in zip(TIME_BINS, bins):
        close(raw[b + "_denominator"], Fraction(n, parent * parent))
    eligible = boolean(raw["common_family_eligible"])
    active = population == "source" or eligible
    out, ins = (outside, inside) if active else (0, 0)
    counts = [sum(bins), sum(bins[:3]), sum(bins[:2]), bins[0]] if active else [0] * 4
    result = {k: raw[k] for k in META}
    result.update(
        event_id=raw["event_id"],
        stratum=raw["stratum"],
        target=raw["target"],
        label=raw["label"],
        population=population,
        parent_records=parent,
        stratum_records=count,
        common_family_eligible=eligible,
        records=out + ins,
        outside_rows=out,
        inside_rows=ins,
        case_mass=Fraction(out + ins, parent),
    )
    for scope, n in zip(SCOPES, counts):
        result.update(
            {
                scope + "_pair_count": n,
                scope + "_pair_mass": Fraction(n, parent * parent),
                scope
                + "_reason": reason(
                    parent, count, eligible, population, out, ins, counts[0], n
                ),
            }
        )
    return result


def case_result(raw, events, target, label, population):
    assert target in TARGETS and label in LABELS and raw["stratum"] in STRATA
    parent, count = integer(raw["parent_records"]), integer(raw["records"])
    outside, inside = (
        integer(raw[f"{target}_{label}_{x}_rows"]) for x in ("outside", "inside")
    )
    assert outside + inside == count <= parent
    eligible = boolean(raw["common_family_eligible"])
    assert len({r["event_id"] for r in events}) == len(
        events
    ), "duplicate event in case selection"
    for r in events:
        assert all(
            r[k] == raw[k] for k in (*META, "stratum")
        ), "wrong-case/stratum event"
        assert r["target"] == target and r["label"] == label
        assert boolean(r["common_family_eligible"]) == eligible
    assert sum(integer(r["records"]) for r in events) == count
    assert sum(integer(r["outside_rows"]) for r in events) == outside
    assert sum(integer(r["inside_rows"]) for r in events) == inside
    if parent:
        close(raw["case_row_weight"], Fraction(1, parent))
    else:
        assert raw["case_row_weight"] == "" and not events
    close(raw["case_mass"], Fraction(count, parent) if parent else 0)
    active = population == "source" or eligible
    out, ins = (outside, inside) if active else (0, 0)
    er = [event_result(r, parent, population) for r in events]
    result = {k: raw[k] for k in META}
    result.update(
        stratum=raw["stratum"],
        target=target,
        label=label,
        population=population,
        parent_records=parent,
        stratum_records=count,
        common_family_eligible=eligible,
        records=out + ins,
        events=sum(r["records"] > 0 for r in er),
        outside_rows=out,
        inside_rows=ins,
        case_mass=Fraction(out + ins, parent) if parent else Fraction(),
        case_pair_count=out * ins,
        case_pair_mass=Fraction(out * ins, parent * parent) if parent else Fraction(),
        positive_only_events=sum(
            r["outside_rows"] > 0 and not r["inside_rows"] for r in er
        ),
        negative_only_events=sum(
            r["inside_rows"] > 0 and not r["outside_rows"] for r in er
        ),
    )
    event_pairs = sum(r["same_event_pair_count"] for r in er)
    for scope in SCOPES:
        n = sum(r[scope + "_pair_count"] for r in er)
        result.update(
            {
                scope + "_pair_count": n,
                scope
                + "_pair_mass": sum((r[scope + "_pair_mass"] for r in er), Fraction()),
                scope
                + "_supported_events": sum(r[scope + "_pair_count"] > 0 for r in er),
                scope
                + "_reason": reason(
                    parent, count, eligible, population, out, ins, event_pairs, n
                ),
            }
        )
    result["other_event_pair_count"] = out * ins - event_pairs
    result["other_event_pair_mass"] = (
        result["case_pair_mass"] - result["same_event_pair_mass"]
    )
    assert result["other_event_pair_count"] >= 0
    return result, er


def summary(rows, metadata, mode):
    assert len({r["case_id"] for r in rows}) == len(rows)
    result = dict(
        metadata,
        mode=mode,
        planned_cases=len(rows),
        parent_represented_cases=sum(r["parent_records"] > 0 for r in rows),
        stratum_represented_cases=sum(r["stratum_records"] > 0 for r in rows),
        family_eligible_cases=sum(r["common_family_eligible"] for r in rows),
        selected_cases=sum(r["records"] > 0 for r in rows),
    )
    for field in (
        "records",
        "events",
        "outside_rows",
        "inside_rows",
        "case_mass",
        "case_pair_count",
        "case_pair_mass",
        "other_event_pair_count",
        "other_event_pair_mass",
        "positive_only_events",
        "negative_only_events",
    ):
        result[field] = sum(
            (r[field] for r in rows), Fraction() if field.endswith("mass") else 0
        )
    for scope in SCOPES:
        for suffix in ("pair_count", "pair_mass", "supported_events"):
            field = scope + "_" + suffix
            result[field] = sum(
                (r[field] for r in rows), Fraction() if suffix == "pair_mass" else 0
            )
        result[scope + "_supported_cases"] = sum(
            r[scope + "_pair_count"] > 0 for r in rows
        )
        for why in REASONS:
            result[scope + "_cases_" + why] = sum(
                r[scope + "_reason"] == why for r in rows
            )
        assert sum(result[scope + "_cases_" + why] for why in REASONS) == len(rows)
        result[scope + "_over_case"] = (
            result[scope + "_pair_mass"] / result["case_pair_mass"]
            if result["case_pair_mass"]
            else None
        )
        if scope != "same_event":
            result[scope + "_over_event"] = (
                result[scope + "_pair_mass"] / result["same_event_pair_mass"]
                if result["same_event_pair_mass"]
                else None
            )
    assert (
        result["other_event_pair_count"] + result["same_event_pair_count"]
        == result["case_pair_count"]
    )
    assert (
        result["other_event_pair_mass"] + result["same_event_pair_mass"]
        == result["case_pair_mass"]
    )
    assert all(
        result[a + "_pair_count"] >= result[b + "_pair_count"]
        for a, b in zip(SCOPES, SCOPES[1:])
    )
    return result


def complete_events(support, timed):
    """Use the original label-blind ledger to retain empty event/stratum cells."""
    index(support, ("event_id", "stratum"))
    lookup = index(timed, ("event_id", "stratum", "target", "label"))
    expected = set()
    result = []
    for raw in support:
        for target, label in product(TARGETS, LABELS):
            key = raw["event_id"], raw["stratum"], target, label
            expected.add(key)
            base = {k: raw[k] for k in META}
            base.update(
                event_id=raw["event_id"],
                stratum=raw["stratum"],
                target=target,
                label=label,
                common_family_eligible=raw["common_family_eligible"],
                records=raw["records"],
                outside_rows=raw[f"{target}_{label}_outside_rows"],
                inside_rows=raw[f"{target}_{label}_inside_rows"],
            )
            if integer(raw["records"]):
                assert key in lookup, "missing nonempty time support"
                timed_row = lookup[key]
                assert all(
                    timed_row[k] == v for k, v in base.items()
                ), "inconsistent original event support"
                result.append(timed_row)
            else:
                assert key not in lookup, "unexpected empty time row"
                for b in TIME_BINS:
                    base[b + "_pair_count"] = "0"
                    base[b + "_denominator"] = "0.0"
                result.append(base)
    assert set(lookup) <= expected, "unknown timed event/stratum/target/label"
    return result


def aggregate(case_strata, event_times):
    index(case_strata, ("case_id", "stratum"))
    index(event_times, ("case_id", "event_id", "stratum", "target", "label"))
    selected = defaultdict(list)
    expected = {(r["case_id"], r["stratum"]) for r in case_strata}
    for e in event_times:
        assert (e["case_id"], e["stratum"]) in expected, "unknown event case"
        assert e["target"] in TARGETS and e["label"] in LABELS
        selected[e["case_id"], e["stratum"], e["target"], e["label"]].append(e)
    cases, events = [], []
    for raw in case_strata:
        for target, label, population in product(TARGETS, LABELS, POPULATIONS):
            c, ee = case_result(
                raw,
                selected[raw["case_id"], raw["stratum"], target, label],
                target,
                label,
                population,
            )
            cases.append(c)
            events.extend(ee)
    groups = defaultdict(list)
    for c in cases:
        assert c["mode"] in MODES
        groups[tuple(c[k] for k in GROUP)].append(c)
    summaries = []
    for group, rr in sorted(groups.items()):
        for mode in (*MODES, "all_modes"):
            ss = rr if mode == "all_modes" else [r for r in rr if r["mode"] == mode]
            summaries.append(summary(ss, dict(zip(GROUP, group)), mode))
    return cases, events, summaries


def coverage(cases, census, event_times):
    by_key = index(cases, KEY)
    census_index = index(census, ("event_id",))
    refs = {}
    for r in event_times:
        assert (r["event_id"],) in census_index, "unknown census event"
        assert all(
            r[k] == census_index[(r["event_id"],)][k] for k in KEY
        ), "wrong-case census event"
        if r["target"] == "midpoint" and r["label"] == "observed":
            assert (r["event_id"], r["stratum"]) not in refs
            refs[r["event_id"], r["stratum"]] = integer(r["records"])
    assert {e for e, _ in refs} <= {r["event_id"] for r in census}
    result = []
    for event in census:
        case = by_key[tuple(event[k] for k in KEY)]
        assert event["noise"] == case["noise"]
        part = "new" if event["partition"] == "existing_new" else event["partition"]
        assert part == case["partition"]
        r = {k: case[k] for k in META}
        r.update(
            {
                k: event[k]
                for k in (
                    "event_id",
                    "event_kind",
                    "scheduled",
                    "received",
                    "precision_activation",
                )
            }
        )
        for flag in ("scheduled", "received", "precision_activation"):
            r[flag] = boolean(r[flag])
        for stratum in STRATA:
            r[stratum + "_records"] = refs.get((event["event_id"], stratum), 0)
        assert sum(r[s + "_records"] for s in STRATA[1:]) == r["all_records"]
        r["rp_referenced"] = r["all_records"] > 0
        result.append(r)
    return result


def planned(cases):
    by_key = index(cases, KEY)
    assert len(cases) == 972 and len({r["case_id"] for r in cases}) == 972
    for part, noise, arm, mode, scenario, planner in product(
        SEEDS, ("white", "correlated"), ("epoch", "curved"), MODES, SCENARIOS, PLANNERS
    ):
        for seed in SEEDS[part]:
            key = (part + "-" + noise, arm, mode, seed, scenario, planner)
            assert key in by_key, "missing planned full case"
            r = by_key[key]
            assert (r["partition"], r["noise"]) == (part, noise)


def prior_checks(summaries, effects, precision):
    lookup = index(summaries, (*GROUP, "mode"))
    checks = 0
    for r in effects:
        s = lookup[tuple(r[k] for k in GROUP) + (r["unit_id"],)]
        scope = r["scope"]
        for actual, expected in (
            ("removed_records", s["records"]),
            ("removed_cases", s["selected_cases"]),
            ("removed_events", s["events"]),
            ("mode_pair_count", s[scope + "_pair_count"]),
            ("mode_events_with_pairs", s[scope + "_supported_events"]),
            ("mode_cases_with_pairs", s[scope + "_supported_cases"]),
            ("mode_denominator", s[scope + "_pair_mass"]),
        ):
            close(r[actual], expected)
            checks += 1
    for r in precision:
        s = lookup[tuple(r[k] for k in GROUP) + ("all_modes",)]
        scope = r["scope"]
        for actual, suffix in (
            ("pair_count", "pair_count"),
            ("events_with_pairs", "supported_events"),
            ("cases_with_pairs", "supported_cases"),
            ("denominator", "pair_mass"),
        ):
            close(r[actual], s[scope + "_" + suffix])
            checks += 1
    return checks


def exported(row):
    return {k: float(v) if isinstance(v, Fraction) else v for k, v in row.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    assert digest(BASE / "sources.json") == BASE_SHA
    source = json.loads((BASE / "sources.json").read_text())
    pinned = {
        **source["input_sha256"],
        "event-loss-mode-influence/sources.json": BASE_SHA,
        **{
            "event-loss-mode-influence/" + k: v
            for k, v in source["output_sha256"].items()
        },
    }
    helpers = source["analysis"]["helpers_sha256"]
    for name, h in pinned.items():
        assert digest(RESULTS / name) == h, name
    for name, h in helpers.items():
        assert digest(ROOT / name) == h, name
    old_cases, census = read_rows(DATA / "cases.csv"), read_rows(DATA / "events.csv")
    planned(old_cases)
    old_lookup = index(old_cases, ("case_id",))
    cs, et = read_rows(DATA / "case-strata.csv"), read_rows(
        DATA / "event-time-support.csv"
    )
    assert {(r["case_id"], r["stratum"]) for r in cs} == {
        (r["case_id"], s) for r in old_cases for s in STRATA
    }
    for r in cs:
        assert all(
            r[k] == old_lookup[(r["case_id"],)][k]
            for k in (*META, "parent_records", "case_row_weight")
        )
    support_name = "event-conditioned-rankings/event-support.csv"
    assert support_name in pinned
    et = complete_events(read_rows(RESULTS / support_name), et)
    cases, events, summaries = aggregate(cs, et)
    census_rows = coverage(old_cases, census, et)
    assert (len(cases), len(events), len(summaries), len(census_rows)) == (
        58320,
        16500,
        2880,
        3997,
    )
    assert sum(r["all_records"] for r in census_rows) == 10995
    assert sum(r["rp_referenced"] for r in census_rows) == 275
    checks = prior_checks(
        summaries,
        read_rows(BASE / "mode-effects.csv"),
        read_rows(RESULTS / "event-score-ties/precision-summary.csv"),
    )
    args.output.mkdir(parents=True)
    for name, rr in (
        ("case-support", cases),
        ("event-support", events),
        ("event-coverage", census_rows),
        ("support-summary", summaries),
    ):
        save_rows(args.output / (name + ".csv"), [exported(r) for r in rr])
    audit = dict(
        planned_cases=len(old_cases),
        families=324,
        source_states=10995,
        source_view_references=32985,
        represented_cases=157,
        census_events=len(census_rows),
        referenced_events=275,
        case_support_rows=len(cases),
        event_support_rows=len(events),
        summary_rows=len(summaries),
        preceding_support_comparisons=checks,
    )
    provenance = dict(
        input_sha256=pinned,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        schema=dict(
            strata=STRATA,
            targets=TARGETS,
            labels=LABELS,
            modes=(*MODES, "all_modes"),
            scopes=SCOPES,
            reason_precedence=REASONS,
            weighting="original full-RP-parent 1/n_case; sum masses before shares",
            case_opportunities="same full case, any event; availability denominator only, no new AUC",
            selection="all planned cases; fixed original common-family eligibility",
            retrospective=True,
        ),
        inherited_validation=source["inherited_validation"],
        audit=audit,
        output_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
