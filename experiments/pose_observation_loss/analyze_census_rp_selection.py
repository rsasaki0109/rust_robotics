#!/usr/bin/env python3
"""Join the full late-timeout census to the fixed RP diagnostic selection."""

import argparse
from collections import Counter, defaultdict
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
DATA = RESULTS / "event-temporal-rankings"
BASE = RESULTS / "event-support-attrition"
BASE_SHA = "ab2e13f3cd055b7585da2387acf1983bea6f5ddffed9872cd8c3d13d314e5407"
PROTOCOL = RESULTS / "census-rp-selection/protocol.md"
EXTRA = {
    "fresh-seed-mode-census/events.csv": "d9eedf0c559abb81f39c9ae68a859e15fcec3471e2ed1274383eb8e0e1b39dfd",
    "fresh-seed-mode-census/accounting.csv": "00e055290eb36175bd7951510ada1a31957eff41e853049816a7d6bc86a90462",
}
KEY = ("cohort", "variant", "mode", "seed", "scenario", "planner")
META = ("case_id", "family_id", *KEY, "partition", "accounting_partition", "noise")
KINDS = (
    "received",
    "missed",
    "precision_only",
    "received_and_precision",
    "missed_and_precision",
)
EVENT_GROUPS = ("all", "received_any", *KINDS)
ACTUAL = ("raw_fallback", "recovery_mean")
PREVIEW = ("motion_filter", "three_epoch")
MODES = ("none", "random", "blackout")
STRATA = (
    "recovery_single",
    "filter_single",
    "fusion_three",
    "recovery_fallback_single",
)
STATE_PREVIEW = {"filter_single": "motion_filter", "fusion_three": "three_epoch"}
SEEDS = {
    "known": ("269", "270", "271"),
    "new": ("274", "275", "276"),
    "fresh": ("277", "278", "279"),
}
SCENARIOS = ("dense_50x50", "maze_50x50", "open_50x50")
PLANNERS = ("A*", "JPS", "Theta*")
GROUP = (
    "partition",
    "noise",
    "variant",
    "mode",
    "event_group",
    "actual_selector",
    "preview_selector",
)
OUTCOME = (
    "status",
    "held",
    "collision",
    "time_s",
    "goal_distance",
    "selected_timeout",
    "intervals",
    "swept_contact_intervals",
    "out_of_bounds_intervals",
)
EVENT_FLAGS = ("scheduled", "received", "precision_activation", "fusion_available")
EVENT_INTS = (
    "recovery_sample_count",
    "motion_filter_sample_count",
    "epoch_fusion_group_count",
    "actual_used_sample_count",
    "preview_used_sample_count",
)
EVENT_FLOATS = (
    "event_time_s",
    "truth_time_s",
    "observation_age",
    "actual_x",
    "actual_y",
    "actual_yaw",
    "preview_x",
    "preview_y",
    "preview_yaw",
)


def index(rows, keys):
    result = {tuple(r[k] for k in keys): r for r in rows}
    assert len(result) == len(rows), "duplicate identity"
    return result


def flag(v):
    assert v in ("True", "False"), "canonical boolean required"
    return v == "True"


def integer(v):
    assert not isinstance(v, bool)
    n = int(v)
    assert n >= 0 and (isinstance(v, int) or str(n) == v)
    return n


def partition(v):
    return "new" if v == "existing_new" else v


def microseconds(v):
    t = float(v)
    assert math.isfinite(t)
    n = round(t * 1_000_000)
    assert abs(n / 1_000_000 - t) <= 1e-7
    return n


def close(actual, expected):
    assert actual != "" and math.isclose(
        float(actual), float(expected), rel_tol=1e-10, abs_tol=2e-12
    ), (actual, expected)


def event_kind(e):
    scheduled, received, precision = (
        flag(e[k]) for k in ("scheduled", "received", "precision_activation")
    )
    assert not received or scheduled
    if not scheduled:
        assert precision
        return "precision_only"
    return ("received" if received else "missed") + (
        "_and_precision" if precision else ""
    )


def validate_plan(cases):
    planned = index(cases, KEY)
    assert len(cases) == len(index(cases, ("case_id",))) == 972
    expected = {
        (part + "-" + noise, arm, mode, seed, scenario, planner)
        for part, noise, arm, mode, scenario, planner in product(
            SEEDS,
            ("white", "correlated"),
            ("epoch", "curved"),
            MODES,
            SCENARIOS,
            PLANNERS,
        )
        for seed in SEEDS[part]
    }
    assert set(planned) == expected


def build(cases, accounting, census, mapping):
    case_lookup, accounts, events = (
        index(cases, KEY),
        index(accounting, KEY),
        index(census, ("event_id",)),
    )
    assert set(case_lookup) == set(accounts)
    index(mapping, ("case_id", "source_id"))
    by_case, by_event, references = (
        defaultdict(list),
        defaultdict(list),
        defaultdict(list),
    )
    by_id = index(cases, ("case_id",))
    for e in census:
        key = tuple(e[k] for k in KEY)
        assert (
            key in case_lookup
            and e["event_kind"] == event_kind(e)
            and e["event_kind"] in KINDS
        )
        info = case_lookup[key]
        assert (
            partition(e["partition"]) == info["partition"]
            and e["noise"] == info["noise"]
        )
        assert e["actual_estimator"] in ACTUAL and e["preview_estimator"] in PREVIEW
        assert flag(e["fusion_available"]) == (e["preview_estimator"] == "three_epoch")
        history_groups = integer(e["epoch_fusion_group_count"])
        assert history_groups <= 3
        assert not flag(e["fusion_available"]) or history_groups == 3
        assert e["actual_estimator"] == (
            "recovery_mean" if integer(e["recovery_sample_count"]) else "raw_fallback"
        )
        assert 150_000_000 <= microseconds(e["event_time_s"]) <= 180_000_000
        assert (
            microseconds(e["truth_time_s"]) == microseconds(e["event_time_s"]) - 20_000
        )
        by_case[info["case_id"]].append(e)
    for r in mapping:
        assert (r["case_id"],) in by_id and (r["event_id"],) in events
        info, event = by_id[(r["case_id"],)], events[(r["event_id"],)]
        assert all(r[k] == info[k] == event[k] for k in KEY), "RP full-case event join"
        assert all(r[k] == info[k] for k in META), "RP reporting identity"
        assert r["stratum"] in STRATA
        assert integer(info["parent_records"]) > 0
        close(r["case_row_weight"], Fraction(1, integer(info["parent_records"])))
        close(r["event_time_s"], event["event_time_s"])
        offset = microseconds(r["time_s"]) - microseconds(event["event_time_s"])
        assert -20_000 <= offset <= 1_500_000, "RP source-event window"
        assert microseconds(r["time_s"]) <= 180_000_000, "RP window clipped at 180 s"
        references[r["case_id"]].append(r)
        by_event[r["event_id"]].append(r)
    case_rows, event_rows = [], []
    for info in cases:
        a = accounts[tuple(info[k] for k in KEY)]
        assert (
            partition(a["partition"]) == info["partition"]
            and a["noise"] == info["noise"]
        )
        selected = flag(a["selected_timeout"])
        assert a["status"] in ("held", "timeout") and flag(a["held"]) == (
            a["status"] == "held"
        )
        assert selected == (a["status"] == "timeout")
        ee, rr = by_case[info["case_id"]], references[info["case_id"]]
        parent = integer(info["parent_records"])
        assert len(rr) == parent
        if selected:
            assert integer(a["intervals"]) == 9000
            assert {
                microseconds(e["event_time_s"]) for e in ee if flag(e["scheduled"])
            } == set(range(150_000_000, 180_000_001, 5_000_000))
        else:
            assert not ee and not rr, "held cases are outside the late-timeout census"
        c = {k: info[k] for k in META}
        c.update({k: a[k] for k in OUTCOME})
        for k in ("held", "collision", "selected_timeout"):
            c[k] = flag(c[k])
        for k in ("intervals", "swept_contact_intervals", "out_of_bounds_intervals"):
            c[k] = integer(c[k])
        for k in ("time_s", "goal_distance"):
            c[k] = float(c[k])
        c.update(
            stage=(
                "held" if not selected else "timeout_with_rp" if rr else "timeout_no_rp"
            ),
            census_events=len(ee),
            received_events=sum(flag(e["received"]) for e in ee),
            rp_referenced_events=sum(bool(by_event[e["event_id"]]) for e in ee),
            rp_records=parent,
            rp_case_mass=Fraction(bool(parent)),
        )
        for estimator in (*ACTUAL, *PREVIEW):
            column = "actual_estimator" if estimator in ACTUAL else "preview_estimator"
            c[estimator + "_events"] = sum(e[column] == estimator for e in ee)
        for stratum in STRATA:
            c["rp_" + stratum + "_records"] = sum(r["stratum"] == stratum for r in rr)
        case_rows.append(c)
        for e in ee:
            records = by_event[e["event_id"]]
            er = {k: info[k] for k in META}
            er.update(
                event_id=e["event_id"],
                source_event_id=e["source_event_id"],
                event_partition=e["partition"],
                event_kind=e["event_kind"],
                actual_estimator=e["actual_estimator"],
                preview_estimator=e["preview_estimator"],
            )
            er.update({k: flag(e[k]) for k in EVENT_FLAGS})
            er.update({k: integer(e[k]) for k in EVENT_INTS})
            er.update({k: float(e[k]) for k in EVENT_FLOATS})
            er.update(
                case_status=a["status"],
                parent_rp_records=parent,
                census_case_weight=Fraction(1, len(ee)),
                received_case_weight=(
                    Fraction(1, c["received_events"]) if er["received"] else None
                ),
                rp_referenced=bool(records),
                rp_records=len(records),
                rp_case_mass=Fraction(len(records), parent) if parent else Fraction(),
            )
            offsets = [
                microseconds(r["time_s"]) - microseconds(e["event_time_s"])
                for r in records
            ]
            er.update(
                min_event_offset_us=min(offsets, default=None),
                max_event_offset_us=max(offsets, default=None),
            )
            for stratum in STRATA:
                n = sum(r["stratum"] == stratum for r in records)
                er["rp_" + stratum + "_records"] = n
                er["rp_" + stratum + "_case_mass"] = (
                    Fraction(n, parent) if parent else Fraction()
                )
            for comparison, predicate in (
                (
                    "same",
                    lambda r: STATE_PREVIEW.get(r["stratum"]) == e["preview_estimator"],
                ),
                (
                    "different",
                    lambda r: r["stratum"] in STATE_PREVIEW
                    and STATE_PREVIEW[r["stratum"]] != e["preview_estimator"],
                ),
                ("other", lambda r: r["stratum"] not in STATE_PREVIEW),
            ):
                er["rp_" + comparison + "_preview_records"] = sum(
                    predicate(r) for r in records
                )
            assert sum(er["rp_" + s + "_records"] for s in STRATA) == er["rp_records"]
            event_rows.append(er)
    return case_rows, event_rows


def accepts(e, kind, actual, preview):
    assert (
        kind in EVENT_GROUPS
        and actual in ("all", *ACTUAL)
        and preview in ("all", *PREVIEW)
    )
    return (
        (
            kind == "all"
            or (e["received"] if kind == "received_any" else e["event_kind"] == kind)
        )
        and (actual == "all" or e["actual_estimator"] == actual)
        and (preview == "all" or e["preview_estimator"] == preview)
    )


def summarize(cases, events, metadata):
    result = dict(
        metadata,
        planned_cases=len(cases),
        held_cases=sum(c["stage"] == "held" for c in cases),
        timeout_cases=sum(c["selected_timeout"] for c in cases),
        timeout_no_rp_cases=sum(c["stage"] == "timeout_no_rp" for c in cases),
        timeout_with_rp_cases=sum(c["stage"] == "timeout_with_rp" for c in cases),
        census_cases=len({e["case_id"] for e in events}),
        census_events=len(events),
        rp_referenced_cases=len({e["case_id"] for e in events if e["rp_referenced"]}),
        rp_referenced_events=sum(e["rp_referenced"] for e in events),
    )
    result["unreferenced_events"] = (
        result["census_events"] - result["rp_referenced_events"]
    )
    result["raw_event_coverage"] = (
        Fraction(result["rp_referenced_events"], len(events)) if events else None
    )
    for name, field in (
        ("all_census", "census_case_weight"),
        ("received_census", "received_case_weight"),
    ):
        den = sum((e[field] or Fraction() for e in events), Fraction())
        num = sum(
            (e[field] or Fraction() for e in events if e["rp_referenced"]), Fraction()
        )
        result[name + "_mass"] = den
        result["referenced_" + name + "_mass"] = num
        result[name + "_weighted_coverage"] = num / den if den else None
    for field in (
        "rp_records",
        "rp_case_mass",
        "rp_same_preview_records",
        "rp_different_preview_records",
        "rp_other_preview_records",
    ):
        result[field] = sum(
            (e[field] for e in events), Fraction() if field == "rp_case_mass" else 0
        )
    for stratum in STRATA:
        prefix = "rp_" + stratum
        result[prefix + "_records"] = sum(e[prefix + "_records"] for e in events)
        mass = sum((e[prefix + "_case_mass"] for e in events), Fraction())
        result[prefix + "_case_mass"] = mass
        result[prefix + "_mass_share"] = (
            mass / result["rp_case_mass"] if result["rp_case_mass"] else None
        )
    assert result["held_cases"] + result["timeout_no_rp_cases"] + result[
        "timeout_with_rp_cases"
    ] == len(cases)
    assert (
        sum(result["rp_" + s + "_case_mass"] for s in STRATA) == result["rp_case_mass"]
    )
    return result


def aggregate(cases, events):
    result = []
    for part, noise, arm, mode in product(
        SEEDS, ("white", "correlated"), ("epoch", "curved"), (*MODES, "all_modes")
    ):
        selected_cases = [
            c
            for c in cases
            if (c["partition"], c["noise"], c["variant"]) == (part, noise, arm)
            and (mode == "all_modes" or c["mode"] == mode)
        ]
        selected_events = [
            e
            for e in events
            if (e["partition"], e["noise"], e["variant"]) == (part, noise, arm)
            and (mode == "all_modes" or e["mode"] == mode)
        ]
        for kind, actual, preview in product(
            EVENT_GROUPS, ("all", *ACTUAL), ("all", *PREVIEW)
        ):
            metadata = dict(zip(GROUP, (part, noise, arm, mode, kind, actual, preview)))
            rr = [e for e in selected_events if accepts(e, kind, actual, preview)]
            result.append(summarize(selected_cases, rr, metadata))
    return result


def check_closure(rows):
    lookup = index(rows, GROUP)
    additive = (
        "census_events",
        "rp_referenced_events",
        "unreferenced_events",
        "all_census_mass",
        "referenced_all_census_mass",
        "received_census_mass",
        "referenced_received_census_mass",
        "rp_records",
        "rp_case_mass",
        "rp_same_preview_records",
        "rp_different_preview_records",
        "rp_other_preview_records",
        *(
            "rp_" + s + suffix
            for s, suffix in product(STRATA, ("_records", "_case_mass"))
        ),
    )
    checks = 0
    for r in rows:
        for field, all_value, pieces in (
            ("mode", "all_modes", MODES),
            ("event_group", "all", KINDS),
            ("actual_selector", "all", ACTUAL),
            ("preview_selector", "all", PREVIEW),
        ):
            if r[field] != all_value:
                continue
            children = [
                lookup[tuple(value if k == field else r[k] for k in GROUP)]
                for value in pieces
            ]
            for name in additive:
                assert sum(c[name] for c in children) == r[name], (field, name)
                checks += 1
    return checks


def prior_checks(cases, events, case_strata, coverage):
    cc, ee = index(cases, ("case_id",)), index(events, ("event_id",))
    checks = 0
    for r in case_strata:
        expected = cc[(r["case_id"],)][
            "rp_records" if r["stratum"] == "all" else "rp_" + r["stratum"] + "_records"
        ]
        close(r["records"], expected)
        checks += 1
    for r in coverage:
        e = ee[(r["event_id"],)]
        for s in ("all", *STRATA):
            close(
                r[s + "_records"],
                e["rp_records"] if s == "all" else e["rp_" + s + "_records"],
            )
            checks += 1
        assert flag(r["rp_referenced"]) == e["rp_referenced"]
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
        "event-support-attrition/sources.json": BASE_SHA,
        **{
            "event-support-attrition/" + k: h
            for k, h in source["output_sha256"].items()
        },
        **EXTRA,
    }
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_event_support_attrition.py": source["analysis"]["script_sha256"],
    }
    for name, h in pinned.items():
        assert digest(RESULTS / name) == h, name
    for name, h in helpers.items():
        assert digest(ROOT / name) == h, name
    cases = read_rows(DATA / "cases.csv")
    validate_plan(cases)
    cc, ee = build(
        cases,
        read_rows(DATA / "accounting.csv"),
        read_rows(DATA / "events.csv"),
        read_rows(DATA / "source-event-map.csv"),
    )
    ss = aggregate(cc, ee)
    assert (len(cc), len(ee), len(ss)) == (972, 3997, 3024)
    assert sum(c["selected_timeout"] for c in cc) == 570
    assert sum(c["rp_records"] for c in cc) == sum(e["rp_records"] for e in ee) == 10995
    assert sum(e["rp_referenced"] for e in ee) == 275
    assert sum(e["census_case_weight"] for e in ee) == 570
    for c in cc:
        group = [e for e in ee if e["case_id"] == c["case_id"]]
        assert sum(e["rp_case_mass"] for e in group) == c["rp_case_mass"]
        assert sum(
            (e["received_case_weight"] or Fraction() for e in group), Fraction()
        ) == bool(c["received_events"])
    closure = check_closure(ss)
    checks = prior_checks(
        cc,
        ee,
        read_rows(DATA / "case-strata.csv"),
        read_rows(BASE / "event-coverage.csv"),
    )
    args.output.mkdir(parents=True)
    for name, rr in (
        ("case-selection", cc),
        ("event-selection", ee),
        ("selection-summary", ss),
    ):
        save_rows(args.output / (name + ".csv"), [exported(r) for r in rr])
    audit = dict(
        planned_cases=972,
        planned_families=324,
        timeout_cases=570,
        source_states=10995,
        source_view_references=32985,
        represented_cases=157,
        census_events=3997,
        referenced_events=275,
        case_selection_rows=len(cc),
        event_selection_rows=len(ee),
        summary_rows=len(ss),
        preceding_coverage_comparisons=checks,
        additive_closure_comparisons=closure,
    )
    meta = dict(
        input_sha256=pinned,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        schema=dict(
            modes=(*MODES, "all_modes"),
            event_groups=EVENT_GROUPS,
            actual_selectors=("all", *ACTUAL),
            preview_selectors=("all", *PREVIEW),
            rp_strata=STRATA,
            weighting="distinct original all-census-event, received-census-event and RP-state parent measures; sum before division",
            selection="fixed original late-timeout census and RP windows; no common-family or label re-selection",
            retrospective=True,
        ),
        inherited_validation=source["inherited_validation"],
        audit=audit,
        output_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
