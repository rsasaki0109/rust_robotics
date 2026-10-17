#!/usr/bin/env python3
"""Independent original census/RP joins and SQL coverage; no analyzer imports."""

import argparse
import csv
from fractions import Fraction
import hashlib
from itertools import product
import json
import math
from pathlib import Path
import sqlite3

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
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
GROUP = (
    "partition",
    "noise",
    "variant",
    "mode",
    "event_group",
    "actual_selector",
    "preview_selector",
)
SEEDS = {
    "known": ("269", "270", "271"),
    "new": ("274", "275", "276"),
    "fresh": ("277", "278", "279"),
}
checks = 0


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def indexed(rows, keys):
    result = {tuple(r[k] for k in keys): r for r in rows}
    assert len(result) == len(rows)
    return result


def check(actual, expected):
    global checks
    checks += 1
    if expected is None:
        assert actual == "", (actual, expected)
    elif isinstance(expected, bool):
        assert actual == str(expected), (actual, expected)
    elif isinstance(expected, str):
        assert actual == expected, (actual, expected)
    elif isinstance(expected, int):
        assert actual == str(expected), (actual, expected)
    else:
        assert actual != "" and math.isclose(
            float(actual), float(expected), rel_tol=1e-10, abs_tol=2e-12
        ), (actual, expected)


def fields(actual, expected):
    assert set(actual) == set(expected), (
        set(actual) - set(expected),
        set(expected) - set(actual),
    )
    for k, value in expected.items():
        check(actual[k], value)


def micros(value):
    t = float(value)
    assert math.isfinite(t)
    n = round(t * 1_000_000)
    assert abs(t - n / 1_000_000) <= 1e-7
    return n


def truth(v):
    assert v in ("True", "False")
    return v == "True"


def build_database(cases):
    db = sqlite3.connect(":memory:")
    db.row_factory = sqlite3.Row
    db.execute(
        "CREATE TABLE cases (cid TEXT PRIMARY KEY, part TEXT, noise TEXT, arm TEXT, mode TEXT, stage TEXT)"
    )
    db.execute(
        "CREATE TABLE events (eid TEXT PRIMARY KEY, cid TEXT, kind TEXT, actual TEXT, preview TEXT, received INTEGER, rp_records INTEGER)"
    )
    db.execute(
        "CREATE TABLE states (cid TEXT, sid TEXT, eid TEXT, stratum TEXT, time_us INTEGER, PRIMARY KEY (cid, sid))"
    )
    canonical = indexed(cases, KEY)
    accounts = indexed(read(RESULTS / "fresh-seed-mode-census/accounting.csv"), KEY)
    broad = read(RESULTS / "fresh-seed-mode-census/events.csv")
    census, event_case = {}, {}
    for e in broad:
        key = tuple(e[k] for k in KEY)
        info = canonical[key]
        prefix, number = e["event_id"].split("_")
        assert (
            prefix in ("historical", "fresh")
            and str(int(number)) == number
            and int(number) > 0
        )
        eid = str(int(number) + (2583 if prefix == "fresh" else 0))
        assert eid not in census
        census[eid] = e
        event_case[eid] = info["case_id"]
        scheduled, received, precision = (
            truth(e[k]) for k in ("scheduled", "received", "precision_activation")
        )
        assert not received or scheduled
        kind = (
            ("received" if received else "missed")
            + ("_and_precision" if precision else "")
            if scheduled
            else "precision_only"
        )
        assert scheduled or precision
        assert e["event_kind"] == kind and kind in KINDS
        assert e["actual_estimator"] in ACTUAL and e["preview_estimator"] in PREVIEW
        assert truth(e["fusion_available"]) == (e["preview_estimator"] == "three_epoch")
        db.execute(
            "INSERT INTO events VALUES (?, ?, ?, ?, ?, ?, 0)",
            (
                eid,
                info["case_id"],
                kind,
                e["actual_estimator"],
                e["preview_estimator"],
                int(received),
            ),
        )
    combined = indexed(
        read(RESULTS / "event-temporal-rankings/events.csv"), ("event_id",)
    )
    assert len(census) == len(combined) == 3997
    for eid, original in census.items():
        row = combined[(eid,)]
        check(row["source_event_id"], original["event_id"])
        for k, value in original.items():
            if k != "event_id":
                check(row[k], value)
    projections = indexed(
        read(RESULTS / "contrast-common-errors/simultaneous.csv"), ("source_id",)
    )
    states = []
    for m in read(RESULTS / "estimator-mode-transfer/membership.csv"):
        if m["view"] != "sim_recovery_preview":
            continue
        p = projections[(m["source_id"],)]
        for k in (*KEY, "source_event_ids"):
            check(p[k], m[k])
        states.append((m, p["time_s"]))
    states.extend(
        (m, m["time_s"])
        for m in read(RESULTS / "fresh-seed-cutoff-transfer/scores.csv")
        if m["view"] == "sim_recovery_preview"
    )
    assert len(states) == 10995
    for m, time in states:
        key = tuple(m[k] for k in KEY)
        info = canonical[key]
        tags = m["source_event_ids"].split("+")
        assert len(tags) == 1 and tags[0] in census
        eid = tags[0]
        assert all(m[k] == census[eid][k] for k in KEY)
        assert event_case[eid] == info["case_id"] and m["stratum"] in STRATA
        t = micros(time)
        assert (
            -20_000 <= t - micros(census[eid]["event_time_s"]) <= 1_500_000
            and t <= 180_000_000
        )
        db.execute(
            "INSERT INTO states VALUES (?, ?, ?, ?, ?)",
            (info["case_id"], m["source_id"], eid, m["stratum"], t),
        )
    db.execute("CREATE INDEX state_event ON states(eid)")
    db.execute("CREATE INDEX state_case ON states(cid)")
    for r in db.execute(
        "SELECT eid, COUNT(*) AS n FROM states GROUP BY eid"
    ).fetchall():
        db.execute("UPDATE events SET rp_records=? WHERE eid=?", (r["n"], r["eid"]))
    case_rows = []
    for info in cases:
        cid = info["case_id"]
        a = accounts[tuple(info[k] for k in KEY)]
        check(info["accounting_partition"], a["partition"])
        check(
            info["partition"],
            "new" if a["partition"] == "existing_new" else a["partition"],
        )
        check(info["noise"], a["noise"])
        parent = db.execute(
            "SELECT COUNT(*) FROM states WHERE cid=?", (cid,)
        ).fetchone()[0]
        check(info["parent_records"], parent)
        event_counts = db.execute(
            "SELECT COUNT(*) AS n, SUM(received) AS received, SUM(rp_records>0) AS referenced FROM events WHERE cid=?",
            (cid,),
        ).fetchone()
        timeout = a["status"] == "timeout"
        assert (
            a["status"] in ("timeout", "held")
            and truth(a["held"]) == (not timeout)
            and truth(a["selected_timeout"]) == timeout
        )
        times = {
            micros(census[r[0]]["event_time_s"])
            for r in db.execute("SELECT eid FROM events WHERE cid=?", (cid,))
            if truth(census[r[0]]["scheduled"])
        }
        assert times == (
            set(range(150_000_000, 180_000_001, 5_000_000)) if timeout else set()
        )
        assert timeout or (not parent and not event_counts["n"])
        stage = (
            "timeout_with_rp"
            if timeout and parent
            else "timeout_no_rp" if timeout else "held"
        )
        db.execute(
            "INSERT INTO cases VALUES (?, ?, ?, ?, ?, ?)",
            (
                cid,
                info["partition"],
                info["noise"],
                info["variant"],
                info["mode"],
                stage,
            ),
        )
        c = {k: info[k] for k in META}
        c.update(
            status=a["status"],
            held=truth(a["held"]),
            collision=truth(a["collision"]),
            time_s=float(a["time_s"]),
            goal_distance=float(a["goal_distance"]),
            selected_timeout=timeout,
            intervals=int(a["intervals"]),
            swept_contact_intervals=int(a["swept_contact_intervals"]),
            out_of_bounds_intervals=int(a["out_of_bounds_intervals"]),
            stage=stage,
            census_events=event_counts["n"],
            received_events=int(event_counts["received"] or 0),
            rp_referenced_events=int(event_counts["referenced"] or 0),
            rp_records=parent,
            rp_case_mass=Fraction(int(parent > 0)),
        )
        for estimator in (*ACTUAL, *PREVIEW):
            column = "actual" if estimator in ACTUAL else "preview"
            c[estimator + "_events"] = db.execute(
                "SELECT COUNT(*) FROM events WHERE cid=? AND " + column + "=?",
                (cid, estimator),
            ).fetchone()[0]
        for stratum in STRATA:
            c["rp_" + stratum + "_records"] = db.execute(
                "SELECT COUNT(*) FROM states WHERE cid=? AND stratum=?", (cid, stratum)
            ).fetchone()[0]
        case_rows.append(c)
    case_index = indexed(case_rows, ("case_id",))
    event_rows = []
    for eid, original in census.items():
        cid = event_case[eid]
        c = case_index[(cid,)]
        state_counts = {
            r["stratum"]: r["n"]
            for r in db.execute(
                "SELECT stratum, COUNT(*) AS n FROM states WHERE eid=? GROUP BY stratum",
                (eid,),
            )
        }
        n = sum(state_counts.values())
        parent = c["rp_records"]
        r = {k: c[k] for k in META}
        r.update(
            event_id=eid,
            source_event_id=original["event_id"],
            event_partition=original["partition"],
            event_kind=original["event_kind"],
            actual_estimator=original["actual_estimator"],
            preview_estimator=original["preview_estimator"],
        )
        for field in (
            "scheduled",
            "received",
            "precision_activation",
            "fusion_available",
        ):
            r[field] = truth(original[field])
        for field in (
            "recovery_sample_count",
            "motion_filter_sample_count",
            "epoch_fusion_group_count",
            "actual_used_sample_count",
            "preview_used_sample_count",
        ):
            r[field] = int(original[field])
        for field in (
            "event_time_s",
            "truth_time_s",
            "observation_age",
            "actual_x",
            "actual_y",
            "actual_yaw",
            "preview_x",
            "preview_y",
            "preview_yaw",
        ):
            r[field] = float(original[field])
        r.update(
            case_status=c["status"],
            parent_rp_records=parent,
            census_case_weight=Fraction(1, c["census_events"]),
            received_case_weight=(
                Fraction(1, c["received_events"]) if r["received"] else None
            ),
            rp_referenced=bool(n),
            rp_records=n,
            rp_case_mass=Fraction(n, parent) if parent else Fraction(),
        )
        bounds = db.execute(
            "SELECT MIN(time_us), MAX(time_us) FROM states WHERE eid=?", (eid,)
        ).fetchone()
        r["min_event_offset_us"] = (
            bounds[0] - micros(original["event_time_s"]) if n else None
        )
        r["max_event_offset_us"] = (
            bounds[1] - micros(original["event_time_s"]) if n else None
        )
        for stratum in STRATA:
            r["rp_" + stratum + "_records"] = state_counts.get(stratum, 0)
            r["rp_" + stratum + "_case_mass"] = (
                Fraction(state_counts.get(stratum, 0), parent) if parent else Fraction()
            )
        filter_n, fusion_n = state_counts.get("filter_single", 0), state_counts.get(
            "fusion_three", 0
        )
        r["rp_same_preview_records"] = (
            filter_n if original["preview_estimator"] == "motion_filter" else fusion_n
        )
        r["rp_different_preview_records"] = (
            fusion_n if original["preview_estimator"] == "motion_filter" else filter_n
        )
        r["rp_other_preview_records"] = n - filter_n - fusion_n
        event_rows.append(r)
    return db, case_rows, event_rows


def sql_summary(db, event_index, part, noise, arm, mode, kind, actual, preview):
    base = ["c.part=?", "c.noise=?", "c.arm=?"]
    params = [part, noise, arm]
    if mode != "all_modes":
        base.append("c.mode=?")
        params.append(mode)
    parents = db.execute(
        "SELECT COUNT(*) AS planned, SUM(stage='held') AS held, SUM(stage='timeout_no_rp') AS no_rp, SUM(stage='timeout_with_rp') AS rp FROM cases c WHERE "
        + " AND ".join(base),
        params,
    ).fetchone()
    clauses = list(base)
    parameters = list(params)
    if kind == "received_any":
        clauses.append("e.received=1")
    elif kind != "all":
        clauses.append("e.kind=?")
        parameters.append(kind)
    if actual != "all":
        clauses.append("e.actual=?")
        parameters.append(actual)
    if preview != "all":
        clauses.append("e.preview=?")
        parameters.append(preview)
    where = " FROM events e JOIN cases c ON e.cid=c.cid WHERE " + " AND ".join(clauses)
    totals = db.execute(
        "SELECT COUNT(*) AS n, COUNT(DISTINCT e.cid) AS cases, SUM(e.rp_records>0) AS referenced, COUNT(DISTINCT CASE WHEN e.rp_records>0 THEN e.cid END) AS rp_cases"
        + where,
        parameters,
    ).fetchone()
    rows = [
        event_index[(r[0],)] for r in db.execute("SELECT e.eid" + where, parameters)
    ]
    n, nr = totals["n"], int(totals["referenced"] or 0)
    result = dict(zip(GROUP, (part, noise, arm, mode, kind, actual, preview)))
    result.update(
        planned_cases=parents["planned"],
        held_cases=int(parents["held"] or 0),
        timeout_cases=int((parents["no_rp"] or 0) + (parents["rp"] or 0)),
        timeout_no_rp_cases=int(parents["no_rp"] or 0),
        timeout_with_rp_cases=int(parents["rp"] or 0),
        census_cases=totals["cases"],
        census_events=n,
        rp_referenced_cases=totals["rp_cases"],
        rp_referenced_events=nr,
        unreferenced_events=n - nr,
        raw_event_coverage=Fraction(nr, n) if n else None,
    )
    for name, field in (
        ("all_census", "census_case_weight"),
        ("received_census", "received_case_weight"),
    ):
        den = sum((r[field] or Fraction() for r in rows), Fraction())
        num = sum(
            (r[field] or Fraction() for r in rows if r["rp_referenced"]), Fraction()
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
            (r[field] for r in rows), Fraction() if field == "rp_case_mass" else 0
        )
    for stratum in STRATA:
        prefix = "rp_" + stratum
        result[prefix + "_records"] = sum(r[prefix + "_records"] for r in rows)
        mass = sum((r[prefix + "_case_mass"] for r in rows), Fraction())
        result[prefix + "_case_mass"] = mass
        result[prefix + "_mass_share"] = (
            mass / result["rp_case_mass"] if result["rp_case_mass"] else None
        )
    return result


def focus(r):
    return r["mode"] in MODES and (
        (
            r["event_group"] == "all"
            and r["actual_selector"] == "all"
            and r["preview_selector"] in ("all", *PREVIEW)
        )
        or (
            r["event_group"] == "received_any"
            and r["actual_selector"] in ACTUAL
            and r["preview_selector"] in PREVIEW
        )
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--figures", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new evidence file")
    source = json.loads((args.results / "sources.json").read_text())
    previous_sha = "ab2e13f3cd055b7585da2387acf1983bea6f5ddffed9872cd8c3d13d314e5407"
    check(sha(RESULTS / "event-support-attrition/sources.json"), previous_sha)
    check(source["input_sha256"]["event-support-attrition/sources.json"], previous_sha)
    for name, h in source["input_sha256"].items():
        check(sha(RESULTS / name), h)
    for name, h in source["analysis"]["helpers_sha256"].items():
        check(sha(ROOT / name), h)
    check(
        sha(ROOT / "analyze_census_rp_selection.py"),
        source["analysis"]["script_sha256"],
    )
    check(
        sha(RESULTS / "census-rp-selection/protocol.md"),
        source["analysis"]["protocol_sha256"],
    )
    for name, h in source["output_sha256"].items():
        check(sha(args.results / name), h)
    cases = read(RESULTS / "event-temporal-rankings/cases.csv")
    planned = indexed(cases, KEY)
    assert len(planned) == 972
    for part, noise, arm, mode, scenario, planner in product(
        SEEDS,
        ("white", "correlated"),
        ("epoch", "curved"),
        MODES,
        ("dense_50x50", "maze_50x50", "open_50x50"),
        ("A*", "JPS", "Theta*"),
    ):
        for seed in SEEDS[part]:
            assert (part + "-" + noise, arm, mode, seed, scenario, planner) in planned
    db, cc, ee = build_database(cases)
    ei = indexed(ee, ("event_id",))
    ss = [
        sql_summary(db, ei, *key)
        for key in product(
            SEEDS,
            ("white", "correlated"),
            ("epoch", "curved"),
            (*MODES, "all_modes"),
            EVENT_GROUPS,
            ("all", *ACTUAL),
            ("all", *PREVIEW),
        )
    ]
    assert (len(cc), len(ee), len(ss)) == (972, 3997, 3024)
    for name, expected, keys in (
        ("case-selection.csv", cc, ("case_id",)),
        ("event-selection.csv", ee, ("event_id",)),
        ("selection-summary.csv", ss, GROUP),
    ):
        actual = indexed(read(args.results / name), keys)
        assert len(actual) == len(expected)
        for r in expected:
            fields(actual[tuple(r[k] for k in keys)], r)
    ci = indexed(cc, ("case_id",))
    prior = 0
    for r in read(RESULTS / "event-temporal-rankings/case-strata.csv"):
        c = ci[(r["case_id"],)]
        check(
            r["records"],
            (
                c["rp_records"]
                if r["stratum"] == "all"
                else c["rp_" + r["stratum"] + "_records"]
            ),
        )
        prior += 1
    for r in read(RESULTS / "event-support-attrition/event-coverage.csv"):
        e = ei[(r["event_id"],)]
        for s in ("all", *STRATA):
            check(
                r[s + "_records"],
                e["rp_records"] if s == "all" else e["rp_" + s + "_records"],
            )
            prior += 1
        check(r["rp_referenced"], e["rp_referenced"])
        prior += 1
    assert prior == 28842
    lookup = indexed(ss, GROUP)
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
    closure = 0
    for r in ss:
        for dimension, all_value, children in (
            ("mode", "all_modes", MODES),
            ("event_group", "all", KINDS),
            ("actual_selector", "all", ACTUAL),
            ("preview_selector", "all", PREVIEW),
        ):
            if r[dimension] != all_value:
                continue
            rr = [
                lookup[tuple(value if k == dimension else r[k] for k in GROUP)]
                for value in children
            ]
            for field in additive:
                assert sum(c[field] for c in rr) == r[field]
                closure += 1
    assert closure == 64080
    plotted = indexed(read(args.figures / "plot-values.csv"), GROUP)
    expected_plot = [r for r in ss if focus(r)]
    assert len(expected_plot) == len(plotted) == 252
    for r in expected_plot:
        fields(plotted[tuple(r[k] for k in GROUP)], r)
    plot = json.loads((args.figures / "plot-sources.json").read_text())
    check(sha(ROOT / "plot_census_rp_selection.py"), plot["script_sha256"])
    check(sha(args.results / "sources.json"), plot["analysis_sources_sha256"])
    for name, h in plot["inputs_sha256"].items():
        check(sha(args.results / name), h)
    for name, h in plot["outputs_sha256"].items():
        check(sha(args.figures / name), h)
    audit = dict(
        planned_cases=972,
        planned_families=324,
        timeout_cases=570,
        source_states=10995,
        source_view_references=32985,
        represented_cases=157,
        census_events=3997,
        referenced_events=275,
        case_selection_rows=972,
        event_selection_rows=3997,
        summary_rows=3024,
        preceding_coverage_comparisons=prior,
        additive_closure_comparisons=closure,
    )
    for k, value in audit.items():
        check(str(source["audit"][k]), value)
    proof = dict(
        comparisons=checks,
        **audit,
        plotted_rows=len(plotted),
        method="original broad-census IDs and historical membership/latest score RP metadata, SQL selection/count queries, Fraction original parent measures",
        script_sha256=sha(Path(__file__)),
        sources_sha256=sha(args.results / "sources.json"),
        plot_sources_sha256=sha(args.figures / "plot-sources.json"),
    )
    args.output.write_text(json.dumps(proof, indent=2) + "\n")
    print(json.dumps(proof, indent=2), flush=True)
    db.close()


if __name__ == "__main__":
    main()
