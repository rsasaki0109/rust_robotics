#!/usr/bin/env python3
"""Independently reconstruct logged transitions with SQL window functions."""

import argparse
from collections import Counter, defaultdict
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
GROUP = ("partition", "noise", "variant", "mode", "event_group", "horizon")
KINDS = (
    "received",
    "missed",
    "precision_only",
    "received_and_precision",
    "missed_and_precision",
)
STRATA = (
    "recovery_single",
    "filter_single",
    "fusion_three",
    "recovery_fallback_single",
)
BASE_SHA = "e58fc36fe4c1768dceb2378c2893ddc6187941e4801807b01ff4347724db1dd6"
comparisons = 0


def sha(path):
    with path.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def index(rows, keys):
    result = {tuple(r[k] for k in keys): r for r in rows}
    assert len(result) == len(rows), "duplicate identity"
    return result


def check(value, expected):
    global comparisons
    comparisons += 1
    if expected is None:
        assert value == "", (value, expected)
    elif isinstance(expected, (str, bool, int)):
        assert value == str(expected), (value, expected)
    else:
        assert value != "" and math.isclose(
            float(value), float(expected), rel_tol=1e-10, abs_tol=2e-12
        ), (value, expected)


def fields(row, expected):
    assert set(row) == set(expected), (
        set(row) - set(expected),
        set(expected) - set(row),
    )
    for k, v in expected.items():
        try:
            check(row[k], v)
        except AssertionError as error:
            raise AssertionError((k, error.args, row.get("event_id"))) from error


def tick(value):
    time = float(value)
    n = round(time * 50)
    assert math.isfinite(time) and 0 <= n <= 9000 and abs(time - n / 50) <= 1e-7
    return n


def truth(value, raw=False):
    no, yes = ("false", "true") if raw else ("False", "True")
    assert value in (no, yes)
    return value == yes


def count(value):
    n = int(value)
    assert n >= 0 and str(n) == value
    return n


def originals(db):
    cases = read(RESULTS / "event-temporal-rankings/cases.csv")
    accounts = index(read(RESULTS / "fresh-seed-mode-census/accounting.csv"), KEY)
    canonical = index(cases, KEY)
    expected_plan = {
        (part + "-" + noise, arm, mode, seed, scenario, planner)
        for part, seeds in (
            ("known", ("269", "270", "271")),
            ("new", ("274", "275", "276")),
            ("fresh", ("277", "278", "279")),
        )
        for noise, arm, mode, seed, scenario, planner in product(
            ("white", "correlated"),
            ("epoch", "curved"),
            ("none", "random", "blackout"),
            seeds,
            ("dense_50x50", "maze_50x50", "open_50x50"),
            ("A*", "JPS", "Theta*"),
        )
    }
    assert set(canonical) == expected_plan and len(cases) == 972
    db.execute(
        "CREATE TABLE cases(cid TEXT PRIMARY KEY, part TEXT, noise TEXT, arm TEXT, mode TEXT, timeout INT, held INT)"
    )
    for c in cases:
        account = accounts[tuple(c[k] for k in KEY)]
        c.update(
            {k: account[k] for k in ("status", "held", "time_s", "selected_timeout")}
        )
        assert truth(c["selected_timeout"]) == (c["status"] == "timeout") and truth(
            c["held"]
        ) == (c["status"] == "held")
        check(c["accounting_partition"], account["partition"])
        check(
            c["partition"],
            "new" if account["partition"] == "existing_new" else account["partition"],
        )
        db.execute(
            "INSERT INTO cases VALUES(?,?,?,?,?,?,?)",
            (
                c["case_id"],
                c["partition"],
                c["noise"],
                c["variant"],
                c["mode"],
                truth(c["selected_timeout"]),
                truth(c["held"]),
            ),
        )
    db.execute(
        "CREATE TABLE events(eid TEXT PRIMARY KEY,cid TEXT,kind TEXT,received INT,origin INT,horizon TEXT,actual TEXT,preview TEXT)"
    )
    events = {}
    for original in read(RESULTS / "fresh-seed-mode-census/events.csv"):
        prefix, number = original["event_id"].split("_")
        assert prefix in ("historical", "fresh") and count(number) > 0
        eid = str(int(number) + (2583 if prefix == "fresh" else 0))
        assert eid not in events
        c = canonical[tuple(original[k] for k in KEY)]
        assert truth(c["selected_timeout"])
        e = {
            **original,
            **{k: c[k] for k in META},
            "source_event_id": original["event_id"],
            "event_id": eid,
        }
        origin = tick(e["event_time_s"])
        assert 7500 <= origin <= 9000 and tick(e["truth_time_s"]) == origin - 1
        forward = min(75, 9000 - origin)
        e["horizon"] = (
            "full" if forward == 75 else "clipped" if forward else "no_forward"
        )
        scheduled, received, precision = (
            truth(e[k]) for k in ("scheduled", "received", "precision_activation")
        )
        assert not received or scheduled
        kind = (
            (
                ("received" if received else "missed")
                + ("_and_precision" if precision else "")
            )
            if scheduled
            else "precision_only"
        )
        assert scheduled or precision
        check(e["event_kind"], kind)
        db.execute(
            "INSERT INTO events VALUES(?,?,?,?,?,?,?,?)",
            (
                eid,
                c["case_id"],
                kind,
                received,
                origin,
                e["horizon"],
                e["actual_estimator"],
                e["preview_estimator"],
            ),
        )
        events[eid] = e
    projections = index(
        read(RESULTS / "contrast-common-errors/simultaneous.csv"), ("source_id",)
    )
    db.execute(
        "CREATE TABLE states(cid TEXT,sid TEXT,eid TEXT,n INT,stratum TEXT,actual TEXT,preview TEXT,PRIMARY KEY(cid,sid))"
    )
    for name in (
        "estimator-mode-transfer/membership.csv",
        "fresh-seed-cutoff-transfer/scores.csv",
    ):
        for m in read(RESULTS / name):
            if m["view"] != "sim_recovery_preview":
                continue
            if "time_s" in m:
                time = m["time_s"]
            else:
                p = projections[(m["source_id"],)]
                for k in (*KEY, "source_event_ids"):
                    check(p[k], m[k])
                time = p["time_s"]
            tags = m["source_event_ids"].split("+")
            assert len(tags) == 1 and tags[0] in events
            eid = tags[0]
            assert all(m[k] == events[eid][k] for k in KEY)
            assert m["stratum"] in STRATA
            db.execute(
                "INSERT INTO states VALUES(?,?,?,?,?,?,?)",
                (
                    events[eid]["case_id"],
                    m["source_id"],
                    eid,
                    tick(time),
                    m["stratum"],
                    m["a_estimator"],
                    m["b_estimator"],
                ),
            )
    db.execute("CREATE INDEX event_states ON states(eid)")
    parents = {
        r[0]: r[1] for r in db.execute("SELECT cid,COUNT(*) FROM states GROUP BY cid")
    }
    for c in cases:
        check(c["parent_records"], parents.get(c["case_id"], 0))
    assert (
        db.execute("SELECT COUNT(*) FROM states").fetchone()[0] == 10995
        and len(parents) == 157
    )
    assert len(events) == 3997
    return cases, events, parents


def raw_checkpoints(db, cases, events, spec, roots):
    planned = defaultdict(dict)
    for c in cases:
        name = f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"
        planned[(c["cohort"], name)][(c["scenario"], c["planner"])] = c
    targets = defaultdict(lambda: defaultdict(list))
    for eid, e in events.items():
        n = tick(e["event_time_s"])
        for j in range(n - 1, min(n + 75, 9000) + 1):
            targets[e["case_id"]][j].append(eid)
    assert set(planned) == {
        (cohort, name) for cohort, s in spec.items() for name in s["raw_sha256"]
    }
    db.execute(
        "CREATE TABLE samples(eid TEXT,n INT,actual TEXT,preview TEXT,speed REAL,age REAL,groups INT,recovery INT,motion INT,scheduled INT,received INT,precision INT,PRIMARY KEY(eid,n))"
    )
    byte_count, timeout_rows = 0, 0
    for ordinal, ((cohort, name), members) in enumerate(sorted(planned.items()), 1):
        path = Path(roots.get(cohort, spec[cohort]["path"])) / name
        assert sha(path) == spec[cohort]["raw_sha256"][name]
        byte_count += path.stat().st_size
        previous, buffer = {}, []
        with path.open(newline="") as f:
            reader = csv.reader(f)
            columns = next(reader)
            assert len(columns) == len(set(columns))
            pos = {
                k: columns.index(k)
                for k in (
                    "scenario",
                    "planner",
                    "time_s",
                    "recovery_sample_count",
                    "motion_filter_sample_count",
                    "epoch_fusion_group_count",
                    "epoch_fusion_available",
                    "measured_speed",
                    "observation_age",
                    "correction",
                    "correction_received",
                    "precision_activation",
                )
            }
            for row in reader:
                key = row[pos["scenario"]], row[pos["planner"]]
                c = members[key]
                n = tick(row[pos["time_s"]])
                assert n == previous.get(key, -1) + 1
                previous[key] = n
                if truth(c["selected_timeout"]):
                    timeout_rows += 1
                eids = targets[c["case_id"]].get(n)
                if not eids:
                    continue
                recovery, motion, groups = (
                    count(row[pos[k]])
                    for k in (
                        "recovery_sample_count",
                        "motion_filter_sample_count",
                        "epoch_fusion_group_count",
                    )
                )
                fusion = truth(row[pos["epoch_fusion_available"]], True)
                speed, age = (
                    float(row[pos[k]]) for k in ("measured_speed", "observation_age")
                )
                assert groups <= 3 and (not fusion or groups == 3)
                assert math.isfinite(speed) and math.isfinite(age) and age >= 0
                flags = [
                    truth(row[pos[k]], True)
                    for k in (
                        "correction",
                        "correction_received",
                        "precision_activation",
                    )
                ]
                for eid in eids:
                    buffer.append(
                        (
                            eid,
                            n,
                            "recovery_mean" if recovery else "raw_fallback",
                            "three_epoch" if fusion else "motion_filter",
                            speed,
                            age,
                            groups,
                            recovery,
                            motion,
                            *flags,
                        )
                    )
        assert set(previous) == set(members)
        for key, c in members.items():
            assert previous[key] == (
                9000 if truth(c["selected_timeout"]) else tick(c["time_s"])
            )
        db.executemany("INSERT INTO samples VALUES(?,?,?,?,?,?,?,?,?,?,?,?)", buffer)
        if ordinal % 12 == 0:
            print(f"Independently checked raw files: {ordinal}/108", flush=True)
    db.execute("""CREATE TABLE checkpoints AS
      WITH lagged AS (SELECT s.*,e.origin,
        LAG(s.actual) OVER(PARTITION BY s.eid ORDER BY s.n) AS prev_actual,
        LAG(s.preview) OVER(PARTITION BY s.eid ORDER BY s.n) AS prev_preview
        FROM samples s JOIN events e USING(eid)),
      marked AS (SELECT *,
        (prev_actual IS NULL OR actual<>prev_actual OR preview<>prev_preview) AS boundary,
        (n>origin AND actual<>prev_actual) AS actual_change,
        (n>origin AND preview<>prev_preview) AS preview_change FROM lagged)
      SELECT *,SUM(boundary) OVER(PARTITION BY eid ORDER BY n) AS run FROM marked""")
    db.execute("CREATE UNIQUE INDEX checkpoint_id ON checkpoints(eid,n)")
    return dict(
        raw_files=len(planned), raw_bytes=byte_count, timeout_grid_rows=timeout_rows
    )


def reconstructed(db, events, parents):
    ne = Counter(e["case_id"] for e in events.values())
    nr = Counter(e["case_id"] for e in events.values() if truth(e["received"]))
    traces, intervals, audits = {}, {}, {}
    for eid, e in events.items():
        cid, origin = e["case_id"], tick(e["event_time_s"])
        start, end = origin - 1, min(origin + 75, 9000)
        common = {k: e[k] for k in (*META, "event_id", "source_event_id")}
        here = db.execute(
            "SELECT * FROM samples WHERE eid=? AND n=?", (eid, origin)
        ).fetchone()
        before = db.execute(
            "SELECT * FROM samples WHERE eid=? AND n=?", (eid, start)
        ).fetchone()
        check(e["actual_estimator"], here["actual"])
        check(e["preview_estimator"], here["preview"])
        for ek, rk in (
            ("scheduled", "scheduled"),
            ("received", "received"),
            ("precision_activation", "precision"),
            ("recovery_sample_count", "recovery"),
            ("motion_filter_sample_count", "motion"),
            ("epoch_fusion_group_count", "groups"),
            ("observation_age", "age"),
        ):
            check(
                e[ek],
                (
                    bool(here[rk])
                    if ek in ("scheduled", "received", "precision_activation")
                    else here[rk]
                ),
            )
        check(e["fusion_available"], here["preview"] == "three_epoch")
        stats = db.execute(
            """SELECT COUNT(*) AS checkpoints,MIN(n) AS lo,MAX(n) AS hi,MAX(run) AS runs,
            SUM(preview='three_epoch') AS fusion,SUM(actual_change) AS ac,SUM(preview_change) AS pc,
            MIN(CASE WHEN actual_change THEN n END) AS first_actual,
            MIN(CASE WHEN preview_change THEN n END) AS first_preview
            FROM checkpoints WHERE eid=?""",
            (eid,),
        ).fetchone()
        assert (stats["checkpoints"], stats["lo"], stats["hi"]) == (
            end - start + 1,
            start,
            end,
        )
        for r in db.execute(
            "SELECT run,MIN(n) AS lo,MAX(n) AS hi,COUNT(*) AS n,MIN(actual) AS a,MAX(actual) AS a2,MIN(preview) AS p,MAX(preview) AS p2 FROM checkpoints WHERE eid=? GROUP BY run",
            (eid,),
        ):
            assert (
                r["a"] == r["a2"]
                and r["p"] == r["p2"]
                and r["n"] == r["hi"] - r["lo"] + 1
            )
            intervals[(eid, str(r["run"]))] = dict(
                **common,
                interval=r["run"],
                start_time_s=r["lo"] / 50,
                end_time_s=r["hi"] / 50,
                start_offset_ms=(r["lo"] - origin) * 20,
                end_offset_ms=(r["hi"] - origin) * 20,
                checkpoint_count=r["n"],
                actual_estimator=r["a"],
                preview_estimator=r["p"],
            )
        parent = parents.get(cid, 0)
        temporal, strata = Counter(), Counter()
        after_offsets = []
        selected = db.execute(
            "SELECT s.*,c.actual AS logged_actual,c.preview AS logged_preview,c.run FROM states s JOIN checkpoints c ON c.eid=s.eid AND c.n=s.n WHERE s.eid=?",
            (eid,),
        ).fetchall()
        assert (
            len(selected)
            == db.execute("SELECT COUNT(*) FROM states WHERE eid=?", (eid,)).fetchone()[
                0
            ]
        )
        for s in selected:
            assert (
                s["cid"] == cid
                and s["actual"] == s["logged_actual"]
                and s["preview"] == s["logged_preview"]
            )
            assert (
                s["stratum"]
                == {"motion_filter": "filter_single", "three_epoch": "fusion_three"}[
                    s["logged_preview"]
                ]
            )
            relation = (
                "before" if s["n"] < origin else "after" if s["n"] > origin else "at"
            )
            different = s["logged_preview"] != here["preview"]
            if different:
                temporal[relation] += 1
                if relation == "after":
                    assert (
                        stats["first_preview"] is not None
                        and stats["first_preview"] <= s["n"]
                    )
                    after_offsets.append((s["n"] - origin) * 20)
            strata[s["stratum"]] += 1
            audits[(cid, s["sid"])] = dict(
                **common,
                source_id=s["sid"],
                time_s=s["n"] / 50,
                truth_time_s=(s["n"] - 1) / 50,
                offset_ms=(s["n"] - origin) * 20,
                stratum=s["stratum"],
                parent_rp_records=parent,
                rp_case_weight=Fraction(1, parent),
                actual_estimator=s["logged_actual"],
                preview_estimator=s["logged_preview"],
                immediate_preview=here["preview"],
                interval=s["run"],
                temporal_relation=relation,
                different_preview=different,
            )
        first = stats["first_preview"]
        boundary = (
            db.execute(
                "SELECT * FROM samples WHERE eid=? AND n=?", (eid, first)
            ).fetchone()
            if first is not None
            else None
        )
        n, different = len(selected), sum(temporal.values())
        out = dict(
            **common,
            event_kind=e["event_kind"],
            event_time_s=origin / 50,
            truth_time_s=(origin - 1) / 50,
            scheduled=truth(e["scheduled"]),
            received=truth(e["received"]),
            precision_activation=truth(e["precision_activation"]),
            horizon=e["horizon"],
            forward_steps=end - origin,
            start_time_s=start / 50,
            end_time_s=end / 50,
            checkpoint_count=stats["checkpoints"],
            interval_count=stats["runs"],
            fusion_checkpoints=stats["fusion"],
            filter_checkpoints=stats["checkpoints"] - stats["fusion"],
            immediate_actual=here["actual"],
            immediate_preview=here["preview"],
            pre_actual=before["actual"],
            pre_preview=before["preview"],
            preview_changes=stats["pc"],
            actual_changes=stats["ac"],
            first_preview_change_offset_ms=(
                (first - origin) * 20 if first is not None else None
            ),
            first_actual_change_offset_ms=(
                (stats["first_actual"] - origin) * 20
                if stats["first_actual"] is not None
                else None
            ),
            first_preview_change_speed=boundary["speed"] if boundary else None,
            first_preview_change_observation_age=boundary["age"] if boundary else None,
            first_preview_change_retained_groups=(
                boundary["groups"] if boundary else None
            ),
            census_case_weight=Fraction(1, ne[cid]),
            received_case_weight=(
                Fraction(1, nr[cid]) if truth(e["received"]) else Fraction()
            ),
            parent_rp_records=parent,
            rp_records=n,
            rp_case_mass=Fraction(n, parent) if parent else Fraction(),
            rp_same_preview_records=n - different,
            rp_different_preview_records=different,
            **{
                "rp_different_" + t + "_records": temporal[t]
                for t in ("before", "at", "after")
            },
            rp_different_after_logged_change_records=temporal["after"],
            first_after_different_rp_offset_ms=(
                min(after_offsets) if after_offsets else None
            ),
            first_after_different_rp_lag_ms=(
                min(after_offsets) - (first - origin) * 20 if after_offsets else None
            ),
        )
        for s in STRATA:
            out["rp_" + s + "_records"] = strata[s]
            out["rp_" + s + "_case_mass"] = (
                Fraction(strata[s], parent) if parent else Fraction()
            )
        traces[eid] = out
    assert (
        sum(e["census_case_weight"] for e in traces.values())
        == sum(e["received_case_weight"] for e in traces.values())
        == 570
    )
    assert sum(e["rp_case_mass"] for e in traces.values()) == 157
    preceding = index(
        read(RESULTS / "census-rp-selection/event-selection.csv"), ("event_id",)
    )
    for eid, t in traces.items():
        for k in (
            "rp_records",
            "rp_case_mass",
            "rp_same_preview_records",
            "rp_different_preview_records",
            *(
                "rp_" + s + suffix
                for s, suffix in product(STRATA, ("_records", "_case_mass"))
            ),
        ):
            check(preceding[(eid,)][k], t[k])
    return traces, intervals, audits


def summaries(db, cases, traces):
    counts = (
        "checkpoint_count",
        "interval_count",
        "fusion_checkpoints",
        "filter_checkpoints",
        "preview_changes",
        "actual_changes",
        "rp_records",
        "rp_same_preview_records",
        "rp_different_preview_records",
        "rp_different_before_records",
        "rp_different_at_records",
        "rp_different_after_records",
        "rp_different_after_logged_change_records",
        *("rp_" + s + "_records" for s in STRATA),
    )
    db.execute(
        "CREATE TABLE metrics(eid TEXT PRIMARY KEY,"
        + ",".join(k + " INT" for k in counts)
        + ")"
    )
    db.executemany(
        "INSERT INTO metrics VALUES("
        + ",".join("?" for _ in range(len(counts) + 1))
        + ")",
        [(eid, *(t[k] for k in counts)) for eid, t in traces.items()],
    )
    result = {}
    for part, noise, arm, mode, kind, horizon in product(
        ("known", "new", "fresh"),
        ("white", "correlated"),
        ("epoch", "curved"),
        ("none", "random", "blackout", "all_modes"),
        ("all", "received_any", *KINDS),
        ("all", "full", "clipped", "no_forward"),
    ):
        condition, params = "c.part=? AND c.noise=? AND c.arm=?", [part, noise, arm]
        if mode != "all_modes":
            condition += " AND c.mode=?"
            params.append(mode)
        cc = db.execute(
            "SELECT COUNT(*) AS planned,SUM(timeout) AS timeout,SUM(held) AS held FROM cases c WHERE "
            + condition,
            params,
        ).fetchone()
        if kind != "all":
            if kind == "received_any":
                condition += " AND e.received=1"
            else:
                condition += " AND e.kind=?"
                params.append(kind)
        if horizon != "all":
            condition += " AND e.horizon=?"
            params.append(horizon)
        base = (
            " FROM events e JOIN cases c ON c.cid=e.cid JOIN metrics m USING(eid) WHERE "
            + condition
        )
        selected = [traces[r[0]] for r in db.execute("SELECT e.eid" + base, params)]
        stats = db.execute(
            "SELECT COUNT(*) AS census_events,COUNT(DISTINCT e.cid) AS census_cases,SUM(e.received) AS received_events,"
            + ",".join("SUM(m." + k + ") AS " + k for k in counts)
            + base,
            params,
        ).fetchone()
        fusion = [e for e in selected if e["immediate_preview"] == "three_epoch"]
        lost = [e for e in fusion if e["preview_changes"]]
        changed = [e for e in selected if e["preview_changes"]]
        mass = lambda rr, key: sum((e[key] for e in rr), Fraction())
        out = dict(zip(GROUP, (part, noise, arm, mode, kind, horizon)))
        out.update(
            planned_cases=cc["planned"],
            timeout_cases=cc["timeout"],
            held_cases=cc["held"],
            **{
                k: stats[k] or 0
                for k in ("census_events", "census_cases", "received_events", *counts)
            },
            rp_referenced_events=sum(bool(t["rp_records"]) for t in selected),
            immediate_fusion_events=len(fusion),
            immediate_filter_events=len(selected) - len(fusion),
            preview_changed_events=len(changed),
            actual_changed_events=sum(bool(t["actual_changes"]) for t in selected),
            fusion_loss_events=len(lost),
            filter_gain_events=len(changed) - len(lost),
            rp_different_events=sum(
                bool(t["rp_different_preview_records"]) for t in selected
            ),
            rp_different_cases=len(
                {t["case_id"] for t in selected if t["rp_different_preview_records"]}
            ),
            census_case_mass=mass(selected, "census_case_weight"),
            received_case_mass=mass(selected, "received_case_weight"),
            immediate_fusion_case_mass=mass(fusion, "census_case_weight"),
            fusion_loss_case_mass=mass(lost, "census_case_weight"),
            preview_changed_case_mass=mass(changed, "census_case_weight"),
            rp_case_mass=mass(selected, "rp_case_mass"),
        )
        for s in STRATA:
            out["rp_" + s + "_case_mass"] = mass(selected, "rp_" + s + "_case_mass")
        for label in ("same", "different"):
            out["rp_" + label + "_preview_case_mass"] = sum(
                (
                    Fraction(
                        t["rp_" + label + "_preview_records"], t["parent_rp_records"]
                    )
                    for t in selected
                    if t["parent_rp_records"]
                ),
                Fraction(),
            )
        out["fusion_loss_event_fraction"] = (
            Fraction(len(lost), len(fusion)) if fusion else None
        )
        out["fusion_loss_case_mass_fraction"] = (
            out["fusion_loss_case_mass"] / out["immediate_fusion_case_mass"]
            if out["immediate_fusion_case_mass"]
            else None
        )
        result[tuple(out[k] for k in GROUP)] = out
    return result


def compare_file(path, expected, keys):
    saved = index(read(path), keys)
    assert set(saved) == set(expected), (path, len(saved), len(expected))
    for key, row in saved.items():
        fields(row, expected[key])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--figures", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--raw-roots", type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use an unused evidence filename")
    source = json.loads((args.results / "sources.json").read_text())
    prior_path = RESULTS / "census-rp-selection/sources.json"
    assert sha(prior_path) == BASE_SHA
    prior = json.loads(prior_path.read_text())
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        "census-rp-selection/sources.json": BASE_SHA,
        **{"census-rp-selection/" + k: h for k, h in prior["output_sha256"].items()},
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_census_rp_selection.py": prior["analysis"]["script_sha256"],
    }
    for name, h in source["input_sha256"].items():
        assert sha(RESULTS / name) == h, name
    for name, h in source["analysis"]["helpers_sha256"].items():
        assert sha(ROOT / name) == h, name
    assert (
        sha(ROOT / "analyze_logged_preview_transitions.py")
        == source["analysis"]["script_sha256"]
    )
    assert (
        sha(RESULTS / "logged-preview-transitions/protocol.md")
        == source["analysis"]["protocol_sha256"]
    )
    for name, h in source["output_sha256"].items():
        assert sha(args.results / name) == h, name
    raw = json.loads((RESULTS / "fresh-seed-cutoff-transfer/sources.json").read_text())[
        "raw"
    ]
    assert (
        raw == source["raw"]
        and source["inherited_validation"] == prior["inherited_validation"]
    )
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(raw)
    db = sqlite3.connect(":memory:")
    db.row_factory = sqlite3.Row
    cases, events, parents = originals(db)
    audit = raw_checkpoints(db, cases, events, raw, roots)
    traces, intervals, states = reconstructed(db, events, parents)
    summary = summaries(db, cases, traces)
    compare_file(
        args.results / "event-trace.csv",
        {(k,): v for k, v in traces.items()},
        ("event_id",),
    )
    compare_file(
        args.results / "mode-intervals.csv", intervals, ("event_id", "interval")
    )
    compare_file(args.results / "rp-state-audit.csv", states, ("case_id", "source_id"))
    compare_file(args.results / "transition-summary.csv", summary, GROUP)
    plots = json.loads((args.figures / "plot-sources.json").read_text())
    assert plots["script_sha256"] == sha(ROOT / "plot_logged_preview_transitions.py")
    assert plots["input_sha256"] == {
        name: sha(args.results / name)
        for name in ("sources.json", "event-trace.csv", "transition-summary.csv")
    }
    for name, h in plots["output_sha256"].items():
        assert sha(args.figures / name) == h, name
    plotted_summary = {
        key: r
        for key, r in summary.items()
        if r["mode"] != "all_modes" and r["event_group"] == "all"
    }
    plotted_events = {
        (eid,): r for eid, r in traces.items() if r["rp_different_after_records"]
    }
    assert len(plotted_summary) == 144
    compare_file(args.figures / "plot-values.csv", plotted_summary, GROUP)
    compare_file(args.figures / "plot-event-values.csv", plotted_events, ("event_id",))
    for name in ("raw_files", "raw_bytes", "timeout_grid_rows"):
        assert audit[name] == source["audit"][name]
    for name, n in dict(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len({e["case_id"] for e in events.values()}),
        census_events=len(traces),
        source_states=len(states),
        represented_cases=len(parents),
        interval_rows=len(intervals),
        checkpoint_references=db.execute("SELECT COUNT(*) FROM samples").fetchone()[0],
        summary_rows=len(summary),
    ).items():
        assert source["audit"][name] == n
        audit[name] = n
    audit.update(
        comparisons=comparisons,
        plotted_summary_rows=len(plotted_summary),
        plotted_event_rows=len(plotted_events),
        method="original raw csv.reader, independent broad-census/RP joins, uncompressed SQL LAG and cumulative run windows, exact parent fractions; no analyzer imports",
        sqlite_version=sqlite3.sqlite_version,
        script_sha256=sha(Path(__file__)),
        sources_sha256=sha(args.results / "sources.json"),
        plot_sources_sha256=sha(args.figures / "plot-sources.json"),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(audit, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
