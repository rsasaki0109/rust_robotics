#!/usr/bin/env python3
"""Trace fixed late-census windows through original logged estimator modes."""

import argparse
from collections import Counter, defaultdict
import csv
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
BASE = RESULTS / "census-rp-selection"
BASE_SHA = "e58fc36fe4c1768dceb2378c2893ddc6187941e4801807b01ff4347724db1dd6"
PROTOCOL = RESULTS / "logged-preview-transitions/protocol.md"
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
MODES = ("none", "random", "blackout")
HORIZONS = ("full", "clipped", "no_forward")
STRATA = (
    "recovery_single",
    "filter_single",
    "fusion_three",
    "recovery_fallback_single",
)
STATE_PREVIEW = {"filter_single": "motion_filter", "fusion_three": "three_epoch"}
RAW_FIELDS = (
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
COUNTS = (
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
MASSES = (
    "census_case_mass",
    "received_case_mass",
    "immediate_fusion_case_mass",
    "fusion_loss_case_mass",
    "preview_changed_case_mass",
    "rp_case_mass",
    "rp_same_preview_case_mass",
    "rp_different_preview_case_mass",
    *("rp_" + s + "_case_mass" for s in STRATA),
)
EVENT_COUNTS = (
    "census_events",
    "received_events",
    "rp_referenced_events",
    "immediate_fusion_events",
    "immediate_filter_events",
    "preview_changed_events",
    "actual_changed_events",
    "fusion_loss_events",
    "filter_gain_events",
    "rp_different_events",
)


def indexed(rows, keys):
    result = {tuple(r[k] for k in keys): r for r in rows}
    assert len(rows) == len(result), "duplicate identity"
    return result


def step(value):
    t = float(value)
    assert math.isfinite(t)
    n = round(t / 0.02)
    assert abs(t - n * 0.02) <= 1e-7 and 0 <= n <= 9000, "off-grid time"
    return n


def integer(value):
    n = int(value)
    assert str(n) == str(value) and n >= 0, "nonnegative canonical count required"
    return n


def flag(value, raw=False):
    choices = ("false", "true") if raw else ("False", "True")
    assert value in choices, "canonical boolean required"
    return value == choices[1]


def window(time):
    origin = step(time)
    assert 7500 <= origin <= 9000
    forward = min(75, 9000 - origin)
    return (
        origin - 1,
        origin + forward,
        ("full" if forward == 75 else "clipped" if forward else "no_forward"),
    )


def logged(row):
    counts = {k: integer(row[k]) for k in RAW_FIELDS[:3]}
    available = flag(row["epoch_fusion_available"], raw=True)
    assert counts["epoch_fusion_group_count"] <= 3
    assert not available or counts["epoch_fusion_group_count"] == 3
    speed, age = float(row["measured_speed"]), float(row["observation_age"])
    assert math.isfinite(speed) and math.isfinite(age) and age >= 0
    return dict(
        **counts,
        available=available,
        speed=speed,
        age=age,
        actual="recovery_mean" if counts["recovery_sample_count"] else "raw_fallback",
        preview="three_epoch" if available else "motion_filter",
        scheduled=flag(row["correction"], raw=True),
        received=flag(row["correction_received"], raw=True),
        precision=flag(row["precision_activation"], raw=True),
    )


def original_states(cases):
    case_lookup = indexed(cases, KEY)
    projections = indexed(
        read_rows(RESULTS / "contrast-common-errors/simultaneous.csv"), ("source_id",)
    )
    result = []
    for name in (
        "estimator-mode-transfer/membership.csv",
        "fresh-seed-cutoff-transfer/scores.csv",
    ):
        for row in read_rows(RESULTS / name):
            if row["view"] != "sim_recovery_preview":
                continue
            if "time_s" not in row:
                p = projections[(row["source_id"],)]
                assert all(p[k] == row[k] for k in (*KEY, "source_event_ids"))
                row = {**row, "time_s": p["time_s"]}
            info = case_lookup[tuple(row[k] for k in KEY)]
            tags = row["source_event_ids"].split("+")
            assert len(tags) == 1 and row["stratum"] in STRATA
            result.append({**row, "case_id": info["case_id"], "event_id": tags[0]})
    indexed(result, ("case_id", "source_id"))
    return result


def read_raw(spec, cases, events, roots):
    by_file, needed = defaultdict(dict), defaultdict(set)
    for c in cases:
        name = f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"
        by_file[(c["cohort"], name)][(c["scenario"], c["planner"])] = c
    for e in events:
        start, end, _ = window(e["event_time_s"])
        needed[e["case_id"]].update(range(start, end + 1))
    result, checked, byte_count, timeout_rows = {}, 0, 0, 0
    assert set(by_file) == {
        (cohort, name) for cohort, s in spec.items() for name in s["raw_sha256"]
    }
    for cohort, name in sorted(by_file):
        source = spec[cohort]
        path = Path(roots.get(cohort, source["path"])) / name
        assert digest(path) == source["raw_sha256"][name], path
        byte_count += path.stat().st_size
        planned = by_file[(cohort, name)]
        last, selected = {}, {
            c["case_id"]: {} for c in planned.values() if flag(c["selected_timeout"])
        }
        with path.open(newline="") as f:
            reader = csv.DictReader(f)
            assert len(reader.fieldnames) == len(set(reader.fieldnames))
            assert set(("scenario", "planner", "time_s", *RAW_FIELDS)) <= set(
                reader.fieldnames
            )
            for row in reader:
                key = row["scenario"], row["planner"]
                assert key in planned, "unknown trajectory"
                n = step(row["time_s"])
                assert (
                    n == last.get(key, -1) + 1
                ), "duplicate, missing or unordered checkpoint"
                last[key] = n
                cid = planned[key]["case_id"]
                if cid in selected:
                    timeout_rows += 1
                    if n in needed[cid]:
                        selected[cid][n] = logged(row)
        assert set(last) == set(planned)
        for key, c in planned.items():
            cid = c["case_id"]
            if cid in selected:
                assert last[key] == 9000 and set(selected[cid]) == needed[cid]
                result[cid] = selected[cid]
            else:
                assert last[key] == step(c["time_s"])
        checked += 1
        if checked % 12 == 0:
            print(f"Validated original raw files: {checked}/108", flush=True)
    return result, dict(
        raw_files=checked, raw_bytes=byte_count, timeout_grid_rows=timeout_rows
    )


def compressed(points, start, end):
    assert set(points) >= set(range(start, end + 1)), "incomplete window"
    result = []
    for n in range(start, end + 1):
        pair = points[n]["actual"], points[n]["preview"]
        if not result or result[-1][2:] != pair:
            result.append((n, n, *pair))
        else:
            a, _, actual, preview = result[-1]
            result[-1] = a, n, actual, preview
    return result


def trace(e, points, states, n_events, n_received):
    origin = step(e["event_time_s"])
    start, end, horizon = window(e["event_time_s"])
    here, before = points[origin], points[start]
    assert (
        here["actual"] == e["actual_estimator"]
        and here["preview"] == e["preview_estimator"]
    )
    for raw_key, event_key in (
        ("scheduled", "scheduled"),
        ("received", "received"),
        ("precision", "precision_activation"),
        ("available", "fusion_available"),
    ):
        assert here[raw_key] == flag(e[event_key])
    for key in RAW_FIELDS[:3]:
        assert here[key] == integer(e[key])
    assert math.isclose(
        here["age"], float(e["observation_age"]), rel_tol=0, abs_tol=1e-9
    )
    runs = compressed(points, start, end)
    metadata = {k: e[k] for k in (*META, "event_id", "source_event_id")}
    intervals, run_for_step = [], {}
    for ordinal, (a, b, actual, preview) in enumerate(runs, 1):
        intervals.append(
            dict(
                **metadata,
                interval=ordinal,
                start_time_s=a / 50,
                end_time_s=b / 50,
                start_offset_ms=(a - origin) * 20,
                end_offset_ms=(b - origin) * 20,
                checkpoint_count=b - a + 1,
                actual_estimator=actual,
                preview_estimator=preview,
            )
        )
        run_for_step.update({n: ordinal for n in range(a, b + 1)})
    changes = {
        kind: [
            n
            for n in range(origin + 1, end + 1)
            if points[n][kind] != points[n - 1][kind]
        ]
        for kind in ("actual", "preview")
    }
    first = changes["preview"][0] if changes["preview"] else None
    parent = integer(e["parent_rp_records"])
    assert parent >= len(states)
    census_weight = Fraction(1, n_events)
    received_weight = Fraction(1, n_received) if flag(e["received"]) else Fraction()
    assert math.isclose(
        float(e["census_case_weight"]), float(census_weight), abs_tol=1e-12
    )
    assert math.isclose(
        float(e["received_case_weight"] or 0), float(received_weight), abs_tol=1e-12
    )
    audits, timing, strata = [], Counter(), Counter()
    for s in sorted(states, key=lambda r: (step(r["time_s"]), int(r["source_id"]))):
        assert s["case_id"] == e["case_id"] and s["event_id"] == e["event_id"]
        assert all(s[k] == e[k] for k in KEY), "wrong-case source"
        n = step(s["time_s"])
        assert start <= n <= end
        raw = points[n]
        assert (
            s["stratum"] in STATE_PREVIEW
            and STATE_PREVIEW[s["stratum"]] == raw["preview"]
        )
        assert s["a_estimator"] == raw["actual"] and s["b_estimator"] == raw["preview"]
        relation = "before" if n < origin else "after" if n > origin else "at"
        different = raw["preview"] != here["preview"]
        if different:
            timing[relation] += 1
            if relation == "after":
                assert first is not None and first <= n, "no logged post-event change"
        strata[s["stratum"]] += 1
        audits.append(
            dict(
                **metadata,
                source_id=s["source_id"],
                time_s=n / 50,
                truth_time_s=(n - 1) / 50,
                offset_ms=(n - origin) * 20,
                stratum=s["stratum"],
                parent_rp_records=parent,
                rp_case_weight=Fraction(1, parent),
                actual_estimator=raw["actual"],
                preview_estimator=raw["preview"],
                immediate_preview=here["preview"],
                interval=run_for_step[n],
                temporal_relation=relation,
                different_preview=different,
            )
        )
    different_after = [
        r["offset_ms"]
        for r in audits
        if r["different_preview"] and r["temporal_relation"] == "after"
    ]
    different = sum(timing.values())
    count = len(states)
    assert count == integer(e["rp_records"]) and different == integer(
        e["rp_different_preview_records"]
    )
    assert count - different == integer(e["rp_same_preview_records"])
    out = dict(
        **metadata,
        event_kind=e["event_kind"],
        event_time_s=origin / 50,
        truth_time_s=(origin - 1) / 50,
        scheduled=flag(e["scheduled"]),
        received=flag(e["received"]),
        precision_activation=flag(e["precision_activation"]),
        horizon=horizon,
        forward_steps=end - origin,
        start_time_s=start / 50,
        end_time_s=end / 50,
        checkpoint_count=end - start + 1,
        interval_count=len(runs),
        fusion_checkpoints=sum(points[n]["available"] for n in range(start, end + 1)),
        filter_checkpoints=sum(
            not points[n]["available"] for n in range(start, end + 1)
        ),
        immediate_actual=here["actual"],
        immediate_preview=here["preview"],
        pre_actual=before["actual"],
        pre_preview=before["preview"],
        preview_changes=len(changes["preview"]),
        actual_changes=len(changes["actual"]),
        first_preview_change_offset_ms=(
            (first - origin) * 20 if first is not None else None
        ),
        first_actual_change_offset_ms=(
            (changes["actual"][0] - origin) * 20 if changes["actual"] else None
        ),
        first_preview_change_speed=(
            points[first]["speed"] if first is not None else None
        ),
        first_preview_change_observation_age=(
            points[first]["age"] if first is not None else None
        ),
        first_preview_change_retained_groups=(
            points[first]["epoch_fusion_group_count"] if first is not None else None
        ),
        census_case_weight=census_weight,
        received_case_weight=received_weight,
        parent_rp_records=parent,
        rp_records=count,
        rp_case_mass=Fraction(count, parent) if parent else Fraction(),
        rp_same_preview_records=count - different,
        rp_different_preview_records=different,
        **{
            "rp_different_" + k + "_records": timing[k]
            for k in ("before", "at", "after")
        },
        rp_different_after_logged_change_records=timing["after"],
        first_after_different_rp_offset_ms=(
            min(different_after) if different_after else None
        ),
        first_after_different_rp_lag_ms=(
            min(different_after) - (first - origin) * 20 if different_after else None
        ),
    )
    for s in STRATA:
        out["rp_" + s + "_records"] = strata[s]
        out["rp_" + s + "_case_mass"] = (
            Fraction(strata[s], parent) if parent else Fraction()
        )
        assert strata[s] == integer(e["rp_" + s + "_records"])
    assert math.isclose(
        float(out["rp_case_mass"]), float(e["rp_case_mass"]), abs_tol=1e-12
    )
    return out, intervals, audits


def summarize(cases, events, metadata):
    fusion = [e for e in events if e["immediate_preview"] == "three_epoch"]
    loss = [e for e in fusion if e["preview_changes"]]
    changed = [e for e in events if e["preview_changes"]]
    mass = lambda rr, field: sum((e[field] for e in rr), Fraction())
    result = dict(
        **metadata,
        planned_cases=len(cases),
        timeout_cases=sum(flag(c["selected_timeout"]) for c in cases),
        held_cases=sum(flag(c["held"]) for c in cases),
        census_cases=len({e["case_id"] for e in events}),
        census_events=len(events),
        received_events=sum(e["received"] for e in events),
        rp_referenced_events=sum(bool(e["rp_records"]) for e in events),
        immediate_fusion_events=len(fusion),
        immediate_filter_events=len(events) - len(fusion),
        preview_changed_events=len(changed),
        actual_changed_events=sum(bool(e["actual_changes"]) for e in events),
        fusion_loss_events=len(loss),
        filter_gain_events=len(changed) - len(loss),
        rp_different_events=sum(
            bool(e["rp_different_preview_records"]) for e in events
        ),
        rp_different_cases=len(
            {e["case_id"] for e in events if e["rp_different_preview_records"]}
        ),
        census_case_mass=mass(events, "census_case_weight"),
        received_case_mass=mass(events, "received_case_weight"),
        immediate_fusion_case_mass=mass(fusion, "census_case_weight"),
        fusion_loss_case_mass=mass(loss, "census_case_weight"),
        preview_changed_case_mass=mass(changed, "census_case_weight"),
        rp_case_mass=mass(events, "rp_case_mass"),
    )
    for name in COUNTS:
        result[name] = sum(e[name] for e in events)
    for s in STRATA:
        result["rp_" + s + "_case_mass"] = mass(events, "rp_" + s + "_case_mass")
    for label in ("same", "different"):
        result["rp_" + label + "_preview_case_mass"] = sum(
            (
                Fraction(e["rp_" + label + "_preview_records"], e["parent_rp_records"])
                for e in events
                if e["parent_rp_records"]
            ),
            Fraction(),
        )
    result["fusion_loss_event_fraction"] = (
        Fraction(len(loss), len(fusion)) if fusion else None
    )
    result["fusion_loss_case_mass_fraction"] = (
        result["fusion_loss_case_mass"] / result["immediate_fusion_case_mass"]
        if result["immediate_fusion_case_mass"]
        else None
    )
    return result


def aggregate(cases, events):
    result = []
    for part, noise, arm, mode in product(
        ("known", "new", "fresh"),
        ("white", "correlated"),
        ("epoch", "curved"),
        (*MODES, "all_modes"),
    ):
        cc = [
            c
            for c in cases
            if (c["partition"], c["noise"], c["variant"]) == (part, noise, arm)
            and (mode == "all_modes" or c["mode"] == mode)
        ]
        ee = [
            e
            for e in events
            if (e["partition"], e["noise"], e["variant"]) == (part, noise, arm)
            and (mode == "all_modes" or e["mode"] == mode)
        ]
        for kind, horizon in product(
            ("all", "received_any", *KINDS), ("all", *HORIZONS)
        ):
            selected = [
                e
                for e in ee
                if (
                    kind == "all"
                    or (
                        e["received"]
                        if kind == "received_any"
                        else e["event_kind"] == kind
                    )
                )
                and (horizon == "all" or e["horizon"] == horizon)
            ]
            result.append(
                summarize(
                    cc,
                    selected,
                    dict(zip(GROUP, (part, noise, arm, mode, kind, horizon))),
                )
            )
    return result


def closure(rows):
    lookup = indexed(rows, GROUP)
    checks = 0
    for row in rows:
        for field, whole, pieces in (
            ("mode", "all_modes", MODES),
            ("event_group", "all", KINDS),
            ("horizon", "all", HORIZONS),
        ):
            if row[field] != whole:
                continue
            children = [
                lookup[tuple(piece if k == field else row[k] for k in GROUP)]
                for piece in pieces
            ]
            for name in (*COUNTS, *MASSES, *EVENT_COUNTS):
                assert sum(c[name] for c in children) == row[name], (field, name)
                checks += 1
    return checks


def exported(row):
    return {k: float(v) if isinstance(v, Fraction) else v for k, v in row.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--raw-roots",
        type=Path,
        help="optional JSON cohort-to-directory map; pinned bytes remain mandatory",
    )
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    assert digest(BASE / "sources.json") == BASE_SHA
    prior = json.loads((BASE / "sources.json").read_text())
    pins = {
        **prior["input_sha256"],
        "census-rp-selection/sources.json": BASE_SHA,
        **{"census-rp-selection/" + k: h for k, h in prior["output_sha256"].items()},
    }
    helpers = {
        **prior["analysis"]["helpers_sha256"],
        "analyze_census_rp_selection.py": prior["analysis"]["script_sha256"],
    }
    for name, h in pins.items():
        assert digest(RESULTS / name) == h, name
    for name, h in helpers.items():
        assert digest(ROOT / name) == h, name
    raw = json.loads((RESULTS / "fresh-seed-cutoff-transfer/sources.json").read_text())[
        "raw"
    ]
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(raw)
    cases, events = read_rows(BASE / "case-selection.csv"), read_rows(
        BASE / "event-selection.csv"
    )
    indexed(cases, KEY)
    indexed(events, ("event_id",))
    states = original_states(cases)
    mapping = indexed(
        read_rows(RESULTS / "event-temporal-rankings/source-event-map.csv"),
        ("case_id", "source_id"),
    )
    parents, by_event = Counter(s["case_id"] for s in states), defaultdict(list)
    for s in states:
        saved = mapping[(s["case_id"], s["source_id"])]
        assert all(s[k] == saved[k] for k in (*KEY, "event_id", "stratum"))
        assert step(s["time_s"]) == step(saved["time_s"])
        by_event[s["event_id"]].append(s)
    for c in cases:
        assert integer(c["rp_records"]) == parents[c["case_id"]]
    histories, audit = read_raw(raw, cases, events, roots)
    ne = Counter(e["case_id"] for e in events)
    nr = Counter(e["case_id"] for e in events if flag(e["received"]))
    traces, intervals, audits = [], [], []
    for e in events:
        assert parents[e["case_id"]] == integer(e["parent_rp_records"])
        t, ii, aa = trace(
            e,
            histories[e["case_id"]],
            by_event[e["event_id"]],
            ne[e["case_id"]],
            nr[e["case_id"]],
        )
        traces.append(t)
        intervals.extend(ii)
        audits.extend(aa)
    summaries = aggregate(cases, traces)
    checks = closure(summaries)
    assert (len(cases), len(histories), len(traces), len(audits), len(summaries)) == (
        972,
        570,
        3997,
        10995,
        1344,
    )
    assert len({c["family_id"] for c in cases}) == 324 and len(parents) == 157
    assert (
        sum(t["census_case_weight"] for t in traces)
        == sum(t["received_case_weight"] for t in traces)
        == 570
    )
    assert sum(t["rp_case_mass"] for t in traces) == 157
    audit.update(
        planned_cases=972,
        planned_families=324,
        timeout_cases=570,
        census_events=len(traces),
        source_states=len(audits),
        represented_cases=len(parents),
        interval_rows=len(intervals),
        checkpoint_references=sum(t["checkpoint_count"] for t in traces),
        summary_rows=len(summaries),
        additive_closure_comparisons=checks,
    )
    args.output.mkdir(parents=True)
    for name, rows in (
        ("event-trace", traces),
        ("mode-intervals", intervals),
        ("rp-state-audit", audits),
        ("transition-summary", summaries),
    ):
        save_rows(args.output / (name + ".csv"), [exported(r) for r in rows])
    meta = dict(
        input_sha256=pins,
        raw=raw,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        schema=dict(
            horizons=("all", *HORIZONS),
            event_groups=("all", "received_any", *KINDS),
            rp_strata=STRATA,
            logged_grid_ms=20,
            window_offsets_ms=[-20, 1500],
            window_end_clip_s=180,
            mode_source="logged recovery_sample_count and epoch_fusion_available; retained group count alone does not define preview",
            weighting="distinct original all-census, received-census and RP-state parents; exact sums before conditional fractions",
            retrospective=True,
        ),
        inherited_validation=prior["inherited_validation"],
        audit=audit,
        output_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
