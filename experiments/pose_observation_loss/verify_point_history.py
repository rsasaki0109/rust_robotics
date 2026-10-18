#!/usr/bin/env python3
"""Independently verify local point histories, run censorship and native traces."""

import argparse
from collections import Counter, defaultdict
import csv
from decimal import Decimal as D, localcontext
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import verify_point_clearance as previous

core = previous.core
ROOT, REPO, RESULTS = core.ROOT, core.REPO, core.RESULTS
BASE = RESULTS / "fusion-return-point-clearance"
STUDY = RESULTS / "fusion-return-point-history"
BASE_MAIN = "9cc0426a527c3b200a0c7312e9d964a7b06932ae"
BASE_SHA = "6752d0e1aa13908e5b402e63f1079d1fac692c68fc70c66129a930e31ec09d58"
PROOF_SHA = "d6f0254d3e70a4bb371ac1cc17cb2cbf506e4f1b6af12751a0201f8df21ab901"
META, KEY, GROUP = previous.META, previous.KEY, previous.GROUP
PHASES, BRANCHES, CATEGORIES = previous.PHASES, previous.BRANCHES, previous.CATEGORIES
STATES = previous.STATES
CODE = previous.CODE
HELPERS = {
    **previous.HELPERS,
    "verify_point_clearance.py": "83c449189bf723aa72f5c85f3bc1515dd6182a0da963e569b46b7df72d73ca5a",
}
H, P, R = previous.H, previous.P, previous.R
RAW = previous.previous.RAW
scalar = previous.scalar
contract = previous.contract

LATER = (
    "clear_observed",
    "unresolved",
    "no_future",
    "no_clear_observed",
    "current_not_blocked",
)
STATUSES = ("observed", "before_source", "after_source", "at_or_after_next_loss")
KINDS = (
    "audited",
    *(f"{k}_current_{s}" for k in ("raw", "mean") for s in STATES),
    *(k + "_left_censored" for k in ("raw", "mean")),
    "raw_new_blocked",
    "raw_persisting_blocked",
    "raw_away_blocked",
    *(f"{k}_later_{s}" for k in ("raw", "mean") for s in LATER),
)
REFS = tuple(
    f"{p}_{b}_history_{k}_refs" for p, b in product(PHASES, BRANCHES) for k in KINDS
)
HAS = tuple(
    "has_available_history_" + k
    for k in ("raw_new_blocked", "raw_persisting_blocked", "raw_later_clear")
)
FIELDS = (
    "history_observed_slots",
    "history_missing_slots",
    "history_before_return_slots",
    *(
        f"history_{k}_{side}_{s}_slots"
        for k in ("raw", "mean")
        for side in ("pre", "post")
        for s in (*STATES, "missing")
    ),
    *(
        f"history_{k}_{f}"
        for k in ("raw", "mean")
        for f in (
            "current_state",
            "run_count",
            "run_start_offset",
            "run_span_s",
            "left_censored",
            "predecessor_state",
            "later_status",
            "first_later_clear_offset_steps",
            "first_later_clear_delay_s",
            "first_later_clear_time_s",
        )
    ),
    "history_raw_new_blocked",
    "history_raw_persisting_blocked",
    "history_raw_away_blocked",
)
SLOT_FIELDS = (
    "logged_speed_margin",
    "margin_lo",
    "margin_hi",
    "window_count",
    "last_clear_step",
    "last_clear_reasons",
    *("input_" + k for k in RAW),
    *(
        f"{k}_{a}_{b}"
        for k in ("raw", "mean")
        for a in ("x", "y")
        for b in ("center", "lo", "hi")
    ),
    *(
        f"{k}_{q}_{b}"
        for k in ("raw", "mean")
        for q in ("distance", "gap")
        for b in ("center", "lo", "hi")
    ),
    *(f"{k}_state" for k in ("raw", "mean")),
    *(f"{k}_{t}_ids" for k in ("raw", "mean") for t in ("minimum", "possible_minimum")),
)


def plan_slots(selected, events):
    result = []
    for d in selected:
        event = events[d["event_id"]]
        origin = core.tick(d["time_s"])
        returned = core.tick(event["return_time_s"])
        loss = core.tick(event["end_time_s"]) if core.truth(event["ended"]) else 9001
        assert returned <= origin <= 9000 and origin < loss
        for offset in range(-25, 26):
            n = origin + offset
            status = "observed"
            if n < 1:
                status = "before_source"
            elif n > 9000:
                status = "after_source"
            elif offset > 0 and core.truth(event["ended"]) and n >= loss:
                status = "at_or_after_next_loss"
            context = ""
            if status == "observed":
                if n < returned:
                    context = "before_return"
                elif offset == 0:
                    context = "reference"
                else:
                    context = "available_context"
            row = {k: d[k] for k in (*META, "event_id")}
            row.update(
                reference_time_s=d["time_s"],
                offset_steps=offset,
                offset_s=offset / 50,
                source_step=n,
                source_time_s=n / 50,
                slot_status=status,
                context_phase=context,
                fixed_obstacle_ids=d["fixed_obstacle_ids"],
                raw_away_reference=d["raw_distance_direction"] == "away",
            )
            result.append(row)
    return result


def context_queries(planned):
    queries = defaultdict(list)
    mapping = []
    for s in planned:
        if s["slot_status"] != "observed":
            continue
        row = dict(
            row_id=len(mapping) + 1,
            case_id=s["case_id"],
            event_id=s["event_id"],
            reference_time_s=s["reference_time_s"],
            offset_steps=s["offset_steps"],
            source_time_s=s["source_time_s"],
        )
        queries[s["case_id"], s["source_step"]].append(row["row_id"])
        mapping.append(row)
    assert sum(map(len, queries.values())) == len(mapping)
    return queries, mapping


def instant(d, snapshot, obstacles, native=None):
    with localcontext() as ctx:
        ctx.prec = 60
        ids = previous.selected_ids(d)
        assert ";".join(ids) == d["fixed_obstacle_ids"]
        by_id = {oid: (x, y) for oid, x, y in obstacles}
        assert len(by_id) == len(obstacles) and set(ids) <= by_id.keys()
        text = snapshot["raw"]["speed_margin"]
        m = scalar(text)
        assert m.is_finite() and m >= 0
        out = dict(
            logged_speed_margin=text,
            margin_lo=m - H,
            margin_hi=m + H,
            window_count=len(snapshot["members"]),
            last_clear_step=snapshot["fields"]["window_last_clear_step"],
            last_clear_reasons=snapshot["fields"]["window_last_clear_reasons"],
        )
        for k in RAW:
            out["input_" + k] = snapshot["point"][k]
        witnesses = [
            dict(
                obstacle_order=j,
                obstacle_id=oid,
                obstacle_x=by_id[oid][0],
                obstacle_y=by_id[oid][1],
                logged_speed_margin=text,
            )
            for j, oid in enumerate(ids, 1)
        ]
        for kind in ("raw", "mean"):
            center = tuple(
                scalar(
                    snapshot["point"]["sensed_" + a + "_center"]
                    if kind == "raw"
                    else snapshot["fields"]["mean_" + a + "_center"]
                )
                for a in ("x", "y")
            )
            radius = R + (2 * P if kind == "mean" and snapshot["members"] else D(0))
            for axis, value in zip(("x", "y"), center):
                for bound, x in zip(
                    ("center", "lo", "hi"), (value, value - radius, value + radius)
                ):
                    out[f"{kind}_{axis}_{bound}"] = x
            distances = {
                oid: previous.distance_box(center, radius, by_id[oid]) for oid in ids
            }
            min_distance = tuple(
                min(distances[oid][i] for oid in ids) for i in range(3)
            )
            rows = [
                (out, min_distance, None),
                *(
                    (w, distances[w["obstacle_id"]], w["obstacle_id"])
                    for w in witnesses
                ),
            ]
            for row, distance, oid in rows:
                gap = previous.difference(distance, (m, m - H, m + H))
                for quantity, values in (("distance", distance), ("gap", gap)):
                    for i, b in enumerate(("center", "lo", "hi")):
                        row[f"{kind}_{quantity}_{b}"] = values[i]
                    if native is not None:
                        n = (
                            min(
                                native[j, kind][0 if quantity == "distance" else 1]
                                for j in ids
                            )
                            if oid is None
                            else native[oid, kind][0 if quantity == "distance" else 1]
                        )
                        assert values[1] <= n <= values[2], (
                            oid,
                            kind,
                            quantity,
                            values,
                            n,
                        )
                row[kind + "_state"] = previous.gap_state(*gap[1:])
            out[kind + "_minimum_ids"] = ";".join(
                oid for oid in ids if distances[oid][0] == min_distance[0]
            )
            out[kind + "_possible_minimum_ids"] = ";".join(
                oid for oid in ids if distances[oid][1] <= min_distance[2]
            )
        assert set(out) == set(SLOT_FIELDS)
        return out, witnesses


def check_previous(d, slot):
    offset = int(slot["offset_steps"])
    assert offset in (-1, 0) and slot["slot_status"] == "observed"
    role = "previous" if offset == -1 else "current"
    core.check(slot["logged_speed_margin"], d[f"point_{role}_margin"])
    core.check(
        str(slot["window_count"]),
        int(d["previous_count" if offset == -1 else "window_replayed_count"]),
    )
    for k in RAW:
        core.check(slot["input_" + k], d[role + "_input_" + k])
    for kind in ("raw", "mean"):
        for axis, bound in product(("x", "y"), ("center", "lo", "hi")):
            assert math.isclose(
                float(slot[f"{kind}_{axis}_{bound}"]),
                float(d[f"point_{kind}_{role}_{axis}_{bound}"]),
                rel_tol=1e-10,
                abs_tol=1e-9,
            )
            core.comparisons += 1
        for quantity, bound in product(("distance", "gap"), ("center", "lo", "hi")):
            assert math.isclose(
                float(slot[f"{kind}_{quantity}_{bound}"]),
                float(d[f"{kind}_{role}_{quantity}_{bound}"]),
                rel_tol=1e-10,
                abs_tol=1e-9,
            )
            core.comparisons += 1
        core.check(slot[kind + "_state"], d[f"{kind}_{role}_state"])
        for t in ("minimum", "possible_minimum"):
            core.check(slot[f"{kind}_{t}_ids"], d[f"{kind}_{role}_{t}_ids"])


def history(d, slots):
    ordered = sorted(slots, key=lambda row: int(row["offset_steps"]))
    assert [int(s["offset_steps"]) for s in ordered] == list(range(-25, 26))
    assert ordered[25]["slot_status"] == "observed"
    result = {
        "history_observed_slots": sum(s["slot_status"] == "observed" for s in ordered),
        "history_missing_slots": sum(s["slot_status"] != "observed" for s in ordered),
        "history_before_return_slots": sum(
            s["context_phase"] == "before_return" for s in ordered
        ),
    }
    for kind in ("raw", "mean"):
        for side, ss in (("pre", ordered[:25]), ("post", ordered[26:])):
            for state in (*STATES, "missing"):
                count = 0
                for s in ss:
                    observed = s["slot_status"] == "observed"
                    if (state == "missing" and not observed) or (
                        observed and s[kind + "_state"] == state
                    ):
                        count += 1
                result[f"history_{kind}_{side}_{state}_slots"] = count
        state = ordered[25][kind + "_state"]
        first = 25
        while first > 0:
            s = ordered[first - 1]
            if s["slot_status"] != "observed" or s[kind + "_state"] != state:
                break
            first -= 1
        left_censored = first == 0 or ordered[first - 1]["slot_status"] != "observed"
        predecessor = None if left_censored else ordered[first - 1][kind + "_state"]
        positive = [s for s in ordered[26:] if s["slot_status"] == "observed"]
        found = None
        if state != "blocked":
            status = "current_not_blocked"
        else:
            for s in positive:
                if s[kind + "_state"] == "clear":
                    found = s
                    break
            if found is not None:
                status = "clear_observed"
            elif any(s[kind + "_state"] == "unknown" for s in positive):
                status = "unresolved"
            elif not positive:
                status = "no_future"
            else:
                status = "no_clear_observed"
        count = 26 - first
        result.update(
            {
                f"history_{kind}_current_state": state,
                f"history_{kind}_run_count": count,
                f"history_{kind}_run_start_offset": first - 25,
                f"history_{kind}_run_span_s": (count - 1) / 50,
                f"history_{kind}_left_censored": left_censored,
                f"history_{kind}_predecessor_state": predecessor,
                f"history_{kind}_later_status": status,
                f"history_{kind}_first_later_clear_offset_steps": (
                    int(found["offset_steps"]) if found else None
                ),
                f"history_{kind}_first_later_clear_delay_s": (
                    int(found["offset_steps"]) / 50 if found else None
                ),
                f"history_{kind}_first_later_clear_time_s": (
                    found["source_time_s"] if found else None
                ),
            }
        )
    previous_state = (
        ordered[24]["raw_state"]
        if ordered[24]["slot_status"] == "observed"
        else "missing"
    )
    blocked = result["history_raw_current_state"] == "blocked"
    result.update(
        history_raw_new_blocked=blocked and previous_state == "clear",
        history_raw_persisting_blocked=blocked and previous_state == "blocked",
        history_raw_away_blocked=blocked and d["raw_distance_direction"] == "away",
    )
    assert set(result) == set(FIELDS)
    return result


def raw_inputs(cases, planned, raw, roots, work):
    files = defaultdict(dict)
    queries, query_rows = context_queries(planned)
    for c in cases:
        files[c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"][
            c["scenario"], c["planner"]
        ] = c
    assert set(files) == {(c, n) for c, s in raw.items() for n in s["raw_sha256"]}
    selected_cases = {c for c, n in queries}
    snapshots = {}
    audit = Counter()
    work.mkdir(parents=True)
    with (work / "native-replay-inputs.tsv").open("w") as wire:
        wire.write(
            "case_id\tstep\tpaused\tcorrection\treceived\tprecision\tspeed\tage\tx\ty\tyaw\tlogged_count\trow_ids\n"
        )
        for i, ((cohort, name), by_identity) in enumerate(sorted(files.items()), 1):
            path = Path(roots.get(cohort, raw[cohort]["path"])) / name
            assert core.sha(path) == raw[cohort]["raw_sha256"][name]
            audit["raw_bytes"] += path.stat().st_size
            last = defaultdict(lambda: -1)
            before = {}
            replays = {}
            with path.open() as f:
                header = next(f).rstrip("\r\n").split(",")
                assert len(header) == len(set(header))
                pos = {
                    k: header.index(k)
                    for k in (
                        "scenario",
                        "planner",
                        "time_s",
                        "effective_position_sigma",
                        "effective_yaw_sigma_rad",
                    )
                }
                for line in f:
                    assert '"' not in line
                    cells = line.rstrip("\r\n").split(",")
                    assert len(cells) == len(header)
                    identity = cells[pos["scenario"]], cells[pos["planner"]]
                    assert identity in by_identity
                    c = by_identity[identity]
                    n = core.tick(cells[pos["time_s"]])
                    assert n == last[identity] + 1
                    last[identity] = n
                    if not core.truth(c["selected_timeout"]):
                        continue
                    audit["timeout_grid_rows"] += 1
                    if n == 0:
                        vv = [
                            D(cells[pos[k]])
                            for k in (
                                "effective_position_sigma",
                                "effective_yaw_sigma_rad",
                            )
                        ]
                        assert all(v.is_finite() and v >= 0 for v in vv) and any(
                            v > 0 for v in vv
                        )
                        audit["noisy_branch_histories"] += 1
                    if c["case_id"] not in selected_cases:
                        continue
                    row = dict(zip(header, cells))
                    if n == 0:
                        replays[identity] = previous.previous.previous.Replay(row)
                        audit["replayed_histories"] += 1
                    else:
                        replay = replays[identity]
                        point, members, age, paused = replay.advance(
                            n, before[identity], row
                        )
                        audit["replayed_steps"] += 1
                        row_ids = queries.get((c["case_id"], n), [])
                        values = [
                            c["case_id"],
                            str(n),
                            str(paused).lower(),
                            row["correction"],
                            row["correction_received"],
                            row["precision_activation"],
                            format(float(row["measured_speed"]), ".17e"),
                            format(age, ".17e"),
                            *(
                                format(point["sensed_" + a + "_center"], ".17e")
                                for a in ("x", "y", "yaw")
                            ),
                            row["recovery_sample_count"],
                            ";".join(map(str, row_ids)),
                        ]
                        wire.write("\t".join(values) + "\n")
                        if row_ids:
                            snapshots[c["case_id"], n] = dict(
                                point=point,
                                members=list(members),
                                raw=dict(row),
                                clear_step=replay.clear_step,
                                reasons=replay.reasons,
                                pushes=len(replay.admissions),
                            )
                    before[identity] = row
            assert set(last) == set(by_identity)
            for identity, c in by_identity.items():
                assert last[identity] == (
                    9000
                    if core.truth(c["selected_timeout"])
                    else core.tick(c["time_s"])
                )
            audit["raw_files"] += 1
            if i % 18 == 0:
                print(f"Independent history contexts read: {i}/108 files", flush=True)
    assert snapshots.keys() == queries.keys()
    audit["unique_context_points"] = len(snapshots)
    return snapshots, query_rows, dict(audit)


def native_points(
    work, rustc, mapping, selected_map, contexts, window_traces, obstacles
):
    types = (REPO / previous.previous.previous.previous.standalone.TYPES).read_text()
    harness = "#![allow(dead_code)]\n#[derive(Clone,Copy)]\n" + core.block(
        types, "pub struct Point2D"
    )
    harness += (
        "\nimpl Point2D {\n"
        + core.block(types, "pub fn new(x: f64, y: f64)")
        + "\n"
        + core.block(types, "pub fn distance(&self, other: &Point2D)")
        + "\n}\n"
    )
    harness += r"""
fn main(){
 let args:Vec<_>=std::env::args().collect();
 println!("row_id,distance,gap");
 for line in std::fs::read_to_string(&args[1]).unwrap().lines().skip(1){
  let f:Vec<_>=line.split('\t').collect();
  let p=Point2D::new(f[7].parse().unwrap(),f[8].parse().unwrap());
  let o=Point2D::new(f[9].parse().unwrap(),f[10].parse().unwrap());
  let margin:f64=f[11].parse().unwrap();let d=p.distance(&o);
  println!("{},{:.17e},{:.17e}",f[0],d,d-margin);
 }
}
#[cfg(test)]mod tests{
 use super::*;
 #[test]fn pythagoras(){assert_eq!(Point2D::new(0.,0.).distance(&Point2D::new(3.,4.)),5.);}
 #[test]fn inclusive_contact(){let d=Point2D::new(0.,0.).distance(&Point2D::new(0.,1.));assert!(d-1.<=0.);}
 #[test]fn coincidence(){let p=Point2D::new(12.,-4.);assert_eq!(p.distance(&p),0.);}
 #[test]fn negative_coordinates(){assert_eq!(Point2D::new(-3.,-4.).distance(&Point2D::new(0.,0.)),5.);}
 #[test]fn margin_change(){let d=Point2D::new(0.,0.).distance(&Point2D::new(3.,4.));assert_eq!((d-6.)-(d-4.),-2.);}
 #[test]fn duplicate_coordinates(){let p=Point2D::new(2.,3.);let a=Point2D::new(3.,4.);let b=Point2D::new(3.,4.);assert_eq!(p.distance(&a),p.distance(&b));}
}
"""
    script = work / "point-geometry.rs"
    script.write_text(harness)
    for flags, name in ((["--test"], "point-tests"), ([], "point-trace")):
        subprocess.run(
            [
                str(rustc),
                "--edition=2021",
                "-O",
                *flags,
                str(script),
                "-o",
                str(work / name),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
    tests = subprocess.run(
        [str(work / "point-tests")], check=True, capture_output=True, text=True
    ).stdout
    assert "6 passed; 0 failed" in tests
    (work / "point-tests.txt").write_text(tests)
    queries = []
    keys = []
    for context_id, q in enumerate(mapping, 1):
        d = selected_map[q["event_id"], q["reference_time_s"]]
        omap = {oid: (x, y) for oid, x, y in obstacles[d["scenario"]]}
        snapshot = contexts[context_id]
        for kind in ("raw", "mean"):
            xy = [
                (
                    snapshot["point"]["sensed_" + a + "_center"]
                    if kind == "raw"
                    else window_traces[context_id]["mean_" + a]
                )
                for a in ("x", "y")
            ]
            for oid in previous.selected_ids(d):
                queries.append(
                    dict(
                        row_id=len(queries) + 1,
                        event_id=d["event_id"],
                        reference_time_s=d["time_s"],
                        offset_steps=q["offset_steps"],
                        source_time_s=q["source_time_s"],
                        point_kind=kind,
                        obstacle_id=oid,
                        x=f"{float(xy[0]):.17e}",
                        y=f"{float(xy[1]):.17e}",
                        obstacle_x=f"{float(omap[oid][0]):.17e}",
                        obstacle_y=f"{float(omap[oid][1]):.17e}",
                        margin=snapshot["raw"]["speed_margin"],
                    )
                )
                keys.append((context_id, oid, kind))
    query = work / "native-point-inputs.tsv"
    with query.open("w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=queries[0], delimiter="\t", lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(queries)
    text = subprocess.run(
        [str(work / "point-trace"), str(query)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    trace = work / "native-point.csv"
    trace.write_text(text)
    rows = core.read(trace)
    assert [int(r["row_id"]) for r in rows] == list(range(1, len(keys) + 1)) and len(
        set(keys)
    ) == len(keys)
    points = {
        key: (scalar(r["distance"]), scalar(r["gap"])) for key, r in zip(keys, rows)
    }
    return points, dict(
        harness_sha256=core.sha(script),
        inputs_sha256=core.sha(query),
        trace_sha256=core.sha(trace),
        native_tests=6,
        traced_point_obstacle_pairs=len(rows),
        method="original Point2D struct/new/distance; rounded raw centers and native reconstructed mean centers; fixed obstacle IDs; point gaps only",
    )


def anchor(e, dd, ne, nr):
    parent = int(e["parent_rp_records"])
    measures = dict(
        census_case_weight=Fraction(1, ne),
        received_case_weight=(
            Fraction(1, nr) if core.truth(e["received"]) else Fraction()
        ),
        rp_case_mass=Fraction(int(e["rp_records"]), parent) if parent else Fraction(),
        rp_different_preview_case_mass=(
            Fraction(int(e["rp_different_preview_records"]), parent)
            if parent
            else Fraction()
        ),
    )
    for k, v in measures.items():
        core.check(e[k], v)
    out = dict(e, **measures, **dict.fromkeys((*REFS, *HAS)))
    if not core.truth(e["returned"] or "False"):
        assert not dd
        return out
    assert all(all(d[k] == e[k] for k in META) for d in dd)
    for p, b in product(PHASES, BRANCHES):
        rows = [
            d
            for d in dd
            if d["phase"] == p and d["branch"] == b and d["history_audited"]
        ]
        counts = {k: 0 for k in KINDS}
        counts["audited"] = len(rows)
        for kind in ("raw", "mean"):
            for s in STATES:
                counts[f"{kind}_current_{s}"] = sum(
                    d[f"history_{kind}_current_state"] == s for d in rows
                )
            for s in LATER:
                counts[f"{kind}_later_{s}"] = sum(
                    d[f"history_{kind}_later_status"] == s for d in rows
                )
            counts[kind + "_left_censored"] = sum(
                d[f"history_{kind}_left_censored"] for d in rows
            )
        for k in ("raw_new_blocked", "raw_persisting_blocked", "raw_away_blocked"):
            counts[k] = sum(d["history_" + k] for d in rows)
        out.update({f"{p}_{b}_history_{k}_refs": v for k, v in counts.items()})
    available = [d for d in dd if d["phase"] == "available" and d["history_audited"]]
    for k in ("raw_new_blocked", "raw_persisting_blocked"):
        out["has_available_history_" + k] = any(d["history_" + k] for d in available)
    out["has_available_history_raw_later_clear"] = any(
        d["history_raw_later_status"] == "clear_observed" for d in available
    )
    return out


def closure(rows):
    checks = 0
    for key, r in rows.items():
        for i, whole, parts in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", CATEGORIES),
        ):
            if key[i] == whole:
                for f in (
                    *REFS,
                    *(h + s for h in HAS for s in ("_events", "_case_mass")),
                ):
                    assert r[f] == sum(
                        rows[key[:i] + (p,) + key[i + 1 :]][f] for p in parts
                    )
                    checks += 1
        for p, b in product(PHASES, BRANCHES):
            prefix = f"{p}_{b}_history_"
            for group in (
                *(tuple(k + "_current_" + s for s in STATES) for k in ("raw", "mean")),
                *(tuple(k + "_later_" + s for s in LATER) for k in ("raw", "mean")),
            ):
                assert (
                    sum(r[prefix + k + "_refs"] for k in group)
                    == r[prefix + "audited_refs"]
                )
    return checks


def compare(path, rows, keys):
    expected = {tuple(str(r[k]) for k in keys): r for r in rows}
    actual = core.index(core.read(path), keys)
    assert len(expected) == len(rows) and actual.keys() == expected.keys()
    for key, row in actual.items():
        assert set(row) == set(expected[key])
        for field, value in expected[key].items():
            if isinstance(value, (D, float)):
                assert math.isclose(
                    float(row[field]), float(value), rel_tol=1e-10, abs_tol=1e-9
                ), (key, field, row[field], value)
                core.comparisons += 1
            else:
                core.check(row[field], value)


def summaries(old, anchors):
    result = {}
    for r in old:
        key = tuple(r[k] for k in GROUP)
        p, n, arm, mode, cat = key
        aa = [
            a
            for a in anchors
            if (a["partition"], a["noise"], a["variant"]) == (p, n, arm)
            and (mode == "all_modes" or a["mode"] == mode)
            and (cat == "all" or a["anchor_category"] == cat)
        ]
        rr = [a for a in aa if core.truth(a["returned"] or "False")]
        mass = lambda rows: sum((a["census_case_weight"] for a in rows), Fraction())
        out = dict(r)
        for field in REFS:
            out[field] = sum(a[field] or 0 for a in aa)
        for h in HAS:
            selected = [a for a in aa if a[h]]
            out.update(
                {
                    h + "_events": len(selected),
                    h + "_case_mass": mass(selected),
                    h + "_fraction": Fraction(len(selected), len(rr)) if rr else None,
                    h
                    + "_case_mass_fraction": mass(selected) / mass(rr) if rr else None,
                }
            )
        core.check(r["census_case_mass"], mass(aa))
        assert key not in result
        result[key] = out
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for n in ("results", "figures", "work", "output"):
        p.add_argument("--" + n, type=Path, required=True)
    p.add_argument("--rustc", type=Path, default=shutil.which("rustc"))
    p.add_argument("--raw-roots", type=Path)
    args = p.parse_args()
    if not __debug__ or args.work.exists() or args.output.exists() or not args.rustc:
        p.error("Use new work/output paths, provide rustc and run without -O")
    prior = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        core.sha(BASE / "sources.json") == BASE_SHA
        and core.sha(BASE / "independent-verification.json") == PROOF_SHA
    )
    freeze = json.loads((STUDY / "freeze-pins.json").read_text())
    assert freeze == dict(
        base_main="9cc0426a527c3b200a0c7312e9d964a7b06932ae",
        sources_sha256=BASE_SHA,
        proof_sha256=PROOF_SHA,
        verifier_helper_sha256=HELPERS["verify_point_clearance.py"],
    )
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        **{
            "fusion-return-point-clearance/" + n: h
            for n, h in prior["output_sha256"].items()
        },
        "fusion-return-point-clearance/sources.json": BASE_SHA,
        "fusion-return-point-clearance/independent-verification.json": PROOF_SHA,
        "fusion-return-point-clearance/protocol.md": prior["analysis"][
            "protocol_sha256"
        ],
        "fusion-return-point-clearance/freeze-pins.json": prior["analysis"][
            "freeze_sha256"
        ],
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_point_clearance.py": prior["analysis"]["script_sha256"],
    }
    assert (
        source["analysis"]["verifier_helpers_sha256"] == HELPERS
        and source["analysis"]["code_sha256"] == CODE
    )
    assert (
        source["raw"] == prior["raw"]
        and source["map_sha256"] == prior["map_sha256"]
        and source["inherited_point_clearance_proof_sha256"] == PROOF_SHA
        and source["inherited_validation"] == prior["inherited_validation"]
    )
    assert source["schema"] == dict(
        half_unit=5e-10,
        elemental_allowance=1e-12,
        sensed_radius=3 * 5e-10 + 2 * 1e-12,
        offset_steps=list(range(-25, 26)),
        slot_statuses=list(STATUSES),
        states=list(STATES),
        later_statuses=list(LATER),
        phases=list(PHASES),
        branches=list(BRANCHES),
        retrospective=True,
        inherited="all prior columns/outcomes/weights; original 25 only; historical context and future strictly before next loss; point gaps do not replace controller predicates",
    )
    for n, h in source["input_sha256"].items():
        assert core.sha(RESULTS / n) == h, n
    for n, h in {**source["analysis"]["helpers_sha256"], **HELPERS}.items():
        assert core.sha(ROOT / n) == h, n
    for n, h in CODE.items():
        assert core.sha(REPO / n) == h, n
    assert source["analysis"]["source_contract"] == contract(
        (REPO / next(iter(core.CODE))).read_text()
    )
    for f, path in (
        ("script_sha256", ROOT / "analyze_point_history.py"),
        ("protocol_sha256", STUDY / "protocol.md"),
        ("freeze_sha256", STUDY / "freeze-pins.json"),
    ):
        assert source["analysis"][f] == core.sha(path)
    assert set(source["output_sha256"]) == {
        "history-decisions.csv",
        "history-slots.csv",
        "history-witnesses.csv",
        "anchor-history.csv",
        "history-summary.csv",
    }
    for n, h in source["output_sha256"].items():
        assert core.sha(args.results / n) == h, n
    compiler = subprocess.run(
        [str(args.rustc), "--version"], check=True, capture_output=True, text=True
    ).stdout.strip()
    assert compiler == proof["native"]["rustc"]
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    for cohort, spec in source["raw"].items():
        path = Path(roots.get(cohort, spec["path"])) / "provenance.json"
        assert core.sha(path) == spec["provenance_sha256"]
        record = json.loads(path.read_text())
        assert (
            record["rustc"] == compiler
            and record["inputs_sha256"]["obstacles.csv"] == source["map_sha256"]
        )
        assert set(record["inputs_by_variant_sha256"]) == {"epoch", "curved"} and all(
            v["obstacles.csv"] == source["map_sha256"]
            for v in record["inputs_by_variant_sha256"].values()
        )
    cases, original = core.original_inputs()
    events = core.read(BASE / "anchor-point.csv")
    assert {e["event_id"] for e in events} == original.keys()
    for e in events:
        for k in (
            *META,
            "source_event_id",
            "event_kind",
            "event_time_s",
            "scheduled",
            "received",
            "precision_activation",
            "horizon",
            "parent_rp_records",
            "rp_records",
            "rp_different_preview_records",
        ):
            core.check(e[k], original[e["event_id"]][k])
    original_checks = core.comparisons
    old = core.read(BASE / "point-decisions.csv")
    selected = [d for d in old if d["prior_certainty"] == "unknown"]
    assert len(selected) == 25 and all(
        d["phase"] == "available" and d["branch"] == "joint" for d in selected
    )
    planned = plan_slots(selected, {e["event_id"]: e for e in events})
    snapshots, query_rows, audit = raw_inputs(
        cases, planned, source["raw"], roots, args.work
    )
    queries = [
        dict(
            case_id=q["case_id"], event_id=q["event_id"], time_s=str(q["source_time_s"])
        )
        for q in query_rows
    ]
    traces, native_proof = previous.previous.previous.native(
        args.work, args.rustc, queries
    )
    assert native_proof["replayed_steps"] == audit["replayed_steps"]
    native_proof["traced_context_queries"] = native_proof.pop("traced_references")
    context_map = args.work / "native-contexts.csv"
    with context_map.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=query_rows[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(query_rows)
    native_proof["context_map_sha256"] = core.sha(context_map)
    contexts = {}
    for q in query_rows:
        key = (q["case_id"], core.tick(q["source_time_s"]))
        snap = snapshots[key]
        trace = traces[q["row_id"]]
        assert int(trace["step"]) == key[1]
        replay = SimpleNamespace(
            clear_step=snap["clear_step"],
            reasons=snap["reasons"],
            admissions=[None] * snap["pushes"],
        )
        fields, _ = previous.previous.previous.reconstruct(
            replay, snap["point"], snap["members"], snap["raw"], trace
        )
        contexts[q["row_id"]] = dict(snap, fields=fields)
    obstacles = previous.previous.previous.previous.standalone.read_map(
        RESULTS / "fusion-return-probe-origin/obstacle-map.csv"
    )
    selected_map = {(d["event_id"], d["time_s"]): d for d in selected}
    native_geometry, geometry_proof = native_points(
        args.work, args.rustc, query_rows, selected_map, contexts, traces, obstacles
    )
    native_proof["point_geometry"] = geometry_proof
    query_ids = {
        (q["event_id"], q["reference_time_s"], q["offset_steps"]): q["row_id"]
        for q in query_rows
    }
    slots = []
    witnesses = []
    by_reference = defaultdict(list)
    for s in planned:
        d = selected_map[s["event_id"], s["reference_time_s"]]
        row = dict(s, **dict.fromkeys(SLOT_FIELDS))
        if s["slot_status"] == "observed":
            context_id = query_ids[
                s["event_id"], s["reference_time_s"], s["offset_steps"]
            ]
            native = {
                (oid, k): native_geometry[context_id, oid, k]
                for oid in previous.selected_ids(d)
                for k in ("raw", "mean")
            }
            fields, ww = instant(
                d, contexts[context_id], obstacles[d["scenario"]], native
            )
            row.update(fields)
            if s["offset_steps"] in (-1, 0):
                check_previous(d, row)
            witnesses.extend(dict(s, **w) for w in ww)
        slots.append(row)
        by_reference[s["event_id"], s["reference_time_s"]].append(row)
    decisions = []
    for d in old:
        audited = (d["event_id"], d["time_s"]) in selected_map
        out = dict(d, history_audited=audited, **dict.fromkeys(FIELDS))
        if audited:
            out.update(history(d, by_reference[d["event_id"], d["time_s"]]))
        decisions.append(out)
    db = defaultdict(list)
    for d in decisions:
        db[d["event_id"]].append(d)
    ne = Counter(e["case_id"] for e in original.values())
    nr = Counter(e["case_id"] for e in original.values() if core.truth(e["received"]))
    anchors = [
        anchor(e, db[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]]) for e in events
    ]
    summary = summaries(core.read(BASE / "point-summary.csv"), anchors)
    for name, rows, keys in (
        ("history-decisions", decisions, ("event_id", "time_s")),
        ("history-slots", slots, ("event_id", "reference_time_s", "offset_steps")),
        (
            "history-witnesses",
            witnesses,
            ("event_id", "reference_time_s", "offset_steps", "obstacle_id"),
        ),
        ("anchor-history", anchors, ("event_id",)),
        ("history-summary", list(summary.values()), GROUP),
    ):
        compare(args.results / (name + ".csv"), rows, keys)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(core.truth(e["returned"] or "False") for e in events),
        decision_references=len(decisions),
        selected_references=len(selected),
        inherited_zero_probes=1096,
        planned_slots=len(slots),
        context_queries=sum(s["slot_status"] == "observed" for s in slots),
        missing_slots=sum(s["slot_status"] != "observed" for s in slots),
        witness_rows=len(witnesses),
        summary_rows=len(summary),
        new_additive_closure_comparisons=closure(summary),
    )
    assert audit == source["audit"], (audit, source["audit"])
    assert (
        sum(a["census_case_weight"] for a in anchors)
        == sum(a["received_case_weight"] for a in anchors)
        == 570
    )
    assert (
        sum(a["rp_case_mass"] for a in anchors) == 157
        and sum(int(a["rp_records"]) for a in anchors) == 10995
    )
    plots = json.loads((args.figures / "plot-sources.json").read_text())
    assert plots["script_sha256"] == core.sha(ROOT / "plot_point_history.py")
    assert plots["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in (
            "sources.json",
            "anchor-history.csv",
            "history-summary.csv",
            "history-slots.csv",
        )
    }
    assert set(plots["output_sha256"]) == {
        "point-history.svg",
        "plot-values.csv",
        "plot-return-values.csv",
        "plot-history.csv",
    }
    for n, h in plots["output_sha256"].items():
        assert core.sha(args.figures / n) == h
    compare(
        args.figures / "plot-values.csv",
        [r for r in summary.values() if r["mode"] != "all_modes"],
        GROUP,
    )
    compare(
        args.figures / "plot-return-values.csv",
        [a for a in anchors if core.truth(a["returned"] or "False")],
        ("event_id",),
    )
    compare(
        args.figures / "plot-history.csv",
        slots,
        ("event_id", "reference_time_s", "offset_steps"),
    )
    evidence = dict(
        **audit,
        comparisons=core.comparisons - original_checks,
        original_join_comparisons=original_checks,
        plotted_summary_rows=252,
        plotted_history_slots=len(slots),
        plotted_return_rows=137,
        decimal_precision=60,
        native=native_proof,
        inherited_point_clearance_proof_sha256=PROOF_SHA,
        script_sha256=core.sha(Path(__file__)),
        sources_sha256=core.sha(args.results / "sources.json"),
        plot_sources_sha256=core.sha(args.figures / "plot-sources.json"),
        method="independent slot planning/run censorship/first certain later-clear and original aggregation; pinned Decimal point/window helpers and original Rust traces; historical context and future strictly before next loss; no new controller outcomes",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
