#!/usr/bin/env python3
"""Independently check fixed-obstacle point gaps with Decimal and Rust."""

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

import verify_window_turnover as previous

core = previous.core
ROOT, REPO, RESULTS = core.ROOT, core.REPO, core.RESULTS
BASE = RESULTS / "fusion-return-window-turnover"
STUDY = RESULTS / "fusion-return-point-clearance"
BASE_SHA = "ce946dad8570b16cc0ca12a8f2d41c526801ec560b620c386fda8e9ed9688f9e"
PROOF_SHA = "f9bd4b69aaf1aee2c2b1ba187b8a20704957ea30611aa548dce92c74a7cc6bb3"
META, KEY, GROUP = previous.META, previous.KEY, previous.GROUP
PHASES, BRANCHES, CATEGORIES = previous.PHASES, previous.BRANCHES, previous.CATEGORIES
H, P = D("5e-10"), D("1e-12")
R = 3 * H + 2 * P
CODE = previous.CODE
HELPERS = {
    **previous.HELPERS,
    "verify_window_turnover.py": "7c4c72644c85a9202940ff407a0422f73e1815bdb7991075c261037108695181",
}

STATES = ("clear", "blocked", "unknown")
TRANSITIONS = tuple(a + "_to_" + b for a, b in product(STATES, repeat=2))
DIRECTIONS = ("toward", "away", "unresolved")
GAP_DIRECTIONS = ("narrower", "wider", "unresolved")
MARGIN_DIRECTIONS = ("decrease", "increase", "unresolved")
KINDS = (
    "audited",
    *(k + "_" + t for k in ("raw", "mean") for t in TRANSITIONS),
    *(k + "_distance_" + t for k in ("raw", "mean") for t in DIRECTIONS),
    *(k + "_gap_" + t for k in ("raw", "mean") for t in GAP_DIRECTIONS),
    *("margin_" + t for t in MARGIN_DIRECTIONS),
    "prior_unknown_raw_narrower",
    "nominal_only_reset_raw_narrower",
)
REFS = tuple(
    f"{p}_{b}_point_{k}_refs" for p, b in product(PHASES, BRANCHES) for k in KINDS
)
HAS = tuple(
    "has_available_point_" + k
    for k in ("raw_clear_to_blocked", "mean_clear_to_blocked", "raw_toward_mean_away")
)
METRICS = (
    *(
        f"{k}_{c}_{q}_{b}"
        for k in ("raw", "mean")
        for c in ("previous", "current")
        for q in ("distance", "gap")
        for b in ("center", "lo", "hi")
    ),
    *(f"{k}_{c}_state" for k in ("raw", "mean") for c in ("previous", "current")),
    *(f"{k}_transition" for k in ("raw", "mean")),
    *(
        f"{k}_{q}_change_{b}"
        for k in ("raw", "mean")
        for q in ("distance", "gap")
        for b in ("center", "lo", "hi")
    ),
    *(f"{k}_{q}_direction" for k in ("raw", "mean") for q in ("distance", "gap")),
    *(f"{k}_gap_closure_residual" for k in ("raw", "mean")),
)
FIELDS = (
    "fixed_obstacle_ids",
    "fixed_obstacle_count",
    *(
        f"point_{c}_margin{b}"
        for c in ("previous", "current")
        for b in ("", "_lo", "_hi")
    ),
    *("point_margin_change_" + b for b in ("center", "lo", "hi")),
    "point_margin_direction",
    *(
        f"point_{k}_{c}_{a}_{b}"
        for k in ("raw", "mean")
        for c in ("previous", "current")
        for a in ("x", "y")
        for b in ("center", "lo", "hi")
    ),
    *METRICS,
    *(
        f"{k}_{c}_{t}_ids"
        for k in ("raw", "mean")
        for c in ("previous", "current")
        for t in ("minimum", "possible_minimum")
    ),
)


def contract(source):
    return dict(
        window=previous.previous.contract(source),
        margin=previous.previous.previous.contract(source),
    )


def scalar(value):
    return value if isinstance(value, D) else D.from_float(float(value))


def selected_ids(d):
    if d["probe_origin"] == "initial_unsafe":
        text, walls = d["definite_obstacle_ids"], d["definite_wall_ids"]
    else:
        assert (
            d["probe_origin"] == "first_chord_by_exclusion"
            and d["direct_first_test_state"] == "unsafe"
        )
        text, walls = d["first_definite_obstacle_ids"], d["first_definite_wall_ids"]
    ids = text.split(";")
    assert text and not walls and len(ids) == len(set(ids))
    return ids


def signed(pair, negative, positive):
    if pair[1] < 0:
        return negative
    if pair[0] > 0:
        return positive
    return "unresolved"


def gap_state(lo, hi):
    if lo > 0:
        return "clear"
    if hi <= 0:
        return "blocked"
    return "unknown"


def difference(a, b):
    return a[0] - b[0], a[1] - b[2] - P, a[2] - b[1] + P


def distance_box(center, radius, obstacle):
    with localcontext() as ctx:
        ctx.prec = 60
        squares = []
        low_squares = []
        high_squares = []
        for value, coordinate in zip(center, obstacle):
            delta = scalar(value) - scalar(coordinate)
            lower, upper = delta - radius - P, delta + radius + P
            squares.append(delta * delta)
            low_squares.append(
                D(0) if lower <= 0 <= upper else min(lower * lower, upper * upper)
            )
            high_squares.append(max(lower * lower, upper * upper))
        return (
            sum(squares, D(0)).sqrt(),
            max(D(0), sum(low_squares, D(0)).sqrt() - P),
            sum(high_squares, D(0)).sqrt() + P,
        )


def metrics(distances, margins, native=None):
    result = {}
    change_margin = difference(margins["current"], margins["previous"])
    for kind in ("raw", "mean"):
        gaps = {
            context: difference(distances[kind, context], margins[context])
            for context in ("previous", "current")
        }
        for context in ("previous", "current"):
            for q, value in (
                ("distance", distances[kind, context]),
                ("gap", gaps[context]),
            ):
                for index, bound in enumerate(("center", "lo", "hi")):
                    result[f"{kind}_{context}_{q}_{bound}"] = value[index]
                if native is not None:
                    n = native[kind, context][0 if q == "distance" else 1]
                    assert value[1] <= n <= value[2], (kind, context, q, value, n)
            result[f"{kind}_{context}_state"] = gap_state(*gaps[context][1:])
        result[kind + "_transition"] = (
            result[kind + "_previous_state"] + "_to_" + result[kind + "_current_state"]
        )
        change_distance = difference(
            distances[kind, "current"], distances[kind, "previous"]
        )
        change_gap = difference(gaps["current"], gaps["previous"])
        prediction = difference(change_distance, change_margin)
        residual = change_gap[0] - prediction[0]
        assert abs(residual) <= P
        assert max(prediction[1], change_gap[1]) <= min(prediction[2], change_gap[2])
        result[kind + "_gap_closure_residual"] = residual
        for q, change, labels in (
            ("distance", change_distance, ("toward", "away")),
            ("gap", change_gap, ("narrower", "wider")),
        ):
            for i, b in enumerate(("center", "lo", "hi")):
                result[f"{kind}_{q}_change_{b}"] = change[i]
            result[f"{kind}_{q}_direction"] = signed(change[1:], *labels)
            if native is not None:
                i = 0 if q == "distance" else 1
                native_change = native[kind, "current"][i] - native[kind, "previous"][i]
                assert change[1] <= native_change <= change[2]
                if q == "gap":
                    assert prediction[1] <= native_change <= prediction[2]
    assert set(result) == set(METRICS)
    return result


def check_contexts(d, before, after):
    previous.check_current(d, after)
    core.check(d["previous_count"], len(before["members"]))
    for axis in ("x", "y"):
        for name, value in (
            ("mean", before["fields"]["mean_" + axis + "_center"]),
            ("sensed", before["point"]["sensed_" + axis + "_center"]),
        ):
            core.check(d[f"previous_{name}_{axis}_center"], float(value))
    for role, snap in (("previous", before), ("current", after)):
        for k in previous.RAW:
            core.check(d[role + "_input_" + k], snap["point"][k])


def geometry(d, before, after, obstacles, native=None):
    with localcontext() as ctx:
        ctx.prec = 60
        ids = selected_ids(d)
        omap = {oid: (x, y) for oid, x, y in obstacles}
        assert len(omap) == len(obstacles) and set(ids) <= omap.keys()
        out = dict(fixed_obstacle_ids=";".join(ids), fixed_obstacle_count=len(ids))
        centers, margins = {}, {}
        for role, snapshot in (("previous", before), ("current", after)):
            text = snapshot["raw"]["speed_margin"]
            value = scalar(text)
            assert value.is_finite() and value >= 0
            margins[role] = (value, value - H, value + H)
            out.update(
                {
                    f"point_{role}_margin": text,
                    f"point_{role}_margin_lo": value - H,
                    f"point_{role}_margin_hi": value + H,
                }
            )
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
                centers[kind, role] = (center, radius)
                for a, value in zip(("x", "y"), center):
                    for b, x in zip(
                        ("center", "lo", "hi"), (value, value - radius, value + radius)
                    ):
                        out[f"point_{kind}_{role}_{a}_{b}"] = x
        change = difference(margins["current"], margins["previous"])
        for b, x in zip(("center", "lo", "hi"), change):
            out["point_margin_change_" + b] = x
        out["point_margin_direction"] = signed(change[1:], "decrease", "increase")
        distances = {}
        witness_rows = []
        for order, oid in enumerate(ids, 1):
            values = {
                key: distance_box(center, radius, omap[oid])
                for key, (center, radius) in centers.items()
            }
            distances[oid] = values
            witness_rows.append(
                dict(
                    **{
                        k: d[k]
                        for k in (*META, "event_id", "time_s", "phase", "branch")
                    },
                    obstacle_order=order,
                    obstacle_id=oid,
                    obstacle_x=omap[oid][0],
                    obstacle_y=omap[oid][1],
                    **metrics(values, margins, native[oid] if native else None),
                )
            )
        minima = {}
        for kind, role in centers:
            minimum = tuple(
                min(distances[i][kind, role][j] for i in ids) for j in range(3)
            )
            minima[kind, role] = minimum
            out[f"{kind}_{role}_minimum_ids"] = ";".join(
                i for i in ids if distances[i][kind, role][0] == minimum[0]
            )
            out[f"{kind}_{role}_possible_minimum_ids"] = ";".join(
                i for i in ids if distances[i][kind, role][1] <= minimum[2]
            )
        native_min = None
        if native:
            native_min = {
                key: tuple(min(native[i][key][j] for i in ids) for j in (0, 1))
                for key in centers
            }
        out.update(metrics(minima, margins, native_min))
        assert set(out) == set(FIELDS)
        return out, witness_rows


def native_points(work, rustc, selected, contexts, window_traces, obstacles):
    types = (REPO / previous.previous.previous.standalone.TYPES).read_text()
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
    for i, d in enumerate(selected):
        omap = {oid: (x, y) for oid, x, y in obstacles[d["scenario"]]}
        for role, context_id in (("previous", 2 * i + 1), ("current", 2 * i + 2)):
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
                for oid in selected_ids(d):
                    queries.append(
                        dict(
                            row_id=len(queries) + 1,
                            event_id=d["event_id"],
                            reference_time_s=d["time_s"],
                            source_time_s=snapshot["point"]["source_step"] / 50,
                            context_role=role,
                            point_kind=kind,
                            obstacle_id=oid,
                            x=f"{float(xy[0]):.17e}",
                            y=f"{float(xy[1]):.17e}",
                            obstacle_x=f"{float(omap[oid][0]):.17e}",
                            obstacle_y=f"{float(omap[oid][1]):.17e}",
                            margin=snapshot["raw"]["speed_margin"],
                        )
                    )
                    keys.append((d["event_id"], d["time_s"], oid, kind, role))
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
            if d["phase"] == p and d["branch"] == b and d["point_clearance_audited"]
        ]
        values = {k: 0 for k in KINDS}
        values["audited"] = len(rows)
        for kind in ("raw", "mean"):
            for t in TRANSITIONS:
                values[kind + "_" + t] = sum(d[kind + "_transition"] == t for d in rows)
            for q, labels in (("distance", DIRECTIONS), ("gap", GAP_DIRECTIONS)):
                for t in labels:
                    values[kind + "_" + q + "_" + t] = sum(
                        d[kind + "_" + q + "_direction"] == t for d in rows
                    )
        for t in MARGIN_DIRECTIONS:
            values["margin_" + t] = sum(d["point_margin_direction"] == t for d in rows)
        values["prior_unknown_raw_narrower"] = sum(
            d["prior_certainty"] == "unknown" and d["raw_gap_direction"] == "narrower"
            for d in rows
        )
        values["nominal_only_reset_raw_narrower"] = sum(
            d["timer_action"] == "reset"
            and d["refined_failed_gates"] == "nominal"
            and d["raw_gap_direction"] == "narrower"
            for d in rows
        )
        out.update({f"{p}_{b}_point_{k}_refs": value for k, value in values.items()})
    available = [
        d for d in dd if d["phase"] == "available" and d["point_clearance_audited"]
    ]
    for k in ("raw", "mean"):
        out[f"has_available_point_{k}_clear_to_blocked"] = any(
            d[k + "_transition"] == "clear_to_blocked" for d in available
        )
    out["has_available_point_raw_toward_mean_away"] = any(
        d["raw_distance_direction"] == "toward"
        and d["mean_distance_direction"] == "away"
        for d in available
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
            prefix = f"{p}_{b}_point_"
            for group in (
                *(tuple(k + "_" + t for t in TRANSITIONS) for k in ("raw", "mean")),
                *(
                    tuple(k + "_distance_" + t for t in DIRECTIONS)
                    for k in ("raw", "mean")
                ),
                *(
                    tuple(k + "_gap_" + t for t in GAP_DIRECTIONS)
                    for k in ("raw", "mean")
                ),
                tuple("margin_" + t for t in MARGIN_DIRECTIONS),
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
            if isinstance(value, D) or (field in FIELDS and isinstance(value, float)):
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
        base_main="9941ce31dcfacd75aa948c0041965565ff1d0bd6",
        sources_sha256=BASE_SHA,
        proof_sha256=PROOF_SHA,
        verifier_helper_sha256=HELPERS["verify_window_turnover.py"],
    )
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        **{
            "fusion-return-window-turnover/" + n: h
            for n, h in prior["output_sha256"].items()
        },
        "fusion-return-window-turnover/sources.json": BASE_SHA,
        "fusion-return-window-turnover/independent-verification.json": PROOF_SHA,
        "fusion-return-window-turnover/protocol.md": prior["analysis"][
            "protocol_sha256"
        ],
        "fusion-return-window-turnover/freeze-pins.json": prior["analysis"][
            "freeze_sha256"
        ],
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_window_turnover.py": prior["analysis"]["script_sha256"],
    }
    assert (
        source["analysis"]["verifier_helpers_sha256"] == HELPERS
        and source["analysis"]["code_sha256"] == CODE
    )
    assert (
        source["raw"] == prior["raw"]
        and source["map_sha256"] == prior["map_sha256"]
        and source["inherited_window_turnover_proof_sha256"] == PROOF_SHA
        and source["inherited_validation"] == prior["inherited_validation"]
    )
    assert source["schema"] == dict(
        half_unit=5e-10,
        elemental_allowance=1e-12,
        sensed_radius=3 * 5e-10 + 2 * 1e-12,
        closure_allowance=1e-12,
        states=list(STATES),
        transitions=list(TRANSITIONS),
        distance_directions=list(DIRECTIONS),
        gap_directions=list(GAP_DIRECTIONS),
        margin_directions=list(MARGIN_DIRECTIONS),
        phases=list(PHASES),
        branches=list(BRANCHES),
        retrospective=True,
        inherited="all prior columns and outcomes; fixed original blocker sets; predecessor context only; point gaps do not replace chord/segment/probe predicates",
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
        ("script_sha256", ROOT / "analyze_point_clearance.py"),
        ("protocol_sha256", STUDY / "protocol.md"),
        ("freeze_sha256", STUDY / "freeze-pins.json"),
    ):
        assert source["analysis"][f] == core.sha(path)
    assert set(source["output_sha256"]) == {
        "point-decisions.csv",
        "point-witnesses.csv",
        "anchor-point.csv",
        "point-summary.csv",
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
    events = core.read(BASE / "anchor-turnover.csv")
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
    old = core.read(BASE / "turnover-decisions.csv")
    selected = [d for d in old if D(d["speed_free_distance"]) == 0]
    assert len(selected) == 1096
    snapshots, query_rows, audit = previous.raw_inputs(
        cases, selected, source["raw"], roots, args.work
    )
    queries = [
        dict(
            case_id=q["case_id"], event_id=q["event_id"], time_s=str(q["source_time_s"])
        )
        for q in query_rows
    ]
    traces, native_proof = previous.previous.native(args.work, args.rustc, queries)
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
        fields, _ = previous.previous.reconstruct(
            replay, snap["point"], snap["members"], snap["raw"], trace
        )
        contexts[q["row_id"]] = dict(snap, fields=fields)
    obstacles = previous.previous.previous.standalone.read_map(
        RESULTS / "fusion-return-probe-origin/obstacle-map.csv"
    )
    native_geometry, geometry_proof = native_points(
        args.work, args.rustc, selected, contexts, traces, obstacles
    )
    native_proof["point_geometry"] = geometry_proof
    decisions = []
    witness_rows = []
    selected_index = {(d["event_id"], d["time_s"]): i for i, d in enumerate(selected)}
    for d in old:
        audited = D(d["speed_free_distance"]) == 0
        out = dict(d, point_clearance_audited=audited, **dict.fromkeys(FIELDS))
        if audited:
            i = selected_index[d["event_id"], d["time_s"]]
            a, b = 2 * i + 1, 2 * i + 2
            before, after = contexts[a], contexts[b]
            check_contexts(d, before, after)
            native = {
                oid: {
                    (kind, role): native_geometry[
                        d["event_id"], d["time_s"], oid, kind, role
                    ]
                    for kind, role in product(("raw", "mean"), ("previous", "current"))
                }
                for oid in selected_ids(d)
            }
            fields, lines = geometry(d, before, after, obstacles[d["scenario"]], native)
            out.update(fields)
            witness_rows.extend(lines)
        decisions.append(out)
    db = defaultdict(list)
    for d in decisions:
        db[d["event_id"]].append(d)
    ne = Counter(e["case_id"] for e in original.values())
    nr = Counter(e["case_id"] for e in original.values() if core.truth(e["received"]))
    anchors = [
        anchor(e, db[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]]) for e in events
    ]
    summary = summaries(core.read(BASE / "turnover-summary.csv"), anchors)
    for name, rows, keys in (
        ("point-decisions", decisions, ("event_id", "time_s")),
        ("point-witnesses", witness_rows, ("event_id", "time_s", "obstacle_id")),
        ("anchor-point", anchors, ("event_id",)),
        ("point-summary", list(summary.values()), GROUP),
    ):
        compare(args.results / (name + ".csv"), rows, keys)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(core.truth(e["returned"] or "False") for e in events),
        decision_references=len(decisions),
        audited_zero_probes=len(selected),
        context_queries=2 * len(selected),
        witness_rows=len(witness_rows),
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
    assert plots["script_sha256"] == core.sha(ROOT / "plot_point_clearance.py")
    assert plots["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in ("sources.json", "anchor-point.csv", "point-summary.csv")
    }
    assert set(plots["output_sha256"]) == {
        "point-clearance.svg",
        "plot-values.csv",
        "plot-return-values.csv",
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
    evidence = dict(
        **audit,
        comparisons=core.comparisons - original_checks,
        original_join_comparisons=original_checks,
        plotted_summary_rows=252,
        plotted_return_rows=137,
        decimal_precision=60,
        native=native_proof,
        inherited_window_turnover_proof_sha256=PROOF_SHA,
        script_sha256=core.sha(Path(__file__)),
        sources_sha256=core.sha(args.results / "sources.json"),
        plot_sources_sha256=core.sha(args.figures / "plot-sources.json"),
        method="independent Decimal fixed-set point distances/gaps and signed changes; pinned independent window replay plus original native window and Point2D.distance; prior fields inherited, context only, no new probe predicate or intervention",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
