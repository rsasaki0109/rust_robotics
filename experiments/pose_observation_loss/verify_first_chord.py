#!/usr/bin/env python3
"""Independently audit first chords using Decimal trig and native source traces."""

import argparse
from collections import Counter, defaultdict
from decimal import Decimal as D, localcontext
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path
import shutil
import subprocess

import verify_epoch_retention as core

ROOT, RESULTS = core.ROOT, core.RESULTS
BASE = RESULTS / "fusion-return-probe-origin"
BASE_SHA = "672d38cb611282f0be3b65f99b2b1bf5815e54c1c7829cbf0a23182b13505112"
PROOF_SHA = "b6dbf95d64cdff2e052c671b48e17e172d54c9d8d7013514291b5369192e450b"
MAP_SHA = "59ff820ccbf076cfd90906da055c4f0b95c58207652d7d28610b294f6c51b3e4"
TYPES = "crates/rust_robotics_core/src/types.rs"
CODE = {
    **core.CODE,
    TYPES: "19c5cf3473851102b84caa21eaa2149c9bffd7d113a0a035622bf53164bde80f",
}
HELPERS = {
    "verify_epoch_retention.py": "417a3e93cc4680e77722fe2c95e24f206d532399bd4e2efc8d9951a24530e9f1"
}
META, KEY = core.META, core.KEY
GROUP = ("partition", "noise", "variant", "mode", "category_selector")
PHASES, BRANCHES = ("available", "next_loss"), ("joint", "guard")
CATEGORIES = (
    "ttl_ended",
    "age_ended",
    "returned_censored",
    "no_return",
    "unobserved_ttl_limit",
    "no_observed_loss",
)
MASKS = ("boundary_only", "obstacle_only", "both")
KINDS = (
    "audited",
    "direct_unsafe",
    "direct_unknown",
    *MASKS,
    "prior_unknown_direct_unsafe",
    "prior_unknown_direct_unknown",
    "nominal_only_reset_direct_unsafe",
    "nominal_only_reset_direct_unknown",
)
REFS = tuple(
    f"{p}_{b}_first_{kind}_refs" for p, b in product(PHASES, BRANCHES) for kind in KINDS
)
HAS = tuple("has_available_first_" + m for m in (*MASKS, "unknown"))
FIELDS = (
    "prior_truth_yaw",
    "noise_yaw",
    "bias_yaw",
    "nominal_steering",
    "sensed_yaw_center",
    "sensed_yaw_lo",
    "sensed_yaw_hi",
    "steering_lo",
    "steering_hi",
    "curvature_lo",
    "curvature_hi",
    "coarse_fallback",
    "first_endpoint_x_center",
    "first_endpoint_y_center",
    "first_endpoint_x_lo",
    "first_endpoint_x_hi",
    "first_endpoint_y_lo",
    "first_endpoint_y_hi",
    "first_chord_radius",
    "first_boundary_gap_lo",
    "first_boundary_gap_hi",
    "first_obstacle_gap_lo",
    "first_obstacle_gap_hi",
    "first_definite_wall_ids",
    "first_ambiguous_wall_ids",
    "first_definite_obstacle_ids",
    "first_ambiguous_obstacle_ids",
    "direct_first_test_state",
    "direct_first_failure_mask",
)
H, P, S = D("5e-10"), D("1e-12"), D("1e-9")
W, L = D.from_float(0.4), D.from_float(0.05)
PI = D(
    "3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825"
)
WALLS = ("x_min", "y_min", "x_max", "y_max")


def contract(source):
    import hashlib

    full = core.block(source, "fn speed_free_distance(")
    calc = full[
        full.index("        let length = step as f64 * 0.05;") : full.index(
            "        if [point.x"
        )
    ]
    for text in (
        "let curvature = steering.tan() / WHEELBASE;",
        "curvature.abs() < 1e-9",
        "(yaw.sin() - sensed.yaw.sin()) / curvature",
        "(yaw.cos() - sensed.yaw.cos()) / curvature",
        "sensed.yaw + curvature * length",
    ):
        assert text in full
    assert "const WHEELBASE: f64 = 0.4;" in source
    assert source.index("let nominal_steering = steering;") < source.index(
        "if safety_paused {\n                    target_speed = 0.0;"
    )
    return dict(
        full_probe_sha256=hashlib.sha256(full.encode()).hexdigest(),
        first_endpoint_arithmetic_sha256=hashlib.sha256(calc.encode()).hexdigest(),
    )


def sincos(value):
    with localcontext() as ctx:
        ctx.prec = 80
        x = D(value) % (2 * PI)
        if x > PI:
            x -= 2 * PI
        sine = term = x
        cosine = cterm = D(1)
        i = 0
        while True:
            i += 1
            term *= -x * x / ((2 * i) * (2 * i + 1))
            cterm *= -x * x / ((2 * i - 1) * (2 * i))
            sine += term
            cosine += cterm
            if abs(term) < D("1e-75") and abs(cterm) < D("1e-75"):
                break
            assert i < 300
        return +sine, +cosine


def plus(a, b):
    return a[0] + b[0] - P, a[1] + b[1] + P


def minus(a, b):
    return a[0] - b[1] - P, a[1] - b[0] + P


def times(a, b):
    values = [x * y for x, y in product(a, b)]
    return min(values) - P, max(values) + P


def quotient(a, b):
    assert not b[0] <= 0 <= b[1]
    values = [x / y for x, y in product(a, b)]
    return min(values) - P, max(values) + P


def trig(pair, cosine=False):
    center = (pair[0] + pair[1]) / 2
    radius = (pair[1] - pair[0]) / 2
    value = sincos(center)[int(cosine)]
    return max(D(-1), value - radius - P), min(D(1), value + radius + P)


def angles(before, current):
    raw = dict(
        prior_truth_yaw=before["yaw"],
        noise_yaw=current["noise_yaw"],
        bias_yaw=current["bias_yaw"],
        nominal_steering=current["nominal_steering"],
    )
    assert all(D(v).is_finite() and abs(D(v)) <= 100 for v in raw.values())
    delta = float(current["nominal_steering"])
    assert abs(delta) <= 0.612
    center = (float(before["yaw"]) + float(current["noise_yaw"])) + float(
        current["bias_yaw"]
    )
    assert abs(center) <= 100
    return dict(
        **raw,
        sensed_yaw_center=center,
        sensed_yaw_lo=D.from_float(center) - 3 * H - P,
        sensed_yaw_hi=D.from_float(center) + 3 * H + P,
        steering_lo=D.from_float(delta) - H,
        steering_hi=D.from_float(delta) + H,
    )


def endpoint(old, angle, trace):
    with localcontext() as ctx:
        ctx.prec = 60
        sx = tuple(D.from_float(float(old[k])) for k in ("sensed_x_lo", "sensed_x_hi"))
        sy = tuple(D.from_float(float(old[k])) for k in ("sensed_y_lo", "sensed_y_hi"))
        da, db = angle["steering_lo"], angle["steering_hi"]
        sn, cs = sincos(da)
        tan_lo = sn / cs - P
        sn, cs = sincos(db)
        tan_hi = sn / cs + P
        curvature = quotient((tan_lo, tan_hi), (W, W))
        coarse = curvature[0] <= D("1e-6") and curvature[1] >= D("-1e-6")
        if coarse:
            ex = (sx[0] - D("0.06"), sx[1] + D("0.06"))
            ey = (sy[0] - D("0.06"), sy[1] + D("0.06"))
        else:
            yaw = (angle["sensed_yaw_lo"], angle["sensed_yaw_hi"])
            theta = plus(yaw, times(curvature, (L, L)))
            ex = plus(sx, quotient(minus(trig(theta), trig(yaw)), curvature))
            ey = minus(
                sy, quotient(minus(trig(theta, True), trig(yaw, True)), curvature)
            )
        cx, cy = float(trace["endpoint_x"]), float(trace["endpoint_y"])
        dx, dy = D.from_float(cx), D.from_float(cy)
        assert ex[0] <= dx <= ex[1] and ey[0] <= dy <= ey[1]
        ax, ay = D.from_float(float(old["sensed_x_center"])), D.from_float(
            float(old["sensed_y_center"])
        )
        ae = (
            max(abs(sx[0] - ax), abs(sx[1] - ax)) ** 2
            + max(abs(sy[0] - ay), abs(sy[1] - ay)) ** 2
        ).sqrt()
        be = (
            max(abs(ex[0] - dx), abs(ex[1] - dx)) ** 2
            + max(abs(ey[0] - dy), abs(ey[1] - dy)) ** 2
        ).sqrt()
        return dict(
            curvature_lo=curvature[0],
            curvature_hi=curvature[1],
            coarse_fallback=coarse,
            first_endpoint_x_center=cx,
            first_endpoint_y_center=cy,
            first_endpoint_x_lo=ex[0],
            first_endpoint_x_hi=ex[1],
            first_endpoint_y_lo=ey[0],
            first_endpoint_y_hi=ey[1],
            first_chord_radius=max(ae, be) + P,
        )


def segment(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    square = dx * dx + dy * dy
    if square < D("1e-20"):
        return ((point[0] - start[0]) ** 2 + (point[1] - start[1]) ** 2).sqrt()
    progress = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / square
    progress = min(D(1), max(D(0), progress))
    return (
        (point[0] - (start[0] + progress * dx)) ** 2
        + (point[1] - (start[1] + progress * dy)) ** 2
    ).sqrt()


def state(pair):
    return "unsafe" if pair[1] <= 0 else "safe" if pair[0] > 0 else "unknown"


def geometry(old, e, obstacles):
    with localcontext() as ctx:
        ctx.prec = 60
        xl, xh, yl, yh = (
            D(e[k])
            for k in (
                "first_endpoint_x_lo",
                "first_endpoint_x_hi",
                "first_endpoint_y_lo",
                "first_endpoint_y_hi",
            )
        )
        margin = D(old["speed_margin"])
        assert 0 < margin < 10
        ml, mh = margin - H, margin + H
        walls = (
            (xl - mh - S, xh - ml + S),
            (yl - mh - S, yh - ml + S),
            (50 - xh - mh - S, 50 - xl - ml + S),
            (50 - yh - mh - S, 50 - yl - ml + S),
        )
        wall_yes = [w for w, pair in zip(WALLS, walls) if state(pair) == "unsafe"]
        wall_maybe = [w for w, pair in zip(WALLS, walls) if state(pair) == "unknown"]
        a = tuple(
            D.from_float(float(old[k])) for k in ("sensed_x_center", "sensed_y_center")
        )
        b = tuple(
            D.from_float(float(e[k]))
            for k in ("first_endpoint_x_center", "first_endpoint_y_center")
        )
        lo, hi, yes, maybe = [], [], [], []
        for oid, x, y in obstacles:
            distance = segment((x, y), a, b)
            low = max(D(0), distance - e["first_chord_radius"]) - mh - S
            high = distance + e["first_chord_radius"] - ml + S
            lo.append(low)
            hi.append(high)
            if state((low, high)) == "unsafe":
                yes.append(oid)
            elif state((low, high)) == "unknown":
                maybe.append(oid)
        boundary = min(v[0] for v in walls), min(v[1] for v in walls)
        obstacle = (min(lo), min(hi)) if lo else None
        full = (
            (min(boundary[0], obstacle[0]), min(boundary[1], obstacle[1]))
            if obstacle
            else boundary
        )
        result = state(full)
        assert result != "safe", "safe direct chord contradicts inherited zero"
        mask = (
            "both"
            if wall_yes and yes
            else "boundary_only" if wall_yes else "obstacle_only" if yes else ""
        )
        assert (result == "unsafe") == bool(mask)
        return dict(
            first_boundary_gap_lo=boundary[0],
            first_boundary_gap_hi=boundary[1],
            first_obstacle_gap_lo=obstacle[0] if obstacle else None,
            first_obstacle_gap_hi=obstacle[1] if obstacle else None,
            first_definite_wall_ids=";".join(wall_yes),
            first_ambiguous_wall_ids=";".join(wall_maybe),
            first_definite_obstacle_ids=";".join(yes),
            first_ambiguous_obstacle_ids=";".join(maybe),
            direct_first_test_state=result,
            direct_first_failure_mask=mask,
        )


def read_map(path):
    assert core.sha(path) == MAP_SHA
    result = defaultdict(list)
    for i, r in enumerate(core.read(path), 1):
        result[r["scenario"]].append(
            (str(i), D.from_float(float(r["x"])), D.from_float(float(r["y"])))
        )
    return result


def raw_inputs(cases, selected, raw, roots):
    files, wanted = defaultdict(dict), set()
    for c in cases:
        files[c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"][
            c["scenario"], c["planner"]
        ] = c
    for d in selected:
        n = core.tick(d["time_s"])
        wanted.update(((d["case_id"], n - 1), (d["case_id"], n)))
    assert set(files) == {(c, n) for c, s in raw.items() for n in s["raw_sha256"]}
    points, audit = {}, Counter()
    for i, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert core.sha(path) == raw[cohort]["raw_sha256"][name]
        audit["raw_bytes"] += path.stat().st_size
        last = defaultdict(lambda: -1)
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
                assert identity in members
                c = members[identity]
                n = core.tick(cells[pos["time_s"]])
                assert n == last[identity] + 1
                last[identity] = n
                if not core.truth(c["selected_timeout"]):
                    continue
                audit["timeout_grid_rows"] += 1
                if n == 0:
                    values = [
                        D(cells[pos[k]])
                        for k in ("effective_position_sigma", "effective_yaw_sigma_rad")
                    ]
                    assert all(x.is_finite() and x >= 0 for x in values) and any(
                        x > 0 for x in values
                    )
                    audit["noisy_branch_histories"] += 1
                if (c["case_id"], n) in wanted:
                    points[c["case_id"], n] = dict(zip(header, cells))
        assert set(last) == set(members)
        for identity, c in members.items():
            assert last[identity] == (
                9000 if core.truth(c["selected_timeout"]) else core.tick(c["time_s"])
            )
        audit["raw_files"] += 1
        if i % 18 == 0:
            print(f"Independent first-chord inputs read: {i}/108 files", flush=True)
    assert points.keys() == wanted
    audit["extracted_raw_points"] = len(points)
    return points, dict(audit)


def native(work, rustc, selected, points, map_path):
    source = (core.REPO / next(iter(core.CODE))).read_text()
    types = (core.REPO / TYPES).read_text()
    vehicle = (
        core.REPO / "crates/rust_robotics_control/src/pure_pursuit.rs"
    ).read_text()
    full = core.block(source, "fn speed_free_distance(")
    prefix = full.split("    let curvature", 1)[0].replace(
        "fn speed_free_distance(", "fn initial_test(", 1
    )
    calc = full[
        full.index("        let length = step as f64 * 0.05;") : full.index(
            "        if [point.x"
        )
    ]
    harness = (
        "#![allow(dead_code,unused_variables)]\n"
        + next(s for s in source.splitlines() if s.startswith("const WHEELBASE:"))
        + "\n#[derive(Clone,Copy)]\n"
        + core.block(types, "pub struct Point2D")
    )
    harness += (
        "\nimpl Point2D {\n"
        + core.block(types, "pub fn new(x: f64, y: f64)")
        + "\n"
        + core.block(types, "pub fn distance(&self, other: &Point2D)")
        + "\n}\n"
    )
    harness += (
        core.block(vehicle, "pub struct VehicleState")
        + "\nimpl VehicleState {\n"
        + core.block(
            vehicle, "pub fn new(x: f64, y: f64, yaw: f64, v: f64, wheelbase: f64)"
        )
        + "\n}\n"
    )
    harness += (
        core.block(source, "fn point_segment_distance(")
        + "\n"
        + full
        + "\n"
        + prefix
        + "    1.0\n}\n"
    )
    harness += (
        "fn first_endpoint(sensed: VehicleState, steering:f64)->Point2D {\nlet start=Point2D::new(sensed.x,sensed.y);\nlet curvature=steering.tan()/WHEELBASE;\nlet step=1;\n"
        + calc
        + "point\n}\n"
    )
    harness += r"""
fn main(){
    let args:Vec<_>=std::env::args().collect();
    let mut map=std::collections::BTreeMap::<String,Vec<(usize,Point2D)>>::new();
    for (i,line) in std::fs::read_to_string(&args[1]).unwrap().lines().skip(1).enumerate(){
        let f:Vec<_>=line.split(',').collect();map.entry(f[0].to_owned()).or_default().push((i+1,Point2D::new(f[1].parse().unwrap(),f[2].parse().unwrap())));
    }
    println!("row_id,endpoint_x,endpoint_y,initial_unsafe,wall_ids,obstacle_ids,free_distance");
    for line in std::fs::read_to_string(&args[2]).unwrap().lines().skip(1){
        let f:Vec<_>=line.split('\t').collect();let x=f[2].parse().unwrap();let y=f[3].parse().unwrap();let yaw=f[4].parse().unwrap();let delta=f[5].parse().unwrap();let margin=f[6].parse().unwrap();
        let state=||VehicleState::new(x,y,yaw,0.0,0.4);let start=Point2D::new(x,y);let end=first_endpoint(state(),delta);
        let obs:Vec<_>=map[f[1]].iter().map(|p|p.1).collect();
        let wall_ids=[end.x,end.y,50.0-end.x,50.0-end.y].iter().zip(["x_min","y_min","x_max","y_max"]).filter(|(d,_)|**d<=margin).map(|(_,name)|name).collect::<Vec<_>>().join(";");
        let ids=map[f[1]].iter().filter(|(_,p)|p.distance(&start)<=2.0+margin && point_segment_distance(*p,start,end)<=margin).map(|(id,_)|id.to_string()).collect::<Vec<_>>().join(";");
        println!("{},{:.17e},{:.17e},{},{},{},{:.9}",f[0],end.x,end.y,initial_test(state(),delta,margin,&obs)==0.0,wall_ids,ids,speed_free_distance(state(),delta,margin,&obs));
    }
}
#[cfg(test)] mod tests {
 use super::*;
 fn s(x:f64,y:f64,yaw:f64)->VehicleState {VehicleState::new(x,y,yaw,0.0,0.4)}
 #[test] fn straight_first_wall(){assert_eq!(speed_free_distance(s(49.475,25.,0.),0.,0.5,&[]),0.0);assert!(first_endpoint(s(49.475,25.,0.),0.).x>49.5);}
 #[test] fn first_obstacle_only(){let obs=[Point2D::new(25.54,25.)];assert_eq!(initial_test(s(25.,25.,0.),0.,0.5,&obs),1.0);assert_eq!(speed_free_distance(s(25.,25.,0.),0.,0.5,&obs),0.0);}
 #[test] fn initial_obstacle(){let obs=[Point2D::new(25.1,25.)];assert_eq!(initial_test(s(25.,25.,0.),0.,0.5,&obs),0.0);assert_eq!(speed_free_distance(s(25.,25.,0.),0.,0.5,&obs),0.0);}
 #[test] fn curved_symmetry(){let a=first_endpoint(s(25.,25.,0.),0.4);let b=first_endpoint(s(25.,25.,0.),-0.4);assert!((a.x-b.x).abs()<1e-12);assert!((a.y+b.y-50.0).abs()<1e-12);assert!(a.y>25. && b.y<25.);}
 #[test] fn segment_interior_inclusive(){let p=Point2D::new(25.025,25.4);let end=first_endpoint(s(25.,25.,0.),0.);assert!(point_segment_distance(p,Point2D::new(25.,25.),end)<=0.4);assert_eq!(speed_free_distance(s(25.,25.,0.),0.,0.4,&[p]),0.0);}
 #[test] fn free_probe(){assert_eq!(speed_free_distance(s(25.,25.,0.),0.,0.5,&[]),2.0);}
}
"""
    work.mkdir(parents=True)
    path = work / "first-chord.rs"
    path.write_text(harness)
    for flags, name in ((["--test"], "native-tests"), ([], "native-trace")):
        subprocess.run(
            [
                str(rustc),
                "--edition=2021",
                "-O",
                *flags,
                str(path),
                "-o",
                str(work / name),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
    tests = subprocess.run(
        [str(work / "native-tests")], check=True, capture_output=True, text=True
    ).stdout
    assert "6 passed; 0 failed" in tests
    (work / "native-tests.txt").write_text(tests)
    query = work / "native-inputs.tsv"
    with query.open("w") as f:
        f.write("row_id\tscenario\tx\ty\tyaw\tdelta\tmargin\n")
        for i, d in enumerate(selected, 1):
            before = points[d["case_id"], core.tick(d["time_s"]) - 1]
            current = points[d["case_id"], core.tick(d["time_s"])]
            an = angles(before, current)
            f.write(
                f"{i}\t{d['scenario']}\t{float(d['sensed_x_center']):.17e}\t{float(d['sensed_y_center']):.17e}\t{an['sensed_yaw_center']:.17e}\t{float(an['nominal_steering']):.17e}\t{float(d['speed_margin']):.17e}\n"
            )
    output = subprocess.run(
        [str(work / "native-trace"), str(map_path), str(query)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    (work / "native-trace.csv").write_text(output)
    rows = core.read(work / "native-trace.csv")
    assert len(rows) == len(selected)
    for i, r in enumerate(rows, 1):
        assert r["row_id"] == str(i) and not core.truth(r["initial_unsafe"], raw=True)
    return rows, dict(
        rustc=subprocess.run(
            [str(rustc), "--version"], check=True, capture_output=True, text=True
        ).stdout.strip(),
        harness_sha256=core.sha(path),
        inputs_sha256=core.sha(query),
        trace_sha256=core.sha(work / "native-trace.csv"),
        native_tests=6,
        traced_centers=len(rows),
        native_zero_centers=sum(float(r["free_distance"]) == 0 for r in rows),
        method="unchanged full probe and segment distance; extracted first endpoint arithmetic; rounded centers, no closed-loop replay",
    )


def build(old, before=None, current=None, obstacles=(), trace=None):
    out = dict(
        old, first_chord_audited=old["probe_origin"] == "first_chord_by_exclusion"
    )
    out.update(dict.fromkeys(FIELDS))
    if not out["first_chord_audited"]:
        assert before is None and current is None and trace is None
        return out
    assert old["initial_test_state"] == "safe"
    for k in old.keys() & current.keys():
        if k in META:
            assert old[k] == current[k]
        elif current[k] in ("true", "false"):
            assert core.truth(old[k]) == core.truth(current[k], raw=True)
        else:
            assert float(old[k]) == float(current[k]), k
    assert core.tick(before["time_s"]) + 1 == core.tick(current["time_s"])
    for k in ("x", "y"):
        assert (
            old["prior_truth_" + k] == before[k]
            and old["noise_" + k] == current["noise_" + k]
            and old["bias_" + k] == current["bias_" + k]
        )
    with localcontext() as ctx:
        ctx.prec = 60
        an = angles(before, current)
        ep = endpoint(old, an, trace)
        geo = geometry(old, ep, obstacles)
    out.update(an)
    out.update(ep)
    out.update(geo)
    assert set(an) | set(ep) | set(geo) == set(FIELDS)
    for field, nativefield in (("wall", "wall_ids"), ("obstacle", "obstacle_ids")):
        definite = set(filter(None, geo["first_definite_" + field + "_ids"].split(";")))
        ambiguous = set(
            filter(None, geo["first_ambiguous_" + field + "_ids"].split(";"))
        )
        actual = set(filter(None, trace[nativefield].split(";")))
        assert definite <= actual <= definite | ambiguous
    if geo["direct_first_test_state"] == "unsafe":
        assert float(trace["free_distance"]) == 0
    return out


def anchor(e, dd, ne, nr):
    parent = int(e["parent_rp_records"])
    out = dict(e)
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
    for k, value in measures.items():
        core.check(e[k], value)
    out.update(measures)
    out.update(dict.fromkeys((*REFS, *HAS)))
    if not core.truth(e["returned"] or "False"):
        assert not dd
        return out
    assert all(all(d[k] == e[k] for k in META) for d in dd)
    for p, b in product(PHASES, BRANCHES):
        rr = [
            d
            for d in dd
            if d["phase"] == p and d["branch"] == b and d["first_chord_audited"]
        ]
        values = dict(
            audited=len(rr),
            direct_unsafe=sum(d["direct_first_test_state"] == "unsafe" for d in rr),
            direct_unknown=sum(d["direct_first_test_state"] == "unknown" for d in rr),
        )
        values.update(
            {m: sum(d["direct_first_failure_mask"] == m for d in rr) for m in MASKS}
        )
        for s in ("unsafe", "unknown"):
            values["prior_unknown_direct_" + s] = sum(
                d["prior_certainty"] == "unknown" and d["direct_first_test_state"] == s
                for d in rr
            )
            values["nominal_only_reset_direct_" + s] = sum(
                d["timer_action"] == "reset"
                and d["refined_failed_gates"] == "nominal"
                and d["direct_first_test_state"] == s
                for d in rr
            )
        out.update(
            {f"{p}_{b}_first_{name}_refs": value for name, value in values.items()}
        )
    for m in (*MASKS, "unknown"):
        out["has_available_first_" + m] = any(
            d["phase"] == "available"
            and d["first_chord_audited"]
            and (
                d["direct_first_test_state"] == "unknown"
                if m == "unknown"
                else d["direct_first_failure_mask"] == m
            )
            for d in dd
        )
    return out


def summaries(old, anchors):
    result = {}
    for row in old:
        key = tuple(row[k] for k in GROUP)
        p, n, arm, mode, category = key
        aa = [
            a
            for a in anchors
            if (a["partition"], a["noise"], a["variant"]) == (p, n, arm)
            and (mode == "all_modes" or a["mode"] == mode)
            and (category == "all" or a["anchor_category"] == category)
        ]
        rr = [a for a in aa if core.truth(a["returned"] or "False")]
        mass = lambda rows: sum((a["census_case_weight"] for a in rows), Fraction())
        out = dict(row)
        out.update({k: sum(a[k] or 0 for a in aa) for k in REFS})
        for name in HAS:
            selected = [a for a in aa if a[name]]
            out.update(
                {
                    name + "_events": len(selected),
                    name + "_case_mass": mass(selected),
                    name
                    + "_fraction": Fraction(len(selected), len(rr)) if rr else None,
                    name
                    + "_case_mass_fraction": mass(selected) / mass(rr) if rr else None,
                }
            )
        core.check(row["census_case_mass"], mass(aa))
        assert key not in result
        result[key] = out
    return result


def closure(rows):
    checks = 0
    for key, row in rows.items():
        for i, whole, parts in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", CATEGORIES),
        ):
            if key[i] == whole:
                for field in (
                    *REFS,
                    *(h + s for h in HAS for s in ("_events", "_case_mass")),
                ):
                    assert row[field] == sum(
                        rows[key[:i] + (p,) + key[i + 1 :]][field] for p in parts
                    )
                    checks += 1
        for p, b in product(PHASES, BRANCHES):
            prefix = p + "_" + b + "_first_"
            assert (
                row[prefix + "audited_refs"]
                == row[prefix + "direct_unsafe_refs"]
                + row[prefix + "direct_unknown_refs"]
            )
            assert (
                sum(row[prefix + m + "_refs"] for m in MASKS)
                == row[prefix + "direct_unsafe_refs"]
            )
            assert row[prefix + "audited_refs"] == int(
                row[p + "_" + b + "_first_chord_by_exclusion_refs"]
            )
    return checks


def compare(path, rows, keys):
    expected = {tuple(str(r[k]) for k in keys): r for r in rows}
    actual = core.index(core.read(path), keys)
    assert actual.keys() == expected.keys()
    for key, row in actual.items():
        check_row(row, expected[key])


def check_row(row, expected):
    assert set(row) == set(expected)
    for field, value in expected.items():
        if field in FIELDS and isinstance(value, (float, D)):
            assert row[field] != "" and math.isclose(
                float(row[field]), float(value), rel_tol=1e-10, abs_tol=1e-9
            ), (field, row[field], value)
            core.comparisons += 1
        else:
            core.check(row[field], value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("results", "figures", "work", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--rustc", type=Path, default=shutil.which("rustc"))
    parser.add_argument("--raw-roots", type=Path)
    args = parser.parse_args()
    if not __debug__ or args.work.exists() or args.output.exists() or not args.rustc:
        parser.error("Use new work/evidence paths, provide rustc and run without -O")
    prior = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        core.sha(BASE / "sources.json") == BASE_SHA
        and core.sha(BASE / "independent-verification.json") == PROOF_SHA
    )
    freeze = RESULTS / "fusion-return-first-chord/freeze-pins.json"
    pins = json.loads(freeze.read_text())
    assert pins == dict(
        base_main="0836de71eaf2cf03e476cb16a1ed3e0022be7285",
        sources_sha256=BASE_SHA,
        proof_sha256=PROOF_SHA,
        map_sha256=MAP_SHA,
    )
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        "fusion-return-probe-origin/sources.json": BASE_SHA,
        "fusion-return-probe-origin/independent-verification.json": PROOF_SHA,
        "fusion-return-probe-origin/protocol.md": prior["analysis"]["protocol_sha256"],
        **{
            "fusion-return-probe-origin/" + k: h
            for k, h in prior["output_sha256"].items()
        },
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_probe_origin.py": prior["analysis"]["script_sha256"],
    }
    assert (
        source["analysis"]["verifier_helpers_sha256"] == HELPERS
        and source["analysis"]["code_sha256"] == CODE
    )
    assert (
        source["raw"] == prior["raw"]
        and source["map_sha256"] == MAP_SHA
        and source["inherited_probe_proof_sha256"] == PROOF_SHA
        and source["inherited_validation"] == prior["inherited_validation"]
    )
    assert source["schema"] == dict(
        half_unit=5e-10,
        elemental_allowance=1e-12,
        segment_allowance=1e-9,
        curvature_fallback=1e-6,
        fallback_expansion=0.06,
        phases=list(PHASES),
        branches=list(BRANCHES),
        retrospective=True,
        inherited="all preceding columns and source-path exclusion; direct first-test fields separate; numerical allowances conditional on stated platform assumptions",
    )
    for name, h in source["input_sha256"].items():
        assert core.sha(RESULTS / name) == h, name
    for name, h in {**source["analysis"]["helpers_sha256"], **HELPERS}.items():
        assert core.sha(ROOT / name) == h, name
    for name, h in CODE.items():
        assert core.sha(core.REPO / name) == h, name
    assert source["analysis"]["source_contract"] == contract(
        (core.REPO / next(iter(core.CODE))).read_text()
    )
    assert (
        source["analysis"]["script_sha256"] == core.sha(ROOT / "analyze_first_chord.py")
        and source["analysis"]["protocol_sha256"]
        == core.sha(RESULTS / "fusion-return-first-chord/protocol.md")
        and source["analysis"]["freeze_sha256"] == core.sha(freeze)
    )
    assert set(source["output_sha256"]) == {
        "first-chord-decisions.csv",
        "anchor-first-chord.csv",
        "first-chord-summary.csv",
    }
    for name, h in source["output_sha256"].items():
        assert core.sha(args.results / name) == h, name
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    compiler = subprocess.run(
        [str(args.rustc), "--version"], check=True, capture_output=True, text=True
    ).stdout.strip()
    assert compiler == proof["native"]["rustc"]
    for cohort, spec in source["raw"].items():
        path = Path(roots.get(cohort, spec["path"])) / "provenance.json"
        assert core.sha(path) == spec["provenance_sha256"]
        record = json.loads(path.read_text())
        assert (
            record["rustc"] == compiler
            and record["inputs_sha256"]["obstacles.csv"] == MAP_SHA
        )
        assert set(record["inputs_by_variant_sha256"]) == {"epoch", "curved"} and all(
            v["obstacles.csv"] == MAP_SHA
            for v in record["inputs_by_variant_sha256"].values()
        )
    cases, original = core.original_inputs()
    events = core.read(BASE / "anchor-probe-origin.csv")
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
    old = core.read(BASE / "probe-decisions.csv")
    selected = [d for d in old if d["probe_origin"] == "first_chord_by_exclusion"]
    assert len(selected) == 61
    points, audit = raw_inputs(cases, selected, source["raw"], roots)
    traces, native_proof = native(
        args.work, args.rustc, selected, points, BASE / "obstacle-map.csv"
    )
    trace_map = {(d["event_id"], d["time_s"]): t for d, t in zip(selected, traces)}
    obstacles = read_map(BASE / "obstacle-map.csv")
    decisions = [
        (
            build(
                d,
                points[d["case_id"], core.tick(d["time_s"]) - 1],
                points[d["case_id"], core.tick(d["time_s"])],
                obstacles[d["scenario"]],
                trace_map[d["event_id"], d["time_s"]],
            )
            if d["probe_origin"] == "first_chord_by_exclusion"
            else build(d)
        )
        for d in old
    ]
    by_event = defaultdict(list)
    for d in decisions:
        by_event[d["event_id"]].append(d)
    ne = Counter(e["case_id"] for e in original.values())
    nr = Counter(e["case_id"] for e in original.values() if core.truth(e["received"]))
    anchors = [
        anchor(e, by_event[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]])
        for e in events
    ]
    summary = summaries(core.read(BASE / "probe-summary.csv"), anchors)
    compare(
        args.results / "first-chord-decisions.csv", decisions, ("event_id", "time_s")
    )
    compare(args.results / "anchor-first-chord.csv", anchors, ("event_id",))
    compare(args.results / "first-chord-summary.csv", list(summary.values()), GROUP)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(core.truth(a["returned"] or "False") for a in anchors),
        decision_references=len(decisions),
        audited_first_chords=61,
        available_audited=sum(
            d["first_chord_audited"] and d["phase"] == "available" for d in decisions
        ),
        boundary_audited=sum(
            d["first_chord_audited"] and d["phase"] == "next_loss" for d in decisions
        ),
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
    assert plots["script_sha256"] == core.sha(ROOT / "plot_first_chord.py")
    assert plots["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in ("sources.json", "anchor-first-chord.csv", "first-chord-summary.csv")
    }
    assert set(plots["output_sha256"]) == {
        "first-chord.svg",
        "plot-values.csv",
        "plot-return-values.csv",
    }
    for name, h in plots["output_sha256"].items():
        assert core.sha(args.figures / name) == h
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
        inherited_probe_proof_sha256=PROOF_SHA,
        script_sha256=core.sha(Path(__file__)),
        sources_sha256=core.sha(args.results / "sources.json"),
        plot_sources_sha256=core.sha(args.figures / "plot-sources.json"),
        method="independent Decimal Taylor trig, endpoint intervals and projection distance; native unchanged full probe at rounded centers; inherited prior columns; stated numerical allowances, no closed-loop replay",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
