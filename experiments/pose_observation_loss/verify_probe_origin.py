#!/usr/bin/env python3
"""Check initial probe origins with Decimal rectangles and unchanged Rust tests."""

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
BASE = RESULTS / "fusion-return-command-sign"
BASE_SHA = "2090042807aaef058d8c43eee5b112ab0ef658b934b1bccad29e81a5af4bc62b"
PROOF_SHA = "3d82c7af8398d4a954043d8179ee244083cb5141b7752cce5e4b165e194dd3b8"
MAP_SHA = "59ff820ccbf076cfd90906da055c4f0b95c58207652d7d28610b294f6c51b3e4"
TYPES = "crates/rust_robotics_core/src/types.rs"
TYPES_SHA = "19c5cf3473851102b84caa21eaa2149c9bffd7d113a0a035622bf53164bde80f"
HELPERS = {
    "verify_epoch_retention.py": "417a3e93cc4680e77722fe2c95e24f206d532399bd4e2efc8d9951a24530e9f1"
}
META, KEY = core.META, core.KEY
GROUP = ("partition", "noise", "variant", "mode", "category_selector")
PHASES, BRANCHES = ("available", "next_loss"), ("joint", "guard")
ORIGINS = (
    "positive_probe",
    "initial_unsafe",
    "first_chord_by_exclusion",
    "ambiguous_zero",
)
ZERO_ORIGINS = ORIGINS[1:]
MASKS = ("boundary_only", "obstacle_only", "both")
CATEGORIES = (
    "ttl_ended",
    "age_ended",
    "returned_censored",
    "no_return",
    "unobserved_ttl_limit",
    "no_observed_loss",
)
REFS = tuple(
    f"{p}_{b}_{name}_refs"
    for p, b in product(PHASES, BRANCHES)
    for name in (
        *ORIGINS,
        *("initial_" + m for m in MASKS),
        *("prior_unknown_" + o for o in ZERO_ORIGINS),
        *("nominal_only_reset_" + o for o in ZERO_ORIGINS),
    )
)
HAS = tuple("has_available_" + o for o in ZERO_ORIGINS)
H, P = D("5e-10"), D("1e-12")
WALLS = ("x_min", "y_min", "x_max", "y_max")


def contract(source, types):
    import hashlib

    initial = core.block(source, "fn speed_free_distance(").split(
        "    let curvature", 1
    )[0]
    for text in (
        "Point2D::new(sensed.x, sensed.y)",
        ".filter(|p| p.distance(&start) <= 2.0 + margin)",
        ".any(|&d| d <= margin)",
        "p.distance(&start) <= margin",
        "return 0.0;",
    ):
        assert text in initial
    assert (
        "for step in 1..=40" in source and "return (step - 1) as f64 * 0.05;" in source
    )
    sensed = source.index("let sensed = if position_sigma == 0.0")
    update = source.index("state.update(acceleration, steering, DT);", sensed)
    output = source.index('writeln!(tracks, "{scenario},{planner},{time:.6}', update)
    assert sensed < update < output
    assert (
        "state.x + noise_x + bias_x" in source
        and "state.y + noise_y + bias_y" in source
    )
    assert "false,false,{position_sigma},{yaw_sigma},false" in source
    assert "((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()" in types
    return dict(
        initial_test_sha256=hashlib.sha256(initial.encode()).hexdigest(),
        pose_order="preceding post-update truth plus current noise plus current bias; two additions",
    )


def load_map(path):
    assert core.sha(path) == MAP_SHA
    groups = defaultdict(list)
    for i, r in enumerate(core.read(path), 1):
        x, y = float(r["x"]), float(r["y"])
        assert math.isfinite(x) and math.isfinite(y) and 0 <= x <= 50 and 0 <= y <= 50
        groups[r["scenario"]].append((str(i), D.from_float(x), D.from_float(y)))
    assert set(groups) == {"open_50x50", "maze_50x50", "dense_50x50"}
    return groups


def provenance(raw, roots, compiler):
    for cohort, spec in raw.items():
        path = Path(roots.get(cohort, spec["path"])) / "provenance.json"
        assert core.sha(path) == spec["provenance_sha256"]
        record = json.loads(path.read_text())
        assert record["rustc"] == compiler
        assert record["inputs_sha256"]["obstacles.csv"] == MAP_SHA
        assert set(record["inputs_by_variant_sha256"]) == {"epoch", "curved"}
        for arm in ("epoch", "curved"):
            assert record["inputs_by_variant_sha256"][arm]["obstacles.csv"] == MAP_SHA


def noise_branch(row):
    values = [
        D(row[k]) for k in ("effective_position_sigma", "effective_yaw_sigma_rad")
    ]
    assert all(x.is_finite() and x >= 0 for x in values)
    assert any(x > 0 for x in values), "noisy source branch not established"


def pose(before, current):
    values = {"prior_truth_" + k: before[k] for k in ("x", "y")}
    values.update(
        {
            k: current[k]
            for k in ("noise_x", "noise_y", "bias_x", "bias_y", "speed_margin")
        }
    )
    assert all(D(v).is_finite() and abs(D(v)) <= 100 for v in values.values())
    assert 0 < D(current["speed_margin"]) < 10
    previous, now = core.tick(before["time_s"]), core.tick(current["time_s"])
    assert now == previous + 1
    result = dict(
        prior_truth_time_s=previous / 50, observation_time_s=now / 50, **values
    )
    with localcontext() as ctx:
        ctx.prec = 60
        for axis in ("x", "y"):
            exact_center = sum(
                (
                    D(before[axis]),
                    D(current["noise_" + axis]),
                    D(current["bias_" + axis]),
                )
            )
            center = (float(before[axis]) + float(current["noise_" + axis])) + float(
                current["bias_" + axis]
            )
            assert abs(exact_center) <= 100 and abs(center) <= 100
            result["sensed_" + axis + "_center"] = center
            result["sensed_" + axis + "_lo"] = exact_center - 3 * H - P
            result["sensed_" + axis + "_hi"] = exact_center + 3 * H + P
    return result


def side(low, high):
    return "unsafe" if high <= 0 else "safe" if low > 0 else "unknown"


def geometry(p, obstacles):
    with localcontext() as ctx:
        ctx.prec = 60
        xl, xh, yl, yh = (
            D(p[k])
            for k in ("sensed_x_lo", "sensed_x_hi", "sensed_y_lo", "sensed_y_hi")
        )
        margin = D(p["speed_margin"])
        ml, mh = margin - H, margin + H
        assert 0 < ml <= mh < 10
        walls = (
            (xl - mh - P, xh - ml + P),
            (yl - mh - P, yh - ml + P),
            (50 - xh - mh - P, 50 - xl - ml + P),
            (50 - yh - mh - P, 50 - yl - ml + P),
        )
        wall_definite, wall_unknown = [], []
        for name, (low, high) in zip(WALLS, walls):
            state = side(low, high)
            if state == "unsafe":
                wall_definite.append(name)
            if state == "unknown":
                wall_unknown.append(name)
        lower_squares, upper_squares, definite, ambiguous = [], [], [], []
        safe_threshold, unsafe_threshold = (mh + P) ** 2, (ml - P) ** 2
        for oid, x, y in obstacles:
            dx = xl - x if x < xl else x - xh if x > xh else D(0)
            dy = yl - y if y < yl else y - yh if y > yh else D(0)
            fx, fy = max(abs(xl - x), abs(xh - x)), max(abs(yl - y), abs(yh - y))
            lower, upper = dx * dx + dy * dy, fx * fx + fy * fy
            lower_squares.append(lower)
            upper_squares.append(upper)
            if upper <= unsafe_threshold:
                definite.append(oid)
            elif lower <= safe_threshold:
                ambiguous.append(oid)
        boundary = min(p[0] for p in walls), min(p[1] for p in walls)
        obstacle = (
            (min(lower_squares).sqrt() - mh - P, min(upper_squares).sqrt() - ml + P)
            if lower_squares
            else None
        )
        low, high = (
            (min(boundary[0], obstacle[0]), min(boundary[1], obstacle[1]))
            if obstacle
            else boundary
        )
        state = side(low, high)
        mask = (
            "both"
            if wall_definite and definite
            else (
                "boundary_only"
                if wall_definite
                else "obstacle_only" if definite else ""
            )
        )
        assert (state == "unsafe") == bool(mask)
        return dict(
            initial_test_state=state,
            initial_failure_mask=mask,
            boundary_gap_lo=boundary[0],
            boundary_gap_hi=boundary[1],
            obstacle_gap_lo=obstacle[0] if obstacle else None,
            obstacle_gap_hi=obstacle[1] if obstacle else None,
            definite_wall_ids=";".join(wall_definite),
            ambiguous_wall_ids=";".join(wall_unknown),
            definite_obstacle_ids=";".join(definite),
            ambiguous_obstacle_ids=";".join(ambiguous),
            obstacle_records_checked=len(obstacles),
        )


def build(old, before, current, obstacles):
    for key in old.keys() & current.keys():
        value = current[key]
        if key in META:
            assert old[key] == value
        elif value in ("true", "false"):
            assert core.truth(old[key]) == core.truth(value, raw=True)
        else:
            assert float(old[key]) == float(value)
    rectangle = pose(before, current)
    g = geometry(rectangle, obstacles)
    free = D(old["speed_free_distance"])
    assert free >= 0
    if free > 0:
        assert g["initial_test_state"] != "unsafe"
        which = "positive_probe"
    else:
        which = {
            "unsafe": "initial_unsafe",
            "safe": "first_chord_by_exclusion",
            "unknown": "ambiguous_zero",
        }[g["initial_test_state"]]
    return dict(**old, **rectangle, **g, probe_origin=which)


def raw_inputs(cases, old, raw, roots):
    files, wanted = defaultdict(dict), set()
    for c in cases:
        files[c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"][
            c["scenario"], c["planner"]
        ] = c
    for d in old:
        step = core.tick(d["time_s"])
        wanted.update(((d["case_id"], step - 1), (d["case_id"], step)))
    assert set(files) == {(c, n) for c, s in raw.items() for n in s["raw_sha256"]}
    points, audit = {}, Counter()
    for i, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert core.sha(path) == raw[cohort]["raw_sha256"][name]
        audit["raw_bytes"] += path.stat().st_size
        last = defaultdict(lambda: -1)
        with path.open() as stream:
            header = next(stream).rstrip("\r\n").split(",")
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
            for line in stream:
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
                    noise_branch({k: cells[j] for k, j in pos.items()})
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
            print(f"Independent initial-probe inputs read: {i}/108 files", flush=True)
    assert points.keys() == wanted
    audit["extracted_raw_points"] = len(points)
    return points, dict(audit)


def native(work, rustc, decisions, map_path):
    source = (core.REPO / next(iter(core.CODE))).read_text()
    types = (core.REPO / TYPES).read_text()
    vehicle = (
        core.REPO / "crates/rust_robotics_control/src/pure_pursuit.rs"
    ).read_text()
    prefix = core.block(source, "fn speed_free_distance(").split(
        "    let curvature", 1
    )[0]
    harness = (
        "#![allow(dead_code, unused_variables)]\n#[derive(Clone, Copy)]\n"
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
    harness += prefix + "    1.0\n}\n"
    harness += r"""
fn test(x:f64, y:f64, margin:f64, obstacles:&[Point2D]) -> bool {
    speed_free_distance(VehicleState::new(x,y,0.0,0.0,0.4),0.0,margin,obstacles) == 0.0
}
fn main() {
    let args: Vec<_> = std::env::args().collect();
    let mut map = std::collections::BTreeMap::<String,Vec<Point2D>>::new();
    for line in std::fs::read_to_string(&args[1]).unwrap().lines().skip(1) {
        let f: Vec<_> = line.split(',').collect();
        map.entry(f[0].to_owned()).or_default().push(Point2D::new(f[1].parse().unwrap(),f[2].parse().unwrap()));
    }
    println!("row_id,initial_unsafe");
    for line in std::fs::read_to_string(&args[2]).unwrap().lines().skip(1) {
        let f: Vec<_> = line.split('\t').collect();
        println!("{},{}", f[0], test(f[2].parse().unwrap(), f[3].parse().unwrap(), f[4].parse().unwrap(), &map[f[1]]));
    }
}
#[cfg(test)] mod tests {
    use super::*;
    #[test] fn strictly_safe() { assert!(!test(25.,25.,1.,&[])); }
    #[test] fn inclusive_walls() {
        for (x,y) in [(1.,25.),(49.,25.),(25.,1.),(25.,49.)] { assert!(test(x,y,1.,&[])); }
        assert!(!test(f64::from_bits(1.0_f64.to_bits()+1),25.,1.,&[]));
    }
    #[test] fn inclusive_obstacle() {
        assert!(test(25.,25.,0.5,&[Point2D::new(25.5,25.)]));
        assert!(!test(25.,25.,0.5,&[Point2D::new(25.50001,25.)]));
    }
    #[test] fn nearby_filter_cannot_remove_blocker() {
        assert!(!test(25.,25.,0.5,&[Point2D::new(28.,25.)]));
        assert!(test(25.,25.,0.5,&[Point2D::new(25.,25.)]));
    }
    #[test] fn simultaneous_and_duplicate_constraints() {
        let p=Point2D::new(2.,25.);
        assert!(test(1.,25.,1.,&[p,p]));
    }
}
"""
    work.mkdir(parents=True)
    path, tests, exe = (
        work / "initial-probe.rs",
        work / "initial-probe-tests",
        work / "initial-probe",
    )
    path.write_text(harness)
    for flags, output in ((["--test"], tests), ([], exe)):
        subprocess.run(
            [str(rustc), "--edition=2021", "-O", *flags, str(path), "-o", str(output)],
            check=True,
            capture_output=True,
            text=True,
        )
    output = subprocess.run(
        [str(tests)], check=True, capture_output=True, text=True
    ).stdout
    assert "5 passed; 0 failed" in output
    (work / "native-tests.txt").write_text(output)
    queries = work / "center-inputs.tsv"
    with queries.open("w") as f:
        f.write("row_id\tscenario\tx\ty\tmargin\n")
        for i, d in enumerate(decisions, 1):
            f.write(
                f"{i}\t{d['scenario']}\t{d['sensed_x_center']:.17e}\t{d['sensed_y_center']:.17e}\t{float(d['speed_margin']):.17e}\n"
            )
    result = subprocess.run(
        [str(exe), str(map_path), str(queries)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    (work / "native-centers.csv").write_text(result)
    rows = core.read(work / "native-centers.csv")
    assert len(rows) == len(decisions)
    compared = 0
    for i, (r, d) in enumerate(zip(rows, decisions), 1):
        assert r["row_id"] == str(i)
        if d["initial_test_state"] != "unknown":
            assert core.truth(r["initial_unsafe"], raw=True) == (
                d["initial_test_state"] == "unsafe"
            )
            compared += 1
    return dict(
        rustc=subprocess.run(
            [str(rustc), "--version"], check=True, capture_output=True, text=True
        ).stdout.strip(),
        harness_sha256=core.sha(path),
        center_inputs_sha256=core.sha(queries),
        center_output_sha256=core.sha(work / "native-centers.csv"),
        native_tests=5,
        certain_center_comparisons=compared,
        unknown_center_rows=len(rows) - compared,
        method="unchanged initial source test and actual Point2D distance; rounded centers checked only for certain enclosures; no chord replay",
    )


def anchor(e, dd, ne, nr):
    a = dict(e)
    parent = int(e["parent_rp_records"])
    weights = dict(
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
    for k, weight in weights.items():
        core.check(e[k], weight)
    a.update(weights)
    a.update(dict.fromkeys((*REFS, *HAS)))
    if not core.truth(e["returned"] or "False"):
        assert not dd
        return a
    start, end = core.tick(e["return_time_s"]), (
        core.tick(e["end_time_s"]) if core.truth(e["ended"]) else 9000
    )
    assert [core.tick(d["time_s"]) for d in dd] == list(range(start, end + 1))
    assert all(all(d[k] == e[k] for k in META) for d in dd)
    for p, b in product(PHASES, BRANCHES):
        rr = [d for d in dd if (d["phase"], d["branch"]) == (p, b)]
        values = {o: sum(d["probe_origin"] == o for d in rr) for o in ORIGINS}
        values.update(
            {
                "initial_"
                + m: sum(
                    d["probe_origin"] == "initial_unsafe"
                    and d["initial_failure_mask"] == m
                    for d in rr
                )
                for m in MASKS
            }
        )
        for o in ZERO_ORIGINS:
            values["prior_unknown_" + o] = sum(
                d["prior_certainty"] == "unknown" and d["probe_origin"] == o for d in rr
            )
            values["nominal_only_reset_" + o] = sum(
                d["timer_action"] == "reset"
                and d["refined_failed_gates"] == "nominal"
                and d["probe_origin"] == o
                for d in rr
            )
        a.update({f"{p}_{b}_{name}_refs": value for name, value in values.items()})
    a.update(
        {
            "has_available_"
            + o: any(d["phase"] == "available" and d["probe_origin"] == o for d in dd)
            for o in ZERO_ORIGINS
        }
    )
    return a


def summaries(old, anchors):
    result = {}
    for r in old:
        key = tuple(r[k] for k in GROUP)
        assert key not in result
        p, n, arm, mode, category = key
        aa = [
            a
            for a in anchors
            if (a["partition"], a["noise"], a["variant"]) == (p, n, arm)
            and (mode == "all_modes" or a["mode"] == mode)
            and (category == "all" or a["anchor_category"] == category)
        ]
        returned = [a for a in aa if core.truth(a["returned"] or "False")]
        mass = lambda rr: sum((a["census_case_weight"] for a in rr), Fraction())
        row = dict(r)
        for name in REFS:
            row[name] = sum(a[name] or 0 for a in aa)
        for name in HAS:
            chosen = [a for a in aa if a[name]]
            row.update(
                {
                    name + "_events": len(chosen),
                    name + "_case_mass": mass(chosen),
                    name
                    + "_fraction": (
                        Fraction(len(chosen), len(returned)) if returned else None
                    ),
                    name
                    + "_case_mass_fraction": (
                        mass(chosen) / mass(returned) if returned else None
                    ),
                }
            )
        for field, value in (
            ("census_case_mass", mass(aa)),
            ("returned_case_mass", mass(returned)),
            (
                "received_case_mass",
                sum((a["received_case_weight"] for a in aa), Fraction()),
            ),
            ("rp_case_mass", sum((a["rp_case_mass"] for a in aa), Fraction())),
        ):
            core.check(r[field], value)
        result[key] = row
    assert len(result) == 336
    return result


def closures(rows):
    checks = 0
    fields = (*REFS, *(h + s for h in HAS for s in ("_events", "_case_mass")))
    for key, r in rows.items():
        for i, whole, parts in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", CATEGORIES),
        ):
            if key[i] == whole:
                for field in fields:
                    assert r[field] == sum(
                        rows[key[:i] + (p,) + key[i + 1 :]][field] for p in parts
                    ), (key, field)
                    checks += 1
        for p, b in product(PHASES, BRANCHES):
            prefix = p + "_" + b + "_"
            assert sum(r[prefix + o + "_refs"] for o in ORIGINS) == int(
                r[prefix + "all_references"]
            )
            assert (
                sum(r[prefix + "initial_" + m + "_refs"] for m in MASKS)
                == r[prefix + "initial_unsafe_refs"]
            )
    return checks


def compare(path, rows, keys):
    expected = {tuple(str(r[k]) for k in keys): r for r in rows}
    got = core.index(core.read(path), keys)
    assert got.keys() == expected.keys()
    for key, row in got.items():
        core.fields(row, expected[key])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("results", "figures", "work", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--rustc", type=Path, default=shutil.which("rustc"))
    parser.add_argument("--raw-roots", type=Path)
    args = parser.parse_args()
    if not __debug__ or args.work.exists() or args.output.exists() or not args.rustc:
        parser.error("Use new work/evidence paths, provide rustc and run without -O")
    assert (
        core.sha(BASE / "sources.json") == BASE_SHA
        and core.sha(BASE / "independent-verification.json") == PROOF_SHA
    )
    prior = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    source = json.loads((args.results / "sources.json").read_text())
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        "fusion-return-command-sign/sources.json": BASE_SHA,
        "fusion-return-command-sign/independent-verification.json": PROOF_SHA,
        "fusion-return-command-sign/protocol.md": prior["analysis"]["protocol_sha256"],
        **{
            "fusion-return-command-sign/" + k: h
            for k, h in prior["output_sha256"].items()
        },
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_command_sign.py": prior["analysis"]["script_sha256"],
    }
    assert source["analysis"]["verifier_helpers_sha256"] == HELPERS and source[
        "analysis"
    ]["code_sha256"] == {**core.CODE, TYPES: TYPES_SHA}
    assert (
        source["raw"] == prior["raw"]
        and source["map_sha256"] == MAP_SHA
        and source["inherited_command_proof_sha256"] == PROOF_SHA
        and source["inherited_validation"] == prior["inherited_validation"]
    )
    assert source["schema"] == dict(
        half_unit=5e-10,
        numerical_guard=1e-12,
        origins=list(ORIGINS),
        phases=list(PHASES),
        branches=list(BRANCHES),
        retrospective=True,
        first_chord="inferred only from source zero plus certainly safe initial test; no chord replay",
        inherited="preceding anchor/summary/admission/timer/command-domain fields and proof",
    )
    for name, h in source["input_sha256"].items():
        assert core.sha(RESULTS / name) == h, name
    for name, h in {**source["analysis"]["helpers_sha256"], **HELPERS}.items():
        assert core.sha(ROOT / name) == h, name
    for name, h in source["analysis"]["code_sha256"].items():
        assert core.sha(core.REPO / name) == h, name
    assert source["analysis"]["source_contract"] == contract(
        (core.REPO / next(iter(core.CODE))).read_text(), (core.REPO / TYPES).read_text()
    )
    assert source["analysis"]["script_sha256"] == core.sha(
        ROOT / "analyze_probe_origin.py"
    ) and source["analysis"]["protocol_sha256"] == core.sha(
        RESULTS / "fusion-return-probe-origin/protocol.md"
    )
    assert set(source["output_sha256"]) == {
        "probe-decisions.csv",
        "anchor-probe-origin.csv",
        "probe-summary.csv",
        "obstacle-map.csv",
    }
    for name, h in source["output_sha256"].items():
        assert core.sha(args.results / name) == h, name
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    compiler = subprocess.run(
        [str(args.rustc), "--version"], check=True, capture_output=True, text=True
    ).stdout.strip()
    assert compiler == proof["native"]["rustc"]
    provenance(source["raw"], roots, compiler)
    obstacles = load_map(args.results / "obstacle-map.csv")
    cases, original = core.original_inputs()
    events = core.read(BASE / "anchor-command-sign.csv")
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
    old = core.read(BASE / "command-decisions.csv")
    core.index(old, ("event_id", "time_s"))
    points, audit = raw_inputs(cases, old, source["raw"], roots)
    decisions = [
        build(
            d,
            points[d["case_id"], core.tick(d["time_s"]) - 1],
            points[d["case_id"], core.tick(d["time_s"])],
            obstacles[d["scenario"]],
        )
        for d in old
    ]
    native_proof = native(
        args.work, args.rustc, decisions, args.results / "obstacle-map.csv"
    )
    by_event = defaultdict(list)
    for d in decisions:
        by_event[d["event_id"]].append(d)
    ne = Counter(e["case_id"] for e in original.values())
    nr = Counter(e["case_id"] for e in original.values() if core.truth(e["received"]))
    anchors = [
        anchor(e, by_event[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]])
        for e in events
    ]
    summary = summaries(core.read(BASE / "command-summary.csv"), anchors)
    compare(args.results / "probe-decisions.csv", decisions, ("event_id", "time_s"))
    compare(args.results / "anchor-probe-origin.csv", anchors, ("event_id",))
    compare(args.results / "probe-summary.csv", list(summary.values()), GROUP)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(core.truth(a["returned"] or "False") for a in anchors),
        decision_references=len(decisions),
        available_references=sum(d["phase"] == "available" for d in decisions),
        next_loss_references=sum(d["phase"] == "next_loss" for d in decisions),
        summary_rows=len(summary),
        new_additive_closure_comparisons=closures(summary),
        obstacle_records=sum(map(len, obstacles.values())),
        obstacle_comparisons=sum(d["obstacle_records_checked"] for d in decisions),
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
    assert plots["script_sha256"] == core.sha(ROOT / "plot_probe_origin.py")
    assert plots["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in ("sources.json", "anchor-probe-origin.csv", "probe-summary.csv")
    }
    assert set(plots["output_sha256"]) == {
        "probe-origin.svg",
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
        inherited_command_proof_sha256=PROOF_SHA,
        script_sha256=core.sha(Path(__file__)),
        sources_sha256=core.sha(args.results / "sources.json"),
        plot_sources_sha256=core.sha(args.figures / "plot-sources.json"),
        method="independent raw grids, Decimal rectangles and squared obstacle distances, original census measures, unchanged native initial guard at rounded centers; inherited old columns; no analyzers or chord replay",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
