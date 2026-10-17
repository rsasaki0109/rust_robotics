#!/usr/bin/env python3
"""Independently verify recovery geometry with Decimal and original Rust."""

import argparse
from collections import Counter, defaultdict
from decimal import Decimal as D, localcontext
from fractions import Fraction
from itertools import product
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess

import verify_first_chord as standalone

core = standalone.core
ROOT, REPO, RESULTS = core.ROOT, core.REPO, core.RESULTS
BASE = RESULTS / "fusion-return-first-chord"
STUDY = RESULTS / "fusion-return-recovery-geometry"
BASE_SHA = "8e2697be1401ca6b576a9fe75163e5823d25474e7a352bb5b49e05fd9d2ee6c4"
PROOF_SHA = "ad7182e599dc7d494c716f4ab7c06770a731833dddf06cd5e798f28edcdf6ec1"
ADMISSION = "fusion-return-admission/admission-decisions.csv"
META, KEY = core.META, core.KEY
GROUP = standalone.GROUP
PHASES, BRANCHES, CATEGORIES = (
    standalone.PHASES,
    standalone.BRANCHES,
    standalone.CATEGORIES,
)
STATES = ("clear", "blocked", "unknown")
KINDS = (
    "audited",
    "blockers_clear",
    "blockers_blocked",
    "blockers_unknown",
    "allmap_clear",
    "allmap_blocked",
    "allmap_unknown",
    "prior_unknown_clear",
    "nominal_only_reset_clear",
)
REFS = tuple(
    f"{p}_{b}_recovery_{k}_refs" for p, b in product(PHASES, BRANCHES) for k in KINDS
)
HAS = tuple("has_available_recovery_blockers_" + s for s in STATES)
INPUTS = (
    "recovery_x",
    "recovery_y",
    "recovery_yaw",
    "recovery_clearance",
    "recovery_sample_count",
)
FIELDS = (
    *INPUTS,
    "original_blocker_ids",
    *(f"recovery_minus_raw_{a}_{s}" for a in ("x", "y") for s in ("lo", "hi")),
    "pose_displacement_lo",
    "pose_displacement_hi",
    *(f"recovery_endpoint_{a}_{s}" for a in ("x", "y") for s in ("center", "lo", "hi")),
    "recovery_segment_radius",
    *(
        f"{k}_gap_{s}"
        for k in ("blocker_start", "blocker_segment", "allmap")
        for s in ("lo", "hi")
    ),
    "allmap_clearance_lo",
    "allmap_clearance_hi",
    "blocker_start_state",
    "blocker_segment_state",
    "allmap_state",
    "recovery_definite_obstacle_ids",
    "recovery_ambiguous_obstacle_ids",
)
H, P, S = D("5e-10"), D("1e-12"), D("1e-9")
L = D.from_float(0.6)
HELPERS = {
    **standalone.HELPERS,
    "verify_first_chord.py": "66545fd3d1f44f732ffa11ef51b70fed6ee47636bdd22dcfe0ab4fbb58a4b263",
}


def contract(source):
    recovery = "".join(source.split("let recovery_margin = ")[1].split(";")[0].split())
    speed = "".join(
        source.split("let speed_margin = if anticipatory_speed {")[1]
        .split("} else")[0]
        .split()
    )
    assert recovery == speed
    for text in (
        "speed_free_distance(sensed, steering, speed_margin, obs)",
        "if averaged_recovery && recovery_sample_count > 0",
        "recovery_samples.iter().map(|p| p.0).sum::<f64>() / n",
        "recovery_samples.iter().map(|p| p.1).sum::<f64>() / n",
        "sin_yaw.atan2(cos_yaw)",
        "(sensed.x, sensed.y, sensed.yaw)",
    ):
        assert text in source
    fragment = source.split("            let preview_end = Point2D::new(", 1)[1].split(
        "            let recovery_margin = ", 1
    )[0]
    fragment = "            let preview_end = Point2D::new(" + fragment
    assert (
        "recovery_x + 0.6 * recovery_yaw.cos()" in fragment
        and "recovery_y + 0.6 * recovery_yaw.sin()" in fragment
    )
    assert ".fold(f64::INFINITY, f64::min)" in fragment
    return dict(
        common_margin_sha256=hashlib.sha256(recovery.encode()).hexdigest(),
        recovery_fragment_sha256=hashlib.sha256(fragment.encode()).hexdigest(),
    )


def classify(lo, hi):
    assert lo <= hi
    if lo > 0:
        return "clear"
    if hi <= 0:
        return "blocked"
    return "unknown"


def witness_ids(d):
    fields = (
        ("definite_obstacle_ids", "definite_wall_ids")
        if d["probe_origin"] == "initial_unsafe"
        else ("first_definite_obstacle_ids", "first_definite_wall_ids")
    )
    assert d["probe_origin"] in ("initial_unsafe", "first_chord_by_exclusion")
    if d["probe_origin"] == "first_chord_by_exclusion":
        assert d["direct_first_test_state"] == "unsafe"
    assert d[fields[0]] and not d[fields[1]]
    ids = d[fields[0]].split(";")
    assert len(ids) == len(set(ids))
    return ids


def geometry(d, a, obs, trace):
    with localcontext() as ctx:
        ctx.prec = 60
        ids = witness_ids(d)
        assert set(ids) <= {r[0] for r in obs}
        q = lambda k: D.from_float(float(a[k]))
        assert all(q(k).is_finite() and abs(q(k)) <= 100 for k in INPUTS[:-1])
        assert int(a["recovery_sample_count"]) in range(26)
        x, y, yaw = q("recovery_x"), q("recovery_y"), q("recovery_yaw")
        margin = D.from_float(float(d["speed_margin"]))
        assert 0 < margin < 10
        si, co = standalone.sincos(yaw)
        # Independent Taylor values, with the same frozen operation allowances.
        ranges = []
        for center, t in ((x, co), (y, si)):
            tr = (max(D(-1), t - H - P), min(D(1), t + H + P))
            ranges.append(
                (center - H + L * tr[0] - 2 * P, center + H + L * tr[1] + 2 * P)
            )
        cx, cy = (D.from_float(float(trace[k])) for k in ("endpoint_x", "endpoint_y"))
        assert all(lo <= c <= hi for (lo, hi), c in zip(ranges, (cx, cy)))
        point_radius = D(2).sqrt() * H
        end_radius = sum(
            max(abs(lo - c), abs(hi - c)) ** 2 for (lo, hi), c in zip(ranges, (cx, cy))
        ).sqrt()
        radius = max(point_radius, end_radius) + P
        out = {k: a[k] for k in INPUTS}
        out.update(original_blocker_ids=";".join(ids), recovery_segment_radius=radius)
        deltas = []
        for axis, c, r, ec in zip(("x", "y"), (x, y), ranges, (cx, cy)):
            out.update(
                {
                    f"recovery_endpoint_{axis}_{s}": v
                    for s, v in zip(("center", "lo", "hi"), (ec, *r))
                }
            )
            dl = c - H - D.from_float(float(d[f"sensed_{axis}_hi"])) - P
            dh = c + H - D.from_float(float(d[f"sensed_{axis}_lo"])) + P
            out[f"recovery_minus_raw_{axis}_lo"] = dl
            out[f"recovery_minus_raw_{axis}_hi"] = dh
            deltas.append((dl, dh))
        out["pose_displacement_lo"] = max(
            D(0),
            sum(
                (D(0) if lo <= 0 <= hi else min(abs(lo), abs(hi))) ** 2
                for lo, hi in deltas
            ).sqrt()
            - P,
        )
        out["pose_displacement_hi"] = (
            sum(max(abs(lo), abs(hi)) ** 2 for lo, hi in deltas).sqrt() + P
        )
        start_pairs, segment_pairs, all_pairs = [], [], []
        definite, ambiguous = [], []
        for oid, ox, oy in obs:
            dist = standalone.segment((ox, oy), (x, y), (cx, cy))
            pair = (max(D(0), dist - radius - S), dist + radius + S)
            all_pairs.append(pair)
            gap = (pair[0] - margin - H, pair[1] - margin + H)
            status = classify(*gap)
            if status == "blocked":
                definite.append(oid)
            if status == "unknown":
                ambiguous.append(oid)
            if oid in ids:
                segment_pairs.append(gap)
                point = ((ox - x) ** 2 + (oy - y) ** 2).sqrt()
                err = point_radius + S
                start_pairs.append((point - err - margin - H, point + err - margin + H))
        clearance = (min(p[0] for p in all_pairs), min(p[1] for p in all_pairs))
        logged = q("recovery_clearance")
        assert clearance[0] <= logged + H and clearance[1] >= logged - H
        assert clearance[0] <= D.from_float(float(trace["clearance"])) <= clearance[1]
        out.update(allmap_clearance_lo=clearance[0], allmap_clearance_hi=clearance[1])
        for name, pairs in (
            ("blocker_start", start_pairs),
            ("blocker_segment", segment_pairs),
            ("allmap", [(clearance[0] - margin - H, clearance[1] - margin + H)]),
        ):
            lo, hi = min(p[0] for p in pairs), min(p[1] for p in pairs)
            out.update(
                {
                    name + "_gap_lo": lo,
                    name + "_gap_hi": hi,
                    name + "_state": classify(lo, hi),
                }
            )
        assert not (a["gate_clearance"] == "pass" and out["allmap_state"] == "blocked")
        assert not (a["gate_clearance"] == "fail" and out["allmap_state"] == "clear")
        out.update(
            recovery_definite_obstacle_ids=";".join(definite),
            recovery_ambiguous_obstacle_ids=";".join(ambiguous),
        )
        assert set(out) == set(FIELDS)
        return out


def admission_join(old, rows):
    db = core.index(rows, ("event_id", "time_s"))
    assert db.keys() == {(d["event_id"], d["time_s"]) for d in old}
    assert len(db) == len(old)
    for d in old:
        a = db[d["event_id"], d["time_s"]]
        for k in d.keys() & a.keys():
            if k in (
                "target_speed",
                "nominal_target_speed",
                "measured_speed",
                "speed_margin",
                "time_s",
                "offset_from_return_ms",
            ):
                assert float(d[k]) == float(a[k]), (k, d[k], a[k])
                core.comparisons += 1
            else:
                core.check(d[k], a[k])
    return db


def build(d, a=None, before=None, current=None, obstacles=(), trace=None):
    audited = D(d["speed_free_distance"]) == 0
    out = dict(d, recovery_geometry_audited=audited, **dict.fromkeys(FIELDS))
    if not audited:
        assert a is None and before is None and current is None and trace is None
        return out
    for k in (*INPUTS, "speed_margin"):
        assert float(a[k]) == float(current[k]), k
        core.comparisons += 1
        if k in d:
            assert float(d[k]) == float(current[k]), k
            core.comparisons += 1
    for k in META:
        if k in current:
            core.check(d[k], current[k])
    core.check(d["prior_truth_time_s"], float(before["time_s"]))
    core.check(d["observation_time_s"], float(current["time_s"]))
    for axis in ("x", "y"):
        core.check(d["prior_truth_" + axis], before[axis])
        for prefix in ("noise_", "bias_"):
            core.check(d[prefix + axis], current[prefix + axis])
        center = (float(before[axis]) + float(current["noise_" + axis])) + float(
            current["bias_" + axis]
        )
        core.check(d[f"sensed_{axis}_center"], center)
        core.check(d[f"sensed_{axis}_lo"], center - 1.501e-9)
        core.check(d[f"sensed_{axis}_hi"], center + 1.501e-9)
    out.update(geometry(d, a, obstacles, trace))
    return out


def native(work, rustc, selected, admissions, map_path):
    source = (REPO / next(iter(core.CODE))).read_text()
    types = (REPO / standalone.TYPES).read_text()
    fragment = source[
        source.index("            let preview_end = Point2D::new(") : source.index(
            "            let recovery_margin = "
        )
    ]
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
    harness += core.block(source, "fn point_segment_distance(")
    harness += (
        "\nfn recovery(recovery_x:f64,recovery_y:f64,recovery_yaw:f64,obs:&[Point2D])->(Point2D,f64){\n"
        + fragment
        + "(preview_end,recovery_clearance)\n}\n"
    )
    harness += r"""
fn main(){
 let args:Vec<_>=std::env::args().collect();
 let mut map=std::collections::BTreeMap::<String,Vec<Point2D>>::new();
 for line in std::fs::read_to_string(&args[1]).unwrap().lines().skip(1){let f:Vec<_>=line.split(',').collect();map.entry(f[0].to_owned()).or_default().push(Point2D::new(f[1].parse().unwrap(),f[2].parse().unwrap()));}
 println!("row_id,endpoint_x,endpoint_y,clearance");
 for line in std::fs::read_to_string(&args[2]).unwrap().lines().skip(1){let f:Vec<_>=line.split('\t').collect();let(e,d)=recovery(f[4].parse().unwrap(),f[5].parse().unwrap(),f[6].parse().unwrap(),&map[f[3]]);println!("{},{:.17e},{:.17e},{:.17e}",f[0],e.x,e.y,d);}
}
#[cfg(test)] mod tests {
 use super::*;
 #[test] fn straight(){let(e,d)=recovery(1.,2.,0.,&[Point2D::new(1.3,3.)]);assert_eq!(e.x,1.6);assert_eq!(e.y,2.);assert!((d-1.).abs()<1e-15);}
 #[test] fn rotated(){let(e,d)=recovery(1.,2.,std::f64::consts::FRAC_PI_2,&[Point2D::new(2.,2.3)]);assert!((e.x-1.).abs()<1e-15);assert_eq!(e.y,2.6);assert!((d-1.).abs()<1e-15);}
 #[test] fn segment_interior(){assert_eq!(point_segment_distance(Point2D::new(0.3,1.),Point2D::new(0.,0.),Point2D::new(0.6,0.)),1.);}
 #[test] fn endpoints(){assert_eq!(point_segment_distance(Point2D::new(-1.,0.),Point2D::new(0.,0.),Point2D::new(0.6,0.)),1.);assert_eq!(point_segment_distance(Point2D::new(1.6,0.),Point2D::new(0.,0.),Point2D::new(0.6,0.)),1.);}
 #[test] fn duplicates(){let p=Point2D::new(1.3,3.);assert_eq!(recovery(1.,2.,0.,&[p,p]).1,1.);}
 #[test] fn inclusive_contact(){assert!(recovery(0.,0.,0.,&[Point2D::new(0.3,0.5)]).1<=0.5);}
}
"""
    work.mkdir(parents=True)
    script = work / "recovery-geometry.rs"
    script.write_text(harness)
    for flags, name in ((["--test"], "native-tests"), ([], "native-trace")):
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
    output = subprocess.run(
        [str(work / "native-tests")], check=True, capture_output=True, text=True
    ).stdout
    assert "6 passed; 0 failed" in output
    (work / "native-tests.txt").write_text(output)
    query = work / "native-inputs.tsv"
    with query.open("w") as f:
        f.write(
            "row_id\tevent_id\ttime_s\tscenario\trecovery_x\trecovery_y\trecovery_yaw\n"
        )
        for i, d in enumerate(selected, 1):
            a = admissions[d["event_id"], d["time_s"]]
            f.write(
                f"{i}\t{d['event_id']}\t{d['time_s']}\t{d['scenario']}\t{float(a['recovery_x']):.17e}\t{float(a['recovery_y']):.17e}\t{float(a['recovery_yaw']):.17e}\n"
            )
    text = subprocess.run(
        [str(work / "native-trace"), str(map_path), str(query)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    trace = work / "native-trace.csv"
    trace.write_text(text)
    rows = core.read(trace)
    assert [int(r["row_id"]) for r in rows] == list(range(1, len(selected) + 1))
    return rows, dict(
        rustc=subprocess.run(
            [str(rustc), "--version"], check=True, capture_output=True, text=True
        ).stdout.strip(),
        harness_sha256=core.sha(script),
        inputs_sha256=core.sha(query),
        trace_sha256=core.sha(trace),
        native_tests=6,
        traced_centers=len(rows),
        method="unchanged recovery fragment and original point/segment distance; rounded logged poses, no closed-loop replay",
    )


def anchor(e, dd, ne, nr):
    # Independent original denominators, computed before any geometry selection.
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
    for k, v in weights.items():
        core.check(e[k], v)
    out = dict(e, **weights, **dict.fromkeys((*REFS, *HAS)))
    if not core.truth(e["returned"] or "False"):
        assert not dd
        return out
    assert all(all(d[k] == e[k] for k in META) for d in dd)
    for p, b in product(PHASES, BRANCHES):
        rows = [
            d
            for d in dd
            if d["phase"] == p and d["branch"] == b and d["recovery_geometry_audited"]
        ]
        counts = {"audited": len(rows)}
        for s in STATES:
            counts["blockers_" + s] = sum(d["blocker_segment_state"] == s for d in rows)
            counts["allmap_" + s] = sum(d["allmap_state"] == s for d in rows)
        counts["prior_unknown_clear"] = sum(
            d["prior_certainty"] == "unknown" and d["blocker_segment_state"] == "clear"
            for d in rows
        )
        counts["nominal_only_reset_clear"] = sum(
            d["timer_action"] == "reset"
            and d["refined_failed_gates"] == "nominal"
            and d["blocker_segment_state"] == "clear"
            for d in rows
        )
        out.update({f"{p}_{b}_recovery_{k}_refs": v for k, v in counts.items()})
    for s in STATES:
        out["has_available_recovery_blockers_" + s] = any(
            d["phase"] == "available"
            and d["recovery_geometry_audited"]
            and d["blocker_segment_state"] == s
            for d in dd
        )
    return out


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
            for k in ("blockers", "allmap"):
                assert (
                    sum(r[f"{p}_{b}_recovery_{k}_{s}_refs"] for s in STATES)
                    == r[f"{p}_{b}_recovery_audited_refs"]
                )
    return checks


def compare(path, rows, keys):
    actual = core.index(core.read(path), keys)
    expected = core.index(rows, keys)
    assert actual.keys() == expected.keys()
    for key, r in actual.items():
        assert set(r) == set(expected[key])
        for f, v in expected[key].items():
            if f in FIELDS and isinstance(v, (float, D)):
                assert math.isclose(
                    float(r[f]), float(v), rel_tol=1e-10, abs_tol=1e-9
                ), (key, f, r[f], v)
                core.comparisons += 1
            else:
                core.check(r[f], v)


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
        base_main="19afeccaef19bb2e590eeac130d42ffb0c059804",
        sources_sha256=BASE_SHA,
        proof_sha256=PROOF_SHA,
        admission_sha256="666ba8d39e81abafde1828c752ad76d1d781a9792fad8af77a761fed2e80b2bf",
        verifier_helper_sha256=HELPERS["verify_first_chord.py"],
    )
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        **{
            "fusion-return-first-chord/" + n: h
            for n, h in prior["output_sha256"].items()
        },
        "fusion-return-first-chord/sources.json": BASE_SHA,
        "fusion-return-first-chord/independent-verification.json": PROOF_SHA,
        "fusion-return-first-chord/protocol.md": prior["analysis"]["protocol_sha256"],
        "fusion-return-first-chord/freeze-pins.json": prior["analysis"][
            "freeze_sha256"
        ],
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_first_chord.py": prior["analysis"]["script_sha256"],
    }
    assert (
        source["analysis"]["verifier_helpers_sha256"] == HELPERS
        and source["analysis"]["code_sha256"] == standalone.CODE
    )
    assert source["raw"] == prior["raw"] and source["map_sha256"] == standalone.MAP_SHA
    assert (
        source["inherited_first_chord_proof_sha256"] == PROOF_SHA
        and source["inherited_validation"] == prior["inherited_validation"]
    )
    assert source["schema"] == dict(
        half_unit=5e-10,
        elemental_allowance=1e-12,
        segment_allowance=1e-9,
        phases=list(PHASES),
        branches=list(BRANCHES),
        retrospective=True,
        inherited="all prior columns and source failure stages; new recovery geometry conditional on stated numerical assumptions",
    )
    for n, h in source["input_sha256"].items():
        assert core.sha(RESULTS / n) == h, n
    for n, h in {**source["analysis"]["helpers_sha256"], **HELPERS}.items():
        assert core.sha(ROOT / n) == h, n
    for n, h in standalone.CODE.items():
        assert core.sha(REPO / n) == h, n
    assert source["analysis"]["source_contract"] == contract(
        (REPO / next(iter(core.CODE))).read_text()
    )
    for field, path in (
        ("script_sha256", ROOT / "analyze_recovery_geometry.py"),
        ("protocol_sha256", STUDY / "protocol.md"),
        ("freeze_sha256", STUDY / "freeze-pins.json"),
    ):
        assert source["analysis"][field] == core.sha(path)
    assert set(source["output_sha256"]) == {
        "recovery-geometry-decisions.csv",
        "anchor-recovery-geometry.csv",
        "recovery-geometry-summary.csv",
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
            and record["inputs_sha256"]["obstacles.csv"] == standalone.MAP_SHA
        )
        assert set(record["inputs_by_variant_sha256"]) == {"epoch", "curved"}
        assert all(
            v["obstacles.csv"] == standalone.MAP_SHA
            for v in record["inputs_by_variant_sha256"].values()
        )
    cases, original = core.original_inputs()
    events = core.read(BASE / "anchor-first-chord.csv")
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
    old = core.read(BASE / "first-chord-decisions.csv")
    admissions = admission_join(old, core.read(RESULTS / ADMISSION))
    selected = [d for d in old if D(d["speed_free_distance"]) == 0]
    assert len(selected) == 1096
    points, audit = standalone.raw_inputs(cases, selected, source["raw"], roots)
    map_path = RESULTS / "fusion-return-probe-origin/obstacle-map.csv"
    traces, native_proof = native(args.work, args.rustc, selected, admissions, map_path)
    trace_by_ref = {(d["event_id"], d["time_s"]): t for d, t in zip(selected, traces)}
    obstacles = standalone.read_map(map_path)
    decisions = [
        (
            build(
                d,
                admissions[d["event_id"], d["time_s"]],
                points[d["case_id"], core.tick(d["time_s"]) - 1],
                points[d["case_id"], core.tick(d["time_s"])],
                obstacles[d["scenario"]],
                trace_by_ref[d["event_id"], d["time_s"]],
            )
            if D(d["speed_free_distance"]) == 0
            else build(d)
        )
        for d in old
    ]
    db = defaultdict(list)
    for d in decisions:
        db[d["event_id"]].append(d)
    ne = Counter(e["case_id"] for e in original.values())
    nr = Counter(e["case_id"] for e in original.values() if core.truth(e["received"]))
    anchors = [
        anchor(e, db[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]]) for e in events
    ]
    summary = summaries(core.read(BASE / "first-chord-summary.csv"), anchors)
    for name, rows, keys in (
        ("recovery-geometry-decisions", decisions, ("event_id", "time_s")),
        ("anchor-recovery-geometry", anchors, ("event_id",)),
        ("recovery-geometry-summary", list(summary.values()), GROUP),
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
        available_audited=sum(d["phase"] == "available" for d in selected),
        boundary_audited=sum(d["phase"] == "next_loss" for d in selected),
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
    assert plots["script_sha256"] == core.sha(ROOT / "plot_recovery_geometry.py")
    assert plots["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in (
            "sources.json",
            "anchor-recovery-geometry.csv",
            "recovery-geometry-summary.csv",
        )
    }
    assert set(plots["output_sha256"]) == {
        "recovery-geometry.svg",
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
        inherited_first_chord_proof_sha256=PROOF_SHA,
        script_sha256=core.sha(Path(__file__)),
        sources_sha256=core.sha(args.results / "sources.json"),
        plot_sources_sha256=core.sha(args.figures / "plot-sources.json"),
        method="independent Decimal Taylor endpoint/Hausdorff and same-obstacle/full-map distances; original Rust recovery fragment on rounded poses; prior source stages inherited; conditional numerical allowances",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
