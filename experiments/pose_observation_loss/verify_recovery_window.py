#!/usr/bin/env python3
"""Independently replay recovery windows and bound their rounded sample means."""

import argparse
from collections import Counter, defaultdict
from decimal import Decimal as D, localcontext
from fractions import Fraction
import hashlib
from itertools import product
import json
import math
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import verify_recovery_geometry as previous

core = previous.core
ROOT, REPO, RESULTS = core.ROOT, core.REPO, core.RESULTS
BASE = RESULTS / "fusion-return-recovery-geometry"
STUDY = RESULTS / "fusion-return-recovery-window"
BASE_SHA = "2427ad221774c58a9f74183efb0f7986e33d1fe91935296f9ae858e01a281177"
PROOF_SHA = "9ef57c15aaed541cab0006cf99c87ba792f2f3d98458bbc65e10b79d87acf7b3"
META, KEY, GROUP = previous.META, previous.KEY, previous.GROUP
PHASES, BRANCHES, CATEGORIES = previous.PHASES, previous.BRANCHES, previous.CATEGORIES
H, P = D("5e-10"), D("1e-12")
R = 3 * H + 2 * P
PI = previous.standalone.PI
HELPERS = {
    **previous.HELPERS,
    "verify_recovery_geometry.py": "2bf6545663584ef03af76b8b1b6697b335f83cb7bf0a1be3f218fafb4eafc05c",
}
KINDS = (
    "audited",
    "mean",
    "fallback",
    "growing",
    "full",
    "evicted",
    "prior_unknown_mean",
    "nominal_only_reset_mean",
)
REFS = tuple(
    f"{p}_{b}_window_{k}_refs" for p, b in product(PHASES, BRANCHES) for k in KINDS
)
HAS = tuple("has_available_window_" + k for k in ("mean", "fallback", "evicted"))
RAW = tuple("prior_truth_" + a for a in ("x", "y", "yaw")) + tuple(
    p + a for p in ("noise_", "bias_") for a in ("x", "y", "yaw")
)
FIELDS = (
    "window_kind",
    "window_member_steps",
    "window_replayed_count",
    "window_last_clear_step",
    "window_last_clear_reasons",
    "window_pushes_since_clear",
    "window_evictions_since_clear",
    *(f"mean_{a}_{s}" for a in ("x", "y") for s in ("center", "lo", "hi")),
    "mean_yaw_center",
    "mean_yaw_radius",
    "mean_yaw_enclosure",
    "mean_log_x_residual",
    "mean_log_y_residual",
    "mean_log_yaw_error",
    *("rounded_mean_minus_current_" + a for a in ("x", "y")),
    *(f"{c}_delta_{a}" for c in ("truth", "noise", "bias") for a in ("x", "y")),
    *(c + "_delta_norm" for c in ("truth", "noise", "bias")),
    "decomposition_x_residual",
    "decomposition_y_residual",
)


def contract(source):
    start = source.index(
        "            if averaged_recovery {",
        source.index("let sensed = if position_sigma"),
    )
    fragment = source[
        start : source.index("            let preview_end = Point2D::new(")
    ]
    for text in (
        "correction_received",
        "precision_activation",
        "(correction && !correction_received)",
        "sensed.v.abs() > 0.02",
        "observation_age > 1.5",
        "safety_paused && sensed.v.abs() <= 0.02 && observation_age <= 1.5",
        "recovery_samples.len() > 25",
        "recovery_samples.pop_front()",
        "recovery_samples.push_back((sensed.x, sensed.y, sensed.yaw))",
        "if averaged_recovery && recovery_sample_count > 0",
        "sin_yaw.atan2(cos_yaw)",
        "(sensed.x, sensed.y, sensed.yaw)",
    ):
        assert text in fragment
    order = [
        fragment.index(t)
        for t in (
            "recovery_samples.clear()",
            "recovery_samples.push_back",
            "recovery_samples.pop_front()",
            "let recovery_sample_count",
        )
    ]
    assert order == sorted(order)
    assert "const DT: f64 = 0.02;" in source
    assert (
        source.index("if correction_received || precision_activation {")
        < source.index(
            "let observation_age = observation_time - last_observation_time;"
        )
        < source.index("// After a fresh correction")
    )
    return dict(
        window_fragment_sha256=hashlib.sha256(fragment.encode()).hexdigest(),
        time_arithmetic="binary64 n*0.02-last_observation_step*0.02",
    )


def atan(value):
    """Independent high precision atan; four half-angle reductions then series."""
    with localcontext() as ctx:
        ctx.prec = 80
        if value < 0:
            return -atan(-value)
        if value > 1:
            return PI / 2 - atan(1 / value)
        z = +value
        for _ in range(4):
            z = z / (1 + (1 + z * z).sqrt())
        power = total = z
        i = 0
        while True:
            i += 1
            power *= -z * z
            term = power / (2 * i + 1)
            total += term
            if abs(term) < D("1e-75"):
                break
            assert i < 500
        return +(total * 16)


def atan2(y, x):
    if x > 0:
        return atan(y / x)
    if x < 0:
        return atan(y / x) + (PI if y >= 0 else -PI)
    return PI / 2 if y > 0 else -PI / 2 if y < 0 else D(0)


def circular_error(a, b):
    delta = (a - b) % (2 * PI)
    if delta > PI:
        delta -= 2 * PI
    if delta < -PI:
        delta += 2 * PI
    return abs(delta)


def point(n, before, row, epoch):
    result = {"prior_truth_" + axis: before[axis] for axis in ("x", "y", "yaw")}
    result.update({k: row[k] for k in RAW if not k.startswith("prior_truth_")})
    assert all(D(v).is_finite() and abs(D(v)) <= 100 for v in result.values())
    centers = {
        axis: (float(before[axis]) + float(row["noise_" + axis]))
        + float(row["bias_" + axis])
        for axis in ("x", "y", "yaw")
    }
    assert all(abs(v) <= 100 for v in centers.values())
    return dict(
        source_step=n,
        source_epoch_step=epoch,
        **result,
        **{"sensed_" + axis + "_center": v for axis, v in centers.items()},
    )


class Replay:
    """Store admissions since the last clear, then select their last 25."""

    def __init__(self, initial):
        self.admissions = []
        self.observation_step = 0
        self.post_pause = core.truth(initial["safety_paused"], raw=True)
        self.clear_step = 0
        self.reasons = ""

    def advance(self, n, before, row):
        correction = core.truth(row["correction"], raw=True)
        received = core.truth(row["correction_received"], raw=True)
        precision = core.truth(row["precision_activation"], raw=True)
        assert not received or correction
        assert core.truth(row["stationary_epoch_fusion"], raw=True)
        if received or precision:
            self.observation_step = n
        age = (float(n) * float(0.02)) - (float(self.observation_step) * float(0.02))
        assert abs(age - float(row["observation_age"])) <= 5e-7 + 1e-12
        speed = float(row["measured_speed"])
        assert math.isfinite(speed)
        paused = self.post_pause or (correction and not received)
        conditions = {
            "received": received,
            "precision": precision,
            "missed": correction and not received,
            "motion": abs(speed) > 0.02,
            "age": age > 1.5,
        }
        if any(conditions.values()):
            self.admissions = []
            self.clear_step = n
            self.reasons = ";".join(
                k
                for k in ("received", "precision", "missed", "motion", "age")
                if conditions[k]
            )
        p = point(n, before, row, self.observation_step)
        if paused and abs(speed) <= 0.02 and age <= 1.5:
            self.admissions.append(p)
        members = self.admissions[-25:]
        assert len(members) == int(row["recovery_sample_count"]), (
            n,
            len(members),
            row["recovery_sample_count"],
        )
        self.post_pause = core.truth(row["safety_paused"], raw=True)
        return p, members, age, paused


def reconstruct(replay, p, members, row, trace):
    with localcontext() as ctx:
        ctx.prec = 60
        effective = members if members else [p]
        n = D(len(effective))
        out = dict(
            window_kind="mean" if members else "fallback",
            window_member_steps=";".join(str(s["source_step"]) for s in members),
            window_replayed_count=len(members),
            window_last_clear_step=replay.clear_step,
            window_last_clear_reasons=replay.reasons,
            window_pushes_since_clear=len(replay.admissions),
            window_evictions_since_clear=max(0, len(replay.admissions) - 25),
        )
        q = lambda value: D.from_float(float(value))
        for axis in ("x", "y"):
            centers = [q(s["sensed_" + axis + "_center"]) for s in effective]
            center = sum(centers, D(0)) / n
            radius = R + 2 * P if members else R
            lo, hi = center - radius, center + radius
            native = q(trace["mean_" + axis])
            logged = q(row["recovery_" + axis])
            assert lo <= native <= hi and lo <= logged + H and hi >= logged - H
            out.update(
                {
                    f"mean_{axis}_center": center,
                    f"mean_{axis}_lo": lo,
                    f"mean_{axis}_hi": hi,
                    f"mean_log_{axis}_residual": center - logged,
                }
            )
            difference = center - q(p["sensed_" + axis + "_center"])
            out["rounded_mean_minus_current_" + axis] = difference
            for comp, prefix in (
                ("truth", "prior_truth_"),
                ("noise", "noise_"),
                ("bias", "bias_"),
            ):
                out[f"{comp}_delta_{axis}"] = sum(
                    (q(s[prefix + axis]) for s in effective), D(0)
                ) / n - q(p[prefix + axis])
            residual = difference - sum(
                (out[f"{comp}_delta_{axis}"] for comp in ("truth", "noise", "bias")),
                D(0),
            )
            assert abs(residual) <= P
            out[f"decomposition_{axis}_residual"] = residual
        for comp in ("truth", "noise", "bias"):
            out[comp + "_delta_norm"] = (
                out[comp + "_delta_x"] ** 2 + out[comp + "_delta_y"] ** 2
            ).sqrt()
        if members:
            pairs = [
                previous.standalone.sincos(q(s["sensed_yaw_center"])) for s in members
            ]
            sine = sum((s for s, c in pairs), D(0)) / n
            cosine = sum((c for s, c in pairs), D(0)) / n
            vr = D(2).sqrt() * (R + 3 * P)
            norm = (sine * sine + cosine * cosine).sqrt()
            bounded = norm > 0 and vr <= norm / 2
            yaw = atan2(sine, cosine) if bounded else D(0)
            radius = 2 * vr / norm + P if bounded else PI
        else:
            yaw = q(p["sensed_yaw_center"])
            radius = R
            bounded = True
        error = circular_error(yaw, q(row["recovery_yaw"]))
        assert error <= radius + H + P
        assert circular_error(yaw, q(trace["mean_yaw"])) <= radius + P
        assert int(trace["sample_count"]) == len(members)
        out.update(
            mean_yaw_center=yaw,
            mean_yaw_radius=radius,
            mean_yaw_enclosure="bounded_circle" if bounded else "full_circle",
            mean_log_yaw_error=error,
        )
        assert set(out) == set(FIELDS)
        return out, effective


def lineage(d, fields, effective):
    rows = []
    for index, p in enumerate(effective, 1):
        row = {k: d[k] for k in (*META, "event_id", "time_s", "phase", "branch")}
        row.update(
            sample_role=(
                "retained" if fields["window_kind"] == "mean" else "raw_fallback"
            ),
            sample_order=index if fields["window_kind"] == "mean" else 0,
            source_time_s=p["source_step"] / 50,
        )
        row.update(p)
        rows.append(row)
    return rows


def raw_inputs(cases, selected, raw, roots, work):
    files = defaultdict(dict)
    wanted = defaultdict(list)
    for c in cases:
        files[c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"][
            c["scenario"], c["planner"]
        ] = c
    for i, d in enumerate(selected, 1):
        wanted[d["case_id"], core.tick(d["time_s"])].append((i, d))
    selected_cases = {d["case_id"] for d in selected}
    assert set(files) == {(c, n) for c, s in raw.items() for n in s["raw_sha256"]}
    snapshots = {}
    audit = Counter()
    work.mkdir(parents=True)
    with (work / "native-replay-inputs.tsv").open("w") as wire:
        wire.write(
            "case_id\tstep\tpaused\tcorrection\treceived\tprecision\tspeed\tage\tx\ty\tyaw\tlogged_count\trow_ids\n"
        )
        for i, ((cohort, name), cases_by_identity) in enumerate(
            sorted(files.items()), 1
        ):
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
                    assert identity in cases_by_identity
                    c = cases_by_identity[identity]
                    n = core.tick(cells[pos["time_s"]])
                    assert n == last[identity] + 1
                    last[identity] = n
                    if not core.truth(c["selected_timeout"]):
                        continue
                    audit["timeout_grid_rows"] += 1
                    if n == 0:
                        sigmas = [
                            D(cells[pos[k]])
                            for k in (
                                "effective_position_sigma",
                                "effective_yaw_sigma_rad",
                            )
                        ]
                        assert all(v.is_finite() and v >= 0 for v in sigmas) and any(
                            v > 0 for v in sigmas
                        )
                        audit["noisy_branch_histories"] += 1
                    if c["case_id"] not in selected_cases:
                        continue
                    row = dict(zip(header, cells))
                    if n == 0:
                        replays[identity] = Replay(row)
                        audit["replayed_histories"] += 1
                    else:
                        replay = replays[identity]
                        p, members, age, paused = replay.advance(
                            n, before[identity], row
                        )
                        audit["replayed_steps"] += 1
                        refs = wanted.get((c["case_id"], n), [])
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
                                format(p["sensed_" + axis + "_center"], ".17e")
                                for axis in ("x", "y", "yaw")
                            ),
                            row["recovery_sample_count"],
                            ";".join(str(i) for i, d in refs),
                        ]
                        wire.write("\t".join(values) + "\n")
                        for row_id, d in refs:
                            for k in (*previous.INPUTS, "speed_margin"):
                                assert float(d[k]) == float(row[k]), k
                            for axis in ("x", "y"):
                                core.check(
                                    d["sensed_" + axis + "_center"],
                                    p["sensed_" + axis + "_center"],
                                )
                                for prefix in ("noise_", "bias_"):
                                    core.check(d[prefix + axis], row[prefix + axis])
                                core.check(
                                    d["prior_truth_" + axis], before[identity][axis]
                                )
                            snapshots[d["event_id"], d["time_s"]] = dict(
                                row_id=row_id,
                                current=dict(row),
                                point=p,
                                members=list(members),
                                clear_step=replay.clear_step,
                                reasons=replay.reasons,
                                pushes=len(replay.admissions),
                            )
                    before[identity] = row
            assert set(last) == set(cases_by_identity)
            for identity, c in cases_by_identity.items():
                assert last[identity] == (
                    9000
                    if core.truth(c["selected_timeout"])
                    else core.tick(c["time_s"])
                )
            audit["raw_files"] += 1
            if i % 18 == 0:
                print(f"Independent recovery windows read: {i}/108 files", flush=True)
    assert snapshots.keys() == {(d["event_id"], d["time_s"]) for d in selected}
    assert audit["replayed_histories"] == len(selected_cases)
    return snapshots, dict(audit)


def native(work, rustc, selected):
    source = (REPO / next(iter(core.CODE))).read_text()
    vehicle = (REPO / "crates/rust_robotics_control/src/pure_pursuit.rs").read_text()
    start = source.index(
        "            if averaged_recovery {",
        source.index("let sensed = if position_sigma"),
    )
    fragment = source[
        start : source.index("            let preview_end = Point2D::new(")
    ]
    harness = (
        "#![allow(dead_code)]\nuse std::collections::{BTreeMap,VecDeque};\nuse std::io::BufRead;\n"
        + next(s for s in source.splitlines() if s.startswith("const DT:"))
        + "\n"
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
        "fn apply(recovery_samples:&mut VecDeque<(f64,f64,f64)>,safety_paused:bool,correction:bool,correction_received:bool,precision_activation:bool,observation_age:f64,sensed:VehicleState)->(usize,f64,f64,f64){let averaged_recovery=true;\n"
        + fragment
        + "(recovery_sample_count,recovery_x,recovery_y,recovery_yaw)\n}\n"
    )
    harness += r"""
fn main(){
 let args:Vec<_>=std::env::args().collect();
 let file=std::fs::File::open(&args[1]).unwrap();let mut buffers=BTreeMap::<String,VecDeque<(f64,f64,f64)>>::new();let mut steps=0usize;
 println!("row_id,step,sample_count,mean_x,mean_y,mean_yaw");
 for line in std::io::BufReader::new(file).lines().skip(1){let line=line.unwrap();let f:Vec<_>=line.split('\t').collect();assert_eq!(f.len(),13);let b=buffers.entry(f[0].to_owned()).or_default();
 let state=VehicleState::new(f[8].parse().unwrap(),f[9].parse().unwrap(),f[10].parse().unwrap(),f[6].parse().unwrap(),0.4);
 let(n,x,y,yaw)=apply(b,f[2].parse().unwrap(),f[3].parse().unwrap(),f[4].parse().unwrap(),f[5].parse().unwrap(),f[7].parse().unwrap(),state);
 assert_eq!(n,f[11].parse::<usize>().unwrap());steps+=1;
 if !f[12].is_empty(){for id in f[12].split(';'){println!("{},{},{},{:.17e},{:.17e},{:.17e}",id,f[1],n,x,y,yaw);}}
 }eprintln!("{}",steps);
}
#[cfg(test)] mod tests {
 use super::*;
 fn s(x:f64,yaw:f64,v:f64)->VehicleState{VehicleState::new(x,2.,yaw,v,0.4)}
 #[test] fn each_clear_reason(){for(c,r,p,v,a) in [(true,true,false,0.,0.),(false,false,true,0.,0.),(true,false,false,0.,0.),(false,false,false,0.02000000001,0.),(false,false,false,0.,1.50000000001)]{let mut b=VecDeque::from([(1.,2.,0.)]);assert_eq!(apply(&mut b,false,c,r,p,a,s(3.,0.,v)).0,0);}}
 #[test] fn simultaneous_clear(){let mut b=VecDeque::from([(1.,2.,0.)]);assert_eq!(apply(&mut b,false,true,true,true,2.,s(3.,0.,0.03)).0,0);}
 #[test] fn clear_then_append(){let mut b=VecDeque::from([(1.,2.,0.)]);let(n,x,_,_)=apply(&mut b,true,true,true,true,0.,s(3.,0.,0.));assert_eq!((n,x),(1,3.));}
 #[test] fn inclusive_edges(){let mut b=VecDeque::new();assert_eq!(apply(&mut b,true,false,false,false,1.5,s(3.,0.,0.02)).0,1);assert_eq!(apply(&mut b,true,false,false,false,1.5+f64::EPSILON,s(3.,0.,0.)).0,0);}
 #[test] fn capacity_and_order(){let mut b=VecDeque::new();for x in 0..26{apply(&mut b,true,false,false,false,0.,s(x as f64,0.,0.));}assert_eq!(b.front().unwrap().0,1.);assert_eq!(apply(&mut b,false,false,false,false,0.,s(99.,0.,0.)).1,13.);}
 #[test] fn post_release_retains_window(){let mut b=VecDeque::new();assert_eq!(apply(&mut b,true,false,false,false,0.,s(3.,0.,0.)).0,1);assert_eq!(apply(&mut b,false,false,false,false,0.,s(5.,0.,0.)).1,3.);}
 #[test] fn circular_wrap(){let mut b=VecDeque::from([(1.,2.,std::f64::consts::PI-0.01),(1.,2.,-std::f64::consts::PI+0.01)]);assert!((apply(&mut b,false,false,false,false,0.,s(3.,0.,0.)).3.abs()-std::f64::consts::PI).abs()<1e-12);}
 #[test] fn empty_fallback(){let mut b=VecDeque::new();assert_eq!(apply(&mut b,false,false,false,false,0.,s(3.,0.7,0.)),(0,3.,2.,0.7));}
 #[test] fn binary64_age_edge(){let age=(0..8926).map(|i|((i+75) as f64*DT)-(i as f64*DT)).find(|a|*a>1.5).unwrap();assert_eq!(format!("{:.6}",age),"1.500000");let mut b=VecDeque::from([(1.,2.,0.)]);assert_eq!(apply(&mut b,true,false,false,false,age,s(3.,0.,0.)).0,0);}
}
"""
    work.mkdir(parents=True, exist_ok=True)
    path = work / "recovery-window.rs"
    assert not path.exists()
    path.write_text(harness)
    for flags, name in ((["--test"], "native-tests"), ([], "native-replay")):
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
    assert "9 passed; 0 failed" in tests
    (work / "native-tests.txt").write_text(tests)
    wire = work / "native-replay-inputs.tsv"
    trace = subprocess.run(
        [str(work / "native-replay"), str(wire)],
        check=True,
        capture_output=True,
        text=True,
    )
    trace_path = work / "native-trace.csv"
    trace_path.write_text(trace.stdout)
    rows = core.read(trace_path)
    assert {int(r["row_id"]) for r in rows} == set(range(1, len(selected) + 1)) and len(
        rows
    ) == len(selected)
    query = work / "native-checkpoints.tsv"
    with query.open("w") as f:
        f.write("row_id\tcase_id\tevent_id\ttime_s\n")
        for i, d in enumerate(selected, 1):
            f.write(f"{i}\t{d['case_id']}\t{d['event_id']}\t{d['time_s']}\n")
    return {int(r["row_id"]): r for r in rows}, dict(
        rustc=subprocess.run(
            [str(rustc), "--version"], check=True, capture_output=True, text=True
        ).stdout.strip(),
        harness_sha256=core.sha(path),
        replay_inputs_sha256=core.sha(wire),
        checkpoints_sha256=core.sha(query),
        trace_sha256=core.sha(trace_path),
        native_tests=9,
        replayed_steps=int(trace.stderr.strip()),
        traced_references=len(rows),
        method="unchanged original window update and mean; all selected-history steps and counts, rounded sensed inputs; no closed-loop replay",
    )


def anchor(e, dd, ne, nr):
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
    for phase, branch in product(PHASES, BRANCHES):
        rows = [
            d
            for d in dd
            if d["phase"] == phase and d["branch"] == branch and d["window_audited"]
        ]
        means = [d for d in rows if d["window_kind"] == "mean"]
        values = dict(
            audited=len(rows),
            mean=len(means),
            fallback=len(rows) - len(means),
            growing=sum(d["window_replayed_count"] < 25 for d in means),
            full=sum(d["window_replayed_count"] == 25 for d in means),
            evicted=sum(d["window_evictions_since_clear"] > 0 for d in means),
            prior_unknown_mean=sum(d["prior_certainty"] == "unknown" for d in means),
            nominal_only_reset_mean=sum(
                d["timer_action"] == "reset" and d["refined_failed_gates"] == "nominal"
                for d in means
            ),
        )
        out.update({f"{phase}_{branch}_window_{k}_refs": v for k, v in values.items()})
    for name in ("mean", "fallback", "evicted"):
        rows = [d for d in dd if d["phase"] == "available" and d["window_audited"]]
        out["has_available_window_" + name] = any(
            (
                d["window_evictions_since_clear"] > 0
                if name == "evicted"
                else d["window_kind"] == name
            )
            for d in rows
        )
    return out


def closure(rows):
    count = 0
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
                    count += 1
        for phase, branch in product(PHASES, BRANCHES):
            prefix = f"{phase}_{branch}_window_"
            assert (
                row[prefix + "mean_refs"] + row[prefix + "fallback_refs"]
                == row[prefix + "audited_refs"]
            )
            assert (
                row[prefix + "growing_refs"] + row[prefix + "full_refs"]
                == row[prefix + "mean_refs"]
            )
            assert row[prefix + "evicted_refs"] <= row[prefix + "full_refs"]
    return count


def compare(path, rows, keys):
    expected = {tuple(str(r[k]) for k in keys): r for r in rows}
    actual = core.index(core.read(path), keys)
    assert len(expected) == len(rows) and actual.keys() == expected.keys()
    for key, row in actual.items():
        assert set(row) == set(expected[key])
        for field, value in expected[key].items():
            if field in FIELDS and isinstance(value, (float, D)):
                if field == "mean_yaw_center":
                    assert float(circular_error(D(row[field]), D(str(value)))) <= 1e-9
                else:
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
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("results", "figures", "work", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--rustc", type=Path, default=shutil.which("rustc"))
    parser.add_argument("--raw-roots", type=Path)
    args = parser.parse_args()
    if not __debug__ or args.work.exists() or args.output.exists() or not args.rustc:
        parser.error("Use new work/output paths, provide rustc and run without -O")
    prior = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        core.sha(BASE / "sources.json") == BASE_SHA
        and core.sha(BASE / "independent-verification.json") == PROOF_SHA
    )
    freeze = json.loads((STUDY / "freeze-pins.json").read_text())
    assert freeze == dict(
        base_main="fe8152e9331cd4a93682fbf87379d45af98253d9",
        sources_sha256=BASE_SHA,
        proof_sha256=PROOF_SHA,
        verifier_helper_sha256=HELPERS["verify_recovery_geometry.py"],
    )
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        **{
            "fusion-return-recovery-geometry/" + n: h
            for n, h in prior["output_sha256"].items()
        },
        "fusion-return-recovery-geometry/sources.json": BASE_SHA,
        "fusion-return-recovery-geometry/independent-verification.json": PROOF_SHA,
        "fusion-return-recovery-geometry/protocol.md": prior["analysis"][
            "protocol_sha256"
        ],
        "fusion-return-recovery-geometry/freeze-pins.json": prior["analysis"][
            "freeze_sha256"
        ],
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_recovery_geometry.py": prior["analysis"]["script_sha256"],
    }
    assert (
        source["analysis"]["verifier_helpers_sha256"] == HELPERS
        and source["analysis"]["code_sha256"] == previous.standalone.CODE
    )
    assert source["raw"] == prior["raw"] and source["map_sha256"] == prior["map_sha256"]
    assert (
        source["inherited_recovery_geometry_proof_sha256"] == PROOF_SHA
        and source["inherited_validation"] == prior["inherited_validation"]
    )
    assert source["schema"] == dict(
        half_unit=5e-10,
        elemental_allowance=1e-12,
        sensed_radius=3 * 5e-10 + 2 * 1e-12,
        closure_allowance=1e-12,
        phases=list(PHASES),
        branches=list(BRANCHES),
        retrospective=True,
        inherited="all prior columns and outcomes; new replay only for selected histories; rounded-input decomposition, no causal intervention",
    )
    for name, h in source["input_sha256"].items():
        assert core.sha(RESULTS / name) == h, name
    for name, h in {**source["analysis"]["helpers_sha256"], **HELPERS}.items():
        assert core.sha(ROOT / name) == h, name
    for name, h in previous.standalone.CODE.items():
        assert core.sha(REPO / name) == h, name
    assert source["analysis"]["source_contract"] == contract(
        (REPO / next(iter(core.CODE))).read_text()
    )
    for field, path in (
        ("script_sha256", ROOT / "analyze_recovery_window.py"),
        ("protocol_sha256", STUDY / "protocol.md"),
        ("freeze_sha256", STUDY / "freeze-pins.json"),
    ):
        assert source["analysis"][field] == core.sha(path)
    assert set(source["output_sha256"]) == {
        "window-decisions.csv",
        "window-lineage.csv",
        "anchor-window.csv",
        "window-summary.csv",
    }
    for name, h in source["output_sha256"].items():
        assert core.sha(args.results / name) == h, name
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
    events = core.read(BASE / "anchor-recovery-geometry.csv")
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
    old = core.read(BASE / "recovery-geometry-decisions.csv")
    selected = [d for d in old if D(d["speed_free_distance"]) == 0]
    assert len(selected) == 1096
    snapshots, audit = raw_inputs(cases, selected, source["raw"], roots, args.work)
    traces, native_proof = native(args.work, args.rustc, selected)
    assert native_proof["replayed_steps"] == audit["replayed_steps"]
    decisions = []
    lineages = []
    for d in old:
        audited = D(d["speed_free_distance"]) == 0
        out = dict(d, window_audited=audited, **dict.fromkeys(FIELDS))
        if audited:
            snap = snapshots[d["event_id"], d["time_s"]]
            trace = traces[snap["row_id"]]
            assert int(trace["step"]) == core.tick(d["time_s"])
            replay = SimpleNamespace(
                clear_step=snap["clear_step"],
                reasons=snap["reasons"],
                admissions=[None] * snap["pushes"],
            )
            fields, effective = reconstruct(
                replay, snap["point"], snap["members"], snap["current"], trace
            )
            out.update(fields)
            lineages.extend(lineage(d, fields, effective))
        decisions.append(out)
    db = defaultdict(list)
    for d in decisions:
        db[d["event_id"]].append(d)
    ne = Counter(e["case_id"] for e in original.values())
    nr = Counter(e["case_id"] for e in original.values() if core.truth(e["received"]))
    anchors = [
        anchor(e, db[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]]) for e in events
    ]
    summary = summaries(core.read(BASE / "recovery-geometry-summary.csv"), anchors)
    for name, rows, keys in (
        ("window-decisions", decisions, ("event_id", "time_s")),
        ("window-lineage", lineages, ("event_id", "time_s", "sample_order")),
        ("anchor-window", anchors, ("event_id",)),
        ("window-summary", list(summary.values()), GROUP),
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
        mean_references=sum(d.get("window_kind") == "mean" for d in decisions),
        fallback_references=sum(d.get("window_kind") == "fallback" for d in decisions),
        lineage_rows=len(lineages),
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
    assert plots["script_sha256"] == core.sha(ROOT / "plot_recovery_window.py")
    assert plots["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in ("sources.json", "anchor-window.csv", "window-summary.csv")
    }
    assert set(plots["output_sha256"]) == {
        "recovery-window.svg",
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
        atan_precision=80,
        native=native_proof,
        inherited_recovery_geometry_proof_sha256=PROOF_SHA,
        script_sha256=core.sha(Path(__file__)),
        sources_sha256=core.sha(args.results / "sources.json"),
        plot_sources_sha256=core.sha(args.figures / "plot-sources.json"),
        method="independent since-clear admissions and last-25 selection; Decimal interval means and circular atan/Taylor trig; original native window fragment on selected histories; prior fields inherited, no closed-loop replay",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
