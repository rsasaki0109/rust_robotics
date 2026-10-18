#!/usr/bin/env python3
"""Check the mean-position probe audit against the unchanged native Rust probe."""

import argparse
import csv
from decimal import Decimal, localcontext
import hashlib
import io
import json
import math
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
STUDY = ROOT / "results/fusion-return-reset-mean-probe"
BASE = ROOT / "results/fusion-return-reset-surplus/surplus.csv"
MAP = ROOT / "results/fusion-return-probe-origin/obstacle-map.csv"
SIMULATOR = REPO / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
TYPES = REPO / "crates/rust_robotics_core/src/types.rs"
VEHICLE = REPO / "crates/rust_robotics_control/src/pure_pursuit.rs"
META = (
    "edge_id",
    "case_id",
    "source_step",
    "source_time_s",
    "cohort",
    "noise",
    "variant",
    "mode",
    "seed",
    "scenario",
    "planner",
    "multiplicity",
)
ORIGINS = ("initial_unsafe", "first_chord_by_exclusion", "ambiguous_zero")
METHOD = "rounded-input, fixed-yaw/steering/margin/map raw-versus-mean full probe; no closed-loop replay"


def need(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def read(path):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        names = reader.fieldnames
        need(names and len(names) == len(set(names)), f"Invalid CSV header: {path}")
        rows = list(reader)
        need(
            all(None not in r and None not in r.values() for r in rows),
            f"Malformed CSV: {path}",
        )
        return rows


def value(text):
    result = float(text)
    need(math.isfinite(result), "Expected finite numeric evidence")
    return result


def source_block(source, marker):
    start = source.index(marker)
    opening = source.index("{", start)
    depth = 0
    for i in range(opening, len(source)):
        depth += (source[i] == "{") - (source[i] == "}")
        if depth == 0:
            return source[start : i + 1]
    raise ValueError("Unclosed pinned Rust source block")


def harness_source():
    source, types, vehicle = (
        SIMULATOR.read_text(),
        TYPES.read_text(),
        VEHICLE.read_text(),
    )
    full = source_block(source, "fn speed_free_distance(")
    initial = full.split("    let curvature", 1)[0].replace(
        "fn speed_free_distance(", "fn initial_clear(", 1
    )
    endpoint = full[
        full.index("        let length = step as f64 * 0.05;") : full.index(
            "        if [point.x"
        )
    ]
    code = (
        "#![allow(dead_code)]\n"
        + next(
            line for line in source.splitlines() if line.startswith("const WHEELBASE:")
        )
        + "\n#[derive(Clone, Copy)]\n"
        + source_block(types, "pub struct Point2D")
        + "\nimpl Point2D {\n"
        + source_block(types, "pub fn new(x: f64, y: f64)")
        + "\n"
        + source_block(types, "pub fn distance(&self, other: &Point2D)")
        + "\n}\n"
        + source_block(vehicle, "pub struct VehicleState")
        + "\nimpl VehicleState {\n"
        + source_block(
            vehicle, "pub fn new(x: f64, y: f64, yaw: f64, v: f64, wheelbase: f64)"
        )
        + "\n}\n"
        + source_block(source, "fn point_segment_distance(")
        + "\n"
        + full
        + "\n"
        + initial
        + "    1.0\n}\n"
        + "fn at_step(sensed: VehicleState, steering: f64, step: usize) -> Point2D {\n"
        + "let start = Point2D::new(sensed.x, sensed.y);\n"
        + "if step == 0 { return start; }\nlet curvature = steering.tan() / WHEELBASE;\n"
        + endpoint
        + "point\n}\n"
    )
    return code + r"""
fn main() {
    let args: Vec<_> = std::env::args().collect();
    let mut map = std::collections::BTreeMap::<String, Vec<(usize, Point2D)>>::new();
    for (i, line) in std::fs::read_to_string(&args[1]).unwrap().lines().skip(1).enumerate() {
        let f: Vec<_> = line.split(',').collect();
        map.entry(f[0].to_owned()).or_default().push((i + 1, Point2D::new(f[1].parse().unwrap(), f[2].parse().unwrap())));
    }
    println!("row_id,initial_clear,free_distance,first_unsafe_step,wall_ids,obstacle_ids");
    for line in std::fs::read_to_string(&args[2]).unwrap().lines().skip(1) {
        let f: Vec<_> = line.split('\t').collect();
        let (x, y, yaw, steering, margin): (f64, f64, f64, f64, f64) = (f[2].parse().unwrap(), f[3].parse().unwrap(), f[4].parse().unwrap(), f[5].parse().unwrap(), f[6].parse().unwrap());
        let state = || VehicleState::new(x, y, yaw, 0.0, WHEELBASE);
        let entries = map.get(f[1]).cloned().unwrap_or_default();
        let obstacles: Vec<_> = entries.iter().map(|(_, p)| *p).collect();
        let clear = initial_clear(state(), steering, margin, &obstacles) != 0.0;
        let free = speed_free_distance(state(), steering, margin, &obstacles);
        let step = if !clear { 0 } else if free < 2.0 { (free / 0.05).round() as usize + 1 } else { 41 };
        let mut walls = Vec::new();
        let mut ids = Vec::new();
        if step <= 40 {
            let point = at_step(state(), steering, step);
            let previous = at_step(state(), steering, step.saturating_sub(1));
            for (name, gap) in ["x_min", "y_min", "x_max", "y_max"].iter().zip([point.x, point.y, 50.0 - point.x, 50.0 - point.y]) {
                if gap <= margin { walls.push(*name); }
            }
            let start = Point2D::new(x, y);
            for (id, p) in entries {
                if p.distance(&start) <= 2.0 + margin && point_segment_distance(p, previous, point) <= margin { ids.push(id.to_string()); }
            }
        }
        let failed = if step <= 40 { step.to_string() } else { String::new() };
        println!("{},{},{:.17e},{},{},{}", f[0], clear, free, failed, walls.join(";"), ids.join(";"));
    }
}
"""


def native(work, rustc, queries, map_path=MAP):
    work.mkdir(parents=True)
    path = work / "reset-mean-probe.rs"
    path.write_text(harness_source())
    built = subprocess.run(
        [
            str(rustc),
            "--edition=2021",
            "-O",
            str(path),
            "-o",
            str(work / "native-probe"),
        ],
        capture_output=True,
        text=True,
    )
    (work / "compiler.log").write_text(built.stdout + built.stderr)
    built.check_returncode()
    query_path = work / "native-inputs.tsv"
    with query_path.open("w", newline="") as stream:
        writer = csv.writer(stream, delimiter="\t", lineterminator="\n")
        writer.writerow(("row_id", "scenario", "x", "y", "yaw", "steering", "margin"))
        writer.writerows(
            (
                str(i),
                row["scenario"],
                *(
                    format(value(row[k]), ".17e")
                    for k in ("x", "y", "yaw", "steering", "margin")
                ),
            )
            for i, row in enumerate(queries, 1)
        )
    traced = subprocess.run(
        [str(work / "native-probe"), str(map_path), str(query_path)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    (work / "native-trace.csv").write_text(traced)
    rows = list(csv.DictReader(io.StringIO(traced)))
    need(
        [r["row_id"] for r in rows] == [str(i) for i in range(1, len(queries) + 1)],
        "Missing/extra native rows",
    )
    return rows, dict(
        rustc=subprocess.check_output([str(rustc), "--version"], text=True).strip(),
        harness_sha256=sha(path),
        inputs_sha256=sha(query_path),
        trace_sha256=sha(work / "native-trace.csv"),
        probe_evaluations=len(rows),
        fresh_trajectories=0,
    )


def verify_inputs(inputs, pins, base):
    selected = read(inputs / "selected-inputs.csv")
    need(
        [r["edge_id"] for r in selected] == [r["edge_id"] for r in base],
        "Selected input order/identities differ",
    )
    expected_fields = {
        "edge_id",
        "raw_path",
        "prior_csv_row",
        "current_csv_row",
        "prior_truth_x",
        "prior_truth_y",
        "prior_truth_yaw",
        "noise_x",
        "noise_y",
        "noise_yaw",
        "bias_x",
        "bias_y",
        "bias_yaw",
        "nominal_steering",
        "speed_margin",
        "speed_free_distance",
        "speed_cap",
        "uncapped_target_speed",
    }
    for row, old in zip(selected, base):
        need(set(row) == expected_fields, "Unexpected selected input fields")
        need(
            row["raw_path"]
            == f"{old['cohort']}/{old['variant']}-{old['mode']}-{old['seed']}/trajectories.csv",
            "Wrong original raw file",
        )
        need(
            int(row["prior_csv_row"]) >= 2
            and int(row["current_csv_row"]) == int(row["prior_csv_row"]) + 1,
            "Invalid original row references",
        )
        for name in expected_fields - {
            "edge_id",
            "raw_path",
            "prior_csv_row",
            "current_csv_row",
        }:
            value(row[name])
            inherited = old.get(
                "decision_current_input_" + name, old.get("decision_" + name)
            )
            if inherited:
                need(row[name] == inherited, f"Logged input changed: {name}")
    extraction = json.loads((inputs / "selected-input-sources.json").read_text())
    need(
        extraction["analysis"] == expected_analysis(),
        "Extraction script/protocol differs",
    )
    need(
        extraction["input_sha256"] == pins["input_sha256"]
        and extraction["raw_sha256"] == pins["raw_sha256"],
        "Extraction provenance differs",
    )
    need(
        extraction["method"]
        == "selected original decision/predecessor raw strings; no regenerated trajectories",
        "Extraction method differs",
    )
    need(
        extraction["output_sha256"]
        == {"selected-inputs.csv": sha(inputs / "selected-inputs.csv")},
        "Selected input bytes differ",
    )
    need(
        set(extraction["raw_rows_scanned"]) == set(pins["raw_sha256"]),
        "Missing raw scan record",
    )
    for row in selected:
        need(
            extraction["raw_rows_scanned"][row["raw_path"]]
            >= int(row["current_csv_row"]) - 1,
            "Inconsistent raw scan count",
        )
    return selected


def expected_analysis():
    return dict(
        script_sha256=sha(ROOT / "analyze_reset_mean_probe.py"),
        protocol_sha256=sha(STUDY / "protocol.md"),
        freeze_sha256=sha(STUDY / "freeze-pins.json"),
    )


def check_rows(rows, base, inputs):
    expected_ids = [(old["edge_id"], arm) for old in base for arm in ("raw", "mean")]
    need(
        [(r["edge_id"], r["arm"]) for r in rows] == expected_ids,
        "Missing/duplicate/reordered probe rows",
    )
    expected_fields = {
        *META,
        "original_probe_origin",
        "original_failed_gates",
        "arm",
        "x",
        "y",
        "yaw",
        "steering",
        "margin",
        "uncapped_target_speed",
        "initial_clear",
        "first_unsafe_step",
        "wall_ids",
        "obstacle_ids",
        "free_distance",
        "speed_cap",
        "nominal_target_speed",
        "positive_distance",
        "full_horizon_clear",
    }
    for i, (old, selected) in enumerate(zip(base, inputs)):
        original = {
            k: (value(selected["prior_truth_" + k]) + value(selected["noise_" + k]))
            + value(selected["bias_" + k])
            for k in ("x", "y", "yaw")
        }
        for row in rows[2 * i : 2 * i + 2]:
            need(set(row) == expected_fields, "Unexpected probe fields")
            need(all(row[k] == old[k] for k in META), "Inherited row metadata differs")
            need(
                row["original_probe_origin"] == old["decision_probe_origin"]
                and row["original_failed_gates"] == old["failed_gates"],
                "Original origin/gate mask differs",
            )
            for k in ("x", "y", "yaw"):
                expected = (
                    value(old["decision_mean_" + k + "_center"])
                    if row["arm"] == "mean" and k != "yaw"
                    else original[k]
                )
                need(value(row[k]) == expected, f"Changed {row['arm']} probe {k}")
            for target, source in (
                ("steering", "nominal_steering"),
                ("margin", "speed_margin"),
                ("uncapped_target_speed", "uncapped_target_speed"),
            ):
                need(
                    value(row[target]) == value(selected[source]),
                    f"Changed fixed probe parameter: {target}",
                )


def check_trace(rows, traces):
    need(len(rows) == len(traces), "Native trace count differs")
    for row, trace in zip(rows, traces):
        need(
            value(row["free_distance"]) == value(trace["free_distance"]),
            "Python/native free distance differs",
        )
        need(
            row["initial_clear"].lower() == trace["initial_clear"]
            and row["initial_clear"] in ("True", "False"),
            "Python/native initial predicate differs",
        )
        for k in ("first_unsafe_step", "wall_ids", "obstacle_ids"):
            need(row[k] == trace[k], f"Python/native {k} differs")
        distance = value(trace["free_distance"])
        need(
            row["positive_distance"] == str(distance > 0),
            "Positive-distance flag differs",
        )
        need(
            row["full_horizon_clear"] == str(distance == 2.0),
            "Full-horizon flag differs",
        )
        if row["arm"] == "raw":
            need(distance == 0, "Original probe was not zero")
            need(
                trace["first_unsafe_step"]
                == {"initial_unsafe": "0", "first_chord_by_exclusion": "1"}[
                    row["original_probe_origin"]
                ],
                "Original failure stage differs",
            )
        with localcontext() as context:
            context.prec = 60
            reserve = Decimal("0.15")
            cap = min(
                Decimal(1),
                max(
                    Decimal(0),
                    (reserve * reserve + Decimal.from_float(distance)).sqrt() - reserve,
                ),
            )
            target = min(Decimal(row["uncapped_target_speed"]), cap)
            need(
                abs(Decimal.from_float(value(row["speed_cap"])) - cap)
                <= Decimal("1e-12"),
                "Speed-cap formula differs",
            )
            need(
                abs(Decimal.from_float(value(row["nominal_target_speed"])) - target)
                <= Decimal("1e-12"),
                "Nominal-target formula differs",
            )


def check_summaries(summary, rows):
    cases = ("all_cases", *sorted({r["case_id"] for r in rows}))
    expected_ids = [
        (case, origin, arm)
        for case in cases
        for origin in ("all_origins", *ORIGINS)
        for arm in ("raw", "mean")
    ]
    need(
        [(r["case_selector"], r["origin_selector"], r["arm"]) for r in summary]
        == expected_ids,
        "Summary cells differ",
    )
    for cell in summary:
        chosen = [
            r
            for r in rows
            if r["arm"] == cell["arm"]
            and (
                cell["case_selector"] == "all_cases"
                or r["case_id"] == cell["case_selector"]
            )
            and (
                cell["origin_selector"] == "all_origins"
                or r["original_probe_origin"] == cell["origin_selector"]
            )
        ]
        counts = dict(
            physical_resets=len(chosen),
            ending_multiplicity=sum(int(r["multiplicity"]) for r in chosen),
            initial_blocked=sum(r["first_unsafe_step"] == "0" for r in chosen),
            first_chord_blocked=sum(r["first_unsafe_step"] == "1" for r in chosen),
            later_chord_blocked=sum(
                bool(r["first_unsafe_step"]) and int(r["first_unsafe_step"]) > 1
                for r in chosen
            ),
            full_horizon_clear=sum(r["first_unsafe_step"] == "" for r in chosen),
            positive_distance=sum(value(r["free_distance"]) > 0 for r in chosen),
            nominal_positive=sum(value(r["nominal_target_speed"]) > 0 for r in chosen),
        )
        need(
            set(cell)
            == {
                "case_selector",
                "origin_selector",
                "arm",
                *counts,
                "max_free_distance",
            },
            "Unexpected summary fields",
        )
        need(all(cell[k] == str(v) for k, v in counts.items()), "Summary counts differ")
        if chosen:
            need(
                value(cell["max_free_distance"])
                == max(value(r["free_distance"]) for r in chosen),
                "Summary maximum differs",
            )
        else:
            need(cell["max_free_distance"] == "", "Empty summary maximum must be blank")


def verify(args):
    pins = json.loads((STUDY / "freeze-pins.json").read_text())
    for name, digest in pins["input_sha256"].items():
        need(sha(REPO / name) == digest, f"Changed frozen input: {name}")
    base = read(BASE)
    need(
        [r["edge_id"] for r in base] == pins["edge_ids"]
        and len(set(pins["edge_ids"])) == 19,
        "Original selection differs",
    )
    selected = verify_inputs(args.inputs, pins, base)
    rows = read(args.results / "reprobes.csv")
    check_rows(rows, base, selected)
    sources = json.loads((args.results / "sources.json").read_text())
    expected = dict(
        method=METHOD,
        analysis=expected_analysis(),
        input_sha256=pins["input_sha256"],
        selected_input_sha256={
            name: sha(args.inputs / name)
            for name in ("selected-inputs.csv", "selected-input-sources.json")
        },
        audit=dict(
            distinct_edges=19,
            ending_multiplicity=sum(int(r["multiplicity"]) for r in base),
            probe_evaluations=38,
            raw_file_reads=0,
            fresh_trajectories=0,
        ),
        output_sha256={
            name: sha(args.results / name)
            for name in ("reprobes.csv", "reprobe-summary.csv")
        },
    )
    need(sources == expected, "Collection provenance/output hashes differ")
    traces, native_proof = native(args.work, args.rustc, rows)
    check_trace(rows, traces)
    check_summaries(read(args.results / "reprobe-summary.csv"), rows)
    return dict(
        method="unchanged native Rust full probe and witnesses; independent 60-digit Decimal cap/target; rounded inputs, no closed-loop replay",
        verifier_sha256=sha(Path(__file__)),
        protocol_sha256=sha(STUDY / "protocol.md"),
        freeze_sha256=sha(STUDY / "freeze-pins.json"),
        input_sha256={**pins["input_sha256"], **expected["selected_input_sha256"]},
        output_sha256={
            name: sha(args.results / name)
            for name in ("reprobes.csv", "reprobe-summary.csv", "sources.json")
        },
        distinct_edges=19,
        ending_multiplicity=expected["audit"]["ending_multiplicity"],
        native=native_proof,
        summary_cells=len(read(args.results / "reprobe-summary.csv")),
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--inputs", type=Path, default=STUDY)
    parser.add_argument("--work", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rustc", type=Path, default=shutil.which("rustc"))
    args = parser.parse_args(argv)
    if not __debug__ or not args.rustc:
        parser.error("Provide rustc and run without Python optimization")
    for path in (args.work, args.output):
        if path.exists() or path.resolve().is_relative_to(REPO.resolve()):
            parser.error("Use fresh work/proof paths outside the repository")
    try:
        proof = verify(args)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x") as stream:
            stream.write(json.dumps(proof, indent=2) + "\n")
    except (
        OSError,
        ValueError,
        KeyError,
        csv.Error,
        subprocess.CalledProcessError,
    ) as error:
        parser.exit(1, f"Reset mean-probe verification failed: {error}\n")
    print(
        f"Verified {proof['native']['probe_evaluations']} native probe evaluations: {args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
