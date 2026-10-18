"""Replay the original recovery line on the pilot's full-precision means."""

import json
from pathlib import Path
import shutil
import subprocess

import build_mean_speed_probe as build
from verify_reset_mean_probe import source_block

TRACE_FIELDS = (
    "id",
    "recovery_clearance",
    "recovery_clearance_9",
    "recovery_bounds",
    "release_margin",
    "release_margin_9",
    "recovery_margin",
    "precision_active",
    "effective_position_sigma_9",
)


def source():
    original = build.BASE.read_text()
    types = (build.REPO / "crates/rust_robotics_core/src/types.rs").read_text()
    margin = """            let recovery_margin = RADIUS
                + 0.1
                + 2.0 * 2.0_f64.sqrt() * effective_position_sigma
                + drift_speed * observation_age;"""
    speed = """                RADIUS
                    + 0.1
                    + 2.0 * 2.0_f64.sqrt() * effective_position_sigma
                    + drift_speed * observation_age"""
    build.require(
        original.count(margin) == original.count(speed) == 1,
        "Recovery and logged speed margin expressions changed",
    )
    start = original.index("            let preview_end = Point2D::new(")
    end = original.index("            let mut resumed = false;", start)
    line = build.once(
        original[start:end], margin, "            let recovery_margin = margin;"
    )
    effective_start = original.index(
        "            let effective_position_sigma = if precision_active {"
    )
    effective_end = original.index(
        "            let effective_yaw_sigma =", effective_start
    )
    guard_start = original.index("            let base_preview_margin = RADIUS")
    guard_end = original.index("            // A flat +0.1", guard_start)
    guard_release = "let guard_release_margin = base_preview_margin + 0.05;"
    build.require(
        original.count(guard_release) == 1, "Guard release expression changed"
    )
    margins = (
        "fn margins(position_sigma: f64, drift_speed: f64, observation_age: f64, precision_active: bool, recovery_guard_buffer: bool) -> (f64, f64, f64) {\n"
        + original[effective_start:effective_end]
        + margin
        + "\n"
        + original[guard_start:guard_end]
        + guard_release
        + "\n(recovery_margin, guard_release_margin, effective_position_sigma)\n}\n"
    )
    return (
        "#![allow(dead_code)]\nuse std::io::{BufRead, Write};\n"
        + next(
            line for line in original.splitlines() if line.startswith("const RADIUS:")
        )
        + "\n#[derive(Clone, Copy)]\n"
        + source_block(types, "pub struct Point2D")
        + "\nimpl Point2D {\n"
        + source_block(types, "pub fn new(x: f64, y: f64)")
        + "\n"
        + source_block(types, "pub fn distance(&self, other: &Point2D)")
        + "\n}\n"
        + source_block(original, "fn point_segment_distance(")
        + "\nfn geometry(recovery_x: f64, recovery_y: f64, recovery_yaw: f64, margin: f64, obs: &[Point2D]) -> (f64, bool) {\n"
        + line
        + "(recovery_clearance, recovery_bounds)\n}\n"
        + margins
        + r"""
fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 3);
    let mut maps = std::collections::BTreeMap::<String, Vec<Point2D>>::new();
    for line in std::fs::read_to_string(&args[1]).unwrap().lines().skip(1) {
        let f: Vec<_> = line.split(',').collect();
        maps.entry(f[0].to_owned()).or_default().push(Point2D::new(f[1].parse().unwrap(), f[2].parse().unwrap()));
    }
    let mut output = std::io::BufWriter::new(std::io::stdout().lock());
    writeln!(output, "id,recovery_clearance,recovery_clearance_9,recovery_bounds,release_margin,release_margin_9,recovery_margin,precision_active,effective_position_sigma_9").unwrap();
    let input = std::io::BufReader::new(std::fs::File::open(&args[2]).unwrap());
    for line in input.lines().skip(1) {
        let line = line.unwrap();
        let f: Vec<_> = line.split('\t').collect();
        assert_eq!(f.len(), 11);
        let values: Vec<f64> = f[2..6].iter().map(|v| v.parse().unwrap()).collect();
        assert!(values.iter().all(|v| v.is_finite()));
        assert!(values[3] > 0.0);
        let precision: bool = f[7].parse().unwrap();
        let (recovery_margin, release_margin, sigma) = margins(f[8].parse().unwrap(), f[9].parse().unwrap(), f[6].parse().unwrap(), precision, f[10].parse().unwrap());
        assert_eq!(recovery_margin, values[3], "Logged recovery margin differs at query {}", f[0]);
        let (clearance, bounds) = geometry(values[0], values[1], values[2], recovery_margin, maps.get(f[1]).unwrap());
        writeln!(output, "{},{:.17e},{:.9},{},{:.17e},{:.9},{:.17e},{},{:.9}", f[0], clearance, clearance, bounds, release_margin, release_margin, recovery_margin, precision, sigma).unwrap();
    }
}
"""
    )


def compile_geometry(work):
    work = build.fresh(work)
    work.mkdir(parents=True)
    (work / "admission-geometry.rs").write_text(source())
    compiler = shutil.which("rustc")
    build.require(compiler is not None, "Activate the Rust compiler")
    command = [
        compiler,
        "--edition=2021",
        "-O",
        "admission-geometry.rs",
        "-o",
        "native-geometry",
    ]
    with (work / "compiler.log").open("w") as log:
        subprocess.run(
            command, cwd=work, stdout=log, stderr=subprocess.STDOUT, check=True
        )
    return dict(
        rustc=subprocess.check_output([compiler, "--version"], text=True).strip(),
        command=command,
        source_sha256=build.sha(work / "admission-geometry.rs"),
        binary_sha256=build.sha(work / "native-geometry"),
        compiler_log_sha256=build.sha(work / "compiler.log"),
    )


def evaluate(work, receipt, queries, count, map_path=build.FIXTURES / "obstacles.csv"):
    work = Path(work)
    build.require(
        Path(queries).resolve() == (work / "queries.tsv").resolve(),
        "Unexpected native query path",
    )
    command = ["./native-geometry", str(map_path), "queries.tsv"]
    with (work / "trace.csv").open("x") as output, (work / "native.log").open(
        "x"
    ) as log:
        subprocess.run(command, cwd=work, stdout=output, stderr=log, check=True)
    # Paths differ between fresh replays; the data hashes identify the inputs.
    receipt = dict(
        receipt,
        evaluate_command=command,
        query_sha256=build.sha(queries),
        trace_sha256=build.sha(work / "trace.csv"),
        obstacle_sha256=build.sha(map_path),
        queries=count,
        fresh_simulator_steps=0,
    )
    (work / "native-sources.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return receipt
