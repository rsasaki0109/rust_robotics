"""Trace recorded and alternative speed probes with unchanged native functions."""

from pathlib import Path
import shutil
import subprocess

import build_mean_speed_probe as build
import native_mean_probe_admission as admission
import verify_reset_mean_probe as original

INPUT_FIELDS = (
    "id",
    "scenario",
    "x",
    "y",
    "mean_yaw",
    "raw_yaw",
    "steering",
    "margin",
    "observation_age",
    "precision_active",
    "position_sigma",
    "drift_speed",
    "recovery_guard_buffer",
    "stop_lo",
    "stop_hi",
    "terminal_lo",
    "terminal_hi",
    "draw",
)
TRACE_FIELDS = (
    "id",
    "recovery_clearance",
    "recovery_clearance_9",
    "recovery_bounds",
    "margin",
    "release_margin",
    "release_margin_9",
    "actual_distance",
    "mean_yaw_distance",
    "straight_distance",
    "start_clearance",
    "start_wall_clearance",
    "line_end_x",
    "line_end_y",
    "heading_delta",
    "nearest_obstacle_ids",
    "stop_lo",
    "stop_hi",
    "terminal_release_lo",
    "terminal_release_hi",
    "release_lo",
    "release_hi",
)
WITNESS_FIELDS = (
    "id",
    "obstacle_id",
    "obstacle_x",
    "obstacle_y",
    "t",
    "closest_x",
    "closest_y",
    "distance",
)
PATH_FIELDS = ("id", "variant", "step", "x", "y")


def source():
    simulator = build.BASE.read_text()
    vehicle = original.VEHICLE.read_text()
    inherited = admission.source()
    build.require(inherited.count("fn main() {") == 1, "Native reader entry changed")
    prefix = inherited.split("fn main() {", 1)[0]
    projection = original.source_block(simulator, "fn point_segment_distance(")
    projection = build.once(projection, "fn point_segment_distance(", "fn projection(")
    projection = build.once(projection, ") -> f64 {", ") -> (f64, Point2D) {")
    projection = build.once(projection, "return p.distance(&a);", "return (0.0, a);")
    projection = build.once(
        projection,
        "p.distance(&Point2D::new(a.x + t * dx, a.y + t * dy))",
        "(t, Point2D::new(a.x + t * dx, a.y + t * dy))",
    )
    probe = original.source_block(simulator, "fn speed_free_distance(")
    build.require("sensed.v" not in probe, "Speed probe now depends on velocity")
    return (
        prefix
        + "\n"
        + next(
            line
            for line in simulator.splitlines()
            if line.startswith("const WHEELBASE:")
        )
        + "\n#[derive(Clone, Copy)]\n"
        + original.source_block(vehicle, "pub struct VehicleState")
        + "\nimpl VehicleState {\n"
        + original.source_block(
            vehicle, "pub fn new(x: f64, y: f64, yaw: f64, v: f64, wheelbase: f64)"
        )
        + "\n}\n"
        + probe
        + "\n"
        + original.source_block(simulator, "fn aligned_release_preview(")
        + "\n"
        + original.source_block(original.harness_source(), "fn at_step(")
        + "\n"
        + projection
        + "\n"
        + r"""
fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 5);
    let mut maps = std::collections::BTreeMap::<String, Vec<(usize, Point2D)>>::new();
    for (i, line) in std::fs::read_to_string(&args[1]).unwrap().lines().skip(1).enumerate() {
        let f: Vec<_> = line.split(',').collect();
        maps.entry(f[0].to_owned()).or_default().push((i + 1, Point2D::new(f[1].parse().unwrap(), f[2].parse().unwrap())));
    }
    let mut trace = std::io::BufWriter::new(std::io::stdout().lock());
    let mut witnesses = std::io::BufWriter::new(std::fs::File::create(&args[3]).unwrap());
    let mut paths = std::io::BufWriter::new(std::fs::File::create(&args[4]).unwrap());
    writeln!(trace, "ID_HEADER").unwrap();
    writeln!(witnesses, "id,obstacle_id,obstacle_x,obstacle_y,t,closest_x,closest_y,distance").unwrap();
    writeln!(paths, "id,variant,step,x,y").unwrap();
    let input = std::io::BufReader::new(std::fs::File::open(&args[2]).unwrap());
    for line in input.lines().skip(1) {
        let line = line.unwrap();
        let f: Vec<_> = line.split('\t').collect();
        assert_eq!(f.len(), 18);
        let v: Vec<f64> = f[2..9].iter().map(|x| x.parse().unwrap()).collect();
        assert!(v.iter().all(|x| x.is_finite()));
        let (margin, release_margin, _) = margins(f[10].parse().unwrap(), f[11].parse().unwrap(), v[6], f[9].parse().unwrap(), f[12].parse().unwrap());
        assert_eq!(margin, v[5], "Recorded margin differs at {}", f[0]);
        let entries = maps.get(f[1]).unwrap();
        let obstacles: Vec<_> = entries.iter().map(|(_, p)| *p).collect();
        let (clearance, bounds) = geometry(v[0], v[1], v[2], margin, &obstacles);
        let start = Point2D::new(v[0], v[1]);
        let end = Point2D::new(v[0] + 0.6 * v[2].cos(), v[1] + 0.6 * v[2].sin());
        let mut ids = Vec::new();
        for (id, point) in entries {
            if point_segment_distance(*point, start, end) == clearance {
                ids.push(id.to_string());
                let (t, closest) = projection(*point, start, end);
                assert_eq!(point.distance(&closest), clearance);
                writeln!(witnesses, "{},{},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e}", f[0], id, point.x, point.y, t, closest.x, closest.y, clearance).unwrap();
            }
        }
        assert!(!ids.is_empty());
        let start_clearance = obstacles.iter().map(|p| p.distance(&start)).fold(f64::INFINITY, f64::min);
        let start_wall = start.x.min(start.y).min(50.0 - start.x).min(50.0 - start.y);
        let mut distances = Vec::new();
        for (name, yaw, steering) in [("raw_curve", v[3], v[4]), ("mean_curve", v[2], v[4]), ("mean_straight", v[2], 0.0)] {
            let state = || VehicleState::new(v[0], v[1], yaw, 0.0, WHEELBASE);
            distances.push(speed_free_distance(state(), steering, margin, &obstacles));
            if f[17].parse::<bool>().unwrap() {
                for step in 0..=40 {
                    let point = at_step(state(), steering, step);
                    writeln!(paths, "{},{},{},{:.17e},{:.17e}", f[0], name, step, point.x, point.y).unwrap();
                }
            }
        }
        let stop_lo: f64 = f[13].parse().unwrap();
        let stop_hi: f64 = f[14].parse().unwrap();
        let term_lo: f64 = f[15].parse().unwrap();
        let term_hi: f64 = f[16].parse().unwrap();
        assert!([stop_lo, stop_hi, term_lo, term_hi].iter().all(|x| x.is_finite()));
        assert!(stop_lo <= stop_hi && term_lo <= term_hi);
        let term_release_lo = aligned_release_preview(f64::INFINITY, term_lo);
        let term_release_hi = aligned_release_preview(f64::INFINITY, term_hi);
        let release_lo = aligned_release_preview(stop_lo, term_lo);
        let release_hi = aligned_release_preview(stop_hi, term_hi);
        let delta = v[3] - v[2];
        let fields = vec![f[0].to_owned(), format!("{:.17e}", clearance), format!("{:.9}", clearance), bounds.to_string(),
            format!("{:.17e}", margin), format!("{:.17e}", release_margin), format!("{:.9}", release_margin),
            format!("{:.17e}", distances[0]), format!("{:.17e}", distances[1]), format!("{:.17e}", distances[2]),
            format!("{:.17e}", start_clearance), format!("{:.17e}", start_wall), format!("{:.17e}", end.x), format!("{:.17e}", end.y),
            format!("{:.17e}", delta.sin().atan2(delta.cos())), ids.join(";"),
            format!("{:.17e}", aligned_release_preview(stop_lo, f64::INFINITY)), format!("{:.17e}", aligned_release_preview(stop_hi, f64::INFINITY)),
            format!("{:.17e}", term_release_lo), format!("{:.17e}", term_release_hi), format!("{:.17e}", release_lo), format!("{:.17e}", release_hi)];
        writeln!(trace, "{}", fields.join(",")).unwrap();
    }
}
""".replace("ID_HEADER", ",".join(TRACE_FIELDS))
    )


def compile_native(work):
    work = build.fresh(work)
    work.mkdir(parents=True)
    (work / "probe-geometry.rs").write_text(source())
    compiler = shutil.which("rustc")
    build.require(compiler is not None, "Activate the Rust compiler")
    command = [
        compiler,
        "--edition=2021",
        "-O",
        "probe-geometry.rs",
        "-o",
        "native-probe-geometry",
    ]
    with (work / "compiler.log").open("w") as log:
        subprocess.run(
            command, cwd=work, stdout=log, stderr=subprocess.STDOUT, check=True
        )
    return dict(
        rustc=subprocess.check_output([compiler, "--version"], text=True).strip(),
        command=command,
        source_sha256=build.sha(work / "probe-geometry.rs"),
        binary_sha256=build.sha(work / "native-probe-geometry"),
    )


def evaluate(work, packet, prefix="", map_path=build.FIXTURES / "obstacles.csv"):
    work, packet = Path(work), Path(packet)
    build.require(
        prefix in ("", "plot-", "representative-"), "Invalid native output prefix"
    )
    build.require(
        packet.parent.resolve() == work.resolve(),
        "Native packet must be in its work directory",
    )
    command = [
        "./native-probe-geometry",
        str(map_path),
        packet.name,
        prefix + "witnesses.csv",
        prefix + "paths.csv",
    ]
    with (work / (prefix + "trace.csv")).open("x") as output, (
        work / (prefix + "native.log")
    ).open("x") as log:
        subprocess.run(command, cwd=work, stdout=output, stderr=log, check=True)
    files = [
        packet.name,
        *(prefix + name for name in ("trace.csv", "witnesses.csv", "paths.csv")),
    ]
    return dict(
        command=command, sha256={name: build.sha(work / name) for name in files}
    )
