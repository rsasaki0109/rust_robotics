"""Replay original recovery, stopping and terminal geometry at every decision."""

from pathlib import Path
import shutil
import subprocess

import build_mean_guard_preview as build
import native_terminal_preview as previous

INPUT_FIELDS = (
    "id",
    "scenario",
    "mean_x",
    "mean_y",
    "mean_yaw",
    "speed_margin",
    "observation_age",
    "precision_active",
    "position_sigma",
    "drift_speed",
    "guard_buffer",
    "preview_x",
    "preview_y",
    "preview_yaw",
    "measured_speed",
    "nominal_target",
    "nominal_steering",
)
TRACE_FIELDS = (
    "id",
    "recovery_clearance",
    "recovery_clearance_9",
    "recovery_bounds",
    "recovery_margin",
    "preview_margin",
    "preview_margin_9",
    "release_margin",
    "release_margin_9",
    "stop",
    "stop_9",
    "terminal",
    "terminal_9",
    "entry",
    "entry_9",
    "release",
    "release_9",
    "effective_position_sigma_9",
)


def source():
    inherited = previous.source()
    build.require(inherited.count("fn emit(") == 1, "Original native prefix changed")
    prefix = inherited.split("fn emit(", 1)[0]
    body = r"""
fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 3);
    let mut maps = std::collections::BTreeMap::<String, Vec<Point2D>>::new();
    for line in std::fs::read_to_string(&args[1]).unwrap().lines().skip(1) {
        let f: Vec<_> = line.split(',').collect();
        assert_eq!(f.len(), 3);
        maps.entry(f[0].to_owned()).or_default().push(Point2D::new(f[1].parse().unwrap(), f[2].parse().unwrap()));
    }
    let mut input = std::io::BufReader::new(std::fs::File::open(&args[2]).unwrap()).lines();
    assert_eq!(input.next().unwrap().unwrap(), "INPUT_HEADER");
    let mut output = std::io::BufWriter::new(std::io::stdout().lock());
    writeln!(output, "TRACE_HEADER").unwrap();
    let mut id = 0usize;
    for line in input {
        let line = line.unwrap();
        let f: Vec<_> = line.split('\t').collect();
        assert_eq!(f.len(), INPUT_COUNT);
        id += 1;
        assert_eq!(f[0].parse::<usize>().unwrap(), id);
        let number = |i: usize| { let value: f64 = f[i].parse().unwrap(); assert!(value.is_finite()); value };
        let (margin, release_margin, sigma, base) = preview_margins(number(8), number(9), number(6), f[7].parse().unwrap(), f[10].parse().unwrap());
        assert_eq!(margin, number(5), "Recovery margin differs at {}", id);
        let obstacles = maps.get(f[1]).unwrap();
        let (line, bounds) = geometry(number(2), number(3), number(4), margin, obstacles);
        let pose = VehicleState::new(number(11), number(12), number(13), number(14), WHEELBASE);
        let (stop, terminal) = stopping_preview(pose, number(15), number(16), base, obstacles, true, true);
        let entry = stop.min(terminal - 0.1);
        let release = aligned_release_preview(stop, terminal);
        let values = vec![id.to_string(), format!("{:.17e}", line), format!("{:.9}", line), bounds.to_string(),
            format!("{:.17e}", margin), format!("{:.17e}", base), format!("{:.9}", base),
            format!("{:.17e}", release_margin), format!("{:.9}", release_margin),
            format!("{:.17e}", stop), format!("{:.9}", stop), format!("{:.17e}", terminal), format!("{:.9}", terminal),
            format!("{:.17e}", entry), format!("{:.9}", entry), format!("{:.17e}", release), format!("{:.9}", release), format!("{:.9}", sigma)];
        writeln!(output, "{}", values.join(",")).unwrap();
    }
    assert!(id > 0);
}
"""
    return prefix + body.replace("INPUT_HEADER", "\t".join(INPUT_FIELDS)).replace(
        "TRACE_HEADER", ",".join(TRACE_FIELDS)
    ).replace("INPUT_COUNT", str(len(INPUT_FIELDS)))


def compile_native(work):
    work = build.fresh(work)
    work.mkdir(parents=True)
    (work / "release-census.rs").write_text(source())
    compiler = shutil.which("rustc")
    build.require(compiler is not None, "Activate Rust")
    command = [
        compiler,
        "--edition=2021",
        "-O",
        "release-census.rs",
        "-o",
        "native-release-census",
    ]
    with (work / "compiler.log").open("x") as log:
        subprocess.run(
            command, cwd=work, stdout=log, stderr=subprocess.STDOUT, check=True
        )
    return dict(
        rustc=subprocess.check_output([compiler, "--version"], text=True).strip(),
        command=command,
        source_sha256=build.sha(work / "release-census.rs"),
        binary_sha256=build.sha(work / "native-release-census"),
    )


def evaluate(work, packet, map_path=build.FIXTURES / "obstacles.csv"):
    work, packet = Path(work), Path(packet)
    build.require(
        packet.parent.resolve() == work.resolve(), "Native packet must be local"
    )
    build.require(
        not any((work / name).exists() for name in ("trace.csv", "native.log")),
        "Native outputs already exist",
    )
    command = ["./native-release-census", str(map_path), packet.name]
    with (work / "trace.csv").open("x") as output, (work / "native.log").open(
        "x"
    ) as log:
        subprocess.run(command, cwd=work, stdout=output, stderr=log, check=True)
    return dict(
        command=command,
        sha256={
            name: build.sha(work / name)
            for name in (packet.name, "trace.csv", "native.log")
        },
    )
