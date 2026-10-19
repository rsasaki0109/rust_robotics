"""Query fixed post-release inputs with the original native preview functions."""

from pathlib import Path
import shutil
import subprocess

import build_mean_guard_preview as build
import native_guard_release_census as census

VARIANTS = ("recorded", "current_recovery_xy", "carried_release_mean_xy")
INPUT_FIELDS = (
    "id",
    "transition_id",
    "phase",
    *census.INPUT_FIELDS[1:],
    "resume_mean_x",
    "resume_mean_y",
    "resume_mean_yaw",
    "resume_speed",
    "resume_applied_steering",
)
POSE_FIELDS = ("x", "y", "yaw", "speed", "target", "steering")
TRACE_FIELDS = (
    "id",
    "transition_id",
    "phase",
    "variant",
    "scenario",
    *POSE_FIELDS,
    *census.TRACE_FIELDS[1:],
)


def source():
    inherited = census.source()
    build.require(inherited.count("fn main() {") == 1, "Native census entry changed")
    prefix = inherited.split("fn main() {", 1)[0]
    constants = "\n".join(
        f"const Q_{field.upper()}: usize = {i};" for i, field in enumerate(INPUT_FIELDS)
    )
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
    let (mut sequence, mut transition, mut previous_phase) = (0usize, 0usize, "".to_owned());
    for line in input {
        let line = line.unwrap();
        let f: Vec<_> = line.split('\t').collect();
        assert_eq!(f.len(), INPUT_COUNT);
        sequence += 1;
        assert_eq!(f[Q_ID].parse::<usize>().unwrap(), sequence);
        let tid = f[Q_TRANSITION_ID].parse::<usize>().unwrap();
        let phase = f[Q_PHASE];
        if phase == "resume" {
            assert_eq!(tid, transition + 1);
            transition = tid;
        } else {
            assert_eq!(phase, "next");
            assert_eq!(tid, transition);
            assert_eq!(previous_phase, "resume");
        }
        previous_phase = phase.to_owned();
        let number = |i: usize| { let value: f64 = f[i].parse().unwrap(); assert!(value.is_finite()); value };
        for column in NUMERIC_COLUMNS { number(column); }
        let (margin, release_margin, sigma, base) = preview_margins(number(Q_POSITION_SIGMA), number(Q_DRIFT_SPEED), number(Q_OBSERVATION_AGE), f[Q_PRECISION_ACTIVE].parse().unwrap(), f[Q_GUARD_BUFFER].parse().unwrap());
        assert_eq!(margin, number(Q_SPEED_MARGIN));
        let obstacles = maps.get(f[Q_SCENARIO]).unwrap();
        let (line, bounds) = geometry(number(Q_MEAN_X), number(Q_MEAN_Y), number(Q_MEAN_YAW), margin, obstacles);
        let actual = VehicleState::new(number(Q_PREVIEW_X), number(Q_PREVIEW_Y), number(Q_PREVIEW_YAW), number(Q_MEASURED_SPEED), WHEELBASE);
        let mut transport = MotionPoseFilter {
            samples: VecDeque::from([(number(Q_RESUME_MEAN_X), number(Q_RESUME_MEAN_Y), number(Q_RESUME_MEAN_YAW))]),
            previous_speed: number(Q_RESUME_SPEED),
        };
        transport.observe(actual, number(Q_RESUME_APPLIED_STEERING), false);
        let carried = transport.samples.front().unwrap();
        for variant in ["recorded", "current_recovery_xy", "carried_release_mean_xy"] {
            if phase == "resume" && variant != "recorded" { continue; }
            let mut pose = actual;
            if variant == "current_recovery_xy" {
                pose.x = number(Q_MEAN_X); pose.y = number(Q_MEAN_Y);
            } else if variant == "carried_release_mean_xy" {
                pose.x = carried.0; pose.y = carried.1;
            }
            assert_eq!((pose.yaw, pose.v, pose.wheelbase, pose.rear_x, pose.rear_y),
                (actual.yaw, actual.v, actual.wheelbase, actual.rear_x, actual.rear_y));
            let target = number(Q_NOMINAL_TARGET);
            let steering = number(Q_NOMINAL_STEERING);
            let (stop, terminal) = stopping_preview(pose, target, steering, base, obstacles, true, true);
            let entry = stop.min(terminal - 0.1);
            let release = aligned_release_preview(stop, terminal);
            let mut values = vec![sequence.to_string(), tid.to_string(), phase.to_owned(), variant.to_owned(), f[Q_SCENARIO].to_owned()];
            values.extend([pose.x, pose.y, pose.yaw, pose.v, target, steering].iter().map(|v| format!("{:.17e}", v)));
            values.extend([format!("{:.17e}", line), format!("{:.9}", line), bounds.to_string(),
                format!("{:.17e}", margin), format!("{:.17e}", base), format!("{:.9}", base),
                format!("{:.17e}", release_margin), format!("{:.9}", release_margin),
                format!("{:.17e}", stop), format!("{:.9}", stop), format!("{:.17e}", terminal), format!("{:.9}", terminal),
                format!("{:.17e}", entry), format!("{:.9}", entry), format!("{:.17e}", release), format!("{:.9}", release), format!("{:.9}", sigma)]);
            writeln!(output, "{}", values.join(",")).unwrap();
        }
    }
    assert!(sequence > 0);
}
"""
    numeric = [
        i
        for i, field in enumerate(INPUT_FIELDS)
        if field
        not in (
            "id",
            "transition_id",
            "phase",
            "scenario",
            "precision_active",
            "guard_buffer",
        )
    ]
    return (
        prefix
        + constants
        + body.replace("INPUT_HEADER", "\t".join(INPUT_FIELDS))
        .replace("TRACE_HEADER", ",".join(TRACE_FIELDS))
        .replace("INPUT_COUNT", str(len(INPUT_FIELDS)))
        .replace("NUMERIC_COLUMNS", str(numeric))
    )


def compile_native(work):
    work = build.fresh(work)
    work.mkdir(parents=True)
    (work / "post-release-transition.rs").write_text(source())
    compiler = shutil.which("rustc")
    build.require(compiler is not None, "Activate Rust")
    command = [
        compiler,
        "--edition=2021",
        "-O",
        "post-release-transition.rs",
        "-o",
        "native-post-release",
    ]
    with (work / "compiler.log").open("x") as log:
        subprocess.run(
            command, cwd=work, stdout=log, stderr=subprocess.STDOUT, check=True
        )
    return dict(
        rustc=subprocess.check_output([compiler, "--version"], text=True).strip(),
        command=command,
        source_sha256=build.sha(work / "post-release-transition.rs"),
        binary_sha256=build.sha(work / "native-post-release"),
    )


def evaluate(work, packet, map_path=build.FIXTURES / "obstacles.csv"):
    work, packet = Path(work), Path(packet)
    build.require(
        packet.parent.resolve() == work.resolve(), "Native packet must be local"
    )
    build.require(
        not any((work / n).exists() for n in ("trace.csv", "native.log")),
        "Native outputs exist",
    )
    command = ["./native-post-release", str(map_path), packet.name]
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
