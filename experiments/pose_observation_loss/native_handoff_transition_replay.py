"""Replay anchored position transport with the original native preview functions."""

from pathlib import Path
import shutil
import subprocess

import build_release_mean_handoff as build
import native_guard_release_census as census

VARIANTS = ("recorded", "carried_diagnostic_xy", "gated_carried_xy")
INPUT_FIELDS = (
    "id",
    "transition_id",
    "lag",
    *census.INPUT_FIELDS[1:],
    "applied_steering",
    "joint_pending",
    "correction",
    "received",
    "precision_activation",
    "count",
)
POSE_FIELDS = ("x", "y", "yaw", "speed", "target", "steering")
TRACE_FIELDS = (
    "id",
    "transition_id",
    "lag",
    "variant",
    "scenario",
    "valid_now",
    "valid_prefix",
    "carry_x",
    "carry_y",
    "carry_yaw",
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
    let (mut sequence, mut transition, mut previous_lag) = (0usize, 0usize, 0usize);
    let mut scenario = String::new();
    let mut carry = (0.0, 0.0, 0.0);
    let (mut previous_speed, mut previous_steering) = (0.0, 0.0);
    let mut valid_prefix = true;
    for line in input {
        let line = line.unwrap();
        let f: Vec<_> = line.split('\t').collect();
        assert_eq!(f.len(), INPUT_COUNT);
        sequence += 1;
        assert_eq!(f[Q_ID].parse::<usize>().unwrap(), sequence);
        let tid = f[Q_TRANSITION_ID].parse::<usize>().unwrap();
        let lag = f[Q_LAG].parse::<usize>().unwrap();
        assert!(lag <= 50);
        let number = |i: usize| { let value: f64 = f[i].parse().unwrap(); assert!(value.is_finite()); value };
        for column in NUMERIC_COLUMNS { number(column); }
        let flag = |i: usize| f[i].parse::<bool>().unwrap();
        let count = f[Q_COUNT].parse::<usize>().unwrap();
        let actual = VehicleState::new(number(Q_PREVIEW_X), number(Q_PREVIEW_Y), number(Q_PREVIEW_YAW), number(Q_MEASURED_SPEED), WHEELBASE);
        let valid_now = !flag(Q_JOINT_PENDING) && !flag(Q_CORRECTION) && !flag(Q_RECEIVED)
            && !flag(Q_PRECISION_ACTIVATION) && actual.v.abs() <= 0.02
            && number(Q_OBSERVATION_AGE) <= 1.5 && count >= 10;
        if lag == 0 {
            assert_eq!(tid, transition + 1);
            transition = tid;
            scenario = f[Q_SCENARIO].to_owned();
            carry = (number(Q_MEAN_X), number(Q_MEAN_Y), number(Q_MEAN_YAW));
            valid_prefix = true;
        } else {
            assert_eq!(tid, transition);
            assert_eq!(lag, previous_lag + 1);
            assert_eq!(scenario, f[Q_SCENARIO]);
            let mut transport = MotionPoseFilter {
                samples: VecDeque::from([carry]), previous_speed,
            };
            transport.observe(actual, previous_steering, false);
            carry = *transport.samples.front().unwrap();
            valid_prefix &= valid_now;
        }
        previous_lag = lag;
        previous_speed = actual.v;
        previous_steering = number(Q_APPLIED_STEERING);
        let (margin, release_margin, sigma, base) = preview_margins(number(Q_POSITION_SIGMA), number(Q_DRIFT_SPEED), number(Q_OBSERVATION_AGE), flag(Q_PRECISION_ACTIVE), flag(Q_GUARD_BUFFER));
        assert_eq!(margin, number(Q_SPEED_MARGIN));
        let obstacles = maps.get(f[Q_SCENARIO]).unwrap();
        let (line, bounds) = geometry(number(Q_MEAN_X), number(Q_MEAN_Y), number(Q_MEAN_YAW), margin, obstacles);
        for variant in ["recorded", "carried_diagnostic_xy", "gated_carried_xy"] {
            if lag == 0 && variant != "recorded" { continue; }
            let mut pose = actual;
            if variant == "carried_diagnostic_xy" || (variant == "gated_carried_xy" && valid_prefix) {
                pose.x = carry.0; pose.y = carry.1;
            }
            assert_eq!((pose.yaw, pose.v, pose.wheelbase, pose.rear_x, pose.rear_y),
                (actual.yaw, actual.v, actual.wheelbase, actual.rear_x, actual.rear_y));
            let target = number(Q_NOMINAL_TARGET);
            let steering = number(Q_NOMINAL_STEERING);
            let (stop, terminal) = stopping_preview(pose, target, steering, base, obstacles, true, true);
            let entry = stop.min(terminal - 0.1);
            let release = aligned_release_preview(stop, terminal);
            let mut values = vec![sequence.to_string(), tid.to_string(), lag.to_string(), variant.to_owned(), scenario.clone(), valid_now.to_string(), valid_prefix.to_string()];
            values.extend([carry.0, carry.1, carry.2, pose.x, pose.y, pose.yaw, pose.v, target, steering].iter().map(|v| format!("{:.17e}", v)));
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
    flags = (
        "precision_active",
        "guard_buffer",
        "joint_pending",
        "correction",
        "received",
        "precision_activation",
    )
    numeric = [
        i for i, field in enumerate(INPUT_FIELDS) if field not in (*flags, "scenario")
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
    (work / "handoff-transition.rs").write_text(source())
    compiler = shutil.which("rustc")
    build.require(compiler is not None, "Activate Rust")
    command = [
        compiler,
        "--edition=2021",
        "-O",
        "handoff-transition.rs",
        "-o",
        "native-handoff-transition",
    ]
    with (work / "compiler.log").open("x") as log:
        subprocess.run(
            command, cwd=work, stdout=log, stderr=subprocess.STDOUT, check=True
        )
    return dict(
        rustc=subprocess.check_output([compiler, "--version"], text=True).strip(),
        command=command,
        source_sha256=build.sha(work / "handoff-transition.rs"),
        binary_sha256=build.sha(work / "native-handoff-transition"),
    )


def evaluate(work, packet, map_path):
    work, packet = Path(work), Path(packet)
    build.require(
        packet.parent.resolve() == work.resolve(), "Native packet must be local"
    )
    build.require(
        not any((work / n).exists() for n in ("trace.csv", "native.log")),
        "Native outputs exist",
    )
    command = ["./native-handoff-transition", str(map_path), packet.name]
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
