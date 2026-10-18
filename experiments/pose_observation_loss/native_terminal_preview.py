"""Replay original sensor-only preview histories and stopping queries in Rust."""

from pathlib import Path
import shutil
import subprocess

import build_mean_speed_probe as build
import native_probe_geometry as previous
import verify_reset_mean_probe as original

VARIANTS = ("recorded", "mean_position", "mean_pose", "zero_target")
HISTORY_FIELDS = (
    "sequence",
    "id",
    "case_id",
    "scenario",
    "step",
    "time_s",
    "raw_x",
    "raw_y",
    "raw_yaw",
    "measured_speed",
    "observation_age",
    "correction",
    "received",
    "precision_activation",
    "applied_steering",
    "nominal_target",
    "nominal_steering",
    "speed_margin",
    "mean_x",
    "mean_y",
    "mean_yaw",
    "precision_active",
    "filter_count",
    "fusion_available",
    "fusion_groups",
    "filter_x_9",
    "filter_y_9",
    "preview_x_9",
    "preview_y_9",
    "preview_yaw_9",
    "position_sigma",
    "drift_speed",
    "guard_buffer",
)
DIRECT_FIELDS = (
    "id",
    "variant",
    "scenario",
    "x",
    "y",
    "yaw",
    "speed",
    "target",
    "steering",
    "preview_margin",
    "release_margin",
)
TRACE_FIELDS = (
    *DIRECT_FIELDS,
    "stop_clearance",
    "terminal_clearance",
    "release_clearance",
    "end_x",
    "end_y",
    "end_yaw",
    "end_speed",
    "motion_steps",
    "motion_length",
    "displacement",
    "turn",
    "stop_9",
    "terminal_9",
    "release_9",
    "release_margin_9",
)
CHECK_FIELDS = ("case_id", "scenario", "decisions", "selected_decisions")


def source():
    base = build.BASE.read_text()
    inherited = previous.source()
    build.require(inherited.count("fn main() {") == 1, "Previous reader entry changed")
    prefix = inherited.split("fn main() {", 1)[0]
    margins = original.source_block(prefix, "fn margins(")
    margins = build.once(margins, "fn margins(", "fn preview_margins(")
    margins = build.once(margins, "-> (f64, f64, f64)", "-> (f64, f64, f64, f64)")
    margins = build.once(
        margins,
        "(recovery_margin, guard_release_margin, effective_position_sigma)",
        "(recovery_margin, guard_release_margin, effective_position_sigma, base_preview_margin)",
    )
    stop = original.source_block(base, "fn stopping_preview(")
    build.require(
        stop.count("    let mut clearance = points\n") == 1, "Stopping prefix changed"
    )
    motion = stop.split("    let mut clearance = points\n", 1)[0]
    motion = build.once(motion, "fn stopping_preview(", "fn stopping_motion(")
    motion = build.once(
        motion, ") -> (f64, f64) {", ") -> (VehicleState, Vec<Point2D>, f64) {"
    )
    motion += "    (predicted, points, length)\n}\n"
    blocks = [
        prefix,
        "use std::collections::VecDeque;",
        next(line for line in base.splitlines() if line.startswith("const DT:")),
        "impl VehicleState {\n"
        + original.source_block(original.VEHICLE.read_text(), "pub fn update(")
        + "\n}",
        original.source_block(base, "fn mean_pose("),
        "#[derive(Default)]\n" + original.source_block(base, "struct MotionPoseFilter"),
        original.source_block(base, "impl MotionPoseFilter"),
        original.source_block(base, "struct ObservationEvent"),
        original.source_block(base, "struct StationaryEpoch {"),
        "#[derive(Default)]\n"
        + original.source_block(base, "struct StationaryEpochFusion"),
        original.source_block(base, "impl StationaryEpochFusion"),
        original.source_block(base, "fn terminal_arc_clearance("),
        stop,
        motion,
        margins,
        "\n".join(
            f"const {name.upper()}: usize = {i};"
            for i, name in enumerate(HISTORY_FIELDS)
        ),
    ]
    body = r"""
fn emit(output: &mut impl Write, id: &str, variant: &str, scenario: &str,
        pose: VehicleState, target: f64, steering: f64, margin: f64,
        release_margin: f64, obstacles: &[Point2D]) {
    assert!([pose.x, pose.y, pose.yaw, pose.v, target, steering, margin, release_margin].iter().all(|v| v.is_finite()));
    let (stop, terminal) = stopping_preview(pose, target, steering, margin, obstacles, true, true);
    let (end, points, length) = stopping_motion(pose, target, steering, margin, obstacles, true, true);
    assert_eq!(terminal, terminal_arc_clearance(end, steering, obstacles), "Stopping endpoint changed at {} {}", id, variant);
    let release = aligned_release_preview(stop, terminal);
    let displacement = Point2D::new(end.x, end.y).distance(&Point2D::new(pose.x, pose.y));
    let delta = end.yaw - pose.yaw;
    let mut fields = vec![id.to_owned(), variant.to_owned(), scenario.to_owned()];
    fields.extend([pose.x, pose.y, pose.yaw, pose.v, target, steering, margin, release_margin,
        stop, terminal, release, end.x, end.y, end.yaw, end.v].iter().map(|v| format!("{:.17e}", v)));
    fields.push((points.len() - 1).to_string());
    fields.extend([length, displacement, delta.sin().atan2(delta.cos())].iter().map(|v| format!("{:.17e}", v)));
    fields.extend([stop, terminal, release, release_margin].iter().map(|v| format!("{:.9}", v)));
    writeln!(output, "{}", fields.join(",")).unwrap();
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 6);
    assert!(args[1] == "history" || args[1] == "direct");
    let mut maps = std::collections::BTreeMap::<String, Vec<Point2D>>::new();
    for line in std::fs::read_to_string(&args[2]).unwrap().lines().skip(1) {
        let f: Vec<_> = line.split(',').collect();
        maps.entry(f[0].to_owned()).or_default().push(Point2D::new(f[1].parse().unwrap(), f[2].parse().unwrap()));
    }
    let mut output = std::io::BufWriter::new(std::io::stdout().lock());
    writeln!(output, "TRACE_HEADER").unwrap();
    let mut checks = std::io::BufWriter::new(std::fs::File::create(&args[4]).unwrap());
    writeln!(checks, "case_id,scenario,decisions,selected_decisions").unwrap();
    assert_eq!(&args[5], "exact-original-functions");
    let mut lines = std::io::BufReader::new(std::fs::File::open(&args[3]).unwrap()).lines();
    let header = lines.next().unwrap().unwrap();
    if args[1] == "direct" {
        assert_eq!(header, "DIRECT_HEADER");
        for line in lines {
            let line = line.unwrap();
            let f: Vec<_> = line.split('\t').collect();
            assert_eq!(f.len(), 11);
            assert!(["recorded", "mean_position", "mean_pose", "zero_target"].contains(&f[1]));
            let v: Vec<f64> = f[3..].iter().map(|v| v.parse().unwrap()).collect();
            emit(&mut output, f[0], f[1], f[2], VehicleState::new(v[0], v[1], v[2], v[3], WHEELBASE), v[4], v[5], v[6], v[7], maps.get(f[2]).unwrap());
        }
        return;
    }
    assert_eq!(header, "HISTORY_HEADER");
    let mut filter = MotionPoseFilter::default();
    let mut fusion = StationaryEpochFusion::default();
    let mut previous_steering = 0.0;
    let mut precision = false;
    let mut current = String::new();
    let mut scenario = String::new();
    let mut seen = std::collections::BTreeSet::new();
    let (mut sequence, mut selected, mut case_steps, mut case_selected) = (0usize, 0usize, 0usize, 0usize);
    for line in lines {
        let line = line.unwrap();
        let f: Vec<_> = line.split('\t').collect();
        assert_eq!(f.len(), HISTORY_COUNT);
        sequence += 1;
        assert_eq!(f[SEQUENCE].parse::<usize>().unwrap(), sequence);
        if f[CASE_ID] != current {
            if !current.is_empty() {
                writeln!(checks, "{},{},{},{}", current, scenario, case_steps, case_selected).unwrap();
            }
            current = f[CASE_ID].to_owned();
            assert!(seen.insert(current.clone()), "Repeated history case");
            scenario = f[SCENARIO].to_owned();
            filter = MotionPoseFilter::default();
            fusion = StationaryEpochFusion::default();
            previous_steering = 0.0;
            precision = false;
            case_steps = 0;
            case_selected = 0;
        }
        assert_eq!(scenario, f[SCENARIO]);
        case_steps += 1;
        assert_eq!(f[STEP].parse::<usize>().unwrap(), case_steps);
        let number = |i: usize| { let v: f64 = f[i].parse().unwrap(); assert!(v.is_finite()); v };
        let flag = |i: usize| f[i].parse::<bool>().unwrap();
        let time = case_steps as f64 * DT;
        assert_eq!(time, number(TIME_S));
        let activation = flag(PRECISION_ACTIVATION);
        precision |= activation;
        assert_eq!(precision, flag(PRECISION_ACTIVE));
        let sensed = VehicleState::new(number(RAW_X), number(RAW_Y), number(RAW_YAW), number(MEASURED_SPEED), WHEELBASE);
        let motion = filter.observe(sensed, previous_steering, flag(CORRECTION) || activation);
        let fused = fusion.observe(sensed, previous_steering, ObservationEvent { time, correction: flag(CORRECTION), received: flag(RECEIVED), precision_activation: activation }, number(OBSERVATION_AGE));
        let mut preview = motion;
        if let Some((x, y)) = fused { preview.x = x; preview.y = y; }
        for (value, column) in [(motion.x, FILTER_X_9), (motion.y, FILTER_Y_9), (preview.x, PREVIEW_X_9), (preview.y, PREVIEW_Y_9), (preview.yaw, PREVIEW_YAW_9)] {
            assert_eq!(format!("{:.9}", value), f[column], "Pose mismatch at {} sequence {} column {}", current, sequence, column);
        }
        assert_eq!(filter.samples.len(), f[FILTER_COUNT].parse::<usize>().unwrap());
        assert_eq!(fused.is_some(), flag(FUSION_AVAILABLE));
        assert_eq!(fusion.groups.len(), f[FUSION_GROUPS].parse::<usize>().unwrap());
        let (speed_margin, release_margin, _, base_margin) = preview_margins(number(POSITION_SIGMA), number(DRIFT_SPEED), number(OBSERVATION_AGE), precision, flag(GUARD_BUFFER));
        assert_eq!(speed_margin, number(SPEED_MARGIN), "Speed margin mismatch at {}", sequence);
        if f[ID] != "0" {
            selected += 1;
            case_selected += 1;
            assert_eq!(f[ID].parse::<usize>().unwrap(), selected);
            let mean_position = VehicleState::new(number(MEAN_X), number(MEAN_Y), preview.yaw, sensed.v, WHEELBASE);
            let mean_pose = VehicleState::new(number(MEAN_X), number(MEAN_Y), number(MEAN_YAW), sensed.v, WHEELBASE);
            let target = number(NOMINAL_TARGET);
            assert!(target > 0.0);
            for (name, pose, proposed) in [("recorded", preview, target), ("mean_position", mean_position, target), ("mean_pose", mean_pose, target), ("zero_target", preview, 0.0)] {
                emit(&mut output, f[ID], name, f[SCENARIO], pose, proposed, number(NOMINAL_STEERING), base_margin, release_margin, maps.get(f[SCENARIO]).unwrap());
            }
        }
        previous_steering = number(APPLIED_STEERING);
    }
    assert!(!current.is_empty());
    writeln!(checks, "{},{},{},{}", current, scenario, case_steps, case_selected).unwrap();
}
"""
    return (
        "\n".join(blocks)
        + "\n"
        + body.replace("TRACE_HEADER", ",".join(TRACE_FIELDS))
        .replace("DIRECT_HEADER", "\t".join(DIRECT_FIELDS))
        .replace("HISTORY_HEADER", "\t".join(HISTORY_FIELDS))
        .replace("HISTORY_COUNT", str(len(HISTORY_FIELDS)))
    )


def compile_native(work):
    work = build.fresh(work)
    work.mkdir(parents=True)
    (work / "terminal-preview.rs").write_text(source())
    compiler = shutil.which("rustc")
    build.require(compiler is not None, "Activate the Rust compiler")
    command = [
        compiler,
        "--edition=2021",
        "-O",
        "terminal-preview.rs",
        "-o",
        "native-terminal-preview",
    ]
    with (work / "compiler.log").open("x") as log:
        subprocess.run(
            command, cwd=work, stdout=log, stderr=subprocess.STDOUT, check=True
        )
    return dict(
        rustc=subprocess.check_output([compiler, "--version"], text=True).strip(),
        command=command,
        source_sha256=build.sha(work / "terminal-preview.rs"),
        binary_sha256=build.sha(work / "native-terminal-preview"),
    )


def evaluate(
    work, packet, mode="history", prefix="", map_path=build.FIXTURES / "obstacles.csv"
):
    work, packet = Path(work), Path(packet)
    build.require(
        mode in ("history", "direct") and prefix in ("", "compact-"),
        "Invalid native replay mode/prefix",
    )
    build.require(
        packet.parent.resolve() == work.resolve(),
        "Native packet must be local to its work directory",
    )
    names = [prefix + "trace.csv", prefix + "history-checks.csv", prefix + "native.log"]
    build.require(
        not any((work / name).exists() for name in names),
        "Native outputs already exist",
    )
    command = [
        "./native-terminal-preview",
        mode,
        str(map_path),
        packet.name,
        names[1],
        "exact-original-functions",
    ]
    with (work / names[0]).open("x") as output, (work / names[2]).open("x") as log:
        subprocess.run(command, cwd=work, stdout=output, stderr=log, check=True)
    return dict(
        command=command,
        sha256={name: build.sha(work / name) for name in (packet.name, *names[:2])},
    )
