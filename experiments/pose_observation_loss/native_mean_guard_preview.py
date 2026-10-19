"""Independently replay every baseline estimator and selected guard query in Rust."""

from pathlib import Path
import shutil
import subprocess

import build_mean_guard_preview as build
import native_terminal_preview as previous

EXTRA_FIELDS = (
    "guard_enabled",
    "guard_used",
    "joint_pending",
    "speed_used",
    "base_x",
    "base_y",
    "base_yaw",
    "selected_x",
    "selected_y",
    "selected_yaw",
    "row_x_9",
    "row_y_9",
    "row_yaw_9",
    "stop_9",
    "terminal_9",
    "release_9",
    "release_margin_9",
    "recovery_count",
    "pre_paused",
)
HISTORY_FIELDS = (*previous.HISTORY_FIELDS, *EXTRA_FIELDS)
TRACE_FIELDS = (
    "id",
    "case_id",
    "scenario",
    "step",
    "used",
    "target",
    "steering",
    "preview_margin",
    "release_margin",
    "x",
    "y",
    "yaw",
    "stop",
    "terminal",
    "release",
)
CHECK_FIELDS = ("case_id", "scenario", "decisions", "guard_uses", "geometry_queries")


def source():
    inherited = previous.source()
    build.require(inherited.count("fn emit(") == 1, "Native reader prefix changed")
    prefix = inherited.split("fn emit(", 1)[0]
    constants = "\n".join(
        f"const {name.upper()}: usize = {i};"
        for i, name in enumerate(HISTORY_FIELDS)
        if name in EXTRA_FIELDS
    )
    body = r"""
fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 5);
    assert_eq!(&args[4], "exact-original-functions");
    let mut maps = std::collections::BTreeMap::<String, Vec<Point2D>>::new();
    for line in std::fs::read_to_string(&args[1]).unwrap().lines().skip(1) {
        let f: Vec<_> = line.split(',').collect();
        assert_eq!(f.len(), 3);
        maps.entry(f[0].to_owned()).or_default().push(Point2D::new(f[1].parse().unwrap(), f[2].parse().unwrap()));
    }
    let mut output = std::io::BufWriter::new(std::io::stdout().lock());
    writeln!(output, "TRACE_HEADER").unwrap();
    let mut checks = std::io::BufWriter::new(std::fs::File::create(&args[3]).unwrap());
    writeln!(checks, "CHECK_HEADER").unwrap();
    let mut lines = std::io::BufReader::new(std::fs::File::open(&args[2]).unwrap()).lines();
    assert_eq!(lines.next().unwrap().unwrap(), "HISTORY_HEADER");
    let mut filter = MotionPoseFilter::default();
    let mut fusion = StationaryEpochFusion::default();
    let mut previous_steering = 0.0;
    let mut precision = false;
    let mut current = String::new();
    let mut scenario = String::new();
    let mut seen = std::collections::BTreeSet::new();
    let (mut sequence, mut selected, mut case_steps, mut case_used, mut case_queries) = (0usize, 0usize, 0usize, 0usize, 0usize);
    for line in lines {
        let line = line.unwrap();
        let f: Vec<_> = line.split('\t').collect();
        assert_eq!(f.len(), HISTORY_COUNT);
        sequence += 1;
        assert_eq!(f[SEQUENCE].parse::<usize>().unwrap(), sequence);
        if f[CASE_ID] != current {
            if !current.is_empty() {
                writeln!(checks, "{},{},{},{},{}", current, scenario, case_steps, case_used, case_queries).unwrap();
            }
            current = f[CASE_ID].to_owned();
            assert!(seen.insert(current.clone()), "Repeated history case");
            scenario = f[SCENARIO].to_owned();
            filter = MotionPoseFilter::default();
            fusion = StationaryEpochFusion::default();
            previous_steering = 0.0;
            precision = false;
            case_steps = 0; case_used = 0; case_queries = 0;
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
        assert_eq!((preview.x, preview.y, preview.yaw), (number(BASE_X), number(BASE_Y), number(BASE_YAW)), "Full baseline mismatch at {} sequence {}", current, sequence);
        for (value, column) in [(motion.x, FILTER_X_9), (motion.y, FILTER_Y_9), (preview.x, PREVIEW_X_9), (preview.y, PREVIEW_Y_9), (preview.yaw, PREVIEW_YAW_9)] {
            assert_eq!(format!("{:.9}", value), f[column], "Baseline rounding mismatch at {} sequence {} column {}", current, sequence, column);
        }
        assert_eq!(filter.samples.len(), f[FILTER_COUNT].parse::<usize>().unwrap());
        assert_eq!(fused.is_some(), flag(FUSION_AVAILABLE));
        assert_eq!(fusion.groups.len(), f[FUSION_GROUPS].parse::<usize>().unwrap());
        assert_eq!(flag(JOINT_PENDING), flag(PRE_PAUSED));
        let eligible = flag(PRE_PAUSED) && sensed.v.abs() <= 0.02 && number(OBSERVATION_AGE) <= 1.5 && f[RECOVERY_COUNT].parse::<usize>().unwrap() >= 10;
        assert_eq!(eligible, flag(SPEED_USED));
        let used = flag(GUARD_ENABLED) && flag(JOINT_PENDING) && eligible;
        assert_eq!(used, flag(GUARD_USED));
        if used { preview.x = number(MEAN_X); preview.y = number(MEAN_Y); case_used += 1; }
        assert_eq!((preview.x, preview.y, preview.yaw), (number(SELECTED_X), number(SELECTED_Y), number(SELECTED_YAW)), "Selected pose differs at {}", sequence);
        for (value, column) in [(preview.x, ROW_X_9), (preview.y, ROW_Y_9), (preview.yaw, ROW_YAW_9)] {
            assert_eq!(format!("{:.9}", value), f[column], "Actual pose rounding differs at {}", sequence);
        }
        let (speed_margin, release_margin, _, base_margin) = preview_margins(number(POSITION_SIGMA), number(DRIFT_SPEED), number(OBSERVATION_AGE), precision, flag(GUARD_BUFFER));
        assert_eq!(speed_margin, number(SPEED_MARGIN), "Speed margin mismatch at {}", sequence);
        assert_eq!(format!("{:.9}", release_margin), f[RELEASE_MARGIN_9]);
        assert!(!used || f[ID] != "0", "Every guard use requires an original stopping query");
        if f[ID] != "0" {
            selected += 1; case_queries += 1;
            assert_eq!(f[ID].parse::<usize>().unwrap(), selected);
            let target = number(NOMINAL_TARGET);
            let steering = number(NOMINAL_STEERING);
            let (stop, terminal) = stopping_preview(preview, target, steering, base_margin, maps.get(f[SCENARIO]).unwrap(), true, true);
            let release = aligned_release_preview(stop, terminal);
            for (value, column) in [(stop, STOP_9), (terminal, TERMINAL_9), (release, RELEASE_9)] {
                assert_eq!(format!("{:.9}", value), f[column], "Geometry mismatch at {} sequence {} column {}", current, sequence, column);
            }
            let mut values = vec![f[ID].to_owned(), current.clone(), scenario.clone(), case_steps.to_string(), used.to_string()];
            values.extend([target, steering, base_margin, release_margin, preview.x, preview.y, preview.yaw, stop, terminal, release].iter().map(|v| format!("{:.17e}", v)));
            writeln!(output, "{}", values.join(",")).unwrap();
        }
        previous_steering = number(APPLIED_STEERING);
    }
    assert!(!current.is_empty());
    writeln!(checks, "{},{},{},{},{}", current, scenario, case_steps, case_used, case_queries).unwrap();
}
"""
    return (
        prefix
        + constants
        + "\n"
        + body.replace("TRACE_HEADER", ",".join(TRACE_FIELDS))
        .replace("CHECK_HEADER", ",".join(CHECK_FIELDS))
        .replace("HISTORY_HEADER", "\t".join(HISTORY_FIELDS))
        .replace("HISTORY_COUNT", str(len(HISTORY_FIELDS)))
    )


def compile_native(work):
    work = build.fresh(work)
    work.mkdir(parents=True)
    (work / "guard-preview.rs").write_text(source())
    compiler = shutil.which("rustc")
    build.require(compiler is not None, "Activate Rust")
    command = [
        compiler,
        "--edition=2021",
        "-O",
        "guard-preview.rs",
        "-o",
        "native-guard-preview",
    ]
    with (work / "compiler.log").open("x") as log:
        subprocess.run(
            command, cwd=work, stdout=log, stderr=subprocess.STDOUT, check=True
        )
    return dict(
        rustc=subprocess.check_output([compiler, "--version"], text=True).strip(),
        command=command,
        source_sha256=build.sha(work / "guard-preview.rs"),
        binary_sha256=build.sha(work / "native-guard-preview"),
    )


def evaluate(work, packet, map_path=build.FIXTURES / "obstacles.csv"):
    work, packet = Path(work), Path(packet)
    build.require(
        packet.parent.resolve() == work.resolve(), "Native packet must be local"
    )
    names = ("trace.csv", "history-checks.csv", "native.log")
    build.require(
        not any((work / n).exists() for n in names), "Native outputs already exist"
    )
    command = [
        "./native-guard-preview",
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
