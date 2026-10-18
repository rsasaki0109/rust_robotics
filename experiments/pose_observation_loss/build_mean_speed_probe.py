#!/usr/bin/env python3
"""Build a source-pinned, opt-in mean-position speed-probe experiment."""

import argparse
import difflib
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
STUDY = ROOT / "results/mean-speed-probe-live-pilot"
BASE = REPO / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
FIXTURES = ROOT / "results/planning-clearance/fixtures/fine-padding"
SIDECAR = (
    "scenario",
    "planner",
    "step",
    "time_s",
    "enabled",
    "used",
    "paused",
    "observation_age",
    "measured_speed",
    "correction",
    "received",
    "precision_activation",
    "count",
    "raw_x",
    "raw_y",
    "raw_yaw",
    "mean_x",
    "mean_y",
    "mean_yaw",
    "probe_x",
    "probe_y",
    "probe_yaw",
    "steering",
    "margin",
    "free_distance",
    "cap",
    "uncapped_target",
    "nominal_target",
    "true_x",
    "true_y",
    "true_yaw",
    "true_v",
    "applied_target",
    "applied_steering",
    "applied_acceleration",
)

SELECTOR = r"""
// The archived probe reads only position and yaw. All other policy inputs stay raw.
fn mean_speed_probe_pose(
    sensed: VehicleState,
    enabled: bool,
    paused: bool,
    age: f64,
    count: usize,
    mean_x: f64,
    mean_y: f64,
) -> (VehicleState, bool) {
    let used = enabled && paused && sensed.v.abs() <= 0.02 && age <= 1.5 && count >= 10;
    if used {
        (VehicleState::new(mean_x, mean_y, sensed.yaw, sensed.v, sensed.wheelbase), true)
    } else {
        (sensed, false)
    }
}

#[cfg(test)]
mod mean_speed_probe_tests {
    use super::*;
    fn state(speed: f64) -> VehicleState { VehicleState::new(10.0, 10.0, 0.7, speed, 0.4) }
    #[test]
    fn disabled_is_exact_raw_fallback() {
        let (p, used) = mean_speed_probe_pose(state(0.0), false, true, 0.0, 25, 20.0, 21.0);
        assert!(!used); assert_eq!((p.x, p.y, p.yaw, p.v), (10.0, 10.0, 0.7, 0.0));
    }
    #[test]
    fn thresholds_are_inclusive_and_heading_speed_unchanged() {
        for speed in [0.02, -0.02] {
            let (p, used) = mean_speed_probe_pose(state(speed), true, true, 1.5, 10, 20.0, 21.0);
            assert!(used); assert_eq!((p.x, p.y, p.yaw, p.v), (20.0, 21.0, 0.7, speed));
        }
    }
    #[test]
    fn insufficient_samples_and_unpaused_keep_raw_pose() {
        for (paused, count) in [(true, 9), (false, 25)] {
            let (p, used) = mean_speed_probe_pose(state(0.0), true, paused, 0.0, count, 20.0, 21.0);
            assert!(!used); assert_eq!((p.x, p.y), (10.0, 10.0));
        }
    }
    #[test]
    fn first_float_above_speed_and_age_thresholds_falls_back() {
        let speed = f64::from_bits(0.02_f64.to_bits() + 1);
        let age = f64::from_bits(1.5_f64.to_bits() + 1);
        assert!(!mean_speed_probe_pose(state(speed), true, true, 0.0, 25, 20.0, 21.0).1);
        assert!(!mean_speed_probe_pose(state(-speed), true, true, 0.0, 25, 20.0, 21.0).1);
        assert!(!mean_speed_probe_pose(state(0.0), true, true, age, 25, 20.0, 21.0).1);
    }
    #[test]
    fn selected_position_reaches_the_original_full_probe() {
        let raw = VehicleState::new(10.0, 10.0, 0.0, 0.0, 0.4);
        let obs = [Point2D::new(10.4, 10.0)];
        let (p, used) = mean_speed_probe_pose(raw, true, true, 0.0, 10, 12.0, 10.0);
        assert!(used);
        assert_eq!(speed_free_distance(raw, 0.0, 0.5, &obs), 0.0);
        assert_eq!(speed_free_distance(p, 0.0, 0.5, &obs), 2.0);
    }
}
"""


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def preflight():
    pins = json.loads((STUDY / "freeze-pins.json").read_text())
    for name, digest in pins["input_sha256"].items():
        require(sha(REPO / name) == digest, f"Changed frozen input: {name}")
    audit = json.loads((STUDY / "seed-audit.json").read_text())
    for name, digest in audit["case_catalogue_sha256"].items():
        require(sha(REPO / name) == digest, f"Changed seed catalogue: {name}")
    require(
        pins["seed"] == audit["chosen_seed"]
        and pins["seed"] not in audit["observed_seeds"],
        "Seed audit differs",
    )
    return pins


def once(source, old, new):
    require(source.count(old) == 1, f"Expected one source anchor: {old[:90]}")
    return source.replace(old, new, 1)


def generate(source):
    source = once(
        source,
        "if !(15..=29).contains(&args.len())",
        "if !(15..=30).contains(&args.len())",
    )
    source = once(
        source,
        "; see experiments/pose_observation_loss/run.py",
        "; optional final MEAN_SPEED_PROBE boolean; see experiments/pose_observation_loss/run_mean_speed_probe.py",
    )
    anchor = "    assert!(!terminal_steering_preview || stationary_epoch_fusion);"
    source = once(
        source,
        anchor,
        anchor + """
    let mean_speed_probe = args.get(29).map(|s| s.parse::<bool>()).transpose()?.unwrap_or(false);
    assert!(!mean_speed_probe || (joint_recovery && averaged_recovery));""",
    )
    anchor = '    let mut tracks = BufWriter::new(File::create(output.join("trajectories.csv"))?);'
    source = once(
        source,
        anchor,
        anchor
        + '\n    let mut probe_log = BufWriter::new(File::create(output.join("speed-probes.csv"))?);\n    writeln!(probe_log, "'
        + ",".join(SIDECAR)
        + '")?;',
    )
    anchor = "            let speed_free_distance = if anticipatory_speed {\n                speed_free_distance(sensed, steering, speed_margin, obs)"
    source = once(
        source,
        anchor,
        """            let speed_probe_paused = safety_paused;
            let (speed_probe_pose, mean_speed_probe_used) = mean_speed_probe_pose(
                sensed, mean_speed_probe, safety_paused, observation_age,
                recovery_sample_count, recovery_x, recovery_y,
            );
            let speed_free_distance = if anticipatory_speed {
                speed_free_distance(speed_probe_pose, steering, speed_margin, obs)""",
    )
    anchor = '            writeln!(tracks, "{scenario},{planner},{time:.6},'
    source = once(
        source,
        anchor,
        r"""            writeln!(probe_log, "{scenario},{planner},{step},{observation_time:.17e},{mean_speed_probe},{mean_speed_probe_used},{speed_probe_paused},{observation_age:.17e},{:.17e},{correction},{correction_received},{precision_activation},{recovery_sample_count},{:.17e},{:.17e},{:.17e},{recovery_x:.17e},{recovery_y:.17e},{recovery_yaw:.17e},{:.17e},{:.17e},{:.17e},{nominal_steering:.17e},{speed_margin:.17e},{speed_free_distance:.17e},{speed_cap:.17e},{uncapped_target_speed:.17e},{nominal_target_speed:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{target_speed:.17e},{steering:.17e},{acceleration:.17e}", sensed.v, sensed.x, sensed.y, sensed.yaw, speed_probe_pose.x, speed_probe_pose.y, speed_probe_pose.yaw, state.x, state.y, state.yaw, state.v)?;
"""
        + anchor,
    )
    return source + SELECTOR


def fresh(path):
    path = Path(path).expanduser().resolve()
    require(
        not path.exists() and not path.is_relative_to(REPO),
        "Use a fresh output directory outside the repository",
    )
    return path


def build(work, rustc=None):
    pins = preflight()
    work = fresh(work)
    work.mkdir(parents=True)
    original = BASE.read_text()
    generated = generate(original)
    source_path = work / "mean-speed-probe.rs"
    source_path.write_text(generated)
    (work / "candidate.diff").write_text(
        "".join(
            difflib.unified_diff(
                original.splitlines(keepends=True),
                generated.splitlines(keepends=True),
                fromfile="pinned-pose-observation-loss.rs",
                tofile="mean-speed-probe.rs",
            )
        )
    )
    cargo_command = [
        "cargo",
        "build",
        "--locked",
        "--release",
        "-p",
        "rust_robotics_control",
        "--no-default-features",
        "--example",
        "pose_observation_loss",
        "--message-format=json",
    ]
    with (work / "cargo-artifacts.jsonl").open("w") as out, (work / "cargo.log").open(
        "w"
    ) as err:
        subprocess.run(cargo_command, cwd=REPO, stdout=out, stderr=err, check=True)
    libraries = {
        name: set() for name in ("rust_robotics_control", "rust_robotics_core")
    }
    originals = set()
    for line in (work / "cargo-artifacts.jsonl").read_text().splitlines():
        artifact = json.loads(line)
        if artifact.get("reason") != "compiler-artifact":
            continue
        name = artifact["target"]["name"]
        if name in libraries:
            libraries[name].update(
                Path(p) for p in artifact["filenames"] if p.endswith(".rlib")
            )
        if name == "pose_observation_loss" and artifact.get("executable"):
            originals.add(Path(artifact["executable"]))
    require(
        len(originals) == 1 and all(len(values) == 1 for values in libraries.values()),
        "Ambiguous original Cargo artifacts",
    )
    links = {name: next(iter(values)) for name, values in libraries.items()}
    require(
        len({p.parent for p in links.values()}) == 1,
        "Cargo rlibs have different dependency directories",
    )
    compiler = str(rustc or shutil.which("rustc") or "rustc")
    command = [
        compiler,
        "--edition=2021",
        "-O",
        "-L",
        "dependency=" + str(next(iter(links.values())).parent),
    ]
    for name, path in links.items():
        command.extend(["--extern", name + "=" + str(path)])
    with (work / "rustc.log").open("w") as log:
        subprocess.run(
            [*command, str(source_path), "-o", str(work / "mean-speed-probe")],
            stdout=log,
            stderr=subprocess.STDOUT,
            check=True,
        )
        subprocess.run(
            [*command, "--test", str(source_path), "-o", str(work / "native-tests")],
            stdout=log,
            stderr=subprocess.STDOUT,
            check=True,
        )
    tests = subprocess.check_output([str(work / "native-tests")], text=True)
    (work / "native-tests.log").write_text(tests)
    count = re.search(r"test result: ok\. (\d+) passed; 0 failed; 0 ignored;", tests)
    require(count is not None and int(count[1]) >= 5, "Native tests did not complete")
    original_binary = next(iter(originals))
    receipt = dict(
        base_main=pins["base_main"],
        generator_sha256=sha(Path(__file__)),
        protocol_sha256=sha(STUDY / "protocol.md"),
        freeze_sha256=sha(STUDY / "freeze-pins.json"),
        input_sha256=pins["input_sha256"],
        rustc=subprocess.check_output([compiler, "--version"], text=True).strip(),
        cargo=subprocess.check_output(["cargo", "--version"], text=True).strip(),
        cargo_command=cargo_command,
        libraries={
            name: dict(filename=path.name, sha256=sha(path))
            for name, path in links.items()
        },
        generated_source_sha256=sha(source_path),
        diff_sha256=sha(work / "candidate.diff"),
        candidate_binary_sha256=sha(work / "mean-speed-probe"),
        original_binary_sha256=sha(original_binary),
        native_tests=int(count[1]),
        native_tests_sha256=sha(work / "native-tests.log"),
    )
    (work / "build-sources.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return work / "mean-speed-probe", original_binary, receipt


def arguments(inputs, output, mode, seed, tau, enabled=None):
    require(mode in ("none", "random", "blackout"), "Invalid loss mode")
    argv = [
        str(inputs),
        str(output),
        "1.0",
        "0.1",
        "2.0",
        str(seed),
        "0.02",
        "0.2",
        "5.0",
        "true",
        "true",
        "true",
        mode,
        "true",
        "true",
        str(tau),
        "true",
        "false",
        "true",
        "true",
        "false",
        "false",
        "true",
        "true",
        "true",
        "true",
        "true",
        "true",
    ]
    if enabled is not None:
        argv.append(str(enabled).lower())
    return argv


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not __debug__:
        parser.error("Run without Python optimization")
    try:
        binary, original, receipt = build(args.output)
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Mean speed-probe build failed: {error}\n")
    print(
        json.dumps(
            dict(
                binary=str(binary),
                original=str(original),
                native_tests=receipt["native_tests"],
            )
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
