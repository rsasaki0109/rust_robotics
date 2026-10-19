#!/usr/bin/env python3
"""Build an opt-in mean-position guard preview from frozen simulator sources."""

import argparse
import difflib
import json
from pathlib import Path
import re
import shutil
import subprocess

import build_mean_speed_probe as legacy

ROOT, REPO, BASE, FIXTURES = legacy.ROOT, legacy.REPO, legacy.BASE, legacy.FIXTURES
STUDY = ROOT / "results/mean-guard-preview-live-pilot"
require, sha, fresh, once = legacy.require, legacy.sha, legacy.fresh, legacy.once
GUARD_FIELDS = (
    "scenario",
    "planner",
    "step",
    "time_s",
    "enabled",
    "used",
    "joint_pending",
    "speed_used",
    "base_x",
    "base_y",
    "base_yaw",
    "base_v",
    "selected_x",
    "selected_y",
    "selected_yaw",
    "selected_v",
    "base_x_9",
    "base_y_9",
    "base_yaw_9",
)
SCRIPTS = (
    "build_mean_guard_preview.py",
    "run_mean_guard_preview.py",
    "native_mean_guard_preview.py",
    "analyze_mean_guard_preview.py",
    "test_mean_guard_preview.py",
)

SELECTOR = r"""
// Only the guard-preview position changes; every other state field is preserved.
fn mean_guard_preview_pose(
    mut preview: VehicleState, enabled: bool, joint_pending: bool,
    mean_speed_probe_used: bool, mean_x: f64, mean_y: f64,
) -> (VehicleState, bool) {
    let used = enabled && joint_pending && mean_speed_probe_used;
    if used { preview.x = mean_x; preview.y = mean_y; }
    (preview, used)
}

#[cfg(test)]
mod mean_guard_preview_tests {
    use super::*;
    #[test]
    fn every_ineligible_flag_combination_preserves_all_fields() {
        for enabled in [false, true] { for pending in [false, true] { for speed_used in [false, true] {
            let raw = VehicleState::new(10.0, 11.0, 0.7, -0.02, 0.4);
            let (pose, used) = mean_guard_preview_pose(raw, enabled, pending, speed_used, 20.0, 21.0);
            assert_eq!(used, enabled && pending && speed_used);
            if !used { assert_eq!((pose.x, pose.y), (raw.x, raw.y)); }
            assert_eq!((pose.yaw, pose.v, pose.wheelbase, pose.rear_x, pose.rear_y),
                (raw.yaw, raw.v, raw.wheelbase, raw.rear_x, raw.rear_y));
        } } }
    }
    #[test]
    fn active_changes_only_position_including_zero_speed() {
        for speed in [0.0, 0.02, -0.02] {
            let raw = VehicleState::new(10.0, 11.0, 0.7, speed, 0.4);
            let (pose, used) = mean_guard_preview_pose(raw, true, true, true, 20.0, 21.0);
            assert!(used); assert_eq!((pose.x, pose.y), (20.0, 21.0));
            assert_eq!((pose.yaw, pose.v, pose.wheelbase, pose.rear_x, pose.rear_y),
                (raw.yaw, raw.v, raw.wheelbase, raw.rear_x, raw.rear_y));
        }
    }
    #[test]
    fn selected_position_reaches_original_stopping_and_terminal_geometry() {
        let raw = VehicleState::new(10.0, 10.0, 0.0, 0.0, 0.4);
        let obs = [Point2D::new(10.4, 10.0)];
        let (pose, _) = mean_guard_preview_pose(raw, true, true, true, 12.0, 10.0);
        for target in [0.0, 0.2] {
            let a = stopping_preview(raw, target, 0.0, 0.5, &obs, true, true);
            let b = stopping_preview(pose, target, 0.0, 0.5, &obs, true, true);
            assert_ne!(a, b); assert!(a.0 < b.0); assert!(a.1 < b.1);
        }
    }
}
"""


def preflight():
    legacy.preflight()
    pins = json.loads((STUDY / "freeze-pins.json").read_text())
    require(
        sha(STUDY / "protocol.md") == pins["protocol_sha256"], "Guard protocol changed"
    )
    require(
        sha(STUDY / "seed-audit.json") == pins["seed_audit_sha256"],
        "Seed audit changed",
    )
    for name, digest in pins["input_sha256"].items():
        require(sha(REPO / name) == digest, f"Changed frozen guard input: {name}")
    audit = json.loads((STUDY / "seed-audit.json").read_text())
    for name, digest in audit["case_catalogue_sha256"].items():
        require(sha(REPO / name) == digest, f"Changed guard seed catalogue: {name}")
    seed = 1001
    observed = set(audit["observed_seeds"])
    while seed in observed:
        seed += 1
    require(
        seed == pins["seed"] == audit["chosen_seed"], "Guard seed selection differs"
    )
    return pins


def generate(original):
    source = legacy.generate(original)
    source = once(
        source,
        "if !(15..=30).contains(&args.len())",
        "if !(15..=31).contains(&args.len())",
    )
    anchor = "    assert!(!mean_speed_probe || (joint_recovery && averaged_recovery));"
    source = once(
        source,
        anchor,
        anchor
        + """
    let mean_guard_preview = args.get(30).map(|s| s.parse::<bool>()).transpose()?.unwrap_or(false);
    assert!(!mean_guard_preview || (mean_speed_probe && joint_recovery && averaged_recovery
        && stopping_guard && motion_filter && stationary_epoch_fusion && terminal_steering_preview
        && recovery_preview && aligned_terminal_release && terminal_entry_only && !consistent_restart));""",
    )
    source = once(
        source,
        "; optional final MEAN_SPEED_PROBE boolean;",
        "; optional MEAN_SPEED_PROBE and final MEAN_GUARD_PREVIEW booleans;",
    )
    anchor = '    let mut tracks = BufWriter::new(File::create(output.join("trajectories.csv"))?);'
    source = once(
        source,
        anchor,
        anchor
        + '\n    let mut guard_log = BufWriter::new(File::create(output.join("guard-preview-inputs.csv"))?);\n    writeln!(guard_log, "'
        + ",".join(GUARD_FIELDS)
        + '")?;',
    )
    anchor = "            let (preview_pose_x, preview_pose_y, preview_pose_yaw) ="
    source = once(
        source,
        anchor,
        """            let mean_guard_base_pose = preview_pose;
            let (selected_guard_pose, mean_guard_preview_used) = mean_guard_preview_pose(
                preview_pose, mean_guard_preview, joint_pending, mean_speed_probe_used,
                recovery_x, recovery_y,
            );
            preview_pose = selected_guard_pose;
""" + anchor,
    )
    anchor = '            writeln!(tracks, "{scenario},{planner},{time:.6},'
    source = once(
        source,
        anchor,
        r"""            writeln!(guard_log, "{scenario},{planner},{step},{observation_time:.17e},{mean_guard_preview},{mean_guard_preview_used},{joint_pending},{mean_speed_probe_used},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.9},{:.9},{:.9}", mean_guard_base_pose.x, mean_guard_base_pose.y, mean_guard_base_pose.yaw, mean_guard_base_pose.v, selected_guard_pose.x, selected_guard_pose.y, selected_guard_pose.yaw, selected_guard_pose.v, mean_guard_base_pose.x, mean_guard_base_pose.y, mean_guard_base_pose.yaw)?;
"""
        + anchor,
    )
    return source + SELECTOR


def arguments(inputs, output, mode, seed, tau, enabled=None):
    args = legacy.arguments(inputs, output, mode, seed, tau, True)
    if enabled is not None:
        args.append(str(enabled).lower())
    return args


def build(work):
    pins = preflight()
    work = fresh(work)
    work.mkdir(parents=True)
    old_binary, original, old_receipt = legacy.build(work / "legacy")
    (work / "original").mkdir()
    archived_original = work / "original/pose_observation_loss"
    shutil.copy2(original, archived_original)
    original = archived_original
    original_source = BASE.read_text()
    source = work / "mean-guard-preview.rs"
    source.write_text(generate(original_source))
    (work / "candidate.diff").write_text(
        "".join(
            difflib.unified_diff(
                original_source.splitlines(keepends=True),
                source.read_text().splitlines(keepends=True),
                fromfile="pinned-pose-observation-loss.rs",
                tofile=source.name,
            )
        )
    )
    links = {}
    for line in (work / "legacy/cargo-artifacts.jsonl").read_text().splitlines():
        artifact = json.loads(line)
        if (
            artifact.get("reason") == "compiler-artifact"
            and artifact["target"]["name"] in old_receipt["libraries"]
        ):
            name = artifact["target"]["name"]
            paths = [Path(p) for p in artifact["filenames"] if p.endswith(".rlib")]
            require(len(paths) == 1, "Ambiguous native library")
            links[name] = paths[0]
    require(set(links) == set(old_receipt["libraries"]), "Missing legacy rlibs")
    for name, path in links.items():
        require(
            sha(path) == old_receipt["libraries"][name]["sha256"],
            "Legacy library changed",
        )
    compiler = shutil.which("rustc")
    require(compiler is not None, "Activate Rust")
    command = [
        compiler,
        "--edition=2021",
        "-O",
        "-L",
        "dependency=" + str(next(iter(links.values())).parent),
    ]
    for name, path in links.items():
        command.extend(["--extern", name + "=" + str(path)])
    commands = [
        [*command, source.name, "-o", "mean-guard-preview"],
        [*command, "--test", source.name, "-o", "native-tests"],
    ]
    with (work / "rustc.log").open("x") as log:
        for cmd in commands:
            subprocess.run(
                cmd, cwd=work, stdout=log, stderr=subprocess.STDOUT, check=True
            )
    tests = subprocess.check_output([str(work / "native-tests")], text=True)
    (work / "native-tests.log").write_text(tests)
    count = re.search(r"test result: ok\. (\d+) passed; 0 failed; 0 ignored;", tests)
    require(count is not None and int(count[1]) >= 27, "Native guard tests incomplete")
    receipt = dict(
        base_main=pins["base_main"],
        generator_sha256=sha(Path(__file__)),
        protocol_sha256=sha(STUDY / "protocol.md"),
        freeze_sha256=sha(STUDY / "freeze-pins.json"),
        input_sha256=pins["input_sha256"],
        rustc=old_receipt["rustc"],
        cargo=old_receipt["cargo"],
        commands=commands,
        libraries=old_receipt["libraries"],
        legacy_receipt_sha256=sha(work / "legacy/build-sources.json"),
        generated_source_sha256=sha(source),
        diff_sha256=sha(work / "candidate.diff"),
        candidate_binary_sha256=sha(work / "mean-guard-preview"),
        legacy_binary_sha256=sha(old_binary),
        original_binary_sha256=sha(original),
        native_tests=int(count[1]),
        native_tests_sha256=sha(work / "native-tests.log"),
    )
    (work / "build-sources.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return work / "mean-guard-preview", old_binary, original, receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not __debug__:
        parser.error("Run without Python optimization")
    try:
        binary, _, _, receipt = build(args.output)
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Mean guard-preview build failed: {error}\n")
    print(json.dumps(dict(binary=str(binary), native_tests=receipt["native_tests"])))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
