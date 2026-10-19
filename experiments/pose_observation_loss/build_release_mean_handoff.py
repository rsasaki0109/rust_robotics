#!/usr/bin/env python3
"""Build a default-false, single-decision sensor mean handoff after joint resume."""

import argparse
import difflib
import json
from pathlib import Path
import re
import shutil
import subprocess

import build_mean_guard_preview as legacy

ROOT, REPO, BASE, FIXTURES = legacy.ROOT, legacy.REPO, legacy.BASE, legacy.FIXTURES
STUDY = ROOT / "results/post-release-mean-handoff-live-pilot"
require, sha, fresh, once = legacy.require, legacy.sha, legacy.fresh, legacy.once
SCRIPTS = (
    "build_release_mean_handoff.py",
    "run_release_mean_handoff.py",
    "native_release_mean_handoff.py",
    "analyze_release_mean_handoff.py",
    "test_release_mean_handoff.py",
)
HANDOFF_FIELDS = (
    "scenario",
    "planner",
    "step",
    "time_s",
    "enabled",
    "available",
    "valid",
    "used",
    "seeded",
    "correction",
    "received",
    "precision_activation",
    "joint_pending",
    "observation_age",
    "count",
    "before_x",
    "before_y",
    "before_yaw",
    "before_v",
    "selected_x",
    "selected_y",
    "selected_yaw",
    "selected_v",
    "carry_x",
    "carry_y",
    "carry_yaw",
    "carry_speed",
    "carry_steering",
)
HELPER = r"""
#[derive(Clone, Copy)]
struct ReleaseMean { x: f64, y: f64, yaw: f64, speed: f64, steering: f64 }

fn release_mean_preview(mut preview: VehicleState, snapshot: Option<ReleaseMean>,
    enabled: bool, valid: bool) -> (VehicleState, bool) {
    if enabled && valid {
        if let Some(old) = snapshot {
            let mut transport = MotionPoseFilter {
                samples: VecDeque::from([(old.x, old.y, old.yaw)]),
                previous_speed: old.speed,
            };
            transport.observe(preview, old.steering, false);
            let carried = transport.samples.front().unwrap();
            preview.x = carried.0; preview.y = carried.1;
            return (preview, true);
        }
    }
    (preview, false)
}

fn release_mean_valid(pending: bool, correction: bool, received: bool,
    precision_activation: bool, speed: f64, age: f64, count: usize) -> bool {
    !pending && !correction && !received && !precision_activation
        && speed.abs() <= 0.02 && age <= 1.5 && count >= 10
}

#[cfg(test)]
mod release_mean_handoff_tests {
    use super::*;
    fn snapshot() -> ReleaseMean { ReleaseMean { x: 12.0, y: 13.0, yaw: 1.1, speed: -0.02, steering: 0.35 } }
    #[test]
    fn consumes_exactly_once_even_without_a_second_resume() {
        let mut stored = Some(snapshot());
        let p = VehicleState::new(10.0, 11.0, -0.4, 0.017, WHEELBASE);
        assert!(release_mean_preview(p, stored.take(), true, true).1);
        assert!(!release_mean_preview(p, stored.take(), true, true).1);
    }
    #[test]
    fn invalid_consumption_cannot_reappear() {
        let mut stored = Some(snapshot());
        let p = VehicleState::new(10.0, 11.0, 0.0, 0.0, WHEELBASE);
        assert!(!release_mean_preview(p, stored.take(), true, false).1);
        assert!(!release_mean_preview(p, stored.take(), true, true).1);
    }
    #[test]
    fn transport_uses_old_measured_speed_and_heading_not_new_heading() {
        let old = snapshot();
        let p = VehicleState::new(10.0, 11.0, -0.4, 0.017, WHEELBASE);
        let (q, used) = release_mean_preview(p, Some(old), true, true);
        assert!(used);
        assert_eq!(q.x, old.x + old.speed * DT * old.yaw.cos());
        assert_eq!(q.y, old.y + old.speed * DT * old.yaw.sin());
        assert_eq!((q.yaw,q.v,q.wheelbase,q.rear_x,q.rear_y), (p.yaw,p.v,p.wheelbase,p.rear_x,p.rear_y));
    }
    #[test]
    fn disabled_or_invalid_preserves_every_field() {
        for enabled in [false,true] { for valid in [false,true] {
            if enabled && valid { continue; }
            let p=VehicleState::new(10.0,11.0,0.7,-0.02,WHEELBASE);
            let (q,used)=release_mean_preview(p,Some(snapshot()),enabled,valid);
            assert!(!used);
            assert_eq!((q.x,q.y,q.yaw,q.v,q.wheelbase,q.rear_x,q.rear_y),(p.x,p.y,p.yaw,p.v,p.wheelbase,p.rear_x,p.rear_y));
        } }
    }
    #[test]
    fn each_reset_or_joint_pause_invalidates() {
        for i in 0..4 {
            let mut flags=[false;4];flags[i]=true;
            assert!(!release_mean_valid(flags[0],flags[1],flags[2],flags[3],0.0,0.0,25));
        }
    }
    #[test]
    fn exact_stationary_age_and_count_boundaries() {
        assert!(release_mean_valid(false,false,false,false,0.02,1.5,10));
        assert!(release_mean_valid(false,false,false,false,-0.02,1.5,10));
        assert!(!release_mean_valid(false,false,false,false,0.020001,1.5,10));
        assert!(!release_mean_valid(false,false,false,false,0.0,1.500001,10));
        assert!(!release_mean_valid(false,false,false,false,0.0,1.5,9));
    }
    #[test]
    fn no_snapshot_cannot_invent_a_handoff() {
        let p=VehicleState::new(10.0,11.0,0.7,0.0,WHEELBASE);
        assert!(!release_mean_preview(p,None,true,true).1);
    }
    #[test]
    fn changed_position_reaches_original_curved_preview() {
        let p=VehicleState::new(10.0,10.0,0.0,0.0,WHEELBASE);
        let (q,_)=release_mean_preview(p,Some(snapshot()),true,true);
        let obs=[Point2D::new(10.4,10.0)];
        assert_ne!(stopping_preview(p,0.4,0.35,0.5,&obs,true,true),stopping_preview(q,0.4,0.35,0.5,&obs,true,true));
    }
}
"""


def preflight():
    legacy.preflight()
    pins = json.loads((STUDY / "freeze-pins.json").read_text())
    require(
        sha(STUDY / "protocol.md") == pins["protocol_sha256"],
        "Handoff protocol changed",
    )
    require(
        sha(STUDY / "seed-audit.json") == pins["seed_audit_sha256"],
        "Handoff seed audit changed",
    )
    for name, digest in pins["input_sha256"].items():
        require(sha(REPO / name) == digest, f"Changed handoff input: {name}")
    audit = json.loads((STUDY / "seed-audit.json").read_text())
    for name, digest in audit["case_catalogue_sha256"].items():
        require(sha(REPO / name) == digest, f"Changed handoff seed catalogue: {name}")
    seed = 1001
    while seed in audit["observed_seeds"]:
        seed += 1
    require(seed == pins["seed"] == audit["chosen_seed"], "Handoff seed choice differs")
    return pins


def generate(original):
    source = legacy.generate(original)
    source = once(
        source,
        "if !(15..=31).contains(&args.len())",
        "if !(15..=32).contains(&args.len())",
    )
    anchor = "    let mean_guard_preview = args.get(30).map(|s| s.parse::<bool>()).transpose()?.unwrap_or(false);"
    source = once(
        source,
        anchor,
        anchor
        + "\n    let post_release_mean_handoff = args.get(31).map(|s| s.parse::<bool>()).transpose()?.unwrap_or(false);\n    assert!(!post_release_mean_handoff || mean_guard_preview);",
    )
    source = once(
        source,
        "; optional MEAN_SPEED_PROBE and final MEAN_GUARD_PREVIEW booleans;",
        "; optional MEAN_SPEED_PROBE, MEAN_GUARD_PREVIEW and final POST_RELEASE_MEAN_HANDOFF booleans;",
    )
    anchor = '    let mut tracks = BufWriter::new(File::create(output.join("trajectories.csv"))?);'
    source = once(
        source,
        anchor,
        anchor
        + '\n    let mut handoff_log = BufWriter::new(File::create(output.join("handoff-inputs.csv"))?);\n    writeln!(handoff_log, "'
        + ",".join(HANDOFF_FIELDS)
        + '")?;',
    )
    anchor = "        let mut previous_steering = 0.0;"
    source = once(
        source,
        anchor,
        anchor + "\n        let mut post_release_snapshot: Option<ReleaseMean> = None;",
    )
    anchor = "            preview_pose = selected_guard_pose;"
    source = once(
        source,
        anchor,
        """            let handoff_before = selected_guard_pose;
            let handoff_snapshot = post_release_snapshot.take();
            let handoff_available = handoff_snapshot.is_some();
            let handoff_valid = release_mean_valid(joint_pending, correction, correction_received,
                precision_activation, measured_speed, observation_age, recovery_sample_count);
            let (selected_guard_pose, handoff_used) = release_mean_preview(selected_guard_pose,
                handoff_snapshot, post_release_mean_handoff, handoff_valid);
            preview_pose = selected_guard_pose;""",
    )
    anchor = '            writeln!(tracks, "{scenario},{planner},{time:.6},'
    log = r"""            let handoff_seeded = post_release_mean_handoff && resumed;
            if handoff_seeded {
                post_release_snapshot = Some(ReleaseMean { x: recovery_x, y: recovery_y,
                    yaw: recovery_yaw, speed: measured_speed, steering });
            }
            let logged_carry = handoff_snapshot.unwrap_or(ReleaseMean { x:0.0,y:0.0,yaw:0.0,speed:0.0,steering:0.0 });
            writeln!(handoff_log, "{scenario},{planner},{step},{observation_time:.17e},{post_release_mean_handoff},{handoff_available},{handoff_valid},{handoff_used},{handoff_seeded},{correction},{correction_received},{precision_activation},{joint_pending},{observation_age:.17e},{recovery_sample_count},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e}", handoff_before.x,handoff_before.y,handoff_before.yaw,handoff_before.v,selected_guard_pose.x,selected_guard_pose.y,selected_guard_pose.yaw,selected_guard_pose.v,logged_carry.x,logged_carry.y,logged_carry.yaw,logged_carry.speed,logged_carry.steering)?;
"""
    return once(source, anchor, log + anchor) + HELPER


def arguments(inputs, output, mode, seed, tau, enabled=None):
    args = legacy.arguments(inputs, output, mode, seed, tau, True)
    if enabled is not None:
        args.append(str(enabled).lower())
    return args


def build(work):
    pins = preflight()
    work = fresh(work)
    work.mkdir(parents=True)
    old, _, original, old_receipt = legacy.build(work / "legacy")
    source = work / "release-mean-handoff.rs"
    source.write_text(generate(BASE.read_text()))
    (work / "candidate.diff").write_text(
        "".join(
            difflib.unified_diff(
                BASE.read_text().splitlines(keepends=True),
                source.read_text().splitlines(keepends=True),
                fromfile="pinned-pose-observation-loss.rs",
                tofile=source.name,
            )
        )
    )
    links = {}
    for line in (work / "legacy/legacy/cargo-artifacts.jsonl").read_text().splitlines():
        item = json.loads(line)
        if (
            item.get("reason") == "compiler-artifact"
            and item["target"]["name"] in old_receipt["libraries"]
        ):
            paths = [Path(p) for p in item["filenames"] if p.endswith(".rlib")]
            require(len(paths) == 1, "Ambiguous handoff rlib")
            links[item["target"]["name"]] = paths[0]
    require(set(links) == set(old_receipt["libraries"]), "Missing handoff rlibs")
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
        require(
            sha(path) == old_receipt["libraries"][name]["sha256"],
            "Handoff rlib changed",
        )
        command.extend(["--extern", name + "=" + str(path)])
    commands = [
        [*command, source.name, "-o", "release-mean-handoff"],
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
    require(
        count is not None and int(count[1]) == 35, "Handoff native tests incomplete"
    )
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
        candidate_binary_sha256=sha(work / "release-mean-handoff"),
        legacy_binary_sha256=sha(old),
        original_binary_sha256=sha(original),
        native_tests=int(count[1]),
        native_tests_sha256=sha(work / "native-tests.log"),
    )
    (work / "build-sources.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return work / "release-mean-handoff", old, original, receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not __debug__:
        parser.error("Run without Python optimization")
    try:
        binary, _, _, receipt = build(args.output)
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Handoff build failed: {error}\n")
    print(json.dumps(dict(binary=str(binary), native_tests=receipt["native_tests"])))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
