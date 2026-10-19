"""Independently replay all sensor histories and a one-shot release snapshot."""

from pathlib import Path
import shutil
import subprocess

import build_release_mean_handoff as build
import native_mean_guard_preview as previous

EXTRA_FIELDS = (
    "handoff_enabled",
    "handoff_available",
    "handoff_valid",
    "handoff_used",
    "handoff_seeded",
    "resumed",
    "before_x",
    "before_y",
    "before_yaw",
)
HISTORY_FIELDS = (*previous.HISTORY_FIELDS, *EXTRA_FIELDS)
CHECK_FIELDS = previous.CHECK_FIELDS
TRACE_FIELDS = (*previous.TRACE_FIELDS, "handoff_used")


def source():
    source = previous.source()
    source = build.once(
        source,
        "assert_eq!(f.len(), " + str(len(previous.HISTORY_FIELDS)) + ");",
        "assert_eq!(f.len(), " + str(len(HISTORY_FIELDS)) + ");",
    )
    source = build.once(
        source, "\t".join(previous.HISTORY_FIELDS), "\t".join(HISTORY_FIELDS)
    )
    source = build.once(source, ",".join(previous.TRACE_FIELDS), ",".join(TRACE_FIELDS))
    constants = "\n".join(
        f"const {name.upper()}: usize = {i};"
        for i, name in enumerate(HISTORY_FIELDS)
        if name in EXTRA_FIELDS
    )
    source = build.once(source, "fn main() {", constants + "\nfn main() {")
    source = build.once(
        source,
        "    let mut previous_steering = 0.0;",
        "    let mut previous_steering = 0.0;\n    let mut handoff: Option<(f64,f64,f64,f64,f64)> = None;",
    )
    source = build.once(
        source,
        "            previous_steering = 0.0;",
        "            previous_steering = 0.0;\n            handoff = None;",
    )
    anchor = '        assert_eq!((preview.x, preview.y, preview.yaw), (number(SELECTED_X), number(SELECTED_Y), number(SELECTED_YAW)), "Selected pose differs at {}", sequence);'
    body = r"""        assert_eq!((preview.x,preview.y,preview.yaw),(number(BEFORE_X),number(BEFORE_Y),number(BEFORE_YAW)), "Before-handoff pose differs at {}", sequence);
        let saved = handoff.take();
        assert_eq!(saved.is_some(),flag(HANDOFF_AVAILABLE));
        let valid = !flag(JOINT_PENDING) && !flag(CORRECTION) && !flag(RECEIVED) && !activation
            && sensed.v.abs() <= 0.02 && number(OBSERVATION_AGE) <= 1.5 && f[RECOVERY_COUNT].parse::<usize>().unwrap() >= 10;
        assert_eq!(valid,flag(HANDOFF_VALID));
        let handoff_used = flag(HANDOFF_ENABLED) && valid && saved.is_some();
        assert_eq!(handoff_used,flag(HANDOFF_USED));
        if handoff_used {
            let (x,y,yaw,v,steering) = saved.unwrap();
            let mut carry = MotionPoseFilter { samples: VecDeque::from([(x,y,yaw)]), previous_speed:v };
            carry.observe(sensed,steering,false);
            let old = carry.samples.front().unwrap(); preview.x = old.0; preview.y = old.1;
        }
"""
    source = build.once(source, anchor, body + anchor)
    source = build.once(
        source,
        '        assert!(!used || f[ID] != "0", "Every guard use requires an original stopping query");',
        '        assert!(!(used || handoff_used) || f[ID] != "0", "Every selector use requires an original stopping query");',
    )
    source = build.once(
        source,
        '            writeln!(output, "{}", values.join(",")).unwrap();',
        '            values.push(handoff_used.to_string());\n            writeln!(output, "{}", values.join(",")).unwrap();',
    )
    anchor = "        previous_steering = number(APPLIED_STEERING);"
    source = build.once(
        source,
        anchor,
        """        let seeded = flag(HANDOFF_ENABLED) && flag(RESUMED);
        assert_eq!(seeded,flag(HANDOFF_SEEDED));
        if seeded {
            assert!(flag(JOINT_PENDING) && eligible);
            handoff = Some((number(MEAN_X),number(MEAN_Y),number(MEAN_YAW),sensed.v,number(APPLIED_STEERING)));
        }
""" + anchor,
    )
    return source


def compile_native(work):
    work = build.fresh(work)
    work.mkdir(parents=True)
    (work / "handoff-history.rs").write_text(source())
    compiler = shutil.which("rustc")
    build.require(compiler is not None, "Activate Rust")
    command = [
        compiler,
        "--edition=2021",
        "-O",
        "handoff-history.rs",
        "-o",
        "native-handoff-history",
    ]
    with (work / "compiler.log").open("x") as log:
        subprocess.run(
            command, cwd=work, stdout=log, stderr=subprocess.STDOUT, check=True
        )
    return dict(
        rustc=subprocess.check_output([compiler, "--version"], text=True).strip(),
        command=command,
        source_sha256=build.sha(work / "handoff-history.rs"),
        binary_sha256=build.sha(work / "native-handoff-history"),
    )


def evaluate(work, packet, map_path=build.FIXTURES / "obstacles.csv"):
    work, packet = Path(work), Path(packet)
    build.require(
        packet.parent.resolve() == work.resolve(), "Handoff history must be local"
    )
    names = ("trace.csv", "history-checks.csv", "native.log")
    build.require(
        not any((work / n).exists() for n in names), "Native handoff outputs exist"
    )
    command = [
        "./native-handoff-history",
        str(map_path),
        packet.name,
        names[1],
        "exact-original-functions",
    ]
    with (work / names[0]).open("x") as output, (work / names[2]).open("x") as log:
        subprocess.run(command, cwd=work, stdout=output, stderr=log, check=True)
    return dict(
        command=command,
        sha256={name: build.sha(work / name) for name in (packet.name, *names)},
    )
