#!/usr/bin/env python3
"""Diagnose every immediately following joint-release decision with fixed inputs."""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path
import subprocess

import analyze_guard_release_census as census
import analyze_mean_guard_preview as live
import analyze_mean_speed_probe as pilot
import build_mean_guard_preview as build
import native_guard_release_census as old_native
import native_post_release_transition as native

STUDY = build.ROOT / "results/post-release-preview-transition"
SCRIPTS = (
    "analyze_post_release_transition.py",
    "native_post_release_transition.py",
    "test_post_release_transition.py",
)
META = census.META
ALTERNATIVES = dict(current="current_recovery_xy", carried="carried_release_mean_xy")
FLIP_FIELDS = ("entry_avoided", "entry_added", "release_gain", "release_loss")
COUNT_FIELDS = (
    "resumes",
    "following",
    "packet_endpoints",
    "actual_reguards",
    "next_pending",
    "guard_switch_off",
    "next_window_retained",
    "next_window_empty",
    "next_window_invalidated",
    "next_moving",
    "next_zero_target",
    "next_fusion_available",
    "target_changed",
    "steering_changed",
    "yaw_changed",
    "speed_changed",
    "precision_changed",
    *(prefix + "_" + field for prefix in ALTERNATIVES for field in FLIP_FIELDS),
)
CASE_FIELDS = (*META, "status", *COUNT_FIELDS)
QUERY_FIELDS = (
    *META,
    "status",
    "global_id",
    "step",
    "time_s",
    *(f for f in native.TRACE_FIELDS if f != "scenario"),
    "would_enter",
    "release_clear",
    "position_delta_from_recorded_m",
)
RESUME_FIELDS = (
    "resume_global_id",
    "resume_step",
    "resume_time_s",
    "resume_guard_used",
    "resume_fusion_available",
    "resume_count",
    "resume_mean_x",
    "resume_mean_y",
    "resume_selected_x",
    "resume_selected_y",
    "resume_yaw",
    "resume_speed",
    "resume_target",
    "resume_steering",
    "resume_applied_steering",
    "resume_entry",
    "resume_release",
    "resume_release_margin",
    "has_next",
)
NEXT_FIELDS = (
    "next_global_id",
    "next_step",
    "next_time_s",
    "actual_reguard",
    "next_pending",
    "guard_switch_off",
    "next_guard_used",
    "next_count",
    "next_window_retained",
    "next_window_empty",
    "next_window_invalidated",
    "next_moving",
    "next_zero_target",
    "next_fusion_available",
    "next_correction",
    "next_received",
    "next_precision_activation",
    "next_age",
    "next_selected_x",
    "next_selected_y",
    "next_yaw",
    "next_speed",
    "next_target",
    "next_steering",
    "next_mean_x",
    "next_mean_y",
    "carried_x",
    "carried_y",
    "selected_position_jump_m",
    "base_position_jump_m",
    "current_position_delta_m",
    "carried_position_delta_m",
    "target_changed",
    "steering_changed",
    "yaw_changed",
    "speed_changed",
    "precision_changed",
    "next_entry",
    "next_release",
    "next_preview_margin",
    "next_release_margin",
    *(prefix + "_" + field for prefix in ALTERNATIVES for field in FLIP_FIELDS),
)
TRANSITION_FIELDS = ("transition_id", *META, "status", *RESUME_FIELDS, *NEXT_FIELDS)
VERIFY_FIELDS = (
    *META,
    "decisions",
    "resumes",
    "following",
    "recorded_geometry_matches",
)
PAIR_FIELDS = (
    *census.PAIR_KEYS,
    *(
        prefix + "_" + field
        for prefix in ("reference", "candidate")
        for field in ("status", *COUNT_FIELDS)
    ),
)
SUMMARY_FIELDS = (
    "noise_selector",
    "mode_selector",
    "arm",
    "outcome_selector",
    "cases",
    *COUNT_FIELDS,
)
need = build.require


def preflight():
    pins = json.loads((STUDY / "freeze-pins.json").read_text())
    need(
        build.sha(STUDY / "protocol.md") == pins["protocol_sha256"],
        "Transition protocol changed",
    )
    for name, digest in pins["input_sha256"].items():
        need(
            build.sha(build.REPO / name) == digest, f"Changed transition input: {name}"
        )
    need(
        tuple(pins["variants"]) == native.VARIANTS, "Frozen transition variants differ"
    )
    return pins


def verify_manifest(root, records):
    root = Path(root).resolve()
    need(
        {str(p.relative_to(root)) for p in root.rglob("*") if p.is_file()}
        == set(records),
        "Manifest file population differs",
    )
    for name, record in records.items():
        path = root / name
        need(
            path.stat().st_size == record["bytes"]
            and build.sha(path) == record["sha256"],
            f"Changed retained file: {name}",
        )


def datum(track, sides, guards, index, offset):
    return dict(
        row=track[index + 1],
        side=sides[index],
        guard=guards[index],
        global_id=offset + index + 1,
        step=index + 1,
    )


def select_case(meta, track, sides, guards, offset):
    need(len(track) == len(sides) + 1 == len(guards) + 1, "Incomplete transition case")
    result = []
    for i, row in enumerate(track[1:]):
        if pilot.truth(row["resumed"]):
            need(
                not pilot.truth(row["guard_active"])
                and not pilot.truth(row["safety_paused"]),
                "Release did not clear both brakes",
            )
            need(
                pilot.truth(guards[i]["joint_pending"]), "Resume was not joint pending"
            )
            result.append(
                dict(
                    meta=meta,
                    resume=datum(track, sides, guards, i, offset),
                    next=(
                        datum(track, sides, guards, i + 1, offset)
                        if i + 1 < len(sides)
                        else None
                    ),
                )
            )
    return result


def census_input(item, scenario):
    row, side, guard = item["row"], item["side"], item["guard"]
    return dict(
        zip(
            old_native.INPUT_FIELDS,
            (
                str(item["global_id"]),
                scenario,
                side["mean_x"],
                side["mean_y"],
                side["mean_yaw"],
                side["margin"],
                side["observation_age"],
                row["precision_active"],
                "0.1",
                "0.02",
                "false",
                guard["selected_x"],
                guard["selected_y"],
                guard["selected_yaw"],
                side["measured_speed"],
                side["nominal_target"],
                side["steering"],
            ),
        )
    )


def bind_geometry(directory, selected, expected):
    found = {}
    with (directory / "native/queries.tsv").open(newline="") as qstream, (
        directory / "native/trace.csv"
    ).open(newline="") as gstream:
        queries, geometries = csv.DictReader(qstream, delimiter="\t"), csv.DictReader(
            gstream
        )
        need(
            tuple(queries.fieldnames) == old_native.INPUT_FIELDS
            and tuple(geometries.fieldnames) == old_native.TRACE_FIELDS,
            "Retained native schema differs",
        )
        count = 0
        for count, pair in enumerate(itertools.zip_longest(queries, geometries), 1):
            query, geometry = pair
            need(
                query is not None and geometry is not None,
                "Retained census lengths differ",
            )
            need(
                query["id"] == geometry["id"] == str(count),
                "Retained census sequence differs",
            )
            need(
                None not in query and None not in geometry,
                "Malformed retained census row",
            )
            if count in selected:
                need(
                    query == selected[count],
                    "Selected input differs from full native census",
                )
                found[count] = geometry
    need(
        count == expected and set(found) == set(selected),
        "Retained census selection incomplete",
    )
    return found


def write_packet(work, transitions):
    packet = work / "queries.tsv"
    sequence = 0
    with packet.open("x", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=native.INPUT_FIELDS, delimiter="\t", lineterminator="\n"
        )
        writer.writeheader()
        for transition in transitions:
            previous = transition["resume"]["side"]
            for phase in ("resume", "next"):
                item = transition[phase]
                if item is None:
                    continue
                sequence += 1
                item["query_id"] = sequence
                record = census_input(item, transition["meta"]["scenario"])
                record.update(
                    id=sequence,
                    transition_id=transition["transition_id"],
                    phase=phase,
                    resume_mean_x=previous["mean_x"],
                    resume_mean_y=previous["mean_y"],
                    resume_mean_yaw=previous["mean_yaw"],
                    resume_speed=previous["measured_speed"],
                    resume_applied_steering=previous["applied_steering"],
                )
                writer.writerow(record)
    return packet, sequence


def threshold(geometry):
    return dict(
        would_enter=pilot.finite(geometry["entry"])
        <= pilot.finite(geometry["preview_margin"]),
        release_clear=pilot.finite(geometry["release"])
        > pilot.finite(geometry["release_margin"]),
    )


def flips(recorded, alternative):
    a, b = threshold(recorded), threshold(alternative)
    return dict(
        entry_avoided=a["would_enter"] and not b["would_enter"],
        entry_added=not a["would_enter"] and b["would_enter"],
        release_gain=not a["release_clear"] and b["release_clear"],
        release_loss=a["release_clear"] and not b["release_clear"],
    )


def validate_recorded(item, geometry, previous):
    for name in old_native.TRACE_FIELDS[1:]:
        need(
            geometry[name] == previous[name],
            f"Recorded native geometry differs: {name}",
        )
    census.gates(item["row"], item["side"], geometry)
    for fresh, old in (
        ("x", "selected_x"),
        ("y", "selected_y"),
        ("yaw", "selected_yaw"),
        ("speed", "selected_v"),
    ):
        need(
            pilot.finite(geometry[fresh]) == pilot.finite(item["guard"][old]),
            "Recorded pose differs",
        )


def consume_queries(transition, rows, old):
    result, exported = {}, []
    for phase in ("resume", "next"):
        item = transition[phase]
        if item is None:
            continue
        phase_rows = {}
        for variant in ("recorded",) if phase == "resume" else native.VARIANTS:
            geometry = next(rows, None)
            need(
                geometry is not None
                and geometry["id"] == str(item["query_id"])
                and geometry["transition_id"] == str(transition["transition_id"])
                and geometry["phase"] == phase
                and geometry["variant"] == variant
                and geometry["scenario"] == transition["meta"]["scenario"],
                "Native transition order differs",
            )
            need(
                None not in geometry and all(v is not None for v in geometry.values()),
                "Malformed native transition row",
            )
            for name in (
                *native.POSE_FIELDS,
                "recovery_clearance",
                "recovery_margin",
                "preview_margin",
                "release_margin",
                "stop",
                "terminal",
                "entry",
                "release",
            ):
                pilot.finite(geometry[name])
            phase_rows[variant] = geometry
            if variant == "recorded":
                validate_recorded(item, geometry, old[item["global_id"]])
            else:
                for name in (
                    "yaw",
                    "speed",
                    "target",
                    "steering",
                    "preview_margin",
                    "release_margin",
                    "recovery_clearance",
                    "recovery_bounds",
                    "recovery_margin",
                ):
                    need(
                        geometry[name] == phase_rows["recorded"][name],
                        f"Variant changed fixed field: {name}",
                    )
            distance = math.dist(
                [pilot.finite(geometry[k]) for k in ("x", "y")],
                [pilot.finite(phase_rows["recorded"][k]) for k in ("x", "y")],
            )
            exported.append(
                dict(
                    **transition["meta"],
                    status=transition["status"],
                    global_id=item["global_id"],
                    step=item["step"],
                    time_s=item["side"]["time_s"],
                    **{k: v for k, v in geometry.items() if k != "scenario"},
                    **threshold(geometry),
                    position_delta_from_recorded_m=distance,
                )
            )
        result[phase] = phase_rows
    return result, exported


def transition_row(transition, geometry):
    before, after = transition["resume"], transition["next"]
    side, guard, row = before["side"], before["guard"], before["row"]
    original = geometry["resume"]["recorded"]
    need(
        all(census.gates(row, side, original).values()),
        "A resume failed an original joint gate",
    )
    result = dict(
        transition_id=transition["transition_id"],
        **transition["meta"],
        status=transition["status"],
        resume_global_id=before["global_id"],
        resume_step=before["step"],
        resume_time_s=side["time_s"],
        resume_guard_used=guard["used"],
        resume_fusion_available=row["epoch_fusion_available"],
        resume_count=side["count"],
        resume_mean_x=side["mean_x"],
        resume_mean_y=side["mean_y"],
        resume_selected_x=guard["selected_x"],
        resume_selected_y=guard["selected_y"],
        resume_yaw=guard["selected_yaw"],
        resume_speed=side["measured_speed"],
        resume_target=side["nominal_target"],
        resume_steering=side["steering"],
        resume_applied_steering=side["applied_steering"],
        resume_entry=original["entry"],
        resume_release=original["release"],
        resume_release_margin=original["release_margin"],
        has_next=after is not None,
    )
    if after is None:
        return result
    new, selected, logged = after["side"], after["guard"], after["row"]
    actual = geometry["next"]["recorded"]
    pending = pilot.truth(selected["joint_pending"])
    empty = int(new["count"]) == 0
    if empty:
        need(
            all(new["mean_" + k] == new["raw_" + k] for k in ("x", "y", "yaw")),
            "Empty recovery window is not raw fallback",
        )
    elif not pending:
        need(
            new["count"] == side["count"]
            and all(new["mean_" + k] == side["mean_" + k] for k in ("x", "y", "yaw")),
            "Unpaused recovery window changed without reset",
        )
    trigger = pilot.truth(logged["guard_triggered"])
    need(
        trigger == threshold(actual)["would_enter"],
        "Next actual guard entry differs from inactive-guard threshold",
    )
    invalidated = (
        any(pilot.truth(new[k]) for k in ("received", "precision_activation"))
        or (pilot.truth(new["correction"]) and not pilot.truth(new["received"]))
        or abs(pilot.finite(new["measured_speed"])) > 0.02
        or pilot.finite(new["observation_age"]) > 1.5
    )
    distance = lambda a, b: math.dist(
        [pilot.finite(a[k]) for k in ("x", "y")],
        [pilot.finite(b[k]) for k in ("x", "y")],
    )
    base_before = {k: guard["base_" + k] for k in ("x", "y")}
    base_after = {k: selected["base_" + k] for k in ("x", "y")}
    result.update(
        next_global_id=after["global_id"],
        next_step=after["step"],
        next_time_s=new["time_s"],
        actual_reguard=trigger,
        next_pending=pending,
        guard_switch_off=pilot.truth(guard["used"])
        and not pilot.truth(selected["used"]),
        next_guard_used=selected["used"],
        next_count=new["count"],
        next_window_retained=not empty and not pending,
        next_window_empty=empty,
        next_window_invalidated=invalidated,
        next_moving=abs(pilot.finite(new["measured_speed"])) > 0.02,
        next_zero_target=pilot.finite(new["nominal_target"]) == 0,
        next_fusion_available=logged["epoch_fusion_available"],
        next_correction=new["correction"],
        next_received=new["received"],
        next_precision_activation=new["precision_activation"],
        next_age=new["observation_age"],
        next_selected_x=selected["selected_x"],
        next_selected_y=selected["selected_y"],
        next_yaw=selected["selected_yaw"],
        next_speed=new["measured_speed"],
        next_target=new["nominal_target"],
        next_steering=new["steering"],
        next_mean_x=new["mean_x"],
        next_mean_y=new["mean_y"],
        carried_x=geometry["next"]["carried_release_mean_xy"]["x"],
        carried_y=geometry["next"]["carried_release_mean_xy"]["y"],
        selected_position_jump_m=distance(actual, original),
        base_position_jump_m=distance(base_after, base_before),
        current_position_delta_m=distance(
            actual, geometry["next"]["current_recovery_xy"]
        ),
        carried_position_delta_m=distance(
            actual, geometry["next"]["carried_release_mean_xy"]
        ),
        target_changed=pilot.finite(new["nominal_target"])
        != pilot.finite(side["nominal_target"]),
        steering_changed=pilot.finite(new["steering"])
        != pilot.finite(side["steering"]),
        yaw_changed=selected["selected_yaw"] != guard["selected_yaw"],
        speed_changed=new["measured_speed"] != side["measured_speed"],
        precision_changed=logged["precision_active"] != row["precision_active"],
        next_entry=actual["entry"],
        next_release=actual["release"],
        next_preview_margin=actual["preview_margin"],
        next_release_margin=actual["release_margin"],
    )
    for prefix, variant in ALTERNATIVES.items():
        result.update(
            {
                prefix + "_" + k: v
                for k, v in flips(actual, geometry["next"][variant]).items()
            }
        )
    return result


def case_counts(meta, prior, rows):
    result = dict(**meta, status=prior["status"], **dict.fromkeys(COUNT_FIELDS, 0))
    need(
        len(rows) == int(prior["joint_resumes"]),
        "Case resume count differs from full census",
    )
    for row in rows:
        result["resumes"] += 1
        following = pilot.truth(str(row["has_next"]).lower())
        result["following"] += following
        result["packet_endpoints"] += not following
        if following:
            result["actual_reguards"] += row["actual_reguard"]
            for field in COUNT_FIELDS[4:]:
                value = row[field]
                result[field] += pilot.truth(value) if isinstance(value, str) else value
    if meta["mode"] == "none":
        need(result["resumes"] == 0, "No-loss control acquired joint resumes")
    return result


def paired(cases):
    groups = {}
    for case in cases:
        key = tuple(case[k] for k in census.PAIR_KEYS)
        need(
            case["arm"] not in groups.setdefault(key, {}),
            "Repeated transition pair arm",
        )
        groups[key][case["arm"]] = case
    result = []
    for key, arms in sorted(groups.items()):
        need(
            set(arms) == {"fused_preview", "mean_preview"},
            "Missing transition pair arm",
        )
        result.append(
            dict(zip(census.PAIR_KEYS, key))
            | {
                prefix + "_" + field: arms[arm][field]
                for prefix, arm in (
                    ("reference", "fused_preview"),
                    ("candidate", "mean_preview"),
                )
                for field in ("status", *COUNT_FIELDS)
            }
        )
    return result


def summarize(cases):
    result = []
    for noise, mode, arm, outcome in itertools.product(
        ("all", "white", "correlated"),
        ("all", "none", "random", "blackout"),
        ("fused_preview", "mean_preview"),
        ("all", "held", "timeout"),
    ):
        selected = [
            c
            for c in cases
            if (noise == "all" or (float(c["tau"]) == 0) == (noise == "white"))
            and (mode == "all" or c["mode"] == mode)
            and c["arm"] == arm
            and (outcome == "all" or c["status"] == outcome)
        ]
        result.append(
            dict(
                noise_selector=noise,
                mode_selector=mode,
                arm=arm,
                outcome_selector=outcome,
                cases=len(selected),
                **{field: sum(c[field] for c in selected) for field in COUNT_FIELDS},
            )
        )
    return result


def collect(raw, retained_census, output):
    pins = preflight()
    raw, retained_census, output = (
        Path(raw).resolve(),
        Path(retained_census).resolve(),
        build.fresh(output),
    )
    need(
        not output.is_relative_to(raw) and not output.is_relative_to(retained_census),
        "Output must be outside retained inputs",
    )
    need(
        build.sha(raw / "provenance.json") == pins["raw_provenance_sha256"],
        "Different transition raw packet",
    )
    provenance = live.check_provenance(raw, build.preflight())
    raw_manifest = json.loads((build.STUDY / "raw-file-manifest.json").read_text())
    verify_manifest(raw, raw_manifest["sha256"])
    repeat = json.loads((census.STUDY / "repeat-verification.json").read_text())
    verify_manifest(retained_census, repeat["sha256"])
    need(
        not subprocess.check_output(
            ["git", "status", "--porcelain=v1"], cwd=build.REPO, text=True
        ).strip(),
        "Commit transition implementation before collecting results",
    )
    implementation = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=build.REPO, text=True
    ).strip()
    prior = {r["case_id"]: r for r in pilot.read(census.STUDY / "cases.csv")}
    expected_events = {
        (r["case_id"], int(r["step"]))
        for r in pilot.read(census.STUDY / "events.csv")
        if r["event"] == "joint_resume"
    }
    output.mkdir(parents=True)
    try:
        transitions, catalog, offset = [], [], 0
        for meta, track, sides, guards in census.iter_cases(raw, provenance):
            live.validate_guard(meta, track, sides, guards)
            need(
                meta["case_id"] in prior
                and all(str(meta[k]) == prior[meta["case_id"]][k] for k in META),
                "Transition case binding differs",
            )
            selected = select_case(meta, track, sides, guards, offset)
            need(
                len(selected) == int(prior[meta["case_id"]]["joint_resumes"]),
                "Selected resume count differs",
            )
            catalog.append((meta, len(sides)))
            for transition in selected:
                transition.update(
                    transition_id=len(transitions) + 1,
                    status=prior[meta["case_id"]]["status"],
                )
                transitions.append(transition)
            offset += len(sides)
        need(
            offset == pins["expected_decisions"]
            and len(catalog)
            == len({m["case_id"] for m, _ in catalog})
            == pins["expected_cases"]
            and {m["case_id"] for m, _ in catalog} == set(prior),
            "Incomplete transition case population",
        )
        actual_events = {
            (t["meta"]["case_id"], t["resume"]["step"]) for t in transitions
        }
        need(
            actual_events == expected_events
            and len(transitions) == len(actual_events) == pins["expected_resumes"],
            "Incomplete transition event population",
        )
        selected_inputs = {
            t[phase]["global_id"]: census_input(t[phase], t["meta"]["scenario"])
            for t in transitions
            for phase in ("resume", "next")
            if t[phase] is not None
        }
        old = bind_geometry(retained_census, selected_inputs, offset)
        print(
            f"Bound all {offset} retained census rows and {len(transitions)} joint releases",
            flush=True,
        )
        work = output / "native"
        compiler = native.compile_native(work)
        packet, input_count = write_packet(work, transitions)
        evaluation = native.evaluate(work, packet)
        rows = iter(pilot.read(work / "trace.csv", native.TRACE_FIELDS))
        queries, records = [], []
        for transition in transitions:
            geometry, exported = consume_queries(transition, rows, old)
            queries.extend(exported)
            records.append(transition_row(transition, geometry))
        need(next(rows, None) is None, "Extra native transition output")
        by_case = {case_id: [] for case_id in prior}
        for record in records:
            by_case[record["case_id"]].append(record)
        cases, verification = [], []
        for meta, decisions in catalog:
            case = case_counts(meta, prior[meta["case_id"]], by_case[meta["case_id"]])
            cases.append(case)
            verification.append(
                dict(
                    **meta,
                    decisions=decisions,
                    resumes=case["resumes"],
                    following=case["following"],
                    recorded_geometry_matches=case["resumes"] + case["following"],
                )
            )
        cases.sort(key=lambda c: c["case_id"])
        verification.sort(key=lambda c: c["case_id"])
        pairs, summary = paired(cases), summarize(cases)
        following = sum(c["following"] for c in cases)
        need(
            len(pairs) == pins["expected_pairs"]
            and len(summary) == pins["expected_summary_cells"]
            and input_count == len(transitions) + following
            and len(queries) == len(transitions) + 3 * following,
            "Transition export conservation failed",
        )
        tables = (
            ("transitions.csv", records, TRANSITION_FIELDS),
            ("queries.csv", queries, QUERY_FIELDS),
            ("cases.csv", cases, CASE_FIELDS),
            ("pairs.csv", pairs, PAIR_FIELDS),
            ("summary.csv", summary, SUMMARY_FIELDS),
            ("verification.csv", verification, VERIFY_FIELDS),
        )
        for name, records, fields in tables:
            pilot.write(output / name, records, fields)
        sources = dict(
            method="fixed-input immediate post-joint-release native position query",
            implementation_commit=implementation,
            working_tree_clean=True,
            protocol_sha256=build.sha(STUDY / "protocol.md"),
            freeze_sha256=build.sha(STUDY / "freeze-pins.json"),
            input_sha256=pins["input_sha256"],
            raw_provenance_sha256=build.sha(raw / "provenance.json"),
            raw_manifest_sha256=build.sha(build.STUDY / "raw-file-manifest.json"),
            retained_census_repeat_sha256=build.sha(
                census.STUDY / "repeat-verification.json"
            ),
            script_sha256={name: build.sha(build.ROOT / name) for name in SCRIPTS},
            native=dict(compiler=compiler, evaluation=evaluation),
            audit=dict(
                cases=len(cases),
                pairs=len(pairs),
                retained_census_rows=offset,
                joint_resumes=len(transitions),
                following_decisions=following,
                packet_endpoints=len(transitions) - following,
                native_inputs=input_count,
                native_queries=len(queries),
                recorded_geometry_matches=input_count,
                fresh_simulator_steps=0,
            ),
            output_sha256={name: build.sha(output / name) for name, _, _ in tables},
        )
        (output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    except (
        OSError,
        ValueError,
        KeyError,
        AssertionError,
        subprocess.CalledProcessError,
    ) as error:
        (output / "failure.txt").write_text(str(error) + "\n")
        raise
    print(json.dumps(sources["audit"]), flush=True)
    return sources


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--census", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not __debug__:
        parser.error("Run without Python optimization")
    try:
        collect(args.raw, args.census, args.output)
    except (
        OSError,
        ValueError,
        KeyError,
        AssertionError,
        subprocess.CalledProcessError,
    ) as error:
        parser.exit(1, f"Post-release query failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
