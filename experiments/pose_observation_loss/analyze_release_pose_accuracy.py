#!/usr/bin/env python3
"""Evaluate frozen sensor poses against aligned pre-update physical checkpoints."""

import argparse
from collections import defaultdict
from contextlib import ExitStack
import csv
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys

import analyze_handoff_transition_replay as previous
import analyze_mean_speed_probe as pilot
import build_release_mean_handoff as build
import native_guard_release_census as native

STUDY = build.ROOT / "results/post-release-pose-accuracy"
SCRIPTS = ("analyze_release_pose_accuracy.py", "test_release_pose_accuracy.py")
VARIANTS = (
    *previous.native.VARIANTS,
    "base_estimate_xy",
    "current_recovery_xy",
    "truth_xy_eval",
    "truth_pose_eval",
)
SCOPES = (
    "lag0",
    "lag1",
    "lag2",
    "within5",
    "within25",
    "within50",
    "triggers_within50",
)
META = previous.META
TIE = 1e-12
DISAGREEMENTS = (
    "entry_only_xy",
    "entry_missed_xy",
    "entry_only_pose",
    "entry_missed_pose",
    "ready_entry_only_xy",
    "ready_entry_missed_xy",
    "ready_entry_only_pose",
    "ready_entry_missed_pose",
    "release_only_pose",
    "release_missed_pose",
)
COUNT_FIELDS = (
    "queries",
    "eligible_queries",
    "ready_queries",
    "zero_target_queries",
    "closer",
    "farther",
    "tied",
    "eligible_closer",
    "eligible_farther",
    "eligible_tied",
    *DISAGREEMENTS,
)
ERROR_FIELDS = (
    "mean_position_error_m",
    "rmse_position_error_m",
    "max_position_error_m",
    "mean_signed_x_error_m",
    "mean_signed_y_error_m",
    "mean_abs_yaw_error_rad",
    "mean_entry_error_xy_m",
    "mean_entry_error_pose_m",
    "eligible_mean_position_error_m",
    "eligible_rmse_position_error_m",
)
METRICS = (*COUNT_FIELDS, *ERROR_FIELDS)
CASE_FIELDS = (*META, "status", "variant", "resumes", *METRICS)
PAIR_FIELDS = (
    *previous.PAIR_KEYS,
    "variant",
    *(
        p + "_" + k
        for p in ("reference", "candidate")
        for k in ("status", "resumes", *METRICS)
    ),
)
SUMMARY_FIELDS = (
    "noise_selector",
    "mode_selector",
    "arm",
    "outcome_selector",
    "scope",
    "variant",
    "cases",
    "resumes",
    *METRICS,
)
VERIFY_COUNTS = (
    "physical_steps",
    "pre_speed_matches",
    "post_state_matches",
    "window_points",
    "prefix_matches",
    "transport_matches",
    "prior_variant_matches",
)
VERIFY_FIELDS = (*META, *VERIFY_COUNTS)
SAMPLE_FIELDS = (
    "query_id",
    "point_id",
    "transition_id",
    *META,
    "status",
    "resume_step",
    "step",
    "lag",
    "variant",
    "eligible",
    "entry_ready",
    "actual_trigger",
    "selected_source",
    "recovery_source",
    "count",
    "observation_age",
    "true_x",
    "true_y",
    "true_yaw",
    "true_v",
    "x",
    "y",
    "yaw",
    "speed",
    "target",
    "steering",
    *(k for k in native.TRACE_FIELDS if k != "id"),
    "error_x_m",
    "error_y_m",
    "position_error_m",
    "abs_yaw_error_rad",
    "entry_error_xy_m",
    "entry_error_pose_m",
    "comparison",
    *DISAGREEMENTS,
)
RESUME_FIELDS = (
    "transition_id",
    *META,
    "status",
    "resume_step",
    "followup_steps",
    "first_invalid_lag",
    "first_trigger_lag",
    *(
        f"lag{lag}_{name}"
        for lag in (0, 1, 2)
        for name in (
            "recorded_error_m",
            "carried_error_m",
            "base_error_m",
            "current_recovery_error_m",
        )
    ),
    "lag2_eligible",
    "lag2_actual_trigger",
    "lag2_recorded_enter",
    "lag2_carried_enter",
    "lag2_truth_xy_enter",
    "lag2_truth_pose_enter",
    "lag2_carried_comparison",
)
need = build.require


def preflight():
    need(sys.flags.optimize == 0, "Python-O is unsupported")
    pins = json.loads((STUDY / "freeze-pins.json").read_text())
    need(
        build.sha(STUDY / "protocol.md") == pins["protocol_sha256"],
        "Accuracy protocol changed",
    )
    for name, digest in pins["input_sha256"].items():
        need(build.sha(build.REPO / name) == digest, f"Changed accuracy input: {name}")
    need(
        tuple(pins["variants"]) == VARIANTS
        and tuple(pins["scopes"]) == SCOPES
        and pins["horizon"] == 50
        and pins["tie_epsilon_m"] == TIE,
        "Accuracy design differs",
    )
    return pins


def angular(value):
    return math.atan2(math.sin(value), math.cos(value))


def physical_checkpoints(track, sides):
    """Sidecar truth is POST-update; retain the predecessor before each update."""
    need(
        len(track) == len(sides) + 1 and float(track[0]["time_s"]) == 0,
        "Physical checkpoint population differs",
    )
    before = tuple(pilot.finite(track[0][k]) for k in ("x", "y", "yaw", "v"))
    checkpoints = []
    for step, (row, side) in enumerate(zip(track[1:], sides), 1):
        need(
            int(side["step"]) == step
            and (row["scenario"], row["planner"])
            == (side["scenario"], side["planner"]),
            "Physical checkpoint join differs",
        )
        need(
            abs(pilot.finite(side["time_s"]) - step * pilot.DT) <= 1e-12
            and abs(pilot.finite(row["time_s"]) - step * pilot.DT) <= 5.1e-7,
            "Physical checkpoint time differs",
        )
        need(
            pilot.finite(side["measured_speed"]) == before[3],
            "Measured speed is not the pre-update physical speed",
        )
        checkpoints.append(before)
        after = tuple(pilot.finite(side["true_" + k]) for k in ("x", "y", "yaw", "v"))
        acceleration, steering = (
            pilot.finite(side[k]) for k in ("applied_acceleration", "applied_steering")
        )
        expected = (
            before[0] + before[3] * math.cos(before[2]) * pilot.DT,
            before[1] + before[3] * math.sin(before[2]) * pilot.DT,
            angular(before[2] + before[3] / 0.4 * math.tan(steering) * pilot.DT),
            before[3] + acceleration * pilot.DT,
        )
        for i, k in enumerate(("x", "y", "yaw", "v")):
            need(
                abs(after[i] - pilot.finite(row[k])) <= 5.1e-10,
                "Post-update truth/trajectory rounding differs",
            )
            error = (
                angular(after[i] - expected[i])
                if k == "yaw"
                else after[i] - expected[i]
            )
            need(abs(error) <= 1e-12, f"Post-update physical dynamics differ: {k}")
        before = after
    return checkpoints


def collect_windows(raw, catalogue, pins):
    expected = {r["case_id"]: r for r in catalogue}
    need(
        len(expected) == len(catalogue) == pins["expected_cases"],
        "Accuracy case catalogue differs",
    )
    checks, windows, seen = [], [], set()
    for tau, mode, arm in itertools.product(
        pins["noise_correlation_times"], pins["modes"], pins["arms"]
    ):
        directory = raw / f"tau{tau:g}/{arm}-{mode}-{pins['seed']}"
        with ExitStack() as stack:
            streams = [
                stack.enter_context((directory / name).open(newline=""))
                for name in (
                    "trajectories.csv",
                    "speed-probes.csv",
                    "guard-preview-inputs.csv",
                    "handoff-inputs.csv",
                )
            ]
            for groups in itertools.zip_longest(*(pilot.grouped(s) for s in streams)):
                need(
                    all(g is not None for g in groups)
                    and len({g[0] for g in groups}) == 1,
                    "Accuracy case grouping differs",
                )
                key = groups[0][0]
                meta = dict(
                    tau=tau,
                    mode=mode,
                    arm=arm,
                    seed=pins["seed"],
                    scenario=key[0],
                    planner=key[1],
                )
                ident = previous.live.case_id(meta)
                need(
                    ident in expected and ident not in seen,
                    "Accuracy case missing/repeated",
                )
                seen.add(ident)
                case = expected[ident]
                data = [g[1] for g in groups]
                local = previous.select_windows(case, *data, pins["horizon"])
                truth = physical_checkpoints(data[0], data[1])
                for window in local:
                    for point in window["points"]:
                        point["physical_pre"] = truth[point["step"] - 1]
                windows.extend(local)
                check = dict(
                    **{k: case[k] for k in META}, **dict.fromkeys(VERIFY_COUNTS, 0)
                )
                check.update(
                    physical_steps=len(truth),
                    pre_speed_matches=len(truth),
                    post_state_matches=len(truth),
                    window_points=sum(len(w["points"]) for w in local),
                )
                checks.append(check)
        print(
            f"Aligned every physical checkpoint tau={tau:g} mode={mode} arm={arm}",
            flush=True,
        )
    need(
        seen == set(expected)
        and sum(c["physical_steps"] for c in checks) == pins["expected_decisions"]
        and len(windows) == pins["expected_resumes"]
        and sum(len(w["points"]) for w in windows) == pins["expected_window_points"],
        "Incomplete physical/window population",
    )
    for i, window in enumerate(windows, 1):
        window["transition_id"] = i
    return windows, checks


def load_previous(directory, expected_queries, expected_points):
    rows = pilot.read(directory / "queries.csv", previous.QUERY_FIELDS)
    need(len(rows) == expected_queries, "Previous replay query count differs")
    groups = {}
    for row in rows:
        key = int(row["transition_id"]), int(row["lag"])
        group = groups.setdefault(key, {})
        need(row["variant"] not in group, "Duplicate previous variant")
        group[row["variant"]] = row
    need(len(groups) == expected_points, "Previous replay point count differs")
    for ident, ((tid, lag), group) in enumerate(groups.items(), 1):
        need(
            tuple(group) == (("recorded",) if lag == 0 else previous.native.VARIANTS),
            "Previous replay variant order differs",
        )
        need(
            all(r["id"] == str(ident) for r in group.values()),
            "Previous replay point sequence differs",
        )
    return groups


def sensor_poses(item, carry, eligible):
    """No physical truth is read in any sensor-only pose or selection."""
    guard, side = item["guard"], item["side"]
    actual = tuple(pilot.finite(guard["selected_" + k]) for k in ("x", "y", "yaw"))
    need(
        pilot.finite(guard["base_yaw"]) == actual[2]
        and pilot.finite(guard["base_v"]) == pilot.finite(guard["selected_v"]),
        "Base changed fixed yaw/speed",
    )
    carried = (carry[0], carry[1], actual[2])
    return dict(
        recorded=actual,
        carried_diagnostic_xy=carried,
        gated_carried_xy=carried if eligible else actual,
        base_estimate_xy=(
            pilot.finite(guard["base_x"]),
            pilot.finite(guard["base_y"]),
            actual[2],
        ),
        current_recovery_xy=(
            pilot.finite(side["mean_x"]),
            pilot.finite(side["mean_y"]),
            actual[2],
        ),
    )


def evaluation_poses(sensor, physical_pre):
    return dict(
        sensor,
        truth_xy_eval=(*physical_pre[:2], sensor["recorded"][2]),
        truth_pose_eval=physical_pre[:3],
    )


def prepare_points(windows, groups, checks):
    checked = {c["case_id"]: c for c in checks}
    points = []
    seen = set()
    for window in windows:
        case = window["case"]
        tid = window["transition_id"]
        carry = tuple(
            pilot.finite(window["points"][0]["side"]["mean_" + k])
            for k in ("x", "y", "yaw")
        )
        eligible = True
        for lag, item in enumerate(window["points"]):
            key = tid, lag
            need(
                key in groups and key not in seen,
                "Missing/repeated previous window point",
            )
            seen.add(key)
            retained = groups[key]
            old = retained["recorded"]
            need(
                all(old[k] == str(case[k]) for k in META)
                and old["status"] == case["status"]
                and int(old["step"]) == item["step"]
                and int(old["resume_step"]) == window["resume_step"],
                "Previous replay metadata join differs",
            )
            valid_now = not previous.invalid_reasons(item)
            if lag:
                carry = previous.transported(carry, window["points"][lag - 1]["side"])
                eligible = eligible and valid_now
                checked[case["case_id"]]["transport_matches"] += 1
            need(
                pilot.truth(old["valid_prefix"]) == eligible
                and pilot.truth(old["valid_now"]) == valid_now,
                "Previous eligibility differs",
            )
            for k, value in zip(("carry_x", "carry_y", "carry_yaw"), carry):
                previous.near(old[k], value, "Retained carried point differs")
            sensor = sensor_poses(
                item,
                tuple(pilot.finite(old["carry_" + k]) for k in ("x", "y", "yaw")),
                eligible,
            )
            for variant, r in retained.items():
                need(
                    tuple(pilot.finite(r[k]) for k in ("x", "y", "yaw"))
                    == sensor[variant],
                    "Retained sensor pose differs",
                )
                need(
                    pilot.finite(r["speed"])
                    == pilot.finite(item["side"]["measured_speed"])
                    and pilot.finite(r["target"])
                    == pilot.finite(item["side"]["nominal_target"])
                    and pilot.finite(r["steering"])
                    == pilot.finite(item["side"]["steering"]),
                    "Retained nominal input differs",
                )
            if lag == 0:
                need(
                    sensor["recorded"] == sensor["carried_diagnostic_xy"],
                    "Release mean differs from recorded release",
                )
            need(
                item["physical_pre"][3] == pilot.finite(item["side"]["measured_speed"]),
                "Evaluation physical speed differs",
            )
            ready = not pilot.truth(
                item["previous_row"]["guard_active"]
            ) and not pilot.truth(item["guard"]["joint_pending"])
            if ready:
                need(
                    previous.previous.threshold(old)["would_enter"]
                    == pilot.truth(item["row"]["guard_triggered"]),
                    "Retained actual guard entry differs",
                )
            checked[case["case_id"]]["prefix_matches"] += 1
            points.append(
                dict(
                    point_id=len(points) + 1,
                    window=window,
                    item=item,
                    lag=lag,
                    eligible=eligible,
                    ready=ready,
                    retained=retained,
                    poses=evaluation_poses(sensor, item["physical_pre"]),
                )
            )
    need(seen == set(groups), "Unused retained window points")
    return points


def write_packet(work, points):
    ident = 0
    packet = work / "queries.tsv"
    with packet.open("x", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=native.INPUT_FIELDS, delimiter="\t", lineterminator="\n"
        )
        writer.writeheader()
        for point in points:
            point["query_ids"] = {}
            for variant, pose in point["poses"].items():
                ident += 1
                point["query_ids"][variant] = ident
                record = previous.previous.census_input(
                    dict(point["item"], global_id=ident),
                    point["window"]["case"]["scenario"],
                )
                record.update(preview_x=pose[0], preview_y=pose[1], preview_yaw=pose[2])
                writer.writerow(record)
    return packet, ident


def compare_errors(value, baseline):
    return (
        "closer"
        if baseline - value > TIE
        else "farther" if value - baseline > TIE else "tied"
    )


def disagreement(geometry, xy, pose, ready):
    g, a, b = (previous.previous.threshold(r) for r in (geometry, xy, pose))
    result = dict(
        entry_only_xy=g["would_enter"] and not a["would_enter"],
        entry_missed_xy=not g["would_enter"] and a["would_enter"],
        entry_only_pose=g["would_enter"] and not b["would_enter"],
        entry_missed_pose=not g["would_enter"] and b["would_enter"],
        release_only_pose=g["release_clear"] and not b["release_clear"],
        release_missed_pose=not g["release_clear"] and b["release_clear"],
    )
    result.update({"ready_" + k: ready and result[k] for k in DISAGREEMENTS[:4]})
    return result


def consume_points(points, trace, checks):
    checked = {c["case_id"]: c for c in checks}
    samples = []
    for point in points:
        window, item = point["window"], point["item"]
        case = window["case"]
        geometries = {}
        for variant in VARIANTS:
            row = next(trace, None)
            need(
                row is not None and row["id"] == str(point["query_ids"][variant]),
                "Native accuracy query sequence differs",
            )
            if variant in point["retained"]:
                old = point["retained"][variant]
                need(
                    all(row[k] == old[k] for k in native.TRACE_FIELDS if k != "id"),
                    "Prior native variant geometry differs",
                )
                checked[case["case_id"]]["prior_variant_matches"] += 1
            geometries[variant] = row
        point["geometries"] = geometries
        physical = item["physical_pre"]
        actual_error = math.hypot(
            point["poses"]["recorded"][0] - physical[0],
            point["poses"]["recorded"][1] - physical[1],
        )
        point["errors"] = {}
        selected_source = (
            "handoff"
            if pilot.truth(item["handoff"]["used"])
            else (
                "joint_mean"
                if pilot.truth(item["guard"]["used"])
                else (
                    "epoch_fusion"
                    if pilot.truth(item["row"]["epoch_fusion_available"])
                    else "motion_filter"
                )
            )
        )
        for variant, pose in point["poses"].items():
            geometry = geometries[variant]
            ex, ey = pose[0] - physical[0], pose[1] - physical[1]
            error = math.hypot(ex, ey)
            point["errors"][variant] = error
            samples.append(
                dict(
                    query_id=point["query_ids"][variant],
                    point_id=point["point_id"],
                    transition_id=window["transition_id"],
                    **{k: case[k] for k in META},
                    status=case["status"],
                    resume_step=window["resume_step"],
                    step=item["step"],
                    lag=point["lag"],
                    variant=variant,
                    eligible=point["eligible"],
                    entry_ready=point["ready"],
                    actual_trigger=pilot.truth(item["row"]["guard_triggered"]),
                    selected_source=selected_source,
                    recovery_source=(
                        "raw_fallback"
                        if int(item["side"]["count"]) == 0
                        else "current_window"
                    ),
                    count=item["side"]["count"],
                    observation_age=item["side"]["observation_age"],
                    **dict(zip(("true_x", "true_y", "true_yaw", "true_v"), physical)),
                    **dict(zip(("x", "y", "yaw"), pose)),
                    speed=item["side"]["measured_speed"],
                    target=item["side"]["nominal_target"],
                    steering=item["side"]["steering"],
                    **{k: v for k, v in geometry.items() if k != "id"},
                    error_x_m=ex,
                    error_y_m=ey,
                    position_error_m=error,
                    abs_yaw_error_rad=abs(angular(pose[2] - physical[2])),
                    entry_error_xy_m=pilot.finite(geometry["entry"])
                    - pilot.finite(geometries["truth_xy_eval"]["entry"]),
                    entry_error_pose_m=pilot.finite(geometry["entry"])
                    - pilot.finite(geometries["truth_pose_eval"]["entry"]),
                    comparison=compare_errors(error, actual_error),
                    **disagreement(
                        geometry,
                        geometries["truth_xy_eval"],
                        geometries["truth_pose_eval"],
                        point["ready"],
                    ),
                )
            )
    need(next(trace, None) is None, "Extra native accuracy rows")
    return samples


def metrics(rows):
    eligible = [r for r in rows if r["eligible"]]
    result = dict(
        queries=len(rows),
        eligible_queries=len(eligible),
        ready_queries=sum(r["entry_ready"] for r in rows),
        zero_target_queries=sum(pilot.finite(r["target"]) == 0 for r in rows),
        **{
            k: sum(r["comparison"] == k for r in rows)
            for k in ("closer", "farther", "tied")
        },
        **{
            "eligible_" + k: sum(r["comparison"] == k for r in eligible)
            for k in ("closer", "farther", "tied")
        },
        **{k: sum(r[k] for r in rows) for k in DISAGREEMENTS},
    )
    result.update(dict.fromkeys(ERROR_FIELDS, ""))
    if rows:
        mean = lambda key: math.fsum(r[key] for r in rows) / len(rows)
        result.update(
            mean_position_error_m=mean("position_error_m"),
            rmse_position_error_m=math.sqrt(
                math.fsum(r["position_error_m"] ** 2 for r in rows) / len(rows)
            ),
            max_position_error_m=max(r["position_error_m"] for r in rows),
            mean_signed_x_error_m=mean("error_x_m"),
            mean_signed_y_error_m=mean("error_y_m"),
            mean_abs_yaw_error_rad=mean("abs_yaw_error_rad"),
            mean_entry_error_xy_m=mean("entry_error_xy_m"),
            mean_entry_error_pose_m=mean("entry_error_pose_m"),
        )
    if eligible:
        result.update(
            eligible_mean_position_error_m=math.fsum(
                r["position_error_m"] for r in eligible
            )
            / len(eligible),
            eligible_rmse_position_error_m=math.sqrt(
                math.fsum(r["position_error_m"] ** 2 for r in eligible) / len(eligible)
            ),
        )
    return result


def in_scope(row, scope):
    lag = row["lag"]
    return (
        lag == int(scope[-1])
        if scope.startswith("lag")
        else (
            (0 < lag <= int(scope[6:]))
            if scope.startswith("within")
            else 0 < lag <= 50 and row["actual_trigger"]
        )
    )


def tables(catalogue, samples):
    index = defaultdict(list)
    for row in samples:
        index[row["case_id"], row["variant"]].append(row)
    cases, pairs, summary = [], [], []
    paired = defaultdict(dict)
    for case in catalogue:
        for variant in VARIANTS:
            record = dict(
                **{k: case[k] for k in META},
                status=case["status"],
                variant=variant,
                resumes=int(case["resume_count"]),
                **metrics(index[case["case_id"], variant]),
            )
            cases.append(record)
            key = (*[case[k] for k in previous.PAIR_KEYS], variant)
            need(case["arm"] not in paired[key], "Duplicate accuracy case arm")
            paired[key][case["arm"]] = record
    for key, arms in paired.items():
        need(
            set(arms) == {"mean_preview", "one_step_handoff"},
            "Incomplete accuracy pair",
        )
        record = dict(zip((*previous.PAIR_KEYS, "variant"), key))
        for prefix, arm in (
            ("reference", "mean_preview"),
            ("candidate", "one_step_handoff"),
        ):
            record.update(
                {
                    prefix + "_" + k: arms[arm][k]
                    for k in ("status", "resumes", *METRICS)
                }
            )
        pairs.append(record)
    for noise, mode, arm, outcome, scope, variant in itertools.product(
        ("all", "white", "correlated"),
        ("all", "none", "random", "blackout"),
        ("mean_preview", "one_step_handoff"),
        ("all", "held", "timeout"),
        SCOPES,
        VARIANTS,
    ):
        selected = [
            c
            for c in catalogue
            if c["arm"] == arm
            and (noise == "all" or (float(c["tau"]) == 0) == (noise == "white"))
            and (mode == "all" or c["mode"] == mode)
            and (outcome == "all" or c["status"] == outcome)
        ]
        rows = [
            r
            for c in selected
            for r in index[c["case_id"], variant]
            if in_scope(r, scope)
        ]
        summary.append(
            dict(
                noise_selector=noise,
                mode_selector=mode,
                arm=arm,
                outcome_selector=outcome,
                scope=scope,
                variant=variant,
                cases=len(selected),
                resumes=sum(int(c["resume_count"]) for c in selected),
                **metrics(rows),
            )
        )
    return cases, pairs, summary


def resume_rows(points):
    groups = defaultdict(list)
    for point in points:
        groups[point["window"]["transition_id"]].append(point)
    result = []
    for tid, rows in groups.items():
        window = rows[0]["window"]
        case = window["case"]
        invalid = next((p["lag"] for p in rows if p["lag"] and not p["eligible"]), "")
        trigger = next(
            (
                p["lag"]
                for p in rows
                if p["lag"] and pilot.truth(p["item"]["row"]["guard_triggered"])
            ),
            "",
        )
        record = dict(
            transition_id=tid,
            **{k: case[k] for k in META},
            status=case["status"],
            resume_step=window["resume_step"],
            followup_steps=len(rows) - 1,
            first_invalid_lag=invalid,
            first_trigger_lag=trigger,
        )
        for lag in (0, 1, 2):
            p = rows[lag] if lag < len(rows) else None
            for prefix, variant in (
                ("recorded", "recorded"),
                ("carried", "carried_diagnostic_xy"),
                ("base", "base_estimate_xy"),
                ("current_recovery", "current_recovery_xy"),
            ):
                record[f"lag{lag}_{prefix}_error_m"] = p["errors"][variant] if p else ""
        p = rows[2] if len(rows) > 2 else None
        record.update(
            lag2_eligible=p["eligible"] if p else "",
            lag2_actual_trigger=(
                pilot.truth(p["item"]["row"]["guard_triggered"]) if p else ""
            ),
        )
        for prefix, variant in (
            ("recorded", "recorded"),
            ("carried", "carried_diagnostic_xy"),
            ("truth_xy", "truth_xy_eval"),
            ("truth_pose", "truth_pose_eval"),
        ):
            record["lag2_" + prefix + "_enter"] = (
                previous.previous.threshold(p["geometries"][variant])["would_enter"]
                if p
                else ""
            )
        record["lag2_carried_comparison"] = (
            compare_errors(
                p["errors"]["carried_diagnostic_xy"], p["errors"]["recorded"]
            )
            if p
            else ""
        )
        result.append(record)
    return result


def write(path, rows, fields):
    pilot.write(
        path,
        (
            {k: str(v).lower() if isinstance(v, bool) else v for k, v in row.items()}
            for row in rows
        ),
        fields,
    )


def evaluate(windows, catalogue, groups, checks, output, map_path):
    points = prepare_points(windows, groups, checks)
    output.mkdir(parents=True)
    work = output / "native"
    compiler = native.compile_native(work)
    packet, queries = write_packet(work, points)
    evaluation = native.evaluate(work, packet, map_path)
    samples = consume_points(
        points, iter(pilot.read(work / "trace.csv", native.TRACE_FIELDS)), checks
    )
    cases, pairs, summary = tables(catalogue, samples)
    resumes = resume_rows(points)
    need(
        queries == len(samples) == len(points) * len(VARIANTS),
        "Accuracy query conservation differs",
    )
    for name, rows, fields in (
        ("cases.csv", cases, CASE_FIELDS),
        ("pairs.csv", pairs, PAIR_FIELDS),
        ("summary.csv", summary, SUMMARY_FIELDS),
        ("verification.csv", checks, VERIFY_FIELDS),
        ("resumes.csv", resumes, RESUME_FIELDS),
        ("samples.csv", samples, SAMPLE_FIELDS),
    ):
        write(output / name, rows, fields)
    counts = dict(
        cases=len(catalogue),
        case_variant_rows=len(cases),
        pairs=len(pairs) // len(VARIANTS),
        pair_variant_rows=len(pairs),
        resumes=len(resumes),
        native_queries=len(samples),
        summary_cells=len(summary),
        **{k: sum(c[k] for c in checks) for k in VERIFY_COUNTS},
    )
    return counts, compiler, evaluation


def collect(raw, live_one, live_two, replay_one, replay_two, output):
    pins = preflight()
    raw, live_one, live_two, replay_one, replay_two = (
        Path(p).resolve() for p in (raw, live_one, live_two, replay_one, replay_two)
    )
    output = build.fresh(output)
    need(
        not any(
            output.is_relative_to(p)
            for p in (raw, live_one, live_two, replay_one, replay_two)
        ),
        "Accuracy output must be outside retained inputs",
    )
    need(
        live_one != live_two and replay_one != replay_two, "Bind distinct previous runs"
    )
    need(
        not subprocess.check_output(
            ["git", "status", "--porcelain=v1"], cwd=build.REPO, text=True
        ).strip(),
        "Commit accuracy implementation before evaluation",
    )
    live = build.REPO / pins["live_study"]
    old = build.REPO / pins["previous_study"]
    raw_manifest = json.loads((live / "raw-file-manifest.json").read_text())
    live_manifest = json.loads((live / "repeat-verification.json").read_text())
    replay_manifest = json.loads((old / "repeat-verification.json").read_text())
    previous.verify_manifest(raw, raw_manifest["sha256"])
    for directory, manifest in (
        (live_one, live_manifest),
        (live_two, live_manifest),
        (replay_one, replay_manifest),
        (replay_two, replay_manifest),
    ):
        previous.verify_manifest(directory, manifest["sha256"])
    need(
        build.sha(raw / "provenance.json") == live_manifest["raw_provenance_sha256"],
        "Accuracy raw provenance differs",
    )
    catalogue = pilot.read(live / "cases.csv")
    windows, checks = collect_windows(raw, catalogue, pins)
    groups = load_previous(
        replay_one, pins["expected_previous_queries"], pins["expected_window_points"]
    )
    try:
        counts, compiler, evaluation = evaluate(
            windows, catalogue, groups, checks, output, build.FIXTURES / "obstacles.csv"
        )
        need(
            counts["native_queries"] == pins["expected_new_queries"]
            and counts["prior_variant_matches"] == pins["expected_previous_queries"],
            "Accuracy replay denominator differs",
        )
        sources = dict(
            method="All-case pre-update physical checkpoint/dynamics alignment; seven fixed-input pose evaluations with truth evaluation-only; prior full contact/hold/estimator audits bound by hashes",
            implementation_commit=subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=build.REPO, text=True
            ).strip(),
            script_sha256={name: build.sha(build.ROOT / name) for name in SCRIPTS},
            protocol_sha256=pins["protocol_sha256"],
            freeze_sha256=build.sha(STUDY / "freeze-pins.json"),
            input_sha256=pins["input_sha256"],
            raw_provenance_sha256=live_manifest["raw_provenance_sha256"],
            raw_manifest_sha256=build.sha(live / "raw-file-manifest.json"),
            live_audit_manifest_sha256=build.sha(live / "repeat-verification.json"),
            previous_replay_manifest_sha256=build.sha(old / "repeat-verification.json"),
            compiler=compiler,
            evaluation=evaluation,
            counts=counts,
            output_sha256={
                str(p.relative_to(output)): build.sha(p)
                for p in sorted(output.rglob("*"))
                if p.is_file()
            },
        )
        (output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    except Exception as exc:
        if output.exists():
            (output / "failure.log").write_text(repr(exc) + "\n")
        raise
    print(json.dumps(counts), flush=True)
    return sources


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("raw", "live-one", "live-two", "replay-one", "replay-two", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    args = parser.parse_args()
    collect(
        args.raw,
        args.live_one,
        args.live_two,
        args.replay_one,
        args.replay_two,
        args.output,
    )


if __name__ == "__main__":
    main()
