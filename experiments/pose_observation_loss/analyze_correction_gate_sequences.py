#!/usr/bin/env python3
"""Audit recorded correction gate conjunctions, interruptions and release timers."""

import argparse
from collections import defaultdict
import csv
from itertools import product
import json
import math
from pathlib import Path

from analyze_commanded_recovery_arc import flag
from analyze_correction_centre_transitions import (
    ActionWindow,
    HORIZON,
    KINDS,
    gate_context,
)
from analyze_epoch_divergence import REQUIREMENTS
from analyze_epoch_position_components import file_digest
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import DT, KEY
from analyze_terminal_arc_live import COHORTS, SIMULATOR
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/correction-centre-transitions"
PROTOCOL = ROOT / "results/correction-gate-sequences/protocol.md"
PINNED = {
    "accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "events.csv": "819b026c74884681e8df9fb282998a19ade0d12f82e8d1ffa3ad1a3b5ce1df6f",
    "snapshots.csv": "9d25ee780dcb7695efee4c586509f8fe9c65edac9bb00b0cc5e4d16f2b8b5fcf",
    "sources.json": "e3dbe3ea3861f520035449371d2740bb12d5304df4dc09101589d2dc31fd71c5",
}
ROUTES = ["joint", "independent", "inactive"]
BASE = ["stationary", "fresh", "samples"]
EXTRA = [k for k in REQUIREMENTS if k not in BASE]
STAGES = [
    "censored",
    "independent_start",
    "inactive_start",
    "joint_resumed",
    "joint_no_intervals",
    "joint_no_base",
    "joint_no_full",
    "joint_short_run",
    "joint_long_no_resume",
]


def check_value(actual, expected):
    if isinstance(actual, bool):
        assert actual == flag(expected), (actual, expected)
    elif isinstance(actual, (float, int)):
        assert abs(actual - float(expected)) < 1e-8, (actual, expected)
    else:
        assert actual == expected, (actual, expected)


def failure_mask(context):
    values = [context[f"ready_{k}"] for k in REQUIREMENTS]
    assert all(v == "" or isinstance(v, bool) for v in values)
    expected = (
        set(REQUIREMENTS)
        if context["route"] == "joint"
        else (
            {"stationary", "stop", "terminal"}
            if context["route"] == "independent"
            else set()
        )
    )
    assert {k for k, v in zip(REQUIREMENTS, values) if v != ""} == expected
    mask = sum(1 << i for i, v in enumerate(values) if v is False)
    return mask, "+".join(k for i, k in enumerate(REQUIREMENTS) if mask & (1 << i))


def audit_timer(before, row, context):
    """Use pre-decision evidence; a release row logs the reset value zero."""
    route, ready = context["route"], context["active_ready"]
    errors = []

    def compare(value, expected):
        error = abs(float(value) - expected)
        assert error < 1e-8, (route, value, expected)
        errors.append(error)

    if route == "inactive":
        assert not context["resumed"] and not context["guard_released"]
        compare(row["recovery_evidence"], float(before["recovery_evidence"]))
        compare(row["guard_release_evidence"], 0)
        return dict(
            timer_before_s="",
            timer_input_s="",
            timer_proposed_s="",
            timer_after_s="",
            timer_threshold_s="",
            timer_action=False,
            timer_error_s=max(errors),
        )
    joint = route == "joint"
    field = "recovery_evidence" if joint else "guard_release_evidence"
    threshold = 1.0 if joint else 0.5
    previous = float(before[field])
    missing_entry = (
        joint
        and not flag(before["safety_paused"])
        and flag(row["correction"])
        and not flag(row["correction_received"])
    )
    initial = 0.0 if missing_entry else previous
    proposed = initial + DT if ready else 0.0
    action = proposed + 1e-9 >= threshold
    assert action == context["resumed" if joint else "guard_released"]
    compare(row[field], 0.0 if action else proposed)
    if joint:
        compare(row["guard_release_evidence"], 0)
        assert flag(row["safety_paused"]) != action
    else:
        assert not context["resumed"]
        compare(row["recovery_evidence"], float(before["recovery_evidence"]))
    return dict(
        timer_before_s=previous,
        timer_input_s=initial,
        timer_proposed_s=proposed,
        timer_after_s=float(row[field]),
        timer_threshold_s=threshold,
        timer_action=action,
        timer_error_s=max(errors),
    )


def ready_runs(rows):
    """Observed joint runs; a horizon is not a failed requirement."""
    result, current = [], []
    for i, row in enumerate(rows):
        ready = row["route"] == "joint" and row["active_ready"]
        if ready:
            current.append(row)
        next_row = rows[i + 1] if i + 1 < len(rows) else None
        next_ready = (
            next_row is not None
            and next_row["route"] == "joint"
            and next_row["active_ready"]
        )
        if current and not next_ready:
            end = (
                "joint_resume"
                if current[-1]["resumed"]
                else (
                    "horizon"
                    if next_row is None and row["offset_steps"] == HORIZON
                    else (
                        "recorded_end"
                        if next_row is None
                        else (
                            "route_change"
                            if next_row["route"] != "joint"
                            else "requirement_failed"
                        )
                    )
                )
            )
            result.append((current, end))
            current = []
    return result


def partition(event, fields):
    if flag(event["post_window_censored"]):
        return "censored"
    if event["event_route"] != "joint":
        return event["event_route"] + "_start"
    if int(event["post_resumes"]):
        return "joint_resumed"
    if not fields["joint_intervals"]:
        return "joint_no_intervals"
    if not fields["joint_base_intervals"]:
        return "joint_no_base"
    if not fields["joint_full_intervals"]:
        return "joint_no_full"
    return (
        "joint_short_run"
        if fields["joint_max_ready_steps"] < 50
        else "joint_long_no_resume"
    )


def event_fields(event, rows):
    post = [r for r in rows if r["offset_steps"] > 0]
    joint = [r for r in post if r["route"] == "joint"]
    base = [r for r in joint if r["joint_base_usable"]]
    full = [r for r in joint if r["active_ready"]]
    runs = ready_runs(post)
    fields = dict(
        joint_intervals=len(joint),
        joint_base_intervals=len(base),
        joint_full_intervals=len(full),
        joint_first_base_delay_s=(
            round(base[0]["offset_steps"] * DT, 6) if base else ""
        ),
        joint_first_full_delay_s=(
            round(full[0]["offset_steps"] * DT, 6) if full else ""
        ),
        joint_max_ready_steps=max((len(r) for r, _ in runs), default=0),
        joint_ready_runs=len(runs),
        **{
            "joint_runs_" + reason: sum(end == reason for _, end in runs)
            for reason in [
                "joint_resume",
                "horizon",
                "recorded_end",
                "route_change",
                "requirement_failed",
            ]
        },
        **{
            f"joint_{k}_true_intervals": sum(r[f"ready_{k}"] for r in joint)
            for k in REQUIREMENTS
        },
        **{
            f"joint_ever_{k}": any(r[f"ready_{k}"] for r in joint) for k in REQUIREMENTS
        },
        **{
            f"base_failed_{k}_intervals": sum(not r[f"ready_{k}"] for r in base)
            for k in EXTRA
        },
    )
    fields["joint_all_individually_seen"] = bool(joint) and all(
        fields[f"joint_ever_{k}"] for k in REQUIREMENTS
    )
    fields["partition"] = partition(event, fields)
    return fields


def make_spans(event, rows, output):
    post = [r for r in rows if r["offset_steps"] > 0]
    groups = []
    for r in post:
        key = tuple(r[k] for k in ["route", "failed_mask", "resumed", "guard_released"])
        if groups and groups[-1][0] == key:
            groups[-1][1].append(r)
        else:
            groups.append((key, [r]))
    for i, (_, group) in enumerate(groups):
        first, last = group[0], group[-1]
        following = groups[i + 1][1][0] if i + 1 < len(groups) else None
        reason = (
            "joint_resume"
            if last["resumed"]
            else (
                "independent_release"
                if last["route"] == "independent" and last["guard_released"]
                else (
                    "horizon"
                    if following is None and last["offset_steps"] == HORIZON
                    else (
                        "recorded_end"
                        if following is None
                        else (
                            "route_change"
                            if following["route"] != last["route"]
                            else (
                                "requirements_changed"
                                if following["failed_mask"] != last["failed_mask"]
                                else "action_changed"
                            )
                        )
                    )
                )
            )
        )
        output.append(
            dict(
                span_id=len(output) + 1,
                event_id=event["event_id"],
                first_interval_id=first["interval_id"],
                last_interval_id=last["interval_id"],
                start_offset_steps=first["offset_steps"],
                end_offset_steps=last["offset_steps"],
                intervals=len(group),
                route=first["route"],
                failed_mask=first["failed_mask"],
                failed_requirements=first["failed_requirements"],
                active_ready=first["active_ready"],
                joint_base_usable=first["joint_base_usable"],
                resumed=first["resumed"],
                guard_released=first["guard_released"],
                first_timer_before_s=first["timer_before_s"],
                last_timer_after_s=last["timer_after_s"],
                continues_at_left_boundary=first["offset_steps"] == 1
                and first["timer_before_s"] != ""
                and first["timer_before_s"] > 0,
                end_reason=reason,
            )
        )


def summarize(events, intervals):
    by_event = {e["event_id"]: e for e in events}
    summary = []
    for cohort, variant, category, route in product(
        ["all", *COHORTS], ["epoch", "curved"], ["all", *KINDS], ["all", *ROUTES]
    ):
        rows = [
            e
            for e in events
            if e["variant"] == variant
            and (cohort == "all" or e["cohort"] == cohort)
            and (category == "all" or e["event_kind"] == category)
            and (route == "all" or e["event_route"] == route)
        ]
        counts = {stage: sum(e["partition"] == stage for e in rows) for stage in STAGES}
        assert sum(counts.values()) == len(rows)
        summary.append(
            dict(
                cohort=cohort,
                variant=variant,
                event_kind=category,
                event_route=route,
                events=len(rows),
                cases=len({tuple(e[k] for k in KEY) for e in rows}),
                **counts,
                all_individually_seen_no_full_events=sum(
                    e["joint_all_individually_seen"] and not e["joint_full_intervals"]
                    for e in rows
                ),
                **{
                    k: sum(e[k] for e in rows)
                    for k in [
                        "joint_intervals",
                        "joint_base_intervals",
                        "joint_full_intervals",
                        "joint_ready_runs",
                        "joint_runs_joint_resume",
                        "joint_runs_horizon",
                        "joint_runs_recorded_end",
                        "joint_runs_route_change",
                        "joint_runs_requirement_failed",
                    ]
                },
            )
        )
    patterns, denominators, requirement_groups = (
        defaultdict(list),
        defaultdict(int),
        defaultdict(list),
    )
    for r in intervals:
        if r["offset_steps"] <= 0:
            continue
        e = by_event[r["event_id"]]
        for cohort, category in product(["all", e["cohort"]], ["all", e["event_kind"]]):
            group = cohort, e["variant"], category
            for scope in [
                "all_post",
                *(["joint_base"] if r["joint_base_usable"] else []),
            ]:
                prefix = (*group, scope, r["route"])
                patterns[(*prefix, r["failed_mask"])].append(r)
                denominators[prefix] += 1
            if r["route"] == "joint":
                requirement_groups[(*group, "joint_all")].append(r)
                if r["joint_base_usable"]:
                    requirement_groups[(*group, "joint_base")].append(r)
    pattern_rows = []
    for key, rows in sorted(patterns.items()):
        event_ids = {r["event_id"] for r in rows}
        pattern_rows.append(
            dict(
                **dict(
                    zip(
                        [
                            "cohort",
                            "variant",
                            "event_kind",
                            "scope",
                            "route",
                            "failed_mask",
                        ],
                        key,
                    )
                ),
                failed_requirements=rows[0]["failed_requirements"],
                intervals=len(rows),
                route_scope_intervals=denominators[key[:-1]],
                events=len(event_ids),
                cases=len({tuple(by_event[e][k] for k in KEY) for e in event_ids}),
            )
        )
    requirements = []
    for cohort, variant, category, scope in product(
        ["all", *COHORTS],
        ["epoch", "curved"],
        ["all", *KINDS],
        ["joint_all", "joint_base"],
    ):
        rows = requirement_groups[cohort, variant, category, scope]
        for requirement in REQUIREMENTS if scope == "joint_all" else EXTRA:
            yes = [r for r in rows if r["ready_" + requirement]]
            no = [r for r in rows if not r["ready_" + requirement]]
            requirements.append(
                dict(
                    cohort=cohort,
                    variant=variant,
                    event_kind=category,
                    scope=scope,
                    requirement=requirement,
                    intervals=len(rows),
                    true_intervals=len(yes),
                    false_intervals=len(no),
                    events=len({r["event_id"] for r in rows}),
                    true_events=len({r["event_id"] for r in yes}),
                    false_events=len({r["event_id"] for r in no}),
                )
            )
    return summary, pattern_rows, requirements


def export_event(event, points, intervals, spans):
    step = round(float(event["event_time_s"]) / DT)
    window, rows = ActionWindow(step), []
    for offset in range(-1, min(HORIZON, 9000 - step) + 1):
        p = points[step + offset]
        row = dict(
            interval_id=len(intervals) + 1,
            event_id=event["event_id"],
            offset_steps=offset,
            **p,
        )
        rows.append(row)
        intervals.append(row)
        window.observe(
            step + offset,
            p,
            p["physical_travel"],
            p["scheduled"],
            p["precision_activation"],
        )
    for field, actual in window.record().items():
        check_value(actual, event[field])
    make_spans(event, rows, spans)
    return dict(event, **event_fields(event, rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for name, sha in PINNED.items():
        assert digest(PRIOR / name) == sha, name
    prior_source = json.loads((PRIOR / "sources.json").read_text())
    helpers = dict(prior_source["analysis"]["helpers_sha256"])
    helpers["analyze_correction_centre_transitions.py"] = prior_source["analysis"][
        "script_sha256"
    ]
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    assert digest(SIMULATOR) == prior_source["analysis"]["simulator_sha256"]
    assert digest(ROOT / "inputs/obstacles.csv") == prior_source["map_sha256"]
    accounting, old_events, snapshots = (
        read_rows(PRIOR / n) for n in ["accounting.csv", "events.csv", "snapshots.csv"]
    )
    assert (
        len(accounting) == 648 and len(old_events) == 2583 and len(snapshots) == 25830
    )
    events_by_case, expected = defaultdict(list), defaultdict(list)
    for e in old_events:
        events_by_case[tuple(e[k] for k in KEY)].append(e)
    assert len(events_by_case) == 368
    assert set(events_by_case) == {
        tuple(a[k] for k in KEY) for a in accounting if flag(a["selected"])
    }
    for s in snapshots:
        if flag(s["available"]):
            expected[tuple(s[k] for k in KEY), round(float(s["time_s"]) / DT)].append(s)
    new_events, intervals, spans = [], [], []
    scanned = checked = matched = 0
    max_timer_error = 0.0
    sources = dict(input_sha256=PINNED, cohorts={})
    for cohort in COHORTS:
        pinned = prior_source["cohorts"][cohort]
        run = (
            args.raw_root / cohort if args.raw_root else Path(pinned["path"])
        ).resolve()
        provenance = json.loads((run / "provenance.json").read_text())
        assert provenance == pinned["provenance"]
        files, hashes = defaultdict(list), {}
        for case in events_by_case:
            if case[0] == cohort:
                files[case[1:4]].append(case)
        cohort_checked = 0
        for (variant, mode, seed), cases in sorted(files.items()):
            name = f"{variant}-{mode}-{seed}/trajectories.csv"
            hashes[name] = file_digest(run / name)
            assert hashes[name] == pinned["raw_sha256"][name], name
            selected = {(case[4], case[5]): case for case in cases}
            needs, points, before, steps = {}, {}, {}, {}
            for case in cases:
                needs[case] = {
                    step + offset
                    for e in events_by_case[case]
                    for step in [round(float(e["event_time_s"]) / DT)]
                    for offset in range(-1, min(HORIZON, 9000 - step) + 1)
                }
                points[case] = {}
            with (run / name).open() as stream:
                reader = csv.DictReader(stream)
                assert len(reader.fieldnames) == 82
                for row in reader:
                    assert None not in row and None not in row.values()
                    case = selected.get((row["scenario"], row["planner"]))
                    if case is None:
                        continue
                    step = round(float(row["time_s"]) / DT)
                    if step == 0:
                        assert case not in before
                        before[case], steps[case] = row, 0
                        continue
                    previous = before[case]
                    assert step == steps[case] + 1
                    assert (
                        abs(float(row["measured_speed"]) - float(previous["v"])) < 1e-9
                    )
                    scanned += 1
                    if step in needs[case]:
                        context = gate_context(previous, row)
                        timer = audit_timer(previous, row, context)
                        mask, failed = failure_mask(context)
                        points[case][step] = dict(
                            time_s=row["time_s"],
                            **context,
                            failed_mask=mask,
                            failed_requirements=failed,
                            **timer,
                            physical_travel=math.dist(
                                tuple(float(previous[k]) for k in ["x", "y"]),
                                tuple(float(row[k]) for k in ["x", "y"]),
                            ),
                            scheduled=flag(row["correction"]),
                            precision_activation=flag(row["precision_activation"]),
                        )
                        for s in expected[case, step]:
                            for field, value in context.items():
                                check_value(value, s[field])
                            matched += 1
                        checked += 1
                        cohort_checked += 1
                        max_timer_error = max(max_timer_error, timer["timer_error_s"])
                    before[case], steps[case] = row, step
            for case in cases:
                assert steps[case] == 9000 and set(points[case]) == needs[case]
                for e in events_by_case[case]:
                    new_events.append(export_event(e, points[case], intervals, spans))
        assert hashes == pinned["raw_sha256"]
        sources["cohorts"][cohort] = dict(
            path=str(run),
            provenance=provenance,
            raw_sha256=hashes,
            checked_unique_decisions=cohort_checked,
        )
        print(
            f"{cohort}: {cohort_checked} distinct gate/timer decisions audited",
            flush=True,
        )
    assert (
        scanned == 3312000 and matched == prior_source["audit"]["available_snapshots"]
    )
    assert len(new_events) == len(old_events)
    assert len(intervals) == sum(int(e["post_intervals"]) + 2 for e in old_events)
    assert sum(s["intervals"] for s in spans) == sum(
        int(e["post_intervals"]) for e in old_events
    )
    summary, patterns, requirements = summarize(new_events, intervals)
    sources["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        protocol_sha256=digest(PROTOCOL),
        helpers_sha256=dict(sorted(helpers.items())),
        simulator_sha256=digest(SIMULATOR),
    )
    sources["schema"] = dict(
        dt=DT,
        requirement_bit_order=REQUIREMENTS,
        post_offsets=[1, HORIZON],
        stages=STAGES,
    )
    sources["inherited_validation"] = dict(
        source_sha256=PINNED["sources.json"],
        full_observation_reconstruction=True,
        physics_contact_timers=True,
        new_rollout=False,
    )
    sources["audit"] = dict(
        accounted_cases=len(accounting),
        cases=len(events_by_case),
        scanned_selected_intervals=scanned,
        checked_unique_decisions=checked,
        matched_prior_snapshots=matched,
        opportunities=len(new_events),
        exported_intervals=len(intervals),
        post_intervals=sum(int(e["post_intervals"]) for e in old_events),
        spans=len(spans),
        max_timer_error_s=max_timer_error,
        summary_rows=len(summary),
        pattern_rows=len(patterns),
        requirement_rows=len(requirements),
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", accounting),
        ("events", new_events),
        ("intervals", intervals),
        ("spans", spans),
        ("summary", summary),
        ("patterns", patterns),
        ("requirements", requirements),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(sources["audit"], indent=2))


if __name__ == "__main__":
    main()
