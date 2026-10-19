#!/usr/bin/env python3
"""Census both release timers using full native geometry at every pilot decision."""

import argparse
from collections import Counter
from contextlib import ExitStack
import csv
import itertools
import json
from pathlib import Path
import subprocess

import analyze_mean_probe_admission as admission
import analyze_mean_guard_preview as live
import analyze_mean_speed_probe as pilot
import build_mean_guard_preview as build
import native_guard_release_census as native

STUDY = build.ROOT / "results/mean-guard-release-census"
SCRIPTS = (
    "analyze_guard_release_census.py",
    "native_guard_release_census.py",
    "test_guard_release_census.py",
)
META = ("case_id", *pilot.CASE_META)
GATES = admission.GATES
COUNT_FIELDS = (
    "decisions",
    "joint_steps",
    "joint_eligible_steps",
    "joint_positive_steps",
    "joint_positive_blocked_steps",
    "joint_latched_zero",
    "joint_probe_zero",
    "joint_ready_steps",
    "joint_resumes",
    "joint_ready_runs",
    "joint_resets",
    "guard_update_steps",
    "guard_ready_steps",
    "guard_releases",
    "guard_triggers",
    "guard_joint_releases",
    "guard_ready_runs",
    "guard_resets",
    "guard_interruptions",
    "speed_used_steps",
    "guard_used_steps",
)
MAX_FIELDS = ("joint_max_ready_run_steps", "guard_max_ready_run_steps")
END_FIELDS = (
    "final_brake_mask",
    "final_paused",
    "final_guard",
    "final_latched",
    "final_hold_s",
    "final_joint_failed_gates",
    "final_guard_failed_gates",
)
CASE_FIELDS = (*META, "status", "held", *COUNT_FIELDS, *MAX_FIELDS, *END_FIELDS)
PATTERN_FIELDS = (*META, "phase", "failed_gates", "steps")
RUN_FIELDS = (
    *META,
    "phase",
    "run",
    "first_step",
    "last_step",
    "steps",
    "duration_s",
    "ending",
    "break_step",
    "failed_gates",
)
EVENT_FIELDS = (
    *META,
    "step",
    "time_s",
    "event",
    "cause",
    "timer_before_reset",
    "guard_used",
    "entry",
    "release",
    "release_margin",
)
SUMMARY_FIELDS = (
    "noise_selector",
    "mode_selector",
    "arm",
    "outcome_selector",
    "cases",
    *COUNT_FIELDS,
    *MAX_FIELDS,
    "endpoint_paused",
    "endpoint_guard",
    "endpoint_latched",
)
VERIFY_FIELDS = (
    *META,
    "geometry_decisions",
    "joint_timer_decisions",
    "guard_timer_decisions",
    "resumes",
    "paused_steps",
)
PAIR_KEYS = ("tau", "mode", "seed", "scenario", "planner")
PAIR_VALUES = ("status", *COUNT_FIELDS, *MAX_FIELDS, "final_brake_mask")
PAIR_FIELDS = (
    *PAIR_KEYS,
    *(arm + "_" + key for arm in ("reference", "candidate") for key in PAIR_VALUES),
)
need = build.require


def preflight():
    pins = json.loads((STUDY / "freeze-pins.json").read_text())
    need(
        build.sha(STUDY / "protocol.md") == pins["protocol_sha256"],
        "Census protocol changed",
    )
    for name, digest in pins["input_sha256"].items():
        need(build.sha(build.REPO / name) == digest, f"Changed census input: {name}")
    return pins


def failed(states):
    return ";".join(name for name, passed in states.items() if not passed)


def gates(row, side, geometry):
    for fresh, old in (
        ("recovery_clearance_9", "recovery_clearance"),
        ("preview_margin_9", "preview_margin"),
        ("release_margin_9", "guard_release_margin"),
        ("stop_9", "stop_preview_clearance_lower_bound"),
        ("terminal_9", "terminal_recovery_clearance"),
        ("entry_9", "preview_clearance_lower_bound"),
        ("release_9", "release_preview_clearance_lower_bound"),
        ("effective_position_sigma_9", "effective_position_sigma"),
    ):
        need(geometry[fresh] == row[old], f"Native geometry differs: {old}")
    need(
        pilot.finite(geometry["recovery_margin"]) == pilot.finite(side["margin"]),
        "Full recovery margin differs",
    )
    states = dict(
        fresh=pilot.finite(side["observation_age"]) <= 1.5,
        stationary=abs(pilot.finite(side["measured_speed"])) <= 0.02,
        samples=int(side["count"]) >= 10,
        clearance=pilot.finite(geometry["recovery_clearance"])
        > pilot.finite(geometry["recovery_margin"]),
        bounds=pilot.truth(geometry["recovery_bounds"]),
        nominal=pilot.finite(side["nominal_target"]) > 0,
        release=pilot.finite(geometry["release"])
        > pilot.finite(geometry["release_margin"]),
    )
    need(tuple(states) == GATES, "Joint gate order differs")
    return states


class ReleaseState:
    """The original ordered branches, with full values instead of rounded logs."""

    def __init__(self):
        self.active = False
        self.joint = self.guard = 0.0

    def step(self, pending, states, entry, margin):
        need(
            type(pending) is bool and all(type(v) is bool for v in states.values()),
            "Invalid release state",
        )
        triggered = released = resumed = guard_updated = False
        guard_amount = joint_amount = 0.0
        if not self.active and entry <= margin:
            self.active, triggered, self.guard = True, True, 0.0
        elif self.active and not pending:
            guard_updated = True
            guard_amount = (
                self.guard + 0.02 if states["stationary"] and states["release"] else 0.0
            )
            self.guard = guard_amount
            if self.guard + 1e-9 >= 0.5:
                self.active, self.guard, released = False, 0.0, True
        if pending:
            joint_amount = self.joint + 0.02 if all(states.values()) else 0.0
            self.joint, self.guard = joint_amount, 0.0
            if self.joint + 1e-9 >= 1.0:
                self.joint, resumed, released, self.active = (
                    0.0,
                    True,
                    self.active,
                    False,
                )
        return dict(
            triggered=triggered,
            released=released,
            resumed=resumed,
            active=self.active,
            paused=pending and not resumed,
            joint_timer=self.joint,
            guard_timer=self.guard,
            guard_updated=guard_updated,
            joint_amount=joint_amount,
            guard_amount=guard_amount,
        )


class Runs:
    def __init__(self, meta, phase, output):
        self.meta, self.phase, self.output = meta, phase, output
        self.active = None
        self.count = self.maximum = self.resets = self.interruptions = 0

    def close(self, ending, step="", mask=""):
        if self.active is None:
            return
        self.output.append(
            dict(
                **self.meta,
                phase=self.phase,
                **self.active,
                duration_s=self.active["steps"] * 0.02,
                ending=ending,
                break_step=step,
                failed_gates=mask,
            )
        )
        self.resets += ending == "failure"
        self.interruptions += ending in ("joint_pause", "inactive")
        self.active = None

    def step(self, step, enabled, ready, released, mask, interruption):
        if not enabled:
            self.close(interruption, step, mask)
        elif not ready:
            self.close("failure", step, mask)
        else:
            if self.active is None:
                self.count += 1
                self.active = dict(
                    run=self.count, first_step=step, last_step=step, steps=0
                )
            self.active["last_step"] = step
            self.active["steps"] += 1
            self.maximum = max(self.maximum, self.active["steps"])
            if released:
                self.close("admission", step)


def iter_cases(raw, provenance):
    expected_paths = {
        (r["scenario"], r["planner"]) for r in pilot.read(build.FIXTURES / "paths.csv")
    }
    for trial in provenance["trials"]:
        directory = raw / trial["directory"]
        with ExitStack() as stack:
            streams = [
                stack.enter_context((directory / name).open(newline=""))
                for name in (
                    "trajectories.csv",
                    "speed-probes.csv",
                    "guard-preview-inputs.csv",
                )
            ]
            seen = []
            for groups in itertools.zip_longest(
                *(pilot.grouped(stream) for stream in streams)
            ):
                need(
                    all(g is not None for g in groups)
                    and len({g[0] for g in groups}) == 1,
                    "Census case grouping differs",
                )
                key = groups[0][0]
                seen.append(key)
                meta = dict(
                    tau=trial["tau"],
                    arm=trial["arm"],
                    mode=trial["mode"],
                    seed=trial["seed"],
                    scenario=key[0],
                    planner=key[1],
                )
                meta = dict(case_id=live.case_id(meta), **meta)
                yield meta, *(g[1] for g in groups)
            need(
                len(seen) == len(set(seen)) == 9 and set(seen) == expected_paths,
                "Incomplete census trial",
            )


def extract(raw, provenance, work):
    count = 0
    packet = work / "queries.tsv"
    with packet.open("x", newline="") as stream:
        writer = csv.writer(stream, delimiter="\t", lineterminator="\n")
        writer.writerow(native.INPUT_FIELDS)
        for meta, track, sides, guards in iter_cases(raw, provenance):
            live.validate_guard(meta, track, sides, guards)
            for row, side, guard in zip(track[1:], sides, guards):
                count += 1
                writer.writerow(
                    (
                        count,
                        meta["scenario"],
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
                    )
                )
    return packet, count


def diagnose(
    meta, track, sides, guards, old, geometries, counter, patterns, runs, events
):
    need(len(track) == len(sides) + 1 == len(guards) + 1, "Incomplete census case")
    machine, counts = ReleaseState(), Counter({name: 0 for name in COUNT_FIELDS})
    joint_runs, guard_runs = Runs(meta, "joint", runs), Runs(meta, "guard_only", runs)
    paused_steps = pauses = 0
    for step, (before, row, side, guard) in enumerate(
        zip(track, track[1:], sides, guards), 1
    ):
        counter[0] += 1
        geometry = next(geometries, None)
        need(
            geometry is not None
            and tuple(geometry) == native.TRACE_FIELDS
            and geometry["id"] == str(counter[0]),
            "Missing/reordered geometry",
        )
        need(int(side["step"]) == int(guard["step"]) == step, "Census step differs")
        pilot.close(side["time_s"], row["time_s"])
        pilot.close(side["time_s"], step * 0.02)
        missing = pilot.truth(row["correction"]) and not pilot.truth(
            row["correction_received"]
        )
        pending = pilot.truth(side["paused"])
        need(
            pending == (pilot.truth(before["safety_paused"]) or missing),
            "Census pending state differs",
        )
        pauses += missing and not pilot.truth(before["safety_paused"])
        states = gates(row, side, geometry)
        mask = failed(states)
        guard_states = {k: states[k] for k in ("stationary", "release")}
        guard_mask = failed(guard_states)
        ready, guard_ready = all(states.values()), all(guard_states.values())
        result = machine.step(
            pending,
            states,
            pilot.finite(geometry["entry"]),
            pilot.finite(geometry["preview_margin"]),
        )
        for name, key in (
            ("triggered", "guard_triggered"),
            ("released", "guard_released"),
            ("active", "guard_active"),
            ("paused", "safety_paused"),
            ("resumed", "resumed"),
        ):
            need(
                result[name] == pilot.truth(row[key]),
                f"Native release flag differs: {key}, {meta['case_id']}, {step}",
            )
        pilot.close(result["joint_timer"], row["recovery_evidence"], 1e-8)
        pilot.close(result["guard_timer"], row["guard_release_evidence"], 1e-8)
        counts["decisions"] += 1
        counts["joint_steps"] += pending
        counts["joint_resumes"] += result["resumed"]
        counts["guard_triggers"] += result["triggered"]
        counts["guard_joint_releases"] += pending and result["released"]
        counts["guard_update_steps"] += result["guard_updated"]
        counts["guard_releases"] += result["guard_updated"] and result["released"]
        counts["guard_ready_steps"] += result["guard_updated"] and guard_ready
        counts["joint_ready_steps"] += pending and ready
        counts["speed_used_steps"] += pilot.truth(side["used"])
        counts["guard_used_steps"] += pilot.truth(guard["used"])
        paused_steps += result["paused"]
        if pending:
            patterns[(*tuple(meta[k] for k in META), "joint_all", mask)] += 1
            eligible = all(states[k] for k in ("fresh", "stationary", "samples"))
            counts["joint_eligible_steps"] += eligible
            if eligible:
                patterns[(*tuple(meta[k] for k in META), "joint_eligible", mask)] += 1
                counts["joint_positive_steps"] += states["nominal"]
                counts["joint_positive_blocked_steps"] += (
                    states["nominal"] and not ready
                )
                if not states["nominal"]:
                    latched = pilot.finite(side["uncapped_target"]) == 0
                    need(
                        (latched and pilot.truth(row["stop_latched"]))
                        or (not latched and pilot.finite(side["free_distance"]) == 0),
                        "Zero target origin differs",
                    )
                    counts["joint_latched_zero" if latched else "joint_probe_zero"] += 1
        if result["guard_updated"]:
            patterns[(*tuple(meta[k] for k in META), "guard_only", guard_mask)] += 1
        joint_runs.step(step, pending, ready, result["resumed"], mask, "not_pending")
        guard_runs.step(
            step,
            result["guard_updated"],
            guard_ready,
            result["guard_updated"] and result["released"],
            guard_mask,
            "joint_pause" if pending else "inactive",
        )
        for flag, name, cause, amount in (
            (result["triggered"], "guard_trigger", "entry", 0),
            (
                result["released"],
                "guard_release",
                "joint" if pending else "guard_only",
                result["joint_amount"] if pending else result["guard_amount"],
            ),
            (result["resumed"], "joint_resume", "joint", result["joint_amount"]),
        ):
            if flag:
                events.append(
                    dict(
                        **meta,
                        step=step,
                        time_s=row["time_s"],
                        event=name,
                        cause=cause,
                        timer_before_reset=amount,
                        guard_used=guard["used"],
                        **{
                            k: geometry[k]
                            for k in ("entry", "release", "release_margin")
                        },
                    )
                )
    joint_runs.close("packet_endpoint")
    guard_runs.close("packet_endpoint")
    counts.update(
        joint_ready_runs=joint_runs.count,
        joint_resets=joint_runs.resets,
        guard_ready_runs=guard_runs.count,
        guard_resets=guard_runs.resets,
        guard_interruptions=guard_runs.interruptions,
    )
    need(
        counts["decisions"] == int(old["decisions"])
        and counts["speed_used_steps"] == int(old["speed_used_steps"])
        and counts["guard_used_steps"] == int(old["guard_used_steps"]),
        "Frozen census case counts differ",
    )
    need(
        counts["joint_resumes"] == int(old["resume_count"])
        and pauses == int(old["pause_count"]),
        "Native pause/resume totals differ",
    )
    pilot.close(paused_steps * 0.02, old["paused_time_s"], 1e-7)
    need(
        counts["joint_steps"] == paused_steps + counts["joint_resumes"],
        "Pending/paused accounting differs",
    )
    need(
        counts["joint_eligible_steps"]
        == counts["joint_positive_steps"]
        + counts["joint_latched_zero"]
        + counts["joint_probe_zero"],
        "Eligible command partition differs",
    )
    need(
        counts["joint_positive_steps"]
        == counts["joint_ready_steps"] + counts["joint_positive_blocked_steps"],
        "Ready/blocked partition differs",
    )
    last = track[-1]
    case = dict(
        **meta,
        status=old["status"],
        held=old["held"],
        **counts,
        joint_max_ready_run_steps=joint_runs.maximum,
        guard_max_ready_run_steps=guard_runs.maximum,
        final_brake_mask=";".join(
            k
            for k in ("stop_latched", "safety_paused", "guard_active", "speed_braking")
            if pilot.truth(last[k])
        ),
        final_paused=last["safety_paused"],
        final_guard=last["guard_active"],
        final_latched=last["stop_latched"],
        final_hold_s=last["hold_time_s"],
        final_joint_failed_gates=mask if pilot.truth(last["safety_paused"]) else "",
        final_guard_failed_gates=(
            guard_mask if pilot.truth(last["guard_active"]) else ""
        ),
    )
    checked = dict(
        **meta,
        geometry_decisions=len(sides),
        joint_timer_decisions=counts["joint_steps"],
        guard_timer_decisions=counts["guard_update_steps"],
        resumes=counts["joint_resumes"],
        paused_steps=paused_steps,
    )
    return case, checked


def summarize(cases):
    result = []
    for noise in ("all", "white", "correlated"):
        for mode in ("all", "none", "random", "blackout"):
            for arm in ("fused_preview", "mean_preview"):
                for status in ("all", "held", "timeout"):
                    selected = [
                        c
                        for c in cases
                        if c["arm"] == arm
                        and (
                            noise == "all"
                            or (float(c["tau"]) == 0) == (noise == "white")
                        )
                        and (mode == "all" or c["mode"] == mode)
                        and (status == "all" or c["status"] == status)
                    ]
                    result.append(
                        dict(
                            noise_selector=noise,
                            mode_selector=mode,
                            arm=arm,
                            outcome_selector=status,
                            cases=len(selected),
                            **{k: sum(c[k] for c in selected) for k in COUNT_FIELDS},
                            **{
                                k: max((c[k] for c in selected), default=0)
                                for k in MAX_FIELDS
                            },
                            **{
                                "endpoint_"
                                + k: sum(pilot.truth(c["final_" + k]) for c in selected)
                                for k in ("paused", "guard", "latched")
                            },
                        )
                    )
    return result


def paired(cases):
    grouped = {}
    for case in cases:
        key = tuple(case[k] for k in PAIR_KEYS)
        need(case["arm"] not in grouped.setdefault(key, {}), "Duplicate paired arm")
        grouped[key][case["arm"]] = case
    rows = []
    for key, arms in sorted(grouped.items()):
        need(set(arms) == {"fused_preview", "mean_preview"}, "Missing census pair")
        rows.append(
            dict(zip(PAIR_KEYS, key))
            | {
                prefix + "_" + field: arms[arm][field]
                for prefix, arm in (
                    ("reference", "fused_preview"),
                    ("candidate", "mean_preview"),
                )
                for field in PAIR_VALUES
            }
        )
    return rows


def collect(raw, output):
    pins = preflight()
    raw, output = Path(raw).resolve(), build.fresh(output)
    need(
        build.sha(raw / "provenance.json") == pins["raw_provenance_sha256"],
        "Different census raw packet",
    )
    provenance = live.check_provenance(raw, build.preflight())
    manifest = json.loads((build.STUDY / "raw-file-manifest.json").read_text())
    for name, record in manifest["sha256"].items():
        path = raw / name
        need(
            path.stat().st_size == record["bytes"]
            and build.sha(path) == record["sha256"],
            f"Changed raw census file: {name}",
        )
    need(
        not subprocess.check_output(
            ["git", "status", "--porcelain=v1"], cwd=build.REPO, text=True
        ).strip(),
        "Commit the census implementation before extracting new results",
    )
    implementation = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=build.REPO, text=True
    ).strip()
    output.mkdir(parents=True)
    try:
        work = output / "native"
        compiler = native.compile_native(work)
        packet, query_count = extract(raw, provenance, work)
        need(
            query_count == pins["expected_decisions"],
            "Census native population differs",
        )
        evaluation = native.evaluate(work, packet)
        prior = {r["case_id"]: r for r in pilot.read(build.STUDY / "cases.csv")}
        need(len(prior) == pins["expected_cases"], "Frozen case catalogue differs")
        cases, checks, runs, events, patterns, counter = [], [], [], [], Counter(), [0]
        with (work / "trace.csv").open(newline="") as stream:
            geometries = csv.DictReader(stream)
            need(
                tuple(geometries.fieldnames) == native.TRACE_FIELDS,
                "Census native schema differs",
            )
            for meta, track, sides, guards in iter_cases(raw, provenance):
                case, checked = diagnose(
                    meta,
                    track,
                    sides,
                    guards,
                    prior[meta["case_id"]],
                    geometries,
                    counter,
                    patterns,
                    runs,
                    events,
                )
                cases.append(case)
                checks.append(checked)
                print(f"Verified both release timers: {meta['case_id']}", flush=True)
            need(
                next(geometries, None) is None and counter[0] == query_count,
                "Extra/missing census geometry",
            )
        need(
            len(cases) == len({c["case_id"] for c in cases}) == pins["expected_cases"]
            and {c["case_id"] for c in cases} == set(prior),
            "Census cases differ",
        )
        cases.sort(key=lambda c: c["case_id"])
        checks.sort(key=lambda c: c["case_id"])
        runs.sort(key=lambda r: (r["case_id"], r["phase"], r["run"]))
        events.sort(key=lambda e: (e["case_id"], e["step"], e["event"]))
        pairs, summary = paired(cases), summarize(cases)
        need(
            len(pairs) == pins["expected_pairs"]
            and len(summary) == pins["expected_summary_cells"],
            "Census export population differs",
        )
        tables = (
            ("cases.csv", cases, CASE_FIELDS),
            ("pairs.csv", pairs, PAIR_FIELDS),
            (
                "gate-patterns.csv",
                [
                    dict(zip(PATTERN_FIELDS, (*key, count)))
                    for key, count in sorted(patterns.items())
                ],
                PATTERN_FIELDS,
            ),
            ("ready-runs.csv", runs, RUN_FIELDS),
            ("events.csv", events, EVENT_FIELDS),
            ("summary.csv", summary, SUMMARY_FIELDS),
            ("verification.csv", checks, VERIFY_FIELDS),
        )
        for name, rows, fields in tables:
            pilot.write(output / name, rows, fields)
        sources = dict(
            method="full-decision original native geometry and exact joint/ordinary guard timer census",
            implementation_commit=implementation,
            working_tree_clean=True,
            protocol_sha256=build.sha(STUDY / "protocol.md"),
            freeze_sha256=build.sha(STUDY / "freeze-pins.json"),
            input_sha256=pins["input_sha256"],
            raw_provenance_sha256=build.sha(raw / "provenance.json"),
            raw_manifest_sha256=build.sha(build.STUDY / "raw-file-manifest.json"),
            script_sha256={name: build.sha(build.ROOT / name) for name in SCRIPTS},
            native=dict(compiler=compiler, evaluation=evaluation),
            audit=dict(
                cases=len(cases),
                pairs=len(pairs),
                geometry_decisions=query_count,
                joint_timer_decisions=sum(c["joint_steps"] for c in cases),
                guard_timer_decisions=sum(c["guard_update_steps"] for c in cases),
                verified_flag_decisions=query_count,
                ready_runs=len(runs),
                events=len(events),
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
    print(
        f"Verified all {query_count} decisions, {len(runs)} ready runs and {len(events)} events",
        flush=True,
    )
    return sources


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not __debug__:
        parser.error("Run without Python optimization")
    try:
        collect(args.raw, args.output)
    except (
        OSError,
        ValueError,
        KeyError,
        AssertionError,
        subprocess.CalledProcessError,
    ) as error:
        parser.exit(1, f"Release census failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
