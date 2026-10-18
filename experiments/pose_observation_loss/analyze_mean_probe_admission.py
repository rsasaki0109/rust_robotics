#!/usr/bin/env python3
"""Diagnose all joint admission decisions in the frozen mean speed-probe pilot."""

import argparse
from collections import Counter
import csv
from decimal import Decimal, localcontext
import itertools
import json
from pathlib import Path
import subprocess

import analyze_mean_speed_probe as pilot
import build_mean_speed_probe as build
import native_mean_probe_admission as native

STUDY = build.ROOT / "results/mean-speed-probe-admission-audit"
PILOT = build.STUDY
META = ("case_id", *pilot.CASE_META)
GATES = ("fresh", "stationary", "samples", "clearance", "bounds", "nominal", "release")
COUNTS = (
    "decisions",
    "pending_steps",
    "eligible_steps",
    "eligible_nominal_positive",
    "eligible_positive_other_fail",
    "eligible_positive_unresolved",
    "eligible_nominal_zero",
    "eligible_latched_zero",
    "eligible_unlatched_probe_zero",
    "eligible_latched_and_probe_zero",
    "joint_ready_steps",
    "release_ambiguous_steps",
    "timer_verified_steps",
    "timer_unverified_steps",
    "resumed",
    "mean_used_steps",
    "ready_runs",
    "resets",
)
CASE_FIELDS = (
    *META,
    "status",
    "held",
    "collision",
    "boundary",
    "time_s",
    "goal_distance",
    *COUNTS,
    "max_ready_run_steps",
    "final_brake_mask",
    "final_paused",
    "final_guard",
    "final_latched",
    "final_hold_s",
    "final_release_gate",
    "last_pending_step",
    "last_pending_failed_gates",
    "last_pending_unresolved_gates",
)
PATTERN_FIELDS = (*META, "phase", "failed_gates", "unresolved_gates", "steps")
RUN_FIELDS = (
    *META,
    "run",
    "first_step",
    "last_step",
    "steps",
    "duration_s",
    "last_timer_verified",
    "ending",
    "break_step",
    "failed_gates",
    "unresolved_gates",
)
SUMMARY_FIELDS = (
    "noise_selector",
    "mode_selector",
    "arm",
    "cases",
    "held",
    "timeouts",
    "pending_cases",
    *COUNTS,
    "max_ready_run_steps",
    "endpoint_paused",
    "endpoint_guard",
    "endpoint_latched",
)
need = build.require


def preflight():
    pins = json.loads((STUDY / "freeze-pins.json").read_text())
    need(
        build.sha(STUDY / "protocol.md") == pins["protocol_sha256"],
        "Diagnostic protocol changed",
    )
    for name, digest in pins["input_sha256"].items():
        need(
            build.sha(build.REPO / name) == digest, f"Changed diagnostic input: {name}"
        )
    amendment = json.loads((STUDY / "amendment-pins.json").read_text())
    need(
        amendment["original_protocol_sha256"] == pins["protocol_sha256"],
        "Amendment targets another protocol",
    )
    need(
        build.sha(STUDY / "protocol-amendment-1.md") == amendment["amendment_sha256"],
        "Margin amendment changed",
    )
    return pins


def release_gate(text, margin, margin_rounded=False):
    """Classify strict > from the closed nine-decimal rounding interval."""
    numeric_margin = pilot.finite(margin)
    pilot.finite(text)
    with localcontext() as context:
        context.prec = 80
        center = Decimal(text)
        need(center.is_finite(), "Nonfinite release clearance")
        half = Decimal("0.0000000005")
        threshold = (
            Decimal(margin) if margin_rounded else Decimal.from_float(numeric_margin)
        )
        margin_half = half if margin_rounded else Decimal(0)
        if center - half > threshold + margin_half:
            return True
        if center + half <= threshold - margin_half:
            return False
    return None


def rounded_matches(text, actual, digits=6):
    actual = pilot.finite(actual)
    pilot.finite(text)
    with localcontext() as context:
        context.prec = 80
        center = Decimal(text)
        need(center.is_finite(), "Nonfinite timer")
        return abs(center - Decimal.from_float(actual)) <= Decimal(5).scaleb(
            -digits - 1
        )


def conjunction(states):
    if any(value is False for value in states.values()):
        return False
    return None if any(value is None for value in states.values()) else True


def masks(states):
    return (
        ";".join(name for name in GATES if states[name] is False),
        ";".join(name for name in GATES if states[name] is None),
    )


def gates(row, side, geometry):
    need(
        rounded_matches(
            geometry["recovery_clearance_9"], geometry["recovery_clearance"], 9
        )
        and rounded_matches(
            geometry["release_margin_9"], geometry["release_margin"], 9
        ),
        "Inconsistent full/rounded native gate values",
    )
    need(
        geometry["recovery_clearance_9"] == row["recovery_clearance"],
        "Native recovery clearance differs",
    )
    need(
        geometry["release_margin_9"] == row["guard_release_margin"],
        "Native release margin differs",
    )
    margin = pilot.finite(side["margin"])
    release_margin = pilot.finite(geometry["release_margin"])
    need(
        pilot.finite(geometry["recovery_margin"]) == margin,
        "Native recovery margin differs",
    )
    need(
        geometry["precision_active"] == row["precision_active"],
        "Native precision epoch differs",
    )
    need(
        geometry["effective_position_sigma_9"] == row["effective_position_sigma"],
        "Native effective sigma differs",
    )
    need(
        int(side["count"]) == int(row["recovery_sample_count"]), "Sample count differs"
    )
    return dict(
        fresh=pilot.finite(side["observation_age"]) <= 1.5,
        stationary=abs(pilot.finite(side["measured_speed"])) <= 0.02,
        samples=int(side["count"]) >= 10,
        clearance=pilot.finite(geometry["recovery_clearance"]) > margin,
        bounds=pilot.truth(geometry["recovery_bounds"]),
        nominal=pilot.finite(side["nominal_target"]) > 0,
        release=release_gate(
            row["release_preview_clearance_lower_bound"], release_margin
        ),
    )


class Timer:
    def __init__(self):
        self.evidence = 0.0

    def step(self, ready, resumed, recorded, paused):
        need(type(ready) is bool or ready is None, "Invalid readiness state")
        amount = (
            0.0
            if ready is False
            else (
                self.evidence + 0.02
                if ready is True and self.evidence is not None
                else None
            )
        )
        if amount is None:
            need(paused is not resumed, "Unresolved timer pause/release state differs")
            need(0 <= pilot.finite(recorded) < 1.0, "Invalid unresolved timer value")
            self.evidence = 0.0 if resumed else None
            return False
        release = amount + 1e-9 >= 1.0
        need(resumed == release and paused != release, "Native joint admission differs")
        self.evidence = 0.0 if release else amount
        need(rounded_matches(recorded, self.evidence), "Native joint timer differs")
        return True


def diagnose_case(
    meta, track, sides, outcome, geometries, query_counter, patterns, runs
):
    need(len(track) >= 2 and len(sides) + 1 == len(track), "Incomplete diagnostic case")
    key = meta["scenario"], meta["planner"]
    need(
        all((r["scenario"], r["planner"]) == key for r in (*track, *sides)),
        "Mixed diagnostic cases",
    )
    counts = Counter({name: 0 for name in COUNTS})
    timer, active, run_index, maximum = Timer(), None, 0, 0
    last_pending, last_failed, last_unknown = "", "", ""

    def close_run(ending, break_step="", failed="", unknown=""):
        nonlocal active
        if active is not None:
            runs.append(
                dict(
                    **meta,
                    **active,
                    duration_s=active["steps"] * 0.02,
                    ending=ending,
                    break_step=break_step,
                    failed_gates=failed,
                    unresolved_gates=unknown,
                )
            )
            counts["ready_runs"] += 1
            counts["resets"] += ending == "failure"
            active = None

    for step, (before, row, side) in enumerate(zip(track, track[1:], sides), 1):
        need(int(side["step"]) == step, "Diagnostic step order differs")
        pilot.close(side["time_s"], step * 0.02)
        pilot.close(row["time_s"], step * 0.02)
        pending = pilot.truth(side["paused"])
        missing = pilot.truth(row["correction"]) and not pilot.truth(
            row["correction_received"]
        )
        need(
            pending == (pilot.truth(before["safety_paused"]) or missing),
            "Pending state differs",
        )
        resumed, paused = pilot.truth(row["resumed"]), pilot.truth(row["safety_paused"])
        counts["decisions"] += 1
        counts["mean_used_steps"] += pilot.truth(side["used"])
        counts["resumed"] += resumed
        if not pending:
            need(
                not resumed
                and not paused
                and rounded_matches(row["recovery_evidence"], 0),
                "Unexpected non-pending timer",
            )
            need(not pilot.truth(side["used"]), "Mean pose used outside a pause")
            timer.evidence = 0.0
            close_run("pause_end", step)
            continue
        counts["pending_steps"] += 1
        geometry = next(geometries, None)
        query_counter[0] += 1
        need(
            geometry is not None and geometry["id"] == str(query_counter[0]),
            "Missing/reordered native gate query",
        )
        states = gates(row, side, geometry)
        failed, unknown = masks(states)
        ready = conjunction(states)
        eligible = all(states[name] for name in ("fresh", "stationary", "samples"))
        need(
            pilot.truth(side["used"]) == (eligible and meta["arm"] == "mean_probe"),
            "Pilot eligibility differs",
        )
        last_pending, last_failed, last_unknown = step, failed, unknown
        for phase in ("pending_all", "mean_eligible") if eligible else ("pending_all",):
            patterns[(*tuple(meta[name] for name in META), phase, failed, unknown)] += 1
        counts["release_ambiguous_steps"] += states["release"] is None
        counts["eligible_steps"] += eligible
        if eligible:
            positive = states["nominal"]
            counts["eligible_nominal_positive"] += positive
            counts["eligible_positive_other_fail"] += positive and ready is False
            counts["eligible_positive_unresolved"] += positive and ready is None
            counts["eligible_nominal_zero"] += not positive
            if not positive:
                latched = pilot.finite(side["uncapped_target"]) == 0.0
                probe_zero = pilot.finite(side["free_distance"]) == 0.0
                need(
                    (latched and pilot.truth(row["stop_latched"]))
                    or (not latched and probe_zero),
                    "Zero nominal command has an unknown origin",
                )
                counts["eligible_latched_zero"] += latched
                counts["eligible_unlatched_probe_zero"] += not latched
                counts["eligible_latched_and_probe_zero"] += latched and probe_zero
        verified = timer.step(ready, resumed, row["recovery_evidence"], paused)
        counts["timer_verified_steps" if verified else "timer_unverified_steps"] += 1
        counts["joint_ready_steps"] += ready is True
        if ready is True:
            if active is None:
                run_index += 1
                active = dict(run=run_index, first_step=step, last_step=step, steps=0)
            active["last_step"] = step
            active["steps"] += 1
            active["last_timer_verified"] = str(verified).lower()
            maximum = max(maximum, active["steps"])
            if resumed:
                close_run("admission", step)
        else:
            close_run(
                "failure" if ready is False else "ambiguity", step, failed, unknown
            )
    close_run("packet_endpoint")
    need(
        counts["eligible_steps"]
        == counts["eligible_nominal_zero"] + counts["eligible_nominal_positive"],
        "Eligible command partition differs",
    )
    need(
        counts["eligible_nominal_positive"]
        == counts["joint_ready_steps"]
        + counts["eligible_positive_other_fail"]
        + counts["eligible_positive_unresolved"],
        "Positive-command partition differs",
    )
    need(
        counts["eligible_nominal_zero"]
        == counts["eligible_latched_zero"] + counts["eligible_unlatched_probe_zero"],
        "Zero-command partition differs",
    )
    need(
        counts["timer_verified_steps"] + counts["timer_unverified_steps"]
        == counts["pending_steps"],
        "Timer coverage differs",
    )
    need(
        counts["decisions"] == int(outcome["decisions"])
        and counts["mean_used_steps"] == int(outcome["mean_used_steps"]),
        "Frozen pilot counts differ",
    )
    last, side = track[-1], sides[-1]
    brakes = ("stop_latched", "safety_paused", "guard_active", "speed_braking")
    release = release_gate(
        last["release_preview_clearance_lower_bound"],
        last["guard_release_margin"],
        margin_rounded=True,
    )
    return dict(
        **meta,
        **{
            name: outcome[name]
            for name in (
                "status",
                "held",
                "collision",
                "boundary",
                "time_s",
                "goal_distance",
            )
        },
        **counts,
        max_ready_run_steps=maximum,
        final_brake_mask=";".join(name for name in brakes if pilot.truth(last[name])),
        final_paused=last["safety_paused"],
        final_guard=last["guard_active"],
        final_latched=last["stop_latched"],
        final_hold_s=last["hold_time_s"],
        final_release_gate=(
            "unresolved" if release is None else "pass" if release else "fail"
        ),
        last_pending_step=last_pending,
        last_pending_failed_gates=last_failed,
        last_pending_unresolved_gates=last_unknown,
    )


def identity(row):
    return (
        float(row["tau"]),
        row["arm"],
        row["mode"],
        int(row["seed"]),
        row["scenario"],
        row["planner"],
    )


def summarize(cases):
    result = []
    for noise in ("all", "white", "correlated"):
        for mode in ("all", "none", "random", "blackout"):
            for arm in ("raw_probe", "mean_probe"):
                selected = [
                    r
                    for r in cases
                    if r["arm"] == arm
                    and (noise == "all" or (float(r["tau"]) == 0) == (noise == "white"))
                    and (mode == "all" or r["mode"] == mode)
                ]
                result.append(
                    dict(
                        noise_selector=noise,
                        mode_selector=mode,
                        arm=arm,
                        cases=len(selected),
                        held=sum(pilot.truth(r["held"]) for r in selected),
                        timeouts=sum(r["status"] == "timeout" for r in selected),
                        pending_cases=sum(r["pending_steps"] > 0 for r in selected),
                        **{name: sum(r[name] for r in selected) for name in COUNTS},
                        max_ready_run_steps=max(
                            (r["max_ready_run_steps"] for r in selected), default=""
                        ),
                        **{
                            "endpoint_"
                            + name: sum(
                                pilot.truth(r["final_" + name]) for r in selected
                            )
                            for name in ("paused", "guard", "latched")
                        },
                    )
                )
    return result


def extract_queries(raw, provenance, directory):
    count = 0
    path = directory / "queries.tsv"
    with path.open("x", newline="") as stream:
        writer = csv.writer(stream, delimiter="\t", lineterminator="\n")
        writer.writerow(
            (
                "id",
                "scenario",
                "x",
                "y",
                "yaw",
                "margin",
                "observation_age",
                "precision_active",
                "position_sigma",
                "drift_speed",
                "recovery_guard_buffer",
            )
        )
        for trial in provenance["trials"]:
            previous_key, precision = None, False
            with (raw / trial["directory"] / "speed-probes.csv").open(
                newline=""
            ) as source:
                reader = csv.DictReader(source)
                need(
                    tuple(reader.fieldnames) == build.SIDECAR,
                    "Diagnostic sidecar schema differs",
                )
                for side in reader:
                    key = side["scenario"], side["planner"]
                    if key != previous_key:
                        previous_key, precision = key, False
                    precision |= pilot.truth(side["precision_activation"])
                    if pilot.truth(side["paused"]):
                        count += 1
                        writer.writerow(
                            (
                                count,
                                side["scenario"],
                                *(
                                    side[name]
                                    for name in (
                                        "mean_x",
                                        "mean_y",
                                        "mean_yaw",
                                        "margin",
                                    )
                                ),
                                side["observation_age"],
                                str(precision).lower(),
                                trial["command"][4],
                                trial["command"][7],
                                trial["command"][21],
                            )
                        )
    return path, count


def collect(raw, output):
    pins = preflight()
    raw, output = Path(raw).resolve(), build.fresh(output)
    need(
        build.sha(raw / "provenance.json") == pins["raw_provenance_sha256"],
        "Different pilot raw packet",
    )
    provenance = pilot.check_provenance(raw, build.preflight())
    need(
        not subprocess.check_output(
            ["git", "status", "--porcelain=v1"], cwd=build.REPO, text=True
        ).strip(),
        "Commit the diagnostic before extracting gate distributions",
    )
    implementation = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=build.REPO, text=True
    ).strip()
    output.mkdir(parents=True)
    try:
        work = output / "native"
        receipt = native.compile_geometry(work)
        queries, query_count = extract_queries(raw, provenance, work)
        receipt = native.evaluate(work, receipt, queries, query_count)
        old_cases = pilot.read(PILOT / "cases.csv")
        index = {
            identity(row): (f"C{i:04d}", row) for i, row in enumerate(old_cases, 1)
        }
        need(
            len(index) == len(old_cases) == pins["expected_cases"],
            "Frozen case catalogue differs",
        )
        cases, runs, patterns, counter = [], [], Counter(), [0]
        with (work / "trace.csv").open(newline="") as stream:
            geometries = csv.DictReader(stream)
            need(
                tuple(geometries.fieldnames) == native.TRACE_FIELDS,
                "Native diagnostic schema differs",
            )
            for trial in provenance["trials"]:
                directory = raw / trial["directory"]
                with (directory / "trajectories.csv").open(newline="") as tracks, (
                    directory / "speed-probes.csv"
                ).open(newline="") as sides:
                    seen = set()
                    for groups in itertools.zip_longest(
                        pilot.grouped(tracks), pilot.grouped(sides)
                    ):
                        need(
                            all(group is not None for group in groups)
                            and groups[0][0] == groups[1][0],
                            "Diagnostic native case grouping differs",
                        )
                        key = groups[0][0]
                        need(key not in seen, "Duplicate diagnostic case group")
                        seen.add(key)
                        meta = dict(
                            tau=trial["tau"],
                            arm=trial["arm"],
                            mode=trial["mode"],
                            seed=trial["seed"],
                            scenario=key[0],
                            planner=key[1],
                        )
                        case_id, old = index[identity(meta)]
                        cases.append(
                            diagnose_case(
                                dict(case_id=case_id, **meta),
                                groups[0][1],
                                groups[1][1],
                                old,
                                geometries,
                                counter,
                                patterns,
                                runs,
                            )
                        )
                    need(len(seen) == 9, "Incomplete diagnostic trial")
                print(f"Audited admission history: {trial['directory']}", flush=True)
            need(
                next(geometries, None) is None and counter[0] == query_count,
                "Extra/missing native geometry queries",
            )
        cases.sort(key=lambda r: r["case_id"])
        runs.sort(key=lambda r: (r["case_id"], r["run"]))
        need(
            len(cases) == len({identity(r) for r in cases}) == pins["expected_cases"],
            "Diagnostic case population differs",
        )
        need(
            sum(r["decisions"] for r in cases) == pins["expected_decisions"],
            "Diagnostic decision population differs",
        )
        tables = (
            ("case-gates.csv", cases, CASE_FIELDS),
            (
                "gate-patterns.csv",
                [
                    dict(zip(PATTERN_FIELDS, (*key, value)))
                    for key, value in sorted(patterns.items())
                ],
                PATTERN_FIELDS,
            ),
            ("admission-runs.csv", runs, RUN_FIELDS),
            ("summary.csv", summarize(cases), SUMMARY_FIELDS),
        )
        for name, rows, fields in tables:
            pilot.write(output / name, rows, fields)
        sources = dict(
            method="all frozen pilot cases; native recovery-line replay; interval-valued release gate; continuous joint admission audit",
            implementation_commit=implementation,
            working_tree_clean=True,
            protocol_sha256=build.sha(STUDY / "protocol.md"),
            freeze_sha256=build.sha(STUDY / "freeze-pins.json"),
            amendment_sha256=build.sha(STUDY / "protocol-amendment-1.md"),
            amendment_pins_sha256=build.sha(STUDY / "amendment-pins.json"),
            input_sha256=pins["input_sha256"],
            raw_provenance_sha256=build.sha(raw / "provenance.json"),
            diagnostic_sha256=build.sha(Path(__file__)),
            native_generator_sha256=build.sha(Path(native.__file__)),
            native_sources_sha256=build.sha(work / "native-sources.json"),
            audit=dict(
                cases=len(cases),
                pairs=pins["expected_pairs"],
                decisions=sum(r["decisions"] for r in cases),
                native_queries=query_count,
                ready_runs=len(runs),
                unresolved_release_gates=sum(
                    r["release_ambiguous_steps"] for r in cases
                ),
                unverified_timer_steps=sum(r["timer_unverified_steps"] for r in cases),
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
        f"Verified {len(cases)} cases, {query_count} native gate queries and {len(runs)} definite-ready runs",
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
        parser.exit(1, f"Admission diagnostic failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
