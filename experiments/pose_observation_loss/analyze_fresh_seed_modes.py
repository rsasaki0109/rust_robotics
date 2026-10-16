#!/usr/bin/env python3
"""Audit additional fixed-seed runs and census immediate late timeout modes."""

import argparse
from collections import Counter, defaultdict
from fractions import Fraction
from itertools import product
import json
from pathlib import Path
import statistics

from analyze_commanded_recovery_arc import flag
from analyze_correction_centre_transitions import KINDS, checked_poses, opportunity_kind
from analyze_epoch_position_components import Replay
from analyze_epoch_preview import CollisionAudit, diagnostics, digest, paired, read_rows
from analyze_joint_admission_geometry import KEY
from analyze_terminal_arc_live import (
    COHORTS,
    SIMULATOR,
    VARIANTS,
    load_tracks,
    shared_prefix,
)
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PROTOCOL = ROOT / "results/fresh-seed-mode-census/protocol.md"
SEEDS = [277, 278, 279]
FRESH = ["fresh-white", "fresh-correlated"]
COHORT_SEEDS = {
    **{
        c: [269, 270, 271] if c.startswith("known-") else [274, 275, 276]
        for c in COHORTS
    },
    **{c: SEEDS for c in FRESH},
}
MODES = ["none", "random", "blackout"]
EVENT_GROUPS = ["all", "received_any", *KINDS]
PREVIEW = ["motion_filter", "three_epoch"]
ACTUAL = ["raw_fallback", "recovery_mean"]
SIM_SHA = "81711bb44ae1741c5f0e1f71180b047284bf098066f9b1246915130b094f9228"
PINNED = {
    "terminal-arc-live/cases.csv": "5dfb2eea39b95d8acd93843359fb71bc80944aeac25f81f6e643bdc6018d7b95",
    "correction-centre-transitions/events.csv": "819b026c74884681e8df9fb282998a19ade0d12f82e8d1ffa3ad1a3b5ce1df6f",
    "correction-centre-transitions/snapshots.csv": "9d25ee780dcb7695efee4c586509f8fe9c65edac9bb00b0cc5e4d16f2b8b5fcf",
    "correction-centre-transitions/sources.json": "e3dbe3ea3861f520035449371d2740bb12d5304df4dc09101589d2dc31fd71c5",
}


def identity(row):
    return tuple(str(row[k]) for k in KEY)


def accounting(row):
    cohort = row["cohort"]
    assert int(row["seed"]) in COHORT_SEEDS[cohort]
    assert flag(row["held"]) == (row["status"] == "held")
    assert int(row["intervals"]) == round(float(row["time_s"]) / 0.02)
    return dict(
        **{k: str(row[k]) for k in KEY},
        partition=(
            "fresh"
            if cohort in FRESH
            else "known" if cohort.startswith("known-") else "existing_new"
        ),
        noise="correlated" if cohort.endswith("correlated") else "white",
        status=row["status"],
        held=flag(row["held"]),
        collision=flag(row["collision"]),
        time_s=float(row["time_s"]),
        goal_distance=float(row["goal_distance"]),
        selected_timeout=row["status"] == "timeout",
        intervals=int(row["intervals"]),
        swept_contact_intervals=int(row["swept_contact_intervals"]),
        out_of_bounds_intervals=int(row["out_of_bounds_intervals"]),
    )


def event_record(event_id, case, event, actual, preview):
    assert case["selected_timeout"]
    time = float(event["event_time_s"])
    step = round(time / 0.02)
    assert 7500 <= step <= 9000 and abs(step * 0.02 - time) < 1e-8
    assert actual["estimator"] in ACTUAL and preview["estimator"] in PREVIEW
    received, scheduled, precision = (
        flag(event[k]) for k in ["received", "scheduled", "precision_activation"]
    )
    assert not received or scheduled
    expected = opportunity_kind(
        dict(
            correction=scheduled,
            correction_received=received,
            precision_activation=precision,
        )
    )
    assert event["event_kind"] == expected and expected in KINDS
    for s in [actual, preview]:
        assert flag(s.get("available", True))
        assert abs(float(s["time_s"]) - time) < 1e-8
        assert abs(float(s["truth_time_s"]) - (step - 1) * 0.02) < 1e-8
    assert flag(preview["fusion_available"]) == (preview["estimator"] == "three_epoch")
    assert int(preview["source_group_count"]) == (
        3 if preview["estimator"] == "three_epoch" else 1
    )
    recovery_count = int(actual["recovery_sample_count"])
    assert actual["estimator"] == (
        "recovery_mean" if recovery_count else "raw_fallback"
    )
    assert int(actual["used_sample_count"]) == max(1, recovery_count)
    assert int(actual["source_group_count"]) == 1
    record = dict(
        event_id=event_id,
        **{k: case[k] for k in KEY},
        partition=case["partition"],
        noise=case["noise"],
        event_kind=expected,
        event_time_s=time,
        truth_time_s=round((step - 1) * 0.02, 6),
        scheduled=scheduled,
        received=received,
        precision_activation=precision,
        preview_estimator=preview["estimator"],
        actual_estimator=actual["estimator"],
        recovery_sample_count=recovery_count,
        motion_filter_sample_count=int(preview["motion_filter_sample_count"]),
        epoch_fusion_group_count=int(preview["epoch_fusion_group_count"]),
        fusion_available=flag(preview["fusion_available"]),
        observation_age=float(preview["observation_age"]),
    )
    for kind, s in [("actual", actual), ("preview", preview)]:
        record[kind + "_used_sample_count"] = int(s["used_sample_count"])
        for axis in ["x", "y", "yaw"]:
            record[kind + "_" + axis] = float(s["pose_" + axis])
    return record


def validate_events(cases, events):
    lookup = {identity(c): c for c in cases}
    assert len(lookup) == len(cases), "duplicate case"
    seen_ids, seen_steps, grouped = set(), set(), defaultdict(list)
    for e in events:
        case = lookup[identity(e)]
        assert case["selected_timeout"] and all(
            e[k] == case[k] for k in ["partition", "noise"]
        )
        key = identity(e), e["event_time_s"]
        assert (
            e["event_id"] not in seen_ids and key not in seen_steps
        ), "duplicate event"
        seen_ids.add(e["event_id"])
        seen_steps.add(key)
        grouped[identity(e)].append(e)
    for c in cases:
        rows = grouped[identity(c)]
        if c["selected_timeout"]:
            assert c["intervals"] == 9000
            assert {e["event_time_s"] for e in rows if e["scheduled"]} == set(
                range(150, 181, 5)
            ), "scheduled timeout coverage"
        else:
            assert not rows


def historical():
    for name, sha in PINNED.items():
        assert digest(ROOT / "results" / name) == sha, name
    cases = [
        accounting(r) for r in read_rows(ROOT / "results/terminal-arc-live/cases.csv")
    ]
    lookup = {identity(c): c for c in cases}
    source = read_rows(ROOT / "results/correction-centre-transitions/events.csv")
    snapshots = defaultdict(dict)
    for s in read_rows(ROOT / "results/correction-centre-transitions/snapshots.csv"):
        if s["phase"] == "immediate":
            assert s["input_kind"] not in snapshots[s["event_id"]]
            snapshots[s["event_id"]][s["input_kind"]] = s
    assert len(cases) == 648 and len(source) == len(snapshots) == 2583
    events = []
    for e in source:
        s = snapshots[e["event_id"]]
        assert set(s) == {"actual", "preview"}
        for r in s.values():
            assert identity(r) == identity(e) and r["event_kind"] == e["event_kind"]
        events.append(
            event_record(
                "historical_" + e["event_id"],
                lookup[identity(e)],
                e,
                s["actual"],
                s["preview"],
            )
        )
    validate_events(cases, events)
    return cases, events


def replay_events(case, track, first_id):
    state, result, maximum = Replay(), [], 0.0
    for step, (before, row) in enumerate(zip(track, track[1:]), 1):
        assert round(float(row["time_s"]) / 0.02) == step
        state.advance(before, row)
        state.check_counts(row)
        kind = opportunity_kind(row) if 7500 <= step <= 9000 else None
        if kind is None:
            continue
        poses, error = checked_poses(state, row)
        maximum = max(maximum, error)
        snapshots = {}
        for name in ["actual", "preview"]:
            estimator, groups = state.view(name)
            snapshots[name] = dict(
                estimator=estimator,
                time_s=row["time_s"],
                truth_time_s=before["time_s"],
                **{
                    f"pose_{axis}": value
                    for axis, value in zip(["x", "y", "yaw"], poses[name])
                },
                used_sample_count=sum(len(g["samples"]) for g in groups),
                source_group_count=len(groups),
                recovery_sample_count=row["recovery_sample_count"],
                motion_filter_sample_count=row["motion_filter_sample_count"],
                epoch_fusion_group_count=row["epoch_fusion_group_count"],
                fusion_available=row["epoch_fusion_available"],
                observation_age=row["observation_age"],
            )
        event = dict(
            event_kind=kind,
            event_time_s=row["time_s"],
            scheduled=flag(row["correction"]),
            received=flag(row["correction_received"]),
            precision_activation=flag(row["precision_activation"]),
        )
        result.append(
            event_record(
                f"fresh_{first_id + len(result)}",
                case,
                event,
                snapshots["actual"],
                snapshots["preview"],
            )
        )
    return result, maximum


def matches(e, kind):
    return kind == "all" or (
        e["received"] if kind == "received_any" else e["event_kind"] == kind
    )


def support(cases, events):
    grouped = defaultdict(list)
    for e in events:
        grouped[identity(e)].append(e)
    result = []
    for c, kind in product(cases, EVENT_GROUPS):
        rows = [e for e in grouped[identity(c)] if matches(e, kind)]
        counts = Counter(e["preview_estimator"] for e in rows)
        result.append(
            dict(
                **{
                    k: c[k]
                    for k in [*KEY, "partition", "noise", "status", "selected_timeout"]
                },
                event_group=kind,
                events=len(rows),
                filter_events=counts[PREVIEW[0]],
                fusion_events=counts[PREVIEW[1]],
                support=(
                    "mixed"
                    if len(counts) == 2
                    else (
                        "filter_only"
                        if counts[PREVIEW[0]]
                        else "fusion_only" if counts[PREVIEW[1]] else "no_events"
                    )
                ),
            )
        )
    return result


def summary(cases, events):
    case_groups, event_groups = defaultdict(list), defaultdict(list)
    for c in cases:
        for seed, mode in product(["all", c["seed"]], ["all", c["mode"]]):
            case_groups[c["cohort"], seed, c["variant"], mode].append(c)
    for e in events:
        for seed, mode, kind in product(
            ["all", e["seed"]],
            ["all", e["mode"]],
            [k for k in EVENT_GROUPS if matches(e, k)],
        ):
            event_groups[e["cohort"], seed, e["variant"], mode, kind].append(e)
    result = []
    for cohort in [*COHORTS, *FRESH]:
        for seed, variant, mode, kind in product(
            ["all", *map(str, COHORT_SEEDS[cohort])],
            VARIANTS,
            ["all", *MODES],
            EVENT_GROUPS,
        ):
            cs = case_groups[cohort, seed, variant, mode]
            es = event_groups[cohort, seed, variant, mode, kind]
            parent = Counter(identity(e) for e in es)
            counts = Counter(e["preview_estimator"] for e in es)
            by_case = defaultdict(set)
            for e in es:
                by_case[identity(e)].add(e["preview_estimator"])
            assert sum(counts.values()) == len(es)
            record = dict(
                cohort=cohort,
                seed=seed,
                variant=variant,
                mode=mode,
                event_group=kind,
                accounting_cases=len(cs),
                timeout_cases=sum(c["selected_timeout"] for c in cs),
                non_timeout_cases=sum(not c["selected_timeout"] for c in cs),
                events=len(es),
                event_cases=len(parent),
                filter_cases=sum(PREVIEW[0] in s for s in by_case.values()),
                fusion_cases=sum(PREVIEW[1] in s for s in by_case.values()),
                mixed_cases=sum(len(s) == 2 for s in by_case.values()),
                no_event_timeout_cases=sum(
                    c["selected_timeout"] and identity(c) not in parent for c in cs
                ),
                raw_fallback_events=sum(
                    e["actual_estimator"] == "raw_fallback" for e in es
                ),
                recovery_mean_events=sum(
                    e["actual_estimator"] == "recovery_mean" for e in es
                ),
            )
            for label, estimator in zip(["filter", "fusion"], PREVIEW):
                mass = sum(
                    (
                        Fraction(1, parent[identity(e)])
                        for e in es
                        if e["preview_estimator"] == estimator
                    ),
                    Fraction(),
                )
                record[label + "_events"] = counts[estimator]
                record[label + "_event_share"] = (
                    counts[estimator] / len(es) if es else ""
                )
                record[label + "_case_mass"] = float(mass)
                record[label + "_case_weighted_share"] = (
                    float(mass / len(parent)) if parent else ""
                )
            assert record["filter_cases"] + record["fusion_cases"] - record[
                "mixed_cases"
            ] == len(parent)
            result.append(record)
    return result


def outcomes(cases, pairs):
    result = []
    for cohort, seed, mode, variant in product(
        FRESH, ["all", *map(str, SEEDS)], ["all", *MODES], VARIANTS
    ):

        def select(r):
            return (
                r["cohort"] == cohort
                and (seed == "all" or r["seed"] == seed)
                and (mode == "all" or r["mode"] == mode)
            )

        cs = [c for c in cases if select(c) and c["variant"] == variant]
        ps = [p for p in pairs if select(p)]
        both = [p for p in ps if p["both_held"]]
        result.append(
            dict(
                cohort=cohort,
                seed=seed,
                mode=mode,
                variant=variant,
                cases=len(cs),
                held=sum(c["held"] for c in cs),
                timeout=sum(c["selected_timeout"] for c in cs),
                collision=sum(c["collision"] for c in cs),
                out_of_bounds=sum(c["status"] == "out_of_bounds" for c in cs),
                paired_cases=len(ps),
                gained_holds=sum(p["gained_hold"] for p in ps),
                lost_holds=sum(p["lost_hold"] for p in ps),
                both_held_pairs=len(both),
                mean_both_held_time_delta_s=(
                    statistics.mean(p["held_time_difference_s"] for p in both)
                    if both
                    else ""
                ),
                mean_matched_wait_delta_s=(
                    statistics.mean(p["matched_wait_difference_s"] for p in ps)
                    if ps
                    else ""
                ),
                changed_command_pairs=sum(
                    bool(p["first_command_change_s"]) for p in ps
                ),
            )
        )
    return result


def audit_fresh(raw_root):
    maps = defaultdict(list)
    for r in read_rows(ROOT / "inputs/obstacles.csv"):
        maps[r["scenario"]].append((float(r["x"]), float(r["y"])))
    audits = {name: CollisionAudit(points) for name, points in maps.items()}
    fixtures = ROOT / "results/planning-clearance/fixtures/fine-padding"
    inputs = {p.name: digest(p) for p in fixtures.glob("*.csv")}
    cases, events, pairs, verification, sources = [], [], [], [], {}
    for cohort, tau in zip(FRESH, [0, 1]):
        run = raw_root / ("white" if tau == 0 else "correlated")
        provenance = json.loads((run / "provenance.json").read_text())
        assert provenance["seeds"] == SEEDS and provenance["variants"] == VARIANTS
        assert provenance["noise_correlation_time_s"] == tau
        assert provenance["simulator_sha256"] == digest(SIMULATOR) == SIM_SHA
        assert provenance["inputs_sha256"] == inputs
        assert all(
            provenance["inputs_by_variant_sha256"][v] == inputs for v in VARIANTS
        )
        results = read_rows(run / "all-results.csv")
        index = {
            (r["variant"], r["mode"], r["seed"], r["scenario"], r["planner"]): r
            for r in results
        }
        assert len(index) == len(results) == 162
        controls = {
            tuple(r[k] for k in ["variant", "mode", "seed", "scenario", "planner"]): r
            for r in read_rows(run / "control-events.csv")
        }
        assert set(controls) == set(index)
        previews = read_rows(run / "preview-audit.csv")
        assert previews and all(
            float(r["terminal_chord_error"])
            <= float(r["terminal_sagitta_bound"]) + 1e-9
            for r in previews
        )
        manifest = json.loads((run / "manifest.json").read_text())
        assert {(r["variant"], r["mode"], r["seed"]) for r in manifest} == set(
            product(VARIANTS, MODES, SEEDS)
        )
        assert len(manifest) == 18
        hashes = {}
        intervals = timeout_intervals = prefixes_checked = 0
        maximum = 0.0
        for mode, seed in product(MODES, SEEDS):
            groups = []
            for variant in VARIANTS:
                trial = run / f"{variant}-{mode}-{seed}"
                trial_results = read_rows(trial / "results.csv")
                assert len(trial_results) == 9
                for result in trial_results:
                    saved = index[
                        variant, mode, str(seed), result["scenario"], result["planner"]
                    ]
                    assert all(
                        saved[k] == value for k, value in result.items()
                    ), "trial summary mismatch"
                for name in ["trajectories.csv", "results.csv", "simulation.log"]:
                    hashes[f"{trial.name}/{name}"] = digest(trial / name)
                groups.append(load_tracks(trial / "trajectories.csv"))
            assert set(groups[0]) == set(groups[1])
            for key in sorted(groups[0]):
                current, prefixes = [], []
                for variant, tracks in zip(VARIANTS, groups):
                    track = tracks[key]
                    row = index[variant, mode, str(seed), *key]
                    diagnostic, prefix = diagnostics(track)
                    contacts, bounds = audits[key[0]].check(track)
                    assert bool(contacts) == flag(row["collision"])
                    assert bool(bounds) == (row["status"] == "out_of_bounds")
                    control = controls[variant, mode, str(seed), *key]
                    assert (
                        abs(
                            diagnostic["wait_time_s"]
                            - float(control["total_wait_time_s"])
                        )
                        < 1e-8
                    )
                    assert track[0]["terminal_steering_preview"] == "false"
                    assert all(
                        flag(r["terminal_steering_preview"]) == (variant == "curved")
                        for r in track[1:]
                    )
                    assert abs(float(track[-1]["time_s"]) - float(row["time_s"])) < 1e-8
                    record = dict(
                        cohort=cohort,
                        **row,
                        **diagnostic,
                        swept_contact_intervals=contacts,
                        out_of_bounds_intervals=bounds,
                    )
                    case = accounting(record)
                    cases.append(case)
                    if case["selected_timeout"]:
                        new_events, error = replay_events(case, track, len(events) + 1)
                        events.extend(new_events)
                        maximum = max(maximum, error)
                        timeout_intervals += len(track) - 1
                    current.append(record)
                    prefixes.append(prefix)
                    intervals += len(track) - 1
                prefixes_checked += shared_prefix(groups[0][key], groups[1][key])
                pairs.append(paired(*current, prefixes, [g[key] for g in groups]))
            del groups
            print(
                f"{cohort} {mode} {seed}: physics/prefix/timeout replay checked",
                flush=True,
            )
        verification.append(
            dict(
                cohort=cohort,
                cases=162,
                physical_intervals=intervals,
                timeout_replayed_intervals=timeout_intervals,
                shared_prefix_intervals=prefixes_checked,
                preview_checkpoints=len(previews),
                max_event_pose_reconstruction_error=maximum,
            )
        )
        sources[cohort] = dict(
            path=str(run),
            provenance=provenance,
            raw_sha256=hashes,
            artifacts_sha256={
                n: digest(run / n)
                for n in [
                    "provenance.json",
                    "manifest.json",
                    "all-results.csv",
                    "control-events.csv",
                    "preview-audit.csv",
                ]
            },
        )
    assert len(cases) == 324 and len(pairs) == 162
    validate_events(cases, events)
    return cases, events, pairs, verification, sources


def repeat_check(raw_root):
    record_path = raw_root / "repeat-invocation.json"
    invocation = json.loads(record_path.read_text())
    first = raw_root / "white/epoch-none-277"
    repeated = raw_root / "repeat-first"
    command = invocation["command"]
    assert command[0].endswith("release/examples/pose_observation_loss")
    assert Path(command[2]) == repeated and command[6] == "277"
    assert invocation["binary_sha256"] == digest(Path(command[0]))
    compared = {}
    for name in ["trajectories.csv", "results.csv"]:
        assert (first / name).read_bytes() == (
            repeated / name
        ).read_bytes(), "deterministic rollout mismatch"
        compared[name] = digest(first / name)
    return dict(
        **invocation,
        record_sha256=digest(record_path),
        identical_sha256=compared,
        cases=9,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    old_cases, old_events = historical()
    fresh_cases, fresh_events, pairs, verification, raw_sources = audit_fresh(
        args.raw_root.resolve()
    )
    cases, events = old_cases + fresh_cases, old_events + fresh_events
    validate_events(cases, events)
    helpers = json.loads(
        (ROOT / "results/correction-centre-transitions/sources.json").read_text()
    )["analysis"]["helpers_sha256"]
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    for name in [
        "run.py",
        "verify.py",
        "verify_motion_filter.py",
        "verify_stopping_preview.py",
        "verify_joint_recovery.py",
        "analyze_terminal_arc_live.py",
        "analyze_correction_centre_transitions.py",
    ]:
        helpers[name] = digest(ROOT / name)
    sources = dict(
        input_sha256=PINNED,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            simulator_sha256=digest(SIMULATOR),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        fresh=raw_sources,
        repeat_first=repeat_check(args.raw_root.resolve()),
        schema=dict(
            fresh_seeds=SEEDS,
            event_groups=EVENT_GROUPS,
            preview_estimators=PREVIEW,
            actual_estimators=ACTUAL,
            population="all immediate [150,180] correction/precision opportunities in every timeout; not the narrow speed-probe audit",
            parent_case_weights="1/n_events in each reporting group before preview conditioning",
            independent_new_seed_streams=3,
        ),
        audit=dict(
            accounted_cases=len(cases),
            fresh_cases=len(fresh_cases),
            historical_events=len(old_events),
            fresh_events=len(fresh_events),
            fresh_timeout_cases=sum(c["selected_timeout"] for c in fresh_cases),
            fresh_physical_intervals=sum(v["physical_intervals"] for v in verification),
        ),
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", cases),
        ("events", events),
        ("case-support", support(cases, events)),
        ("summary", summary(cases, events)),
        ("fresh-pairs", pairs),
        ("fresh-outcomes", outcomes(fresh_cases, pairs)),
        ("verification", verification),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(sources["audit"], sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
