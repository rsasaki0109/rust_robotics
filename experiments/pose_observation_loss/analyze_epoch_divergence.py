#!/usr/bin/env python3
"""Diagnose first control forks and late release gates in pinned live runs."""

import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path

from analyze_epoch_preview import COHORTS, SIMULATOR, digest, read_rows
from analyze_filter_stalls import GOALS, conditions, pending
from analyze_recovery_stalls import MODES, command_mode
from summarize_stopping_preview import save_rows
from verify_joint_recovery import release_clearance, release_margin
from verify_stopping_preview import exact_release_preview, preview_components

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/epoch-preview"
KEY = ["cohort", "mode", "seed", "scenario", "planner"]
COMMAND = ["target_speed", "steering", "acceleration"]
ACTION = [
    "guard_active",
    "guard_triggered",
    "guard_released",
    "safety_paused",
    "resumed",
]
PHYSICAL = ["x", "y", "yaw", "v"]
INPUTS = [
    "nominal_target_speed",
    "nominal_steering",
    "measured_speed",
    "preview_margin",
    "guard_release_margin",
    "motion_filter_pose_x",
    "motion_filter_pose_y",
    "preview_pose_yaw",
    "observation_age",
    "effective_position_sigma",
    "noise_x",
    "noise_y",
    "noise_yaw",
    "bias_x",
    "bias_y",
    "bias_yaw",
    "correction",
    "correction_received",
    "precision_activation",
    "stop_latched",
    "recovery_x",
    "recovery_y",
    "recovery_yaw",
    "recovery_sample_count",
    "recovery_clearance",
]
REQUIREMENTS = [
    "stationary",
    "fresh",
    "samples",
    "recovery_geometry",
    "positive_command",
    "stop",
    "terminal",
]
MASKS = dict(
    actual=set(),
    without_stop={"stop"},
    without_terminal={"terminal"},
    without_preview={"stop", "terminal"},
)
DT = 0.02
EPS = 2e-8
PINNED = {
    "cases.csv": "a99ebb8073828d1c300b5a025a3e5640950f17aeb2c7b75b16cfe93c57c77b63",
    "pairs.csv": "fc53ccab5ef561e2433e555904f355a948e663dc2c3e685cdd8603dc3c26ecd7",
    "sources.json": "ceea11fd0eaaa4b992140caf2e8a9a62233ef4b38202ec0c33b6970e41362c28",
}


def different(a, b, fields):
    return any(a[k] != b[k] for k in fields)


def shared_inputs(before_a, row_a, before_b, row_b):
    return not different(before_a, before_b, PHYSICAL) and not different(
        row_a, row_b, INPUTS
    )


def release_inputs(before, row):
    """Read the active gate using the precise measured speed, including bounds."""
    if pending(before, row):
        route = "joint"
        result = conditions(dict(before, v=row["measured_speed"]), row)
        del result["guard_preview"]
    elif before["guard_active"] == "true":
        route = "independent"
        result = dict(stationary=abs(float(row["measured_speed"])) <= 0.02)
    else:
        return "inactive", {}
    margin = release_margin(row)
    stop = float(row["stop_preview_clearance_lower_bound"])
    terminal = float(row["terminal_recovery_clearance"]) - 0.05
    result.update(stop=stop > margin, terminal=terminal > margin)
    combined = release_clearance(row) > margin
    if (result["stop"] and result["terminal"]) != combined:
        assert min(abs(stop - margin), abs(terminal - margin)) < 2e-9
        raise AssertionError("Rounded gate boundary is ambiguous")
    return route, result


def first_fork(reference, candidate, published_time):
    n = min(len(reference), len(candidate))
    indices = {
        name: next(
            (i for i in range(1, n) if different(reference[i], candidate[i], fields)),
            None,
        )
        for name, fields in [
            ("action", ACTION),
            ("command", COMMAND),
            ("physical", PHYSICAL),
        ]
    }
    assert indices["action"] is not None and indices["command"] is not None
    assert float(candidate[indices["command"]]["time_s"]) == float(published_time)
    assert indices["action"] <= indices["command"]
    if indices["physical"] is not None:
        assert indices["command"] <= indices["physical"]
    indices.update(entry=None, gate=None, evidence=None)
    for i in range(1, indices["action"] + 1):
        a, b = reference[i], candidate[i]
        assert shared_inputs(reference[i - 1], a, candidate[i - 1], b), (
            i,
            "prefix input mismatch",
        )
        entry = [
            float(r["preview_clearance_lower_bound"]) <= float(r["preview_margin"])
            for r in [a, b]
        ]
        if (
            reference[i - 1]["guard_active"] != "true"
            and entry[0] != entry[1]
            and indices["entry"] is None
        ):
            indices["entry"] = i
        gates = [
            release_inputs(track[i - 1], track[i]) for track in [reference, candidate]
        ]
        assert gates[0][0] == gates[1][0]
        ready = [
            bool(requirements) and all(requirements.values())
            for _, requirements in gates
        ]
        if ready[0] != ready[1] and indices["gate"] is None:
            indices["gate"] = i
        if (
            different(a, b, ["guard_release_evidence", "recovery_evidence"])
            and indices["evidence"] is None
        ):
            indices["evidence"] = i
    i = indices["action"]
    a, b = reference[i], candidate[i]
    if a["resumed"] != b["resumed"]:
        kind = "joint_resume"
        earlier = "epoch" if b["resumed"] == "true" else "aligned"
    elif a["guard_triggered"] != b["guard_triggered"]:
        kind = "entry"
        earlier = "epoch" if b["guard_triggered"] == "true" else "aligned"
    else:
        assert a["guard_released"] != b["guard_released"]
        kind = "independent_release"
        earlier = "epoch" if b["guard_released"] == "true" else "aligned"
    if kind == "entry":
        indices["origin"] = i
    else:
        timer = (
            "recovery_evidence" if kind == "joint_resume" else "guard_release_evidence"
        )
        # Locate the last resynchronization of this timer. An earlier isolated
        # geometry difference that was later reset cannot explain this release.
        equal = max(j for j in range(i) if reference[j][timer] == candidate[j][timer])
        indices["origin"] = equal + 1
        origin_gates = [
            release_inputs(t[equal], t[equal + 1])[1] for t in [reference, candidate]
        ]
        assert all(origin_gates)
        assert all(origin_gates[0].values()) != all(origin_gates[1].values())
    origin = indices["origin"]
    if kind == "entry":
        indices["decisive"] = i
        winning_run = 0
    else:
        winner, loser = (
            (candidate, reference) if earlier == "epoch" else (reference, candidate)
        )
        expected_route = "joint" if kind == "joint_resume" else "independent"
        uninterrupted = []
        for j in range(i, 0, -1):
            route, ready = release_inputs(winner[j - 1], winner[j])
            if route != expected_route or not all(ready.values()):
                break
            uninterrupted.append(j)
        assert uninterrupted
        failures = [
            j
            for j in uninterrupted
            if not all(release_inputs(loser[j - 1], loser[j])[1].values())
        ]
        assert failures, "No failed readiness check explains the later release"
        indices["decisive"] = max(failures)
        winning_run = len(uninterrupted)
    decisive = indices["decisive"]
    original_error = math.hypot(
        *(
            float(candidate[origin][f"motion_filter_pose_{axis}"])
            - float(candidate[origin - 1][axis])
            for axis in ["x", "y"]
        )
    )
    fused_error = math.hypot(
        *(
            float(candidate[origin][f"preview_pose_{axis}"])
            - float(candidate[origin - 1][axis])
            for axis in ["x", "y"]
        )
    )
    stop_pass = [release_clearance(r) > release_margin(r) for r in [a, b]]
    record = dict(
        kind=kind,
        first_actor=earlier,
        current_release_geometry_differs=stop_pass[0] != stop_pass[1],
        history_only_release=kind != "entry" and stop_pass[0] == stop_pass[1],
        fusion_available_at_action=b["epoch_fusion_available"] == "true",
        original_position_error=math.hypot(
            *(
                float(b[f"motion_filter_pose_{axis}"]) - float(candidate[i - 1][axis])
                for axis in ["x", "y"]
            )
        ),
        fused_position_error=math.hypot(
            *(
                float(b[f"preview_pose_{axis}"]) - float(candidate[i - 1][axis])
                for axis in ["x", "y"]
            )
        ),
        origin_fusion_available=candidate[origin]["epoch_fusion_available"] == "true",
        origin_original_position_error=original_error,
        origin_fused_position_error=fused_error,
        origin_to_action_s=round((i - origin) * DT, 6),
        winning_ready_run_s=round(winning_run * DT, 6),
        decisive_to_action_s=round((i - decisive) * DT, 6),
        decisive_fusion_available=candidate[decisive]["epoch_fusion_available"]
        == "true",
        aligned_prior_guard_evidence=reference[i - 1]["guard_release_evidence"],
        epoch_prior_guard_evidence=candidate[i - 1]["guard_release_evidence"],
        aligned_prior_joint_evidence=reference[i - 1]["recovery_evidence"],
        epoch_prior_joint_evidence=candidate[i - 1]["recovery_evidence"],
    )
    record.update(
        {
            f"first_{name}_s": candidate[index]["time_s"] if index is not None else ""
            for name, index in indices.items()
        }
    )
    return indices, record


def pose_views(before, row):
    logged = tuple(float(row[f"preview_pose_{a}"]) for a in ["x", "y", "yaw"])
    return dict(
        logged=logged,
        original=(
            float(row["motion_filter_pose_x"]),
            float(row["motion_filter_pose_y"]),
            logged[2],
        ),
        true_xy=(float(before["x"]), float(before["y"]), logged[2]),
        truth=tuple(float(before[a]) for a in ["x", "y", "yaw"]),
    )


def geometry(before, row, obstacles):
    result, cache = [], {}
    for name, pose in pose_views(before, row).items():
        if pose not in cache:
            probe = dict(
                row,
                **{
                    f"preview_pose_{a}": str(v) for a, v in zip(["x", "y", "yaw"], pose)
                },
            )
            cache[pose] = preview_components(
                dict(before, v=row["measured_speed"]), probe, obstacles
            )
        stop, terminal, steps = cache[pose]
        entry = min(stop, terminal - 0.1)
        release = exact_release_preview(stop, terminal, row)
        entry_margin = float(row["preview_margin"])
        margin = release_margin(row)
        if name == "logged":
            assert float(row["stop_preview_clearance_lower_bound"]) <= stop + EPS
            assert abs(terminal - float(row["terminal_recovery_clearance"])) < EPS
            for exact, logged, threshold in [
                (entry, float(row["preview_clearance_lower_bound"]), entry_margin),
                (release, release_clearance(row), margin),
            ]:
                assert logged <= exact + EPS
                if abs(exact - threshold) > EPS and abs(logged - threshold) > EPS:
                    assert (exact > threshold) == (logged > threshold)
        yaw_error = math.atan2(
            math.sin(pose[2] - float(before["yaw"])),
            math.cos(pose[2] - float(before["yaw"])),
        )
        result.append(
            dict(
                pose_kind=name,
                pose_x=pose[0],
                pose_y=pose[1],
                pose_yaw=pose[2],
                position_error=math.dist(
                    pose[:2], (float(before["x"]), float(before["y"]))
                ),
                yaw_error_deg=math.degrees(abs(yaw_error)),
                stop_exact=stop,
                terminal_exact=terminal,
                entry_slack=entry - entry_margin,
                release_slack=release - margin,
                entry_clear=entry > entry_margin,
                release_clear=release > margin,
                entry_ambiguous=abs(entry - entry_margin) <= EPS,
                release_ambiguous=abs(release - margin) <= EPS,
                preview_steps=steps,
            )
        )
    return result


def tail_summary(track, end):
    stop = round(end / DT)
    start = max(0, stop - 1500)
    assert abs(float(track[stop]["time_s"]) - end) < 1e-8
    modes = Counter()
    counts = {route: Counter() for route in ["independent", "joint"]}
    runs = {route: Counter() for route in counts}
    maxima = {route: Counter() for route in counts}
    travel = 0.0
    distances = []
    goal = GOALS[track[0]["scenario"]]
    for row in track[start : stop + 1]:
        distances.append(
            math.hypot(float(row["x"]) - goal[0], float(row["y"]) - goal[1])
        )
    for i in range(start + 1, stop + 1):
        before, row = track[i - 1], track[i]
        travel += math.hypot(
            float(row["x"]) - float(before["x"]), float(row["y"]) - float(before["y"])
        )
        modes[command_mode(row)] += 1
        route, ready = release_inputs(before, row)
        for name in counts:
            if route != name:
                runs[name].clear()
        if route == "inactive":
            continue
        counts[route]["eligible"] += 1
        for requirement, passed in ready.items():
            counts[route][f"fail_{requirement}"] += not passed
        for mask, excluded in MASKS.items():
            passed = all(
                value
                for requirement, value in ready.items()
                if requirement not in excluded
            )
            runs[route][mask] = runs[route][mask] + 1 if passed else 0
            maxima[route][mask] = max(maxima[route][mask], runs[route][mask])
            counts[route][f"ready_{mask}"] += passed
    record = dict(
        window_start_s=float(track[start]["time_s"]),
        window_end_s=end,
        duration_s=round((stop - start) * DT, 6),
        travel=travel,
        wait_s=round(
            sum(modes[name] for name in ["both", "loss_only", "guard_only"]) * DT, 6
        ),
        initial_goal_distance=distances[0],
        final_goal_distance=distances[-1],
        minimum_goal_distance=min(distances),
        goal_progress=distances[0] - distances[-1],
        **{f"{name}_s": round(modes[name] * DT, 6) for name in MODES},
    )
    requirements = []
    for route in counts:
        requirements.append(
            dict(
                route=route,
                eligible_s=round(counts[route]["eligible"] * DT, 6),
                **{
                    f"{name}_fail_s": round(counts[route][f"fail_{name}"] * DT, 6)
                    for name in REQUIREMENTS
                },
                **{
                    f"{name}_ready_s": round(counts[route][f"ready_{name}"] * DT, 6)
                    for name in MASKS
                },
                **{
                    f"max_{name}_ready_s": round(maxima[route][name] * DT, 6)
                    for name in MASKS
                },
            )
        )
    return record, requirements


def selected_checkpoints(tracks, indices):
    common = min(len(track) for track in tracks) - 1
    selections = []
    for track in tracks:
        chosen = defaultdict(list)
        for name in ["gate", "evidence", "origin", "decisive", "action", "command"]:
            if indices.get(name) is not None:
                chosen[indices[name]].append("first_" + name)
        chosen[common].append("common_endpoint")
        chosen[len(track) - 1].append("own_endpoint")
        fresh = next(
            (
                i
                for i in range(max(1, len(track) - 1500), len(track))
                if abs(float(track[i]["measured_speed"])) <= 0.02
                and abs(float(track[i]["observation_age"]) - 0.5) < 1e-8
            ),
            None,
        )
        if fresh is not None:
            chosen[fresh].append("first_fresh_stationary_in_final_window")
        selections.append(chosen)
    return selections


def summarize(forks):
    rows = []
    for cohort in [*COHORTS, "all"]:
        for outcome in [
            "all",
            "held/held",
            "timeout/timeout",
            "held/timeout",
            "timeout/held",
        ]:
            group = [
                r
                for r in forks
                if (cohort == "all" or r["cohort"] == cohort)
                and (outcome == "all" or r["outcome"] == outcome)
            ]
            if not group:
                continue
            rows.append(
                dict(
                    cohort=cohort,
                    outcome=outcome,
                    pairs=len(group),
                    **{
                        f"{actor}_{kind}": sum(
                            r["first_actor"] == actor and r["kind"] == kind
                            for r in group
                        )
                        for actor in ["aligned", "epoch"]
                        for kind in ["entry", "independent_release", "joint_resume"]
                    },
                    history_only_release=sum(r["history_only_release"] for r in group),
                    fusion_unavailable_at_action=sum(
                        not r["fusion_available_at_action"] for r in group
                    ),
                    origin_position_better=sum(
                        r["origin_fused_position_error"]
                        < r["origin_original_position_error"] - 1e-9
                        for r in group
                    ),
                    origin_position_worse=sum(
                        r["origin_fused_position_error"]
                        > r["origin_original_position_error"] + 1e-9
                        for r in group
                    ),
                )
            )
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--raw-root",
        type=Path,
        help="Override directory containing the four pinned cohorts",
    )
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    args = parser.parse_args()
    assert __debug__, "Run without Python -O"
    for filename, expected in PINNED.items():
        assert digest(PRIOR / filename) == expected
    prior_sources = json.loads((PRIOR / "sources.json").read_text())
    assert digest(SIMULATOR) == prior_sources["analysis"]["simulator_sha256"]
    for filename, expected in prior_sources["analysis"]["helpers_sha256"].items():
        assert digest(ROOT / filename) == expected, filename
    cases = read_rows(PRIOR / "cases.csv")
    published = read_rows(PRIOR / "pairs.csv")
    selected = [r for r in published if r["first_command_change_s"]]
    assert len(published) == 324 and len(selected) == 121
    accounting = [
        dict(r, selected=bool(r["first_command_change_s"])) for r in published
    ]
    for r in accounting:
        if not r["selected"]:
            assert r["reference_status"] == r["candidate_status"]
            assert r["reference_time_s"] == r["candidate_time_s"]
    forks, prefixes, checkpoints, tails, requirements = [], [], [], [], []
    sources = dict(prior_sha256=PINNED, cohorts={})
    for cohort in COHORTS:
        pinned = prior_sources[cohort]
        run = (
            args.raw_root / cohort if args.raw_root else Path(pinned["path"])
        ).resolve()
        provenance = json.loads((run / "provenance.json").read_text())
        for field in [
            "variants",
            "seeds",
            "noise_correlation_time_s",
            "simulator_sha256",
            "inputs_sha256",
            "inputs_by_variant_sha256",
        ]:
            assert provenance[field] == pinned["provenance"][field]
        maps = defaultdict(list)
        for row in read_rows(run / "planning-inputs/fine-padding/obstacles.csv"):
            maps[row["scenario"]].append((float(row["x"]), float(row["y"])))
        assert (
            digest(run / "planning-inputs/fine-padding/obstacles.csv")
            == provenance["inputs_sha256"]["obstacles.csv"]
        )
        groups = defaultdict(list)
        for row in selected:
            if row["cohort"] == cohort:
                groups[row["mode"], row["seed"]].append(row)
        hashes = {}
        intervals = shared_steps = 0
        for (mode, seed), pairs in groups.items():
            tracks = []
            wanted = {(r["scenario"], r["planner"]) for r in pairs}
            for variant in ["aligned", "epoch"]:
                name = f"{variant}-{mode}-{seed}/trajectories.csv"
                assert digest(run / name) == pinned["raw_sha256"][name]
                hashes[name] = pinned["raw_sha256"][name]
                current = defaultdict(list)
                with (run / name).open() as stream:
                    for row in csv.DictReader(stream):
                        key = row["scenario"], row["planner"]
                        if key in wanted:
                            current[key].append(row)
                assert set(current) == wanted
                tracks.append(current)
            for pair in pairs:
                key = pair["scenario"], pair["planner"]
                current = [t[key] for t in tracks]
                indices, fork = first_fork(*current, pair["first_command_change_s"])
                identity = {k: pair[k] for k in KEY}
                outcome = pair["reference_status"] + "/" + pair["candidate_status"]
                forks.append(dict(**identity, outcome=outcome, **fork))
                shared_steps += indices["action"]
                for reason in [
                    "entry",
                    "gate",
                    "evidence",
                    "origin",
                    "decisive",
                    "action",
                    "command",
                ]:
                    index = indices[reason]
                    if index is None:
                        continue
                    for variant, track in zip(["aligned", "epoch"], current):
                        row, before = track[index], track[index - 1]
                        route, ready = release_inputs(before, row)
                        prefixes.append(
                            dict(
                                **identity,
                                variant=variant,
                                reason="first_" + reason,
                                time_s=row["time_s"],
                                route=route,
                                ready=bool(ready) and all(ready.values()),
                                failed="+".join(k for k, v in ready.items() if not v),
                                entry_clear=float(row["preview_clearance_lower_bound"])
                                > float(row["preview_margin"]),
                                release_clear=release_clearance(row)
                                > release_margin(row),
                                guard_evidence=row["guard_release_evidence"],
                                joint_evidence=row["recovery_evidence"],
                                guard_active=row["guard_active"],
                                guard_triggered=row["guard_triggered"],
                                guard_released=row["guard_released"],
                                resumed=row["resumed"],
                                fusion_available=row["epoch_fusion_available"],
                            )
                        )
                choices = selected_checkpoints(current, indices)
                for arm, (variant, track) in enumerate(
                    zip(["aligned", "epoch"], current)
                ):
                    case = next(
                        r
                        for r in cases
                        if r["variant"] == variant and all(r[k] == pair[k] for k in KEY)
                    )
                    intervals += len(track) - 1
                    for scope, end in [
                        ("common", min(float(t[-1]["time_s"]) for t in current)),
                        ("final", float(track[-1]["time_s"])),
                    ]:
                        summary, gates = tail_summary(track, end)
                        tail_identity = dict(
                            **identity,
                            variant=variant,
                            scope=scope,
                            outcome=outcome,
                            status=case["status"],
                        )
                        tails.append(dict(**tail_identity, **summary))
                        requirements.extend(
                            dict(**tail_identity, **row) for row in gates
                        )
                    for index, reasons in sorted(choices[arm].items()):
                        row, before = track[index], track[index - 1]
                        same = index < len(current[1 - arm]) and shared_inputs(
                            before,
                            row,
                            current[1 - arm][index - 1],
                            current[1 - arm][index],
                        )
                        for result in geometry(before, row, maps[key[0]]):
                            checkpoints.append(
                                dict(
                                    **identity,
                                    variant=variant,
                                    outcome=outcome,
                                    time_s=row["time_s"],
                                    reasons="+".join(reasons),
                                    paired_inputs_equal=same,
                                    fusion_available=row["epoch_fusion_available"],
                                    observation_age=row["observation_age"],
                                    measured_speed=row["measured_speed"],
                                    nominal_target_speed=row["nominal_target_speed"],
                                    nominal_steering=row["nominal_steering"],
                                    entry_margin=row["preview_margin"],
                                    release_margin=row["guard_release_margin"],
                                    **result,
                                )
                            )
            del tracks
        sources["cohorts"][cohort] = dict(
            path=str(run),
            provenance=provenance,
            provenance_sha256=digest(run / "provenance.json"),
            raw_sha256=hashes,
            selected_pairs=sum(len(g) for g in groups.values()),
            selected_intervals=intervals,
            shared_prefix_steps=shared_steps,
        )
        print(
            f"{cohort}: {sources['cohorts'][cohort]['selected_pairs']} pairs, {intervals} pinned intervals",
            flush=True,
        )
    sources["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        simulator_sha256=digest(SIMULATOR),
        protocol_sha256=digest(ROOT / "results/epoch-divergence/protocol.md"),
        helpers_sha256={
            name: digest(ROOT / name)
            for name in [
                "analyze_epoch_preview.py",
                "analyze_filter_stalls.py",
                "analyze_recovery_stalls.py",
                "verify_joint_recovery.py",
                "verify_stopping_preview.py",
                "summarize_stopping_preview.py",
            ]
        },
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for filename, rows in [
        ("accounting", accounting),
        ("forks", forks),
        ("prefixes", prefixes),
        ("geometry", checkpoints),
        ("tails", tails),
        ("requirements", requirements),
        ("summary", summarize(forks)),
    ]:
        save_rows(args.output / f"{filename}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")


if __name__ == "__main__":
    main()
