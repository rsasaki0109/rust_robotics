#!/usr/bin/env python3
"""Trace clearance loss before the final stops of eight frozen joint cases."""

import argparse
from collections import defaultdict
import csv
import json
import math
import os
from pathlib import Path
import tempfile

from analyze_recovery_stalls import center_clearance, digest
from summarize_stopping_preview import save_rows
from verify_stopping_preview import exact_preview, segment_distance

ROOT = Path(__file__).resolve().parent
MARGIN = 0.6 + 2 * math.sqrt(2) * 0.1
DT = 0.02
FIELDS = ["mode", "seed", "scenario", "planner"]


class CenterIndex:
    """Exact point/boundary clearance using expanding unit-cell rings."""

    def __init__(self, obstacles):
        self.cells = defaultdict(list)
        for x, y in obstacles:
            self.cells[math.floor(x), math.floor(y)].append((x, y))

    def clearance(self, x, y):
        best = min(x, y, 50 - x, 50 - y)
        if best <= 0 or not self.cells:
            return best
        ix, iy = math.floor(x), math.floor(y)
        for ring in range(27):
            for bx in range(ix - ring, ix + ring + 1):
                for by in range(iy - ring, iy + ring + 1):
                    if ring and abs(bx - ix) < ring and abs(by - iy) < ring:
                        continue
                    for px, py in self.cells.get((bx, by), ()):
                        best = min(best, math.hypot(x - px, y - py))
            # Unvisited cells are outside this square; none can be closer.
            outside = min(
                x - (ix - ring), ix + ring + 1 - x, y - (iy - ring), iy + ring + 1 - y
            )
            if best <= outside:
                return best
        raise AssertionError("Boundary distance should terminate the search")


def braking(row):
    return any(
        row[k] == "true"
        for k in ["safety_paused", "guard_active", "stop_latched", "speed_braking"]
    )


def brake_rollout(pose, speed, steering):
    """Immediate bounded strong brake, holding this step's actual steering."""
    x, y, yaw = pose
    states = [(x, y, yaw, speed)]
    for _ in range(200):
        acceleration = max(-1, min(0, -4 * speed))
        x += speed * math.cos(yaw) * DT
        y += speed * math.sin(yaw) * DT
        yaw += speed / 0.4 * math.tan(steering) * DT
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        speed += acceleration * DT
        states.append((x, y, yaw, speed))
        if abs(speed) <= 0.001:
            break
    assert abs(speed) <= 0.001
    return states


def swept_clearance(states, obstacles):
    points = [s[:2] for s in states]
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    clearance = min(min(x, y, 50 - x, 50 - y) for x, y in points)
    for distance, point in sorted((math.dist(points[0], p), p) for p in obstacles):
        if distance - length >= clearance:
            break
        clearance = min(
            clearance,
            *(segment_distance(point, a, b) for a, b in zip(points, points[1:])),
        )
    return clearance, length


def build_signals(track, index):
    signals = [None]
    for before, row in zip(track, track[1:]):
        x, y, yaw, speed = (float(before[k]) for k in ["x", "y", "yaw", "v"])
        time = float(row["time_s"])
        assert float(row["effective_position_sigma"]) == 0.1
        assert (
            abs(
                float(row["preview_margin"])
                - (0.5 + 2 * math.sqrt(2) * 0.1 + 0.02 * float(row["observation_age"]))
            )
            < 3e-9
        )
        acceleration, steering = float(row["acceleration"]), float(row["steering"])
        assert abs(time - float(before["time_s"]) - DT) < 1e-8
        assert abs(float(row["x"]) - x - speed * math.cos(yaw) * DT) < 2e-9
        assert abs(float(row["y"]) - y - speed * math.sin(yaw) * DT) < 2e-9
        assert abs(float(row["v"]) - speed - acceleration * DT) < 2e-9
        dyaw = yaw + speed / 0.4 * math.tan(steering) * DT - float(row["yaw"])
        assert abs(math.atan2(math.sin(dyaw), math.cos(dyaw))) < 3e-9
        strong = braking(row)
        expected = (
            max(-1, min(0, -4 * speed))
            if strong
            else float(row["target_speed"]) - speed
        )
        assert abs(acceleration - expected) < 4e-9
        observed = tuple(
            float(before[k]) + float(row[f"noise_{k}"]) + float(row[f"bias_{k}"])
            for k in ["x", "y", "yaw"]
        )
        assert all(
            abs(observed[a] - float(row[f"preview_pose_{k}"])) < 3e-9
            for a, k in enumerate(["x", "y", "yaw"])
        )
        signals.append(
            dict(
                time_s=time,
                true_before=index.clearance(x, y),
                true_after=index.clearance(float(row["x"]), float(row["y"])),
                observed_before=index.clearance(*observed[:2]),
                speed_before=speed,
                speed_after=float(row["v"]),
                strong_brake=strong,
                guard_active=row["guard_active"],
                safety_paused=row["safety_paused"],
                speed_braking=row["speed_braking"],
                guard_triggered=row["guard_triggered"],
                observation_age=float(row["observation_age"]),
                position_error=math.hypot(observed[0] - x, observed[1] - y),
            )
        )
    return signals


def forecast(track, signals, step, obstacles, identity, reason):
    before, row = track[step - 1], track[step]
    pose = tuple(float(before[k]) for k in ["x", "y", "yaw"])
    observed = tuple(
        pose[a] + float(row[f"noise_{k}"]) + float(row[f"bias_{k}"])
        for a, k in enumerate(["x", "y", "yaw"])
    )
    speed, steering = float(before["v"]), float(row["steering"])
    true_states = brake_rollout(pose, speed, steering)
    sensed_states = brake_rollout(observed, speed, steering)
    true_clearance, distance = swept_clearance(true_states, obstacles)
    sensed_clearance, _ = swept_clearance(sensed_states, obstacles)
    exact, _ = exact_preview(before, row, obstacles)
    true_row = dict(
        row, **{f"preview_pose_{k}": pose[a] for a, k in enumerate(["x", "y", "yaw"])}
    )
    true_preview, _ = exact_preview(before, true_row, obstacles)
    lower, margin = float(row["preview_clearance_lower_bound"]), float(
        row["preview_margin"]
    )
    assert lower <= exact + 2e-8
    for threshold in [margin, margin + 0.05]:
        if abs(exact - threshold) > 2e-8 and abs(lower - threshold) > 2e-8:
            assert (lower > threshold) == (exact > threshold)
    # Match the counterfactual to the real trajectory only while the actual
    # brake mode and steering remain the same. Later commands are not assumed.
    prefix = 0
    for offset, state in enumerate(true_states[1:]):
        if step + offset >= len(track):
            break
        actual = track[step + offset]
        if not braking(actual) or abs(float(actual["steering"]) - steering) > 1e-9:
            break
        assert (
            max(abs(state[a] - float(actual[k])) for a, k in enumerate(["x", "y"]))
            < 1e-7
        )
        assert abs(state[3] - float(actual["v"])) < 1e-7
        angle = state[2] - float(actual["yaw"])
        assert abs(math.atan2(math.sin(angle), math.cos(angle))) < 1e-7
        prefix += 1
    return dict(
        **identity,
        reason=reason,
        **signals[step],
        steering=steering,
        target_speed=row["target_speed"],
        nominal_target_speed=row["nominal_target_speed"],
        nominal_steering=row["nominal_steering"],
        guard_margin=margin,
        logged_preview_lower=lower,
        observed_nominal_preview=exact,
        true_nominal_preview=true_preview,
        true_immediate_brake_clearance=true_clearance,
        true_immediate_brake_lower=true_clearance - abs(true_states[-1][3]) / 4,
        observed_immediate_brake_clearance=sensed_clearance,
        observed_immediate_brake_lower=sensed_clearance - abs(sensed_states[-1][3]) / 4,
        immediate_brake_distance=distance,
        immediate_brake_horizon_s=(len(true_states) - 1) * DT,
        matching_actual_brake_steps=prefix,
        true_brake_end_center=center_clearance(*true_states[-1][:2], obstacles),
    )


def analyze_case(track, obstacles, identity):
    index = CenterIndex(obstacles)
    signals = build_signals(track, index)
    assert len(track) == 9001 and float(track[-1]["time_s"]) == 180
    crossings = [
        i
        for i in range(1, len(track))
        if signals[i]["true_before"] > MARGIN and signals[i]["true_after"] <= MARGIN
    ]
    persistent = crossings[-1] if signals[-1]["true_after"] <= MARGIN else None
    # Last contiguous interval at measured speed <= 0.02, ending at t=180.
    moving = [i for i, row in enumerate(track) if abs(float(row["v"])) > 0.02]
    arrest = moving[-1] + 1
    assert arrest < len(track)
    brake_start = arrest if signals[arrest]["strong_brake"] else None
    if brake_start:
        while brake_start > 1 and signals[brake_start - 1]["strong_brake"]:
            brake_start -= 1
    selected = defaultdict(list)
    for step, reason in [
        (crossings[0] if crossings else None, "first_center_loss"),
        (persistent, "persistent_center_loss"),
        (
            max(1, persistent - 50) if persistent else None,
            "one_second_before_persistent_loss",
        ),
        (brake_start, "terminal_brake_start"),
        (arrest, "terminal_stationary"),
    ]:
        if step is not None:
            selected[step].append(reason)
    checkpoints = [
        forecast(track, signals, i, obstacles, identity, "+".join(reasons))
        for i, reasons in sorted(selected.items())
    ]
    audits = []
    for i in sorted(set(range(50, len(track), 50)) | set(selected)):
        before, row = track[i - 1], track[i]
        poses = [
            tuple(float(row[k]) for k in ["x", "y"]),
            tuple(
                float(before[k]) + float(row[f"noise_{k}"]) + float(row[f"bias_{k}"])
                for k in ["x", "y"]
            ),
        ]
        for kind, pose, recorded in zip(
            ["truth_after", "observed_before"],
            poses,
            [signals[i]["true_after"], signals[i]["observed_before"]],
        ):
            full = center_clearance(*pose, obstacles)
            assert abs(full - recorded) < 1e-12
            audits.append(
                dict(
                    **identity,
                    time_s=signals[i]["time_s"],
                    pose_kind=kind,
                    indexed_clearance=recorded,
                    full_scan_clearance=full,
                )
            )
    stop = next((r for r in checkpoints if "terminal_brake_start" in r["reason"]), None)
    record = dict(
        **identity,
        center_crossings=len(crossings),
        first_center_loss_s=signals[crossings[0]]["time_s"] if crossings else "",
        persistent_center_loss_s=signals[persistent]["time_s"] if persistent else "",
        terminal_brake_start_s=signals[brake_start]["time_s"] if brake_start else "",
        terminal_stationary_s=signals[arrest]["time_s"],
        terminal_stationary_duration_s=round((len(track) - 1 - arrest) * DT, 6),
        final_center_clearance=signals[-1]["true_after"],
        terminal_brake_before_center_loss=(
            brake_start <= persistent if brake_start and persistent else ""
        ),
        brake_start_true_center=stop["true_before"] if stop else "",
        brake_start_observed_center=stop["observed_before"] if stop else "",
        brake_start_speed=stop["speed_before"] if stop else "",
        true_immediate_brake_lower=stop["true_immediate_brake_lower"] if stop else "",
        observed_immediate_brake_lower=(
            stop["observed_immediate_brake_lower"] if stop else ""
        ),
        true_immediate_brake_clearance=(
            stop["true_immediate_brake_clearance"] if stop else ""
        ),
        observed_immediate_brake_clearance=(
            stop["observed_immediate_brake_clearance"] if stop else ""
        ),
    )
    traces = [
        dict(
            **identity,
            **s,
            relative_to_arrest_s=round(s["time_s"] - signals[arrest]["time_s"], 6),
        )
        for i, s in enumerate(signals[1:], 1)
        if i % 5 == 0
        or i in selected
        or abs(s["time_s"] - signals[arrest]["time_s"]) <= 5
    ]
    return record, checkpoints, audits, traces


def plot(directory, cases, traces):
    with tempfile.TemporaryDirectory(prefix="stop-entry-") as cache:
        os.environ.setdefault("MPLCONFIGDIR", cache)
        os.environ.setdefault("XDG_CACHE_HOME", cache)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.rcParams.update({"svg.fonttype": "none", "svg.hashsalt": "stop-entry"})
        fig, axes = plt.subplots(2, 4, figsize=(15, 7), layout="constrained")
        for ax, case in zip(axes.flat, cases):
            chosen = [
                r
                for r in traces
                if all(r[k] == case[k] for k in ["cohort", *FIELDS])
                and -5 <= r["relative_to_arrest_s"] <= 2
            ]
            t = [r["relative_to_arrest_s"] for r in chosen]
            ax.plot(
                t,
                [r["true_before"] for r in chosen],
                color="#111827",
                label="True center clearance",
            )
            ax.plot(
                t,
                [r["observed_before"] for r in chosen],
                color="#2563eb",
                alpha=0.5,
                linewidth=0.7,
                label="Observed center clearance",
            )
            ax.axhline(
                MARGIN, color="#dc2626", linestyle="--", label="Recovery margin (age 0)"
            )
            ax.axvline(0, color="#94a3b8", linestyle=":")
            speed_ax = ax.twinx()
            speed_ax.plot(
                t,
                [r["speed_before"] for r in chosen],
                color="#9333ea",
                linestyle="--",
                linewidth=1,
                label="Measured speed",
            )
            speed_ax.set(ylim=(0, 1.1), ylabel="Speed")
            ax.set(
                xlim=(-5, 2),
                ylim=(0, 2),
                xlabel="Seconds relative to final stationary interval",
                ylabel="Center clearance",
                title=f'{"W" if case["cohort"]=="known-white" else "C"} {case["mode"]} {case["seed"]}\n{case["scenario"].replace("_50x50","")} {case["planner"]}',
            )
        handles, labels = axes.flat[-1].get_legend_handles_labels()
        h, l = speed_ax.get_legend_handles_labels()
        fig.legend(
            handles + h, labels + l, loc="outside lower center", ncol=4, fontsize=9
        )
        fig.suptitle(
            "Eight frozen stalls: clearance before the final stationary interval\nTimes denote logged decisions; clearance and speed are before the physical update"
        )
        svg = directory / "stop-entry.svg"
        fig.savefig(svg, metadata={"Date": None})
        svg.write_text(
            "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(directory / "stop-entry.png", dpi=160)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--known-white", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    frozen = ROOT / "results/recovery-arcs/sources.json"
    prior = json.loads(frozen.read_text())
    selection = ROOT / "results/recovery-stalls/lost-holds.csv"
    assert digest(selection) == prior["analysis"]["selection_sha256"]
    selected = [
        r for r in csv.DictReader(selection.open()) if r["near_stationary"] == "True"
    ]
    cases, checkpoints, audits, traces, sources = [], [], [], [], {}
    for cohort, run, tau in [
        ("known-white", args.known_white, 0),
        ("reference", args.reference, 1),
    ]:
        info = json.loads((run / "provenance.json").read_text())
        assert (
            info["variants"] == ["paced", "joint"]
            and info["noise_correlation_time_s"] == tau
        )
        assert (
            info["simulator_sha256"] == prior[cohort]["provenance"]["simulator_sha256"]
        )
        assert (
            digest(run / "all-results.csv")
            == prior[cohort]["source_sha256"]["all-results.csv"]
        )
        inputs = run / "planning-inputs/fine-padding"
        assert (
            {p.name: digest(p) for p in inputs.glob("*.csv")}
            == prior[cohort]["provenance"]["inputs_by_variant_sha256"]["joint"]
            == info["inputs_by_variant_sha256"]["joint"]
        )
        maps = {}
        for row in csv.DictReader((inputs / "obstacles.csv").open()):
            maps.setdefault(row["scenario"], []).append(
                (float(row["x"]), float(row["y"]))
            )
        grouped = defaultdict(list)
        for case in selected:
            if (case["cohort"] == "white-validation") == (cohort == "known-white"):
                grouped[case["mode"], case["seed"]].append(case)
        hashes = {}
        for (mode, seed), chosen in grouped.items():
            path = run / f"joint-{mode}-{seed}/trajectories.csv"
            sha = digest(path)
            relative = str(path.relative_to(run))
            assert sha == prior[cohort]["source_sha256"][relative]
            hashes[relative] = sha
            tracks = {(r["scenario"], r["planner"]): [] for r in chosen}
            for row in csv.DictReader(path.open()):
                key = row["scenario"], row["planner"]
                if key in tracks:
                    tracks[key].append(row)
            for case in chosen:
                identity = dict(cohort=cohort, **{k: case[k] for k in FIELDS})
                record, points, checked, series = analyze_case(
                    tracks[case["scenario"], case["planner"]],
                    maps[case["scenario"]],
                    identity,
                )
                cases.append(record)
                checkpoints.extend(points)
                audits.extend(checked)
                traces.extend(series)
        sources[cohort] = dict(provenance=info, trajectory_sha256=hashes)
    assert len(cases) == 8
    sources["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        frozen_source_sha256=digest(frozen),
        fresh_recovery_margin=MARGIN,
        physical_intervals_checked=9000 * len(cases),
        index_audits=len(audits),
        preloss_lookback_s=1.0,
        brake_rollout_cutoff_speed=0.001,
        brake_residual_travel_bound="abs(v_end)/4",
        trace_sampling_s=0.1,
        full_trace_window_about_stationary_s=5.0,
        helper_sha256={
            n: digest(ROOT / n)
            for n in [
                "analyze_recovery_stalls.py",
                "summarize_stopping_preview.py",
                "verify_stopping_preview.py",
                "test_stop_entry_geometry.py",
            ]
        },
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, rows in [
        ("cases.csv", cases),
        ("checkpoints.csv", checkpoints),
        ("center-audit.csv", audits),
        ("traces.csv", traces),
    ]:
        save_rows(args.output / name, rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    if args.plot:
        plot(args.output, cases, traces)
    for r in cases:
        print(r)
    print(
        f"Checked {len(cases)} cases, {len(checkpoints)} forecasts and {len(audits)} full-scan center audits"
    )


if __name__ == "__main__":
    main()
