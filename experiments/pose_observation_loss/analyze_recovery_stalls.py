#!/usr/bin/env python3
"""Diagnose the recorded speed-cap study's lost holds; no policy is changed."""

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import tempfile

from summarize_stopping_preview import save_rows
from verify_stopping_preview import segment_distance

ROOT = Path(__file__).resolve().parent
MODES = [
    "both",
    "loss_only",
    "guard_only",
    "goal_stop",
    "speed_zero",
    "speed_limited",
    "ordinary",
]


def digest(path):
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(block)
    return sha.hexdigest()


def center_clearance(x, y, obstacles):
    return min(
        x, y, 50 - x, 50 - y, *(math.hypot(x - p[0], y - p[1]) for p in obstacles)
    )


def forward_clearance(x, y, yaw, obstacles):
    end = (x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw))
    return min(
        x,
        y,
        50 - x,
        50 - y,
        end[0],
        end[1],
        50 - end[0],
        50 - end[1],
        *(segment_distance(p, (x, y), end) for p in obstacles),
    )


def command_mode(row):
    loss, guard = row["safety_paused"] == "true", row["guard_active"] == "true"
    if loss and guard:
        return "both"
    if loss:
        return "loss_only"
    if guard:
        return "guard_only"
    if row["stop_latched"] == "true":
        return "goal_stop"
    if float(row["speed_cap"]) <= 1e-9:
        return "speed_zero"
    if float(row["speed_cap"]) < float(row["uncapped_target_speed"]) - 1e-8:
        return "speed_limited"
    return "ordinary"


def checkpoint(cohort, case, before, row, obstacles, reason):
    x, y = float(before["x"]), float(before["y"])
    observed = (
        x + float(row["noise_x"]) + float(row["bias_x"]),
        y + float(row["noise_y"]) + float(row["bias_y"]),
    )
    true_clearance = center_clearance(x, y, obstacles)
    sensed_clearance = center_clearance(*observed, obstacles)
    sigma = float(row["effective_position_sigma"])
    # Every preview contains its initial point: this is a necessary condition,
    # independent of the chosen future steering. Truth is used only offline.
    guard_min = 0.55 + 2 * math.sqrt(2) * sigma
    recovery_min = 0.6 + 2 * math.sqrt(2) * sigma
    assert float(row["preview_clearance_lower_bound"]) <= sensed_clearance + 2e-8
    rx, ry, yaw = (float(row[k]) for k in ["recovery_x", "recovery_y", "recovery_yaw"])
    end = (rx + 0.6 * math.cos(yaw), ry + 0.6 * math.sin(yaw))
    recovery_clearance = min(segment_distance(p, (rx, ry), end) for p in obstacles)
    assert abs(recovery_clearance - float(row["recovery_clearance"])) < 3e-9
    recovery_margin = recovery_min + 0.02 * float(row["observation_age"])
    bounds = all(recovery_margin <= p <= 50 - recovery_margin for p in [rx, ry, *end])
    recovery_ready = (
        float(row["observation_age"]) <= 1.5
        and abs(float(before["v"])) <= 0.02
        and int(row["recovery_sample_count"]) >= 10
        and recovery_clearance > recovery_margin
        and bounds
    )
    guard_ready = (
        abs(float(before["v"])) <= 0.02
        and float(row["preview_clearance_lower_bound"])
        > float(row["preview_margin"]) + 0.05
    )
    return dict(
        cohort=cohort,
        **{k: case[k] for k in ["mode", "seed", "scenario", "planner"]},
        reason=reason,
        before_x=x,
        before_y=y,
        before_yaw=before["yaw"],
        before_v=before["v"],
        true_center_clearance=true_clearance,
        sensed_center_clearance=sensed_clearance,
        true_inside_min_guard_release_margin=true_clearance <= guard_min,
        true_inside_min_recovery_margin=true_clearance <= recovery_min,
        true_forward_recovery_clearance=forward_clearance(
            x, y, float(before["yaw"]), obstacles
        ),
        min_guard_release_margin=guard_min,
        min_recovery_margin=recovery_min,
        recovery_ready=recovery_ready,
        guard_ready=guard_ready,
        **{
            k: row[k]
            for k in [
                "time_s",
                "v",
                "guard_active",
                "safety_paused",
                "stop_latched",
                "observation_age",
                "correction_received",
                "precision_active",
                "effective_position_sigma",
                "preview_clearance_lower_bound",
                "preview_margin",
                "nominal_target_speed",
                "nominal_steering",
                "speed_cap",
                "speed_free_distance",
                "guard_released",
                "resumed",
                "guard_release_evidence",
                "recovery_evidence",
                "recovery_clearance",
                "recovery_sample_count",
            ]
        },
    )


def analyze_case(cohort, case, reference, track, obstacles):
    assert case["status"] == "timeout" and float(case["time_s"]) == 180
    assert float(track[-1]["time_s"]) == 180
    pairs = [(a, b) for a, b in zip(track, track[1:]) if float(b["time_s"]) > 150]
    assert len(pairs) == 1500
    counts = {mode: 0 for mode in MODES}
    moving = fresh = 0
    releases = resumes = 0
    release_in_loss = resume_in_guard = release_zero = 0
    travel = 0.0
    samples = []
    for before, row in pairs:
        assert abs(float(row["time_s"]) - float(before["time_s"]) - 0.02) < 1e-8
        x, y, yaw, speed = (float(before[k]) for k in ["x", "y", "yaw", "v"])
        assert all(
            math.isfinite(float(row[k]))
            for k in ["x", "y", "yaw", "v", "acceleration", "steering"]
        )
        assert abs(float(row["x"]) - (x + speed * math.cos(yaw) * 0.02)) < 2e-9
        assert abs(float(row["y"]) - (y + speed * math.sin(yaw) * 0.02)) < 2e-9
        assert abs(float(row["v"]) - (speed + float(row["acceleration"]) * 0.02)) < 2e-9
        yaw_error = (
            yaw
            + speed / 0.4 * math.tan(float(row["steering"])) * 0.02
            - float(row["yaw"])
        )
        assert abs(math.atan2(math.sin(yaw_error), math.cos(yaw_error))) < 3e-9
        counts[command_mode(row)] += 1
        moving += abs(float(before["v"])) > 0.02
        fresh += float(row["observation_age"]) <= 1.5
        releases += row["guard_released"] == "true"
        resumes += row["resumed"] == "true"
        release_in_loss += (
            row["guard_released"] == "true" and row["safety_paused"] == "true"
        )
        release_zero += (
            row["guard_released"] == "true" and float(row["nominal_target_speed"]) == 0
        )
        resume_in_guard += row["resumed"] == "true" and row["guard_active"] == "true"
        travel += math.hypot(
            float(row["x"]) - float(before["x"]), float(row["y"]) - float(before["y"])
        )
        # Fixed observation ages, every gate release, and the final decision.
        reasons = []
        if (
            abs(float(row["observation_age"]) - 0.5) < 1e-8
            and abs(float(before["v"])) <= 0.02
        ):
            reasons.append("fresh_stationary")
        if row["guard_released"] == "true":
            reasons.append("guard_release")
        if row["resumed"] == "true":
            reasons.append("loss_resume")
        if float(row["time_s"]) == 180:
            reasons.append("final")
        if reasons:
            samples.append(
                checkpoint(cohort, case, before, row, obstacles, "+".join(reasons))
            )
    assert sum(counts.values()) == 1500 and samples
    final = samples[-1]
    record = dict(
        cohort=cohort,
        **{k: case[k] for k in ["mode", "seed", "scenario", "planner"]},
        reference_held_time_s=reference["time_s"],
        final_goal_distance=case["goal_distance"],
        first_goal_entry_s=case["first_goal_entry_s"],
        tail_start_s=150,
        tail_end_s=180,
        tail_travel_distance=travel,
        near_stationary=travel < 0.1,
        moving_time_s=round(moving * 0.02, 6),
        fresh_time_s=round(fresh * 0.02, 6),
        guard_releases=releases,
        loss_resumes=resumes,
        guard_releases_in_loss=release_in_loss,
        guard_releases_with_zero_target=release_zero,
        loss_resumes_in_guard=resume_in_guard,
        stop_union_time_s=round(
            (counts["both"] + counts["loss_only"] + counts["guard_only"]) * 0.02, 6
        ),
        true_final_clearance=center_clearance(
            float(track[-1]["x"]), float(track[-1]["y"]), obstacles
        ),
        true_final_forward_clearance=forward_clearance(
            float(track[-1]["x"]),
            float(track[-1]["y"]),
            float(track[-1]["yaw"]),
            obstacles,
        ),
        min_guard_release_margin=final["min_guard_release_margin"],
        min_recovery_margin=final["min_recovery_margin"],
        **{f"{mode}_time_s": round(count * 0.02, 6) for mode, count in counts.items()},
    )
    record["true_final_inside_min_guard_release_margin"] = (
        record["true_final_clearance"] <= record["min_guard_release_margin"]
    )
    record["true_final_forward_blocked_at_min_margin"] = (
        record["true_final_forward_clearance"] <= record["min_recovery_margin"]
    )
    return record, samples


def plot(directory, cases):
    with tempfile.TemporaryDirectory(prefix="recovery-stall-plot-") as cache:
        os.environ.setdefault("MPLCONFIGDIR", cache)
        os.environ.setdefault("XDG_CACHE_HOME", cache)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.rcParams["svg.fonttype"] = "none"
        plt.rcParams["svg.hashsalt"] = "recovery-stalls"
        fig, ax = plt.subplots(figsize=(13, 9), layout="constrained")
        styles = [
            ("both", "#6d28d9", "Loss + guard"),
            ("loss_only", "#ea580c", "Loss only"),
            ("guard_only", "#2563eb", "Guard only"),
            ("goal_stop", "#64748b", "Goal stop"),
            ("speed_zero", "#dc2626", "Speed cap zero"),
            ("speed_limited", "#16a34a", "Speed limited"),
            ("ordinary", "#cbd5e1", "Ordinary"),
        ]
        left = [0.0] * len(cases)
        for key, color, label in styles:
            values = [r[f"{key}_time_s"] for r in cases]
            if not any(values):
                continue
            ax.barh(range(len(cases)), values, left=left, color=color, label=label)
            left = [a + b for a, b in zip(left, values)]
        labels = [
            f'{"W" if r["cohort"]=="white-validation" else "C"} {r["mode"]:8} {r["seed"]} {r["scenario"].replace("_50x50","")} {r["planner"]}'
            for r in cases
        ]
        ax.set_yticks(range(len(cases)), labels, fontsize=9)
        for i, r in enumerate(cases):
            ax.text(
                30.4, i, f'{r["tail_travel_distance"]:.3f}', va="center", fontsize=9
            )
        ax.text(30.4, -1.2, "Travel", fontsize=9)
        ax.invert_yaxis()
        ax.set(
            xlim=(0, 34),
            xticks=[0, 5, 10, 15, 20, 25, 30],
            xlabel="Total seconds per command state (not a timeline); travel at right",
            title="20 lost holds: aggregated command durations in the final 30 s\nW: independent noise; C: correlation time 1 s",
        )
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=3, fontsize=9)
        svg = directory / "stall-command-states.svg"
        fig.savefig(svg, metadata={"Date": None})
        svg.write_text(
            "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(directory / "stall-command-states.png", dpi=160)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--white", type=Path, required=True)
    parser.add_argument("--correlated", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    parser.add_argument("--plot", action="store_true", help="Requires matplotlib")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    pinned = list(
        csv.DictReader((ROOT / "results/anticipatory-speed/all-results.csv").open())
    )
    pinned_sources = json.loads(
        (ROOT / "results/anticipatory-speed/sources.json").read_text()
    )
    cases, samples, sources = [], [], {}
    for cohort, run, tau in [
        ("white-validation", args.white, 0),
        ("correlated-validation", args.correlated, 1),
    ]:
        provenance = json.loads((run / "provenance.json").read_text())
        assert (
            provenance["variants"] == ["clearance", "paced"]
            and provenance["noise_correlation_time_s"] == tau
        )
        assert (
            provenance["simulator_sha256"]
            == pinned_sources[cohort]["provenance"]["simulator_sha256"]
        )
        rows = list(csv.DictReader((run / "all-results.csv").open()))
        expected = [
            {k: v for k, v in r.items() if k != "cohort"}
            for r in pinned
            if r["cohort"] == cohort
        ]
        assert rows == expected, "Run differs from the recorded study"
        inputs = run / "planning-inputs/fine-padding"
        assert (
            {p.name: digest(p) for p in inputs.glob("*.csv")}
            == provenance["inputs_by_variant_sha256"]["paced"]
            == provenance["inputs_by_variant_sha256"]["clearance"]
        )
        maps = {}
        for r in csv.DictReader((inputs / "obstacles.csv").open()):
            maps.setdefault(r["scenario"], []).append((float(r["x"]), float(r["y"])))
        lookup = {
            (r["variant"], r["mode"], r["seed"], r["scenario"], r["planner"]): r
            for r in rows
        }
        grouped = {}
        for r in rows:
            if r["variant"] != "paced":
                continue
            ref = lookup[
                ("clearance", r["mode"], r["seed"], r["scenario"], r["planner"])
            ]
            if ref["held"] == "true" and r["held"] == "false":
                grouped.setdefault((r["mode"], r["seed"]), []).append((r, ref))
        hashes = {"all-results.csv": digest(run / "all-results.csv")}
        for (mode, seed), selected in grouped.items():
            path = run / f"paced-{mode}-{seed}" / "trajectories.csv"
            hashes[str(path.relative_to(run))] = digest(path)
            tracks = {(r["scenario"], r["planner"]): [] for r, _ in selected}
            for row in csv.DictReader(path.open()):
                key = row["scenario"], row["planner"]
                if key in tracks and float(row["time_s"]) >= 150:
                    tracks[key].append(row)
            for r, ref in selected:
                record, checked = analyze_case(
                    cohort,
                    r,
                    ref,
                    tracks[r["scenario"], r["planner"]],
                    maps[r["scenario"]],
                )
                cases.append(record)
                samples.extend(checked)
        sources[cohort] = dict(provenance=provenance, source_sha256=hashes)
    assert cases and len(
        {
            tuple(r[k] for k in ["cohort", "mode", "seed", "scenario", "planner"])
            for r in cases
        }
    ) == len(cases)
    summary = []
    for cohort in ["white-validation", "correlated-validation"]:
        selected = [r for r in cases if r["cohort"] == cohort]
        summary.append(
            dict(
                cohort=cohort,
                lost_holds=len(selected),
                near_stationary=sum(r["near_stationary"] for r in selected),
                fully_stop_gated=sum(
                    r["stop_union_time_s"] >= 30 - 1e-8 for r in selected
                ),
                inside_min_guard_margin=sum(
                    r["true_final_inside_min_guard_release_margin"] for r in selected
                ),
                near_stationary_forward_blocked=sum(
                    r["near_stationary"]
                    and r["true_final_forward_blocked_at_min_margin"]
                    for r in selected
                ),
                mean_travel_distance=statistics.mean(
                    r["tail_travel_distance"] for r in selected
                ),
                min_fresh_time_s=min(r["fresh_time_s"] for r in selected),
            )
        )
    sources["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        selection_snapshot_sha256=digest(
            ROOT / "results/anticipatory-speed/all-results.csv"
        ),
        window_start_exclusive_s=150,
        window_end_inclusive_s=180,
        near_stationary_distance_threshold=0.1,
        physical_intervals_checked=1500 * len(cases),
        geometry_checkpoints=len(samples),
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, rows in [
        ("lost-holds.csv", cases),
        ("checkpoints.csv", samples),
        ("summary.csv", summary),
    ]:
        save_rows(args.output / name, rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    if args.plot:
        plot(args.output, cases)
    for row in summary:
        print(row)
    print(f"Audited {len(cases)} lost holds and {len(samples)} geometry checkpoints")


if __name__ == "__main__":
    main()
