#!/usr/bin/env python3
"""Audit counterfactual forward arcs for eight frozen joint-recovery stalls."""

import argparse
from collections import deque
import csv
import json
import math
import os
from pathlib import Path
import tempfile

from analyze_recovery_stalls import digest
from summarize_stopping_preview import save_rows
from verify_stopping_preview import segment_distance

ROOT = Path(__file__).resolve().parent
FIELDS = ["mode", "seed", "scenario", "planner"]
LENGTH = 0.6
WHEELBASE = 0.4
ANGLES = list(range(-35, 36))
# A point on an arc is integral_0^s (cos(yaw+k*u), sin(yaw+k*u)) du.
# Its sensitivity to k is <= s^2/2. Nearest grid steering is <= 0.5 degree
# away and |dk/ddelta| <= sec(35 deg)^2 / wheelbase over the entire interval.
GRID_ERROR_BOUND = (
    LENGTH**2 / 2 / (WHEELBASE * math.cos(math.radians(35)) ** 2) * math.radians(0.5)
)


def arc_point(pose, steering, length):
    x, y, yaw = pose
    half_angle = math.tan(steering) / WHEELBASE * length / 2
    distance = length * (math.sin(half_angle) / half_angle if half_angle else 1)
    return x + distance * math.cos(yaw + half_angle), y + distance * math.sin(
        yaw + half_angle
    )


def arc_clearance(pose, steering, obstacles):
    """Exact center-to-point and map-boundary minimum on a forward 0.6 arc."""
    x, y, yaw = pose
    start, end = (x, y), arc_point(pose, steering, LENGTH)
    curvature = math.tan(steering) / WHEELBASE
    if curvature == 0:
        return min(
            *start,
            *end,
            50 - x,
            50 - y,
            50 - end[0],
            50 - end[1],
            *(segment_distance(p, start, end) for p in obstacles),
        )
    sign = 1 if curvature > 0 else -1
    extrema = [start, end]
    # Derivatives of x and y vanish at cardinal tangent headings.
    for heading in [0, math.pi / 2, math.pi, 3 * math.pi / 2]:
        progress = ((heading - yaw) * sign) % math.tau
        if progress <= abs(curvature) * LENGTH:
            extrema.append(arc_point(pose, steering, progress / abs(curvature)))
    clearance = min(min(px, py, 50 - px, 50 - py) for px, py in extrema)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    for point in obstacles:
        dx, dy = point[0] - x, point[1] - y
        forward, left = dx * cosine + dy * sine, -dx * sine + dy * cosine
        # Signed angular projection onto the circle, expressed in local
        # coordinates so near-straight arcs do not subtract huge radii.
        progress = math.atan2(curvature * forward, 1 - curvature * left) / curvature
        if 0 <= progress <= LENGTH:
            norm = math.hypot(curvature * forward, 1 - curvature * left)
            distance = abs(curvature * (forward**2 + left**2) - 2 * left) / (norm + 1)
        else:
            distance = min(math.dist(point, start), math.dist(point, end))
        clearance = min(clearance, distance)
    return clearance


def chord_check(pose, steering, obstacles, exact):
    """Independent midpoint integration and polyline distances, bounded by sagitta."""
    curvature = math.tan(steering) / WHEELBASE
    n = 120
    ds = LENGTH / n
    # Incremental midpoint integration constructs 120 chords independently of
    # the analytic radial projection and wall extrema in arc_clearance.
    half_angle = curvature * ds / 2
    chord = ds * (math.sin(half_angle) / half_angle if half_angle else 1)
    x, y, yaw = pose
    points = [(x, y)]
    for _ in range(n):
        x += chord * math.cos(yaw + half_angle)
        y += chord * math.sin(yaw + half_angle)
        yaw += 2 * half_angle
        points.append((x, y))
    clearance = min(min(px, py, 50 - px, 50 - py) for px, py in points)
    # Exact pruning for the polyline: farther obstacles cannot reduce its min.
    for distance, point in sorted((math.dist(points[0], p), p) for p in obstacles):
        if distance - LENGTH >= clearance:
            break
        clearance = min(
            clearance,
            *(segment_distance(point, a, b) for a, b in zip(points, points[1:])),
        )
    sagitta = (1 - math.cos(half_angle)) / abs(curvature) if curvature else 0
    error = abs(clearance - exact)
    assert error <= sagitta + 1e-9, (pose, steering, clearance, exact, sagitta)
    return error, sagitta


def status(best, upper, margin):
    if best > margin + 1e-9:
        return "grid_pass"
    return "all_fixed_steering_blocked" if upper <= margin else "unresolved"


def inspect_pose(identity, pose, row, kind, obstacles):
    values = [(a, arc_clearance(pose, math.radians(a), obstacles)) for a in ANGLES]
    maximum = max(c for _, c in values)
    # Prefer a small steering angle when minima coincide at the starting point.
    best_angle, best = min(
        (p for p in values if p[1] >= maximum - 1e-10), key=lambda p: (abs(p[0]), p[0])
    )
    center = min(
        *pose[:2],
        50 - pose[0],
        50 - pose[1],
        *(math.dist(pose[:2], p) for p in obstacles),
    )
    upper = min(center, maximum + GRID_ERROR_BOUND) + 1e-9
    sigma, age = float(row["effective_position_sigma"]), float(row["observation_age"])
    margin = 0.6 + 2 * math.sqrt(2) * sigma
    nominal = float(row["nominal_steering"])
    record = dict(
        **identity,
        pose_kind=kind,
        x=pose[0],
        y=pose[1],
        yaw=pose[2],
        center_clearance=center,
        straight_clearance=dict(values)[0],
        nominal_steering_deg=math.degrees(nominal),
        nominal_clearance=arc_clearance(pose, nominal, obstacles),
        best_steering_deg=best_angle,
        best_clearance=best,
        grid_max_clearance=maximum,
        continuous_clearance_upper=upper,
        physical_grid_passes=sum(c > 0.5 for _, c in values),
        buffer_grid_passes=sum(c > 0.6 for _, c in values),
        recovery_zero_age_grid_passes=sum(c > margin for _, c in values),
        recovery_current_age_grid_passes=sum(
            c > margin + 0.02 * age for _, c in values
        ),
        start_blocks_recovery_zero_age=center <= margin,
        recovery_zero_age_status=status(maximum, upper, margin),
        recovery_current_age_status=status(maximum, upper, margin + 0.02 * age),
        sigma_grid_limit=(maximum - 0.6) / (2 * math.sqrt(2)),
        sigma_continuous_upper_limit=(upper - 0.6) / (2 * math.sqrt(2)),
        observation_age=age,
        effective_position_sigma=sigma,
        recovery_zero_age_margin=margin,
        recovery_current_age_margin=margin + 0.02 * age,
        recovery_sample_count=row["recovery_sample_count"],
        safety_paused=row["safety_paused"],
        guard_active=row["guard_active"],
    )
    # All final-pose grid arcs; straight/best/current-command arcs at every checkpoint.
    selected = (
        values
        if identity["reason"] == "final"
        else [(0, dict(values)[0]), (best_angle, best)]
    )
    selected = [(math.radians(a), c) for a, c in selected] + [
        (nominal, record["nominal_clearance"])
    ]
    audits = []
    for angle, exact in dict(selected).items():
        error, bound = chord_check(pose, angle, obstacles, exact)
        audits.append(
            dict(
                **identity,
                pose_kind=kind,
                steering_deg=math.degrees(angle),
                exact_clearance=exact,
                chord_error=error,
                sagitta_bound=bound,
            )
        )
    curves = [
        dict(**identity, pose_kind=kind, steering_deg=a, clearance=c) for a, c in values
    ]
    assert maximum <= center + 1e-9
    return record, curves, audits


def selected_checkpoints(track):
    samples = deque(maxlen=25)
    for before, row in zip(track, track[1:]):
        t, speed, age = (
            float(row["time_s"]),
            float(before["v"]),
            float(row["observation_age"]),
        )
        observed = tuple(
            float(before[k]) + float(row[f"noise_{k}"]) + float(row[f"bias_{k}"])
            for k in ["x", "y", "yaw"]
        )
        missing = row["correction"] == "true" and row["correction_received"] == "false"
        if (
            row["correction_received"] == "true"
            or row["precision_activation"] == "true"
            or missing
            or abs(speed) > 0.02
            or age > 1.5
        ):
            samples.clear()
        if (
            (before["safety_paused"] == "true" or missing)
            and abs(speed) <= 0.02
            and age <= 1.5
        ):
            samples.append(observed)
        if t <= 150:
            continue
        assert len(samples) == int(row["recovery_sample_count"])
        mean = None
        if samples:
            mean = (
                sum(p[0] for p in samples) / len(samples),
                sum(p[1] for p in samples) / len(samples),
                math.atan2(
                    sum(math.sin(p[2]) for p in samples),
                    sum(math.cos(p[2]) for p in samples),
                ),
            )
            assert (
                math.dist(
                    mean[:2], (float(row["recovery_x"]), float(row["recovery_y"]))
                )
                < 3e-9
            )
            yaw_error = mean[2] - float(row["recovery_yaw"])
            assert abs(math.atan2(math.sin(yaw_error), math.cos(yaw_error))) < 3e-9
        fixed_fresh = (
            abs(t % 5 - 0.5) < 1e-8 and abs(age - 0.5) < 1e-8 and abs(speed) <= 0.02
        )
        if t == 180 or fixed_fresh:
            poses = [
                ("truth", tuple(float(before[k]) for k in ["x", "y", "yaw"])),
                ("raw_observed", observed),
            ]
            if len(samples) >= 10:
                poses.append(("recovery_mean", mean))
            yield row, "final" if t == 180 else "fresh_stationary", poses


def check_tail(track, result):
    tail = [(a, b) for a, b in zip(track, track[1:]) if float(b["time_s"]) > 150]
    assert len(tail) == 1500
    travel = 0.0
    for before, row in tail:
        x, y, yaw, v = (float(before[k]) for k in ["x", "y", "yaw", "v"])
        assert all(
            math.isfinite(float(row[k]))
            for k in ["x", "y", "yaw", "v", "acceleration", "steering"]
        )
        assert abs(float(row["time_s"]) - float(before["time_s"]) - 0.02) < 1e-8
        assert abs(float(row["x"]) - x - v * math.cos(yaw) * 0.02) < 2e-9
        assert abs(float(row["y"]) - y - v * math.sin(yaw) * 0.02) < 2e-9
        assert abs(float(row["v"]) - v - float(row["acceleration"]) * 0.02) < 2e-9
        delta = (
            yaw
            + v / WHEELBASE * math.tan(float(row["steering"])) * 0.02
            - float(row["yaw"])
        )
        assert abs(math.atan2(math.sin(delta), math.cos(delta))) < 3e-9
        travel += math.hypot(float(row["x"]) - x, float(row["y"]) - y)
    assert travel < 0.1
    goal = (48, 48) if result["scenario"] == "dense_50x50" else (35, 45)
    final = track[-1]
    assert (
        abs(
            math.hypot(float(final["x"]) - goal[0], float(final["y"]) - goal[1])
            - float(result["goal_distance"])
        )
        < 2e-9
    )
    assert final["v"] == result["final_speed"]


def plot(directory, curves, records):
    with tempfile.TemporaryDirectory(prefix="recovery-arcs-") as cache:
        os.environ.setdefault("MPLCONFIGDIR", cache)
        os.environ.setdefault("XDG_CACHE_HOME", cache)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.rcParams.update({"svg.fonttype": "none", "svg.hashsalt": "recovery-arcs"})
        finals = [
            r for r in records if r["reason"] == "final" and r["pose_kind"] == "truth"
        ]
        fig, axes = plt.subplots(
            2, 4, figsize=(14, 7), sharex=True, sharey=True, layout="constrained"
        )
        for ax, r in zip(axes.flat, finals):
            selected = [
                c
                for c in curves
                if all(c[k] == r[k] for k in ["cohort", *FIELDS, "time_s", "pose_kind"])
            ]
            ax.plot(
                [c["steering_deg"] for c in selected],
                [c["clearance"] for c in selected],
                color="#2563eb",
                label="Forward arc (0.6)",
            )
            for margin, color, style, label in [
                (0.5, "#64748b", ":", "Physical radius"),
                (0.6, "#a16207", ":", "Radius + buffer"),
                (
                    r["recovery_zero_age_margin"],
                    "#dc2626",
                    "--",
                    "Recovery margin (age 0)",
                ),
            ]:
                ax.axhline(
                    margin, color=color, linestyle=style, linewidth=1, label=label
                )
            ax.scatter([0], [r["straight_clearance"]], color="#2563eb", s=16)
            ax.set(
                title=f'{"W" if r["cohort"] == "known-white" else "C"} {r["mode"]} {r["seed"]}\n{r["scenario"].replace("_50x50", "")} {r["planner"]}',
                xlim=(-35, 35),
                ylim=(0, 1),
                xlabel="Steering (degrees)",
                ylabel="Minimum center clearance",
            )
        handles, labels = axes.flat[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc="outside lower center", ncol=4, fontsize=9)
        fig.suptitle(
            "Eight frozen stalls: forward-arc geometry from true pose before t=180\nOffline counterfactual; fixed sigma 0.1, not a recovery controller"
        )
        svg = directory / "recovery-arcs.svg"
        fig.savefig(svg, metadata={"Date": None})
        svg.write_text(
            "\n".join(s.rstrip() for s in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(directory / "recovery-arcs.png", dpi=160)
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
    snapshot = ROOT / "results/joint-recovery"
    pinned = list(csv.DictReader((snapshot / "all-results.csv").open()))
    sources = json.loads((snapshot / "sources.json").read_text())
    selection = ROOT / "results/recovery-stalls/lost-holds.csv"
    cases = [
        r for r in csv.DictReader(selection.open()) if r["near_stationary"] == "True"
    ]
    assert len(cases) == 8
    records, curves, audits, provenance = [], [], [], {}
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
            info["simulator_sha256"]
            == sources[cohort]["provenance"]["simulator_sha256"]
        )
        assert list(csv.DictReader((run / "all-results.csv").open())) == [
            {k: v for k, v in r.items() if k != "cohort"}
            for r in pinned
            if r["cohort"] == cohort
        ]
        inputs = run / "planning-inputs/fine-padding"
        assert (
            {p.name: digest(p) for p in inputs.glob("*.csv")}
            == info["inputs_by_variant_sha256"]["joint"]
            == sources[cohort]["provenance"]["inputs_by_variant_sha256"]["joint"]
        )
        maps = {}
        for r in csv.DictReader((inputs / "obstacles.csv").open()):
            maps.setdefault(r["scenario"], []).append((float(r["x"]), float(r["y"])))
        grouped = {}
        for c in cases:
            if (c["cohort"] == "white-validation") == (cohort == "known-white"):
                grouped.setdefault((c["mode"], c["seed"]), []).append(c)
        hashes = {"all-results.csv": digest(run / "all-results.csv")}
        for (mode, seed), selected in grouped.items():
            path = run / f"joint-{mode}-{seed}/trajectories.csv"
            hashes[str(path.relative_to(run))] = digest(path)
            tracks = {(r["scenario"], r["planner"]): [] for r in selected}
            for r in csv.DictReader(path.open()):
                key = r["scenario"], r["planner"]
                if key in tracks and float(r["time_s"]) >= 149.5:
                    tracks[key].append(r)
            for case in selected:
                key = case["scenario"], case["planner"]
                track = tracks[key]
                assert float(track[-1]["time_s"]) == 180
                result = next(
                    r
                    for r in pinned
                    if r["cohort"] == cohort
                    and r["variant"] == "joint"
                    and all(r[k] == case[k] for k in FIELDS)
                )
                assert result["status"] == "timeout" and result["collision"] == "false"
                check_tail(track, result)
                for row, reason, poses in selected_checkpoints(track):
                    identity = dict(
                        cohort=cohort,
                        **{k: case[k] for k in FIELDS},
                        time_s=row["time_s"],
                        reason=reason,
                    )
                    for kind, pose in poses:
                        record, grid, checked = inspect_pose(
                            identity, pose, row, kind, maps[key[0]]
                        )
                        records.append(record)
                        curves.extend(grid)
                        audits.extend(checked)
        provenance[cohort] = dict(provenance=info, source_sha256=hashes)
    summary = []
    for reason in ["final", "fresh_stationary"]:
        for kind in ["truth", "raw_observed", "recovery_mean"]:
            chosen = [
                r for r in records if r["reason"] == reason and r["pose_kind"] == kind
            ]
            if chosen:
                summary.append(
                    dict(
                        reason=reason,
                        pose_kind=kind,
                        checkpoints=len(chosen),
                        start_blocked=sum(
                            r["start_blocks_recovery_zero_age"] for r in chosen
                        ),
                        straight_passes=sum(
                            r["straight_clearance"] > r["recovery_zero_age_margin"]
                            for r in chosen
                        ),
                        grid_passes=sum(
                            r["recovery_zero_age_status"] == "grid_pass" for r in chosen
                        ),
                        all_fixed_steering_blocked=sum(
                            r["recovery_zero_age_status"]
                            == "all_fixed_steering_blocked"
                            for r in chosen
                        ),
                        unresolved=sum(
                            r["recovery_zero_age_status"] == "unresolved"
                            for r in chosen
                        ),
                    )
                )
    provenance["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        selection_sha256=digest(selection),
        snapshot_sha256=digest(snapshot / "all-results.csv"),
        forward_length=LENGTH,
        steering_grid_degrees=ANGLES,
        continuous_grid_error_bound=GRID_ERROR_BOUND,
        window_start_exclusive_s=150,
        window_end_inclusive_s=180,
        chord_intervals=120,
        physical_intervals_checked=1500 * len(cases),
        helpers_sha256={
            name: digest(ROOT / name)
            for name in [
                "analyze_recovery_stalls.py",
                "summarize_stopping_preview.py",
                "verify_stopping_preview.py",
                "test_recovery_arc_geometry.py",
            ]
        },
        geometry_audits=len(audits),
        numeric_tolerance=1e-9,
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, data in [
        ("checkpoints.csv", records),
        ("steering-grid.csv", curves),
        ("geometry-audit.csv", audits),
        ("summary.csv", summary),
    ]:
        save_rows(args.output / name, data)
    (args.output / "sources.json").write_text(json.dumps(provenance, indent=2) + "\n")
    if args.plot:
        plot(args.output, curves, records)
    for row in summary:
        print(row)
    print(
        f"Checked {len(records)} poses, {len(curves)} grid arcs and {len(audits)} independent geometry audits"
    )


if __name__ == "__main__":
    main()
