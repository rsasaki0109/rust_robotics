#!/usr/bin/env python3
"""Audit stopping previews with exact obstacle geometry at fixed checkpoints."""

import argparse
import csv
import json
import math
from pathlib import Path

from compare_correlations import check_contacts
from verify_restart_consistency import check_restart_track


def segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    norm = dx * dx + dy * dy
    t = (
        max(0, min(1, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / norm))
        if norm
        else 0
    )
    return math.hypot(point[0] - start[0] - t * dx, point[1] - start[1] - t * dy)


def exact_preview(before, row, obstacles):
    # Reconstruct only the observed pose from the prior physical state and the
    # logged errors. Recalculate the full geometry, rather than reusing the
    # simulator's optimized clearance lower bound.
    x = float(before["x"]) + float(row["noise_x"]) + float(row["bias_x"])
    y = float(before["y"]) + float(row["noise_y"]) + float(row["bias_y"])
    yaw = float(before["yaw"]) + float(row["noise_yaw"]) + float(row["bias_yaw"])
    if "preview_pose_x" in row:
        # These fields are independently reconstructed by verify.py first.
        x, y, yaw = (float(row[f"preview_pose_{axis}"]) for axis in ["x", "y", "yaw"])
    v = float(before["v"])
    target = float(row["nominal_target_speed"])
    steering = float(row["nominal_steering"])
    points = [(x, y)]
    length = 0.0
    for step in range(200):
        a = target - v if step < 10 else max(-1, min(0, -4 * v))
        x += v * math.cos(yaw) * 0.02
        y += v * math.sin(yaw) * 0.02
        yaw += v / 0.4 * math.tan(steering) * 0.02
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        v += a * 0.02
        length += math.dist(points[-1], (x, y))
        points.append((x, y))
        if step >= 10 and abs(v) <= 0.001:
            break
    assert abs(v) <= 0.001 and len(points) <= 201
    clearance = min(min(px, py, 50 - px, 50 - py) for px, py in points)
    # Nearest-first pruning computes the exact minimum: all unvisited obstacles
    # have distance >= current minimum + total travel and cannot reduce it.
    ordered = sorted((math.dist(point, points[0]), point) for point in obstacles)
    for initial_distance, point in ordered:
        if initial_distance - length >= clearance:
            break
        clearance = min(
            clearance,
            *(segment_distance(point, a, b) for a, b in zip(points, points[1:])),
        )
    return clearance, len(points) - 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O; assertions are required")
    run = args.run
    provenance = json.loads((run / "provenance.json").read_text())
    results = list(csv.DictReader((run / "all-results.csv").open()))
    variants = provenance["variants"]
    assert variants in [["averaged", "guarded"], ["guarded", "consistent"]]
    candidate = variants[-1]
    assert {r["variant"] for r in results} == set(variants)
    maps = {}
    with (
        Path(__file__).resolve().parent / "inputs" / "obstacles.csv"
    ).open() as source:
        for r in csv.DictReader(source):
            maps.setdefault(r["scenario"], []).append((float(r["x"]), float(r["y"])))
    audits = []
    for item in json.loads((run / "manifest.json").read_text()):
        if item["variant"] != candidate:
            continue
        groups = {}
        with (run / item["directory"] / "trajectories.csv").open() as source:
            for row in csv.DictReader(source):
                groups.setdefault((row["scenario"], row["planner"]), []).append(row)
        for key, track in groups.items():
            if "consistency_ready" in track[0]:
                check_restart_track(track, candidate == "consistent")
            selected = [1]
            for flag in ["guard_triggered", "guard_released"]:
                first = next(
                    (i for i, r in enumerate(track[1:], 1) if r[flag] == "true"), None
                )
                if first is not None:
                    selected.append(first)
            result = next(
                r
                for r in results
                if r["variant"] == candidate
                and r["seed"] == str(item["seed"])
                and r["mode"] == item["mode"]
                and (r["scenario"], r["planner"]) == key
            )
            if result["collision"] == "true":
                selected.append(len(track) - 1)
            for i in range(1000, len(track), 1000):
                if len(set(selected)) >= 8:
                    break
                selected.append(i)
            for i in sorted(set(selected)):
                row = track[i]
                exact, steps = exact_preview(track[i - 1], row, maps[key[0]])
                lower = float(row["preview_clearance_lower_bound"])
                margin = float(row["preview_margin"])
                assert lower <= exact + 2e-8, (
                    item,
                    key,
                    i,
                    "not a lower bound",
                    lower,
                    exact,
                )
                for threshold in [margin, margin + 0.05]:
                    if abs(exact - threshold) > 2e-8 and abs(lower - threshold) > 2e-8:
                        assert (lower > threshold) == (exact > threshold), (
                            item,
                            key,
                            i,
                            "wrong clearance classification",
                        )
                audits.append(
                    dict(
                        mode=item["mode"],
                        seed=item["seed"],
                        scenario=key[0],
                        planner=key[1],
                        time_s=row["time_s"],
                        preview_steps=steps,
                        lower_bound=lower,
                        exact_clearance=exact,
                        margin=margin,
                    )
                )
    with (run / "preview-audit.csv").open("w", newline="") as target:
        writer = csv.DictWriter(target, fieldnames=list(audits[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(audits)
    contacts = check_contacts(
        run,
        provenance.get("noise_correlation_time_s", 0),
        results,
        provenance["inputs_sha256"],
    )
    if contacts:
        with (run / "contacts.csv").open("w", newline="") as target:
            writer = csv.DictWriter(
                target, fieldnames=list(contacts[0]), lineterminator="\n"
            )
            writer.writeheader()
            writer.writerows(contacts)
    print(
        f"Exact geometry: {len(audits)} stopping-preview checkpoints and {len(contacts)} contact rows verified."
    )


if __name__ == "__main__":
    main()
