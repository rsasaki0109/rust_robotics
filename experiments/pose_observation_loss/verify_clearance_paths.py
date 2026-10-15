"""Audit every exported path segment against the original point obstacles."""

import csv
import json
import hashlib
import math
from pathlib import Path
import sys

from verify_stopping_preview import segment_distance


def audit(directory):
    if not __debug__:
        raise RuntimeError("Run without Python -O")
    source = Path(__file__).resolve().parent / "inputs"
    result = []
    source_maps = (source / "obstacles.csv").read_bytes()
    for name in ["coarse-nominal", "coarse-padding", "fine-nominal", "fine-padding"]:
        root = directory / name
        assert (root / "obstacles.csv").read_bytes() == source_maps
        maps, paths = {}, {}
        for r in csv.DictReader((root / "obstacles.csv").open()):
            maps.setdefault(r["scenario"], []).append((float(r["x"]), float(r["y"])))
        for r in csv.DictReader((root / "paths.csv").open()):
            path = paths.setdefault((r["scenario"], r["planner"]), [])
            assert int(r["index"]) == len(path)
            path.append((float(r["x"]), float(r["y"])))
        metrics = list(csv.DictReader((root / "planning.csv").open()))
        assert len(metrics) == 9
        expected = {
            (scenario, planner)
            for scenario in ["open_50x50", "dense_50x50", "maze_50x50"]
            for planner in ["A*", "JPS", "Theta*"]
        }
        assert {(r["scenario"], r["planner"]) for r in metrics} == expected
        resolution, radius = {
            "coarse-nominal": (1, 0.5),
            "coarse-padding": (1, 0.9),
            "fine-nominal": (0.125, 0.5),
            "fine-padding": (0.125, 0.99),
        }[name]
        assert set(paths) == {
            (r["scenario"], r["planner"]) for r in metrics if r["status"] == "planned"
        }
        for row in metrics:
            assert (
                float(row["resolution"]) == resolution
                and float(row["planning_radius"]) == radius
            )
            assert row["status"] in ["planned", "planning_failed"]
            key = row["scenario"], row["planner"]
            if row["status"] == "planning_failed":
                assert key not in paths
                result.append(
                    dict(
                        fixture=name,
                        **row,
                        min_clearance="",
                        requested_radius_met=False
                    )
                )
                continue
            path = paths[key]
            assert len(path) == int(row["waypoints"])
            start, goal = (
                ((5, 5), (35, 45)) if key[0] == "maze_50x50" else ((2, 2), (48, 48))
            )
            assert path[0] == start and path[-1] == goal
            assert all(math.isfinite(v) and 0 <= v <= 50 for p in path for v in p)
            assert all(
                abs(v / resolution - round(v / resolution)) < 1e-9
                for p in path
                for v in p
            )
            assert all(math.dist(a, b) > 0 for a, b in zip(path, path[1:]))
            length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
            assert abs(length - float(row["path_length"])) < 2e-8
            clearance = min(
                segment_distance(p, a, b)
                for a, b in zip(path, path[1:])
                for p in maps[key[0]]
            )
            clearance = min(clearance, *(min(x, y, 50 - x, 50 - y) for x, y in path))
            assert clearance > 0.5, (name, key, clearance)
            if name == "fine-padding":
                assert clearance >= 0.9, (
                    "candidate misses fixed target",
                    key,
                    clearance,
                )
            result.append(
                dict(
                    fixture=name,
                    **row,
                    min_clearance=clearance,
                    requested_radius_met=clearance >= float(row["planning_radius"])
                )
            )
    with (directory / "path-audit.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(result[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(result)
    # Save hashes after a full geometric audit; the runner rechecks them.
    manifest = {
        str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
        for name in ["coarse-nominal", "coarse-padding", "fine-nominal", "fine-padding"]
        for p in sorted((directory / name).glob("*.csv"))
    }
    (directory / "fixture-hashes.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    return result


if __name__ == "__main__":
    for row in audit(Path(sys.argv[1])):
        print(row)
