#!/usr/bin/env python3
"""Verify serialized pose witnesses, geometry and complete refinement covers."""

import argparse
from collections import defaultdict
from fractions import Fraction
import json
import math
from pathlib import Path

from analyze_admission_envelope_refinement import (
    Cell,
    MAX_CELLS,
    PINNED,
    PRIOR,
    PROTOCOL,
    ROOT,
    verify_cover,
    witness_state,
)
from analyze_commanded_recovery_arc import META, arc_components, flag
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import EPS
from analyze_terminal_arc_live import SIMULATOR


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--compare", type=Path)
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    output = args.results
    source = json.loads((output / "sources.json").read_text())
    if args.compare:
        for name in [
            "accounting.csv",
            "decisions.csv",
            "witnesses.csv",
            "certificates.csv",
            "geometry-audit.csv",
            "summary.csv",
            "sources.json",
        ]:
            assert (output / name).read_bytes() == (
                args.compare / name
            ).read_bytes(), name
    assert source["input_sha256"] == PINNED
    for name, sha in PINNED.items():
        assert digest(PRIOR / name) == sha, name
    assert (output / "accounting.csv").read_bytes() == (
        PRIOR / "accounting.csv"
    ).read_bytes()
    analysis = source["analysis"]
    assert (
        digest(ROOT / "analyze_admission_envelope_refinement.py")
        == analysis["script_sha256"]
    )
    assert digest(PROTOCOL) == analysis["protocol_sha256"]
    assert digest(SIMULATOR) == analysis["simulator_sha256"]
    for name, sha in analysis["helpers_sha256"].items():
        assert digest(ROOT / name) == sha, name
    map_path = ROOT / "inputs/obstacles.csv"
    assert digest(map_path) == source["map_sha256"]
    maps = defaultdict(list)
    for row in read_rows(map_path):
        maps[row["scenario"]].append((float(row["x"]), float(row["y"])))
    previous, rows = read_rows(PRIOR / "decisions.csv"), read_rows(
        output / "decisions.csv"
    )
    assert len(previous) == len(rows) == 25440
    original_geometry = {
        r["audit_id"]: r for r in read_rows(PRIOR / "geometry-audit.csv")
    }
    geometry = {
        r["audit_id"]: {k: (v if k == "scenario" else float(v)) for k, v in r.items()}
        for r in read_rows(output / "geometry-audit.csv")
    }
    for g in geometry.values():
        pose = tuple(g["pose_" + a] for a in ["x", "y", "yaw"])
        obstacle, wall = arc_components(pose, g["steering"], maps[g["scenario"]])
        assert abs(obstacle - g["obstacle"]) < EPS and abs(wall - g["wall"]) < EPS
        assert abs(min(obstacle, wall) - g["clearance"]) < EPS
        assert g["chord_error"] <= g["sagitta_bound"] + 1e-9
    by_witness, by_leaf = defaultdict(list), defaultdict(list)
    for witness in read_rows(output / "witnesses.csv"):
        by_witness[int(witness["source_row"])].append(witness)
    for leaf in read_rows(output / "certificates.csv"):
        by_leaf[int(leaf["source_row"])].append(leaf)
    used_ids = set()
    for n, (old, new) in enumerate(zip(previous, rows), 1):
        assert int(new["source_row"]) == n
        assert all(
            old[k] == new[k]
            for k in META
            + ["input_kind", "path", "envelope", "radius", "heading_half_width_deg"]
        )
        assert new["prior_state"] == old["state"]
        assert flag(new["truth_clear"]) == flag(old["truth_clear"])
        assert flag(new["covered"]) == flag(old["covered"])
        if old["state"] != "unresolved":
            assert new["state"] == old["state"] and int(new["cell_visits"]) == 0
        if new["state"] in ["permit", "blocked"]:
            assert (new["state"] == "permit") == flag(old["baseline_clear"])
            if flag(new["covered"]):
                assert (new["state"] == "permit") == flag(new["truth_clear"])
        origin_row = original_geometry[old["centre_audit_id"]]
        origin = tuple(float(origin_row["pose_" + a]) for a in ["x", "y", "yaw"])
        radius, margin = float(old["radius"]), float(old["margin"])
        if new["state"] == "mixed":
            ws = by_witness[n]
            assert len(ws) == 2 and {w["state"] for w in ws} == {"permit", "blocked"}
            for w in ws:
                nx, ny, angle = (
                    float(w[k])
                    for k in ["normalized_x", "normalized_y", "heading_offset_deg"]
                )
                assert math.hypot(nx, ny) <= 1 + 1e-12 and abs(angle) <= 4
                g = geometry[w["geometry_audit_id"]]
                assert g["scenario"] == old["scenario"] and g["steering"] == float(
                    origin_row["steering"]
                )
                assert witness_state(g, margin) == w["state"]
                expected = (
                    origin[0] + radius * nx,
                    origin[1] + radius * ny,
                    origin[2] + math.radians(angle),
                )
                assert all(
                    abs(g["pose_" + a] - value) < EPS
                    for a, value in zip(["x", "y", "yaw"], expected)
                )
                used_ids.add(w["geometry_audit_id"])
        else:
            assert n not in by_witness
        if flag(new["new_permit"]) or flag(new["new_blocked"]):
            exported = by_leaf[n]
            ids = sorted(l["cell_id"] for l in exported)
            assert all(i.startswith("r") and set(i[1:]) <= set("01") for i in ids)
            assert all(not b.startswith(a) for a, b in zip(ids, ids[1:])), (
                n,
                "overlapping leaves",
            )
            assert (
                sum((Fraction(1, 2 ** (len(i) - 1)) for i in ids), Fraction()) == 1
            ), (n, "incomplete cover")
            leaves = []
            for leaf in exported:
                cell = Cell(
                    leaf["cell_id"],
                    tuple(
                        float(leaf[k])
                        for k in [
                            "x_min",
                            "x_max",
                            "y_min",
                            "y_max",
                            "heading_min_deg",
                            "heading_max_deg",
                        ]
                    ),
                )
                item = dict(cell=cell, state=leaf["state"], geometry=None)
                if leaf["state"] != "outside_disk":
                    item["geometry"] = geometry[leaf["geometry_audit_id"]]
                    assert item["geometry"]["scenario"] == old["scenario"] and item[
                        "geometry"
                    ]["steering"] == float(origin_row["steering"])
                    item.update(
                        {
                            k: float(leaf[k])
                            for k in [
                                "allowance",
                                "obstacle_lower",
                                "obstacle_upper",
                                "wall_lower",
                                "wall_upper",
                            ]
                        }
                    )
                    used_ids.add(leaf["geometry_audit_id"])
                leaves.append(item)
            assert verify_cover(leaves, radius, margin, origin) == new["state"]
        else:
            assert n not in by_leaf
        assert 0 <= int(new["cell_visits"]) <= MAX_CELLS
        assert flag(new["budget_exhausted"]) == (new["state"] == "unresolved")
    assert used_ids == set(geometry)
    assert len(by_witness) == source["audit"]["mixed"]
    assert (
        len(by_leaf) == source["audit"]["new_permit"] + source["audit"]["new_blocked"]
    )
    print(
        json.dumps(
            dict(
                decision_rows=len(rows),
                mixed_rows=len(by_witness),
                complete_covers=len(by_leaf),
                verified_geometries=len(geometry),
                repeated_bytes=bool(args.compare),
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
