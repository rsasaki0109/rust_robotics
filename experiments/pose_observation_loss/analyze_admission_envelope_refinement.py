#!/usr/bin/env python3
"""Certify or witness ambiguity within the unchanged pinned pose envelopes."""

import argparse
from collections import defaultdict, deque
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path

from analyze_commanded_arc_components import measured_geometry
from analyze_commanded_recovery_arc import META, arc_components, flag
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import EPS, KEY, subsets
from analyze_model_admission_envelopes import gate_state
from analyze_recovery_arcs import LENGTH, arc_clearance
from analyze_terminal_arc_live import SIMULATOR
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/model-admission-envelopes"
PROTOCOL = ROOT / "results/admission-envelope-refinement/protocol.md"
PINNED = {
    "accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "decisions.csv": "7eeaa32e09c5235c70bd9e95db9b3998f68f835ff392036b554c56536395b394",
    "geometry-audit.csv": "ffae1282b46e9da0e792840a1d5eae65c9219dfe98c864bae149fc491b32a6f3",
    "sources.json": "b13e005d5e0e327c8951519823dd73c9f8916573f68ce647d33635b4d8ad1313",
}
MAX_CELLS = 256
DISK_EPS = 1e-12
STATES = ["permit", "blocked", "mixed", "unresolved"]
FLAGS = [
    *STATES,
    "prior_unresolved",
    "new_permit",
    "new_blocked",
    "heading_mixed",
    "position_mixed",
    "both_axes_mixed",
    "one_axis_mixed",
    "mixed_after_cover",
    "budget_exhausted",
    "true_permit",
    "false_permit",
    "true_block",
    "false_block",
    "unresolved_true_clear",
    "unresolved_true_blocked",
    "covered",
    "covered_definite_error",
]
GROUPS = ["all", "prior_optimistic", "prior_conservative", "optimistic_arc_gain"]


@dataclass(frozen=True)
class Cell:
    name: str = "r"
    box: tuple = (-1.0, 1.0, -1.0, 1.0, -4.0, 4.0)

    def middle(self):
        return tuple((self.box[i] + self.box[i + 1]) / 2 for i in [0, 2, 4])

    def closest_xy(self):
        return tuple(max(self.box[i], min(0.0, self.box[i + 1])) for i in [0, 2])

    def outside_disk(self):
        return math.hypot(*self.closest_xy()) > 1 + DISK_EPS

    def contributions(self, radius):
        hx, hy, ha = ((self.box[i + 1] - self.box[i]) / 2 for i in [0, 2, 4])
        return radius * hx, radius * hy, 2 * LENGTH * math.sin(math.radians(ha) / 2)

    def allowance(self, radius):
        x, y, angle = self.contributions(radius)
        return math.hypot(x, y) + angle + EPS

    def children(self, radius):
        contributions = self.contributions(radius)
        axis = max(range(3), key=lambda i: (contributions[i], -i))
        offset = 2 * axis
        middle = (self.box[offset] + self.box[offset + 1]) / 2
        lower, upper = list(self.box), list(self.box)
        lower[offset + 1], upper[offset] = middle, middle
        return Cell(self.name + "0", tuple(lower)), Cell(self.name + "1", tuple(upper))


def cell_bounds(geometry, allowance):
    return {
        f"{part}_{bound}": geometry[part] + sign * allowance
        for part in ["obstacle", "wall"]
        for bound, sign in [("lower", -1), ("upper", 1)]
    }


def witness_state(geometry, margin):
    if min(abs(geometry[k] - margin) for k in ["obstacle", "wall"]) <= EPS:
        return None
    return (
        "permit"
        if geometry["obstacle"] > margin and geometry["wall"] >= margin
        else "blocked"
    )


def refine(radius, margin, measure, budget=MAX_CELLS):
    """No truth input. Missing examples never imply a definite decision."""
    assert radius >= 0 and budget >= 0
    witnesses, axes = {}, {"heading": set(), "position": set()}

    def observe(parameters, phase, geometry=None):
        nx, ny, angle = parameters
        assert math.hypot(nx, ny) <= 1 + DISK_EPS and abs(angle) <= 4
        geometry = measure(parameters) if geometry is None else geometry
        state = witness_state(geometry, margin)
        if state:
            if phase in axes:
                axes[phase].add(state)
            witnesses.setdefault(
                state,
                dict(
                    state=state, parameters=parameters, phase=phase, geometry=geometry
                ),
            )

    for angle in range(-4, 5):
        observe((0, 0, angle), "heading")
    observe((0, 0, 0), "position")
    for distance in [0.5, 1 - 1e-9]:
        for direction in range(8):
            angle = direction * math.pi / 4
            observe(
                (distance * math.cos(angle), distance * math.sin(angle), 0), "position"
            )

    heading_mixed = len(axes["heading"]) == 2
    position_mixed = len(axes["position"]) == 2
    queue, leaves, visits = deque([Cell()]), [], 0
    initial_mixed = len(witnesses) == 2
    while not initial_mixed and queue and visits < budget:
        cell = queue.popleft()
        if cell.outside_disk():
            leaves.append(dict(cell=cell, state="outside_disk", geometry=None))
            continue
        visits += 1
        parameters = cell.middle()
        geometry = measure(parameters)
        if math.hypot(*parameters[:2]) <= 1:
            observe(parameters, "cover", geometry)
        else:
            nearest = cell.closest_xy()
            if math.hypot(*nearest) <= 1:
                observe((*nearest, parameters[2]), "cover")
        if len(witnesses) == 2:
            break
        allowance = cell.allowance(radius)
        bounds = cell_bounds(geometry, allowance)
        state = gate_state(bounds, margin)
        if state == "unresolved":
            queue.extend(cell.children(radius))
        else:
            leaves.append(
                dict(
                    cell=cell,
                    state=state,
                    geometry=geometry,
                    allowance=allowance,
                    **bounds,
                )
            )
    if len(witnesses) == 2:
        state, leaves = "mixed", []
    elif not queue:
        states = {leaf["state"] for leaf in leaves if leaf["state"] != "outside_disk"}
        assert len(states) == 1, states
        state = states.pop()
    else:
        state, leaves = "unresolved", []
    return dict(
        state=state,
        witnesses=list(witnesses.values()) if state == "mixed" else [],
        leaves=leaves,
        heading_mixed=heading_mixed,
        position_mixed=position_mixed,
        both_axes_mixed=heading_mixed and position_mixed,
        one_axis_mixed=heading_mixed != position_mixed,
        mixed_after_cover=state == "mixed" and not initial_mixed,
        budget_exhausted=state == "unresolved" and visits == budget,
        cell_visits=visits,
    )


def verify_cover(leaves, radius, margin, origin):
    """Check a complete binary partition, not just a collection of good cells."""
    by_name = {leaf["cell"].name: leaf for leaf in leaves}
    assert len(by_name) == len(leaves) and by_name
    consumed, states = set(), set()

    def visit(cell):
        if cell.name not in by_name:
            assert any(name.startswith(cell.name) for name in by_name), cell.name
            for child in cell.children(radius):
                visit(child)
            return
        leaf = by_name[cell.name]
        assert leaf["cell"] == cell
        consumed.add(cell.name)
        if leaf["state"] == "outside_disk":
            assert cell.outside_disk()
            return
        assert not cell.outside_disk()
        geometry, parameters = leaf["geometry"], cell.middle()
        expected = (
            origin[0] + radius * parameters[0],
            origin[1] + radius * parameters[1],
            origin[2] + math.radians(parameters[2]),
        )
        assert all(
            abs(geometry[f"pose_{k}"] - value) < EPS
            for k, value in zip(["x", "y", "yaw"], expected)
        )
        allowance = cell.allowance(radius)
        assert allowance == leaf["allowance"]
        bounds = cell_bounds(geometry, allowance)
        assert all(bounds[k] == leaf[k] for k in bounds)
        assert gate_state(bounds, margin) == leaf["state"] != "unresolved"
        states.add(leaf["state"])

    visit(Cell())
    assert consumed == set(by_name) and len(states) == 1
    return states.pop()


def evaluated_record(previous, outcome, source_row, requests):
    state = outcome["state"]
    prior_unresolved = previous["state"] == "unresolved"
    truth, covered = flag(previous["truth_clear"]), flag(previous["covered"])
    if not prior_unresolved:
        assert state == previous["state"]
    error = (state == "permit" and not truth) or (state == "blocked" and truth)
    assert not (covered and error)
    return dict(
        source_row=source_row,
        **{k: previous[k] for k in META + ["input_kind", "path", "envelope"]},
        **{k: flag(previous[k]) for k in GROUPS if k != "all"},
        radius=previous["radius"],
        heading_half_width_deg=previous["heading_half_width_deg"],
        prior_state=previous["state"],
        state=state,
        truth_clear=truth,
        covered=covered,
        **{k: state == k for k in STATES},
        prior_unresolved=prior_unresolved,
        new_permit=prior_unresolved and state == "permit",
        new_blocked=prior_unresolved and state == "blocked",
        **{
            k: outcome[k]
            for k in [
                "heading_mixed",
                "position_mixed",
                "both_axes_mixed",
                "one_axis_mixed",
                "mixed_after_cover",
                "budget_exhausted",
                "cell_visits",
            ]
        },
        geometry_requests=requests,
        certificate_leaves=len(outcome["leaves"]),
        true_permit=state == "permit" and truth,
        false_permit=state == "permit" and not truth,
        true_block=state == "blocked" and not truth,
        false_block=state == "blocked" and truth,
        unresolved_true_clear=state in {"mixed", "unresolved"} and truth,
        unresolved_true_blocked=state in {"mixed", "unresolved"} and not truth,
        covered_definite_error=covered and error,
    )


def summarize(records):
    grouped = defaultdict(list)
    dimensions = ["cohort", "variant", "subset", "input_kind", "path", "envelope"]
    for row in records:
        for cohort in ["all", row["cohort"]]:
            for subset in subsets(row):
                grouped[
                    cohort,
                    row["variant"],
                    subset,
                    row["input_kind"],
                    row["path"],
                    row["envelope"],
                ].append(row)
    result = []
    for key, all_rows in sorted(grouped.items()):
        for group in GROUPS:
            rows = all_rows if group == "all" else [r for r in all_rows if r[group]]
            record = dict(
                **dict(zip(dimensions, key)),
                group=group,
                checkpoints=len(rows),
                cases=len({tuple(r[k] for k in KEY) for r in rows}),
                **{k: sum(r[k] for r in rows) for k in FLAGS},
                cell_visits=sum(r["cell_visits"] for r in rows),
                max_cell_visits=max((r["cell_visits"] for r in rows), default=0),
                geometry_requests=sum(r["geometry_requests"] for r in rows),
                certificate_leaves=sum(r["certificate_leaves"] for r in rows),
            )
            assert sum(record[k] for k in STATES) == len(rows)
            assert (
                record["new_permit"]
                + record["new_blocked"]
                + record["mixed"]
                + record["unresolved"]
                == record["prior_unresolved"]
            )
            assert record["true_permit"] + record["false_permit"] == record["permit"]
            assert record["true_block"] + record["false_block"] == record["blocked"]
            assert (
                record["unresolved_true_clear"] + record["unresolved_true_blocked"]
                == record["mixed"] + record["unresolved"]
            )
            result.append(record)
    return result


WITNESS_FIELDS = [
    "source_row",
    "state",
    "phase",
    "normalized_x",
    "normalized_y",
    "heading_offset_deg",
    "radius",
    "margin",
    "geometry_audit_id",
    "obstacle",
    "wall",
    "position_domain_roundoff",
    "heading_domain_roundoff_deg",
]
CERTIFICATE_FIELDS = [
    "source_row",
    "cell_id",
    "x_min",
    "x_max",
    "y_min",
    "y_max",
    "heading_min_deg",
    "heading_max_deg",
    "radius",
    "margin",
    "state",
    "geometry_audit_id",
    "allowance",
    "obstacle_lower",
    "obstacle_upper",
    "wall_lower",
    "wall_upper",
]


def save_optional_rows(path, rows, fields):
    if rows:
        save_rows(path, rows)
    else:
        with path.open("w", newline="") as target:
            csv.writer(target, lineterminator="\n").writerow(fields)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    if args.output.exists():
        parser.error("Output directory must be new")
    for name, sha in PINNED.items():
        assert digest(PRIOR / name) == sha, name
    prior_source = json.loads((PRIOR / "sources.json").read_text())
    helpers = dict(prior_source["analysis"]["helpers_sha256"])
    helpers["analyze_model_admission_envelopes.py"] = prior_source["analysis"][
        "script_sha256"
    ]
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    assert digest(SIMULATOR) == prior_source["analysis"]["simulator_sha256"]
    map_path = ROOT / "inputs/obstacles.csv"
    assert digest(map_path) == prior_source["map_sha256"]
    maps = defaultdict(list)
    for row in read_rows(map_path):
        maps[row["scenario"]].append((float(row["x"]), float(row["y"])))
    previous = read_rows(PRIOR / "decisions.csv")
    accounting = read_rows(PRIOR / "accounting.csv")
    old_geometry = {
        r["audit_id"]: {k: float(v) if k != "scenario" else v for k, v in r.items()}
        for r in read_rows(PRIOR / "geometry-audit.csv")
    }
    cache = {
        (
            r["scenario"],
            tuple(r[f"pose_{a}"] for a in ["x", "y", "yaw"]),
            r["steering"],
        ): r
        for r in old_geometry.values()
    }
    old_cache_size = len(cache)
    assert (
        len(accounting) == 648 and sum(flag(r["selected"]) for r in accounting) == 368
    )
    assert len(previous) == 25440
    assert {tuple(r[k] for k in KEY) for r in accounting if flag(r["selected"])} == {
        tuple(r[k] for k in KEY) for r in previous
    }
    published, records, witnesses, certificates = {}, [], [], []
    for source_row, row in enumerate(previous, 1):
        origin_record = old_geometry[row["centre_audit_id"]]
        origin = tuple(origin_record[f"pose_{a}"] for a in ["x", "y", "yaw"])
        radius, margin = float(row["radius"]), float(row["margin"])
        steering, scenario = origin_record["steering"], row["scenario"]
        requests = 0

        def measure(parameters):
            nonlocal requests
            requests += 1
            nx, ny, angle = parameters
            pose = (
                origin[0] + radius * nx,
                origin[1] + radius * ny,
                origin[2] + math.radians(angle),
            )
            key = scenario, pose, steering
            if key not in cache:
                obstacle, wall = arc_components(pose, steering, maps[scenario])
                assert (
                    abs(
                        min(obstacle, wall)
                        - arc_clearance(pose, steering, maps[scenario])
                    )
                    < EPS
                )
                cache[key] = dict(
                    scenario=scenario,
                    pose_x=pose[0],
                    pose_y=pose[1],
                    pose_yaw=pose[2],
                    steering=steering,
                    obstacle=obstacle,
                    wall=wall,
                )
            return cache[key]

        def publish(geometry):
            pose = tuple(geometry[f"pose_{a}"] for a in ["x", "y", "yaw"])
            audit = measured_geometry(
                scenario, pose, steering, maps[scenario], published
            )
            assert all(abs(audit[k] - geometry[k]) < EPS for k in ["obstacle", "wall"])
            return audit["audit_id"]

        if row["state"] != "unresolved":
            outcome = dict(
                state=row["state"],
                witnesses=[],
                leaves=[],
                heading_mixed=False,
                position_mixed=False,
                both_axes_mixed=False,
                one_axis_mixed=False,
                mixed_after_cover=False,
                budget_exhausted=False,
                cell_visits=0,
            )
        else:
            assert (
                row["envelope"] != "central"
                and float(row["heading_half_width_deg"]) == 4
            )
            outcome = refine(radius, margin, measure)
            if outcome["state"] in {"permit", "blocked"}:
                assert (
                    verify_cover(outcome["leaves"], radius, margin, origin)
                    == outcome["state"]
                )
        for witness in outcome["witnesses"]:
            nx, ny, angle = witness["parameters"]
            geometry = witness["geometry"]
            assert witness_state(geometry, margin) == witness["state"]
            assert math.hypot(nx, ny) <= 1 + DISK_EPS and abs(angle) <= 4
            position_roundoff = max(
                0.0,
                math.dist(origin[:2], (geometry["pose_x"], geometry["pose_y"]))
                - radius,
            )
            yaw_roundoff = max(
                0.0, abs(math.degrees(geometry["pose_yaw"] - origin[2])) - 4
            )
            assert position_roundoff <= EPS and yaw_roundoff <= 1e-10
            witnesses.append(
                dict(
                    source_row=source_row,
                    state=witness["state"],
                    phase=witness["phase"],
                    normalized_x=nx,
                    normalized_y=ny,
                    heading_offset_deg=angle,
                    radius=radius,
                    margin=margin,
                    geometry_audit_id=publish(geometry),
                    obstacle=geometry["obstacle"],
                    wall=geometry["wall"],
                    position_domain_roundoff=position_roundoff,
                    heading_domain_roundoff_deg=yaw_roundoff,
                )
            )
        for leaf in outcome["leaves"]:
            cell = leaf["cell"]
            certificates.append(
                dict(
                    source_row=source_row,
                    cell_id=cell.name,
                    **dict(
                        zip(
                            [
                                "x_min",
                                "x_max",
                                "y_min",
                                "y_max",
                                "heading_min_deg",
                                "heading_max_deg",
                            ],
                            cell.box,
                        )
                    ),
                    radius=radius,
                    margin=margin,
                    state=leaf["state"],
                    geometry_audit_id=(
                        publish(leaf["geometry"]) if leaf["geometry"] else ""
                    ),
                    **{
                        k: leaf.get(k, "")
                        for k in [
                            "allowance",
                            "obstacle_lower",
                            "obstacle_upper",
                            "wall_lower",
                            "wall_upper",
                        ]
                    },
                )
            )
        # Saved truth and coverage are used only after the geometric search.
        records.append(evaluated_record(row, outcome, source_row, requests))
    assert len(witnesses) == 2 * sum(r["mixed"] for r in records)
    assert {r["source_row"] for r in certificates} == {
        r["source_row"] for r in records if r["new_permit"] or r["new_blocked"]
    }
    source = dict(
        input_sha256=PINNED,
        map_sha256=digest(map_path),
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            simulator_sha256=digest(SIMULATOR),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        fixed_search=dict(
            intersecting_cell_budget=MAX_CELLS,
            disk_pruning_tolerance=DISK_EPS,
            distance_padding=EPS,
        ),
        inherited_validation=prior_source["inherited_validation"],
        audit=dict(
            decision_rows=len(records),
            prior_unresolved=sum(r["prior_unresolved"] for r in records),
            mixed=sum(r["mixed"] for r in records),
            new_permit=sum(r["new_permit"] for r in records),
            new_blocked=sum(r["new_blocked"] for r in records),
            unresolved=sum(r["unresolved"] for r in records),
            budget_exhausted=sum(r["budget_exhausted"] for r in records),
            cell_visits=sum(r["cell_visits"] for r in records),
            max_cell_visits=max(r["cell_visits"] for r in records),
            geometry_requests=sum(r["geometry_requests"] for r in records),
            new_exact_geometry_evaluations=len(cache) - old_cache_size,
            published_geometry_audits=len(published),
            witness_rows=len(witnesses),
            certificate_leaves=len(certificates),
            max_published_chord_error=max(
                (r["chord_error"] for r in published.values()), default=0
            ),
            max_published_sagitta_bound=max(
                (r["sagitta_bound"] for r in published.values()), default=0
            ),
            covered_definite_errors=sum(r["covered_definite_error"] for r in records),
        ),
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", accounting),
        ("decisions", records),
        ("summary", summarize(records)),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    for name, rows, fields in [
        ("witnesses", witnesses, WITNESS_FIELDS),
        ("certificates", certificates, CERTIFICATE_FIELDS),
        (
            "geometry-audit",
            list(published.values()),
            [
                "audit_id",
                "scenario",
                "pose_x",
                "pose_y",
                "pose_yaw",
                "steering",
                "obstacle",
                "wall",
                "clearance",
                "chord_error",
                "sagitta_bound",
            ],
        ),
    ]:
        save_optional_rows(args.output / f"{name}.csv", rows, fields)
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(source["audit"], indent=2))


if __name__ == "__main__":
    main()
