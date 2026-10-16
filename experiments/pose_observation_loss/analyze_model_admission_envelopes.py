#!/usr/bin/env python3
"""Compare central/noise/shared-anchor three-way gates on pinned recovery paths."""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics

from analyze_commanded_arc_components import measured_geometry
from analyze_commanded_recovery_arc import META, classify, flag, identity
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import EPS, KEY, subsets
from analyze_recovery_arcs import LENGTH
from analyze_terminal_arc_live import SIMULATOR
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PROTOCOL = ROOT / "results/model-admission-envelopes/protocol.md"
PINNED = {
    "epoch-position-components/accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "epoch-position-components/positions.csv": "67fe28537f1e84d47ee8711b11f3df0cc52f8b770e694da0055d6265604f321e",
    "epoch-position-components/sources.json": "01e6b94b68c892621047829585df1db46c6471e7a3dfdd819c40e7fa6fe136db",
    "commanded-arc-components/components.csv": "78a789a7fe70e1c5327a12fa13a836fb5e25c27b9ffc78b8c154cea801696226",
    "commanded-arc-components/geometry-audit.csv": "1d8495ee1ac862976eab27a9c3c25b6566ff1463560d673203b5f4612c518d81",
    "commanded-arc-components/sources.json": "52e838cc4934e9ce63be5da1d76f7d8d5f5ff1d197ce3a5b65e51fded4f4d7c3",
}
RULES = ["central", "noise_envelope", "noise_anchor_envelope"]
GROUPS = ["all", "prior_optimistic", "prior_conservative", "optimistic_arc_gain"]
OFFSETS = list(range(-4, 5))
YAW_INTERPOLATION = 2 * LENGTH * math.sin(math.radians(0.5) / 2)
FLAGS = [
    "permit",
    "blocked",
    "unresolved",
    "true_permit",
    "false_permit",
    "true_block",
    "false_block",
    "unresolved_true_clear",
    "unresolved_true_blocked",
    "baseline_true_permit_deferred",
    "baseline_false_permit_deferred",
    "baseline_true_block_deferred",
    "baseline_false_block_deferred",
    "position_covered",
    "heading_covered",
    "covered",
    "false_permit_covered",
    "false_block_covered",
    "ambiguous",
]


def component_bounds(samples, radius, interpolation):
    assert samples and math.isfinite(radius) and radius >= 0 and interpolation >= 0
    allowance = radius + interpolation
    return {
        f"{part}_{bound}": function(s[part] for s in samples) + sign * allowance
        for part in ["obstacle", "wall"]
        for bound, function, sign in [("lower", min, -1), ("upper", max, 1)]
    }


def gate_state(bounds, margin):
    assert bounds["obstacle_lower"] <= bounds["obstacle_upper"]
    assert bounds["wall_lower"] <= bounds["wall_upper"]
    permit = bounds["obstacle_lower"] > margin and bounds["wall_lower"] >= margin
    blocked = bounds["obstacle_upper"] <= margin or bounds["wall_upper"] < margin
    assert not (permit and blocked)
    return "permit" if permit else "blocked" if blocked else "unresolved"


def radius_for(model, envelope):
    if envelope == "central":
        return 0.0
    field = {
        "noise_envelope": "model_noise_sigma_per_axis",
        "noise_anchor_envelope": "model_noise_anchor_sigma_per_axis",
    }[envelope]
    return 2 * math.sqrt(2) * float(model[field])


def decision_record(previous, model, samples, truth, envelope):
    central = samples[OFFSETS.index(0)]
    radius = radius_for(model, envelope)
    enveloped = envelope != "central"
    interpolation = YAW_INTERPOLATION if enveloped else 0.0
    bounds = component_bounds(
        samples if enveloped else [central], radius, interpolation
    )
    margin = float(previous["margin"])
    state = gate_state(bounds, margin)
    # Truth is evaluation-only, after radius, bounds and classification.
    baseline, true_clear = flag(previous["estimate_clear"]), flag(
        previous["truth_clear"]
    )
    assert classify(central["obstacle"], central["wall"], margin)["clear"] == baseline
    assert classify(truth["obstacle"], truth["wall"], margin)["clear"] == true_clear
    assert (
        abs(min(central["obstacle"], central["wall"]) - float(previous["FF_distance"]))
        < EPS
    )
    assert (
        abs(min(truth["obstacle"], truth["wall"]) - float(previous["TT_distance"]))
        < EPS
    )
    if not enveloped:
        assert (state == "permit") == baseline and state != "unresolved"
    position_error = math.hypot(
        central["pose_x"] - truth["pose_x"], central["pose_y"] - truth["pose_y"]
    )
    yaw = central["pose_yaw"] - truth["pose_yaw"]
    yaw_error = abs(math.degrees(math.atan2(math.sin(yaw), math.cos(yaw))))
    assert abs(position_error - float(model["error_norm"])) < 1e-8
    heading_range = 4.0 if enveloped else 0.0
    position_covered, heading_covered = (
        position_error <= radius,
        yaw_error <= heading_range,
    )
    covered = position_covered and heading_covered
    ambiguous = any(abs(value - margin) <= EPS for value in bounds.values())
    if covered:
        for part in ["obstacle", "wall"]:
            assert bounds[part + "_lower"] <= truth[part] + EPS
            assert truth[part] <= bounds[part + "_upper"] + EPS
        if not ambiguous:
            assert state != "permit" or true_clear
            assert state != "blocked" or not true_clear
    assert state != "permit" or baseline
    assert state != "blocked" or not baseline
    return dict(
        **{k: previous[k] for k in META + ["input_kind"]},
        path=previous["rule"],
        envelope=envelope,
        estimator=model["estimator"],
        prior_optimistic=flag(previous["optimistic"]),
        prior_conservative=flag(previous["conservative"]),
        optimistic_arc_gain=flag(previous["optimistic_arc_gain"]),
        baseline_clear=baseline,
        truth_clear=true_clear,
        used_sample_count=model["used_sample_count"],
        model_noise_sigma_per_axis=model["model_noise_sigma_per_axis"],
        model_noise_anchor_sigma_per_axis=model["model_noise_anchor_sigma_per_axis"],
        radius=radius,
        heading_half_width_deg=heading_range,
        angle_interpolation_bound=interpolation,
        centre_audit_id=central["audit_id"],
        heading_audit_ids="+".join(
            str(s["audit_id"]) for s in (samples if enveloped else [central])
        ),
        truth_source_audit_id=previous["TT_audit_id"],
        **bounds,
        truth_obstacle=truth["obstacle"],
        truth_wall=truth["wall"],
        position_error=position_error,
        yaw_error_deg=yaw_error,
        position_covered=position_covered,
        heading_covered=heading_covered,
        covered=covered,
        state=state,
        permit=state == "permit",
        blocked=state == "blocked",
        unresolved=state == "unresolved",
        true_permit=state == "permit" and true_clear,
        false_permit=state == "permit" and not true_clear,
        true_block=state == "blocked" and not true_clear,
        false_block=state == "blocked" and true_clear,
        unresolved_true_clear=state == "unresolved" and true_clear,
        unresolved_true_blocked=state == "unresolved" and not true_clear,
        baseline_true_permit_deferred=state == "unresolved" and baseline and true_clear,
        baseline_false_permit_deferred=state == "unresolved"
        and baseline
        and not true_clear,
        baseline_true_block_deferred=state == "unresolved"
        and not baseline
        and not true_clear,
        baseline_false_block_deferred=state == "unresolved"
        and not baseline
        and true_clear,
        false_permit_covered=state == "permit" and not true_clear and covered,
        false_block_covered=state == "blocked" and true_clear and covered,
        ambiguous=ambiguous,
    )


def summarize(records):
    groups = defaultdict(list)
    dimensions = ["cohort", "variant", "subset", "input_kind", "path", "envelope"]
    for row in records:
        for cohort in ["all", row["cohort"]]:
            for subset in subsets(row):
                groups[
                    cohort,
                    row["variant"],
                    subset,
                    row["input_kind"],
                    row["path"],
                    row["envelope"],
                ].append(row)
    result = []
    for key, full in sorted(groups.items()):
        for group in GROUPS:
            rows = full if group == "all" else [r for r in full if r[group]]
            record = dict(
                **dict(zip(dimensions, key)),
                group=group,
                checkpoints=len(rows),
                cases=len({tuple(r[k] for k in KEY) for r in rows}),
                **{k: sum(r[k] for r in rows) for k in FLAGS},
                mean_radius=statistics.fmean(r["radius"] for r in rows) if rows else "",
            )
            assert record["permit"] + record["blocked"] + record["unresolved"] == len(
                rows
            )
            assert record["true_permit"] + record["false_permit"] == record["permit"]
            assert record["true_block"] + record["false_block"] == record["blocked"]
            assert (
                record["unresolved_true_clear"] + record["unresolved_true_blocked"]
                == record["unresolved"]
            )
            result.append(record)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    if args.output.exists():
        parser.error("Output directory must be new")
    for name, expected in PINNED.items():
        assert digest(ROOT / "results" / name) == expected, name
    model_source = json.loads(
        (ROOT / "results/epoch-position-components/sources.json").read_text()
    )
    component_source = json.loads(
        (ROOT / "results/commanded-arc-components/sources.json").read_text()
    )
    assert (
        digest(ROOT / "analyze_epoch_position_components.py")
        == model_source["analysis"]["script_sha256"]
    )
    assert (
        digest(ROOT / "analyze_commanded_arc_components.py")
        == component_source["analysis"]["script_sha256"]
    )
    assert digest(SIMULATOR) == model_source["analysis"]["simulator_sha256"]
    for name, expected in model_source["analysis"]["helpers_sha256"].items():
        assert digest(ROOT / name) == expected, name
    map_path = ROOT / "inputs/obstacles.csv"
    assert digest(map_path) == component_source["map_sha256"]
    maps = defaultdict(list)
    for row in read_rows(map_path):
        maps[row["scenario"]].append((float(row["x"]), float(row["y"])))
    accounting = read_rows(ROOT / "results/epoch-position-components/accounting.csv")
    previous = read_rows(ROOT / "results/commanded-arc-components/components.csv")
    audits = {
        r["audit_id"]: {k: float(v) if k != "scenario" else v for k, v in r.items()}
        for r in read_rows(ROOT / "results/commanded-arc-components/geometry-audit.csv")
    }
    models = {
        (identity(r), r["input_kind"]): r
        for r in read_rows(ROOT / "results/epoch-position-components/positions.csv")
    }
    assert (
        len(accounting) == 648 and sum(flag(r["selected"]) for r in accounting) == 368
    )
    assert len(previous) == 8480 and len(models) == 4240
    selected = {tuple(r[k] for k in KEY) for r in accounting if flag(r["selected"])}
    assert selected == {tuple(r[k] for k in KEY) for r in previous}
    cache, records = {}, []
    for row in previous:
        model = models[identity(row), row["input_kind"]]
        assert all(row[k] == model[k] for k in META + ["input_kind"])
        assert float(model["model_noise_anchor_sigma_per_axis"]) >= float(
            model["model_noise_sigma_per_axis"]
        )
        origin, truth = (audits[row[k + "_audit_id"]] for k in ["FF", "TT"])
        pose = tuple(origin[f"pose_{a}"] for a in ["x", "y", "yaw"])
        samples = [
            measured_geometry(
                row["scenario"],
                (*pose[:2], pose[2] + math.radians(offset)),
                origin["steering"],
                maps[row["scenario"]],
                cache,
            )
            for offset in OFFSETS
        ]
        items = [decision_record(row, model, samples, truth, rule) for rule in RULES]
        central, noise, shared = items
        for name in ["obstacle", "wall"]:
            assert (
                shared[name + "_lower"]
                <= noise[name + "_lower"]
                <= central[name + "_lower"]
            )
            assert (
                central[name + "_upper"]
                <= noise[name + "_upper"]
                <= shared[name + "_upper"]
            )
        for wider, narrower in [(noise, central), (shared, noise)]:
            assert wider["state"] == "unresolved" or wider["state"] == narrower["state"]
        records.extend(items)
    source = dict(
        input_sha256=PINNED,
        map_sha256=digest(map_path),
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            simulator_sha256=digest(SIMULATOR),
            helpers_sha256={
                name: digest(ROOT / name)
                for name in sorted(
                    set(model_source["analysis"]["helpers_sha256"])
                    | {"analyze_epoch_position_components.py"}
                )
            },
        ),
        inherited_validation=dict(
            cases=368,
            original_intervals=sum(
                c["reconstructed_intervals"] for c in model_source["cohorts"].values()
            ),
            raw_reloaded=False,
            new_physics_validation=False,
            prior_window_max_reconstruction_error=model_source["audit"][
                "max_reconstruction_error"
            ],
        ),
        audit=dict(
            baseline_rows=len(previous),
            decision_rows=len(records),
            estimated_geometry_requests=len(previous) * len(OFFSETS),
            unique_geometry_audits=len(cache),
            max_chord_error=max(r["chord_error"] for r in cache.values()),
            max_sagitta_bound=max(r["sagitta_bound"] for r in cache.values()),
            ambiguous_rows=sum(r["ambiguous"] for r in records),
            covered_definite_errors=sum(
                r["false_permit_covered"] or r["false_block_covered"] for r in records
            ),
        ),
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", accounting),
        ("decisions", records),
        ("geometry-audit", list(cache.values())),
        ("summary", summarize(records)),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(source["audit"], indent=2))


if __name__ == "__main__":
    main()
