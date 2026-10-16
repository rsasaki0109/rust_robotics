#!/usr/bin/env python3
"""Compare ideal information batches at pinned centres, headings and margins."""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics

from analyze_commanded_recovery_arc import META, flag, identity
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import EPS, KEY, subsets
from analyze_model_admission_envelopes import YAW_INTERPOLATION
from analyze_terminal_arc_live import SIMULATOR
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PROTOCOL = ROOT / "results/admission-information-floors/protocol.md"
PINNED = {
    "epoch-position-components/positions.csv": "67fe28537f1e84d47ee8711b11f3df0cc52f8b770e694da0055d6265604f321e",
    "epoch-position-components/sources.json": "01e6b94b68c892621047829585df1db46c6471e7a3dfdd819c40e7fa6fe136db",
    "model-admission-envelopes/accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "model-admission-envelopes/decisions.csv": "7eeaa32e09c5235c70bd9e95db9b3998f68f835ff392036b554c56536395b394",
    "model-admission-envelopes/geometry-audit.csv": "ffae1282b46e9da0e792840a1d5eae65c9219dfe98c864bae149fc491b32a6f3",
    "model-admission-envelopes/sources.json": "b13e005d5e0e327c8951519823dd73c9f8916573f68ce647d33635b4d8ad1313",
}
STRATEGIES = ["same_anchors", "independent_anchors"]
BATCHES = [1, 3, 10, 25, None]
STATUSES = ["certified_clear", "some_pose_blocks", "angle_gap"]
BOTTLENECKS = [
    "already_certified",
    "fixed_centre_not_clear",
    "heading_interval_blocks",
    "heading_grid_gap",
    "shared_anchor_floor",
    "anchor_floor_grid_gap",
    "noise_limit_certifies",
]
GROUPS = ["all", "prior_optimistic", "prior_conservative", "optimistic_arc_gain"]
FLAGS = [
    *STATUSES,
    "new_certified_clear",
    "fixed_true_permit",
    "fixed_false_permit",
    "new_fixed_true_permit",
    "new_fixed_false_permit",
    "recorded_pose_covered",
    "lost_recorded_coverage",
    "recorded_true_clear_uncertified",
    "recorded_true_blocked_uncertified",
]


def information_radius(noise_variance, anchor_variance, strategy, batches):
    assert strategy in STRATEGIES
    assert all(math.isfinite(v) and v >= 0 for v in [noise_variance, anchor_variance])
    assert batches is None or isinstance(batches, int) and batches >= 1
    divisor = math.inf if batches is None else batches
    variance = (
        anchor_variance + noise_variance / divisor
        if strategy == "same_anchors"
        else (noise_variance + anchor_variance) / divisor
    )
    return 2 * math.sqrt(2) * math.sqrt(variance)


def radius_caps(samples, margin, interpolation=YAW_INTERPOLATION):
    assert samples and interpolation >= 0
    return {
        f"{part}_{kind}_cap": min(s[part] for s in samples) - margin - allowance
        for part in ["obstacle", "wall"]
        for kind, allowance in [("sufficient", interpolation), ("necessary", 0.0)]
    }


def radius_status(caps, radius):
    assert math.isfinite(radius) and radius >= 0
    clear = (
        radius < caps["obstacle_sufficient_cap"]
        and radius <= caps["wall_sufficient_cap"]
    )
    blocks = (
        radius >= caps["obstacle_necessary_cap"] or radius > caps["wall_necessary_cap"]
    )
    assert not (clear and blocks)
    return "certified_clear" if clear else "some_pose_blocks" if blocks else "angle_gap"


def bottleneck(caps, current_radius, floor_radius, central_clear):
    assert floor_radius <= current_radius + EPS
    if radius_status(caps, current_radius) == "certified_clear":
        assert central_clear
        return "already_certified"
    if not central_clear:
        assert radius_status(caps, 0) == "some_pose_blocks"
        return "fixed_centre_not_clear"
    zero = radius_status(caps, 0)
    if zero == "some_pose_blocks":
        return "heading_interval_blocks"
    if zero == "angle_gap":
        return "heading_grid_gap"
    floor = radius_status(caps, floor_radius)
    return {
        "some_pose_blocks": "shared_anchor_floor",
        "angle_gap": "anchor_floor_grid_gap",
        "certified_clear": "noise_limit_certifies",
    }[floor]


def strategy_records(base, caps, noise_variance, anchor_variance):
    """Decide all radii before evaluating the unchanged recorded pose error."""
    decisions = []
    previous = {}
    for strategy in STRATEGIES:
        for batches in BATCHES:
            radius = information_radius(
                noise_variance, anchor_variance, strategy, batches
            )
            status = radius_status(caps, radius)
            if strategy in previous:
                wider_radius, wider_status = previous[strategy]
                assert radius <= wider_radius
                assert wider_status != "certified_clear" or status == "certified_clear"
                assert (
                    status != "some_pose_blocks" or wider_status == "some_pose_blocks"
                )
            previous[strategy] = radius, status
            decisions.append((strategy, batches, radius, status))
    paired = {
        (strategy, batches): (radius, status)
        for strategy, batches, radius, status in decisions
    }
    for batches in BATCHES:
        same, independent = (paired[s, batches] for s in STRATEGIES)
        assert independent[0] <= same[0]
        assert same[1] != "certified_clear" or independent[1] == "certified_clear"
        assert independent[1] != "some_pose_blocks" or same[1] == "some_pose_blocks"
    result = []
    for strategy, batches, radius, status in decisions:
        # Recorded truth/error is used only for this fixed-centre evaluation.
        truth = base["recorded_truth_clear"]
        covered = (
            base["recorded_position_error"] <= radius
            and base["recorded_yaw_error_deg"] <= 4
        )
        permit = status == "certified_clear"
        new = permit and not base["baseline_certified_clear"]
        assert not (permit and covered and not truth)
        result.append(
            dict(
                base_row=base["base_row"],
                strategy=strategy,
                batches="limit" if batches is None else str(batches),
                radius=radius,
                status=status,
                **{k: status == k for k in STATUSES},
                new_certified_clear=new,
                fixed_true_permit=permit and truth,
                fixed_false_permit=permit and not truth,
                new_fixed_true_permit=new and truth,
                new_fixed_false_permit=new and not truth,
                recorded_pose_covered=covered,
                lost_recorded_coverage=base["baseline_recorded_pose_covered"]
                and not covered,
                recorded_true_clear_uncertified=not permit and truth,
                recorded_true_blocked_uncertified=not permit and not truth,
            )
        )
    return result


def grouped_rows(records, include_strategy):
    dimensions = ["cohort", "variant", "subset", "input_kind", "path"]
    if include_strategy:
        dimensions += ["strategy", "batches"]
    groups = defaultdict(list)
    for row in records:
        for cohort in ["all", row["cohort"]]:
            for subset in subsets(row):
                key = (
                    cohort,
                    row["variant"],
                    subset,
                    *(row[k] for k in dimensions[3:]),
                )
                groups[key].append(row)
    for key, full in sorted(groups.items()):
        for group in GROUPS:
            rows = full if group == "all" else [r for r in full if r[group]]
            yield dict(zip(dimensions, key)), group, rows


def summarize(bases, curves):
    by_id = {r["base_row"]: r for r in bases}
    merged = [{**by_id[r["base_row"]], **r} for r in curves]
    result = []
    for dimensions, group, rows in grouped_rows(merged, True):
        record = dict(
            **dimensions,
            group=group,
            checkpoints=len(rows),
            cases=len({tuple(r[k] for k in KEY) for r in rows}),
            **{k: sum(r[k] for r in rows) for k in FLAGS},
            mean_radius=statistics.fmean(r["radius"] for r in rows) if rows else "",
        )
        assert sum(record[k] for k in STATUSES) == len(rows)
        assert (
            record["fixed_true_permit"] + record["fixed_false_permit"]
            == record["certified_clear"]
        )
        assert (
            record["new_fixed_true_permit"] + record["new_fixed_false_permit"]
            == record["new_certified_clear"]
        )
        assert (
            record["recorded_true_clear_uncertified"]
            + record["recorded_true_blocked_uncertified"]
            == len(rows) - record["certified_clear"]
        )
        result.append(record)
    floors = []
    for dimensions, group, rows in grouped_rows(bases, False):
        record = dict(
            **dimensions,
            group=group,
            checkpoints=len(rows),
            cases=len({tuple(r[k] for k in KEY) for r in rows}),
            **{k: sum(r["bottleneck"] == k for r in rows) for k in BOTTLENECKS},
        )
        assert sum(record[k] for k in BOTTLENECKS) == len(rows)
        floors.append(record)
    return result, floors


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
    prior_source = json.loads(
        (ROOT / "results/model-admission-envelopes/sources.json").read_text()
    )
    model_source = json.loads(
        (ROOT / "results/epoch-position-components/sources.json").read_text()
    )
    helpers = dict(prior_source["analysis"]["helpers_sha256"])
    helpers["analyze_model_admission_envelopes.py"] = prior_source["analysis"][
        "script_sha256"
    ]
    assert (
        helpers["analyze_epoch_position_components.py"]
        == model_source["analysis"]["script_sha256"]
    )
    for name, expected in helpers.items():
        assert digest(ROOT / name) == expected, name
    assert digest(SIMULATOR) == prior_source["analysis"]["simulator_sha256"]
    map_path = ROOT / "inputs/obstacles.csv"
    assert digest(map_path) == prior_source["map_sha256"]
    prior = read_rows(ROOT / "results/model-admission-envelopes/decisions.csv")
    by_kind = {
        (identity(r), r["input_kind"], r["path"], r["envelope"]): r for r in prior
    }
    accounting = read_rows(ROOT / "results/model-admission-envelopes/accounting.csv")
    geometry = {
        r["audit_id"]: r
        for r in read_rows(
            ROOT / "results/model-admission-envelopes/geometry-audit.csv"
        )
    }
    models = {
        (identity(r), r["input_kind"]): r
        for r in read_rows(ROOT / "results/epoch-position-components/positions.csv")
    }
    assert len(prior) == len(by_kind) == 25440 and len(models) == 4240
    assert (
        len(accounting) == 648 and sum(flag(r["selected"]) for r in accounting) == 368
    )
    bases, curves, radius_errors = [], [], []
    for source_row, row in enumerate(prior, 1):
        if row["envelope"] != "noise_anchor_envelope":
            continue
        model = models[identity(row), row["input_kind"]]
        assert all(row[k] == model[k] for k in META + ["input_kind"])
        ids = row["heading_audit_ids"].split("+")
        assert len(ids) == 9 and ids[4] == row["centre_audit_id"]
        samples = [
            {k: float(geometry[i][k]) for k in ["obstacle", "wall"]} for i in ids
        ]
        centre = geometry[ids[4]]
        for offset, audit_id in zip(range(-4, 5), ids):
            audit = geometry[audit_id]
            assert (
                audit["scenario"] == row["scenario"]
                and audit["steering"] == centre["steering"]
            )
            assert (
                abs(
                    math.degrees(float(audit["pose_yaw"]) - float(centre["pose_yaw"]))
                    - offset
                )
                < 1e-10
            )
            assert all(audit[k] == centre[k] for k in ["pose_x", "pose_y"])
        margin = float(row["margin"])
        caps = radius_caps(samples, margin)
        noise_variance = float(model["model_noise_sigma_per_axis"]) ** 2
        anchor_variance = float(model["model_anchor_sigma_per_axis"]) ** 2
        assert (
            abs(
                math.sqrt(noise_variance + anchor_variance)
                - float(model["model_noise_anchor_sigma_per_axis"])
            )
            < 1e-12
        )
        current_radius = information_radius(
            noise_variance, anchor_variance, "same_anchors", 1
        )
        floor_radius = information_radius(
            noise_variance, anchor_variance, "same_anchors", None
        )
        radius_errors.append(abs(current_radius - float(row["radius"])))
        assert radius_errors[-1] < 1e-12
        noise_reference = by_kind[
            identity(row), row["input_kind"], row["path"], "noise_envelope"
        ]
        assert (
            abs(
                information_radius(noise_variance, 0, "same_anchors", 1)
                - float(noise_reference["radius"])
            )
            < 1e-12
        )
        assert (
            abs(
                floor_radius
                - 2 * math.sqrt(2) * float(model["model_anchor_sigma_per_axis"])
            )
            < 1e-12
        )
        for part in ["obstacle", "wall"]:
            assert (
                abs(
                    caps[part + "_sufficient_cap"]
                    - (float(row[part + "_lower"]) - margin + float(row["radius"]))
                )
                < EPS
            )
        current_status = radius_status(caps, current_radius)
        baseline_permit, central_clear = flag(row["permit"]), flag(
            row["baseline_clear"]
        )
        assert (current_status == "certified_clear") == baseline_permit
        category = bottleneck(caps, current_radius, floor_radius, central_clear)
        # These recorded error and truth fields are not arguments to the classification.
        base = dict(
            base_row=len(bases) + 1,
            source_row=source_row,
            **{k: row[k] for k in META + ["input_kind", "path", "estimator"]},
            **{k: flag(row[k]) for k in GROUPS if k != "all"},
            used_sample_count=model["used_sample_count"],
            source_group_count=model["source_group_count"],
            stochastic_anchor_count=model["stochastic_anchor_count"],
            model_noise_sigma_per_axis=model["model_noise_sigma_per_axis"],
            model_anchor_sigma_per_axis=model["model_anchor_sigma_per_axis"],
            model_noise_anchor_sigma_per_axis=model[
                "model_noise_anchor_sigma_per_axis"
            ],
            noise_variance=noise_variance,
            anchor_variance=anchor_variance,
            anchor_variance_fraction=(
                anchor_variance / (noise_variance + anchor_variance)
                if noise_variance + anchor_variance
                else 0.0
            ),
            current_radius=current_radius,
            same_anchor_limit_radius=floor_radius,
            current_status=current_status,
            same_anchor_limit_status=radius_status(caps, floor_radius),
            zero_radius_status=radius_status(caps, 0),
            central_clear=central_clear,
            baseline_certified_clear=baseline_permit,
            bottleneck=category,
            source_centre_audit_id=row["centre_audit_id"],
            source_heading_audit_ids=row["heading_audit_ids"],
            angle_interpolation_bound=YAW_INTERPOLATION,
            **caps,
            recorded_truth_clear=flag(row["truth_clear"]),
            recorded_position_error=float(row["position_error"]),
            recorded_yaw_error_deg=float(row["yaw_error_deg"]),
            baseline_recorded_pose_covered=flag(row["covered"]),
        )
        items = strategy_records(base, caps, noise_variance, anchor_variance)
        for item in items:
            if item["batches"] == "1":
                assert item["certified_clear"] == baseline_permit
                assert (
                    item["recorded_pose_covered"]
                    == base["baseline_recorded_pose_covered"]
                )
        bases.append(base)
        curves.extend(items)
    assert len(bases) == 8480 and len(curves) == 84800
    assert {tuple(r[k] for k in KEY) for r in accounting if flag(r["selected"])} == {
        tuple(r[k] for k in KEY) for r in bases
    }
    summary, floor_summary = summarize(bases, curves)
    source = dict(
        input_sha256=PINNED,
        map_sha256=digest(map_path),
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            simulator_sha256=digest(SIMULATOR),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        fixed_information_model=dict(
            batches=["limit" if k is None else k for k in BATCHES],
            strategies=STRATEGIES,
            independent_noise_between_batches=True,
            estimated_centres_unchanged=True,
            new_observation_or_wait_prediction=False,
        ),
        inherited_validation=prior_source["inherited_validation"],
        inherited_geometry_validation={
            k: prior_source["audit"][k]
            for k in ["unique_geometry_audits", "max_chord_error", "max_sagitta_bound"]
        },
        audit=dict(
            base_rows=len(bases),
            curve_rows=len(curves),
            summary_rows=len(summary),
            bottleneck_summary_rows=len(floor_summary),
            max_K1_radius_difference=max(radius_errors),
            new_geometry_evaluations=0,
            covered_fixed_false_permits=sum(
                r["fixed_false_permit"] and r["recorded_pose_covered"] for r in curves
            ),
            bottleneck_counts={
                k: sum(r["bottleneck"] == k for r in bases) for k in BOTTLENECKS
            },
        ),
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", accounting),
        ("checkpoints", bases),
        ("curves", curves),
        ("summary", summary),
        ("bottleneck-summary", floor_summary),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(source["audit"], indent=2))


if __name__ == "__main__":
    main()
