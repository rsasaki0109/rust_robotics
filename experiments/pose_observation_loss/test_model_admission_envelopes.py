"""Conditional pose-disk bounds, three-way predicates and evaluation-only truth."""

import math
import random
import unittest

from analyze_commanded_recovery_arc import META, arc_components, classify
from analyze_model_admission_envelopes import (
    OFFSETS,
    YAW_INTERPOLATION,
    component_bounds,
    decision_record,
    gate_state,
    radius_for,
    summarize,
)


def geometry(pose, steering, obstacles, audit_id=1):
    obstacle, wall = arc_components(pose, steering, obstacles)
    return dict(
        obstacle=obstacle,
        wall=wall,
        pose_x=pose[0],
        pose_y=pose[1],
        pose_yaw=pose[2],
        audit_id=audit_id,
    )


def fixture(estimate, true_pose, obstacles, margin=1):
    samples = [
        geometry((*estimate[:2], estimate[2] + math.radians(offset)), 0, obstacles, i)
        for i, offset in enumerate(OFFSETS)
    ]
    central, truth = samples[OFFSETS.index(0)], geometry(true_pose, 0, obstacles)
    baseline = classify(central["obstacle"], central["wall"], margin)["clear"]
    true_clear = classify(truth["obstacle"], truth["wall"], margin)["clear"]
    previous = dict.fromkeys(META, "0")
    previous.update(
        cohort="known-white",
        variant="epoch",
        mode="random",
        seed="269",
        scenario="fixture",
        planner="A*",
        time_s="150.5",
        reasons="age_0p5",
        input_kind="actual",
        rule="straight",
        margin=str(margin),
        TT_audit_id="1",
        estimate_clear=baseline,
        truth_clear=true_clear,
        optimistic=baseline and not true_clear,
        conservative=not baseline and true_clear,
        optimistic_arc_gain=False,
        FF_distance=min(central["obstacle"], central["wall"]),
        TT_distance=min(truth["obstacle"], truth["wall"]),
    )
    model = dict(
        estimator="fixture",
        used_sample_count=25,
        error_norm=math.dist(estimate[:2], true_pose[:2]),
        model_noise_sigma_per_axis=0.01,
        model_noise_anchor_sigma_per_axis=0.12,
    )
    return previous, model, samples, truth


class ModelAdmissionEnvelopeTests(unittest.TestCase):
    def test_central_reproduces_inclusive_wall_and_strict_obstacle_equality(self):
        wall = decision_record(*fixture((1, 25, 0), (1, 25, 0), [(40, 40)]), "central")
        obstacle = decision_record(
            *fixture((25, 25, 0), (25, 25, 0), [(25.3, 26)]), "central"
        )
        self.assertEqual(wall["state"], "permit")
        self.assertEqual(obstacle["state"], "blocked")
        self.assertTrue(wall["covered"])

    def test_upper_wall_equality_is_unresolved_but_obstacle_equality_blocks(self):
        wall = dict(obstacle_lower=2, obstacle_upper=3, wall_lower=0.9, wall_upper=1)
        obstacle = dict(
            obstacle_lower=0.9, obstacle_upper=1, wall_lower=2, wall_upper=3
        )
        self.assertEqual(gate_state(wall, 1), "unresolved")
        self.assertEqual(gate_state(obstacle, 1), "blocked")

    def test_wider_uncertainty_defers_both_kinds_of_definite_decision(self):
        for distance, expected in [(1.2, "permit"), (0.8, "blocked")]:
            samples = [dict(obstacle=distance, wall=5)]
            narrow = component_bounds(samples, 0.05, 0)
            wide = component_bounds(samples, 0.4, 0)
            self.assertEqual(gate_state(narrow, 1), expected)
            self.assertEqual(gate_state(wide, 1), "unresolved")

    def test_joint_translation_heading_bounds_cover_interior_non_grid_poses(self):
        rng = random.Random(20261016)
        obstacles = [(24.7, 25.8), (25.6, 24.4), (26.1, 26)]
        for steering in [-math.radians(35), 0, math.radians(35)]:
            pose = (25, 25, 0.3)
            samples = [
                geometry((*pose[:2], pose[2] + math.radians(o)), steering, obstacles)
                for o in OFFSETS
            ]
            bounds = component_bounds(samples, 0.12, YAW_INTERPOLATION)
            for _ in range(40):
                angle = rng.uniform(-math.pi, math.pi)
                distance = rng.uniform(0, 0.12)
                truth = (
                    pose[0] + distance * math.cos(angle),
                    pose[1] + distance * math.sin(angle),
                    pose[2] + math.radians(rng.uniform(-4, 4)),
                )
                actual = geometry(truth, steering, obstacles)
                for part in ["obstacle", "wall"]:
                    self.assertLessEqual(bounds[part + "_lower"], actual[part] + 1e-10)
                    self.assertLessEqual(actual[part], bounds[part + "_upper"] + 1e-10)

    def test_half_degree_interpolation_needs_the_geometric_allowance(self):
        yaw = math.radians(0.5)
        obstacle = (25 + 0.6 * math.cos(yaw), 25 + 0.6 * math.sin(yaw))
        samples = [geometry((25, 25, math.radians(o)), 0, [obstacle]) for o in OFFSETS]
        truth = geometry((25, 25, yaw), 0, [obstacle])
        self.assertGreater(
            min(s["obstacle"] for s in samples), truth["obstacle"] + 1e-3
        )
        bounds = component_bounds(samples, 0, YAW_INTERPOLATION)
        self.assertLessEqual(bounds["obstacle_lower"], truth["obstacle"] + 1e-12)

    def test_truth_changes_evaluation_without_changing_the_decision_bounds(self):
        obstacles = [(25.3, 26.5)]
        good = decision_record(
            *fixture((25, 25, 0), (25, 25, 0), obstacles), "noise_envelope"
        )
        outside = decision_record(
            *fixture((25, 25, 0), (25, 25.5, 0), obstacles), "noise_envelope"
        )
        self.assertTrue(good["covered"])
        self.assertFalse(outside["covered"])
        self.assertTrue(outside["false_permit"])
        for field in [
            "state",
            "radius",
            "obstacle_lower",
            "obstacle_upper",
            "wall_lower",
            "wall_upper",
        ]:
            self.assertEqual(good[field], outside[field])

    def test_radius_uses_the_exported_model_not_realized_error_components(self):
        model = dict(
            model_noise_sigma_per_axis=0.02, model_noise_anchor_sigma_per_axis=0.102
        )
        altered = dict(model, error_norm=100, anchor_x=999, noise_x=-999)
        for rule in ["central", "noise_envelope", "noise_anchor_envelope"]:
            self.assertEqual(radius_for(model, rule), radius_for(altered, rule))
        self.assertGreater(
            radius_for(model, "noise_anchor_envelope"),
            radius_for(model, "noise_envelope"),
        )

    def test_shared_anchor_can_defer_a_false_block_instead_of_turning_it_into_permission(
        self,
    ):
        args = fixture((25, 25, 0), (25, 24.7, 0), [(25.3, 25.8)])
        noise = decision_record(*args, "noise_envelope")
        shared = decision_record(*args, "noise_anchor_envelope")
        self.assertTrue(noise["false_block"])
        self.assertEqual(shared["state"], "unresolved")
        self.assertTrue(shared["covered"])
        self.assertTrue(shared["unresolved_true_clear"])

    def test_summary_partitions_states_and_keeps_empty_subgroups(self):
        args = fixture((25, 25, 0), (25, 25, 0), [(25.3, 26.5)])
        first = decision_record(*args, "noise_envelope")
        last = dict(first, time_s="155.5")
        rows = summarize([first, last])
        chosen = {
            r["group"]: r
            for r in rows
            if r["cohort"] == "all" and r["subset"] == "age_0p5"
        }
        self.assertEqual(chosen["all"]["checkpoints"], 2)
        self.assertEqual(chosen["all"]["cases"], 1)
        self.assertEqual(
            chosen["all"]["permit"]
            + chosen["all"]["blocked"]
            + chosen["all"]["unresolved"],
            2,
        )
        self.assertEqual(chosen["optimistic_arc_gain"]["checkpoints"], 0)
        self.assertEqual(chosen["optimistic_arc_gain"]["mean_radius"], "")


if __name__ == "__main__":
    unittest.main()
