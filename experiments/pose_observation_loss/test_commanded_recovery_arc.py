"""Fixed-control curve comparisons, boundary semantics and paired accounting."""

import math
import unittest

from analyze_commanded_recovery_arc import (
    META,
    arc_components,
    classify,
    decisions_at,
    inspect_pose,
    summarize,
    transitions_at,
)
from analyze_joint_admission_geometry import geometry


def pose_row(pose, steering, obstacles, kind="truth", margin=0.89, floor=0.88):
    return dict(
        scenario="fixture",
        pose_kind=kind,
        pose_x=pose[0],
        pose_y=pose[1],
        pose_yaw=pose[2],
        nominal_steering=steering,
        margin=margin,
        zero_age_margin=floor,
        recovery_zero_age_clear=geometry(pose, steering, floor, obstacles)[
            "recovery_clear"
        ],
        **geometry(pose, steering, margin, obstacles),
    )


def fixture(estimate, truth, obstacles, steering=math.radians(35)):
    common = dict.fromkeys(META, "0")
    common.update(
        cohort="known-white",
        variant="curved",
        mode="random",
        seed="270",
        scenario="fixture",
        planner="Theta*",
        time_s="150.500000",
        reasons="first_usable_epoch+age_0p5",
    )
    poses = dict(
        sensed=estimate,
        recovery=estimate,
        preview=estimate,
        truth_xy_sensed_yaw=(*truth[:2], estimate[2]),
        truth_xy_recovery_yaw=(*truth[:2], estimate[2]),
        truth=truth,
    )
    cache = {}
    rows = [
        inspect_pose(pose_row(p, steering, obstacles, k), obstacles, cache)
        for k, p in poses.items()
    ]
    return common, rows


class CommandedArcTests(unittest.TestCase):
    def test_wall_minimum_includes_the_interior_and_permits_equality(self):
        pose, angle = (49.5, 25, math.pi / 2 - 0.3), math.atan(0.4)
        obstacle, wall = arc_components(pose, angle, [(40, 40)])
        self.assertAlmostEqual(wall, math.cos(0.3) - 0.5)
        self.assertGreater(obstacle, wall)
        self.assertTrue(classify(obstacle, wall, wall)["clear"])
        self.assertFalse(classify(wall, obstacle, wall)["clear"])

    def test_clockwise_reflection_preserves_obstacle_and_wall_components(self):
        left = arc_components((25, 25, 0.2), 0.4, [(25.6, 25.8)])
        right = arc_components((25, 25, -0.2), -0.4, [(25.6, 24.2)])
        for a, b in zip(left, right):
            self.assertAlmostEqual(a, b)

    def test_zero_steering_preserves_geometry_and_all_paired_decisions(self):
        common, rows = fixture((25, 25, 0), (25, 24.85, 0), [(25.6, 24.2)], 0)
        self.assertTrue(all(r["arc_delta"] == 0 for r in rows))
        changes = transitions_at(decisions_at(common, rows))
        self.assertTrue(
            all(
                not r[k]
                for r in changes
                for k in ["estimate_gain", "estimate_loss", "truth_gain", "truth_loss"]
            )
        )

    def test_recorded_turn_can_clear_an_obstacle_blocking_the_line(self):
        obstacles = [(25.6, 24.2)]
        row = inspect_pose(
            pose_row((25, 25, 0), math.radians(35), obstacles), obstacles, {}
        )
        self.assertFalse(row["recovery_clear"])
        self.assertTrue(row["arc_clear"])

    def test_recorded_turn_can_lose_a_line_that_was_clear(self):
        obstacles = [(25.6, 26)]
        row = inspect_pose(
            pose_row((25, 25, 0), math.radians(35), obstacles), obstacles, {}
        )
        self.assertTrue(row["recovery_clear"])
        self.assertFalse(row["arc_clear"])

    def test_geometry_cache_does_not_reuse_the_age_dependent_decision(self):
        obstacles, cache = [(25, 25.89)], {}
        a = inspect_pose(
            pose_row((25, 25, 0), 0, obstacles, margin=0.9), obstacles, cache
        )
        b = inspect_pose(
            pose_row((25, 25, 0), 0, obstacles, margin=0.88), obstacles, cache
        )
        self.assertEqual(len(cache), 1)
        self.assertFalse(a["arc_clear"])
        self.assertTrue(b["arc_clear"])

    def test_zero_age_refresh_and_the_current_margin_stay_separate(self):
        obstacles = [(25, 25.89)]
        row = inspect_pose(
            pose_row((25, 25, 0), 0, obstacles, margin=0.9), obstacles, {}
        )
        self.assertFalse(row["arc_clear"])
        self.assertTrue(row["arc_zero_age_clear"])

    def test_estimated_gain_can_be_optimistic_without_physical_contact(self):
        common, rows = fixture((25, 25, 0), (25, 24.85, 0), [(25.6, 24.2)])
        decisions = decisions_at(common, rows)
        actual = next(
            r
            for r in decisions
            if r["input_kind"] == "actual" and r["rule"] == "commanded"
        )
        self.assertTrue(actual["optimistic"])
        self.assertFalse(actual["accepted_physical_violation"])
        change = transitions_at(decisions)[0]
        self.assertTrue(change["estimate_gain"])
        self.assertTrue(change["estimated_gain_truth_blocked"])
        self.assertFalse(change["truth_gain"])

    def test_true_curve_gain_does_not_imply_an_estimated_permission(self):
        common, rows = fixture((25, 24.7, 0), (25, 25, 0), [(25.6, 24.2)])
        decisions = decisions_at(common, rows)
        actual = next(
            r
            for r in decisions
            if r["input_kind"] == "actual" and r["rule"] == "commanded"
        )
        self.assertTrue(actual["conservative"])
        self.assertTrue(actual["heading_evaluable"])
        self.assertTrue(
            all(
                not r["heading_evaluable"]
                for r in decisions
                if r["input_kind"] == "preview"
            )
        )
        change = transitions_at(decisions)[0]
        self.assertTrue(change["truth_gain"])
        self.assertFalse(change["estimate_gain"])

    def test_overlapping_subsets_keep_case_and_checkpoint_denominators(self):
        common, rows = fixture((25, 25, 0), (25, 25, 0), [(25.6, 24.2)])
        first = decisions_at(common, rows)
        last = [dict(r, time_s="180.000000", reasons="endpoint") for r in first]
        summary = summarize(first + last)
        selected = {
            r["subset"]: r
            for r in summary
            if r["cohort"] == "all"
            and r["input_kind"] == "actual"
            and r["rule"] == "commanded"
        }
        self.assertEqual(selected["all"]["checkpoints"], 2)
        self.assertEqual(selected["all"]["cases"], 1)
        self.assertEqual(selected["first_usable_epoch"]["checkpoints"], 1)
        self.assertEqual(selected["age_0p5"]["checkpoints"], 1)
        self.assertEqual(selected["endpoint"]["checkpoints"], 1)


if __name__ == "__main__":
    unittest.main()
