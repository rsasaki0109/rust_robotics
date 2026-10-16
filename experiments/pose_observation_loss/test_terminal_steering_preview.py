"""Forecast and geometry checks for the frozen terminal-steering comparison."""

import math
import unittest

from analyze_recovery_arcs import arc_clearance, arc_point, chord_check
from analyze_terminal_steering_preview import (
    IDENTITY,
    decision_records,
    slacks,
    stopping_endpoint,
    subsets,
    summarize_transitions,
)


class TerminalForecastTests(unittest.TestCase):
    def test_stationary_forecast_still_has_eleven_steps(self):
        pose = (10.0, 20.0, 0.0)
        end, steps = stopping_endpoint(pose, 0.0, 0.0, math.radians(35))
        self.assertEqual(end, pose)
        self.assertEqual(steps, 11)

    def test_straight_stopping_distance_from_geometric_series(self):
        # Below .25 speed, braking stays proportional. Integrate the two
        # geometric series analytically, independently of the Euler loop.
        initial = 0.1
        after_probe = initial * 0.98**10
        braking_steps = math.ceil(math.log(0.001 / after_probe) / math.log(0.92))
        expected = 0.02 * (
            initial * (1 - 0.98**10) / (1 - 0.98)
            + after_probe * (1 - 0.92**braking_steps) / (1 - 0.92)
        )
        end, steps = stopping_endpoint((10, 20, 0), initial, 0, 0)
        self.assertAlmostEqual(end[0] - 10, expected, places=12)
        self.assertEqual(end[1:], (20, 0))
        self.assertEqual(steps, 10 + braking_steps)

    def test_nominal_acceleration_changes_forecast_from_rest(self):
        end, steps = stopping_endpoint((10, 20, 0), 0, 1, math.radians(35))
        self.assertGreater(end[0], 10)
        self.assertGreater(end[1], 20)
        self.assertGreater(end[2], 0)
        self.assertGreater(steps, 11)

    def test_turn_direction_reflects_forecast(self):
        left, a = stopping_endpoint((10, 20, 0), 0.7, 1, 0.5)
        right, b = stopping_endpoint((10, 20, 0), 0.7, 1, -0.5)
        self.assertEqual(a, b)
        self.assertAlmostEqual(left[0], right[0])
        self.assertAlmostEqual(left[1] - 20, 20 - right[1])
        self.assertAlmostEqual(left[2], -right[2])

    def test_invalid_or_unstoppable_inputs_fail(self):
        for speed, steering in [(math.nan, 0), (1, math.radians(36)), (100, 0)]:
            with self.subTest(speed=speed, steering=steering):
                with self.assertRaises(AssertionError):
                    stopping_endpoint((10, 20, 0), speed, 1, steering)


class TerminalGeometryTests(unittest.TestCase):
    def test_curve_can_both_gain_and_lose_physical_clearance(self):
        pose, steering = (10, 10, 0), math.radians(35)
        below, above = [(10.6, 9.5)], [(10.6, 10.5)]
        straight = arc_clearance(pose, 0, below)
        self.assertAlmostEqual(straight, 0.5)
        self.assertGreater(arc_clearance(pose, steering, below), 0.6)
        self.assertLess(arc_clearance(pose, steering, above), 0.5)
        for obstacles in [below, above]:
            exact = arc_clearance(pose, steering, obstacles)
            error, bound = chord_check(pose, steering, obstacles, exact)
            self.assertLessEqual(error, bound + 1e-9)

    def test_wall_minimum_can_be_inside_arc(self):
        pose = (10, 49.8, 3 * math.pi / 4)
        steering = math.radians(35)
        curvature = math.tan(steering) / 0.4
        expected = 0.2 - (1 + math.cos(pose[2])) / curvature
        exact = arc_clearance(pose, steering, [])
        end = arc_point(pose, steering, 0.6)
        self.assertAlmostEqual(exact, expected)
        self.assertLess(exact, min(50 - pose[1], 50 - end[1]))
        chord_check(pose, steering, [], exact)

    def test_near_zero_steering_converges_to_straight(self):
        pose, obstacles = (10, 10, 0.3), [(10.4, 10.7), (12, 13)]
        straight = arc_clearance(pose, 0, obstacles)
        for steering in [-1e-12, 1e-12, -1e-9, 1e-9]:
            self.assertAlmostEqual(
                arc_clearance(pose, steering, obstacles), straight, places=8
            )

    def test_stop_prefix_can_still_block_a_clear_terminal_arc(self):
        score = slacks(0.7, 2.0, 0.8, 0.85)
        self.assertLess(score["entry"], 0)
        self.assertLess(score["release"], 0)
        # Equal safety thresholds in the aligned policy, with both terminal
        # offsets retained; entry/release do not accidentally use radius .5.
        score = slacks(2.0, 0.95, 0.8, 0.85)
        self.assertAlmostEqual(score["entry"], 0.05)
        self.assertAlmostEqual(score["release"], 0.05)


class TerminalDecisionTests(unittest.TestCase):
    def fixture(self):
        shared = dict.fromkeys(IDENTITY, "fixture")
        shared.update(
            measured_speed="0",
            nominal_target_speed="1",
            nominal_steering="0.5",
            entry_margin="0.8",
            release_margin="0.85",
            preview_steps="50",
            stop_exact="2",
            straight_terminal="1",
            nominal_terminal="1",
            entry_slack="0.1",
            release_slack="0.1",
            nominal_entry_slack="0.1",
            nominal_release_slack="0.1",
        )
        return [
            dict(shared, pose_kind=name)
            for name in ["logged", "original", "true_xy", "truth"]
        ]

    def test_truth_evaluates_same_command_and_exposes_optimistic_arc(self):
        rows = self.fixture()
        rows[-1].update(
            nominal_entry_slack="-0.5",
            nominal_release_slack="-0.5",
            nominal_terminal="0.4",
        )
        result = decision_records(rows)
        curve = [r for r in result if r["rule"] == "nominal"]
        self.assertTrue(all(r["optimistic"] for r in curve))
        self.assertTrue(all(r["heading_changes_truth"] for r in curve))
        self.assertTrue(all(r["accepted_physical_violation"] for r in curve))
        self.assertTrue(all(r["nominal_steering"] == "0.5" for r in result))

    def test_pose_evaluation_rejects_a_changed_control(self):
        rows = self.fixture()
        rows[-1]["nominal_steering"] = "0.2"
        with self.assertRaises(AssertionError):
            decision_records(rows)

    def test_duplicate_pose_cannot_hide_missing_truth(self):
        rows = self.fixture()
        rows[-1]["pose_kind"] = "logged"
        with self.assertRaises(AssertionError):
            decision_records(rows)

    def test_overlapping_endpoint_reasons_are_deduplicated(self):
        self.assertEqual(
            subsets({"reasons": "common_endpoint+own_endpoint+first_decisive"}),
            ["all", "decisive", "endpoints"],
        )

    def test_transition_counts_keep_optimistic_and_justified_gains_separate(self):
        rows = self.fixture()
        rows[0].update(entry_slack="-0.1", release_slack="-0.1")
        rows[-1].update(nominal_entry_slack="-0.1", nominal_release_slack="0.1")
        summary = summarize_transitions(decision_records(rows))
        all_rows = {r["gate"]: r for r in summary if r["cohort"] == "all"}
        self.assertEqual(all_rows["entry"]["estimate_gained_pass"], 1)
        self.assertEqual(all_rows["entry"]["gained_true_block"], 1)
        self.assertEqual(all_rows["entry"]["gained_true_clear"], 0)
        self.assertEqual(all_rows["release"]["gained_true_block"], 0)
        self.assertEqual(all_rows["release"]["gained_true_clear"], 1)


if __name__ == "__main__":
    unittest.main()
