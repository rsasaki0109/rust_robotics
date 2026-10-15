"""Analytic geometry and interaction fixtures for reentry input attribution."""

import unittest

from analyze_reentry_inputs import decompose, hybrid_preview, shapley


class ReentryInputTests(unittest.TestCase):
    def test_known_additive_changes_and_dummy_inputs(self):
        values = [2 + 3 * bool(m & 1) - 5 * bool(m & 2) for m in range(16)]
        result = shapley(values)
        for name, expected in zip(result, [3, -5, 0, 0]):
            self.assertAlmostEqual(result[name], expected)

    def test_pure_interaction_splits_equally_without_order_bias(self):
        result = shapley([8 * bool(m & 1) * bool(m & 2) for m in range(16)])
        self.assertAlmostEqual(result["physical_pose"], 4)
        self.assertAlmostEqual(result["observation_error"], 4)
        self.assertEqual(result["initial_speed"], 0)
        self.assertEqual(result["planned_command"], 0)

    def test_full_four_way_interaction(self):
        result = shapley([24 if m == 15 else 0 for m in range(16)])
        for value in result.values():
            self.assertAlmostEqual(value, 6)

    def test_stationary_translation_and_error_have_known_clearance_changes(self):
        start = dict(
            physical_pose=(25, 25, 0),
            observation_error=(0, 0, 0),
            initial_speed=0,
            planned_command=(0, 0),
            base_margin=0.4,
            speed_reserve=0,
        )
        end = dict(
            start,
            physical_pose=(25.1, 25, 0),
            observation_error=(0.1, 0, 0),
            base_margin=0.41,
            speed_reserve=0.02,
        )
        values, changes, freezes, first, last = decompose(start, end, [(25.5, 25)])
        self.assertAlmostEqual(values[0], 0.5)
        self.assertAlmostEqual(values[15], 0.3)
        self.assertAlmostEqual(changes["physical_pose"], -0.1)
        self.assertAlmostEqual(changes["observation_error"], -0.1)
        self.assertAlmostEqual(changes["base_margin"], -0.01)
        self.assertAlmostEqual(changes["speed_reserve"], -0.02)
        self.assertAlmostEqual(first, 0.1)
        self.assertAlmostEqual(last, -0.13)
        self.assertAlmostEqual(freezes["physical_pose"], -0.03)

    def test_moving_forecast_checks_obstacle_along_segment(self):
        state = dict(
            physical_pose=(25, 25, 0),
            observation_error=(0, 0, 0),
            initial_speed=1,
            planned_command=(1, 0),
        )
        self.assertAlmostEqual(hybrid_preview(state, state, 0, [(25.15, 25.2)]), 0.2)


if __name__ == "__main__":
    unittest.main()
