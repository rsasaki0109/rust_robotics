"""Endpoint-budget, timeline and pose-substitution fixtures for the diagnosis."""

import copy
import unittest

from analyze_filter_stalls import (
    category,
    command_spans,
    filtered_speed_distance,
    hold_budget,
)
from verify_stopping_preview import exact_speed_distance


class FilterStallTests(unittest.TestCase):
    def test_first_endpoint_can_still_credit_ten_seconds_at_170_02(self):
        self.assertAlmostEqual(hold_budget(170.02), 10)
        self.assertEqual(category(170.02, 0), "earlier_entry_without_hold")

    def test_next_endpoint_is_too_late_even_under_ideal_hold_conditions(self):
        self.assertAlmostEqual(hold_budget(170.04), 9.98)
        self.assertEqual(category(170.04, 0), "late_entry")

    def test_no_entry_and_strict_stationary_distance_boundary(self):
        self.assertEqual(hold_budget(-1), 0)
        self.assertEqual(category(-1, 0.099), "no_entry_stationary")
        self.assertEqual(category(-1, 0.1), "no_entry_moving")
        self.assertEqual(category(160, 0), "earlier_entry_without_hold")

    def test_timeline_uses_command_intervals_and_merges_only_adjacent_states(self):
        def row(t, loss, guard):
            return dict(
                time_s=str(t),
                safety_paused=loss,
                guard_active=guard,
                stop_latched="false",
                speed_cap="1",
                uncapped_target_speed="1",
            )

        track = [
            row(0, "true", "true"),
            row(0.02, "true", "true"),
            row(0.04, "true", "false"),
            row(0.06, "true", "false"),
        ]
        self.assertEqual(
            command_spans(track),
            [
                dict(start_s=0, duration_s=0.02, state="both"),
                dict(start_s=0.02, duration_s=0.04, state="loss_only"),
            ],
        )

    def test_speed_probe_substitutes_pose_without_changing_margin_or_inputs(self):
        before = dict(x="25.2", y="25", yaw="0", v="0")
        row = dict(
            preview_pose_x="25",
            preview_pose_y="25",
            preview_pose_yaw="0",
            nominal_steering="0",
            speed_margin="0.85",
        )
        row.update(
            {
                f"{kind}_{axis}": "0"
                for kind in ["noise", "bias"]
                for axis in ["x", "y", "yaw"]
            }
        )
        original = copy.deepcopy((before, row))
        obstacles = [(26.02, 25)]
        self.assertEqual(exact_speed_distance(before, row, obstacles), 0)
        self.assertAlmostEqual(filtered_speed_distance(before, row, obstacles), 0.15)
        self.assertEqual((before, row), original)


if __name__ == "__main__":
    unittest.main()
