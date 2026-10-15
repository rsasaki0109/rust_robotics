"""Pin cohort selection, pre-command epochs and paired feasibility labels."""

import math
import unittest

from analyze_aligned_arcs import paired_row, select_cases, selected_pairs


class AlignedArcTests(unittest.TestCase):
    def test_selection_excludes_known_moving_and_late_entry_cases(self):
        rows = [
            dict(cohort="white-validation", classification="no_entry_stationary"),
            dict(cohort="correlated-validation", classification="no_entry_stationary"),
            dict(cohort="known-white", classification="no_entry_stationary"),
            dict(cohort="white-validation", classification="no_entry_moving"),
            dict(cohort="correlated-validation", classification="late_entry"),
        ]
        self.assertEqual(select_cases(rows), rows[:2])

    @staticmethod
    def track(t=180, age=0.02, speed=0):
        before = dict(time_s=str(t - 0.02), x="25", y="26", yaw="0.1", v=str(speed))
        row = dict(
            time_s=str(t),
            x="25.1",
            y="26.1",
            yaw="0.2",
            observation_age=str(age),
            preview_pose_x="24.9",
            preview_pose_y="25.8",
            preview_pose_yaw="0.09",
        )
        return [before, row]

    def test_final_truth_uses_prior_state_not_endpoint_or_filtered_state(self):
        before, row, reason, poses = list(selected_pairs(self.track()))[0]
        self.assertEqual(reason, "final")
        self.assertAlmostEqual(float(before["time_s"]), 179.98)
        self.assertEqual(float(row["time_s"]), 180)
        self.assertEqual(
            dict(poses), dict(truth=(25, 26, 0.1), filtered=(24.9, 25.8, 0.09))
        )

    def test_fresh_checkpoint_requires_fixed_time_age_and_stationary_speed(self):
        self.assertEqual(len(list(selected_pairs(self.track(150.5, 0.5, 0.02)))), 1)
        for values in [
            (150, 0.5, 0),
            (150.52, 0.5, 0),
            (150.5, 0.7, 0),
            (150.5, 0.5, 0.020001),
        ]:
            self.assertEqual(list(selected_pairs(self.track(*values))), [])

    @staticmethod
    def record(kind, state):
        return dict(
            cohort="white-validation",
            mode="none",
            seed="265",
            scenario="maze_50x50",
            planner="A*",
            time_s="180",
            decision_time_s="179.98",
            reason="final",
            pose_kind=kind,
            x=25.0,
            y=25.0,
            yaw=math.pi - 0.01,
            motion_filter_sample_count=1,
            grid_max_clearance=0.8,
            recovery_zero_age_status=state,
            recovery_current_age_status=state,
            recovery_zero_age_margin=0.85,
            recovery_current_age_margin=0.86,
        )

    def test_unresolved_true_arcs_are_not_certified_blocked_pairs(self):
        truth = self.record("truth", "unresolved")
        filtered = self.record("filtered", "grid_pass")
        self.assertFalse(
            paired_row([truth, filtered])["filtered_pass_truth_blocked_zero_age"]
        )
        truth.update(recovery_zero_age_status="all_fixed_steering_blocked")
        self.assertTrue(
            paired_row([truth, filtered])["filtered_pass_truth_blocked_zero_age"]
        )

    def test_pairing_is_order_independent_and_wraps_heading_error(self):
        truth = self.record("truth", "grid_pass")
        filtered = dict(
            self.record("filtered", "grid_pass"), x=25.3, y=25.4, yaw=-math.pi + 0.01
        )
        result = paired_row([filtered, truth])
        self.assertAlmostEqual(result["position_error"], 0.5)
        self.assertAlmostEqual(result["yaw_error_rad"], 0.02)
        with self.assertRaises(AssertionError):
            paired_row([truth, truth])

    def test_two_pose_arc_witnesses_need_not_use_the_same_steering(self):
        truth = dict(
            self.record("truth", "grid_pass"),
            grid_max_clearance=1,
            best_clearance=1,
            best_steering_deg=0,
        )
        filtered = dict(
            self.record("filtered", "grid_pass"),
            grid_max_clearance=1.1,
            best_clearance=1.1,
            best_steering_deg=30,
        )
        result = paired_row([truth, filtered], {0: 1, 30: 0.7})
        self.assertTrue(result["filtered_selected_passes_zero_age"])
        self.assertFalse(result["true_selected_passes_zero_age"])
        self.assertEqual(result["true_clearance_at_filtered_best"], 0.7)


if __name__ == "__main__":
    unittest.main()
