"""Check causal prefix attribution, exact pose substitutions and gate runs."""

from copy import deepcopy
import unittest

from analyze_epoch_divergence import (
    first_fork,
    geometry,
    pose_views,
    release_inputs,
    selected_checkpoints,
    tail_summary,
)


def row(time=0):
    return dict(
        scenario="maze_50x50",
        planner="Theta*",
        time_s=str(time),
        x="25",
        y="25",
        yaw="0",
        v="0",
        measured_speed="0",
        nominal_target_speed="0",
        nominal_steering="0",
        target_speed="0",
        steering="0",
        acceleration="0",
        motion_filter_pose_x="24.5",
        motion_filter_pose_y="25",
        preview_pose_x="24.5",
        preview_pose_y="25",
        preview_pose_yaw="0",
        preview_margin="0.9",
        guard_release_margin="0.95",
        stop_preview_clearance_lower_bound="1.7",
        terminal_recovery_clearance="1.1",
        preview_clearance_lower_bound="1.0",
        release_preview_clearance_lower_bound="1.05",
        recovery_preview_active="true",
        aligned_terminal_release="true",
        observation_age="0.5",
        effective_position_sigma="0",
        noise_x="0",
        noise_y="0",
        noise_yaw="0",
        bias_x="0",
        bias_y="0",
        bias_yaw="0",
        correction="false",
        correction_received="false",
        precision_activation="false",
        stop_latched="false",
        recovery_x="25",
        recovery_y="25",
        recovery_yaw="0",
        recovery_sample_count="25",
        recovery_clearance="1.5",
        guard_active="true",
        guard_triggered="false",
        guard_released="false",
        safety_paused="false",
        resumed="false",
        epoch_fusion_available="false",
        guard_release_evidence="0",
        recovery_evidence="0",
        speed_cap="1",
        uncapped_target_speed="1",
    )


def historical_release():
    reference = [row(i * 0.02) for i in range(3)]
    candidate = deepcopy(reference)
    reference[1].update(
        stop_preview_clearance_lower_bound="0.8",
        preview_clearance_lower_bound="0.8",
        release_preview_clearance_lower_bound="0.8",
    )
    candidate[1].update(epoch_fusion_available="true", guard_release_evidence="0.48")
    reference[2]["guard_release_evidence"] = "0.02"
    candidate[2].update(
        guard_active="false", guard_released="true", target_speed="0.5", v="0.01"
    )
    return reference, candidate


class EpochDivergenceTests(unittest.TestCase):
    def test_precise_speed_preserves_the_strict_stationarity_gate(self):
        before, current = row(), row(0.02)
        before["v"] = "0.020000000"
        current["measured_speed"] = "2.00000000000000039e-2"
        route, required = release_inputs(before, current)
        self.assertEqual(route, "independent")
        self.assertFalse(required["stationary"])

    def test_joint_gate_keeps_raw_recovery_and_terminal_requirements(self):
        before, current = row(), row(0.02)
        before["safety_paused"] = "true"
        current.update(
            nominal_target_speed="0.5",
            recovery_sample_count="9",
            terminal_recovery_clearance="1.0",
            release_preview_clearance_lower_bound="0.95",
        )
        route, required = release_inputs(before, current)
        self.assertEqual(route, "joint")
        self.assertFalse(required["samples"])
        self.assertTrue(required["stop"])
        self.assertFalse(required["terminal"])
        self.assertTrue(required["recovery_geometry"])
        self.assertTrue(required["positive_command"])

    def test_better_position_can_correctly_block_an_optimistic_preview(self):
        before, current = row(), row(0.02)
        views = {r["pose_kind"]: r for r in geometry(before, current, [(26.2, 25)])}
        self.assertTrue(views["logged"]["entry_clear"])
        self.assertFalse(views["truth"]["entry_clear"])
        self.assertAlmostEqual(views["logged"]["position_error"], 0.5)
        self.assertEqual(views["truth"]["position_error"], 0)
        self.assertAlmostEqual(views["truth"]["terminal_exact"], 0.6)

    def test_true_position_substitution_preserves_the_estimated_heading(self):
        before, current = row(), row(0.02)
        before["yaw"] = "0.7"
        current["preview_pose_yaw"] = "0.2"
        views = pose_views(before, current)
        self.assertEqual(views["true_xy"], (25, 25, 0.2))
        self.assertEqual(views["truth"], (25, 25, 0.7))
        self.assertEqual(views["original"], views["logged"])

    def test_prior_evidence_can_release_with_equal_current_geometry_and_no_fusion(self):
        reference, candidate = historical_release()
        indices, fork = first_fork(reference, candidate, "0.04")
        self.assertEqual(indices["gate"], 1)
        self.assertEqual(indices["evidence"], 1)
        self.assertEqual(indices["action"], 2)
        self.assertEqual(indices["physical"], 2)
        self.assertEqual(fork["first_actor"], "epoch")
        self.assertTrue(fork["history_only_release"])
        self.assertFalse(fork["fusion_available_at_action"])

    def test_prefix_with_different_nominal_inputs_is_not_a_shared_state_comparison(
        self,
    ):
        reference, candidate = historical_release()
        candidate[1]["nominal_steering"] = "0.01"
        with self.assertRaises(AssertionError):
            first_fork(reference, candidate, "0.04")

    def test_release_origin_excludes_an_earlier_difference_that_was_reset(self):
        reference = [row(i * 0.02) for i in range(5)]
        candidate = deepcopy(reference)
        for index in [1, 2, 3]:
            reference[index].update(
                stop_preview_clearance_lower_bound="0.8",
                release_preview_clearance_lower_bound="0.8",
            )
        candidate[1].update(
            epoch_fusion_available="true", guard_release_evidence="0.02"
        )
        candidate[2].update(
            stop_preview_clearance_lower_bound="0.8",
            release_preview_clearance_lower_bound="0.8",
        )
        candidate[3].update(
            epoch_fusion_available="true", guard_release_evidence="0.48"
        )
        reference[4]["guard_release_evidence"] = "0.02"
        candidate[4].update(
            guard_active="false", guard_released="true", target_speed="0.5", v="0.01"
        )
        indices, fork = first_fork(reference, candidate, "0.08")
        self.assertEqual(indices["gate"], 1)
        self.assertEqual(indices["origin"], 3)
        self.assertEqual(fork["origin_to_action_s"], 0.02)

    def test_overlapping_gate_failures_and_motion_break_consecutive_upper_bound(self):
        track = [row(i * 0.02) for i in range(5)]
        for current in track[1:]:
            current.update(
                stop_preview_clearance_lower_bound="0.8",
                release_preview_clearance_lower_bound="0.8",
            )
        track[3]["measured_speed"] = "0.03"
        summary, gates = tail_summary(track, 0.08)
        independent = gates[0]
        self.assertEqual(summary["wait_s"], 0.08)
        self.assertEqual(independent["stop_fail_s"], 0.08)
        self.assertEqual(independent["stationary_fail_s"], 0.02)
        self.assertEqual(independent["max_actual_ready_s"], 0)
        self.assertEqual(independent["max_without_stop_ready_s"], 0.04)
        self.assertEqual(independent["max_without_terminal_ready_s"], 0)

    def test_checkpoint_selection_deduplicates_and_uses_each_arm_endpoint(self):
        first = [row(i * 0.02) for i in range(5)]
        second = [row(i * 0.02) for i in range(6)]
        chosen = selected_checkpoints([first, second], dict(action=2, command=2))
        self.assertEqual(chosen[0][2], ["first_action", "first_command"])
        self.assertIn("own_endpoint", chosen[0][4])
        self.assertIn("common_endpoint", chosen[1][4])
        self.assertIn("own_endpoint", chosen[1][5])
        self.assertIn("first_fresh_stationary_in_final_window", chosen[1][1])

    def test_last_failed_gate_handles_a_timer_lead_reversal_without_equality(self):
        reference = [row(i * 0.02) for i in range(5)]
        candidate = deepcopy(reference)
        reference[1].update(
            stop_preview_clearance_lower_bound="0.8",
            release_preview_clearance_lower_bound="0.8",
        )
        candidate[1]["guard_release_evidence"] = "0.02"
        reference[2]["guard_release_evidence"] = "0.02"
        candidate[2].update(
            stop_preview_clearance_lower_bound="0.8",
            release_preview_clearance_lower_bound="0.8",
        )
        reference[3]["guard_release_evidence"] = "0.04"
        candidate[3]["guard_release_evidence"] = "0.02"
        reference[4].update(
            guard_active="false", guard_released="true", target_speed="0.5", v="0.01"
        )
        candidate[4]["guard_release_evidence"] = "0.04"
        indices, fork = first_fork(reference, candidate, "0.08")
        self.assertEqual(indices["origin"], 1)
        self.assertEqual(indices["decisive"], 2)
        self.assertTrue(fork["history_only_release"])
        self.assertEqual(fork["first_actor"], "aligned")

    def test_direct_entry_uses_the_current_geometry_without_a_release_timer(self):
        reference = [row(), row(0.02)]
        candidate = deepcopy(reference)
        for track in [reference, candidate]:
            track[0]["guard_active"] = "false"
            track[1]["nominal_target_speed"] = "0.5"
        reference[1].update(guard_active="false", target_speed="0.5", v="0.01")
        candidate[1].update(
            guard_triggered="true",
            preview_clearance_lower_bound="0.8",
            epoch_fusion_available="true",
        )
        indices, fork = first_fork(reference, candidate, "0.02")
        self.assertEqual(fork["kind"], "entry")
        self.assertEqual(fork["first_actor"], "epoch")
        self.assertEqual(indices["decisive"], indices["action"])
        self.assertEqual(fork["winning_ready_run_s"], 0)


if __name__ == "__main__":
    unittest.main()
