"""Command source, strict boundaries, adjacent routes and matched error changes."""

import math
import unittest

import analyze_correction_gate_boundaries as analysis
from test_correction_gate_sequences import context


def fixture():
    before = dict(x="10", y="10", yaw="0", v="0", time_s="149.980000")
    row = dict(
        time_s="150.000000",
        noise_x="0",
        noise_y="0",
        noise_yaw="0",
        bias_x="0",
        bias_y="0",
        bias_yaw="0",
        effective_position_sigma=".1",
        observation_age=".5",
        measured_speed="0",
        recovery_sample_count="25",
        recovery_x="10",
        recovery_y="10",
        recovery_yaw="0",
        preview_pose_x="10",
        preview_pose_y="10",
        preview_pose_yaw="0",
        recovery_clearance="2",
        stop_preview_clearance_lower_bound="2",
        terminal_recovery_clearance="2",
        guard_release_margin="1",
        speed_margin="1",
        nominal_target_speed=".4",
        target_speed="0",
        uncapped_target_speed=".4",
        speed_cap=".4",
        speed_free_distance=".28",
        nominal_steering="0",
        stop_latched="false",
        restart_limited="false",
        sensed_goal_distance="5",
        recovery_preview_active="true",
        terminal_steering_preview="false",
    )
    return before, row


def transition(label="joint_lost"):
    return dict(
        transition_id=1,
        event_id="1",
        phase="post",
        end_offset_steps=10,
        label=label,
        pre_state_id=1,
        post_state_id=2,
    )


def old_current(failed="positive_command"):
    return dict(
        route="joint",
        failed_mask=str(1 << analysis.REQUIREMENTS.index(failed)),
        failed_requirements=failed,
        timer_before_s=".38",
    )


def event():
    return dict(
        event_id="1",
        cohort="known-white",
        variant="epoch",
        event_kind="received",
        event_route="joint",
        mode="none",
        seed="269",
        scenario="fixture",
        planner="A*",
        partition="joint_short_run",
        post_window_censored=True,
    )


class CorrectionGateBoundaryTests(unittest.TestCase):
    def test_all_seven_transition_labels_keep_successful_route_exit_distinct(self):
        cases = [
            (context(), context(), "joint_retained"),
            (context(), context(failed=["stop"]), "joint_lost"),
            (context(failed=["stop"]), context(), "joint_acquired"),
            (context(failed=["stop"]), context(failed=["terminal"]), "joint_not_ready"),
            (context(resumed=True), context("inactive"), "joint_route_exit"),
            (context("inactive"), context(), "joint_route_entry"),
            (context("independent"), context("inactive"), "other"),
        ]
        for a, b, label in cases:
            self.assertEqual(analysis.label_pair(a, b), label)

    def test_nominal_zero_reasons_separate_uncapped_speed_cap_and_both(self):
        cases = [
            (0.28, 0.4, 0.4, 0.4, "positive"),
            (0.28, 0.4, 0, 0, "uncapped_zero"),
            (0, 0, 0.4, 0, "speed_cap_zero"),
            (0, 0, 0, 0, "both_zero"),
        ]
        for free, cap, uncapped, nominal, reason in cases:
            self.assertEqual(
                analysis.nominal_command(free, cap, uncapped, nominal)[0], reason
            )

    def test_rounding_bucket_does_not_claim_either_positive_input_is_exactly_zero(self):
        free = 1e-10
        cap = math.sqrt(0.0225 + free) - 0.15
        self.assertGreater(cap, 0)
        self.assertEqual(
            analysis.nominal_command(free, cap, 0.4, 0)[0], "rounded_zero_other"
        )

    def test_corrupted_speed_cap_or_nominal_minimum_is_rejected(self):
        for values in [(0, 0.1, 0.4, 0.1), (0.28, 0.4, 0.2, 0.3)]:
            with self.assertRaises(AssertionError):
                analysis.nominal_command(*values)

    def test_zero_paused_target_does_not_make_the_positive_command_condition_false(
        self,
    ):
        before, row = fixture()
        state, error = analysis.signed_state(before, row, context())
        self.assertEqual(state["actual_target_speed"], 0)
        self.assertGreater(state["nominal_target_speed"], 0)
        self.assertEqual(state["command_zero_reason"], "positive")
        self.assertLess(error, analysis.EPS)

    def test_raw_pose_uses_preintegration_truth_plus_noise_and_bias(self):
        before, row = fixture()
        row.update(x="99", y="99", noise_x=".2", bias_x=".1")
        state, _ = analysis.signed_state(before, row, context())
        self.assertAlmostEqual(state["raw_x"], 10.3)
        self.assertEqual(state["truth_x"], 10)
        self.assertEqual(state["truth_time_s"], "149.980000")

    def test_stationary_inclusive_boundary_uses_precise_measured_speed(self):
        before, row = fixture()
        row["measured_speed"] = ".02"
        state, _ = analysis.signed_state(before, row, context())
        self.assertEqual(state["stationary_gap"], 0)
        row["measured_speed"] = ".020000000000000004"
        with self.assertRaises(AssertionError):
            analysis.signed_state(before, row, context())
        state, _ = analysis.signed_state(before, row, context(failed=["stationary"]))
        self.assertLess(state["stationary_gap"], 0)

    def test_strict_stop_and_terminal_equalities_fail(self):
        before, row = fixture()
        row.update(
            stop_preview_clearance_lower_bound="1", terminal_recovery_clearance="1.05"
        )
        state, _ = analysis.signed_state(
            before, row, context(failed=["stop", "terminal"])
        )
        self.assertEqual(state["stop_gap"], 0)
        self.assertEqual(state["terminal_gap"], 0)

    def test_geometric_depth_preserves_wall_equality_and_one_centimetre_boundary(self):
        state = dict.fromkeys(analysis.DISTANCE_GAPS, 0.1)
        state["recovery_wall_gap"] = 0
        self.assertEqual(analysis.geometric_depth(state), "no_geometric_failure")
        state["stop_gap"] = 0
        self.assertEqual(analysis.geometric_depth(state), "within_1cm")
        state["terminal_gap"] = -0.01
        self.assertEqual(analysis.geometric_depth(state), "within_1cm")
        state["terminal_gap"] = -0.01000001
        self.assertEqual(analysis.geometric_depth(state), "beyond_1cm")

    def test_equal_physical_and_estimated_motion_does_not_change_error(self):
        before, row = fixture()
        a, _ = analysis.signed_state(before, row, context())
        b = dict(a)
        for k in ["truth", "raw", "recovery", "preview"]:
            b[k + "_x"] += 0.2
        record = analysis.change_record(transition(), a, b, old_current())
        self.assertAlmostEqual(record["truth_shift_m"], 0.2)
        for k in ["raw", "recovery", "preview"]:
            self.assertAlmostEqual(record[k + "_error_change_m"], 0)

    def test_raw_noise_and_bias_change_must_close_to_raw_error_change(self):
        before, row = fixture()
        a, _ = analysis.signed_state(before, row, context())
        b = dict(a)
        b["raw_x"] += 0.2
        b["noise_x"] += 0.15
        b["bias_x"] += 0.05
        record = analysis.change_record(transition(), a, b, old_current())
        self.assertAlmostEqual(record["delta_raw_error_x"], 0.2)
        b["bias_x"] += 0.1
        with self.assertRaises(AssertionError):
            analysis.change_record(transition(), a, b, old_current())

    def test_heading_change_wraps_across_pi(self):
        before, row = fixture()
        a, _ = analysis.signed_state(before, row, context())
        b = dict(a)
        a["preview_yaw"] = math.radians(179)
        b["preview_yaw"] = math.radians(-179)
        record = analysis.change_record(transition(), a, b, old_current())
        self.assertAlmostEqual(record["preview_yaw_change_deg"], 2)

    def test_geometry_probe_uses_raw_pose_while_release_geometry_uses_other_inputs(
        self,
    ):
        before, row = fixture()
        row.update(
            noise_x="5",
            speed_free_distance="0",
            speed_cap="0",
            nominal_target_speed="0",
            recovery_clearance="4.5",
            stop_preview_clearance_lower_bound="5.1",
            terminal_recovery_clearance="4.5",
        )
        state, _ = analysis.signed_state(
            before, row, context(failed=["positive_command"])
        )
        state.update(state_id=1, source_interval_id="1")
        result = analysis.geometry_record(state, before, row, [(15.1, 10)])
        self.assertEqual(result["speed_free_exact"], 0)
        self.assertAlmostEqual(result["stop_exact"], 5.1)
        self.assertAlmostEqual(result["terminal_exact"], 4.5)
        self.assertAlmostEqual(result["recovery_obstacle_exact"], 4.5)

    def test_invalid_geometry_and_lower_bound_are_rejected(self):
        before, row = fixture()
        row.update(
            speed_free_distance="2",
            speed_cap="1",
            nominal_target_speed="0",
            uncapped_target_speed="0",
            stop_latched="true",
        )
        state, _ = analysis.signed_state(
            before, row, context(failed=["positive_command"])
        )
        state.update(state_id=1, source_interval_id="1")
        row["stop_preview_clearance_lower_bound"] = "2.01"
        with self.assertRaises(AssertionError):
            analysis.geometry_record(state, before, row, [(10, 12)])

    def test_summary_keeps_censored_source_denominator_and_empty_change_groups(self):
        t = transition()
        t["detailed"] = True
        before, row = fixture()
        a, _ = analysis.signed_state(before, row, context())
        c = analysis.change_record(t, a, a, old_current())
        summary, matched, patterns = analysis.summarize([event()], [t], [c])
        r = next(
            r
            for r in summary
            if (
                r["cohort"],
                r["variant"],
                r["event_kind"],
                r["event_route"],
                r["phase"],
            )
            == ("all", "epoch", "received", "all", "post")
        )
        self.assertEqual(
            (
                r["source_events"],
                r["censored_source_events"],
                r["observed_events"],
                r["joint_lost"],
            ),
            (1, 1, 1, 1),
        )
        empty = [r for r in matched if r["event_kind"] == "precision_only"]
        self.assertTrue(empty)
        self.assertTrue(
            all(r["pairs"] == 0 and r["pre_raw_error_rms_m"] == "" for r in empty)
        )
        self.assertTrue(patterns)


if __name__ == "__main__":
    unittest.main()
