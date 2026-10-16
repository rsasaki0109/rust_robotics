"""Geometry boundaries, positive-speed probes and continuous admission checks."""

import math
import unittest

from analyze_joint_admission_geometry import (
    checkpoint,
    decision_records,
    geometry,
    tail_readiness,
)
from test_epoch_divergence import row
from verify_stopping_preview import exact_speed_distance


def joint_track(count):
    track = [row(i * 0.02) for i in range(count + 1)]
    for item in track:
        item.update(safety_paused="true", nominal_target_speed="0.4")
    return track


def probe_fixture(obstacles):
    before, current = row(), row(0.02)
    before.update(x="10", y="10", yaw="0", safety_paused="true")
    current.update(
        recovery_x="10",
        recovery_y="10",
        recovery_yaw="0",
        preview_pose_x="10",
        preview_pose_y="10",
        preview_pose_yaw="0",
        nominal_steering="0",
        recovery_sample_count="0",
        effective_position_sigma="0",
        observation_age="0.5",
        speed_margin="0.61",
        uncapped_target_speed="0.4",
    )
    full = exact_speed_distance(before, current, obstacles)
    current["speed_free_distance"] = str(full)
    current["nominal_target_speed"] = str(
        min(0.4, max(0, min(1, math.sqrt(0.0225 + full) - 0.15)))
    )
    current["recovery_clearance"] = str(
        geometry((10, 10, 0), 0, 0.61, obstacles)["recovery_obstacle"]
    )
    return before, current


class AdmissionGeometryTests(unittest.TestCase):
    def test_recovery_wall_equality_is_inclusive_but_speed_is_strict(self):
        result = geometry((1, 25, 0), 0, 1, [])
        self.assertTrue(result["recovery_clear"])
        self.assertFalse(result["speed_geometry_clear"])
        self.assertEqual(result["recovery_slack"], 0)
        self.assertTrue(result["recovery_ambiguous"])

    def test_obstacle_equality_blocks_both_gates(self):
        result = geometry((10, 10, 0), 0, 1, [(10, 11)])
        self.assertFalse(result["recovery_clear"])
        self.assertFalse(result["speed_geometry_clear"])
        self.assertEqual(result["recovery_obstacle"], 1)

    def test_first_chord_can_block_speed_even_with_a_clear_start(self):
        obstacles = [(10.525, 10)]
        result = geometry((10, 10, 0), 0, 0.5, obstacles)
        self.assertGreater(result["center_clearance"], 0.5)
        self.assertFalse(result["speed_geometry_clear"])
        before, current = probe_fixture(obstacles)
        current["speed_margin"] = ".5"
        self.assertEqual(exact_speed_distance(before, current, obstacles), 0)

    def test_positive_first_chord_does_not_claim_two_meters_are_clear(self):
        obstacles = [(11, 10)]
        result = geometry((10, 10, 0), 0, 0.5, obstacles)
        self.assertTrue(result["speed_geometry_clear"])
        before, current = probe_fixture(obstacles)
        current["speed_margin"] = ".5"
        full = exact_speed_distance(before, current, obstacles)
        self.assertGreater(full, 0)
        self.assertLess(full, 2)

    def test_start_below_margin_blocks_every_considered_direction(self):
        for yaw in [-math.pi / 2, 0, math.pi / 2]:
            result = geometry((10, 10, yaw), 0.4, 0.8, [(10, 10.7)])
            self.assertLess(result["center_clearance"], 0.8)
            self.assertFalse(result["recovery_clear"])
            self.assertFalse(result["speed_geometry_clear"])

    def test_zero_samples_use_a_valid_raw_fallback_pose(self):
        before, current = probe_fixture([(11.5, 10)])
        common, values = checkpoint(before, current, [(11.5, 10)])
        self.assertFalse(common["recovery_averaged"])
        self.assertFalse(common["base_usable"])
        self.assertEqual(len(values), 6)
        self.assertTrue(
            next(r for r in values if r["pose_kind"] == "recovery")["recovery_clear"]
        )

    def test_goal_stop_remains_zero_when_geometry_allows_speed(self):
        before, current = probe_fixture([(11.5, 10)])
        current.update(uncapped_target_speed="0", nominal_target_speed="0")
        common, values = checkpoint(before, current, [(11.5, 10)])
        self.assertTrue(common["goal_command_zero"])
        self.assertFalse(common["speed_geometry_zero"])
        rows = [r for r in decision_records(common, values) if r["gate"] == "speed"]
        self.assertTrue(
            all(not r["estimate_clear"] and not r["truth_clear"] for r in rows)
        )


class AdmissionReadinessTests(unittest.TestCase):
    def test_two_raw_gates_can_jointly_block_when_either_mask_is_insufficient(self):
        track = joint_track(50)
        for item in track:
            item.update(recovery_clearance="0.5", nominal_target_speed="0")
        result, _ = tail_readiness(track)
        self.assertEqual(result["actual_max_ready_s"], 0)
        self.assertEqual(result["without_recovery_max_ready_s"], 0)
        self.assertEqual(result["without_speed_max_ready_s"], 0)
        self.assertEqual(result["without_raw_gates_max_ready_s"], 1)
        self.assertEqual(result["base_only_max_ready_s"], 1)

    def test_stale_or_missing_samples_reset_even_the_base_only_run(self):
        track = joint_track(61)
        track[31].update(observation_age="1.6", recovery_sample_count="0")
        result, _ = tail_readiness(track)
        self.assertEqual(result["base_only_max_ready_s"], 0.6)
        self.assertEqual(result["fresh_fail_s"], 0.02)
        self.assertEqual(result["samples_fail_s"], 0.02)

    def test_joint_route_end_does_not_extend_its_confirmation_window(self):
        track = joint_track(70)
        for item in track[50:]:
            item.update(safety_paused="false", guard_active="false")
        track[50]["resumed"] = "true"
        result, _ = tail_readiness(track)
        self.assertEqual(result["eligible_s"], 1)
        self.assertEqual(result["actual_max_ready_s"], 1)

    def test_first_usable_epoch_and_age_half_second_are_deduplicated(self):
        track = joint_track(25)
        for i, item in enumerate(track):
            item.update(
                observation_age=str(i * 0.02), recovery_sample_count=str(min(25, i + 1))
            )
        result, choices = tail_readiness(track)
        self.assertEqual(result["usable_epochs"], 1)
        self.assertEqual(choices[9], ["first_usable_epoch"])
        self.assertEqual(set(choices[25]), {"endpoint", "age_0p5"})
        self.assertEqual(len(choices), 2)

    def test_precision_age_reset_starts_a_new_sampling_epoch(self):
        track = joint_track(50)
        for i, item in enumerate(track):
            age_ticks = i if i < 26 else i - 26
            item.update(
                observation_age=str(age_ticks * 0.02),
                recovery_sample_count=str(min(25, age_ticks + 1)),
            )
        result, choices = tail_readiness(track)
        self.assertEqual(result["usable_epochs"], 2)
        self.assertIn("first_usable_epoch", choices[9])
        self.assertIn("first_usable_epoch", choices[35])


if __name__ == "__main__":
    unittest.main()
