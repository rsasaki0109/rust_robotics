"""Hand-calculated interval and censoring cases for the offline diagnosis."""

import unittest

from analyze_entry_cycles import physical_intervals, release_bouts, wait_seconds


def row(time):
    return dict(
        time_s=str(time),
        safety_paused="false",
        guard_active="false",
        guard_triggered="false",
        guard_released="false",
        target_speed="1",
    )


class EntryCycleTests(unittest.TestCase):
    def test_travel_includes_release_interval_and_excludes_reentry_interval(self):
        track = [row(0), row(0.02), row(0.04), row(0.06)]
        track[1]["guard_released"] = "true"
        track[3]["guard_triggered"] = "true"
        # Release starts at the initial point. The final .01 of travel occurs
        # under the new brake command, so it is outside the release bout.
        bouts = release_bouts(track, [0, 0, 0.006, 0.016])
        self.assertEqual(len(bouts), 1)
        self.assertAlmostEqual(bouts[0]["travel_before_entry"], 0.006)
        self.assertEqual(bouts[0]["lag_s"], 0.04)
        self.assertTrue(bouts[0]["short_reentry"])

    def test_one_second_inclusive_and_distance_threshold_exclusive(self):
        track = [row(0), row(0.02), row(1), row(1.02)]
        track[1]["guard_released"] = "true"
        track[3]["guard_triggered"] = "true"
        self.assertTrue(release_bouts(track, [0, 0, 0.099, 0.11])[0]["short_reentry"])
        self.assertFalse(release_bouts(track, [0, 0, 0.1, 0.11])[0]["short_reentry"])

    def test_loss_interruption_is_retained_but_not_short_reentry(self):
        track = [row(0), row(0.02), row(0.04), row(0.06)]
        track[1]["guard_released"] = "true"
        track[2]["safety_paused"] = "true"
        track[3]["guard_triggered"] = "true"
        bout = release_bouts(track, [0, 0, 0, 0])[0]
        self.assertTrue(bout["loss_interrupted"])
        self.assertFalse(bout["short_reentry"])

    def test_zero_target_release_is_not_a_failed_acceleration(self):
        track = [row(0), row(0.02), row(0.04)]
        track[1].update(guard_released="true", target_speed="0")
        track[2]["guard_triggered"] = "true"
        bout = release_bouts(track, [0, 0, 0])[0]
        self.assertFalse(bout["release_positive_target"])
        self.assertFalse(bout["short_reentry"])

    def test_initial_entry_and_unfinished_final_release_are_not_complete_bouts(self):
        track = [row(0), row(0.02), row(0.04)]
        track[1]["guard_triggered"] = "true"
        track[2]["guard_released"] = "true"
        self.assertEqual(release_bouts(track, [0, 0, 0]), [])

    def test_wait_counts_union_at_inclusive_common_horizon(self):
        track = [row(0), row(0.02), row(0.04), row(0.06)]
        track[0]["guard_active"] = "true"  # No initial command interval.
        track[1].update(guard_active="true", safety_paused="true")
        track[2]["safety_paused"] = "true"
        track[3]["guard_active"] = "true"
        self.assertEqual(wait_seconds(track, 0.04), 0.04)

    def test_known_straight_step_and_reject_misaligned_physical_record(self):
        start = dict(time_s="0", x="25", y="25", yaw="0", v="1")
        end = dict(
            time_s="0.02",
            x="25.02",
            y="25",
            yaw="0",
            v="1",
            acceleration="0",
            steering="0",
        )
        self.assertAlmostEqual(physical_intervals([start, end])[-1], 0.02)
        with self.assertRaises(AssertionError):
            physical_intervals([start, dict(end, time_s="0.04")])
        with self.assertRaises(AssertionError):
            physical_intervals([start, dict(end, x="25.01")])


if __name__ == "__main__":
    unittest.main()
