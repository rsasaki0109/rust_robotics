"""Pin causal epoch identity, retention, equal weights and truth-free selection."""

import math
import unittest

from analyze_stationary_epoch_fusion import StationaryEpochFusion, select_arc


def epoch(estimator, time, position=0, count=25):
    for i in range(count):
        estimator.observe(
            time + i * 0.02, (position, 0, 0), 0, 0, i == 0, False, i * 0.02
        )


def ready():
    estimator = StationaryEpochFusion()
    epoch(estimator, 0)
    epoch(estimator, 5)
    epoch(estimator, 10, count=1)
    return estimator


class StationaryEpochFusionTests(unittest.TestCase):
    def test_equal_epoch_weights_do_not_overweight_an_old_long_window(self):
        estimator = StationaryEpochFusion()
        epoch(estimator, 0, position=0)
        epoch(estimator, 5, position=3)
        epoch(estimator, 10, position=9, count=1)
        self.assertEqual(estimator.position(), (4, 0))
        self.assertEqual([g["samples"] for g in estimator.means()], [25, 25, 1])

    def test_missing_correction_is_not_an_independent_epoch_or_fresh_data(self):
        estimator = ready()
        estimator.observe(15, (100, 0, 0), 0, 0, False, False, 5)
        self.assertEqual(len(estimator.groups), 3)
        self.assertEqual(estimator.groups[-1]["epoch"], 10)
        self.assertIsNone(estimator.position())
        self.assertEqual(estimator.groups[-1]["samples"][-1][0], 0)

    def test_fifteen_second_horizon_is_inclusive_then_expires(self):
        estimator = StationaryEpochFusion()
        epoch(estimator, 0)
        epoch(estimator, 5)
        epoch(estimator, 15, count=1)
        self.assertEqual(estimator.position(), (0, 0))
        estimator.observe(15.02, (0, 0, 0), 0, 0, False, False, 0.02)
        self.assertIsNone(estimator.position())
        self.assertEqual(len(estimator.groups), 2)

    def test_motion_threshold_and_precision_change_discard_stationary_history(self):
        estimator = ready()
        estimator.observe(10.02, (0, 0, 0), 0.02, 0, False, False, 0.02)
        self.assertIsNotNone(estimator.position())
        estimator.observe(10.04, (0, 0, 0), 0.020001, 0, False, False, 0.04)
        self.assertEqual(len(estimator.groups), 0)
        self.assertIsNone(estimator.position())
        estimator = ready()
        estimator.observe(10.02, (1, 0, 0), 0, 0, False, True, 0)
        self.assertEqual(len(estimator.groups), 1)
        self.assertEqual(estimator.groups[0]["epoch"], 10.02)
        self.assertIsNone(estimator.position())

    def test_only_fresh_samples_enter_a_bounded_epoch_window(self):
        estimator = StationaryEpochFusion()
        estimator.observe(-1, (99, 0, 0), 0, 0, False, False, 0)
        self.assertEqual(len(estimator.groups), 0)
        epoch(estimator, 0, count=40)
        means = estimator.means()
        self.assertEqual(means[0]["samples"], 25)
        self.assertAlmostEqual(means[0]["first_sample_time_s"], 0.02)
        self.assertAlmostEqual(means[0]["last_sample_time_s"], 0.5)

    def test_older_epochs_need_ten_samples_but_current_may_have_one(self):
        estimator = StationaryEpochFusion()
        epoch(estimator, 0, count=9)
        epoch(estimator, 5, count=25)
        epoch(estimator, 10, count=1)
        self.assertIsNone(estimator.position())
        estimator = StationaryEpochFusion()
        epoch(estimator, 0, count=10)
        epoch(estimator, 5, count=10)
        epoch(estimator, 10, count=1)
        self.assertEqual(estimator.position(), (0, 0))

    def test_retained_observations_use_previous_speed_and_applied_steering(self):
        estimator = StationaryEpochFusion()
        epoch(estimator, 0)
        epoch(estimator, 5)
        steering = math.atan(0.4)
        estimator.observe(10, (0, 0, 0), 0.01, steering, True, False, 0)
        estimator.observe(
            10.02, (0.0002, 0, 0.0002), 0.01, steering, False, False, 0.02
        )
        self.assertAlmostEqual(estimator.position()[0], 0.0002)
        self.assertAlmostEqual(estimator.position()[1], 0)
        x = 0.0002 + 0.0002 * math.cos(0.0002)
        y = 0.0002 * math.sin(0.0002)
        estimator.observe(10.04, (x, y, 0.0004), 0, steering, False, False, 0.04)
        self.assertAlmostEqual(estimator.position()[0], x)
        self.assertAlmostEqual(estimator.position()[1], y)

    def test_truth_changes_evaluation_but_never_selected_angle_or_acceptance(self):
        pose = (1, 25, math.pi)
        first, _ = select_arc({}, "test", pose, (25, 25, 0), 0.5, [], True)
        second, _ = select_arc({}, "test", pose, (0.7, 25, math.pi), 0.5, [], True)
        self.assertEqual(first["steering_deg"], second["steering_deg"])
        self.assertEqual(first["estimated_clearance"], second["estimated_clearance"])
        self.assertTrue(first["accepted"] and second["accepted"])
        self.assertTrue(first["accepted_true_pass"])
        self.assertTrue(second["mismatch"])


if __name__ == "__main__":
    unittest.main()
