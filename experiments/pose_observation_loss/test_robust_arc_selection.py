"""Check uncertainty bounds, exact pruning and fixed selection boundaries."""

import math
import random
import unittest

from analyze_recovery_arcs import arc_clearance
from analyze_robust_arc_selection import (
    YAW_LIMIT,
    accepts,
    candidate_scores,
    choose,
    envelope_covers,
    nearby_obstacles,
    summarize,
)


class RobustArcSelectionTests(unittest.TestCase):
    def test_pruning_preserves_full_clearance_including_walls(self):
        rng = random.Random(1701)
        obstacles = [(rng.uniform(0, 50), rng.uniform(0, 50)) for _ in range(200)]
        for pose in [(0.1, 25, 2.9), (25, 49.9, 0.4), (25, 25, -1.2)]:
            nearby = nearby_obstacles(pose, obstacles)
            self.assertLess(len(nearby), len(obstacles))
            for angle in [-35, -10, 0, 17, 35]:
                for offset in [-4, 0, 4]:
                    shifted = pose[0], pose[1], pose[2] + math.radians(offset)
                    self.assertAlmostEqual(
                        arc_clearance(shifted, math.radians(angle), nearby),
                        arc_clearance(shifted, math.radians(angle), obstacles),
                        places=10,
                    )
        self.assertEqual(nearby_obstacles((25, 25, 0), []), [])

    def test_grid_bound_covers_unsampled_headings(self):
        rng = random.Random(1702)
        obstacles = [(11.0, 10.7), (10.5, 9.2), (9.8, 10.8)]
        for pose in [(10, 10, 0.4), (0.7, 25, math.pi)]:
            scores = candidate_scores(pose, obstacles, [-35, -12, 0, 23, 35])
            for angle, score in scores.items():
                for _ in range(25):
                    shifted = (
                        pose[0],
                        pose[1],
                        pose[2] + rng.uniform(-YAW_LIMIT, YAW_LIMIT),
                    )
                    actual = arc_clearance(shifted, math.radians(angle), obstacles)
                    self.assertGreaterEqual(actual + 1e-10, score["yaw_lower"])

    def test_pose_bound_covers_simultaneous_translation_and_rotation(self):
        rng = random.Random(1703)
        pose = (10, 10, -0.2)
        obstacles = [(10.8, 10.4), (9.5, 9.6), (11, 9.7)]
        radius = 0.3
        scores = candidate_scores(pose, obstacles, [-35, 0, 35])
        for angle, score in scores.items():
            for _ in range(100):
                bearing = rng.uniform(-math.pi, math.pi)
                distance = rng.uniform(0, radius)
                truth = (
                    pose[0] + distance * math.cos(bearing),
                    pose[1] + distance * math.sin(bearing),
                    pose[2] + rng.uniform(-YAW_LIMIT, YAW_LIMIT),
                )
                self.assertTrue(envelope_covers(pose, truth, radius)[0])
                self.assertGreaterEqual(
                    arc_clearance(truth, math.radians(angle), obstacles) + 1e-10,
                    score["yaw_lower"] - radius,
                )

    def test_coverage_wraps_heading_and_checks_both_envelope_edges(self):
        pose = (10, 10, math.pi - 0.01)
        truth = (10.3, 10, -math.pi - 0.01 + YAW_LIMIT)
        covered, position, yaw = envelope_covers(pose, truth, 0.3)
        self.assertTrue(covered)
        self.assertAlmostEqual(position, 0.3)
        self.assertAlmostEqual(yaw, YAW_LIMIT)
        self.assertFalse(envelope_covers(pose, (10.30001, 10, truth[2]), 0.3)[0])
        self.assertFalse(envelope_covers(pose, (10, 10, truth[2] + 1e-5), 0.3)[0])

    def test_ties_and_strict_numerical_acceptance(self):
        self.assertEqual(choose({-20: 1, 20: 1, 5: 1 - 5e-11, -5: 1}), -5)
        self.assertEqual(choose({-20: 1, 0: 1 - 2e-10}), -20)
        self.assertFalse(accepts(0.9, 0.9))
        self.assertFalse(accepts(0.9 + 0.5e-9, 0.9))
        self.assertTrue(accepts(0.9 + 2e-9, 0.9))

    def test_summary_counts_gains_and_losses_despite_equal_total_passes(self):
        rows = []
        for reason in ["final", "fresh_stationary"]:
            for rule in ["nominal", "yaw_min", "pose_bound"]:
                for i in range(3):
                    accepted = rule != "pose_bound" and (rule == "nominal" or i != 0)
                    true_pass = i != 2 if rule == "nominal" else i != 0
                    rows.append(
                        dict(
                            cohort="test",
                            mode="none",
                            seed="1",
                            scenario="test",
                            planner="A*",
                            time_s=str(i),
                            reason=reason,
                            rule=rule,
                            accepted=accepted,
                            accepted_true_pass=accepted and true_pass,
                            mismatch=accepted and not true_pass,
                            abstained_with_true_grid_witness=not accepted,
                            envelope_covers=True,
                            steering_changed=rule != "nominal",
                        )
                    )
        result = summarize(rows)
        for row in result:
            if row["rule"] == "yaw_min":
                self.assertEqual(row["accepted_true_pass"], 2)
                self.assertEqual(row["mismatch"], 0)
                self.assertEqual(row["abstained"], 1)
                self.assertEqual(row["gained_accepted_true_pass"], 1)
                self.assertEqual(row["lost_accepted_true_pass"], 1)
            elif row["rule"] == "pose_bound":
                self.assertEqual(row["accepted"], 0)
                self.assertEqual(row["lost_accepted_true_pass"], 2)


if __name__ == "__main__":
    unittest.main()
