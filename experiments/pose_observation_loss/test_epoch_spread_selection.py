"""Check conditional spread bounds and limits of agreement between epoch means."""

import math
import random
import unittest

from analyze_epoch_spread_selection import score_angles
from analyze_recovery_arcs import arc_clearance
from analyze_robust_arc_selection import accepts, envelope_covers


class EpochSpreadSelectionTests(unittest.TestCase):
    def test_clear_vertices_do_not_certify_their_convex_hull(self):
        positions = [(9, 10), (11, 10), (10, 12)]
        center = (10, 10 + 2 / 3, 0)
        obstacles = [center[:2]]
        _, scores = score_angles(center, positions, True, obstacles, [0])
        self.assertTrue(accepts(scores[0]["epoch_min"], 0.5))
        self.assertEqual(arc_clearance(center, 0, obstacles), 0)
        self.assertFalse(accepts(scores[0]["spread_yaw_bound"], 0.5))

    def test_bound_covers_simultaneous_position_and_heading_errors(self):
        rng = random.Random(1901)
        obstacles = [(10.7, 10.4), (0.6, 25.5), (49, 49)]
        for center in [(1, 25, math.pi - 0.01), (10, 10, 0.7)]:
            positions = [
                (center[0] - 0.2, center[1]),
                (center[0] + 0.1, center[1] - 0.1),
                (center[0] + 0.1, center[1] + 0.1),
            ]
            radius, scores = score_angles(
                center, positions, True, obstacles, [-35, 0, 35]
            )
            for angle, values in scores.items():
                for _ in range(50):
                    bearing, distance = rng.uniform(-math.pi, math.pi), rng.uniform(
                        0, radius
                    )
                    truth = (
                        center[0] + distance * math.cos(bearing),
                        center[1] + distance * math.sin(bearing),
                        center[2] + math.radians(rng.uniform(-4, 4)),
                    )
                    self.assertTrue(envelope_covers(center, truth, radius)[0])
                    self.assertGreaterEqual(
                        arc_clearance(truth, math.radians(angle), obstacles) + 1e-10,
                        values["spread_yaw_bound"],
                    )

    def test_coincident_means_can_share_an_uncovered_position_bias(self):
        center = (1.2, 25, math.pi)
        radius, scores = score_angles(center, [center[:2]] * 3, True, [], [0])
        truth = (0.7, 25, math.pi)
        self.assertEqual(radius, 0)
        self.assertTrue(accepts(scores[0]["spread_yaw_bound"], 0.5))
        self.assertFalse(accepts(arc_clearance(truth, 0, []), 0.5))
        self.assertFalse(envelope_covers(center, truth, radius)[0])

    def test_unavailable_rules_reproduce_the_same_central_scores(self):
        center, obstacles = (2, 2, 0.3), [(3, 2.5), (1, 1)]
        radius, scores = score_angles(center, [], False, obstacles, [-35, 0, 35])
        self.assertEqual(radius, 0)
        for angle, values in scores.items():
            expected = arc_clearance(center, math.radians(angle), obstacles)
            for name in ["fused_center", "epoch_min", "spread_yaw_bound"]:
                self.assertAlmostEqual(values[name], expected)

    def test_epoch_min_matches_full_scans_at_distinct_positions(self):
        center = (10.2, 10.1, 0.7)
        positions = [(10, 10), (10.4, 10), (10.2, 10.3)]
        obstacles = [(10.7, 10.5), (10.2, 9.2), (49, 49), (0, 0)]
        _, scores = score_angles(
            center, positions, True, obstacles, [-35, -5, 0, 10, 35]
        )
        for angle, values in scores.items():
            full = [
                arc_clearance((*p, center[2]), math.radians(angle), obstacles)
                for p in positions
            ]
            self.assertAlmostEqual(values["epoch_min"], min(full))
            self.assertAlmostEqual(full[values["worst_epoch_index"]], min(full))

    def test_radius_contains_all_convex_combinations_of_epoch_means(self):
        rng = random.Random(1902)
        positions = [(10, 10), (10.5, 10.1), (9.8, 10.8)]
        center = tuple(sum(p[i] for p in positions) / 3 for i in range(2))
        radius, _ = score_angles((*center, 0), positions, True, [], [0])
        for _ in range(100):
            weights = [rng.random() for _ in positions]
            total = sum(weights)
            point = tuple(
                sum(w * p[i] for w, p in zip(weights, positions)) / total
                for i in range(2)
            )
            self.assertLessEqual(math.dist(point, center), radius + 1e-12)


if __name__ == "__main__":
    unittest.main()
