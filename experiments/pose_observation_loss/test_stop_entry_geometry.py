"""Independent fixtures for the stop-entry index and brake geometry."""

import math
import random
import unittest

from analyze_recovery_stalls import center_clearance
from analyze_stop_entry import CenterIndex, brake_rollout, swept_clearance


class StopEntryGeometryTests(unittest.TestCase):
    def test_index_matches_full_scan_inside_and_outside_map(self):
        rng = random.Random(20261016)
        obstacles = [(rng.uniform(0, 50), rng.uniform(0, 50)) for _ in range(100)]
        index = CenterIndex(obstacles)
        for _ in range(200):
            point = rng.uniform(-2, 52), rng.uniform(-2, 52)
            self.assertAlmostEqual(
                index.clearance(*point), center_clearance(*point, obstacles), places=12
            )

    def test_empty_map_uses_boundary(self):
        index = CenterIndex([])
        for point, expected in [
            ((25, 25), 25),
            ((0, 25), 0),
            ((49.9, 10), 0.1),
            ((-1, 25), -1),
        ]:
            self.assertAlmostEqual(index.clearance(*point), expected)

    def test_cell_corner_search_does_not_stop_at_first_obstacle(self):
        obstacles = [(0, 0), (2.0001, 0.999), (2.0001, 0.999)]
        # Translate away from the map boundary so the obstacle determines min.
        obstacles = [(x + 20, y + 20) for x, y in obstacles]
        self.assertAlmostEqual(CenterIndex(obstacles).clearance(20.999, 20.999), 1.0011)

    def test_linear_brake_tail_has_known_total_distance(self):
        states = brake_rollout((25, 25, 0), 0.1, 0)
        actual = states[-1][0] - 25
        # In this speed range, a=-4v throughout: sum(v*dt)=(v0-v_end)/4.
        self.assertAlmostEqual(actual, (0.1 - states[-1][3]) / 4, places=12)
        self.assertLessEqual(states[-1][3], 0.001)
        self.assertTrue(all(s[1] == 25 and s[2] == 0 for s in states))

    def test_stationary_preview_contains_initial_point(self):
        states = brake_rollout((25, 25, 0), 0, 0.4)
        clearance, length = swept_clearance(states, [(25.5, 25)])
        self.assertEqual(clearance, 0.5)
        self.assertEqual(length, 0)

    def test_swept_distance_checks_segment_interior(self):
        clearance, length = swept_clearance(
            [(25, 25, 0, 1), (26, 25, 0, 0)], [(25.5, 25.2)]
        )
        self.assertAlmostEqual(clearance, 0.2)
        self.assertEqual(length, 1)

    def test_brake_left_and_right_are_mirror_symmetric(self):
        left = brake_rollout((25, 25, 0), 0.8, math.radians(30))
        right = brake_rollout((25, 25, 0), 0.8, math.radians(-30))
        self.assertEqual(len(left), len(right))
        for a, b in zip(left, right):
            self.assertAlmostEqual(a[0], b[0])
            self.assertAlmostEqual(a[1] - 25, 25 - b[1])
            self.assertAlmostEqual(a[2], -b[2])
            self.assertEqual(a[3], b[3])


if __name__ == "__main__":
    unittest.main()
