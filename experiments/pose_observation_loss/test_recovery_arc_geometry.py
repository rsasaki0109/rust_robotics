"""Check circular-arc geometry against closed forms and an independent polyline."""

import math
import random
import unittest

from analyze_recovery_arcs import (
    ANGLES,
    GRID_ERROR_BOUND,
    arc_clearance,
    arc_point,
    chord_check,
)


class ArcGeometryTests(unittest.TestCase):
    def test_straight_projection_and_endpoint(self):
        self.assertAlmostEqual(arc_clearance((25, 25, 0), 0, [(25.3, 26)]), 1)
        self.assertAlmostEqual(arc_clearance((25, 25, 0), 0, [(26.1, 25)]), 0.5)

    def test_radial_projection_and_circle_center(self):
        steering = math.atan(0.4)  # radius 1
        for radius in [0.5, 1, 2]:
            obstacle = (25 + radius * math.sin(0.3), 26 - radius * math.cos(0.3))
            self.assertAlmostEqual(
                arc_clearance((25, 25, 0), steering, [obstacle]), abs(radius - 1)
            )
        self.assertAlmostEqual(arc_clearance((25, 25, 0), steering, [(25, 26)]), 1)

    def test_projection_outside_arc_uses_endpoint(self):
        obstacle = (25 + math.sin(0.9), 26 - math.cos(0.9))
        self.assertAlmostEqual(
            arc_clearance((25, 25, 0), math.atan(0.4), [obstacle]), 2 * math.sin(0.15)
        )

    def test_boundary_extremum_inside_arc(self):
        pose = (49.5, 25, math.pi / 2 - 0.3)
        self.assertAlmostEqual(
            arc_clearance(pose, math.atan(0.4), []), math.cos(0.3) - 0.5
        )
        # Neither endpoint reaches the wall's minimum clearance.
        self.assertAlmostEqual(arc_point(pose, math.atan(0.4), 0.6)[0], 49.5)

    def test_clockwise_counterclockwise_and_angle_wrap(self):
        for yaw in [-math.pi + 0.01, math.pi - 0.01, -1.5, 1.5]:
            for steering in [-35, -1, 0, 1, 35]:
                pose = (25, 25, yaw)
                angle = math.radians(steering)
                obstacles = [(24.8, 25.4), (25.2, 24.8), (26, 26)]
                exact = arc_clearance(pose, angle, obstacles)
                chord_check(pose, angle, obstacles, exact)

    def test_random_geometry_against_independent_chords(self):
        rng = random.Random(20261015)
        for _ in range(40):
            pose = (
                rng.uniform(0.7, 49.3),
                rng.uniform(0.7, 49.3),
                rng.uniform(-math.pi, math.pi),
            )
            obstacles = [
                (pose[0] + rng.uniform(-1, 1), pose[1] + rng.uniform(-1, 1))
                for _ in range(8)
            ]
            steering = rng.uniform(-math.radians(35), math.radians(35))
            chord_check(
                pose, steering, obstacles, arc_clearance(pose, steering, obstacles)
            )

    def test_near_straight_arcs_remain_numerically_stable(self):
        pose, obstacles = (25, 25, 0.3), [(25.3, 25.8), (25.5, 25.1)]
        straight = arc_clearance(pose, 0, obstacles)
        for angle in [-1e-11, -1e-9, 1e-11, 1e-9]:
            exact = arc_clearance(pose, angle, obstacles)
            self.assertAlmostEqual(exact, straight, places=8)
            chord_check(pose, angle, obstacles, exact)

    def test_continuous_steering_grid_bound(self):
        rng = random.Random(314159)
        pose, obstacles = (25, 25, 0.8), [(25.4, 25.6), (25.8, 24.7)]
        grid = {a: arc_clearance(pose, math.radians(a), obstacles) for a in ANGLES}
        for _ in range(100):
            angle = rng.uniform(-35, 35)
            exact = arc_clearance(pose, math.radians(angle), obstacles)
            self.assertLessEqual(
                abs(exact - grid[round(angle)]), GRID_ERROR_BOUND + 1e-9
            )
            self.assertLessEqual(exact, max(grid.values()) + GRID_ERROR_BOUND + 1e-9)


if __name__ == "__main__":
    unittest.main()
