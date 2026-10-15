"""Analytic fixtures for the additional terminal recovery segment."""

import math
import unittest

from verify_stopping_preview import exact_preview, preview_components


def inputs(x=25, y=25, yaw=0, speed=0, target=0, steering=0, active=True):
    before = dict(x=x, y=y, yaw=yaw, v=speed)
    row = {
        f"{kind}_{axis}": 0 for kind in ["noise", "bias"] for axis in ["x", "y", "yaw"]
    }
    row.update(
        nominal_target_speed=target,
        nominal_steering=steering,
        recovery_preview_active="true" if active else "false",
    )
    return before, row


class RecoveryPreviewGeometryTests(unittest.TestCase):
    def test_segment_interior_is_closer_than_stationary_stop(self):
        before, row = inputs()
        stop, terminal, _ = preview_components(before, row, [(25.3, 25.2)])
        self.assertAlmostEqual(stop, math.hypot(0.3, 0.2))
        self.assertAlmostEqual(terminal, 0.2)
        self.assertAlmostEqual(exact_preview(before, row, [(25.3, 25.2)])[0], 0.1)

    def test_forward_segment_uses_terminal_heading(self):
        before, row = inputs(yaw=math.pi / 2)
        stop, terminal, _ = preview_components(before, row, [(25, 26.2)])
        self.assertAlmostEqual(stop, 1.2)
        self.assertAlmostEqual(terminal, 0.6)

    def test_terminal_segment_includes_map_boundary(self):
        before, row = inputs(x=49)
        self.assertAlmostEqual(exact_preview(before, row, [])[0], 0.3)

    def test_disabled_extension_retains_original_clearance(self):
        before, row = inputs(active=False)
        stop, terminal, _ = preview_components(before, row, [(26.2, 25)])
        self.assertAlmostEqual(stop, 1.2)
        self.assertEqual(terminal, 0)
        self.assertAlmostEqual(exact_preview(before, row, [(26.2, 25)])[0], 1.2)

    def test_curved_left_and_right_are_mirror_symmetric(self):
        left = inputs(speed=0.7, target=0.8, steering=0.4)
        right = inputs(speed=0.7, target=0.8, steering=-0.4)
        a = preview_components(*left, [(26, 25.6)])
        b = preview_components(*right, [(26, 24.4)])
        self.assertAlmostEqual(a[0], b[0])
        self.assertAlmostEqual(a[1], b[1])
        self.assertEqual(a[2], b[2])


if __name__ == "__main__":
    unittest.main()
