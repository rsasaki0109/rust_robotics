"""Live-preview reconstruction and paired outcome accounting checks."""

import math
import unittest

from analyze_terminal_arc_live import summarize
from verify_stopping_preview import preview_components


class LiveTerminalArcTests(unittest.TestCase):
    def fixture(self, x=10, y=10, yaw=0):
        before = dict(x=x, y=y, yaw=yaw, v=0)
        row = {
            f"{kind}_{axis}": "0"
            for kind in ["noise", "bias"]
            for axis in ["x", "y", "yaw"]
        }
        row.update(
            preview_pose_x=x,
            preview_pose_y=y,
            preview_pose_yaw=yaw,
            measured_speed="0",
            nominal_target_speed="0",
            nominal_steering=str(math.radians(35)),
            recovery_preview_active="true",
        )
        return before, row

    def test_opt_in_changes_only_terminal_geometry(self):
        before, row = self.fixture()
        obstacles = [(10.6, 9.5)]
        straight = preview_components(before, row, obstacles)
        curve = preview_components(
            before, dict(row, terminal_steering_preview="true"), obstacles
        )
        explicit_false = preview_components(
            before, dict(row, terminal_steering_preview="false"), obstacles
        )
        self.assertEqual(straight, explicit_false)
        self.assertEqual(straight[0], curve[0])
        self.assertEqual(straight[2], curve[2])
        self.assertAlmostEqual(straight[1], 0.5)
        self.assertGreater(curve[1], 0.6)

    def test_reconstructed_curve_includes_interior_wall_extremum(self):
        before, row = self.fixture(10, 49.8, 3 * math.pi / 4)
        stop, terminal, steps = preview_components(
            before, dict(row, terminal_steering_preview="true"), []
        )
        expected = 0.2 - (1 + math.cos(3 * math.pi / 4)) / (
            math.tan(math.radians(35)) / 0.4
        )
        self.assertAlmostEqual(stop, 0.2)
        self.assertAlmostEqual(terminal, expected)
        self.assertEqual(steps, 11)

    def test_summary_keeps_gain_and_loss_and_common_horizon_wait(self):
        cases = []
        for variant, statuses in [
            ("epoch", ["held", "timeout"]),
            ("curved", ["timeout", "held"]),
        ]:
            for status in statuses:
                cases.append(
                    dict(
                        cohort="known-white",
                        mode="none",
                        variant=variant,
                        held=str(status == "held").lower(),
                        status=status,
                        collision="false",
                        wait_time_s=10.0,
                        stationary_wait_time_s="8",
                        short_reentries="2",
                        goal_distance="1",
                    )
                )
        pairs = [
            dict(
                cohort="known-white",
                mode="none",
                gained_hold=gain,
                lost_hold=not gain,
                both_held=False,
                first_command_change_s="5",
                matched_wait_difference_s=delta,
            )
            for gain, delta in [(True, -3), (False, 1)]
        ]
        result = [
            r
            for r in summarize(cases, pairs)
            if r["cohort"] == "all" and r["mode"] == "all"
        ]
        self.assertEqual(len(result), 2)
        for row in result:
            self.assertEqual(row["held"], 1)
            self.assertEqual(row["gained_holds"], 1)
            self.assertEqual(row["lost_holds"], 1)
            self.assertEqual(row["mean_matched_wait_delta_s"], -1)
            self.assertEqual(row["mean_both_held_time_delta_s"], "")


if __name__ == "__main__":
    unittest.main()
