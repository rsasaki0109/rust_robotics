"""Release-metric, interval and true-pose snapshot fixtures for the diagnosis."""

import copy
import unittest

from analyze_aligned_stalls import (
    checkpoint,
    gate_state,
    release_requirements,
    tail_gates,
)


def row():
    return dict(
        guard_active="true",
        safety_paused="false",
        correction="false",
        correction_received="false",
        v="0",
        effective_position_sigma="0",
        observation_age="0",
        recovery_x="25",
        recovery_y="25",
        recovery_yaw="0",
        recovery_sample_count="10",
        recovery_clearance="1",
        nominal_target_speed="0.4",
        preview_clearance_lower_bound="0.52",
        preview_margin="0.5",
        guard_release_margin="0.55",
        stop_preview_clearance_lower_bound="0.8",
        terminal_recovery_clearance="0.62",
    )


class AlignedStallTests(unittest.TestCase):
    def test_aligned_release_uses_terminal_entry_threshold_and_not_entry_metric(self):
        before, current = row(), row()
        before["v"] = "0.02"
        route, ready = release_requirements(before, current)
        self.assertEqual(route, "independent")
        self.assertEqual(gate_state(ready), "ready")
        self.assertLess(float(current["preview_clearance_lower_bound"]), 0.55)
        self.assertLess(float(current["terminal_recovery_clearance"]), 0.65)

    def test_strict_boundaries_and_partition_priority(self):
        before, current = row(), row()
        current["terminal_recovery_clearance"] = str(0.55 + 0.05)
        self.assertEqual(
            gate_state(release_requirements(before, current)[1]), "terminal_only"
        )
        current["stop_preview_clearance_lower_bound"] = "0.55"
        self.assertEqual(
            gate_state(release_requirements(before, current)[1]), "both_blocked"
        )
        current["terminal_recovery_clearance"] = "0.7"
        self.assertEqual(
            gate_state(release_requirements(before, current)[1]), "stop_only"
        )
        before["v"] = "0.020001"
        self.assertEqual(
            gate_state(release_requirements(before, current)[1]), "other_blocked"
        )

    def test_missing_correction_overrides_independent_route_and_checks_samples(self):
        before, current = row(), row()
        before["guard_active"] = "false"
        self.assertEqual(release_requirements(before, current), (None, {}))
        current.update(correction="true", recovery_sample_count="9")
        route, ready = release_requirements(before, current)
        self.assertEqual(route, "joint")
        self.assertFalse(ready["samples"])
        self.assertEqual(gate_state(ready), "other_blocked")

    def test_runs_reset_on_blockers_and_route_changes(self):
        rows = [row() for _ in range(52)]
        rows[25]["terminal_recovery_clearance"] = "0.59"
        rows[50].update(correction="true", safety_paused="true")
        counts, maxima = tail_gates(list(zip(rows, rows[1:])))
        self.assertEqual(maxima["independent"], dict(actual=24, without_terminal=49))
        self.assertEqual(maxima["joint"], dict(actual=2, without_terminal=2))
        self.assertEqual(counts["independent"]["terminal_only"], 1)

    def test_noneligible_interval_breaks_consecutive_evidence(self):
        inactive = row()
        inactive["guard_active"] = "false"
        pairs = [(row(), row())] * 24 + [(inactive, row())] + [(row(), row())] * 24
        _, maxima = tail_gates(pairs)
        self.assertEqual(maxima["independent"], dict(actual=24, without_terminal=24))

    def test_truth_snapshot_keeps_command_and_margin_and_does_not_mutate_inputs(self):
        before = dict(row(), x="25", y="25", yaw="0")
        current = dict(
            row(),
            time_s="180",
            preview_pose_x="25.1",
            preview_pose_y="25",
            preview_pose_yaw="0",
            nominal_target_speed="0",
            nominal_steering="0",
            recovery_preview_active="true",
            aligned_terminal_release="true",
            stop_preview_clearance_lower_bound="1.15",
            terminal_recovery_clearance="0.55",
            preview_clearance_lower_bound="0.45",
            release_preview_clearance_lower_bound="0.5",
            recovery_clearance="0.65",
        )
        current.update(
            {
                f"{kind}_{axis}": "0"
                for kind in ["noise", "bias"]
                for axis in ["x", "y", "yaw"]
            }
        )
        original = copy.deepcopy((before, current))
        case = dict(
            cohort="test", mode="none", seed="265", scenario="open_50x50", planner="A*"
        )
        result = checkpoint(case, before, current, [(26.25, 25)], ["fixture"])
        self.assertLess(result["release_exact"], result["release_margin"])
        self.assertTrue(result["true_pose_release_passes"])
        self.assertAlmostEqual(result["true_pose_release_exact"], 0.6)
        self.assertEqual((before, current), original)


if __name__ == "__main__":
    unittest.main()
