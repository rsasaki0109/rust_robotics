"""Fixtures for release-route boundaries and continuous evidence windows."""

import unittest

from analyze_preview_gates import episodes, gate


def row(t=0, **changes):
    return dict(
        time_s=str(t),
        guard_active="true",
        safety_paused="false",
        correction="false",
        correction_received="false",
        resumed="false",
        guard_released="false",
        v="0",
        guard_release_margin="0.55",
        stop_preview_clearance_lower_bound="0.6",
        terminal_recovery_clearance="0.7",
        effective_position_sigma="0",
        observation_age="0",
        recovery_x="25",
        recovery_y="25",
        recovery_yaw="0",
        recovery_sample_count="10",
        recovery_clearance="1",
        nominal_target_speed="0.4",
        preview_clearance_lower_bound="0.6",
        **changes,
    )


class PreviewGateTests(unittest.TestCase):
    def test_strict_component_boundaries_and_disjoint_failures(self):
        before = row()
        self.assertEqual(gate(before, row()), ("independent", "ready"))
        for stop, terminal, expected in [
            ("0.55", "0.7", "stop_only"),
            ("0.6", "0.65", "terminal_only"),
            ("0.55", "0.65", "both_blocked"),
        ]:
            current = row()
            current.update(
                stop_preview_clearance_lower_bound=stop,
                terminal_recovery_clearance=terminal,
            )
            self.assertEqual(gate(before, current), ("independent", expected))
        before["v"] = "0.0200001"
        self.assertEqual(gate(before, row()), ("independent", "other_blocked"))
        before["v"] = "0.02"
        self.assertEqual(gate(before, row()), ("independent", "ready"))

    def test_pre_command_state_and_missing_correction_select_route(self):
        before, current = row(), row()
        before["guard_active"] = "false"
        current["guard_active"] = "true"
        self.assertIsNone(gate(before, current))
        current["correction"] = "true"
        self.assertEqual(gate(before, current), ("joint", "ready"))
        current["recovery_sample_count"] = "9"
        self.assertEqual(gate(before, current), ("joint", "other_blocked"))

    def test_old_window_without_new_is_only_recorded_input_readiness(self):
        track = [row()]
        for i in range(1, 31):
            current = row(i * 0.02)
            current["terminal_recovery_clearance"] = "0.64"
            track.append(current)
        item = episodes(track)[0]
        self.assertEqual(item["terminal_only_s"], 0.6)
        self.assertTrue(item["old_window_reached"])
        self.assertFalse(item["new_window_reached"])
        self.assertEqual(item["actual_releases"], 0)

    def test_continuity_resets_on_blocker_and_route_change(self):
        track = [row()]
        for i in range(1, 25):
            track.append(row(i * 0.02))
        blocked = row(0.5)
        blocked["stop_preview_clearance_lower_bound"] = "0.54"
        track.append(blocked)
        for i in range(26, 50):
            track.append(row(i * 0.02))
        missing = row(1)
        missing["correction"] = "true"
        track.append(missing)
        items = episodes(track)
        self.assertEqual([r["route"] for r in items], ["independent", "joint"])
        self.assertEqual(items[0]["max_new_ready_s"], 0.48)
        self.assertEqual(items[1]["max_new_ready_s"], 0.02)
        self.assertFalse(any(r["new_window_reached"] for r in items))

    def test_release_interval_is_included_in_half_second_window(self):
        track = [row()] + [row(i * 0.02) for i in range(1, 26)]
        track[-1].update(guard_active="false", guard_released="true")
        track.append(row(0.52))
        item = episodes(track)[0]
        self.assertEqual(item["intervals"], 25)
        self.assertEqual(item["max_new_ready_s"], 0.5)
        self.assertEqual(item["actual_releases"], 1)

    def test_joint_release_requires_full_second(self):
        track = [row(i * 0.02) for i in range(51)]
        for current in track:
            current["safety_paused"] = "true"
        item = episodes(track[:-1])[0]
        self.assertEqual(item["max_new_ready_s"], 0.98)
        self.assertFalse(item["new_window_reached"])
        track[-1].update(safety_paused="false", guard_active="false", resumed="true")
        item = episodes(track)[0]
        self.assertEqual(item["max_new_ready_s"], 1)
        self.assertEqual(item["actual_releases"], 1)


if __name__ == "__main__":
    unittest.main()
