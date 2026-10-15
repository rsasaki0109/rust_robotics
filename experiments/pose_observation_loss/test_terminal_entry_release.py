"""Admission remains combined while both release timers use the stop footprint."""

import copy
import unittest

from summarize_stopping_preview import release_entry_gaps

from verify_joint_recovery import (
    check_guard_track,
    check_joint_track,
    release_clearance,
)


def row():
    return dict(
        v="0",
        safety_paused="false",
        correction="false",
        correction_received="false",
        preview_clearance_lower_bound="0.4",
        preview_margin="0.5",
        guard_release_margin="0.55",
        stop_preview_clearance_lower_bound="1",
        release_preview_clearance_lower_bound="1",
        guard_active="true",
        guard_triggered="false",
        guard_released="false",
        guard_release_evidence="0",
        recovery_evidence="0",
        resumed="false",
        effective_position_sigma="0",
        observation_age="0",
        recovery_x="25",
        recovery_y="25",
        recovery_yaw="0",
        recovery_sample_count="10",
        recovery_clearance="1",
        nominal_target_speed="0.4",
        uncapped_target_speed="0.4",
        sensed_goal_distance="10",
        stop_latched="false",
        restart_limited="false",
        target_speed="0.4",
        nominal_steering="0",
        steering="0",
    )


def initial():
    start = row()
    start["guard_active"] = "false"
    return start


class TerminalEntryReleaseTests(unittest.TestCase):
    @staticmethod
    def audit(time, release=False, entry=False, lower="0.4", planner="A*"):
        return dict(
            mode="none",
            seed="262",
            scenario="open_50x50",
            planner=planner,
            time_s=str(time),
            guard_released=str(release).lower(),
            guard_triggered=str(entry).lower(),
            lower_bound=lower,
            margin="0.5",
        )

    def test_gap_counts_distinguish_next_step_from_later_and_unfinished_entries(self):
        rows = [
            self.audit(1, release=True),
            self.audit(1.01),
            self.audit(1.02, entry=True),
            self.audit(3, release=True, lower="0.6"),
            self.audit(3.04, entry=True),
            self.audit(5, release=True, lower="0.5"),
        ]
        result = release_entry_gaps(rows)[0]
        self.assertEqual(result["guard_releases"], 3)
        self.assertEqual(result["entry_blocked_at_release"], 2)
        self.assertEqual(result["immediate_reentries"], 1)
        self.assertEqual(result["blocked_release_then_immediate_entry"], 1)

    def test_gap_pairing_never_crosses_case_boundary(self):
        rows = [
            self.audit(1, release=True),
            self.audit(1.02, entry=True, planner="JPS"),
        ]
        self.assertEqual(
            sum(r["immediate_reentries"] for r in release_entry_gaps(rows)), 0
        )

    def test_legacy_clearance_falls_back_to_combined_metric(self):
        current = row()
        del current["release_preview_clearance_lower_bound"]
        self.assertEqual(release_clearance(current), 0.4)

    def test_independent_release_can_be_followed_by_immediate_combined_entry(self):
        track = [initial()]
        for i in range(1, 28):
            current = row()
            current["guard_triggered"] = "true" if i in [1, 27] else "false"
            if i == 26:
                current.update(guard_active="false", guard_released="true")
            elif 1 < i < 26:
                current["guard_release_evidence"] = str((i - 1) * 0.02)
            track.append(current)
        check_guard_track(track)
        corrupted = copy.deepcopy(track)
        corrupted[10]["release_preview_clearance_lower_bound"] = "0.4"
        with self.assertRaises(AssertionError):
            check_guard_track(corrupted)

    def test_combined_release_stays_blocked_on_the_same_inputs(self):
        track = [initial()] + [row() for _ in range(30)]
        track[1]["guard_triggered"] = "true"
        for current in track[1:]:
            del current["release_preview_clearance_lower_bound"]
        check_guard_track(track)

    def test_joint_release_uses_stop_only_for_the_full_second(self):
        track = [initial()]
        for i in range(1, 51):
            current = row()
            current.update(safety_paused="true", recovery_evidence=str(i * 0.02))
            if i == 1:
                current.update(correction="true", guard_triggered="true")
            if i == 50:
                current.update(
                    resumed="true",
                    safety_paused="false",
                    guard_active="false",
                    guard_released="true",
                    recovery_evidence="0",
                )
            track.append(current)
        check_joint_track(track)
        check_guard_track(track)
        corrupted = copy.deepcopy(track)
        corrupted[20]["release_preview_clearance_lower_bound"] = "0.4"
        with self.assertRaises(AssertionError):
            check_joint_track(corrupted)


if __name__ == "__main__":
    unittest.main()
