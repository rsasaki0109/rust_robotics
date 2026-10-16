"""Check arm translation, timer history, local geometry and shared windows."""

from copy import deepcopy
import math
import unittest

from analyze_terminal_arc_divergence import (
    checkpoint_choices,
    decisive_classification,
    exact_geometry,
    locate_fork,
    same_inputs,
    window_summary,
)
from test_epoch_divergence import row


def release_tracks(joint=False, history=False):
    count = 50 if joint else 25
    reference = [row(i * 0.02) for i in range(count + 1)]
    for item in reference:
        item.update(nominal_target_speed="0.5", terminal_steering_preview="false")
        if joint:
            item["safety_paused"] = "true"
    candidate = deepcopy(reference)
    for i in range(1, count + 1):
        candidate[i]["terminal_steering_preview"] = "true"
        if not history or i == 5:
            reference[i].update(
                terminal_recovery_clearance="0.9",
                preview_clearance_lower_bound="0.8",
                release_preview_clearance_lower_bound="0.85",
            )
        field = "recovery_evidence" if joint else "guard_release_evidence"
        candidate[i][field] = str(i * 0.02 if i < count else 0)
        if history:
            reference[i][field] = str(i * 0.02 if i < 5 else (i - 5) * 0.02)
    candidate[-1].update(
        guard_active="false", guard_released="true", target_speed="0.5", v="0.01"
    )
    if joint:
        candidate[-1].update(resumed="true", safety_paused="false")
    return reference, candidate


class TerminalArcForkTests(unittest.TestCase):
    def test_curved_release_translates_legacy_names_and_keeps_half_second(self):
        a, b = release_tracks()
        indices, record = locate_fork(a, b, "0.5")
        self.assertEqual(record["first_actor"], "curved")
        self.assertEqual(record["permitting_arm"], "curved")
        self.assertEqual(record["kind"], "independent_release")
        self.assertEqual(record["winning_ready_run_s"], 0.5)
        self.assertEqual(indices["decisive"], 25)
        self.assertEqual(record["curved_prior_guard_evidence"], "0.48")

    def test_joint_resume_keeps_one_second_confirmation(self):
        a, b = release_tracks(joint=True)
        _, record = locate_fork(a, b, "1.0")
        self.assertEqual(record["kind"], "joint_resume")
        self.assertEqual(record["winning_ready_run_s"], 1)
        self.assertEqual(record["curved_prior_joint_evidence"], "0.98")

    def test_later_action_can_have_equal_current_geometry(self):
        a, b = release_tracks(history=True)
        indices, record = locate_fork(a, b, "0.5")
        self.assertEqual(indices["decisive"], 5)
        self.assertEqual(indices["action"], 25)
        self.assertTrue(record["history_only_release"])
        self.assertAlmostEqual(record["decisive_to_action_s"], 0.4)

    def test_straight_arm_can_release_first(self):
        a, b = release_tracks()
        _, record = locate_fork(b, a, "0.5")
        self.assertEqual(record["first_actor"], "epoch")
        self.assertEqual(record["permitting_arm"], "epoch")

    def test_entry_actor_is_the_opposite_of_permitting_arm(self):
        a = [row(0), row(0.02)]
        for item in a:
            item.update(guard_active="false", terminal_steering_preview="false")
        b = deepcopy(a)
        a[-1].update(
            guard_active="true",
            guard_triggered="true",
            terminal_recovery_clearance="0.8",
            preview_clearance_lower_bound="0.7",
            release_preview_clearance_lower_bound="0.75",
        )
        b[-1].update(target_speed="0.5", v="0.01", terminal_steering_preview="true")
        for left, right, actor, permit in [
            (a, b, "epoch", "curved"),
            (b, a, "curved", "epoch"),
        ]:
            _, record = locate_fork(left, right, ".02")
            self.assertEqual((record["kind"], record["gate"]), ("entry", "entry"))
            self.assertEqual(record["first_actor"], actor)
            self.assertEqual(record["permitting_arm"], permit)

    def test_shared_inputs_include_the_actual_preview_position(self):
        a, b = row(), row()
        b["preview_pose_x"] = "24.6"
        self.assertFalse(same_inputs(a, a, a, b))

    def test_postfork_endpoints_clip_to_the_shared_record(self):
        a, b = release_tracks()
        indices, _ = locate_fork(a, b, ".5")
        choices = checkpoint_choices([a, b], indices)
        self.assertIn("postfork_5s", choices[0][25])
        self.assertIn("postfork_30s", choices[0][25])
        self.assertIn("first_positive_release_after_fork", choices[1][25])
        self.assertNotIn("first_positive_release_after_fork", choices[0][25])


class TerminalArcGeometryTests(unittest.TestCase):
    def test_truth_substitution_and_path_change_preserve_prefix(self):
        before, current = row(), row(0.02)
        before.update(x="10.1", y="10", yaw="0")
        current.update(
            preview_pose_x="10",
            preview_pose_y="10",
            preview_pose_yaw="0",
            nominal_steering=str(math.radians(35)),
            nominal_target_speed="0",
            preview_margin="0.6",
            guard_release_margin="0.65",
            terminal_steering_preview="false",
            stop_preview_clearance_lower_bound=str(math.hypot(0.6, 0.5)),
            terminal_recovery_clearance="0.5",
            preview_clearance_lower_bound="0.4",
            release_preview_clearance_lower_bound="0.45",
        )
        cache = {}
        result = exact_geometry(before, current, [(10.6, 9.5)], cache)
        self.assertEqual(len(result), 6)
        self.assertEqual(len(cache), 2)  # True x,y and full truth share yaw here.
        logged = {r["rule"]: r for r in result if r["pose_kind"] == "logged"}
        self.assertEqual(
            logged["straight"]["stop_exact"], logged["curved"]["stop_exact"]
        )
        self.assertGreater(logged["curved"]["terminal_exact"], 0.6)
        self.assertAlmostEqual(logged["straight"]["position_error"], 0.1)
        self.assertEqual(current["terminal_steering_preview"], "false")

    def test_optimistic_permission_is_distinct_from_a_physical_contact(self):
        records = []
        for variant, rule in [("epoch", "straight"), ("curved", "curved")]:
            for pose in ["logged", "true_xy", "truth"]:
                clear = (variant == "curved") != (pose == "truth")
                records.append(
                    dict(
                        variant=variant,
                        rule=rule,
                        pose_kind=pose,
                        entry_clear=clear,
                        entry_slack=0.1 if clear else -0.1,
                        entry_ambiguous=False,
                        physical_clearance=0.6,
                        entry_margin="0.7",
                        stop_exact=2.0,
                        terminal_exact=0.9 if clear else 0.7,
                    )
                )
        result = decisive_classification(
            dict(gate="entry", permitting_arm="curved"), records
        )
        self.assertEqual(result["truth_relation"], "deny_only")
        self.assertTrue(result["optimistic_permission"])
        self.assertTrue(result["conservative_denial"])
        self.assertTrue(result["heading_changes_permit"])
        self.assertGreater(result["permit_physical_clearance"], 0.5)
        self.assertEqual(result["permit_failure_component"], "terminal")


class TerminalArcWindowTests(unittest.TestCase):
    def test_window_excludes_initial_events_and_counts_wait_union(self):
        track = [row(i * 0.02) for i in range(4)]
        for i, item in enumerate(track):
            item.update(x="35", y=str(39 + i), guard_active="false")
        track[1]["guard_triggered"] = "true"  # State before the chosen window.
        track[2].update(
            guard_active="true", safety_paused="true", guard_triggered="true"
        )
        track[3].update(target_speed="0.5", v="0.5")
        result = window_summary(track, 1, 3)
        self.assertEqual(result["guard_entries"], 1)
        self.assertEqual(result["wait_s"], 0.02)
        self.assertEqual(result["positive_command_s"], 0.02)
        self.assertEqual(result["duration_s"], 0.04)
        self.assertEqual(result["travel"], 2)
        self.assertEqual(result["goal_progress"], 2)

    def test_zero_duration_window_has_no_intervals(self):
        result = window_summary([row()], 0, 0)
        self.assertEqual(result["travel"], 0)
        self.assertEqual(result["wait_s"], 0)
        self.assertEqual(result["duration_s"], 0)


if __name__ == "__main__":
    unittest.main()
