"""Hand-built speed-probe boundaries, fixed-pose geometry and truth comparisons."""

import math
import unittest

import analyze_speed_probe_starts as analysis


def change(mask="16", reason="speed_cap_zero", label="joint_lost", phase="post"):
    return dict(
        transition_id="1",
        event_id="1",
        phase=phase,
        label=label,
        end_offset_steps="28",
        pre_state_id="10",
        post_state_id="11",
        failed_mask=mask,
        command_zero_reason=reason,
    )


def record(pose_id, pose, obstacles, kind="raw"):
    r = analysis.query(pose, 0, 1, obstacles)
    r.update(
        pose_id=pose_id,
        pose_kind=kind,
        cohort="known-white",
        variant="epoch",
        mode="none",
        seed="269",
        scenario="fixture",
        planner="A*",
        source_event_kinds="received",
    )
    return r


class SpeedProbeStartTests(unittest.TestCase):
    def test_obstacle_at_start_margin_blocks_even_when_chord_moves_away(self):
        r = analysis.query((10, 10, math.pi), 0, 1, [(11, 10)])
        self.assertEqual(r["start_obstacle_gap"], 0)
        self.assertEqual(r["category"], "start_obstacle")
        self.assertFalse(r["first_chord_clear"])

    def test_wall_equality_is_blocked_in_speed_probe(self):
        r = analysis.query((1, 10, 0), 0, 1, [])
        self.assertEqual(r["start_wall_gap"], 0)
        self.assertEqual(r["category"], "start_wall")

    def test_simultaneous_start_wall_and_obstacle_have_one_category(self):
        r = analysis.query((1, 10, 0), 0, 1, [(2, 10)])
        self.assertEqual(r["category"], "start_both")

    def test_clear_start_can_hit_obstacle_in_first_chord(self):
        r = analysis.query((10, 10, 0), 0, 1, [(11.025, 10)])
        self.assertGreater(r["start_obstacle_gap"], 0)
        self.assertLess(r["first_chord_obstacle_gap"], 0)
        self.assertEqual(r["category"], "first_chord_obstacle")

    def test_first_chord_wall_and_combined_obstacle_categories(self):
        wall = analysis.query((48.975, 10, 0), 0, 1, [])
        both = analysis.query((48.975, 10, 0), 0, 1, [(50, 10)])
        self.assertEqual(wall["category"], "first_chord_wall")
        self.assertEqual(both["category"], "first_chord_both")

    def test_positive_first_chord_does_not_assert_whole_two_metre_probe_is_clear(self):
        r = analysis.query((10, 10, 0), 0, 1, [(11.5, 10)])
        self.assertEqual(r["category"], "positive_first_chord")
        analysis.check_source_free(r, 0.45)
        with self.assertRaises(AssertionError):
            analysis.check_source_free(r, 0)
        with self.assertRaises(AssertionError):
            analysis.check_source_free(r, 0.025)

    def test_initial_block_cannot_be_removed_by_heading_or_steering_alone(self):
        for yaw in [-math.pi, -0.3, 0, math.pi / 2]:
            for steering in [-math.radians(35), 0, math.radians(35)]:
                r = analysis.query((10, 10, yaw), steering, 1, [(10.8, 10)])
                self.assertEqual(r["category"], "start_obstacle")

    def test_stable_endpoint_matches_independent_simpson_integration(self):
        pose = (10, 12, 0.4)
        for steering in [-math.radians(35), -0.2, 0, 0.2, math.radians(35)]:
            k = math.tan(steering) / 0.4
            n = 32
            ds = 0.05 / n
            values = []
            for trig in [math.cos, math.sin]:
                total = sum(
                    (1 if i in [0, n] else 4 if i % 2 else 2)
                    * trig(pose[2] + k * i * ds)
                    for i in range(n + 1)
                )
                values.append(ds * total / 3)
            actual = analysis.first_endpoint(pose, steering)
            self.assertLess(
                math.dist(actual, (pose[0] + values[0], pose[1] + values[1])), 1e-11
            )

    def test_nearly_straight_endpoint_keeps_small_curvature_stable(self):
        for steering in [0, 1e-12, 4.0000001e-10]:
            actual = analysis.first_endpoint((10, 10, 0), steering)
            self.assertAlmostEqual(actual[0], 10.05, places=11)
            self.assertAlmostEqual(actual[1], 10, places=10)

    def test_start_and_chord_can_have_different_nearest_obstacles(self):
        r = analysis.query((10, 10, 0), 0, 0.1, [(9.99, 10), (10.025, 10.002)])
        self.assertEqual(r["start_obstacle_id"], 1)
        self.assertEqual(r["first_chord_obstacle_id"], 2)
        self.assertAlmostEqual(r["first_chord_obstacle_gap"], -0.098)
        tie = analysis.query((10, 10, 0), 0, 0.1, [(11, 11), (11, 9)])
        self.assertEqual(tie["start_obstacle_id"], 1)
        self.assertEqual(tie["first_chord_obstacle_id"], 1)

    def test_boundary_marker_does_not_relax_the_start_rule(self):
        r = analysis.query((10, 10, 0), 0, 1, [(11 + 1e-9, 10)])
        self.assertTrue(r["near_boundary"])
        self.assertGreater(r["start_obstacle_gap"], 0)
        self.assertEqual(r["category"], "first_chord_obstacle")

    def test_all_four_truth_relations(self):
        actual = [
            analysis.truth_relation(a, b)
            for a, b in [(True, True), (True, False), (False, True), (False, False)]
        ]
        self.assertEqual(actual, analysis.RELATIONS)

    def test_estimate_clearing_chord_can_still_disagree_with_truth_at_same_margin(self):
        obstacles = [(10.5, 10)]
        raw = record(1, (10, 10, 0), obstacles)
        estimate = record(2, (8, 10, 0), obstacles, "preview")
        truth = record(3, (10, 10, 0), obstacles, "truth")
        r = analysis.pair_record(
            change(), "preview", estimate, estimate, truth, truth, raw
        )
        self.assertEqual(r["change"], "held_clear")
        self.assertTrue(r["new_clear_from_raw"])
        self.assertEqual(r["post_truth_relation"], "estimate_clear_truth_blocks")
        self.assertTrue(r["raw_zero_loss"] and r["command_only_zero_loss"])

    def test_fixed_source_subset_keeps_multiple_failures_and_excludes_upstream_zero(
        self,
    ):
        self.assertEqual(analysis.source_subsets(change()), (True, True))
        self.assertEqual(analysis.source_subsets(change(mask="24")), (True, False))
        self.assertEqual(
            analysis.source_subsets(change(reason="uncapped_zero")), (False, False)
        )
        self.assertEqual(
            analysis.source_subsets(change(label="joint_retained")), (False, False)
        )
        self.assertEqual(
            analysis.source_subsets(change(phase="immediate")), (False, False)
        )

    def test_summary_keeps_input_views_separate_and_empty_categories(self):
        obstacles = [(10.5, 10)]
        raw = record(1, (10, 10, 0), obstacles)
        recovery = record(2, (8, 10, 0), obstacles, "recovery")
        preview = record(3, (8, 10, 0), obstacles, "preview")
        truth = record(4, (10, 10, 0), obstacles, "truth")
        poses = [raw, recovery, preview, truth]
        for r in poses:
            r["truth_relation"] = analysis.truth_relation(
                r["first_chord_clear"], truth["first_chord_clear"]
            )
        pairs = [
            analysis.pair_record(change(), kind, r, r, truth, truth, raw)
            for kind, r in zip(analysis.ESTIMATES, poses)
        ]
        e = dict(
            event_id="1", **{k: raw[k] for k in analysis.KEY}, event_kind="received"
        )
        ss, ps = analysis.summarize(poses, pairs, [e])
        chosen = [
            r
            for r in ps
            if (
                r["cohort"],
                r["variant"],
                r["event_kind"],
                r["phase"],
                r["source_label"],
                r["subset"],
            )
            == ("all", "epoch", "received", "post", "joint_lost", "raw_zero_loss")
        ]
        self.assertEqual(len(chosen), 3)
        self.assertTrue(
            all(r["pairs"] == r["opportunities"] == r["cases"] == 1 for r in chosen)
        )
        self.assertEqual(
            next(r for r in chosen if r["pose_kind"] == "preview")[
                "new_clear_truth_blocks"
            ],
            1,
        )
        self.assertTrue(
            all(r["pairs"] == 0 for r in ps if r["event_kind"] == "precision_only")
        )
        self.assertTrue(
            all(
                r["states"] == 1
                for r in ss
                if (r["cohort"], r["variant"], r["event_kind"])
                == ("all", "epoch", "received")
            )
        )


if __name__ == "__main__":
    unittest.main()
