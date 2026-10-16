"""Bidirectional recovery predicates, hybrid audits and paired path attribution."""

import math
import unittest

from analyze_commanded_arc_components import (
    component_record,
    correction_category,
    paired_effects,
    summarize,
)
from analyze_commanded_recovery_arc import META, classify


def wall_fixture():
    previous = dict.fromkeys(META, "0")
    previous.update(
        cohort="known-white",
        variant="epoch",
        mode="random",
        seed="270",
        scenario="fixture",
        planner="A*",
        time_s="150.500000",
        reasons="first_usable_epoch+age_0p5",
        rule="straight",
        input_kind="actual",
        margin="1",
        truth_center_clearance=".9",
        estimate_clear="True",
        truth_clear="False",
        optimistic="True",
        conservative="False",
        estimate_slack=".1",
        truth_slack="-.7",
        truth_physical_clearance=".3",
        accepted_physical_violation="True",
    )
    paired = dict(estimate_gain="False", estimated_gain_truth_blocked="False")
    return previous, paired, (1.1, 25, 0), (0.9, 25, math.pi), [(40, 40)]


class CommandedArcComponentTests(unittest.TestCase):
    def test_partition_works_for_both_permission_and_blocking_errors(self):
        for target in [False, True]:
            for position, heading, category in [
                (True, False, "position_only"),
                (False, True, "heading_only"),
                (True, True, "either_alone"),
                (False, False, "joint_required"),
            ]:
                passed = dict(
                    FF=not target,
                    TT=target,
                    TF=target if position else not target,
                    FT=target if heading else not target,
                )
                self.assertEqual(correction_category(passed), category)

    def test_matching_endpoints_are_not_classified_as_mismatches(self):
        for value in [False, True]:
            self.assertEqual(
                correction_category(
                    dict(FF=value, TT=value, TF=not value, FT=not value)
                ),
                "same_decision",
            )

    def test_wall_equality_and_obstacle_equality_use_different_predicates(self):
        wall = classify(2, 1, 1)["clear"]
        obstacle = classify(1, 2, 1)["clear"]
        self.assertEqual(
            correction_category(dict(FF=wall, TF=obstacle, FT=wall, TT=obstacle)),
            "position_only",
        )

    def test_hybrid_wall_distances_and_signed_contributions_have_known_values(self):
        row = component_record(*wall_fixture(), {})
        for name, expected in dict(FF=1.1, TF=0.9, FT=0.5, TT=0.3).items():
            self.assertAlmostEqual(row[name + "_distance"], expected)
        self.assertAlmostEqual(row["position_loss"], 0.2)
        self.assertAlmostEqual(row["heading_loss"], 0.6)
        self.assertEqual(row["category"], "either_alone")
        self.assertAlmostEqual(row["yaw_error_deg"], 180)
        self.assertTrue(row["FF_clear"])

    def test_cache_reuses_geometry_but_rechecks_a_changed_margin(self):
        previous, paired, estimate, truth, obstacles = wall_fixture()
        cache = {}
        a = component_record(previous, paired, estimate, truth, obstacles, cache)
        updated = dict(
            previous,
            margin="1.2",
            estimate_clear="False",
            optimistic="False",
            estimate_slack="-.1",
            truth_slack="-.9",
            accepted_physical_violation="False",
        )
        b = component_record(updated, paired, estimate, truth, obstacles, cache)
        self.assertEqual(len(cache), 4)
        self.assertTrue(a["FF_clear"])
        self.assertFalse(b["FF_clear"])
        self.assertEqual(b["category"], "same_decision")
        self.assertEqual(a["FF_audit_id"], b["FF_audit_id"])

    def test_a_changed_prior_decision_is_rejected(self):
        previous, paired, estimate, truth, obstacles = wall_fixture()
        previous["estimate_clear"] = "False"
        with self.assertRaisesRegex(AssertionError, "estimate_clear"):
            component_record(previous, paired, estimate, truth, obstacles, {})

    def test_path_error_change_preserves_signed_coordinate_interactions(self):
        previous, *_ = wall_fixture()
        common = {k: previous[k] for k in META + ["input_kind"]}
        old = dict(
            common,
            rule="straight",
            FF_distance=1.1,
            TT_distance=0.9,
            distance_error=0.2,
            position_loss=0.3,
            heading_loss=-0.1,
            category="position_only",
        )
        new = dict(
            common,
            rule="commanded",
            FF_distance=1.3,
            TT_distance=1.05,
            distance_error=0.25,
            position_loss=0.2,
            heading_loss=0.05,
            category="joint_required",
        )
        result = paired_effects([old, new], {})
        self.assertAlmostEqual(result["path_error_delta"], 0.05)
        self.assertAlmostEqual(result["position_loss_delta"], -0.1)
        self.assertAlmostEqual(result["heading_loss_delta"], 0.15)
        self.assertAlmostEqual(result["estimated_path_delta"], 0.2)
        self.assertAlmostEqual(result["truth_path_delta"], 0.15)

    def test_zero_recorded_steering_has_no_path_effect(self):
        previous, paired, estimate, truth, obstacles = wall_fixture()
        cache = {}
        old = component_record(previous, paired, estimate, truth, obstacles, cache)
        new = component_record(
            dict(previous, rule="commanded"), paired, estimate, truth, obstacles, cache
        )
        result = paired_effects([old, new], {})
        for field in [
            "path_error_delta",
            "position_loss_delta",
            "heading_loss_delta",
            "estimated_path_delta",
            "truth_path_delta",
        ]:
            self.assertEqual(result[field], 0)

    def test_empty_and_fixed_gain_groups_preserve_denominators_and_matching_rows(self):
        row = component_record(*wall_fixture(), {})
        match = dict(
            row,
            optimistic=False,
            estimate_clear=False,
            category="same_decision",
            optimistic_arc_gain=True,
            time_s="155.500000",
        )
        summaries = summarize([row, match])
        chosen = {
            r["group"]: r
            for r in summaries
            if r["cohort"] == "all" and r["subset"] == "age_0p5"
        }
        self.assertEqual(chosen["all"]["checkpoints"], 2)
        self.assertEqual(chosen["all"]["cases"], 1)
        self.assertEqual(chosen["optimistic"]["checkpoints"], 1)
        self.assertEqual(chosen["optimistic"]["either_alone"], 1)
        self.assertEqual(chosen["optimistic_arc_gain"]["same_decision"], 1)
        self.assertEqual(chosen["conservative"]["checkpoints"], 0)
        self.assertEqual(chosen["conservative"]["mean_position_loss"], "")


if __name__ == "__main__":
    unittest.main()
