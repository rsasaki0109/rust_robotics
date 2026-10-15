"""Check fixed-pose substitutions and signed, order-symmetric attribution."""

import math
import unittest

from analyze_arc_pose_components import (
    CATEGORIES,
    hybrid_poses,
    mismatch_category,
    signed_losses,
    summarize,
)
from analyze_recovery_arcs import arc_clearance


class ArcPoseComponentsTests(unittest.TestCase):
    def test_substitution_changes_only_the_named_coordinates(self):
        poses = hybrid_poses((1, 2, 0.3), (4, 5, -0.6))
        self.assertEqual(
            poses,
            dict(FF=(1, 2, 0.3), TF=(4, 5, 0.3), FT=(1, 2, -0.6), TT=(4, 5, -0.6)),
        )

    def test_zero_component_and_interchange_symmetry(self):
        self.assertEqual(signed_losses(dict(FF=2, TF=2, FT=1, TT=1)), (0, 1))
        values = dict(FF=2.0, TF=1.7, FT=1.9, TT=1.2)
        position, heading = signed_losses(values)
        swapped = dict(values, TF=values["FT"], FT=values["TF"])
        self.assertEqual(signed_losses(swapped), (heading, position))
        self.assertAlmostEqual(position, 0.5)
        self.assertAlmostEqual(heading, 0.3)

    def test_negative_contribution_is_retained_when_errors_cancel(self):
        position, heading = signed_losses(dict(FF=1.2, TF=0.7, FT=1.3, TT=0.9))
        self.assertAlmostEqual(position, 0.45)
        self.assertAlmostEqual(heading, -0.15)
        self.assertAlmostEqual(position + heading, 0.3)

    def test_threshold_partition_covers_all_four_interactions(self):
        expected = {
            (0.9, 1.1): "position_alone",
            (1.1, 0.9): "heading_alone",
            (0.9, 0.9): "either_alone",
            (1.1, 1.1): "joint_required",
        }
        for (tf, ft), category in expected.items():
            self.assertEqual(
                mismatch_category(dict(FF=1.1, TF=tf, FT=ft, TT=0.9), 1), category
            )
        self.assertEqual(
            mismatch_category(dict(FF=1.1, TF=1 + 0.5e-9, FT=1.1, TT=1), 1),
            "position_alone",
        )
        with self.assertRaises(AssertionError):
            mismatch_category(dict(FF=1.1, TF=0.9, FT=1.1, TT=1.1), 1)

    def test_wall_geometry_matches_independent_distance_decomposition(self):
        poses = hybrid_poses((1.1, 25, 0), (0.9, 25, math.pi))
        values = {k: arc_clearance(p, 0, []) for k, p in poses.items()}
        for key, expected in dict(FF=1.1, TF=0.9, FT=0.5, TT=0.3).items():
            self.assertAlmostEqual(values[key], expected)
        position, heading = signed_losses(values)
        self.assertAlmostEqual(position, 0.2)
        self.assertAlmostEqual(heading, 0.6)

    def test_summary_uses_only_accepted_mismatches_and_handles_empty_groups(self):
        base = dict(
            reason="final",
            rule="nominal",
            accepted=True,
            accepted_true_pass=False,
            mismatch=True,
            category="position_alone",
            true_start_blocks=True,
            position_loss=0.2,
            heading_loss=-0.1,
            total_loss=0.1,
        )
        rows = [
            base,
            dict(
                base,
                accepted_true_pass=True,
                mismatch=False,
                category="not_mismatch",
                total_loss=100,
            ),
            dict(
                base,
                accepted=False,
                mismatch=False,
                category="not_mismatch",
                total_loss=100,
            ),
        ]
        summaries = summarize(rows)
        actual = summaries[0]
        self.assertEqual(actual["checkpoints"], 3)
        self.assertEqual(actual["accepted"], 2)
        self.assertEqual(actual["accepted_true_pass"], 1)
        self.assertEqual(sum(actual[k] for k in CATEGORIES), 1)
        self.assertEqual(actual["true_start_blocks_mismatches"], 1)
        self.assertEqual(actual["negative_heading_loss"], 1)
        self.assertEqual(actual["mean_total_loss"], 0.1)
        for row in summaries[1:]:
            self.assertEqual(row["mismatches"], 0)
            self.assertEqual(row["mean_total_loss"], "")


if __name__ == "__main__":
    unittest.main()
