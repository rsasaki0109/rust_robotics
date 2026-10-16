"""Pin outcome-independent holdout selection and logged precision margins."""

import math
import unittest

from analyze_epoch_selection_holdout import (
    case_outcome,
    checkpoint_reason,
    reference_decision,
)


class EpochSelectionHoldoutTests(unittest.TestCase):
    def test_final_is_kept_when_moving_stale_or_on_a_fresh_grid_time(self):
        before = dict(v="1")
        for time in [40, 150.5, 180]:
            self.assertEqual(
                checkpoint_reason(
                    before, dict(time_s=str(time), observation_age="10"), True
                ),
                "final",
            )

    def test_fixed_fresh_times_include_both_endpoints_only(self):
        for time in [150.5, 155.5, 160.5, 165.5, 170.5, 175.5]:
            self.assertEqual(
                checkpoint_reason(
                    dict(v="-0.02"),
                    dict(time_s=str(time), observation_age="0.5"),
                    False,
                ),
                "fresh_stationary",
            )
        for time in [149.5, 150, 150.50001, 151, 180.5]:
            self.assertIsNone(
                checkpoint_reason(
                    dict(v="0"), dict(time_s=str(time), observation_age="0.5"), False
                )
            )

    def test_fresh_requires_age_and_speed_but_allows_precision_sigma(self):
        row = dict(
            time_s="150.5",
            observation_age="0.5",
            effective_position_sigma="0.03",
            precision_active="true",
        )
        self.assertEqual(
            checkpoint_reason(dict(v="0.02"), row, False), "fresh_stationary"
        )
        self.assertIsNone(checkpoint_reason(dict(v="0.020001"), row, False))
        self.assertIsNone(
            checkpoint_reason(dict(v="0"), dict(row, observation_age="0.6"), False)
        )

    def test_case_outcomes_are_labels_including_held_and_contacts(self):
        self.assertEqual(
            case_outcome(dict(collision="false", held="true", status="complete")),
            "held",
        )
        self.assertEqual(
            case_outcome(dict(collision="false", held="false", status="timeout")),
            "timeout",
        )
        self.assertEqual(
            case_outcome(dict(collision="true", held="false", status="collision")),
            "collision",
        )
        self.assertEqual(
            case_outcome(dict(collision="false", held="false", status="other")), "other"
        )

    def test_precision_margin_is_retained_instead_of_forcing_sigma_point_one(self):
        pose = (0.75, 25, 0)
        ordinary = reference_decision(pose, pose, 0.6 + 2 * math.sqrt(2) * 0.1, [])
        precise = reference_decision(pose, pose, 0.6 + 2 * math.sqrt(2) * 0.03, [])
        self.assertEqual(ordinary["steering_deg"], precise["steering_deg"])
        self.assertAlmostEqual(ordinary["estimated_clearance"], 0.75)
        self.assertEqual(ordinary["accepted"], "False")
        self.assertEqual(precise["accepted"], "True")
        self.assertEqual(precise["accepted_true_pass"], "True")


if __name__ == "__main__":
    unittest.main()
