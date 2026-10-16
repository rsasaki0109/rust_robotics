"""Linear projection identities and complete common-error accounting."""

import math
import random
import unittest

import analyze_contrast_common_errors as analysis


def point(error=(0.2, 0.1), parts=None):
    parts = parts or dict(
        noise=(0.0, 0.0), anchor=error, drift=(0.0, 0.0), transport=(0.0, 0.0)
    )
    return dict(
        error_x=error[0],
        error_y=error[1],
        error_norm=math.hypot(*error),
        **{
            p + "_" + axis: v
            for p, xy in parts.items()
            for axis, v in zip(["x", "y"], xy)
        }
    )


def record(sid="1", pairing="raw_recovery"):
    return dict(
        source_id=sid,
        cohort="known-white",
        variant="epoch",
        mode="none",
        seed="269",
        scenario="fixture",
        planner="A*",
        source_event_ids="1+2",
        pairing=pairing,
        anchor_mode="shared_single",
        available=True,
        category="agree_both_outside",
        a_estimator="recovery_mean",
        b_estimator="motion_filter",
        **analysis.project(0.0104, 0.0104, 0.0104, point(), point())
    )


class ContrastCommonErrorTests(unittest.TestCase):
    def test_independent_equal_inputs_leave_nonzero_midpoint_residual(self):
        r = analysis.projection_model(0.02, 0.02, 0)
        self.assertEqual(r["projection_coefficient"], 0)
        self.assertAlmostEqual(r["contrast_variance"], 0.04)
        self.assertAlmostEqual(r["residual_variance"], 0.01)

    def test_shared_offset_remains_after_contrast_projection(self):
        r = analysis.projection_model(0.02, 0.012, 0.01)
        self.assertAlmostEqual(r["a_weight"], 1 / 6)
        self.assertAlmostEqual(r["b_weight"], 5 / 6)
        self.assertAlmostEqual(r["residual_variance"], 0.01 + 0.01 / 6)

    def test_identical_biased_inputs_have_zero_contrast_and_nonzero_residual(self):
        r = analysis.project(0.0104, 0.0104, 0.0104, point(), point())
        self.assertTrue(r["zero_or_roundoff_contrast"])
        self.assertEqual(r["contrast_norm"], 0)
        self.assertAlmostEqual(r["residual_variance"], 0.0104)
        self.assertAlmostEqual(r["residual_norm"], math.hypot(0.2, 0.1))
        self.assertEqual(r["residual_anchor_x"], 0.2)

    def test_unequal_independent_variances_give_fixed_model_weights(self):
        r = analysis.projection_model(0.04, 0.01, 0)
        self.assertAlmostEqual(r["a_weight"], 0.2)
        self.assertAlmostEqual(r["b_weight"], 0.8)
        self.assertAlmostEqual(r["residual_variance"], 0.008)

    def test_common_translation_is_invisible_in_contrast_and_remains_in_residual(self):
        a, b = point((0.3, 0.1)), point((-0.1, 0.4))
        r = analysis.project(0.02, 0.012, 0.01, a, b)
        shifted = analysis.project(
            0.02, 0.012, 0.01, point((0.8, -0.2)), point((0.4, 0.1))
        )
        for axis, delta in [("x", 0.5), ("y", -0.3)]:
            self.assertAlmostEqual(r["contrast_" + axis], shifted["contrast_" + axis])
            self.assertAlmostEqual(
                shifted["midpoint_" + axis] - r["midpoint_" + axis], delta
            )
            self.assertAlmostEqual(
                shifted["residual_" + axis] - r["residual_" + axis], delta
            )

    def test_swapping_inputs_reverses_contrast_and_coefficient_but_preserves_residual(
        self,
    ):
        a, b = point((0.3, 0.1)), point((-0.1, 0.4))
        r = analysis.project(0.02, 0.012, 0.01, a, b)
        swapped = analysis.project(0.012, 0.02, 0.01, b, a)
        self.assertAlmostEqual(
            r["projection_coefficient"], -swapped["projection_coefficient"]
        )
        self.assertAlmostEqual(r["a_weight"], swapped["b_weight"])
        for axis in ["x", "y"]:
            self.assertAlmostEqual(r["contrast_" + axis], -swapped["contrast_" + axis])
            self.assertAlmostEqual(r["midpoint_" + axis], swapped["midpoint_" + axis])
            self.assertAlmostEqual(r["residual_" + axis], swapped["residual_" + axis])

    def test_negative_weights_are_preserved(self):
        r = analysis.projection_model(0.04, 0.01, 0.015)
        self.assertTrue(r["negative_weight"])
        self.assertAlmostEqual(r["a_weight"], -0.25)
        self.assertAlmostEqual(r["b_weight"], 1.25)
        self.assertAlmostEqual(r["residual_variance"], 0.00875)

    def test_rank_one_scaled_errors_can_have_zero_model_residual(self):
        r = analysis.project(0.04, 0.01, 0.02, point((0.4, 0)), point((0.2, 0)))
        self.assertAlmostEqual(r["residual_variance"], 0)
        self.assertAlmostEqual(r["residual_norm"], 0)

    def test_anticorrelated_inputs_can_have_zero_midpoint(self):
        r = analysis.project(0.01, 0.01, -0.01, point((0.2, 0)), point((-0.2, 0)))
        self.assertAlmostEqual(r["midpoint_variance"], 0)
        self.assertAlmostEqual(r["residual_variance"], 0)
        self.assertAlmostEqual(r["contrast_variance"], 0.04)
        self.assertAlmostEqual(r["residual_norm"], 0)

    def test_near_zero_contrast_uses_explicit_roundoff_branch(self):
        r = analysis.projection_model(0.01, 0.01, 0.01 - 5e-14)
        self.assertTrue(r["zero_or_roundoff_contrast"])
        self.assertGreater(r["contrast_variance"], 0)
        self.assertEqual(r["projection_coefficient"], 0)
        self.assertEqual(r["residual_variance"], r["midpoint_variance"])
        with self.assertRaises(AssertionError):
            analysis.projection_model(-0.01, 0.01, 0)
        with self.assertRaises(AssertionError):
            analysis.projection_model(0.01, 0.01, 0.02)

    def test_linear_component_vectors_close_with_cancellation(self):
        a = point(
            (0.2, 0.1),
            dict(
                noise=(0.1, -0.1), anchor=(0.2, 0.2), drift=(-0.1, 0), transport=(0, 0)
            ),
        )
        b = point(
            (0.1, -0.2),
            dict(
                noise=(-0.1, 0),
                anchor=(0.3, -0.1),
                drift=(-0.1, -0.1),
                transport=(0, 0),
            ),
        )
        r = analysis.project(0.02, 0.012, 0.01, a, b)
        for axis in ["x", "y"]:
            self.assertAlmostEqual(
                sum(r["residual_" + p + "_" + axis] for p in analysis.PARTS),
                r["residual_" + axis],
            )
        with self.assertRaises(AssertionError):
            analysis.project(0.02, 0.012, 0.01, dict(a, anchor_x=0), b)

    def test_variance_partition_and_orthogonality_on_psd_covariances(self):
        rng = random.Random(275)
        for _ in range(100):
            x, y = [[rng.uniform(-0.1, 0.1) for _ in range(4)] for _ in range(2)]
            va, vb = sum(v * v for v in x), sum(v * v for v in y)
            c = sum(a * b for a, b in zip(x, y))
            r = analysis.projection_model(va, vb, c)
            self.assertAlmostEqual(
                r["midpoint_variance"],
                r["residual_variance"] + r["explained_midpoint_variance"],
            )
            self.assertAlmostEqual(r["residual_contrast_covariance"], 0)
            self.assertGreaterEqual(r["residual_variance"], 0)

    def test_observed_vectors_do_not_choose_projection_coefficient(self):
        a = analysis.project(0.04, 0.01, 0.015, point(), point())
        b = analysis.project(0.04, 0.01, 0.015, point((12.0, -8.0)), point((-0.4, 2.0)))
        self.assertEqual(a["projection_coefficient"], b["projection_coefficient"])
        self.assertEqual(a["residual_variance"], b["residual_variance"])

    def test_simultaneous_summary_preserves_both_loss_populations_and_empty_groups(
        self,
    ):
        rows = [
            record(str(i), a + "_" + b) for i, (a, b) in enumerate(analysis.PAIRINGS, 1)
        ]
        link = {
            **{k: rows[0][k] for k in analysis.KEY},
            "event_id": "1",
            "command_only_zero_loss": "True",
            **{
                a + "_" + b + "_pair_id": str(i)
                for i, (a, b) in enumerate(analysis.PAIRINGS, 1)
            },
        }
        events = {"1": dict(event_kind="received"), "2": dict(event_kind="missed")}
        summary = analysis.simultaneous_summary(rows, [link], events)
        self.assertEqual(len(summary), 540)
        for pop in analysis.POPULATIONS:
            r = next(
                s
                for s in summary
                if (
                    s["cohort"],
                    s["variant"],
                    s["event_kind"],
                    s["population"],
                    s["pairing"],
                )
                == ("all", "epoch", "all", pop, "recovery_preview")
            )
            self.assertEqual(
                (
                    r["records"],
                    r["matched"],
                    r["zero_or_roundoff_pairs"],
                    r["category_agree_both_outside"],
                ),
                (1, 1, 1, 1),
            )
            self.assertEqual(r["opportunities"], 2 if pop == "all_states" else 1)
        self.assertTrue(
            all(
                s["records"] == 0 and s["observed_residual_rms"] == ""
                for s in summary
                if s["event_kind"] == "precision_only"
            )
        )

    def test_lagged_summary_keeps_missing_projection_blank_and_source_labels(self):
        good = dict(
            record(),
            event_id="1",
            event_kind="received",
            phase="immediate",
            input_kind="actual",
            category="both_outside",
        )
        missing = dict(
            good,
            source_id="2",
            event_id="2",
            available=False,
            category="unavailable",
            b_estimator="",
        )
        for field in analysis.project(0.0104, 0.0104, 0.0104, point(), point()):
            missing[field] = ""
        summary = analysis.lagged_summary([good, missing], {})
        self.assertEqual(len(summary), 480)
        r = next(
            s
            for s in summary
            if (s["cohort"], s["variant"], s["event_kind"], s["phase"], s["input_kind"])
            == ("all", "epoch", "all", "immediate", "actual")
        )
        self.assertEqual(
            (
                r["records"],
                r["matched"],
                r["unavailable"],
                r["category_both_outside"],
                r["category_unavailable"],
            ),
            (2, 1, 1, 1, 1),
        )
        self.assertEqual(r["b_estimator_unavailable"], 1)
        self.assertAlmostEqual(r["pooled_sigma_residual_variance"], math.sqrt(0.0104))


if __name__ == "__main__":
    unittest.main()
