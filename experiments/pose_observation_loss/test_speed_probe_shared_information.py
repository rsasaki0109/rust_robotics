"""Sparse AR covariance, shared offsets, PSD checks and agreement accounting."""

import math
import random
import unittest

import analyze_speed_probe_shared_information as analysis


def source(pid="1", kind="raw", noise=0.02):
    return dict(
        pose_id=pid,
        state_id="1",
        chain_id="1",
        chain_index="0",
        cohort="known-white",
        variant="epoch",
        mode="none",
        seed="269",
        scenario="fixture",
        planner="A*",
        time_s="150.02",
        source_event_ids="1",
        truth_x="10",
        truth_y="10",
        pose_kind=kind,
        estimator="raw_observation" if kind == "raw" else "recovery_mean",
        pose_x=10 + 0.25 + noise,
        pose_y=10.0,
        pose_yaw=0.0,
        error_norm=0.25 + noise,
        noise_x=noise,
        noise_y=0.0,
        anchor_x=0.25,
        anchor_y=0.0,
        drift_x=0.0,
        drift_y=0.0,
        transport_x=0.0,
        transport_y=0.0,
    )


def info(step):
    return dict(
        samples={step: 0.1},
        anchors={"100": 1.0},
        noise=0.01,
        anchor=0.01,
        rho=0.0,
        current_anchor="100",
    )


class SharedInformationTests(unittest.TestCase):
    def test_white_noise_covariance_only_counts_shared_steps(self):
        a, b = {1: 0.05, 2: 0.05}, {2: 0.1, 3: 0.1}
        self.assertAlmostEqual(analysis.ar_covariance(a, b, 0), 0.005)
        self.assertEqual(analysis.ar_covariance({1: 0.1}, {3: 0.1}, 0), 0)

    def test_disjoint_observations_can_share_temporal_ar_noise(self):
        self.assertAlmostEqual(
            analysis.ar_covariance({1: 0.1}, {5: 0.2}, 0.9), 0.02 * 0.9**4
        )

    def test_sparse_recurrence_matches_full_double_sum(self):
        rng = random.Random(275)
        for rho in [0, 0.3, math.exp(-0.02), 1]:
            for _ in range(20):
                a = {s: rng.uniform(-0.2, 0.2) for s in rng.sample(range(1, 70), 13)}
                b = {s: rng.uniform(-0.2, 0.2) for s in rng.sample(range(1, 70), 17)}
                expected = math.fsum(
                    x * y * rho ** abs(i - j)
                    for i, x in a.items()
                    for j, y in b.items()
                )
                self.assertAlmostEqual(
                    analysis.ar_covariance(a, b, rho), expected, places=13
                )
                self.assertAlmostEqual(
                    analysis.ar_covariance(a, b, rho),
                    analysis.ar_covariance(b, a, rho),
                    places=14,
                )

    def test_perfect_temporal_correlation_uses_total_weights(self):
        a, b = {1: 0.02, 10: 0.03}, {50: 0.04, 100: 0.06}
        self.assertAlmostEqual(analysis.ar_covariance(a, b, 1), 0.05 * 0.1)

    def test_identical_estimates_can_have_large_shared_error_and_zero_difference_variance(
        self,
    ):
        a, b = source(), source("2", "recovery")
        p = analysis.pair_record(1, a, b, info(1), info(1), 0.01)
        self.assertAlmostEqual(p["position_separation"], 0)
        self.assertAlmostEqual(p["difference_variance_joint"], 0)
        self.assertAlmostEqual(p["correlation_joint"], 1)
        self.assertGreater(p["independent_sigma_per_axis"], 0.19)
        self.assertEqual(analysis.category(p, 0.02), "agree_both_outside")

    def test_shared_anchor_cancels_in_difference_not_in_marginals(self):
        p = analysis.model(0.0004, 0.01, 0.0004, 0.01, 0, 0.01)
        self.assertAlmostEqual(p["difference_variance_anchor"], 0)
        self.assertAlmostEqual(p["difference_variance_joint"], 0.0008)
        self.assertAlmostEqual(p["independent_variance_joint"], 0.0208)

    def test_three_epoch_cross_covariance_only_shares_current_anchor(self):
        p = analysis.model(0, 0.01, 0, 0.01 / 3, 0, 0.01 / 3)
        self.assertAlmostEqual(p["difference_variance_anchor"], 0.01 * 2 / 3)
        self.assertAlmostEqual(p["correlation_joint"], 1 / math.sqrt(3))

    def test_negative_roundoff_is_clamped_but_invalid_variance_is_rejected(self):
        self.assertEqual(analysis.nonnegative(-5e-13), 0)
        self.assertEqual(analysis.nonnegative(5e-15), 5e-15)
        with self.assertRaises(AssertionError):
            analysis.nonnegative(-1e-8)
        with self.assertRaises(AssertionError):
            analysis.model(0.01, 0, 0.01, 0, 0.02, 0)

    def test_three_input_psd_catches_incompatible_pairwise_correlations(self):
        with self.assertRaises(AssertionError):
            analysis.audit_matrix([1, 1, 1], [0.9, 0.9, -0.9])
        self.assertAlmostEqual(analysis.audit_matrix([1, 1, 1], [1, 1, 1]), 0)
        self.assertEqual(analysis.audit_matrix([0, 0, 0], [0, 0, 0]), 0)

    def test_observed_difference_keeps_component_closure_and_truth_cancels(self):
        p = analysis.pair_record(
            1, source(), source("2", "recovery", -0.01), info(1), info(2), 0.01
        )
        self.assertAlmostEqual(p["delta_x"], 0.03)
        self.assertAlmostEqual(p["delta_noise_norm"], 0.03)
        self.assertAlmostEqual(p["delta_anchor_norm"], 0)
        self.assertEqual(analysis.category(p, 0.05), "agree_both_outside")

    def test_all_agreement_bins_and_inclusive_threshold(self):
        expected = analysis.BINS[:4]
        actual = [
            analysis.category(
                dict(position_separation=0.05, a_inside=a, b_inside=b), 0.05
            )
            for a, b in [(True, True), (True, False), (False, True), (False, False)]
        ]
        self.assertEqual(actual, expected)
        self.assertEqual(
            analysis.category(
                dict(position_separation=math.nextafter(0.05, math.inf)), 0.05
            ),
            "disagree",
        )

    def test_information_matches_marginals_and_anchor_weights(self):
        p = dict(
            pose_kind="recovery",
            sample_sigma_min=".1",
            sample_sigma_max=".1",
            used_sample_count="2",
            source_group_count="1",
            anchor_x=".25",
            anchor_y="0",
            model_anchor_sigma_per_axis=".1",
            model_noise_sigma_per_axis=str(0.1 / math.sqrt(2)),
            model_noise_anchor_sigma_per_axis=str(math.sqrt(0.015)),
        )
        g = dict(
            sample_count="2",
            group_weight="1",
            first_sample_step="10",
            last_sample_step="11",
            anchor_ids="5",
            anchor_x=".25",
            anchor_y="0",
        )
        r = analysis.information(p, [g], "5", 0.01, 0)
        self.assertEqual(r["anchors"], {"5": 1})
        self.assertEqual(r["samples"], {10: 0.05, 11: 0.05})
        with self.assertRaises(AssertionError):
            analysis.information(dict(p, sample_sigma_max=".2"), [g], "5", 0.01, 0)
        with self.assertRaises(AssertionError):
            analysis.information(p, [dict(g, anchor_ids="5+6")], "5", 0.01, 0)
        with self.assertRaises(AssertionError):
            analysis.information(p, [g, g], "5", 0.01, 0)

    def test_summary_preserves_full_loss_denominator_and_empty_anchor_modes(self):
        pairs = [
            analysis.pair_record(
                i, source(kind=a), source(str(i + 4), b), info(1), info(1), 0.01
            )
            for i, (a, b) in enumerate(analysis.PAIRINGS, 1)
        ]
        links = [
            dict(
                event_id="1",
                command_only_zero_loss=True,
                **{
                    a + "_" + b + "_pair_id": i
                    for i, (a, b) in enumerate(analysis.PAIRINGS, 1)
                }
            )
        ]
        rows = analysis.summarize(pairs, links, {"1": dict(event_kind="received")})
        rr = [
            r
            for r in rows
            if (
                r["cohort"],
                r["variant"],
                r["event_kind"],
                r["population"],
                r["anchor_mode"],
                r["agreement_threshold_m"],
            )
            == ("all", "epoch", "all", "raw_zero_loss", "all", 0.05)
        ]
        self.assertEqual(len(rr), 3)
        self.assertTrue(
            all(
                r["records"]
                == r["cases"]
                == r["opportunities"]
                == r["agree_both_outside"]
                == 1
                for r in rr
            )
        )
        self.assertTrue(
            all(
                r["records"] == 0 and r["pooled_difference_sigma"] == ""
                for r in rows
                if r["anchor_mode"] == "preview_three"
            )
        )
        self.assertTrue(
            all(r["records"] == 0 for r in rows if r["event_kind"] == "precision_only")
        )


if __name__ == "__main__":
    unittest.main()
