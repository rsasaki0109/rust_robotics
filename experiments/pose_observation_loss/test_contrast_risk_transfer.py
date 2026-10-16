"""Seed separation, exact quantiles, tied AUC and case-balanced retention."""

from fractions import Fraction
import random
import unittest

import analyze_contrast_risk_transfer as analysis


def row(
    sid="1",
    partition="known",
    noise="white",
    variant="epoch",
    seed=None,
    tick=10,
    outside=True,
    view=None,
):
    seed = seed or ("269" if partition == "known" else "274")
    return dict(
        source_family="simultaneous",
        source_id=sid,
        cohort=partition + "-" + noise,
        partition=partition,
        noise=noise,
        variant=variant,
        mode="none",
        seed=seed,
        scenario="fixture",
        planner="A*",
        view=view or analysis.VIEWS[0],
        available=True,
        source_event_ids="1",
        zero_or_roundoff_contrast=False,
        **{name + "_tick": tick for name in analysis.SCORES},
        **{target + "_outside": outside for target in analysis.TARGETS},
        **{target + "_error_m": 0.2 if outside else 0.1 for target in analysis.TARGETS}
    )


def threshold(rows, **wanted):
    fields = dict(noise="white", view=analysis.VIEWS[0], score="contrast", budget="q50")
    fields.update(wanted)
    return next(
        t
        for t in analysis.fit_thresholds(rows)
        if all(t[k] == v for k, v in fields.items())
    )


class ContrastRiskTransferTests(unittest.TestCase):
    def test_exact_seed_membership_and_disjoint_partitions(self):
        self.assertEqual(analysis.membership(row()), ("known", "white"))
        self.assertEqual(
            analysis.membership(row(partition="new", noise="correlated")),
            ("new", "correlated"),
        )
        self.assertFalse(analysis.PARTITIONS["known"] & analysis.PARTITIONS["new"])
        for r in [
            dict(row(), seed="274"),
            dict(row(), cohort="unknown-white"),
            dict(row(), cohort="known-other"),
        ]:
            with self.assertRaises(AssertionError):
                analysis.membership(r)

    def test_score_ticks_join_reconstruction_scale_ties(self):
        self.assertEqual(analysis.score_tick(0.1), analysis.score_tick(0.1 + 1e-10))
        self.assertEqual(
            analysis.score_tick(0.1 + 1.1e-8), analysis.score_tick(0.1) + 1
        )
        for value in [float("nan"), float("inf"), -0.1]:
            with self.assertRaises(AssertionError):
                analysis.score_tick(value)

    def test_each_case_has_unit_weight_despite_different_reference_counts(self):
        rows = [row(str(i)) for i in range(9)] + [row("10", seed="270")]
        weights = analysis.case_weights(rows)
        self.assertEqual(sum(weights[:9]), 1)
        self.assertEqual(weights[-1], 1)
        self.assertEqual(sum(weights), 2)
        with self.assertRaises(AssertionError):
            analysis.case_weights([dict(row(), available=False)])

    def test_exact_fraction_quantile_does_not_jump_at_accumulation_boundary(self):
        ticks = [1, 1, 1, 9, 9, 9, 9, 9, 9, 9, 9, 9]
        weights = [Fraction(1, 3)] * 3 + [Fraction(1, 9)] * 9
        self.assertEqual(analysis.weighted_quantile(ticks, weights, Fraction(1, 2)), 1)
        self.assertEqual(analysis.weighted_quantile(ticks, weights, Fraction(3, 4)), 9)

    def test_quantile_ties_are_all_retained_even_beyond_requested_fraction(self):
        rows = [
            row("1", tick=4),
            row("2", seed="270", tick=4),
            row("3", seed="271", tick=4),
        ]
        t = threshold(rows, budget="q25")
        metrics = analysis.retained_metrics(
            rows, analysis.case_weights(rows), "contrast", t
        )
        self.assertEqual(t["threshold_tick"], 4)
        self.assertEqual(metrics["retained_rows"], 3)
        self.assertEqual(metrics["case_weighted_coverage"], 1)

    def test_new_cohort_scores_and_labels_cannot_change_fitted_thresholds(self):
        known = [row("1", tick=2), row("2", seed="270", tick=5)]
        new = row("3", partition="new", tick=0, outside=False)
        a = analysis.fit_thresholds([*known, new])
        altered = dict(
            new,
            **{name + "_tick": 999999 for name in analysis.SCORES},
            **{target + "_outside": True for target in analysis.TARGETS}
        )
        self.assertEqual(a, analysis.fit_thresholds([*known, altered]))
        self.assertTrue(all(t["training_partition"] == "known" for t in a))

    def test_known_arms_pool_into_one_shared_threshold(self):
        rows = [row("1", tick=2), row("2", variant="curved", tick=8)]
        t = threshold(rows)
        self.assertEqual(
            (t["training_rows"], t["training_cases"], t["threshold_tick"]), (2, 2, 2)
        )
        new = [
            row("3", partition="new", tick=3),
            row("4", partition="new", variant="curved", tick=3),
        ]
        for r in new:
            m = analysis.retained_metrics([r], [Fraction(1)], "contrast", t)
            self.assertEqual(m["retained_rows"], 0)

    def test_noise_and_view_training_groups_remain_separate(self):
        rows = [
            row("1", tick=2),
            row("2", noise="correlated", tick=8),
            row("3", view=analysis.VIEWS[1], tick=15),
        ]
        self.assertEqual(threshold(rows)["threshold_tick"], 2)
        self.assertEqual(threshold(rows, noise="correlated")["threshold_tick"], 8)
        self.assertEqual(threshold(rows, view=analysis.VIEWS[1])["threshold_tick"], 15)

    def test_empty_fit_does_not_borrow_evaluation_data(self):
        rows = [row(partition="new", tick=4)]
        t = threshold(rows)
        self.assertEqual(
            (t["rule"], t["threshold_tick"], t["training_rows"]),
            ("no_training_data", "", 0),
        )
        self.assertEqual(
            analysis.retained_metrics(rows, [Fraction(1)], "contrast", t)[
                "retained_rows"
            ],
            "",
        )
        all_rule = threshold(rows, budget="all")
        self.assertEqual(
            analysis.retained_metrics(rows, [Fraction(1)], "contrast", all_rule)[
                "retained_rows"
            ],
            1,
        )

    def test_auc_orientation_ties_and_single_class_are_explicit(self):
        self.assertEqual(analysis.auc([0, 1], [False, True], [1, 1]), 1)
        self.assertEqual(analysis.auc([1, 0], [False, True], [1, 1]), 0)
        self.assertEqual(analysis.auc([4, 4], [False, True], [1, 1]), 0.5)
        self.assertEqual(analysis.auc([1, 2], [True, True], [1, 1]), "")
        self.assertEqual(analysis.auc([], [], []), "")

    def test_weighted_auc_matches_independent_positive_negative_pair_comparisons(self):
        rng = random.Random(275)
        for _ in range(30):
            ticks = [rng.randrange(4) for _ in range(9)]
            labels = [False, True, *[bool(rng.randrange(2)) for _ in range(7)]]
            weights = [Fraction(1, rng.randrange(1, 8)) for _ in ticks]
            credit = Fraction()
            denominator = Fraction()
            for i in range(len(ticks)):
                for j in range(len(ticks)):
                    if labels[i] and not labels[j]:
                        pair = weights[i] * weights[j]
                        denominator += pair
                        credit += pair * (
                            1
                            if ticks[i] > ticks[j]
                            else Fraction(1, 2) if ticks[i] == ticks[j] else 0
                        )
            self.assertAlmostEqual(
                analysis.auc(ticks, labels, weights), float(credit / denominator)
            )

    def test_retention_keeps_original_case_weights_without_renormalizing_selected_cases(
        self,
    ):
        rows = [row("1", tick=0), row("2", tick=4), row("3", seed="270", tick=4)]
        weights = analysis.case_weights(rows)
        t = dict(rule="fitted", threshold_tick=0)
        r = analysis.retained_metrics(rows, weights, "contrast", t)
        self.assertEqual(
            (r["retained_rows"], r["retained_cases"], r["retained_case_mass"]),
            (1, 1, 0.5),
        )
        self.assertEqual(r["case_weighted_coverage"], 0.25)
        self.assertAlmostEqual(r["row_coverage"], 1 / 3)
        self.assertEqual(r["residual_case_weighted_risk"], 1)
        self.assertAlmostEqual(r["residual_case_weighted_rms"], 0.2)

    def test_all_available_reference_keeps_empty_and_full_group_semantics(self):
        t = dict(rule="all_available")
        r = analysis.retained_metrics([], [], "contrast", t)
        self.assertEqual(r["retained_rows"], 0)
        self.assertEqual(r["case_weighted_coverage"], "")
        rows = [row("1", outside=True), row("2", seed="270", outside=False)]
        r = analysis.retained_metrics(rows, analysis.case_weights(rows), "contrast", t)
        self.assertEqual(r["case_weighted_coverage"], 1)
        self.assertEqual(r["residual_case_weighted_risk"], 0.5)

    def test_source_scoring_preserves_closed_error_boundary_and_censoring(self):
        base = dict(
            row(),
            a_id="1",
            b_id="2",
            category="agree_both_inside",
            pairing="raw_recovery",
            available="True",
            zero_or_roundoff_contrast="True",
            contrast_norm="0",
            midpoint_variance=".01",
            residual_variance=".01",
            midpoint_norm=".1",
            residual_norm=str(0.1 + 1e-10),
        )
        lag = dict(
            base,
            source_id="2",
            event_id="1",
            input_kind="actual",
            phase="immediate",
            available="False",
            category="unavailable",
        )
        for field in [
            "contrast_norm",
            "midpoint_variance",
            "residual_variance",
            "midpoint_norm",
            "residual_norm",
        ]:
            lag[field] = ""
        events = {"1": {k: base[k] for k in analysis.KEY}}
        scored = analysis.score_rows([base], [lag], [], events)
        self.assertFalse(scored[0]["midpoint_outside"])
        self.assertTrue(scored[0]["residual_outside"])
        self.assertEqual(scored[0]["contrast_tick"], 0)
        self.assertTrue(scored[1]["oracle_contrast"])
        self.assertFalse(scored[1]["available"])
        for k in ["contrast_tick", "midpoint_outside", "residual_error_m"]:
            self.assertEqual(scored[1][k], "")

    def test_complete_summary_groups_keep_event_unions_and_unavailable_requests(self):
        good = dict(row(), source_event_ids="1+2")
        missing = dict(
            row("2", partition="new"), available=False, zero_or_roundoff_contrast=""
        )
        events = {"1": dict(event_kind="received"), "2": dict(event_kind="missed")}
        thresholds = analysis.fit_thresholds([good, missing])
        ranking, risks = analysis.summaries([good, missing], thresholds, events)
        self.assertEqual((len(ranking), len(risks)), (1584, 6336))
        select = (
            lambda r, part, kind: r["partition"] == part
            and r["noise"] == "white"
            and r["variant"] == "epoch"
            and r["event_kind"] == kind
            and r["view"] == analysis.VIEWS[0]
            and r["score"] == "contrast"
        )
        a = next(r for r in ranking if select(r, "known", "all"))
        self.assertEqual(
            (a["records"], a["matched"], a["opportunities"], a["case_mass"]),
            (1, 1, 2, 1),
        )
        self.assertEqual(a["residual_case_weighted_auc"], "")
        for kind in ["received", "missed"]:
            r = next(r for r in ranking if select(r, "known", kind))
            self.assertEqual((r["records"], r["opportunities"]), (1, 1))
        b = next(r for r in ranking if select(r, "new", "all"))
        self.assertEqual((b["records"], b["matched"], b["unavailable"]), (1, 0, 1))
        self.assertTrue(
            all(
                r["records"] == 0 and r["residual_case_weighted_auc"] == ""
                for r in ranking
                if r["event_kind"] == "precision_only"
            )
        )


if __name__ == "__main__":
    unittest.main()
