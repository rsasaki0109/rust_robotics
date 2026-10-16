"""Training isolation, conditional weights, ties, support and selection overlap."""

from fractions import Fraction as F
import unittest

import analyze_mode_conditional_cutoffs as analysis
from analyze_contrast_risk_transfer import fit_thresholds


def row(
    sid="1",
    seed=None,
    partition="known",
    noise="white",
    variant="epoch",
    stratum="filter_single",
    tick=2,
    error=0.2,
    view=None,
):
    seed = seed or ("269" if partition == "known" else "274")
    estimator, anchor = analysis.STRATA[stratum]
    view = view or (
        "sim_raw_recovery" if stratum == "recovery_single" else "sim_recovery_preview"
    )
    return dict(
        source_family="simultaneous",
        source_id=sid,
        cohort=partition + "-" + noise,
        partition=partition,
        noise=noise,
        variant=variant,
        mode="random",
        seed=seed,
        scenario="fixture",
        planner="A*",
        view=view,
        source_event_ids="1",
        a_id="10",
        b_id="11",
        stratum=stratum,
        available=True,
        b_estimator=estimator,
        anchor_mode=anchor,
        **{s + "_tick": tick for s in analysis.SCORES},
        **{s + "_score_m": tick * analysis.QUANTUM for s in analysis.SCORES},
        **{t + "_outside": error > 0.1 for t in analysis.TARGETS},
        **{t + "_error_m": error for t in analysis.TARGETS}
    )


def cut(rows, **wanted):
    fields = dict(
        noise="white",
        view="sim_recovery_preview",
        stratum="filter_single",
        score="contrast",
        budget="q50",
    )
    fields.update(wanted)
    return next(
        t
        for t in analysis.fit_mode_thresholds(rows)
        if all(t[k] == v for k, v in fields.items())
    )


def event_rows(r, kinds=("received",)):
    return {
        str(i + 1): dict(event_kind=k, **{field: r[field] for field in analysis.KEY})
        for i, k in enumerate(kinds)
    }


class ModeConditionalCutoffsTests(unittest.TestCase):
    def test_fit_ignores_new_scores_labels_and_case_lengths(self):
        known = [row("1", tick=2), row("2", seed="270", tick=8)]
        initial = analysis.fit_mode_thresholds(
            [*known, row("3", partition="new", tick=0, error=0.01)]
        )
        changed = analysis.fit_mode_thresholds(
            [
                *known,
                *[
                    row(str(3 + i), partition="new", tick=999, error=1.0)
                    for i in range(30)
                ],
            ]
        )
        self.assertEqual(initial, changed)

    def test_known_labels_do_not_affect_conditional_quantiles(self):
        rr = [row("1", tick=2), row("2", seed="270", tick=8)]
        changed = [
            dict(
                r,
                midpoint_outside=False,
                residual_outside=False,
                midpoint_error_m=0.01,
                residual_error_m=0.01,
            )
            for r in rr
        ]
        self.assertEqual(
            analysis.fit_mode_thresholds(rr), analysis.fit_mode_thresholds(changed)
        )

    def test_both_arms_share_one_fitted_mode_cutoff(self):
        rr = [row("1", tick=2), row("2", variant="curved", tick=8)]
        t = cut(rr)
        self.assertEqual(
            (t["threshold_tick"], t["training_rows"], t["training_cases"]), (2, 2, 2)
        )
        self.assertNotIn("variant", t)
        for arm in analysis.ARMS:
            self.assertFalse(
                analysis.decision(
                    row(partition="new", variant=arm, tick=3), "contrast", t
                )
            )

    def test_parent_case_weights_are_not_rebalanced_within_modes(self):
        rr = [
            row("1", tick=1),
            *[row(str(i + 2), stratum="fusion_three", tick=4) for i in range(3)],
            row("5", seed="270", tick=9),
        ]
        t = cut(rr)
        self.assertEqual(t["training_case_mass"], F(5, 4))
        self.assertEqual(t["training_parent_case_mass"], 2)
        self.assertEqual(t["threshold_tick"], 9)
        self.assertEqual(t["training_cases"], 2)

    def test_exact_fraction_boundary_uses_lower_conditional_quantile(self):
        rr = []
        for seed, total, in_mode, tick in [("269", 9, 3, 1), ("270", 27, 9, 9)]:
            for i in range(total):
                rr.append(
                    row(
                        str(len(rr) + 1),
                        seed=seed,
                        stratum="filter_single" if i < in_mode else "fusion_three",
                        tick=tick,
                    )
                )
        t = cut(rr)
        self.assertEqual(t["training_case_mass"], F(2, 3))
        self.assertEqual(t["threshold_tick"], 1)
        self.assertEqual(t["training_retained_case_mass"], F(1, 3))

    def test_ties_stay_whole_and_training_overshoot_is_explicit(self):
        rr = [
            row(str(i), seed=seed, tick=5)
            for i, seed in enumerate(["269", "270", "271"], 1)
        ]
        t = cut(rr, budget="q25")
        self.assertEqual(t["threshold_tick"], 5)
        self.assertEqual(t["training_mass_below"], 0)
        self.assertEqual(t["training_mass_at"], 3)
        self.assertEqual(t["training_achieved_case_fraction"], 1)
        self.assertEqual(t["training_achieved_row_fraction"], 1)
        self.assertTrue(all(analysis.decision(r, "contrast", t) for r in rr))

    def test_noise_view_and_mode_fit_populations_are_separate(self):
        rr = [
            row("1", tick=2),
            row("2", noise="correlated", tick=8),
            row("3", view="sim_raw_preview", tick=5),
            row("4", stratum="fusion_three", tick=12),
        ]
        self.assertEqual(cut(rr)["threshold_tick"], 2)
        self.assertEqual(cut(rr, noise="correlated")["threshold_tick"], 8)
        self.assertEqual(cut(rr, view="sim_raw_preview")["threshold_tick"], 5)
        self.assertEqual(cut(rr, stratum="fusion_three")["threshold_tick"], 12)

    def test_weight_fingerprint_captures_other_mode_rows_of_same_case(self):
        a = cut([row()])
        b = cut([row(), row("2", stratum="fusion_three")])
        self.assertEqual(a["training_reference_sha256"], b["training_reference_sha256"])
        self.assertNotEqual(
            a["training_weighted_reference_sha256"],
            b["training_weighted_reference_sha256"],
        )
        self.assertEqual(b["training_case_mass"], F(1, 2))

    def test_empty_mode_fit_does_not_borrow_new_data(self):
        rr = [row(), row("2", partition="new", stratum="fusion_three", tick=7)]
        t = cut(rr, stratum="fusion_three")
        self.assertEqual(t["rule"], "no_training_data")
        self.assertIsNone(t["threshold_tick"])
        self.assertEqual(t["training_rows"], 0)
        self.assertIsNone(analysis.decision(rr[1], "contrast", t))
        self.assertTrue(
            analysis.decision(
                rr[1], "contrast", cut(rr, stratum="fusion_three", budget="all")
            )
        )

    def test_unknown_conditional_decision_keeps_complete_metrics_missing(self):
        rr = [row(), row("2", seed="270", error=0.05)]
        stats = analysis.method_metrics(rr, [F(1), F(1)], [True, None])
        self.assertTrue(all(v is None for v in stats.values()))
        overlap = analysis.overlap_metrics(rr, [F(1), F(1)], [True, True], [True, None])
        self.assertTrue(all(v is None for v in overlap.values()))

    def test_reporting_weights_are_preserved_after_selection(self):
        rr = [row("1"), row("2"), row("3", error=0.05), row("4", seed="270", error=0.1)]
        stats = analysis.method_metrics(
            rr, analysis.case_weights(rr), [True, True, False, True]
        )
        self.assertEqual(stats["retained_case_mass"], F(5, 3))
        self.assertEqual(stats["case_weighted_coverage"], F(5, 6))
        self.assertEqual(stats["residual_case_weighted_risk"], F(2, 5))
        self.assertAlmostEqual(stats["residual_case_weighted_rms"], 0.022**0.5)

    def test_equal_coverage_can_add_outside_mass_and_raise_retained_risk(self):
        rr = [
            row("1"),
            row("2", error=0.05),
            row("3"),
            row("4", seed="270", error=0.05),
        ]
        ww = analysis.case_weights(rr)
        p, c = [True, True, False, False], [True, False, True, False]
        a, b = analysis.method_metrics(rr, ww, p), analysis.method_metrics(rr, ww, c)
        overlap = analysis.overlap_metrics(rr, ww, p, c)
        self.assertEqual(a["case_weighted_coverage"], b["case_weighted_coverage"])
        self.assertEqual(a["residual_case_weighted_risk"], F(1, 2))
        self.assertEqual(b["residual_case_weighted_risk"], 1)
        self.assertEqual(
            b["residual_error_case_mass"] - a["residual_error_case_mass"],
            overlap["conditional_only_residual_error_case_mass"]
            - overlap["pooled_only_residual_error_case_mass"],
        )
        self.assertEqual(sum(overlap[s + "_case_mass"] for s in analysis.OVERLAP), 2)

    def test_empty_selection_has_zero_coverage_but_empty_population_has_no_coverage(
        self,
    ):
        empty = analysis.method_metrics([row()], [F(1)], [False])
        self.assertEqual(empty["retained_rows"], 0)
        self.assertEqual(empty["case_weighted_coverage"], 0)
        self.assertIsNone(empty["residual_case_weighted_risk"])
        missing = analysis.method_metrics([], [], [])
        self.assertEqual(missing["retained_case_mass"], 0)
        self.assertIsNone(missing["case_weighted_coverage"])

    def test_single_mode_raw_recovery_reproduces_original_pooled_cutoffs(self):
        rr = [
            row("1", stratum="recovery_single", tick=1),
            row("2", seed="270", stratum="recovery_single", tick=9),
        ]
        original = fit_thresholds(rr)
        modes = analysis.fit_mode_thresholds(rr)
        for score, budget in analysis.product(analysis.SCORES, analysis.BUDGETS):
            a = next(
                t
                for t in original
                if (t["noise"], t["view"], t["score"], t["budget"])
                == ("white", "sim_raw_recovery", score, budget)
            )
            b = next(
                t
                for t in modes
                if (t["noise"], t["view"], t["stratum"], t["score"], t["budget"])
                == ("white", "sim_raw_recovery", "recovery_single", score, budget)
            )
            self.assertEqual(a["rule"], b["rule"])
            if budget != "all":
                self.assertEqual(a["threshold_tick"], b["threshold_tick"])
            self.assertEqual(
                [analysis.decision(r, score, a) for r in rr],
                [analysis.decision(r, score, b) for r in rr],
            )

    def test_summary_preserves_unsupported_modes_and_all_available_still_works(self):
        known = row()
        new = row("2", partition="new", stratum="fusion_three")
        ee = dict(event_rows(known), **{"2": event_rows(new)["1"]})
        new["source_event_ids"] = "2"
        rr = [known, new]
        summaries = analysis.summaries(
            rr, fit_thresholds(rr), analysis.fit_mode_thresholds(rr), ee
        )
        wanted = [
            r
            for r in summaries
            if (
                r["partition"],
                r["noise"],
                r["variant"],
                r["event_kind"],
                r["view"],
                r["stratum"],
                r["score"],
            )
            == ("new", "white", "epoch", "all", new["view"], "all", "contrast")
        ]
        median = next(r for r in wanted if r["budget"] == "q50")
        self.assertEqual(median["conditional_support"], "unsupported")
        self.assertEqual(median["conditional_unsupported_rows"], 1)
        self.assertEqual(median["conditional_unsupported_case_mass"], 1)
        self.assertEqual(median["conditional_unsupported_strata"], "fusion_three")
        self.assertEqual(median["conditional_retained_rows"], "")
        self.assertEqual(median["conditional_only_rows"], "")
        all_row = next(r for r in wanted if r["budget"] == "all")
        self.assertEqual(all_row["conditional_support"], "complete")
        self.assertEqual(
            all_row["pooled_retained_rows"], all_row["conditional_retained_rows"]
        )
        self.assertEqual(all_row["case_weighted_coverage_delta"], 0)

    def test_event_kind_unions_and_empty_structural_groups_stay_complete(self):
        r = dict(row(), source_event_ids="1+2+3")
        ee = event_rows(r, ("received", "received", "missed"))
        summaries = analysis.summaries(
            [r], fit_thresholds([r]), analysis.fit_mode_thresholds([r]), ee
        )
        self.assertEqual(len(summaries), 6912)
        for kind in ["all", "received", "missed"]:
            match = [
                s
                for s in summaries
                if (
                    s["partition"],
                    s["noise"],
                    s["variant"],
                    s["event_kind"],
                    s["view"],
                    s["stratum"],
                    s["score"],
                    s["budget"],
                )
                == (
                    "known",
                    "white",
                    "epoch",
                    kind,
                    r["view"],
                    "all",
                    "contrast",
                    "q50",
                )
            ]
            self.assertEqual(len(match), 1)
            self.assertEqual(match[0]["records"], 1)
        self.assertTrue(
            all(
                s["conditional_support"] == "empty"
                and s["case_weighted_coverage_delta"] == ""
                for s in summaries
                if s["event_kind"] == "precision_only"
            )
        )

    def test_scored_members_assert_identity_seed_membership_labels_and_tick_scale(self):
        r = row()
        m = {
            k: r[k]
            for k in [
                "source_family",
                "source_id",
                *analysis.KEY,
                "partition",
                "noise",
                "view",
                "source_event_ids",
                "a_id",
                "b_id",
                "stratum",
                "available",
                "b_estimator",
                "anchor_mode",
            ]
        }
        ee = event_rows(r)
        self.assertEqual(
            analysis.scored_members([m], [r], ee)[0]["residual_error_m"], 0.2
        )
        for bad in [
            dict(r, a_id="99"),
            dict(r, contrast_tick=99),
            dict(r, residual_outside=False),
        ]:
            with self.assertRaises(AssertionError):
                analysis.scored_members([m], [bad], ee)
        with self.assertRaises(AssertionError):
            analysis.scored_members([dict(m, seed="274")], [dict(r, seed="274")], ee)

    def test_prior_crosscheck_rejects_changed_original_risk(self):
        r = row()
        comparison = {k: r[k] for k in analysis.GROUP if k != "event_kind"}
        comparison.update(
            event_kind="all",
            stratum="filter_single",
            score="contrast",
            budget="all",
            records=1,
            cases=1,
            case_mass=1,
            group_case_mass=1,
            case_mass_share=1,
        )
        comparison.update(
            {
                "pooled_" + k: v
                for k, v in analysis.method_metrics([r], [F(1)], [True]).items()
            }
        )
        comparison = analysis.exported(comparison)
        prior = []
        for target in analysis.TARGETS:
            p = {
                k: comparison[k]
                for k in [
                    *analysis.GROUP,
                    "stratum",
                    "score",
                    "budget",
                    "records",
                    "cases",
                    "case_mass",
                    "group_case_mass",
                    "case_mass_share",
                ]
            }
            p.update(
                target=target,
                retained_rows=1,
                retained_cases=1,
                retained_case_mass=1,
                case_weighted_coverage=1,
                observed_outside_rows=1,
                observed_outside_case_mass=1,
                observed_outside_fraction=1,
                observed_rms_m=0.2,
            )
            prior.append(p)
        self.assertEqual(analysis.check_prior([comparison], prior), 2)
        comparison["pooled_residual_case_weighted_risk"] = 0
        with self.assertRaises(AssertionError):
            analysis.check_prior([comparison], prior)


if __name__ == "__main__":
    unittest.main()
