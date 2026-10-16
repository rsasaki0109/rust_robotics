"""Seed comparison weights, unsupported classes, exact blocks and cohort isolation."""

from fractions import Fraction as F
import unittest

import analyze_seed_conditioned_rankings as analysis
from analyze_estimator_mode_components import vector_terms


def row(
    case="A", seed="277", tick=1, error=0.2, stratum="filter_single", partition="fresh"
):
    terms = vector_terms(
        dict(noise=(0, 0), anchor=(error, 0), drift=(0, 0), transport=(0, 0)), 0.01
    )
    return dict(
        cohort=partition + "-white",
        partition=partition,
        noise="white",
        variant="epoch",
        mode="none",
        seed=seed,
        scenario=case,
        planner="A*",
        available=True,
        view=analysis.VIEW,
        stratum=stratum,
        midpoint=terms,
        residual=terms,
        **{s + "_tick": tick for s in analysis.SCORES},
    )


def decompose(rr, ww=None, label="observed"):
    return analysis.decompose(
        rr,
        [F(1)] * len(rr) if ww is None else ww,
        "residual",
        "residual_sigma",
        label,
        analysis.SEEDS["fresh"],
    )


class SeedConditionedTests(unittest.TestCase):
    def test_high_pooled_auc_can_coexist_with_zero_within_seed_auc(self):
        # High-score seed: nine outside cases below its one inside case.
        # Low-score seed: one outside case below its nine inside cases.
        rr = [row(case=f"A{i}", tick=10) for i in range(9)]
        rr += [row(case="An", tick=11, error=0.05)]
        rr += [row(case="Bp", seed="278", tick=1)]
        rr += [row(case=f"B{i}", seed="278", tick=2, error=0.05) for i in range(9)]
        result, blocks = decompose(rr)
        self.assertEqual(result["auc"], F(81, 100))
        self.assertEqual(result["same_seed_auc"], 0)
        self.assertEqual(result["same_seed_pair_share"], F(18, 100))
        self.assertEqual(result["seeds_with_both_classes"], 2)

    def test_comparison_weighted_within_seed_auc_is_not_mean_seed_auc(self):
        rr = [
            row(case="Ap", tick=9),
            row(case="An", tick=1, error=0.05),
            row(case="Bp", seed="278", tick=1),
            row(case="Bn", seed="278", tick=9, error=0.05),
        ]
        result, _ = decompose(rr, [F(1, 2), F(1, 2), F(1), F(1)])
        # Seed A: AUC=1 with denominator .25. Seed B: AUC=0 with denominator 1.
        self.assertEqual(result["same_seed_auc"], F(1, 5))
        self.assertNotEqual(result["same_seed_auc"], F(1, 2))
        self.assertEqual(result["same_seed_pair_share"], F(5, 9))

    def test_between_seed_only_auc_keeps_within_seed_undefined(self):
        result, blocks = decompose(
            [row(tick=9), row(case="B", seed="278", tick=1, error=0.05)]
        )
        self.assertEqual(result["auc"], 1)
        self.assertEqual(result["same_seed_pair_share"], 0)
        self.assertIsNone(result["same_seed_auc"])
        self.assertIsNone(result["pooled_minus_same_seed_auc"])
        self.assertEqual(result["seeds_with_both_classes"], 0)
        self.assertEqual(len(blocks), 9)
        self.assertEqual(sum(b["denominator"] > 0 for b in blocks), 1)

    def test_all_blocks_stay_explicit_for_absent_seed_and_one_class(self):
        result, blocks = decompose(
            [row(error=0.05), row(case="B", seed="278", error=0.05)]
        )
        self.assertIsNone(result["auc"])
        self.assertIsNone(result["same_seed_pair_share"])
        self.assertIsNone(result["between_seed_pair_share"])
        self.assertEqual(len(blocks), 9)
        self.assertTrue(all(b["auc"] is None and b["denominator"] == 0 for b in blocks))
        self.assertTrue(all(b["denominator_share"] is None for b in blocks))

    def test_one_represented_seed_makes_full_and_within_auc_identical(self):
        result, _ = decompose([row(tick=9), row(case="B", tick=1, error=0.05)])
        self.assertEqual(result["auc"], result["same_seed_auc"])
        self.assertEqual(result["same_seed_pair_share"], 1)
        self.assertEqual(result["between_seed_pair_share"], 0)
        self.assertIsNone(result["between_seed_auc"])

    def test_equal_ticks_receive_half_credit_without_reversal(self):
        result, blocks = decompose([row(tick=9), row(case="B", tick=9, error=0.05)])
        self.assertEqual(result["auc"], F(1, 2))
        self.assertEqual(result["tied_pair_mass"], 1)
        self.assertEqual(result["concordant_pair_mass"], 0)
        self.assertEqual(result["same_seed_auc"], F(1, 2))

    def test_block_mass_and_auc_contributions_close_exactly(self):
        rr = [
            row(tick=2),
            row(case="B", tick=1, error=0.05),
            row(case="C", seed="278", tick=1),
            row(case="D", seed="278", tick=2, error=0.05),
        ]
        result, blocks = decompose(rr, [F(1, 3), F(1, 7), F(1, 5), F(1, 11)])
        for field in analysis.PAIR_FIELDS:
            self.assertEqual(sum(b[field] for b in blocks), result[field])
        self.assertEqual(sum(b["denominator_share"] for b in blocks), 1)
        self.assertEqual(
            sum(b["numerator_contribution"] for b in blocks), result["auc"]
        )
        self.assertEqual(
            result["same_seed_numerator"] + result["between_seed_numerator"],
            result["numerator"],
        )
        self.assertEqual(
            result["same_seed_denominator"] + result["between_seed_denominator"],
            result["denominator"],
        )

    def test_empty_full_grid_preserves_missing_historical_fallback(self):
        components, seed_support, ranks, blocks = analysis.summaries([])
        self.assertEqual(
            tuple(map(len, [components, seed_support, ranks, blocks])),
            (120, 360, 1080, 9720),
        )
        self.assertTrue(
            all(
                r["records"] == 0 and r["auc"] == "" and r["same_seed_auc"] == ""
                for r in ranks
            )
        )
        self.assertTrue(
            all(r["denominator"] == 0 and r["denominator_share"] == "" for r in blocks)
        )

    def test_mode_conditioning_keeps_original_parent_case_mass(self):
        rr = [
            row(tick=9),
            row(stratum="fusion_three"),
            row(case="B", seed="278", tick=1, error=0.05),
        ]
        components, seed_support, ranks, _ = analysis.summaries(rr)
        pick = dict(
            partition="fresh",
            noise="white",
            variant="epoch",
            stratum="filter_single",
            target="residual",
        )
        comp = next(r for r in components if all(r[k] == v for k, v in pick.items()))
        self.assertEqual(comp["case_mass"], 1.5)
        self.assertEqual(comp["group_case_mass"], 2)
        self.assertEqual(comp["case_mass_share"], 0.75)
        seeds = {
            r["seed"]: r
            for r in seed_support
            if all(r[k] == v for k, v in pick.items())
        }
        self.assertEqual(seeds["277"]["case_mass"], 0.5)
        self.assertEqual(seeds["278"]["case_mass"], 1)
        self.assertEqual(seeds["279"]["case_mass"], 0)
        rank = next(
            r
            for r in ranks
            if all(r[k] == v for k, v in pick.items())
            and r["score"] == "residual_sigma"
            and r["label"] == "observed"
        )
        self.assertEqual(rank["denominator"], 0.5)
        self.assertEqual(rank["auc"], 1)
        self.assertEqual(rank["same_seed_auc"], "")

    def test_cohorts_reject_other_groups_seed_ids(self):
        with self.assertRaises(AssertionError):
            analysis.summaries([row(partition="known", seed="277")])
        with self.assertRaises(AssertionError):
            decompose([row(seed="271")])

    def test_unsupported_label_is_not_imputed_and_boundary_stays_closed(self):
        rr = [row(error=0.1), row(case="B", tick=9, error=0.05)]
        result, _ = decompose(rr)
        self.assertEqual(result["outside_rows"], 0)
        self.assertIsNone(result["auc"])
        u, _ = decompose(rr, label="unmodeled")
        self.assertIsNone(u["auc"])
        self.assertEqual(u["outside_case_mass"], 0)

    def test_row_permutation_preserves_exact_comparison_masses(self):
        rr = [
            row(tick=8),
            row(case="B", tick=7, error=0.05),
            row(case="C", seed="278", tick=4),
        ]
        ww = [F(1, 3), F(1, 5), F(1, 7)]
        self.assertEqual(decompose(rr, ww), decompose(rr[::-1], ww[::-1]))


if __name__ == "__main__":
    unittest.main()
