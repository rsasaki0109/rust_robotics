"""Exact pair contributions, frozen parent weights and missing support."""

from fractions import Fraction as F
import unittest

import analyze_fresh_seed_case_influence as analysis
from analyze_estimator_mode_components import vector_terms


def row(case="A", seed="277", tick=1, error=0.2, mode="filter_single"):
    terms = vector_terms(
        dict(noise=(0, 0), anchor=(error, 0), drift=(0, 0), transport=(0, 0)), 0.01
    )
    return dict(
        case_id=case,
        cohort="fresh-white",
        noise="white",
        variant="epoch",
        mode="none",
        seed=seed,
        scenario=case,
        planner="A*",
        available=True,
        view=analysis.VIEW,
        stratum=mode,
        midpoint=terms,
        residual=terms,
        **{s + "_tick": tick for s in analysis.previous.SCORES}
    )


class CaseInfluenceTests(unittest.TestCase):
    def test_ordered_pair_direction_and_half_credit_for_ties(self):
        # Positive scores 1 and 3 against negative 2 and 3: one win and one tie.
        p = analysis.pair_masses({2: F(1), 3: F(1)}, {1: F(1), 3: F(1)})
        self.assertEqual(p["concordant_pair_mass"], 1)
        self.assertEqual(p["tied_pair_mass"], 1)
        self.assertEqual(p["discordant_pair_mass"], 2)
        self.assertEqual(p["auc"], F(3, 8))

    def test_pair_masses_respect_unequal_weights(self):
        # The upper positive has mass 3/4, the lower positive only 1/4.
        p = analysis.pair_masses({2: F(1, 3)}, {1: F(1, 4), 3: F(3, 4)})
        self.assertEqual(p["numerator"], F(1, 4))
        self.assertEqual(p["denominator"], F(1, 3))
        self.assertEqual(p["auc"], F(3, 4))

    def test_missing_class_and_empty_group_have_no_auc(self):
        for negative, positive in [({}, {}), ({1: F(2)}, {}), ({}, {9: F(3)})]:
            p = analysis.pair_masses(negative, positive)
            self.assertEqual(p["denominator"], 0)
            self.assertEqual(p["numerator"], 0)
            self.assertIsNone(p["auc"])

    def test_mixed_mode_subsets_keep_parent_weights(self):
        parent = [row(), row(mode="fusion_three"), row(case="B", seed="278")]
        weights = analysis.case_weights(parent)
        filtered = [
            (r, w) for r, w in zip(parent, weights) if r["stratum"] == "filter_single"
        ]
        rr, ww = [r for r, w in filtered], [w for r, w in filtered]
        scopes = {(s, n): (r, w) for s, n, r, w in analysis.scopes(rr, ww)}
        self.assertEqual(scopes["all", "all"][1], [F(1, 2), F(1)])
        self.assertEqual(scopes["only_case", "A"][1], [F(1, 2)])
        self.assertEqual(scopes["omit_case", "B"][1], [F(1, 2)])
        self.assertEqual(scopes["only_seed", "277"][1], [F(1, 2)])
        self.assertEqual(scopes["omit_seed", "278"][1], [F(1, 2)])

    def test_all_three_seeds_remain_explicit_when_absent(self):
        rr = [row()]
        scopes = {(s, n): (r, w) for s, n, r, w in analysis.scopes(rr, [F(1)])}
        self.assertEqual(scopes["only_seed", "279"], ([], []))
        self.assertEqual(scopes["omit_seed", "279"], (rr, [F(1)]))
        self.assertEqual(scopes["omit_seed", "277"], ([], []))
        self.assertEqual(scopes["omit_case", "A"], ([], []))

    def test_case_blocks_sum_pair_masses_and_retain_zero_support(self):
        rr = [
            row(tick=1),
            row(tick=3, error=0.05),
            row(case="B", tick=2),
            row(case="B", tick=2, error=0.05),
        ]
        weights = [F(1, 2)] * 4
        blocks = analysis.pair_blocks(
            rr, weights, "residual", "residual_sigma", "observed"
        )
        whole = analysis.pair_masses(
            *analysis.weighted_histogram(
                rr, weights, "residual", "residual_sigma", "observed"
            )
        )
        self.assertEqual(len(blocks), 4)
        for field in [
            "numerator",
            "denominator",
            "concordant_pair_mass",
            "tied_pair_mass",
            "discordant_pair_mass",
        ]:
            self.assertEqual(sum(p[field] for p in blocks), whole[field])
        self.assertEqual(whole["auc"], F(1, 8))
        # U has one class, but all four case-pair cells remain explicit.
        empty = analysis.pair_blocks(
            rr, weights, "residual", "residual_sigma", "unmodeled"
        )
        self.assertEqual(len(empty), 4)
        self.assertTrue(all(p["auc"] is None for p in empty))

    def test_same_case_and_between_case_auc_are_not_averaged(self):
        rr = [row(case="A", tick=9), row(case="B", tick=1, error=0.05)]
        blocks = analysis.pair_blocks(
            rr, [F(1), F(1)], "residual", "residual_sigma", "observed"
        )
        self.assertTrue(all(p["auc"] is None for p in blocks if p["same_case"]))
        numerator = sum(p["numerator"] for p in blocks)
        denominator = sum(p["denominator"] for p in blocks)
        self.assertEqual(numerator / denominator, 1)

    def test_complete_grid_and_single_case_omission_stay_undefined(self):
        cases, components, rankings, blocks = analysis.analyze([row()])
        self.assertEqual(cases[0]["inherited_row_weight"], 1)
        self.assertEqual(sum(r["scope"] == "all" for r in components), 40)
        self.assertEqual(sum(r["scope"] == "all" for r in rankings), 360)
        kept = [r for r in rankings if r["scope"] == "omit_case"]
        self.assertTrue(kept)
        self.assertTrue(
            all(
                r["records"] == 0 and r["auc"] == "" and r["auc_difference"] == ""
                for r in kept
            )
        )
        self.assertTrue(
            all(r["auc"] == "" for r in rankings if r["label"] == "unmodeled")
        )

    def test_case_ids_use_full_case_key_and_are_order_stable(self):
        a, b = row(case="A"), row(case="B")
        first = analysis.analyze([a, b])[0]
        second = analysis.analyze([b, a])[0]
        self.assertEqual(first, second)
        self.assertEqual([r["case_id"] for r in first], ["C001", "C002"])
        self.assertEqual([r["scenario"] for r in first], ["A", "B"])

    def test_defined_auc_and_omission_difference_export_as_numbers(self):
        rr = [row(tick=3), row(tick=1, error=0.05), row(case="B", tick=1)]
        rankings = analysis.analyze(rr)[2]
        values = {
            (r["scope"], r["selection_id"]): r
            for r in rankings
            if r["noise"] == "white"
            and r["variant"] == "epoch"
            and r["stratum"] == "all"
            and r["target"] == "residual"
            and r["score"] == "residual_sigma"
            and r["label"] == "observed"
        }
        self.assertAlmostEqual(values["all", "all"]["auc"], 2 / 3)
        self.assertAlmostEqual(values["omit_case", "C002"]["auc"], 1)
        self.assertAlmostEqual(values["omit_case", "C002"]["auc_difference"], 1 / 3)
        self.assertEqual(values["omit_case", "C001"]["auc"], "")

    def test_boundary_trace_keeps_closed_disk_and_signed_cancellation(self):
        r = row(error=0.1)
        # Saved E=.10 is inside, P=.11 is outside, U=-.01 cancels P.
        terms = vector_terms(
            dict(noise=(0.11, 0), anchor=(0, 0), drift=(-0.01, 0), transport=(0, 0)),
            0.01,
            observed=[0.1, 0],
        )
        r.update(
            source_id="1",
            state_id="7",
            time_s="150",
            truth_time_s="149.98",
            source_event_ids="1+2",
            a_estimator="recovery_mean",
            b_estimator="motion_filter",
            anchor_mode="shared_single",
            midpoint=terms,
            residual=terms,
        )
        cases = analysis.analyze([r])[0]
        p = {
            "1": {
                t + "_" + axis: value
                for t in ["midpoint", "residual"]
                for axis, value in [("x", 0.1), ("y", 0)]
            }
        }
        trace = analysis.boundary_trace([r], cases, p)
        self.assertEqual(len(trace), 2)
        for t in trace:
            self.assertFalse(t["observed_outside"])
            self.assertTrue(t["modeled_outside"])
            self.assertEqual(t["observed_boundary_margin_m"], 0)
            self.assertEqual(t["cell"], "modeled_outside_observed_inside")
            self.assertLess(t["modeled_unmodeled_cross_m2"], 0)
            self.assertEqual(t["source_event_ids"], "1+2")


if __name__ == "__main__":
    unittest.main()
