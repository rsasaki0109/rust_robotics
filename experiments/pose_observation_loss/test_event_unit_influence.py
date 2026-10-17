"""Exhaustive deletions, fixed populations, exact support loss and side changes."""

from copy import deepcopy
from fractions import Fraction as F
import unittest

import analyze_event_unit_influence as analysis
from test_event_score_ties import fixture
from test_planner_window_sensitivity import account


def cases():
    a, _ = fixture()
    b, _ = fixture([(0, 2, 2e-8, 0.2), (0.02, 1, 1e-8, 0.05)], planner="JPS")
    c, _ = fixture([(0, 1, 1e-8, 0.2), (0.02, 2, 2e-8, 0.05)], planner="Theta*")
    for r in c:
        r["scenario"] = "second"
    rows = [*a, *b, *c]
    _, _, lookup = analysis.window.audit_units(account(rows), rows)
    return rows, lookup


def case_id(lookup, planner):
    return next(
        info["case_id"] for info in lookup.values() if info["planner"] == planner
    )


def chosen(result, kind, uid):
    return next(r for r in result if r["unit_kind"] == kind and r["unit_id"] == uid)


class EventUnitInfluenceTests(unittest.TestCase):
    def test_hand_calculated_case_deletions_keep_original_unequal_weights(self):
        rr, lookup = cases()
        base, deletions, summary = analysis.influence(
            rr, lookup, "residual", "residual_sigma", "observed"
        )[0]
        self.assertEqual(base["denominator"], F(23, 36))
        self.assertEqual(base["auc"], F(1, 2))
        expected = {"A*": F(1, 2), "JPS": F(5, 28), "Theta*": F(23, 28)}
        for planner, value in expected.items():
            r = chosen(deletions, "case", case_id(lookup, planner))
            self.assertEqual(r["auc"], value)
        r = chosen(deletions, "case", case_id(lookup, "JPS"))
        self.assertEqual(r["removed_pair_share"], F(9, 23))
        self.assertEqual(r["tick_unequal_concordance"], F(1, 11))
        self.assertEqual(r["removed_case_mass"], 1)
        self.assertEqual(summary["case"]["auc_min"], F(5, 28))
        self.assertEqual(summary["case"]["auc_max"], F(23, 28))

    def test_family_removes_all_its_selected_planner_cases(self):
        rr, lookup = cases()
        base, deletions, summary = analysis.influence(
            rr, lookup, "residual", "residual_sigma", "observed"
        )[0]
        fid = lookup[analysis.previous.identity(rr[0])]["family_id"]
        r = chosen(deletions, "family", fid)
        self.assertEqual(r["removed_cases"], 2)
        self.assertEqual(r["removed_records"], 8)
        self.assertEqual(r["removed_pair_share"], F(14, 23))
        self.assertEqual(r["cases"], 1)
        self.assertEqual(r["auc"], 0)
        self.assertEqual(summary["family"]["max_removed_pair_share"], F(14, 23))

    def test_every_metric_and_scope_matches_direct_remaining_state_recomputation(self):
        rr, lookup = cases()
        units, _ = analysis.unit_audits(rr, lookup, {})
        results = analysis.influence(
            rr, lookup, "residual", "residual_sigma", "observed", units
        )
        for unit in units:
            ids = set(unit["case_ids"].split("+"))
            remaining = [
                r
                for r in rr
                if lookup[analysis.previous.identity(r)]["case_id"] not in ids
            ]
            ww = [
                lookup[analysis.previous.identity(r)]["case_row_weight"]
                for r in remaining
            ]
            direct = analysis.prior.decomposition(
                remaining, ww, lookup, "residual", "residual_sigma", "observed"
            )[0]
            for i, expected in enumerate(direct):
                actual = chosen(results[i][1], unit["unit_kind"], unit["unit_id"])
                for field in [
                    *analysis.METRICS,
                    "denominator",
                    "numerator",
                    "native_numerator",
                    *analysis.COUNTS[:9],
                ]:
                    self.assertEqual(
                        actual[field], expected[field], (unit["unit_id"], i, field)
                    )

    def test_additive_baseline_reconstructs_every_previous_precision_field(self):
        rr, lookup = cases()
        ww = [lookup[analysis.previous.identity(r)]["case_row_weight"] for r in rr]
        before = analysis.prior.decomposition(
            rr, ww, lookup, "residual", "residual_sigma", "observed"
        )[0]
        after = analysis.influence(rr, lookup, "residual", "residual_sigma", "observed")
        self.assertEqual([r[0] for r in after], before)

    def test_fixed_common_family_population_keeps_surviving_planners(self):
        rr, lookup = cases()
        for r in rr:
            r["scenario"] = "fixture"
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        units, _ = analysis.unit_audits(rr, lookup, {"population": "common_family"})
        _, deletions, _ = analysis.influence(
            rr, lookup, "residual", "residual_sigma", "observed", units
        )[0]
        for r in deletions:
            if r["unit_kind"] == "case":
                self.assertEqual(r["cases"], 2)
                self.assertGreater(r["pair_count"], 0)

    def test_pair_free_selected_case_is_a_retained_noop_for_metrics(self):
        rr, lookup = cases()
        spare, _ = fixture([(0, 1, 1e-8, 0.05)], planner="JPS")
        spare[0]["scenario"] = "pair-free"
        rr += spare
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        uid = lookup[analysis.previous.identity(spare[0])]["case_id"]
        base, deletions, summary = analysis.influence(
            rr, lookup, "residual", "residual_sigma", "observed"
        )[0]
        r = chosen(deletions, "case", uid)
        self.assertEqual(r["removed_pair_count"], 0)
        self.assertEqual(r["removed_records"], 1)
        self.assertEqual(r["removed_pair_share"], 0)
        self.assertEqual(r["delta_auc"], 0)
        self.assertEqual(summary["case"]["units"], 4)
        self.assertEqual(summary["case"]["pair_supported_units"], 3)

    def test_only_support_removal_keeps_undefined_auc_and_change(self):
        rr, lookup = fixture([(0, 2, 2e-8, 0.2), (0.02, 1, 1e-8, 0.05)])
        base, deletions, summary = analysis.influence(
            rr, lookup, "residual", "residual_sigma", "observed"
        )[0]
        self.assertEqual(base["auc"], 1)
        for r in deletions:
            self.assertIsNone(r["auc"])
            self.assertIsNone(r["delta_auc"])
            self.assertEqual(r["denominator"], 0)
            self.assertEqual(r["removed_pair_share"], 1)
        self.assertEqual(summary["case"]["auc_undefined_deletions"], 1)
        self.assertIsNone(summary["case"]["auc_min"])
        self.assertEqual(summary["case"]["auc_strict_side_flips"], 0)

    def test_all_tie_baseline_does_not_invent_unequal_concordance(self):
        rr, lookup = fixture([(0, 2, 2e-8, 0.2), (0.02, 2, 2e-8, 0.05)])
        base, deletions, summary = analysis.influence(
            rr, lookup, "residual", "residual_sigma", "observed"
        )[0]
        self.assertEqual(base["auc"], F(1, 2))
        self.assertIsNone(base["tick_unequal_concordance"])
        self.assertIsNone(summary["case"]["max_removed_tick_unequal_share"])
        self.assertEqual(summary["case"]["tick_unequal_supported_units"], 0)

    def test_empty_and_single_class_groups_keep_explicit_summaries(self):
        rr, lookup = cases()
        for rows, label in [([], "observed"), (rr, "unmodeled")]:
            for base, deletions, summary in analysis.influence(
                rows, lookup, "residual", "residual_sigma", label
            ):
                self.assertIsNone(base["auc"])
                for kind in analysis.KINDS:
                    self.assertEqual(summary[kind]["auc_defined_deletions"], 0)
                    self.assertIsNone(summary[kind]["auc_min"])
                    self.assertEqual(summary[kind]["auc_strict_side_flips"], 0)
                if not rows:
                    self.assertEqual(deletions, [])

    def test_exact_half_departures_are_not_strict_side_flips(self):
        rr, lookup = cases()
        _, _, summary = analysis.influence(
            rr, lookup, "residual", "residual_sigma", "observed"
        )[0]
        s = summary["case"]
        self.assertEqual(
            (s["auc_below_half"], s["auc_at_half"], s["auc_above_half"]), (1, 1, 1)
        )
        self.assertEqual(s["auc_strict_side_flips"], 0)

    def test_strict_side_flip_requires_both_defined_opposite_sides(self):
        a, _ = fixture([(0, 2, 2e-8, 0.2), (0.02, 1, 1e-8, 0.05)])
        b, _ = fixture(
            [(0, 1, 1e-8, 0.2), (0.02, 1, 1e-8, 0.2), (0.04, 2, 2e-8, 0.05)],
            planner="JPS",
        )
        rr = [*a, *b]
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        base, _, summary = analysis.influence(
            rr, lookup, "residual", "residual_sigma", "observed"
        )[0]
        self.assertEqual(base["auc"], F(9, 17))
        self.assertEqual(summary["case"]["auc_strict_side_flips"], 1)
        self.assertEqual(summary["case"]["auc_min"], 0)
        self.assertEqual(summary["case"]["auc_max"], 1)

    def test_stratum_selection_keeps_parent_case_row_weights(self):
        rr, lookup = cases()
        rr[0] = dict(rr[0], stratum="fusion_three")
        selected = [r for r in rr if r["stratum"] == "filter_single"]
        units, _ = analysis.unit_audits(selected, lookup, {})
        u = chosen(units, "case", case_id(lookup, "A*"))
        self.assertEqual(u["records"], 5)
        self.assertEqual(u["case_mass"], F(5, 6))
        _, deletions, _ = analysis.influence(
            selected, lookup, "residual", "residual_sigma", "observed", units
        )[0]
        r = chosen(deletions, "case", u["unit_id"])
        self.assertEqual(r["removed_case_mass"], F(5, 6))

    def test_same_tag_in_another_case_does_not_create_cross_case_comparisons(self):
        rr, lookup = cases()
        original = analysis.influence(
            rr, lookup, "residual", "residual_sigma", "observed"
        )
        renamed = [dict(r, _event_id="99") if r["planner"] == "JPS" else r for r in rr]
        self.assertEqual(
            original,
            analysis.influence(
                renamed, lookup, "residual", "residual_sigma", "observed"
            ),
        )

    def test_population_and_rows_are_not_mutated(self):
        rr, lookup = cases()
        original = deepcopy(rr)
        analysis.influence(rr, lookup, "residual", "residual_sigma", "observed")
        self.assertEqual(rr, original)

    def test_subtraction_rejects_an_impossible_removed_subset(self):
        rr, lookup = cases()
        base, _, _ = analysis.influence(
            rr, lookup, "residual", "residual_sigma", "observed"
        )[0]
        removed = dict(base, denominator=base["denominator"] + 1)
        with self.assertRaises(AssertionError):
            analysis.subtract(base, removed)


if __name__ == "__main__":
    unittest.main()
