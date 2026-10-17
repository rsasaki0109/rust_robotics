"""Planned seed coverage, exact subset weights and direct pair recomputation."""

from copy import deepcopy
from fractions import Fraction as F
import unittest

import analyze_event_seed_influence as analysis
from test_event_score_ties import fixture
from test_event_unit_influence import cases as case_fixture
from test_planner_window_sensitivity import account


def cases():
    rows, _ = case_fixture()
    seeds = dict(zip(["A*", "JPS", "Theta*"], ["277", "278", "279"]))
    for r in rows:
        r["seed"] = seeds[r["planner"]]
    _, _, lookup = analysis.window.audit_units(account(rows), rows)
    return rows, lookup


def run(rows, lookup, label="observed"):
    return analysis.influence(
        rows, lookup, "fresh", "residual", "residual_sigma", label
    )


def chosen(rows, seed):
    return next(r for r in rows if r["unit_id"] == seed)


class EventSeedInfluenceTests(unittest.TestCase):
    def test_hand_calculated_seed_deletion_and_seed_only_use_original_weights(self):
        rr, lookup = cases()
        base, effects, summary = run(rr, lookup)[0]
        self.assertEqual(base["denominator"], F(23, 36))
        self.assertEqual(base["auc"], F(1, 2))
        r = chosen(effects, "278")
        self.assertEqual(r["auc"], F(5, 28))
        self.assertEqual(r["seed_auc"], 1)
        self.assertEqual(r["removed_pair_share"], F(9, 23))
        self.assertEqual(r["seed_denominator"], F(1, 4))
        self.assertEqual(r["denominator"], F(14, 36))
        self.assertEqual(summary["auc_min"], F(5, 28))
        self.assertEqual(summary["auc_max"], F(23, 28))
        self.assertEqual(summary["auc_seed_only_min"], 0)
        self.assertEqual(summary["auc_seed_only_max"], 1)

    def test_seed_groups_multiple_cases_and_families(self):
        rr, _ = cases()
        for r in rr:
            if r["seed"] == "279":
                r["seed"] = "278"
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        audit = analysis.seed_audits(rr, lookup, "fresh")
        u = chosen(audit, "278")
        self.assertEqual(u["cases"], 2)
        self.assertEqual(len(u["family_ids"].split("+")), 2)
        _, effects, summary = run(rr, lookup)[0]
        r = chosen(effects, "278")
        self.assertEqual(r["removed_cases"], 2)
        self.assertEqual(r["seed_pair_count"], 2)
        self.assertEqual(r["seed_auc"], F(1, 2))
        self.assertEqual(r["cases"], 1)
        self.assertEqual(summary["represented_units"], 2)

    def test_all_metrics_scopes_targets_scores_labels_match_direct_subsets(self):
        rr, lookup = cases()
        for target in analysis.previous.TARGETS:
            for score in analysis.previous.SCORES:
                for label in analysis.previous.historical.LABELS:
                    result = analysis.influence(
                        rr, lookup, "fresh", target, score, label
                    )
                    for seed in analysis.previous.SEEDS["fresh"]:
                        for retain in [False, True]:
                            subset = [r for r in rr if (r["seed"] == seed) == retain]
                            weights = [
                                lookup[analysis.previous.identity(r)]["case_row_weight"]
                                for r in subset
                            ]
                            direct = analysis.ties.decomposition(
                                subset, weights, lookup, target, score, label
                            )[0]
                            for i, expected in enumerate(direct):
                                actual = chosen(result[i][1], seed)
                                prefix = "seed_" if retain else ""
                                for field in analysis.SEED_FIELDS:
                                    self.assertEqual(
                                        actual[prefix + field],
                                        expected[field],
                                        (seed, retain, i, field),
                                    )

    def test_baseline_preserves_every_previous_precision_field(self):
        rr, lookup = cases()
        weights = [lookup[analysis.previous.identity(r)]["case_row_weight"] for r in rr]
        expected = analysis.ties.decomposition(
            rr, weights, lookup, "residual", "residual_sigma", "observed"
        )[0]
        self.assertEqual([v[0] for v in run(rr, lookup)], expected)

    def test_absent_seed_is_deletion_noop_and_seed_only_undefined(self):
        rr, lookup = cases()
        rr = [r for r in rr if r["seed"] != "279"]
        base, effects, summary = run(rr, lookup)[0]
        r = chosen(effects, "279")
        self.assertEqual(r["removed_records"], 0)
        self.assertEqual(r["removed_pair_share"], 0)
        self.assertEqual(r["auc"], base["auc"])
        self.assertEqual(r["delta_auc"], 0)
        self.assertIsNone(r["seed_auc"])
        self.assertEqual(summary["units"], 3)
        self.assertEqual(summary["auc_seed_only_undefined"], 1)

    def test_pair_free_seed_keeps_available_records_but_no_pairs(self):
        rr, _ = cases()
        rr = [r for r in rr if r["seed"] != "279"]
        spare, _ = fixture([(0, 1, 1e-8, 0.05)], planner="Theta*")
        spare[0]["seed"] = "279"
        rr += spare
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        base, effects, summary = run(rr, lookup)[0]
        r = chosen(effects, "279")
        self.assertEqual(r["removed_records"], 1)
        self.assertEqual(r["removed_pair_count"], 0)
        self.assertEqual(r["auc"], base["auc"])
        self.assertIsNone(r["seed_auc"])
        self.assertEqual(summary["represented_units"], 3)
        self.assertEqual(summary["pair_supported_units"], 2)

    def test_all_empty_groups_keep_three_audits_and_effects_per_scope(self):
        rr, lookup = cases()
        audit = analysis.seed_audits([], lookup, "fresh")
        self.assertEqual(len(audit), 3)
        self.assertTrue(all(u["case_ids"] == u["family_ids"] == "" for u in audit))
        for base, effects, summary in run([], lookup):
            self.assertEqual(len(effects), 3)
            self.assertIsNone(base["auc"])
            self.assertEqual(summary["represented_units"], 0)
            self.assertEqual(summary["auc_undefined_deletions"], 3)
            self.assertEqual(summary["auc_seed_only_undefined"], 3)
            self.assertIsNone(summary["max_removed_pair_share"])

    def test_single_class_and_cross_seed_classes_create_no_same_case_pairs(self):
        a, _ = fixture([(0, 2, 2e-8, 0.2)])
        b, _ = fixture([(0, 1, 1e-8, 0.05)])
        b[0]["seed"] = "278"
        rr = a + b
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        for base, effects, summary in run(rr, lookup):
            self.assertEqual(base["pair_count"], 0)
            self.assertTrue(all(r["seed_auc"] is None for r in effects))
            self.assertEqual(summary["auc_strict_side_flips"], 0)

    def test_all_ties_preserve_half_and_undefined_unequal_concordance(self):
        rr, lookup = fixture([(0, 2, 2e-8, 0.2), (0.02, 2, 2e-8, 0.05)])
        _, effects, summary = run(rr, lookup)[0]
        self.assertEqual(chosen(effects, "277")["seed_auc"], F(1, 2))
        self.assertIsNone(chosen(effects, "277")["seed_tick_unequal_concordance"])
        self.assertEqual(summary["tick_unequal_concordance_seed_only_undefined"], 3)

    def test_native_tie_resolution_keeps_both_score_orders(self):
        rr, lookup = fixture([(0, 2, 2.1e-8, 0.2), (0.02, 2, 2e-8, 0.05)])
        _, effects, summary = run(rr, lookup)[0]
        r = chosen(effects, "277")
        self.assertEqual(r["seed_auc"], F(1, 2))
        self.assertEqual(r["seed_native_auc"], 1)
        self.assertIsNone(r["seed_tick_unequal_concordance"])
        self.assertEqual(r["seed_native_unequal_concordance"], 1)

    def test_only_support_seed_removal_stays_undefined_not_zero_or_flip(self):
        rr, lookup = fixture([(0, 2, 2e-8, 0.2), (0.02, 1, 1e-8, 0.05)])
        _, effects, summary = run(rr, lookup)[0]
        r = chosen(effects, "277")
        self.assertIsNone(r["auc"])
        self.assertIsNone(r["delta_auc"])
        self.assertEqual(r["removed_pair_share"], 1)
        self.assertEqual(summary["auc_undefined_deletions"], 1)
        self.assertEqual(summary["auc_strict_side_flips"], 0)

    def test_exact_half_departures_are_not_strict_side_flips(self):
        rr, lookup = cases()
        _, _, summary = run(rr, lookup)[0]
        self.assertEqual(
            (
                summary["auc_below_half"],
                summary["auc_at_half"],
                summary["auc_above_half"],
            ),
            (1, 1, 1),
        )
        self.assertEqual(summary["auc_strict_side_flips"], 0)

    def test_strict_flip_needs_two_defined_values_opposite_half(self):
        a, _ = fixture([(0, 2, 2e-8, 0.2), (0.02, 1, 1e-8, 0.05)])
        b, _ = fixture([(0, 1, 1e-8, 0.2), (0.02, 1, 1e-8, 0.2), (0.04, 2, 2e-8, 0.05)])
        for r in b:
            r["seed"] = "278"
        rr = a + b
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        base, _, summary = run(rr, lookup)[0]
        self.assertEqual(base["auc"], F(9, 17))
        self.assertEqual(summary["auc_strict_side_flips"], 1)

    def test_stratum_selection_keeps_full_parent_case_weights(self):
        rr, lookup = cases()
        selected = [r for i, r in enumerate(rr) if i != 0]
        u = chosen(analysis.seed_audits(selected, lookup, "fresh"), "277")
        self.assertEqual(u["case_mass"], F(5, 6))
        _, effects, _ = run(selected, lookup)[0]
        self.assertEqual(chosen(effects, "277")["removed_case_mass"], F(5, 6))

    def test_fixed_family_eligibility_retains_all_selected_planners_of_other_seed(self):
        rr, _ = cases()
        for r in rr:
            r["seed"] = "277"
            r["scenario"] = "fixture"
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        _, effects, _ = run(rr, lookup)[0]
        self.assertEqual(chosen(effects, "278")["cases"], 3)
        self.assertEqual(chosen(effects, "277")["removed_cases"], 3)
        self.assertEqual(chosen(effects, "277")["seed_cases_with_pairs"], 3)

    def test_same_case_different_events_never_create_pairs(self):
        rr, lookup = fixture([(0, 2, 2e-8, 0.2), (0.02, 1, 1e-8, 0.05)])
        rr[1]["_event_id"] = "other"
        self.assertEqual(run(rr, lookup)[0][0]["pair_count"], 0)

    def test_seed_only_includes_exact_time_boundaries_without_rounding_leaks(self):
        spec = [(0, 2, 2e-8, 0.2)] + [
            (t, 1, 1e-8, 0.05) for t in [0.02, 0.020001, 0.1, 0.100001, 0.5, 0.500001]
        ]
        rr, lookup = fixture(spec)
        result = run(rr, lookup)
        self.assertEqual(
            [chosen(r[1], "277")["seed_pair_count"] for r in result], [6, 5, 3, 1]
        )
        self.assertEqual([r[0]["pair_count"] for r in result], [6, 5, 3, 1])

    def test_rejects_unknown_duplicate_and_malformed_planned_or_observed_seeds(self):
        for partition, seeds in [
            ("unknown", None),
            ("fresh", ["277", "277", "279"]),
            ("fresh", ["277", "278", "280"]),
            ("fresh", ["0277", "278", "279"]),
            ("fresh", [277, "278", "279"]),
            ("fresh", ["277", "278"]),
        ]:
            with self.assertRaises(AssertionError):
                analysis.planned_seeds(partition, seeds)
        rr, lookup = cases()
        rr[0]["seed"] = "280"
        with self.assertRaises(AssertionError):
            analysis.seed_audits(rr, lookup, "fresh")

    def test_rows_and_lookup_are_not_mutated(self):
        rr, lookup = cases()
        before = deepcopy((rr, lookup))
        run(rr, lookup)
        self.assertEqual((rr, lookup), before)


if __name__ == "__main__":
    unittest.main()
