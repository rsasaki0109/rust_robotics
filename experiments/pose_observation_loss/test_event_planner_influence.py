"""Planner coverage, exact subset weights and direct pair recomputation."""

from copy import deepcopy
from fractions import Fraction as F
import unittest

import analyze_event_planner_influence as analysis
from test_event_score_ties import fixture as score_fixture
from test_event_unit_influence import cases as case_fixture
from test_planner_window_sensitivity import account


def cases():
    rows, _ = case_fixture()
    for r in rows:
        r["scenario"] = {
            "A*": "dense_50x50",
            "JPS": "maze_50x50",
            "Theta*": "open_50x50",
        }[r["planner"]]
        r["seed"] = {"A*": "277", "JPS": "278", "Theta*": "279"}[r["planner"]]
    _, _, lookup = analysis.window.audit_units(account(rows), rows)
    return rows, lookup


def fixture(spec=None, planner="A*"):
    rows, _ = score_fixture(spec, planner)
    for r in rows:
        r["scenario"] = "dense_50x50"
    _, _, lookup = analysis.window.audit_units(account(rows), rows)
    return rows, lookup


def run(rows, lookup, label="observed"):
    return analysis.influence(
        rows, lookup, "fresh", "residual", "residual_sigma", label
    )


def chosen(rows, planner):
    return next(r for r in rows if r["unit_id"] == planner)


class EventPlannerInfluenceTests(unittest.TestCase):
    def test_hand_calculated_planner_deletion_and_planner_only_use_original_weights(
        self,
    ):
        rr, lookup = cases()
        base, effects, summary, _ = run(rr, lookup)[0]
        self.assertEqual(base["denominator"], F(23, 36))
        self.assertEqual(base["auc"], F(1, 2))
        r = chosen(effects, "JPS")
        self.assertEqual(r["auc"], F(5, 28))
        self.assertEqual(r["planner_auc"], 1)
        self.assertEqual(r["removed_pair_share"], F(9, 23))
        self.assertEqual(r["planner_denominator"], F(1, 4))
        self.assertEqual(r["denominator"], F(14, 36))
        self.assertEqual(summary["auc_min"], F(5, 28))
        self.assertEqual(summary["auc_max"], F(23, 28))
        self.assertEqual(summary["auc_planner_only_min"], 0)
        self.assertEqual(summary["auc_planner_only_max"], 1)

    def test_planner_groups_multiple_cases_and_families(self):
        rr, _ = cases()
        for r in rr:
            if r["planner"] == "Theta*":
                r["planner"] = "JPS"
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        audit = analysis.planner_audits(rr, lookup, "fresh")[0]
        u = chosen(audit, "JPS")
        self.assertEqual(u["cases"], 2)
        self.assertEqual(len(u["family_ids"].split("+")), 2)
        _, effects, summary, _ = run(rr, lookup)[0]
        r = chosen(effects, "JPS")
        self.assertEqual(r["removed_cases"], 2)
        self.assertEqual(r["planner_pair_count"], 2)
        self.assertEqual(r["planner_auc"], F(1, 2))
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
                    for planner in analysis.PLANNERS:
                        for retain in [False, True]:
                            subset = [
                                r for r in rr if (r["planner"] == planner) == retain
                            ]
                            weights = [
                                lookup[analysis.previous.identity(r)]["case_row_weight"]
                                for r in subset
                            ]
                            direct = analysis.ties.decomposition(
                                subset, weights, lookup, target, score, label
                            )[0]
                            for i, expected in enumerate(direct):
                                actual = chosen(result[i][1], planner)
                                prefix = "planner_" if retain else ""
                                for field in analysis.SUBSET_FIELDS:
                                    self.assertEqual(
                                        actual[prefix + field],
                                        expected[field],
                                        (planner, retain, i, field),
                                    )

    def test_baseline_preserves_every_previous_precision_field(self):
        rr, lookup = cases()
        weights = [lookup[analysis.previous.identity(r)]["case_row_weight"] for r in rr]
        expected = analysis.ties.decomposition(
            rr, weights, lookup, "residual", "residual_sigma", "observed"
        )[0]
        self.assertEqual([v[0] for v in run(rr, lookup)], expected)

    def test_absent_planner_is_deletion_noop_and_planner_only_undefined(self):
        rr, lookup = cases()
        rr = [r for r in rr if r["planner"] != "Theta*"]
        base, effects, summary, _ = run(rr, lookup)[0]
        r = chosen(effects, "Theta*")
        self.assertEqual(r["removed_records"], 0)
        self.assertEqual(r["removed_pair_share"], 0)
        self.assertEqual(r["auc"], base["auc"])
        self.assertEqual(r["delta_auc"], 0)
        self.assertIsNone(r["planner_auc"])
        self.assertEqual(summary["units"], 3)
        self.assertEqual(summary["auc_planner_only_undefined"], 1)

    def test_pair_free_planner_keeps_available_records_but_no_pairs(self):
        rr, _ = cases()
        rr = [r for r in rr if r["planner"] != "Theta*"]
        spare, _ = fixture([(0, 1, 1e-8, 0.05)], planner="Theta*")
        spare[0]["planner"] = "Theta*"
        rr += spare
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        base, effects, summary, _ = run(rr, lookup)[0]
        r = chosen(effects, "Theta*")
        self.assertEqual(r["removed_records"], 1)
        self.assertEqual(r["removed_pair_count"], 0)
        self.assertEqual(r["auc"], base["auc"])
        self.assertIsNone(r["planner_auc"])
        self.assertEqual(summary["represented_units"], 3)
        self.assertEqual(summary["pair_supported_units"], 2)

    def test_all_empty_groups_keep_three_audits_and_effects_per_scope(self):
        rr, lookup = cases()
        audit, cells = analysis.planner_audits([], lookup, "fresh")
        self.assertEqual(len(cells), 27)
        self.assertEqual(len(audit), 3)
        self.assertTrue(all(u["case_ids"] == u["family_ids"] == "" for u in audit))
        for base, effects, summary, cells in run([], lookup):
            self.assertEqual(len(effects), 3)
            self.assertIsNone(base["auc"])
            self.assertEqual(summary["represented_units"], 0)
            self.assertEqual(summary["auc_undefined_deletions"], 3)
            self.assertEqual(summary["auc_planner_only_undefined"], 3)
            self.assertIsNone(summary["max_removed_pair_share"])

    def test_single_class_and_cross_planner_classes_create_no_same_case_pairs(self):
        a, _ = fixture([(0, 2, 2e-8, 0.2)])
        b, _ = fixture([(0, 1, 1e-8, 0.05)])
        b[0]["planner"] = "JPS"
        rr = a + b
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        for base, effects, summary, cells in run(rr, lookup):
            self.assertEqual(base["pair_count"], 0)
            self.assertTrue(all(r["planner_auc"] is None for r in effects))
            self.assertEqual(summary["auc_strict_side_flips"], 0)

    def test_all_ties_preserve_half_and_undefined_unequal_concordance(self):
        rr, lookup = fixture([(0, 2, 2e-8, 0.2), (0.02, 2, 2e-8, 0.05)])
        _, effects, summary, _ = run(rr, lookup)[0]
        self.assertEqual(chosen(effects, "A*")["planner_auc"], F(1, 2))
        self.assertIsNone(chosen(effects, "A*")["planner_tick_unequal_concordance"])
        self.assertEqual(summary["tick_unequal_concordance_planner_only_undefined"], 3)

    def test_native_tie_resolution_keeps_both_score_orders(self):
        rr, lookup = fixture([(0, 2, 2.1e-8, 0.2), (0.02, 2, 2e-8, 0.05)])
        _, effects, summary, _ = run(rr, lookup)[0]
        r = chosen(effects, "A*")
        self.assertEqual(r["planner_auc"], F(1, 2))
        self.assertEqual(r["planner_native_auc"], 1)
        self.assertIsNone(r["planner_tick_unequal_concordance"])
        self.assertEqual(r["planner_native_unequal_concordance"], 1)

    def test_only_support_planner_removal_stays_undefined_not_zero_or_flip(self):
        rr, lookup = fixture([(0, 2, 2e-8, 0.2), (0.02, 1, 1e-8, 0.05)])
        _, effects, summary, _ = run(rr, lookup)[0]
        r = chosen(effects, "A*")
        self.assertIsNone(r["auc"])
        self.assertIsNone(r["delta_auc"])
        self.assertEqual(r["removed_pair_share"], 1)
        self.assertEqual(summary["auc_undefined_deletions"], 1)
        self.assertEqual(summary["auc_strict_side_flips"], 0)

    def test_exact_half_departures_are_not_strict_side_flips(self):
        rr, lookup = cases()
        _, _, summary, _ = run(rr, lookup)[0]
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
            r["planner"] = "JPS"
        rr = a + b
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        base, _, summary, _ = run(rr, lookup)[0]
        self.assertEqual(base["auc"], F(9, 17))
        self.assertEqual(summary["auc_strict_side_flips"], 1)

    def test_stratum_selection_keeps_full_parent_case_weights(self):
        rr, lookup = cases()
        selected = [r for i, r in enumerate(rr) if i != 0]
        u = chosen(analysis.planner_audits(selected, lookup, "fresh")[0], "A*")
        self.assertEqual(u["case_mass"], F(5, 6))
        _, effects, _, _ = run(selected, lookup)[0]
        self.assertEqual(chosen(effects, "A*")["removed_case_mass"], F(5, 6))

    def test_fixed_common_family_keeps_other_two_planners_after_deletion(self):
        rr, _ = cases()
        for r in rr:
            r["scenario"] = "dense_50x50"
            r["seed"] = "277"
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        audits = analysis.planner_audits(
            rr, lookup, "fresh", {"population": "common_family"}
        )
        base, effects, summary, cells = analysis.influence(
            rr, lookup, "fresh", "residual", "residual_sigma", "observed", audits
        )[0]
        removed = chosen(effects, "A*")
        self.assertEqual(removed["cases"], 2)
        self.assertEqual(removed["removed_cases"], 1)
        self.assertEqual(removed["pair_count"], 2)
        self.assertEqual({u["unit_id"] for u in audits[0]}, set(analysis.PLANNERS))
        self.assertEqual(len({info["family_id"] for info in lookup.values()}), 1)

    def test_same_case_different_events_never_create_pairs(self):
        rr, lookup = fixture([(0, 2, 2e-8, 0.2), (0.02, 1, 1e-8, 0.05)])
        rr[1]["_event_id"] = "other"
        self.assertEqual(run(rr, lookup)[0][0]["pair_count"], 0)

    def test_planner_only_includes_exact_time_boundaries_without_rounding_leaks(self):
        spec = [(0, 2, 2e-8, 0.2)] + [
            (t, 1, 1e-8, 0.05) for t in [0.02, 0.020001, 0.1, 0.100001, 0.5, 0.500001]
        ]
        rr, lookup = fixture(spec)
        result = run(rr, lookup)
        self.assertEqual(
            [chosen(r[1], "A*")["planner_pair_count"] for r in result],
            [6, 5, 3, 1],
        )
        self.assertEqual([r[0]["pair_count"] for r in result], [6, 5, 3, 1])

    def test_rejects_missing_duplicate_unknown_or_malformed_planners_and_seeds(self):
        for values in [
            ("A*", "A*", "Theta*"),
            ("A*", "JPS"),
            ("A*", "JPS", "unknown"),
            ("A*", "JPS", "Theta* "),
            ("A*", "JPS", 3),
        ]:
            with self.assertRaises(AssertionError):
                analysis.planned_planners(values)
        for key, value in [
            ("planner", "unknown"),
            ("seed", "280"),
            ("partition", "unknown"),
        ]:
            rr, lookup = cases()
            rr[0][key] = value
            with self.assertRaises(AssertionError):
                analysis.planner_audits(rr, lookup, "fresh")

    def test_all_27_cells_recompute_directly_and_close_to_baseline(self):
        rr, lookup = cases()
        for i, (base, effects, summary, cells) in enumerate(run(rr, lookup)):
            self.assertEqual(len(cells), 27)
            for c in cells:
                subset = [
                    r
                    for r in rr
                    if r["seed"] == c["seed"]
                    and r["planner"] == c["unit_id"]
                    and r["scenario"] == c["scenario"]
                ]
                weights = [
                    lookup[analysis.previous.identity(r)]["case_row_weight"]
                    for r in subset
                ]
                direct = analysis.ties.decomposition(
                    subset, weights, lookup, "residual", "residual_sigma", "observed"
                )[0][i]
                for field in analysis.CELL_FIELDS:
                    self.assertEqual(
                        c[field], direct[field], (i, c["seed"], c["unit_id"], field)
                    )
            for field in [
                *analysis.CELL_MASS,
                "numerator",
                "denominator",
                "native_numerator",
                *analysis.COUNTS[:9],
            ]:
                self.assertEqual(sum(c[field] for c in cells), base[field], field)

    def test_planner_removal_covers_multiple_seeds_without_cross_seed_pairs(self):
        rr, _ = cases()
        for r in rr:
            r["planner"] = "JPS"
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        base, effects, summary, cells = run(rr, lookup)[0]
        removed = chosen(effects, "JPS")
        self.assertEqual(removed["removed_cases"], 3)
        self.assertEqual(removed["removed_records"], 10)
        self.assertIsNone(removed["auc"])
        self.assertEqual(removed["planner_pair_count"], 7)
        self.assertEqual(removed["planner_auc"], F(1, 2))
        self.assertEqual(sum(c["pair_count"] > 0 for c in cells), 3)
        self.assertEqual(summary["represented_units"], 1)

    def test_empty_leaf_audits_preserve_all_27_identifier_triples(self):
        rr, lookup = cases()
        units, cells = analysis.planner_audits([], lookup, "fresh")
        self.assertEqual(
            {(c["unit_id"], c["scenario"], c["seed"]) for c in cells},
            {
                (p, s, t)
                for p in analysis.PLANNERS
                for s in analysis.SCENARIOS
                for t in ["277", "278", "279"]
            },
        )
        self.assertTrue(
            all(
                c["case_ids"] == c["family_ids"] == "" and c["case_mass"] == 0
                for c in cells
            )
        )

    def test_leaf_sums_reproduce_previous_seed_scenario_values(self):
        rr, lookup = cases()
        before = analysis.prior.influence(
            rr, lookup, "fresh", "residual", "residual_sigma", "observed"
        )
        for i, (_, _, _, cells) in enumerate(run(rr, lookup)):
            collapsed = analysis.collapse_cells(cells)
            self.assertEqual(collapsed, before[i][3])

    def test_focused_leaf_export_does_not_change_aggregate_effects(self):
        rr, lookup = cases()
        full = run(rr, lookup)
        compact = analysis.influence(
            rr,
            lookup,
            "fresh",
            "residual",
            "residual_sigma",
            "observed",
            include_cells=False,
        )
        self.assertEqual([r[:3] for r in full], [r[:3] for r in compact])
        self.assertTrue(all(not r[3] for r in compact))
        for score, stratum, label, expected in [
            ("residual_sigma", "filter_single", "observed", True),
            ("residual_sigma", "fusion_three", "modeled", True),
            ("contrast", "filter_single", "observed", False),
            ("residual_sigma", "recovery_single", "observed", False),
            ("residual_sigma", "filter_single", "unmodeled", False),
        ]:
            self.assertEqual(
                analysis.focused(dict(score=score, stratum=stratum, label=label)),
                expected,
            )

    def test_rows_and_lookup_are_not_mutated(self):
        rr, lookup = cases()
        before = deepcopy((rr, lookup))
        run(rr, lookup)
        self.assertEqual((rr, lookup), before)


if __name__ == "__main__":
    unittest.main()
