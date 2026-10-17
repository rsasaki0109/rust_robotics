"""Loss-mode subsets, exact original weights and full-case identity."""

from copy import deepcopy
from fractions import Fraction as F
import unittest

import analyze_event_loss_mode_influence as analysis
from test_event_planner_influence import (
    cases as planner_cases,
    fixture as planner_fixture,
)
from test_planner_window_sensitivity import account


def cases():
    rows, _ = planner_cases()
    for r in rows:
        r["mode"] = {"A*": "none", "JPS": "random", "Theta*": "blackout"}[r["planner"]]
    _, _, lookup = analysis.window.audit_units(account(rows), rows)
    return rows, lookup


def fixture(spec=None, mode="none"):
    rows, _ = planner_fixture(spec)
    for r in rows:
        r["mode"] = mode
    _, _, lookup = analysis.window.audit_units(account(rows), rows)
    return rows, lookup


def run(rows, lookup, label="observed"):
    return analysis.influence(
        rows, lookup, "fresh", "residual", "residual_sigma", label
    )


def chosen(rows, mode):
    return next(r for r in rows if r["unit_id"] == mode)


class EventLossModeInfluenceTests(unittest.TestCase):
    def test_hand_calculated_mode_deletion_and_mode_only_use_original_weights(
        self,
    ):
        rr, lookup = cases()
        base, effects, summary, _ = run(rr, lookup)[0]
        self.assertEqual(base["denominator"], F(23, 36))
        self.assertEqual(base["auc"], F(1, 2))
        r = chosen(effects, "random")
        self.assertEqual(r["auc"], F(5, 28))
        self.assertEqual(r["mode_auc"], 1)
        self.assertEqual(r["removed_pair_share"], F(9, 23))
        self.assertEqual(r["mode_denominator"], F(1, 4))
        self.assertEqual(r["denominator"], F(14, 36))
        self.assertEqual(summary["auc_min"], F(5, 28))
        self.assertEqual(summary["auc_max"], F(23, 28))
        self.assertEqual(summary["auc_mode_only_min"], 0)
        self.assertEqual(summary["auc_mode_only_max"], 1)

    def test_mode_groups_multiple_cases_and_families(self):
        rr, _ = cases()
        for r in rr:
            if r["mode"] == "blackout":
                r["mode"] = "random"
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        audit = analysis.mode_audits(rr, lookup, "fresh")[0]
        u = chosen(audit, "random")
        self.assertEqual(u["cases"], 2)
        self.assertEqual(len(u["family_ids"].split("+")), 2)
        _, effects, summary, _ = run(rr, lookup)[0]
        r = chosen(effects, "random")
        self.assertEqual(r["removed_cases"], 2)
        self.assertEqual(r["mode_pair_count"], 2)
        self.assertEqual(r["mode_auc"], F(1, 2))
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
                    for mode in analysis.LOSS_MODES:
                        for retain in [False, True]:
                            subset = [r for r in rr if (r["mode"] == mode) == retain]
                            weights = [
                                lookup[analysis.previous.identity(r)]["case_row_weight"]
                                for r in subset
                            ]
                            direct = analysis.ties.decomposition(
                                subset, weights, lookup, target, score, label
                            )[0]
                            for i, expected in enumerate(direct):
                                actual = chosen(result[i][1], mode)
                                prefix = "mode_" if retain else ""
                                for field in analysis.SUBSET_FIELDS:
                                    self.assertEqual(
                                        actual[prefix + field],
                                        expected[field],
                                        (mode, retain, i, field),
                                    )

    def test_baseline_preserves_every_previous_precision_field(self):
        rr, lookup = cases()
        weights = [lookup[analysis.previous.identity(r)]["case_row_weight"] for r in rr]
        expected = analysis.ties.decomposition(
            rr, weights, lookup, "residual", "residual_sigma", "observed"
        )[0]
        self.assertEqual([v[0] for v in run(rr, lookup)], expected)

    def test_absent_mode_is_deletion_noop_and_mode_only_undefined(self):
        rr, lookup = cases()
        rr = [r for r in rr if r["mode"] != "blackout"]
        base, effects, summary, _ = run(rr, lookup)[0]
        r = chosen(effects, "blackout")
        self.assertEqual(r["removed_records"], 0)
        self.assertEqual(r["removed_pair_share"], 0)
        self.assertEqual(r["auc"], base["auc"])
        self.assertEqual(r["delta_auc"], 0)
        self.assertIsNone(r["mode_auc"])
        self.assertEqual(summary["units"], 3)
        self.assertEqual(summary["auc_mode_only_undefined"], 1)

    def test_pair_free_mode_keeps_available_records_but_no_pairs(self):
        rr, _ = cases()
        rr = [r for r in rr if r["mode"] != "blackout"]
        spare, _ = fixture([(0, 1, 1e-8, 0.05)], mode="blackout")
        spare[0]["mode"] = "blackout"
        rr += spare
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        base, effects, summary, _ = run(rr, lookup)[0]
        r = chosen(effects, "blackout")
        self.assertEqual(r["removed_records"], 1)
        self.assertEqual(r["removed_pair_count"], 0)
        self.assertEqual(r["auc"], base["auc"])
        self.assertIsNone(r["mode_auc"])
        self.assertEqual(summary["represented_units"], 3)
        self.assertEqual(summary["pair_supported_units"], 2)

    def test_all_empty_groups_keep_three_audits_and_effects_per_scope(self):
        rr, lookup = cases()
        audit, cells = analysis.mode_audits([], lookup, "fresh")
        self.assertEqual(len(cells), 81)
        self.assertEqual(len(audit), 3)
        self.assertTrue(all(u["case_ids"] == u["family_ids"] == "" for u in audit))
        for base, effects, summary, cells in run([], lookup):
            self.assertEqual(len(effects), 3)
            self.assertIsNone(base["auc"])
            self.assertEqual(summary["represented_units"], 0)
            self.assertEqual(summary["auc_undefined_deletions"], 3)
            self.assertEqual(summary["auc_mode_only_undefined"], 3)
            self.assertIsNone(summary["max_removed_pair_share"])

    def test_single_class_and_cross_mode_classes_create_no_same_case_pairs(self):
        a, _ = fixture([(0, 2, 2e-8, 0.2)])
        b, _ = fixture([(0, 1, 1e-8, 0.05)])
        b[0]["mode"] = "random"
        rr = a + b
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        for base, effects, summary, cells in run(rr, lookup):
            self.assertEqual(base["pair_count"], 0)
            self.assertTrue(all(r["mode_auc"] is None for r in effects))
            self.assertEqual(summary["auc_strict_side_flips"], 0)

    def test_all_ties_preserve_half_and_undefined_unequal_concordance(self):
        rr, lookup = fixture([(0, 2, 2e-8, 0.2), (0.02, 2, 2e-8, 0.05)])
        _, effects, summary, _ = run(rr, lookup)[0]
        self.assertEqual(chosen(effects, "none")["mode_auc"], F(1, 2))
        self.assertIsNone(chosen(effects, "none")["mode_tick_unequal_concordance"])
        self.assertEqual(summary["tick_unequal_concordance_mode_only_undefined"], 3)

    def test_native_tie_resolution_keeps_both_score_orders(self):
        rr, lookup = fixture([(0, 2, 2.1e-8, 0.2), (0.02, 2, 2e-8, 0.05)])
        _, effects, summary, _ = run(rr, lookup)[0]
        r = chosen(effects, "none")
        self.assertEqual(r["mode_auc"], F(1, 2))
        self.assertEqual(r["mode_native_auc"], 1)
        self.assertIsNone(r["mode_tick_unequal_concordance"])
        self.assertEqual(r["mode_native_unequal_concordance"], 1)

    def test_only_support_mode_removal_stays_undefined_not_zero_or_flip(self):
        rr, lookup = fixture([(0, 2, 2e-8, 0.2), (0.02, 1, 1e-8, 0.05)])
        _, effects, summary, _ = run(rr, lookup)[0]
        r = chosen(effects, "none")
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
            r["mode"] = "random"
        rr = a + b
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        base, _, summary, _ = run(rr, lookup)[0]
        self.assertEqual(base["auc"], F(9, 17))
        self.assertEqual(summary["auc_strict_side_flips"], 1)

    def test_stratum_selection_keeps_full_parent_case_weights(self):
        rr, lookup = cases()
        selected = [r for i, r in enumerate(rr) if i != 0]
        u = chosen(analysis.mode_audits(selected, lookup, "fresh")[0], "none")
        self.assertEqual(u["case_mass"], F(5, 6))
        _, effects, _, _ = run(selected, lookup)[0]
        self.assertEqual(chosen(effects, "none")["removed_case_mass"], F(5, 6))

    def test_fixed_common_family_preserves_other_mode_families(self):
        original, _ = planner_cases()
        for r in original:
            r["scenario"] = "dense_50x50"
            r["seed"] = "277"
            r["mode"] = "random"
        other = deepcopy(original)
        for r in other:
            r["mode"] = "blackout"
        rows = original + other
        _, _, lookup = analysis.window.audit_units(account(rows), rows)
        audits = analysis.mode_audits(
            rows, lookup, "fresh", {"population": "common_family"}
        )
        baseline, effects, summary, cells = analysis.influence(
            rows, lookup, "fresh", "residual", "residual_sigma", "observed", audits
        )[0]
        removed = chosen(effects, "random")
        self.assertEqual(removed["cases"], 3)
        self.assertEqual(removed["removed_cases"], 3)
        self.assertEqual(removed["pair_count"], 7)
        self.assertEqual(len({v["family_id"] for v in lookup.values()}), 2)
        self.assertEqual(removed["auc"], F(1, 2))

    def test_same_case_different_events_never_create_pairs(self):
        rr, lookup = fixture([(0, 2, 2e-8, 0.2), (0.02, 1, 1e-8, 0.05)])
        rr[1]["_event_id"] = "other"
        self.assertEqual(run(rr, lookup)[0][0]["pair_count"], 0)

    def test_loss_modes_remain_distinct_full_cases_with_no_cross_mode_pairs(self):
        positive, _ = fixture([(0, 2, 2e-8, 0.2)], mode="random")
        negative, _ = fixture([(0, 1, 1e-8, 0.05)], mode="blackout")
        rows = positive + negative
        _, _, lookup = analysis.window.audit_units(account(rows), rows)
        units, audits = analysis.mode_audits(rows, lookup, "fresh")
        nonempty = [a for a in audits if a["records"]]
        self.assertEqual(len(nonempty), 2)
        self.assertTrue(all(a["cases"] == a["events"] == 1 for a in nonempty))
        for baseline, effects, summary, cells in run(rows, lookup):
            self.assertEqual(baseline["pair_count"], 0)
            self.assertTrue(all(c["pair_count"] == 0 for c in cells))

    def test_mode_only_includes_exact_time_boundaries_without_rounding_leaks(self):
        spec = [(0, 2, 2e-8, 0.2)] + [
            (t, 1, 1e-8, 0.05) for t in [0.02, 0.020001, 0.1, 0.100001, 0.5, 0.500001]
        ]
        rr, lookup = fixture(spec)
        result = run(rr, lookup)
        self.assertEqual(
            [chosen(r[1], "none")["mode_pair_count"] for r in result],
            [6, 5, 3, 1],
        )
        self.assertEqual([r[0]["pair_count"] for r in result], [6, 5, 3, 1])

    def test_rejects_missing_duplicate_unknown_or_malformed_modes_and_seeds(self):
        for values in [
            ("none", "none", "blackout"),
            ("none", "random"),
            ("none", "random", "unknown"),
            ("none", "random", "blackout "),
            ("none", "random", 3),
        ]:
            with self.assertRaises(AssertionError):
                analysis.planned_modes(values)
        for key, value in [
            ("mode", "unknown"),
            ("seed", "280"),
            ("partition", "unknown"),
            ("planner", "unknown"),
            ("scenario", "unknown"),
        ]:
            rr, lookup = cases()
            rr[0][key] = value
            with self.assertRaises(AssertionError):
                analysis.mode_audits(rr, lookup, "fresh")

    def test_all_81_cells_recompute_directly_and_close_to_baseline(self):
        rr, lookup = cases()
        for i, (base, effects, summary, cells) in enumerate(run(rr, lookup)):
            self.assertEqual(len(cells), 81)
            for c in cells:
                subset = [
                    r
                    for r in rr
                    if r["seed"] == c["seed"]
                    and r["mode"] == c["unit_id"]
                    and r["scenario"] == c["scenario"]
                    and r["planner"] == c["planner"]
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

    def test_mode_removal_covers_multiple_seeds_without_cross_seed_pairs(self):
        rr, _ = cases()
        for r in rr:
            r["mode"] = "random"
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        base, effects, summary, cells = run(rr, lookup)[0]
        removed = chosen(effects, "random")
        self.assertEqual(removed["removed_cases"], 3)
        self.assertEqual(removed["removed_records"], 10)
        self.assertIsNone(removed["auc"])
        self.assertEqual(removed["mode_pair_count"], 7)
        self.assertEqual(removed["mode_auc"], F(1, 2))
        self.assertEqual(sum(c["pair_count"] > 0 for c in cells), 3)
        self.assertEqual(summary["represented_units"], 1)

    def test_empty_leaf_audits_preserve_all_81_complete_identifier_keys(self):
        rows, lookup = cases()
        units, cells = analysis.mode_audits([], lookup, "fresh")
        self.assertEqual(
            {(c["unit_id"], c["scenario"], c["seed"], c["planner"]) for c in cells},
            {
                (m, s, t, p)
                for m in analysis.LOSS_MODES
                for s in analysis.SCENARIOS
                for t in ["277", "278", "279"]
                for p in analysis.PLANNERS
            },
        )
        self.assertTrue(
            all(
                c["case_ids"] == c["family_ids"] == "" and c["case_mass"] == 0
                for c in cells
            )
        )

    def test_complete_leaf_case_id_is_blank_or_exactly_one_selected_full_case(self):
        rows, lookup = cases()
        for baseline, effects, summary, cells in run(rows, lookup):
            for cell in cells:
                selected = [
                    r
                    for r in rows
                    if r["mode"] == cell["unit_id"]
                    and r["planner"] == cell["planner"]
                    and r["scenario"] == cell["scenario"]
                    and r["seed"] == cell["seed"]
                ]
                ids = {
                    lookup[analysis.previous.identity(r)]["case_id"] for r in selected
                }
                self.assertLessEqual(len(ids), 1)
                self.assertEqual(cell["case_id"], next(iter(ids), ""))
                self.assertEqual(cell["cases"], len(ids))

    def test_absent_none_is_not_zero_auc_and_deletion_keeps_observed_modes(self):
        rows, lookup = cases()
        rows = [r for r in rows if r["mode"] != "none"]
        baseline, effects, summary, cells = run(rows, lookup)[0]
        missing = chosen(effects, "none")
        self.assertIsNone(missing["mode_auc"])
        self.assertEqual(missing["auc"], baseline["auc"])
        self.assertEqual(missing["removed_pair_share"], 0)
        self.assertEqual(missing["removed_records"], 0)
        self.assertEqual(summary["units"], 3)

    def test_mode_sums_reproduce_previous_planner_leaves(self):
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
