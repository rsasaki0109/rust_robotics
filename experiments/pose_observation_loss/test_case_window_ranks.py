"""Exact case/window partitions, inherited mode mass and missing support."""

from copy import deepcopy
from fractions import Fraction as F
import unittest

import analyze_case_window_ranks as analysis
from test_planner_window_sensitivity import account, row


def audit(rr):
    planned = {}
    for r in rr:
        for planner in analysis.parent.PLANNERS:
            p = dict(r, planner=planner)
            planned[analysis.previous.identity(p)] = p
    return analysis.window.audit_units(account(list(planned.values())), rr)


def fixture():
    rr = [
        row(planner=p, time=150 + i * 0.02, tick=tick, error=error)
        for p in ["A*", "JPS"]
        for i, (tick, error) in enumerate([(3, 0.2), (1, 0.05)])
    ]
    rr += [
        row(planner="Theta*", tick=1, error=0.2),
        row(scenario="other", tick=3, error=0.05),
        row(planner="Theta*", seed="278", tick=0, error=0.2),
    ]
    families, cases, lookup = audit(rr)
    weights = [lookup[analysis.previous.identity(r)]["case_row_weight"] for r in rr]
    return rr, weights, families, lookup


def decompose(rr, ww, lookup, label="observed", parent_keys=None):
    return analysis.decompose(
        rr, ww, lookup, "residual", "residual_sigma", label, parent_keys
    )


class CaseWindowRankTests(unittest.TestCase):
    def test_all_five_bins_have_hand_calculated_ordered_pair_masses(self):
        rr, ww, _, lookup = fixture()
        extra, bins, cases, full = decompose(rr, ww, lookup)
        self.assertEqual(full["denominator"], 6)
        self.assertEqual(full["numerator"], 2)
        self.assertEqual(full["auc"], F(1, 3))
        self.assertEqual(
            [b["denominator"] for b in bins], [F(1, 2), F(1, 2), F(1), F(2), F(2)]
        )
        self.assertEqual([b["numerator"] for b in bins], [F(1, 2)] * 4 + [F(0)])
        self.assertEqual([b["candidate_case_pairs"] for b in bins], [5, 2, 4, 6, 8])
        self.assertEqual([b["supported_case_pairs"] for b in bins], [2, 2, 2, 3, 3])
        self.assertEqual(sum(b["denominator_share"] for b in bins), 1)
        self.assertEqual(sum(b["numerator_contribution"] for b in bins), F(1, 3))

    def test_nested_scopes_and_unit_class_support_preserve_original_mass(self):
        rr, ww, _, lookup = fixture()
        extra, _, _, full = decompose(rr, ww, lookup)
        self.assertEqual(extra["same_case_auc"], 1)
        self.assertEqual(extra["same_window_auc"], 1)
        self.assertEqual(extra["same_family_auc"], F(3, 4))
        self.assertEqual(
            full["same_seed_numerator"] / full["same_seed_denominator"], F(1, 2)
        )
        self.assertEqual(extra["same_case_pair_share"], F(1, 12))
        self.assertEqual(extra["same_window_pair_share"], F(1, 6))
        self.assertEqual(extra["same_family_pair_share"], F(1, 3))
        self.assertEqual(extra["between_case_auc"], F(3, 11))
        self.assertEqual(extra["cases_with_both_classes"], 2)
        self.assertEqual(extra["windows_with_both_classes"], 1)
        self.assertEqual(extra["families_with_both_classes"], 1)

    def test_identical_payload_in_another_family_is_not_the_same_window(self):
        rr = [row(), row(scenario="another")]
        _, _, lookup = audit(rr)
        a, b = [lookup[analysis.previous.identity(r)] for r in rr]
        self.assertEqual(a["window_sha256"], b["window_sha256"])
        self.assertNotEqual(a["window_id"], b["window_id"])
        self.assertEqual(analysis.category(a, b), "other_family_same_seed")

    def test_seed_is_retained_in_family_and_window_identity(self):
        rr = [row(), row(seed="278")]
        _, _, lookup = audit(rr)
        a, b = [lookup[analysis.previous.identity(r)] for r in rr]
        self.assertEqual(a["window_sha256"], b["window_sha256"])
        self.assertNotEqual(a["family_id"], b["family_id"])
        self.assertEqual(analysis.category(a, b), "other_seed")

    def test_one_class_cases_can_support_family_auc_without_case_auc(self):
        rr = [row(tick=2, error=0.2), row(planner="JPS", tick=1, error=0.05)]
        _, _, lookup = audit(rr)
        extra, bins, cases, full = decompose(rr, [F(1), F(1)], lookup)
        self.assertEqual(full["auc"], 1)
        self.assertIsNone(extra["same_case_auc"])
        self.assertIsNone(extra["same_window_auc"])
        self.assertEqual(extra["same_family_auc"], 1)
        self.assertEqual(extra["cases_with_both_classes"], 0)
        self.assertEqual(extra["families_with_both_classes"], 1)
        self.assertEqual(bins[2]["denominator"], 1)

    def test_same_case_auc_sums_pair_mass_instead_of_averaging_auc(self):
        rr = [
            row(time=150 + i * 0.02, tick=tick, error=error)
            for i, (tick, error) in enumerate([(1, 0.2), (2, 0.2), (0, 0.05)])
        ]
        rr += [
            row(planner="JPS", tick=0, error=0.2),
            row(planner="JPS", time=150.02, tick=1, error=0.05),
        ]
        _, _, lookup = audit(rr)
        ww = [lookup[analysis.previous.identity(r)]["case_row_weight"] for r in rr]
        extra, _, cases, _ = decompose(rr, ww, lookup)
        self.assertEqual(sorted(c["auc"] for c in cases.values()), [0, 1])
        self.assertEqual(extra["same_case_auc"], F(8, 17))
        self.assertNotEqual(extra["same_case_auc"], F(1, 2))

    def test_mode_subsetting_retains_parent_weights_and_empty_case_rows(self):
        rr, ww, _, lookup = fixture()
        rr[0]["stratum"] = "fusion_three"
        selected = [(r, w) for r, w in zip(rr, ww) if r["stratum"] == "filter_single"]
        keys = set(analysis.previous.identity(r) for r in rr)
        _, _, cases, _ = decompose(
            [r for r, w in selected], [w for r, w in selected], lookup, parent_keys=keys
        )
        astar = cases[analysis.previous.identity(rr[0])]
        self.assertEqual(astar["records"], 1)
        self.assertEqual(astar["case_mass"], F(1, 2))
        self.assertIsNone(astar["auc"])
        empty, bins, cases, full = decompose([], [], lookup, parent_keys=keys)
        self.assertEqual(len(cases), 5)
        self.assertTrue(
            all(c["records"] == 0 and c["auc"] is None for c in cases.values())
        )
        self.assertTrue(
            all(b["denominator"] == 0 and b["denominator_share"] is None for b in bins)
        )
        self.assertIsNone(empty["same_case_auc"])

    def test_empty_and_unmodeled_one_class_groups_never_impute_auc(self):
        rr, ww, _, lookup = fixture()
        for rows, weights, label in [([], [], "observed"), (rr, ww, "unmodeled")]:
            extra, bins, _, full = decompose(rows, weights, lookup, label)
            self.assertIsNone(full["auc"])
            self.assertEqual(full["denominator"], 0)
            self.assertTrue(
                all(
                    b["auc"] is None and b["numerator_contribution"] is None
                    for b in bins
                )
            )
            self.assertTrue(
                all(
                    extra[name + "_auc"] is None
                    for name in [
                        "same_case",
                        "same_window",
                        "same_family",
                        "between_case",
                    ]
                )
            )

    def test_invalid_or_renormalized_weights_and_duplicate_parent_keys_fail(self):
        rr, ww, _, lookup = fixture()
        for wrong in [[], [F(0)] * len(rr), [F(1)] * len(rr)]:
            with self.assertRaises(AssertionError):
                decompose(rr, wrong, lookup)
        key = analysis.previous.identity(rr[0])
        with self.assertRaises(AssertionError):
            decompose([], [], lookup, parent_keys=[key, key])

    def test_all_planned_cases_and_populations_keep_missing_support(self):
        rr, ww, families, lookup = fixture()
        audit_rows, ranks, bins, case_ranks, family_audit = analysis.summaries(
            rr, families, lookup
        )
        self.assertEqual(len(audit_rows), 9 * 5)
        self.assertEqual(len(case_ranks), 5 * 5 * 2 * 9)
        missing = [r for r in audit_rows if r["parent_records"] == 0]
        self.assertEqual(len(missing), 4 * 5)
        self.assertTrue(
            all(
                r["case_row_weight"] == ""
                and r["case_mass"] == 0
                and not r["common_family_eligible"]
                for r in missing
            )
        )
        fresh = [
            r
            for r in ranks
            if all(
                r[k] == v
                for k, v in dict(
                    partition="fresh",
                    noise="white",
                    variant="epoch",
                    stratum="filter_single",
                    target="residual",
                    score="residual_sigma",
                    label="observed",
                ).items()
            )
        ]
        source = next(r for r in fresh if r["population"] == "source")
        common = next(r for r in fresh if r["population"] == "common_family")
        self.assertEqual((source["records"], common["records"]), (7, 5))
        self.assertEqual((source["case_mass"], common["case_mass"]), (5, 3))
        self.assertEqual(source["same_case_auc"], common["same_case_auc"])
        self.assertEqual(common["auc"], 0.75)
        self.assertEqual(common["same_case_pair_share"], 0.25)
        self.assertTrue(
            all(
                r["case_row_weight"] == 0.5
                for r in case_ranks
                if r["planner"] in ["A*", "JPS"]
                and r["scenario"] == "fixture"
                and r["seed"] == "277"
            )
        )

    def test_baseline_comparison_catches_changed_fields_and_ignores_new_columns(self):
        old = [dict(key="fixture", auc="0.5", unsupported="", eligible="True")]
        new = [dict(key="fixture", auc=0.5, unsupported="", eligible=True, extra="new")]
        self.assertEqual(analysis.compare(new, old, ["key"]), 4)
        new[0]["auc"] = 0.6
        with self.assertRaises(AssertionError):
            analysis.compare(new, old, ["key"])


if __name__ == "__main__":
    unittest.main()
