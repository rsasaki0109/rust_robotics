"""Planner support matching and exact weighted comparison populations."""

from copy import deepcopy
from fractions import Fraction as F
from unittest.mock import patch
import unittest

import analyze_planner_conditioned_rankings as analysis
from test_planner_window_sensitivity import account, row


def units(rows, planned=None):
    planned = planned or [row(planner=p) for p in analysis.PLANNERS]
    return analysis.window.audit_units(account(planned), rows)


class PlannerConditionedTests(unittest.TestCase):
    def test_all_support_does_not_imply_same_mode_support(self):
        rr = [
            row(planner=p, mode="fusion_three" if p == "Theta*" else "filter_single")
            for p in analysis.PLANNERS
        ]
        families, _, lookup = units(rr)
        audit, eligible = analysis.family_support(families, rr, lookup)
        self.assertEqual(eligible["all"], {"F001"})
        self.assertFalse(eligible["filter_single"])
        self.assertFalse(eligible["fusion_three"])
        fusion = next(r for r in audit if r["stratum"] == "fusion_three")
        self.assertEqual(fusion["missing_source_planners"], "A*+JPS")
        self.assertEqual(fusion["theta_records"], 1)

    def test_eligibility_does_not_depend_on_scores_labels_or_order(self):
        rr = [
            row(planner=p, tick=i, error=0.05 if i else 0.2)
            for i, p in enumerate(analysis.PLANNERS)
        ]
        families, _, lookup = units(rr)
        first = analysis.family_support(families, rr, lookup)[1]
        changed = deepcopy(rr)
        for r in changed:
            r["midpoint"]["outside"]["observed"] = not r["midpoint"]["outside"][
                "observed"
            ]
            r["residual_sigma_tick"] *= -99
        self.assertEqual(
            first, analysis.family_support(families, list(reversed(changed)), lookup)[1]
        )

    def test_missing_and_entirely_empty_families_remain_in_audit(self):
        planned = [
            row(planner=p, scenario=s)
            for p in analysis.PLANNERS
            for s in ["fixture", "empty"]
        ]
        rr = [row()]
        families, _, lookup = units(rr, planned)
        audit, eligible = analysis.family_support(families, rr, lookup)
        self.assertEqual(len(audit), 10)
        self.assertFalse(any(eligible.values()))
        missing = [r for r in audit if r["scenario"] == "empty"]
        self.assertTrue(
            all(r["source_records"] == 0 and not r["common_family"] for r in missing)
        )
        self.assertTrue(
            all(r["missing_source_planners"] == "A*+JPS+Theta*" for r in missing)
        )

    def test_unexpected_planned_planner_is_rejected(self):
        rr = [row()]
        families, _, lookup = units(
            rr, [row(planner=p) for p in [*analysis.PLANNERS, "unknown"]]
        )
        with self.assertRaises(AssertionError):
            analysis.family_support(families, rr, lookup)

    @staticmethod
    def comparison_fixture():
        rr = [
            row(planner="A*", seed="277", tick=3, error=0.2),
            row(planner="A*", seed="278", tick=3, error=0.05),
            row(planner="JPS", seed="277", tick=1, error=0.05),
            row(planner="Theta*", seed="278", tick=0, error=0.2),
        ]
        return rr, [F(1, 2), F(1, 4), F(1), F(1, 3)]

    def decompose(self, rr, ww, label="observed"):
        return analysis.planner_decomposition(
            rr, ww, "residual", "residual_sigma", label, ["277", "278", "279"]
        )

    def test_ordered_blocks_ties_and_seed_planner_intersection(self):
        result, blocks = self.decompose(*self.comparison_fixture())
        self.assertEqual(len(blocks), 9)
        self.assertEqual(result["denominator"], F(25, 24))
        self.assertEqual(result["numerator"], F(9, 16))
        self.assertEqual(result["auc"], F(27, 50))
        self.assertEqual(result["same_seed_auc"], F(6, 7))
        self.assertEqual(result["same_planner_auc"], F(1, 2))
        self.assertEqual(result["same_planner_pair_share"], F(3, 25))
        self.assertEqual(result["same_seed_planner_denominator"], 0)
        self.assertIsNone(result["same_seed_planner_auc"])
        self.assertEqual(result["same_seed_planner_pair_share"], 0)
        forward = next(
            b
            for b in blocks
            if b["positive_planner"] == "A*" and b["negative_planner"] == "JPS"
        )
        reverse = next(
            b
            for b in blocks
            if b["positive_planner"] == "JPS" and b["negative_planner"] == "A*"
        )
        self.assertEqual(forward["auc"], 1)
        self.assertEqual(forward["same_seed_denominator"], F(1, 2))
        self.assertEqual(reverse["denominator"], 0)
        self.assertIsNone(reverse["auc"])

    def test_replication_preserves_pair_mass_when_weights_are_split(self):
        rr, ww = self.comparison_fixture()
        before, blocks = self.decompose(rr, ww)
        after, repeated = self.decompose(
            [r for r in rr for _ in range(2)], [w / 2 for w in ww for _ in range(2)]
        )
        for field in [
            *analysis.previous.PAIR_FIELDS,
            "auc",
            "same_seed_auc",
            "same_planner_auc",
            "same_seed_planner_auc",
        ]:
            self.assertEqual(before[field], after[field])
        for a, b in zip(blocks, repeated):
            for field in [
                *analysis.previous.PAIR_FIELDS,
                "same_seed_numerator",
                "same_seed_denominator",
            ]:
                self.assertEqual(a[field], b[field])

    def test_empty_and_one_class_cells_keep_zero_mass_and_blank_auc(self):
        for rr, ww, label in [
            ([], [], "observed"),
            ([row()], [F(1)], "observed"),
            (self.comparison_fixture()[0], self.comparison_fixture()[1], "unmodeled"),
        ]:
            result, blocks = self.decompose(rr, ww, label)
            for field in [
                "auc",
                "same_seed_auc",
                "same_planner_auc",
                "same_seed_planner_auc",
            ]:
                self.assertIsNone(result[field])
            self.assertEqual(result["denominator"], 0)
            self.assertTrue(
                all(b["denominator"] == 0 and b["auc"] is None for b in blocks)
            )

    def test_invalid_weights_seed_or_planner_are_rejected(self):
        for rr, ww in [
            ([row()], []),
            ([row()], [F(0)]),
            ([row(seed="280")], [F(1)]),
            ([row(planner="unknown")], [F(1)]),
        ]:
            with self.assertRaises(AssertionError):
                self.decompose(rr, ww)

    def test_matching_retains_parent_mode_weights_and_all_missing_seed_rows(self):
        rr = [
            row(
                planner="A*",
                time=150 + i * 0.02,
                mode="filter_single" if i < 2 else "fusion_three",
            )
            for i in range(4)
        ]
        rr += [row(planner=p) for p in ["JPS", "Theta*"]]
        rr += [row(planner="Theta*", scenario="unmatched")]
        planned = [
            row(planner=p, scenario=s)
            for p in analysis.PLANNERS
            for s in ["fixture", "unmatched"]
        ]
        families, _, lookup = units(rr, planned)
        audit, components, ranks, blocks, seeds = analysis.summaries(
            rr, families, lookup
        )

        def selected(values, **conditions):
            return [
                r
                for r in values
                if r["partition"] == "fresh"
                and r["noise"] == "white"
                and r["variant"] == "epoch"
                and r["target"] == "residual"
                and r["stratum"] == "filter_single"
                and all(r[k] == v for k, v in conditions.items())
            ]

        astar = selected(
            components, population="common_family", planner_selection="A*"
        )[0]
        self.assertEqual(astar["records"], 2)
        self.assertEqual(astar["case_mass"], 0.5)
        self.assertEqual(astar["group_case_mass"], 4)
        self.assertEqual(astar["case_mass_share"], 0.125)
        theta_source = selected(
            components, population="source", planner_selection="Theta*"
        )[0]
        theta_common = selected(
            components, population="common_family", planner_selection="Theta*"
        )[0]
        self.assertEqual((theta_source["cases"], theta_common["cases"]), (2, 1))
        self.assertEqual((theta_source["case_mass"], theta_common["case_mass"]), (2, 1))
        empty = selected(
            seeds, population="common_family", planner_selection="A*", seed="279"
        )[0]
        self.assertEqual(empty["records"], 0)
        self.assertEqual(empty["case_mass"], 0)
        self.assertEqual(empty["observed_outside_fraction"], "")
        one_planner = selected(
            ranks, population="common_family", planner_selection="A*"
        )
        self.assertTrue(
            all(
                r["same_planner_auc"] == r["auc"]
                and r["same_seed_planner_auc"] == r["same_seed_auc"]
                for r in one_planner
            )
        )

    def test_baseline_check_rejects_changed_prior_fields(self):
        base = dict(
            partition="fresh",
            noise="white",
            variant="curved",
            stratum="all",
            target="residual",
            population="source",
            planner_selection="all",
            cases=1,
            case_mass=1,
            group_case_mass=1,
            case_mass_share=1,
            auc=0.5,
            same_seed_auc="",
            score="residual_sigma",
            label="observed",
        )
        old = {
            k: str(v)
            for k, v in base.items()
            if k not in ["population", "planner_selection"]
        }
        old.update(
            weighting="case",
            analysis_units=old.pop("cases"),
            analysis_mass=old.pop("case_mass"),
            parent_units="1",
            parent_mass=old.pop("group_case_mass"),
            mass_share=old.pop("case_mass_share"),
            auc_difference="0.0",
            same_seed_auc_difference="",
        )
        with patch.object(analysis, "read_rows", return_value=[old]):
            self.assertEqual(analysis.check_previous([base], [base]), 2 * len(old))
            with self.assertRaises(AssertionError):
                analysis.check_previous([dict(base, case_mass=0.5)], [base])


if __name__ == "__main__":
    unittest.main()
