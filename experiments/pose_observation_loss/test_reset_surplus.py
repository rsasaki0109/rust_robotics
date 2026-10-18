"""Check fraction identities, eligibility, guarded denominators and retained counts."""

from copy import deepcopy
from decimal import Decimal as D
import math
from pathlib import Path
import tempfile
import unittest

import analyze_reset_surplus as a
import verify_reset_surplus as v
from test_reset_margin import fixture as margin_fixture


def fixture(**kwargs):
    row = a.inherited.decompose([margin_fixture(**kwargs)])[0]
    return {key: "" if value is None else str(value) for key, value in row.items()}


def decimal_box(values):
    return v.Interval(*(D.from_float(float(value)) for value in values))


class FractionIdentities(unittest.TestCase):
    def calculate(self, **kwargs):
        row = fixture(**kwargs)
        aa, vv = a.compute(row), v.compute(row)
        self.assertEqual(set(aa), set(vv))
        for key in aa:
            if isinstance(vv[key], D):
                self.assertTrue(
                    math.isclose(aa[key], float(vv[key]), abs_tol=1e-12, rel_tol=1e-10),
                    key,
                )
            else:
                self.assertEqual(aa[key], vv[key], key)
        return aa

    def test_deficit_and_surplus_partition_recorded_radial_component(self):
        r = self.calculate()
        self.assertAlmostEqual(r["raw_deficit_center"], 0.2)
        self.assertAlmostEqual(r["radial_surplus_center"], 0.8)
        self.assertAlmostEqual(r["required_fraction_center"], 0.2)
        self.assertAlmostEqual(r["surplus_fraction_center"], 0.8)
        self.assertEqual(
            (r["required_fraction_relation"], r["surplus_fraction_state"]),
            ("below_one", "positive"),
        )

    def test_larger_deficit_requires_larger_fraction(self):
        r = self.calculate(margin=1.9)
        self.assertAlmostEqual(r["required_fraction_center"], 0.9)
        self.assertAlmostEqual(r["surplus_fraction_center"], 0.1)

    def test_near_zero_deficit_fraction_remains_numeric(self):
        r = self.calculate(margin=1.000001)
        self.assertEqual(r["surplus_status"], "complete")
        self.assertAlmostEqual(r["required_fraction_center"], 1e-6)
        self.assertGreater(r["required_fraction_lo"], 0)

    def test_near_one_fraction_retains_small_positive_surplus(self):
        r = self.calculate(margin=1.999999)
        self.assertEqual(r["required_fraction_relation"], "below_one")
        self.assertAlmostEqual(r["surplus_fraction_center"], 1e-6)
        self.assertGreater(r["surplus_fraction_lo"], 0)

    def test_other_complete_categories_are_nonapplicable_not_zero(self):
        for kwargs in (
            {"margin": 0.1},
            {"mean": (1.0, 1.0)},
            {"mean": (1.1, 1.0), "margin": 1.1},
            {"margin": 3.0},
        ):
            r = self.calculate(**kwargs)
            self.assertEqual(r["surplus_status"], "not_applicable_category")
            self.assertEqual(r["required_fraction_relation"], "not_applicable")
            self.assertEqual(r["surplus_fraction_state"], "not_applicable")
            self.assertTrue(
                all(
                    r[q + "_" + s] is None
                    for q in a.QUANTITIES
                    for s in ("center", "lo", "hi")
                )
            )

    def test_incomplete_margin_retains_unknown_blank_ratios(self):
        for kwargs in ({"raw": (0.0, 0.0)}, {"mean": (-2.0, 0.0)}):
            r = self.calculate(**kwargs)
            self.assertEqual(r["surplus_status"], "unresolved_margin")
            self.assertEqual(r["required_fraction_relation"], "unknown")
            self.assertIsNone(r["required_fraction_center"])

    def test_wide_boxes_are_not_clipped_into_unit_interval(self):
        row = dict(fixture(), radial_component_lo="0.1", radial_component_hi="2.0")
        for m in (a, v):
            r = m.compute(row)
            self.assertEqual(r["surplus_status"], "complete")
            self.assertGreater(r["required_fraction_hi"], 1)
            self.assertLess(r["surplus_fraction_lo"], 0)
            self.assertGreater(r["surplus_fraction_hi"], 1)
            self.assertEqual(
                (r["required_fraction_relation"], r["surplus_fraction_state"]),
                ("unknown", "unknown"),
            )


class GuardedClassification(unittest.TestCase):
    def test_required_fraction_relations_use_strict_full_bounds(self):
        for values, label in (
            ((1, 1, 1), "unknown"),
            ((0.9, 0.8, 1), "unknown"),
            ((1.1, 1, 1.2), "unknown"),
            ((0.999999999999999, 0.9, 0.9999999999999999), "below_one"),
            ((1.000000000000001, 1.0000000000000002, 2), "above_one"),
        ):
            self.assertEqual(a.relation(a.box(*values)), label)
            self.assertEqual(v.relation(decimal_box(values)), label)

    def test_surplus_signs_use_no_tolerance_cutoff(self):
        for values, label in (
            ((1e-15, 0, 1), "unknown"),
            ((0, -1, 0), "unknown"),
            ((1e-15, 1e-16, 1), "positive"),
            ((-1e-15, -1, -1e-16), "negative"),
        ):
            self.assertEqual(a.sign(a.box(*values)), label)
            self.assertEqual(v.sign(decimal_box(values)), label)

    def test_zero_containing_radial_box_is_not_resolved_by_positive_center(self):
        for lo in ("0", "-1"):
            row = dict(fixture(), radial_component_lo=lo)
            for m in (a, v):
                r = m.compute(row)
                self.assertEqual(r["surplus_status"], "unresolved_radial_denominator")
                self.assertIsNone(r["required_fraction_center"])
                self.assertEqual(r["required_fraction_relation"], "unknown")

    def test_nonpositive_radial_denominator_stays_unresolved(self):
        row = fixture()
        row.update(
            radial_component_center="-1",
            radial_component_lo="-2",
            radial_component_hi="-0.5",
        )
        for m in (a, v):
            self.assertEqual(
                m.compute(row)["surplus_status"], "unresolved_radial_denominator"
            )

    def test_invalid_bounds_statuses_and_category_are_rejected(self):
        row = fixture()
        for field, text in (
            ("radial_component_lo", "2"),
            ("witness_raw_current_gap_hi", "nan"),
            ("margin_status", "invalid"),
            ("point_gap_category", "invalid"),
            ("point_gap_category", "already_clear"),
            ("radial_gap_state", "blocked"),
            ("witness_raw_current_state", "clear"),
        ):
            for m in (a, v):
                with self.assertRaises(AssertionError):
                    m.compute(dict(row, **{field: text}))

    def test_unresolved_margin_cannot_claim_complete_category(self):
        row = dict(fixture(raw=(0.0, 0.0)), point_gap_category="already_clear")
        for m in (a, v):
            with self.assertRaises(AssertionError):
                m.compute(row)

    def test_corrupt_deficit_radial_or_gap_identity_is_rejected(self):
        row = fixture()
        for field in ("radial_component", "witness_raw_current_gap", "radial_gap"):
            bad = row.copy()
            for side in ("center", "lo", "hi"):
                bad[field + "_" + side] = str(float(bad[field + "_" + side]) + 0.01)
            for m in (a, v):
                with self.assertRaises(AssertionError):
                    m.compute(bad)

    def test_zero_containment_is_required_even_inside_center_tolerance(self):
        row = fixture()
        for field in ("witness_raw_current_gap", "radial_component", "radial_gap"):
            c = float(row[field + "_center"]) + (5e-11 if field == "radial_gap" else 0)
            for side in ("center", "lo", "hi"):
                row[field + "_" + side] = str(c)
        for m in (a, v):
            with self.assertRaises(AssertionError):
                m.compute(row)


class RetainedPopulations(unittest.TestCase):
    def test_every_source_string_is_copied(self):
        row = fixture()
        for m in (a, v):
            result = m.calculate([row])[0]
            self.assertTrue(all(result[k] == text for k, text in row.items()))
            self.assertEqual(result["literal"], "001.2300")

    def test_duplicate_edges_are_rejected(self):
        row = fixture()
        for m in (a, v):
            with self.assertRaises(AssertionError):
                m.calculate([row, row])

    def test_nonapplicable_rows_have_no_computed_multiplicity(self):
        for m in (a, v):
            table = m.summaries(m.calculate([fixture(margin=0.1)]))
            total = table[0]
            self.assertEqual(
                (
                    total["physical_resets"],
                    total["overlapping_endings"],
                    total["computed_overlapping_endings"],
                ),
                (1, 3, 0),
            )
            self.assertEqual(
                (
                    total["status_not_applicable_category"],
                    total["required_not_applicable"],
                ),
                (1, 1),
            )
            self.assertTrue(all(total[f] is None for f in a.EXTREMA))
            self.assertEqual(m.closure(table), 90)

    def test_empty_cells_have_zero_counts_and_blank_extrema(self):
        for m in (a, v):
            table = m.summaries(m.calculate([fixture()]))
            empty = next(
                r
                for r in table
                if r["case_selector"] == "all_cases"
                and r["origin_selector"] == "ambiguous_zero"
            )
            self.assertTrue(all(empty[f] == 0 for f in a.COUNTS))
            self.assertTrue(all(empty[f] is None for f in a.EXTREMA))

    def test_interval_extrema_and_computed_counts_ignore_blank_ratios(self):
        for m in (a, v):
            r = m.calculate([fixture()])[0]
            first = dict(
                r,
                required_fraction_center=1,
                required_fraction_lo=0,
                required_fraction_hi=10,
            )
            second = dict(
                r,
                case_id="C2",
                required_fraction_center=2,
                required_fraction_lo=1,
                required_fraction_hi=3,
            )
            third = m.calculate(
                [dict(fixture(margin=0.1), case_id="C3", edge_id="C3@5")]
            )[0]
            table = m.summaries([first, second, third])
            self.assertEqual(
                [
                    table[0][f]
                    for f in (
                        "min_required_fraction_lo",
                        "min_required_fraction_hi",
                        "max_required_fraction_lo",
                        "max_required_fraction_hi",
                    )
                ],
                [0, 3, 1, 10],
            )
            self.assertEqual(
                (
                    table[0]["physical_resets"],
                    table[0]["overlapping_endings"],
                    table[0]["computed_overlapping_endings"],
                ),
                (3, 9, 6),
            )
            self.assertEqual(m.closure(table), 120)

    def test_additive_and_partition_count_closures_reject_changes(self):
        for m in (a, v):
            table = m.summaries(m.calculate([fixture()]))
            for field in (
                "computed_overlapping_endings",
                "required_below_one",
                "status_complete",
                "surplus_positive",
            ):
                bad = deepcopy(table)
                bad[1][field] += 1
                with self.assertRaises(AssertionError):
                    m.closure(bad)

    def test_verifier_checks_exact_strings_and_complete_boxes(self):
        actual, expected = a.calculate([fixture()]), v.calculate([fixture()])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rows.csv"
            a.write(path, actual, tuple(actual[0]))
            v.compare(path, expected, ("edge_id",))
            for key, text in (("literal", "1.23"), ("required_fraction_lo", "")):
                bad = [dict(actual[0], **{key: text})]
                a.write(path, bad, tuple(bad[0]))
                with self.assertRaises((AssertionError, ValueError)):
                    v.compare(path, expected, ("edge_id",))

    def test_verifier_preserves_nonapplicable_blank_ratios(self):
        actual, expected = a.calculate([fixture(margin=0.1)]), v.calculate(
            [fixture(margin=0.1)]
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rows.csv"
            a.write(path, actual, tuple(actual[0]))
            v.compare(path, expected, ("edge_id",))
            self.assertEqual(
                expected[0]["required_fraction_relation"], "not_applicable"
            )


if __name__ == "__main__":
    unittest.main()
