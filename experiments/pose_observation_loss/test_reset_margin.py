"""Check point-gap category boundaries, identities and retained populations."""

from copy import deepcopy
from decimal import Decimal as D
import math
from pathlib import Path
import tempfile
import unittest

import analyze_reset_margin as a
import verify_reset_margin as v
from test_reset_projection import fixture as projection_fixture


def fixture(**kwargs):
    row = a.inherited.project([projection_fixture(**kwargs)])[0]
    return {key: "" if value is None else str(value) for key, value in row.items()}


def decimal_box(values):
    return v.Interval(*(D.from_float(float(value)) for value in values))


class PointGapDecomposition(unittest.TestCase):
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

    def test_blocked_raw_with_radial_clearance(self):
        r = self.calculate()
        self.assertEqual(r["point_gap_category"], "blocked_to_clear_radial_clear")
        self.assertEqual(
            (r["radial_gap_state"], r["rebuilt_mean_gap_state"]), ("clear", "clear")
        )
        self.assertAlmostEqual(r["radial_gap_center"], 0.8)

    def test_blocked_radial_gap_before_transverse_remainder(self):
        r = self.calculate(mean=(1.0, 1.0))
        self.assertEqual(r["point_gap_category"], "blocked_to_clear_radial_blocked")
        self.assertEqual(
            (r["radial_gap_state"], r["rebuilt_mean_gap_state"]), ("blocked", "clear")
        )
        self.assertAlmostEqual(r["radial_gap_center"], -0.2)
        self.assertGreater(r["remainder_gap_lo"], 0)

    def test_radial_unknown_is_retained_at_boundary(self):
        r = self.calculate(mean=(1.1, 1.0), margin=1.1)
        self.assertEqual(r["point_gap_category"], "blocked_to_clear_radial_unknown")
        self.assertEqual(r["radial_gap_state"], "unknown")
        self.assertLess(r["radial_gap_lo"], 0)
        self.assertGreater(r["radial_gap_hi"], 0)

    def test_original_clear_pair_is_already_clear(self):
        r = self.calculate(margin=0.1)
        self.assertEqual(r["point_gap_category"], "already_clear")
        self.assertEqual(r["radial_gap_state"], "clear")

    def test_other_original_states_remain_explicit(self):
        r = self.calculate(margin=3.0)
        self.assertEqual(r["point_gap_category"], "other_point_states")
        self.assertEqual(r["rebuilt_mean_gap_state"], "blocked")
        r = self.calculate(margin=1.0)
        self.assertEqual(r["point_gap_category"], "other_point_states")

    def test_all_original_state_pairs_have_frozen_category(self):
        for raw in a.STATES:
            for mean in a.STATES:
                for radial in a.STATES:
                    aa, vv = a.category(raw, mean, radial), v.category(
                        raw, mean, radial
                    )
                    self.assertEqual(aa, vv)
                    self.assertIn(aa, a.CATEGORIES)
        for fn in (a.category, v.category):
            with self.assertRaises(AssertionError):
                fn("invalid", "clear", "clear")

    def test_zero_and_near_zero_common_margins_preserve_identities(self):
        for margin in (0.0, 1e-13):
            r = self.calculate(margin=margin)
            self.assertEqual(r["margin_status"], "complete")
            self.assertLessEqual(abs(r["gap_identity_residual_center"]), 1e-10)

    def test_zero_gap_is_blocked_and_tiny_positive_lower_is_clear(self):
        for values, expected in (
            ((0, 0, 0), "blocked"),
            ((-1, -2, 0), "blocked"),
            ((1e-15, 0, 1), "unknown"),
            ((1e-15, 1e-16, 1), "clear"),
        ):
            self.assertEqual(a.state(a.box(*values)), expected)
            self.assertEqual(v.state(decimal_box(values)), expected)

    def test_both_incomplete_projection_statuses_retain_blank_quantities(self):
        for kwargs in ({"raw": (0.0, 0.0)}, {"mean": (-2.0, 0.0)}):
            r = self.calculate(**kwargs)
            self.assertEqual(r["margin_status"], "unresolved_projection")
            self.assertEqual(r["point_gap_category"], "unresolved_projection")
            self.assertEqual(
                (r["radial_gap_state"], r["rebuilt_mean_gap_state"]),
                ("unknown", "unknown"),
            )
            self.assertTrue(
                all(
                    r[q + "_" + side] is None
                    for q in a.QUANTITIES
                    for side in ("center", "lo", "hi")
                )
            )

    def test_first_chord_evidence_is_not_replaced_by_point_category(self):
        row = fixture(margin=0.1)
        row.update(
            stage="first_chord",
            decision_probe_origin="first_chord_by_exclusion",
            decision_direct_first_test_state="unsafe",
        )
        for m in (a, v):
            r = m.decompose([row])[0]
            self.assertEqual(r["point_gap_category"], "already_clear")
            self.assertEqual(r["decision_direct_first_test_state"], "unsafe")


class InvalidEvidence(unittest.TestCase):
    def test_nonfinite_reversed_and_negative_margin_bounds_are_rejected(self):
        row = fixture()
        for field, text in (
            ("radial_component_lo", "2"),
            ("witness_mean_current_gap_hi", "nan"),
            ("decision_point_current_margin_lo", "-1"),
        ):
            for m in (a, v):
                with self.assertRaises(AssertionError):
                    m.compute(dict(row, **{field: text}))

    def test_unknown_projection_status_is_rejected(self):
        row = dict(fixture(), projection_status="invalid")
        for m in (a, v):
            with self.assertRaises(AssertionError):
                m.compute(row)

    def test_common_margin_string_and_source_states_must_agree(self):
        row = fixture()
        for field, text in (
            ("common_margin", "1.200"),
            ("witness_raw_current_state", "clear"),
            ("witness_mean_current_state", "blocked"),
        ):
            for m in (a, v):
                with self.assertRaises(AssertionError):
                    m.compute(dict(row, **{field: text}))

    def test_corrupted_component_endpoint_or_distance_identity_is_rejected(self):
        row = fixture()
        for field in (
            "radial_component",
            "distance_gain_beyond_radial",
            "radial_endpoint",
            "reconstructed_distance",
        ):
            bad = row.copy()
            for side in ("center", "lo", "hi"):
                bad[field + "_" + side] = str(float(bad[field + "_" + side]) + 0.01)
            for m in (a, v):
                with self.assertRaises(AssertionError):
                    m.compute(bad)

    def test_zero_containment_cannot_be_replaced_by_center_tolerance(self):
        row = fixture()
        for field in (
            "witness_raw_current_gap",
            "witness_mean_current_gap",
            "radial_component",
            "distance_gain_beyond_radial",
            "radial_endpoint",
            "reconstructed_distance",
        ):
            c = float(row[field + "_center"]) + (
                5e-11 if field == "witness_mean_current_gap" else 0
            )
            for side in ("center", "lo", "hi"):
                row[field + "_" + side] = str(c)
        row["decision_point_current_margin_lo"] = row[
            "decision_point_current_margin_hi"
        ] = row["common_margin"]
        for m in (a, v):
            with self.assertRaises(AssertionError):
                m.compute(row)


class RetentionAndAccounting(unittest.TestCase):
    def test_every_original_string_and_margin_literal_is_preserved(self):
        row = fixture()
        row["common_margin"] = row["decision_point_current_margin"] = "01.2000"
        for m in (a, v):
            result = m.decompose([row])[0]
            self.assertTrue(all(result[key] == text for key, text in row.items()))
            self.assertEqual(result["common_margin"], "01.2000")
            self.assertEqual(result["literal"], "001.2300")

    def test_duplicate_edges_are_rejected(self):
        row = fixture()
        for m in (a, v):
            with self.assertRaises(AssertionError):
                m.decompose([row, row])

    def test_empty_cells_keep_zero_counts_and_blank_extrema(self):
        for m in (a, v):
            table = m.summaries(m.decompose([fixture()]))
            empty = next(
                r
                for r in table
                if r["case_selector"] == "all_cases"
                and r["origin_selector"] == "ambiguous_zero"
            )
            self.assertTrue(all(empty[f] == 0 for f in a.COUNTS))
            self.assertTrue(all(empty[f] is None for f in a.EXTREMA))

    def test_unresolved_rows_remain_in_summary_counts(self):
        for m in (a, v):
            table = m.summaries(m.decompose([fixture(raw=(0.0, 0.0))]))
            total = table[0]
            self.assertEqual(
                (
                    total["physical_resets"],
                    total["overlapping_endings"],
                    total["status_unresolved_projection"],
                    total["category_unresolved_projection"],
                ),
                (1, 3, 1, 1),
            )
            self.assertTrue(all(total[f] is None for f in a.EXTREMA))
            self.assertEqual(m.closure(table), 96)

    def test_interval_extrema_use_available_endpoints(self):
        for m in (a, v):
            r = m.decompose([fixture()])[0]
            first = dict(r, radial_gap_center=1, radial_gap_lo=0, radial_gap_hi=10)
            second = dict(
                r, case_id="C2", radial_gap_center=2, radial_gap_lo=1, radial_gap_hi=3
            )
            third = m.decompose(
                [dict(fixture(raw=(0.0, 0.0)), case_id="C3", edge_id="C3@5")]
            )[0]
            table = m.summaries([first, second, third])
            self.assertEqual(
                [
                    table[0][f]
                    for f in (
                        "min_radial_gap_lo",
                        "min_radial_gap_hi",
                        "max_radial_gap_lo",
                        "max_radial_gap_hi",
                    )
                ],
                [0, 3, 1, 10],
            )
            self.assertEqual(
                (table[0]["physical_resets"], table[0]["overlapping_endings"]), (3, 9)
            )
            self.assertEqual(m.closure(table), 128)

    def test_count_closures_and_category_totals_reject_changes(self):
        for m in (a, v):
            table = m.summaries(m.decompose([fixture()]))
            for field in (
                "overlapping_endings",
                "category_already_clear",
                "radial_clear",
                "status_complete",
            ):
                bad = deepcopy(table)
                bad[1][field] += 1
                with self.assertRaises(AssertionError):
                    m.closure(bad)

    def test_verifier_checks_exact_strings_and_rejects_partial_blank_boxes(self):
        actual, expected = a.decompose([fixture()]), v.decompose([fixture()])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rows.csv"
            a.write(path, actual, tuple(actual[0]))
            v.compare(path, expected, ("edge_id",))
            for key, text in (("literal", "1.23"), ("radial_gap_lo", "")):
                bad = [dict(actual[0], **{key: text})]
                a.write(path, bad, tuple(bad[0]))
                with self.assertRaises((AssertionError, ValueError)):
                    v.compare(path, expected, ("edge_id",))


if __name__ == "__main__":
    unittest.main()
