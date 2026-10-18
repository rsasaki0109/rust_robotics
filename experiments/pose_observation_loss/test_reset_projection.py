"""Check projection identities, guarded denominators, signs and retained populations."""

from copy import deepcopy
from decimal import Decimal as D
import math
from pathlib import Path
import tempfile
import unittest

import analyze_reset_projection as a
import verify_reset_projection as v
from test_reset_displacement import fixture as displacement_fixture


def fixture(**kwargs):
    p, w = displacement_fixture(**kwargs)
    old = a.inherited.join([p], [w])[0]
    return {k: "" if value is None else str(value) for k, value in old.items()}


class ProjectionIdentities(unittest.TestCase):
    def calculate(self, **kwargs):
        row = fixture(**kwargs)
        aa, vv = a.compute(row), v.compute(row)
        self.assertEqual(set(aa), set(vv))
        for f in aa:
            if isinstance(vv[f], D):
                self.assertTrue(
                    math.isclose(aa[f], float(vv[f]), abs_tol=1e-12, rel_tol=1e-10), f
                )
            else:
                self.assertEqual(aa[f], vv[f], f)
        return aa

    def test_outward_projection_and_zero_remainder(self):
        r = self.calculate()
        self.assertEqual(r["radial_component_center"], 1)
        self.assertEqual(r["transverse_component_center"], 0)
        self.assertEqual(
            (r["radial_state"], r["transverse_state"], r["remainder_state"]),
            ("outward", "unknown", "unknown"),
        )
        self.assertEqual(r["projection_status"], "complete")
        self.assertLess(r["distance_gain_beyond_radial_lo"], 0)

    def test_inward_projection(self):
        r = self.calculate(mean=(0.5, 0.0))
        self.assertEqual(r["radial_component_center"], -0.5)
        self.assertEqual(r["reconstructed_distance_center"], 0.5)
        self.assertEqual(r["radial_state"], "inward")

    def test_tangential_projection_and_finite_remainder(self):
        r = self.calculate(mean=(1.0, 1.0))
        self.assertEqual(
            (r["radial_component_center"], r["transverse_component_center"]), (0, 1)
        )
        self.assertEqual(r["transverse_state"], "positive")
        self.assertAlmostEqual(
            r["distance_gain_beyond_radial_center"], math.sqrt(2) - 1
        )
        self.assertEqual(r["remainder_state"], "positive")

    def test_negative_transverse_orientation(self):
        r = self.calculate(mean=(1.0, -1.0))
        self.assertEqual(r["transverse_component_center"], -1)
        self.assertEqual(r["transverse_abs_center"], 1)
        self.assertEqual(r["transverse_state"], "negative")

    def test_mixed_projection_preserves_norm_and_distance(self):
        r = self.calculate(raw=(3.0, 4.0), mean=(5.0, 3.0))
        self.assertAlmostEqual(r["radial_component_center"], 0.4)
        self.assertAlmostEqual(r["transverse_component_center"], -2.2)
        self.assertAlmostEqual(r["projection_norm_squared_center"], 5)
        self.assertAlmostEqual(r["reconstructed_distance_center"], math.sqrt(34))

    def test_rotating_both_vectors_preserves_projection(self):
        r = self.calculate(raw=(0.0, 1.0), mean=(-1.0, 1.0))
        self.assertEqual(
            (r["radial_component_center"], r["transverse_component_center"]), (0, 1)
        )

    def test_small_inward_component_with_transverse_difference(self):
        r = self.calculate(mean=(0.999, 0.2))
        self.assertEqual(r["radial_state"], "inward")
        self.assertGreater(r["distance_gain_beyond_radial_center"], 0.001)
        self.assertEqual(r["remainder_state"], "positive")

    def test_zero_displacement_has_no_sign_inference(self):
        r = self.calculate(mean=(1.0, 0.0))
        self.assertEqual(r["projection_status"], "complete")
        self.assertEqual(
            [r[k + "_state"] for k in ("radial", "transverse", "remainder")],
            ["unknown"] * 3,
        )
        self.assertEqual(r["projection_norm_squared_center"], 0)

    def test_overshoot_keeps_distance_but_leaves_rational_formula_unresolved(self):
        r = self.calculate(mean=(-2.0, 0.0))
        self.assertEqual(r["projection_status"], "unresolved_remainder_denominator")
        self.assertEqual(r["radial_component_center"], -3)
        self.assertEqual(r["reconstructed_distance_center"], 2)
        self.assertEqual(r["distance_gain_beyond_radial_center"], 4)
        self.assertIsNone(r["rational_remainder_center"])
        self.assertIsNone(r["remainder_residual_center"])

    def test_mean_at_obstacle_retains_guarded_zero_denominator(self):
        r = self.calculate(mean=(0.0, 0.0))
        self.assertEqual(r["projection_status"], "unresolved_remainder_denominator")
        self.assertEqual(r["reconstructed_distance_center"], 0)
        self.assertIsNone(r["rational_remainder_center"])

    def test_zero_raw_distance_retains_row_with_unknown_projections(self):
        row = fixture(raw=(0.0, 0.0))
        for m in (a, v):
            result = m.project([row])[0]
            self.assertEqual(result["projection_status"], "unresolved_raw_distance")
            self.assertTrue(
                all(
                    result[q + "_" + s] is None
                    for q in a.QUANTITIES
                    for s in ("center", "lo", "hi")
                )
            )
            self.assertEqual(result["edge_id"], row["edge_id"])


class IntervalGuards(unittest.TestCase):
    def test_zero_included_in_raw_box_cannot_be_resolved_by_positive_center(self):
        row = dict(fixture(), witness_raw_current_distance_lo="0")
        for m in (a, v):
            self.assertEqual(
                m.compute(row)["projection_status"], "unresolved_raw_distance"
            )

    def test_division_uses_all_endpoint_quotients(self):
        b = a.divide(a.box(0, -2, 3), a.box(2, 1, 4))
        c = v.divide(v.Interval(D(0), D(-2), D(3)), v.Interval(D(2), D(1), D(4)))
        self.assertEqual(b, (0, -2 - a.P, 3 + a.P))
        self.assertEqual((c.c, c.a, c.b), (D(0), D(-2) - v.P, D(3) + v.P))

    def test_negative_nonzero_divisor_primitive(self):
        b = a.divide(a.box(2, 1, 3), a.box(-2, -4, -1))
        c = v.divide(v.Interval(D(2), D(1), D(3)), v.Interval(D(-2), D(-4), D(-1)))
        self.assertEqual(b, (-1, -3 - a.P, -0.25 + a.P))
        self.assertEqual((c.c, c.a, c.b), (D(-1), D(-3) - v.P, D("-0.25") + v.P))

    def test_zero_containing_divisor_is_rejected(self):
        for lo, hi in ((-1, 1), (0, 1), (-1, 0), (0, 0)):
            with self.assertRaises(AssertionError):
                a.divide(a.box(1, 1, 1), a.box(0, lo, hi))
            with self.assertRaises(AssertionError):
                v.divide(v.Interval(D(1), D(1), D(1)), v.Interval(D(0), D(lo), D(hi)))

    def test_positive_divisor_has_no_tolerance_cutoff(self):
        b = a.divide(a.box(1, 1, 1), a.box(1e-13, 1e-13, 1e-13))
        c = v.divide(v.Interval(D(1), D(1), D(1)), v.Interval.exact(1e-13))
        self.assertAlmostEqual(b[0] / 1e13, float(c.c) / 1e13)

    def test_absolute_bounds_crossing_zero(self):
        b = a.absolute(a.box(-1, -2, 3))
        c = v.absolute(v.Interval(D(-1), D(-2), D(3)))
        self.assertEqual(b, (1, 0, 3 + a.P))
        self.assertEqual((c.c, c.a, c.b), (D(1), D(0), D(3) + v.P))

    def test_absolute_negative_bounds(self):
        b = a.absolute(a.box(-2, -3, -1))
        c = v.absolute(v.Interval(D(-2), D(-3), D(-1)))
        self.assertEqual(b, (2, 1 - a.P, 3 + a.P))
        self.assertEqual((c.c, c.a, c.b), (D(2), D(1) - v.P, D(3) + v.P))

    def test_reversed_nonfinite_or_negative_distance_inputs_are_rejected(self):
        row = fixture()
        for field, text in (
            ("raw_offset_x_lo", "2"),
            ("delta_x_hi", "nan"),
            ("witness_raw_current_distance_lo", "-1"),
            ("distance_gain_center", "inf"),
        ):
            for m in (a, v):
                with self.assertRaises(AssertionError):
                    m.compute(dict(row, **{field: text}))

    def test_corrupted_norm_distance_or_gain_identity_is_rejected(self):
        row = fixture()
        for field in (
            "quadratic_term",
            "witness_mean_current_distance",
            "distance_gain",
        ):
            bad = row.copy()
            for side in ("center", "lo", "hi"):
                bad[field + "_" + side] = str(float(bad[field + "_" + side]) + 0.01)
            for m in (a, v):
                with self.assertRaises(AssertionError):
                    m.compute(bad)

    def test_residual_zero_containment_is_required_within_center_tolerance(self):
        row = fixture()
        for field in (
            "raw_offset_x",
            "raw_offset_y",
            "delta_x",
            "delta_y",
            "witness_raw_current_distance",
            "witness_mean_current_distance",
            "quadratic_term",
            "distance_gain",
        ):
            c = float(row[field + "_center"]) + (
                5e-11 if field == "distance_gain" else 0
            )
            for side in ("center", "lo", "hi"):
                row[field + "_" + side] = str(c)
        for m in (a, v):
            with self.assertRaises(AssertionError):
                m.compute(row)


class RetainedRowsAndSummaries(unittest.TestCase):
    def test_every_source_field_is_copied_exactly(self):
        row = fixture()
        for m in (a, v):
            projected = m.project([row])[0]
            self.assertTrue(all(projected[k] == text for k, text in row.items()))
            self.assertEqual(projected["literal"], "001.2300")

    def test_duplicate_edges_are_rejected(self):
        row = fixture()
        for m in (a, v):
            with self.assertRaises(AssertionError):
                m.project([row, row])

    def test_unresolved_rows_remain_in_counts_and_blank_extrema(self):
        row = fixture(raw=(0.0, 0.0))
        for m in (a, v):
            table = m.summaries(m.project([row]))
            self.assertEqual(
                (
                    table[0]["physical_resets"],
                    table[0]["overlapping_endings"],
                    table[0]["unresolved_raw_distance"],
                ),
                (1, 3, 1),
            )
            self.assertTrue(all(table[0][f] is None for f in a.EXTREMA))
            self.assertEqual(table[0]["radial_unknown"], 1)
            self.assertEqual(m.closure(table), 84)

    def test_empty_origin_cells_have_zero_counts_and_blank_extrema(self):
        for m in (a, v):
            table = m.summaries(m.project([fixture()]))
            empty = next(
                r
                for r in table
                if r["case_selector"] == "all_cases"
                and r["origin_selector"] == "ambiguous_zero"
            )
            self.assertTrue(all(empty[f] == 0 for f in a.COUNTS))
            self.assertTrue(all(empty[f] is None for f in a.EXTREMA))

    def test_extrema_use_available_intervals_not_center_row_or_unresolved_zero(self):
        for m in (a, v):
            r = m.project([fixture()])[0]
            first = dict(
                r,
                radial_component_center=1,
                radial_component_lo=0,
                radial_component_hi=10,
            )
            second = dict(
                r,
                case_id="C2",
                radial_component_center=2,
                radial_component_lo=1,
                radial_component_hi=3,
            )
            unresolved = m.project(
                [dict(fixture(raw=(0.0, 0.0)), edge_id="C3@5", case_id="C3")]
            )[0]
            table = m.summaries([first, second, unresolved])
            self.assertEqual(
                [
                    table[0][f]
                    for f in (
                        "min_radial_component_lo",
                        "min_radial_component_hi",
                        "max_radial_component_lo",
                        "max_radial_component_hi",
                    )
                ],
                [0, 3, 1, 10],
            )
            self.assertEqual(table[0]["physical_resets"], 3)
            self.assertEqual(table[0]["overlapping_endings"], 9)
            self.assertEqual(m.closure(table), 112)

    def test_closure_rejects_changed_status_and_partition_counts(self):
        for m in (a, v):
            table = m.summaries(m.project([fixture()]))
            for field in ("complete", "overlapping_endings"):
                bad = deepcopy(table)
                bad[1][field] += 1
                with self.assertRaises(AssertionError):
                    m.closure(bad)

    def test_verifier_rejects_changed_source_string_and_partial_blank_box(self):
        row = fixture()
        actual, expected = a.project([row]), v.project([row])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rows.csv"
            a.write(path, actual, tuple(actual[0]))
            v.compare(path, expected, ("edge_id",))
            for key, text in (("literal", "1.23"), ("radial_component_lo", "")):
                bad = [dict(actual[0], **{key: text})]
                a.write(path, bad, tuple(bad[0]))
                with self.assertRaises((AssertionError, ValueError)):
                    v.compare(path, expected, ("edge_id",))


if __name__ == "__main__":
    unittest.main()
