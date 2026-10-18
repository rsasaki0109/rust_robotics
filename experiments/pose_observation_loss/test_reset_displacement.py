"""Check spatial identities, interval signs, exact joins and summary accounting."""

from copy import deepcopy
from decimal import Decimal as D
import math
from pathlib import Path
import tempfile
import unittest

import analyze_reset_displacement as a
import verify_reset_displacement as v


def put(row, key, center, width=1e-9, nonnegative=False):
    row.update(
        {
            key + "_center": repr(center),
            key
            + "_lo": repr(max(0, center - width) if nonnegative else center - width),
            key + "_hi": repr(center + width),
        }
    )


def fixture(raw=(1.0, 0.0), mean=(2.0, 0.0), obstacle=(0.0, 0.0), margin=1.2):
    p = {key: "fixture" for key in a.META}
    p.update(
        case_id="C1",
        edge_id="C1@5",
        multiplicity="3",
        source_step="5",
        source_time_s="0.10",
        event_ids="1",
        decision_event_id="1",
        decision_phase="available",
        decision_branch="joint",
        decision_probe_origin="initial_unsafe",
        selected_blocker_ids="2",
        blocker_rows="1",
        stage="initial",
        literal="001.2300",
    )
    for kind, position in (("raw", raw), ("mean", mean)):
        for axis, value in zip(("x", "y"), position):
            put(p, "decision_point_" + kind + "_current_" + axis, value)
    put(p, "decision_point_current_margin", margin, 1e-12, nonnegative=True)
    # The source margin is a center string, with no _center suffix.
    p["decision_point_current_margin"] = p.pop("decision_point_current_margin_center")
    w = {key: p[key] for key in a.META}
    w.update(
        edge_id=p["edge_id"],
        multiplicity=p["multiplicity"],
        event_id="1",
        time_s="0.1",
        phase="available",
        branch="joint",
        probe_origin="initial_unsafe",
        obstacle_id="2",
        obstacle_order="1",
        map_x=repr(obstacle[0]),
        map_y=repr(obstacle[1]),
        obstacle_x=repr(obstacle[0]),
        obstacle_y=repr(obstacle[1]),
        literal="02.5000",
    )
    for kind, position in (("raw", raw), ("mean", mean)):
        d = math.sqrt(sum((x - y) ** 2 for x, y in zip(position, obstacle)))
        put(w, kind + "_current_distance", d, 1e-8, nonnegative=True)
        put(w, kind + "_current_gap", d - margin, 1e-8)
        w[kind + "_current_state"] = a.inherited.point_state(
            w[kind + "_current_gap_lo"], w[kind + "_current_gap_hi"]
        )
    return p, w


class SpatialComparison(unittest.TestCase):
    def calculate(self, **kwargs):
        p, w = fixture(**kwargs)
        aa, vv = a.compute(p, w), v.compute(p, w)
        self.assertEqual(set(aa), set(vv))
        for f in aa:
            if isinstance(vv[f], D):
                self.assertTrue(
                    math.isclose(aa[f], float(vv[f]), abs_tol=1e-12, rel_tol=1e-10), f
                )
            else:
                self.assertEqual(aa[f], vv[f], f)
        return aa

    def test_outward_identity(self):
        r = self.calculate()
        self.assertEqual(
            [
                r[q + "_center"]
                for q in ("linear_term", "quadratic_term", "squared_distance_change")
            ],
            [2, 1, 3],
        )
        self.assertEqual(
            (r["distance_gain_state"], r["linear_state"]), ("increase", "outward")
        )

    def test_tangential_increase_has_unknown_linear_sign(self):
        r = self.calculate(mean=(1.0, 1.0))
        self.assertAlmostEqual(r["distance_gain_center"], math.sqrt(2) - 1)
        self.assertEqual(r["linear_term_center"], 0)
        self.assertEqual(
            (r["distance_gain_state"], r["linear_state"]), ("increase", "unknown")
        )

    def test_inward_distance_decreases(self):
        r = self.calculate(mean=(0.5, 0.0))
        self.assertEqual(r["distance_gain_center"], -0.5)
        self.assertEqual(
            (r["distance_gain_state"], r["linear_state"]), ("decrease", "inward")
        )

    def test_overshoot_increases_distance_despite_inward_linear_term(self):
        r = self.calculate(mean=(-2.0, 0.0))
        self.assertEqual(
            [
                r[q + "_center"]
                for q in ("linear_term", "quadratic_term", "squared_distance_change")
            ],
            [-6, 9, 3],
        )
        self.assertEqual(
            (r["distance_gain_state"], r["linear_state"]), ("increase", "inward")
        )

    def test_zero_raw_distance_needs_no_division(self):
        r = self.calculate(raw=(0.0, 0.0), mean=(0.0, 2.0))
        self.assertEqual(r["linear_term_center"], 0)
        self.assertEqual(r["displacement_norm_center"], 2)
        self.assertEqual(r["distance_gain_state"], "increase")

    def test_zero_displacement_remains_unknown(self):
        r = self.calculate(mean=(1.0, 0.0))
        self.assertEqual(r["displacement_norm_center"], 0)
        self.assertEqual(r["displacement_norm_lo"], 0)
        self.assertEqual(
            (r["distance_gain_state"], r["linear_state"]), ("unknown", "unknown")
        )

    def test_translation_preserves_spatial_quantities(self):
        base = self.calculate()
        translated = self.calculate(
            raw=(4.0, 4.0), mean=(5.0, 4.0), obstacle=(3.0, 4.0)
        )
        for name in a.QUANTITIES:
            self.assertAlmostEqual(base[name + "_center"], translated[name + "_center"])

    def test_nonzero_and_near_zero_margins_cancel(self):
        for margin in (1.2, 1e-13, 0.0):
            r = self.calculate(margin=margin)
            self.assertAlmostEqual(r["gap_gain_center"], r["distance_gain_center"])
            self.assertLessEqual(r["margin_cancellation_residual_lo"], 0)
            self.assertGreaterEqual(r["margin_cancellation_residual_hi"], 0)

    def test_blocked_to_clear_flag_retains_original_pair(self):
        r = self.calculate()
        self.assertTrue(r["raw_blocked_mean_clear"])
        self.assertEqual(r["point_pair"], "blocked/clear")

    def test_first_chord_origin_is_preserved_with_clear_points(self):
        p, w = fixture(margin=0.1)
        p.update(decision_probe_origin="first_chord_by_exclusion", stage="first_chord")
        w["probe_origin"] = "first_chord_by_exclusion"
        for m in (a, v):
            r = m.join([p], [w])[0]
            self.assertEqual(r["stage"], "first_chord")
            self.assertEqual(r["point_pair"], "clear/clear")
            self.assertFalse(r["raw_blocked_mean_clear"])


class IntervalChecks(unittest.TestCase):
    def test_square_crosses_zero_and_keeps_nonnegative_lower(self):
        self.assertEqual(a.square(a.box(1, -2, 3)), (1, 0, 9 + a.P))
        b = v.Interval(D(1), D(-2), D(3)).squared()
        self.assertEqual((b.c, b.a, b.b), (D(1), D(0), D(9) + v.P))

    def test_negative_square_uses_nearest_endpoint(self):
        self.assertEqual(a.square(a.box(-2, -3, -1)), (4, 1 - a.P, 9 + a.P))
        b = v.Interval(D(-2), D(-3), D(-1)).squared()
        self.assertEqual((b.c, b.a, b.b), (D(4), D(1) - v.P, D(9) + v.P))

    def test_product_considers_all_endpoint_pairs(self):
        self.assertEqual(
            a.mul(a.box(0, -2, 3), a.box(0, -5, 7)), (0, -15 - a.P, 21 + a.P)
        )
        b = v.Interval(D(0), D(-2), D(3)) * v.Interval(D(0), D(-5), D(7))
        self.assertEqual((b.a, b.b), (D(-15) - v.P, D(21) + v.P))

    def test_unknown_sign_is_not_resolved_by_center_or_tolerance(self):
        self.assertEqual(a.sign(a.box(1e-15, 0, 1), "up", "down"), "unknown")
        self.assertEqual(
            v.Interval(D("1e-15"), D(0), D(1)).direction("up", "down"), "unknown"
        )
        self.assertEqual(a.sign(a.box(1e-15, 1e-16, 1), "up", "down"), "up")
        self.assertEqual(
            v.Interval(D("1e-15"), D("1e-16"), D(1)).direction("up", "down"), "up"
        )

    def test_nonfinite_reversed_or_excluded_centers_are_rejected(self):
        for values in (
            (0, 1, 2),
            (0, 1, -1),
            (float("nan"), -1, 1),
            (0, -1, float("inf")),
        ):
            with self.assertRaises(AssertionError):
                a.box(*values)
            with self.assertRaises(AssertionError):
                v.Interval(*(D.from_float(float(x)) for x in values))

    def test_corrupt_input_rectangle_is_rejected(self):
        p, w = fixture()
        for field, value in (
            ("decision_point_raw_current_x_lo", "2.0"),
            ("decision_point_mean_current_y_hi", "nan"),
        ):
            for m in (a, v):
                with self.assertRaises(AssertionError):
                    m.compute(dict(p, **{field: value}), w)

    def test_wrong_witness_distances_or_coordinates_are_rejected(self):
        p, w = fixture()
        for fields in (
            {"map_x": "1.0"},
            {"map_x": "1.0", "obstacle_x": "1.0"},
            {
                "raw_current_distance_center": "1.000001",
                "raw_current_distance_hi": "1.1",
            },
        ):
            for m in (a, v):
                with self.assertRaises(AssertionError):
                    m.compute(p, dict(w, **fields))

    def test_different_margin_or_state_is_rejected(self):
        p, w = fixture()
        for fields in (
            {"raw_current_state": "clear"},
            {
                "mean_current_gap_center": "0.7",
                "mean_current_gap_lo": "0.69",
                "mean_current_gap_hi": "0.71",
            },
        ):
            for m in (a, v):
                with self.assertRaises(AssertionError):
                    m.compute(p, dict(w, **fields))

    def test_center_tolerance_does_not_replace_zero_containment(self):
        p, w = fixture()
        for kind in ("raw", "mean"):
            for field in ("distance", "gap"):
                name = kind + "_current_" + field
                c = float(w[name + "_center"]) + (
                    5e-11 if kind == "mean" and field == "gap" else 0
                )
                put(w, name, c, 0)
        for m in (a, v):
            with self.assertRaises(AssertionError):
                m.compute(p, w)


class JoinsAndSummaries(unittest.TestCase):
    def test_exact_source_strings_are_preserved(self):
        p, w = fixture()
        for m in (a, v):
            r = m.join([p], [w])[0]
            self.assertTrue(all(r[k] == text for k, text in p.items()))
            self.assertTrue(all(r["witness_" + k] == text for k, text in w.items()))
            self.assertEqual(r["literal"], "001.2300")
            self.assertEqual(r["witness_literal"], "02.5000")
            self.assertEqual(r["common_margin"], p["decision_point_current_margin"])

    def test_duplicate_missing_or_extra_witnesses_are_rejected(self):
        p, w = fixture()
        for probes, witnesses in (
            ([p], []),
            ([p], [w, w]),
            ([p, p], [w]),
            ([p], [w, dict(w, edge_id="C2@5")]),
        ):
            for m in (a, v):
                with self.assertRaises(AssertionError):
                    m.join(probes, witnesses)

    def test_wrong_identity_event_step_and_obstacle_are_rejected(self):
        p, w = fixture()
        for field, text in (
            ("case_id", "C2"),
            ("event_id", "2"),
            ("time_s", "0.12"),
            ("obstacle_id", "3"),
            ("multiplicity", "2"),
            ("obstacle_order", "2"),
            ("probe_origin", "first_chord_by_exclusion"),
        ):
            for m in (a, v):
                with self.assertRaises(AssertionError):
                    m.join([p], [dict(w, **{field: text})])

    def test_empty_cells_keep_blank_extrema(self):
        p, w = fixture()
        for m in (a, v):
            table = m.summaries(m.join([p], [w]))
            self.assertEqual(len(table), 8)
            empty = next(
                r
                for r in table
                if r["case_selector"] == "all_cases"
                and r["origin_selector"] == "ambiguous_zero"
            )
            self.assertTrue(all(empty[f] == 0 for f in a.COUNTS))
            self.assertTrue(all(empty[f] is None for f in a.EXTREMA))

    def test_interval_extrema_are_not_selected_by_center(self):
        p, w = fixture()
        for m in (a, v):
            r = m.join([p], [w])[0]
            first = dict(
                r, distance_gain_center=1, distance_gain_lo=0, distance_gain_hi=10
            )
            second = dict(
                r,
                case_id="C2",
                distance_gain_center=2,
                distance_gain_lo=1,
                distance_gain_hi=3,
            )
            table = m.summaries([first, second])
            self.assertEqual(
                [
                    table[0][f]
                    for f in (
                        "min_distance_gain_lo",
                        "min_distance_gain_hi",
                        "max_distance_gain_lo",
                        "max_distance_gain_hi",
                    )
                ],
                [0, 3, 1, 10],
            )
            self.assertEqual(table[0]["physical_resets"], 2)
            self.assertEqual(table[0]["overlapping_endings"], 6)
            self.assertEqual(m.closure(table), 63)

    def test_additive_closure_rejects_changed_counts(self):
        p, w = fixture()
        for m in (a, v):
            table = m.summaries(m.join([p], [w]))
            bad = deepcopy(table)
            bad[1]["gain_increase"] += 1
            with self.assertRaises(AssertionError):
                m.closure(bad)

    def test_verifier_compares_copied_strings_exactly(self):
        p, w = fixture()
        aa, vv = a.join([p], [w]), v.join([p], [w])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rows.csv"
            a.write(path, aa, tuple(aa[0]))
            v.compare(path, vv, ("edge_id",))
            bad = [dict(aa[0], literal="1.23")]
            a.write(path, bad, tuple(bad[0]))
            with self.assertRaises(AssertionError):
                v.compare(path, vv, ("edge_id",))


if __name__ == "__main__":
    unittest.main()
