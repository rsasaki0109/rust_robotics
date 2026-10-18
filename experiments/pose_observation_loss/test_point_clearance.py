"""Exercise signed geometry, fixed IDs, margins and original measures."""

from decimal import Decimal as D, localcontext
from fractions import Fraction
import itertools
import math
from pathlib import Path
import shutil
import tempfile
import unittest

import analyze_point_clearance as a
import verify_point_clearance as v
import test_window_turnover as fixtures


def context(x, y=0, mx=None, my=None, margin="1.000000000", count=2, n=26):
    p = fixtures.point(n, x, y)
    c = fixtures.context([p] * count, p)
    c["fields"].update(
        mean_x_center=x if mx is None else mx, mean_y_center=y if my is None else my
    )
    c["raw"] = {"speed_margin": margin}
    return c


def decision(ids="1", first=False):
    return dict(
        fixtures.decision(),
        probe_origin="first_chord_by_exclusion" if first else "initial_unsafe",
        definite_obstacle_ids="" if first else ids,
        definite_wall_ids="",
        first_definite_obstacle_ids=ids if first else "",
        first_definite_wall_ids="",
        direct_first_test_state="unsafe",
    )


def calculate(before, after, obstacles=(("1", 0.0, 0.0),), d=None):
    d = decision() if d is None else d
    actual, witnesses = a.geometry(d, before, after, obstacles)
    expected, other = v.geometry(d, before, after, obstacles)
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp) / "point.csv"
        a.write(p, [dict(id="1", **actual)])
        v.compare(p, [dict(id="1", **expected)], ("id",))
        a.write(p, witnesses)
        v.compare(p, other, ("event_id", "time_s", "obstacle_id"))
    return actual, witnesses, d


class SignedGeometry(unittest.TestCase):
    def test_raw_and_mean_can_move_in_opposite_directions(self):
        r, _, _ = calculate(context(2, mx=2), context(0.5, mx=3))
        self.assertEqual(r["raw_transition"], "clear_to_blocked")
        self.assertEqual(r["mean_transition"], "clear_to_clear")
        self.assertEqual(r["raw_distance_direction"], "toward")
        self.assertEqual(r["mean_distance_direction"], "away")

    def test_larger_margin_can_block_an_unchanged_point(self):
        r, _, _ = calculate(
            context(1.5, margin="1.000000000"), context(1.5, margin="2.000000000")
        )
        self.assertEqual(r["raw_transition"], "clear_to_blocked")
        self.assertEqual(r["point_margin_direction"], "increase")
        self.assertEqual(r["raw_distance_direction"], "unresolved")
        self.assertEqual(r["raw_gap_direction"], "narrower")
        self.assertAlmostEqual(r["raw_gap_change_center"], -1)

    def test_decreasing_margin_can_widen_gap_while_moving_toward(self):
        r, _, _ = calculate(
            context(2, margin="1.800000000"), context(1.5, margin="0.800000000")
        )
        self.assertEqual(r["raw_distance_direction"], "toward")
        self.assertEqual(r["raw_gap_direction"], "wider")
        self.assertEqual(r["point_margin_direction"], "decrease")
        self.assertAlmostEqual(r["raw_gap_change_center"], 0.5)

    def test_rectangle_bounds_include_all_corners_and_interior(self):
        for center, radius, obstacle in (
            ((3.0, 4.0), 0.2, (0.0, 0.0)),
            ((0.1, -0.1), 0.3, (0.0, 0.0)),
            ((0.0, 0.0), 0.0, (0.0, 0.0)),
        ):
            value, lo, hi = a.distance_box(center, radius, obstacle)
            dec = v.distance_box(center, D(str(radius)), obstacle)
            for x, y in itertools.product(
                *[(c - radius, c, c + radius) for c in center]
            ):
                dist = math.sqrt((x - obstacle[0]) ** 2 + (y - obstacle[1]) ** 2)
                self.assertLessEqual(lo, dist)
                self.assertGreaterEqual(hi, dist)
            self.assertAlmostEqual(value, float(dec[0]))
            self.assertAlmostEqual(lo, float(dec[1]))
            self.assertAlmostEqual(hi, float(dec[2]))
        self.assertEqual(a.distance_box((0.0, 0.0), 1.0, (0.0, 0.0))[1], 0)

    def test_gap_equality_is_inclusive_and_zero_overlap_is_unknown(self):
        for fn in (a.state, lambda p: v.gap_state(*p)):
            self.assertEqual(fn((-1, 0)), "blocked")
            self.assertEqual(fn((0, 1)), "unknown")
            self.assertEqual(fn((0, 0)), "blocked")
            self.assertEqual(fn((1, 2)), "clear")
        r, _, _ = calculate(context(2), context(1))
        self.assertEqual(r["raw_current_state"], "unknown")

    def test_change_zero_is_unresolved_including_endpoint_contact(self):
        for fn in (a.classify, v.signed):
            for pair in ((0, 0), (-1, 0), (0, 1), (-1, 1)):
                self.assertEqual(fn(pair, "minus", "plus"), "unresolved")
            self.assertEqual(fn((-2, -1), "minus", "plus"), "minus")
            self.assertEqual(fn((1, 2), "minus", "plus"), "plus")

    def test_same_logged_margin_keeps_rounding_uncertainty(self):
        r, _, _ = calculate(context(2), context(2))
        self.assertLess(r["point_margin_change_lo"], 0)
        self.assertGreater(r["point_margin_change_hi"], 0)
        self.assertEqual(r["point_margin_direction"], "unresolved")
        self.assertEqual(r["raw_gap_direction"], "unresolved")

    def test_minimum_can_switch_without_dropping_fixed_obstacles(self):
        r, ww, _ = calculate(
            context(1), context(9), (("1", 0.0, 0.0), ("2", 10.0, 0.0)), decision("1;2")
        )
        self.assertEqual(
            (r["raw_previous_minimum_ids"], r["raw_current_minimum_ids"]), ("1", "2")
        )
        self.assertEqual(r["fixed_obstacle_ids"], "1;2")
        self.assertEqual([w["obstacle_id"] for w in ww], ["1", "2"])
        self.assertEqual(ww[0]["raw_distance_direction"], "away")
        self.assertEqual(ww[1]["raw_distance_direction"], "toward")
        self.assertEqual(r["raw_distance_direction"], "unresolved")

    def test_duplicate_coordinates_keep_distinct_ids_and_ties(self):
        r, ww, _ = calculate(
            context(2),
            context(0.5),
            (("8", 0.0, 0.0), ("2", 0.0, 0.0)),
            decision("8;2"),
        )
        self.assertEqual([w["obstacle_id"] for w in ww], ["8", "2"])
        self.assertEqual(r["raw_current_minimum_ids"], "8;2")
        self.assertEqual(r["raw_current_possible_minimum_ids"], "8;2")

    def test_possible_minimum_tie_includes_rounding_overlap(self):
        r, _, _ = calculate(
            context(2),
            context(2),
            (("1", 0.0, 0.0), ("2", 1e-10, 0.0)),
            decision("1;2"),
        )
        self.assertEqual(r["raw_current_minimum_ids"], "2")
        self.assertEqual(r["raw_current_possible_minimum_ids"], "1;2")

    def test_extra_map_obstacle_does_not_enter_fixed_set(self):
        r, ww, _ = calculate(
            context(2), context(1.5), (("1", 0.0, 0.0), ("2", 1.5, 0.0))
        )
        self.assertEqual(len(ww), 1)
        self.assertEqual(r["raw_current_state"], "clear")

    def test_first_chord_blocker_does_not_force_point_to_be_blocked(self):
        r, _, d = calculate(context(2), context(1.1), d=decision(first=True))
        self.assertEqual(r["raw_current_state"], "clear")
        self.assertEqual(d["probe_origin"], "first_chord_by_exclusion")
        self.assertEqual(d["direct_first_test_state"], "unsafe")

    def test_empty_window_mean_uses_same_raw_point_and_radius(self):
        r, _, _ = calculate(context(2, count=0), context(0.5, count=0))
        for k in a.METRICS:
            if k.startswith("raw_"):
                self.assertEqual(r[k], r["mean_" + k[4:]])
        self.assertEqual(r["point_raw_previous_x_lo"], r["point_mean_previous_x_lo"])

    def test_invalid_or_duplicate_fixed_ids_are_rejected(self):
        for ids in ("", "1;1", "4"):
            for fn in (a.geometry, v.geometry):
                with self.assertRaises(AssertionError):
                    fn(decision(ids), context(2), context(0.5), (("1", 0.0, 0.0),))

    def test_decimal_native_gap_changes_are_enclosed(self):
        before, after = context(3, 4, margin="4.000000000"), context(
            0, 2, margin="1.000000000"
        )
        native = {
            "1": {
                (k, c): (D(dist), D(gap))
                for k in ("raw", "mean")
                for c, dist, gap in (("previous", 5, 1), ("current", 2, 1))
            }
        }
        fields, _ = v.geometry(decision(), before, after, (("1", 0.0, 0.0),), native)
        self.assertEqual(fields["raw_gap_direction"], "unresolved")
        self.assertLessEqual(abs(fields["raw_gap_closure_residual"]), v.P)
        native["1"]["raw", "current"] = (D(8), D(7))
        with self.assertRaises(AssertionError):
            v.geometry(decision(), before, after, (("1", 0.0, 0.0),), native)

    def test_raw_margin_string_is_preserved_and_compared_exactly(self):
        r, _, _ = calculate(context(2, margin="1.000000000"), context(0.5))
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "point.csv"
            a.write(p, [dict(id="1", **r)])
            other = dict(r, id="1", point_previous_margin="1.0")
            with self.assertRaises(AssertionError):
                v.compare(p, [other], ("id",))

    def test_source_contract_pins_margin_and_window_routing(self):
        source = (a.REPO / next(iter(v.CODE))).read_text()
        self.assertEqual(a.contract(source), v.contract(source))
        self.assertIn("common_margin_sha256", a.contract(source)["margin"])


class MeasuresAndNative(unittest.TestCase):
    def test_censoring_and_nonreturn_fields(self):
        r, _, d = calculate(context(2), context(0.5))
        d = dict(
            d, **r, point_clearance_audited=True, phase="next_loss", branch="guard"
        )
        for fn in (a.anchor, v.anchor):
            cell = fn(fixtures.fixtures.old.event(), [d], 2, 2)
            self.assertEqual(cell["next_loss_guard_point_raw_clear_to_blocked_refs"], 1)
            self.assertEqual(cell["censored_return"], "True")
            self.assertEqual(cell["end_time_s"], "")
            self.assertFalse(cell["has_available_point_raw_clear_to_blocked"])
            blank = fn(fixtures.fixtures.old.event(False), [], 2, 2)
            self.assertTrue(all(blank[k] is None for k in (*a.REFS, *a.HAS)))

    def test_unequal_original_measures_closures_and_empty_denominators(self):
        r, _, d = calculate(context(2), context(0.5))
        d = dict(d, **r, point_clearance_audited=True)
        events = [
            fixtures.fixtures.old.event(),
            dict(fixtures.fixtures.old.event(False), event_id="2"),
            dict(
                fixtures.fixtures.old.event(),
                event_id="3",
                case_id="C2",
                family_id="F2",
                census_case_weight="1.0",
                received_case_weight="1.0",
            ),
        ]
        other = dict(
            d,
            **{k: events[2][k] for k in (*a.META, "event_id")},
            raw_transition="blocked_to_blocked"
        )
        aa = [
            a.anchor(e, dd, n, n)
            for e, dd, n in zip(events, ([d], [], [other]), (2, 2, 1))
        ]
        self.assertEqual(
            aa,
            [
                v.anchor(e, dd, n, n)
                for e, dd, n in zip(events, ([d], [], [other]), (2, 2, 1))
            ],
        )
        cells = []
        for mode, cat in itertools.product(
            ("none", "random", "blackout", "all_modes"), ("all", *a.CATEGORIES)
        ):
            ee = [
                e
                for e in events
                if mode in ("all_modes", e["mode"])
                and cat in ("all", e["anchor_category"])
            ]
            cells.append(
                dict(
                    zip(a.GROUP, ("fresh", "white", "epoch", mode, cat)),
                    census_case_mass=str(
                        float(sum(Fraction(e["census_case_weight"]) for e in ee))
                    ),
                )
            )
        rr = a.summaries(cells, aa)
        ss = v.summaries(cells, aa)
        self.assertEqual(a.closure(rr), v.closure(ss))
        cell = next(
            r for r in rr if r["mode"] == "random" and r["category_selector"] == "all"
        )
        self.assertEqual(
            cell["has_available_point_raw_clear_to_blocked_fraction"], Fraction(1, 2)
        )
        self.assertEqual(
            cell["has_available_point_raw_clear_to_blocked_case_mass_fraction"],
            Fraction(1, 3),
        )
        self.assertTrue(
            any(
                r["has_available_point_raw_clear_to_blocked_fraction"] is None
                for r in rr
            )
        )
        cell["available_joint_point_audited_refs"] += 1
        with self.assertRaises(AssertionError):
            a.closure(rr)

    def test_native_original_distance_and_query_mapping(self):
        rustc = (
            shutil.which("rustc")
            or "/workspace/.cloud-env/rust_robotics/rustup/toolchains/1.99.0-x86_64-unknown-linux-gnu/bin/rustc"
        )
        if not Path(rustc).exists():
            self.skipTest("rustc unavailable")
        before, after = context(3, 4, n=25, margin="4.000000000"), context(0, 2, n=26)
        d = dict(decision(), scenario="fixture")
        contexts = {1: before, 2: after}
        traces = {
            i: {
                "mean_x": c["fields"]["mean_x_center"],
                "mean_y": c["fields"]["mean_y_center"],
            }
            for i, c in contexts.items()
        }
        with tempfile.TemporaryDirectory() as tmp:
            points, proof = v.native_points(
                Path(tmp),
                Path(rustc),
                [d],
                contexts,
                traces,
                {"fixture": [("1", D(0), D(0))]},
            )
            self.assertEqual(proof["native_tests"], 6)
            self.assertEqual(proof["traced_point_obstacle_pairs"], 4)
            self.assertEqual(
                points[d["event_id"], d["time_s"], "1", "mean", "previous"],
                (D(5), D(1)),
            )
            self.assertEqual(
                points[d["event_id"], d["time_s"], "1", "raw", "current"], (D(2), D(1))
            )
            queries = v.core.read(Path(tmp) / "native-point.csv")
            self.assertEqual(len(queries), 4)


if __name__ == "__main__":
    unittest.main()
