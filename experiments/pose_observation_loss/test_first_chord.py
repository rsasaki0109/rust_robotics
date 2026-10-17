"""Test first-chord source intervals, native traces and preserved parent masses."""

import copy
from decimal import Decimal as D, localcontext
from fractions import Fraction
import itertools
import math
from pathlib import Path
import shutil
import tempfile
import unittest

import analyze_first_chord as a
import verify_first_chord as v
import test_probe_origin as f


def example(
    x="25.000000000",
    y="25.000000000",
    margin="0.500000000",
    delta="0.400000000",
    **changes
):
    before, current = f.inputs(x, y, margin)
    before["yaw"] = "0.000000000"
    current.update(
        yaw="0.000000000",
        noise_yaw="0.000000000",
        bias_yaw="0.000000000",
        nominal_steering=delta,
        steering="0.000000000",
    )
    current.update(changes)
    old = a.previous.build(f.old_decision(), before, current, [])
    return {k: str(value) for k, value in old.items()}, before, current


def ep_data(old, before, current):
    an = a.angles(before, current)
    ep = a.endpoint(old, an)
    trace = dict(
        endpoint_x=repr(ep["first_endpoint_x_center"]),
        endpoint_y=repr(ep["first_endpoint_y_center"]),
    )
    with localcontext() as ctx:
        ctx.prec = 60
        independent = v.endpoint(old, v.angles(before, current), trace)
    return ep, independent


class SourceArithmetic(unittest.TestCase):
    def test_interval_operations_enclose_negative_and_positive_products(self):
        for left, right in (((-2.0, 3.0), (4.0, 5.0)), ((-4.0, -1.0), (-3.0, 2.0))):
            for x, y in itertools.product(left, right):
                for function, expected in (
                    (a.add, x + y),
                    (a.sub, x - y),
                    (a.mul, x * y),
                ):
                    low, high = function(left, right)
                    self.assertLessEqual(low, expected)
                    self.assertGreaterEqual(high, expected)
        for division in (a.div, v.quotient):
            with self.assertRaises(AssertionError):
                division((-1, 1), (-1, 1))

    def test_trig_ranges_include_extrema_and_decimal_taylor_values(self):
        for center in (0.0, math.pi / 2, -math.pi / 2, math.pi, 99.0):
            interval = (center - 0.01, center + 0.01)
            for cosine in (False, True):
                low, high = a.trig(interval, cosine)
                for x in (interval[0], center, interval[1]):
                    actual = math.cos(x) if cosine else math.sin(x)
                    self.assertLessEqual(low, actual)
                    self.assertGreaterEqual(high, actual)
                s, c = v.sincos(D.from_float(center))
                self.assertAlmostEqual(float(s), math.sin(center), places=14)
                self.assertAlmostEqual(float(c), math.cos(center), places=14)

    def test_yaw_addition_order_and_nominal_steering(self):
        old, before, current = example(
            noise_yaw="0.000000001", bias_yaw="-16.000000000"
        )
        before["yaw"] = "16.000000000"
        angle = a.angles(before, current)
        self.assertEqual(angle["sensed_yaw_center"], (16.0 + 1e-9) - 16.0)
        self.assertEqual(angle["nominal_steering"], "0.400000000")
        self.assertNotEqual(angle["nominal_steering"], current["steering"])

    def test_invalid_angles_and_steering_are_rejected(self):
        for changes in (
            dict(noise_yaw="nan"),
            dict(bias_yaw="101.000000000"),
            dict(nominal_steering="0.613000000"),
        ):
            old, before, current = example(**changes)
            for fn in (a.angles, v.angles):
                with self.assertRaises(AssertionError):
                    fn(before, current)

    def test_near_zero_uses_coarse_enclosure_and_keeps_unknown(self):
        for delta in ("0.000000000", "0.000000100", "-0.000000100"):
            old, before, current = example(delta=delta)
            ep, independent = ep_data(old, before, current)
            self.assertTrue(ep["coarse_fallback"])
            self.assertTrue(independent["coarse_fallback"])
            self.assertGreater(ep["first_chord_radius"], 0.08)
            result = a.geometry(old, ep, [("1", 25.54, 25.0)])
            self.assertEqual(result["direct_first_test_state"], "unknown")

    def test_curved_endpoint_bounds_enclose_source_corner_evaluations(self):
        for delta in ("0.400000000", "-0.400000000", "0.000001000"):
            old, before, current = example(delta=delta)
            angle = a.angles(before, current)
            ep, independent = ep_data(old, before, current)
            self.assertFalse(ep["coarse_fallback"])
            for key in ep:
                if isinstance(ep[key], float):
                    self.assertAlmostEqual(
                        ep[key],
                        float(independent[key]),
                        places=9 if abs(float(delta)) < 0.001 else 11,
                    )
            for yaw, d in itertools.product(
                (angle["sensed_yaw_lo"], angle["sensed_yaw_hi"]),
                (angle["steering_lo"], angle["steering_hi"]),
            ):
                k = math.tan(d) / 0.4
                theta = yaw + k * 0.05
                x = 25 + (math.sin(theta) - math.sin(yaw)) / k
                y = 25 - (math.cos(theta) - math.cos(yaw)) / k
                self.assertLessEqual(ep["first_endpoint_x_lo"], x)
                self.assertGreaterEqual(ep["first_endpoint_x_hi"], x)
                self.assertLessEqual(ep["first_endpoint_y_lo"], y)
                self.assertGreaterEqual(ep["first_endpoint_y_hi"], y)


class DirectGeometry(unittest.TestCase):
    def test_segment_interior_projection_and_hausdorff_bound(self):
        start, end = (25.0, 25.0), (25.05, 25.001)
        obstacle = (25.025, 25.4)
        center = a.segment_distance(obstacle, start, end)
        with localcontext() as ctx:
            ctx.prec = 60
            precise = v.segment(
                tuple(D.from_float(x) for x in obstacle),
                tuple(D.from_float(x) for x in start),
                tuple(D.from_float(x) for x in end),
            )
        self.assertAlmostEqual(center, float(precise), places=12)
        radius = math.hypot(0.0002, 0.0002)
        for ax, ay, bx, by in itertools.product(
            (-0.0001, 0.0001), (-0.0001, 0.0001), (-0.0002, 0.0002), (-0.0002, 0.0002)
        ):
            displaced = a.segment_distance(
                obstacle, (start[0] + ax, start[1] + ay), (end[0] + bx, end[1] + by)
            )
            self.assertLessEqual(abs(displaced - center), radius + 1e-12)

    def test_first_obstacle_and_duplicate_witness_ids(self):
        old, before, current = example()
        ep, independent = ep_data(old, before, current)
        g = a.geometry(old, ep, [("1", 25.54, 25.0), ("2", 25.54, 25.0)])
        q = v.geometry(
            old, independent, [("1", D("25.54"), D(25)), ("2", D("25.54"), D(25))]
        )
        self.assertEqual(g["direct_first_failure_mask"], "obstacle_only")
        self.assertEqual(g["first_definite_obstacle_ids"], "1;2")
        self.assertEqual(g["direct_first_test_state"], q["direct_first_test_state"])

    def test_boundary_and_simultaneous_failures_are_kept(self):
        old, before, current = example(x="49.475000000")
        ep, _ = ep_data(old, before, current)
        g = a.geometry(old, ep, [])
        self.assertEqual(g["direct_first_failure_mask"], "boundary_only")
        self.assertEqual(g["first_definite_wall_ids"], "x_max")
        g = a.geometry(old, ep, [("1", 49.95, 25.2)])
        self.assertEqual(g["direct_first_failure_mask"], "both")

    def test_exact_contacts_remain_unknown_and_safe_results_fail(self):
        old, before, current = example(delta="0.000000000")
        ep, _ = ep_data(old, before, current)
        self.assertEqual(
            a.geometry(old, ep, [("1", 25.55, 25.0)])["direct_first_test_state"],
            "unknown",
        )
        with self.assertRaises(AssertionError):
            a.geometry(old, ep, [])

    def test_unaudited_fields_are_blank_and_prior_columns_do_not_change(self):
        old, before, current = example()
        prior = copy.deepcopy(old)
        audited = a.build(old, before, current, [("1", 25.54, 25.0)])
        for key in old:
            self.assertEqual(audited[key], old[key])
        self.assertEqual(old, prior)
        unaudited = dict(old, probe_origin="initial_unsafe")
        blank = a.build(unaudited)
        self.assertFalse(blank["first_chord_audited"])
        self.assertTrue(all(blank[k] is None for k in a.FIELDS))
        self.assertEqual(blank, v.build(unaudited))

    def test_wrong_raw_time_or_case_is_rejected(self):
        old, before, current = example()
        for changes in (dict(time_s="179.980000"), dict(planner="A*")):
            wrong = dict(current, **changes)
            with self.assertRaises(AssertionError):
                a.build(old, before, wrong, [("1", 25.54, 25.0)])


class OriginalMeasures(unittest.TestCase):
    def bases(self):
        e1, e2, e3 = (
            f.fixtures.event(),
            f.fixtures.event(False, rp=7),
            f.fixtures.event(),
        )
        e2["event_id"] = "2"
        e3.update(
            case_id="C2",
            family_id="F2",
            event_id="3",
            parent_rp_records="3",
            rp_records="3",
            rp_different_preview_records="3",
            census_case_weight="1.0",
            received_case_weight="1.0",
            rp_case_mass="1.0",
            rp_different_preview_case_mass="1.0",
        )
        old, before, current = example()
        other = dict(old, **{k: e3[k] for k in (*a.META, "event_id")})
        command = {
            k: a.flag(value) if value in ("True", "False") else value
            for k, value in f.old_decision().items()
        }
        othercommand = dict(command, **{k: e3[k] for k in (*a.META, "event_id")})
        command_bases = [
            a.previous.previous.anchor(e1, [command], 2, 2),
            a.previous.previous.anchor(e2, [], 2, 2),
            a.previous.previous.anchor(e3, [othercommand], 1, 1),
        ]
        bases = [
            a.previous.extend_anchor(base, dd)
            for base, dd in zip(command_bases, ([old], [], [other]))
        ]
        unsafe = a.build(old, before, current, [("1", 25.54, 25.0)])
        other_before, other_current = dict(before), dict(
            current, nominal_steering="0.000000000"
        )
        unknown = a.build(other, other_before, other_current, [("1", 25.54, 25.0)])
        return bases, [unsafe, unknown], (e1, e2, e3)

    def test_parent_weights_new_shares_and_closures(self):
        bases, (unsafe, unknown), events = self.bases()
        dd = ([unsafe], [], [unknown])
        ne = (2, 2, 1)
        anchors = [
            a.anchor(f.strings(base), rows, n, n)
            for base, rows, n in zip(bases, dd, ne)
        ]
        independent = [
            v.anchor(f.strings(base), rows, n, n)
            for base, rows, n in zip(bases, dd, ne)
        ]
        self.assertEqual(anchors, independent)
        self.assertEqual(sum(x["census_case_weight"] for x in anchors), 2)
        self.assertEqual(sum(x["rp_case_mass"] for x in anchors), 2)
        cases = [
            {**{k: e[k] for k in a.META}, "selected_timeout": "True", "held": "False"}
            for e in (events[0], events[2])
        ]
        oldsummary = [f.strings(r) for r in a.previous.summaries(cases, bases)]
        rows = a.summaries(oldsummary, anchors)
        expected = v.summaries(oldsummary, independent)
        self.assertEqual(a.closure(rows), v.closure(expected))
        cell = next(
            r
            for r in rows
            if tuple(r[k] for k in a.GROUP)
            == ("fresh", "white", "epoch", "random", "all")
        )
        self.assertEqual(
            cell["has_available_first_obstacle_only_fraction"], Fraction(1, 2)
        )
        self.assertEqual(
            cell["has_available_first_obstacle_only_case_mass_fraction"], Fraction(1, 3)
        )
        self.assertTrue(
            any(r["has_available_first_unknown_fraction"] is None for r in rows)
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "summary.csv"
            a.write(path, rows)
            v.compare(path, list(expected.values()), a.GROUP)
        broken = copy.deepcopy(rows)
        broken[rows.index(cell)]["available_joint_first_audited_refs"] += 1
        with self.assertRaises(AssertionError):
            a.closure(broken)

    def test_nonreturns_and_censored_endpoint_keep_prior_fields(self):
        bases, (unsafe, unknown), events = self.bases()
        empty = a.anchor(f.strings(bases[1]), [], 2, 2)
        self.assertTrue(all(empty[k] is None for k in (*a.REFS, *a.HAS)))
        returned = a.anchor(f.strings(bases[0]), [unsafe], 2, 2)
        self.assertEqual(returned["end_time_s"], "")
        self.assertEqual(returned["censored_return"], "True")
        self.assertEqual(returned["available_joint_first_audited_refs"], 1)

    def test_phase_branch_and_nominal_only_reset_counts(self):
        bases, (unsafe, unknown), events = self.bases()
        d = dict(
            unsafe,
            phase="next_loss",
            branch="guard",
            timer_action="reset",
            refined_failed_gates="nominal",
        )
        result = a.anchor(f.strings(bases[0]), [d], 2, 2)
        self.assertEqual(
            result["next_loss_guard_first_nominal_only_reset_direct_unsafe_refs"], 1
        )
        self.assertEqual(result["available_joint_first_audited_refs"], 0)


class NativeSource(unittest.TestCase):
    def test_numeric_enclosure_tolerance_does_not_relax_witness_ids(self):
        v.check_row(
            {"first_chord_radius": "0.0010000005"}, {"first_chord_radius": D("0.001")}
        )
        with self.assertRaises(AssertionError):
            v.check_row(
                {"first_chord_radius": "0.001000002"},
                {"first_chord_radius": D("0.001")},
            )
        with self.assertRaises(AssertionError):
            v.check_row(
                {"first_definite_obstacle_ids": "2"},
                {"first_definite_obstacle_ids": "1"},
            )

    def test_source_arithmetic_contract(self):
        source = (
            a.REPO / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
        ).read_text()
        self.assertEqual(a.source_contract(source), v.contract(source))
        for changed in (
            source.replace("curvature.abs() < 1e-9", "curvature.abs() < 1e-8"),
            source.replace(
                "const WHEELBASE: f64 = 0.4;", "const WHEELBASE: f64 = 0.5;"
            ),
        ):
            for checker in (a.source_contract, v.contract):
                with self.assertRaises(AssertionError):
                    checker(changed)

    def test_native_full_probe_and_direct_witness_cross_check(self):
        rustc = (
            shutil.which("rustc")
            or "/workspace/.cloud-env/rust_robotics/rustup/toolchains/1.99.0-x86_64-unknown-linux-gnu/bin/rustc"
        )
        if not Path(rustc).exists():
            self.skipTest("rustc unavailable")
        old, before, current = example()
        points = {(old["case_id"], 8999): before, (old["case_id"], 9000): current}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "map.csv"
            path.write_text("scenario,x,y\nopen_50x50,25.54,25\n")
            traces, proof = v.native(
                Path(tmp) / "native", Path(rustc), [old], points, path
            )
            self.assertEqual(proof["native_tests"], 6)
            self.assertEqual(proof["native_zero_centers"], 1)
            expected = v.build(
                old, before, current, [("1", D.from_float(25.54), D(25))], traces[0]
            )
            actual = a.build(old, before, current, [("1", 25.54, 25.0)])
            result = Path(tmp) / "decision.csv"
            a.write(result, [actual])
            v.compare(result, [expected], ("event_id", "time_s"))


if __name__ == "__main__":
    unittest.main()
