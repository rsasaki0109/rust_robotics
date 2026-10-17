"""Test rounded initial geometry, source-path exclusion and original measures."""

import copy
from decimal import Decimal
from fractions import Fraction
import math
from pathlib import Path
import shutil
import tempfile
import unittest

import analyze_probe_origin as a
import verify_probe_origin as v
import test_command_sign as fixtures


def inputs(x="25.000000000", y="25.000000000", margin="1.000000000", **changes):
    before = dict(time_s="179.980000", x=x, y=y)
    current = dict(
        fixtures.raw(0),
        time_s="180.000000",
        x="26.000000000",
        y="26.000000000",
        noise_x="0.000000000",
        noise_y="0.000000000",
        bias_x="0.000000000",
        bias_y="0.000000000",
        speed_margin=margin,
        scenario="open_50x50",
        planner="JPS",
    )
    current.update(changes)
    return before, current


def old_decision(**changes):
    return {k: str(value) for k, value in fixtures.refined(**changes).items()}


def decision(which="initial_unsafe", **changes):
    before, current = inputs()
    obstacles = [("1", 25.5, 25.0)] if which == "initial_unsafe" else []
    return a.build(old_decision(**changes), before, current, obstacles)


def strings(row):
    return {
        k: (
            ""
            if value is None
            else str(float(value)) if isinstance(value, Fraction) else str(value)
        )
        for k, value in row.items()
    }


class ReconstructedPose(unittest.TestCase):
    def test_preceding_truth_and_current_noise_bias_not_current_truth(self):
        before, current = inputs(
            x="10.000000000", noise_x="0.200000000", bias_x="0.300000000"
        )
        result = a.pose(before, current)
        self.assertEqual(result["sensed_x_center"], 10.5)
        self.assertEqual(result["prior_truth_time_s"], 179.98)
        self.assertEqual(result["observation_time_s"], 180.0)
        self.assertNotEqual(result["sensed_x_center"], float(current["x"]))
        self.assertAlmostEqual(
            result["sensed_x_hi"] - result["sensed_x_lo"], 6 * a.H + 2 * a.P, places=12
        )

    def test_two_addition_order_and_decimal_enclosure(self):
        before, current = inputs(
            x="16.000000000", noise_x="0.000000001", bias_x="-16.000000000"
        )
        result = a.pose(before, current)
        self.assertEqual(result["sensed_x_center"], (16.0 + 1e-9) - 16.0)
        independent = v.pose(before, current)
        self.assertLessEqual(
            independent["sensed_x_lo"], Decimal.from_float(result["sensed_x_center"])
        )
        self.assertGreaterEqual(
            independent["sensed_x_hi"], Decimal.from_float(result["sensed_x_center"])
        )

    def test_step_zero_noise_establishes_source_branch(self):
        for x, y in (("0.1", "0"), ("0", "0.01")):
            row = dict(effective_position_sigma=x, effective_yaw_sigma_rad=y)
            a.noisy_branch(row)
            v.noise_branch(row)
        for x, y in (("0", "0"), ("-0.1", "0"), ("nan", "0")):
            for fn in (a.noisy_branch, v.noise_branch):
                with self.assertRaises(AssertionError):
                    fn(dict(effective_position_sigma=x, effective_yaw_sigma_rad=y))

    def test_nonadjacent_rows_are_rejected(self):
        before, current = inputs()
        before["time_s"] = "179.960000"
        for fn in (a.pose, v.pose):
            with self.assertRaises(AssertionError):
                fn(before, current)

    def test_finite_coordinate_and_margin_guards(self):
        for changes in (
            dict(noise_x="nan"),
            dict(bias_y="101.000000000"),
            dict(speed_margin="0.000000000"),
            dict(speed_margin="10.000000000"),
        ):
            before, current = inputs(**changes)
            for fn in (a.pose, v.pose):
                with self.subTest(changes=changes), self.assertRaises(AssertionError):
                    fn(before, current)


class BoundedInitialTest(unittest.TestCase):
    def geometry(
        self, x="25.000000000", y="25.000000000", obstacles=(), margin="1.000000000"
    ):
        before, current = inputs(x, y, margin)
        result = a.geometry(a.pose(before, current), obstacles)
        decimal_obstacles = [
            (oid, Decimal.from_float(float(ox)), Decimal.from_float(float(oy)))
            for oid, ox, oy in obstacles
        ]
        independent = v.geometry(v.pose(before, current), decimal_obstacles)
        for key in result:
            if isinstance(result[key], float):
                self.assertAlmostEqual(result[key], float(independent[key]), places=11)
            else:
                self.assertEqual(result[key], independent[key])
        return result

    def test_safe_start_and_empty_obstacle_list(self):
        result = self.geometry()
        self.assertEqual(result["initial_test_state"], "safe")
        self.assertIsNone(result["obstacle_gap_lo"])
        self.assertEqual(result["obstacle_records_checked"], 0)

    def test_inclusive_walls_keep_rounded_contact_unknown(self):
        self.assertEqual(a.classify((0, 0)), "unsafe")
        self.assertEqual(v.side(0, 0), "unsafe")
        self.assertEqual(
            self.geometry(x="1.000000000")["initial_test_state"], "unknown"
        )
        self.assertEqual(self.geometry(x="0.999000000")["initial_test_state"], "unsafe")
        self.assertEqual(self.geometry(x="1.001000000")["initial_test_state"], "safe")

    def test_obstacle_thresholds_preserve_unknowns_and_original_ids(self):
        self.assertEqual(
            self.geometry(obstacles=[("2", 26.0, 25.0)])["initial_test_state"],
            "unknown",
        )
        result = self.geometry(obstacles=[("2", 25.5, 25.0), ("3", 25.5, 25.0)])
        self.assertEqual(result["initial_failure_mask"], "obstacle_only")
        self.assertEqual(result["definite_obstacle_ids"], "2;3")
        self.assertEqual(
            self.geometry(obstacles=[("2", 26.1, 25.0)])["initial_test_state"], "safe"
        )

    def test_all_ambiguous_witnesses_are_saved(self):
        result = self.geometry(x="1.000000000", obstacles=[("4", 2.0, 25.0)])
        self.assertEqual(result["initial_test_state"], "unknown")
        self.assertEqual(result["ambiguous_wall_ids"], "x_min")
        self.assertEqual(result["ambiguous_obstacle_ids"], "4")

    def test_simultaneous_wall_and_obstacle_failures_have_no_priority(self):
        result = self.geometry(
            x="0.500000000", obstacles=[("1", 1.0, 25.0), ("2", 1.0, 25.0)]
        )
        self.assertEqual(result["initial_failure_mask"], "both")
        self.assertEqual(result["definite_wall_ids"], "x_min")
        self.assertEqual(result["definite_obstacle_ids"], "1;2")

    def test_rectangle_corner_and_interior_distances_are_enclosed(self):
        before, current = inputs(noise_x="0.100000000", bias_y="-0.100000000")
        p = a.pose(before, current)
        g = a.geometry(p, [("1", 26.0, 25.0)])
        for x in (p["sensed_x_lo"], p["sensed_x_center"], p["sensed_x_hi"]):
            for y in (p["sensed_y_lo"], p["sensed_y_center"], p["sensed_y_hi"]):
                gap = math.hypot(x - 26, y - 25) - 1
                self.assertLessEqual(g["obstacle_gap_lo"], gap)
                self.assertGreaterEqual(g["obstacle_gap_hi"], gap)

    def test_nearby_filter_cannot_remove_a_possible_blocker(self):
        result = self.geometry(obstacles=[("far", 30.0, 25.0), ("near", 25.5, 25.0)])
        self.assertEqual(result["definite_obstacle_ids"], "near")

    def test_source_path_exclusion_and_positive_contradiction(self):
        for state, expected in (
            ("unsafe", "initial_unsafe"),
            ("safe", "first_chord_by_exclusion"),
            ("unknown", "ambiguous_zero"),
        ):
            self.assertEqual(a.origin("0.000000000", state), expected)
        self.assertEqual(a.origin("0.050000000", "safe"), "positive_probe")
        self.assertEqual(a.origin("0.050000000", "unknown"), "positive_probe")
        with self.assertRaises(AssertionError):
            a.origin("0.050000000", "unsafe")


class JoinsAndAccounting(unittest.TestCase):
    def test_raw_join_checks_identity_and_preserves_admission_columns(self):
        before, current = inputs()
        old = old_decision()
        before_copy, current_copy, old_copy = map(copy.deepcopy, (before, current, old))
        result = a.build(old, before, current, [("1", 25.5, 25.0)])
        independent = v.build(
            old, before, current, [("1", Decimal("25.5"), Decimal(25))]
        )
        for k, value in old.items():
            self.assertEqual(result[k], value)
        self.assertEqual(result["probe_origin"], independent["probe_origin"])
        self.assertEqual((before, current, old), (before_copy, current_copy, old_copy))
        current["planner"] = "A*"
        with self.assertRaises(AssertionError):
            a.build(old, before, current, [])

    def test_prior_unknowns_and_nominal_only_resets_are_counted_by_origin(self):
        d = decision(timer_action="reset", timer_before_ticks="1")
        e = fixtures.event()
        base = a.previous.anchor(
            e,
            [
                {
                    k: a.flag(value) if value in ("True", "False") else value
                    for k, value in old_decision(
                        timer_action="reset", timer_before_ticks="1"
                    ).items()
                }
            ],
            2,
            2,
        )
        got = a.extend_anchor(base, [d])
        self.assertEqual(got["available_joint_prior_unknown_initial_unsafe_refs"], 1)
        self.assertEqual(
            got["available_joint_nominal_only_reset_initial_unsafe_refs"], 1
        )
        self.assertEqual(got["available_joint_initial_obstacle_only_refs"], 1)
        expected = v.anchor(strings(base), [d], 2, 2)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "anchor.csv"
            a.write(path, [got])
            v.core.fields(a.read(path)[0], expected)

    def test_nonreturns_blank_new_fields_and_censored_endpoint_is_unchanged(self):
        base = a.previous.anchor(fixtures.event(False), [], 2, 2)
        got = a.extend_anchor(base, [])
        self.assertTrue(all(got[k] is None for k in (*a.REFS, *a.HAS)))
        d = decision()
        base = a.previous.anchor(
            fixtures.event(),
            [
                {
                    k: a.flag(value) if value in ("True", "False") else value
                    for k, value in old_decision().items()
                }
            ],
            2,
            2,
        )
        got = a.extend_anchor(base, [d])
        self.assertTrue(got["censored_return"])
        self.assertIsNone(got["end_time_s"])
        self.assertEqual(got["decision_references"], 1)

    def test_branch_and_phase_counts_do_not_collapse(self):
        d = decision()
        d["phase"] = "next_loss"
        d["branch"] = "guard"
        base = dict(returned=True, **{k: d[k] for k in a.META})
        got = a.extend_anchor(base, [d])
        self.assertEqual(got["next_loss_guard_initial_unsafe_refs"], 1)
        self.assertEqual(got["available_joint_initial_unsafe_refs"], 0)

    def test_original_unequal_measures_and_blank_shares_close(self):
        e1, e2, e3 = fixtures.event(), fixtures.event(False, rp=7), fixtures.event()
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
        d1, d3 = decision(), decision("first_chord_by_exclusion")
        d3.update({k: e3[k] for k in (*a.META, "event_id")})
        old1 = {
            k: a.flag(value) if value in ("True", "False") else value
            for k, value in old_decision().items()
        }
        old3 = dict(old1, **{k: e3[k] for k in (*a.META, "event_id")})
        bases = [
            a.previous.anchor(e1, [old1], 2, 2),
            a.previous.anchor(e2, [], 2, 2),
            a.previous.anchor(e3, [old3], 1, 1),
        ]
        anchors = [a.extend_anchor(b, d) for b, d in zip(bases, ([d1], [], [d3]))]
        self.assertEqual(sum(r["census_case_weight"] for r in anchors), 2)
        self.assertEqual(sum(r["rp_case_mass"] for r in anchors), 2)
        cases = [
            {**{k: e[k] for k in a.META}, "selected_timeout": "True", "held": "False"}
            for e in (e1, e3)
        ]
        rows = a.summaries(cases, anchors)
        cell = next(
            r
            for r in rows
            if tuple(r[k] for k in a.GROUP)
            == ("fresh", "white", "epoch", "random", "all")
        )
        self.assertEqual(cell["has_available_initial_unsafe_fraction"], Fraction(1, 2))
        self.assertEqual(
            cell["has_available_initial_unsafe_case_mass_fraction"], Fraction(1, 3)
        )
        self.assertTrue(
            any(r["has_available_initial_unsafe_fraction"] is None for r in rows)
        )
        oldsummary = [strings(r) for r in a.previous.summaries(cases, bases)]
        independent_anchors = [
            v.anchor(strings(b), dd, n, n)
            for b, dd, n in zip(bases, ([d1], [], [d3]), (2, 2, 1))
        ]
        independent = v.summaries(oldsummary, independent_anchors)
        self.assertEqual(a.closure(rows), v.closures(independent))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "summary.csv"
            a.write(path, rows)
            v.compare(path, list(independent.values()), a.GROUP)
        broken = copy.deepcopy(rows)
        broken[rows.index(cell)]["available_joint_initial_unsafe_refs"] += 1
        with self.assertRaises(AssertionError):
            a.closure(broken)

    def test_grid_gaps_and_unestablished_noise_branch_are_rejected(self):
        for times, sigma in ((("0.000000", "0.040000"), "0.1"), (("0.000000",), "0")):
            with tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp) / "epoch-random-277/trajectories.csv"
                path.parent.mkdir()
                import csv

                row = dict(
                    scenario="open_50x50",
                    planner="JPS",
                    time_s=times[0],
                    effective_position_sigma=sigma,
                    effective_yaw_sigma_rad="0",
                )
                with path.open("w", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=row, lineterminator="\n")
                    writer.writeheader()
                    for time in times:
                        writer.writerow(dict(row, time_s=time))
                case = {
                    **{k: old_decision()[k] for k in a.META},
                    "selected_timeout": "True",
                    "time_s": "180.0",
                }
                source = {
                    "fresh-white": {
                        "path": tmp,
                        "raw_sha256": {
                            "epoch-random-277/trajectories.csv": a.sha(path)
                        },
                    }
                }
                with self.assertRaises(AssertionError):
                    a.collect([case], [], source, {})


class SourceAndNative(unittest.TestCase):
    def test_source_guard_and_pose_order_are_checked(self):
        source = (
            a.REPO / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
        ).read_text()
        types = (a.REPO / a.TYPES).read_text()
        self.assertEqual(a.source_contract(source, types), v.contract(source, types))
        for changed in (
            source.replace(
                "p.distance(&start) <= margin", "p.distance(&start) < margin"
            ),
            source.replace("state.x + noise_x + bias_x", "state.x + bias_x + noise_x"),
        ):
            for fn in (a.source_contract, v.contract):
                with self.assertRaises(AssertionError):
                    fn(changed, types)

    def test_map_coordinates_use_binary64_and_keep_duplicate_records(self):
        ordinary = a.maps(a.MAP)
        independent = v.load_map(a.MAP)
        self.assertEqual(
            sum(map(len, ordinary.values())), sum(map(len, independent.values()))
        )
        for scenario, points in ordinary.items():
            for (oid, x, y), (iid, dx, dy) in zip(points, independent[scenario]):
                self.assertEqual(oid, iid)
                self.assertEqual(dx, Decimal.from_float(x))
                self.assertEqual(dy, Decimal.from_float(y))

    def test_native_initial_source_and_rounded_center_cross_check(self):
        rustc = (
            shutil.which("rustc")
            or "/workspace/.cloud-env/rust_robotics/rustup/toolchains/1.99.0-x86_64-unknown-linux-gnu/bin/rustc"
        )
        if not Path(rustc).exists():
            self.skipTest("rustc unavailable")
        rows = []
        for x, y, margin in (
            ("0.500000000", "25.000000000", "1.000000000"),
            ("25.000000000", "30.000000000", "0.500000000"),
            ("25.000000000", "25.000000000", "0.500000000"),
        ):
            before, current = inputs(x, y, margin)
            p = v.pose(before, current)
            rows.append(
                dict(
                    scenario="open_50x50",
                    **p,
                    **v.geometry(p, [("1", Decimal("25.5"), Decimal(25))])
                )
            )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "map.csv"
            path.write_text("scenario,x,y\nopen_50x50,25.5,25\n")
            proof = v.native(Path(tmp) / "native", Path(rustc), rows, path)
            self.assertEqual(proof["native_tests"], 5)
            self.assertEqual(proof["certain_center_comparisons"], 2)
            self.assertEqual(proof["unknown_center_rows"], 1)


if __name__ == "__main__":
    unittest.main()
