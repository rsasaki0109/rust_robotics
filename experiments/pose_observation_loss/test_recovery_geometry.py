"""Exercise source routing, rounded recovery geometry and original accounting."""

import copy
from decimal import Decimal as D
from fractions import Fraction
import itertools
import math
from pathlib import Path
import shutil
import tempfile
import unittest

import analyze_recovery_geometry as a
import verify_recovery_geometry as v
import test_probe_origin as fixtures
import test_command_sign as command


def example(x=24.0, y=25.0, yaw=math.pi, obstacles=None, samples=25):
    obs = obstacles if obstacles is not None else [("1", 25.3, 25.0)]
    before, current = fixtures.inputs(margin="0.500000000")
    old = fixtures.strings(
        fixtures.a.build(fixtures.old_decision(), before, current, [("1", 25.3, 25.0)])
    )
    ad = {
        k: old[k]
        for k in (*a.META, "event_id", "time_s", "speed_margin", "gate_clearance")
    }
    ad.update(
        recovery_x=f"{x:.9f}",
        recovery_y=f"{y:.9f}",
        recovery_yaw=f"{yaw:.9f}",
        recovery_sample_count=str(samples),
    )
    rx, ry, theta = (float(ad[k]) for k in ("recovery_x", "recovery_y", "recovery_yaw"))
    end = rx + 0.6 * math.cos(theta), ry + 0.6 * math.sin(theta)
    distance = min(
        a.prior.segment_distance((ox, oy), (rx, ry), end) for _, ox, oy in obs
    )
    ad["recovery_clearance"] = f"{distance:.9f}"
    ad["gate_clearance"] = "pass" if distance > 0.5 else "fail"
    old["gate_clearance"] = ad["gate_clearance"]
    current.update({k: ad[k] for k in a.INPUTS})
    trace = dict(
        endpoint_x=repr(end[0]), endpoint_y=repr(end[1]), clearance=repr(distance)
    )
    return old, ad, before, current, obs, trace


def calculate(*args, **kwargs):
    d, ad, before, current, obs, trace = example(*args, **kwargs)
    actual = a.geometry(d, ad, obs)
    precise = v.geometry(
        d, ad, [(i, D.from_float(x), D.from_float(y)) for i, x, y in obs], trace
    )
    for k, val in precise.items():
        if isinstance(val, D):
            assert math.isclose(
                float(actual[k]), float(val), rel_tol=1e-10, abs_tol=1e-9
            ), (k, actual[k], val)
        else:
            assert actual[k] == val, (k, actual[k], val)
    return actual


class SourceAndJoins(unittest.TestCase):
    def test_same_margin_and_distinct_pose_routes(self):
        source = (
            a.REPO / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
        ).read_text()
        self.assertEqual(a.contract(source), v.contract(source))
        for changed in (
            source.replace(
                "speed_free_distance(sensed, steering, speed_margin, obs)",
                "speed_free_distance(recovery, steering, speed_margin, obs)",
            ),
            source.replace(
                "let recovery_margin = RADIUS", "let recovery_margin = RADIUS + 0.01"
            ),
            source.replace(
                "recovery_x + 0.6 * recovery_yaw.cos()",
                "recovery_x + 0.7 * recovery_yaw.cos()",
            ),
            source.replace(
                "(sensed.x, sensed.y, sensed.yaw)", "(state.x, state.y, state.yaw)"
            ),
        ):
            for fn in (a.contract, v.contract):
                with self.assertRaises(AssertionError):
                    fn(changed)

    def test_numeric_serialization_join_and_metadata_identity(self):
        d, ad, *_ = example()
        d["target_speed"] = "0.000000000"
        ad["target_speed"] = "0.0"
        for fn in (a.join, v.admission_join):
            self.assertEqual(fn([d], [ad])[d["event_id"], d["time_s"]], ad)
            for field, value in (
                ("case_id", "Cother"),
                ("target_speed", "0.000000001"),
                ("gate_clearance", "fail"),
            ):
                with self.assertRaises(AssertionError):
                    fn([d], [dict(ad, **{field: value})])

    def test_missing_and_duplicate_join_keys_fail(self):
        d, ad, *_ = example()
        for fn in (a.join, v.admission_join):
            for rows in ([], [ad, ad], [dict(ad, time_s="179.98")]):
                with self.assertRaises(AssertionError):
                    fn([d], rows)

    def test_initial_and_first_chord_keep_their_original_ids(self):
        d, *_ = example()
        for fn in (a.blockers, v.witness_ids):
            self.assertEqual(fn(d), ["1"])
            first = dict(
                d,
                probe_origin="first_chord_by_exclusion",
                direct_first_test_state="unsafe",
                first_definite_obstacle_ids="7;8",
                first_definite_wall_ids="",
            )
            self.assertEqual(fn(first), ["7", "8"])
            for changed in (
                dict(first, direct_first_test_state="unknown"),
                dict(d, definite_obstacle_ids=""),
                dict(d, definite_wall_ids="x_min"),
                dict(d, definite_obstacle_ids="1;1"),
            ):
                with self.assertRaises(AssertionError):
                    fn(changed)


class Geometry(unittest.TestCase):
    def test_mean_start_and_segment_clear_the_original_obstacle(self):
        r = calculate()
        self.assertEqual(r["blocker_start_state"], "clear")
        self.assertEqual(r["blocker_segment_state"], "clear")
        self.assertEqual(r["allmap_state"], "clear")
        self.assertLess(r["pose_displacement_lo"], 1.0)
        self.assertGreater(r["pose_displacement_hi"], 1.0)

    def test_raw_fallback_retains_blocked_start(self):
        r = calculate(x=25.0, yaw=0.0, samples=0)
        self.assertEqual(r["recovery_sample_count"], "0")
        self.assertEqual(r["blocker_start_state"], "blocked")
        self.assertEqual(r["blocker_segment_state"], "blocked")
        self.assertEqual(r["pose_displacement_lo"], 0.0)

    def test_other_map_point_can_block_after_original_blocker_clears(self):
        r = calculate(obstacles=[("1", 25.3, 25.0), ("2", 23.7, 25.2)])
        self.assertEqual(r["blocker_segment_state"], "clear")
        self.assertEqual(r["allmap_state"], "blocked")
        self.assertEqual(r["recovery_definite_obstacle_ids"], "2")

    def test_duplicate_map_witnesses_survive(self):
        r = calculate(x=25.0, yaw=0.0, obstacles=[("1", 25.3, 25.0), ("2", 25.3, 25.0)])
        self.assertEqual(r["recovery_definite_obstacle_ids"], "1;2")

    def test_inclusive_contact_remains_rounding_unknown(self):
        r = calculate(x=25.0, yaw=0.0, obstacles=[("1", 25.3, 25.5)])
        self.assertEqual(r["blocker_segment_state"], "unknown")
        self.assertEqual(r["recovery_ambiguous_obstacle_ids"], "1")
        self.assertEqual(a.state((0, 0)), "blocked")
        self.assertEqual(v.classify(D(0), D(0)), "blocked")

    def test_segment_and_start_are_separate_predicates(self):
        r = calculate(x=24.0, yaw=0.0, obstacles=[("1", 24.6, 25.0)])
        self.assertEqual(r["blocker_start_state"], "clear")
        self.assertEqual(r["blocker_segment_state"], "blocked")

    def test_rotated_endpoint_rectangles_enclose_input_corners(self):
        for yaw in (0.0, math.pi / 2, -math.pi, 99.0):
            r = calculate(yaw=yaw)
            theta = float(r["recovery_yaw"])
            for dx, dy, dt in itertools.product((-a.H, a.H), repeat=3):
                x = 24 + dx + 0.6 * math.cos(theta + dt)
                y = 25 + dy + 0.6 * math.sin(theta + dt)
                self.assertLessEqual(r["recovery_endpoint_x_lo"], x)
                self.assertGreaterEqual(r["recovery_endpoint_x_hi"], x)
                self.assertLessEqual(r["recovery_endpoint_y_lo"], y)
                self.assertGreaterEqual(r["recovery_endpoint_y_hi"], y)

    def test_native_endpoint_and_clearance_membership_reject_corruption(self):
        d, ad, _, _, obs, trace = example()
        decimal = [(i, D.from_float(x), D.from_float(y)) for i, x, y in obs]
        for changed in (dict(trace, endpoint_x="0"), dict(trace, clearance="0")):
            with self.assertRaises(AssertionError):
                v.geometry(d, ad, decimal, changed)

    def test_nonfinite_and_out_of_domain_inputs_fail(self):
        d, ad, _, _, obs, trace = example()
        for key, value in (
            ("recovery_x", "nan"),
            ("recovery_yaw", "101"),
            ("recovery_sample_count", "26"),
        ):
            bad = dict(ad, **{key: value})
            with self.assertRaises(AssertionError):
                a.geometry(d, bad, obs)
            with self.assertRaises(AssertionError):
                v.geometry(
                    d,
                    bad,
                    [(i, D.from_float(x), D.from_float(y)) for i, x, y in obs],
                    trace,
                )

    def test_logged_clearance_and_inherited_gate_cannot_contradict(self):
        d, ad, _, _, obs, trace = example()
        for changed in (
            dict(ad, recovery_clearance="0"),
            dict(ad, gate_clearance="fail"),
        ):
            with self.assertRaises(AssertionError):
                a.geometry(d, changed, obs)
            with self.assertRaises(AssertionError):
                v.geometry(
                    d,
                    changed,
                    [(i, D.from_float(x), D.from_float(y)) for i, x, y in obs],
                    trace,
                )


class BuildingAndAccounting(unittest.TestCase):
    def test_raw_join_uses_previous_truth_and_exact_recovery_inputs(self):
        d, ad, before, current, obs, trace = example()
        result = a.build(d, ad, before, current, obs)
        expected = v.build(
            d,
            ad,
            before,
            current,
            [(i, D.from_float(x), D.from_float(y)) for i, x, y in obs],
            trace,
        )
        self.assertTrue(result["recovery_geometry_audited"])
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "rows.csv"
            a.write(path, [result])
            v.compare(path, [expected], ("event_id", "time_s"))
        for key, val in (("recovery_x", "24.000000001"), ("noise_x", "0.000000001")):
            with self.assertRaises(AssertionError):
                a.build(d, ad, before, dict(current, **{key: val}), obs)

    def test_positive_probes_have_blank_new_geometry(self):
        d, *_ = example()
        d["speed_free_distance"] = "0.050000000"
        for fn in (a.build, v.build):
            r = fn(d)
            self.assertFalse(r["recovery_geometry_audited"])
            self.assertTrue(all(r[k] is None for k in a.FIELDS))

    def test_phase_branch_and_nominal_subgroups_are_reference_counts(self):
        d, ad, before, current, obs, _ = example()
        d = a.build(d, ad, before, current, obs)
        d.update(
            phase="next_loss",
            branch="guard",
            prior_certainty="unknown",
            timer_action="reset",
            refined_failed_gates="nominal",
        )
        e = command.event()
        for fn in (a.anchor, v.anchor):
            r = fn(e, [d], 2, 2)
            self.assertEqual(r["next_loss_guard_recovery_prior_unknown_clear_refs"], 1)
            self.assertEqual(
                r["next_loss_guard_recovery_nominal_only_reset_clear_refs"], 1
            )
            self.assertEqual(r["available_joint_recovery_audited_refs"], 0)
            self.assertFalse(r["has_available_recovery_blockers_clear"])

    def test_nonreturn_fields_stay_blank_and_censoring_is_inherited(self):
        e = command.event(False)
        for fn in (a.anchor, v.anchor):
            r = fn(e, [], 2, 2)
            self.assertTrue(all(r[k] is None for k in (*a.REFS, *a.HAS)))
            self.assertEqual(r["census_case_weight"], Fraction(1, 2))
        d, ad, before, current, obs, _ = example()
        r = a.anchor(command.event(), [a.build(d, ad, before, current, obs)], 2, 2)
        self.assertEqual(r["censored_return"], "True")
        self.assertEqual(r["end_time_s"], "")

    def test_unequal_measures_empty_shares_and_additive_closures(self):
        d, ad, before, current, obs, _ = example()
        clear = a.build(d, ad, before, current, obs)
        events = [
            command.event(),
            dict(command.event(False), event_id="2"),
            dict(
                command.event(),
                event_id="3",
                case_id="C2",
                family_id="F2",
                census_case_weight="1.0",
                received_case_weight="1.0",
            ),
        ]
        other = dict(
            clear,
            **{k: events[2][k] for k in (*a.META, "event_id")},
            blocker_segment_state="blocked",
        )
        rows = ([clear], [], [other])
        aa = [a.anchor(e, dd, n, n) for e, dd, n in zip(events, rows, (2, 2, 1))]
        self.assertEqual(
            aa, [v.anchor(e, dd, n, n) for e, dd, n in zip(events, rows, (2, 2, 1))]
        )
        old = []
        for mode, cat in itertools.product(
            ("none", "random", "blackout", "all_modes"), ("all", *a.CATEGORIES)
        ):
            selected = [
                e
                for e in events
                if (mode in ("all_modes", e["mode"]))
                and (cat in ("all", e["anchor_category"]))
            ]
            old.append(
                dict(
                    zip(a.GROUP, ("fresh", "white", "epoch", mode, cat)),
                    census_case_mass=str(
                        float(sum(Fraction(e["census_case_weight"]) for e in selected))
                    ),
                )
            )
        rr = a.summaries(old, aa)
        ss = v.summaries(old, aa)
        self.assertEqual(a.closure(rr), v.closure(ss))
        cell = next(
            r for r in rr if r["mode"] == "random" and r["category_selector"] == "all"
        )
        self.assertEqual(
            cell["has_available_recovery_blockers_clear_fraction"], Fraction(1, 2)
        )
        self.assertEqual(
            cell["has_available_recovery_blockers_clear_case_mass_fraction"],
            Fraction(1, 3),
        )
        self.assertTrue(
            any(r["has_available_recovery_blockers_clear_fraction"] is None for r in rr)
        )
        broken = copy.deepcopy(rr)
        broken[rr.index(cell)]["available_joint_recovery_audited_refs"] += 1
        with self.assertRaises(AssertionError):
            a.closure(broken)

    def test_comparison_tolerance_keeps_witness_strings_exact(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "row.csv"
            a.write(
                p,
                [
                    dict(
                        id="1",
                        recovery_segment_radius="0.0010000005",
                        original_blocker_ids="1",
                    )
                ],
            )
            expected = dict(
                id="1", recovery_segment_radius=D("0.001"), original_blocker_ids="1"
            )
            v.compare(p, [expected], ("id",))
            with self.assertRaises(AssertionError):
                v.compare(p, [dict(expected, original_blocker_ids="2")], ("id",))

    def test_native_source_fragment_at_a_logged_pose(self):
        rustc = (
            shutil.which("rustc")
            or "/workspace/.cloud-env/rust_robotics/rustup/toolchains/1.99.0-x86_64-unknown-linux-gnu/bin/rustc"
        )
        if not Path(rustc).exists():
            self.skipTest("rustc unavailable")
        d, ad, before, current, obs, _ = example()
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "map.csv"
            p.write_text("scenario,x,y\nopen_50x50,25.3,25\n")
            traces, proof = v.native(
                Path(tmp) / "native",
                Path(rustc),
                [d],
                {(d["event_id"], d["time_s"]): ad},
                p,
            )
            self.assertEqual(proof["native_tests"], 6)
            self.assertEqual(proof["traced_centers"], 1)
            v.geometry(d, ad, [("1", D.from_float(25.3), D(25))], traces[0])


if __name__ == "__main__":
    unittest.main()
