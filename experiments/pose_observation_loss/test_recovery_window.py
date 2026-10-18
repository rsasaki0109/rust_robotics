"""Test recovery-window predicates, circular means and inherited accounting."""

import copy
from decimal import Decimal as D
from fractions import Fraction
import itertools
import math
from pathlib import Path
import shutil
import tempfile
import unittest

import analyze_recovery_window as a
import verify_recovery_window as v
import test_command_sign as old


def initial(paused=True):
    return dict(
        safety_paused=str(paused).lower(),
        x="25.000000000",
        y="25.000000000",
        yaw="0.000000000",
    )


def row(n, count, **changes):
    result = dict(
        correction="false",
        correction_received="false",
        precision_activation="false",
        safety_paused="true",
        stationary_epoch_fusion="true",
        observation_age=f"{n*0.02:.6f}",
        measured_speed="0.00000000000000000e+00",
        recovery_sample_count=str(count),
        v="0.000000000",
    )
    result.update(
        {
            p + axis: "0.000000000"
            for p in ("noise_", "bias_")
            for axis in ("x", "y", "yaw")
        }
    )
    result.update(changes)
    return result


def pair(paused=True):
    return a.Window(initial(paused)), v.Replay(initial(paused))


def advance(both, n, count, **changes):
    r = row(n, count, **changes)
    before = initial()
    p = both[0].update(n, before, r)
    q, members, age, paused = both[1].advance(n, before, r)
    assert p == q and list(both[0].members) == members
    assert (
        both[0].last_clear == both[1].clear_step
        and both[0].clear_reasons == both[1].reasons
    )
    return r, p, members


def means(angles=(0.0, 0.0), changes=None):
    both = pair()
    current = None
    p = None
    members = None
    for n, yaw in enumerate(angles, 1):
        before = initial()
        before["yaw"] = f"{yaw:.9f}"
        current = row(n, n, **((changes or [{}] * len(angles))[n - 1]))
        p = both[0].update(n, before, current)
        q, members, _, _ = both[1].advance(n, before, current)
        assert p == q
    centers = {
        axis: a.fold(s["sensed_" + axis + "_center"] for s in members) / len(members)
        for axis in ("x", "y")
    }
    sn = a.fold(math.sin(s["sensed_yaw_center"]) for s in members)
    cs = a.fold(math.cos(s["sensed_yaw_center"]) for s in members)
    angle = math.atan2(sn, cs)
    current.update(
        recovery_x=f"{centers['x']:.9f}",
        recovery_y=f"{centers['y']:.9f}",
        recovery_yaw=f"{angle:.9f}",
    )
    trace = dict(
        mean_x=repr(centers["x"]),
        mean_y=repr(centers["y"]),
        mean_yaw=repr(angle),
        sample_count=str(len(members)),
    )
    actual, effective = a.reconstruct(both[0], p, current)
    independent, other = v.reconstruct(both[1], p, members, current, trace)
    assert effective == other
    with tempfile.TemporaryDirectory() as tmp:
        file = Path(tmp) / "mean.csv"
        a.write(file, [dict(id="1", **actual)])
        v.compare(file, [dict(id="1", **independent)], ("id",))
    return actual, effective, current, trace, both, p


class WindowPredicates(unittest.TestCase):
    def test_each_clear_reason_discards_all_previous_members(self):
        for change, reason in (
            (
                dict(
                    correction="true", correction_received="true", observation_age="0"
                ),
                "received",
            ),
            (dict(precision_activation="true", observation_age="0"), "precision"),
            (dict(correction="true"), "missed"),
            (dict(measured_speed="0.020000000000001"), "motion"),
            (dict(observation_age="1.520000"), "age"),
        ):
            both = pair()
            advance(both, 1, 1, safety_paused="false")
            n = 76 if reason == "age" else 2
            advance(both, n, int(reason == "missed"), safety_paused="false", **change)
            self.assertEqual(both[0].clear_reasons, reason)
            self.assertNotIn(1, [p["source_step"] for p in both[0].members])

    def test_simultaneous_reasons_and_clear_before_append(self):
        both = pair()
        advance(both, 1, 1)
        advance(
            both,
            2,
            1,
            correction="true",
            correction_received="true",
            precision_activation="true",
            observation_age="0.000000",
        )
        self.assertEqual(both[0].clear_reasons, "received;precision")
        self.assertEqual([p["source_step"] for p in both[0].members], [2])
        self.assertEqual(both[0].pushes, 1)

    def test_missed_observation_enters_pause_before_admission(self):
        both = pair(False)
        advance(both, 1, 1, correction="true", correction_received="false")
        self.assertEqual(both[0].clear_reasons, "missed")

    def test_inclusive_speed_and_age_thresholds(self):
        both = pair()
        advance(
            both,
            75,
            1,
            measured_speed="0.02000000000000000e+00",
            observation_age="1.500000",
        )
        advance(
            both,
            76,
            0,
            measured_speed="0.02000000000000001",
            observation_age="1.520000",
        )
        self.assertEqual(both[0].clear_reasons, "motion;age")

    def test_binary64_age_can_clear_when_printed_age_is_1_5(self):
        base = next(i for i in range(8926) if (i + 75) * 0.02 - i * 0.02 > 1.5)
        both = pair()
        both[0].last_observation = both[1].observation_step = base
        advance(both, base + 75, 0, observation_age="1.500000")
        self.assertEqual(both[0].clear_reasons, "age")

    def test_speed_uses_roundtrip_measurement_not_printed_v(self):
        both = pair()
        advance(both, 1, 0, measured_speed="0.020000000000001", v="0.000000000")
        self.assertEqual(both[0].clear_reasons, "motion")

    def test_pre_update_pause_survives_post_step_release(self):
        both = pair()
        advance(both, 1, 1, safety_paused="false")
        advance(both, 2, 1, safety_paused="false")
        self.assertEqual([p["source_step"] for p in both[0].members], [1])

    def test_capacity_preserves_latest_25_in_source_order(self):
        both = pair()
        for n in range(1, 27):
            advance(both, n, min(n, 25))
        self.assertEqual(
            [p["source_step"] for p in both[0].members], list(range(2, 27))
        )
        self.assertEqual((both[0].pushes, both[0].evictions), (26, 1))

    def test_membership_is_not_assumed_contiguous(self):
        both = pair()
        advance(both, 1, 1, safety_paused="false")
        advance(both, 2, 1, safety_paused="true")
        advance(both, 3, 2)
        self.assertEqual([p["source_step"] for p in both[0].members], [1, 3])

    def test_logged_count_contradiction_fails_both_readers(self):
        for obj in pair():
            with self.assertRaises(AssertionError):
                (obj.update if isinstance(obj, a.Window) else obj.advance)(
                    1, initial(), row(1, 0)
                )


class MeanGeometry(unittest.TestCase):
    def test_decimal_atan2_all_quadrants_and_argument_scales(self):
        for y, x in (
            (1, 1),
            (-1, 1),
            (1, -1),
            (-1, -1),
            (1, 0),
            (-1, 0),
            (0, -1),
            (1e-50, 1),
            (1, 1e-50),
        ):
            result = v.atan2(D(str(y)), D(str(x)))
            self.assertAlmostEqual(float(result), math.atan2(y, x), places=14)

    def test_circular_wrap_uses_short_arc(self):
        result, *_ = means((math.pi - 0.01, -math.pi + 0.01))
        self.assertEqual(result["mean_yaw_enclosure"], "bounded_circle")
        self.assertLess(a.circular_error(result["mean_yaw_center"], math.pi), 1e-9)

    def test_weak_resultant_retains_full_circle(self):
        result, *_ = means((0.0, math.pi))
        self.assertEqual(result["mean_yaw_enclosure"], "full_circle")
        self.assertEqual(result["mean_yaw_center"], 0.0)
        self.assertEqual(result["mean_yaw_radius"], math.pi)

    def test_mean_interval_covers_rounded_component_corners(self):
        result, effective, *_ = means(
            changes=[dict(noise_x="0.100000000"), dict(noise_x="-0.200000000")]
        )
        center = result["mean_x_center"]
        for sign in (-1, 1):
            perturbed = center + sign * (3 * a.H + 0.5 * a.P)
            self.assertLessEqual(result["mean_x_lo"], perturbed)
            self.assertGreaterEqual(result["mean_x_hi"], perturbed)
        self.assertEqual(len(effective), 2)

    def test_truth_noise_bias_decomposition_closes(self):
        result, *_ = means(
            changes=[
                dict(
                    noise_x="0.300000000", noise_y="-0.200000000", bias_x="0.010000000"
                ),
                dict(
                    noise_x="0.100000000", noise_y="0.200000000", bias_x="0.020000000"
                ),
            ]
        )
        self.assertAlmostEqual(result["noise_delta_x"], 0.1)
        self.assertAlmostEqual(result["noise_delta_y"], -0.2)
        self.assertAlmostEqual(result["bias_delta_x"], -0.005)
        self.assertEqual(result["truth_delta_norm"], 0.0)
        self.assertLessEqual(abs(result["decomposition_x_residual"]), a.P)

    def test_raw_fallback_has_no_retained_members_and_zero_components(self):
        both = pair(False)
        current, p, members = advance(both, 1, 0, safety_paused="false")
        current.update(
            recovery_x="25.000000000",
            recovery_y="25.000000000",
            recovery_yaw="0.000000000",
        )
        actual, effective = a.reconstruct(both[0], p, current)
        expected, _ = v.reconstruct(
            both[1],
            p,
            members,
            current,
            dict(mean_x="25", mean_y="25", mean_yaw="0", sample_count="0"),
        )
        self.assertEqual(actual["window_kind"], "fallback")
        self.assertEqual(actual["window_member_steps"], "")
        for c in ("truth", "noise", "bias"):
            self.assertEqual(actual[c + "_delta_norm"], 0)
        d = old.old(old.raw())
        line = a.lineage(d, actual, effective)
        self.assertEqual(line[0]["sample_role"], "raw_fallback")
        self.assertEqual(line[0]["sample_order"], 0)
        self.assertEqual(set(actual), set(expected))

    def test_wrong_logged_mean_and_native_trace_are_rejected(self):
        result, members, current, trace, both, p = means()
        for bad in (dict(current, recovery_x="24"), dict(current, recovery_yaw="1")):
            with self.assertRaises(AssertionError):
                a.reconstruct(both[0], p, bad)
            with self.assertRaises(AssertionError):
                v.reconstruct(both[1], p, members, bad, trace)
        with self.assertRaises(AssertionError):
            v.reconstruct(both[1], p, members, current, dict(trace, mean_x="24"))

    def test_nonfinite_and_unbounded_sensed_components_fail(self):
        for changes in (dict(noise_x="nan"), dict(bias_yaw="101")):
            current = row(1, 1, **changes)
            for fn in (a.sample, v.point):
                with self.assertRaises(AssertionError):
                    fn(1, initial(), current, 0)


class AccountingAndSource(unittest.TestCase):
    def test_source_contract_keeps_update_order_and_thresholds(self):
        source = (
            a.REPO / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
        ).read_text()
        self.assertEqual(a.contract(source), v.contract(source))
        for changed in (
            source.replace(
                "recovery_samples.len() > 25", "recovery_samples.len() > 24"
            ),
            source.replace("sensed.v.abs() > 0.02", "sensed.v.abs() >= 0.02"),
            source.replace(
                "recovery_samples.pop_front()", "recovery_samples.pop_back()"
            ),
        ):
            for fn in (a.contract, v.contract):
                with self.assertRaises(AssertionError):
                    fn(changed)

    def test_lineage_order_and_raw_strings_are_checked(self):
        fields, effective, *_ = means()
        d = old.old(old.raw())
        actual = a.lineage(d, fields, effective)
        expected = v.lineage(d, fields, effective)
        self.assertEqual(actual, expected)
        self.assertEqual([r["sample_order"] for r in actual], [1, 2])
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "lineage.csv"
            a.write(p, actual)
            v.compare(p, expected, ("event_id", "time_s", "sample_order"))
            bad = copy.deepcopy(expected)
            bad[0]["noise_x"] = "0.0"
            with self.assertRaises(AssertionError):
                v.compare(p, bad, ("event_id", "time_s", "sample_order"))

    def test_branch_phase_subgroups_and_nonreturns(self):
        fields, *_ = means()
        d = dict(
            old.old(old.raw()),
            **fields,
            window_audited=True,
            phase="next_loss",
            branch="guard",
            prior_certainty="unknown",
            timer_action="reset",
            refined_failed_gates="nominal",
        )
        for fn in (a.anchor, v.anchor):
            r = fn(old.event(), [d], 2, 2)
            self.assertEqual(r["next_loss_guard_window_prior_unknown_mean_refs"], 1)
            self.assertEqual(
                r["next_loss_guard_window_nominal_only_reset_mean_refs"], 1
            )
            self.assertFalse(r["has_available_window_mean"])
            blank = fn(old.event(False), [], 2, 2)
            self.assertTrue(all(blank[k] is None for k in (*a.REFS, *a.HAS)))
            self.assertEqual(blank["census_case_weight"], Fraction(1, 2))

    def test_unequal_measures_empty_shares_and_closures(self):
        fields, *_ = means()
        d = dict(
            old.old(old.raw()),
            **fields,
            window_audited=True,
            prior_certainty="unknown",
            refined_failed_gates="nominal",
        )
        events = [
            old.event(),
            dict(old.event(False), event_id="2"),
            dict(
                old.event(),
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
            window_kind="fallback",
            window_replayed_count=0,
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
        self.assertEqual(cell["has_available_window_mean_fraction"], Fraction(1, 2))
        self.assertEqual(
            cell["has_available_window_mean_case_mass_fraction"], Fraction(1, 3)
        )
        self.assertTrue(
            any(r["has_available_window_mean_fraction"] is None for r in rr)
        )
        cell["available_joint_window_mean_refs"] += 1
        with self.assertRaises(AssertionError):
            a.closure(rr)

    def test_native_original_fragment_replays_counts_and_means(self):
        rustc = (
            shutil.which("rustc")
            or "/workspace/.cloud-env/rust_robotics/rustup/toolchains/1.99.0-x86_64-unknown-linux-gnu/bin/rustc"
        )
        if not Path(rustc).exists():
            self.skipTest("rustc unavailable")
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            wire = work / "native-replay-inputs.tsv"
            wire.write_text(
                "case_id\tstep\tpaused\tcorrection\treceived\tprecision\tspeed\tage\tx\ty\tyaw\tlogged_count\trow_ids\nC1\t1\ttrue\tfalse\tfalse\tfalse\t0\t0.02\t1\t2\t0\t1\t\nC1\t2\ttrue\tfalse\tfalse\tfalse\t0\t0.04\t3\t2\t0\t2\t1\n"
            )
            d = dict(case_id="C1", event_id="1", time_s="0.04")
            traces, proof = v.native(work, Path(rustc), [d])
            self.assertEqual(proof["native_tests"], 9)
            self.assertEqual(proof["replayed_steps"], 2)
            self.assertEqual(float(traces[1]["mean_x"]), 2.0)


if __name__ == "__main__":
    unittest.main()
