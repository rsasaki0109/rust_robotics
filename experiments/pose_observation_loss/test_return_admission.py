"""Test bounded scalar gates, original clocks and admission counter recurrence."""

import copy
import csv
from fractions import Fraction
import math
from pathlib import Path
import tempfile
import unittest

import analyze_return_admission as a
import verify_return_admission as v


def event(returned=True, end=True, cid="C1"):
    e = dict.fromkeys(a.BASE_FIELDS, "")
    e.update(
        dict(
            zip(
                a.META,
                (
                    cid,
                    "F1",
                    "fresh-white",
                    "epoch",
                    "random",
                    "277",
                    "open_50x50",
                    "JPS",
                    "fresh",
                    "fresh",
                    "white",
                ),
            )
        )
    )
    e.update(
        event_id="1",
        source_event_id="fresh_1",
        event_kind="received",
        event_time_s="150.0",
        scheduled="True",
        received="True",
        precision_activation="False",
        horizon="full",
        preview_loss="True",
        loss_operation="ttl_expiry",
        loss_time_s="150.02",
        returned=str(returned),
        return_time_s="155.0" if returned else "",
        ended=str(end) if returned else "",
        censored_return=str(not end) if returned else "",
        no_return=str(not returned),
        end_time_s="155.02" if returned and end else "",
        end_operation="ttl_expiry" if returned and end else "",
        ttl_limit_without_next_step="False",
        parent_rp_records="8",
        rp_records="2",
        rp_different_preview_records="1",
        anchor_category=(
            "ttl_ended"
            if returned and end
            else "returned_censored" if returned else "no_return"
        ),
    )
    return e


def point(n=7750, last=7750, **changes):
    row = {k: "false" for k in a.BOOLS}
    row.update({k: "0.000000000" for k in a.FLOATS})
    row.update(
        safety_paused="true",
        guard_active="true",
        stationary_epoch_fusion="true",
        epoch_fusion_available="true",
        recovery_sample_count="1",
        observation_age=format(n * 0.02 - last * 0.02, ".6f"),
        speed_margin="1.000000000",
        recovery_clearance="0.500000000",
        recovery_x="25.000000000",
        recovery_y="25.000000000",
        recovery_yaw="0.000000000",
        nominal_target_speed="0.400000000",
        release_preview_clearance_lower_bound="2.000000000",
        guard_release_margin="1.000000000",
    )
    row.update(changes)
    return a.decoded(row, n, last)


def decision(before=None, current=None):
    return a.decision(
        event(), 7750, before or point(7749, 7500), current or point(), "available"
    )


class RoundedConditions(unittest.TestCase):
    def test_strict_and_inclusive_boundaries_keep_unknowns(self):
        self.assertEqual(a.compare_interval((0, 0)), "fail")
        self.assertEqual(a.compare_interval((0, 0), inclusive=True), "pass")
        self.assertEqual(a.compare_interval(a.interval(0)), "unknown")
        self.assertEqual(a.compare_interval(a.interval(0), inclusive=True), "unknown")
        self.assertEqual(a.compare_interval((1, 2)), "pass")
        self.assertEqual(a.compare_interval((-2, -1)), "fail")

    def test_rounded_nominal_zero_differs_from_source_stop_latch_zero(self):
        r = decision(current=point(nominal_target_speed="0.000000000"))
        self.assertEqual(r["gate_nominal"], "unknown")
        self.assertIn("nominal", r["unknown_gates"])
        r = decision(
            current=point(nominal_target_speed="0.000000000", stop_latched="true")
        )
        self.assertEqual(r["gate_nominal"], "fail")
        self.assertEqual((r["nominal_gap_lo"], r["nominal_gap_hi"]), (0, 0))

    def test_simultaneous_failures_do_not_pick_one_cause(self):
        r = decision()
        self.assertEqual(r["gate_samples"], "fail")
        self.assertEqual(r["gate_clearance"], "fail")
        self.assertEqual(r["failed_gates"], "samples;clearance")
        self.assertEqual(r["certainty"], "fail")

    def test_clearance_and_inclusive_bounds_near_rounded_equality(self):
        r = decision(
            current=point(recovery_clearance="1.000000000", recovery_x="1.000000000")
        )
        self.assertEqual(r["gate_clearance"], "unknown")
        self.assertEqual(r["gate_bounds"], "unknown")
        r = decision(current=point(recovery_x="0.500000000"))
        self.assertEqual(r["gate_bounds"], "fail")

    def test_yaw_uncertainty_and_segment_endpoint_bounds(self):
        for yaw in ("0.000000000", "1.570796327", "-1.570796327", "3.141592654"):
            with self.subTest(yaw=yaw):
                ss = point(recovery_yaw=yaw)
                env = a.condition_intervals(ss)
                self.assertEqual(a.compare_interval(env["bounds"], True), "pass")
        ss = point(recovery_x="1.300000000", recovery_yaw="3.141592654")
        self.assertEqual(
            a.compare_interval(a.condition_intervals(ss)["bounds"], True), "fail"
        )

    def test_age_uses_exact_binary64_value_at_the_source_boundary(self):
        for age, state in (
            (math.nextafter(1.5, 0), "pass"),
            (1.5, "pass"),
            (math.nextafter(1.5, 2), "fail"),
        ):
            ss = point()
            ss["observation_age"] = 1.5
            ss["age_exact_s"] = age
            ss["age_exact_hex"] = age.hex()
            self.assertEqual(decision(current=ss)["gate_age"], state)

    def test_source_clock_and_shared_margin_contract(self):
        ss = point(7825, 7750)
        self.assertEqual(ss["age_exact_s"], 7825 * 0.02 - 7750 * 0.02)
        self.assertEqual(float.fromhex(ss["age_exact_hex"]), ss["age_exact_s"])
        with self.assertRaises(AssertionError):
            point(7825, 7750, observation_age="1.500001")
        self.assertEqual(len(a.source_contract()), 64)

    def test_guard_branch_has_only_two_applicable_conditions(self):
        before = point(7749, 7500, safety_paused="false")
        ss = point(
            safety_paused="false",
            recovery_sample_count="0",
            release_preview_clearance_lower_bound="0.5",
        )
        r = decision(before, ss)
        self.assertEqual(r["branch"], "guard")
        self.assertEqual(r["gate_age"], "not_applicable")
        self.assertEqual(r["gate_samples"], "not_applicable")
        self.assertEqual(r["failed_gates"], "release")

    def test_unknown_conjunction_stays_unknown_after_timer_evidence(self):
        ss = point(
            recovery_sample_count="25",
            recovery_clearance="2.000000000",
            nominal_target_speed="0.000000000",
        )
        r = decision(current=ss)
        self.assertEqual(r["certainty"], "unknown")
        self.assertFalse(r["ready_from_timer"])
        self.assertEqual(r["gate_nominal"], "unknown")


class Timers(unittest.TestCase):
    def test_joint_49th_tick_accumulates_and_50th_releases(self):
        before = point(recovery_evidence="0.96")
        ss = point(recovery_evidence="0.98")
        r = a.transition(before, ss, "joint")
        self.assertEqual(r["timer_action"], "accumulate")
        self.assertEqual(r["timer_after_ticks"], 49)
        before = ss
        ss = point(
            recovery_evidence="0",
            safety_paused="false",
            guard_active="false",
            resumed="true",
            guard_released="true",
        )
        r = a.transition(before, ss, "joint")
        self.assertEqual(r["timer_action"], "release")
        self.assertEqual(r["threshold_ticks"], 50)

    def test_guard_24th_tick_accumulates_and_25th_releases(self):
        before = point(safety_paused="false", guard_release_evidence="0.46")
        ss = point(safety_paused="false", guard_release_evidence="0.48")
        self.assertEqual(a.transition(before, ss, "guard")["timer_after_ticks"], 24)
        after = point(
            safety_paused="false", guard_active="false", guard_released="true"
        )
        self.assertEqual(a.transition(ss, after, "guard")["timer_action"], "release")

    def test_reset_is_distinct_from_waiting_at_zero(self):
        r = a.transition(point(recovery_evidence="0.46"), point(), "joint")
        self.assertEqual(r["timer_action"], "reset")
        self.assertFalse(r["ready_from_timer"])
        self.assertEqual(
            a.transition(point(), point(), "joint")["timer_action"], "stay_zero"
        )

    def test_carried_counter_and_new_pause_reset(self):
        r = a.transition(
            point(recovery_evidence="0.1"), point(recovery_evidence="0.12"), "joint"
        )
        self.assertEqual(r["timer_before_ticks"], 5)
        before = point(safety_paused="false", recovery_evidence="0.1")
        r = a.transition(before, point(recovery_evidence="0.02"), "joint")
        self.assertEqual(r["timer_before_ticks"], 0)

    def test_impossible_counter_and_release_records_fail(self):
        for before, after, branch in (
            (point(), point(recovery_evidence="0.04"), "joint"),
            (
                point(),
                point(
                    safety_paused="false",
                    guard_active="false",
                    guard_released="true",
                    resumed="true",
                ),
                "joint",
            ),
            (point(), point(guard_release_evidence="0.02"), "joint"),
            (
                point(safety_paused="false", guard_release_evidence="0.5"),
                point(safety_paused="false"),
                "guard",
            ),
        ):
            with self.subTest(branch=branch), self.assertRaises(AssertionError):
                a.transition(before, after, branch)

    def test_known_gate_certainty_must_agree_with_timer(self):
        with self.assertRaises(AssertionError):
            decision(current=point(recovery_evidence="0.02"))
        with self.assertRaises(AssertionError):
            decision(
                current=point(
                    recovery_sample_count="25", recovery_clearance="2.000000000"
                )
            )


class IndependentArithmetic(unittest.TestCase):
    def test_decimal_series_and_float_intervals_agree_at_boundaries(self):
        from decimal import Decimal

        sn, cs = v.sincos(Decimal("1.570796326794896619231321691639751442"))
        self.assertAlmostEqual(float(sn), 1, places=15)
        self.assertAlmostEqual(float(cs), 0, places=15)
        for changes in (
            {},
            {"recovery_x": "1.000000000"},
            {"recovery_yaw": "3.141592654"},
            {"nominal_target_speed": "0.000000000"},
            {"nominal_target_speed": "0.000000000", "stop_latched": "true"},
            {"release_preview_clearance_lower_bound": "1.000000000"},
        ):
            with self.subTest(changes=changes):
                ss = point(**changes)
                aa = a.condition_intervals(ss)
                bb = v.geometry(ss)
                for k in aa:
                    self.assertEqual(
                        a.compare_interval(aa[k], k == "bounds"),
                        v.classify_range(bb[k], k == "bounds"),
                    )
                    for x, y in zip(aa[k], bb[k]):
                        self.assertAlmostEqual(x, float(y), delta=2e-12)
                left = decision(current=ss)
                right = v.reconstruct_decision(
                    event(), 7750, point(7749, 7500), ss, "available"
                )
                self.assertEqual(left.keys(), right.keys())
                for k in left:
                    if isinstance(left[k], float):
                        self.assertAlmostEqual(left[k], right[k], delta=2e-12)
                    else:
                        self.assertEqual(left[k], right[k])

    def test_independent_enumeration_matches_both_counter_branches(self):
        for branch, limit in (("joint", 50), ("guard", 25)):
            field = (
                "recovery_evidence" if branch == "joint" else "guard_release_evidence"
            )
            for prior in (0, 1, limit - 2, limit - 1):
                for ready in (False, True):
                    release = ready and prior + 1 == limit
                    before = point(
                        safety_paused=str(branch == "joint").lower(),
                        **{field: str(prior / 50)}
                    )
                    changes = {
                        field: str((prior + 1) / 50 if ready and not release else 0),
                        "safety_paused": str(branch == "joint" and not release).lower(),
                        "guard_active": str(not release).lower(),
                        "guard_released": str(release).lower(),
                        "resumed": str(branch == "joint" and release).lower(),
                    }
                    after = point(**changes)
                    self.assertEqual(
                        a.transition(before, after, branch),
                        v.infer_timer(before, after, branch),
                    )

    def test_sql_runs_and_summary_are_independent(self):
        import sqlite3

        e = event()
        pp = {
            7749: point(7749, 7500),
            7750: point(),
            7751: point(7751, 7750, epoch_fusion_available="false"),
        }
        aa, dd, rr = a.build(e, pp, 7, 3)
        decisions = {(r["event_id"], str(a.step(r["time_s"]))): r for r in dd}
        with sqlite3.connect(":memory:") as db:
            other = v.maximal_runs(db, decisions, {"1": e})
        self.assertEqual(
            a.indexed(rr, ("event_id", "run")),
            {(k[0], int(k[1])): r for k, r in other.items()},
        )
        self.assertEqual(aa, v.anchor(e, dd, rr, 7, 3))
        rows = a.summaries([], [aa])
        independent = v.summaries([], {("1",): aa})
        self.assertEqual(a.indexed(rows, a.GROUP), independent)
        self.assertEqual(a.closure(rows), v.closures(independent))


class Ledgers(unittest.TestCase):
    def test_end_boundary_is_separate_from_available_decisions(self):
        e = event()
        pp = {
            7749: point(7749, 7500),
            7750: point(),
            7751: point(7751, 7750, epoch_fusion_available="false"),
        }
        r, dd, rr = a.build(e, pp, 7, 3)
        self.assertEqual((r["available_references"], r["next_loss_references"]), (1, 1))
        self.assertEqual([x["phase"] for x in dd], ["available", "next_loss"])
        self.assertEqual(sum(x["checkpoint_count"] for x in rr), 2)

    def test_endpoint_return_has_one_tick_and_unknown_future_failure(self):
        e = event(end=False)
        e["return_time_s"] = "180.0"
        pp = {8999: point(8999, 8750), 9000: point(9000, 9000)}
        r, dd, rr = a.build(e, pp, 7, 3)
        self.assertEqual(r["available_references"], 1)
        self.assertEqual(r["next_loss_references"], 0)
        self.assertIsNone(r["next_loss_failed_gates"])
        self.assertEqual(len(dd), 1)

    def test_absent_return_has_blank_admission_fields(self):
        r, dd, rr = a.build(event(returned=False), {}, 7, 3)
        self.assertTrue(all(r[k] is None for k in a.NEW))
        self.assertEqual((dd, rr), ([], []))

    def test_parent_weights_closure_and_nonmutation(self):
        e = event()
        pp = {
            7749: point(7749, 7500),
            7750: point(),
            7751: point(7751, 7750, epoch_fusion_available="false"),
        }
        saved = copy.deepcopy((e, pp))
        r, _, _ = a.build(e, pp, 7, 3)
        other, _, _ = a.build(dict(e, event_id="2", case_id="C2"), pp, 8, 4)
        c = {k: r[k] for k in a.META}
        c.update(selected_timeout="True", held="False")
        rows = a.summaries([c], [r, other])
        self.assertEqual(len(rows), 336)
        self.assertGreater(a.closure(rows), 0)
        full = next(
            x
            for x in rows
            if x["mode"] == "all_modes"
            and x["category_selector"] == "all"
            and x["partition"] == "fresh"
            and x["noise"] == "white"
            and x["variant"] == "epoch"
        )
        self.assertEqual(full["returned_case_mass"], Fraction(1, 7) + Fraction(1, 8))
        self.assertEqual(full["received_case_mass"], Fraction(1, 3) + Fraction(1, 4))
        self.assertEqual(full["rp_case_mass"], Fraction(1, 2))
        self.assertEqual((e, pp), saved)

    def test_empty_denominator_is_blank_and_observed_zero_is_zero(self):
        rows = a.summaries([], [])
        self.assertTrue(all(r["has_ready_tick_fraction"] is None for r in rows))
        self.assertTrue(all(r["max_available_counter_ticks"] is None for r in rows))

    def test_raw_grid_membership_and_clock_are_checked(self):
        e = event()
        c = {k: e[k] for k in a.META}
        c.update(selected_timeout="False", time_s="0.02")
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp) / "epoch-random-277/trajectories.csv"
            p.parent.mkdir()
            cells = {k: "false" for k in a.BOOLS}
            cells.update({k: "0" for k in a.FLOATS})
            cells.update(
                recovery_sample_count="0", scenario=c["scenario"], planner=c["planner"]
            )
            rows = [dict(cells, time_s=str(n / 50)) for n in (0, 1)]

            def save(rr):
                with p.open("w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=rows[0])
                    w.writeheader()
                    w.writerows(rr)

            def check():
                return a.collect(
                    [c],
                    [],
                    {
                        c["cohort"]: dict(
                            path=temp,
                            raw_sha256={
                                "epoch-random-277/trajectories.csv": a.digest(p)
                            },
                        )
                    },
                    {},
                )

            save(rows)
            self.assertEqual(check()[1]["raw_files"], 1)
            for rr in (
                rows[:1],
                [*rows, rows[-1]],
                [rows[0], dict(rows[1], planner="Other")],
            ):
                save(rr)
                with self.assertRaises(AssertionError):
                    check()


if __name__ == "__main__":
    unittest.main()
