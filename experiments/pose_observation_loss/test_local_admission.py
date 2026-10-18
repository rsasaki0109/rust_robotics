"""Exercise source gates, actual counter transitions, local runs and native replay."""

from fractions import Fraction
import itertools
import math
from pathlib import Path
import shutil
import tempfile
import unittest

import analyze_local_admission as a
import verify_local_admission as v
import test_command_sign as commands
import test_point_history as histories


def fixture(
    k=1,
    stop=False,
    branch="joint",
    before_ticks=0,
    n=7750,
    last=7740,
    ready_override=None,
    **changes
):
    raw = {k: "false" for k in a.admission.BOOLS}
    raw.update({k: "0.000000000" for k in a.admission.FLOATS})
    raw.update(commands.raw(k, stop))
    raw.update(
        stationary_epoch_fusion="true",
        epoch_fusion_available="true",
        recovery_sample_count="10",
        observation_age=format(n * 0.02 - last * 0.02, ".6f"),
        speed_margin="1.000000000",
        recovery_clearance="2.000000000",
        recovery_x="25.000000000",
        recovery_y="25.000000000",
        recovery_yaw="0.000000000",
        release_preview_clearance_lower_bound="2.000000000",
        guard_release_margin="1.050000000",
        resumed="false",
        guard_released="false",
        guard_triggered="false",
    )
    paused = branch == "joint"
    guarded = branch in ("joint", "guard")
    raw.update(safety_paused=str(paused).lower(), guard_active=str(guarded).lower())
    raw.update(changes)
    speed = abs(float(raw["measured_speed"])) <= 0.02
    release = float(raw["release_preview_clearance_lower_bound"]) > float(
        raw["guard_release_margin"]
    )
    margin = float(raw["speed_margin"])
    x = float(raw["recovery_x"])
    y = float(raw["recovery_y"])
    yaw = float(raw["recovery_yaw"])
    inside = all(
        margin <= c <= 50 - margin
        for c in (x, y, x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw))
    )
    ready = speed and release
    if paused:
        ready &= (
            (n * 0.02 - last * 0.02) <= 1.5
            and int(raw["recovery_sample_count"]) >= 10
            and float(raw["recovery_clearance"]) > margin
            and inside
            and float(raw["nominal_target_speed"]) > 0
        )
    if branch == "not_applicable":
        ready = False
    if ready_override is not None:
        ready = ready_override
    threshold = 50 if paused else 25
    released = ready and before_ticks + 1 == threshold
    after = 0 if not ready or released else before_ticks + 1
    raw["recovery_evidence"] = format((after if paused else 0) / 50, ".6f")
    raw["guard_release_evidence"] = format(
        (after if branch == "guard" else 0) / 50, ".6f"
    )
    if released:
        raw.update(guard_active="false", guard_released=str(guarded).lower())
        if paused:
            raw.update(resumed="true", safety_paused="false")
    raw["target_speed"] = (
        "0.000000000"
        if raw["safety_paused"] == "true" or raw["guard_active"] == "true"
        else raw["nominal_target_speed"]
    )
    before = dict(
        recovery_evidence=format((before_ticks if paused else 0) / 50, ".6f"),
        guard_release_evidence=format(
            (before_ticks if branch == "guard" else 0) / 50, ".6f"
        ),
        safety_paused=str(paused).lower(),
        guard_active=str(guarded).lower(),
    )
    snap = histories.fixtures.context(
        25, 25, count=int(raw["recovery_sample_count"]), n=n
    )
    snap.update(
        raw=raw, before_raw=before, last_observation_step=last, clear_step=0, reasons=""
    )
    e = histories.old_events.event()
    e["return_time_s"] = "155.0"
    old = dict(
        **{k: e[k] for k in (*a.META, "event_id")},
        reference_time_s="155.0",
        offset_steps=0,
        source_step=str(n),
        source_time_s=str(n / 50),
        slot_status="observed",
        context_phase="reference",
        logged_speed_margin=raw["speed_margin"],
        window_count=str(len(snap["members"])),
        last_clear_step="0",
        last_clear_reasons="",
        raw_state="clear",
        mean_state="clear"
    )
    old.update({"input_" + k: snap["point"][k] for k in a.prior.RAW})
    return old, e, snap


def calculate(**kwargs):
    old, e, snap = fixture(**kwargs)
    actual = a.local_slot(old, e, snap, a.command.lattice())
    lookup = {row["speed_free_distance"]: row for row in a.command.lattice()}
    expected = v.local_slot(old, e, snap, lookup)
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp) / "slots.csv"
        a.write(p, [dict(id="1", **actual)])
        v.compare(p, [dict(id="1", **expected)], ("id",))
    return dict(old, local_audited=True, **actual), e, snap


def context_slots():
    source, e, snap = calculate(k=0)
    rows = []
    for offset in range(-25, 26):
        row = dict(
            source,
            offset_steps=offset,
            source_step=str(7750 + offset),
            source_time_s=str((7750 + offset) / 50),
            context_phase="reference" if offset == 0 else "available_context",
        )
        rows.append(row)
    return rows, e


def pass_at(rows, offset, branch="joint", before=0, action="accumulate"):
    s = rows[offset + 25]
    s.update(
        local_branch=branch,
        local_source_certainty="pass",
        local_timer_before_ticks=before,
        local_timer_after_ticks=0 if action == "release" else before + 1,
        local_threshold_ticks=50 if branch == "joint" else 25,
        local_timer_action=action,
        local_ready_from_timer=True,
        local_source_nominal_positive=True,
        local_failed_gates="",
        local_unknown_gates="",
    )
    return s


def decision(e, first=""):
    return dict(
        e,
        time_s="155.0",
        phase="available",
        branch="joint",
        history_raw_first_later_clear_offset_steps=first,
    )


class GateAndTimerStates(unittest.TestCase):
    def test_source_zero_is_known_without_using_timer_as_sign_input(self):
        row, _, _ = calculate(k=0)
        self.assertEqual(row["local_prior_gate_nominal"], "unknown")
        self.assertEqual(row["local_source_gate_nominal"], "fail")
        self.assertEqual(row["local_failed_gates"], "nominal")
        self.assertEqual(row["local_source_zero_reason"], "cap_zero")
        with self.assertRaises(AssertionError):
            calculate(k=0, ready_override=True)

    def test_positive_gate_accumulates_from_a_carried_counter(self):
        row, _, _ = calculate(k=1, before_ticks=8)
        self.assertEqual(row["local_source_certainty"], "pass")
        self.assertEqual(
            (row["local_timer_before_ticks"], row["local_timer_after_ticks"]), (8, 9)
        )
        self.assertEqual(row["local_timer_action"], "accumulate")

    def test_cap_latch_and_both_zero_reasons_remain_distinct(self):
        for k, stop, reason in (
            (0, False, "cap_zero"),
            (1, True, "latch_zero"),
            (0, True, "both_zero"),
        ):
            with self.subTest(reason=reason):
                row, _, _ = calculate(k=k, stop=stop)
                self.assertFalse(row["local_source_nominal_positive"])
                self.assertEqual(row["local_source_zero_reason"], reason)

    def test_simultaneous_gate_failures_keep_full_mask(self):
        row, _, _ = calculate(
            k=0,
            recovery_sample_count="1",
            recovery_clearance=".500000000",
            release_preview_clearance_lower_bound=".500000000",
        )
        self.assertEqual(row["local_failed_gates"], "samples;clearance;nominal;release")
        self.assertEqual(row["local_source_certainty"], "fail")

    def test_guard_and_inactive_applicability(self):
        row, _, _ = calculate(
            branch="guard",
            k=0,
            recovery_sample_count="1",
            recovery_clearance=".500000000",
        )
        self.assertEqual(row["local_source_certainty"], "pass")
        self.assertEqual(row["local_source_gate_nominal"], "not_applicable")
        self.assertEqual(row["local_threshold_ticks"], 25)
        row, _, _ = calculate(branch="not_applicable")
        self.assertEqual(row["local_source_certainty"], "not_applicable")
        self.assertIsNone(row["local_ready_from_timer"])
        self.assertIsNone(row["local_timer_after_ticks"])

    def test_joint_and_guard_release_thresholds(self):
        for branch, ticks in (("joint", 49), ("guard", 24)):
            row, _, _ = calculate(branch=branch, before_ticks=ticks)
            self.assertEqual(row["local_timer_action"], "release")
            self.assertEqual(row["local_timer_after_ticks"], 0)
            self.assertTrue(row["local_ready_from_timer"])

    def test_reset_and_stay_zero_are_distinct(self):
        row, _, _ = calculate(k=0, before_ticks=8)
        self.assertEqual(row["local_timer_action"], "reset")
        row, _, _ = calculate(k=0, before_ticks=0)
        self.assertEqual(row["local_timer_action"], "stay_zero")

    def test_exact_age_is_not_replaced_by_six_place_log(self):
        pair = next(
            (n, n - 75) for n in range(5000, 9001) if n * 0.02 - (n - 75) * 0.02 > 1.5
        )
        row, _, _ = calculate(n=pair[0], last=pair[1])
        self.assertEqual(row["local_input_observation_age"], "1.500000")
        self.assertEqual(row["local_source_gate_age"], "fail")
        self.assertGreater(float.fromhex(row["local_age_exact_hex"]), 1.5)

    def test_speed_roundtrip_and_near_equal_intervals(self):
        row, _, _ = calculate(measured_speed=format(math.nextafter(0.02, 1), ".17e"))
        self.assertEqual(row["local_source_gate_stationary"], "fail")
        row, _, _ = calculate(
            recovery_clearance="1.000000000", recovery_x="1.000000000"
        )
        self.assertEqual(row["local_source_gate_clearance"], "unknown")
        self.assertEqual(row["local_source_gate_bounds"], "unknown")
        self.assertEqual(row["local_source_certainty"], "unknown")

    def test_new_missed_observation_pause_resets_stale_counter(self):
        old, e, snap = fixture(
            k=0, before_ticks=9, correction="true", recovery_sample_count="1"
        )
        snap["before_raw"]["safety_paused"] = "false"
        for fn, table in (
            (a.local_slot, a.command.lattice()),
            (v.local_slot, {r["speed_free_distance"]: r for r in a.command.lattice()}),
        ):
            row = fn(old, e, snap, table)
            self.assertTrue(row["local_joint_pending"])
            self.assertEqual(row["local_timer_before_ticks"], 0)
            self.assertEqual(row["local_timer_action"], "stay_zero")

    def test_raw_count_and_margin_string_fidelity(self):
        old, e, snap = fixture()
        for bad in (
            dict(old, window_count="010"),
            dict(old, logged_speed_margin="1.0"),
        ):
            for fn in (a.check_context, v.check_context):
                with self.assertRaises(AssertionError):
                    fn(bad, snap)


class KnownPassSegments(unittest.TestCase):
    def run_both(self, rows, e):
        d = decision(e)
        left = a.ready_runs(d, rows)
        right = v.ready_runs(d, rows)
        self.assertEqual(left, right)
        self.assertEqual(a.metrics(d, rows, left), v.metrics(d, rows, right))
        return left

    def test_run_span_differs_from_evidence_added_and_preserves_carry(self):
        rows, e = context_slots()
        for offset, before in ((1, 5), (2, 6), (3, 7)):
            pass_at(rows, offset, before=before)
        runs = self.run_both(rows, e)
        self.assertEqual(len(runs), 1)
        run = runs[0]
        self.assertEqual(
            (
                run["checkpoint_count"],
                run["checkpoint_span_s"],
                run["evidence_added_s"],
            ),
            (3, 0.04, 0.06),
        )
        self.assertEqual((run["timer_before_ticks"], run["timer_after_ticks"]), (5, 8))

    def test_unknown_missing_branch_and_release_break_segments(self):
        rows, e = context_slots()
        for offset in (1, 2, 3, 4, 5, 6, 7, 8):
            pass_at(rows, offset)
        rows[2 + 25]["local_source_certainty"] = "unknown"
        rows[4 + 25].update(local_audited=False, local_source_certainty=None)
        pass_at(rows, 6, branch="guard")
        pass_at(rows, 7, branch="guard", before=24, action="release")
        pass_at(rows, 8, branch="guard")
        runs = self.run_both(rows, e)
        self.assertEqual([r["checkpoint_count"] for r in runs], [1, 1, 1, 2, 1])
        self.assertEqual(runs[0]["successor_certainty"], "unknown")
        self.assertTrue(runs[3]["observed_release"])
        self.assertTrue(runs[1]["right_context_limited"])

    def test_context_edges_and_before_return_do_not_reset_wait(self):
        rows, e = context_slots()
        for offset in range(-25, -21):
            pass_at(rows, offset, before=offset + 25)
        rows[0]["context_phase"] = "before_return"
        runs = self.run_both(rows, e)
        self.assertTrue(runs[0]["left_context_limited"])
        self.assertTrue(runs[0]["touches_before_return"])
        self.assertEqual(runs[0]["checkpoint_count"], 4)
        self.assertEqual(runs[0]["timer_after_ticks"], 4)
        self.assertFalse(runs[0]["right_context_limited"])

    def test_fixed_first_later_clear_join_and_current_clear_blank(self):
        rows, e = context_slots()
        pass_at(rows, 2)
        d = decision(e, first="2")
        runs = a.ready_runs(d, rows)
        for fn in (a.metrics, v.metrics):
            out = fn(d, rows, runs)
            self.assertEqual(out["local_first_clear_certainty"], "pass")
            self.assertEqual(out["local_first_clear_timer_after_ticks"], 1)
            blank = fn(decision(e), rows, runs)
            self.assertTrue(
                all(blank["local_first_clear_" + k] is None for k in a.FIRST)
            )

    def test_positive_segments_are_clipped_at_zero_for_local_maximum(self):
        rows, e = context_slots()
        for offset in range(-3, 4):
            pass_at(rows, offset, before=offset + 3)
        rr = self.run_both(rows, e)
        for fn in (a.metrics, v.metrics):
            result = fn(decision(e), rows, rr)
            self.assertEqual(result["local_joint_max_known_pass_ticks"], 7)
            self.assertEqual(result["local_joint_post_max_known_pass_ticks"], 3)
            self.assertIsNone(result["local_guard_max_after_ticks"])

    def test_missing_slots_are_counted_separately_from_failure(self):
        rows, e = context_slots()
        for row in rows:
            if int(row["offset_steps"]) > 2:
                row.update(local_audited=False, local_source_certainty=None)
        rr = self.run_both(rows, e)
        result = a.metrics(decision(e), rows, rr)
        self.assertEqual(result["local_post_missing_slots"], 23)
        self.assertEqual(result["local_post_joint_fail_slots"], 2)
        self.assertEqual(result["local_raw_clear_nominal_zero_slots"], 28)


class JoinsAccountingAndNative(unittest.TestCase):
    def test_original_reference_join_keeps_counter_and_gate_identity(self):
        row, e, _ = calculate(k=0)
        original = dict(case_id=row["case_id"], phase="available")
        for gate in a.GATES:
            original["source_nominal_gate" if gate == "nominal" else "gate_" + gate] = (
                row["local_source_gate_" + gate]
            )
        for key, target in (
            ("source_certainty", "refined_certainty"),
            ("failed_gates", "refined_failed_gates"),
            ("unknown_gates", "refined_unknown_gates"),
            ("branch", "branch"),
            ("source_zero_reason", "source_zero_reason"),
            ("timer_action", "timer_action"),
            ("source_nominal_positive", "source_nominal_positive"),
            ("ready_from_timer", "ready_from_timer"),
            ("timer_before_ticks", "timer_before_ticks"),
            ("timer_after_ticks", "timer_after_ticks"),
        ):
            value = row["local_" + key]
            original[target] = "" if value is None else str(value)
        db = {(row["event_id"], int(row["source_step"])): original}
        for fn in (a.join_original, v.join_original):
            fn(row, db)
            broken = {k: dict(val, timer_after_ticks="1") for k, val in db.items()}
            with self.assertRaises(AssertionError):
                fn(row, broken)
            fn(dict(row, local_original_reference_joined=False), {})

    def test_nonreturns_unequal_original_measures_empty_denominators_and_closures(self):
        rows, e = context_slots()
        d = decision(e, first="1")
        r = a.metrics(d, rows, a.ready_runs(d, rows))
        d = dict(d, **r, local_audited=True)
        events = [
            e,
            dict(histories.old_events.event(False), event_id="2"),
            dict(
                e,
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
            local_raw_clear_nominal_zero_slots=0
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
        self.assertTrue(all(aa[1][k] is None for k in (*a.REFS, *a.HAS)))
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
            cell["has_available_local_raw_clear_nominal_zero_fraction"], Fraction(1, 2)
        )
        self.assertEqual(
            cell["has_available_local_raw_clear_nominal_zero_case_mass_fraction"],
            Fraction(1, 3),
        )
        self.assertTrue(
            any(
                r["has_available_local_raw_clear_nominal_zero_fraction"] is None
                for r in rr
            )
        )
        cell["available_joint_local_first_clear_audited_refs"] += 1
        with self.assertRaises(AssertionError):
            a.closure(rr)

    def test_native_source_thresholds_and_unknown_center_mismatch(self):
        rustc = (
            shutil.which("rustc")
            or "/workspace/.cloud-env/rust_robotics/rustup/toolchains/1.99.0-x86_64-unknown-linux-gnu/bin/rustc"
        )
        if not Path(rustc).exists():
            self.skipTest("rustc unavailable")
        examples = [
            calculate(before_ticks=49),
            calculate(branch="guard", before_ticks=24),
            calculate(k=0, before_ticks=7),
            calculate(recovery_clearance="1.000000000", ready_override=True),
        ]
        contexts = {i: snap for i, (_, _, snap) in enumerate(examples, 1)}
        mapping = [
            dict(
                row_id=i,
                event_id=row["event_id"],
                reference_time_s=row["reference_time_s"],
                offset_steps=0,
                source_time_s=float(row["source_time_s"]),
            )
            for i, (row, _, _) in enumerate(examples, 1)
        ]
        with tempfile.TemporaryDirectory() as tmp:
            traces, proof = v.native_admission(
                Path(tmp) / "native", Path(rustc), v.native_queries(mapping, contexts)
            )
            self.assertEqual(proof["native_tests"], 8)
            self.assertEqual(proof["traced_context_queries"], 4)
            for i, (row, e, snap) in enumerate(examples, 1):
                mismatch = v.check_native(row, traces[i], snap["raw"])
                self.assertEqual(mismatch, int(i == 4))
            self.assertEqual(examples[3][0]["local_source_certainty"], "unknown")

    def test_contracts_and_empty_history_neighbor_limits(self):
        source = (a.REPO / next(iter(v.CODE))).read_text()
        self.assertEqual(a.contract(source), v.contract(source))
        rows, e = context_slots()
        self.assertEqual(a.ready_runs(decision(e), rows), [])
        self.assertEqual(v.ready_runs(decision(e), rows), [])


if __name__ == "__main__":
    unittest.main()
