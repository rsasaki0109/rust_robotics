"""Test logged source/latch distinctions and fixed returned-spell windows."""

import copy
import csv
from fractions import Fraction
from pathlib import Path
import sqlite3
import tempfile
import unittest

import analyze_return_controls as a
import verify_return_controls as v


def raw(n, available=True, paused=True, guarded=True, **changes):
    row = {k: "false" for k in a.RAW_BOOL}
    row.update({k: "0.0" for k in a.RAW_FLOAT})
    row.update({k: "1" for k in a.RAW_COUNTS})
    row.update(
        time_s=str(n / 50),
        safety_paused=str(paused).lower(),
        guard_active=str(guarded).lower(),
        stationary_epoch_fusion="true",
        recovery_preview_active="true",
        epoch_fusion_available=str(available).lower(),
        epoch_fusion_group_count="3" if available else "2",
        nominal_target_speed="0.4",
        target_speed="0.0" if paused or guarded else "0.4",
        release_preview_clearance_lower_bound="1.0",
        guard_release_margin="0.8",
    )
    row.update(changes)
    return row


def fixture(start=7750, end=7751):
    e = dict(
        zip(
            a.META,
            (
                "C1",
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
    e.update(
        event_id="1",
        source_event_id="fresh_1",
        event_time_s="150.0",
        scheduled="True",
        received="True",
        precision_activation="False",
        immediate_actual="recovery_mean",
        immediate_preview="three_epoch",
        preview_loss="True",
        returned="True",
        ended=str(end is not None),
        censored_return=str(end is None),
        no_return="False",
        ttl_limit_without_next_step="False",
        return_time_s=str(start / 50),
        end_time_s=str(end / 50) if end else "",
        end_operation="ttl_expiry" if end == start + 1 else "age_gate" if end else "",
        parent_rp_records="8",
        rp_records="2",
        rp_different_preview_records="1",
    )
    hi = end if end else 9000
    pp = {
        n: a.logged(
            raw(
                n,
                available=start <= n and (n < hi or end is None),
                recovery_sample_count=str(min(25, max(1, n - start + 1))),
            )
        )
        for n in range(start - 1, hi + 1)
    }
    return e, pp


def trace(f):
    return a.anchor(*f, 7, 3)


class LoggedControlRules(unittest.TestCase):
    def test_all_latch_states_have_independent_source_identifiers(self):
        for paused, guarded, state in (
            (True, True, "loss_and_guard"),
            (True, False, "loss_only"),
            (False, True, "guard_only"),
            (False, False, "clear"),
        ):
            with self.subTest(state=state):
                row = raw(
                    7750,
                    False,
                    paused,
                    guarded,
                    recovery_sample_count="0",
                    epoch_fusion_group_count="3",
                )
                s = a.logged(row)
                a.command_check(s)
                self.assertEqual(s, v.decode(row))
                self.assertEqual(s["latch_state"], state)
                self.assertEqual(s["pose_branch"], "raw_fallback")
                self.assertEqual(s["preview_branch"], "motion_filter")

    def test_nominal_positive_does_not_make_a_paused_command_positive(self):
        r, _, _ = trace(fixture())
        self.assertTrue(r["return_nominal_positive"])
        self.assertFalse(r["return_target_positive"])
        self.assertTrue(r["any_positive_nominal"])
        self.assertFalse(r["any_positive_target"])

    def test_clear_latches_do_not_require_positive_speed(self):
        row = raw(
            7750,
            paused=False,
            guarded=False,
            target_speed="0",
            nominal_target_speed="0",
            stop_latched="true",
        )
        s = a.logged(row)
        a.command_check(s)
        self.assertEqual(s["latch_state"], "clear")
        self.assertFalse(s["target_positive"])
        self.assertEqual(s, v.decode(row))

    def test_command_overwrites_are_enforced(self):
        for changes in (
            dict(target_speed="0.1"),
            dict(steering="0.2"),
            dict(resumed="true"),
            dict(target_speed="-0.1"),
        ):
            with self.subTest(changes=changes):
                row = raw(7750, **changes)
                with self.assertRaises(AssertionError):
                    a.command_check(a.logged(row))
                with self.assertRaises(AssertionError):
                    v.decode(row)
        row = raw(7750, paused=False, guarded=False, target_speed="0.3")
        with self.assertRaises(AssertionError):
            a.command_check(a.logged(row))
        with self.assertRaises(AssertionError):
            v.decode(row)

    def test_guard_only_steering_can_differ_from_nominal(self):
        row = raw(
            7750, paused=False, guarded=True, steering="0.2", nominal_steering="0.1"
        )
        a.command_check(a.logged(row))
        self.assertEqual(a.logged(row), v.decode(row))

    def test_strict_flags_counts_and_finite_scalars(self):
        for changes in (
            dict(resumed="True"),
            dict(recovery_sample_count="-1"),
            dict(recovery_sample_count="01"),
            dict(epoch_fusion_group_count="4"),
            dict(observation_age="-1"),
            dict(target_speed="nan"),
        ):
            with self.subTest(changes=changes):
                row = raw(7750, **changes)
                with self.assertRaises(AssertionError):
                    a.logged(row)
                with self.assertRaises(AssertionError):
                    v.decode(row)


class SpellWindows(unittest.TestCase):
    def test_short_spell_includes_one_available_checkpoint_and_both_sides(self):
        r, rr, ss = trace(fixture())
        self.assertEqual(r["within_available_checkpoints"], 1)
        self.assertEqual(r["control_window_checkpoints"], 3)
        self.assertEqual(sum(x["checkpoint_count"] for x in rr), 3)
        self.assertEqual(
            [x["role"] for x in ss],
            ["return_before", "return", "end_before", "end_after"],
        )
        self.assertFalse(r["any_sample_ready"])

    def test_end_checkpoint_is_excluded_from_within_available_measures(self):
        e, pp = fixture()
        pp[7751] = a.logged(raw(7751, False, False, False, resumed="true"))
        r, _, _ = trace((e, pp))
        self.assertFalse(r["any_resume"])
        self.assertFalse(r["any_positive_target"])
        self.assertTrue(r["boundary_resumed"])
        self.assertTrue(r["end_target_positive_changed"])
        self.assertTrue(r["end_latch_changed"])

    def test_pulse_is_preserved_in_maximal_runs(self):
        e, pp = fixture(end=7754)
        for n in range(7750, 7754):
            pp[n] = a.logged(raw(n, True, False, False, resumed=str(n == 7751).lower()))
        r, rr, _ = trace((e, pp))
        self.assertTrue(r["any_resume"])
        self.assertEqual(r["within_resumed_checkpoints"], 1)
        self.assertEqual(sum(x["checkpoint_count"] for x in rr if x["resumed"]), 1)
        self.assertTrue(r["any_positive_target"])

    def test_integer_sample_requirement_does_not_imply_resumption(self):
        r, _, _ = trace(fixture(end=7826))
        self.assertEqual(r["within_available_checkpoints"], 76)
        self.assertEqual(r["min_available_recovery_samples"], 1)
        self.assertEqual(r["max_available_recovery_samples"], 25)
        self.assertTrue(r["any_sample_ready"])
        self.assertFalse(r["any_resume"])
        self.assertEqual(r["within_sample_ready_checkpoints"], 67)

    def test_endpoint_return_has_one_observed_checkpoint_and_unknown_end_change(self):
        r, rr, ss = trace(fixture(start=9000, end=None))
        self.assertEqual(r["within_available_checkpoints"], 1)
        self.assertEqual(r["control_window_checkpoints"], 2)
        self.assertIsNone(r["end_pose_changed"])
        self.assertIsNone(r["end_latch_changed"])
        self.assertEqual(
            [x["role"] for x in ss], ["return_before", "return", "censored_endpoint"]
        )
        self.assertEqual(sum(x["checkpoint_count"] for x in rr), 2)

    def test_no_return_has_blank_control_fields(self):
        e, pp = fixture()
        e.update(returned="False", ended="", censored_return="", no_return="True")
        r, rr, ss = trace((e, pp))
        self.assertEqual(r["anchor_category"], "no_return")
        self.assertTrue(all(r[k] is None for k in a.NEW_FIELDS))
        self.assertEqual((rr, ss), ([], []))

    def test_missing_window_and_changed_availability_fail(self):
        e, pp = fixture()
        with self.assertRaises(AssertionError):
            a.compressed({k: x for k, x in pp.items() if k != 7750}, 7749, 7751)
        pp[7750]["epoch_fusion_available"] = False
        with self.assertRaises(AssertionError):
            trace((e, pp))

    def test_repeated_anchor_references_and_nonmutation(self):
        f = fixture()
        before = copy.deepcopy(f)
        r, rr, ss = trace(f)
        e, pp = f
        other, other_runs, _ = trace(
            (dict(e, event_id="99", source_event_id="fresh_99"), pp)
        )
        self.assertEqual(f, before)
        self.assertEqual(r["return_time_s"], other["return_time_s"])
        self.assertNotEqual(rr[0]["event_id"], other_runs[0]["event_id"])

    def test_sql_runs_match_and_isolate_full_cases(self):
        e, pp = fixture()
        _, expected, _ = trace((e, pp))
        with sqlite3.connect(":memory:") as db:
            db.executescript(
                "CREATE TABLE anchors(eid TEXT,cid TEXT,lo INT,hi INT);CREATE TABLE points(cid TEXT,n INT,pose TEXT,preview TEXT,paused INT,guarded INT,resumed INT,positive INT);"
            )
            fixed = {}
            for eid, cid in (("1", "C1"), ("2", "C2")):
                fixed[eid,] = dict(e, event_id=eid, case_id=cid)
                db.execute(
                    "INSERT INTO anchors VALUES(?,?,?,?)", (eid, cid, 7749, 7751)
                )
                for n, s in pp.items():
                    db.execute(
                        "INSERT INTO points VALUES(?,?,?,?,?,?,?,?)",
                        (
                            cid,
                            n,
                            s["pose_branch"],
                            s["preview_branch"],
                            s["safety_paused"],
                            s["guard_active"],
                            s["resumed"],
                            s["target_positive"],
                        ),
                    )
            actual = v.sql_runs(db, fixed)
        self.assertEqual(
            [actual["1", str(i)] for i in range(1, len(expected) + 1)], expected
        )
        self.assertTrue(
            all(actual[k]["case_id"] == "C2" for k in actual if k[0] == "2")
        )


class AccountingAndRawGrids(unittest.TestCase):
    def test_fixed_unequal_parents_remain_separate(self):
        e, pp = fixture()
        r1, _, _ = a.anchor(e, pp, 7, 3)
        r2, _, _ = a.anchor(dict(e, event_id="2", case_id="C2"), pp, 8, 4)
        result = a.summarize([], [r1, r2], {})
        self.assertEqual(result["returned_case_mass"], Fraction(1, 7) + Fraction(1, 8))
        self.assertEqual(result["received_case_mass"], Fraction(1, 3) + Fraction(1, 4))
        self.assertEqual(result["rp_case_mass"], Fraction(1, 2))
        self.assertEqual(result["any_resume_fraction"], 0)

    def test_empty_denominator_is_blank_and_observed_zero_is_zero(self):
        empty = a.summarize([], [], {})
        self.assertIsNone(empty["any_resume_fraction"])
        self.assertIsNone(empty["any_positive_target_case_mass_fraction"])
        r, _, _ = trace(fixture())
        observed = a.summarize([], [r], {})
        self.assertEqual(observed["any_resume_fraction"], 0)
        self.assertEqual(observed["any_positive_target_case_mass_fraction"], 0)

    def test_all_336_cells_close_and_independent_summary_matches(self):
        r, _, _ = trace(fixture())
        c = {k: r[k] for k in a.META}
        c.update(selected_timeout="True", held="False")
        rows = a.aggregate([c], [r])
        self.assertEqual(len(rows), 336)
        other = v.summaries([c], {("1",): r})
        self.assertEqual(a.indexed(rows, a.GROUP), other)
        self.assertEqual(a.closure(rows), v.closures(other))

    def test_original_grid_rejects_missing_duplicate_or_wrong_case_rows(self):
        e, _ = fixture()
        c = {k: e[k] for k in a.META}
        c.update(selected_timeout="False", time_s="0.02")
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            p = root / "epoch-random-277/trajectories.csv"
            p.parent.mkdir()
            rows = [
                dict(scenario=c["scenario"], planner=c["planner"], **raw(n))
                for n in (0, 1)
            ]

            def save(rr):
                with p.open("w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=rows[0])
                    w.writeheader()
                    w.writerows(rr)

            def check():
                spec = {
                    c["cohort"]: dict(
                        path=str(root),
                        raw_sha256={"epoch-random-277/trajectories.csv": a.digest(p)},
                    )
                }
                return a.collect([c], [], spec, {})

            save(rows)
            self.assertEqual(check()[3]["raw_files"], 1)
            for rr in (
                rows[:1],
                [*rows, rows[-1]],
                [rows[0], dict(rows[1], time_s="0.04")],
                [rows[0], dict(rows[1], planner="Other")],
            ):
                save(rr)
                with self.assertRaises(AssertionError):
                    check()


if __name__ == "__main__":
    unittest.main()
