"""Test adjacent membership identities, interval comparisons and context roles."""

import copy
from decimal import Decimal as D
from fractions import Fraction
import itertools
import math
from pathlib import Path
import shutil
import tempfile
import unittest

import analyze_window_turnover as a
import verify_window_turnover as v
import test_recovery_window as fixtures


def point(i, x=None, y=25.0):
    before = fixtures.initial()
    before.update(x=f"{25+i/100 if x is None else x:.9f}", y=f"{y:.9f}")
    return a.prior.sample(i, before, fixtures.row(i, 0), 0)


def context(members, current, clear=0):
    effective = members or [current]
    fields = {
        "mean_"
        + axis
        + "_center": a.prior.fold(s["sensed_" + axis + "_center"] for s in effective)
        / len(effective)
        for axis in ("x", "y")
    }
    fields.update(
        window_last_clear_step=clear,
        window_last_clear_reasons="received" if clear else "",
    )
    return dict(members=members, point=current, fields=fields)


def decision(n=26):
    return dict(
        fixtures.old.old(fixtures.old.raw()),
        time_s=str(n / 50),
        prior_certainty="unknown",
        refined_failed_gates="nominal",
        timer_action="reset",
    )


def calculate(before, after, n=26):
    d = decision(n)
    actual, rows = a.turnover(d, before, after)
    expected, other = v.turnover(d, before, after)
    assert rows == other
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp) / "turnover.csv"
        a.write(p, [dict(id="1", **actual)])
        v.compare(p, [dict(id="1", **expected)], ("id",))
    return actual, rows, d


class MembershipIdentities(unittest.TestCase):
    def test_growing_window_uses_previous_mean_and_new_count(self):
        before = context([point(24, 24), point(25, 25)], point(25, 25))
        after = context([point(24, 24), point(25, 25), point(26, 27)], point(26, 27))
        r, _, _ = calculate(before, after)
        self.assertEqual(r["turnover_class"], "grow")
        self.assertAlmostEqual(r["mean_step_x_center"], (27 - 24.5) / 3)
        self.assertEqual((r["added_steps"], r["removed_steps"]), ("26", ""))

    def test_full_slide_equals_new_minus_removed_divided_by_25(self):
        old = [point(i) for i in range(1, 26)]
        new = old[1:] + [point(26)]
        r, rows, _ = calculate(context(old, old[-1]), context(new, new[-1]))
        expected = (new[-1]["sensed_x_center"] - old[0]["sensed_x_center"]) / 25
        self.assertEqual(r["turnover_class"], "slide")
        self.assertAlmostEqual(r["membership_prediction_x_center"], expected, places=13)
        self.assertEqual(
            (r["added_count"], r["removed_count"], r["shared_count"]), (1, 1, 24)
        )
        self.assertEqual(
            [x["member_role"] for x in rows], ["removed"] + ["shared"] * 24 + ["added"]
        )

    def test_first_full_window_is_growth_without_eviction(self):
        old = [point(i) for i in range(2, 26)]
        new = old + [point(26)]
        r, _, _ = calculate(context(old, old[-1]), context(new, new[-1]))
        self.assertEqual(r["turnover_class"], "grow")
        self.assertEqual(r["removed_count"], 0)

    def test_reset_removes_all_previous_members_before_new_sample(self):
        old = [point(i) for i in range(1, 26)]
        new = [point(26, 28)]
        r, _, _ = calculate(context(old, old[-1]), context(new, new[-1], clear=26))
        self.assertEqual(r["turnover_class"], "reset")
        self.assertEqual((r["removed_count"], r["added_count"]), (25, 1))
        self.assertLessEqual(abs(r["membership_residual_x"]), a.P)

    def test_identical_memberships_hold_mean_when_current_pose_changes(self):
        members = [point(20), point(21)]
        r, _, _ = calculate(
            context(members, point(25, 25)), context(members, point(26, 26))
        )
        self.assertEqual(r["turnover_class"], "held")
        self.assertEqual(r["mean_step_x_center"], 0.0)
        self.assertEqual(r["gap_step_x_center"], -1.0)
        self.assertEqual(r["step_comparison"], "raw_larger")

    def test_empty_side_keeps_membership_law_blank(self):
        for before, after, kind in (
            (context([], point(25)), context([point(26)], point(26), 26), "new"),
            (context([point(25)], point(25)), context([], point(26), 26), "fallback"),
            (context([], point(25)), context([], point(26)), "fallback"),
        ):
            r, _, _ = calculate(before, after)
            self.assertEqual(r["turnover_class"], kind)
            self.assertEqual(r["step_comparison"], "not_comparable")
            self.assertTrue(
                all(r[k] is None for k in a.FIELDS if k.startswith("membership_"))
            )

    def test_noncontiguous_growth_and_unexpected_change_are_distinct(self):
        old = [point(1), point(7)]
        r, _, _ = calculate(
            context(old, point(25)), context(old + [point(26)], point(26))
        )
        self.assertEqual(r["turnover_class"], "grow")
        r, _, _ = calculate(
            context(old, point(25)), context([point(1), point(8)], point(26))
        )
        self.assertEqual(r["turnover_class"], "other")
        self.assertEqual(r["shared_steps"], "1")

    def test_duplicate_ids_and_inconsistent_shared_inputs_fail(self):
        p = point(25)
        before = context([p], p)
        for after in (
            context([p, p], point(26)),
            context([dict(p, noise_x="0.0"), point(26)], point(26)),
        ):
            for fn in (a.turnover, v.turnover):
                with self.assertRaises(AssertionError):
                    fn(decision(), before, after)


class BoundedSteps(unittest.TestCase):
    def test_norm_boxes_cover_positive_negative_and_zero_crossing_corners(self):
        for pairs in (
            ((-1.0, 2.0), (-3.0, 4.0)),
            ((1.0, 2.0), (3.0, 4.0)),
            ((-2.0, -1.0), (-4.0, -3.0)),
        ):
            low, high = a.norm_box(pairs)
            dl, dh = v.box_norm(tuple((D(str(x)), D(str(y))) for x, y in pairs))
            self.assertAlmostEqual(low, float(dl), places=12)
            self.assertAlmostEqual(high, float(dh), places=12)
            for x, y in itertools.product(*pairs):
                self.assertLessEqual(low, math.hypot(x, y))
                self.assertGreaterEqual(high, math.hypot(x, y))

    def test_norm_comparisons_are_strict_and_empty_sides_are_separate(self):
        for raw, mean, eligible, label in (
            ((1.0, 2.0), (2.0, 3.0), True, "overlap"),
            ((3.01, 4.0), (2.0, 3.0), True, "raw_larger"),
            ((1.0, 2.0), (2.01, 3.0), True, "mean_larger"),
            ((3.0, 4.0), (1.0, 2.0), False, "not_comparable"),
        ):
            self.assertEqual(a.compare_norms(raw, mean, eligible), label)
            self.assertEqual(
                v.norm_comparison(tuple(map(D, raw)), tuple(map(D, mean)), eligible),
                label,
            )

    def test_mean_can_change_more_than_raw_without_relabeling_membership(self):
        members = [point(24, 20), point(25, 25)]
        r, _, _ = calculate(
            context(members, point(25, 25)),
            context(members + [point(26, 25.001)], point(26, 25.001)),
        )
        self.assertEqual(r["turnover_class"], "grow")
        self.assertEqual(r["step_comparison"], "mean_larger")

    def test_interval_contains_changes_of_consistently_perturbed_members(self):
        members = [point(i) for i in range(1, 26)]
        new = members[1:] + [point(26)]
        before, after = context(members, members[-1]), context(new, new[-1])
        r, _, _ = calculate(before, after)
        for sign in (-1, 1):
            modified = {
                p["source_step"]: dict(
                    p,
                    sensed_x_center=p["sensed_x_center"]
                    + sign * a.R * (-1 if p["source_step"] == 1 else 1),
                )
                for p in members + new
            }
            pm = (
                a.prior.fold(
                    modified[p["source_step"]]["sensed_x_center"] for p in members
                )
                / 25
            )
            cm = (
                a.prior.fold(modified[p["source_step"]]["sensed_x_center"] for p in new)
                / 25
            )
            self.assertLessEqual(r["mean_step_x_lo"], cm - pm)
            self.assertGreaterEqual(r["mean_step_x_hi"], cm - pm)
            self.assertLessEqual(r["membership_prediction_x_lo"], cm - pm)
            self.assertGreaterEqual(r["membership_prediction_x_hi"], cm - pm)

    def test_signed_gap_identity_closes_both_coordinates(self):
        old = [point(24, 24, 26), point(25, 25, 25)]
        new = old + [point(26, 26, 24)]
        r, _, _ = calculate(context(old, old[-1]), context(new, new[-1]))
        for axis in ("x", "y"):
            self.assertAlmostEqual(
                r[f"gap_step_{axis}_center"],
                r[f"mean_step_{axis}_center"] - r[f"raw_step_{axis}_center"],
                places=12,
            )
            self.assertLessEqual(abs(r[f"gap_identity_residual_{axis}"]), a.P)

    def test_inconsistent_mean_or_native_change_is_rejected(self):
        old = [point(24), point(25)]
        new = old + [point(26)]
        before, after = context(old, old[-1]), context(new, new[-1])
        bad = copy.deepcopy(after)
        bad["fields"]["mean_x_center"] += 0.01
        for fn in (a.turnover, v.turnover):
            with self.assertRaises(AssertionError):
                fn(decision(), before, bad)
        with self.assertRaises(AssertionError):
            v.turnover(
                decision(),
                before,
                after,
                dict(mean_x="25", mean_y="25"),
                dict(mean_x="100", mean_y="25"),
            )


class ContextsAndAccounting(unittest.TestCase):
    def test_context_queries_keep_reference_ids_and_duplicate_source_points(self):
        selected = [
            dict(case_id="C1", event_id="E1", time_s="0.52"),
            dict(case_id="C1", event_id="E2", time_s="0.54"),
        ]
        queries, rows = v.context_queries(selected)
        self.assertEqual(queries["C1", 26], [2, 3])
        self.assertEqual([q["context_role"] for q in rows], ["previous", "current"] * 2)
        self.assertEqual(rows[2]["reference_time_s"], "0.54")
        self.assertEqual(rows[2]["source_time_s"], 0.52)
        self.assertEqual(rows[2]["event_id"], "E2")

    def test_member_strings_roles_and_order_are_verified(self):
        old = [point(24), point(25)]
        new = old + [point(26)]
        _, rows, d = calculate(context(old, old[-1]), context(new, new[-1]))
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "members.csv"
            a.write(p, rows)
            v.compare(p, rows, ("event_id", "time_s", "union_order"))
            wrong = copy.deepcopy(rows)
            wrong[0]["noise_x"] = "0.0"
            with self.assertRaises(AssertionError):
                v.compare(p, wrong, ("event_id", "time_s", "union_order"))

    def test_phase_branch_subgroups_censoring_and_nonreturns(self):
        old = [point(24), point(25)]
        new = old + [point(26)]
        r, _, d = calculate(context(old, old[-1]), context(new, new[-1]))
        d = dict(d, **r, turnover_audited=True, phase="next_loss", branch="guard")
        for fn in (a.anchor, v.anchor):
            cell = fn(fixtures.old.event(), [d], 2, 2)
            self.assertEqual(cell["next_loss_guard_turnover_grow_refs"], 1)
            self.assertEqual(
                cell["next_loss_guard_turnover_prior_unknown_raw_larger_refs"], 1
            )
            self.assertEqual(cell["censored_return"], "True")
            self.assertEqual(cell["end_time_s"], "")
            self.assertFalse(cell["has_available_turnover_grow"])
            blank = fn(fixtures.old.event(False), [], 2, 2)
            self.assertTrue(all(blank[k] is None for k in (*a.REFS, *a.HAS)))

    def test_original_unequal_measures_empty_shares_and_closures(self):
        old = [point(24), point(25)]
        new = old + [point(26)]
        r, _, d = calculate(context(old, old[-1]), context(new, new[-1]))
        d = dict(d, **r, turnover_audited=True)
        events = [
            fixtures.old.event(),
            dict(fixtures.old.event(False), event_id="2"),
            dict(
                fixtures.old.event(),
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
            turnover_class="fallback",
            step_comparison="not_comparable",
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
        self.assertEqual(cell["has_available_turnover_grow_fraction"], Fraction(1, 2))
        self.assertEqual(
            cell["has_available_turnover_grow_case_mass_fraction"], Fraction(1, 3)
        )
        self.assertTrue(
            any(r["has_available_turnover_grow_fraction"] is None for r in rr)
        )
        cell["available_joint_turnover_audited_refs"] += 1
        with self.assertRaises(AssertionError):
            a.closure(rr)

    def test_native_both_roles_match_same_source_context_when_shared(self):
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
                "case_id\tstep\tpaused\tcorrection\treceived\tprecision\tspeed\tage\tx\ty\tyaw\tlogged_count\trow_ids\nC1\t1\ttrue\tfalse\tfalse\tfalse\t0\t0.02\t1\t2\t0\t1\t1\nC1\t2\ttrue\tfalse\tfalse\tfalse\t0\t0.04\t3\t2\t0\t2\t2;3\nC1\t3\ttrue\tfalse\tfalse\tfalse\t0\t0.06\t5\t2\t0\t3\t4\n"
            )
            selected = [
                dict(case_id="C1", event_id=str(i), time_s=str(n / 50))
                for i, n in enumerate((1, 2, 2, 3), 1)
            ]
            traces, proof = v.previous.native(work, Path(rustc), selected)
            self.assertEqual(proof["native_tests"], 9)
            self.assertEqual(proof["replayed_steps"], 3)
            self.assertEqual(traces[2]["mean_x"], traces[3]["mean_x"])
            self.assertEqual(len(traces), 4)


if __name__ == "__main__":
    unittest.main()
