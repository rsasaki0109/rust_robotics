"""Test finite-grid histories, strict follow-up limits and original measures."""

from fractions import Fraction
import itertools
from pathlib import Path
import shutil
import tempfile
import unittest

import analyze_point_history as a
import verify_point_history as v
import test_point_clearance as fixtures

old_events = fixtures.fixtures.fixtures.old


def decision():
    return dict(
        fixtures.decision(), fixed_obstacle_ids="1", raw_distance_direction="toward"
    )


def event(returned=0.4, loss=0.62, censored=False):
    return dict(
        return_time_s=str(returned),
        ended=str(not censored),
        end_time_s="" if censored else str(loss),
    )


def slots(raw="blocked", mean="clear"):
    return [
        dict(
            offset_steps=i,
            source_time_s=(26 + i) / 50,
            slot_status="observed",
            context_phase="reference" if i == 0 else "available_context",
            raw_state=raw,
            mean_state=mean,
        )
        for i in range(-25, 26)
    ]


def calculate(rows, d=None):
    d = decision() if d is None else d
    result = a.history(d, rows)
    expected = v.history(d, rows)
    if result != expected:
        raise AssertionError((result, expected))
    return result


class SlotPlanning(unittest.TestCase):
    def compare(self, d, e):
        rows = a.plan_slots([d], {d["event_id"]: e})
        self.assertEqual(rows, v.plan_slots([d], {d["event_id"]: e}))
        self.assertEqual(len(rows), 51)
        return {r["offset_steps"]: r for r in rows}

    def test_fixed_offsets_and_exact_next_loss_exclusion(self):
        rows = self.compare(decision(), event())
        self.assertEqual((rows[-25]["source_step"], rows[25]["source_step"]), (1, 51))
        self.assertEqual(rows[4]["slot_status"], "observed")
        self.assertEqual(rows[5]["slot_status"], "at_or_after_next_loss")
        self.assertEqual(rows[25]["slot_status"], "at_or_after_next_loss")
        self.assertEqual(rows[5]["context_phase"], "")

    def test_censored_return_ends_at_source_last_step(self):
        rows = self.compare(
            dict(decision(), time_s="180"), event(returned=179, censored=True)
        )
        self.assertEqual(rows[0]["slot_status"], "observed")
        self.assertEqual(rows[1]["slot_status"], "after_source")
        self.assertEqual(rows[25]["slot_status"], "after_source")

    def test_source_boundary_has_priority_over_next_loss(self):
        rows = self.compare(
            dict(decision(), time_s="179.98"), event(returned=179, loss=180)
        )
        self.assertEqual(rows[1]["slot_status"], "at_or_after_next_loss")
        self.assertEqual(rows[2]["slot_status"], "after_source")

    def test_before_source_and_before_return_are_distinct(self):
        rows = self.compare(
            dict(decision(), time_s=".04"), event(returned=0.04, loss=0.4)
        )
        self.assertEqual(rows[-2]["slot_status"], "before_source")
        self.assertEqual(rows[-1]["context_phase"], "before_return")
        self.assertEqual(rows[0]["context_phase"], "reference")
        self.assertEqual(rows[1]["context_phase"], "available_context")

    def test_duplicate_source_queries_preserve_reference_and_offset(self):
        d = decision()
        other = dict(d, time_s=".54")
        planned = a.plan_slots([d, other], {d["event_id"]: event()})
        queries, mapping = v.context_queries(planned)
        shared = queries[d["case_id"], 26]
        self.assertEqual(len(shared), 2)
        self.assertEqual(len(set(shared)), 2)
        rr = [r for r in mapping if r["row_id"] in shared]
        self.assertEqual([r["offset_steps"] for r in rr], [0, -1])
        self.assertEqual([r["reference_time_s"] for r in rr], ["0.52", ".54"])
        self.assertTrue(
            all(r["source_time_s"] < 0.62 for r in mapping if r["offset_steps"] > 0)
        )


class FiniteRuns(unittest.TestCase):
    def test_inclusive_run_count_and_checkpoint_span(self):
        rr = slots(raw="clear")
        for s in rr:
            if -2 <= s["offset_steps"] <= 0:
                s["raw_state"] = "blocked"
        r = calculate(rr)
        self.assertEqual(r["history_raw_run_count"], 3)
        self.assertEqual(r["history_raw_run_start_offset"], -2)
        self.assertEqual(r["history_raw_run_span_s"], 0.04)
        self.assertFalse(r["history_raw_left_censored"])
        self.assertEqual(r["history_raw_predecessor_state"], "clear")
        self.assertTrue(r["history_raw_persisting_blocked"])

    def test_left_limit_censors_onset_with_26_points_and_half_second_span(self):
        r = calculate(slots())
        self.assertEqual(r["history_raw_run_count"], 26)
        self.assertEqual(r["history_raw_run_span_s"], 0.5)
        self.assertTrue(r["history_raw_left_censored"])
        self.assertIsNone(r["history_raw_predecessor_state"])

    def test_new_entry_has_one_point_and_zero_span(self):
        rr = slots(raw="clear")
        rr[25]["raw_state"] = "blocked"
        r = calculate(rr)
        self.assertEqual(r["history_raw_run_count"], 1)
        self.assertEqual(r["history_raw_run_span_s"], 0)
        self.assertTrue(r["history_raw_new_blocked"])
        self.assertFalse(r["history_raw_persisting_blocked"])

    def test_unknown_predecessor_breaks_run_without_forcing_entry_label(self):
        rr = slots()
        rr[24]["raw_state"] = "unknown"
        r = calculate(rr)
        self.assertEqual(r["history_raw_run_count"], 1)
        self.assertEqual(r["history_raw_predecessor_state"], "unknown")
        self.assertFalse(r["history_raw_left_censored"])
        self.assertFalse(r["history_raw_new_blocked"])
        self.assertFalse(r["history_raw_persisting_blocked"])

    def test_missing_predecessor_breaks_run_and_censors_onset(self):
        rr = slots()
        rr[24].update(slot_status="before_source", raw_state=None, mean_state=None)
        r = calculate(rr)
        self.assertEqual(r["history_raw_run_count"], 1)
        self.assertTrue(r["history_raw_left_censored"])
        self.assertEqual(r["history_raw_pre_missing_slots"], 1)

    def test_first_certain_later_clear_can_follow_an_unknown(self):
        rr = slots()
        rr[26]["raw_state"] = "unknown"
        rr[28]["raw_state"] = "clear"
        rr[31]["raw_state"] = "clear"
        r = calculate(rr)
        self.assertEqual(r["history_raw_later_status"], "clear_observed")
        self.assertEqual(r["history_raw_first_later_clear_offset_steps"], 3)
        self.assertEqual(r["history_raw_first_later_clear_delay_s"], 0.06)
        self.assertEqual(r["history_raw_first_later_clear_time_s"], 0.58)

    def test_future_unknown_without_clear_remains_unresolved(self):
        rr = slots()
        rr[27]["raw_state"] = "unknown"
        r = calculate(rr)
        self.assertEqual(r["history_raw_later_status"], "unresolved")
        self.assertIsNone(r["history_raw_first_later_clear_delay_s"])

    def test_truncated_future_all_blocked_is_not_a_full_half_second(self):
        rr = slots()
        for s in rr:
            if s["offset_steps"] > 2:
                s.update(
                    slot_status="at_or_after_next_loss", raw_state=None, mean_state=None
                )
        r = calculate(rr)
        self.assertEqual(r["history_raw_later_status"], "no_clear_observed")
        self.assertEqual(r["history_raw_post_blocked_slots"], 2)
        self.assertEqual(r["history_raw_post_missing_slots"], 23)
        self.assertEqual(r["history_missing_slots"], 23)

    def test_no_eligible_future_is_separate_from_no_clear(self):
        rr = slots()
        for s in rr:
            if s["offset_steps"] > 0:
                s.update(slot_status="after_source", raw_state=None, mean_state=None)
        r = calculate(rr)
        self.assertEqual(r["history_raw_later_status"], "no_future")
        self.assertEqual(r["history_raw_post_missing_slots"], 25)
        self.assertEqual(r["history_mean_later_status"], "current_not_blocked")

    def test_current_clear_and_unknown_keep_later_fields_blank(self):
        for state in ("clear", "unknown"):
            r = calculate(slots(raw=state))
            self.assertEqual(r["history_raw_later_status"], "current_not_blocked")
            self.assertIsNone(r["history_raw_first_later_clear_offset_steps"])
            self.assertIsNone(r["history_raw_first_later_clear_time_s"])

    def test_run_order_is_by_offset_and_invalid_grids_fail(self):
        rr = slots()
        self.assertEqual(calculate(rr), calculate(list(reversed(rr))))
        for bad in (rr[:-1], rr[:-1] + [rr[0]]):
            for fn in (a.history, v.history):
                with self.assertRaises(AssertionError):
                    fn(decision(), bad)

    def test_pre_and_post_states_include_missing_separately(self):
        rr = slots()
        rr[0].update(
            slot_status="before_source",
            context_phase="",
            raw_state=None,
            mean_state=None,
        )
        rr[1]["raw_state"] = "unknown"
        rr[2].update(raw_state="clear", context_phase="before_return")
        r = calculate(rr)
        self.assertEqual(r["history_raw_pre_clear_slots"], 1)
        self.assertEqual(r["history_raw_pre_unknown_slots"], 1)
        self.assertEqual(r["history_raw_pre_blocked_slots"], 22)
        self.assertEqual(r["history_raw_pre_missing_slots"], 1)
        self.assertEqual(r["history_before_return_slots"], 1)

    def test_raw_away_still_blocked_is_an_inherited_subgroup(self):
        r = calculate(slots(), dict(decision(), raw_distance_direction="away"))
        self.assertTrue(r["history_raw_away_blocked"])
        r = calculate(
            slots(raw="clear"), dict(decision(), raw_distance_direction="away")
        )
        self.assertFalse(r["history_raw_away_blocked"])


class GeometryMeasuresAndNative(unittest.TestCase):
    def test_instant_keeps_duplicate_coordinate_ids_and_exact_margin(self):
        d = dict(decision(), fixed_obstacle_ids="1;8", definite_obstacle_ids="1;8")
        snap = fixtures.context(3, 4, margin="4.000000000")
        obstacles = [("1", 0.0, 0.0), ("8", 0.0, 0.0)]
        actual, ww = a.instant(d, snap, obstacles)
        expected, other = v.instant(d, snap, obstacles)
        self.assertEqual(actual["raw_minimum_ids"], "1;8")
        self.assertEqual([w["obstacle_id"] for w in ww], ["1", "8"])
        self.assertEqual(actual["raw_gap_center"], 1)
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "instant.csv"
            a.write(p, [dict(id="1", **actual)])
            v.compare(p, [dict(id="1", **expected)], ("id",))
            wrong = dict(expected, logged_speed_margin="4.0")
            with self.assertRaises(AssertionError):
                v.compare(p, [dict(id="1", **wrong)], ("id",))
            a.write(p, ww)
            v.compare(p, other, ("obstacle_id",))

    def test_inherited_count_join_uses_exact_count_from_reconstructed_slot(self):
        d = decision()
        snap = fixtures.context(3, 4, margin="4.000000000")
        fields, _ = a.instant(d, snap, [("1", 0.0, 0.0)])
        for offset, role in ((-1, "previous"), (0, "current")):
            slot = dict(fields, offset_steps=offset, slot_status="observed")
            d["point_" + role + "_margin"] = fields["logged_speed_margin"]
            d["previous_count" if offset == -1 else "window_replayed_count"] = str(
                fields["window_count"]
            )
            for k in a.RAW:
                d[role + "_input_" + k] = fields["input_" + k]
            for kind in ("raw", "mean"):
                for axis, bound in itertools.product(
                    ("x", "y"), ("center", "lo", "hi")
                ):
                    d[f"point_{kind}_{role}_{axis}_{bound}"] = str(
                        fields[f"{kind}_{axis}_{bound}"]
                    )
                for quantity, bound in itertools.product(
                    ("distance", "gap"), ("center", "lo", "hi")
                ):
                    d[f"{kind}_{role}_{quantity}_{bound}"] = str(
                        fields[f"{kind}_{quantity}_{bound}"]
                    )
                d[f"{kind}_{role}_state"] = fields[kind + "_state"]
                for t in ("minimum", "possible_minimum"):
                    d[f"{kind}_{role}_{t}_ids"] = fields[f"{kind}_{t}_ids"]
            for fn in (a.check_previous, v.check_previous):
                fn(d, slot)
                with self.assertRaises(AssertionError):
                    fn(d, dict(slot, window_count=fields["window_count"] + 1))

    def test_nonreturns_and_original_unequal_measures(self):
        d = dict(decision(), **calculate(slots()), history_audited=True)
        d["history_raw_new_blocked"] = True
        d["history_raw_persisting_blocked"] = False
        events = [
            old_events.event(),
            dict(old_events.event(False), event_id="2"),
            dict(
                old_events.event(),
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
            history_raw_new_blocked=False,
            history_raw_persisting_blocked=True,
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
            cell["has_available_history_raw_new_blocked_fraction"], Fraction(1, 2)
        )
        self.assertEqual(
            cell["has_available_history_raw_new_blocked_case_mass_fraction"],
            Fraction(1, 3),
        )
        self.assertTrue(
            any(r["has_available_history_raw_new_blocked_fraction"] is None for r in rr)
        )
        cell["available_joint_history_audited_refs"] += 1
        with self.assertRaises(AssertionError):
            a.closure(rr)

    def test_native_context_mapping_and_point_bounds(self):
        rustc = (
            shutil.which("rustc")
            or "/workspace/.cloud-env/rust_robotics/rustup/toolchains/1.99.0-x86_64-unknown-linux-gnu/bin/rustc"
        )
        if not Path(rustc).exists():
            self.skipTest("rustc unavailable")
        d = dict(decision(), scenario="fixture")
        before, after = fixtures.context(
            3, 4, n=25, margin="4.000000000"
        ), fixtures.context(0, 2, n=26)
        contexts = {1: before, 2: after}
        mapping = [
            dict(
                event_id=d["event_id"],
                reference_time_s=d["time_s"],
                offset_steps=i - 2,
                source_time_s=c["point"]["source_step"] / 50,
            )
            for i, c in contexts.items()
        ]
        traces = {
            i: {
                "mean_x": c["fields"]["mean_x_center"],
                "mean_y": c["fields"]["mean_y_center"],
            }
            for i, c in contexts.items()
        }
        obstacles = {"fixture": [("1", 0.0, 0.0)]}
        with tempfile.TemporaryDirectory() as tmp:
            points, proof = v.native_points(
                Path(tmp),
                Path(rustc),
                mapping,
                {(d["event_id"], d["time_s"]): d},
                contexts,
                traces,
                obstacles,
            )
            self.assertEqual(proof["native_tests"], 6)
            self.assertEqual(proof["traced_point_obstacle_pairs"], 4)
            self.assertEqual(float(points[1, "1", "raw"][0]), 5)
            self.assertEqual(float(points[2, "1", "mean"][1]), 1)
            for i, c in contexts.items():
                native = {
                    (oid, k): points[i, oid, k]
                    for oid in ["1"]
                    for k in ("raw", "mean")
                }
                v.instant(d, c, obstacles["fixture"], native)


if __name__ == "__main__":
    unittest.main()
