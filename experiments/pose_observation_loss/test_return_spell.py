"""Test returned-spell boundaries, gate evidence, isolation and fixed parents."""

import copy
import csv
from fractions import Fraction
from pathlib import Path
import tempfile
import unittest

import analyze_return_spell as a
import verify_return_spell as v


def point(n, available=False, epochs=None, age=0, received=False):
    epochs = epochs or ((n - 500, n - 250, n) if available else (n - 250, n))
    counts = (25,) * (len(epochs) - 1) + (1,)
    return dict(
        time_s=n / 50,
        current_epoch_time_s=epochs[-1] / 50,
        measured_speed=0.0,
        observation_age=age,
        received=received,
        precision_activation=False,
        moving=False,
        fusion_available=available,
        retained_groups=len(epochs),
        epoch_steps=";".join(map(str, epochs)),
        sample_counts=";".join(map(str, counts)),
        prior_epoch_steps="",
        prior_sample_counts="",
        cleared_epoch_steps="",
        expired_epoch_steps="",
        capacity_evicted_epoch_steps="",
    )


def fixture(start=7750, finish=7751):
    meta = dict(
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
    e = dict(
        **meta,
        event_id="1",
        source_event_id="fresh_1",
        event_kind="received",
        event_time_s="150",
        scheduled="True",
        received="True",
        precision_activation="False",
        horizon="full",
        immediate_actual="recovery_mean",
        immediate_preview="three_epoch",
        preview_loss="True",
        loss_operation="ttl_expiry",
        loss_offset_ms="20",
        loss_time_s="150.02",
        returned="True",
        censor_wait_lower_bound_s="",
        first_return_time_s=str(start / 50),
        first_return_spell_end_time_s=str((finish if finish else 9000) / 50),
        first_return_spell_right_censored=str(finish is None),
        return_category=(
            "returned_one_checkpoint"
            if finish == start + 1
            else "returned_longer_ended" if finish else "returned_spell_censored"
        ),
        followup_status="returned",
        ttl_limit_without_next_step="False",
        parent_rp_records="8",
        rp_records="2",
        rp_different_preview_records="1",
    )
    opps = [
        dict(
            e,
            event_id=str(i + 1),
            source_event_id=f"fresh_{i+1}",
            event_time_s=str(150 + i * 5),
        )
        for i in range(7)
    ]
    points = {n: point(n, received=n % 250 == 0) for n in range(7500, 9001)}
    points[7500] = point(7500, True, received=True)
    epochs = (
        (start - 750, start - 250, start)
        if finish == start + 1
        else (start - 500, start - 250, start)
    )
    for n in range(start, finish if finish else 9001):
        points[n] = point(n, True, epochs, (n - start) / 50, n % 250 == 0)
    if finish:
        if finish == start + 1:
            points[finish]["expired_epoch_steps"] = str(epochs[0])
        else:
            points[finish] = point(finish, False, epochs, (finish - start) / 50)
    return e, points, opps


def collect(f):
    return a.anchor(*f, 7, 3)


class ReturnedBoundaries(unittest.TestCase):
    def test_ttl_limit_is_inclusive_then_expires(self):
        r, ss, gg = collect(fixture())
        self.assertEqual(r["return_oldest_ttl_ms"], 0)
        self.assertEqual(r["conditional_ttl_first_invalid_offset_ms"], 20)
        self.assertTrue(r["one_checkpoint"])
        self.assertTrue(r["end_at_ttl_deadline"])
        self.assertEqual(r["end_operation"], "ttl_expiry")
        self.assertEqual([s["role"] for s in ss], ["return", "end_before", "end_after"])
        self.assertEqual(
            next(g["ttl_remaining_ms"] for g in gg if g["role"] == "return"), 0
        )

    def test_age_gate_is_inclusive_at_1500_ms(self):
        e, pp, oo = fixture(finish=7826)
        r, ss, _ = collect((e, pp, oo))
        self.assertEqual(r["conditional_age_first_invalid_offset_ms"], 1520)
        self.assertEqual(r["end_operation"], "age_gate")
        self.assertTrue(r["end_at_age_deadline"])
        self.assertFalse(r["end_at_ttl_deadline"])
        self.assertEqual(ss[1]["observation_age"], 1.5)
        self.assertFalse(ss[1]["age_blocked"])
        self.assertTrue(ss[2]["age_blocked"])
        self.assertFalse(ss[2]["group_count_blocked"])

    def test_conditional_age_deadline_boundaries(self):
        self.assertEqual(a.age_deadline_ms(0), 1520)
        self.assertEqual(a.age_deadline_ms(1.5), 20)
        self.assertEqual(a.age_deadline_ms(1.5000000005), 20)
        for age in (-0.01, 1.52, float("nan"), float("inf")):
            with self.subTest(age=age), self.assertRaises(AssertionError):
                a.age_deadline_ms(age)

    def test_endpoint_return_with_zero_ttl_is_unknown_after_log(self):
        e, pp, oo = fixture(start=9000, finish=None)
        pp[9000] = point(9000, True, (8250, 8750, 9000), received=True)
        r, ss, _ = collect((e, pp, oo))
        self.assertEqual(r["returned_spell_span_s"], 0)
        self.assertTrue(r["return_ttl_limit"])
        self.assertTrue(r["censored_return"])
        for k in (
            "end_time_s",
            "end_operation",
            "one_checkpoint",
            "end_at_ttl_deadline",
            "end_vs_next_received",
        ):
            self.assertIsNone(r[k])
        self.assertEqual([s["time_s"] for s in ss], [180, 180])
        self.assertIsNone(r["next_received_time_s"])

    def test_no_return_preserves_separate_lower_bound(self):
        e, pp, oo = fixture()
        pp[7750] = point(7750)
        e.update(
            returned="False",
            censor_wait_lower_bound_s="29.98",
            followup_status="right_censored",
            return_category="no_return_censored",
        )
        r, ss, gg = collect((e, pp, oo))
        self.assertTrue(r["no_return"])
        self.assertEqual(r["censor_wait_lower_bound_s"], 29.98)
        self.assertIsNone(r["returned_spell_span_s"])
        self.assertIsNone(r["end_operation"])
        self.assertEqual((ss, gg), ([], []))

    def test_original_unobserved_ttl_limit_is_outside_loss_denominator(self):
        e, pp, oo = fixture()
        e.update(
            event_time_s="180",
            preview_loss="False",
            returned="",
            loss_offset_ms="",
            loss_time_s="",
            ttl_limit_without_next_step="True",
            return_category="",
            followup_status="unobserved_ttl_limit",
        )
        pp[9000] = point(9000, True, received=True)
        r, ss, gg = collect((e, pp, oo))
        self.assertFalse(r["preview_loss"])
        self.assertIsNone(r["returned"])
        self.assertIsNone(r["ended"])
        self.assertFalse(r["no_return"])
        self.assertEqual((ss, gg), ([], []))


class GatesAndOpportunities(unittest.TestCase):
    def test_priority_retains_simultaneous_gate_evidence(self):
        e, pp, oo = fixture()
        s = pp[7751]
        s.update(
            moving=True,
            measured_speed=0.021,
            precision_activation=True,
            observation_age=1.52,
            cleared_epoch_steps="7000;7500;7750",
            capacity_evicted_epoch_steps="7000",
        )
        r, _, _ = collect((e, pp, oo))
        self.assertEqual(r["end_operation"], "motion_and_precision_clear")
        for k in (
            "end_moving",
            "end_precision_activation",
            "end_age_blocked",
            "end_ttl_expired",
            "end_capacity_evicted",
        ):
            self.assertTrue(r[k])

    def test_all_ordered_operations_match_independent_classifier(self):
        for op, change in (
            (
                "motion_and_precision_clear",
                dict(moving=True, precision_activation=True),
            ),
            ("motion_clear", dict(moving=True)),
            ("precision_clear", dict(precision_activation=True)),
            ("ttl_expiry", dict(expired_epoch_steps="7000")),
            ("capacity_eviction", dict(capacity_evicted_epoch_steps="7000")),
            ("age_gate", dict(observation_age=1.52)),
            ("latest_epoch_gate", dict(current_epoch_time_s=155.02)),
            ("older_samples_gate", dict(sample_counts="9;25;1")),
            (
                "other_gate",
                dict(retained_groups=2, epoch_steps="7500;7750", sample_counts="25;1"),
            ),
        ):
            with self.subTest(op=op):
                s = dict(point(7751, True, (7000, 7500, 7750)), **change)
                self.assertEqual(a.operation(s), op)
                self.assertEqual(v.end_operation(s, v.gate_evidence(s)), op)

    def test_next_receipt_is_strictly_after_return(self):
        r, _, _ = collect(fixture())
        self.assertEqual(r["return_event_id"], "2")
        self.assertEqual(r["next_received_event_id"], "3")
        self.assertEqual(r["next_received_offset_ms"], 5000)
        self.assertEqual(r["end_vs_next_received"], "before")

    def test_missed_next_opportunity_is_separate_from_next_receipt(self):
        e, pp, oo = fixture()
        oo[2].update(received="False", event_kind="missed")
        r, _, _ = collect((e, pp, oo))
        self.assertEqual(r["next_opportunity_event_id"], "3")
        self.assertEqual(r["next_opportunity_kind"], "missed")
        self.assertEqual(r["next_received_event_id"], "4")
        self.assertEqual(r["next_received_offset_ms"], 10000)

    def test_no_future_received_event_is_explicit(self):
        e, pp, oo = fixture()
        for o in oo[2:]:
            o.update(received="False", event_kind="missed")
        r, _, _ = collect((e, pp, oo))
        self.assertEqual(r["end_vs_next_received"], "no_next_received_in_log")
        self.assertIsNone(r["next_received_time_s"])

    def test_same_checkpoint_and_after_relations(self):
        for finish, relation in ((8000, "same_checkpoint"), (8001, "after")):
            with self.subTest(finish=finish):
                r, _, _ = collect(fixture(finish=finish))
                self.assertEqual(r["end_vs_next_received"], relation)

    def test_case_and_duplicate_opportunity_isolation(self):
        e, pp, oo = fixture()
        for change in (dict(case_id="C2"), dict(mode="none"), dict(seed="278")):
            with self.subTest(change=change), self.assertRaises(AssertionError):
                collect((e, pp, [dict(oo[0], **change), *oo[1:]]))
        with self.assertRaises(AssertionError):
            collect((e, pp, [*oo, oo[-1]]))

    def test_reused_return_checkpoint_has_separate_anchor_references(self):
        e, pp, oo = fixture()
        r1, ss1, _ = collect((e, pp, oo))
        r2, ss2, _ = collect(
            (dict(e, event_id="99", source_event_id="fresh_99"), pp, oo)
        )
        self.assertEqual(r1["return_time_s"], r2["return_time_s"])
        self.assertNotEqual(ss1[0]["event_id"], ss2[0]["event_id"])

    def test_inputs_are_not_mutated(self):
        f = fixture()
        before = copy.deepcopy(f)
        collect(f)
        self.assertEqual(f, before)


class CacheAndAccounting(unittest.TestCase):
    def test_cache_rejects_tampering_and_grid_or_case_errors(self):
        members = [dict(case_id="C1", selected_timeout="True")]
        rows = []
        for n in range(7500, 9001):
            s = point(n)
            s = {k: str(v).lower() if isinstance(v, bool) else v for k, v in s.items()}
            rows.append(dict(event_id="C1", role=f"late_{n}", case_id="C1", **s))
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp) / "trace.csv"

            def save(rr):
                with p.open("w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=rows[0])
                    w.writeheader()
                    w.writerows(rr)

            save(rows)
            pinned = a.digest(p)
            self.assertEqual(len(a.cached_trace(p, members, pinned)["C1"]), 1501)
            p.write_text(p.read_text() + "\n")
            with self.assertRaisesRegex(AssertionError, "hash mismatch"):
                a.cached_trace(p, members, pinned)
            for rr in (
                rows[:-1],
                [*rows, rows[-1]],
                [dict(rows[0], case_id="C2"), *rows[1:]],
            ):
                save(rr)
                with self.assertRaises(AssertionError):
                    a.cached_trace(p, members, a.digest(p))

    def test_fixed_parents_precede_endpoint_selection(self):
        e, pp, oo = fixture()
        r1, _, _ = a.anchor(e, pp, oo, 7, 3)
        r2, _, _ = a.anchor(
            dict(e, event_id="2", case_id="C2"),
            pp,
            [dict(o, case_id="C2") for o in oo],
            8,
            4,
        )
        r2["ended"], r2["censored_return"] = False, True
        r2["one_checkpoint"], r2["end_at_ttl_deadline"], r2["end_at_age_deadline"] = (
            None,
            None,
            None,
        )
        rr = a.summarize([], [r1, r2], {})
        self.assertEqual(rr["returned_case_mass"], Fraction(1, 7) + Fraction(1, 8))
        self.assertEqual(rr["ended_case_mass"], Fraction(1, 7))
        self.assertEqual(rr["received_case_mass"], Fraction(1, 3) + Fraction(1, 4))
        self.assertEqual(rr["rp_case_mass"], Fraction(1, 2))
        self.assertEqual(rr["ttl_expiry_end_case_mass_fraction"], 1)

    def test_empty_end_denominator_is_blank_and_observed_zero_is_zero(self):
        empty = a.summarize([], [], {})
        self.assertIsNone(empty["ttl_expiry_end_fraction"])
        self.assertEqual(empty["ended_events"], 0)
        r, _, _ = collect(fixture())
        actual = a.summarize([], [r], {})
        self.assertEqual(actual["age_gate_end_fraction"], 0)
        self.assertEqual(actual["age_gate_end_case_mass_fraction"], 0)

    def test_all_528_cells_close_without_adding_distinct_case_counts(self):
        r, _, _ = collect(fixture())
        case = {k: r[k] for k in a.META}
        case.update(selected_timeout="True", held="False")
        rows = a.aggregate([case], [r])
        self.assertEqual(len(rows), 528)
        self.assertGreater(a.closure(rows), 0)
        independent = v.summaries([case], {(r["event_id"],): r})
        self.assertEqual(a.indexed(rows, a.GROUP), independent)
        self.assertEqual(a.closure(rows), v.closures(independent))


if __name__ == "__main__":
    unittest.main()
