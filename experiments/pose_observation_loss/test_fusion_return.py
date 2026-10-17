"""Test logged-return endpoints, censoring and original parent measures."""

import copy
from fractions import Fraction
import unittest

import analyze_fusion_return as a


def event(eid="1", time=150, loss=True, cid="C1"):
    row = dict(
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
    row.update(
        event_id=eid,
        source_event_id="fresh_" + eid,
        event_kind="received",
        event_time_s=str(time),
        scheduled="True",
        received="True",
        precision_activation="False",
        horizon="full" if time <= 178.5 else "no_forward" if time == 180 else "clipped",
        immediate_actual="recovery_mean",
        immediate_preview="three_epoch",
        preview_loss=str(loss),
        loss_operation="ttl_expiry" if loss else "no_observed_loss",
        loss_offset_ms="20" if loss else "",
        immediate_at_ttl_limit="True",
        has_forward_sample=str(time < 180),
        parent_rp_records="4",
        rp_records="2",
        rp_different_preview_records="1" if loss else "0",
    )
    return row


def point(n, available=False, received=False):
    epochs = (n - 500, n - 250, n) if available else (n - 250, n)
    return dict(
        fusion_available=available,
        retained_groups=len(epochs),
        epoch_steps=";".join(map(str, epochs)),
        sample_counts="25;25;1" if available else "25;2",
        received=received,
        precision_activation=False,
    )


def fixture():
    points = {n: point(n) for n in range(7500, 9001)}
    points[7500] = point(7500, True, True)
    opportunities = [event(str(i + 1), 150 + 5 * i, False) for i in range(7)]
    return event(), points, opportunities


def trace(e, points, opportunities):
    return a.followup(e, points, opportunities, 7, 3)


class ReturnBoundaries(unittest.TestCase):
    def test_first_later_checkpoint_is_a_return(self):
        e, points, opps = fixture()
        points[7502] = point(7502, True)
        out, _ = trace(e, points, opps)
        self.assertEqual(out["first_return_time_s"], 150.04)
        self.assertEqual(out["return_wait_s"], 0.02)
        self.assertEqual(out["first_return_spell_span_s"], 0.02)
        self.assertTrue(out["first_return_one_checkpoint"])

    def test_return_at_180_is_observed_and_spell_censored(self):
        e, points, opps = fixture()
        points[9000] = point(9000, True, True)
        out, _ = trace(e, points, opps)
        self.assertTrue(out["returned"])
        self.assertEqual(out["first_return_time_s"], 180)
        self.assertEqual(out["return_wait_s"], 29.98)
        self.assertEqual(out["first_return_spell_span_s"], 0)
        self.assertTrue(out["first_return_spell_right_censored"])
        self.assertIsNone(out["first_return_one_checkpoint"])
        self.assertEqual(out["return_category"], "returned_spell_censored")

    def test_no_return_has_lower_bound_and_blank_wait(self):
        out, _ = trace(*fixture())
        self.assertFalse(out["returned"])
        self.assertEqual(out["followup_status"], "right_censored")
        self.assertEqual(out["censor_wait_lower_bound_s"], 29.98)
        self.assertIsNone(out["return_wait_s"])
        self.assertIsNone(out["first_return_time_s"])
        self.assertEqual(out["return_category"], "no_return_censored")

    def test_loss_at_final_checkpoint_has_zero_observed_bound(self):
        _, points, opps = fixture()
        e = event(time=179.98)
        points[8999] = point(8999, True)
        out, refs = trace(e, points, opps)
        self.assertFalse(out["returned"])
        self.assertEqual(out["censor_wait_lower_bound_s"], 0)
        self.assertEqual(refs, [])

    def test_return_longer_than_one_checkpoint_with_observed_end(self):
        e, points, opps = fixture()
        for n in range(7750, 7775):
            points[n] = point(n, True)
        out, _ = trace(e, points, opps)
        self.assertEqual(out["return_category"], "returned_longer_ended")
        self.assertEqual(out["first_return_spell_span_s"], 0.5)
        self.assertFalse(out["first_return_one_checkpoint"])
        self.assertFalse(out["first_return_spell_right_censored"])

    def test_return_without_census_event_is_preserved(self):
        e, points, opps = fixture()
        points[7751] = point(7751, True)
        out, _ = trace(e, points, opps)
        self.assertEqual(out["first_return_time_s"], 155.02)
        self.assertIsNone(out["first_return_event_id"])
        self.assertFalse(out["first_return_received"])

    def test_censored_returned_spell_is_observed_span(self):
        e, points, opps = fixture()
        for n in range(7750, 9001):
            points[n] = point(n, True, n == 7750)
        out, _ = trace(e, points, opps)
        self.assertEqual(out["return_wait_s"], 4.98)
        self.assertEqual(out["first_return_spell_span_s"], 25)
        self.assertIsNone(out["first_return_one_checkpoint"])
        self.assertTrue(out["first_return_spell_right_censored"])

    def test_unobserved_ttl_at_180_is_not_a_censored_loss(self):
        _, points, opps = fixture()
        e = event(time=180, loss=False)
        points[9000] = point(9000, True, True)
        out, refs = trace(e, points, opps)
        self.assertEqual(out["followup_status"], "unobserved_ttl_limit")
        self.assertIsNone(out["returned"])
        self.assertIsNone(out["censor_wait_lower_bound_s"])
        self.assertEqual(refs, [])

    def test_original_loss_predecessor_and_time_are_checked(self):
        e, points, opps = fixture()
        for change in (
            {"loss_offset_ms": "21"},
            {"loss_offset_ms": "0"},
            {"loss_offset_ms": "1520"},
        ):
            with self.subTest(change=change), self.assertRaises(AssertionError):
                trace({**e, **change}, points, opps)
        points[7501] = point(7501, True)
        with self.assertRaises(AssertionError):
            trace(e, points, opps)


class OpportunityReferences(unittest.TestCase):
    def test_return_opportunity_inclusive_and_later_references_kept(self):
        e, points, opps = fixture()
        points[7750] = point(7750, True, True)
        out, refs = trace(e, points, opps)
        self.assertEqual(out["next_opportunity_event_id"], "2")
        self.assertEqual(out["next_received_event_id"], "2")
        self.assertEqual(out["first_return_event_id"], "2")
        self.assertEqual(out["future_opportunity_references"], 6)
        self.assertEqual(out["opportunities_through_endpoint"], 1)
        self.assertEqual(refs[0]["relation"], "at_or_before_first_return")
        self.assertEqual(refs[0]["offset_from_return_ms"], 0)
        self.assertEqual(refs[-1]["relation"], "after_first_return")
        self.assertEqual(refs[-1]["opportunity_time_s"], 180)

    def test_missed_opportunity_is_separate_from_next_received(self):
        e, points, opps = fixture()
        opps[1] = {**opps[1], "received": "False", "event_kind": "missed"}
        points[8000] = point(8000, True, True)
        out, _ = trace(e, points, opps)
        self.assertEqual(out["next_opportunity_event_id"], "2")
        self.assertEqual(out["next_received_event_id"], "3")
        self.assertEqual(out["opportunities_through_endpoint"], 2)
        self.assertEqual(out["missed_through_endpoint"], 1)
        self.assertEqual(out["received_through_endpoint"], 1)

    def test_precision_only_and_received_flags_are_distinct(self):
        e, points, opps = fixture()
        opps[1] = {
            **opps[1],
            "scheduled": "False",
            "received": "False",
            "precision_activation": "True",
            "event_kind": "precision_only",
        }
        points[8000] = point(8000, True)
        out, _ = trace(e, points, opps)
        self.assertEqual(out["opportunities_through_endpoint"], 2)
        self.assertEqual(out["received_through_endpoint"], 1)
        self.assertEqual(out["missed_through_endpoint"], 0)
        self.assertEqual(out["precision_through_endpoint"], 1)

    def test_duplicate_or_wrong_case_opportunity_is_rejected(self):
        e, points, opps = fixture()
        bad = [
            opps + [opps[1]],
            opps + [{**opps[1], "event_time_s": "155.00", "event_id": "99"}],
            [*opps[:1], {**opps[1], "mode": "none"}, *opps[2:]],
            [*opps[:1], {**opps[1], "case_id": "C2"}, *opps[2:]],
        ]
        for rows in bad:
            with self.assertRaises(AssertionError):
                trace(e, points, rows)

    def test_repeated_loss_anchors_can_reuse_future_events(self):
        e, points, opps = fixture()
        points[7750] = point(7750, True, True)
        for n in range(8000, 9001):
            points[n] = point(n, True, n == 8000)
        first, refs1 = trace(e, points, opps)
        second, refs2 = trace(event("2", 155), points, opps)
        self.assertEqual(first["first_return_time_s"], 155)
        self.assertTrue(first["first_return_one_checkpoint"])
        self.assertEqual(second["first_return_time_s"], 160)
        self.assertTrue(second["first_return_spell_right_censored"])
        self.assertIn("3", {r["opportunity_event_id"] for r in refs1})
        self.assertIn("3", {r["opportunity_event_id"] for r in refs2})
        self.assertEqual(first["census_case_weight"], Fraction(1, 7))
        self.assertEqual(second["census_case_weight"], Fraction(1, 7))
        self.assertEqual(a.summarize([], [first, second], {})["returned_cases"], 1)

    def test_input_nonmutation(self):
        args = fixture()
        frozen = copy.deepcopy(args)
        trace(*args)
        self.assertEqual(args, frozen)


class RunsAndMeasures(unittest.TestCase):
    def test_maximal_runs_reconstruct_inclusive_checkpoints(self):
        points = {n: point(n, n in (7501, 7502, 7504)) for n in range(7500, 7505)}
        runs = a.availability_runs(points, 7500, 7504)
        self.assertEqual(
            runs,
            [
                (7500, 7500, False),
                (7501, 7502, True),
                (7503, 7503, False),
                (7504, 7504, True),
            ],
        )
        rebuilt = {n: v for lo, hi, v in runs for n in range(lo, hi + 1)}
        self.assertEqual(rebuilt, {n: p["fusion_available"] for n, p in points.items()})

    def test_missing_extra_or_noncanonical_grid_is_rejected(self):
        points = {n: point(n) for n in range(7500, 7505)}
        for bad in (
            {n: p for n, p in points.items() if n != 7502},
            {**points, 7505: point(7505)},
            {**points, 7501: {**points[7501], "fusion_available": "False"}},
        ):
            with self.assertRaises(AssertionError):
                a.availability_runs(bad, 7500, 7504)

    def test_rp_parent_mass_and_received_weight_precede_selection(self):
        out, refs = trace(*fixture())
        self.assertEqual(out["rp_case_mass"], Fraction(1, 2))
        self.assertEqual(out["rp_different_preview_case_mass"], Fraction(1, 4))
        self.assertEqual(out["census_case_weight"], Fraction(1, 7))
        self.assertEqual(out["received_case_weight"], Fraction(1, 3))
        self.assertTrue(
            all(r["anchor_census_case_weight"] == Fraction(1, 7) for r in refs)
        )

    def test_sum_mass_before_return_fraction(self):
        e, points, opps = fixture()
        points[7750] = point(7750, True)
        returned, _ = trace(e, points, opps)
        points[7750] = point(7750)
        censored, _ = trace(e, points, opps)
        censored["census_case_weight"] = Fraction(1, 2)
        row = a.summarize([], [returned, censored], {})
        self.assertEqual(row["return_fraction"], Fraction(1, 2))
        self.assertEqual(row["return_case_mass_fraction"], Fraction(2, 9))

    def test_blank_return_denominator_distinct_from_zero_returns(self):
        empty = a.summarize([], [], {})
        censored, _ = trace(*fixture())
        observed = a.summarize([], [censored], {})
        self.assertIsNone(empty["return_fraction"])
        self.assertEqual(observed["return_fraction"], 0)
        self.assertIsNone(observed["min_observed_return_wait_s"])
        self.assertEqual(observed["min_censored_wait_lower_bound_s"], 29.98)

    def test_all_empty_cells_and_exact_closure(self):
        rows = a.aggregate([], [])
        self.assertEqual(len(rows), 528)
        self.assertGreater(a.closure(rows), 0)
        rows[0]["returned_case_mass"] = Fraction(1, 2)
        with self.assertRaises(AssertionError):
            a.closure(rows)


if __name__ == "__main__":
    unittest.main()
