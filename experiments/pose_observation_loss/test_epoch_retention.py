"""Pin inclusive TTL, reset order, original parents and observation horizons."""

import copy
from fractions import Fraction
import unittest

import analyze_epoch_retention as a


def history(epochs=(1, 251, 751), end=751):
    state = a.Metadata()
    snapshot = None
    latest = None
    for n in range(1, end + 1):
        if n in epochs:
            latest = n
        snapshot = state.advance(
            n, 0, (n - latest) / 50 if latest is not None else 10, n in epochs, False
        )
    return state, snapshot


def event(loss=True, forward=True):
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
    return dict(
        **meta,
        event_id="1",
        source_event_id="fresh_1",
        event_kind="received",
        event_time_s="150.02" if forward else "180.0",
        scheduled="True",
        received="True",
        precision_activation="False",
        immediate_actual="recovery_mean",
        immediate_preview="three_epoch",
        forward_steps="75" if forward else "0",
        horizon="full" if forward else "no_forward",
        preview_changes="1" if loss else "0",
        first_preview_change_offset_ms="20" if loss else "",
        first_preview_change_speed="0" if loss else "",
        first_preview_change_observation_age="0.02" if loss else "",
        first_preview_change_retained_groups="2" if loss else "",
        parent_rp_records="4",
        rp_records="1",
        rp_different_preview_records="1" if loss else "0",
        census_case_weight="1",
        received_case_weight="1"
    )


def snapshots(forward=True, loss=True):
    state, here = history()
    n = 7501 if forward else 9000
    delta = n - 751

    def shifted(snapshot):
        s = copy.deepcopy(snapshot)
        s["time_s"] += delta / 50
        if s["current_epoch_time_s"] is not None:
            s["current_epoch_time_s"] += delta / 50
        for key in (
            "epoch_steps",
            "prior_epoch_steps",
            "cleared_epoch_steps",
            "expired_epoch_steps",
            "capacity_evicted_epoch_steps",
        ):
            s[key] = ";".join(str(v + delta) for v in a.numbers(s[key]))
        return s

    result = {("1", "immediate"): shifted(here)}
    if loss:
        result[("1", "loss_before")] = shifted(here)
        result[("1", "loss_after")] = shifted(state.advance(752, 0, 0.02, False, False))
    return result


class RetainedHistory(unittest.TestCase):
    def test_ttl_inclusive_then_expires(self):
        state, here = history()
        self.assertTrue(here["fusion_available"])
        self.assertEqual(a.numbers(here["epoch_steps"]), (1, 251, 751))
        after = state.advance(752, 0, 0.02, False, False)
        self.assertFalse(after["fusion_available"])
        self.assertEqual(a.numbers(after["expired_epoch_steps"]), (1,))
        self.assertEqual(a.numbers(after["epoch_steps"]), (251, 751))
        self.assertEqual(a.operation(after), "ttl_expiry")

    def test_speed_boundary_is_inclusive(self):
        state, here = history((1, 251, 501), 501)
        self.assertTrue(here["fusion_available"])
        self.assertTrue(
            state.advance(502, 0.02, 0.02, False, False)["fusion_available"]
        )
        after = state.advance(503, 0.020000001, 0.04, False, False)
        self.assertFalse(after["fusion_available"])
        self.assertEqual(after["retained_groups"], 0)
        self.assertEqual(a.operation(after), "motion_clear")

    def test_motion_reset_preempts_due_expiry(self):
        state, _ = history()
        after = state.advance(752, 0.03, 0.02, False, False)
        self.assertEqual(a.numbers(after["cleared_epoch_steps"]), (1, 251, 751))
        self.assertEqual(after["expired_epoch_steps"], "")
        self.assertEqual(a.operation(after), "motion_clear")

    def test_precision_clears_then_adds_current_epoch(self):
        state, _ = history()
        after = state.advance(752, 0, 0, False, True)
        self.assertEqual(after["epoch_steps"], "752")
        self.assertEqual(after["sample_counts"], "1")
        self.assertEqual(after["expired_epoch_steps"], "")
        self.assertEqual(a.operation(after), "precision_clear")

    def test_combined_reset_keeps_both_flags(self):
        state, _ = history()
        after = state.advance(752, 0.03, 0, True, True)
        self.assertEqual(a.operation(after), "motion_and_precision_clear")
        self.assertTrue(after["moving"] and after["precision_activation"])

    def test_capacity_is_distinct_from_ttl(self):
        _, after = history((1, 251, 501, 751), 751)
        self.assertEqual(after["capacity_evicted_epoch_steps"], "1")
        self.assertEqual(after["expired_epoch_steps"], "")
        self.assertTrue(after["fusion_available"])
        self.assertEqual(a.numbers(after["epoch_steps"]), (251, 501, 751))

    def test_missed_observation_does_not_start_epoch(self):
        state, here = history((1, 251, 501), 501)
        after = state.advance(502, 0, 5, False, False)
        self.assertEqual(after["current_epoch_time_s"], here["current_epoch_time_s"])
        self.assertEqual(after["sample_counts"], here["sample_counts"])
        self.assertEqual(after["retained_groups"], 3)
        self.assertFalse(after["fusion_available"])
        self.assertEqual(a.operation(after), "age_gate")

    def test_older_sample_gate_and_single_current_sample(self):
        _, ready = history((1, 11, 21), 21)
        _, short = history((1, 10, 20), 20)
        self.assertEqual(ready["sample_counts"], "10;10;1")
        self.assertTrue(ready["fusion_available"])
        self.assertFalse(short["fusion_available"])
        self.assertEqual(a.operation(short), "older_samples_gate")

    def test_freshness_and_sample_cap(self):
        state, here = history((1,), 30)
        self.assertEqual(here["sample_counts"], "25")
        state = a.Metadata()
        state.advance(1, 0, 0, True, False)
        self.assertEqual(state.advance(2, 0, 0.5, False, False)["sample_counts"], "2")
        self.assertEqual(
            state.advance(3, 0, 0.500001, False, False)["sample_counts"], "2"
        )

    def test_availability_age_boundary(self):
        state, _ = history((1, 251, 501), 501)
        self.assertTrue(state.advance(502, 0, 1.5, False, False)["fusion_available"])
        self.assertFalse(
            state.advance(503, 0, 1.500001, False, False)["fusion_available"]
        )

    def test_invalid_grid_flags_and_nonfinite_inputs(self):
        for args in (
            (2, 0, 0, False, False),
            (1, float("nan"), 0, False, False),
            (1, 0, -1, False, False),
            (1, 0, 0, "false", False),
        ):
            with self.subTest(args=args), self.assertRaises(AssertionError):
                a.Metadata().advance(*args)

    def test_saved_snapshot_is_immutable_evidence(self):
        state, here = history()
        frozen = copy.deepcopy(here)
        state.advance(752, 0, 0.02, False, False)
        self.assertEqual(here, frozen)


class EventEvidence(unittest.TestCase):
    def test_duplicate_event_requests_fail(self):
        with self.assertRaises(AssertionError):
            a.requests([event(), event()])

    def test_invalid_or_clipped_loss_requests_fail(self):
        for value in ("0", "21", "1520"):
            with self.subTest(value=value), self.assertRaises(AssertionError):
                a.requests([{**event(), "first_preview_change_offset_ms": value}])
        with self.assertRaises(AssertionError):
            a.requests([event(forward=False)])

    def test_roles_keep_the_same_checkpoint_as_distinct_references(self):
        wanted = a.requests([event()])
        self.assertEqual(
            set(wanted["C1"][7501]), {("1", "immediate"), ("1", "loss_before")}
        )
        ee, ss, gg = a.build([event()], snapshots())
        self.assertEqual(len(ss), 3)
        self.assertEqual(len(gg), 8)
        self.assertEqual(ee[0]["loss_operation"], "ttl_expiry")
        self.assertEqual(ee[0]["immediate_oldest_ttl_ms"], 0)
        self.assertEqual(ee[0]["loss_expired_groups"], 1)

    def test_original_rp_parent_weight_survives_one_event_subset(self):
        ee, _, _ = a.build([event()], snapshots())
        self.assertEqual(ee[0]["rp_case_mass"], Fraction(1, 4))
        self.assertEqual(ee[0]["rp_different_preview_case_mass"], Fraction(1, 4))
        self.assertEqual(ee[0]["census_case_weight"], 1)

    def test_final_event_has_no_observed_next_step(self):
        ee, _, _ = a.build([event(False, False)], snapshots(False, False))
        self.assertIsNone(ee[0]["next_step_loss"])
        row = a.summarize([], ee, {})
        self.assertEqual(row["ttl_limit_events"], 1)
        self.assertEqual(row["eligible_ttl_limit_events"], 0)
        self.assertIsNone(row["ttl_limit_one_step_loss_fraction"])

    def test_empty_history_has_no_placeholder_groups(self):
        state = a.Metadata()
        s = state.advance(1, 0, 10, False, False)
        self.assertEqual(a.group_rows({}, s), [])

    def test_parent_mass_share_sums_before_division(self):
        rr, _, _ = a.build([event()], snapshots())
        lost = rr[0]
        stable = {
            **lost,
            "case_id": "C2",
            "preview_loss": False,
            "loss_operation": "no_observed_loss",
            "next_step_loss": False,
            "census_case_weight": Fraction(1, 10),
        }
        lost = {**lost, "census_case_weight": Fraction(1, 2)}
        summary = a.summarize([], [lost, stable], {})
        self.assertEqual(summary["ttl_limit_one_step_loss_fraction"], Fraction(1, 2))
        self.assertEqual(
            summary["ttl_limit_one_step_loss_case_mass_fraction"], Fraction(5, 6)
        )

    def test_empty_categories_and_additive_closure(self):
        rows = a.aggregate([], [])
        self.assertEqual(len(rows), 1344)
        self.assertGreater(a.closure(rows), 0)
        rows[0]["ttl_expiry_events"] = 1
        with self.assertRaises(AssertionError):
            a.closure(rows)


if __name__ == "__main__":
    unittest.main()
