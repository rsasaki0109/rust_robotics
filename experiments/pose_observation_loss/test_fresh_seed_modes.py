"""Regression checks for census denominators, mode support and event integrity."""

from copy import deepcopy
import unittest

from analyze_fresh_seed_modes import (
    EVENT_GROUPS,
    accounting,
    event_record,
    outcomes,
    summary,
    support,
    validate_events,
)


def case(scenario="open_50x50", timeout=True):
    return accounting(
        dict(
            cohort="fresh-white",
            variant="epoch",
            mode="none",
            seed="277",
            scenario=scenario,
            planner="A*",
            status="timeout" if timeout else "held",
            held=not timeout,
            collision=False,
            time_s=180 if timeout else 120,
            goal_distance=10,
            intervals=9000 if timeout else 6000,
            swept_contact_intervals=0,
            out_of_bounds_intervals=0,
        )
    )


def event(c, time=150, estimator="motion_filter", received=True, precision=False):
    scheduled = not precision
    received = received and scheduled
    kind = "precision_only" if precision else "received" if received else "missed"
    a = dict(
        estimator="raw_fallback",
        used_sample_count=1,
        source_group_count=1,
        recovery_sample_count=0,
        time_s=time,
        truth_time_s=time - 0.02,
        pose_x=1,
        pose_y=2,
        pose_yaw=0.1,
    )
    p = dict(
        a,
        estimator=estimator,
        used_sample_count=51 if estimator == "three_epoch" else 1,
        source_group_count=3 if estimator == "three_epoch" else 1,
        fusion_available=estimator == "three_epoch",
        motion_filter_sample_count=1,
        epoch_fusion_group_count=3,
        observation_age=0,
    )
    e = dict(
        event_time_s=time,
        event_kind=kind,
        received=received,
        scheduled=scheduled,
        precision_activation=precision,
    )
    return event_record(f'{c["scenario"]}_{time}', c, e, a, p)


def selected(rows, kind="all", mode="all", seed="all"):
    return next(
        r
        for r in rows
        if r["cohort"] == "fresh-white"
        and r["variant"] == "epoch"
        and r["seed"] == seed
        and r["mode"] == mode
        and r["event_group"] == kind
    )


class FreshSeedCensusTests(unittest.TestCase):
    def test_parent_weights_differ_from_event_fraction_and_keep_mixed_cases(self):
        a, b = case(), case("dense_50x50")
        events = [
            event(a, 150),
            event(a, 155),
            event(a, 160, "three_epoch"),
            event(b, 150, "three_epoch"),
        ]
        r = selected(summary([a, b], events))
        self.assertEqual(
            (
                r["events"],
                r["event_cases"],
                r["filter_cases"],
                r["fusion_cases"],
                r["mixed_cases"],
            ),
            (4, 2, 1, 2, 1),
        )
        self.assertEqual(r["filter_event_share"], 0.5)
        self.assertAlmostEqual(r["filter_case_weighted_share"], 1 / 3)
        self.assertAlmostEqual(r["fusion_case_weighted_share"], 2 / 3)
        self.assertAlmostEqual(r["filter_case_mass"] + r["fusion_case_mass"], 2)

    def test_empty_groups_and_no_event_cases_stay_explicit(self):
        rows = summary([case(), case("dense_50x50", False)], [])
        self.assertEqual(len(rows), 6 * 4 * 2 * 4 * len(EVENT_GROUPS))
        r = selected(rows)
        self.assertEqual(
            (
                r["accounting_cases"],
                r["timeout_cases"],
                r["non_timeout_cases"],
                r["no_event_timeout_cases"],
            ),
            (2, 1, 1, 1),
        )
        self.assertEqual(r["events"], 0)
        for label in ["filter", "fusion"]:
            self.assertEqual(r[label + "_event_share"], "")
            self.assertEqual(r[label + "_case_weighted_share"], "")
            self.assertEqual(r[label + "_case_mass"], 0)

    def test_receipt_union_includes_received_and_precision_once(self):
        c = case()
        a, b, m = event(c), event(c, 155, precision=True), event(c, 160, received=False)
        b.update(event_kind="received_and_precision", scheduled=True, received=True)
        r = selected(summary([c], [a, b, m]), "received_any")
        self.assertEqual(r["events"], 2)
        self.assertEqual(r["filter_case_weighted_share"], 1)
        self.assertEqual(selected(summary([c], [a, b, m]), "received")["events"], 1)

    def test_case_support_exposes_non_disjoint_modes_and_absent_groups(self):
        c = case()
        rows = support([c], [event(c), event(c, 155, "three_epoch")])
        all_row = next(r for r in rows if r["event_group"] == "all")
        self.assertEqual(
            (all_row["support"], all_row["filter_events"], all_row["fusion_events"]),
            ("mixed", 1, 1),
        )
        self.assertEqual(
            next(r for r in rows if r["event_group"] == "missed")["support"],
            "no_events",
        )

    def test_loss_modes_and_seeds_do_not_leak(self):
        a, b = case(), case("dense_50x50")
        b.update(seed="278", mode="blackout")
        rows = summary([a, b], [event(a), event(b, estimator="three_epoch")])
        r = selected(rows, mode="none", seed="277")
        self.assertEqual(
            (r["accounting_cases"], r["events"], r["filter_cases"], r["fusion_cases"]),
            (1, 1, 1, 0),
        )
        self.assertEqual(selected(rows, mode="none", seed="278")["events"], 0)

    def test_all_seven_scheduled_events_and_extra_precision_are_kept(self):
        c = case()
        rows = [event(c, t) for t in range(150, 181, 5)]
        rows.append(event(c, 151, precision=True))
        validate_events([c], rows)
        self.assertEqual(rows[-2]["event_time_s"], 180)
        self.assertEqual(rows[-2]["truth_time_s"], 179.98)

    def test_missing_endpoint_is_rejected(self):
        c = case()
        with self.assertRaisesRegex(AssertionError, "scheduled timeout coverage"):
            validate_events([c], [event(c, t) for t in range(150, 180, 5)])

    def test_duplicate_event_id_or_time_is_rejected(self):
        c = case()
        rows = [event(c, t) for t in range(150, 181, 5)]
        for duplicate in [dict(rows[0]), dict(rows[0], event_id="different")]:
            with self.subTest(duplicate=duplicate["event_id"]):
                with self.assertRaisesRegex(AssertionError, "duplicate event"):
                    validate_events([c], [*rows, duplicate])

    def test_non_timeout_events_and_duplicate_accounting_are_rejected(self):
        c = case()
        with self.assertRaises(AssertionError):
            event(case(timeout=False))
        with self.assertRaisesRegex(AssertionError, "duplicate case"):
            validate_events([c, deepcopy(c)], [])

    def test_combined_event_flags_match_kind_and_truth_is_prior_row(self):
        c = case()
        a = dict(
            estimator="recovery_mean",
            used_sample_count=1,
            source_group_count=1,
            recovery_sample_count=1,
            time_s=150,
            truth_time_s=149.98,
            pose_x=0,
            pose_y=0,
            pose_yaw=0,
        )
        p = dict(
            a,
            estimator="motion_filter",
            fusion_available=False,
            motion_filter_sample_count=1,
            epoch_fusion_group_count=1,
            observation_age=0,
        )
        e = dict(
            event_time_s=150,
            event_kind="received_and_precision",
            received=True,
            scheduled=True,
            precision_activation=True,
        )
        row = event_record("a", c, e, a, p)
        self.assertEqual(row["actual_estimator"], "recovery_mean")
        with self.assertRaises(AssertionError):
            event_record("a", c, dict(e, event_kind="received"), a, p)
        with self.assertRaises(AssertionError):
            event_record("a", c, e, dict(a, truth_time_s=150), p)

    def test_unavailable_or_inconsistent_mode_is_rejected(self):
        c = case()
        a = dict(
            estimator="raw_fallback",
            used_sample_count=1,
            source_group_count=1,
            recovery_sample_count=0,
            time_s=150,
            truth_time_s=149.98,
            pose_x=0,
            pose_y=0,
            pose_yaw=0,
        )
        p = dict(
            a,
            estimator="motion_filter",
            fusion_available=False,
            motion_filter_sample_count=1,
            epoch_fusion_group_count=1,
            observation_age=0,
        )
        e = dict(
            event_time_s=150,
            event_kind="received",
            received=True,
            scheduled=True,
            precision_activation=False,
        )
        for bad in [
            dict(p, available=False),
            dict(p, estimator="three_epoch"),
            dict(p, source_group_count=3),
        ]:
            with self.assertRaises(AssertionError):
                event_record("a", c, e, a, bad)
        with self.assertRaises(AssertionError):
            event_record("a", c, e, dict(a, used_sample_count=2), p)

    def test_seed_and_status_integrity_in_accounting(self):
        for changes in [dict(seed="276"), dict(held=True), dict(intervals=8999)]:
            c = case()
            c.update(changes)
            with self.assertRaises(AssertionError):
                accounting(c)

    def test_gained_and_lost_holds_are_both_reported(self):
        a, b = case(), case("dense_50x50", False)
        pairs = [
            dict(
                cohort="fresh-white",
                seed="277",
                mode="none",
                gained_hold=gain,
                lost_hold=not gain,
                both_held=False,
                matched_wait_difference_s=wait,
                held_time_difference_s="",
                first_command_change_s=5,
            )
            for gain, wait in [(True, -3), (False, 5)]
        ]
        r = next(
            r
            for r in outcomes([a, b], pairs)
            if r["cohort"] == "fresh-white"
            and r["seed"] == "277"
            and r["mode"] == "none"
            and r["variant"] == "epoch"
        )
        self.assertEqual(
            (r["held"], r["timeout"], r["gained_holds"], r["lost_holds"]), (1, 1, 1, 1)
        )
        self.assertEqual(r["mean_both_held_time_delta_s"], "")
        self.assertEqual(r["mean_matched_wait_delta_s"], 1)


if __name__ == "__main__":
    unittest.main()
