"""Hand-calculated census/RP coverage, identifiers and parent-weight boundaries."""

import copy
from fractions import Fraction
import unittest

import analyze_census_rp_selection as a


def fixture(
    case_id="C1",
    mode="random",
    planner="A*",
    received=(0, 1),
    rp=((0, "filter_single", 0),),
    precision=False,
    held=False,
    preview="motion_filter",
    report="known",
):
    meta = dict(
        case_id=case_id,
        family_id="F1",
        cohort=report + "-white",
        variant="epoch",
        mode=mode,
        seed="269" if report == "known" else "274",
        scenario="maze_50x50",
        planner=planner,
        partition=report,
        accounting_partition="existing_new" if report == "new" else report,
        noise="white",
    )
    case = dict(meta, parent_records=str(len(rp)))
    account = {k: meta[k] for k in a.KEY}
    account.update(
        partition=meta["accounting_partition"],
        noise="white",
        status="held" if held else "timeout",
        held=str(held),
        collision="False",
        time_s="120.0" if held else "180.0",
        goal_distance="0.1" if held else "3.0",
        selected_timeout=str(not held),
        intervals="6000" if held else "9000",
        swept_contact_intervals="0",
        out_of_bounds_intervals="0",
    )
    events = []
    for i in range(0 if held else 7 + int(precision)):
        scheduled = i < 7
        arrived = scheduled and i in received
        t = 150 + i * 5 if scheduled else 150.5
        event = {k: meta[k] for k in a.KEY}
        event.update(
            event_id=case_id + "E" + str(i),
            source_event_id=case_id + "original" + str(i),
            partition=meta["accounting_partition"],
            noise="white",
            event_kind=(
                "received" if arrived else "missed" if scheduled else "precision_only"
            ),
            actual_estimator="recovery_mean",
            preview_estimator=preview,
            scheduled=str(scheduled),
            received=str(arrived),
            precision_activation=str(not scheduled),
            fusion_available=str(preview == "three_epoch"),
            recovery_sample_count="10",
            motion_filter_sample_count="10",
            epoch_fusion_group_count="3" if preview == "three_epoch" else "1",
            actual_used_sample_count="10",
            preview_used_sample_count="30" if preview == "three_epoch" else "10",
            event_time_s=str(t),
            truth_time_s=str(t - 0.02),
            observation_age="0.0",
            actual_x="0.0",
            actual_y="0.0",
            actual_yaw="0.0",
            preview_x="0.0",
            preview_y="0.0",
            preview_yaw="0.0",
        )
        events.append(event)
    mapping = []
    for i, (event, stratum, offset_us) in enumerate(rp):
        e = events[event]
        mapping.append(
            dict(
                meta,
                source_id=str(i),
                event_id=e["event_id"],
                stratum=stratum,
                time_s=str(float(e["event_time_s"]) + offset_us / 1_000_000),
                event_time_s=e["event_time_s"],
                case_row_weight=str(1 / len(rp)),
            )
        )
    return [case], [account], events, mapping


def built(**kwargs):
    return a.build(*fixture(**kwargs))


class CensusCoverageTests(unittest.TestCase):
    def test_history_group_count_is_not_the_selected_filter_group_count(self):
        inputs = fixture()
        inputs[2][0]["epoch_fusion_group_count"] = "2"
        _, events = a.build(*inputs)
        self.assertEqual(events[0]["epoch_fusion_group_count"], 2)
        self.assertFalse(events[0]["fusion_available"])
        self.assertEqual(events[0]["preview_estimator"], "motion_filter")

    def test_final_event_window_is_clipped_at_180_seconds(self):
        built(rp=((6, "filter_single", 0),))
        with self.assertRaises(AssertionError):
            built(rp=((6, "filter_single", 1),))

    def test_all_planned_selection_stages(self):
        for kwargs, stage in (
            (dict(held=True, rp=()), "held"),
            (dict(rp=()), "timeout_no_rp"),
            ({}, "timeout_with_rp"),
        ):
            with self.subTest(stage=stage):
                cases, _ = built(**kwargs)
                self.assertEqual(cases[0]["stage"], stage)
                self.assertEqual(cases[0]["case_id"], "C1")

    def test_held_case_has_no_late_census_without_changing_outcome(self):
        cases, events = built(held=True, rp=())
        self.assertFalse(events)
        self.assertTrue(cases[0]["held"])
        self.assertEqual(cases[0]["goal_distance"], 0.1)
        self.assertEqual(cases[0]["rp_case_mass"], 0)

    def test_census_and_rp_measures_remain_distinct(self):
        cases, events = built(rp=((0, "filter_single", 0), (1, "fusion_three", 20_000)))
        self.assertEqual(sum(e["census_case_weight"] for e in events), 1)
        self.assertEqual(sum(e["received_case_weight"] or 0 for e in events), 1)
        self.assertEqual(sum(e["rp_case_mass"] for e in events), 1)
        self.assertEqual(events[0]["census_case_weight"], Fraction(1, 7))
        self.assertEqual(events[0]["received_case_weight"], Fraction(1, 2))
        self.assertEqual(events[0]["rp_case_mass"], Fraction(1, 2))
        self.assertEqual(cases[0]["rp_records"], 2)

    def test_received_parent_weight_precedes_preview_selection(self):
        _, events = built()
        selected = [
            e
            for e in events
            if a.accepts(e, "received_any", "recovery_mean", "motion_filter")
        ]
        self.assertEqual(len(selected), 2)
        self.assertEqual(
            [e["received_case_weight"] for e in selected], [Fraction(1, 2)] * 2
        )

    def test_weighted_received_coverage_differs_from_raw_event_ratio(self):
        inputs = [fixture(), fixture(case_id="C2", planner="JPS", received=range(7))]
        cases, events = a.build(*(sum((v[i] for v in inputs), []) for i in range(4)))
        selected = [e for e in events if e["received"]]
        s = a.summarize(cases, selected, {})
        self.assertEqual(s["raw_event_coverage"], Fraction(2, 9))
        self.assertEqual(s["received_census_weighted_coverage"], Fraction(9, 28))

    def test_all_census_weights_include_precision_only_before_filtering(self):
        _, events = built(precision=True)
        self.assertEqual(len(events), 8)
        self.assertTrue(all(e["census_case_weight"] == Fraction(1, 8) for e in events))
        self.assertIsNone(events[-1]["received_case_weight"])
        self.assertEqual(events[-1]["event_kind"], "precision_only")

    def test_rp_stratum_shares_use_mass_not_raw_record_count(self):
        inputs = [
            fixture(rp=((0, "filter_single", 0), (1, "filter_single", 0))),
            fixture(case_id="C2", planner="JPS", rp=((0, "fusion_three", 0),)),
        ]
        cases, events = a.build(*(sum((v[i] for v in inputs), []) for i in range(4)))
        s = a.summarize(cases, events, {})
        self.assertEqual(s["rp_records"], 3)
        self.assertEqual(s["rp_filter_single_mass_share"], Fraction(1, 2))
        self.assertEqual(s["rp_fusion_three_mass_share"], Fraction(1, 2))

    def test_zero_coverage_and_empty_denominator_differ(self):
        cases, events = built(rp=())
        s = a.summarize(cases, events, {})
        self.assertEqual(s["raw_event_coverage"], 0)
        self.assertEqual(s["received_census_weighted_coverage"], 0)
        self.assertIsNone(s["rp_filter_single_mass_share"])
        empty = a.summarize(cases, [], {})
        self.assertIsNone(empty["raw_event_coverage"])
        self.assertIsNone(empty["received_census_weighted_coverage"])

    def test_immediate_preview_and_window_stratum_are_not_equated(self):
        _, events = built(
            preview="three_epoch",
            rp=((0, "filter_single", -20_000), (0, "fusion_three", 20_000)),
        )
        event = events[0]
        self.assertEqual(event["rp_same_preview_records"], 1)
        self.assertEqual(event["rp_different_preview_records"], 1)
        self.assertEqual(event["rp_other_preview_records"], 0)

    def test_other_planned_rp_strata_stay_explicit(self):
        _, events = built(
            rp=((0, "recovery_single", 0), (0, "recovery_fallback_single", 0))
        )
        self.assertEqual(events[0]["rp_other_preview_records"], 2)
        self.assertEqual(events[0]["rp_same_preview_records"], 0)

    def test_original_partition_alias_is_preserved(self):
        cases, events = built(report="new")
        self.assertEqual(cases[0]["partition"], "new")
        self.assertEqual(cases[0]["accounting_partition"], "existing_new")
        self.assertEqual(events[0]["event_partition"], "existing_new")

    def test_all_original_event_flag_combinations(self):
        for scheduled, received, precision, expected in (
            (True, True, False, "received"),
            (True, False, False, "missed"),
            (False, False, True, "precision_only"),
            (True, True, True, "received_and_precision"),
            (True, False, True, "missed_and_precision"),
        ):
            e = dict(
                scheduled=str(scheduled),
                received=str(received),
                precision_activation=str(precision),
            )
            self.assertEqual(a.event_kind(e), expected)

    def test_received_any_uses_received_flag(self):
        _, events = built()
        event = dict(
            events[0], event_kind="received_and_precision", precision_activation=True
        )
        self.assertTrue(a.accepts(event, "received_any", "all", "all"))
        self.assertFalse(a.accepts(event, "received", "all", "all"))

    def test_inclusive_source_window_and_microsecond_tolerance(self):
        _, events = built(
            rp=((0, "filter_single", -20_000), (0, "filter_single", 1_500_000))
        )
        self.assertEqual(
            (events[0]["min_event_offset_us"], events[0]["max_event_offset_us"]),
            (-20_000, 1_500_000),
        )
        for offset in (-20_001, 1_500_001):
            with self.assertRaises(AssertionError):
                built(rp=((0, "filter_single", offset),))
        self.assertEqual(a.microseconds("150.00000004"), 150_000_000)
        with self.assertRaises(AssertionError):
            a.microseconds("150.0000002")

    def test_reject_ambiguous_wrong_case_and_unknown_event_references(self):
        for mutation in (
            lambda m: m.update(event_id="unknown"),
            lambda m: m.update(mode="blackout"),
            lambda m: m.update(case_id="unknown"),
            lambda m: m.update(stratum="unknown"),
        ):
            inputs = fixture()
            mutation(inputs[3][0])
            with self.assertRaises(AssertionError):
                a.build(*inputs)

    def test_reject_duplicate_state_and_census_event(self):
        inputs = fixture()
        inputs[3].append(dict(inputs[3][0]))
        with self.assertRaises(AssertionError):
            a.build(*inputs)
        inputs = fixture()
        inputs[2].append(dict(inputs[2][0]))
        with self.assertRaises(AssertionError):
            a.build(*inputs)

    def test_reject_missing_scheduled_event(self):
        inputs = fixture()
        inputs[2].pop()
        with self.assertRaises(AssertionError):
            a.build(*inputs)

    def test_reject_held_case_with_census_or_inconsistent_timeout_flag(self):
        for change in (
            {"status": "held", "held": "True", "selected_timeout": "False"},
            {"selected_timeout": "False"},
        ):
            inputs = fixture()
            inputs[1][0].update(change)
            with self.assertRaises(AssertionError):
                a.build(*inputs)

    def test_reject_invalid_flag_kind_and_estimator(self):
        for change in (
            {"received": "True", "scheduled": "False"},
            {"event_kind": "unknown"},
            {"actual_estimator": "unknown"},
            {"preview_estimator": "unknown"},
            {"fusion_available": "True"},
        ):
            inputs = fixture()
            inputs[2][0].update(change)
            with self.assertRaises(AssertionError):
                a.build(*inputs)

    def test_reject_inconsistent_original_parent_weight(self):
        inputs = fixture()
        inputs[3][0]["case_row_weight"] = "0.5"
        with self.assertRaises(AssertionError):
            a.build(*inputs)

    def test_no_input_mutation(self):
        inputs = fixture(precision=True)
        before = copy.deepcopy(inputs)
        a.build(*inputs)
        self.assertEqual(inputs, before)

    def test_complete_grid_and_disjoint_additive_closures(self):
        cases, events = built()
        rows = a.aggregate(cases, events)
        self.assertEqual(len(rows), 3024)
        self.assertGreater(a.check_closure(rows), 0)
        empty = [r for r in rows if r["mode"] == "none" and r["partition"] == "known"]
        self.assertTrue(all(r["census_events"] == 0 for r in empty))
        self.assertTrue(all(r["raw_event_coverage"] is None for r in empty))


if __name__ == "__main__":
    unittest.main()
