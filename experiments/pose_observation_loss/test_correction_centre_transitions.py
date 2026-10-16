"""Time alignment, complete opportunity accounting, paired errors and censoring."""

from collections import deque
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import analyze_correction_centre_transitions as analysis
from analyze_epoch_position_components import Replay, Sample


def context(route="joint", ready=False, resumed=False, released=False, positive=False):
    return dict(
        route=route,
        active_ready=ready,
        joint_base_usable=route == "joint",
        resumed=resumed,
        guard_released=released,
        positive_target=positive,
    )


def event(event_id=1):
    return dict(
        event_id=event_id,
        cohort="known-white",
        variant="epoch",
        mode="random",
        seed="269",
        scenario="fixture",
        planner="A*",
        event_kind="received",
        event_time_s="150.000000",
        event_route="joint",
        scheduled=True,
        received=True,
        precision_activation=False,
        immediate_resumed=False,
        immediate_guard_released=False,
        immediate_positive_target=False,
    )


def snapshot(x=1, y=0, tx=0, ty=0, snapshot_id=1, estimator="raw_fallback"):
    dx, dy = x - tx, y - ty
    result = dict(
        snapshot_id=snapshot_id,
        available=True,
        estimator=estimator,
        pose_x=x,
        pose_y=y,
        pose_yaw=0,
        truth_x=tx,
        truth_y=ty,
        truth_yaw=0,
        error_x=dx,
        error_y=dy,
        error_norm=math.hypot(dx, dy),
        yaw_error_deg=0,
        used_sample_count=1,
        **context(),
    )
    for part in analysis.PARTS:
        a, b = (dx, dy) if part == "noise" else (0, 0)
        result.update(
            {part + "_x": a, part + "_y": b, part + "_norm": math.hypot(a, b)}
        )
    result.update(dict(zip(analysis.MODEL_FIELDS, [0.1, 0.1, math.sqrt(0.02)])))
    return result


def real_state():
    state = Replay()
    value = Sample(
        7500,
        (10, 10),
        (0.2, -0.1),
        (0.1, 0.05),
        (0.1, 0.05),
        7500,
        0.1,
        0.1,
        0.03,
        (0, 0, 0),
    )
    state.current = value
    state.motion = deque([value], maxlen=25)
    state.recovery = deque([value], maxlen=25)
    return state


class CorrectionCentreTransitionTests(unittest.TestCase):
    def test_all_opportunity_categories_include_misses_and_combined_precision(self):
        categories = []
        for scheduled, received, precision in [
            (False, False, False),
            (True, True, False),
            (True, False, False),
            (False, False, True),
            (True, True, True),
            (True, False, True),
        ]:
            categories.append(
                analysis.opportunity_kind(
                    dict(
                        correction=scheduled,
                        correction_received=received,
                        precision_activation=precision,
                    )
                )
            )
        self.assertEqual(categories, [None, *analysis.KINDS])

    def test_receipt_without_a_scheduled_correction_is_rejected(self):
        with self.assertRaises(AssertionError):
            analysis.opportunity_kind(
                dict(
                    correction=False,
                    correction_received=True,
                    precision_activation=False,
                )
            )

    def test_snapshot_uses_preintegration_truth_instead_of_the_integrated_log_position(
        self,
    ):
        state = real_state()
        before = dict(time_s="149.980000", x="10", y="10", yaw="0")
        row = dict(time_s="150.000000", x="40", y="40", yaw="2")
        result = analysis.input_snapshot(
            state, before, row, "actual", 0, state.pose("actual")
        )
        self.assertEqual(result["truth_time_s"], "149.980000")
        self.assertAlmostEqual(result["error_x"], 0.3)
        self.assertAlmostEqual(result["error_y"], -0.05)
        self.assertAlmostEqual(result["yaw_error_deg"], math.degrees(0.03))

    def test_logged_pose_check_accepts_wrapped_heading_and_rejects_wrong_coordinates(
        self,
    ):
        state = real_state()
        pose = state.pose("actual")
        row = dict(
            recovery_x=str(pose[0]),
            recovery_y=str(pose[1]),
            recovery_yaw=str(pose[2] + math.tau),
            preview_pose_x=str(pose[0]),
            preview_pose_y=str(pose[1]),
            preview_pose_yaw=str(pose[2]),
            motion_filter_pose_x=str(pose[0]),
            motion_filter_pose_y=str(pose[1]),
        )
        _, maximum = analysis.checked_poses(state, row)
        self.assertLess(maximum, 1e-10)
        row["recovery_x"] = str(pose[0] + 0.1)
        with self.assertRaises(AssertionError):
            analysis.checked_poses(state, row)

    def test_action_window_is_open_at_event_and_closed_at_the_last_interval(self):
        window = analysis.ActionWindow(50)
        window.observe(50, context(resumed=True), 99)
        for offset in range(1, 76):
            window.observe(
                50 + offset,
                context(ready=True, resumed=offset == 50, positive=True),
                0.01,
                scheduled=offset == 12,
                precision=offset == 23,
            )
        window.observe(126, context(resumed=True), 99)
        record = window.record()
        self.assertTrue(record["post_window_complete"])
        self.assertEqual(record["post_intervals"], 75)
        self.assertEqual(record["post_resumes"], 1)
        self.assertEqual(record["first_resume_delay_s"], 1)
        self.assertEqual(record["post_positive_target_intervals"], 75)
        self.assertEqual(record["joint_max_ready_s"], 1.5)
        self.assertEqual(record["intervening_scheduled"], 1)
        self.assertEqual(record["intervening_precision"], 1)
        self.assertAlmostEqual(record["post_physical_travel"], 0.75)

    def test_ready_runs_reset_on_failure_and_route_changes(self):
        window = analysis.ActionWindow(0)
        sequence = [
            ("joint", True),
            ("joint", True),
            ("joint", False),
            ("joint", True),
            ("independent", True),
            ("independent", True),
            ("inactive", False),
            ("independent", True),
        ]
        for step, (route, ready) in enumerate(sequence, 1):
            window.observe(step, context(route, ready), 0)
        record = window.record()
        self.assertEqual(record["joint_ready_s"], 0.06)
        self.assertEqual(record["joint_max_ready_s"], 0.04)
        self.assertEqual(record["independent_ready_s"], 0.06)
        self.assertEqual(record["independent_max_ready_s"], 0.04)

    def test_partial_window_keeps_its_observed_length_without_imputing_completion(self):
        window = analysis.ActionWindow(8990)
        for step in range(8991, 9001):
            window.observe(step, context(), 0)
        record = window.record()
        self.assertTrue(record["post_window_censored"])
        self.assertFalse(record["post_window_complete"])
        self.assertEqual(record["post_intervals"], 10)
        self.assertEqual(record["first_resume_delay_s"], "")

    def test_skipping_a_post_interval_is_rejected(self):
        with self.assertRaises(AssertionError):
            analysis.ActionWindow(50).observe(52, context(), 0)

    def test_equal_estimate_and_true_motion_is_not_an_error_improvement(self):
        before = snapshot(1, 0, 0, 0)
        after = snapshot(1.2, 0.1, 0.2, 0.1, snapshot_id=2)
        pair = analysis.pair_record(event(), "after_0p5", "actual", before, after)
        self.assertTrue(pair["within_tolerance"])
        self.assertAlmostEqual(pair["pose_shift_norm"], math.hypot(0.2, 0.1))
        self.assertAlmostEqual(pair["truth_shift_norm"], pair["pose_shift_norm"])
        self.assertAlmostEqual(pair["error_shift_norm"], 0)

    def test_a_large_estimate_jump_can_reduce_error_after_true_motion_is_accounted_for(
        self,
    ):
        pair = analysis.pair_record(
            event(),
            "after_0p5",
            "actual",
            snapshot(11, 10, 10, 10),
            snapshot(10.3, 10, 10.1, 10, snapshot_id=2),
        )
        self.assertTrue(pair["better"])
        self.assertAlmostEqual(pair["error_norm_change"], -0.8)
        self.assertAlmostEqual(
            pair["delta_pose_x"] - pair["delta_truth_x"], pair["delta_error_x"]
        )

    def test_inconsistent_component_change_is_rejected(self):
        after = snapshot(0.2, snapshot_id=2)
        after["noise_x"] = 0
        with self.assertRaises(AssertionError):
            analysis.pair_record(event(), "after_0p5", "actual", snapshot(), after)

    def test_censored_pairs_have_blank_post_errors_and_matched_rms_denominators(self):
        first = analysis.pair_record(
            event(), "after_0p5", "actual", snapshot(1), snapshot(0.5, snapshot_id=2)
        )
        censored = analysis.pair_record(
            event(2),
            "after_0p5",
            "actual",
            snapshot(100),
            dict(snapshot(snapshot_id=4), available=False),
        )
        self.assertEqual(censored["post_error_norm"], "")
        self.assertTrue(censored["unavailable"])
        row = next(
            r
            for r in analysis.summarize_pairs([first, censored])
            if (
                r["cohort"],
                r["variant"],
                r["event_kind"],
                r["phase"],
                r["input_kind"],
                r["route_group"],
                r["transition_group"],
            )
            == ("all", "epoch", "all", "after_0p5", "actual", "all", "all")
        )
        self.assertEqual(row["events"], 2)
        self.assertEqual(row["paired_available"], 1)
        self.assertEqual(row["pre_error_rms"], 1)
        self.assertEqual(row["post_error_rms"], 0.5)

    def test_estimator_transitions_and_censored_only_groups_are_retained(self):
        pair = analysis.pair_record(
            event(),
            "after_0p5",
            "preview",
            snapshot(estimator="motion_filter"),
            dict(snapshot(snapshot_id=2), available=False),
        )
        self.assertEqual(pair["estimator_transition"], "motion_filter->unavailable")
        row = next(
            r
            for r in analysis.summarize_pairs([pair])
            if r["cohort"] == "all"
            and r["variant"] == "epoch"
            and r["event_kind"] == "received"
            and r["phase"] == "after_0p5"
            and r["input_kind"] == "preview"
            and r["route_group"] == "all"
            and r["transition_group"] == "motion_filter->unavailable"
        )
        self.assertEqual(row["unavailable"], 1)
        self.assertEqual(row["pre_error_rms"], "")
        self.assertEqual(row["post_error_rms"], "")

    def test_event_action_summary_separates_censored_observations_from_complete_windows(
        self,
    ):
        complete, partial, empty = (
            analysis.ActionWindow(0),
            analysis.ActionWindow(0),
            analysis.ActionWindow(0),
        )
        for step in range(1, 76):
            complete.observe(step, context(resumed=step == 50), 0)
        partial.observe(1, context(resumed=True), 0)
        rows = [
            dict(event(i), **window.record())
            for i, window in enumerate([complete, partial, empty], 1)
        ]
        chosen = next(
            r
            for r in analysis.summarize_events(rows)
            if (r["cohort"], r["variant"], r["event_kind"], r["route_group"])
            == ("all", "epoch", "all", "all")
        )
        self.assertEqual(chosen["complete_windows"], 1)
        self.assertEqual(chosen["censored_windows"], 2)
        self.assertEqual(chosen["complete_resume_events"], 1)
        self.assertEqual(chosen["censored_observed_resume_events"], 1)

    def test_empty_opportunity_categories_keep_blank_rms_instead_of_zero(self):
        rows = analysis.summarize_pairs([])
        self.assertTrue(rows)
        self.assertTrue(
            all(
                r["events"] == 0
                and r["pre_error_rms"] == ""
                and r["post_error_rms"] == ""
                for r in rows
            )
        )
        self.assertEqual({r["event_kind"] for r in rows}, {"all", *analysis.KINDS})

    def test_track_orchestration_keeps_every_scheduled_miss_and_final_censored_phase(
        self,
    ):
        # Isolate event scheduling from the separately tested estimator and
        # gate calculations. The real raw replay checks their logged values.
        class Clock:
            current = None

            def advance(self, before, row):
                self.current = SimpleNamespace(
                    step=round(float(row["time_s"]) / analysis.DT)
                )

            def check_counts(self, row):
                pass

        def fake_snapshot(state, before, row, kind, rho, pose):
            data = snapshot(0)
            for key in ["snapshot_id", "available", *context()]:
                data.pop(key)
            data.update(time_s=row["time_s"], truth_time_s=before["time_s"])
            return data

        track = analysis.TrackReplay(event(), 0, {})
        track.state = Clock()
        with patch.object(
            analysis,
            "checked_poses",
            return_value=({k: (0, 0, 0) for k in ["actual", "preview", "motion"]}, 0),
        ), patch.object(analysis, "gate_context", return_value=context()), patch.object(
            analysis, "input_snapshot", side_effect=fake_snapshot
        ):
            for step in range(9001):
                scheduled = step > 0 and step % 250 == 0
                row = dict(
                    time_s=f"{step * analysis.DT:.6f}",
                    x="0",
                    y="0",
                    v="0",
                    measured_speed="0",
                    noise_rho="0",
                    correction=scheduled,
                    correction_received=scheduled and step != 8000,
                    precision_activation=False,
                )
                track.process(row)
        stats = track.validate()
        self.assertEqual(stats["scheduled_opportunities"], 7)
        events, snapshots, pairs = [], [], []
        track.export(events, snapshots, pairs)
        self.assertEqual(sum(e["event_kind"] == "missed" for e in events), 1)
        self.assertEqual(sum(e["post_window_complete"] for e in events), 6)
        self.assertEqual(sum(e["post_window_censored"] for e in events), 1)
        self.assertEqual(len(snapshots), 70)
        self.assertEqual(sum(not s["available"] for s in snapshots), 6)
        pre = next(s for s in snapshots if s["event_id"] == 1 and s["phase"] == "pre")
        self.assertEqual(pre["time_s"], "149.980000")
        self.assertEqual(pre["truth_time_s"], "149.960000")
        after = next(
            s for s in snapshots if s["event_id"] == 1 and s["phase"] == "after_0p18"
        )
        self.assertEqual(after["time_s"], "150.180000")
        self.assertEqual(after["truth_time_s"], "150.160000")


if __name__ == "__main__":
    unittest.main()
