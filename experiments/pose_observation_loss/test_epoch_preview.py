"""Check live fusion parity, immutable heading and disabled/pre-use regression."""

from collections import deque
from copy import deepcopy
import csv
from pathlib import Path
import tempfile
import random
import unittest

from analyze_epoch_preview import (
    CollisionAudit,
    NEW_COLUMNS,
    compare_disabled,
    diagnostics,
    paired,
)
from verify_stopping_preview import segment_distance
from analyze_stationary_epoch_fusion import StationaryEpochFusion
from verify_motion_filter import check_epoch_pair, check_motion_filter_track


def fixture(enabled=True):
    initial = dict(
        time_s=0,
        x=25,
        y=25,
        yaw=0,
        v=0,
        motion_filter_sample_count=0,
        stationary_epoch_fusion="false",
        epoch_fusion_available="false",
        epoch_fusion_group_count=0,
    )
    track = [initial]
    fusion = StationaryEpochFusion()
    samples = deque(maxlen=25)
    last_received = 0
    bias = 0.2
    for step in range(1, 826):
        time = step * 0.02
        correction = step == 1 or step % 250 == 0
        received = correction and step != 750
        if received:
            last_received = time
            bias = {1: 0.2, 250: 0.4, 500: 0.6}[step]
        if correction:
            samples.clear()
        samples.append(25 + bias)
        base = sum(samples) / len(samples)
        fusion.observe(
            time, (25 + bias, 25, 0), 0, 0, received, False, time - last_received
        )
        position = fusion.position() if enabled else None
        row = dict(
            time_s=time,
            x=25,
            y=25,
            yaw=0,
            v=0,
            measured_speed=0,
            noise_x=bias,
            noise_y=0,
            noise_yaw=0,
            bias_x=0,
            bias_y=0,
            bias_yaw=0,
            correction=str(correction).lower(),
            correction_received=str(received).lower(),
            precision_activation="false",
            steering=0,
            observation_age=time - last_received,
            motion_filter_sample_count=len(samples),
            motion_filter_pose_x=base,
            motion_filter_pose_y=25,
            preview_pose_x=base if position is None else position[0],
            preview_pose_y=25 if position is None else position[1],
            preview_pose_yaw=0,
            stationary_epoch_fusion=str(enabled).lower(),
            epoch_fusion_available=str(position is not None).lower(),
            epoch_fusion_group_count=len(fusion.groups) if enabled else 0,
        )
        track.append(row)
    return track


class EpochPreviewTests(unittest.TestCase):
    def test_spatial_contact_query_catches_sweeps_and_matches_full_scans(self):
        audit = CollisionAudit([(0.5, 0.49)])
        self.assertTrue(audit.contact((0, 0), (1, 0)))
        self.assertFalse(CollisionAudit([(0.5, 0.501)]).contact((0, 0), (1, 0)))
        rng = random.Random(1739)
        obstacles = [(rng.uniform(-2, 3), rng.uniform(-2, 3)) for _ in range(80)]
        audit = CollisionAudit(obstacles)
        for _ in range(200):
            start = tuple(rng.uniform(-2, 3) for _ in range(2))
            end = tuple(rng.uniform(-2, 3) for _ in range(2))
            expected = min(segment_distance(p, start, end) for p in obstacles) <= 0.5
            self.assertEqual(audit.contact(start, end), expected)

    def test_parity_includes_fresh_use_and_stale_fallback_after_a_missed_correction(
        self,
    ):
        track = fixture()
        check_motion_filter_track(track, True, True)
        self.assertEqual(track[499]["epoch_fusion_available"], "false")
        self.assertEqual(track[500]["epoch_fusion_available"], "true")
        self.assertAlmostEqual(track[500]["preview_pose_x"], 25.4)
        self.assertAlmostEqual(track[500]["motion_filter_pose_x"], 25.6)
        self.assertEqual(track[750]["epoch_fusion_group_count"], 3)
        self.assertEqual(track[750]["epoch_fusion_available"], "false")
        self.assertAlmostEqual(track[750]["preview_pose_x"], 25.6)

    def test_rejects_heading_changes_even_when_fusion_is_available(self):
        track = fixture()
        track[501]["preview_pose_yaw"] = 0.001
        with self.assertRaises(AssertionError):
            check_motion_filter_track(track, True, True)

    def test_round_trip_speed_preserves_the_strict_motion_reset_boundary(self):
        track = fixture()[:502]
        track[500]["v"] = "0.020000000"
        track[501].update(
            measured_speed="2.00000000000000039e-2",
            epoch_fusion_available="false",
            epoch_fusion_group_count=0,
            preview_pose_x=track[501]["motion_filter_pose_x"],
        )
        check_motion_filter_track(track, True, True)
        track[501]["measured_speed"] = "0.02"
        with self.assertRaises(AssertionError):
            check_motion_filter_track(track, True, True)

    def test_rejects_wrong_fused_position_and_wrong_fallback(self):
        for index in [499, 500, 750]:
            with self.subTest(index=index):
                track = fixture()
                track[index]["preview_pose_x"] += 0.001
                with self.assertRaises(AssertionError):
                    check_motion_filter_track(track, True, True)

    def test_rejects_wrong_availability_and_history_size(self):
        for field, value in [
            ("epoch_fusion_available", "false"),
            ("epoch_fusion_group_count", 2),
            ("motion_filter_pose_x", 25),
        ]:
            with self.subTest(field=field):
                track = fixture()
                track[500][field] = value
                with self.assertRaises(AssertionError):
                    check_motion_filter_track(track, True, True)

    def test_disabled_fusion_uses_original_filter_and_cannot_claim_availability(self):
        track = fixture(False)
        check_motion_filter_track(track, True)
        track[500]["epoch_fusion_available"] = "true"
        with self.assertRaises(AssertionError):
            check_motion_filter_track(track, True)

    def test_paired_control_matches_until_first_available_position(self):
        baseline, candidate = fixture(False), fixture()
        check_epoch_pair(baseline, candidate)
        wrong = deepcopy(candidate)
        wrong[400]["steering"] = 0.01
        with self.assertRaises(AssertionError):
            check_epoch_pair(baseline, wrong)
        # A changed command is permitted at the first use; physical and guard
        # verifiers independently check whether that command is justified.
        candidate[500]["steering"] = 0.01
        check_epoch_pair(baseline, candidate)

    def test_disabled_regression_checks_all_old_fields_and_full_length(self):
        with tempfile.TemporaryDirectory() as directory:
            old, new = (Path(directory) / name for name in ["old.csv", "new.csv"])
            old.write_text("time_s,x\n0,25\n0.02,25\n")
            fields = ["time_s", "x", *sorted(NEW_COLUMNS)]
            with new.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows([dict(time_s=t, x=25) for t in [0, 0.02]])
            self.assertEqual(compare_disabled(new, old), 2)
            changed = new.read_text().replace("0.02,25", "0.02,26")
            new.write_text(changed)
            with self.assertRaises(AssertionError):
                compare_disabled(new, old)
            new.write_text("\n".join(changed.splitlines()[:2]) + "\n")
            with self.assertRaises(AssertionError):
                compare_disabled(new, old)

    def test_error_metrics_use_the_pre_command_pose(self):
        track = fixture()
        subset = deepcopy(track[499:501])
        subset[0]["x"] = 10
        subset[1]["x"] = 11
        subset[1]["preview_pose_x"] = 10.1
        subset[1]["motion_filter_pose_x"] = 10.2
        subset[1]["guard_active"] = "true"
        subset[1]["safety_paused"] = "true"
        for flag in ["guard_triggered", "guard_released", "resumed"]:
            subset[1][flag] = "false"
        metrics, prefix = diagnostics(subset)
        self.assertAlmostEqual(metrics["base_position_rms"], 0.2)
        self.assertAlmostEqual(metrics["fused_position_rms"], 0.1)
        self.assertEqual(metrics["position_better_steps"], 1)
        self.assertEqual(metrics["wait_time_s"], 0.02)  # Union, not sum.
        self.assertEqual(prefix, [0, 1])

    def test_matched_wait_uses_the_shorter_trajectory_horizon(self):
        reference = dict(
            cohort="known-white",
            mode="none",
            seed="268",
            scenario="open_50x50",
            planner="astar",
            time_s="0.04",
            status="held",
            held="true",
        )
        candidate = dict(reference, time_s="0.06", fusion_available_steps=1)
        first = [
            dict(time_s=t, target_speed=0, steering=0, acceleration=0)
            for t in [0, 0.02, 0.04]
        ]
        second = deepcopy(first) + [dict(first[-1], time_s=0.06)]
        second[2]["steering"] = 0.1
        result = paired(
            reference, candidate, [[0, 0, 1], [0, 1, 1, 2]], [first, second]
        )
        self.assertEqual(result["common_horizon_s"], 0.04)
        self.assertEqual(result["reference_matched_wait_s"], 0.02)
        self.assertEqual(result["candidate_matched_wait_s"], 0.02)
        self.assertEqual(result["matched_wait_difference_s"], 0)
        self.assertEqual(result["first_command_change_s"], 0.04)
        self.assertAlmostEqual(result["held_time_difference_s"], 0.02)


if __name__ == "__main__":
    unittest.main()
