"""Check live fusion parity, immutable heading and disabled/pre-use regression."""

from collections import deque
from copy import deepcopy
import unittest

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


if __name__ == "__main__":
    unittest.main()
