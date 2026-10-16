"""Weighted observation/anchor covariance and actual recovery/fusion windows."""

import math
from collections import deque
import random
import unittest

from analyze_epoch_position_components import (
    Replay,
    Sample,
    decompose,
    model_variances,
    summarize,
    transported,
)


def sample(
    step,
    anchor=(0, 0),
    anchor_id=1,
    sigma=0.1,
    anchor_sigma=0.1,
    noise=(0, 0),
    drift=(0, 0),
    truth=(0, 0),
    yaw=0,
    odom=(0, 0, 0),
):
    return Sample(
        step,
        truth,
        noise,
        tuple(a + d for a, d in zip(anchor, drift)),
        anchor,
        anchor_id,
        anchor_sigma,
        sigma,
        yaw,
        odom,
    )


def observe(
    replay,
    step,
    speed=0,
    steering=0,
    correction=False,
    received=False,
    precision=False,
    age=0,
    bias=(0, 0),
    truth=(0, 0, 0),
    sigma=0.1,
):
    before = dict(
        x=str(truth[0]),
        y=str(truth[1]),
        yaw=str(truth[2]),
        safety_paused="true",
        v="999",
    )
    row = dict(
        time_s=str(step * 0.02),
        measured_speed=str(speed),
        steering=str(steering),
        correction=str(correction).lower(),
        correction_received=str(received).lower(),
        precision_activation=str(precision).lower(),
        observation_age=str(age),
        effective_position_sigma=str(sigma),
        bias_x=str(bias[0]),
        bias_y=str(bias[1]),
        bias_yaw="0",
        noise_x="0",
        noise_y="0",
        noise_yaw="0",
    )
    replay.advance(before, row)


class EpochPositionComponentsTests(unittest.TestCase):
    def test_preview_heading_uses_the_motion_filter_with_fused_position(self):
        replay = Replay()
        for base, yaw in [(250, 1.0), (500, 0.8)]:
            for i in range(25):
                observe(
                    replay,
                    base + i,
                    correction=i == 0,
                    received=i == 0,
                    age=i * 0.02,
                    truth=(0, 0, yaw),
                )
        observe(replay, 750, correction=True, received=True, truth=(0, 0, 0.2))
        self.assertTrue(replay.available())
        self.assertAlmostEqual(replay.pose("preview")[2], 0.2)
        self.assertAlmostEqual(replay.pose("motion")[2], 0.2)

    def test_shared_odometry_matches_direct_transport_under_changing_controls(self):
        replay, direct = Replay(), deque(maxlen=25)
        rng, previous = random.Random(102916), None
        for step in range(1, 131):
            correction = step % 37 == 0
            if correction:
                direct.clear()
            if previous is not None:
                speed, steering = previous
                distance, turn = speed * 0.02, speed * 0.02 / 0.4 * math.tan(steering)
                for item in direct:
                    item[0] += distance * math.cos(item[2])
                    item[1] += distance * math.sin(item[2])
                    item[2] += turn
            speed, steering = rng.uniform(-0.4, 0.4), rng.uniform(-0.6, 0.6)
            truth = (
                rng.uniform(24, 26),
                rng.uniform(24, 26),
                rng.uniform(-math.pi, math.pi),
            )
            observe(
                replay,
                step,
                speed=speed,
                steering=steering,
                truth=truth,
                correction=correction,
            )
            direct.append(list(truth))
            expected = (
                sum(p[0] for p in direct) / len(direct),
                sum(p[1] for p in direct) / len(direct),
                math.atan2(
                    sum(math.sin(p[2]) for p in direct),
                    sum(math.cos(p[2]) for p in direct),
                ),
            )
            for a, b in zip(replay.pose("motion"), expected):
                self.assertAlmostEqual(a, b, places=11)
            previous = speed, steering

    def test_actual_spacing_weights_and_changing_sigma_match_full_covariance(self):
        values = [
            sample(t, sigma=s, anchor_id=i)
            for i, (t, s) in enumerate([(1, 0.1), (2, 0.1), (250, 0.03), (251, 0.03)])
        ]
        weights = [1 / 6, 1 / 6, 1 / 3, 1 / 3]
        for rho in [0, math.exp(-0.02), 1]:
            result = model_variances(list(zip(values, weights)), rho)
            noise = sum(
                wa * wb * a.sigma * b.sigma * rho ** abs(a.step - b.step)
                for a, wa in zip(values, weights)
                for b, wb in zip(values, weights)
            )
            unit = sum(
                wa * wb * rho ** abs(a.step - b.step)
                for a, wa in zip(values, weights)
                for b, wb in zip(values, weights)
            )
            self.assertAlmostEqual(result["model_noise_sigma_per_axis"] ** 2, noise)
            self.assertAlmostEqual(result["noise_effective_samples"], 1 / unit)

    def test_repeated_anchor_is_not_reduced_as_independent_samples(self):
        weighted = [(sample(i, anchor=(0.2, 0)), 1 / 25) for i in range(25)]
        result = model_variances(weighted, 0)
        self.assertAlmostEqual(result["model_noise_sigma_per_axis"], 0.02)
        self.assertAlmostEqual(result["model_anchor_sigma_per_axis"], 0.1)
        self.assertAlmostEqual(
            result["model_noise_anchor_sigma_per_axis"], math.sqrt(0.0104)
        )

    def test_distinct_equal_epoch_anchors_reduce_by_epoch_count(self):
        values = [
            (sample(i * 250 + j, anchor_id=i), 1 / 75)
            for i in range(3)
            for j in range(25)
        ]
        result = model_variances(values, 0)
        self.assertAlmostEqual(
            result["model_anchor_sigma_per_axis"], 0.1 / math.sqrt(3)
        )
        self.assertEqual(result["stochastic_anchor_count"], 3)

    def test_three_epochs_keep_equal_weights_with_one_latest_sample(self):
        groups = [
            dict(
                samples=[
                    sample(g * 250 + i, anchor=(a, 0), anchor_id=g) for i in range(n)
                ],
                epoch=g * 250,
                weight=1 / 3,
                transport=True,
            )
            for g, (n, a) in enumerate([(25, 0.3), (25, 0), (1, 0)], 1)
        ]
        result, details = decompose(groups, (0, 0), (0, 0, 0), 0)
        self.assertAlmostEqual(result["reconstructed_x"], 0.1)
        self.assertEqual(result["used_sample_count"], 51)
        self.assertAlmostEqual(result["weight_effective_samples"], 25 / 3)
        self.assertAlmostEqual(
            sum(g["group_weight"] * g["anchor_x"] for g in details), 0.1
        )

    def test_correlated_samples_across_epochs_use_the_actual_gap(self):
        rho = math.exp(-0.02)
        near = model_variances([(sample(1), 0.5), (sample(2, anchor_id=2), 0.5)], rho)
        far = model_variances([(sample(1), 0.5), (sample(251, anchor_id=2), 0.5)], rho)
        self.assertLess(near["noise_effective_samples"], far["noise_effective_samples"])
        self.assertAlmostEqual(far["noise_effective_samples"], 2 / (1 + math.exp(-5)))

    def test_four_vectors_and_signed_cancellation_are_retained(self):
        item = sample(
            1, anchor=(0.2, -0.1), noise=(0.1, 0.2), drift=(0.03, 0.04), truth=(1, 2)
        )
        result, _ = decompose(
            [dict(samples=[item], epoch=1, weight=1, transport=True)],
            (1.1, 1.7),
            (0.2, -0.4, 0),
            0,
        )
        self.assertAlmostEqual(result["error_x"], 0.43)
        self.assertAlmostEqual(result["error_y"], 0.04)
        zero = sample(1, noise=(-0.3, 0), anchor=(0.3, 0))
        result, _ = decompose(
            [dict(samples=[zero], epoch=1, weight=1, transport=False)],
            (0, 0),
            (0, 0, 0),
            0,
        )
        self.assertEqual(result["largest_projection"], "none")
        self.assertAlmostEqual(result["noise_norm"], 0.3)
        self.assertAlmostEqual(result["anchor_norm"], 0.3)

    def test_empty_recovery_uses_one_raw_sample_and_initial_anchor_is_deterministic(
        self,
    ):
        replay = Replay()
        observe(replay, 1, speed=1, age=0.02, bias=(0.0004, 0))
        estimator, groups = replay.view("actual")
        self.assertEqual(estimator, "raw_fallback")
        self.assertEqual(len(replay.recovery), 0)
        result, _ = decompose(groups, (0, 0), replay.odom, 0)
        self.assertEqual(result["used_sample_count"], 1)
        self.assertEqual(result["model_anchor_sigma_per_axis"], 0)
        self.assertAlmostEqual(result["drift_x"], 0.0004)

    def test_missed_correction_resets_windows_without_replacing_the_anchor(self):
        replay = Replay()
        observe(replay, 250, correction=True, received=True, bias=(0.2, 0))
        observe(replay, 251, age=0.02, bias=(0.2004, 0))
        observe(replay, 500, correction=True, received=False, age=5, bias=(0.3, 0))
        self.assertEqual(len(replay.motion), 1)
        self.assertEqual(len(replay.recovery), 0)
        self.assertEqual(replay.current.anchor_id, 250)
        self.assertEqual(replay.current.anchor, (0.2, 0))
        self.assertFalse(replay.available())

    def test_partial_latest_epoch_is_available_then_precision_clears_old_groups(self):
        replay = Replay()
        for base in [250, 500]:
            for k in range(25):
                observe(
                    replay, base + k, correction=k == 0, received=k == 0, age=k * 0.02
                )
        observe(replay, 750, correction=True, received=True)
        self.assertTrue(replay.available())
        self.assertEqual([len(g["samples"]) for g in replay.groups], [25, 25, 1])
        observe(replay, 751, precision=True, sigma=0.03)
        self.assertFalse(replay.available())
        self.assertEqual(len(replay.groups), 1)
        self.assertEqual(replay.current.anchor_id, 751)
        self.assertEqual(replay.current.anchor_sigma, 0.03)

    def test_transport_uses_previous_measured_controls_in_the_sample_heading(self):
        replay = Replay()
        observe(replay, 1, speed=1, steering=math.atan(0.4))
        first = replay.current
        observe(replay, 2, speed=0, steering=0, truth=(0.02, 0, 0))
        delta, yaw = transported(first, replay.odom, True)
        self.assertAlmostEqual(delta[0], 0.02)
        self.assertAlmostEqual(delta[1], 0)
        self.assertAlmostEqual(yaw, 0.02)
        moved = sample(1, yaw=math.pi / 2)
        delta, _ = transported(moved, replay.odom, True)
        self.assertAlmostEqual(delta[0], 0)
        self.assertAlmostEqual(delta[1], 0.02)
        before = replay.odom
        observe(replay, 3, speed=0)
        self.assertEqual(replay.odom, before)

    def test_motion_clears_all_fusion_groups(self):
        replay = Replay()
        observe(replay, 250, correction=True, received=True)
        self.assertEqual(len(replay.groups), 1)
        observe(replay, 251, speed=0.021, age=0.02)
        self.assertEqual(len(replay.groups), 0)
        self.assertEqual(len(replay.recovery), 0)

    def test_empty_summary_groups_remain_distinct_from_zero_error(self):
        item = sample(1)
        result, _ = decompose(
            [dict(samples=[item], epoch=1, weight=1, transport=False)],
            (0, 0),
            (0, 0, 0),
            0,
        )
        row = dict(
            cohort="known-white",
            variant="epoch",
            mode="none",
            seed="269",
            scenario="fixture",
            planner="A*",
            time_s="150.5",
            reasons="age_0p5",
            input_kind="actual",
            estimator="recovery_mean",
            optimistic=False,
            conservative=False,
            optimistic_arc_gain=False,
            **result
        )
        summaries = summarize([row])
        chosen = {
            r["group"]: r
            for r in summaries
            if r["cohort"] == "all"
            and r["subset"] == "age_0p5"
            and r["estimator"] == "all"
        }
        self.assertEqual(chosen["all"]["rms_error"], 0)
        self.assertEqual(chosen["optimistic"]["checkpoints"], 0)
        self.assertEqual(chosen["optimistic"]["rms_error"], "")


if __name__ == "__main__":
    unittest.main()
