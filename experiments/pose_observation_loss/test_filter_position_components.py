"""Check common-bias persistence, signed error decomposition and reset timing."""

import math
import unittest

from analyze_filter_position_components import (
    Window,
    decompose,
    effective_samples,
    summarize,
)


def sample(noise=(0, 0), anchor=(0, 0), drift=(0, 0), truth=(0, 0), delta=(0, 0)):
    row = dict(sigma=0.1, rho=0.0)
    for i, axis in enumerate(["x", "y"]):
        row.update(
            {
                f"noise_{axis}": noise[i],
                f"anchor_{axis}": anchor[i],
                f"bias_{axis}": anchor[i] + drift[i],
                f"true_{axis}": truth[i],
                f"delta_{axis}": delta[i],
            }
        )
    return row


def observe(
    window,
    time,
    x,
    speed,
    bias,
    count,
    expected,
    correction=False,
    received=False,
    precision=False,
):
    before = dict(x=str(x), y="0", yaw="0", v=str(speed))
    row = dict(
        time_s=str(time),
        correction=str(correction).lower(),
        correction_received=str(received).lower(),
        precision_activation=str(precision).lower(),
        bias_x=str(bias),
        bias_y="0",
        bias_yaw="0",
        noise_x="0",
        noise_y="0",
        noise_yaw="0",
        steering="0",
        effective_position_sigma="0.1",
        noise_rho="0",
        preview_pose_x=str(expected),
        preview_pose_y="0",
        preview_pose_yaw="0",
        motion_filter_sample_count=str(count),
    )
    window.advance(before, row)


class FilterPositionComponentsTests(unittest.TestCase):
    def test_effective_count_matches_full_covariance_matrix(self):
        for n in [1, 7, 25]:
            for rho in [0, 0.3, math.exp(-0.02), 1]:
                covariance = sum(
                    1 if i == j else rho ** abs(i - j)
                    for i in range(n)
                    for j in range(n)
                )
                self.assertAlmostEqual(effective_samples(n, rho), n * n / covariance)
        self.assertEqual(effective_samples(25, 0), 25)
        self.assertEqual(effective_samples(25, 1), 1)

    def test_averaging_noise_does_not_reduce_a_common_anchor(self):
        samples = [
            sample(noise=(e, 0), anchor=(0.2, 0)) for e in [-0.1, 0.1] * 12 + [0]
        ]
        result = decompose(samples, dict(x=0, y=0))
        self.assertAlmostEqual(result["noise_norm"], 0)
        self.assertAlmostEqual(result["anchor_norm"], 0.2)
        self.assertAlmostEqual(result["error_norm"], 0.2)
        self.assertEqual(result["largest_projection"], "anchor")

    def test_signed_cancellation_and_zero_error_keep_components(self):
        row = decompose([sample(noise=(-0.1, 0), anchor=(0.3, 0))], dict(x=0, y=0))
        self.assertAlmostEqual(row["noise_projection"], -0.1)
        self.assertAlmostEqual(row["anchor_projection"], 0.3)
        self.assertAlmostEqual(row["error_norm"], 0.2)
        zero = decompose([sample(noise=(-0.3, 0), anchor=(0.3, 0))], dict(x=0, y=0))
        self.assertEqual(zero["error_norm"], 0)
        self.assertAlmostEqual(zero["anchor_norm"], 0.3)
        self.assertEqual(zero["largest_projection"], "none")
        self.assertEqual(zero["noise_projection"], 0)

    def test_four_vectors_recover_both_axes_with_transport(self):
        row = decompose(
            [
                sample(
                    noise=(0.1, 0.2),
                    anchor=(0.2, -0.1),
                    drift=(0.03, 0.04),
                    truth=(1, 2),
                    delta=(0.2, -0.4),
                )
            ],
            dict(x=1.1, y=1.7),
        )
        self.assertAlmostEqual(row["transport_x"], 0.1)
        self.assertAlmostEqual(row["transport_y"], -0.1)
        self.assertAlmostEqual(row["error_x"], 0.43)
        self.assertAlmostEqual(row["error_y"], 0.04)
        self.assertAlmostEqual(row["error_norm"], math.hypot(0.43, 0.04))

    def test_motion_uses_prior_speed_then_stops_transport_when_stationary(self):
        window = Window()
        observe(window, 0.02, 0, 1, 0.2, 1, 0.2)
        observe(window, 0.04, 0.02, 0, 0.2, 2, 0.22)
        observe(window, 0.06, 0.02, 0, 0.2, 3, 0.22)
        row = decompose(window.samples, dict(x=0.02, y=0))
        self.assertAlmostEqual(row["transport_norm"], 0)
        self.assertAlmostEqual(window.samples[0]["delta_x"], 0.02)

    def test_missed_correction_resets_window_but_preserves_received_anchor(self):
        window = Window()
        observe(window, 0.02, 0, 0, 0.2, 1, 0.2, correction=True, received=True)
        observe(window, 0.04, 0, 0, 0.3, 1, 0.3, correction=True)
        row = decompose(window.samples, dict(x=0, y=0))
        self.assertAlmostEqual(row["anchor_x"], 0.2)
        self.assertAlmostEqual(row["drift_x"], 0.1)
        self.assertEqual(window.anchor_time, 0.02)
        observe(window, 0.06, 0, 0, -0.1, 1, -0.1, correction=True, received=True)
        self.assertEqual(window.anchor["x"], -0.1)
        observe(window, 0.08, 0, 0, 0.05, 1, 0.05, precision=True)
        self.assertEqual(window.anchor_time, 0.08)
        self.assertEqual(window.anchor["x"], 0.05)

    def test_summary_preserves_signed_means_and_nonadditive_rms(self):
        rows = []
        for cohort in ["white-validation", "correlated-validation"]:
            for reason in ["final", "fresh_stationary"]:
                for noise in [-0.1, 0.1]:
                    rows.append(
                        dict(
                            cohort=cohort,
                            reason=reason,
                            **decompose(
                                [sample(noise=(noise, 0), anchor=(0.2, 0))],
                                dict(x=0, y=0),
                            ),
                        )
                    )
        for row in summarize(rows):
            self.assertEqual(row["checkpoints"], 2)
            self.assertAlmostEqual(row["mean_noise_projection"], 0)
            self.assertAlmostEqual(row["mean_anchor_projection"], 0.2)
            self.assertAlmostEqual(row["rms_error"], math.sqrt(0.05))
            self.assertNotAlmostEqual(
                row["rms_error"], row["rms_noise"] + row["rms_anchor"]
            )


if __name__ == "__main__":
    unittest.main()
