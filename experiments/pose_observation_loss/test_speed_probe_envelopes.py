"""Analytic bounds, transformed geometry, coverage and population accounting."""

import math
import unittest

import analyze_speed_probe_envelopes as analysis
from analyze_speed_probe_starts import query


def source_record(pose_id, pose, kind, obstacles, steering=0, margin=1):
    r = query(pose, steering, margin, obstacles)
    r.update(
        pose_id=str(pose_id),
        state_id="10",
        source_interval_id="100",
        truth_pose_id="4",
        pose_kind=kind,
        cohort="known-white",
        variant="epoch",
        mode="none",
        seed="269",
        scenario="fixture",
        planner="A*",
        source_event_kinds="received",
        time_s="150.56",
    )
    return r


def paired(estimate=(10, 10, 0), truth=(10, 10, 0), obstacles=None):
    obstacles = obstacles if obstacles is not None else [(12, 10)]
    a = source_record(1, estimate, "raw", obstacles)
    b = source_record(4, truth, "truth", obstacles)
    return analysis.pose_record(a, b, {"1"})


class SpeedProbeEnvelopeTests(unittest.TestCase):
    def test_wrapped_heading_error_across_pi(self):
        self.assertAlmostEqual(
            analysis.wrapped_error(math.pi - 0.02, -math.pi + 0.03), 0.05
        )
        self.assertAlmostEqual(analysis.wrapped_error(10 * math.pi, 0), 0, places=12)
        self.assertEqual(analysis.wrapped_error(0, math.pi), math.pi)

    def test_heading_displacement_matches_endpoint_rotation(self):
        length = 0.04998
        for yaw in [0, 0.001, math.radians(4), math.pi]:
            rotated = length * math.cos(yaw), length * math.sin(yaw)
            self.assertAlmostEqual(
                analysis.heading_displacement(length, yaw),
                math.dist((length, 0), rotated),
                places=14,
            )

    def test_invalid_uncertainty_inputs_are_rejected(self):
        for length, yaw in [(-1, 0), (0.05, -0.1), (0.05, math.pi + 0.01)]:
            with self.assertRaises(AssertionError):
                analysis.heading_displacement(length, yaw)
        with self.assertRaises(AssertionError):
            analysis.certificate(1, -1, -0.1)
        with self.assertRaises(AssertionError):
            analysis.budgets([math.nan, 1, 1, 1], 0)

    def test_heading_uncertainty_does_not_weaken_start_block_certificate(self):
        clear, block = analysis.budgets([-0.1, 9, -0.1, 9], 0.05)
        self.assertAlmostEqual(clear, -0.15)
        self.assertEqual(block, 0.1)
        self.assertEqual(analysis.certificate(clear, block, 0.09), "blocked")
        self.assertEqual(analysis.certificate(clear, block, 0.11), "unresolved")

    def test_chord_only_block_uses_rotation_displacement(self):
        clear, block = analysis.budgets([0.01, 9, -0.03, 9], 0.02)
        self.assertAlmostEqual(clear, -0.05)
        self.assertAlmostEqual(block, 0.01)
        self.assertEqual(analysis.certificate(clear, block, 0.005), "blocked")
        self.assertEqual(analysis.certificate(clear, block, 0.02), "unresolved")

    def test_walls_participate_in_clear_and_block_budgets(self):
        clear, block = analysis.budgets([10, 0.01, 10, -0.03], 0.02)
        self.assertAlmostEqual(clear, -0.05)
        self.assertAlmostEqual(block, 0.01)
        self.assertEqual(analysis.certificate(clear, block, 0), "blocked")

    def test_rounding_guard_leaves_exact_and_near_boundaries_unresolved(self):
        for gap in [0, analysis.EPS / 2, -analysis.EPS / 2]:
            self.assertEqual(analysis.certificate(gap, -gap, 0), "unresolved")
        self.assertEqual(
            analysis.certificate(analysis.EPS, -analysis.EPS, 0), "unresolved"
        )
        self.assertEqual(
            analysis.certificate(-analysis.EPS, analysis.EPS, 0), "blocked"
        )

    def test_clearance_decreases_monotonically_as_disk_or_yaw_grows(self):
        row = paired()
        row["clear_budget_0deg"] = 0.16
        row["clear_budget_4deg"] = 0.156
        for yaw in analysis.YAWS:
            statuses = [
                analysis.decision(row, r, yaw)["status"] for r in analysis.RADII
            ]
            self.assertEqual(statuses, ["clear"] * 4 + ["unresolved"] * 2)

    def test_zero_uncertainty_matches_central_classification_away_from_boundary(self):
        for x in [9, 10, 11.5]:
            r = paired(estimate=(x, 10, 0))
            d = analysis.decision(r, 0, 0)
            self.assertEqual(d["status"], "clear" if r["central_clear"] else "blocked")

    def test_position_and_heading_coverage_are_separate_closed_conditions(self):
        row = paired()
        row["position_error"] = 0.1
        row["heading_error_rad"] = math.radians(4)
        d = analysis.decision(row, 0.1, 4)
        self.assertTrue(d["covered"])
        self.assertFalse(analysis.decision(row, 0.05, 4)["position_covered"])
        self.assertFalse(analysis.decision(row, 0.1, 0)["heading_covered"])

    def test_uncovered_truth_can_disagree_with_a_clear_certificate(self):
        row = paired(estimate=(9, 10, 0), truth=(11.5, 10, 0))
        d = analysis.decision(row, 0.1, 4)
        self.assertEqual(d["status"], "clear")
        self.assertTrue(d["clear_truth_blocks"])
        self.assertFalse(d["covered"])
        self.assertGreater(row["truth_shortfall"], 0)
        self.assertEqual(row["oracle_status"], "unresolved")

    def test_oracle_interval_contains_true_gap_with_position_and_heading_changes(self):
        row = paired(estimate=(10, 10, 0.8), truth=(10.12, 9.94, -0.3))
        self.assertLessEqual(row["oracle_lower_gap"], row["truth_gap"] + analysis.EPS)
        self.assertGreaterEqual(
            row["oracle_upper_gap"], row["truth_gap"] - analysis.EPS
        )
        self.assertAlmostEqual(row["position_error"], math.hypot(0.12, 0.06))
        self.assertAlmostEqual(row["heading_error_rad"], 1.1)

    def test_corrupted_covered_truth_certificate_raises(self):
        row = paired()
        row["truth_clear"] = False
        with self.assertRaises(AssertionError):
            analysis.decision(row, 0.1, 4)

    def test_certificates_hold_for_translated_rotated_curved_chords(self):
        tested = set()
        obstacles = [(12, 10), (9.4, 11.2), (49.9, 10)]
        for center in [
            (9, 10, 0.3),
            (10.97, 10, 0.1),
            (11.7, 10, -0.5),
            (48.98, 10, 0.1),
        ]:
            for steering in [-0.5, 0, 0.5]:
                base = query(center, steering, 1, obstacles)
                gaps = [base[k] for k in analysis.GAPS]
                length = math.dist(
                    center[:2], (base["first_end_x"], base["first_end_y"])
                )
                for radius in [0.0, 0.02, 0.1]:
                    yaw = math.radians(4)
                    cb, bb = analysis.budgets(
                        gaps, analysis.heading_displacement(length, yaw)
                    )
                    status = analysis.certificate(cb, bb, radius)
                    tested.add(status)
                    for angle, fraction, delta in product_samples():
                        r = radius * fraction
                        pose = (
                            center[0] + r * math.cos(angle),
                            center[1] + r * math.sin(angle),
                            center[2] + delta * yaw,
                        )
                        actual = query(pose, steering, 1, obstacles)
                        actual_gap = min(actual[k] for k in analysis.GAPS)
                        self.assertGreaterEqual(actual_gap + analysis.EPS, cb - radius)
                        self.assertLessEqual(actual_gap - analysis.EPS, radius - bb)
                        if status != "unresolved":
                            self.assertEqual(
                                status == "clear", actual["first_chord_clear"]
                            )
        self.assertEqual(tested, set(analysis.STATUSES))

    def test_summary_preserves_fixed_loss_population_inputs_and_empty_groups(self):
        obstacles = [(12, 10)]
        truth = source_record(4, (11.5, 10, 0), "truth", obstacles)
        poses = []
        for pid, kind in enumerate(analysis.ESTIMATES, 1):
            p = source_record(pid, (9, 10, 0), kind, obstacles)
            poses.append(analysis.pose_record(p, truth, {"1", "2"}))
        events = {
            "1": dict(event_kind="received"),
            "2": dict(event_kind="precision_only"),
        }
        blocks = [
            dict(
                event_id="1",
                raw_pose_id="1",
                recovery_pose_id="2",
                preview_pose_id="3",
                command_only_zero_loss="True",
            )
        ]
        rows = analysis.summarize(poses, blocks, events)
        selected = [
            r
            for r in rows
            if (
                r["cohort"],
                r["variant"],
                r["event_kind"],
                r["population"],
                r["radius_m"],
                r["yaw_bound_deg"],
            )
            == ("all", "epoch", "all", "raw_zero_loss", 0.1, 4)
        ]
        self.assertEqual(len(selected), 3)
        self.assertTrue(
            all(r["records"] == r["cases"] == r["opportunities"] == 1 for r in selected)
        )
        self.assertTrue(all(r["clear_truth_blocks"] == 1 for r in selected))
        for r in rows:
            if (
                r["cohort"] == "all"
                and r["variant"] == "epoch"
                and r["population"] == "all_states"
            ):
                self.assertEqual(
                    r["records"],
                    int(r["event_kind"] in ["all", "received", "precision_only"]),
                )
                self.assertEqual(
                    r["opportunities"], 2 if r["event_kind"] == "all" else r["records"]
                )
            if r["event_kind"] == "precision_only" and r["population"] != "all_states":
                self.assertEqual(r["records"], 0)
                self.assertEqual(r["mean_position_error"], "")

    def test_empty_conflicts_have_blank_means_not_zero_errors(self):
        metrics = analysis.population_metrics([paired()])
        self.assertEqual(metrics["central_clear_truth_blocks"], 0)
        self.assertEqual(metrics["conflict_mean_position_error"], "")
        self.assertEqual(metrics["mean_position_error"], 0)
        self.assertEqual(analysis.population_metrics([])["rms_position_error"], "")


def product_samples():
    for angle in [i * math.pi / 4 for i in range(8)]:
        for fraction in [0, 0.5, 1]:
            for delta in [-1, 0, 1]:
                yield angle, fraction, delta


if __name__ == "__main__":
    unittest.main()
