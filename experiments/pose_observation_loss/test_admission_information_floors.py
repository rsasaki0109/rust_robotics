"""Shared-anchor floors, ideal batch independence and robust-radius predicates."""

import math
import unittest

from analyze_admission_information_floors import (
    BATCHES,
    BOTTLENECKS,
    STRATEGIES,
    bottleneck,
    information_radius,
    radius_caps,
    radius_status,
    strategy_records,
    summarize,
)
from analyze_commanded_recovery_arc import META, arc_components, classify
from analyze_epoch_position_components import Sample, model_variances
from analyze_recovery_arcs import arc_point


def geometry(pose, steering, obstacles):
    obstacle, wall = arc_components(pose, steering, obstacles)
    return dict(obstacle=obstacle, wall=wall)


def caps(obstacle=2, wall=3, interpolation=0):
    return radius_caps([dict(obstacle=obstacle, wall=wall)], 1, interpolation)


def base_record(truth=True, error=0.5):
    result = dict.fromkeys(META, "0")
    result.update(
        base_row=1,
        cohort="known-white",
        variant="epoch",
        mode="random",
        seed="269",
        scenario="fixture",
        planner="A*",
        time_s="150.5",
        reasons="age_0p5",
        input_kind="actual",
        path="commanded",
        prior_optimistic=False,
        prior_conservative=False,
        optimistic_arc_gain=False,
        baseline_certified_clear=False,
        baseline_recorded_pose_covered=False,
        recorded_truth_clear=truth,
        recorded_position_error=error,
        recorded_yaw_error_deg=0,
        bottleneck="shared_anchor_floor",
    )
    return result


class AdmissionInformationFloorTests(unittest.TestCase):
    def test_repeated_noise_retains_anchor_floor_while_independent_anchors_tend_to_zero(
        self,
    ):
        same = [information_radius(0.0004, 0.01, "same_anchors", k) for k in BATCHES]
        fresh = [
            information_radius(0.0004, 0.01, "independent_anchors", k) for k in BATCHES
        ]
        self.assertEqual(same[0], fresh[0])
        self.assertGreater(same[-1], 0)
        self.assertAlmostEqual(same[-1], 2 * math.sqrt(2) * 0.1)
        self.assertEqual(fresh[-1], 0)
        self.assertTrue(all(a >= b for a, b in zip(same, same[1:])))
        self.assertTrue(all(a >= b for a, b in zip(fresh, fresh[1:])))
        self.assertTrue(all(a >= b for a, b in zip(same, fresh)))

    def test_independent_batch_model_is_optimistic_for_correlated_epoch_noise(self):
        def sample(step, anchor_id):
            return Sample(
                step, (0, 0), (0, 0), (0, 0), (0, 0), anchor_id, 0.1, 0.1, 0, (0, 0, 0)
            )

        first = [(sample(i, 1), 1 / 25) for i in range(25)]
        three = [
            (sample(epoch * 250 + i, epoch + 1), 1 / 75)
            for epoch in range(3)
            for i in range(25)
        ]
        for rho in [0, math.exp(-0.02)]:
            original, combined = model_variances(first, rho), model_variances(
                three, rho
            )
            ideal = information_radius(
                original["model_noise_sigma_per_axis"] ** 2,
                original["model_anchor_sigma_per_axis"] ** 2,
                "independent_anchors",
                3,
            )
            actual = 2 * math.sqrt(2) * combined["model_noise_anchor_sigma_per_axis"]
            if rho:
                self.assertGreater(actual, ideal + 1e-5)
            else:
                self.assertAlmostEqual(actual, ideal, places=14)

    def test_noiseless_and_zero_anchor_cases_keep_distinct_limits(self):
        self.assertEqual(
            information_radius(0, 0.01, "same_anchors", 25),
            information_radius(0, 0.01, "same_anchors", 1),
        )
        for k in BATCHES:
            self.assertEqual(
                information_radius(0.01, 0, "same_anchors", k),
                information_radius(0.01, 0, "independent_anchors", k),
            )
            for strategy in STRATEGIES:
                self.assertEqual(information_radius(0, 0, strategy, k), 0)

    def test_radius_equality_preserves_inclusive_wall_and_strict_obstacle(self):
        self.assertEqual(radius_status(caps(obstacle=4, wall=2), 1), "certified_clear")
        self.assertEqual(radius_status(caps(obstacle=2, wall=4), 1), "some_pose_blocks")

    def test_interpolation_gap_is_kept_separate_from_known_blocking_poses(self):
        bounds = caps(obstacle=4, wall=2, interpolation=0.25)
        self.assertEqual(radius_status(bounds, 0.75), "certified_clear")
        self.assertEqual(radius_status(bounds, 1), "angle_gap")
        self.assertEqual(radius_status(bounds, 1.01), "some_pose_blocks")

    def test_negative_radius_caps_are_not_clipped_into_zero_radius_permission(self):
        bounds = caps(obstacle=0.9, wall=4)
        self.assertLess(bounds["obstacle_necessary_cap"], 0)
        self.assertEqual(radius_status(bounds, 0), "some_pose_blocks")

    def test_sufficient_caps_cover_dense_heading_and_translation_interiors(self):
        origin, obstacles, margin = (25, 25, 0.3), [(25.7, 26.3), (24.5, 25.9)], 0.7
        permitted = 0
        for steering in [-math.radians(35), 0, math.radians(35)]:
            samples = [
                geometry(
                    (*origin[:2], origin[2] + math.radians(a)), steering, obstacles
                )
                for a in range(-4, 5)
            ]
            bounds = radius_caps(samples, margin)
            for radius in [0, 0.05, 0.2, 0.4]:
                if radius_status(bounds, radius) != "certified_clear":
                    continue
                permitted += 1
                for i in range(33):
                    yaw = origin[2] + math.radians(-4 + i / 4)
                    for fraction in [0, 0.5, 1]:
                        for direction in range(16):
                            angle = direction * math.tau / 16
                            pose = (
                                origin[0] + fraction * radius * math.cos(angle),
                                origin[1] + fraction * radius * math.sin(angle),
                                yaw,
                            )
                            measured = geometry(pose, steering, obstacles)
                            self.assertTrue(
                                classify(
                                    measured["obstacle"], measured["wall"], margin
                                )["clear"]
                            )
        self.assertGreater(permitted, 3)

    def test_translation_toward_a_point_obstacle_constructs_a_blocking_pose(self):
        origin, point, margin = (25, 25, 0.3), (25.7, 26.3), 0.7
        for steering in [-math.radians(35), 0, math.radians(35)]:
            poses = [(*origin[:2], origin[2] + math.radians(a)) for a in range(-4, 5)]
            samples = [geometry(p, steering, [point]) for p in poses]
            index = min(range(9), key=lambda i: samples[i]["obstacle"])
            radius = max(0, samples[index]["obstacle"] - margin) + 0.01
            self.assertEqual(
                radius_status(radius_caps(samples, margin), radius), "some_pose_blocks"
            )
            # Independently approximate the nearest path point; extra .01 m
            # exceeds the .003 m point spacing and avoids predicate ambiguity.
            pose = poses[index]
            nearest = min(
                (arc_point(pose, steering, 0.6 * i / 200) for i in range(201)),
                key=lambda p: math.dist(p, point),
            )
            distance = math.dist(nearest, point)
            amount = min(radius, distance)
            moved = (
                pose[0] + amount * (point[0] - nearest[0]) / distance,
                pose[1] + amount * (point[1] - nearest[1]) / distance,
                pose[2],
            )
            self.assertLessEqual(geometry(moved, steering, [point])["obstacle"], margin)

    def test_translation_toward_wall_attains_the_necessary_radius_threshold(self):
        origin, obstacles, margin = (1.5, 25, 0), [(40, 40)], 1
        samples = [
            geometry((*origin[:2], math.radians(a)), 0, obstacles) for a in range(-4, 5)
        ]
        bounds = radius_caps(samples, margin)
        self.assertEqual(radius_status(bounds, 0.5), "angle_gap")
        self.assertEqual(radius_status(bounds, 0.6), "some_pose_blocks")
        at_boundary = geometry((1, 25, 0), 0, obstacles)
        beyond_boundary = geometry((0.9, 25, 0), 0, obstacles)
        self.assertTrue(classify(**at_boundary, margin=margin)["clear"])
        self.assertFalse(classify(**beyond_boundary, margin=margin)["clear"])

    def test_bottleneck_partition_covers_all_predeclared_categories(self):
        fixtures = [
            (caps(), 0.2, 0.1, True),
            (caps(obstacle=0.9), 0.2, 0.1, False),
            (caps(obstacle=0.9), 0.2, 0.1, True),
            (caps(obstacle=1.01, interpolation=0.02), 0.2, 0.1, True),
            (caps(obstacle=1.25), 0.4, 0.3, True),
            (caps(obstacle=1.25, interpolation=0.05), 0.4, 0.225, True),
            (caps(obstacle=1.25), 0.4, 0.1, True),
        ]
        self.assertEqual([bottleneck(*values) for values in fixtures], BOTTLENECKS)

    def test_successful_wall_limit_does_not_imply_a_finite_batch_count_succeeds(self):
        bounds = caps(obstacle=4, wall=1)
        base = base_record(error=0)
        records = strategy_records(base, bounds, 0.01, 0)
        for row in records:
            self.assertEqual(
                row["status"],
                "certified_clear" if row["batches"] == "limit" else "some_pose_blocks",
            )
        self.assertEqual(bottleneck(bounds, 0.3, 0, True), "noise_limit_certifies")

    def test_truth_changes_only_fixed_centre_evaluation_and_can_expose_uncovered_false_permits(
        self,
    ):
        bounds = caps(obstacle=1.25)
        good = strategy_records(base_record(), bounds, 0.01, 0.01)
        bad = strategy_records(base_record(truth=False), bounds, 0.01, 0.01)
        for a, b in zip(good, bad):
            for key in ["radius", "status", "strategy", "batches"]:
                self.assertEqual(a[key], b[key])
        self.assertTrue(any(r["fixed_false_permit"] for r in bad))
        self.assertTrue(
            all(not r["recorded_pose_covered"] for r in bad if r["fixed_false_permit"])
        )

    def test_a_blocking_pose_does_not_make_the_centre_blocked(self):
        origin, obstacles = (25, 25, 0), [(25.3, 26.05)]
        samples = [
            geometry((*origin[:2], math.radians(a)), 0, obstacles) for a in range(-4, 5)
        ]
        self.assertEqual(
            radius_status(radius_caps(samples, 1), 0.2), "some_pose_blocks"
        )
        centre = geometry(origin, 0, obstacles)
        self.assertTrue(classify(**centre, margin=1)["clear"])

    def test_summaries_partition_statuses_and_keep_empty_subgroups(self):
        bounds = caps(obstacle=1.25)
        bases = [
            base_record(),
            dict(base_record(truth=False), base_row=2, time_s="155.5"),
        ]
        curves = [
            r for base in bases for r in strategy_records(base, bounds, 0.01, 0.01)
        ]
        summary, floors = summarize(bases, curves)
        chosen = [
            r
            for r in summary
            if r["cohort"] == "all" and r["subset"] == "age_0p5" and r["group"] == "all"
        ]
        self.assertEqual(len(chosen), 10)
        self.assertTrue(all(r["checkpoints"] == 2 and r["cases"] == 1 for r in chosen))
        empty = [r for r in summary if r["group"] == "prior_conservative"]
        self.assertTrue(
            all(r["checkpoints"] == 0 and r["mean_radius"] == "" for r in empty)
        )
        chosen_floors = next(
            r
            for r in floors
            if r["cohort"] == "all" and r["subset"] == "age_0p5" and r["group"] == "all"
        )
        self.assertEqual(chosen_floors["shared_anchor_floor"], 2)
        self.assertEqual(sum(chosen_floors[k] for k in BOTTLENECKS), 2)


if __name__ == "__main__":
    unittest.main()
