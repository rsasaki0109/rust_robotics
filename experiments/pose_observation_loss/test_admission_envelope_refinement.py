"""Check continuous covers, finite examples, budgets and truth isolation."""

import math
import random
import unittest

from analyze_admission_envelope_refinement import (
    Cell,
    cell_bounds,
    evaluated_record,
    refine,
    summarize,
    verify_cover,
    witness_state,
)
from analyze_commanded_recovery_arc import META, arc_components
from analyze_joint_admission_geometry import EPS

ORIGIN = (25, 25, 0)


def measurement(radius, obstacles, origin=ORIGIN, steering=0):
    def measure(parameters):
        nx, ny, angle = parameters
        pose = (
            origin[0] + radius * nx,
            origin[1] + radius * ny,
            origin[2] + math.radians(angle),
        )
        obstacle, wall = arc_components(pose, steering, obstacles)
        return dict(
            pose_x=pose[0],
            pose_y=pose[1],
            pose_yaw=pose[2],
            obstacle=obstacle,
            wall=wall,
        )

    return measure


def fixture_previous(state="unresolved", truth=True, covered=False):
    row = dict.fromkeys(META, "0")
    row.update(
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
        envelope="noise_anchor_envelope",
        prior_optimistic=False,
        prior_conservative=False,
        optimistic_arc_gain=False,
        radius="0.12",
        heading_half_width_deg="4",
        state=state,
        truth_clear=truth,
        covered=covered,
    )
    return row


class EnvelopeRefinementTests(unittest.TestCase):
    def test_heading_examples_can_mix_while_position_stencil_stays_clear(self):
        result = refine(0.001, 1, measurement(0.001, [(25.6, 26.01)]))
        self.assertEqual(result["state"], "mixed")
        self.assertTrue(result["heading_mixed"])
        self.assertFalse(result["position_mixed"])
        self.assertEqual(result["cell_visits"], 0)
        self.assertEqual(
            {w["state"] for w in result["witnesses"]}, {"permit", "blocked"}
        )

    def test_position_examples_can_mix_while_heading_stencil_stays_clear(self):
        result = refine(0.12, 1, measurement(0.12, [(25.3, 26.05)]))
        self.assertEqual(result["state"], "mixed")
        self.assertTrue(result["position_mixed"])
        self.assertFalse(result["heading_mixed"])
        for witness in result["witnesses"]:
            self.assertLessEqual(math.hypot(*witness["parameters"][:2]), 1)
            self.assertEqual(witness["state"], witness_state(witness["geometry"], 1))

    def test_axis_flags_retain_overlapping_explanations(self):
        result = refine(0.12, 1, measurement(0.12, [(25.6, 26.01)]))
        self.assertTrue(result["both_axes_mixed"])
        self.assertFalse(result["one_axis_mixed"])

    def test_joint_search_can_find_ambiguity_missed_by_both_axis_stencils(self):
        result = refine(0.03, 1, measurement(0.03, [(25.6, 26.052)]))
        self.assertEqual(result["state"], "mixed")
        self.assertFalse(result["heading_mixed"])
        self.assertFalse(result["position_mixed"])
        self.assertTrue(result["mixed_after_cover"])
        self.assertGreater(result["cell_visits"], 0)

    def test_exhausted_search_does_not_certify_missing_examples(self):
        result = refine(0.03, 1, measurement(0.03, [(25.6, 26.052)]), budget=0)
        self.assertEqual(result["state"], "unresolved")
        self.assertTrue(result["budget_exhausted"])
        self.assertEqual(result["cell_visits"], 0)
        self.assertEqual(result["leaves"], [])
        self.assertEqual(result["witnesses"], [])

    def test_opposing_obstacles_can_be_certified_without_a_blocking_global_upper_bound(
        self,
    ):
        obstacles = [(25.3, 24.1), (25.3, 25.9)]
        centre = measurement(0.2, obstacles)((0, 0, 0))
        self.assertGreater(centre["obstacle"] + 0.2, 1)
        result = refine(0.2, 1, measurement(0.2, obstacles))
        self.assertEqual(result["state"], "blocked")
        self.assertGreater(len(result["leaves"]), 1)
        self.assertEqual(verify_cover(result["leaves"], 0.2, 1, ORIGIN), "blocked")

    def test_uniformly_clear_envelope_has_a_complete_root_certificate(self):
        result = refine(0.2, 1, measurement(0.2, [(40, 40)]))
        self.assertEqual(result["state"], "permit")
        self.assertEqual(len(result["leaves"]), 1)
        self.assertEqual(verify_cover(result["leaves"], 0.2, 1, ORIGIN), "permit")

    def test_a_missing_cover_branch_is_rejected(self):
        cell = Cell().children(0.2)[0]
        geometry = measurement(0.2, [(40, 40)])(cell.middle())
        allowance = cell.allowance(0.2)
        leaf = dict(
            cell=cell,
            state="permit",
            geometry=geometry,
            allowance=allowance,
            **cell_bounds(geometry, allowance)
        )
        with self.assertRaises(AssertionError):
            verify_cover([leaf], 0.2, 1, ORIGIN)

    def test_overlapping_parent_child_proofs_are_rejected(self):
        result = refine(0.2, 1, measurement(0.2, [(40, 40)]))
        root = result["leaves"][0]
        extra = dict(root, cell=Cell().children(0.2)[0])
        with self.assertRaises(AssertionError):
            verify_cover([root, extra], 0.2, 1, ORIGIN)

    def test_disk_pruning_retains_tangencies_and_rejects_false_outside_leaves(self):
        self.assertFalse(Cell("t", (1, 1.1, 0, 0.1, -4, 4)).outside_disk())
        self.assertFalse(Cell("t", (1 + 5e-13, 1.1, 0, 0.1, -4, 4)).outside_disk())
        self.assertTrue(Cell("t", (1 + 2e-12, 1.1, 0, 0.1, -4, 4)).outside_disk())
        with self.assertRaises(AssertionError):
            verify_cover(
                [dict(cell=Cell(), state="outside_disk", geometry=None)], 0.2, 1, ORIGIN
            )

    def test_cell_displacement_bounds_cover_interior_curved_paths_and_wall_extrema(
        self,
    ):
        rng = random.Random(20261016)
        radius, origin = 0.2, (1.1, 1.3, -0.4)
        obstacles = [(1.8, 1.9), (2.1, 0.6)]
        cells = [Cell()]
        for _ in range(7):
            cells = [child for cell in cells for child in cell.children(radius)]
        for steering in [-math.radians(35), 0, math.radians(35)]:
            measure = measurement(radius, obstacles, origin, steering)
            for cell in rng.sample(cells, 12):
                bounds = cell_bounds(measure(cell.middle()), cell.allowance(radius))
                for _ in range(5):
                    parameters = tuple(
                        rng.uniform(cell.box[i], cell.box[i + 1]) for i in [0, 2, 4]
                    )
                    geometry = measure(parameters)
                    for part in ["obstacle", "wall"]:
                        self.assertLessEqual(bounds[part + "_lower"], geometry[part])
                        self.assertLessEqual(geometry[part], bounds[part + "_upper"])

    def test_near_boundary_examples_are_not_used_as_robust_witnesses(self):
        self.assertIsNone(witness_state(dict(obstacle=2, wall=1), 1))
        self.assertIsNone(witness_state(dict(obstacle=1 + EPS / 2, wall=3), 1))
        self.assertEqual(witness_state(dict(obstacle=1 + EPS * 2, wall=3), 1), "permit")
        self.assertEqual(
            witness_state(dict(obstacle=1 - EPS * 2, wall=3), 1), "blocked"
        )

    def test_truth_changes_evaluation_but_not_search_and_uncovered_errors_remain_possible(
        self,
    ):
        outcome = refine(0.2, 1, measurement(0.2, [(40, 40)]))
        good = evaluated_record(fixture_previous(truth=True), outcome, 1, 27)
        bad = evaluated_record(fixture_previous(truth=False), outcome, 1, 27)
        self.assertTrue(good["true_permit"])
        self.assertTrue(bad["false_permit"])
        for key in ["state", "new_permit", "geometry_requests", "certificate_leaves"]:
            self.assertEqual(good[key], bad[key])
        with self.assertRaises(AssertionError):
            evaluated_record(
                fixture_previous(truth=False, covered=True), outcome, 1, 27
            )

    def test_summary_keeps_inherited_decisions_and_all_prior_unresolved_states(self):
        permit = refine(0.2, 1, measurement(0.2, [(40, 40)]))
        blocked = refine(0.2, 1, measurement(0.2, [(25.3, 24.1), (25.3, 25.9)]))
        mixed = refine(0.12, 1, measurement(0.12, [(25.3, 26.05)]))
        unknown = refine(0.03, 1, measurement(0.03, [(25.6, 26.052)]), budget=0)
        rows = [
            evaluated_record(fixture_previous("permit"), permit, 1, 0),
            evaluated_record(fixture_previous(truth=False), blocked, 2, 27),
            evaluated_record(fixture_previous(), mixed, 3, 27),
            evaluated_record(fixture_previous(), unknown, 4, 27),
        ]
        summary = {
            r["group"]: r
            for r in summarize(rows)
            if r["cohort"] == "all" and r["subset"] == "age_0p5"
        }
        all_rows = summary["all"]
        self.assertEqual(all_rows["checkpoints"], 4)
        self.assertEqual(all_rows["cases"], 1)
        self.assertEqual(all_rows["prior_unresolved"], 3)
        self.assertEqual(
            [all_rows[s] for s in ["permit", "blocked", "mixed", "unresolved"]],
            [1, 1, 1, 1],
        )
        self.assertEqual(all_rows["new_blocked"], 1)
        self.assertEqual(summary["optimistic_arc_gain"]["checkpoints"], 0)


if __name__ == "__main__":
    unittest.main()
