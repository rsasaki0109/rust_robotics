"""Regression tests for original state selection and frozen-cutoff transfer."""

from collections import deque
from copy import deepcopy
from itertools import product
import unittest

from analyze_fresh_seed_cutoffs import (
    BUDGETS,
    NOISES,
    SCORES,
    SIM_VIEWS,
    STRATA,
    edge_windows,
    evaluate,
    focused,
    frozen,
    stratum,
    support,
)
from analyze_epoch_position_components import Replay


def row(seed="277", step=1, mode="filter_single", view="sim_recovery_preview"):
    return dict(
        source_id=step,
        cohort="fresh-white",
        partition="fresh",
        noise="white",
        variant="epoch",
        mode="none",
        seed=seed,
        scenario="open_50x50",
        planner="A*",
        state_id=step,
        view=view,
        stratum=mode,
        source_event_ids="1+2",
        available=True,
        a_estimator="recovery_mean",
        contrast_tick=1,
        midpoint_sigma_tick=1,
        residual_sigma_tick=1,
        midpoint_error_m=0.1,
        midpoint_outside=False,
        residual_error_m=0.2,
        residual_outside=True,
    )


def cutoffs():
    pooled = []
    conditional = []
    for noise, view, score, budget in product(NOISES, SIM_VIEWS, SCORES, BUDGETS):
        t = dict(
            noise=noise,
            view=view,
            score=score,
            budget=budget,
            threshold_tick=5,
            training_partition="known",
            rule="all_available" if budget == "all" else "fitted",
            threshold_id=len(pooled) + 1,
        )
        pooled.append(t)
        for s in STRATA:
            conditional.append(dict(t, stratum=s))
    return pooled, conditional


def choose(rows, stratum="all", budget="q50", view="sim_recovery_preview", kind="all"):
    return next(
        r
        for r in rows
        if r["noise"] == "white"
        and r["variant"] == "epoch"
        and r["event_kind"] == kind
        and r["view"] == view
        and r["stratum"] == stratum
        and r["score"] == "residual_sigma"
        and r["budget"] == budget
    )


class FreshCutoffTests(unittest.TestCase):
    def test_windows_include_pre_adjacent_edge_and_clipped_endpoint(self):
        edges = edge_windows(
            [dict(event_id=1, event_time_s=150), dict(event_id=2, event_time_s=180)]
        )
        self.assertEqual(edges[7500], [("1", 0)])
        self.assertEqual(edges[7575], [("1", 75)])
        self.assertEqual(edges[9000], [("2", 0)])
        self.assertNotIn(9001, edges)
        self.assertEqual(len(edges), 77)

    def test_overlapping_event_windows_retain_both_memberships(self):
        edges = edge_windows(
            [dict(event_id=1, event_time_s=150), dict(event_id=2, event_time_s=150.02)]
        )
        self.assertEqual(edges[7501], [("1", 1), ("2", 0)])

    def test_focus_requires_joint_route_and_readiness_at_either_endpoint(self):
        joint = dict(route="joint", active_ready=True)
        other = dict(route="independent", active_ready=True)
        blocked = dict(route="joint", active_ready=False)
        self.assertTrue(focused(joint, blocked))
        self.assertTrue(focused(blocked, joint))
        self.assertFalse(focused(other, blocked))
        self.assertFalse(focused(blocked, blocked))

    def test_snapshot_does_not_follow_later_window_mutations(self):
        state = Replay()
        state.motion.append("a")
        state.recovery.append("b")
        state.groups.append(dict(epoch=1, samples=deque(["c"], maxlen=25)))
        saved = frozen(state)
        state.motion.clear()
        state.recovery.append("d")
        state.groups[0]["samples"].clear()
        state.odom = (1, 2, 3)
        self.assertEqual(list(saved.motion), ["a"])
        self.assertEqual(list(saved.recovery), ["b"])
        self.assertEqual(list(saved.groups[0]["samples"]), ["c"])
        self.assertEqual(saved.odom, (0, 0, 0))

    def test_fallback_does_not_masquerade_as_a_trained_recovery_mean(self):
        self.assertEqual(
            stratum("sim_raw_recovery", "raw_fallback", "shared_single"),
            "recovery_fallback_single",
        )
        self.assertEqual(
            stratum("sim_raw_recovery", "recovery_mean", "shared_single"),
            "recovery_single",
        )
        with self.assertRaises(AssertionError):
            stratum("sim_raw_recovery", "raw_fallback", "preview_three")

    def test_missing_training_support_blanks_whole_group_except_all_available(self):
        rows = [
            row(mode="recovery_single", view="sim_raw_recovery"),
            row(
                seed="278",
                step=2,
                mode="recovery_fallback_single",
                view="sim_raw_recovery",
            ),
        ]
        p, c = cutoffs()
        saved = deepcopy((p, c))
        values = evaluate(
            rows,
            p,
            c,
            {"1": dict(event_kind="received"), "2": dict(event_kind="received")},
            ["fresh"],
        )
        r = choose(values, view="sim_raw_recovery")
        self.assertEqual(r["conditional_support"], "unsupported")
        self.assertEqual(r["conditional_unsupported_rows"], 1)
        self.assertEqual(r["conditional_unsupported_case_mass"], 1)
        self.assertEqual(r["pooled_case_weighted_coverage"], 1)
        self.assertEqual(r["conditional_case_weighted_coverage"], "")
        self.assertEqual(r["both_rows"], "")
        self.assertEqual(
            choose(values, budget="all", view="sim_raw_recovery")[
                "conditional_case_weighted_coverage"
            ],
            1,
        )
        self.assertEqual((p, c), saved)

    def test_case_weights_precede_modes_and_retention(self):
        rows = [row(), row(step=2, mode="fusion_three"), row(seed="278", step=3)]
        rows[0]["residual_sigma_tick"] = 9
        p, c = cutoffs()
        values = evaluate(
            rows,
            p,
            c,
            {"1": dict(event_kind="received"), "2": dict(event_kind="received")},
            ["fresh"],
        )
        r = choose(values, stratum="filter_single")
        self.assertEqual(r["case_mass"], 1.5)
        self.assertEqual(r["group_case_mass"], 2)
        self.assertEqual(r["pooled_retained_case_mass"], 1)
        self.assertAlmostEqual(r["pooled_case_weighted_coverage"], 2 / 3)
        self.assertEqual(choose(values)["pooled_case_weighted_coverage"], 0.75)

    def test_event_kind_union_does_not_duplicate_states(self):
        p, c = cutoffs()
        values = evaluate(
            [row()],
            p,
            c,
            {"1": dict(event_kind="received"), "2": dict(event_kind="received")},
            ["fresh"],
        )
        self.assertEqual(choose(values)["records"], 1)
        self.assertEqual(choose(values, kind="received")["records"], 1)
        self.assertEqual(choose(values, kind="missed")["records"], 0)
        self.assertEqual(
            choose(values, kind="missed")["pooled_residual_case_weighted_risk"], ""
        )

    def test_empty_grid_and_unused_fallback_stratum_are_explicit(self):
        p, c = cutoffs()
        values = evaluate([], p, c, {}, ["fresh"])
        self.assertEqual(len(values), 4320)
        r = choose(values, stratum="recovery_fallback_single", view="sim_raw_recovery")
        self.assertEqual(r["conditional_support"], "empty")
        self.assertEqual(r["pooled_case_weighted_coverage"], "")

    def test_case_support_keeps_mixed_and_no_audit_cases(self):
        a = row()
        b = row(seed="278")
        values = support(
            [a, b], [a, dict(a, source_id=2, state_id=2, stratum="fusion_three")]
        )
        self.assertEqual(values[0]["support"], "mixed")
        self.assertEqual(values[0]["audited_states"], 2)
        self.assertEqual(values[1]["support"], "no_audit")


if __name__ == "__main__":
    unittest.main()
