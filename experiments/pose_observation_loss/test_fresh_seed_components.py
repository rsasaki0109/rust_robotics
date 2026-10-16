"""Additional-seed joins, signed components, inherited weights and fixed AUCs."""

from copy import deepcopy
from itertools import product
import math
import unittest

import analyze_fresh_seed_components as analysis


def fixture():
    common = dict(
        cohort="fresh-white",
        partition="fresh",
        noise="white",
        variant="epoch",
        mode="none",
        seed="277",
        scenario="fixture",
        planner="A*",
        state_id="7",
        time_s="150.0",
        truth_time_s="149.98",
        source_event_ids="1",
    )
    poses = []
    for kind, estimator, noise in [
        ("raw", "raw_observation", 0.1),
        ("recovery", "recovery_mean", -0.1),
        ("preview", "motion_filter", -0.1),
    ]:
        poses.append(
            dict(
                **common,
                pose_kind=kind,
                estimator=estimator,
                error_x=0.2 + noise,
                error_y=0,
                noise_x=noise,
                noise_y=0,
                anchor_x=0.2,
                anchor_y=0,
                drift_x=0,
                drift_y=0,
                transport_x=0,
                transport_y=0
            )
        )
    p = dict(
        **common,
        source_id="1",
        view="sim_raw_recovery",
        a_estimator="raw_observation",
        b_estimator="recovery_mean",
        anchor_mode="shared_single",
        stratum="recovery_single",
        a_weight=-0.25,
        b_weight=1.25,
        midpoint_variance=0.01,
        residual_variance=0.005,
        midpoint_x=0.2,
        midpoint_y=0,
        midpoint_norm=0.2,
        residual_x=0.05,
        residual_y=0,
        residual_norm=0.05
    )
    for part, x in [("noise", -0.15), ("anchor", 0.2), ("drift", 0), ("transport", 0)]:
        p["residual_" + part + "_x"] = x
        p["residual_" + part + "_y"] = 0
    s = {
        k: v
        for k, v in p.items()
        if k
        in [
            *common,
            "source_id",
            "view",
            "a_estimator",
            "b_estimator",
            "anchor_mode",
            "stratum",
        ]
    }
    s.update(
        source_family="simultaneous",
        available="True",
        midpoint_error_m=0.2,
        residual_error_m=0.05,
        midpoint_outside="True",
        residual_outside="False",
    )
    for score, value in [
        ("contrast", 0.2),
        ("midpoint_sigma", 0.1),
        ("residual_sigma", math.sqrt(0.005)),
    ]:
        s[score + "_score_m"] = value
        s[score + "_tick"] = analysis.score_tick(value)
    return [s], [p], poses


def row(
    seed="277", mode="filter_single", tick=1, anchor=0.2, view="sim_recovery_preview"
):
    terms = analysis.vector_terms(
        dict(noise=(0, 0), anchor=(anchor, 0), drift=(0, 0), transport=(0, 0)), 0.01
    )
    return dict(
        cohort="fresh-white",
        partition="fresh",
        noise="white",
        variant="epoch",
        mode="none",
        seed=seed,
        scenario="fixture",
        planner="A*",
        stratum=mode,
        view=view,
        source_event_ids="1+2",
        available=True,
        **{score + "_tick": tick for score in analysis.SCORES},
        midpoint=terms,
        residual=terms
    )


def cuts():
    p = []
    c = []
    for noise, view, score, budget in product(
        analysis.NOISES, analysis.SIM_VIEWS, analysis.SCORES, analysis.BUDGETS
    ):
        t = dict(
            noise=noise,
            view=view,
            score=score,
            budget=budget,
            rule="all_available" if budget == "all" else "fitted",
            threshold_tick=5,
        )
        p.append(t)
        for mode in analysis.STRATUM_NAMES[:-1]:
            c.append(dict(t, stratum=mode))
    return p, c


def selected(
    rows,
    mode="all",
    method="pooled",
    budget="q50",
    view="sim_recovery_preview",
    kind="all",
):
    return next(
        r
        for r in rows
        if r["noise"] == "white"
        and r["variant"] == "epoch"
        and r["event_kind"] == kind
        and r["view"] == view
        and r["stratum"] == mode
        and r["method"] == method
        and r["budget"] == budget
        and r["score"] == "residual_sigma"
        and r["target"] == "residual"
    )


EVENTS = {
    k: dict(
        cohort="fresh-white",
        partition="fresh",
        noise="white",
        variant="epoch",
        mode="none",
        seed="277",
        scenario="fixture",
        planner="A*",
        event_kind="received",
    )
    for k in ["1", "2"]
}


def summary(rows):
    events = deepcopy(EVENTS)
    for r in rows:
        # Give each case a distinct event union while retaining repeated-kind unions.
        if r["seed"] != "277":
            r = r.copy()
            r["source_event_ids"] = "3+4"
            for k in ["3", "4"]:
                events[k] = dict(EVENTS["1"], seed=r["seed"])
        yield r, events


def summarize(rows):
    pairs = list(summary(rows))
    events = {}
    for r, e in pairs:
        events.update(e)
    p, c = cuts()
    return analysis.summaries([r for r, e in pairs], p, c, events)


class FreshComponentTests(unittest.TestCase):
    def test_negative_saved_weights_and_signed_cancellation_survive_adapter(self):
        ss, pp, poses = fixture()
        rows, audits = analysis.reconstruct(ss, pp, poses)
        self.assertAlmostEqual(
            rows[0]["residual"]["energy"]["noise_anchor_cross_m2"], -0.06
        )
        self.assertAlmostEqual(
            rows[0]["residual"]["energy"]["observed_energy_m2"], 0.0025
        )
        self.assertTrue(rows[0]["midpoint"]["outside"]["observed"])
        self.assertFalse(rows[0]["residual"]["outside"]["observed"])
        self.assertLess(audits[0]["residual_vector_closure_m"], 1e-8)

    def test_missing_and_duplicate_pair_ids_are_rejected(self):
        ss, pp, poses = fixture()
        for scores, projections in [(ss, []), (ss + ss, pp), (ss, pp + pp)]:
            with self.assertRaisesRegex(AssertionError, "pair ID support"):
                analysis.reconstruct(scores, projections, poses)

    def test_pose_support_uses_full_case_and_rejects_duplicates(self):
        ss, pp, poses = fixture()
        bad = deepcopy(poses)
        bad[0]["seed"] = "278"
        for points in [bad, poses[:-1], poses + [poses[0]]]:
            with self.assertRaises(AssertionError):
                analysis.reconstruct(ss, pp, points)

    def test_event_union_and_estimator_mismatches_are_rejected(self):
        ss, pp, poses = fixture()
        for field, value in [("source_event_ids", "9"), ("estimator", "raw_fallback")]:
            bad = deepcopy(poses)
            bad[1][field] = value
            with self.assertRaises(AssertionError):
                analysis.reconstruct(ss, pp, bad)

    def test_observed_labels_and_score_ticks_cannot_silently_change(self):
        ss, pp, poses = fixture()
        for field, value in [("residual_outside", "True"), ("residual_sigma_tick", 7)]:
            bad = deepcopy(ss)
            bad[0][field] = value
            with self.assertRaises(AssertionError):
                analysis.reconstruct(bad, pp, poses)

    def test_parent_case_weights_precede_modes_and_retention(self):
        rr = [
            row(tick=9, anchor=0.1),
            row(mode="fusion_three", anchor=0.3),
            row(seed="278", anchor=0.2),
        ]
        values, _ = summarize(rr)
        retained = selected(values, mode="filter_single")
        self.assertEqual(retained["case_mass"], 1.5)
        self.assertEqual(retained["group_case_mass"], 2)
        self.assertEqual(retained["retained_case_mass"], 1)
        self.assertAlmostEqual(retained["case_weighted_coverage"], 2 / 3)
        self.assertAlmostEqual(retained["retained_share_of_group"], 0.5)
        self.assertAlmostEqual(retained["observed_energy_m2"], 0.04)
        available = selected(values, mode="filter_single", budget="all")
        self.assertAlmostEqual(available["observed_energy_m2"], 0.03)

    def test_missing_conditional_fit_blanks_whole_group(self):
        rr = [
            row(mode="recovery_single", view="sim_raw_recovery"),
            row(seed="278", mode="recovery_fallback_single", view="sim_raw_recovery"),
        ]
        values, _ = summarize(rr)
        r = selected(values, method="conditional", view="sim_raw_recovery")
        self.assertEqual(r["retention_support"], "unsupported")
        for field in [
            "retained_rows",
            "retained_case_mass",
            "observed_energy_m2",
            "observed_outside_rows",
            "observed_outside_fraction",
        ]:
            self.assertEqual(r[field], "")
        self.assertEqual(
            selected(
                values, method="conditional", budget="all", view="sim_raw_recovery"
            )["retained_rows"],
            2,
        )

    def test_empty_retention_has_zero_counts_and_blank_energies(self):
        values, _ = summarize([row(tick=9)])
        r = selected(values)
        self.assertEqual(r["retained_rows"], 0)
        self.assertEqual(r["retained_case_mass"], 0)
        self.assertEqual(r["case_weighted_coverage"], 0)
        self.assertEqual(r["observed_energy_m2"], "")
        self.assertEqual(r["observed_outside_fraction"], "")

    def test_auc_direction_is_never_reversed(self):
        _, ranking = summarize(
            [row(tick=1, anchor=0.2), row(seed="278", tick=9, anchor=0.05)]
        )
        r = next(
            r
            for r in ranking
            if all(
                r[k] == v
                for k, v in dict(
                    noise="white",
                    variant="epoch",
                    event_kind="all",
                    view="sim_recovery_preview",
                    stratum="all",
                    score="residual_sigma",
                    target="residual",
                ).items()
            )
        )
        self.assertEqual(r["observed_case_weighted_auc"], 0)
        self.assertEqual(r["modeled_case_weighted_auc"], 0)
        self.assertEqual(r["unmodeled_case_weighted_auc"], "")

    def test_auc_ties_get_half_credit_and_event_unions_do_not_duplicate(self):
        values, ranking = summarize([row(anchor=0.2), row(seed="278", anchor=0.05)])
        r = next(
            r
            for r in ranking
            if all(
                r[k] == v
                for k, v in dict(
                    noise="white",
                    variant="epoch",
                    event_kind="received",
                    view="sim_recovery_preview",
                    stratum="all",
                    score="residual_sigma",
                    target="residual",
                ).items()
            )
        )
        self.assertEqual(r["records"], 2)
        self.assertEqual(r["observed_case_weighted_auc"], 0.5)
        self.assertEqual(selected(values, kind="received")["records"], 2)

    def test_empty_grid_is_complete_and_rules_stay_immutable(self):
        p, c = cuts()
        original = deepcopy((p, c))
        values, ranking = analysis.summaries([], p, c, {})
        self.assertEqual((len(values), len(ranking)), (17280, 2160))
        self.assertEqual((p, c), original)
        self.assertEqual(selected(values)["observed_energy_m2"], "")


if __name__ == "__main__":
    unittest.main()
