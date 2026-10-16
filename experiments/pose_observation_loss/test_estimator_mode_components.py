"""Signed energy identities, frozen weights/labels and mode-conditional metrics."""

from fractions import Fraction as F
import math
import random
import unittest

import analyze_estimator_mode_components as analysis
from analyze_contrast_risk_transfer import fit_thresholds


def parts(noise=(0.0, 0.0), anchor=(0.0, 0.0), drift=(0.0, 0.0), transport=(0.0, 0.0)):
    return dict(noise=noise, anchor=anchor, drift=drift, transport=transport)


def row(
    sid="1", seed="269", stratum="filter_single", tick=2, vectors=None, variance=0.01
):
    return dict(
        source_id=sid,
        source_family="simultaneous",
        cohort="known-white",
        partition="known",
        noise="white",
        variant="epoch",
        mode="random",
        seed=seed,
        scenario="fixture",
        planner="A*",
        view="sim_recovery_preview",
        stratum=stratum,
        available=True,
        source_event_ids="1",
        **{s + "_tick": tick for s in analysis.SCORES},
        **{
            t: analysis.vector_terms(vectors or parts(anchor=(0.2, 0.0)), variance)
            for t in analysis.TARGETS
        },
    )


def point(pid, estimator, vectors):
    r = row()
    error = [sum(vectors[p][j] for p in analysis.PARTS) for j in range(2)]
    return dict(
        pose_id=pid,
        estimator=estimator,
        state_id="7",
        time_s="1.0",
        source_event_ids="1",
        **{k: r[k] for k in analysis.KEY},
        error_x=error[0],
        error_y=error[1],
        **{
            p + "_" + axis: vectors[p][j]
            for p in analysis.PARTS
            for j, axis in enumerate(["x", "y"])
        },
    )


def fixture():
    r = row(stratum="recovery_single")
    m = {
        k: r[k]
        for k in [
            "source_id",
            "source_family",
            *analysis.KEY,
            "partition",
            "noise",
            "source_event_ids",
            "stratum",
        ]
    }
    m.update(
        view="sim_raw_recovery",
        a_id="10",
        b_id="11",
        a_estimator="raw_observation",
        b_estimator="recovery_mean",
        anchor_mode="shared_single",
        available="True",
    )
    a = point("10", "raw_observation", parts(noise=(0.1, 0.0), anchor=(0.2, 0.0)))
    b = point("11", "recovery_mean", parts(noise=(-0.1, 0.0), anchor=(0.2, 0.0)))
    p = {
        k: m[k]
        for k in [
            "source_id",
            *analysis.KEY,
            "a_id",
            "b_id",
            "source_event_ids",
            "a_estimator",
            "b_estimator",
            "anchor_mode",
        ]
    }
    p.update(
        pairing="raw_recovery",
        state_id="7",
        time_s="1.0",
        a_weight=-0.25,
        b_weight=1.25,
        midpoint_variance=0.01,
        residual_variance=0.005,
        midpoint_x=0.2,
        midpoint_y=0.0,
        midpoint_norm=0.2,
        residual_x=0.05,
        residual_y=0.0,
        residual_norm=0.05,
    )
    for part, xy in parts(noise=(-0.15, 0.0), anchor=(0.2, 0.0)).items():
        for j, axis in enumerate(["x", "y"]):
            p["residual_" + part + "_" + axis] = xy[j]
    s = dict(
        m,
        **{q + "_tick": "2" for q in analysis.SCORES},
        midpoint_outside="True",
        residual_outside="False",
    )
    return m, p, [a, b], s


class EstimatorModeComponentsTests(unittest.TestCase):
    def test_four_component_energy_keeps_all_six_signed_cross_terms(self):
        p = parts((0.1, 0.2), (0.2, -0.1), (-0.1, 0.1), (0.05, -0.2))
        t = analysis.vector_terms(p, 0.02)
        e = t["energy"]
        self.assertAlmostEqual(e["observed_energy_m2"], 0.25**2)
        self.assertAlmostEqual(
            e["observed_energy_m2"],
            sum(e[x + "_energy_m2"] for x in analysis.PARTS)
            + sum(
                e[a + "_" + b + "_cross_m2"]
                for a, b in analysis.combinations(analysis.PARTS, 2)
            ),
        )
        self.assertAlmostEqual(
            e["observed_energy_m2"],
            e["modeled_energy_m2"]
            + e["unmodeled_energy_m2"]
            + e["modeled_unmodeled_cross_m2"],
        )
        self.assertLess(e["modeled_unmodeled_cross_m2"], 0)

    def test_cancellation_can_make_observed_inside_with_modeled_outside(self):
        t = analysis.vector_terms(parts(anchor=(0.2, 0), drift=(-0.15, 0)), 0.01)
        self.assertEqual(t["cell"], "modeled_outside_observed_inside")
        self.assertTrue(t["outside"]["unmodeled"])
        self.assertAlmostEqual(t["energy"]["modeled_unmodeled_cross_m2"], -0.06)
        self.assertGreater(
            t["energy"]["modeled_energy_m2"], t["energy"]["observed_energy_m2"]
        )

    def test_unmodeled_component_can_push_modeled_inside_to_observed_outside(self):
        t = analysis.vector_terms(parts(noise=(0.02, 0), transport=(0.2, 0)), 0.01)
        self.assertEqual(t["cell"], "modeled_inside_observed_outside")
        self.assertGreater(t["energy"]["modeled_unmodeled_cross_m2"], 0)

    def test_closed_accuracy_disk_and_two_axis_model_energy(self):
        t = analysis.vector_terms(parts(anchor=(0.1, 0)), 0.012)
        self.assertFalse(t["outside"]["observed"])
        self.assertFalse(t["outside"]["modeled"])
        self.assertEqual(t["energy"]["partial_model_energy_m2"], 0.024)
        self.assertTrue(
            analysis.vector_terms(parts(anchor=(0.10000001, 0)), 0.012)["outside"][
                "observed"
            ]
        )

    def test_rotation_preserves_every_energy_and_outside_label(self):
        rng = random.Random(276)
        for _ in range(25):
            p = {
                k: (rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3))
                for k in analysis.PARTS
            }
            angle = rng.random() * math.tau
            c, s = math.cos(angle), math.sin(angle)
            rotated = {k: (c * x - s * y, s * x + c * y) for k, (x, y) in p.items()}
            a, b = analysis.vector_terms(p, 0.01), analysis.vector_terms(rotated, 0.01)
            for field in analysis.ENERGIES:
                self.assertAlmostEqual(a["energy"][field], b["energy"][field])
            self.assertEqual(a["outside"], b["outside"])

    def test_negative_residual_weights_are_kept_in_every_component(self):
        m, p, poses, s = fixture()
        rows, audits = analysis.reconstruct([m], [p], poses, [s])
        r = rows[0]
        self.assertAlmostEqual(r["residual"]["parts"]["noise"][0], -0.15)
        self.assertAlmostEqual(r["residual"]["parts"]["anchor"][0], 0.2)
        self.assertAlmostEqual(r["midpoint"]["parts"]["noise"][0], 0)
        self.assertFalse(r["residual"]["outside"]["observed"])
        self.assertTrue(r["midpoint"]["outside"]["observed"])
        self.assertLess(audits[0]["residual_component_difference_m"], 1e-12)

    def test_bad_component_sum_and_inconsistent_source_vector_are_rejected(self):
        m, p, poses, s = fixture()
        bad = [dict(poses[0], anchor_x=0.4), poses[1]]
        with self.assertRaises(AssertionError):
            analysis.reconstruct([m], [p], bad, [s])
        with self.assertRaises(AssertionError):
            analysis.vector_terms(parts(anchor=(0.2, 0)), 0.01, observed=(0.3, 0))

    def test_join_checks_original_labels_metadata_and_unique_ids(self):
        m, p, poses, s = fixture()
        for mm, pp, ii, ss in [
            ([m, m], [p], poses, [s]),
            ([m], [p], poses + [poses[0]], [s]),
            ([m], [p], poses, [s, s]),
            ([m], [dict(p, state_id="8")], poses, [s]),
            ([m], [dict(p, anchor_mode="preview_three")], poses, [s]),
            ([m], [p], poses, [dict(s, residual_outside="True")]),
        ]:
            with self.subTest(pp=pp), self.assertRaises(AssertionError):
                analysis.reconstruct(mm, pp, ii, ss)

    def test_empty_selection_keeps_counts_zero_and_undefined_means_risks(self):
        d = analysis.component_metrics([], [], "residual")
        self.assertTrue(all(d[k] is None for k in analysis.ENERGIES))
        self.assertEqual(d["observed_outside_rows"], 0)
        self.assertEqual(d["observed_outside_case_mass"], 0)
        self.assertIsNone(d["observed_outside_fraction"])
        self.assertIsNone(d["observed_to_partial_model_energy_ratio"])

    def test_zero_model_reference_keeps_ratio_undefined_despite_observed_energy(self):
        r = row(variance=0.0)
        d = analysis.component_metrics([r], [F(1)], "residual")
        self.assertAlmostEqual(d["observed_energy_m2"], 0.04)
        self.assertEqual(d["partial_model_rms_m"], 0)
        self.assertIsNone(d["observed_to_partial_model_energy_ratio"])

    def test_retained_metrics_do_not_rebalance_cases_after_selection(self):
        rr = [
            row("1"),
            row("2"),
            row("3", tick=3, vectors=parts(anchor=(0.05, 0))),
            row("4", seed="270", vectors=parts(anchor=(0.1, 0))),
        ]
        ww = analysis.case_weights(rr)
        kept = [
            (r, w)
            for r, w in zip(rr, ww)
            if analysis.chosen(r, "contrast", dict(rule="fitted", threshold_tick=2))
        ]
        d = analysis.component_metrics(
            [r for r, w in kept], [w for r, w in kept], "residual"
        )
        self.assertAlmostEqual(d["observed_energy_m2"], 0.022)
        self.assertEqual(d["observed_outside_case_mass"], F(2, 3))
        self.assertEqual(d["observed_outside_fraction"], F(2, 5))

    def test_stratum_crossing_cases_keep_parent_weights_in_energy_means(self):
        rr = [row(str(i)) for i in range(3)] + [
            row("4", stratum="fusion_three"),
            row(
                "5", seed="270", stratum="fusion_three", vectors=parts(anchor=(0.1, 0))
            ),
        ]
        ww = analysis.case_weights(rr)
        sub = [(r, w) for r, w in zip(rr, ww) if r["stratum"] == "fusion_three"]
        d = analysis.component_metrics(
            [r for r, w in sub], [w for r, w in sub], "residual"
        )
        self.assertEqual(sum(w for r, w in sub), F(5, 4))
        self.assertAlmostEqual(d["observed_energy_m2"], 0.016)
        self.assertEqual(d["observed_outside_fraction"], F(1, 5))

    def test_diagnostic_auc_has_distinct_labels_and_half_credit_for_ties(self):
        rr = [
            row("1", tick=1, vectors=parts(anchor=(0.15, 0), drift=(-0.15, 0))),
            row("2", tick=2, vectors=parts(anchor=(0.02, 0), drift=(0.2, 0))),
            row("3", tick=1, vectors=parts(anchor=(0.02, 0))),
        ]
        d = analysis.diagnostic_ranking(rr, [F(1)] * 3, "residual", "contrast")
        self.assertEqual(d["observed_case_weighted_auc"], 1)
        self.assertEqual(d["modeled_case_weighted_auc"], 0.25)
        self.assertEqual(d["unmodeled_case_weighted_auc"], 0.75)

    def test_absent_auc_classes_stay_blank(self):
        d = analysis.diagnostic_ranking([row()], [F(1)], "midpoint", "residual_sigma")
        self.assertTrue(all(d[k + "_case_weighted_auc"] == "" for k in analysis.LABELS))
        self.assertEqual(
            analysis.diagnostic_ranking([], [], "midpoint", "contrast")[
                "observed_case_weighted_auc"
            ],
            "",
        )

    def test_all_four_diagnostic_cells_partition_inherited_mass(self):
        rr = [
            row("1", vectors=parts(anchor=(0.05, 0))),
            row("2", vectors=parts(anchor=(0.05, 0), drift=(0.2, 0))),
            row("3", vectors=parts(anchor=(0.2, 0), drift=(-0.2, 0))),
            row("4", vectors=parts(anchor=(0.2, 0))),
        ]
        weights = [F(1, 4), F(1, 2), F(3, 4), F(1)]
        d = analysis.component_metrics(rr, weights, "midpoint")
        for c, w in zip(analysis.CELLS, weights):
            self.assertEqual(d[c + "_rows"], 1)
            self.assertEqual(d[c + "_case_mass"], w)
        self.assertEqual(sum(d[c + "_fraction"] for c in analysis.CELLS), 1)

    def test_frozen_selection_retains_ties_and_ignores_component_outcomes(self):
        a = row(tick=2)
        b = row(tick=2, vectors=parts(anchor=(0.02, 0)))
        t = dict(rule="fitted", threshold_tick="2")
        self.assertTrue(analysis.chosen(a, "contrast", t))
        self.assertTrue(analysis.chosen(b, "contrast", t))
        self.assertFalse(analysis.chosen(row(tick=3), "contrast", t))
        self.assertTrue(
            analysis.chosen(row(tick=999), "contrast", dict(rule="all_available"))
        )

    def test_complete_groups_keep_empty_modes_and_deduplicate_event_kind_unions(self):
        r = dict(row(), source_event_ids="1+2+3")
        ee = {
            str(i + 1): dict(event_kind=k)
            for i, k in enumerate(["received", "received", "missed"])
        }
        summary, ranks = analysis.summaries([r], fit_thresholds([r]), ee)
        self.assertEqual((len(summary), len(ranks)), (13824, 3456))
        for kind in ["all", "received", "missed"]:
            match = [
                s
                for s in summary
                if (
                    s["partition"],
                    s["noise"],
                    s["variant"],
                    s["event_kind"],
                    s["view"],
                    s["stratum"],
                    s["score"],
                    s["budget"],
                    s["target"],
                )
                == (
                    "known",
                    "white",
                    "epoch",
                    kind,
                    r["view"],
                    r["stratum"],
                    "contrast",
                    "all",
                    "residual",
                )
            ]
            self.assertEqual(len(match), 1)
            self.assertEqual(match[0]["records"], 1)
        self.assertTrue(
            all(
                s["records"] == 0 and s["observed_energy_m2"] == ""
                for s in summary
                if s["event_kind"] == "precision_only"
            )
        )

    def test_invalid_model_variance_and_nonfinite_components_fail(self):
        for variance, p in [
            (-0.1, parts()),
            (float("nan"), parts()),
            (0.1, parts(noise=(float("inf"), 0))),
        ]:
            with self.subTest(variance=variance), self.assertRaises(AssertionError):
                analysis.vector_terms(p, variance)


if __name__ == "__main__":
    unittest.main()
