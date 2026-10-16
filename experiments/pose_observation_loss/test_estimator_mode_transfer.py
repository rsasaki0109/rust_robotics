"""Parent weights, structural joins, support and exact shift decompositions."""

from fractions import Fraction as F
import unittest

import analyze_estimator_mode_transfer as analysis
from analyze_contrast_risk_transfer import fit_thresholds


def row(
    sid="1",
    seed="269",
    partition="known",
    stratum="filter_single",
    tick=2,
    outside=True,
):
    estimator, anchor = analysis.STRATA[stratum]
    return dict(
        source_family="simultaneous",
        source_id=sid,
        cohort=partition + "-white",
        partition=partition,
        noise="white",
        variant="epoch",
        mode="random",
        seed=seed,
        scenario="fixture",
        planner="A*",
        view=(
            "sim_raw_recovery"
            if stratum == "recovery_single"
            else "sim_recovery_preview"
        ),
        source_event_ids="1",
        a_id="10",
        b_id="11",
        available=True,
        zero_or_roundoff_contrast=False,
        stratum=stratum,
        a_estimator=(
            "raw_observation" if stratum == "recovery_single" else "recovery_mean"
        ),
        b_estimator=estimator,
        anchor_mode=anchor,
        oracle_contrast=False,
        **{s + "_tick": tick for s in analysis.SCORES},
        **{t + "_outside": outside for t in analysis.TARGETS},
    )


def threshold(rule="fitted", tick=2):
    return dict(rule=rule, threshold_tick=tick)


def events(r, kinds=("received",)):
    return {
        str(i + 1): dict(event_kind=kind, **{k: r[k] for k in analysis.KEY})
        for i, kind in enumerate(kinds)
    }


def source(r):
    s = {k: str(v) for k, v in r.items()}
    meta = dict(s, pairing=r["view"][4:])
    return s, meta


class EstimatorModeTransferTests(unittest.TestCase):
    def test_structural_join_uses_family_and_full_source_identity(self):
        r = row()
        s, meta = source(r)
        lag = dict(s, source_family="lagged", oracle_contrast="True")
        joined = analysis.join_records([s, lag], [meta], events(r))
        self.assertEqual(len(joined), 1)
        self.assertEqual(joined[0]["stratum"], "filter_single")
        self.assertEqual(joined[0]["contrast_tick"], 2)
        self.assertIs(joined[0]["residual_outside"], True)
        for field, value in [
            ("seed", "270"),
            ("a_id", "99"),
            ("source_event_ids", "2"),
        ]:
            with self.subTest(field=field), self.assertRaises(AssertionError):
                analysis.join_records([dict(s, **{field: value})], [meta], events(r))

    def test_join_rejects_duplicates_missing_records_and_bad_estimator_anchor_pairs(
        self,
    ):
        r = row()
        s, meta = source(r)
        for ss, mm in [
            ([s, s], [meta]),
            ([s], [meta, meta]),
            ([], [meta]),
            ([s], [dict(meta, anchor_mode="preview_three")]),
            ([s], [dict(meta, b_estimator="recovery_mean")]),
        ]:
            with self.subTest(ss=ss, mm=mm), self.assertRaises(AssertionError):
                analysis.join_records(ss, mm, events(r))

    def test_parent_weights_survive_cases_that_cross_strata(self):
        rr = [row(str(i)) for i in range(3)] + [
            row("4", stratum="fusion_three"),
            row("5", seed="270", stratum="fusion_three"),
        ]
        weights = analysis.case_weights(rr)
        self.assertEqual(weights, [F(1, 4)] * 4 + [F(1)])
        cells = []
        for name in ["filter_single", "fusion_three"]:
            sub = [(r, w) for r, w in zip(rr, weights) if r["stratum"] == name]
            cells.append(
                analysis.cell(
                    [r for r, w in sub],
                    [w for r, w in sub],
                    "contrast",
                    threshold(),
                    F(2),
                )
            )
        self.assertEqual([c["case_mass"] for c in cells], [F(3, 4), F(5, 4)])
        self.assertEqual(sum(c["case_mass_share"] for c in cells), 1)
        self.assertEqual(sum(c["cases"] for c in cells), 3)
        self.assertEqual(len({tuple(r[k] for k in analysis.KEY) for r in rr}), 2)

    def test_retention_preserves_parent_weights_and_entire_tied_tick(self):
        rr = [
            row("1", tick=2),
            row("2", tick=2, outside=False),
            row("3", seed="270", tick=3),
        ]
        c = analysis.cell(rr, analysis.case_weights(rr), "contrast", threshold(), F(2))
        self.assertEqual(c["retained_rows"], 2)
        self.assertEqual(c["retained_case_mass"], 1)
        self.assertEqual(c["case_weighted_coverage"], F(1, 2))
        self.assertEqual(c["residual_joint_fraction"], F(1, 4))
        self.assertEqual(c["residual_case_weighted_risk"], F(1, 2))

    def test_empty_retention_has_zero_coverage_and_joint_mass_but_no_risk(self):
        c = analysis.cell([row(tick=3)], [F(1)], "contrast", threshold(), F(1))
        self.assertEqual(c["case_weighted_coverage"], 0)
        self.assertEqual(c["residual_joint_fraction"], 0)
        self.assertIsNone(c["residual_case_weighted_risk"])
        self.assertIsNone(analysis.ratio(F(), F()))

    def test_empty_stratum_and_missing_rule_keep_undefined_rates(self):
        c = analysis.cell([], [], "contrast", threshold(), F(2))
        self.assertEqual(c["case_mass_share"], 0)
        self.assertEqual(c["retained_case_mass"], 0)
        self.assertIsNone(c["case_weighted_coverage"])
        self.assertEqual(c["residual_case_weighted_auc"], "")
        c = analysis.cell(
            [row()], [F(1)], "contrast", threshold("no_training_data"), F(1)
        )
        for k in [
            "retained_rows",
            "retained_case_mass",
            "case_weighted_coverage",
            "residual_joint_fraction",
        ]:
            self.assertIsNone(c[k], k)
        self.assertEqual(analysis.export(c)["retained_rows"], "")

    def test_auc_keeps_parent_weights_direction_ties_and_missing_class(self):
        rr = [
            row("1", tick=1, outside=True),
            row("2", tick=1, outside=False),
            row("3", tick=2, outside=False),
        ]
        c = analysis.cell(
            rr, [F(1, 4), F(1, 4), F(1)], "contrast", threshold(), F(3, 2)
        )
        self.assertAlmostEqual(c["residual_case_weighted_auc"], 0.1)
        self.assertEqual(
            analysis.cell([row()], [F(1)], "contrast", threshold(), F(1))[
                "residual_case_weighted_auc"
            ],
            "",
        )

    def test_pure_composition_shift_is_exact(self):
        d = analysis.decomposition(
            [F(1, 2)] * 2, [F(3, 4), F(1, 4)], [F(1, 5), F(4, 5)], [F(1, 5), F(4, 5)]
        )
        self.assertEqual(d["known"], F(1, 2))
        self.assertEqual(d["new"], F(7, 20))
        self.assertEqual(d["composition"], F(-3, 20))
        self.assertEqual(d["within"], 0)
        self.assertEqual(d["closure_error"], 0)

    def test_pure_within_shift_and_order_dependent_interaction(self):
        pk, pn = [F(1, 2)] * 2, [F(3, 4), F(1, 4)]
        vk, vn = [F(1, 5), F(4, 5)], [F(2, 5), F(1, 5)]
        d = analysis.decomposition(pk, pk, vk, vn)
        self.assertEqual(d["composition"], 0)
        self.assertEqual(d["within"], F(-1, 5))
        d = analysis.decomposition(pk, pn, vk, vn)
        self.assertEqual(d["composition"], F(-3, 20))
        self.assertEqual(d["within"], 0)
        self.assertEqual(d["known_mix_new_rates"], F(3, 10))
        self.assertNotEqual(d["new_mix_known_rates"], d["known_mix_new_rates"])
        self.assertEqual(d["delta"], d["composition"] + d["within"])

    def test_new_only_stratum_blocks_forward_decomposition_without_imputation(self):
        d = analysis.decomposition(
            [F(1), F()], [F(1, 2)] * 2, [F(1, 5), None], [F(3, 5), F(4, 5)]
        )
        self.assertEqual(d["delta"], F(1, 2))
        self.assertIsNone(d["composition"])
        self.assertIsNone(d["within"])
        self.assertIsNone(d["new_mix_known_rates"])
        self.assertEqual(d["known_mix_new_rates"], F(3, 5))

    def test_known_only_stratum_blocks_reverse_standardization(self):
        d = analysis.decomposition(
            [F(1, 2)] * 2, [F(1), F()], [F(1, 5), F(4, 5)], [F(3, 5), None]
        )
        self.assertIsNone(d["known_mix_new_rates"])
        self.assertEqual(d["new_mix_known_rates"], F(1, 5))
        self.assertEqual(d["composition"], F(-3, 10))
        self.assertEqual(d["within"], F(2, 5))

    def test_zero_mass_on_both_sides_needs_no_invented_rate(self):
        d = analysis.decomposition(
            [F(1), F()], [F(1), F()], [F(1, 5), None], [F(3, 5), None]
        )
        self.assertEqual(d["composition"], 0)
        self.assertEqual(d["within"], F(2, 5))
        self.assertEqual(d["known_mix_new_rates"], F(3, 5))

    def test_empty_partition_keeps_single_observed_value_and_blank_comparison(self):
        d = analysis.decomposition(
            [F(), F()], [F(1), F()], [None, None], [F(1, 2), None]
        )
        self.assertIsNone(d["known"])
        self.assertEqual(d["new"], F(1, 2))
        for k in ["delta", "composition", "within", "known_mix_new_rates"]:
            self.assertIsNone(d[k])

    def test_standardized_risk_is_joint_mass_over_coverage(self):
        pk, pn = [F(9, 10), F(1, 10)], [F(1, 2)] * 2
        c = analysis.decomposition(pk, pn, [F(1)] * 2, [F(1, 10), F(1)])
        j = analysis.decomposition(pk, pn, [F(1, 2)] * 2, [F(1, 20), F(9, 10)])
        risk = analysis.ratio(j["known_mix_new_rates"], c["known_mix_new_rates"])
        self.assertEqual(risk, F(27, 38))
        self.assertNotEqual(risk, F(9, 10) * F(1, 2) + F(1, 10) * F(9, 10))

    def test_all_available_has_zero_coverage_shift_despite_mode_change(self):
        d = analysis.decomposition(
            [F(1, 4), F(3, 4)], [F(2, 3), F(1, 3)], [F(1)] * 2, [F(1)] * 2
        )
        self.assertEqual(d["known"], 1)
        self.assertEqual(d["new"], 1)
        for k in ["delta", "composition", "within", "closure_error"]:
            self.assertEqual(d[k], 0)

    def test_event_unions_keep_one_reference_per_kind_and_empty_groups(self):
        r = dict(row(), source_event_ids="1+2+3")
        ee = events(r, ("received", "received", "missed"))
        strata, transfer, aggregates = analysis.summaries([r], fit_thresholds([r]), ee)
        for kind in ["all", "received", "missed"]:
            key = ("known", "white", "epoch", kind, r["view"], "contrast", "q50")
            self.assertEqual(aggregates[key]["records"], 1)
        empty = [s for s in strata if s["event_kind"] == "precision_only"]
        self.assertTrue(empty)
        self.assertTrue(
            all(s["records"] == 0 and s["case_mass_share"] == "" for s in empty)
        )
        self.assertEqual((len(strata), len(transfer)), (5184, 864))

    def test_prior_crosscheck_detects_changed_retained_counts(self):
        r = row()
        c = analysis.cell([r], [F(1)], "contrast", threshold(), F(1))
        key = ("known", "white", "epoch", "all", r["view"], "contrast", "q50")
        prior = dict(zip([*analysis.GROUP, "score", "budget"], key))
        prior.update({k: str(v) for k, v in analysis.export(c).items()})
        prior["matched"] = "1"
        self.assertEqual(analysis.check_prior({key: c}, [prior]), 1)
        prior["retained_rows"] = "0"
        with self.assertRaises(AssertionError):
            analysis.check_prior({key: c}, [prior])


if __name__ == "__main__":
    unittest.main()
