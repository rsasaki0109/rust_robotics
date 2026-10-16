"""Whole-seed isolation, missing mode support and complete-fold sensitivity."""

from collections import Counter
import unittest

import analyze_seed_exclusion_cutoffs as analysis


def row(
    sid,
    seed,
    partition="known",
    stratum="fusion_three",
    tick=2,
    noise="white",
    variant="epoch",
    mode="random",
    view="sim_recovery_preview",
):
    return dict(
        source_family="simultaneous",
        source_id=str(sid),
        cohort=partition + "-" + noise,
        partition=partition,
        noise=noise,
        variant=variant,
        mode=mode,
        seed=str(seed),
        scenario="fixture",
        planner="A*",
        view=view,
        stratum=stratum,
        available=True,
        source_event_ids=str(sid),
        **{s + "_tick": tick for s in analysis.SCORES},
        **{t + "_error_m": 0.2 for t in analysis.TARGETS},
        **{t + "_outside": True for t in analysis.TARGETS}
    )


def population():
    return [
        row(1, 269),
        row(2, 270),
        row(3, 271, stratum="filter_single", tick=9),
        row(4, 274, partition="new", stratum="filter_single", tick=9),
        row(5, 275, partition="new"),
    ]


def account(rr):
    return [{**{k: r[k] for k in analysis.KEY}, "selected": True} for r in rr]


def reference_stub(partition="new"):
    r = dict.fromkeys(analysis.REFERENCE_FIELDS, "")
    r.update(
        partition=partition,
        noise="white",
        variant="epoch",
        event_kind="all",
        view="sim_recovery_preview",
        stratum="all",
        score="residual_sigma",
        budget="q50",
        records=2,
        cases=1,
        case_mass=1.0,
        group_case_mass=1.0,
        case_mass_share=1.0,
        conditional_support="complete",
        conditional_unsupported_rows=0,
        conditional_unsupported_case_mass=0.0,
        conditional_unsupported_strata="",
    )
    for m in analysis.METHODS:
        r[m + "_case_weighted_coverage"] = 0.4 if m == "pooled" else 0.6
        for t in analysis.TARGETS:
            r[m + "_" + t + "_case_weighted_risk"] = 0.2 if m == "pooled" else 0.4
    for t in analysis.TARGETS:
        r[t + "_case_weighted_risk_delta"] = 0.2
    return r


def fold_stubs(reference):
    return [
        dict(reference, **analysis.fold_info(seed), evaluation="existing_new")
        for seed in analysis.SEEDS
    ]


class SeedExclusionCutoffsTests(unittest.TestCase):
    def test_each_omitted_seed_is_excluded_with_all_of_its_cases(self):
        rr = []
        for seed in analysis.SEEDS:
            for noise in analysis.NOISES:
                for variant in analysis.ARMS:
                    for mode in ["random", "blackout"]:
                        rr.append(
                            row(
                                len(rr) + 1,
                                seed,
                                noise=noise,
                                variant=variant,
                                mode=mode,
                            )
                        )
        rr.append(row(len(rr) + 1, 274, partition="new"))
        for seed in analysis.SEEDS:
            train, evaluation = analysis.split_fold(rr, seed)
            self.assertEqual(
                {int(r["seed"]) for r in train}, set(analysis.SEEDS) - {seed}
            )
            self.assertEqual(sum(r["partition"] == "known" for r in evaluation), 8)
            self.assertTrue(
                all(
                    int(r["seed"]) == seed
                    for r in evaluation
                    if r["partition"] == "known"
                )
            )
            self.assertFalse(
                {tuple(r[k] for k in analysis.KEY) for r in train}
                & {tuple(r[k] for k in analysis.KEY) for r in evaluation}
            )

    def test_held_and_existing_new_scores_and_labels_cannot_change_a_fold_fit(self):
        rr = population()
        a, b, _ = analysis.fit_fold(rr, 269)
        changed = [
            (
                dict(
                    r,
                    **{s + "_tick": 999 for s in analysis.SCORES},
                    **{t + "_error_m": 0.01 for t in analysis.TARGETS},
                    **{t + "_outside": False for t in analysis.TARGETS}
                )
                if r["seed"] == "269" or r["partition"] == "new"
                else r
            )
            for r in rr
        ]
        c, d, _ = analysis.fit_fold(changed, 269)
        self.assertEqual(a, c)
        self.assertEqual(b, d)
        for t in [*a, *b]:
            self.assertNotIn("269", t["training_seeds"].split("+"))
            self.assertNotIn("274", t["training_seeds"].split("+"))

    def test_omitting_unique_filter_seed_preserves_missing_conditional_fit(self):
        pooled, conditional, evaluation = analysis.fit_fold(population(), 271)
        cut = next(
            t
            for t in conditional
            if (t["noise"], t["view"], t["stratum"], t["score"], t["budget"])
            == (
                "white",
                "sim_recovery_preview",
                "filter_single",
                "residual_sigma",
                "q50",
            )
        )
        self.assertEqual(cut["rule"], "no_training_data")
        self.assertEqual(cut["training_rows"], 0)
        self.assertIsNone(cut["threshold_tick"])
        self.assertEqual(cut["training_seeds"], "")
        self.assertTrue(
            any(
                r["partition"] == "known" and r["stratum"] == "filter_single"
                for r in evaluation
            )
        )
        self.assertTrue(all(t["training_seeds"] in ["", "269+270"] for t in pooled))

    def test_unsupported_whole_population_stays_blank_while_fusion_remains_defined(
        self,
    ):
        rr = population()
        p, c, e = analysis.fit_fold(rr, 271)
        events = {r["source_event_ids"]: dict(event_kind="received") for r in rr}
        summaries = analysis.summaries(e, p, c, events)
        selected = [
            r
            for r in summaries
            if (
                r["partition"],
                r["noise"],
                r["variant"],
                r["event_kind"],
                r["view"],
                r["score"],
                r["budget"],
            )
            == (
                "new",
                "white",
                "epoch",
                "all",
                "sim_recovery_preview",
                "residual_sigma",
                "q50",
            )
        ]
        whole = next(r for r in selected if r["stratum"] == "all")
        self.assertEqual(whole["records"], 2)
        self.assertEqual(whole["conditional_unsupported_rows"], 1)
        self.assertEqual(whole["conditional_support"], "unsupported")
        self.assertEqual(whole["conditional_retained_rows"], "")
        self.assertEqual(whole["conditional_only_rows"], "")
        fusion = next(r for r in selected if r["stratum"] == "fusion_three")
        self.assertEqual(fusion["conditional_support"], "complete")
        self.assertEqual(fusion["conditional_retained_rows"], 1)
        all_available = next(
            r
            for r in summaries
            if (
                r["partition"],
                r["noise"],
                r["variant"],
                r["event_kind"],
                r["view"],
                r["stratum"],
                r["score"],
                r["budget"],
            )
            == (
                "new",
                "white",
                "epoch",
                "all",
                "sim_recovery_preview",
                "all",
                "residual_sigma",
                "all",
            )
        )
        self.assertEqual(all_available["conditional_retained_rows"], 2)

    def test_every_accounted_known_case_is_held_once_and_new_cases_recur(self):
        rr = population()
        aa = account(rr)
        aa.append(dict(aa[0], scenario="unaudited", selected=False))
        roles, folds = analysis.case_roles(aa, rr)
        self.assertEqual(len(roles), 3 * len(aa))
        self.assertEqual(len(folds), 3)
        held = Counter(
            tuple(r[k] for k in analysis.KEY)
            for r in roles
            if r["role"] == "held_known"
        )
        self.assertEqual(len(held), 4)
        self.assertTrue(all(n == 1 for n in held.values()))
        recurring = Counter(
            tuple(r[k] for k in analysis.KEY)
            for r in roles
            if r["role"] == "existing_new"
        )
        self.assertTrue(all(n == 3 for n in recurring.values()))
        self.assertEqual(sum(r["simultaneous_pairs"] for r in roles), 3 * len(rr))
        self.assertTrue(
            all(
                r["simultaneous_pairs"] == 0
                for r in roles
                if r["scenario"] == "unaudited"
            )
        )

    def test_duplicate_accounting_and_unaccounted_source_cases_are_rejected(self):
        rr = population()
        aa = account(rr)
        with self.assertRaises(AssertionError):
            analysis.case_roles([*aa, aa[0]], rr)
        with self.assertRaises(AssertionError):
            analysis.case_roles(aa[:-1], rr)

    def test_all_known_reference_is_exported_only_for_existing_new_evaluation(self):
        result = analysis.new_reference(
            [reference_stub("known"), reference_stub("new")]
        )
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["partition"], "new")
        self.assertNotIn("held_seed", result[0])

    def test_complete_fold_ranges_keep_min_max_span_without_averaging(self):
        ref = reference_stub()
        folds = fold_stubs(ref)
        for r, v in zip(folds, [0.2, 0.4, 0.8]):
            r["pooled_case_weighted_coverage"] = v
        result = analysis.stability([ref], folds)[0]
        self.assertEqual(result["pooled_case_weighted_coverage_defined_folds"], 3)
        self.assertEqual(result["pooled_case_weighted_coverage_min"], 0.2)
        self.assertEqual(result["pooled_case_weighted_coverage_max"], 0.8)
        self.assertAlmostEqual(result["pooled_case_weighted_coverage_span"], 0.6)
        self.assertEqual(result["cases"], 1)
        self.assertEqual(result["full_fit_pooled_case_weighted_coverage"], 0.4)

    def test_missing_mode_does_not_turn_two_fold_spread_into_a_complete_range(self):
        ref = reference_stub()
        folds = fold_stubs(ref)
        folds[-1].update(
            conditional_support="unsupported", conditional_unsupported_rows=1
        )
        for field in analysis.METRICS:
            if field.startswith("conditional_"):
                folds[-1][field] = ""
        for field in analysis.DIFFERENCES:
            folds[-1][field] = ""
        result = analysis.stability([ref], folds)[0]
        self.assertEqual(result["conditional_case_weighted_coverage_defined_folds"], 2)
        self.assertEqual(
            result["conditional_case_weighted_coverage_undefined_folds"], 1
        )
        self.assertEqual(result["conditional_case_weighted_coverage_span"], "")
        self.assertEqual(result["conditional_unsupported_folds"], 1)
        self.assertEqual(result["residual_case_weighted_risk_delta_higher_folds"], 2)
        self.assertEqual(result["residual_case_weighted_risk_delta_undefined_folds"], 1)

    def test_empty_retention_can_hide_risk_without_hiding_coverage(self):
        ref = reference_stub()
        folds = fold_stubs(ref)
        folds[0]["conditional_case_weighted_coverage"] = 0
        for t in analysis.TARGETS:
            folds[0]["conditional_" + t + "_case_weighted_risk"] = ""
            folds[0][t + "_case_weighted_risk_delta"] = ""
        result = analysis.stability([ref], folds)[0]
        self.assertEqual(result["conditional_case_weighted_coverage_defined_folds"], 3)
        self.assertEqual(result["conditional_case_weighted_coverage_min"], 0)
        self.assertEqual(
            result["conditional_residual_case_weighted_risk_defined_folds"], 2
        )
        self.assertEqual(result["conditional_residual_case_weighted_risk_span"], "")
        self.assertEqual(result["conditional_unsupported_folds"], 0)

    def test_empty_populations_keep_zero_counts_and_undefined_ranges(self):
        ref = reference_stub()
        ref.update(
            records=0,
            cases=0,
            case_mass=0.0,
            group_case_mass=0.0,
            case_mass_share="",
            conditional_support="empty",
        )
        for field in [*analysis.METRICS, *analysis.DIFFERENCES]:
            ref[field] = ""
        result = analysis.stability([ref], fold_stubs(ref))[0]
        self.assertEqual(result["empty_population_folds"], 3)
        self.assertEqual(result["pooled_case_weighted_coverage_defined_folds"], 0)
        self.assertEqual(result["pooled_case_weighted_coverage_min"], "")
        self.assertEqual(result["conditional_unsupported_folds"], 0)

    def test_difference_signs_count_only_defined_fold_values(self):
        ref = reference_stub()
        folds = fold_stubs(ref)
        for r, v in zip(folds, [-0.2, 0.0, 0.1]):
            r["residual_case_weighted_risk_delta"] = v
        result = analysis.stability([ref], folds)[0]
        for sign in ["higher", "equal", "lower"]:
            self.assertEqual(
                result["residual_case_weighted_risk_delta_" + sign + "_folds"], 1
            )
        self.assertAlmostEqual(result["residual_case_weighted_risk_delta_span"], 0.3)

    def test_missing_duplicate_and_mislabeled_folds_are_rejected(self):
        ref = reference_stub()
        folds = fold_stubs(ref)
        for rr in [
            folds[:-1],
            [*folds, folds[0]],
            [dict(folds[0], fold_id="omit_999"), *folds[1:]],
        ]:
            with self.subTest(rows=len(rr)), self.assertRaises(AssertionError):
                analysis.stability([ref], rr)

    def test_existing_new_population_cannot_shrink_between_refits(self):
        ref = reference_stub()
        folds = fold_stubs(ref)
        folds[0]["records"] = 1
        with self.assertRaises(AssertionError):
            analysis.stability([ref], folds)

    def test_missing_pooled_parent_is_detected_without_switching_fold(self):
        rr = [
            row(1, 269),
            row(2, 270, stratum="recovery_single", view="sim_raw_recovery"),
            row(3, 271, stratum="recovery_single", view="sim_raw_recovery"),
        ]
        with self.assertRaisesRegex(AssertionError, "no training population"):
            analysis.split_fold(rr, 269)

    def test_bad_seed_membership_and_nonfinite_ranges_are_rejected(self):
        with self.assertRaises(AssertionError):
            analysis.fold_info(268)
        with self.assertRaises(AssertionError):
            analysis.role(row(1, 274, partition="known"), 269)
        with self.assertRaises(AssertionError):
            analysis.range_fields("metric", [0.0, float("nan"), 1.0])

    def test_full_reference_check_requires_exact_saved_values(self):
        analysis.assert_csv_equal(
            [dict(value=1.0, text="same")], [dict(value="1.0", text="same")]
        )
        with self.assertRaises(AssertionError):
            analysis.assert_csv_equal([dict(value=1.0)], [dict(value="2.0")])


if __name__ == "__main__":
    unittest.main()
