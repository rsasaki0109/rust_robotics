"""Score precision, all-tie conditional support and exact original weights."""

from collections import defaultdict
from copy import deepcopy
from fractions import Fraction as F
import math
import unittest

import analyze_event_score_ties as analysis
from test_planner_window_sensitivity import account, row


def fixture(spec=None, planner="A*", historical=False):
    spec = spec or [
        (0, 2, 2e-8, 0.2),
        (0.02, 1, 1e-8, 0.05),
        (0.04, 3, 3e-8, 0.05),
        (0.06, 2, 1.9e-8, 0.05),
        (0.08, 2, 2e-8, 0.05),
        (0.1, 2, 2.1e-8, 0.05),
    ]
    rows = [
        row(planner=planner, time=150 + t, tick=tick, error=error)
        for t, tick, native, error in spec
    ]
    for i, (r, (_, tick, native, _)) in enumerate(zip(rows, spec)):
        r.update(
            source_id=str(i),
            _event_id="1",
            _saved_scores={s: native for s in analysis.previous.SCORES},
        )
        if historical:
            r.update(cohort="known-white", partition="known", seed="269")
    _, _, lookup = analysis.window.audit_units(account(rows), rows)
    attached, _ = analysis.prior.attach_times(rows, lookup)
    return attached, lookup


def scores(rows):
    result = []
    for r in rows:
        s = {
            k: str(r[k])
            for k in [
                *analysis.window.KEY,
                "source_id",
                "partition",
                "noise",
                "view",
                "source_event_ids",
                "stratum",
                "a_estimator",
                "b_estimator",
                "anchor_mode",
            ]
        }
        s["source_family"] = "simultaneous"
        for score in analysis.previous.SCORES:
            s[score + "_score_m"] = repr(r["_saved_scores"][score])
            s[score + "_tick"] = str(r[score + "_tick"])
        result.append(s)
    return result


def decompose(rows, lookup, label="observed", weights=None, cache=None):
    ww = (
        [lookup[analysis.previous.identity(r)]["case_row_weight"] for r in rows]
        if weights is None
        else weights
    )
    return analysis.decomposition(
        rows, ww, lookup, "residual", "residual_sigma", label, cache
    )


class EventScoreTieTests(unittest.TestCase):
    def test_five_hand_calculated_score_order_classes(self):
        rr, lookup = fixture()
        scopes, bins = decompose(rr, lookup)
        total = scopes[0]
        self.assertEqual([total[c + "_pair_count"] for c in analysis.CLASSES], [1] * 5)
        self.assertEqual(
            [total[f] for f in analysis.FIELDS],
            [F(1, 36), F(3, 36), F(1, 36), F(5, 72), F(5, 36)],
        )
        self.assertEqual(total["auc"], F(1, 2))
        self.assertEqual(total["native_auc"], F(1, 2))
        self.assertEqual(total["tick_tied_share"], F(3, 5))
        self.assertEqual(total["native_tied_share"], F(1, 5))
        self.assertEqual(total["resolved_tick_tie_share"], F(2, 5))
        self.assertEqual(total["tick_unequal_pair_count"], 2)
        self.assertEqual(total["native_unequal_pair_count"], 4)
        self.assertEqual(total["tick_unequal_concordance"], F(1, 2))
        self.assertEqual(total["native_unequal_concordance"], F(1, 2))
        self.assertEqual([b["pair_count"] for b in bins], [1, 4, 0, 0])

    def test_rounding_tie_resolution_moves_auc_with_fixed_sign_and_denominator(self):
        for native, expected in [(2.1e-8, 1), (1.9e-8, 0)]:
            rr, lookup = fixture([(0, 2, native, 0.2), (0.02, 2, 2e-8, 0.05)])
            r = decompose(rr, lookup)[0][0]
            self.assertEqual(r["auc"], F(1, 2))
            self.assertEqual(r["native_auc"], expected)
            self.assertEqual(r["native_auc_delta"], expected - F(1, 2))
            self.assertEqual(r["denominator"], F(1, 4))
            self.assertIsNone(r["tick_unequal_concordance"])
            self.assertEqual(r["native_unequal_events"], 1)
            self.assertEqual(r["tick_unequal_events"], 0)

    def test_all_ties_and_balanced_wins_losses_have_different_conditional_support(self):
        rr, lookup = fixture()
        mixed = decompose(rr, lookup)[0][0]
        equal = [
            dict(
                r,
                **{s + "_tick": 2 for s in analysis.previous.SCORES},
                _saved_scores={s: 2e-8 for s in analysis.previous.SCORES}
            )
            for r in rr
        ]
        tied = decompose(equal, lookup)[0][0]
        self.assertEqual(mixed["auc"], tied["auc"])
        self.assertEqual(tied["tick_tied_share"], 1)
        self.assertIsNone(tied["tick_unequal_concordance"])
        self.assertIsNone(tied["native_unequal_concordance"])
        self.assertEqual(tied["tick_unequal_cases"], 0)
        self.assertEqual(mixed["tick_unequal_cases"], 1)

    def test_strict_saved_order_uses_no_tolerance(self):
        base = 2e-8
        self.assertEqual(
            analysis.order_class(2, 2, math.nextafter(base, math.inf), base), 2
        )
        self.assertEqual(
            analysis.order_class(2, 2, math.nextafter(base, -math.inf), base), 4
        )
        self.assertEqual(analysis.order_class(2, 2, base, base), 3)
        for args in [(2, 1, base, base), (1, 2, base, base), (2, 2, math.nan, base)]:
            with self.assertRaises(AssertionError):
                analysis.order_class(*args)

    def test_score_audit_preserves_strings_hex_ticks_and_does_not_mutate(self):
        for historical in [False, True]:
            rr, lookup = fixture(historical=historical)
            raw = scores(rr)
            raw[0]["contrast_score_m"] = "2.0000000000000000e-8"
            original = [
                {k: v for k, v in r.items() if k != "_saved_scores"} for r in rr
            ]
            attached, audit = analysis.attach_scores(
                original, lookup, raw if historical else [], [] if historical else raw
            )
            self.assertTrue(all("_saved_scores" not in r for r in original))
            self.assertEqual(audit[0]["contrast_score_m"], raw[0]["contrast_score_m"])
            self.assertEqual(
                audit[0]["contrast_score_hex"], float(raw[0]["contrast_score_m"]).hex()
            )
            self.assertEqual(
                audit[0]["score_source"], "historical" if historical else "latest"
            )
            self.assertEqual(attached[0]["_saved_scores"]["contrast"], 2e-8)

    def test_score_metadata_and_rounding_mismatches_fail(self):
        rr, lookup = fixture()
        for field, value in [
            ("noise", "correlated"),
            ("source_event_ids", "9"),
            ("stratum", "fusion_three"),
            ("contrast_tick", "99"),
            ("contrast_score_m", "1.5"),
            ("contrast_score_m", "nan"),
            ("contrast_score_m", "inf"),
        ]:
            raw = scores(rr)
            raw[0][field] = value
            with self.assertRaises(AssertionError):
                analysis.attach_scores(rr, lookup, [], raw)

    def test_duplicate_missing_unused_and_wrong_case_score_keys_fail(self):
        rr, lookup = fixture()
        raw = scores(rr)
        extra = dict(raw[0], source_id="extra")
        wrong = deepcopy(raw)
        wrong[0]["planner"] = "JPS"
        for source in [[*raw, raw[0]], raw[1:], [*raw, extra], wrong]:
            with self.assertRaises(AssertionError):
                analysis.attach_scores(rr, lookup, [], source)

    def test_other_views_do_not_enter_score_audit(self):
        rr, lookup = fixture()
        raw = scores(rr)
        other = dict(rr[0], view="sim_raw_preview")
        attached, audit = analysis.attach_scores([*rr, other], lookup, [], raw)
        self.assertEqual(len(audit), len(rr))
        self.assertIs(attached[-1], other)

    def test_different_events_are_not_merged_by_equal_saved_scores(self):
        rr, lookup = fixture()
        changed = [dict(r, _event_id="2") if r is rr[1] else r for r in rr]
        original = decompose(rr, lookup)[0][0]
        new = decompose(changed, lookup)[0][0]
        self.assertEqual(new["pair_count"], original["pair_count"] - 1)
        self.assertEqual(new["concordant_pair_mass"], 0)
        self.assertEqual(new["events_with_pairs"], 1)

    def test_inherited_unequal_case_weights_and_case_isolation(self):
        a, _ = fixture()
        b, _ = fixture([(0, 2, 2e-8, 0.2), (0.02, 2, 2e-8, 0.05)], planner="JPS")
        rr = [*a, *b]
        _, _, lookup = analysis.window.audit_units(account(rr), rr)
        total = decompose(rr, lookup)[0][0]
        self.assertEqual(total["denominator"], F(5, 36) + F(1, 4))
        self.assertEqual(total["tick_tied_share"], F(6, 7))
        self.assertEqual(total["tick_unequal_pair_count"], 2)
        self.assertEqual(total["tick_unequal_cases"], 1)
        self.assertEqual(total["cases_with_pairs"], 2)
        self.assertEqual(total["tick_unequal_concordance"], F(1, 2))
        self.assertEqual(total["events_with_pairs"], 2)

    def test_original_time_bins_and_cumulative_masses_are_preserved(self):
        rr, lookup = fixture(
            [
                (0, 2, 2e-8, 0.2),
                (0.02, 1, 1e-8, 0.05),
                (0.1, 2, 1.9e-8, 0.05),
                (0.5, 2, 2e-8, 0.05),
                (0.6, 2, 2.1e-8, 0.05),
            ]
        )
        scopes, bins = decompose(rr, lookup)
        weights = [lookup[analysis.previous.identity(r)]["case_row_weight"] for r in rr]
        old, before, _ = analysis.prior.temporal_decomposition(
            rr, weights, lookup, "residual", "residual_sigma", "observed"
        )
        self.assertEqual([b["pair_count"] for b in bins], [1] * 4)
        for field in analysis.FIELDS:
            self.assertEqual(sum(b[field] for b in bins), scopes[0][field])
            self.assertEqual([b[field] for b in bins], [b[field] for b in before])
            for name, scope in zip(analysis.SCOPES[1:], scopes[1:]):
                self.assertEqual(scope[field], old[name + "_" + field])
        self.assertEqual(scopes[0]["tick_unequal_events"], 1)

    def test_empty_and_single_class_groups_keep_blank_metrics(self):
        rr, lookup = fixture()
        for subset, label in [([], "observed"), (rr, "unmodeled")]:
            scopes, bins = decompose(subset, lookup, label=label)
            for r in [*scopes, *bins]:
                for field in [
                    "auc",
                    "native_auc",
                    "native_auc_delta",
                    "tick_tied_share",
                    "tick_unequal_concordance",
                    "native_unequal_concordance",
                ]:
                    self.assertIsNone(r[field])
                self.assertEqual(r["pair_count"], 0)

    def test_invalid_weights_fail(self):
        rr, lookup = fixture()
        for weights in [[], [F(0)] * len(rr), [F(1)] * len(rr)]:
            with self.assertRaises(AssertionError):
                decompose(rr, lookup, weights=weights)

    def test_summary_baseline_regression_including_empty_scope_fields(self):
        rr, lookup = fixture()
        for mode in ["filter_single", "fusion_three"]:
            selected = [r for r in rr if r["stratum"] == mode]
            ww = [
                lookup[analysis.previous.identity(r)]["case_row_weight"]
                for r in selected
            ]
            event, _, _ = analysis.prior.prior.event_decomposition(
                selected, ww, lookup, "residual", "residual_sigma", "observed"
            )
            temporal, pieces, _ = analysis.prior.temporal_decomposition(
                selected, ww, lookup, "residual", "residual_sigma", "observed"
            )
            metadata = dict(
                partition="fresh",
                noise="white",
                variant="epoch",
                stratum=mode,
                target="residual",
                population="source",
                planner_selection="all",
                score="residual_sigma",
                label="observed",
            )
            reference = analysis.previous.historical.exported(
                dict(**metadata, **event, **temporal)
            )
            old_bins = [
                analysis.previous.historical.exported(dict(**metadata, **b))
                for b in pieces
            ]
            summary, bins, checks = analysis.summaries(
                rr, lookup, [reference], old_bins, defaultdict(set)
            )
            self.assertEqual((len(summary), len(bins)), (4, 4))
            self.assertGreater(checks, 80)
            self.assertEqual(summary[0]["auc"], 0.5 if selected else "")

    def test_cache_separates_saved_values_and_conditional_subsets(self):
        rr, lookup = fixture()
        cache = {}
        decompose(rr, lookup, cache=cache)
        changed = [
            (
                dict(r, _saved_scores={s: 2e-8 for s in analysis.previous.SCORES})
                if r["residual_sigma_tick"] == 2
                else r
            )
            for r in rr
        ]
        for subset in [rr[1:], changed]:
            self.assertEqual(
                decompose(subset, lookup, cache=cache), decompose(subset, lookup)
            )


if __name__ == "__main__":
    unittest.main()
