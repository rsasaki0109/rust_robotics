"""Frozen time boundaries, exact mass conservation and unchanged case weights."""

from collections import defaultdict
from copy import deepcopy
from fractions import Fraction as F
import math
import unittest

import analyze_event_temporal_rankings as analysis
from test_planner_window_sensitivity import account, row


def fixture(second_case=False):
    spec = [
        (0, 2, 0.2),
        (0.02, 1, 0.05),
        (0.1, 2, 0.05),
        (0.5, 3, 0.05),
        (0.6, 0, 0.2),
        (0.62, 0, 0.05),
    ]
    rr = [row(time=150 + t, tick=tick, error=e) for t, tick, e in spec]
    if second_case:
        rr.extend(
            [
                row(planner="JPS", time=150 + i * 0.02, tick=10, error=e)
                for i, e in enumerate([0.2, 0.05])
            ]
        )
    for i, r in enumerate(rr):
        r.update(source_id=str(i), _event_id="1", source_event_ids="1")
    _, _, lookup = analysis.window.audit_units(account(rr), rr)
    attached, audit = analysis.attach_times(rr, lookup)
    return rr, attached, lookup, audit


def decompose(rows, lookup, weights=None, label="observed", cache=None):
    ww = (
        [lookup[analysis.previous.identity(r)]["case_row_weight"] for r in rows]
        if weights is None
        else weights
    )
    return analysis.temporal_decomposition(
        rows, ww, lookup, "residual", "residual_sigma", label, cache
    )


class EventTemporalTests(unittest.TestCase):
    def test_frozen_inclusive_integer_endpoints(self):
        for gap, expected in [
            (0, 0),
            (20_000, 0),
            (20_001, 1),
            (100_000, 1),
            (100_001, 2),
            (500_000, 2),
            (500_001, 3),
        ]:
            self.assertEqual(analysis.time_bin(gap), expected)
        for gap in [-1, 0.5, math.nan]:
            with self.assertRaises(AssertionError):
                analysis.time_bin(gap)

    def test_float_boundary_noise_is_removed_without_shifting_endpoints(self):
        start = 175.16
        for gap in [0.02, 0.1, 0.5]:
            for noise in [-1e-11, 0, 1e-11]:
                actual = analysis.microseconds(
                    start + gap + noise
                ) - analysis.microseconds(start)
                self.assertEqual(actual, round(gap * 1_000_000))
        self.assertEqual(analysis.microseconds("175.180001"), 175_180_001)

    def test_nonfinite_and_off_grid_times_fail(self):
        for value in [math.nan, math.inf, -math.inf, 150.0000002, 150.0000005]:
            with self.assertRaises(AssertionError):
                analysis.microseconds(value)

    def test_hand_calculated_four_bins_cumulative_auc_and_support(self):
        _, rr, lookup, _ = fixture()
        extra, bins, total = decompose(rr, lookup)
        self.assertEqual([b["pair_count"] for b in bins], [2, 2, 2, 2])
        self.assertEqual([b["denominator"] for b in bins], [F(1, 18)] * 4)
        self.assertEqual(
            [b["numerator"] for b in bins], [F(1, 24), F(1, 72), 0, F(1, 36)]
        )
        self.assertEqual([b["auc"] for b in bins], [F(3, 4), F(1, 4), 0, F(1, 2)])
        self.assertEqual(
            [extra[s + "_auc"] for s in analysis.SCOPES], [F(3, 4), F(1, 2), F(1, 3)]
        )
        self.assertEqual(
            [extra[s + "_event_pair_share"] for s in analysis.SCOPES],
            [F(1, 4), F(1, 2), F(3, 4)],
        )
        self.assertEqual(total["numerator"] / total["denominator"], F(3, 8))
        self.assertEqual(extra["same_event_pair_count"], 8)
        for b in bins:
            self.assertEqual(b["events_with_pairs"], 1)
            self.assertEqual(b["cases_with_pairs"], 1)
            self.assertEqual(b["event_pair_share"], F(1, 4))
            self.assertEqual(b["full_pair_share"], F(1, 4))

    def test_unequal_case_weights_use_pair_masses(self):
        _, rr, lookup, _ = fixture(True)
        extra, bins, total = decompose(rr, lookup)
        self.assertEqual(bins[0]["denominator"], F(11, 36))
        self.assertEqual(bins[0]["numerator"], F(1, 6))
        self.assertEqual(extra["le_20ms_auc"], F(6, 11))
        self.assertEqual(extra["le_20ms_pair_count"], 3)
        self.assertEqual(extra["le_20ms_events_with_pairs"], 2)
        self.assertEqual(extra["le_20ms_cases_with_pairs"], 2)
        self.assertEqual(total["numerator"] / total["denominator"], F(15, 34))
        self.assertNotEqual(extra["le_20ms_auc"], (F(3, 4) + F(1, 2)) / 2)

    def test_zero_time_gap_between_distinct_states_keeps_half_ties(self):
        _, rr, lookup, _ = fixture(True)
        subset = [dict(r, _time_us=150_000_000) for r in rr[-2:]]
        extra, bins, _ = decompose(subset, lookup)
        self.assertEqual(bins[0]["pair_count"], 1)
        self.assertEqual(bins[0]["tied_pair_mass"], F(1, 4))
        self.assertEqual(extra["le_20ms_auc"], F(1, 2))

    def test_absolute_time_gap_counts_both_chronological_directions(self):
        _, rr, lookup, _ = fixture()
        reflected = [dict(r, _time_us=300_000_000 - r["_time_us"]) for r in rr]
        self.assertEqual(decompose(rr, lookup), decompose(reflected, lookup))
        self.assertEqual(decompose(rr, lookup), decompose(list(reversed(rr)), lookup))

    def test_same_tag_and_time_do_not_merge_different_cases(self):
        _, rr, lookup, _ = fixture(True)
        before = decompose(rr, lookup)
        renamed = [dict(r, _event_id="99") if r["planner"] == "JPS" else r for r in rr]
        self.assertEqual(before, decompose(renamed, lookup))

    def test_nearby_states_in_different_events_are_excluded(self):
        _, rr, lookup, _ = fixture()
        changed = [dict(r, _event_id="2") if r is rr[1] else r for r in rr]
        extra, bins, total = decompose(changed, lookup)
        self.assertEqual(bins[0]["pair_count"], 1)
        self.assertEqual(bins[0]["auc"], F(1, 2))
        self.assertEqual(extra["same_event_pair_count"], 6)
        self.assertEqual(total["denominator"], F(1, 6))

    def test_stratum_selection_preserves_parent_weight(self):
        _, rr, lookup, _ = fixture()
        rr[0] = dict(rr[0], stratum="fusion_three")
        selected = [r for r in rr if r["stratum"] == "filter_single"]
        _, bins, _ = decompose(selected, lookup)
        self.assertEqual([b["denominator"] for b in bins], [F(1, 36)] * 4)
        self.assertNotEqual(bins[0]["denominator"], F(1, 25))

    def test_invalid_or_event_normalized_weights_fail(self):
        _, rr, lookup, _ = fixture(True)
        for weights in [[], [F(0)] * len(rr), [F(1)] * len(rr), [F(1, 8)] * len(rr)]:
            with self.assertRaises(AssertionError):
                decompose(rr, lookup, weights=weights)

    def test_missing_classes_and_empty_groups_keep_undefined_auc(self):
        _, rr, lookup, _ = fixture()
        for subset, label in [([], "observed"), (rr, "unmodeled")]:
            extra, bins, total = decompose(subset, lookup, label=label)
            self.assertEqual(total["denominator"], 0)
            for b in bins:
                self.assertIsNone(b["auc"])
                self.assertIsNone(b["event_pair_share"])
                self.assertIsNone(b["full_pair_share"])
                self.assertEqual(b["pair_count"], 0)
            self.assertTrue(all(extra[s + "_auc"] is None for s in analysis.SCOPES))

    def test_distant_classes_keep_zero_nearby_share_but_blank_auc(self):
        _, rr, lookup, _ = fixture()
        subset = [rr[0], rr[-1]]
        extra, bins, _ = decompose(subset, lookup)
        for scope in analysis.SCOPES:
            self.assertIsNone(extra[scope + "_auc"])
            self.assertEqual(extra[scope + "_event_pair_share"], 0)
            self.assertEqual(extra[scope + "_full_pair_share"], 0)
        self.assertEqual(bins[-1]["auc"], 1)
        self.assertEqual(bins[-1]["event_pair_share"], 1)

    def test_all_time_bins_reconstruct_the_original_event_pair_masses(self):
        _, rr, lookup, _ = fixture(True)
        weights = [lookup[analysis.previous.identity(r)]["case_row_weight"] for r in rr]
        _, old, _ = analysis.prior.event_decomposition(
            rr, weights, lookup, "residual", "residual_sigma", "observed"
        )
        _, bins, total = decompose(rr, lookup)
        for field in analysis.FIELDS:
            self.assertEqual(sum(b[field] for b in bins), old[0][field])
            self.assertEqual(total[field], old[0][field])

    def test_time_audit_does_not_mutate_rows_and_rejects_duplicate_keys(self):
        original, attached, lookup, audit = fixture()
        self.assertTrue(all("_time_us" not in r for r in original))
        self.assertEqual(audit[1]["time_us"], 150_020_000)
        self.assertLessEqual(max(abs(r["reconstruction_error_s"]) for r in audit), 1e-7)
        other = dict(original[0], view="sim_raw_preview")
        rows, second = analysis.attach_times([*original, other], lookup)
        self.assertEqual(second, audit)
        self.assertIs(rows[-1], other)
        changed = deepcopy(original)
        changed[1]["source_id"] = changed[0]["source_id"]
        with self.assertRaises(AssertionError):
            analysis.attach_times(changed, lookup)

    def test_cache_separates_state_subsets_and_labels(self):
        _, rr, lookup, _ = fixture()
        cache = {}
        for subset, label in [
            (rr, "observed"),
            (rr[1:], "observed"),
            (rr, "modeled"),
            (rr, "unmodeled"),
        ]:
            self.assertEqual(
                decompose(subset, lookup, label=label, cache=cache),
                decompose(subset, lookup, label=label),
            )

    def test_event_support_counts_retain_class_mass_and_fixed_bins(self):
        _, rr, lookup, _ = fixture()
        support = analysis.event_time_support(rr, lookup, defaultdict(set))
        self.assertEqual(len(support), 12)
        observed = next(
            r
            for r in support
            if r["stratum"] == "filter_single"
            and r["target"] == "residual"
            and r["label"] == "observed"
        )
        self.assertEqual(observed["outside_rows"], 2)
        self.assertEqual(observed["inside_rows"], 4)
        self.assertEqual(observed["outside_case_mass"], float(F(1, 3)))
        self.assertEqual(observed["inside_case_mass"], float(F(2, 3)))
        self.assertEqual([observed[b + "_pair_count"] for b in analysis.BINS], [2] * 4)
        self.assertEqual(
            [observed[b + "_denominator"] for b in analysis.BINS], [float(F(1, 18))] * 4
        )
        self.assertFalse(observed["common_family_eligible"])


if __name__ == "__main__":
    unittest.main()
