"""Saved event identity, exact pair masses and inherited case weighting."""

from collections import defaultdict
from copy import deepcopy
from fractions import Fraction as F
import unittest

import analyze_event_conditioned_rankings as analysis
from test_planner_window_sensitivity import account, row


def fixture():
    rr = [
        row(time=150 + i * 0.02, tick=tick, error=error)
        for i, (tick, error) in enumerate(
            [(2, 0.2), (3, 0.2), (1, 0.05), (0, 0.2), (2, 0.05)]
        )
    ]
    rr.append(row(planner="JPS", tick=0, error=0.05))
    for i, r in enumerate(rr, 1):
        r["source_id"] = str(i)
        r["source_event_ids"] = "1" if i <= 3 else "2" if i <= 5 else "3"
    planned = [row(planner=p) for p in analysis.parent.PLANNERS]
    families, _, lookup = analysis.window.audit_units(account(planned), rr)
    events = {}
    for eid, source in [("1", rr[0]), ("2", rr[3]), ("3", rr[5]), ("4", planned[2])]:
        events[eid] = dict(
            source,
            event_id=eid,
            source_event_id="42",
            event_kind="received" if eid != "4" else "missed",
            event_time_s="150.0",
            truth_time_s="149.98",
            scheduled="True",
            received="True" if eid != "4" else "False",
            precision_activation="False",
        )
    attached, mapping = analysis.attach_events(rr, lookup, events)
    return rr, attached, families, lookup, events, mapping


def decompose(rr, lookup, label="observed", weights=None):
    ww = (
        [lookup[analysis.previous.identity(r)]["case_row_weight"] for r in rr]
        if weights is None
        else weights
    )
    return analysis.event_decomposition(
        rr, ww, lookup, "residual", "residual_sigma", label
    )


class EventConditionedTests(unittest.TestCase):
    def test_hand_calculated_three_bin_masses_and_conditional_shares(self):
        _, rr, _, lookup, _, _ = fixture()
        extra, bins, same_case = decompose(rr, lookup)
        self.assertEqual(
            [b["denominator"] for b in bins], [F(3, 25), F(3, 25), F(15, 25)]
        )
        self.assertEqual([b["numerator"] for b in bins], [F(2, 25), F(3, 50), F(1, 2)])
        self.assertEqual([b["auc"] for b in bins], [F(2, 3), F(1, 2), F(5, 6)])
        self.assertEqual(
            [b["denominator_share"] for b in bins], [F(1, 7), F(1, 7), F(5, 7)]
        )
        self.assertEqual(
            [b["within_case_share"] for b in bins], [F(1, 2), F(1, 2), None]
        )
        self.assertEqual(sum(b["numerator_contribution"] for b in bins), F(16, 21))
        self.assertEqual(same_case["numerator"] / same_case["denominator"], F(7, 12))
        self.assertEqual(extra["events"], 3)
        self.assertEqual(extra["events_with_both_classes"], 2)
        self.assertEqual(extra["cases_with_event_support"], 1)

    def test_event_auc_sums_pair_mass_instead_of_averaging_events(self):
        _, rr, _, lookup, events, _ = fixture()
        _, _, ranks = analysis.event_tables(rr, lookup, events, defaultdict(set))
        selected = [
            r
            for r in ranks
            if r["stratum"] == "filter_single"
            and r["target"] == "residual"
            and r["score"] == "residual_sigma"
            and r["label"] == "observed"
            and r["auc"] != ""
        ]
        self.assertEqual(sorted(r["auc"] for r in selected), [0, 1])
        self.assertEqual(decompose(rr, lookup)[0]["same_event_auc"], F(2, 3))

    def test_ambiguous_empty_unknown_and_other_case_tags_are_rejected(self):
        original, _, _, _, events, _ = fixture()
        for tag in ["", "1+2", "1+1", "999", "3"]:
            r = dict(original[0], source_event_ids=tag)
            with self.assertRaises(AssertionError):
                analysis.event_id(r, events)

    def test_event_partition_alias_and_reporting_group_checks(self):
        original, _, _, _, events, _ = fixture()
        r = dict(original[0], partition="new")
        ee = deepcopy(events)
        ee["1"]["partition"] = "existing_new"
        self.assertEqual(analysis.event_id(r, ee), "1")
        for bad in [
            dict(r, noise="correlated"),
            dict(r, partition="known"),
            dict(r, planner="Theta*"),
        ]:
            with self.assertRaises(AssertionError):
                analysis.event_id(bad, ee)

    def test_same_event_tag_is_still_scoped_to_the_full_case(self):
        _, rr, _, lookup, _, _ = fixture()
        changed = [dict(r, _event_id="1") if r["planner"] == "JPS" else r for r in rr]
        before, bins, _ = decompose(rr, lookup)
        after, other, _ = decompose(changed, lookup)
        self.assertEqual(before, after)
        self.assertEqual(bins, other)

    def test_equal_timestamps_and_underlying_ids_do_not_merge_events(self):
        _, rr, _, lookup, events, _ = fixture()
        self.assertEqual(len({e["event_time_s"] for e in events.values()}), 1)
        self.assertEqual(len({e["source_event_id"] for e in events.values()}), 1)
        extra, bins, _ = decompose(rr, lookup)
        self.assertEqual(extra["events"], 3)
        self.assertGreater(bins[1]["denominator"], 0)

    def test_mode_selection_preserves_original_case_weights(self):
        _, rr, _, lookup, events, _ = fixture()
        rr[0] = dict(rr[0], stratum="fusion_three")
        ss = [r for r in rr if r["stratum"] == "filter_single"]
        extra, bins, _ = decompose(ss, lookup)
        coverage, support, ranks = analysis.event_tables(
            rr, lookup, events, defaultdict(set)
        )
        item = next(
            r
            for r in support
            if r["event_id"] == "1" and r["stratum"] == "filter_single"
        )
        self.assertEqual(item["records"], 2)
        self.assertEqual(item["parent_event_records"], 3)
        self.assertEqual(item["parent_case_records"], 5)
        self.assertEqual(item["case_row_weight"], 0.2)
        self.assertEqual(item["case_mass"], 0.4)
        self.assertEqual(bins[0]["denominator"], F(2, 25))

    def test_event_one_class_can_coexist_with_supported_case_auc(self):
        original, _, _, lookup, events, _ = fixture()
        rr = [
            dict(original[0], source_event_ids="1"),
            dict(original[2], source_event_ids="2"),
        ]
        attached, _ = analysis.attach_events(rr, lookup, events)
        extra, bins, same = decompose(attached, lookup)
        self.assertIsNone(extra["same_event_auc"])
        self.assertEqual(extra["same_event_pair_share"], 0)
        self.assertEqual(extra["same_event_within_case_share"], 0)
        self.assertEqual(extra["other_event_same_case_auc"], 1)
        self.assertEqual(extra["events_with_both_classes"], 0)
        self.assertEqual(same["numerator"] / same["denominator"], 1)

    def test_empty_and_unmodeled_one_class_cells_keep_blank_auc(self):
        _, rr, _, lookup, _, _ = fixture()
        for rows, label in [([], "observed"), (rr, "unmodeled")]:
            extra, bins, same = decompose(rows, lookup, label)
            self.assertTrue(
                all(
                    b["auc"] is None
                    and b["denominator_share"] is None
                    and b["within_case_share"] is None
                    for b in bins
                )
            )
            self.assertEqual(extra["events_with_both_classes"], 0)
            self.assertEqual(same["denominator"], 0)

    def test_invalid_or_event_normalized_weights_fail(self):
        _, rr, _, lookup, _, _ = fixture()
        for ww in [
            [],
            [F(0)] * len(rr),
            [F(1)] * len(rr),
            [F(1, 3)] * 3 + [F(1, 2)] * 2 + [F(1)],
        ]:
            with self.assertRaises(AssertionError):
                decompose(rr, lookup, weights=ww)

    def test_full_event_census_and_empty_strata_are_not_imputed(self):
        _, rr, _, lookup, events, _ = fixture()
        events["3"].update(
            event_kind="precision_only", received="False", precision_activation="True"
        )
        coverage, support, ranks = analysis.event_tables(
            rr, lookup, events, defaultdict(set)
        )
        self.assertEqual(len(coverage), 4)
        missing = next(r for r in coverage if r["event_id"] == "4")
        self.assertEqual(missing["records"], 0)
        self.assertEqual(missing["case_row_weight"], "")
        self.assertEqual(missing["min_time_s"], "")
        self.assertEqual(missing["received"], "False")
        self.assertEqual(len(support), 15)
        self.assertEqual(sum(r["records"] > 0 for r in support), 6)
        self.assertEqual(len(ranks), 108)
        self.assertEqual({r["event_id"] for r in ranks}, {"1", "2", "3"})
        self.assertTrue(any(r["event_kind"] == "precision_only" for r in ranks))

    def test_mapping_is_unique_does_not_mutate_rows_and_skips_other_views(self):
        original, _, _, lookup, events, mapping = fixture()
        self.assertTrue(all("_event_id" not in r for r in original))
        raw = dict(original[0], view="sim_raw_preview", source_event_ids="unknown")
        attached, second = analysis.attach_events([*original, raw], lookup, events)
        self.assertEqual(second, mapping)
        self.assertIs(attached[-1], raw)
        self.assertEqual(len(mapping), 6)
        changed = deepcopy(original)
        changed[1]["source_id"] = changed[0]["source_id"]
        with self.assertRaises(AssertionError):
            analysis.attach_events(changed, lookup, events)

    def test_event_bins_reconstruct_existing_case_and_full_pair_masses(self):
        _, rr, _, lookup, _, _ = fixture()
        ww = [lookup[analysis.previous.identity(r)]["case_row_weight"] for r in rr]
        _, old, _, full = analysis.prior.decompose(
            rr, ww, lookup, "residual", "residual_sigma", "observed"
        )
        _, bins, same = decompose(rr, lookup)
        for field in analysis.FIELDS:
            self.assertEqual(same[field], old[0][field])
            self.assertEqual(sum(b[field] for b in bins), full[field])


if __name__ == "__main__":
    unittest.main()
