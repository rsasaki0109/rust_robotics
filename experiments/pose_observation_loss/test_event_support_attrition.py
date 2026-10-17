"""Hand-calculated support attrition and frozen-source boundary cases."""

import copy
from fractions import Fraction
import unittest

import analyze_event_support_attrition as a


def fixture(
    events=(), parent=None, eligible=True, mode="random", case="C1", planner="A*"
):
    n = sum(out + ins for out, ins, bins in events)
    parent = n if parent is None else parent
    raw = dict(
        case_id=case,
        family_id="F1",
        cohort="fresh-white",
        variant="epoch",
        mode=mode,
        seed="277",
        scenario="maze_50x50",
        planner=planner,
        partition="fresh",
        accounting_partition="fresh",
        noise="white",
        stratum="filter_single",
        parent_records=str(parent),
        records=str(n),
        case_row_weight=str(1 / parent) if parent else "",
        case_mass=str(n / parent) if parent else "0.0",
        common_family_eligible=str(eligible),
    )
    for target in a.TARGETS:
        for label in a.LABELS:
            raw[f"{target}_{label}_outside_rows"] = str(sum(e[0] for e in events))
            raw[f"{target}_{label}_inside_rows"] = str(sum(e[1] for e in events))
    rows = []
    for i, (out, ins, bins) in enumerate(events):
        r = {k: raw[k] for k in a.META}
        r.update(
            event_id=str(i + 1),
            stratum=raw["stratum"],
            target="residual",
            label="observed",
            common_family_eligible=str(eligible),
            records=str(out + ins),
            outside_rows=str(out),
            inside_rows=str(ins),
        )
        for b, count in zip(a.TIME_BINS, bins):
            r[b + "_pair_count"] = str(count)
            r[b + "_denominator"] = str(count / parent**2)
        rows.append(r)
    return raw, rows


def result(events=(), **kwargs):
    population = kwargs.pop("population", "source")
    raw, ee = fixture(events, **kwargs)
    return a.case_result(raw, ee, "residual", "observed", population)


class AttritionTests(unittest.TestCase):
    def test_original_empty_event_ledger_expands_without_inventing_states(self):
        raw, _ = fixture(parent=10)
        raw["event_id"] = "2"
        complete = a.complete_events([raw], [])
        self.assertEqual(len(complete), 6)
        self.assertTrue(all(r["records"] == "0" for r in complete))
        self.assertTrue(
            all(r[b + "_pair_count"] == "0" for r in complete for b in a.TIME_BINS)
        )
        self.assertEqual(
            {(r["target"], r["label"]) for r in complete},
            set(a.product(a.TARGETS, a.LABELS)),
        )

    def test_missing_nonempty_time_support_is_not_completed_with_zero(self):
        raw, _ = fixture([(1, 1, (1, 0, 0, 0))])
        raw["event_id"] = "1"
        with self.assertRaises(AssertionError):
            a.complete_events([raw], [])

    def test_reject_unknown_observed_loss_mode(self):
        raw, _ = fixture(mode="unknown")
        with self.assertRaises(AssertionError):
            a.aggregate([raw], [])

    def test_no_parent_precedes_all_other_reasons(self):
        c, ee = result(eligible=False, population="common_family")
        self.assertFalse(ee)
        self.assertEqual([c[s + "_reason"] for s in a.SCOPES], ["no_parent_rp"] * 4)
        self.assertEqual(c["case_id"], "C1")

    def test_no_stratum_precedes_family_exclusion(self):
        c, _ = result(parent=10, eligible=False, population="common_family")
        self.assertEqual(c["same_event_reason"], "no_stratum_rp")

    def test_fixed_family_exclusion_precedes_class_check(self):
        c, ee = result(
            [(1, 1, (1, 0, 0, 0))],
            parent=10,
            eligible=False,
            population="common_family",
        )
        self.assertEqual(c["records"], 0)
        self.assertEqual(c["stratum_records"], 2)
        self.assertEqual(c["le_20ms_reason"], "family_ineligible")
        self.assertEqual(ee[0]["le_20ms_reason"], "family_ineligible")
        self.assertEqual(c["case_mass"], 0)

    def test_source_keeps_ineligible_family(self):
        c, _ = result([(1, 1, (1, 0, 0, 0))], parent=10, eligible=False)
        self.assertEqual(c["le_20ms_reason"], "supported")
        self.assertEqual(c["case_mass"], Fraction(1, 5))

    def test_no_outside(self):
        c, ee = result([(0, 3, (0, 0, 0, 0))])
        self.assertEqual(c["same_event_reason"], "no_outside")
        self.assertEqual(ee[0]["same_event_reason"], "no_outside")
        self.assertEqual(c["negative_only_events"], 1)

    def test_no_inside(self):
        c, _ = result([(3, 0, (0, 0, 0, 0))])
        self.assertEqual(c["same_event_reason"], "no_inside")
        self.assertEqual(c["positive_only_events"], 1)

    def test_opposite_classes_in_distinct_events_are_not_event_pairs(self):
        c, ee = result([(2, 0, (0, 0, 0, 0)), (0, 3, (0, 0, 0, 0))], parent=10)
        self.assertEqual(c["case_pair_count"], 6)
        self.assertEqual(c["case_pair_mass"], Fraction(3, 50))
        self.assertEqual(c["other_event_pair_count"], 6)
        self.assertEqual(c["same_event_pair_count"], 0)
        self.assertEqual(c["le_20ms_reason"], "event_separated")
        self.assertEqual(
            [e["same_event_reason"] for e in ee], ["no_inside", "no_outside"]
        )

    def test_gap_exclusion_is_not_missing_class_or_event(self):
        c, ee = result([(1, 1, (0, 0, 1, 0))])
        self.assertEqual(c["same_event_reason"], "supported")
        self.assertEqual(c["le_500ms_reason"], "supported")
        self.assertEqual(c["le_100ms_reason"], "gap_excluded")
        self.assertEqual(c["le_20ms_reason"], "gap_excluded")
        self.assertEqual(ee[0]["le_20ms_reason"], "gap_excluded")

    def test_supported_can_still_lose_most_pairs(self):
        c, _ = result([(2, 2, (1, 1, 1, 1))], parent=10)
        self.assertEqual([c[s + "_pair_count"] for s in a.SCOPES], [4, 3, 2, 1])
        self.assertEqual([c[s + "_reason"] for s in a.SCOPES], ["supported"] * 4)
        self.assertEqual(c["le_20ms_pair_mass"], Fraction(1, 100))

    def test_original_parent_weight_precedes_stratum_selection(self):
        c, _ = result([(1, 1, (1, 0, 0, 0))], parent=20)
        self.assertEqual(c["case_mass"], Fraction(1, 10))
        self.assertEqual(c["case_pair_mass"], Fraction(1, 400))

    def test_sum_masses_before_shares_not_average_case_ratios(self):
        c1, _ = result([(2, 1, (1, 1, 0, 0)), (1, 1, (0, 0, 1, 0))], parent=10)
        c2, _ = result([(1, 1, (1, 0, 0, 0))], parent=20, case="C2", planner="JPS")
        s = a.summary([c1, c2], {}, "random")
        self.assertEqual(s["case_pair_count"], 7)
        self.assertEqual(s["same_event_pair_count"], 4)
        self.assertEqual(s["same_event_over_case"], Fraction(13, 25))
        self.assertEqual(s["le_20ms_over_case"], Fraction(1, 5))
        self.assertEqual(s["le_20ms_over_event"], Fraction(5, 13))

    def test_no_opportunities_have_na_shares(self):
        c, _ = result([(0, 3, (0, 0, 0, 0))])
        s = a.summary([c], {}, "random")
        self.assertIsNone(s["same_event_over_case"])
        self.assertIsNone(s["le_20ms_over_event"])

    def test_positive_opportunity_zero_retention_is_zero_not_na(self):
        c, _ = result([(2, 0, (0, 0, 0, 0)), (0, 1, (0, 0, 0, 0))])
        s = a.summary([c], {}, "random")
        self.assertEqual(s["same_event_over_case"], 0)
        self.assertIsNone(s["le_20ms_over_event"])

    def test_loss_mode_cases_do_not_create_pairs(self):
        c1, _ = result([(2, 0, (0, 0, 0, 0))], mode="random")
        c2, _ = result([(0, 2, (0, 0, 0, 0))], mode="blackout", case="C2")
        s = a.summary([c1, c2], {}, "all_modes")
        self.assertEqual(s["case_pair_count"], 0)
        self.assertEqual(s["same_event_pair_count"], 0)

    def test_reason_counts_partition_every_planned_case_at_each_scope(self):
        definitions = [
            result()[0],
            result(parent=10, case="C2")[0],
            result([(1, 0, (0, 0, 0, 0))], case="C3")[0],
            result([(1, 1, (0, 0, 0, 1))], case="C4")[0],
        ]
        s = a.summary(definitions, {}, "all_modes")
        for scope in a.SCOPES:
            self.assertEqual(sum(s[scope + "_cases_" + why] for why in a.REASONS), 4)
        self.assertEqual(s["le_20ms_cases_gap_excluded"], 1)
        self.assertEqual(s["same_event_supported_cases"], 1)

    def test_input_nonmutation(self):
        raw, ee = fixture([(1, 1, (0, 1, 0, 0))], parent=10)
        original = copy.deepcopy((raw, ee))
        a.case_result(raw, ee, "residual", "observed", "source")
        a.case_result(raw, ee, "residual", "observed", "common_family")
        self.assertEqual((raw, ee), original)

    def test_reject_negative_and_noncanonical_counts(self):
        for value in ("-1", "1.0", "01", "nan", True, 1.5):
            with self.subTest(value=value), self.assertRaises(
                (AssertionError, ValueError)
            ):
                a.integer(value)

    def test_reject_inconsistent_time_bins(self):
        raw, ee = fixture([(1, 1, (0, 0, 0, 0))])
        with self.assertRaises(AssertionError):
            a.case_result(raw, ee, "residual", "observed", "source")

    def test_reject_inconsistent_inherited_mass(self):
        raw, ee = fixture([(1, 1, (1, 0, 0, 0))], parent=10)
        ee[0]["le_20ms_denominator"] = "0.25"
        with self.assertRaises(AssertionError):
            a.case_result(raw, ee, "residual", "observed", "source")

    def test_reject_duplicate_event_and_wrong_case(self):
        raw, ee = fixture([(1, 1, (1, 0, 0, 0))])
        with self.assertRaises(AssertionError):
            a.case_result(raw, ee * 2, "residual", "observed", "source")
        ee[0]["mode"] = "blackout"
        with self.assertRaises(AssertionError):
            a.case_result(raw, ee, "residual", "observed", "source")

    def test_reject_missing_event_class_records(self):
        raw, _ = fixture([(1, 1, (1, 0, 0, 0))])
        with self.assertRaises(AssertionError):
            a.case_result(raw, [], "residual", "observed", "source")

    def test_reject_duplicate_case_in_summary(self):
        c, _ = result()
        with self.assertRaises(AssertionError):
            a.summary([c, c], {}, "none")

    def test_reject_unknown_target_population_and_stratum(self):
        raw, ee = fixture()
        for target, pop in (("unknown", "source"), ("residual", "unknown")):
            with self.subTest(target=target, population=pop), self.assertRaises(
                AssertionError
            ):
                a.case_result(raw, ee, target, "observed", pop)
        raw["stratum"] = "unknown"
        with self.assertRaises(AssertionError):
            a.case_result(raw, ee, "residual", "observed", "source")

    def test_aggregate_retains_empty_modes_and_mode_closure(self):
        cases, events = [], []
        for i, mode in enumerate(a.MODES):
            raw, ee = fixture(
                [] if mode == "none" else [(1, 1, (1, 0, 0, 0))],
                case="C" + str(i),
                mode=mode,
            )
            cases.append(raw)
            for target, label in a.product(a.TARGETS, a.LABELS):
                events.extend(dict(r, target=target, label=label) for r in ee)
        cc, ev, ss = a.aggregate(cases, events)
        self.assertEqual((len(cc), len(ev), len(ss)), (36, 24, 48))
        for pop in a.POPULATIONS:
            rows = [
                r
                for r in ss
                if r["target"] == "residual"
                and r["label"] == "observed"
                and r["population"] == pop
            ]
            all_modes = next(r for r in rows if r["mode"] == "all_modes")
            for field in (
                "planned_cases",
                "records",
                "case_pair_mass",
                "same_event_pair_count",
                "le_20ms_cases_no_parent_rp",
            ):
                self.assertEqual(
                    all_modes[field],
                    sum(r[field] for r in rows if r["mode"] != "all_modes"),
                )

    def test_census_keeps_unreferenced_events_and_original_flags(self):
        raw, _ = fixture()
        case = {k: raw[k] for k in a.META}
        census = [
            dict(
                case,
                event_id="1",
                event_kind="missed",
                scheduled="True",
                received="False",
                precision_activation="False",
            )
        ]
        coverage = a.coverage([case], census, [])
        self.assertEqual(len(coverage), 1)
        self.assertEqual(coverage[0]["event_kind"], "missed")
        self.assertFalse(coverage[0]["received"])
        self.assertFalse(coverage[0]["rp_referenced"])
        self.assertEqual(coverage[0]["all_records"], 0)

    def test_reject_unknown_and_wrong_case_census_event(self):
        raw, ee = fixture([(1, 1, (1, 0, 0, 0))])
        case = {k: raw[k] for k in a.META}
        census = [
            dict(
                case,
                event_id="2",
                event_kind="received",
                scheduled="True",
                received="True",
                precision_activation="False",
            )
        ]
        with self.assertRaises(AssertionError):
            a.coverage([case], census, ee)
        census[0]["event_id"] = "1"
        census[0]["mode"] = "blackout"
        with self.assertRaises(AssertionError):
            a.coverage([case], census, ee)


class IndependentMaskTests(unittest.TestCase):
    @staticmethod
    def state(outside, time, event="1", case="C1"):
        return dict(
            case_id=case,
            event_id=event,
            time_us=time,
            labels={"residual": {"observed": outside}},
        )

    def test_inclusive_20_100_500ms_boundaries_and_next_microsecond(self):
        import verify_event_support_attrition as v

        rows = [self.state(True, 0)] + [
            self.state(False, t)
            for t in (20_000, 20_001, 100_000, 100_001, 500_000, 500_001)
        ]
        r = v.direct(rows, "residual", "observed")
        self.assertEqual([r[s + "_pair_count"] for s in v.SCOPES], [6, 5, 3, 1])
        self.assertEqual(v.microseconds("0.02000004"), 20_000)
        with self.assertRaises(AssertionError):
            v.microseconds("0.0200002")

    def test_equal_time_different_events_do_not_create_event_pairs(self):
        import verify_event_support_attrition as v

        r = v.direct(
            [self.state(True, 0), self.state(False, 0, event="2")],
            "residual",
            "observed",
        )
        self.assertEqual(r["case_pair_count"], 1)
        self.assertEqual(r["same_event_pair_count"], 0)
        self.assertEqual(r["le_20ms_pair_count"], 0)

    def test_full_case_boundary_remains_mandatory(self):
        import verify_event_support_attrition as v

        with self.assertRaises(AssertionError):
            v.direct(
                [self.state(True, 0), self.state(False, 0, case="C2")],
                "residual",
                "observed",
            )

    def test_original_closed_disk_and_strict_outside_boundary(self):
        import verify_event_support_attrition as v

        self.assertFalse(v.outside((0.1, 0)))
        self.assertFalse(v.outside((-0.1, 0)))
        self.assertTrue(v.outside((0.10000000001, 0)))


if __name__ == "__main__":
    unittest.main()
