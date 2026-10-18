"""Check exact reset joins and inherited stage/map/witness distinctions."""

from copy import deepcopy
import unittest

import analyze_reset_probes as a
import verify_reset_probes as v


def fixture(first=False, case="C1", event="1"):
    meta = dict(
        zip(
            a.META,
            (
                case,
                "F1",
                "fresh-white",
                "epoch",
                "random",
                "278",
                "S",
                "A*",
                "fresh",
                "fresh",
                "white",
            ),
        )
    )
    edge = dict(
        meta,
        edge_id=case + "@5",
        source_step="5",
        source_time_s="0.1",
        event_ids=event,
        branch="joint",
        failed_gates="nominal",
        unknown_gates="",
        nominal_positive="False",
        zero_reason="cap_zero",
        timer_before_ticks="4",
        timer_after_ticks="0",
        lost_ticks="4",
        multiplicity="3",
    )
    edge.update({"gate_" + g: "fail" if g == "nominal" else "pass" for g in a.GATES})
    d = dict(
        meta,
        event_id=event,
        time_s="0.1",
        phase="available",
        branch="joint",
        refined_failed_gates="nominal",
        refined_unknown_gates="",
        source_nominal_positive="False",
        source_nominal_gate="fail",
        source_zero_reason="cap_zero",
        lattice_index="0",
        stop_latched="False",
        speed_free_distance="0.000000000",
        speed_cap="0.000000000",
        nominal_target_speed="0.000000000",
        timer_before_ticks="4",
        timer_after_ticks="0",
        timer_action="reset",
        ready_from_timer="False",
        probe_origin="first_chord_by_exclusion" if first else "initial_unsafe",
        initial_test_state="safe" if first else "unsafe",
        initial_failure_mask="" if first else "obstacle_only",
        definite_obstacle_ids="" if first else "2;3",
        definite_wall_ids="",
        boundary_gap_lo="1.0",
        boundary_gap_hi="2.0",
        obstacle_gap_lo="0.1" if first else "-0.1",
        obstacle_gap_hi="0.2" if first else "-0.05",
        first_chord_audited=str(first),
        direct_first_test_state="unsafe" if first else "",
        direct_first_failure_mask="obstacle_only" if first else "",
        first_definite_obstacle_ids="2;3" if first else "",
        first_definite_wall_ids="",
        first_boundary_gap_lo="1.0",
        first_boundary_gap_hi="2.0",
        first_obstacle_gap_lo="-0.01",
        first_obstacle_gap_hi="-0.005",
        original_blocker_ids="2;3",
        fixed_obstacle_ids="2;3",
        point_clearance_audited="True",
        input_literal="001.2300",
    )
    d.update({"gate_" + g: edge["gate_" + g] for g in a.GATES if g != "nominal"})
    witnesses = []
    for number, oid in enumerate(("2", "3"), 1):
        witnesses.append(
            dict(
                meta,
                event_id=event,
                time_s="0.1",
                phase="available",
                branch="joint",
                obstacle_order=str(number),
                obstacle_id=oid,
                obstacle_x="3.0",
                obstacle_y="4.0",
                raw_current_gap_lo="0.1" if first else "-0.1",
                raw_current_gap_hi="0.2" if first else "-0.05",
                raw_current_state="clear" if first else "blocked",
                mean_current_gap_lo="0.1",
                mean_current_gap_hi="0.2",
                mean_current_state="clear",
            )
        )
    obstacles = [
        dict(scenario="other", x="0.0", y="0.0"),
        dict(scenario="S", x="3.000000", y="4.000000"),
        dict(scenario="S", x="3.000000", y="4.000000"),
    ]
    return edge, d, witnesses, obstacles


def both(edge, d, witnesses=None, obstacles=None):
    aa = a.join(edge, a.decision_index([d]))
    vv = v.join(edge, v.decision_index([d]))
    assert aa == vv
    if witnesses is not None:
        left = a.blocker_rows([aa], witnesses, obstacles)
        right = v.blocker_rows([vv], witnesses, obstacles)
        assert left == right
        return aa, left
    return aa


class ResetProbeJoins(unittest.TestCase):
    def test_initial_evidence_and_aggregate_bound(self):
        e, d, w, o = fixture()
        p, b = both(e, d, w, o)
        self.assertEqual(
            (p["stage"], p["origin_gap_lo"], p["origin_gap_hi"]),
            ("initial", -0.1, -0.05),
        )
        self.assertEqual(p["blocker_rows"], 2)
        self.assertTrue(all(x["raw_current_state"] == "blocked" for x in b))

    def test_safe_start_does_not_replace_direct_first_chord(self):
        e, d, w, o = fixture(first=True)
        p, b = both(e, d, w, o)
        self.assertEqual(p["stage"], "first_chord")
        self.assertLess(p["origin_gap_hi"], 0)
        self.assertTrue(all(x["raw_current_state"] == "clear" for x in b))
        for f, value in (
            ("initial_test_state", "unsafe"),
            ("first_chord_audited", "False"),
            ("direct_first_test_state", "safe"),
        ):
            for module in (a, v):
                with self.assertRaises(AssertionError):
                    module.stage(dict(d, **{f: value}))

    def test_ambiguous_origin_keeps_stage_unresolved(self):
        e, d, _, o = fixture()
        d.update(
            probe_origin="ambiguous_zero",
            initial_test_state="unknown",
            original_blocker_ids="",
            fixed_obstacle_ids="",
            point_clearance_audited="False",
        )
        p, b = both(e, d, [], o)
        self.assertEqual(p["stage"], "unresolved")
        self.assertIsNone(p["origin_gap_lo"])
        self.assertEqual(b, [])

    def test_exact_case_event_and_step_identity(self):
        e, d, _, _ = fixture()
        for field, value in (
            ("case_id", "C2"),
            ("event_ids", "2"),
            ("source_step", "6"),
            ("source_time_s", "0.12"),
            ("source_time_s", "0.101"),
        ):
            bad = dict(e, **{field: value})
            for module in (a, v):
                with self.assertRaises((AssertionError, KeyError)):
                    module.join(bad, module.decision_index([d]))

    def test_duplicate_decision_keys_are_rejected(self):
        _, d, _, _ = fixture()
        for module in (a, v):
            with self.assertRaises(AssertionError):
                module.decision_index([d, d])

    def test_reset_command_and_mask_mismatches_are_rejected(self):
        e, d, _, _ = fixture()
        for f, value in (
            ("refined_failed_gates", "clearance;nominal"),
            ("timer_before_ticks", "3"),
            ("timer_after_ticks", "1"),
            ("timer_action", "accumulate"),
            ("ready_from_timer", "True"),
            ("speed_free_distance", "0.05"),
            ("source_zero_reason", "latch_zero"),
            ("stop_latched", "True"),
        ):
            for module in (a, v):
                with self.assertRaises(AssertionError):
                    module.join(e, module.decision_index([dict(d, **{f: value})]))

    def test_zero_lost_ticks_or_multiple_event_aliases_are_rejected(self):
        e, d, _, _ = fixture()
        for bad in (dict(e, lost_ticks="0"), dict(e, event_ids="1;2")):
            for module in (a, v):
                with self.assertRaises(AssertionError):
                    module.join(bad, module.decision_index([d]))

    def test_fixed_blocker_ids_and_order_must_agree(self):
        _, d, _, _ = fixture()
        for bad in (
            dict(d, original_blocker_ids="3;2"),
            dict(d, fixed_obstacle_ids="2"),
            dict(d, definite_obstacle_ids="2;2"),
        ):
            for module in (a, v):
                with self.assertRaises(AssertionError):
                    module.stage(bad)

    def test_inconsistent_unsafe_enclosure_is_rejected(self):
        _, d, _, _ = fixture()
        for bad in (
            dict(d, obstacle_gap_hi="0.1"),
            dict(d, obstacle_gap_lo="0.1", obstacle_gap_hi="-0.05"),
        ):
            for module in (a, v):
                with self.assertRaises(AssertionError):
                    module.stage(bad)

    def test_source_strings_are_copied_without_reformatting(self):
        e, d, w, o = fixture()
        p, b = both(e, d, w, o)
        self.assertTrue(all(p[k] == value for k, value in e.items()))
        self.assertTrue(all(p["decision_" + k] == value for k, value in d.items()))
        self.assertEqual(p["decision_input_literal"], "001.2300")
        self.assertTrue(all(b[0][k] == value for k, value in w[0].items()))
        self.assertEqual(b[0]["map_x"], "3.000000")


class BlockerEvidence(unittest.TestCase):
    def test_one_based_global_ids_keep_duplicate_coordinates(self):
        e, d, w, o = fixture()
        p, b = both(e, d, w, o)
        self.assertEqual([r["obstacle_id"] for r in b], ["2", "3"])
        for module in (a, v):
            summary = module.summaries([p], b)
            total = summary[0]
            self.assertEqual(
                (total["distinct_obstacle_ids"], total["distinct_coordinate_groups"]),
                (2, 1),
            )

    def test_missing_duplicate_or_reordered_witnesses_are_rejected(self):
        e, d, w, o = fixture()
        p = both(e, d)
        for bad in (w[:1], [w[0], w[0], w[1]], list(reversed(w))):
            for module in (a, v):
                with self.assertRaises(AssertionError):
                    module.blocker_rows([p], bad, o)

    def test_metadata_scenario_and_coordinate_mismatches_are_rejected(self):
        e, d, w, o = fixture()
        p = both(e, d)
        for field, value in (
            ("scenario", "other"),
            ("obstacle_x", "3.000001"),
            ("obstacle_order", "2"),
            ("family_id", "F2"),
            ("raw_current_state", "clear"),
        ):
            bad = [dict(w[0], **{field: value}), w[1]]
            for module in (a, v):
                with self.assertRaises(AssertionError):
                    module.blocker_rows([p], bad, o)

    def test_current_point_interval_state_boundaries(self):
        for lo, hi, state in (
            ("0", "0", "blocked"),
            ("-1", "0", "blocked"),
            ("0", "1", "unknown"),
            ("-1", "1", "unknown"),
            ("0.000000000000001", "1", "clear"),
        ):
            self.assertEqual(a.point_state(lo, hi), state)
            self.assertEqual(v.point_state(lo, hi), state)
        for fn in (a.point_state, v.point_state):
            with self.assertRaises(AssertionError):
                fn("1", "0")


class SummaryAccounting(unittest.TestCase):
    def test_empty_cells_keep_blank_maximum(self):
        e, d, w, o = fixture()
        p, b = both(e, d, w, o)
        for module in (a, v):
            rows = module.summaries([p], b)
            self.assertEqual(len(rows), 8)
            empty = next(
                r
                for r in rows
                if r["case_selector"] == "all_cases"
                and r["origin_selector"] == "ambiguous_zero"
            )
            self.assertEqual(empty["physical_resets"], 0)
            self.assertIsNone(empty["max_lost_ticks"])
            self.assertEqual(empty["obstacle_ids"], "")

    def test_maxima_and_id_sets_are_not_summed(self):
        probes, blocks = [], []
        for case, event, loss in (("C1", "1", "4"), ("C2", "2", "9")):
            e, d, w, o = fixture(case=case, event=event)
            e.update(lost_ticks=loss, timer_before_ticks=loss)
            d["timer_before_ticks"] = loss
            p, b = both(e, d, w, o)
            probes.append(p)
            blocks.extend(b)
        aa, vv = a.summaries(probes, blocks), v.summaries(probes, blocks)
        self.assertEqual(aa, vv)
        self.assertEqual(a.closure(aa), v.closure(vv))
        self.assertEqual(
            (
                aa[0]["physical_resets"],
                aa[0]["overlapping_endings"],
                aa[0]["max_lost_ticks"],
            ),
            (2, 6, 9),
        )
        self.assertEqual(aa[0]["distinct_obstacle_ids"], 2)
        bad = deepcopy(aa)
        bad[1]["physical_resets"] += 1
        for module in (a, v):
            with self.assertRaises(AssertionError):
                module.closure(bad)


if __name__ == "__main__":
    unittest.main()
