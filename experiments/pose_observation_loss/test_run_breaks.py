"""Check reset causes, context boundaries and physical-edge identity."""

from copy import deepcopy
from fractions import Fraction
import itertools
import unittest

import analyze_run_breaks as a
import verify_run_breaks as v
import test_local_admission as old


def strings(row):
    return {k: "" if x is None else str(x) for k, x in row.items()}


def context(center=7750, reference="155.0", base_id=0):
    rows, event = old.context_slots()
    result = []
    for offset, row in zip(range(-25, 26), rows):
        s = strings(row)
        s.update(
            reference_time_s=reference,
            offset_steps=str(offset),
            source_step=str(center + offset),
            source_time_s=str((center + offset) / 50),
        )
        result.append(s)
    links = {
        (s["event_id"], reference, s["offset_steps"]): base_id + i
        for i, s in enumerate(result, 1)
    }
    return result, event, links


def passing(rows, start, stop, carry=0, branch="joint", release=False):
    for i, offset in enumerate(range(start, stop + 1)):
        s = rows[offset + 25]
        action = "release" if release and offset == stop else "accumulate"
        s.update(
            local_audited="True",
            local_branch=branch,
            local_source_certainty="pass",
            local_failed_gates="",
            local_unknown_gates="",
            local_source_nominal_positive="True",
            local_source_zero_reason="positive",
            local_timer_before_ticks=str(carry + i),
            local_timer_after_ticks=str(0 if action == "release" else carry + i + 1),
            local_timer_action=action,
            local_ready_from_timer="True",
            local_threshold_ticks="50" if branch == "joint" else "25",
        )
        for g in a.GATES:
            s["local_source_gate_" + g] = (
                "pass"
                if branch == "joint" or g in ("stationary", "release")
                else "not_applicable"
            )
    return rows


def failing(rows, offset, before, gates=("nominal",)):
    s = rows[offset + 25]
    s.update(
        local_source_certainty="fail",
        local_failed_gates=";".join(g for g in a.GATES if g in gates),
        local_unknown_gates="",
        local_timer_before_ticks=str(before),
        local_timer_after_ticks="0",
        local_timer_action="reset",
        local_ready_from_timer="False",
        local_source_nominal_positive=str("nominal" not in gates),
        local_source_zero_reason="cap_zero" if "nominal" in gates else "positive",
        local_prior_recovery_evidence=format(before / 50, ".6f"),
        local_input_recovery_evidence="0.000000",
    )
    for g in a.GATES:
        s["local_source_gate_" + g] = "fail" if g in gates else "pass"
    return s


def one_end(rows, links, number=1):
    left, right = a.segments(rows), v.scan(rows)
    assert left == right and len(left) >= number
    piece = left[number - 1]
    aa, vv = a.original_run(piece, rows, number), v.reconstruct_original(
        piece, rows, number
    )
    assert aa == vv
    run = strings(aa)
    first = a.ending(run, piece, rows, links)
    second = v.ending(run, piece, rows, links)
    assert first.keys() == second.keys()
    for f in first:
        if isinstance(first[f], float):
            assert abs(first[f] - second[f]) <= 1e-12
        else:
            assert first[f] == second[f], (f, first[f], second[f])
    return first


def reset_context(gates=("nominal",), **kwargs):
    rows, event, links = context(**kwargs)
    passing(rows, 0, 2, 5)
    failing(rows, 3, 8, gates)
    return rows, event, links


def reference(event):
    result = {k: event[k] for k in a.META}
    result.update(
        event_id=event["event_id"],
        time_s="155.0",
        phase="available",
        branch="joint",
        raw_distance_direction="toward",
        probe_origin="initial_unsafe",
        timer_action="reset",
        local_joint_max_known_pass_ticks="3",
        local_joint_post_max_known_pass_ticks="3",
        history_raw_post_missing_slots="0",
    )
    return result


def native_case():
    row, event, snap = old.calculate()
    row = strings(row)
    row["offset_steps"] = "0"
    mapping = dict(
        row_id=1,
        case_id=row["case_id"],
        event_id=row["event_id"],
        reference_time_s=row["reference_time_s"],
        offset_steps=0,
        source_time_s=row["source_time_s"],
    )
    q = strings(old.v.native_queries([mapping], {1: snap})[0])
    trace = dict(row_id="1", branch="joint", recovery_ready="true", ready="true")
    trace.update({"gate_" + g: "true" for g in a.GATES})
    for f in (
        "recovery_evidence",
        "guard_release_evidence",
        "safety_paused",
        "guard_active",
        "resumed",
        "guard_released",
    ):
        trace[f] = row["local_input_" + f]
    return row, strings(mapping), q, trace


class SegmentEnds(unittest.TestCase):
    def test_schema_constants_agree(self):
        for name in (
            "META",
            "GROUP",
            "GATES",
            "KINDS",
            "STATUSES",
            "METRICS",
            "HAS",
            "REF_BASE",
            "ANCHOR_BASE",
            "SUMMARY_BASE",
            "EXTRA",
            "EDGE_FIELDS",
        ):
            self.assertEqual(getattr(a, name), getattr(v, name))

    def test_carried_ticks_are_included_in_reset_loss(self):
        rows, _, links = reset_context()
        end = one_end(rows, links)
        self.assertEqual(
            (end["lost_ticks"], end["carried_ticks"], end["segment_ticks"]), (8, 5, 3)
        )
        self.assertEqual(end["checkpoint_span_s"], "0.04")
        self.assertEqual(end["evidence_added_s"], "0.06")
        self.assertAlmostEqual(end["lost_evidence_s"], 0.16)

    def test_all_simultaneous_causes_survive_classification(self):
        for gates, kind in (
            (("nominal",), "nominal_only"),
            (("clearance", "nominal"), "nominal_with_other"),
            (("clearance", "release"), "other_only"),
        ):
            rows, _, links = reset_context(gates)
            end = one_end(rows, links)
            self.assertEqual(end["failure_kind"], kind)
            self.assertEqual(end["next_failed_gates"], ";".join(gates))

    def test_terminator_side_controls_post_counts(self):
        for stop, side in ((-2, "pre"), (-1, "reference"), (0, "post")):
            rows, e, links = context()
            passing(rows, stop, stop, 2)
            failing(rows, stop + 1, 3)
            end = one_end(rows, links)
            self.assertEqual(end["next_side"], side)
            for module in (a, v):
                r = module.reference(reference(e), [end])
                self.assertEqual(r["post_gate_failure_ends"], int(side == "post"))

    def test_right_context_is_not_a_reset(self):
        rows, _, links = context()
        passing(rows, 24, 25, 2)
        end = one_end(rows, links)
        self.assertEqual(end["end_status"], "right_context_limit")
        self.assertIsNone(end["lost_ticks"])
        self.assertIsNone(end["next_offset_steps"])

    def test_next_loss_missing_context_is_not_a_failure(self):
        rows, _, links = context()
        passing(rows, 1, 2)
        rows[28].update(local_audited="False", slot_status="at_or_after_next_loss")
        end = one_end(rows, links)
        self.assertEqual(end["end_status"], "missing_context")
        self.assertEqual(end["next_slot_status"], "at_or_after_next_loss")
        self.assertIsNone(end["native_next_row_id"])
        self.assertIsNone(end["lost_ticks"])

    def test_unknown_with_observed_reset_does_not_certify_cause(self):
        rows, _, links = reset_context()
        rows[28].update(
            local_source_certainty="unknown",
            local_failed_gates="",
            local_unknown_gates="nominal",
            local_source_gate_nominal="unknown",
        )
        end = one_end(rows, links)
        self.assertEqual(end["end_status"], "unknown_boundary")
        self.assertEqual(end["next_timer_action"], "reset")
        self.assertIsNone(end["failure_kind"])
        self.assertIsNone(end["lost_ticks"])

    def test_branch_change_precedes_gate_failure(self):
        rows, _, links = reset_context()
        rows[28]["local_branch"] = "guard"
        end = one_end(rows, links)
        self.assertEqual(end["end_status"], "branch_change")
        self.assertIsNone(end["lost_ticks"])

    def test_recorded_release_precedes_context_boundary(self):
        for offset in (2, 25):
            rows, _, links = context()
            passing(rows, offset, offset, 49, release=True)
            if offset < 25:
                rows[offset + 26]["local_audited"] = "False"
            end = one_end(rows, links)
            self.assertEqual(end["end_status"], "recorded_release")
            self.assertEqual(end["decomposition"], "released_at_threshold")
            self.assertIsNone(end["lost_ticks"])

    def test_guard_release_uses_25_tick_threshold(self):
        rows, _, links = context()
        passing(rows, 0, 1, 23, branch="guard", release=True)
        end = one_end(rows, links)
        self.assertEqual(end["threshold_ticks"], "25")
        self.assertEqual(end["end_status"], "recorded_release")

    def test_counter_disagreement_cannot_be_reported_as_a_reset(self):
        for field, value in (
            ("local_timer_before_ticks", "7"),
            ("local_timer_after_ticks", "1"),
            ("local_timer_action", "stay_zero"),
            ("local_ready_from_timer", "True"),
        ):
            rows, _, links = reset_context()
            rows[28][field] = value
            for module in (a, v):
                with self.assertRaises(AssertionError):
                    module.ending(
                        strings(a.original_run(a.segments(rows)[0], rows, 1)),
                        a.segments(rows)[0],
                        rows,
                        links,
                    )

    def test_source_steps_must_be_consecutive_inside_and_after_segment(self):
        for index in (26, 28):
            rows, _, links = reset_context()
            rows[index]["source_step"] = str(int(rows[index]["source_step"]) + 1)
            for module in (a, v):
                with self.assertRaises(AssertionError):
                    module.ending(
                        strings(a.original_run(a.segments(rows)[0], rows, 1)),
                        a.segments(rows)[0],
                        rows,
                        links,
                    )

    def test_scan_covers_pass_slots_and_breaks_unknown_missing_release(self):
        rows, _, _ = context()
        passing(rows, -3, -2, 48, release=True)
        passing(rows, -1, 0)
        passing(rows, 2, 3)
        rows[28].update(local_audited="False", slot_status="at_or_after_next_loss")
        rows[26].update(local_source_certainty="unknown")
        for scan in (a.segments, v.scan):
            pieces = scan(rows)
            self.assertEqual([len(p) for p in pieces], [2, 2, 1])
            self.assertEqual(sum(map(len, pieces)), 5)

    def test_no_certified_reset_keeps_maxima_blank(self):
        rows, e, links = context()
        passing(rows, 25, 25)
        for module in (a, v):
            r = module.reference(reference(e), [one_end(rows, links)])
            self.assertEqual(r["lost_ticks"], 0)
            self.assertIsNone(r["max_lost_ticks"])
            self.assertIsNone(r["post_max_lost_ticks"])


class PhysicalEdges(unittest.TestCase):
    def fixture(self):
        first, _, ids1 = reset_context()
        second, _, ids2 = context(center=7751, reference="155.02", base_id=1000)
        passing(second, -1, 1, 5)
        failing(second, 2, 8)
        ends = [one_end(first, ids1), one_end(second, ids2)]
        smap = {
            (s["event_id"], s["reference_time_s"], s["offset_steps"]): s
            for s in (*first, *second)
        }
        traces = {
            str(e["native_next_row_id"]): dict(
                row_id=str(e["native_next_row_id"]), ready="false", branch="joint"
            )
            for e in ends
        }
        return ends, smap, traces

    def test_duplicate_windows_form_one_physical_reset(self):
        ends, slots, traces = self.fixture()
        aa, vv = a.physical_edges(ends, slots, traces), v.physical_edges(
            ends, slots, traces
        )
        self.assertEqual(aa, vv)
        self.assertEqual(len(aa), 1)
        self.assertEqual(
            (aa[0]["multiplicity"], aa[0]["lost_ticks"], aa[0]["source_step"]),
            (2, 8, 7753),
        )
        self.assertEqual(len(aa[0]["reference_keys"].split(";")), 2)

    def test_disagreeing_raw_inputs_or_native_outputs_are_rejected(self):
        for native in (False, True):
            ends, slots, traces = self.fixture()
            if native:
                traces[str(ends[1]["native_next_row_id"])]["ready"] = "true"
            else:
                slots[
                    ends[1]["event_id"],
                    ends[1]["reference_time_s"],
                    str(ends[1]["next_offset_steps"]),
                ]["local_input_speed_margin"] = "1.1"
            for module in (a, v):
                with self.assertRaises(AssertionError):
                    module.physical_edges(ends, slots, traces)

    def test_same_reference_cannot_count_twice_at_one_edge(self):
        ends, slots, traces = self.fixture()
        for module in (a, v):
            with self.assertRaises(AssertionError):
                module.physical_edges([ends[0], ends[0]], slots, traces)


class NativeLinks(unittest.TestCase):
    def test_all_query_rows_cover_unique_reference_slots(self):
        s, m, q, t = native_case()
        for module in (a, v):
            links, _ = module.native_links([s], [m], [q], [t])
            self.assertEqual(list(links.values()), [1])
            with self.assertRaises(AssertionError):
                module.native_links([s], [m, m], [q], [t])

    def test_native_wire_identity_and_scalar_mismatches_are_rejected(self):
        s, m, q, t = native_case()
        for f, value in (
            ("source_step", "7751"),
            ("measured_speed", "0.1"),
            ("before_joint_ticks", "1"),
            ("age_exact", "0.21"),
            ("row_id", "2"),
        ):
            for module in (a, v):
                with self.assertRaises(AssertionError):
                    module.native_slot(s, dict(q, **{f: value}), t)

    def test_native_gate_counter_and_flag_mismatches_are_rejected(self):
        s, m, q, t = native_case()
        for f, value in (
            ("gate_nominal", "false"),
            ("ready", "false"),
            ("recovery_evidence", "0.04"),
            ("safety_paused", "false"),
        ):
            for module in (a, v):
                with self.assertRaises(AssertionError):
                    module.native_slot(s, q, dict(t, **{f: value}))

    def test_unknown_center_can_differ_but_query_integrity_is_required(self):
        s, _, q, t = native_case()
        s.update(local_source_certainty="unknown", local_source_gate_nominal="unknown")
        t.update(gate_nominal="false", ready="false", recovery_evidence="0.000000")
        for module in (a, v):
            module.native_slot(s, q, t)
            with self.assertRaises(AssertionError):
                module.native_slot(s, dict(q, nominal="0.1"), t)


class Accounting(unittest.TestCase):
    def events(self):
        _, e, _ = old.calculate()
        e = dict(
            e,
            returned="True",
            received="True",
            census_case_weight="1/2",
            received_case_weight="1/2",
            parent_rp_records="0",
            rp_records="0",
            rp_different_preview_records="0",
            rp_case_mass="0",
            rp_different_preview_case_mass="0",
        )
        return [
            e,
            dict(e, event_id="2", returned="False"),
            dict(
                e,
                case_id="C2",
                family_id="F2",
                event_id="3",
                census_case_weight="1",
                received_case_weight="1",
            ),
        ]

    def test_original_unequal_weights_nonreturns_empty_shares_and_closures(self):
        events = self.events()
        rr = {f: "" for f in a.REF_BASE}
        rr.update({f: events[0][f] for f in a.META})
        rr.update(
            event_id=events[0]["event_id"],
            phase="available",
            **{m: 0 for m in a.METRICS}
        )
        rr.update(
            audited_references=1,
            runs=1,
            gate_failure_ends=1,
            nominal_only_ends=1,
            post_gate_failure_ends=1,
            post_nominal_only_ends=1,
            lost_ticks=3,
            post_lost_ticks=3,
        )
        aa, vv = a.anchors(events, [rr]), v.anchors(events, [rr])
        self.assertEqual(aa, vv)
        self.assertTrue(all(aa[1][f] is None for f in (*a.METRICS, *a.HAS)))
        cells = []
        for mode, cat in itertools.product(
            ("none", "random", "blackout", "all_modes"), ("all", *a.CATEGORIES)
        ):
            selected = [
                e
                for e in events
                if mode in ("all_modes", e["mode"])
                and cat in ("all", e["anchor_category"])
            ]
            returned = [e for e in selected if e["returned"] == "True"]
            cell = {f: "0" for f in a.SUMMARY_BASE}
            cell.update(
                dict(
                    zip(
                        a.GROUP,
                        (
                            events[0]["partition"],
                            events[0]["noise"],
                            events[0]["variant"],
                            mode,
                            cat,
                        ),
                    )
                )
            )
            cell.update(
                census_events=str(len(selected)),
                returned_events=str(len(returned)),
                census_case_mass=str(
                    sum(
                        (Fraction(e["census_case_weight"]) for e in selected),
                        Fraction(),
                    )
                ),
                received_case_mass=str(
                    sum(
                        (Fraction(e["received_case_weight"]) for e in selected),
                        Fraction(),
                    )
                ),
                returned_case_mass=str(
                    sum(
                        (Fraction(e["census_case_weight"]) for e in returned),
                        Fraction(),
                    )
                ),
            )
            cells.append(cell)
        ac, vc = a.summaries(cells, aa), v.summaries(cells, vv)
        self.assertEqual(ac, vc)
        self.assertEqual(a.closure(ac), v.closure(vc))
        cell = next(
            r for r in ac if r["mode"] == "random" and r["category_selector"] == "all"
        )
        h = a.HAS[0]
        self.assertEqual(cell[h + "_fraction"], Fraction(1, 2))
        self.assertEqual(cell[h + "_case_mass_fraction"], Fraction(1, 3))
        self.assertTrue(any(r[h + "_fraction"] is None for r in ac))
        bad = deepcopy(ac)
        bad[0]["nominal_only_ends"] += 1
        for fn in (a.closure, v.closure):
            with self.assertRaises(AssertionError):
                fn(bad)

    def test_wrong_original_weight_and_inexact_fraction_are_rejected(self):
        events = self.events()
        events[0]["census_case_weight"] = "1"
        for module in (a, v):
            with self.assertRaises(AssertionError):
                module.anchors(events, [])
            with self.assertRaises(AssertionError):
                module.check("0.3333333333333333", Fraction(1, 3))

    def test_decimal_weight_strings_keep_exact_count_based_sevenths(self):
        e = self.events()[0]
        events = [
            dict(
                e,
                event_id=str(i),
                census_case_weight="0.14285714285714285",
                received_case_weight="0.14285714285714285",
            )
            for i in range(7)
        ]
        for module in (a, v):
            rows = module.anchors(events, [])
            self.assertEqual(sum(r["exact_census_case_weight"] for r in rows), 1)
            self.assertTrue(
                all(r["exact_census_case_weight"] == Fraction(1, 7) for r in rows)
            )
            self.assertTrue(
                all(r["census_case_weight"] == "0.14285714285714285" for r in rows)
            )


if __name__ == "__main__":
    unittest.main()
