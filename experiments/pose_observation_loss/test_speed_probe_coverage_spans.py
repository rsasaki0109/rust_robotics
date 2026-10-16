"""Physical chain topology, coverage boundaries and truncated elapsed support."""

import math
import unittest

import analyze_speed_probe_coverage_spans as analysis


def fixture(times=(150.0, 150.02, 150.04, 150.06)):
    nodes, poses = {}, {}
    for i, time in enumerate(times, 1):
        n = dict(
            cohort="known-white",
            variant="epoch",
            mode="none",
            seed="269",
            scenario="fixture",
            planner="A*",
            time_s=str(time),
            source_event_ids="1",
        )
        for j, kind in enumerate(analysis.ESTIMATES):
            pid = str((i - 1) * 3 + j + 1)
            n[kind + "_pose_id"] = pid
            poses[pid] = dict(
                pose_id=pid,
                state_id=str(i),
                pose_kind=kind,
                truth_clear=i not in [2, 3],
                position_error=0.25 if i in [2, 3] else 0.04,
                heading_error_rad=0,
                **{k: n[k] for k in analysis.KEY},
                source_event_ids="1",
            )
            for yaw in analysis.YAWS:
                poses[pid][f"clear_budget_{yaw}deg"] = 0.15
                poses[pid][f"block_budget_{yaw}deg"] = -0.15
        nodes[str(i)] = n
    return nodes, poses


def edge(ref="1", event="1"):
    return [dict(transition_id=ref, event_id=event)]


class CoverageSpanTests(unittest.TestCase):
    def test_four_exclusive_coverage_components(self):
        self.assertEqual(
            [
                analysis.coverage(p, h)
                for p, h in [(True, True), (False, True), (True, False), (False, False)]
            ],
            analysis.COVERAGES,
        )

    def test_exact_radius_and_yaw_boundaries_are_covered(self):
        _, pp = fixture((150.0,))
        p = pp["1"]
        p["position_error"] = 0.1
        p["heading_error_rad"] = math.radians(4)
        self.assertEqual(analysis.evaluate(p, 0.1, 4)["coverage"], "covered")
        p["position_error"] = math.nextafter(0.1, math.inf)
        self.assertEqual(
            analysis.evaluate(p, 0.1, 4)["coverage"], "position_only_outside"
        )
        p["heading_error_rad"] = math.nextafter(math.radians(4), math.inf)
        self.assertEqual(analysis.evaluate(p, 0.1, 4)["coverage"], "both_outside")
        p["position_error"] = 0.1
        self.assertEqual(
            analysis.evaluate(p, 0.1, 4)["coverage"], "heading_only_outside"
        )

    def test_runs_are_maximal_and_keep_return_to_previous_label(self):
        self.assertEqual(
            list(analysis.runs(["a", "a", "b", "a"])),
            [(0, 1, "a"), (2, 2, "b"), (3, 3, "a")],
        )
        self.assertEqual(list(analysis.runs(["a"])), [(0, 0, "a")])
        with self.assertRaises(AssertionError):
            list(analysis.runs([]))

    def test_explicit_edges_define_chains_without_filling_a_missing_edge(self):
        nn, _ = fixture()
        cc, mm = analysis.build_chains(nn, {("1", "2"): edge(), ("3", "4"): edge("2")})
        self.assertEqual([c["states"] for c in cc], [2, 2])
        self.assertEqual([m["chain_index"] for m in mm], [0, 1, 0, 1])
        self.assertTrue(all(c["elapsed_s"] == 0.02 for c in cc))

    def test_duplicate_opportunity_refs_do_not_duplicate_physical_time(self):
        nn, _ = fixture((150.0, 150.02))
        cc, mm = analysis.build_chains(nn, {("1", "2"): edge("3") + edge("2", "2")})
        self.assertEqual(
            (cc[0]["states"], cc[0]["edges"], cc[0]["source_pair_refs"]), (2, 1, 2)
        )
        self.assertEqual(mm[1]["incoming_transition_ids"], "2+3")
        self.assertEqual(mm[1]["incoming_event_ids"], "1+2")
        self.assertEqual(cc[0]["elapsed_s"], 0.02)

    def test_isolated_nodes_have_zero_elapsed_support(self):
        nn, _ = fixture((150.0, 150.04))
        cc, mm = analysis.build_chains(nn, {})
        self.assertEqual(len(cc), 2)
        self.assertTrue(all(c["states"] == 1 and c["elapsed_s"] == 0 for c in cc))
        self.assertTrue(all(m["incoming_transition_ids"] == "" for m in mm))

    def test_chains_cannot_cross_cases(self):
        nn, _ = fixture((150.0, 150.02))
        nn["2"]["seed"] = "270"
        with self.assertRaises(AssertionError):
            analysis.build_chains(nn, {("1", "2"): edge()})

    def test_chains_cannot_cross_time_gaps(self):
        nn, _ = fixture((150.0, 150.04))
        with self.assertRaises(AssertionError):
            analysis.build_chains(nn, {("1", "2"): edge()})

    def test_duplicate_physical_decision_timestamps_are_rejected(self):
        nn, _ = fixture((150.0, 150.0))
        with self.assertRaises(AssertionError):
            analysis.build_chains(nn, {})

    def test_invalid_grid_and_reverse_edges_are_rejected(self):
        with self.assertRaises(AssertionError):
            analysis.step(150.001)
        nn, _ = fixture((150.0, 150.02))
        with self.assertRaises(AssertionError):
            analysis.build_chains(nn, {("2", "1"): edge()})

    def test_chain_ids_and_member_order_are_deterministic(self):
        nn, _ = fixture()
        edges = {("1", "2"): edge(), ("3", "4"): edge("2")}
        a = analysis.build_chains(nn, edges)
        b = analysis.build_chains(
            dict(reversed(list(nn.items()))), dict(reversed(list(edges.items())))
        )
        self.assertEqual(a, b)

    def test_elapsed_support_and_internal_vs_audit_boundaries(self):
        nn, pp = fixture()
        cc, mm = analysis.build_chains(
            nn, {("1", "2"): edge(), ("2", "3"): edge("2"), ("3", "4"): edge("3")}
        )
        spans, assignments = analysis.make_spans(cc, mm, pp)
        rr = [
            s
            for s in spans
            if (s["pose_kind"], s["radius_m"], s["yaw_bound_deg"], s["family"])
            == ("raw", 0.1, 4, "coverage")
        ]
        self.assertEqual(
            [s["label"] for s in rr], ["covered", "position_only_outside", "covered"]
        )
        self.assertEqual([s["elapsed_s"] for s in rr], [0, 0.02, 0])
        self.assertEqual(
            [(s["left_boundary"], s["right_boundary"]) for s in rr],
            [
                ("audit_start", "label_change"),
                ("label_change", "label_change"),
                ("label_change", "audit_end"),
            ],
        )
        ss = [
            s
            for s in spans
            if (
                s["pose_kind"],
                s["radius_m"],
                s["yaw_bound_deg"],
                s["family"],
                s["label"],
            )
            == ("raw", 0.1, 4, "clear_conflict", "conflict")
        ]
        self.assertEqual(len(ss), 1)
        self.assertEqual((ss[0]["states"], ss[0]["elapsed_s"]), (2, 0.02))
        self.assertEqual(assignments["4", 0.1, 4]["clear_conflict"], ss[0]["span_id"])
        self.assertEqual(assignments["7", 0.1, 4]["clear_conflict"], ss[0]["span_id"])

    def test_same_label_on_opposite_sides_of_missing_edge_stays_separate(self):
        nn, pp = fixture()
        cc, mm = analysis.build_chains(nn, {("1", "2"): edge(), ("3", "4"): edge("2")})
        spans, _ = analysis.make_spans(cc, mm, pp)
        rr = [
            s
            for s in spans
            if (
                s["pose_kind"],
                s["radius_m"],
                s["yaw_bound_deg"],
                s["family"],
                s["label"],
            )
            == ("raw", 0.1, 4, "clear_conflict", "conflict")
        ]
        self.assertEqual(len(rr), 2)
        self.assertTrue(all(s["states"] == 1 and s["elapsed_s"] == 0 for s in rr))
        self.assertEqual(
            (rr[0]["right_boundary"], rr[1]["left_boundary"]),
            ("audit_end", "audit_start"),
        )

    def test_span_summary_retains_empty_groups_and_truncation_counts(self):
        nn, pp = fixture((150.0,))
        cc, mm = analysis.build_chains(nn, {})
        spans, _ = analysis.make_spans(cc, mm, pp)
        summary = analysis.span_summary(spans, {"1": dict(event_kind="received")})
        r = next(
            r
            for r in summary
            if (
                r["cohort"],
                r["variant"],
                r["event_kind"],
                r["pose_kind"],
                r["radius_m"],
                r["yaw_bound_deg"],
                r["family"],
                r["label"],
            )
            == ("all", "epoch", "all", "raw", 0.1, 4, "coverage", "covered")
        )
        self.assertEqual(
            (
                r["spans"],
                r["states"],
                r["singleton_spans"],
                r["left_audit_truncated"],
                r["right_audit_truncated"],
                r["both_audit_truncated"],
            ),
            (1, 1, 1, 1, 1, 1),
        )
        self.assertEqual(
            (r["cases"], r["opportunities"], r["median_elapsed_s"], r["max_elapsed_s"]),
            (1, 1, 0, 0),
        )
        r = next(r for r in summary if r["event_kind"] == "precision_only")
        self.assertEqual(
            (r["spans"], r["median_elapsed_s"], r["max_elapsed_s"]), (0, "", "")
        )


if __name__ == "__main__":
    unittest.main()
