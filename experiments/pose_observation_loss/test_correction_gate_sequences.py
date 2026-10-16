"""Recorded timer boundaries, simultaneous conditions and observed run endings."""

import unittest

import analyze_correction_gate_sequences as analysis


def context(route="joint", failed=(), resumed=False, released=False):
    present = (
        analysis.REQUIREMENTS
        if route == "joint"
        else ["stationary", "stop", "terminal"] if route == "independent" else []
    )
    return dict(
        route=route,
        active_ready=bool(present) and not failed,
        joint_base_usable=route == "joint"
        and not set(failed).intersection(analysis.BASE),
        **{
            f"ready_{k}": k not in failed if k in present else ""
            for k in analysis.REQUIREMENTS
        },
        resumed=resumed,
        guard_released=released,
        positive_target=False,
    )


def raw(recovery=0, guard=0, paused=True, scheduled=False, received=False):
    return dict(
        recovery_evidence=str(recovery),
        guard_release_evidence=str(guard),
        safety_paused=paused,
        correction=scheduled,
        correction_received=received,
    )


def event(route="joint", censored=False, resumes=0):
    return dict(
        event_id="1",
        cohort="known-white",
        variant="epoch",
        mode="random",
        seed="269",
        scenario="fixture",
        planner="A*",
        event_kind="received",
        event_time_s="150.000000",
        event_route=route,
        post_window_censored=censored,
        post_window_complete=not censored,
        post_resumes=str(resumes),
    )


def interval(
    offset, route="joint", failed=(), resumed=False, released=False, carried=0
):
    c = context(route, failed, resumed, released)
    mask, names = analysis.failure_mask(c)
    return dict(
        interval_id=offset + 2,
        event_id="1",
        offset_steps=offset,
        **c,
        failed_mask=mask,
        failed_requirements=names,
        timer_before_s=carried if route != "inactive" else "",
        timer_after_s=0 if route != "inactive" else "",
    )


class CorrectionGateSequenceTests(unittest.TestCase):
    def test_absent_independent_requirements_are_not_failures(self):
        c = context("independent", ["stop"])
        mask, names = analysis.failure_mask(c)
        self.assertEqual(mask, 1 << analysis.REQUIREMENTS.index("stop"))
        self.assertEqual(names, "stop")
        self.assertEqual(analysis.failure_mask(context("inactive")), (0, ""))
        c["ready_fresh"] = False
        with self.assertRaises(AssertionError):
            analysis.failure_mask(c)

    def test_failure_mask_keeps_multiple_simultaneous_requirements(self):
        mask, names = analysis.failure_mask(
            context(failed=["fresh", "samples", "terminal"])
        )
        self.assertEqual(mask, (1 << 1) | (1 << 2) | (1 << 6))
        self.assertEqual(names, "fresh+samples+terminal")

    def test_joint_threshold_uses_proposed_evidence_before_logged_reset(self):
        result = analysis.audit_timer(
            raw(0.98), raw(0, paused=False), context(resumed=True, released=True)
        )
        self.assertAlmostEqual(result["timer_proposed_s"], 1)
        self.assertEqual(result["timer_after_s"], 0)
        self.assertEqual(result["timer_threshold_s"], 1)
        self.assertTrue(result["timer_action"])
        result = analysis.audit_timer(raw(0.96), raw(0.98), context())
        self.assertFalse(result["timer_action"])

    def test_independent_guard_keeps_evidence_carried_into_window(self):
        result = analysis.audit_timer(
            raw(guard=0.48, paused=False),
            raw(guard=0, paused=False),
            context("independent", released=True),
        )
        self.assertEqual(result["timer_before_s"], 0.48)
        self.assertEqual(result["timer_threshold_s"], 0.5)
        self.assertTrue(result["timer_action"])

    def test_a_failed_requirement_resets_existing_timer(self):
        result = analysis.audit_timer(raw(0.8), raw(0), context(failed=["terminal"]))
        self.assertEqual(result["timer_proposed_s"], 0)
        result = analysis.audit_timer(
            raw(guard=0.4, paused=False),
            raw(guard=0, paused=False),
            context("independent", failed=["stationary"]),
        )
        self.assertEqual(result["timer_after_s"], 0)

    def test_newly_missed_correction_enters_joint_route_with_reset_evidence(self):
        result = analysis.audit_timer(
            raw(0.8, paused=False),
            raw(0, scheduled=True),
            context(failed=["fresh", "samples"]),
        )
        self.assertEqual(result["timer_before_s"], 0.8)
        self.assertEqual(result["timer_input_s"], 0)
        self.assertFalse(result["timer_action"])

    def test_wrong_logged_timer_or_release_flag_is_rejected(self):
        with self.assertRaises(AssertionError):
            analysis.audit_timer(raw(0.98), raw(1, paused=False), context(resumed=True))
        with self.assertRaises(AssertionError):
            analysis.audit_timer(raw(0.98), raw(0), context())
        with self.assertRaises(AssertionError):
            analysis.audit_timer(raw(0.2), raw(0.22, guard=0.1), context())

    def test_each_condition_can_hold_without_their_conjunction_ever_holding(self):
        rows = [
            interval(i, failed=["stop" if i % 2 else "terminal"]) for i in range(1, 76)
        ]
        fields = analysis.event_fields(event(), rows)
        self.assertTrue(fields["joint_all_individually_seen"])
        self.assertEqual(fields["joint_base_intervals"], 75)
        self.assertEqual(fields["joint_full_intervals"], 0)
        self.assertEqual(fields["partition"], "joint_no_full")

    def test_separated_ready_intervals_do_not_accumulate_to_a_continuous_second(self):
        rows = [
            interval(i, failed=[] if i <= 25 or 27 <= i <= 51 else ["terminal"])
            for i in range(1, 76)
        ]
        fields = analysis.event_fields(event(), rows)
        self.assertEqual(fields["joint_full_intervals"], 50)
        self.assertEqual(fields["joint_max_ready_steps"], 25)
        self.assertEqual(fields["joint_runs_requirement_failed"], 2)
        self.assertEqual(fields["partition"], "joint_short_run")

    def test_a_run_touching_horizon_is_not_a_failed_requirement(self):
        rows = [
            interval(i, failed=["terminal"] if i <= 50 else []) for i in range(1, 76)
        ]
        fields = analysis.event_fields(event(), rows)
        self.assertEqual(fields["joint_runs_horizon"], 1)
        self.assertEqual(fields["joint_runs_requirement_failed"], 0)
        self.assertEqual(fields["joint_max_ready_steps"], 25)

    def test_route_change_is_distinct_from_a_requirement_reset(self):
        rows = [interval(1), interval(2, route="inactive")]
        self.assertEqual(analysis.ready_runs(rows)[0][1], "route_change")
        fields = analysis.event_fields(event(censored=True), rows)
        self.assertEqual(fields["partition"], "censored")
        self.assertEqual(fields["joint_runs_route_change"], 1)

    def test_resume_with_carried_timer_takes_precedence_over_short_observed_run(self):
        rows = [interval(1, resumed=True, carried=0.98)] + [
            interval(i, route="inactive") for i in range(2, 76)
        ]
        fields = analysis.event_fields(event(resumes=1), rows)
        self.assertEqual(fields["joint_max_ready_steps"], 1)
        self.assertEqual(fields["joint_runs_joint_resume"], 1)
        self.assertEqual(fields["partition"], "joint_resumed")

    def test_spans_cover_post_intervals_and_preserve_left_boundary_evidence(self):
        rows = [
            interval(-1),
            interval(0),
            interval(1, route="independent", carried=0.4),
            interval(2, route="independent", carried=0.42),
            interval(3, route="independent", released=True),
            interval(4, route="inactive"),
        ]
        spans = []
        analysis.make_spans(event("independent", True), rows, spans)
        self.assertEqual(sum(s["intervals"] for s in spans), 4)
        self.assertEqual([s["intervals"] for s in spans], [2, 1, 1])
        self.assertTrue(spans[0]["continues_at_left_boundary"])
        self.assertEqual(spans[1]["end_reason"], "independent_release")
        self.assertEqual(spans[2]["end_reason"], "recorded_end")

    def test_final_event_is_censored_with_no_fabricated_post_decisions(self):
        e = event(censored=True)
        e.update(event_time_s="180.000000", **analysis.ActionWindow(9000).record())
        points = {
            step: dict(
                interval(0),
                physical_travel=0,
                scheduled=False,
                precision_activation=False,
            )
            for step in [8999, 9000]
        }
        for p in points.values():
            for field in ["interval_id", "event_id", "offset_steps"]:
                p.pop(field)
        intervals, spans = [], []
        result = analysis.export_event(e, points, intervals, spans)
        self.assertEqual([r["offset_steps"] for r in intervals], [-1, 0])
        self.assertEqual(spans, [])
        self.assertEqual(result["partition"], "censored")

    def test_initial_route_partition_is_retained_after_a_later_route_change(self):
        fields = analysis.event_fields(
            event("inactive"), [interval(i) for i in range(1, 76)]
        )
        self.assertEqual(fields["partition"], "inactive_start")
        fields = analysis.event_fields(
            event(), [interval(i, failed=["fresh"]) for i in range(1, 76)]
        )
        self.assertEqual(fields["partition"], "joint_no_base")

    def test_pattern_partition_and_overlapping_marginal_failures_have_distinct_counts(
        self,
    ):
        e = event()
        rr = [interval(i, failed=["stop", "terminal"]) for i in range(1, 76)]
        e.update(analysis.event_fields(e, rr))
        summary, patterns, requirements = analysis.summarize([e], rr)
        s = next(
            s
            for s in summary
            if (s["cohort"], s["variant"], s["event_kind"], s["event_route"])
            == ("all", "epoch", "received", "all")
        )
        self.assertEqual(s["events"], 1)
        self.assertEqual(s["joint_no_full"], 1)
        p = next(
            p
            for p in patterns
            if (p["cohort"], p["event_kind"], p["scope"])
            == ("all", "received", "joint_base")
        )
        self.assertEqual(p["intervals"], 75)
        marginal = [
            r
            for r in requirements
            if (r["cohort"], r["variant"], r["event_kind"], r["scope"])
            == ("all", "epoch", "received", "joint_base")
        ]
        self.assertEqual(sum(r["false_intervals"] for r in marginal), 150)
        empty = [s for s in summary if s["event_kind"] == "precision_only"]
        self.assertTrue(empty)
        self.assertTrue(
            all(s["events"] == sum(s[k] for k in analysis.STAGES) == 0 for s in empty)
        )


if __name__ == "__main__":
    unittest.main()
