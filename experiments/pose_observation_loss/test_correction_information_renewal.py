"""Anchor replacement, cross-time noise and censored correction pairs."""

import math
import unittest

import analyze_correction_information_renewal as analysis
from analyze_epoch_position_components import Sample


def sample(step, anchor=100, sigma=0.1, anchor_sigma=0.1, offset=(0.2, 0)):
    return Sample(
        step,
        (0.0, 0.0),
        (0.02, 0.0),
        offset,
        offset,
        anchor,
        anchor_sigma,
        sigma,
        0.0,
        (0.0, 0.0, 0.0),
    )


def group(samples, weight=1):
    return dict(
        samples=samples, weight=weight, epoch=samples[0].anchor_id, transport=False
    )


def info(groups, rho=0, current=None, estimator="recovery_mean"):
    r = analysis.information(groups, (0.0, 0.0), (0.0, 0.0, 0.0), rho)
    r.update(
        current_anchor_id=(
            current if current is not None else groups[-1]["samples"][-1].anchor_id
        ),
        estimator=estimator,
    )
    return r


def event(eid="1", received=True):
    return dict(
        event_id=eid,
        cohort="known-white",
        variant="epoch",
        mode="none",
        seed="269",
        scenario="fixture",
        planner="A*",
        event_kind="received" if received else "missed",
        event_time_s="4.000000",
        received=str(received),
        precision_activation="False",
    )


def snapshot(sid="1", phase="pre", truth=(0.0, 0.0), error=(0.2, 0.0)):
    return dict(
        snapshot_id=sid,
        phase=phase,
        input_kind="actual",
        error_x=error[0],
        error_y=error[1],
        error_norm=math.hypot(*error),
        truth_x=truth[0],
        truth_y=truth[1],
        pose_x=truth[0] + error[0],
        pose_y=truth[1] + error[1],
    )


def pair(ia, ib, e=None, pre=None, post=None, event_info=None):
    return analysis.transition(
        e or event(),
        pre or snapshot(),
        post or snapshot("2", "immediate"),
        ia,
        ib,
        event_info or ib,
        0,
    )


class CorrectionInformationTests(unittest.TestCase):
    def test_new_anchor_and_white_disjoint_samples_have_zero_covariance(self):
        a = info([group([sample(190)])])
        b = info([group([sample(200, 200)])])
        r = pair(a, b)
        self.assertTrue(r["event_anchor_replaced"])
        self.assertEqual((r["shared_samples"], r["shared_anchors"]), (0, 0))
        self.assertEqual(r["covariance_joint"], 0)
        self.assertAlmostEqual(
            r["difference_variance_joint"], r["independent_variance_joint"]
        )

    def test_noise_correlation_survives_independent_anchor_replacement(self):
        a = info([group([sample(190)])], rho=0.9)
        b = info([group([sample(200, 200)])], rho=0.9)
        r = pair(a, b)
        self.assertEqual(r["covariance_anchor"], 0)
        self.assertAlmostEqual(r["covariance_noise"], 0.01 * 0.9**10)
        self.assertLess(r["difference_variance_joint"], r["independent_variance_joint"])

    def test_missed_correction_can_clear_samples_without_replacing_anchor(self):
        a = info([group([sample(190)])])
        b = info([group([sample(200)])])
        r = pair(a, b, e=event(received=False))
        self.assertFalse(r["event_anchor_replaced"])
        self.assertEqual(r["shared_samples"], 0)
        self.assertAlmostEqual(r["covariance_anchor"], 0.01)
        self.assertAlmostEqual(r["post_shared_anchor_weight"], 1)
        self.assertAlmostEqual(r["post_event_anchor_weight"], 1)
        self.assertEqual(r["post_event_new_anchor_weight"], 0)

    def test_three_epoch_preview_distinguishes_shared_new_event_and_other_old_anchor(
        self,
    ):
        a = info([group([sample(190)])], estimator="motion_filter")
        b = info(
            [
                group([sample(90, 90)], 1 / 3),
                group([sample(190)], 1 / 3),
                group([sample(200, 200)], 1 / 3),
            ],
            estimator="three_epoch",
        )
        r = pair(a, b)
        self.assertEqual((r["shared_samples"], r["shared_anchors"]), (1, 1))
        for f in [
            "post_shared_anchor_weight",
            "post_event_new_anchor_weight",
            "post_other_anchor_weight",
        ]:
            self.assertAlmostEqual(r[f], 1 / 3)
        self.assertAlmostEqual(r["covariance_anchor"], 0.01 / 3)
        self.assertAlmostEqual(r["covariance_noise"], 0.01 / 3)

    def test_exact_sample_lists_preserve_gaps(self):
        r = info([group([sample(1), sample(8)])])
        self.assertEqual(r["noise"], {1: 0.05, 8: 0.05})
        self.assertEqual(r["details"][0]["sample_steps"], "1+8")
        self.assertAlmostEqual(r["noise_variance"], 0.005)

    def test_group_can_preserve_different_sample_sigmas(self):
        r = info([group([sample(1, sigma=0.1), sample(2, sigma=0.2)])])
        self.assertEqual(r["noise"], {1: 0.05, 2: 0.1})
        self.assertEqual(r["details"][0]["sample_sigmas"], "0.1+0.2")
        self.assertAlmostEqual(r["noise_variance"], 0.0125)

    def test_initial_anchor_is_deterministic(self):
        r = info([group([sample(1, 0, anchor_sigma=0, offset=(0.0, 0.0))])])
        self.assertEqual(r["anchor_variance"], 0)
        with self.assertRaises(AssertionError):
            info([group([sample(1, 0)])])

    def test_duplicate_observations_or_inconsistent_anchor_metadata_are_rejected(self):
        for groups in [
            [group([sample(1)], 0.5), group([sample(1)], 0.5)],
            [group([sample(1)], 0.5), group([sample(2, anchor_sigma=0.2)], 0.5)],
        ]:
            with self.assertRaises(AssertionError):
                info(groups)

    def test_cross_time_shared_anchor_metadata_must_match(self):
        a = info([group([sample(190)])])
        b = info([group([sample(200, anchor_sigma=0.2)])])
        with self.assertRaises(AssertionError):
            pair(a, b, e=event(received=False))

    def test_truth_displacement_separates_pose_jump_from_error_change(self):
        a = info([group([sample(190)])])
        b = info([group([sample(200, 200)])])
        r = pair(
            a, b, pre=snapshot(), post=snapshot("2", "immediate", truth=(3.0, 4.0))
        )
        self.assertAlmostEqual(r["pose_jump"], 5)
        self.assertAlmostEqual(r["truth_displacement"], 5)
        self.assertAlmostEqual(r["error_change_norm"], 0)
        self.assertEqual(r["error_change"], "within_tolerance")

    def test_all_accuracy_transitions_use_closed_disk(self):
        self.assertEqual(
            [
                analysis.accuracy(a, b)
                for a, b in [
                    (0.1, 0.1),
                    (0.1, 0.2),
                    (0.2, 0.1),
                    (0.2, 0.2),
                    (0.1, None),
                ]
            ],
            analysis.ACCURACY,
        )
        self.assertEqual(
            analysis.accuracy(math.nextafter(0.1, math.inf), 0.1), "outside_to_inside"
        )

    def test_censored_pair_keeps_event_and_snapshot_ids_but_blank_models(self):
        a = info([group([sample(190)])])
        immediate = info([group([sample(200, 200)])])
        r = pair(a, None, event_info=immediate)
        self.assertFalse(r["available"])
        self.assertTrue(r["event_anchor_replaced"])
        self.assertEqual((r["pre_snapshot_id"], r["post_snapshot_id"]), ("1", "2"))
        for f in [
            "pre_variance",
            "post_variance",
            "difference_variance_joint",
            "post_error",
            "shared_anchors",
        ]:
            self.assertEqual(r[f], "")
        self.assertEqual(r["accuracy_transition"], "unavailable")

    def test_summary_keeps_requested_denominator_empty_kinds_and_missing_estimator(
        self,
    ):
        a = info([group([sample(190)])])
        b = info([group([sample(200, 200)])])
        rows = [pair(a, b), pair(a, None, e=event("2"), event_info=b)]
        summary = analysis.summarize(rows)
        self.assertEqual(len(summary), 2880)
        all_row = next(
            s
            for s in summary
            if tuple(
                s[k]
                for k in [
                    "cohort",
                    "variant",
                    "event_kind",
                    "phase",
                    "input_kind",
                    "post_estimator",
                ]
            )
            == ("all", "epoch", "all", "immediate", "actual", "all")
        )
        self.assertEqual(
            (
                all_row["records"],
                all_row["matched"],
                all_row["unavailable"],
                all_row["opportunities"],
            ),
            (2, 1, 1, 2),
        )
        self.assertEqual(all_row["both_outside"], 1)
        self.assertTrue(
            all(
                s["records"] == 0 and s["pooled_change_sigma"] == ""
                for s in summary
                if s["event_kind"] == "precision_only"
            )
        )
        unavailable = next(
            s
            for s in summary
            if tuple(
                s[k]
                for k in [
                    "cohort",
                    "variant",
                    "event_kind",
                    "phase",
                    "input_kind",
                    "post_estimator",
                ]
            )
            == ("all", "epoch", "all", "immediate", "actual", "unavailable")
        )
        self.assertEqual(
            (
                unavailable["records"],
                unavailable["matched"],
                unavailable["pooled_change_sigma"],
            ),
            (1, 0, ""),
        )

    def test_source_snapshot_component_and_truth_checks_can_fail(self):
        state = analysis.Replay()
        s = sample(200)
        state.current = s
        state.recovery.append(s)
        pose = state.pose("actual")
        source = snapshot(error=(0.22, 0.0))
        source.update(
            input_kind="actual",
            estimator="recovery_mean",
            truth_time_s="3.980000",
            time_s="4.000000",
            used_sample_count="1",
            source_group_count="1",
            model_noise_sigma_per_axis=0.1,
            model_anchor_sigma_per_axis=0.1,
            model_noise_anchor_sigma_per_axis=math.sqrt(0.02),
        )
        for p, v in [
            ("noise", (0.02, 0)),
            ("anchor", (0.2, 0)),
            ("drift", (0, 0)),
            ("transport", (0, 0)),
        ]:
            for axis, x in zip(["x", "y"], v):
                source[p + "_" + axis] = x
        before = dict(x=0, y=0, time_s="3.980000")
        row = dict(time_s="4.000000")
        _, error = analysis.check_snapshot(
            state, before, row, source, {"actual": pose}, 0
        )
        self.assertLess(error, 1e-12)
        with self.assertRaises(AssertionError):
            analysis.check_snapshot(
                state, before, row, dict(source, anchor_x=0.3), {"actual": pose}, 0
            )
        with self.assertRaises(AssertionError):
            analysis.check_snapshot(
                state,
                before,
                row,
                dict(source, truth_time_s="4.000000"),
                {"actual": pose},
                0,
            )

    def test_tracks_reject_time_gaps_and_false_availability(self):
        t = analysis.Track(event(), 0, [], [])
        t.process(dict(time_s="0"))
        with self.assertRaises(AssertionError):
            t.process(dict(time_s=".04"))
        with self.assertRaises(AssertionError):
            analysis.Track(
                event(), 0, [dict(available="False", requested_time_s="179")], []
            )


if __name__ == "__main__":
    unittest.main()
