"""Signed decomposition, persistent offsets, pooled moments and source joins."""

import math
import unittest

import analyze_speed_probe_error_components as analysis
from analyze_epoch_position_components import Replay, Sample, decompose


def sample(noise=(0, 0), anchor=(0, 0), drift=(0, 0)):
    return Sample(
        250,
        (10, 10),
        noise,
        tuple(a + d for a, d in zip(anchor, drift)),
        anchor,
        250,
        0.1,
        0.1,
        0,
        (0, 0, 0),
    )


def record(pid="1", kind="raw", noise=(0.02, 0), anchor=(0.12, 0), drift=(0, 0)):
    s = sample(noise, anchor, drift)
    groups = [dict(samples=[s], epoch=250, weight=1.0, transport=False)]
    r, _ = decompose(groups, (10, 10), (0, 0, 0), 0)
    r.update(
        pose_id=pid,
        state_id=pid,
        pose_kind=kind,
        estimator="raw_observation",
        cohort="known-white",
        variant="epoch",
        mode="none",
        seed="269",
        scenario="fixture",
        planner="A*",
        source_event_ids="1",
        central_clear_truth_blocks=False,
        envelope_clear_truth_blocks=False,
    )
    return r


class ErrorComponentTests(unittest.TestCase):
    def test_raw_group_uses_current_sample_and_preserves_anchor(self):
        state = Replay()
        state.current = sample(
            noise=(0.04, -0.03), anchor=(0.1, 0.2), drift=(0.001, -0.002)
        )
        name, groups = analysis.input_groups(state, "raw")
        self.assertEqual(name, "raw_observation")
        self.assertEqual(groups[0]["samples"], [state.current])
        self.assertEqual(
            (groups[0]["epoch"], groups[0]["weight"], groups[0]["transport"]),
            (250, 1, False),
        )
        r, _ = decompose(groups, (10, 10), state.odom, 0)
        self.assertAlmostEqual(r["error_x"], 0.141)
        self.assertAlmostEqual(r["error_y"], 0.168)
        self.assertAlmostEqual(r["anchor_x"], 0.1)
        self.assertAlmostEqual(r["drift_y"], -0.002)

    def test_recovery_empty_window_uses_raw_fallback(self):
        state = Replay()
        state.current = sample()
        name, groups = analysis.input_groups(state, "recovery")
        self.assertEqual(name, "raw_fallback")
        self.assertEqual(groups[0]["samples"], [state.current])
        state.recovery.extend(
            [sample(noise=(0.1, 0)), sample(noise=(-0.1, 0))._replace(step=251)]
        )
        name, groups = analysis.input_groups(state, "recovery")
        self.assertEqual(name, "recovery_mean")
        r, _ = decompose(groups, (10, 10), state.odom, 0)
        self.assertAlmostEqual(r["noise_norm"], 0)
        self.assertEqual(r["used_sample_count"], 2)

    def test_constant_offset_has_zero_centered_rms(self):
        rr = [dict(anchor_x=0.3, anchor_y=0.4)] * 4
        m = analysis.moments(rr, "anchor")
        self.assertAlmostEqual(m["anchor_rms"], 0.5)
        self.assertAlmostEqual(m["anchor_mean_norm"], 0.5)
        self.assertEqual(m["anchor_centered_rms"], 0)

    def test_cancelling_vectors_have_zero_mean_and_nonzero_variation(self):
        m = analysis.moments(
            [dict(noise_x=1.0, noise_y=0.0), dict(noise_x=-1.0, noise_y=0.0)], "noise"
        )
        self.assertEqual(
            (m["noise_rms"], m["noise_mean_norm"], m["noise_centered_rms"]), (1, 0, 1)
        )

    def test_non_axis_moments_obey_vector_variance_identity(self):
        rr = [dict(error_x=x, error_y=y) for x, y in [(1, 2), (3, -1), (-2, 5)]]
        m = analysis.moments(rr, "error")
        self.assertAlmostEqual(
            m["error_rms"] ** 2,
            m["error_mean_norm"] ** 2 + m["error_centered_rms"] ** 2,
        )
        self.assertAlmostEqual(m["error_mean_x"], 2 / 3)
        self.assertAlmostEqual(m["error_mean_y"], 2)

    def test_singleton_variation_zero_and_empty_moments_blank(self):
        m = analysis.moments([dict(error_x=0.2, error_y=-0.3)], "error")
        self.assertEqual(m["error_centered_rms"], 0)
        self.assertAlmostEqual(m["error_mean_norm"], math.hypot(0.2, 0.3))
        self.assertTrue(all(v == "" for v in analysis.moments([], "error").values()))

    def test_negative_component_projections_are_retained(self):
        r = record(noise=(-1, 0), anchor=(3, 0))
        m = analysis.metrics([r])
        self.assertAlmostEqual(m["noise_mean_projection"], -1)
        self.assertAlmostEqual(m["anchor_mean_projection"], 3)
        self.assertAlmostEqual(
            sum(m[p + "_mean_projection"] for p in analysis.PARTS), 2
        )
        self.assertEqual(m["anchor_largest"], 1)

    def test_component_norms_do_not_add_even_when_projections_do(self):
        r = record(noise=(1, 1), anchor=(1, -1))
        m = analysis.metrics([r])
        self.assertAlmostEqual(m["error_rms"], 2)
        self.assertGreater(m["noise_rms"] + m["anchor_rms"], m["error_rms"])
        self.assertAlmostEqual(
            m["noise_mean_projection"] + m["anchor_mean_projection"], 2
        )

    def test_zero_error_direction_has_explicit_none_label(self):
        r = record(noise=(-1, 0), anchor=(1, 0))
        self.assertEqual(r["largest_projection"], "none")
        m = analysis.metrics([r])
        self.assertEqual(m["none_largest"], 1)
        self.assertEqual(sum(m[p + "_largest"] for p in analysis.PARTS), 0)

    def test_source_opportunities_stay_attached_to_selected_loss_rows(self):
        a, b = record("1"), record("2")
        a["source_event_ids"] = "1+2"
        b["source_event_ids"] = "2"
        a["central_clear_truth_blocks"] = True
        a["envelope_clear_truth_blocks"] = True
        events = {
            "1": dict(event_kind="received"),
            "2": dict(event_kind="precision_only"),
        }
        losses = [
            dict(
                pose_id="1",
                radius_m="0.0",
                yaw_bound_deg="0",
                event_id="1",
                command_only_zero_loss="True",
            ),
            dict(
                pose_id="2",
                radius_m="0.0",
                yaw_bound_deg="0",
                event_id="2",
                command_only_zero_loss="False",
            ),
        ]
        ss = analysis.summarize([a, b], losses, events)

        def choose(pop, condition, kind="all"):
            return next(
                s
                for s in ss
                if (
                    s["cohort"],
                    s["variant"],
                    s["event_kind"],
                    s["pose_kind"],
                    s["population"],
                    s["condition"],
                )
                == ("all", "epoch", kind, "raw", pop, condition)
            )

        self.assertEqual(
            choose("all_states", "central_clear_truth_blocks")["opportunities"], 2
        )
        self.assertEqual(
            choose("raw_zero_loss", "central_clear_truth_blocks")["opportunities"], 1
        )
        self.assertEqual(choose("raw_zero_loss", "all")["records"], 2)
        self.assertEqual(choose("command_only_zero_loss", "all")["records"], 1)
        self.assertEqual(
            choose("raw_zero_loss", "central_clear_truth_blocks", "precision_only")[
                "records"
            ],
            0,
        )
        self.assertTrue(
            all(s["records"] == 0 for s in ss if s["pose_kind"] == "preview")
        )

    def test_span_moments_keep_membership_boundaries_and_comparison_family(self):
        a, b = record("1", noise=(0.1, 0)), record("2", noise=(-0.1, 0))
        mm = [dict(chain_id="1", raw_pose_id="1"), dict(chain_id="1", raw_pose_id="2")]
        base = dict(
            chain_id="1",
            first_chain_index="0",
            last_chain_index="1",
            first_pose_id="1",
            last_pose_id="2",
            states="2",
            radius_m="0.1",
            yaw_bound_deg="4",
            pose_kind="raw",
            left_boundary="audit_start",
            right_boundary="audit_end",
        )
        spans = [
            dict(base, span_id="1", family="clear_conflict", label="no_conflict"),
            dict(base, span_id="2", family="coverage", label="position_only_outside"),
            dict(
                base,
                span_id="3",
                family="coverage",
                label="position_only_outside",
                radius_m="0.2",
            ),
        ]
        rr = analysis.span_components(spans, mm, [a, b])
        self.assertEqual([s["span_id"] for s in rr], ["1", "2"])
        self.assertTrue(
            all(
                s["left_boundary"] == "audit_start"
                and s["right_boundary"] == "audit_end"
                for s in rr
            )
        )
        self.assertAlmostEqual(rr[0]["noise_mean_norm"], 0)
        self.assertAlmostEqual(rr[0]["noise_centered_rms"], 0.1)
        self.assertAlmostEqual(rr[0]["anchor_mean_norm"], 0.12)
        self.assertAlmostEqual(rr[0]["anchor_centered_rms"], 0)

    def test_span_member_count_mismatch_is_rejected(self):
        s = dict(
            chain_id="1",
            first_chain_index="0",
            last_chain_index="1",
            states="2",
            radius_m="0.1",
            yaw_bound_deg="4",
        )
        with self.assertRaises(AssertionError):
            analysis.span_components(
                [s], [dict(chain_id="1", raw_pose_id="1")], [record()]
            )

    def test_replayed_case_without_audited_targets_has_blank_match_errors(self):
        r = record()
        track = analysis.ComponentReplay(r, 0, {}, {}, {}, {})
        track.steps = 9000
        v = track.validate()
        self.assertEqual(
            (v["replayed_intervals"], v["audited_states"], v["reconstructed_poses"]),
            (9000, 0, 0),
        )
        self.assertEqual((v["max_pose_error"], v["max_norm_error"]), ("", ""))


if __name__ == "__main__":
    unittest.main()
