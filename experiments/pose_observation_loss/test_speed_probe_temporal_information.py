"""Temporal covariance and explicit chain boundaries for agreement diagnostics."""

import math
import unittest

import analyze_speed_probe_temporal_information as analysis


def fixture(n=3, maps=None, rho=0, errors=None, anchors=None, anchor_variance=0.01):
    maps = maps or [{100 + i: 0.1} for i in range(n)]
    anchors = anchors or ["50"] * n
    case = dict(
        cohort="known-white",
        variant="epoch",
        mode="none",
        seed="269",
        scenario="fixture",
        planner="A*",
    )
    chain = dict(chain_id="1", states=n, **case)
    members, pairs, noise, inputs, components, signatures = [], [], {}, {}, {}, {}
    for i in range(n):
        pids = {
            kind: str(3 * i + j + 1)
            for j, kind in enumerate(["raw", "recovery", "preview"])
        }
        members.append(
            dict(
                chain_id="1",
                chain_index=str(i),
                state_id=str(i + 1),
                time_s=str(150 + i * 0.02),
                source_event_ids="1+2",
                incoming_transition_ids=str(i) if i else "",
                **{kind + "_pose_id": pid for kind, pid in pids.items()}
            )
        )
        for kind, pid in pids.items():
            noise[pid] = maps[i]
            inputs[pid] = dict(variance_anchor=anchor_variance)
            x, y = (errors or {}).get((i, kind), (0.2, 0))
            components[pid] = dict(error_x=x, error_y=y)
            signatures[pid] = ((anchors[i], 1.0),)
        vn = analysis.ar_covariance(maps[i], maps[i], rho)
        for a, b in analysis.PAIRINGS:
            pa, pb = pids[a], pids[b]
            dx, dy = [
                components[pa]["error_" + axis] - components[pb]["error_" + axis]
                for axis in ["x", "y"]
            ]
            m = analysis.model(
                vn, anchor_variance, vn, anchor_variance, vn, anchor_variance
            )
            pairs.append(
                dict(
                    pair_id=str(len(pairs) + 1),
                    state_id=str(i + 1),
                    chain_id="1",
                    chain_index=str(i),
                    pairing=a + "_" + b,
                    a_pose_id=pa,
                    b_pose_id=pb,
                    anchor_mode="shared_single",
                    rho=rho,
                    a_variance_joint=vn + anchor_variance,
                    b_variance_joint=vn + anchor_variance,
                    delta_x=dx,
                    delta_y=dy,
                    position_separation=math.hypot(dx, dy),
                    a_inside=math.hypot(
                        *[components[pa]["error_" + axis] for axis in ["x", "y"]]
                    )
                    <= 0.1,
                    b_inside=math.hypot(
                        *[components[pb]["error_" + axis] for axis in ["x", "y"]]
                    )
                    <= 0.1,
                    **case,
                    **m
                )
            )
    return [chain], members, pairs, noise, inputs, components, signatures


def metrics(data):
    _, _, pairs, noise, inputs, components, signatures = data
    return analysis.block_metrics(
        [p for p in pairs if p["pairing"] == "recovery_preview"],
        noise,
        inputs,
        components,
        signatures,
    )


class TemporalInformationTests(unittest.TestCase):
    def test_repeating_one_observation_does_not_reduce_its_noise(self):
        maps = [{8: 0.1}] * 4
        result = metrics(fixture(4, maps=maps, anchor_variance=0))
        self.assertAlmostEqual(result["a_mean_noise_variance"], 0.01)
        self.assertAlmostEqual(result["a_time_independent_variance"], 0.0025)
        self.assertEqual(result["a_observation_uses"], 4)
        self.assertEqual(result["a_unique_observations"], 1)

    def test_overlapping_windows_retain_duplicate_observation_weights(self):
        maps = [{1: 0.05, 2: 0.05}, {2: 0.05, 3: 0.05}]
        self.assertEqual(
            analysis.mean_coefficients(maps), {1: 0.025, 2: 0.05, 3: 0.025}
        )
        result = metrics(fixture(2, maps=maps, anchor_variance=0))
        self.assertAlmostEqual(result["a_mean_noise_variance"], 0.00375)
        self.assertAlmostEqual(result["a_time_independent_variance"], 0.0025)
        self.assertEqual(
            (result["a_observation_uses"], result["a_unique_observations"]), (4, 3)
        )

    def test_disjoint_white_observations_match_independent_time_model(self):
        result = metrics(fixture(anchor_variance=0))
        self.assertAlmostEqual(result["a_mean_variance"], 0.01 / 3)
        self.assertAlmostEqual(
            result["a_mean_variance"], result["a_time_independent_variance"]
        )

    def test_disjoint_correlated_observations_keep_cross_time_covariance(self):
        rho = 0.8
        result = metrics(fixture(rho=rho, anchor_variance=0))
        expected = sum(0.01 * rho ** abs(i - j) for i in range(3) for j in range(3)) / 9
        self.assertAlmostEqual(result["a_mean_variance"], expected)
        self.assertGreater(
            result["a_mean_variance"], result["a_time_independent_variance"]
        )

    def test_fixed_anchor_variance_remains_in_mean(self):
        result = metrics(fixture())
        self.assertAlmostEqual(result["a_mean_anchor_variance"], 0.01)
        self.assertAlmostEqual(result["a_mean_variance"], 0.01 + 0.01 / 3)
        self.assertAlmostEqual(result["a_time_independent_variance"], 0.02 / 3)

    def test_identical_inputs_keep_common_error_despite_zero_difference_variance(self):
        result = metrics(fixture())
        self.assertAlmostEqual(result["mean_difference_variance"], 0)
        self.assertAlmostEqual(result["time_independent_difference_variance"], 0)
        self.assertGreater(result["independent_inputs_and_time_variance"], 0)
        self.assertAlmostEqual(result["a_mean_error_norm"], 0.2)
        self.assertAlmostEqual(result["b_mean_error_norm"], 0.2)
        self.assertGreater(result["a_mean_variance"], 0.01)

    def test_singletons_have_zero_elapsed_support_and_original_variance(self):
        spans, blocks, assignments = analysis.build(*fixture(1))
        self.assertEqual(len(assignments), 3 * 4)
        self.assertTrue(all(s["states"] == 1 and s["elapsed_s"] == 0 for s in spans))
        self.assertTrue(
            all(
                b["a_mean_variance"] == b["a_time_independent_variance"] for b in blocks
            )
        )
        self.assertTrue(
            all(
                b["left_boundary"] == "audit_start"
                and b["right_boundary"] == "audit_end"
                for b in blocks
            )
        )

    def test_signature_change_splits_information_without_splitting_agreement(self):
        spans, blocks, assignments = analysis.build(
            *fixture(4, anchors=["50", "50", "60", "60"])
        )
        self.assertEqual((len(spans), len(blocks), len(assignments)), (12, 24, 48))
        self.assertTrue(all(s["states"] == 4 for s in spans))
        first, second = blocks[:2]
        self.assertEqual(first["span_id"], second["span_id"])
        self.assertEqual(
            (first["right_boundary"], second["left_boundary"]),
            ("signature_change", "signature_change"),
        )
        self.assertAlmostEqual(first["elapsed_s"], 0.02)

    def test_missing_edge_or_time_gap_is_rejected(self):
        for broken in ["edge", "time"]:
            data = fixture()
            if broken == "edge":
                data[1][1]["incoming_transition_ids"] = ""
            else:
                data[1][1]["time_s"] = "150.04"
            with self.assertRaises(AssertionError):
                analysis.build(*data)

    def test_category_changes_partition_every_rule_without_crossing_case(self):
        data = fixture(3, errors={(1, "recovery"): (0.28, 0)})
        spans, blocks, assignments = analysis.build(*data)
        selected = [
            s
            for s in spans
            if s["pairing"] == "recovery_preview" and s["rule"] == "0p05"
        ]
        self.assertEqual(
            [s["label"] for s in selected],
            ["agree_both_outside", "disagree", "agree_both_outside"],
        )
        self.assertEqual(selected[0]["right_boundary"], "category_change")
        self.assertEqual(selected[1]["left_boundary"], "category_change")
        self.assertEqual(len(assignments), 36)
        data[2][0]["seed"] = "270"
        with self.assertRaises(AssertionError):
            analysis.build(*data)

    def test_mean_error_is_norm_of_mean_vector_with_difference_closure(self):
        result = metrics(
            fixture(2, errors={(1, "recovery"): (-0.2, 0), (1, "preview"): (-0.2, 0)})
        )
        self.assertEqual(result["a_mean_error_norm"], 0)
        self.assertEqual(result["b_mean_error_norm"], 0)
        self.assertEqual(result["mean_delta_x"], 0)

    def test_block_rejects_changing_anchor_variance_or_signature(self):
        data = fixture()
        data[4]["4"]["variance_anchor"] *= 2
        with self.assertRaises(AssertionError):
            analysis.block_metrics(data[2][::3], *data[3:])
        data = fixture(anchors=["50", "60", "60"])
        with self.assertRaises(AssertionError):
            metrics(data)

    def test_summary_keeps_empty_bins_and_overlapping_event_unions(self):
        spans, blocks, _ = analysis.build(*fixture())
        events = {
            "1": dict(event_kind="received"),
            "2": dict(event_kind="precision_only"),
        }
        ss = analysis.summarize(spans, events, False)
        bs = analysis.summarize(blocks, events, True)
        self.assertEqual((len(ss), len(bs)), (3600, 10800))
        selected = [
            s
            for s in ss
            if s["cohort"] == "all"
            and s["variant"] == "epoch"
            and s["pairing"] == "recovery_preview"
            and s["rule"] == "0p05"
            and s["label"] == "agree_both_outside"
        ]
        all_row = next(s for s in selected if s["event_kind"] == "all")
        self.assertEqual(
            (
                all_row["records"],
                all_row["states"],
                all_row["cases"],
                all_row["opportunities"],
            ),
            (1, 3, 1, 2),
        )
        self.assertEqual(
            (
                all_row["left_audit_truncated"],
                all_row["right_audit_truncated"],
                all_row["any_audit_truncated"],
            ),
            (1, 1, 1),
        )
        for kind in ["received", "precision_only"]:
            row = next(s for s in selected if s["event_kind"] == kind)
            self.assertEqual((row["states"], row["opportunities"]), (3, 1))
        self.assertTrue(
            all(
                s["records"] == 0 and s["pooled_sigma_a_mean_variance"] == ""
                for s in bs
                if s["anchor_mode"] == "preview_three"
            )
        )

    def test_noise_maps_use_actual_sample_indices_and_validate_marginals(self):
        p = dict(sample_sigma_min=".1", sample_sigma_max=".1")
        g = dict(
            sample_count="2",
            first_sample_step="10",
            last_sample_step="11",
            group_weight="1",
        )
        inp = dict(sample_count="2", variance_noise=".005", rho="0")
        self.assertEqual(
            analysis.noise_maps({"1": p}, {"1": [g]}, {"1": inp}),
            {"1": {10: 0.05, 11: 0.05}},
        )
        with self.assertRaises(AssertionError):
            analysis.noise_maps(
                {"1": p}, {"1": [dict(g, last_sample_step="12")]}, {"1": inp}
            )
        with self.assertRaises(AssertionError):
            analysis.noise_maps(
                {"1": p}, {"1": [g]}, {"1": dict(inp, variance_noise=".01")}
            )


if __name__ == "__main__":
    unittest.main()
