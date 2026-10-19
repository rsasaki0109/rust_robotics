"""Exercise truth alignment, evaluation isolation and complete native error audits."""

import copy
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import analyze_release_pose_accuracy as audit
import analyze_handoff_transition_replay as previous
import analyze_mean_speed_probe as pilot
import build_release_mean_handoff as build
from test_handoff_transition_replay import synthetic_case
from verify_reset_mean_probe import source_block


def physical_fixture(arm, mode):
    case, track, sides, guards, hands = synthetic_case(arm, mode)
    before = (12.0 if arm == "mean_preview" else 10.0, 10.0, 0.0, 0.0)
    track[0].update(dict(zip(("x", "y", "yaw", "v"), map(str, before))))
    for step, (row, side, guard) in enumerate(zip(track[1:], sides, guards), 1):
        a = (0.5, 0.5, 0.5, -1.0, -0.5, 0.0)[step - 1]
        steer = 0.15 if step % 2 else -0.1
        side.update(
            measured_speed=str(before[3]),
            applied_acceleration=str(a),
            applied_steering=str(steer),
            count="0" if abs(before[3]) > 0.02 else "25",
        )
        if side["count"] == "0":
            side.update(mean_x="9", mean_y="10")
        guard.update(
            base_x="10",
            base_y="10",
            base_yaw="0",
            base_v=str(before[3]),
            selected_v=str(before[3]),
        )
        after = (
            before[0] + before[3] * math.cos(before[2]) * 0.02,
            before[1] + before[3] * math.sin(before[2]) * 0.02,
            audit.angular(before[2] + before[3] / 0.4 * math.tan(steer) * 0.02),
            before[3] + a * 0.02,
        )
        for key, value in zip(("x", "y", "yaw", "v"), after):
            side["true_" + key] = format(value, ".17e")
            row[key] = format(value, ".9f")
        row["epoch_fusion_available"] = "true" if step >= 3 else "false"
        before = after
    return case, track, sides, guards, hands


class AlignmentTests(unittest.TestCase):
    def test_truth_is_previous_full_precision_post_state(self):
        _, track, sides, _, _ = physical_fixture("mean_preview", "random")
        pre = audit.physical_checkpoints(track, sides)
        self.assertEqual(
            pre[0], tuple(float(track[0][k]) for k in ("x", "y", "yaw", "v"))
        )
        self.assertEqual(
            pre[2], tuple(float(sides[1]["true_" + k]) for k in ("x", "y", "yaw", "v"))
        )
        self.assertNotEqual(pre[2][0], float(sides[2]["true_x"]))
        self.assertNotEqual(pre[3][2], float(track[3]["yaw"]))

    def test_post_state_speed_join_rounding_order_and_time_corruption_fail(self):
        for kind in ("shift", "speed", "rounding", "order", "time"):
            _, track, sides, _, _ = physical_fixture("mean_preview", "random")
            if kind == "shift":
                sides[2]["true_x"] = sides[1]["true_x"]
                track[3]["x"] = track[2]["x"]
            if kind == "speed":
                sides[2]["measured_speed"] = ".123"
            if kind == "rounding":
                track[3]["y"] = "12"
            if kind == "order":
                sides[2]["step"] = "2"
            if kind == "time":
                sides[2]["time_s"] = ".04"
            with self.assertRaises(ValueError, msg=kind):
                audit.physical_checkpoints(track, sides)

    def test_wrap_and_truth_cannot_change_sensor_poses(self):
        _, track, sides, guards, hands = physical_fixture("mean_preview", "random")
        item = dict(row=track[2], side=sides[1], guard=guards[1], handoff=hands[1])
        before = audit.sensor_poses(item, (12.0, 10.0, 0.0), True)
        other = copy.deepcopy(item)
        other["side"].update(true_x="-900", true_y="900", true_yaw="3", true_v="4")
        self.assertEqual(before, audit.sensor_poses(other, (12.0, 10.0, 0.0), True))
        one = audit.evaluation_poses(before, (12.0, 10.0, 3.14, 0.0))
        two = audit.evaluation_poses(before, (20.0, 30.0, -3.14, 0.0))
        for variant in audit.VARIANTS[:5]:
            self.assertEqual(one[variant], two[variant])
        self.assertEqual(one["truth_xy_eval"][2], before["recorded"][2])
        self.assertEqual(one["truth_pose_eval"][2], 3.14)
        self.assertAlmostEqual(
            abs(audit.angular(math.pi - 0.001 - (-math.pi + 0.001))), 0.002
        )

    def test_numerical_ties_and_empty_cells(self):
        self.assertEqual(audit.compare_errors(1.0, 1.0 + 0.5e-12), "tied")
        self.assertEqual(audit.compare_errors(1.0, 1.0 + 2e-12), "closer")
        self.assertEqual(audit.compare_errors(1.0 + 2e-12, 1.0), "farther")
        empty = audit.metrics([])
        self.assertTrue(all(empty[k] == 0 for k in audit.COUNT_FIELDS))
        self.assertTrue(all(empty[k] == "" for k in audit.ERROR_FIELDS))

    def test_complete_freeze_and_optimized_python_rejection(self):
        pins = audit.preflight()
        self.assertEqual(len(pins["input_sha256"]), 209)
        self.assertEqual(pins["expected_new_queries"], 7 * 12801)
        result = subprocess.run(
            [
                sys.executable,
                "-O",
                "-B",
                "-c",
                "import analyze_release_pose_accuracy as a; a.preflight()",
            ],
            cwd=build.ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b"Python-O", result.stderr)


class NativePipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.maps = cls.root / "maps.csv"
        cls.maps.write_text("scenario,x,y\nline,11,10\n")
        cls.data = [
            physical_fixture(arm, mode)
            for mode in ("none", "random")
            for arm in ("mean_preview", "one_step_handoff")
        ]
        cls.catalogue = [d[0] for d in cls.data]
        cls.windows = []
        cls.checks = []
        for case, track, sides, guards, hands in cls.data:
            truth = audit.physical_checkpoints(track, sides)
            windows = previous.select_windows(case, track, sides, guards, hands, 50)
            for w in windows:
                for p in w["points"]:
                    p["physical_pre"] = truth[p["step"] - 1]
            cls.windows.extend(windows)
            c = dict(
                **{k: case[k] for k in audit.META},
                **dict.fromkeys(audit.VERIFY_COUNTS, 0),
            )
            c.update(
                physical_steps=len(truth),
                pre_speed_matches=len(truth),
                post_state_matches=len(truth),
                window_points=sum(len(w["points"]) for w in windows),
            )
            cls.checks.append(c)
        for i, w in enumerate(cls.windows, 1):
            w["transition_id"] = i
        work = cls.root / "prior"
        previous.native.compile_native(work)
        packet, _ = previous.write_packet(work, cls.windows)
        previous.native.evaluate(work, packet, cls.maps)
        geometries = pilot.read(work / "trace.csv", previous.native.TRACE_FIELDS)
        retained = []
        by = {
            (w["transition_id"], lag): (w, p)
            for w in cls.windows
            for lag, p in enumerate(w["points"])
        }
        for g in geometries:
            w, p = by[int(g["transition_id"]), int(g["lag"])]
            case = w["case"]
            retained.append(
                dict(
                    transition_id=w["transition_id"],
                    **{k: case[k] for k in audit.META},
                    status=case["status"],
                    resume_step=w["resume_step"],
                    step=p["step"],
                    **{
                        k: v
                        for k, v in g.items()
                        if k not in ("transition_id", "scenario")
                    },
                    **previous.previous.threshold(g),
                )
            )
            if g["variant"] == "recorded":
                prior = p["previous_row"]
                ready = (
                    prior["guard_active"] == "false"
                    and p["guard"]["joint_pending"] == "false"
                )
                trigger = ready and previous.previous.threshold(g)["would_enter"]
                p["row"]["guard_triggered"] = str(trigger).lower()
                p["row"]["guard_active"] = str(
                    trigger or prior["guard_active"] == "true"
                ).lower()
        previous.write(work / "queries.csv", retained, previous.QUERY_FIELDS)
        cls.groups = audit.load_previous(work, 32, 12)
        cls.receipts = []
        for name in ("one", "two"):
            cls.receipts.append(
                audit.evaluate(
                    copy.deepcopy(cls.windows),
                    cls.catalogue,
                    cls.groups,
                    copy.deepcopy(cls.checks),
                    cls.root / name,
                    cls.maps,
                )
            )

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_original_native_functions_and_every_fresh_output_match(self):
        source = audit.native.source()
        for marker in (
            "fn stopping_preview(",
            "fn terminal_arc_clearance(",
            "fn aligned_release_preview(",
            "fn point_segment_distance(",
        ):
            self.assertIn(source_block(build.BASE.read_text(), marker), source)
        one, two = self.root / "one", self.root / "two"
        self.assertEqual(
            {p.relative_to(one) for p in one.rglob("*") if p.is_file()},
            {p.relative_to(two) for p in two.rglob("*") if p.is_file()},
        )
        for p in one.rglob("*"):
            if p.is_file():
                self.assertEqual(
                    p.read_bytes(), (two / p.relative_to(one)).read_bytes()
                )

    def test_all_cases_variants_and_empty_controls_are_retained(self):
        result = self.receipts[0][0]
        self.assertEqual(
            (
                result["cases"],
                result["case_variant_rows"],
                result["pairs"],
                result["pair_variant_rows"],
                result["resumes"],
                result["window_points"],
                result["native_queries"],
            ),
            (4, 28, 2, 14, 2, 12, 84),
        )
        self.assertEqual(result["prior_variant_matches"], 32)
        self.assertEqual(result["physical_steps"], 24)
        self.assertEqual(result["summary_cells"], 3528)
        cases = pilot.read(self.root / "one/cases.csv", audit.CASE_FIELDS)
        self.assertEqual({r["status"] for r in cases}, {"held", "timeout"})
        for r in cases:
            if r["mode"] == "none":
                self.assertEqual(r["queries"], "0")
                self.assertEqual(r["rmse_position_error_m"], "")

    def test_truth_references_are_zero_error_and_yaw_is_isolated(self):
        samples = pilot.read(self.root / "one/samples.csv", audit.SAMPLE_FIELDS)
        for r in samples:
            if r["variant"] in ("truth_xy_eval", "truth_pose_eval"):
                self.assertEqual(float(r["position_error_m"]), 0.0)
            if r["variant"] == "truth_pose_eval":
                self.assertEqual(float(r["abs_yaw_error_rad"]), 0.0)
                self.assertEqual(float(r["entry_error_pose_m"]), 0.0)
            if r["variant"] == "truth_xy_eval":
                self.assertEqual(float(r["entry_error_xy_m"]), 0.0)
        self.assertTrue(
            any(
                float(r["abs_yaw_error_rad"]) > 0
                for r in samples
                if r["variant"] == "truth_xy_eval"
            )
        )

    def test_favorable_unfavorable_geometry_and_expired_rows_survive(self):
        rows = pilot.read(self.root / "one/samples.csv", audit.SAMPLE_FIELDS)
        carry = [r for r in rows if r["variant"] == "carried_diagnostic_xy"]
        self.assertTrue(any(r["comparison"] == "closer" for r in carry))
        self.assertTrue(any(r["comparison"] == "farther" for r in carry))
        self.assertTrue(any(r["entry_only_pose"] == "true" for r in rows))
        self.assertTrue(any(r["entry_missed_pose"] == "true" for r in rows))
        self.assertTrue(any(r["recovery_source"] == "raw_fallback" for r in rows))
        self.assertTrue(
            all(r["eligible"] == "false" for r in carry if int(r["lag"]) >= 3)
        )
        self.assertTrue(any(float(r["target"]) == 0 for r in rows))

    def test_statistics_match_direct_pooled_errors_and_eligibility(self):
        rows = pilot.read(self.root / "one/samples.csv", audit.SAMPLE_FIELDS)
        cases = pilot.read(self.root / "one/cases.csv", audit.CASE_FIELDS)
        for c in cases:
            data = [
                r
                for r in rows
                if r["case_id"] == c["case_id"] and r["variant"] == c["variant"]
            ]
            self.assertEqual(int(c["queries"]), len(data))
            if data:
                errors = [float(r["position_error_m"]) for r in data]
                self.assertAlmostEqual(
                    float(c["mean_position_error_m"]), sum(errors) / len(errors)
                )
                self.assertAlmostEqual(
                    float(c["rmse_position_error_m"]),
                    math.sqrt(sum(e * e for e in errors) / len(errors)),
                )
                self.assertEqual(float(c["max_position_error_m"]), max(errors))
                self.assertEqual(
                    int(c["closer"]) + int(c["farther"]) + int(c["tied"]), len(data)
                )

    def test_corrupt_retained_pose_validity_geometry_and_sequence_fail(self):
        for field, value in (
            ("x", "99"),
            ("valid_prefix", "false"),
            ("carry_x", "99"),
            ("status", "other"),
        ):
            groups = copy.deepcopy(self.groups)
            groups[1, 1]["recorded"][field] = value
            with self.assertRaises(ValueError, msg=field):
                audit.prepare_points(
                    copy.deepcopy(self.windows), groups, copy.deepcopy(self.checks)
                )
        groups = copy.deepcopy(self.groups)
        groups[1, 1]["recorded"]["stop"] = "99"
        with self.assertRaisesRegex(ValueError, "Prior native variant geometry"):
            audit.evaluate(
                copy.deepcopy(self.windows),
                self.catalogue,
                groups,
                copy.deepcopy(self.checks),
                self.root / "bad-geometry",
                self.maps,
            )
        original = audit.native.evaluate

        def extra(work, packet, maps):
            result = original(work, packet, maps)
            p = work / "trace.csv"
            with p.open("a") as stream:
                stream.write(p.read_text().splitlines()[-1] + "\n")
            return result

        with mock.patch.object(audit.native, "evaluate", side_effect=extra):
            with self.assertRaisesRegex(ValueError, "Extra native accuracy"):
                audit.evaluate(
                    copy.deepcopy(self.windows),
                    self.catalogue,
                    self.groups,
                    copy.deepcopy(self.checks),
                    self.root / "extra",
                    self.maps,
                )

    def test_full_case_collection_checks_all_physical_steps(self):
        raw = self.root / "raw"
        raw.mkdir()
        for case, track, sides, guards, hands in self.data:
            directory = raw / f"tau0/{case['arm']}-{case['mode']}-17"
            directory.mkdir(parents=True)
            for name, rows in (
                ("trajectories.csv", track),
                ("speed-probes.csv", sides),
                ("guard-preview-inputs.csv", guards),
                ("handoff-inputs.csv", hands),
            ):
                pilot.write(directory / name, rows, tuple(rows[0]))
        pins = dict(
            noise_correlation_times=[0.0],
            modes=["none", "random"],
            arms=["mean_preview", "one_step_handoff"],
            seed=17,
            expected_cases=4,
            expected_decisions=24,
            expected_resumes=2,
            expected_window_points=12,
            horizon=50,
        )
        windows, checks = audit.collect_windows(raw, self.catalogue, pins)
        self.assertEqual(sum(c["physical_steps"] for c in checks), 24)
        self.assertEqual(len(windows), 2)
        points = audit.prepare_points(windows, self.groups, checks)
        self.assertEqual(len(points), 12)


if __name__ == "__main__":
    unittest.main()
