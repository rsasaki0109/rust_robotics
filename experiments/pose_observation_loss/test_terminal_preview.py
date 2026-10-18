"""Check native estimator history, exact stopping inputs and paired diagnostics."""

import copy
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import analyze_terminal_preview as audit
import build_mean_speed_probe as build
import native_terminal_preview as native
import verify_reset_mean_probe as original


def packet_row(sequence, case, step, x, *, selected="0", epoch=0, precision=False):
    start = epoch * 50 + 1
    reset = step == start
    available = epoch == 2
    margin = 0.5 + 0.1 + 2.0 * math.sqrt(2.0) * (0.03 if precision else 0.0)
    return dict(
        zip(
            native.HISTORY_FIELDS,
            (
                sequence,
                selected,
                case,
                "history",
                step,
                step * 0.02,
                x,
                10.0,
                0.0,
                0.0,
                (step - start) * 0.02,
                str(reset).lower(),
                str(reset).lower(),
                str(precision).lower(),
                0.0,
                0.4,
                0.0,
                margin,
                x,
                10.0,
                0.0,
                str(precision).lower(),
                min(step - start + 1, 25),
                str(available).lower(),
                epoch + 1,
                f"{x:.9f}",
                "10.000000000",
                "12.000000000" if available else f"{x:.9f}",
                "10.000000000",
                "0.000000000",
                0.0,
                0.0,
                "false",
            ),
        )
    )


class NativeHistoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if shutil.which("rustc") is None:
            raise RuntimeError("Activate Rust for terminal-preview tests")
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.map = cls.root / "map.csv"
        cls.map.write_text(
            "scenario,x,y\nhistory,30,30\nflip,11.22,10\nleft,11.2,10.8\nright,11.2,9.2\nwall,100,100\n"
        )
        cls.history = []
        for step in range(1, 126):
            epoch = (step - 1) // 50
            selected = "1" if step == 101 else "2" if step == 125 else "0"
            cls.history.append(
                packet_row(
                    step, "C1", step, 10.0 + 2 * epoch, selected=selected, epoch=epoch
                )
            )
        cls.history.append(packet_row(126, "C2", 1, 8.0))
        cls.history.append(packet_row(127, "C3", 1, 8.0, precision=True))
        cls.receipts = []
        for name in ("one", "two"):
            work = cls.root / name
            compiler = native.compile_native(work)
            audit.write_packet(work / "history.tsv", cls.history, native.HISTORY_FIELDS)
            evaluation = native.evaluate(work, work / "history.tsv", map_path=cls.map)
            cls.receipts.append(dict(compiler=compiler, evaluation=evaluation))
        cls.work = cls.root / "one"
        cls.trace = audit.pilot.read(cls.work / "trace.csv", native.TRACE_FIELDS)
        cls.checks = audit.pilot.read(
            cls.work / "history-checks.csv", native.CHECK_FIELDS
        )
        cls.direct = []
        for ident, x, mean in (("1", 10.0, 9.9), ("2", 9.9, 10.1)):
            for variant in native.VARIANTS:
                cls.direct.append(
                    dict(
                        zip(
                            native.DIRECT_FIELDS,
                            (
                                ident,
                                variant,
                                "flip",
                                (
                                    mean
                                    if variant in ("mean_position", "mean_pose")
                                    else x
                                ),
                                10.0,
                                0.0,
                                0.0,
                                0.0 if variant == "zero_target" else 0.4,
                                0.0,
                                0.5,
                                0.55,
                            ),
                        )
                    )
                )
        for ident, scenario, steering in (
            ("3", "left", 0.35),
            ("4", "right", -0.35),
            ("5", "wall", 0.0),
        ):
            cls.direct.append(
                dict(
                    zip(
                        native.DIRECT_FIELDS,
                        (
                            ident,
                            "recorded",
                            scenario,
                            49.0 if scenario == "wall" else 10.0,
                            25.0 if scenario == "wall" else 10.0,
                            0.0,
                            0.0,
                            0.0 if scenario == "wall" else 0.4,
                            steering,
                            0.5,
                            0.55,
                        ),
                    )
                )
            )
        audit.write_packet(cls.work / "direct.tsv", cls.direct, native.DIRECT_FIELDS)
        native.evaluate(
            cls.work, cls.work / "direct.tsv", "direct", "compact-", cls.map
        )
        cls.direct_trace = audit.pilot.read(
            cls.work / "compact-trace.csv", native.TRACE_FIELDS
        )

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_original_estimator_and_geometry_functions_are_preserved(self):
        source, base = native.source(), build.BASE.read_text()
        for marker in (
            "fn mean_pose(",
            "impl MotionPoseFilter",
            "impl StationaryEpochFusion",
            "fn stopping_preview(",
            "fn terminal_arc_clearance(",
            "fn aligned_release_preview(",
        ):
            self.assertIn(original.source_block(base, marker), source)
        self.assertIn(
            original.source_block(original.VEHICLE.read_text(), "pub fn update("),
            source,
        )
        prefix = (
            original.source_block(base, "fn stopping_preview(")
            .split("    let mut clearance = points\n", 1)[0]
            .split("    let initial =", 1)[1]
        )
        self.assertEqual(source.count("    let initial =" + prefix), 2)

    def test_fresh_native_builds_and_full_history_outputs_are_identical(self):
        self.assertEqual(self.receipts[0], self.receipts[1])
        for name in (
            "terminal-preview.rs",
            "native-terminal-preview",
            "history.tsv",
            "trace.csv",
            "history-checks.csv",
            "compiler.log",
            "native.log",
        ):
            self.assertEqual(
                (self.root / "one" / name).read_bytes(),
                (self.root / "two" / name).read_bytes(),
                name,
            )

    def test_three_epoch_position_replaces_only_position_and_all_cases_reset(self):
        self.assertEqual(len(self.trace), 8)
        first = self.trace[:4]
        self.assertEqual([r["variant"] for r in first], list(native.VARIANTS))
        self.assertEqual(float(first[0]["x"]), 12.0)
        self.assertEqual(float(first[1]["x"]), 14.0)
        self.assertEqual(float(first[2]["x"]), 14.0)
        self.assertEqual(first[0]["yaw"], first[1]["yaw"])
        self.assertEqual(first[0]["x"], first[3]["x"])
        self.assertEqual([r["case_id"] for r in self.checks], ["C1", "C2", "C3"])
        self.assertEqual([int(r["decisions"]) for r in self.checks], [125, 1, 1])
        self.assertEqual([int(r["selected_decisions"]) for r in self.checks], [2, 0, 0])

    def failed_history(self, name, mutate):
        work = self.root / name
        work.mkdir()
        shutil.copy2(
            self.work / "native-terminal-preview", work / "native-terminal-preview"
        )
        rows = copy.deepcopy(self.history)
        mutate(rows)
        audit.write_packet(work / "history.tsv", rows, native.HISTORY_FIELDS)
        with self.assertRaises(subprocess.CalledProcessError):
            native.evaluate(work, work / "history.tsv", map_path=self.map)
        self.assertTrue((work / "native.log").read_text())

    def test_history_rejects_a_wrong_rounded_preview_instead_of_using_it(self):
        self.failed_history(
            "bad-pose", lambda rows: rows[100].update(preview_x_9="14.000000000")
        )

    def test_full_precision_margin_and_selected_order_are_checked(self):
        self.failed_history(
            "bad-margin", lambda rows: rows[100].update(speed_margin="0.600000001")
        )
        self.failed_history("bad-selection", lambda rows: rows[100].update(id="2"))

    def test_history_rejects_missing_steps_and_carried_fusion_state(self):
        self.failed_history("bad-step", lambda rows: rows[50].update(step=52))
        self.failed_history(
            "bad-fusion",
            lambda rows: rows[125].update(fusion_available="true", fusion_groups=3),
        )

    def test_zero_target_retains_speed_steering_and_the_original_zero_motion_case(self):
        actual, zero = self.direct_trace[0], self.direct_trace[3]
        self.assertEqual(actual["speed"], zero["speed"])
        self.assertEqual(actual["steering"], zero["steering"])
        self.assertGreater(float(actual["motion_length"]), 0.0)
        self.assertEqual(float(zero["motion_length"]), 0.0)
        self.assertEqual(float(zero["end_x"]), 10.0)
        self.assertEqual(int(zero["motion_steps"]), 11)
        self.assertAlmostEqual(float(zero["terminal_clearance"]), 0.62)
        self.assertFalse(audit.classify(actual)["release_pass"])
        self.assertTrue(audit.classify(zero)["release_pass"])

    def test_stopping_arc_reflection_and_wall_minima_have_expected_geometry(self):
        left, right, wall = self.direct_trace[-3:]
        for field in (
            "stop_clearance",
            "terminal_clearance",
            "release_clearance",
            "motion_length",
        ):
            self.assertAlmostEqual(float(left[field]), float(right[field]), places=12)
        self.assertAlmostEqual(float(left["end_x"]), float(right["end_x"]), places=12)
        self.assertAlmostEqual(
            float(left["end_y"]) + float(right["end_y"]), 20.0, places=12
        )
        self.assertAlmostEqual(float(wall["terminal_clearance"]), 0.4, places=12)
        self.assertEqual(float(wall["stop_clearance"]), 1.0)

    def test_terminal_arc_matches_the_independent_frozen_geometry_evaluator(self):
        from analyze_recovery_arcs import arc_clearance

        for row, obstacle in zip(self.direct_trace[-3:-1], ((11.2, 10.8), (11.2, 9.2))):
            expected = arc_clearance(
                tuple(float(row[k]) for k in ("end_x", "end_y", "end_yaw")),
                float(row["steering"]),
                [obstacle],
            )
            self.assertAlmostEqual(
                float(row["terminal_clearance"]), expected, places=11
            )

    def records(self):
        result = []
        for i, mean in ((0, 9.9), (4, 10.1)):
            trace = self.direct_trace[i]
            result.append(
                dict(
                    id=trace["id"],
                    case_id="C" + trace["id"],
                    tau="0",
                    arm="mean_probe",
                    mode="random",
                    seed="1001",
                    scenario="flip",
                    planner="p",
                    step="1",
                    time_s="0.02",
                    logged_stop=trace["stop_9"],
                    logged_terminal=trace["terminal_9"],
                    logged_release=trace["release_9"],
                    logged_recovery="1.000000000",
                    logged_margin="0.6",
                    logged_release_margin=trace["release_margin_9"],
                    mean_x=str(mean),
                    mean_y="10",
                    mean_yaw="0",
                )
            )
        return result

    def test_diagnostic_keeps_favorable_and_unfavorable_position_flips(self):
        records = self.records()
        rows = [
            r
            for i, record in enumerate(records)
            for r in audit.diagnose(record, self.direct_trace[i * 4 : i * 4 + 4])
        ]
        self.assertEqual(rows[0]["original_mask"], "release")
        self.assertEqual(rows[4]["original_mask"], "ready")
        position = audit.stats([r for r in rows if r["variant"] == "mean_position"])
        self.assertEqual(position["from_parent_fail_to_pass"], 1)
        self.assertEqual(position["from_parent_pass_to_fail"], 1)
        heading = audit.stats([r for r in rows if r["variant"] == "mean_pose"])
        self.assertEqual(heading["from_parent_fail_to_pass"], 0)
        self.assertEqual(heading["from_parent_pass_to_fail"], 0)

    def test_diagnostic_rejects_reordered_variants_wrong_logs_and_changed_fixed_inputs(
        self,
    ):
        record = self.records()[0]
        for change in (
            lambda rows: rows.reverse(),
            lambda rows: rows[0].update(stop_9="0.1"),
            lambda rows: rows[1].update(speed="0.01"),
            lambda rows: rows[3].update(target="0.4"),
        ):
            rows = copy.deepcopy(self.direct_trace[:4])
            change(rows)
            with self.assertRaises(ValueError):
                audit.diagnose(record, rows)

    def test_native_output_protection_checks_auxiliary_files_too(self):
        before = (self.work / "trace.csv").read_bytes()
        with self.assertRaises(ValueError):
            native.evaluate(self.work, self.work / "history.tsv", map_path=self.map)
        self.assertEqual((self.work / "trace.csv").read_bytes(), before)
        work = self.root / "only-aux"
        work.mkdir()
        (work / "history-checks.csv").write_text("retained\n")
        with self.assertRaises(ValueError):
            native.evaluate(work, work / "queries.tsv")
        self.assertEqual((work / "history-checks.csv").read_text(), "retained\n")


class PopulationAndCliTests(unittest.TestCase):
    def test_summary_keeps_zero_cases_and_every_mask_variant_cell(self):
        meta = dict(
            case_id="C1",
            tau="0",
            arm="mean_probe",
            mode="none",
            seed="1001",
            scenario="s",
            planner="p",
        )
        cases, summary = audit.summarize([meta], [])
        self.assertEqual(len(cases), 4)
        self.assertEqual(len(summary), 192)
        self.assertTrue(all(r["selected_decisions"] == 0 for r in cases + summary))
        self.assertTrue(all(r["min_terminal_gap"] == "" for r in cases + summary))

    def test_previous_representatives_preserve_identity_roles_and_controls(self):
        record = dict(
            id="1",
            case_id="C1",
            tau="0",
            arm="mean_probe",
            mode="random",
            seed="1001",
            scenario="s",
            planner="p",
            step="5",
            time_s="0.1",
        )
        previous = dict(
            record, mask="ready", roles="first;minimum_recovery_gap", plot="false"
        )
        rows = [dict(record, variant="recorded", original_mask="ready")]
        reps = audit.representatives([record], rows, [previous])
        self.assertEqual(reps[0]["roles"], previous["roles"])
        self.assertEqual(reps[0]["original_mask"], "ready")
        bad = dict(previous, step="6")
        with self.assertRaises(ValueError):
            audit.representatives([record], rows, [bad])

    def test_protocol_inputs_and_supported_configuration_are_checked(self):
        self.assertEqual(len(audit.preflight()["input_sha256"]), 85)
        command = ["unused"] * 30
        for i, v in {
            4: "0.1",
            7: "0.02",
            17: "true",
            18: "false",
            19: "true",
            21: "false",
            22: "false",
            23: "true",
            24: "true",
            26: "true",
            27: "true",
            28: "true",
            29: "true",
        }.items():
            command[i] = v
        self.assertEqual(
            audit.configuration(dict(command=command, arm="mean_probe")),
            ("0.1", "0.02", "false"),
        )
        command[18] = "true"
        with self.assertRaises(ValueError):
            audit.configuration(dict(command=command, arm="mean_probe"))

    def test_optimized_cli_and_existing_output_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "new"
            result = subprocess.run(
                [
                    sys.executable,
                    "-O",
                    str(build.ROOT / "analyze_terminal_preview.py"),
                    "--raw",
                    tmp,
                    "--output",
                    str(output),
                ],
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("without Python optimization", result.stderr)
            self.assertFalse(output.exists())
            with self.assertRaisesRegex(ValueError, "fresh output directory"):
                audit.collect(tmp, tmp)


if __name__ == "__main__":
    unittest.main()
