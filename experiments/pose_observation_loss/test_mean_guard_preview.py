"""Exercise default compatibility, live eligibility and independent audit failures."""

import copy
import csv
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import analyze_mean_guard_preview as audit
import analyze_mean_speed_probe as pilot
import analyze_terminal_preview as terminal
import build_mean_guard_preview as build
import native_mean_guard_preview as native
from test_terminal_preview import packet_row
from verify_reset_mean_probe import source_block


class SourceTests(unittest.TestCase):
    def test_original_geometry_and_estimators_are_preserved(self):
        original = build.BASE.read_text()
        for generated in (build.generate(original), native.source()):
            for marker in (
                "fn stopping_preview(",
                "fn terminal_arc_clearance(",
                "fn aligned_release_preview(",
                "impl MotionPoseFilter",
                "impl StationaryEpochFusion",
            ):
                with self.subTest(marker=marker):
                    self.assertIn(source_block(original, marker), generated)
        generated = build.generate(original)
        self.assertLess(
            generated.index("let mean_guard_base_pose ="),
            generated.index("let (preview_pose_x,"),
        )
        self.assertLess(
            generated.index("if let Some((x, y)) = fused_position"),
            generated.index("let mean_guard_base_pose ="),
        )
        self.assertIn("let mean_guard_preview = args.get(30)", generated)
        self.assertIn("transpose()?.unwrap_or(false)", generated)
        self.assertIn("!consistent_restart", generated)

    def test_argument_positions_and_required_mean_speed_probe(self):
        args = build.arguments("input", "output", "blackout", 17, 1)
        self.assertEqual(len(args), 29)
        self.assertEqual(args[28], "true")
        self.assertEqual(
            build.arguments("input", "output", "blackout", 17, 1, False),
            [*args, "false"],
        )
        self.assertEqual(
            build.arguments("input", "output", "blackout", 17, 1, True), [*args, "true"]
        )

    def test_fixed_sample_retains_all_uses_and_both_transition_sides(self):
        guards = [dict(used="false") for _ in range(600)]
        for row in guards[10:20]:
            row["used"] = "true"
        self.assertEqual(
            audit.guard_sample_steps(guards), {1, *range(10, 22), 250, 500, 600}
        )

    def test_preflight_and_regression_grid_are_fixed(self):
        pins = build.preflight()
        self.assertEqual(pins["seed"], 1002)
        self.assertEqual(pins["expected_cases"], 108)
        self.assertEqual(pins["expected_regression_cases"], 72)
        plan = audit.runner.regression_plan(pins)
        self.assertEqual(len(plan), 8)
        self.assertEqual(sum(r["kind"] == "legacy" for r in plan), 6)

    def test_outputs_and_optimized_cli_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            for output in (Path(tmp), build.REPO / "forbidden-guard-output"):
                with self.assertRaises(ValueError):
                    build.fresh(output)
            for script in (
                "build_mean_guard_preview.py",
                "run_mean_guard_preview.py",
                "analyze_mean_guard_preview.py",
            ):
                command = [
                    sys.executable,
                    "-O",
                    str(build.ROOT / script),
                    "--output",
                    str(Path(tmp) / "new"),
                ]
                if script.startswith("analyze"):
                    command.extend(["--raw", tmp])
                result = subprocess.run(command, capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("without Python optimization", result.stderr)
                self.assertFalse((Path(tmp) / "new").exists())


class NativeHistoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if shutil.which("rustc") is None:
            raise RuntimeError("Activate Rust for guard-preview history tests")
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.map = cls.root / "map.csv"
        cls.map.write_text("scenario,x,y\nhistory,30,30\n")
        cls.history = []
        ident = 0
        for step in range(1, 126):
            epoch = (step - 1) // 50
            row = packet_row(step, "C1", step, 10.0 + 2 * epoch, epoch=epoch)
            paused = step >= 10
            used = paused and int(row["filter_count"]) >= 10
            selected = step in (1, 125) or used
            if selected:
                ident += 1
            row["id"] = str(ident) if selected else "0"
            row["nominal_target"] = "0.0" if step % 2 else "0.4"
            row["recovery_count"] = row["filter_count"]
            row["pre_paused"] = str(paused).lower()
            base = row["preview_x_9"]
            x = row["mean_x"] if used else base
            row.update(
                guard_enabled="true",
                guard_used=str(used).lower(),
                joint_pending=str(paused).lower(),
                speed_used=str(used).lower(),
                base_x=base,
                base_y="10",
                base_yaw="0",
                selected_x=x,
                selected_y="10",
                selected_yaw="0",
                row_x_9=f"{float(x):.9f}",
                row_y_9="10.000000000",
                row_yaw_9="0.000000000",
                stop_9="10.000000000",
                terminal_9="10.000000000",
                release_9="9.950000000",
                release_margin_9="0.550000000",
            )
            cls.history.append({k: row[k] for k in native.HISTORY_FIELDS})
        cls.receipt = native.compile_native(cls.root / "compiled")
        for name in ("one", "two"):
            work = cls.root / name
            work.mkdir()
            shutil.copyfile(
                cls.root / "compiled/native-guard-preview",
                work / "native-guard-preview",
            )
            shutil.copymode(
                cls.root / "compiled/native-guard-preview",
                work / "native-guard-preview",
            )
            terminal.write_packet(
                work / "history.tsv", cls.history, native.HISTORY_FIELDS
            )
            native.evaluate(work, work / "history.tsv", cls.map)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_full_history_and_every_active_query_are_reproducible(self):
        for name in ("history.tsv", "trace.csv", "history-checks.csv"):
            self.assertEqual(
                (self.root / "one" / name).read_bytes(),
                (self.root / "two" / name).read_bytes(),
            )
        checks = pilot.read(self.root / "one/history-checks.csv")
        self.assertEqual(int(checks[0]["decisions"]), 125)
        self.assertGreater(int(checks[0]["guard_uses"]), 80)
        trace = pilot.read(self.root / "one/trace.csv", native.TRACE_FIELDS)
        self.assertTrue(
            any(float(r["target"]) == 0 and r["used"] == "true" for r in trace)
        )
        self.assertTrue(
            any(float(r["target"]) > 0 and r["used"] == "true" for r in trace)
        )

    def test_native_history_rejects_corrupted_inputs_and_geometry(self):
        for field, value in (
            ("base_x", "5"),
            ("selected_x", "5"),
            ("selected_yaw", "1"),
            ("guard_used", "true"),
            ("time_s", "0.03"),
            ("stop_9", "0.000000000"),
            ("filter_count", "5"),
            ("release_margin_9", "0.650000000"),
        ):
            rows = copy.deepcopy(self.history)
            rows[0][field] = value
            packet = self.root / (field + ".tsv")
            terminal.write_packet(packet, rows, native.HISTORY_FIELDS)
            with self.subTest(field=field), (self.root / (field + ".log")).open(
                "w"
            ) as log:
                result = subprocess.run(
                    [
                        str(self.root / "compiled/native-guard-preview"),
                        str(self.map),
                        str(packet),
                        str(self.root / (field + "-checks.csv")),
                        "exact-original-functions",
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
                self.assertNotEqual(result.returncode, 0)

    def test_every_guard_use_requires_a_native_geometry_query(self):
        rows = copy.deepcopy(self.history)
        first = next(r for r in rows if r["guard_used"] == "true")
        first["id"] = "0"
        packet = self.root / "missing-query.tsv"
        terminal.write_packet(packet, rows, native.HISTORY_FIELDS)
        with (self.root / "missing-query.log").open("w") as log:
            result = subprocess.run(
                [
                    str(self.root / "compiled/native-guard-preview"),
                    str(self.map),
                    str(packet),
                    str(self.root / "missing-query-checks.csv"),
                    "exact-original-functions",
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
            )
        self.assertNotEqual(result.returncode, 0)


class NativeProgramTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if shutil.which("rustc") is None or shutil.which("cargo") is None:
            raise RuntimeError("Activate Rust for guard-preview program tests")
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.binary, cls.legacy, cls.original, cls.receipt = build.build(
            cls.root / "build"
        )
        fixtures = cls.root / "fixtures"
        fixtures.mkdir()
        with (fixtures / "paths.csv").open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(("scenario", "planner", "index", "x", "y"))
            for scenario, (x, y) in pilot.GOALS.items():
                for planner in ("A*", "JPS", "Theta*"):
                    writer.writerow((scenario, planner, 0, x - 0.15, y))
                    writer.writerow((scenario, planner, 1, x, y))
        (fixtures / "obstacles.csv").write_text(
            "scenario,x,y\n" + "".join(f"{s},0,0\n" for s in pilot.GOALS)
        )
        cls.fixtures = fixtures
        loss_fixtures = cls.root / "loss-fixtures"
        loss_fixtures.mkdir()
        shutil.copyfile(fixtures / "obstacles.csv", loss_fixtures / "obstacles.csv")
        with (loss_fixtures / "paths.csv").open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(("scenario", "planner", "index", "x", "y"))
            for scenario, (x, y) in pilot.GOALS.items():
                for planner in ("A*", "JPS", "Theta*"):
                    writer.writerow((scenario, planner, 0, 2, 2))
                    writer.writerow((scenario, planner, 1, x, y))
        for name, binary, enabled, mode in (
            ("legacy", cls.legacy, True, "none"),
            ("omitted", cls.binary, None, "none"),
            ("disabled", cls.binary, False, "none"),
            ("enabled", cls.binary, True, "none"),
            ("active_loss", cls.binary, True, "blackout"),
        ):
            target = cls.root / name
            target.mkdir()
            args = (build.legacy if name == "legacy" else build).arguments(
                loss_fixtures if name == "active_loss" else fixtures,
                target,
                mode,
                17,
                0.0,
                enabled,
            )
            with (target / "simulation.log").open("w") as log:
                subprocess.run(
                    [str(binary), *args],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    check=True,
                )

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def case(self, name):
        key = ("open_50x50", "A*")
        directory = self.root / name

        def rows(filename):
            return [
                r
                for r in pilot.read(directory / filename)
                if (r["scenario"], r["planner"]) == key
            ]

        meta = dict(
            tau=0,
            arm=(
                "mean_preview"
                if name in ("enabled", "active_loss")
                else "fused_preview"
            ),
            mode="blackout" if name == "active_loss" else "none",
            seed=17,
            scenario=key[0],
            planner=key[1],
        )
        return (
            meta,
            rows("trajectories.csv"),
            rows("speed-probes.csv"),
            rows("guard-preview-inputs.csv"),
            rows("results.csv")[0],
        )

    def test_default_and_disabled_preserve_all_legacy_bytes(self):
        self.assertEqual(self.receipt["native_tests"], 27)
        for name in ("omitted", "disabled", "enabled"):
            for filename in audit.runner.BYTE_FILES:
                with self.subTest(name=name, filename=filename):
                    self.assertEqual(
                        (self.root / name / filename).read_bytes(),
                        (self.root / "legacy" / filename).read_bytes(),
                    )

    def test_actual_program_passes_full_audit_and_loss_eligibility(self):
        for name in ("disabled", "enabled", "active_loss"):
            meta, track, sides, guards, outcome = self.case(name)
            case, checked = audit.audit_track(
                meta, track, sides, guards, outcome, [(1, 0.0, 0.0)], []
            )
            self.assertEqual(case["status"], "held")
            self.assertEqual(checked["physical_steps"], len(sides))
            if name == "active_loss":
                self.assertGreater(case["guard_used_steps"], 0)
            else:
                self.assertEqual(case["guard_used_steps"], 0)

    def test_full_native_replay_joins_actual_live_input_logs(self):
        meta, track, sides, guards, _ = self.case("active_loss")
        work = self.root / "live-native"
        native.compile_native(work)
        selected = audit.guard_sample_steps(guards)
        rows = []
        ident = 0
        for step, (row, side, guard) in enumerate(zip(track[1:], sides, guards), 1):
            if step in selected:
                ident += 1
            rows.append(
                audit.history_row(
                    step, ident if step in selected else 0, meta, step, row, side, guard
                )
            )
        terminal.write_packet(work / "history.tsv", rows, native.HISTORY_FIELDS)
        native.evaluate(work, work / "history.tsv", self.fixtures / "obstacles.csv")
        self.assertEqual(
            int(pilot.read(work / "history-checks.csv")[0]["decisions"]), len(sides)
        )

    def test_corrupted_selection_pose_heading_and_speed_are_rejected(self):
        meta, track, sides, guards, outcome = self.case("enabled")
        for field, value in (
            ("used", "true"),
            ("enabled", "false"),
            ("joint_pending", "true"),
            ("selected_x", "1"),
            ("selected_yaw", "1"),
            ("selected_v", "1"),
            ("base_v", "1"),
            ("base_x_9", "1"),
        ):
            changed = copy.deepcopy(guards)
            changed[0][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                audit.audit_track(
                    meta, track, sides, changed, outcome, [(1, 0.0, 0.0)], []
                )

    def test_physics_and_outcome_checks_still_receive_actual_native_rows(self):
        meta, track, sides, guards, outcome = self.case("enabled")
        for field, value in (
            ("true_x", "1"),
            ("applied_acceleration", "1"),
            ("margin", "0.1"),
        ):
            changed = copy.deepcopy(sides)
            changed[0][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                audit.audit_track(
                    meta, track, changed, guards, outcome, [(1, 0.0, 0.0)], []
                )
        with self.assertRaises(ValueError):
            audit.audit_track(
                meta,
                track,
                sides,
                guards,
                dict(outcome, held="false"),
                [(1, 0.0, 0.0)],
                [],
            )

    def test_estimator_dependency_is_local_and_original_auditor_unchanged(self):
        meta, track, sides, guards, outcome = self.case("enabled")
        original = pilot.audit_track.__globals__["check_motion_filter_track"]
        with mock.patch.object(
            pilot, "check_motion_filter_track", wraps=original
        ) as checker:
            audit.audit_track(meta, track, sides, guards, outcome, [(1, 0.0, 0.0)], [])
            self.assertEqual(checker.call_count, 1)
            passed = checker.call_args.args[0]
            self.assertIsNot(passed, track)
            self.assertEqual(passed, track)
        self.assertIs(
            pilot.audit_track.__globals__["check_motion_filter_track"], original
        )

    def test_no_loss_pair_and_latent_corruption_detection(self):
        a, left, ls, lg, lo = self.case("disabled")
        b, right, rs, rg, ro = self.case("enabled")
        lc, _ = audit.audit_track(a, left, ls, lg, lo, [(1, 0.0, 0.0)], [])
        rc, _ = audit.audit_track(b, right, rs, rg, ro, [(1, 0.0, 0.0)], [])
        pair = audit.pair(b, left, right, ls, rs, lg, rg, lc, rc)
        self.assertEqual(pair["first_difference_step"], "")
        self.assertEqual(pair["outcome"], "both_held")
        corrupted = copy.deepcopy(right)
        corrupted[1]["noise_innovation_x"] = "10"
        with self.assertRaises(ValueError):
            audit.pair(b, left, corrupted, ls, rs, lg, rg, lc, rc)

    def test_enabled_guard_requires_mean_speed_probe(self):
        directory = self.root / "invalid-configuration"
        directory.mkdir()
        args = build.arguments(self.fixtures, directory, "none", 17, 0, True)
        args[28] = "false"
        with (directory / "log").open("w") as log:
            result = subprocess.run(
                [str(self.binary), *args], stdout=log, stderr=subprocess.STDOUT
            )
        self.assertNotEqual(result.returncode, 0)

    def test_first_changed_original_row_requires_a_guard_use(self):
        meta, track, sides, guards, outcome = self.case("active_loss")
        case, _ = audit.audit_track(
            meta, track, sides, guards, outcome, [(1, 0.0, 0.0)], []
        )
        step = next(i for i, g in enumerate(guards, 1) if g["used"] == "true")
        changed = copy.deepcopy(track)
        changed[step]["preview_pose_x"] = "1.000000000"
        result = audit.pair(
            meta, track, changed, sides, sides, guards, guards, case, case
        )
        self.assertEqual(result["first_difference_step"], step)
        bad = copy.deepcopy(guards)
        bad[step - 1]["used"] = "false"
        with self.assertRaisesRegex(ValueError, "no guard-preview use"):
            audit.pair(meta, track, changed, sides, sides, guards, bad, case, case)


if __name__ == "__main__":
    unittest.main()
