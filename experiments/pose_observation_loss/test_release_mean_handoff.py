"""Exercise one-shot state, complete live audits and independent native replay."""

import copy
import csv
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import analyze_release_mean_handoff as audit
import analyze_mean_speed_probe as pilot
import build_release_mean_handoff as build
import native_release_mean_handoff as native
from analyze_terminal_preview import write_packet
from verify_reset_mean_probe import source_block


class SourceAndWindowTests(unittest.TestCase):
    def test_original_functions_and_default_false_argument_are_preserved(self):
        original = build.BASE.read_text()
        for source in (build.generate(original), native.source()):
            for marker in (
                "fn stopping_preview(",
                "fn terminal_arc_clearance(",
                "fn aligned_release_preview(",
                "impl MotionPoseFilter",
                "impl StationaryEpochFusion",
            ):
                self.assertIn(source_block(original, marker), source)
        source = build.generate(original)
        self.assertIn("let post_release_mean_handoff = args.get(31)", source)
        self.assertIn(
            "assert!(!post_release_mean_handoff || mean_guard_preview)", source
        )
        self.assertIn("post_release_snapshot.take()", source)
        self.assertLess(
            source.index("post_release_snapshot.take()"),
            source.index("let (preview_pose_x,"),
        )
        args = build.arguments("in", "out", "blackout", 17, 1)
        self.assertEqual(len(args), 30)
        self.assertEqual(args[-2:], ["true", "true"])
        self.assertEqual(
            build.arguments("in", "out", "blackout", 17, 1, False), [*args, "false"]
        )
        self.assertEqual(
            build.arguments("in", "out", "blackout", 17, 1, True), [*args, "true"]
        )

    def test_frozen_grid_and_default_regression_cover_active_loss(self):
        pins = build.preflight()
        self.assertEqual(pins["seed"], 1003)
        self.assertEqual(len(pins["input_sha256"]), 164)
        specs = audit.runner.regression_plan(pins)
        self.assertEqual(sum(s["kind"] == "legacy" for s in specs), 6)
        self.assertEqual(
            next(s for s in specs if s["kind"] == "omitted")["mode"], "random"
        )

    def test_fixed_sampling_includes_fifty_decisions_after_every_resume(self):
        track = [dict(resumed="false") for _ in range(101)]
        track[20]["resumed"] = "true"
        guards = [dict(used="false") for _ in range(100)]
        hands = [dict(used="false") for _ in range(100)]
        hands[89]["used"] = "true"
        selected = audit.sample_steps(track, guards, hands)
        self.assertTrue(set(range(20, 71)).issubset(selected))
        self.assertTrue({89, 90, 91, 1, 100}.issubset(selected))

    def test_followup_keeps_lag_two_barriers_and_endpoint_censoring(self):
        meta = dict(
            tau=0,
            arm="one_step_handoff",
            mode="random",
            seed=17,
            scenario="S",
            planner="A",
        )
        track = [
            dict(
                resumed="false",
                guard_active="false",
                safety_paused="false",
                guard_triggered="false",
                time_s=str(i * 0.02),
            )
            for i in range(6)
        ]
        track[1]["resumed"] = track[5]["resumed"] = "true"
        track[3]["guard_triggered"] = "true"
        guards = [dict(joint_pending="false") for _ in range(5)]
        guards[3]["joint_pending"] = "true"
        hands = [dict(available="false", valid="true", used="false") for _ in range(5)]
        events = audit.release_followups(meta, track, guards, hands, "timeout")
        self.assertEqual(len(events), 2)
        self.assertEqual(
            (
                events[0]["first_entry_lag"],
                events[0]["followup_steps"],
                events[0]["end_reason"],
            ),
            (2, 2, "next_joint_pause"),
        )
        self.assertTrue(events[0]["entry_within_2"])
        self.assertFalse(events[0]["entry_within_1"] or events[0]["observed_5"])
        self.assertEqual(
            (
                events[1]["followup_steps"],
                events[1]["first_entry_lag"],
                events[1]["next_used"],
            ),
            (0, "", ""),
        )
        track[3]["guard_triggered"] = "false"
        track[4]["guard_triggered"] = "true"
        self.assertEqual(
            audit.release_followups(meta, track, guards, hands, "timeout")[0][
                "first_entry_lag"
            ],
            "",
        )

    def test_optimized_cli_and_internal_output_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            for output in (Path(tmp), build.REPO / "forbidden-handoff-output"):
                with self.assertRaises(ValueError):
                    build.fresh(output)
            for script in (
                "build_release_mean_handoff.py",
                "run_release_mean_handoff.py",
                "analyze_release_mean_handoff.py",
            ):
                cmd = [
                    sys.executable,
                    "-O",
                    str(build.ROOT / script),
                    "--output",
                    str(Path(tmp) / "new"),
                ]
                if script.startswith("analyze"):
                    cmd.extend(["--raw", tmp])
                result = subprocess.run(cmd, capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("without Python optimization", result.stderr)


class ProgramTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if shutil.which("rustc") is None or shutil.which("cargo") is None:
            raise RuntimeError("Activate Rust for handoff tests")
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.binary, cls.legacy, _, cls.receipt = build.build(cls.root / "build")
        for fixture, long_route in (("near", False), ("loss", True)):
            directory = cls.root / fixture
            directory.mkdir()
            with (directory / "paths.csv").open("w", newline="") as stream:
                writer = csv.writer(stream, lineterminator="\n")
                writer.writerow(("scenario", "planner", "index", "x", "y"))
                for scenario, (x, y) in pilot.GOALS.items():
                    for planner in ("A*", "JPS", "Theta*"):
                        writer.writerow(
                            (
                                scenario,
                                planner,
                                0,
                                2 if long_route else x - 0.15,
                                2 if long_route else y,
                            )
                        )
                        writer.writerow((scenario, planner, 1, x, y))
            (directory / "obstacles.csv").write_text(
                "scenario,x,y\n" + "".join(f"{s},0,0\n" for s in pilot.GOALS)
            )
        for name, binary, enabled, loss in (
            ("legacy", cls.legacy, True, False),
            ("omitted", cls.binary, None, False),
            ("disabled", cls.binary, False, False),
            ("enabled", cls.binary, True, False),
            ("active", cls.binary, True, True),
            ("active_disabled", cls.binary, False, True),
            ("active_omitted", cls.binary, None, True),
            ("legacy_loss", cls.legacy, True, True),
        ):
            target = cls.root / name
            target.mkdir()
            args = (build.legacy if name.startswith("legacy") else build).arguments(
                cls.root / ("loss" if loss else "near"),
                target,
                "blackout" if loss else "none",
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
        directory = self.root / name

        def rows(filename):
            return [
                r
                for r in pilot.read(directory / filename)
                if (r["scenario"], r["planner"]) == ("open_50x50", "A*")
            ]

        meta = dict(
            tau=0,
            arm="one_step_handoff" if name in ("enabled", "active") else "mean_preview",
            mode="blackout" if name.startswith("active") else "none",
            seed=17,
            scenario="open_50x50",
            planner="A*",
        )
        return (
            meta,
            rows("trajectories.csv"),
            rows("speed-probes.csv"),
            rows("guard-preview-inputs.csv"),
            rows("handoff-inputs.csv"),
            rows("results.csv")[0],
        )

    def test_default_and_false_match_legacy_under_active_and_no_loss(self):
        self.assertEqual(self.receipt["native_tests"], 35)
        for name, legacy in (
            ("omitted", "legacy"),
            ("disabled", "legacy"),
            ("enabled", "legacy"),
            ("active_disabled", "legacy_loss"),
            ("active_omitted", "legacy_loss"),
        ):
            for filename in audit.runner.BYTE_FILES:
                self.assertEqual(
                    (self.root / name / filename).read_bytes(),
                    (self.root / legacy / filename).read_bytes(),
                    (name, filename),
                )

    def test_actual_program_passes_full_physics_window_and_handoff_audit(self):
        for name in ("disabled", "enabled", "active", "active_disabled"):
            meta, track, sides, guards, hands, outcome = self.case(name)
            events = []
            case, checked = audit.audit_track(
                meta, track, sides, guards, hands, outcome, [(1, 0.0, 0.0)], [], events
            )
            self.assertEqual(case["status"], "held")
            self.assertEqual(checked["physical_steps"], len(sides))
            self.assertEqual(set(case), set(audit.CASE_FIELDS))
            self.assertEqual(set(checked), set(audit.VERIFY_FIELDS))
            self.assertEqual(len(events), int(outcome["resume_count"]))
            if name == "active":
                self.assertGreater(case["handoff_used_steps"], 0)
                self.assertEqual(
                    case["handoff_available_steps"],
                    case["handoff_used_steps"] + case["handoff_invalidated_steps"],
                )
            else:
                self.assertEqual(case["handoff_used_steps"], 0)

    def test_active_pair_has_common_inputs_and_first_difference_at_handoff(self):
        left = self.case("active_disabled")
        right = self.case("active")
        cases = [
            audit.audit_track(*data, [(1, 0.0, 0.0)], [], [])[0]
            for data in (left, right)
        ]
        result = audit.pair(
            right[0],
            left[1],
            right[1],
            left[2],
            right[2],
            left[3],
            right[3],
            left[4],
            right[4],
            *cases,
        )
        self.assertNotEqual(result["first_difference_step"], "")
        self.assertEqual(right[4][result["first_difference_step"] - 1]["used"], "true")
        self.assertEqual(
            set(audit.renamed(result)),
            set(map(audit.renamed_key, pilot.PAIR_FIELDS))
            | set(audit.PAIR_EXTRA_FIELDS),
        )

    def native_packet(self):
        meta, track, sides, guards, hands, _ = self.case("active")
        selected = audit.sample_steps(track, guards, hands)
        rows, ident = [], 0
        for step, (row, side, guard, hand) in enumerate(
            zip(track[1:], sides, guards, hands), 1
        ):
            if step in selected:
                ident += 1
            rows.append(
                audit.history_row(
                    step,
                    ident if step in selected else 0,
                    meta,
                    step,
                    row,
                    side,
                    guard,
                    hand,
                )
            )
        return rows, len(sides)

    def test_independent_full_native_history_and_handoff_queries_repeat(self):
        rows, decisions = self.native_packet()
        for name in ("native-one", "native-two"):
            work = self.root / name
            native.compile_native(work)
            write_packet(work / "history.tsv", rows, native.HISTORY_FIELDS)
            native.evaluate(
                work, work / "history.tsv", self.root / "loss/obstacles.csv"
            )
        for name in (
            "handoff-history.rs",
            "native-handoff-history",
            "history.tsv",
            "trace.csv",
            "history-checks.csv",
            "native.log",
            "compiler.log",
        ):
            self.assertEqual(
                (self.root / "native-one" / name).read_bytes(),
                (self.root / "native-two" / name).read_bytes(),
            )
        trace = pilot.read(self.root / "native-one/trace.csv", native.TRACE_FIELDS)
        self.assertTrue(any(r["handoff_used"] == "true" for r in trace))
        self.assertEqual(
            int(
                pilot.read(self.root / "native-one/history-checks.csv")[0]["decisions"]
            ),
            decisions,
        )

    def test_corrupt_handoff_state_snapshot_and_field_preservation_fail(self):
        meta, track, sides, guards, hands, outcome = self.case("active")
        index = next(i for i, h in enumerate(hands) if h["used"] == "true")
        for field, value in (
            ("available", "false"),
            ("used", "false"),
            ("valid", "false"),
            ("seeded", "true"),
            ("carry_x", "1"),
            ("carry_yaw", "1"),
            ("carry_speed", "1"),
            ("carry_steering", "1"),
            ("before_x", "1"),
            ("selected_yaw", "1"),
            ("selected_v", "1"),
            ("count", "1"),
            ("observation_age", "2"),
        ):
            changed = [dict(h) for h in hands]
            changed[index][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                audit.validate_handoff(meta, track, sides, guards, changed)

    def test_physics_auditor_receives_actual_data_and_detects_changes(self):
        meta, track, sides, guards, hands, outcome = self.case("active")
        for field in ("true_x", "applied_acceleration", "margin"):
            changed = [dict(s) for s in sides]
            changed[0][field] = str(float(changed[0][field]) + 1.0)
            with self.subTest(field=field), self.assertRaises(ValueError):
                audit.audit_track(
                    meta,
                    track,
                    changed,
                    guards,
                    hands,
                    outcome,
                    [(1, 0.0, 0.0)],
                    [],
                    [],
                )
        with self.assertRaises(ValueError):
            audit.audit_track(
                meta,
                track,
                sides,
                guards,
                hands,
                dict(outcome, held="false"),
                [(1, 0.0, 0.0)],
                [],
                [],
            )

    def test_native_replay_rejects_corrupt_one_shot_flag_and_full_pose(self):
        rows, _ = self.native_packet()
        work = self.root / "corrupt-native"
        native.compile_native(work)
        index = next(i for i, r in enumerate(rows) if r["handoff_used"] == "true")
        for field, value in (
            ("handoff_available", "false"),
            ("handoff_used", "false"),
            ("before_x", "1"),
            ("selected_x", "nan"),
            ("sequence", "999"),
        ):
            changed = [dict(r) for r in rows]
            changed[index][field] = value
            write_packet(work / (field + ".tsv"), changed, native.HISTORY_FIELDS)
            result = subprocess.run(
                [
                    str(work / "native-handoff-history"),
                    str(self.root / "loss/obstacles.csv"),
                    str(work / (field + ".tsv")),
                    str(work / "check.csv"),
                    "exact-original-functions",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.assertNotEqual(result.returncode, 0, field)

    def test_complete_provenance_and_export_pipeline_on_synthetic_cases(self):
        raw = self.root / "pipeline-raw"
        raw.mkdir()
        shutil.copytree(self.root / "build", raw / "build")
        pins = dict(
            build.preflight(),
            seed=17,
            noise_correlation_times=[0],
            modes=["blackout"],
            expected_cases=18,
            expected_pairs=9,
            expected_regression_cases=18,
        )
        trials, all_rows = [], []
        for arm, name, enabled in (
            ("mean_preview", "active_disabled", False),
            ("one_step_handoff", "active", True),
        ):
            relative = f"tau0/{arm}-blackout-17"
            shutil.copytree(self.root / name, raw / relative)
            command = [
                str(self.binary),
                *build.arguments(
                    self.root / "loss", self.root / name, "blackout", 17, 0, enabled
                ),
            ]
            trials.append(
                dict(
                    tau=0,
                    arm=arm,
                    mode="blackout",
                    seed=17,
                    directory=relative,
                    command=command,
                    sha256={
                        f: build.sha(raw / relative / f) for f in audit.runner.FILES
                    },
                )
            )
            all_rows.extend(
                dict(tau=0, arm=arm, mode="blackout", seed=17, **r)
                for r in pilot.read(raw / relative / "results.csv")
            )
        pilot.write(raw / "all-results.csv", all_rows, tuple(all_rows[0]))
        specs = [
            dict(name="legacy", kind="legacy", tau=0, mode="blackout"),
            dict(name="omitted", kind="omitted", tau=0, mode="blackout"),
        ]
        regressions = []
        for spec, name in zip(specs, ("legacy_loss", "active_omitted")):
            relative = "regression/" + spec["name"]
            shutil.copytree(self.root / name, raw / relative)
            files = tuple(
                f
                for f in audit.runner.FILES
                if spec["kind"] == "omitted" or f != "handoff-inputs.csv"
            )
            args = (build if spec["kind"] == "omitted" else build.legacy).arguments(
                self.root / "loss",
                self.root / name,
                "blackout",
                17,
                0,
                None if spec["kind"] == "omitted" else True,
            )
            command = [
                str(self.binary if spec["kind"] == "omitted" else self.legacy),
                *args,
            ]
            regressions.append(
                dict(
                    **spec,
                    command=command,
                    directory=relative,
                    sha256={f: build.sha(raw / relative / f) for f in files},
                    compared=list(audit.runner.BYTE_FILES),
                    cases=9,
                )
            )
        record = dict(
            method=audit.runner.METHOD,
            source_commit=subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=build.REPO, text=True
            ).strip(),
            working_tree_clean=True,
            script_sha256={
                name: build.sha(build.ROOT / name) for name in build.SCRIPTS
            },
            protocol_sha256=build.sha(build.STUDY / "protocol.md"),
            freeze_sha256=build.sha(build.STUDY / "freeze-pins.json"),
            input_sha256=pins["input_sha256"],
            build_sources_sha256=build.sha(raw / "build/build-sources.json"),
            cases=18,
            trials=trials,
            regressions=regressions,
            all_results_sha256=build.sha(raw / "all-results.csv"),
            **{
                k: self.receipt[k]
                for k in (
                    "generated_source_sha256",
                    "candidate_binary_sha256",
                    "legacy_binary_sha256",
                    "original_binary_sha256",
                    "native_tests",
                )
            },
        )
        (raw / "provenance.json").write_text(json.dumps(record, indent=2) + "\n")
        with mock.patch.object(
            build, "preflight", return_value=pins
        ), mock.patch.object(build, "FIXTURES", self.root / "loss"), mock.patch.object(
            audit.runner, "regression_plan", return_value=specs
        ):
            self.assertEqual(audit.check_provenance(raw, pins)["cases"], 18)
            result = audit.audit(raw, self.root / "pipeline-audit")
            self.assertEqual(
                (result["audit"]["cases"], result["audit"]["pairs"]), (18, 9)
            )
            self.assertGreater(result["audit"]["handoff_uses"], 0)
            self.assertEqual(
                len(pilot.read(self.root / "pipeline-audit/summary.csv")), 12
            )
            changed = dict(record, generated_source_sha256="0" * 64)
            (raw / "provenance.json").write_text(json.dumps(changed))
            with self.assertRaises(ValueError):
                audit.check_provenance(raw, pins)


if __name__ == "__main__":
    unittest.main()
