"""Test pinned source edits, native defaults and complete pilot audit checks."""

import copy
import csv
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import analyze_mean_speed_probe as audit
import build_mean_speed_probe as build
from verify_reset_mean_probe import source_block


class SourceTests(unittest.TestCase):
    def test_source_edits_preserve_original_geometry_and_every_other_helper(self):
        old = build.BASE.read_text()
        new = build.generate(old)
        for marker in (
            "fn speed_free_distance(",
            "fn point_segment_distance(",
            "fn stopping_preview(",
            "fn mean_pose(",
            "struct NoiseRng",
        ):
            with self.subTest(marker=marker):
                self.assertIn(source_block(old, marker), new)
        self.assertEqual(
            new.count(
                "speed_free_distance(speed_probe_pose, steering, speed_margin, obs)"
            ),
            1,
        )
        self.assertNotIn(
            "speed_free_distance(sensed, steering, speed_margin, obs)", new
        )
        self.assertIn("unwrap_or(false)", new)
        self.assertIn("joint_recovery && averaged_recovery", new)

    def test_missing_and_duplicate_anchors_fail(self):
        for text in ("not here", "needle needle"):
            with self.assertRaises(ValueError):
                build.once(text, "needle", "replacement")
        self.assertEqual(build.once("one needle", "needle", "two"), "one two")

    def test_original_argument_order_and_optional_flag(self):
        args = build.arguments(Path("/input"), Path("/output"), "random", 17, 1.0)
        self.assertEqual(len(args), 28)
        self.assertEqual(
            args[12:20],
            ["random", "true", "true", "1.0", "true", "false", "true", "true"],
        )
        self.assertEqual(
            build.arguments("/input", "/output", "random", 17, 1.0, False),
            [*args, "false"],
        )
        self.assertEqual(
            build.arguments("/input", "/output", "random", 17, 1.0, True),
            [*args, "true"],
        )

    def test_output_protection_and_optimized_audit(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                build.fresh(Path(tmp))
            with self.assertRaises(ValueError):
                build.fresh(build.REPO / "new-mean-probe-output")
            output = Path(tmp) / "new"
            result = subprocess.run(
                [
                    sys.executable,
                    "-O",
                    str(build.ROOT / "analyze_mean_speed_probe.py"),
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


class AggregationTests(unittest.TestCase):
    def test_sampling_is_fixed_at_grid_endpoints_and_use_transitions(self):
        sides = [dict(used="false") for _ in range(600)]
        for row in sides[10:20]:
            row["used"] = "true"
        self.assertEqual(
            audit.geometry_sample_steps(sides), {1, 10, 11, 20, 21, 250, 500, 600}
        )

    def test_ordered_mean_uses_the_original_sequential_fold(self):
        values = [(1e16,), (1.0,), (-1e16,)]
        self.assertEqual(audit.ordered_mean(values, 0), 0)

    def test_invalid_native_boolean_and_nonfinite_numbers_fail(self):
        for value in ("True", "0", ""):
            with self.assertRaises(ValueError):
                audit.truth(value)
        for value in ("nan", "inf", "-inf"):
            with self.assertRaises(ValueError):
                audit.finite(value)

    def test_summary_retains_gains_losses_and_empty_denominators(self):
        template = dict(
            tau=0.0,
            mode="random",
            raw_held="false",
            mean_held="true",
            raw_collision="false",
            mean_collision="false",
            raw_boundary="false",
            mean_boundary="false",
            raw_status="timeout",
            mean_status="held",
            raw_common_paused_s=5.0,
            mean_common_paused_s=1.0,
            raw_goal_distance="4.0",
            mean_goal_distance="0.1",
            first_difference_step=10,
            mean_used_steps=20,
            outcome="gained_hold",
        )
        loss = dict(
            template,
            raw_held="true",
            mean_held="false",
            raw_status="held",
            mean_status="timeout",
            outcome="lost_hold",
        )
        cells = audit.summarize([template, loss])
        total = cells[0]
        self.assertEqual(
            (total["pairs"], total["gained_hold"], total["lost_hold"]), (2, 1, 1)
        )
        self.assertEqual((total["raw_held"], total["mean_held"]), (1, 1))
        self.assertEqual(total["mean_used_steps"], 40)
        empty = next(
            r
            for r in cells
            if r["noise_selector"] == "correlated" and r["mode_selector"] == "all"
        )
        self.assertEqual(empty["pairs"], 0)
        self.assertEqual(empty["raw_mean_goal_distance"], "")


class NativeSmokeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if shutil.which("rustc") is None or shutil.which("cargo") is None:
            raise RuntimeError(
                "Activate the Rust toolchain for generated-program tests"
            )
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.candidate, cls.original, cls.receipt = build.build(cls.root / "build")
        fixtures = cls.root / "fixtures"
        fixtures.mkdir()
        with (fixtures / "paths.csv").open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(("scenario", "planner", "index", "x", "y"))
            for scenario, (x, y) in audit.GOALS.items():
                for planner in ("A*", "JPS", "Theta*"):
                    writer.writerow((scenario, planner, 0, x - 0.15, y))
                    writer.writerow((scenario, planner, 1, x, y))
        with (fixtures / "obstacles.csv").open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(("scenario", "x", "y"))
            for scenario in audit.GOALS:
                writer.writerow((scenario, 0, 0))
        for name, binary, enabled in (
            ("original", cls.original, None),
            ("disabled", cls.candidate, False),
            ("enabled", cls.candidate, True),
            ("omitted", cls.candidate, None),
        ):
            target = cls.root / name
            target.mkdir()
            with (target / "simulation.log").open("w") as log:
                subprocess.run(
                    [
                        str(binary),
                        *build.arguments(fixtures, target, "none", 17, 0.0, enabled),
                    ],
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
        track = [
            r
            for r in audit.read(directory / "trajectories.csv")
            if (r["scenario"], r["planner"]) == key
        ]
        sides = [
            r
            for r in audit.read(directory / "speed-probes.csv")
            if (r["scenario"], r["planner"]) == key
        ]
        outcome = next(
            r
            for r in audit.read(directory / "results.csv")
            if (r["scenario"], r["planner"]) == key
        )
        meta = dict(
            tau=0.0,
            arm="mean_probe" if name == "enabled" else "raw_probe",
            mode="none",
            seed=17,
            scenario=key[0],
            planner=key[1],
        )
        return meta, track, sides, outcome

    def test_native_tests_and_disabled_omitted_original_bytes_match(self):
        self.assertEqual(self.receipt["native_tests"], 24)
        for filename in ("trajectories.csv", "results.csv"):
            expected = (self.root / "original" / filename).read_bytes()
            for name in ("disabled", "enabled", "omitted"):
                with self.subTest(name=name, filename=filename):
                    self.assertEqual(
                        (self.root / name / filename).read_bytes(), expected
                    )
        self.assertEqual(
            (self.root / "disabled/speed-probes.csv").read_bytes(),
            (self.root / "omitted/speed-probes.csv").read_bytes(),
        )
        self.assertEqual(len(audit.read(self.root / "enabled/trajectories.csv")[0]), 82)

    def test_generated_sidecar_passes_complete_native_case_audit(self):
        for name in ("disabled", "enabled"):
            meta, track, sides, outcome = self.case(name)
            samples = []
            case, checked = audit.audit_track(
                meta, track, sides, outcome, [(1, 0.0, 0.0)], samples
            )
            self.assertGreater(checked["physical_steps"], 500)
            self.assertEqual(checked["sidecar_decisions"], len(sides))
            self.assertEqual(case["status"], "held")
            self.assertEqual(case["mean_used_steps"], 0)
            self.assertGreater(len(samples), 1)

    def test_audit_rejects_corrupted_policy_inputs_and_physics(self):
        meta, track, sides, outcome = self.case("enabled")
        for key, value in (
            ("used", "true"),
            ("mean_x", "1.0"),
            ("probe_x", "1.0"),
            ("margin", "0.1"),
            ("true_v", "0.5"),
            ("applied_acceleration", "1.0"),
        ):
            changed = copy.deepcopy(sides)
            changed[0][key] = value
            with self.subTest(field=key), self.assertRaises(ValueError):
                audit.audit_track(meta, track, changed, outcome, [(1, 0.0, 0.0)], [])

    def test_audit_rejects_wrong_native_outcome_and_sampled_geometry(self):
        meta, track, sides, outcome = self.case("enabled")
        with self.assertRaises(ValueError):
            audit.audit_track(
                meta, track, sides, dict(outcome, held="false"), [(1, 0.0, 0.0)], []
            )
        wrong = dict(
            free_distance=0.0, first_unsafe_step=0, wall_ids="x_min", obstacle_ids=""
        )
        samples = []
        with mock.patch.object(
            audit, "probe", return_value=wrong
        ), self.assertRaisesRegex(ValueError, "Sampled geometry differs"):
            audit.audit_track(meta, track, sides, outcome, [(1, 0.0, 0.0)], samples)
        self.assertEqual(len(samples), 1)

    def test_no_loss_pair_and_latent_corruption_detection(self):
        raw_meta, raw, raw_sides, raw_outcome = self.case("disabled")
        meta, candidate, mean_sides, mean_outcome = self.case("enabled")
        raw_case, _ = audit.audit_track(
            raw_meta, raw, raw_sides, raw_outcome, [(1, 0.0, 0.0)], []
        )
        mean_case, _ = audit.audit_track(
            meta, candidate, mean_sides, mean_outcome, [(1, 0.0, 0.0)], []
        )
        pair = audit.pair(
            meta, raw, candidate, raw_sides, mean_sides, raw_case, mean_case
        )
        self.assertEqual(pair["first_difference_step"], "")
        self.assertEqual(pair["outcome"], "both_held")
        corrupted = copy.deepcopy(candidate)
        corrupted[1]["noise_innovation_x"] = "10"
        with self.assertRaises(ValueError):
            audit.pair(meta, raw, corrupted, raw_sides, mean_sides, raw_case, mean_case)


if __name__ == "__main__":
    unittest.main()
