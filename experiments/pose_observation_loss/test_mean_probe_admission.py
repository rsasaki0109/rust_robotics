"""Exercise native geometry, uncertain gates and the continuous admission timer."""

from collections import Counter
import copy
import csv
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import analyze_mean_probe_admission as audit
import build_mean_speed_probe as build
import native_mean_probe_admission as native
from verify_reset_mean_probe import source_block


class IntervalAndTimerTests(unittest.TestCase):
    def test_strict_release_uses_interval_and_keeps_equal_center_unresolved(self):
        self.assertIs(audit.release_gate("0.650000001", 0.65), True)
        self.assertIs(audit.release_gate("0.649999999", 0.65), False)
        self.assertIsNone(audit.release_gate("0.650000000", 0.65))
        for value in ("NaN", "Infinity"):
            with self.assertRaises(ValueError):
                audit.release_gate(value, 0.65)

    def test_a_failed_gate_decides_conjunction_despite_an_unresolved_gate(self):
        states = dict.fromkeys(audit.GATES, True)
        states["release"] = None
        self.assertIsNone(audit.conjunction(states))
        states["samples"] = states["nominal"] = False
        self.assertIs(audit.conjunction(states), False)
        self.assertEqual(audit.masks(states), ("samples;nominal", "release"))

    def test_49_steps_do_not_admit_and_50_steps_admit_atomically(self):
        timer = audit.Timer()
        for step in range(1, 50):
            self.assertTrue(timer.step(True, False, f"{step * 0.02:.6f}", True))
        self.assertAlmostEqual(timer.evidence, 0.98)
        self.assertTrue(timer.step(True, True, "0.000000", False))
        self.assertEqual(timer.evidence, 0)
        with self.assertRaises(ValueError):
            audit.Timer().step(True, True, "0.000000", False)

    def test_failure_resets_the_full_continuous_requirement(self):
        timer = audit.Timer()
        for step in range(1, 50):
            timer.step(True, False, f"{step * 0.02:.6f}", True)
        timer.step(False, False, "0.000000", True)
        timer.step(True, False, "0.020000", True)
        self.assertAlmostEqual(timer.evidence, 0.02)
        with self.assertRaises(ValueError):
            timer.step(True, False, "0.980000", True)

    def test_unknown_gate_does_not_borrow_readiness_from_the_native_timer(self):
        timer = audit.Timer()
        self.assertFalse(timer.step(None, False, "0.000000", True))
        self.assertIsNone(timer.evidence)
        self.assertFalse(timer.step(True, False, "0.020000", True))
        self.assertIsNone(timer.evidence)
        self.assertTrue(timer.step(False, False, "0.000000", True))
        self.assertTrue(timer.step(True, False, "0.020000", True))
        self.assertFalse(timer.step(None, True, "0.000000", False))
        self.assertEqual(timer.evidence, 0)

    def test_optimized_cli_rejects_before_creating_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "output"
            result = subprocess.run(
                [
                    sys.executable,
                    "-O",
                    str(build.ROOT / "analyze_mean_probe_admission.py"),
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


class NativeAndCaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if shutil.which("rustc") is None:
            raise RuntimeError(
                "Activate the Rust compiler for admission geometry tests"
            )
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.map = cls.root / "obstacles.csv"
        cls.map.write_text("scenario,x,y\ns,10,10\n")
        cls.queries = [
            (11.0, 10.0, 0.0, 0.6),
            (10.5, 10.0, 0.0, 0.6),
            (9.8, 10.0, 0.0, 0.6),
            (0.6, 20.0, 0.0, 0.6),
            (math.nextafter(0.6, -math.inf), 20.0, 0.0, 0.6),
            (49.4, 20.0, 0.0, 0.6),
            (10.0, 9.0, math.pi / 2, 0.6),
        ]
        cls.receipts = []
        for name in ("native-one", "native-two"):
            work = cls.root / name
            receipt = native.compile_geometry(work)
            with (work / "queries.tsv").open("w", newline="") as stream:
                writer = csv.writer(stream, delimiter="\t", lineterminator="\n")
                writer.writerow(("id", "scenario", "x", "y", "yaw", "margin"))
                writer.writerows(
                    (i, "s", *(format(v, ".17e") for v in values))
                    for i, values in enumerate(cls.queries, 1)
                )
            cls.receipts.append(
                native.evaluate(
                    work, receipt, work / "queries.tsv", len(cls.queries), cls.map
                )
            )
        cls.rows = audit.pilot.read(
            cls.root / "native-one/trace.csv", native.TRACE_FIELDS
        )

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_native_source_preserves_the_original_segment_and_line_body(self):
        original = build.BASE.read_text()
        source = native.source()
        self.assertIn(source_block(original, "fn point_segment_distance("), source)
        self.assertIn("let preview_end = Point2D::new(", source)
        self.assertIn(
            "(recovery_margin..=(50.0 - recovery_margin)).contains(p)", source
        )
        self.assertIn("let recovery_margin = margin;", source)
        self.assertNotIn("effective_position_sigma", source)

    def test_native_geometry_covers_endpoint_interior_turn_and_inclusive_bounds(self):
        clearances = [float(r["recovery_clearance"]) for r in self.rows]
        self.assertEqual(clearances[:3], [1.0, 0.5, 0.0])
        self.assertAlmostEqual(clearances[6], 0.4)
        self.assertEqual(
            [r["recovery_bounds"] for r in self.rows],
            ["true", "true", "true", "true", "false", "false", "true"],
        )
        self.assertTrue(
            all(float(r["release_margin"]) == 0.6 + 0.05 for r in self.rows)
        )

    def test_fresh_build_paths_keep_source_queries_trace_binary_and_receipt_identical(
        self,
    ):
        for name in (
            "admission-geometry.rs",
            "queries.tsv",
            "trace.csv",
            "native-geometry",
            "native-sources.json",
        ):
            with self.subTest(name=name):
                self.assertEqual(
                    (self.root / "native-one" / name).read_bytes(),
                    (self.root / "native-two" / name).read_bytes(),
                )

    def inputs(self):
        row = dict(
            recovery_clearance="1.000000000",
            guard_release_margin="0.650000000",
            recovery_sample_count="10",
            release_preview_clearance_lower_bound="1.000000000",
        )
        side = dict(
            margin="0.6",
            observation_age="1.5",
            measured_speed="0.02",
            count="10",
            nominal_target="0.4",
        )
        return row, side, copy.deepcopy(self.rows[0])

    def test_inclusive_age_speed_samples_and_strict_clearance_nominal(self):
        row, side, geometry = self.inputs()
        self.assertTrue(all(audit.gates(row, side, geometry).values()))
        for name, value, gate in (
            ("observation_age", math.nextafter(1.5, math.inf), "fresh"),
            ("measured_speed", math.nextafter(0.02, math.inf), "stationary"),
        ):
            changed = dict(side, **{name: str(value)})
            self.assertFalse(audit.gates(row, changed, geometry)[gate])
        row["recovery_sample_count"], side["count"] = "9", "9"
        side["nominal_target"] = "0"
        geometry["recovery_clearance"] = "0.6"
        geometry["recovery_clearance_9"] = row["recovery_clearance"] = "0.600000000"
        states = audit.gates(row, side, geometry)
        self.assertEqual(audit.masks(states), ("samples;clearance;nominal", ""))

    def test_corrupted_native_identity_clearance_and_margin_are_rejected(self):
        row, side, geometry = self.inputs()
        for name in ("recovery_clearance_9", "release_margin_9", "release_margin"):
            changed = dict(geometry, **{name: "9.000000000"})
            with self.subTest(name=name), self.assertRaises(ValueError):
                audit.gates(row, side, changed)

    def synthetic_case(self):
        meta = dict(
            case_id="C0001",
            tau=0.0,
            arm="mean_probe",
            mode="random",
            seed=17,
            scenario="s",
            planner="P",
        )
        track = [dict(scenario="s", planner="P", time_s="0", safety_paused="false")]
        sides, geometries = [], []
        for step in range(1, 61):
            pending, ready, resumed = step <= 59, 10 <= step <= 59, step == 59
            side = dict(
                scenario="s",
                planner="P",
                step=str(step),
                time_s=str(step * 0.02),
                paused=str(pending).lower(),
                used=str(ready).lower(),
                count=str(min(step, 25)),
                margin="0.6",
                observation_age=str(step * 0.02),
                measured_speed="0",
                nominal_target="0.4",
                uncapped_target="0.4",
                free_distance="2.0",
            )
            row = dict(
                scenario="s",
                planner="P",
                time_s=side["time_s"],
                correction=str(step == 1).lower(),
                correction_received="false",
                resumed=str(resumed).lower(),
                safety_paused=str(step < 59).lower(),
                recovery_evidence=f"{(step - 9) * 0.02 if ready and not resumed else 0:.6f}",
                recovery_clearance="1.000000000",
                recovery_sample_count=side["count"],
                guard_release_margin="0.650000000",
                release_preview_clearance_lower_bound="1.000000000",
                stop_latched="false",
                guard_active="false",
                speed_braking="false",
                hold_time_s="0.000000",
            )
            sides.append(side)
            track.append(row)
            if pending:
                geometries.append(dict(self.rows[0], id=str(step)))
        outcome = dict(
            status="timeout",
            held="false",
            collision="false",
            boundary="false",
            time_s="1.2",
            goal_distance="1",
            decisions="60",
            mean_used_steps="50",
        )
        return meta, track, sides, outcome, geometries

    def diagnose(self, packet):
        meta, track, sides, outcome, geometries = copy.deepcopy(packet)
        patterns, runs, count = Counter(), [], [0]
        case = audit.diagnose_case(
            meta, track, sides, outcome, iter(geometries), count, patterns, runs
        )
        return case, patterns, runs, count

    def test_complete_case_retains_warmup_and_one_50_step_admission(self):
        case, patterns, runs, count = self.diagnose(self.synthetic_case())
        self.assertEqual(
            (case["pending_steps"], case["eligible_steps"], case["resumed"]),
            (59, 50, 1),
        )
        self.assertEqual(
            (case["timer_verified_steps"], case["timer_unverified_steps"]), (59, 0)
        )
        self.assertEqual(
            (
                runs[0]["first_step"],
                runs[0]["last_step"],
                runs[0]["steps"],
                runs[0]["ending"],
            ),
            (10, 59, 50, "admission"),
        )
        self.assertEqual(
            sum(v for k, v in patterns.items() if k[-3] == "pending_all"), 59
        )
        self.assertEqual(count, [59])
        cells = audit.summarize([case])
        empty = next(r for r in cells if r["noise_selector"] == "correlated")
        self.assertEqual((empty["cases"], empty["max_ready_run_steps"]), (0, ""))

    def test_full_case_rejects_reordered_queries_and_corrupted_timer_or_pause(self):
        for kind in ("query", "timer", "pause", "nominal"):
            packet = self.synthetic_case()
            if kind == "query":
                packet[4][0]["id"] = "2"
            if kind == "timer":
                packet[1][20]["recovery_evidence"] = "0.980000"
            if kind == "pause":
                packet[2][19]["paused"] = "false"
            if kind == "nominal":
                packet[2][19]["nominal_target"] = "0"
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                self.diagnose(packet)

    def test_case_preserves_a_rounding_ambiguity_and_unverified_timer_suffix(self):
        packet = self.synthetic_case()
        packet[1][20]["release_preview_clearance_lower_bound"] = "0.650000000"
        case, _, runs, _ = self.diagnose(packet)
        self.assertEqual(
            (case["release_ambiguous_steps"], case["timer_unverified_steps"]), (1, 40)
        )
        self.assertEqual(
            [(r["steps"], r["ending"], r["last_timer_verified"]) for r in runs],
            [(10, "ambiguity", "true"), (39, "admission", "false")],
        )
        self.assertEqual(runs[0]["unresolved_gates"], "release")

    def test_case_retains_every_failed_run_and_packet_endpoint(self):
        packet = self.synthetic_case()
        _, track, sides, outcome, geometries = packet
        sides[19]["nominal_target"] = "0"
        sides[19]["free_distance"] = "0"
        for step in range(20, 61):
            track[step]["recovery_evidence"] = f"{(step - 20) * 0.02:.6f}"
            track[step]["resumed"] = "false"
            track[step]["safety_paused"] = "true"
        sides[-1]["paused"] = sides[-1]["used"] = "true"
        geometries.append(dict(self.rows[0], id="60"))
        outcome["mean_used_steps"] = "51"
        case, _, runs, _ = self.diagnose(packet)
        self.assertEqual(
            [(r["steps"], r["ending"]) for r in runs],
            [(10, "failure"), (40, "packet_endpoint")],
        )
        self.assertEqual(runs[0]["failed_gates"], "nominal")
        self.assertEqual(
            (case["resets"], case["max_ready_run_steps"], case["resumed"]), (1, 40, 0)
        )
        self.assertEqual(case["eligible_unlatched_probe_zero"], 1)


if __name__ == "__main__":
    unittest.main()
