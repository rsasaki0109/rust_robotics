"""Check full native clearances, exact branch order and continuous release runs."""

import copy
import csv
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from collections import Counter

import analyze_guard_release_census as audit
import analyze_mean_speed_probe as pilot
import build_mean_guard_preview as build
import native_guard_release_census as native
from verify_reset_mean_probe import source_block


def states(**changes):
    return dict.fromkeys(audit.GATES, True) | changes


class TimerTests(unittest.TestCase):
    def test_joint_requires_fifty_steps_and_releases_guard_atomically(self):
        machine = audit.ReleaseState()
        for step in range(1, 50):
            result = machine.step(True, states(), 0.0, 0.5)
            self.assertFalse(result["resumed"])
            self.assertTrue(result["active"])
            self.assertEqual(result["guard_timer"], 0)
            self.assertAlmostEqual(result["joint_timer"], step * 0.02)
        result = machine.step(True, states(), 0.0, 0.5)
        self.assertTrue(result["resumed"] and result["released"])
        self.assertFalse(result["active"] or result["paused"])
        self.assertEqual(result["joint_timer"], 0)

    def test_joint_resume_does_not_invent_a_guard_release(self):
        machine = audit.ReleaseState()
        for _ in range(50):
            result = machine.step(True, states(), 10.0, 0.5)
        self.assertTrue(result["resumed"])
        self.assertFalse(result["released"])

    def test_guard_entry_skips_update_and_zero_target_can_release_after_twentyfive(
        self,
    ):
        machine = audit.ReleaseState()
        result = machine.step(False, states(nominal=False), 0.5, 0.5)
        self.assertTrue(result["triggered"])
        self.assertFalse(result["guard_updated"])
        self.assertEqual(result["guard_timer"], 0)
        for _ in range(24):
            result = machine.step(False, states(nominal=False), 10.0, 0.5)
            self.assertFalse(result["released"])
        result = machine.step(False, states(nominal=False), 10.0, 0.5)
        self.assertTrue(result["released"])
        self.assertFalse(result["active"] or result["resumed"])
        self.assertEqual(result["guard_timer"], 0)

    def test_each_joint_failure_resets_continuity(self):
        for gate in audit.GATES:
            machine = audit.ReleaseState()
            for _ in range(49):
                machine.step(True, states(), 10.0, 0.5)
            result = machine.step(True, states(**{gate: False}), 10.0, 0.5)
            self.assertEqual(result["joint_timer"], 0)
            self.assertFalse(machine.step(True, states(), 10.0, 0.5)["resumed"])

    def test_joint_pause_clears_guard_evidence(self):
        machine = audit.ReleaseState()
        machine.step(False, states(), 0.0, 0.5)
        for _ in range(24):
            machine.step(False, states(), 10.0, 0.5)
        result = machine.step(True, states(nominal=False), 10.0, 0.5)
        self.assertEqual(result["guard_timer"], 0)
        self.assertFalse(result["guard_updated"] or result["released"])

    def test_runs_keep_failures_joint_interruptions_and_endpoint(self):
        output = []
        runs = audit.Runs({}, "guard_only", output)
        runs.step(1, True, True, False, "", "inactive")
        runs.step(2, True, False, False, "release", "inactive")
        runs.step(3, True, True, False, "", "inactive")
        runs.step(4, False, True, False, "", "joint_pause")
        runs.step(5, True, True, False, "", "inactive")
        runs.close("packet_endpoint")
        self.assertEqual(
            [r["ending"] for r in output], ["failure", "joint_pause", "packet_endpoint"]
        )
        self.assertEqual(
            (runs.count, runs.maximum, runs.resets, runs.interruptions), (3, 1, 1, 1)
        )


def query(
    ident,
    scenario="line",
    *,
    mean_x=10.0,
    target=0.4,
    steering=0.0,
    age=0.0,
    precision=False,
    sigma=0.0,
    drift=0.0
):
    effective = 0.03 if precision else sigma
    margin = 0.5 + 0.1 + 2 * math.sqrt(2) * effective + drift * age
    return dict(
        zip(
            native.INPUT_FIELDS,
            (
                ident,
                scenario,
                mean_x,
                10.0,
                0.0,
                margin,
                age,
                str(precision).lower(),
                sigma,
                drift,
                "false",
                10.0,
                10.0,
                0.0,
                0.0,
                target,
                steering,
            ),
        )
    )


def logged(geometry):
    return {
        old: geometry[new]
        for new, old in (
            ("recovery_clearance_9", "recovery_clearance"),
            ("preview_margin_9", "preview_margin"),
            ("release_margin_9", "guard_release_margin"),
            ("stop_9", "stop_preview_clearance_lower_bound"),
            ("terminal_9", "terminal_recovery_clearance"),
            ("entry_9", "preview_clearance_lower_bound"),
            ("release_9", "release_preview_clearance_lower_bound"),
            ("effective_position_sigma_9", "effective_position_sigma"),
        )
    }


class NativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if shutil.which("rustc") is None:
            raise RuntimeError("Activate Rust for release census tests")
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.map = cls.root / "maps.csv"
        cls.map.write_text("scenario,x,y\nline,11.22,10\nsafe,30,30\nturn,11.2,10.8\n")
        cls.queries = [
            query(1),
            query(2, target=0),
            query(3, "safe", mean_x=0.6),
            query(4, "safe", mean_x=math.nextafter(0.6, -math.inf)),
            query(5, "safe", age=1.5, precision=True, sigma=0.1, drift=0.02),
            query(6, "turn", steering=0.35),
            query(7, "turn", steering=-0.35),
            query(8, "safe"),
        ]
        cls.receipts = []
        for name in ("one", "two"):
            work = cls.root / name
            cls.receipts.append(native.compile_native(work))
            with (work / "queries.tsv").open("x", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=native.INPUT_FIELDS,
                    delimiter="\t",
                    lineterminator="\n",
                )
                writer.writeheader()
                writer.writerows(cls.queries)
            native.evaluate(work, work / "queries.tsv", cls.map)
        cls.trace = pilot.read(cls.root / "one/trace.csv", native.TRACE_FIELDS)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_original_functions_and_both_binaries_are_identical(self):
        original = build.BASE.read_text()
        source = native.source()
        for marker in (
            "fn stopping_preview(",
            "fn terminal_arc_clearance(",
            "fn aligned_release_preview(",
            "fn point_segment_distance(",
        ):
            self.assertIn(source_block(original, marker), source)
        for name in (
            "release-census.rs",
            "native-release-census",
            "trace.csv",
            "queries.tsv",
            "compiler.log",
            "native.log",
        ):
            self.assertEqual(
                (self.root / "one" / name).read_bytes(),
                (self.root / "two" / name).read_bytes(),
            )

    def test_positive_target_from_rest_changes_terminal_clearance(self):
        positive, zero = self.trace[:2]
        self.assertLess(float(positive["release"]), float(positive["release_margin"]))
        self.assertGreater(float(zero["release"]), float(zero["release_margin"]))
        self.assertAlmostEqual(float(zero["recovery_clearance"]), 0.62)

    def test_native_bounds_include_equality_and_margins_keep_separate_buffers(self):
        self.assertEqual(
            [r["recovery_bounds"] for r in self.trace[2:4]], ["true", "false"]
        )
        row = self.trace[4]
        self.assertAlmostEqual(
            float(row["recovery_margin"]) - float(row["preview_margin"]), 0.1
        )
        expected = 0.5 + 0.0 + 2 * math.sqrt(2) * 0.03 + 0.02 * 1.5 + 0.05
        self.assertEqual(float(row["release_margin"]), expected)

    def test_curved_terminal_uses_the_actual_steering(self):
        self.assertNotEqual(self.trace[5]["terminal"], self.trace[6]["terminal"])

    def test_native_reader_rejects_reordered_nonfinite_and_wrong_margin(self):
        for field, value in (
            ("id", 2),
            ("preview_x", "nan"),
            ("speed_margin", 0.5),
            ("precision_active", "True"),
        ):
            rows = copy.deepcopy(self.queries)
            rows[0][field] = value
            packet = self.root / (field + ".tsv")
            with packet.open("w", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=native.INPUT_FIELDS,
                    delimiter="\t",
                    lineterminator="\n",
                )
                writer.writeheader()
                writer.writerows(rows)
            with (self.root / (field + ".log")).open("w") as log:
                result = subprocess.run(
                    [
                        str(self.root / "one/native-release-census"),
                        str(self.map),
                        str(packet),
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
            self.assertNotEqual(result.returncode, 0)

    def test_gate_thresholds_and_corrupted_logged_clearance(self):
        geometry = dict(self.trace[1])
        row = logged(geometry)
        side = dict(
            observation_age="1.5",
            measured_speed="-0.02",
            count="10",
            nominal_target="0.4",
            margin=geometry["recovery_margin"],
        )
        self.assertTrue(all(audit.gates(row, side, geometry).values()))
        for key, value, gate in (
            ("observation_age", str(math.nextafter(1.5, math.inf)), "fresh"),
            ("measured_speed", str(math.nextafter(0.02, math.inf)), "stationary"),
            ("count", "9", "samples"),
            ("nominal_target", "0", "nominal"),
        ):
            self.assertFalse(
                audit.gates(row, dict(side, **{key: value}), geometry)[gate]
            )
        for field, threshold, gate in (
            ("recovery_clearance", "recovery_margin", "clearance"),
            ("release", "release_margin", "release"),
        ):
            changed = dict(geometry, **{field: geometry[threshold]})
            self.assertFalse(audit.gates(row, side, changed)[gate])
        with self.assertRaisesRegex(ValueError, "Native geometry differs"):
            audit.gates(
                dict(row, terminal_recovery_clearance="0.000000000"), side, geometry
            )

    def synthetic_case(self, phase):
        meta = dict(
            case_id="synthetic",
            tau=0,
            arm="mean_preview",
            mode="random" if phase == "joint" else "none",
            seed=17,
            scenario="safe" if phase == "joint" else "line",
            planner="A*",
        )
        track = [dict(safety_paused="false")]
        sides, guards, geometries = [], [], []
        count = 51 if phase == "joint" else 26
        for step in range(1, count + 1):
            pending = phase == "joint" and step <= 50
            geom = dict(
                (
                    self.trace[7]
                    if phase == "joint"
                    else self.trace[0] if step == 1 else self.trace[1]
                ),
                id=str(step),
            )
            row = logged(geom) | dict(
                time_s=str(step * 0.02),
                correction=str(phase == "joint" and step == 1).lower(),
                correction_received="false",
                guard_triggered=str(phase != "joint" and step == 1).lower(),
                guard_released=str(phase != "joint" and step == 26).lower(),
                guard_active=str(phase != "joint" and step < 26).lower(),
                resumed=str(phase == "joint" and step == 50).lower(),
                safety_paused=str(pending and step < 50).lower(),
                recovery_evidence=str(step * 0.02 if pending and step < 50 else 0),
                guard_release_evidence=str(
                    (step - 1) * 0.02 if phase != "joint" and step < 26 else 0
                ),
                stop_latched=str(phase != "joint" and step > 1).lower(),
                speed_braking="false",
                hold_time_s="0",
            )
            sides.append(
                dict(
                    step=str(step),
                    time_s=row["time_s"],
                    paused=str(pending).lower(),
                    used=str(pending).lower(),
                    observation_age="0",
                    measured_speed="0",
                    count="10" if pending else "0",
                    nominal_target="0.4" if phase == "joint" or step == 1 else "0",
                    margin=geom["recovery_margin"],
                    uncapped_target="0.4" if phase == "joint" or step == 1 else "0",
                    free_distance="2",
                )
            )
            guards.append(dict(step=str(step), used=str(pending).lower()))
            track.append(row)
            geometries.append(geom)
        old = dict(
            decisions=count,
            speed_used_steps=50 if phase == "joint" else 0,
            guard_used_steps=50 if phase == "joint" else 0,
            resume_count=1 if phase == "joint" else 0,
            pause_count=1 if phase == "joint" else 0,
            paused_time_s=0.98 if phase == "joint" else 0,
            status="held",
            held="true",
        )
        return meta, track, sides, guards, old, geometries

    def test_complete_joint_and_guard_logs_match_independent_known_timers(self):
        for phase in ("joint", "guard"):
            meta, track, sides, guards, old, geometries = self.synthetic_case(phase)
            patterns, runs, events = Counter(), [], []
            case, checked = audit.diagnose(
                meta,
                track,
                sides,
                guards,
                old,
                iter(geometries),
                [0],
                patterns,
                runs,
                events,
            )
            self.assertEqual(checked["geometry_decisions"], len(sides))
            if phase == "joint":
                self.assertEqual(
                    (case["joint_resumes"], case["joint_max_ready_run_steps"]), (1, 50)
                )
            else:
                self.assertEqual(
                    (case["guard_releases"], case["guard_max_ready_run_steps"]), (1, 25)
                )
            self.assertEqual(runs[0]["ending"], "admission")

    def test_census_rejects_wrong_timer_flags_and_native_join(self):
        data = self.synthetic_case("joint")
        for field, value in (
            ("recovery_evidence", "0.03"),
            ("resumed", "true"),
            ("guard_released", "true"),
            ("safety_paused", "false"),
        ):
            meta, track, sides, guards, old, geometries = copy.deepcopy(data)
            track[1][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                audit.diagnose(
                    meta,
                    track,
                    sides,
                    guards,
                    old,
                    iter(geometries),
                    [0],
                    Counter(),
                    [],
                    [],
                )
        meta, track, sides, guards, old, geometries = copy.deepcopy(data)
        geometries[0]["id"] = "2"
        with self.assertRaises(ValueError):
            audit.diagnose(
                meta,
                track,
                sides,
                guards,
                old,
                iter(geometries),
                [0],
                Counter(),
                [],
                [],
            )

    def test_summaries_keep_all_outcomes_and_empty_cells_and_pairs(self):
        meta, track, sides, guards, old, geometries = self.synthetic_case("joint")
        case, _ = audit.diagnose(
            meta, track, sides, guards, old, iter(geometries), [0], Counter(), [], []
        )
        summary = audit.summarize([case])
        self.assertEqual(len(summary), 72)
        self.assertTrue(
            any(c["outcome_selector"] == "timeout" and c["cases"] == 0 for c in summary)
        )
        reference = dict(case, arm="fused_preview")
        self.assertEqual(len(audit.paired([reference, case])), 1)
        with self.assertRaises(ValueError):
            audit.paired([case])


class ProtectionTests(unittest.TestCase):
    def test_frozen_inputs_and_population(self):
        pins = audit.preflight()
        self.assertEqual(
            (pins["expected_cases"], pins["expected_decisions"]), (108, 799688)
        )

    def test_optimized_cli_existing_and_internal_outputs_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            for path in (Path(tmp), build.REPO / "forbidden-census-output"):
                with self.assertRaises(ValueError):
                    build.fresh(path)
            output = Path(tmp) / "new"
            result = subprocess.run(
                [
                    sys.executable,
                    "-O",
                    str(build.ROOT / "analyze_guard_release_census.py"),
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


if __name__ == "__main__":
    unittest.main()
