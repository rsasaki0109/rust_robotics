"""Check original geometry, witness ties, conservative intervals and selection."""

import copy
from decimal import Decimal
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import analyze_probe_geometry as audit
import build_mean_speed_probe as build
import native_probe_geometry as native
import verify_reset_mean_probe as original


class IntervalAndSelectionTests(unittest.TestCase):
    def test_outward_enclosure_contains_the_exact_decimal_endpoints(self):
        for text in ("0.000000000", "0.650000000", "-1.000000000", "1.234567890"):
            lo, hi = audit.rounding_box(text)
            self.assertLessEqual(
                Decimal.from_float(lo), Decimal(text) - Decimal("0.0000000005")
            )
            self.assertGreaterEqual(
                Decimal.from_float(hi), Decimal(text) + Decimal("0.0000000005")
            )
        for text in ("nan", "inf", "-inf"):
            with self.assertRaises(ValueError):
                audit.rounding_box(text)

    def test_strict_interval_comparison_retains_threshold_uncertainty(self):
        self.assertEqual(audit.interval_gate(0.6, 0.65, 0.65), "fail")
        self.assertEqual(
            audit.interval_gate(math.nextafter(0.65, math.inf), 0.7, 0.65), "pass"
        )
        self.assertEqual(audit.interval_gate(0.65, 0.7, 0.65), "unresolved")
        with self.assertRaises(ValueError):
            audit.interval_gate(0.7, 0.6, 0.65)

    def test_representatives_keep_controls_merge_roles_and_break_ties_by_order(self):
        rows = [
            dict(id=str(i), case_id=case, mask=mask, recovery_gap=gap)
            for i, (case, mask, gap) in enumerate(
                (
                    ("C1", "ready", 0.3),
                    ("C1", "ready", 0.1),
                    ("C1", "ready", 0.1),
                    ("C1", "release", 0.2),
                    ("C2", "clearance", -0.1),
                    ("C2", "clearance;release", -0.2),
                ),
                1,
            )
        ]
        reps = audit.representatives(rows)
        self.assertEqual([r["id"] for r in reps], ["1", "2", "4", "5", "6"])
        self.assertEqual(reps[0]["roles"], "first")
        self.assertEqual(reps[1]["roles"], "minimum_recovery_gap")
        self.assertEqual(reps[2]["roles"], "first;minimum_recovery_gap")
        self.assertEqual(
            [r["id"] for r in reps if r["plot"] == "true"], ["4", "5", "6"]
        )

    def test_empty_cases_and_complete_summary_cells_are_preserved(self):
        meta = dict(
            case_id="C1",
            tau="0",
            arm="mean_probe",
            mode="none",
            seed="1001",
            scenario="s",
            planner="p",
        )
        cases, summary = audit.summaries([meta], [], {}, {})
        self.assertEqual(len(cases), 1)
        self.assertEqual(cases[0]["selected_decisions"], 0)
        self.assertEqual(len(summary), 48)
        self.assertTrue(
            all(
                r["selected_decisions"] == 0 and r["selected_cases"] == 0
                for r in summary
            )
        )
        self.assertEqual(
            sum(
                r["cases"]
                for r in summary
                if r["noise_selector"] == "all" and r["mode_selector"] == "all"
            ),
            4,
        )

    def test_previous_masks_reject_omitted_controls_and_preserve_zero_cases(self):
        cases = [dict(case_id="C1"), dict(case_id="C2")]
        old = [
            dict(case_id="C1", arm="mean_probe", eligible_nominal_positive="2"),
            dict(case_id="C2", arm="mean_probe", eligible_nominal_positive="0"),
        ]
        rows = [
            dict(case_id="C1", mask="ready"),
            dict(case_id="C1", mask="clearance;release"),
        ]
        patterns = [
            dict(
                case_id="C1",
                arm="mean_probe",
                phase="mean_eligible",
                failed_gates=mask,
                unresolved_gates="",
                steps="1",
            )
            for mask in ("", "clearance;release", "nominal")
        ]
        audit.check_previous(cases, rows, old, patterns)
        for incomplete in (rows[:1], rows[1:]):
            with self.assertRaises(ValueError):
                audit.check_previous(cases, incomplete, old, patterns)
        with self.assertRaises(ValueError):
            audit.check_previous(cases[:1], rows, old, patterns)

    def test_both_optimized_clis_reject_before_output_creation(self):
        with tempfile.TemporaryDirectory() as tmp:
            for script, flag in (
                ("analyze_probe_geometry.py", "--raw"),
                ("plot_probe_geometry.py", "--data"),
            ):
                output = Path(tmp) / script
                result = subprocess.run(
                    [
                        sys.executable,
                        "-O",
                        str(build.ROOT / script),
                        flag,
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


class NativeWitnessTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if shutil.which("rustc") is None:
            raise RuntimeError("Activate Rust for native witness tests")
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.map = cls.root / "map.csv"
        cls.map.write_text("scenario,x,y\ns,10,10\ns,10,10\ns,30,30\n")
        cls.queries = []
        for i, (x, y, mean, raw, steering, age, precision, sigma, buffer) in enumerate(
            (
                (8.0, 10.0, 0.0, math.pi / 2, 0.0, 0.0, False, 0.0, False),
                (9.0, 9.8, math.pi / 2, math.pi, 0.0, 0.0, False, 0.0, False),
                (10.5, 10.0, 0.0, 0.0, 0.0, 0.0, False, 0.0, False),
                (8.0, 10.0, 0.0, 0.0, 0.5, 0.0, False, 0.0, False),
                (0.6, 20.0, 0.0, 0.0, 0.0, 0.0, False, 0.0, False),
                (11.0, 10.0, 0.0, 0.0, 0.0, 5.0, False, 0.1, False),
                (11.0, 10.0, 0.0, 0.0, 0.0, 1.0, True, 0.1, True),
            ),
            1,
        ):
            effective_sigma = 0.03 if precision else sigma
            margin = 0.5 + 0.1 + 2.0 * math.sqrt(2.0) * effective_sigma + 0.02 * age
            cls.queries.append(
                dict(
                    zip(
                        native.INPUT_FIELDS,
                        (
                            str(i),
                            "s",
                            x,
                            y,
                            mean,
                            raw,
                            steering,
                            margin,
                            age,
                            str(precision).lower(),
                            sigma,
                            0.02,
                            str(buffer).lower(),
                            1.2,
                            1.3,
                            1.0,
                            1.1,
                            "true",
                        ),
                    )
                )
            )
        cls.receipts = []
        for name in ("one", "two"):
            work = cls.root / name
            compiler = native.compile_native(work)
            audit.write_queries(work / "queries.tsv", cls.queries)
            receipt = native.evaluate(work, work / "queries.tsv", map_path=cls.map)
            cls.receipts.append(dict(compiler=compiler, evaluation=receipt))
        cls.work = cls.root / "one"
        cls.traces = audit.pilot.read(cls.work / "trace.csv", native.TRACE_FIELDS)
        cls.witnesses = audit.pilot.read(
            cls.work / "witnesses.csv", native.WITNESS_FIELDS
        )
        cls.paths = audit.pilot.read(cls.work / "paths.csv", native.PATH_FIELDS)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_generated_source_keeps_original_probe_and_release_functions(self):
        source, base = native.source(), build.BASE.read_text()
        for marker in (
            "fn speed_free_distance(",
            "fn point_segment_distance(",
            "fn aligned_release_preview(",
        ):
            self.assertIn(original.source_block(base, marker), source)
        self.assertIn(
            original.source_block(original.harness_source(), "fn at_step("), source
        )

    def test_fresh_builds_produce_identical_source_binary_queries_and_all_evidence(
        self,
    ):
        self.assertEqual(self.receipts[0], self.receipts[1])
        for name in (
            "probe-geometry.rs",
            "native-probe-geometry",
            "queries.tsv",
            "trace.csv",
            "witnesses.csv",
            "paths.csv",
        ):
            self.assertEqual(
                (self.root / "one" / name).read_bytes(),
                (self.root / "two" / name).read_bytes(),
                name,
            )

    def test_all_duplicate_nearest_witnesses_keep_global_ids_and_projection(self):
        for ident in ("1", "2", "3"):
            rows = [w for w in self.witnesses if w["id"] == ident]
            self.assertEqual([w["obstacle_id"] for w in rows], ["1", "2"])
        self.assertEqual(float(self.witnesses[0]["t"]), 1.0)
        interior = next(w for w in self.witnesses if w["id"] == "2")
        self.assertGreater(float(interior["t"]), 0)
        self.assertLess(float(interior["t"]), 1)
        endpoint = next(w for w in self.witnesses if w["id"] == "3")
        self.assertEqual(float(endpoint["t"]), 0)

    def test_heading_and_steering_alternatives_are_separate_native_queries(self):
        first, curved = self.traces[0], self.traces[3]
        self.assertEqual(float(first["actual_distance"]), 2.0)
        self.assertLess(float(first["mean_yaw_distance"]), 2.0)
        self.assertEqual(first["mean_yaw_distance"], first["straight_distance"])
        self.assertEqual(curved["actual_distance"], curved["mean_yaw_distance"])
        self.assertGreater(
            float(curved["mean_yaw_distance"]), float(curved["straight_distance"])
        )
        self.assertEqual(len(self.paths), len(self.queries) * 3 * 41)
        for query in self.queries:
            points = [
                p for p in self.paths if p["id"] == query["id"] and p["step"] == "0"
            ]
            self.assertEqual(len(points), 3)
            self.assertTrue(
                all(
                    float(p["x"]) == query["x"] and float(p["y"]) == query["y"]
                    for p in points
                )
            )

    def test_initial_contact_and_inclusive_recovery_bounds_keep_original_semantics(
        self,
    ):
        self.assertEqual(float(self.traces[2]["actual_distance"]), 0.0)
        self.assertEqual(self.traces[4]["recovery_bounds"], "true")
        self.assertEqual(float(self.traces[4]["actual_distance"]), 0.0)

    def test_release_components_use_original_subtraction_and_separate_margins(self):
        first, ordinary, precision = self.traces[0], self.traces[-2], self.traces[-1]
        self.assertEqual(float(first["terminal_release_lo"]), 1.0 - 0.05)
        self.assertEqual(first["release_lo"], first["terminal_release_lo"])
        self.assertEqual(first["release_hi"], first["terminal_release_hi"])
        self.assertEqual(ordinary["release_margin_9"], "0.932842712")
        self.assertAlmostEqual(
            float(precision["release_margin"]), float(precision["margin"]) + 0.05
        )

    def test_native_component_enclosures_preserve_fail_pass_and_unresolved(self):
        margin = float(self.traces[0]["release_margin"])
        queries = []
        for i, (stop, terminal, expected) in enumerate(
            (
                ("0.550000000", "1.000000000", "unresolved"),
                ("0.500000000", "1.000000000", "fail"),
                ("1.000000000", "1.000000000", "pass"),
            ),
            100,
        ):
            query = dict(self.queries[0], id=str(i), draw="false")
            query["stop_lo"], query["stop_hi"] = audit.rounding_box(stop)
            query["terminal_lo"], query["terminal_hi"] = audit.rounding_box(terminal)
            queries.append((query, expected))
        packet = self.work / "representative-queries.tsv"
        audit.write_queries(packet, [q for q, _ in queries])
        native.evaluate(self.work, packet, "representative-", self.map)
        traces = audit.pilot.read(
            self.work / "representative-trace.csv", native.TRACE_FIELDS
        )
        for trace, (_, expected) in zip(traces, queries):
            self.assertEqual(
                audit.interval_gate(trace["release_lo"], trace["release_hi"], margin),
                expected,
            )

    def inputs(self):
        query = {k: str(v) for k, v in self.queries[0].items()}
        trace = copy.deepcopy(self.traces[0])
        stop, terminal = audit.rounding_box("1.200000000"), audit.rounding_box(
            "1.000000000"
        )
        for key, value in zip(
            (
                "stop_lo",
                "stop_hi",
                "terminal_release_lo",
                "terminal_release_hi",
                "release_lo",
                "release_hi",
            ),
            (*stop, *(v - 0.05 for v in terminal), *(v - 0.05 for v in terminal)),
        ):
            trace[key] = str(value)
        record = dict(
            id="1",
            case_id="C1",
            scenario="s",
            logged_distance=trace["actual_distance"],
            logged_recovery=trace["recovery_clearance_9"],
            logged_release_margin=trace["release_margin_9"],
            logged_release="0.950000000",
        )
        witnesses = [copy.deepcopy(w) for w in self.witnesses if w["id"] == "1"]
        obstacle_index = {i: ("s", 10.0, 10.0) for i in (1, 2)}
        return record, query, trace, witnesses, obstacle_index

    def test_join_keeps_all_ties_and_checks_original_distance_and_combined_interval(
        self,
    ):
        values = self.inputs()
        row = audit.diagnose(*values)
        self.assertEqual(row["mask"], "ready")
        self.assertEqual(row["witnesses"], 2)
        self.assertEqual(row["tied_witnesses"], "true")
        self.assertEqual(row["limiting_component"], "terminal")
        for key, value in (
            ("actual_distance", "1.9"),
            ("id", "2"),
            ("release_lo", "1.1"),
            ("release_margin_9", "0.650000000"),
        ):
            bad = copy.deepcopy(values)
            bad[2][key] = value
            with self.assertRaises(ValueError, msg=key):
                audit.diagnose(*bad)
        bad = copy.deepcopy(values)
        bad[3].pop()
        with self.assertRaises(ValueError):
            audit.diagnose(*bad)

    def test_fixture_row_coordinates_cannot_be_substituted(self):
        bad = self.inputs()
        bad[3][0]["obstacle_x"] = "10.000000000000002"
        with self.assertRaises(ValueError):
            audit.diagnose(*bad)

    def test_simultaneous_component_failures_are_retained(self):
        record, query, trace, witnesses, index = self.inputs()
        stop, terminal = audit.rounding_box("0.500000000"), audit.rounding_box(
            "0.550000000"
        )
        for key, value in zip(
            (
                "stop_lo",
                "stop_hi",
                "terminal_release_lo",
                "terminal_release_hi",
                "release_lo",
                "release_hi",
            ),
            (
                *stop,
                *(v - 0.05 for v in terminal),
                min(stop[0], terminal[0] - 0.05),
                min(stop[1], terminal[1] - 0.05),
            ),
        ):
            trace[key] = str(value)
        record["logged_release"] = "0.500000000"
        row = audit.diagnose(record, query, trace, witnesses, index)
        self.assertEqual(row["mask"], "release")
        self.assertEqual(row["release_component_failures"], "stop;terminal")
        self.assertEqual(audit.counts([row])["both_components_fail"], 1)

    def test_existing_native_outputs_are_not_overwritten(self):
        before = (self.work / "trace.csv").read_bytes()
        with self.assertRaises(FileExistsError):
            native.evaluate(self.work, self.work / "queries.tsv", map_path=self.map)
        self.assertEqual((self.work / "trace.csv").read_bytes(), before)
        with self.assertRaises(ValueError):
            native.compile_native(self.work)


if __name__ == "__main__":
    unittest.main()
