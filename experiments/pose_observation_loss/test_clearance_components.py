"""Check exact threshold semantics, native identities and complete annotations."""

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

import analyze_clearance_components as audit
import analyze_mean_speed_probe as pilot
import native_guard_release_census as native
from test_guard_release_census import query


def fixture(followup=2):
    catalogue, events, rows = [], [], []
    for mode in ("none", "random"):
        for arm in ("mean_preview", "one_step_handoff"):
            c = dict(
                tau="0",
                arm=arm,
                mode=mode,
                seed="17",
                scenario="line",
                planner="A",
                status="timeout" if arm == "mean_preview" else "held",
                resume_count="1" if mode == "random" else "0",
            )
            c["case_id"] = f"tau0/{arm}/{mode}/17/line/A"
            catalogue.append(c)
            if mode == "none":
                continue
            tid = len(events) + 1
            start = tid * 100
            events.append(
                dict(
                    transition_id=str(tid),
                    **{k: c[k] for k in audit.META},
                    status=c["status"],
                    resume_step=str(start),
                    followup_steps=str(followup),
                    first_trigger_lag="1" if followup else "",
                )
            )
            for lag in range(followup + 1):
                pid = len(rows) // 7 + 1
                stop, terminal = ((1.1, 1.3), (1.0, 0.8), (0.7, 1.2))[lag]
                if arm == "one_step_handoff":
                    stop, terminal = ((1.2, 1.0), (0.49, 0.9), (0.65, 0.7))[lag]
                b = 0.5 + 0 + 2 * math.sqrt(2) * 0.1
                g = b + 0.05
                for variant in audit.VARIANTS:
                    r = dict.fromkeys(audit.previous.SAMPLE_FIELDS, "0")
                    for k in audit.BOOL_FIELDS:
                        r[k] = "false"
                    r.update(
                        query_id=str(len(rows) + 1),
                        point_id=str(pid),
                        transition_id=str(tid),
                        **{k: c[k] for k in audit.META},
                        status=c["status"],
                        resume_step=str(start),
                        step=str(start + lag),
                        lag=str(lag),
                        variant=variant,
                        eligible=str(lag < 2).lower(),
                        entry_ready=str(lag > 0).lower(),
                        actual_trigger=str(lag == 1).lower(),
                        selected_source="epoch_fusion",
                        recovery_source="current_window",
                        count="25",
                        target=".4",
                        effective_position_sigma_9=".1",
                        stop=str(stop),
                        terminal=str(terminal),
                        entry=str(min(stop, terminal - 0.1)),
                        release=str(min(stop, terminal - 0.05)),
                        preview_margin=str(b),
                        release_margin=str(g),
                        comparison="tied",
                    )
                    rows.append(r)
    return catalogue, events, rows


def write_input(directory, rows):
    directory.mkdir()
    pilot.write(directory / "samples.csv", rows, audit.previous.SAMPLE_FIELDS)


class ScalarTests(unittest.TestCase):
    def test_mechanisms_binding_and_release_blocks(self):
        expected = [
            (0.7, 2.0, "stop_only", "stop", "stop_only"),
            (2.0, 0.85, "terminal_only", "terminal", "terminal_only"),
            (0.7, 0.8, "both", "tie", "both"),
            (2.0, 2.0, "clear", "terminal", "clear"),
        ]
        for stop, terminal, entry, binding, release in expected:
            r = audit.classify(stop, terminal, 0.8, 0.8 + 0.05)
            self.assertEqual(
                (r["entry_mechanism"], r["binding"], r["release_block"]),
                (entry, binding, release),
            )

    def test_all_allowance_layers_include_either_and_joint_requirements(self):
        expected = [
            (0.4, 2.0, 0.8, "radius_level"),
            (0.7, 2.0, 0.8, "margin_only"),
            (2.0, 0.59, 0.52, "reserve_only"),
            (2.0, 0.59, 0.8, "either_sufficient"),
            (2.0, 0.85, 0.8, "jointly_required"),
            (2.0, 2.0, 0.8, "clear"),
        ]
        for stop, terminal, b, label in expected:
            r = audit.classify(stop, terminal, b, b + 0.05)
            self.assertEqual(r["layer"], label)
            self.assertEqual(r["would_enter"], label != "clear")

    def test_exact_inclusive_entry_and_strict_release_boundaries(self):
        b = 1.0
        g = b + 0.05
        for stop in (math.nextafter(b, -math.inf), b, math.nextafter(b, math.inf)):
            self.assertEqual(audit.classify(stop, 3.0, b, g)["would_enter"], stop <= b)
        for stop in (math.nextafter(g, -math.inf), g, math.nextafter(g, math.inf)):
            self.assertEqual(audit.classify(stop, 3.0, b, g)["release_clear"], stop > g)
        for terminal in (
            math.nextafter(b + 0.1, -math.inf),
            b + 0.1,
            math.nextafter(b + 0.1, math.inf),
        ):
            self.assertEqual(
                audit.classify(3.0, terminal, b, g)["would_enter"], terminal - 0.1 <= b
            )
        r = audit.classify(1.0, 1.1 + 5e-13, 1.0, 1.05)
        self.assertEqual(r["binding"], "tie")
        self.assertEqual(r["entry_mechanism"], "stop_only")

    def test_gaps_use_recorded_base_and_never_release_subtraction(self):
        b = math.nextafter(1.0, 0.0)
        g = b + 0.05
        self.assertNotEqual(g - 0.05, b)
        r = audit.classify(b, 3.0, b, g)
        self.assertTrue(r["would_enter"])
        self.assertEqual(r["stop_entry_gap_m"], 0.0)
        self.assertEqual(r["allowance_m"], b - 0.5)
        self.assertEqual(r["raw_radius_gap_m"], b - 0.5)
        for values in (
            (float("nan"), 2.0, 0.8, 0.85),
            (2.0, float("inf"), 0.8, 0.85),
            (2.0, 2.0, 0.4, 0.45),
            (2.0, 2.0, 0.8, 0.9),
        ):
            with self.assertRaises(ValueError):
                audit.classify(*values)

    def test_native_entry_and_release_identities(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            maps = root / "maps.csv"
            maps.write_text("scenario,x,y\nline,11.22,10\nsafe,30,30\nturn,11.2,10.8\n")
            work = root / "native"
            native.compile_native(work)
            rows = [
                query(1),
                query(2, target=0),
                query(3, "safe"),
                query(4, "turn", steering=0.35),
                query(5, "turn", steering=-0.35),
                query(6, "safe", precision=True, sigma=0.1, drift=0.02, age=1.5),
            ]
            with (work / "queries.tsv").open("x", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=native.INPUT_FIELDS,
                    delimiter="\t",
                    lineterminator="\n",
                )
                writer.writeheader()
                writer.writerows(rows)
            native.evaluate(work, work / "queries.tsv", maps)
            for g in pilot.read(work / "trace.csv", native.TRACE_FIELDS):
                stop, terminal, b, release = (
                    float(g[k])
                    for k in ("stop", "terminal", "preview_margin", "release_margin")
                )
                r = audit.classify(stop, terminal, b, release)
                self.assertEqual(float(g["entry"]), min(stop, terminal - 0.1))
                self.assertEqual(float(g["release"]), min(stop, terminal - 0.05))
                self.assertEqual(r["would_enter"], float(g["entry"]) <= b)
                self.assertEqual(r["release_clear"], float(g["release"]) > release)


class CompletePipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.catalogue, cls.events, cls.rows = fixture()
        cls.input = cls.root / "input"
        write_input(cls.input, cls.rows)
        cls.receipts = []
        for name in ("one", "two"):
            cls.receipts.append(
                audit.evaluate(cls.input, cls.catalogue, cls.events, cls.root / name)
            )

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_every_case_variant_pair_and_zero_control_is_retained(self):
        r = self.receipts[0]
        self.assertEqual(
            (
                r["cases"],
                r["case_variant_rows"],
                r["pairs"],
                r["pair_variant_rows"],
                r["resumes"],
                r["points"],
                r["rows"],
                r["summary_cells"],
            ),
            (4, 28, 2, 14, 2, 6, 42, 3528),
        )
        self.assertEqual(r["native_identity_matches"], 42)
        self.assertEqual(r["margin_matches"], 42)
        for c in pilot.read(self.root / "one/cases.csv", audit.CASE_FIELDS):
            if c["mode"] == "none":
                self.assertEqual(c["rows"], "0")
                self.assertTrue(all(c[k] == "" for k in audit.STATS))

    def test_all_fresh_output_files_are_byte_identical(self):
        one, two = self.root / "one", self.root / "two"
        self.assertEqual(
            {p.name for p in one.iterdir()}, {p.name for p in two.iterdir()}
        )
        for p in one.iterdir():
            self.assertEqual(p.read_bytes(), (two / p.name).read_bytes())

    def test_partition_counts_and_pooled_gaps_conserve_all_rows(self):
        rows, _ = audit.read_annotations(
            self.input / "samples.csv", self.catalogue, self.events
        )
        result = audit.metrics(rows)
        for prefix, labels in (
            ("entry_", audit.MECHANISMS),
            ("binding_", audit.BINDINGS),
            ("layer_", audit.LAYERS),
            ("release_block_", audit.MECHANISMS),
        ):
            self.assertEqual(sum(result[prefix + k] for k in labels), len(rows))
        self.assertAlmostEqual(
            result["mean_entry_gap_m"], sum(r["entry_gap_m"] for r in rows) / len(rows)
        )
        self.assertEqual(result["entry_rows"], len(rows) - result["entry_clear"])
        empty = audit.metrics([])
        self.assertTrue(all(empty[k] == 0 for k in audit.COUNTS))
        self.assertTrue(all(empty[k] == "" for k in audit.STATS))

    def test_all_resumes_retain_truth_first_entry_and_lag_two_components(self):
        rows = pilot.read(self.root / "one/resumes.csv", audit.RESUME_FIELDS)
        self.assertEqual(len(rows), 2)
        self.assertTrue(
            all(
                r["first_truth_entry_lag"] == "1"
                and r["first_actual_trigger_lag"] == "1"
                for r in rows
            )
        )
        self.assertEqual(rows[0]["lag2_truth_entry_mechanism"], "stop_only")
        self.assertEqual(rows[1]["lag2_truth_entry_mechanism"], "both")

    def test_changed_scalar_margin_boolean_and_unused_numeric_fields_fail(self):
        for field, value in (
            ("entry", "99"),
            ("release", "99"),
            ("preview_margin", ".7"),
            ("release_margin", ".9"),
            ("eligible", "True"),
            ("stop", "nan"),
            ("error_x_m", "inf"),
        ):
            r = copy.deepcopy(self.rows[0])
            r[field] = value
            with self.assertRaises(ValueError, msg=field):
                audit.annotate(r)

    def test_missing_reordered_mixed_and_truncated_inputs_fail(self):
        for i, kind in enumerate(
            ("missing", "variant", "query", "case", "shared", "window", "last")
        ):
            rows = copy.deepcopy(self.rows)
            if kind == "missing":
                rows.pop(8)
            if kind == "variant":
                rows[1]["variant"] = "recorded"
            if kind == "query":
                rows[7]["query_id"] = "1"
            if kind == "case":
                rows[7]["case_id"] = "unknown"
            if kind == "shared":
                rows[8]["actual_trigger"] = "false"
            if kind == "window":
                rows[7]["lag"] = "2"
            if kind == "last":
                rows = rows[:-7]
            directory = self.root / f"bad-{i}"
            write_input(directory, rows)
            with self.assertRaises(ValueError, msg=kind):
                audit.read_annotations(
                    directory / "samples.csv", self.catalogue, self.events
                )

    def test_endpoint_resumes_remain_with_empty_following_statistics(self):
        catalogue, events, rows = fixture(followup=0)
        directory = self.root / "endpoint"
        write_input(directory, rows)
        result = audit.evaluate(
            directory, catalogue, events, self.root / "endpoint-result"
        )
        self.assertEqual(
            (result["resumes"], result["points"], result["rows"]), (2, 2, 14)
        )
        exported = pilot.read(
            self.root / "endpoint-result/resumes.csv", audit.RESUME_FIELDS
        )
        self.assertTrue(
            all(
                r["first_truth_entry_lag"] == r["lag2_truth_layer"] == ""
                for r in exported
            )
        )


class IntegrityTests(unittest.TestCase):
    def test_freeze_optimized_python_dirty_git_and_output_guards(self):
        pins = audit.preflight()
        self.assertEqual(len(pins["input_sha256"]), 223)
        result = subprocess.run(
            [
                sys.executable,
                "-O",
                "-B",
                "-c",
                "import analyze_clearance_components as a; a.preflight()",
            ],
            cwd=audit.build.ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b"Python-O", result.stderr)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            one, two = root / "one", root / "two"
            one.mkdir()
            two.mkdir()
            with mock.patch.object(
                audit.subprocess, "check_output", return_value="dirty"
            ):
                with self.assertRaisesRegex(ValueError, "Commit component"):
                    audit.collect(one, two, root / "fresh")
            with self.assertRaises(ValueError):
                audit.build.fresh(root)
            with self.assertRaises(ValueError):
                audit.build.fresh(audit.build.ROOT / "internal-component-test")


if __name__ == "__main__":
    unittest.main()
