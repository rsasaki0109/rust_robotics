"""Exercise full-probe geometry, original-input joins and native cross-checks."""

import copy
import csv
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import analyze_reset_mean_probe as a
import verify_reset_mean_probe as v


def fixture():
    old = dict(
        edge_id="C1@2",
        case_id="C1",
        source_step="2",
        source_time_s="0.04",
        cohort="fresh-white",
        noise="white",
        variant="epoch",
        mode="random",
        seed="1",
        scenario="map",
        planner="A*",
        multiplicity="7",
        decision_probe_origin="initial_unsafe",
        failed_gates="nominal;release",
        decision_sensed_x_center="10.0",
        decision_sensed_y_center="10.0",
        decision_mean_x_center="12.0",
        decision_mean_y_center="10.0",
        decision_mean_yaw_center="2.8",
    )
    inputs = dict(
        edge_id="C1@2",
        raw_path="fresh-white/epoch-random-1/trajectories.csv",
        prior_csv_row="2",
        current_csv_row="3",
        prior_truth_x="10.0",
        prior_truth_y="10.0",
        prior_truth_yaw="0.0",
        noise_x="0.0",
        noise_y="0.0",
        noise_yaw="0.0",
        bias_x="0.0",
        bias_y="0.0",
        bias_yaw="0.0",
        nominal_steering="0.0",
        speed_margin="0.5",
        speed_free_distance="0.0",
        speed_cap="0.0",
        uncapped_target_speed="0.4",
    )
    for key in (*("prior_truth_" + k for k in a.PRIOR), *a.CURRENT):
        old[
            (
                "decision_current_input_"
                if key.startswith(("prior_truth_", "noise_", "bias_"))
                else "decision_"
            )
            + key
        ] = inputs[key]
    return old, inputs


def strings(rows):
    return [{k: str(value) for k, value in row.items()} for row in rows]


class ProbeGeometryTests(unittest.TestCase):
    def test_initial_obstacle_and_inclusive_wall(self):
        obstacle = a.probe(25, 25, 0, 0, 0.5, [(1, 25.5, 25)])
        self.assertFalse(obstacle["initial_clear"])
        self.assertEqual(obstacle["first_unsafe_step"], 0)
        self.assertEqual(obstacle["obstacle_ids"], "1")
        wall = a.probe(0.5, 25, 0, 0, 0.5, [])
        self.assertEqual(wall["wall_ids"], "x_min")
        self.assertEqual(wall["free_distance"], 0)

    def test_first_chord_is_zero_distance_with_clear_start(self):
        result = a.probe(25, 25, 0, 0, 0.5, [(1, 25.54, 25)])
        self.assertTrue(result["initial_clear"])
        self.assertEqual(result["first_unsafe_step"], 1)
        self.assertEqual(result["free_distance"], 0)

    def test_later_chord_preserves_partial_distance(self):
        result = a.probe(25, 25, 0, 0, 0.5, [(1, 26, 25)])
        self.assertEqual(result["first_unsafe_step"], 10)
        self.assertEqual(result["free_distance"], 9 * 0.05)

    def test_last_chord_and_nearby_cutoff_are_inclusive(self):
        on_cutoff = a.probe(25, 25, 0, 0, 0.5, [(1, 27.5, 25)])
        beyond = a.probe(25, 25, 0, 0, 0.5, [(1, 27.500001, 25)])
        self.assertEqual(on_cutoff["first_unsafe_step"], 40)
        self.assertEqual(on_cutoff["free_distance"], 39 * 0.05)
        self.assertEqual(beyond["first_unsafe_step"], "")
        self.assertEqual(beyond["free_distance"], 2)

    def test_segment_interior_catches_obstacle_clear_of_both_endpoints(self):
        point, start, end = (25.025, 25), (25, 25), (25.05, 25)
        self.assertGreater(a.distance(point, start), 0.01)
        self.assertGreater(a.distance(point, end), 0.01)
        self.assertEqual(a.segment_distance(point, start, end), 0)
        self.assertEqual(
            a.probe(*start, 0, 0, 0.01, [(1, *point)])["first_unsafe_step"], 1
        )

    def test_degenerate_segment_uses_start_distance(self):
        self.assertEqual(a.segment_distance((1, 2), (0, 0), (1e-11, 0)), math.sqrt(5))

    def test_simultaneous_walls_and_duplicate_obstacles_keep_all_ids(self):
        result = a.probe(0.5, 0.5, 0, 0, 0.5, [(1, 1, 0.5), (2, 1, 0.5)])
        self.assertEqual(result["wall_ids"], "x_min;y_min")
        self.assertEqual(result["obstacle_ids"], "1;2")

    def test_curved_endpoints_mirror_and_free_horizon(self):
        left = a.endpoint(25, 25, 0, 0.4, 40)
        right = a.endpoint(25, 25, 0, -0.4, 40)
        self.assertAlmostEqual(left[0], right[0])
        self.assertAlmostEqual(left[1] + right[1], 50)
        self.assertGreater(left[1], 25)
        self.assertEqual(a.probe(25, 25, 0, 0.4, 0.5, [])["free_distance"], 2)

    def test_near_zero_branch_and_invalid_inputs(self):
        yaw = 0.7
        straight = a.endpoint(25, 25, yaw, 0, 40)
        self.assertEqual(
            straight, a.endpoint(25, 25, yaw, math.atan(0.4 * 0.999e-9), 40)
        )
        for steering in (math.atan(0.4 * 1.001e-9), -math.atan(0.4 * 1.001e-9)):
            self.assertTrue(
                all(math.isfinite(x) for x in a.endpoint(25, 25, yaw, steering, 40))
            )
        for args in (
            (math.nan, 25, 0, 0, 0.5),
            (25, 25, 0, 0.7, 0.5),
            (25, 25, 0, 0, 0),
        ):
            with self.subTest(args=args), self.assertRaises(ValueError):
                a.probe(*args, [])


class AuditTests(unittest.TestCase):
    def test_selected_packet_rejects_changed_hashes_and_provenance(self):
        old, inputs = fixture()
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            a.write(directory / "selected-inputs.csv", [inputs], a.INPUT_FIELDS)
            pins = {
                "input_sha256": {"fixed": "digest"},
                "raw_sha256": {inputs["raw_path"]: "raw-digest"},
            }
            original = dict(
                method="selected original decision/predecessor raw strings; no regenerated trajectories",
                analysis=a.analysis(),
                input_sha256=pins["input_sha256"],
                raw_sha256=pins["raw_sha256"],
                raw_rows_scanned={inputs["raw_path"]: 2},
                output_sha256={
                    "selected-inputs.csv": a.sha(directory / "selected-inputs.csv")
                },
            )
            source = directory / "selected-input-sources.json"
            source.write_text(json.dumps(original))
            self.assertEqual(a.load_inputs(directory, pins, [old]), [inputs])
            for field in ("analysis", "input_sha256", "raw_sha256", "output_sha256"):
                changed = dict(original, **{field: {}})
                source.write_text(json.dumps(changed))
                with self.subTest(field=field), self.assertRaises(ValueError):
                    a.load_inputs(directory, pins, [old])
            source.write_text(json.dumps(original))
            with (directory / "selected-inputs.csv").open("a") as stream:
                stream.write("\n")
            with self.assertRaisesRegex(ValueError, "bytes"):
                a.load_inputs(directory, pins, [old])

    def test_native_comparison_rejects_changed_geometry_flags_and_speed_formula(self):
        old, inputs = fixture()
        rows = strings(a.calculate([old], [inputs], {"map": [(1, 10.4, 10)]}))
        traces = [
            dict(
                initial_clear=r["initial_clear"].lower(),
                **{
                    k: r[k]
                    for k in (
                        "free_distance",
                        "first_unsafe_step",
                        "wall_ids",
                        "obstacle_ids",
                    )
                },
            )
            for r in rows
        ]
        v.check_trace(rows, traces)
        for key, new in (
            ("free_distance", "1.95"),
            ("initial_clear", "False"),
            ("first_unsafe_step", "40"),
            ("obstacle_ids", "3"),
            ("speed_cap", "0.5"),
            ("nominal_target_speed", "0.3"),
            ("full_horizon_clear", "False"),
            ("positive_distance", "False"),
        ):
            changed = copy.deepcopy(rows)
            changed[1][key] = new
            with self.subTest(key=key), self.assertRaises(ValueError):
                v.check_trace(changed, traces)

    def test_only_position_changes_and_source_addition_order_is_preserved(self):
        old, row = fixture()
        row.update(
            prior_truth_x="1e16",
            noise_x="-1e16",
            bias_x="1",
            prior_truth_yaw="0.1",
            noise_yaw="0.2",
            bias_yaw="0.3",
        )
        old["decision_sensed_x_center"] = "1"
        centers = a.centers(old, row)
        self.assertEqual(centers["raw"][0], 1)
        self.assertEqual(centers["raw"][2], (0.1 + 0.2) + 0.3)
        self.assertEqual(centers["mean"][:2], (12, 10))
        self.assertEqual(centers["mean"][2], centers["raw"][2])

    def test_calculation_keeps_masks_and_distinguishes_nominal_from_geometry(self):
        old, inputs = fixture()
        rows = a.calculate([old], [inputs], {"map": [(1, 10.4, 10)]})
        self.assertEqual([r["arm"] for r in rows], ["raw", "mean"])
        self.assertEqual(rows[1]["free_distance"], 2)
        self.assertEqual(rows[1]["nominal_target_speed"], 0.4)
        self.assertEqual(rows[1]["speed_cap"], 1)
        self.assertTrue(rows[1]["full_horizon_clear"])
        self.assertEqual(rows[1]["original_failed_gates"], "nominal;release")
        inputs["uncapped_target_speed"] = "0"
        zero = a.calculate([old], [inputs], {"map": [(1, 10.4, 10)]})[1]
        self.assertTrue(zero["positive_distance"])
        self.assertEqual(zero["nominal_target_speed"], 0)

    def test_original_probe_disagreement_fails_instead_of_relabeling(self):
        old, inputs = fixture()
        with self.assertRaisesRegex(ValueError, "Original probe differs"):
            a.calculate([old], [inputs], {"map": []})
        old["decision_probe_origin"] = "first_chord_by_exclusion"
        with self.assertRaisesRegex(ValueError, "failure stage"):
            a.calculate([old], [inputs], {"map": [(1, 10.4, 10)]})

    def test_selected_inputs_reject_wrong_identity_field_and_predecessor(self):
        old, inputs = fixture()
        a.validate_inputs([inputs], [old])
        for key, value in (
            ("edge_id", "C2@2"),
            ("raw_path", "wrong"),
            ("prior_csv_row", "1"),
            ("noise_x", "0.1"),
            ("speed_cap", "nan"),
            ("nominal_steering", "0.01"),
        ):
            changed = dict(inputs, **{key: value})
            with self.subTest(key=key), self.assertRaises(ValueError):
                a.validate_inputs([changed], [old])
        with self.assertRaises(ValueError):
            a.validate_inputs([inputs, inputs], [old])

    def test_summaries_keep_multiplicity_empty_cells_and_partition_counts(self):
        old, inputs = fixture()
        rows = a.calculate([old], [inputs], {"map": [(1, 10.4, 10)]})
        cells = a.summaries(rows)
        self.assertEqual(len(cells), 16)
        all_mean = next(
            r
            for r in cells
            if (r["case_selector"], r["origin_selector"], r["arm"])
            == ("all_cases", "all_origins", "mean")
        )
        self.assertEqual(
            (
                all_mean["physical_resets"],
                all_mean["ending_multiplicity"],
                all_mean["full_horizon_clear"],
            ),
            (1, 7, 1),
        )
        empty = next(r for r in cells if r["origin_selector"] == "ambiguous_zero")
        self.assertEqual(empty["max_free_distance"], "")
        v.check_summaries(strings(cells), strings(rows))
        changed = strings(cells)
        changed[0]["ending_multiplicity"] = "8"
        with self.assertRaisesRegex(ValueError, "Summary counts"):
            v.check_summaries(changed, strings(rows))
        changed = strings(cells)
        next(r for r in changed if r["origin_selector"] == "ambiguous_zero")[
            "max_free_distance"
        ] = "0"
        with self.assertRaisesRegex(ValueError, "must be blank"):
            v.check_summaries(changed, strings(rows))

    def test_verifier_rejects_changed_parameters_order_and_masks(self):
        old, inputs = fixture()
        rows = strings(a.calculate([old], [inputs], {"map": [(1, 10.4, 10)]}))
        v.check_rows(rows, [old], [inputs])
        for key, value in (
            ("yaw", "0.1"),
            ("steering", "0.1"),
            ("margin", "0.1"),
            ("x", "13"),
            ("original_failed_gates", "nominal"),
        ):
            changed = copy.deepcopy(rows)
            changed[1][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                v.check_rows(changed, [old], [inputs])
        with self.assertRaises(ValueError):
            v.check_rows(list(reversed(rows)), [old], [inputs])

    def test_csv_reader_rejects_duplicate_headers_and_ragged_rows(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.csv"
            for text in ("a,a\n1,2\n", "a,b\n1\n", "a\n1,2\n"):
                path.write_text(text)
                with self.subTest(text=text), self.assertRaises(ValueError):
                    a.read(path)
                with self.assertRaises(ValueError):
                    v.read(path)

    def test_output_protection_and_optimized_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                a.fresh_output(Path(tmp))
            with self.assertRaises(ValueError):
                a.fresh_output(a.REPO / "new-reset-probe-output")
            new = Path(tmp) / "new"
            self.assertEqual(a.fresh_output(new), new)
            result = subprocess.run(
                [
                    sys.executable,
                    "-O",
                    str(a.ROOT / "analyze_reset_mean_probe.py"),
                    "--output",
                    str(new),
                ],
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("without Python optimization", result.stderr)
            self.assertFalse(new.exists())

    def test_extraction_checks_full_raw_hash_and_exact_overlaps(self):
        old, inputs = fixture()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            raw = root / "raw/epoch-random-1/trajectories.csv"
            raw.parent.mkdir(parents=True)
            fields = ("scenario", "planner", "time_s", "x", "y", "yaw", *a.CURRENT)
            before = dict(
                scenario="map",
                planner="A*",
                time_s="0.02",
                x="10.0",
                y="10.0",
                yaw="0.0",
                **{k: inputs[k] for k in a.CURRENT},
            )
            current = dict(before, time_s="0.04")
            a.write(raw, [before, current], fields)
            digest = a.sha(raw)
            parent = root / "results/fusion-return-first-chord"
            parent.mkdir(parents=True)
            (parent / "sources.json").write_text(
                json.dumps(
                    {
                        "raw": {
                            "fresh-white": {
                                "path": str(root / "raw"),
                                "raw_sha256": {
                                    "epoch-random-1/trajectories.csv": digest
                                },
                            }
                        }
                    }
                )
            )
            pins = {"raw_sha256": {inputs["raw_path"]: digest}}
            with mock.patch.object(a, "RESULTS", root / "results"):
                rows, scanned = a.extract(pins, [old])
                self.assertEqual(rows, [inputs])
                self.assertEqual(scanned, {inputs["raw_path"]: 2})
                with self.assertRaises(ValueError):
                    a.extract(pins, [old, old])
                with raw.open("a") as stream:
                    stream.write("\n")
                with self.assertRaisesRegex(ValueError, "Changed original raw file"):
                    a.extract(pins, [old])


class NativeProbeTests(unittest.TestCase):
    def test_recorded_19_edges_replay_and_native_verification(self):
        rustc = shutil.which("rustc")
        self.assertIsNotNone(
            rustc, "Activate the Rust toolchain for native geometry tests"
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            result = root / "results"
            a.run(
                SimpleNamespace(
                    output=result, extract_raw=False, raw_root=None, inputs=a.STUDY
                )
            )
            for name in ("reprobes.csv", "reprobe-summary.csv", "sources.json"):
                self.assertEqual(
                    (result / name).read_bytes(), (a.STUDY / name).read_bytes(), name
                )
            proof = v.verify(
                SimpleNamespace(
                    inputs=a.STUDY,
                    results=result,
                    work=root / "native",
                    rustc=Path(rustc),
                )
            )
            self.assertEqual(proof["distinct_edges"], 19)
            self.assertEqual(proof["ending_multiplicity"], 185)
            self.assertEqual(proof["native"]["probe_evaluations"], 38)
            stored = json.loads((a.STUDY / "independent-verification.json").read_text())
            self.assertEqual(stored["verifier_sha256"], v.sha(Path(v.__file__)))
            self.assertEqual(stored["protocol_sha256"], proof["protocol_sha256"])
            self.assertEqual(stored["freeze_sha256"], proof["freeze_sha256"])
            self.assertEqual(stored["input_sha256"], proof["input_sha256"])
            self.assertEqual(stored["output_sha256"], proof["output_sha256"])
            self.assertEqual(
                stored["native"]["trace_sha256"], v.sha(a.STUDY / "native-trace.csv")
            )

    def test_unchanged_rust_probe_matches_synthetic_geometry_and_all_witnesses(self):
        rustc = shutil.which("rustc")
        self.assertIsNotNone(
            rustc, "Activate the Rust toolchain for native geometry tests"
        )
        cases = [
            (25, 25, 0, 0, 0.5, []),
            (25, 25, 0, 0, 0.5, [(25.5, 25)]),
            (25, 25, 0, 0, 0.5, [(25.54, 25)]),
            (25, 25, 0, 0, 0.5, [(26, 25)]),
            (25, 25, 0, 0, 0.5, [(27.5, 25)]),
            (25, 25, 0, 0, 0.5, [(27.500001, 25)]),
            (25, 25, 0, 0, 0.01, [(25.025, 25)]),
            (49.4, 25, 0, 0, 0.5, []),
            (0.5, 0.5, 0, 0, 0.5, [(1, 0.5), (1, 0.5)]),
            (25, 25, 0, 0.4, 0.06, [a.endpoint(25, 25, 0, 0.4, 10)]),
            (25, 25, 0, -0.4, 0.06, [a.endpoint(25, 25, 0, -0.4, 10)]),
            (25, 25, 0.7, math.atan(0.4 * 0.999e-9), 0.5, []),
            (25, 25, 0.7, math.atan(0.4 * 1.001e-9), 0.5, []),
        ]
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            map_path = root / "map.csv"
            queries, expected, map_rows = [], [], []
            for i, (x, y, yaw, steering, margin, obstacles) in enumerate(cases):
                scenario = f"case{i}"
                numbered = []
                for ox, oy in obstacles:
                    map_rows.append(dict(scenario=scenario, x=ox, y=oy))
                    numbered.append((len(map_rows), ox, oy))
                queries.append(
                    dict(
                        scenario=scenario,
                        x=str(x),
                        y=str(y),
                        yaw=str(yaw),
                        steering=str(steering),
                        margin=str(margin),
                    )
                )
                expected.append(a.probe(x, y, yaw, steering, margin, numbered))
            a.write(map_path, map_rows, ("scenario", "x", "y"))
            native, proof = v.native(root / "native", Path(rustc), queries, map_path)
            self.assertEqual(proof["probe_evaluations"], len(cases))
            self.assertIn(
                v.source_block(v.SIMULATOR.read_text(), "fn speed_free_distance("),
                (root / "native/reset-mean-probe.rs").read_text(),
            )
            for i, (row, wanted) in enumerate(zip(native, expected)):
                with self.subTest(case=i):
                    self.assertEqual(
                        float(row["free_distance"]), wanted["free_distance"]
                    )
                    self.assertEqual(
                        row["initial_clear"], str(wanted["initial_clear"]).lower()
                    )
                    for key in ("first_unsafe_step", "wall_ids", "obstacle_ids"):
                        self.assertEqual(row[key], str(wanted[key]))


if __name__ == "__main__":
    unittest.main()
