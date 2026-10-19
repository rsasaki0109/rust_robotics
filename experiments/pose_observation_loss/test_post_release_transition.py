"""Verify fixed-input geometry, original transport and exhaustive release joins."""

import copy
import csv
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import analyze_post_release_transition as audit
import analyze_mean_speed_probe as pilot
import build_mean_guard_preview as build
import native_guard_release_census as census
import native_post_release_transition as native
from test_guard_release_census import query as old_query, logged
from verify_reset_mean_probe import source_block


def query(ident, transition, phase, **changes):
    old = old_query(ident)
    old.update(mean_x=12.0, preview_x=10.0)
    result = dict(id=ident, transition_id=transition, phase=phase)
    result.update({k: v for k, v in old.items() if k != "id"})
    result.update(
        resume_mean_x=12.0,
        resume_mean_y=10.0,
        resume_mean_yaw=0.0,
        resume_speed=0.0,
        resume_applied_steering=0.0,
    )
    result.update(changes)
    return result


def packet(path, rows, fields=native.INPUT_FIELDS):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=fields, delimiter="\t", lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)


class NativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if shutil.which("rustc") is None:
            raise RuntimeError("Activate Rust for post-release native tests")
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.maps = cls.root / "maps.csv"
        cls.maps.write_text("scenario,x,y\nline,11.22,10\nsafe,30,30\nturn,11.2,10.8\n")
        cls.inputs = [
            query(1, 1, "resume"),
            query(2, 1, "next"),
            query(3, 2, "resume", preview_x=12, mean_x=10, resume_mean_x=10),
            query(4, 2, "next", preview_x=12, mean_x=10, resume_mean_x=10),
            query(
                5,
                3,
                "resume",
                scenario="turn",
                resume_mean_yaw=1.1,
                resume_speed=-0.02,
                resume_applied_steering=0.35,
            ),
            query(
                6,
                3,
                "next",
                scenario="turn",
                preview_yaw=-0.4,
                measured_speed=0.017,
                resume_mean_yaw=1.1,
                resume_speed=-0.02,
                resume_applied_steering=0.35,
            ),
            query(7, 4, "resume", scenario="safe"),
            query(8, 4, "next", scenario="safe", nominal_target=0, mean_x=0.6),
            query(
                9,
                5,
                "resume",
                scenario="safe",
                precision_active="true",
                speed_margin=0.5 + 0.1 + 2 * math.sqrt(2) * 0.03,
            ),
            query(10, 6, "resume", scenario="safe"),
        ]
        for name in ("one", "two"):
            work = cls.root / name
            native.compile_native(work)
            packet(work / "queries.tsv", cls.inputs)
            native.evaluate(work, work / "queries.tsv", cls.maps)
        cls.trace = pilot.read(cls.root / "one/trace.csv", native.TRACE_FIELDS)
        cls.groups = {
            (r["transition_id"], r["phase"], r["variant"]): r for r in cls.trace
        }
        work = cls.root / "original"
        census.compile_native(work)
        packet(
            work / "queries.tsv",
            [{k: row[k] for k in census.INPUT_FIELDS} for row in cls.inputs],
            census.INPUT_FIELDS,
        )
        census.evaluate(work, work / "queries.tsv", cls.maps)
        cls.original = pilot.read(work / "trace.csv", census.TRACE_FIELDS)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_original_functions_and_repeat_bytes(self):
        original = build.BASE.read_text()
        source = native.source()
        for marker in (
            "fn stopping_preview(",
            "fn terminal_arc_clearance(",
            "fn aligned_release_preview(",
            "impl MotionPoseFilter",
            "fn point_segment_distance(",
        ):
            self.assertIn(source_block(original, marker), source)
        for name in (
            "post-release-transition.rs",
            "native-post-release",
            "queries.tsv",
            "trace.csv",
            "compiler.log",
            "native.log",
        ):
            self.assertEqual(
                (self.root / "one" / name).read_bytes(),
                (self.root / "two" / name).read_bytes(),
            )

    def test_all_recorded_geometry_matches_original_census(self):
        recorded = [row for row in self.trace if row["variant"] == "recorded"]
        self.assertEqual(len(recorded), len(self.original))
        for fresh, old in zip(recorded, self.original):
            for field in census.TRACE_FIELDS:
                self.assertEqual(fresh[field], old[field])

    def test_variants_preserve_every_fixed_field(self):
        for row in self.trace:
            original = self.inputs[int(row["id"]) - 1]
            for field, input_field in (
                ("yaw", "preview_yaw"),
                ("speed", "measured_speed"),
                ("target", "nominal_target"),
                ("steering", "nominal_steering"),
            ):
                self.assertEqual(float(row[field]), float(original[input_field]))
            if row["variant"] == "current_recovery_xy":
                self.assertEqual(float(row["x"]), float(original["mean_x"]))
                self.assertEqual(float(row["y"]), float(original["mean_y"]))
        self.assertEqual(len(self.trace), 6 + 3 * 4)

    def test_original_transport_uses_resume_speed_and_heading(self):
        row = self.groups["3", "next", "carried_release_mean_xy"]
        distance = -0.02 * 0.02
        self.assertEqual(float(row["x"]), 12.0 + distance * math.cos(1.1))
        self.assertEqual(float(row["y"]), 10.0 + distance * math.sin(1.1))
        self.assertEqual(float(row["yaw"]), -0.4)
        self.assertEqual(float(row["speed"]), 0.017)

    def test_native_witnesses_retain_both_flip_directions(self):
        for prefix, variant in audit.ALTERNATIVES.items():
            gained = audit.flips(
                self.groups["1", "next", "recorded"], self.groups["1", "next", variant]
            )
            lost = audit.flips(
                self.groups["2", "next", "recorded"], self.groups["2", "next", variant]
            )
            self.assertEqual(
                gained,
                dict(
                    entry_avoided=True,
                    entry_added=False,
                    release_gain=True,
                    release_loss=False,
                ),
                prefix,
            )
            self.assertEqual(
                lost,
                dict(
                    entry_avoided=False,
                    entry_added=True,
                    release_gain=False,
                    release_loss=True,
                ),
                prefix,
            )

    def test_endpoint_resumes_do_not_create_next_queries(self):
        self.assertEqual(
            [r["variant"] for r in self.trace if r["transition_id"] in ("5", "6")],
            ["recorded", "recorded"],
        )
        zero = self.groups["4", "next", "recorded"]
        self.assertEqual(float(zero["target"]), 0)
        precision = self.groups["5", "resume", "recorded"]
        self.assertEqual(float(precision["effective_position_sigma_9"]), 0.03)
        self.assertEqual(
            float(precision["release_margin"]), 0.5 + 2 * math.sqrt(2) * 0.03 + 0.05
        )

    def test_native_rejects_nonfinite_margins_order_phase_and_boolean(self):
        for field, value, index in (
            ("id", 7, 0),
            ("transition_id", 2, 0),
            ("phase", "next", 0),
            ("phase", "next", 2),
            ("resume_speed", "nan", 1),
            ("preview_x", "inf", 1),
            ("speed_margin", 0.5, 1),
            ("precision_active", "True", 1),
        ):
            rows = copy.deepcopy(self.inputs)
            rows[index][field] = value
            path = self.root / "corrupt.tsv"
            packet(path, rows)
            result = subprocess.run(
                [str(self.root / "one/native-post-release"), str(self.maps), str(path)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.assertNotEqual(result.returncode, 0, (field, index))

    def test_baseline_validator_rejects_changed_full_geometry(self):
        row = self.groups["1", "next", "recorded"]
        item = dict(
            row=logged(row),
            side=dict(
                margin=row["recovery_margin"],
                observation_age="0",
                measured_speed="0",
                count="25",
                nominal_target="0.4",
            ),
            guard=dict(
                selected_x=row["x"],
                selected_y=row["y"],
                selected_yaw=row["yaw"],
                selected_v=row["speed"],
            ),
        )
        audit.validate_recorded(item, row, self.original[1])
        for field in (
            "stop",
            "terminal",
            "entry",
            "release",
            "preview_margin",
            "release_margin",
        ):
            changed = dict(row)
            changed[field] = str(math.nextafter(float(changed[field]), math.inf))
            with self.assertRaises(ValueError):
                audit.validate_recorded(item, changed, self.original[1])


class PopulationTests(unittest.TestCase):
    def test_entry_includes_equality_release_excludes_equality(self):
        row = dict(
            entry="0.5", preview_margin="0.5", release="0.55", release_margin="0.55"
        )
        self.assertEqual(
            audit.threshold(row), dict(would_enter=True, release_clear=False)
        )
        row.update(
            entry=math.nextafter(0.5, math.inf), release=math.nextafter(0.55, math.inf)
        )
        self.assertEqual(
            audit.threshold(row), dict(would_enter=False, release_clear=True)
        )

    def test_selection_keeps_ordinary_following_and_packet_endpoint(self):
        row = dict(resumed="true", guard_active="false", safety_paused="false")
        track = [{}, row, dict(row, resumed="false"), row]
        sides = [{}, {}, {}]
        guards = [dict(joint_pending="true") for _ in sides]
        selected = audit.select_case({}, track, sides, guards, 20)
        self.assertEqual([r["resume"]["global_id"] for r in selected], [21, 23])
        self.assertEqual(selected[0]["next"]["global_id"], 22)
        self.assertIsNone(selected[1]["next"])
        with self.assertRaises(ValueError):
            audit.select_case({}, track, sides[:-1], guards, 20)
        bad = copy.deepcopy(track)
        bad[1]["guard_active"] = "true"
        with self.assertRaises(ValueError):
            audit.select_case({}, bad, sides, guards, 20)

    def test_case_endpoint_and_summary_conservation_with_zero_controls(self):
        cases = []
        for arm in ("fused_preview", "mean_preview"):
            meta = dict(
                case_id=arm,
                tau=0,
                arm=arm,
                mode="random",
                seed=9,
                scenario="S",
                planner="A",
            )
            following = dict.fromkeys(audit.COUNT_FIELDS, False) | dict(
                has_next=True,
                actual_reguard=True,
                next_fusion_available="false",
                current_entry_avoided=True,
            )
            result = audit.case_counts(
                meta,
                dict(status="timeout", joint_resumes=2),
                [following, dict(has_next=False)],
            )
            self.assertEqual(
                (
                    result["resumes"],
                    result["following"],
                    result["packet_endpoints"],
                    result["actual_reguards"],
                ),
                (2, 1, 1, 1),
            )
            cases.append(result)
            zero = audit.case_counts(
                dict(meta, case_id=arm + "-none", mode="none"),
                dict(status="held", joint_resumes=0),
                [],
            )
            self.assertFalse(any(zero[k] for k in audit.COUNT_FIELDS))
            cases.append(zero)
        summary = audit.summarize(cases)
        self.assertEqual(len(summary), 72)
        for cell in summary:
            if cell["outcome_selector"] == "all":
                parts = [
                    r
                    for r in summary
                    if all(
                        r[k] == cell[k]
                        for k in ("noise_selector", "mode_selector", "arm")
                    )
                    and r["outcome_selector"] != "all"
                ]
                for field in ("cases", *audit.COUNT_FIELDS):
                    self.assertEqual(cell[field], sum(p[field] for p in parts))
        self.assertEqual(len(audit.paired(cases)), 2)
        with self.assertRaises(ValueError):
            audit.paired(cases[:-1])

    def test_manifests_reject_corruption_and_unlisted_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            path = root / "evidence"
            path.write_text("original")
            records = dict(
                evidence=dict(bytes=path.stat().st_size, sha256=build.sha(path))
            )
            audit.verify_manifest(root, records)
            path.write_text("modified")
            with self.assertRaises(ValueError):
                audit.verify_manifest(root, records)
            path.write_text("original")
            (root / "extra").write_text("")
            with self.assertRaises(ValueError):
                audit.verify_manifest(root, records)

    def test_retained_census_join_rejects_missing_reordered_and_changed_inputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            work = root / "native"
            work.mkdir()
            queries = [old_query(1), old_query(2)]
            queries = [{k: str(v) for k, v in row.items()} for row in queries]
            geometries = [
                dict.fromkeys(census.TRACE_FIELDS, "0") | dict(id=str(i))
                for i in (1, 2)
            ]
            packet(work / "queries.tsv", queries, census.INPUT_FIELDS)
            pilot.write(work / "trace.csv", geometries, census.TRACE_FIELDS)
            selected = {1: queries[0]}
            self.assertEqual(set(audit.bind_geometry(root, selected, 2)), {1})
            with self.assertRaises(ValueError):
                audit.bind_geometry(root, {1: dict(queries[0], preview_x="999")}, 2)
            packet(work / "queries.tsv", list(reversed(queries)), census.INPUT_FIELDS)
            with self.assertRaises(ValueError):
                audit.bind_geometry(root, selected, 2)
            packet(work / "queries.tsv", queries[:1], census.INPUT_FIELDS)
            with self.assertRaises(ValueError):
                audit.bind_geometry(root, selected, 2)

    def test_frozen_inputs_and_cli_guards(self):
        self.assertEqual(len(audit.preflight()["input_sha256"]), 148)
        with tempfile.TemporaryDirectory() as tmp:
            for target in (Path(tmp), build.REPO / "forbidden-transition-output"):
                with self.assertRaises(ValueError):
                    build.fresh(target)
            output = Path(tmp) / "fresh"
            result = subprocess.run(
                [
                    sys.executable,
                    "-O",
                    str(build.ROOT / "analyze_post_release_transition.py"),
                    "--raw",
                    tmp,
                    "--census",
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
            with self.assertRaisesRegex(ValueError, "outside retained inputs"):
                audit.collect(tmp, tmp, output)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
