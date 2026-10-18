"""Check portable provenance equivalence and fresh digest bindings."""

from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import ci_reset_replay as replay
from test_check_reset_review import completed_fixture

review = replay.review


def reversed_objects(value):
    if isinstance(value, dict):
        return {k: reversed_objects(v) for k, v in reversed(list(value.items()))}
    if isinstance(value, list):
        return [reversed_objects(v) for v in value]
    return value


def save(path, value):
    path.write_text(json.dumps(reversed_objects(value), indent=2) + "\n")


class PortableReplay(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.packet = self.root / "CI packet"
        original = completed_fixture(self.packet)
        (self.packet / "review-sources.json").unlink()
        for stage in original["stage_replays"]:
            work = self.packet / "stages" / stage["stage"]
            source = work / "collector/sources.json"
            save(source, review.read_json(source))
            plot = work / "figures/plot-sources.json"
            metadata = review.read_json(plot)
            metadata["input_sha256"]["sources.json"] = review.sha(source)
            save(plot, metadata)
            proof = work / "independent-verification.json"
            metadata = review.read_json(proof)
            metadata["sources_sha256"] = review.sha(source)
            metadata["plot_sources_sha256"] = review.sha(plot)
            save(proof, metadata)
        self.pins, self.inputs = review.preflight()
        records, index = replay.inspect_stages(self.packet, self.pins)
        review.write_csv(self.packet / "study-index.csv", index, review.INDEX_FIELDS)
        save(
            self.packet / replay.MANIFEST,
            replay.manifest(self.packet, records, self.inputs),
        )
        self.work = self.packet / "stages/run_breaks"

    def fixture_child(self, calls, after_phase=None):
        def execute(command, log):
            stem, phase = log.parent.name, log.stem
            calls.append((stem, phase))
            target = Path(command[command.index("--output") + 1])
            source = self.packet / "stages" / stem
            if phase == "verifier":
                shutil.copyfile(source / "independent-verification.json", target)
            else:
                folder = "collector" if phase == "collector" else "figures"
                shutil.copytree(source / folder, target)
            log.write_text("Completed fixture phase\n")
            if after_phase:
                after_phase(stem, phase, log.parent)

        return execute

    def test_changed_review_references_stop_before_creating_output_or_children(self):
        cases = (
            ("protocol.md", None),
            ("freeze-pins.json", None),
            ("freeze-pins.json", b"{}\n"),
            ("reset-evidence.csv", None),
            ("study-index.csv", None),
            ("review-sources.json", None),
        )
        for index, (name, replacement) in enumerate(cases):
            with self.subTest(name=name, replacement=replacement):
                study = self.root / f"changed reference {index}"
                shutil.copytree(review.STUDY, study)
                path = study / name
                path.write_bytes(
                    replacement
                    if replacement is not None
                    else path.read_bytes() + b"\n"
                )
                target = self.root / f"rejected replay {index}"
                calls = []
                out, err = io.StringIO(), io.StringIO()
                with patch.object(review, "STUDY", study), patch.object(
                    review, "run_command", side_effect=self.fixture_child(calls)
                ), redirect_stdout(out), redirect_stderr(err):
                    code = replay.main(["--output", str(target)])
                self.assertEqual(code, 1)
                self.assertEqual(calls, [])
                self.assertFalse(target.exists())
                self.assertIn(name, err.getvalue())

    def test_stored_packet_cannot_ignore_changed_protocol_or_recorded_index(self):
        for name in ("protocol.md", "study-index.csv"):
            with self.subTest(name=name):
                study = self.root / ("changed " + name)
                shutil.copytree(review.STUDY, study)
                path = study / name
                path.write_bytes(path.read_bytes() + b"\n")
                with patch.object(review, "STUDY", study):
                    with self.assertRaises(ValueError) as error:
                        replay.check_packet(self.packet)
                self.assertIn(name, str(error.exception))

    def test_reference_changes_during_replay_prevent_completion(self):
        for name in ("protocol.md", "freeze-pins.json"):
            with self.subTest(name=name):
                study = self.root / ("later change " + name)
                shutil.copytree(review.STUDY, study)
                path = study / name
                target = self.root / ("incomplete " + name)
                calls = []

                def change_reference(stem, phase, _work):
                    if stem == review.STEMS[-1] and phase == "verifier":
                        path.write_bytes(path.read_bytes() + b"\n")

                out, err = io.StringIO(), io.StringIO()
                with patch.object(review, "STUDY", study), patch.object(
                    review,
                    "run_command",
                    side_effect=self.fixture_child(calls, change_reference),
                ), redirect_stdout(out), redirect_stderr(err):
                    code = replay.main(["--output", str(target)])
                self.assertEqual(code, 1)
                self.assertEqual(len(calls), 18)
                self.assertFalse((target / replay.MANIFEST).exists())
                self.assertIn(name, err.getvalue())

    def test_first_and_middle_stage_mismatches_stop_before_the_next_stage(self):
        for bad_index in (0, 2):
            for corruption in ("csv", "json"):
                bad_stem = review.STEMS[bad_index]
                target = self.root / f"bad-stage-{bad_index}-{corruption}"
                calls = []

                def corrupt(stem, phase, work):
                    if stem == bad_stem and phase == "verifier":
                        if corruption == "csv":
                            path = next(
                                p
                                for p in (work / "collector").iterdir()
                                if p.suffix == ".csv"
                            )
                            path.write_bytes(path.read_bytes() + b"\n")
                        else:
                            path = work / "collector/sources.json"
                            source = review.read_json(path)
                            source["audit"]["fresh_raw_reads"] = 1
                            save(path, source)

                out, err = io.StringIO(), io.StringIO()
                with patch.object(
                    review,
                    "run_command",
                    side_effect=self.fixture_child(calls, corrupt),
                ), redirect_stdout(out), redirect_stderr(err):
                    code = replay.main(["--output", str(target)])
                with self.subTest(stage=bad_stem, corruption=corruption):
                    self.assertEqual(code, 1)
                    self.assertEqual(
                        calls,
                        [
                            (s, p)
                            for s in review.STEMS[: bad_index + 1]
                            for p in ("collector", "plotter", "verifier")
                        ],
                    )
                    self.assertIn(bad_stem, err.getvalue())
                    self.assertFalse(
                        (target / "stages" / review.STEMS[bad_index + 1]).exists()
                    )
                    self.assertFalse((target / replay.MANIFEST).exists())
                    self.assertEqual(
                        (target / "stages" / bad_stem / "verifier.log").read_text(),
                        "Completed fixture phase\n",
                    )

    def test_all_valid_stages_publish_a_packet_that_passes_the_read_only_check(self):
        target = self.root / "successful-run"
        calls = []
        out = io.StringIO()
        with patch.object(
            review, "run_command", side_effect=self.fixture_child(calls)
        ), redirect_stdout(out):
            code = replay.main(["--output", str(target)])
        self.assertEqual(code, 0)
        self.assertEqual(
            calls,
            [
                (s, p)
                for s in review.STEMS
                for p in ("collector", "plotter", "verifier")
            ],
        )
        self.assertEqual(
            json.loads(out.getvalue().splitlines()[-1])["completion_manifest"],
            replay.MANIFEST,
        )
        self.assertEqual(replay.check_packet(target)["stored_artifact_files"], 64)

    def test_final_snapshot_detects_a_later_child_changing_a_validated_stage(self):
        target = self.root / "late-change"
        calls = []

        def change_earlier_stage(stem, phase, work):
            if stem == review.STEMS[-1] and phase == "verifier":
                path = work.parent / "run_breaks/collector/reset-edges.csv"
                path.write_bytes(path.read_bytes() + b"\n")

        out, err = io.StringIO(), io.StringIO()
        with patch.object(
            review,
            "run_command",
            side_effect=self.fixture_child(calls, change_earlier_stage),
        ), redirect_stdout(out), redirect_stderr(err):
            code = replay.main(["--output", str(target)])
        self.assertEqual(code, 1)
        self.assertEqual(len(calls), 18)
        self.assertIn("Byte mismatch", err.getvalue())
        self.assertIn("run_breaks", err.getvalue())
        self.assertFalse((target / replay.MANIFEST).exists())

    def test_all_nested_object_orders_can_change_with_identical_values(self):
        result = replay.check_packet(self.packet)
        self.assertEqual(result["stored_artifact_files"], 64)
        self.assertEqual(result["retained_resets"], 19)
        self.assertEqual(result["stored_stages"], 6)
        fresh = self.work / "collector/sources.json"
        frozen = review.RESULTS / "fusion-return-run-breaks/sources.json"
        self.assertNotEqual(fresh.read_bytes(), frozen.read_bytes())
        self.assertEqual(review.read_json(fresh), review.read_json(frozen))

    def test_stale_source_and_plot_digest_bindings_are_rejected(self):
        paths = (
            self.work / "figures/plot-sources.json",
            self.work / "independent-verification.json",
        )
        for path in paths:
            original = path.read_bytes()
            for field in (
                ("sources.json",)
                if path.name == "plot-sources.json"
                else ("sources_sha256", "plot_sources_sha256")
            ):
                metadata = json.loads(original)
                if field == "sources.json":
                    metadata["input_sha256"][field] = self.pins["studies"][
                        "run_breaks"
                    ]["sources_sha256"]
                else:
                    key = (
                        "sources_sha256"
                        if field == "sources_sha256"
                        else "plot_sources_sha256"
                    )
                    metadata[field] = self.pins["studies"]["run_breaks"][key]
                save(path, metadata)
                with self.subTest(path=path, field=field), self.assertRaisesRegex(
                    ValueError, "JSON content mismatch"
                ):
                    replay.check_packet(self.packet)
            path.write_bytes(original)

    def test_arithmetic_audit_methods_and_script_claims_remain_frozen(self):
        cases = (
            (self.work / "collector/sources.json", "audit", {"fresh_raw_reads": 1}),
            (self.work / "independent-verification.json", "comparisons", -1),
            (self.work / "independent-verification.json", "method", "changed"),
            (self.work / "independent-verification.json", "script_sha256", "0" * 64),
            (self.work / "figures/plot-sources.json", "script_sha256", "0" * 64),
        )
        for path, field, value in cases:
            original = path.read_bytes()
            metadata = json.loads(original)
            metadata[field] = value
            save(path, metadata)
            with self.subTest(path=path), self.assertRaisesRegex(
                ValueError, "JSON content mismatch"
            ):
                replay.check_packet(self.packet)
            path.write_bytes(original)

    def test_stage_logs_are_optional_but_extra_artifacts_are_rejected(self):
        for path in self.packet.glob("stages/*/*.log"):
            path.unlink()
        self.assertEqual(replay.check_packet(self.packet)["stored_artifact_files"], 64)
        (self.work / "figures/unexpected.txt").write_text("extra artifact")
        with self.assertRaisesRegex(ValueError, "extra"):
            replay.check_packet(self.packet)

    def test_csv_svg_and_overview_bytes_remain_exact(self):
        for path in (
            self.packet / "reset-evidence.csv",
            self.work / "collector/reset-edges.csv",
            self.work / "figures/run-breaks.svg",
        ):
            original = path.read_bytes()
            path.write_bytes(original + b"\n")
            with self.subTest(path=path), self.assertRaisesRegex(
                AssertionError, "Byte mismatch"
            ):
                replay.check_packet(self.packet)
            path.write_bytes(original)

    def test_index_must_reference_actual_fresh_hashes(self):
        path = self.packet / "study-index.csv"
        source = self.work / "collector/sources.json"
        path.write_text(
            path.read_text().replace(
                review.sha(source), self.pins["studies"]["run_breaks"]["sources_sha256"]
            )
        )
        with self.assertRaisesRegex(ValueError, "Study index"):
            replay.check_packet(self.packet)

    def test_manifest_counts_types_and_extra_claims_are_rejected(self):
        path = self.packet / replay.MANIFEST
        original = review.read_json(path)
        for key, value in (
            ("replayed_stages", 5),
            ("fresh_raw_reads", False),
            ("fresh_native_dataset_runs", 1),
            ("method", "changed"),
            ("unexpected", 1),
        ):
            save(path, dict(original, **{key: value}))
            with self.subTest(key=key), self.assertRaisesRegex(
                ValueError, "JSON content mismatch"
            ):
                replay.check_packet(self.packet)

    def test_duplicate_json_keys_and_list_order_changes_are_rejected(self):
        path = self.work / "collector/sources.json"
        original = path.read_text()
        path.write_text('{"input_sha256": {},' + original[1:])
        with self.assertRaisesRegex(ValueError, "Duplicate JSON key"):
            replay.check_packet(self.packet)
        path.write_text(original)
        path = self.packet / replay.MANIFEST
        metadata = review.read_json(path)
        metadata["stage_replays"].reverse()
        save(path, metadata)
        with self.assertRaisesRegex(ValueError, "JSON content mismatch"):
            replay.check_packet(self.packet)

    def test_png_changes_require_an_updated_actual_manifest(self):
        path = self.work / "figures/run-breaks.png"
        path.write_bytes(path.read_bytes() + b"encoding variation")
        with self.assertRaisesRegex(ValueError, "JSON content mismatch"):
            replay.check_packet(self.packet)
        records, _ = replay.inspect_stages(self.packet, self.pins)
        save(
            self.packet / replay.MANIFEST,
            replay.manifest(self.packet, records, self.inputs),
        )
        self.assertEqual(replay.check_packet(self.packet)["stored_artifact_files"], 64)

    def test_check_is_read_only_and_needs_only_the_standard_library(self):
        before = {
            p: (p.stat().st_mtime_ns, review.sha(p))
            for p in self.packet.rglob("*")
            if p.is_file()
        }
        result = subprocess.run(
            [sys.executable, "-S", "-B", replay.__file__, "--check", str(self.packet)],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["stored_artifact_files"], 64)
        self.assertEqual(result.stderr, "")
        self.assertEqual(
            before,
            {
                p: (p.stat().st_mtime_ns, review.sha(p))
                for p in self.packet.rglob("*")
                if p.is_file()
            },
        )

    def test_child_status_logs_and_incomplete_packet_are_preserved(self):
        phases = ("collector", "plotter", "verifier")
        all_commands = [(stem, phase) for stem in review.STEMS for phase in phases]
        payload = b"deliberate child failure\nraw output: \xff\n"
        for stage_index in (0, 2):
            failed_stem = review.STEMS[stage_index]
            for phase_index, failed_phase in enumerate(phases):
                for status in (7, -9):
                    with self.subTest(
                        stage=failed_stem, phase=failed_phase, status=status
                    ):
                        target = (
                            self.root / f"failed {failed_stem} {failed_phase} {status}"
                        )
                        calls = []

                        def fail(stem, phase, work):
                            if (stem, phase) == (failed_stem, failed_phase):
                                (work / (phase + ".log")).write_bytes(payload)
                                raise subprocess.CalledProcessError(
                                    status, ["fixture child"]
                                )

                        out, err = io.StringIO(), io.StringIO()
                        with patch.object(
                            review,
                            "run_command",
                            side_effect=self.fixture_child(calls, fail),
                        ), redirect_stdout(out), redirect_stderr(err):
                            code = replay.main(
                                ["--output", str(target / ".." / target.name)]
                            )
                        self.assertEqual(code, status if status > 0 else 128 - status)
                        completed = stage_index * len(phases) + phase_index + 1
                        self.assertEqual(calls, all_commands[:completed])
                        log = target / "stages" / failed_stem / (failed_phase + ".log")
                        self.assertIn(f"status {status}", err.getvalue())
                        self.assertIn(f"{failed_stem}: {failed_phase}", err.getvalue())
                        self.assertIn(str(log), err.getvalue())
                        self.assertEqual(log.read_bytes(), payload)
                        self.assertFalse((target / replay.MANIFEST).exists())
                        next_stem, next_phase = all_commands[completed]
                        self.assertFalse(
                            (
                                target / "stages" / next_stem / (next_phase + ".log")
                            ).exists()
                        )

    def test_optimized_python_rejects_both_modes_without_creating_output(self):
        for mode in ("--output", "--check"):
            target = self.root / "absent"
            result = subprocess.run(
                [sys.executable, "-O", "-B", replay.__file__, mode, str(target)],
                capture_output=True,
                text=True,
            )
            self.assertEqual(result.returncode, 1)
            self.assertIn("Run without Python optimization", result.stderr)
            self.assertFalse(target.exists())


if __name__ == "__main__":
    unittest.main()
