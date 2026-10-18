"""Check portable provenance equivalence and fresh digest bindings."""

from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
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
        for status in (7, -9):
            target = self.root / ("failed-" + str(status))

            def fail(command, log):
                log.write_text("deliberate child failure")
                raise subprocess.CalledProcessError(status, command)

            out, err = io.StringIO(), io.StringIO()
            with patch.object(review, "run_command", side_effect=fail), redirect_stdout(
                out
            ), redirect_stderr(err):
                code = replay.main(["--output", str(target)])
            self.assertEqual(code, status if status > 0 else 128 - status)
            self.assertFalse((target / replay.MANIFEST).exists())
            self.assertEqual(
                (target / "stages/run_breaks/collector.log").read_text(),
                "deliberate child failure",
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
