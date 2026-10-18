"""Exercise completed replay integrity checks without running the experiment."""

import base64
from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import check_reset_review as check

review = check.review
PNG = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+/l9sAAAAASUVORK5CYII="
)


def completed_fixture(path):
    """Copy tracked evidence and supply small PNGs with their own recorded hashes."""
    path.mkdir()
    manifest = review.read_json(review.STUDY / "review-sources.json")
    for name in check.TOP_FILES[:2]:
        shutil.copyfile(review.STUDY / name, path / name)
    for stage in manifest["stage_replays"]:
        work = path / "stages" / stage["stage"]
        reference = review.RESULTS / stage["study"]
        work.mkdir(parents=True)
        for key, folder in (
            ("collector_sha256", "collector"),
            ("plot_sha256", "figures"),
        ):
            target = work / folder
            target.mkdir()
            for name in stage[key]:
                output = target / name
                if name.endswith(".png"):
                    output.write_bytes(PNG)
                    stage[key][name] = review.sha(output)
                else:
                    shutil.copyfile(reference / name, output)
        shutil.copyfile(
            reference / "independent-verification.json",
            work / "independent-verification.json",
        )
        for name in check.LOG_FILES:
            (work / name).write_text("Original run log\n")
    (path / "review-sources.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


class CompletedReplay(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bundle = self.root / "received review"
        self.manifest = completed_fixture(self.bundle)
        self.work = self.bundle / "stages/run_breaks"
        self.manifest_path = self.bundle / "review-sources.json"

    def save_manifest(self, value=None):
        self.manifest_path.write_text(
            json.dumps(self.manifest if value is None else value)
        )

    def test_complete_packet_checks_64_artifacts_and_never_changes_files(self):
        def snapshot():
            return {
                str(p.relative_to(self.bundle)): (p.stat().st_mtime_ns, review.sha(p))
                for p in self.bundle.rglob("*")
                if p.is_file()
            }

        before = snapshot()
        with patch.object(
            review, "run_command", side_effect=AssertionError("No replay")
        ):
            result = check.check_bundle(self.bundle)
        self.assertEqual(result["stored_artifact_files"], 64)
        self.assertEqual(result["stored_stages"], 6)
        self.assertEqual(result["retained_resets"], 19)
        self.assertEqual(result["frozen_inputs"], 91)
        self.assertEqual(result["pngs_checked_against_bundle_manifest"], 6)
        self.assertEqual(result["manifest_sha256"], review.sha(self.manifest_path))
        self.assertEqual(snapshot(), before)

    def test_moved_packet_and_discarded_logs_are_supported(self):
        for path in self.bundle.glob("stages/*/*.log"):
            path.unlink()
        moved = self.root / "moved review"
        self.bundle.rename(moved)
        self.assertEqual(check.check_bundle(moved)["stored_artifact_files"], 64)

    def test_missing_manifest_or_artifact_never_counts_as_completed(self):
        paths = (
            self.manifest_path,
            self.bundle / "study-index.csv",
            self.work / "collector/reset-edges.csv",
            self.work / "figures/run-breaks.png",
            self.work / "independent-verification.json",
        )
        for path in paths:
            with self.subTest(path=path):
                original = path.read_bytes()
                path.unlink()
                try:
                    with self.assertRaisesRegex(ValueError, "missing"):
                        check.check_bundle(self.bundle)
                finally:
                    path.write_bytes(original)

    def test_unexpected_files_directories_and_internal_symlinks_are_rejected(self):
        for folder in (
            self.bundle,
            self.bundle / "stages",
            self.work,
            self.work / "collector",
            self.work / "figures",
        ):
            extra = folder / "unexpected"
            extra.mkdir()
            with self.subTest(folder=folder), self.assertRaisesRegex(
                ValueError, "extra"
            ):
                check.check_bundle(self.bundle)
            extra.rmdir()
        for path in (self.work / "collector.log", self.work / "figures/run-breaks.png"):
            original = path.read_bytes()
            outside = self.root / "external-file"
            outside.write_bytes(original)
            path.unlink()
            path.symlink_to(outside)
            with self.subTest(path=path), self.assertRaisesRegex(
                ValueError, "file type"
            ):
                check.check_bundle(self.bundle)
            path.unlink()
            path.write_bytes(original)
        collector = self.work / "collector"
        outside_dir = self.root / "external-directory"
        collector.rename(outside_dir)
        collector.symlink_to(outside_dir, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "file type"):
            check.check_bundle(self.bundle)

    def test_file_replaced_by_directory_is_rejected(self):
        path = self.work / "collector/reset-edges.csv"
        path.unlink()
        path.mkdir()
        with self.assertRaisesRegex(ValueError, "file type"):
            check.check_bundle(self.bundle)

    def test_changed_tables_plots_and_proofs_fail_even_with_updated_manifest_hashes(
        self,
    ):
        cases = (
            (
                self.bundle / "reset-evidence.csv",
                ("output_sha256", "reset-evidence.csv"),
            ),
            (
                self.work / "collector/reset-edges.csv",
                ("collector_sha256", "reset-edges.csv"),
            ),
            (self.work / "figures/run-breaks.svg", ("plot_sha256", "run-breaks.svg")),
            (
                self.work / "independent-verification.json",
                ("independent_proof_sha256",),
            ),
        )
        for path, keys in cases:
            with self.subTest(path=path):
                original = path.read_bytes()
                altered = deepcopy(self.manifest)
                path.write_bytes(original + b"\n")
                parent = (
                    altered
                    if keys[0] == "output_sha256"
                    else altered["stage_replays"][0]
                )
                for key in keys[:-1]:
                    parent = parent[key]
                parent[keys[-1]] = review.sha(path)
                self.save_manifest(altered)
                try:
                    with self.assertRaisesRegex(AssertionError, "Byte mismatch"):
                        check.check_bundle(self.bundle)
                finally:
                    path.write_bytes(original)
                    self.save_manifest()

    def test_png_hash_must_match_its_own_manifest_without_a_committed_image_baseline(
        self,
    ):
        path = self.work / "figures/run-breaks.png"
        path.write_bytes(PNG + b"different encoding")
        with self.assertRaisesRegex(ValueError, "Completion manifest"):
            check.check_bundle(self.bundle)
        self.manifest["stage_replays"][0]["plot_sha256"][path.name] = review.sha(path)
        self.save_manifest()
        self.assertEqual(check.check_bundle(self.bundle)["stored_artifact_files"], 64)

    def test_changed_provenance_counts_and_execution_claims_are_rejected(self):
        for key, value in (
            ("analysis", {}),
            ("input_sha256", {}),
            ("overview_fields", []),
            ("replayed_stages", 5),
            ("retained_resets", 18),
            ("fresh_source_observations", 1),
            ("fresh_raw_reads", 1),
            ("fresh_native_dataset_runs", 1),
            ("method", "different method"),
            ("unexpected", "extra metadata"),
        ):
            self.save_manifest(dict(self.manifest, **{key: value}))
            with self.subTest(key=key), self.assertRaisesRegex(
                ValueError, "Completion manifest"
            ):
                check.check_bundle(self.bundle)

    def test_stage_order_duplicates_and_missing_stages_are_rejected(self):
        stages = self.manifest["stage_replays"]
        for altered in (list(reversed(stages)), [stages[0]] * 6, stages[:-1]):
            self.save_manifest(dict(self.manifest, stage_replays=altered))
            with self.assertRaisesRegex(ValueError, "Completion manifest"):
                check.check_bundle(self.bundle)

    def test_manifest_paths_are_compared_as_claims_and_never_followed(self):
        self.manifest["stage_replays"][0]["collector_sha256"]["../../outside.csv"] = (
            "0" * 64
        )
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, "Completion manifest"):
            check.check_bundle(self.bundle)

    def test_duplicate_root_or_nested_json_keys_are_rejected(self):
        text = json.dumps(self.manifest)
        for changed in (
            '{"retained_resets": 19,' + text[1:],
            text.replace(
                '"script_sha256":', '"script_sha256": "duplicate", "script_sha256":', 1
            ),
        ):
            self.manifest_path.write_text(changed)
            with self.assertRaisesRegex(ValueError, "Duplicate JSON key"):
                check.check_bundle(self.bundle)

    def test_false_and_float_zero_cannot_replace_integer_zero(self):
        for value in (False, 0.0):
            self.save_manifest(dict(self.manifest, fresh_raw_reads=value))
            with self.assertRaisesRegex(ValueError, "Completion manifest"):
                check.check_bundle(self.bundle)

    def test_nonfinite_json_numbers_are_rejected(self):
        for value in (float("nan"), float("inf"), -float("inf")):
            self.save_manifest(dict(self.manifest, fresh_raw_reads=value))
            with self.assertRaisesRegex(ValueError, "Non-finite JSON"):
                check.check_bundle(self.bundle)

    def test_changed_recorded_manifest_or_runner_is_rejected(self):
        with patch.object(check, "RECORDED_REVIEW_SHA256", "0" * 64):
            with self.assertRaisesRegex(ValueError, "Recorded review manifest changed"):
                check.check_bundle(self.bundle)
        sha = review.sha
        runner = review.ROOT / "review_reset_evidence.py"
        with patch.object(
            review, "sha", side_effect=lambda p: "0" * 64 if p == runner else sha(p)
        ):
            with self.assertRaisesRegex(
                ValueError, "runner/protocol/catalogue changed"
            ):
                check.check_bundle(self.bundle)

    def test_cli_success_uses_only_the_standard_library(self):
        result = subprocess.run(
            [
                sys.executable,
                "-S",
                "-B",
                str(Path(check.__file__)),
                "--review",
                str(self.bundle),
            ],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["stored_artifact_files"], 64)
        self.assertEqual(result.stderr, "")

    def test_cli_failure_has_no_success_output_or_traceback(self):
        for contents in ('{"unfinished":', "[]"):
            self.manifest_path.write_text(contents)
            stdout, stderr = io.StringIO(), io.StringIO()
            with redirect_stdout(stdout), redirect_stderr(stderr):
                code = check.main(["--review", str(self.bundle)])
            self.assertEqual(code, 1)
            self.assertEqual(stdout.getvalue(), "")
            self.assertIn("Review check failed:", stderr.getvalue())
            self.assertNotIn("Traceback", stderr.getvalue())

    def test_optimized_python_rejects_before_inspecting_an_absent_packet(self):
        result = subprocess.run(
            [
                sys.executable,
                "-O",
                "-B",
                str(Path(check.__file__)),
                "--review",
                str(self.root / "absent"),
            ],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 1)
        self.assertEqual(result.stdout, "")
        self.assertIn("Run without Python optimization", result.stderr)


if __name__ == "__main__":
    unittest.main()
