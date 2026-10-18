"""Check exact review exports, strict evidence matching and runner failure behavior."""

from contextlib import redirect_stderr
from copy import deepcopy
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import review_reset_evidence as r


def row(edge):
    result = {field: "value-" + field for field in r.OVERVIEW_FIELDS}
    result.update(
        edge_id=edge,
        source_time_s="001.2300",
        required_fraction_lo="",
        required_fraction_hi="",
        surplus_fraction_lo="",
        surplus_fraction_hi="",
    )
    result["extra_source_column"] = "retained in full study"
    return result


class ExactOverview(unittest.TestCase):
    def test_strings_blanks_and_original_row_order_are_preserved(self):
        originals = [row("C2@5"), row("C1@6")]
        projected = r.project_overview(originals)
        self.assertEqual([x["edge_id"] for x in projected], ["C2@5", "C1@6"])
        self.assertEqual(projected[0]["source_time_s"], "001.2300")
        self.assertEqual(projected[0]["required_fraction_lo"], "")
        self.assertTrue(all(tuple(x) == r.OVERVIEW_FIELDS for x in projected))
        self.assertTrue(
            all(
                x[f] == old[f]
                for x, old in zip(projected, originals)
                for f in r.OVERVIEW_FIELDS
            )
        )
        self.assertIn("extra_source_column", originals[0])

    def test_duplicate_missing_or_nonstring_inputs_are_rejected(self):
        original = row("C1@5")
        missing = original.copy()
        del missing["failed_gates"]
        for inputs in (
            [original, original],
            [missing],
            [dict(original, source_step=5)],
        ):
            with self.assertRaises(AssertionError):
                r.project_overview(inputs)

    def test_csv_round_trip_keeps_empty_values_and_exact_literals(self):
        projected = r.project_overview([row("C1@5")])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "review.csv"
            r.write_csv(path, projected, r.OVERVIEW_FIELDS)
            self.assertEqual(r.read_csv(path), projected)

    def test_duplicate_header_extra_or_truncated_cells_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "input.csv"
            for text in ("a,a\n1,2\n", "a,b\n1,2,3\n", "a,b\n1\n"):
                path.write_text(text)
                with self.assertRaises(AssertionError):
                    r.read_csv(path)


class StrictEvidenceBytes(unittest.TestCase):
    def test_expected_files_match_and_unbaselined_png_is_hashed(self):
        with tempfile.TemporaryDirectory() as directory:
            base, replay = Path(directory) / "base", Path(directory) / "replay"
            base.mkdir()
            replay.mkdir()
            for folder in (base, replay):
                (folder / "values.csv").write_text("value\n001.2000\n")
            (replay / "figure.png").write_bytes(b"png-fixture")
            hashes = r.match_directory(base, replay, ["values.csv"], ["figure.png"])
            self.assertEqual(
                hashes, {n: r.sha(replay / n) for n in ("figure.png", "values.csv")}
            )

    def test_numerically_equal_csv_or_json_reformat_is_not_byte_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            left, right = Path(directory) / "left", Path(directory) / "right"
            for a, b in (
                ("value\n001.2000\n", "value\n1.2\n"),
                ('{"a":1}\n', '{ "a": 1 }\n'),
            ):
                left.write_text(a)
                right.write_text(b)
                with self.assertRaises(AssertionError):
                    r.same_file(left, right)

    def test_missing_extra_or_directory_outputs_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            base, replay = Path(directory) / "base", Path(directory) / "replay"
            base.mkdir()
            replay.mkdir()
            (base / "values.csv").write_text("a\n1\n")
            with self.assertRaises(AssertionError):
                r.match_directory(base, replay, ["values.csv"])
            (replay / "values.csv").write_text("a\n1\n")
            (replay / "unexpected.log").write_text("extra")
            with self.assertRaises(AssertionError):
                r.match_directory(base, replay, ["values.csv"])
            (replay / "unexpected.log").unlink()
            (replay / "nested").mkdir()
            with self.assertRaises(AssertionError):
                r.match_directory(base, replay, ["values.csv"])


class FrozenMetadata(unittest.TestCase):
    def fixtures(self):
        pinned = dict(
            sources_sha256="source",
            plot_sources_sha256="plot",
            scripts_sha256={
                "analyze_example.py": "collector",
                "verify_example.py": "verifier",
                "plot_example.py": "plotter",
            },
        )
        source = dict(
            analysis=dict(script_sha256="collector"),
            audit=dict(fresh_raw_reads=0, fresh_native_runs=0),
        )
        proof = dict(
            script_sha256="verifier",
            sources_sha256="source",
            plot_sources_sha256="plot",
            fresh_raw_reads=0,
            fresh_native_runs=0,
        )
        plot = dict(script_sha256="plotter")
        return pinned, source, proof, plot

    def test_changed_script_or_proof_source_metadata_is_rejected(self):
        pinned, source, proof, plot = self.fixtures()
        r.validate_metadata("example", pinned, source, proof, plot)
        for index, path in (
            (0, ("analysis", "script_sha256")),
            (1, ("script_sha256",)),
            (1, ("sources_sha256",)),
            (1, ("plot_sources_sha256",)),
            (2, ("script_sha256",)),
        ):
            copies = deepcopy([source, proof, plot])
            parent = copies[index]
            for key in path[:-1]:
                parent = parent[key]
            parent[path[-1]] = "changed"
            with self.assertRaises(AssertionError):
                r.validate_metadata("example", pinned, *copies)

    def test_nonzero_raw_or_native_audit_cannot_claim_zero_execution(self):
        pinned, source, proof, plot = self.fixtures()
        for field in ("fresh_raw_reads", "fresh_native_runs"):
            for target in ("source", "proof"):
                copies = deepcopy([source, proof, plot])
                if target == "source":
                    copies[0]["audit"][field] = 1
                else:
                    copies[1][field] = 1
                with self.assertRaises(AssertionError):
                    r.validate_metadata("example", pinned, *copies)


class SafeRunnerExecution(unittest.TestCase):
    def test_fresh_external_destination_is_resolved_without_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            repo = root / "repository"
            repo.mkdir()
            target = root / "new output" / "review"
            self.assertEqual(r.check_destination(target, repo), target.resolve())
            self.assertFalse(target.exists())

    def test_existing_inside_repository_and_symlinked_destinations_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            repo = root / "repository"
            repo.mkdir()
            alias = root / "alias"
            alias.symlink_to(repo, target_is_directory=True)
            for path in (root, repo / "fresh", alias / "fresh"):
                with self.assertRaises(ValueError):
                    r.check_destination(path, repo)

    def test_argument_vector_preserves_spaces_and_logs_both_streams(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "child log.txt"
            r.run_command(
                [
                    sys.executable,
                    "-c",
                    "import sys; print(sys.argv[1]); print('stderr-value', file=sys.stderr)",
                    "argument with spaces",
                ],
                path,
            )
            text = path.read_text()
            self.assertIn("argument with spaces\n", text)
            self.assertIn("stderr-value\n", text)

    def test_original_child_failure_code_and_output_are_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "failure.log"
            with self.assertRaises(subprocess.CalledProcessError) as caught:
                r.run_command(
                    [
                        sys.executable,
                        "-c",
                        "print('before failure', flush=True); raise SystemExit(7)",
                    ],
                    path,
                )
            self.assertEqual(caught.exception.returncode, 7)
            self.assertEqual(path.read_text(), "before failure\n")

    def test_failed_stage_never_publishes_completion_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "review"
            pins = dict(studies={s: {} for s in r.STEMS})
            with patch.object(
                sys, "argv", ["review", "--output", str(target)]
            ), patch.object(r, "preflight", return_value=(pins, {})), patch.object(
                r,
                "replay_stage",
                side_effect=subprocess.CalledProcessError(7, ["failure"]),
            ), redirect_stderr(
                io.StringIO()
            ):
                self.assertEqual(r.main(), 7)
            self.assertFalse((target / "review-sources.json").exists())
            self.assertFalse((target / "reset-evidence.csv").exists())

    def test_optimized_python_is_rejected_before_creating_output(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "review"
            completed = subprocess.run(
                [
                    sys.executable,
                    "-O",
                    "-B",
                    str(r.ROOT / "review_reset_evidence.py"),
                    "--output",
                    str(target),
                ],
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 2)
            self.assertIn("optimization", completed.stderr)
            self.assertFalse(target.exists())


if __name__ == "__main__":
    unittest.main()
