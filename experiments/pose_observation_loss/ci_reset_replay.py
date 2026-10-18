#!/usr/bin/env python3
"""Replay reset evidence or check a saved CI packet, allowing JSON key ordering.

CSV/SVG bytes remain frozen. JSON content remains frozen with source/plot
digests bound to the actual generated files. Generated files are never reformatted.
"""

import argparse
import csv
import io
import json
from pathlib import Path
import subprocess
import sys

import check_reset_review as integrity

review = integrity.review
MANIFEST = "ci-replay-sources.json"
TOP_FILES = ("reset-evidence.csv", "study-index.csv", MANIFEST)
METHOD = "fresh six-stage numerical replay; byte-identical CSV/SVG; strict JSON content equivalence with actual source/plot digest bindings; no output rewriting"


def equivalent(path, expected):
    actual = integrity.read_manifest(path)
    integrity.require(
        integrity.canonical(actual) == integrity.canonical(expected),
        f"JSON content mismatch: {path}",
    )
    return actual


def csv_bytes(rows, fields):
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().encode()


def preflight():
    reference = review.STUDY / "review-sources.json"
    integrity.require(
        review.sha(reference) == integrity.RECORDED_REVIEW_SHA256,
        f"Recorded reset review changed: {reference}",
    )
    recorded = integrity.read_manifest(reference)
    for field, path in (
        ("script_sha256", review.ROOT / "review_reset_evidence.py"),
        ("protocol_sha256", review.STUDY / "protocol.md"),
        ("freeze_sha256", review.STUDY / "freeze-pins.json"),
    ):
        integrity.require(
            review.sha(path) == recorded["analysis"][field],
            f"Recorded review input changed: {path}",
        )
    for name, digest in recorded["output_sha256"].items():
        path = review.STUDY / name
        integrity.require(
            review.sha(path) == digest, f"Recorded review export changed: {path}"
        )
    pins, inputs = review.preflight()
    integrity.require(
        inputs == recorded["input_sha256"], "Recorded reset input pins changed"
    )
    return pins, inputs


def inspect_stage(output, stem, pinned):
    reference = review.RESULTS / pinned["study"]
    work = output / "stages" / stem
    integrity.directory(
        work,
        ("independent-verification.json",),
        ("collector", "figures"),
        integrity.LOG_FILES,
    )
    source = review.read_json(reference / "sources.json")
    plot = review.read_json(reference / "plot-sources.json")
    proof = review.read_json(reference / "independent-verification.json")
    png = stem.replace("_", "-") + ".png"
    integrity.directory(work / "collector", [*source["output_sha256"], "sources.json"])
    integrity.directory(
        work / "figures", [*plot["output_sha256"], "plot-sources.json", png]
    )
    collector_hashes = review.match_directory(
        reference, work / "collector", source["output_sha256"], ("sources.json",)
    )
    plot_hashes = review.match_directory(
        reference,
        work / "figures",
        plot["output_sha256"],
        ("plot-sources.json", png),
    )
    equivalent(work / "collector/sources.json", source)
    plot["input_sha256"]["sources.json"] = collector_hashes["sources.json"]
    equivalent(work / "figures/plot-sources.json", plot)
    proof["sources_sha256"] = collector_hashes["sources.json"]
    proof["plot_sources_sha256"] = plot_hashes["plot-sources.json"]
    equivalent(work / "independent-verification.json", proof)
    proof_hash = review.sha(work / "independent-verification.json")
    record = dict(
        stage=stem,
        study=pinned["study"],
        collector_sha256=collector_hashes,
        plot_sha256=plot_hashes,
        independent_proof_sha256=proof_hash,
        frozen_sources_sha256=pinned["sources_sha256"],
        frozen_plot_sources_sha256=pinned["plot_sources_sha256"],
        frozen_proof_sha256=pinned["proof_sha256"],
    )
    entry = dict(
        study=pinned["study"],
        collector_script=f"analyze_{stem}.py",
        verifier_script=f"verify_{stem}.py",
        plotter_script=f"plot_{stem}.py",
        report_path=str((reference / "report.md").relative_to(review.REPO)),
        sources_sha256=collector_hashes["sources.json"],
        proof_sha256=proof_hash,
        plot_sources_sha256=plot_hashes["plot-sources.json"],
        replayed_collector_files=len(collector_hashes),
        replayed_plot_files=len(plot_hashes),
        independent_comparisons=proof["comparisons"],
        original_identity_comparisons=proof["original_join_comparisons"],
        audit_json=json.dumps(source["audit"], sort_keys=True, separators=(",", ":")),
    )
    return record, entry


def inspect_stages(output, pins):
    integrity.directory(output / "stages", directories=review.STEMS)
    records, index = [], []
    for stem in review.STEMS:
        record, entry = inspect_stage(output, stem, pins["studies"][stem])
        records.append(record)
        index.append(entry)
    return records, index


def manifest(output, records, inputs):
    reference = review.STUDY / "review-sources.json"
    integrity.require(
        review.sha(reference) == integrity.RECORDED_REVIEW_SHA256,
        "Recorded reset review changed",
    )
    return dict(
        analysis=dict(
            script_sha256=review.sha(Path(__file__)),
            helper_sha256={
                name: review.sha(review.ROOT / name)
                for name in ("review_reset_evidence.py", "check_reset_review.py")
            },
            freeze_sha256=review.sha(review.STUDY / "freeze-pins.json"),
            recorded_review_sha256=review.sha(reference),
            requirements_sha256={
                name: review.sha(review.ROOT / name)
                for name in ("requirements-ci.txt", "requirements-replay.txt")
            },
        ),
        input_sha256=inputs,
        stage_replays=records,
        overview_fields=review.OVERVIEW_FIELDS,
        retained_resets=19,
        replayed_stages=len(records),
        fresh_source_observations=0,
        fresh_raw_reads=0,
        fresh_native_dataset_runs=0,
        method=METHOD,
        output_sha256={name: review.sha(output / name) for name in TOP_FILES[:2]},
    )


def publish_manifest(output, value):
    temporary = output / ("." + MANIFEST + ".tmp")
    stream = temporary.open("x", encoding="utf-8")
    try:
        with stream:
            stream.write(json.dumps(value, indent=2) + "\n")
        temporary.replace(output / MANIFEST)
    finally:
        temporary.unlink(missing_ok=True)


def check_packet(output):
    integrity.require(__debug__, "Run without Python optimization")
    output = Path(output).expanduser().resolve()
    integrity.directory(output, TOP_FILES, ("stages",))
    pins, inputs = preflight()
    records, index = inspect_stages(output, pins)
    review.same_file(review.STUDY / "reset-evidence.csv", output / "reset-evidence.csv")
    integrity.require(
        (output / "study-index.csv").read_bytes()
        == csv_bytes(index, review.INDEX_FIELDS),
        "Study index differs from actual stage evidence",
    )
    equivalent(output / MANIFEST, manifest(output, records, inputs))
    return dict(
        check="stored CI replay integrity",
        stored_stages=len(records),
        stored_artifact_files=len(TOP_FILES)
        + sum(len(r["collector_sha256"]) + len(r["plot_sha256"]) + 1 for r in records),
        retained_resets=19,
        frozen_inputs=len(inputs),
        manifest_sha256=review.sha(output / MANIFEST),
    )


def run(output):
    integrity.require(__debug__, "Run without Python optimization")
    output = review.check_destination(output)
    pins, inputs = preflight()
    output.mkdir(parents=True)
    for stem in review.STEMS:
        work = output / "stages" / stem
        work.mkdir(parents=True)
        collector, figures = work / "collector", work / "figures"
        for phase, args in (
            ("collector", [f"analyze_{stem}.py", "--output", str(collector)]),
            (
                "plotter",
                [
                    f"plot_{stem}.py",
                    "--results",
                    str(collector),
                    "--output",
                    str(figures),
                ],
            ),
            (
                "verifier",
                [
                    f"verify_{stem}.py",
                    "--results",
                    str(collector),
                    "--figures",
                    str(figures),
                    "--output",
                    str(work / "independent-verification.json"),
                ],
            ),
        ):
            print(f"{stem}: {phase}", flush=True)
            log = work / (phase + ".log")
            try:
                review.run_command(
                    [sys.executable, "-B", str(review.ROOT / args[0]), *args[1:]],
                    log,
                )
            except subprocess.CalledProcessError as error:
                print(
                    f"Replay child failed with status {error.returncode} at {stem}: {phase}; inspect {log}",
                    file=sys.stderr,
                )
                raise
        inspect_stage(output, stem, pins["studies"][stem])
        integrity.directory(output, directories=("stages",))
    # Recheck the final snapshot as well: later children could change earlier files.
    records, index = inspect_stages(output, pins)
    rows = review.project_overview(
        review.read_csv(output / "stages/reset_surplus/collector/surplus.csv")
    )
    integrity.require(len(rows) == 19, "Expected all 19 reset rows")
    review.write_csv(output / "reset-evidence.csv", rows, review.OVERVIEW_FIELDS)
    review.write_csv(output / "study-index.csv", index, review.INDEX_FIELDS)
    review.same_file(review.STUDY / "reset-evidence.csv", output / "reset-evidence.csv")
    integrity.require(
        preflight() == (pins, inputs), "Frozen inputs changed during replay"
    )
    publish_manifest(output, manifest(output, records, inputs))
    return dict(
        replayed_stages=len(records),
        retained_resets=len(rows),
        completion_manifest=MANIFEST,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--output", type=Path, help="Fresh external directory for a full replay"
    )
    mode.add_argument("--check", type=Path, help="Read-only check of a saved CI replay")
    args = parser.parse_args(argv)
    try:
        result = (
            run(args.output) if args.output is not None else check_packet(args.check)
        )
    except subprocess.CalledProcessError as error:
        return error.returncode if error.returncode > 0 else 128 - error.returncode
    except (OSError, ValueError, AssertionError) as error:
        print(
            f"CI replay failed: {error or 'Frozen evidence mismatch'}", file=sys.stderr
        )
        return 1
    print(json.dumps(result, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
