#!/usr/bin/env python3
"""Check a completed reset replay's stored artifacts against this pinned checkout.

This read-only integrity check uses the standard library. Run
review_reset_evidence.py for fresh collector, plotter and numerical verification.
"""

import argparse
import json
from pathlib import Path
import sys

import review_reset_evidence as review

RECORDED_REVIEW_SHA256 = (
    "ba103d3933edb43a0ec543400e5c4c076a4f6a6c6fb13e40ec6de725f2fa546b"
)
TOP_FILES = ("reset-evidence.csv", "study-index.csv", "review-sources.json")
LOG_FILES = ("collector.log", "plotter.log", "verifier.log")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def reject_constant(value):
    raise ValueError(f"Non-finite JSON number: {value}")


def read_manifest(path):
    return json.loads(
        path.read_text(),
        object_pairs_hook=unique_object,
        parse_constant=reject_constant,
    )


def canonical(value):
    # Keep number types distinct: JSON false, 0 and 0.0 are different claims.
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def directory(path, files=(), directories=(), optional_files=()):
    require(path.is_dir() and not path.is_symlink(), f"Expected directory: {path}")
    entries = {entry.name: entry for entry in path.iterdir()}
    required = set(files) | set(directories)
    missing = required - set(entries)
    extra = set(entries) - required - set(optional_files)
    require(
        not missing and not extra,
        f"{path}: missing {sorted(missing)}, extra {sorted(extra)}",
    )
    for name, entry in entries.items():
        correct_type = entry.is_dir() if name in directories else entry.is_file()
        require(
            correct_type and not entry.is_symlink(), f"Unexpected file type: {entry}"
        )


def check_bundle(path):
    require(__debug__, "Run without Python optimization")
    path = Path(path).expanduser().resolve()
    directory(path, TOP_FILES, ("stages",))
    directory(path / "stages", directories=review.STEMS)
    pins, inputs = review.preflight()
    reference_manifest = review.STUDY / "review-sources.json"
    require(
        review.sha(reference_manifest) == RECORDED_REVIEW_SHA256,
        f"Recorded review manifest changed: {reference_manifest}",
    )
    expected = read_manifest(reference_manifest)
    require(expected["input_sha256"] == inputs, "Recorded review input pins changed")
    analysis = dict(
        script_sha256=review.sha(review.ROOT / "review_reset_evidence.py"),
        protocol_sha256=review.sha(review.STUDY / "protocol.md"),
        freeze_sha256=review.sha(review.STUDY / "freeze-pins.json"),
    )
    require(
        expected["analysis"] == analysis, "Recorded runner/protocol/catalogue changed"
    )
    for name in TOP_FILES[:2]:
        digest = review.same_file(review.STUDY / name, path / name)
        require(
            digest == expected["output_sha256"][name],
            f"Recorded export changed: {name}",
        )

    records = []
    artifact_files = len(TOP_FILES)
    for stem in review.STEMS:
        pinned = pins["studies"][stem]
        reference = review.RESULTS / pinned["study"]
        work = path / "stages" / stem
        directory(
            work,
            ("independent-verification.json",),
            ("collector", "figures"),
            LOG_FILES,
        )
        source = review.read_json(reference / "sources.json")
        plot = review.read_json(reference / "plot-sources.json")
        collector_names = [*source["output_sha256"], "sources.json"]
        plot_names = [*plot["output_sha256"], "plot-sources.json"]
        png = stem.replace("_", "-") + ".png"
        directory(work / "collector", collector_names)
        directory(work / "figures", [*plot_names, png])
        collector = review.match_directory(
            reference, work / "collector", collector_names
        )
        figures = review.match_directory(
            reference, work / "figures", plot_names, (png,)
        )
        proof = review.same_file(
            reference / "independent-verification.json",
            work / "independent-verification.json",
        )
        records.append(
            dict(
                stage=stem,
                study=pinned["study"],
                collector_sha256=collector,
                plot_sha256=figures,
                independent_proof_sha256=proof,
            )
        )
        artifact_files += len(collector) + len(figures) + 1
    # PNGs have no committed image baseline. Their actual hashes must match the
    # supplied completion manifest; all other outputs also match pinned bytes.
    expected["stage_replays"] = records
    manifest = path / "review-sources.json"
    require(
        canonical(read_manifest(manifest)) == canonical(expected),
        "Completion manifest differs from verified artifacts or pinned metadata",
    )
    return dict(
        check="stored replay integrity",
        stored_stages=len(records),
        stored_artifact_files=artifact_files,
        retained_resets=expected["retained_resets"],
        frozen_inputs=len(inputs),
        pngs_checked_against_bundle_manifest=len(records),
        manifest_sha256=review.sha(manifest),
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--review", type=Path, required=True, help="Completed replay directory"
    )
    args = parser.parse_args(argv)
    try:
        result = check_bundle(args.review)
    except (OSError, ValueError, AssertionError) as error:
        print(
            f"Review check failed: {error or 'Frozen provenance mismatch'}",
            file=sys.stderr,
        )
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
