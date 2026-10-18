#!/usr/bin/env python3
"""Replay six pinned reset audits and export their existing evidence for review."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
REPO, RESULTS = ROOT.parents[1], ROOT / "results"
STUDY = RESULTS / "fusion-return-reset-review"
BASE_MAIN = "eac4b2a1adfcc63f0fe56a27a04ac12b666250fd"
STEMS = (
    "run_breaks",
    "reset_probes",
    "reset_displacement",
    "reset_projection",
    "reset_margin",
    "reset_surplus",
)
OVERVIEW_FIELDS = (
    "edge_id",
    "case_id",
    "decision_event_id",
    "source_step",
    "source_time_s",
    "decision_probe_origin",
    "stage",
    "witness_obstacle_id",
    "witness_map_x",
    "witness_map_y",
    "multiplicity",
    "failed_gates",
    "unknown_gates",
    "zero_reason",
    "nominal_positive",
    "decision_nominal_target_speed",
    "decision_speed_free_distance",
    "decision_speed_cap",
    "lost_ticks",
    "timer_before_ticks",
    "timer_after_ticks",
    "decision_timer_action",
    "decision_ready_from_timer",
    "witness_raw_current_state",
    "witness_raw_current_gap_lo",
    "witness_raw_current_gap_hi",
    "witness_mean_current_state",
    "witness_mean_current_gap_lo",
    "witness_mean_current_gap_hi",
    "radial_gap_state",
    "radial_gap_lo",
    "radial_gap_hi",
    "distance_gain_lo",
    "distance_gain_hi",
    "surplus_status",
    "required_fraction_lo",
    "required_fraction_hi",
    "surplus_fraction_lo",
    "surplus_fraction_hi",
)
INDEX_FIELDS = (
    "study",
    "collector_script",
    "verifier_script",
    "plotter_script",
    "report_path",
    "sources_sha256",
    "proof_sha256",
    "plot_sources_sha256",
    "replayed_collector_files",
    "replayed_plot_files",
    "independent_comparisons",
    "original_identity_comparisons",
    "audit_json",
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path):
    return json.loads(path.read_text())


def read_csv(path):
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames
        assert fields and len(fields) == len(set(fields)), path
        rows = list(reader)
    assert all(
        set(row) == set(fields) and all(isinstance(v, str) for v in row.values())
        for row in rows
    ), path
    return rows


def write_csv(path, rows, fields):
    assert len(fields) == len(set(fields)) and all(
        set(row) == set(fields) for row in rows
    )
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def project_overview(rows):
    assert all(set(OVERVIEW_FIELDS) <= set(row) for row in rows)
    ids = [row["edge_id"] for row in rows]
    assert len(ids) == len(set(ids))
    assert all(isinstance(row[f], str) for row in rows for f in OVERVIEW_FIELDS)
    return [{field: row[field] for field in OVERVIEW_FIELDS} for row in rows]


def same_file(reference, replay):
    assert (
        reference.read_bytes() == replay.read_bytes()
    ), f"Byte mismatch: {replay} versus {reference}"
    return sha(replay)


def match_directory(reference, replay, names, extra=()):
    expected = set(names) | set(extra)
    actual = list(replay.iterdir())
    assert (
        all(path.is_file() for path in actual) and {p.name for p in actual} == expected
    ), replay
    for name in names:
        same_file(reference / name, replay / name)
    return {name: sha(replay / name) for name in sorted(expected)}


def check_destination(path, repo=REPO):
    path = path.expanduser().resolve()
    if path.exists():
        raise ValueError("Use a fresh output directory")
    if path.is_relative_to(repo.resolve()):
        raise ValueError("Keep generated review outputs outside the repository")
    return path


def run_command(command, log):
    with log.open("wb") as stream:
        subprocess.run(
            command, cwd=REPO, stdout=stream, stderr=subprocess.STDOUT, check=True
        )


def validate_metadata(stem, pinned, source, proof, plot):
    scripts = pinned["scripts_sha256"]
    assert source["analysis"]["script_sha256"] == scripts[f"analyze_{stem}.py"]
    assert proof["script_sha256"] == scripts[f"verify_{stem}.py"]
    assert plot["script_sha256"] == scripts[f"plot_{stem}.py"]
    assert proof["sources_sha256"] == pinned["sources_sha256"]
    assert proof["plot_sources_sha256"] == pinned["plot_sources_sha256"]
    assert source["audit"]["fresh_raw_reads"] == proof["fresh_raw_reads"] == 0
    assert source["audit"]["fresh_native_runs"] == proof["fresh_native_runs"] == 0


def preflight():
    pins = read_json(STUDY / "freeze-pins.json")
    assert pins["base_main"] == BASE_MAIN and tuple(pins["studies"]) == STEMS
    assert pins["overview_fields"] == list(OVERVIEW_FIELDS)
    inputs = {}
    for stem in STEMS:
        pinned = pins["studies"][stem]
        assert pinned["study"] == "fusion-return-" + stem.replace("_", "-")
        reference = RESULTS / pinned["study"]
        assert {p.name for p in reference.iterdir()} == set(pinned["artifact_sha256"])
        for name, digest in pinned["artifact_sha256"].items():
            assert sha(reference / name) == digest, reference / name
            inputs[str((reference / name).relative_to(REPO))] = digest
        assert pinned["sources_sha256"] == pinned["artifact_sha256"]["sources.json"]
        assert (
            pinned["proof_sha256"]
            == pinned["artifact_sha256"]["independent-verification.json"]
        )
        assert (
            pinned["plot_sources_sha256"]
            == pinned["artifact_sha256"]["plot-sources.json"]
        )
        assert set(pinned["scripts_sha256"]) == {
            f"{prefix}_{stem}.py" for prefix in ("analyze", "plot", "verify")
        }
        for name, digest in pinned["scripts_sha256"].items():
            assert sha(ROOT / name) == digest, name
            inputs[str((ROOT / name).relative_to(REPO))] = digest
        validate_metadata(
            stem,
            pinned,
            *(
                read_json(reference / name)
                for name in (
                    "sources.json",
                    "independent-verification.json",
                    "plot-sources.json",
                )
            ),
        )
    return pins, inputs


def replay_stage(stem, pinned, output):
    reference = RESULTS / pinned["study"]
    work = output / "stages" / stem
    work.mkdir(parents=True)
    collector, figures, evidence = (
        work / "collector",
        work / "figures",
        work / "independent-verification.json",
    )
    commands = (
        ("collector", [f"analyze_{stem}.py", "--output", str(collector)]),
        (
            "plotter",
            [f"plot_{stem}.py", "--results", str(collector), "--output", str(figures)],
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
                str(evidence),
            ],
        ),
    )
    for phase, arguments in commands:
        print(f"{stem}: {phase}", flush=True)
        run_command(
            [sys.executable, "-B", str(ROOT / arguments[0]), *arguments[1:]],
            work / (phase + ".log"),
        )
    source, proof, plot = (
        read_json(reference / name)
        for name in (
            "sources.json",
            "independent-verification.json",
            "plot-sources.json",
        )
    )
    collector_hashes = match_directory(
        reference, collector, [*source["output_sha256"], "sources.json"]
    )
    plot_hashes = match_directory(
        reference,
        figures,
        [*plot["output_sha256"], "plot-sources.json"],
        [stem.replace("_", "-") + ".png"],
    )
    proof_hash = same_file(reference / "independent-verification.json", evidence)
    assert proof_hash == pinned["proof_sha256"]
    record = dict(
        stage=stem,
        study=pinned["study"],
        collector_sha256=collector_hashes,
        plot_sha256=plot_hashes,
        independent_proof_sha256=proof_hash,
    )
    index = dict(
        study=pinned["study"],
        collector_script=f"analyze_{stem}.py",
        verifier_script=f"verify_{stem}.py",
        plotter_script=f"plot_{stem}.py",
        report_path=str((reference / "report.md").relative_to(REPO)),
        sources_sha256=pinned["sources_sha256"],
        proof_sha256=proof_hash,
        plot_sources_sha256=pinned["plot_sources_sha256"],
        replayed_collector_files=len(collector_hashes),
        replayed_plot_files=len(plot_hashes),
        independent_comparisons=proof["comparisons"],
        original_identity_comparisons=proof["original_join_comparisons"],
        audit_json=json.dumps(source["audit"], sort_keys=True, separators=(",", ":")),
    )
    return record, index


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python optimization")
    try:
        output = check_destination(args.output)
    except ValueError as error:
        parser.error(str(error))
    pins, inputs = preflight()
    output.mkdir(parents=True)
    records, index = [], []
    try:
        for stem in STEMS:
            replay, entry = replay_stage(stem, pins["studies"][stem], output)
            records.append(replay)
            index.append(entry)
    except subprocess.CalledProcessError as error:
        print(
            f"Stage command failed with status {error.returncode}; inspect logs under {output / 'stages'}",
            file=sys.stderr,
        )
        return error.returncode if error.returncode > 0 else 128 - error.returncode
    rows = project_overview(
        read_csv(output / "stages/reset_surplus/collector/surplus.csv")
    )
    assert len(rows) == 19 and len(index) == 6
    write_csv(output / "reset-evidence.csv", rows, OVERVIEW_FIELDS)
    write_csv(output / "study-index.csv", index, INDEX_FIELDS)
    # Recheck the source pins after all children, before publishing completion.
    assert preflight() == (pins, inputs)
    manifest = dict(
        analysis=dict(
            script_sha256=sha(Path(__file__)),
            protocol_sha256=sha(STUDY / "protocol.md"),
            freeze_sha256=sha(STUDY / "freeze-pins.json"),
        ),
        input_sha256=inputs,
        overview_fields=OVERVIEW_FIELDS,
        stage_replays=records,
        replayed_stages=len(records),
        retained_resets=len(rows),
        fresh_source_observations=0,
        fresh_raw_reads=0,
        fresh_native_dataset_runs=0,
        method="fresh six-stage collector/plot/standalone-verifier replays; byte identity against frozen evidence; overview strings copied only",
        output_sha256={
            name: sha(output / name)
            for name in ("reset-evidence.csv", "study-index.csv")
        },
    )
    (output / "review-sources.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(
        json.dumps(
            dict(
                replayed_stages=6,
                retained_resets=19,
                overview_fields=39,
                completion_manifest="review-sources.json",
            )
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
