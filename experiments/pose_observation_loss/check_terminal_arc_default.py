#!/usr/bin/env python3
"""Check that omitting the new terminal flag equals explicit false (nine cases)."""

import argparse
import csv
import json
from pathlib import Path
import subprocess

from analyze_epoch_preview import digest

ROOT = Path(__file__).resolve().parent
REPOSITORY = ROOT.parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    run = args.raw_root.resolve() / "known-white"
    provenance = json.loads((run / "provenance.json").read_text())
    source = (
        REPOSITORY / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
    )
    assert digest(source) == provenance["simulator_sha256"]
    assert provenance["variants"] == ["epoch", "curved"]
    assert provenance["noise_correlation_time_s"] == 0
    reference = run / "epoch-none-269"
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    command = [
        str(REPOSITORY / "target/release/examples/pose_observation_loss"),
        str(run / "planning-inputs/fine-padding"),
        str(output),
        "1.0",
        "0.1",
        "2.0",
        "269",
        "0.02",
        "0.2",
        "5.0",
        "true",
        "true",
        "true",
        "none",
        "true",
        "true",
        "0.0",
        "true",
        "false",
        "true",
        "true",
        "false",
        "false",
        "true",
        "true",
        "true",
        "true",
        "true",
    ]
    assert len(command) == 28  # New optional argument would be position 28.
    with (output / "simulation.log").open("w") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    for filename in ["trajectories.csv", "results.csv"]:
        assert (output / filename).read_bytes() == (
            reference / filename
        ).read_bytes(), filename
    with (output / "trajectories.csv").open() as stream:
        reader = csv.DictReader(stream)
        assert len(reader.fieldnames) == 82
        count = 0
        for row in reader:
            assert None not in row and None not in row.values()
            assert row["terminal_steering_preview"] == "false"
            count += 1
    proof = dict(
        cases=9,
        rows=count,
        simulator_sha256=digest(source),
        script_sha256=digest(Path(__file__)),
        reference=str(reference),
        trajectories_sha256=digest(output / "trajectories.csv"),
        summary_sha256=digest(output / "results.csv"),
    )
    (output / "default-disabled.json").write_text(json.dumps(proof, indent=2) + "\n")
    print(f"Omitted and explicit-false bytes match: 9 cases, {count} rows")


if __name__ == "__main__":
    main()
