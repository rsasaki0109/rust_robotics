#!/usr/bin/env python3
"""Repeat the protocol's first nine-case invocation and compare complete CSVs."""

import argparse
import json
from pathlib import Path
import subprocess

from analyze_epoch_preview import digest
from analyze_fresh_seed_modes import ROOT, SEEDS, SIM_SHA


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path, required=True)
    parser.add_argument(
        "--binary",
        type=Path,
        default=ROOT.parents[1] / "target/release/examples/pose_observation_loss",
    )
    args = parser.parse_args()
    raw, binary = args.raw_root.resolve(), args.binary.resolve()
    output = raw / "repeat-first"
    if not __debug__ or output.exists() or (raw / "repeat-invocation.json").exists():
        parser.error("Run without -O; repeat output/metadata must be new")
    provenance = json.loads((raw / "white/provenance.json").read_text())
    assert provenance["seeds"] == SEEDS and provenance["variants"] == [
        "epoch",
        "curved",
    ]
    assert (
        provenance["noise_correlation_time_s"] == 0
        and provenance["simulator_sha256"] == SIM_SHA
    )
    command = [
        str(binary),
        str(raw / "white/planning-inputs/fine-padding"),
        str(output),
        "1.0",
        "0.1",
        "2.0",
        "277",
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
        "false",
    ]
    binary_sha = digest(binary)
    output.mkdir()
    with (output / "simulation.log").open("w") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    assert digest(binary) == binary_sha
    for name in ["trajectories.csv", "results.csv"]:
        assert (output / name).read_bytes() == (
            raw / "white/epoch-none-277" / name
        ).read_bytes(), name
    record = dict(command=command, binary_sha256=binary_sha)
    (raw / "repeat-invocation.json").write_text(json.dumps(record, indent=2) + "\n")
    print("First nine-case invocation: trajectory/result bytes identical")


if __name__ == "__main__":
    main()
