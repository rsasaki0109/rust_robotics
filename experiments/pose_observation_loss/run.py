#!/usr/bin/env python3
"""Reproduce the paired observation-loss experiment using Cargo and Python 3."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parent
REPOSITORY = ROOT.parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, help="New directory for raw trajectories"
    )
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(201, 211)))
    comparison = parser.add_mutually_exclusive_group()
    comparison.add_argument(
        "--recovery-comparison",
        action="store_true",
        help="Compare original pause/recovery with stationary pose averaging",
    )
    comparison.add_argument(
        "--stopping-preview-comparison",
        action="store_true",
        help="Compare averaged recovery with an always-on stopping preview guard",
    )
    comparison.add_argument(
        "--restart-consistency-comparison",
        action="store_true",
        help="Compare stopping guard with fresh stationary correction consistency",
    )
    parser.add_argument(
        "--noise-correlation-time",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="Local pose noise correlation time (0 preserves independent noise)",
    )
    args = parser.parse_args()
    if (
        not math.isfinite(args.noise_correlation_time)
        or args.noise_correlation_time < 0
    ):
        parser.error("noise correlation time must be finite and nonnegative")
    variants = (
        ["aware", "averaged"] if args.recovery_comparison else ["baseline", "aware"]
    )
    if args.stopping_preview_comparison:
        variants = ["averaged", "guarded"]
    if args.restart_consistency_comparison:
        variants = ["guarded", "consistent"]
    if len(set(args.seeds)) != len(args.seeds) or any(seed < 0 for seed in args.seeds):
        parser.error("seeds must be distinct nonnegative integers")
    started = datetime.now(timezone.utc)
    run = args.output or ROOT / "runs" / started.strftime("%Y%m%dT%H%M%S.%fZ")
    run = run.resolve()
    run.mkdir(parents=True, exist_ok=False)
    simulator_source = (
        REPOSITORY / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
    )
    simulator_sha256 = hashlib.sha256(simulator_source.read_bytes()).hexdigest()
    subprocess.run(
        [
            "cargo",
            "build",
            "--locked",
            "--release",
            "-p",
            "rust_robotics_control",
            "--no-default-features",
            "--example",
            "pose_observation_loss",
        ],
        cwd=REPOSITORY,
        check=True,
    )
    metadata = json.loads(
        subprocess.check_output(
            [
                "cargo",
                "metadata",
                "--locked",
                "--no-deps",
                "--format-version",
                "1",
            ],
            cwd=REPOSITORY,
            text=True,
        )
    )
    binary = (
        Path(metadata["target_directory"])
        / "release"
        / "examples"
        / "pose_observation_loss"
    )
    all_rows, manifest = [], []
    for variant in variants:
        for mode in ["none", "random", "blackout"]:
            for seed in args.seeds:
                trial = run / f"{variant}-{mode}-{seed}"
                trial.mkdir()
                with (trial / "simulation.log").open("w") as log:
                    subprocess.run(
                        [
                            str(binary),
                            str(ROOT / "inputs"),
                            str(trial),
                            "1.0",
                            "0.1",
                            "2.0",
                            str(seed),
                            "0.02",
                            "0.2",
                            "5.0",
                            "true",
                            "true",
                            "true",
                            mode,
                            str(variant != "baseline").lower(),
                            str(
                                variant in ["averaged", "guarded", "consistent"]
                            ).lower(),
                            str(args.noise_correlation_time),
                            str(variant in ["guarded", "consistent"]).lower(),
                            str(variant == "consistent").lower(),
                        ],
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        check=True,
                    )
                with (trial / "results.csv").open() as source:
                    rows = list(csv.DictReader(source))
                assert len(rows) == 9
                all_rows.extend(
                    dict(variant=variant, mode=mode, seed=seed, **row) for row in rows
                )
                manifest.append(
                    dict(variant=variant, mode=mode, seed=seed, directory=trial.name)
                )
                print(
                    f"{trial.name}: held {sum(r['held'] == 'true' for r in rows)}/9, "
                    f"contacts {sum(r['collision'] == 'true' for r in rows)}",
                    flush=True,
                )
    with (run / "all-results.csv").open("w", newline="") as target:
        writer = csv.DictWriter(
            target, fieldnames=list(all_rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(all_rows)
    (run / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    provenance = dict(
        started_utc=started.isoformat(),
        variants=variants,
        noise_correlation_time_s=args.noise_correlation_time,
        seeds=args.seeds,
        commit=subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=REPOSITORY, text=True
        ).strip(),
        dirty=bool(
            subprocess.check_output(
                ["git", "status", "--porcelain"], cwd=REPOSITORY, text=True
            )
        ),
        rustc=subprocess.check_output(["rustc", "--version"], text=True).strip(),
        simulator_sha256=simulator_sha256,
        inputs_sha256={
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (ROOT / "inputs").glob("*.csv")
        },
    )
    assert (
        hashlib.sha256(simulator_source.read_bytes()).hexdigest() == simulator_sha256
    ), "simulator changed during run"
    (run / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    subprocess.run(["python3", str(ROOT / "verify.py"), str(run)], check=True)
    if args.stopping_preview_comparison or args.restart_consistency_comparison:
        subprocess.run(
            ["python3", str(ROOT / "verify_stopping_preview.py"), str(run)], check=True
        )
    print(f"Verified results: {run}")


if __name__ == "__main__":
    main()
