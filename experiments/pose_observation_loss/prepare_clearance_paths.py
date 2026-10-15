#!/usr/bin/env python3
"""Export and audit fixed coarse/fine path-clearance fixtures with Cargo."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from datetime import datetime, timezone

from verify_clearance_paths import audit

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]


def source_hashes():
    paths = [
        REPO / "crates/rust_robotics_planning" / "src" / f"{name}.rs"
        for name in ["a_star", "jps", "theta_star", "grid"]
    ]
    paths.append(
        REPO / "crates/rust_robotics_planning/examples/export_clearance_paths.rs"
    )
    return {
        str(p.relative_to(REPO)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in paths
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    args = parser.parse_args()
    started = datetime.now(timezone.utc).isoformat()
    sha = source_hashes()
    subprocess.run(
        [
            "cargo",
            "build",
            "--locked",
            "--release",
            "-p",
            "rust_robotics_planning",
            "--example",
            "export_clearance_paths",
        ],
        cwd=REPO,
        check=True,
    )
    metadata = json.loads(
        subprocess.check_output(
            ["cargo", "metadata", "--locked", "--no-deps", "--format-version", "1"],
            cwd=REPO,
            text=True,
        )
    )
    binary = (
        Path(metadata["target_directory"]) / "release/examples/export_clearance_paths"
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, res, radius in [
        ("coarse-nominal", 1, 0.5),
        ("coarse-padding", 1, 0.9),
        ("fine-nominal", 0.125, 0.5),
        ("fine-padding", 0.125, 0.99),
    ]:
        subprocess.run(
            [
                str(binary),
                str(ROOT / "inputs"),
                str(args.output / name),
                str(res),
                str(radius),
            ],
            check=True,
        )
    rows = audit(args.output)
    assert sha == source_hashes(), "planner sources changed during export"
    provenance = dict(
        started_utc=started,
        commit=subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=REPO, text=True
        ).strip(),
        dirty=bool(
            subprocess.check_output(
                ["git", "status", "--porcelain"], cwd=REPO, text=True
            )
        ),
        source_sha256=sha,
        rustc=subprocess.check_output(["rustc", "--version"], text=True).strip(),
        required_candidate_clearance=0.9,
        inputs_sha256={
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (ROOT / "inputs").glob("*.csv")
        },
    )
    (args.output / "provenance.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(f"Audited {len(rows)} planned/failed path outcomes: {args.output}")


if __name__ == "__main__":
    main()
