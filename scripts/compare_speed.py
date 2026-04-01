#!/usr/bin/env python3
"""Compare Rust vs Python benchmark results.

Reads two CSV files (from speed_comparison.rs and speed_comparison.py)
and prints a formatted comparison table.

Usage:
    # 1. Generate Rust results (release mode recommended):
    cargo run -p rust_robotics --example speed_comparison --features full --release > /tmp/rust_bench.csv

    # 2. Generate Python results:
    python3 scripts/speed_comparison.py > /tmp/python_bench.csv

    # 3. Compare:
    python3 scripts/compare_speed.py /tmp/rust_bench.csv /tmp/python_bench.csv
"""

import csv
import sys


def read_csv(path):
    results = {}
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            algo = row.get("algorithm", "").strip()
            ms_key = [k for k in row.keys() if k.endswith("_ms")][0]
            ms = float(row[ms_key])
            runs = int(row["runs"])
            results[algo] = (ms, runs)
    return results


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <rust_csv> <python_csv>", file=sys.stderr)
        sys.exit(1)

    rust_results = read_csv(sys.argv[1])
    python_results = read_csv(sys.argv[2])

    algorithms = list(rust_results.keys())

    # Header
    header = f"{'Algorithm':<16} | {'Rust (ms)':>12} | {'Python (ms)':>12} | {'Speedup':>10} | {'Runs':>6}"
    sep = "-" * len(header)

    print()
    print(sep)
    print(header)
    print(sep)

    for algo in algorithms:
        rust_ms, runs = rust_results[algo]
        if algo in python_results:
            python_ms, _ = python_results[algo]
            if rust_ms > 0:
                speedup = python_ms / rust_ms
                speedup_str = f"{speedup:.1f}x"
            else:
                speedup_str = "inf"
            print(
                f"{algo:<16} | {rust_ms:>12.3f} | {python_ms:>12.3f} | {speedup_str:>10} | {runs:>6}"
            )
        else:
            print(
                f"{algo:<16} | {rust_ms:>12.3f} | {'N/A':>12} | {'N/A':>10} | {runs:>6}"
            )

    print(sep)
    print()


if __name__ == "__main__":
    main()
