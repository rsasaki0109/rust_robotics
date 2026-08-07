#!/usr/bin/env python3
"""Compare a freshly generated benchmark CSV against its committed baseline.

Functional columns must match to within a small numeric tolerance. Wall-clock
columns (headers ending in ``_ms`` / ``_us`` / ``_ns``, e.g. ``elapsed_ms`` or
``mean_plan_time_us``) are ignored because they are not reproducible across
machines. Header order, column names, and row count must match exactly so the
gate also catches layout drift.

Usage:
  check_benchmark_gate.py --baseline <committed.csv> --generated <fresh.csv>
"""

import argparse
import csv
import sys

WALL_CLOCK_SUFFIXES = ("_ms", "_us", "_ns")


def load(path: str) -> list[list[str]]:
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.reader(f))


def is_wall_clock(header: str) -> bool:
    return header.endswith(WALL_CLOCK_SUFFIXES)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline", required=True, help="committed baseline CSV")
    ap.add_argument("--generated", required=True, help="freshly generated CSV")
    ap.add_argument("--rtol", type=float, default=1e-6)
    ap.add_argument("--atol", type=float, default=1e-6)
    args = ap.parse_args()

    base = load(args.baseline)
    gen = load(args.generated)

    if not base or not gen:
        print(f"FAIL: empty CSV ({args.baseline} or {args.generated})")
        return 1

    if base[0] != gen[0]:
        print("FAIL: header mismatch")
        print(f"  baseline : {base[0]}")
        print(f"  generated: {gen[0]}")
        return 1

    header = base[0]
    if len(base) != len(gen):
        print(
            f"FAIL: row count mismatch in {args.generated}: "
            f"baseline={len(base) - 1} generated={len(gen) - 1}"
        )
        return 1

    ignored = [c for c in header if is_wall_clock(c)]
    ignored_idx = {i for i, c in enumerate(header) if is_wall_clock(c)}
    if ignored:
        print(f"(ignoring wall-clock columns: {', '.join(ignored)})")

    errors: list[str] = []
    for r, (row_b, row_g) in enumerate(zip(base[1:], gen[1:]), start=2):
        for i, (cell_b, cell_g) in enumerate(zip(row_b, row_g)):
            if i in ignored_idx:
                continue
            col = header[i]
            try:
                vb, vg = float(cell_b), float(cell_g)
                # Exact equality also covers inf == inf (inf - inf is NaN).
                if vb != vg and not abs(vb - vg) <= args.atol + args.rtol * abs(vg):
                    errors.append(
                        f"row {r} col '{col}': baseline={cell_b} generated={cell_g}"
                    )
            except ValueError:
                if cell_b != cell_g:
                    errors.append(
                        f"row {r} col '{col}': baseline={cell_b!r} generated={cell_g!r}"
                    )

    if errors:
        print(f"FAIL: {len(errors)} regression(s) in {args.generated}")
        for e in errors[:30]:
            print("  " + e)
        return 1

    print(f"PASS: {args.generated} matches {args.baseline}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
