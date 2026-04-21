#!/usr/bin/env python3
"""Summarize corrected-SLAM revaluation CSV reports.

Usage:
    python3 scripts/summarize_slam_revaluation.py reports/slam_revaluation/run.csv
    python3 scripts/summarize_slam_revaluation.py reports/slam_revaluation/*.csv
"""

import argparse
import csv
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path


REQUIRED_COLUMNS = {
    "scenario",
    "exit_code",
    "mission_completed",
    "improvement_xy",
    "slam_better_xy",
    "gate_reason_counts",
    "diagnostics_status_counts",
}


def parse_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes"}


def parse_float(value: str, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def parse_counts(value: str) -> Counter:
    counts = Counter()
    for part in value.split(";"):
        if not part:
            continue
        name, sep, count = part.partition(":")
        if not sep:
            counts[name] += 1
            continue
        try:
            counts[name] += int(count)
        except ValueError:
            counts[name] += 1
    return counts


def format_counts(counts: Counter) -> str:
    if not counts:
        return "-"
    return ";".join(f"{name}:{count}" for name, count in sorted(counts.items()))


@dataclass
class ProfileSummary:
    rows: int = 0
    completed: int = 0
    better_xy: int = 0
    improvements: list[float] = field(default_factory=list)
    gate_counts: Counter = field(default_factory=Counter)
    status_counts: Counter = field(default_factory=Counter)

    def add(self, row: dict[str, str]) -> None:
        self.rows += 1
        self.completed += int(parse_bool(row.get("mission_completed", "")))
        self.better_xy += int(parse_bool(row.get("slam_better_xy", "")))
        self.improvements.append(parse_float(row.get("improvement_xy", "")))
        self.gate_counts.update(parse_counts(row.get("gate_reason_counts", "")))
        self.status_counts.update(parse_counts(row.get("diagnostics_status_counts", "")))

    @property
    def avg_improvement(self) -> float:
        if not self.improvements:
            return 0.0
        return sum(self.improvements) / len(self.improvements)

    @property
    def min_improvement(self) -> float:
        return min(self.improvements) if self.improvements else 0.0

    @property
    def max_improvement(self) -> float:
        return max(self.improvements) if self.improvements else 0.0


def load_rows(paths: list[Path]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for path in paths:
        with path.open(newline="") as f:
            reader = csv.DictReader(f)
            missing = REQUIRED_COLUMNS - set(reader.fieldnames or [])
            if missing:
                missing_list = ", ".join(sorted(missing))
                raise ValueError(f"{path}: missing required column(s): {missing_list}")
            for row in reader:
                row["_source"] = str(path)
                row.setdefault("profile", "default")
                rows.append(row)
    return rows


def profile_sort_key(item: tuple[str, ProfileSummary]) -> tuple[int, float, str]:
    profile, summary = item
    return (-summary.completed, -summary.avg_improvement, profile)


def scenario_winners(rows: list[dict[str, str]]) -> list[tuple[str, dict[str, str]]]:
    by_scenario: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        by_scenario[row["scenario"]].append(row)

    winners = []
    for scenario, scenario_rows in sorted(by_scenario.items()):
        best = max(
            scenario_rows,
            key=lambda row: (
                parse_bool(row.get("mission_completed", "")),
                parse_float(row.get("improvement_xy", "")),
                parse_bool(row.get("slam_better_xy", "")),
            ),
        )
        winners.append((scenario, best))
    return winners


def print_profile_summary(rows: list[dict[str, str]]) -> None:
    summaries: dict[str, ProfileSummary] = defaultdict(ProfileSummary)
    for row in rows:
        summaries[row["profile"]].add(row)

    print("Profile summary")
    print(
        f"{'profile':<18} {'completed':>11} {'better_xy':>10} "
        f"{'avg_xy':>9} {'min_xy':>9} {'max_xy':>9} gate_counts"
    )
    print("-" * 96)
    for profile, summary in sorted(summaries.items(), key=profile_sort_key):
        print(
            f"{profile:<18} "
            f"{summary.completed:>5}/{summary.rows:<5} "
            f"{summary.better_xy:>4}/{summary.rows:<5} "
            f"{summary.avg_improvement:>9.4f} "
            f"{summary.min_improvement:>9.4f} "
            f"{summary.max_improvement:>9.4f} "
            f"{format_counts(summary.gate_counts)}"
        )


def print_scenario_winners(rows: list[dict[str, str]]) -> None:
    print()
    print("Scenario winners")
    print(f"{'scenario':<18} {'profile':<18} {'improvement_xy':>14} {'completed':>10}")
    print("-" * 68)
    for scenario, row in scenario_winners(rows):
        print(
            f"{scenario:<18} "
            f"{row['profile']:<18} "
            f"{parse_float(row.get('improvement_xy', '')):>14.4f} "
            f"{row.get('mission_completed', ''):>10}"
        )


def print_status_summary(rows: list[dict[str, str]]) -> None:
    gate_counts = Counter()
    status_counts = Counter()
    for row in rows:
        gate_counts.update(parse_counts(row.get("gate_reason_counts", "")))
        status_counts.update(parse_counts(row.get("diagnostics_status_counts", "")))

    print()
    print("Aggregate diagnostics")
    print(f"gate_reason_counts={format_counts(gate_counts)}")
    print(f"diagnostics_status_counts={format_counts(status_counts)}")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Summarize corrected-SLAM revaluation CSV reports."
    )
    parser.add_argument("csv", nargs="+", type=Path, help="CSV report path(s)")
    args = parser.parse_args(argv)

    try:
        rows = load_rows(args.csv)
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    if not rows:
        print("No rows found.")
        return 0

    print(f"Loaded {len(rows)} row(s) from {len(args.csv)} file(s).")
    print()
    print_profile_summary(rows)
    print_scenario_winners(rows)
    print_status_summary(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
