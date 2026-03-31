#!/usr/bin/env python3
"""Generate publication-quality planning algorithm comparison figures.

Expected CSV format (header + data rows):
    scenario,algorithm,time_us,path_length,nodes_expanded

Outputs:
    img/paper/planning_time.pdf
    img/paper/planning_quality.pdf
    img/paper/planning_pareto.pdf
"""

import os
import sys
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------------------------
# IEEE / ICRA publication style
# ---------------------------------------------------------------------------
plt.rcParams.update({
    "font.family": "serif",
    "font.size": 8,
    "axes.labelsize": 9,
    "axes.titlesize": 9,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "legend.fontsize": 7,
    "figure.figsize": (3.5, 2.5),
    "figure.dpi": 300,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
    "axes.grid": False,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_FILE = REPO_ROOT / "img" / "paper" / "planning_benchmark.csv"
OUT_DIR = REPO_ROOT / "img" / "paper"

# Colour palette (colour-blind friendly)
COLORS = ["#4477AA", "#EE6677", "#228833", "#CCBB44", "#66CCEE", "#AA3377"]


def load_csv(path: Path):
    """Return list of dicts from *path*, or None if the file is missing."""
    if not path.is_file():
        print(f"Data file not found: {path}")
        return None
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        return list(reader)


def grouped_bar(ax, scenarios, algorithms, values, ylabel):
    """Draw a grouped bar chart on *ax*.

    *values* is a dict[algorithm] -> list[float] aligned with *scenarios*.
    """
    n_algo = len(algorithms)
    x = np.arange(len(scenarios))
    width = 0.8 / n_algo

    for i, algo in enumerate(algorithms):
        offset = (i - (n_algo - 1) / 2) * width
        ax.bar(x + offset, values[algo], width, label=algo,
               color=COLORS[i % len(COLORS)], edgecolor="white", linewidth=0.3)

    ax.set_xticks(x)
    ax.set_xticklabels(scenarios, rotation=30, ha="right")
    ax.set_ylabel(ylabel)
    ax.legend(frameon=False, ncol=min(n_algo, 3))


def plot_time(rows):
    scenarios = sorted(set(r["scenario"] for r in rows))
    algorithms = sorted(set(r["algorithm"] for r in rows))

    vals = {a: [] for a in algorithms}
    for scen in scenarios:
        for algo in algorithms:
            matched = [float(r["time_us"]) for r in rows
                       if r["scenario"] == scen and r["algorithm"] == algo]
            vals[algo].append(np.mean(matched) if matched else 0.0)

    fig, ax = plt.subplots()
    grouped_bar(ax, scenarios, algorithms, vals, "Computation time [$\\mu$s]")
    fig.savefig(OUT_DIR / "planning_time.pdf")
    plt.close(fig)
    print(f"Saved {OUT_DIR / 'planning_time.pdf'}")


def plot_quality(rows):
    scenarios = sorted(set(r["scenario"] for r in rows))
    algorithms = sorted(set(r["algorithm"] for r in rows))

    vals = {a: [] for a in algorithms}
    for scen in scenarios:
        for algo in algorithms:
            matched = [float(r["path_length"]) for r in rows
                       if r["scenario"] == scen and r["algorithm"] == algo]
            vals[algo].append(np.mean(matched) if matched else 0.0)

    fig, ax = plt.subplots()
    grouped_bar(ax, scenarios, algorithms, vals, "Path length")
    fig.savefig(OUT_DIR / "planning_quality.pdf")
    plt.close(fig)
    print(f"Saved {OUT_DIR / 'planning_quality.pdf'}")


def plot_pareto(rows):
    algorithms = sorted(set(r["algorithm"] for r in rows))

    fig, ax = plt.subplots()
    for i, algo in enumerate(algorithms):
        times = [float(r["time_us"]) for r in rows if r["algorithm"] == algo]
        lengths = [float(r["path_length"]) for r in rows if r["algorithm"] == algo]
        ax.scatter(times, lengths, label=algo, s=18,
                   color=COLORS[i % len(COLORS)], edgecolors="none", alpha=0.85)

    ax.set_xlabel("Computation time [$\\mu$s]")
    ax.set_ylabel("Path length")
    ax.legend(frameon=False, fontsize=7)
    fig.savefig(OUT_DIR / "planning_pareto.pdf")
    plt.close(fig)
    print(f"Saved {OUT_DIR / 'planning_pareto.pdf'}")


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    rows = load_csv(DATA_FILE)
    if rows is None:
        return

    plot_time(rows)
    plot_quality(rows)
    plot_pareto(rows)


if __name__ == "__main__":
    main()
