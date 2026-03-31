#!/usr/bin/env python3
"""Generate publication-quality localization filter comparison figures.

Expected CSV format (header + data rows):
    filter,rmse,time_ms,final_error

Outputs:
    img/paper/localization_rmse.pdf
    img/paper/localization_time.pdf
    img/paper/localization_table.pdf
"""

import os
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
DATA_FILE = REPO_ROOT / "img" / "paper" / "localization_benchmark.csv"
OUT_DIR = REPO_ROOT / "img" / "paper"

COLORS = ["#4477AA", "#EE6677", "#228833", "#CCBB44", "#66CCEE", "#AA3377"]


def load_csv(path: Path):
    if not path.is_file():
        print(f"Data file not found: {path}")
        return None
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def plot_rmse(rows):
    filters = [r["filter"] for r in rows]
    rmse = [float(r["rmse"]) for r in rows]

    fig, ax = plt.subplots()
    x = np.arange(len(filters))
    bars = ax.bar(x, rmse, color=COLORS[:len(filters)],
                  edgecolor="white", linewidth=0.3, width=0.6)

    # Value annotations
    for bar, val in zip(bars, rmse):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                f"{val:.3f}", ha="center", va="bottom", fontsize=6)

    ax.set_xticks(x)
    ax.set_xticklabels(filters, rotation=30, ha="right")
    ax.set_ylabel("RMSE")
    fig.savefig(OUT_DIR / "localization_rmse.pdf")
    plt.close(fig)
    print(f"Saved {OUT_DIR / 'localization_rmse.pdf'}")


def plot_time(rows):
    filters = [r["filter"] for r in rows]
    times = [float(r["time_ms"]) for r in rows]

    fig, ax = plt.subplots()
    x = np.arange(len(filters))
    bars = ax.bar(x, times, color=COLORS[:len(filters)],
                  edgecolor="white", linewidth=0.3, width=0.6)

    for bar, val in zip(bars, times):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                f"{val:.2f}", ha="center", va="bottom", fontsize=6)

    ax.set_xticks(x)
    ax.set_xticklabels(filters, rotation=30, ha="right")
    ax.set_ylabel("Computation time [ms]")
    fig.savefig(OUT_DIR / "localization_time.pdf")
    plt.close(fig)
    print(f"Saved {OUT_DIR / 'localization_time.pdf'}")


def plot_table(rows):
    """Render a metrics table as a figure (useful for papers)."""
    columns = ["Filter", "RMSE", "Time [ms]", "Final err."]
    cell_text = []
    for r in rows:
        cell_text.append([
            r["filter"],
            f"{float(r['rmse']):.4f}",
            f"{float(r['time_ms']):.2f}",
            f"{float(r['final_error']):.4f}",
        ])

    fig, ax = plt.subplots(figsize=(3.5, 0.3 + 0.25 * len(rows)))
    ax.axis("off")
    table = ax.table(cellText=cell_text, colLabels=columns,
                     loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(7)
    table.scale(1.0, 1.3)

    # Style header row
    for j in range(len(columns)):
        table[0, j].set_facecolor("#DDDDDD")
        table[0, j].set_edgecolor("white")

    for i in range(1, len(rows) + 1):
        for j in range(len(columns)):
            table[i, j].set_edgecolor("#CCCCCC")

    fig.savefig(OUT_DIR / "localization_table.pdf")
    plt.close(fig)
    print(f"Saved {OUT_DIR / 'localization_table.pdf'}")


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    rows = load_csv(DATA_FILE)
    if rows is None:
        return

    plot_rmse(rows)
    plot_time(rows)
    plot_table(rows)


if __name__ == "__main__":
    main()
