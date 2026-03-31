#!/usr/bin/env python3
"""Generate publication-quality Rust vs Python speed comparison figure.

Expected CSV format (header + data rows):
    algorithm,rust_ms,python_ms

Output:
    img/paper/rust_vs_python.pdf
"""

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
DATA_FILE = REPO_ROOT / "img" / "paper" / "speed_comparison.csv"
OUT_DIR = REPO_ROOT / "img" / "paper"

RUST_COLOR = "#4477AA"
PYTHON_COLOR = "#EE6677"


def load_csv(path: Path):
    if not path.is_file():
        print(f"Data file not found: {path}")
        return None
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    rows = load_csv(DATA_FILE)
    if rows is None:
        return

    algorithms = [r["algorithm"] for r in rows]
    rust_ms = [float(r["rust_ms"]) for r in rows]
    python_ms = [float(r["python_ms"]) for r in rows]

    x = np.arange(len(algorithms))
    width = 0.35

    fig, ax = plt.subplots()
    bars_rust = ax.bar(x - width / 2, rust_ms, width, label="Rust",
                       color=RUST_COLOR, edgecolor="white", linewidth=0.3)
    bars_python = ax.bar(x + width / 2, python_ms, width, label="Python",
                         color=PYTHON_COLOR, edgecolor="white", linewidth=0.3)

    # Speedup annotations above the taller (Python) bar
    for i, (r, p) in enumerate(zip(rust_ms, python_ms)):
        if r > 0:
            speedup = p / r
            y_pos = max(r, p)
            ax.text(x[i], y_pos, f"{speedup:.1f}x",
                    ha="center", va="bottom", fontsize=6, fontweight="bold")

    ax.set_xticks(x)
    ax.set_xticklabels(algorithms, rotation=30, ha="right")
    ax.set_ylabel("Computation time [ms]")
    ax.legend(frameon=False)

    fig.savefig(OUT_DIR / "rust_vs_python.pdf")
    plt.close(fig)
    print(f"Saved {OUT_DIR / 'rust_vs_python.pdf'}")


if __name__ == "__main__":
    main()
