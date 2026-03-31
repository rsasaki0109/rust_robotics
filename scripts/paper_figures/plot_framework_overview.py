#!/usr/bin/env python3
"""Generate a framework architecture diagram using matplotlib patches.

Output:
    img/paper/framework_overview.pdf
"""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

# ---------------------------------------------------------------------------
# IEEE / ICRA publication style
# ---------------------------------------------------------------------------
plt.rcParams.update({
    "font.family": "serif",
    "font.size": 8,
    "axes.labelsize": 9,
    "axes.titlesize": 9,
    "figure.dpi": 300,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = REPO_ROOT / "img" / "paper"

# Colour palette
C_CORE = "#4477AA"
C_DOMAIN = "#228833"
C_APP = "#EE6677"
C_VIZ = "#CCBB44"
C_BG = "#F5F5F5"
C_TEXT = "#222222"
C_ARROW = "#888888"


def _box(ax, x, y, w, h, text, color, fontsize=7):
    """Draw a rounded box with centred text."""
    patch = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.02",
        facecolor=color, edgecolor="white", linewidth=0.8, alpha=0.90,
    )
    ax.add_patch(patch)
    ax.text(x + w / 2, y + h / 2, text,
            ha="center", va="center", fontsize=fontsize,
            color="white", fontweight="bold")


def _arrow(ax, x0, y0, x1, y1):
    arrow = FancyArrowPatch(
        (x0, y0), (x1, y1),
        arrowstyle="->,head_width=3,head_length=2",
        color=C_ARROW, linewidth=0.8,
        connectionstyle="arc3,rad=0.0",
    )
    ax.add_patch(arrow)


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(3.5, 3.0))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 8)
    ax.set_aspect("equal")
    ax.axis("off")

    # --- Layer labels (left side) ---
    ax.text(0.3, 7.0, "Application", fontsize=6, color="#999999",
            ha="left", va="center", style="italic")
    ax.text(0.3, 5.0, "Domain", fontsize=6, color="#999999",
            ha="left", va="center", style="italic")
    ax.text(0.3, 2.5, "Foundation", fontsize=6, color="#999999",
            ha="left", va="center", style="italic")

    # --- Top layer: application crate ---
    _box(ax, 2.5, 6.5, 5.0, 0.9, "rust_robotics\n(binary / examples)", C_APP)

    # --- Middle layer: domain crates ---
    bw, bh = 2.0, 0.8
    y_mid = 4.6
    _box(ax, 1.2, y_mid, bw, bh, "localization", C_DOMAIN)
    _box(ax, 4.0, y_mid, bw, bh, "planning", C_DOMAIN)
    _box(ax, 6.8, y_mid, bw, bh, "control", C_DOMAIN)

    y_mid2 = 3.5
    _box(ax, 1.2, y_mid2, bw, bh, "mapping", C_DOMAIN)
    _box(ax, 4.0, y_mid2, bw, bh, "slam", C_DOMAIN)
    _box(ax, 6.8, y_mid2, bw, bh, "viz", C_VIZ)

    # --- Bottom layer: core ---
    _box(ax, 2.5, 1.6, 5.0, 0.9, "rust_robotics_core\n(types, math, traits)", C_CORE)

    # --- Arrows: app -> domain crates ---
    for cx in [2.2, 5.0, 7.8]:
        _arrow(ax, 5.0, 6.5, cx, y_mid + bh)

    # --- Arrows: domain crates -> core ---
    for cx in [2.2, 5.0, 7.8]:
        _arrow(ax, cx, y_mid2, 5.0, 1.6 + 0.9)
    # Also from middle row (localization, planning, control)
    for cx in [2.2, 5.0, 7.8]:
        _arrow(ax, cx, y_mid, cx, y_mid2 + bh)

    fig.savefig(OUT_DIR / "framework_overview.pdf")
    plt.close(fig)
    print(f"Saved {OUT_DIR / 'framework_overview.pdf'}")


if __name__ == "__main__":
    main()
