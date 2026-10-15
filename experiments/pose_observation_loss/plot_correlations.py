#!/usr/bin/env python3
"""Plot a completed correlation analysis (optional: requires matplotlib)."""

import argparse
import csv
import os
from pathlib import Path
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("analysis", type=Path)
    args = parser.parse_args()
    rows = list(csv.DictReader((args.analysis / "summary.csv").open()))
    noise = list(csv.DictReader((args.analysis / "noise-statistics.csv").open()))
    taus = sorted({float(row["tau_s"]) for row in rows})
    with tempfile.TemporaryDirectory(prefix="robotics-plot-") as cache:
        os.environ.setdefault("MPLCONFIGDIR", cache)
        os.environ.setdefault("XDG_CACHE_HOME", cache)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.rcParams["svg.fonttype"] = "none"
        plt.rcParams["svg.hashsalt"] = "pose-noise-correlation"

        fig, axes = plt.subplots(1, 2, figsize=(11, 4), layout="constrained")
        positions = list(range(len(taus)))
        for variant, color, label in [
            ("aware", "#2563eb", "Instantaneous recovery pose"),
            ("averaged", "#16a34a", "Averaged recovery pose"),
        ]:
            selected = [
                next(
                    row
                    for row in rows
                    if float(row["tau_s"]) == tau
                    and row["mode"] == "random"
                    and row["variant"] == variant
                )
                for tau in taus
            ]
            axes[0].plot(
                positions,
                [int(row["held"]) for row in selected],
                marker="o",
                color=color,
                label=label,
            )
        axes[0].set(
            title="Random observation loss: 10 s goal holds",
            ylabel="Successful cases (same 90 inputs)",
            ylim=(0, 90),
        )
        axes[0].legend(fontsize=9)
        sd = [
            float(
                next(row for row in noise if float(row["tau_s"]) == tau)[
                    "theoretical_25_sample_sd"
                ]
            )
            for tau in taus
        ]
        axes[1].plot(
            positions, sd, color="#9333ea", marker="o", label="SD of a 25-sample mean"
        )
        axes[1].axhline(1, color="#64748b", linestyle="--", label="Single-sample SD")
        axes[1].set(
            title="Noise reduction from averaging (theoretical)",
            ylabel="SD / single-sample SD",
            ylim=(0, 1.1),
        )
        axes[1].legend(fontsize=9)
        for ax in axes:
            ax.set_xticks(
                positions, ["Independent" if tau == 0 else f"{tau:g} s" for tau in taus]
            )
            ax.set_xlabel("Local pose-noise correlation time")
            ax.grid(alpha=0.2)
        svg_path = args.analysis / "correlation-comparison.svg"
        fig.savefig(svg_path, metadata={"Date": None})
        svg_path.write_text(
            "\n".join(line.rstrip() for line in svg_path.read_text().splitlines())
            + "\n"
        )
        fig.savefig(args.analysis / "correlation-comparison.png", dpi=160)
        plt.close(fig)


if __name__ == "__main__":
    main()
