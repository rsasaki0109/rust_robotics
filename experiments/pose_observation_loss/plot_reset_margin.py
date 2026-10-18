#!/usr/bin/env python3
"""Plot all frozen original/radial/mean point gaps and estimate contributions."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "svg.fonttype": "none",
        "svg.hashsalt": "fusion-return-reset-margin-v1",
    }
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def figure(rows, output):
    assert len(rows) == 19
    fig, axes = plt.subplots(1, 2, figsize=(15, 10.5), sharey=True)
    panels = (
        (
            ("witness_raw_current_gap", "Original raw gap", "#4477aa", -0.20),
            ("radial_gap", "Raw gap + radial component", "#228833", 0),
            ("witness_mean_current_gap", "Original mean gap", "#ee7733", 0.20),
        ),
        (
            ("radial_component", "Radial component", "#4477aa", -0.14),
            ("distance_gain_beyond_radial", "Gain beyond radial", "#ee7733", 0.14),
        ),
    )
    for ax, metrics in zip(axes, panels):
        bounds = [0.0]
        for field, label, color, offset in metrics:
            for i, row in enumerate(rows):
                if row[field + "_center"] == "":
                    continue
                c, lo, hi = [
                    float(row[field + "_" + s]) for s in ("center", "lo", "hi")
                ]
                assert lo <= c <= hi
                bounds.extend((lo, hi))
                ax.hlines(i + offset, lo, hi, color=color, linewidth=2)
                ax.plot(
                    c,
                    i + offset,
                    color=color,
                    markersize=4.5,
                    marker="D" if row["stage"] == "first_chord" else "o",
                )
        pad = max((max(bounds) - min(bounds)) * 0.06, 1e-6)
        ax.set_xlim(min(bounds) - pad, max(bounds) + pad)
        ax.axvline(0, color="#555555", linestyle="--", linewidth=1)
        ax.grid(axis="x", alpha=0.15)
        ax.set_axisbelow(True)
        ax.set_xlabel("m")
        ax.legend(
            handles=[
                Line2D([0], [0], color=c, marker="o", label=l) for _, l, c, _ in metrics
            ],
            loc="upper right",
            frameon=False,
            fontsize=9,
        )
    labels = []
    for row in rows:
        label = f"{row['edge_id']} / {float(row['source_time_s']):.2f} s"
        if row["stage"] == "first_chord":
            label += " *"
        if row["margin_status"] != "complete":
            label += " [unresolved projection]"
        labels.append(label)
    axes[0].set_yticks(range(len(rows)), labels, fontsize=9)
    axes[0].set_ylim(19, -2.0)
    axes[0].set_title("Point gaps to the same original blocker", pad=12)
    axes[1].set_title("Terms added to the raw point gap", pad=12)
    fig.suptitle("Point-gap decomposition at all 19 reset edges", fontsize=16, y=0.985)
    fig.text(
        0.5,
        0.035,
        "* / diamond: original first-chord failure. Full conditional bounds are drawn; many are narrower than the markers.\n"
        "A same-step point-gap identity; these categories do not replace the original curved-chord or release tests.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=(0, 0.08, 1, 0.96), w_pad=2)
    fig.savefig(output / "reset-margin.svg", metadata={"Date": None})
    fig.savefig(
        output / "reset-margin.png",
        dpi=130,
        metadata={"Software": "pose-observation-loss-reset-margin"},
    )
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Use a fresh output directory without -O")
    source = json.loads((args.results / "sources.json").read_text())
    for name, digest in source["output_sha256"].items():
        assert sha(args.results / name) == digest
    rows, summary = [
        read(args.results / n) for n in ("margins.csv", "margin-summary.csv")
    ]
    args.output.mkdir(parents=True)
    for name, target in (
        ("margins.csv", "plot-margins.csv"),
        ("margin-summary.csv", "plot-values.csv"),
    ):
        shutil.copyfile(args.results / name, args.output / target)
    figure(rows, args.output)
    plot = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in ("sources.json", "margins.csv", "margin-summary.csv")
        },
        output_sha256={
            n: sha(args.output / n)
            for n in ("reset-margin.svg", "plot-margins.csv", "plot-values.csv")
        },
    )
    (args.output / "plot-sources.json").write_text(json.dumps(plot, indent=2) + "\n")
    print(
        json.dumps(dict(plotted_pairs=len(rows), summary_cells=len(summary), panels=2))
    )


if __name__ == "__main__":
    main()
