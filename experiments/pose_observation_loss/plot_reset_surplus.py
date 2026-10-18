#!/usr/bin/env python3
"""Plot deficit/radial fractions, retaining both nonapplicable chord rows."""

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
        "svg.hashsalt": "fusion-return-reset-surplus-v1",
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
            ("required_fraction", "Deficit / radial component", "#4477aa"),
            ("surplus_fraction", "Surplus / radial component", "#ee7733"),
        ),
        (
            ("raw_deficit", "Original raw deficit", "#4477aa"),
            ("radial_surplus", "Radial contribution beyond deficit", "#ee7733"),
        ),
    )
    for j, (ax, metrics) in enumerate(zip(axes, panels)):
        scale = 100 if j == 0 else 1
        bounds = [0.0, 100.0] if j == 0 else [0.0]
        for (field, label, color), offset in zip(metrics, (-0.14, 0.14)):
            for i, row in enumerate(rows):
                if row[field + "_center"] == "":
                    continue
                c, lo, hi = [
                    scale * float(row[field + "_" + s]) for s in ("center", "lo", "hi")
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
        for reference in ((0, 100) if j == 0 else (0,)):
            ax.axvline(reference, color="#555555", linestyle="--", linewidth=1)
        ax.grid(axis="x", alpha=0.15)
        ax.set_axisbelow(True)
        ax.set_xlabel("% of recorded radial component" if j == 0 else "m")
        ax.legend(
            handles=[
                Line2D([0], [0], color=c, marker="o", label=l) for _, l, c in metrics
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
        if row["surplus_status"] == "not_applicable_category":
            label += " [N/A]"
        elif row["surplus_status"] != "complete":
            label += " [unresolved]"
        labels.append(label)
    axes[0].set_yticks(range(len(rows)), labels, fontsize=9)
    axes[0].set_ylim(19, -1.7)
    axes[0].set_title("Fractions of the recorded radial component", pad=12)
    axes[1].set_title("Original deficit and the remaining radial contribution", pad=12)
    fig.suptitle(
        "Radial contribution relative to point deficit: all 19 reset edges",
        fontsize=15,
        y=0.985,
    )
    fig.text(
        0.5,
        0.035,
        "* marks original first-chord rows; N/A retains their original already-clear point category. Blank values are not zero.\n"
        "Full conditional bounds; same-step algebraic fractions, not controller tuning or recovery probabilities.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=(0, 0.08, 1, 0.96), w_pad=2)
    fig.savefig(output / "reset-surplus.svg", metadata={"Date": None})
    fig.savefig(
        output / "reset-surplus.png",
        dpi=130,
        metadata={"Software": "pose-observation-loss-reset-surplus"},
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
        read(args.results / n) for n in ("surplus.csv", "surplus-summary.csv")
    ]
    args.output.mkdir(parents=True)
    for name, target in (
        ("surplus.csv", "plot-surplus.csv"),
        ("surplus-summary.csv", "plot-values.csv"),
    ):
        shutil.copyfile(args.results / name, args.output / target)
    figure(rows, args.output)
    plot = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in ("sources.json", "surplus.csv", "surplus-summary.csv")
        },
        output_sha256={
            n: sha(args.output / n)
            for n in ("reset-surplus.svg", "plot-surplus.csv", "plot-values.csv")
        },
    )
    (args.output / "plot-sources.json").write_text(json.dumps(plot, indent=2) + "\n")
    print(
        json.dumps(dict(retained_pairs=len(rows), summary_cells=len(summary), panels=2))
    )


if __name__ == "__main__":
    main()
