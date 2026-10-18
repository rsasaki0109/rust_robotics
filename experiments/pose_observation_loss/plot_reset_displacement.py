#!/usr/bin/env python3
"""Plot all frozen raw/mean point gaps and spatial displacement comparisons."""

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
        "svg.hashsalt": "fusion-return-reset-displacement-v1",
    }
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def figure(rows, out):
    assert len(rows) == 19
    fig, axes = plt.subplots(1, 2, figsize=(15, 10.5), sharey=True)
    specifications = (
        (
            ("witness_raw_current_gap", "Raw point gap", "#4477aa"),
            ("witness_mean_current_gap", "Mean point gap", "#ee7733"),
        ),
        (
            ("distance_gain", "Distance gain", "#4477aa"),
            ("displacement_norm", "Position difference length", "#ee7733"),
        ),
    )
    for ax, metrics in zip(axes, specifications):
        bounds = [0.0]
        for (field, label, color), offset in zip(metrics, (-0.14, 0.14)):
            for i, r in enumerate(rows):
                c, lo, hi = [float(r[field + "_" + s]) for s in ("center", "lo", "hi")]
                assert lo <= c <= hi
                bounds.extend((lo, hi))
                ax.hlines(i + offset, lo, hi, color=color, linewidth=2)
                ax.plot(
                    c,
                    i + offset,
                    marker="D" if r["stage"] == "first_chord" else "o",
                    color=color,
                    markersize=4.5,
                )
        pad = max((max(bounds) - min(bounds)) * 0.06, 1e-6)
        ax.set_xlim(min(bounds) - pad, max(bounds) + pad)
        ax.axvline(0, color="#555555", linestyle="--", linewidth=1)
        ax.grid(axis="x", alpha=0.15)
        ax.set_axisbelow(True)
        ax.set_xlabel("m")
        ax.legend(
            handles=[
                Line2D([0], [0], color=c, marker="o", label=l) for _, l, c in metrics
            ],
            loc="upper right",
            frameon=False,
            fontsize=9,
        )
    axes[0].set_yticks(
        range(len(rows)),
        [
            f"{r['edge_id']} / {float(r['source_time_s']):.2f} s"
            + (" *" if r["stage"] == "first_chord" else "")
            for r in rows
        ],
        fontsize=9,
    )
    axes[0].set_ylim(19.0, -1.6)
    axes[0].set_title("Inherited point gaps to the same original blocker", pad=12)
    axes[1].set_title("Mean minus raw distance and position difference", pad=12)
    fig.suptitle(
        "Raw and mean position estimates at all 19 reset edges", fontsize=16, y=0.985
    )
    fig.text(
        0.5,
        0.035,
        "* / diamond: original first-chord failure (2 rows). Other rows: initial-point failure (17).\n"
        "Full conditional bounds are drawn; many are narrower than the markers. Same-step comparison, not temporal motion.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=(0, 0.08, 1, 0.96), w_pad=2)
    fig.savefig(out / "reset-displacement.svg", metadata={"Date": None})
    fig.savefig(
        out / "reset-displacement.png",
        dpi=130,
        metadata={"Software": "pose-observation-loss-reset-displacement"},
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
    for name, h in source["output_sha256"].items():
        assert sha(args.results / name) == h
    rows = read(args.results / "displacements.csv")
    summary = read(args.results / "displacement-summary.csv")
    args.output.mkdir(parents=True)
    for name, target in (
        ("displacements.csv", "plot-displacements.csv"),
        ("displacement-summary.csv", "plot-values.csv"),
    ):
        shutil.copyfile(args.results / name, args.output / target)
    figure(rows, args.output)
    provenance = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in ("sources.json", "displacements.csv", "displacement-summary.csv")
        },
        output_sha256={
            n: sha(args.output / n)
            for n in (
                "reset-displacement.svg",
                "plot-displacements.csv",
                "plot-values.csv",
            )
        },
    )
    (args.output / "plot-sources.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(
        json.dumps(dict(plotted_resets=len(rows), summary_cells=len(summary), panels=2))
    )


if __name__ == "__main__":
    main()
