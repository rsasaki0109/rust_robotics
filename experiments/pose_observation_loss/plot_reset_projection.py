#!/usr/bin/env python3
"""Plot every frozen projection pair and its finite-distance remainder."""

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
        "svg.hashsalt": "fusion-return-reset-projection-v1",
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
    metrics = (
        (
            ("radial_component", "Radial component", "#4477aa"),
            ("transverse_component", "Signed transverse component", "#ee7733"),
        ),
        (
            ("distance_gain", "Inherited distance gain", "#4477aa"),
            ("distance_gain_beyond_radial", "Gain beyond radial component", "#ee7733"),
        ),
    )
    for ax, pairs in zip(axes, metrics):
        bounds = [0.0]
        for (field, label, color), offset in zip(pairs, (-0.14, 0.14)):
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
                    marker="D" if row["stage"] == "first_chord" else "o",
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
                Line2D([0], [0], color=c, marker="o", label=l) for _, l, c in pairs
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
        if row["projection_status"] != "complete":
            label += " [" + row["projection_status"] + "]"
        labels.append(label)
    axes[0].set_yticks(range(len(rows)), labels, fontsize=9)
    axes[0].set_ylim(19, -1.6)
    axes[0].set_title(
        "Position difference in the original blocker's radial frame", pad=12
    )
    axes[1].set_title("Finite distance gain and the part beyond radial", pad=12)
    fig.suptitle(
        "Radial and transverse projections at all 19 reset edges", fontsize=16, y=0.985
    )
    fig.text(
        0.5,
        0.035,
        "* / diamond: original first-chord failure. Full conditional bounds are drawn; many are narrower than the markers.\n"
        "Signed transverse uses the recorded x/y convention. Same-step estimates, not temporal robot motion.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=(0, 0.08, 1, 0.96), w_pad=2)
    fig.savefig(output / "reset-projection.svg", metadata={"Date": None})
    fig.savefig(
        output / "reset-projection.png",
        dpi=130,
        metadata={"Software": "pose-observation-loss-reset-projection"},
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
        read(args.results / n) for n in ("projections.csv", "projection-summary.csv")
    ]
    args.output.mkdir(parents=True)
    for name, target in (
        ("projections.csv", "plot-projections.csv"),
        ("projection-summary.csv", "plot-values.csv"),
    ):
        shutil.copyfile(args.results / name, args.output / target)
    figure(rows, args.output)
    plot = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in ("sources.json", "projections.csv", "projection-summary.csv")
        },
        output_sha256={
            n: sha(args.output / n)
            for n in ("reset-projection.svg", "plot-projections.csv", "plot-values.csv")
        },
    )
    (args.output / "plot-sources.json").write_text(json.dumps(plot, indent=2) + "\n")
    print(
        json.dumps(dict(plotted_pairs=len(rows), summary_cells=len(summary), panels=2))
    )


if __name__ == "__main__":
    main()
