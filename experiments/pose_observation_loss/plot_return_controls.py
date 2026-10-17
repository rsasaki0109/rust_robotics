#!/usr/bin/env python3
"""Plot logged loss/guard latches at every fixed first Fusion return."""

import argparse
import csv
import hashlib
from itertools import product
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

PARTS = ("known", "new", "fresh")
MODES = ("none", "random", "blackout")
CONDITIONS = tuple(product(("white", "correlated"), ("epoch", "curved"), MODES))
GROUP = ("partition", "noise", "variant", "mode", "category_selector")
NAMES = {"known": "Known", "new": "Previous additional", "fresh": "Latest additional"}
STATES = (
    ("loss_and_guard", "Loss pause and guard active", "#882255"),
    ("loss_only", "Loss pause only", "#dd8844"),
    ("guard_only", "Guard only", "#4477aa"),
    ("clear", "Loss/guard latches clear", "#44aa99"),
)
plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "svg.fonttype": "none",
        "svg.hashsalt": "fusion-return-controls-v1",
    }
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def write(path, rows):
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0], lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def plot(rows, out):
    lookup = {tuple(r[k] for k in GROUP): r for r in rows}
    maximum = max(
        int(lookup[(p, n, a, m, "all")]["returned_events"])
        for p in PARTS
        for n, a, m in CONDITIONS
    )
    fig, axes = plt.subplots(1, 3, figsize=(19, 8.5), sharey=True)
    for ax, p in zip(axes, PARTS):
        rr = [lookup[(p, n, a, m, "all")] for n, a, m in CONDITIONS]
        left = np.zeros(12)
        for state, _, color in STATES:
            values = np.array([int(r["return_latch_" + state + "_events"]) for r in rr])
            ax.barh(
                np.arange(12), values, left=left, color=color, height=0.67, zorder=2
            )
            left += values
        for i, r in enumerate(rr):
            assert left[i] == int(r["returned_events"])
            text = r["returned_events"] if left[i] else "NA (0 returns)"
            ax.text(left[i] + max(maximum, 1) * 0.025, i, text, va="center", fontsize=9)
        ax.set_xlim(0, max(maximum, 1) * 1.48)
        ax.set_ylim(11.65, -0.65)
        ax.set_yticks(np.arange(12))
        if ax is axes[0]:
            ax.set_yticklabels(
                [
                    f"{n} / {'baseline' if a=='epoch' else 'curved'} / {m}"
                    for n, a, m in CONDITIONS
                ]
            )
        else:
            ax.tick_params(labelleft=False)
        ax.set_xlabel("Original first-return anchor references")
        ax.set_title(NAMES[p], fontweight="bold", pad=12)
        ax.grid(axis="x", alpha=0.2, zorder=0)
        ax.spines[["right", "top"]].set_visible(False)
    fig.suptitle("Logged control latches at first Fusion return", fontsize=16, y=0.98)
    fig.legend(
        handles=[Patch(facecolor=c, label=label) for _, label, c in STATES],
        loc="upper center",
        bbox_to_anchor=(0.59, 0.93),
        ncol=2,
        frameon=False,
    )
    fig.text(
        0.51,
        0.025,
        "All 137 first returns, including 48 at log end. Latches, resumption flags, pose sources and positive speed commands are separate logged quantities.",
        ha="center",
        fontsize=9.5,
    )
    fig.subplots_adjust(left=0.205, right=0.98, top=0.78, bottom=0.12, wspace=0.16)
    fig.savefig(
        out / "return-latches.svg",
        metadata={"Date": None, "Creator": "fusion-return-controls"},
    )
    fig.savefig(
        out / "return-latches.png",
        dpi=125,
        metadata={"Software": "fusion-return-controls"},
    )
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    rows = read(args.results / "control-summary.csv")
    anchors = read(args.results / "anchor-controls.csv")
    selected = [r for r in rows if r["mode"] in MODES]
    returned = [r for r in anchors if r["returned"] == "True"]
    assert len(rows) == 336 and len(selected) == 252 and len(returned) == 137
    args.output.mkdir(parents=True)
    write(args.output / "plot-values.csv", selected)
    write(args.output / "plot-return-values.csv", returned)
    plot(rows, args.output)
    source = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in ("sources.json", "anchor-controls.csv", "control-summary.csv")
        },
        selection="all 36 original conditions, all 252 original-mode category summaries and all 137 returned anchors",
        output_sha256={
            p.name: sha(p) for p in sorted(args.output.iterdir()) if p.suffix != ".png"
        },
    )
    (args.output / "plot-sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(dict(summary_rows=252, returned_anchors=137, figures=1), indent=2))


if __name__ == "__main__":
    main()
