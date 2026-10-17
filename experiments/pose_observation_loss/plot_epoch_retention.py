#!/usr/bin/env python3
"""Plot every fixed condition's full-window preview-loss operations."""

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
GROUP = ("partition", "noise", "variant", "mode", "event_group", "horizon")
OPERATIONS = (
    ("motion_and_precision_clear", "Motion + precision clear", "#882255"),
    ("motion_clear", "Motion clear", "#dd8844"),
    ("precision_clear", "Precision clear", "#cc6677"),
    ("ttl_expiry", "TTL expiry", "#4477aa"),
    ("capacity_eviction", "Capacity eviction", "#aa4499"),
    ("age_gate", "Age gate", "#999933"),
    ("latest_epoch_gate", "Latest epoch gate", "#44aa99"),
    ("older_samples_gate", "Older sample gate", "#117733"),
    ("other_gate", "Other gate", "#999999"),
)
NAMES = {"known": "Known", "new": "Previous additional", "fresh": "Latest additional"}
plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "svg.fonttype": "none",
        "svg.hashsalt": "epoch-retention-mechanisms-v1",
    }
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    rows = read(args.results / "retention-summary.csv")
    lookup = {tuple(r[k] for k in GROUP): r for r in rows}
    assert len(rows) == len(lookup) == 1344
    selected = [r for r in rows if r["mode"] in MODES and r["event_group"] == "all"]
    assert len(selected) == 144
    args.output.mkdir(parents=True)
    with (args.output / "plot-values.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(selected)
    fig, axes = plt.subplots(1, 3, figsize=(19, 8.5), sharey=True)
    maximum = max(
        int(r["observed_loss_events"]) for r in selected if r["horizon"] == "full"
    )
    for ax, part in zip(axes, PARTS):
        left = np.zeros(12)
        for op, _, color in OPERATIONS:
            counts = [
                int(lookup[(part, noise, arm, mode, "all", "full")][op + "_events"])
                for noise, arm, mode in CONDITIONS
            ]
            ax.barh(
                np.arange(12), counts, left=left, color=color, height=0.67, zorder=2
            )
            left += counts
        for j, total in enumerate(left):
            ax.text(
                total + max(1, maximum) * 0.02,
                j,
                str(int(total)),
                va="center",
                fontsize=9,
            )
        ax.set_xlim(0, max(1, maximum) * 1.2)
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
        ax.set_title(NAMES[part], fontweight="bold", pad=12)
        ax.set_xlabel("Events with logged Fusion → Filter loss")
        ax.grid(axis="x", alpha=0.2, zorder=0)
        ax.spines[["right", "top"]].set_visible(False)
    fig.suptitle(
        "Original epoch-history operations at the first preview loss",
        fontsize=16,
        y=0.98,
    )
    fig.legend(
        handles=[Patch(facecolor=color, label=label) for _, label, color in OPERATIONS],
        loc="upper center",
        bbox_to_anchor=(0.59, 0.93),
        ncol=5,
        frameon=False,
        fontsize=9,
    )
    fig.text(
        0.51,
        0.025,
        "Full 1.50 s forward windows only. Reset precedes TTL pruning, then capacity; categories describe the original algorithm, not controller benefits.",
        ha="center",
        fontsize=10,
    )
    fig.subplots_adjust(left=0.205, right=0.98, top=0.78, bottom=0.12, wspace=0.16)
    fig.savefig(
        args.output / "loss-mechanisms.svg",
        metadata={"Date": None, "Creator": "epoch-retention-mechanisms"},
    )
    fig.savefig(
        args.output / "loss-mechanisms.png",
        dpi=125,
        metadata={"Software": "epoch-retention-mechanisms"},
    )
    plt.close(fig)
    meta = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n) for n in ("sources.json", "retention-summary.csv")
        },
        selection="all 36 original conditions, all event kinds and full forward horizons; export all 144 original-mode/all-kind horizon rows",
        output_sha256={
            p.name: sha(p) for p in sorted(args.output.iterdir()) if p.suffix != ".png"
        },
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(
        json.dumps(
            dict(plotted_summary_rows=144, condition_rows=36, figures=1), indent=2
        )
    )


if __name__ == "__main__":
    main()
