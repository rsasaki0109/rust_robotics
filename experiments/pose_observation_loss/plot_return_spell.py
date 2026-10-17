#!/usr/bin/env python3
"""Plot observed next-loss operations and censoring for all first returns."""

import argparse
import csv
import hashlib
from itertools import product
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np

PARTS = ("known", "new", "fresh")
MODES = ("none", "random", "blackout")
CONDITIONS = tuple(product(("white", "correlated"), ("epoch", "curved"), MODES))
GROUP = ("partition", "noise", "variant", "mode", "operation_selector")
NAMES = {"known": "Known", "new": "Previous additional", "fresh": "Latest additional"}
COLORS = {
    "motion_and_precision_clear": "#882255",
    "motion_clear": "#dd8844",
    "precision_clear": "#aa4499",
    "ttl_expiry": "#cc6677",
    "capacity_eviction": "#999933",
    "age_gate": "#4477aa",
    "latest_epoch_gate": "#117733",
    "older_samples_gate": "#66ccee",
    "other_gate": "#777777",
}
plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "svg.fonttype": "none",
        "svg.hashsalt": "return-spell-mechanisms-v1",
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


def save(fig, out, name):
    fig.savefig(
        out / (name + ".svg"),
        metadata={"Date": None, "Creator": "return-spell-mechanisms"},
    )
    fig.savefig(
        out / (name + ".png"), dpi=125, metadata={"Software": "return-spell-mechanisms"}
    )
    plt.close(fig)


def operations(rows, out):
    lookup = {tuple(r[k] for k in GROUP): r for r in rows}
    selected = [lookup[(p, n, a, m, "all")] for p in PARTS for n, a, m in CONDITIONS]
    present = [op for op in COLORS if any(int(r[op + "_end_events"]) for r in selected)]
    categories = [
        (op + "_end_events", op.replace("_", " "), COLORS[op]) for op in present
    ]
    categories.append(
        ("censored_return_events", "Returned spell censored at log end", "#44aa99")
    )
    maximum = max(int(r["returned_events"]) for r in selected)
    fig, axes = plt.subplots(1, 3, figsize=(19, 8.5), sharey=True)
    for ax, p in zip(axes, PARTS):
        rr = [lookup[(p, n, a, m, "all")] for n, a, m in CONDITIONS]
        left = np.zeros(12)
        for field, _, color in categories:
            values = np.array([int(r[field]) for r in rr])
            ax.barh(
                np.arange(12), values, left=left, color=color, height=0.67, zorder=2
            )
            left += values
        for i, r in enumerate(rr):
            assert left[i] == int(r["returned_events"])
            label = (
                f"{r['ended_events']}/{r['returned_events']}"
                if left[i]
                else "NA (0 returns)"
            )
            ax.text(
                left[i] + max(maximum, 1) * 0.025, i, label, va="center", fontsize=9
            )
        ax.set_xlim(0, max(maximum, 1) * 1.48)
        ax.set_ylim(11.65, -0.65)
        ax.set_yticks(np.arange(12))
        if ax is axes[0]:
            ax.set_yticklabels(
                [
                    f"{n} / {'baseline' if a == 'epoch' else 'curved'} / {m}"
                    for n, a, m in CONDITIONS
                ]
            )
        else:
            ax.tick_params(labelleft=False)
        ax.set_xlabel("Original first-return anchor references")
        ax.set_title(NAMES[p], fontweight="bold", pad=12)
        ax.grid(axis="x", alpha=0.2, zorder=0)
        ax.spines[["right", "top"]].set_visible(False)
    fig.suptitle("What ends the first returned Fusion spell?", fontsize=16, y=0.98)
    fig.legend(
        handles=[Patch(facecolor=c, label=label) for _, label, c in categories],
        loc="upper center",
        bbox_to_anchor=(0.59, 0.93),
        ncol=min(3, len(categories)),
        frameon=False,
    )
    fig.text(
        0.51,
        0.025,
        "Labels: observed ends / first returns. Censored spans do not establish a future loss. Original conditions with no returns remain shown.",
        ha="center",
        fontsize=9.5,
    )
    fig.subplots_adjust(left=0.205, right=0.98, top=0.79, bottom=0.12, wspace=0.16)
    save(fig, out, "spell-operations")


def spans(rows, out):
    fig, axes = plt.subplots(1, 3, figsize=(17, 6.5), sharex=True, sharey=True)
    present = sorted({r["end_operation"] for r in rows if r["ended"] == "True"})
    maximum = max(float(r["returned_spell_span_s"]) for r in rows)
    for ax, p in zip(axes, PARTS):
        rr = [r for r in rows if r["partition"] == p]
        for r in rr:
            ended = r["ended"] == "True"
            color = COLORS[r["end_operation"]] if ended else "#44aa99"
            ax.scatter(
                float(r["return_time_s"]),
                float(r["returned_spell_span_s"]),
                marker="o" if ended else "^",
                facecolors=color if ended else "none",
                edgecolors=color,
                linewidth=1.2,
                s=55,
                alpha=0.7,
            )
            if r["return_ttl_limit"] == "True":
                ax.scatter(
                    float(r["return_time_s"]),
                    float(r["returned_spell_span_s"]),
                    marker="x",
                    c="#222222",
                    s=26,
                    linewidth=0.8,
                    alpha=0.6,
                )
        n = sum(r["ended"] == "True" for r in rr)
        ax.set_title(f"{NAMES[p]}: {n}/{len(rr)} observed ends", fontweight="bold")
        ax.set_xlim(149, 181)
        ax.set_ylim(-0.13, max(1, maximum) * 1.15)
        ax.set_xticks([150, 155, 160, 165, 170, 175, 180])
        ax.set_xlabel("Original first-return time (s)")
        ax.grid(alpha=0.2)
        ax.spines[["right", "top"]].set_visible(False)
    axes[0].set_ylabel("Observed span after return (s)")
    handles = [
        Line2D(
            [],
            [],
            marker="o",
            color=COLORS[op],
            linestyle="",
            label=op.replace("_", " "),
        )
        for op in present
    ]
    handles += [
        Line2D(
            [],
            [],
            marker="^",
            color="#44aa99",
            markerfacecolor="none",
            linestyle="",
            label="Censored observed span",
        ),
        Line2D(
            [],
            [],
            marker="x",
            color="#222222",
            linestyle="",
            label="Oldest epoch at inclusive TTL limit at return",
        ),
    ]
    fig.suptitle(
        "All first returns: observed next-loss spans and censoring", fontsize=16, y=0.98
    )
    fig.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.53, 0.91),
        ncol=min(4, len(handles)),
        frameon=False,
    )
    fig.text(
        0.51,
        0.025,
        "All 137 returned anchor references; coincident points overlap. Spans end at the next logged loss or at 180 s. No extension beyond the original log.",
        ha="center",
        fontsize=10,
    )
    fig.subplots_adjust(left=0.07, right=0.98, top=0.77, bottom=0.15, wspace=0.19)
    save(fig, out, "spell-spans")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    summary = read(args.results / "spell-summary.csv")
    anchors = read(args.results / "return-endpoints.csv")
    selected = [r for r in summary if r["mode"] in MODES]
    returned = [r for r in anchors if r["returned"] == "True"]
    assert len(summary) == 528 and len(selected) == 396 and len(returned) == 137
    args.output.mkdir(parents=True)
    write(args.output / "plot-values.csv", selected)
    write(args.output / "plot-return-values.csv", returned)
    operations(summary, args.output)
    spans(returned, args.output)
    source = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in ("sources.json", "return-endpoints.csv", "spell-summary.csv")
        },
        selection="all 396 original-mode operation summaries and all 137 first-return anchors, including 48 censored returned spells",
        output_sha256={
            p.name: sha(p) for p in sorted(args.output.iterdir()) if p.suffix != ".png"
        },
    )
    (args.output / "plot-sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(dict(summary_rows=396, returned_anchors=137, figures=2), indent=2))


if __name__ == "__main__":
    main()
