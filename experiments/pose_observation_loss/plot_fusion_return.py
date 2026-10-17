#!/usr/bin/env python3
"""Plot frozen return outcomes and observed waits/censoring bounds."""

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
CATEGORIES = (
    ("returned_one_checkpoint", "Returned, then lost after 20 ms", "#cc6677"),
    ("returned_longer_ended", "Returned longer, with observed end", "#4477aa"),
    ("returned_spell_censored", "Returned, available through log end", "#44aa99"),
    ("no_return_censored", "No return observed by log end", "#bbbbbb"),
)
plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "svg.fonttype": "none",
        "svg.hashsalt": "fusion-return-followup-v1",
    }
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def write(path, rows, columns):
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def save(fig, out, name):
    fig.savefig(
        out / (name + ".svg"),
        metadata={"Date": None, "Creator": "fusion-return-followup"},
    )
    fig.savefig(
        out / (name + ".png"), dpi=125, metadata={"Software": "fusion-return-followup"}
    )
    plt.close(fig)


def coverage(lookup, out):
    fig, axes = plt.subplots(1, 3, figsize=(19, 8.5), sharey=True)
    maximum = max(
        int(lookup[(p, n, a, m, "all")]["observed_loss_events"])
        for p in PARTS
        for n, a, m in CONDITIONS
    )
    for ax, part in zip(axes, PARTS):
        left = np.zeros(12)
        for category, _, color in CATEGORIES:
            values = [
                int(lookup[(part, n, a, m, "all")][category + "_events"])
                for n, a, m in CONDITIONS
            ]
            ax.barh(
                np.arange(12), values, left=left, color=color, height=0.67, zorder=2
            )
            left += values
        for i, (noise, arm, mode) in enumerate(CONDITIONS):
            r = lookup[(part, noise, arm, mode, "all")]
            assert left[i] == int(r["observed_loss_events"])
            text = (
                f"{r['returned_events']}/{r['observed_loss_events']}"
                if left[i]
                else "NA (0 losses)"
            )
            ax.text(left[i] + max(1, maximum) * 0.025, i, text, va="center", fontsize=9)
        ax.set_xlim(0, max(1, maximum) * 1.52)
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
        ax.set_xlabel("Original observed-loss anchors")
        ax.grid(axis="x", alpha=0.2, zorder=0)
        ax.spines[["right", "top"]].set_visible(False)
    fig.suptitle(
        "First Fusion return by the original 180 s log endpoint", fontsize=16, y=0.98
    )
    fig.legend(
        handles=[Patch(facecolor=color, label=label) for _, label, color in CATEGORIES],
        loc="upper center",
        bbox_to_anchor=(0.59, 0.93),
        ncol=2,
        frameon=False,
        fontsize=10,
    )
    fig.text(
        0.51,
        0.025,
        "Labels: observed returns / observed losses. A return at 180 s is observed; its returned spell is censored. No return by 180 s is right-censored.",
        ha="center",
        fontsize=9.5,
    )
    fig.subplots_adjust(left=0.205, right=0.98, top=0.78, bottom=0.12, wspace=0.16)
    save(fig, out, "return-coverage")


def waits(events, out):
    colors = {"ttl_expiry": "#4477aa", "motion_clear": "#dd8844"}
    assert {r["loss_operation"] for r in events} <= set(colors)
    fig, axes = plt.subplots(1, 3, figsize=(17, 6.5), sharex=True, sharey=True)
    for ax, part in zip(axes, PARTS):
        selected = [r for r in events if r["partition"] == part]
        for r in selected:
            returned = r["returned"] == "True"
            y = float(
                r["return_wait_s"] if returned else r["censor_wait_lower_bound_s"]
            )
            color = colors[r["loss_operation"]]
            ax.scatter(
                float(r["loss_time_s"]),
                y,
                marker="o" if returned else "^",
                facecolors=color if returned else "none",
                edgecolors=color,
                s=48,
                linewidth=1.1,
                alpha=0.75,
            )
        n = sum(r["returned"] == "True" for r in selected)
        ax.set_title(f"{NAMES[part]}: {n}/{len(selected)} returned", fontweight="bold")
        if not selected:
            ax.text(0.5, 0.5, "No observed losses", transform=ax.transAxes, ha="center")
        ax.set_xlim(149, 181)
        ax.set_ylim(-0.6, 31)
        ax.set_xticks([150, 155, 160, 165, 170, 175, 180])
        ax.set_yticks([0, 5, 10, 15, 20, 25, 30])
        ax.grid(alpha=0.2)
        ax.spines[["right", "top"]].set_visible(False)
        ax.set_xlabel("Original logged loss time (s)")
    axes[0].set_ylabel("Return wait / censoring lower bound (s)")
    handles = [
        Line2D([], [], color=c, linewidth=3, label=l)
        for c, l in [("#4477aa", "TTL expiry"), ("#dd8844", "Motion clear")]
    ]
    handles += [
        Line2D(
            [], [], marker="o", color="#444444", linestyle="", label="Observed return"
        ),
        Line2D(
            [],
            [],
            marker="^",
            markerfacecolor="none",
            color="#444444",
            linestyle="",
            label="Right-censored lower bound (≥)",
        ),
    ]
    fig.suptitle(
        "All fixed loss anchors: observed waits and censoring bounds",
        fontsize=16,
        y=0.98,
    )
    fig.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.53, 0.91),
        ncol=4,
        frameon=False,
    )
    fig.text(
        0.51,
        0.025,
        "All 152 anchors; coincident points overlap. Censoring bounds are not observed return waits. Follow-up ends at the original 180 s log.",
        ha="center",
        fontsize=10,
    )
    fig.subplots_adjust(left=0.07, right=0.98, top=0.77, bottom=0.15, wspace=0.19)
    save(fig, out, "return-wait")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    rows = read(args.results / "followup-summary.csv")
    anchors = read(args.results / "loss-followup.csv")
    lookup = {tuple(r[k] for k in GROUP): r for r in rows}
    assert len(rows) == len(lookup) == 528
    selected = [r for r in rows if r["mode"] in MODES]
    events = [r for r in anchors if r["preview_loss"] == "True"]
    assert len(selected) == 396 and len(events) == 152
    args.output.mkdir(parents=True)
    write(args.output / "plot-values.csv", selected, rows[0].keys())
    write(args.output / "plot-event-values.csv", events, anchors[0].keys())
    coverage(lookup, args.output)
    waits(events, args.output)
    meta = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in ("sources.json", "loss-followup.csv", "followup-summary.csv")
        },
        selection="all original-mode operation summaries (396) and all 152 previously fixed loss anchors; original log endpoint 180 s",
        output_sha256={
            p.name: sha(p) for p in sorted(args.output.iterdir()) if p.suffix != ".png"
        },
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(json.dumps(dict(summary_rows=396, loss_anchors=152, figures=2), indent=2))


if __name__ == "__main__":
    main()
