#!/usr/bin/env python3
"""Plot fixed horizons, logged Fusion losses and RP disagreement timing."""

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
GROUP = ("partition", "noise", "variant", "mode", "event_group", "horizon")
NAMES = {"known": "Known", "new": "Previous additional", "fresh": "Latest additional"}
plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "svg.fonttype": "none",
        "svg.hashsalt": "logged-preview-transitions-v1",
    }
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def write(path, rows, columns):
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def save(fig, out, name):
    fig.savefig(
        out / (name + ".svg"),
        metadata={"Date": None, "Creator": "logged-preview-transitions"},
    )
    fig.savefig(
        out / (name + ".png"),
        dpi=125,
        metadata={"Software": "logged-preview-transitions"},
    )
    plt.close(fig)


def row(lookup, part, condition, horizon):
    noise, arm, mode = condition
    return lookup[(part, noise, arm, mode, "all", horizon)]


def decorate(ax, part, labels=True):
    ax.set_title(NAMES[part], fontweight="bold", pad=14)
    ax.set_yticks(np.arange(12))
    if labels:
        ax.set_yticklabels(
            [
                f"{n} / {'baseline' if a == 'epoch' else 'curved'} / {m}"
                for n, a, m in CONDITIONS
            ]
        )
    else:
        ax.tick_params(labelleft=False)
    ax.set_ylim(11.65, -0.65)
    ax.grid(axis="x", alpha=0.2, zorder=0)
    ax.spines[["right", "top"]].set_visible(False)


def horizons(lookup, out):
    fig, axes = plt.subplots(1, 3, figsize=(19, 8), sharey=True)
    colors = ("#4477aa", "#eeaa44", "#bbbbbb")
    categories = ("full", "clipped", "no_forward")
    maximum = max(
        int(row(lookup, p, c, "all")["census_events"])
        for p, c in product(PARTS, CONDITIONS)
    )
    for ax, part in zip(axes, PARTS):
        left = np.zeros(12)
        for category, color in zip(categories, colors):
            values = [
                int(row(lookup, part, c, category)["census_events"]) for c in CONDITIONS
            ]
            ax.barh(
                np.arange(12), values, left=left, color=color, height=0.67, zorder=2
            )
            left += values
        for j, total in enumerate(left):
            ax.text(
                total + maximum * 0.018, j, str(int(total)), va="center", fontsize=9
            )
        ax.set_xlim(0, maximum * 1.17)
        ax.set_xlabel("Census events (not independent samples)")
        decorate(ax, part, ax is axes[0])
    fig.suptitle("Original post-event observation horizons", fontsize=16, y=0.98)
    fig.legend(
        handles=[
            Patch(facecolor=c, label=l)
            for c, l in zip(
                colors,
                (
                    "Full: 1.50 s",
                    "Clipped: 0.02–1.48 s",
                    "No forward sample: event at 180 s",
                ),
            )
        ],
        loc="upper center",
        bbox_to_anchor=(0.59, 0.93),
        ncol=3,
        frameon=False,
    )
    fig.text(
        0.51,
        0.025,
        "All 3,997 events; each window also includes its -20 ms predecessor. No-forward events cannot reveal a later change.",
        ha="center",
    )
    fig.subplots_adjust(left=0.205, right=0.98, top=0.84, bottom=0.12, wspace=0.16)
    save(fig, out, "observation-horizons")


def fusion_losses(lookup, out):
    fig, axes = plt.subplots(1, 3, figsize=(19, 8), sharey=True)
    for ax, part in zip(axes, PARTS):
        for j, condition in enumerate(CONDITIONS):
            r = row(lookup, part, condition, "full")
            raw, weighted = (
                r["fusion_loss_event_fraction"],
                r["fusion_loss_case_mass_fraction"],
            )
            if raw != "":
                ax.plot(
                    float(raw) * 100,
                    j - 0.13,
                    "o",
                    color="#cc6677",
                    markersize=6,
                    markerfacecolor="white",
                    zorder=3,
                )
                ax.plot(
                    float(weighted) * 100,
                    j + 0.13,
                    "D",
                    color="#332288",
                    markersize=5,
                    zorder=3,
                )
            label = (
                f"{r['fusion_loss_events']}/{r['immediate_fusion_events']}"
                if raw != ""
                else "NA (0)"
            )
            ax.text(104, j, label, va="center", fontsize=9)
        ax.set_xlim(-3, 132)
        ax.set_xticks([0, 25, 50, 75, 100])
        ax.set_xlabel("Observed Fusion → Filter events (%)")
        decorate(ax, part, ax is axes[0])
        ax.text(104, -0.85, "events", fontsize=9, ha="left")
    fig.suptitle(
        "Fusion losses within full 1.50 s forward windows", fontsize=16, y=0.98
    )
    fig.legend(
        handles=[
            Line2D(
                [],
                [],
                marker="o",
                color="#cc6677",
                markerfacecolor="white",
                linestyle="",
                label="Raw event fraction",
            ),
            Line2D(
                [],
                [],
                marker="D",
                color="#332288",
                linestyle="",
                label="Original census-case mass fraction",
            ),
        ],
        loc="upper center",
        bbox_to_anchor=(0.59, 0.93),
        ncol=2,
        frameon=False,
    )
    fig.text(
        0.51,
        0.025,
        "Both fractions condition on immediate Fusion. Case mass is 1/n_original_census_events_case before any filtering; counts are loss/Fusion events.",
        ha="center",
    )
    fig.subplots_adjust(left=0.205, right=0.98, top=0.84, bottom=0.12, wspace=0.16)
    save(fig, out, "full-horizon-fusion-loss")


def lags(events, out):
    fig, axes = plt.subplots(1, 3, figsize=(17, 6), sharex=True, sharey=True)
    markers = {
        ("white", "epoch"): "o",
        ("white", "curved"): "s",
        ("correlated", "epoch"): "^",
        ("correlated", "curved"): "D",
    }
    colors = {"none": "#999999", "random": "#dd7744", "blackout": "#4477aa"}
    for ax, part in zip(axes, PARTS):
        rr = [e for e in events if e["partition"] == part]
        ax.plot([0, 1500], [0, 1500], "--", color="#999999", linewidth=1, zorder=0)
        for e in rr:
            x, y = float(e["first_preview_change_offset_ms"]), float(
                e["first_after_different_rp_offset_ms"]
            )
            assert 0 < x <= y <= 1500
            ax.scatter(
                x,
                y,
                marker=markers[(e["noise"], e["variant"])],
                color=colors[e["mode"]],
                edgecolor="#222222",
                linewidth=0.5,
                s=55,
                alpha=0.8,
            )
        ax.set_title(f"{NAMES[part]}: {len(rr)} events", fontweight="bold")
        if not rr:
            ax.text(
                0.5,
                0.5,
                "No post-event differing RP preview",
                transform=ax.transAxes,
                ha="center",
                va="center",
                fontsize=10,
            )
        ax.set_xlim(-35, 1540)
        ax.set_ylim(-35, 1540)
        ax.set_xticks([0, 500, 1000, 1500])
        ax.set_yticks([0, 500, 1000, 1500])
        ax.grid(alpha=0.2)
        ax.set_xlabel("First logged post-event preview change (ms)")
        ax.spines[["right", "top"]].set_visible(False)
    axes[0].set_ylabel("First differing RP preview record (ms)")
    handles = [
        Line2D([], [], color=colors[m], marker="o", linestyle="", label=m)
        for m in MODES
    ]
    handles += [
        Line2D(
            [],
            [],
            color="#444444",
            marker=s,
            linestyle="",
            label=f"{n} / {'baseline' if a == 'epoch' else 'curved'}",
        )
        for (n, a), s in markers.items()
    ]
    fig.suptitle(
        "Logged preview change and first differing RP record", fontsize=16, y=0.98
    )
    fig.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.52, 0.91),
        ncol=7,
        frameon=False,
        fontsize=9,
    )
    fig.text(
        0.51,
        0.025,
        "All events with a post-event RP preview different from the immediate preview; coincident points overlap. Diagonal: same logged checkpoint.",
        ha="center",
        fontsize=10,
    )
    fig.subplots_adjust(left=0.07, right=0.98, top=0.77, bottom=0.15, wspace=0.19)
    save(fig, out, "rp-transition-lag")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    summary = read(args.results / "transition-summary.csv")
    traces = read(args.results / "event-trace.csv")
    lookup = {tuple(r[k] for k in GROUP): r for r in summary}
    assert len(lookup) == len(summary) == 1344
    rows = [r for r in summary if r["mode"] in MODES and r["event_group"] == "all"]
    events = [e for e in traces if int(e["rp_different_after_records"])]
    assert len(rows) == 144
    args.output.mkdir(parents=True)
    write(args.output / "plot-values.csv", rows, summary[0].keys())
    write(args.output / "plot-event-values.csv", events, traces[0].keys())
    horizons(lookup, args.output)
    fusion_losses(lookup, args.output)
    lags(events, args.output)
    meta = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in ("sources.json", "event-trace.csv", "transition-summary.csv")
        },
        selection="144 all-kind horizon rows in original modes; every event with a post-event differing RP preview",
        output_sha256={
            p.name: sha(p) for p in sorted(args.output.iterdir()) if p.suffix != ".png"
        },
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(
        json.dumps(
            dict(summary_rows=len(rows), event_rows=len(events), figures=3), indent=2
        )
    )


if __name__ == "__main__":
    main()
