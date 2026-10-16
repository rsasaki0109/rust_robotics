#!/usr/bin/env python3
"""Render paired hold changes and common-horizon wait from the live study."""

import argparse
import csv
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_terminal_arc_live import COHORTS
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output directory must be new")
    summary = read_rows(args.results / "summary.csv")
    series = []
    for cohort in COHORTS:
        rows = {
            r["variant"]: r
            for r in summary
            if r["cohort"] == cohort and r["mode"] == "all"
        }
        assert set(rows) == {"epoch", "curved"}
        old, new = rows["epoch"], rows["curved"]
        assert int(old["cases"]) == int(new["cases"]) == 81
        assert int(new["held"]) - int(old["held"]) == int(new["gained_holds"]) - int(
            new["lost_holds"]
        )
        series.append(
            dict(
                cohort=cohort,
                pairs=81,
                straight_held=int(old["held"]),
                curved_held=int(new["held"]),
                gained=int(new["gained_holds"]),
                lost=int(new["lost_holds"]),
                mean_matched_wait_delta_s=float(new["mean_matched_wait_delta_s"]),
            )
        )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "svg.fonttype": "none",
            "svg.hashsalt": "terminal-arc-live",
        }
    )
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8), layout="constrained")
    labels = ["Known / white", "Known / correlated", "New / white", "New / correlated"]
    y = range(4)
    gains = [r["gained"] for r in series]
    losses = [-r["lost"] for r in series]
    axes[0].barh(y, gains, color="#238b45", label="Gained hold")
    axes[0].barh(y, losses, color="#cb4545", label="Lost hold")
    axes[0].set_yticks(y, labels)
    axes[0].set_xlabel("Paired hold changes (81 pairs per cohort)")
    axes[0].set_title("Curved terminal preview vs straight")
    span = max(max(gains), max(-v for v in losses), 1) + 1.5
    axes[0].set_xlim(-span, span)
    for i, row in enumerate(series):
        axes[0].text(
            row["gained"] + 0.12, i, f'+{row["gained"]}', va="center", ha="left"
        )
        axes[0].text(-row["lost"] - 0.12, i, f'-{row["lost"]}', va="center", ha="right")
    axes[0].legend(
        loc="lower center", bbox_to_anchor=(0.5, -0.42), ncol=2, frameon=False
    )
    delta = [r["mean_matched_wait_delta_s"] for r in series]
    axes[1].barh(y, delta, color=["#cb4545" if v > 0 else "#238b45" for v in delta])
    axes[1].set_yticks(y, labels)
    axes[1].set_xlabel("Mean paired wait change (seconds)")
    axes[1].set_title("Wait measured to the shorter endpoint")
    lo, hi = min(0, *delta), max(0, *delta)
    padding = max(hi - lo, 1) * 0.28
    axes[1].set_xlim(lo - padding, hi + padding)
    for i, value in enumerate(delta):
        axes[1].text(
            value + (0.08 if value >= 0 else -0.08),
            i,
            f"{value:+.2f}",
            va="center",
            ha="left" if value >= 0 else "right",
        )
    for ax in axes:
        ax.axvline(0, color="#555555", linewidth=0.8)
        ax.invert_yaxis()
        ax.spines[["top", "right"]].set_visible(False)
        ax.set_axisbelow(True)
        ax.grid(axis="x", alpha=0.2)
    fig.suptitle(
        "Fixed simulator: 324 pairs; matched observations within each pair", fontsize=12
    )
    args.output.mkdir(parents=True)
    save_rows(args.output / "paired-changes.csv", series)
    svg = args.output / "paired-changes.svg"
    fig.savefig(svg, metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(args.output / "paired-changes.png", dpi=150)
    plt.close(fig)
    sources = dict(
        script_sha256=digest(Path(__file__)),
        inputs_sha256={
            name: digest(args.results / name)
            for name in ["summary.csv", "sources.json"]
        },
        series_sha256=digest(args.output / "paired-changes.csv"),
        svg_sha256=digest(svg),
        matplotlib_version=matplotlib.__version__,
    )
    (args.output / "paired-changes-sources.json").write_text(
        json.dumps(sources, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
