#!/usr/bin/env python3
"""Plot every truth/estimate category for the fixed fresh recovery checkpoints."""

import argparse
from collections import Counter
import json
from pathlib import Path

from analyze_commanded_recovery_arc import flag
from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

CATEGORIES = ["both_clear", "optimistic", "conservative", "both_blocked"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output directory must be new")
    decisions = [
        r
        for r in read_rows(args.results / "decisions.csv")
        if "age_0p5" in r["reasons"].split("+")
    ]
    series = []
    for variant, expected in [("epoch", 415), ("curved", 461)]:
        for input_kind in ["actual", "preview"]:
            for rule in ["straight", "commanded"]:
                rows = [
                    r
                    for r in decisions
                    if (r["variant"], r["input_kind"], r["rule"])
                    == (variant, input_kind, rule)
                ]
                assert len(rows) == expected
                counts = Counter()
                for row in rows:
                    estimate, truth = flag(row["estimate_clear"]), flag(
                        row["truth_clear"]
                    )
                    category = {
                        (True, True): "both_clear",
                        (True, False): "optimistic",
                        (False, True): "conservative",
                        (False, False): "both_blocked",
                    }[estimate, truth]
                    counts[category] += 1
                assert sum(counts.values()) == expected
                for category in CATEGORIES:
                    series.append(
                        dict(
                            variant=variant,
                            input_kind=input_kind,
                            rule=rule,
                            category=category,
                            count=counts[category],
                            total=expected,
                            percent=100 * counts[category] / expected,
                        )
                    )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "commanded-recovery-arc",
        }
    )
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, layout="constrained")
    colors = ["#238b45", "#d95f02", "#3182bd", "#d9d9d9"]
    names = [
        "Estimate and truth permit",
        "Estimate permits; truth blocks",
        "Estimate blocks; truth permits",
        "Estimate and truth block",
    ]
    labels = [
        "Recovery input / line",
        "Recovery input / commanded arc",
        "Preview input / line",
        "Preview input / commanded arc",
    ]
    groups = [(i, r) for i in ["actual", "preview"] for r in ["straight", "commanded"]]
    for ax, variant, title in zip(
        axes,
        ["epoch", "curved"],
        [
            "Straight terminal baseline: 415 checkpoints from 87 timeout cases",
            "Curved terminal arm: 461 checkpoints from 93 timeout cases",
        ],
    ):
        left = [0.0] * len(groups)
        for category, color, name in zip(CATEGORIES, colors, names):
            rows = [
                next(
                    r
                    for r in series
                    if (r["variant"], r["input_kind"], r["rule"], r["category"])
                    == (variant, input_kind, rule, category)
                )
                for input_kind, rule in groups
            ]
            widths = [r["percent"] for r in rows]
            ax.barh(range(4), widths, left=left, height=0.68, color=color, label=name)
            for i, row in enumerate(rows):
                if row["percent"] >= 4:
                    ax.text(
                        left[i] + widths[i] / 2,
                        i,
                        str(row["count"]),
                        ha="center",
                        va="center",
                        fontsize=8,
                        color="black" if category == "both_blocked" else "white",
                    )
                left[i] += widths[i]
        ax.set_yticks(range(4), labels)
        ax.invert_yaxis()
        ax.set_title(title, fontsize=10, loc="left")
        ax.set_xlim(0, 100)
        ax.set_xticks([0, 20, 40, 60, 80, 100])
        ax.spines[["top", "right"]].set_visible(False)
    axes[1].set_xlabel(
        "Share of recorded age-0.5 s checkpoints (%)\nNumbers inside bars are checkpoint counts"
    )
    fig.suptitle(
        "Fixed-pose recovery-path decisions; no controller rollout", fontsize=12
    )
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        ncol=2,
        frameon=False,
        loc="lower center",
        bbox_to_anchor=(0.58, -0.065),
    )
    args.output.mkdir(parents=True)
    series_path = args.output / "recovery-decisions.csv"
    save_rows(series_path, series)
    svg = args.output / "recovery-decisions.svg"
    fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(args.output / "recovery-decisions.png", dpi=160, bbox_inches="tight")
    plt.close(fig)
    sources = dict(
        script_sha256=digest(Path(__file__)),
        inputs_sha256={
            name: digest(args.results / name)
            for name in ["decisions.csv", "sources.json"]
        },
        artifacts_sha256={p.name: digest(p) for p in [series_path, svg]},
        matplotlib_version=matplotlib.__version__,
    )
    (args.output / "recovery-decisions-sources.json").write_text(
        json.dumps(sources, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
