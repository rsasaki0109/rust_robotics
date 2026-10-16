#!/usr/bin/env python3
"""Plot fixed fresh-checkpoint mismatch categories for both recovery paths."""

import argparse
import json
from pathlib import Path

from analyze_commanded_arc_components import CATEGORIES
from analyze_epoch_preview import digest, read_rows
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
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        digest(Path(__file__).with_name("analyze_commanded_arc_components.py"))
        == source["analysis"]["script_sha256"]
    )
    rows = {
        (r["variant"], r["input_kind"], r["rule"], r["group"]): r
        for r in read_rows(args.results / "summary.csv")
        if r["cohort"] == "all" and r["subset"] == "age_0p5"
    }
    groups = [
        (rule, group)
        for group in ["optimistic", "conservative"]
        for rule in ["straight", "commanded"]
    ]
    series = []
    for variant, total in [("epoch", 415), ("curved", 461)]:
        for input_kind in ["actual", "preview"]:
            for rule, group in groups:
                row = rows[variant, input_kind, rule, group]
                assert (
                    int(rows[variant, input_kind, rule, "all"]["checkpoints"]) == total
                )
                assert sum(int(row[k]) for k in CATEGORIES) == int(row["checkpoints"])
                assert int(row["same_decision"]) == 0
                for category in CATEGORIES:
                    series.append(
                        dict(
                            variant=variant,
                            input_kind=input_kind,
                            rule=rule,
                            group=group,
                            category=category,
                            count=int(row[category]),
                            mismatch_points=int(row["checkpoints"]),
                            total_points=total,
                        )
                    )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "commanded-arc-components",
        }
    )
    fig, axes = plt.subplots(2, 2, figsize=(11, 6), sharex=True, layout="constrained")
    colors = ["#1f78b4", "#33a02c", "#ff7f00", "#6a3d9a"]
    names = ["Position only", "Heading only", "Either alone", "Both required"]
    labels = [
        "Line / optimistic",
        "Arc / optimistic",
        "Line / conservative",
        "Arc / conservative",
    ]
    maximum = max(r["mismatch_points"] for r in series)
    for row_index, (variant, arm, total) in enumerate(
        [
            ("epoch", "Baseline", 415),
            ("curved", "Curved terminal", 461),
        ]
    ):
        for column, (input_kind, label) in enumerate(
            [("actual", "recovery input"), ("preview", "preview input")]
        ):
            ax = axes[row_index, column]
            left = [0] * 4
            for category, color, name in zip(CATEGORIES, colors, names):
                chosen = [
                    next(
                        r
                        for r in series
                        if (
                            r["variant"],
                            r["input_kind"],
                            r["rule"],
                            r["group"],
                            r["category"],
                        )
                        == (variant, input_kind, rule, group, category)
                    )
                    for rule, group in groups
                ]
                counts = [r["count"] for r in chosen]
                ax.barh(
                    range(4), counts, left=left, height=0.65, color=color, label=name
                )
                for i, count in enumerate(counts):
                    if count >= 4:
                        ax.text(
                            left[i] + count / 2,
                            i,
                            str(count),
                            ha="center",
                            va="center",
                            fontsize=8,
                            color="black" if category == "either_alone" else "white",
                        )
                    left[i] += count
            for i, count in enumerate(left):
                ax.text(count + 1, i, str(count), ha="left", va="center", fontsize=8)
            ax.set_yticks(range(4), labels)
            ax.invert_yaxis()
            ax.set_xlim(0, maximum * 1.13 + 2)
            ax.set_title(
                f"{arm} / {label}\n{total} recorded age-0.5 s points",
                fontsize=10,
                loc="left",
            )
            ax.spines[["top", "right"]].set_visible(False)
            if row_index == 1:
                ax.set_xlabel("Mismatch checkpoints (repeated within cases)")
    fig.suptitle(
        "Fixed poses and steering: which truth substitution changes the decision?",
        fontsize=12,
    )
    handles, legend_labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(
        handles,
        legend_labels,
        ncol=4,
        frameon=False,
        loc="lower center",
        bbox_to_anchor=(0.56, -0.04),
    )
    args.output.mkdir(parents=True)
    series_path = args.output / "coordinate-effects.csv"
    save_rows(series_path, series)
    svg = args.output / "coordinate-effects.svg"
    fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(args.output / "coordinate-effects.png", dpi=160, bbox_inches="tight")
    plt.close(fig)
    sources = dict(
        script_sha256=digest(Path(__file__)),
        inputs_sha256={
            n: digest(args.results / n) for n in ["summary.csv", "sources.json"]
        },
        artifacts_sha256={p.name: digest(p) for p in [series_path, svg]},
        matplotlib_version=matplotlib.__version__,
    )
    (args.output / "coordinate-effects-sources.json").write_text(
        json.dumps(sources, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
