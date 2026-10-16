#!/usr/bin/env python3
"""Plot the complete fixed-centre bottleneck partition at recorded age .5."""

import argparse
import json
from pathlib import Path

from analyze_admission_information_floors import BOTTLENECKS
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
        digest(Path(__file__).with_name("analyze_admission_information_floors.py"))
        == source["analysis"]["script_sha256"]
    )
    rows = [
        r
        for r in read_rows(args.results / "bottleneck-summary.csv")
        if (r["cohort"], r["subset"], r["group"]) == ("all", "age_0p5", "all")
    ]
    assert len(rows) == 8
    series = []
    for variant, expected in [("epoch", 415), ("curved", 461)]:
        for kind in ["actual", "preview"]:
            for path in ["straight", "commanded"]:
                row = next(
                    r
                    for r in rows
                    if (r["variant"], r["input_kind"], r["path"])
                    == (variant, kind, path)
                )
                assert int(row["checkpoints"]) == expected
                assert sum(int(row[k]) for k in BOTTLENECKS) == expected
                for category in BOTTLENECKS:
                    count = int(row[category])
                    series.append(
                        dict(
                            variant=variant,
                            input_kind=kind,
                            path=path,
                            checkpoints=expected,
                            cases=int(row["cases"]),
                            bottleneck=category,
                            count=count,
                            percent=100 * count / expected,
                        )
                    )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "admission-information-floors",
        }
    )
    fig, axes = plt.subplots(2, 2, figsize=(12, 5.6), sharex=True, layout="constrained")
    colors = [
        "#238b45",
        "#636363",
        "#756bb1",
        "#bcbddc",
        "#e6550d",
        "#fdd0a2",
        "#3182bd",
    ]
    names = [
        "Already certified",
        "Fixed central pose fails",
        "Heading interval has a blocking pose",
        "Heading interpolation gap",
        "Shared-anchor floor has a blocking pose",
        "Anchor-floor interpolation gap",
        "Noise-free same-anchor limit certifies",
    ]
    for i, (variant, arm, expected) in enumerate(
        [("epoch", "Baseline log", 415), ("curved", "Curved-terminal log", 461)]
    ):
        for j, (kind, input_label) in enumerate(
            [("actual", "recovery input"), ("preview", "preview input")]
        ):
            ax = axes[i, j]
            left = [0.0, 0.0]
            for category, color, label in zip(BOTTLENECKS, colors, names):
                chosen = [
                    next(
                        r
                        for r in series
                        if (r["variant"], r["input_kind"], r["path"], r["bottleneck"])
                        == (variant, kind, path, category)
                    )
                    for path in ["straight", "commanded"]
                ]
                widths = [r["percent"] for r in chosen]
                ax.barh(
                    [0, 1], widths, left=left, height=0.56, color=color, label=label
                )
                for y, (start, width, row) in enumerate(zip(left, widths, chosen)):
                    if width >= 4:
                        ax.text(
                            start + width / 2,
                            y,
                            str(row["count"]),
                            ha="center",
                            va="center",
                            fontsize=8,
                            color=(
                                "white"
                                if category
                                not in {"heading_grid_gap", "anchor_floor_grid_gap"}
                                else "#222222"
                            ),
                        )
                left = [a + b for a, b in zip(left, widths)]
            ax.set_yticks([0, 1], ["Line", "Recorded-steering arc"])
            ax.invert_yaxis()
            ax.set_xlim(0, 100)
            ax.set_xticks([0, 25, 50, 75, 100])
            ax.set_title(
                f"{arm} / {input_label} (N={expected})", fontsize=10, loc="left"
            )
            ax.spines[["top", "right"]].set_visible(False)
            if i == 1:
                ax.set_xlabel("All recorded checkpoints (%); labels are counts")
    fig.suptitle(
        "Fixed centres and full margin: information limits at recorded age 0.5 s",
        fontsize=12,
    )
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        ncol=2,
        frameon=False,
        loc="lower center",
        bbox_to_anchor=(0.56, -0.19),
        fontsize=8,
    )
    args.output.mkdir(parents=True)
    numeric = args.output / "information-bottlenecks.csv"
    save_rows(numeric, series)
    svg = args.output / "information-bottlenecks.svg"
    fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(
        args.output / "information-bottlenecks.png", dpi=160, bbox_inches="tight"
    )
    plt.close(fig)
    fingerprints = dict(
        script_sha256=digest(Path(__file__)),
        inputs_sha256={
            n: digest(args.results / n)
            for n in ["bottleneck-summary.csv", "sources.json"]
        },
        artifacts_sha256={p.name: digest(p) for p in [numeric, svg]},
        matplotlib_version=matplotlib.__version__,
    )
    (args.output / "information-bottlenecks-sources.json").write_text(
        json.dumps(fingerprints, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
