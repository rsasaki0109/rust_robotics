#!/usr/bin/env python3
"""Plot fixed age-.5 position/component RMS with actual estimator denominators."""

import argparse
import json
import math
from pathlib import Path
import statistics

from analyze_epoch_position_components import PARTS
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
        digest(Path(__file__).with_name("analyze_epoch_position_components.py"))
        == source["analysis"]["script_sha256"]
    )
    rows = [
        r
        for r in read_rows(args.results / "positions.csv")
        if "age_0p5" in r["reasons"].split("+")
    ]
    assert len(rows) == 1752
    assert all(
        r["estimator"] == "recovery_mean" for r in rows if r["input_kind"] == "actual"
    )
    for variant, n in [("epoch", 415), ("curved", 461)]:
        for kind in ["actual", "preview"]:
            assert (
                sum((r["variant"], r["input_kind"]) == (variant, kind) for r in rows)
                == n
            )
    metrics = ["error", *PARTS]
    series = []
    for noise in ["white", "correlated"]:
        for variant in ["epoch", "curved"]:
            for kind, estimators in [
                ("actual", ["recovery_mean"]),
                ("preview", ["motion_filter", "three_epoch"]),
            ]:
                for estimator in estimators:
                    chosen = [
                        r
                        for r in rows
                        if r["cohort"].endswith(noise)
                        and (r["variant"], r["input_kind"], r["estimator"])
                        == (variant, kind, estimator)
                    ]
                    for metric in metrics:
                        value = (
                            math.sqrt(
                                statistics.fmean(
                                    float(r[metric + "_norm"]) ** 2 for r in chosen
                                )
                            )
                            if chosen
                            else ""
                        )
                        series.append(
                            dict(
                                noise=noise,
                                variant=variant,
                                input_kind=kind,
                                estimator=estimator,
                                checkpoints=len(chosen),
                                metric=metric,
                                rms=value,
                            )
                        )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "epoch-position-components",
        }
    )
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True, layout="constrained")
    colors = ["#333333", "#3182bd", "#e6550d", "#31a354", "#756bb1"]
    legend = [
        "Position error",
        "Observation noise",
        "Shared correction anchor",
        "Drift",
        "Transport residual",
    ]
    maximum = max(float(r["rms"]) for r in series if r["rms"] != "")
    for row_index, (noise, title) in enumerate(
        [("white", "Independent noise"), ("correlated", "Correlation time 1 s")]
    ):
        for column, (kind, input_label) in enumerate(
            [("actual", "Recovery input"), ("preview", "Preview input")]
        ):
            ax = axes[row_index, column]
            groups = [
                (v, e)
                for v in ["epoch", "curved"]
                for e in (
                    ["recovery_mean"]
                    if kind == "actual"
                    else ["motion_filter", "three_epoch"]
                )
            ]
            labels = []
            for variant, estimator in groups:
                record = next(
                    r
                    for r in series
                    if (r["noise"], r["variant"], r["input_kind"], r["estimator"])
                    == (noise, variant, kind, estimator)
                )
                arm = "Baseline" if variant == "epoch" else "Curved terminal"
                name = {
                    "recovery_mean": "current epoch",
                    "motion_filter": "filter fallback",
                    "three_epoch": "3 epochs",
                }[estimator]
                labels.append(f"{arm} / {name}\nN={record['checkpoints']}")
            for offset, (metric, color, name) in enumerate(
                zip(metrics, colors, legend)
            ):
                values = [
                    next(
                        r["rms"]
                        for r in series
                        if (
                            r["noise"],
                            r["variant"],
                            r["input_kind"],
                            r["estimator"],
                            r["metric"],
                        )
                        == (noise, v, kind, e, metric)
                    )
                    for v, e in groups
                ]
                ax.barh(
                    [i + (offset - 2) * 0.14 for i in range(len(groups))],
                    [0 if v == "" else v for v in values],
                    height=0.12,
                    color=color,
                    label=name,
                )
            ax.set_yticks(range(len(groups)), labels, fontsize=8)
            ax.invert_yaxis()
            ax.set_xlim(0, maximum * 1.12)
            ax.set_title(f"{title} / {input_label}", fontsize=10, loc="left")
            ax.spines[["top", "right"]].set_visible(False)
            ax.grid(axis="x", alpha=0.15)
            ax.set_axisbelow(True)
            if row_index == 1:
                ax.set_xlabel("2D RMS length (m); components are not additive")
    fig.suptitle(
        "Recorded age-0.5 s checkpoints: position error and weighted components",
        fontsize=12,
    )
    handles, names = axes[0, 0].get_legend_handles_labels()
    fig.legend(
        handles,
        names,
        ncol=3,
        frameon=False,
        loc="lower center",
        bbox_to_anchor=(0.56, -0.055),
    )
    args.output.mkdir(parents=True)
    numeric = args.output / "position-components.csv"
    save_rows(numeric, series)
    svg = args.output / "position-components.svg"
    fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(args.output / "position-components.png", dpi=160, bbox_inches="tight")
    plt.close(fig)
    fingerprints = dict(
        script_sha256=digest(Path(__file__)),
        inputs_sha256={
            n: digest(args.results / n) for n in ["positions.csv", "sources.json"]
        },
        artifacts_sha256={p.name: digest(p) for p in [numeric, svg]},
        matplotlib_version=matplotlib.__version__,
    )
    (args.output / "position-components-sources.json").write_text(
        json.dumps(fingerprints, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
