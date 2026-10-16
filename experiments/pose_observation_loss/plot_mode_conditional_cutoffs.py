#!/usr/bin/env python3
"""Plot pooled/mode-conditional coverage and error fractions at fixed q50."""

import argparse
from itertools import product
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_mode_conditional_cutoffs import (
    ARMS,
    NOISES,
    PARTITIONS,
    SCORES,
    SIM_VIEWS,
    TARGETS,
)
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

VIEWS = ["Raw / Recovery", "Raw / Preview", "Recovery / Preview"]
SCORE_LABELS = ["|D|", "sigma M", "sigma R"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        digest(Path(__file__).with_name("analyze_mode_conditional_cutoffs.py"))
        == source["analysis"]["script_sha256"]
    )
    base = [
        r
        for r in read_rows(args.results / "comparison.csv")
        if r["event_kind"] == "all" and r["stratum"] == "all" and r["budget"] == "q50"
    ]
    numeric = []
    for r, target in product(base, TARGETS):
        numeric.append(
            dict(
                **{
                    k: r[k]
                    for k in [
                        "partition",
                        "noise",
                        "variant",
                        "view",
                        "score",
                        "budget",
                        "records",
                        "cases",
                        "case_mass",
                        "pooled_threshold_id",
                        "conditional_threshold_ids",
                        "conditional_support",
                        "conditional_unsupported_rows",
                    ]
                },
                target=target,
                **{
                    method + "_" + field: r[method + "_" + field]
                    for method in ["pooled", "conditional"]
                    for field in [
                        "retained_rows",
                        "retained_cases",
                        "retained_case_mass",
                        "case_weighted_coverage",
                    ]
                },
                **{
                    method
                    + "_outside_fraction": r[
                        method + "_" + target + "_case_weighted_risk"
                    ]
                    for method in ["pooled", "conditional"]
                },
            )
        )
    assert len(numeric) == 144
    lookup = {
        (
            r["partition"],
            r["noise"],
            r["variant"],
            r["view"],
            r["score"],
            r["target"],
        ): r
        for r in numeric
    }
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "mode-conditional-cutoffs",
        }
    )
    cmap = plt.get_cmap("viridis").copy()
    cmap.set_bad("#dddddd")
    args.output.mkdir(parents=True)
    for target in TARGETS:
        fig, axes = plt.subplots(4, 2, figsize=(14.6, 17.2))
        fig.subplots_adjust(
            left=0.22, right=0.98, top=0.94, bottom=0.12, wspace=0.72, hspace=0.48
        )
        for ri, (partition, noise) in enumerate(product(PARTITIONS, NOISES)):
            for ai, arm in enumerate(ARMS):
                ax = axes[ri, ai]
                values = np.full((9, 4), np.nan)
                labels = []
                for yi, (vi, si) in enumerate(product(range(3), range(3))):
                    r = lookup[partition, noise, arm, SIM_VIEWS[vi], SCORES[si], target]
                    fields = [
                        "pooled_case_weighted_coverage",
                        "conditional_case_weighted_coverage",
                        "pooled_outside_fraction",
                        "conditional_outside_fraction",
                    ]
                    for ci, f in enumerate(fields):
                        if r[f] != "":
                            values[yi, ci] = float(r[f])
                    labels.append(VIEWS[vi] + "; " + SCORE_LABELS[si])
                handle = ax.imshow(
                    np.ma.masked_invalid(values),
                    vmin=0,
                    vmax=1,
                    cmap=cmap,
                    aspect="auto",
                )
                ax.set_yticks(range(9), labels, fontsize=8)
                ax.set_xticks(
                    range(4),
                    [
                        "Pooled\ncoverage",
                        "Mode\ncoverage",
                        "Pooled\noutside",
                        "Mode\noutside",
                    ],
                    fontsize=8,
                )
                ax.set_title(
                    f'{partition.title()} / {noise}; {"Baseline" if arm=="epoch" else "Curved"}',
                    pad=9,
                )
                for yi, ci in product(range(9), range(4)):
                    value = values[yi, ci]
                    ax.text(
                        ci,
                        yi,
                        "NA" if np.isnan(value) else f"{100*value:.1f}",
                        ha="center",
                        va="center",
                        fontsize=8,
                        color="black" if np.isnan(value) or value >= 0.5 else "white",
                    )
        fig.suptitle(
            f"Pooled versus mode-conditional q50 cutoffs: {target} error",
            fontsize=15,
            y=0.985,
        )
        cax = fig.add_axes([0.40, 0.067, 0.38, 0.012])
        bar = fig.colorbar(handle, cax=cax, orientation="horizontal")
        bar.set_label(
            "Case-weighted coverage / retained outside fraction (cells show %)"
        )
        bar.ax.xaxis.set_label_position("top")
        bar.ax.xaxis.labelpad = 6
        fig.text(
            0.07,
            0.011,
            "All event kinds. Outside: observed "
            + target
            + " norm >.10 m. Grey NA: empty retained set or unavailable fit.\nKnown seeds alone fit both shared-arm rules; modes retain the original parent case weights. Tied ticks stay together.\nBoth cohorts were observed previously. A conditional fitting quantile does not calibrate evaluation coverage or error risk.",
            fontsize=9,
        )
        stem = "mode-conditional-" + target
        svg = args.output / (stem + ".svg")
        fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
        svg.write_text(
            "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(
            args.output / (stem + ".png"),
            dpi=150,
            bbox_inches="tight",
            metadata={"Software": "Matplotlib"},
        )
        plt.close(fig)
    save_rows(args.output / "mode-conditional-cutoffs.csv", numeric)
    provenance = dict(
        script_sha256=digest(Path(__file__)),
        input_sha256={
            n: digest(args.results / n) for n in ["comparison.csv", "sources.json"]
        },
        numeric_rows=len(numeric),
        targets=TARGETS,
        filters=dict(event_kind="all", stratum="all", budget="q50"),
        retrospective=True,
    )
    (args.output / "mode-conditional-cutoffs-sources.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(
        json.dumps(
            dict(numeric_rows=len(numeric), figures=2, output=str(args.output)),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
