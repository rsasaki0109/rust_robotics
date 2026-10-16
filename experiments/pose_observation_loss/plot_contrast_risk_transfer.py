#!/usr/bin/env python3
"""Plot both retrospective cohorts, arms, noises, views and risk scores."""

import argparse
from itertools import product
import json
from pathlib import Path

from analyze_contrast_risk_transfer import (
    ARMS,
    NOISES,
    PARTITIONS,
    SCORES,
    TARGETS,
    VIEWS,
)
from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

LABELS = [
    "S: Raw / Recovery",
    "S: Raw / Preview",
    "S: Recovery / Preview",
    "L: Recovery now",
    "L: Preview now",
    "L: Recovery +.18 s",
    "L: Preview +.18 s",
    "L: Recovery +.50 s",
    "L: Preview +.50 s",
    "L: Recovery +1.50 s",
    "L: Preview +1.50 s",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        digest(Path(__file__).with_name("analyze_contrast_risk_transfer.py"))
        == source["analysis"]["script_sha256"]
    )
    ranks = read_rows(args.results / "ranking.csv")
    numeric = []
    for r, target in product(ranks, TARGETS):
        numeric.append(
            dict(
                **{
                    k: r[k]
                    for k in [
                        "partition",
                        "noise",
                        "variant",
                        "event_kind",
                        "view",
                        "score",
                        "records",
                        "matched",
                        "unavailable",
                        "cases",
                        "matched_cases",
                        "case_mass",
                        "zero_or_roundoff_pairs",
                    ]
                },
                target=target,
                case_weighted_auc=r[target + "_case_weighted_auc"],
                row_auc=r[target + "_row_auc"],
                baseline_case_weighted_risk=r[target + "_baseline_case_weighted_risk"],
                outside_rows=r[target + "_outside_rows"],
                inside_rows=r[target + "_inside_rows"],
            )
        )
    assert len(numeric) == 3168
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
        if r["event_kind"] == "all"
    }
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "contrast-risk-transfer",
        }
    )
    cmap = plt.get_cmap("viridis").copy()
    cmap.set_bad("#dddddd")
    args.output.mkdir(parents=True)
    for target in TARGETS:
        fig, axes = plt.subplots(4, 2, figsize=(14.4, 17.5))
        fig.subplots_adjust(
            left=0.20, right=0.98, top=0.94, bottom=0.10, wspace=0.75, hspace=0.48
        )
        for i, (partition, noise) in enumerate(product(PARTITIONS, NOISES)):
            for j, arm in enumerate(ARMS):
                ax = axes[i, j]
                data = np.full((len(VIEWS), len(SCORES)), np.nan)
                counts = []
                for vi, view in enumerate(VIEWS):
                    for si, score in enumerate(SCORES):
                        r = lookup[partition, noise, arm, view, score, target]
                        if r["case_weighted_auc"] != "":
                            data[vi, si] = float(r["case_weighted_auc"])
                    counts.append((r["matched"], r["records"]))
                handle = ax.imshow(
                    np.ma.masked_invalid(data), vmin=0, vmax=1, cmap=cmap, aspect="auto"
                )
                ax.set_xticks(
                    range(3),
                    ["Contrast |D|", "Model sigma M", "Model sigma R"],
                    fontsize=8,
                )
                ax.set_yticks(
                    range(11),
                    [
                        f"{label}  {n}/{total}"
                        for label, (n, total) in zip(LABELS, counts)
                    ],
                    fontsize=8,
                )
                ax.set_title(
                    f'{partition.title()} / {noise}; {"Baseline" if arm=="epoch" else "Curved"}',
                    pad=9,
                )
                for vi, si in product(range(11), range(3)):
                    value = data[vi, si]
                    ax.text(
                        si,
                        vi,
                        "NA" if np.isnan(value) else f"{value:.2f}",
                        ha="center",
                        va="center",
                        fontsize=8,
                        color="black" if np.isnan(value) or value > 0.58 else "white",
                    )
        fig.suptitle(
            f"Case-balanced AUC: {target} error norm > .10 m\nRetrospective known/new seed-cohort comparison; fixed score directions",
            y=0.985,
            fontsize=13,
        )
        bar = fig.add_axes([0.28, 0.052, 0.58, 0.012])
        fig.colorbar(
            handle, cax=bar, orientation="horizontal", ticks=[0, 0.25, 0.5, 0.75, 1]
        )
        bar.set_xlabel(
            "AUC; .5 = chance ranking. Higher score predicts larger error; directions are never flipped.",
            fontsize=9,
        )
        bar.xaxis.set_label_position("top")
        bar.xaxis.labelpad = 6
        fig.text(
            0.10,
            0.010,
            "S: simultaneous audited pair; L: lagged pre/post pair. Row labels show matched/requested references. Grey NA: an outcome class is absent.\n"
            "Lagged |D| uses known truth motion and is an oracle diagnostic. Cases are equally weighted within each reporting group.\n"
            "Both seed cohorts were observed previously. No calibrated probability, confidence interval or new release/controller is claimed.",
            fontsize=9,
        )
        stem = "contrast-risk-" + target
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
    save_rows(args.output / "contrast-risk-ranking.csv", numeric)
    provenance = dict(
        script_sha256=digest(Path(__file__)),
        input_sha256={
            n: digest(args.results / n) for n in ["ranking.csv", "sources.json"]
        },
        numeric_rows=len(numeric),
        targets=TARGETS,
        weighting="case-balanced AUC with exact score-tick ties",
        retrospective=True,
    )
    (args.output / "contrast-risk-ranking-sources.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(
        json.dumps(
            dict(
                numeric_rows=len(numeric), figures=len(TARGETS), output=str(args.output)
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
