#!/usr/bin/env python3
"""Plot source estimator mixtures and fixed median-cutoff coverage transfer."""

import argparse
from itertools import product
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_estimator_mode_transfer import (
    ARMS,
    NOISES,
    PARTITIONS,
    SCORES,
    SIM_VIEWS,
    STRATA,
)
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np

LABELS = ["Raw / Recovery", "Raw / Preview", "Recovery / Preview"]
COLORS = ["#8c79bb", "#e69f00", "#009e73"]
MODE_LABELS = [
    "Recovery: current anchor",
    "Filter: current anchor",
    "Fusion: three anchors",
]


def save(fig, output, stem):
    svg = output / (stem + ".svg")
    fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(
        output / (stem + ".png"),
        dpi=150,
        bbox_inches="tight",
        metadata={"Software": "Matplotlib"},
    )
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        digest(Path(__file__).with_name("analyze_estimator_mode_transfer.py"))
        == source["analysis"]["script_sha256"]
    )
    strata = [
        r
        for r in read_rows(args.results / "stratum-summary.csv")
        if r["event_kind"] == "all"
        and r["score"] == "contrast"
        and r["budget"] == "all"
    ]
    transfer = [
        r
        for r in read_rows(args.results / "transfer-summary.csv")
        if r["event_kind"] == "all" and r["budget"] == "q50"
    ]
    assert (len(strata), len(transfer)) == (72, 36)
    modes = {
        (r["partition"], r["noise"], r["variant"], r["view"], r["stratum"]): r
        for r in strata
    }
    rates = {(r["noise"], r["variant"], r["view"], r["score"]): r for r in transfer}
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "svg.fonttype": "none",
            "svg.hashsalt": "estimator-mode-transfer",
        }
    )
    args.output.mkdir(parents=True)
    fig, axes = plt.subplots(2, 2, figsize=(14.6, 9.5))
    fig.subplots_adjust(
        left=0.21, right=0.98, top=0.90, bottom=0.18, wspace=0.72, hspace=0.46
    )
    for ni, noise in enumerate(NOISES):
        for ai, arm in enumerate(ARMS):
            ax = axes[ni, ai]
            labels = []
            for yi, (vi, partition) in enumerate(product(range(3), PARTITIONS)):
                view = SIM_VIEWS[vi]
                left = 0
                for s, color in zip(STRATA, COLORS):
                    value = float(
                        modes[partition, noise, arm, view, s]["case_mass_share"]
                    )
                    ax.barh(yi, value, left=left, color=color, height=0.72)
                    if value >= 0.08:
                        ax.text(
                            left + value / 2,
                            yi,
                            f"{100*value:.1f}%",
                            ha="center",
                            va="center",
                            fontsize=9,
                        )
                    left += value
                assert abs(left - 1) < 1e-12
                rate = rates[noise, arm, view, "contrast"]
                labels.append(
                    f'{LABELS[vi]}\n{partition}: {rate[partition+"_records"]} refs / {rate[partition+"_cases"]} cases'
                )
            ax.set_yticks(range(6), labels, fontsize=9)
            ax.set_xlim(0, 1)
            ax.set_xticks(np.arange(0, 1.01, 0.25), ["0", "25", "50", "75", "100"])
            ax.invert_yaxis()
            ax.set_xlabel("Share of parent case weight (%)")
            ax.set_title(
                f'{noise.title()} / {"Baseline" if arm=="epoch" else "Curved"}', pad=10
            )
    fig.suptitle(
        "Estimator mixture in the recorded simultaneous comparisons",
        fontsize=15,
        y=0.98,
    )
    fig.legend(
        handles=[Patch(facecolor=c, label=l) for c, l in zip(COLORS, MODE_LABELS)],
        loc="lower center",
        bbox_to_anchor=(0.55, 0.095),
        ncol=3,
        frameon=False,
    )
    fig.text(
        0.06,
        0.020,
        "Known and new seed cohorts were both observed previously. Each case has weight one in the parent reporting group.\nStratum weights are carried from that group; cases may cross strata. All event kinds, including overlapping unions, remain in the saved analysis.",
        fontsize=10,
    )
    save(fig, args.output, "estimator-mode-composition")

    fig, axes = plt.subplots(2, 3, figsize=(16.0, 9.1))
    fig.subplots_adjust(
        left=0.13, right=0.98, top=0.90, bottom=0.19, wspace=0.40, hspace=0.45
    )
    markers = [
        ("coverage_known", "o", "#0072b2", "none"),
        ("coverage_new", "o", "#d55e00", "#d55e00"),
        ("coverage_known_mix_new_rates", "s", "#009e73", "none"),
    ]
    for ni, noise in enumerate(NOISES):
        for si, score in enumerate(SCORES):
            ax = axes[ni, si]
            labels = []
            for yi, (arm, vi) in enumerate(product(ARMS, range(3))):
                r = rates[noise, arm, SIM_VIEWS[vi], score]
                ax.plot(
                    [float(r["coverage_known"]), float(r["coverage_new"])],
                    [yi, yi],
                    color="#bbbbbb",
                    linewidth=1.5,
                )
                for field, marker, color, face in markers:
                    if r[field] != "":
                        ax.scatter(
                            float(r[field]),
                            yi,
                            marker=marker,
                            edgecolor=color,
                            facecolor=face,
                            s=60,
                            linewidth=1.5,
                            zorder=3,
                        )
                labels.append(
                    f'{"Baseline" if arm=="epoch" else "Curved"}: {LABELS[vi]}'
                )
            ax.set_yticks(range(6), labels if si == 0 else [], fontsize=9)
            ax.set_ylim(5.6, -0.6)
            ax.set_xlim(-0.035, 1.035)
            ax.set_xticks(np.arange(0, 1.01, 0.25), ["0", "25", "50", "75", "100"])
            ax.grid(axis="x", alpha=0.25)
            ax.set_xlabel("Retained parent case weight (%)")
            ax.set_title(
                noise.title()
                + " / "
                + ["Contrast |D|", "Model sigma M", "Model sigma R"][si],
                pad=10,
            )
    fig.suptitle(
        "Coverage under the same known-cohort median cutoff", fontsize=15, y=0.98
    )
    handles = [
        Line2D(
            [],
            [],
            linestyle="none",
            marker=marker,
            markeredgecolor=color,
            markerfacecolor=face,
            markersize=8,
            label=label,
        )
        for (field, marker, color, face), label in zip(
            markers,
            [
                "Known observed",
                "New observed",
                "New within-mode rates at known mixture",
            ],
        )
    ]
    fig.legend(
        handles=handles,
        loc="lower center",
        bbox_to_anchor=(0.55, 0.10),
        ncol=3,
        frameon=False,
    )
    fig.text(
        0.06,
        0.020,
        "All event kinds; both arms use the same fixed known-cohort q50 cutoff within each view/noise/score. Tied ticks stay together.\nThe square standardizes new within-mode rates to the known mixture. This is descriptive and order dependent; no causal or calibrated-risk claim.",
        fontsize=10,
    )
    save(fig, args.output, "estimator-mode-coverage")
    save_rows(args.output / "estimator-mode-composition.csv", strata)
    save_rows(args.output / "estimator-mode-coverage.csv", transfer)
    provenance = dict(
        script_sha256=digest(Path(__file__)),
        input_sha256={
            n: digest(args.results / n)
            for n in ["stratum-summary.csv", "transfer-summary.csv", "sources.json"]
        },
        composition_rows=len(strata),
        coverage_rows=len(transfer),
        filters=dict(event_kind="all", coverage_budget="q50", composition_budget="all"),
        retrospective=True,
    )
    (args.output / "estimator-mode-figure-sources.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(
        json.dumps(
            dict(
                composition_rows=len(strata),
                coverage_rows=len(transfer),
                figures=2,
                output=str(args.output),
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
