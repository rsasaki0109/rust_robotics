#!/usr/bin/env python3
"""Plot complete correction populations and retained anchor information."""

import argparse
from itertools import product
import json
import math
from pathlib import Path

from analyze_correction_information_renewal import INPUTS, KINDS, OFFSETS
from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

PARTITIONS = {
    "Shared anchor with pre": "shared_anchor_pairs",
    "No shared anchor": "anchor_disjoint",
    "Unavailable": "unavailable",
}
WEIGHTS = {
    "IDs shared with pre": "mean_post_shared_anchor_weight",
    "Event's new anchor": "mean_post_event_new_anchor_weight",
    "Other anchor IDs": "mean_post_other_anchor_weight",
}
SIGMAS = {
    "Retained covariance": "pooled_change_sigma",
    "Independent pre/post": "pooled_independent_sigma",
}
ARMS = ["epoch", "curved"]
PHASES = list(OFFSETS)[1:]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    source = json.loads((args.results / "sources.json").read_text())
    analyzer = Path(__file__).with_name("analyze_correction_information_renewal.py")
    assert digest(analyzer) == source["analysis"]["script_sha256"]
    summary = {
        (r["variant"], r["event_kind"], r["phase"], r["input_kind"]): r
        for r in read_rows(args.results / "summary.csv")
        if r["cohort"] == "all" and r["post_estimator"] == "all"
    }
    numeric = []
    for arm, kind, phase, inp in product(ARMS, ["all", *KINDS], PHASES, INPUTS):
        r = summary[arm, kind, phase, inp]
        n, matched, shared, missing = [
            int(r[k])
            for k in ["records", "matched", "shared_anchor_pairs", "unavailable"]
        ]
        assert matched + missing == n
        common = dict(
            variant=arm,
            event_kind=kind,
            phase=phase,
            input_kind=inp,
            records=n,
            matched=matched,
            unavailable=missing,
            event_anchor_replaced=int(r["event_anchor_replaced"]),
            shared_observation_pairs=int(r["shared_observation_pairs"]),
        )
        for category, count in zip(PARTITIONS, [shared, matched - shared, missing]):
            numeric.append(
                dict(
                    **common,
                    family="requested_anchor_partition",
                    category=category,
                    count=count,
                    percent=100 * count / n if n else 0.0,
                    value_m="",
                )
            )
        if matched:
            assert math.isclose(
                sum(float(r[field]) for field in WEIGHTS.values()), 1.0, abs_tol=1e-12
            )
        for category, field in WEIGHTS.items():
            numeric.append(
                dict(
                    **common,
                    family="matched_anchor_weight",
                    category=category,
                    count="",
                    percent=100 * float(r[field]) if matched else "",
                    value_m="",
                )
            )
        for category, field in SIGMAS.items():
            numeric.append(
                dict(
                    **common,
                    family="error_change_sigma",
                    category=category,
                    count="",
                    percent="",
                    value_m=float(r[field]) if matched else "",
                )
            )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "correction-information-renewal",
        }
    )
    fig = plt.figure(figsize=(12.4, 12.4))
    grid = fig.add_gridspec(
        5,
        2,
        height_ratios=[1, 1, 0.20, 1, 0.30],
        left=0.10,
        right=0.99,
        bottom=0.09,
        top=0.84,
        wspace=0.27,
        hspace=0.56,
    )
    axes = [[fig.add_subplot(grid[row, j]) for j in range(2)] for row in [0, 1, 3]]
    legend_axes = [fig.add_subplot(grid[row, :]) for row in [2, 4]]
    for ax in legend_axes:
        ax.axis("off")
    palette = ["#756bb1", "#238b45", "#bdbdbd"]
    weight_colors = ["#756bb1", "#2b8cbe", "#e08214"]
    for j, inp in enumerate(INPUTS):

        def one(arm, phase, family, category):
            return next(
                r
                for r in numeric
                if r["variant"] == arm
                and r["phase"] == phase
                and r["input_kind"] == inp
                and r["event_kind"] == "all"
                and r["family"] == family
                and r["category"] == category
            )

        for row, family, fields, colors in [
            (0, "requested_anchor_partition", PARTITIONS, palette),
            (1, "matched_anchor_weight", WEIGHTS, weight_colors),
        ]:
            ax = axes[row][j]
            for ai, arm in enumerate(ARMS):
                positions = [i + (-0.18 if ai == 0 else 0.18) for i in range(4)]
                bottom = [0.0] * 4
                for category, color in zip(fields, colors):
                    values = [one(arm, p, family, category)["percent"] for p in PHASES]
                    ax.bar(
                        positions,
                        values,
                        width=0.32,
                        bottom=bottom,
                        color=color,
                        hatch="" if ai == 0 else "//",
                        label=category if ai == 0 else None,
                    )
                    bottom = [a + b for a, b in zip(bottom, values)]
            ax.set_ylim(0, 100)
            ax.set_xticks(range(4), ["Immediate", "+.18 s", "+.50 s", "+1.50 s"])
            if j == 0:
                ax.set_ylabel(
                    "Percent of requested pairs"
                    if row == 0
                    else "Mean post anchor weight (%)"
                )
            if row == 0:
                counts = [
                    summary[arm, "all", PHASES[0], inp]["records"] for arm in ARMS
                ]
                ax.set_title(
                    f'{"Recovery" if inp=="actual" else "Preview"}: anchor sharing with pre\n'
                    f"{counts[0]} baseline / {counts[1]} curved requests per phase",
                    pad=12,
                )
            else:
                ax.set_title("Anchor-weight partition among matched pairs", pad=10)
            if row == 1 and j == 0:
                legend_axes[0].legend(
                    *ax.get_legend_handles_labels(),
                    ncol=3,
                    loc="center",
                    frameon=False,
                    fontsize=9,
                )
        ax = axes[2][j]
        colors = ["#2166ac", "#92c5de", "#b35806", "#fdb863"]
        for k, (arm, (category, field)) in enumerate(product(ARMS, SIGMAS.items())):
            positions = [i - 0.36 + 0.24 * k for i in range(4)]
            values = [
                100 * one(arm, p, "error_change_sigma", category)["value_m"]
                for p in PHASES
            ]
            ax.bar(
                positions,
                values,
                width=0.21,
                color=colors[k],
                label=f'{"Baseline" if arm=="epoch" else "Curved"}: {category.lower()}',
            )
        cap = max(
            r["value_m"]
            for r in numeric
            if r["family"] == "error_change_sigma" and r["event_kind"] == "all"
        )
        ax.set_ylim(0, 120 * cap)
        ax.set_xticks(range(4), ["Immediate", "+.18 s", "+.50 s", "+1.50 s"])
        ax.set_title("Pre/post error change: partial-model sigma", pad=10)
        if j == 0:
            ax.set_ylabel("Sigma per axis (cm)")
            legend_axes[1].legend(
                *ax.get_legend_handles_labels(),
                ncol=2,
                loc="center",
                frameon=False,
                fontsize=9,
            )
        ax.grid(axis="y", alpha=0.2)
        ax.set_axisbelow(True)
    fig.legend(
        [Patch(facecolor=c, label=n) for n, c in zip(PARTITIONS, palette)]
        + [
            Patch(facecolor="white", edgecolor="black", label="Baseline (left)"),
            Patch(
                facecolor="white", edgecolor="black", hatch="//", label="Curved (right)"
            ),
        ],
        [*PARTITIONS, "Baseline (left)", "Curved (right)"],
        loc="upper center",
        bbox_to_anchor=(0.5, 0.995),
        ncol=3,
        frameon=False,
    )
    fig.suptitle(
        "All 2,583 recorded correction opportunities; all event kinds and both inputs",
        y=0.90,
        fontsize=12,
    )
    fig.text(
        0.10,
        0.018,
        "Top: complete requested populations, including unavailable post snapshots. Middle/bottom: matched pairs only.\n"
        "Partial noise + anchor model; excludes drift/transport. Inputs and phases overlap; recorded control and held goals remain fixed.",
        fontsize=9,
    )
    args.output.mkdir(parents=True)
    stem = "correction-information-comparisons"
    save_rows(args.output / (stem + ".csv"), numeric)
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
    provenance = dict(
        script_sha256=digest(Path(__file__)),
        input_sha256={
            n: digest(args.results / n) for n in ["summary.csv", "sources.json"]
        },
        numeric_rows=len(numeric),
        weighting="one matched event/input/phase pair; requested partition retains censoring",
    )
    (args.output / (stem + "-sources.json")).write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(
        json.dumps(dict(numeric_rows=len(numeric), output=str(args.output)), indent=2)
    )


if __name__ == "__main__":
    main()
