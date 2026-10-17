#!/usr/bin/env python3
"""Display fixed cumulative AUC scopes and disjoint comparison mass."""

import argparse
from itertools import product
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows

PARTITIONS = ["known", "new", "fresh"]
TITLES = {"known": "Known", "new": "Previous additional", "fresh": "Latest additional"}
CONDITIONS = list(product(["white", "correlated"], ["epoch", "curved"]))
MODES = ["filter_single", "fusion_three"]
POPULATIONS = ["source", "common_family"]
SCOPES = ["full", "same_seed", "same_family", "same_window", "same_case"]
BINS = [
    "same_case",
    "other_case_same_window",
    "other_window_same_family",
    "other_family_same_seed",
    "other_seed",
]
KEY = [
    "partition",
    "noise",
    "variant",
    "stratum",
    "target",
    "population",
    "planner_selection",
    "score",
    "label",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new output directory")
    args.output.mkdir(parents=True)
    ranks = read_rows(args.results / "ranking-summary.csv")
    bins = read_rows(args.results / "hierarchy-bins.csv")
    lookup = {tuple(r[k] for k in KEY): r for r in ranks}
    bin_lookup = {tuple(r[k] for k in [*KEY, "bin"]): r for r in bins}
    assert len(lookup) == len(ranks) and len(bin_lookup) == len(bins)
    plt.rcParams.update(
        {
            "font.size": 10,
            "svg.fonttype": "none",
            "svg.hashsalt": "case-window-rank-decomposition",
        }
    )
    cmap = plt.get_cmap("RdBu_r").copy()
    cmap.set_bad("#dddddd")

    def save(fig, name):
        p = args.output / (name + ".svg")
        fig.savefig(p, metadata={"Date": None})
        p.write_text(
            "\n".join(line.rstrip() for line in p.read_text().splitlines()) + "\n"
        )
        fig.savefig(
            args.output / (name + ".png"),
            dpi=130,
            metadata={"Software": "case-window-rank-decomposition"},
        )
        plt.close(fig)

    def selected(r):
        return (
            r["score"] == "residual_sigma"
            and r["label"] in ["observed", "modeled"]
            and r["stratum"] in MODES
        )

    rank_values = [r for r in ranks if selected(r)]
    bin_values = [r for r in bins if selected(r)]
    assert (len(rank_values), len(bin_values)) == (192, 960)
    save_rows(args.output / "ranking-values.csv", rank_values)
    save_rows(args.output / "bin-values.csv", bin_values)
    rows = [(noise, arm, mode) for noise, arm in CONDITIONS for mode in MODES]
    names = [
        f'{noise} / {"baseline" if arm == "epoch" else "curved"}\n{"Filter" if mode == "filter_single" else "Fusion"}'
        for noise, arm, mode in rows
    ]
    for partition, label in [
        *((p, "observed") for p in PARTITIONS),
        ("fresh", "modeled"),
    ]:
        fig, axes = plt.subplots(1, 2, figsize=(19, 9))
        for ax, population in zip(axes, POPULATIONS):
            rr = [
                lookup[
                    partition,
                    n,
                    a,
                    m,
                    "residual",
                    population,
                    "all",
                    "residual_sigma",
                    label,
                ]
                for n, a, m in rows
            ]
            values = np.array(
                [
                    [
                        (
                            float(r["auc" if scope == "full" else scope + "_auc"])
                            if r["auc" if scope == "full" else scope + "_auc"]
                            else np.nan
                        )
                        for scope in SCOPES
                    ]
                    for r in rr
                ]
            )
            artist = ax.imshow(
                np.ma.masked_invalid(values), cmap=cmap, vmin=0, vmax=1, aspect="auto"
            )
            ax.set_title(
                (
                    "All source families"
                    if population == "source"
                    else "Common families in the same stratum"
                ),
                pad=38,
                fontsize=12,
            )
            ax.set_xticks(
                range(5),
                [
                    "Full",
                    "Within\nseed",
                    "Within\nfamily",
                    "Within exact\nwindow",
                    "Within\ncase",
                ],
            )
            ax.xaxis.tick_top()
            ax.set_yticks(
                range(8), names if population == "source" else [""] * 8, fontsize=9
            )
            for y, r in enumerate(rr):
                for x, scope in enumerate(SCOPES):
                    value = values[y, x]
                    text = "NA" if np.isnan(value) else f"{value:.3f}"
                    if scope == "full":
                        detail = f'{r["records"]} states / {r["cases"]} cases'
                    else:
                        support_field = {
                            "same_seed": "seeds",
                            "same_family": "families",
                            "same_window": "windows",
                            "same_case": "cases",
                        }[scope]
                        share = (
                            f'{float(r[scope+"_pair_share"]):.1%}'
                            if r[scope + "_pair_share"]
                            else "NA"
                        )
                        detail = f'{r[support_field+"_with_both_classes"]} {support_field}; {share}'
                    ax.text(
                        x,
                        y,
                        text + "\n" + detail,
                        ha="center",
                        va="center",
                        fontsize=9,
                        color=(
                            "white"
                            if np.isfinite(value) and (value < 0.17 or value > 0.83)
                            else "black"
                        ),
                    )
            for y in [1.5, 3.5, 5.5]:
                ax.axhline(y, color="black", linewidth=0.6)
            fig.colorbar(
                artist, ax=ax, orientation="horizontal", fraction=0.035, pad=0.025
            )
        label_name = (
            "observed E" if label == "observed" else "modeled P (noise + anchor)"
        )
        fig.suptitle(
            f"{TITLES[partition]}: residual {label_name}, nested comparison scopes",
            fontsize=16,
            y=0.985,
        )
        fig.text(
            0.5,
            0.015,
            "Fixed residual-sigma direction; original 1/n_case weights. Annotation: supported units and fraction of full pair mass.\nCumulative scopes sum pair numerators/denominators before division. Exact windows concern saved diagnostics; units are not independent samples.\nNA = no positive-negative comparisons. Both targets and E/P are retained in the numeric exports.",
            ha="center",
            fontsize=10,
        )
        fig.tight_layout(rect=[0, 0.085, 1, 0.95], w_pad=2)
        save(fig, f"case-window-{partition}-{label}")

    colors = ["#4d91af", "#9ac5d0", "#d9b75b", "#df8d62", "#8e83ac"]
    bin_names = [
        "Same case",
        "Other case, same exact window",
        "Other window, same family",
        "Other family, same seed",
        "Other seed",
    ]
    bars = [
        (n, a, m, label, pop)
        for n, a, m in rows
        for label in ["observed", "modeled"]
        for pop in POPULATIONS
    ]
    fig, axes = plt.subplots(1, 3, figsize=(21, 14))
    for ax, partition in zip(axes, PARTITIONS):
        y_names = []
        for y, (noise, arm, mode, label, population) in enumerate(bars):
            key = (
                partition,
                noise,
                arm,
                mode,
                "residual",
                population,
                "all",
                "residual_sigma",
                label,
            )
            rr = [bin_lookup[*key, b] for b in BINS]
            y_names.append(
                f'{"W" if noise=="white" else "C"}/{"B" if arm=="epoch" else "C"} {"Filter" if mode=="filter_single" else "Fusion"} {"E" if label=="observed" else "P"} {"src" if population=="source" else "common"}'
            )
            left = 0.0
            if rr[0]["denominator_share"] == "":
                ax.barh(y, 1, color="#eeeeee", height=0.78)
                ax.text(0.5, y, "NA (no pairs)", ha="center", va="center", fontsize=8)
            else:
                for r, color in zip(rr, colors):
                    width = float(r["denominator_share"])
                    ax.barh(y, width, left=left, color=color, height=0.78)
                    if width >= 0.1:
                        ax.text(
                            left + width / 2,
                            y,
                            f"{width:.0%}",
                            ha="center",
                            va="center",
                            fontsize=8,
                        )
                    left += width
                assert abs(left - 1) < 1e-12
        ax.set_yticks(range(len(bars)), y_names, fontsize=8)
        ax.invert_yaxis()
        ax.set_xlim(0, 1)
        ax.set_xticks([0, 0.25, 0.5, 0.75, 1], ["0%", "25%", "50%", "75%", "100%"])
        ax.set_title(TITLES[partition], fontsize=13, pad=12)
        ax.set_xlabel("Share of full comparison mass")
        for y in np.arange(3.5, len(bars), 4):
            ax.axhline(y, color="black", linewidth=0.4)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in colors]
    fig.legend(
        handles,
        bin_names,
        loc="upper center",
        ncol=3,
        bbox_to_anchor=(0.5, 0.955),
        fontsize=10,
    )
    fig.suptitle(
        "Residual E/P: all five disjoint case/window/seed comparison bins",
        fontsize=16,
        y=0.99,
    )
    fig.text(
        0.5,
        0.015,
        "W/C noise = white/correlated; B/C arm = baseline/curved. src/common = source/common-family population.\nAll original case weights remain unchanged. Bin shares sum to one only with class support; no-pair groups stay explicitly NA.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=[0, 0.055, 1, 0.9], w_pad=2)
    save(fig, "case-window-comparison-mass")
    meta = dict(
        script_sha256=digest(Path(__file__)),
        analysis_sources_sha256=digest(args.results / "sources.json"),
        inputs_sha256={
            name: digest(args.results / name)
            for name in ["ranking-summary.csv", "hierarchy-bins.csv"]
        },
        outputs_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(
        f"5 figures; {len(rank_values)} ranking and {len(bin_values)} bin values",
        flush=True,
    )


if __name__ == "__main__":
    main()
