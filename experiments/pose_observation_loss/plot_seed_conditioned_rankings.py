#!/usr/bin/env python3
"""Display ranking values together with the seed comparison support they use."""

import argparse
from itertools import product
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows

PARTITIONS = ["known", "new", "fresh"]
NAMES = {"known": "Known", "new": "Previous additional", "fresh": "Latest additional"}
CONDITIONS = list(product(["white", "correlated"], ["epoch", "curved"]))
MODES = ["filter_single", "fusion_three"]
LABELS = ["observed", "modeled"]
GROUP = ["partition", "noise", "variant", "stratum", "target", "score", "label"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new output directory")
    args.output.mkdir(parents=True)
    plt.rcParams.update(
        {
            "font.size": 10,
            "svg.hashsalt": "seed-conditioned-rankings",
            "svg.fonttype": "none",
        }
    )
    ranking = read_rows(args.results / "ranking-summary.csv")
    blocks = read_rows(args.results / "seed-blocks.csv")
    sources = json.loads((args.results / "sources.json").read_text())
    lookup = {tuple(r[k] for k in GROUP): r for r in ranking}
    block_key = [*GROUP, "positive_seed", "negative_seed"]
    block_lookup = {tuple(r[k] for k in block_key): r for r in blocks}
    assert len(lookup) == len(ranking) and len(block_lookup) == len(blocks)
    rank_values, block_values = [], []

    def save(fig, name):
        p = args.output / (name + ".svg")
        fig.savefig(p, metadata={"Date": None})
        p.write_text(
            "\n".join(line.rstrip() for line in p.read_text().splitlines()) + "\n"
        )
        fig.savefig(
            args.output / (name + ".png"),
            dpi=135,
            metadata={"Software": "seed-conditioned-rankings"},
        )
        plt.close(fig)

    groups = [(p, n, a, m) for p in PARTITIONS for n, a in CONDITIONS for m in MODES]
    auc_cmap = plt.get_cmap("RdBu_r").copy()
    share_cmap = plt.get_cmap("Blues").copy()
    for cmap in [auc_cmap, share_cmap]:
        cmap.set_bad("#dddddd")
    for target in ["midpoint", "residual"]:
        fig, axes = plt.subplots(
            1, 3, figsize=(16, 16), gridspec_kw={"width_ratios": [1, 1, 1]}
        )
        matrix = {
            field: [] for field in ["auc", "same_seed_auc", "same_seed_pair_share"]
        }
        row_labels = []
        support = []
        for partition, noise, arm, mode in groups:
            rr = [
                lookup[partition, noise, arm, mode, target, "residual_sigma", label]
                for label in LABELS
            ]
            r = rr[0]
            estimator = "Filter" if mode == "filter_single" else "Fusion"
            condition = f'{noise} / {"baseline" if arm == "epoch" else "curved"}'
            row_labels.append(
                f'{NAMES[partition]}: {condition} / {estimator}\nn={r["records"]}, cases={r["cases"]}, seeds={r["seed_count"]}/3'
            )
            support.append(
                [(r["seeds_with_both_classes"], r["seed_count"]) for r in rr]
            )
            for field in matrix:
                matrix[field].append(
                    [float(r[field]) if r[field] != "" else np.nan for r in rr]
                )
            for r in rr:
                rank_values.append(
                    {
                        k: r[k]
                        for k in [
                            *GROUP,
                            "records",
                            "cases",
                            "seeds",
                            "seed_count",
                            "case_mass",
                            "outside_rows",
                            "inside_rows",
                            "outside_case_mass",
                            "inside_case_mass",
                            "auc",
                            "same_seed_auc",
                            "same_seed_pair_share",
                            "same_seed_numerator",
                            "same_seed_denominator",
                            "between_seed_auc",
                            "between_seed_pair_share",
                            "seeds_with_both_classes",
                            "pooled_minus_same_seed_auc",
                        ]
                    }
                )
        for ax, (field, title) in zip(
            axes,
            [
                ("auc", "Full AUC"),
                ("same_seed_auc", "AUC using comparisons within each seed"),
                ("same_seed_pair_share", "Within seed share of comparison mass"),
            ],
        ):
            data = np.array(matrix[field])
            artist = ax.imshow(
                np.ma.masked_invalid(data),
                aspect="auto",
                cmap=share_cmap if field.endswith("share") else auc_cmap,
                vmin=0,
                vmax=1,
            )
            ax.set_xticks([0, 1], ["E: observed", "P: noise + anchor"])
            ax.xaxis.tick_top()
            ax.set_title(title, fontsize=11, pad=30)
            ax.set_yticks(
                range(24), row_labels if field == "auc" else [""] * 24, fontsize=9
            )
            for y, values in enumerate(data):
                for x, value in enumerate(values):
                    text = (
                        "NA"
                        if np.isnan(value)
                        else (
                            f"{value:.1%}"
                            if field.endswith("share")
                            else f"{value:.3f}"
                        )
                    )
                    if field == "same_seed_auc":
                        k, represented = support[y][x]
                        text += f"\n{k}/{represented} seeds"
                    color = (
                        "white"
                        if np.isfinite(value)
                        and (
                            value > 0.77
                            if field.endswith("share")
                            else value < 0.17 or value > 0.83
                        )
                        else "black"
                    )
                    ax.text(
                        x, y, text, ha="center", va="center", fontsize=10, color=color
                    )
            for line in [7.5, 15.5]:
                ax.axhline(line, color="black", linewidth=1)
            fig.colorbar(
                artist,
                ax=ax,
                orientation="horizontal",
                fraction=0.025,
                pad=0.018,
                ticks=[0, 0.25, 0.5, 0.75, 1],
            )
        fig.suptitle(
            f"{target.title()} target: fixed residual-sigma direction, unchanged parent weights",
            fontsize=15,
            y=0.985,
        )
        fig.text(
            0.5,
            0.018,
            "NA = no positive-negative comparison support; gray cells are not zero AUC. Within seed AUC sums numerators/denominators, not seed AUCs.\nSupport annotation = seeds with both classes / represented seeds. Each cohort has three planned seed values.\nThe comparison subset changes the diagnostic question; it does not improve the estimator or provide independent validation.",
            ha="center",
            fontsize=9,
        )
        fig.tight_layout(rect=[0, 0.075, 1, 0.96], w_pad=2)
        save(fig, "seed-conditioned-" + target)

    for label in LABELS:
        fig, axes = plt.subplots(3, 4, figsize=(20, 14))
        for row_index, partition in enumerate(PARTITIONS):
            seeds = sources["schema"]["seeds"][partition]
            for col, (noise, arm) in enumerate(CONDITIONS):
                ax = axes[row_index, col]
                key = (
                    partition,
                    noise,
                    arm,
                    "fusion_three",
                    "residual",
                    "residual_sigma",
                    label,
                )
                full = lookup[key]
                matrix = []
                for positive in seeds:
                    line = []
                    for negative in seeds:
                        r = block_lookup[*key, positive, negative]
                        line.append(float(r["auc"]) if r["auc"] != "" else np.nan)
                        block_values.append(dict(r))
                    matrix.append(line)
                data = np.array(matrix)
                artist = ax.imshow(
                    np.ma.masked_invalid(data), cmap=auc_cmap, vmin=0, vmax=1
                )
                for y, positive in enumerate(seeds):
                    for x, negative in enumerate(seeds):
                        r = block_lookup[*key, positive, negative]
                        value = data[y, x]
                        text = "NA" if np.isnan(value) else f"{value:.3f}"
                        text += "\n" + (
                            f'{float(r["denominator_share"]):.1%}'
                            if r["denominator_share"] != ""
                            else "no mass"
                        )
                        ax.text(
                            x,
                            y,
                            text,
                            ha="center",
                            va="center",
                            fontsize=10,
                            color=(
                                "white"
                                if np.isfinite(value) and (value < 0.17 or value > 0.83)
                                else "black"
                            ),
                        )
                        if x == y:
                            ax.add_patch(
                                Rectangle(
                                    (x - 0.48, y - 0.48),
                                    0.96,
                                    0.96,
                                    fill=False,
                                    edgecolor="black",
                                    linewidth=1.2,
                                )
                            )
                ax.set_xticks(range(3), seeds)
                ax.set_yticks(range(3), seeds)
                ax.set_xlabel("Negative (inside) seed")
                ax.set_ylabel("Positive (outside) seed")
                ax.set_title(
                    f'{NAMES[partition]}: {noise} / {"baseline" if arm == "epoch" else "curved"}\nFusion R: n={full["records"]}, cases={full["cases"]}, seeds={full["seed_count"]}/3',
                    fontsize=10,
                )
        fig.suptitle(
            f'Fusion residual {"E (observed)" if label == "observed" else "P (noise + anchor)"}: every ordered seed comparison',
            fontsize=16,
            y=0.975,
        )
        fig.text(
            0.5,
            0.025,
            "Each cell shows its AUC and share of all positive-negative comparison mass; black borders mark the same seed.\nNA = absent class support; no mass = full AUC also undefined. All seeds and zero-mass blocks remain visible.\nScores keep high residual sigma = outside. Repeated histories and three seeds per cohort are not independent time samples.",
            ha="center",
            fontsize=10,
        )
        fig.subplots_adjust(
            left=0.055, right=0.91, top=0.89, bottom=0.12, wspace=0.35, hspace=0.55
        )
        cax = fig.add_axes([0.935, 0.23, 0.012, 0.5])
        fig.colorbar(artist, cax=cax, label="Block AUC", ticks=[0, 0.25, 0.5, 0.75, 1])
        save(fig, "seed-comparison-blocks-" + label)
    assert len(rank_values) == 96 and len(block_values) == 216
    save_rows(args.output / "ranking-values.csv", rank_values)
    save_rows(args.output / "block-values.csv", block_values)
    metadata = dict(
        script_sha256=digest(Path(__file__)),
        analysis_sources_sha256=digest(args.results / "sources.json"),
        inputs_sha256={
            n: digest(args.results / n)
            for n in ["ranking-summary.csv", "seed-blocks.csv"]
        },
        ranking_rows=len(rank_values),
        block_rows=len(block_values),
        matplotlib=matplotlib.__version__,
        outputs_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "plot-sources.json").write_text(
        json.dumps(metadata, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
