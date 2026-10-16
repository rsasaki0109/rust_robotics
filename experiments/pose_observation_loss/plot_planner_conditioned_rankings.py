#!/usr/bin/env python3
"""Plot fixed planner/population comparisons with complete numeric exports."""

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
STRATA = [
    "all",
    "recovery_single",
    "filter_single",
    "fusion_three",
    "recovery_fallback_single",
]
PLANNERS = ["A*", "JPS", "Theta*"]
SELECTIONS = ["all", *PLANNERS]
POPULATIONS = ["source", "common_family"]
GROUP = [
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
    families = read_rows(args.results / "family-support.csv")
    blocks = read_rows(args.results / "planner-blocks.csv")
    lookup = {tuple(r[k] for k in GROUP): r for r in ranks}
    assert len(lookup) == len(ranks)
    plt.rcParams.update(
        {
            "font.size": 10,
            "svg.fonttype": "none",
            "svg.hashsalt": "planner-conditioned-rankings",
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
            metadata={"Software": "planner-conditioned-rankings"},
        )
        plt.close(fig)

    rank_values = [
        r
        for r in ranks
        if r["score"] == "residual_sigma"
        and r["label"] in ["observed", "modeled"]
        and r["stratum"] in MODES
    ]
    assert len(rank_values) == 768
    save_rows(args.output / "ranking-values.csv", rank_values)
    columns = list(product(POPULATIONS, SELECTIONS))
    names = [
        "Source\n" + p if pop == "source" else "Common\n" + p for pop, p in columns
    ]
    row_groups = [(noise, arm, mode) for noise, arm in CONDITIONS for mode in MODES]
    row_names = [
        f'{n} / {"baseline" if a == "epoch" else "curved"}\n{"Filter" if m == "filter_single" else "Fusion"}'
        for n, a, m in row_groups
    ]
    for partition, label in [
        *((p, "observed") for p in PARTITIONS),
        ("fresh", "modeled"),
    ]:
        fig, axes = plt.subplots(1, 2, figsize=(20, 8.8))
        selected = [
            [
                lookup[
                    partition,
                    n,
                    a,
                    m,
                    "residual",
                    pop,
                    planner,
                    "residual_sigma",
                    label,
                ]
                for pop, planner in columns
            ]
            for n, a, m in row_groups
        ]
        for ax, field, title in zip(
            axes,
            ["auc", "same_seed_auc"],
            [
                "Full AUC (annotation: states / cases)",
                "Within-seed AUC (supported seeds; comparison share)",
            ],
        ):
            values = np.array(
                [
                    [float(r[field]) if r[field] else np.nan for r in line]
                    for line in selected
                ]
            )
            artist = ax.imshow(
                np.ma.masked_invalid(values), vmin=0, vmax=1, cmap=cmap, aspect="auto"
            )
            ax.set_title(title, pad=40)
            ax.set_xticks(range(8), names, fontsize=9)
            ax.xaxis.tick_top()
            ax.set_yticks(
                range(8), row_names if field == "auc" else [""] * 8, fontsize=9
            )
            for y, line in enumerate(selected):
                for x, r in enumerate(line):
                    value = values[y, x]
                    value_text = "NA" if np.isnan(value) else f"{value:.3f}"
                    if field == "auc":
                        detail = f'{r["records"]} / {r["cases"]}'
                    else:
                        share = (
                            f'{float(r["same_seed_pair_share"]):.0%}'
                            if r["same_seed_pair_share"]
                            else "NA"
                        )
                        detail = f'{r["seeds_with_both_classes"]}/{r["seed_count"]} seeds; {share}'
                    ax.text(
                        x,
                        y,
                        value_text + "\n" + detail,
                        ha="center",
                        va="center",
                        fontsize=8,
                        color=(
                            "white"
                            if np.isfinite(value) and (value < 0.17 or value > 0.83)
                            else "black"
                        ),
                    )
            ax.axvline(3.5, color="black", linewidth=2)
            for y in [1.5, 3.5, 5.5]:
                ax.axhline(y, color="black", linewidth=0.6)
            fig.colorbar(
                artist, ax=ax, orientation="horizontal", fraction=0.035, pad=0.025
            )
        definition = (
            "observed E" if label == "observed" else "modeled P (noise + anchor)"
        )
        fig.suptitle(
            f"{TITLES[partition]}: residual {definition}, residual-sigma ranking by planner",
            fontsize=16,
            y=0.99,
        )
        fig.text(
            0.5,
            0.015,
            "Common = every planner represented in that same stratum/family. Original 1/n_case weights remain unchanged.\nNA = no positive-negative comparisons. Matching does not align states or provide independent streams. Full numeric exports include both targets and E/P.",
            ha="center",
            fontsize=10,
        )
        fig.tight_layout(rect=[0, 0.065, 1, 0.955], w_pad=2)
        save(fig, f"planner-ranking-{partition}-{label}")

    support_values = []
    for partition, (noise, arm), mode in product(PARTITIONS, CONDITIONS, STRATA):
        ff = [
            r
            for r in families
            if (r["partition"], r["noise"], r["variant"], r["stratum"])
            == (partition, noise, arm, mode)
        ]
        matched = [r for r in ff if r["common_family"] == "True"]
        source = [r for r in ff if int(r["source_records"]) > 0]
        record = dict(
            partition=partition,
            noise=noise,
            variant=arm,
            stratum=mode,
            planned_families=len(ff),
            represented_families=len(source),
            common_families=len(matched),
            source_records=sum(int(r["source_records"]) for r in ff),
            common_records=sum(int(r["source_records"]) for r in matched),
        )
        record["common_family_share"] = len(matched) / len(source) if source else ""
        for prefix in ["astar", "jps", "theta"]:
            for scope, rr in [("source", ff), ("common", matched)]:
                record[scope + "_" + prefix + "_records"] = sum(
                    int(r[prefix + "_records"]) for r in rr
                )
                record[scope + "_" + prefix + "_case_mass"] = sum(
                    float(r[prefix + "_case_mass"]) for r in rr
                )
        support_values.append(record)
    save_rows(args.output / "support-values.csv", support_values)
    support_lookup = {
        (r["partition"], r["noise"], r["variant"], r["stratum"]): r
        for r in support_values
    }
    groups = [(p, n, a) for p in PARTITIONS for n, a in CONDITIONS]
    fig, ax = plt.subplots(figsize=(12, 10))
    values = np.array(
        [
            [support_lookup[p, n, a, m]["common_family_share"] or 0 for m in STRATA]
            for p, n, a in groups
        ],
        dtype=float,
    )
    share_cmap = plt.get_cmap("Blues")
    artist = ax.imshow(values, cmap=share_cmap, vmin=0, vmax=1, aspect="auto")
    ax.set_xticks(range(5), ["All", "Recovery", "Filter", "Fusion", "Fallback"])
    ax.xaxis.tick_top()
    ax.set_yticks(
        range(12),
        [
            f'{TITLES[p]}\n{n} / {"baseline" if a == "epoch" else "curved"}'
            for p, n, a in groups
        ],
        fontsize=9,
    )
    for y, (p, n, a) in enumerate(groups):
        for x, mode in enumerate(STRATA):
            r = support_lookup[p, n, a, mode]
            ax.text(
                x,
                y,
                f'{r["common_families"]} / {r["represented_families"]}\n{r["common_records"]} / {r["source_records"]} states',
                ha="center",
                va="center",
                fontsize=9,
                color="white" if values[y, x] > 0.7 else "black",
            )
    for y in [3.5, 7.5]:
        ax.axhline(y, color="black", linewidth=1)
    fig.colorbar(
        artist,
        ax=ax,
        orientation="horizontal",
        fraction=0.03,
        pad=0.025,
        label="Common families / represented families (0/0 shown at zero)",
    )
    fig.suptitle(
        "Planner support: common families and retained states by stratum",
        fontsize=15,
        y=0.985,
    )
    fig.text(
        0.5,
        0.015,
        "Cell: common / represented families, then retained / source states. Every cell has 27 planned families.\nEligibility uses source availability only. Counts are descriptive; matching can retain different mode fractions across planners.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=[0, 0.065, 1, 0.95])
    save(fig, "planner-common-support")

    block_values = [
        r
        for r in blocks
        if (
            r["partition"],
            r["noise"],
            r["variant"],
            r["stratum"],
            r["target"],
            r["score"],
            r["label"],
        )
        == (
            "fresh",
            "correlated",
            "curved",
            "fusion_three",
            "residual",
            "residual_sigma",
            "modeled",
        )
    ]
    assert len(block_values) == 18
    save_rows(args.output / "block-values.csv", block_values)
    fig, axes = plt.subplots(1, 2, figsize=(12, 6.5))
    for ax, population in zip(axes, POPULATIONS):
        rr = {
            (r["positive_planner"], r["negative_planner"]): r
            for r in block_values
            if r["population"] == population
        }
        values = np.array(
            [
                [
                    float(rr[p, n]["auc"]) if rr[p, n]["auc"] else np.nan
                    for n in PLANNERS
                ]
                for p in PLANNERS
            ]
        )
        artist = ax.imshow(np.ma.masked_invalid(values), vmin=0, vmax=1, cmap=cmap)
        ax.set_title(
            "All source families" if population == "source" else "Common families",
            pad=12,
        )
        ax.set_xticks(range(3), PLANNERS)
        ax.set_yticks(range(3), PLANNERS)
        ax.set_xlabel("Negative (P inside) planner")
        ax.set_ylabel("Positive (P outside) planner")
        for y, p in enumerate(PLANNERS):
            for x, n in enumerate(PLANNERS):
                r = rr[p, n]
                value = values[y, x]
                text = "NA" if np.isnan(value) else f"{value:.3f}"
                share = (
                    f'{float(r["denominator_share"]):.1%}'
                    if r["denominator_share"]
                    else "NA"
                )
                ax.text(
                    x,
                    y,
                    text + "\n" + share + " pairs",
                    ha="center",
                    va="center",
                    color=(
                        "white"
                        if np.isfinite(value) and (value < 0.17 or value > 0.83)
                        else "black"
                    ),
                )
        fig.colorbar(artist, ax=ax, orientation="horizontal", fraction=0.045, pad=0.13)
    fig.suptitle(
        "Latest correlated / curved Fusion residual P: ordered planner blocks",
        fontsize=14,
    )
    fig.text(
        0.5,
        0.015,
        "AUC and share of the population's positive-negative comparison mass. High score = outside; ties count half.\nShares weight block numerators/denominators. A high mixed value can depend on comparisons across seeds and planners.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=[0, 0.095, 1, 0.94], w_pad=2)
    save(fig, "planner-blocks-fresh-correlated-curved-fusion-P")
    meta = dict(
        script_sha256=digest(Path(__file__)),
        analysis_sources_sha256=digest(args.results / "sources.json"),
        inputs_sha256={
            name: digest(args.results / name)
            for name in [
                "ranking-summary.csv",
                "family-support.csv",
                "planner-blocks.csv",
            ]
        },
        outputs_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(
        f"6 figures, {len(rank_values)} rankings, {len(support_values)} support rows, {len(block_values)} blocks",
        flush=True,
    )


if __name__ == "__main__":
    main()
