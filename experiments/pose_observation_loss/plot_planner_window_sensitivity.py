#!/usr/bin/env python3
"""Compare all prespecified planner/window weightings with explicit support."""

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
NAMES = {"known": "Known", "new": "Previous additional", "fresh": "Latest additional"}
CONDITIONS = list(product(["white", "correlated"], ["epoch", "curved"]))
MODES = ["filter_single", "fusion_three"]
METHODS = ["case", "window", "family"]
GROUP = [
    "partition",
    "noise",
    "variant",
    "stratum",
    "target",
    "score",
    "label",
    "weighting",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new output directory")
    args.output.mkdir(parents=True)
    plt.rcParams.update(
        {
            "font.size": 10,
            "svg.hashsalt": "planner-window-sensitivity",
            "svg.fonttype": "none",
        }
    )
    ranks = read_rows(args.results / "ranking-summary.csv")
    units = read_rows(args.results / "unit-summary.csv")
    families = read_rows(args.results / "families.csv")
    lookup = {tuple(r[k] for k in GROUP): r for r in ranks}
    unit_lookup = {
        (r["partition"], r["noise"], r["variant"], r["weighting"]): r for r in units
    }
    assert len(lookup) == len(ranks) and len(unit_lookup) == len(units)
    rank_values, unit_values = [], []

    def save(fig, name):
        p = args.output / (name + ".svg")
        fig.savefig(p, metadata={"Date": None})
        p.write_text(
            "\n".join(line.rstrip() for line in p.read_text().splitlines()) + "\n"
        )
        fig.savefig(
            args.output / (name + ".png"),
            dpi=130,
            metadata={"Software": "planner-window-sensitivity"},
        )
        plt.close(fig)

    auc_cmap = plt.get_cmap("RdBu_r").copy()
    share_cmap = plt.get_cmap("Blues").copy()
    for cmap in [auc_cmap, share_cmap]:
        cmap.set_bad("#dddddd")
    row_groups = [
        (p, n, a, m) for p in PARTITIONS for n, a in CONDITIONS for m in MODES
    ]
    for target, label in product(["midpoint", "residual"], ["observed", "modeled"]):
        fig, axes = plt.subplots(1, 3, figsize=(20, 16))
        data = {field: [] for field in ["auc", "same_seed_auc", "same_seed_pair_share"]}
        support, names = [], []
        for partition, noise, arm, mode in row_groups:
            rr = [
                lookup[
                    partition, noise, arm, mode, target, "residual_sigma", label, method
                ]
                for method in METHODS
            ]
            r = rr[0]
            counts = "/".join(row["parent_units"] for row in rr)
            names.append(
                f'{NAMES[partition]}: {noise} / {"baseline" if arm == "epoch" else "curved"} / {"Filter" if mode == "filter_single" else "Fusion"}\nn={r["records"]}, cases={r["cases"]}; parent C/W/F={counts}'
            )
            support.append(
                [(row["seeds_with_both_classes"], row["seed_count"]) for row in rr]
            )
            for field in data:
                data[field].append(
                    [float(row[field]) if row[field] != "" else np.nan for row in rr]
                )
            for row in rr:
                rank_values.append(
                    {
                        k: row[k]
                        for k in [
                            *GROUP,
                            "records",
                            "cases",
                            "seed_count",
                            "seeds",
                            "analysis_units",
                            "analysis_mass",
                            "parent_units",
                            "parent_mass",
                            "outside_rows",
                            "inside_rows",
                            "outside_analysis_mass",
                            "inside_analysis_mass",
                            "auc",
                            "same_seed_auc",
                            "same_seed_pair_share",
                            "seeds_with_both_classes",
                            "auc_difference",
                            "same_seed_auc_difference",
                        ]
                    }
                )
        for ax, (field, title) in zip(
            axes,
            [
                ("auc", "Full AUC"),
                ("same_seed_auc", "Within seed AUC"),
                ("same_seed_pair_share", "Within seed share of comparison mass"),
            ],
        ):
            matrix = np.array(data[field])
            artist = ax.imshow(
                np.ma.masked_invalid(matrix),
                cmap=share_cmap if field.endswith("share") else auc_cmap,
                vmin=0,
                vmax=1,
                aspect="auto",
            )
            ax.set_title(title, pad=32, fontsize=12)
            ax.set_xticks(range(3), ["Case", "Window", "Family"])
            ax.xaxis.tick_top()
            ax.set_yticks(range(24), names if field == "auc" else [""] * 24, fontsize=9)
            for y, line in enumerate(matrix):
                for x, value in enumerate(line):
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
        definition = "E (observed)" if label == "observed" else "P (noise + anchor)"
        fig.suptitle(
            f"{target.title()} {definition}: sensitivity to planner/window weighting",
            fontsize=16,
            y=0.985,
        )
        fig.text(
            0.5,
            0.018,
            "C/W/F = represented parent cases / exact complete saved windows / planner families. Parent weights precede mode/seed conditioning.\nWindow equality uses exact diagnostic inputs, not trajectory identity. All original rows and labels remain present. NA = absent class support.\nSupport annotation = seeds with both classes / represented seeds. Weighting units are not independent sample counts or estimator improvements.",
            ha="center",
            fontsize=10,
        )
        fig.tight_layout(rect=[0, 0.075, 1, 0.96], w_pad=2)
        save(fig, f"planner-weighting-{target}-{label}")

    fig, ax = plt.subplots(figsize=(15, 9))
    colors = ["#3578a0", "#db8d37", "#6b8b59"]
    groups = [(p, n, a) for p in PARTITIONS for n, a in CONDITIONS]
    names = []
    max_count = max(int(r["parent_units"]) for r in units)
    for y, (partition, noise, arm) in enumerate(groups):
        ff = [
            r
            for r in families
            if (r["partition"], r["noise"], r["variant"]) == (partition, noise, arm)
        ]
        shared = sum(int(r["shared_window_cases"]) for r in ff)
        extra = sum(int(r["extra_window_copies"]) for r in ff)
        multiplicity = max(int(r["max_window_multiplicity"]) for r in ff)
        planned = len(ff)
        names.append(
            f'{NAMES[partition]}: {noise} / {"baseline" if arm == "epoch" else "curved"}'
        )
        for method, offset, color in zip(METHODS, [-0.24, 0, 0.24], colors):
            r = unit_lookup[partition, noise, arm, method]
            value = int(r["parent_units"])
            ax.barh(
                y + offset,
                value,
                height=0.22,
                color=color,
                label=method.title() if y == 0 else None,
            )
            ax.text(value + 0.2, y + offset, str(value), va="center", fontsize=9)
            unit_values.append(
                dict(
                    **r,
                    planned_families=planned,
                    shared_window_cases=shared,
                    extra_window_copies=extra,
                    max_window_multiplicity=multiplicity,
                )
            )
        ax.text(
            max_count + 2,
            y,
            f'shared cases={shared}; extra copies={extra}; max copies={multiplicity}\nrepresented families={unit_lookup[partition, noise, arm, "family"]["parent_units"]}/{planned}',
            va="center",
            fontsize=9,
        )
    ax.set_yticks(range(12), names, fontsize=10)
    ax.set_ylim(11.6, -0.7)
    ax.set_xlim(0, max_count * 1.85)
    ax.set_xlabel("Represented weighting units; each is one unit of full-parent mass")
    ax.set_title(
        "Every cohort/noise/arm: cases, exact complete diagnostic windows and planner families",
        fontsize=13,
        pad=15,
    )
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, 1.015), ncol=3)
    ax.grid(axis="x", alpha=0.15)
    fig.text(
        0.5,
        0.025,
        "Exact windows must match every saved Recovery/Preview state in a case; partial overlaps and unequal values remain distinct.\nAll planned families, including zero-source families, remain in families.csv. These counts do not establish independence.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=[0, 0.075, 1, 1])
    save(fig, "planner-window-units")
    assert len(rank_values) == 288 and len(unit_values) == 36
    save_rows(args.output / "ranking-values.csv", rank_values)
    save_rows(args.output / "unit-values.csv", unit_values)
    metadata = dict(
        script_sha256=digest(Path(__file__)),
        analysis_sources_sha256=digest(args.results / "sources.json"),
        inputs_sha256={
            name: digest(args.results / name)
            for name in ["ranking-summary.csv", "unit-summary.csv", "families.csv"]
        },
        ranking_rows=len(rank_values),
        unit_rows=len(unit_values),
        matplotlib=matplotlib.__version__,
        outputs_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "plot-sources.json").write_text(
        json.dumps(metadata, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
