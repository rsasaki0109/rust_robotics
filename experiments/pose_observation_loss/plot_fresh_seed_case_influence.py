#!/usr/bin/env python3
"""Plot fixed seed/case omissions and all Fusion residual boundary cells."""

import argparse
from itertools import product
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows

CONDITIONS = list(product(["white", "correlated"], ["epoch", "curved"]))
MODES = ["filter_single", "fusion_three"]
LABELS = ["observed", "modeled"]
COLORS = {"observed": "#2878b5", "modeled": "#d16b16"}
NAMES = {"observed": "E (observed)", "modeled": "P (noise + anchor)"}


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
            "svg.hashsalt": "fresh-seed-case-influence",
            "svg.fonttype": "none",
        }
    )
    ranking = [
        r
        for r in read_rows(args.results / "ranking-influence.csv")
        if r["score"] == "residual_sigma" and r["label"] in LABELS
    ]
    components = read_rows(args.results / "component-influence.csv")
    cases = {r["case_id"]: r for r in read_rows(args.results / "cases.csv")}
    key = ["noise", "variant", "stratum", "target", "scope", "selection_id", "label"]
    lookup = {tuple(r[k] for k in key): r for r in ranking}
    assert len(lookup) == len(ranking)
    seed_values, case_values, boundary_values = [], [], []

    def save(fig, name):
        svg = args.output / (name + ".svg")
        fig.savefig(svg, metadata={"Date": None})
        svg.write_text(
            "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(
            args.output / (name + ".png"),
            dpi=130,
            metadata={"Software": "fresh-seed-case-influence"},
        )
        plt.close(fig)

    def draw_point(ax, row, y, label):
        if row["auc"] == "":
            ax.text(0.025, y, "NA", color=COLORS[label], fontsize=8, va="center")
        else:
            ax.plot(
                float(row["auc"]),
                y,
                "o" if label == "observed" else "s",
                color=COLORS[label],
                markersize=5,
            )

    def numeric(row):
        return {
            k: row[k]
            for k in [
                *key,
                "records",
                "cases",
                "seeds",
                "case_mass",
                "outside_rows",
                "inside_rows",
                "outside_case_mass",
                "inside_case_mass",
                "auc",
                "baseline_auc",
                "auc_difference",
            ]
        }

    legend = [
        Line2D(
            [],
            [],
            color=COLORS[label],
            marker="o" if label == "observed" else "s",
            linestyle="none",
            label=NAMES[label],
        )
        for label in LABELS
    ]
    seed_scopes = [
        ("all", "all"),
        *[("only_seed", seed) for seed in ["277", "278", "279"]],
        *[("omit_seed", seed) for seed in ["277", "278", "279"]],
    ]
    for target in ["midpoint", "residual"]:
        for kind in ["seeds", "cases"]:
            fig, axes = plt.subplots(2, 4, figsize=(23, 12 if kind == "cases" else 10))
            for col, (noise, arm) in enumerate(CONDITIONS):
                for row_index, mode in enumerate(MODES):
                    ax = axes[row_index, col]
                    prefix = (noise, arm, mode, target)
                    base = lookup[*prefix, "all", "all", "observed"]
                    if kind == "seeds":
                        choices = seed_scopes
                    else:
                        ids = sorted(
                            {
                                r["selection_id"]
                                for r in ranking
                                if all(r[k] == v for k, v in zip(key[:4], prefix))
                                and r["scope"] == "omit_case"
                            }
                        )
                        choices = [("omit_case", case) for case in ids]
                    ticks = []
                    for y, (scope, selected) in enumerate(choices):
                        r = lookup[*prefix, scope, selected, "observed"]
                        support = f'{r["records"]}/{r["cases"]}'
                        if kind == "seeds":
                            name = (
                                "all"
                                if scope == "all"
                                else f'{"only" if scope == "only_seed" else "omit"} {selected}'
                            )
                        else:
                            name = f'{selected} (s{cases[selected]["seed"]})'
                        ticks.append(f"{name}  [{support}]")
                        for label, offset in [("observed", -0.12), ("modeled", 0.12)]:
                            value = lookup[*prefix, scope, selected, label]
                            draw_point(ax, value, y + offset, label)
                            (seed_values if kind == "seeds" else case_values).append(
                                numeric(value)
                            )
                    for label in LABELS:
                        b = lookup[*prefix, "all", "all", label]["auc"]
                        if b != "":
                            ax.axvline(
                                float(b),
                                color=COLORS[label],
                                linewidth=0.8,
                                alpha=0.5,
                                linestyle="--",
                            )
                    ax.axvline(0.5, color="#aaaaaa", linewidth=0.6, zorder=0)
                    ax.set_xlim(-0.02, 1.02)
                    ax.set_xticks([0, 0.25, 0.5, 0.75, 1])
                    ax.set_ylim(max(len(choices), 1) - 0.5, -0.5)
                    ax.set_yticks(
                        range(len(choices)), ticks, fontsize=9 if kind == "seeds" else 8
                    )
                    if not choices:
                        ax.text(
                            0.5,
                            0.5,
                            "No represented cases",
                            transform=ax.transAxes,
                            ha="center",
                        )
                    condition = f'{noise.title()} / {"baseline" if arm == "epoch" else "curved"}'
                    estimator = "Filter" if mode == "filter_single" else "Fusion"
                    ax.set_title(
                        f'{condition} / {estimator}\nSource: {base["records"]} states, {base["cases"]} cases',
                        fontsize=10,
                    )
                    ax.set_xlabel("AUC (high residual sigma = outside)")
                    ax.grid(axis="x", alpha=0.15)
            title = (
                "Single seeds and omitted seeds"
                if kind == "seeds"
                else "Omit each represented full case"
            )
            fig.suptitle(f"{target.title()} target: {title}", fontsize=16, y=0.98)
            fig.legend(
                handles=legend,
                loc="upper center",
                bbox_to_anchor=(0.5, 0.945),
                ncol=2,
                frameon=False,
            )
            fig.text(
                0.5,
                0.018,
                "Labels show remaining states/cases. Dashed lines = full-group E/P AUC. NA = empty or one class absent.\nOriginal parent weights remain fixed. These omissions are influence diagnostics, not confidence intervals or holdouts.",
                ha="center",
                fontsize=10,
            )
            fig.tight_layout(rect=[0, 0.07, 1, 0.91], w_pad=2, h_pad=3)
            save(fig, f"fresh-{kind}-auc-{target}")

    cells = [
        "modeled_inside_observed_inside",
        "modeled_inside_observed_outside",
        "modeled_outside_observed_inside",
        "modeled_outside_observed_outside",
    ]
    cell_colors = ["#9bc6de", "#b82f36", "#e4a34a", "#42647d"]
    cell_names = ["P in / E in", "P in / E out", "P out / E in", "P out / E out"]
    fig, axes = plt.subplots(1, 4, figsize=(22, 7.5))
    for ax, (noise, arm) in zip(axes, CONDITIONS):
        subset = sorted(
            [
                r
                for r in components
                if r["noise"] == noise
                and r["variant"] == arm
                and r["stratum"] == "fusion_three"
                and r["target"] == "residual"
                and r["scope"] == "only_case"
            ],
            key=lambda r: r["selection_id"],
        )
        names = []
        for y, r in enumerate(subset):
            start = 0
            names.append(
                f'{r["selection_id"]} / s{cases[r["selection_id"]]["seed"]} / n={r["records"]}'
            )
            for cell, color in zip(cells, cell_colors):
                mass = float(r[cell + "_case_mass"])
                ax.barh(y, mass, left=start, height=0.65, color=color)
                start += mass
            ax.text(
                start + 0.025,
                y,
                f'flip={r["modeled_outside_observed_inside_rows"]}',
                va="center",
                fontsize=9,
            )
            boundary_values.append(
                {
                    k: r[k]
                    for k in [
                        "noise",
                        "variant",
                        "target",
                        "stratum",
                        "selection_id",
                        "records",
                        "case_mass",
                        *[
                            c + suffix
                            for c in cells
                            for suffix in ["_rows", "_case_mass", "_fraction"]
                        ],
                    ]
                }
            )
        ax.set_yticks(range(len(subset)), names, fontsize=9)
        ax.set_ylim(max(len(subset), 1) - 0.5, -0.5)
        ax.set_xlim(0, 1.3)
        ax.set_xticks([0, 0.25, 0.5, 0.75, 1])
        ax.set_xlabel("Inherited case mass (1 = full parent case)")
        ax.set_title(
            f'{noise.title()} / {"baseline" if arm == "epoch" else "curved"}\nFusion residual: {sum(int(r["records"]) for r in subset)} states / {len(subset)} cases'
        )
    fig.suptitle(
        "Fusion residual boundary labels in every represented case", fontsize=16, y=0.98
    )
    fig.legend(
        handles=[Patch(color=c, label=n) for c, n in zip(cell_colors, cell_names)],
        loc="upper center",
        bbox_to_anchor=(0.5, 0.93),
        ncol=4,
        frameon=False,
    )
    fig.text(
        0.5,
        0.025,
        "Each bar keeps its original parent-case mass. Flip counts = P outside but observed E inside; all four cells are retained.\nCase IDs map to full keys in cases.csv. Same-seed/planner histories are repeated observations, not independent samples.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=[0, 0.09, 1, 0.87], w_pad=2)
    save(fig, "fresh-case-boundary-cells")
    for name, values in [
        ("seed-auc-values", seed_values),
        ("case-auc-values", case_values),
        ("boundary-values", boundary_values),
    ]:
        save_rows(args.output / (name + ".csv"), values)
    meta = dict(
        script_sha256=digest(Path(__file__)),
        analysis_sources_sha256=digest(args.results / "sources.json"),
        inputs_sha256={
            n: digest(args.results / n)
            for n in ["ranking-influence.csv", "component-influence.csv", "cases.csv"]
        },
        seed_auc_rows=len(seed_values),
        case_auc_rows=len(case_values),
        boundary_rows=len(boundary_values),
        matplotlib=matplotlib.__version__,
        outputs_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")


if __name__ == "__main__":
    main()
