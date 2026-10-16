#!/usr/bin/env python3
"""Plot signed observed error energies before/after a fixed median cutoff."""

import argparse
from itertools import product
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_estimator_mode_components import ARMS, NOISES, TARGETS
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

TERMS = ["modeled_energy_m2", "unmodeled_energy_m2", "modeled_unmodeled_cross_m2"]
COLORS = ["#009e73", "#e69f00", "#cc79a7"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        digest(Path(__file__).with_name("analyze_estimator_mode_components.py"))
        == source["analysis"]["script_sha256"]
    )
    numeric = [
        r
        for r in read_rows(args.results / "component-summary.csv")
        if r["event_kind"] == "all"
        and r["view"] == "sim_recovery_preview"
        and r["score"] == "residual_sigma"
        and r["budget"] in ["all", "q50"]
        and r["stratum"] in ["filter_single", "fusion_three"]
    ]
    assert len(numeric) == 64
    lookup = {
        (
            r["partition"],
            r["noise"],
            r["variant"],
            r["stratum"],
            r["budget"],
            r["target"],
        ): r
        for r in numeric
    }
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "svg.fonttype": "none",
            "svg.hashsalt": "estimator-mode-components",
        }
    )
    args.output.mkdir(parents=True)
    for target in TARGETS:
        fig, axes = plt.subplots(2, 2, figsize=(15.4, 11.4))
        fig.subplots_adjust(
            left=0.19, right=0.98, top=0.91, bottom=0.19, wspace=0.58, hspace=0.40
        )
        for ni, noise in enumerate(NOISES):
            for ai, arm in enumerate(ARMS):
                ax = axes[ni, ai]
                labels = []
                for yi, (partition, stratum, budget) in enumerate(
                    product(
                        ["known", "new"],
                        ["filter_single", "fusion_three"],
                        ["all", "q50"],
                    )
                ):
                    r = lookup[partition, noise, arm, stratum, budget, target]
                    label = f'{partition.title()} {"Filter" if stratum=="filter_single" else "Fusion"} {"all" if budget=="all" else "q50"}'
                    labels.append(f'{label}   {r["retained_rows"]}/{r["records"]}')
                    if not int(r["retained_rows"]):
                        ax.text(
                            0.02,
                            yi,
                            "No retained references",
                            transform=ax.get_yaxis_transform(),
                            va="center",
                            color="#777777",
                            fontsize=9,
                        )
                        continue
                    values = [1e4 * float(r[field]) for field in TERMS]
                    left = 0.0
                    for value, color in zip(values, COLORS):
                        ax.barh(
                            yi,
                            value,
                            left=left if value >= 0 else 0,
                            height=0.62,
                            color=color,
                        )
                        if value >= 0:
                            left += value
                    observed = 1e4 * float(r["observed_energy_m2"])
                    partial = 1e4 * float(r["partial_model_energy_m2"])
                    ax.scatter(observed, yi, color="black", marker="o", s=35, zorder=4)
                    ax.scatter(
                        partial,
                        yi,
                        edgecolor="black",
                        facecolor="white",
                        marker="D",
                        s=36,
                        zorder=5,
                    )
                ax.axvline(0, color="#555555", linewidth=0.8)
                ax.set_yticks(range(8), labels, fontsize=9)
                ax.set_ylim(7.65, -0.65)
                ax.grid(axis="x", alpha=0.20)
                ax.set_xlabel("Inherited-weight mean squared 2D error (cm²)")
                ax.set_title(
                    f'{noise.title()} / {"Baseline" if arm=="epoch" else "Curved"}',
                    pad=10,
                )
        handles = [
            Patch(facecolor=c, label=l)
            for c, l in zip(
                COLORS,
                ["P: noise + anchor", "U: drift + transport", "Signed cross: 2 P·U"],
            )
        ]
        handles += [
            Line2D(
                [],
                [],
                marker="o",
                linestyle="none",
                color="black",
                label="Observed total",
            ),
            Line2D(
                [],
                [],
                marker="D",
                linestyle="none",
                markeredgecolor="black",
                markerfacecolor="white",
                label="Partial model: 2V",
            ),
        ]
        fig.legend(
            handles=handles,
            loc="lower center",
            bbox_to_anchor=(0.56, 0.095),
            ncol=3,
            frameon=False,
        )
        fig.suptitle(
            f"{target.title()} error components in Recovery / Preview",
            fontsize=16,
            y=0.98,
        )
        fig.text(
            0.04,
            0.025,
            "All: all_available; q50: the same fixed known-cohort residual-sigma median rule. Labels give retained/available references.\nPositive bars stack right; negative cross terms extend left. Observed energy is the signed sum. Panel scales may differ.\nBoth seed cohorts were previously observed. Component removal is arithmetic in the recorded frame, without a new rollout or calibrated bound.",
            fontsize=10,
        )
        stem = "estimator-mode-components-" + target
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
    save_rows(args.output / "estimator-mode-components.csv", numeric)
    provenance = dict(
        script_sha256=digest(Path(__file__)),
        input_sha256={
            n: digest(args.results / n)
            for n in ["component-summary.csv", "sources.json"]
        },
        numeric_rows=len(numeric),
        targets=TARGETS,
        filters=dict(
            view="sim_recovery_preview",
            event_kind="all",
            score="residual_sigma",
            budgets=["all", "q50"],
            strata=["filter_single", "fusion_three"],
        ),
        energy_unit="cm^2 in figures, m^2 in numeric source",
        retrospective=True,
    )
    (args.output / "estimator-mode-components-sources.json").write_text(
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
