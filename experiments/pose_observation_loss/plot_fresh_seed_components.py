#!/usr/bin/env python3
"""Plot prespecified component energies and fixed-direction additional-seed AUCs."""

import argparse
from itertools import product
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    import numpy as np

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.hashsalt": "fresh-seed-components",
        }
    )
    args.output.mkdir(parents=True)
    summaries = read_rows(args.results / "component-summary.csv")
    ranking = read_rows(args.results / "ranking.csv")
    keys = ["noise", "variant", "stratum", "method", "budget", "target"]
    selected = [
        r
        for r in summaries
        if r["event_kind"] == "all"
        and r["view"] == "sim_recovery_preview"
        and r["score"] == "residual_sigma"
    ]
    lookup = {tuple(r[k] for k in keys): r for r in selected}
    assert len(lookup) == len(selected)
    energies = []
    auc_values = []
    energy_fields = [
        "modeled_energy_m2",
        "unmodeled_energy_m2",
        "modeled_unmodeled_cross_m2",
        "observed_energy_m2",
        "partial_model_energy_m2",
    ]
    choices = [
        ("available", "pooled", "all"),
        ("pooled q50", "pooled", "q50"),
        ("conditional q50", "conditional", "q50"),
    ]

    def save(fig, name):
        p = args.output / (name + ".svg")
        fig.savefig(p, metadata={"Date": None})
        p.write_text(
            "\n".join(line.rstrip() for line in p.read_text().splitlines()) + "\n"
        )
        fig.savefig(
            args.output / (name + ".png"),
            dpi=150,
            metadata={"Software": "fresh-seed-components"},
        )
        plt.close(fig)

    for target in ["midpoint", "residual"]:
        panels = []
        lower = 0
        upper = 1
        for noise, arm in product(["white", "correlated"], ["epoch", "curved"]):
            rows = []
            for mode in ["filter_single", "fusion_three"]:
                for label, method, budget in choices:
                    r = lookup[noise, arm, mode, method, budget, target]
                    rows.append((mode, label, r))
                    energies.append(
                        dict(
                            noise=noise,
                            variant=arm,
                            target=target,
                            stratum=mode,
                            selection=label,
                            method=method,
                            budget=budget,
                            **{
                                k: r[k]
                                for k in [
                                    "records",
                                    "cases",
                                    "case_mass",
                                    "group_case_mass",
                                    "retained_rows",
                                    "retained_cases",
                                    "retained_case_mass",
                                    "retention_support",
                                    *energy_fields,
                                ]
                            },
                        )
                    )
                    if r["observed_energy_m2"] != "":
                        vals = [float(r[k]) * 1e4 for k in energy_fields]
                        lower = min(lower, vals[2])
                        upper = max(
                            upper, vals[0] + vals[1] + max(0, vals[2]), vals[3], vals[4]
                        )
            panels.append((noise, arm, rows))
        fig, axes = plt.subplots(2, 2, figsize=(16, 9.5))
        for (noise, arm, rows), ax in zip(panels, axes.flat):
            labels = []
            for y, (mode, label, r) in enumerate(rows):
                labels.append(
                    ("Filter" if mode == "filter_single" else "Fusion") + " / " + label
                )
                if r["observed_energy_m2"] == "":
                    ax.text(
                        0,
                        y,
                        (
                            "Unsupported"
                            if r["retention_support"] == "unsupported"
                            else "Empty"
                        ),
                        va="center",
                        fontsize=8,
                        color=".4",
                    )
                else:
                    positive = negative = 0
                    for field, color in zip(
                        energy_fields[:3], ["#4477aa", "#ee9944", "#66aa88"]
                    ):
                        value = float(r[field]) * 1e4
                        ax.barh(
                            y,
                            value,
                            left=positive if value >= 0 else negative,
                            height=0.52,
                            color=color,
                        )
                        if value >= 0:
                            positive += value
                        else:
                            negative += value
                    ax.scatter(
                        float(r["observed_energy_m2"]) * 1e4,
                        y,
                        s=26,
                        c="black",
                        zorder=4,
                    )
                    ax.scatter(
                        float(r["partial_model_energy_m2"]) * 1e4,
                        y,
                        s=40,
                        marker="D",
                        facecolors="none",
                        edgecolors="#883388",
                        linewidths=1.3,
                        zorder=5,
                    )
                mass = (
                    f"{float(r['retained_case_mass']):.2f}"
                    if r["retained_case_mass"] != ""
                    else "NA"
                )
                n = r["retained_rows"] if r["retained_rows"] != "" else "NA"
                c = r["retained_cases"] if r["retained_cases"] != "" else "NA"
                ax.text(
                    upper * 1.05,
                    y,
                    f'n={n}/{r["records"]}; c={c}/{r["cases"]}; m={mass}',
                    va="center",
                    fontsize=8,
                )
            ax.set_yticks(range(6), labels)
            ax.set_ylim(5.6, -0.6)
            ax.set_xlim(lower - 0.04 * upper, upper * 1.95)
            ax.set_xticks(np.linspace(0, upper, 4))
            ax.axvline(0, color=".4", lw=0.7)
            ax.axhline(2.5, color=".7", lw=0.7)
            ax.set_title(
                f'{noise.title()} / {"baseline" if arm=="epoch" else "curved"}'
            )
            ax.set_xlabel("Signed mean component energy (cm²)")
            ax.spines[["top", "right"]].set_visible(False)
        fig.suptitle(
            f"Additional-seed {target} components under fixed cutoffs\nRecovery / Preview; original audit; all source-event kinds",
            fontsize=13,
            y=0.98,
        )
        fig.legend(
            handles=[
                Patch(color="#4477aa", label="P = noise + anchor"),
                Patch(color="#ee9944", label="U = drift + transport"),
                Patch(color="#66aa88", label="Signed 2 P·U"),
                Line2D(
                    [],
                    [],
                    color="black",
                    marker="o",
                    linestyle="None",
                    label="Observed |E|²",
                ),
                Line2D(
                    [],
                    [],
                    color="#883388",
                    marker="D",
                    markerfacecolor="none",
                    linestyle="None",
                    label="Partial-model 2V",
                ),
            ],
            ncol=5,
            loc="lower center",
            bbox_to_anchor=(0.5, 0.06),
            frameon=False,
        )
        fig.text(
            0.5,
            0.017,
            "n = retained / source state pairs; c = retained / source cases; m = retained parent case mass.\nWeights precede mode and retention. Signed components preserve cancellation; empty rows are not zero energy. No calibration or causal inference.",
            ha="center",
            fontsize=9,
        )
        fig.tight_layout(rect=[0, 0.11, 1, 0.92], w_pad=2, h_pad=2)
        save(fig, "fresh-components-" + target)
    fig, ax = plt.subplots(figsize=(12, 6.6))
    matrix = []
    labels = []
    rk = [
        r
        for r in ranking
        if r["event_kind"] == "all"
        and r["view"] == "sim_recovery_preview"
        and r["score"] == "residual_sigma"
    ]
    for noise, arm, mode in product(
        ["white", "correlated"], ["epoch", "curved"], ["filter_single", "fusion_three"]
    ):
        row = []
        for target, label in product(
            ["midpoint", "residual"], ["observed", "modeled", "unmodeled"]
        ):
            r = next(
                r
                for r in rk
                if r["noise"] == noise
                and r["variant"] == arm
                and r["stratum"] == mode
                and r["target"] == target
            )
            value = r[label + "_case_weighted_auc"]
            row.append(float(value) if value != "" else np.nan)
            auc_values.append(
                dict(
                    noise=noise,
                    variant=arm,
                    stratum=mode,
                    target=target,
                    label=label,
                    auc=value,
                    records=r["records"],
                    cases=r["cases"],
                    case_mass=r["case_mass"],
                    outside_rows=r[label + "_outside_rows"],
                    outside_case_mass=r[label + "_outside_case_mass"],
                )
            )
        labels.append(
            f'{noise.title()} / {"baseline" if arm=="epoch" else "curved"} / {"Filter" if mode=="filter_single" else "Fusion"} (n={r["records"]}, c={r["cases"]})'
        )
        matrix.append(row)
    cmap = plt.get_cmap("RdBu_r").copy()
    cmap.set_bad("#dddddd")
    artist = ax.imshow(
        np.ma.masked_invalid(matrix), cmap=cmap, vmin=0, vmax=1, aspect="auto"
    )
    ax.set_xticks(range(6), ["M: E", "M: P", "M: U", "R: E", "R: P", "R: U"])
    ax.set_yticks(range(8), labels)
    for y, row in enumerate(matrix):
        for x, v in enumerate(row):
            ax.text(
                x,
                y,
                "NA" if np.isnan(v) else f"{v:.3f}",
                ha="center",
                va="center",
                color=(
                    "white" if not np.isnan(v) and (v < 0.15 or v > 0.85) else "black"
                ),
            )
    fig.colorbar(
        artist,
        ax=ax,
        label="AUC (high residual sigma = outside)",
        ticks=[0, 0.25, 0.5, 0.75, 1],
    )
    ax.set_title(
        "Additional-seed within-mode ranking, all_available\nE = observed; P = noise + anchor; U = drift + transport",
        pad=12,
    )
    fig.text(
        0.5,
        0.025,
        "Outside = norm >0.1 m. NA = one class absent; ties get half credit; scores are never reversed.\nCounts repeat within three seed streams. Some mode groups have only one source case.",
        ha="center",
        fontsize=9,
    )
    fig.tight_layout(rect=[0, 0.105, 1, 1])
    save(fig, "fresh-component-ranking")
    assert len(energies) == 48 and len(auc_values) == 48
    save_rows(args.output / "energy-values.csv", energies)
    save_rows(args.output / "auc-values.csv", auc_values)
    meta = dict(
        script_sha256=digest(Path(__file__)),
        analysis_sources_sha256=digest(args.results / "sources.json"),
        inputs_sha256={
            n: digest(args.results / n)
            for n in ["component-summary.csv", "ranking.csv"]
        },
        energy_rows=len(energies),
        auc_rows=len(auc_values),
        matplotlib=matplotlib.__version__,
        outputs_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")


if __name__ == "__main__":
    main()
