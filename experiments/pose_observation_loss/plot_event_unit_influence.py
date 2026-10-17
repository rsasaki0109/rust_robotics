#!/usr/bin/env python3
"""Show exhaustive case/family deletion ranges and concentration of pair support."""

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
SCOPES = ["same_event", "le_500ms", "le_100ms", "le_20ms"]
PARTITIONS = ["known", "new", "fresh"]
TITLES = dict(known="Known", new="Previous additional", fresh="Latest additional")
MODES = ["filter_single", "fusion_three"]
POPULATIONS = ["source", "common_family"]
CONDITIONS = list(product(["white", "correlated"], ["epoch", "curved"]))


def share_text(value):
    if value == "":
        return "NA"
    v = float(value)
    return "<0.1%" if 0 < v < 0.001 else f"{v:.1%}"


def auc_text(value):
    return "NA" if value == "" else f"{float(value):.4f}"


def range_text(r, metric="auc"):
    lo, hi = r[metric + "_min"], r[metric + "_max"]
    return "NA" if lo == "" else f"[{float(lo):.3f},{float(hi):.3f}]"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new output directory")
    summary = read_rows(args.results / "influence-summary.csv")
    lookup = {tuple(r[k] for k in [*KEY, "scope", "unit_kind"]): r for r in summary}
    assert len(lookup) == len(summary) == 17280
    args.output.mkdir(parents=True)
    plt.rcParams.update(
        {
            "font.size": 10,
            "svg.fonttype": "none",
            "svg.hashsalt": "event-unit-influence",
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
            metadata={"Software": "event-unit-influence"},
        )
        plt.close(fig)

    selected = [
        r
        for r in summary
        if r["score"] == "residual_sigma"
        and r["label"] in ["observed", "modeled"]
        and r["stratum"] in MODES
    ]
    concentration = [
        r for r in selected if r["target"] == "residual" and r["scope"] == "le_20ms"
    ]
    assert (len(selected), len(concentration)) == (1536, 192)
    save_rows(args.output / "summary-values.csv", selected)
    save_rows(args.output / "concentration-values.csv", concentration)
    rows = [(n, a, m) for n, a in CONDITIONS for m in MODES]
    names = [
        f'{n} / {"baseline" if a=="epoch" else "curved"}\n{"Filter" if m=="filter_single" else "Fusion"}'
        for n, a, m in rows
    ]
    for partition, label in [
        *((p, "observed") for p in PARTITIONS),
        ("fresh", "modeled"),
    ]:
        fig, axes = plt.subplots(1, 2, figsize=(21, 11))
        for ax, population in zip(axes, POPULATIONS):
            matrix = [
                [
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
                        scope,
                        "case",
                    ]
                    for scope in SCOPES
                ]
                for n, a, m in rows
            ]
            z = np.array(
                [
                    [
                        float(r["baseline_auc"]) if r["baseline_auc"] else np.nan
                        for r in rr
                    ]
                    for rr in matrix
                ]
            )
            artist = ax.imshow(
                np.ma.masked_invalid(z), cmap=cmap, vmin=0, vmax=1, aspect="auto"
            )
            ax.set_xticks(
                range(4),
                [
                    "Within event\nall times",
                    "Time gap\n<=500 ms",
                    "Time gap\n<=100 ms",
                    "Time gap\n<=20 ms",
                ],
            )
            ax.xaxis.tick_top()
            ax.set_yticks(
                range(8), names if population == "source" else [""] * 8, fontsize=9
            )
            ax.set_title(
                (
                    "All source families"
                    if population == "source"
                    else "Common families in the same stratum"
                ),
                pad=42,
                fontsize=13,
            )
            for y, rr in enumerate(matrix):
                n, a, m = rows[y]
                for x, r in enumerate(rr):
                    f = lookup[
                        partition,
                        n,
                        a,
                        m,
                        "residual",
                        population,
                        "all",
                        "residual_sigma",
                        label,
                        SCOPES[x],
                        "family",
                    ]
                    text = "base " + auc_text(r["baseline_auc"])
                    text += f'\nC {range_text(r)}; NA {r["auc_undefined_deletions"]}/{r["units"]}'
                    text += f'\nF {range_text(f)}; NA {f["auc_undefined_deletions"]}/{f["units"]}'
                    v = z[y, x]
                    ax.text(
                        x,
                        y,
                        text,
                        ha="center",
                        va="center",
                        fontsize=8.1,
                        color=(
                            "white"
                            if np.isfinite(v) and (v < 0.17 or v > 0.83)
                            else "black"
                        ),
                    )
            for y in [1.5, 3.5, 5.5]:
                ax.axhline(y, color="black", linewidth=0.6)
            fig.colorbar(
                artist, ax=ax, orientation="horizontal", fraction=0.035, pad=0.025
            )
        title = "observed E" if label == "observed" else "modeled P (noise + anchor)"
        fig.suptitle(
            f"{TITLES[partition]}: residual {title}, delete-one case/family sensitivity",
            fontsize=16,
            y=0.987,
        )
        fig.text(
            0.5,
            0.015,
            "Cell: baseline tick AUC; C/F = case/family deletion [min,max] among defined results; NA = undefined deletions / all selected units.\nBackground = original AUC. Ranges are finite-sample deletion sensitivities, not confidence intervals. A unit may contribute no class pairs.\nOriginal 1/n_case weights and common-family eligibility stay fixed. Both targets and tick/native unequal-score concordance remain in numeric exports.",
            ha="center",
            fontsize=10,
        )
        fig.tight_layout(rect=[0, 0.085, 1, 0.95], w_pad=2)
        save(fig, f"delete-one-{partition}-{label}")

    bars = [
        (n, a, m, label, pop)
        for n, a, m in rows
        for label in ["observed", "modeled"]
        for pop in POPULATIONS
    ]
    concentration_cmap = plt.get_cmap("YlOrRd").copy()
    concentration_cmap.set_bad("#dddddd")
    columns = [
        ("case", "max_removed_pair_share"),
        ("family", "max_removed_pair_share"),
        ("case", "max_removed_tick_unequal_share"),
        ("family", "max_removed_tick_unequal_share"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(21, 16))
    for ax, partition in zip(axes, PARTITIONS):
        matrix = []
        labels = []
        records = []
        for n, a, m, label, pop in bars:
            rr = [
                lookup[
                    partition,
                    n,
                    a,
                    m,
                    "residual",
                    pop,
                    "all",
                    "residual_sigma",
                    label,
                    "le_20ms",
                    kind,
                ]
                for kind, field in columns
            ]
            records.append(rr)
            matrix.append(
                [
                    float(r[field]) if r[field] else np.nan
                    for r, (kind, field) in zip(rr, columns)
                ]
            )
            labels.append(
                f'{"W" if n=="white" else "C"}/{"B" if a=="epoch" else "C"} {"Filter" if m=="filter_single" else "Fusion"} {"E" if label=="observed" else "P"} {"src" if pop=="source" else "common"}'
            )
        z = np.array(matrix)
        artist = ax.imshow(
            np.ma.masked_invalid(z),
            cmap=concentration_cmap,
            vmin=0,
            vmax=1,
            aspect="auto",
        )
        ax.set_xticks(
            range(4),
            [
                "Case\nall pairs",
                "Family\nall pairs",
                "Case\nunequal",
                "Family\nunequal",
            ],
            fontsize=9,
        )
        ax.xaxis.tick_top()
        ax.set_yticks(range(len(bars)), labels, fontsize=8)
        ax.set_title(TITLES[partition], fontsize=13, pad=42)
        for y, rr in enumerate(records):
            for x, (r, (kind, field)) in enumerate(zip(rr, columns)):
                count = r[
                    (
                        "pair_supported_units"
                        if "unequal" not in field
                        else "tick_unequal_supported_units"
                    )
                ]
                text = share_text(r[field]) + f'\n{count}/{r["units"]} units'
                ax.text(
                    x,
                    y,
                    text,
                    ha="center",
                    va="center",
                    fontsize=7.5,
                    color=(
                        "white" if np.isfinite(z[y, x]) and z[y, x] > 0.75 else "black"
                    ),
                )
        for y in np.arange(3.5, len(bars), 4):
            ax.axhline(y, color="black", linewidth=0.4)
        fig.colorbar(artist, ax=ax, orientation="horizontal", fraction=0.02, pad=0.025)
    fig.suptitle(
        "Residual E/P at <=20 ms: maximum comparison mass removed by one unit",
        fontsize=16,
        y=0.99,
    )
    fig.text(
        0.5,
        0.015,
        "Each cell shows the largest removed share and supported/all selected units. Unequal = original unequal ticks; absent denominators stay NA.\nW/C noise = white/correlated; B/C arm = baseline/curved. src/common = source/common-family population.\nCases and planner families are dependent diagnostic units. Concentration and deletion ranges are not uncertainty intervals or new physical trials.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=[0, 0.07, 1, 0.95], w_pad=2)
    save(fig, "delete-one-concentration")
    meta = dict(
        script_sha256=digest(Path(__file__)),
        analysis_sources_sha256=digest(args.results / "sources.json"),
        inputs_sha256={
            "influence-summary.csv": digest(args.results / "influence-summary.csv")
        },
        outputs_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(
        f"5 figures; {len(selected)} summary values and {len(concentration)} concentration values",
        flush=True,
    )


if __name__ == "__main__":
    main()
