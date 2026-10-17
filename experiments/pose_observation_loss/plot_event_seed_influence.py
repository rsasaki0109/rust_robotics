#!/usr/bin/env python3
"""Show every planned seed, seed-only AUCs and leave-one-seed sensitivity."""

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
    summary = read_rows(args.results / "sensitivity-summary.csv")
    effects = read_rows(args.results / "seed-effects.csv")
    effect_lookup = {
        tuple(r[k] for k in [*KEY, "scope", "unit_kind", "unit_id"]): r for r in effects
    }
    assert len(effect_lookup) == len(effects) == 25920
    lookup = {tuple(r[k] for k in [*KEY, "scope", "unit_kind"]): r for r in summary}
    assert len(lookup) == len(summary) == 8640
    args.output.mkdir(parents=True)
    plt.rcParams.update(
        {
            "font.size": 10,
            "svg.fonttype": "none",
            "svg.hashsalt": "event-seed-influence",
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
            metadata={"Software": "event-seed-influence"},
        )
        plt.close(fig)

    selected = [
        r
        for r in summary
        if r["score"] == "residual_sigma"
        and r["label"] in ["observed", "modeled"]
        and r["stratum"] in MODES
    ]
    seed_values = [
        r
        for r in effects
        if r["score"] == "residual_sigma"
        and r["label"] in ["observed", "modeled"]
        and r["stratum"] in MODES
        and r["target"] == "residual"
        and r["scope"] == "le_20ms"
    ]
    assert (len(selected), len(seed_values)) == (768, 288)
    save_rows(args.output / "summary-values.csv", selected)
    save_rows(args.output / "seed-values.csv", seed_values)
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
                        "seed",
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
                    text = "base " + auc_text(r["baseline_auc"])
                    text += (
                        f'\ndrop {range_text(r)}; NA {r["auc_undefined_deletions"]}/3'
                    )
                    text += f'\nonly {range_text(r, "auc_seed_only")}; NA {r["auc_seed_only_undefined"]}/3'
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
            f"{TITLES[partition]}: residual {title}, seed deletion and seed-only sensitivity",
            fontsize=16,
            y=0.987,
        )
        fig.text(
            0.5,
            0.015,
            "Cell: baseline tick AUC; drop/only = leave-one-seed / seed-only [min,max] among defined results; NA = undefined results / 3 planned seeds.\nBackground = original AUC. Ranges are retrospective sensitivities, not confidence intervals or held-out prediction. Absent seeds stay included.\nOriginal 1/n_case weights and common-family eligibility stay fixed. Both targets and tick/native unequal-score concordance remain in numeric exports.",
            ha="center",
            fontsize=10,
        )
        fig.tight_layout(rect=[0, 0.085, 1, 0.95], w_pad=2)
        save(fig, f"seed-ranges-{partition}-{label}")

    bars = [
        (n, a, m, label, pop)
        for n, a, m in rows
        for label in ["observed", "modeled"]
        for pop in POPULATIONS
    ]
    seeds = {
        "known": ["269", "270", "271"],
        "new": ["274", "275", "276"],
        "fresh": ["277", "278", "279"],
    }
    fig, axes = plt.subplots(1, 3, figsize=(27, 16))
    for ax, partition in zip(axes, PARTITIONS):
        records, names, columns = (
            [],
            [],
            [(only, seed) for only in [True, False] for seed in seeds[partition]],
        )
        for n, a, m, label, pop in bars:
            records.append(
                [
                    effect_lookup[
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
                        "seed",
                        seed,
                    ]
                    for only, seed in columns
                ]
            )
            names.append(
                f'{"W" if n=="white" else "C"}/{"B" if a=="epoch" else "C"} {"Filter" if m=="filter_single" else "Fusion"} {"E" if label=="observed" else "P"} {"src" if pop=="source" else "common"}'
            )
        z = np.array(
            [
                [
                    (
                        float(r["seed_auc" if only else "auc"])
                        if r["seed_auc" if only else "auc"]
                        else np.nan
                    )
                    for r, (only, seed) in zip(rr, columns)
                ]
                for rr in records
            ]
        )
        artist = ax.imshow(
            np.ma.masked_invalid(z), cmap=cmap, vmin=0, vmax=1, aspect="auto"
        )
        ax.set_xticks(
            range(6),
            [f'{"Only" if only else "Drop"}\n{seed}' for only, seed in columns],
            fontsize=10,
        )
        ax.xaxis.tick_top()
        ax.set_yticks(range(len(bars)), names, fontsize=8)
        ax.set_title(TITLES[partition], fontsize=13, pad=42)
        ax.axvline(2.5, color="black", linewidth=1.2)
        for y, rr in enumerate(records):
            for x, (r, (only, seed)) in enumerate(zip(rr, columns)):
                value = r["seed_auc" if only else "auc"]
                share = r["removed_pair_share"]
                if share != "":
                    share = float(share) if only else 1 - float(share)
                text = auc_text(value) + "\n" + share_text(share)
                ax.text(
                    x,
                    y,
                    text,
                    ha="center",
                    va="center",
                    fontsize=7.7,
                    color=(
                        "white"
                        if np.isfinite(z[y, x]) and (z[y, x] < 0.17 or z[y, x] > 0.83)
                        else "black"
                    ),
                )
        for y in np.arange(3.5, len(bars), 4):
            ax.axhline(y, color="black", linewidth=0.4)
        fig.colorbar(artist, ax=ax, orientation="horizontal", fraction=0.02, pad=0.025)
    fig.suptitle(
        "Residual E/P at <=20 ms: all three planned seeds, retained alone or deleted",
        fontsize=17,
        y=0.99,
    )
    fig.text(
        0.5,
        0.015,
        "Cell: tick AUC and fraction of original scope comparison mass retained; absent denominators stay NA. Only = seed alone; Drop = all other seeds.\nW/C noise = white/correlated; B/C arm = baseline/curved. src/common = source/common-family population. Empty seeds remain explicit.\nOriginal 1/n_case weights and eligibility stay fixed. These are retrospective sensitivities, not held-out validation or independent physical trials.",
        ha="center",
        fontsize=11,
    )
    fig.tight_layout(rect=[0, 0.07, 1, 0.95], w_pad=2)
    save(fig, "seed-specific-near-time")
    meta = dict(
        script_sha256=digest(Path(__file__)),
        analysis_sources_sha256=digest(args.results / "sources.json"),
        inputs_sha256={
            "sensitivity-summary.csv": digest(args.results / "sensitivity-summary.csv"),
            "seed-effects.csv": digest(args.results / "seed-effects.csv"),
        },
        outputs_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(
        f"5 figures; {len(selected)} summary values and {len(seed_values)} seed effect values",
        flush=True,
    )


if __name__ == "__main__":
    main()
