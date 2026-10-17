#!/usr/bin/env python3
"""Compare fixed saved-event scopes and conditional within-case pair shares."""

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
SCOPES = ["full", "same_case", "same_event", "other_event_same_case", "between_case"]
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


def share_text(value):
    if value == "":
        return "NA"
    share = float(value)
    return "<0.1%" if 0 < share < 0.001 else f"{share:.1%}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new output directory")
    args.output.mkdir(parents=True)
    ranks = read_rows(args.results / "ranking-summary.csv")
    bins = read_rows(args.results / "event-pair-bins.csv")
    lookup = {tuple(r[k] for k in KEY): r for r in ranks}
    bin_lookup = {tuple(r[k] for k in [*KEY, "bin"]): r for r in bins}
    assert len(lookup) == len(ranks) and len(bin_lookup) == len(bins)
    plt.rcParams.update(
        {
            "font.size": 10,
            "svg.fonttype": "none",
            "svg.hashsalt": "event-conditioned-rankings",
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
            metadata={"Software": "event-conditioned-rankings"},
        )
        plt.close(fig)

    def selected(r):
        return (
            r["score"] == "residual_sigma"
            and r["label"] in ["observed", "modeled"]
            and r["stratum"] in MODES
        )

    rank_values, bin_values = [r for r in ranks if selected(r)], [
        r for r in bins if selected(r)
    ]
    assert (len(rank_values), len(bin_values)) == (192, 576)
    save_rows(args.output / "ranking-values.csv", rank_values)
    save_rows(args.output / "bin-values.csv", bin_values)
    rows = [(n, a, m) for n, a in CONDITIONS for m in MODES]
    names = [
        f'{n} / {"baseline" if a=="epoch" else "curved"}\n{"Filter" if m=="filter_single" else "Fusion"}'
        for n, a, m in rows
    ]
    for partition, label in [
        *((p, "observed") for p in PARTITIONS),
        ("fresh", "modeled"),
    ]:
        fig, axes = plt.subplots(1, 2, figsize=(20, 9))
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
                            float(r["auc" if s == "full" else s + "_auc"])
                            if r["auc" if s == "full" else s + "_auc"]
                            else np.nan
                        )
                        for s in SCOPES
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
                pad=40,
                fontsize=12,
            )
            ax.set_xticks(
                range(5),
                [
                    "Full",
                    "Within\ncase",
                    "Within saved\nevent",
                    "Other event,\nsame case",
                    "Between\ncases",
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
                        share = share_text(r[scope + "_pair_share"])
                        if scope == "same_case":
                            detail = f'{r["cases_with_both_classes"]} cases; {share}'
                        elif scope == "same_event":
                            detail = f'{r["events_with_both_classes"]} events; {share}'
                        else:
                            detail = share + " of full pairs"
                    ax.text(
                        x,
                        y,
                        text + "\n" + detail,
                        ha="center",
                        va="center",
                        fontsize=8.5,
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
        title = "observed E" if label == "observed" else "modeled P (noise + anchor)"
        fig.suptitle(
            f"{TITLES[partition]}: residual {title}, saved source-event comparisons",
            fontsize=16,
            y=0.985,
        )
        fig.text(
            0.5,
            0.015,
            "Fixed residual-sigma direction; original 1/n_case weights. Annotation: class-supported counts and fraction of full comparison mass.\nWithin-event = same full case and same saved event_id; the tag is not complete estimator information ancestry or an independent stream.\nNA = no positive-negative support. Both targets, E/P and conditional within-case shares are retained in numeric exports.",
            ha="center",
            fontsize=10,
        )
        fig.tight_layout(rect=[0, 0.085, 1, 0.95], w_pad=2)
        save(fig, f"event-ranking-{partition}-{label}")

    bars = [
        (n, a, m, label, pop)
        for n, a, m in rows
        for label in ["observed", "modeled"]
        for pop in POPULATIONS
    ]
    fig, axes = plt.subplots(1, 3, figsize=(21, 14))
    colors = ["#4d91af", "#dfac62"]
    for ax, partition in zip(axes, PARTITIONS):
        labels = []
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
            rr = [bin_lookup[*key, b] for b in ["same_event", "other_event_same_case"]]
            labels.append(
                f'{"W" if noise=="white" else "C"}/{"B" if arm=="epoch" else "C"} {"Filter" if mode=="filter_single" else "Fusion"} {"E" if label=="observed" else "P"} {"src" if population=="source" else "common"}'
            )
            if rr[0]["within_case_share"] == "":
                ax.barh(y, 1, height=0.78, color="#eeeeee")
                ax.text(
                    0.5,
                    y,
                    "NA (no within-case pairs)",
                    ha="center",
                    va="center",
                    fontsize=7.5,
                )
            else:
                left = 0.0
                for r, color in zip(rr, colors):
                    width = float(r["within_case_share"])
                    ax.barh(y, width, left=left, height=0.78, color=color)
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
        ax.set_yticks(range(len(bars)), labels, fontsize=8)
        ax.invert_yaxis()
        ax.set_xlim(0, 1)
        ax.set_xticks([0, 0.25, 0.5, 0.75, 1], ["0%", "25%", "50%", "75%", "100%"])
        ax.set_title(TITLES[partition], fontsize=13, pad=12)
        ax.set_xlabel("Share of within-case comparison mass")
        for y in np.arange(3.5, len(bars), 4):
            ax.axhline(y, color="black", linewidth=0.4)
    fig.legend(
        [plt.Rectangle((0, 0), 1, 1, color=c) for c in colors],
        ["Same saved source event", "Other saved event, same case"],
        loc="upper center",
        ncol=2,
        bbox_to_anchor=(0.5, 0.95),
        fontsize=11,
    )
    fig.suptitle(
        "Residual E/P: event composition of comparisons within each case",
        fontsize=16,
        y=0.99,
    )
    fig.text(
        0.5,
        0.015,
        "W/C noise = white/correlated; B/C arm = baseline/curved. src/common = source/common-family population.\nDenominator is within-case pair mass, not full pair mass. Blank support stays NA; all original case weights remain unchanged.\nA saved event tag does not enumerate Fusion's complete sample history or imply independent information.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=[0, 0.075, 1, 0.91], w_pad=2)
    save(fig, "event-within-case-composition")
    meta = dict(
        script_sha256=digest(Path(__file__)),
        analysis_sources_sha256=digest(args.results / "sources.json"),
        inputs_sha256={
            name: digest(args.results / name)
            for name in ["ranking-summary.csv", "event-pair-bins.csv"]
        },
        outputs_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(
        f"5 figures; {len(rank_values)} rankings and {len(bin_values)} bin values",
        flush=True,
    )


if __name__ == "__main__":
    main()
