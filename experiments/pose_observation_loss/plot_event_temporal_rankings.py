#!/usr/bin/env python3
"""Compare frozen time horizons within the same saved source event."""

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
SCOPES = ["same_event", "le_500ms", "le_100ms", "le_20ms"]
BINS = ["le_20ms", "20_to_100ms", "100_to_500ms", "gt_500ms"]
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
    bins = read_rows(args.results / "temporal-pair-bins.csv")
    lookup = {tuple(r[k] for k in KEY): r for r in ranks}
    bin_lookup = {tuple(r[k] for k in [*KEY, "bin"]): r for r in bins}
    assert len(lookup) == len(ranks) and len(bin_lookup) == len(bins)
    plt.rcParams.update(
        {
            "font.size": 10,
            "svg.fonttype": "none",
            "svg.hashsalt": "event-temporal-rankings",
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
            metadata={"Software": "event-temporal-rankings"},
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
    assert (len(rank_values), len(bin_values)) == (192, 768)
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
                range(4),
                [
                    "Within saved\nevent (all times)",
                    "Time gap\n<=500 ms",
                    "Time gap\n<=100 ms",
                    "Time gap\n<=20 ms",
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
                    if scope == "same_event":
                        count = r["events_with_both_classes"]
                        share = "100.0%" if r["same_event_auc"] else "NA"
                    else:
                        count = r[scope + "_events_with_pairs"]
                        share = share_text(r[scope + "_event_pair_share"])
                    detail = f'{count}/{r["events"]} events; {share}'
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
            f"{TITLES[partition]}: residual {title}, time proximity within saved events",
            fontsize=16,
            y=0.985,
        )
        fig.text(
            0.5,
            0.015,
            "Fixed residual-sigma direction; original 1/n_case weights. Annotation: supported/available events and fraction of same-event pair mass.\nAbsolute saved state-time gap; full case and saved event_id must match. The tag does not establish complete ancestry or independent information.\nNA = no positive-negative pairs in that horizon. Both targets, case support and full-denominator shares remain in numeric exports.",
            ha="center",
            fontsize=10,
        )
        fig.tight_layout(rect=[0, 0.085, 1, 0.95], w_pad=2)
        save(fig, f"event-time-ranking-{partition}-{label}")

    bars = [
        (n, a, m, label, pop)
        for n, a, m in rows
        for label in ["observed", "modeled"]
        for pop in POPULATIONS
    ]
    fig, axes = plt.subplots(1, 3, figsize=(21, 14))
    colors = ["#225ea8", "#41b6c4", "#f5cb62", "#d88744"]
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
            rr = [bin_lookup[*key, b] for b in BINS]
            labels.append(
                f'{"W" if noise=="white" else "C"}/{"B" if arm=="epoch" else "C"} {"Filter" if mode=="filter_single" else "Fusion"} {"E" if label=="observed" else "P"} {"src" if population=="source" else "common"}'
            )
            if rr[0]["event_pair_share"] == "":
                ax.barh(y, 1, height=0.78, color="#eeeeee")
                ax.text(
                    0.5,
                    y,
                    "NA (no same-event pairs)",
                    ha="center",
                    va="center",
                    fontsize=7.5,
                )
            else:
                left = 0.0
                for r, color in zip(rr, colors):
                    width = float(r["event_pair_share"])
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
        ax.set_xlabel("Share of same-event comparison mass")
        for y in np.arange(3.5, len(bars), 4):
            ax.axhline(y, color="black", linewidth=0.4)
    fig.legend(
        [plt.Rectangle((0, 0), 1, 1, color=c) for c in colors],
        ["[0,20] ms", "(20,100] ms", "(100,500] ms", ">500 ms"],
        loc="upper center",
        ncol=4,
        bbox_to_anchor=(0.5, 0.95),
        fontsize=11,
    )
    fig.suptitle(
        "Residual E/P: time-gap composition of comparisons within saved events",
        fontsize=16,
        y=0.99,
    )
    fig.text(
        0.5,
        0.015,
        "W/C noise = white/correlated; B/C arm = baseline/curved. src/common = source/common-family population.\nDenominator is same-event pair mass. Blank support stays NA; all original case weights remain unchanged.\nTime proximity is retrospective, symmetric in time, and does not imply independent information or a future forecast.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=[0, 0.075, 1, 0.91], w_pad=2)
    save(fig, "event-time-gap-composition")
    meta = dict(
        script_sha256=digest(Path(__file__)),
        analysis_sources_sha256=digest(args.results / "sources.json"),
        inputs_sha256={
            name: digest(args.results / name)
            for name in ["ranking-summary.csv", "temporal-pair-bins.csv"]
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
