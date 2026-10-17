#!/usr/bin/env python3
"""Show saved-score precision, conditional concordance and explicit tie support."""

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new output directory")
    summary = read_rows(args.results / "precision-summary.csv")
    bins = read_rows(args.results / "precision-bins.csv")
    lookup = {tuple(r[k] for k in [*KEY, "scope"]): r for r in summary}
    assert len(lookup) == len(summary) == 8640
    args.output.mkdir(parents=True)
    plt.rcParams.update(
        {"font.size": 10, "svg.fonttype": "none", "svg.hashsalt": "event-score-ties"}
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
            metadata={"Software": "event-score-ties"},
        )
        plt.close(fig)

    def selected(r):
        return (
            r["score"] == "residual_sigma"
            and r["label"] in ["observed", "modeled"]
            and r["stratum"] in MODES
        )

    values = [r for r in summary if selected(r)]
    bin_values = [r for r in bins if selected(r)]
    assert (len(values), len(bin_values)) == (768, 768)
    save_rows(args.output / "summary-values.csv", values)
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
                    ]
                    for scope in SCOPES
                ]
                for n, a, m in rows
            ]
            z = np.array(
                [[float(r["auc"]) if r["auc"] else np.nan for r in rr] for rr in matrix]
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
                for x, r in enumerate(rr):
                    changed = (
                        r["native_auc_delta"] != ""
                        and float(r["native_auc_delta"]) != 0
                    )
                    line1 = (
                        auc_text(r["auc"])
                        + " / "
                        + auc_text(r["native_auc"])
                        + ("*" if changed else "")
                    )
                    line2 = (
                        "uneq "
                        + auc_text(r["tick_unequal_concordance"])
                        + f' ({r["tick_unequal_pair_count"]} pairs)'
                    )
                    line3 = f'ties {share_text(r["tick_tied_share"])}; {r["tick_unequal_events"]}/{r["events_with_pairs"]} events'
                    v = z[y, x]
                    ax.text(
                        x,
                        y,
                        line1 + "\n" + line2 + "\n" + line3,
                        ha="center",
                        va="center",
                        fontsize=8.3,
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
            f"{TITLES[partition]}: residual {title}, saved-score tie diagnosis",
            fontsize=16,
            y=0.987,
        )
        fig.text(
            0.5,
            0.015,
            "Cell: tick AUC / saved-score AUC; unequal-tick concordance and pair count; tick tie fraction and unequal/all-pair supported events.\nBackground = original tick AUC. * = saved-score AUC changes, even if displayed rounding hides it. All-tie conditional concordance stays NA.\nOriginal 1/n_case weights; same full case and saved event_id. Unequal-score concordance conditions on different pairs; it is not a replacement AUC.",
            ha="center",
            fontsize=10,
        )
        fig.tight_layout(rect=[0, 0.085, 1, 0.95], w_pad=2)
        save(fig, f"score-ties-{partition}-{label}")

    bars = [
        (n, a, m, label, pop)
        for n, a, m in rows
        for label in ["observed", "modeled"]
        for pop in POPULATIONS
    ]
    colors = ["#438aa8", "#b9b9b9", "#e6aa53"]
    fig, axes = plt.subplots(1, 3, figsize=(21, 14))
    for ax, partition in zip(axes, PARTITIONS):
        labels = []
        for y, (n, a, m, label, pop) in enumerate(bars):
            r = lookup[
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
            ]
            labels.append(
                f'{"W" if n=="white" else "C"}/{"B" if a=="epoch" else "C"} {"Filter" if m=="filter_single" else "Fusion"} {"E" if label=="observed" else "P"} {"src" if pop=="source" else "common"}'
            )
            if r["tick_tied_share"] == "":
                ax.barh(y, 1, height=0.78, color="#eeeeee")
                ax.text(
                    0.5,
                    y,
                    "NA (no same-event class pairs)",
                    ha="center",
                    va="center",
                    fontsize=7.5,
                )
            else:
                shares = [
                    1 - float(r["tick_tied_share"]),
                    float(r["native_tied_share"]),
                    float(r["resolved_tick_tie_share"]),
                ]
                left = 0.0
                for width, color in zip(shares, colors):
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
        ax.set_xlabel("Share of <=20 ms same-event comparison mass")
        for y in np.arange(3.5, len(bars), 4):
            ax.axhline(y, color="black", linewidth=0.4)
    fig.legend(
        [plt.Rectangle((0, 0), 1, 1, color=c) for c in colors],
        [
            "Unequal original ticks",
            "Equal saved score",
            "Rounded tie, unequal saved scores",
        ],
        loc="upper center",
        ncol=3,
        bbox_to_anchor=(0.5, 0.95),
        fontsize=11,
    )
    fig.suptitle("Residual E/P: why nearby comparisons tie", fontsize=16, y=0.99)
    fig.text(
        0.5,
        0.015,
        "W/C noise = white/correlated; B/C arm = baseline/curved. src/common = source/common-family population.\nOriginal ticks use 1e-8 m; saved-score equality uses the existing export, without a new tolerance. NA preserves absent class support.\nRepeated states/events/planners are dependent. This is a retrospective precision diagnostic, not a calibrated uncertainty measure.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=[0, 0.075, 1, 0.91], w_pad=2)
    save(fig, "nearby-tie-composition")
    meta = dict(
        script_sha256=digest(Path(__file__)),
        analysis_sources_sha256=digest(args.results / "sources.json"),
        inputs_sha256={
            name: digest(args.results / name)
            for name in ["precision-summary.csv", "precision-bins.csv"]
        },
        outputs_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(
        f"5 figures; {len(values)} scope values and {len(bin_values)} bin values",
        flush=True,
    )


if __name__ == "__main__":
    main()
