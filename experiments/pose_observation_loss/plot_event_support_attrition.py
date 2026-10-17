#!/usr/bin/env python3
"""Plot fixed class-pair availability and exhaustive case-reason counts."""

import argparse
import csv
import hashlib
from itertools import product
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

ROOT = Path(__file__).resolve().parent
MODES = ("none", "random", "blackout")
STRATA = ("filter_single", "fusion_three")
LABELS = ("observed", "modeled")
POPULATIONS = ("source", "common_family")
CONDITIONS = tuple(product(("white", "correlated"), ("epoch", "curved")))
REASONS = (
    "no_parent_rp",
    "no_stratum_rp",
    "family_ineligible",
    "no_outside",
    "no_inside",
    "event_separated",
    "gap_excluded",
    "supported",
)
NAMES = {
    "filter_single": "Filter",
    "fusion_three": "Fusion",
    "observed": "E",
    "modeled": "P",
    "source": "source",
    "common_family": "common",
    "epoch": "baseline",
    "curved": "curved",
}
REASON_NAMES = (
    "No parent RP",
    "No stratum RP",
    "Family excluded",
    "No outside class",
    "No inside class",
    "Classes in separate events",
    "All pairs beyond gap",
    "Supported",
)
COLORS = (
    "#dddddd",
    "#999999",
    "#c5a451",
    "#b25047",
    "#e8a16a",
    "#8273a8",
    "#4a90b1",
    "#348565",
)
plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "svg.fonttype": "none",
        "svg.hashsalt": "event-support-attrition-v1",
    }
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def percent(value):
    if value == "":
        return "NA"
    v = float(value)
    if v == 0:
        return "0%"
    if v < 0.0001:
        return "<0.01%"
    return f"{100 * v:.2f}%"


def save(fig, output, name):
    fig.savefig(
        output / (name + ".svg"),
        metadata={"Date": None, "Creator": "event-support-attrition"},
    )
    fig.savefig(
        output / (name + ".png"),
        dpi=130,
        metadata={"Software": "event-support-attrition"},
    )
    plt.close(fig)


def retention(rows, output, partition):
    index = {
        (
            r["noise"],
            r["variant"],
            r["stratum"],
            r["label"],
            r["population"],
            r["mode"],
        ): r
        for r in rows
        if r["partition"] == partition
    }
    layout = tuple(product(STRATA, LABELS, POPULATIONS))
    fig, axes = plt.subplots(2, 2, figsize=(19, 13))
    cmap = plt.get_cmap("YlGnBu").copy()
    cmap.set_bad("#e2e2e2")
    title = {
        "known": "Known",
        "new": "Previous additional",
        "fresh": "Latest additional",
    }[partition]
    for ax, (noise, arm) in zip(axes.flat, CONDITIONS):
        data = np.full((len(layout), len(MODES)), np.nan)
        cells = []
        for y, (stratum, label, population) in enumerate(layout):
            for x, mode in enumerate(MODES):
                r = index[noise, arm, stratum, label, population, mode]
                if r["le_20ms_over_case"]:
                    data[y, x] = float(r["le_20ms_over_case"])
                text = "C {:,}  V {:,}  T {:,}\nV/C {}; T/C {}".format(
                    int(r["case_pair_count"]),
                    int(r["same_event_pair_count"]),
                    int(r["le_20ms_pair_count"]),
                    percent(r["same_event_over_case"]),
                    percent(r["le_20ms_over_case"]),
                )
                cells.append((y, x, text))
        im = ax.imshow(data, cmap=cmap, vmin=0, vmax=1, aspect="auto")
        ax.set_xticks(range(3), MODES)
        ax.xaxis.tick_top()
        ax.set_yticks(
            range(len(layout)), [" / ".join(NAMES[v] for v in row) for row in layout]
        )
        ax.tick_params(axis="both", length=0, pad=7)
        ax.set_title(noise + " / " + NAMES[arm], pad=30)
        for y, x, text in cells:
            ax.text(
                x,
                y,
                text,
                ha="center",
                va="center",
                fontsize=9,
                color="white" if data[y, x] > 0.65 else "black",
            )
        for edge in (1.5, 3.5, 5.5):
            ax.axhline(edge, color="white", linewidth=1.2)
    fig.suptitle(
        title + ": residual-target class-pair retention by original loss mode",
        fontsize=16,
        y=0.98,
    )
    fig.subplots_adjust(
        left=0.15, right=0.98, top=0.91, bottom=0.18, wspace=0.35, hspace=0.35
    )
    cax = fig.add_axes((0.28, 0.115, 0.44, 0.015))
    fig.colorbar(
        im,
        cax=cax,
        orientation="horizontal",
        label="Weighted <=20 ms / within-case opportunity mass",
    )
    fig.text(
        0.5,
        0.012,
        "C/V/T = raw positive-negative pairs within full case / same saved event / same event and <=20 ms.\nShares sum original 1/n_case squared masses before division. NA = no case opportunities; zero retention is 0%.\nE = observed, P = noise + anchor. Empty modes and populations remain explicit. Retrospective availability, no new AUC or causal effect.",
        ha="center",
        va="bottom",
        fontsize=10,
    )
    save(fig, output, "pair-retention-" + partition)


def reasons(rows, output):
    index = {
        (r["noise"], r["variant"], r["stratum"], r["population"], r["mode"]): r
        for r in rows
        if r["partition"] == "fresh" and r["label"] == "observed"
    }
    layout = [
        (noise, arm, stratum, mode)
        for noise, arm in CONDITIONS
        for stratum, mode in product(STRATA, MODES)
    ]
    fig, axes = plt.subplots(1, 2, figsize=(21, 14))
    for ax, population in zip(axes, POPULATIONS):
        left = np.zeros(len(layout))
        for why, color in zip(REASONS, COLORS):
            counts = np.asarray(
                [
                    int(
                        index[noise, arm, stratum, population, mode][
                            "le_20ms_cases_" + why
                        ]
                    )
                    for noise, arm, stratum, mode in layout
                ]
            )
            ax.barh(
                range(len(layout)),
                counts,
                left=left,
                color=color,
                height=0.83,
                edgecolor="white",
                linewidth=0.3,
            )
            for y, n in enumerate(counts):
                if n:
                    ax.text(
                        left[y] + n / 2,
                        y,
                        str(n),
                        ha="center",
                        va="center",
                        fontsize=9,
                        color=(
                            "white"
                            if why in ("no_outside", "event_separated", "supported")
                            else "black"
                        ),
                    )
            left += counts
        assert np.all(left == 27)
        ax.set_xlim(0, 27)
        ax.set_xticks((0, 9, 18, 27))
        ax.set_yticks(
            range(len(layout)),
            [
                " / ".join((noise, NAMES[arm], NAMES[stratum], mode))
                for noise, arm, stratum, mode in layout
            ],
            fontsize=9,
        )
        ax.invert_yaxis()
        ax.set_title(
            (
                "All source families"
                if population == "source"
                else "Common families fixed before class/time selection"
            ),
            pad=15,
        )
        ax.set_xlabel("Planned full cases (27 per row)")
        ax.tick_params(axis="y", length=0)
        for y in (5.5, 11.5, 17.5):
            ax.axhline(y, color="#555555", linewidth=0.8)
    fig.suptitle(
        "Latest additional: residual observed E case support at <=20 ms",
        fontsize=17,
        y=0.98,
    )
    fig.subplots_adjust(left=0.19, right=0.98, top=0.93, bottom=0.14, wspace=0.8)
    fig.legend(
        handles=[Patch(facecolor=c, label=n) for c, n in zip(COLORS, REASON_NAMES)],
        loc="lower center",
        bbox_to_anchor=(0.5, 0.055),
        ncol=4,
        frameon=False,
        fontsize=10,
    )
    fig.text(
        0.5,
        0.013,
        "Exactly one reason per case in the frozen precedence order; reasons describe saved diagnostic selection, not physical failures.\nNo parent/stratum RP does not establish an unrun experiment or zero error. Supported cases can still lose many pairs.\nClass/event/time conditions and original common-family eligibility stay fixed. No new simulation, AUC or causal comparison.",
        ha="center",
        va="bottom",
        fontsize=10,
    )
    save(fig, output, "case-reasons-fresh")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    sources = json.loads((args.results / "sources.json").read_text())
    assert (
        sha(args.results / "support-summary.csv")
        == sources["output_sha256"]["support-summary.csv"]
    )
    rows = [
        r
        for r in read(args.results / "support-summary.csv")
        if r["target"] == "residual"
        and r["stratum"] in STRATA
        and r["label"] in LABELS
        and r["mode"] in MODES
    ]
    assert len(rows) == 288
    args.output.mkdir(parents=True)
    with (args.output / "plot-values.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    for partition in ("known", "new", "fresh"):
        retention(rows, args.output, partition)
    reasons(rows, args.output)
    for path in args.output.glob("*.svg"):
        path.write_text(
            "\n".join(line.rstrip() for line in path.read_text().splitlines()) + "\n"
        )
    provenance = dict(
        script_sha256=sha(Path(__file__)),
        analysis_sources_sha256=sha(args.results / "sources.json"),
        inputs_sha256={
            "support-summary.csv": sha(args.results / "support-summary.csv")
        },
        outputs_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "plot-sources.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(json.dumps(dict(figures=4, plotted_rows=len(rows)), indent=2), flush=True)


if __name__ == "__main__":
    main()
