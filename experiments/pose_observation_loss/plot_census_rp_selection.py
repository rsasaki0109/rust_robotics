#!/usr/bin/env python3
"""Plot exhaustive planned-case, census-event and RP-window coverage."""

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

MODES = ("none", "random", "blackout")
PARTS = ("known", "new", "fresh")
CONDITIONS = tuple(product(("white", "correlated"), ("epoch", "curved")))
ACTUAL = ("raw_fallback", "recovery_mean")
PREVIEW = ("motion_filter", "three_epoch")
STRATA = (
    "recovery_single",
    "filter_single",
    "fusion_three",
    "recovery_fallback_single",
)
NAMES = {
    "known": "Known",
    "new": "Previous additional",
    "fresh": "Latest additional",
    "epoch": "baseline",
    "curved": "curved",
    "motion_filter": "Filter",
    "three_epoch": "Fusion",
    "raw_fallback": "raw",
    "recovery_mean": "mean",
}
plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "svg.fonttype": "none",
        "svg.hashsalt": "census-rp-selection-v1",
    }
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def focus(r):
    return r["mode"] in MODES and (
        (
            r["event_group"] == "all"
            and r["actual_selector"] == "all"
            and r["preview_selector"] in ("all", *PREVIEW)
        )
        or (
            r["event_group"] == "received_any"
            and r["actual_selector"] in ACTUAL
            and r["preview_selector"] in PREVIEW
        )
    )


def pct(value):
    if value == "":
        return "NA"
    v = float(value)
    if v == 0:
        return "0%"
    return "<0.01%" if v < 0.0001 else f"{100*v:.2f}%"


def save(fig, output, name):
    fig.savefig(
        output / (name + ".svg"),
        metadata={"Date": None, "Creator": "census-rp-selection"},
    )
    fig.savefig(
        output / (name + ".png"), dpi=130, metadata={"Software": "census-rp-selection"}
    )
    plt.close(fig)


def cases(rows, output):
    lookup = {
        (r["partition"], r["noise"], r["variant"], r["mode"]): r
        for r in rows
        if r["event_group"] == "all"
        and r["actual_selector"] == "all"
        and r["preview_selector"] == "all"
    }
    layout = [(noise, arm, mode) for noise, arm in CONDITIONS for mode in MODES]
    labels = [" / ".join((noise, NAMES[arm], mode)) for noise, arm, mode in layout]
    fig, axes = plt.subplots(1, 3, figsize=(23, 10))
    fields = ("held_cases", "timeout_no_rp_cases", "timeout_with_rp_cases")
    colors = ("#cccccc", "#ddad6f", "#32866b")
    for ax, part in zip(axes, PARTS):
        left = np.zeros(len(layout))
        for field, color in zip(fields, colors):
            values = np.asarray(
                [
                    int(lookup[part, noise, arm, mode][field])
                    for noise, arm, mode in layout
                ]
            )
            ax.barh(
                range(len(layout)),
                values,
                left=left,
                color=color,
                height=0.8,
                edgecolor="white",
                linewidth=0.4,
            )
            for y, n in enumerate(values):
                if n:
                    ax.text(
                        left[y] + n / 2,
                        y,
                        str(n),
                        ha="center",
                        va="center",
                        fontsize=10,
                        color="white" if field == "timeout_with_rp_cases" else "black",
                    )
            left += values
        assert np.all(left == 27)
        ax.set_yticks(range(len(layout)), labels, fontsize=10)
        ax.set_xticks((0, 9, 18, 27))
        ax.set_xlim(0, 27)
        ax.invert_yaxis()
        ax.set_title(NAMES[part], fontsize=14)
        ax.set_xlabel("Planned full cases")
        ax.tick_params(axis="y", length=0)
        for y in (2.5, 5.5, 8.5):
            ax.axhline(y, color="#555555", linewidth=0.7)
    fig.suptitle(
        "Planned cases and the fixed late-timeout RP diagnostic selection",
        fontsize=17,
        y=0.98,
    )
    fig.subplots_adjust(left=0.12, right=0.98, top=0.92, bottom=0.17, wspace=0.65)
    fig.legend(
        handles=[
            Patch(facecolor=c, label=n)
            for c, n in zip(
                colors,
                (
                    "Held: outside late-timeout census",
                    "Timeout without RP records",
                    "Timeout with RP records",
                ),
            )
        ],
        loc="lower center",
        bbox_to_anchor=(0.5, 0.085),
        ncol=3,
        frameon=False,
    )
    fig.text(
        0.5,
        0.02,
        "Each row retains all 27 planned full cases. Original recorded outcomes are unchanged.\nNo selected RP diagnostics does not mean an unrun case, zero error or failed sensor data. Retrospective coverage, no new controller comparison.",
        ha="center",
        va="bottom",
    )
    save(fig, output, "case-selection")


def matrices(rows, output, crosswalk=False):
    if crosswalk:
        selected = [
            r
            for r in rows
            if r["event_group"] == "all"
            and r["actual_selector"] == "all"
            and r["preview_selector"] in PREVIEW
        ]
        keys = ("partition", "noise", "variant", "mode", "preview_selector")
        layout = [
            (noise, arm, mode, preview)
            for noise, arm in CONDITIONS
            for mode, preview in product(MODES, PREVIEW)
        ]
        labels = [
            " / ".join((noise, NAMES[arm], mode, NAMES[preview] + "@event"))
            for noise, arm, mode, preview in layout
        ]
        columns = ("Recovery", "Filter", "Fusion", "Fallback")
        size = (24, 16)
    else:
        selected = [
            r
            for r in rows
            if r["event_group"] == "received_any"
            and r["actual_selector"] in ACTUAL
            and r["preview_selector"] in PREVIEW
        ]
        keys = (
            "partition",
            "noise",
            "variant",
            "mode",
            "actual_selector",
            "preview_selector",
        )
        layout = [(noise, arm, mode) for noise, arm in CONDITIONS for mode in MODES]
        labels = [" / ".join((noise, NAMES[arm], mode)) for noise, arm, mode in layout]
        columns = [NAMES[a] + " / " + NAMES[p] for a, p in product(ACTUAL, PREVIEW)]
        size = (23, 11)
    lookup = {tuple(r[k] for k in keys): r for r in selected}
    fig, axes = plt.subplots(1, 3, figsize=size)
    cmap = plt.get_cmap("YlGnBu").copy()
    cmap.set_bad("#dddddd")
    for ax, part in zip(axes, PARTS):
        data = np.full((len(layout), 4), np.nan)
        annotations = []
        for y, entry in enumerate(layout):
            for x in range(4):
                if crosswalk:
                    r = lookup[(part, *entry)]
                    share = r["rp_" + STRATA[x] + "_mass_share"]
                    count = int(r["rp_" + STRATA[x] + "_records"])
                    text = f"{count:,}\n{pct(share)}"
                else:
                    actual, preview = tuple(product(ACTUAL, PREVIEW))[x]
                    r = lookup[(part, *entry, actual, preview)]
                    share = r["received_census_weighted_coverage"]
                    text = f"{int(r['rp_referenced_events'])}/{int(r['census_events'])}\n{pct(share)}"
                if share != "":
                    data[y, x] = float(share)
                annotations.append((y, x, text))
        im = ax.imshow(data, vmin=0, vmax=1, cmap=cmap, aspect="auto")
        ax.set_xticks(range(4), columns, fontsize=9)
        ax.xaxis.tick_top()
        ax.set_yticks(range(len(layout)), labels, fontsize=8.5 if crosswalk else 9.5)
        ax.tick_params(axis="both", length=0, pad=5)
        ax.set_title(NAMES[part], fontsize=14, pad=30)
        for y, x, text in annotations:
            ax.text(
                x,
                y,
                text,
                ha="center",
                va="center",
                fontsize=8.5 if crosswalk else 9,
                color="white" if data[y, x] > 0.65 else "black",
            )
        for y in ((5.5, 11.5, 17.5) if crosswalk else (2.5, 5.5, 8.5)):
            ax.axhline(y, color="white", linewidth=1)
    if crosswalk:
        title = "Immediate event preview identifier versus selected RP-window stratum"
        color_label = (
            "Original RP case mass share within the displayed immediate-preview group"
        )
        footer = "Cells: raw RP state count and stratum fraction of original RP mass. Columns are window-state strata; rows name the event's immediate preview.\nWeights remain 1/n_RP_parent_case before every subset. Different identifiers do not establish a temporal transition or causal mode effect.\nAll original loss modes and empty categories remain explicit; NA = no selected RP mass. Event tags are not complete Fusion ancestry."
    else:
        title = "Received census events referenced by RP diagnostics, by immediate input identifiers"
        color_label = "Received-parent-case-weighted event reference coverage"
        footer = "Cells: RP-referenced / received census event count; coverage sums original 1/n_received_events_case weights before division.\nColumns name immediate actual / preview input. Parent weights precede all identifier/loss-mode selection. Raw counts and weighted coverage differ.\nNA = no events in this cell; 0% = observed events but none referenced. No new AUC, rollout or causal selection claim."
    fig.suptitle(title, fontsize=17, y=0.985)
    fig.subplots_adjust(
        left=0.14 if crosswalk else 0.12,
        right=0.985,
        top=0.92 if crosswalk else 0.91,
        bottom=0.16 if crosswalk else 0.19,
        wspace=0.8 if crosswalk else 0.6,
    )
    cax = fig.add_axes((0.28, 0.105 if crosswalk else 0.12, 0.44, 0.014))
    fig.colorbar(im, cax=cax, orientation="horizontal", label=color_label)
    fig.text(0.5, 0.012, footer, ha="center", va="bottom", fontsize=10)
    save(
        fig,
        output,
        "preview-mode-crosswalk" if crosswalk else "event-reference-coverage",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    sources = json.loads((args.results / "sources.json").read_text())
    assert (
        sha(args.results / "selection-summary.csv")
        == sources["output_sha256"]["selection-summary.csv"]
    )
    rows = [r for r in read(args.results / "selection-summary.csv") if focus(r)]
    assert len(rows) == 252
    args.output.mkdir(parents=True)
    with (args.output / "plot-values.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    cases(rows, args.output)
    matrices(rows, args.output)
    matrices(rows, args.output, crosswalk=True)
    for p in args.output.glob("*.svg"):
        p.write_text(
            "\n".join(line.rstrip() for line in p.read_text().splitlines()) + "\n"
        )
    meta = dict(
        script_sha256=sha(Path(__file__)),
        analysis_sources_sha256=sha(args.results / "sources.json"),
        inputs_sha256={
            "selection-summary.csv": sha(args.results / "selection-summary.csv")
        },
        outputs_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(json.dumps(dict(figures=3, plotted_rows=len(rows)), indent=2), flush=True)


if __name__ == "__main__":
    main()
