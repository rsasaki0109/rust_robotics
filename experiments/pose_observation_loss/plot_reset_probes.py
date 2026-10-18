#!/usr/bin/env python3
"""Plot inherited probe stages and aggregate test-gap bounds at every reset."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "svg.fonttype": "none",
        "svg.hashsalt": "fusion-return-reset-probes-v1",
    }
)
ORIGINS = ("initial_unsafe", "first_chord_by_exclusion", "ambiguous_zero")
LABELS = ("Initial point unsafe", "First chord (direct check)", "Ambiguous zero")
COLORS = dict(initial="#4477aa", first_chord="#ee7733", unresolved="#999999")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def figure(probes, summary, out):
    assert len(probes) == 19
    fig, axes = plt.subplots(
        2, 1, figsize=(13, 12), gridspec_kw={"height_ratios": [1, 3]}
    )
    ax = axes[0]
    rows = {
        r["origin_selector"]: r for r in summary if r["case_selector"] == "all_cases"
    }
    for shift, field, label, color in (
        (-0.2, "physical_resets", "Distinct case/step resets", "#4477aa"),
        (0.2, "overlapping_endings", "Window endings (overlaps counted)", "#ee7733"),
    ):
        bars = ax.bar(
            [i + shift for i in range(3)],
            [int(rows[o][field]) for o in ORIGINS],
            width=0.38,
            label=label,
            color=color,
        )
        ax.bar_label(bars, padding=3)
    ax.set_xticks(range(3), LABELS)
    ax.set_ylabel("Records")
    ax.set_title("Original source-test stage: different counting units")
    ax.set_ylim(0, max(int(r["overlapping_endings"]) for r in rows.values()) * 1.18)
    ax.legend(frameon=False, fontsize=9)
    ax.grid(axis="y", alpha=0.15)
    ax.set_axisbelow(True)
    ax = axes[1]
    all_bounds = [0.0]
    for i, p in enumerate(probes):
        if p["stage"] == "unresolved":
            continue
        lo, hi = float(p["origin_gap_lo"]), float(p["origin_gap_hi"])
        all_bounds.extend((lo, hi))
        color = COLORS[p["stage"]]
        ax.hlines(i, lo, hi, color=color, linewidth=2)
        ax.plot((lo + hi) / 2, i, marker="o", markersize=4, color=color)
    ax.axvline(0, color="#555555", linestyle="--", linewidth=1)
    width = max(all_bounds) - min(all_bounds)
    pad = max(width * 0.06, 1e-6)
    ax.set_xlim(min(all_bounds) - pad, max(all_bounds) + pad)
    ax.set_yticks(
        range(19),
        [f"{p['edge_id']} / {float(p['source_time_s']):.2f} s" for p in probes],
        fontsize=9,
    )
    ax.invert_yaxis()
    ax.set_xlabel("Inherited minimum boundary/obstacle test gap (m)")
    ax.set_title("All 19 reset edges: aggregate source-test enclosures")
    ax.legend(
        handles=[
            Line2D([0], [0], color=COLORS[k], marker="o", label=label)
            for k, label in (
                ("initial", "Initial point test"),
                ("first_chord", "Direct first-chord test"),
            )
        ],
        frameon=False,
        fontsize=9,
    )
    ax.grid(axis="x", alpha=0.15)
    ax.set_axisbelow(True)
    fig.suptitle(
        "Which original probe test produced zero at the reset edges",
        fontsize=15,
        y=0.995,
    )
    fig.text(
        0.5,
        0.012,
        "Bounds and stages are inherited verified geometry. A clear start point does not establish a clear curved first chord.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=(0, 0.04, 1, 0.97), h_pad=3)
    fig.savefig(out / "reset-probes.svg", metadata={"Date": None})
    fig.savefig(
        out / "reset-probes.png",
        dpi=130,
        metadata={"Software": "pose-observation-loss-reset-probes"},
    )
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--results", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    if not __debug__ or args.output.exists():
        p.error("Use a fresh output directory without -O")
    source = json.loads((args.results / "sources.json").read_text())
    for name, h in source["output_sha256"].items():
        assert sha(args.results / name) == h
    probes, summary = [
        read(args.results / name) for name in ("reset-probes.csv", "probe-summary.csv")
    ]
    args.output.mkdir(parents=True)
    for name, target in (
        ("reset-probes.csv", "plot-reset-probes.csv"),
        ("reset-blockers.csv", "plot-reset-blockers.csv"),
        ("probe-summary.csv", "plot-values.csv"),
    ):
        shutil.copyfile(args.results / name, args.output / target)
    figure(probes, summary, args.output)
    data = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in (
                "sources.json",
                "reset-probes.csv",
                "reset-blockers.csv",
                "probe-summary.csv",
            )
        },
        output_sha256={
            n: sha(args.output / n)
            for n in (
                "reset-probes.svg",
                "plot-reset-probes.csv",
                "plot-reset-blockers.csv",
                "plot-values.csv",
            )
        },
    )
    (args.output / "plot-sources.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps(dict(plotted_resets=19, summary_cells=len(summary), figures=2)))


if __name__ == "__main__":
    main()
