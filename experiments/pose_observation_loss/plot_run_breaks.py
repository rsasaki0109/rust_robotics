#!/usr/bin/env python3
"""Plot all segment endings and distinguish overlapping from physical resets."""

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import shutil

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "svg.fonttype": "none",
        "svg.hashsalt": "fusion-return-run-breaks-v1",
    }
)
KINDS = (
    "nominal_only",
    "nominal_with_other",
    "other_only",
    "right_context_limit",
    "missing_context",
    "unknown_boundary",
    "branch_change",
    "recorded_release",
)
LABELS = (
    "Nominal only",
    "Nominal + other gates",
    "Other gates only",
    "Window boundary",
    "Missing next slot",
    "Unknown next gate",
    "Branch change",
    "Recorded release",
)
COLORS = (
    "#4477aa",
    "#cc6677",
    "#ee7733",
    "#aaaaaa",
    "#dddddd",
    "#aa3377",
    "#228833",
    "#66ccee",
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def write(path, rows):
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0], lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def figure(refs, ends, edges, output):
    assert len(refs) == 25 and len(ends) == 191
    counts = Counter(
        (
            r["event_id"],
            r["reference_time_s"],
            r["failure_kind"] if r["end_status"] == "gate_failure" else r["end_status"],
        )
        for r in ends
    )
    fig, axes = plt.subplots(
        2, 1, figsize=(15, 14), gridspec_kw={"height_ratios": [3, 1.2]}
    )
    ax = axes[0]
    left = [0] * len(refs)
    for kind, label, color in zip(KINDS, LABELS, COLORS):
        values = [counts[r["event_id"], r["reference_time_s"], kind] for r in refs]
        ax.barh(
            range(len(refs)), values, left=left, label=label, color=color, height=0.72
        )
        left = [a + b for a, b in zip(left, values)]
    assert sum(left) == len(ends)
    labels = [
        f"{r['case_id']} / {r['event_id']} / {float(r['reference_time_s']):.2f} s"
        + ("; raw away" if r["raw_distance_direction"] == "away" else "")
        for r in refs
    ]
    ax.set_yticks(range(len(refs)), labels, fontsize=9)
    ax.invert_yaxis()
    ax.set_xlim(0, max(left) + 1.5)
    ax.set_xticks(range(max(left) + 1))
    ax.set_xlabel("Known-pass segment endings per reference window (overlaps counted)")
    for i, n in enumerate(left):
        ax.text(n + 0.12, i, str(n), va="center", fontsize=9)
    ax.set_title("All 25 original reference windows", pad=56, fontsize=13)
    ax.legend(
        loc="lower center",
        bbox_to_anchor=(0.5, 1.01),
        ncol=4,
        frameon=False,
        fontsize=9,
    )
    ax.grid(axis="x", alpha=0.15)
    ax.set_axisbelow(True)
    ax = axes[1]
    repeated = Counter(
        r["next_failed_gates"] for r in ends if r["end_status"] == "gate_failure"
    )
    distinct = Counter(r["failed_gates"] for r in edges)
    masks = sorted(set(repeated) | set(distinct))
    for shift, data, label, color in (
        (-0.2, repeated, "Window endings (overlaps counted)", "#4477aa"),
        (0.2, distinct, "Distinct case/step resets", "#ee7733"),
    ):
        bars = ax.bar(
            [i + shift for i in range(len(masks))],
            [data[k] for k in masks],
            width=0.38,
            label=label,
            color=color,
        )
        ax.bar_label(bars, padding=3, fontsize=9)
    ax.set_xticks(
        range(len(masks)),
        [m.replace(";", " + ") for m in masks],
        rotation=20 if len(masks) > 4 else 0,
        ha="right" if len(masks) > 4 else "center",
        fontsize=9,
    )
    ax.set_ylabel("Observed reset records")
    ax.set_xlabel("Full simultaneous failed-gate mask at the following checkpoint")
    ax.set_ylim(0, max([*repeated.values(), *distinct.values(), 1]) * 1.18)
    ax.set_title(
        "Observed failures: two explicitly different counting units", fontsize=12
    )
    ax.legend(frameon=False, fontsize=9)
    ax.grid(axis="y", alpha=0.15)
    ax.set_axisbelow(True)
    fig.suptitle("Where the original known-pass segments end", fontsize=16, y=0.995)
    fig.text(
        0.5,
        0.012,
        "Window and missing-slot boundaries do not establish a physical reset. Distinct resets are descriptive, not independent trials.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=(0, 0.04, 1, 0.97), h_pad=4)
    fig.savefig(output / "run-breaks.svg", metadata={"Date": None})
    fig.savefig(
        output / "run-breaks.png",
        dpi=130,
        metadata={"Software": "pose-observation-loss-run-breaks"},
    )
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--results", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    if not __debug__ or args.output.exists():
        p.error("Use a new output directory without -O")
    source = json.loads((args.results / "sources.json").read_text())
    for n, h in source["output_sha256"].items():
        assert sha(args.results / n) == h
    refs, ends, edges = (
        read(args.results / (name + ".csv"))
        for name in ("reference-breaks", "run-ends", "reset-edges")
    )
    summary = [
        r for r in read(args.results / "break-summary.csv") if r["mode"] != "all_modes"
    ]
    returned = [
        e for e in read(args.results / "anchor-breaks.csv") if e["returned"] == "True"
    ]
    assert len(summary) == 252 and len(returned) == 137
    args.output.mkdir(parents=True)
    for name in ("run-ends", "reset-edges"):
        shutil.copyfile(
            args.results / (name + ".csv"), args.output / ("plot-" + name + ".csv")
        )
    write(args.output / "plot-values.csv", summary)
    write(args.output / "plot-return-values.csv", returned)
    figure(refs, ends, edges, args.output)
    meta = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in (
                "sources.json",
                "run-ends.csv",
                "reset-edges.csv",
                "reference-breaks.csv",
                "anchor-breaks.csv",
                "break-summary.csv",
            )
        },
        output_sha256={
            n: sha(args.output / n)
            for n in (
                "run-breaks.svg",
                "plot-run-ends.csv",
                "plot-reset-edges.csv",
                "plot-values.csv",
                "plot-return-values.csv",
            )
        },
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(
        json.dumps(
            dict(
                plotted_summary_rows=252,
                plotted_return_rows=137,
                plotted_run_ends=len(ends),
                plotted_reset_edges=len(edges),
                figures=2,
            )
        )
    )


if __name__ == "__main__":
    main()
