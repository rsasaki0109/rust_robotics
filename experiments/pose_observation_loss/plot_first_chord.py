#!/usr/bin/env python3
"""Plot direct first-chord failure masks with all original empty branches."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "svg.fonttype": "none",
        "svg.hashsalt": "fusion-return-first-chord-v1",
    }
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


def figure(rows, out):
    fig, axes = plt.subplots(2, 3, figsize=(17, 8))
    kinds = ("boundary_only", "obstacle_only", "both", "direct_unknown")
    labels = ("Boundary only", "Obstacle only", "Both", "Unresolved")
    for j, (part, title) in enumerate(
        zip(
            ("known", "new", "fresh"),
            ("Known", "Previous additional", "Latest additional"),
        )
    ):
        selected = [
            r
            for r in rows
            if r["partition"] == part
            and r["category_selector"] == "all"
            and r["mode"] != "all_modes"
        ]
        for i, branch in enumerate(("joint", "guard")):
            ax = axes[i, j]
            counts = [
                sum(int(r[f"available_{branch}_first_{kind}_refs"]) for r in selected)
                for kind in kinds
            ]
            total = sum(counts)
            assert total == sum(
                int(r[f"available_{branch}_first_audited_refs"]) for r in selected
            )
            ax.barh(
                range(4),
                counts,
                color=("#ddaa33", "#cc6677", "#882255", "#bbbbbb"),
                height=0.58,
            )
            if total:
                for k, value in enumerate(counts):
                    ax.text(value + 0.025 * total, k, f"{value:,}", va="center")
            else:
                ax.text(
                    0.5,
                    0.5,
                    "N/A: no first-chord references",
                    ha="center",
                    va="center",
                    transform=ax.transAxes,
                )
            ax.set_yticks(range(4), labels if j == 0 else ("", "", "", ""))
            ax.set_ylim(3.7, -0.6)
            ax.set_xlim(0, max(1, total) * 1.22)
            ax.set_title(
                f"{title}: {branch}\n{total:,} available first-chord references"
            )
            ax.set_xlabel("Anchor/checkpoint references")
            ax.grid(axis="x", alpha=0.2)
            ax.set_axisbelow(True)
    fig.suptitle("Direct bounded tests of the first speed-probe chord")
    fig.text(
        0.5,
        0.025,
        "Source arithmetic with stated numerical allowances; prior source-path exclusion remains a separate result.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=(0, 0.055, 1, 0.95))
    fig.savefig(out / "first-chord.svg", metadata={"Date": None})
    fig.savefig(
        out / "first-chord.png",
        dpi=130,
        metadata={"Software": "pose-observation-loss-first-chord"},
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
    for n in ("anchor-first-chord.csv", "first-chord-summary.csv"):
        assert sha(args.results / n) == source["output_sha256"][n]
    rows = read(args.results / "first-chord-summary.csv")
    selected = [r for r in rows if r["mode"] != "all_modes"]
    returned = [
        r
        for r in read(args.results / "anchor-first-chord.csv")
        if r["returned"] == "True"
    ]
    assert len(rows) == 336 and len(selected) == 252 and len(returned) == 137
    args.output.mkdir(parents=True)
    write(args.output / "plot-values.csv", selected)
    write(args.output / "plot-return-values.csv", returned)
    figure(rows, args.output)
    meta = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in (
                "sources.json",
                "anchor-first-chord.csv",
                "first-chord-summary.csv",
            )
        },
        output_sha256={
            n: sha(args.output / n)
            for n in ("first-chord.svg", "plot-values.csv", "plot-return-values.csv")
        },
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(
        json.dumps(dict(plotted_summary_rows=252, plotted_return_rows=137, figures=2)),
        flush=True,
    )


if __name__ == "__main__":
    main()
