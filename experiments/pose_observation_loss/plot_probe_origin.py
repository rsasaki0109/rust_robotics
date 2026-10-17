#!/usr/bin/env python3
"""Plot bounded initial-test origins, keeping branch denominators separate."""

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
        "svg.hashsalt": "fusion-return-probe-origin-v1",
    }
)
ORIGINS = ("initial_unsafe", "first_chord_by_exclusion", "ambiguous_zero")


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
    labels = ("Initial test unsafe", "First chord by exclusion", "Ambiguous start")
    for j, (part, title) in enumerate(
        zip(
            ("known", "new", "fresh"),
            ("Known", "Previous additional", "Latest additional"),
        )
    ):
        rr = [
            r
            for r in rows
            if r["partition"] == part
            and r["category_selector"] == "all"
            and r["mode"] != "all_modes"
        ]
        for i, branch in enumerate(("joint", "guard")):
            ax = axes[i, j]
            counts = [
                sum(int(r[f"available_{branch}_{o}_refs"]) for r in rr) for o in ORIGINS
            ]
            total = sum(counts)
            assert total == sum(
                int(r[f"available_{branch}_cap_zero_references"])
                + int(r[f"available_{branch}_both_zero_references"])
                for r in rr
            )
            ax.barh(
                range(3), counts, color=("#cc6677", "#4477aa", "#bbbbbb"), height=0.6
            )
            if total:
                for k, value in enumerate(counts):
                    ax.text(value + 0.025 * total, k, f"{value:,}", va="center")
            else:
                ax.text(
                    0.5,
                    0.5,
                    "N/A: no cap-zero references",
                    ha="center",
                    va="center",
                    transform=ax.transAxes,
                )
            ax.set_yticks(range(3), labels if j == 0 else ("", "", ""))
            ax.set_ylim(2.7, -0.6)
            ax.set_xlim(0, max(1, total) * 1.22)
            ax.set_title(f"{title}: {branch}\n{total:,} available cap-zero references")
            ax.set_xlabel("Anchor/checkpoint references")
            ax.grid(axis="x", alpha=0.2)
            ax.set_axisbelow(True)
    fig.suptitle("Initial geometry of original zero speed-probe returns")
    fig.text(
        0.5,
        0.025,
        "First-chord origin uses a certainly safe start plus the recorded zero probe; it is not a chord-geometry replay.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=(0, 0.055, 1, 0.95))
    fig.savefig(out / "probe-origin.svg", metadata={"Date": None})
    fig.savefig(
        out / "probe-origin.png",
        dpi=130,
        metadata={"Software": "pose-observation-loss-probe-origin"},
    )
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Use a new output directory and run without -O")
    source = json.loads((args.results / "sources.json").read_text())
    for name in ("anchor-probe-origin.csv", "probe-summary.csv"):
        assert sha(args.results / name) == source["output_sha256"][name]
    rows = read(args.results / "probe-summary.csv")
    selected = [r for r in rows if r["mode"] != "all_modes"]
    returned = [
        r
        for r in read(args.results / "anchor-probe-origin.csv")
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
            for n in ("sources.json", "anchor-probe-origin.csv", "probe-summary.csv")
        },
        output_sha256={
            n: sha(args.output / n)
            for n in ("probe-origin.svg", "plot-values.csv", "plot-return-values.csv")
        },
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(
        json.dumps(dict(plotted_summary_rows=252, plotted_return_rows=137, figures=2)),
        flush=True,
    )


if __name__ == "__main__":
    main()
