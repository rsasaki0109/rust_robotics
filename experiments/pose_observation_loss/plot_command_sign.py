#!/usr/bin/env python3
"""Compare inherited interval and independently refined command-domain certainty."""

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
        "svg.hashsalt": "fusion-return-command-sign-v1",
    }
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def write(path, rows):
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def figure(rows, output):
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    for ax, partition, title in zip(
        axes,
        ("known", "new", "fresh"),
        ("Known", "Previous additional", "Latest additional"),
    ):
        rr = [
            r
            for r in rows
            if r["partition"] == partition
            and r["category_selector"] == "all"
            and r["mode"] != "all_modes"
        ]
        total = sum(int(r["available_joint_all_references"]) for r in rr)
        for y, kind in enumerate(("prior", "refined")):
            offset = 0
            counts = []
            for state, color in (
                ("pass", "#4477aa"),
                ("fail", "#cc6677"),
                ("unknown", "#bbbbbb"),
            ):
                value = sum(
                    int(r[f"available_joint_{kind}_{state}_references"]) for r in rr
                )
                counts.append(value)
                ax.barh(
                    y,
                    value,
                    left=offset,
                    height=0.55,
                    color=color,
                    edgecolor="#555555" if state == "unknown" else "none",
                    hatch="///" if state == "unknown" else None,
                    label=state.capitalize() if y == 0 else None,
                )
                offset += value
            assert offset == total
            if total:
                ax.text(
                    0,
                    y + 0.38,
                    "Pass {:,} / fail {:,} / unknown {:,}".format(*counts),
                    va="top",
                    fontsize=9,
                )
        if not total:
            ax.text(
                0.5,
                0.5,
                "N/A: no joint references",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
        ax.set_title(f"{title}\n{total:,} available joint references")
        ax.set_yticks((0, 1), ("Prior intervals", "Source domain"))
        ax.set_xlim(0, max(1, total) * 1.04)
        ax.set_ylim(1.8, -0.5)
        ax.set_xlabel("Anchor/checkpoint references")
        ax.grid(axis="x", alpha=0.2)
        ax.set_axisbelow(True)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, frameon=False)
    fig.suptitle(
        "Joint admission after first Fusion returns: source-domain nominal signs"
    )
    fig.text(
        0.5,
        0.09,
        "Same recorded decisions; sign inference uses the speed-probe lattice and command floor before timer checks.",
        ha="center",
        fontsize=9,
    )
    fig.tight_layout(rect=(0, 0.14, 1, 0.95))
    fig.savefig(output / "command-sign.svg", metadata={"Date": None})
    fig.savefig(
        output / "command-sign.png",
        dpi=140,
        metadata={"Software": "pose-observation-loss-command-sign"},
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
    for name in ("anchor-command-sign.csv", "command-summary.csv"):
        assert sha(args.results / name) == source["output_sha256"][name]
    rows = read(args.results / "command-summary.csv")
    exported = [r for r in rows if r["mode"] != "all_modes"]
    returned = [
        r
        for r in read(args.results / "anchor-command-sign.csv")
        if r["returned"] == "True"
    ]
    assert len(rows) == 336 and len(exported) == 252 and len(returned) == 137
    args.output.mkdir(parents=True)
    write(args.output / "plot-values.csv", exported)
    write(args.output / "plot-return-values.csv", returned)
    figure(rows, args.output)
    manifest = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in ("sources.json", "anchor-command-sign.csv", "command-summary.csv")
        },
        output_sha256={
            n: sha(args.output / n)
            for n in ("command-sign.svg", "plot-values.csv", "plot-return-values.csv")
        },
    )
    (args.output / "plot-sources.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    print(json.dumps(dict(summary_rows=252, returned_rows=137, figures=2)), flush=True)


if __name__ == "__main__":
    main()
