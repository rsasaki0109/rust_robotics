#!/usr/bin/env python3
"""Plot simultaneous admission failures and rounded-input unknowns."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

PARTS = ("known", "new", "fresh")
MODES = ("none", "random", "blackout")
NAMES = {"known": "Known", "new": "Previous additional", "fresh": "Latest additional"}
GATES = {
    "joint": (
        "age",
        "stationary",
        "samples",
        "clearance",
        "bounds",
        "nominal",
        "release",
    ),
    "guard": ("stationary", "release"),
}
LABELS = {
    "age": "Observation age <= 1.5 s",
    "stationary": "Stationary measured speed",
    "samples": "At least 10 recovery samples",
    "clearance": "Recovery-line clearance",
    "bounds": "Recovery-line map bounds",
    "nominal": "Positive nominal speed",
    "release": "Release-preview clearance",
}
plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "svg.fonttype": "none",
        "svg.hashsalt": "fusion-return-admission-v1",
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
    selected = [
        r for r in rows if r["category_selector"] == "all" and r["mode"] in MODES
    ]
    fig, axes = plt.subplots(
        2, 3, figsize=(18, 10), sharey="row", gridspec_kw={"height_ratios": [3.5, 1.8]}
    )
    for j, part in enumerate(PARTS):
        rr = [r for r in selected if r["partition"] == part]
        for i, (branch, gates) in enumerate(GATES.items()):
            ax = axes[i, j]
            denom = sum(int(r["available_" + branch + "_references"]) for r in rr)
            fail = np.array(
                [
                    sum(int(r[f"available_{branch}_{g}_fail_references"]) for r in rr)
                    for g in gates
                ]
            )
            unknown = np.array(
                [
                    sum(
                        int(r[f"available_{branch}_{g}_unknown_references"]) for r in rr
                    )
                    for g in gates
                ]
            )
            assert np.all(fail + unknown <= denom)
            y = np.arange(len(gates))
            ax.barh(y, fail, color="#cc6677", height=0.62, zorder=2)
            ax.barh(
                y,
                unknown,
                left=fail,
                color="#bbbbbb",
                edgecolor="#555555",
                hatch="///",
                height=0.62,
                zorder=2,
            )
            if denom:
                for k, (f, u) in enumerate(zip(fail, unknown)):
                    ax.text(
                        f + u + denom * 0.025,
                        k,
                        f"{f:,} + {u:,}?",
                        va="center",
                        fontsize=9,
                    )
            else:
                ax.text(
                    0.5,
                    0.5,
                    "N/A: no checkpoints in this branch",
                    transform=ax.transAxes,
                    ha="center",
                    fontsize=9,
                )
            ax.set_xlim(0, max(1, denom) * 1.38)
            ax.set_ylim(len(gates) - 0.35, -0.65)
            ax.set_yticks(y)
            if j == 0:
                ax.set_yticklabels([LABELS[g] for g in gates])
            else:
                ax.tick_params(labelleft=False)
            ax.set_title(
                f"{NAMES[part]} / {branch}\n{denom:,} available checkpoint references",
                fontweight="bold",
                fontsize=11,
                pad=10,
            )
            ax.grid(axis="x", alpha=0.2, zorder=0)
            ax.spines[["right", "top"]].set_visible(False)
            ax.set_xlabel("Condition failure + unknown references")
    fig.suptitle(
        "Original admission conditions while returned Fusion is available",
        fontsize=16,
        y=0.98,
    )
    fig.legend(
        handles=[
            Patch(facecolor="#cc6677", label="Certainly failed"),
            Patch(
                facecolor="#bbbbbb",
                edgecolor="#555555",
                hatch="///",
                label="Unknown from rounded input (?)",
            ),
        ],
        loc="upper center",
        bbox_to_anchor=(0.58, 0.94),
        ncol=2,
        frameon=False,
    )
    fig.text(
        0.52,
        0.025,
        "All 137 fixed first returns. Several conditions can fail on one checkpoint. Branches have different required gates and timers; counts are references, not independent trials.",
        ha="center",
        fontsize=9.5,
    )
    fig.subplots_adjust(
        left=0.18, right=0.98, top=0.83, bottom=0.10, hspace=0.65, wspace=0.22
    )
    fig.savefig(
        out / "admission-conditions.svg",
        metadata={"Date": None, "Creator": "fusion-return-admission"},
    )
    fig.savefig(
        out / "admission-conditions.png",
        dpi=125,
        metadata={"Software": "fusion-return-admission"},
    )
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    rows = read(args.results / "admission-summary.csv")
    anchors = read(args.results / "anchor-admission.csv")
    selected = [r for r in rows if r["mode"] in MODES]
    returned = [r for r in anchors if r["returned"] == "True"]
    assert len(rows) == 336 and len(selected) == 252 and len(returned) == 137
    args.output.mkdir(parents=True)
    write(args.output / "plot-values.csv", selected)
    write(args.output / "plot-return-values.csv", returned)
    figure(rows, args.output)
    source = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in ("sources.json", "anchor-admission.csv", "admission-summary.csv")
        },
        selection="available checkpoints; cohort sums of all original modes/noises/arms, branches separate; 252 original-mode numeric summary rows and all 137 returned anchors",
        output_sha256={
            p.name: sha(p) for p in sorted(args.output.iterdir()) if p.suffix != ".png"
        },
    )
    (args.output / "plot-sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(dict(summary_rows=252, returned_anchors=137, figures=1), indent=2))


if __name__ == "__main__":
    main()
