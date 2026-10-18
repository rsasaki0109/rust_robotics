#!/usr/bin/env python3
"""Plot all original 25 point-gap histories and explicit missing contexts."""

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
        "svg.hashsalt": "fusion-return-point-history-v1",
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
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    grouped = {}
    for row in rows:
        grouped.setdefault((row["event_id"], row["reference_time_s"]), []).append(row)
    assert len(grouped) == 25
    bounds = [
        float(row[k + "_gap_" + b])
        for row in rows
        if row["slot_status"] == "observed"
        for k in ("raw", "mean")
        for b in ("lo", "hi")
    ]
    low, high = min(0, min(bounds)), max(0, max(bounds))
    pad = max((high - low) * 0.04, 1e-3)
    fig, axes = plt.subplots(5, 5, figsize=(18, 16), sharex=True, sharey=True)
    for i, ((event, reference), rr) in enumerate(grouped.items()):
        ax = axes.flat[i]
        assert [int(row["offset_steps"]) for row in rr] == list(range(-25, 26))
        x = [int(row["offset_steps"]) / 50 for row in rr]
        before = [t for t, row in zip(x, rr) if row["context_phase"] == "before_return"]
        if before:
            ax.axvspan(-0.5, max(before), color="#eeeeee", zorder=0)
        for kind, color in (("raw", "#4477aa"), ("mean", "#ee7733")):
            values = {
                b: [
                    (
                        float(row[f"{kind}_gap_{b}"])
                        if row["slot_status"] == "observed"
                        else float("nan")
                    )
                    for row in rr
                ]
                for b in ("center", "lo", "hi")
            }
            ax.plot(x, values["center"], color=color, linewidth=1.2)
            ax.fill_between(x, values["lo"], values["hi"], color=color, alpha=0.25)
        ax.axhline(0, color="#555555", linewidth=0.8)
        ax.axvline(0, color="#777777", linestyle=":", linewidth=0.8)
        suffix = "; raw away" if rr[0]["raw_away_reference"] == "True" else ""
        ax.set_title(
            f"{rr[0]['case_id']} / {event} / {float(reference):.2f} s{suffix}",
            fontsize=8.5,
        )
        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(low - pad, high + pad)
        ax.set_xticks((-0.5, 0, 0.5))
        ax.tick_params(labelsize=8)
        if i % 5 == 0:
            ax.set_ylabel("Point gap (m)", fontsize=9)
        if i >= 20:
            ax.set_xlabel("Offset from reference (s)", fontsize=9)
        ax.grid(alpha=0.15)
    fig.suptitle(
        "Fixed original obstacle point gaps: all 25 nominal-only conjunction failures",
        fontsize=15,
        y=0.985,
    )
    fig.legend(
        handles=[
            Line2D([0], [0], color="#4477aa", label="Raw start point"),
            Line2D([0], [0], color="#ee7733", label="Reconstructed mean start point"),
            Patch(
                facecolor="#eeeeee", label="Before first return (historical context)"
            ),
        ],
        loc="lower center",
        bbox_to_anchor=(0.5, 0.035),
        ncol=3,
        frameon=False,
    )
    fig.text(
        0.5,
        0.018,
        "Missing future slots break lines at the original next loss or source end. Point gaps do not replace chord or controller predicates.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=(0, 0.075, 1, 0.96))
    fig.savefig(out / "point-history.svg", metadata={"Date": None})
    fig.savefig(
        out / "point-history.png",
        dpi=130,
        metadata={"Software": "pose-observation-loss-point-history"},
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
    for n in ("anchor-history.csv", "history-summary.csv", "history-slots.csv"):
        assert sha(args.results / n) == source["output_sha256"][n]
    rows = read(args.results / "history-summary.csv")
    selected = [r for r in rows if r["mode"] != "all_modes"]
    returned = [
        r for r in read(args.results / "anchor-history.csv") if r["returned"] == "True"
    ]
    assert len(rows) == 336 and len(selected) == 252 and len(returned) == 137
    args.output.mkdir(parents=True)
    write(args.output / "plot-values.csv", selected)
    write(args.output / "plot-return-values.csv", returned)
    slots = read(args.results / "history-slots.csv")
    assert len(slots) == 1275
    write(args.output / "plot-history.csv", slots)
    figure(slots, args.output)
    meta = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in (
                "sources.json",
                "anchor-history.csv",
                "history-summary.csv",
                "history-slots.csv",
            )
        },
        output_sha256={
            n: sha(args.output / n)
            for n in (
                "point-history.svg",
                "plot-history.csv",
                "plot-values.csv",
                "plot-return-values.csv",
            )
        },
    )
    (args.output / "plot-sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(
        json.dumps(dict(plotted_summary_rows=252, plotted_return_rows=137, figures=2)),
        flush=True,
    )


if __name__ == "__main__":
    main()
