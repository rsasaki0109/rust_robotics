#!/usr/bin/env python3
"""Plot actual admission timers at all fixed local contexts."""

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
        "svg.hashsalt": "fusion-return-local-admission-v1",
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
    fig, axes = plt.subplots(5, 5, figsize=(18, 16), sharex=True, sharey=True)
    for i, ((event, reference), rr) in enumerate(grouped.items()):
        ax = axes.flat[i]
        assert [int(row["offset_steps"]) for row in rr] == list(range(-25, 26))
        x = [int(row["offset_steps"]) / 50 for row in rr]
        before = [t for t, row in zip(x, rr) if row["context_phase"] == "before_return"]
        if before:
            ax.axvspan(-0.5, max(before), color="#eeeeee", zorder=0)
        for branch, color in (("joint", "#4477aa"), ("guard", "#ee7733")):
            y = [
                (
                    int(row["local_timer_after_ticks"])
                    if row["local_audited"] == "True" and row["local_branch"] == branch
                    else float("nan")
                )
                for row in rr
            ]
            ax.plot(x, y, color=color, linewidth=1.3)
        ax.axhline(50, color="#4477aa", linestyle="--", linewidth=0.8)
        ax.axhline(25, color="#ee7733", linestyle="--", linewidth=0.8)
        ax.axvline(0, color="#777777", linestyle=":", linewidth=0.8)
        suffix = "; raw away" if rr[0]["raw_away_reference"] == "True" else ""
        ax.set_title(
            f"{rr[0]['case_id']} / {event} / {float(reference):.2f} s{suffix}",
            fontsize=8.5,
        )
        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(0, 50)
        ax.set_xticks((-0.5, 0, 0.5))
        ax.set_yticks((0, 25, 50))
        ax.tick_params(labelsize=8)
        if i % 5 == 0:
            ax.set_ylabel("Actual timer after (ticks)", fontsize=9)
        if i >= 20:
            ax.set_xlabel("Offset from reference (s)", fontsize=9)
        ax.grid(alpha=0.15)
    fig.suptitle(
        "Original admission timers over the fixed 25 local histories",
        fontsize=15,
        y=0.985,
    )
    fig.legend(
        handles=[
            Line2D([0], [0], color="#4477aa", label="Joint (50 ticks = 1 s)"),
            Line2D([0], [0], color="#ee7733", label="Guard (25 ticks = 0.5 s)"),
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
        "Each ready decision adds 0.02 s; failed decisions reset the active timer. Missing future slots break lines.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=(0, 0.075, 1, 0.96))
    fig.savefig(out / "local-admission.svg", metadata={"Date": None})
    fig.savefig(
        out / "local-admission.png",
        dpi=130,
        metadata={"Software": "pose-observation-loss-local-admission"},
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
    for n in ("anchor-local.csv", "local-summary.csv", "local-slots.csv"):
        assert sha(args.results / n) == source["output_sha256"][n]
    rows = read(args.results / "local-summary.csv")
    selected = [r for r in rows if r["mode"] != "all_modes"]
    returned = [
        r for r in read(args.results / "anchor-local.csv") if r["returned"] == "True"
    ]
    assert len(rows) == 336 and len(selected) == 252 and len(returned) == 137
    args.output.mkdir(parents=True)
    write(args.output / "plot-values.csv", selected)
    write(args.output / "plot-return-values.csv", returned)
    slots = read(args.results / "local-slots.csv")
    assert len(slots) == 1275
    write(args.output / "plot-local.csv", slots)
    figure(slots, args.output)
    meta = dict(
        script_sha256=sha(Path(__file__)),
        input_sha256={
            n: sha(args.results / n)
            for n in (
                "sources.json",
                "anchor-local.csv",
                "local-summary.csv",
                "local-slots.csv",
            )
        },
        output_sha256={
            n: sha(args.output / n)
            for n in (
                "local-admission.svg",
                "plot-local.csv",
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
