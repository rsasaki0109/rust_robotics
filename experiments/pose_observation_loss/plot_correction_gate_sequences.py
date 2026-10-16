#!/usr/bin/env python3
"""Plot all recorded correction windows and the conditional joint-start subset."""

import argparse
import json
from pathlib import Path

from analyze_correction_gate_sequences import STAGES
from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

LABELS = dict(
    censored="Post window unavailable",
    independent_start="Independent guard at event",
    inactive_start="Inactive at event",
    joint_resumed="Joint: actual resume",
    joint_no_intervals="Joint: no later joint decisions",
    joint_no_base="Joint: base never usable",
    joint_no_full="Joint: base usable, never all ready",
    joint_short_run="Joint: ready, longest run <1 s",
    joint_long_no_resume="Joint: run >=1 s, no resume",
)
COLORS = [
    "#bdbdbd",
    "#9ecae1",
    "#deebf7",
    "#238b45",
    "#756bb1",
    "#636363",
    "#e6550d",
    "#fdae6b",
    "#9e9ac8",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output directory must be new")
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        digest(Path(__file__).with_name("analyze_correction_gate_sequences.py"))
        == source["analysis"]["script_sha256"]
    )
    events, summary = (
        read_rows(args.results / n) for n in ["events.csv", "summary.csv"]
    )
    names = dict(
        received="Received",
        missed="Missed",
        precision_only="Precision",
        received_and_precision="Received + precision",
        missed_and_precision="Missed + precision",
    )
    categories = [k for k in names if any(e["event_kind"] == k for e in events)]
    series = []
    for variant in ["epoch", "curved"]:
        for scope in ["all", "joint"]:
            for kind in categories:
                row = next(
                    r
                    for r in summary
                    if (r["cohort"], r["variant"], r["event_kind"], r["event_route"])
                    == ("all", variant, kind, scope)
                )
                n = int(row["events"])
                assert n == sum(
                    e["variant"] == variant
                    and e["event_kind"] == kind
                    and (scope == "all" or e["event_route"] == "joint")
                    for e in events
                )
                assert sum(int(row[k]) for k in STAGES) == n
                for stage in STAGES:
                    count = int(row[stage])
                    series.append(
                        dict(
                            variant=variant,
                            event_route=scope,
                            event_kind=kind,
                            stage=stage,
                            events=n,
                            count=count,
                            percent=100 * count / n if n else 0.0,
                        )
                    )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "correction-gate-sequences",
        }
    )
    fig, axes = plt.subplots(2, 2, figsize=(12, 6), sharex=True, layout="constrained")
    for i, (variant, title) in enumerate(
        [("epoch", "Baseline log"), ("curved", "Curved-terminal log")]
    ):
        for j, (scope, scope_title) in enumerate(
            [("all", "all opportunities"), ("joint", "joint at event only")]
        ):
            ax = axes[i, j]
            left = [0.0] * len(categories)
            ticks = []
            for kind in categories:
                row = next(
                    r
                    for r in series
                    if (r["variant"], r["event_route"], r["event_kind"])
                    == (variant, scope, kind)
                )
                ticks.append(f"{names[kind]} (N={row['events']})")
            for stage, color in zip(STAGES, COLORS):
                rows = [
                    next(
                        r
                        for r in series
                        if (r["variant"], r["event_route"], r["event_kind"], r["stage"])
                        == (variant, scope, kind, stage)
                    )
                    for kind in categories
                ]
                widths = [r["percent"] for r in rows]
                ax.barh(
                    range(len(categories)),
                    widths,
                    left=left,
                    height=0.58,
                    color=color,
                    label=LABELS[stage],
                )
                for y, (start, width, row) in enumerate(zip(left, widths, rows)):
                    if width >= 4:
                        ax.text(
                            start + width / 2,
                            y,
                            str(row["count"]),
                            ha="center",
                            va="center",
                            fontsize=9,
                            color=(
                                "white"
                                if stage
                                in ["joint_resumed", "joint_no_base", "joint_no_full"]
                                else "#222222"
                            ),
                        )
                left = [a + b for a, b in zip(left, widths)]
            ax.set_yticks(range(len(categories)), ticks)
            ax.invert_yaxis()
            ax.set_xlim(0, 100)
            ax.set_xticks([0, 25, 50, 75, 100])
            ax.set_title(f"{title} / {scope_title}", fontsize=10, loc="left")
            ax.spines[["top", "right"]].set_visible(False)
            if i == 1:
                ax.set_xlabel("Opportunities (%); post-event horizon 1.5 s")
    fig.suptitle(
        "Timeout records: simultaneous requirements and uninterrupted release evidence",
        fontsize=12,
    )
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        ncol=3,
        frameon=False,
        loc="lower center",
        bbox_to_anchor=(0.52, -0.18),
        fontsize=8,
    )
    args.output.mkdir(parents=True)
    numeric = args.output / "gate-sequence-partitions.csv"
    save_rows(numeric, series)
    svg = args.output / "gate-sequence-partitions.svg"
    fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(
        args.output / "gate-sequence-partitions.png", dpi=160, bbox_inches="tight"
    )
    plt.close(fig)
    fingerprints = dict(
        script_sha256=digest(Path(__file__)),
        inputs_sha256={
            n: digest(args.results / n)
            for n in ["summary.csv", "events.csv", "sources.json"]
        },
        artifacts_sha256={p.name: digest(p) for p in [numeric, svg]},
        matplotlib_version=matplotlib.__version__,
    )
    (args.output / "gate-sequence-partitions-sources.json").write_text(
        json.dumps(fingerprints, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
