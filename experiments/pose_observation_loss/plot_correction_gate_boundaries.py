#!/usr/bin/env python3
"""Plot every post-event joint-readiness loss with its full pair denominator."""

import argparse
from collections import Counter
import json
from pathlib import Path

from analyze_correction_gate_boundaries import DEPTHS, KINDS
from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

MASKS = [
    "command_only",
    "recovery_only",
    "stop_only",
    "terminal_only",
    "other_single",
    "multiple",
]
NAMES = dict(
    command_only="Nominal command only",
    recovery_only="Recovery geometry only",
    stop_only="Stopping preview only",
    terminal_only="Terminal geometry only",
    other_single="Other single requirement",
    multiple="Multiple requirements",
    no_geometric_failure="No failed geometric gate",
    within_1cm="All failed geometric gaps >= -1 cm",
    beyond_1cm="A failed geometric gap < -1 cm",
)


def mask_group(value):
    mask = int(value)
    assert mask > 0
    if mask.bit_count() > 1:
        return "multiple"
    return {
        16: "command_only",
        8: "recovery_only",
        32: "stop_only",
        64: "terminal_only",
    }.get(mask, "other_single")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output directory must be new")
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        digest(Path(__file__).with_name("analyze_correction_gate_boundaries.py"))
        == source["analysis"]["script_sha256"]
    )
    events, changes, summary = (
        read_rows(args.results / n)
        for n in ["events.csv", "changes.csv", "summary.csv"]
    )
    E = {e["event_id"]: e for e in events}
    categories = [k for k in KINDS if any(e["event_kind"] == k for e in events)]
    labels = dict(
        received="Received",
        missed="Missed",
        precision_only="Precision",
        received_and_precision="Received + precision",
        missed_and_precision="Missed + precision",
    )
    losses = [c for c in changes if (c["phase"], c["label"]) == ("post", "joint_lost")]
    series = []
    for variant in ["epoch", "curved"]:
        for kind in categories:
            z = [
                c
                for c in losses
                if (E[c["event_id"]]["variant"], E[c["event_id"]]["event_kind"])
                == (variant, kind)
            ]
            expected = next(
                r
                for r in summary
                if (
                    r["cohort"],
                    r["variant"],
                    r["event_kind"],
                    r["event_route"],
                    r["phase"],
                )
                == ("all", variant, kind, "all", "post")
            )
            assert len(z) == int(expected["joint_lost"])
            for family, bins in [("failed_set", MASKS), ("geometric_depth", DEPTHS)]:
                counts = Counter(
                    (
                        mask_group(c["failed_mask"])
                        if family == "failed_set"
                        else c["geometric_depth"]
                    )
                    for c in z
                )
                assert sum(counts.values()) == len(z) and set(counts) <= set(bins)
                for category in bins:
                    n = counts[category]
                    series.append(
                        dict(
                            variant=variant,
                            event_kind=kind,
                            family=family,
                            category=category,
                            loss_pairs=len(z),
                            count=n,
                            percent=100 * n / len(z) if z else 0.0,
                        )
                    )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "correction-gate-boundaries",
        }
    )
    fig, axes = plt.subplots(2, 2, figsize=(12, 6), sharex=True, layout="constrained")
    for i, (variant, title) in enumerate(
        [("epoch", "Baseline log"), ("curved", "Curved-terminal log")]
    ):
        for j, (family, bins, colors, subtitle) in enumerate(
            [
                (
                    "failed_set",
                    MASKS,
                    ["#756bb1", "#e6550d", "#3182bd", "#31a354", "#969696", "#fdae6b"],
                    "failed requirements",
                ),
                (
                    "geometric_depth",
                    DEPTHS,
                    ["#756bb1", "#9ecae1", "#e6550d"],
                    "geometric failure depth",
                ),
            ]
        ):
            ax = axes[i, j]
            left = [0.0] * len(categories)
            for category, color in zip(bins, colors):
                rr = [
                    next(
                        r
                        for r in series
                        if (r["variant"], r["event_kind"], r["family"], r["category"])
                        == (variant, kind, family, category)
                    )
                    for kind in categories
                ]
                widths = [r["percent"] for r in rr]
                ax.barh(
                    range(len(categories)),
                    widths,
                    left=left,
                    height=0.58,
                    color=color,
                    label=NAMES[category],
                )
                for y, (start, width, row) in enumerate(zip(left, widths, rr)):
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
                                if color in ["#756bb1", "#e6550d", "#3182bd", "#31a354"]
                                else "#222222"
                            ),
                        )
                left = [a + b for a, b in zip(left, widths)]
            ticks = []
            for kind in categories:
                row = next(
                    r
                    for r in series
                    if (r["variant"], r["event_kind"], r["family"])
                    == (variant, kind, family)
                )
                ticks.append(f"{labels[kind]} (N={row['loss_pairs']})")
            ax.set_yticks(range(len(categories)), ticks)
            ax.invert_yaxis()
            ax.set_xlim(0, 100)
            ax.set_xticks([0, 25, 50, 75, 100])
            ax.set_title(f"{title} / {subtitle}", fontsize=10, loc="left")
            ax.spines[["top", "right"]].set_visible(False)
            if i == 1:
                ax.set_xlabel("All observed joint-readiness losses (%)")
    fig.suptitle(
        "Post-event joint readiness losses within 1.5 s: recorded boundaries",
        fontsize=12,
    )
    for j in [0, 1]:
        handles, texts = axes[0, j].get_legend_handles_labels()
        fig.legend(
            handles,
            texts,
            ncol=2 if j == 0 else 1,
            frameon=False,
            loc="lower center",
            bbox_to_anchor=(0.26 if j == 0 else 0.78, -0.15),
            fontsize=8,
        )
    args.output.mkdir(parents=True)
    numeric = args.output / "gate-boundary-losses.csv"
    save_rows(numeric, series)
    svg = args.output / "gate-boundary-losses.svg"
    fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(args.output / "gate-boundary-losses.png", dpi=160, bbox_inches="tight")
    plt.close(fig)
    fingerprints = dict(
        script_sha256=digest(Path(__file__)),
        inputs_sha256={
            n: digest(args.results / n)
            for n in ["events.csv", "changes.csv", "summary.csv", "sources.json"]
        },
        artifacts_sha256={p.name: digest(p) for p in [numeric, svg]},
        matplotlib_version=matplotlib.__version__,
    )
    (args.output / "gate-boundary-losses-sources.json").write_text(
        json.dumps(fingerprints, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
