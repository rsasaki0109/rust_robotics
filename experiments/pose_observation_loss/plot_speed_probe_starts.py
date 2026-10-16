#!/usr/bin/env python3
"""Plot the complete raw-zero-loss population under fixed geometric pose queries."""

import argparse
import json
from pathlib import Path

from analyze_commanded_recovery_arc import flag
from analyze_speed_probe_starts import CATEGORIES, ESTIMATES, KINDS, RELATIONS
from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

NAMES = dict(
    start_obstacle="Start: obstacle",
    start_wall="Start: wall",
    start_both="Start: obstacle and wall",
    first_chord_obstacle="First chord: obstacle",
    first_chord_wall="First chord: wall",
    first_chord_both="First chord: obstacle and wall",
    positive_first_chord="First chord clear",
    both_clear="Estimate and truth clear",
    estimate_clear_truth_blocks="Estimate clear, truth blocks",
    estimate_blocks_truth_clear="Estimate blocks, truth clear",
    both_block="Estimate and truth block",
)
FAMILIES = [
    (
        "stage",
        CATEGORIES,
        ["#e6550d", "#756bb1", "#636363", "#fdae6b", "#9e9ac8", "#bdbdbd", "#238b45"],
    ),
    ("truth_relation", RELATIONS, ["#238b45", "#e6550d", "#3182bd", "#969696"]),
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
        digest(Path(__file__).with_name("analyze_speed_probe_starts.py"))
        == source["analysis"]["script_sha256"]
    )
    summary, events, blocks = (
        read_rows(args.results / n)
        for n in ["pair-summary.csv", "events.csv", "start-blocks.csv"]
    )
    E = {e["event_id"]: e for e in events}
    kinds = ["all", *[k for k in KINDS if any(e["event_kind"] == k for e in events)]]
    series = []
    for variant in ["epoch", "curved"]:
        for kind in kinds:
            expected = sum(
                r["variant"] == variant
                and (kind == "all" or E[r["event_id"]]["event_kind"] == kind)
                for r in blocks
            )
            for pose in ESTIMATES:
                r = next(
                    r
                    for r in summary
                    if (
                        r["cohort"],
                        r["variant"],
                        r["event_kind"],
                        r["phase"],
                        r["source_label"],
                        r["pose_kind"],
                        r["subset"],
                    )
                    == (
                        "all",
                        variant,
                        kind,
                        "post",
                        "joint_lost",
                        pose,
                        "raw_zero_loss",
                    )
                )
                assert int(r["pairs"]) == expected
                for family, bins, _ in FAMILIES:
                    assert sum(int(r[k]) for k in bins) == expected
                    for category in bins:
                        n = int(r[category])
                        series.append(
                            dict(
                                variant=variant,
                                event_kind=kind,
                                pose_kind=pose,
                                family=family,
                                category=category,
                                source_loss_pairs=expected,
                                count=n,
                                percent=100 * n / expected if expected else 0.0,
                            )
                        )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "speed-probe-starts",
        }
    )
    fig, axes = plt.subplots(2, 2, figsize=(12, 6), sharex=True, layout="constrained")
    pose_labels = dict(raw="Raw", recovery="Recovery", preview="Preview")
    for i, (variant, title) in enumerate(
        [("epoch", "Baseline log"), ("curved", "Curved-terminal log")]
    ):
        for j, (family, bins, colors) in enumerate(FAMILIES):
            ax = axes[i, j]
            left = [0.0] * 3
            for category, color in zip(bins, colors):
                rr = [
                    next(
                        r
                        for r in series
                        if (
                            r["variant"],
                            r["event_kind"],
                            r["pose_kind"],
                            r["family"],
                            r["category"],
                        )
                        == (variant, "all", pose, family, category)
                    )
                    for pose in ESTIMATES
                ]
                widths = [r["percent"] for r in rr]
                ax.barh(
                    range(3),
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
                                if color
                                in [
                                    "#e6550d",
                                    "#756bb1",
                                    "#636363",
                                    "#238b45",
                                    "#3182bd",
                                ]
                                else "#222222"
                            ),
                        )
                left = [a + b for a, b in zip(left, widths)]
            ticks = []
            for pose in ESTIMATES:
                r = next(
                    r
                    for r in series
                    if (r["variant"], r["event_kind"], r["pose_kind"], r["family"])
                    == (variant, "all", pose, family)
                )
                ticks.append(f"{pose_labels[pose]} (N={r['source_loss_pairs']})")
            ax.set_yticks(range(3), ticks)
            ax.invert_yaxis()
            ax.set_xlim(0, 100)
            ax.set_xticks([0, 25, 50, 75, 100])
            ax.set_title(
                f"{title} / {'first-chord classification' if j==0 else 'same-margin truth comparison'}",
                loc="left",
                fontsize=10,
            )
            ax.spines[["top", "right"]].set_visible(False)
            if i == 1:
                ax.set_xlabel("All recorded raw-zero loss pairs (%)")
    fig.suptitle(
        "Fixed steering and margin: first-chord queries at recorded raw-zero losses",
        fontsize=12,
    )
    for j in [0, 1]:
        handles, labels = axes[0, j].get_legend_handles_labels()
        fig.legend(
            handles,
            labels,
            ncol=2 if j == 0 else 1,
            frameon=False,
            loc="lower center",
            bbox_to_anchor=(0.27 if j == 0 else 0.77, -0.18),
            fontsize=8,
        )
    args.output.mkdir(parents=True)
    numeric = args.output / "probe-start-comparisons.csv"
    save_rows(numeric, series)
    svg = args.output / "probe-start-comparisons.svg"
    fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(
        args.output / "probe-start-comparisons.png", dpi=160, bbox_inches="tight"
    )
    plt.close(fig)
    fingerprints = dict(
        script_sha256=digest(Path(__file__)),
        inputs_sha256={
            n: digest(args.results / n)
            for n in [
                "pair-summary.csv",
                "events.csv",
                "start-blocks.csv",
                "sources.json",
            ]
        },
        artifacts_sha256={p.name: digest(p) for p in [numeric, svg]},
        matplotlib_version=matplotlib.__version__,
    )
    (args.output / "probe-start-comparisons-sources.json").write_text(
        json.dumps(fingerprints, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
