#!/usr/bin/env python3
"""Plot conditional certificates and actual coverage on every raw-zero loss."""

import argparse
from collections import defaultdict
from itertools import product
import json
from pathlib import Path

from analyze_commanded_recovery_arc import flag
from analyze_epoch_preview import digest, read_rows
from analyze_speed_probe_envelopes import ESTIMATES, KINDS, RADII, STATUSES, YAWS
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    sources = json.loads((args.results / "sources.json").read_text())
    analyzer = Path(__file__).with_name("analyze_speed_probe_envelopes.py")
    assert digest(analyzer) == sources["analysis"]["script_sha256"]
    events = {r["event_id"]: r for r in read_rows(args.results / "events.csv")}
    groups = defaultdict(list)
    for r in read_rows(args.results / "loss-decisions.csv"):
        e = events[r["event_id"]]
        for kind in ["all", e["event_kind"]]:
            groups[
                e["variant"],
                kind,
                r["pose_kind"],
                float(r["radius_m"]),
                int(r["yaw_bound_deg"]),
            ].append(r)
    summary = read_rows(args.results / "summary.csv")
    series = []
    for variant, kind, pose, radius, yaw in product(
        ["epoch", "curved"], ["all", *KINDS], ESTIMATES, RADII, YAWS
    ):
        s = next(
            r
            for r in summary
            if (
                r["cohort"],
                r["variant"],
                r["event_kind"],
                r["pose_kind"],
                r["population"],
                float(r["radius_m"]),
                int(r["yaw_bound_deg"]),
            )
            == ("all", variant, kind, pose, "raw_zero_loss", radius, yaw)
        )
        rows = groups[variant, kind, pose, radius, yaw]
        assert len(rows) == int(s["records"])
        counts = {k: sum(r["status"] == k for r in rows) for k in STATUSES}
        counts.update(
            {
                k: sum(flag(r[k]) for r in rows)
                for k in [
                    "covered",
                    "position_covered",
                    "heading_covered",
                    "clear_truth_blocks",
                    "blocked_truth_clear",
                ]
            }
        )
        assert all(v == int(s[k]) for k, v in counts.items())
        series.append(
            dict(
                variant=variant,
                event_kind=kind,
                pose_kind=pose,
                radius_m=radius,
                yaw_bound_deg=yaw,
                source_loss_pairs=len(rows),
                **counts,
                **{
                    k + "_percent": 100 * v / len(rows) if rows else 0.0
                    for k, v in counts.items()
                },
            )
        )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "speed-probe-envelopes",
        }
    )
    fig, axes = plt.subplots(
        2, 3, figsize=(12, 7), sharex=True, sharey=True, layout="constrained"
    )
    colors = ["#238b45", "#807dba", "#e6e6e6"]
    for i, variant in enumerate(["epoch", "curved"]):
        for j, pose in enumerate(ESTIMATES):
            ax = axes[i, j]
            rows = [
                next(
                    r
                    for r in series
                    if (
                        r["variant"],
                        r["event_kind"],
                        r["pose_kind"],
                        r["radius_m"],
                        r["yaw_bound_deg"],
                    )
                    == (variant, "all", pose, radius, 4)
                )
                for radius in RADII
            ]
            x = list(range(len(RADII)))
            bottom = [0.0] * len(rows)
            for status, color in zip(STATUSES, colors):
                values = [r[status + "_percent"] for r in rows]
                ax.bar(
                    x,
                    values,
                    width=0.72,
                    bottom=bottom,
                    color=color,
                    label=(
                        "Certified " + status
                        if status != "unresolved"
                        else "Unresolved"
                    ),
                )
                if status == "clear":
                    for xx, r, value in zip(x, rows, values):
                        if value >= 20:
                            ax.text(
                                xx,
                                value / 2,
                                str(r[status]),
                                ha="center",
                                va="center",
                                color="white",
                                fontsize=8,
                            )
                bottom = [a + b for a, b in zip(bottom, values)]
            ax.plot(
                x,
                [r["covered_percent"] for r in rows],
                "o--",
                color="#222222",
                linewidth=1.2,
                markersize=3,
                label="True pose inside disk + yaw interval",
            )
            ax.plot(
                x,
                [r["clear_truth_blocks_percent"] for r in rows],
                "D-",
                color="#cb181d",
                linewidth=1.2,
                markersize=4,
                label="Certified clear; same-margin truth blocks",
            )
            for xx, r in zip(x, rows):
                if r["clear_truth_blocks"]:
                    ax.annotate(
                        str(r["clear_truth_blocks"]),
                        (xx, r["clear_truth_blocks_percent"]),
                        xytext=(0, 5),
                        textcoords="offset points",
                        ha="center",
                        color="#a50f15",
                        fontsize=8,
                    )
            arm = "Baseline log" if variant == "epoch" else "Curved-terminal log"
            ax.set_title(
                f"{arm} / {pose.title()} (N={rows[0]['source_loss_pairs']})",
                fontsize=10,
            )
            ax.set_ylim(0, 105)
            ax.set_xticks(x, [f"{r:g}" for r in RADII])
            ax.spines[["top", "right"]].set_visible(False)
            if j == 0:
                ax.set_ylabel("All recorded raw-zero loss pairs (%)")
            if i == 1:
                ax.set_xlabel("Position disk radius (m); heading +/-4 deg")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside upper center", ncol=3, fontsize=8)
    fig.supxlabel(
        "Fixed first chord, steering and margin; all 550 recorded losses. "
        "Red numbers label truth conflicts.\n"
        "Chosen radii are sensitivity settings, not confidence levels.",
        fontsize=9,
    )
    args.output.mkdir(parents=True)
    save_rows(args.output / "probe-envelope-comparisons.csv", series)
    fig.savefig(args.output / "probe-envelope-comparisons.svg", metadata={"Date": None})
    fig.savefig(
        args.output / "probe-envelope-comparisons.png",
        dpi=160,
        metadata={"Software": "rust_robotics speed-probe-envelopes"},
    )
    plt.close(fig)
    provenance = dict(
        input_sha256={
            n: digest(args.results / n)
            for n in [
                "accounting.csv",
                "events.csv",
                "poses.csv",
                "loss-decisions.csv",
                "summary.csv",
                "sources.json",
            ]
        },
        script_sha256=digest(Path(__file__)),
        analysis_sha256=digest(analyzer),
        numeric_rows=len(series),
        matplotlib_version=matplotlib.__version__,
        plotted_population="all 550 raw-zero-loss pairs, separately by arm and input",
        plotted_yaw_bound_deg=4,
    )
    (args.output / "probe-envelope-comparisons-sources.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(json.dumps({"numeric_rows": len(series), "panels": 6}, indent=2))


if __name__ == "__main__":
    main()
