#!/usr/bin/env python3
"""Plot complete loss coverage and audited clear-conflict elapsed support."""

import argparse
from collections import defaultdict
from itertools import product
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_speed_probe_coverage_spans import COVERAGES
from analyze_speed_probe_starts import ESTIMATES, KINDS
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

RADIUS, YAW = 0.1, 4
BINS = ["singleton", "0.02_to_0.08", "0.10_to_0.48", "0.50_or_more"]


def support_bin(states):
    assert states >= 1
    return BINS[0 if states == 1 else 1 if states <= 5 else 2 if states <= 25 else 3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    source = json.loads((args.results / "sources.json").read_text())
    analyzer = Path(__file__).with_name("analyze_speed_probe_coverage_spans.py")
    assert digest(analyzer) == source["analysis"]["script_sha256"]
    events = {r["event_id"]: r for r in read_rows(args.results / "events.csv")}
    cs = {
        (r["variant"], r["event_kind"], r["pose_kind"]): r
        for r in read_rows(args.results / "coverage-summary.csv")
        if r["cohort"] == "all"
        and r["population"] == "raw_zero_loss"
        and float(r["radius_m"]) == RADIUS
        and int(r["yaw_bound_deg"]) == YAW
    }
    ss = {
        (r["variant"], r["event_kind"], r["pose_kind"]): r
        for r in read_rows(args.results / "span-summary.csv")
        if r["cohort"] == "all"
        and r["family"] == "clear_conflict"
        and r["label"] == "conflict"
        and float(r["radius_m"]) == RADIUS
        and int(r["yaw_bound_deg"]) == YAW
    }
    losses, conflicts = defaultdict(list), defaultdict(list)
    for r in read_rows(args.results / "loss-memberships.csv"):
        if float(r["radius_m"]) == RADIUS and int(r["yaw_bound_deg"]) == YAW:
            e = events[r["event_id"]]
            for kind in ["all", e["event_kind"]]:
                losses[e["variant"], kind, r["pose_kind"]].append(r)
    for s in read_rows(args.results / "spans.csv"):
        if (
            float(s["radius_m"]) == RADIUS
            and int(s["yaw_bound_deg"]) == YAW
            and s["family"] == "clear_conflict"
            and s["label"] == "conflict"
        ):
            for kind in {
                "all",
                *[events[e]["event_kind"] for e in s["source_event_ids"].split("+")],
            }:
                conflicts[s["variant"], kind, s["pose_kind"]].append(s)
    series = []
    for variant, kind, pose in product(["epoch", "curved"], ["all", *KINDS], ESTIMATES):
        key = variant, kind, pose
        rows = losses[key]
        assert len(rows) == int(cs[key]["records"])
        for category in COVERAGES:
            n = sum(r["coverage"] == category for r in rows)
            assert n == int(cs[key]["coverage_" + category])
            series.append(
                dict(
                    variant=variant,
                    event_kind=kind,
                    pose_kind=pose,
                    radius_m=RADIUS,
                    yaw_bound_deg=YAW,
                    family="loss_coverage",
                    category=category,
                    audit_boundary="not_applicable",
                    denominator=len(rows),
                    count=n,
                    percent=100 * n / len(rows) if rows else 0.0,
                )
            )
        spans = conflicts[key]
        assert len(spans) == int(ss[key]["spans"])
        assert sum(int(s["states"]) for s in spans) == int(ss[key]["states"])
        for category, boundary in product(BINS, ["interior", "truncated"]):
            n = sum(
                support_bin(int(s["states"])) == category
                and (
                    s["left_boundary"] == "audit_start"
                    or s["right_boundary"] == "audit_end"
                )
                == (boundary == "truncated")
                for s in spans
            )
            series.append(
                dict(
                    variant=variant,
                    event_kind=kind,
                    pose_kind=pose,
                    radius_m=RADIUS,
                    yaw_bound_deg=YAW,
                    family="clear_conflict_support",
                    category=category,
                    audit_boundary=boundary,
                    denominator=len(spans),
                    count=n,
                    percent=100 * n / len(spans) if spans else 0.0,
                )
            )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "speed-probe-coverage-spans",
        }
    )
    fig, axes = plt.subplots(2, 3, figsize=(12, 7))
    fig.subplots_adjust(
        left=0.12, right=0.99, bottom=0.15, top=0.87, wspace=0.55, hspace=0.5
    )
    coverage_names = [
        "Inside both bounds",
        "Position only outside",
        "Heading only outside",
        "Both outside",
    ]
    coverage_colors = ["#238b45", "#e6550d", "#3182bd", "#756bb1"]
    arm_colors = ["#3182bd", "#e6550d"]
    for j, pose in enumerate(ESTIMATES):
        ax = axes[0, j]
        left = [0.0, 0.0]
        for category, label, color in zip(COVERAGES, coverage_names, coverage_colors):
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
                    == (arm, "all", pose, "loss_coverage", category)
                )
                for arm in ["epoch", "curved"]
            ]
            widths = [r["percent"] for r in rr]
            ax.barh([0, 1], widths, left=left, color=color, height=0.55, label=label)
            for y, start, width, r in zip([0, 1], left, widths, rr):
                if width >= 6:
                    ax.text(
                        start + width / 2,
                        y,
                        str(r["count"]),
                        ha="center",
                        va="center",
                        color="white",
                        fontsize=9,
                    )
            left = [a + b for a, b in zip(left, widths)]
        ax.set_yticks([0, 1], ["Baseline (N=197)", "Curved (N=353)"])
        ax.invert_yaxis()
        ax.set_xlim(0, 100)
        ax.set_xlabel("All recorded raw-zero loss pairs (%)")
        ax.set_title(pose.title() + "\nr=.10 m, yaw +/-4 deg", fontsize=10)
        ax.spines[["top", "right"]].set_visible(False)
        ax = axes[1, j]
        for i, (arm, color) in enumerate(zip(["epoch", "curved"], arm_colors)):
            rr = [
                r
                for r in series
                if (r["variant"], r["event_kind"], r["pose_kind"], r["family"])
                == (arm, "all", pose, "clear_conflict_support")
            ]
            counts = [sum(r["count"] for r in rr if r["category"] == b) for b in BINS]
            truncated = [
                next(
                    r["count"]
                    for r in rr
                    if r["category"] == b and r["audit_boundary"] == "truncated"
                )
                for b in BINS
            ]
            xx = [k + (i - 0.5) * 0.36 for k in range(4)]
            ax.bar(xx, counts, width=0.36, color=color)
            ax.bar(
                xx,
                truncated,
                width=0.36,
                facecolor="none",
                edgecolor="#222222",
                linewidth=0.3,
                hatch="///",
            )
            for x, n in zip(xx, counts):
                if n:
                    ax.annotate(
                        str(n),
                        (x, n),
                        xytext=(0, 3),
                        textcoords="offset points",
                        ha="center",
                        fontsize=8,
                    )
        ax.set_title(pose.title() + " / all audited clear-conflict spans", fontsize=10)
        ax.set_xticks(range(4), ["Single\nsample", ".02–.08", ".10–.48", ">=.50"])
        ax.set_xlabel("Elapsed support (s); singleton = 0")
        ax.set_ylabel("Spans (count)")
        ax.set_ylim(0, max(1, ax.get_ylim()[1]) * 1.17)
        ax.spines[["top", "right"]].set_visible(False)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.995),
        ncol=4,
        fontsize=9,
    )
    axes[1, 2].legend(
        handles=[
            Patch(color=arm_colors[0], label="Baseline log"),
            Patch(color=arm_colors[1], label="Curved log"),
            Patch(
                facecolor="none",
                edgecolor="#222222",
                hatch="///",
                label="Touches audit boundary",
            ),
        ],
        fontsize=8,
    )
    fig.supxlabel(
        "Coverage uses all 550 loss pairs. Duration uses all audited states; "
        "each span stops at a missing audit edge.\n"
        "Elapsed support does not establish onset/resolution or a calibrated error probability.",
        fontsize=9,
    )
    args.output.mkdir(parents=True)
    save_rows(args.output / "coverage-span-comparisons.csv", series)
    svg = args.output / "coverage-span-comparisons.svg"
    fig.savefig(svg, metadata={"Date": None}, bbox_inches="tight")
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(
        args.output / "coverage-span-comparisons.png",
        dpi=160,
        metadata={"Software": "rust_robotics speed-probe-coverage-spans"},
        bbox_inches="tight",
    )
    plt.close(fig)
    provenance = dict(
        input_sha256={
            n: digest(args.results / n)
            for n in [
                "accounting.csv",
                "events.csv",
                "chains.csv",
                "members.csv",
                "spans.csv",
                "loss-memberships.csv",
                "coverage-summary.csv",
                "span-summary.csv",
                "sources.json",
            ]
        },
        script_sha256=digest(Path(__file__)),
        analysis_sha256=digest(analyzer),
        matplotlib_version=matplotlib.__version__,
        numeric_rows=len(series),
        radius_m=RADIUS,
        yaw_bound_deg=YAW,
        span_bins_by_states=["1", "2..5", "6..25", ">=26"],
        svg_normalization="strip trailing whitespace from generated XML lines",
    )
    (args.output / "coverage-span-comparisons-sources.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(json.dumps(dict(numeric_rows=len(series), panels=6), indent=2))


if __name__ == "__main__":
    main()
