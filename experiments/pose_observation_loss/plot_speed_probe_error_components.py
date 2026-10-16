#!/usr/bin/env python3
"""Plot signed loss-error contributions and within-conflict component moments."""

import argparse
from collections import defaultdict
from itertools import product
import json
import math
from pathlib import Path
import statistics

from analyze_epoch_position_components import PARTS
from analyze_epoch_preview import digest, read_rows
from analyze_speed_probe_starts import ESTIMATES, KINDS
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    source = json.loads((args.results / "sources.json").read_text())
    analyzer = Path(__file__).with_name("analyze_speed_probe_error_components.py")
    assert digest(analyzer) == source["analysis"]["script_sha256"]
    components = {r["pose_id"]: r for r in read_rows(args.results / "components.csv")}
    events = {r["event_id"]: r for r in read_rows(args.results / "events.csv")}
    summary = {
        (r["variant"], r["event_kind"], r["pose_kind"]): r
        for r in read_rows(args.results / "summary.csv")
        if r["cohort"] == "all"
        and r["population"] == "raw_zero_loss"
        and r["condition"] == "all"
    }
    losses = defaultdict(list)
    for l in read_rows(args.results / "loss-memberships.csv"):
        if float(l["radius_m"]) == 0 and int(l["yaw_bound_deg"]) == 0:
            p, e = components[l["pose_id"]], events[l["event_id"]]
            for kind in ["all", e["event_kind"]]:
                losses[p["variant"], kind, p["pose_kind"]].append(p)
    spans = defaultdict(list)
    for s in read_rows(args.results / "span-components.csv"):
        if s["family"] == "clear_conflict" and s["label"] == "conflict":
            for kind in {
                "all",
                *[events[e]["event_kind"] for e in s["source_event_ids"].split("+")],
            }:
                spans[s["variant"], kind, s["pose_kind"]].append(s)
    series = []
    for arm, kind, pose in product(["epoch", "curved"], ["all", *KINDS], ESTIMATES):
        rr = losses[arm, kind, pose]
        assert len(rr) == int(summary[arm, kind, pose]["records"])
        mean = statistics.fmean(float(r["error_norm"]) for r in rr) if rr else ""
        if rr:
            assert math.isclose(
                sum(
                    float(summary[arm, kind, pose][p + "_mean_projection"])
                    for p in PARTS
                ),
                mean,
                abs_tol=1e-10,
            )
        for part in PARTS:
            value = (
                statistics.fmean(float(r[part + "_projection"]) for r in rr)
                if rr
                else ""
            )
            if rr:
                assert math.isclose(
                    value,
                    float(summary[arm, kind, pose][part + "_mean_projection"]),
                    abs_tol=1e-12,
                )
            series.append(
                dict(
                    variant=arm,
                    event_kind=kind,
                    pose_kind=pose,
                    family="loss_projection",
                    part=part,
                    quantity="signed_mean_projection",
                    records=len(rr),
                    spans="",
                    value_m=value,
                    pooled_total_rms_m="",
                    mean_error_norm_m=mean,
                    largest_count=sum(r["largest_projection"] == part for r in rr),
                )
            )
        ss = spans[arm, kind, pose]
        n = sum(int(s["states"]) for s in ss)
        for part in PARTS:
            values = {}
            for quantity, field in [
                ("mean_offset_rms", "mean_norm"),
                ("centered_rms", "centered_rms"),
                ("total", "rms"),
            ]:
                values[quantity] = (
                    math.sqrt(
                        sum(
                            int(s["states"]) * float(s[part + "_" + field]) ** 2
                            for s in ss
                        )
                        / n
                    )
                    if n
                    else ""
                )
            if n:
                assert math.isclose(
                    values["total"] ** 2,
                    values["mean_offset_rms"] ** 2 + values["centered_rms"] ** 2,
                    rel_tol=1e-10,
                    abs_tol=1e-13,
                )
            for quantity in ["mean_offset_rms", "centered_rms"]:
                series.append(
                    dict(
                        variant=arm,
                        event_kind=kind,
                        pose_kind=pose,
                        family="conflict_span_moments",
                        part=part,
                        quantity=quantity,
                        records=n,
                        spans=len(ss),
                        value_m=values[quantity],
                        pooled_total_rms_m=values["total"],
                        mean_error_norm_m="",
                        largest_count="",
                    )
                )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "speed-probe-error-components",
        }
    )
    fig, axes = plt.subplots(2, 3, figsize=(12, 7))
    fig.subplots_adjust(
        left=0.12, right=0.99, bottom=0.16, top=0.86, wspace=0.55, hspace=0.55
    )
    part_colors = ["#3182bd", "#e6550d", "#31a354", "#756bb1"]
    part_names = [
        "Observation noise",
        "Correction anchor",
        "Within-anchor drift",
        "Motion transport",
    ]
    arm_colors = ["#3182bd", "#e6550d"]
    for j, pose in enumerate(ESTIMATES):
        ax = axes[0, j]
        positive, negative = [0.0, 0.0], [0.0, 0.0]
        for part, color, label in zip(PARTS, part_colors, part_names):
            rr = [
                next(
                    r
                    for r in series
                    if (
                        r["variant"],
                        r["event_kind"],
                        r["pose_kind"],
                        r["family"],
                        r["part"],
                    )
                    == (arm, "all", pose, "loss_projection", part)
                )
                for arm in ["epoch", "curved"]
            ]
            values = [100 * float(r["value_m"]) for r in rr]
            starts = [
                positive[i] if v >= 0 else negative[i] for i, v in enumerate(values)
            ]
            ax.barh([0, 1], values, left=starts, height=0.5, color=color, label=label)
            for i, v in enumerate(values):
                if v >= 0:
                    positive[i] += v
                else:
                    negative[i] += v
        means = [
            next(
                float(r["mean_error_norm_m"]) * 100
                for r in series
                if (r["variant"], r["event_kind"], r["pose_kind"], r["family"])
                == (arm, "all", pose, "loss_projection")
            )
            for arm in ["epoch", "curved"]
        ]
        ax.plot(means, [0, 1], "kD", markersize=4)
        lo, hi = min(0.0, *negative, *means), max(*positive, *means)
        extent = max(hi - lo, 1.0)
        ax.set_xlim(lo - 0.05 * extent, hi + 0.08 * extent)
        ax.axvline(0, color="#777777", linewidth=0.6)
        ax.set_yticks([0, 1], ["Baseline (N=197)", "Curved (N=353)"])
        ax.invert_yaxis()
        ax.set_title(pose.title() + "\nAll 550 raw-zero losses", fontsize=10)
        ax.set_xlabel("Signed mean contribution to error (cm)")
        ax.spines[["top", "right"]].set_visible(False)
        ax = axes[1, j]
        for i, (arm, color) in enumerate(zip(["epoch", "curved"], arm_colors)):
            for k, quantity in enumerate(["mean_offset_rms", "centered_rms"]):
                values = [
                    next(
                        float(r["value_m"]) * 100
                        for r in series
                        if (
                            r["variant"],
                            r["event_kind"],
                            r["pose_kind"],
                            r["family"],
                            r["part"],
                            r["quantity"],
                        )
                        == (arm, "all", pose, "conflict_span_moments", part, quantity)
                    )
                    for part in PARTS
                ]
                xx = [x + (i * 2 + k - 1.5) * 0.18 for x in range(4)]
                ax.bar(
                    xx,
                    values,
                    width=0.18,
                    color=color,
                    hatch="///" if k else None,
                    edgecolor="#222222" if k else color,
                    linewidth=0.3,
                )
        ax.set_xticks(range(4), ["Noise", "Anchor", "Drift", "Transport"])
        ax.set_ylabel("Component size (cm)")
        ax.set_title(
            pose.title() + "\nAll clear-conflict spans at .10 m / +/-4 deg", fontsize=10
        )
        ax.set_xlabel("Pooled state-weighted RMS across spans")
        ax.set_ylim(0, max(1, ax.get_ylim()[1]) * 1.4)
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
            Patch(color=arm_colors[0], label="Baseline offset"),
            Patch(color=arm_colors[1], label="Curved offset"),
            Patch(
                facecolor="white",
                edgecolor="#222222",
                hatch="///",
                label="Centered variation (each arm)",
            ),
        ],
        fontsize=7,
        loc="upper right",
    )
    fig.supxlabel(
        "Top: component projections add to mean error norm (black diamond); component norms do not add.\n"
        "Bottom: solid = span mean-vector RMS, hatched = centered RMS. Audited boundaries truncate the spans.",
        fontsize=9,
    )
    args.output.mkdir(parents=True)
    save_rows(args.output / "error-component-comparisons.csv", series)
    svg = args.output / "error-component-comparisons.svg"
    fig.savefig(svg, metadata={"Date": None}, bbox_inches="tight")
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(
        args.output / "error-component-comparisons.png",
        dpi=160,
        bbox_inches="tight",
        metadata={"Software": "rust_robotics speed-probe-error-components"},
    )
    plt.close(fig)
    provenance = dict(
        input_sha256={
            n: digest(args.results / n)
            for n in [
                "accounting.csv",
                "events.csv",
                "loss-memberships.csv",
                "reconstruction.csv",
                "components.csv",
                "groups.csv",
                "summary.csv",
                "span-components.csv",
                "sources.json",
            ]
        },
        script_sha256=digest(Path(__file__)),
        analysis_sha256=digest(analyzer),
        matplotlib_version=matplotlib.__version__,
        numeric_rows=len(series),
        plotted_population="all 550 losses; separately all default-setting clear-conflict spans",
        svg_normalization="strip trailing whitespace from generated XML lines",
    )
    (args.output / "error-component-comparisons-sources.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(json.dumps(dict(numeric_rows=len(series), panels=6), indent=2))


if __name__ == "__main__":
    main()
