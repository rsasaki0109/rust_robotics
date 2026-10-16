#!/usr/bin/env python3
"""Plot complete loss agreement and partial-model difference uncertainty."""

import argparse
from collections import defaultdict
from itertools import product
import json
import math
from pathlib import Path

from analyze_commanded_recovery_arc import flag
from analyze_epoch_preview import digest, read_rows
from analyze_speed_probe_shared_information import BINS, PAIRINGS, category
from analyze_speed_probe_starts import KINDS
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

THRESHOLD = 0.05


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    source = json.loads((args.results / "sources.json").read_text())
    analyzer = Path(__file__).with_name("analyze_speed_probe_shared_information.py")
    assert digest(analyzer) == source["analysis"]["script_sha256"]
    events = {r["event_id"]: r for r in read_rows(args.results / "events.csv")}
    pairs = {r["pair_id"]: r for r in read_rows(args.results / "pairs.csv")}
    grouped = defaultdict(list)
    for l in read_rows(args.results / "loss-links.csv"):
        e = events[l["event_id"]]
        for a, b in PAIRINGS:
            p = dict(pairs[l[a + "_" + b + "_pair_id"]])
            for k in [
                "position_separation",
                "difference_variance_joint",
                "independent_variance_joint",
            ]:
                p[k] = float(p[k])
            for k in ["a_inside", "b_inside"]:
                p[k] = flag(p[k])
            for kind in ["all", e["event_kind"]]:
                grouped[e["variant"], kind, a + "_" + b].append(p)
    summary = {
        (r["variant"], r["event_kind"], r["pairing"]): r
        for r in read_rows(args.results / "summary.csv")
        if r["cohort"] == "all"
        and r["population"] == "raw_zero_loss"
        and r["anchor_mode"] == "all"
        and float(r["agreement_threshold_m"]) == THRESHOLD
    }
    series = []
    for arm, kind, (a, b) in product(["epoch", "curved"], ["all", *KINDS], PAIRINGS):
        pairing = a + "_" + b
        rr = grouped[arm, kind, pairing]
        s = summary[arm, kind, pairing]
        assert len(rr) == int(s["records"])
        for bin_name in BINS:
            n = sum(category(p, THRESHOLD) == bin_name for p in rr)
            assert n == int(s[bin_name])
            series.append(
                dict(
                    variant=arm,
                    event_kind=kind,
                    pairing=pairing,
                    agreement_threshold_m=THRESHOLD,
                    family="position_agreement",
                    category=bin_name,
                    records=len(rr),
                    count=n,
                    percent=100 * n / len(rr) if rr else 0.0,
                    value_m="",
                )
            )
        for name, field, summary_field in [
            (
                "shared_information",
                "difference_variance_joint",
                "pooled_difference_sigma",
            ),
            (
                "independent_inputs",
                "independent_variance_joint",
                "pooled_independent_sigma",
            ),
        ]:
            value = math.sqrt(sum(p[field] for p in rr) / len(rr)) if rr else ""
            assert (value == "" and s[summary_field] == "") or math.isclose(
                value, float(s[summary_field]), abs_tol=1e-12
            )
            series.append(
                dict(
                    variant=arm,
                    event_kind=kind,
                    pairing=pairing,
                    agreement_threshold_m=THRESHOLD,
                    family="partial_difference_sigma",
                    category=name,
                    records=len(rr),
                    count="",
                    percent="",
                    value_m=value,
                )
            )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "speed-probe-shared-information",
        }
    )
    fig, axes = plt.subplots(2, 3, figsize=(12, 7.4))
    fig.subplots_adjust(
        left=0.12, right=0.99, bottom=0.17, top=0.82, wspace=0.55, hspace=0.55
    )
    colors = ["#238b45", "#3182bd", "#e6550d", "#756bb1", "#bdbdbd"]
    names = [
        "Agree; both within .10 m of truth",
        "Agree; A inside, B outside",
        "Agree; A outside, B inside",
        "Agree; both outside .10 m",
        "Position difference > .05 m",
    ]
    for j, (a, b) in enumerate(PAIRINGS):
        pairing = a + "_" + b
        ax = axes[0, j]
        left = [0.0, 0.0]
        for bin_name, label, color in zip(BINS, names, colors):
            rr = [
                next(
                    r
                    for r in series
                    if (
                        r["variant"],
                        r["event_kind"],
                        r["pairing"],
                        r["family"],
                        r["category"],
                    )
                    == (arm, "all", pairing, "position_agreement", bin_name)
                )
                for arm in ["epoch", "curved"]
            ]
            widths = [r["percent"] for r in rr]
            ax.barh([0, 1], widths, left=left, height=0.5, color=color, label=label)
            for y, start, width, r in zip([0, 1], left, widths, rr):
                if width >= 6:
                    ax.text(
                        start + width / 2,
                        y,
                        str(r["count"]),
                        ha="center",
                        va="center",
                        color="white" if color != "#bdbdbd" else "#222222",
                        fontsize=9,
                    )
            left = [x + y for x, y in zip(left, widths)]
        ax.set_yticks([0, 1], ["Baseline (N=197)", "Curved (N=353)"])
        ax.invert_yaxis()
        ax.set_xlim(0, 100)
        ax.set_title(
            f"A={a.title()}, B={b.title()}\nAll 550 raw-zero losses", fontsize=10
        )
        ax.set_xlabel("Position agreement <= .05 m (%)")
        ax.spines[["top", "right"]].set_visible(False)
        ax = axes[1, j]
        maximum = 0.0
        for i, (name, label, color) in enumerate(
            [
                ("shared_information", "Shared information model", "#3182bd"),
                ("independent_inputs", "Independent input assumption", "#e6550d"),
            ]
        ):
            values = [
                100
                * next(
                    float(r["value_m"])
                    for r in series
                    if (
                        r["variant"],
                        r["event_kind"],
                        r["pairing"],
                        r["family"],
                        r["category"],
                    )
                    == (arm, "all", pairing, "partial_difference_sigma", name)
                )
                for arm in ["epoch", "curved"]
            ]
            yy = [y + (i - 0.5) * 0.28 for y in [0, 1]]
            ax.barh(yy, values, height=0.25, color=color, label=label)
            for y, value in zip(yy, values):
                ax.annotate(
                    f"{value:.2f}",
                    (value, y),
                    xytext=(3, 0),
                    textcoords="offset points",
                    va="center",
                    fontsize=8,
                )
            maximum = max(maximum, *values)
        ax.set_xlim(0, max(maximum, 1) * 1.25)
        ax.set_yticks([0, 1], ["Baseline (N=197)", "Curved (N=353)"])
        ax.invert_yaxis()
        ax.set_xlabel("Pooled model sigma of A-B, per axis (cm)")
        ax.set_title("Noise + anchor model of position difference", fontsize=10)
        ax.spines[["top", "right"]].set_visible(False)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.995),
        ncol=3,
        fontsize=8,
    )
    fig.supxlabel(
        "Bottom: blue = shared information; orange = independent input assumption. Both use every loss row.\n"
        "Position agreement is not position accuracy. Partial-model covariance excludes drift and nonlinear transport.",
        fontsize=9,
    )
    args.output.mkdir(parents=True)
    save_rows(args.output / "shared-information-comparisons.csv", series)
    svg = args.output / "shared-information-comparisons.svg"
    fig.savefig(svg, metadata={"Date": None}, bbox_inches="tight")
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(
        args.output / "shared-information-comparisons.png",
        dpi=160,
        bbox_inches="tight",
        metadata={"Software": "rust_robotics speed-probe-shared-information"},
    )
    plt.close(fig)
    provenance = dict(
        input_sha256={
            n: digest(args.results / n)
            for n in [
                "accounting.csv",
                "events.csv",
                "inputs.csv",
                "anchor-weights.csv",
                "pairs.csv",
                "loss-links.csv",
                "summary.csv",
                "sources.json",
            ]
        },
        script_sha256=digest(Path(__file__)),
        analysis_sha256=digest(analyzer),
        matplotlib_version=matplotlib.__version__,
        numeric_rows=len(series),
        agreement_threshold_m=THRESHOLD,
        truth_position_radius_m=0.1,
        svg_normalization="strip trailing whitespace from generated XML lines",
    )
    (args.output / "shared-information-comparisons-sources.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(json.dumps(dict(numeric_rows=len(series), panels=6), indent=2))


if __name__ == "__main__":
    main()
