#!/usr/bin/env python3
"""Plot full agreement spans and covariance of temporal means at .05 m."""

import argparse
from itertools import product
import json
import math
from pathlib import Path
import statistics

from analyze_epoch_preview import digest, read_rows
from analyze_speed_probe_shared_information import BINS, PAIRINGS
from analyze_speed_probe_starts import KINDS
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

RULE = "0p05"
MEANS = {
    "A: retained covariance": "a_mean_variance",
    "A: independent times": "a_time_independent_variance",
    "B: retained covariance": "b_mean_variance",
    "B: independent times": "b_time_independent_variance",
}
DIFFERENCES = {
    "Retained covariance": "mean_difference_variance",
    "Independent times; same-time A/B covariance": "time_independent_difference_variance",
    "Independent inputs and times": "independent_inputs_and_time_variance",
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    source = json.loads((args.results / "sources.json").read_text())
    analyzer = Path(__file__).with_name("analyze_speed_probe_temporal_information.py")
    assert digest(analyzer) == source["analysis"]["script_sha256"]
    events = {r["event_id"]: r for r in read_rows(args.results / "events.csv")}
    spans = [r for r in read_rows(args.results / "spans.csv") if r["rule"] == RULE]
    blocks = [r for r in read_rows(args.results / "blocks.csv") if r["rule"] == RULE]
    span_summary, block_summary = [
        {
            (r["variant"], r["event_kind"], r["pairing"], r["label"]): r
            for r in read_rows(args.results / (name + ".csv"))
            if r["cohort"] == "all" and r["anchor_mode"] == "all" and r["rule"] == RULE
        }
        for name in ["span-summary", "block-summary"]
    ]
    series = []
    for arm, kind, (a, b) in product(["epoch", "curved"], ["all", *KINDS], PAIRINGS):
        pairing = a + "_" + b

        def select(rows):
            return [
                r
                for r in rows
                if r["variant"] == arm
                and r["pairing"] == pairing
                and (
                    kind == "all"
                    or any(
                        events[e]["event_kind"] == kind
                        for e in r["source_event_ids"].split("+")
                    )
                )
            ]

        ss, bb = select(spans), select(blocks)
        common = dict(
            variant=arm,
            event_kind=kind,
            pairing=pairing,
            rule=RULE,
            agreement_threshold_m=0.05,
            spans=len(ss),
            blocks=len(bb),
            states=sum(int(r["states"]) for r in ss),
            singleton_spans=sum(int(r["states"]) == 1 for r in ss),
            audit_truncated_spans=sum(
                r["left_boundary"] == "audit_start"
                or r["right_boundary"] == "audit_end"
                for r in ss
            ),
        )
        for label in BINS:
            count = sum(r["label"] == label for r in ss)
            sr = span_summary[arm, kind, pairing, label]
            assert count == int(sr["records"])
            assert sum(int(r["states"]) for r in ss if r["label"] == label) == int(
                sr["states"]
            )
            series.append(
                dict(
                    **common,
                    family="span_partition",
                    category=label,
                    count=count,
                    percent=100 * count / len(ss) if ss else 0.0,
                    value_m="",
                )
            )
        for family, fields in [
            ("temporal_mean_sigma", MEANS),
            ("temporal_difference_sigma", DIFFERENCES),
        ]:
            for name, field in fields.items():
                value = (
                    math.sqrt(statistics.fmean(float(r[field]) for r in bb))
                    if bb
                    else ""
                )
                summaries = [block_summary[arm, kind, pairing, label] for label in BINS]
                total = sum(int(r["records"]) for r in summaries)
                assert total == len(bb)
                expected = (
                    math.sqrt(
                        sum(
                            int(r["records"]) * float(r["pooled_sigma_" + field]) ** 2
                            for r in summaries
                            if int(r["records"])
                        )
                        / total
                    )
                    if total
                    else ""
                )
                assert value == expected == "" or math.isclose(
                    value, expected, abs_tol=1e-12
                )
                series.append(
                    dict(
                        **common,
                        family=family,
                        category=name,
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
            "svg.hashsalt": "speed-probe-temporal-information",
        }
    )
    fig, axes = plt.subplots(3, 3, figsize=(13.5, 11.4))
    fig.subplots_adjust(
        left=0.12, right=0.99, bottom=0.1, top=0.84, wspace=0.39, hspace=0.66
    )
    colors = ["#238b45", "#3182bd", "#e6550d", "#756bb1", "#bdbdbd"]
    names = [
        "Agree; both within .10 m of truth",
        "Agree; A inside, B outside",
        "Agree; A outside, B inside",
        "Agree; both outside .10 m",
        "Position difference > .05 m",
    ]
    handles = []
    for j, (a, b) in enumerate(PAIRINGS):
        pairing = a + "_" + b

        def one(arm, family, category):
            return next(
                r
                for r in series
                if r["variant"] == arm
                and r["event_kind"] == "all"
                and r["pairing"] == pairing
                and r["family"] == family
                and r["category"] == category
            )

        ax = axes[0, j]
        left = [0.0, 0.0]
        for label, name, color in zip(BINS, names, colors):
            values = [
                one(arm, "span_partition", label)["percent"]
                for arm in ["epoch", "curved"]
            ]
            bars = ax.barh([0, 1], values, left=left, color=color, label=name)
            if j == 0:
                handles.append(bars[0])
            left = [x + y for x, y in zip(left, values)]
        ax.set_yticks(
            [0, 1],
            [
                f'{name}\n{one(arm,"span_partition",BINS[0])["spans"]:,} spans / '
                f'{one(arm,"span_partition",BINS[0])["states"]:,} states'
                for arm, name in [("epoch", "Baseline"), ("curved", "Curved")]
            ],
        )
        ax.invert_yaxis()
        ax.set_xlim(0, 100)
        ax.set_xlabel("Percent of all agreement spans")
        ax.set_title(f"A: {a.title()} / B: {b.title()}", pad=12)
        for row, (family, fields, palette) in enumerate(
            [
                (
                    "temporal_mean_sigma",
                    MEANS,
                    ["#2166ac", "#92c5de", "#b35806", "#fdb863"],
                ),
                (
                    "temporal_difference_sigma",
                    DIFFERENCES,
                    ["#2166ac", "#777777", "#e08214"],
                ),
            ],
            1,
        ):
            ax = axes[row, j]
            width = 0.8 / len(fields)
            for k, (name, color) in enumerate(zip(fields, palette)):
                values = [
                    100 * one(arm, family, name)["value_m"]
                    for arm in ["epoch", "curved"]
                ]
                positions = [i - 0.4 + width * (k + 0.5) for i in [0, 1]]
                ax.bar(positions, values, width=width, color=color, label=name)
            cap = max(
                r["value_m"]
                for r in series
                if r["family"] == family and r["event_kind"] == "all"
            )
            ax.set_ylim(0, 120 * cap)
            ax.set_xticks([0, 1], ["Baseline", "Curved"])
            ax.set_title(
                (
                    "Temporal mean: input sigmas"
                    if row == 1
                    else "Temporal mean: difference sigma"
                ),
                pad=10,
            )
            if j == 0:
                ax.set_ylabel("Partial-model sigma per axis (cm)")
            ax.grid(axis="y", alpha=0.2)
            ax.set_axisbelow(True)
            if j == 0:
                ax.legend(
                    fontsize=7,
                    loc="upper left",
                    bbox_to_anchor=(0, -0.18),
                    frameon=False,
                )
    fig.legend(
        handles,
        names,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.995),
        ncol=2,
        frameon=False,
        fontsize=9,
    )
    fig.suptitle(
        "All audited chains at .05 m agreement; temporal models retain every information block",
        y=0.89,
        fontsize=12,
    )
    fig.text(
        0.12,
        0.015,
        "Top: full span partition, including singletons and audit-truncated spans. Middle/bottom: one variance weight per block.\n"
        "Partial noise + anchor model; excludes drift/transport. Event-kind unions overlap. Recorded control and held goals remain fixed.",
        fontsize=9,
    )
    args.output.mkdir(parents=True)
    stem = "temporal-information-comparisons"
    save_rows(args.output / (stem + ".csv"), series)
    svg = args.output / (stem + ".svg")
    fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(
        args.output / (stem + ".png"),
        dpi=150,
        bbox_inches="tight",
        metadata={"Software": "Matplotlib"},
    )
    plt.close(fig)
    provenance = dict(
        script_sha256=digest(Path(__file__)),
        input_sha256={
            n: digest(args.results / n)
            for n in [
                "events.csv",
                "spans.csv",
                "blocks.csv",
                "span-summary.csv",
                "block-summary.csv",
                "sources.json",
            ]
        },
        rule=RULE,
        agreement_threshold_m=0.05,
        numeric_rows=len(series),
        weighting="one weight per information block; whole-span event-kind unions overlap",
    )
    (args.output / (stem + "-sources.json")).write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(json.dumps(dict(numeric_rows=len(series), output=str(args.output)), indent=2))


if __name__ == "__main__":
    main()
