#!/usr/bin/env python3
"""Plot every opportunity's error-change/censor partition in both recorded arms."""

import argparse
import json
from pathlib import Path

from analyze_correction_centre_transitions import CHANGES, KINDS, OFFSETS
from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output directory must be new")
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        digest(Path(__file__).with_name("analyze_correction_centre_transitions.py"))
        == source["analysis"]["script_sha256"]
    )
    events = read_rows(args.results / "events.csv")
    categories = [k for k in KINDS if any(e["event_kind"] == k for e in events)]
    rows = [
        r
        for r in read_rows(args.results / "pair-summary.csv")
        if (r["cohort"], r["route_group"], r["transition_group"])
        == ("all", "all", "all")
        and r["event_kind"] in categories
    ]
    phases = list(OFFSETS)[1:]
    groups = [(category, phase) for category in categories for phase in phases]
    series = []
    for variant in ["epoch", "curved"]:
        for kind in ["actual", "preview"]:
            for category, phase in groups:
                row = next(
                    r
                    for r in rows
                    if (r["variant"], r["input_kind"], r["event_kind"], r["phase"])
                    == (variant, kind, category, phase)
                )
                n = int(row["events"])
                assert n == sum(
                    (e["variant"], e["event_kind"]) == (variant, category)
                    for e in events
                )
                assert sum(int(row[k]) for k in CHANGES) == n
                for change in CHANGES:
                    count = int(row[change])
                    series.append(
                        dict(
                            variant=variant,
                            input_kind=kind,
                            event_kind=category,
                            phase=phase,
                            events=n,
                            cases=int(row["cases"]),
                            paired_available=int(row["paired_available"]),
                            change=change,
                            count=count,
                            percent=100 * count / n if n else 0.0,
                        )
                    )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "correction-centre-transitions",
        }
    )
    fig, axes = plt.subplots(
        2, 2, figsize=(12, 3 + 0.7 * len(groups)), sharex=True, layout="constrained"
    )
    colors = ["#238b45", "#e6550d", "#a6a6a6", "#9ecae1"]
    names = [
        "Lower position error",
        "Higher position error",
        "Within 2e-8 m",
        "Post sample unavailable",
    ]
    labels = {
        "received": "Received",
        "missed": "Missed",
        "precision_only": "Precision",
        "received_and_precision": "Received + precision",
        "missed_and_precision": "Missed + precision",
    }
    for i, (variant, arm) in enumerate(
        [("epoch", "Baseline log"), ("curved", "Curved-terminal log")]
    ):
        for j, (kind, input_label) in enumerate(
            [("actual", "recovery input"), ("preview", "preview input")]
        ):
            ax = axes[i, j]
            left = [0.0] * len(groups)
            ticks = []
            for category, phase in groups:
                row = next(
                    r
                    for r in series
                    if (r["variant"], r["input_kind"], r["event_kind"], r["phase"])
                    == (variant, kind, category, phase)
                )
                ticks.append(
                    f"{labels[category]} / +{OFFSETS[phase] * .02:g} s (N={row['events']})"
                )
            for change, color, name in zip(CHANGES, colors, names):
                chosen = [
                    next(
                        r
                        for r in series
                        if (
                            r["variant"],
                            r["input_kind"],
                            r["event_kind"],
                            r["phase"],
                            r["change"],
                        )
                        == (variant, kind, category, phase, change)
                    )
                    for category, phase in groups
                ]
                widths = [r["percent"] for r in chosen]
                ax.barh(
                    range(len(groups)),
                    widths,
                    left=left,
                    height=0.68,
                    color=color,
                    label=name,
                )
                for y, (start, width, row) in enumerate(zip(left, widths, chosen)):
                    if width >= 4:
                        ax.text(
                            start + width / 2,
                            y,
                            str(row["count"]),
                            ha="center",
                            va="center",
                            fontsize=8,
                            color=(
                                "white" if change in {"better", "worse"} else "#222222"
                            ),
                        )
                left = [a + b for a, b in zip(left, widths)]
            for boundary in range(1, len(categories)):
                ax.axhline(boundary * len(phases) - 0.5, color="white", linewidth=3)
            ax.set_yticks(range(len(groups)), ticks, fontsize=8)
            ax.invert_yaxis()
            ax.set_xlim(0, 100)
            ax.set_xticks([0, 25, 50, 75, 100])
            ax.set_title(f"{arm} / {input_label}", fontsize=10, loc="left")
            ax.spines[["top", "right"]].set_visible(False)
            if i == 1:
                ax.set_xlabel(
                    "All opportunities (%); errors paired with the pre-event input"
                )
    fig.suptitle(
        "Timeout trajectories: recorded position-error changes around corrections",
        fontsize=12,
    )
    handles, legend_labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(
        handles,
        legend_labels,
        ncol=2,
        frameon=False,
        loc="lower center",
        bbox_to_anchor=(0.57, -0.06),
    )
    args.output.mkdir(parents=True)
    numeric = args.output / "correction-error-changes.csv"
    save_rows(numeric, series)
    svg = args.output / "correction-error-changes.svg"
    fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(
        args.output / "correction-error-changes.png", dpi=160, bbox_inches="tight"
    )
    plt.close(fig)
    fingerprints = dict(
        script_sha256=digest(Path(__file__)),
        inputs_sha256={
            n: digest(args.results / n)
            for n in ["pair-summary.csv", "events.csv", "sources.json"]
        },
        artifacts_sha256={p.name: digest(p) for p in [numeric, svg]},
        matplotlib_version=matplotlib.__version__,
    )
    (args.output / "correction-error-changes-sources.json").write_text(
        json.dumps(fingerprints, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
