#!/usr/bin/env python3
"""Plot all frozen age-.5 three-way decisions by input, path, rule and arm."""

import argparse
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_model_admission_envelopes import RULES
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
        digest(Path(__file__).with_name("analyze_model_admission_envelopes.py"))
        == source["analysis"]["script_sha256"]
    )
    rows = [
        r
        for r in read_rows(args.results / "summary.csv")
        if (r["cohort"], r["subset"], r["group"]) == ("all", "age_0p5", "all")
    ]
    assert len(rows) == 24
    states = ["permit", "blocked", "unresolved"]
    series = []
    for variant, expected in [("epoch", 415), ("curved", 461)]:
        for kind in ["actual", "preview"]:
            for path in ["straight", "commanded"]:
                for rule in RULES:
                    row = next(
                        r
                        for r in rows
                        if (r["variant"], r["input_kind"], r["path"], r["envelope"])
                        == (variant, kind, path, rule)
                    )
                    n = int(row["checkpoints"])
                    assert n == expected and sum(int(row[s]) for s in states) == n
                    for state in states:
                        count = int(row[state])
                        series.append(
                            dict(
                                variant=variant,
                                input_kind=kind,
                                path=path,
                                envelope=rule,
                                checkpoints=n,
                                cases=int(row["cases"]),
                                state=state,
                                count=count,
                                percent=100 * count / n,
                            )
                        )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "model-admission-envelopes",
        }
    )
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5), sharex=True, layout="constrained")
    labels = {
        "central": "central",
        "noise_envelope": "noise",
        "noise_anchor_envelope": "noise + anchor",
    }
    colors = ["#238b45", "#a6a6a6", "#fdb863"]
    groups = [(path, rule) for path in ["straight", "commanded"] for rule in RULES]
    for i, (variant, arm, n) in enumerate(
        [("epoch", "Baseline log", 415), ("curved", "Curved-terminal log", 461)]
    ):
        for j, (kind, input_label) in enumerate(
            [("actual", "recovery input"), ("preview", "preview input")]
        ):
            ax = axes[i, j]
            left = [0.0] * len(groups)
            for state, color in zip(states, colors):
                chosen = [
                    next(
                        r
                        for r in series
                        if (
                            r["variant"],
                            r["input_kind"],
                            r["path"],
                            r["envelope"],
                            r["state"],
                        )
                        == (variant, kind, path, rule, state)
                    )
                    for path, rule in groups
                ]
                widths = [r["percent"] for r in chosen]
                ax.barh(
                    range(len(groups)),
                    widths,
                    left=left,
                    height=0.66,
                    color=color,
                    label=state.capitalize(),
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
                            color="white" if state == "permit" else "#222222",
                        )
                left = [start + width for start, width in zip(left, widths)]
            ax.set_yticks(
                range(len(groups)),
                [
                    f"{'Line' if path == 'straight' else 'Arc'} / {labels[rule]}"
                    for path, rule in groups
                ],
                fontsize=8,
            )
            ax.axhline(2.5, color="white", linewidth=3)
            ax.invert_yaxis()
            ax.set_xlim(0, 100)
            ax.set_xticks([0, 25, 50, 75, 100])
            ax.set_title(f"{arm} / {input_label} (N={n})", fontsize=10, loc="left")
            ax.spines[["top", "right"]].set_visible(False)
            if i == 1:
                ax.set_xlabel("Checkpoints (%); labels are counts")
    fig.suptitle(
        "Recorded age-0.5 s checkpoints: three-way admission at unchanged margin",
        fontsize=12,
    )
    handles, names = axes[0, 0].get_legend_handles_labels()
    fig.legend(
        handles,
        names,
        ncol=3,
        frameon=False,
        loc="lower center",
        bbox_to_anchor=(0.56, -0.04),
    )
    args.output.mkdir(parents=True)
    numeric = args.output / "admission-states.csv"
    save_rows(numeric, series)
    svg = args.output / "admission-states.svg"
    fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(args.output / "admission-states.png", dpi=160, bbox_inches="tight")
    plt.close(fig)
    fingerprints = dict(
        script_sha256=digest(Path(__file__)),
        inputs_sha256={
            n: digest(args.results / n) for n in ["summary.csv", "sources.json"]
        },
        artifacts_sha256={p.name: digest(p) for p in [numeric, svg]},
        matplotlib_version=matplotlib.__version__,
    )
    (args.output / "admission-states-sources.json").write_text(
        json.dumps(fingerprints, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
