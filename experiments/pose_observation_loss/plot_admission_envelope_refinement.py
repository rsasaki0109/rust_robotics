#!/usr/bin/env python3
"""Partition every prior-unresolved age-.5 envelope, with explicit denominators."""

import argparse
import json
from pathlib import Path

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
        digest(Path(__file__).with_name("analyze_admission_envelope_refinement.py"))
        == source["analysis"]["script_sha256"]
    )
    rows = [
        r
        for r in read_rows(args.results / "summary.csv")
        if (r["cohort"], r["subset"], r["group"]) == ("all", "age_0p5", "all")
        and r["envelope"] != "central"
    ]
    assert len(rows) == 16
    states = ["new_permit", "new_blocked", "mixed", "unresolved"]
    groups = [
        (path, rule)
        for path in ["straight", "commanded"]
        for rule in ["noise_envelope", "noise_anchor_envelope"]
    ]
    series = []
    for variant, expected in [("epoch", 415), ("curved", 461)]:
        for kind in ["actual", "preview"]:
            for path, rule in groups:
                row = next(
                    r
                    for r in rows
                    if (r["variant"], r["input_kind"], r["path"], r["envelope"])
                    == (variant, kind, path, rule)
                )
                n = int(row["prior_unresolved"])
                assert int(row["checkpoints"]) == expected
                assert sum(int(row[s]) for s in states) == n
                for state in states:
                    count = int(row[state])
                    series.append(
                        dict(
                            variant=variant,
                            input_kind=kind,
                            path=path,
                            envelope=rule,
                            checkpoints=expected,
                            prior_unresolved=n,
                            cases=int(row["cases"]),
                            state=state,
                            count=count,
                            percent=100 * count / n if n else 0.0,
                        )
                    )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "admission-envelope-refinement",
        }
    )
    fig, axes = plt.subplots(2, 2, figsize=(12, 6.5), sharex=True, layout="constrained")
    colors = ["#238b45", "#636363", "#fdb863", "#9ecae1"]
    legends = [
        "Proved permit",
        "Proved blocked",
        "Both outcomes witnessed",
        "Search unfinished",
    ]
    for i, (variant, arm) in enumerate(
        [("epoch", "Baseline log"), ("curved", "Curved-terminal log")]
    ):
        for j, (kind, input_label) in enumerate(
            [("actual", "recovery input"), ("preview", "preview input")]
        ):
            ax = axes[i, j]
            left = [0.0] * len(groups)
            labels = []
            for path, rule in groups:
                record = next(
                    r
                    for r in series
                    if (r["variant"], r["input_kind"], r["path"], r["envelope"])
                    == (variant, kind, path, rule)
                )
                model = "noise" if rule == "noise_envelope" else "noise + anchor"
                labels.append(
                    f"{'Line' if path == 'straight' else 'Arc'} / {model}"
                    f"\nN={record['prior_unresolved']}"
                )
            for state, color, name in zip(states, colors, legends):
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
                                "white"
                                if state in {"new_permit", "new_blocked"}
                                else "#222222"
                            ),
                        )
                left = [start + width for start, width in zip(left, widths)]
            ax.set_yticks(range(len(groups)), labels, fontsize=8)
            ax.invert_yaxis()
            ax.set_xlim(0, 100)
            ax.set_xticks([0, 25, 50, 75, 100])
            ax.set_title(f"{arm} / {input_label}", fontsize=10, loc="left")
            ax.spines[["top", "right"]].set_visible(False)
            if i == 1:
                ax.set_xlabel("Prior-unresolved checkpoints (%); labels are counts")
    fig.suptitle(
        "Same pose envelopes and margin: refinement at recorded age-0.5 s checkpoints",
        fontsize=12,
    )
    handles, names = axes[0, 0].get_legend_handles_labels()
    fig.legend(
        handles,
        names,
        ncol=2,
        frameon=False,
        loc="lower center",
        bbox_to_anchor=(0.57, -0.085),
    )
    args.output.mkdir(parents=True)
    numeric = args.output / "envelope-refinement.csv"
    save_rows(numeric, series)
    svg = args.output / "envelope-refinement.svg"
    fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(args.output / "envelope-refinement.png", dpi=160, bbox_inches="tight")
    plt.close(fig)
    fingerprints = dict(
        script_sha256=digest(Path(__file__)),
        inputs_sha256={
            n: digest(args.results / n) for n in ["summary.csv", "sources.json"]
        },
        artifacts_sha256={p.name: digest(p) for p in [numeric, svg]},
        matplotlib_version=matplotlib.__version__,
    )
    (args.output / "envelope-refinement-sources.json").write_text(
        json.dumps(fingerprints, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
