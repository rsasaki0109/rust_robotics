#!/usr/bin/env python3
"""Plot the fixed, previously reported lost-hold example from audited records."""

import argparse
import csv
import json
import math
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_terminal_arc_divergence import PRIOR
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def wait_spans(track, field):
    spans = []
    for before, row in zip(track, track[1:]):
        if row[field] != "true":
            continue
        start, end = (float(r["time_s"]) for r in [before, row])
        if spans and abs(spans[-1]["end_s"] - start) < 1e-8:
            spans[-1]["end_s"] = end
        else:
            spans.append(dict(start_s=start, end_s=end))
    return spans


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--diagnosis", type=Path, required=True)
    parser.add_argument("--raw-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output directory must be new")
    if not __debug__:
        parser.error("Run without Python -O")
    sources = json.loads((args.diagnosis / "sources.json").read_text())
    assert digest(PRIOR / "cases.csv") == sources["prior_sha256"]["cases.csv"]
    cohort = "new-white"
    identity = dict(
        cohort=cohort,
        mode="blackout",
        seed="274",
        scenario="maze_50x50",
        planner="Theta*",
    )
    forks = [
        r
        for r in read_rows(args.diagnosis / "forks.csv")
        if all(r[k] == v for k, v in identity.items())
    ]
    assert len(forks) == 1 and forks[0]["outcome"] == "held/timeout"
    fork_time = float(forks[0]["first_action_s"])
    pinned = sources["cohorts"][cohort]
    run = args.raw_root / cohort if args.raw_root else Path(pinned["path"])
    series, spans, cases, raw_hashes = [], [], {}, {}
    for variant in ["epoch", "curved"]:
        name = f"{variant}-blackout-274/trajectories.csv"
        path = run / name
        raw_hashes[name] = digest(path)
        assert raw_hashes[name] == pinned["raw_sha256"][name]
        with path.open() as stream:
            track = [
                r
                for r in csv.DictReader(stream)
                if r["scenario"] == identity["scenario"]
                and r["planner"] == identity["planner"]
            ]
        chosen = {0, len(track) - 1}
        chosen.update(range(0, len(track), 10))
        chosen.update(
            i
            for i in range(1, len(track))
            if any(
                track[i][field] != track[i - 1][field]
                for field in ["guard_active", "safety_paused"]
            )
        )
        for index in sorted(chosen):
            row = track[index]
            series.append(
                dict(
                    variant=variant,
                    time_s=row["time_s"],
                    goal_distance=math.hypot(
                        float(row["x"]) - 35, float(row["y"]) - 45
                    ),
                    speed=row["v"],
                    guard=row["guard_active"],
                    loss_wait=row["safety_paused"],
                )
            )
        for field in ["guard_active", "safety_paused"]:
            spans.extend(
                dict(variant=variant, field=field, **span)
                for span in wait_spans(track, field)
            )
        case = [
            r
            for r in read_rows(PRIOR / "cases.csv")
            if r["variant"] == variant and all(r[k] == v for k, v in identity.items())
        ]
        assert len(case) == 1
        cases[variant] = case[0]
        assert abs(float(case[0]["goal_distance"]) - series[-1]["goal_distance"]) < 2e-8
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "svg.fonttype": "none",
            "svg.hashsalt": "terminal-arc-divergence",
        }
    )
    fig, axes = plt.subplots(
        2,
        1,
        figsize=(9, 6),
        sharex=True,
        gridspec_kw={"height_ratios": [2, 1]},
        layout="constrained",
    )
    colors = {"epoch": "#2166ac", "curved": "#d95f02"}
    labels = {"epoch": "Straight terminal", "curved": "Curved terminal"}
    for variant in colors:
        rows = [r for r in series if r["variant"] == variant]
        axes[0].plot(
            [float(r["time_s"]) for r in rows],
            [r["goal_distance"] for r in rows],
            color=colors[variant],
            label=labels[variant],
            linewidth=1.5,
        )
        axes[0].scatter(
            float(rows[-1]["time_s"]),
            rows[-1]["goal_distance"],
            color=colors[variant],
            s=24,
        )
        axes[0].annotate(
            cases[variant]["status"],
            (float(rows[-1]["time_s"]), rows[-1]["goal_distance"]),
            xytext=(-5, 8),
            textcoords="offset points",
            ha="right",
            color=colors[variant],
        )
    axes[0].axhline(
        0.25, color="#555555", linestyle=":", linewidth=0.8, label="Goal radius 0.25 m"
    )
    axes[0].set_ylabel("True goal distance (m)")
    axes[0].set_title("New / white, blackout, seed 274, maze / Theta*")
    axes[0].legend(loc="upper right", frameon=False)
    bands = [
        ("epoch", "guard_active"),
        ("epoch", "safety_paused"),
        ("curved", "guard_active"),
        ("curved", "safety_paused"),
    ]
    for index, (variant, field) in enumerate(bands):
        intervals = [
            (s["start_s"], s["end_s"] - s["start_s"])
            for s in spans
            if s["variant"] == variant and s["field"] == field
        ]
        axes[1].broken_barh(
            intervals,
            (index - 0.3, 0.6),
            color=colors[variant],
            alpha=1 if field == "guard_active" else 0.45,
        )
    axes[1].set_yticks(
        range(4),
        [
            "Straight: guard",
            "Straight: loss wait",
            "Curved: guard",
            "Curved: loss wait",
        ],
    )
    axes[1].set_ylim(3.6, -0.6)
    axes[1].set_xlabel("Recorded time (s)")
    axes[1].set_xlim(0, 182)
    for ax in axes:
        ax.axvline(fork_time, color="#555555", linestyle="--", linewidth=0.8)
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="x", alpha=0.2)
    fig.suptitle(
        f"Recorded lost hold: first different guard/loss action at {fork_time:.2f} s",
        fontsize=12,
    )
    args.output.mkdir(parents=True)
    save_rows(args.output / "lost-hold-series.csv", series)
    save_rows(args.output / "lost-hold-waits.csv", spans)
    svg = args.output / "lost-hold-example.svg"
    fig.savefig(svg, metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(args.output / "lost-hold-example.png", dpi=150)
    plt.close(fig)
    info = dict(
        script_sha256=digest(Path(__file__)),
        identity=identity,
        raw_sha256=raw_hashes,
        diagnosis_sha256={
            name: digest(args.diagnosis / name)
            for name in ["sources.json", "forks.csv"]
        },
        artifacts_sha256={
            name: digest(args.output / name)
            for name in [
                "lost-hold-series.csv",
                "lost-hold-waits.csv",
                "lost-hold-example.svg",
            ]
        },
        matplotlib_version=matplotlib.__version__,
    )
    (args.output / "lost-hold-sources.json").write_text(
        json.dumps(info, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
