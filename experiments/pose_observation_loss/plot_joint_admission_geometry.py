#!/usr/bin/env python3
"""Plot continuous readiness and fresh-checkpoint geometric obstructions."""

import argparse
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import MASKS
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
    readiness = read_rows(args.results / "readiness.csv")
    geometry = [
        r
        for r in read_rows(args.results / "geometry.csv")
        if r["pose_kind"] == "truth" and "age_0p5" in r["reasons"].split("+")
    ]
    series = []
    for variant in ["epoch", "curved"]:
        cases = [r for r in readiness if r["variant"] == variant]
        assert len(cases) == 184
        counts = {
            name: sum(float(r[name + "_max_ready_s"]) >= 1 - 1e-8 for r in cases)
            for name in MASKS
        }
        assert all(
            counts["actual"] <= n <= counts["base_only"] for n in counts.values()
        )
        for name, count in counts.items():
            series.append(
                dict(
                    panel="readiness",
                    variant=variant,
                    measure=name,
                    count=count,
                    total=len(cases),
                    percent=100 * count / len(cases),
                )
            )
        points = [r for r in geometry if r["variant"] == variant]
        assert points
        for name, field, invert in [
            ("start_below_floor", "truth_center_below_floor", False),
            ("recovery_blocked", "recovery_clear", True),
            ("speed_chord_blocked", "speed_geometry_clear", True),
        ]:
            count = sum((r[field] == "True") != invert for r in points)
            series.append(
                dict(
                    panel="geometry",
                    variant=variant,
                    measure=name,
                    count=count,
                    total=len(points),
                    percent=100 * count / len(points),
                )
            )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "joint-admission-geometry",
        }
    )
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8), layout="constrained")
    masks = list(MASKS)
    names = [
        "Actual requirements",
        "Omit recovery line",
        "Omit positive command",
        "Omit both raw gates",
        "Omit stop/terminal preview",
        "Only stopped/fresh/samples",
    ]
    measures = ["start_below_floor", "recovery_blocked", "speed_chord_blocked"]
    geometry_names = [
        "True start below\nzero-age margin floor",
        "True recovery line\nblocked by current margin",
        "True first speed chord\nblocked by current margin",
    ]
    colors = {"epoch": "#2166ac", "curved": "#d95f02"}
    labels = {"epoch": "Straight terminal", "curved": "Curved terminal"}
    for variant, offset in [("epoch", -0.18), ("curved", 0.18)]:
        for ax, panel, keys, percent in [
            (axes[0], "readiness", masks, False),
            (axes[1], "geometry", measures, True),
        ]:
            rows = {
                r["measure"]: r
                for r in series
                if r["panel"] == panel and r["variant"] == variant
            }
            values = [rows[k]["percent" if percent else "count"] for k in keys]
            ax.barh(
                [i + offset for i in range(len(keys))],
                values,
                height=0.32,
                color=colors[variant],
                label=labels[variant],
            )
            for i, value in enumerate(values):
                text = f"{value:.1f}%" if percent else str(value)
                ax.text(
                    value + (0.8 if percent else 0.6),
                    i + offset,
                    text,
                    va="center",
                    fontsize=8,
                )
    axes[0].set_yticks(range(len(masks)), names)
    axes[0].set_xlim(
        0, max(r["count"] for r in series if r["panel"] == "readiness") * 1.2 + 1
    )
    axes[0].set_xlabel(
        "Timeout cases with a >=1 s ready run\ninside the recorded final 30 s (184 per arm)"
    )
    axes[0].set_title("Recorded requirement masks")
    axes[1].set_yticks(range(3), geometry_names)
    axes[1].set_xlim(0, 113)
    axes[1].set_xlabel(
        "Share of age-0.5 s joint checkpoints (%)\nRepeated observations, not independent cases"
    )
    axes[1].set_title("True geometry at fresh stationary checkpoints")
    for ax in axes:
        ax.invert_yaxis()
        ax.spines[["top", "right"]].set_visible(False)
        ax.set_axisbelow(True)
        ax.grid(axis="x", alpha=0.2)
    axes[0].legend(
        loc="lower center", bbox_to_anchor=(0.5, -0.4), ncol=2, frameon=False
    )
    fig.suptitle(
        "Fixed recorded inputs: masks and pose substitutions are not policy rollouts",
        fontsize=11,
    )
    args.output.mkdir(parents=True)
    save_rows(args.output / "admission-limits.csv", series)
    svg = args.output / "admission-limits.svg"
    fig.savefig(svg, metadata={"Date": None})
    svg.write_text(
        "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
    )
    fig.savefig(args.output / "admission-limits.png", dpi=150)
    plt.close(fig)
    sources = dict(
        script_sha256=digest(Path(__file__)),
        inputs_sha256={
            name: digest(args.results / name)
            for name in ["readiness.csv", "geometry.csv", "sources.json"]
        },
        artifacts_sha256={
            name: digest(args.output / name)
            for name in ["admission-limits.csv", "admission-limits.svg"]
        },
        matplotlib_version=matplotlib.__version__,
    )
    (args.output / "admission-limits-sources.json").write_text(
        json.dumps(sources, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
