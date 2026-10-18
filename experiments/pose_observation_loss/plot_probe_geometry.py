#!/usr/bin/env python3
"""Render the three preselected native geometric diagnostics reproducibly."""

import argparse
import json
from pathlib import Path
import platform

import analyze_probe_geometry as audit
import analyze_mean_speed_probe as pilot
import build_mean_speed_probe as build
import native_probe_geometry as native


def plot(data, output):
    data, output = Path(data).resolve(), build.fresh(output)
    sources = json.loads((data / "sources.json").read_text())
    for name, digest in sources["output_sha256"].items():
        build.require(build.sha(data / name) == digest, f"Changed plot input: {name}")
    build.require(
        sources["script_sha256"][Path(__file__).name] == build.sha(Path(__file__)),
        "Plot implementation changed",
    )
    records = [
        r
        for r in pilot.read(data / "representatives.csv", audit.REP_FIELDS)
        if pilot.truth(r["plot"])
    ]
    build.require(
        len(records) == 3
        and {r["mask"] for r in records} == set(audit.MASKS) - {"ready"},
        "Plot selection differs",
    )
    inputs = {
        r["id"]: r for r in audit.read_queries(data / "representative-inputs.tsv")
    }
    traces = {
        r["id"]: r
        for r in pilot.read(data / "representative-native.csv", native.TRACE_FIELDS)
    }
    witnesses = pilot.read(data / "representative-witnesses.csv", native.WITNESS_FIELDS)
    paths = pilot.read(data / "plot-paths.csv", native.PATH_FIELDS)
    obstacles = pilot.read(build.FIXTURES / "obstacles.csv")
    fixture_sha = build.sha(build.FIXTURES / "obstacles.csv")
    key = str((build.FIXTURES / "obstacles.csv").relative_to(build.REPO))
    build.require(sources["input_sha256"][key] == fixture_sha, "Plot obstacles changed")
    import matplotlib

    build.require(
        matplotlib.__version__ == "3.10.8"
        and platform.python_version_tuple()[:2] == ("3", "12"),
        "Use the pinned Python 3.12 / Matplotlib 3.10.8 replay profile",
    )
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Circle

    output.mkdir(parents=True)
    with plt.rc_context(
        {
            "svg.hashsalt": "mean-probe-geometry-v1",
            "font.family": "DejaVu Sans",
            "font.size": 9,
        }
    ):
        fig, axes = plt.subplots(1, 3, figsize=(15, 6))
        variants = (
            ("raw_curve", "#2166ac", "recorded yaw + steering"),
            ("mean_curve", "#e08214", "mean yaw + steering"),
            ("mean_straight", "#737373", "mean yaw + zero steering"),
        )
        for axis, record in zip(axes, records):
            ident = record["id"]
            query, trace = inputs[ident], traces[ident]
            x, y, margin = map(float, (query["x"], query["y"], query["margin"]))
            selected_witnesses = [w for w in witnesses if w["id"] == ident]
            selected_paths = [p for p in paths if p["id"] == ident]
            build.require(
                len(selected_paths) == 123 and selected_witnesses,
                "Plot witness/path coverage differs",
            )
            extent_x, extent_y = [x, float(record["guard_x"])], [
                y,
                float(record["guard_y"]),
            ]
            for variant, color, _ in variants:
                points = [p for p in selected_paths if p["variant"] == variant]
                build.require(
                    [int(p["step"]) for p in points] == list(range(41)),
                    "Plot path steps differ",
                )
                xx, yy = [float(p["x"]) for p in points], [
                    float(p["y"]) for p in points
                ]
                axis.plot(
                    xx,
                    yy,
                    color=color,
                    linewidth=1.8,
                    linestyle="--" if variant == "mean_straight" else "-",
                    zorder=3,
                )
                extent_x.extend(xx)
                extent_y.extend(yy)
            axis.plot(
                [x, float(trace["line_end_x"])],
                [y, float(trace["line_end_y"])],
                color="#b2182b",
                linewidth=3.4,
                zorder=4,
            )
            axis.scatter([x], [y], color="black", s=28, marker="o", zorder=6)
            axis.scatter(
                [float(record["guard_x"])],
                [float(record["guard_y"])],
                color="#762a83",
                s=60,
                marker="x",
                linewidth=1.8,
                zorder=6,
            )
            for witness in selected_witnesses:
                ox, oy = float(witness["obstacle_x"]), float(witness["obstacle_y"])
                extent_x.append(ox)
                extent_y.append(oy)
                axis.add_patch(
                    Circle(
                        (ox, oy),
                        margin,
                        fill=False,
                        color="#b2182b",
                        alpha=0.65,
                        linewidth=1,
                    )
                )
                axis.plot(
                    [ox, float(witness["closest_x"])],
                    [oy, float(witness["closest_y"])],
                    color="#b2182b",
                    linestyle=":",
                    linewidth=1.2,
                    zorder=4,
                )
                axis.scatter([ox], [oy], color="#b2182b", s=28, zorder=5)
            center_x, center_y = (min(extent_x) + max(extent_x)) / 2, (
                min(extent_y) + max(extent_y)
            ) / 2
            half = (
                max(max(extent_x) - min(extent_x), max(extent_y) - min(extent_y), 2.0)
                / 2
                + 0.35
            )
            visible = [
                o
                for o in obstacles
                if o["scenario"] == record["scenario"]
                and abs(float(o["x"]) - center_x) <= half
                and abs(float(o["y"]) - center_y) <= half
            ]
            axis.scatter(
                [float(o["x"]) for o in visible],
                [float(o["y"]) for o in visible],
                color="black",
                s=8,
                alpha=0.55,
                zorder=2,
            )
            axis.set(
                xlim=(center_x - half, center_x + half),
                ylim=(center_y - half, center_y + half),
                xlabel="x (model units)",
                ylabel="y (model units)",
            )
            axis.set_aspect("equal")
            axis.grid(alpha=0.15)
            axis.set_title(
                f"{record['mask']}\n{record['case_id']} / {record['planner']} / t={float(record['time_s']):.2f}s",
                fontsize=10,
            )
            label = (
                f"probe distance A/B/C: {float(trace['actual_distance']):.2f} / {float(trace['mean_yaw_distance']):.2f} / {float(trace['straight_distance']):.2f}\n"
                f"recovery clearance - margin: {float(record['recovery_gap']):+.4f}\n"
                f"stop / terminal release: {record['stop_gate']} / {record['terminal_gate']}"
            )
            axis.text(
                0.02,
                0.02,
                label,
                transform=axis.transAxes,
                fontsize=8,
                va="bottom",
                bbox=dict(facecolor="white", edgecolor="0.8", alpha=0.93, pad=4),
            )
        handles = [
            Line2D(
                [],
                [],
                color=c,
                linewidth=2,
                linestyle="--" if v == "mean_straight" else "-",
                label=label,
            )
            for v, c, label in variants
        ]
        handles.extend(
            (
                Line2D(
                    [],
                    [],
                    color="#b2182b",
                    linewidth=3,
                    label="original 0.6 recovery line / nearest witness",
                ),
                Line2D(
                    [],
                    [],
                    color="#762a83",
                    marker="x",
                    linestyle="",
                    label="guard pose (rounded)",
                ),
            )
        )
        fig.legend(
            handles=handles,
            loc="lower center",
            ncol=3,
            bbox_to_anchor=(0.5, 0.07),
            frameon=False,
            fontsize=9,
        )
        fig.suptitle(
            "First decision of each failure mask: static geometric queries at the same mean position",
            fontsize=12,
            y=0.97,
        )
        fig.text(
            0.5,
            0.035,
            "A/B/C share position, margin and map. Paths are queries, not traveled trajectories. Red circles show the original required margin.\nAll nearest ties are retained in the evidence; the purple guard-pose marker comes from rounded trajectory logs.",
            ha="center",
            fontsize=9,
        )
        fig.subplots_adjust(top=0.84, bottom=0.26, left=0.05, right=0.985, wspace=0.23)
        fig.savefig(
            output / "geometry.svg",
            metadata={"Date": None, "Creator": "plot_probe_geometry.py"},
        )
        fig.savefig(
            output / "geometry.png",
            dpi=170,
            metadata={"Software": "plot_probe_geometry.py"},
        )
        plt.close(fig)
    receipt = dict(
        python=platform.python_version(),
        matplotlib=matplotlib.__version__,
        plotter_sha256=build.sha(Path(__file__)),
        sources_sha256=build.sha(data / "sources.json"),
        fixture_sha256=fixture_sha,
        selected_ids=[r["id"] for r in records],
        input_sha256=sources["output_sha256"],
        output_sha256={
            name: build.sha(output / name) for name in ("geometry.svg", "geometry.png")
        },
    )
    (output / "plot-sources.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not __debug__:
        parser.error("Run without Python optimization")
    try:
        plot(args.data, args.output)
    except (OSError, ValueError, KeyError, AssertionError, ImportError) as error:
        parser.exit(1, f"Probe geometry plot failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
