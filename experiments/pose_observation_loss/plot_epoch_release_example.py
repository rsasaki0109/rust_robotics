#!/usr/bin/env python3
"""Plot the shared-history release decision in one recorded lost-hold example."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import tempfile

from analyze_epoch_divergence import (
    ROOT,
    digest,
    read_rows,
    release_inputs,
    shared_inputs,
)
from summarize_stopping_preview import save_rows

CASE = dict(
    cohort="new-white", mode="none", seed="271", scenario="maze_50x50", planner="Theta*"
)


def window(path, start, end):
    selected, before = [], None
    with path.open() as stream:
        for row in csv.DictReader(stream):
            if any(row[k] != CASE[k] for k in ["scenario", "planner"]):
                continue
            if (
                before is not None
                and start - 1e-8 <= float(row["time_s"]) <= end + 1e-8
            ):
                selected.append((before, row))
            before = row
    return selected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--diagnosis", type=Path, default=ROOT / "results/epoch-divergence"
    )
    parser.add_argument("--raw-root", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    args = parser.parse_args()
    assert __debug__, "Run without Python -O"
    case = next(
        r
        for r in read_rows(args.diagnosis / "forks.csv")
        if all(r[k] == v for k, v in CASE.items())
    )
    end = float(case["first_action_s"])
    start = end - 0.76
    sources = json.loads((ROOT / "results/epoch-preview/sources.json").read_text())
    pinned = sources[CASE["cohort"]]
    raw = args.raw_root / CASE["cohort"] if args.raw_root else Path(pinned["path"])
    tracks, hashes = [], {}
    for variant in ["aligned", "epoch"]:
        name = f"{variant}-{CASE['mode']}-{CASE['seed']}/trajectories.csv"
        assert digest(raw / name) == pinned["raw_sha256"][name]
        hashes[name] = pinned["raw_sha256"][name]
        tracks.append(window(raw / name, start, end))
    assert len(tracks[0]) == len(tracks[1]) == 39
    rows = []
    for (before_a, a), (before_b, b) in zip(*tracks):
        assert a["time_s"] == b["time_s"]
        assert shared_inputs(before_a, a, before_b, b)
        row = dict(
            time_s=float(a["time_s"]), fusion_available=b["epoch_fusion_available"]
        )
        for variant, before, current in [
            ("aligned", before_a, a),
            ("epoch", before_b, b),
        ]:
            route, required = release_inputs(before, current)
            assert route in ["inactive", "independent"]
            evidence = (
                float(before["guard_release_evidence"]) + 0.02
                if required and all(required.values())
                else 0.0
            )
            logged = 0.0 if current["guard_released"] == "true" else evidence
            assert abs(logged - float(current["guard_release_evidence"])) < 1e-8
            row.update(
                {
                    f"{variant}_position_error": math.hypot(
                        *(
                            float(current[f"preview_pose_{axis}"]) - float(before[axis])
                            for axis in ["x", "y"]
                        )
                    ),
                    f"{variant}_release_slack": float(
                        current["release_preview_clearance_lower_bound"]
                    )
                    - float(current["guard_release_margin"]),
                    f"{variant}_confirmation_s": evidence,
                }
            )
        rows.append(row)
    assert tracks[0][-1][1]["guard_released"] == "true"
    assert tracks[1][-1][1]["guard_released"] == "false"
    args.output.mkdir(parents=True, exist_ok=False)
    save_rows(args.output / "release-example.csv", rows)
    with tempfile.TemporaryDirectory(prefix="epoch-release-plot-") as cache:
        os.environ.setdefault("MPLCONFIGDIR", cache)
        os.environ.setdefault("XDG_CACHE_HOME", cache)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.rcParams["svg.fonttype"] = "none"
        plt.rcParams["svg.hashsalt"] = "epoch-preview-release-example"
        fig, axes = plt.subplots(
            3, 1, figsize=(8, 7.2), sharex=True, layout="constrained"
        )
        for variant, color in [("aligned", "#2563eb"), ("epoch", "#d97706")]:
            for ax, field in zip(
                axes, ["position_error", "release_slack", "confirmation_s"]
            ):
                ax.plot(
                    [r["time_s"] for r in rows],
                    [r[f"{variant}_{field}"] for r in rows],
                    label=variant,
                    color=color,
                    linewidth=2,
                )
        axes[0].set_title("Lost hold: new-white / none / seed 271 / maze / Theta*")
        axes[0].set_ylabel("Position error\n(model units)")
        axes[0].legend(loc="upper left", ncol=2)
        axes[1].set_ylabel("Release clearance\nminus threshold")
        axes[1].axhline(0, color="#64748b", linestyle="--", linewidth=1)
        axes[2].set_ylabel("Confirmation before\nreset (seconds)")
        axes[2].axhline(0.5, color="#64748b", linestyle="--", linewidth=1)
        axes[2].set_xlabel("Observation/control event time (seconds)")
        for ax in axes:
            ax.axvline(end, color="#475569", linestyle=":", linewidth=1)
            ax.grid(alpha=0.2)
            ax.set_xlim(start, end + 0.03)
        axes[2].annotate(
            "aligned releases",
            xy=(end, 0.5),
            xytext=(end - 0.36, 0.43),
            arrowprops=dict(arrowstyle="->", color="#475569"),
        )
        fig.savefig(args.output / "release-example.svg", metadata={"Date": None})
        svg = args.output / "release-example.svg"
        svg.write_text(
            "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(args.output / "release-example.png", dpi=150)
        plt.close(fig)
    (args.output / "release-example-sources.json").write_text(
        json.dumps(
            dict(
                case=CASE,
                first_action_s=end,
                script_sha256=digest(Path(__file__)),
                diagnostic_forks_sha256=digest(args.diagnosis / "forks.csv"),
                diagnostic_sources_sha256=digest(args.diagnosis / "sources.json"),
                diagnostic_helper_sha256=digest(ROOT / "analyze_epoch_divergence.py"),
                raw_sha256=hashes,
                shared_pre_command_rows=len(rows),
                matplotlib_version=matplotlib.__version__,
            ),
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
