#!/usr/bin/env python3
"""Summarize stopping-preview runs; optionally plot with matplotlib."""

import argparse
import csv
import json
import os
from pathlib import Path
import statistics
import tempfile


def save_rows(path, rows):
    with path.open("w", newline="") as target:
        writer = csv.DictWriter(target, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def plot(directory, summary):
    with tempfile.TemporaryDirectory(prefix="preview-plot-") as cache:
        os.environ.setdefault("MPLCONFIGDIR", cache)
        os.environ.setdefault("XDG_CACHE_HOME", cache)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.rcParams["svg.fonttype"] = "none"
        plt.rcParams["svg.hashsalt"] = "stopping-preview"
        fig, axes = plt.subplots(1, 3, figsize=(14, 4.5), layout="constrained")
        for ax, mode in zip(axes, ["none", "random", "blackout"]):
            selected = [
                next(
                    r
                    for r in summary
                    if r["cohort"] == cohort
                    and r["mode"] == mode
                    and r["variant"] == variant
                )
                for cohort in ["white-validation", "correlated-validation"]
                for variant in ["averaged", "guarded"]
            ]
            bottom = [0] * 4
            for field, color, label in [
                ("held", "#16a34a", "Held 10 s"),
                ("timeout", "#94a3b8", "Timeout"),
                ("collision", "#dc2626", "Contact"),
            ]:
                heights = [r[field] for r in selected]
                bars = ax.bar(
                    range(4), heights, bottom=bottom, color=color, label=label
                )
                ax.bar_label(
                    bars,
                    labels=[str(v) if v >= 3 else "" for v in heights],
                    label_type="center",
                )
                bottom = [a + b for a, b in zip(bottom, heights)]
            for i, r in enumerate(selected):
                if r["collision"]:
                    ax.text(
                        i,
                        r["trials"] + 1.5,
                        f'{r["collision"]} contact'
                        + ("s" if r["collision"] != 1 else ""),
                        ha="center",
                        color="#b91c1c",
                        fontsize=9,
                    )
            ax.set(
                title=mode,
                ylim=(0, max(r["trials"] for r in selected) * 1.12),
                ylabel="Cases (same 90 inputs per row)",
            )
            ax.set_xticks(
                range(4),
                ["White\nAverage", "White\nPreview", "1 s\nAverage", "1 s\nPreview"],
                fontsize=8,
            )
        axes[-1].legend(
            loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=3, fontsize=8
        )
        fig.suptitle("Stopping preview: fixed settings, paired new seeds, 180 s budget")
        svg = directory / "stopping-preview.svg"
        fig.savefig(svg, metadata={"Date": None})
        svg.write_text(
            "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(directory / "stopping-preview.png", dpi=160)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--white", type=Path, required=True)
    parser.add_argument("--correlated", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    parser.add_argument("--plot", action="store_true", help="Requires matplotlib")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    combined = []
    summary = []
    transitions = []
    sources = {}
    validation_seeds = None
    for cohort, run, tau in [
        ("reference", args.reference, 1),
        ("white-validation", args.white, 0),
        ("correlated-validation", args.correlated, 1),
    ]:
        provenance = json.loads((run / "provenance.json").read_text())
        assert provenance["noise_correlation_time_s"] == tau
        assert provenance["variants"] == ["averaged", "guarded"]
        rows = list(csv.DictReader((run / "all-results.csv").open()))
        events = list(csv.DictReader((run / "control-events.csv").open()))
        audits = list(csv.DictReader((run / "preview-audit.csv").open()))
        seeds = sorted({r["seed"] for r in rows})
        assert len(rows) == 54 * len(seeds) and audits
        if cohort != "reference":
            if validation_seeds is None:
                validation_seeds = seeds
            else:
                assert seeds == validation_seeds
        sources[cohort] = dict(provenance=provenance, geometry_checkpoints=len(audits))
        combined.extend(dict(cohort=cohort, **r) for r in rows)
        lookup = {
            (r["variant"], r["mode"], r["seed"], r["scenario"], r["planner"]): r
            for r in rows
        }
        for mode in ["none", "random", "blackout"]:
            gained = lost = 0
            for variant in ["averaged", "guarded"]:
                selected = [
                    r for r in rows if r["mode"] == mode and r["variant"] == variant
                ]
                ev = [
                    r for r in events if r["mode"] == mode and r["variant"] == variant
                ]
                assert len(selected) == len(ev) == 9 * len(seeds)
                summary.append(
                    dict(
                        cohort=cohort,
                        tau_s=tau,
                        mode=mode,
                        variant=variant,
                        trials=len(selected),
                        held=sum(r["held"] == "true" for r in selected),
                        collision=sum(r["collision"] == "true" for r in selected),
                        timeout=sum(r["status"] == "timeout" for r in selected),
                        out_of_bounds=sum(
                            r["status"] == "out_of_bounds" for r in selected
                        ),
                        mean_guard_time_s=statistics.mean(
                            float(r["guard_time_s"]) for r in ev
                        ),
                        mean_guard_entries=statistics.mean(
                            int(r["guard_entries"]) for r in ev
                        ),
                        mean_loss_pause_s=statistics.mean(
                            float(r["paused_time_s"]) for r in selected
                        ),
                    )
                )
                if variant == "guarded":
                    for r in selected:
                        ref = lookup[
                            ("averaged", mode, r["seed"], r["scenario"], r["planner"])
                        ]
                        gained += ref["held"] == "false" and r["held"] == "true"
                        lost += ref["held"] == "true" and r["held"] == "false"
            transitions.append(
                dict(cohort=cohort, mode=mode, gained_holds=gained, lost_holds=lost)
            )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, rows in [
        ("all-results.csv", combined),
        ("summary.csv", summary),
        ("paired-holds.csv", transitions),
    ]:
        save_rows(args.output / name, rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    if args.plot:
        plot(args.output, summary)
    for row in summary:
        print(row)


if __name__ == "__main__":
    main()
