#!/usr/bin/env python3
"""Plot the prespecified late-timeout mode census, with per-seed denominators."""

import argparse
from itertools import product
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_fresh_seed_modes import COHORT_SEEDS, FRESH, ROOT, VARIANTS
from summarize_stopping_preview import save_rows


def plot(results, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    source = read_rows(results / "summary.csv")
    keys = ["cohort", "seed", "variant", "mode", "event_group"]
    lookup = {tuple(r[k] for k in keys): r for r in source}
    assert len(lookup) == len(source)
    numeric = []
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.hashsalt": "fresh-seed-mode-census",
        }
    )
    for group in ["all", "received_any"]:
        fig, axes = plt.subplots(2, 2, figsize=(13, 9.4))
        for (noise, variant), ax in zip(
            product(["white", "correlated"], VARIANTS), axes.flat
        ):
            labels = []
            for cohort in ["known-" + noise, "new-" + noise, "fresh-" + noise]:
                for seed in COHORT_SEEDS[cohort]:
                    r = lookup[cohort, str(seed), variant, "all", group]
                    y = len(labels)
                    prefix = (
                        "Known"
                        if cohort.startswith("known-")
                        else "Existing" if cohort.startswith("new-") else "Fresh"
                    )
                    labels.append(f"{prefix} {seed}")
                    numeric.append(r)
                    if int(r["event_cases"]):
                        f, u = float(r["filter_case_weighted_share"]), float(
                            r["fusion_case_weighted_share"]
                        )
                        assert abs(f + u - 1) < 1e-12
                        ax.barh(y, f * 100, color="#4477aa", height=0.62)
                        ax.barh(y, u * 100, left=f * 100, color="#ee9944", height=0.62)
                        ax.text(
                            50,
                            y,
                            f"{f*100:.1f}% Filter",
                            ha="center",
                            va="center",
                            fontsize=8,
                            bbox=dict(
                                facecolor="white", edgecolor="none", alpha=0.85, pad=1.5
                            ),
                        )
                    else:
                        ax.text(50, y, "No event cases", ha="center", va="center")
                    ax.text(
                        103,
                        y,
                        f'n={r["events"]}; c={r["event_cases"]}/{r["timeout_cases"]}',
                        va="center",
                        fontsize=8,
                    )
            ax.set_yticks(range(len(labels)), labels)
            for label in ax.get_yticklabels()[6:]:
                label.set_weight("bold")
            ax.axhline(2.5, color=".7", lw=0.7)
            ax.axhline(5.5, color=".7", lw=0.7)
            ax.set_xlim(0, 141)
            ax.set_ylim(8.7, -0.7)
            ax.set_xticks([0, 25, 50, 75, 100], ["0", "25", "50", "75", "100"])
            ax.set_xlabel("Parent-case-weighted event share (%)")
            ax.set_title(
                f'{"White noise" if noise == "white" else "Correlated noise (tau=1 s)"} / {"baseline" if variant == "epoch" else "curved"}'
            )
            ax.spines[["top", "right"]].set_visible(False)
        fig.suptitle(
            "Immediate preview modes at late timeout opportunities\n"
            + (
                "All correction / precision opportunities"
                if group == "all"
                else "Received corrections, including combined precision events"
            ),
            fontsize=13,
            y=0.98,
        )
        fig.legend(
            handles=[
                Patch(color="#4477aa", label="Filter (motion_filter)"),
                Patch(color="#ee9944", label="Fusion (three_epoch)"),
            ],
            loc="lower center",
            ncol=2,
            bbox_to_anchor=(0.5, 0.063),
            frameon=False,
        )
        fig.text(
            0.5,
            0.025,
            "n = events; c = event cases / timeout cases. One case has total weight 1 before mode conditioning.\nAll timeouts, logged 150–180 s; wider than the earlier speed-probe audit. Repeated events and paired arms share seed streams.",
            ha="center",
            fontsize=9,
        )
        fig.tight_layout(rect=[0, 0.1, 1, 0.92], w_pad=2, h_pad=2)
        name = "fresh-seed-modes-" + group.replace("_", "-")
        fig.savefig(output / f"{name}.svg", metadata={"Date": None})
        svg = output / f"{name}.svg"
        svg.write_text(
            "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(
            output / f"{name}.png",
            dpi=150,
            metadata={"Software": "fresh-seed-mode-census"},
        )
        plt.close(fig)
    assert len(numeric) == 72
    save_rows(output / "plot-values.csv", numeric)
    sources = dict(
        analysis_sha256=digest(results / "sources.json"),
        summary_sha256=digest(results / "summary.csv"),
        script_sha256=digest(Path(__file__)),
        analyzer_sha256=digest(ROOT / "analyze_fresh_seed_modes.py"),
        primary_event_groups=["all", "received_any"],
        rows=len(numeric),
        fresh_cohorts=FRESH,
        outputs_sha256={p.name: digest(p) for p in sorted(output.iterdir())},
        matplotlib=matplotlib.__version__,
    )
    (output / "plot-sources.json").write_text(json.dumps(sources, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    args.output.mkdir(parents=True)
    plot(args.results, args.output)


if __name__ == "__main__":
    main()
