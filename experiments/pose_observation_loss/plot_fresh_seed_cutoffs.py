#!/usr/bin/env python3
"""Plot original-audit source support and prespecified fixed-cutoff outcomes."""

import argparse
from itertools import product
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_fresh_seed_modes import COHORT_SEEDS
from summarize_stopping_preview import save_rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.hashsalt": "fresh-seed-cutoff-transfer",
        }
    )
    args.output.mkdir(parents=True)
    cases = read_rows(args.results / "case-support.csv")
    comparison = read_rows(args.results / "comparison.csv")
    support_values = []
    cutoff_values = []
    colors = {
        "filter_only": "#4477aa",
        "fusion_only": "#ee9944",
        "mixed": "#66aa88",
        "no_audit": "#dddddd",
    }
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    for (noise, arm), ax in zip(
        product(["white", "correlated"], ["epoch", "curved"]), axes.flat
    ):
        labels = []
        for cohort in ["known-" + noise, "new-" + noise, "fresh-" + noise]:
            for seed in COHORT_SEEDS[cohort]:
                group = [
                    c
                    for c in cases
                    if c["cohort"] == cohort
                    and c["variant"] == arm
                    and int(c["seed"]) == seed
                ]
                assert len(group) == 27
                counts = {k: sum(c["support"] == k for c in group) for k in colors}
                assert sum(counts.values()) == 27
                r = dict(
                    cohort=cohort,
                    variant=arm,
                    seed=seed,
                    accounting_cases=27,
                    timeout_cases=sum(c["selected_timeout"] == "True" for c in group),
                    audited_cases=27 - counts["no_audit"],
                    **counts,
                )
                support_values.append(r)
                y = len(labels)
                left = 0
                for k, color in colors.items():
                    ax.barh(y, counts[k], left=left, height=0.65, color=color)
                    left += counts[k]
                prefix = (
                    "Known"
                    if cohort.startswith("known-")
                    else "Existing" if cohort.startswith("new-") else "Fresh"
                )
                labels.append(f"{prefix} {seed}")
                ax.text(
                    27.7,
                    y,
                    f'audit={r["audited_cases"]}/{r["timeout_cases"]}',
                    va="center",
                    fontsize=8,
                )
        ax.set_yticks(range(9), labels)
        for label in ax.get_yticklabels()[6:]:
            label.set_weight("bold")
        ax.set_xlim(0, 38)
        ax.set_ylim(8.7, -0.7)
        ax.set_xticks([0, 9, 18, 27])
        ax.set_xlabel("Accounting cases (all loss modes / maps / planners)")
        ax.set_title(f'{noise.title()} / {"baseline" if arm=="epoch" else "curved"}')
        ax.axhline(2.5, color=".7", lw=0.7)
        ax.axhline(5.5, color=".7", lw=0.7)
        ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle(
        "Original joint-ready adjacent-state audit: per-seed preview-mode support",
        fontsize=13,
        y=0.985,
    )
    fig.legend(
        handles=[
            Patch(
                color=v,
                label={
                    "filter_only": "Filter only",
                    "fusion_only": "Fusion only",
                    "mixed": "Both modes",
                    "no_audit": "No audit states",
                }[k],
            )
            for k, v in colors.items()
        ],
        loc="lower center",
        ncol=4,
        bbox_to_anchor=(0.5, 0.045),
        frameon=False,
    )
    fig.text(
        0.5,
        0.013,
        "Audit / timeout counts are case counts. Mixed cases occupy one segment; repeated states are not independent trials.",
        ha="center",
        fontsize=9,
    )
    fig.tight_layout(rect=[0, 0.085, 1, 0.95], h_pad=2, w_pad=2)

    def save(fig, name):
        p = args.output / (name + ".svg")
        fig.savefig(p, metadata={"Date": None})
        p.write_text(
            "\n".join(line.rstrip() for line in p.read_text().splitlines()) + "\n"
        )
        fig.savefig(
            args.output / (name + ".png"),
            dpi=150,
            metadata={"Software": "fresh-seed-cutoff-transfer"},
        )
        plt.close(fig)

    save(fig, "fresh-audit-support")
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    selected = [
        r
        for r in comparison
        if r["event_kind"] == "all"
        and r["view"] == "sim_recovery_preview"
        and r["stratum"] == "all"
        and r["score"] == "residual_sigma"
        and r["budget"] == "q50"
    ]
    assert len(selected) == 4
    for (noise, arm), ax in zip(
        product(["white", "correlated"], ["epoch", "curved"]), axes.flat
    ):
        r = next(r for r in selected if r["noise"] == noise and r["variant"] == arm)
        for mi, method in enumerate(["pooled", "conditional"]):
            for x, metric in enumerate(
                [
                    "case_weighted_coverage",
                    "midpoint_case_weighted_risk",
                    "residual_case_weighted_risk",
                ]
            ):
                value = r[method + "_" + metric]
                pos = x + (mi - 0.5) * 0.34
                cutoff_values.append(
                    dict(
                        noise=noise,
                        variant=arm,
                        method=method,
                        metric=metric,
                        value=value,
                        records=r["records"],
                        cases=r["cases"],
                        conditional_support=r["conditional_support"],
                        retained_rows=r[method + "_retained_rows"],
                        retained_case_mass=r[method + "_retained_case_mass"],
                    )
                )
                if value != "":
                    ax.bar(
                        pos,
                        float(value) * 100,
                        width=0.32,
                        color=["#4477aa", "#ee9944"][mi],
                    )
                    ax.text(
                        pos,
                        float(value) * 100 + 1,
                        f"{float(value)*100:.1f}",
                        ha="center",
                        fontsize=8,
                    )
                else:
                    ax.text(pos, 2, "NA", ha="center", fontsize=8)
        ax.set_ylim(0, 110)
        ax.set_yticks([0, 25, 50, 75, 100])
        ax.set_ylabel("Case-weighted percentage")
        ax.set_xticks([0, 1, 2], ["Retained", "Midpoint outside", "Residual outside"])
        ax.set_title(
            f'{noise.title()} / {"baseline" if arm=="epoch" else "curved"}\n{r["records"]} state pairs / {r["cases"]} cases'
        )
        ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle(
        "Additional seeds under fixed known-seed residual-sigma q50 cutoffs\nRecovery / Preview; all source-event kinds; original audit selection",
        fontsize=12,
        y=0.98,
    )
    fig.legend(
        handles=[
            Patch(color="#4477aa", label="Pooled cutoff"),
            Patch(color="#ee9944", label="Mode-conditional cutoffs"),
        ],
        loc="lower center",
        ncol=2,
        bbox_to_anchor=(0.5, 0.058),
        frameon=False,
    )
    fig.text(
        0.5,
        0.015,
        "Outside = observed position error norm >0.1 m. NA is undefined; no zero filling.\nThresholds remain trained on 269/270/271; added seeds do not refit them. Three seed streams, repeated paired states.",
        ha="center",
        fontsize=9,
    )
    fig.tight_layout(rect=[0, 0.105, 1, 0.91], w_pad=2, h_pad=2)
    save(fig, "fresh-fixed-cutoff-risk")
    save_rows(args.output / "support-values.csv", support_values)
    save_rows(args.output / "cutoff-values.csv", cutoff_values)
    sources = dict(
        script_sha256=digest(Path(__file__)),
        analysis_sources_sha256=digest(args.results / "sources.json"),
        inputs_sha256={
            n: digest(args.results / n) for n in ["case-support.csv", "comparison.csv"]
        },
        support_rows=len(support_values),
        cutoff_rows=len(cutoff_values),
        matplotlib=matplotlib.__version__,
        outputs_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "plot-sources.json").write_text(json.dumps(sources, indent=2) + "\n")


if __name__ == "__main__":
    main()
