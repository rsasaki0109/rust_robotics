#!/usr/bin/env python3
"""Plot omitted-known and repeated-new evaluations without filling missing modes."""

import argparse
from itertools import product
import json
from pathlib import Path

from analyze_epoch_preview import digest, read_rows
from analyze_seed_exclusion_cutoffs import ARMS, NOISES, SEEDS, TARGETS
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def focus(r):
    return (r["view"], r["score"], r["budget"], r["event_kind"], r["stratum"]) == (
        "sim_recovery_preview",
        "residual_sigma",
        "q50",
        "all",
        "all",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        digest(Path(__file__).with_name("analyze_seed_exclusion_cutoffs.py"))
        == source["analysis"]["script_sha256"]
    )
    folds = [r for r in read_rows(args.results / "fold-comparison.csv") if focus(r)]
    refs = [
        dict(
            r,
            fold_id="all_known",
            held_seed="",
            fold_training_seeds="269+270+271",
            evaluation="existing_new",
        )
        for r in read_rows(args.results / "reference.csv")
        if focus(r)
    ]
    assert (len(folds), len(refs)) == (24, 4)
    numeric = []
    for r, target in product([*folds, *refs], TARGETS):
        n = {
            k: r[k]
            for k in [
                "fold_id",
                "held_seed",
                "fold_training_seeds",
                "evaluation",
                "partition",
                "noise",
                "variant",
                "view",
                "score",
                "budget",
                "stratum",
                "records",
                "cases",
                "case_mass",
                "conditional_support",
                "conditional_unsupported_rows",
                "conditional_unsupported_case_mass",
                "conditional_unsupported_strata",
            ]
        }
        n.update(target=target)
        n.update(
            {
                m + "_" + f: r[m + "_" + f]
                for m in ["pooled", "conditional"]
                for f in [
                    "retained_rows",
                    "retained_cases",
                    "retained_case_mass",
                    "case_weighted_coverage",
                ]
            }
        )
        n.update(
            {
                m + "_outside_fraction": r[m + "_" + target + "_case_weighted_risk"]
                for m in ["pooled", "conditional"]
            }
        )
        numeric.append(n)
    assert len(numeric) == 56
    lookup = {
        (r["noise"], r["variant"], r["evaluation"], r["fold_id"], r["target"]): r
        for r in numeric
    }
    displays = (
        [("held_known", "omit_" + str(s)) for s in SEEDS]
        + [("existing_new", "omit_" + str(s)) for s in SEEDS]
        + [("existing_new", "all_known")]
    )
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "seed-exclusion-cutoffs",
        }
    )
    cmap = plt.get_cmap("viridis").copy()
    cmap.set_bad("#dddddd")
    args.output.mkdir(parents=True)
    for target in TARGETS:
        fig, axes = plt.subplots(2, 2, figsize=(15.8, 10.8))
        fig.subplots_adjust(
            left=0.24, right=0.98, top=0.89, bottom=0.21, wspace=0.70, hspace=0.48
        )
        for ni, noise in enumerate(NOISES):
            for ai, arm in enumerate(ARMS):
                ax = axes[ni, ai]
                values = np.full((7, 4), np.nan)
                labels, unsupported = [], []
                for yi, (evaluation, fold) in enumerate(displays):
                    r = lookup[noise, arm, evaluation, fold, target]
                    fields = [
                        "pooled_case_weighted_coverage",
                        "conditional_case_weighted_coverage",
                        "pooled_outside_fraction",
                        "conditional_outside_fraction",
                    ]
                    for ci, field in enumerate(fields):
                        if r[field] != "":
                            values[yi, ci] = float(r[field])
                    label = (
                        "Held " + r["held_seed"]
                        if evaluation == "held_known"
                        else (
                            "New; fit all known"
                            if fold == "all_known"
                            else "New; omit " + r["held_seed"]
                        )
                    )
                    label += f'  {r["records"]} refs / {r["cases"]} cases'
                    if int(r["conditional_unsupported_rows"]):
                        label += (
                            f'\nu={r["conditional_unsupported_rows"]} unsupported refs'
                        )
                    labels.append(label)
                    unsupported.append(int(r["conditional_unsupported_rows"]) > 0)
                handle = ax.imshow(
                    np.ma.masked_invalid(values),
                    vmin=0,
                    vmax=1,
                    cmap=cmap,
                    aspect="auto",
                )
                ax.set_yticks(range(7), labels, fontsize=8)
                ax.set_xticks(
                    range(4),
                    [
                        "Pooled\ncoverage",
                        "Mode\ncoverage",
                        "Pooled\noutside",
                        "Mode\noutside",
                    ],
                    fontsize=8,
                )
                ax.set_title(
                    f'{noise.title()} / {"Baseline" if arm=="epoch" else "Curved"}',
                    pad=10,
                )
                for yi, ci in product(range(7), range(4)):
                    v = values[yi, ci]
                    text = (
                        ("NA*" if ci in [1, 3] and unsupported[yi] else "NA")
                        if np.isnan(v)
                        else f"{100*v:.1f}"
                    )
                    ax.text(
                        ci,
                        yi,
                        text,
                        ha="center",
                        va="center",
                        fontsize=9,
                        color="black" if np.isnan(v) or v >= 0.5 else "white",
                    )
                for y in [2.5, 5.5]:
                    ax.axhline(y, color="white", linewidth=2)
        fig.suptitle(f"Whole-known-seed exclusion: {target} error", fontsize=16, y=0.98)
        fig.text(
            0.5,
            0.936,
            "Recovery / Preview; residual-sigma q50; all event kinds",
            ha="center",
            fontsize=11,
        )
        cax = fig.add_axes([0.43, 0.11, 0.35, 0.016])
        bar = fig.colorbar(handle, cax=cax, orientation="horizontal")
        bar.set_label("Case-weighted fractions; cells show %")
        bar.ax.xaxis.set_label_position("top")
        bar.ax.xaxis.labelpad = 6
        fig.text(
            0.05,
            0.022,
            "Held: evaluate the omitted known seed using the other two; New: the same existing new cases under each fit. Outside: norm >.10 m.\nNA*: a represented mode has no training fit; keep the whole conditional result undefined. NA: empty population/retention. u counts unsupported references.\nBoth cohorts were observed previously. Repeated new evaluations are sensitivity checks, without independent replication or calibrated error guarantees.",
            fontsize=9,
        )
        stem = "seed-exclusion-" + target
        svg = args.output / (stem + ".svg")
        fig.savefig(svg, bbox_inches="tight", metadata={"Date": None})
        svg.write_text(
            "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(
            args.output / (stem + ".png"),
            dpi=150,
            bbox_inches="tight",
            metadata={"Software": "Matplotlib"},
        )
        plt.close(fig)
    save_rows(args.output / "seed-exclusion-cutoffs.csv", numeric)
    provenance = dict(
        script_sha256=digest(Path(__file__)),
        input_sha256={
            n: digest(args.results / n)
            for n in ["fold-comparison.csv", "reference.csv", "sources.json"]
        },
        numeric_rows=len(numeric),
        targets=TARGETS,
        filters=dict(
            view="sim_recovery_preview",
            score="residual_sigma",
            budget="q50",
            event_kind="all",
            stratum="all",
        ),
        retrospective=True,
    )
    (args.output / "seed-exclusion-cutoffs-sources.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(
        json.dumps(
            dict(numeric_rows=len(numeric), figures=2, output=str(args.output)),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
