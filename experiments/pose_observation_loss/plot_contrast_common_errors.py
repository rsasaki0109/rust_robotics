#!/usr/bin/env python3
"""Plot full simultaneous losses and requested lagged contrast/common modes."""

import argparse
from itertools import product
import json
from pathlib import Path

from analyze_contrast_common_errors import INPUTS, KINDS, PAIRINGS, PHASES
from analyze_epoch_preview import digest, read_rows
from summarize_stopping_preview import save_rows

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

METRICS = {
    "Contrast D": "contrast_variance",
    "Midpoint M": "midpoint_variance",
    "Common residual R": "residual_variance",
}
ARMS = ["epoch", "curved"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        digest(Path(__file__).with_name("analyze_contrast_common_errors.py"))
        == source["analysis"]["script_sha256"]
    )
    simultaneous = {
        (r["variant"], r["event_kind"], r["pairing"]): r
        for r in read_rows(args.results / "simultaneous-summary.csv")
        if r["cohort"] == "all" and r["population"] == "raw_zero_loss"
    }
    lagged = {
        (r["variant"], r["event_kind"], r["phase"], r["input_kind"]): r
        for r in read_rows(args.results / "lagged-summary.csv")
        if r["cohort"] == "all"
    }
    numeric = []
    for family in ["simultaneous", "lagged"]:
        lookup = simultaneous if family == "simultaneous" else lagged
        for key, r in lookup.items():
            common = dict(
                family=family,
                variant=r["variant"],
                event_kind=r["event_kind"],
                pairing=r.get("pairing", ""),
                phase=r.get("phase", ""),
                input_kind=r.get("input_kind", ""),
                records=int(r["records"]),
                matched=int(r["matched"]),
                unavailable=int(r["unavailable"]),
                zero_or_roundoff_pairs=int(r["zero_or_roundoff_pairs"]),
                negative_weight_pairs=int(r["negative_weight_pairs"]),
            )
            for name, field in METRICS.items():
                numeric.append(
                    dict(
                        **common,
                        quantity=name,
                        value_m=(
                            float(r["pooled_sigma_" + field])
                            if int(r["matched"])
                            else ""
                        ),
                    )
                )
    assert len(numeric) == 396
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "svg.fonttype": "none",
            "svg.hashsalt": "contrast-common-errors",
        }
    )
    fig = plt.figure(figsize=(13.7, 9.7))
    grid = fig.add_gridspec(
        2,
        6,
        left=0.09,
        right=0.99,
        bottom=0.19,
        top=0.82,
        wspace=0.75,
        hspace=0.75,
        height_ratios=[1, 1.15],
    )
    colors = ["#969696", "#3182bd", "#e08214"]
    handles = []
    for arm in ARMS:
        for name, color in zip(METRICS, colors):
            handles.append(
                Patch(
                    facecolor=color,
                    hatch="" if arm == "epoch" else "//",
                    label=f'{"Baseline" if arm=="epoch" else "Curved"}: {name}',
                )
            )
    fig.legend(
        handles,
        [h.get_label() for h in handles],
        loc="upper center",
        bbox_to_anchor=(0.5, 0.995),
        ncol=3,
        frameon=False,
    )
    fig.suptitle(
        "Partial noise + anchor modes: complete simultaneous loss and lagged opportunity populations",
        y=0.90,
        fontsize=12,
    )
    cap_sim = (
        max(
            r["value_m"]
            for r in numeric
            if r["family"] == "simultaneous" and r["event_kind"] == "all"
        )
        * 120
    )
    cap_lag = (
        max(
            r["value_m"]
            for r in numeric
            if r["family"] == "lagged" and r["event_kind"] == "all"
        )
        * 120
    )
    for j, (a, b) in enumerate(PAIRINGS):
        ax = fig.add_subplot(grid[0, 2 * j : 2 * j + 2])
        pairing = a + "_" + b
        for ai, arm in enumerate(ARMS):
            r = simultaneous[arm, "all", pairing]
            assert (
                int(r["records"])
                == int(r["matched"])
                == (197 if arm == "epoch" else 353)
            )
            for mi, (name, field) in enumerate(METRICS.items()):
                ax.bar(
                    ai + (mi - 1) * 0.25,
                    100 * float(r["pooled_sigma_" + field]),
                    width=0.23,
                    color=colors[mi],
                    hatch="" if ai == 0 else "//",
                )
        ax.set_xticks([0, 1], ["Baseline\n197/197 matched", "Curved\n353/353 matched"])
        ax.set_ylim(0, cap_sim)
        ax.set_title(f"{a.title()} / {b.title()}\nAll original raw-zero losses", pad=10)
        if j == 0:
            ax.set_ylabel("Model sigma per axis (cm)")
        ax.grid(axis="y", alpha=0.2)
        ax.set_axisbelow(True)
    for j, inp in enumerate(INPUTS):
        ax = fig.add_subplot(grid[1, 3 * j : 3 * j + 3])
        for pi, phase in enumerate(PHASES):
            for ai, arm in enumerate(ARMS):
                r = lagged[arm, "all", phase, inp]
                for mi, (name, field) in enumerate(METRICS.items()):
                    x = pi - 0.375 + (ai * 3 + mi + 0.5) * 0.125
                    ax.bar(
                        x,
                        100 * float(r["pooled_sigma_" + field]),
                        width=0.115,
                        color=colors[mi],
                        hatch="" if ai == 0 else "//",
                    )
        ax.set_xticks(range(4), ["Immediate", "+.18 s", "+.50 s", "+1.50 s"])
        ax.set_ylim(0, cap_lag)
        ax.set_title(
            f'{"Recovery" if inp=="actual" else "Preview"}: pre/post oracle error contrast',
            pad=10,
        )
        if j == 0:
            ax.set_ylabel("Model sigma per axis (cm)")
        ax.grid(axis="y", alpha=0.2)
        ax.set_axisbelow(True)
        counts = []
        for arm, label in [("epoch", "Baseline"), ("curved", "Curved")]:
            rr = [lagged[arm, "all", p, inp] for p in PHASES]
            assert len({r["records"] for r in rr}) == 1
            counts.append(
                f"{label} matched: "
                + ", ".join(r["matched"] for r in rr)
                + f' / {rr[0]["records"]} each'
            )
        ax.text(
            0, -0.27, "\n".join(counts), transform=ax.transAxes, fontsize=8, va="top"
        )
    fig.text(
        0.09,
        0.025,
        "D = eA - eB; M = (eA + eB)/2; R = M - kD. Coefficients use the pinned covariance, with a separate zero-contrast branch.\n"
        "Lagged D subtracts known truth motion and is an offline oracle diagnostic. Missing projections are blank; pooled values use matched pairs.\n"
        "Model excludes drift/transport and is not calibrated. Inputs and phases overlap; recorded control and held goals remain fixed.",
        fontsize=9,
    )
    args.output.mkdir(parents=True)
    stem = "contrast-common-comparisons"
    save_rows(args.output / (stem + ".csv"), numeric)
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
    provenance = dict(
        script_sha256=digest(Path(__file__)),
        input_sha256={
            n: digest(args.results / n)
            for n in ["simultaneous-summary.csv", "lagged-summary.csv", "sources.json"]
        },
        numeric_rows=len(numeric),
        weighting="one matched source reference; simultaneous raw-zero-loss and lagged all-request denominators retained",
    )
    (args.output / (stem + "-sources.json")).write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(
        json.dumps(dict(numeric_rows=len(numeric), output=str(args.output)), indent=2)
    )


if __name__ == "__main__":
    main()
