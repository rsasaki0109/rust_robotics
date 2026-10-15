#!/usr/bin/env python3
"""Separate position and heading effects at previously fixed selected angles."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import statistics
import tempfile

from analyze_recovery_arcs import arc_clearance, chord_check
from analyze_recovery_stalls import center_clearance, digest
from analyze_robust_arc_selection import KEY, accepts
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
RULES = ["nominal", "yaw_min"]
CATEGORIES = ["position_alone", "heading_alone", "either_alone", "joint_required"]


def hybrid_poses(filtered, truth):
    return dict(
        FF=filtered,
        TF=(truth[0], truth[1], filtered[2]),
        FT=(filtered[0], filtered[1], truth[2]),
        TT=truth,
    )


def signed_losses(clearances):
    ff, tf, ft, tt = (clearances[k] for k in ["FF", "TF", "FT", "TT"])
    position = ((ff - tf) + (ft - tt)) / 2
    heading = ((ff - ft) + (tf - tt)) / 2
    assert abs(position + heading - (ff - tt)) < 1e-10
    return position, heading


def mismatch_category(clearances, margin):
    assert accepts(clearances["FF"], margin)
    assert not accepts(clearances["TT"], margin)
    position_fails = not accepts(clearances["TF"], margin)
    heading_fails = not accepts(clearances["FT"], margin)
    if position_fails and heading_fails:
        return "either_alone"
    if position_fails:
        return "position_alone"
    return "heading_alone" if heading_fails else "joint_required"


def summarize(rows):
    summaries = []
    for reason in ["final", "fresh_stationary"]:
        for rule in RULES:
            group = [r for r in rows if r["reason"] == reason and r["rule"] == rule]
            failures = [r for r in group if r["mismatch"]]
            row = dict(
                reason=reason,
                rule=rule,
                checkpoints=len(group),
                accepted=sum(r["accepted"] for r in group),
                accepted_true_pass=sum(r["accepted_true_pass"] for r in group),
                mismatches=len(failures),
            )
            row.update(
                {k: sum(r["category"] == k for r in failures) for k in CATEGORIES}
            )
            assert sum(row[k] for k in CATEGORIES) == len(failures)
            row["true_start_blocks_mismatches"] = sum(
                r["true_start_blocks"] for r in failures
            )
            row["position_larger_loss"] = sum(
                r["position_loss"] > r["heading_loss"] + 1e-10 for r in failures
            )
            row["negative_position_loss"] = sum(
                r["position_loss"] < -1e-10 for r in failures
            )
            row["negative_heading_loss"] = sum(
                r["heading_loss"] < -1e-10 for r in failures
            )
            for field in ["total_loss", "position_loss", "heading_loss"]:
                values = [r[field] for r in failures]
                row[f"mean_{field}"] = statistics.mean(values) if values else ""
                row[f"median_{field}"] = statistics.median(values) if values else ""
            summaries.append(row)
    return summaries


def plot(output, summaries):
    with tempfile.TemporaryDirectory(prefix="pose-component-plot-") as cache:
        os.environ.setdefault("MPLCONFIGDIR", cache)
        os.environ.setdefault("XDG_CACHE_HOME", cache)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.rcParams.update({"svg.fonttype": "none", "svg.hashsalt": "pose-components"})
        fig, ax = plt.subplots(figsize=(10, 5.5), layout="constrained")
        bottom = [0] * len(summaries)
        for category, color, label in zip(
            CATEGORIES,
            ["#2563eb", "#eab308", "#a855f7", "#94a3b8"],
            [
                "Only position substitution fails",
                "Only heading substitution fails",
                "Either substitution alone fails",
                "Both substitutions needed to fail",
            ],
        ):
            values = [r[category] for r in summaries]
            ax.bar(range(4), values, bottom=bottom, label=label, color=color)
            for i, value in enumerate(values):
                if value:
                    ax.text(
                        i, bottom[i] + value / 2, str(value), ha="center", va="center"
                    )
            bottom = [a + b for a, b in zip(bottom, values)]
        ax.set_xticks(
            range(4),
            ["Final\nnominal", "Final\nyaw_min", "Fresh\nnominal", "Fresh\nyaw_min"],
        )
        ax.set(
            ylabel="Accepted candidates failing the same true margin",
            title="Fixed-angle pose substitution on saved failed-run checkpoints",
        )
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=2, fontsize=9)
        svg = output / "arc-pose-components.svg"
        fig.savefig(svg, metadata={"Date": None})
        svg.write_text(
            "\n".join(s.rstrip() for s in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(output / "arc-pose-components.png", dpi=160)
        plt.close(fig)


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="New directory")
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    previous = ROOT / "results/robust-arc-selection"
    paired = ROOT / "results/aligned-arcs"
    provenance = json.loads((previous / "sources.json").read_text())
    for name, expected in provenance["inputs_sha256"].items():
        assert digest(paired / name) == expected, name
    obstacles_path = (
        ROOT / "results/planning-clearance/fixtures/fine-padding/obstacles.csv"
    )
    assert digest(obstacles_path) == provenance["obstacles_sha256"]
    records = read_rows(paired / "checkpoints.csv")
    assert len(records) == 994
    poses = {(tuple(r[k] for k in KEY), r["pose_kind"]): r for r in records}
    all_decisions = read_rows(previous / "decisions.csv")
    assert len(all_decisions) == 1491
    decisions = [r for r in all_decisions if r["rule"] in RULES]
    assert len(decisions) == 994
    candidates = {
        (tuple(r[k] for k in KEY), int(r["steering_deg"])): r
        for r in read_rows(previous / "candidate-scores.csv")
    }
    maps = {}
    for r in read_rows(obstacles_path):
        maps.setdefault(r["scenario"], []).append((float(r["x"]), float(r["y"])))
    rows, audits, cache = [], [], {}
    for decision in decisions:
        key = tuple(decision[k] for k in KEY)
        identity = {k: decision[k] for k in [*KEY, "decision_time_s", "reason"]}
        filtered_record, true_record = (poses[(key, k)] for k in ["filtered", "truth"])
        filtered, truth = (
            tuple(float(r[k]) for k in ["x", "y", "yaw"])
            for r in [filtered_record, true_record]
        )
        angle = int(decision["steering_deg"])
        obstacles = maps[decision["scenario"]]
        if (key, angle) not in cache:
            clearances = {}
            for kind, pose in hybrid_poses(filtered, truth).items():
                clearance = arc_clearance(pose, math.radians(angle), obstacles)
                error, bound = chord_check(
                    pose, math.radians(angle), obstacles, clearance
                )
                clearances[kind] = clearance
                audits.append(
                    dict(
                        **identity,
                        steering_deg=angle,
                        pose_kind=kind,
                        x=pose[0],
                        y=pose[1],
                        yaw=pose[2],
                        exact_clearance=clearance,
                        chord_error=error,
                        sagitta_bound=bound,
                    )
                )
            cache[(key, angle)] = clearances
        values = cache[(key, angle)]
        prior_candidate = candidates[(key, angle)]
        assert abs(values["FF"] - float(prior_candidate["central"])) < 1e-9
        assert abs(values["TT"] - float(decision["true_clearance"])) < 1e-9
        margin = float(decision["target_margin"])
        assert (
            abs(margin - float(filtered_record["recovery_current_age_margin"])) < 1e-12
        )
        score = float(
            prior_candidate["central" if decision["rule"] == "nominal" else "yaw_lower"]
        )
        assert abs(score - float(decision["score"])) < 1e-12
        accepted = decision["accepted"] == "True"
        assert accepted == accepts(score, margin)
        true_pass = accepts(values["TT"], margin)
        mismatch = accepted and not true_pass
        assert mismatch == (decision["mismatch"] == "True")
        start = center_clearance(truth[0], truth[1], obstacles)
        assert abs(start - float(true_record["center_clearance"])) < 1e-9
        position_loss, heading_loss = signed_losses(values)
        rows.append(
            dict(
                **identity,
                rule=decision["rule"],
                steering_deg=angle,
                accepted=accepted,
                accepted_true_pass=accepted and true_pass,
                mismatch=mismatch,
                target_margin=margin,
                **{f"clearance_{k}": v for k, v in values.items()},
                **{f"passes_{k}": accepts(v, margin) for k, v in values.items()},
                total_loss=values["FF"] - values["TT"],
                position_loss=position_loss,
                heading_loss=heading_loss,
                category=(
                    mismatch_category(values, margin) if mismatch else "not_mismatch"
                ),
                true_start_clearance=start,
                true_start_blocks=start <= margin,
            )
        )
    summaries = summarize(rows)
    prior_summary = {
        (r["reason"], r["rule"]): r for r in read_rows(previous / "summary.csv")
    }
    for row in summaries:
        prior = prior_summary[(row["reason"], row["rule"])]
        for field in ["checkpoints", "accepted", "accepted_true_pass"]:
            assert row[field] == int(prior[field])
        assert row["mismatches"] == int(prior["mismatch"])
    inputs = [
        paired / "checkpoints.csv",
        paired / "sources.json",
        previous / "decisions.csv",
        previous / "candidate-scores.csv",
        previous / "summary.csv",
        previous / "sources.json",
        obstacles_path,
    ]
    helpers = [
        "analyze_recovery_arcs.py",
        "analyze_recovery_stalls.py",
        "analyze_robust_arc_selection.py",
        "summarize_stopping_preview.py",
        "verify_stopping_preview.py",
    ]
    sources = dict(
        inputs_sha256={str(p.relative_to(ROOT)): digest(p) for p in inputs},
        script_sha256=digest(Path(__file__)),
        helpers_sha256={name: digest(ROOT / name) for name in helpers},
        protocol_sha256=digest(ROOT / "results/arc-pose-components/protocol.md"),
        checkpoint_rule_rows=len(rows),
        distinct_checkpoint_angles=len(cache),
        geometry_audits=len(audits),
        note="Oracle pose substitutions at fixed steering; descriptive geometry, no estimator or controller change.",
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, data in [
        ("components.csv", rows),
        ("geometry-audit.csv", audits),
        ("summary.csv", summaries),
    ]:
        save_rows(args.output / name, data)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    if args.plot:
        plot(args.output, summaries)
    for row in summaries:
        print(row)
    print(f"Checked {len(rows)} rule/checkpoint rows and {len(audits)} chord audits")


if __name__ == "__main__":
    main()
