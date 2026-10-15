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


def release_entry_gaps(audits):
    """Describe audited candidate guard events; includes zero-speed releases."""
    key_fields = ["mode", "seed", "scenario", "planner"]
    groups = {}
    for row in audits:
        groups.setdefault(tuple(row[k] for k in key_fields), []).append(row)
    records = []
    for key, rows in groups.items():
        record = dict(zip(key_fields, key))
        record.update(
            guard_releases=0,
            entry_blocked_at_release=0,
            immediate_reentries=0,
            blocked_release_then_immediate_entry=0,
        )
        release = None
        for row in sorted(rows, key=lambda r: float(r["time_s"])):
            if row["guard_triggered"] == "true" and release is not None:
                lag = float(row["time_s"]) - release[0]
                assert lag > 0
                immediate = abs(lag - 0.02) < 1e-8
                record["immediate_reentries"] += immediate
                record["blocked_release_then_immediate_entry"] += (
                    immediate and release[1]
                )
                release = None
            if row["guard_released"] == "true":
                blocked = float(row["lower_bound"]) <= float(row["margin"])
                record["guard_releases"] += 1
                record["entry_blocked_at_release"] += blocked
                release = float(row["time_s"]), blocked
        records.append(record)
    return records


def plot(directory, summary, variants):
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
                for variant in variants
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
                ylabel="Paired cases",
            )
            labels = {v: v for v in variants}
            if "clearance" in variants:
                labels = {"guarded": "Nominal path", "clearance": "Padded path"}
            if "paced" in variants:
                labels = {"clearance": "Original speed", "paced": "Arc speed cap"}
            if "joint" in variants:
                labels = {"paced": "Separate gates", "joint": "Joint recovery"}
            if "buffered" in variants:
                labels = {"joint": "Original guard", "buffered": "Guard +0.1"}
            if "entry" in variants:
                labels = {"joint": "Original guard", "entry": "Entry reserve"}
            if "filtered" in variants:
                labels = {"joint": "Original guard", "filtered": "Filtered pose"}
            if "recoverable" in variants:
                labels = {
                    "filtered": "Filtered stop",
                    "recoverable": "Recovery line",
                }
            if "entryline" in variants:
                labels = {
                    "recoverable": "Combined",
                    "entryline": "Stop only",
                }
            if "aligned" in variants:
                labels = {"entryline": "Stop only", "aligned": "Aligned"}
            ax.set_xticks(
                range(4),
                [
                    f"{noise}\n{labels[variant]}"
                    for noise in ["White", "1 s"]
                    for variant in variants
                ],
                fontsize=8,
            )
        axes[-1].legend(
            loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=3, fontsize=8
        )
        title = (
            "Restart consistency" if "consistent" in variants else "Stopping preview"
        )
        prefix = (
            "restart-consistency" if "consistent" in variants else "stopping-preview"
        )
        if "clearance" in variants:
            title, prefix = "Planning clearance", "planning-clearance"
        if "paced" in variants:
            title, prefix = "Anticipatory speed", "anticipatory-speed"
        if "joint" in variants:
            title, prefix = "Joint recovery", "joint-recovery"
        if "buffered" in variants:
            title, prefix = "Recovery guard buffer", "guard-buffer"
        if "entry" in variants:
            title, prefix = "Motion entry buffer", "entry-buffer"
        if "filtered" in variants:
            title, prefix = "Motion-compensated preview", "motion-filter"
        if "recoverable" in variants:
            title, prefix = "Terminal recovery preview", "recovery-preview"
        if "entryline" in variants:
            title, prefix = "Terminal entry, stop-only release", "terminal-entry-only"
        if "aligned" in variants:
            title, prefix = "Entry-compatible terminal release", "aligned-release"
        fig.suptitle(f"{title}: fixed settings, paired new seeds, 180 s budget")
        svg = directory / f"{prefix}.svg"
        fig.savefig(svg, metadata={"Date": None})
        svg.write_text(
            "\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n"
        )
        fig.savefig(directory / f"{prefix}.png", dpi=160)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument(
        "--known-white", type=Path, help="Optional known white-noise joint run"
    )
    parser.add_argument("--white", type=Path, required=True)
    parser.add_argument("--correlated", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    parser.add_argument("--plot", action="store_true", help="Requires matplotlib")
    comparison = parser.add_mutually_exclusive_group()
    comparison.add_argument(
        "--restart-consistency",
        action="store_true",
        help="Summarize guarded/consistent instead of averaged/guarded",
    )
    comparison.add_argument(
        "--path-clearance",
        action="store_true",
        help="Summarize fine-grid nominal/padded paths",
    )
    comparison.add_argument(
        "--anticipatory-speed",
        action="store_true",
        help="Summarize padded paths with/without arc-based speed capping",
    )
    comparison.add_argument(
        "--joint-recovery",
        action="store_true",
        help="Summarize paced control with/without joint recovery",
    )
    comparison.add_argument(
        "--guard-buffer",
        action="store_true",
        help="Summarize joint recovery with/without the extra guard buffer",
    )
    comparison.add_argument(
        "--entry-buffer",
        action="store_true",
        help="Summarize motion-dependent entry buffer with original release",
    )
    comparison.add_argument(
        "--motion-filter",
        action="store_true",
        help="Summarize motion-compensated stopping-preview poses",
    )
    comparison.add_argument(
        "--recovery-preview",
        action="store_true",
        help="Summarize filtered terminal recovery previews",
    )
    comparison.add_argument(
        "--terminal-entry-only",
        action="store_true",
        help="Summarize terminal entry with stop-only release",
    )
    comparison.add_argument(
        "--aligned-release",
        action="store_true",
        help="Summarize entry-compatible terminal release",
    )
    args = parser.parse_args()
    if args.known_white and not (
        args.joint_recovery
        or args.guard_buffer
        or args.entry_buffer
        or args.motion_filter
        or args.recovery_preview
        or args.terminal_entry_only
        or args.aligned_release
    ):
        parser.error(
            "--known-white requires --joint-recovery, --guard-buffer, --entry-buffer, --motion-filter or --recovery-preview"
        )
    variants = (
        ["guarded", "consistent"]
        if args.restart_consistency
        else ["averaged", "guarded"]
    )
    if args.path_clearance:
        variants = ["guarded", "clearance"]
    if args.anticipatory_speed:
        variants = ["clearance", "paced"]
    if args.joint_recovery:
        variants = ["paced", "joint"]
    if args.guard_buffer:
        variants = ["joint", "buffered"]
    if args.entry_buffer:
        variants = ["joint", "entry"]
    if args.motion_filter:
        variants = ["joint", "filtered"]
    if args.recovery_preview:
        variants = ["filtered", "recoverable"]
    if args.terminal_entry_only:
        variants = ["recoverable", "entryline"]
    if args.aligned_release:
        variants = ["entryline", "aligned"]
    if not __debug__:
        parser.error("Run without Python -O")
    combined = []
    summary = []
    transitions = []
    combined_events, combined_audits, combined_contacts = [], [], []
    combined_release_gaps = []
    sources = {}
    validation_seeds = None
    cohorts = [
        ("reference", args.reference, 1),
        ("white-validation", args.white, 0),
        ("correlated-validation", args.correlated, 1),
    ]
    if args.known_white:
        cohorts.insert(1, ("known-white", args.known_white, 0))
    for cohort, run, tau in cohorts:
        provenance = json.loads((run / "provenance.json").read_text())
        assert provenance["noise_correlation_time_s"] == tau
        assert provenance["variants"] == variants
        rows = list(csv.DictReader((run / "all-results.csv").open()))
        events = list(csv.DictReader((run / "control-events.csv").open()))
        if (
            args.joint_recovery
            or args.guard_buffer
            or args.entry_buffer
            or args.motion_filter
            or args.recovery_preview
            or args.terminal_entry_only
            or args.aligned_release
        ):
            assert all(
                r.get(k, "") != ""
                for r in events
                for k in [
                    "guard_releases_while_loss_paused",
                    "guard_releases_with_zero_nominal",
                    "loss_resumes_with_guard",
                    "simultaneous_releases",
                    "final_window_s",
                    "final_window_travel",
                    "final_window_wait_s",
                ]
            ), "Rerun verify.py to produce joint recovery event metrics"
        audits = list(csv.DictReader((run / "preview-audit.csv").open()))
        if args.terminal_entry_only or args.aligned_release:
            gaps = release_entry_gaps(audits)
            for gap in gaps:
                event = next(
                    r
                    for r in events
                    if r["variant"] == variants[-1]
                    and all(
                        r[k] == gap[k] for k in ["mode", "seed", "scenario", "planner"]
                    )
                )
                assert gap["guard_releases"] == int(event["guard_exits"])
            combined_release_gaps.extend(dict(cohort=cohort, **r) for r in gaps)
        contacts = (
            list(csv.DictReader((run / "contacts.csv").open()))
            if (run / "contacts.csv").exists()
            else []
        )
        assert len(contacts) == sum(r["collision"] == "true" for r in rows)
        seeds = sorted({r["seed"] for r in rows})
        assert len(rows) == 54 * len(seeds) and audits
        if cohort in ["white-validation", "correlated-validation"]:
            if validation_seeds is None:
                validation_seeds = seeds
            else:
                assert seeds == validation_seeds
        sources[cohort] = dict(provenance=provenance, geometry_checkpoints=len(audits))
        combined.extend(dict(cohort=cohort, **r) for r in rows)
        combined_events.extend(dict(cohort=cohort, **r) for r in events)
        combined_audits.extend(dict(cohort=cohort, **r) for r in audits)
        combined_contacts.extend(dict(cohort=cohort, **r) for r in contacts)
        lookup = {
            (r["variant"], r["mode"], r["seed"], r["scenario"], r["planner"]): r
            for r in rows
        }
        for mode in ["none", "random", "blackout"]:
            gained = lost = 0
            for variant in variants:
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
                if "paced" in variants or any(
                    v in variants
                    for v in [
                        "buffered",
                        "entry",
                        "filtered",
                        "recoverable",
                        "entryline",
                        "aligned",
                    ]
                ):
                    summary[-1].update(
                        mean_speed_limited_time_s=statistics.mean(
                            float(r["speed_limited_time_s"]) for r in ev
                        ),
                        mean_speed_braking_time_s=statistics.mean(
                            float(r["speed_braking_time_s"]) for r in ev
                        ),
                    )
                if (
                    args.joint_recovery
                    or args.guard_buffer
                    or args.entry_buffer
                    or args.motion_filter
                    or args.recovery_preview
                    or args.terminal_entry_only
                    or args.aligned_release
                ):
                    summary[-1].update(
                        {
                            k: sum(int(r[k]) for r in ev)
                            for k in [
                                "guard_releases_while_loss_paused",
                                "guard_releases_with_zero_nominal",
                                "loss_resumes_with_guard",
                                "simultaneous_releases",
                            ]
                        }
                    )
                    summary[-1]["near_stationary_timeouts"] = sum(
                        r["status"] == "timeout"
                        and float(r["final_window_travel"]) < 0.1
                        for r in ev
                    )
                if (
                    args.guard_buffer
                    or args.entry_buffer
                    or args.motion_filter
                    or args.recovery_preview
                    or args.terminal_entry_only
                    or args.aligned_release
                ):
                    summary[-1].update(
                        mean_wait_time_s=statistics.mean(
                            float(r["total_wait_time_s"]) for r in ev
                        ),
                        mean_stationary_wait_time_s=statistics.mean(
                            float(r["stationary_wait_time_s"]) for r in ev
                        ),
                    )
                if (
                    args.motion_filter
                    or args.recovery_preview
                    or args.terminal_entry_only
                    or args.aligned_release
                ):
                    summary[-1]["short_reentries"] = sum(
                        int(r["short_reentries"]) for r in ev
                    )
                if (
                    args.recovery_preview
                    or args.terminal_entry_only
                    or args.aligned_release
                ):
                    summary[-1]["terminal_only_guard_entries"] = sum(
                        int(r["terminal_only_guard_entries"]) for r in ev
                    )
                if variant == variants[-1]:
                    for r in selected:
                        ref = lookup[
                            (variants[0], mode, r["seed"], r["scenario"], r["planner"])
                        ]
                        gained += ref["held"] == "false" and r["held"] == "true"
                        lost += ref["held"] == "true" and r["held"] == "false"
            transitions.append(
                dict(cohort=cohort, mode=mode, gained_holds=gained, lost_holds=lost)
            )
    if (
        args.joint_recovery
        or args.guard_buffer
        or args.entry_buffer
        or args.motion_filter
        or args.recovery_preview
        or args.terminal_entry_only
        or args.aligned_release
    ):
        known = {
            r["seed"] for r in combined if r["cohort"] in ["reference", "known-white"]
        }
        assert not known.intersection(validation_seeds), "Known and new seeds overlap"
    args.output.mkdir(parents=True, exist_ok=False)
    for name, rows in [
        ("all-results.csv", combined),
        ("summary.csv", summary),
        ("paired-holds.csv", transitions),
        ("control-events.csv", combined_events),
        ("preview-audit.csv", combined_audits),
    ]:
        save_rows(args.output / name, rows)
    if combined_contacts:
        save_rows(args.output / "contacts.csv", combined_contacts)
    if combined_release_gaps:
        save_rows(args.output / "release-entry-gaps.csv", combined_release_gaps)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    if args.plot:
        plot(args.output, summary, variants)
    for row in summary:
        print(row)


if __name__ == "__main__":
    main()
