#!/usr/bin/env python3
"""Decompose filtered position error on 497 pinned failed-run checkpoints."""

import argparse
from collections import deque
import csv
import json
import math
from pathlib import Path
import statistics

from analyze_recovery_stalls import digest
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
KEY = ["cohort", "mode", "seed", "scenario", "planner", "time_s"]
PARTS = ["noise", "anchor", "drift", "transport"]


def effective_samples(n, rho):
    assert n >= 1 and 0 <= rho <= 1
    return n * n / (n + 2 * sum((n - k) * rho**k for k in range(1, n)))


def decompose(samples, truth):
    n = len(samples)
    vectors = {name: [] for name in PARTS}
    error = []
    for axis in ["x", "y"]:
        noise = sum(s[f"noise_{axis}"] for s in samples) / n
        anchor = sum(s[f"anchor_{axis}"] for s in samples) / n
        drift = sum(s[f"bias_{axis}"] - s[f"anchor_{axis}"] for s in samples) / n
        transport = (
            sum(s[f"true_{axis}"] + s[f"delta_{axis}"] - truth[axis] for s in samples)
            / n
        )
        for name, value in zip(PARTS, [noise, anchor, drift, transport]):
            vectors[name].append(value)
        error.append(
            sum(
                s[f"true_{axis}"]
                + s[f"noise_{axis}"]
                + s[f"bias_{axis}"]
                + s[f"delta_{axis}"]
                for s in samples
            )
            / n
            - truth[axis]
        )
        assert abs(sum(vectors[name][-1] for name in PARTS) - error[-1]) < 1e-10
    magnitude = math.hypot(*error)
    row = dict(sample_count=n, error_x=error[0], error_y=error[1], error_norm=magnitude)
    for name, vector in vectors.items():
        row.update(
            {
                f"{name}_x": vector[0],
                f"{name}_y": vector[1],
                f"{name}_norm": math.hypot(*vector),
                f"{name}_projection": (
                    sum(a * b for a, b in zip(vector, error)) / magnitude
                    if magnitude > 1e-12
                    else 0.0
                ),
            }
        )
    assert abs(sum(row[f"{name}_projection"] for name in PARTS) - magnitude) < 1e-10
    row["largest_projection"] = (
        max(PARTS, key=lambda k: row[f"{k}_projection"])
        if magnitude > 1e-12
        else "none"
    )
    assert len({s["sigma"] for s in samples}) == len({s["rho"] for s in samples}) == 1
    row["noise_effective_samples"] = effective_samples(n, samples[0]["rho"])
    row["theoretical_noise_sigma_per_axis"] = samples[0]["sigma"] / math.sqrt(
        row["noise_effective_samples"]
    )
    return row


class Window:
    """Direct sample transport, separate from the existing anchor-frame audit."""

    def __init__(self):
        self.samples = deque(maxlen=25)
        self.previous_motion = None
        self.anchor = dict(x=0.0, y=0.0)
        self.anchor_time = -1.0
        self.max_error = 0.0

    def advance(self, before, row):
        if row["correction"] == "true" or row["precision_activation"] == "true":
            self.samples.clear()
        if (
            row["correction_received"] == "true"
            or row["precision_activation"] == "true"
        ):
            self.anchor = {a: float(row[f"bias_{a}"]) for a in ["x", "y"]}
            self.anchor_time = float(row["time_s"])
        if self.previous_motion is not None:
            speed, steering = self.previous_motion
            distance = speed * 0.02
            turn = distance / 0.4 * math.tan(steering)
            for s in self.samples:
                s["delta_x"] += distance * math.cos(s["heading"])
                s["delta_y"] += distance * math.sin(s["heading"])
                s["heading"] = math.atan2(
                    math.sin(s["heading"] + turn), math.cos(s["heading"] + turn)
                )
        sample = dict(
            sample_time_s=row["time_s"],
            anchor_time_s=self.anchor_time,
            delta_x=0.0,
            delta_y=0.0,
            heading=float(before["yaw"])
            + float(row["noise_yaw"])
            + float(row["bias_yaw"]),
            sigma=float(row["effective_position_sigma"]),
            rho=float(row["noise_rho"]),
        )
        for axis in ["x", "y"]:
            sample.update(
                {
                    f"true_{axis}": float(before[axis]),
                    f"noise_{axis}": float(row[f"noise_{axis}"]),
                    f"bias_{axis}": float(row[f"bias_{axis}"]),
                    f"anchor_{axis}": self.anchor[axis],
                }
            )
        self.samples.append(sample)
        n = len(self.samples)
        assert n == int(row["motion_filter_sample_count"])
        for axis in ["x", "y"]:
            reconstructed = (
                sum(
                    s[f"true_{axis}"]
                    + s[f"noise_{axis}"]
                    + s[f"bias_{axis}"]
                    + s[f"delta_{axis}"]
                    for s in self.samples
                )
                / n
            )
            error = abs(reconstructed - float(row[f"preview_pose_{axis}"]))
            assert error < 1e-8, (axis, row["time_s"], error)
            self.max_error = max(self.max_error, error)
        heading = math.atan2(
            sum(math.sin(s["heading"]) for s in self.samples),
            sum(math.cos(s["heading"]) for s in self.samples),
        )
        error = heading - float(row["preview_pose_yaw"])
        error = abs(math.atan2(math.sin(error), math.cos(error)))
        assert error < 1e-8, ("yaw", row["time_s"], error)
        self.max_error = max(self.max_error, error)
        self.previous_motion = float(before["v"]), float(row["steering"])


def summarize(rows):
    result = []
    for cohort in ["white-validation", "correlated-validation"]:
        for reason in ["final", "fresh_stationary"]:
            group = [r for r in rows if r["cohort"] == cohort and r["reason"] == reason]
            assert group
            row = dict(
                cohort=cohort,
                reason=reason,
                checkpoints=len(group),
                sample_count_min=min(r["sample_count"] for r in group),
                sample_count_max=max(r["sample_count"] for r in group),
                mean_error=statistics.mean(r["error_norm"] for r in group),
                median_error=statistics.median(r["error_norm"] for r in group),
                rms_error=math.sqrt(
                    statistics.mean(r["error_norm"] ** 2 for r in group)
                ),
                mean_noise_effective_samples=statistics.mean(
                    r["noise_effective_samples"] for r in group
                ),
                theoretical_noise_sigma_per_axis=statistics.mean(
                    r["theoretical_noise_sigma_per_axis"] for r in group
                ),
            )
            for name in PARTS:
                row[f"rms_{name}"] = math.sqrt(
                    statistics.mean(r[f"{name}_norm"] ** 2 for r in group)
                )
                row[f"mean_{name}_projection"] = statistics.mean(
                    r[f"{name}_projection"] for r in group
                )
                row[f"{name}_largest"] = sum(
                    r["largest_projection"] == name for r in group
                )
            assert (
                abs(
                    sum(row[f"mean_{name}_projection"] for name in PARTS)
                    - row["mean_error"]
                )
                < 1e-10
            )
            result.append(row)
    return result


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--white", type=Path, required=True)
    parser.add_argument("--correlated", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    pinned = ROOT / "results/aligned-arcs"
    provenance = json.loads((pinned / "sources.json").read_text())
    simulator = (
        ROOT.parents[1]
        / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
    )
    filtered = [
        r for r in read_rows(pinned / "checkpoints.csv") if r["pose_kind"] == "filtered"
    ]
    assert len(filtered) == 497
    expected = {tuple(r[k] for k in KEY): r for r in filtered}
    assert len(expected) == 497
    checkpoint_rows, sample_rows, raw_hashes = [], [], {}
    steps = 0
    max_error = 0.0
    for cohort, run in [
        ("white-validation", args.white),
        ("correlated-validation", args.correlated),
    ]:
        source = provenance[cohort]
        assert json.loads((run / "provenance.json").read_text()) == source["provenance"]
        assert digest(simulator) == source["provenance"]["simulator_sha256"]
        cohort_rows = [r for r in filtered if r["cohort"] == cohort]
        hashes = {}
        for mode, seed in sorted({(r["mode"], r["seed"]) for r in cohort_rows}):
            selected = {
                (r["scenario"], r["planner"])
                for r in cohort_rows
                if (r["mode"], r["seed"]) == (mode, seed)
            }
            relative = f"aligned-{mode}-{seed}/trajectories.csv"
            path = run / relative
            hashes[relative] = digest(path)
            assert hashes[relative] == source["raw_sha256"][relative]
            windows = {k: Window() for k in selected}
            previous = {}
            with path.open() as stream:
                for row in csv.DictReader(stream):
                    track_key = row["scenario"], row["planner"]
                    if track_key not in windows:
                        continue
                    before = previous.get(track_key)
                    previous[track_key] = row
                    if before is None:
                        assert float(row["time_s"]) == 0
                        continue
                    assert (
                        abs(float(row["time_s"]) - float(before["time_s"]) - 0.02)
                        < 1e-8
                    )
                    window = windows[track_key]
                    window.advance(before, row)
                    steps += 1
                    key = cohort, mode, seed, *track_key, row["time_s"]
                    if key not in expected:
                        continue
                    pinned_row = expected[key]
                    identity = {
                        k: pinned_row[k] for k in [*KEY, "decision_time_s", "reason"]
                    }
                    assert before["time_s"] == identity["decision_time_s"]
                    values = decompose(
                        window.samples, {a: float(before[a]) for a in ["x", "y"]}
                    )
                    assert values["sample_count"] == int(
                        pinned_row["motion_filter_sample_count"]
                    )
                    for axis in ["x", "y"]:
                        assert (
                            abs(
                                float(row[f"preview_pose_{axis}"])
                                - float(pinned_row[axis])
                            )
                            < 1e-12
                        )
                        assert (
                            abs(
                                values[f"error_{axis}"]
                                - (float(pinned_row[axis]) - float(before[axis]))
                            )
                            < 1e-8
                        )
                    checkpoint_rows.append(dict(**identity, **values))
                    sample_rows.extend(dict(**identity, **s) for s in window.samples)
            assert all(float(r["time_s"]) == 180 for r in previous.values())
            max_error = max(max_error, *(w.max_error for w in windows.values()))
            print(f"Checked {cohort} {mode} {seed}: {len(selected)} tracks", flush=True)
        raw_hashes[cohort] = hashes
    assert steps == 729000
    assert len(checkpoint_rows) == 497 and len(sample_rows) == 10481
    assert len({tuple(r[k] for k in KEY) for r in checkpoint_rows}) == 497
    summary = summarize(checkpoint_rows)
    sources = dict(
        inputs_sha256={
            name: digest(pinned / name) for name in ["checkpoints.csv", "sources.json"]
        },
        raw_sha256=raw_hashes,
        script_sha256=digest(Path(__file__)),
        simulator_sha256=digest(simulator),
        helpers_sha256={
            name: digest(ROOT / name)
            for name in [
                "analyze_recovery_stalls.py",
                "summarize_stopping_preview.py",
                "verify_stopping_preview.py",
            ]
        },
        protocol_sha256=digest(ROOT / "results/filter-position-components/protocol.md"),
        filter_steps_checked=steps,
        checkpoints=len(checkpoint_rows),
        retained_sample_rows=len(sample_rows),
        max_logged_filter_error=max_error,
        note="Exact logged-noise decomposition on selected failed runs; no new estimator, drive or pose-confidence claim.",
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, rows in [
        ("checkpoints.csv", checkpoint_rows),
        ("window-samples.csv", sample_rows),
        ("summary.csv", summary),
    ]:
        save_rows(args.output / name, rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    for row in summary:
        print(row)


if __name__ == "__main__":
    main()
