#!/usr/bin/env python3
"""Compare verified correlation-time runs and inspect a fixed noise prefix."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics


def read_noise(run, seed):
    """Use the same open-map, pre-terminal 50 s in every condition."""
    samples, innovations = [], []
    source = run / f"aware-none-{seed}" / "trajectories.csv"
    with source.open() as handle:
        for row in csv.DictReader(handle):
            if (row["scenario"], row["planner"]) != ("open_50x50", "A*"):
                continue
            if float(row["time_s"]) == 0:
                continue
            assert row["precision_active"] == "false"
            sigma = [0.1, 0.1, math.radians(2)]
            latent = tuple(
                float(row[f"noise_{axis}"]) / s
                for axis, s in zip(["x", "y", "yaw"], sigma)
            )
            innovation = (
                tuple(
                    float(row[f"noise_innovation_{axis}"]) for axis in ["x", "y", "yaw"]
                )
                if "noise_innovation_x" in row
                else latent
            )
            samples.append(latent)
            innovations.append(innovation)
            if len(samples) == 2500:
                break
    assert len(samples) == 2500, (run, seed, "incomplete fixed noise window")
    return samples, innovations


def check_contacts(run, tau, rows, expected_inputs):
    """Recompute final swept-segment clearance from logged true positions."""
    fixture = Path(__file__).resolve().parent / "inputs" / "obstacles.csv"
    assert (
        hashlib.sha256(fixture.read_bytes()).hexdigest()
        == expected_inputs["obstacles.csv"]
    )
    obstacles = {}
    with fixture.open() as handle:
        for row in csv.DictReader(handle):
            obstacles.setdefault(row["scenario"], []).append(
                (float(row["x"]), float(row["y"]))
            )
    diagnostics = []
    for result in rows:
        if result["collision"] != "true":
            continue
        source = (
            run
            / f'{result["variant"]}-{result["mode"]}-{result["seed"]}'
            / "trajectories.csv"
        )
        tail = []
        with source.open() as handle:
            for row in csv.DictReader(handle):
                if (row["scenario"], row["planner"]) == (
                    result["scenario"],
                    result["planner"],
                ):
                    tail = (tail + [row])[-2:]
        before, after = tail
        ax, ay, bx, by = (
            float(v) for v in (before["x"], before["y"], after["x"], after["y"])
        )
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy

        def distance(point):
            t = (
                max(0, min(1, ((point[0] - ax) * dx + (point[1] - ay) * dy) / length2))
                if length2
                else 0
            )
            return math.hypot(point[0] - ax - t * dx, point[1] - ay - t * dy)

        closest = min(obstacles[result["scenario"]], key=distance)
        clearance = distance(closest)
        assert (
            clearance <= 0.5
            and abs(clearance - float(after["step_min_obstacle_distance"])) < 2e-9
        )
        diagnostics.append(
            dict(
                tau_s=tau,
                **{
                    key: result[key]
                    for key in [
                        "variant",
                        "mode",
                        "seed",
                        "scenario",
                        "planner",
                        "time_s",
                    ]
                },
                obstacle_x=closest[0],
                obstacle_y=closest[1],
                recomputed_swept_distance=clearance,
                safety_paused=after["safety_paused"],
                observation_age=after["observation_age"],
            )
        )
    return diagnostics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "runs", type=Path, nargs="+", help="Include one white-noise run"
    )
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O; assertions are required")
    runs = {}
    for run in args.runs:
        provenance = json.loads((run / "provenance.json").read_text())
        tau = provenance.get("noise_correlation_time_s", 0.0)
        assert tau not in runs and tau >= 0 and math.isfinite(tau)
        rows = list(csv.DictReader((run / "all-results.csv").open()))
        runs[tau] = (run, provenance, rows)
    assert 0.0 in runs
    white_run, white_provenance, white_rows = runs[0.0]
    seeds = sorted({int(r["seed"]) for r in white_rows})
    keys = {
        (r["mode"], r["variant"], r["seed"], r["scenario"], r["planner"])
        for r in white_rows
    }
    assert len(keys) == len(white_rows)
    white_noise = {seed: read_noise(white_run, seed)[1] for seed in seeds}
    summaries, noise_rows, contacts = [], [], []
    for tau, (run, provenance, rows) in sorted(runs.items()):
        assert provenance["inputs_sha256"] == white_provenance["inputs_sha256"]
        assert {
            (r["mode"], r["variant"], r["seed"], r["scenario"], r["planner"])
            for r in rows
        } == keys
        assert len(rows) == len(keys)
        contacts.extend(check_contacts(run, tau, rows, provenance["inputs_sha256"]))
        series = []
        for seed in seeds:
            samples, innovations = read_noise(run, seed)
            assert all(
                abs(a - b) < 2e-7
                for left, right in zip(innovations, white_noise[seed])
                for a, b in zip(left, right)
            ), (tau, seed, "unpaired innovations")
            series.append(samples)
        rho = math.exp(-0.02 / tau) if tau else 0.0
        n = 25
        window_sd = math.sqrt(
            (n + 2 * sum((n - k) * rho**k for k in range(1, n))) / n**2
        )
        for axis, name in enumerate(["x", "y", "yaw"]):
            values = [v[axis] for block in series for v in block]
            mean = statistics.mean(values)
            variance = statistics.pvariance(values)
            left = [v[axis] for block in series for v in block[:-1]]
            right = [v[axis] for block in series for v in block[1:]]
            acf = (
                statistics.mean((a - mean) * (b - mean) for a, b in zip(left, right))
                / variance
            )
            # Broad sanity bounds; these are not confidence intervals or efficacy tests.
            assert abs(mean) < 0.3 and 0.5 < variance < 1.5 and abs(acf - rho) < 0.03, (
                tau,
                name,
                mean,
                variance,
                acf,
            )
            noise_rows.append(
                dict(
                    tau_s=tau,
                    axis=name,
                    samples=len(values),
                    mean=mean,
                    variance=variance,
                    lag1_correlation=acf,
                    expected_lag1=rho,
                    theoretical_25_sample_sd=window_sd,
                )
            )
        for mode in ["none", "random", "blackout"]:
            for variant in ["aware", "averaged"]:
                selected = [
                    r for r in rows if r["mode"] == mode and r["variant"] == variant
                ]
                assert len(selected) == 9 * len(seeds)
                summaries.append(
                    dict(
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
                        mean_wait_s=statistics.mean(
                            float(r["paused_time_s"]) for r in selected
                        ),
                    )
                )
    args.output.mkdir(parents=True, exist_ok=False)
    output_rows = [
        ("summary.csv", summaries),
        ("noise-statistics.csv", noise_rows),
    ]
    if contacts:
        output_rows.append(("contacts.csv", contacts))
    for name, rows in output_rows:
        with (args.output / name).open("w", newline="") as target:
            writer = csv.DictWriter(
                target, fieldnames=list(rows[0]), lineterminator="\n"
            )
            writer.writeheader()
            writer.writerows(rows)
    (args.output / "sources.json").write_text(
        json.dumps(
            {
                str(tau): dict(provenance=provenance)
                for tau, (_, provenance, _) in sorted(runs.items())
            },
            indent=2,
        )
        + "\n"
    )
    for row in summaries:
        print(row)
    print("Fixed-window noise checks and cross-condition innovation pairing passed.")


if __name__ == "__main__":
    main()
