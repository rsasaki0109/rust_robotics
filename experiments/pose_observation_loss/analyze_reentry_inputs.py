#!/usr/bin/env python3
"""Decompose frozen release-to-reentry changes without new policy runs."""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path

from analyze_recovery_stalls import digest
from summarize_stopping_preview import save_rows
from verify_stopping_preview import exact_preview

ROOT = Path(__file__).resolve().parent
KEY = ["cohort", "mode", "seed", "scenario", "planner"]
FACTORS = ["physical_pose", "observation_error", "initial_speed", "planned_command"]
TERMS = FACTORS + ["base_margin", "speed_reserve"]


def shapley(values):
    """Average each input's finite difference over all 24 replacement orders."""
    assert len(values) == 16 and all(math.isfinite(v) for v in values)
    result = {name: 0.0 for name in FACTORS}
    for order in itertools.permutations(range(4)):
        mask = 0
        for i in order:
            new_mask = mask | (1 << i)
            result[FACTORS[i]] += (values[new_mask] - values[mask]) / 24
            mask = new_mask
    assert abs(sum(result.values()) - (values[-1] - values[0])) < 1e-10
    return result


def endpoint(before, row):
    result = dict(
        physical_pose=tuple(float(before[k]) for k in ["x", "y", "yaw"]),
        observation_error=tuple(
            float(row[f"noise_{k}"]) + float(row[f"bias_{k}"])
            for k in ["x", "y", "yaw"]
        ),
        initial_speed=float(before["v"]),
        planned_command=(
            float(row["nominal_target_speed"]),
            float(row["nominal_steering"]),
        ),
        base_margin=0.5
        + 2 * math.sqrt(2) * float(row["effective_position_sigma"])
        + 0.02 * float(row["observation_age"]),
        speed_reserve=0.1 * min(abs(float(before["v"])) / 0.4, 1),
    )
    for i, axis in enumerate(["x", "y", "yaw"]):
        assert (
            abs(
                result["physical_pose"][i]
                + result["observation_error"][i]
                - float(row[f"preview_pose_{axis}"])
            )
            < 3e-9
        )
    assert (
        abs(
            result["base_margin"]
            + result["speed_reserve"]
            - float(row["preview_margin"])
        )
        < 3e-9
    )
    assert abs(result["base_margin"] + 0.05 - float(row["guard_release_margin"])) < 3e-9
    return result


def hybrid_preview(start, end, mask, obstacles):
    chosen = {
        name: (end if mask & (1 << i) else start)[name]
        for i, name in enumerate(FACTORS)
    }
    before = dict(zip(["x", "y", "yaw"], chosen["physical_pose"]))
    before["v"] = chosen["initial_speed"]
    row = {
        f"noise_{axis}": error
        for axis, error in zip(["x", "y", "yaw"], chosen["observation_error"])
    }
    row.update({f"bias_{axis}": 0 for axis in ["x", "y", "yaw"]})
    row["nominal_target_speed"], row["nominal_steering"] = chosen["planned_command"]
    return exact_preview(before, row, obstacles)[0]


def decompose(start, end, obstacles):
    clearances = [hybrid_preview(start, end, mask, obstacles) for mask in range(16)]
    contributions = shapley(clearances)
    for name in ["base_margin", "speed_reserve"]:
        contributions[name] = start[name] - end[name]
    start_margin = start["base_margin"] + start["speed_reserve"]
    end_margin = end["base_margin"] + end["speed_reserve"]
    start_slack, end_slack = clearances[0] - start_margin, clearances[-1] - end_margin
    assert abs(sum(contributions.values()) - (end_slack - start_slack)) < 1e-10
    freezes = {
        name: clearances[15 ^ (1 << i)] - end_margin for i, name in enumerate(FACTORS)
    }
    freezes["base_margin"] = (
        clearances[-1] - start["base_margin"] - end["speed_reserve"]
    )
    freezes["speed_reserve"] = (
        clearances[-1] - end["base_margin"] - start["speed_reserve"]
    )
    freezes["speed_both"] = clearances[11] - end["base_margin"] - start["speed_reserve"]
    return clearances, contributions, freezes, start_slack, end_slack


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--white", type=Path, required=True)
    parser.add_argument("--correlated", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    snapshot = ROOT / "results/entry-cycles"
    sources = json.loads((snapshot / "sources.json").read_text())
    selected = [
        r
        for r in csv.DictReader((snapshot / "release-bouts.csv").open())
        if r["short_reentry"] == "True"
    ]
    assert len(selected) == 27
    audits = {
        tuple(r[k] for k in KEY + ["time_s"]): r
        for r in csv.DictReader((snapshot / "guard-events.csv").open())
    }
    records, hybrids, evidence = [], [], {}
    endpoint_error = 0.0
    for cohort, run in [
        ("white-validation", args.white),
        ("correlated-validation", args.correlated),
    ]:
        provenance = json.loads((run / "provenance.json").read_text())
        for field in [
            "variants",
            "noise_correlation_time_s",
            "seeds",
            "simulator_sha256",
            "inputs_by_variant_sha256",
        ]:
            assert provenance[field] == sources[cohort]["provenance"][field]
        assert digest(run / "all-results.csv") == sources[cohort]["summary_sha256"]
        inputs = run / "planning-inputs/fine-padding"
        hashes = {p.name: digest(p) for p in inputs.glob("*.csv")}
        assert hashes == provenance["inputs_by_variant_sha256"]["entry"]
        maps = {}
        for r in csv.DictReader((inputs / "obstacles.csv").open()):
            maps.setdefault(r["scenario"], []).append((float(r["x"]), float(r["y"])))
        cache, raw_hashes = {}, {}
        for b in (r for r in selected if r["cohort"] == cohort):
            path = run / f'entry-{b["mode"]}-{b["seed"]}/trajectories.csv'
            relative = str(path.relative_to(run))
            if path not in cache:
                raw_hashes[relative] = digest(path)
                assert raw_hashes[relative] == sources[cohort]["raw_sha256"][relative]
                wanted = {
                    (r["scenario"], r["planner"])
                    for r in selected
                    if r["cohort"] == cohort
                    and (r["mode"], r["seed"]) == (b["mode"], b["seed"])
                }
                cache[path] = {key: [] for key in wanted}
                with path.open() as stream:
                    for row in csv.DictReader(stream):
                        key = row["scenario"], row["planner"]
                        if key in wanted:
                            cache[path][key].append(row)
            track = cache[path][b["scenario"], b["planner"]]
            indices = [
                round(float(b[name]) / 0.02)
                for name in ["release_time_s", "entry_time_s"]
            ]
            a, z = (track[i] for i in indices)
            assert a["guard_released"] == "true" and z["guard_triggered"] == "true"
            assert (
                a["time_s"] == b["release_time_s"] and z["time_s"] == b["entry_time_s"]
            )
            assert float(a["target_speed"]) > 0 and all(
                r["safety_paused"] == "false"
                for r in track[indices[0] : indices[1] + 1]
            )
            start, end = (endpoint(track[i - 1], track[i]) for i in indices)
            values, changes, freezes, start_slack, end_slack = decompose(
                start, end, maps[b["scenario"]]
            )
            assert start_slack > 2e-8 and end_slack <= 2e-8
            for time, value in [(a["time_s"], values[0]), (z["time_s"], values[15])]:
                old = audits[(*(b[k] for k in KEY), time)]
                error = abs(value - float(old["exact_clearance"]))
                endpoint_error = max(endpoint_error, error)
                assert error < 1e-8
            record = {
                k: b[k]
                for k in KEY
                + [
                    "release_time_s",
                    "entry_time_s",
                    "lag_s",
                    "travel_before_entry",
                    "reserve_only_entry",
                ]
            }
            record.update(
                release_clearance=values[0],
                entry_clearance=values[15],
                release_entry_slack=start_slack,
                release_release_slack=values[0] - start["base_margin"] - 0.05,
                entry_slack=end_slack,
                largest_closing_term=min(changes, key=changes.get),
                release_speed=start["initial_speed"],
                entry_speed=end["initial_speed"],
                release_reserve=start["speed_reserve"],
                entry_reserve=end["speed_reserve"],
                **{f"delta_{name}": value for name, value in changes.items()},
                **{
                    f"freeze_{name}_entry_slack": value
                    for name, value in freezes.items()
                },
            )
            records.append(record)
            for mask, value in enumerate(values):
                hybrids.append(
                    dict(
                        **{
                            k: record[k]
                            for k in KEY + ["release_time_s", "entry_time_s"]
                        },
                        mask=mask,
                        physical_pose_from_entry=bool(mask & 1),
                        observation_error_from_entry=bool(mask & 2),
                        initial_speed_from_entry=bool(mask & 4),
                        planned_command_from_entry=bool(mask & 8),
                        exact_clearance=value,
                    )
                )
        evidence[cohort] = dict(provenance=provenance, raw_sha256=raw_hashes)
    assert len(records) == 27 and len(hybrids) == 432
    summary = [
        dict(
            term=name,
            mean_slack_change=sum(r[f"delta_{name}"] for r in records) / len(records),
            largest_closing_count=sum(
                r["largest_closing_term"] == name for r in records
            ),
            freeze_restores_entry_pass=sum(
                r[f"freeze_{name}_entry_slack"] > 2e-8 for r in records
            ),
        )
        for name in TERMS
    ]
    evidence["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        helper_sha256={
            name: digest(ROOT / name)
            for name in [
                "analyze_recovery_stalls.py",
                "summarize_stopping_preview.py",
                "verify_stopping_preview.py",
            ]
        },
        pinned_sha256={
            name: digest(snapshot / name)
            for name in ["sources.json", "release-bouts.csv", "guard-events.csv"]
        },
        selection="all 27 recorded short reentries in the ten lost-hold pairs",
        hybrid_geometry_count=432,
        endpoint_count=54,
        max_endpoint_clearance_error=endpoint_error,
        geometry_factor_order=FACTORS,
        attribution="mean of all 24 replacement orders; margin changes are additive",
        pass_tolerance=2e-8,
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, rows in [
        ("reentries.csv", records),
        ("hybrids.csv", hybrids),
        ("summary.csv", summary),
    ]:
        save_rows(args.output / name, rows)
    (args.output / "sources.json").write_text(json.dumps(evidence, indent=2) + "\n")
    for row in summary:
        print(row)
    print(
        f"Reproduced 54 endpoint forecasts and 432 hybrid forecasts; max endpoint error {endpoint_error:.3g}"
    )


if __name__ == "__main__":
    main()
