#!/usr/bin/env python3
"""Run the predeclared 108-case closed-loop mean speed-probe pilot."""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess

import build_mean_speed_probe as build


def run_trial(binary, directory, mode, seed, tau, enabled):
    directory.mkdir(parents=True)
    command = [
        str(binary),
        *build.arguments(build.FIXTURES, directory, mode, seed, tau, enabled),
    ]
    with (directory / "simulation.log").open("w") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    with (directory / "results.csv").open(newline="") as stream:
        results = list(csv.DictReader(stream))
    build.require(
        len(results) == 9
        and len({(r["scenario"], r["planner"]) for r in results}) == 9,
        "Incomplete native case grid",
    )
    return results, command


def run(output):
    pins = build.preflight()
    output = build.fresh(output)
    build.require(
        not subprocess.check_output(
            ["git", "status", "--porcelain=v1"], cwd=build.REPO, text=True
        ).strip(),
        "Commit the implementation before producing pilot outcomes",
    )
    build.require(
        shutil.disk_usage(output.parent).free >= 2 * 1024**3,
        "At least 2 GiB of free disk space is required for the fixed pilot",
    )
    started = datetime.now(timezone.utc).isoformat()
    commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=build.REPO, text=True
    ).strip()
    output.mkdir(parents=True)
    candidate, original, built = build.build(output / "build")
    trials, all_rows = [], []
    for tau in pins["noise_correlation_times"]:
        for mode in pins["modes"]:
            for arm in pins["arms"]:
                relative = f"tau{tau:g}/{arm}-{mode}-{pins['seed']}"
                directory = output / relative
                rows, command = run_trial(
                    candidate, directory, mode, pins["seed"], tau, arm == "mean_probe"
                )
                files = (
                    "trajectories.csv",
                    "results.csv",
                    "speed-probes.csv",
                    "simulation.log",
                )
                trials.append(
                    dict(
                        tau=tau,
                        arm=arm,
                        mode=mode,
                        seed=pins["seed"],
                        directory=relative,
                        command=command,
                        sha256={name: build.sha(directory / name) for name in files},
                    )
                )
                all_rows.extend(
                    dict(tau=tau, arm=arm, mode=mode, seed=pins["seed"], **row)
                    for row in rows
                )
                print(
                    f"{relative}: held={sum(r['held']=='true' for r in rows)}/9 contacts={sum(r['collision']=='true' for r in rows)}",
                    flush=True,
                )
    build.require(len(all_rows) == pins["expected_cases"], "Pilot case count differs")
    reference = output / f"tau0/raw_probe-none-{pins['seed']}"
    regressions = []
    for name, binary, enabled in (
        ("original", original, None),
        ("omitted", candidate, None),
    ):
        directory = output / "regression" / name
        _, command = run_trial(binary, directory, "none", pins["seed"], 0.0, enabled)
        compared = {}
        for filename in ("trajectories.csv", "results.csv"):
            actual, expected = build.sha(directory / filename), build.sha(
                reference / filename
            )
            build.require(
                actual == expected,
                f"Disabled/default output differs from original: {name}/{filename}",
            )
            compared[filename] = actual
        if name == "omitted":
            build.require(
                build.sha(directory / "speed-probes.csv")
                == build.sha(reference / "speed-probes.csv"),
                "Omitted/default sidecar differs",
            )
        regressions.append(dict(name=name, command=command, sha256=compared, cases=9))
        print(
            f"{name}: all nine disabled/default cases match byte for byte", flush=True
        )
    with (output / "all-results.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=tuple(all_rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(all_rows)
    provenance = dict(
        method="fixed 108-case mean-position speed-probe pilot; original recovery window; no retuning",
        started_at=started,
        source_commit=commit,
        working_tree_clean=True,
        runner_sha256=build.sha(Path(__file__)),
        generator_sha256=build.sha(Path(build.__file__)),
        protocol_sha256=build.sha(build.STUDY / "protocol.md"),
        freeze_sha256=build.sha(build.STUDY / "freeze-pins.json"),
        input_sha256=pins["input_sha256"],
        build_sources_sha256=build.sha(output / "build/build-sources.json"),
        generated_source_sha256=built["generated_source_sha256"],
        candidate_binary_sha256=built["candidate_binary_sha256"],
        original_binary_sha256=built["original_binary_sha256"],
        native_tests=built["native_tests"],
        cases=len(all_rows),
        trials=trials,
        regressions=regressions,
        all_results_sha256=build.sha(output / "all-results.csv"),
    )
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(f"Saved and checked all {len(all_rows)} pilot cases: {output}", flush=True)
    return provenance


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not __debug__:
        parser.error("Run without Python optimization")
    try:
        run(args.output)
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Mean speed-probe pilot failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
