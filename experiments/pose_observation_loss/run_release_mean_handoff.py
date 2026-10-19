#!/usr/bin/env python3
"""Run the fixed 108-case live guard-preview pilot and 72 byte regressions."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess

import analyze_mean_speed_probe as pilot
import build_release_mean_handoff as build

METHOD = "fixed 108-case one-decision post-release mean handoff pilot; both mean selectors enabled; no retuning"
FILES = (
    "trajectories.csv",
    "results.csv",
    "speed-probes.csv",
    "guard-preview-inputs.csv",
    "handoff-inputs.csv",
    "simulation.log",
)
BYTE_FILES = (
    "trajectories.csv",
    "results.csv",
    "speed-probes.csv",
    "guard-preview-inputs.csv",
)


def trial(binary, directory, arguments, files):
    directory.mkdir(parents=True)
    command = [str(binary), *arguments]
    with (directory / "simulation.log").open("x") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    rows = pilot.read(directory / "results.csv")
    build.require(
        len(rows) == len({(r["scenario"], r["planner"]) for r in rows}) == 9,
        "Native case grid differs",
    )
    return rows, command, {name: build.sha(directory / name) for name in files}


def regression_plan(pins):
    records = [
        dict(name=f"legacy-tau{tau:g}-{mode}", kind="legacy", tau=tau, mode=mode)
        for tau in pins["noise_correlation_times"]
        for mode in pins["modes"]
    ]
    return [
        *records,
        dict(name="omitted", kind="omitted", tau=0, mode="random"),
        dict(name="original", kind="original", tau=0, mode="none"),
    ]


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
        shutil.disk_usage(output.parent).free >= pins["minimum_free_bytes"],
        "At least 5 GiB free is required for the fixed handoff pilot",
    )
    commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=build.REPO, text=True
    ).strip()
    started = datetime.now(timezone.utc).isoformat()
    output.mkdir(parents=True)
    try:
        candidate, legacy, original, receipt = build.build(output / "build")
        trials, all_rows = [], []
        for tau in pins["noise_correlation_times"]:
            for mode in pins["modes"]:
                for arm in pins["arms"]:
                    relative = f"tau{tau:g}/{arm}-{mode}-{pins['seed']}"
                    directory = output / relative
                    rows, command, hashes = trial(
                        candidate,
                        directory,
                        build.arguments(
                            build.FIXTURES,
                            directory,
                            mode,
                            pins["seed"],
                            tau,
                            arm == "one_step_handoff",
                        ),
                        FILES,
                    )
                    trials.append(
                        dict(
                            tau=tau,
                            arm=arm,
                            mode=mode,
                            seed=pins["seed"],
                            directory=relative,
                            command=command,
                            sha256=hashes,
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
        build.require(
            len(all_rows) == pins["expected_cases"], "Pilot case count differs"
        )
        regressions = []
        for spec in regression_plan(pins):
            kind, tau, mode = spec["kind"], spec["tau"], spec["mode"]
            directory = output / "regression" / spec["name"]
            if kind == "legacy":
                binary = legacy
                args = build.legacy.arguments(
                    build.FIXTURES, directory, mode, pins["seed"], tau, True
                )
            elif kind == "omitted":
                binary = candidate
                args = build.arguments(
                    build.FIXTURES, directory, mode, pins["seed"], tau
                )
            else:
                binary = original
                args = build.legacy.legacy.arguments(
                    build.FIXTURES, directory, mode, pins["seed"], tau
                )
            files = tuple(
                name
                for name in FILES
                if (kind == "omitted" or name != "handoff-inputs.csv")
                and (
                    kind != "original"
                    or name in ("trajectories.csv", "results.csv", "simulation.log")
                )
            )
            _, command, hashes = trial(binary, directory, args, files)
            reference = output / f"tau{tau:g}/mean_preview-{mode}-{pins['seed']}"
            compared = BYTE_FILES[:2] if kind == "original" else BYTE_FILES
            for filename in compared:
                build.require(
                    hashes[filename] == build.sha(reference / filename),
                    f"Disabled/default byte regression differs: {spec['name']}/{filename}",
                )
            regressions.append(
                dict(
                    **spec,
                    command=command,
                    directory=str(directory.relative_to(output)),
                    sha256=hashes,
                    compared=list(compared),
                    cases=9,
                )
            )
            print(
                f"{spec['name']}: all nine legacy/default cases match byte for byte",
                flush=True,
            )
        build.require(
            sum(r["cases"] for r in regressions) == pins["expected_regression_cases"],
            "Regression count differs",
        )
        pilot.write(output / "all-results.csv", all_rows, tuple(all_rows[0]))
        provenance = dict(
            method=METHOD,
            started_at=started,
            source_commit=commit,
            working_tree_clean=True,
            script_sha256={
                name: build.sha(build.ROOT / name) for name in build.SCRIPTS
            },
            protocol_sha256=build.sha(build.STUDY / "protocol.md"),
            freeze_sha256=build.sha(build.STUDY / "freeze-pins.json"),
            input_sha256=pins["input_sha256"],
            build_sources_sha256=build.sha(output / "build/build-sources.json"),
            generated_source_sha256=receipt["generated_source_sha256"],
            candidate_binary_sha256=receipt["candidate_binary_sha256"],
            legacy_binary_sha256=receipt["legacy_binary_sha256"],
            original_binary_sha256=receipt["original_binary_sha256"],
            native_tests=receipt["native_tests"],
            cases=len(all_rows),
            trials=trials,
            regressions=regressions,
            all_results_sha256=build.sha(output / "all-results.csv"),
        )
        (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    except (
        OSError,
        ValueError,
        KeyError,
        AssertionError,
        subprocess.CalledProcessError,
    ) as error:
        (output / "failure.txt").write_text(str(error) + "\n")
        raise
    print(
        f"Saved all {len(all_rows)} pilot and 72 regression cases: {output}", flush=True
    )
    return provenance


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not __debug__:
        parser.error("Run without Python optimization")
    try:
        run(args.output)
    except (
        OSError,
        ValueError,
        KeyError,
        AssertionError,
        subprocess.CalledProcessError,
    ) as error:
        parser.exit(1, f"Post-release handoff pilot failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
