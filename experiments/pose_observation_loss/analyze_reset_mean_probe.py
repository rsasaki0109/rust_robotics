#!/usr/bin/env python3
"""Replay the frozen reset probes with raw versus mean position centers."""

import argparse
from collections import defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
RESULTS = ROOT / "results"
STUDY = RESULTS / "fusion-return-reset-mean-probe"
BASE = RESULTS / "fusion-return-reset-surplus/surplus.csv"
MAP = RESULTS / "fusion-return-probe-origin/obstacle-map.csv"
ARMS = ("raw", "mean")
ORIGINS = ("initial_unsafe", "first_chord_by_exclusion", "ambiguous_zero")
PRIOR = ("x", "y", "yaw")
CURRENT = (
    "noise_x",
    "noise_y",
    "noise_yaw",
    "bias_x",
    "bias_y",
    "bias_yaw",
    "nominal_steering",
    "speed_margin",
    "speed_free_distance",
    "speed_cap",
    "uncapped_target_speed",
)
INPUT_FIELDS = (
    "edge_id",
    "raw_path",
    "prior_csv_row",
    "current_csv_row",
    *("prior_truth_" + k for k in PRIOR),
    *CURRENT,
)
META = (
    "edge_id",
    "case_id",
    "source_step",
    "source_time_s",
    "cohort",
    "noise",
    "variant",
    "mode",
    "seed",
    "scenario",
    "planner",
    "multiplicity",
)
FIELDS = (
    *META,
    "original_probe_origin",
    "original_failed_gates",
    "arm",
    "x",
    "y",
    "yaw",
    "steering",
    "margin",
    "uncapped_target_speed",
    "initial_clear",
    "first_unsafe_step",
    "wall_ids",
    "obstacle_ids",
    "free_distance",
    "speed_cap",
    "nominal_target_speed",
    "positive_distance",
    "full_horizon_clear",
)
COUNTS = (
    "physical_resets",
    "ending_multiplicity",
    "initial_blocked",
    "first_chord_blocked",
    "later_chord_blocked",
    "full_horizon_clear",
    "positive_distance",
    "nominal_positive",
)
SUMMARY_FIELDS = (
    "case_selector",
    "origin_selector",
    "arm",
    *COUNTS,
    "max_free_distance",
)
METHOD = "rounded-input, fixed-yaw/steering/margin/map raw-versus-mean full probe; no closed-loop replay"


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def read(path, fields=None):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        names = reader.fieldnames
        require(
            names and len(names) == len(set(names)),
            f"Duplicate/empty CSV header: {path}",
        )
        if fields is not None:
            require(tuple(names) == tuple(fields), f"Unexpected CSV fields: {path}")
        rows = list(reader)
        require(
            all(None not in r and None not in r.values() for r in rows),
            f"Malformed CSV: {path}",
        )
        return rows


def write(path, rows, fields):
    with Path(path).open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def number(value):
    result = float(value)
    require(math.isfinite(result), "Expected a finite number")
    return result


def tick(value):
    time = number(value)
    step = round(time * 50)
    require(
        abs(time - step / 50) <= 1e-10, "Time is not on the original 0.02-second grid"
    )
    return step


def raw_path(row):
    return (
        f"{row['cohort']}/{row['variant']}-{row['mode']}-{row['seed']}/trajectories.csv"
    )


def preflight():
    pins = json.loads((STUDY / "freeze-pins.json").read_text())
    for relative, digest in pins["input_sha256"].items():
        require(sha(REPO / relative) == digest, f"Changed frozen input: {relative}")
    rows = read(BASE)
    ids = [r["edge_id"] for r in rows]
    require(
        ids == pins["edge_ids"] and len(ids) == len(set(ids)) == 19,
        "Changed reset selection",
    )
    require(
        {raw_path(r) for r in rows} == set(pins["raw_sha256"]), "Changed raw selection"
    )
    return pins, rows


def analysis():
    return dict(
        script_sha256=sha(Path(__file__)),
        protocol_sha256=sha(STUDY / "protocol.md"),
        freeze_sha256=sha(STUDY / "freeze-pins.json"),
    )


def validate_inputs(selected, base):
    require(len(selected) == len(base), "Missing or extra selected inputs")
    require(
        [r["edge_id"] for r in selected] == [r["edge_id"] for r in base],
        "Changed selected input order/identity",
    )
    for row, old in zip(selected, base):
        require(set(row) == set(INPUT_FIELDS), "Unexpected selected input fields")
        require(row["raw_path"] == raw_path(old), "Wrong selected raw file")
        a, b = int(row["prior_csv_row"]), int(row["current_csv_row"])
        require(a >= 2 and b == a + 1, "Invalid consecutive CSV row references")
        for name in (*("prior_truth_" + k for k in PRIOR), *CURRENT):
            number(row[name])
            inherited = old.get("decision_current_input_" + name)
            if inherited is None:
                inherited = old.get("decision_" + name)
            if inherited:
                require(
                    row[name] == inherited, f"Changed logged {name}: {row['edge_id']}"
                )
        require(
            number(row["speed_free_distance"]) == number(row["speed_cap"]) == 0,
            "Original probe/cap was not zero",
        )
        require(
            0 <= number(row["uncapped_target_speed"]) <= 1,
            "Invalid original uncapped target",
        )


def extract(pins, base, raw_root=None):
    prior_sources = json.loads(
        (RESULTS / "fusion-return-first-chord/sources.json").read_text()
    )["raw"]
    selected, scanned = {}, {}
    for relative, digest in pins["raw_sha256"].items():
        cohort, short = relative.split("/", 1)
        require(
            prior_sources[cohort]["raw_sha256"][short] == digest,
            "Raw pin differs from original manifest",
        )
        directory = (
            Path(raw_root)
            if raw_root is not None
            else Path(prior_sources[cohort]["path"])
        )
        path = directory / short
        require(sha(path) == digest, f"Changed original raw file: {path}")
        wanted = {
            (r["scenario"], r["planner"], int(r["source_step"])): r
            for r in base
            if raw_path(r) == relative
        }
        require(
            len(wanted) == sum(raw_path(r) == relative for r in base),
            "Duplicate selected raw key",
        )
        previous = {}
        count = 0
        with path.open(newline="") as stream:
            reader = csv.DictReader(stream)
            require(
                reader.fieldnames
                and len(reader.fieldnames) == len(set(reader.fieldnames)),
                "Invalid raw header",
            )
            for line, current in enumerate(reader, 2):
                count += 1
                key = current["scenario"], current["planner"]
                step = tick(current["time_s"])
                old = wanted.get((*key, step))
                if old is not None:
                    require(
                        old["edge_id"] not in selected and key in previous,
                        "Duplicate/missing raw checkpoint",
                    )
                    before_line, before = previous[key]
                    require(
                        tick(before["time_s"]) == step - 1,
                        "Missing immediate predecessor",
                    )
                    row = dict(
                        edge_id=old["edge_id"],
                        raw_path=relative,
                        prior_csv_row=str(before_line),
                        current_csv_row=str(line),
                        **{"prior_truth_" + k: before[k] for k in PRIOR},
                        **{k: current[k] for k in CURRENT},
                    )
                    selected[row["edge_id"]] = row
                previous[key] = line, current
        scanned[relative] = count
    result = [selected[r["edge_id"]] for r in base]
    validate_inputs(result, base)
    return result, scanned


def load_inputs(directory, pins, base):
    path = directory / "selected-inputs.csv"
    sources = json.loads((directory / "selected-input-sources.json").read_text())
    require(
        sources["method"]
        == "selected original decision/predecessor raw strings; no regenerated trajectories",
        "Changed extraction method",
    )
    require(sources["analysis"] == analysis(), "Changed extraction code/protocol")
    require(
        sources["input_sha256"] == pins["input_sha256"], "Changed extraction inputs"
    )
    require(sources["raw_sha256"] == pins["raw_sha256"], "Changed extraction raw pins")
    require(
        sources["output_sha256"] == {"selected-inputs.csv": sha(path)},
        "Changed selected input bytes",
    )
    selected = read(path, INPUT_FIELDS)
    validate_inputs(selected, base)
    return selected


def distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    squared = dx * dx + dy * dy
    if squared < 1e-20:
        return distance(point, a)
    t = min(1.0, max(0.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / squared))
    return distance(point, (a[0] + t * dx, a[1] + t * dy))


def endpoint(x, y, yaw, steering, step):
    if step == 0:
        return x, y
    curvature = math.tan(steering) / 0.4
    length = step * 0.05
    angle = yaw + curvature * length
    if abs(curvature) < 1e-9:
        return x + length * math.cos(yaw), y + length * math.sin(yaw)
    return (
        x + (math.sin(angle) - math.sin(yaw)) / curvature,
        y - (math.cos(angle) - math.cos(yaw)) / curvature,
    )


def probe(x, y, yaw, steering, margin, obstacles):
    require(
        all(math.isfinite(v) and abs(v) <= 100 for v in (x, y, yaw)),
        "Invalid probe center/heading",
    )
    require(math.isfinite(steering) and abs(steering) <= 0.612, "Invalid steering")
    require(math.isfinite(margin) and 0 < margin < 10, "Invalid margin")
    start = x, y
    nearby = [
        (i, ox, oy)
        for i, ox, oy in obstacles
        if distance((ox, oy), start) <= 2.0 + margin
    ]
    previous = start
    for step in range(41):
        point = endpoint(x, y, yaw, steering, step)
        walls = [
            name
            for name, gap in zip(
                ("x_min", "y_min", "x_max", "y_max"),
                (point[0], point[1], 50 - point[0], 50 - point[1]),
            )
            if gap <= margin
        ]
        blockers = [
            str(i)
            for i, ox, oy in nearby
            if segment_distance((ox, oy), previous, point) <= margin
        ]
        if walls or blockers:
            return dict(
                initial_clear=step != 0,
                first_unsafe_step=step,
                wall_ids=";".join(walls),
                obstacle_ids=";".join(blockers),
                free_distance=max(step - 1, 0) * 0.05,
            )
        previous = point
    return dict(
        initial_clear=True,
        first_unsafe_step="",
        wall_ids="",
        obstacle_ids="",
        free_distance=2.0,
    )


def centers(old, selected):
    raw = {
        axis: (
            number(selected["prior_truth_" + axis]) + number(selected["noise_" + axis])
        )
        + number(selected["bias_" + axis])
        for axis in PRIOR
    }
    for axis in ("x", "y"):
        require(
            abs(raw[axis] - number(old["decision_sensed_" + axis + "_center"]))
            <= 1e-12,
            "Original sensed reconstruction disagrees",
        )
    return {
        "raw": (raw["x"], raw["y"], raw["yaw"]),
        "mean": (
            number(old["decision_mean_x_center"]),
            number(old["decision_mean_y_center"]),
            raw["yaw"],
        ),
    }


def calculate(base, selected, obstacles):
    rows = []
    for old, inputs in zip(base, selected):
        for arm, (x, y, yaw) in centers(old, inputs).items():
            steering, margin = number(inputs["nominal_steering"]), number(
                inputs["speed_margin"]
            )
            result = probe(x, y, yaw, steering, margin, obstacles[old["scenario"]])
            if arm == "raw":
                require(
                    result["free_distance"] == number(inputs["speed_free_distance"]),
                    f"Original probe differs: {old['edge_id']}",
                )
                expected = {"initial_unsafe": 0, "first_chord_by_exclusion": 1}[
                    old["decision_probe_origin"]
                ]
                require(
                    result["first_unsafe_step"] == expected,
                    "Original probe failure stage differs",
                )
            cap = min(
                1.0, max(0.0, math.sqrt(0.15**2 + result["free_distance"]) - 0.15)
            )
            rows.append(
                dict(
                    **{k: old[k] for k in META},
                    original_probe_origin=old["decision_probe_origin"],
                    original_failed_gates=old["failed_gates"],
                    arm=arm,
                    x=x,
                    y=y,
                    yaw=yaw,
                    steering=steering,
                    margin=margin,
                    uncapped_target_speed=inputs["uncapped_target_speed"],
                    **result,
                    speed_cap=cap,
                    nominal_target_speed=min(
                        number(inputs["uncapped_target_speed"]), cap
                    ),
                    positive_distance=result["free_distance"] > 0,
                    full_horizon_clear=result["first_unsafe_step"] == "",
                )
            )
    return rows


def summaries(rows):
    result = []
    for case in ("all_cases", *sorted({r["case_id"] for r in rows})):
        for origin in ("all_origins", *ORIGINS):
            for arm in ARMS:
                selected = [
                    r
                    for r in rows
                    if r["arm"] == arm
                    and case in ("all_cases", r["case_id"])
                    and origin in ("all_origins", r["original_probe_origin"])
                ]
                counts = dict(
                    physical_resets=len(selected),
                    ending_multiplicity=sum(int(r["multiplicity"]) for r in selected),
                    initial_blocked=sum(r["first_unsafe_step"] == 0 for r in selected),
                    first_chord_blocked=sum(
                        r["first_unsafe_step"] == 1 for r in selected
                    ),
                    later_chord_blocked=sum(
                        isinstance(r["first_unsafe_step"], int)
                        and r["first_unsafe_step"] > 1
                        for r in selected
                    ),
                    full_horizon_clear=sum(r["full_horizon_clear"] for r in selected),
                    positive_distance=sum(r["positive_distance"] for r in selected),
                    nominal_positive=sum(
                        r["nominal_target_speed"] > 0 for r in selected
                    ),
                )
                require(
                    sum(
                        counts[k]
                        for k in (
                            "initial_blocked",
                            "first_chord_blocked",
                            "later_chord_blocked",
                            "full_horizon_clear",
                        )
                    )
                    == len(selected),
                    "Nonexhaustive probe classifications",
                )
                result.append(
                    dict(
                        case_selector=case,
                        origin_selector=origin,
                        arm=arm,
                        **counts,
                        max_free_distance=max(
                            (r["free_distance"] for r in selected), default=""
                        ),
                    )
                )
    return result


def fresh_output(path):
    path = path.expanduser().resolve()
    require(not path.exists(), "Use a fresh output directory")
    require(
        not path.is_relative_to(REPO.resolve()),
        "Keep generated outputs outside the repository",
    )
    return path


def run(args):
    output = fresh_output(args.output)
    pins, base = preflight()
    if args.extract_raw:
        rows, scanned = extract(pins, base, args.raw_root)
        output.mkdir(parents=True)
        write(output / "selected-inputs.csv", rows, INPUT_FIELDS)
        sources = dict(
            method="selected original decision/predecessor raw strings; no regenerated trajectories",
            analysis=analysis(),
            input_sha256=pins["input_sha256"],
            raw_sha256=pins["raw_sha256"],
            raw_rows_scanned=scanned,
            output_sha256={"selected-inputs.csv": sha(output / "selected-inputs.csv")},
        )
    else:
        require(args.raw_root is None, "--raw-root requires --extract-raw")
        selected = load_inputs(args.inputs, pins, base)
        obstacles = defaultdict(list)
        for i, row in enumerate(read(MAP, ("scenario", "x", "y")), 1):
            obstacles[row["scenario"]].append((i, number(row["x"]), number(row["y"])))
        rows = calculate(base, selected, obstacles)
        summary = summaries(rows)
        output.mkdir(parents=True)
        write(output / "reprobes.csv", rows, FIELDS)
        write(output / "reprobe-summary.csv", summary, SUMMARY_FIELDS)
        sources = dict(
            method=METHOD,
            analysis=analysis(),
            input_sha256=pins["input_sha256"],
            selected_input_sha256={
                name: sha(args.inputs / name)
                for name in ("selected-inputs.csv", "selected-input-sources.json")
            },
            audit=dict(
                distinct_edges=len(base),
                ending_multiplicity=sum(int(r["multiplicity"]) for r in base),
                probe_evaluations=len(rows),
                raw_file_reads=0,
                fresh_trajectories=0,
            ),
            output_sha256={
                name: sha(output / name)
                for name in ("reprobes.csv", "reprobe-summary.csv")
            },
        )
    name = "selected-input-sources.json" if args.extract_raw else "sources.json"
    (output / name).write_text(json.dumps(sources, indent=2) + "\n")
    print(
        f"Saved {len(rows)} {'selected raw inputs' if args.extract_raw else 'rounded-input probes'}: {output}"
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--inputs",
        type=Path,
        default=STUDY,
        help="Directory containing the compact selected input packet",
    )
    parser.add_argument("--extract-raw", action="store_true")
    parser.add_argument(
        "--raw-root",
        type=Path,
        help="Relocated original fresh-white raw root; hashes must still match",
    )
    args = parser.parse_args(argv)
    if not __debug__:
        parser.error("Run without Python optimization")
    try:
        run(args)
    except (OSError, ValueError, KeyError, csv.Error) as error:
        parser.exit(1, f"Reset mean-probe audit failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
