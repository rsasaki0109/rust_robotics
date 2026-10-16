#!/usr/bin/env python3
"""Replay a fixed causal three-epoch position prototype on saved failed runs."""

import argparse
from collections import deque
import csv
import json
import math
from pathlib import Path
import statistics

from analyze_recovery_arcs import ANGLES, arc_clearance, chord_check
from analyze_recovery_stalls import digest
from analyze_robust_arc_selection import KEY, accepts, choose, nearby_obstacles
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
RULES = ["baseline", "three_epoch"]


class StationaryEpochFusion:
    """Uses measured pose/motion/events only; no map, truth or error components."""

    def __init__(self):
        self.groups = deque(maxlen=3)
        self.epoch = None
        self.previous_motion = None
        self.time = None
        self.speed = 0.0
        self.age = math.inf

    def observe(self, time, measured_pose, speed, steering, received, precision, age):
        assert self.time is None or time > self.time
        self.time, self.speed, self.age = time, speed, age
        if abs(speed) > 0.02 or precision:
            self.groups.clear()
        if received or precision:
            self.epoch = time
        while self.groups and time - self.groups[0]["epoch"] > 15 + 1e-9:
            self.groups.popleft()
        if self.previous_motion is not None:
            velocity, applied_steering = self.previous_motion
            distance = velocity * 0.02
            turn = distance / 0.4 * math.tan(applied_steering)
            for group in self.groups:
                for sample in group["samples"]:
                    sample[0] += distance * math.cos(sample[2])
                    sample[1] += distance * math.sin(sample[2])
                    sample[2] = math.atan2(
                        math.sin(sample[2] + turn), math.cos(sample[2] + turn)
                    )
        if abs(speed) <= 0.02 and self.epoch is not None and 0 <= age <= 0.5 + 1e-9:
            if not self.groups or self.groups[-1]["epoch"] != self.epoch:
                self.groups.append(dict(epoch=self.epoch, samples=deque(maxlen=25)))
            self.groups[-1]["samples"].append([*measured_pose, time])
        self.previous_motion = speed, steering

    def means(self):
        return [
            dict(
                epoch_time_s=g["epoch"],
                samples=len(g["samples"]),
                mean_x=sum(s[0] for s in g["samples"]) / len(g["samples"]),
                mean_y=sum(s[1] for s in g["samples"]) / len(g["samples"]),
                first_sample_time_s=g["samples"][0][3],
                last_sample_time_s=g["samples"][-1][3],
            )
            for g in self.groups
        ]

    def position(self):
        if abs(self.speed) > 0.02 or self.age > 1.5 + 1e-9 or len(self.groups) != 3:
            return None
        if self.groups[-1]["epoch"] != self.epoch or any(
            len(g["samples"]) < 10 for g in list(self.groups)[:-1]
        ):
            return None
        means = self.means()
        return tuple(sum(m[f"mean_{a}"] for m in means) / 3 for a in ["x", "y"])


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def select_arc(identity, rule, pose, truth, margin, obstacles, available):
    nearby = nearby_obstacles(pose, obstacles)
    values = {a: arc_clearance(pose, math.radians(a), nearby) for a in ANGLES}
    angle = choose(values)
    audits = []
    distances = {}
    for kind, current in [("estimated", pose), ("truth", truth)]:
        clearance = arc_clearance(current, math.radians(angle), obstacles)
        error, bound = chord_check(current, math.radians(angle), obstacles, clearance)
        distances[kind] = clearance
        audits.append(
            dict(
                **identity,
                rule=rule,
                pose_kind=kind,
                steering_deg=angle,
                exact_clearance=clearance,
                chord_error=error,
                sagitta_bound=bound,
            )
        )
    assert abs(distances["estimated"] - values[angle]) < 1e-9
    accepted = accepts(distances["estimated"], margin)
    true_pass = accepts(distances["truth"], margin)
    row = dict(
        **identity,
        rule=rule,
        available=available,
        steering_deg=angle,
        estimated_clearance=distances["estimated"],
        true_clearance=distances["truth"],
        target_margin=margin,
        accepted=accepted,
        accepted_true_pass=accepted and true_pass,
        mismatch=accepted and not true_pass,
        abstained=not accepted,
        position_error=math.dist(pose[:2], truth[:2]),
    )
    return row, audits


def summarize(decisions):
    result = []
    for cohort in ["white-validation", "correlated-validation"]:
        for reason in ["final", "fresh_stationary"]:
            for subset in ["all", "available", "fallback"]:
                for rule in RULES:
                    group = [
                        r
                        for r in decisions
                        if r["cohort"] == cohort
                        and r["reason"] == reason
                        and r["rule"] == rule
                        and (
                            subset == "all" or r["available"] == (subset == "available")
                        )
                    ]
                    errors = [r["position_error"] for r in group]
                    row = dict(
                        cohort=cohort,
                        reason=reason,
                        subset=subset,
                        rule=rule,
                        checkpoints=len(group),
                        mean_position_error=statistics.mean(errors) if errors else "",
                        median_position_error=(
                            statistics.median(errors) if errors else ""
                        ),
                        rms_position_error=(
                            math.sqrt(statistics.mean(e * e for e in errors))
                            if errors
                            else ""
                        ),
                    )
                    row.update(
                        {
                            k: sum(r[k] for r in group)
                            for k in [
                                "available",
                                "accepted",
                                "accepted_true_pass",
                                "mismatch",
                                "abstained",
                                "angle_changed",
                                "gained_true_pass",
                                "lost_true_pass",
                            ]
                        }
                    )
                    result.append(row)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--white", type=Path, required=True)
    parser.add_argument("--correlated", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory")
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    pinned = ROOT / "results/aligned-arcs"
    previous = ROOT / "results/robust-arc-selection"
    provenance = json.loads((pinned / "sources.json").read_text())
    records = read_rows(pinned / "checkpoints.csv")
    lookup = {(tuple(r[k] for k in KEY), r["pose_kind"]): r for r in records}
    checkpoints = [r for r in records if r["pose_kind"] == "filtered"]
    assert len(records) == 994 and len(checkpoints) == 497
    expected = {tuple(r[k] for k in KEY): r for r in checkpoints}
    prior = {
        tuple(r[k] for k in KEY): r
        for r in read_rows(previous / "decisions.csv")
        if r["rule"] == "nominal"
    }
    obstacles_path = (
        ROOT / "results/planning-clearance/fixtures/fine-padding/obstacles.csv"
    )
    maps = {}
    for r in read_rows(obstacles_path):
        maps.setdefault(r["scenario"], []).append((float(r["x"]), float(r["y"])))
    estimates, epoch_rows, decisions, audits, raw_hashes = [], [], [], [], {}
    steps = 0
    for cohort, run in [
        ("white-validation", args.white),
        ("correlated-validation", args.correlated),
    ]:
        source = provenance[cohort]
        assert json.loads((run / "provenance.json").read_text()) == source["provenance"]
        assert (
            digest(obstacles_path)
            == source["provenance"]["inputs_by_variant_sha256"]["aligned"][
                "obstacles.csv"
            ]
        )
        subset = [r for r in checkpoints if r["cohort"] == cohort]
        hashes = {}
        for mode, seed in sorted({(r["mode"], r["seed"]) for r in subset}):
            selected = {
                (r["scenario"], r["planner"])
                for r in subset
                if (r["mode"], r["seed"]) == (mode, seed)
            }
            relative = f"aligned-{mode}-{seed}/trajectories.csv"
            path = run / relative
            hashes[relative] = digest(path)
            assert hashes[relative] == source["raw_sha256"][relative]
            filters = {k: StationaryEpochFusion() for k in selected}
            previous_rows = {}
            with path.open() as stream:
                for row in csv.DictReader(stream):
                    track_key = row["scenario"], row["planner"]
                    if track_key not in filters:
                        continue
                    before = previous_rows.get(track_key)
                    previous_rows[track_key] = row
                    if before is None:
                        assert float(row["time_s"]) == 0
                        continue
                    assert (
                        abs(float(row["time_s"]) - float(before["time_s"]) - 0.02)
                        < 1e-8
                    )
                    # Log adapter reconstructs exactly the sensed pose; estimator sees no decomposition.
                    measured = tuple(
                        float(before[a])
                        + float(row[f"noise_{a}"])
                        + float(row[f"bias_{a}"])
                        for a in ["x", "y", "yaw"]
                    )
                    estimator = filters[track_key]
                    estimator.observe(
                        float(row["time_s"]),
                        measured,
                        float(before["v"]),
                        float(row["steering"]),
                        row["correction_received"] == "true",
                        row["precision_activation"] == "true",
                        float(row["observation_age"]),
                    )
                    steps += 1
                    key = cohort, mode, seed, *track_key, row["time_s"]
                    if key not in expected:
                        continue
                    record = expected[key]
                    identity = {
                        k: record[k] for k in [*KEY, "decision_time_s", "reason"]
                    }
                    assert before["time_s"] == identity["decision_time_s"]
                    baseline = tuple(
                        float(row[f"preview_pose_{a}"]) for a in ["x", "y", "yaw"]
                    )
                    truth = tuple(float(before[a]) for a in ["x", "y", "yaw"])
                    for kind, pose in [("filtered", baseline), ("truth", truth)]:
                        assert all(
                            abs(pose[i] - float(lookup[(key, kind)][a])) < 1e-12
                            for i, a in enumerate(["x", "y", "yaw"])
                        )
                    position = estimator.position()
                    available = position is not None
                    candidate = (*position, baseline[2]) if available else baseline
                    means = estimator.means()
                    estimates.append(
                        dict(
                            **identity,
                            available=available,
                            retained_epochs=len(means),
                            observation_age=row["observation_age"],
                            sigma=row["effective_position_sigma"],
                            baseline_x=baseline[0],
                            baseline_y=baseline[1],
                            candidate_x=candidate[0],
                            candidate_y=candidate[1],
                            heading=baseline[2],
                            truth_x=truth[0],
                            truth_y=truth[1],
                            baseline_error=math.dist(baseline[:2], truth[:2]),
                            candidate_error=math.dist(candidate[:2], truth[:2]),
                        )
                    )
                    epoch_rows.extend(
                        dict(**identity, available=available, **m) for m in means
                    )
                    pair = []
                    margin = float(record["recovery_current_age_margin"])
                    for rule, pose in [
                        ("baseline", baseline),
                        ("three_epoch", candidate),
                    ]:
                        decision, checked = select_arc(
                            identity,
                            rule,
                            pose,
                            truth,
                            margin,
                            maps[row["scenario"]],
                            available,
                        )
                        pair.append(decision)
                        audits.extend(checked)
                    saved = prior[key]
                    assert pair[0]["steering_deg"] == int(saved["steering_deg"])
                    assert (
                        abs(pair[0]["estimated_clearance"] - float(saved["score"]))
                        < 1e-9
                    )
                    assert (
                        abs(pair[0]["true_clearance"] - float(saved["true_clearance"]))
                        < 1e-9
                    )
                    for field in ["accepted", "accepted_true_pass", "mismatch"]:
                        assert pair[0][field] == (saved[field] == "True")
                    for decision in pair:
                        decision.update(
                            angle_changed=decision["steering_deg"]
                            != pair[0]["steering_deg"],
                            gained_true_pass=decision["accepted_true_pass"]
                            and not pair[0]["accepted_true_pass"],
                            lost_true_pass=not decision["accepted_true_pass"]
                            and pair[0]["accepted_true_pass"],
                        )
                    if not available:
                        assert (
                            pair[0]["estimated_clearance"]
                            == pair[1]["estimated_clearance"]
                        )
                        assert pair[0]["steering_deg"] == pair[1]["steering_deg"]
                    decisions.extend(pair)
            assert all(float(r["time_s"]) == 180 for r in previous_rows.values())
            print(
                f"Replayed {cohort} {mode} {seed}: {len(selected)} tracks", flush=True
            )
        raw_hashes[cohort] = hashes
    assert (
        steps == 729000
        and len(estimates) == 497
        and len(decisions) == 994
        and len(audits) == 1988
    )
    assert len({tuple(r[k] for k in KEY) for r in estimates}) == 497
    summary = summarize(decisions)
    sources = dict(
        inputs_sha256={
            str(p.relative_to(ROOT)): digest(p)
            for p in [
                pinned / "checkpoints.csv",
                pinned / "sources.json",
                previous / "decisions.csv",
                obstacles_path,
            ]
        },
        raw_sha256=raw_hashes,
        script_sha256=digest(Path(__file__)),
        helpers_sha256={
            name: digest(ROOT / name)
            for name in [
                "analyze_recovery_arcs.py",
                "analyze_recovery_stalls.py",
                "analyze_robust_arc_selection.py",
                "summarize_stopping_preview.py",
                "verify_stopping_preview.py",
            ]
        },
        protocol_sha256=digest(ROOT / "results/stationary-epoch-fusion/protocol.md"),
        filter_steps_replayed=steps,
        checkpoints=len(estimates),
        epoch_rows=len(epoch_rows),
        geometry_audits=len(audits),
        note="Causal position prototype on fixed selected failed runs; recorded dynamics unchanged, no hold or safety claim.",
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, data in [
        ("estimates.csv", estimates),
        ("epoch-means.csv", epoch_rows),
        ("decisions.csv", decisions),
        ("geometry-audit.csv", audits),
        ("summary.csv", summary),
    ]:
        save_rows(args.output / name, data)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    for r in summary:
        if r["subset"] == "all":
            print(r)


if __name__ == "__main__":
    main()
