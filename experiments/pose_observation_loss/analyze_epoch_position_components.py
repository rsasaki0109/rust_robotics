#!/usr/bin/env python3
"""Replay pinned recovery/fusion weights and decompose their position error."""

import argparse
from collections import defaultdict, deque, namedtuple
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

from analyze_commanded_recovery_arc import META, flag, identity
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import KEY, subsets
from analyze_terminal_arc_live import COHORTS, SIMULATOR
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/commanded-arc-components"
RAW_SOURCES = ROOT / "results/joint-admission-geometry/sources.json"
PROTOCOL = ROOT / "results/epoch-position-components/protocol.md"
PINNED = {
    "accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "components.csv": "78a789a7fe70e1c5327a12fa13a836fb5e25c27b9ffc78b8c154cea801696226",
    "sources.json": "52e838cc4934e9ce63be5da1d76f7d8d5f5ff1d197ce3a5b65e51fded4f4d7c3",
}
RAW_SOURCE_SHA = "ccd59c809aa90313f8f96af8857dca180ce8b3b92600b57fb14757a0edca4da4"
PARTS = ["noise", "anchor", "drift", "transport"]
GROUPS = ["all", "optimistic", "conservative", "optimistic_arc_gain"]
DT = 0.02
Sample = namedtuple(
    "Sample", "step truth noise bias anchor anchor_id anchor_sigma sigma yaw odom"
)


def transported(sample, odom, enabled):
    if not enabled:
        return (0.0, 0.0), sample.yaw
    dx, dy = odom[0] - sample.odom[0], odom[1] - sample.odom[1]
    angle = sample.yaw - sample.odom[2]
    delta = (
        dx * math.cos(angle) - dy * math.sin(angle),
        dx * math.sin(angle) + dy * math.cos(angle),
    )
    return delta, sample.yaw + odom[2] - sample.odom[2]


def model_variances(weighted, rho):
    """Actual sample spacing and weights; independent anchors share their ID."""
    assert weighted and abs(sum(w for _, w in weighted) - 1) < 1e-12
    assert len({s.step for s, _ in weighted}) == len(weighted)
    assert 0 <= rho <= 1
    unit = sum(w * w for _, w in weighted)
    variance = sum((w * s.sigma) ** 2 for s, w in weighted)
    if rho:
        for i, (a, wa) in enumerate(weighted):
            for b, wb in weighted[i + 1 :]:
                covariance = 2 * wa * wb * rho ** abs(a.step - b.step)
                unit += covariance
                variance += covariance * a.sigma * b.sigma
    anchor_weights = defaultdict(float)
    for sample, weight in weighted:
        anchor_weights[sample.anchor_id] += weight * sample.anchor_sigma
    anchor_variance = sum(w * w for w in anchor_weights.values())
    return dict(
        noise_effective_samples=1 / unit,
        weight_effective_samples=1 / sum(w * w for _, w in weighted),
        model_noise_sigma_per_axis=math.sqrt(variance),
        model_anchor_sigma_per_axis=math.sqrt(anchor_variance),
        model_noise_anchor_sigma_per_axis=math.sqrt(variance + anchor_variance),
        stochastic_anchor_count=sum(w > 0 for w in anchor_weights.values()),
    )


def decompose(groups, truth, odom, rho):
    assert groups and abs(sum(g["weight"] for g in groups) - 1) < 1e-12
    vectors = {part: [0.0, 0.0] for part in PARTS}
    position, weighted, details = [0.0, 0.0], [], []
    for index, group in enumerate(groups, 1):
        samples, weight = group["samples"], group["weight"]
        n = len(samples)
        assert n > 0
        pieces = {part: [0.0, 0.0] for part in PARTS}
        for sample in samples:
            delta, _ = transported(sample, odom, group["transport"])
            weighted.append((sample, weight / n))
            for axis in range(2):
                terms = [
                    sample.noise[axis],
                    sample.anchor[axis],
                    sample.bias[axis] - sample.anchor[axis],
                    sample.truth[axis] + delta[axis] - truth[axis],
                ]
                for name, term in zip(PARTS, terms):
                    pieces[name][axis] += term / n
                position[axis] += (
                    weight
                    / n
                    * (
                        sample.truth[axis]
                        + sample.noise[axis]
                        + sample.bias[axis]
                        + delta[axis]
                    )
                )
        for part in PARTS:
            for axis in range(2):
                vectors[part][axis] += weight * pieces[part][axis]
        details.append(
            dict(
                group_index=index,
                epoch_step=group["epoch"],
                group_weight=weight,
                sample_count=n,
                transported=group["transport"],
                first_sample_step=min(s.step for s in samples),
                last_sample_step=max(s.step for s in samples),
                anchor_ids="+".join(
                    str(i) for i in sorted({s.anchor_id for s in samples})
                ),
                **{
                    f"{p}_{a}": pieces[p][axis]
                    for p in PARTS
                    for axis, a in enumerate(["x", "y"])
                },
            )
        )
    error = [position[a] - truth[a] for a in range(2)]
    norm = math.hypot(*error)
    record = dict(
        reconstructed_x=position[0],
        reconstructed_y=position[1],
        error_x=error[0],
        error_y=error[1],
        error_norm=norm,
        used_sample_count=len(weighted),
        source_group_count=len(groups),
        sample_sigma_min=min(s.sigma for s, _ in weighted),
        sample_sigma_max=max(s.sigma for s, _ in weighted),
        **model_variances(weighted, rho),
    )
    for name, vector in vectors.items():
        record.update(
            {
                f"{name}_x": vector[0],
                f"{name}_y": vector[1],
                f"{name}_norm": math.hypot(*vector),
                f"{name}_projection": (
                    sum(a * b for a, b in zip(vector, error)) / norm
                    if norm > 1e-12
                    else 0.0
                ),
            }
        )
    for axis in range(2):
        assert abs(sum(vectors[p][axis] for p in PARTS) - error[axis]) < 1e-10
    assert abs(sum(record[p + "_projection"] for p in PARTS) - norm) < 1e-10
    record["largest_projection"] = (
        max(PARTS, key=lambda p: record[p + "_projection"]) if norm > 1e-12 else "none"
    )
    return record, details


class Replay:
    """Immutable observations with a shared odometry frame, no truth in weights."""

    def __init__(self):
        self.motion, self.recovery = deque(maxlen=25), deque(maxlen=25)
        self.groups, self.epoch, self.previous_motion = deque(maxlen=3), None, None
        self.anchor, self.anchor_id, self.anchor_sigma = (0.0, 0.0), 0, 0.0
        self.odom, self.current = (0.0, 0.0, 0.0), None
        self.speed, self.age = 0.0, 0.0

    def advance(self, before, row):
        step = round(float(row["time_s"]) / DT)
        received, precision = flag(row["correction_received"]), flag(
            row["precision_activation"]
        )
        correction = flag(row["correction"])
        self.speed, self.age = float(row["measured_speed"]), float(
            row["observation_age"]
        )
        if self.previous_motion is not None:
            speed, steering = self.previous_motion
            distance = speed * DT
            x, y, yaw = self.odom
            self.odom = (
                x + distance * math.cos(yaw),
                y + distance * math.sin(yaw),
                yaw + distance / 0.4 * math.tan(steering),
            )
        if received or precision:
            self.anchor = tuple(float(row[f"bias_{a}"]) for a in ["x", "y"])
            self.anchor_id, self.anchor_sigma = step, float(
                row["effective_position_sigma"]
            )
        sample = Sample(
            step,
            tuple(float(before[a]) for a in ["x", "y"]),
            tuple(float(row[f"noise_{a}"]) for a in ["x", "y"]),
            tuple(float(row[f"bias_{a}"]) for a in ["x", "y"]),
            self.anchor,
            self.anchor_id,
            self.anchor_sigma,
            float(row["effective_position_sigma"]),
            float(before["yaw"]) + float(row["noise_yaw"]) + float(row["bias_yaw"]),
            self.odom,
        )
        self.current = sample
        if correction or precision:
            self.motion.clear()
        self.motion.append(sample)
        missing = correction and not received
        if received or precision or missing or abs(self.speed) > 0.02 or self.age > 1.5:
            self.recovery.clear()
        pending = flag(before["safety_paused"]) or missing
        if pending and abs(self.speed) <= 0.02 and self.age <= 1.5:
            self.recovery.append(sample)
        if abs(self.speed) > 0.02 or precision:
            self.groups.clear()
        if received or precision:
            self.epoch = step
        while self.groups and step - self.groups[0]["epoch"] > 750:
            self.groups.popleft()
        if (
            abs(self.speed) <= 0.02
            and self.epoch is not None
            and 0 <= self.age <= 0.5 + 1e-9
        ):
            if not self.groups or self.groups[-1]["epoch"] != self.epoch:
                self.groups.append(dict(epoch=self.epoch, samples=deque(maxlen=25)))
            self.groups[-1]["samples"].append(sample)
        self.previous_motion = self.speed, float(row["steering"])

    def available(self):
        return (
            abs(self.speed) <= 0.02
            and self.age <= 1.5 + 1e-9
            and len(self.groups) == 3
            and self.groups[-1]["epoch"] == self.epoch
            and all(len(g["samples"]) >= 10 for g in list(self.groups)[:2])
        )

    def check_counts(self, row):
        assert not flag(row["consistency_ready"])
        assert flag(row["stationary_epoch_fusion"])
        assert len(self.motion) == int(row["motion_filter_sample_count"])
        assert len(self.recovery) == int(row["recovery_sample_count"])
        assert len(self.groups) == int(row["epoch_fusion_group_count"])
        assert self.available() == flag(row["epoch_fusion_available"])

    def view(self, kind):
        if kind == "actual":
            samples = self.recovery if self.recovery else [self.current]
            return ("recovery_mean" if self.recovery else "raw_fallback"), [
                dict(
                    samples=samples,
                    epoch=samples[0].anchor_id,
                    weight=1.0,
                    transport=False,
                )
            ]
        if kind == "preview" and self.available():
            return "three_epoch", [
                dict(g, weight=1 / 3, transport=True) for g in self.groups
            ]
        assert kind in {"preview", "motion"}
        return "motion_filter", [
            dict(
                samples=self.motion,
                epoch=self.motion[0].anchor_id,
                weight=1.0,
                transport=True,
            )
        ]

    def pose(self, kind):
        _, groups = self.view(kind)
        x = y = sine = cosine = 0.0
        for group in groups:
            for sample in group["samples"]:
                weight = group["weight"] / len(group["samples"])
                delta, yaw = transported(sample, self.odom, group["transport"])
                x += weight * (
                    sample.truth[0] + sample.noise[0] + sample.bias[0] + delta[0]
                )
                y += weight * (
                    sample.truth[1] + sample.noise[1] + sample.bias[1] + delta[1]
                )
                sine += weight * math.sin(yaw)
                cosine += weight * math.cos(yaw)
        heading = math.atan2(sine, cosine)
        if kind == "preview" and self.available():
            heading = self.pose("motion")[2]
        return x, y, heading


def summarize(records):
    groups = defaultdict(list)
    for row in records:
        for cohort in ["all", row["cohort"]]:
            for subset in subsets(row):
                for estimator in ["all", row["estimator"]]:
                    groups[
                        cohort, row["variant"], subset, row["input_kind"], estimator
                    ].append(row)
    result = []
    for key, full in sorted(groups.items()):
        for group in GROUPS:
            rows = full if group == "all" else [r for r in full if r[group]]
            result.append(
                dict(
                    **dict(
                        zip(
                            ["cohort", "variant", "subset", "input_kind", "estimator"],
                            key,
                        )
                    ),
                    group=group,
                    checkpoints=len(rows),
                    cases=len({tuple(r[k] for k in KEY) for r in rows}),
                    sample_count_min=min(
                        (r["used_sample_count"] for r in rows), default=""
                    ),
                    sample_count_max=max(
                        (r["used_sample_count"] for r in rows), default=""
                    ),
                    **{
                        f"rms_{p}": (
                            math.sqrt(
                                statistics.fmean(r[p + "_norm"] ** 2 for r in rows)
                            )
                            if rows
                            else ""
                        )
                        for p in ["error", *PARTS]
                    },
                    **{
                        f"mean_{p}_projection": (
                            statistics.fmean(r[p + "_projection"] for r in rows)
                            if rows
                            else ""
                        )
                        for p in PARTS
                    },
                    **{
                        p + "_largest": sum(r["largest_projection"] == p for r in rows)
                        for p in PARTS
                    },
                    **{
                        f"mean_{p}": (
                            statistics.fmean(r[p] for r in rows) if rows else ""
                        )
                        for p in [
                            "error_norm",
                            "noise_effective_samples",
                            "weight_effective_samples",
                            "model_noise_sigma_per_axis",
                            "model_anchor_sigma_per_axis",
                            "model_noise_anchor_sigma_per_axis",
                        ]
                    },
                )
            )
    return result


def file_digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__:
        parser.error("Run without Python -O")
    if args.output.exists():
        parser.error("Output directory must be new")
    for name, expected in PINNED.items():
        assert digest(PRIOR / name) == expected, name
    assert digest(RAW_SOURCES) == RAW_SOURCE_SHA
    previous = json.loads((PRIOR / "sources.json").read_text())
    raw = json.loads(RAW_SOURCES.read_text())
    assert digest(SIMULATOR) == previous["analysis"]["simulator_sha256"]
    assert (
        digest(ROOT / "analyze_commanded_arc_components.py")
        == previous["analysis"]["script_sha256"]
    )
    for name, expected in previous["analysis"]["helpers_sha256"].items():
        assert digest(ROOT / name) == expected, name
    accounting = read_rows(PRIOR / "accounting.csv")
    checkpoints = [
        r for r in read_rows(PRIOR / "components.csv") if r["rule"] == "commanded"
    ]
    expected = {(identity(r), r["input_kind"]): r for r in checkpoints}
    assert (
        len(accounting) == 648 and sum(flag(r["selected"]) for r in accounting) == 368
    )
    assert len(expected) == len(checkpoints) == 4240
    records, group_rows, reconstructions, seen = [], [], [], set()
    sources = dict(input_sha256=PINNED, raw_source_sha256=RAW_SOURCE_SHA, cohorts={})
    max_error = 0.0
    for cohort in COHORTS:
        pinned = raw["cohorts"][cohort]
        run = (
            args.raw_root / cohort if args.raw_root else Path(pinned["path"])
        ).resolve()
        provenance = json.loads((run / "provenance.json").read_text())
        for field in [
            "variants",
            "seeds",
            "noise_correlation_time_s",
            "simulator_sha256",
            "inputs_sha256",
            "inputs_by_variant_sha256",
        ]:
            assert provenance[field] == pinned["provenance"][field], field
        tau = provenance["noise_correlation_time_s"]
        rho = math.exp(-DT / tau) if tau else 0.0
        files = defaultdict(list)
        for case in accounting:
            if case["cohort"] == cohort and flag(case["selected"]):
                files[case["variant"], case["mode"], case["seed"]].append(case)
        hashes, intervals = {}, 0
        for (variant, mode, seed), cases in sorted(files.items()):
            name = f"{variant}-{mode}-{seed}/trajectories.csv"
            path = run / name
            hashes[name] = file_digest(path)
            assert hashes[name] == pinned["raw_sha256"][name], name
            selected = {(r["scenario"], r["planner"]) for r in cases}
            states, before_rows, statistics_by_track = {}, {}, {}
            with path.open() as stream:
                for row in csv.DictReader(stream):
                    track_key = row["scenario"], row["planner"]
                    if track_key not in selected:
                        continue
                    if float(row["time_s"]) == 0:
                        assert track_key not in states
                        states[track_key], before_rows[track_key] = Replay(), row
                        statistics_by_track[track_key] = dict(
                            intervals=0,
                            checkpoints=0,
                            fusion_available_intervals=0,
                            recovery_empty_intervals=0,
                            max_reconstruction_error=0.0,
                        )
                        continue
                    state, before = states[track_key], before_rows[track_key]
                    assert abs(float(row["noise_rho"]) - rho) < 1e-12
                    assert abs(float(row["measured_speed"]) - float(before["v"])) < 1e-9
                    state.advance(before, row)
                    state.check_counts(row)
                    intervals += 1
                    stats = statistics_by_track[track_key]
                    stats["intervals"] += 1
                    stats["fusion_available_intervals"] += flag(
                        row["epoch_fusion_available"]
                    )
                    stats["recovery_empty_intervals"] += not state.recovery
                    case_id = (cohort, variant, mode, seed, *track_key, row["time_s"])
                    if (case_id, "actual") in expected:
                        stats["checkpoints"] += 1
                        for kind in ["actual", "motion", "preview"]:
                            reconstructed = state.pose(kind)
                            prefix = {
                                "actual": "recovery",
                                "motion": "motion_filter_pose",
                                "preview": "preview_pose",
                            }[kind]
                            for axis in range(2):
                                error = abs(
                                    reconstructed[axis]
                                    - float(row[prefix + "_" + ["x", "y"][axis]])
                                )
                                assert error < 1e-8, (case_id, kind, error)
                                max_error = max(max_error, error)
                                stats["max_reconstruction_error"] = max(
                                    stats["max_reconstruction_error"], error
                                )
                            if kind in {"actual", "motion"}:
                                heading = float(
                                    row[
                                        (
                                            "recovery_yaw"
                                            if kind == "actual"
                                            else "preview_pose_yaw"
                                        )
                                    ]
                                )
                                error = abs(
                                    math.atan2(
                                        math.sin(reconstructed[2] - heading),
                                        math.cos(reconstructed[2] - heading),
                                    )
                                )
                                assert error < 1e-8, (case_id, kind, "yaw", error)
                                max_error = max(max_error, error)
                                stats["max_reconstruction_error"] = max(
                                    stats["max_reconstruction_error"], error
                                )
                        for kind in ["actual", "preview"]:
                            prior = expected[case_id, kind]
                            estimator, groups = state.view(kind)
                            decomposition, details = decompose(
                                groups,
                                tuple(float(before[a]) for a in ["x", "y"]),
                                state.odom,
                                rho,
                            )
                            assert (
                                abs(
                                    decomposition["error_norm"]
                                    - float(prior["position_error"])
                                )
                                < 1e-8
                            )
                            context = {k: prior[k] for k in META + ["input_kind"]}
                            records.append(
                                dict(
                                    context,
                                    estimator=estimator,
                                    **{
                                        k: flag(prior[k])
                                        for k in [
                                            "optimistic",
                                            "conservative",
                                            "optimistic_arc_gain",
                                            "estimated_arc_gain",
                                        ]
                                    },
                                    **decomposition,
                                )
                            )
                            group_rows.extend(
                                dict(context, estimator=estimator, **g) for g in details
                            )
                            seen.add((case_id, kind))
                    before_rows[track_key] = row
            assert len(states) == len(selected)
            assert all(float(r["time_s"]) == 180 for r in before_rows.values())
            for case in cases:
                stats = statistics_by_track[case["scenario"], case["planner"]]
                assert stats["intervals"] == 9000
                reconstructions.append(dict(**{k: case[k] for k in KEY}, **stats))
        assert intervals == pinned["selected_intervals"]
        sources["cohorts"][cohort] = dict(
            path=str(run),
            provenance=provenance,
            raw_sha256=hashes,
            reconstructed_intervals=intervals,
        )
        print(f"{cohort}: {intervals} window-state intervals reconstructed", flush=True)
    assert seen == set(expected)
    sources["analysis"] = dict(
        script_sha256=digest(Path(__file__)),
        protocol_sha256=digest(PROTOCOL),
        simulator_sha256=digest(SIMULATOR),
        helpers_sha256={
            name: digest(ROOT / name)
            for name in sorted(
                set(previous["analysis"]["helpers_sha256"])
                | {"analyze_commanded_arc_components.py"}
            )
        },
    )
    sources["audit"] = dict(
        checkpoints=len(expected) // 2,
        position_records=len(records),
        group_records=len(group_rows),
        max_reconstruction_error=max_error,
        new_physics_validation=False,
    )
    args.output.mkdir(parents=True)
    for name, rows in [
        ("accounting", accounting),
        ("reconstruction", reconstructions),
        ("positions", records),
        ("epoch-groups", group_rows),
        ("summary", summarize(records)),
    ]:
        save_rows(args.output / f"{name}.csv", rows)
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(sources["audit"], indent=2))


if __name__ == "__main__":
    main()
