"""Check sequential transport, non-revival, exhaustive joins and native geometry."""

import copy
import csv
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import analyze_handoff_transition_replay as audit
import analyze_mean_speed_probe as pilot
import build_release_mean_handoff as build
import native_guard_release_census as census
import native_handoff_transition_replay as native
from test_guard_release_census import query as original_query
from verify_reset_mean_probe import source_block


def query(ident, transition, lag, **changes):
    result = original_query(ident)
    result.update(
        transition_id=transition,
        lag=lag,
        mean_x=12.0,
        preview_x=10.0,
        applied_steering=0.0,
        joint_pending="false",
        correction="false",
        received="false",
        precision_activation="false",
        count=25,
    )
    result.update(changes)
    # Preserve the source's ordered operations, including drift after the buffer.
    sigma = (
        0.03
        if result["precision_active"] == "true"
        else float(result["position_sigma"])
    )
    result["speed_margin"] = (
        0.5
        + 0.1
        + 2 * math.sqrt(2) * sigma
        + float(result["drift_speed"]) * float(result["observation_age"])
    )
    return result


def packet(path, rows, fields=native.INPUT_FIELDS):
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=fields, delimiter="\t", lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)


class NativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if shutil.which("rustc") is None:
            raise RuntimeError("Activate Rust for handoff replay tests")
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.maps = cls.root / "maps.csv"
        cls.maps.write_text("scenario,x,y\nline,11.22,10\nsafe,30,30\nturn,11.2,10.8\n")
        cls.inputs = []

        def append(tid, lag, **changes):
            cls.inputs.append(query(len(cls.inputs) + 1, tid, lag, **changes))

        append(1, 0, preview_x=12)
        append(1, 1)
        append(1, 2, measured_speed=0.021, count=0)
        append(1, 3, measured_speed=0, count=25)
        append(2, 0, mean_x=10, preview_x=10)
        append(2, 1, mean_x=10, preview_x=12)
        append(
            3,
            0,
            scenario="turn",
            mean_yaw=1.1,
            measured_speed=-0.02,
            applied_steering=0.35,
        )
        append(
            3,
            1,
            scenario="turn",
            preview_yaw=-0.4,
            measured_speed=0.017,
            applied_steering=-0.2,
        )
        append(3, 2, scenario="turn", preview_yaw=-0.7, measured_speed=0)
        violations = [
            dict(joint_pending="true"),
            dict(correction="true"),
            dict(received="true"),
            dict(precision_activation="true"),
            dict(measured_speed=math.nextafter(0.02, math.inf)),
            dict(observation_age=math.nextafter(1.5, math.inf)),
            dict(count=9),
        ]
        for tid, changes in enumerate(violations, 4):
            append(tid, 0)
            append(tid, 1, **changes)
            append(tid, 2)
        append(11, 0)
        append(11, 1, measured_speed=-0.02, observation_age=1.5, count=10)
        append(12, 0, scenario="safe")  # Endpoint resume has no invented next query.
        append(13, 0, scenario="safe")
        append(13, 1, scenario="safe", nominal_target=0)
        for lag in range(51):
            append(
                14,
                lag,
                scenario="safe",
                mean_yaw=0.9,
                measured_speed=0.01,
                applied_steering=0.35 if lag % 2 else -0.2,
            )
        for name in ("one", "two"):
            work = cls.root / name
            native.compile_native(work)
            packet(work / "queries.tsv", cls.inputs)
            native.evaluate(work, work / "queries.tsv", cls.maps)
        cls.trace = pilot.read(cls.root / "one/trace.csv", native.TRACE_FIELDS)
        cls.groups = {
            (r["transition_id"], r["lag"], r["variant"]): r for r in cls.trace
        }
        work = cls.root / "original"
        census.compile_native(work)
        packet(
            work / "queries.tsv",
            [{k: r[k] for k in census.INPUT_FIELDS} for r in cls.inputs],
            census.INPUT_FIELDS,
        )
        census.evaluate(work, work / "queries.tsv", cls.maps)
        cls.original = pilot.read(work / "trace.csv", census.TRACE_FIELDS)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_original_functions_and_every_repeat_artifact(self):
        for marker in (
            "fn stopping_preview(",
            "fn terminal_arc_clearance(",
            "fn aligned_release_preview(",
            "impl MotionPoseFilter",
            "fn point_segment_distance(",
        ):
            self.assertIn(source_block(build.BASE.read_text(), marker), native.source())
        for name in (
            "handoff-transition.rs",
            "native-handoff-transition",
            "queries.tsv",
            "trace.csv",
            "compiler.log",
            "native.log",
        ):
            self.assertEqual(
                (self.root / "one" / name).read_bytes(),
                (self.root / "two" / name).read_bytes(),
            )

    def test_every_recorded_geometry_matches_independent_original_census(self):
        recorded = [r for r in self.trace if r["variant"] == "recorded"]
        self.assertEqual(len(recorded), len(self.original))
        for fresh, old in zip(recorded, self.original):
            for key in census.TRACE_FIELDS:
                self.assertEqual(fresh[key], old[key])

    def test_sequential_transport_uses_each_predecessor_speed_and_applied_steering(
        self,
    ):
        carry = (12.0, 10.0, 1.1)
        for lag, index in ((1, 6), (2, 7)):
            carry = audit.transported(
                carry, {k: str(v) for k, v in self.inputs[index].items()}
            )
            row = self.groups["3", str(lag), "carried_diagnostic_xy"]
            for key, value in zip(("carry_x", "carry_y", "carry_yaw"), carry):
                self.assertAlmostEqual(float(row[key]), value, places=13)
            self.assertEqual(
                float(row["yaw"]), float(self.inputs[index + 1]["preview_yaw"])
            )
        self.assertNotEqual(
            float(self.groups["3", "1", "carried_diagnostic_xy"]["carry_yaw"]), 1.1
        )

    def test_invalidity_never_revives_and_gated_geometry_falls_back(self):
        for tid, lag in [
            (1, 2),
            (1, 3),
            *((tid, lag) for tid in range(4, 11) for lag in (1, 2)),
        ]:
            a = self.groups[str(tid), str(lag), "recorded"]
            b = self.groups[str(tid), str(lag), "gated_carried_xy"]
            self.assertEqual(b["valid_prefix"], "false")
            for key in ("x", "y", *census.TRACE_FIELDS[1:]):
                self.assertEqual(a[key], b[key])
        self.assertEqual(self.groups["1", "3", "gated_carried_xy"]["valid_now"], "true")
        self.assertEqual(
            self.groups["11", "1", "gated_carried_xy"]["valid_prefix"], "true"
        )

    def test_anchor_is_not_evicted_at_the_filter_window_limit(self):
        rows = [q for q in self.inputs if q["transition_id"] == 14]
        carry = (12.0, 10.0, 0.9)
        for previous in rows[:-1]:
            carry = audit.transported(carry, {k: str(v) for k, v in previous.items()})
        last = self.groups["14", "50", "carried_diagnostic_xy"]
        for key, value in zip(("carry_x", "carry_y", "carry_yaw"), carry):
            self.assertAlmostEqual(float(last[key]), value, places=12)
        self.assertGreater(abs(float(last["carry_x"]) - 12), 0.001)

    def test_both_flip_directions_and_zero_targets_are_retained(self):
        for prefix, variant in audit.ALTERNATIVES.items():
            first = audit.previous.flips(
                self.groups["1", "1", "recorded"], self.groups["1", "1", variant]
            )
            second = audit.previous.flips(
                self.groups["2", "1", "recorded"], self.groups["2", "1", variant]
            )
            self.assertTrue(first["entry_avoided"] and first["release_gain"], prefix)
            self.assertTrue(second["entry_added"] and second["release_loss"], prefix)
        self.assertEqual(
            self.groups["13", "1", "gated_carried_xy"]["target"],
            "0.00000000000000000e0",
        )
        self.assertNotIn(("12", "1", "recorded"), self.groups)

    def test_all_variants_preserve_current_inputs(self):
        for row in self.trace:
            q = self.inputs[int(row["id"]) - 1]
            for key, old in (
                ("yaw", "preview_yaw"),
                ("speed", "measured_speed"),
                ("target", "nominal_target"),
                ("steering", "nominal_steering"),
            ):
                self.assertEqual(float(row[key]), float(q[old]))

    def test_bad_sequence_nonfinite_schema_and_duplicate_anchor_fail(self):
        changes = [
            (1, "lag", 2),
            (1, "id", 99),
            (1, "transition_id", 2),
            (1, "preview_x", "nan"),
            (1, "count", -1),
            (1, "correction", "False"),
            (1, "scenario", "safe"),
            (1, "lag", 0),
        ]
        for i, (index, key, value) in enumerate(changes):
            rows = copy.deepcopy(self.inputs[:4])
            rows[index][key] = value
            p = self.root / f"bad-{i}.tsv"
            packet(p, rows)
            result = subprocess.run(
                [
                    str(self.root / "one/native-handoff-transition"),
                    str(self.maps),
                    str(p),
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            self.assertNotEqual(result.returncode, 0, (key, value))


def synthetic_case(arm, mode, decisions=6):
    case = dict(
        tau="0",
        arm=arm,
        mode=mode,
        seed="17",
        scenario="line",
        planner="A",
        status="held" if arm == "one_step_handoff" else "timeout",
        decisions=str(decisions),
        resume_count="1" if mode == "random" else "0",
    )
    case["case_id"] = audit.live.case_id(case)
    header = (
        re.search(
            r'"(scenario,planner,time_s,x,y,yaw,v,[^"]+)"', build.BASE.read_text()
        )
        .group(1)
        .split(",")
    )
    track = [dict.fromkeys(header, "0")]
    track[0].update(scenario="line", planner="A", guard_active="false", time_s="0")
    sides, guards, hands = [], [], []
    for step in range(1, decisions + 1):
        q = query(step, 1, step - 1, position_sigma=0.1, drift_speed=0.02)
        recorded = 12 if step == 1 or (arm == "one_step_handoff" and step == 2) else 10
        side = dict.fromkeys(build.legacy.legacy.SIDECAR, "0")
        side.update(
            scenario="line",
            planner="A",
            step=str(step),
            time_s=str(step * 0.02),
            count="25",
            observation_age="0",
            measured_speed="0",
            correction="false",
            received="false",
            precision_activation="false",
            mean_x="12",
            mean_y="10",
            mean_yaw="0",
            steering="0",
            margin=str(q["speed_margin"]),
            nominal_target="0.4",
            applied_steering="0",
        )
        if step == 4:
            side.update(measured_speed=".021", count="0")
        if step == 6:
            side["nominal_target"] = "0"
        guard = dict.fromkeys(build.legacy.GUARD_FIELDS, "0")
        guard.update(
            scenario="line",
            planner="A",
            step=str(step),
            time_s=side["time_s"],
            used="true" if step == 1 else "false",
            joint_pending="true" if step == 1 else "false",
            selected_x=str(recorded),
            selected_y="10",
            selected_yaw="0",
            selected_v=side["measured_speed"],
        )
        hand = dict.fromkeys(build.HANDOFF_FIELDS, "0")
        hand.update(
            scenario="line",
            planner="A",
            step=str(step),
            time_s=side["time_s"],
            used=(
                "true"
                if arm == "one_step_handoff" and step == 2 and mode == "random"
                else "false"
            ),
        )
        row = dict.fromkeys(header, "0")
        row.update(
            scenario="line",
            planner="A",
            time_s=side["time_s"],
            resumed="true" if step == 1 and mode == "random" else "false",
            guard_active="false",
            safety_paused="false",
            precision_active="false",
            guard_triggered="false",
        )
        track.append(row)
        sides.append(side)
        guards.append(guard)
        hands.append(hand)
    return case, track, sides, guards, hands


class ExhaustivePipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.maps = cls.root / "maps.csv"
        cls.maps.write_text("scenario,x,y\nline,11.22,10\n")
        cls.data = [
            synthetic_case(arm, mode)
            for mode in ("random", "none")
            for arm in ("mean_preview", "one_step_handoff")
        ]
        cls.catalogue = [d[0] for d in cls.data]
        cls.windows = []
        for data in cls.data:
            cls.windows.extend(audit.select_windows(*data, 50))
        for i, w in enumerate(cls.windows, 1):
            w["transition_id"] = i
        work = cls.root / "original"
        census.compile_native(work)
        inputs = []
        for window in cls.windows:
            for item in window["points"]:
                item["baseline_id"] = len(inputs) + 1
                inputs.append(
                    audit.previous.census_input(
                        dict(item, global_id=len(inputs) + 1), "line"
                    )
                )
        packet(work / "queries.tsv", inputs, census.INPUT_FIELDS)
        census.evaluate(work, work / "queries.tsv", cls.maps)
        geometries = pilot.read(work / "trace.csv", census.TRACE_FIELDS)
        cls.baselines = {}
        for window in cls.windows:
            active = False
            for lag, item in enumerate(window["points"]):
                g = geometries[item["baseline_id"] - 1]
                row, side, guard, hand = (
                    item[k] for k in ("row", "side", "guard", "handoff")
                )
                ready = not active and not pilot.truth(guard["joint_pending"])
                triggered = ready and audit.previous.threshold(g)["would_enter"]
                active = active or triggered
                row.update(
                    guard_active=str(active).lower(),
                    guard_triggered=str(triggered).lower(),
                )
                for key, column in (
                    ("stop_9", "stop_preview_clearance_lower_bound"),
                    ("terminal_9", "terminal_recovery_clearance"),
                    ("entry_9", "preview_clearance_lower_bound"),
                    ("release_9", "release_preview_clearance_lower_bound"),
                    ("preview_margin_9", "preview_margin"),
                    ("release_margin_9", "guard_release_margin"),
                ):
                    row[column] = g[key]
                baseline = dict(
                    case_id=window["case"]["case_id"],
                    scenario="line",
                    step=str(item["step"]),
                    used=guard["used"],
                    handoff_used=hand["used"],
                    x=format(float(guard["selected_x"]), ".17e"),
                    y=format(float(guard["selected_y"]), ".17e"),
                    yaw=format(float(guard["selected_yaw"]), ".17e"),
                    target=format(float(side["nominal_target"]), ".17e"),
                    steering=format(float(side["steering"]), ".17e"),
                    **{
                        k: g[k]
                        for k in (
                            "preview_margin",
                            "release_margin",
                            "stop",
                            "terminal",
                            "release",
                        )
                    },
                )
                # Native Rust writes unpadded exponents.
                for k in ("x", "y", "yaw", "target", "steering"):
                    baseline[k] = re.sub(
                        r"e([+-])0+(\d+)$",
                        lambda m: "e" + ("" if m[1] == "+" else "-") + m[2],
                        baseline[k],
                    )
                cls.baselines[baseline["case_id"], item["step"]] = baseline
        cls.receipts = []
        for name in ("one", "two"):
            cls.receipts.append(
                audit.replay(
                    copy.deepcopy(cls.windows),
                    cls.catalogue,
                    cls.baselines,
                    cls.root / name,
                    cls.maps,
                )
            )

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_complete_pipeline_retains_controls_outcomes_and_all_queries(self):
        result = self.receipts[0][0]
        self.assertEqual(
            (
                result["cases"],
                result["pairs"],
                result["resumes"],
                result["following"],
                result["native_queries"],
            ),
            (4, 2, 2, 10, 32),
        )
        self.assertEqual(result["recorded_geometry_matches"], 12)
        self.assertEqual(result["actual_handoff_matches"], 1)
        cases = pilot.read(self.root / "one/cases.csv", audit.CASE_FIELDS)
        self.assertEqual({c["status"] for c in cases}, {"held", "timeout"})
        self.assertTrue(
            all(int(c["resumes"]) == 0 for c in cases if c["mode"] == "none")
        )
        self.assertEqual(
            len(pilot.read(self.root / "one/summary.csv", audit.SUMMARY_FIELDS)), 360
        )

    def test_complete_pipeline_repeats_every_file(self):
        one, two = self.root / "one", self.root / "two"
        self.assertEqual(
            {p.relative_to(one) for p in one.rglob("*") if p.is_file()},
            {p.relative_to(two) for p in two.rglob("*") if p.is_file()},
        )
        for p in one.rglob("*"):
            if p.is_file():
                self.assertEqual(
                    p.read_bytes(), (two / p.relative_to(one)).read_bytes()
                )

    def test_first_invalid_lag_and_fixed_horizons(self):
        events = pilot.read(self.root / "one/transitions.csv", audit.EVENT_FIELDS)
        self.assertTrue(
            all(
                e["first_invalid_lag"] == "3"
                and e["first_invalid_reasons"] == "moving|insufficient_samples"
                for e in events
            )
        )
        self.assertTrue(
            all(
                e["valid_through_2"] == "true"
                and e["valid_through_5"] == "false"
                and e["observed_5"] == "true"
                and e["observed_25"] == "false"
                for e in events
            )
        )
        rows = pilot.read(self.root / "one/followups.csv", audit.FOLLOW_FIELDS)
        self.assertTrue(
            all(
                r["valid_now"] == "true" and r["valid_prefix"] == "false"
                for r in rows
                if r["lag"] == "4"
            )
        )

    def test_corrupted_baseline_trace_and_extra_geometry_are_rejected(self):
        base = copy.deepcopy(self.baselines)
        key = next(iter(base))
        base[key]["stop"] = "99"
        with self.assertRaisesRegex(ValueError, "Recorded native geometry differs"):
            audit.replay(
                copy.deepcopy(self.windows),
                self.catalogue,
                base,
                self.root / "corrupt",
                self.maps,
            )
        trace = pilot.read(self.root / "one/native/trace.csv", native.TRACE_FIELDS)
        window = copy.deepcopy(self.windows[0])
        audit.write_packet(self.root, [window])
        trace[0]["lag"] = "2"
        with self.assertRaisesRegex(ValueError, "sequence/join"):
            audit.consume_window(window, iter(trace), self.baselines)
        original_evaluate = native.evaluate

        def extra_trace(work, packet, maps):
            result = original_evaluate(work, packet, maps)
            path = work / "trace.csv"
            with path.open("a") as stream:
                stream.write(path.read_text().splitlines()[-1] + "\n")
            return result

        with mock.patch.object(native, "evaluate", side_effect=extra_trace):
            with self.assertRaisesRegex(ValueError, "Extra native trace"):
                audit.replay(
                    copy.deepcopy(self.windows),
                    self.catalogue,
                    self.baselines,
                    self.root / "extra-trace",
                    self.maps,
                )

    def test_all_grouped_cases_and_resumes_are_selected(self):
        raw = self.root / "raw"
        raw.mkdir()
        for case, track, sides, guards, hands in self.data:
            directory = raw / f"tau0/{case['arm']}-{case['mode']}-17"
            directory.mkdir(parents=True)
            for name, rows in (
                ("trajectories.csv", track),
                ("speed-probes.csv", sides),
                ("guard-preview-inputs.csv", guards),
                ("handoff-inputs.csv", hands),
            ):
                pilot.write(directory / name, rows, tuple(rows[0]))
        pins = dict(
            noise_correlation_times=[0.0],
            modes=["none", "random"],
            arms=["mean_preview", "one_step_handoff"],
            seed=17,
            expected_cases=4,
            expected_resumes=2,
            expected_decisions=24,
            horizon=50,
        )
        windows = audit.collect_windows(raw, self.catalogue, pins)
        self.assertEqual(len(windows), 2)
        self.assertEqual(sum(len(w["points"]) - 1 for w in windows), 10)
        (raw / "tau0/mean_preview-random-17/handoff-inputs.csv").write_text(
            "scenario,planner\n"
        )
        with self.assertRaisesRegex(ValueError, "grouping"):
            audit.collect_windows(raw, self.catalogue, pins)


class BoundaryAndIntegrityTests(unittest.TestCase):
    def test_freeze_and_exact_validity_boundaries(self):
        pins = audit.preflight()
        self.assertEqual(len(pins["input_sha256"]), 194)
        item = dict(
            side=dict(
                measured_speed="-.02",
                observation_age="1.5",
                count="10",
                correction="false",
                received="false",
                precision_activation="false",
            ),
            guard=dict(joint_pending="false"),
        )
        self.assertEqual(audit.invalid_reasons(item), [])
        for changes, reason in (
            (dict(measured_speed=".020000000000000004"), "moving"),
            (dict(observation_age="1.5000000000000002"), "stale"),
            (dict(count="9"), "insufficient_samples"),
        ):
            other = copy.deepcopy(item)
            other["side"].update(changes)
            self.assertEqual(audit.invalid_reasons(other), [reason])

    def test_reordered_missing_and_mixed_rows_fail(self):
        for kind in ("missing", "step", "case", "resume"):
            data = list(synthetic_case("mean_preview", "random"))
            if kind == "missing":
                data[2].pop()
            if kind == "step":
                data[3][1]["step"] = "1"
            if kind == "case":
                data[4][2]["planner"] = "B"
            if kind == "resume":
                data[0]["resume_count"] = "0"
            with self.assertRaises(ValueError):
                audit.select_windows(*data, 50)

    def test_endpoint_and_overlapping_resumes_are_not_dropped(self):
        data = list(synthetic_case("mean_preview", "random"))
        for step in (3, 6):
            data[1][step].update(resumed="true")
            data[3][step - 1]["joint_pending"] = "true"
        data[0]["resume_count"] = "3"
        windows = audit.select_windows(*data, 50)
        self.assertEqual([len(w["points"]) - 1 for w in windows], [5, 3, 0])

    def test_manifests_reject_mutation_extra_and_missing_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            p = root / "one"
            p.write_bytes(b"original")
            records = {"one": dict(bytes=p.stat().st_size, sha256=build.sha(p))}
            audit.verify_manifest(root, records)
            p.write_bytes(b"mutated!")
            with self.assertRaisesRegex(ValueError, "Changed retained"):
                audit.verify_manifest(root, records)
            p.write_bytes(b"original")
            (root / "extra").touch()
            with self.assertRaisesRegex(ValueError, "population"):
                audit.verify_manifest(root, records)
            p.unlink()
            with self.assertRaisesRegex(ValueError, "population"):
                audit.verify_manifest(root, records)

    def test_optimized_python_and_unsafe_outputs_fail(self):
        result = subprocess.run(
            [
                sys.executable,
                "-O",
                "-B",
                "-c",
                "import analyze_handoff_transition_replay as a; a.preflight()",
            ],
            cwd=build.ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b"Python-O", result.stderr)
        with self.assertRaises(ValueError):
            build.fresh(build.ROOT / "internal-test")
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                build.fresh(directory)


if __name__ == "__main__":
    unittest.main()
