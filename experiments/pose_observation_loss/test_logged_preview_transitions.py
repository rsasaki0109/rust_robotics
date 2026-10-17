"""Boundary, provenance, mode and fixed-parent tests for logged transitions."""

import copy
import csv
from fractions import Fraction
import hashlib
from pathlib import Path
import tempfile
import unittest

import analyze_logged_preview_transitions as a


def point(preview="three_epoch", actual="recovery_mean"):
    return a.logged(
        dict(
            recovery_sample_count="1" if actual == "recovery_mean" else "0",
            motion_filter_sample_count="1",
            epoch_fusion_group_count="3",
            epoch_fusion_available=str(preview == "three_epoch").lower(),
            measured_speed="0.0",
            observation_age="0.0",
            correction="true",
            correction_received="true",
            precision_activation="false",
        )
    )


def event(time=150, parent=4, states=()):
    e = dict(
        zip(
            a.META,
            (
                "C1",
                "F1",
                "fresh-white",
                "epoch",
                "random",
                "277",
                "open_50x50",
                "JPS",
                "fresh",
                "fresh",
                "white",
            ),
        )
    )
    counts = {s: sum(r["stratum"] == s for r in states) for s in a.STRATA}
    different = counts["filter_single"]
    e.update(
        event_id="1",
        source_event_id="fresh_1",
        event_kind="received",
        event_time_s=str(time),
        actual_estimator="recovery_mean",
        preview_estimator="three_epoch",
        scheduled="True",
        received="True",
        precision_activation="False",
        fusion_available="True",
        recovery_sample_count="1",
        motion_filter_sample_count="1",
        epoch_fusion_group_count="3",
        observation_age="0",
        parent_rp_records=str(parent),
        census_case_weight="0.25",
        received_case_weight="0.5",
        rp_records=str(len(states)),
        rp_case_mass=str(len(states) / parent if parent else 0),
        rp_different_preview_records=str(different),
        rp_same_preview_records=str(len(states) - different),
        **{"rp_" + s + "_records": str(n) for s, n in counts.items()}
    )
    return e


def source(offset, preview="motion_filter", sid="1"):
    return dict(
        case_id="C1",
        event_id="1",
        source_id=sid,
        time_s=str(150 + offset / 1000),
        stratum="filter_single" if preview == "motion_filter" else "fusion_three",
        a_estimator="recovery_mean",
        b_estimator=preview,
        **{k: event()[k] for k in a.KEY}
    )


def trace(points, states=(), time=150):
    return a.trace(event(time, states=states), points, list(states), 4, 2)


class LoggedModes(unittest.TestCase):
    def test_history_groups_do_not_select_preview(self):
        p = point("motion_filter")
        self.assertEqual(p["epoch_fusion_group_count"], 3)
        self.assertEqual(p["preview"], "motion_filter")

    def test_no_recovery_samples_select_raw(self):
        self.assertEqual(point(actual="raw_fallback")["actual"], "raw_fallback")

    def test_fusion_requires_three_retained_groups(self):
        row = dict(
            zip(
                a.RAW_FIELDS, ("1", "1", "2", "true", "0", "0", "true", "true", "false")
            )
        )
        with self.assertRaises(AssertionError):
            a.logged(row)

    def test_malformed_flags_counts_and_floats(self):
        row = dict(
            zip(
                a.RAW_FIELDS,
                ("1", "1", "3", "false", "0", "0", "true", "true", "false"),
            )
        )
        for key, value in (
            ("epoch_fusion_available", "False"),
            ("correction_received", "1"),
            ("recovery_sample_count", "-1"),
            ("motion_filter_sample_count", "01"),
            ("epoch_fusion_group_count", "4"),
            ("measured_speed", "nan"),
            ("observation_age", "-1"),
        ):
            with self.subTest(key=key, value=value), self.assertRaises(AssertionError):
                a.logged({**row, key: value})


class TraceBoundaries(unittest.TestCase):
    def setUp(self):
        self.points = {n: point() for n in range(7499, 7576)}

    def test_exact_grid_and_microsecond_tolerance(self):
        self.assertEqual(a.step("150.00000001"), 7500)
        for value in ("150.001", "nan", "-0.02", "180.02"):
            with self.assertRaises(AssertionError):
                a.step(value)

    def test_no_forward_and_partial_horizon(self):
        self.assertEqual(a.window(180), (8999, 9000, "no_forward"))
        self.assertEqual(a.window(179), (8949, 9000, "clipped"))
        self.assertEqual(a.window(178.5), (8924, 9000, "full"))

    def test_pre_event_change_is_not_post_event(self):
        self.points[7499] = point("motion_filter", "raw_fallback")
        out, rr, _ = trace(self.points)
        self.assertEqual(len(rr), 2)
        self.assertEqual((out["preview_changes"], out["actual_changes"]), (0, 0))
        self.assertIsNone(out["first_preview_change_offset_ms"])

    def test_first_and_last_forward_checkpoints(self):
        for n, expected in ((7501, 20), (7575, 1500)):
            pp = copy.deepcopy(self.points)
            for j in range(n, 7576):
                pp[j] = point("motion_filter")
            out, _, _ = trace(pp)
            self.assertEqual(out["first_preview_change_offset_ms"], expected)
            self.assertEqual(out["preview_changes"], 1)

    def test_maximal_runs_reconstruct_both_modes(self):
        self.points[7501] = point("motion_filter", "raw_fallback")
        out, rr, _ = trace(self.points)
        self.assertEqual(out["preview_changes"], 2)
        self.assertEqual(out["actual_changes"], 2)
        self.assertEqual(sum(r["checkpoint_count"] for r in rr), 77)
        self.assertEqual([r["checkpoint_count"] for r in rr], [2, 1, 74])
        self.assertEqual([r["start_offset_ms"] for r in rr], [-20, 20, 40])
        for left, right in zip(rr, rr[1:]):
            self.assertEqual(right["start_offset_ms"] - left["end_offset_ms"], 20)
            self.assertNotEqual(
                (left["actual_estimator"], left["preview_estimator"]),
                (right["actual_estimator"], right["preview_estimator"]),
            )

    def test_missing_checkpoint_rejected(self):
        del self.points[7502]
        with self.assertRaises(AssertionError):
            trace(self.points)

    def test_final_event_has_two_checkpoints_no_post_change(self):
        pp = {8999: point("motion_filter"), 9000: point()}
        out, rr, _ = trace(pp, time=180)
        self.assertEqual(
            (out["horizon"], out["checkpoint_count"], out["preview_changes"]),
            ("no_forward", 2, 0),
        )
        self.assertEqual(len(rr), 2)

    def test_rp_same_time_modes_and_original_parent(self):
        for n in range(7505, 7576):
            self.points[n] = point("motion_filter")
        states = [source(120), source(20, "three_epoch", "2")]
        out, _, aa = trace(self.points, states)
        self.assertEqual(out["first_preview_change_offset_ms"], 100)
        self.assertEqual(out["first_after_different_rp_offset_ms"], 120)
        self.assertEqual(out["first_after_different_rp_lag_ms"], 20)
        self.assertEqual(out["rp_case_mass"], Fraction(1, 2))
        self.assertEqual([r["rp_case_weight"] for r in aa], [Fraction(1, 4)] * 2)
        self.assertEqual([r["interval"] for r in aa], [1, 2])

    def test_pre_event_difference_needs_no_post_event_change(self):
        self.points[7499] = point("motion_filter")
        out, _, aa = trace(self.points, [source(-20)])
        self.assertEqual(out["rp_different_before_records"], 1)
        self.assertEqual(out["rp_different_after_records"], 0)
        self.assertIsNone(out["first_after_different_rp_lag_ms"])
        self.assertEqual(aa[0]["temporal_relation"], "before")

    def test_wrong_case_event_stratum_or_actual_rejected(self):
        s = source(0, "three_epoch")
        for key, value in (
            ("case_id", "C2"),
            ("event_id", "2"),
            ("mode", "none"),
            ("stratum", "filter_single"),
            ("a_estimator", "raw_fallback"),
            ("b_estimator", "motion_filter"),
            ("time_s", "149.96"),
        ):
            with self.subTest(key=key), self.assertRaises(AssertionError):
                trace(self.points, [{**s, key: value}])

    def test_immediate_saved_count_mismatch_rejected(self):
        e = event()
        e["motion_filter_sample_count"] = "2"
        with self.assertRaises(AssertionError):
            a.trace(e, self.points, [], 4, 2)

    def test_input_nonmutation(self):
        frozen = copy.deepcopy(self.points)
        trace(self.points)
        self.assertEqual(self.points, frozen)


class ParentMeasures(unittest.TestCase):
    def test_sum_before_conditional_share(self):
        pp = {n: point() for n in range(7499, 7576)}
        stable, _, _ = trace(pp)
        for n in range(7501, 7576):
            pp[n] = point("motion_filter")
        changed, _, _ = trace(pp)
        changed["census_case_weight"] = Fraction(1, 2)
        stable["census_case_weight"] = Fraction(1, 10)
        rr = a.summarize([], [changed, stable], {})
        self.assertEqual(rr["fusion_loss_event_fraction"], Fraction(1, 2))
        self.assertEqual(rr["fusion_loss_case_mass_fraction"], Fraction(5, 6))

    def test_empty_denominator_distinct_from_observed_zero(self):
        empty = a.summarize([], [], {})
        stable, _, _ = trace({n: point() for n in range(7499, 7576)})
        observed = a.summarize([], [stable], {})
        self.assertIsNone(empty["fusion_loss_event_fraction"])
        self.assertEqual(observed["fusion_loss_event_fraction"], 0)
        self.assertEqual(empty["rp_case_mass"], 0)

    def test_all_empty_planned_summary_cells_and_closure(self):
        rows = a.aggregate([], [])
        self.assertEqual(len(rows), 1344)
        self.assertGreater(a.closure(rows), 0)
        row = next(
            r
            for r in rows
            if r["mode"] == "all_modes"
            and r["event_group"] == "all"
            and r["horizon"] == "all"
        )
        row["rp_case_mass"] = Fraction(1, 2)
        with self.assertRaises(AssertionError):
            a.closure(rows)

    def test_duplicate_full_source_identity_rejected(self):
        with self.assertRaises(AssertionError):
            a.indexed([source(0), source(20)], ("case_id", "source_id"))


class RawInputChecks(unittest.TestCase):
    def check_fixture(self, times, expected_failure=False, bad_hash=False):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            name = "epoch-random-277/trajectories.csv"
            path = root / name
            path.parent.mkdir()
            with path.open("w", newline="") as f:
                w = csv.DictWriter(
                    f, fieldnames=("scenario", "planner", "time_s", *a.RAW_FIELDS)
                )
                w.writeheader()
                for t in times:
                    w.writerow(
                        dict(
                            scenario="open_50x50",
                            planner="JPS",
                            time_s=t,
                            **dict(
                                zip(
                                    a.RAW_FIELDS,
                                    (
                                        "1",
                                        "1",
                                        "3",
                                        "false",
                                        "0",
                                        "0",
                                        "false",
                                        "false",
                                        "false",
                                    ),
                                )
                            )
                        )
                    )
            h = hashlib.sha256(path.read_bytes()).hexdigest()
            spec = {
                "fresh-white": dict(
                    path=d, raw_sha256={name: "0" * 64 if bad_hash else h}
                )
            }
            case = {**event(), "selected_timeout": "False", "time_s": "0.04"}
            if expected_failure:
                with self.assertRaises(AssertionError):
                    a.read_raw(spec, [case], [], {})
            else:
                histories, counts = a.read_raw(spec, [case], [], {})
                self.assertEqual(histories, {})
                self.assertEqual(counts["raw_files"], 1)

    def test_contiguous_original_grid(self):
        self.check_fixture((0, 0.02, 0.04))

    def test_duplicate_missing_or_unordered_grid(self):
        for times in ((0, 0.02, 0.02, 0.04), (0, 0.04), (0, 0.04, 0.02)):
            with self.subTest(times=times):
                self.check_fixture(times, True)

    def test_raw_hash_is_mandatory(self):
        self.check_fixture((0, 0.02, 0.04), True, True)


if __name__ == "__main__":
    unittest.main()
