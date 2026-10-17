"""Test source-domain signs, native caps and preserved original accounting."""

import copy
import csv
from decimal import InvalidOperation
from fractions import Fraction
import math
from pathlib import Path
import shutil
import tempfile
import unittest

import analyze_command_sign as a
import verify_command_sign as v


def raw(k=1, stop=False, **changes):
    cap = a.lattice()[k]["speed_cap"]
    nominal = "0.000000000" if stop else min("0.400000000", cap)
    out = dict.fromkeys(a.FLAGS, "false")
    out.update(
        speed_free_distance=format(k * 0.05, ".9f"),
        speed_cap=cap,
        uncapped_target_speed="0.000000000" if stop else "0.400000000",
        nominal_target_speed=nominal,
        target_speed="0.000000000",
        stop_latched=str(stop).lower(),
        safety_paused="true",
        guard_active="true",
        stationary_epoch_fusion="true",
        measured_speed="0.00000000000000000e+00",
    )
    out.update(changes)
    return out


def old(row, branch="joint", **changes):
    out = dict(
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
    out.update(
        event_id="1",
        source_event_id="fresh_1",
        time_s="180.0",
        offset_from_return_ms="0",
        phase="available",
        branch=branch,
    )
    out.update(
        {
            k: str(a.previous.flag(row[k], raw=True))
            for k in a.FLAGS
            if k != "speed_braking" and k != "restart_limited"
        }
    )
    out.update(
        {
            k: str(float(row[k]))
            for k in ("nominal_target_speed", "target_speed", "measured_speed")
        }
    )
    out["joint_pending"] = str(branch == "joint")
    for g in a.GATES:
        out["gate_" + g] = (
            "pass"
            if branch == "joint" or g in ("stationary", "release")
            else "not_applicable"
        )
    if branch == "joint" and float(row["nominal_target_speed"]) == 0:
        out["gate_nominal"] = "fail" if row["stop_latched"] == "true" else "unknown"
    state = out["gate_nominal"]
    result = state if branch == "joint" else "pass"
    out.update(
        certainty=result,
        failed_gates="nominal" if result == "fail" else "",
        unknown_gates="nominal" if result == "unknown" else "",
        timer_before_ticks="0",
        timer_after_ticks="1" if result == "pass" else "0",
        threshold_ticks="50" if branch == "joint" else "25",
        ready_from_timer=str(result == "pass"),
        timer_action="accumulate" if result == "pass" else "stay_zero",
    )
    out.update(changes)
    return out


def refined(k=0, stop=False, branch="joint", **changes):
    r = raw(k, stop)
    if branch == "guard":
        r["safety_paused"] = "false"
    p = a.command(r, a.lattice())
    p["joint_pending"] = branch == "joint"
    prior = old(r, branch, **changes)
    return a.refine(prior, p)


def event(returned=True, rp=1):
    row = dict.fromkeys(a.ANCHOR_FIELDS, "")
    row.update({k: str(v) for k, v in old(raw()).items() if k in a.META})
    row.update(
        event_id="1",
        source_event_id="fresh_1",
        event_kind="received",
        event_time_s="175.0",
        scheduled="True",
        received="True",
        precision_activation="False",
        horizon="full",
        preview_loss=str(returned),
        loss_operation="ttl_expiry" if returned else "no_observed_loss",
        loss_time_s="175.02" if returned else "",
        returned=str(returned) if returned else "",
        return_time_s="180.0" if returned else "",
        return_wait_s="4.98" if returned else "",
        ended="False" if returned else "",
        censored_return="True" if returned else "",
        no_return="False",
        ttl_limit_without_next_step="False",
        parent_rp_records="8",
        rp_records=str(rp),
        rp_different_preview_records=str(rp),
        anchor_category="returned_censored" if returned else "no_observed_loss",
        census_case_weight="0.5",
        received_case_weight="0.5",
        rp_case_mass=str(rp / 8),
        rp_different_preview_case_mass=str(rp / 8),
    )
    return row


class NumericDomain(unittest.TestCase):
    def test_nanounits_are_exact_and_match_decimal_reader(self):
        for value, expected in (
            ("0.000000000", 0),
            ("0.000000001", 1),
            ("0.119258240", 119258240),
            ("2.000000000", 2000000000),
        ):
            self.assertEqual(a.nano(value), expected)
            self.assertEqual(v.units(value), expected)

    def test_invalid_nonnumeric_nonfinite_negative_and_wrong_precision(self):
        for value in (
            "-0.000000000",
            "-0.100000000",
            "nan",
            "inf",
            "0.1",
            "1e-9",
            "00.000000000",
            "+0.000000000",
            "0.0000000000",
        ):
            for parser in (a.nano, v.units):
                with self.subTest(
                    parser=parser.__name__, value=value
                ), self.assertRaises((AssertionError, InvalidOperation)):
                    parser(value)

    def test_off_lattice_and_out_of_range_probes_are_rejected(self):
        table = a.lattice()
        lookup = {r["speed_free_distance"]: r for r in table}
        for value in ("0.000000001", "0.049999999", "2.050000000"):
            r = raw(speed_free_distance=value)
            for function, argument in ((a.command, table), (v.inspect, lookup)):
                with self.subTest(value=value), self.assertRaises(AssertionError):
                    function(r, argument)

    def test_all_four_structural_sign_reasons_are_timer_independent(self):
        for k, stop, reason in (
            (1, False, "positive"),
            (0, False, "cap_zero"),
            (1, True, "latch_zero"),
            (0, True, "both_zero"),
        ):
            r = raw(k, stop, ready_from_timer="nonsense", timer_action="nonsense")
            before = copy.deepcopy(r)
            p = a.command(r, a.lattice())
            q = v.inspect(r, {s["speed_free_distance"]: s for s in a.lattice()})
            self.assertEqual(p, q)
            self.assertEqual(p["source_zero_reason"], reason)
            self.assertEqual(p["source_nominal_positive"], reason == "positive")
            self.assertEqual(r, before)

    def test_uncapped_positive_floor_and_latch_are_enforced(self):
        for changes in (
            dict(uncapped_target_speed="0.099999999"),
            dict(stop_latched="true"),
            dict(uncapped_target_speed="1.000000001"),
        ):
            with self.subTest(changes=changes), self.assertRaises(AssertionError):
                a.command(raw(**changes), a.lattice())
        p = a.command(
            raw(
                uncapped_target_speed="0.100000000", nominal_target_speed="0.100000000"
            ),
            a.lattice(),
        )
        self.assertTrue(p["source_nominal_positive"])

    def test_min_of_rounded_commands_is_an_exact_nanounit_identity(self):
        # Monotone rounding commutes with min, including values on both
        # sides of a rounding midpoint and binary64 adjacent values.
        values = [
            0.0,
            0.1,
            0.4,
            0.1192582405,
            math.nextafter(0.1192582405, 0),
            math.nextafter(0.1192582405, 1),
        ]
        for x in values:
            for y in values:
                self.assertEqual(
                    a.nano(format(min(x, y), ".9f")),
                    min(a.nano(format(x, ".9f")), a.nano(format(y, ".9f"))),
                )
        with self.assertRaises(AssertionError):
            a.command(raw(nominal_target_speed="0.119258241"), a.lattice())

    def test_final_pause_and_guard_overrides(self):
        for paused, guard in (
            (True, True),
            (False, True),
            (True, False),
            (False, False),
        ):
            r = raw(safety_paused=str(paused).lower(), guard_active=str(guard).lower())
            r["target_speed"] = (
                "0.000000000" if paused or guard else r["nominal_target_speed"]
            )
            a.command(r, a.lattice())
        with self.assertRaises(AssertionError):
            a.command(raw(target_speed=raw()["nominal_target_speed"]), a.lattice())

    def test_exact_cap_braking_threshold(self):
        cap = float.fromhex("0x0.0p+0")
        for value, expected in (
            (cap + 1e-9, False),
            (math.nextafter(cap + 1e-9, 1), True),
            (math.nextafter(cap + 1e-9, 0), False),
        ):
            r = raw(
                0,
                measured_speed=format(value, ".17e"),
                speed_braking=str(expected).lower(),
            )
            self.assertEqual(a.command(r, a.lattice())["speed_braking"], expected)

    def test_flag_implications_are_required(self):
        with self.assertRaises(AssertionError):
            a.command(raw(stationary_epoch_fusion="false"), a.lattice())


class RefinedConditions(unittest.TestCase):
    def test_rounded_zero_refines_without_changing_prior_result(self):
        r = refined()
        self.assertEqual(
            (r["prior_nominal_gate"], r["source_nominal_gate"]), ("unknown", "fail")
        )
        self.assertEqual(
            (r["prior_certainty"], r["refined_certainty"]), ("unknown", "fail")
        )
        self.assertEqual(r["refined_failed_gates"], "nominal")
        self.assertFalse(r["ready_from_timer"])

    def test_simultaneous_failures_retain_all_conditions(self):
        r = refined(gate_clearance="fail", certainty="fail", failed_gates="clearance")
        self.assertEqual(r["refined_failed_gates"], "clearance;nominal")

    def test_guard_has_no_nominal_admission_gate(self):
        r = refined(branch="guard")
        self.assertFalse(r["source_nominal_positive"])
        self.assertEqual(r["source_nominal_gate"], "not_applicable")
        self.assertEqual(r["refined_certainty"], "pass")

    def test_sign_is_derived_before_inconsistent_timer_is_rejected(self):
        with self.assertRaises(AssertionError):
            refined(ready_from_timer="True")

    def test_source_disagreement_with_a_previously_certain_gate_is_rejected(self):
        with self.assertRaises(AssertionError):
            refined(gate_nominal="pass")

    def test_verifier_rejoins_raw_fields_and_reconstructs_same_result(self):
        row = raw()
        p = a.command(row, a.lattice())
        p["joint_pending"] = True
        prior = old(row)
        self.assertEqual(a.refine(prior, p), v.reconstruct(prior, p, row))
        tampered = dict(row, measured_speed="0.00100000000000000e+00")
        with self.assertRaises(AssertionError):
            v.reconstruct(prior, p, tampered)


class OriginalAccounting(unittest.TestCase):
    def test_nonreturn_fields_are_blank_and_censored_endpoint_is_one_tick(self):
        empty = a.anchor(event(False), [], 2, 2)
        self.assertTrue(all(empty[k] is None for k in (*a.REFS, *a.HAS)))
        e = event()
        got = a.anchor(e, [refined()], 2, 2)
        self.assertEqual(got, v.anchor(e, [refined()], 2, 2))
        self.assertEqual(got["decision_references"], 1)
        self.assertTrue(got["censored_return"])
        self.assertIsNone(got["end_time_s"])

    def test_missing_endpoint_or_wrong_case_cannot_join(self):
        with self.assertRaises(AssertionError):
            a.anchor(event(), [], 2, 2)
        d = refined()
        d["case_id"] = "C2"
        with self.assertRaises(AssertionError):
            a.anchor(event(), [d], 2, 2)

    def test_parent_measures_precede_selection_and_shares_keep_blanks(self):
        e1, e2 = event(rp=1), event(False, rp=7)
        e2["event_id"] = "2"
        anchors = [a.anchor(e1, [refined()], 2, 2), a.anchor(e2, [], 2, 2)]
        self.assertEqual(sum(x["census_case_weight"] for x in anchors), 1)
        self.assertEqual(
            [x["rp_case_mass"] for x in anchors], [Fraction(1, 8), Fraction(7, 8)]
        )
        case = {
            **{k: e1[k] for k in a.META},
            "selected_timeout": "True",
            "held": "False",
        }
        before = copy.deepcopy(anchors)
        rows = a.summaries([case], anchors)
        self.assertEqual(rows, list(v.summaries([case], anchors).values()))
        self.assertEqual(a.closures(rows), v.closures(v.summaries([case], anchors)))
        populated = next(
            r
            for r in rows
            if tuple(r[k] for k in a.GROUP)
            == ("fresh", "white", "epoch", "random", "all")
        )
        self.assertEqual(populated["has_prior_unknown_fraction"], 1)
        self.assertEqual(populated["has_refined_unknown_fraction"], 0)
        self.assertEqual(populated["has_nominal_zero_case_mass"], Fraction(1, 2))
        self.assertTrue(any(r["has_prior_unknown_fraction"] is None for r in rows))
        self.assertEqual(before, anchors)
        broken = copy.deepcopy(rows)
        populated_index = rows.index(populated)
        broken[populated_index]["available_joint_cap_zero_references"] += 1
        with self.assertRaises(AssertionError):
            a.closures(broken)

    def test_raw_grid_gaps_and_duplicate_steps_are_rejected(self):
        for times in (("0.000000", "0.040000"), ("0.000000", "0.000000")):
            with self.subTest(times=times), tempfile.TemporaryDirectory() as tmp:
                base = Path(tmp)
                path = base / "epoch-random-277/trajectories.csv"
                path.parent.mkdir()
                r = dict(raw(), scenario="open_50x50", planner="JPS", time_s=times[0])
                with path.open("w", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=r, lineterminator="\n")
                    writer.writeheader()
                    for t in times:
                        writer.writerow(dict(r, time_s=t))
                c = {
                    **{k: old(raw())[k] for k in a.META},
                    "selected_timeout": "True",
                    "time_s": "180.0",
                }
                source = {
                    "fresh-white": {
                        "path": str(base),
                        "raw_sha256": {
                            "epoch-random-277/trajectories.csv": a.sha(path)
                        },
                    }
                }
                with self.assertRaises(AssertionError):
                    a.collect([c], [], source, {}, a.lattice())


class SourceAndNative(unittest.TestCase):
    def test_source_contract_detects_domain_or_order_changes(self):
        source = (
            a.REPO / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
        ).read_text()
        self.assertEqual(a.source_contract(source), v.contract(source))
        for changed in (
            source.replace("for step in 1..=40", "for step in 1..=41"),
            source.replace("(step - 1) as f64 * 0.05", "(step - 1) as f64 * 0.04"),
            source.replace(
                ".clamp(0.1, if restart_limited", ".clamp(0.0, if restart_limited"
            ),
            source.replace(
                "assert!(!stationary_epoch_fusion || aligned_terminal_release);", ""
            ),
        ):
            for checker in (a.source_contract, v.contract):
                with self.assertRaises(AssertionError):
                    checker(changed)

    def test_native_caps_match_all_collector_bits_and_formatting(self):
        rustc = (
            shutil.which("rustc")
            or "/workspace/.cloud-env/rust_robotics/rustup/toolchains/1.99.0-x86_64-unknown-linux-gnu/bin/rustc"
        )
        if not Path(rustc).exists():
            self.skipTest("rustc unavailable")
        with tempfile.TemporaryDirectory() as tmp:
            table, proof = v.native(Path(tmp) / "native", Path(rustc))
            self.assertEqual(table, a.lattice())
            self.assertEqual(proof["native_tests"], 4)
            self.assertEqual(table[0]["speed_cap_bits"], "0000000000000000")
            self.assertGreater(float(table[1]["speed_cap"]), 0.1)
            self.assertEqual(table[-1]["speed_cap"], "1.000000000")


if __name__ == "__main__":
    unittest.main()
