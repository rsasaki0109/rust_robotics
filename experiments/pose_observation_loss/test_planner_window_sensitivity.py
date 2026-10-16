"""Whole-window identity and inherited unit weights across planners and modes."""

from copy import deepcopy
from fractions import Fraction as F
import math
import unittest

import analyze_planner_window_sensitivity as analysis


def row(
    planner="A*",
    scenario="fixture",
    seed="277",
    time=150.0,
    error=0.2,
    tick=1,
    mode="filter_single",
):
    terms = analysis.previous.historical.vector_terms(
        dict(noise=(0, 0), anchor=(error, 0), drift=(0, 0), transport=(0, 0)), 0.01
    )
    r = dict(
        cohort="fresh-white",
        partition="fresh",
        noise="white",
        variant="epoch",
        mode="none",
        seed=seed,
        scenario=scenario,
        planner=planner,
        source_id="1",
        state_id="1",
        source_event_ids="1",
        available=True,
        view=analysis.previous.VIEW,
        stratum=mode,
        a_estimator="recovery_mean",
        b_estimator="motion_filter" if mode == "filter_single" else "three_epoch",
        anchor_mode="shared_single" if mode == "filter_single" else "preview_three",
        midpoint=terms,
        residual=terms,
        **{s + "_tick": tick for s in analysis.previous.SCORES}
    )
    p = dict(
        time_s=time,
        a_weight=0.5,
        b_weight=0.5,
        **{
            t + suffix: v
            for t in ["midpoint", "residual"]
            for suffix, v in [("_x", error), ("_y", 0), ("_variance", 0.01)]
        }
    )
    event = dict(
        r,
        event_kind="received",
        event_time_s=150.0,
        truth_time_s=149.98,
        scheduled="True",
        received="True",
        precision_activation="False",
    )
    r["_window"] = analysis.payload(r, p, time - 0.02, {"1": event})
    return r


def account(rr):
    return list(
        {
            analysis.previous.identity(r): {
                k: r[k] for k in [*analysis.KEY, "partition", "noise"]
            }
            for r in rr
        }.values()
    )


def units(rr, accounting=None):
    return analysis.audit_units(account(rr) if accounting is None else accounting, rr)


class PlannerWindowTests(unittest.TestCase):
    def test_signature_is_exact_order_stable_and_normalizes_signed_zero(self):
        a, b = row()["_window"], row(time=150.02)["_window"]
        original = analysis.window_bytes([a, b])
        self.assertEqual(original, analysis.window_bytes([b, a]))
        changed = deepcopy(a)
        changed["targets"]["midpoint"]["observed"][1] = -0.0
        self.assertEqual(analysis.window_bytes([a]), analysis.window_bytes([changed]))
        changed["targets"]["midpoint"]["observed"][0] = math.nextafter(0.2, math.inf)
        self.assertNotEqual(
            analysis.window_bytes([a]), analysis.window_bytes([changed])
        )

    def test_signature_keeps_ticks_labels_times_variance_and_signed_parts(self):
        base = row()["_window"]
        variants = []
        p = deepcopy(base)
        p["scores"]["contrast"] += 1
        variants.append(p)
        p = deepcopy(base)
        p["targets"]["residual"]["outside"]["modeled"] = False
        variants.append(p)
        p = deepcopy(base)
        p["truth_time_s"] -= 0.02
        variants.append(p)
        p = deepcopy(base)
        p["targets"]["midpoint"]["variance"] *= 2
        variants.append(p)
        p = deepcopy(base)
        p["targets"]["residual"]["parts"]["anchor"] = (-0.2, 0)
        variants.append(p)
        p = deepcopy(base)
        p["events"][0] = ("missed", *p["events"][0][1:])
        variants.append(p)
        self.assertTrue(
            all(
                analysis.window_bytes([base]) != analysis.window_bytes([p])
                for p in variants
            )
        )

    def test_identifiers_and_planner_names_are_excluded_but_events_retained(self):
        a, b = row(), row(planner="JPS")
        b.update(source_id="77", state_id="99", source_event_ids="2")
        p = dict(
            time_s=150.0,
            a_weight=0.5,
            b_weight=0.5,
            **{
                t + s: v
                for t in ["midpoint", "residual"]
                for s, v in [("_x", 0.2), ("_y", 0), ("_variance", 0.01)]
            }
        )
        e = dict(
            b,
            event_kind="received",
            event_time_s=150.0,
            truth_time_s=149.98,
            scheduled="True",
            received="True",
            precision_activation="False",
        )
        b["_window"] = analysis.payload(b, p, 149.98, {"2": e})
        self.assertEqual(
            analysis.window_bytes([a["_window"]]), analysis.window_bytes([b["_window"]])
        )
        families, cases, lookup = units([a, b])
        self.assertEqual(families[0]["exact_windows"], 1)
        self.assertEqual(families[0]["shared_window_cases"], 2)
        self.assertTrue(all(c["window_row_weight"] == 0.5 for c in cases))

    def test_equal_hashes_in_different_families_are_not_collapsed(self):
        rr = [row(), row(scenario="other")]
        families, cases, lookup = units(rr)
        self.assertEqual(len(families), 2)
        self.assertEqual(cases[0]["window_sha256"], cases[1]["window_sha256"])
        self.assertNotEqual(cases[0]["window_id"], cases[1]["window_id"])
        self.assertTrue(all(c["duplicate_cases"] == 1 for c in cases))

    def test_whole_windows_with_partial_overlap_stay_distinct(self):
        rr = [row(), row(planner="JPS"), row(planner="JPS", time=150.02)]
        families, cases, lookup = units(rr)
        self.assertEqual(families[0]["exact_windows"], 2)
        self.assertTrue(all(c["duplicate_cases"] == 1 for c in cases))
        self.assertEqual([c["parent_records"] for c in cases], [1, 2])

    def test_missing_planners_do_not_get_imputed_rows_or_weights(self):
        rr = [row()]
        accounting = account([*rr, row(planner="JPS"), row(planner="Theta*")])
        families, cases, lookup = units(rr, accounting)
        self.assertEqual(families[0]["planned_cases"], 3)
        self.assertEqual(families[0]["represented_cases"], 1)
        self.assertEqual(families[0]["missing_source_planners"], "JPS+Theta*")
        self.assertEqual(cases[0]["family_row_weight"], 1)
        self.assertTrue(
            all(
                c["window_id"] == c["window_sha256"] == c["case_row_weight"] == ""
                for c in cases[1:]
            )
        )

    def test_group_weights_precede_mode_conditioning(self):
        rr = [
            row(),
            row(time=150.02, error=0.3, mode="fusion_three"),
            row(planner="JPS", error=0.05, tick=9),
        ]
        _, _, lookup = units(rr)
        weights = [
            lookup[analysis.previous.identity(r)]["family_row_weight"] for r in rr
        ]
        self.assertEqual(weights, [F(1, 4), F(1, 4), F(1, 2)])
        selected = [
            (r, w) for r, w in zip(rr, weights) if r["stratum"] == "filter_single"
        ]
        self.assertEqual(sum(w for r, w in selected), F(3, 4))
        metrics, _ = analysis.previous.decompose(
            [r for r, w in selected],
            [w for r, w in selected],
            "residual",
            "residual_sigma",
            "observed",
            analysis.previous.SEEDS["fresh"],
        )
        self.assertEqual(metrics["denominator"], F(1, 8))
        self.assertEqual(metrics["auc"], 0)

    def test_replication_changes_case_auc_but_not_window_or_family_auc(self):
        original = [
            row(error=0.2, tick=1),
            row(time=150.02, error=0.05, tick=3),
            row(scenario="other", error=0.2, tick=3),
            row(scenario="other", time=150.02, error=0.05, tick=1),
        ]
        repeated = [
            *original,
            row(planner="JPS", error=0.2, tick=1),
            row(planner="JPS", time=150.02, error=0.05, tick=3),
        ]
        results = []
        for rr in [original, repeated]:
            _, _, lookup = units(rr)
            values = {}
            for method in analysis.METHODS:
                weights = [
                    lookup[analysis.previous.identity(r)][method + "_row_weight"]
                    for r in rr
                ]
                metric, _ = analysis.previous.decompose(
                    rr,
                    weights,
                    "residual",
                    "residual_sigma",
                    "observed",
                    analysis.previous.SEEDS["fresh"],
                )
                values[method] = metric["auc"]
            results.append(values)
        self.assertEqual(results[0], dict(case=F(1, 2), window=F(1, 2), family=F(1, 2)))
        self.assertEqual(results[1], dict(case=F(1, 3), window=F(1, 2), family=F(1, 2)))

    def test_duplicate_accounting_or_state_times_are_rejected(self):
        rr = [row()]
        accounting = account(rr)
        with self.assertRaisesRegex(AssertionError, "duplicate accounting"):
            units(rr, accounting + accounting)
        with self.assertRaisesRegex(AssertionError, "duplicate state time"):
            units(rr + rr)

    def test_unaccounted_cases_and_nonfinite_signatures_are_rejected(self):
        with self.assertRaisesRegex(AssertionError, "unaccounted source"):
            units([row()], [])
        p = row()["_window"]
        p["targets"]["midpoint"]["variance"] = float("nan")
        with self.assertRaises(AssertionError):
            analysis.window_bytes([p])

    def test_empty_families_and_grid_keep_undefined_metrics(self):
        families, cases, lookup = units([], account([row(), row(planner="JPS")]))
        self.assertEqual(families[0]["represented_cases"], 0)
        self.assertEqual(families[0]["exact_windows"], 0)
        components, ranks, blocks, summary = analysis.summaries([], lookup)
        self.assertEqual(
            tuple(map(len, [components, ranks, blocks, summary])),
            (360, 3240, 29160, 36),
        )
        self.assertTrue(
            all(
                r["auc"] == r["same_seed_auc"] == r["auc_difference"] == ""
                for r in ranks
            )
        )
        self.assertTrue(
            all(
                r["analysis_units"] == 0 and r["observed_rms_m"] == ""
                for r in components
            )
        )

    def test_ids_and_classification_are_stable_under_input_reordering(self):
        rr = [row(), row(planner="JPS"), row(scenario="other", error=0.05)]
        first = units(rr)[:2]
        second = units(rr[::-1], account(rr)[::-1])[:2]
        self.assertEqual(first, second)


if __name__ == "__main__":
    unittest.main()
