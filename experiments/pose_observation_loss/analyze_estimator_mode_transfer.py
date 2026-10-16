#!/usr/bin/env python3
"""Decompose retrospective fixed-cutoff shifts by source estimator mode."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path

from analyze_commanded_recovery_arc import flag
from analyze_contrast_risk_transfer import (
    ARMS,
    BUDGETS,
    KINDS,
    NOISES,
    PARTITIONS,
    SCORES,
    SIM_VIEWS,
    TARGETS,
    auc,
    case_weights,
    membership,
)
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import KEY
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/contrast-risk-transfer"
COMMON = ROOT / "results/contrast-common-errors"
PROTOCOL = ROOT / "results/estimator-mode-transfer/protocol.md"
PINNED = {
    "scores.csv": "96e963f962a138f066f01d582d34e1a33a5d315de693f160fa37b3a3beca52e6",
    "thresholds.csv": "e877f83cb96aaf14c2a273c06416eb41998237fed8ffdfac0529b729a5c70207",
    "risk-summary.csv": "32a7547376b7e719acf87cb09cc80f971477995a4a8155d4c63765fe422dceba",
    "sources.json": "e69444ae8bd9f645aeb96e1caf1f979ea855ff3650ba5d3be87ed3907bbdc331",
    "accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "events.csv": "819b026c74884681e8df9fb282998a19ade0d12f82e8d1ffa3ad1a3b5ce1df6f",
    "loss-links.csv": "b25ba5f8fe9f6cafbe1bc20a32b318455804adf3dc1639f10d335607fea7222b",
}
COMMON_PINNED = {
    "simultaneous.csv": "25e405945fd3e18ea93738866cc9b265bda55017ec0d04422ea03f52c6b31031",
    "sources.json": "6603a1e1960b94a83ef0f4b2a7e844ed18bbe2722a1d22df0465e7a5f3a10aa2",
}
STRATA = {
    "recovery_single": ("recovery_mean", "shared_single"),
    "filter_single": ("motion_filter", "shared_single"),
    "fusion_three": ("three_epoch", "preview_three"),
}
GROUP = ["partition", "noise", "variant", "event_kind", "view"]
SHIFT_FIELDS = [
    "known",
    "new",
    "delta",
    "new_mix_known_rates",
    "known_mix_new_rates",
    "composition",
    "within",
    "closure_error",
]


def join_records(scores, simultaneous, events):
    """Use explicit source identities; lagged records never acquire a stratum."""
    indices = {(r["source_family"], r["source_id"]): r for r in scores}
    assert len(indices) == len(scores)
    source = {r["source_id"]: r for r in simultaneous}
    assert len(source) == len(simultaneous)
    assert set(source) == {sid for family, sid in indices if family == "simultaneous"}
    result = []
    reverse = {v: k for k, v in STRATA.items()}
    for sid, meta in source.items():
        r = dict(indices["simultaneous", sid])
        assert all(r[k] == meta[k] for k in [*KEY, "a_id", "b_id", "source_event_ids"])
        assert r["view"] == "sim_" + meta["pairing"] in SIM_VIEWS
        assert membership(r) == (r["partition"], r["noise"])
        assert flag(r["available"]) and flag(meta["available"])
        assert not flag(r["oracle_contrast"])
        expected_a = (
            "recovery_mean"
            if r["view"] == "sim_recovery_preview"
            else "raw_observation"
        )
        assert meta["a_estimator"] == expected_a
        mode = (meta["b_estimator"], meta["anchor_mode"])
        assert mode in reverse
        stratum = reverse[mode]
        assert (stratum == "recovery_single") == (r["view"] == "sim_raw_recovery")
        for eid in r["source_event_ids"].split("+"):
            assert all(events[eid][k] == r[k] for k in KEY)
        for k in ["available", "zero_or_roundoff_contrast"]:
            r[k] = flag(r[k])
        for score in SCORES:
            r[score + "_tick"] = int(r[score + "_tick"])
        for target in TARGETS:
            r[target + "_outside"] = flag(r[target + "_outside"])
        r.update(
            stratum=stratum,
            a_estimator=meta["a_estimator"],
            b_estimator=meta["b_estimator"],
            anchor_mode=meta["anchor_mode"],
        )
        result.append(r)
    return result


def membership_rows(rows):
    fields = [
        "source_family",
        "source_id",
        *KEY,
        "partition",
        "noise",
        "view",
        "source_event_ids",
        "a_id",
        "b_id",
        "a_estimator",
        "b_estimator",
        "anchor_mode",
        "stratum",
        "available",
    ]
    return [{k: r[k] for k in fields} for r in rows]


def selected(row, score, threshold):
    if threshold["rule"] == "all_available":
        return True
    if threshold["rule"] == "no_training_data":
        return None
    assert threshold["rule"] == "fitted"
    return row[score + "_tick"] <= int(threshold["threshold_tick"])


def cell(rows, weights, score, threshold, group_mass):
    """Keep the parent group's weights, including cases crossing strata."""
    assert len(rows) == len(weights)
    mass = sum(weights, Fraction())
    rule_available = threshold["rule"] != "no_training_data"
    rr = [(r, w) for r, w in zip(rows, weights) if selected(r, score, threshold)]
    kept = sum((w for r, w in rr), Fraction()) if rule_available else None
    result = dict(
        records=len(rows),
        cases=len({tuple(r[k] for k in KEY) for r in rows}),
        case_mass=mass,
        case_mass_share=mass / group_mass if group_mass else None,
        retained_rows=len(rr) if rule_available else None,
        retained_cases=(
            len({tuple(r[k] for k in KEY) for r, w in rr}) if rule_available else None
        ),
        retained_case_mass=kept,
        case_weighted_coverage=kept / mass if mass and kept is not None else None,
        retained_share_of_group=(
            kept / group_mass if group_mass and kept is not None else None
        ),
        zero_or_roundoff_pairs=sum(r["zero_or_roundoff_contrast"] for r in rows),
    )
    for target in TARGETS:
        bad = sum(
            (w for r, w in zip(rows, weights) if r[target + "_outside"]), Fraction()
        )
        retained_bad = (
            sum((w for r, w in rr if r[target + "_outside"]), Fraction())
            if rule_available
            else None
        )
        result.update(
            {
                target + "_outside_rows": sum(r[target + "_outside"] for r in rows),
                target + "_outside_case_mass": bad,
                target + "_baseline_case_weighted_risk": bad / mass if mass else None,
                target
                + "_error_rows": (
                    sum(r[target + "_outside"] for r, w in rr)
                    if rule_available
                    else None
                ),
                target + "_error_case_mass": retained_bad,
                target
                + "_joint_fraction": (
                    retained_bad / mass if mass and retained_bad is not None else None
                ),
                target + "_case_weighted_risk": retained_bad / kept if kept else None,
                target
                + "_case_weighted_auc": auc(
                    [r[score + "_tick"] for r in rows],
                    [r[target + "_outside"] for r in rows],
                    weights,
                ),
            }
        )
    return result


def average(shares, values):
    """Zero-mass strata need no rate; positive unsupported strata stay missing."""
    if not sum(shares) or any(p and v is None for p, v in zip(shares, values)):
        return None
    assert sum(shares) == 1
    return sum((p * v for p, v in zip(shares, values) if p), Fraction())


def decomposition(pk, pn, vk, vn):
    assert len(pk) == len(pn) == len(vk) == len(vn)
    result = dict.fromkeys(SHIFT_FIELDS)
    known, new = average(pk, vk), average(pn, vn)
    result.update(known=known, new=new)
    if known is None or new is None:
        return result
    result["delta"] = new - known
    mix = average(pn, vk)
    standard = average(pk, vn)
    result.update(new_mix_known_rates=mix, known_mix_new_rates=standard)
    if mix is not None:
        result["composition"] = mix - known
        result["within"] = new - mix
        result["closure_error"] = (
            result["delta"] - result["composition"] - result["within"]
        )
        assert result["closure_error"] == 0
    return result


def ratio(numerator, denominator):
    return numerator / denominator if numerator is not None and denominator else None


def export(row):
    return {
        k: "" if v is None else float(v) if isinstance(v, Fraction) else v
        for k, v in row.items()
    }


def summaries(rows, thresholds, events):
    groups = defaultdict(list)
    for r in rows:
        kinds = {
            "all",
            *[events[e]["event_kind"] for e in r["source_event_ids"].split("+")],
        }
        for kind in kinds:
            groups[r["partition"], r["noise"], r["variant"], kind, r["view"]].append(r)
    cuts = {(t["noise"], t["view"], t["score"], t["budget"]): t for t in thresholds}
    states, aggregates, strata = {}, {}, []
    for partition, noise, arm, kind, view in product(
        PARTITIONS, NOISES, ARMS, ["all", *KINDS], SIM_VIEWS
    ):
        key = partition, noise, arm, kind, view
        rr = groups[key]
        weights = case_weights(rr)
        mass = sum(weights, Fraction())
        common = dict(zip(GROUP, key))
        subsets = {
            s: [(r, w) for r, w in zip(rr, weights) if r["stratum"] == s]
            for s in STRATA
        }
        for score, budget in product(SCORES, BUDGETS):
            t = cuts[noise, view, score, budget]
            info = dict(
                **common,
                score=score,
                budget=budget,
                threshold_id=t["threshold_id"],
                rule=t["rule"],
                threshold_tick=t["threshold_tick"]
            )
            aggregates[(*key, score, budget)] = cell(rr, weights, score, t, mass)
            for s, subset in subsets.items():
                cr = [r for r, w in subset]
                cw = [w for r, w in subset]
                state = cell(cr, cw, score, t, mass)
                states[(*key, score, budget, s)] = state
                strata.append(export(dict(**info, stratum=s, **state)))
    transfer = []
    for noise, arm, kind, view, score, budget in product(
        NOISES, ARMS, ["all", *KINDS], SIM_VIEWS, SCORES, BUDGETS
    ):
        key = noise, arm, kind, view, score, budget
        kk = ("known", *key)
        nk = ("new", *key)
        known = [states[(*kk, s)] for s in STRATA]
        new = [states[(*nk, s)] for s in STRATA]
        pk = [s["case_mass_share"] or Fraction() for s in known]
        pn = [s["case_mass_share"] or Fraction() for s in new]
        missing_known = [s for s, a, b in zip(STRATA, pk, pn) if b and not a]
        missing_new = [s for s, a, b in zip(STRATA, pk, pn) if a and not b]
        t = cuts[noise, view, score, budget]
        record = dict(
            zip(["noise", "variant", "event_kind", "view", "score", "budget"], key)
        )
        record.update(
            threshold_id=t["threshold_id"],
            rule=t["rule"],
            threshold_tick=t["threshold_tick"],
            missing_known_strata="+".join(missing_known),
            missing_new_strata="+".join(missing_new),
            composition_supported=bool(
                sum(pk)
                and sum(pn)
                and not missing_known
                and t["rule"] != "no_training_data"
            ),
            standardization_supported=bool(
                sum(pk)
                and sum(pn)
                and not missing_new
                and t["rule"] != "no_training_data"
            ),
        )
        for partition, g in [("known", aggregates[kk]), ("new", aggregates[nk])]:
            for field in [
                "records",
                "cases",
                "case_mass",
                "retained_rows",
                "retained_cases",
                "retained_case_mass",
            ]:
                record[partition + "_" + field] = g[field]
        coverage = decomposition(
            pk,
            pn,
            [s["case_weighted_coverage"] for s in known],
            [s["case_weighted_coverage"] for s in new],
        )
        record.update({"coverage_" + k: v for k, v in coverage.items()})
        for target in TARGETS:
            joint = decomposition(
                pk,
                pn,
                [s[target + "_joint_fraction"] for s in known],
                [s[target + "_joint_fraction"] for s in new],
            )
            record.update({target + "_joint_" + k: v for k, v in joint.items()})
            for name in ["known", "new", "new_mix_known_rates", "known_mix_new_rates"]:
                record[target + "_risk_" + name] = ratio(joint[name], coverage[name])
        transfer.append(export(record))
    return strata, transfer, aggregates


def check_prior(aggregates, prior):
    """Every parent aggregate must reproduce the previously tracked retention."""
    rows = [r for r in prior if r["view"] in SIM_VIEWS]
    assert len(rows) == len(aggregates)
    fields = [
        "retained_rows",
        "retained_cases",
        "retained_case_mass",
        "case_weighted_coverage",
    ]
    for target in TARGETS:
        fields += [
            target + "_error_rows",
            target + "_error_case_mass",
            target + "_case_weighted_risk",
        ]
    for r in rows:
        a = aggregates[tuple(r[k] for k in [*GROUP, "score", "budget"])]
        assert a["records"] == int(r["records"]) == int(r["matched"])
        assert a["cases"] == int(r["cases"])
        for field in fields:
            if a[field] is None:
                assert r[field] == "", (field, r)
            else:
                assert math.isclose(
                    float(a[field]), float(r[field]), rel_tol=1e-12, abs_tol=1e-12
                ), (field, r)
    return len(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for root, pins in [(PRIOR, PINNED), (COMMON, COMMON_PINNED)]:
        for name, sha in pins.items():
            assert digest(root / name) == sha, name
    prior = json.loads((PRIOR / "sources.json").read_text())
    helpers = dict(prior["analysis"]["helpers_sha256"])
    helpers["analyze_contrast_risk_transfer.py"] = prior["analysis"]["script_sha256"]
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    scores, thresholds, ee, accounting, links = [
        read_rows(PRIOR / n)
        for n in [
            "scores.csv",
            "thresholds.csv",
            "events.csv",
            "accounting.csv",
            "loss-links.csv",
        ]
    ]
    assert (len(scores), len(thresholds), len(ee), len(accounting), len(links)) == (
        40494,
        264,
        2583,
        648,
        550,
    )
    assert sum(flag(r["selected"]) for r in accounting) == 368
    assert sum(flag(r["command_only_zero_loss"]) for r in links) == 509
    events = {e["event_id"]: e for e in ee}
    rows = join_records(scores, read_rows(COMMON / "simultaneous.csv"), events)
    assert len(rows) == 19830
    assert len({tuple(r[k] for k in KEY) for r in rows}) == 100
    strata, transfer, aggregates = summaries(rows, thresholds, events)
    checked = check_prior(aggregates, read_rows(PRIOR / "risk-summary.csv"))
    assert (len(strata), len(transfer), checked) == (5184, 864, 1728)
    audit = dict(
        accounted_cases=648,
        timeout_cases=368,
        opportunities=2583,
        simultaneous_references=len(rows),
        simultaneous_cases=100,
        inherited_lagged_requested=20664,
        inherited_lagged_available=18456,
        inherited_lagged_unavailable=2208,
        raw_zero_loss_links=550,
        command_only_links=509,
        unchanged_thresholds=len(thresholds),
        stratum_rows=len(strata),
        transfer_rows=len(transfer),
        prior_retention_rows_checked=checked,
        nonempty_transfer_groups=sum(
            bool(r["known_records"] and r["new_records"]) for r in transfer
        ),
        composition_supported=sum(r["composition_supported"] for r in transfer),
        standardization_supported=sum(r["standardization_supported"] for r in transfer),
    )
    source = dict(
        input_sha256={"contrast-risk-transfer/" + n: h for n, h in PINNED.items()},
        common_input_sha256={
            "contrast-common-errors/" + n: h for n, h in COMMON_PINNED.items()
        },
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        schema=dict(
            strata=STRATA,
            partitions={k: sorted(v) for k, v in PARTITIONS.items()},
            views=SIM_VIEWS,
            scores=SCORES,
            targets=TARGETS,
            budgets=list(BUDGETS),
            weighting="parent reporting-group 1/n_case, retained within strata",
            decomposition_order="known rates, new mixture first",
            retrospective=True,
            evaluation_previously_observed=True,
        ),
        inherited_validation=dict(
            source_sha256=PINNED["sources.json"],
            observation_and_physics=True,
            new_rollout=False,
            held_goals=[140, 140],
        ),
        audit=audit,
    )
    args.output.mkdir(parents=True)
    for name in ["accounting.csv", "events.csv", "loss-links.csv", "thresholds.csv"]:
        (args.output / name).write_bytes((PRIOR / name).read_bytes())
        assert digest(args.output / name) == PINNED[name]
    for name, rr in [
        ("membership", membership_rows(rows)),
        ("stratum-summary", strata),
        ("transfer-summary", transfer),
    ]:
        save_rows(args.output / (name + ".csv"), rr)
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
