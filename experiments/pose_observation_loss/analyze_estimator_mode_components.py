#!/usr/bin/env python3
"""Observed midpoint/residual component energies within source estimator modes."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import combinations, product
import json
import math
from pathlib import Path

from analyze_commanded_recovery_arc import flag
from analyze_contrast_risk_transfer import auc, case_weights, membership
from analyze_epoch_preview import digest, read_rows
from analyze_estimator_mode_transfer import (
    ARMS,
    BUDGETS,
    GROUP,
    KINDS,
    NOISES,
    PARTITIONS,
    SCORES,
    SIM_VIEWS,
    STRATA,
    TARGETS,
)
from analyze_joint_admission_geometry import KEY
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
PRIOR = RESULTS / "estimator-mode-transfer"
PROTOCOL = RESULTS / "estimator-mode-components/protocol.md"
PINNED = {
    "estimator-mode-transfer/membership.csv": "e2924bd538b435b26ffab25e4bdeba7d93119e12355f6fed2f1d5371eac4f79c",
    "estimator-mode-transfer/stratum-summary.csv": "4aa032e61ea9ee647130a47afc0bdbea64168ab437cc127206994caecfc932e5",
    "estimator-mode-transfer/sources.json": "161adcd313a2abdaf58a6f4ef1eb08ed084402da7bc31614dbd9133ccca781b0",
    "estimator-mode-transfer/accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "estimator-mode-transfer/events.csv": "819b026c74884681e8df9fb282998a19ade0d12f82e8d1ffa3ad1a3b5ce1df6f",
    "estimator-mode-transfer/loss-links.csv": "b25ba5f8fe9f6cafbe1bc20a32b318455804adf3dc1639f10d335607fea7222b",
    "estimator-mode-transfer/thresholds.csv": "e877f83cb96aaf14c2a273c06416eb41998237fed8ffdfac0529b729a5c70207",
    "contrast-risk-transfer/scores.csv": "96e963f962a138f066f01d582d34e1a33a5d315de693f160fa37b3a3beca52e6",
    "contrast-risk-transfer/risk-summary.csv": "32a7547376b7e719acf87cb09cc80f971477995a4a8155d4c63765fe422dceba",
    "contrast-risk-transfer/ranking.csv": "da778a3433aa5159b01a67ae8eb93875a949b823ca5bd3436d7ab8393b141025",
    "contrast-risk-transfer/sources.json": "e69444ae8bd9f645aeb96e1caf1f979ea855ff3650ba5d3be87ed3907bbdc331",
    "contrast-common-errors/simultaneous.csv": "25e405945fd3e18ea93738866cc9b265bda55017ec0d04422ea03f52c6b31031",
    "speed-probe-error-components/components.csv": "60efb9edbba466a857c25043e60c0d9b851f6d3ff418d5e0f8e999281df991e8",
}
PARTS = ["noise", "anchor", "drift", "transport"]
LABELS = ["observed", "modeled", "unmodeled"]
CELLS = [
    "modeled_" + p + "_observed_" + e
    for p, e in product(["inside", "outside"], repeat=2)
]
ENERGIES = (
    [p + "_energy_m2" for p in PARTS]
    + [a + "_" + b + "_cross_m2" for a, b in combinations(PARTS, 2)]
    + [p + "_energy_m2" for p in ["observed", "modeled", "unmodeled", "partial_model"]]
    + ["modeled_unmodeled_cross_m2"]
)


def vector_terms(parts, variance, observed=None):
    """Use all signed cross terms; the saved observed vector sets its own label."""
    assert set(parts) == set(PARTS) and variance >= 0 and math.isfinite(variance)
    assert all(len(v) == 2 and all(math.isfinite(x) for x in v) for v in parts.values())
    total = [math.fsum(parts[p][j] for p in PARTS) for j in range(2)]
    observed = total if observed is None else observed
    closure = max(abs(a - b) for a, b in zip(total, observed))
    assert closure < 1e-8 and all(math.isfinite(v) for v in observed)
    modeled = [parts["noise"][j] + parts["anchor"][j] for j in range(2)]
    unmodeled = [parts["drift"][j] + parts["transport"][j] for j in range(2)]
    energy = {p + "_energy_m2": math.fsum(x * x for x in xy) for p, xy in parts.items()}
    energy.update(
        {
            a
            + "_"
            + b
            + "_cross_m2": 2 * math.fsum(x * y for x, y in zip(parts[a], parts[b]))
            for a, b in combinations(PARTS, 2)
        }
    )
    energy.update(
        {
            p + "_energy_m2": math.fsum(x * x for x in xy)
            for p, xy in [
                ("observed", observed),
                ("modeled", modeled),
                ("unmodeled", unmodeled),
            ]
        }
    )
    energy["partial_model_energy_m2"] = 2 * variance
    energy["modeled_unmodeled_cross_m2"] = 2 * math.fsum(
        x * y for x, y in zip(modeled, unmodeled)
    )
    assert (
        abs(
            energy["observed_energy_m2"]
            - energy["modeled_energy_m2"]
            - energy["unmodeled_energy_m2"]
            - energy["modeled_unmodeled_cross_m2"]
        )
        < 1e-8
    )
    outside = {
        p: math.hypot(*xy) > 0.1
        for p, xy in [
            ("observed", observed),
            ("modeled", modeled),
            ("unmodeled", unmodeled),
        ]
    }
    cell = (
        "modeled_"
        + ("outside" if outside["modeled"] else "inside")
        + "_observed_"
        + ("outside" if outside["observed"] else "inside")
    )
    return dict(
        energy=energy, outside=outside, cell=cell, parts=parts, vector_closure_m=closure
    )


def weighted_parts(a, b, weights):
    assert len(weights) == 2 and abs(sum(weights) - 1) < 1e-12
    for p in [a, b]:
        for axis in ["x", "y"]:
            assert (
                abs(
                    math.fsum(float(p[k + "_" + axis]) for k in PARTS)
                    - float(p["error_" + axis])
                )
                < 1e-8
            )
    return {
        p: [
            math.fsum(
                [
                    weights[0] * float(a[p + "_" + axis]),
                    weights[1] * float(b[p + "_" + axis]),
                ]
            )
            for axis in ["x", "y"]
        ]
        for p in PARTS
    }


def reconstruct(members, projections, poses, scores):
    pp = {p["source_id"]: p for p in projections}
    ii = {p["pose_id"]: p for p in poses}
    ss = {(r["source_family"], r["source_id"]): r for r in scores}
    assert (
        len(pp) == len(projections) and len(ii) == len(poses) and len(ss) == len(scores)
    )
    assert {m["source_id"] for m in members} == set(pp) and len(members) == len(pp)
    result, audits = [], []
    for m in members:
        assert m["source_family"] == "simultaneous" and flag(m["available"])
        p = pp[m["source_id"]]
        s = ss["simultaneous", m["source_id"]]
        a, b = ii[m["a_id"]], ii[m["b_id"]]
        assert all(
            m[k] == p[k] == s[k] for k in [*KEY, "a_id", "b_id", "source_event_ids"]
        )
        assert m["view"] == "sim_" + p["pairing"] == s["view"]
        assert membership(m) == (m["partition"], m["noise"])
        assert all(
            a[k] == b[k] == p[k]
            for k in [*KEY, "state_id", "time_s", "source_event_ids"]
        )
        assert (b["estimator"], m["anchor_mode"]) == STRATA[m["stratum"]]
        assert m["anchor_mode"] == p["anchor_mode"]
        assert a["estimator"] == m["a_estimator"] == p["a_estimator"]
        assert b["estimator"] == m["b_estimator"] == p["b_estimator"]
        r = dict(m, available=True)
        for score in SCORES:
            r[score + "_tick"] = int(s[score + "_tick"])
        audit = dict(source_id=m["source_id"], a_id=m["a_id"], b_id=m["b_id"])
        for target, weights in [
            ("midpoint", (0.5, 0.5)),
            ("residual", (float(p["a_weight"]), float(p["b_weight"]))),
        ]:
            parts = weighted_parts(a, b, weights)
            observed = [float(p[target + "_" + axis]) for axis in ["x", "y"]]
            terms = vector_terms(parts, float(p[target + "_variance"]), observed)
            norm_difference = abs(math.hypot(*observed) - float(p[target + "_norm"]))
            assert norm_difference < 1e-8
            assert terms["outside"]["observed"] == flag(s[target + "_outside"])
            audit[target + "_vector_closure_m"] = terms["vector_closure_m"]
            audit[target + "_norm_difference_m"] = norm_difference
            if target == "residual":
                difference = max(
                    abs(parts[part][j] - float(p["residual_" + part + "_" + axis]))
                    for part in PARTS
                    for j, axis in enumerate(["x", "y"])
                )
                assert difference < 1e-8
                audit["residual_component_difference_m"] = difference
            r[target] = terms
        result.append(r)
        audits.append(audit)
    return result, audits


def chosen(r, score, threshold):
    assert threshold["rule"] in ["fitted", "all_available"]
    return threshold["rule"] == "all_available" or r[score + "_tick"] <= int(
        threshold["threshold_tick"]
    )


def exported(row):
    return {
        k: "" if v is None else float(v) if isinstance(v, Fraction) else v
        for k, v in row.items()
    }


def component_metrics(rows, weights, target):
    assert len(rows) == len(weights) and all(w > 0 for w in weights)
    mass = sum(weights, Fraction())
    means = {
        name: (
            math.fsum(
                float(w / mass) * r[target]["energy"][name]
                for r, w in zip(rows, weights)
            )
            if mass
            else None
        )
        for name in ENERGIES
    }
    if mass:
        closure = (
            means["observed_energy_m2"]
            - means["modeled_energy_m2"]
            - means["unmodeled_energy_m2"]
            - means["modeled_unmodeled_cross_m2"]
        )
        assert abs(closure) < 1e-8
    else:
        closure = None
    means["energy_closure_m2"] = closure
    for name in ["observed", "modeled", "unmodeled", "partial_model"]:
        value = means[name + "_energy_m2"]
        means[name + "_rms_m"] = math.sqrt(value) if value is not None else None
    means["observed_to_partial_model_energy_ratio"] = (
        means["observed_energy_m2"] / means["partial_model_energy_m2"]
        if means["partial_model_energy_m2"]
        else None
    )
    for name in [*LABELS, *CELLS]:
        bad = [
            (r, w)
            for r, w in zip(rows, weights)
            if (
                r[target]["outside"][name]
                if name in LABELS
                else r[target]["cell"] == name
            )
        ]
        bad_mass = sum((w for r, w in bad), Fraction())
        prefix = name + "_outside" if name in LABELS else name
        means[prefix + "_rows"] = len(bad)
        means[prefix + "_case_mass"] = bad_mass
        means[prefix + "_fraction"] = bad_mass / mass if mass else None
    assert sum(means[c + "_rows"] for c in CELLS) == len(rows)
    assert sum(means[c + "_case_mass"] for c in CELLS) == mass
    return means


def diagnostic_ranking(rows, weights, target, score):
    result = {}
    for label in LABELS:
        outside = [r[target]["outside"][label] for r in rows]
        result[label + "_outside_rows"] = sum(outside)
        result[label + "_outside_case_mass"] = sum(
            (w for w, b in zip(weights, outside) if b), Fraction()
        )
        result[label + "_case_weighted_auc"] = auc(
            [r[score + "_tick"] for r in rows], outside, weights
        )
    return result


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
    summaries, ranking = [], []
    for key in product(PARTITIONS, NOISES, ARMS, ["all", *KINDS], SIM_VIEWS):
        rr = groups[key]
        ww = case_weights(rr)
        group_mass = sum(ww, Fraction())
        for stratum in ["all", *STRATA]:
            sub = [
                (r, w)
                for r, w in zip(rr, ww)
                if stratum == "all" or r["stratum"] == stratum
            ]
            mass = sum((w for r, w in sub), Fraction())
            common = dict(zip(GROUP, key))
            common.update(
                stratum=stratum,
                records=len(sub),
                cases=len({tuple(r[k] for k in KEY) for r, w in sub}),
                case_mass=mass,
                group_case_mass=group_mass,
                case_mass_share=mass / group_mass if group_mass else None,
            )
            for score in SCORES:
                for target in TARGETS:
                    ranking.append(
                        exported(
                            dict(
                                **common,
                                score=score,
                                target=target,
                                **diagnostic_ranking(
                                    [r for r, w in sub],
                                    [w for r, w in sub],
                                    target,
                                    score,
                                )
                            )
                        )
                    )
                for budget in BUDGETS:
                    t = cuts[key[1], key[4], score, budget]
                    kept = [(r, w) for r, w in sub if chosen(r, score, t)]
                    kept_mass = sum((w for r, w in kept), Fraction())
                    retained = dict(
                        retained_rows=len(kept),
                        retained_cases=len({tuple(r[k] for k in KEY) for r, w in kept}),
                        retained_case_mass=kept_mass,
                        case_weighted_coverage=kept_mass / mass if mass else None,
                        retained_share_of_group=(
                            kept_mass / group_mass if group_mass else None
                        ),
                    )
                    for target in TARGETS:
                        summaries.append(
                            exported(
                                dict(
                                    **common,
                                    score=score,
                                    budget=budget,
                                    target=target,
                                    threshold_id=t["threshold_id"],
                                    rule=t["rule"],
                                    threshold_tick=t["threshold_tick"],
                                    **retained,
                                    **component_metrics(
                                        [r for r, w in kept],
                                        [w for r, w in kept],
                                        target,
                                    )
                                )
                            )
                        )
    return summaries, ranking


def close(a, b, context):
    if a in ["", None] or b in ["", None]:
        assert a in ["", None] and b in ["", None], context
    else:
        assert math.isclose(float(a), float(b), rel_tol=1e-11, abs_tol=1e-12), context


def check_prior(components, ranking, strata, risks, ranks):
    skey = [*GROUP, "score", "budget", "stratum"]
    previous = {tuple(r[k] for k in skey): r for r in strata}
    pooled = {
        tuple(r[k] for k in [*GROUP, "score", "budget"]): r
        for r in risks
        if r["view"] in SIM_VIEWS
    }
    for r in components:
        is_pooled = r["stratum"] == "all"
        fields = skey[:-1] if is_pooled else skey
        p = (pooled if is_pooled else previous)[tuple(r[k] for k in fields)]
        for f in [
            "records",
            "cases",
            "retained_rows",
            "retained_cases",
            "retained_case_mass",
            "case_weighted_coverage",
        ]:
            close(r[f], p[f], (f, r))
        for new, old in [
            ("observed_outside_rows", "_error_rows"),
            ("observed_outside_case_mass", "_error_case_mass"),
            ("observed_outside_fraction", "_case_weighted_risk"),
        ]:
            close(r[new], p[r["target"] + old], (new, r))
    pooled_ranks = {
        tuple(r[k] for k in [*GROUP, "score"]): r
        for r in ranks
        if r["view"] in SIM_VIEWS
    }
    for r in ranking:
        if r["stratum"] == "all":
            p = pooled_ranks[tuple(r[k] for k in [*GROUP, "score"])]
        else:
            p = previous[tuple(r[k] for k in [*GROUP, "score"]) + ("all", r["stratum"])]
        close(r["observed_case_weighted_auc"], p[r["target"] + "_case_weighted_auc"], r)
    return len(components), len(ranking)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for name, sha in PINNED.items():
        assert digest(RESULTS / name) == sha, name
    prior = json.loads((PRIOR / "sources.json").read_text())
    helpers = dict(prior["analysis"]["helpers_sha256"])
    helpers["analyze_estimator_mode_transfer.py"] = prior["analysis"]["script_sha256"]
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    members, projections, poses, scores, thresholds, ee, accounting, links = [
        read_rows(RESULTS / name)
        for name in [
            "estimator-mode-transfer/membership.csv",
            "contrast-common-errors/simultaneous.csv",
            "speed-probe-error-components/components.csv",
            "contrast-risk-transfer/scores.csv",
            "estimator-mode-transfer/thresholds.csv",
            "estimator-mode-transfer/events.csv",
            "estimator-mode-transfer/accounting.csv",
            "estimator-mode-transfer/loss-links.csv",
        ]
    ]
    assert (len(members), len(projections), len(poses), len(scores)) == (
        19830,
        19830,
        19830,
        40494,
    )
    assert (len(thresholds), len(ee), len(accounting), len(links)) == (
        264,
        2583,
        648,
        550,
    )
    assert sum(flag(r["selected"]) for r in accounting) == 368
    assert sum(flag(r["command_only_zero_loss"]) for r in links) == 509
    rows, audits = reconstruct(members, projections, poses, scores)
    summary, ranking = summaries(rows, thresholds, {e["event_id"]: e for e in ee})
    assert (len(summary), len(ranking)) == (13824, 3456)
    checked = check_prior(
        summary,
        ranking,
        read_rows(PRIOR / "stratum-summary.csv"),
        read_rows(RESULTS / "contrast-risk-transfer/risk-summary.csv"),
        read_rows(RESULTS / "contrast-risk-transfer/ranking.csv"),
    )
    audit = dict(
        accounted_cases=648,
        timeout_cases=368,
        opportunities=2583,
        simultaneous_references=len(rows),
        simultaneous_states=6610,
        simultaneous_cases=100,
        source_opportunities=171,
        inherited_lagged_requested=20664,
        inherited_lagged_unavailable=2208,
        raw_zero_loss_links=550,
        command_only_links=509,
        reconstruction_rows=len(audits),
        component_summary_rows=len(summary),
        diagnostic_ranking_rows=len(ranking),
        prior_retention_target_rows_checked=checked[0],
        prior_auc_target_rows_checked=checked[1],
        max_vector_closure_m=max(
            r[t + "_vector_closure_m"] for r in audits for t in TARGETS
        ),
        max_residual_component_difference_m=max(
            r["residual_component_difference_m"] for r in audits
        ),
    )
    source = dict(
        input_sha256=PINNED,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        schema=dict(
            parts=PARTS,
            targets=TARGETS,
            strata=["all", *STRATA],
            views=SIM_VIEWS,
            scores=SCORES,
            budgets=list(BUDGETS),
            labels=LABELS,
            accuracy_radius_m=0.1,
            energy_reference="two-axis 2*partial variance",
            weighting="parent reporting-group 1/n_case retained through stratification/selection",
            retrospective=True,
            evaluation_previously_observed=True,
        ),
        inherited_validation=dict(
            source_sha256=PINNED["estimator-mode-transfer/sources.json"],
            observation_and_physics=True,
            new_rollout=False,
            held_goals=[140, 140],
        ),
        audit=audit,
    )
    args.output.mkdir(parents=True)
    for name in [
        "accounting.csv",
        "events.csv",
        "loss-links.csv",
        "thresholds.csv",
        "membership.csv",
    ]:
        (args.output / name).write_bytes((PRIOR / name).read_bytes())
        assert digest(args.output / name) == PINNED["estimator-mode-transfer/" + name]
    for name, data in [
        ("reconstruction", audits),
        ("component-summary", summary),
        ("diagnostic-ranking", ranking),
    ]:
        save_rows(args.output / (name + ".csv"), data)
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
