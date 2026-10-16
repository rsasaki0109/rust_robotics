#!/usr/bin/env python3
"""Diagnose additional-seed target components under the immutable cutoff rules."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path
import shutil

from analyze_commanded_recovery_arc import flag
from analyze_contrast_risk_transfer import (
    ARMS,
    BUDGETS,
    KINDS,
    NOISES,
    SCORES,
    SIM_VIEWS,
    TARGETS,
    case_weights,
    score_tick,
)
from analyze_epoch_preview import digest, read_rows
from analyze_estimator_mode_components import (
    CELLS,
    ENERGIES,
    LABELS,
    PARTS,
    component_metrics,
    diagnostic_ranking,
    vector_terms,
    weighted_parts,
)
from analyze_fresh_seed_cutoffs import STRATUM_NAMES, stratum
from analyze_fresh_seed_modes import identity
from analyze_joint_admission_geometry import KEY
from analyze_mode_conditional_cutoffs import decision, exported
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/fresh-seed-cutoff-transfer"
PROTOCOL = ROOT / "results/fresh-seed-components/protocol.md"
PINNED = {
    "scores.csv": "1c7d5e57b00bdf85b278a08e044eac475d68e9815e6af04700f1b66100d197a9",
    "poses.csv": "faf7be87a3c65ec7c7d06b9c74f0495ebc5538997088bfe9d2a34a1ebc726c0e",
    "projections.csv": "e13db26d96825437be55d91596863e5bce0e1f000ceb7f90c5552aec78d7ea47",
    "accounting.csv": "00e055290eb36175bd7951510ada1a31957eff41e853049816a7d6bc86a90462",
    "case-support.csv": "5e9c11451f5cba49a7602d0a0f97556be60940874c392917f44812d4ea4e78e8",
    "events.csv": "3a3d0600d53f996cf7be766a5adc98b041c7a6ad378be6083a2403efd9a37679",
    "loss-links.csv": "5fff8b5999fdbff09d6120399c55f18e9232d6643e3d14f8a8bc24e174f6e65b",
    "thresholds.csv": "e877f83cb96aaf14c2a273c06416eb41998237fed8ffdfac0529b729a5c70207",
    "conditional-thresholds.csv": "4d20ddb96399552a932aa4c0db904b7e6c085291863b9273e451c5692410c040",
    "comparison.csv": "a0f21efb75f7811e235a1c8bc5a9462c7d77731ba4ce9ed30c3712a970c76cf3",
    "sources.json": "a99a3d2aa0644a26b632a47c21efdccee37f936e62b1a33795c828742a20a5fd",
}
METHODS = ["pooled", "conditional"]
GROUP = ["partition", "noise", "variant", "event_kind", "view"]


def reconstruct(scores, projections, poses):
    pp = {p["source_id"]: p for p in projections}
    ss = {s["source_id"]: s for s in scores}
    pose_key = lambda p: (identity(p), p["state_id"], p["pose_kind"])
    ii = {pose_key(p): p for p in poses}
    assert (
        len(ss) == len(scores) and len(pp) == len(projections) and set(ss) == set(pp)
    ), "pair ID support"
    assert len(ii) == len(poses), "duplicate pose key"
    expected = {
        (identity(s), s["state_id"], kind)
        for s in scores
        for kind in ["raw", "recovery", "preview"]
    }
    assert set(ii) == expected, "pose support"
    result, audits = [], []
    for s in scores:
        p = pp[s["source_id"]]
        assert (
            s["source_family"] == "simultaneous"
            and flag(s["available"])
            and s["partition"] == "fresh"
        )
        assert s["view"] in SIM_VIEWS and int(s["seed"]) in {277, 278, 279}
        assert s["cohort"] == "fresh-" + s["noise"]
        for k in [
            *KEY,
            "partition",
            "noise",
            "state_id",
            "time_s",
            "truth_time_s",
            "view",
            "source_event_ids",
            "a_estimator",
            "b_estimator",
            "anchor_mode",
            "stratum",
        ]:
            assert s[k] == p[k], (s["source_id"], k)
        ak, bk = s["view"][4:].split("_")
        a, b = [ii[identity(s), s["state_id"], kind] for kind in [ak, bk]]
        for pose, est in [(a, s["a_estimator"]), (b, s["b_estimator"])]:
            assert pose["estimator"] == est
            for k in [
                *KEY,
                "partition",
                "noise",
                "state_id",
                "time_s",
                "truth_time_s",
                "source_event_ids",
            ]:
                assert pose[k] == s[k], k
        assert s["stratum"] == stratum(s["view"], s["b_estimator"], s["anchor_mode"])
        r = dict(s, available=True)
        for score in SCORES:
            r[score + "_tick"] = int(s[score + "_tick"])
            assert r[score + "_tick"] == score_tick(float(s[score + "_score_m"]))
        audit = dict(
            source_id=s["source_id"],
            **{k: s[k] for k in [*KEY, "state_id", "time_s", "view"]}
        )
        for target, weights in [
            ("midpoint", (0.5, 0.5)),
            ("residual", (float(p["a_weight"]), float(p["b_weight"]))),
        ]:
            parts = weighted_parts(a, b, weights)
            observed = [float(p[target + "_" + axis]) for axis in ["x", "y"]]
            terms = vector_terms(parts, float(p[target + "_variance"]), observed)
            delta = abs(math.hypot(*observed) - float(s[target + "_error_m"]))
            assert (
                delta < 1e-8
                and abs(math.hypot(*observed) - float(p[target + "_norm"])) < 1e-8
            )
            assert terms["outside"]["observed"] == flag(s[target + "_outside"])
            audit[target + "_vector_closure_m"] = terms["vector_closure_m"]
            audit[target + "_norm_difference_m"] = delta
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


def selection(rows, method, score, budget, pooled, conditional):
    choices = []
    for r in rows:
        if method == "pooled":
            cut = pooled[r["noise"], r["view"], score, budget]
        else:
            cut = conditional.get(
                (r["noise"], r["view"], r["stratum"], score, budget),
                dict(rule="all_available" if budget == "all" else "no_training_data"),
            )
        choices.append(decision(r, score, cut))
    return choices


def summaries(rows, pooled_rows, conditional_rows, events):
    grouped = defaultdict(list)
    for r in rows:
        ids = r["source_event_ids"].split("+")
        for eid in ids:
            assert identity(events[eid]) == identity(r)
        for kind in {"all", *[events[e]["event_kind"] for e in ids]}:
            grouped[r["partition"], r["noise"], r["variant"], kind, r["view"]].append(r)
    pooled = {(t["noise"], t["view"], t["score"], t["budget"]): t for t in pooled_rows}
    conditional = {
        (t["noise"], t["view"], t["stratum"], t["score"], t["budget"]): t
        for t in conditional_rows
    }
    output, ranking = [], []
    for key in product(["fresh"], NOISES, ARMS, ["all", *KINDS], SIM_VIEWS):
        parent = grouped[key]
        ww = case_weights(parent)
        parent_mass = sum(ww, Fraction())
        for mode in ["all", *STRATUM_NAMES]:
            sub = [
                (r, w)
                for r, w in zip(parent, ww)
                if mode == "all" or r["stratum"] == mode
            ]
            rr, weights = [r for r, w in sub], [w for r, w in sub]
            mass = sum(weights, Fraction())
            common = dict(zip(GROUP, key))
            common.update(
                stratum=mode,
                records=len(rr),
                cases=len({identity(r) for r in rr}),
                case_mass=mass,
                group_case_mass=parent_mass,
                case_mass_share=mass / parent_mass if parent_mass else None,
            )
            for score in SCORES:
                for target in TARGETS:
                    ranking.append(
                        exported(
                            dict(
                                **common,
                                score=score,
                                target=target,
                                **diagnostic_ranking(rr, weights, target, score)
                            )
                        )
                    )
                for budget, method in product(BUDGETS, METHODS):
                    choices = selection(rr, method, score, budget, pooled, conditional)
                    unknown = any(f is None for f in choices)
                    kept = [
                        (r, w) for r, w, f in zip(rr, weights, choices) if f is True
                    ]
                    retained_mass = sum((w for r, w in kept), Fraction())
                    retained = dict(
                        retained_rows=None if unknown else len(kept),
                        retained_cases=(
                            None if unknown else len({identity(r) for r, w in kept})
                        ),
                        retained_case_mass=None if unknown else retained_mass,
                        case_weighted_coverage=(
                            None if unknown or not mass else retained_mass / mass
                        ),
                        retained_share_of_group=(
                            None
                            if unknown or not parent_mass
                            else retained_mass / parent_mass
                        ),
                        retention_support=(
                            "unsupported" if unknown else "complete" if rr else "empty"
                        ),
                    )
                    for target in TARGETS:
                        metrics = component_metrics(
                            [r for r, w in kept], [w for r, w in kept], target
                        )
                        if unknown:
                            metrics = dict.fromkeys(metrics)
                        output.append(
                            exported(
                                dict(
                                    **common,
                                    score=score,
                                    budget=budget,
                                    method=method,
                                    target=target,
                                    **retained,
                                    **metrics
                                )
                            )
                        )
    return output, ranking


def check_previous(rows, previous):
    fields = [*GROUP, "stratum", "score", "budget"]
    lookup = {tuple(r[k] for k in fields): r for r in previous}
    assert len(lookup) == len(previous)
    comparisons = 0
    maximum = 0.0
    for r in rows:
        p = lookup[tuple(r[k] for k in fields)]
        method = r["method"]
        target = r["target"]
        pairs = [
            (f, f)
            for f in [
                "records",
                "cases",
                "case_mass",
                "group_case_mass",
                "case_mass_share",
            ]
        ]
        pairs += [
            (f, method + "_" + f)
            for f in [
                "retained_rows",
                "retained_cases",
                "retained_case_mass",
                "case_weighted_coverage",
            ]
        ]
        pairs += [
            ("observed_outside_rows", method + "_" + target + "_error_rows"),
            ("observed_outside_case_mass", method + "_" + target + "_error_case_mass"),
            (
                "observed_outside_fraction",
                method + "_" + target + "_case_weighted_risk",
            ),
            ("observed_rms_m", method + "_" + target + "_case_weighted_rms"),
        ]
        for new, old in pairs:
            a, b = r[new], p[old]
            if a == "":
                assert b == "", (new, a, b)
            else:
                assert b != "" and math.isclose(
                    float(a), float(b), rel_tol=1e-11, abs_tol=1e-12
                ), (new, a, b)
                maximum = max(maximum, abs(float(a) - float(b)))
            comparisons += 1
    return dict(
        component_rows_checked=len(rows),
        field_comparisons=comparisons,
        max_numeric_discrepancy=maximum,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for n, sha in PINNED.items():
        assert digest(PRIOR / n) == sha, n
    prior = json.loads((PRIOR / "sources.json").read_text())
    for n, sha in prior["analysis"]["helpers_sha256"].items():
        assert digest(ROOT / n) == sha, n
    assert (
        digest(ROOT / "analyze_fresh_seed_cutoffs.py")
        == prior["analysis"]["script_sha256"]
    )
    scores, projections, poses, pooled, conditional, ee, previous = [
        read_rows(PRIOR / n)
        for n in [
            "scores.csv",
            "projections.csv",
            "poses.csv",
            "thresholds.csv",
            "conditional-thresholds.csv",
            "events.csv",
            "comparison.csv",
        ]
    ]
    assert len(scores) == len(projections) == len(poses) == 13155 and len(ee) == 3997
    assert (
        len(pooled) == 264
        and sum(t["view"] in SIM_VIEWS for t in pooled) == 72
        and len(conditional) == 216
    )
    events = {e["event_id"]: e for e in ee}
    assert len(events) == len(ee)
    rows, audits = reconstruct(scores, projections, poses)
    components, ranking = summaries(rows, pooled, conditional, events)
    assert len(components) == 17280 and len(ranking) == 2160
    checked = check_previous(components, previous)
    print("Saved cutoff comparisons matched", checked, flush=True)
    args.output.mkdir(parents=True)
    for n, rr in [
        ("reconstruction", audits),
        ("component-summary", components),
        ("ranking", ranking),
    ]:
        save_rows(args.output / (n + ".csv"), rr)
    copied = [
        "accounting.csv",
        "case-support.csv",
        "events.csv",
        "loss-links.csv",
        "thresholds.csv",
        "conditional-thresholds.csv",
    ]
    for n in copied:
        shutil.copyfile(PRIOR / n, args.output / n)
        assert digest(args.output / n) == PINNED[n]
    helpers = dict(prior["analysis"]["helpers_sha256"])
    for n in [
        "analyze_fresh_seed_cutoffs.py",
        "analyze_estimator_mode_components.py",
        "analyze_contrast_risk_transfer.py",
        "analyze_mode_conditional_cutoffs.py",
    ]:
        helpers[n] = digest(ROOT / n)
    audit = dict(
        accounted_cases=972,
        fresh_cases=324,
        fresh_timeouts=202,
        source_pairs=len(rows),
        source_states=len(poses) // 3,
        source_cases=len({identity(r) for r in rows}),
        source_opportunities=len(
            {e for r in rows for e in r["source_event_ids"].split("+")}
        ),
        loss_links=101,
        command_only_links=94,
        component_summary_rows=len(components),
        ranking_rows=len(ranking),
        max_vector_closure_m=max(
            a[t + "_vector_closure_m"] for a in audits for t in TARGETS
        ),
        max_residual_component_difference_m=max(
            a["residual_component_difference_m"] for a in audits
        ),
        **checked
    )
    sources = dict(
        input_sha256=PINNED,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        audit=audit,
        schema=dict(
            targets=TARGETS,
            parts=PARTS,
            component_labels=LABELS,
            energy_fields=ENERGIES,
            strata=STRATUM_NAMES,
            methods=METHODS,
            auc_direction="high score = outside; ties half credit; one-class undefined",
            parent_weights="1/n_case before stratum/retention",
            training_seeds=[269, 270, 271],
            evaluated_seeds=[277, 278, 279],
        ),
        inherited_validation=dict(
            new_rollout=False,
            raw_replay=False,
            prior_source_sha256=PINNED["sources.json"],
            additional_live_held="62->60",
        ),
    )
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(audit), flush=True)


if __name__ == "__main__":
    main()
