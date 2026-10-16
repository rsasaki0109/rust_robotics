#!/usr/bin/env python3
"""Separate within-seed and between-seed ranking comparisons without refitting."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path
import shutil

from analyze_contrast_risk_transfer import (
    ARMS,
    NOISES,
    SCORES,
    TARGETS,
    auc,
    case_weights,
)
from analyze_epoch_preview import digest, read_rows
import analyze_estimator_mode_components as historical
import analyze_fresh_seed_case_influence as influence
import analyze_fresh_seed_components as fresh
from analyze_fresh_seed_modes import identity
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
PROTOCOL = RESULTS / "seed-conditioned-rankings/protocol.md"
SEEDS = {
    "known": ["269", "270", "271"],
    "new": ["274", "275", "276"],
    "fresh": ["277", "278", "279"],
}
VIEW = "sim_recovery_preview"
STRATA = ["all", *fresh.STRATUM_NAMES]
GROUP = ["partition", "noise", "variant", "stratum", "target"]
RANK = [*GROUP, "score", "label"]
PAIR_FIELDS = [
    "concordant_pair_mass",
    "tied_pair_mass",
    "discordant_pair_mass",
    "numerator",
    "denominator",
]
PINNED = {
    **historical.PINNED,
    **influence.PINNED,
    "estimator-mode-components/sources.json": "ffc790d6c7722fb72cbe6f0579a454b5721f7e69e01e39a68afc239b9348aa10",
    "estimator-mode-components/component-summary.csv": "fc29821c98ffb1e5a43eb815fc8ab4e03eb1fea66b79f6b20fa9269cf9809565",
    "estimator-mode-components/diagnostic-ranking.csv": "c294aa0adb4b556cfeea7a7057d9f4eed2309f136b76e732e14d02aea0b769d9",
    "fresh-seed-case-influence/sources.json": "68fcb928c5684f873f4e03a4ad4003081a2cdde7e99075842e1c7653520870b9",
    "fresh-seed-case-influence/ranking-influence.csv": "41c89435497d7239e9b6851685058b36228cb540a1c1abb63c837d8682e77214",
    "fresh-seed-case-influence/case-pairs.csv": "382c06c327b9c1f79f3c849a10f0ac2847f90733d9a96bab8b30691df98952e9",
    "fresh-seed-case-influence/cases.csv": "d633c7eeb5851062960b83005f01a9629280927db8079555767bc1cab366bcdd",
}
COPIES = {
    **{
        name: "fresh-seed-cutoff-transfer/" + name
        for name in [
            "accounting.csv",
            "case-support.csv",
            "events.csv",
            "thresholds.csv",
            "conditional-thresholds.csv",
        ]
    },
    "historical-loss-links.csv": "estimator-mode-transfer/loss-links.csv",
    "fresh-loss-links.csv": "fresh-seed-cutoff-transfer/loss-links.csv",
}


def decompose(rows, weights, target, score, label, seeds):
    assert len(rows) == len(weights) and all(w > 0 for w in weights)
    assert len(seeds) == len(set(seeds)) and all(r["seed"] in seeds for r in rows)
    full_hist = influence.weighted_histogram(rows, weights, target, score, label)
    full = influence.pair_masses(*full_hist)
    by_seed = {}
    for seed in seeds:
        subset = [(r, w) for r, w in zip(rows, weights) if r["seed"] == seed]
        rr, ww = [r for r, w in subset], [w for r, w in subset]
        hist = influence.weighted_histogram(rr, ww, target, score, label)
        by_seed[seed] = dict(
            histogram=hist,
            positive_rows=sum(r[target]["outside"][label] for r in rr),
            negative_rows=sum(not r[target]["outside"][label] for r in rr),
            positive_cases=len(
                {identity(r) for r in rr if r[target]["outside"][label]}
            ),
            negative_cases=len(
                {identity(r) for r in rr if not r[target]["outside"][label]}
            ),
        )
    blocks = []
    for pos, neg in product(seeds, repeat=2):
        p, n = by_seed[pos], by_seed[neg]
        mass = influence.pair_masses(n["histogram"][0], p["histogram"][1])
        blocks.append(
            dict(
                positive_seed=pos,
                negative_seed=neg,
                same_seed=pos == neg,
                **{k: p[k] for k in ["positive_rows", "positive_cases"]},
                **{k: n[k] for k in ["negative_rows", "negative_cases"]},
                positive_case_mass=sum(p["histogram"][1].values(), Fraction()),
                negative_case_mass=sum(n["histogram"][0].values(), Fraction()),
                **mass,
                denominator_share=(
                    mass["denominator"] / full["denominator"]
                    if full["denominator"]
                    else None
                ),
                numerator_contribution=(
                    mass["numerator"] / full["denominator"]
                    if full["denominator"]
                    else None
                ),
            )
        )
    for field in PAIR_FIELDS:
        assert sum((b[field] for b in blocks), Fraction()) == full[field]
    result = dict(
        outside_rows=sum(r[target]["outside"][label] for r in rows),
        inside_rows=sum(not r[target]["outside"][label] for r in rows),
        outside_cases=len({identity(r) for r in rows if r[target]["outside"][label]}),
        inside_cases=len(
            {identity(r) for r in rows if not r[target]["outside"][label]}
        ),
        outside_case_mass=sum(full_hist[1].values(), Fraction()),
        inside_case_mass=sum(full_hist[0].values(), Fraction()),
        **full,
    )
    for same, name in [(True, "same_seed"), (False, "between_seed")]:
        num = sum(
            (b["numerator"] for b in blocks if b["same_seed"] == same), Fraction()
        )
        den = sum(
            (b["denominator"] for b in blocks if b["same_seed"] == same), Fraction()
        )
        result[name + "_numerator"] = num
        result[name + "_denominator"] = den
        result[name + "_auc"] = num / den if den else None
        result[name + "_pair_share"] = (
            den / full["denominator"] if full["denominator"] else None
        )
    result["seeds_with_both_classes"] = sum(
        b["denominator"] > 0 for b in blocks if b["same_seed"]
    )
    result["pooled_minus_same_seed_auc"] = (
        full["auc"] - result["same_seed_auc"]
        if full["auc"] is not None and result["same_seed_auc"] is not None
        else None
    )
    expected = auc(
        [r[score + "_tick"] for r in rows],
        [r[target]["outside"][label] for r in rows],
        weights,
    )
    assert expected == (float(full["auc"]) if full["auc"] is not None else "")
    return result, blocks


def support(rows, weights):
    return dict(
        records=len(rows),
        cases=len({identity(r) for r in rows}),
        case_mass=sum(weights, Fraction()),
        seeds="+".join(sorted({r["seed"] for r in rows})),
        seed_count=len({r["seed"] for r in rows}),
    )


def summaries(rows):
    parents = defaultdict(list)
    for r in rows:
        assert r["partition"] in SEEDS and r["seed"] in SEEDS[r["partition"]]
        if r["view"] == VIEW:
            parents[r["partition"], r["noise"], r["variant"]].append(r)
    components, seed_support, ranks, blocks = [], [], [], []
    for partition, noise, variant in product(SEEDS, NOISES, ARMS):
        parent = parents[partition, noise, variant]
        parent_weights = case_weights(parent)
        parent_mass = sum(parent_weights, Fraction())
        for mode, target in product(STRATA, TARGETS):
            subset = [
                (r, w)
                for r, w in zip(parent, parent_weights)
                if mode == "all" or r["stratum"] == mode
            ]
            rr, ww = [r for r, w in subset], [w for r, w in subset]
            common = dict(
                partition=partition,
                noise=noise,
                variant=variant,
                stratum=mode,
                target=target,
                **support(rr, ww),
                group_case_mass=parent_mass,
                case_mass_share=(
                    sum(ww, Fraction()) / parent_mass if parent_mass else None
                ),
            )
            components.append(
                historical.exported(
                    dict(**common, **historical.component_metrics(rr, ww, target))
                )
            )
            for seed in SEEDS[partition]:
                selected = [(r, w) for r, w in zip(rr, ww) if r["seed"] == seed]
                ss, sw = [r for r, w in selected], [w for r, w in selected]
                item = dict(
                    **dict(zip(GROUP, [partition, noise, variant, mode, target])),
                    seed=seed,
                    **support(ss, sw),
                    group_case_mass=parent_mass,
                    stratum_case_mass=sum(ww, Fraction()),
                )
                for label in historical.LABELS:
                    outside = [
                        (r, w) for r, w in selected if r[target]["outside"][label]
                    ]
                    mass = sum((w for r, w in outside), Fraction())
                    item[label + "_outside_rows"] = len(outside)
                    item[label + "_outside_cases"] = len(
                        {identity(r) for r, w in outside}
                    )
                    item[label + "_outside_case_mass"] = mass
                    item[label + "_outside_fraction"] = (
                        mass / sum(sw, Fraction()) if sw else None
                    )
                seed_support.append(historical.exported(item))
            for score, label in product(SCORES, historical.LABELS):
                metrics, pieces = decompose(
                    rr, ww, target, score, label, SEEDS[partition]
                )
                ranks.append(
                    historical.exported(
                        dict(**common, score=score, label=label, **metrics)
                    )
                )
                for piece in pieces:
                    blocks.append(
                        historical.exported(
                            dict(
                                **{k: common[k] for k in GROUP},
                                score=score,
                                label=label,
                                **piece,
                            )
                        )
                    )
    return components, seed_support, ranks, blocks


def close(a, b):
    if a == "" or b == "":
        assert a == b == "", (a, b)
    else:
        assert math.isclose(float(a), float(b), rel_tol=1e-11, abs_tol=1e-12), (a, b)


def check_previous(components, ranks, blocks):
    previous_components = [
        *read_rows(RESULTS / "estimator-mode-components/component-summary.csv"),
        *read_rows(RESULTS / "fresh-seed-components/component-summary.csv"),
    ]
    pc = {
        tuple(r[k] for k in GROUP): r
        for r in previous_components
        if r["view"] == VIEW
        and r["event_kind"] == "all"
        and r["budget"] == "all"
        and r["score"] == "residual_sigma"
        and r.get("method", "pooled") == "pooled"
    }
    comparisons = component_rows = ranking_rows = 0
    for r in components:
        p = pc.get(tuple(r[k] for k in GROUP))
        if p is None:
            assert (
                r["partition"] != "fresh"
                and r["stratum"] == "recovery_fallback_single"
                and r["records"] == 0
            )
            continue
        for field in [
            "records",
            "cases",
            "case_mass",
            "group_case_mass",
            "case_mass_share",
            *historical.component_metrics([], [], "residual"),
        ]:
            close(r[field], p[field])
            comparisons += 1
        component_rows += 1
    previous_ranks = [
        *read_rows(RESULTS / "estimator-mode-components/diagnostic-ranking.csv"),
        *read_rows(RESULTS / "fresh-seed-components/ranking.csv"),
    ]
    pr = {
        tuple(r[k] for k in [*GROUP, "score"]): r
        for r in previous_ranks
        if r["view"] == VIEW and r["event_kind"] == "all"
    }
    for r in ranks:
        p = pr.get(tuple(r[k] for k in [*GROUP, "score"]))
        if p is None:
            assert (
                r["partition"] != "fresh"
                and r["stratum"] == "recovery_fallback_single"
                and r["records"] == 0
                and r["auc"] == ""
            )
            continue
        for field in ["records", "cases", "case_mass", "group_case_mass"]:
            close(r[field], p[field])
            comparisons += 1
        for field, old in [
            ("auc", "case_weighted_auc"),
            ("outside_rows", "outside_rows"),
            ("outside_case_mass", "outside_case_mass"),
        ]:
            close(r[field], p[r["label"] + "_" + old])
            comparisons += 1
        ranking_rows += 1
    assert (component_rows, ranking_rows) == (104, 936)
    cases = {
        r["case_id"]: r
        for r in read_rows(RESULTS / "fresh-seed-case-influence/cases.csv")
    }
    accumulated = defaultdict(lambda: defaultdict(list))
    for r in read_rows(RESULTS / "fresh-seed-case-influence/case-pairs.csv"):
        key = (
            *[r[k] for k in RANK[1:]],
            cases[r["positive_case_id"]]["seed"],
            cases[r["negative_case_id"]]["seed"],
        )
        for field in PAIR_FIELDS:
            accumulated[key][field].append(float(r[field]))
    old_seed = {
        tuple(r[k] for k in RANK[1:]) + (r["selection_id"],): r
        for r in read_rows(RESULTS / "fresh-seed-case-influence/ranking-influence.csv")
        if r["scope"] == "only_seed"
    }
    fresh_blocks = fresh_diagonals = 0
    for r in blocks:
        if r["partition"] != "fresh":
            continue
        key = (*[r[k] for k in RANK[1:]], r["positive_seed"], r["negative_seed"])
        for field in PAIR_FIELDS:
            close(r[field], math.fsum(accumulated[key][field]))
            comparisons += 1
        fresh_blocks += 1
        if r["same_seed"]:
            p = old_seed[tuple(r[k] for k in RANK[1:]) + (r["positive_seed"],)]
            for field, old in [
                ("auc", "auc"),
                ("positive_rows", "outside_rows"),
                ("negative_rows", "inside_rows"),
                ("positive_case_mass", "outside_case_mass"),
                ("negative_case_mass", "inside_case_mass"),
            ]:
                close(r[field], p[old])
                comparisons += 1
            fresh_diagonals += 1
    assert (fresh_blocks, fresh_diagonals) == (3240, 1080)
    return dict(
        previous_component_rows=component_rows,
        previous_ranking_rows=ranking_rows,
        previous_fresh_seed_blocks=fresh_blocks,
        previous_fresh_diagonals=fresh_diagonals,
        previous_field_comparisons=comparisons,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for name, sha in PINNED.items():
        assert digest(RESULTS / name) == sha, name
    helpers = {}
    for directory, script in [
        ("estimator-mode-components", "analyze_estimator_mode_components.py"),
        ("fresh-seed-components", "analyze_fresh_seed_components.py"),
        ("fresh-seed-case-influence", "analyze_fresh_seed_case_influence.py"),
    ]:
        source = json.loads((RESULTS / directory / "sources.json").read_text())
        for name, sha in {
            **source["analysis"]["helpers_sha256"],
            script: source["analysis"]["script_sha256"],
        }.items():
            assert name not in helpers or helpers[name] == sha, name
            assert digest(ROOT / name) == sha, name
            helpers[name] = sha
    old_rows, old_audits = historical.reconstruct(
        *[
            read_rows(RESULTS / name)
            for name in [
                "estimator-mode-transfer/membership.csv",
                "contrast-common-errors/simultaneous.csv",
                "speed-probe-error-components/components.csv",
                "contrast-risk-transfer/scores.csv",
            ]
        ]
    )
    fresh_rows, fresh_audits = fresh.reconstruct(
        *[
            read_rows(fresh.PRIOR / name)
            for name in ["scores.csv", "projections.csv", "poses.csv"]
        ]
    )
    assert (len(old_rows), len(fresh_rows)) == (19830, 13155)
    rows = [*old_rows, *fresh_rows]
    events_list = read_rows(RESULTS / COPIES["events.csv"])
    events = {r["event_id"]: r for r in events_list}
    assert len(events) == len(events_list) == 3997
    for r in rows:
        for event_id in r["source_event_ids"].split("+"):
            assert identity(events[event_id]) == identity(r)
    components, seed_support, ranks, blocks = summaries(rows)
    assert (len(components), len(seed_support), len(ranks), len(blocks)) == (
        120,
        360,
        1080,
        9720,
    )
    checked = check_previous(components, ranks, blocks)
    all_audits = [*old_audits, *fresh_audits]
    audit = dict(
        source_pairs=len(rows),
        source_states=len(rows) // 3,
        source_cases=len({identity(r) for r in rows}),
        source_opportunities=len(
            {e for r in rows for e in r["source_event_ids"].split("+")}
        ),
        accounted_cases=972,
        timeout_cases=570,
        opportunities=len(events),
        historical_loss_links=550,
        historical_command_only_links=509,
        fresh_loss_links=101,
        fresh_command_only_links=94,
        component_rows=len(components),
        seed_support_rows=len(seed_support),
        ranking_rows=len(ranks),
        seed_block_rows=len(blocks),
        max_vector_closure_m=max(
            r[t + "_vector_closure_m"] for r in all_audits for t in TARGETS
        ),
        max_residual_component_difference_m=max(
            r["residual_component_difference_m"] for r in all_audits
        ),
        **checked,
    )
    assert (audit["source_cases"], audit["source_opportunities"]) == (157, 275)
    args.output.mkdir(parents=True)
    for name, rr in [
        ("component-summary", components),
        ("seed-support", seed_support),
        ("ranking-summary", ranks),
        ("seed-blocks", blocks),
    ]:
        save_rows(args.output / (name + ".csv"), rr)
    for name, source in COPIES.items():
        shutil.copyfile(RESULTS / source, args.output / name)
        assert digest(args.output / name) == PINNED[source]
    sources = dict(
        input_sha256=PINNED,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        schema=dict(
            seeds=SEEDS,
            view=VIEW,
            event_kind="all",
            budget="all_available",
            targets=TARGETS,
            scores=SCORES,
            labels=historical.LABELS,
            strata=STRATA,
            weighting="full-parent 1/n_case before mode/seed conditioning; no seed normalization",
            direction="high score = outside; half ties; zero comparison mass undefined",
            same_seed_auc="sum diagonal numerators / sum diagonal denominators; not mean seed AUC",
            retrospective=True,
        ),
        inherited_validation=dict(
            new_rollout=False,
            raw_replay=False,
            historical_live_held="140->140",
            latest_live_held="62->60",
        ),
        audit=audit,
    )
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
