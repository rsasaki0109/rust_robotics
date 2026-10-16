#!/usr/bin/env python3
"""Trace fixed additional-seed boundary labels and case/seed AUC influence."""

import argparse
from collections import defaultdict
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path
import shutil

import analyze_fresh_seed_components as previous
from analyze_contrast_risk_transfer import auc, case_weights
from analyze_epoch_preview import digest, read_rows
from analyze_estimator_mode_components import LABELS, component_metrics, exported
from analyze_fresh_seed_modes import identity
from analyze_joint_admission_geometry import KEY
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
PROTOCOL = RESULTS / "fresh-seed-case-influence/protocol.md"
VIEW = "sim_recovery_preview"
SEEDS = ["277", "278", "279"]
STRATA = ["all", *previous.STRATUM_NAMES]
GROUP = ["noise", "variant", "stratum", "target"]
PINNED = {
    **{"fresh-seed-cutoff-transfer/" + n: s for n, s in previous.PINNED.items()},
    "fresh-seed-components/component-summary.csv": "388ef049a18cb482124908d17ad6673996937a7a53bef6c273fbf766fb7d55a3",
    "fresh-seed-components/ranking.csv": "fd9710959d1f46f3426e81651845f0b24c46cff0b4ad4d6d8b9179cc4d90c9b2",
    "fresh-seed-components/sources.json": "73af8d2604eac9aa738c616e71696e32058edd1fc7cc43c84c1b728e37773698",
}


def weighted_histogram(rows, weights, target, score, label):
    assert len(rows) == len(weights) and all(w > 0 for w in weights)
    hist = [defaultdict(Fraction), defaultdict(Fraction)]
    for r, w in zip(rows, weights):
        hist[int(r[target]["outside"][label])][r[score + "_tick"]] += w
    return hist


def pair_masses(negative, positive):
    """Exact ordered positive-negative mass; higher positive score wins."""
    below = concordant = tied = Fraction()
    for tick in sorted(set(negative) | set(positive)):
        p, n = positive.get(tick, Fraction()), negative.get(tick, Fraction())
        concordant += p * below
        tied += p * n
        below += n
    denominator = sum(positive.values(), Fraction()) * sum(
        negative.values(), Fraction()
    )
    numerator = concordant + tied / 2
    return dict(
        concordant_pair_mass=concordant,
        tied_pair_mass=tied,
        discordant_pair_mass=denominator - concordant - tied,
        numerator=numerator,
        denominator=denominator,
        auc=numerator / denominator if denominator else None,
    )


def scopes(rows, weights):
    """Keep original parent weights, even for mixed-mode or single-seed cases."""
    yield "all", "all", rows, weights
    cases = sorted({r["case_id"] for r in rows})
    for scope, names, field, equal in [
        ("only_case", cases, "case_id", True),
        ("omit_case", cases, "case_id", False),
        ("only_seed", SEEDS, "seed", True),
        ("omit_seed", SEEDS, "seed", False),
    ]:
        for name in names:
            subset = [
                (r, w) for r, w in zip(rows, weights) if (r[field] == name) == equal
            ]
            yield scope, name, [r for r, w in subset], [w for r, w in subset]


def pair_blocks(rows, weights, target, score, label):
    cases = sorted({r["case_id"] for r in rows})
    by_case = {}
    for case in cases:
        subset = [(r, w) for r, w in zip(rows, weights) if r["case_id"] == case]
        rr, ww = [r for r, w in subset], [w for r, w in subset]
        hist = weighted_histogram(rr, ww, target, score, label)
        positive_rows = sum(r[target]["outside"][label] for r in rr)
        by_case[case] = (hist, positive_rows, len(rr) - positive_rows)
    blocks = []
    for pos, neg in product(cases, repeat=2):
        pp, positive_rows, _ = by_case[pos]
        nn, _, negative_rows = by_case[neg]
        blocks.append(
            dict(
                positive_case_id=pos,
                negative_case_id=neg,
                same_case=pos == neg,
                positive_rows=positive_rows,
                negative_rows=negative_rows,
                positive_case_mass=sum(pp[1].values(), Fraction()),
                negative_case_mass=sum(nn[0].values(), Fraction()),
                **pair_masses(nn[0], pp[1]),
            )
        )
    return blocks


def support(rows, weights):
    return dict(
        records=len(rows),
        cases=len({r["case_id"] for r in rows}),
        seeds="+".join(sorted({r["seed"] for r in rows})),
        seed_count=len({r["seed"] for r in rows}),
        case_mass=sum(weights, Fraction()),
    )


def analyze(rows):
    case_keys = sorted({identity(r) for r in rows})
    ids = {key: f"C{i:03d}" for i, key in enumerate(case_keys, 1)}
    parents = defaultdict(list)
    for original in rows:
        if original["view"] == VIEW:
            r = dict(original, case_id=ids[identity(original)])
            parents[r["noise"], r["variant"]].append(r)
    cases, components, rankings, blocks = [], [], [], []
    for noise, variant in product(previous.NOISES, previous.ARMS):
        parent = parents[noise, variant]
        parent_weights = case_weights(parent)
        for key in sorted({identity(r) for r in parent}):
            subset = [r for r in parent if identity(r) == key]
            cases.append(
                exported(
                    dict(
                        case_id=ids[key],
                        **dict(zip(KEY, key)),
                        noise=noise,
                        parent_records=len(subset),
                        inherited_row_weight=Fraction(1, len(subset)),
                        **{
                            mode + "_records": sum(r["stratum"] == mode for r in subset)
                            for mode in previous.STRATUM_NAMES
                        },
                    )
                )
            )
        for mode, target in product(STRATA, previous.TARGETS):
            subset = [
                (r, w)
                for r, w in zip(parent, parent_weights)
                if mode == "all" or r["stratum"] == mode
            ]
            rr, ww = [r for r, w in subset], [w for r, w in subset]
            source = support(rr, ww)
            common = dict(
                noise=noise,
                variant=variant,
                stratum=mode,
                target=target,
                **{"source_" + k: v for k, v in source.items()},
                group_case_mass=sum(parent_weights, Fraction()),
            )
            baseline = {}
            for scope, selected, kept, weights in scopes(rr, ww):
                context = dict(
                    **common,
                    scope=scope,
                    selection_id=selected,
                    **support(kept, weights),
                )
                components.append(
                    exported(
                        dict(**context, **component_metrics(kept, weights, target))
                    )
                )
                for score, label in product(previous.SCORES, LABELS):
                    hist = weighted_histogram(kept, weights, target, score, label)
                    masses = pair_masses(*hist)
                    count = sum(r[target]["outside"][label] for r in kept)
                    extra = {}
                    if scope == "all":
                        baseline[score, label] = masses["auc"]
                        pieces = pair_blocks(kept, weights, target, score, label)
                        for field in [
                            "concordant_pair_mass",
                            "tied_pair_mass",
                            "discordant_pair_mass",
                            "numerator",
                            "denominator",
                        ]:
                            assert (
                                sum((p[field] for p in pieces), Fraction())
                                == masses[field]
                            )
                        for same, name in [
                            (True, "same_case"),
                            (False, "between_case"),
                        ]:
                            for field in ["numerator", "denominator"]:
                                extra[name + "_" + field] = sum(
                                    (
                                        p[field]
                                        for p in pieces
                                        if p["same_case"] == same
                                    ),
                                    Fraction(),
                                )
                        for piece in pieces:
                            blocks.append(
                                exported(
                                    dict(
                                        **{k: common[k] for k in GROUP},
                                        score=score,
                                        label=label,
                                        **piece,
                                        numerator_contribution=(
                                            piece["numerator"] / masses["denominator"]
                                            if masses["denominator"]
                                            else None
                                        ),
                                        denominator_share=(
                                            piece["denominator"] / masses["denominator"]
                                            if masses["denominator"]
                                            else None
                                        ),
                                    )
                                )
                            )
                    reference = baseline[score, label]
                    expected = auc(
                        [r[score + "_tick"] for r in kept],
                        [r[target]["outside"][label] for r in kept],
                        weights,
                    )
                    assert expected == (
                        float(masses["auc"]) if masses["auc"] is not None else ""
                    )
                    rankings.append(
                        exported(
                            dict(
                                **context,
                                score=score,
                                label=label,
                                outside_rows=count,
                                inside_rows=len(kept) - count,
                                outside_case_mass=sum(hist[1].values(), Fraction()),
                                inside_case_mass=sum(hist[0].values(), Fraction()),
                                **masses,
                                baseline_auc=reference,
                                auc_difference=(
                                    masses["auc"] - reference
                                    if masses["auc"] is not None
                                    and reference is not None
                                    else None
                                ),
                                **{
                                    k: extra.get(k)
                                    for k in [
                                        "same_case_numerator",
                                        "same_case_denominator",
                                        "between_case_numerator",
                                        "between_case_denominator",
                                    ]
                                },
                            )
                        )
                    )
    return cases, components, rankings, blocks


def boundary_trace(rows, cases, projections):
    case_lookup = {tuple(c[k] for k in KEY): c for c in cases}
    output = []
    for r in rows:
        if r["view"] != VIEW:
            continue
        case = case_lookup[identity(r)]
        for target in previous.TARGETS:
            terms = r[target]
            p = [
                terms["parts"]["noise"][j] + terms["parts"]["anchor"][j]
                for j in range(2)
            ]
            u = [
                terms["parts"]["drift"][j] + terms["parts"]["transport"][j]
                for j in range(2)
            ]
            e = [
                float(projections[r["source_id"]][target + "_" + a]) for a in ["x", "y"]
            ]
            norms = {label: math.hypot(*xy) for label, xy in zip(LABELS, [e, p, u])}
            assert all((norms[k] > 0.1) == terms["outside"][k] for k in LABELS)
            output.append(
                dict(
                    case_id=case["case_id"],
                    **{
                        k: r[k]
                        for k in [
                            *KEY,
                            "noise",
                            "source_id",
                            "state_id",
                            "time_s",
                            "truth_time_s",
                            "source_event_ids",
                            "stratum",
                            "a_estimator",
                            "b_estimator",
                            "anchor_mode",
                        ]
                    },
                    target=target,
                    parent_records=case["parent_records"],
                    inherited_case_weight=case["inherited_row_weight"],
                    **{s + "_tick": r[s + "_tick"] for s in previous.SCORES},
                    **{
                        label + "_" + axis: xy[j]
                        for label, xy in zip(LABELS, [e, p, u])
                        for j, axis in enumerate(["x", "y"])
                    },
                    **{label + "_norm_m": norms[label] for label in LABELS},
                    **{
                        label + "_boundary_margin_m": norms[label] - 0.1
                        for label in LABELS
                    },
                    **{label + "_outside": terms["outside"][label] for label in LABELS},
                    cell=terms["cell"],
                    **{
                        label + "_energy_m2": terms["energy"][label + "_energy_m2"]
                        for label in LABELS
                    },
                    modeled_unmodeled_cross_m2=terms["energy"][
                        "modeled_unmodeled_cross_m2"
                    ],
                )
            )
    return output


def check_previous(components, rankings):
    prior_components = read_rows(
        RESULTS / "fresh-seed-components/component-summary.csv"
    )
    lookup = {
        tuple(r[k] for k in GROUP): r
        for r in prior_components
        if r["view"] == VIEW
        and r["event_kind"] == "all"
        and r["budget"] == "all"
        and r["method"] == "pooled"
        and r["score"] == "residual_sigma"
    }
    checks = 0
    for r in components:
        if r["scope"] != "all":
            continue
        p = lookup[tuple(r[k] for k in GROUP)]
        fields = ["records", "cases", "case_mass", "group_case_mass"]
        fields += list(component_metrics([], [], "residual"))
        for field in fields:
            a, b = r[field], p[field]
            assert (a == b == "") or (
                a != ""
                and b != ""
                and math.isclose(float(a), float(b), rel_tol=1e-11, abs_tol=1e-12)
            ), field
            checks += 1
    prior_rankings = read_rows(RESULTS / "fresh-seed-components/ranking.csv")
    lookup = {
        (*[r[k] for k in GROUP], r["score"]): r
        for r in prior_rankings
        if r["view"] == VIEW and r["event_kind"] == "all"
    }
    for r in rankings:
        if r["scope"] == "all":
            p = lookup[(*[r[k] for k in GROUP], r["score"])]
            for field, old in [
                ("auc", r["label"] + "_case_weighted_auc"),
                ("outside_rows", r["label"] + "_outside_rows"),
                ("outside_case_mass", r["label"] + "_outside_case_mass"),
            ]:
                a, b = r[field], p[old]
                assert (a == b == "") or (
                    a != ""
                    and b != ""
                    and math.isclose(float(a), float(b), rel_tol=1e-11, abs_tol=1e-12)
                ), field
                checks += 1
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for name, sha in PINNED.items():
        assert digest(RESULTS / name) == sha, name
    source = json.loads((RESULTS / "fresh-seed-components/sources.json").read_text())
    helpers = dict(source["analysis"]["helpers_sha256"])
    helpers["analyze_fresh_seed_components.py"] = source["analysis"]["script_sha256"]
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    scores, projections, poses = [
        read_rows(previous.PRIOR / n)
        for n in ["scores.csv", "projections.csv", "poses.csv"]
    ]
    rows, audits = previous.reconstruct(scores, projections, poses)
    assert len(rows) == len(audits) == 13155
    cases, components, rankings, blocks = analyze(rows)
    trace = boundary_trace(rows, cases, {p["source_id"]: p for p in projections})
    assert len(cases) == 57 and len(trace) == 8770
    checks = check_previous(components, rankings)
    args.output.mkdir(parents=True)
    for name, values in [
        ("cases", cases),
        ("boundary-trace", trace),
        ("component-influence", components),
        ("ranking-influence", rankings),
        ("case-pairs", blocks),
    ]:
        save_rows(args.output / (name + ".csv"), values)
    for name in [
        "accounting.csv",
        "case-support.csv",
        "events.csv",
        "loss-links.csv",
        "thresholds.csv",
        "conditional-thresholds.csv",
    ]:
        shutil.copyfile(previous.PRIOR / name, args.output / name)
    audit = dict(
        source_pairs=len(rows),
        source_states=4385,
        source_cases=len(cases),
        source_opportunities=104,
        accounted_cases=972,
        additional_cases=324,
        additional_timeouts=202,
        opportunities=3997,
        loss_links=101,
        command_only_links=94,
        boundary_trace_rows=len(trace),
        component_influence_rows=len(components),
        ranking_influence_rows=len(rankings),
        case_pair_rows=len(blocks),
        previous_field_comparisons=checks,
        baseline_component_rows=sum(r["scope"] == "all" for r in components),
        baseline_ranking_rows=sum(r["scope"] == "all" for r in rankings),
    )
    sources = dict(
        input_sha256=PINNED,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        schema=dict(
            view=VIEW,
            event_kind="all",
            budget="all_available",
            targets=previous.TARGETS,
            scores=previous.SCORES,
            labels=LABELS,
            strata=STRATA,
            seeds=SEEDS,
            parent_weights="original full-parent 1/n_case retained in every subset",
            auc_direction="high score = outside; half ties; absent class undefined",
            omission_interpretation="deterministic influence only; not uncertainty intervals or holdouts",
        ),
        inherited_validation=dict(
            prior_sources_sha256=PINNED["fresh-seed-components/sources.json"],
            new_rollout=False,
            raw_replay=False,
            additional_live_held="62->60",
        ),
        audit=audit,
    )
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
