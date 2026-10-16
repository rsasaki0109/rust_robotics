#!/usr/bin/env python3
"""Whole-known-seed exclusion checks for pooled and conditional cutoff rules."""

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

from analyze_commanded_recovery_arc import flag
from analyze_contrast_risk_transfer import fit_thresholds, membership
from analyze_epoch_preview import digest, read_rows
from analyze_mode_conditional_cutoffs import (
    ARMS,
    BUDGETS,
    GROUP,
    KEY,
    METHODS,
    NOISES,
    PARTITIONS,
    SCORES,
    SIM_VIEWS,
    STRATA,
    TARGETS,
    exported,
    fit_mode_thresholds,
    scored_members,
    summaries,
)
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
PRIOR = RESULTS / "mode-conditional-cutoffs"
PROTOCOL = RESULTS / "seed-exclusion-cutoffs/protocol.md"
PINNED = {
    "mode-conditional-cutoffs/accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "mode-conditional-cutoffs/events.csv": "819b026c74884681e8df9fb282998a19ade0d12f82e8d1ffa3ad1a3b5ce1df6f",
    "mode-conditional-cutoffs/loss-links.csv": "b25ba5f8fe9f6cafbe1bc20a32b318455804adf3dc1639f10d335607fea7222b",
    "mode-conditional-cutoffs/membership.csv": "e2924bd538b435b26ffab25e4bdeba7d93119e12355f6fed2f1d5371eac4f79c",
    "mode-conditional-cutoffs/thresholds.csv": "e877f83cb96aaf14c2a273c06416eb41998237fed8ffdfac0529b729a5c70207",
    "mode-conditional-cutoffs/conditional-thresholds.csv": "4d20ddb96399552a932aa4c0db904b7e6c085291863b9273e451c5692410c040",
    "mode-conditional-cutoffs/comparison.csv": "4dd9971f43171627831f07be71b25cd24bd52b24ee4d41c14a314af35a84ac01",
    "mode-conditional-cutoffs/sources.json": "869d66d5459f39fd673901dbe90a891a40d0f58a19d18c9b7775bcdaf2b45103",
    "contrast-risk-transfer/scores.csv": "96e963f962a138f066f01d582d34e1a33a5d315de693f160fa37b3a3beca52e6",
    "contrast-risk-transfer/sources.json": "e69444ae8bd9f645aeb96e1caf1f979ea855ff3650ba5d3be87ed3907bbdc331",
}
SEEDS = sorted(PARTITIONS["known"])
ROLES = ["train_known", "held_known", "existing_new"]
REPORT_KEY = [*GROUP, "stratum", "score", "budget"]
POPULATION = ["records", "cases", "case_mass", "group_case_mass", "case_mass_share"]
METHOD_FIELDS = [
    "retained_rows",
    "retained_cases",
    "retained_case_mass",
    "row_coverage",
    "case_weighted_coverage",
    *[
        t + s
        for t in TARGETS
        for s in [
            "_error_rows",
            "_error_case_mass",
            "_case_weighted_risk",
            "_row_risk",
            "_case_weighted_rms",
        ]
    ],
]
METRICS = [
    m + "_" + s
    for m in METHODS
    for s in ["case_weighted_coverage", *[t + "_case_weighted_risk" for t in TARGETS]]
]
DIFFERENCES = [t + "_case_weighted_risk_delta" for t in TARGETS]
REFERENCE_FIELDS = [
    *REPORT_KEY,
    *POPULATION,
    "conditional_support",
    "conditional_unsupported_rows",
    "conditional_unsupported_case_mass",
    "conditional_unsupported_strata",
    *[m + "_" + s for m in METHODS for s in METHOD_FIELDS],
    *DIFFERENCES,
]


def fold_info(seed):
    assert seed in SEEDS
    return dict(
        fold_id="omit_" + str(seed),
        held_seed=seed,
        fold_training_seeds="+".join(str(s) for s in SEEDS if s != seed),
    )


def role(r, seed):
    fold_info(seed)
    partition, _ = membership(r)
    return (
        "existing_new"
        if partition == "new"
        else "held_known" if int(r["seed"]) == seed else "train_known"
    )


def case_roles(accounting, rows):
    cases = {tuple(a[k] for k in KEY): a for a in accounting}
    assert len(cases) == len(accounting)
    source = Counter(tuple(r[k] for k in KEY) for r in rows)
    assert set(source) <= set(cases)
    result, folds = [], []
    for seed in SEEDS:
        by_role = defaultdict(list)
        for key, a in cases.items():
            assigned = role(a, seed)
            by_role[assigned].append(key)
            result.append(
                dict(
                    **fold_info(seed),
                    **{k: a[k] for k in KEY},
                    role=assigned,
                    selected=flag(a["selected"]),
                    simultaneous_pairs=source[key]
                )
            )
        info = fold_info(seed)
        for label in ROLES:
            kk = by_role[label]
            info[label + "_account_cases"] = len(kk)
            info[label + "_timeout_cases"] = sum(flag(cases[k]["selected"]) for k in kk)
            info[label + "_source_cases"] = sum(source[k] > 0 for k in kk)
            info[label + "_source_pairs"] = sum(source[k] for k in kk)
        assert sum(info[r + "_account_cases"] for r in ROLES) == len(accounting)
        assert sum(info[r + "_source_pairs"] for r in ROLES) == len(rows)
        folds.append(info)
    held = Counter(
        tuple(r[k] for k in KEY) for r in result if r["role"] == "held_known"
    )
    assert held == Counter(
        {k: 1 for k, a in cases.items() if a["cohort"].startswith("known-")}
    )
    return result, folds


def split_fold(rows, seed):
    train = [r for r in rows if role(r, seed) == "train_known"]
    evaluation = [r for r in rows if role(r, seed) != "train_known"]
    assert {int(r["seed"]) for r in train} == set(SEEDS) - {seed}
    a = {tuple(r[k] for k in KEY) for r in train}
    b = {tuple(r[k] for k in KEY) for r in evaluation}
    assert not a & b and a | b == {tuple(r[k] for k in KEY) for r in rows}
    parents = {(r["noise"], r["view"]) for r in train}
    required = {(r["noise"], r["view"]) for r in evaluation}
    assert required <= parents, (
        "Pooled evaluation group has no training population",
        required - parents,
    )
    return train, evaluation


def fit_fold(rows, seed):
    train, evaluation = split_fold(rows, seed)
    pooled = [t for t in fit_thresholds(train) if t["view"] in SIM_VIEWS]
    conditional = fit_mode_thresholds(train)
    for t in [*pooled, *conditional]:
        actual = {int(s) for s in t["training_seeds"].split("+") if s}
        assert actual <= set(SEEDS) - {seed}
    return pooled, conditional, evaluation


def assert_csv_equal(rows, saved):
    assert len(rows) == len(saved)
    for r, s in zip(rows, saved):
        assert {k: str(v) for k, v in r.items()} == s, (r, s)


def new_reference(comparisons):
    return [
        {k: r[k] for k in REFERENCE_FIELDS}
        for r in comparisons
        if r["partition"] == "new"
    ]


def value(x):
    if x in [None, ""]:
        return None
    assert math.isfinite(float(x))
    return float(x)


def range_fields(prefix, values):
    valid = [value(v) for v in values if value(v) is not None]
    complete = len(valid) == len(SEEDS)
    return {
        prefix + "_defined_folds": len(valid),
        prefix + "_undefined_folds": len(SEEDS) - len(valid),
        prefix + "_min": min(valid) if complete else None,
        prefix + "_max": max(valid) if complete else None,
        prefix + "_span": max(valid) - min(valid) if complete else None,
    }


def stability(reference, fold_rows):
    groups = defaultdict(dict)
    for r in fold_rows:
        if r["evaluation"] != "existing_new":
            continue
        key = tuple(r[k] for k in REPORT_KEY)
        seed = int(r["held_seed"])
        assert seed in SEEDS and seed not in groups[key]
        assert r["fold_id"] == fold_info(seed)["fold_id"]
        groups[key][seed] = r
    assert set(groups) == {tuple(r[k] for k in REPORT_KEY) for r in reference}
    result = []
    for ref in reference:
        assert ref["partition"] == "new"
        key = tuple(ref[k] for k in REPORT_KEY)
        assert set(groups[key]) == set(SEEDS)
        rr = [groups[key][s] for s in SEEDS]
        for r in rr:
            assert all(r[k] == ref[k] for k in POPULATION), (r, ref)
        row = {k: ref[k] for k in [*REPORT_KEY, *POPULATION]}
        row.update(
            conditional_supported_folds=sum(
                r["conditional_support"] == "complete" for r in rr
            ),
            conditional_unsupported_folds=sum(
                r["conditional_support"] == "unsupported" for r in rr
            ),
            empty_population_folds=sum(not r["records"] for r in rr),
            conditional_unsupported_rows_max=max(
                r["conditional_unsupported_rows"] for r in rr
            ),
        )
        for field in [*METRICS, *DIFFERENCES]:
            row["full_fit_" + field] = ref[field]
            row.update(range_fields(field, [r[field] for r in rr]))
            if field in DIFFERENCES:
                valid = [value(r[field]) for r in rr if value(r[field]) is not None]
                row[field + "_higher_folds"] = sum(v > 0 for v in valid)
                row[field + "_equal_folds"] = sum(v == 0 for v in valid)
                row[field + "_lower_folds"] = sum(v < 0 for v in valid)
        result.append(exported(row))
    return result


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
    helpers["analyze_mode_conditional_cutoffs.py"] = prior["analysis"]["script_sha256"]
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    (
        members,
        scores,
        ee,
        accounting,
        links,
        old_pooled,
        old_conditional,
        old_comparison,
    ) = [
        read_rows(p)
        for p in [
            PRIOR / "membership.csv",
            RESULTS / "contrast-risk-transfer/scores.csv",
            PRIOR / "events.csv",
            PRIOR / "accounting.csv",
            PRIOR / "loss-links.csv",
            PRIOR / "thresholds.csv",
            PRIOR / "conditional-thresholds.csv",
            PRIOR / "comparison.csv",
        ]
    ]
    assert (len(members), len(scores), len(ee), len(accounting), len(links)) == (
        19830,
        40494,
        2583,
        648,
        550,
    )
    assert sum(flag(r["selected"]) for r in accounting) == 368
    assert sum(flag(r["command_only_zero_loss"]) for r in links) == 509
    events = {e["event_id"]: e for e in ee}
    rows = scored_members(members, scores, events)
    roles, folds = case_roles(accounting, rows)
    pooled_full = [t for t in fit_thresholds(rows) if t["view"] in SIM_VIEWS]
    conditional_full = fit_mode_thresholds(rows)
    assert_csv_equal(pooled_full, [t for t in old_pooled if t["view"] in SIM_VIEWS])
    assert_csv_equal([exported(t) for t in conditional_full], old_conditional)
    full = summaries(rows, pooled_full, conditional_full, events)
    assert_csv_equal(full, old_comparison)
    reference = new_reference(full)
    pooled_rows, conditional_rows, comparisons = [], [], []
    for seed in SEEDS:
        pooled, conditional, evaluation = fit_fold(rows, seed)
        info = fold_info(seed)
        pooled_rows.extend(exported(dict(**info, **t)) for t in pooled)
        conditional_rows.extend(exported(dict(**info, **t)) for t in conditional)
        cc = summaries(evaluation, pooled, conditional, events)
        comparisons.extend(
            dict(
                **info,
                evaluation=(
                    "held_known" if r["partition"] == "known" else "existing_new"
                ),
                **r
            )
            for r in cc
        )
    stable = stability(reference, comparisons)
    assert (
        len(roles),
        len(folds),
        len(pooled_rows),
        len(conditional_rows),
        len(comparisons),
        len(reference),
        len(stable),
    ) == (1944, 3, 216, 648, 20736, 3456, 3456)
    audit = dict(
        accounted_cases=648,
        timeout_cases=368,
        opportunities=2583,
        simultaneous_references=19830,
        simultaneous_cases=100,
        source_opportunities=171,
        inherited_lagged_requested=20664,
        inherited_lagged_unavailable=2208,
        raw_zero_loss_links=550,
        command_only_links=509,
        folds=len(folds),
        case_role_rows=len(roles),
        pooled_fold_thresholds=len(pooled_rows),
        conditional_fold_thresholds=len(conditional_rows),
        fold_comparisons=len(comparisons),
        existing_new_reference_rows=len(reference),
        stability_rows=len(stable),
        full_fit_thresholds_checked=len(pooled_full) + len(conditional_full),
        full_fit_comparisons_checked=len(full),
        unsupported_fold_comparisons=sum(
            r["conditional_support"] == "unsupported" for r in comparisons
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
            omitted_known_seeds=SEEDS,
            training="remaining two known seeds, all arms/noises/maps/planners/times",
            evaluations=["held_known", "existing_new"],
            strata=["all", *STRATA],
            views=SIM_VIEWS,
            scores=SCORES,
            budgets=list(BUDGETS),
            targets=TARGETS,
            range_requires_all_folds=True,
            retrospective=True,
            evaluation_previously_observed=True,
        ),
        inherited_validation=dict(
            source_sha256=PINNED["mode-conditional-cutoffs/sources.json"],
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
        "membership.csv",
        "thresholds.csv",
    ]:
        (args.output / name).write_bytes((PRIOR / name).read_bytes())
        assert digest(args.output / name) == PINNED["mode-conditional-cutoffs/" + name]
    for name, data in [
        ("folds", folds),
        ("case-roles", roles),
        ("pooled-fold-thresholds", pooled_rows),
        ("conditional-fold-thresholds", conditional_rows),
        ("fold-comparison", comparisons),
        ("reference", reference),
        ("stability", stable),
    ]:
        save_rows(args.output / (name + ".csv"), data)
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
