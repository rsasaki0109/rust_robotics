#!/usr/bin/env python3
"""Compare existing pooled cutoffs with known-only source-mode quantiles."""

import argparse
from collections import defaultdict
from fractions import Fraction
import hashlib
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
    QUANTUM,
    SCORES,
    SIM_VIEWS,
    TARGETS,
    case_weights,
    membership,
    score_tick,
    weighted_quantile,
)
from analyze_epoch_preview import digest, read_rows
from analyze_estimator_mode_transfer import GROUP, STRATA
from analyze_joint_admission_geometry import KEY
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
PRIOR = RESULTS / "estimator-mode-components"
PROTOCOL = RESULTS / "mode-conditional-cutoffs/protocol.md"
PINNED = {
    "estimator-mode-components/accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "estimator-mode-components/events.csv": "819b026c74884681e8df9fb282998a19ade0d12f82e8d1ffa3ad1a3b5ce1df6f",
    "estimator-mode-components/loss-links.csv": "b25ba5f8fe9f6cafbe1bc20a32b318455804adf3dc1639f10d335607fea7222b",
    "estimator-mode-components/membership.csv": "e2924bd538b435b26ffab25e4bdeba7d93119e12355f6fed2f1d5371eac4f79c",
    "estimator-mode-components/thresholds.csv": "e877f83cb96aaf14c2a273c06416eb41998237fed8ffdfac0529b729a5c70207",
    "estimator-mode-components/component-summary.csv": "fc29821c98ffb1e5a43eb815fc8ab4e03eb1fea66b79f6b20fa9269cf9809565",
    "estimator-mode-components/sources.json": "ffc790d6c7722fb72cbe6f0579a454b5721f7e69e01e39a68afc239b9348aa10",
    "contrast-risk-transfer/scores.csv": "96e963f962a138f066f01d582d34e1a33a5d315de693f160fa37b3a3beca52e6",
    "contrast-risk-transfer/sources.json": "e69444ae8bd9f645aeb96e1caf1f979ea855ff3650ba5d3be87ed3907bbdc331",
}
METHODS = ["pooled", "conditional"]
OVERLAP = {
    "both": (True, True),
    "pooled_only": (True, False),
    "conditional_only": (False, True),
    "neither": (False, False),
}


def scored_members(members, scores, events):
    lookup = {(r["source_family"], r["source_id"]): r for r in scores}
    assert len(lookup) == len(scores)
    assert len({m["source_id"] for m in members}) == len(members)
    assert {m["source_id"] for m in members} == {
        sid for family, sid in lookup if family == "simultaneous"
    }
    result = []
    for m in members:
        assert m["source_family"] == "simultaneous" and m["stratum"] in STRATA
        s = lookup["simultaneous", m["source_id"]]
        assert all(
            m[k] == s[k]
            for k in [
                *KEY,
                "partition",
                "noise",
                "view",
                "source_event_ids",
                "a_id",
                "b_id",
            ]
        )
        assert membership(m) == (m["partition"], m["noise"])
        assert (m["b_estimator"], m["anchor_mode"]) == STRATA[m["stratum"]]
        assert (m["stratum"] == "recovery_single") == (m["view"] == "sim_raw_recovery")
        assert m["view"] in SIM_VIEWS and flag(m["available"]) and flag(s["available"])
        for eid in m["source_event_ids"].split("+"):
            assert all(events[eid][k] == m[k] for k in KEY)
        r = dict(s, **m)
        r["available"] = True
        for score in SCORES:
            r[score + "_tick"] = int(s[score + "_tick"])
            assert r[score + "_tick"] == score_tick(float(s[score + "_score_m"]))
        for target in TARGETS:
            error = float(s[target + "_error_m"])
            assert math.isfinite(error) and error >= 0
            r[target + "_error_m"] = error
            r[target + "_outside"] = flag(s[target + "_outside"])
            assert r[target + "_outside"] == (error > 0.1)
        result.append(r)
    return result


def references(rows, weights=None):
    records = sorted(
        zip(rows, weights or [None] * len(rows)),
        key=lambda rw: (rw[0]["source_family"], int(rw[0]["source_id"])),
    )
    text = "".join(
        f'{r["source_family"]}:{r["source_id"]}'
        + (f":{w.numerator}/{w.denominator}" if w is not None else "")
        + "\n"
        for r, w in records
    )
    return hashlib.sha256(text.encode()).hexdigest()


def fit_mode_thresholds(rows):
    groups = defaultdict(list)
    for r in rows:
        if r["partition"] == "known" and r["available"]:
            assert membership(r) == ("known", r["noise"])
            groups[r["noise"], r["view"]].append(r)
    result = []
    for noise, view in product(NOISES, SIM_VIEWS):
        parent = groups[noise, view]
        weights = case_weights(parent)
        parent_mass = sum(weights, Fraction())
        for stratum in STRATA:
            sub = [(r, w) for r, w in zip(parent, weights) if r["stratum"] == stratum]
            train, ww = [r for r, w in sub], [w for r, w in sub]
            mass = sum(ww, Fraction())
            for score, budget in product(SCORES, BUDGETS):
                q = BUDGETS[budget]
                rule = (
                    "all_available"
                    if q is None
                    else "fitted" if train else "no_training_data"
                )
                tick = (
                    weighted_quantile([r[score + "_tick"] for r in train], ww, q)
                    if rule == "fitted"
                    else None
                )
                below = (
                    sum((w for r, w in sub if r[score + "_tick"] < tick), Fraction())
                    if tick is not None
                    else None
                )
                tied = (
                    sum((w for r, w in sub if r[score + "_tick"] == tick), Fraction())
                    if tick is not None
                    else None
                )
                retained = (
                    mass
                    if rule == "all_available"
                    else below + tied if tick is not None else Fraction()
                )
                retained_rows = (
                    len(train)
                    if rule == "all_available"
                    else (
                        sum(r[score + "_tick"] <= tick for r in train)
                        if tick is not None
                        else 0
                    )
                )
                result.append(
                    dict(
                        conditional_threshold_id=len(result) + 1,
                        noise=noise,
                        view=view,
                        stratum=stratum,
                        score=score,
                        budget=budget,
                        training_partition="known",
                        training_parent_rows=len(parent),
                        training_parent_cases=len(
                            {tuple(r[k] for k in KEY) for r in parent}
                        ),
                        training_parent_case_mass=parent_mass,
                        training_rows=len(train),
                        training_cases=len({tuple(r[k] for k in KEY) for r in train}),
                        training_case_mass=mass,
                        training_seeds="+".join(
                            sorted({r["seed"] for r in train}, key=int)
                        ),
                        training_parent_reference_sha256=references(parent),
                        training_reference_sha256=references(train),
                        training_weighted_reference_sha256=references(train, ww),
                        rule=rule,
                        quantile=q,
                        threshold_tick=tick,
                        threshold_m=tick * QUANTUM if tick is not None else None,
                        training_mass_below=below,
                        training_mass_at=tied,
                        training_retained_rows=retained_rows,
                        training_retained_case_mass=retained,
                        training_achieved_row_fraction=(
                            retained_rows / len(train) if train else None
                        ),
                        training_achieved_case_fraction=(
                            retained / mass if mass else None
                        ),
                    )
                )
    return result


def decision(r, score, threshold):
    if threshold["rule"] == "all_available":
        return True
    if threshold["rule"] == "no_training_data":
        return None
    assert threshold["rule"] == "fitted"
    return r[score + "_tick"] <= int(threshold["threshold_tick"])


def method_metrics(rows, weights, flags):
    assert len(rows) == len(weights) == len(flags) and all(
        f in [True, False, None] for f in flags
    )
    fields = [
        "retained_rows",
        "retained_cases",
        "retained_case_mass",
        "row_coverage",
        "case_weighted_coverage",
    ]
    fields += [
        t + suffix
        for t in TARGETS
        for suffix in [
            "_error_rows",
            "_error_case_mass",
            "_case_weighted_risk",
            "_row_risk",
            "_case_weighted_rms",
        ]
    ]
    if any(f is None for f in flags):
        return dict.fromkeys(fields)
    kept = [(r, w) for r, w, f in zip(rows, weights, flags) if f]
    mass, total = sum((w for r, w in kept), Fraction()), sum(weights, Fraction())
    result = dict(
        retained_rows=len(kept),
        retained_cases=len({tuple(r[k] for k in KEY) for r, w in kept}),
        retained_case_mass=mass,
        row_coverage=len(kept) / len(rows) if rows else None,
        case_weighted_coverage=mass / total if total else None,
    )
    for target in TARGETS:
        bad = sum((w for r, w in kept if r[target + "_outside"]), Fraction())
        count = sum(r[target + "_outside"] for r, w in kept)
        result.update(
            {
                target + "_error_rows": count,
                target + "_error_case_mass": bad,
                target + "_case_weighted_risk": bad / mass if mass else None,
                target + "_row_risk": count / len(kept) if kept else None,
                target
                + "_case_weighted_rms": (
                    math.sqrt(
                        math.fsum(
                            float(w / mass) * r[target + "_error_m"] ** 2
                            for r, w in kept
                        )
                    )
                    if mass
                    else None
                ),
            }
        )
    return result


def overlap_metrics(rows, weights, pooled, conditional):
    result = {}
    complete = not any(f is None for f in conditional)
    for label, pair in OVERLAP.items():
        flags = (
            [p == pair[0] and c == pair[1] for p, c in zip(pooled, conditional)]
            if complete
            else [None] * len(rows)
        )
        stats = method_metrics(rows, weights, flags)
        for new, old in [
            ("rows", "retained_rows"),
            ("cases", "retained_cases"),
            ("case_mass", "retained_case_mass"),
        ]:
            result[label + "_" + new] = stats[old]
        for target in TARGETS:
            for suffix in ["_error_rows", "_error_case_mass", "_case_weighted_risk"]:
                result[label + "_" + target + suffix] = stats[target + suffix]
    if complete:
        assert sum(result[label + "_rows"] for label in OVERLAP) == len(rows)
        assert sum(result[label + "_case_mass"] for label in OVERLAP) == sum(
            weights, Fraction()
        )
    return result


def difference(a, b):
    return a - b if a is not None and b is not None else None


def exported(row):
    return {
        k: "" if v is None else float(v) if isinstance(v, Fraction) else v
        for k, v in row.items()
    }


def summaries(rows, pooled_thresholds, conditional_thresholds, events):
    groups = defaultdict(list)
    for r in rows:
        kinds = {
            "all",
            *[events[e]["event_kind"] for e in r["source_event_ids"].split("+")],
        }
        for kind in kinds:
            groups[r["partition"], r["noise"], r["variant"], kind, r["view"]].append(r)
    pcuts = {
        (t["noise"], t["view"], t["score"], t["budget"]): t for t in pooled_thresholds
    }
    ccuts = {
        (t["noise"], t["view"], t["stratum"], t["score"], t["budget"]): t
        for t in conditional_thresholds
    }
    result = []
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
            sr, sw = [r for r, w in sub], [w for r, w in sub]
            mass = sum(sw, Fraction())
            common = dict(zip(GROUP, key))
            common.update(
                stratum=stratum,
                records=len(sr),
                cases=len({tuple(r[k] for k in KEY) for r in sr}),
                case_mass=mass,
                group_case_mass=group_mass,
                case_mass_share=mass / group_mass if group_mass else None,
            )
            for score, budget in product(SCORES, BUDGETS):
                pt = pcuts[key[1], key[4], score, budget]
                pflags = [decision(r, score, pt) for r in sr]
                assert all(f is not None for f in pflags)
                cflags = [
                    decision(
                        r, score, ccuts[key[1], key[4], r["stratum"], score, budget]
                    )
                    for r in sr
                ]
                unsupported = [(r, w) for r, w, f in zip(sr, sw, cflags) if f is None]
                stats = {
                    "pooled": method_metrics(sr, sw, pflags),
                    "conditional": method_metrics(sr, sw, cflags),
                }
                overlap = overlap_metrics(sr, sw, pflags, cflags)
                record = dict(
                    **common,
                    score=score,
                    budget=budget,
                    pooled_threshold_id=pt["threshold_id"],
                    pooled_rule=pt["rule"],
                    conditional_threshold_ids="+".join(
                        str(
                            ccuts[key[1], key[4], s, score, budget][
                                "conditional_threshold_id"
                            ]
                        )
                        for s in (STRATA if stratum == "all" else [stratum])
                    ),
                    conditional_support=(
                        "unsupported" if unsupported else "complete" if sr else "empty"
                    ),
                    conditional_unsupported_rows=len(unsupported),
                    conditional_unsupported_case_mass=sum(
                        (w for r, w in unsupported), Fraction()
                    ),
                    conditional_unsupported_strata="+".join(
                        s
                        for s in STRATA
                        if any(r["stratum"] == s for r, w in unsupported)
                    ),
                )
                for method, mm in stats.items():
                    record.update({method + "_" + k: v for k, v in mm.items()})
                record.update(overlap)
                for field in [
                    "retained_rows",
                    "retained_case_mass",
                    "case_weighted_coverage",
                    *[
                        t + s
                        for t in TARGETS
                        for s in ["_error_case_mass", "_case_weighted_risk"]
                    ],
                ]:
                    record[field + "_delta"] = difference(
                        stats["conditional"][field], stats["pooled"][field]
                    )
                if not unsupported:
                    for method, only in [
                        ("pooled", "pooled_only"),
                        ("conditional", "conditional_only"),
                    ]:
                        assert (
                            stats[method]["retained_case_mass"]
                            == overlap["both_case_mass"] + overlap[only + "_case_mass"]
                        )
                        for target in TARGETS:
                            assert (
                                stats[method][target + "_error_case_mass"]
                                == overlap["both_" + target + "_error_case_mass"]
                                + overlap[only + "_" + target + "_error_case_mass"]
                            )
                    assert (
                        record["retained_case_mass_delta"]
                        == overlap["conditional_only_case_mass"]
                        - overlap["pooled_only_case_mass"]
                    )
                    for target in TARGETS:
                        assert (
                            record[target + "_error_case_mass_delta"]
                            == overlap[
                                "conditional_only_" + target + "_error_case_mass"
                            ]
                            - overlap["pooled_only_" + target + "_error_case_mass"]
                        )
                    if budget == "all" or key[4] == "sim_raw_recovery":
                        assert pflags == cflags
                result.append(exported(record))
    return result


def check_prior(comparisons, prior):
    fields = [*GROUP, "stratum", "score", "budget"]
    current = {tuple(r[k] for k in fields): r for r in comparisons}
    assert len(current) == len(comparisons)
    for p in prior:
        r = current[tuple(p[k] for k in fields)]
        pairs = [
            (k, k)
            for k in [
                "records",
                "cases",
                "case_mass",
                "group_case_mass",
                "case_mass_share",
            ]
        ]
        pairs += [
            ("pooled_" + k, k)
            for k in [
                "retained_rows",
                "retained_cases",
                "retained_case_mass",
                "case_weighted_coverage",
            ]
        ]
        pairs += [
            ("pooled_" + p["target"] + a, b)
            for a, b in [
                ("_error_rows", "observed_outside_rows"),
                ("_error_case_mass", "observed_outside_case_mass"),
                ("_case_weighted_risk", "observed_outside_fraction"),
                ("_case_weighted_rms", "observed_rms_m"),
            ]
        ]
        for a, b in pairs:
            if r[a] == "" or p[b] == "":
                assert r[a] == p[b] == "", (a, p)
            else:
                assert math.isclose(
                    float(r[a]), float(p[b]), rel_tol=1e-11, abs_tol=1e-12
                ), (a, p)
    assert len(prior) == 2 * len(comparisons)
    return len(prior)


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
    helpers["analyze_estimator_mode_components.py"] = prior["analysis"]["script_sha256"]
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    members, scores, pooled, ee, accounting, links = [
        read_rows(p)
        for p in [
            PRIOR / "membership.csv",
            RESULTS / "contrast-risk-transfer/scores.csv",
            PRIOR / "thresholds.csv",
            PRIOR / "events.csv",
            PRIOR / "accounting.csv",
            PRIOR / "loss-links.csv",
        ]
    ]
    assert (
        len(members),
        len(scores),
        len(pooled),
        len(ee),
        len(accounting),
        len(links),
    ) == (19830, 40494, 264, 2583, 648, 550)
    assert (
        sum(flag(r["selected"]) for r in accounting) == 368
        and sum(flag(r["command_only_zero_loss"]) for r in links) == 509
    )
    events = {e["event_id"]: e for e in ee}
    rows = scored_members(members, scores, events)
    conditional = fit_mode_thresholds(rows)
    comparisons = summaries(rows, pooled, conditional, events)
    checked = check_prior(comparisons, read_rows(PRIOR / "component-summary.csv"))
    assert (len(conditional), len(comparisons), checked) == (216, 6912, 13824)
    audit = dict(
        accounted_cases=648,
        timeout_cases=368,
        opportunities=2583,
        simultaneous_references=len(rows),
        simultaneous_cases=100,
        source_opportunities=171,
        inherited_lagged_requested=20664,
        inherited_lagged_unavailable=2208,
        raw_zero_loss_links=550,
        command_only_links=509,
        unchanged_pooled_thresholds=len(pooled),
        conditional_thresholds=len(conditional),
        empty_quantile_fits=sum(t["rule"] == "no_training_data" for t in conditional),
        comparison_rows=len(comparisons),
        prior_target_rows_checked=checked,
        unsupported_comparisons=sum(
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
            partitions={k: sorted(v) for k, v in PARTITIONS.items()},
            views=SIM_VIEWS,
            strata=STRATA,
            scores=SCORES,
            budgets=list(BUDGETS),
            targets=TARGETS,
            methods=METHODS,
            score_quantum_m=QUANTUM,
            weighting="parent known-fit/reporting case weights retained within modes and selection",
            retrospective=True,
            evaluation_previously_observed=True,
        ),
        inherited_validation=dict(
            source_sha256=PINNED["estimator-mode-components/sources.json"],
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
        assert digest(args.output / name) == PINNED["estimator-mode-components/" + name]
    save_rows(
        args.output / "conditional-thresholds.csv", [exported(t) for t in conditional]
    )
    save_rows(args.output / "comparison.csv", comparisons)
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
