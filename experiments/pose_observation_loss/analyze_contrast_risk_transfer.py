#!/usr/bin/env python3
"""Retrospective seed-cohort transfer of fixed contrast/common risk scores."""

import argparse
from collections import Counter, defaultdict
from fractions import Fraction
import hashlib
from itertools import product
import json
import math
from pathlib import Path

from analyze_commanded_recovery_arc import flag
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import KEY
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/contrast-common-errors"
PROTOCOL = ROOT / "results/contrast-risk-transfer/protocol.md"
PINNED = {
    "accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "events.csv": "819b026c74884681e8df9fb282998a19ade0d12f82e8d1ffa3ad1a3b5ce1df6f",
    "loss-links.csv": "b25ba5f8fe9f6cafbe1bc20a32b318455804adf3dc1639f10d335607fea7222b",
    "simultaneous.csv": "25e405945fd3e18ea93738866cc9b265bda55017ec0d04422ea03f52c6b31031",
    "lagged.csv": "70c574b08b42f6a04ee4b06a5c94e65f98779cbf03c8e2c8b38b6a9bacc6fab8",
    "sources.json": "6603a1e1960b94a83ef0f4b2a7e844ed18bbe2722a1d22df0465e7a5f3a10aa2",
}
PARTITIONS = {"known": {269, 270, 271}, "new": {274, 275, 276}}
NOISES = ["white", "correlated"]
ARMS = ["epoch", "curved"]
KINDS = [
    "received",
    "missed",
    "precision_only",
    "received_and_precision",
    "missed_and_precision",
]
SIM_VIEWS = ["sim_raw_recovery", "sim_raw_preview", "sim_recovery_preview"]
PHASES = ["immediate", "after_0p18", "after_0p5", "after_1p5"]
VIEWS = SIM_VIEWS + [
    "lag_" + inp + "_" + phase for phase in PHASES for inp in ["actual", "preview"]
]
SCORES = ["contrast", "midpoint_sigma", "residual_sigma"]
BUDGETS = {
    "q25": Fraction(1, 4),
    "q50": Fraction(1, 2),
    "q75": Fraction(3, 4),
    "all": None,
}
TARGETS = ["midpoint", "residual"]
QUANTUM = 1e-8


def membership(r):
    partition, noise = r["cohort"].split("-", 1)
    assert partition in PARTITIONS and noise in NOISES
    assert int(r["seed"]) in PARTITIONS[partition]
    return partition, noise


def score_tick(value):
    assert math.isfinite(value) and value >= 0
    return int(round(value / QUANTUM))


def case_weights(rows):
    assert all(r["available"] for r in rows)
    counts = Counter(tuple(r[k] for k in KEY) for r in rows)
    return [Fraction(1, counts[tuple(r[k] for k in KEY)]) for r in rows]


def weighted_quantile(ticks, weights, q):
    assert len(ticks) == len(weights) and ticks and 0 < q <= 1
    levels = defaultdict(Fraction)
    for tick, w in zip(ticks, weights):
        assert w > 0
        levels[tick] += w
    target = q * sum(weights, Fraction())
    cumulative = Fraction()
    for tick in sorted(levels):
        cumulative += levels[tick]
        if cumulative >= target:
            return tick
    raise AssertionError("Quantile did not reach total weight")


def auc(ticks, labels, weights):
    assert len(ticks) == len(labels) == len(weights)
    levels = defaultdict(lambda: [Fraction(), Fraction()])
    for tick, label, w in zip(ticks, labels, weights):
        assert label in [True, False] and w > 0
        levels[tick][int(label)] += w
    positives = sum((p for n, p in levels.values()), Fraction())
    negatives = sum((n for n, p in levels.values()), Fraction())
    if not positives or not negatives:
        return ""
    previous, credit = Fraction(), Fraction()
    for tick in sorted(levels):
        n, p = levels[tick]
        credit += p * (previous + n / 2)
        previous += n
    return float(credit / (positives * negatives))


def score_rows(sim, lag, links, events):
    raw_counts, command_counts = Counter(), Counter()
    for l in links:
        for name in ["raw_recovery", "raw_preview", "recovery_preview"]:
            sid = l[name + "_pair_id"]
            raw_counts[sid] += 1
            if flag(l["command_only_zero_loss"]):
                command_counts[sid] += 1
    result = []
    for family, rows in [("simultaneous", sim), ("lagged", lag)]:
        for r in rows:
            partition, noise = membership(r)
            view = (
                "sim_" + r["pairing"]
                if family == "simultaneous"
                else "lag_" + r["input_kind"] + "_" + r["phase"]
            )
            assert view in VIEWS
            ids = r["source_event_ids"] if family == "simultaneous" else r["event_id"]
            for eid in ids.split("+"):
                assert all(events[eid][k] == r[k] for k in KEY)
            available = flag(r["available"])
            values = {}
            for name, field in [
                ("contrast", "contrast_norm"),
                ("midpoint_sigma", "midpoint_variance"),
                ("residual_sigma", "residual_variance"),
            ]:
                value = float(r[field]) if available else None
                if value is not None and name != "contrast":
                    assert value >= 0
                    value = math.sqrt(value)
                values[name + "_score_m"] = value if available else ""
                values[name + "_tick"] = score_tick(value) if available else ""
            for target in TARGETS:
                values[target + "_error_m"] = (
                    float(r[target + "_norm"]) if available else ""
                )
                values[target + "_outside"] = (
                    float(r[target + "_norm"]) > 0.1 if available else ""
                )
            result.append(
                dict(
                    source_family=family,
                    source_id=r["source_id"],
                    **{k: r[k] for k in KEY},
                    partition=partition,
                    noise=noise,
                    view=view,
                    source_event_ids=ids,
                    a_id=r["a_id"],
                    b_id=r["b_id"],
                    source_category=r["category"],
                    available=available,
                    zero_or_roundoff_contrast=(
                        flag(r["zero_or_roundoff_contrast"]) if available else ""
                    ),
                    oracle_contrast=family == "lagged",
                    raw_zero_loss_references=(
                        raw_counts[r["source_id"]] if family == "simultaneous" else 0
                    ),
                    command_only_loss_references=(
                        command_counts[r["source_id"]]
                        if family == "simultaneous"
                        else 0
                    ),
                    **values,
                )
            )
    assert sum(r["raw_zero_loss_references"] for r in result) == len(links) * 3
    assert (
        sum(r["command_only_loss_references"] for r in result)
        == sum(flag(l["command_only_zero_loss"]) for l in links) * 3
    )
    return result


def fit_thresholds(rows):
    groups = defaultdict(list)
    for r in rows:
        if r["partition"] == "known" and r["available"]:
            groups[r["noise"], r["view"]].append(r)
    result = []
    for noise, view, score in product(NOISES, VIEWS, SCORES):
        train = groups[noise, view]
        assert all(
            r["partition"] == "known" and int(r["seed"]) in PARTITIONS["known"]
            for r in train
        )
        weights = case_weights(train)
        ids = sorted((r["source_family"], int(r["source_id"])) for r in train)
        fingerprint = hashlib.sha256(
            "".join(f"{family}:{sid}\n" for family, sid in ids).encode()
        ).hexdigest()
        for budget, q in BUDGETS.items():
            rule = (
                "all_available"
                if q is None
                else "fitted" if train else "no_training_data"
            )
            cut = (
                weighted_quantile([r[score + "_tick"] for r in train], weights, q)
                if rule == "fitted"
                else ""
            )
            result.append(
                dict(
                    threshold_id=len(result) + 1,
                    noise=noise,
                    view=view,
                    score=score,
                    budget=budget,
                    training_partition="known",
                    training_rows=len(train),
                    training_cases=len({tuple(r[k] for k in KEY) for r in train}),
                    training_total_weight=float(sum(weights, Fraction())),
                    training_seeds="+".join(
                        sorted({r["seed"] for r in train}, key=int)
                    ),
                    training_reference_sha256=fingerprint,
                    rule=rule,
                    quantile=float(q) if q is not None else "",
                    threshold_tick=cut,
                    threshold_m=cut * QUANTUM if cut != "" else "",
                )
            )
    return result


def retained_metrics(rows, weights, score, threshold):
    fields = [
        "retained_rows",
        "retained_cases",
        "retained_case_mass",
        "row_coverage",
        "case_weighted_coverage",
    ]
    for target in TARGETS:
        fields += [
            target + "_error_rows",
            target + "_error_case_mass",
            target + "_case_weighted_risk",
            target + "_row_risk",
            target + "_case_weighted_rms",
        ]
    if threshold["rule"] == "no_training_data":
        return dict.fromkeys(fields, "")
    assert threshold["rule"] in ["fitted", "all_available"]
    selected = [
        (r, w)
        for r, w in zip(rows, weights)
        if threshold["rule"] == "all_available"
        or r[score + "_tick"] <= threshold["threshold_tick"]
    ]
    mass = sum((w for r, w in selected), Fraction())
    total = sum(weights, Fraction())
    result = dict(
        retained_rows=len(selected),
        retained_cases=len({tuple(r[k] for k in KEY) for r, w in selected}),
        retained_case_mass=float(mass),
        row_coverage=len(selected) / len(rows) if rows else "",
        case_weighted_coverage=float(mass / total) if total else "",
    )
    for target in TARGETS:
        bad = sum((w for r, w in selected if r[target + "_outside"]), Fraction())
        count = sum(r[target + "_outside"] for r, w in selected)
        result.update(
            {
                target + "_error_rows": count,
                target + "_error_case_mass": float(bad),
                target + "_case_weighted_risk": float(bad / mass) if mass else "",
                target + "_row_risk": count / len(selected) if selected else "",
                target
                + "_case_weighted_rms": (
                    math.sqrt(
                        math.fsum(
                            float(w) * r[target + "_error_m"] ** 2 for r, w in selected
                        )
                        / float(mass)
                    )
                    if mass
                    else ""
                ),
            }
        )
    return result


def summaries(rows, thresholds, events):
    groups = defaultdict(list)
    event_ids = defaultdict(set)
    for r in rows:
        ids = r["source_event_ids"].split("+")
        kinds = {"all", *[events[e]["event_kind"] for e in ids]}
        for kind in kinds:
            key = r["partition"], r["noise"], r["variant"], kind, r["view"]
            groups[key].append(r)
            event_ids[key].update(
                e for e in ids if kind == "all" or events[e]["event_kind"] == kind
            )
    lookup = {(t["noise"], t["view"], t["score"], t["budget"]): t for t in thresholds}
    ranking, risks = [], []
    for partition, noise, arm, kind, view in product(
        PARTITIONS, NOISES, ARMS, ["all", *KINDS], VIEWS
    ):
        key = partition, noise, arm, kind, view
        rr = groups[key]
        matched = [r for r in rr if r["available"]]
        weights = case_weights(matched)
        mass = sum(weights, Fraction())
        common = dict(
            partition=partition,
            noise=noise,
            variant=arm,
            event_kind=kind,
            view=view,
            records=len(rr),
            matched=len(matched),
            unavailable=len(rr) - len(matched),
            cases=len({tuple(r[k] for k in KEY) for r in rr}),
            matched_cases=len({tuple(r[k] for k in KEY) for r in matched}),
            opportunities=len(event_ids[key]),
            case_mass=float(mass),
            zero_or_roundoff_pairs=sum(r["zero_or_roundoff_contrast"] for r in matched),
        )
        for score in SCORES:
            rank = dict(**common, score=score)
            for target in TARGETS:
                labels = [r[target + "_outside"] for r in matched]
                bad = sum(
                    (w for r, w in zip(matched, weights) if r[target + "_outside"]),
                    Fraction(),
                )
                rank[target + "_outside_rows"] = sum(labels)
                rank[target + "_inside_rows"] = len(labels) - sum(labels)
                rank[target + "_outside_case_mass"] = float(bad)
                rank[target + "_baseline_case_weighted_risk"] = (
                    float(bad / mass) if mass else ""
                )
                ticks = [r[score + "_tick"] for r in matched]
                rank[target + "_case_weighted_auc"] = auc(ticks, labels, weights)
                rank[target + "_row_auc"] = auc(
                    ticks, labels, [Fraction(1)] * len(matched)
                )
            ranking.append(rank)
            for budget in BUDGETS:
                t = lookup[noise, view, score, budget]
                risks.append(
                    dict(
                        **common,
                        score=score,
                        budget=budget,
                        threshold_id=t["threshold_id"],
                        rule=t["rule"],
                        threshold_tick=t["threshold_tick"],
                        **retained_metrics(matched, weights, score, t),
                    )
                )
    return ranking, risks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for n, h in PINNED.items():
        assert digest(PRIOR / n) == h, n
    prior = json.loads((PRIOR / "sources.json").read_text())
    helpers = dict(prior["analysis"]["helpers_sha256"])
    helpers["analyze_contrast_common_errors.py"] = prior["analysis"]["script_sha256"]
    for n, h in helpers.items():
        assert digest(ROOT / n) == h, n
    accounting, ee, links, sim, lag = [
        read_rows(PRIOR / n)
        for n in [
            "accounting.csv",
            "events.csv",
            "loss-links.csv",
            "simultaneous.csv",
            "lagged.csv",
        ]
    ]
    assert (len(accounting), len(ee), len(links), len(sim), len(lag)) == (
        648,
        2583,
        550,
        19830,
        20664,
    )
    assert sum(flag(l["command_only_zero_loss"]) for l in links) == 509
    for c in accounting:
        membership(c)
    rows = score_rows(sim, lag, links, {e["event_id"]: e for e in ee})
    assert len(rows) == 40494 and sum(r["available"] for r in rows) == 38286
    cases = {tuple(c[k] for k in KEY) for c in accounting if flag(c["selected"])}
    assert len(cases) == 368 and {tuple(r[k] for k in KEY) for r in rows} == cases
    thresholds = fit_thresholds(rows)
    ranking, risks = summaries(rows, thresholds, {e["event_id"]: e for e in ee})
    assert (len(thresholds), len(ranking), len(risks)) == (264, 1584, 6336)
    audit = dict(
        accounted_cases=648,
        timeout_cases=368,
        opportunities=2583,
        source_rows=len(rows),
        simultaneous_pairs=19830,
        requested_lagged_pairs=20664,
        matched_pairs=38286,
        unavailable_lagged_pairs=2208,
        known_rows=sum(r["partition"] == "known" for r in rows),
        new_rows=sum(r["partition"] == "new" for r in rows),
        threshold_rows=len(thresholds),
        ranking_rows=len(ranking),
        risk_rows=len(risks),
        unavailable_fits=sum(t["rule"] == "no_training_data" for t in thresholds),
        raw_zero_loss_links=550,
        command_only_links=509,
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
            noise=NOISES,
            views=VIEWS,
            scores=SCORES,
            score_quantum_m=QUANTUM,
            budgets={
                k: float(v) if v is not None else None for k, v in BUDGETS.items()
            },
            targets=TARGETS,
            accuracy_radius_m=0.1,
            retrospective=True,
            evaluation_previously_observed=True,
            weighting="one total weight per represented case before retention",
        ),
        inherited_validation=dict(
            source_sha256=PINNED["sources.json"],
            observation_and_physics=True,
            new_rollout=False,
        ),
        audit=audit,
    )
    args.output.mkdir(parents=True)
    for n, rr in [
        ("accounting", accounting),
        ("events", ee),
        ("loss-links", links),
        ("scores", rows),
        ("thresholds", thresholds),
        ("ranking", ranking),
        ("risk-summary", risks),
    ]:
        save_rows(args.output / (n + ".csv"), rr)
    for n in ["accounting", "events", "loss-links"]:
        assert digest(args.output / (n + ".csv")) == PINNED[n + ".csv"]
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
