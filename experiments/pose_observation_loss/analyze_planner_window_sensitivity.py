#!/usr/bin/env python3
"""Audit exact saved diagnostic windows and three prespecified planner weights."""

import argparse
from collections import Counter, defaultdict
from fractions import Fraction
import hashlib
from itertools import product
import json
import math
from pathlib import Path
import shutil

from analyze_commanded_recovery_arc import flag
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import KEY
import analyze_seed_conditioned_rankings as previous
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
PROTOCOL = RESULTS / "planner-window-sensitivity/protocol.md"
FAMILY = [k for k in KEY if k != "planner"]
METHODS = ["case", "window", "family"]
PINNED = {
    **previous.PINNED,
    "seed-conditioned-rankings/sources.json": "2bdf94ae3c3ce99ff72534a80261ec5bee8101f1de0b2c7ffe41c85ebff02ab1",
    "seed-conditioned-rankings/component-summary.csv": "3d51a71e670b4f8993752213465f9b71d65207a01618f79ce34e7ec2d72a6785",
    "seed-conditioned-rankings/ranking-summary.csv": "aa747a8a6288909d5d273b1d35be972d28186ec2bbd9f229d37c7cd28b5abd54",
    "seed-conditioned-rankings/seed-blocks.csv": "818d5024cf638389f197dbd68e41ab4c9caa2ac60a0d188ad897f3bbd5fb56b0",
}
EVENT_FIELDS = [
    "event_kind",
    "event_time_s",
    "truth_time_s",
    "scheduled",
    "received",
    "precision_activation",
]


def canonical(value):
    if isinstance(value, float):
        assert math.isfinite(value)
        return {"float": (0.0 if value == 0 else value).hex()}
    if value is None or type(value) in [str, int, bool]:
        return value
    if isinstance(value, dict):
        return {k: canonical(v) for k, v in value.items()}
    assert isinstance(value, (tuple, list)), type(value)
    return [canonical(v) for v in value]


def window_bytes(payloads):
    ordered = sorted(payloads, key=lambda p: (p["time_s"], p["truth_time_s"]))
    assert len({(p["time_s"], p["truth_time_s"]) for p in ordered}) == len(
        ordered
    ), "duplicate state time"
    return json.dumps(
        canonical(ordered), sort_keys=True, separators=(",", ":")
    ).encode()


def payload(row, projection, truth_time, events):
    semantics = []
    for event_id in row["source_event_ids"].split("+"):
        event = events[event_id]
        assert previous.identity(event) == previous.identity(row)
        semantics.append(
            (
                event["event_kind"],
                float(event["event_time_s"]),
                float(event["truth_time_s"]),
                *[flag(event[k]) for k in EVENT_FIELDS[3:]],
            )
        )
    return dict(
        time_s=float(projection["time_s"]),
        truth_time_s=float(truth_time),
        **{k: row[k] for k in ["a_estimator", "b_estimator", "anchor_mode", "stratum"]},
        scores={s: int(row[s + "_tick"]) for s in previous.SCORES},
        residual_weights=[float(projection[k + "_weight"]) for k in ["a", "b"]],
        events=sorted(semantics),
        targets={
            target: dict(
                observed=[float(projection[target + "_" + a]) for a in ["x", "y"]],
                variance=float(projection[target + "_variance"]),
                parts=row[target]["parts"],
                outside=row[target]["outside"],
            )
            for target in previous.TARGETS
        },
    )


def audit_units(accounting, rows):
    cases = {
        previous.identity(r): dict(
            r,
            accounting_partition=r["partition"],
            partition="new" if r["partition"] == "existing_new" else r["partition"],
        )
        for r in accounting
    }
    assert len(cases) == len(accounting), "duplicate accounting case"
    by_case, by_family = defaultdict(list), defaultdict(list)
    for r in rows:
        if r["view"] == previous.VIEW:
            assert previous.identity(r) in cases, "unaccounted source case"
            ledger = cases[previous.identity(r)]
            assert (r["partition"], r["noise"]) == (
                ledger["partition"],
                ledger["noise"],
            ), "accounting reporting group"
            by_case[previous.identity(r)].append(r)
    for key in sorted(cases):
        by_family[tuple(cases[key][k] for k in FAMILY)].append(key)
    families, case_rows, lookup = [], [], {}
    ids = {key: f"C{i:04d}" for i, key in enumerate(sorted(cases), 1)}
    for index, (family_key, keys) in enumerate(sorted(by_family.items()), 1):
        family_id = f"F{index:03d}"
        represented = [key for key in keys if by_case[key]]
        signatures, byte_check = {}, {}
        for key in represented:
            data = window_bytes([r["_window"] for r in by_case[key]])
            sha = hashlib.sha256(data).hexdigest()
            assert (
                sha not in byte_check or byte_check[sha] == data
            ), "window hash collision"
            byte_check[sha] = data
            signatures[key] = sha
        multiplicity = Counter(signatures.values())
        window_ids = {
            sha: f"{family_id}-W{i:02d}"
            for i, sha in enumerate(sorted(multiplicity), 1)
        }
        first = cases[keys[0]]
        planned_planners = {cases[key]["planner"] for key in keys}
        planners = {cases[key]["planner"] for key in represented}
        families.append(
            dict(
                family_id=family_id,
                **dict(zip(FAMILY, family_key)),
                partition=first["partition"],
                accounting_partition=first["accounting_partition"],
                noise=first["noise"],
                planned_cases=len(keys),
                planned_planners="+".join(sorted(planned_planners)),
                represented_cases=len(represented),
                represented_planners="+".join(sorted(planners)),
                missing_source_planners="+".join(sorted(planned_planners - planners)),
                source_records=sum(len(by_case[key]) for key in represented),
                exact_windows=len(multiplicity),
                shared_window_cases=sum(n for n in multiplicity.values() if n > 1),
                extra_window_copies=len(represented) - len(multiplicity),
                max_window_multiplicity=max(multiplicity.values(), default=0),
                window_multiplicities="+".join(
                    str(n) for n in sorted(multiplicity.values())
                ),
            )
        )
        for key in keys:
            n = len(by_case[key])
            sha = signatures.get(key, "")
            copies = multiplicity[sha] if n else 0
            record = dict(
                case_id=ids[key],
                family_id=family_id,
                **{
                    k: cases[key][k]
                    for k in [*KEY, "partition", "accounting_partition", "noise"]
                },
                parent_records=n,
                source_event_ids="+".join(
                    sorted(
                        {
                            e
                            for r in by_case[key]
                            for e in r["source_event_ids"].split("+")
                        },
                        key=int,
                    )
                ),
                window_sha256=sha,
                window_id=window_ids.get(sha, ""),
                duplicate_cases=copies,
                family_source_cases=len(represented),
                case_row_weight=Fraction(1, n) if n else None,
                window_row_weight=Fraction(1, n * copies) if n else None,
                family_row_weight=Fraction(1, n * len(represented)) if n else None,
            )
            lookup[key] = record
            case_rows.append(previous.historical.exported(record))
    return families, case_rows, lookup


def unit_id(info, method):
    return info[
        {"case": "case_id", "window": "window_id", "family": "family_id"}[method]
    ]


def renamed(metrics):
    return {k.replace("_case_mass", "_analysis_mass"): v for k, v in metrics.items()}


def summaries(rows, lookup):
    parents = defaultdict(list)
    for r in rows:
        assert (
            r["partition"] in previous.SEEDS
            and r["seed"] in previous.SEEDS[r["partition"]]
        )
        if r["view"] == previous.VIEW:
            parents[r["partition"], r["noise"], r["variant"]].append(r)
    components, ranks, blocks, units = [], [], [], []
    baseline = {}
    for partition, noise, variant in product(
        previous.SEEDS, previous.NOISES, previous.ARMS
    ):
        parent = parents[partition, noise, variant]
        for method in METHODS:
            weights = [
                lookup[previous.identity(r)][method + "_row_weight"] for r in parent
            ]
            parent_units = len(
                {unit_id(lookup[previous.identity(r)], method) for r in parent}
            )
            parent_mass = sum(weights, Fraction())
            assert parent_mass == parent_units
            units.append(
                previous.historical.exported(
                    dict(
                        partition=partition,
                        noise=noise,
                        variant=variant,
                        weighting=method,
                        records=len(parent),
                        cases=len({previous.identity(r) for r in parent}),
                        parent_units=parent_units,
                        parent_mass=parent_mass,
                    )
                )
            )
            for mode, target in product(previous.STRATA, previous.TARGETS):
                selected = [
                    (r, w)
                    for r, w in zip(parent, weights)
                    if mode == "all" or r["stratum"] == mode
                ]
                rr, ww = [r for r, w in selected], [w for r, w in selected]
                mass = sum(ww, Fraction())
                common = dict(
                    partition=partition,
                    noise=noise,
                    variant=variant,
                    stratum=mode,
                    target=target,
                    weighting=method,
                    records=len(rr),
                    cases=len({previous.identity(r) for r in rr}),
                    seeds="+".join(sorted({r["seed"] for r in rr})),
                    seed_count=len({r["seed"] for r in rr}),
                    analysis_units=len(
                        {unit_id(lookup[previous.identity(r)], method) for r in rr}
                    ),
                    analysis_mass=mass,
                    parent_units=parent_units,
                    parent_mass=parent_mass,
                    mass_share=mass / parent_mass if parent_mass else None,
                )
                components.append(
                    previous.historical.exported(
                        dict(
                            **common,
                            **renamed(
                                previous.historical.component_metrics(rr, ww, target)
                            ),
                        )
                    )
                )
                for score, label in product(
                    previous.SCORES, previous.historical.LABELS
                ):
                    metrics, pieces = previous.decompose(
                        rr, ww, target, score, label, previous.SEEDS[partition]
                    )
                    key = tuple(common[k] for k in previous.GROUP) + (score, label)
                    if method == "case":
                        baseline[key] = metrics
                    reference = baseline[key]
                    differences = {
                        field
                        + "_difference": (
                            metrics[field] - reference[field]
                            if metrics[field] is not None
                            and reference[field] is not None
                            else None
                        )
                        for field in ["auc", "same_seed_auc"]
                    }
                    ranks.append(
                        previous.historical.exported(
                            dict(
                                **common,
                                score=score,
                                label=label,
                                **renamed(metrics),
                                **differences,
                            )
                        )
                    )
                    for piece in pieces:
                        blocks.append(
                            previous.historical.exported(
                                dict(
                                    **{k: common[k] for k in previous.GROUP},
                                    weighting=method,
                                    score=score,
                                    label=label,
                                    **renamed(piece),
                                )
                            )
                        )
    return components, ranks, blocks, units


def check_previous(components, ranks, blocks):
    mapping = {
        "analysis_mass": "case_mass",
        "parent_mass": "group_case_mass",
        "mass_share": "case_mass_share",
    }
    count = 0
    for filename, values, key in [
        ("component-summary.csv", components, previous.GROUP),
        ("ranking-summary.csv", ranks, previous.RANK),
        ("seed-blocks.csv", blocks, [*previous.RANK, "positive_seed", "negative_seed"]),
    ]:
        old = read_rows(RESULTS / "seed-conditioned-rankings" / filename)
        lookup = {
            tuple(r[k] for k in key): r for r in values if r["weighting"] == "case"
        }
        assert len(lookup) == len(old)
        for p in old:
            r = lookup[tuple(p[k] for k in key)]
            for field, expected in p.items():
                new = next(
                    (k for k, v in mapping.items() if v == field),
                    field.replace("_case_mass", "_analysis_mass"),
                )
                value = r[new]
                if isinstance(value, bool) or isinstance(value, str):
                    assert str(value) == expected, (field, value, expected)
                else:
                    previous.close(value, expected)
                count += 1
    return count


def load_rows(events):
    names = [
        "estimator-mode-transfer/membership.csv",
        "contrast-common-errors/simultaneous.csv",
        "speed-probe-error-components/components.csv",
        "contrast-risk-transfer/scores.csv",
    ]
    members, projections, poses, scores = [read_rows(RESULTS / name) for name in names]
    old, old_audits = previous.historical.reconstruct(
        members, projections, poses, scores
    )
    projection_lookup = {p["source_id"]: p for p in projections}
    pose_lookup = {p["pose_id"]: p for p in poses}
    for r in old:
        if r["view"] == previous.VIEW:
            r["_window"] = payload(
                r,
                projection_lookup[r["source_id"]],
                pose_lookup[r["a_id"]]["truth_time_s"],
                events,
            )
    scores, projections, poses = [
        read_rows(previous.fresh.PRIOR / n)
        for n in ["scores.csv", "projections.csv", "poses.csv"]
    ]
    latest, fresh_audits = previous.fresh.reconstruct(scores, projections, poses)
    projection_lookup = {p["source_id"]: p for p in projections}
    for r in latest:
        if r["view"] == previous.VIEW:
            p = projection_lookup[r["source_id"]]
            r["_window"] = payload(r, p, p["truth_time_s"], events)
    return [*old, *latest], [*old_audits, *fresh_audits]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for name, sha in PINNED.items():
        assert digest(RESULTS / name) == sha, name
    source = json.loads(
        (RESULTS / "seed-conditioned-rankings/sources.json").read_text()
    )
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_seed_conditioned_rankings.py": source["analysis"]["script_sha256"],
    }
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    accounting = read_rows(RESULTS / previous.COPIES["accounting.csv"])
    events_list = read_rows(RESULTS / previous.COPIES["events.csv"])
    events = {r["event_id"]: r for r in events_list}
    assert len(accounting) == 972 and len(events) == len(events_list) == 3997
    rows, reconstruction = load_rows(events)
    assert len(rows) == 32985
    families, cases, lookup = audit_units(accounting, rows)
    assert len(families) == 324 and all(f["planned_cases"] == 3 for f in families)
    assert sum(c["parent_records"] > 0 for c in cases) == 157
    components, ranks, blocks, units = summaries(rows, lookup)
    assert tuple(map(len, [components, ranks, blocks, units])) == (360, 3240, 29160, 36)
    comparisons = check_previous(components, ranks, blocks)
    args.output.mkdir(parents=True)
    for name, data in [
        ("families", families),
        ("cases", cases),
        ("component-summary", components),
        ("ranking-summary", ranks),
        ("seed-blocks", blocks),
        ("unit-summary", units),
    ]:
        save_rows(args.output / (name + ".csv"), data)
    for name, original in previous.COPIES.items():
        shutil.copyfile(RESULTS / original, args.output / name)
        assert digest(args.output / name) == PINNED[original]
    audit = dict(
        source_pairs=len(rows),
        source_states=len(rows) // 3,
        source_cases=157,
        source_opportunities=275,
        accounted_cases=len(accounting),
        planned_families=len(families),
        represented_families=sum(f["represented_cases"] > 0 for f in families),
        represented_windows=sum(f["exact_windows"] for f in families),
        shared_window_cases=sum(f["shared_window_cases"] for f in families),
        extra_window_copies=sum(f["extra_window_copies"] for f in families),
        max_window_multiplicity=max(f["max_window_multiplicity"] for f in families),
        opportunities=len(events),
        timeout_cases=570,
        component_rows=len(components),
        ranking_rows=len(ranks),
        seed_block_rows=len(blocks),
        previous_field_comparisons=comparisons,
        max_vector_closure_m=max(
            a[t + "_vector_closure_m"] for a in reconstruction for t in previous.TARGETS
        ),
        max_residual_component_difference_m=max(
            a["residual_component_difference_m"] for a in reconstruction
        ),
    )
    sources = dict(
        input_sha256=PINNED,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
        ),
        schema=dict(
            weightings=METHODS,
            family_key=FAMILY,
            view=previous.VIEW,
            event_kind="all",
            budget="all_available",
            targets=previous.TARGETS,
            scores=previous.SCORES,
            labels=previous.historical.LABELS,
            strata=previous.STRATA,
            seeds=previous.SEEDS,
            fingerprint="exact typed float.hex JSON; signed zero normalized; whole saved RP diagnostic window only",
            event_fields=EVENT_FIELDS,
            weighting="1/n_case divided by exact window copies or represented family cases before mode/seed conditioning",
            analysis_units="weighting units, not independent samples",
            mass_fields="analysis mass replaces legacy case_mass naming",
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
