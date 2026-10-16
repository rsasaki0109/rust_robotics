"""Independent NumPy vectors and direct weighted pairs; no analyzer imports."""

import argparse
import csv
from collections import Counter, defaultdict
from fractions import Fraction
import hashlib
from itertools import combinations, product
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--results", required=True, type=Path)
parser.add_argument("--figures", required=True, type=Path)
parser.add_argument("--output", required=True, type=Path)
args = parser.parse_args()
if not __debug__ or args.output.exists():
    parser.error("Run without -O and use a new evidence file")
DATA, PLOT = args.results, args.figures
KEY = ["cohort", "variant", "mode", "seed", "scenario", "planner"]
GROUP = ["partition", "noise", "variant", "stratum", "target"]
RANK = [*GROUP, "score", "label"]
PARTS = ["noise", "anchor", "drift", "transport"]
LABELS = ["observed", "modeled", "unmodeled"]
SCORES = ["contrast", "midpoint_sigma", "residual_sigma"]
STRATA = [
    "all",
    "recovery_single",
    "filter_single",
    "fusion_three",
    "recovery_fallback_single",
]
SEEDS = {
    "known": ["269", "270", "271"],
    "new": ["274", "275", "276"],
    "fresh": ["277", "278", "279"],
}
CELLS = [
    "modeled_" + a + "_observed_" + b
    for a, b in product(["inside", "outside"], repeat=2)
]
PAIR_FIELDS = [
    "concordant_pair_mass",
    "tied_pair_mass",
    "discordant_pair_mass",
    "numerator",
    "denominator",
]
checks = 0


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check(actual, expected):
    global checks
    checks += 1
    if expected is None:
        assert actual == "", (actual, expected)
    elif isinstance(expected, bool):
        assert actual == expected or actual == str(expected), (actual, expected)
    elif isinstance(expected, str):
        assert actual == expected, (actual, expected)
    else:
        assert actual != "" and math.isclose(
            float(actual), float(expected), rel_tol=1e-10, abs_tol=2e-12
        ), (actual, expected)


def fields(actual, expected):
    assert set(actual) == set(expected), (
        set(actual) - set(expected),
        set(expected) - set(actual),
    )
    for key, value in expected.items():
        check(actual[key], value)


sources = json.loads((DATA / "sources.json").read_text())
for name, digest in sources["input_sha256"].items():
    check(sha(RESULTS / name), digest)
for name, digest in sources["analysis"]["helpers_sha256"].items():
    check(sha(ROOT / name), digest)
check(
    sha(ROOT / "analyze_planner_conditioned_rankings.py"),
    sources["analysis"]["script_sha256"],
)
check(
    sha(RESULTS / "planner-conditioned-rankings/protocol.md"),
    sources["analysis"]["protocol_sha256"],
)
copies = {
    name: "fresh-seed-cutoff-transfer/" + name
    for name in [
        "accounting.csv",
        "case-support.csv",
        "events.csv",
        "thresholds.csv",
        "conditional-thresholds.csv",
    ]
}
copies.update(
    {
        "historical-loss-links.csv": "estimator-mode-transfer/loss-links.csv",
        "fresh-loss-links.csv": "fresh-seed-cutoff-transfer/loss-links.csv",
    }
)
for name, original in copies.items():
    check((DATA / name).read_bytes() == (RESULTS / original).read_bytes(), True)
events = {r["event_id"]: r for r in read(DATA / "events.csv")}
accounting = read(DATA / "accounting.csv")
assert (
    len(accounting) == 972
    and sum(r["selected_timeout"] == "True" for r in accounting) == 570
)
assert len(events) == 3997
assert len(read(DATA / "historical-loss-links.csv")) == 550
assert len(read(DATA / "fresh-loss-links.csv")) == 101

members = read(RESULTS / "estimator-mode-transfer/membership.csv")
old_p = {
    r["source_id"]: r for r in read(RESULTS / "contrast-common-errors/simultaneous.csv")
}
old_pose = {
    r["pose_id"]: r
    for r in read(RESULTS / "speed-probe-error-components/components.csv")
}
old_s = {
    (r["source_family"], r["source_id"]): r
    for r in read(RESULTS / "contrast-risk-transfer/scores.csv")
}
new_s = read(RESULTS / "fresh-seed-cutoff-transfer/scores.csv")
new_p = {
    r["source_id"]: r
    for r in read(RESULTS / "fresh-seed-cutoff-transfer/projections.csv")
}
casekey = lambda r: tuple(r[k] for k in KEY)
new_pose = {
    casekey(r) + (r["state_id"], r["pose_kind"]): r
    for r in read(RESULTS / "fresh-seed-cutoff-transfer/poses.csv")
}
focus = []
for m in members:
    if m["view"] != "sim_recovery_preview":
        continue
    p, s = old_p[m["source_id"]], old_s["simultaneous", m["source_id"]]
    for field in [*KEY, "a_id", "b_id", "source_event_ids"]:
        check(p[field], m[field])
        check(s[field], m[field])
    pp = [old_pose[m["a_id"]], old_pose[m["b_id"]]]
    focus.append(("historical", m, p, s, pp))
for s in new_s:
    if s["view"] != "sim_recovery_preview":
        continue
    p = new_p[s["source_id"]]
    for field in [
        *KEY,
        "state_id",
        "time_s",
        "truth_time_s",
        "source_event_ids",
        "view",
        "stratum",
        "a_estimator",
        "b_estimator",
        "anchor_mode",
    ]:
        check(p[field], s[field])
    pp = [
        new_pose[casekey(s) + (s["state_id"], kind)] for kind in ["recovery", "preview"]
    ]
    focus.append(("latest", s, p, s, pp))
assert (
    len(focus) == 10995
    and len({(family, m["source_id"]) for family, m, p, s, pp in focus}) == 10995
)
counts = Counter(casekey(m) for family, m, p, s, pp in focus)
assert len(counts) == 157
all_data = []
window_inputs = defaultdict(list)
for family, m, p, s, poses in focus:
    key = casekey(m)
    assert m["seed"] in SEEDS[m["partition"]]
    for pose, estimator in zip(poses, [m["a_estimator"], m["b_estimator"]]):
        check(pose["estimator"], estimator)
        for field in [*KEY, "state_id", "time_s", "source_event_ids"]:
            check(pose[field], p[field])
    for event_id in m["source_event_ids"].split("+"):
        for field in KEY:
            check(events[event_id][field], m[field])
    vectors = np.array(
        [
            [[float(pose[part + "_" + axis]) for axis in ["x", "y"]] for part in PARTS]
            for pose in poses
        ]
    )
    assert (
        np.max(
            np.abs(
                vectors.sum(axis=1)
                - np.array(
                    [
                        [float(pose["error_" + axis]) for axis in ["x", "y"]]
                        for pose in poses
                    ]
                )
            )
        )
        < 1e-8
    )
    semantic = []
    for event_id in m["source_event_ids"].split("+"):
        event = events[event_id]
        semantic.append(
            (
                event["event_kind"],
                float(event["event_time_s"]),
                float(event["truth_time_s"]),
                *[
                    event[k] == "True"
                    for k in ["scheduled", "received", "precision_activation"]
                ],
            )
        )
    window = dict(
        time_s=float(p["time_s"]),
        truth_time_s=float(poses[0]["truth_time_s"]),
        **{k: m[k] for k in ["a_estimator", "b_estimator", "anchor_mode", "stratum"]},
        scores={score: int(s[score + "_tick"]) for score in SCORES},
        residual_weights=[float(p[k + "_weight"]) for k in ["a", "b"]],
        events=sorted(semantic),
        targets={},
    )
    for target in ["midpoint", "residual"]:
        w = np.array(
            [0.5, 0.5]
            if target == "midpoint"
            else [float(p["a_weight"]), float(p["b_weight"])]
        )
        parts = np.einsum("i,ijk->jk", w, vectors)
        exact_parts = {
            part: [
                float(
                    sum(
                        (
                            Fraction(float(w[i]) * float(vectors[i, j, k]))
                            for i in range(2)
                        ),
                        Fraction(),
                    )
                )
                for k in range(2)
            ]
            for j, part in enumerate(PARTS)
        }
        exact_e = [float(p[target + "_" + axis]) for axis in ["x", "y"]]
        exact_p = [exact_parts["noise"][k] + exact_parts["anchor"][k] for k in range(2)]
        exact_u = [
            exact_parts["drift"][k] + exact_parts["transport"][k] for k in range(2)
        ]
        window["targets"][target] = dict(
            observed=exact_e,
            variance=float(p[target + "_variance"]),
            parts=exact_parts,
            outside={
                label: math.hypot(*v) > 0.1
                for label, v in zip(LABELS, [exact_e, exact_p, exact_u])
            },
        )
        observed = np.array([float(p[target + "_" + axis]) for axis in ["x", "y"]])
        assert np.max(np.abs(parts.sum(axis=0) - observed)) < 1e-8
        modeled, unmodeled = parts[0] + parts[1], parts[2] + parts[3]
        labels = {
            label: float(np.hypot(*v)) > 0.1
            for label, v in zip(LABELS, [observed, modeled, unmodeled])
        }
        check(s[target + "_outside"], labels["observed"])
        energy = {part + "_energy_m2": float(v @ v) for part, v in zip(PARTS, parts)}
        energy.update(
            {
                PARTS[a]
                + "_"
                + PARTS[b]
                + "_cross_m2": float(2 * (parts[a] @ parts[b]))
                for a, b in combinations(range(4), 2)
            }
        )
        energy.update(
            {
                label + "_energy_m2": float(v @ v)
                for label, v in zip(LABELS, [observed, modeled, unmodeled])
            }
        )
        energy["partial_model_energy_m2"] = 2 * float(p[target + "_variance"])
        energy["modeled_unmodeled_cross_m2"] = float(2 * (modeled @ unmodeled))
        cell = (
            "modeled_"
            + ("outside" if labels["modeled"] else "inside")
            + "_observed_"
            + ("outside" if labels["observed"] else "inside")
        )
        r = dict(
            m,
            target=target,
            weight=1 / counts[key],
            case_key=key,
            energy=energy,
            outside=labels,
            cell=cell,
        )
        for score in SCORES:
            tick = int(s[score + "_tick"])
            assert tick == round(float(s[score + "_score_m"]) / 1e-8)
            r[score + "_tick"] = tick
        all_data.append(r)
    window_inputs[key].append(window)
assert len(all_data) == 21990


def support(rr):
    return dict(
        records=len(rr),
        cases=len({r["case_key"] for r in rr}),
        case_mass=math.fsum(r["weight"] for r in rr),
        seeds="+".join(sorted({r["seed"] for r in rr})),
        seed_count=len({r["seed"] for r in rr}),
    )


def direct_pairs(positive, negative, score):
    p = np.array([r[score + "_tick"] for r in positive], dtype=np.int64)
    n = np.array([r[score + "_tick"] for r in negative], dtype=np.int64)
    mass = (
        np.array([r["weight"] for r in positive])[:, None]
        * np.array([r["weight"] for r in negative])[None, :]
    )
    concordant = float(np.sum(mass * (p[:, None] > n[None, :])))
    tied = float(np.sum(mass * (p[:, None] == n[None, :])))
    denominator = math.fsum(r["weight"] for r in positive) * math.fsum(
        r["weight"] for r in negative
    )
    numerator = concordant + 0.5 * tied
    return dict(
        concordant_pair_mass=concordant,
        tied_pair_mass=tied,
        discordant_pair_mass=denominator - concordant - tied,
        numerator=numerator,
        denominator=denominator,
        auc=numerator / denominator if denominator else None,
    )


METHODS = ["case", "window", "family"]
FAMILY = [k for k in KEY if k != "planner"]


def encoded(v):
    if type(v) is float:
        assert math.isfinite(v)
        return {"float": float.hex(0.0 if v == 0 else v)}
    if isinstance(v, dict):
        return {k: encoded(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return list(map(encoded, v))
    assert v is None or type(v) in [str, int, bool]
    return v


ledger = {tuple(r[k] for k in KEY): r for r in accounting}
assert len(ledger) == 972
family_cases = defaultdict(list)
for key in sorted(ledger):
    family_cases[tuple(ledger[key][k] for k in FAMILY)].append(key)
assert len(family_cases) == 324
fingerprints = {}
canonical_windows = {}
for key, payloads in window_inputs.items():
    ordered = sorted(payloads, key=lambda p: (p["time_s"], p["truth_time_s"]))
    assert len({(p["time_s"], p["truth_time_s"]) for p in ordered}) == len(ordered)
    serialized = json.dumps(
        encoded(ordered), sort_keys=True, separators=(",", ":")
    ).encode()
    fingerprints[key] = hashlib.sha256(serialized).hexdigest()
    canonical_windows[key] = serialized
assert len(fingerprints) == 157
families_actual = {r["family_id"]: r for r in read(DATA / "families.csv")}
cases_actual = {r["case_id"]: r for r in read(DATA / "cases.csv")}
assert len(families_actual) == 324 and len(cases_actual) == 972
case_ids = {key: f"C{i:04d}" for i, key in enumerate(sorted(ledger), 1)}
case_info, expected_families = {}, {}
for index, (family_key, keys) in enumerate(sorted(family_cases.items()), 1):
    family_id = f"F{index:03d}"
    represented = [key for key in keys if counts[key]]
    multiplicity = Counter(fingerprints[key] for key in represented)
    window_ids = {
        h: f"{family_id}-W{i:02d}" for i, h in enumerate(sorted(multiplicity), 1)
    }
    by_hash = {}
    for key in represented:
        h = fingerprints[key]
        assert h not in by_hash or by_hash[h] == canonical_windows[key]
        by_hash[h] = canonical_windows[key]
    first = ledger[keys[0]]
    partition = "new" if first["partition"] == "existing_new" else first["partition"]
    planned = {ledger[k]["planner"] for k in keys}
    planners = {ledger[k]["planner"] for k in represented}
    expected = dict(
        family_id=family_id,
        **dict(zip(FAMILY, family_key)),
        partition=partition,
        accounting_partition=first["partition"],
        noise=first["noise"],
        planned_cases=len(keys),
        planned_planners="+".join(sorted(planned)),
        represented_cases=len(represented),
        represented_planners="+".join(sorted(planners)),
        missing_source_planners="+".join(sorted(planned - planners)),
        source_records=sum(counts[k] for k in represented),
        exact_windows=len(multiplicity),
        shared_window_cases=sum(n for n in multiplicity.values() if n > 1),
        extra_window_copies=len(represented) - len(multiplicity),
        max_window_multiplicity=max(multiplicity.values(), default=0),
        window_multiplicities="+".join(str(n) for n in sorted(multiplicity.values())),
    )
    fields(families_actual[family_id], expected)
    expected_families[family_id] = expected
    for key in keys:
        r = ledger[key]
        n = counts[key]
        h = fingerprints.get(key, "")
        m = multiplicity[h] if n else 0
        event_ids = {
            event
            for family, meta, p, s, poses in focus
            if tuple(meta[k] for k in KEY) == key
            for event in meta["source_event_ids"].split("+")
        }
        expected = dict(
            case_id=case_ids[key],
            family_id=family_id,
            **{k: r[k] for k in KEY},
            partition=partition,
            accounting_partition=r["partition"],
            noise=r["noise"],
            parent_records=n,
            source_event_ids="+".join(sorted(event_ids, key=int)),
            window_sha256=h,
            window_id=window_ids.get(h, ""),
            duplicate_cases=m,
            family_source_cases=len(represented),
            case_row_weight=Fraction(1, n) if n else None,
            window_row_weight=Fraction(1, n * m) if n else None,
            family_row_weight=Fraction(1, n * len(represented)) if n else None,
        )
        fields(cases_actual[case_ids[key]], expected)
        case_info[key] = expected
assert sum(f["represented_cases"] > 0 for f in expected_families.values()) == 72
assert sum(f["exact_windows"] for f in expected_families.values()) == 125
assert sum(f["shared_window_cases"] for f in expected_families.values()) == 64

PLANNERS = ["A*", "JPS", "Theta*"]
NAMES = {"A*": "astar", "JPS": "jps", "Theta*": "theta"}
POPULATIONS = ["source", "common_family"]
NEWGROUP = [*GROUP, "population", "planner_selection"]
NEWRANK = [*NEWGROUP, "score", "label"]


def index(name, keys):
    rr = read(DATA / name)
    out = {tuple(r[k] for k in keys): r for r in rr}
    assert len(out) == len(rr)
    return out


for name in ["cases.csv", "families.csv"]:
    check(
        (DATA / name).read_bytes()
        == (RESULTS / "planner-window-sensitivity" / name).read_bytes(),
        True,
    )
for name, h in sources["output_sha256"].items():
    check(sha(DATA / name), h)
family_support = index("family-support.csv", ["family_id", "stratum"])
eligibility = defaultdict(set)
expected_family_support = {}
for family_id, f in expected_families.items():
    parent = [
        r
        for r in all_data
        if r["target"] == "midpoint"
        and case_info[r["case_key"]]["family_id"] == family_id
    ]
    check(f["planned_planners"], "+".join(PLANNERS))
    for mode in STRATA:
        rr = [r for r in parent if mode == "all" or r["stratum"] == mode]
        planners = {r["planner"] for r in rr}
        common = planners == set(PLANNERS)
        if common:
            eligibility[mode].add(family_id)
        expected = {
            k: f[k]
            for k in [
                "family_id",
                *FAMILY,
                "partition",
                "accounting_partition",
                "noise",
                "planned_planners",
            ]
        }
        expected.update(
            stratum=mode,
            source_records=len(rr),
            represented_planners="+".join(sorted(planners)),
            missing_source_planners="+".join(sorted(set(PLANNERS) - planners)),
            common_family=common,
        )
        for planner in PLANNERS:
            selected = [r for r in rr if r["planner"] == planner]
            expected[NAMES[planner] + "_records"] = len(selected)
            expected[NAMES[planner] + "_case_mass"] = math.fsum(
                r["weight"] for r in selected
            )
        fields(family_support[family_id, mode], expected)
        expected_family_support[family_id, mode] = expected
assert len(family_support) == len(expected_family_support) == 1620


def source_support(rr):
    return dict(
        **support(rr), families=len({case_info[r["case_key"]]["family_id"] for r in rr})
    )


def component_metrics(rr):
    mass = math.fsum(r["weight"] for r in rr)
    metrics = {
        field: (
            math.fsum(r["weight"] * r["energy"][field] for r in rr) / mass
            if mass
            else None
        )
        for field in all_data[0]["energy"]
    }
    metrics["energy_closure_m2"] = (
        metrics["observed_energy_m2"]
        - metrics["modeled_energy_m2"]
        - metrics["unmodeled_energy_m2"]
        - metrics["modeled_unmodeled_cross_m2"]
        if mass
        else None
    )
    metrics.update(
        {
            label + "_rms_m": math.sqrt(metrics[label + "_energy_m2"]) if mass else None
            for label in [*LABELS, "partial_model"]
        }
    )
    metrics["observed_to_partial_model_energy_ratio"] = (
        metrics["observed_energy_m2"] / metrics["partial_model_energy_m2"]
        if mass and metrics["partial_model_energy_m2"]
        else None
    )
    for label in [*LABELS, *CELLS]:
        bad = (
            [r for r in rr if r["outside"][label]]
            if label in LABELS
            else [r for r in rr if r["cell"] == label]
        )
        prefix = label + "_outside" if label in LABELS else label
        metrics[prefix + "_rows"] = len(bad)
        metrics[prefix + "_case_mass"] = math.fsum(r["weight"] for r in bad)
        metrics[prefix + "_fraction"] = (
            metrics[prefix + "_case_mass"] / mass if mass else None
        )
    return metrics


def ranking_metrics(rr, score, label, seeds):
    positive = [r for r in rr if r["outside"][label]]
    negative = [r for r in rr if not r["outside"][label]]
    full = direct_pairs(positive, negative, score)
    result = dict(
        outside_rows=len(positive),
        inside_rows=len(negative),
        outside_cases=len({r["case_key"] for r in positive}),
        inside_cases=len({r["case_key"] for r in negative}),
        outside_case_mass=math.fsum(r["weight"] for r in positive),
        inside_case_mass=math.fsum(r["weight"] for r in negative),
        **full,
    )
    seed_blocks = []
    for pos, neg in product(seeds, repeat=2):
        piece = direct_pairs(
            [r for r in positive if r["seed"] == pos],
            [r for r in negative if r["seed"] == neg],
            score,
        )
        piece["same_seed"] = pos == neg
        seed_blocks.append(piece)
    for same, name in [(True, "same_seed"), (False, "between_seed")]:
        num = math.fsum(p["numerator"] for p in seed_blocks if p["same_seed"] == same)
        den = math.fsum(p["denominator"] for p in seed_blocks if p["same_seed"] == same)
        result[name + "_numerator"] = num
        result[name + "_denominator"] = den
        result[name + "_auc"] = num / den if den else None
        result[name + "_pair_share"] = (
            den / full["denominator"] if full["denominator"] else None
        )
    result["seeds_with_both_classes"] = sum(
        b["denominator"] > 0 for b in seed_blocks if b["same_seed"]
    )
    result["pooled_minus_same_seed_auc"] = (
        full["auc"] - result["same_seed_auc"]
        if full["auc"] is not None and result["same_seed_auc"] is not None
        else None
    )
    for name, groupfields in [
        ("same_planner", ["planner"]),
        ("same_seed_planner", ["planner", "seed"]),
    ]:
        num = den = 0.0
        groups = {tuple(r[k] for k in groupfields) for r in rr}
        for group in groups:
            piece = direct_pairs(
                [r for r in positive if tuple(r[k] for k in groupfields) == group],
                [r for r in negative if tuple(r[k] for k in groupfields) == group],
                score,
            )
            num += piece["numerator"]
            den += piece["denominator"]
        result[name + "_numerator"] = num
        result[name + "_denominator"] = den
        result[name + "_auc"] = num / den if den else None
        result[name + "_pair_share"] = (
            den / full["denominator"] if full["denominator"] else None
        )
    return result


components = index("component-summary.csv", NEWGROUP)
rankings = index("ranking-summary.csv", NEWRANK)
seed_support = index("seed-support.csv", [*NEWGROUP, "seed"])
blocks = index(
    "planner-blocks.csv", [*RANK, "population", "positive_planner", "negative_planner"]
)
expected_keys = [set() for _ in range(4)]
defined_auc = defined_same = 0
for partition, noise, arm, mode, target, population in product(
    SEEDS,
    ["white", "correlated"],
    ["epoch", "curved"],
    STRATA,
    ["midpoint", "residual"],
    POPULATIONS,
):
    group = partition, noise, arm, mode, target
    parent = [
        r
        for r in all_data
        if (r["partition"], r["noise"], r["variant"], r["target"])
        == (partition, noise, arm, target)
    ]
    parent_mass = math.fsum(r["weight"] for r in parent)
    selected = [
        r
        for r in parent
        if (mode == "all" or r["stratum"] == mode)
        and (
            population == "source"
            or case_info[r["case_key"]]["family_id"] in eligibility[mode]
        )
    ]
    for planner in ["all", *PLANNERS]:
        rr = [r for r in selected if planner == "all" or r["planner"] == planner]
        key = (*group, population, planner)
        common = dict(zip(NEWGROUP, key))
        common.update(
            **source_support(rr),
            group_case_mass=parent_mass,
            case_mass_share=(
                math.fsum(r["weight"] for r in rr) / parent_mass
                if parent_mass
                else None
            ),
        )
        fields(components[key], dict(**common, **component_metrics(rr)))
        expected_keys[0].add(key)
        for seed in SEEDS[partition]:
            ss = [r for r in rr if r["seed"] == seed]
            record = dict(zip(NEWGROUP, key))
            record.update(
                seed=seed,
                **source_support(ss),
                group_case_mass=parent_mass,
                stratum_case_mass=math.fsum(r["weight"] for r in rr),
            )
            mass = math.fsum(r["weight"] for r in ss)
            for label in LABELS:
                bad = [r for r in ss if r["outside"][label]]
                badmass = math.fsum(r["weight"] for r in bad)
                record[label + "_outside_rows"] = len(bad)
                record[label + "_outside_cases"] = len({r["case_key"] for r in bad})
                record[label + "_outside_case_mass"] = badmass
                record[label + "_outside_fraction"] = badmass / mass if mass else None
            fields(seed_support[*key, seed], record)
            expected_keys[2].add((*key, seed))
        for score, label in product(SCORES, LABELS):
            metrics = ranking_metrics(rr, score, label, SEEDS[partition])
            record = dict(**common, score=score, label=label, **metrics)
            fields(rankings[*key, score, label], record)
            expected_keys[1].add((*key, score, label))
            defined_auc += metrics["auc"] is not None
            defined_same += metrics["same_seed_auc"] is not None
    for score, label in product(SCORES, LABELS):
        positive = [r for r in selected if r["outside"][label]]
        negative = [r for r in selected if not r["outside"][label]]
        full = direct_pairs(positive, negative, score)
        pieces = []
        for pos, neg in product(PLANNERS, repeat=2):
            pp = [r for r in positive if r["planner"] == pos]
            nn = [r for r in negative if r["planner"] == neg]
            result = direct_pairs(pp, nn, score)
            diag = [
                direct_pairs(
                    [r for r in pp if r["seed"] == s],
                    [r for r in nn if r["seed"] == s],
                    score,
                )
                for s in SEEDS[partition]
            ]
            num = math.fsum(b["numerator"] for b in diag)
            den = math.fsum(b["denominator"] for b in diag)
            record = dict(zip(RANK, (*group, score, label)))
            record.update(
                population=population,
                positive_planner=pos,
                negative_planner=neg,
                same_planner=pos == neg,
                positive_rows=len(pp),
                positive_cases=len({r["case_key"] for r in pp}),
                negative_rows=len(nn),
                negative_cases=len({r["case_key"] for r in nn}),
                positive_case_mass=math.fsum(r["weight"] for r in pp),
                negative_case_mass=math.fsum(r["weight"] for r in nn),
                **result,
                same_seed_numerator=num,
                same_seed_denominator=den,
                same_seed_auc=num / den if den else None,
                same_seed_pair_share=(
                    den / result["denominator"] if result["denominator"] else None
                ),
                denominator_share=(
                    result["denominator"] / full["denominator"]
                    if full["denominator"]
                    else None
                ),
                numerator_contribution=(
                    result["numerator"] / full["denominator"]
                    if full["denominator"]
                    else None
                ),
            )
            key = (*group, score, label, population, pos, neg)
            fields(blocks[key], record)
            expected_keys[3].add(key)
            pieces.append(record)
            defined_auc += result["auc"] is not None
        for field in PAIR_FIELDS:
            check(full[field], math.fsum(b[field] for b in pieces))
for actual, expected, size in zip(
    [components, rankings, seed_support, blocks],
    expected_keys,
    [960, 8640, 2880, 19440],
):
    assert set(actual) == expected and len(actual) == size

plotted = read(PLOT / "ranking-values.csv")
expected_plot = {
    k
    for k, r in rankings.items()
    if r["score"] == "residual_sigma"
    and r["label"] in LABELS[:2]
    and r["stratum"] in ["filter_single", "fusion_three"]
}
assert {tuple(r[k] for k in NEWRANK) for r in plotted} == expected_plot and len(
    plotted
) == 768
for r in plotted:
    fields(r, rankings[tuple(r[k] for k in NEWRANK)])
support_values = read(PLOT / "support-values.csv")
assert len(support_values) == 60
for r in support_values:
    ff = [
        f
        for f in expected_family_support.values()
        if all(f[k] == r[k] for k in ["partition", "noise", "variant", "stratum"])
    ]
    matched = [f for f in ff if f["common_family"]]
    available = [f for f in ff if f["source_records"]]
    expected = {k: r[k] for k in ["partition", "noise", "variant", "stratum"]}
    expected.update(
        planned_families=len(ff),
        represented_families=len(available),
        common_families=len(matched),
        source_records=sum(f["source_records"] for f in ff),
        common_records=sum(f["source_records"] for f in matched),
        common_family_share=len(matched) / len(available) if available else None,
    )
    for prefix in NAMES.values():
        for scope, rr in [("source", ff), ("common", matched)]:
            expected[scope + "_" + prefix + "_records"] = sum(
                f[prefix + "_records"] for f in rr
            )
            expected[scope + "_" + prefix + "_case_mass"] = math.fsum(
                f[prefix + "_case_mass"] for f in rr
            )
    fields(r, expected)
block_values = read(PLOT / "block-values.csv")
assert len(block_values) == 18
for r in block_values:
    key = tuple(
        r[k] for k in [*RANK, "population", "positive_planner", "negative_planner"]
    )
    fields(r, blocks[key])
meta = json.loads((PLOT / "plot-sources.json").read_text())
check(sha(ROOT / "plot_planner_conditioned_rankings.py"), meta["script_sha256"])
check(sha(DATA / "sources.json"), meta["analysis_sources_sha256"])
for name, h in meta["inputs_sha256"].items():
    check(sha(DATA / name), h)
for name, h in meta["outputs_sha256"].items():
    check(sha(PLOT / name), h)
evidence = dict(
    comparisons=checks,
    reconstructed_states=len(focus),
    reconstructed_targets=len(all_data),
    case_rows=len(cases_actual),
    family_rows=len(families_actual),
    family_support_rows=len(family_support),
    component_rows=len(components),
    ranking_rows=len(rankings),
    planner_block_rows=len(blocks),
    seed_support_rows=len(seed_support),
    defined_full_or_block_auc_checks=defined_auc,
    defined_same_seed_auc_checks=defined_same,
    plotted_rows=len(plotted) + len(support_values) + len(block_values),
    script_sha256=sha(Path(__file__)),
    sources_sha256=sha(DATA / "sources.json"),
)
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(evidence, indent=2) + "\n")
print(json.dumps(evidence, indent=2))
