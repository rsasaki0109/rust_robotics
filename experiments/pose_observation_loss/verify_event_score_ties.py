"""Independent NumPy vectors and direct weighted pairs; no analyzer imports."""

import argparse
import csv
from collections import Counter, defaultdict
from fractions import Fraction
from decimal import Decimal, ROUND_HALF_EVEN
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
NEW, DATA, PLOT = args.results, RESULTS / "event-temporal-rankings", args.figures
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


new_sources = json.loads((NEW / "sources.json").read_text())
new_base_sha = "19d64587f8fb0c5893ecf21312ed38a7f9261ab3898c9c6fc7fd1c565d0f6e80"
check(sha(DATA / "sources.json"), new_base_sha)
check(new_sources["input_sha256"]["event-temporal-rankings/sources.json"], new_base_sha)
for name, h in new_sources["input_sha256"].items():
    check(sha(RESULTS / name), h)
for name, h in new_sources["analysis"]["helpers_sha256"].items():
    check(sha(ROOT / name), h)
check(
    sha(ROOT / "analyze_event_score_ties.py"), new_sources["analysis"]["script_sha256"]
)
check(
    sha(RESULTS / "event-score-ties/protocol.md"),
    new_sources["analysis"]["protocol_sha256"],
)
for name, h in new_sources["output_sha256"].items():
    check(sha(NEW / name), h)


def new_indexed(name, keys):
    rr = read(NEW / name)
    result = {tuple(r[k] for k in keys): r for r in rr}
    assert len(result) == len(rr)
    return result


sources = json.loads((DATA / "sources.json").read_text())
for name, digest in sources["input_sha256"].items():
    check(sha(RESULTS / name), digest)
for name, digest in sources["analysis"]["helpers_sha256"].items():
    check(sha(ROOT / name), digest)
check(
    sha(ROOT / "analyze_event_temporal_rankings.py"),
    sources["analysis"]["script_sha256"],
)
check(
    sha(RESULTS / "event-temporal-rankings/protocol.md"),
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
POPULATIONS = ["source", "common_family"]
NEWGROUP = [*GROUP, "population", "planner_selection"]
NEWRANK = [*NEWGROUP, "score", "label"]


def source_support(rr):
    return dict(
        **support(rr), families=len({case_info[r["case_key"]]["family_id"] for r in rr})
    )


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


def state_bins(rr, score, label):
    """Direct state-pair masks; no histograms or collector category function."""
    pp = [r for r in rr if r["outside"][label]]
    nn = [r for r in rr if not r["outside"][label]]

    def values(rows, field):
        if field == "seed":
            return np.array([r[field] for r in rows], dtype=object)
        return np.array([case_info[r["case_key"]][field] for r in rows], dtype=object)

    same = {
        field: values(pp, field)[:, None] == values(nn, field)[None, :]
        for field in ["case_id", "window_id", "family_id", "seed"]
    }
    masks = [
        same["case_id"],
        same["window_id"] & ~same["case_id"],
        same["family_id"] & ~same["window_id"],
        same["seed"] & ~same["family_id"],
        ~same["seed"],
    ]
    assert np.all(sum(mask.astype(int) for mask in masks) == 1)
    assert np.all(~same["case_id"] | same["window_id"])
    assert np.all(~same["window_id"] | same["family_id"])
    assert np.all(~same["family_id"] | same["seed"])
    p, n = np.array([r[score + "_tick"] for r in pp], dtype=np.int64), np.array(
        [r[score + "_tick"] for r in nn], dtype=np.int64
    )
    mass = (
        np.array([r["weight"] for r in pp])[:, None]
        * np.array([r["weight"] for r in nn])[None, :]
    )
    greater, equal = p[:, None] > n[None, :], p[:, None] == n[None, :]
    results = []
    for mask in masks:
        den = float(np.sum(mass[mask]))
        con = float(np.sum(mass[mask & greater]))
        tied = float(np.sum(mass[mask & equal]))
        num = con + tied / 2
        results.append(
            dict(
                concordant_pair_mass=con,
                tied_pair_mass=tied,
                discordant_pair_mass=den - con - tied,
                numerator=num,
                denominator=den,
                auc=num / den if den else None,
                candidate_case_pairs=0,
                supported_case_pairs=0,
            )
        )
    active = {r["case_key"] for r in rr}
    positive_cases = {r["case_key"] for r in pp}
    negative_cases = {r["case_key"] for r in nn}
    for pkey, nkey in product(active, repeat=2):
        pinfo, ninfo = case_info[pkey], case_info[nkey]
        equality = [
            pinfo[field] == ninfo[field]
            for field in ["case_id", "window_id", "family_id", "seed"]
        ]
        index = next((i for i, equal in enumerate(equality) if equal), 4)
        results[index]["candidate_case_pairs"] += 1
        results[index]["supported_case_pairs"] += (
            pkey in positive_cases and nkey in negative_cases
        )
    return results


def indexed(name, keys):
    rr = read(DATA / name)
    result = {tuple(r[k] for k in keys): r for r in rr}
    assert len(result) == len(rr)
    return result


for name, original in [
    ("cases.csv", "planner-window-sensitivity/cases.csv"),
    ("families.csv", "planner-window-sensitivity/families.csv"),
    ("family-support.csv", "planner-conditioned-rankings/family-support.csv"),
    ("case-strata.csv", "case-window-rank-decomposition/case-strata.csv"),
]:
    check((DATA / name).read_bytes() == (RESULTS / original).read_bytes(), True)
for name, h in sources["output_sha256"].items():
    check(sha(DATA / name), h)
CASE_META = ["case_id", "family_id", *KEY, "partition", "accounting_partition", "noise"]
EVENT_META = [
    "event_id",
    "source_event_id",
    "event_kind",
    "event_time_s",
    "truth_time_s",
    "scheduled",
    "received",
    "precision_activation",
]
family_support = indexed("family-support.csv", ["family_id", "stratum"])
eligibility = defaultdict(set)
for family_id, f in expected_families.items():
    parent_rows = [
        r
        for r in all_data
        if r["target"] == "midpoint"
        and case_info[r["case_key"]]["family_id"] == family_id
    ]
    for mode in STRATA:
        rr = [r for r in parent_rows if mode == "all" or r["stratum"] == mode]
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
        for planner, prefix in zip(PLANNERS, ["astar", "jps", "theta"]):
            selected = [r for r in rr if r["planner"] == planner]
            expected[prefix + "_records"] = len(selected)
            expected[prefix + "_case_mass"] = math.fsum(r["weight"] for r in selected)
        fields(family_support[family_id, mode], expected)

mapping = indexed("source-event-map.csv", ["case_id", "source_id"])
by_event_map = defaultdict(list)
expected_mapping = set()
for origin, m, p, s, poses in focus:
    key = casekey(m)
    info = case_info[key]
    tags = m["source_event_ids"].split("+")
    assert len(tags) == 1 and tags[0] in events
    event = events[tags[0]]
    assert casekey(event) == key
    partition = "new" if event["partition"] == "existing_new" else event["partition"]
    check(partition, m["partition"])
    check(event["noise"], m["noise"])
    expected = {k: info[k] for k in CASE_META}
    time = float(p["time_s"])
    expected.update(
        source_id=m["source_id"],
        event_id=tags[0],
        stratum=m["stratum"],
        time_s=time,
        truth_time_s=float(poses[0]["truth_time_s"]),
        event_time_s=float(event["event_time_s"]),
        event_truth_time_s=float(event["truth_time_s"]),
        event_kind=event["event_kind"],
        event_age_s=time - float(event["event_time_s"]),
        case_row_weight=Fraction(1, counts[key]),
    )
    mapkey = info["case_id"], m["source_id"]
    fields(mapping[mapkey], expected)
    expected_mapping.add(mapkey)
    by_event_map[tags[0]].append(expected)
assert set(mapping) == expected_mapping and len(mapping) == 10995
# Times come from original projection strings, independently of exported maps.
base_sources = RESULTS / "event-conditioned-rankings/sources.json"
base_sha = "1e0656c67622cd7f673fd26e75af7bc84774d21491334c7aac94358c94611349"
check(sha(base_sources), base_sha)
check(sources["input_sha256"]["event-conditioned-rankings/sources.json"], base_sha)
for name in json.loads(base_sources.read_text())["output_sha256"]:
    if name != "ranking-summary.csv":
        check(
            (DATA / name).read_bytes() == (base_sources.parent / name).read_bytes(),
            True,
        )
time_audit = indexed("source-time-audit.csv", ["case_id", "source_id"])
independent_times = {}
for origin, m, p, s, poses in focus:
    key = casekey(m)
    saved = Decimal(p["time_s"])
    assert saved.is_finite()
    tick = int((saved * Decimal(1000000)).to_integral_value(rounding=ROUND_HALF_EVEN))
    assert abs(Decimal(tick) / Decimal(1000000) - saved) <= Decimal("1e-7")
    independent_times[key, m["source_id"]] = tick
    expected = dict(
        case_id=case_info[key]["case_id"],
        source_id=m["source_id"],
        time_s=float(saved),
        time_us=tick,
        reconstruction_error_s=tick / 1000000 - float(saved),
    )
    fields(time_audit[case_info[key]["case_id"], m["source_id"]], expected)
assert len(time_audit) == len(independent_times) == 10995
for r in all_data:
    r["_time_us"] = independent_times[r["case_key"], r["source_id"]]

score_audit = new_indexed("score-audit.csv", ["case_id", "source_id"])
expected_score_audit = set()
decimal_values = {score: set() for score in SCORES}
for origin, m, p, s, poses in focus:
    for score in SCORES:
        native = Decimal(s[score + "_score_m"])
        assert native.is_finite()
        decimal_values[score].add(native)
# Establish that all distinct consumed Decimal values remain strictly ordered in
# binary64, rather than assuming that distinct export strings imply distinct floats.
decimal_order = {}
for score in SCORES:
    ordered = sorted(decimal_values[score])
    parsed = [float(value) for value in ordered]
    assert all(math.isfinite(value) for value in parsed)
    assert all(a < b for a, b in zip(parsed, parsed[1:])), score
    decimal_order[score] = {value: i for i, value in enumerate(ordered)}
score_orders = {}
for origin, m, p, s, poses in focus:
    key = casekey(m)
    expected = dict(
        case_id=case_info[key]["case_id"],
        source_id=m["source_id"],
        event_id=m["source_event_ids"],
        score_source=origin,
    )
    order = {}
    for score in SCORES:
        raw = s[score + "_score_m"]
        value = float(raw)
        tick = int(s[score + "_tick"])
        assert round(value / 1e-8) == tick
        expected.update(
            {
                score + "_score_m": raw,
                score + "_score_hex": value.hex(),
                score + "_tick": tick,
            }
        )
        order[score] = decimal_order[score][Decimal(raw)]
    map_key = case_info[key]["case_id"], m["source_id"]
    fields(score_audit[map_key], expected)
    expected_score_audit.add(map_key)
    score_orders[key, m["source_id"]] = order
assert set(score_audit) == expected_score_audit and len(score_audit) == 10995
for r in all_data:
    r["_native_order"] = score_orders[r["case_key"], r["source_id"]]

by_event_target, by_case_target = defaultdict(list), defaultdict(list)
for r in all_data:
    tags = r["source_event_ids"].split("+")
    assert len(tags) == 1
    r["_event_id"] = tags[0]
    by_event_target[tags[0], r["target"]].append(r)
    by_case_target[r["case_key"], r["target"]].append(r)

coverage = indexed("event-coverage.csv", ["event_id"])
event_support = indexed("event-support.csv", ["event_id", "stratum"])
event_ranks = indexed(
    "event-rankings.csv", ["event_id", "stratum", "target", "score", "label"]
)
expected_coverage, expected_support, expected_event_ranks = set(), set(), set()
for eid, event in events.items():
    info = case_info[casekey(event)]
    metadata = {
        **{k: info[k] for k in CASE_META},
        **{k: event[k] for k in EVENT_META},
        "event_partition": event["partition"],
        "parent_case_records": info["parent_records"],
        "case_row_weight": info["case_row_weight"],
    }
    rr = by_event_map[eid]
    times = [r["time_s"] for r in rr]
    expected = dict(
        **metadata,
        records=len(rr),
        strata="+".join(sorted({r["stratum"] for r in rr})),
        min_time_s=min(times) if times else None,
        max_time_s=max(times) if times else None,
        min_event_age_s=min(times) - float(event["event_time_s"]) if times else None,
        max_event_age_s=max(times) - float(event["event_time_s"]) if times else None,
    )
    fields(coverage[eid,], expected)
    expected_coverage.add((eid,))
    if not rr:
        continue
    for mode in STRATA:
        ss = [r for r in rr if mode == "all" or r["stratum"] == mode]
        common = dict(
            **metadata,
            stratum=mode,
            parent_event_records=len(rr),
            records=len(ss),
            case_mass=len(ss) / counts[casekey(event)],
            common_family_eligible=info["family_id"] in eligibility[mode],
        )
        expected = dict(common)
        for target, label in product(["midpoint", "residual"], LABELS):
            rows = [
                r
                for r in by_event_target[eid, target]
                if mode == "all" or r["stratum"] == mode
            ]
            pp, nn = [r for r in rows if r["outside"][label]], [
                r for r in rows if not r["outside"][label]
            ]
            prefix = target + "_" + label
            expected[prefix + "_outside_rows"] = len(pp)
            expected[prefix + "_inside_rows"] = len(nn)
            expected[prefix + "_outside_case_mass"] = math.fsum(r["weight"] for r in pp)
            expected[prefix + "_inside_case_mass"] = math.fsum(r["weight"] for r in nn)
        fields(event_support[eid, mode], expected)
        expected_support.add((eid, mode))
        if not ss:
            continue
        for target, score, label in product(["midpoint", "residual"], SCORES, LABELS):
            rows = [
                r
                for r in by_event_target[eid, target]
                if mode == "all" or r["stratum"] == mode
            ]
            pp, nn = [r for r in rows if r["outside"][label]], [
                r for r in rows if not r["outside"][label]
            ]
            expected = dict(
                **common,
                target=target,
                score=score,
                label=label,
                outside_rows=len(pp),
                inside_rows=len(nn),
                outside_case_mass=math.fsum(r["weight"] for r in pp),
                inside_case_mass=math.fsum(r["weight"] for r in nn),
                **direct_pairs(pp, nn, score),
            )
            key = eid, mode, target, score, label
            fields(event_ranks[key], expected)
            expected_event_ranks.add(key)
assert set(coverage) == expected_coverage and len(coverage) == 3997
assert set(event_support) == expected_support and len(event_support) == 1375
assert set(event_ranks) == expected_event_ranks and len(event_ranks) == 9900

case_strata = indexed("case-strata.csv", ["case_id", "stratum"])
for key, info in case_info.items():
    for mode in STRATA:
        rr = [
            r
            for r in by_case_target[key, "midpoint"]
            if mode == "all" or r["stratum"] == mode
        ]
        expected = {
            k: info[k] for k in [*CASE_META, "parent_records", "case_row_weight"]
        }
        expected.update(
            stratum=mode,
            records=len(rr),
            case_mass=math.fsum(r["weight"] for r in rr),
            common_family_eligible=info["family_id"] in eligibility[mode],
        )
        for target, label in product(["midpoint", "residual"], LABELS):
            rows = [
                r
                for r in by_case_target[key, target]
                if mode == "all" or r["stratum"] == mode
            ]
            pp, nn = [r for r in rows if r["outside"][label]], [
                r for r in rows if not r["outside"][label]
            ]
            prefix = target + "_" + label
            expected[prefix + "_outside_rows"] = len(pp)
            expected[prefix + "_inside_rows"] = len(nn)
            expected[prefix + "_outside_case_mass"] = math.fsum(r["weight"] for r in pp)
            expected[prefix + "_inside_case_mass"] = math.fsum(r["weight"] for r in nn)
        fields(case_strata[info["case_id"], mode], expected)
assert len(case_strata) == 4860


def event_pair_masks(rr, score, label):
    pp, nn = [r for r in rr if r["outside"][label]], [
        r for r in rr if not r["outside"][label]
    ]
    pcase = np.array([case_info[r["case_key"]]["case_id"] for r in pp], dtype=object)
    ncase = np.array([case_info[r["case_key"]]["case_id"] for r in nn], dtype=object)
    pevent = np.array([r["_event_id"] for r in pp], dtype=object)
    nevent = np.array([r["_event_id"] for r in nn], dtype=object)
    case_match = pcase[:, None] == ncase[None, :]
    event_match = pevent[:, None] == nevent[None, :]
    masks = [case_match & event_match, case_match & ~event_match, ~case_match]
    assert np.all(sum(mask.astype(int) for mask in masks) == 1)
    mass = (
        np.array([r["weight"] for r in pp])[:, None]
        * np.array([r["weight"] for r in nn])[None, :]
    )
    p, n = np.array([r[score + "_tick"] for r in pp], dtype=np.int64), np.array(
        [r[score + "_tick"] for r in nn], dtype=np.int64
    )
    greater, equal = p[:, None] > n[None, :], p[:, None] == n[None, :]
    results = []
    for mask in masks:
        den = float(np.sum(mass[mask]))
        con = float(np.sum(mass[mask & greater]))
        tied = float(np.sum(mass[mask & equal]))
        num = con + tied / 2
        results.append(
            dict(
                concordant_pair_mass=con,
                tied_pair_mass=tied,
                discordant_pair_mass=den - con - tied,
                numerator=num,
                denominator=den,
                auc=num / den if den else None,
            )
        )
    return results


TIME_BINS = ["le_20ms", "20_to_100ms", "100_to_500ms", "gt_500ms"]
TIME_SCOPES = ["le_20ms", "le_100ms", "le_500ms"]


def time_pair_masks(rr, score, label):
    positive = [r for r in rr if r["outside"][label]]
    negative = [r for r in rr if not r["outside"][label]]
    pcase = np.array(
        [case_info[r["case_key"]]["case_id"] for r in positive], dtype=object
    )
    ncase = np.array(
        [case_info[r["case_key"]]["case_id"] for r in negative], dtype=object
    )
    pevent = np.array([r["_event_id"] for r in positive], dtype=object)
    nevent = np.array([r["_event_id"] for r in negative], dtype=object)
    event_match = (pcase[:, None] == ncase[None, :]) & (
        pevent[:, None] == nevent[None, :]
    )
    ptime = np.array([r["_time_us"] for r in positive], dtype=np.int64)
    ntime = np.array([r["_time_us"] for r in negative], dtype=np.int64)
    gap = np.abs(ptime[:, None] - ntime[None, :])
    masks = [
        event_match & (gap <= 20000),
        event_match & (gap > 20000) & (gap <= 100000),
        event_match & (gap > 100000) & (gap <= 500000),
        event_match & (gap > 500000),
    ]
    assert np.all(sum(mask.astype(int) for mask in masks) == event_match.astype(int))
    mass = (
        np.array([r["weight"] for r in positive])[:, None]
        * np.array([r["weight"] for r in negative])[None, :]
    )
    pscore = np.array([r[score + "_tick"] for r in positive], dtype=np.int64)
    nscore = np.array([r[score + "_tick"] for r in negative], dtype=np.int64)
    greater, equal = (
        pscore[:, None] > nscore[None, :],
        pscore[:, None] == nscore[None, :],
    )
    full_den, event_den = float(np.sum(mass)), float(np.sum(mass[event_match]))

    def summarize(mask):
        den = float(np.sum(mass[mask]))
        con = float(np.sum(mass[mask & greater]))
        tied = float(np.sum(mass[mask & equal]))
        num = con + tied / 2
        positions = np.flatnonzero(np.any(mask, axis=1))
        events = {
            (positive[i]["case_key"], positive[i]["_event_id"]) for i in positions
        }
        return dict(
            concordant_pair_mass=con,
            tied_pair_mass=tied,
            discordant_pair_mass=den - con - tied,
            numerator=num,
            denominator=den,
            auc=num / den if den else None,
            pair_count=int(np.count_nonzero(mask)),
            events_with_pairs=len(events),
            cases_with_pairs=len({case for case, event in events}),
            full_pair_share=den / full_den if full_den else None,
            event_pair_share=den / event_den if event_den else None,
            numerator_full_contribution=num / full_den if full_den else None,
        )

    pieces = [summarize(mask) for mask in masks]
    extra = {"same_event_pair_count": int(np.count_nonzero(event_match))}
    for i, scope in enumerate(TIME_SCOPES, 1):
        mask = np.logical_or.reduce(masks[:i])
        extra.update({scope + "_" + k: value for k, value in summarize(mask).items()})
    return extra, pieces


time_support = indexed(
    "event-time-support.csv", ["event_id", "stratum", "target", "label"]
)
expected_time_support = set()
for (eid, target), parent_rows in by_event_target.items():
    info = case_info[parent_rows[0]["case_key"]]
    for mode, label in product(STRATA, LABELS):
        rr = [r for r in parent_rows if mode == "all" or r["stratum"] == mode]
        if not rr:
            continue
        positive = [r for r in rr if r["outside"][label]]
        negative = [r for r in rr if not r["outside"][label]]
        _, pieces = time_pair_masks(rr, "contrast", label)
        expected = {k: info[k] for k in CASE_META}
        expected.update(
            event_id=eid,
            stratum=mode,
            target=target,
            label=label,
            common_family_eligible=info["family_id"] in eligibility[mode],
            records=len(rr),
            outside_rows=len(positive),
            inside_rows=len(negative),
            outside_case_mass=math.fsum(r["weight"] for r in positive),
            inside_case_mass=math.fsum(r["weight"] for r in negative),
            min_time_us=min(r["_time_us"] for r in rr),
            max_time_us=max(r["_time_us"] for r in rr),
        )
        for name, piece in zip(TIME_BINS, pieces):
            expected[name + "_pair_count"] = piece["pair_count"]
            expected[name + "_denominator"] = piece["denominator"]
        fields(time_support[eid, mode, target, label], expected)
        expected_time_support.add((eid, mode, target, label))
assert set(time_support) == expected_time_support and len(time_support) == 3300

time_bins = indexed("temporal-pair-bins.csv", [*NEWRANK, "bin"])
expected_time_bins = set()
defined_time = {scope: 0 for scope in TIME_SCOPES}
defined_time_bins = 0

PRECISION_SCOPES = ["same_event", "le_500ms", "le_100ms", "le_20ms"]
SCORE_CLASSES = [
    "tick_concordant",
    "tick_discordant",
    "tick_tied_native_concordant",
    "tick_tied_native_equal",
    "tick_tied_native_discordant",
]
precision_summary = new_indexed("precision-summary.csv", [*NEWRANK, "scope"])
precision_bins = new_indexed("precision-bins.csv", [*NEWRANK, "bin"])
expected_precision_summary, expected_precision_bins = set(), set()
defined_unequal = defined_native_unequal = 0


def precision_masks(rr, score, label):
    positive = [r for r in rr if r["outside"][label]]
    negative = [r for r in rr if not r["outside"][label]]
    pcase = np.array(
        [case_info[r["case_key"]]["case_id"] for r in positive], dtype=object
    )
    ncase = np.array(
        [case_info[r["case_key"]]["case_id"] for r in negative], dtype=object
    )
    pevent = np.array([r["_event_id"] for r in positive], dtype=object)
    nevent = np.array([r["_event_id"] for r in negative], dtype=object)
    same_event = (pcase[:, None] == ncase[None, :]) & (
        pevent[:, None] == nevent[None, :]
    )
    ptime = np.array([r["_time_us"] for r in positive], dtype=np.int64)
    ntime = np.array([r["_time_us"] for r in negative], dtype=np.int64)
    gap = np.abs(ptime[:, None] - ntime[None, :])
    time_masks = [
        same_event & (gap <= 20000),
        same_event & (gap > 20000) & (gap <= 100000),
        same_event & (gap > 100000) & (gap <= 500000),
        same_event & (gap > 500000),
    ]
    ptick = np.array([r[score + "_tick"] for r in positive], dtype=np.int64)
    ntick = np.array([r[score + "_tick"] for r in negative], dtype=np.int64)
    pnative = np.array([r["_native_order"][score] for r in positive], dtype=np.int64)
    nnative = np.array([r["_native_order"][score] for r in negative], dtype=np.int64)
    greater, equal, less = (
        ptick[:, None] > ntick[None, :],
        ptick[:, None] == ntick[None, :],
        ptick[:, None] < ntick[None, :],
    )
    native_greater, native_equal, native_less = (
        pnative[:, None] > nnative[None, :],
        pnative[:, None] == nnative[None, :],
        pnative[:, None] < nnative[None, :],
    )
    assert not np.any(greater & ~native_greater)
    assert not np.any(less & ~native_less)
    classes = [
        greater,
        less,
        equal & native_greater,
        equal & native_equal,
        equal & native_less,
    ]
    assert np.all(sum(mask.astype(int) for mask in classes) == 1)
    mass = (
        np.array([r["weight"] for r in positive])[:, None]
        * np.array([r["weight"] for r in negative])[None, :]
    )
    full_den, event_den = float(np.sum(mass)), float(np.sum(mass[same_event]))

    def supported(mask):
        indices = np.flatnonzero(np.any(mask, axis=1))
        events = {(positive[i]["case_key"], positive[i]["_event_id"]) for i in indices}
        return len(events), len({key for key, event in events})

    def summarize(mask):
        counts = [int(np.count_nonzero(mask & category)) for category in classes]
        m = [float(np.sum(mass[mask & category])) for category in classes]
        con, dis, resolved_con, native_tied, resolved_dis = m
        tied = resolved_con + native_tied + resolved_dis
        den, num = sum(m), con + tied / 2
        native_con, native_dis = con + resolved_con, dis + resolved_dis
        native_num = native_con + native_tied / 2
        auc = num / den if den else None
        native_auc = native_num / den if den else None
        delta = native_auc - auc if den else None
        events, cases = supported(mask)
        tick_events, tick_cases = supported(mask & ~equal)
        native_events, native_cases = supported(mask & ~native_equal)
        result = dict(
            concordant_pair_mass=con,
            tied_pair_mass=tied,
            discordant_pair_mass=dis,
            numerator=num,
            denominator=den,
            auc=auc,
            pair_count=sum(counts),
            events_with_pairs=events,
            cases_with_pairs=cases,
            full_pair_share=den / full_den if full_den else None,
            event_pair_share=den / event_den if event_den else None,
            numerator_full_contribution=num / full_den if full_den else None,
            native_concordant_pair_mass=native_con,
            native_tied_pair_mass=native_tied,
            native_discordant_pair_mass=native_dis,
            native_numerator=native_num,
            native_auc=native_auc,
            native_auc_delta=delta,
            tick_tied_native_concordant_mass=resolved_con,
            tick_tied_native_discordant_mass=resolved_dis,
            tick_tied_share=tied / den if den else None,
            native_tied_share=native_tied / den if den else None,
            resolved_tick_tie_share=(
                (resolved_con + resolved_dis) / den if den else None
            ),
            tick_unequal_concordance=con / (con + dis) if con + dis else None,
            native_unequal_concordance=(
                native_con / (native_con + native_dis)
                if native_con + native_dis
                else None
            ),
            tick_unequal_pair_count=counts[0] + counts[1],
            native_unequal_pair_count=counts[0] + counts[1] + counts[2] + counts[4],
            tick_unequal_events=tick_events,
            tick_unequal_cases=tick_cases,
            native_unequal_events=native_events,
            native_unequal_cases=native_cases,
        )
        result.update(
            {name + "_pair_count": n for name, n in zip(SCORE_CLASSES, counts)}
        )
        if den:
            check(delta, (resolved_con - resolved_dis) / (2 * den))
        return result

    return (
        [
            summarize(mask)
            for mask in [
                same_event,
                same_event & (gap <= 500000),
                same_event & (gap <= 100000),
                same_event & (gap <= 20000),
            ]
        ],
        [summarize(mask) for mask in time_masks],
    )


rankings = indexed("ranking-summary.csv", NEWRANK)
bins = indexed("event-pair-bins.csv", [*NEWRANK, "bin"])
expected_ranks, expected_bins = set(), set()
defined_full = defined_event = defined_bin = 0
for partition, noise, arm, mode, target, population in product(
    SEEDS,
    ["white", "correlated"],
    ["epoch", "curved"],
    STRATA,
    ["midpoint", "residual"],
    POPULATIONS,
):
    parent_rows = [
        r
        for r in all_data
        if (r["partition"], r["noise"], r["variant"], r["target"])
        == (partition, noise, arm, target)
    ]
    parent_mass = math.fsum(r["weight"] for r in parent_rows)
    rr = [
        r
        for r in parent_rows
        if (mode == "all" or r["stratum"] == mode)
        and (
            population == "source"
            or case_info[r["case_key"]]["family_id"] in eligibility[mode]
        )
    ]
    key = partition, noise, arm, mode, target, population, "all"
    common = dict(zip(NEWGROUP, key))
    common.update(
        **source_support(rr),
        group_case_mass=parent_mass,
        case_mass_share=(
            math.fsum(r["weight"] for r in rr) / parent_mass if parent_mass else None
        ),
    )
    for score, label in product(SCORES, LABELS):
        metrics = ranking_metrics(rr, score, label, SEEDS[partition])
        old = state_bins(rr, score, label)
        extra = {}
        for name, indices in [
            ("same_case", [0]),
            ("same_window", [0, 1]),
            ("same_family", [0, 1, 2]),
            ("between_case", [1, 2, 3, 4]),
        ]:
            num = math.fsum(old[i]["numerator"] for i in indices)
            den = math.fsum(old[i]["denominator"] for i in indices)
            extra[name + "_numerator"] = num
            extra[name + "_denominator"] = den
            extra[name + "_auc"] = num / den if den else None
            extra[name + "_pair_share"] = (
                den / metrics["denominator"] if metrics["denominator"] else None
            )
        for unit, field in [
            ("cases", "case_id"),
            ("windows", "window_id"),
            ("families", "family_id"),
        ]:
            pp = {case_info[r["case_key"]][field] for r in rr if r["outside"][label]}
            nn = {
                case_info[r["case_key"]][field] for r in rr if not r["outside"][label]
            }
            extra[unit + "_with_both_classes"] = len(pp & nn)
        pieces = event_pair_masks(rr, score, label)
        for field in PAIR_FIELDS:
            check(metrics[field], math.fsum(b[field] for b in pieces))
            check(old[0][field], math.fsum(b[field] for b in pieces[:2]))
        positive = {(r["case_key"], r["_event_id"]) for r in rr if r["outside"][label]}
        negative = {
            (r["case_key"], r["_event_id"]) for r in rr if not r["outside"][label]
        }
        extra["events"] = len(positive | negative)
        extra["events_with_both_classes"] = len(positive & negative)
        extra["cases_with_event_support"] = len(
            {case for case, event in positive & negative}
        )
        for name, piece in zip(
            ["same_event", "other_event_same_case", "other_case"], pieces
        ):
            share = (
                piece["denominator"] / metrics["denominator"]
                if metrics["denominator"]
                else None
            )
            within = (
                piece["denominator"] / old[0]["denominator"]
                if name != "other_case" and old[0]["denominator"]
                else None
            )
            if name != "other_case":
                for field in ["numerator", "denominator", "auc"]:
                    extra[name + "_" + field] = piece[field]
                extra[name + "_pair_share"] = share
                extra[name + "_within_case_share"] = within
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(
                bin=name,
                **piece,
                denominator_share=share,
                numerator_contribution=(
                    piece["numerator"] / metrics["denominator"]
                    if metrics["denominator"]
                    else None
                ),
                within_case_share=within,
            )
            fields(bins[*key, score, label, name], expected)
            expected_bins.add((*key, score, label, name))
            defined_bin += piece["auc"] is not None
        time_extra, time_pieces = time_pair_masks(rr, score, label)
        for field in PAIR_FIELDS:
            check(pieces[0][field], math.fsum(b[field] for b in time_pieces))
        for name, piece in zip(TIME_BINS, time_pieces):
            expected_time = dict(zip(NEWRANK, (*key, score, label)))
            expected_time.update(bin=name, **piece)
            fields(time_bins[*key, score, label, name], expected_time)
            expected_time_bins.add((*key, score, label, name))
            defined_time_bins += piece["auc"] is not None
        extra.update(time_extra)
        for scope in TIME_SCOPES:
            defined_time[scope] += extra[scope + "_auc"] is not None
        scope_values, bin_values = precision_masks(rr, score, label)
        for field in PAIR_FIELDS:
            check(pieces[0][field], scope_values[0][field])
            check(pieces[0][field], math.fsum(b[field] for b in bin_values))
        for scope, values in zip(PRECISION_SCOPES, scope_values):
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(scope=scope, **values)
            fields(precision_summary[*key, score, label, scope], expected)
            expected_precision_summary.add((*key, score, label, scope))
            defined_unequal += values["tick_unequal_concordance"] is not None
            defined_native_unequal += values["native_unequal_concordance"] is not None
        for name, values in zip(TIME_BINS, bin_values):
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(bin=name, **values)
            fields(precision_bins[*key, score, label, name], expected)
            expected_precision_bins.add((*key, score, label, name))
        fields(
            rankings[*key, score, label],
            dict(**common, score=score, label=label, **metrics, **extra),
        )
        expected_ranks.add((*key, score, label))
        defined_full += metrics["auc"] is not None
        defined_event += extra["same_event_auc"] is not None
assert set(rankings) == expected_ranks and len(rankings) == 2160
assert set(bins) == expected_bins and len(bins) == 6480

assert set(time_bins) == expected_time_bins and len(time_bins) == 8640

assert (
    set(precision_summary) == expected_precision_summary
    and len(precision_summary) == 8640
)
assert set(precision_bins) == expected_precision_bins and len(precision_bins) == 8640
plot_rows = 0
for name, values, keys in [
    ("summary-values.csv", precision_summary, [*NEWRANK, "scope"]),
    ("bin-values.csv", precision_bins, [*NEWRANK, "bin"]),
]:
    plotted = read(PLOT / name)
    expected = {
        k
        for k, r in values.items()
        if r["score"] == "residual_sigma"
        and r["label"] in LABELS[:2]
        and r["stratum"] in ["filter_single", "fusion_three"]
    }
    assert {tuple(r[k] for k in keys) for r in plotted} == expected
    assert len(plotted) == len(expected)
    for r in plotted:
        fields(r, values[tuple(r[k] for k in keys)])
    plot_rows += len(plotted)
assert plot_rows == 1536
meta = json.loads((PLOT / "plot-sources.json").read_text())
check(sha(ROOT / "plot_event_score_ties.py"), meta["script_sha256"])
check(sha(NEW / "sources.json"), meta["analysis_sources_sha256"])
for name, h in meta["inputs_sha256"].items():
    check(sha(NEW / name), h)
for name, h in meta["outputs_sha256"].items():
    check(sha(PLOT / name), h)
evidence = dict(
    comparisons=checks,
    reconstructed_states=len(focus),
    reconstructed_targets=len(all_data),
    case_rows=len(cases_actual),
    family_rows=len(families_actual),
    baseline_ranking_rows=len(rankings),
    baseline_time_bin_rows=len(time_bins),
    score_audit_rows=len(score_audit),
    saved_score_values=len(score_audit) * 3,
    distinct_saved_scores={score: len(v) for score, v in decimal_values.items()},
    precision_scope_rows=len(precision_summary),
    precision_bin_rows=len(precision_bins),
    defined_tick_unequal_scope_concordances=defined_unequal,
    defined_native_unequal_scope_concordances=defined_native_unequal,
    plotted_rows=plot_rows,
    script_sha256=sha(Path(__file__)),
    sources_sha256=sha(NEW / "sources.json"),
    plot_sources_sha256=sha(PLOT / "plot-sources.json"),
)
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(evidence, indent=2) + "\n")
print(json.dumps(evidence, indent=2))
