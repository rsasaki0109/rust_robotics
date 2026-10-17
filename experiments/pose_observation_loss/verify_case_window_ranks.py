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
    sha(ROOT / "analyze_case_window_ranks.py"),
    sources["analysis"]["script_sha256"],
)
check(
    sha(RESULTS / "case-window-rank-decomposition/protocol.md"),
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
BINS = [
    "same_case",
    "other_case_same_window",
    "other_window_same_family",
    "other_family_same_seed",
    "other_seed",
]
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


def indexed(name, fields):
    rr = read(DATA / name)
    lookup = {tuple(r[k] for k in fields): r for r in rr}
    assert len(lookup) == len(rr)
    return lookup


for name in ["cases.csv", "families.csv"]:
    check(
        (DATA / name).read_bytes()
        == (RESULTS / "planner-window-sensitivity" / name).read_bytes(),
        True,
    )
check(
    (DATA / "family-support.csv").read_bytes()
    == (RESULTS / "planner-conditioned-rankings/family-support.csv").read_bytes(),
    True,
)
for name, h in sources["output_sha256"].items():
    check(sha(DATA / name), h)

family_support = indexed("family-support.csv", ["family_id", "stratum"])
eligibility = defaultdict(set)
by_case_target = defaultdict(list)
for r in all_data:
    by_case_target[r["case_key"], r["target"]].append(r)
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

case_strata = indexed("case-strata.csv", ["case_id", "stratum"])
case_ranks = indexed(
    "case-rankings.csv", ["case_id", "stratum", "target", "score", "label"]
)
expected_case_strata, expected_case_ranks = set(), set()
for casekey_value, info in case_info.items():
    for mode in STRATA:
        rr = [
            r
            for r in by_case_target[casekey_value, "midpoint"]
            if mode == "all" or r["stratum"] == mode
        ]
        expected = {
            k: info[k]
            for k in [
                "case_id",
                "family_id",
                *KEY,
                "partition",
                "accounting_partition",
                "noise",
                "parent_records",
                "case_row_weight",
            ]
        }
        expected.update(
            stratum=mode,
            records=len(rr),
            case_mass=math.fsum(r["weight"] for r in rr),
            common_family_eligible=info["family_id"] in eligibility[mode],
        )
        for target, label in product(["midpoint", "residual"], LABELS):
            ss = [
                r
                for r in by_case_target[casekey_value, target]
                if mode == "all" or r["stratum"] == mode
            ]
            pp, nn = [r for r in ss if r["outside"][label]], [
                r for r in ss if not r["outside"][label]
            ]
            prefix = target + "_" + label
            expected[prefix + "_outside_rows"] = len(pp)
            expected[prefix + "_inside_rows"] = len(nn)
            expected[prefix + "_outside_case_mass"] = math.fsum(r["weight"] for r in pp)
            expected[prefix + "_inside_case_mass"] = math.fsum(r["weight"] for r in nn)
        key = info["case_id"], mode
        fields(case_strata[key], expected)
        expected_case_strata.add(key)
        if not info["parent_records"]:
            continue
        for target, score, label in product(["midpoint", "residual"], SCORES, LABELS):
            ss = [
                r
                for r in by_case_target[casekey_value, target]
                if mode == "all" or r["stratum"] == mode
            ]
            pp, nn = [r for r in ss if r["outside"][label]], [
                r for r in ss if not r["outside"][label]
            ]
            expected = {
                k: info[k]
                for k in [
                    "case_id",
                    "family_id",
                    "window_id",
                    *KEY,
                    "partition",
                    "noise",
                    "parent_records",
                    "case_row_weight",
                ]
            }
            expected.update(
                stratum=mode,
                target=target,
                score=score,
                label=label,
                common_family_eligible=info["family_id"] in eligibility[mode],
                records=len(ss),
                case_mass=math.fsum(r["weight"] for r in ss),
                outside_rows=len(pp),
                inside_rows=len(nn),
                outside_case_mass=math.fsum(r["weight"] for r in pp),
                inside_case_mass=math.fsum(r["weight"] for r in nn),
                **direct_pairs(pp, nn, score),
            )
            key = info["case_id"], mode, target, score, label
            fields(case_ranks[key], expected)
            expected_case_ranks.add(key)
assert set(case_strata) == expected_case_strata and len(case_strata) == 4860
assert set(case_ranks) == expected_case_ranks and len(case_ranks) == 14130


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


rankings = indexed("ranking-summary.csv", NEWRANK)
bins = indexed("hierarchy-bins.csv", [*NEWRANK, "bin"])
expected_ranks, expected_bins = set(), set()
defined_full = defined_case = defined_bin = 0
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
        pieces = state_bins(rr, score, label)
        for field in PAIR_FIELDS:
            check(metrics[field], math.fsum(b[field] for b in pieces))
        extra = {}
        for name, indices in [
            ("same_case", [0]),
            ("same_window", [0, 1]),
            ("same_family", [0, 1, 2]),
            ("between_case", [1, 2, 3, 4]),
        ]:
            num = math.fsum(pieces[i]["numerator"] for i in indices)
            den = math.fsum(pieces[i]["denominator"] for i in indices)
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
            positive = {
                case_info[r["case_key"]][field] for r in rr if r["outside"][label]
            }
            negative = {
                case_info[r["case_key"]][field] for r in rr if not r["outside"][label]
            }
            extra[unit + "_with_both_classes"] = len(positive & negative)
        record = dict(**common, score=score, label=label, **metrics, **extra)
        rankkey = (*key, score, label)
        fields(rankings[rankkey], record)
        expected_ranks.add(rankkey)
        defined_full += metrics["auc"] is not None
        defined_case += extra["same_case_auc"] is not None
        for name, piece in zip(BINS, pieces):
            expected = dict(zip(NEWRANK, rankkey))
            expected.update(
                bin=name,
                **piece,
                denominator_share=(
                    piece["denominator"] / metrics["denominator"]
                    if metrics["denominator"]
                    else None
                ),
                numerator_contribution=(
                    piece["numerator"] / metrics["denominator"]
                    if metrics["denominator"]
                    else None
                ),
            )
            fields(bins[*rankkey, name], expected)
            expected_bins.add((*rankkey, name))
            defined_bin += piece["auc"] is not None
assert set(rankings) == expected_ranks and len(rankings) == 2160
assert set(bins) == expected_bins and len(bins) == 10800

plot_rows = 0
for name, values, keys in [
    ("ranking-values.csv", rankings, NEWRANK),
    ("bin-values.csv", bins, [*NEWRANK, "bin"]),
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
assert plot_rows == 1152
meta = json.loads((PLOT / "plot-sources.json").read_text())
check(sha(ROOT / "plot_case_window_ranks.py"), meta["script_sha256"])
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
    case_stratum_rows=len(case_strata),
    case_ranking_rows=len(case_ranks),
    ranking_rows=len(rankings),
    bin_rows=len(bins),
    defined_full_auc_checks=defined_full,
    defined_same_case_auc_checks=defined_case,
    defined_bin_auc_checks=defined_bin,
    plotted_rows=plot_rows,
    script_sha256=sha(Path(__file__)),
    sources_sha256=sha(DATA / "sources.json"),
)
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(evidence, indent=2) + "\n")
print(json.dumps(evidence, indent=2))
