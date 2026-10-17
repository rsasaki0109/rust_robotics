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
FUTURE, NEW, DATA, PLOT = (
    RESULTS / "event-unit-influence",
    RESULTS / "event-score-ties",
    RESULTS / "event-temporal-rankings",
    args.figures,
)
MODE_DATA = args.results
PLANNER_DATA = RESULTS / "event-planner-influence"
SCENARIO_DATA = RESULTS / "event-scenario-influence"
SEED_DATA = RESULTS / "event-seed-influence"
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
SCENARIOS = ("dense_50x50", "maze_50x50", "open_50x50")
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


mode_sources = json.loads((MODE_DATA / "sources.json").read_text())
mode_base_sha = "8a18934fc97d3bb4e3c36e305274cc8d4001ee277ed1d90a0e240dcd92d1d85d"
check(sha(PLANNER_DATA / "sources.json"), mode_base_sha)
check(
    mode_sources["input_sha256"]["event-planner-influence/sources.json"], mode_base_sha
)
for name, h in mode_sources["input_sha256"].items():
    check(sha(RESULTS / name), h)
for name, h in mode_sources["analysis"]["helpers_sha256"].items():
    check(sha(ROOT / name), h)
check(
    sha(ROOT / "analyze_event_loss_mode_influence.py"),
    mode_sources["analysis"]["script_sha256"],
)
check(
    sha(RESULTS / "event-loss-mode-influence/protocol.md"),
    mode_sources["analysis"]["protocol_sha256"],
)
for name, h in mode_sources["output_sha256"].items():
    check(sha(MODE_DATA / name), h)


planner_sources = json.loads((PLANNER_DATA / "sources.json").read_text())
planner_base_sha = "1e88e25387f0c03a3ede1365868de0c77aac721287c172ca0b7ff81856cfec40"
check(sha(SCENARIO_DATA / "sources.json"), planner_base_sha)
check(
    planner_sources["input_sha256"]["event-scenario-influence/sources.json"],
    planner_base_sha,
)
for name, h in planner_sources["input_sha256"].items():
    check(sha(RESULTS / name), h)
for name, h in planner_sources["analysis"]["helpers_sha256"].items():
    check(sha(ROOT / name), h)
check(
    sha(ROOT / "analyze_event_planner_influence.py"),
    planner_sources["analysis"]["script_sha256"],
)
check(
    sha(RESULTS / "event-planner-influence/protocol.md"),
    planner_sources["analysis"]["protocol_sha256"],
)
for name, h in planner_sources["output_sha256"].items():
    check(sha(PLANNER_DATA / name), h)


scenario_sources = json.loads((SCENARIO_DATA / "sources.json").read_text())
scenario_base_sha = "fd8943676129642ab3edd66476faa2579f677aca29fe71a1576439159aac4176"
check(sha(SEED_DATA / "sources.json"), scenario_base_sha)
check(
    scenario_sources["input_sha256"]["event-seed-influence/sources.json"],
    scenario_base_sha,
)
for name, h in scenario_sources["input_sha256"].items():
    check(sha(RESULTS / name), h)
for name, h in scenario_sources["analysis"]["helpers_sha256"].items():
    check(sha(ROOT / name), h)
check(
    sha(ROOT / "analyze_event_scenario_influence.py"),
    scenario_sources["analysis"]["script_sha256"],
)
check(
    sha(RESULTS / "event-scenario-influence/protocol.md"),
    scenario_sources["analysis"]["protocol_sha256"],
)
for name, h in scenario_sources["output_sha256"].items():
    check(sha(SCENARIO_DATA / name), h)


seed_sources = json.loads((SEED_DATA / "sources.json").read_text())
seed_base_sha = "48d23a8c0deca5eaa9755644fecbd5960cafafb03e504cc96c54760f4e90b71f"
check(sha(FUTURE / "sources.json"), seed_base_sha)
check(seed_sources["input_sha256"]["event-unit-influence/sources.json"], seed_base_sha)
for name, h in seed_sources["input_sha256"].items():
    check(sha(RESULTS / name), h)
for name, h in seed_sources["analysis"]["helpers_sha256"].items():
    check(sha(ROOT / name), h)
check(
    sha(ROOT / "analyze_event_seed_influence.py"),
    seed_sources["analysis"]["script_sha256"],
)
check(
    sha(RESULTS / "event-seed-influence/protocol.md"),
    seed_sources["analysis"]["protocol_sha256"],
)
for name, h in seed_sources["output_sha256"].items():
    check(sha(SEED_DATA / name), h)


influence_sources = json.loads((FUTURE / "sources.json").read_text())
influence_base_sha = "b1b180955af97033109e3147f83864291083856b68c426ef4e7ae5d1b0f8f1b2"
check(sha(NEW / "sources.json"), influence_base_sha)
check(
    influence_sources["input_sha256"]["event-score-ties/sources.json"],
    influence_base_sha,
)
for name, h in influence_sources["input_sha256"].items():
    check(sha(RESULTS / name), h)
for name, h in influence_sources["analysis"]["helpers_sha256"].items():
    check(sha(ROOT / name), h)
check(
    sha(ROOT / "analyze_event_unit_influence.py"),
    influence_sources["analysis"]["script_sha256"],
)
check(
    sha(RESULTS / "event-unit-influence/protocol.md"),
    influence_sources["analysis"]["protocol_sha256"],
)
for name, h in influence_sources["output_sha256"].items():
    check(sha(FUTURE / name), h)


def influence_indexed(name, keys):
    rr = read(FUTURE / name)
    result = {tuple(r[k] for k in keys): r for r in rr}
    assert len(result) == len(rr)
    return result


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


SELECTION = [
    "partition",
    "noise",
    "variant",
    "stratum",
    "population",
    "planner_selection",
]
INFLUENCE_METRICS = [
    "auc",
    "native_auc",
    "tick_unequal_concordance",
    "native_unequal_concordance",
]
unit_audit = influence_indexed("unit-audit.csv", [*SELECTION, "unit_kind", "unit_id"])
deletions = influence_indexed(
    "delete-one.csv", [*NEWRANK, "scope", "unit_kind", "unit_id"]
)
influence_summary = influence_indexed(
    "influence-summary.csv", [*NEWRANK, "scope", "unit_kind"]
)
expected_unit_audit, expected_deletions, expected_influence_summary = (
    set(),
    set(),
    set(),
)
selection_cache = {}


def seed_indexed(name, keys):
    rr = read(SEED_DATA / name)
    result = {tuple(r[k] for k in keys): r for r in rr}
    assert len(result) == len(rr)
    return result


seed_audit = seed_indexed("seed-audit.csv", [*SELECTION, "unit_kind", "unit_id"])
seed_effects = seed_indexed(
    "seed-effects.csv", [*NEWRANK, "scope", "unit_kind", "unit_id"]
)
seed_summary = seed_indexed("sensitivity-summary.csv", [*NEWRANK, "scope", "unit_kind"])
expected_seed_audit, expected_seed_effects, expected_seed_summary = set(), set(), set()
seed_selection_cache = {}


def planned_seed_units(rr, metadata):
    skey = tuple(metadata[k] for k in SELECTION)
    if skey in seed_selection_cache:
        return seed_selection_cache[skey]
    seeds = SEEDS[metadata["partition"]]
    assert len(seeds) == len(set(seeds)) == 3
    assert all(r["seed"] in seeds for r in rr)
    units = []
    for seed in seeds:
        subset = [r for r in rr if r["seed"] == seed]
        cids = sorted({case_info[r["case_key"]]["case_id"] for r in subset})
        fids = sorted({case_info[r["case_key"]]["family_id"] for r in subset})
        expected = dict(
            **metadata,
            unit_kind="seed",
            unit_id=seed,
            case_ids="+".join(cids),
            family_ids="+".join(fids),
            records=len(subset),
            cases=len(cids),
            events=len({(r["case_key"], r["_event_id"]) for r in subset}),
            case_mass=sum(
                (Fraction(1, counts[r["case_key"]]) for r in subset), Fraction()
            ),
        )
        fields(seed_audit[*skey, "seed", seed], expected)
        expected_seed_audit.add((*skey, "seed", seed))
        units.append((expected, set(cids)))
    seed_selection_cache[skey] = units
    return units


def scenario_indexed(name, keys):
    rr = read(SCENARIO_DATA / name)
    result = {tuple(r[k] for k in keys): r for r in rr}
    assert len(result) == len(rr)
    return result


scenario_audit = scenario_indexed(
    "scenario-audit.csv", [*SELECTION, "unit_kind", "unit_id", "seed"]
)
scenario_effects = scenario_indexed(
    "scenario-effects.csv", [*NEWRANK, "scope", "unit_kind", "unit_id"]
)
scenario_summary = scenario_indexed(
    "sensitivity-summary.csv", [*NEWRANK, "scope", "unit_kind"]
)
scenario_cells = scenario_indexed(
    "seed-scenario-pairs.csv", [*NEWRANK, "scope", "unit_kind", "unit_id", "seed"]
)
(
    expected_scenario_audit,
    expected_scenario_effects,
    expected_scenario_summary,
    expected_scenario_cells,
) = (set(), set(), set(), set())
scenario_selection_cache = {}
SCENARIO_CELL_FIELDS = [
    prefix + cls + "_pair_mass"
    for prefix in ["", "native_"]
    for cls in ["concordant", "tied", "discordant"]
] + [
    "numerator",
    "denominator",
    "native_numerator",
    "pair_count",
    "events_with_pairs",
    "cases_with_pairs",
    "tick_unequal_pair_count",
    "native_unequal_pair_count",
    "tick_unequal_events",
    "tick_unequal_cases",
    "native_unequal_events",
    "native_unequal_cases",
    *INFLUENCE_METRICS,
]


def planned_scenario_units(rr, metadata):
    skey = tuple(metadata[k] for k in SELECTION)
    if skey in scenario_selection_cache:
        return scenario_selection_cache[skey]
    assert all(
        r["scenario"] in SCENARIOS and r["seed"] in SEEDS[metadata["partition"]]
        for r in rr
    )

    def form(subset, scenario, seed=None):
        cids = sorted({case_info[r["case_key"]]["case_id"] for r in subset})
        fids = sorted({case_info[r["case_key"]]["family_id"] for r in subset})
        expected = dict(
            **metadata,
            unit_kind="scenario",
            unit_id=scenario,
            case_ids="+".join(cids),
            family_ids="+".join(fids),
            records=len(subset),
            cases=len(cids),
            events=len({(r["case_key"], r["_event_id"]) for r in subset}),
            case_mass=sum(
                (Fraction(1, counts[r["case_key"]]) for r in subset), Fraction()
            ),
        )
        if seed is not None:
            expected["seed"] = seed
        return expected, set(cids)

    units, cells = [], []
    for scenario in SCENARIOS:
        subset = [r for r in rr if r["scenario"] == scenario]
        units.append(form(subset, scenario))
        for seed in SEEDS[metadata["partition"]]:
            expected, ids = form(
                [r for r in subset if r["seed"] == seed], scenario, seed
            )
            fields(scenario_audit[*skey, "scenario", scenario, seed], expected)
            expected_scenario_audit.add((*skey, "scenario", scenario, seed))
            cells.append((expected, ids))
    scenario_selection_cache[skey] = units, cells
    return units, cells


def planner_indexed(name, keys):
    rr = read(PLANNER_DATA / name)
    result = {tuple(r[k] for k in keys): r for r in rr}
    assert len(result) == len(rr)
    return result


planner_audit = planner_indexed(
    "planner-audit.csv", [*SELECTION, "unit_kind", "unit_id", "seed", "scenario"]
)
planner_effects = planner_indexed(
    "planner-effects.csv", [*NEWRANK, "scope", "unit_kind", "unit_id"]
)
planner_summary = planner_indexed(
    "sensitivity-summary.csv", [*NEWRANK, "scope", "unit_kind"]
)
planner_cells = planner_indexed(
    "planner-cells.csv", [*NEWRANK, "scope", "unit_kind", "unit_id", "seed", "scenario"]
)
(
    expected_planner_audit,
    expected_planner_effects,
    expected_planner_summary,
    expected_planner_cells,
    expected_collapsed_planner_cells,
) = (set(), set(), set(), set(), set())
planner_selection_cache = {}


def planned_planner_units(rr, metadata):
    skey = tuple(metadata[k] for k in SELECTION)
    if skey in planner_selection_cache:
        return planner_selection_cache[skey]
    assert all(
        r["planner"] in PLANNERS
        and r["seed"] in SEEDS[metadata["partition"]]
        and r["scenario"] in SCENARIOS
        for r in rr
    )

    def form(subset, planner, seed=None, scenario=None):
        cids = sorted({case_info[r["case_key"]]["case_id"] for r in subset})
        fids = sorted({case_info[r["case_key"]]["family_id"] for r in subset})
        expected = dict(
            **metadata,
            unit_kind="planner",
            unit_id=planner,
            case_ids="+".join(cids),
            family_ids="+".join(fids),
            records=len(subset),
            cases=len(cids),
            events=len({(r["case_key"], r["_event_id"]) for r in subset}),
            case_mass=sum(
                (Fraction(1, counts[r["case_key"]]) for r in subset), Fraction()
            ),
        )
        if seed is not None:
            expected.update(seed=seed, scenario=scenario)
        return expected, set(cids)

    units, cells = [], []
    for planner in PLANNERS:
        subset = [r for r in rr if r["planner"] == planner]
        units.append(form(subset, planner))
        for scenario, seed in product(SCENARIOS, SEEDS[metadata["partition"]]):
            expected, ids = form(
                [r for r in subset if r["scenario"] == scenario and r["seed"] == seed],
                planner,
                seed,
                scenario,
            )
            fields(planner_audit[*skey, "planner", planner, seed, scenario], expected)
            expected_planner_audit.add((*skey, "planner", planner, seed, scenario))
            cells.append((expected, ids))
    planner_selection_cache[skey] = units, cells
    return units, cells


def verify_planner_cells(
    rr, key, score, label, masks, evaluate, keep_without, baselines, cells
):
    if (
        score != "residual_sigma"
        or label not in LABELS[:2]
        or key[3] not in ["filter_single", "fusion_three"]
    ):
        return
    direct = [{} for scope in PRECISION_SCOPES]
    by_id = {(c["unit_id"], c["seed"], c["scenario"]): c for c, ids in cells}
    assert len(cells) == len(by_id) == 27
    for cell, ids in cells:
        keep = ~keep_without(ids)
        leaf = cell["unit_id"], cell["seed"], cell["scenario"]
        for i, (scope, mask) in enumerate(zip(PRECISION_SCOPES, masks)):
            value = evaluate(mask & keep)
            direct[i][leaf] = value
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(
                scope=scope,
                unit_kind="planner",
                unit_id=cell["unit_id"],
                seed=cell["seed"],
                scenario=cell["scenario"],
                **{f: cell[f] for f in ["records", "cases", "events", "case_mass"]},
                **{f: value[f] for f in SCENARIO_CELL_FIELDS},
            )
            ckey = (*key, score, label, scope, "planner", *leaf)
            fields(planner_cells[ckey], expected)
            expected_planner_cells.add(ckey)
    for i, (scope, baseline) in enumerate(zip(PRECISION_SCOPES, baselines)):
        for field in SCENARIO_CELL_FIELDS:
            if field not in INFLUENCE_METRICS:
                assert (
                    sum((v[field] for v in direct[i].values()), Fraction())
                    == baseline[field]
                )
        for scenario, seed in product(SCENARIOS, SEEDS[key[0]]):
            leaves = [(planner, seed, scenario) for planner in PLANNERS]
            combined = {
                f: sum((direct[i][leaf][f] for leaf in leaves), Fraction())
                for f in SCENARIO_CELL_FIELDS
                if f not in INFLUENCE_METRICS
            }
            den = combined["denominator"]
            c, d, nc, nd = [
                combined[f]
                for f in [
                    "concordant_pair_mass",
                    "discordant_pair_mass",
                    "native_concordant_pair_mass",
                    "native_discordant_pair_mass",
                ]
            ]
            combined.update(
                auc=combined["numerator"] / den if den else None,
                native_auc=combined["native_numerator"] / den if den else None,
                tick_unequal_concordance=c / (c + d) if c + d else None,
                native_unequal_concordance=nc / (nc + nd) if nc + nd else None,
            )
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(
                scope=scope,
                unit_kind="scenario",
                unit_id=scenario,
                seed=seed,
                **combined,
                **{
                    f: sum((by_id[leaf][f] for leaf in leaves), Fraction())
                    for f in ["records", "cases", "events", "case_mass"]
                },
            )
            skey = (*key, score, label, scope, "scenario", scenario, seed)
            fields(scenario_cells[skey], expected)
            expected_collapsed_planner_cells.add(skey)


LOSS_MODES = ("none", "random", "blackout")


def mode_indexed(name, keys):
    rr = read(MODE_DATA / name)
    result = {tuple(r[k] for k in keys): r for r in rr}
    assert len(result) == len(rr)
    return result


mode_audit = mode_indexed(
    "mode-audit.csv",
    [*SELECTION, "unit_kind", "unit_id", "seed", "scenario", "planner"],
)
mode_effects = mode_indexed(
    "mode-effects.csv", [*NEWRANK, "scope", "unit_kind", "unit_id"]
)
mode_summary = mode_indexed("sensitivity-summary.csv", [*NEWRANK, "scope", "unit_kind"])
mode_cells = mode_indexed(
    "mode-cells.csv",
    [*NEWRANK, "scope", "unit_kind", "unit_id", "seed", "scenario", "planner"],
)
(
    expected_mode_audit,
    expected_mode_effects,
    expected_mode_summary,
    expected_mode_cells,
    expected_collapsed_mode_cells,
) = (set(), set(), set(), set(), set())
mode_selection_cache = {}


def planned_mode_units(rr, metadata):
    skey = tuple(metadata[k] for k in SELECTION)
    if skey in mode_selection_cache:
        return mode_selection_cache[skey]
    assert all(
        r["mode"] in LOSS_MODES
        and r["planner"] in PLANNERS
        and r["seed"] in SEEDS[metadata["partition"]]
        and r["scenario"] in SCENARIOS
        for r in rr
    )

    def form(subset, mode, seed=None, scenario=None, planner=None):
        cids = sorted({case_info[r["case_key"]]["case_id"] for r in subset})
        fids = sorted({case_info[r["case_key"]]["family_id"] for r in subset})
        expected = dict(
            **metadata,
            unit_kind="loss_mode",
            unit_id=mode,
            case_ids="+".join(cids),
            family_ids="+".join(fids),
            records=len(subset),
            cases=len(cids),
            events=len({(r["case_key"], r["_event_id"]) for r in subset}),
            case_mass=sum(
                (Fraction(1, counts[r["case_key"]]) for r in subset), Fraction()
            ),
        )
        if seed is not None:
            assert len(cids) <= 1
            expected.update(seed=seed, scenario=scenario, planner=planner)
        return expected, set(cids)

    units, cells = [], []
    for mode in LOSS_MODES:
        subset = [r for r in rr if r["mode"] == mode]
        units.append(form(subset, mode))
        for scenario, seed, planner in product(
            SCENARIOS, SEEDS[metadata["partition"]], PLANNERS
        ):
            expected, ids = form(
                [
                    r
                    for r in subset
                    if r["scenario"] == scenario
                    and r["seed"] == seed
                    and r["planner"] == planner
                ],
                mode,
                seed,
                scenario,
                planner,
            )
            fields(
                mode_audit[*skey, "loss_mode", mode, seed, scenario, planner], expected
            )
            expected_mode_audit.add((*skey, "loss_mode", mode, seed, scenario, planner))
            cells.append((expected, ids))
    mode_selection_cache[skey] = units, cells
    return units, cells


def verify_mode_cells(
    rr, key, score, label, masks, evaluate, keep_without, baselines, cells
):
    if (
        score != "residual_sigma"
        or label not in LABELS[:2]
        or key[3] not in ["filter_single", "fusion_three"]
    ):
        return
    direct = [{} for scope in PRECISION_SCOPES]
    by_id = {
        (c["unit_id"], c["seed"], c["scenario"], c["planner"]): c for c, ids in cells
    }
    assert len(cells) == len(by_id) == 81
    for cell, ids in cells:
        keep = ~keep_without(ids)
        leaf = cell["unit_id"], cell["seed"], cell["scenario"], cell["planner"]
        for i, (scope, mask) in enumerate(zip(PRECISION_SCOPES, masks)):
            value = evaluate(mask & keep)
            direct[i][leaf] = value
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(
                scope=scope,
                unit_kind="loss_mode",
                unit_id=cell["unit_id"],
                seed=cell["seed"],
                scenario=cell["scenario"],
                planner=cell["planner"],
                case_id=cell["case_ids"],
                **{f: cell[f] for f in ["records", "cases", "events", "case_mass"]},
                **{f: value[f] for f in SCENARIO_CELL_FIELDS},
            )
            ckey = (*key, score, label, scope, "loss_mode", *leaf)
            fields(mode_cells[ckey], expected)
            expected_mode_cells.add(ckey)
    for i, (scope, baseline) in enumerate(zip(PRECISION_SCOPES, baselines)):
        for field in SCENARIO_CELL_FIELDS:
            if field not in INFLUENCE_METRICS:
                assert (
                    sum((v[field] for v in direct[i].values()), Fraction())
                    == baseline[field]
                )
        for planner, scenario, seed in product(PLANNERS, SCENARIOS, SEEDS[key[0]]):
            leaves = [(mode, seed, scenario, planner) for mode in LOSS_MODES]
            combined = {
                f: sum((direct[i][leaf][f] for leaf in leaves), Fraction())
                for f in SCENARIO_CELL_FIELDS
                if f not in INFLUENCE_METRICS
            }
            den = combined["denominator"]
            c, d, nc, nd = [
                combined[f]
                for f in [
                    "concordant_pair_mass",
                    "discordant_pair_mass",
                    "native_concordant_pair_mass",
                    "native_discordant_pair_mass",
                ]
            ]
            combined.update(
                auc=combined["numerator"] / den if den else None,
                native_auc=combined["native_numerator"] / den if den else None,
                tick_unequal_concordance=c / (c + d) if c + d else None,
                native_unequal_concordance=nc / (nc + nd) if nc + nd else None,
            )
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(
                scope=scope,
                unit_kind="planner",
                unit_id=planner,
                seed=seed,
                scenario=scenario,
                **combined,
                **{
                    f: sum((by_id[leaf][f] for leaf in leaves), Fraction())
                    for f in ["records", "cases", "events", "case_mass"]
                },
            )
            pkey = (*key, score, label, scope, "planner", planner, seed, scenario)
            fields(planner_cells[pkey], expected)
            expected_collapsed_mode_cells.add(pkey)


def selected_units(rr, metadata):
    skey = tuple(metadata[k] for k in SELECTION)
    if skey in selection_cache:
        return selection_cache[skey]
    units = []
    for kind in ["case", "family"]:
        groups = defaultdict(list)
        for r in rr:
            uid = case_info[r["case_key"]]["case_id" if kind == "case" else "family_id"]
            groups[uid].append(r)
        for uid, subset in sorted(groups.items()):
            cids = sorted({case_info[r["case_key"]]["case_id"] for r in subset})
            fids = sorted({case_info[r["case_key"]]["family_id"] for r in subset})
            expected = dict(
                **metadata,
                unit_kind=kind,
                unit_id=uid,
                case_ids="+".join(cids),
                family_ids="+".join(fids),
                records=len(subset),
                cases=len(cids),
                events=len({(r["case_key"], r["_event_id"]) for r in subset}),
                case_mass=sum(
                    (Fraction(1, counts[r["case_key"]]) for r in subset), Fraction()
                ),
            )
            fields(unit_audit[*skey, kind, uid], expected)
            expected_unit_audit.add((*skey, kind, uid))
            units.append((expected, set(cids)))
    selection_cache[skey] = units
    return units


def exact_direct_pairs(rr, score, label):
    # Build only admissible same-case/event pairs from original states. Every
    # deletion masks remaining pairs directly; no removed-mass subtraction.
    groups = defaultdict(list)
    for r in rr:
        groups[r["case_key"], r["_event_id"]].append(r)
    case_ids = sorted({case_info[r["case_key"]]["case_id"] for r in rr})
    case_code = {cid: i for i, cid in enumerate(case_ids)}
    categories, gaps, parent_counts, case_codes, event_codes = [], [], [], [], []
    for eid, ((case_key, event_id), subset) in enumerate(sorted(groups.items())):
        pp = [r for r in subset if r["outside"][label]]
        nn = [r for r in subset if not r["outside"][label]]
        if not pp or not nn:
            continue
        p = np.array([r[score + "_tick"] for r in pp], dtype=np.int64)[:, None]
        n = np.array([r[score + "_tick"] for r in nn], dtype=np.int64)[None, :]
        pn = np.array([r["_native_order"][score] for r in pp], dtype=np.int64)[:, None]
        npart = np.array([r["_native_order"][score] for r in nn], dtype=np.int64)[
            None, :
        ]
        assert not np.any((p > n) & ~(pn > npart)) and not np.any(
            (p < n) & ~(pn < npart)
        )
        category = np.where(
            p > n,
            0,
            np.where(p < n, 1, np.where(pn > npart, 2, np.where(pn == npart, 3, 4))),
        ).ravel()
        pt = np.array([r["_time_us"] for r in pp], dtype=np.int64)[:, None]
        nt = np.array([r["_time_us"] for r in nn], dtype=np.int64)[None, :]
        size = len(pp) * len(nn)
        categories.append(category)
        gaps.append(np.abs(pt - nt).ravel())
        parent_counts.append(np.full(size, counts[case_key], dtype=np.int64))
        case_codes.append(
            np.full(size, case_code[case_info[case_key]["case_id"]], dtype=np.int64)
        )
        event_codes.append(np.full(size, eid, dtype=np.int64))
    concat = lambda arrays: (
        np.concatenate(arrays) if arrays else np.array([], dtype=np.int64)
    )
    category, gap, nparent, pcase, pevent = map(
        concat, [categories, gaps, parent_counts, case_codes, event_codes]
    )
    sizes = np.unique(nparent)
    combined = np.searchsorted(sizes, nparent) * 5 + category
    scopes = [
        np.ones(len(category), dtype=bool),
        gap <= 500000,
        gap <= 100000,
        gap <= 20000,
    ]

    def evaluate(mask):
        raw = np.bincount(combined[mask], minlength=len(sizes) * 5).reshape(
            len(sizes), 5
        )
        m = [
            sum(
                (Fraction(int(raw[i, j]), int(n * n)) for i, n in enumerate(sizes)),
                Fraction(),
            )
            for j in range(5)
        ]
        c, d, rc, nt, rd = m
        t = rc + nt + rd
        nc, nd = c + rc, d + rd
        den = sum(m, Fraction())
        num, nnum = c + t / 2, nc + nt / 2
        unequal = mask & ((category == 0) | (category == 1))
        native_unequal = mask & (category != 3)
        supported = lambda values, sel: int(np.count_nonzero(np.bincount(values[sel])))
        return dict(
            concordant_pair_mass=c,
            tied_pair_mass=t,
            discordant_pair_mass=d,
            native_concordant_pair_mass=nc,
            native_tied_pair_mass=nt,
            native_discordant_pair_mass=nd,
            numerator=num,
            denominator=den,
            native_numerator=nnum,
            pair_count=int(np.count_nonzero(mask)),
            events_with_pairs=supported(pevent, mask),
            cases_with_pairs=supported(pcase, mask),
            tick_unequal_pair_count=int(np.count_nonzero(unequal)),
            native_unequal_pair_count=int(np.count_nonzero(native_unequal)),
            tick_unequal_events=supported(pevent, unequal),
            tick_unequal_cases=supported(pcase, unequal),
            native_unequal_events=supported(pevent, native_unequal),
            native_unequal_cases=supported(pcase, native_unequal),
            auc=num / den if den else None,
            native_auc=nnum / den if den else None,
            tick_unequal_concordance=c / (c + d) if c + d else None,
            native_unequal_concordance=nc / (nc + nd) if nc + nd else None,
        )

    def keep_without(ids):
        return ~np.isin(pcase, [case_code[cid] for cid in ids])

    return scopes, evaluate, keep_without


def influence_statistics(base, rows, selected):
    summary = dict(
        **selected,
        baseline_pair_mass=base["denominator"],
        baseline_pair_count=base["pair_count"],
        baseline_tick_unequal_mass=base["concordant_pair_mass"]
        + base["discordant_pair_mass"],
        baseline_native_unequal_mass=base["native_concordant_pair_mass"]
        + base["native_discordant_pair_mass"],
        baseline_tick_unequal_pair_count=base["tick_unequal_pair_count"],
        baseline_native_unequal_pair_count=base["native_unequal_pair_count"],
        units=len(rows),
    )
    for prefix in ["", "tick_unequal_", "native_unequal_"]:
        summary[prefix + "supported_units" if prefix else "pair_supported_units"] = sum(
            r["removed_" + prefix + "pair_count"] > 0 for r in rows
        )
    for field in [
        "removed_pair_share",
        "removed_tick_unequal_share",
        "removed_native_unequal_share",
    ]:
        values = [r[field] for r in rows if r[field] is not None]
        summary["max_" + field] = max(values, default=None)
    half = Fraction(1, 2)
    for metric in INFLUENCE_METRICS:
        values = [r[metric] for r in rows if r[metric] is not None]
        b = base[metric]
        summary["baseline_" + metric] = b
        summary[metric + "_defined_deletions"] = len(values)
        summary[metric + "_undefined_deletions"] = len(rows) - len(values)
        summary[metric + "_min"] = min(values, default=None)
        summary[metric + "_max"] = max(values, default=None)
        summary[metric + "_max_abs_delta"] = (
            max((abs(v - b) for v in values), default=None) if b is not None else None
        )
        summary[metric + "_below_half"] = sum(v < half for v in values)
        summary[metric + "_at_half"] = sum(v == half for v in values)
        summary[metric + "_above_half"] = sum(v > half for v in values)
        summary[metric + "_strict_side_flips"] = (
            sum((v - half) * (b - half) < 0 for v in values) if b is not None else 0
        )
    return summary


def verify_influence(rr, key, score, label):
    metadata = dict(zip(NEWGROUP, key))
    selection_meta = {k: metadata[k] for k in SELECTION}
    units = selected_units(rr, selection_meta)
    selected = dict(
        records=len(rr),
        cases=len({r["case_key"] for r in rr}),
        events=len({(r["case_key"], r["_event_id"]) for r in rr}),
    )
    masks, evaluate, keep_without = exact_direct_pairs(rr, score, label)
    baselines = [evaluate(mask) for mask in masks]
    rows_by_scope = [[] for scope in PRECISION_SCOPES]
    for unit, removed_ids in units:
        keep = keep_without(removed_ids)
        remaining_rows = [
            r for r in rr if case_info[r["case_key"]]["case_id"] not in removed_ids
        ]
        remaining_meta = dict(
            records=len(remaining_rows),
            cases=len({r["case_key"] for r in remaining_rows}),
            events=len({(r["case_key"], r["_event_id"]) for r in remaining_rows}),
        )
        for i, (scope, mask, baseline) in enumerate(
            zip(PRECISION_SCOPES, masks, baselines)
        ):
            left = evaluate(mask & keep)
            removed = evaluate(mask & ~keep)
            for field in [
                "concordant_pair_mass",
                "tied_pair_mass",
                "discordant_pair_mass",
                "native_concordant_pair_mass",
                "native_tied_pair_mass",
                "native_discordant_pair_mass",
                "denominator",
            ]:
                assert baseline[field] == left[field] + removed[field]
            den = baseline["denominator"]
            tick_den = (
                baseline["concordant_pair_mass"] + baseline["discordant_pair_mass"]
            )
            native_den = (
                baseline["native_concordant_pair_mass"]
                + baseline["native_discordant_pair_mass"]
            )
            row = dict(
                unit_kind=unit["unit_kind"],
                unit_id=unit["unit_id"],
                removed_records=unit["records"],
                removed_cases=unit["cases"],
                removed_events=unit["events"],
                removed_case_mass=unit["case_mass"],
                removed_pair_count=removed["pair_count"],
                removed_pair_share=removed["denominator"] / den if den else None,
                removed_tick_unequal_pair_count=removed["tick_unequal_pair_count"],
                removed_native_unequal_pair_count=removed["native_unequal_pair_count"],
                removed_tick_unequal_share=(
                    (removed["concordant_pair_mass"] + removed["discordant_pair_mass"])
                    / tick_den
                    if tick_den
                    else None
                ),
                removed_native_unequal_share=(
                    (
                        removed["native_concordant_pair_mass"]
                        + removed["native_discordant_pair_mass"]
                    )
                    / native_den
                    if native_den
                    else None
                ),
                **remaining_meta,
            )
            for prefix in ["", "native_"]:
                for cls in ["concordant", "tied", "discordant"]:
                    row["removed_" + prefix + cls + "_mass"] = removed[
                        prefix + cls + "_pair_mass"
                    ]
            for field in [
                "numerator",
                "denominator",
                "native_numerator",
                "pair_count",
                "events_with_pairs",
                "cases_with_pairs",
                "tick_unequal_pair_count",
                "native_unequal_pair_count",
                "tick_unequal_events",
                "tick_unequal_cases",
                "native_unequal_events",
                "native_unequal_cases",
            ]:
                row[field] = left[field]
            for metric in INFLUENCE_METRICS:
                row[metric] = left[metric]
                row["delta_" + metric] = (
                    left[metric] - baseline[metric]
                    if left[metric] is not None and baseline[metric] is not None
                    else None
                )
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(scope=scope, **row)
            dkey = (*key, score, label, scope, unit["unit_kind"], unit["unit_id"])
            fields(deletions[dkey], expected)
            expected_deletions.add(dkey)
            rows_by_scope[i].append(row)
    for i, (scope, baseline) in enumerate(zip(PRECISION_SCOPES, baselines)):
        for kind in ["case", "family"]:
            selected_rows = [r for r in rows_by_scope[i] if r["unit_kind"] == kind]
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(
                scope=scope,
                unit_kind=kind,
                **influence_statistics(baseline, selected_rows, selected),
            )
            skey = (*key, score, label, scope, kind)
            fields(influence_summary[skey], expected)
            expected_influence_summary.add(skey)

    verify_seed(rr, key, score, label, masks, evaluate, keep_without, baselines)
    verify_scenario(rr, key, score, label, masks, evaluate, keep_without, baselines)
    verify_planner(rr, key, score, label, masks, evaluate, keep_without, baselines)
    verify_loss_mode(rr, key, score, label, masks, evaluate, keep_without, baselines)


def verify_seed(rr, key, score, label, masks, evaluate, keep_without, baselines):
    metadata = dict(zip(NEWGROUP, key))
    selection_meta = {k: metadata[k] for k in SELECTION}
    units = planned_seed_units(rr, selection_meta)
    selected = dict(
        records=len(rr),
        cases=len({r["case_key"] for r in rr}),
        events=len({(r["case_key"], r["_event_id"]) for r in rr}),
    )
    rows_by_scope = [[] for scope in PRECISION_SCOPES]
    for unit, removed_ids in units:
        keep = keep_without(removed_ids)
        remaining_rows = [
            r for r in rr if case_info[r["case_key"]]["case_id"] not in removed_ids
        ]
        remaining_meta = dict(
            records=len(remaining_rows),
            cases=len({r["case_key"] for r in remaining_rows}),
            events=len({(r["case_key"], r["_event_id"]) for r in remaining_rows}),
        )
        for i, (scope, mask, baseline) in enumerate(
            zip(PRECISION_SCOPES, masks, baselines)
        ):
            left = evaluate(mask & keep)
            removed = evaluate(mask & ~keep)
            for field in [
                "concordant_pair_mass",
                "tied_pair_mass",
                "discordant_pair_mass",
                "native_concordant_pair_mass",
                "native_tied_pair_mass",
                "native_discordant_pair_mass",
                "denominator",
            ]:
                assert baseline[field] == left[field] + removed[field]
            den = baseline["denominator"]
            tick_den = (
                baseline["concordant_pair_mass"] + baseline["discordant_pair_mass"]
            )
            native_den = (
                baseline["native_concordant_pair_mass"]
                + baseline["native_discordant_pair_mass"]
            )
            row = dict(
                unit_kind=unit["unit_kind"],
                unit_id=unit["unit_id"],
                removed_records=unit["records"],
                removed_cases=unit["cases"],
                removed_events=unit["events"],
                removed_case_mass=unit["case_mass"],
                removed_pair_count=removed["pair_count"],
                removed_pair_share=removed["denominator"] / den if den else None,
                removed_tick_unequal_pair_count=removed["tick_unequal_pair_count"],
                removed_native_unequal_pair_count=removed["native_unequal_pair_count"],
                removed_tick_unequal_share=(
                    (removed["concordant_pair_mass"] + removed["discordant_pair_mass"])
                    / tick_den
                    if tick_den
                    else None
                ),
                removed_native_unequal_share=(
                    (
                        removed["native_concordant_pair_mass"]
                        + removed["native_discordant_pair_mass"]
                    )
                    / native_den
                    if native_den
                    else None
                ),
                **remaining_meta,
            )
            for prefix in ["", "native_"]:
                for cls in ["concordant", "tied", "discordant"]:
                    row["removed_" + prefix + cls + "_mass"] = removed[
                        prefix + cls + "_pair_mass"
                    ]
            for field in [
                "numerator",
                "denominator",
                "native_numerator",
                "pair_count",
                "events_with_pairs",
                "cases_with_pairs",
                "tick_unequal_pair_count",
                "native_unequal_pair_count",
                "tick_unequal_events",
                "tick_unequal_cases",
                "native_unequal_events",
                "native_unequal_cases",
            ]:
                row[field] = left[field]
            for metric in INFLUENCE_METRICS:
                row[metric] = left[metric]
                row["delta_" + metric] = (
                    left[metric] - baseline[metric]
                    if left[metric] is not None and baseline[metric] is not None
                    else None
                )
            for field in [
                "numerator",
                "denominator",
                "native_numerator",
                "pair_count",
                "events_with_pairs",
                "cases_with_pairs",
                "tick_unequal_pair_count",
                "native_unequal_pair_count",
                "tick_unequal_events",
                "tick_unequal_cases",
                "native_unequal_events",
                "native_unequal_cases",
                *INFLUENCE_METRICS,
            ]:
                row["seed_" + field] = removed[field]
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(scope=scope, **row)
            dkey = (*key, score, label, scope, unit["unit_kind"], unit["unit_id"])
            fields(seed_effects[dkey], expected)
            expected_seed_effects.add(dkey)
            rows_by_scope[i].append(row)
    for i, (scope, baseline) in enumerate(zip(PRECISION_SCOPES, baselines)):
        for kind in ["seed"]:
            selected_rows = [r for r in rows_by_scope[i] if r["unit_kind"] == kind]
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(
                scope=scope,
                unit_kind=kind,
                **influence_statistics(baseline, selected_rows, selected),
            )
            skey = (*key, score, label, scope, kind)
            expected["represented_units"] = sum(
                r["removed_records"] > 0 for r in selected_rows
            )
            for metric in INFLUENCE_METRICS:
                values = [
                    r["seed_" + metric]
                    for r in selected_rows
                    if r["seed_" + metric] is not None
                ]
                expected[metric + "_seed_only_defined"] = len(values)
                expected[metric + "_seed_only_undefined"] = 3 - len(values)
                expected[metric + "_seed_only_min"] = min(values, default=None)
                expected[metric + "_seed_only_max"] = max(values, default=None)
            fields(seed_summary[skey], expected)
            expected_seed_summary.add(skey)
        for field in [
            "concordant_pair_mass",
            "tied_pair_mass",
            "discordant_pair_mass",
            "native_concordant_pair_mass",
            "native_tied_pair_mass",
            "native_discordant_pair_mass",
        ]:
            prefix, cls = (
                ("native_", field[7:-10])
                if field.startswith("native_")
                else ("", field[:-10])
            )
            assert (
                sum(
                    (r["removed_" + prefix + cls + "_mass"] for r in rows_by_scope[i]),
                    Fraction(),
                )
                == baseline[field]
            )
        assert (
            sum(r["seed_pair_count"] for r in rows_by_scope[i])
            == baseline["pair_count"]
        )


def verify_scenario(rr, key, score, label, masks, evaluate, keep_without, baselines):
    metadata = dict(zip(NEWGROUP, key))
    selection_meta = {k: metadata[k] for k in SELECTION}
    units, cells = planned_scenario_units(rr, selection_meta)
    selected = dict(
        records=len(rr),
        cases=len({r["case_key"] for r in rr}),
        events=len({(r["case_key"], r["_event_id"]) for r in rr}),
    )
    rows_by_scope = [[] for scope in PRECISION_SCOPES]
    direct_cells = [[] for scope in PRECISION_SCOPES]
    for cell, ids in cells:
        keep = ~keep_without(ids)
        for i, (scope, mask) in enumerate(zip(PRECISION_SCOPES, masks)):
            value = evaluate(mask & keep)
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(
                scope=scope,
                unit_kind="scenario",
                unit_id=cell["unit_id"],
                seed=cell["seed"],
                **{f: cell[f] for f in ["records", "cases", "events", "case_mass"]},
                **{f: value[f] for f in SCENARIO_CELL_FIELDS},
            )
            ckey = (
                *key,
                score,
                label,
                scope,
                "scenario",
                cell["unit_id"],
                cell["seed"],
            )
            fields(scenario_cells[ckey], expected)
            expected_scenario_cells.add(ckey)
            direct_cells[i].append(value)
    for i, baseline in enumerate(baselines):
        for field in SCENARIO_CELL_FIELDS:
            if field not in INFLUENCE_METRICS:
                assert (
                    sum((v[field] for v in direct_cells[i]), Fraction())
                    == baseline[field]
                )
    for unit, removed_ids in units:
        keep = keep_without(removed_ids)
        remaining_rows = [
            r for r in rr if case_info[r["case_key"]]["case_id"] not in removed_ids
        ]
        remaining_meta = dict(
            records=len(remaining_rows),
            cases=len({r["case_key"] for r in remaining_rows}),
            events=len({(r["case_key"], r["_event_id"]) for r in remaining_rows}),
        )
        for i, (scope, mask, baseline) in enumerate(
            zip(PRECISION_SCOPES, masks, baselines)
        ):
            left = evaluate(mask & keep)
            removed = evaluate(mask & ~keep)
            for field in [
                "concordant_pair_mass",
                "tied_pair_mass",
                "discordant_pair_mass",
                "native_concordant_pair_mass",
                "native_tied_pair_mass",
                "native_discordant_pair_mass",
                "denominator",
            ]:
                assert baseline[field] == left[field] + removed[field]
            den = baseline["denominator"]
            tick_den = (
                baseline["concordant_pair_mass"] + baseline["discordant_pair_mass"]
            )
            native_den = (
                baseline["native_concordant_pair_mass"]
                + baseline["native_discordant_pair_mass"]
            )
            row = dict(
                unit_kind=unit["unit_kind"],
                unit_id=unit["unit_id"],
                removed_records=unit["records"],
                removed_cases=unit["cases"],
                removed_events=unit["events"],
                removed_case_mass=unit["case_mass"],
                removed_pair_count=removed["pair_count"],
                removed_pair_share=removed["denominator"] / den if den else None,
                removed_tick_unequal_pair_count=removed["tick_unequal_pair_count"],
                removed_native_unequal_pair_count=removed["native_unequal_pair_count"],
                removed_tick_unequal_share=(
                    (removed["concordant_pair_mass"] + removed["discordant_pair_mass"])
                    / tick_den
                    if tick_den
                    else None
                ),
                removed_native_unequal_share=(
                    (
                        removed["native_concordant_pair_mass"]
                        + removed["native_discordant_pair_mass"]
                    )
                    / native_den
                    if native_den
                    else None
                ),
                **remaining_meta,
            )
            for prefix in ["", "native_"]:
                for cls in ["concordant", "tied", "discordant"]:
                    row["removed_" + prefix + cls + "_mass"] = removed[
                        prefix + cls + "_pair_mass"
                    ]
            for field in [
                "numerator",
                "denominator",
                "native_numerator",
                "pair_count",
                "events_with_pairs",
                "cases_with_pairs",
                "tick_unequal_pair_count",
                "native_unequal_pair_count",
                "tick_unequal_events",
                "tick_unequal_cases",
                "native_unequal_events",
                "native_unequal_cases",
            ]:
                row[field] = left[field]
            for metric in INFLUENCE_METRICS:
                row[metric] = left[metric]
                row["delta_" + metric] = (
                    left[metric] - baseline[metric]
                    if left[metric] is not None and baseline[metric] is not None
                    else None
                )
            for field in [
                "numerator",
                "denominator",
                "native_numerator",
                "pair_count",
                "events_with_pairs",
                "cases_with_pairs",
                "tick_unequal_pair_count",
                "native_unequal_pair_count",
                "tick_unequal_events",
                "tick_unequal_cases",
                "native_unequal_events",
                "native_unequal_cases",
                *INFLUENCE_METRICS,
            ]:
                row["scenario_" + field] = removed[field]
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(scope=scope, **row)
            dkey = (*key, score, label, scope, unit["unit_kind"], unit["unit_id"])
            fields(scenario_effects[dkey], expected)
            expected_scenario_effects.add(dkey)
            rows_by_scope[i].append(row)
    for i, (scope, baseline) in enumerate(zip(PRECISION_SCOPES, baselines)):
        for kind in ["scenario"]:
            selected_rows = [r for r in rows_by_scope[i] if r["unit_kind"] == kind]
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(
                scope=scope,
                unit_kind=kind,
                **influence_statistics(baseline, selected_rows, selected),
            )
            skey = (*key, score, label, scope, kind)
            expected["represented_units"] = sum(
                r["removed_records"] > 0 for r in selected_rows
            )
            for metric in INFLUENCE_METRICS:
                values = [
                    r["scenario_" + metric]
                    for r in selected_rows
                    if r["scenario_" + metric] is not None
                ]
                expected[metric + "_scenario_only_defined"] = len(values)
                expected[metric + "_scenario_only_undefined"] = 3 - len(values)
                expected[metric + "_scenario_only_min"] = min(values, default=None)
                expected[metric + "_scenario_only_max"] = max(values, default=None)
            fields(scenario_summary[skey], expected)
            expected_scenario_summary.add(skey)
        for field in [
            "concordant_pair_mass",
            "tied_pair_mass",
            "discordant_pair_mass",
            "native_concordant_pair_mass",
            "native_tied_pair_mass",
            "native_discordant_pair_mass",
        ]:
            prefix, cls = (
                ("native_", field[7:-10])
                if field.startswith("native_")
                else ("", field[:-10])
            )
            assert (
                sum(
                    (r["removed_" + prefix + cls + "_mass"] for r in rows_by_scope[i]),
                    Fraction(),
                )
                == baseline[field]
            )
        assert (
            sum(r["scenario_pair_count"] for r in rows_by_scope[i])
            == baseline["pair_count"]
        )


def verify_planner(rr, key, score, label, masks, evaluate, keep_without, baselines):
    metadata = dict(zip(NEWGROUP, key))
    selection_meta = {k: metadata[k] for k in SELECTION}
    units, cells = planned_planner_units(rr, selection_meta)
    selected = dict(
        records=len(rr),
        cases=len({r["case_key"] for r in rr}),
        events=len({(r["case_key"], r["_event_id"]) for r in rr}),
    )
    rows_by_scope = [[] for scope in PRECISION_SCOPES]
    for unit, removed_ids in units:
        keep = keep_without(removed_ids)
        remaining_rows = [
            r for r in rr if case_info[r["case_key"]]["case_id"] not in removed_ids
        ]
        remaining_meta = dict(
            records=len(remaining_rows),
            cases=len({r["case_key"] for r in remaining_rows}),
            events=len({(r["case_key"], r["_event_id"]) for r in remaining_rows}),
        )
        for i, (scope, mask, baseline) in enumerate(
            zip(PRECISION_SCOPES, masks, baselines)
        ):
            left = evaluate(mask & keep)
            removed = evaluate(mask & ~keep)
            for field in [
                "concordant_pair_mass",
                "tied_pair_mass",
                "discordant_pair_mass",
                "native_concordant_pair_mass",
                "native_tied_pair_mass",
                "native_discordant_pair_mass",
                "denominator",
            ]:
                assert baseline[field] == left[field] + removed[field]
            den = baseline["denominator"]
            tick_den = (
                baseline["concordant_pair_mass"] + baseline["discordant_pair_mass"]
            )
            native_den = (
                baseline["native_concordant_pair_mass"]
                + baseline["native_discordant_pair_mass"]
            )
            row = dict(
                unit_kind=unit["unit_kind"],
                unit_id=unit["unit_id"],
                removed_records=unit["records"],
                removed_cases=unit["cases"],
                removed_events=unit["events"],
                removed_case_mass=unit["case_mass"],
                removed_pair_count=removed["pair_count"],
                removed_pair_share=removed["denominator"] / den if den else None,
                removed_tick_unequal_pair_count=removed["tick_unequal_pair_count"],
                removed_native_unequal_pair_count=removed["native_unequal_pair_count"],
                removed_tick_unequal_share=(
                    (removed["concordant_pair_mass"] + removed["discordant_pair_mass"])
                    / tick_den
                    if tick_den
                    else None
                ),
                removed_native_unequal_share=(
                    (
                        removed["native_concordant_pair_mass"]
                        + removed["native_discordant_pair_mass"]
                    )
                    / native_den
                    if native_den
                    else None
                ),
                **remaining_meta,
            )
            for prefix in ["", "native_"]:
                for cls in ["concordant", "tied", "discordant"]:
                    row["removed_" + prefix + cls + "_mass"] = removed[
                        prefix + cls + "_pair_mass"
                    ]
            for field in [
                "numerator",
                "denominator",
                "native_numerator",
                "pair_count",
                "events_with_pairs",
                "cases_with_pairs",
                "tick_unequal_pair_count",
                "native_unequal_pair_count",
                "tick_unequal_events",
                "tick_unequal_cases",
                "native_unequal_events",
                "native_unequal_cases",
            ]:
                row[field] = left[field]
            for metric in INFLUENCE_METRICS:
                row[metric] = left[metric]
                row["delta_" + metric] = (
                    left[metric] - baseline[metric]
                    if left[metric] is not None and baseline[metric] is not None
                    else None
                )
            for field in [
                "numerator",
                "denominator",
                "native_numerator",
                "pair_count",
                "events_with_pairs",
                "cases_with_pairs",
                "tick_unequal_pair_count",
                "native_unequal_pair_count",
                "tick_unequal_events",
                "tick_unequal_cases",
                "native_unequal_events",
                "native_unequal_cases",
                *INFLUENCE_METRICS,
            ]:
                row["planner_" + field] = removed[field]
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(scope=scope, **row)
            dkey = (*key, score, label, scope, unit["unit_kind"], unit["unit_id"])
            fields(planner_effects[dkey], expected)
            expected_planner_effects.add(dkey)
            rows_by_scope[i].append(row)
    for i, (scope, baseline) in enumerate(zip(PRECISION_SCOPES, baselines)):
        for kind in ["planner"]:
            selected_rows = [r for r in rows_by_scope[i] if r["unit_kind"] == kind]
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(
                scope=scope,
                unit_kind=kind,
                **influence_statistics(baseline, selected_rows, selected),
            )
            skey = (*key, score, label, scope, kind)
            expected["represented_units"] = sum(
                r["removed_records"] > 0 for r in selected_rows
            )
            for metric in INFLUENCE_METRICS:
                values = [
                    r["planner_" + metric]
                    for r in selected_rows
                    if r["planner_" + metric] is not None
                ]
                expected[metric + "_planner_only_defined"] = len(values)
                expected[metric + "_planner_only_undefined"] = 3 - len(values)
                expected[metric + "_planner_only_min"] = min(values, default=None)
                expected[metric + "_planner_only_max"] = max(values, default=None)
            fields(planner_summary[skey], expected)
            expected_planner_summary.add(skey)
        for field in [
            "concordant_pair_mass",
            "tied_pair_mass",
            "discordant_pair_mass",
            "native_concordant_pair_mass",
            "native_tied_pair_mass",
            "native_discordant_pair_mass",
        ]:
            prefix, cls = (
                ("native_", field[7:-10])
                if field.startswith("native_")
                else ("", field[:-10])
            )
            assert (
                sum(
                    (r["removed_" + prefix + cls + "_mass"] for r in rows_by_scope[i]),
                    Fraction(),
                )
                == baseline[field]
            )
        assert (
            sum(r["planner_pair_count"] for r in rows_by_scope[i])
            == baseline["pair_count"]
        )

    verify_planner_cells(
        rr, key, score, label, masks, evaluate, keep_without, baselines, cells
    )


def verify_loss_mode(rr, key, score, label, masks, evaluate, keep_without, baselines):
    metadata = dict(zip(NEWGROUP, key))
    selection_meta = {k: metadata[k] for k in SELECTION}
    units, cells = planned_mode_units(rr, selection_meta)
    selected = dict(
        records=len(rr),
        cases=len({r["case_key"] for r in rr}),
        events=len({(r["case_key"], r["_event_id"]) for r in rr}),
    )
    rows_by_scope = [[] for scope in PRECISION_SCOPES]
    for unit, removed_ids in units:
        keep = keep_without(removed_ids)
        remaining_rows = [
            r for r in rr if case_info[r["case_key"]]["case_id"] not in removed_ids
        ]
        remaining_meta = dict(
            records=len(remaining_rows),
            cases=len({r["case_key"] for r in remaining_rows}),
            events=len({(r["case_key"], r["_event_id"]) for r in remaining_rows}),
        )
        for i, (scope, mask, baseline) in enumerate(
            zip(PRECISION_SCOPES, masks, baselines)
        ):
            left = evaluate(mask & keep)
            removed = evaluate(mask & ~keep)
            for field in [
                "concordant_pair_mass",
                "tied_pair_mass",
                "discordant_pair_mass",
                "native_concordant_pair_mass",
                "native_tied_pair_mass",
                "native_discordant_pair_mass",
                "denominator",
            ]:
                assert baseline[field] == left[field] + removed[field]
            den = baseline["denominator"]
            tick_den = (
                baseline["concordant_pair_mass"] + baseline["discordant_pair_mass"]
            )
            native_den = (
                baseline["native_concordant_pair_mass"]
                + baseline["native_discordant_pair_mass"]
            )
            row = dict(
                unit_kind=unit["unit_kind"],
                unit_id=unit["unit_id"],
                removed_records=unit["records"],
                removed_cases=unit["cases"],
                removed_events=unit["events"],
                removed_case_mass=unit["case_mass"],
                removed_pair_count=removed["pair_count"],
                removed_pair_share=removed["denominator"] / den if den else None,
                removed_tick_unequal_pair_count=removed["tick_unequal_pair_count"],
                removed_native_unequal_pair_count=removed["native_unequal_pair_count"],
                removed_tick_unequal_share=(
                    (removed["concordant_pair_mass"] + removed["discordant_pair_mass"])
                    / tick_den
                    if tick_den
                    else None
                ),
                removed_native_unequal_share=(
                    (
                        removed["native_concordant_pair_mass"]
                        + removed["native_discordant_pair_mass"]
                    )
                    / native_den
                    if native_den
                    else None
                ),
                **remaining_meta,
            )
            for prefix in ["", "native_"]:
                for cls in ["concordant", "tied", "discordant"]:
                    row["removed_" + prefix + cls + "_mass"] = removed[
                        prefix + cls + "_pair_mass"
                    ]
            for field in [
                "numerator",
                "denominator",
                "native_numerator",
                "pair_count",
                "events_with_pairs",
                "cases_with_pairs",
                "tick_unequal_pair_count",
                "native_unequal_pair_count",
                "tick_unequal_events",
                "tick_unequal_cases",
                "native_unequal_events",
                "native_unequal_cases",
            ]:
                row[field] = left[field]
            for metric in INFLUENCE_METRICS:
                row[metric] = left[metric]
                row["delta_" + metric] = (
                    left[metric] - baseline[metric]
                    if left[metric] is not None and baseline[metric] is not None
                    else None
                )
            for field in [
                "numerator",
                "denominator",
                "native_numerator",
                "pair_count",
                "events_with_pairs",
                "cases_with_pairs",
                "tick_unequal_pair_count",
                "native_unequal_pair_count",
                "tick_unequal_events",
                "tick_unequal_cases",
                "native_unequal_events",
                "native_unequal_cases",
                *INFLUENCE_METRICS,
            ]:
                row["mode_" + field] = removed[field]
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(scope=scope, **row)
            dkey = (*key, score, label, scope, unit["unit_kind"], unit["unit_id"])
            fields(mode_effects[dkey], expected)
            expected_mode_effects.add(dkey)
            rows_by_scope[i].append(row)
    for i, (scope, baseline) in enumerate(zip(PRECISION_SCOPES, baselines)):
        for kind in ["loss_mode"]:
            selected_rows = [r for r in rows_by_scope[i] if r["unit_kind"] == kind]
            expected = dict(zip(NEWRANK, (*key, score, label)))
            expected.update(
                scope=scope,
                unit_kind=kind,
                **influence_statistics(baseline, selected_rows, selected),
            )
            skey = (*key, score, label, scope, kind)
            expected["represented_units"] = sum(
                r["removed_records"] > 0 for r in selected_rows
            )
            for metric in INFLUENCE_METRICS:
                values = [
                    r["mode_" + metric]
                    for r in selected_rows
                    if r["mode_" + metric] is not None
                ]
                expected[metric + "_mode_only_defined"] = len(values)
                expected[metric + "_mode_only_undefined"] = 3 - len(values)
                expected[metric + "_mode_only_min"] = min(values, default=None)
                expected[metric + "_mode_only_max"] = max(values, default=None)
            fields(mode_summary[skey], expected)
            expected_mode_summary.add(skey)
        for field in [
            "concordant_pair_mass",
            "tied_pair_mass",
            "discordant_pair_mass",
            "native_concordant_pair_mass",
            "native_tied_pair_mass",
            "native_discordant_pair_mass",
        ]:
            prefix, cls = (
                ("native_", field[7:-10])
                if field.startswith("native_")
                else ("", field[:-10])
            )
            assert (
                sum(
                    (r["removed_" + prefix + cls + "_mass"] for r in rows_by_scope[i]),
                    Fraction(),
                )
                == baseline[field]
            )
        assert (
            sum(r["mode_pair_count"] for r in rows_by_scope[i])
            == baseline["pair_count"]
        )

    verify_mode_cells(
        rr, key, score, label, masks, evaluate, keep_without, baselines, cells
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
        verify_influence(rr, key, score, label)
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
assert set(unit_audit) == expected_unit_audit and len(unit_audit) == 782
assert set(deletions) == expected_deletions and len(deletions) == 56304
assert (
    set(influence_summary) == expected_influence_summary
    and len(influence_summary) == 17280
)
assert set(seed_audit) == expected_seed_audit and len(seed_audit) == 360
assert set(seed_effects) == expected_seed_effects and len(seed_effects) == 25920
assert set(seed_summary) == expected_seed_summary and len(seed_summary) == 8640
assert set(scenario_audit) == expected_scenario_audit and len(scenario_audit) == 1080
assert (
    set(scenario_effects) == expected_scenario_effects
    and len(scenario_effects) == 25920
)
assert (
    set(scenario_summary) == expected_scenario_summary and len(scenario_summary) == 8640
)
assert set(scenario_cells) == expected_scenario_cells and len(scenario_cells) == 77760
assert {r["scenario"] for r in cases_actual.values()} == set(SCENARIOS)
assert set(planner_audit) == expected_planner_audit and len(planner_audit) == 3240
assert (
    set(planner_effects) == expected_planner_effects and len(planner_effects) == 25920
)
assert set(planner_summary) == expected_planner_summary and len(planner_summary) == 8640
assert set(planner_cells) == expected_planner_cells and len(planner_cells) == 20736
assert len(expected_collapsed_planner_cells) == 6912
assert {r["planner"] for r in cases_actual.values()} == set(PLANNERS)
assert set(mode_audit) == expected_mode_audit and len(mode_audit) == 9720
assert set(mode_effects) == expected_mode_effects and len(mode_effects) == 25920
assert set(mode_summary) == expected_mode_summary and len(mode_summary) == 8640
assert set(mode_cells) == expected_mode_cells and len(mode_cells) == 62208
assert len(expected_collapsed_mode_cells) == 20736
assert {r["mode"] for r in cases_actual.values()} == set(LOSS_MODES)
plot_rows = 0
for name, data, kind in [
    ("summary-values.csv", mode_summary, "summary"),
    ("mode-values.csv", mode_effects, "mode"),
    ("mode-cell-values.csv", mode_cells, "leaf"),
]:
    plotted = read(PLOT / name)
    expected = {
        k
        for k, r in data.items()
        if r["score"] == "residual_sigma"
        and r["label"] in LABELS[:2]
        and r["stratum"] in ["filter_single", "fusion_three"]
        and (
            kind == "summary" or (r["target"] == "residual" and r["scope"] == "le_20ms")
        )
    }
    keys = (
        [*NEWRANK, "scope", "unit_kind"]
        + (["unit_id"] if kind != "summary" else [])
        + (["seed", "scenario", "planner"] if kind == "leaf" else [])
    )
    assert {tuple(r[k] for k in keys) for r in plotted} == expected and len(
        plotted
    ) == len(expected)
    for r in plotted:
        fields(r, data[tuple(r[k] for k in keys)])
    plot_rows += len(plotted)
assert plot_rows == 8832
meta = json.loads((PLOT / "plot-sources.json").read_text())
check(sha(ROOT / "plot_event_loss_mode_influence.py"), meta["script_sha256"])
check(sha(MODE_DATA / "sources.json"), meta["analysis_sources_sha256"])
for name, h in meta["inputs_sha256"].items():
    check(sha(MODE_DATA / name), h)
for name, h in meta["outputs_sha256"].items():
    check(sha(PLOT / name), h)
evidence = dict(
    comparisons=checks,
    reconstructed_states=len(focus),
    reconstructed_targets=len(all_data),
    case_rows=len(cases_actual),
    family_rows=len(families_actual),
    prior_precision_scope_rows=len(precision_summary),
    prior_precision_bin_rows=len(precision_bins),
    prior_unit_audit_rows=len(unit_audit),
    prior_delete_one_rows=len(deletions),
    prior_influence_summary_rows=len(influence_summary),
    prior_seed_audit_rows=len(seed_audit),
    prior_seed_effect_rows=len(seed_effects),
    prior_seed_summary_rows=len(seed_summary),
    prior_scenario_audit_rows=len(scenario_audit),
    prior_scenario_effect_rows=len(scenario_effects),
    prior_scenario_summary_rows=len(scenario_summary),
    prior_seed_scenario_pair_rows=len(scenario_cells),
    prior_planner_audit_rows=len(planner_audit),
    prior_planner_effect_rows=len(planner_effects),
    prior_planner_summary_rows=len(planner_summary),
    prior_planner_cell_rows=len(planner_cells),
    prior_focused_scenario_rows=len(expected_collapsed_planner_cells),
    mode_audit_rows=len(mode_audit),
    mode_effect_rows=len(mode_effects),
    summary_rows=len(mode_summary),
    mode_cell_rows=len(mode_cells),
    prior_focused_planner_rows=len(expected_collapsed_mode_cells),
    exact_half_comparisons="Fraction weights from direct NumPy pair-count masks, original parent n_case",
    plotted_rows=plot_rows,
    script_sha256=sha(Path(__file__)),
    sources_sha256=sha(MODE_DATA / "sources.json"),
    plot_sources_sha256=sha(PLOT / "plot-sources.json"),
)
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(evidence, indent=2) + "\n")
print(json.dumps(evidence, indent=2))
