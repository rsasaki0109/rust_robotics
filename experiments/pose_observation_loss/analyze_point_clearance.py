#!/usr/bin/env python3
"""Bound adjacent point gaps to the inherited fixed obstacle sets."""

import argparse
from collections import Counter, defaultdict
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path

import analyze_window_turnover as prior

ROOT, REPO, RESULTS = prior.ROOT, prior.REPO, prior.RESULTS
BASE = RESULTS / "fusion-return-window-turnover"
STUDY = RESULTS / "fusion-return-point-clearance"
BASE_SHA = "ce946dad8570b16cc0ca12a8f2d41c526801ec560b620c386fda8e9ed9688f9e"
PROOF_SHA = "f9bd4b69aaf1aee2c2b1ba187b8a20704957ea30611aa548dce92c74a7cc6bb3"
META, KEY, GROUP = prior.META, prior.KEY, prior.GROUP
PHASES, BRANCHES, CATEGORIES = prior.PHASES, prior.BRANCHES, prior.CATEGORIES
STATES = ("clear", "blocked", "unknown")
TRANSITIONS = tuple(a + "_to_" + b for a, b in product(STATES, repeat=2))
DIRECTIONS = ("toward", "away", "unresolved")
GAP_DIRECTIONS = ("narrower", "wider", "unresolved")
MARGIN_DIRECTIONS = ("decrease", "increase", "unresolved")
KINDS = (
    "audited",
    *(k + "_" + t for k in ("raw", "mean") for t in TRANSITIONS),
    *(k + "_distance_" + t for k in ("raw", "mean") for t in DIRECTIONS),
    *(k + "_gap_" + t for k in ("raw", "mean") for t in GAP_DIRECTIONS),
    *("margin_" + t for t in MARGIN_DIRECTIONS),
    "prior_unknown_raw_narrower",
    "nominal_only_reset_raw_narrower",
)
REFS = tuple(
    f"{p}_{b}_point_{k}_refs" for p, b in product(PHASES, BRANCHES) for k in KINDS
)
HAS = tuple(
    "has_available_point_" + k
    for k in ("raw_clear_to_blocked", "mean_clear_to_blocked", "raw_toward_mean_away")
)
H, P, R = prior.H, prior.P, prior.R
sha, read, write, flag, step = (
    prior.sha,
    prior.read,
    prior.write,
    prior.flag,
    prior.step,
)

METRICS = (
    *(
        f"{k}_{c}_{q}_{b}"
        for k in ("raw", "mean")
        for c in ("previous", "current")
        for q in ("distance", "gap")
        for b in ("center", "lo", "hi")
    ),
    *(f"{k}_{c}_state" for k in ("raw", "mean") for c in ("previous", "current")),
    *(f"{k}_transition" for k in ("raw", "mean")),
    *(
        f"{k}_{q}_change_{b}"
        for k in ("raw", "mean")
        for q in ("distance", "gap")
        for b in ("center", "lo", "hi")
    ),
    *(f"{k}_{q}_direction" for k in ("raw", "mean") for q in ("distance", "gap")),
    *(f"{k}_gap_closure_residual" for k in ("raw", "mean")),
)
FIELDS = (
    "fixed_obstacle_ids",
    "fixed_obstacle_count",
    *(
        f"point_{c}_margin{b}"
        for c in ("previous", "current")
        for b in ("", "_lo", "_hi")
    ),
    *("point_margin_change_" + b for b in ("center", "lo", "hi")),
    "point_margin_direction",
    *(
        f"point_{k}_{c}_{a}_{b}"
        for k in ("raw", "mean")
        for c in ("previous", "current")
        for a in ("x", "y")
        for b in ("center", "lo", "hi")
    ),
    *METRICS,
    *(
        f"{k}_{c}_{t}_ids"
        for k in ("raw", "mean")
        for c in ("previous", "current")
        for t in ("minimum", "possible_minimum")
    ),
)


def contract(source):
    return dict(
        window=prior.prior.contract(source), margin=prior.prior.prior.contract(source)
    )


def classify(pair, negative, positive):
    return negative if pair[1] < 0 else positive if pair[0] > 0 else "unresolved"


def state(pair):
    return "clear" if pair[0] > 0 else "blocked" if pair[1] <= 0 else "unknown"


def subtract(a, b):
    return a[0] - b[1] - P, a[1] - b[0] + P


def distance_box(center, radius, obstacle):
    offsets = [
        (v - o - radius - P, v - o + radius + P) for v, o in zip(center, obstacle)
    ]
    lo = math.sqrt(
        sum((0 if a <= 0 <= b else min(abs(a), abs(b))) ** 2 for a, b in offsets)
    )
    hi = math.sqrt(sum(max(abs(a), abs(b)) ** 2 for a, b in offsets))
    value = math.sqrt(sum((v - o) ** 2 for v, o in zip(center, obstacle)))
    return value, max(0.0, lo - P), hi + P


def bounds(values):
    return values[1], values[2]


def triple_sub(a, b):
    return a[0] - b[0], *subtract(bounds(a), bounds(b))


def metrics(distances, margins):
    out = {}
    margin_change = triple_sub(margins["current"], margins["previous"])
    for kind in ("raw", "mean"):
        gaps = {}
        for context in ("previous", "current"):
            distance = distances[kind, context]
            gaps[context] = triple_sub(distance, margins[context])
            for q, values in (("distance", distance), ("gap", gaps[context])):
                out.update(
                    {
                        f"{kind}_{context}_{q}_{b}": v
                        for b, v in zip(("center", "lo", "hi"), values)
                    }
                )
            out[f"{kind}_{context}_state"] = state(bounds(gaps[context]))
        out[kind + "_transition"] = (
            out[kind + "_previous_state"] + "_to_" + out[kind + "_current_state"]
        )
        changes = {}
        for q, values, labels in (
            (
                "distance",
                {c: distances[kind, c] for c in ("previous", "current")},
                ("toward", "away"),
            ),
            ("gap", gaps, ("narrower", "wider")),
        ):
            change = triple_sub(values["current"], values["previous"])
            changes[q] = change
            out.update(
                {
                    f"{kind}_{q}_change_{b}": v
                    for b, v in zip(("center", "lo", "hi"), change)
                }
            )
            out[f"{kind}_{q}_direction"] = classify(bounds(change), *labels)
        prediction = triple_sub(changes["distance"], margin_change)
        residual = changes["gap"][0] - prediction[0]
        assert abs(residual) <= P
        assert prediction[1] <= changes["gap"][2] and prediction[2] >= changes["gap"][1]
        out[kind + "_gap_closure_residual"] = residual
    assert set(out) == set(METRICS)
    return out


def check_contexts(d, before, after):
    prior.check_current(d, after)
    assert int(d["previous_count"]) == len(before["members"])
    for axis in ("x", "y"):
        for name, value in (
            ("mean", before["fields"]["mean_" + axis + "_center"]),
            ("sensed", before["point"]["sensed_" + axis + "_center"]),
        ):
            assert float(d[f"previous_{name}_{axis}_center"]) == float(value)
    for context, snap in (("previous", before), ("current", after)):
        for k in prior.RAW:
            assert d[context + "_input_" + k] == snap["point"][k]


def geometry(d, before, after, obstacles):
    ids = prior.prior.prior.blockers(d)
    omap = {oid: (x, y) for oid, x, y in obstacles}
    assert len(omap) == len(obstacles) and set(ids) <= omap.keys()
    out = dict(fixed_obstacle_ids=";".join(ids), fixed_obstacle_count=len(ids))
    points, margins = {}, {}
    for context, snap in (("previous", before), ("current", after)):
        margin = snap["raw"]["speed_margin"]
        m = float(margin)
        assert math.isfinite(m) and m >= 0
        margins[context] = (m, m - H, m + H)
        out.update(
            {
                f"point_{context}_margin": margin,
                f"point_{context}_margin_lo": m - H,
                f"point_{context}_margin_hi": m + H,
            }
        )
        for kind in ("raw", "mean"):
            center = tuple(
                float(
                    snap["point"]["sensed_" + a + "_center"]
                    if kind == "raw"
                    else snap["fields"]["mean_" + a + "_center"]
                )
                for a in ("x", "y")
            )
            radius = R if kind == "raw" or not snap["members"] else R + 2 * P
            points[kind, context] = (center, radius)
            for axis, value in zip(("x", "y"), center):
                out.update(
                    {
                        f"point_{kind}_{context}_{axis}_{b}": v
                        for b, v in zip(
                            ("center", "lo", "hi"),
                            (value, value - radius, value + radius),
                        )
                    }
                )
    change = triple_sub(margins["current"], margins["previous"])
    out.update(
        {"point_margin_change_" + b: v for b, v in zip(("center", "lo", "hi"), change)}
    )
    out["point_margin_direction"] = classify(bounds(change), "decrease", "increase")
    by_id = {
        oid: {key: distance_box(*points[key], omap[oid]) for key in points}
        for oid in ids
    }
    distances = {}
    for kind, context in points:
        vals = {oid: by_id[oid][kind, context] for oid in ids}
        minimum = tuple(min(v[i] for v in vals.values()) for i in range(3))
        distances[kind, context] = minimum
        out[f"{kind}_{context}_minimum_ids"] = ";".join(
            i for i in ids if vals[i][0] == minimum[0]
        )
        out[f"{kind}_{context}_possible_minimum_ids"] = ";".join(
            i for i in ids if vals[i][1] <= minimum[2]
        )
    out.update(metrics(distances, margins))
    witnesses = []
    for order, oid in enumerate(ids, 1):
        witnesses.append(
            dict(
                **{k: d[k] for k in (*META, "event_id", "time_s", "phase", "branch")},
                obstacle_order=order,
                obstacle_id=oid,
                obstacle_x=omap[oid][0],
                obstacle_y=omap[oid][1],
                **metrics(by_id[oid], margins),
            )
        )
    assert set(out) == set(FIELDS)
    return out, witnesses


def anchor(e, dd, ne, nr):
    out = dict(e, **prior.weights(e, ne, nr), **dict.fromkeys((*REFS, *HAS)))
    if not flag(e["returned"] or "False"):
        assert not dd
        return out
    assert all(all(d[k] == e[k] for k in META) for d in dd)
    counts = Counter()
    for d in dd:
        if not d["point_clearance_audited"]:
            continue
        kinds = [
            "audited",
            *(k + "_" + d[k + "_transition"] for k in ("raw", "mean")),
            *(
                k + "_" + q + "_" + d[k + "_" + q + "_direction"]
                for k in ("raw", "mean")
                for q in ("distance", "gap")
            ),
            "margin_" + d["point_margin_direction"],
        ]
        if d["raw_gap_direction"] == "narrower":
            if d["prior_certainty"] == "unknown":
                kinds.append("prior_unknown_raw_narrower")
            if d["timer_action"] == "reset" and d["refined_failed_gates"] == "nominal":
                kinds.append("nominal_only_reset_raw_narrower")
        for k in kinds:
            counts[f"{d['phase']}_{d['branch']}_point_{k}_refs"] += 1
    out.update({k: counts[k] for k in REFS})
    rows = [d for d in dd if d["phase"] == "available" and d["point_clearance_audited"]]
    for kind in ("raw", "mean"):
        out[f"has_available_point_{kind}_clear_to_blocked"] = any(
            d[kind + "_transition"] == "clear_to_blocked" for d in rows
        )
    out["has_available_point_raw_toward_mean_away"] = any(
        d["raw_distance_direction"] == "toward"
        and d["mean_distance_direction"] == "away"
        for d in rows
    )
    return out


def closure(rows):
    db = {tuple(r[k] for k in GROUP): r for r in rows}
    assert len(db) == len(rows)
    checks = 0
    for key, row in db.items():
        for i, whole, parts in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", CATEGORIES),
        ):
            if key[i] == whole:
                for field in (
                    *REFS,
                    *(h + s for h in HAS for s in ("_events", "_case_mass")),
                ):
                    assert row[field] == sum(
                        db[key[:i] + (p,) + key[i + 1 :]][field] for p in parts
                    )
                    checks += 1
        for p, b in product(PHASES, BRANCHES):
            pre = f"{p}_{b}_point_"
            for group in (
                *(tuple(k + "_" + t for t in TRANSITIONS) for k in ("raw", "mean")),
                *(
                    tuple(k + "_distance_" + t for t in DIRECTIONS)
                    for k in ("raw", "mean")
                ),
                *(
                    tuple(k + "_gap_" + t for t in GAP_DIRECTIONS)
                    for k in ("raw", "mean")
                ),
                tuple("margin_" + t for t in MARGIN_DIRECTIONS),
            ):
                assert row[pre + "audited_refs"] == sum(
                    row[pre + k + "_refs"] for k in group
                )
    return checks


def summaries(old, anchors):
    rows = []
    for fixed in old:
        row = dict(fixed)
        p, n, arm, mode, category = (row[k] for k in GROUP)
        aa = [
            a
            for a in anchors
            if (a["partition"], a["noise"], a["variant"]) == (p, n, arm)
            and (mode == "all_modes" or a["mode"] == mode)
            and (category == "all" or a["anchor_category"] == category)
        ]
        returned = [a for a in aa if flag(a["returned"] or "False")]
        mass = lambda rr: sum((a["census_case_weight"] for a in rr), Fraction())
        row.update({k: sum(a[k] or 0 for a in aa) for k in REFS})
        for name in HAS:
            selected = [a for a in aa if a[name]]
            row.update(
                {
                    name + "_events": len(selected),
                    name + "_case_mass": mass(selected),
                    name
                    + "_fraction": (
                        Fraction(len(selected), len(returned)) if returned else None
                    ),
                    name
                    + "_case_mass_fraction": (
                        mass(selected) / mass(returned) if returned else None
                    ),
                }
            )
        assert math.isclose(
            float(fixed["census_case_mass"]), float(mass(aa)), abs_tol=2e-12
        )
        rows.append(row)
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--raw-roots", type=Path)
    args = p.parse_args()
    if not __debug__ or args.output.exists():
        p.error("Use a new output directory without -O")
    freeze = json.loads((STUDY / "freeze-pins.json").read_text())
    assert freeze["sources_sha256"] == sha(BASE / "sources.json") == BASE_SHA
    assert (
        freeze["proof_sha256"]
        == sha(BASE / "independent-verification.json")
        == PROOF_SHA
    )
    source = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    pins = {
        **source["input_sha256"],
        **{
            "fusion-return-window-turnover/" + n: h
            for n, h in source["output_sha256"].items()
        },
        "fusion-return-window-turnover/sources.json": BASE_SHA,
        "fusion-return-window-turnover/independent-verification.json": PROOF_SHA,
        "fusion-return-window-turnover/protocol.md": source["analysis"][
            "protocol_sha256"
        ],
        "fusion-return-window-turnover/freeze-pins.json": source["analysis"][
            "freeze_sha256"
        ],
    }
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_window_turnover.py": source["analysis"]["script_sha256"],
    }
    vhelpers = {
        **source["analysis"]["verifier_helpers_sha256"],
        "verify_window_turnover.py": freeze["verifier_helper_sha256"],
    }
    for n, h in pins.items():
        assert sha(RESULTS / n) == h, n
    for n, h in {**helpers, **vhelpers}.items():
        assert sha(ROOT / n) == h, n
    for n, h in source["analysis"]["code_sha256"].items():
        assert sha(REPO / n) == h, n
    source_contract = contract(
        (REPO / next(iter(source["analysis"]["code_sha256"]))).read_text()
    )
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    prior.probe.provenance(source["raw"], roots, proof["native"]["rustc"])
    cases = read(RESULTS / "census-rp-selection/case-selection.csv")
    old = read(BASE / "turnover-decisions.csv")
    selected = [d for d in old if float(d["speed_free_distance"]) == 0]
    assert len(selected) == 1096
    snapshots, audit = prior.collect(cases, selected, source["raw"], roots)
    decisions = []
    witnesses = []
    map_path = RESULTS / "fusion-return-probe-origin/obstacle-map.csv"
    obstacles = prior.probe.maps(map_path)
    for d in old:
        audited = float(d["speed_free_distance"]) == 0
        out = dict(d, point_clearance_audited=audited, **dict.fromkeys(FIELDS))
        if audited:
            n = step(d["time_s"])
            before = snapshots[d["case_id"], n - 1]
            after = snapshots[d["case_id"], n]
            check_contexts(d, before, after)
            fields, lines = geometry(d, before, after, obstacles[d["scenario"]])
            out.update(fields)
            witnesses.extend(lines)
        decisions.append(out)
    events = read(BASE / "anchor-turnover.csv")
    ne = Counter(e["case_id"] for e in events)
    nr = Counter(e["case_id"] for e in events if flag(e["received"]))
    db = defaultdict(list)
    for d in decisions:
        db[d["event_id"]].append(d)
    anchors = [
        anchor(e, db[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]]) for e in events
    ]
    summary = summaries(read(BASE / "turnover-summary.csv"), anchors)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(flag(e["returned"] or "False") for e in events),
        decision_references=len(decisions),
        audited_zero_probes=len(selected),
        context_queries=2 * len(selected),
        witness_rows=len(witnesses),
        summary_rows=len(summary),
        new_additive_closure_comparisons=closure(summary),
    )
    assert (
        len(cases),
        len(ne),
        len(anchors),
        len(decisions),
        len(summary),
        audit["replayed_histories"],
        audit["replayed_steps"],
    ) == (972, 570, 3997, 3751, 336, 63, 567000)
    assert (
        sum(a["census_case_weight"] for a in anchors)
        == sum(a["received_case_weight"] for a in anchors)
        == 570
    )
    assert (
        sum(a["rp_case_mass"] for a in anchors) == 157
        and sum(int(a["rp_records"]) for a in anchors) == 10995
    )
    args.output.mkdir(parents=True)
    for name, rows in (
        ("point-decisions", decisions),
        ("point-witnesses", witnesses),
        ("anchor-point", anchors),
        ("point-summary", summary),
    ):
        write(args.output / (name + ".csv"), rows)
    out = dict(
        input_sha256=pins,
        raw=source["raw"],
        map_sha256=source["map_sha256"],
        analysis=dict(
            script_sha256=sha(Path(__file__)),
            protocol_sha256=sha(STUDY / "protocol.md"),
            freeze_sha256=sha(STUDY / "freeze-pins.json"),
            helpers_sha256=helpers,
            verifier_helpers_sha256=vhelpers,
            code_sha256=source["analysis"]["code_sha256"],
            source_contract=source_contract,
        ),
        schema=dict(
            half_unit=H,
            elemental_allowance=P,
            sensed_radius=R,
            closure_allowance=P,
            states=STATES,
            transitions=TRANSITIONS,
            distance_directions=DIRECTIONS,
            gap_directions=GAP_DIRECTIONS,
            margin_directions=MARGIN_DIRECTIONS,
            phases=PHASES,
            branches=BRANCHES,
            retrospective=True,
            inherited="all prior columns and outcomes; fixed original blocker sets; predecessor context only; point gaps do not replace chord/segment/probe predicates",
        ),
        inherited_window_turnover_proof_sha256=PROOF_SHA,
        inherited_validation=source["inherited_validation"],
        audit=audit,
        output_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
