#!/usr/bin/env python3
"""Bound the source first-chord arithmetic at every previously inferred return."""

import argparse
from collections import Counter, defaultdict
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path

import analyze_probe_origin as previous

ROOT, RESULTS, REPO = previous.ROOT, previous.RESULTS, previous.REPO
BASE = RESULTS / "fusion-return-probe-origin"
BASE_SHA = "672d38cb611282f0be3b65f99b2b1bf5815e54c1c7829cbf0a23182b13505112"
PROOF_SHA = "b6dbf95d64cdff2e052c671b48e17e172d54c9d8d7013514291b5369192e450b"
PROTOCOL = RESULTS / "fusion-return-first-chord/protocol.md"
FREEZE = PROTOCOL.with_name("freeze-pins.json")
META, KEY, GROUP = previous.META, previous.KEY, previous.GROUP
PHASES, BRANCHES, CATEGORIES = previous.PHASES, previous.BRANCHES, previous.CATEGORIES
HELPERS = previous.HELPERS
H, P, SEGMENT_PAD = 5e-10, 1e-12, 1e-9
WHEELBASE, LENGTH, FALLBACK = 0.4, 0.05, 1e-6
MASKS = ("boundary_only", "obstacle_only", "both")
KINDS = (
    "audited",
    "direct_unsafe",
    "direct_unknown",
    *MASKS,
    "prior_unknown_direct_unsafe",
    "prior_unknown_direct_unknown",
    "nominal_only_reset_direct_unsafe",
    "nominal_only_reset_direct_unknown",
)
REFS = tuple(
    f"{p}_{b}_first_{k}_refs" for p, b in product(PHASES, BRANCHES) for k in KINDS
)
HAS = tuple("has_available_first_" + m for m in (*MASKS, "unknown"))
FIELDS = (
    "prior_truth_yaw",
    "noise_yaw",
    "bias_yaw",
    "nominal_steering",
    "sensed_yaw_center",
    "sensed_yaw_lo",
    "sensed_yaw_hi",
    "steering_lo",
    "steering_hi",
    "curvature_lo",
    "curvature_hi",
    "coarse_fallback",
    "first_endpoint_x_center",
    "first_endpoint_y_center",
    "first_endpoint_x_lo",
    "first_endpoint_x_hi",
    "first_endpoint_y_lo",
    "first_endpoint_y_hi",
    "first_chord_radius",
    "first_boundary_gap_lo",
    "first_boundary_gap_hi",
    "first_obstacle_gap_lo",
    "first_obstacle_gap_hi",
    "first_definite_wall_ids",
    "first_ambiguous_wall_ids",
    "first_definite_obstacle_ids",
    "first_ambiguous_obstacle_ids",
    "direct_first_test_state",
    "direct_first_failure_mask",
)
sha, read, write, flag, step = (
    previous.sha,
    previous.read,
    previous.write,
    previous.flag,
    previous.step,
)


def source_contract(source):
    import hashlib

    full = previous.previous.extract(source, "fn speed_free_distance(")
    calc = full[
        full.index("        let length = step as f64 * 0.05;") : full.index(
            "        if [point.x"
        )
    ]
    assert "let curvature = steering.tan() / WHEELBASE;" in full
    assert (
        "curvature.abs() < 1e-9" in calc
        and "(yaw.sin() - sensed.yaw.sin()) / curvature" in calc
    )
    assert "(yaw.cos() - sensed.yaw.cos()) / curvature" in calc
    assert "sensed.yaw + curvature * length" in calc
    assert "const WHEELBASE: f64 = 0.4;" in source
    assert source.index("let nominal_steering = steering;") < source.index(
        "if safety_paused {\n                    target_speed = 0.0;"
    )
    return dict(
        full_probe_sha256=hashlib.sha256(full.encode()).hexdigest(),
        first_endpoint_arithmetic_sha256=hashlib.sha256(calc.encode()).hexdigest(),
    )


def add(a, b):
    return a[0] + b[0] - P, a[1] + b[1] + P


def sub(a, b):
    return a[0] - b[1] - P, a[1] - b[0] + P


def mul(a, b):
    values = [x * y for x, y in product(a, b)]
    return min(values) - P, max(values) + P


def div(a, b):
    assert not b[0] <= 0 <= b[1]
    values = [x / y for x, y in product(a, b)]
    return min(values) - P, max(values) + P


def trig(pair, cosine=False):
    lo, hi = pair
    assert lo <= hi
    center = (lo + hi) / 2
    radius = (hi - lo) / 2
    value = math.cos(center) if cosine else math.sin(center)
    return max(-1.0, value - radius - P), min(1.0, value + radius + P)


def angles(before, current):
    values = {
        "prior_truth_yaw": before["yaw"],
        "noise_yaw": current["noise_yaw"],
        "bias_yaw": current["bias_yaw"],
        "nominal_steering": current["nominal_steering"],
    }
    assert all(
        math.isfinite(float(v)) and abs(float(v)) <= 100 for v in values.values()
    )
    delta = float(current["nominal_steering"])
    assert abs(delta) <= 0.612
    yaw = (float(before["yaw"]) + float(current["noise_yaw"])) + float(
        current["bias_yaw"]
    )
    assert abs(yaw) <= 100
    return dict(
        **values,
        sensed_yaw_center=yaw,
        sensed_yaw_lo=yaw - 3 * H - P,
        sensed_yaw_hi=yaw + 3 * H + P,
        steering_lo=delta - H,
        steering_hi=delta + H,
    )


def endpoint(a, angle):
    x, y = float(a["sensed_x_center"]), float(a["sensed_y_center"])
    yaw, delta = angle["sensed_yaw_center"], float(angle["nominal_steering"])
    k = math.tan(delta) / WHEELBASE
    theta = yaw + k * LENGTH
    center = (
        (x + LENGTH * math.cos(yaw), y + LENGTH * math.sin(yaw))
        if abs(k) < 1e-9
        else (
            x + (math.sin(theta) - math.sin(yaw)) / k,
            y - (math.cos(theta) - math.cos(yaw)) / k,
        )
    )
    start_x = (float(a["sensed_x_lo"]), float(a["sensed_x_hi"]))
    start_y = (float(a["sensed_y_lo"]), float(a["sensed_y_hi"]))
    steering = (angle["steering_lo"], angle["steering_hi"])
    curvature = div(
        (math.tan(steering[0]) - P, math.tan(steering[1]) + P), (WHEELBASE, WHEELBASE)
    )
    coarse = curvature[0] <= FALLBACK and curvature[1] >= -FALLBACK
    if coarse:
        end_x = (start_x[0] - 0.06, start_x[1] + 0.06)
        end_y = (start_y[0] - 0.06, start_y[1] + 0.06)
    else:
        yr = (angle["sensed_yaw_lo"], angle["sensed_yaw_hi"])
        theta_range = add(yr, mul(curvature, (LENGTH, LENGTH)))
        end_x = add(start_x, div(sub(trig(theta_range), trig(yr)), curvature))
        end_y = sub(
            start_y, div(sub(trig(theta_range, True), trig(yr, True)), curvature)
        )
    assert end_x[0] <= center[0] <= end_x[1] and end_y[0] <= center[1] <= end_y[1]
    start_error = math.hypot(
        max(abs(start_x[0] - x), abs(start_x[1] - x)),
        max(abs(start_y[0] - y), abs(start_y[1] - y)),
    )
    end_error = math.hypot(
        max(abs(end_x[0] - center[0]), abs(end_x[1] - center[0])),
        max(abs(end_y[0] - center[1]), abs(end_y[1] - center[1])),
    )
    return dict(
        curvature_lo=curvature[0],
        curvature_hi=curvature[1],
        coarse_fallback=coarse,
        first_endpoint_x_center=center[0],
        first_endpoint_y_center=center[1],
        first_endpoint_x_lo=end_x[0],
        first_endpoint_x_hi=end_x[1],
        first_endpoint_y_lo=end_y[0],
        first_endpoint_y_hi=end_y[1],
        first_chord_radius=max(start_error, end_error) + P,
    )


def segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    squared = dx * dx + dy * dy
    if squared < 1e-20:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    t = max(
        0.0,
        min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / squared),
    )
    return math.hypot(point[0] - (start[0] + t * dx), point[1] - (start[1] + t * dy))


def geometry(a, e, obstacles):
    xl, xh, yl, yh = (
        e[k]
        for k in (
            "first_endpoint_x_lo",
            "first_endpoint_x_hi",
            "first_endpoint_y_lo",
            "first_endpoint_y_hi",
        )
    )
    margin = float(a["speed_margin"])
    assert 0 < margin < 10
    ml, mh = margin - H, margin + H
    walls = (
        (xl - mh - SEGMENT_PAD, xh - ml + SEGMENT_PAD),
        (yl - mh - SEGMENT_PAD, yh - ml + SEGMENT_PAD),
        (50 - xh - mh - SEGMENT_PAD, 50 - xl - ml + SEGMENT_PAD),
        (50 - yh - mh - SEGMENT_PAD, 50 - yl - ml + SEGMENT_PAD),
    )
    wall_yes = [
        name
        for name, pair in zip(previous.WALLS, walls)
        if previous.classify(pair) == "unsafe"
    ]
    wall_maybe = [
        name
        for name, pair in zip(previous.WALLS, walls)
        if previous.classify(pair) == "unknown"
    ]
    start = (float(a["sensed_x_center"]), float(a["sensed_y_center"]))
    end = (e["first_endpoint_x_center"], e["first_endpoint_y_center"])
    low_values, high_values, yes, maybe = [], [], [], []
    for oid, x, y in obstacles:
        distance = segment_distance((x, y), start, end)
        low = max(0.0, distance - e["first_chord_radius"]) - mh - SEGMENT_PAD
        high = distance + e["first_chord_radius"] - ml + SEGMENT_PAD
        low_values.append(low)
        high_values.append(high)
        state = previous.classify((low, high))
        if state == "unsafe":
            yes.append(oid)
        if state == "unknown":
            maybe.append(oid)
    boundary = min(p[0] for p in walls), min(p[1] for p in walls)
    obstacle = (min(low_values), min(high_values)) if low_values else None
    total = (
        (min(boundary[0], obstacle[0]), min(boundary[1], obstacle[1]))
        if obstacle
        else boundary
    )
    state = previous.classify(total)
    assert state != "safe", "certainly safe chord contradicts the preceding zero probe"
    mask = (
        "both"
        if wall_yes and yes
        else "boundary_only" if wall_yes else "obstacle_only" if yes else ""
    )
    assert (state == "unsafe") == bool(mask)
    return dict(
        first_boundary_gap_lo=boundary[0],
        first_boundary_gap_hi=boundary[1],
        first_obstacle_gap_lo=obstacle[0] if obstacle else None,
        first_obstacle_gap_hi=obstacle[1] if obstacle else None,
        first_definite_wall_ids=";".join(wall_yes),
        first_ambiguous_wall_ids=";".join(wall_maybe),
        first_definite_obstacle_ids=";".join(yes),
        first_ambiguous_obstacle_ids=";".join(maybe),
        direct_first_test_state=state,
        direct_first_failure_mask=mask,
    )


def build(old, before=None, current=None, obstacles=()):
    out = dict(
        old, first_chord_audited=old["probe_origin"] == "first_chord_by_exclusion"
    )
    out.update(dict.fromkeys(FIELDS))
    if not out["first_chord_audited"]:
        assert before is None and current is None
        return out
    assert old["initial_test_state"] == "safe"
    for key in old.keys() & current.keys():
        if key in META:
            assert old[key] == current[key]
        elif current[key] in ("true", "false"):
            assert flag(old[key]) == flag(current[key], raw=True)
        else:
            assert float(old[key]) == float(current[key]), key
    p = previous.pose(before, current)
    for key in p:
        assert (
            math.isclose(float(old[key]), float(p[key]), abs_tol=2e-12)
            if key
            not in (
                "prior_truth_x",
                "prior_truth_y",
                "noise_x",
                "noise_y",
                "bias_x",
                "bias_y",
                "speed_margin",
            )
            else old[key] == p[key]
        )
    an = angles(before, current)
    ep = endpoint(old, an)
    geo = geometry(old, ep, obstacles)
    out.update(an)
    out.update(ep)
    out.update(geo)
    assert set(an) | set(ep) | set(geo) == set(FIELDS)
    return out


def weights(e, ne, nr):
    parent = int(e["parent_rp_records"])
    result = dict(
        census_case_weight=Fraction(1, ne),
        received_case_weight=Fraction(1, nr) if flag(e["received"]) else Fraction(),
        rp_case_mass=Fraction(int(e["rp_records"]), parent) if parent else Fraction(),
        rp_different_preview_case_mass=(
            Fraction(int(e["rp_different_preview_records"]), parent)
            if parent
            else Fraction()
        ),
    )
    assert all(
        math.isclose(float(e[k]), float(value), abs_tol=2e-12)
        for k, value in result.items()
    )
    return result


def anchor(e, dd, ne, nr):
    out = dict(e, **weights(e, ne, nr))
    out.update(dict.fromkeys((*REFS, *HAS)))
    if not flag(e["returned"] or "False"):
        assert not dd
        return out
    assert all(all(d[k] == e[k] for k in META) for d in dd)
    counts = Counter()
    for d in dd:
        if not d["first_chord_audited"]:
            continue
        prefix = d["phase"] + "_" + d["branch"] + "_first_"
        kinds = ["audited", "direct_" + d["direct_first_test_state"]]
        if d["direct_first_failure_mask"]:
            kinds.append(d["direct_first_failure_mask"])
        if d["prior_certainty"] == "unknown":
            kinds.append("prior_unknown_direct_" + d["direct_first_test_state"])
        if d["timer_action"] == "reset" and d["refined_failed_gates"] == "nominal":
            kinds.append("nominal_only_reset_direct_" + d["direct_first_test_state"])
        for kind in kinds:
            counts[prefix + kind + "_refs"] += 1
    out.update({k: counts[k] for k in REFS})
    for kind in (*MASKS, "unknown"):
        out["has_available_first_" + kind] = any(
            d["phase"] == "available"
            and d["first_chord_audited"]
            and (
                d["direct_first_test_state"] == "unknown"
                if kind == "unknown"
                else d["direct_first_failure_mask"] == kind
            )
            for d in dd
        )
    return out


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


def closure(rows):
    lookup = {tuple(r[k] for k in GROUP): r for r in rows}
    assert len(lookup) == len(rows)
    checks = 0
    for key, r in lookup.items():
        for i, whole, parts in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", CATEGORIES),
        ):
            if key[i] == whole:
                children = [lookup[key[:i] + (p,) + key[i + 1 :]] for p in parts]
                for field in (
                    *REFS,
                    *(h + s for h in HAS for s in ("_events", "_case_mass")),
                ):
                    assert r[field] == sum(c[field] for c in children), (key, field)
                    checks += 1
        for p, b in product(PHASES, BRANCHES):
            prefix = p + "_" + b + "_first_"
            assert (
                r[prefix + "audited_refs"]
                == r[prefix + "direct_unsafe_refs"] + r[prefix + "direct_unknown_refs"]
            )
            assert (
                sum(r[prefix + m + "_refs"] for m in MASKS)
                == r[prefix + "direct_unsafe_refs"]
            )
            assert r[prefix + "audited_refs"] == int(
                r[p + "_" + b + "_first_chord_by_exclusion_refs"]
            )
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--raw-roots", type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Use a new output directory and run without -O")
    freeze = json.loads(FREEZE.read_text())
    assert freeze["sources_sha256"] == BASE_SHA and freeze["proof_sha256"] == PROOF_SHA
    assert (
        sha(BASE / "sources.json") == BASE_SHA
        and sha(BASE / "independent-verification.json") == PROOF_SHA
    )
    prior = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    pins = {
        **prior["input_sha256"],
        "fusion-return-probe-origin/sources.json": BASE_SHA,
        "fusion-return-probe-origin/independent-verification.json": PROOF_SHA,
        "fusion-return-probe-origin/protocol.md": prior["analysis"]["protocol_sha256"],
        **{
            "fusion-return-probe-origin/" + k: h
            for k, h in prior["output_sha256"].items()
        },
    }
    helpers = {
        **prior["analysis"]["helpers_sha256"],
        "analyze_probe_origin.py": prior["analysis"]["script_sha256"],
    }
    for name, h in pins.items():
        assert sha(RESULTS / name) == h, name
    for name, h in {**helpers, **HELPERS}.items():
        assert sha(ROOT / name) == h, name
    for name, h in prior["analysis"]["code_sha256"].items():
        assert sha(REPO / name) == h, name
    contract = source_contract(
        (
            REPO / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
        ).read_text()
    )
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(prior["raw"])
    previous.provenance(prior["raw"], roots, proof["native"]["rustc"])
    obstacles = previous.maps(BASE / "obstacle-map.csv")
    cases = read(RESULTS / "census-rp-selection/case-selection.csv")
    events = read(BASE / "anchor-probe-origin.csv")
    old = read(BASE / "probe-decisions.csv")
    selected = [d for d in old if d["probe_origin"] == "first_chord_by_exclusion"]
    assert len(selected) == 61
    points, audit = previous.collect(cases, selected, prior["raw"], roots)
    decisions = [
        (
            build(
                d,
                points[d["case_id"], step(d["time_s"]) - 1],
                points[d["case_id"], step(d["time_s"])],
                obstacles[d["scenario"]],
            )
            if d["probe_origin"] == "first_chord_by_exclusion"
            else build(d)
        )
        for d in old
    ]
    by_event = defaultdict(list)
    for d in decisions:
        by_event[d["event_id"]].append(d)
    ne = Counter(e["case_id"] for e in events)
    nr = Counter(e["case_id"] for e in events if flag(e["received"]))
    anchors = [
        anchor(e, by_event[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]])
        for e in events
    ]
    summary = summaries(read(BASE / "probe-summary.csv"), anchors)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(flag(a["returned"] or "False") for a in anchors),
        decision_references=len(decisions),
        audited_first_chords=61,
        available_audited=sum(
            d["first_chord_audited"] and d["phase"] == "available" for d in decisions
        ),
        boundary_audited=sum(
            d["first_chord_audited"] and d["phase"] == "next_loss" for d in decisions
        ),
        summary_rows=len(summary),
        new_additive_closure_comparisons=closure(summary),
    )
    assert (
        len(cases),
        len(ne),
        len(events),
        len(decisions),
        audit["available_audited"],
        audit["boundary_audited"],
        len(summary),
    ) == (972, 570, 3997, 3751, 60, 1, 336)
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
        ("first-chord-decisions", decisions),
        ("anchor-first-chord", anchors),
        ("first-chord-summary", summary),
    ):
        write(args.output / (name + ".csv"), rows)
    source = dict(
        input_sha256=pins,
        raw=prior["raw"],
        map_sha256=prior["map_sha256"],
        analysis=dict(
            script_sha256=sha(Path(__file__)),
            protocol_sha256=sha(PROTOCOL),
            freeze_sha256=sha(FREEZE),
            helpers_sha256=helpers,
            verifier_helpers_sha256=HELPERS,
            code_sha256=prior["analysis"]["code_sha256"],
            source_contract=contract,
        ),
        schema=dict(
            half_unit=H,
            elemental_allowance=P,
            segment_allowance=SEGMENT_PAD,
            curvature_fallback=FALLBACK,
            fallback_expansion=0.06,
            phases=PHASES,
            branches=BRANCHES,
            retrospective=True,
            inherited="all preceding columns and source-path exclusion; direct first-test fields separate; numerical allowances conditional on stated platform assumptions",
        ),
        inherited_probe_proof_sha256=PROOF_SHA,
        inherited_validation=prior["inherited_validation"],
        audit=audit,
        output_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
