#!/usr/bin/env python3
"""Compare the original zero-probe obstacles with the logged recovery pose."""

import argparse
from collections import Counter, defaultdict
from fractions import Fraction
from itertools import product
import hashlib
import json
import math
from pathlib import Path

import analyze_first_chord as prior

ROOT, REPO, RESULTS = prior.ROOT, prior.REPO, prior.RESULTS
BASE = RESULTS / "fusion-return-first-chord"
STUDY = RESULTS / "fusion-return-recovery-geometry"
BASE_SHA = "8e2697be1401ca6b576a9fe75163e5823d25474e7a352bb5b49e05fd9d2ee6c4"
PROOF_SHA = "ad7182e599dc7d494c716f4ab7c06770a731833dddf06cd5e798f28edcdf6ec1"
ADMISSION = "fusion-return-admission/admission-decisions.csv"
META, KEY, GROUP = prior.META, prior.KEY, prior.GROUP
PHASES, BRANCHES, CATEGORIES = prior.PHASES, prior.BRANCHES, prior.CATEGORIES
H, P, S = 5e-10, 1e-12, 1e-9
STATES = ("clear", "blocked", "unknown")
KINDS = (
    "audited",
    *("blockers_" + s for s in STATES),
    *("allmap_" + s for s in STATES),
    "prior_unknown_clear",
    "nominal_only_reset_clear",
)
REFS = tuple(
    f"{p}_{b}_recovery_{k}_refs" for p, b in product(PHASES, BRANCHES) for k in KINDS
)
HAS = tuple("has_available_recovery_blockers_" + s for s in STATES)
INPUTS = (
    "recovery_x",
    "recovery_y",
    "recovery_yaw",
    "recovery_clearance",
    "recovery_sample_count",
)
FIELDS = (
    *INPUTS,
    "original_blocker_ids",
    *(f"recovery_minus_raw_{a}_{s}" for a in ("x", "y") for s in ("lo", "hi")),
    "pose_displacement_lo",
    "pose_displacement_hi",
    *(f"recovery_endpoint_{a}_{s}" for a in ("x", "y") for s in ("center", "lo", "hi")),
    "recovery_segment_radius",
    *(
        f"{k}_gap_{s}"
        for k in ("blocker_start", "blocker_segment", "allmap")
        for s in ("lo", "hi")
    ),
    "allmap_clearance_lo",
    "allmap_clearance_hi",
    "blocker_start_state",
    "blocker_segment_state",
    "allmap_state",
    "recovery_definite_obstacle_ids",
    "recovery_ambiguous_obstacle_ids",
)
sha, read, write, flag, step = (
    prior.sha,
    prior.read,
    prior.write,
    prior.flag,
    prior.step,
)


def contract(source):
    a = source.split("let recovery_margin = ", 1)[1].split(";", 1)[0]
    b = source.split("let speed_margin = if anticipatory_speed {", 1)[1].split(
        "} else", 1
    )[0]
    assert "".join(a.split()) == "".join(b.split())
    for text in (
        "speed_free_distance(sensed, steering, speed_margin, obs)",
        "if averaged_recovery && recovery_sample_count > 0",
        "recovery_samples.iter().map(|p| p.0).sum::<f64>() / n",
        "recovery_samples.iter().map(|p| p.1).sum::<f64>() / n",
        "sin_yaw.atan2(cos_yaw)",
        "(sensed.x, sensed.y, sensed.yaw)",
    ):
        assert text in source
    fragment = source[
        source.index("            let preview_end = Point2D::new(") : source.index(
            "            let recovery_margin = "
        )
    ]
    assert "recovery_x + 0.6 * recovery_yaw.cos()" in fragment
    assert "recovery_y + 0.6 * recovery_yaw.sin()" in fragment
    assert ".fold(f64::INFINITY, f64::min)" in fragment
    return dict(
        common_margin_sha256=hashlib.sha256("".join(a.split()).encode()).hexdigest(),
        recovery_fragment_sha256=hashlib.sha256(fragment.encode()).hexdigest(),
    )


def state(pair):
    return "clear" if pair[0] > 0 else "blocked" if pair[1] <= 0 else "unknown"


def blockers(d):
    if d["probe_origin"] == "initial_unsafe":
        ids, walls = d["definite_obstacle_ids"], d["definite_wall_ids"]
    else:
        assert d["probe_origin"] == "first_chord_by_exclusion"
        assert d["direct_first_test_state"] == "unsafe"
        ids, walls = d["first_definite_obstacle_ids"], d["first_definite_wall_ids"]
    assert ids and not walls
    result = ids.split(";")
    assert len(set(result)) == len(result)
    return result


def join(old, admissions):
    lookup = {(a["event_id"], a["time_s"]): a for a in admissions}
    assert len(lookup) == len(admissions) == len(old)
    assert lookup.keys() == {(d["event_id"], d["time_s"]) for d in old}
    for d in old:
        a = lookup[d["event_id"], d["time_s"]]
        for k in d.keys() & a.keys():
            if k in (
                "target_speed",
                "nominal_target_speed",
                "measured_speed",
                "speed_margin",
                "time_s",
                "offset_from_return_ms",
            ):
                assert float(d[k]) == float(a[k]), (k, d[k], a[k])
            else:
                assert d[k] == a[k], (k, d[k], a[k])
    return lookup


def geometry(d, a, obstacles):
    ids = blockers(d)
    assert set(ids) <= {o[0] for o in obstacles}
    values = [float(a[k]) for k in INPUTS[:-1]]
    assert all(math.isfinite(v) and abs(v) <= 100 for v in values)
    x, y, yaw, logged = values
    assert int(a["recovery_sample_count"]) in range(26)
    margin = float(d["speed_margin"])
    assert 0 < margin < 10
    sx, sy = (x - H, x + H), (y - H, y + H)
    ex = prior.add(sx, prior.mul((0.6, 0.6), prior.trig((yaw - H, yaw + H), True)))
    ey = prior.add(sy, prior.mul((0.6, 0.6), prior.trig((yaw - H, yaw + H))))
    center = x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw)
    radius = (
        max(
            math.sqrt(2) * H,
            math.hypot(
                max(abs(v - center[0]) for v in ex), max(abs(v - center[1]) for v in ey)
            ),
        )
        + P
    )
    out = {k: a[k] for k in INPUTS}
    out.update(original_blocker_ids=";".join(ids), recovery_segment_radius=radius)
    delta = []
    for axis, rr, ee, cc in (("x", sx, ex, center[0]), ("y", sy, ey, center[1])):
        out.update(
            {
                f"recovery_endpoint_{axis}_{s}": v
                for s, v in zip(("center", "lo", "hi"), (cc, *ee))
            }
        )
        dd = prior.sub(
            rr, (float(d[f"sensed_{axis}_lo"]), float(d[f"sensed_{axis}_hi"]))
        )
        out.update(
            {f"recovery_minus_raw_{axis}_{s}": v for s, v in zip(("lo", "hi"), dd)}
        )
        delta.append(dd)
    out["pose_displacement_lo"] = max(
        0.0,
        math.hypot(*(0 if lo <= 0 <= hi else min(abs(lo), abs(hi)) for lo, hi in delta))
        - P,
    )
    out["pose_displacement_hi"] = (
        math.hypot(*(max(abs(lo), abs(hi)) for lo, hi in delta)) + P
    )
    point_ranges, segment_ranges, all_ranges = [], [], []
    yes, maybe = [], []
    for oid, ox, oy in obstacles:
        distance = prior.segment_distance((ox, oy), (x, y), center)
        interval = max(0.0, distance - radius - S), distance + radius + S
        all_ranges.append(interval)
        gap = interval[0] - margin - H, interval[1] - margin + H
        if state(gap) == "blocked":
            yes.append(oid)
        if state(gap) == "unknown":
            maybe.append(oid)
        if oid in ids:
            segment_ranges.append(gap)
            pd = math.hypot(ox - x, oy - y)
            er = math.sqrt(2) * H + S
            point_ranges.append((pd - er - margin - H, pd + er - margin + H))
    clearance = min(i[0] for i in all_ranges), min(i[1] for i in all_ranges)
    assert clearance[0] <= logged + H and clearance[1] >= logged - H, (
        clearance,
        logged,
    )
    out.update(allmap_clearance_lo=clearance[0], allmap_clearance_hi=clearance[1])
    for name, pairs in (
        ("blocker_start", point_ranges),
        ("blocker_segment", segment_ranges),
        ("allmap", [(clearance[0] - margin - H, clearance[1] - margin + H)]),
    ):
        interval = min(i[0] for i in pairs), min(i[1] for i in pairs)
        out.update(
            {
                f"{name}_gap_lo": interval[0],
                f"{name}_gap_hi": interval[1],
                f"{name}_state": state(interval),
            }
        )
    assert not (a["gate_clearance"] == "pass" and out["allmap_state"] == "blocked")
    assert not (a["gate_clearance"] == "fail" and out["allmap_state"] == "clear")
    out.update(
        recovery_definite_obstacle_ids=";".join(yes),
        recovery_ambiguous_obstacle_ids=";".join(maybe),
    )
    assert set(out) == set(FIELDS)
    return out


def build(d, a=None, before=None, current=None, obstacles=()):
    audited = float(d["speed_free_distance"]) == 0
    out = dict(d, recovery_geometry_audited=audited, **dict.fromkeys(FIELDS))
    if not audited:
        assert a is None and before is None and current is None
        return out
    for k in (*INPUTS, "speed_margin"):
        assert float(a[k]) == float(current[k]) == float(d[k] if k in d else a[k])
    for k in META:
        if k in current:
            assert d[k] == current[k]
    p = prior.previous.pose(before, current)
    for k, v in p.items():
        assert (
            d[k] == v
            if isinstance(v, str)
            else math.isclose(float(d[k]), v, abs_tol=2e-12)
        )
    out.update(geometry(d, a, obstacles))
    return out


def anchor(e, dd, ne, nr):
    out = dict(e, **prior.weights(e, ne, nr), **dict.fromkeys((*REFS, *HAS)))
    if not flag(e["returned"] or "False"):
        assert not dd
        return out
    assert all(all(d[k] == e[k] for k in META) for d in dd)
    counts = Counter()
    for d in dd:
        if not d["recovery_geometry_audited"]:
            continue
        kinds = [
            "audited",
            "blockers_" + d["blocker_segment_state"],
            "allmap_" + d["allmap_state"],
        ]
        if d["blocker_segment_state"] == "clear":
            if d["prior_certainty"] == "unknown":
                kinds.append("prior_unknown_clear")
            if d["timer_action"] == "reset" and d["refined_failed_gates"] == "nominal":
                kinds.append("nominal_only_reset_clear")
        for k in kinds:
            counts[f"{d['phase']}_{d['branch']}_recovery_{k}_refs"] += 1
    out.update({k: counts[k] for k in REFS})
    for s in STATES:
        out["has_available_recovery_blockers_" + s] = any(
            d["phase"] == "available"
            and d["recovery_geometry_audited"]
            and d["blocker_segment_state"] == s
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
                for f in (
                    *REFS,
                    *(h + s for h in HAS for s in ("_events", "_case_mass")),
                ):
                    assert r[f] == sum(
                        lookup[key[:i] + (p,) + key[i + 1 :]][f] for p in parts
                    )
                    checks += 1
        for p, b in product(PHASES, BRANCHES):
            prefix = f"{p}_{b}_recovery_"
            for name in ("blockers", "allmap"):
                assert r[prefix + "audited_refs"] == sum(
                    r[prefix + name + "_" + s + "_refs"] for s in STATES
                )
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--raw-roots", type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Use a new output directory without -O")
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
            "fusion-return-first-chord/" + n: h
            for n, h in source["output_sha256"].items()
        },
        "fusion-return-first-chord/sources.json": BASE_SHA,
        "fusion-return-first-chord/independent-verification.json": PROOF_SHA,
        "fusion-return-first-chord/protocol.md": source["analysis"]["protocol_sha256"],
        "fusion-return-first-chord/freeze-pins.json": source["analysis"][
            "freeze_sha256"
        ],
    }
    assert pins[ADMISSION] == freeze["admission_sha256"]
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_first_chord.py": source["analysis"]["script_sha256"],
    }
    vhelpers = {
        **source["analysis"]["verifier_helpers_sha256"],
        "verify_first_chord.py": freeze["verifier_helper_sha256"],
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
    prior.previous.provenance(source["raw"], roots, proof["native"]["rustc"])
    obstacles = prior.previous.maps(
        RESULTS / "fusion-return-probe-origin/obstacle-map.csv"
    )
    cases = read(RESULTS / "census-rp-selection/case-selection.csv")
    old = read(BASE / "first-chord-decisions.csv")
    admissions = join(old, read(RESULTS / ADMISSION))
    selected = [d for d in old if float(d["speed_free_distance"]) == 0]
    assert len(selected) == 1096
    points, audit = prior.previous.collect(cases, selected, source["raw"], roots)
    decisions = [
        (
            build(
                d,
                admissions[d["event_id"], d["time_s"]],
                points[d["case_id"], step(d["time_s"]) - 1],
                points[d["case_id"], step(d["time_s"])],
                obstacles[d["scenario"]],
            )
            if float(d["speed_free_distance"]) == 0
            else build(d)
        )
        for d in old
    ]
    events = read(BASE / "anchor-first-chord.csv")
    ne = Counter(e["case_id"] for e in events)
    nr = Counter(e["case_id"] for e in events if flag(e["received"]))
    db = defaultdict(list)
    for d in decisions:
        db[d["event_id"]].append(d)
    anchors = [
        anchor(e, db[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]]) for e in events
    ]
    summary = summaries(read(BASE / "first-chord-summary.csv"), anchors)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(flag(e["returned"] or "False") for e in events),
        decision_references=len(decisions),
        audited_zero_probes=len(selected),
        available_audited=sum(d["phase"] == "available" for d in selected),
        boundary_audited=sum(d["phase"] == "next_loss" for d in selected),
        summary_rows=len(summary),
        new_additive_closure_comparisons=closure(summary),
    )
    assert (
        len(cases),
        len(ne),
        len(anchors),
        len(decisions),
        len(summary),
        audit["available_audited"],
        audit["boundary_audited"],
    ) == (972, 570, 3997, 3751, 336, 1067, 29)
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
        ("recovery-geometry-decisions", decisions),
        ("anchor-recovery-geometry", anchors),
        ("recovery-geometry-summary", summary),
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
            segment_allowance=S,
            phases=PHASES,
            branches=BRANCHES,
            retrospective=True,
            inherited="all prior columns and source failure stages; new recovery geometry conditional on stated numerical assumptions",
        ),
        inherited_first_chord_proof_sha256=PROOF_SHA,
        inherited_validation=source["inherited_validation"],
        audit=audit,
        output_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
