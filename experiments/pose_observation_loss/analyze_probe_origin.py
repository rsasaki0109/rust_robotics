#!/usr/bin/env python3
"""Bound the original speed probe's initial test before interpreting zero returns."""

import argparse
from collections import Counter, defaultdict
import csv
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path
import shutil

import analyze_command_sign as previous

ROOT, RESULTS, REPO = previous.ROOT, previous.RESULTS, previous.REPO
BASE = RESULTS / "fusion-return-command-sign"
BASE_SHA = "2090042807aaef058d8c43eee5b112ab0ef658b934b1bccad29e81a5af4bc62b"
PROOF_SHA = "3d82c7af8398d4a954043d8179ee244083cb5141b7752cce5e4b165e194dd3b8"
PROTOCOL = RESULTS / "fusion-return-probe-origin/protocol.md"
MAP = RESULTS / "planning-clearance/fixtures/fine-padding/obstacles.csv"
MAP_SHA = "59ff820ccbf076cfd90906da055c4f0b95c58207652d7d28610b294f6c51b3e4"
TYPES = "crates/rust_robotics_core/src/types.rs"
TYPES_SHA = "19c5cf3473851102b84caa21eaa2149c9bffd7d113a0a035622bf53164bde80f"
META, KEY, GROUP = previous.META, previous.KEY, previous.GROUP
HELPERS, CATEGORIES = previous.HELPERS, previous.CATEGORIES
PHASES, BRANCHES = previous.PHASES, previous.BRANCHES
ORIGINS = (
    "positive_probe",
    "initial_unsafe",
    "first_chord_by_exclusion",
    "ambiguous_zero",
)
ZERO_ORIGINS = ORIGINS[1:]
MASKS = ("boundary_only", "obstacle_only", "both")
REFS = tuple(
    f"{p}_{b}_{name}_refs"
    for p, b in product(PHASES, BRANCHES)
    for name in (
        *ORIGINS,
        *("initial_" + m for m in MASKS),
        *("prior_unknown_" + o for o in ZERO_ORIGINS),
        *("nominal_only_reset_" + o for o in ZERO_ORIGINS),
    )
)
HAS = tuple("has_available_" + o for o in ZERO_ORIGINS)
H, P = 5e-10, 1e-12
WALLS = ("x_min", "y_min", "x_max", "y_max")
INPUTS = ("x", "y", "noise_x", "noise_y", "bias_x", "bias_y", "speed_margin")
sha, read, write = previous.sha, previous.read, previous.write
flag, step = previous.previous.flag, previous.previous.step


def source_contract(source, types):
    prefix = previous.extract(source, "fn speed_free_distance(").split(
        "    let curvature", 1
    )[0]
    assert "let start = Point2D::new(sensed.x, sensed.y);" in prefix
    assert ".filter(|p| p.distance(&start) <= 2.0 + margin)" in prefix
    assert (
        ".any(|&d| d <= margin)" in prefix and "p.distance(&start) <= margin" in prefix
    )
    assert prefix.count("return 0.0;") == 1
    assert (
        "return (step - 1) as f64 * 0.05;" in source and "for step in 1..=40" in source
    )
    assert (
        "state.x + noise_x + bias_x" in source
        and "state.y + noise_y + bias_y" in source
    )
    assert (
        source.index("let sensed = if position_sigma == 0.0")
        < source.index("state.update(acceleration, steering, DT);")
        < source.index('writeln!(tracks, "{scenario},{planner},{time:.6}')
    )
    assert "false,false,{position_sigma},{yaw_sigma},false" in source
    assert "((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()" in types
    return dict(
        initial_test_sha256=__import__("hashlib").sha256(prefix.encode()).hexdigest(),
        pose_order="preceding post-update truth plus current noise plus current bias; two additions",
    )


def maps(path):
    assert sha(path) == MAP_SHA
    output = defaultdict(list)
    for i, row in enumerate(read(path), 1):
        x, y = float(row["x"]), float(row["y"])
        assert math.isfinite(x) and math.isfinite(y) and 0 <= x <= 50 and 0 <= y <= 50
        output[row["scenario"]].append((str(i), x, y))
    assert set(output) == {"open_50x50", "maze_50x50", "dense_50x50"}
    return output


def provenance(raw, roots, compiler):
    for cohort, spec in raw.items():
        path = Path(roots.get(cohort, spec["path"])) / "provenance.json"
        assert sha(path) == spec["provenance_sha256"]
        p = json.loads(path.read_text())
        assert p["rustc"] == compiler
        assert p["inputs_sha256"]["obstacles.csv"] == MAP_SHA
        assert set(p["inputs_by_variant_sha256"]) == {"epoch", "curved"}
        assert all(
            v["obstacles.csv"] == MAP_SHA
            for v in p["inputs_by_variant_sha256"].values()
        )


def noisy_branch(row):
    sigma, yaw = float(row["effective_position_sigma"]), float(
        row["effective_yaw_sigma_rad"]
    )
    assert math.isfinite(sigma) and math.isfinite(yaw) and sigma >= 0 and yaw >= 0
    assert sigma > 0 or yaw > 0, "noisy sensed-pose branch is not established"


def pose(before, current):
    values = {"prior_truth_" + k: before[k] for k in ("x", "y")}
    values.update({k: current[k] for k in INPUTS if k not in ("x", "y")})
    for v in values.values():
        assert math.isfinite(float(v)) and abs(float(v)) <= 100
    out = dict(
        prior_truth_time_s=float(before["time_s"]),
        observation_time_s=float(current["time_s"]),
        **values,
    )
    assert step(out["observation_time_s"]) == step(out["prior_truth_time_s"]) + 1
    for axis in ("x", "y"):
        center = (float(before[axis]) + float(current["noise_" + axis])) + float(
            current["bias_" + axis]
        )
        assert abs(center) <= 100
        out["sensed_" + axis + "_center"] = center
        out["sensed_" + axis + "_lo"] = center - 3 * H - P
        out["sensed_" + axis + "_hi"] = center + 3 * H + P
    margin = float(current["speed_margin"])
    assert 0 < margin < 10
    return out


def classify(pair):
    return "safe" if pair[0] > 0 else "unsafe" if pair[1] <= 0 else "unknown"


def geometry(p, obstacles):
    xl, xh = p["sensed_x_lo"], p["sensed_x_hi"]
    yl, yh = p["sensed_y_lo"], p["sensed_y_hi"]
    ml, mh = float(p["speed_margin"]) - H, float(p["speed_margin"]) + H
    assert 0 < ml <= mh < 10
    walls = (
        (xl - mh - P, xh - ml + P),
        (yl - mh - P, yh - ml + P),
        (50 - xh - mh - P, 50 - xl - ml + P),
        (50 - yh - mh - P, 50 - yl - ml + P),
    )
    definite_walls = [w for w, pair in zip(WALLS, walls) if classify(pair) == "unsafe"]
    ambiguous_walls = [
        w for w, pair in zip(WALLS, walls) if classify(pair) == "unknown"
    ]
    lows, highs, definite, ambiguous = [], [], [], []
    for oid, x, y in obstacles:
        near_x, near_y = max(xl - x, 0.0, x - xh), max(yl - y, 0.0, y - yh)
        far_x, far_y = max(abs(xl - x), abs(xh - x)), max(abs(yl - y), abs(yh - y))
        low, high = (
            math.hypot(near_x, near_y) - mh - P,
            math.hypot(far_x, far_y) - ml + P,
        )
        lows.append(low)
        highs.append(high)
        state = classify((low, high))
        if state == "unsafe":
            definite.append(oid)
        elif state == "unknown":
            ambiguous.append(oid)
    boundary = min(x[0] for x in walls), min(x[1] for x in walls)
    obstacle = (min(lows), min(highs)) if lows else None
    total = (
        (min(boundary[0], obstacle[0]), min(boundary[1], obstacle[1]))
        if obstacle
        else boundary
    )
    state = classify(total)
    mask = (
        "both"
        if definite_walls and definite
        else "boundary_only" if definite_walls else "obstacle_only" if definite else ""
    )
    assert (state == "unsafe") == bool(mask)
    return dict(
        initial_test_state=state,
        initial_failure_mask=mask,
        boundary_gap_lo=boundary[0],
        boundary_gap_hi=boundary[1],
        obstacle_gap_lo=obstacle[0] if obstacle else None,
        obstacle_gap_hi=obstacle[1] if obstacle else None,
        definite_wall_ids=";".join(definite_walls),
        ambiguous_wall_ids=";".join(ambiguous_walls),
        definite_obstacle_ids=";".join(definite),
        ambiguous_obstacle_ids=";".join(ambiguous),
        obstacle_records_checked=len(obstacles),
    )


def origin(free, initial):
    if float(free) > 0:
        assert (
            initial != "unsafe"
        ), "positive probe contradicts a certainly unsafe start"
        return "positive_probe"
    assert float(free) == 0
    return {
        "unsafe": "initial_unsafe",
        "safe": "first_chord_by_exclusion",
        "unknown": "ambiguous_zero",
    }[initial]


def collect(cases, decisions, raw, roots):
    files, wanted = defaultdict(dict), set()
    for c in cases:
        files[c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"][
            c["scenario"], c["planner"]
        ] = c
    for d in decisions:
        n = step(d["time_s"])
        wanted.update(((d["case_id"], n - 1), (d["case_id"], n)))
    assert set(files) == {(p, n) for p, s in raw.items() for n in s["raw_sha256"]}
    points, audit = {}, Counter()
    for i, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert sha(path) == raw[cohort]["raw_sha256"][name]
        audit["raw_bytes"] += path.stat().st_size
        last = {}
        with path.open(newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
            assert len(header) == len(set(header))
            pos = {
                k: header.index(k)
                for k in (
                    "scenario",
                    "planner",
                    "time_s",
                    "effective_position_sigma",
                    "effective_yaw_sigma_rad",
                )
            }
            for cells in reader:
                assert len(cells) == len(header)
                identity = cells[pos["scenario"]], cells[pos["planner"]]
                assert identity in members
                c = members[identity]
                n = step(cells[pos["time_s"]])
                assert n == last.get(identity, -1) + 1
                last[identity] = n
                if not flag(c["selected_timeout"]):
                    continue
                audit["timeout_grid_rows"] += 1
                if n == 0:
                    noisy_branch({k: cells[j] for k, j in pos.items()})
                    audit["noisy_branch_histories"] += 1
                if (c["case_id"], n) in wanted:
                    points[c["case_id"], n] = dict(zip(header, cells))
        assert set(last) == set(members)
        for identity, c in members.items():
            assert last[identity] == (
                9000 if flag(c["selected_timeout"]) else step(c["time_s"])
            )
        audit["raw_files"] += 1
        if i % 18 == 0:
            print(f"Original probe geometry inputs read: {i}/108 files", flush=True)
    assert points.keys() == wanted
    audit["extracted_raw_points"] = len(points)
    return points, dict(audit)


def build(old, before, current, obstacles):
    for k in old.keys() & current.keys():
        if k in META:
            assert old[k] == current[k]
        elif current[k] in ("true", "false"):
            assert flag(old[k]) == flag(current[k], raw=True), k
        else:
            assert float(old[k]) == float(current[k]), k
    p = pose(before, current)
    g = geometry(p, obstacles)
    return dict(
        **old,
        **p,
        **g,
        probe_origin=origin(old["speed_free_distance"], g["initial_test_state"]),
    )


def extend_anchor(base, decisions):
    result = dict(base)
    result.update(dict.fromkeys((*REFS, *HAS)))
    if not base["returned"]:
        assert not decisions
        return result
    counts = Counter()
    for d in decisions:
        assert all(d[k] == base[k] for k in META)
        prefix = d["phase"] + "_" + d["branch"] + "_"
        which = d["probe_origin"]
        counts[prefix + which + "_refs"] += 1
        if which == "initial_unsafe":
            counts[prefix + "initial_" + d["initial_failure_mask"] + "_refs"] += 1
        if d["prior_certainty"] == "unknown":
            assert which in ZERO_ORIGINS
            counts[prefix + "prior_unknown_" + which + "_refs"] += 1
        if d["timer_action"] == "reset" and d["refined_failed_gates"] == "nominal":
            assert which in ZERO_ORIGINS
            counts[prefix + "nominal_only_reset_" + which + "_refs"] += 1
    result.update({k: counts[k] for k in REFS})
    for o in ZERO_ORIGINS:
        result["has_available_" + o] = any(
            d["phase"] == "available" and d["probe_origin"] == o for d in decisions
        )
    return result


def summaries(cases, anchors):
    rows = previous.summaries(cases, anchors)
    for r in rows:
        aa = [
            a
            for a in anchors
            if (a["partition"], a["noise"], a["variant"])
            == tuple(r[k] for k in GROUP[:3])
            and (r["mode"] == "all_modes" or a["mode"] == r["mode"])
            and (
                r["category_selector"] == "all"
                or a["anchor_category"] == r["category_selector"]
            )
        ]
        returned = [a for a in aa if a["returned"]]
        mass = lambda rr: sum((a["census_case_weight"] for a in rr), Fraction())
        r.update({k: sum(a[k] or 0 for a in aa) for k in REFS})
        for name in HAS:
            selected = [a for a in aa if a[name]]
            r[name + "_events"], r[name + "_case_mass"] = len(selected), mass(selected)
            r[name + "_fraction"] = (
                Fraction(len(selected), len(returned)) if returned else None
            )
            r[name + "_case_mass_fraction"] = (
                mass(selected) / mass(returned) if returned else None
            )
    return rows


def closure(rows):
    previous.closures(rows)
    lookup = {tuple(r[k] for k in GROUP): r for r in rows}
    checks = 0
    for key, r in lookup.items():
        for i, whole, pieces in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", CATEGORIES),
        ):
            if key[i] == whole:
                children = [lookup[key[:i] + (p,) + key[i + 1 :]] for p in pieces]
                for name in (
                    *REFS,
                    *(h + suffix for h in HAS for suffix in ("_events", "_case_mass")),
                ):
                    assert r[name] == sum(c[name] for c in children), (key, name)
                    checks += 1
        for p, b in product(PHASES, BRANCHES):
            prefix = p + "_" + b + "_"
            assert (
                sum(r[prefix + o + "_refs"] for o in ORIGINS)
                == r[prefix + "all_references"]
            )
            assert (
                sum(r[prefix + "initial_" + m + "_refs"] for m in MASKS)
                == r[prefix + "initial_unsafe_refs"]
            )
    return checks


def check_inherited(rows, old, keys):
    expected = {tuple(r[k] for k in keys): r for r in old}
    assert len(expected) == len(rows)
    for row in rows:
        before = expected[tuple(str(row[k]) for k in keys)]
        for k, value in before.items():
            got = row[k]
            if value == "":
                assert got is None
            elif isinstance(got, (str, bool, int)):
                assert str(got) == value, (k, got, value)
            else:
                assert math.isclose(float(got), float(value), abs_tol=2e-12)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--raw-roots", type=Path)
    parser.add_argument("--obstacles", type=Path, default=MAP)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Use a new output directory without -O")
    assert (
        sha(BASE / "sources.json") == BASE_SHA
        and sha(BASE / "independent-verification.json") == PROOF_SHA
    )
    prior = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    pins = {
        **prior["input_sha256"],
        "fusion-return-command-sign/sources.json": BASE_SHA,
        "fusion-return-command-sign/independent-verification.json": PROOF_SHA,
        "fusion-return-command-sign/protocol.md": prior["analysis"]["protocol_sha256"],
        **{
            "fusion-return-command-sign/" + k: h
            for k, h in prior["output_sha256"].items()
        },
    }
    helpers = {
        **prior["analysis"]["helpers_sha256"],
        "analyze_command_sign.py": prior["analysis"]["script_sha256"],
    }
    code = {**prior["analysis"]["code_sha256"], TYPES: TYPES_SHA}
    for name, h in pins.items():
        assert sha(RESULTS / name) == h, name
    for name, h in {**helpers, **HELPERS}.items():
        assert sha(ROOT / name) == h, name
    for name, h in code.items():
        assert sha(REPO / name) == h, name
    contract = source_contract(
        (
            REPO / "crates/rust_robotics_control/examples/pose_observation_loss.rs"
        ).read_text(),
        (REPO / TYPES).read_text(),
    )
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(prior["raw"])
    provenance(prior["raw"], roots, proof["native"]["rustc"])
    obstacles = maps(args.obstacles)
    cases = read(RESULTS / "census-rp-selection/case-selection.csv")
    events = read(BASE / "anchor-command-sign.csv")
    old = read(BASE / "command-decisions.csv")
    previous.previous.indexed(cases, KEY)
    previous.previous.indexed(old, ("event_id", "time_s"))
    points, audit = collect(cases, old, prior["raw"], roots)
    decisions = [
        build(
            d,
            points[d["case_id"], step(d["time_s"]) - 1],
            points[d["case_id"], step(d["time_s"])],
            obstacles[d["scenario"]],
        )
        for d in old
    ]
    by_event, old_by_event = defaultdict(list), defaultdict(list)
    for d in decisions:
        by_event[d["event_id"]].append(d)
    for d in old:
        old_by_event[d["event_id"]].append(
            {k: flag(v) if v in ("True", "False") else v for k, v in d.items()}
        )
    ne = Counter(e["case_id"] for e in events)
    nr = Counter(e["case_id"] for e in events if flag(e["received"]))
    anchors = [
        extend_anchor(
            previous.anchor(
                e, old_by_event[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]]
            ),
            by_event[e["event_id"]],
        )
        for e in events
    ]
    summary = summaries(cases, anchors)
    check_inherited(anchors, events, ("event_id",))
    check_inherited(summary, read(BASE / "command-summary.csv"), GROUP)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(bool(a["returned"]) for a in anchors),
        decision_references=len(decisions),
        available_references=sum(d["phase"] == "available" for d in decisions),
        next_loss_references=sum(d["phase"] == "next_loss" for d in decisions),
        summary_rows=len(summary),
        new_additive_closure_comparisons=closure(summary),
        obstacle_records=sum(map(len, obstacles.values())),
        obstacle_comparisons=sum(d["obstacle_records_checked"] for d in decisions),
    )
    assert (
        audit["raw_files"],
        audit["timeout_grid_rows"],
        audit["noisy_branch_histories"],
    ) == (108, 5130570, 570)
    assert (
        len(cases),
        audit["planned_families"],
        len(ne),
        len(anchors),
        audit["returned_anchors"],
        len(decisions),
        audit["available_references"],
        len(summary),
    ) == (972, 324, 570, 3997, 137, 3751, 3662, 336)
    assert (
        sum(a["census_case_weight"] for a in anchors)
        == sum(a["received_case_weight"] for a in anchors)
        == 570
    )
    assert (
        sum(a["rp_case_mass"] for a in anchors) == 157
        and sum(a["rp_records"] for a in anchors) == 10995
    )
    args.output.mkdir(parents=True)
    for name, rows in (
        ("probe-decisions", decisions),
        ("anchor-probe-origin", anchors),
        ("probe-summary", summary),
    ):
        write(args.output / (name + ".csv"), rows)
    shutil.copyfile(args.obstacles, args.output / "obstacle-map.csv")
    source = dict(
        input_sha256=pins,
        raw=prior["raw"],
        map_sha256=MAP_SHA,
        analysis=dict(
            script_sha256=sha(Path(__file__)),
            protocol_sha256=sha(PROTOCOL),
            helpers_sha256=helpers,
            verifier_helpers_sha256=HELPERS,
            code_sha256=code,
            source_contract=contract,
        ),
        schema=dict(
            half_unit=H,
            numerical_guard=P,
            origins=ORIGINS,
            phases=PHASES,
            branches=BRANCHES,
            retrospective=True,
            first_chord="inferred only from source zero plus certainly safe initial test; no chord replay",
            inherited="preceding anchor/summary/admission/timer/command-domain fields and proof",
        ),
        inherited_command_proof_sha256=PROOF_SHA,
        inherited_validation=prior["inherited_validation"],
        audit=audit,
        output_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
