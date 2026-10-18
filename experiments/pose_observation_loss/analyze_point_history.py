#!/usr/bin/env python3
"""Audit fixed half-second point-gap histories with explicit missing slots."""

import argparse
from collections import Counter, defaultdict
from fractions import Fraction
from itertools import product
import csv
import json
import math
from pathlib import Path

import analyze_point_clearance as prior

ROOT, REPO, RESULTS = prior.ROOT, prior.REPO, prior.RESULTS
BASE = RESULTS / "fusion-return-point-clearance"
STUDY = RESULTS / "fusion-return-point-history"
BASE_MAIN = "9cc0426a527c3b200a0c7312e9d964a7b06932ae"
BASE_SHA = "6752d0e1aa13908e5b402e63f1079d1fac692c68fc70c66129a930e31ec09d58"
PROOF_SHA = "d6f0254d3e70a4bb371ac1cc17cb2cbf506e4f1b6af12751a0201f8df21ab901"
META, KEY, GROUP = prior.META, prior.KEY, prior.GROUP
PHASES, BRANCHES, CATEGORIES = prior.PHASES, prior.BRANCHES, prior.CATEGORIES
STATES = prior.STATES
LATER = (
    "clear_observed",
    "unresolved",
    "no_future",
    "no_clear_observed",
    "current_not_blocked",
)
STATUSES = ("observed", "before_source", "after_source", "at_or_after_next_loss")
KINDS = (
    "audited",
    *(f"{k}_current_{s}" for k in ("raw", "mean") for s in STATES),
    *(k + "_left_censored" for k in ("raw", "mean")),
    "raw_new_blocked",
    "raw_persisting_blocked",
    "raw_away_blocked",
    *(f"{k}_later_{s}" for k in ("raw", "mean") for s in LATER),
)
REFS = tuple(
    f"{p}_{b}_history_{k}_refs" for p, b in product(PHASES, BRANCHES) for k in KINDS
)
HAS = tuple(
    "has_available_history_" + k
    for k in ("raw_new_blocked", "raw_persisting_blocked", "raw_later_clear")
)
FIELDS = (
    "history_observed_slots",
    "history_missing_slots",
    "history_before_return_slots",
    *(
        f"history_{k}_{side}_{s}_slots"
        for k in ("raw", "mean")
        for side in ("pre", "post")
        for s in (*STATES, "missing")
    ),
    *(
        f"history_{k}_{f}"
        for k in ("raw", "mean")
        for f in (
            "current_state",
            "run_count",
            "run_start_offset",
            "run_span_s",
            "left_censored",
            "predecessor_state",
            "later_status",
            "first_later_clear_offset_steps",
            "first_later_clear_delay_s",
            "first_later_clear_time_s",
        )
    ),
    "history_raw_new_blocked",
    "history_raw_persisting_blocked",
    "history_raw_away_blocked",
)
RAW = prior.prior.RAW
H, P, R = prior.H, prior.P, prior.R
window = prior.prior.prior
sha, read, write, flag, step = (
    prior.sha,
    prior.read,
    prior.write,
    prior.flag,
    prior.step,
)
contract = prior.contract
SLOT_FIELDS = (
    "logged_speed_margin",
    "margin_lo",
    "margin_hi",
    "window_count",
    "last_clear_step",
    "last_clear_reasons",
    *("input_" + k for k in RAW),
    *(
        f"{k}_{a}_{b}"
        for k in ("raw", "mean")
        for a in ("x", "y")
        for b in ("center", "lo", "hi")
    ),
    *(
        f"{k}_{q}_{b}"
        for k in ("raw", "mean")
        for q in ("distance", "gap")
        for b in ("center", "lo", "hi")
    ),
    *(f"{k}_state" for k in ("raw", "mean")),
    *(f"{k}_{t}_ids" for k in ("raw", "mean") for t in ("minimum", "possible_minimum")),
)


def plan_slots(selected, events):
    slots = []
    for d in selected:
        e = events[d["event_id"]]
        n = step(d["time_s"])
        start = step(e["return_time_s"])
        loss = step(e["end_time_s"]) if flag(e["ended"]) else None
        assert start <= n <= 9000 and (loss is None or n < loss)
        for offset in range(-25, 26):
            source = n + offset
            status = (
                "before_source"
                if source < 1
                else (
                    "after_source"
                    if source > 9000
                    else (
                        "at_or_after_next_loss"
                        if offset > 0 and loss is not None and source >= loss
                        else "observed"
                    )
                )
            )
            phase = (
                ""
                if status != "observed"
                else (
                    "before_return"
                    if source < start
                    else "reference" if offset == 0 else "available_context"
                )
            )
            slots.append(
                dict(
                    **{k: d[k] for k in (*META, "event_id")},
                    reference_time_s=d["time_s"],
                    offset_steps=offset,
                    offset_s=offset / 50,
                    source_step=source,
                    source_time_s=source / 50,
                    slot_status=status,
                    context_phase=phase,
                    fixed_obstacle_ids=d["fixed_obstacle_ids"],
                    raw_away_reference=d["raw_distance_direction"] == "away",
                )
            )
    return slots


def instant(d, snap, obstacles):
    ids = prior.prior.prior.prior.blockers(d)
    omap = {i: (x, y) for i, x, y in obstacles}
    assert set(ids) <= omap.keys() and len(omap) == len(obstacles)
    assert ";".join(ids) == d["fixed_obstacle_ids"]
    margin = snap["raw"]["speed_margin"]
    m = float(margin)
    assert math.isfinite(m) and m >= 0
    out = dict(
        logged_speed_margin=margin,
        margin_lo=m - H,
        margin_hi=m + H,
        window_count=len(snap["members"]),
        last_clear_step=snap["fields"]["window_last_clear_step"],
        last_clear_reasons=snap["fields"]["window_last_clear_reasons"],
    )
    out.update({"input_" + k: snap["point"][k] for k in RAW})
    witnesses = [
        dict(
            obstacle_order=j,
            obstacle_id=i,
            obstacle_x=omap[i][0],
            obstacle_y=omap[i][1],
            logged_speed_margin=margin,
        )
        for j, i in enumerate(ids, 1)
    ]
    for kind in ("raw", "mean"):
        center = tuple(
            float(
                snap["point"]["sensed_" + a + "_center"]
                if kind == "raw"
                else snap["fields"]["mean_" + a + "_center"]
            )
            for a in ("x", "y")
        )
        radius = R + 2 * P if kind == "mean" and snap["members"] else R
        for a, x in zip(("x", "y"), center):
            out.update(
                {
                    f"{kind}_{a}_{b}": v
                    for b, v in zip(("center", "lo", "hi"), (x, x - radius, x + radius))
                }
            )
        distances = {i: prior.distance_box(center, radius, omap[i]) for i in ids}
        minimum = tuple(min(distances[i][j] for i in ids) for j in range(3))
        for target, distance in [
            (out, minimum),
            *((w, distances[w["obstacle_id"]]) for w in witnesses),
        ]:
            gap = prior.triple_sub(distance, (m, m - H, m + H))
            for q, values in (("distance", distance), ("gap", gap)):
                target.update(
                    {
                        f"{kind}_{q}_{b}": v
                        for b, v in zip(("center", "lo", "hi"), values)
                    }
                )
            target[kind + "_state"] = prior.state(gap[1:])
        out[kind + "_minimum_ids"] = ";".join(
            i for i in ids if distances[i][0] == minimum[0]
        )
        out[kind + "_possible_minimum_ids"] = ";".join(
            i for i in ids if distances[i][1] <= minimum[2]
        )
    assert set(out) == set(SLOT_FIELDS)
    return out, witnesses


def check_previous(d, slot):
    offset = int(slot["offset_steps"])
    assert offset in (-1, 0) and slot["slot_status"] == "observed"
    role = "previous" if offset == -1 else "current"
    assert slot["logged_speed_margin"] == d[f"point_{role}_margin"]
    assert slot["window_count"] == int(
        d["previous_count" if offset == -1 else "window_replayed_count"]
    )
    for k in RAW:
        assert slot["input_" + k] == d[role + "_input_" + k]
    for kind in ("raw", "mean"):
        for a, b in product(("x", "y"), ("center", "lo", "hi")):
            assert slot[f"{kind}_{a}_{b}"] == float(d[f"point_{kind}_{role}_{a}_{b}"])
        for q, b in product(("distance", "gap"), ("center", "lo", "hi")):
            assert slot[f"{kind}_{q}_{b}"] == float(d[f"{kind}_{role}_{q}_{b}"])
        assert slot[kind + "_state"] == d[f"{kind}_{role}_state"]
        for t in ("minimum", "possible_minimum"):
            assert slot[f"{kind}_{t}_ids"] == d[f"{kind}_{role}_{t}_ids"]


def history(d, slots):
    db = {int(s["offset_steps"]): s for s in slots}
    assert len(db) == len(slots) == 51 and set(db) == set(range(-25, 26))
    assert db[0]["slot_status"] == "observed"
    out = dict(
        history_observed_slots=sum(s["slot_status"] == "observed" for s in slots),
        history_missing_slots=sum(s["slot_status"] != "observed" for s in slots),
        history_before_return_slots=sum(
            s["context_phase"] == "before_return" for s in slots
        ),
    )
    for kind in ("raw", "mean"):
        for side, offsets in (("pre", range(-25, 0)), ("post", range(1, 26))):
            counts = Counter(
                (
                    db[i][kind + "_state"]
                    if db[i]["slot_status"] == "observed"
                    else "missing"
                )
                for i in offsets
            )
            out.update(
                {
                    f"history_{kind}_{side}_{s}_slots": counts[s]
                    for s in (*STATES, "missing")
                }
            )
        current = db[0][kind + "_state"]
        start = 0
        for offset in range(-1, -26, -1):
            if (
                db[offset]["slot_status"] != "observed"
                or db[offset][kind + "_state"] != current
            ):
                break
            start = offset
        censored = start == -25 or db[start - 1]["slot_status"] != "observed"
        predecessor = None if censored else db[start - 1][kind + "_state"]
        future = [i for i in range(1, 26) if db[i]["slot_status"] == "observed"]
        clears = [i for i in future if db[i][kind + "_state"] == "clear"]
        found = clears[0] if current == "blocked" and clears else None
        later = (
            "current_not_blocked"
            if current != "blocked"
            else (
                "clear_observed"
                if found is not None
                else (
                    "unresolved"
                    if any(db[i][kind + "_state"] == "unknown" for i in future)
                    else "no_future" if not future else "no_clear_observed"
                )
            )
        )
        out.update(
            {
                f"history_{kind}_current_state": current,
                f"history_{kind}_run_count": 1 - start,
                f"history_{kind}_run_start_offset": start,
                f"history_{kind}_run_span_s": -start / 50,
                f"history_{kind}_left_censored": censored,
                f"history_{kind}_predecessor_state": predecessor,
                f"history_{kind}_later_status": later,
                f"history_{kind}_first_later_clear_offset_steps": found,
                f"history_{kind}_first_later_clear_delay_s": (
                    found / 50 if found is not None else None
                ),
                f"history_{kind}_first_later_clear_time_s": (
                    db[found]["source_time_s"] if found is not None else None
                ),
            }
        )
    prev = db[-1]["raw_state"] if db[-1]["slot_status"] == "observed" else None
    blocked = out["history_raw_current_state"] == "blocked"
    out.update(
        history_raw_new_blocked=blocked and prev == "clear",
        history_raw_persisting_blocked=blocked and prev == "blocked",
        history_raw_away_blocked=blocked and d["raw_distance_direction"] == "away",
    )
    assert set(out) == set(FIELDS)
    return out


def collect(cases, planned, raw, roots):
    files = defaultdict(dict)
    wanted = {
        (s["case_id"], s["source_step"])
        for s in planned
        if s["slot_status"] == "observed"
    }
    selected_cases = {c for c, n in wanted}
    for c in cases:
        files[c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"][
            c["scenario"], c["planner"]
        ] = c
    assert set(files) == {(c, n) for c, s in raw.items() for n in s["raw_sha256"]}
    snapshots = {}
    audit = Counter()
    for i, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert sha(path) == raw[cohort]["raw_sha256"][name]
        audit["raw_bytes"] += path.stat().st_size
        last = {}
        before = {}
        windows = {}
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
                    prior.prior.probe.noisy_branch(
                        {k: cells[j] for k, j in pos.items()}
                    )
                    audit["noisy_branch_histories"] += 1
                if c["case_id"] not in selected_cases:
                    continue
                row = dict(zip(header, cells))
                if n == 0:
                    windows[identity] = window.Window(row)
                    audit["replayed_histories"] += 1
                else:
                    w = windows[identity]
                    point = w.update(n, before[identity], row)
                    audit["replayed_steps"] += 1
                    if (c["case_id"], n) in wanted:
                        fields, _ = window.reconstruct(w, point, row)
                        snapshots[c["case_id"], n] = dict(
                            point=point, members=list(w.members), fields=fields, raw=row
                        )
                before[identity] = row
        assert set(last) == set(members)
        for identity, c in members.items():
            assert last[identity] == (
                9000 if flag(c["selected_timeout"]) else step(c["time_s"])
            )
        audit["raw_files"] += 1
        if i % 18 == 0:
            print(f"History contexts read: {i}/108 files", flush=True)
    assert snapshots.keys() == wanted
    audit["unique_context_points"] = len(snapshots)
    return snapshots, dict(audit)


def anchor(e, dd, ne, nr):
    out = dict(e, **prior.prior.weights(e, ne, nr), **dict.fromkeys((*REFS, *HAS)))
    if not flag(e["returned"] or "False"):
        assert not dd
        return out
    assert all(all(d[k] == e[k] for k in META) for d in dd)
    counts = Counter()
    for d in dd:
        if not d["history_audited"]:
            continue
        kinds = [
            "audited",
            *(
                k + "_current_" + d[f"history_{k}_current_state"]
                for k in ("raw", "mean")
            ),
            *(k + "_later_" + d[f"history_{k}_later_status"] for k in ("raw", "mean")),
        ]
        for k in ("raw", "mean"):
            if d[f"history_{k}_left_censored"]:
                kinds.append(k + "_left_censored")
        kinds.extend(
            k
            for k in ("raw_new_blocked", "raw_persisting_blocked", "raw_away_blocked")
            if d["history_" + k]
        )
        for k in kinds:
            counts[f"{d['phase']}_{d['branch']}_history_{k}_refs"] += 1
    out.update({k: counts[k] for k in REFS})
    available = [d for d in dd if d["phase"] == "available" and d["history_audited"]]
    for k in ("raw_new_blocked", "raw_persisting_blocked"):
        out["has_available_history_" + k] = any(d["history_" + k] for d in available)
    out["has_available_history_raw_later_clear"] = any(
        d["history_raw_later_status"] == "clear_observed" for d in available
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
            pre = f"{p}_{b}_history_"
            for group in (
                *(tuple(k + "_current_" + t for t in STATES) for k in ("raw", "mean")),
                *(tuple(k + "_later_" + t for t in LATER) for k in ("raw", "mean")),
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
            "fusion-return-point-clearance/" + n: h
            for n, h in source["output_sha256"].items()
        },
        "fusion-return-point-clearance/sources.json": BASE_SHA,
        "fusion-return-point-clearance/independent-verification.json": PROOF_SHA,
        "fusion-return-point-clearance/protocol.md": source["analysis"][
            "protocol_sha256"
        ],
        "fusion-return-point-clearance/freeze-pins.json": source["analysis"][
            "freeze_sha256"
        ],
    }
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_point_clearance.py": source["analysis"]["script_sha256"],
    }
    vhelpers = {
        **source["analysis"]["verifier_helpers_sha256"],
        "verify_point_clearance.py": freeze["verifier_helper_sha256"],
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
    prior.prior.probe.provenance(source["raw"], roots, proof["native"]["rustc"])
    cases = read(RESULTS / "census-rp-selection/case-selection.csv")
    old = read(BASE / "point-decisions.csv")
    selected = [d for d in old if d["prior_certainty"] == "unknown"]
    assert len(selected) == 25 and all(
        d["phase"] == "available" and d["branch"] == "joint" for d in selected
    )
    events = read(BASE / "anchor-point.csv")
    event_map = {e["event_id"]: e for e in events}
    assert len(event_map) == len(events)
    planned = plan_slots(selected, event_map)
    snapshots, audit = collect(cases, planned, source["raw"], roots)
    obstacles = prior.prior.probe.maps(
        RESULTS / "fusion-return-probe-origin/obstacle-map.csv"
    )
    selected_map = {(d["event_id"], d["time_s"]): d for d in selected}
    slots = []
    witnesses = []
    by_reference = defaultdict(list)
    for s in planned:
        d = selected_map[s["event_id"], s["reference_time_s"]]
        row = dict(s, **dict.fromkeys(SLOT_FIELDS))
        if s["slot_status"] == "observed":
            snap = snapshots[s["case_id"], s["source_step"]]
            fields, ww = instant(d, snap, obstacles[d["scenario"]])
            row.update(fields)
            if s["offset_steps"] in (-1, 0):
                check_previous(d, row)
            witnesses.extend(dict(s, **w) for w in ww)
        slots.append(row)
        by_reference[s["event_id"], s["reference_time_s"]].append(row)
    decisions = []
    for d in old:
        audited = (d["event_id"], d["time_s"]) in selected_map
        out = dict(d, history_audited=audited, **dict.fromkeys(FIELDS))
        if audited:
            out.update(history(d, by_reference[d["event_id"], d["time_s"]]))
        decisions.append(out)
    ne = Counter(e["case_id"] for e in events)
    nr = Counter(e["case_id"] for e in events if flag(e["received"]))
    db = defaultdict(list)
    for d in decisions:
        db[d["event_id"]].append(d)
    anchors = [
        anchor(e, db[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]]) for e in events
    ]
    summary = summaries(read(BASE / "point-summary.csv"), anchors)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(flag(e["returned"] or "False") for e in events),
        decision_references=len(decisions),
        selected_references=len(selected),
        inherited_zero_probes=1096,
        planned_slots=len(slots),
        context_queries=sum(s["slot_status"] == "observed" for s in slots),
        missing_slots=sum(s["slot_status"] != "observed" for s in slots),
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
    ) == (972, 570, 3997, 3751, 336, 4, 36000)
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
        ("history-decisions", decisions),
        ("history-witnesses", witnesses),
        ("history-slots", slots),
        ("anchor-history", anchors),
        ("history-summary", summary),
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
            offset_steps=list(range(-25, 26)),
            slot_statuses=STATUSES,
            states=STATES,
            later_statuses=LATER,
            phases=PHASES,
            branches=BRANCHES,
            retrospective=True,
            inherited="all prior columns/outcomes/weights; original 25 only; historical context and future strictly before next loss; point gaps do not replace controller predicates",
        ),
        inherited_point_clearance_proof_sha256=PROOF_SHA,
        inherited_validation=source["inherited_validation"],
        audit=audit,
        output_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
