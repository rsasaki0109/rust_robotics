#!/usr/bin/env python3
"""Audit adjacent membership updates and raw/mean pose steps at zero probes."""

import argparse
from collections import Counter, defaultdict
import csv
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path

import analyze_recovery_window as prior

ROOT, REPO, RESULTS = prior.ROOT, prior.REPO, prior.RESULTS
BASE = RESULTS / "fusion-return-recovery-window"
STUDY = RESULTS / "fusion-return-window-turnover"
BASE_SHA = "c6c4b152bba280d0ee1efb28839a964cf8de3832e22056dce9a7823a21f58410"
PROOF_SHA = "d983f93d23068fb17272d91919d669eada7089f4eca4e45de0796d79b1023570"
META, KEY, GROUP = prior.META, prior.KEY, prior.GROUP
PHASES, BRANCHES, CATEGORIES = prior.PHASES, prior.BRANCHES, prior.CATEGORIES
CLASSES = ("fallback", "new", "reset", "grow", "slide", "held", "other")
COMPARISONS = ("raw_larger", "mean_larger", "overlap", "not_comparable")
KINDS = (
    "audited",
    *CLASSES,
    *COMPARISONS,
    "prior_unknown_raw_larger",
    "nominal_only_reset_raw_larger",
)
REFS = tuple(
    f"{p}_{b}_turnover_{k}_refs" for p, b in product(PHASES, BRANCHES) for k in KINDS
)
HAS = tuple("has_available_turnover_" + k for k in ("grow", "slide", "raw_larger"))
RAW = prior.RAW
H, P, R = prior.H, prior.P, prior.SENSE_RADIUS
FIELDS = (
    "previous_step",
    "previous_count",
    "previous_member_steps",
    "previous_clear_step",
    "previous_clear_reasons",
    "added_steps",
    "removed_steps",
    "shared_steps",
    "added_count",
    "removed_count",
    "shared_count",
    "current_window_cleared",
    "turnover_class",
    "step_comparison",
    *(f"previous_{k}_{a}_center" for k in ("mean", "sensed") for a in ("x", "y")),
    *(context + "_input_" + k for context in ("previous", "current") for k in RAW),
    *(
        f"{kind}_step_{axis}_{bound}"
        for kind in ("mean", "raw", "gap")
        for axis in ("x", "y")
        for bound in ("center", "lo", "hi")
    ),
    *(
        f"{kind}_step_norm_{bound}"
        for kind in ("mean", "raw", "gap")
        for bound in ("lo", "hi")
    ),
    *(
        f"membership_prediction_{axis}_{bound}"
        for axis in ("x", "y")
        for bound in ("center", "lo", "hi")
    ),
    *(
        f"{kind}_residual_{axis}"
        for kind in ("membership", "gap_identity")
        for axis in ("x", "y")
    ),
)
sha, read, write, flag, step = (
    prior.sha,
    prior.read,
    prior.write,
    prior.flag,
    prior.step,
)
probe = prior.prior.prior.previous
weights = prior.prior.prior.weights


def memberships(before, after, n):
    left = {s["source_step"]: s for s in before["members"]}
    right = {s["source_step"]: s for s in after["members"]}
    assert len(left) == len(before["members"]) and len(right) == len(after["members"])
    for k in left.keys() & right.keys():
        assert left[k] == right[k], k
    old = list(left)
    new = list(right)
    added = sorted(right.keys() - left.keys())
    removed = sorted(left.keys() - right.keys())
    shared = sorted(left.keys() & right.keys())
    cleared = after["fields"]["window_last_clear_step"] == n
    if not new:
        kind = "fallback"
    elif not old:
        kind = "new"
    elif cleared:
        kind = "reset"
    elif new[:-1] == old and new[-1] == n:
        kind = "grow"
    elif len(new) == len(old) == 25 and new[:-1] == old[1:] and new[-1] == n:
        kind = "slide"
    elif new == old:
        kind = "held"
    else:
        kind = "other"
    return left, right, added, removed, shared, cleared, kind


def plus(a, b):
    return a[0] + b[0] - P, a[1] + b[1] + P


def minus(a, b):
    return a[0] - b[1] - P, a[1] - b[0] + P


def norm_box(pairs):
    lower = math.sqrt(
        sum((0 if lo <= 0 <= hi else min(abs(lo), abs(hi))) ** 2 for lo, hi in pairs)
    )
    upper = math.sqrt(sum(max(abs(lo), abs(hi)) ** 2 for lo, hi in pairs))
    return max(0.0, lower - P), upper + P


def compare_norms(raw, mean, eligible):
    if not eligible:
        return "not_comparable"
    if raw[0] > mean[1]:
        return "raw_larger"
    if mean[0] > raw[1]:
        return "mean_larger"
    return "overlap"


def turnover(d, before, after):
    n = step(d["time_s"])
    left, right, added, removed, shared, cleared, kind = memberships(before, after, n)
    out = dict(
        previous_step=n - 1,
        previous_count=len(left),
        previous_member_steps=";".join(map(str, left)),
        previous_clear_step=before["fields"]["window_last_clear_step"],
        previous_clear_reasons=before["fields"]["window_last_clear_reasons"],
        added_steps=";".join(map(str, added)),
        removed_steps=";".join(map(str, removed)),
        shared_steps=";".join(map(str, shared)),
        added_count=len(added),
        removed_count=len(removed),
        shared_count=len(shared),
        current_window_cleared=cleared,
        turnover_class=kind,
    )
    out.update(dict.fromkeys(k for k in FIELDS if k.startswith("membership_")))
    for name, context in (("previous", before), ("current", after)):
        out.update({name + "_input_" + k: context["point"][k] for k in RAW})
    radius_before = R + 2 * P if left else R
    radius_after = R + 2 * P if right else R
    boxes = defaultdict(list)
    for axis in ("x", "y"):
        pm = float(before["fields"]["mean_" + axis + "_center"])
        cm = float(after["fields"]["mean_" + axis + "_center"])
        ps = before["point"]["sensed_" + axis + "_center"]
        cs = after["point"]["sensed_" + axis + "_center"]
        out["previous_mean_" + axis + "_center"] = pm
        out["previous_sensed_" + axis + "_center"] = ps
        values = {"mean": cm - pm, "raw": cs - ps, "gap": (cm - cs) - (pm - ps)}
        radii = {
            "mean": radius_after + radius_before + P,
            "raw": 2 * R + P,
            "gap": radius_after + radius_before + 2 * R + 3 * P,
        }
        for k, value in values.items():
            bounds = value - radii[k], value + radii[k]
            boxes[k].append(bounds)
            out.update(
                {
                    f"{k}_step_{axis}_{b}": v
                    for b, v in zip(("center", "lo", "hi"), (value, *bounds))
                }
            )
        residual = values["gap"] - (values["mean"] - values["raw"])
        assert abs(residual) <= P
        out["gap_identity_residual_" + axis] = residual
        if left and right:
            add_center = prior.fold(
                right[i]["sensed_" + axis + "_center"] - pm for i in added
            )
            remove_center = prior.fold(
                left[i]["sensed_" + axis + "_center"] - pm for i in removed
            )
            prediction = (add_center - remove_center) / len(right)
            sums = []
            for ids, members in ((added, right), (removed, left)):
                interval = (0.0, 0.0)
                for i in ids:
                    value = members[i]["sensed_" + axis + "_center"]
                    interval = plus(
                        interval,
                        minus(
                            (value - R, value + R),
                            (pm - radius_before, pm + radius_before),
                        ),
                    )
                sums.append(interval)
            lo, hi = minus(*sums)
            lo = lo / len(right) - P
            hi = hi / len(right) + P
            assert lo <= values["mean"] <= hi
            assert lo <= boxes["mean"][-1][1] and hi >= boxes["mean"][-1][0]
            residual = prediction - values["mean"]
            assert abs(residual) <= P
            out.update(
                {
                    f"membership_prediction_{axis}_{b}": v
                    for b, v in zip(("center", "lo", "hi"), (prediction, lo, hi))
                }
            )
            out["membership_residual_" + axis] = residual
    norms = {}
    for kind, pairs in boxes.items():
        norms[kind] = norm_box(pairs)
        out.update(
            {f"{kind}_step_norm_{b}": v for b, v in zip(("lo", "hi"), norms[kind])}
        )
    out["step_comparison"] = compare_norms(
        norms["raw"], norms["mean"], bool(left and right)
    )
    assert set(out) == set(FIELDS), (set(out) - set(FIELDS), set(FIELDS) - set(out))
    members = []
    for order, i in enumerate(sorted(left.keys() | right.keys()), 1):
        point = right[i] if i in right else left[i]
        members.append(
            dict(
                **{k: d[k] for k in (*META, "event_id", "time_s", "phase", "branch")},
                union_order=order,
                member_role=(
                    "shared"
                    if i in left and i in right
                    else "added" if i in right else "removed"
                ),
                in_previous=i in left,
                in_current=i in right,
                **point,
            )
        )
    return out, members


def check_current(d, context):
    for k, value in context["fields"].items():
        if isinstance(value, (str, int, bool)):
            assert d[k] == str(value), (k, d[k], value)
        else:
            assert math.isclose(float(d[k]), value, rel_tol=1e-10, abs_tol=2e-12), (
                k,
                d[k],
                value,
            )
    for k in (
        "recovery_x",
        "recovery_y",
        "recovery_yaw",
        "recovery_clearance",
        "recovery_sample_count",
        "speed_margin",
    ):
        assert float(d[k]) == float(context["raw"][k]), k


def collect(cases, selected, raw, roots):
    files = defaultdict(dict)
    wanted = set()
    selected_cases = {d["case_id"] for d in selected}
    for c in cases:
        files[c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"][
            c["scenario"], c["planner"]
        ] = c
    for d in selected:
        n = step(d["time_s"])
        assert n > 1
        wanted.update(((d["case_id"], n - 1), (d["case_id"], n)))
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
                    probe.noisy_branch({k: cells[j] for k, j in pos.items()})
                    audit["noisy_branch_histories"] += 1
                if c["case_id"] not in selected_cases:
                    continue
                row = dict(zip(header, cells))
                if n == 0:
                    windows[identity] = prior.Window(row)
                    audit["replayed_histories"] += 1
                else:
                    w = windows[identity]
                    point = w.update(n, before[identity], row)
                    audit["replayed_steps"] += 1
                    if (c["case_id"], n) in wanted:
                        fields, _ = prior.reconstruct(w, point, row)
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
            print(f"Turnover contexts read: {i}/108 files", flush=True)
    assert snapshots.keys() == wanted
    audit["unique_context_points"] = len(snapshots)
    return snapshots, dict(audit)


def anchor(e, dd, ne, nr):
    out = dict(e, **weights(e, ne, nr), **dict.fromkeys((*REFS, *HAS)))
    if not flag(e["returned"] or "False"):
        assert not dd
        return out
    assert all(all(d[k] == e[k] for k in META) for d in dd)
    counts = Counter()
    for d in dd:
        if not d["turnover_audited"]:
            continue
        kinds = ["audited", d["turnover_class"], d["step_comparison"]]
        if d["step_comparison"] == "raw_larger":
            if d["prior_certainty"] == "unknown":
                kinds.append("prior_unknown_raw_larger")
            if d["timer_action"] == "reset" and d["refined_failed_gates"] == "nominal":
                kinds.append("nominal_only_reset_raw_larger")
        for k in kinds:
            counts[f"{d['phase']}_{d['branch']}_turnover_{k}_refs"] += 1
    out.update({k: counts[k] for k in REFS})
    for kind in ("grow", "slide", "raw_larger"):
        out["has_available_turnover_" + kind] = any(
            d["phase"] == "available"
            and d["turnover_audited"]
            and (d["step_comparison"] if kind == "raw_larger" else d["turnover_class"])
            == kind
            for d in dd
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
            prefix = f"{p}_{b}_turnover_"
            assert (
                row[prefix + "audited_refs"]
                == sum(row[prefix + k + "_refs"] for k in CLASSES)
                == sum(row[prefix + k + "_refs"] for k in COMPARISONS)
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
            "fusion-return-recovery-window/" + n: h
            for n, h in source["output_sha256"].items()
        },
        "fusion-return-recovery-window/sources.json": BASE_SHA,
        "fusion-return-recovery-window/independent-verification.json": PROOF_SHA,
        "fusion-return-recovery-window/protocol.md": source["analysis"][
            "protocol_sha256"
        ],
        "fusion-return-recovery-window/freeze-pins.json": source["analysis"][
            "freeze_sha256"
        ],
    }
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_recovery_window.py": source["analysis"]["script_sha256"],
    }
    vhelpers = {
        **source["analysis"]["verifier_helpers_sha256"],
        "verify_recovery_window.py": freeze["verifier_helper_sha256"],
    }
    for n, h in pins.items():
        assert sha(RESULTS / n) == h, n
    for n, h in {**helpers, **vhelpers}.items():
        assert sha(ROOT / n) == h, n
    for n, h in source["analysis"]["code_sha256"].items():
        assert sha(REPO / n) == h, n
    contract = prior.contract(
        (REPO / next(iter(source["analysis"]["code_sha256"]))).read_text()
    )
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    probe.provenance(source["raw"], roots, proof["native"]["rustc"])
    cases = read(RESULTS / "census-rp-selection/case-selection.csv")
    old = read(BASE / "window-decisions.csv")
    selected = [d for d in old if float(d["speed_free_distance"]) == 0]
    assert len(selected) == 1096
    snapshots, audit = collect(cases, selected, source["raw"], roots)
    decisions = []
    members = []
    for d in old:
        audited = float(d["speed_free_distance"]) == 0
        out = dict(d, turnover_audited=audited, **dict.fromkeys(FIELDS))
        if audited:
            n = step(d["time_s"])
            before = snapshots[d["case_id"], n - 1]
            after = snapshots[d["case_id"], n]
            check_current(d, after)
            fields, lines = turnover(d, before, after)
            out.update(fields)
            members.extend(lines)
        decisions.append(out)
    events = read(BASE / "anchor-window.csv")
    ne = Counter(e["case_id"] for e in events)
    nr = Counter(e["case_id"] for e in events if flag(e["received"]))
    db = defaultdict(list)
    for d in decisions:
        db[d["event_id"]].append(d)
    anchors = [
        anchor(e, db[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]]) for e in events
    ]
    summary = summaries(read(BASE / "window-summary.csv"), anchors)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(flag(e["returned"] or "False") for e in events),
        decision_references=len(decisions),
        audited_zero_probes=len(selected),
        context_queries=2 * len(selected),
        member_rows=len(members),
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
        ("turnover-decisions", decisions),
        ("turnover-members", members),
        ("anchor-turnover", anchors),
        ("turnover-summary", summary),
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
            source_contract=contract,
        ),
        schema=dict(
            half_unit=H,
            elemental_allowance=P,
            sensed_radius=R,
            closure_allowance=P,
            classes=CLASSES,
            comparisons=COMPARISONS,
            phases=PHASES,
            branches=BRANCHES,
            retrospective=True,
            inherited="all prior columns and outcomes; predecessor inputs are context, not new decisions; bounded adjacent source steps, no causal intervention",
        ),
        inherited_recovery_window_proof_sha256=PROOF_SHA,
        inherited_validation=source["inherited_validation"],
        audit=audit,
        output_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
