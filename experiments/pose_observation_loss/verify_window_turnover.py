#!/usr/bin/env python3
"""Independently bound adjacent recovery-window changes with Decimal."""

import argparse
from collections import Counter, defaultdict
import csv
from decimal import Decimal as D, localcontext
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import verify_recovery_window as previous

core = previous.core
ROOT, REPO, RESULTS = core.ROOT, core.REPO, core.RESULTS
BASE = RESULTS / "fusion-return-recovery-window"
STUDY = RESULTS / "fusion-return-window-turnover"
BASE_SHA = "c6c4b152bba280d0ee1efb28839a964cf8de3832e22056dce9a7823a21f58410"
PROOF_SHA = "d983f93d23068fb17272d91919d669eada7089f4eca4e45de0796d79b1023570"
META, KEY, GROUP = previous.META, previous.KEY, previous.GROUP
PHASES, BRANCHES, CATEGORIES = previous.PHASES, previous.BRANCHES, previous.CATEGORIES
RAW = previous.RAW
H, P = D("5e-10"), D("1e-12")
R = 3 * H + 2 * P
CODE = previous.previous.standalone.CODE
HELPERS = {
    **previous.HELPERS,
    "verify_recovery_window.py": "28edbff6f46596caaed55326bdc12e93157891a5049d21b2fc6b9d6ff4a6889f",
}

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


def ids_and_class(left, right, cleared, n):
    old = [p["source_step"] for p in left]
    new = [p["source_step"] for p in right]
    assert len(set(old)) == len(old) and len(set(new)) == len(new)
    before = {p["source_step"]: p for p in left}
    after = {p["source_step"]: p for p in right}
    for k in before.keys() & after.keys():
        assert before[k] == after[k]
    added = [k for k in sorted(after) if k not in before]
    removed = [k for k in sorted(before) if k not in after]
    shared = sorted(set(old).intersection(new))
    if len(new) == 0:
        kind = "fallback"
    elif len(old) == 0:
        kind = "new"
    elif cleared:
        kind = "reset"
    elif len(new) == len(old) + 1 and new[-1] == n and new[:-1] == old:
        kind = "grow"
    elif len(old) == 25 and len(new) == 25 and new[-1] == n and old[1:] == new[:-1]:
        kind = "slide"
    elif old == new:
        kind = "held"
    else:
        kind = "other"
    return before, after, added, removed, shared, kind


def box_norm(pairs):
    low = D(0)
    high = D(0)
    for lo, hi in pairs:
        assert lo <= hi
        low += (D(0) if lo <= 0 <= hi else min(abs(lo), abs(hi))) ** 2
        high += max(abs(lo), abs(hi)) ** 2
    return max(D(0), low.sqrt() - P), high.sqrt() + P


def norm_comparison(raw, mean, eligible):
    if not eligible:
        return "not_comparable"
    if raw[0] > mean[1]:
        return "raw_larger"
    if mean[0] > raw[1]:
        return "mean_larger"
    return "overlap"


def turnover(d, before, after, native_before=None, native_after=None):
    with localcontext() as ctx:
        ctx.prec = 60
        n = core.tick(d["time_s"])
        cleared = after["fields"]["window_last_clear_step"] == n
        left, right, added, removed, shared, kind = ids_and_class(
            before["members"], after["members"], cleared, n
        )
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
        for name, point in (("previous", before["point"]), ("current", after["point"])):
            for k in RAW:
                out[name + "_input_" + k] = point[k]
        scalar = lambda value: (
            value if isinstance(value, D) else D.from_float(float(value))
        )
        previous_radius = R + 2 * P if left else R
        current_radius = R + 2 * P if right else R
        bounds = {k: [] for k in ("mean", "raw", "gap")}
        for axis in ("x", "y"):
            pm = scalar(before["fields"]["mean_" + axis + "_center"])
            cm = scalar(after["fields"]["mean_" + axis + "_center"])
            ps = scalar(before["point"]["sensed_" + axis + "_center"])
            cs = scalar(after["point"]["sensed_" + axis + "_center"])
            out["previous_mean_" + axis + "_center"] = pm
            out["previous_sensed_" + axis + "_center"] = ps
            differences = dict(mean=cm - pm, raw=cs - ps, gap=(cm - cs) - (pm - ps))
            radii = dict(
                mean=current_radius + previous_radius + P,
                raw=2 * R + P,
                gap=current_radius + previous_radius + 2 * R + 3 * P,
            )
            for name, value in differences.items():
                lo, hi = value - radii[name], value + radii[name]
                bounds[name].append((lo, hi))
                out.update(
                    {
                        f"{name}_step_{axis}_center": value,
                        f"{name}_step_{axis}_lo": lo,
                        f"{name}_step_{axis}_hi": hi,
                    }
                )
            residual = differences["gap"] - (differences["mean"] - differences["raw"])
            assert abs(residual) <= P
            out["gap_identity_residual_" + axis] = residual
            if left and right:
                numerator = D(0)
                uncertainty = D(0)
                # Separate Decimal sums; the interval radius follows each frozen
                # subtract/add, the sum subtraction and final division.
                for indices, mapping, sign in (
                    (added, right, D(1)),
                    (removed, left, D(-1)),
                ):
                    for i in indices:
                        numerator += sign * (
                            scalar(mapping[i]["sensed_" + axis + "_center"]) - pm
                        )
                        uncertainty += R + previous_radius + 2 * P
                prediction = numerator / len(right)
                radius = (uncertainty + P) / len(right) + P
                lo, hi = prediction - radius, prediction + radius
                assert lo <= differences["mean"] <= hi
                assert lo <= bounds["mean"][-1][1] and hi >= bounds["mean"][-1][0]
                residual = prediction - differences["mean"]
                assert abs(residual) <= P
                out.update(
                    {
                        f"membership_prediction_{axis}_center": prediction,
                        f"membership_prediction_{axis}_lo": lo,
                        f"membership_prediction_{axis}_hi": hi,
                        f"membership_residual_{axis}": residual,
                    }
                )
                if native_before is not None:
                    native_change = D.from_float(
                        float(native_after["mean_" + axis])
                    ) - D.from_float(float(native_before["mean_" + axis]))
                    assert lo <= native_change <= hi
                    assert (
                        bounds["mean"][-1][0] <= native_change <= bounds["mean"][-1][1]
                    )
        norms = {name: box_norm(pairs) for name, pairs in bounds.items()}
        for name, pair in norms.items():
            out.update(
                {f"{name}_step_norm_{s}": value for s, value in zip(("lo", "hi"), pair)}
            )
        out["step_comparison"] = norm_comparison(
            norms["raw"], norms["mean"], bool(left and right)
        )
        assert set(out) == set(FIELDS)
        member_rows = []
        for ordinal, i in enumerate(sorted(set(left).union(right)), 1):
            row = {k: d[k] for k in (*META, "event_id", "time_s", "phase", "branch")}
            row.update(
                union_order=ordinal,
                member_role=(
                    "shared"
                    if i in left and i in right
                    else "added" if i in right else "removed"
                ),
                in_previous=i in left,
                in_current=i in right,
            )
            row.update(right.get(i, left.get(i)))
            member_rows.append(row)
        return out, member_rows


def check_current(d, context):
    for k, value in context["fields"].items():
        if isinstance(value, (str, int, bool)):
            core.check(d[k], value)
        else:
            if k == "mean_yaw_center":
                assert float(previous.circular_error(D(d[k]), D(str(value)))) <= 1e-9
            else:
                assert math.isclose(
                    float(d[k]), float(value), rel_tol=1e-10, abs_tol=1e-9
                ), (k, d[k], value)
            core.comparisons += 1
    for k in (
        "recovery_x",
        "recovery_y",
        "recovery_yaw",
        "recovery_clearance",
        "recovery_sample_count",
        "speed_margin",
    ):
        assert float(d[k]) == float(context["raw"][k]), k


def context_queries(selected):
    queries = defaultdict(list)
    query_rows = []
    for i, d in enumerate(selected):
        n = core.tick(d["time_s"])
        assert n > 1
        for offset, role, j in ((1, "previous", n - 1), (2, "current", n)):
            row_id = 2 * i + offset
            queries[d["case_id"], j].append(row_id)
            query_rows.append(
                dict(
                    row_id=row_id,
                    case_id=d["case_id"],
                    event_id=d["event_id"],
                    reference_time_s=d["time_s"],
                    context_role=role,
                    source_time_s=j / 50,
                )
            )
    return queries, query_rows


def raw_inputs(cases, selected, raw, roots, work):
    files = defaultdict(dict)
    queries, query_rows = context_queries(selected)
    for c in cases:
        files[c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"][
            c["scenario"], c["planner"]
        ] = c
    assert set(files) == {(c, n) for c, s in raw.items() for n in s["raw_sha256"]}
    selected_cases = {d["case_id"] for d in selected}
    snapshots = {}
    audit = Counter()
    work.mkdir(parents=True)
    with (work / "native-replay-inputs.tsv").open("w") as wire:
        wire.write(
            "case_id\tstep\tpaused\tcorrection\treceived\tprecision\tspeed\tage\tx\ty\tyaw\tlogged_count\trow_ids\n"
        )
        for i, ((cohort, name), by_identity) in enumerate(sorted(files.items()), 1):
            path = Path(roots.get(cohort, raw[cohort]["path"])) / name
            assert core.sha(path) == raw[cohort]["raw_sha256"][name]
            audit["raw_bytes"] += path.stat().st_size
            last = defaultdict(lambda: -1)
            before = {}
            replays = {}
            with path.open() as f:
                header = next(f).rstrip("\r\n").split(",")
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
                for line in f:
                    assert '"' not in line
                    cells = line.rstrip("\r\n").split(",")
                    assert len(cells) == len(header)
                    identity = cells[pos["scenario"]], cells[pos["planner"]]
                    assert identity in by_identity
                    c = by_identity[identity]
                    n = core.tick(cells[pos["time_s"]])
                    assert n == last[identity] + 1
                    last[identity] = n
                    if not core.truth(c["selected_timeout"]):
                        continue
                    audit["timeout_grid_rows"] += 1
                    if n == 0:
                        vv = [
                            D(cells[pos[k]])
                            for k in (
                                "effective_position_sigma",
                                "effective_yaw_sigma_rad",
                            )
                        ]
                        assert all(v.is_finite() and v >= 0 for v in vv) and any(
                            v > 0 for v in vv
                        )
                        audit["noisy_branch_histories"] += 1
                    if c["case_id"] not in selected_cases:
                        continue
                    row = dict(zip(header, cells))
                    if n == 0:
                        replays[identity] = previous.Replay(row)
                        audit["replayed_histories"] += 1
                    else:
                        replay = replays[identity]
                        point, members, age, paused = replay.advance(
                            n, before[identity], row
                        )
                        audit["replayed_steps"] += 1
                        row_ids = queries.get((c["case_id"], n), [])
                        values = [
                            c["case_id"],
                            str(n),
                            str(paused).lower(),
                            row["correction"],
                            row["correction_received"],
                            row["precision_activation"],
                            format(float(row["measured_speed"]), ".17e"),
                            format(age, ".17e"),
                            *(
                                format(point["sensed_" + a + "_center"], ".17e")
                                for a in ("x", "y", "yaw")
                            ),
                            row["recovery_sample_count"],
                            ";".join(map(str, row_ids)),
                        ]
                        wire.write("\t".join(values) + "\n")
                        if row_ids:
                            snapshots[c["case_id"], n] = dict(
                                point=point,
                                members=list(members),
                                raw=dict(row),
                                clear_step=replay.clear_step,
                                reasons=replay.reasons,
                                pushes=len(replay.admissions),
                            )
                    before[identity] = row
            assert set(last) == set(by_identity)
            for identity, c in by_identity.items():
                assert last[identity] == (
                    9000
                    if core.truth(c["selected_timeout"])
                    else core.tick(c["time_s"])
                )
            audit["raw_files"] += 1
            if i % 18 == 0:
                print(f"Independent turnover contexts read: {i}/108 files", flush=True)
    assert snapshots.keys() == queries.keys()
    audit["unique_context_points"] = len(snapshots)
    return snapshots, query_rows, dict(audit)


def anchor(e, dd, ne, nr):
    parent = int(e["parent_rp_records"])
    measures = dict(
        census_case_weight=Fraction(1, ne),
        received_case_weight=(
            Fraction(1, nr) if core.truth(e["received"]) else Fraction()
        ),
        rp_case_mass=Fraction(int(e["rp_records"]), parent) if parent else Fraction(),
        rp_different_preview_case_mass=(
            Fraction(int(e["rp_different_preview_records"]), parent)
            if parent
            else Fraction()
        ),
    )
    for k, v in measures.items():
        core.check(e[k], v)
    out = dict(e, **measures, **dict.fromkeys((*REFS, *HAS)))
    if not core.truth(e["returned"] or "False"):
        assert not dd
        return out
    assert all(all(d[k] == e[k] for k in META) for d in dd)
    for p, b in product(PHASES, BRANCHES):
        rows = [
            d
            for d in dd
            if d["phase"] == p and d["branch"] == b and d["turnover_audited"]
        ]
        values = dict(audited=len(rows))
        values.update({k: sum(d["turnover_class"] == k for d in rows) for k in CLASSES})
        values.update(
            {k: sum(d["step_comparison"] == k for d in rows) for k in COMPARISONS}
        )
        values["prior_unknown_raw_larger"] = sum(
            d["prior_certainty"] == "unknown" and d["step_comparison"] == "raw_larger"
            for d in rows
        )
        values["nominal_only_reset_raw_larger"] = sum(
            d["timer_action"] == "reset"
            and d["refined_failed_gates"] == "nominal"
            and d["step_comparison"] == "raw_larger"
            for d in rows
        )
        out.update({f"{p}_{b}_turnover_{k}_refs": v for k, v in values.items()})
    rows = [d for d in dd if d["phase"] == "available" and d["turnover_audited"]]
    for k in ("grow", "slide", "raw_larger"):
        out["has_available_turnover_" + k] = any(
            d["step_comparison"] == k if k == "raw_larger" else d["turnover_class"] == k
            for d in rows
        )
    return out


def closure(rows):
    checks = 0
    for key, r in rows.items():
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
                        rows[key[:i] + (p,) + key[i + 1 :]][f] for p in parts
                    )
                    checks += 1
        for p, b in product(PHASES, BRANCHES):
            prefix = f"{p}_{b}_turnover_"
            for group in (CLASSES, COMPARISONS):
                assert (
                    sum(r[prefix + k + "_refs"] for k in group)
                    == r[prefix + "audited_refs"]
                )
    return checks


def compare(path, rows, keys):
    expected = {tuple(str(r[k]) for k in keys): r for r in rows}
    actual = core.index(core.read(path), keys)
    assert len(expected) == len(rows) and actual.keys() == expected.keys()
    for key, row in actual.items():
        assert set(row) == set(expected[key])
        for field, value in expected[key].items():
            if field in FIELDS and isinstance(value, (D, float)):
                assert math.isclose(
                    float(row[field]), float(value), rel_tol=1e-10, abs_tol=1e-9
                ), (key, field, row[field], value)
                core.comparisons += 1
            else:
                core.check(row[field], value)


def summaries(old, anchors):
    result = {}
    for r in old:
        key = tuple(r[k] for k in GROUP)
        p, n, arm, mode, cat = key
        aa = [
            a
            for a in anchors
            if (a["partition"], a["noise"], a["variant"]) == (p, n, arm)
            and (mode == "all_modes" or a["mode"] == mode)
            and (cat == "all" or a["anchor_category"] == cat)
        ]
        rr = [a for a in aa if core.truth(a["returned"] or "False")]
        mass = lambda rows: sum((a["census_case_weight"] for a in rows), Fraction())
        out = dict(r)
        for field in REFS:
            out[field] = sum(a[field] or 0 for a in aa)
        for h in HAS:
            selected = [a for a in aa if a[h]]
            out.update(
                {
                    h + "_events": len(selected),
                    h + "_case_mass": mass(selected),
                    h + "_fraction": Fraction(len(selected), len(rr)) if rr else None,
                    h
                    + "_case_mass_fraction": mass(selected) / mass(rr) if rr else None,
                }
            )
        core.check(r["census_case_mass"], mass(aa))
        assert key not in result
        result[key] = out
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for n in ("results", "figures", "work", "output"):
        p.add_argument("--" + n, type=Path, required=True)
    p.add_argument("--rustc", type=Path, default=shutil.which("rustc"))
    p.add_argument("--raw-roots", type=Path)
    args = p.parse_args()
    if not __debug__ or args.work.exists() or args.output.exists() or not args.rustc:
        p.error("Use new work/output paths, provide rustc and run without -O")
    prior = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    source = json.loads((args.results / "sources.json").read_text())
    assert (
        core.sha(BASE / "sources.json") == BASE_SHA
        and core.sha(BASE / "independent-verification.json") == PROOF_SHA
    )
    freeze = json.loads((STUDY / "freeze-pins.json").read_text())
    assert freeze == dict(
        base_main="bd57feeb517c31034e78e703ff77048c6a25104d",
        sources_sha256=BASE_SHA,
        proof_sha256=PROOF_SHA,
        verifier_helper_sha256=HELPERS["verify_recovery_window.py"],
    )
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        **{
            "fusion-return-recovery-window/" + n: h
            for n, h in prior["output_sha256"].items()
        },
        "fusion-return-recovery-window/sources.json": BASE_SHA,
        "fusion-return-recovery-window/independent-verification.json": PROOF_SHA,
        "fusion-return-recovery-window/protocol.md": prior["analysis"][
            "protocol_sha256"
        ],
        "fusion-return-recovery-window/freeze-pins.json": prior["analysis"][
            "freeze_sha256"
        ],
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_recovery_window.py": prior["analysis"]["script_sha256"],
    }
    assert (
        source["analysis"]["verifier_helpers_sha256"] == HELPERS
        and source["analysis"]["code_sha256"] == CODE
    )
    assert (
        source["raw"] == prior["raw"]
        and source["map_sha256"] == prior["map_sha256"]
        and source["inherited_recovery_window_proof_sha256"] == PROOF_SHA
        and source["inherited_validation"] == prior["inherited_validation"]
    )
    assert source["schema"] == dict(
        half_unit=5e-10,
        elemental_allowance=1e-12,
        sensed_radius=3 * 5e-10 + 2 * 1e-12,
        closure_allowance=1e-12,
        classes=list(CLASSES),
        comparisons=list(COMPARISONS),
        phases=list(PHASES),
        branches=list(BRANCHES),
        retrospective=True,
        inherited="all prior columns and outcomes; predecessor inputs are context, not new decisions; bounded adjacent source steps, no causal intervention",
    )
    for n, h in source["input_sha256"].items():
        assert core.sha(RESULTS / n) == h, n
    for n, h in {**source["analysis"]["helpers_sha256"], **HELPERS}.items():
        assert core.sha(ROOT / n) == h, n
    for n, h in CODE.items():
        assert core.sha(REPO / n) == h, n
    assert source["analysis"]["source_contract"] == previous.contract(
        (REPO / next(iter(core.CODE))).read_text()
    )
    for f, path in (
        ("script_sha256", ROOT / "analyze_window_turnover.py"),
        ("protocol_sha256", STUDY / "protocol.md"),
        ("freeze_sha256", STUDY / "freeze-pins.json"),
    ):
        assert source["analysis"][f] == core.sha(path)
    assert set(source["output_sha256"]) == {
        "turnover-decisions.csv",
        "turnover-members.csv",
        "anchor-turnover.csv",
        "turnover-summary.csv",
    }
    for n, h in source["output_sha256"].items():
        assert core.sha(args.results / n) == h, n
    compiler = subprocess.run(
        [str(args.rustc), "--version"], check=True, capture_output=True, text=True
    ).stdout.strip()
    assert compiler == proof["native"]["rustc"]
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    for cohort, spec in source["raw"].items():
        path = Path(roots.get(cohort, spec["path"])) / "provenance.json"
        assert core.sha(path) == spec["provenance_sha256"]
        record = json.loads(path.read_text())
        assert (
            record["rustc"] == compiler
            and record["inputs_sha256"]["obstacles.csv"] == source["map_sha256"]
        )
        assert set(record["inputs_by_variant_sha256"]) == {"epoch", "curved"} and all(
            v["obstacles.csv"] == source["map_sha256"]
            for v in record["inputs_by_variant_sha256"].values()
        )
    cases, original = core.original_inputs()
    events = core.read(BASE / "anchor-window.csv")
    assert {e["event_id"] for e in events} == original.keys()
    for e in events:
        for k in (
            *META,
            "source_event_id",
            "event_kind",
            "event_time_s",
            "scheduled",
            "received",
            "precision_activation",
            "horizon",
            "parent_rp_records",
            "rp_records",
            "rp_different_preview_records",
        ):
            core.check(e[k], original[e["event_id"]][k])
    original_checks = core.comparisons
    old = core.read(BASE / "window-decisions.csv")
    selected = [d for d in old if D(d["speed_free_distance"]) == 0]
    assert len(selected) == 1096
    snapshots, query_rows, audit = raw_inputs(
        cases, selected, source["raw"], roots, args.work
    )
    queries = [
        dict(
            case_id=q["case_id"], event_id=q["event_id"], time_s=str(q["source_time_s"])
        )
        for q in query_rows
    ]
    traces, native_proof = previous.native(args.work, args.rustc, queries)
    assert native_proof["replayed_steps"] == audit["replayed_steps"]
    native_proof["traced_context_queries"] = native_proof.pop("traced_references")
    context_map = args.work / "native-contexts.csv"
    with context_map.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=query_rows[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(query_rows)
    native_proof["context_map_sha256"] = core.sha(context_map)
    contexts = {}
    for q in query_rows:
        key = (q["case_id"], core.tick(q["source_time_s"]))
        snap = snapshots[key]
        trace = traces[q["row_id"]]
        assert int(trace["step"]) == key[1]
        replay = SimpleNamespace(
            clear_step=snap["clear_step"],
            reasons=snap["reasons"],
            admissions=[None] * snap["pushes"],
        )
        fields, _ = previous.reconstruct(
            replay, snap["point"], snap["members"], snap["raw"], trace
        )
        contexts[q["row_id"]] = dict(snap, fields=fields)
    decisions = []
    member_rows = []
    selected_index = {(d["event_id"], d["time_s"]): i for i, d in enumerate(selected)}
    for d in old:
        audited = D(d["speed_free_distance"]) == 0
        out = dict(d, turnover_audited=audited, **dict.fromkeys(FIELDS))
        if audited:
            i = selected_index[d["event_id"], d["time_s"]]
            a, b = 2 * i + 1, 2 * i + 2
            before, after = contexts[a], contexts[b]
            check_current(d, after)
            fields, lines = turnover(d, before, after, traces[a], traces[b])
            out.update(fields)
            member_rows.extend(lines)
        decisions.append(out)
    db = defaultdict(list)
    for d in decisions:
        db[d["event_id"]].append(d)
    ne = Counter(e["case_id"] for e in original.values())
    nr = Counter(e["case_id"] for e in original.values() if core.truth(e["received"]))
    anchors = [
        anchor(e, db[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]]) for e in events
    ]
    summary = summaries(core.read(BASE / "window-summary.csv"), anchors)
    for name, rows, keys in (
        ("turnover-decisions", decisions, ("event_id", "time_s")),
        ("turnover-members", member_rows, ("event_id", "time_s", "union_order")),
        ("anchor-turnover", anchors, ("event_id",)),
        ("turnover-summary", list(summary.values()), GROUP),
    ):
        compare(args.results / (name + ".csv"), rows, keys)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(core.truth(e["returned"] or "False") for e in events),
        decision_references=len(decisions),
        audited_zero_probes=len(selected),
        context_queries=2 * len(selected),
        member_rows=len(member_rows),
        summary_rows=len(summary),
        new_additive_closure_comparisons=closure(summary),
    )
    assert audit == source["audit"], (audit, source["audit"])
    assert (
        sum(a["census_case_weight"] for a in anchors)
        == sum(a["received_case_weight"] for a in anchors)
        == 570
    )
    assert (
        sum(a["rp_case_mass"] for a in anchors) == 157
        and sum(int(a["rp_records"]) for a in anchors) == 10995
    )
    plots = json.loads((args.figures / "plot-sources.json").read_text())
    assert plots["script_sha256"] == core.sha(ROOT / "plot_window_turnover.py")
    assert plots["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in ("sources.json", "anchor-turnover.csv", "turnover-summary.csv")
    }
    assert set(plots["output_sha256"]) == {
        "window-turnover.svg",
        "plot-values.csv",
        "plot-return-values.csv",
    }
    for n, h in plots["output_sha256"].items():
        assert core.sha(args.figures / n) == h
    compare(
        args.figures / "plot-values.csv",
        [r for r in summary.values() if r["mode"] != "all_modes"],
        GROUP,
    )
    compare(
        args.figures / "plot-return-values.csv",
        [a for a in anchors if core.truth(a["returned"] or "False")],
        ("event_id",),
    )
    evidence = dict(
        **audit,
        comparisons=core.comparisons - original_checks,
        original_join_comparisons=original_checks,
        plotted_summary_rows=252,
        plotted_return_rows=137,
        decimal_precision=60,
        native=native_proof,
        inherited_recovery_window_proof_sha256=PROOF_SHA,
        script_sha256=core.sha(Path(__file__)),
        sources_sha256=core.sha(args.results / "sources.json"),
        plot_sources_sha256=core.sha(args.figures / "plot-sources.json"),
        method="independent Decimal adjacent vector intervals and membership identities; pinned independent replay and original native window at both contexts; prior fields inherited, no new follow-up or causal intervention",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
