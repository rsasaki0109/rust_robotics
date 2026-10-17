#!/usr/bin/env python3
"""Join frozen Fusion returned spells to original logged control decisions."""

import argparse
from collections import Counter, defaultdict
import csv
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path

from analyze_return_spell import (
    KEY,
    META,
    digest,
    flag,
    indexed,
    read_rows,
    step,
    write,
)

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
RESULTS = ROOT / "results"
BASE = RESULTS / "return-spell-mechanisms"
BASE_SHA = "e251852bc1bc8c1f2f1f974acaeaaf5660fb0be8b15f16eabd95b3fd3bdf914b"
PROOF_SHA = "b260b4fe2654a5026ca862b827e96cddcfc290fc85e3b09cfb1e6994fcef3be8"
PROTOCOL = RESULTS / "fusion-return-controls/protocol.md"
VERIFY_HELPERS = {
    "verify_epoch_retention.py": "417a3e93cc4680e77722fe2c95e24f206d532399bd4e2efc8d9951a24530e9f1",
}
CATEGORIES = (
    "ttl_ended",
    "age_ended",
    "returned_censored",
    "no_return",
    "unobserved_ttl_limit",
    "no_observed_loss",
)
LATCHES = ("loss_and_guard", "loss_only", "guard_only", "clear")
POSES = ("recovery_mean", "raw_fallback")
GROUP = ("partition", "noise", "variant", "mode", "category_selector")
RAW_BOOL = (
    "safety_paused",
    "guard_active",
    "resumed",
    "guard_triggered",
    "guard_released",
    "stop_latched",
    "speed_braking",
    "correction",
    "correction_received",
    "precision_activation",
    "epoch_fusion_available",
    "stationary_epoch_fusion",
    "recovery_preview_active",
)
RAW_FLOAT = (
    "time_s",
    "measured_speed",
    "observation_age",
    "target_speed",
    "nominal_target_speed",
    "steering",
    "nominal_steering",
    "recovery_evidence",
    "guard_release_evidence",
    "release_preview_clearance_lower_bound",
    "guard_release_margin",
)
RAW_COUNTS = (
    "recovery_sample_count",
    "motion_filter_sample_count",
    "epoch_fusion_group_count",
)
RAW_FIELDS = (*RAW_BOOL, *RAW_FLOAT, *RAW_COUNTS)
DESCRIBE = (
    "pose_branch",
    "preview_branch",
    "latch_state",
    "resumed",
    "target_positive",
    "nominal_positive",
    "target_speed",
    "nominal_target_speed",
    "recovery_sample_count",
    "sample_ready",
    "recovery_evidence",
    "safety_paused",
    "guard_active",
)
WITHIN = (
    "within_available_checkpoints",
    *("within_" + s + "_checkpoints" for s in (*LATCHES, *POSES)),
    *(
        "within_" + k + "_checkpoints"
        for k in (
            "resumed",
            "guard_triggered",
            "guard_released",
            "target_positive",
            "nominal_positive",
            "sample_ready",
        )
    ),
)
NEW_FLAGS = (
    "any_resume",
    "any_positive_target",
    "any_positive_nominal",
    "any_sample_ready",
    "all_available_loss_paused",
    "all_available_guarded",
    "all_available_zero_target",
    "end_pose_changed",
    "end_latch_changed",
    "end_target_positive_changed",
)
SUMMARY_FLAGS = (
    "preview_loss",
    "returned",
    "ended",
    "censored_return",
    "no_return",
    *NEW_FLAGS,
)
NEW_FIELDS = (
    "control_window_checkpoints",
    "control_run_count",
    *WITHIN,
    *NEW_FLAGS,
    "max_available_recovery_evidence_s",
    "min_available_recovery_samples",
    "max_available_recovery_samples",
    *(prefix + k for prefix in ("return_", "boundary_") for k in DESCRIBE),
)
RUN_FIELDS = (
    "pose_branch",
    "preview_branch",
    "safety_paused",
    "guard_active",
    "resumed",
    "target_positive",
)


def logged(row):
    out = {k: flag(row[k], raw=True) for k in RAW_BOOL}
    for k in RAW_FLOAT:
        out[k] = float(row[k])
        assert math.isfinite(out[k])
    for k in RAW_COUNTS:
        out[k] = int(row[k])
        assert str(out[k]) == row[k] and out[k] >= 0
    assert out["observation_age"] >= 0 and out["epoch_fusion_group_count"] <= 3
    assert out["recovery_sample_count"] <= 25
    assert not out["epoch_fusion_available"] or out["epoch_fusion_group_count"] == 3
    assert out["stationary_epoch_fusion"] and out["recovery_preview_active"]
    out.update(
        pose_branch="recovery_mean" if out["recovery_sample_count"] else "raw_fallback",
        preview_branch=(
            "three_epoch" if out["epoch_fusion_available"] else "motion_filter"
        ),
        latch_state=LATCHES[
            (0 if out["safety_paused"] else 2) + (0 if out["guard_active"] else 1)
        ],
        target_positive=out["target_speed"] > 0,
        nominal_positive=out["nominal_target_speed"] > 0,
        sample_ready=out["recovery_sample_count"] >= 10,
    )
    return out


def command_check(s):
    same = lambda x, y: abs(x - y) <= 2e-9
    assert s["target_speed"] >= 0 and s["nominal_target_speed"] >= 0
    if s["safety_paused"] or s["guard_active"]:
        assert same(s["target_speed"], 0), "latched target must be zero"
        if s["safety_paused"]:
            assert same(s["steering"], 0), "loss pause steering must be zero"
    else:
        assert same(s["target_speed"], s["nominal_target_speed"])
        assert same(s["steering"], s["nominal_steering"])
    if s["resumed"]:
        assert (
            not s["safety_paused"] and not s["guard_active"]
        ), "resume must clear both latches"


def category(e):
    if flag(e["returned"] or "False"):
        if flag(e["censored_return"]):
            return "returned_censored"
        return {"ttl_expiry": "ttl_ended", "age_gate": "age_ended"}[e["end_operation"]]
    if flag(e["preview_loss"]):
        return "no_return"
    return (
        "unobserved_ttl_limit"
        if flag(e["ttl_limit_without_next_step"])
        else "no_observed_loss"
    )


def bounds(e):
    if not flag(e["returned"] or "False"):
        return None
    r = step(e["return_time_s"])
    end = step(e["end_time_s"]) if flag(e["ended"]) else 9000
    assert 7501 <= r <= end <= 9000
    return r - 1, end


def compressed(points, lo, hi):
    assert set(points) >= set(range(lo, hi + 1)), "incomplete control window"
    result = []
    for n in range(lo, hi + 1):
        values = tuple(points[n][k] for k in RUN_FIELDS)
        if result and result[-1][2] == values:
            result[-1] = result[-1][0], n, values
        else:
            result.append((n, n, values))
    return result


def anchor(e, points, ne, nr):
    out = {
        k: None if v == "" else flag(v) if v in ("True", "False") else v
        for k, v in e.items()
    }
    out["anchor_category"] = category(e)
    parent = int(e["parent_rp_records"])
    for k in ("parent_rp_records", "rp_records", "rp_different_preview_records"):
        out[k] = int(e[k])
    out.update(
        census_case_weight=Fraction(1, ne),
        received_case_weight=Fraction(1, nr) if flag(e["received"]) else Fraction(),
        rp_case_mass=Fraction(out["rp_records"], parent) if parent else Fraction(),
        rp_different_preview_case_mass=(
            Fraction(out["rp_different_preview_records"], parent)
            if parent
            else Fraction()
        ),
    )
    out.update(dict.fromkeys(NEW_FIELDS))
    window = bounds(e)
    if window is None:
        return out, [], []
    lo, hi = window
    r, ended = lo + 1, flag(e["ended"])
    available_end = hi - 1 if ended else hi
    assert not points[lo]["epoch_fusion_available"]
    available = [points[n] for n in range(r, available_end + 1)]
    assert available and all(s["epoch_fusion_available"] for s in available)
    assert not ended or not points[hi]["epoch_fusion_available"]
    runs = []
    common = {k: e[k] for k in (*META, "event_id", "source_event_id")}
    for i, (start, finish, values) in enumerate(compressed(points, lo, hi), 1):
        runs.append(
            dict(
                **common,
                run=i,
                start_time_s=start / 50,
                end_time_s=finish / 50,
                checkpoint_count=finish - start + 1,
                **dict(zip(RUN_FIELDS, values)),
            )
        )
    out.update(
        control_window_checkpoints=hi - lo + 1,
        control_run_count=len(runs),
        within_available_checkpoints=len(available),
    )
    for s in LATCHES:
        out["within_" + s + "_checkpoints"] = sum(
            p["latch_state"] == s for p in available
        )
    for s in POSES:
        out["within_" + s + "_checkpoints"] = sum(
            p["pose_branch"] == s for p in available
        )
    for k in (
        "resumed",
        "guard_triggered",
        "guard_released",
        "target_positive",
        "nominal_positive",
        "sample_ready",
    ):
        out["within_" + k + "_checkpoints"] = sum(s[k] for s in available)
    out.update(
        any_resume=any(s["resumed"] for s in available),
        any_positive_target=any(s["target_positive"] for s in available),
        any_positive_nominal=any(s["nominal_positive"] for s in available),
        any_sample_ready=any(s["sample_ready"] for s in available),
        all_available_loss_paused=all(s["safety_paused"] for s in available),
        all_available_guarded=all(s["guard_active"] for s in available),
        all_available_zero_target=all(s["target_speed"] == 0 for s in available),
        max_available_recovery_evidence_s=max(
            s["recovery_evidence"] for s in available
        ),
        min_available_recovery_samples=min(
            s["recovery_sample_count"] for s in available
        ),
        max_available_recovery_samples=max(
            s["recovery_sample_count"] for s in available
        ),
    )
    for prefix, n in (("return_", r), ("boundary_", hi)):
        out.update({prefix + k: points[n][k] for k in DESCRIBE})
    roles = [("return_before", lo), ("return", r)]
    if ended:
        out.update(
            end_pose_changed=points[hi - 1]["pose_branch"] != points[hi]["pose_branch"],
            end_latch_changed=points[hi - 1]["latch_state"]
            != points[hi]["latch_state"],
            end_target_positive_changed=points[hi - 1]["target_positive"]
            != points[hi]["target_positive"],
        )
        roles += [("end_before", hi - 1), ("end_after", hi)]
    else:
        roles.append(("censored_endpoint", hi))
    snapshots = [
        dict(**common, role=role, offset_from_return_ms=(n - r) * 20, **points[n])
        for role, n in roles
    ]
    return out, runs, snapshots


def collect(cases, events, raw, roots):
    files, needed, by_case, event_at = (
        defaultdict(dict),
        defaultdict(set),
        defaultdict(list),
        {},
    )
    for c in cases:
        files[
            (c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv")
        ][(c["scenario"], c["planner"])] = c
    for e in events:
        cid = e["case_id"]
        by_case[cid].append(e)
        key = cid, step(e["event_time_s"])
        assert key not in event_at
        event_at[key] = e
        w = bounds(e)
        if w:
            needed[cid].update(range(w[0], w[1] + 1))
    assert set(files) == {
        (cohort, name) for cohort, s in raw.items() for name in s["raw_sha256"]
    }
    anchors, runs, snapshots, audit = [], [], [], Counter()
    for i, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert digest(path) == raw[cohort]["raw_sha256"][name], name
        audit["raw_bytes"] += path.stat().st_size
        late, last = defaultdict(dict), {}
        with path.open(newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
            assert len(header) == len(set(header))
            pos = {k: header.index(k) for k in ("scenario", "planner", *RAW_FIELDS)}
            for values in reader:
                assert len(values) == len(header)
                identity = values[pos["scenario"]], values[pos["planner"]]
                assert identity in members, "unknown trajectory"
                c = members[identity]
                n = step(values[pos["time_s"]])
                assert (
                    n == last.get(identity, -1) + 1
                ), "duplicate, missing or unordered checkpoint"
                last[identity] = n
                if not flag(c["selected_timeout"]):
                    continue
                audit["timeout_grid_rows"] += 1
                if n < 7500:
                    continue
                s = logged({k: values[v] for k, v in pos.items() if k in RAW_FIELDS})
                command_check(s)
                audit["late_command_decisions"] += 1
                cid = c["case_id"]
                if (cid, n) in event_at:
                    e = event_at[(cid, n)]
                    for a, b in (
                        ("correction", "scheduled"),
                        ("correction_received", "received"),
                        ("precision_activation", "precision_activation"),
                    ):
                        assert s[a] == flag(e[b])
                    assert (
                        s["pose_branch"] == e["immediate_actual"]
                        and s["preview_branch"] == e["immediate_preview"]
                    )
                    audit["census_joins"] += 1
                if n in needed[cid]:
                    late[cid][n] = s
        assert set(last) == set(members)
        for identity, c in members.items():
            assert last[identity] == (
                9000 if flag(c["selected_timeout"]) else step(c["time_s"])
            )
            cid = c["case_id"]
            assert set(late[cid]) == needed[cid]
            ee = by_case[cid]
            assert all(all(e[k] == c[k] for k in META) for e in ee)
            for e in ee:
                a, rr, ss = anchor(
                    e, late[cid], len(ee), sum(flag(e["received"]) for e in ee)
                )
                anchors.append(a)
                runs.extend(rr)
                snapshots.extend(ss)
        audit["raw_files"] += 1
        if i % 12 == 0:
            print(f"Original controls read: {i}/108 files", flush=True)
    anchors.sort(key=lambda r: int(r["event_id"]))
    runs.sort(key=lambda r: (int(r["event_id"]), r["run"]))
    snapshots.sort(key=lambda r: (int(r["event_id"]), r["role"]))
    return anchors, runs, snapshots, dict(audit)


def summarize(cases, rows, meta):
    mass = lambda rr, k="census_case_weight": sum((r[k] for r in rr), Fraction())
    returned = [r for r in rows if r["returned"]]
    out = dict(
        **meta,
        planned_cases=len(cases),
        timeout_cases=sum(flag(c["selected_timeout"]) for c in cases),
        held_cases=sum(flag(c["held"]) for c in cases),
        census_cases=len({r["case_id"] for r in rows}),
        returned_cases=len({r["case_id"] for r in returned}),
        census_events=len(rows),
        received_events=sum(r["received"] for r in rows),
        census_case_mass=mass(rows),
        received_case_mass=mass(rows, "received_case_weight"),
        rp_records=sum(r["rp_records"] for r in rows),
        rp_different_preview_records=sum(
            r["rp_different_preview_records"] for r in rows
        ),
        rp_case_mass=mass(rows, "rp_case_mass"),
        rp_different_preview_case_mass=mass(rows, "rp_different_preview_case_mass"),
    )
    subsets = {k: [r for r in rows if r[k]] for k in SUMMARY_FLAGS}
    subsets.update(
        {
            "return_latch_" + s: [r for r in returned if r["return_latch_state"] == s]
            for s in LATCHES
        }
    )
    subsets.update(
        {
            "return_pose_" + s: [r for r in returned if r["return_pose_branch"] == s]
            for s in POSES
        }
    )
    for k, selected in subsets.items():
        out[k + "_events"], out[k + "_case_mass"] = len(selected), mass(selected)
    for k in (*WITHIN, "control_window_checkpoints", "control_run_count"):
        out[k] = sum(r[k] or 0 for r in rows)
    for k in ("any_resume", "any_positive_target"):
        out[k + "_fraction"] = (
            Fraction(len(subsets[k]), len(returned)) if returned else None
        )
        out[k + "_case_mass_fraction"] = (
            mass(subsets[k]) / mass(returned) if returned else None
        )
    return out


def aggregate(cases, rows):
    result = []
    for p, n, a, m, category_name in product(
        ("known", "new", "fresh"),
        ("white", "correlated"),
        ("epoch", "curved"),
        ("none", "random", "blackout", "all_modes"),
        ("all", *CATEGORIES),
    ):
        accept = lambda r: (r["partition"], r["noise"], r["variant"]) == (p, n, a) and (
            m == "all_modes" or r["mode"] == m
        )
        cc = [c for c in cases if accept(c)]
        rr = [
            r
            for r in rows
            if accept(r)
            and (category_name == "all" or r["anchor_category"] == category_name)
        ]
        result.append(summarize(cc, rr, dict(zip(GROUP, (p, n, a, m, category_name)))))
    return result


def closure(rows):
    lookup = indexed(rows, GROUP)
    additive = [
        k
        for k in rows[0]
        if k.endswith(("_events", "_records", "_case_mass", "_checkpoints"))
        or k == "control_run_count"
    ]
    checks = 0
    for r in rows:
        for field, whole, parts in (
            ("mode", "all_modes", ("none", "random", "blackout")),
            ("category_selector", "all", CATEGORIES),
        ):
            if r[field] == whole:
                children = [
                    lookup[tuple(v if k == field else r[k] for k in GROUP)]
                    for v in parts
                ]
                for k in additive:
                    assert sum(c[k] for c in children) == r[k], (field, k)
                    checks += 1
        for prefix, kinds in (("return_latch_", LATCHES), ("return_pose_", POSES)):
            for suffix in ("_events", "_case_mass"):
                assert (
                    sum(r[prefix + s + suffix] for s in kinds) == r["returned" + suffix]
                )
        assert (
            sum(r["within_" + s + "_checkpoints"] for s in LATCHES)
            == r["within_available_checkpoints"]
        )
        assert (
            sum(r["within_" + s + "_checkpoints"] for s in POSES)
            == r["within_available_checkpoints"]
        )
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--raw-roots", type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    assert (
        digest(BASE / "sources.json") == BASE_SHA
        and digest(BASE / "independent-verification.json") == PROOF_SHA
    )
    prior = json.loads((BASE / "sources.json").read_text())
    pins = {
        **prior["input_sha256"],
        "return-spell-mechanisms/sources.json": BASE_SHA,
        "return-spell-mechanisms/independent-verification.json": PROOF_SHA,
        "return-spell-mechanisms/protocol.md": prior["analysis"]["protocol_sha256"],
        **{
            "return-spell-mechanisms/" + k: h for k, h in prior["output_sha256"].items()
        },
    }
    helpers = {
        **prior["analysis"]["helpers_sha256"],
        "analyze_return_spell.py": prior["analysis"]["script_sha256"],
    }
    for name, h in pins.items():
        assert digest(RESULTS / name) == h, name
    for name, h in {**helpers, **VERIFY_HELPERS}.items():
        assert digest(ROOT / name) == h, name
    for name, h in prior["analysis"]["code_sha256"].items():
        assert digest(REPO / name) == h, name
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(prior["raw"])
    cases = read_rows(RESULTS / "census-rp-selection/case-selection.csv")
    events = read_rows(BASE / "return-endpoints.csv")
    indexed(cases, KEY)
    indexed(events, ("event_id",))
    anchors, runs, snapshots, audit = collect(cases, events, prior["raw"], roots)
    summary = aggregate(cases, anchors)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=570,
        census_events=len(anchors),
        returned_anchors=sum(bool(r["returned"]) for r in anchors),
        control_runs=len(runs),
        window_checkpoint_references=sum(
            r["control_window_checkpoints"] or 0 for r in anchors
        ),
        available_checkpoint_references=sum(
            r["within_available_checkpoints"] or 0 for r in anchors
        ),
        snapshots=len(snapshots),
        summary_rows=len(summary),
        additive_closure_comparisons=closure(summary),
    )
    assert (
        audit["raw_files"],
        audit["timeout_grid_rows"],
        audit["late_command_decisions"],
        audit["census_joins"],
    ) == (108, 5130570, 855570, 3997)
    assert (
        len(cases),
        audit["planned_families"],
        len(anchors),
        audit["returned_anchors"],
        len(snapshots),
        len(summary),
    ) == (972, 324, 3997, 137, 500, 336)
    assert (
        sum(r["census_case_weight"] for r in anchors)
        == sum(r["received_case_weight"] for r in anchors)
        == 570
    )
    assert (
        sum(r["rp_records"] for r in anchors) == 10995
        and sum(r["rp_case_mass"] for r in anchors) == 157
    )
    args.output.mkdir(parents=True)
    for name, rows in (
        ("anchor-controls", anchors),
        ("control-runs", runs),
        ("control-snapshots", snapshots),
        ("control-summary", summary),
    ):
        write(args.output / (name + ".csv"), rows)
    source = dict(
        input_sha256=pins,
        raw=prior["raw"],
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=helpers,
            verifier_helpers_sha256=VERIFY_HELPERS,
            code_sha256=prior["analysis"]["code_sha256"],
        ),
        schema=dict(
            step_ms=20,
            log_end_s=180,
            categories=CATEGORIES,
            latch_states=LATCHES,
            source_identifiers="recovery pose branch and preview branch; neither is a control latch or a resumption",
            within_spell="return through last available checkpoint, inclusive; no future span for 180 s returns",
            weighting="original census, received census and RP parents before selection; checkpoint references separate",
            command_validation="logged overwrite invariants only; no full eligibility, geometry or controller replay",
            retrospective=True,
        ),
        inherited_validation=prior["inherited_validation"],
        inherited_native_proof_sha256=PROOF_SHA,
        audit=audit,
        output_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
