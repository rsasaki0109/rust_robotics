#!/usr/bin/env python3
"""Independently read original controls and reconstruct their runs with SQL."""

import argparse
from collections import Counter, defaultdict
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path
import sqlite3

import verify_epoch_retention as core

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
BASE_SHA = "e251852bc1bc8c1f2f1f974acaeaaf5660fb0be8b15f16eabd95b3fd3bdf914b"
PROOF_SHA = "b260b4fe2654a5026ca862b827e96cddcfc290fc85e3b09cfb1e6994fcef3be8"
HELPERS = {
    "verify_epoch_retention.py": "417a3e93cc4680e77722fe2c95e24f206d532399bd4e2efc8d9951a24530e9f1"
}
META, KEY = core.META, core.KEY
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
BOOLS = (
    "safety_paused guard_active resumed guard_triggered guard_released stop_latched speed_braking correction correction_received "
    "precision_activation epoch_fusion_available stationary_epoch_fusion recovery_preview_active"
).split()
NUMBERS = (
    "time_s measured_speed observation_age target_speed nominal_target_speed steering nominal_steering recovery_evidence "
    "guard_release_evidence release_preview_clearance_lower_bound guard_release_margin"
).split()
COUNTS = (
    "recovery_sample_count motion_filter_sample_count epoch_fusion_group_count"
).split()
DESCRIBE = (
    "pose_branch preview_branch latch_state resumed target_positive nominal_positive target_speed nominal_target_speed "
    "recovery_sample_count sample_ready recovery_evidence safety_paused guard_active"
).split()
FLAGS = (
    "preview_loss returned ended censored_return no_return any_resume any_positive_target any_positive_nominal any_sample_ready "
    "all_available_loss_paused all_available_guarded all_available_zero_target end_pose_changed end_latch_changed end_target_positive_changed"
).split()
WITHIN = (
    "within_available_checkpoints",
    *("within_" + s + "_checkpoints" for s in (*LATCHES, *POSES)),
    *(
        "within_" + s + "_checkpoints"
        for s in (
            "resumed",
            "guard_triggered",
            "guard_released",
            "target_positive",
            "nominal_positive",
            "sample_ready",
        )
    ),
)


def decode(cells):
    r = {k: core.truth(cells[k], True) for k in BOOLS}
    r.update({k: float(cells[k]) for k in NUMBERS})
    assert all(math.isfinite(r[k]) for k in NUMBERS)
    for k in COUNTS:
        r[k] = int(cells[k])
        assert str(r[k]) == cells[k] and r[k] >= 0
    assert (
        0 <= r["recovery_sample_count"] <= 25
        and 0 <= r["epoch_fusion_group_count"] <= 3
    )
    assert r["observation_age"] >= 0
    assert r["stationary_epoch_fusion"] and r["recovery_preview_active"]
    assert not r["epoch_fusion_available"] or r["epoch_fusion_group_count"] == 3
    r["pose_branch"] = POSES[0] if r["recovery_sample_count"] > 0 else POSES[1]
    r["preview_branch"] = (
        "three_epoch" if r["epoch_fusion_available"] else "motion_filter"
    )
    if r["safety_paused"]:
        r["latch_state"] = "loss_and_guard" if r["guard_active"] else "loss_only"
    else:
        r["latch_state"] = "guard_only" if r["guard_active"] else "clear"
    r["target_positive"] = 0 < r["target_speed"]
    r["nominal_positive"] = 0 < r["nominal_target_speed"]
    r["sample_ready"] = not r["recovery_sample_count"] < 10
    assert r["target_speed"] >= 0 and r["nominal_target_speed"] >= 0
    expected = {
        "target_speed": r["nominal_target_speed"],
        "steering": r["nominal_steering"],
    }
    if r["safety_paused"] or r["guard_active"]:
        expected["target_speed"] = 0
    if r["safety_paused"]:
        expected["steering"] = 0
    for k, value in expected.items():
        if k == "steering" and r["guard_active"] and not r["safety_paused"]:
            continue
        assert math.isclose(r[k], value, rel_tol=0, abs_tol=2e-9), (k, r[k], value)
    assert not r["resumed"] or not (r["guard_active"] or r["safety_paused"])
    return r


def classify(e):
    if core.truth(e["returned"] or "False"):
        return (
            "returned_censored"
            if core.truth(e["censored_return"])
            else {"ttl_expiry": "ttl_ended", "age_gate": "age_ended"}[
                e["end_operation"]
            ]
        )
    if core.truth(e["preview_loss"]):
        return "no_return"
    return (
        "unobserved_ttl_limit"
        if core.truth(e["ttl_limit_without_next_step"])
        else "no_observed_loss"
    )


def read_original(db, cases, fixed, original, raw, roots):
    db.executescript("""
      CREATE TABLE anchors(eid TEXT PRIMARY KEY,cid TEXT,lo INT,hi INT);
      CREATE TABLE points(cid TEXT,n INT,pose TEXT,preview TEXT,paused INT,guarded INT,resumed INT,positive INT,PRIMARY KEY(cid,n));
    """)
    files, needed, event_at = defaultdict(dict), defaultdict(set), {}
    for c in cases:
        files[
            (c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv")
        ][(c["scenario"], c["planner"])] = c
    for (eid,), e in fixed.items():
        cid, origin = e["case_id"], core.tick(e["event_time_s"])
        assert (cid, origin) not in event_at
        event_at[cid, origin] = original[eid]
        if core.truth(e["returned"] or "False"):
            start = core.tick(e["return_time_s"])
            end = core.tick(e["end_time_s"]) if core.truth(e["ended"]) else 9000
            needed[cid].update(range(start - 1, end + 1))
            db.execute(
                "INSERT INTO anchors VALUES(?,?,?,?)", (eid, cid, start - 1, end)
            )
    assert set(files) == {(p, n) for p, s in raw.items() for n in s["raw_sha256"]}
    points, audit = defaultdict(dict), Counter()
    for i, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert core.sha(path) == raw[cohort]["raw_sha256"][name], name
        audit["raw_bytes"] += path.stat().st_size
        last = {k: -1 for k in members}
        # The pinned Rust writer produces unquoted scalar fields. Reject other dialects.
        with path.open() as f:
            header = f.readline().rstrip("\n").split(",")
            assert len(header) == len(set(header)) and not any('"' in k for k in header)
            pos = {
                k: header.index(k)
                for k in ("scenario", "planner", *BOOLS, *NUMBERS, *COUNTS)
            }
            for line in f:
                assert '"' not in line
                cells = line.rstrip("\n").split(",")
                assert len(cells) == len(header)
                key = cells[pos["scenario"]], cells[pos["planner"]]
                assert key in members
                c = members[key]
                n = core.tick(cells[pos["time_s"]])
                assert last[key] + 1 == n, "duplicate, missing or unordered checkpoint"
                last[key] = n
                if not core.truth(c["selected_timeout"]):
                    continue
                audit["timeout_grid_rows"] += 1
                if n < 7500:
                    continue
                s = decode({k: cells[index] for k, index in pos.items()})
                audit["late_command_decisions"] += 1
                cid = c["case_id"]
                if (cid, n) in event_at:
                    e = event_at[cid, n]
                    assert (
                        s["pose_branch"] == e["immediate_actual"]
                        and s["preview_branch"] == e["immediate_preview"]
                    )
                    assert (
                        s["correction"],
                        s["correction_received"],
                        s["precision_activation"],
                    ) == tuple(
                        core.truth(e[k])
                        for k in ("scheduled", "received", "precision_activation")
                    )
                    audit["census_joins"] += 1
                if n in needed[cid]:
                    assert n not in points[cid]
                    points[cid][n] = s
                    db.execute(
                        "INSERT INTO points VALUES(?,?,?,?,?,?,?,?)",
                        (
                            cid,
                            n,
                            s["pose_branch"],
                            s["preview_branch"],
                            s["safety_paused"],
                            s["guard_active"],
                            s["resumed"],
                            s["target_positive"],
                        ),
                    )
        for key, c in members.items():
            assert last[key] == (
                9000 if core.truth(c["selected_timeout"]) else core.tick(c["time_s"])
            )
        audit["raw_files"] += 1
        if i % 12 == 0:
            print(f"Independent controls read: {i}/108 files", flush=True)
    for cid, ns in needed.items():
        assert set(points[cid]) == ns
    return points, dict(audit)


def sql_runs(db, fixed):
    rows = db.execute("""
      WITH reference AS (
        SELECT a.eid,p.* FROM anchors a JOIN points p ON a.cid=p.cid AND p.n BETWEEN a.lo AND a.hi
      ), lagged AS (
        SELECT *,CASE WHEN
          pose=LAG(pose) OVER w AND preview=LAG(preview) OVER w AND
          paused=LAG(paused) OVER w AND guarded=LAG(guarded) OVER w AND
          resumed=LAG(resumed) OVER w AND positive=LAG(positive) OVER w THEN 0 ELSE 1 END AS changed
        FROM reference WINDOW w AS (PARTITION BY eid ORDER BY n)
      ), numbered AS (
        SELECT *,SUM(changed) OVER(PARTITION BY eid ORDER BY n) AS ordinal FROM lagged
      )
      SELECT eid,ordinal,MIN(n),MAX(n),COUNT(*),pose,preview,paused,guarded,resumed,positive
      FROM numbered GROUP BY eid,ordinal ORDER BY CAST(eid AS INT),ordinal
    """).fetchall()
    result = {}
    for (
        eid,
        ordinal,
        lo,
        hi,
        count,
        pose,
        preview,
        paused,
        guarded,
        resumed,
        positive,
    ) in rows:
        assert count == hi - lo + 1
        e = fixed[(eid,)]
        result[eid, str(ordinal)] = dict(
            **{k: e[k] for k in (*META, "event_id", "source_event_id")},
            run=ordinal,
            start_time_s=lo / 50,
            end_time_s=hi / 50,
            checkpoint_count=count,
            pose_branch=pose,
            preview_branch=preview,
            safety_paused=bool(paused),
            guard_active=bool(guarded),
            resumed=bool(resumed),
            target_positive=bool(positive),
        )
    return result


def reconstruct(e, pp, ne, nr, runs):
    out = {
        k: None if v == "" else v == "True" if v in ("True", "False") else v
        for k, v in e.items()
    }
    out["anchor_category"] = classify(e)
    parent = int(e["parent_rp_records"])
    for k in ("parent_rp_records", "rp_records", "rp_different_preview_records"):
        out[k] = int(e[k])
    out["census_case_weight"], out["received_case_weight"] = Fraction(1, ne), (
        Fraction(1, nr) if core.truth(e["received"]) else Fraction()
    )
    for k, v in (
        ("rp_case_mass", "rp_records"),
        ("rp_different_preview_case_mass", "rp_different_preview_records"),
    ):
        out[k] = Fraction(out[v], parent) if parent else Fraction()
    blank = [
        "control_window_checkpoints",
        "control_run_count",
        *WITHIN,
        *FLAGS[5:],
        "max_available_recovery_evidence_s",
        "min_available_recovery_samples",
        "max_available_recovery_samples",
        *(prefix + k for prefix in ("return_", "boundary_") for k in DESCRIBE),
    ]
    out.update(dict.fromkeys(blank))
    if not out["returned"]:
        return out, {}
    start = core.tick(e["return_time_s"])
    ended = out["ended"]
    boundary = core.tick(e["end_time_s"]) if ended else 9000
    available_steps = list(range(start, boundary + (0 if ended else 1)))
    available = [pp[n] for n in available_steps]
    assert available and all(s["epoch_fusion_available"] for s in available)
    assert not pp[start - 1]["epoch_fusion_available"]
    assert not ended or not pp[boundary]["epoch_fusion_available"]
    count = Counter()
    for s in available:
        count["within_available_checkpoints"] += 1
        count["within_" + s["pose_branch"] + "_checkpoints"] += 1
        count["within_" + s["latch_state"] + "_checkpoints"] += 1
        for k in (
            "resumed",
            "guard_triggered",
            "guard_released",
            "target_positive",
            "nominal_positive",
            "sample_ready",
        ):
            count["within_" + k + "_checkpoints"] += s[k]
    out.update({k: count[k] for k in WITHIN})
    out.update(
        control_window_checkpoints=boundary - start + 2,
        control_run_count=len(runs),
        any_resume=count["within_resumed_checkpoints"] > 0,
        any_positive_target=count["within_target_positive_checkpoints"] > 0,
        any_positive_nominal=count["within_nominal_positive_checkpoints"] > 0,
        any_sample_ready=count["within_sample_ready_checkpoints"] > 0,
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
    for prefix, n in (("return_", start), ("boundary_", boundary)):
        for k in DESCRIBE:
            out[prefix + k] = pp[n][k]
    roles = {"return_before": start - 1, "return": start}
    if ended:
        before, after = pp[boundary - 1], pp[boundary]
        out.update(
            end_pose_changed=before["pose_branch"] != after["pose_branch"],
            end_latch_changed=before["latch_state"] != after["latch_state"],
            end_target_positive_changed=before["target_positive"]
            != after["target_positive"],
        )
        roles.update(end_before=boundary - 1, end_after=boundary)
    else:
        roles["censored_endpoint"] = boundary
    snapshots = {
        (e["event_id"], role): dict(
            **{k: e[k] for k in (*META, "event_id", "source_event_id")},
            role=role,
            offset_from_return_ms=(n - start) * 20,
            **pp[n],
        )
        for role, n in roles.items()
    }
    return out, snapshots


def summaries(cases, anchors):
    result = {}
    for key in product(
        ("known", "new", "fresh"),
        ("white", "correlated"),
        ("epoch", "curved"),
        ("none", "random", "blackout", "all_modes"),
        ("all", *CATEGORIES),
    ):
        p, n, a, m, category = key
        accepts = lambda r: (r["partition"], r["noise"], r["variant"]) == (
            p,
            n,
            a,
        ) and (m == "all_modes" or r["mode"] == m)
        cc = [c for c in cases if accepts(c)]
        rr = [
            r
            for r in anchors.values()
            if accepts(r) and (category == "all" or r["anchor_category"] == category)
        ]
        returned = [r for r in rr if r["returned"]]
        mass = lambda selected, k="census_case_weight": sum(
            (r[k] for r in selected), Fraction()
        )
        out = dict(zip(GROUP, key))
        out.update(
            planned_cases=len(cc),
            timeout_cases=sum(core.truth(c["selected_timeout"]) for c in cc),
            held_cases=sum(core.truth(c["held"]) for c in cc),
            census_cases=len({r["case_id"] for r in rr}),
            returned_cases=len({r["case_id"] for r in returned}),
            census_events=len(rr),
            received_events=sum(r["received"] for r in rr),
            census_case_mass=mass(rr),
            received_case_mass=mass(rr, "received_case_weight"),
            rp_records=sum(r["rp_records"] for r in rr),
            rp_different_preview_records=sum(
                r["rp_different_preview_records"] for r in rr
            ),
            rp_case_mass=mass(rr, "rp_case_mass"),
            rp_different_preview_case_mass=mass(rr, "rp_different_preview_case_mass"),
        )
        selected = {k: [r for r in rr if r[k]] for k in FLAGS}
        for prefix, field, choices in (
            ("return_latch_", "return_latch_state", LATCHES),
            ("return_pose_", "return_pose_branch", POSES),
        ):
            for choice in choices:
                selected[prefix + choice] = [r for r in returned if r[field] == choice]
        for name, subset in selected.items():
            out[name + "_events"], out[name + "_case_mass"] = len(subset), mass(subset)
        for k in (*WITHIN, "control_window_checkpoints", "control_run_count"):
            out[k] = sum(r[k] or 0 for r in rr)
        for name in ("any_resume", "any_positive_target"):
            out[name + "_fraction"] = (
                Fraction(len(selected[name]), len(returned)) if returned else None
            )
            out[name + "_case_mass_fraction"] = (
                mass(selected[name]) / mass(returned) if returned else None
            )
        result[key] = out
    return result


def closures(rows):
    comparisons = 0
    for key, r in rows.items():
        for i, whole, pieces in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", CATEGORIES),
        ):
            if key[i] == whole:
                children = [rows[key[:i] + (p,) + key[i + 1 :]] for p in pieces]
                for name in r:
                    if (
                        name.endswith(
                            ("_events", "_records", "_case_mass", "_checkpoints")
                        )
                        or name == "control_run_count"
                    ):
                        assert sum(c[name] for c in children) == r[name], (key, name)
                        comparisons += 1
        for prefix, choices in (("return_latch_", LATCHES), ("return_pose_", POSES)):
            for suffix in ("_events", "_case_mass"):
                assert (
                    sum(r[prefix + s + suffix] for s in choices)
                    == r["returned" + suffix]
                )
        assert (
            sum(r["within_" + s + "_checkpoints"] for s in LATCHES)
            == r["within_available_checkpoints"]
        )
        assert (
            sum(r["within_" + s + "_checkpoints"] for s in POSES)
            == r["within_available_checkpoints"]
        )
    return comparisons


def compare(path, expected, keys):
    rows = core.index(core.read(path), keys)
    assert rows.keys() == expected.keys(), (path, len(rows), len(expected))
    for key, row in rows.items():
        core.fields(row, expected[key])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("results", "figures", "work", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--raw-roots", type=Path)
    args = parser.parse_args()
    if not __debug__ or args.work.exists() or args.output.exists():
        parser.error("Run without -O, a new work directory and unused output filename")
    base = RESULTS / "return-spell-mechanisms"
    assert (
        core.sha(base / "sources.json") == BASE_SHA
        and core.sha(base / "independent-verification.json") == PROOF_SHA
    )
    prior = json.loads((base / "sources.json").read_text())
    source = json.loads((args.results / "sources.json").read_text())
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        "return-spell-mechanisms/sources.json": BASE_SHA,
        "return-spell-mechanisms/independent-verification.json": PROOF_SHA,
        "return-spell-mechanisms/protocol.md": prior["analysis"]["protocol_sha256"],
        **{
            "return-spell-mechanisms/" + k: h for k, h in prior["output_sha256"].items()
        },
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_return_spell.py": prior["analysis"]["script_sha256"],
    }
    assert (
        source["analysis"]["verifier_helpers_sha256"] == HELPERS
        and source["analysis"]["code_sha256"] == core.CODE
    )
    assert (
        source["raw"] == prior["raw"]
        and source["inherited_validation"] == prior["inherited_validation"]
    )
    assert source["inherited_native_proof_sha256"] == PROOF_SHA
    for name, h in source["input_sha256"].items():
        assert core.sha(RESULTS / name) == h, name
    for name, h in {**source["analysis"]["helpers_sha256"], **HELPERS}.items():
        assert core.sha(ROOT / name) == h, name
    for name, h in core.CODE.items():
        assert core.sha(core.REPO / name) == h, name
    assert (
        core.sha(ROOT / "analyze_return_controls.py")
        == source["analysis"]["script_sha256"]
    )
    assert (
        core.sha(RESULTS / "fusion-return-controls/protocol.md")
        == source["analysis"]["protocol_sha256"]
    )
    assert set(source["output_sha256"]) == {
        "anchor-controls.csv",
        "control-runs.csv",
        "control-snapshots.csv",
        "control-summary.csv",
    }
    for name, h in source["output_sha256"].items():
        assert core.sha(args.results / name) == h, name
    cases, original = core.original_inputs()
    fixed = core.index(core.read(base / "return-endpoints.csv"), ("event_id",))
    assert {k[0] for k in fixed} == set(original)
    for (eid,), e in fixed.items():
        for k in (
            *META,
            "source_event_id",
            "event_time_s",
            "event_kind",
            "scheduled",
            "received",
            "precision_activation",
            "horizon",
            "immediate_actual",
            "immediate_preview",
            "parent_rp_records",
            "rp_records",
            "rp_different_preview_records",
        ):
            core.check(e[k], original[eid][k])
    original_checks = core.comparisons
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    args.work.mkdir(parents=True)
    db = sqlite3.connect(args.work / "controls.sqlite")
    points, audit = read_original(db, cases, fixed, original, source["raw"], roots)
    runs = sql_runs(db, fixed)
    by_event = defaultdict(list)
    for (eid, _), r in runs.items():
        by_event[eid].append(r)
    ne = Counter(e["case_id"] for e in original.values())
    nr = Counter(e["case_id"] for e in original.values() if core.truth(e["received"]))
    anchors, snapshots = {}, {}
    for (eid,), e in fixed.items():
        a, ss = reconstruct(
            e, points[e["case_id"]], ne[e["case_id"]], nr[e["case_id"]], by_event[eid]
        )
        anchors[eid,] = a
        assert not set(ss) & set(snapshots)
        snapshots.update(ss)
    summary = summaries(cases, anchors)
    closure_checks = closures(summary)
    for name, expected, keys in (
        ("anchor-controls", anchors, ("event_id",)),
        ("control-runs", runs, ("event_id", "run")),
        ("control-snapshots", snapshots, ("event_id", "role")),
        ("control-summary", summary, GROUP),
    ):
        compare(args.results / (name + ".csv"), expected, keys)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(bool(r["returned"]) for r in anchors.values()),
        control_runs=len(runs),
        window_checkpoint_references=sum(
            r["control_window_checkpoints"] or 0 for r in anchors.values()
        ),
        available_checkpoint_references=sum(
            r["within_available_checkpoints"] or 0 for r in anchors.values()
        ),
        snapshots=len(snapshots),
        summary_rows=len(summary),
        additive_closure_comparisons=closure_checks,
    )
    assert audit == source["audit"], (audit, source["audit"])
    assert (
        sum(r["census_case_weight"] for r in anchors.values())
        == sum(r["received_case_weight"] for r in anchors.values())
        == 570
    )
    assert (
        sum(r["rp_case_mass"] for r in anchors.values()) == 157
        and sum(r["rp_records"] for r in anchors.values()) == 10995
    )
    plots = json.loads((args.figures / "plot-sources.json").read_text())
    assert plots["script_sha256"] == core.sha(ROOT / "plot_return_controls.py")
    assert plots["input_sha256"] == {
        n: core.sha(args.results / n)
        for n in ("sources.json", "anchor-controls.csv", "control-summary.csv")
    }
    assert set(plots["output_sha256"]) == {
        "return-latches.svg",
        "plot-values.csv",
        "plot-return-values.csv",
    }
    for n, h in plots["output_sha256"].items():
        assert core.sha(args.figures / n) == h, n
    selected_summary = {k: r for k, r in summary.items() if r["mode"] != "all_modes"}
    selected_returns = {k: r for k, r in anchors.items() if r["returned"]}
    assert len(selected_summary) == 252 and len(selected_returns) == 137
    compare(args.figures / "plot-values.csv", selected_summary, GROUP)
    compare(args.figures / "plot-return-values.csv", selected_returns, ("event_id",))
    evidence = dict(
        **audit,
        comparisons=core.comparisons - original_checks,
        original_join_comparisons=original_checks,
        plotted_summary_rows=252,
        plotted_return_rows=137,
        sqlite_version=sqlite3.sqlite_version,
        inherited_native_proof_sha256=PROOF_SHA,
        script_sha256=core.sha(Path(__file__)),
        sources_sha256=core.sha(args.results / "sources.json"),
        plot_sources_sha256=core.sha(args.figures / "plot-sources.json"),
        method="fresh independent unquoted-scalar raw reader; SQL window functions on uncompressed controls; no analyzer imports or new native metadata replay",
    )
    db.commit()
    db.close()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
