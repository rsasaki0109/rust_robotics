#!/usr/bin/env python3
"""Partition envelope coverage and measure support on explicit audited chains."""

import argparse
from collections import defaultdict
from itertools import product
import json
import math
from pathlib import Path
import statistics

from analyze_commanded_recovery_arc import flag
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import KEY
from analyze_speed_probe_envelopes import RADII, YAWS, decision
from analyze_speed_probe_starts import ESTIMATES, KINDS
from analyze_terminal_arc_live import COHORTS, SIMULATOR
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/speed-probe-envelopes"
PAIRS = ROOT / "results/speed-probe-starts/pairs.csv"
PROTOCOL = ROOT / "results/speed-probe-coverage-spans/protocol.md"
DT = 0.02
COVERAGES = ["covered", "position_only_outside", "heading_only_outside", "both_outside"]
FAMILIES = dict(coverage=COVERAGES, clear_conflict=["conflict", "no_conflict"])
PINNED = {
    "accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "events.csv": "bc7a40a8da19e573d443c09fec79d4cb5fa1023de74b0bfbcfad2d4d40b07274",
    "poses.csv": "966d626f68fe514c7b0e64902a8b894201a624353e2afe78808890080108a052",
    "loss-decisions.csv": "c8c60d289c2700984a062dcde981ac448479f362b8c15b214c09e2717619475f",
    "summary.csv": "c71ab0136d28fb88faa210548239a631a30921c82e3cac1e3c7cb67d85a3ad0b",
    "sources.json": "44b88b02beee2f8682edba2087297be2835ea8d3f6a029ac706e87673b2aa77b",
}
PAIR_SHA = "15408caa093f53379fa3313ec3e087789399484eacef8159f7efdb91f06a1778"


def step(time):
    index = round(float(time) / DT)
    assert abs(float(time) - index * DT) < 1e-7
    return index


def coverage(position, heading):
    return (
        "covered"
        if position and heading
        else (
            "position_only_outside"
            if heading
            else "heading_only_outside" if position else "both_outside"
        )
    )


def evaluate(pose, radius, yaw):
    d = decision(pose, radius, yaw)
    d["coverage"] = coverage(d["position_covered"], d["heading_covered"])
    return d


def runs(labels):
    """Inclusive indices; chain endpoints indicate audit truncation."""
    assert labels
    start = 0
    for end in range(1, len(labels) + 1):
        if end == len(labels) or labels[end] != labels[start]:
            yield start, end - 1, labels[start]
            start = end


def build_chains(nodes, edges):
    assert len(
        {(*[n[k] for k in KEY], step(n["time_s"])) for n in nodes.values()}
    ) == len(nodes)
    incoming, outgoing = {}, {}
    for (pre, post), refs in edges.items():
        a, b = nodes[pre], nodes[post]
        assert refs and pre != post
        assert tuple(a[k] for k in KEY) == tuple(b[k] for k in KEY)
        assert step(b["time_s"]) == step(a["time_s"]) + 1
        assert pre not in outgoing and post not in incoming
        outgoing[pre], incoming[post] = post, pre
    starts = sorted(
        set(nodes) - set(incoming),
        key=lambda sid: (
            *[nodes[sid][k] for k in KEY],
            step(nodes[sid]["time_s"]),
            int(sid),
        ),
    )
    seen, chains, members = set(), [], []
    for start in starts:
        ids, sid = [], start
        while sid is not None:
            assert sid not in seen
            seen.add(sid)
            ids.append(sid)
            sid = outgoing.get(sid)
        cid = len(chains) + 1
        all_events, refs_count = set(), 0
        for i, sid in enumerate(ids):
            n = nodes[sid]
            all_events.update(n["source_event_ids"].split("+"))
            refs = edges[ids[i - 1], sid] if i else []
            refs_count += len(refs)
            members.append(
                dict(
                    chain_id=cid,
                    chain_index=i,
                    state_id=sid,
                    time_s=n["time_s"],
                    **{kind + "_pose_id": n[kind + "_pose_id"] for kind in ESTIMATES},
                    source_event_ids=n["source_event_ids"],
                    incoming_transition_ids="+".join(
                        sorted({r["transition_id"] for r in refs}, key=int)
                    ),
                    incoming_event_ids="+".join(
                        sorted({r["event_id"] for r in refs}, key=int)
                    ),
                )
            )
        chains.append(
            dict(
                chain_id=cid,
                **{k: nodes[start][k] for k in KEY},
                first_state_id=ids[0],
                last_state_id=ids[-1],
                first_time_s=nodes[ids[0]]["time_s"],
                last_time_s=nodes[ids[-1]]["time_s"],
                states=len(ids),
                edges=len(ids) - 1,
                source_pair_refs=refs_count,
                elapsed_s=(len(ids) - 1) * DT,
                source_event_ids="+".join(sorted(all_events, key=int)),
            )
        )
    assert seen == set(nodes), "cycle or unreachable audited node"
    assert sum(c["edges"] for c in chains) == len(edges)
    return chains, members


def make_spans(chains, members, poses):
    grouped = defaultdict(list)
    for m in members:
        grouped[m["chain_id"]].append(m)
    spans, assignments = [], {}
    for chain, kind, radius, yaw in product(chains, ESTIMATES, RADII, YAWS):
        mm = grouped[chain["chain_id"]]
        pp = [poses[m[kind + "_pose_id"]] for m in mm]
        dd = [evaluate(p, radius, yaw) for p in pp]
        labels = dict(
            coverage=[d["coverage"] for d in dd],
            clear_conflict=[
                "conflict" if d["clear_truth_blocks"] else "no_conflict" for d in dd
            ],
        )
        for family in FAMILIES:
            for a, b, label in runs(labels[family]):
                events = {
                    eid
                    for m in mm[a : b + 1]
                    for eid in m["source_event_ids"].split("+")
                }
                r = dict(
                    span_id=len(spans) + 1,
                    chain_id=chain["chain_id"],
                    **{k: chain[k] for k in KEY},
                    pose_kind=kind,
                    radius_m=radius,
                    yaw_bound_deg=yaw,
                    family=family,
                    label=label,
                    first_chain_index=a,
                    last_chain_index=b,
                    first_state_id=mm[a]["state_id"],
                    last_state_id=mm[b]["state_id"],
                    first_pose_id=pp[a]["pose_id"],
                    last_pose_id=pp[b]["pose_id"],
                    first_time_s=mm[a]["time_s"],
                    last_time_s=mm[b]["time_s"],
                    states=b - a + 1,
                    elapsed_s=(b - a) * DT,
                    left_boundary="audit_start" if a == 0 else "label_change",
                    right_boundary="audit_end" if b == len(mm) - 1 else "label_change",
                    source_event_ids="+".join(sorted(events, key=int)),
                )
                assert (
                    abs(
                        float(r["last_time_s"])
                        - float(r["first_time_s"])
                        - r["elapsed_s"]
                    )
                    < 1e-7
                )
                spans.append(r)
                for p in pp[a : b + 1]:
                    key = p["pose_id"], radius, yaw
                    assignments.setdefault(key, {})[family] = r["span_id"]
    assert len(assignments) == len(poses) * len(RADII) * len(YAWS)
    assert all(set(v) == set(FAMILIES) for v in assignments.values())
    for family in FAMILIES:
        assert sum(s["states"] for s in spans if s["family"] == family) == len(
            assignments
        )
    return spans, assignments


def coverage_summary(poses, losses, events, prior_summary):
    groups, opps = defaultdict(list), defaultdict(set)

    def add(p, pop, ids):
        kinds = {"all", *[events[e]["event_kind"] for e in ids]}
        for cohort, kind in product(["all", p["cohort"]], kinds):
            key = cohort, p["variant"], kind, p["pose_kind"], pop
            groups[key].append(p)
            opps[key].update(
                e for e in ids if kind == "all" or events[e]["event_kind"] == kind
            )

    for p in poses.values():
        add(p, "all_states", p["source_event_ids"].split("+"))
    for loss in losses:
        if float(loss["radius_m"]) == 0 and int(loss["yaw_bound_deg"]) == 0:
            p = poses[loss["pose_id"]]
            add(p, "raw_zero_loss", [loss["event_id"]])
            if flag(loss["command_only_zero_loss"]):
                add(p, "command_only_zero_loss", [loss["event_id"]])
    summary = []
    for source in prior_summary:
        key = tuple(
            source[k]
            for k in ["cohort", "variant", "event_kind", "pose_kind", "population"]
        )
        rr = groups[key]
        assert len(rr) == int(source["records"])
        assert len({r["state_id"] for r in rr}) == int(source["states"])
        assert len({tuple(r[k] for k in KEY) for r in rr}) == int(source["cases"])
        assert len(opps[key]) == int(source["opportunities"])
        dd = [
            evaluate(p, float(source["radius_m"]), int(source["yaw_bound_deg"]))
            for p in rr
        ]
        for status in ["clear", "blocked", "unresolved"]:
            assert sum(d["status"] == status for d in dd) == int(source[status])
        for field in [
            "covered",
            "position_covered",
            "heading_covered",
            "clear_truth_blocks",
            "blocked_truth_clear",
        ]:
            assert sum(d[field] for d in dd) == int(source[field])
        counts = {
            f"coverage_{c}": sum(d["coverage"] == c for d in dd) for c in COVERAGES
        }
        assert sum(counts.values()) == len(rr)
        for prefix, field in [
            ("clear_conflict", "clear_truth_blocks"),
            ("blocked_conflict", "blocked_truth_clear"),
        ]:
            for c in COVERAGES:
                counts[f"{prefix}_{c}"] = sum(
                    d[field] and d["coverage"] == c for d in dd
                )
            assert counts[prefix + "_covered"] == 0
        summary.append(
            dict(
                **{
                    k: source[k]
                    for k in [
                        "cohort",
                        "variant",
                        "event_kind",
                        "pose_kind",
                        "population",
                        "radius_m",
                        "yaw_bound_deg",
                        "records",
                        "states",
                        "cases",
                        "opportunities",
                        "clear",
                        "blocked",
                        "unresolved",
                        "position_covered",
                        "heading_covered",
                        "covered",
                        "clear_truth_blocks",
                        "blocked_truth_clear",
                    ]
                },
                **counts,
            )
        )
    return summary


def span_summary(spans, events):
    groups = defaultdict(list)
    for s in spans:
        kinds = {
            "all",
            *[events[e]["event_kind"] for e in s["source_event_ids"].split("+")],
        }
        for cohort, kind in product(["all", s["cohort"]], kinds):
            groups[
                cohort,
                s["variant"],
                kind,
                s["pose_kind"],
                s["radius_m"],
                s["yaw_bound_deg"],
                s["family"],
                s["label"],
            ].append(s)
    summary = []
    for cohort, variant, kind, pose, radius, yaw in product(
        ["all", *COHORTS],
        ["epoch", "curved"],
        ["all", *KINDS],
        ESTIMATES,
        RADII,
        YAWS,
    ):
        for family, labels in FAMILIES.items():
            for label in labels:
                rr = groups[cohort, variant, kind, pose, radius, yaw, family, label]
                ids = {e for r in rr for e in r["source_event_ids"].split("+")}
                durations = [r["elapsed_s"] for r in rr]
                summary.append(
                    dict(
                        cohort=cohort,
                        variant=variant,
                        event_kind=kind,
                        pose_kind=pose,
                        radius_m=radius,
                        yaw_bound_deg=yaw,
                        family=family,
                        label=label,
                        spans=len(rr),
                        states=sum(r["states"] for r in rr),
                        chains=len({r["chain_id"] for r in rr}),
                        cases=len({tuple(r[k] for k in KEY) for r in rr}),
                        opportunities=sum(
                            kind == "all" or events[e]["event_kind"] == kind
                            for e in ids
                        ),
                        singleton_spans=sum(r["states"] == 1 for r in rr),
                        left_audit_truncated=sum(
                            r["left_boundary"] == "audit_start" for r in rr
                        ),
                        right_audit_truncated=sum(
                            r["right_boundary"] == "audit_end" for r in rr
                        ),
                        both_audit_truncated=sum(
                            r["left_boundary"] == "audit_start"
                            and r["right_boundary"] == "audit_end"
                            for r in rr
                        ),
                        median_elapsed_s=(
                            statistics.median(durations) if durations else ""
                        ),
                        max_elapsed_s=max(durations, default=""),
                    )
                )
    return summary


def load_pose(row):
    p = dict(row)
    for field in [
        "position_error",
        "heading_error_rad",
        *[f"{b}_budget_{y}deg" for b, y in product(["clear", "block"], YAWS)],
    ]:
        p[field] = float(p[field])
        assert math.isfinite(p[field])
    p["truth_clear"] = flag(p["truth_clear"])
    return p


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for name, sha in PINNED.items():
        assert digest(PRIOR / name) == sha, name
    assert digest(PAIRS) == PAIR_SHA
    prior = json.loads((PRIOR / "sources.json").read_text())
    helpers = dict(prior["analysis"]["helpers_sha256"])
    helpers["analyze_speed_probe_envelopes.py"] = prior["analysis"]["script_sha256"]
    for name, sha in helpers.items():
        assert digest(ROOT / name) == sha, name
    assert digest(SIMULATOR) == prior["analysis"]["simulator_sha256"]
    assert digest(ROOT / "inputs/obstacles.csv") == prior["map_sha256"]
    accounting, event_rows, pose_rows, losses, old_summary = (
        read_rows(PRIOR / n)
        for n in [
            "accounting.csv",
            "events.csv",
            "poses.csv",
            "loss-decisions.csv",
            "summary.csv",
        ]
    )
    assert (
        len(accounting),
        len(event_rows),
        len(pose_rows),
        len(losses),
        len(old_summary),
    ) == (648, 2583, 19830, 19800, 6480)
    events = {e["event_id"]: e for e in event_rows}
    poses = {p["pose_id"]: load_pose(p) for p in pose_rows}
    nodes = {}
    for p in poses.values():
        sid = p["state_id"]
        base = {k: p[k] for k in [*KEY, "time_s", "source_event_ids"]}
        if sid in nodes:
            assert all(nodes[sid][k] == v for k, v in base.items())
        else:
            nodes[sid] = base
        nodes[sid][p["pose_kind"] + "_pose_id"] = p["pose_id"]
    assert len(nodes) == 6610 and len(poses) == 19830
    edges = defaultdict(list)
    pair_rows = read_rows(PAIRS)
    assert len(pair_rows) == 18561
    pair_ids = defaultdict(set)
    for p in pair_rows:
        pair_ids[p["transition_id"]].add(p["pose_kind"])
        if p["pose_kind"] == "raw":
            edges[p["pre_state_id"], p["post_state_id"]].append(p)
        assert all(
            nodes[p["post_state_id"]][k] == events[p["event_id"]][k] for k in KEY
        )
    assert len(pair_ids) == 6187 and all(v == set(ESTIMATES) for v in pair_ids.values())
    assert sum(map(len, edges.values())) == 6187
    chains, members = build_chains(nodes, edges)
    spans, assignments = make_spans(chains, members, poses)
    member_lookup = {m["state_id"]: m for m in members}
    memberships = []
    for loss in losses:
        p = poses[loss["pose_id"]]
        radius, yaw = float(loss["radius_m"]), int(loss["yaw_bound_deg"])
        d = evaluate(p, radius, yaw)
        for k in [
            "status",
            "covered",
            "position_covered",
            "heading_covered",
            "clear_truth_blocks",
            "blocked_truth_clear",
        ]:
            assert str(d[k]) == loss[k]
        m = member_lookup[p["state_id"]]
        joined = assignments[p["pose_id"], radius, yaw]
        memberships.append(
            dict(
                **loss,
                chain_id=m["chain_id"],
                chain_index=m["chain_index"],
                coverage=d["coverage"],
                coverage_span_id=joined["coverage"],
                conflict_span_id=joined["clear_conflict"],
            )
        )
    cs = coverage_summary(poses, losses, events, old_summary)
    ss = span_summary(spans, events)
    audit = dict(
        accounted_cases=648,
        source_timeout_cases=sum(flag(r["selected"]) for r in accounting),
        opportunities=2583,
        audited_states=len(nodes),
        estimated_poses=len(poses),
        source_pairs=6187,
        unique_physical_edges=len(edges),
        chains=len(chains),
        isolated_chains=sum(c["states"] == 1 for c in chains),
        duplicate_edge_refs=6187 - len(edges),
        pose_settings=len(assignments),
        spans=len(spans),
        coverage_spans=sum(s["family"] == "coverage" for s in spans),
        conflict_family_spans=sum(s["family"] == "clear_conflict" for s in spans),
        loss_memberships=len(memberships),
        raw_zero_loss_pairs=550,
        command_only_zero_loss_pairs=509,
        coverage_summary_rows=len(cs),
        span_summary_rows=len(ss),
    )
    sources = dict(
        input_sha256=PINNED,
        pair_input_sha256=PAIR_SHA,
        map_sha256=prior["map_sha256"],
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
            simulator_sha256=digest(SIMULATOR),
        ),
        schema=dict(
            coverage_categories=COVERAGES,
            families=FAMILIES,
            radii_m=RADII,
            yaw_bounds_deg=YAWS,
            decision_dt_s=DT,
            temporal_edges="explicit audited source pairs only; deduplicated by state IDs",
            span_support="last decision time minus first; singleton zero; audit boundaries truncate",
        ),
        inherited_validation=dict(
            source_sha256=PINNED["sources.json"],
            observation_and_physics=True,
            same_margin_geometry=True,
            new_rollout=False,
        ),
        audit=audit,
    )
    args.output.mkdir(parents=True)
    for name, rr in [
        ("accounting", accounting),
        ("events", event_rows),
        ("chains", chains),
        ("members", members),
        ("spans", spans),
        ("loss-memberships", memberships),
        ("coverage-summary", cs),
        ("span-summary", ss),
    ]:
        save_rows(args.output / f"{name}.csv", rr)
    for n in ["accounting.csv", "events.csv"]:
        assert digest(args.output / n) == PINNED[n]
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
