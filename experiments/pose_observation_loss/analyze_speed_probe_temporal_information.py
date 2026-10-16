#!/usr/bin/env python3
"""Audit agreement persistence and temporal means with shared observations."""

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
from analyze_speed_probe_coverage_spans import DT, runs
from analyze_speed_probe_shared_information import (
    BINS,
    PAIRINGS,
    ar_covariance,
    category,
    model,
)
from analyze_speed_probe_starts import KINDS
from analyze_terminal_arc_live import COHORTS, SIMULATOR
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PROTOCOL = ROOT / "results/speed-probe-temporal-information/protocol.md"
RULES = {"reconstruction": 1e-8, "0p02": 0.02, "0p05": 0.05, "0p10": 0.10}
MODES = ["all", "shared_single", "preview_three"]
PINNED = {
    "speed-probe-shared-information/accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "speed-probe-shared-information/events.csv": "bc7a40a8da19e573d443c09fec79d4cb5fa1023de74b0bfbcfad2d4d40b07274",
    "speed-probe-shared-information/pairs.csv": "f48da3e2e64fe6ad264890137944f5776034408fa7bf359baecae0d0c2c9744b",
    "speed-probe-shared-information/loss-links.csv": "b25ba5f8fe9f6cafbe1bc20a32b318455804adf3dc1639f10d335607fea7222b",
    "speed-probe-shared-information/inputs.csv": "63a674580021cab8153a875d8b5c796236155c3f8e1ee56da2b1497b277abd49",
    "speed-probe-shared-information/anchor-weights.csv": "ab1f112c287a0f2c856586110ba7dfcbb84f137039149ff63155962b7dc9d97a",
    "speed-probe-shared-information/sources.json": "56444ffd8625ce3eabff48ecf6439470234d00e591beb8e33f7f195e55f4a6a0",
    "speed-probe-coverage-spans/chains.csv": "2bdacbd6e17706455901c601a57d4041f8ca2b112d7fac40e88b75753f22ea61",
    "speed-probe-coverage-spans/members.csv": "fe6db79ea0af46dbc0953a8fabb7c27ac58b8c31b2a7766747efb154d9a3fda9",
    "speed-probe-coverage-spans/sources.json": "19bf2b76e72b4e05549902e8414909c1a6e497c58c6118c7ddcc87082a395cd9",
    "speed-probe-error-components/components.csv": "60efb9edbba466a857c25043e60c0d9b851f6d3ff418d5e0f8e999281df991e8",
    "speed-probe-error-components/groups.csv": "c2cf508a23447808a711f7d760c6f32874e58db24acce097e93941088fc01af8",
}
EPS = 1e-12


def mean_coefficients(maps):
    assert maps
    result = defaultdict(float)
    for m in maps:
        for step, coefficient in m.items():
            result[step] += coefficient / len(maps)
    return dict(result)


def noise_maps(components, groups, inputs):
    result = {}
    for pid, p in components.items():
        sigma = float(p["sample_sigma_min"])
        assert sigma == float(p["sample_sigma_max"])
        m = {}
        for g in groups[pid]:
            n = int(g["sample_count"])
            lo, hi = int(g["first_sample_step"]), int(g["last_sample_step"])
            assert hi - lo + 1 == n
            for step in range(lo, hi + 1):
                assert step not in m
                m[step] = float(g["group_weight"]) * sigma / n
        assert len(m) == int(inputs[pid]["sample_count"])
        assert (
            abs(
                ar_covariance(m, m, float(inputs[pid]["rho"]))
                - float(inputs[pid]["variance_noise"])
            )
            < EPS
        )
        result[pid] = m
    return result


def block_metrics(rows, noise, inputs, components, signatures):
    assert rows
    n = len(rows)
    a_ids, b_ids = ([p[side + "_pose_id"] for p in rows] for side in ["a", "b"])
    for ids in [a_ids, b_ids]:
        assert all(signatures[pid] == signatures[ids[0]] for pid in ids)
    aa, ab = (float(inputs[ids[0]]["variance_anchor"]) for ids in [a_ids, b_ids])
    ca = rows[0]["covariance_anchor"]
    rho = rows[0]["rho"]
    for p in rows:
        assert p["rho"] == rho and abs(p["covariance_anchor"] - ca) < EPS
        assert abs(float(inputs[p["a_pose_id"]]["variance_anchor"]) - aa) < EPS
        assert abs(float(inputs[p["b_pose_id"]]["variance_anchor"]) - ab) < EPS
    ma, mb = (mean_coefficients([noise[pid] for pid in ids]) for ids in [a_ids, b_ids])
    na, nb = ar_covariance(ma, ma, rho), ar_covariance(mb, mb, rho)
    cn = ar_covariance(ma, mb, rho)
    joint = model(na, aa, nb, ab, cn, ca)
    output = dict(
        anchor_mode=rows[0]["anchor_mode"],
        a_anchor_signature=json.dumps(signatures[a_ids[0]], separators=(",", ":")),
        b_anchor_signature=json.dumps(signatures[b_ids[0]], separators=(",", ":")),
        a_observation_uses=sum(len(noise[pid]) for pid in a_ids),
        b_observation_uses=sum(len(noise[pid]) for pid in b_ids),
        a_unique_observations=len(ma),
        b_unique_observations=len(mb),
        a_mean_noise_variance=na,
        b_mean_noise_variance=nb,
        a_mean_anchor_variance=aa,
        b_mean_anchor_variance=ab,
        a_mean_variance=na + aa,
        b_mean_variance=nb + ab,
        a_time_independent_variance=sum(p["a_variance_joint"] for p in rows) / n**2,
        b_time_independent_variance=sum(p["b_variance_joint"] for p in rows) / n**2,
        mean_covariance_noise=cn,
        mean_covariance_anchor=ca,
        mean_difference_variance=joint["difference_variance_joint"],
        time_independent_difference_variance=sum(
            p["difference_variance_joint"] for p in rows
        )
        / n**2,
        independent_inputs_and_time_variance=sum(
            p["independent_variance_joint"] for p in rows
        )
        / n**2,
    )
    for side, ids in [("a", a_ids), ("b", b_ids)]:
        x, y = [
            statistics.fmean(float(components[pid]["error_" + axis]) for pid in ids)
            for axis in ["x", "y"]
        ]
        output[side + "_mean_error_x"], output[side + "_mean_error_y"] = x, y
        output[side + "_mean_error_norm"] = math.hypot(x, y)
        assert (
            output[side + "_mean_variance"] + EPS
            >= output[side + "_time_independent_variance"]
        )
        assert (
            output[side + "_mean_variance"]
            <= n * output[side + "_time_independent_variance"] + EPS
        )
    for axis in ["x", "y"]:
        delta = statistics.fmean(p["delta_" + axis] for p in rows)
        assert (
            abs(
                delta
                - (output["a_mean_error_" + axis] - output["b_mean_error_" + axis])
            )
            < 1e-10
        )
        output["mean_delta_" + axis] = delta
    return output


def frame(chain, mm, pp, a, b):
    assert 0 <= a <= b < len(mm)
    assert abs(float(mm[b]["time_s"]) - float(mm[a]["time_s"]) - (b - a) * DT) < 1e-7
    ids = {e for m in mm[a : b + 1] for e in m["source_event_ids"].split("+")}
    return dict(
        chain_id=chain["chain_id"],
        **{k: chain[k] for k in KEY},
        first_chain_index=a,
        last_chain_index=b,
        first_state_id=mm[a]["state_id"],
        last_state_id=mm[b]["state_id"],
        first_pair_id=pp[a]["pair_id"],
        last_pair_id=pp[b]["pair_id"],
        first_a_pose_id=pp[a]["a_pose_id"],
        last_a_pose_id=pp[b]["a_pose_id"],
        first_b_pose_id=pp[a]["b_pose_id"],
        last_b_pose_id=pp[b]["b_pose_id"],
        first_time_s=mm[a]["time_s"],
        last_time_s=mm[b]["time_s"],
        states=b - a + 1,
        elapsed_s=(b - a) * DT,
        source_event_ids="+".join(sorted(ids, key=int)),
    )


def build(chains, members, pairs, noise, inputs, components, signatures):
    cm = defaultdict(list)
    for m in members:
        cm[m["chain_id"]].append(m)
    lookup = {(p["state_id"], p["pairing"]): p for p in pairs}
    spans, blocks, assignments = [], [], {}
    for chain, (kind_a, kind_b) in product(chains, PAIRINGS):
        pairing = kind_a + "_" + kind_b
        mm = cm[chain["chain_id"]]
        assert len(mm) == int(chain["states"])
        assert [int(m["chain_index"]) for m in mm] == list(range(len(mm)))
        for i, m in enumerate(mm):
            if i:
                assert m["incoming_transition_ids"]
                assert abs(float(m["time_s"]) - float(mm[i - 1]["time_s"]) - DT) < 1e-7
            else:
                assert not m["incoming_transition_ids"]
        pp = [lookup[m["state_id"], pairing] for m in mm]
        assert all(
            all(p[k] == chain[k] for k in KEY) and p["chain_id"] == chain["chain_id"]
            for p in pp
        )
        for rule, threshold in RULES.items():
            labels = [category(p, threshold) for p in pp]
            for a, b, label in runs(labels):
                span = dict(
                    span_id=len(spans) + 1,
                    **frame(chain, mm, pp, a, b),
                    pairing=pairing,
                    rule=rule,
                    agreement_threshold_m=threshold,
                    label=label,
                    left_boundary="audit_start" if a == 0 else "category_change",
                    right_boundary=(
                        "audit_end" if b == len(mm) - 1 else "category_change"
                    ),
                )
                spans.append(span)
                sigs = [
                    (signatures[p["a_pose_id"]], signatures[p["b_pose_id"]])
                    for p in pp[a : b + 1]
                ]
                for lo, hi, _ in runs(sigs):
                    first, last = a + lo, a + hi
                    block = dict(
                        block_id=len(blocks) + 1,
                        span_id=span["span_id"],
                        **frame(chain, mm, pp, first, last),
                        pairing=pairing,
                        rule=rule,
                        agreement_threshold_m=threshold,
                        label=label,
                        left_boundary=(
                            span["left_boundary"] if lo == 0 else "signature_change"
                        ),
                        right_boundary=(
                            span["right_boundary"]
                            if hi == b - a
                            else "signature_change"
                        ),
                        **block_metrics(
                            pp[first : last + 1], noise, inputs, components, signatures
                        ),
                    )
                    blocks.append(block)
                    for p in pp[first : last + 1]:
                        key = p["pair_id"], rule
                        assert key not in assignments
                        assignments[key] = (span["span_id"], block["block_id"])
    assert len(assignments) == len(pairs) * len(RULES)
    assert (
        sum(s["states"] for s in spans)
        == sum(b["states"] for b in blocks)
        == len(assignments)
    )
    return spans, blocks, assignments


def summarize(rows, events, is_block):
    groups = defaultdict(list)
    for r in rows:
        kinds = {
            "all",
            *[events[e]["event_kind"] for e in r["source_event_ids"].split("+")],
        }
        for co, kind, mode in product(
            ["all", r["cohort"]],
            kinds,
            ["all", r["anchor_mode"]] if is_block else ["all"],
        ):
            groups[
                co, r["variant"], kind, r["pairing"], r["rule"], r["label"], mode
            ].append(r)
    result = []
    for co, arm, kind, (a, b), rule, label, mode in product(
        ["all", *COHORTS],
        ["epoch", "curved"],
        ["all", *KINDS],
        PAIRINGS,
        RULES,
        BINS,
        MODES if is_block else ["all"],
    ):
        rr = groups[co, arm, kind, a + "_" + b, rule, label, mode]
        ids = {e for r in rr for e in r["source_event_ids"].split("+")}
        item = dict(
            cohort=co,
            variant=arm,
            event_kind=kind,
            pairing=a + "_" + b,
            rule=rule,
            label=label,
            anchor_mode=mode,
            records=len(rr),
            states=sum(r["states"] for r in rr),
            spans=len({r["span_id"] for r in rr}),
            chains=len({r["chain_id"] for r in rr}),
            cases=len({tuple(r[k] for k in KEY) for r in rr}),
            opportunities=sum(
                kind == "all" or events[e]["event_kind"] == kind for e in ids
            ),
            singleton_records=sum(r["states"] == 1 for r in rr),
            left_audit_truncated=sum(r["left_boundary"] == "audit_start" for r in rr),
            right_audit_truncated=sum(r["right_boundary"] == "audit_end" for r in rr),
            any_audit_truncated=sum(
                r["left_boundary"] == "audit_start"
                or r["right_boundary"] == "audit_end"
                for r in rr
            ),
            median_elapsed_s=(
                statistics.median(r["elapsed_s"] for r in rr) if rr else ""
            ),
            max_elapsed_s=max((r["elapsed_s"] for r in rr), default=""),
        )
        if is_block:
            for field in [
                "a_mean_variance",
                "b_mean_variance",
                "a_time_independent_variance",
                "b_time_independent_variance",
                "mean_difference_variance",
                "time_independent_difference_variance",
                "independent_inputs_and_time_variance",
            ]:
                item["pooled_sigma_" + field] = (
                    math.sqrt(statistics.fmean(r[field] for r in rr)) if rr else ""
                )
            for side in ["a", "b"]:
                item["pooled_" + side + "_mean_error"] = (
                    math.sqrt(
                        statistics.fmean(r[side + "_mean_error_norm"] ** 2 for r in rr)
                    )
                    if rr
                    else ""
                )
                for field in ["observation_uses", "unique_observations"]:
                    item["mean_" + side + "_" + field] = (
                        statistics.fmean(r[side + "_" + field] for r in rr)
                        if rr
                        else ""
                    )
        result.append(item)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for n, h in PINNED.items():
        assert digest(ROOT / "results" / n) == h, n
    prior = json.loads(
        (ROOT / "results/speed-probe-shared-information/sources.json").read_text()
    )
    helpers = dict(prior["analysis"]["helpers_sha256"])
    helpers["analyze_speed_probe_shared_information.py"] = prior["analysis"][
        "script_sha256"
    ]
    for n, h in helpers.items():
        assert digest(ROOT / n) == h, n
    assert digest(SIMULATOR) == prior["analysis"]["simulator_sha256"]
    assert digest(ROOT / "inputs/obstacles.csv") == prior["map_sha256"]
    base = ROOT / "results/speed-probe-shared-information"
    accounting, ee, pair_rows, loss_links, input_rows, anchor_rows = (
        read_rows(base / n)
        for n in [
            "accounting.csv",
            "events.csv",
            "pairs.csv",
            "loss-links.csv",
            "inputs.csv",
            "anchor-weights.csv",
        ]
    )
    chains = read_rows(ROOT / "results/speed-probe-coverage-spans/chains.csv")
    members = read_rows(ROOT / "results/speed-probe-coverage-spans/members.csv")
    components = {
        p["pose_id"]: p
        for p in read_rows(ROOT / "results/speed-probe-error-components/components.csv")
    }
    groups = defaultdict(list)
    for g in read_rows(ROOT / "results/speed-probe-error-components/groups.csv"):
        groups[g["pose_id"]].append(g)
    inputs = {p["pose_id"]: p for p in input_rows}
    sigs = defaultdict(list)
    for a in anchor_rows:
        sigs[a["pose_id"]].append((a["anchor_id"], float(a["weight"])))
    signatures = {
        pid: tuple(sorted(sig, key=lambda kv: int(kv[0]))) for pid, sig in sigs.items()
    }
    assert (
        len(accounting),
        len(ee),
        len(chains),
        len(members),
        len(pair_rows),
        len(loss_links),
    ) == (648, 2583, 423, 6610, 19830, 550)
    assert set(inputs) == set(components) == set(signatures)
    noise = noise_maps(components, groups, inputs)
    pairs = []
    for p in pair_rows:
        p = dict(p)
        for field in [
            "position_separation",
            "rho",
            "covariance_anchor",
            "a_variance_joint",
            "b_variance_joint",
            "difference_variance_joint",
            "independent_variance_joint",
            "delta_x",
            "delta_y",
        ]:
            p[field] = float(p[field])
        for field in ["a_inside", "b_inside"]:
            p[field] = flag(p[field])
        pairs.append(p)
    spans, blocks, assignments = build(
        chains, members, pairs, noise, inputs, components, signatures
    )
    pair_lookup = {p["pair_id"]: p for p in pairs}
    memberships = []
    for link, (a, b), rule in product(loss_links, PAIRINGS, RULES):
        pid = link[a + "_" + b + "_pair_id"]
        p = pair_lookup[pid]
        assert p["state_id"] == link["state_id"]
        sid, bid = assignments[pid, rule]
        memberships.append(
            dict(
                transition_id=link["transition_id"],
                event_id=link["event_id"],
                state_id=p["state_id"],
                pair_id=pid,
                pairing=a + "_" + b,
                rule=rule,
                command_only_zero_loss=link["command_only_zero_loss"],
                label=category(p, RULES[rule]),
                span_id=sid,
                block_id=bid,
                chain_id=p["chain_id"],
                chain_index=p["chain_index"],
            )
        )
    assert (
        len(memberships) == 6600
        and sum(flag(l["command_only_zero_loss"]) for l in loss_links) == 509
    )
    events = {e["event_id"]: e for e in ee}
    ss, bs = summarize(spans, events, False), summarize(blocks, events, True)
    audit = dict(
        accounted_cases=648,
        timeout_cases=368,
        opportunities=2583,
        audited_states=6610,
        chains=len(chains),
        pairs=len(pairs),
        pair_rule_decisions=len(assignments),
        spans=len(spans),
        blocks=len(blocks),
        signature_splits=len(blocks) - len(spans),
        loss_memberships=len(memberships),
        raw_zero_loss_links=550,
        command_only_links=509,
        span_summary_rows=len(ss),
        block_summary_rows=len(bs),
    )
    source = dict(
        input_sha256=PINNED,
        map_sha256=prior["map_sha256"],
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
            simulator_sha256=digest(SIMULATOR),
        ),
        schema=dict(
            rules=RULES,
            labels=BINS,
            anchor_modes=MODES,
            dt_s=DT,
            variance_guard_m2=EPS,
            block_weighting="one weight per block; covariance of temporal means preserves shared observations and anchors",
        ),
        inherited_validation=dict(
            source_sha256=PINNED["speed-probe-shared-information/sources.json"],
            observation_and_physics=True,
            new_rollout=False,
        ),
        audit=audit,
    )
    args.output.mkdir(parents=True)
    for n, rr in [
        ("accounting", accounting),
        ("events", ee),
        ("chains", chains),
        ("members", members),
        ("spans", spans),
        ("blocks", blocks),
        ("loss-memberships", memberships),
        ("span-summary", ss),
        ("block-summary", bs),
    ]:
        save_rows(args.output / f"{n}.csv", rr)
    for n, prefix in [
        ("accounting", "speed-probe-shared-information"),
        ("events", "speed-probe-shared-information"),
        ("chains", "speed-probe-coverage-spans"),
        ("members", "speed-probe-coverage-spans"),
    ]:
        assert digest(args.output / f"{n}.csv") == PINNED[f"{prefix}/{n}.csv"]
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
