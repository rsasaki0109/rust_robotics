#!/usr/bin/env python3
"""Audit first returned Fusion spells from pinned full-late native traces."""

import argparse
from collections import Counter, defaultdict
import csv
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path

from analyze_epoch_retention import (
    ALL_OPS,
    KEY,
    META,
    OPERATIONS,
    digest,
    flag,
    indexed,
    numbers,
    operation,
    read_rows,
    step,
)

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
REPO = ROOT.parents[1]
BASE = RESULTS / "fusion-return-followup"
BASE_SHA = "076db5ab572c491d53a88f14493006e830ba75be637bae6a3dd5bd24ebdc3cc7"
PROOF_SHA = "700e730c54999c7aa2ab5297346302106593815413ddb7659bd4bb684b6488b2"
PROTOCOL = RESULTS / "return-spell-mechanisms/protocol.md"
VERIFY_HELPERS = {
    "verify_epoch_retention.py": "417a3e93cc4680e77722fe2c95e24f206d532399bd4e2efc8d9951a24530e9f1",
    "epoch_retention_native.rs": "6ee51bce1a585838bbdae153fba16e18a6537ade38c9da72dadd92e50982a8e6",
    "verify_fusion_return.py": "cfd2b0377e84697cec6e2b58735d135f58ebda876b8013e5ff718cbd6e35fe34",
}
GROUP = ("partition", "noise", "variant", "mode", "operation_selector")
RELATIONS = ("before", "same_checkpoint", "after", "no_next_received_in_log")
FLAGS = (
    "preview_loss",
    "returned",
    "ended",
    "censored_return",
    "no_return",
    "one_checkpoint",
    "return_ttl_limit",
    "end_at_ttl_deadline",
    "end_at_age_deadline",
)
BOOLEAN_INPUTS = ("received", "precision_activation", "moving", "fusion_available")
BASE_STRINGS = (
    *META,
    "event_id",
    "source_event_id",
    "event_kind",
    "horizon",
    "immediate_actual",
    "immediate_preview",
    "loss_operation",
    "return_category",
    "followup_status",
)
NULLABLE = (
    "return_time_s",
    "return_wait_s",
    "return_event_id",
    "returned_spell_span_s",
    "returned_spell_right_censored",
    "ended",
    "censored_return",
    "one_checkpoint",
    "end_time_s",
    "end_operation",
    "return_epoch_steps",
    "return_sample_counts",
    "return_observation_age_s",
    "return_oldest_ttl_ms",
    "return_ttl_limit",
    "conditional_ttl_first_invalid_offset_ms",
    "conditional_age_first_invalid_offset_ms",
    "end_at_ttl_deadline",
    "end_at_age_deadline",
    "end_moving",
    "end_precision_activation",
    "end_ttl_expired",
    "end_capacity_evicted",
    "end_age_blocked",
    "end_latest_epoch_blocked",
    "end_older_samples_blocked",
    "end_group_count_blocked",
    "next_opportunity_event_id",
    "next_opportunity_kind",
    "next_opportunity_time_s",
    "next_opportunity_offset_ms",
    "next_received_event_id",
    "next_received_time_s",
    "next_received_offset_ms",
    "end_vs_next_received",
)


def predicates(s):
    epochs, counts = numbers(s["epoch_steps"]), numbers(s["sample_counts"])
    assert len(epochs) == len(counts) == s["retained_groups"]
    return dict(
        age_blocked=s["observation_age"] > 1.5 + 1e-9,
        group_count_blocked=len(epochs) != 3,
        latest_epoch_blocked=bool(epochs)
        and (
            s["current_epoch_time_s"] is None
            or epochs[-1] != step(s["current_epoch_time_s"])
        ),
        older_samples_blocked=any(c < 10 for c in counts[:-1]),
    )


def age_deadline_ms(age):
    assert math.isfinite(age) and 0 <= age <= 1.5 + 1e-9
    return (math.floor((1.5 + 1e-9 - age) * 50) + 1) * 20


def parse_trace(path, members):
    selected = {c["case_id"] for c in members if flag(c["selected_timeout"])}
    points = {cid: {} for cid in selected}
    with path.open(newline="") as f:
        for r in csv.DictReader(f):
            cid, n = r["case_id"], step(r["time_s"])
            assert cid in points and n not in points[cid]
            assert r["event_id"] == cid and r["role"] == "late_" + str(n)
            s = {k: v for k, v in r.items() if k not in ("case_id", "event_id", "role")}
            for k in BOOLEAN_INPUTS:
                s[k] = flag(s[k], raw=True)
            for k in (
                "time_s",
                "measured_speed",
                "observation_age",
                "current_epoch_time_s",
            ):
                s[k] = float(s[k]) if s[k] else None
                assert s[k] is None or math.isfinite(s[k])
            s["retained_groups"] = int(s["retained_groups"])
            epochs, counts = numbers(s["epoch_steps"]), numbers(s["sample_counts"])
            assert tuple(sorted(set(epochs))) == epochs
            assert all(0 <= n - e <= 750 for e in epochs)
            assert all(1 <= c <= 25 for c in counts)
            gates = predicates(s)
            assert s["moving"] == (abs(s["measured_speed"]) > 0.02)
            assert s["fusion_available"] == (
                not s["moving"] and not any(gates.values())
            )
            points[cid][n] = s
    assert all(set(pp) == set(range(7500, 9001)) for pp in points.values())
    return points


def cached_trace(path, members, expected_sha):
    assert digest(path) == expected_sha, "native trace cache hash mismatch"
    return parse_trace(path, members)


def anchor(e, points, opportunities, ne, nr):
    cid = e["case_id"]
    indexed(opportunities, ("event_id",))
    indexed(opportunities, ("event_time_s",))
    assert all(all(o[k] == e[k] for k in (*KEY, "case_id")) for o in opportunities)
    origin = step(e["event_time_s"])
    assert points[origin]["fusion_available"] == (
        e["immediate_preview"] == "three_epoch"
    )
    assert points[origin]["received"] == flag(e["received"])
    assert points[origin]["precision_activation"] == flag(e["precision_activation"])
    loss = flag(e["preview_loss"])
    returned = flag(e["returned"]) if loss else None
    parent = int(e["parent_rp_records"])
    out = {k: e[k] for k in BASE_STRINGS}
    out.update(
        event_time_s=origin / 50,
        scheduled=flag(e["scheduled"]),
        received=flag(e["received"]),
        precision_activation=flag(e["precision_activation"]),
        preview_loss=loss,
        loss_offset_ms=int(e["loss_offset_ms"]) if loss else None,
        loss_time_s=float(e["loss_time_s"]) if loss else None,
        returned=returned,
        no_return=loss and not returned,
        censor_wait_lower_bound_s=(
            float(e["censor_wait_lower_bound_s"]) if loss and not returned else None
        ),
        ttl_limit_without_next_step=flag(e["ttl_limit_without_next_step"]),
        census_case_weight=Fraction(1, ne),
        received_case_weight=Fraction(1, nr) if flag(e["received"]) else Fraction(),
        parent_rp_records=parent,
        rp_records=int(e["rp_records"]),
        rp_different_preview_records=int(e["rp_different_preview_records"]),
        rp_case_mass=Fraction(int(e["rp_records"]), parent) if parent else Fraction(),
        rp_different_preview_case_mass=(
            Fraction(int(e["rp_different_preview_records"]), parent)
            if parent
            else Fraction()
        ),
    )
    out.update({k: None for k in NULLABLE})
    if not loss:
        return out, [], []
    lost = step(e["loss_time_s"])
    assert lost == origin + int(e["loss_offset_ms"]) // 20
    assert points[lost - 1]["fusion_available"] and not points[lost]["fusion_available"]
    restored = next(
        (n for n in range(lost + 1, 9001) if points[n]["fusion_available"]), None
    )
    assert (restored is not None) == returned
    if restored is None:
        return out, [], []
    assert restored == step(e["first_return_time_s"])
    end = next(
        (n for n in range(restored + 1, 9001) if not points[n]["fusion_available"]),
        None,
    )
    assert flag(e["first_return_spell_right_censored"]) == (end is None)
    assert step(e["first_return_spell_end_time_s"]) == (
        end if end is not None else 9000
    )
    r = points[restored]
    epochs = numbers(r["epoch_steps"])
    ttl = (epochs[0] + 750 - restored) * 20
    age_deadline = age_deadline_ms(r["observation_age"])
    event = next(
        (o for o in opportunities if step(o["event_time_s"]) == restored), None
    )
    out.update(
        return_time_s=restored / 50,
        return_wait_s=(restored - lost) / 50,
        return_event_id=event["event_id"] if event else None,
        returned_spell_span_s=((end if end is not None else 9000) - restored) / 50,
        returned_spell_right_censored=end is None,
        ended=end is not None,
        censored_return=end is None,
        one_checkpoint=end == restored + 1 if end is not None else None,
        return_epoch_steps=r["epoch_steps"],
        return_sample_counts=r["sample_counts"],
        return_observation_age_s=r["observation_age"],
        return_oldest_ttl_ms=ttl,
        return_ttl_limit=ttl == 0,
        conditional_ttl_first_invalid_offset_ms=ttl + 20,
        conditional_age_first_invalid_offset_ms=age_deadline,
    )
    future = sorted(
        (o for o in opportunities if step(o["event_time_s"]) > restored),
        key=lambda o: step(o["event_time_s"]),
    )
    received = next((o for o in future if flag(o["received"])), None)
    if future:
        o = future[0]
        out.update(
            next_opportunity_event_id=o["event_id"],
            next_opportunity_kind=o["event_kind"],
            next_opportunity_time_s=float(o["event_time_s"]),
            next_opportunity_offset_ms=(step(o["event_time_s"]) - restored) * 20,
        )
    if received:
        out.update(
            next_received_event_id=received["event_id"],
            next_received_time_s=float(received["event_time_s"]),
            next_received_offset_ms=(step(received["event_time_s"]) - restored) * 20,
        )
    roles = [("return", restored)]
    if end is not None:
        s = points[end]
        p = predicates(s)
        delta = (end - restored) * 20
        out.update(
            end_time_s=end / 50,
            end_operation=operation(s),
            end_at_ttl_deadline=delta == ttl + 20,
            end_at_age_deadline=delta == age_deadline,
            end_moving=s["moving"],
            end_precision_activation=s["precision_activation"],
            end_ttl_expired=bool(s["expired_epoch_steps"]),
            end_capacity_evicted=bool(s["capacity_evicted_epoch_steps"]),
            **{"end_" + k: v for k, v in p.items()},
            end_vs_next_received=(
                "no_next_received_in_log"
                if received is None
                else (
                    "before"
                    if end < step(received["event_time_s"])
                    else (
                        "same_checkpoint"
                        if end == step(received["event_time_s"])
                        else "after"
                    )
                )
            ),
        )
        roles += [("end_before", end - 1), ("end_after", end)]
    else:
        roles.append(("censored_endpoint", 9000))
    snapshots, groups = [], []
    for role, n in roles:
        s = points[n]
        common = dict(
            **{k: e[k] for k in (*META, "event_id", "source_event_id")}, role=role
        )
        snapshots.append(
            dict(
                **common,
                offset_from_return_ms=(n - restored) * 20,
                **s,
                **predicates(s),
            )
        )
        for i, (epoch, count) in enumerate(
            zip(numbers(s["epoch_steps"]), numbers(s["sample_counts"])), 1
        ):
            groups.append(
                dict(
                    **common,
                    rank=i,
                    epoch_step=epoch,
                    epoch_time_s=epoch / 50,
                    sample_count=count,
                    age_ms=(n - epoch) * 20,
                    ttl_remaining_ms=(epoch + 750 - n) * 20,
                )
            )
    return out, snapshots, groups


def summarize(cases, rows, meta):
    mass = lambda rr, name="census_case_weight": sum((r[name] for r in rr), Fraction())
    ended = [r for r in rows if r["ended"]]
    out = dict(
        **meta,
        planned_cases=len(cases),
        timeout_cases=sum(flag(c["selected_timeout"]) for c in cases),
        held_cases=sum(flag(c["held"]) for c in cases),
        census_cases=len({r["case_id"] for r in rows}),
        ended_cases=len({r["case_id"] for r in ended}),
        census_events=len(rows),
        received_events=sum(r["received"] for r in rows),
        unobserved_ttl_limit_events=sum(r["ttl_limit_without_next_step"] for r in rows),
        census_case_mass=mass(rows),
        received_case_mass=mass(rows, "received_case_weight"),
        rp_records=sum(r["rp_records"] for r in rows),
        rp_different_preview_records=sum(
            r["rp_different_preview_records"] for r in rows
        ),
        rp_case_mass=mass(rows, "rp_case_mass"),
        rp_different_preview_case_mass=mass(rows, "rp_different_preview_case_mass"),
    )
    selections = {name: [r for r in rows if r[name]] for name in FLAGS}
    selections.update(
        {
            op + "_end": [r for r in ended if r["end_operation"] == op]
            for op in OPERATIONS
        }
    )
    selections.update(
        {
            "end_" + rel: [r for r in ended if r["end_vs_next_received"] == rel]
            for rel in RELATIONS
        }
    )
    for name, selected in selections.items():
        out[name + "_events"], out[name + "_case_mass"] = len(selected), mass(selected)
    for op in OPERATIONS:
        rr = selections[op + "_end"]
        out[op + "_end_fraction"] = Fraction(len(rr), len(ended)) if ended else None
        out[op + "_end_case_mass_fraction"] = mass(rr) / mass(ended) if ended else None
    return out


def aggregate(cases, anchors):
    result = []
    for part, noise, arm, mode, op in product(
        ("known", "new", "fresh"),
        ("white", "correlated"),
        ("epoch", "curved"),
        ("none", "random", "blackout", "all_modes"),
        ("all", *ALL_OPS),
    ):
        accept = lambda r: (r["partition"], r["noise"], r["variant"]) == (
            part,
            noise,
            arm,
        ) and (mode == "all_modes" or r["mode"] == mode)
        cc = [c for c in cases if accept(c)]
        rr = [
            r
            for r in anchors
            if accept(r) and (op == "all" or r["loss_operation"] == op)
        ]
        result.append(summarize(cc, rr, dict(zip(GROUP, (part, noise, arm, mode, op)))))
    return result


def closure(rows):
    lookup = indexed(rows, GROUP)
    additive = [k for k in rows[0] if k.endswith(("_events", "_records", "_case_mass"))]
    checks = 0
    for r in rows:
        for field, whole, parts in (
            ("mode", "all_modes", ("none", "random", "blackout")),
            ("operation_selector", "all", ALL_OPS),
        ):
            if r[field] != whole:
                continue
            children = [
                lookup[tuple(p if k == field else r[k] for k in GROUP)] for p in parts
            ]
            for k in additive:
                assert sum(c[k] for c in children) == r[k], (field, k)
                checks += 1
        for suffix in ("_events", "_case_mass"):
            assert (
                r["preview_loss" + suffix]
                == r["returned" + suffix] + r["no_return" + suffix]
            )
            assert (
                r["returned" + suffix]
                == r["ended" + suffix] + r["censored_return" + suffix]
            )
            assert r["ended" + suffix] == sum(
                r[op + "_end" + suffix] for op in OPERATIONS
            )
            assert r["ended" + suffix] == sum(
                r["end_" + rel + suffix] for rel in RELATIONS
            )
    return checks


def write(path, rows):
    assert rows
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0], lineterminator="\n")
        w.writeheader()
        w.writerows(
            {k: float(v) if isinstance(v, Fraction) else v for k, v in r.items()}
            for r in rows
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native-traces", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    assert digest(BASE / "sources.json") == BASE_SHA
    assert digest(BASE / "independent-verification.json") == PROOF_SHA
    prior = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    assert (
        proof["sources_sha256"] == BASE_SHA
        and proof["script_sha256"] == VERIFY_HELPERS["verify_fusion_return.py"]
    )
    pins = {
        **prior["input_sha256"],
        "fusion-return-followup/sources.json": BASE_SHA,
        "fusion-return-followup/independent-verification.json": PROOF_SHA,
        "fusion-return-followup/protocol.md": prior["analysis"]["protocol_sha256"],
        **{"fusion-return-followup/" + k: h for k, h in prior["output_sha256"].items()},
    }
    helpers = {
        **prior["analysis"]["helpers_sha256"],
        "analyze_fusion_return.py": prior["analysis"]["script_sha256"],
    }
    for name, h in pins.items():
        assert digest(RESULTS / name) == h, name
    for name, h in {**helpers, **VERIFY_HELPERS}.items():
        assert digest(ROOT / name) == h, name
    for name, h in prior["analysis"]["code_sha256"].items():
        assert digest(REPO / name) == h, name
    cases = read_rows(RESULTS / "census-rp-selection/case-selection.csv")
    events = read_rows(BASE / "loss-followup.csv")
    indexed(cases, KEY)
    indexed(events, ("event_id",))
    files, by_case = defaultdict(list), defaultdict(list)
    for c in cases:
        files[
            c["cohort"] + f"/{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"
        ].append(c)
    for e in events:
        by_case[e["case_id"]].append(e)
    assert set(files) == set(proof["native_trace_sha256"])
    anchors, snapshots, groups = [], [], []
    checkpoint_count, cached_bytes = 0, 0
    for i, name in enumerate(sorted(files), 1):
        path = args.native_traces / f"{i:03d}-native.csv"
        cached_bytes += path.stat().st_size
        points = cached_trace(path, files[name], proof["native_trace_sha256"][name])
        checkpoint_count += sum(map(len, points.values()))
        for c in files[name]:
            cid = c["case_id"]
            if cid not in points:
                assert not by_case[cid]
                continue
            ee = by_case[cid]
            assert all(all(e[k] == c[k] for k in META) for e in ee)
            ne, nr = len(ee), sum(flag(e["received"]) for e in ee)
            assert ne and nr
            for e in ee:
                a, ss, gg = anchor(e, points[cid], ee, ne, nr)
                anchors.append(a)
                snapshots.extend(ss)
                groups.extend(gg)
        if i % 12 == 0:
            print(f"Pinned native traces audited: {i}/108", flush=True)
    anchors.sort(key=lambda r: int(r["event_id"]))
    snapshots.sort(key=lambda r: (int(r["event_id"]), r["role"]))
    groups.sort(key=lambda r: (int(r["event_id"]), r["role"], r["rank"]))
    summaries = aggregate(cases, anchors)
    closure_checks = closure(summaries)
    assert len(cases) == 972 and len({c["family_id"] for c in cases}) == 324
    assert len(anchors) == 3997 and checkpoint_count == 855570
    assert (
        sum(r["census_case_weight"] for r in anchors)
        == sum(r["received_case_weight"] for r in anchors)
        == 570
    )
    assert (
        sum(r["rp_records"] for r in anchors) == 10995
        and sum(r["rp_case_mass"] for r in anchors) == 157
    )
    audit = dict(
        cached_trace_files=len(files),
        cached_trace_bytes=cached_bytes,
        cached_late_checkpoints=checkpoint_count,
        planned_cases=len(cases),
        planned_families=324,
        timeout_cases=570,
        census_events=len(anchors),
        observed_loss_anchors=sum(r["preview_loss"] for r in anchors),
        returned_anchors=sum(bool(r["returned"]) for r in anchors),
        ended_spells=sum(bool(r["ended"]) for r in anchors),
        censored_returned_spells=sum(bool(r["censored_return"]) for r in anchors),
        no_return_anchors=sum(r["no_return"] for r in anchors),
        unobserved_ttl_limit_anchors=sum(
            r["ttl_limit_without_next_step"] for r in anchors
        ),
        snapshots=len(snapshots),
        group_references=len(groups),
        summary_rows=len(summaries),
        additive_closure_comparisons=closure_checks,
    )
    assert (
        audit["observed_loss_anchors"],
        audit["returned_anchors"],
        audit["ended_spells"],
        audit["censored_returned_spells"],
        audit["no_return_anchors"],
        audit["unobserved_ttl_limit_anchors"],
    ) == (152, 137, 89, 48, 15, 18)
    args.output.mkdir(parents=True)
    for name, rows in (
        ("return-endpoints", anchors),
        ("snapshots", snapshots),
        ("group-history", groups),
        ("spell-summary", summaries),
    ):
        write(args.output / (name + ".csv"), rows)
    source = dict(
        input_sha256=pins,
        raw=prior["raw"],
        native_trace_sha256=proof["native_trace_sha256"],
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=helpers,
            verifier_helpers_sha256=VERIFY_HELPERS,
            code_sha256=prior["analysis"]["code_sha256"],
        ),
        schema=dict(
            step_ms=20,
            late_start_s=150,
            log_end_s=180,
            operations=OPERATIONS,
            original_operation_selectors=("all", *ALL_OPS),
            next_received_relations=RELATIONS,
            collection="pinned preceding full-late native trace cache; no raw reread in collector",
            deadlines="conditional on unchanged oldest epoch/no new receipt; not forecasts",
            weighting="original all-census, received-census and RP parents, before selection",
            metadata_only=True,
            retrospective=True,
        ),
        inherited_validation=prior["inherited_validation"],
        audit=audit,
        output_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
