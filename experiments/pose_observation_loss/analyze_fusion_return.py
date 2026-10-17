#!/usr/bin/env python3
"""Follow frozen Fusion losses through original logs, including censoring."""

import argparse
from collections import Counter, defaultdict
import csv
from fractions import Fraction
from itertools import product
import json
from pathlib import Path

from analyze_epoch_retention import (
    ALL_OPS,
    KEY,
    META,
    Metadata,
    digest,
    flag,
    indexed,
    integer,
    numbers,
    read_rows,
    step,
)
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
RESULTS = ROOT / "results"
BASE = RESULTS / "epoch-retention-mechanisms"
BASE_SHA = "b849e7b25de666fa74f1a8dfde5052627684398fa77e4c92dbea4d2a92d187d1"
PROTOCOL = RESULTS / "fusion-return-followup/protocol.md"
VERIFY_HELPERS = {
    "verify_epoch_retention.py": "417a3e93cc4680e77722fe2c95e24f206d532399bd4e2efc8d9951a24530e9f1",
    "epoch_retention_native.rs": "6ee51bce1a585838bbdae153fba16e18a6537ade38c9da72dadd92e50982a8e6",
}
GROUP = ("partition", "noise", "variant", "mode", "operation_selector")
OUTCOMES = (
    "returned_one_checkpoint",
    "returned_longer_ended",
    "returned_spell_censored",
    "no_return_censored",
)
COUNTS = (
    "census_events",
    "received_events",
    "unobserved_ttl_limit_events",
    "observed_loss_events",
    "returned_events",
    "censored_events",
    "non_event_return_events",
    "full_anchors",
    "clipped_anchors",
    "no_forward_anchors",
    "rp_records",
    "rp_different_preview_records",
    "future_opportunity_references",
    "opportunities_through_endpoint",
    "received_through_endpoint",
    "missed_through_endpoint",
    "precision_through_endpoint",
    *(s + "_events" for s in OUTCOMES),
)
MASSES = (
    "census_case_mass",
    "received_case_mass",
    "observed_loss_case_mass",
    "returned_case_mass",
    "censored_case_mass",
    "rp_case_mass",
    "rp_different_preview_case_mass",
    *(s + "_case_mass" for s in OUTCOMES),
)


def availability_runs(points, start=7500, end=9000):
    assert set(points) == set(
        range(start, end + 1)
    ), "incomplete or duplicate late grid"
    result = []
    for n in range(start, end + 1):
        available = points[n]["fusion_available"]
        assert isinstance(available, bool)
        if not result or result[-1][2] != available:
            result.append((n, n, available))
        else:
            a, _, v = result[-1]
            result[-1] = (a, n, v)
    return result


def followup(e, points, opportunities, ne, nr, end=9000):
    origin = step(e["event_time_s"])
    loss = flag(e["preview_loss"])
    parent = integer(e["parent_rp_records"])
    meta = {k: e[k] for k in (*META, "event_id", "source_event_id")}
    assert points[origin]["fusion_available"] == (
        e["immediate_preview"] == "three_epoch"
    )
    census_weight = Fraction(1, ne)
    received_weight = Fraction(1, nr) if flag(e["received"]) else Fraction()
    out = dict(
        **meta,
        event_kind=e["event_kind"],
        event_time_s=origin / 50,
        scheduled=flag(e["scheduled"]),
        received=flag(e["received"]),
        precision_activation=flag(e["precision_activation"]),
        horizon=e["horizon"],
        immediate_actual=e["immediate_actual"],
        immediate_preview=e["immediate_preview"],
        preview_loss=loss,
        loss_operation=e["loss_operation"],
        loss_offset_ms=integer(e["loss_offset_ms"]) if loss else None,
        ttl_limit_without_next_step=flag(e["immediate_at_ttl_limit"])
        and not flag(e["has_forward_sample"]),
        census_case_weight=census_weight,
        received_case_weight=received_weight,
        parent_rp_records=parent,
        rp_records=integer(e["rp_records"]),
        rp_different_preview_records=integer(e["rp_different_preview_records"]),
        rp_case_mass=(
            Fraction(integer(e["rp_records"]), parent) if parent else Fraction()
        ),
        rp_different_preview_case_mass=(
            Fraction(integer(e["rp_different_preview_records"]), parent)
            if parent
            else Fraction()
        ),
    )
    nullable = (
        "loss_time_s",
        "followup_end_time_s",
        "observed_followup_s",
        "returned",
        "first_return_time_s",
        "return_wait_s",
        "censor_wait_lower_bound_s",
        "first_return_received",
        "first_return_precision",
        "first_return_event_id",
        "first_return_groups",
        "first_return_epoch_steps",
        "first_return_sample_counts",
        "first_return_oldest_ttl_ms",
        "first_return_at_ttl_limit",
        "first_return_spell_end_time_s",
        "first_return_spell_span_s",
        "first_return_spell_right_censored",
        "first_return_one_checkpoint",
        "return_category",
        "next_opportunity_event_id",
        "next_opportunity_time_s",
        "next_opportunity_kind",
        "next_received_event_id",
        "next_received_time_s",
        "future_opportunity_references",
        "opportunities_through_endpoint",
        "received_through_endpoint",
        "missed_through_endpoint",
        "precision_through_endpoint",
    )
    out.update({k: None for k in nullable})
    if not loss:
        out["followup_status"] = (
            "unobserved_ttl_limit"
            if out["ttl_limit_without_next_step"]
            else "no_observed_loss"
        )
        return out, []
    lost = origin + integer(e["loss_offset_ms"]) // 20
    assert (
        origin < lost <= min(origin + 75, end)
        and integer(e["loss_offset_ms"]) % 20 == 0
    )
    assert points[lost - 1]["fusion_available"] and not points[lost]["fusion_available"]
    returned = next(
        (n for n in range(lost + 1, end + 1) if points[n]["fusion_available"]), None
    )
    out.update(
        loss_time_s=lost / 50,
        followup_end_time_s=end / 50,
        observed_followup_s=(end - lost) / 50,
        returned=returned is not None,
        first_return_time_s=returned / 50 if returned is not None else None,
        return_wait_s=(returned - lost) / 50 if returned is not None else None,
        censor_wait_lower_bound_s=(end - lost) / 50 if returned is None else None,
        followup_status="returned" if returned is not None else "right_censored",
    )
    by_time = indexed(opportunities, ("event_time_s",))
    indexed(opportunities, ("event_id",))
    index_steps = {step(r["event_time_s"]): r for r in opportunities}
    assert len(index_steps) == len(by_time)
    for r in opportunities:
        assert all(r[k] == e[k] for k in (*KEY, "case_id")), "wrong-case opportunity"
    future = sorted(
        (r for r in opportunities if lost < step(r["event_time_s"]) <= end),
        key=lambda r: step(r["event_time_s"]),
    )
    received = [r for r in future if flag(r["received"])]
    if future:
        out.update(
            next_opportunity_event_id=future[0]["event_id"],
            next_opportunity_time_s=float(future[0]["event_time_s"]),
            next_opportunity_kind=future[0]["event_kind"],
        )
    if received:
        out.update(
            next_received_event_id=received[0]["event_id"],
            next_received_time_s=float(received[0]["event_time_s"]),
        )
    if returned is not None:
        s = points[returned]
        epochs = numbers(s["epoch_steps"])
        assert len(epochs) == s["retained_groups"] == 3
        event = index_steps.get(returned)
        ended = next(
            (
                n
                for n in range(returned + 1, end + 1)
                if not points[n]["fusion_available"]
            ),
            None,
        )
        one = ended == returned + 1 if ended is not None else None
        category = (
            "returned_spell_censored"
            if ended is None
            else "returned_one_checkpoint" if one else "returned_longer_ended"
        )
        out.update(
            first_return_received=s["received"],
            first_return_precision=s["precision_activation"],
            first_return_event_id=event["event_id"] if event else None,
            first_return_groups=s["retained_groups"],
            first_return_epoch_steps=s["epoch_steps"],
            first_return_sample_counts=s["sample_counts"],
            first_return_oldest_ttl_ms=15000 - (returned - epochs[0]) * 20,
            first_return_at_ttl_limit=returned - epochs[0] == 750,
            first_return_spell_end_time_s=(ended if ended is not None else end) / 50,
            first_return_spell_span_s=((ended if ended is not None else end) - returned)
            / 50,
            first_return_spell_right_censored=ended is None,
            first_return_one_checkpoint=one,
            return_category=category,
        )
    else:
        out["return_category"] = "no_return_censored"
    through = [
        r
        for r in future
        if step(r["event_time_s"]) <= (returned if returned is not None else end)
    ]
    out.update(
        future_opportunity_references=len(future),
        opportunities_through_endpoint=len(through),
        received_through_endpoint=sum(flag(r["received"]) for r in through),
        missed_through_endpoint=sum(
            flag(r["scheduled"]) and not flag(r["received"]) for r in through
        ),
        precision_through_endpoint=sum(
            flag(r["precision_activation"]) for r in through
        ),
    )
    references = []
    for r in future:
        n = step(r["event_time_s"])
        s = points[n]
        references.append(
            dict(
                **{k: e[k] for k in META},
                anchor_event_id=e["event_id"],
                anchor_source_event_id=e["source_event_id"],
                anchor_loss_operation=e["loss_operation"],
                anchor_loss_time_s=lost / 50,
                opportunity_event_id=r["event_id"],
                opportunity_source_event_id=r["source_event_id"],
                opportunity_kind=r["event_kind"],
                opportunity_time_s=n / 50,
                scheduled=flag(r["scheduled"]),
                received=flag(r["received"]),
                precision_activation=flag(r["precision_activation"]),
                fusion_available=s["fusion_available"],
                retained_groups=s["retained_groups"],
                epoch_steps=s["epoch_steps"],
                sample_counts=s["sample_counts"],
                relation=(
                    "no_return_observed"
                    if returned is None
                    else (
                        "at_or_before_first_return"
                        if n <= returned
                        else "after_first_return"
                    )
                ),
                offset_from_loss_ms=(n - lost) * 20,
                offset_from_return_ms=(
                    (n - returned) * 20 if returned is not None else None
                ),
                anchor_census_case_weight=census_weight,
                anchor_received_case_weight=received_weight,
            )
        )
    return out, references


def collect(cases, events, raw, roots):
    by_case = defaultdict(list)
    for e in events:
        by_case[e["case_id"]].append(e)
    files = defaultdict(dict)
    for c in cases:
        name = f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"
        files[(c["cohort"], name)][(c["scenario"], c["planner"])] = c
    assert set(files) == {
        (cohort, name) for cohort, s in raw.items() for name in s["raw_sha256"]
    }
    runs, anchors, references = [], [], []
    byte_count = decisions = grid_rows = 0
    for ordinal, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert digest(path) == raw[cohort]["raw_sha256"][name]
        byte_count += path.stat().st_size
        states = {
            c["case_id"]: Metadata()
            for c in members.values()
            if flag(c["selected_timeout"])
        }
        late = {cid: {} for cid in states}
        last = {}
        lookup = {
            cid: {step(e["event_time_s"]): e for e in by_case[cid]} for cid in states
        }
        with path.open(newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
            assert len(header) == len(set(header))
            positions = {
                k: header.index(k)
                for k in (
                    "scenario",
                    "planner",
                    "time_s",
                    "measured_speed",
                    "observation_age",
                    "correction",
                    "correction_received",
                    "precision_activation",
                    "epoch_fusion_available",
                    "epoch_fusion_group_count",
                    "stationary_epoch_fusion",
                )
            }
            for row in reader:
                identity = row[positions["scenario"]], row[positions["planner"]]
                c = members[identity]
                n = step(row[positions["time_s"]])
                assert (
                    n == last.get(identity, -1) + 1
                ), "duplicate, missing or unordered raw checkpoint"
                last[identity] = n
                cid = c["case_id"]
                if cid not in states:
                    continue
                grid_rows += 1
                if n == 0:
                    continue
                assert flag(row[positions["stationary_epoch_fusion"]], raw=True)
                s = states[cid].advance(
                    n,
                    float(row[positions["measured_speed"]]),
                    float(row[positions["observation_age"]]),
                    flag(row[positions["correction_received"]], raw=True),
                    flag(row[positions["precision_activation"]], raw=True),
                )
                assert s["fusion_available"] == flag(
                    row[positions["epoch_fusion_available"]], raw=True
                ), (cid, n)
                assert s["retained_groups"] == integer(
                    row[positions["epoch_fusion_group_count"]]
                ), (cid, n)
                decisions += 1
                if n >= 7500:
                    late[cid][n] = s
                    scheduled = flag(row[positions["correction"]], raw=True)
                    assert (n in lookup[cid]) == (
                        scheduled or s["precision_activation"]
                    )
                    if n in lookup[cid]:
                        e = lookup[cid][n]
                        assert (
                            scheduled,
                            s["received"],
                            s["precision_activation"],
                        ) == tuple(
                            flag(e[k])
                            for k in ("scheduled", "received", "precision_activation")
                        )
        assert set(last) == set(members)
        for identity, c in members.items():
            cid = c["case_id"]
            assert last[identity] == (9000 if cid in states else step(c["time_s"]))
            if cid not in states:
                continue
            points = late[cid]
            for run, (a, b, available) in enumerate(availability_runs(points), 1):
                runs.append(
                    dict(
                        **{k: c[k] for k in META},
                        run=run,
                        start_time_s=a / 50,
                        end_time_s=b / 50,
                        checkpoint_count=b - a + 1,
                        fusion_available=available,
                    )
                )
            ne = len(by_case[cid])
            nr = sum(flag(e["received"]) for e in by_case[cid])
            for e in by_case[cid]:
                out, refs = followup(e, points, by_case[cid], ne, nr)
                anchors.append(out)
                references.extend(refs)
        if ordinal % 12 == 0:
            print(f"Followed original histories: {ordinal}/108 raw files", flush=True)
    runs.sort(key=lambda r: (r["case_id"], r["run"]))
    anchors.sort(key=lambda r: (r["case_id"], r["event_time_s"]))
    references.sort(
        key=lambda r: (r["case_id"], int(r["anchor_event_id"]), r["opportunity_time_s"])
    )
    return (
        runs,
        anchors,
        references,
        dict(
            raw_files=len(files),
            raw_bytes=byte_count,
            verified_decisions=decisions,
            timeout_grid_rows=grid_rows,
        ),
    )


def summarize(cases, anchors, meta):
    loss = [r for r in anchors if r["preview_loss"]]
    returned = [r for r in loss if r["returned"]]
    censored = [r for r in loss if not r["returned"]]
    mass = lambda rr, k="census_case_weight": sum((r[k] for r in rr), Fraction())
    total = lambda field: sum(r[field] or 0 for r in anchors)
    waits = [r["return_wait_s"] for r in returned]
    bounds = [r["censor_wait_lower_bound_s"] for r in censored]
    out = dict(
        **meta,
        planned_cases=len(cases),
        timeout_cases=sum(flag(c["selected_timeout"]) for c in cases),
        held_cases=sum(flag(c["held"]) for c in cases),
        census_cases=len({r["case_id"] for r in anchors}),
        observed_loss_cases=len({r["case_id"] for r in loss}),
        returned_cases=len({r["case_id"] for r in returned}),
        censored_cases=len({r["case_id"] for r in censored}),
        census_events=len(anchors),
        received_events=sum(r["received"] for r in anchors),
        unobserved_ttl_limit_events=sum(
            r["ttl_limit_without_next_step"] for r in anchors
        ),
        observed_loss_events=len(loss),
        returned_events=len(returned),
        censored_events=len(censored),
        non_event_return_events=sum(
            r["first_return_event_id"] is None for r in returned
        ),
        census_case_mass=mass(anchors),
        received_case_mass=mass(anchors, "received_case_weight"),
        observed_loss_case_mass=mass(loss),
        returned_case_mass=mass(returned),
        censored_case_mass=mass(censored),
        rp_records=total("rp_records"),
        rp_different_preview_records=total("rp_different_preview_records"),
        rp_case_mass=mass(anchors, "rp_case_mass"),
        rp_different_preview_case_mass=mass(anchors, "rp_different_preview_case_mass"),
        **{
            name: total(name)
            for name in (
                "future_opportunity_references",
                "opportunities_through_endpoint",
                "received_through_endpoint",
                "missed_through_endpoint",
                "precision_through_endpoint",
            )
        },
        **{
            h + "_anchors": sum(r["horizon"] == h for r in anchors)
            for h in ("full", "clipped", "no_forward")
        },
        min_observed_return_wait_s=min(waits) if waits else None,
        max_observed_return_wait_s=max(waits) if waits else None,
        min_censored_wait_lower_bound_s=min(bounds) if bounds else None,
        max_censored_wait_lower_bound_s=max(bounds) if bounds else None,
        return_fraction=Fraction(len(returned), len(loss)) if loss else None,
        return_case_mass_fraction=mass(returned) / mass(loss) if loss else None,
    )
    for category in OUTCOMES:
        selected = [r for r in loss if r["return_category"] == category]
        out[category + "_events"] = len(selected)
        out[category + "_case_mass"] = mass(selected)
    return out


def aggregate(cases, anchors):
    result = []
    for part, noise, arm, mode in product(
        ("known", "new", "fresh"),
        ("white", "correlated"),
        ("epoch", "curved"),
        ("none", "random", "blackout", "all_modes"),
    ):
        accepts = lambda r: (r["partition"], r["noise"], r["variant"]) == (
            part,
            noise,
            arm,
        ) and (mode == "all_modes" or r["mode"] == mode)
        cc = [c for c in cases if accepts(c)]
        rr = [r for r in anchors if accepts(r)]
        for operation in ("all", *ALL_OPS):
            selected = [
                r for r in rr if operation == "all" or r["loss_operation"] == operation
            ]
            result.append(
                summarize(
                    cc, selected, dict(zip(GROUP, (part, noise, arm, mode, operation)))
                )
            )
    return result


def closure(rows):
    lookup = indexed(rows, GROUP)
    checks = 0
    for r in rows:
        for field, whole, pieces in (
            ("mode", "all_modes", ("none", "random", "blackout")),
            ("operation_selector", "all", ALL_OPS),
        ):
            if r[field] != whole:
                continue
            children = [
                lookup[tuple(v if k == field else r[k] for k in GROUP)] for v in pieces
            ]
            for name in (*COUNTS, *MASSES):
                assert sum(c[name] for c in children) == r[name], (field, name)
                checks += 1
        assert r["returned_events"] + r["censored_events"] == r["observed_loss_events"]
        assert sum(r[s + "_events"] for s in OUTCOMES) == r["observed_loss_events"]
        assert (
            sum(r[s + "_case_mass"] for s in OUTCOMES) == r["observed_loss_case_mass"]
        )
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--raw-roots", type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    assert digest(BASE / "sources.json") == BASE_SHA
    prior = json.loads((BASE / "sources.json").read_text())
    pins = {
        **prior["input_sha256"],
        "epoch-retention-mechanisms/sources.json": BASE_SHA,
        **{
            "epoch-retention-mechanisms/" + k: h
            for k, h in prior["output_sha256"].items()
        },
    }
    helpers = {
        **prior["analysis"]["helpers_sha256"],
        "analyze_epoch_retention.py": prior["analysis"]["script_sha256"],
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
    events = read_rows(BASE / "event-mechanisms.csv")
    indexed(cases, KEY)
    indexed(events, ("event_id",))
    runs, anchors, refs, audit = collect(cases, events, prior["raw"], roots)
    summaries = aggregate(cases, anchors)
    checks = closure(summaries)
    assert len(cases) == 972 and len({c["family_id"] for c in cases}) == 324
    assert (
        len(anchors) == 3997
        and sum(r["preview_loss"] for r in anchors) == 152
        and sum(r["ttl_limit_without_next_step"] for r in anchors) == 18
    )
    assert (
        len({r["case_id"] for r in runs}) == 570
        and sum(r["checkpoint_count"] for r in runs) == 855570
    )
    assert (
        len(summaries) == 528
        and audit["verified_decisions"] == 5130000
        and audit["timeout_grid_rows"] == 5130570
    )
    assert (
        sum(r["census_case_weight"] for r in anchors)
        == sum(r["received_case_weight"] for r in anchors)
        == 570
    )
    assert (
        sum(r["rp_records"] for r in anchors) == 10995
        and sum(r["rp_case_mass"] for r in anchors) == 157
    )
    audit.update(
        planned_cases=972,
        planned_families=324,
        timeout_cases=570,
        census_events=len(anchors),
        observed_loss_anchors=152,
        unobserved_ttl_limit_anchors=18,
        late_checkpoint_references=855570,
        availability_runs=len(runs),
        opportunity_references=len(refs),
        summary_rows=len(summaries),
        additive_closure_comparisons=checks,
    )
    args.output.mkdir(parents=True)
    for name, rr in (
        ("availability-runs", runs),
        ("loss-followup", anchors),
        ("opportunity-followup", refs),
        ("followup-summary", summaries),
    ):
        save_rows(
            args.output / (name + ".csv"),
            [
                {k: float(v) if isinstance(v, Fraction) else v for k, v in r.items()}
                for r in rr
            ],
        )
    meta = dict(
        input_sha256=pins,
        raw=prior["raw"],
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
            verifier_helpers_sha256=VERIFY_HELPERS,
            code_sha256=prior["analysis"]["code_sha256"],
        ),
        schema=dict(
            late_start_s=150,
            followup_end_s=180,
            step_ms=20,
            operation_selectors=("all", *ALL_OPS),
            return_categories=OUTCOMES,
            weighting="original all-census, received-census and RP parents before selection; return fractions condition on observed losses",
            censoring="no observed return by 180 s is right censored; a return at 180 s is observed and its spell is right censored",
            metadata_only=True,
            retrospective=True,
        ),
        inherited_validation=prior["inherited_validation"],
        audit=audit,
        output_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
