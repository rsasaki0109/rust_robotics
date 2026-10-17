#!/usr/bin/env python3
"""Reconstruct first returns from all late native Rust availability runs."""

import argparse
from collections import Counter, defaultdict
import csv
from fractions import Fraction
from itertools import groupby, product
import json
import math
from pathlib import Path
import shutil
import subprocess

import verify_epoch_retention as native

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
BASE_SHA = "b849e7b25de666fa74f1a8dfde5052627684398fa77e4c92dbea4d2a92d187d1"
HELPERS = {
    "verify_epoch_retention.py": "417a3e93cc4680e77722fe2c95e24f206d532399bd4e2efc8d9951a24530e9f1",
    "epoch_retention_native.rs": "6ee51bce1a585838bbdae153fba16e18a6537ade38c9da72dadd92e50982a8e6",
}
META = native.META
KEY = native.KEY
GROUP = ("partition", "noise", "variant", "mode", "operation_selector")
OPS = native.OPS
OUTCOMES = (
    "returned_one_checkpoint",
    "returned_longer_ended",
    "returned_spell_censored",
    "no_return_censored",
)
checks = 0


def check(value, expected):
    global checks
    checks += 1
    if expected is None:
        assert value == "", (value, expected)
    elif isinstance(expected, (str, int, bool)):
        assert value == str(expected), (value, expected)
    else:
        assert value != "" and math.isclose(
            float(value), float(expected), rel_tol=1e-10, abs_tol=2e-12
        ), (value, expected)


def fields(row, expected):
    assert set(row) == set(expected), (
        set(row) - set(expected),
        set(expected) - set(row),
    )
    for k, v in expected.items():
        try:
            check(row[k], v)
        except AssertionError as exc:
            raise AssertionError((k, row.get("event_id"), exc.args)) from exc


def original_events():
    cases, broad = native.original_inputs()
    events = native.index(
        native.read(RESULTS / "epoch-retention-mechanisms/event-mechanisms.csv"),
        ("event_id",),
    )
    assert set(broad) == {k[0] for k in events}
    for (eid,), e in events.items():
        original = broad[eid]
        for k in (
            *META,
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
            check(e[k], original[k])
        check(e["preview_loss"], bool(int(original["preview_changes"])))
        check(
            e["loss_offset_ms"],
            (
                int(original["first_preview_change_offset_ms"])
                if int(original["preview_changes"])
                else None
            ),
        )
    return cases, {eid: e for (eid,), e in events.items()}


def reconstruct_case(c, events, points):
    cid = c["case_id"]
    assert set(points) == set(range(7500, 9001))
    grouped = []
    for available, items in groupby(
        sorted(points), key=lambda n: points[n]["fusion_available"]
    ):
        steps = list(items)
        assert steps == list(range(steps[0], steps[-1] + 1))
        grouped.append((steps[0], steps[-1], available))
    runs = {}
    for i, (lo, hi, available) in enumerate(grouped, 1):
        runs[(cid, str(i))] = dict(
            **{k: c[k] for k in META},
            run=i,
            start_time_s=lo / 50,
            end_time_s=hi / 50,
            checkpoint_count=hi - lo + 1,
            fusion_available=available,
        )
    ne = len(events)
    nr = sum(native.truth(e["received"]) for e in events)
    event_at = {native.tick(e["event_time_s"]): e for e in events}
    assert len(event_at) == len(events)
    for n, e in event_at.items():
        assert points[n]["received"] == native.truth(e["received"]) and points[n][
            "precision_activation"
        ] == native.truth(e["precision_activation"])
    anchors, refs = {}, {}
    for e in events:
        eid = e["event_id"]
        origin = native.tick(e["event_time_s"])
        loss = native.truth(e["preview_loss"])
        assert all(e[k] == c[k] for k in META)
        assert points[origin]["fusion_available"] == (
            e["immediate_preview"] == "three_epoch"
        )
        meta = {k: e[k] for k in (*META, "event_id", "source_event_id")}
        parent = int(e["parent_rp_records"])
        out = dict(
            **meta,
            event_kind=e["event_kind"],
            event_time_s=origin / 50,
            scheduled=native.truth(e["scheduled"]),
            received=native.truth(e["received"]),
            precision_activation=native.truth(e["precision_activation"]),
            horizon=e["horizon"],
            immediate_actual=e["immediate_actual"],
            immediate_preview=e["immediate_preview"],
            preview_loss=loss,
            loss_operation=e["loss_operation"],
            loss_offset_ms=int(e["loss_offset_ms"]) if loss else None,
            ttl_limit_without_next_step=native.truth(e["immediate_at_ttl_limit"])
            and not native.truth(e["has_forward_sample"]),
            census_case_weight=Fraction(1, ne),
            received_case_weight=(
                Fraction(1, nr) if native.truth(e["received"]) else Fraction()
            ),
            parent_rp_records=parent,
            rp_records=int(e["rp_records"]),
            rp_different_preview_records=int(e["rp_different_preview_records"]),
            rp_case_mass=(
                Fraction(int(e["rp_records"]), parent) if parent else Fraction()
            ),
            rp_different_preview_case_mass=(
                Fraction(int(e["rp_different_preview_records"]), parent)
                if parent
                else Fraction()
            ),
        )
        out.update(
            {
                k: None
                for k in (
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
            }
        )
        if not loss:
            out["followup_status"] = (
                "unobserved_ttl_limit"
                if out["ttl_limit_without_next_step"]
                else "no_observed_loss"
            )
            anchors[(eid,)] = out
            continue
        lost = origin + int(e["loss_offset_ms"]) // 20
        assert (
            not points[lost]["fusion_available"]
            and points[lost - 1]["fusion_available"]
        )
        ordinal = next(i for i, (lo, hi, _) in enumerate(grouped) if lo <= lost <= hi)
        lo, hi, available = grouped[ordinal]
        assert lo == lost and not available
        restored = grouped[ordinal + 1] if ordinal + 1 < len(grouped) else None
        returned = restored[0] if restored else None
        out.update(
            loss_time_s=lost / 50,
            followup_end_time_s=180.0,
            observed_followup_s=(9000 - lost) / 50,
            returned=restored is not None,
            first_return_time_s=returned / 50 if returned is not None else None,
            return_wait_s=(returned - lost) / 50 if returned is not None else None,
            censor_wait_lower_bound_s=(9000 - lost) / 50 if restored is None else None,
            followup_status="returned" if restored else "right_censored",
        )
        if restored:
            start, finish, available = restored
            assert available
            s = points[start]
            epochs = native.ints(s["epoch_steps"])
            assert s["retained_groups"] == len(epochs) == 3
            ended = finish + 1 if finish < 9000 else None
            one = (finish == start) if ended is not None else None
            end = ended if ended is not None else 9000
            event = event_at.get(start)
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
                first_return_oldest_ttl_ms=15000 - (start - epochs[0]) * 20,
                first_return_at_ttl_limit=start - epochs[0] == 750,
                first_return_spell_end_time_s=end / 50,
                first_return_spell_span_s=(end - start) / 50,
                first_return_spell_right_censored=ended is None,
                first_return_one_checkpoint=one,
                return_category=category,
            )
        else:
            out["return_category"] = "no_return_censored"
        future = [(n, r) for n, r in sorted(event_at.items()) if lost < n <= 9000]
        endpoint = returned if returned is not None else 9000
        through = [r for n, r in future if n <= endpoint]
        if future:
            n, r = future[0]
            out.update(
                next_opportunity_event_id=r["event_id"],
                next_opportunity_time_s=n / 50,
                next_opportunity_kind=r["event_kind"],
            )
        received = [(n, r) for n, r in future if native.truth(r["received"])]
        if received:
            n, r = received[0]
            out.update(
                next_received_event_id=r["event_id"], next_received_time_s=n / 50
            )
        out.update(
            future_opportunity_references=len(future),
            opportunities_through_endpoint=len(through),
            received_through_endpoint=sum(native.truth(r["received"]) for r in through),
            missed_through_endpoint=sum(
                native.truth(r["scheduled"]) and not native.truth(r["received"])
                for r in through
            ),
            precision_through_endpoint=sum(
                native.truth(r["precision_activation"]) for r in through
            ),
        )
        for n, r in future:
            s = points[n]
            refs[(eid, r["event_id"])] = dict(
                **{k: e[k] for k in META},
                anchor_event_id=eid,
                anchor_source_event_id=e["source_event_id"],
                anchor_loss_operation=e["loss_operation"],
                anchor_loss_time_s=lost / 50,
                opportunity_event_id=r["event_id"],
                opportunity_source_event_id=r["source_event_id"],
                opportunity_kind=r["event_kind"],
                opportunity_time_s=n / 50,
                scheduled=native.truth(r["scheduled"]),
                received=native.truth(r["received"]),
                precision_activation=native.truth(r["precision_activation"]),
                fusion_available=s["fusion_available"],
                retained_groups=s["retained_groups"],
                epoch_steps=s["epoch_steps"],
                sample_counts=s["sample_counts"],
                relation=(
                    "no_return_observed"
                    if restored is None
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
                anchor_census_case_weight=Fraction(1, ne),
                anchor_received_case_weight=(
                    Fraction(1, nr) if native.truth(e["received"]) else Fraction()
                ),
            )
        anchors[(eid,)] = out
    return runs, anchors, refs


def native_reconstruction(work, executable, cases, events, raw, roots):
    files = defaultdict(list)
    by_case = defaultdict(list)
    for c in cases:
        name = f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"
        files[(c["cohort"], name)].append(c)
    for e in events.values():
        by_case[e["case_id"]].append(e)
    assert set(files) == {
        (cohort, name) for cohort, s in raw.items() for name in s["raw_sha256"]
    }
    run_rows, anchors, refs = {}, {}, {}
    audit = Counter()
    trace_hashes = {}
    for ordinal, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert native.sha(path) == raw[cohort]["raw_sha256"][name]
        audit["raw_bytes"] += path.stat().st_size
        memberfile = work / f"{ordinal:03d}-cases.tsv"
        queryfile = work / f"{ordinal:03d}-queries.tsv"
        output = work / f"{ordinal:03d}-native.csv"
        memberfile.write_text(
            "".join(
                "\t".join(
                    (
                        c["case_id"],
                        c["scenario"],
                        c["planner"],
                        str(native.truth(c["selected_timeout"])).lower(),
                        str(native.tick(c["time_s"])),
                    )
                )
                + "\n"
                for c in members
            )
        )
        queryfile.write_text(
            "".join(
                "\t".join(
                    (
                        c["case_id"],
                        "late_" + str(n),
                        c["scenario"],
                        c["planner"],
                        str(n),
                    )
                )
                + "\n"
                for c in members
                if native.truth(c["selected_timeout"])
                for n in range(7500, 9001)
            )
        )
        with output.open("w") as f:
            result = subprocess.run(
                [str(executable), str(path), str(memberfile), str(queryfile)],
                stdout=f,
                stderr=subprocess.PIPE,
                text=True,
            )
        assert result.returncode == 0, (ordinal, result.stderr)
        counts = json.loads(result.stderr)
        audit["verified_decisions"] += counts["verified_decisions"]
        audit["timeout_grid_rows"] += counts["timeout_grid_rows"]
        audit["late_checkpoint_references"] += counts["snapshot_rows"]
        audit["raw_files"] += 1
        trace_hashes[cohort + "/" + name] = native.sha(output)
        late = defaultdict(dict)
        for row in native.read(output):
            cid = row["case_id"]
            n = native.tick(row["time_s"])
            assert (
                row["event_id"] == cid
                and row["role"] == "late_" + str(n)
                and n not in late[cid]
            )
            late[cid][n] = dict(
                fusion_available=native.truth(row["fusion_available"], True),
                received=native.truth(row["received"], True),
                precision_activation=native.truth(row["precision_activation"], True),
                retained_groups=int(row["retained_groups"]),
                epoch_steps=row["epoch_steps"],
                sample_counts=row["sample_counts"],
            )
        selected = {
            c["case_id"] for c in members if native.truth(c["selected_timeout"])
        }
        assert set(late) == selected
        for c in members:
            cid = c["case_id"]
            if cid not in selected:
                continue
            rr, aa, oo = reconstruct_case(c, by_case[cid], late[cid])
            assert (
                not set(rr) & set(run_rows)
                and not set(aa) & set(anchors)
                and not set(oo) & set(refs)
            )
            run_rows.update(rr)
            anchors.update(aa)
            refs.update(oo)
        if ordinal % 12 == 0:
            print(f"Original Rust followed: {ordinal}/108 raw files", flush=True)
    return run_rows, anchors, refs, dict(audit), trace_hashes


def summaries(cases, anchors):
    result = {}
    for part, noise, arm, mode, op in product(
        ("known", "new", "fresh"),
        ("white", "correlated"),
        ("epoch", "curved"),
        ("none", "random", "blackout", "all_modes"),
        ("all", *OPS),
    ):
        cc = [
            c
            for c in cases
            if (c["partition"], c["noise"], c["variant"]) == (part, noise, arm)
            and (mode == "all_modes" or c["mode"] == mode)
        ]
        rr = [
            r
            for r in anchors.values()
            if (r["partition"], r["noise"], r["variant"]) == (part, noise, arm)
            and (mode == "all_modes" or r["mode"] == mode)
            and (op == "all" or r["loss_operation"] == op)
        ]
        loss = [r for r in rr if r["preview_loss"]]
        returned = [r for r in loss if r["returned"]]
        censored = [r for r in loss if not r["returned"]]
        mass = lambda selected, k="census_case_weight": sum(
            (r[k] for r in selected), Fraction()
        )
        out = dict(zip(GROUP, (part, noise, arm, mode, op)))
        out.update(
            planned_cases=len(cc),
            timeout_cases=sum(native.truth(c["selected_timeout"]) for c in cc),
            held_cases=sum(native.truth(c["held"]) for c in cc),
            census_cases=len({r["case_id"] for r in rr}),
            observed_loss_cases=len({r["case_id"] for r in loss}),
            returned_cases=len({r["case_id"] for r in returned}),
            censored_cases=len({r["case_id"] for r in censored}),
            census_events=len(rr),
            received_events=sum(r["received"] for r in rr),
            unobserved_ttl_limit_events=sum(
                r["ttl_limit_without_next_step"] for r in rr
            ),
            observed_loss_events=len(loss),
            returned_events=len(returned),
            censored_events=len(censored),
            non_event_return_events=sum(
                r["first_return_event_id"] is None for r in returned
            ),
            census_case_mass=mass(rr),
            received_case_mass=mass(rr, "received_case_weight"),
            observed_loss_case_mass=mass(loss),
            returned_case_mass=mass(returned),
            censored_case_mass=mass(censored),
            rp_records=sum(r["rp_records"] for r in rr),
            rp_different_preview_records=sum(
                r["rp_different_preview_records"] for r in rr
            ),
            rp_case_mass=mass(rr, "rp_case_mass"),
            rp_different_preview_case_mass=mass(rr, "rp_different_preview_case_mass"),
        )
        for field in (
            "future_opportunity_references",
            "opportunities_through_endpoint",
            "received_through_endpoint",
            "missed_through_endpoint",
            "precision_through_endpoint",
        ):
            out[field] = sum(r[field] or 0 for r in rr)
        for horizon in ("full", "clipped", "no_forward"):
            out[horizon + "_anchors"] = sum(r["horizon"] == horizon for r in rr)
        for category in OUTCOMES:
            selected = [r for r in loss if r["return_category"] == category]
            out[category + "_events"] = len(selected)
            out[category + "_case_mass"] = mass(selected)
        waits = [r["return_wait_s"] for r in returned]
        bounds = [r["censor_wait_lower_bound_s"] for r in censored]
        out.update(
            min_observed_return_wait_s=min(waits) if waits else None,
            max_observed_return_wait_s=max(waits) if waits else None,
            min_censored_wait_lower_bound_s=min(bounds) if bounds else None,
            max_censored_wait_lower_bound_s=max(bounds) if bounds else None,
            return_fraction=Fraction(len(returned), len(loss)) if loss else None,
            return_case_mass_fraction=mass(returned) / mass(loss) if loss else None,
        )
        result[tuple(out[k] for k in GROUP)] = out
    return result


def closures(rows):
    distinct = (
        "planned_cases",
        "timeout_cases",
        "held_cases",
        "census_cases",
        "observed_loss_cases",
        "returned_cases",
        "censored_cases",
    )
    additive = [
        k
        for k, v in next(iter(rows.values())).items()
        if isinstance(v, (int, Fraction)) and k not in distinct and "fraction" not in k
    ]
    comparisons = 0
    for r in rows.values():
        for field, whole, pieces in (
            ("mode", "all_modes", ("none", "random", "blackout")),
            ("operation_selector", "all", OPS),
        ):
            if r[field] != whole:
                continue
            children = [
                rows[tuple(v if k == field else r[k] for k in GROUP)] for v in pieces
            ]
            for name in additive:
                assert sum(c[name] for c in children) == r[name], (field, name)
                comparisons += 1
        assert r["returned_events"] + r["censored_events"] == r["observed_loss_events"]
        assert sum(r[k + "_events"] for k in OUTCOMES) == r["observed_loss_events"]
        assert (
            sum(r[k + "_case_mass"] for k in OUTCOMES) == r["observed_loss_case_mass"]
        )
    return comparisons


def compare_file(path, expected, keys):
    saved = native.index(native.read(path), keys)
    assert set(saved) == set(expected), (path, len(saved), len(expected))
    for key, row in saved.items():
        fields(row, expected[key])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--figures", type=Path, required=True)
    parser.add_argument("--work", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--raw-roots", type=Path)
    parser.add_argument("--rustc", type=Path, default=shutil.which("rustc"))
    args = parser.parse_args()
    if not __debug__ or args.output.exists() or args.work.exists() or not args.rustc:
        parser.error(
            "Run without -O, with rustc on PATH or --rustc, a new work directory and unused output filename"
        )
    source = json.loads((args.results / "sources.json").read_text())
    base = RESULTS / "epoch-retention-mechanisms/sources.json"
    assert native.sha(base) == BASE_SHA
    prior = json.loads(base.read_text())
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        "epoch-retention-mechanisms/sources.json": BASE_SHA,
        **{
            "epoch-retention-mechanisms/" + k: h
            for k, h in prior["output_sha256"].items()
        },
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_epoch_retention.py": prior["analysis"]["script_sha256"],
    }
    assert source["analysis"]["verifier_helpers_sha256"] == HELPERS
    assert (
        source["analysis"]["code_sha256"] == native.CODE
        and source["raw"] == prior["raw"]
        and source["inherited_validation"] == prior["inherited_validation"]
    )
    for name, h in source["input_sha256"].items():
        assert native.sha(RESULTS / name) == h, name
    for name, h in {**source["analysis"]["helpers_sha256"], **HELPERS}.items():
        assert native.sha(ROOT / name) == h, name
    assert (
        native.sha(ROOT / "analyze_fusion_return.py")
        == source["analysis"]["script_sha256"]
    )
    assert (
        native.sha(RESULTS / "fusion-return-followup/protocol.md")
        == source["analysis"]["protocol_sha256"]
    )
    assert set(source["output_sha256"]) == {
        "availability-runs.csv",
        "loss-followup.csv",
        "opportunity-followup.csv",
        "followup-summary.csv",
    }
    for name, h in source["output_sha256"].items():
        assert native.sha(args.results / name) == h, name
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    args.work.mkdir(parents=True)
    executable, native_meta = native.compile_native(args.work, args.rustc)
    cases, events = original_events()
    runs, anchors, refs, audit, trace_hashes = native_reconstruction(
        args.work, executable, cases, events, source["raw"], roots
    )
    summary = summaries(cases, anchors)
    closure_checks = closures(summary)
    for name, expected, keys in (
        ("availability-runs", runs, ("case_id", "run")),
        ("loss-followup", anchors, ("event_id",)),
        ("opportunity-followup", refs, ("anchor_event_id", "opportunity_event_id")),
        ("followup-summary", summary, GROUP),
    ):
        compare_file(args.results / (name + ".csv"), expected, keys)
    assert (
        len(cases) == 972
        and len({c["family_id"] for c in cases}) == 324
        and len(anchors) == 3997
    )
    assert (
        len({r["case_id"] for r in runs.values()}) == 570
        and sum(r["checkpoint_count"] for r in runs.values()) == 855570
    )
    assert (
        sum(r["census_case_weight"] for r in anchors.values())
        == sum(r["received_case_weight"] for r in anchors.values())
        == 570
    )
    assert (
        sum(r["rp_case_mass"] for r in anchors.values()) == 157
        and sum(r["rp_records"] for r in anchors.values()) == 10995
    )
    audit.update(
        planned_cases=972,
        planned_families=324,
        timeout_cases=570,
        census_events=len(anchors),
        observed_loss_anchors=sum(r["preview_loss"] for r in anchors.values()),
        unobserved_ttl_limit_anchors=sum(
            r["ttl_limit_without_next_step"] for r in anchors.values()
        ),
        availability_runs=len(runs),
        opportunity_references=len(refs),
        summary_rows=len(summary),
        additive_closure_comparisons=closure_checks,
    )
    assert audit == source["audit"], (audit, source["audit"])
    plots = json.loads((args.figures / "plot-sources.json").read_text())
    assert plots["script_sha256"] == native.sha(ROOT / "plot_fusion_return.py")
    assert plots["input_sha256"] == {
        n: native.sha(args.results / n)
        for n in ("sources.json", "loss-followup.csv", "followup-summary.csv")
    }
    assert set(plots["output_sha256"]) == {
        "return-coverage.svg",
        "return-wait.svg",
        "plot-values.csv",
        "plot-event-values.csv",
    }
    for name, h in plots["output_sha256"].items():
        assert native.sha(args.figures / name) == h, name
    plotted_summary = {k: r for k, r in summary.items() if r["mode"] != "all_modes"}
    plotted_events = {k: r for k, r in anchors.items() if r["preview_loss"]}
    assert len(plotted_summary) == 396 and len(plotted_events) == 152
    compare_file(args.figures / "plot-values.csv", plotted_summary, GROUP)
    compare_file(args.figures / "plot-event-values.csv", plotted_events, ("event_id",))
    audit.update(
        comparisons=checks,
        original_census_join_comparisons=native.comparisons,
        plotted_summary_rows=396,
        plotted_event_rows=152,
        native=native_meta,
        native_trace_sha256=trace_hashes,
        script_sha256=native.sha(Path(__file__)),
        sources_sha256=native.sha(args.results / "sources.json"),
        plot_sources_sha256=native.sha(args.figures / "plot-sources.json"),
        method="unchanged original Rust observe and six tests; every late checkpoint in every timeout case; first returns from independent maximal native runs; no analyzer imports",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(audit, indent=2) + "\n")
    print(
        json.dumps(
            {k: v for k, v in audit.items() if k not in ("native_trace_sha256",)},
            indent=2,
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
