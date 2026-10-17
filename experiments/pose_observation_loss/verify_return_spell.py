#!/usr/bin/env python3
"""Verify returned-spell mechanisms with a fresh original Rust metadata replay."""

import argparse
from collections import Counter, defaultdict
from fractions import Fraction
from itertools import groupby, product
import json
from pathlib import Path
import shutil

import verify_fusion_return as previous

native = previous.native
ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
BASE_SHA = "076db5ab572c491d53a88f14493006e830ba75be637bae6a3dd5bd24ebdc3cc7"
PROOF_SHA = "700e730c54999c7aa2ab5297346302106593815413ddb7659bd4bb684b6488b2"
HELPERS = {
    "verify_epoch_retention.py": "417a3e93cc4680e77722fe2c95e24f206d532399bd4e2efc8d9951a24530e9f1",
    "epoch_retention_native.rs": "6ee51bce1a585838bbdae153fba16e18a6537ade38c9da72dadd92e50982a8e6",
    "verify_fusion_return.py": "cfd2b0377e84697cec6e2b58735d135f58ebda876b8013e5ff718cbd6e35fe34",
}
META, KEY = native.META, native.KEY
OPS = native.OPS[:-1]
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


def gate_evidence(s):
    epochs, counts = native.ints(s["epoch_steps"]), native.ints(s["sample_counts"])
    assert len(epochs) == len(counts) == s["retained_groups"]
    return {
        "age_blocked": not s["observation_age"] <= 1.5 + 1e-9,
        "group_count_blocked": len(epochs) != 3,
        "latest_epoch_blocked": len(epochs) > 0
        and (
            s["current_epoch_time_s"] is None
            or epochs[-1] != native.tick(s["current_epoch_time_s"])
        ),
        "older_samples_blocked": not all(
            c >= 10 for c in counts[: max(0, len(counts) - 1)]
        ),
    }


def end_operation(s, gates):
    conditions = (
        (s["moving"] and s["precision_activation"], "motion_and_precision_clear"),
        (s["moving"], "motion_clear"),
        (s["precision_activation"], "precision_clear"),
        (bool(s["expired_epoch_steps"]), "ttl_expiry"),
        (bool(s["capacity_evicted_epoch_steps"]), "capacity_eviction"),
        (gates["age_blocked"], "age_gate"),
        (
            s["retained_groups"] == 3 and gates["latest_epoch_blocked"],
            "latest_epoch_gate",
        ),
        (
            s["retained_groups"] == 3 and gates["older_samples_blocked"],
            "older_samples_gate",
        ),
    )
    return next((name for condition, name in conditions if condition), "other_gate")


def full_points(path, members):
    result = defaultdict(dict)
    for row in native.read(path):
        cid, n = row["case_id"], native.tick(row["time_s"])
        assert row["event_id"] == cid and row["role"] == "late_" + str(n)
        assert n not in result[cid]
        s = {k: v for k, v in row.items() if k not in ("case_id", "role", "event_id")}
        for k in ("fusion_available", "moving", "received", "precision_activation"):
            s[k] = native.truth(s[k], True)
        for k in (
            "time_s",
            "current_epoch_time_s",
            "measured_speed",
            "observation_age",
        ):
            s[k] = float(s[k]) if s[k] else None
        s["retained_groups"] = int(s["retained_groups"])
        result[cid][n] = s
    assert set(result) == {
        c["case_id"] for c in members if native.truth(c["selected_timeout"])
    }
    assert all(sorted(pp) == list(range(7500, 9001)) for pp in result.values())
    return result


def reconstruct(e, points, opportunities):
    out = {
        k: e[k]
        for k in (
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
            "event_time_s",
            "scheduled",
            "received",
            "precision_activation",
            "preview_loss",
            "loss_offset_ms",
            "loss_time_s",
            "returned",
            "censor_wait_lower_bound_s",
            "ttl_limit_without_next_step",
            "census_case_weight",
            "received_case_weight",
            "parent_rp_records",
            "rp_records",
            "rp_different_preview_records",
            "rp_case_mass",
            "rp_different_preview_case_mass",
        )
    }
    out["no_return"] = e["preview_loss"] and not e["returned"]
    empty = (
        "return_time_s return_wait_s return_event_id returned_spell_span_s returned_spell_right_censored ended censored_return one_checkpoint "
        "end_time_s end_operation return_epoch_steps return_sample_counts return_observation_age_s return_oldest_ttl_ms return_ttl_limit "
        "conditional_ttl_first_invalid_offset_ms conditional_age_first_invalid_offset_ms end_at_ttl_deadline end_at_age_deadline "
        "end_moving end_precision_activation end_ttl_expired end_capacity_evicted end_age_blocked end_latest_epoch_blocked "
        "end_older_samples_blocked end_group_count_blocked next_opportunity_event_id next_opportunity_kind next_opportunity_time_s "
        "next_opportunity_offset_ms next_received_event_id next_received_time_s next_received_offset_ms end_vs_next_received"
    ).split()
    out.update(dict.fromkeys(empty))
    if not e["returned"]:
        return out, {}, {}
    intervals = []
    for available, items in groupby(
        sorted(points), lambda n: points[n]["fusion_available"]
    ):
        ns = list(items)
        intervals.append((ns[0], ns[-1], available))
    loss = native.tick(e["loss_time_s"])
    available_runs = [(lo, hi) for lo, hi, v in intervals if v and lo > loss]
    start, last = available_runs[0]
    assert start == native.tick(e["first_return_time_s"])
    finish = last + 1 if last < 9000 else None
    assert e["first_return_spell_right_censored"] == (finish is None)
    assert native.tick(e["first_return_spell_end_time_s"]) == (
        finish if finish else 9000
    )
    s = points[start]
    oldest = native.ints(s["epoch_steps"])[0]
    ttl_deadline = next(i * 20 for i in range(1, 752) if start + i - oldest > 750)
    age_deadline = next(
        i * 20 for i in range(1, 77) if s["observation_age"] + i / 50 > 1.5 + 1e-9
    )
    event_at = {native.tick(o["event_time_s"]): o for o in opportunities}
    assert len(event_at) == len(opportunities)
    assert all(all(o[k] == e[k] for k in (*KEY, "case_id")) for o in opportunities)
    returned_event = event_at.get(start)
    out.update(
        return_time_s=start / 50,
        return_wait_s=(start - loss) / 50,
        return_event_id=returned_event["event_id"] if returned_event else None,
        returned_spell_span_s=((finish if finish else 9000) - start) / 50,
        returned_spell_right_censored=finish is None,
        ended=finish is not None,
        censored_return=finish is None,
        one_checkpoint=(last == start) if finish else None,
        return_epoch_steps=s["epoch_steps"],
        return_sample_counts=s["sample_counts"],
        return_observation_age_s=s["observation_age"],
        return_oldest_ttl_ms=(750 - start + oldest) * 20,
        return_ttl_limit=(start - oldest == 750),
        conditional_ttl_first_invalid_offset_ms=ttl_deadline,
        conditional_age_first_invalid_offset_ms=age_deadline,
    )
    future = [event_at[n] for n in sorted(event_at) if n > start]
    recv = next((o for o in future if o["received"]), None)
    if future:
        o = future[0]
        out.update(
            next_opportunity_event_id=o["event_id"],
            next_opportunity_kind=o["event_kind"],
            next_opportunity_time_s=o["event_time_s"],
            next_opportunity_offset_ms=(native.tick(o["event_time_s"]) - start) * 20,
        )
    if recv:
        out.update(
            next_received_event_id=recv["event_id"],
            next_received_time_s=recv["event_time_s"],
            next_received_offset_ms=(native.tick(recv["event_time_s"]) - start) * 20,
        )
    roles = {"return": start}
    if finish:
        a = points[finish]
        gates = gate_evidence(a)
        relation = "no_next_received_in_log"
        if recv:
            delta = finish - native.tick(recv["event_time_s"])
            relation = (
                "same_checkpoint" if delta == 0 else "before" if delta < 0 else "after"
            )
        out.update(
            end_time_s=finish / 50,
            end_operation=end_operation(a, gates),
            end_at_ttl_deadline=(finish - start) * 20 == ttl_deadline,
            end_at_age_deadline=(finish - start) * 20 == age_deadline,
            end_moving=a["moving"],
            end_precision_activation=a["precision_activation"],
            end_ttl_expired=bool(a["expired_epoch_steps"]),
            end_capacity_evicted=bool(a["capacity_evicted_epoch_steps"]),
            end_vs_next_received=relation,
            **{"end_" + k: v for k, v in gates.items()},
        )
        roles.update(end_before=finish - 1, end_after=finish)
    else:
        roles["censored_endpoint"] = 9000
    snapshots, groups = {}, {}
    for role, n in roles.items():
        a = points[n]
        common = {
            **{k: e[k] for k in (*META, "event_id", "source_event_id")},
            "role": role,
        }
        snapshots[(e["event_id"], role)] = dict(
            **common, offset_from_return_ms=(n - start) * 20, **a, **gate_evidence(a)
        )
        for i, (epoch, count) in enumerate(
            zip(native.ints(a["epoch_steps"]), native.ints(a["sample_counts"])), 1
        ):
            groups[(e["event_id"], role, str(i))] = dict(
                **common,
                rank=i,
                epoch_step=epoch,
                epoch_time_s=epoch / 50,
                sample_count=count,
                age_ms=(n - epoch) * 20,
                ttl_remaining_ms=(750 - n + epoch) * 20,
            )
    return out, snapshots, groups


def summaries(cases, anchors):
    buckets = defaultdict(list)
    for r in anchors.values():
        for mode, op in product((r["mode"], "all_modes"), (r["loss_operation"], "all")):
            buckets[(r["partition"], r["noise"], r["variant"], mode, op)].append(r)
    result = {}
    for key in product(
        ("known", "new", "fresh"),
        ("white", "correlated"),
        ("epoch", "curved"),
        ("none", "random", "blackout", "all_modes"),
        ("all", *native.OPS),
    ):
        part, noise, arm, mode, _ = key
        cc = [
            c
            for c in cases
            if (c["partition"], c["noise"], c["variant"]) == (part, noise, arm)
            and (mode == "all_modes" or c["mode"] == mode)
        ]
        rr = buckets[key]
        counts, masses = Counter(), defaultdict(Fraction)
        ended_cases = set()
        for r in rr:
            counts["census_events"] += 1
            counts["received_events"] += r["received"]
            counts["unobserved_ttl_limit_events"] += r["ttl_limit_without_next_step"]
            for k in ("rp_records", "rp_different_preview_records"):
                counts[k] += r[k]
            for k, v in (
                ("census_case_mass", "census_case_weight"),
                ("received_case_mass", "received_case_weight"),
                ("rp_case_mass", "rp_case_mass"),
                ("rp_different_preview_case_mass", "rp_different_preview_case_mass"),
            ):
                masses[k] += r[v]
            selected = [name for name in FLAGS if r[name]]
            if r["ended"]:
                ended_cases.add(r["case_id"])
                selected += [
                    r["end_operation"] + "_end",
                    "end_" + r["end_vs_next_received"],
                ]
            for name in selected:
                counts[name + "_events"] += 1
                masses[name + "_case_mass"] += r["census_case_weight"]
        out = dict(zip(GROUP, key))
        out.update(
            planned_cases=len(cc),
            timeout_cases=sum(native.truth(c["selected_timeout"]) for c in cc),
            held_cases=sum(native.truth(c["held"]) for c in cc),
            census_cases=len({r["case_id"] for r in rr}),
            ended_cases=len(ended_cases),
        )
        for k in (
            "census_events",
            "received_events",
            "unobserved_ttl_limit_events",
            "rp_records",
            "rp_different_preview_records",
        ):
            out[k] = counts[k]
        for k in (
            "census_case_mass",
            "received_case_mass",
            "rp_case_mass",
            "rp_different_preview_case_mass",
        ):
            out[k] = masses[k]
        for name in (
            *FLAGS,
            *(op + "_end" for op in OPS),
            *("end_" + rel for rel in RELATIONS),
        ):
            out[name + "_events"], out[name + "_case_mass"] = (
                counts[name + "_events"],
                masses[name + "_case_mass"],
            )
        for op in OPS:
            n, denom = counts[op + "_end_events"], counts["ended_events"]
            out[op + "_end_fraction"] = Fraction(n, denom) if denom else None
            out[op + "_end_case_mass_fraction"] = (
                masses[op + "_end_case_mass"] / masses["ended_case_mass"]
                if denom
                else None
            )
        result[key] = out
    return result


def closures(rows):
    comparisons = 0
    for key, r in rows.items():
        for ordinal, whole, parts in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", native.OPS),
        ):
            if key[ordinal] != whole:
                continue
            children = [rows[key[:ordinal] + (p,) + key[ordinal + 1 :]] for p in parts]
            for name in r:
                if name.endswith(("_events", "_records", "_case_mass")):
                    assert sum(c[name] for c in children) == r[name], (key, name)
                    comparisons += 1
        for suffix in ("_events", "_case_mass"):
            assert (
                r["returned" + suffix] + r["no_return" + suffix]
                == r["preview_loss" + suffix]
            )
            assert (
                r["ended" + suffix] + r["censored_return" + suffix]
                == r["returned" + suffix]
            )
            assert sum(r[op + "_end" + suffix] for op in OPS) == r["ended" + suffix]
            assert (
                sum(r["end_" + rel + suffix] for rel in RELATIONS)
                == r["ended" + suffix]
            )
    return comparisons


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("results", "figures", "work", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--raw-roots", type=Path)
    parser.add_argument("--rustc", type=Path, default=shutil.which("rustc"))
    args = parser.parse_args()
    if not __debug__ or args.work.exists() or args.output.exists() or not args.rustc:
        parser.error(
            "Run without -O, with rustc, a new work directory and unused output filename"
        )
    base = RESULTS / "fusion-return-followup"
    assert (
        native.sha(base / "sources.json") == BASE_SHA
        and native.sha(base / "independent-verification.json") == PROOF_SHA
    )
    prior = json.loads((base / "sources.json").read_text())
    proof = json.loads((base / "independent-verification.json").read_text())
    source = json.loads((args.results / "sources.json").read_text())
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        "fusion-return-followup/sources.json": BASE_SHA,
        "fusion-return-followup/independent-verification.json": PROOF_SHA,
        "fusion-return-followup/protocol.md": prior["analysis"]["protocol_sha256"],
        **{"fusion-return-followup/" + k: h for k, h in prior["output_sha256"].items()},
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_fusion_return.py": prior["analysis"]["script_sha256"],
    }
    assert source["analysis"]["verifier_helpers_sha256"] == HELPERS
    assert (
        source["analysis"]["code_sha256"] == native.CODE
        and source["raw"] == prior["raw"]
    )
    assert source["inherited_validation"] == prior["inherited_validation"]
    assert source["native_trace_sha256"] == proof["native_trace_sha256"]
    assert (
        proof["sources_sha256"] == BASE_SHA
        and proof["script_sha256"] == HELPERS["verify_fusion_return.py"]
    )
    for name, h in source["input_sha256"].items():
        assert native.sha(RESULTS / name) == h, name
    for name, h in {**source["analysis"]["helpers_sha256"], **HELPERS}.items():
        assert native.sha(ROOT / name) == h, name
    assert (
        native.sha(ROOT / "analyze_return_spell.py")
        == source["analysis"]["script_sha256"]
    )
    assert (
        native.sha(RESULTS / "return-spell-mechanisms/protocol.md")
        == source["analysis"]["protocol_sha256"]
    )
    assert set(source["output_sha256"]) == {
        "return-endpoints.csv",
        "snapshots.csv",
        "group-history.csv",
        "spell-summary.csv",
    }
    for name, h in source["output_sha256"].items():
        assert native.sha(args.results / name) == h, name
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    args.work.mkdir(parents=True)
    executable, native_meta = native.compile_native(args.work, args.rustc)
    cases, events = previous.original_events()
    _, original, _, audit, trace_hashes = previous.native_reconstruction(
        args.work, executable, cases, events, source["raw"], roots
    )
    assert trace_hashes == source["native_trace_sha256"]
    previous.compare_file(base / "loss-followup.csv", original, ("event_id",))
    prior_comparisons = previous.checks
    files, by_case = defaultdict(list), defaultdict(list)
    for c in cases:
        files[
            c["cohort"] + f"/{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"
        ].append(c)
    for e in original.values():
        by_case[e["case_id"]].append(e)
    anchors, snapshots, groups = {}, {}, {}
    cached_bytes, checkpoint_count = 0, 0
    for i, name in enumerate(sorted(files), 1):
        path = args.work / f"{i:03d}-native.csv"
        cached_bytes += path.stat().st_size
        points = full_points(path, files[name])
        checkpoint_count += sum(map(len, points.values()))
        for cid, pp in points.items():
            for e in by_case[cid]:
                a, ss, gg = reconstruct(e, pp, by_case[cid])
                assert (
                    (e["event_id"],) not in anchors
                    and not set(ss) & set(snapshots)
                    and not set(gg) & set(groups)
                )
                anchors[(e["event_id"],)] = a
                snapshots.update(ss)
                groups.update(gg)
    summary = summaries(cases, anchors)
    closure_checks = closures(summary)
    for name, expected, keys in (
        ("return-endpoints", anchors, ("event_id",)),
        ("snapshots", snapshots, ("event_id", "role")),
        ("group-history", groups, ("event_id", "role", "rank")),
        ("spell-summary", summary, GROUP),
    ):
        previous.compare_file(args.results / (name + ".csv"), expected, keys)
    expected_audit = dict(
        cached_trace_files=108,
        cached_trace_bytes=cached_bytes,
        cached_late_checkpoints=checkpoint_count,
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(by_case),
        census_events=len(anchors),
        observed_loss_anchors=sum(r["preview_loss"] for r in anchors.values()),
        returned_anchors=sum(bool(r["returned"]) for r in anchors.values()),
        ended_spells=sum(bool(r["ended"]) for r in anchors.values()),
        censored_returned_spells=sum(
            bool(r["censored_return"]) for r in anchors.values()
        ),
        no_return_anchors=sum(r["no_return"] for r in anchors.values()),
        unobserved_ttl_limit_anchors=sum(
            r["ttl_limit_without_next_step"] for r in anchors.values()
        ),
        snapshots=len(snapshots),
        group_references=len(groups),
        summary_rows=len(summary),
        additive_closure_comparisons=closure_checks,
    )
    assert source["audit"] == expected_audit, (source["audit"], expected_audit)
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
    assert plots["script_sha256"] == native.sha(ROOT / "plot_return_spell.py")
    assert plots["input_sha256"] == {
        n: native.sha(args.results / n)
        for n in ("sources.json", "return-endpoints.csv", "spell-summary.csv")
    }
    assert set(plots["output_sha256"]) == {
        "spell-operations.svg",
        "spell-spans.svg",
        "plot-values.csv",
        "plot-return-values.csv",
    }
    for name, h in plots["output_sha256"].items():
        assert native.sha(args.figures / name) == h, name
    plotted_summary = {k: r for k, r in summary.items() if r["mode"] != "all_modes"}
    plotted_returns = {k: r for k, r in anchors.items() if r["returned"]}
    assert len(plotted_summary) == 396 and len(plotted_returns) == 137
    previous.compare_file(args.figures / "plot-values.csv", plotted_summary, GROUP)
    previous.compare_file(
        args.figures / "plot-return-values.csv", plotted_returns, ("event_id",)
    )
    evidence = dict(
        **expected_audit,
        comparisons=previous.checks - prior_comparisons,
        previous_anchor_comparisons=prior_comparisons,
        original_census_join_comparisons=native.comparisons,
        plotted_summary_rows=len(plotted_summary),
        plotted_return_rows=len(plotted_returns),
        fresh_raw_validation=audit,
        native=native_meta,
        native_trace_sha256=trace_hashes,
        frozen_previous_proof_sha256=PROOF_SHA,
        script_sha256=native.sha(Path(__file__)),
        sources_sha256=native.sha(args.results / "sources.json"),
        plot_sources_sha256=native.sha(args.figures / "plot-sources.json"),
        method="fresh unchanged original Rust observe and six tests; all late points independently grouped into runs; fresh traces equal pinned cache; no analyzer imports",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(
        json.dumps(
            {k: v for k, v in evidence.items() if k != "native_trace_sha256"}, indent=2
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
