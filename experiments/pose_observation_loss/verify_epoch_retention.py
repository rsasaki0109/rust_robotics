#!/usr/bin/env python3
"""Check epoch metadata using unchanged original Rust observe and tests."""

import argparse
from collections import Counter, defaultdict
import csv
from fractions import Fraction
import hashlib
from itertools import product
import json
import math
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
RESULTS = ROOT / "results"
BASE_SHA = "2c68a7619a074237496637f90580f96e5769301139a4326b3adbed62faa84f36"
CODE = {
    "crates/rust_robotics_control/examples/pose_observation_loss.rs": "81711bb44ae1741c5f0e1f71180b047284bf098066f9b1246915130b094f9228",
    "crates/rust_robotics_control/src/pure_pursuit.rs": "e6a818f32a3f396b7056a9b1d788d2f8d2b497511bad4f926614bbfc40063deb",
}
KEY = ("cohort", "variant", "mode", "seed", "scenario", "planner")
META = ("case_id", "family_id", *KEY, "partition", "accounting_partition", "noise")
GROUP = ("partition", "noise", "variant", "mode", "event_group", "horizon")
KINDS = (
    "received",
    "missed",
    "precision_only",
    "received_and_precision",
    "missed_and_precision",
)
OPS = (
    "motion_and_precision_clear",
    "motion_clear",
    "precision_clear",
    "ttl_expiry",
    "capacity_eviction",
    "age_gate",
    "latest_epoch_gate",
    "older_samples_gate",
    "other_gate",
    "no_observed_loss",
)
comparisons = 0


def sha(path):
    with path.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def index(rows, keys):
    result = {tuple(r[k] for k in keys): r for r in rows}
    assert len(result) == len(rows), "duplicate identity"
    return result


def check(value, expected):
    global comparisons
    comparisons += 1
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


def tick(value):
    t = float(value)
    n = round(t * 50)
    assert math.isfinite(t) and 0 <= n <= 9000 and abs(t - n / 50) <= 1e-7
    return n


def truth(value, raw=False):
    choices = ("false", "true") if raw else ("False", "True")
    assert value in choices
    return value == choices[1]


def ints(value):
    values = tuple(int(x) for x in value.split(";")) if value else ()
    assert all(n >= 0 for n in values) and ";".join(map(str, values)) == value
    return values


def block(source, marker):
    start = source.index(marker)
    opening = source.index("{", start)
    depth = 0
    for i in range(opening, len(source)):
        depth += (source[i] == "{") - (source[i] == "}")
        if depth == 0:
            return source[start : i + 1]
    raise AssertionError("unclosed pinned Rust source block")


def compile_native(work, rustc):
    for name, h in CODE.items():
        assert sha(REPO / name) == h, name
    source = (REPO / next(iter(CODE))).read_text()
    vehicle = (REPO / "crates/rust_robotics_control/src/pure_pursuit.rs").read_text()
    pieces = dict(
        constants="\n".join(
            next(
                line
                for line in source.splitlines()
                if line.startswith("const " + k + ":")
            )
            for k in ("DT", "WHEELBASE")
        ),
        vehicle=block(vehicle, "pub struct VehicleState"),
        vehicle_new=block(
            vehicle, "pub fn new(x: f64, y: f64, yaw: f64, v: f64, wheelbase: f64)"
        ),
        mean_pose=block(source, "fn mean_pose("),
        fusion=source[
            source.index("struct ObservationEvent {") : source.index(
                "\nimpl RestartConsistency {"
            )
        ],
        tests=block(source, "#[cfg(test)]\nmod epoch_fusion_tests"),
    )
    assert pieces["tests"].count("#[test]") == 6
    harness = (
        "#![allow(dead_code)]\nuse std::collections::VecDeque;\n"
        + pieces["constants"]
        + "\n"
        + pieces["vehicle"]
        + "\nimpl VehicleState {\n"
        + pieces["vehicle_new"]
        + "\n}\n"
        + pieces["mean_pose"]
        + "\n"
        + pieces["fusion"]
        + "\n"
        + pieces["tests"]
        + "\n"
        + (ROOT / "epoch_retention_native.rs").read_text()
    )
    generated = work / "harness.rs"
    generated.write_text(harness)
    executable = work / "epoch-retention-native"
    tests = work / "original-tests"
    flags = ["--edition=2021", "-O"]
    for output, extra in ((tests, ["--test"]), (executable, [])):
        subprocess.run(
            [str(rustc), *flags, *extra, str(generated), "-o", str(output)],
            check=True,
            capture_output=True,
            text=True,
        )
    result = subprocess.run(
        [str(tests), "--test-threads=1"], check=True, capture_output=True, text=True
    )
    (work / "original-tests.log").write_text(result.stdout + result.stderr)
    assert "test result: ok. 6 passed; 0 failed; 0 ignored;" in result.stdout
    return executable, dict(
        rustc_version=subprocess.check_output(
            [str(rustc), "--version"], text=True
        ).strip(),
        rustc_flags=flags,
        original_rust_tests_passed=6,
        source_slices_sha256={
            k: hashlib.sha256(v.encode()).hexdigest() for k, v in pieces.items()
        },
        generated_harness_sha256=sha(generated),
        native_template_sha256=sha(ROOT / "epoch_retention_native.rs"),
        geometry_inputs="zero dummy pose and steering; original logged speed/age/event flags; fused positions ignored",
    )


def original_inputs():
    cases = read(RESULTS / "event-temporal-rankings/cases.csv")
    accounts = index(read(RESULTS / "fresh-seed-mode-census/accounting.csv"), KEY)
    lookup = index(cases, KEY)
    assert len(cases) == 972 and len({c["family_id"] for c in cases}) == 324
    for c in cases:
        account = accounts[tuple(c[k] for k in KEY)]
        for k in ("status", "held", "time_s", "selected_timeout"):
            c[k] = account[k]
        assert truth(c["selected_timeout"]) == (c["status"] == "timeout")
    old = index(
        read(RESULTS / "logged-preview-transitions/event-trace.csv"), ("event_id",)
    )
    events = {}
    for original in read(RESULTS / "fresh-seed-mode-census/events.csv"):
        prefix, number = original["event_id"].split("_")
        assert prefix in ("historical", "fresh") and int(number) > 0
        eid = str(int(number) + (2583 if prefix == "fresh" else 0))
        assert eid not in events
        c = lookup[tuple(original[k] for k in KEY)]
        assert truth(c["selected_timeout"])
        e = old[(eid,)]
        for k in (
            *META,
            "event_kind",
            "event_time_s",
            "scheduled",
            "received",
            "precision_activation",
        ):
            if k in c and k in META:
                check(e[k], c[k])
            elif k in original:
                check(e[k], original[k])
        check(e["source_event_id"], original["event_id"])
        check(e["parent_rp_records"], int(c["parent_records"]))
        events[eid] = e
    assert len(events) == 3997
    return cases, events


def native_snapshots(work, executable, cases, events, raw, roots):
    files = defaultdict(list)
    queries = defaultdict(list)
    for c in cases:
        name = f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"
        files[(c["cohort"], name)].append(c)
    for eid, e in events.items():
        n = tick(e["event_time_s"])
        queries[e["case_id"]].append((eid, "immediate", n))
        if int(e["preview_changes"]):
            assert int(e["preview_changes"]) == 1
            changed = n + int(e["first_preview_change_offset_ms"]) // 20
            assert n < changed <= min(n + 75, 9000)
            queries[e["case_id"]] += [
                (eid, "loss_before", changed - 1),
                (eid, "loss_after", changed),
            ]
    assert set(files) == {
        (cohort, name) for cohort, s in raw.items() for name in s["raw_sha256"]
    }
    snapshots = {}
    audit = Counter()
    for ordinal, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert sha(path) == raw[cohort]["raw_sha256"][name]
        audit["raw_bytes"] += path.stat().st_size
        memberfile = work / f"{ordinal:03d}-cases.tsv"
        queryfile = work / f"{ordinal:03d}-queries.tsv"
        memberfile.write_text(
            "".join(
                "\t".join(
                    (
                        c["case_id"],
                        c["scenario"],
                        c["planner"],
                        str(truth(c["selected_timeout"])).lower(),
                        str(tick(c["time_s"])),
                    )
                )
                + "\n"
                for c in members
            )
        )
        queryfile.write_text(
            "".join(
                "\t".join((eid, role, c["scenario"], c["planner"], str(n))) + "\n"
                for c in members
                for eid, role, n in queries[c["case_id"]]
            )
        )
        output = work / f"{ordinal:03d}-native.csv"
        with output.open("w") as f:
            run = subprocess.run(
                [str(executable), str(path), str(memberfile), str(queryfile)],
                stdout=f,
                stderr=subprocess.PIPE,
                text=True,
            )
        assert run.returncode == 0, (ordinal, run.stderr)
        native = json.loads(run.stderr)
        audit.update(native)
        audit["raw_files"] += 1
        for r in read(output):
            eid, role = r.pop("event_id"), r.pop("role")
            assert (eid, role) not in snapshots and r.pop("case_id") == events[eid][
                "case_id"
            ]
            parsed = {}
            for k, v in r.items():
                if k in (
                    "received",
                    "precision_activation",
                    "moving",
                    "fusion_available",
                ):
                    parsed[k] = truth(v, True)
                elif k == "retained_groups":
                    parsed[k] = int(v)
                elif k.endswith("_steps") or k in (
                    "sample_counts",
                    "prior_sample_counts",
                ):
                    ints(v)
                    parsed[k] = v
                else:
                    parsed[k] = float(v) if v else None
            snapshots[(eid, role)] = parsed
        if ordinal % 12 == 0:
            print(f"Original Rust checked: {ordinal}/108 raw files", flush=True)
    assert (
        len(snapshots) == 4301
        and audit["snapshot_rows"] == 4301
        and audit["verified_decisions"] == 5130000
    )
    return snapshots, dict(audit)


def event_evidence(events, snapshots):
    ne = Counter(e["case_id"] for e in events.values())
    nr = Counter(e["case_id"] for e in events.values() if truth(e["received"]))
    output, ss, gg = {}, {}, {}
    for eid, e in events.items():
        meta = {k: e[k] for k in (*META, "event_id", "source_event_id")}
        cid, origin = e["case_id"], tick(e["event_time_s"])
        here = snapshots[(eid, "immediate")]
        assert here["fusion_available"] == (e["immediate_preview"] == "three_epoch")
        loss = bool(int(e["preview_changes"]))
        after = snapshots[(eid, "loss_after")] if loss else None
        for role in (
            ("immediate", "loss_before", "loss_after") if loss else ("immediate",)
        ):
            s = snapshots[(eid, role)]
            common = dict(
                **meta,
                role=role,
                time_s=s["time_s"],
                offset_ms=(tick(s["time_s"]) - origin) * 20,
            )
            ss[(eid, role)] = {**common, **s}
            epochs, counts = ints(s["epoch_steps"]), ints(s["sample_counts"])
            assert len(epochs) == len(counts) == s["retained_groups"] and all(
                1 <= c <= 25 for c in counts
            )
            for rank, (epoch, count) in enumerate(zip(epochs, counts), 1):
                age = (tick(s["time_s"]) - epoch) * 20
                gg[(eid, role, str(rank))] = dict(
                    **common,
                    rank=rank,
                    epoch_step=epoch,
                    epoch_time_s=epoch / 50,
                    sample_count=count,
                    age_ms=age,
                    ttl_remaining_ms=15000 - age,
                )
        if loss:
            before = snapshots[(eid, "loss_before")]
            assert before["fusion_available"] and not after["fusion_available"]
            assert tick(before["time_s"]) + 1 == tick(after["time_s"])
            check(e["first_preview_change_speed"], after["measured_speed"])
            check(e["first_preview_change_observation_age"], after["observation_age"])
            check(e["first_preview_change_retained_groups"], after["retained_groups"])
            assert (tick(after["time_s"]) - origin) * 20 == int(
                e["first_preview_change_offset_ms"]
            )
        epochs = ints(here["epoch_steps"])
        ttl = 15000 - (origin - epochs[0]) * 20 if epochs else None
        reason = "no_observed_loss"
        if loss:
            if after["moving"] and after["precision_activation"]:
                reason = "motion_and_precision_clear"
            elif after["moving"]:
                reason = "motion_clear"
            elif after["precision_activation"]:
                reason = "precision_clear"
            elif ints(after["expired_epoch_steps"]):
                reason = "ttl_expiry"
            elif ints(after["capacity_evicted_epoch_steps"]):
                reason = "capacity_eviction"
            elif after["observation_age"] > 1.5 + 1e-9:
                reason = "age_gate"
            elif len(ints(after["epoch_steps"])) == 3 and ints(after["epoch_steps"])[
                -1
            ] != tick(after["current_epoch_time_s"]):
                reason = "latest_epoch_gate"
            elif len(ints(after["epoch_steps"])) == 3 and any(
                c < 10 for c in ints(after["sample_counts"])[:2]
            ):
                reason = "older_samples_gate"
            else:
                reason = "other_gate"
        parent = int(e["parent_rp_records"])
        has_forward = int(e["forward_steps"]) > 0
        assert has_forward == (e["horizon"] != "no_forward")
        out = dict(
            **meta,
            event_kind=e["event_kind"],
            event_time_s=origin / 50,
            scheduled=truth(e["scheduled"]),
            received=truth(e["received"]),
            precision_activation=truth(e["precision_activation"]),
            horizon=e["horizon"],
            has_forward_sample=has_forward,
            immediate_actual=e["immediate_actual"],
            immediate_preview=e["immediate_preview"],
            immediate_groups=here["retained_groups"],
            immediate_oldest_epoch_step=epochs[0] if epochs else None,
            immediate_oldest_epoch_time_s=epochs[0] / 50 if epochs else None,
            immediate_oldest_age_ms=15000 - ttl if ttl is not None else None,
            immediate_oldest_ttl_ms=ttl,
            immediate_at_ttl_limit=here["fusion_available"] and ttl == 0,
            preview_loss=loss,
            loss_offset_ms=int(e["first_preview_change_offset_ms"]) if loss else None,
            loss_operation=reason,
            next_step_loss=(
                (loss and int(e["first_preview_change_offset_ms"]) == 20)
                if has_forward
                else None
            ),
            loss_motion_clear=after["moving"] if loss else None,
            loss_precision_clear=after["precision_activation"] if loss else None,
            loss_expired_groups=(
                len(ints(after["expired_epoch_steps"])) if loss else None
            ),
            loss_capacity_evicted_groups=(
                len(ints(after["capacity_evicted_epoch_steps"])) if loss else None
            ),
            loss_age_gate=after["observation_age"] > 1.5 + 1e-9 if loss else None,
            loss_group_count_gate=after["retained_groups"] != 3 if loss else None,
            loss_latest_epoch_gate=(
                (
                    not ints(after["epoch_steps"])
                    or ints(after["epoch_steps"])[-1]
                    != tick(after["current_epoch_time_s"])
                )
                if loss
                else None
            ),
            loss_older_sample_gate=(
                any(c < 10 for c in ints(after["sample_counts"])[:2]) if loss else None
            ),
            census_case_weight=Fraction(1, ne[cid]),
            received_case_weight=(
                Fraction(1, nr[cid]) if truth(e["received"]) else Fraction()
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
        output[(eid,)] = out
    return output, ss, gg


def summary_rows(cases, events):
    result = {}
    for part, noise, arm, mode, kind, horizon in product(
        ("known", "new", "fresh"),
        ("white", "correlated"),
        ("epoch", "curved"),
        ("none", "random", "blackout", "all_modes"),
        ("all", "received_any", *KINDS),
        ("all", "full", "clipped", "no_forward"),
    ):
        cc = [
            c
            for c in cases
            if (c["partition"], c["noise"], c["variant"]) == (part, noise, arm)
            and (mode == "all_modes" or c["mode"] == mode)
        ]
        rr = [
            e
            for e in events.values()
            if (e["partition"], e["noise"], e["variant"]) == (part, noise, arm)
            and (mode == "all_modes" or e["mode"] == mode)
            and (
                kind == "all"
                or (
                    e["received"] if kind == "received_any" else e["event_kind"] == kind
                )
            )
            and (horizon == "all" or e["horizon"] == horizon)
        ]
        f = [e for e in rr if e["immediate_preview"] == "three_epoch"]
        at = [e for e in f if e["immediate_at_ttl_limit"]]
        eligible = [e for e in f if e["has_forward_sample"]]
        risk = [e for e in eligible if e["immediate_at_ttl_limit"]]
        other = [e for e in eligible if not e["immediate_at_ttl_limit"]]
        riskloss = [e for e in risk if e["next_step_loss"]]
        otherloss = [e for e in other if e["next_step_loss"]]
        mass = lambda seq, key="census_case_weight": sum(
            (e[key] for e in seq), Fraction()
        )
        out = dict(zip(GROUP, (part, noise, arm, mode, kind, horizon)))
        out.update(
            planned_cases=len(cc),
            timeout_cases=sum(truth(c["selected_timeout"]) for c in cc),
            held_cases=sum(truth(c["held"]) for c in cc),
            census_cases=len({e["case_id"] for e in rr}),
            census_events=len(rr),
            received_events=sum(e["received"] for e in rr),
            fusion_events=len(f),
            ttl_limit_events=len(at),
            observed_loss_events=sum(e["preview_loss"] for e in rr),
            eligible_fusion_events=len(eligible),
            eligible_ttl_limit_events=len(risk),
            eligible_other_fusion_events=len(other),
            one_step_ttl_limit_loss_events=len(riskloss),
            one_step_other_fusion_loss_events=len(otherloss),
            census_case_mass=mass(rr),
            received_case_mass=mass(rr, "received_case_weight"),
            fusion_case_mass=mass(f),
            ttl_limit_case_mass=mass(at),
            eligible_ttl_limit_case_mass=mass(risk),
            eligible_other_fusion_case_mass=mass(other),
            one_step_ttl_limit_loss_case_mass=mass(riskloss),
            one_step_other_fusion_loss_case_mass=mass(otherloss),
            rp_records=sum(e["rp_records"] for e in rr),
            rp_case_mass=mass(rr, "rp_case_mass"),
            rp_different_preview_records=sum(
                e["rp_different_preview_records"] for e in rr
            ),
            rp_different_preview_case_mass=mass(rr, "rp_different_preview_case_mass"),
        )
        for op in OPS:
            selected = [e for e in rr if e["loss_operation"] == op]
            out[op + "_events"] = len(selected)
            out[op + "_case_mass"] = mass(selected)
            out["rp_" + op + "_records"] = sum(e["rp_records"] for e in selected)
            out["rp_" + op + "_case_mass"] = mass(selected, "rp_case_mass")
        for label, den, num in (
            ("ttl_limit", risk, riskloss),
            ("other_fusion", other, otherloss),
        ):
            out[label + "_one_step_loss_fraction"] = (
                Fraction(len(num), len(den)) if den else None
            )
            out[label + "_one_step_loss_case_mass_fraction"] = (
                mass(num) / mass(den) if den else None
            )
        result[tuple(out[k] for k in GROUP)] = out
    return result


def closures(rows):
    additive = [
        k
        for k, v in next(iter(rows.values())).items()
        if isinstance(v, (int, Fraction))
        and k not in ("planned_cases", "timeout_cases", "held_cases", "census_cases")
        and "fraction" not in k
    ]
    checks = 0
    for r in rows.values():
        for field, whole, pieces in (
            ("mode", "all_modes", ("none", "random", "blackout")),
            ("event_group", "all", KINDS),
            ("horizon", "all", ("full", "clipped", "no_forward")),
        ):
            if r[field] != whole:
                continue
            children = [
                rows[tuple(piece if k == field else r[k] for k in GROUP)]
                for piece in pieces
            ]
            for k in additive:
                assert sum(c[k] for c in children) == r[k], (field, k)
                checks += 1
    return checks


def compare(path, expected, keys):
    rows = index(read(path), keys)
    assert set(rows) == set(expected), (path, len(rows), len(expected))
    for key, row in rows.items():
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
    path = RESULTS / "logged-preview-transitions/sources.json"
    assert sha(path) == BASE_SHA
    prior = json.loads(path.read_text())
    assert source["input_sha256"] == {
        **prior["input_sha256"],
        "logged-preview-transitions/sources.json": BASE_SHA,
        **{
            "logged-preview-transitions/" + k: h
            for k, h in prior["output_sha256"].items()
        },
    }
    assert source["analysis"]["helpers_sha256"] == {
        **prior["analysis"]["helpers_sha256"],
        "analyze_logged_preview_transitions.py": prior["analysis"]["script_sha256"],
    }
    assert (
        source["analysis"]["code_sha256"] == CODE
        and source["raw"] == prior["raw"]
        and source["inherited_validation"] == prior["inherited_validation"]
    )
    for name, h in source["input_sha256"].items():
        assert sha(RESULTS / name) == h, name
    for name, h in source["analysis"]["helpers_sha256"].items():
        assert sha(ROOT / name) == h, name
    assert (
        sha(ROOT / "analyze_epoch_retention.py") == source["analysis"]["script_sha256"]
    )
    assert (
        sha(RESULTS / "epoch-retention-mechanisms/protocol.md")
        == source["analysis"]["protocol_sha256"]
    )
    assert set(source["output_sha256"]) == {
        "event-mechanisms.csv",
        "snapshots.csv",
        "group-history.csv",
        "retention-summary.csv",
    }
    for name, h in source["output_sha256"].items():
        assert sha(args.results / name) == h, name
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    args.work.mkdir(parents=True)
    executable, native = compile_native(args.work, args.rustc)
    cases, events = original_inputs()
    snapshots, audit = native_snapshots(
        args.work, executable, cases, events, source["raw"], roots
    )
    ee, ss, gg = event_evidence(events, snapshots)
    summary = summary_rows(cases, ee)
    checks = closures(summary)
    for name, rows, keys in (
        ("event-mechanisms", ee, ("event_id",)),
        ("snapshots", ss, ("event_id", "role")),
        ("group-history", gg, ("event_id", "role", "rank")),
        ("retention-summary", summary, GROUP),
    ):
        compare(args.results / (name + ".csv"), rows, keys)
    plots = json.loads((args.figures / "plot-sources.json").read_text())
    assert plots["script_sha256"] == sha(ROOT / "plot_epoch_retention.py")
    assert plots["input_sha256"] == {
        n: sha(args.results / n) for n in ("sources.json", "retention-summary.csv")
    }
    assert set(plots["output_sha256"]) == {"loss-mechanisms.svg", "plot-values.csv"}
    for name, h in plots["output_sha256"].items():
        assert sha(args.figures / name) == h, name
    selected = {
        k: r
        for k, r in summary.items()
        if r["mode"] != "all_modes" and r["event_group"] == "all"
    }
    assert len(selected) == 144
    compare(args.figures / "plot-values.csv", selected, GROUP)
    expected = dict(
        planned_cases=972,
        planned_families=324,
        timeout_cases=570,
        census_events=len(ee),
        snapshot_rows=len(ss),
        group_rows=len(gg),
        summary_rows=len(summary),
        additive_closure_comparisons=checks,
    )
    audit.update(expected)
    assert audit == source["audit"], (audit, source["audit"])
    audit.update(
        comparisons=comparisons,
        plotted_rows=144,
        native=native,
        script_sha256=sha(Path(__file__)),
        sources_sha256=sha(args.results / "sources.json"),
        plot_sources_sha256=sha(args.figures / "plot-sources.json"),
        method="original Rust observe/types/mean_pose and six tests unchanged; native independent raw CSV reads; Python expected-field and exact-parent comparisons; no analyzer imports",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(audit, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
