#!/usr/bin/env python3
"""Reconstruct original epoch metadata and explain frozen preview losses."""

import argparse
from collections import Counter, defaultdict
import csv
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path

from analyze_logged_preview_transitions import (
    KEY,
    META,
    digest,
    flag,
    indexed,
    integer,
    read_rows,
    step,
)
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
RESULTS = ROOT / "results"
BASE = RESULTS / "logged-preview-transitions"
BASE_SHA = "2c68a7619a074237496637f90580f96e5769301139a4326b3adbed62faa84f36"
PROTOCOL = RESULTS / "epoch-retention-mechanisms/protocol.md"
SIMULATOR = "crates/rust_robotics_control/examples/pose_observation_loss.rs"
VEHICLE = "crates/rust_robotics_control/src/pure_pursuit.rs"
SIMULATOR_SHA = "81711bb44ae1741c5f0e1f71180b047284bf098066f9b1246915130b094f9228"
VEHICLE_SHA = "e6a818f32a3f396b7056a9b1d788d2f8d2b497511bad4f926614bbfc40063deb"
OPERATIONS = (
    "motion_and_precision_clear",
    "motion_clear",
    "precision_clear",
    "ttl_expiry",
    "capacity_eviction",
    "age_gate",
    "latest_epoch_gate",
    "older_samples_gate",
    "other_gate",
)
ALL_OPS = (*OPERATIONS, "no_observed_loss")
KINDS = (
    "received",
    "missed",
    "precision_only",
    "received_and_precision",
    "missed_and_precision",
)
GROUP = ("partition", "noise", "variant", "mode", "event_group", "horizon")
COUNTS = (
    "census_events",
    "received_events",
    "fusion_events",
    "ttl_limit_events",
    "observed_loss_events",
    "eligible_fusion_events",
    "eligible_ttl_limit_events",
    "eligible_other_fusion_events",
    "one_step_ttl_limit_loss_events",
    "one_step_other_fusion_loss_events",
    "rp_records",
    "rp_different_preview_records",
    *(op + "_events" for op in ALL_OPS),
    *("rp_" + op + "_records" for op in ALL_OPS),
)
MASSES = (
    "census_case_mass",
    "received_case_mass",
    "fusion_case_mass",
    "ttl_limit_case_mass",
    "eligible_ttl_limit_case_mass",
    "eligible_other_fusion_case_mass",
    "one_step_ttl_limit_loss_case_mass",
    "one_step_other_fusion_loss_case_mass",
    "rp_case_mass",
    "rp_different_preview_case_mass",
    *(op + "_case_mass" for op in ALL_OPS),
    *("rp_" + op + "_case_mass" for op in ALL_OPS),
)


class Metadata:
    """Integer-grid group identities/counts; no pose or gate selection replay."""

    def __init__(self):
        self.groups, self.epoch, self.last = [], None, 0

    def advance(self, n, speed, age, received, precision):
        assert (
            n == self.last + 1
            and math.isfinite(speed)
            and math.isfinite(age)
            and age >= 0
        )
        assert isinstance(received, bool) and isinstance(precision, bool)
        self.last = n
        prior = tuple(self.groups)
        moving = abs(speed) > 0.02
        cleared = tuple(g[0] for g in prior) if moving or precision else ()
        if moving or precision:
            self.groups = []
        if received or precision:
            self.epoch = n
        expired = tuple(g[0] for g in self.groups if n - g[0] > 750)
        self.groups = [g for g in self.groups if n - g[0] <= 750]
        capacity = ()
        if not moving and self.epoch is not None and 0 <= age <= 0.5 + 1e-9:
            if not self.groups or self.groups[-1][0] != self.epoch:
                self.groups.append((self.epoch, 0))
                if len(self.groups) > 3:
                    capacity = (self.groups.pop(0)[0],)
            epoch, count = self.groups[-1]
            self.groups[-1] = epoch, min(25, count + 1)
        available = (
            not moving
            and age <= 1.5 + 1e-9
            and len(self.groups) == 3
            and self.groups[-1][0] == self.epoch
            and all(c >= 10 for _, c in self.groups[:2])
        )
        encode = lambda seq: ";".join(map(str, seq))
        return dict(
            time_s=n / 50,
            current_epoch_time_s=self.epoch / 50 if self.epoch is not None else None,
            measured_speed=speed,
            observation_age=age,
            received=received,
            precision_activation=precision,
            moving=moving,
            fusion_available=available,
            retained_groups=len(self.groups),
            epoch_steps=encode(g[0] for g in self.groups),
            sample_counts=encode(g[1] for g in self.groups),
            prior_epoch_steps=encode(g[0] for g in prior),
            prior_sample_counts=encode(g[1] for g in prior),
            cleared_epoch_steps=encode(cleared),
            expired_epoch_steps=encode(expired),
            capacity_evicted_epoch_steps=encode(capacity),
        )


def numbers(value):
    return tuple(map(int, value.split(";"))) if value else ()


def requests(events):
    result = defaultdict(lambda: defaultdict(list))
    indexed(events, ("event_id",))
    for e in events:
        origin = step(e["event_time_s"])
        roles = [("immediate", origin)]
        if integer(e["preview_changes"]):
            assert (
                integer(e["preview_changes"]) == 1
                and e["immediate_preview"] == "three_epoch"
            )
            offset = integer(e["first_preview_change_offset_ms"])
            assert (
                20 <= offset <= 1500
                and offset % 20 == 0
                and origin + offset // 20 <= 9000
            )
            roles += [
                ("loss_before", origin + offset // 20 - 1),
                ("loss_after", origin + offset // 20),
            ]
        for role, n in roles:
            result[e["case_id"]][n].append((e["event_id"], role))
    return result


def collect(cases, events, raw, roots):
    wanted = requests(events)
    files = defaultdict(dict)
    for c in cases:
        name = f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"
        files[(c["cohort"], name)][(c["scenario"], c["planner"])] = c
    assert set(files) == {
        (cohort, name) for cohort, s in raw.items() for name in s["raw_sha256"]
    }
    snapshots, decisions, gridrows, byte_count = {}, 0, 0, 0
    for ordinal, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert digest(path) == raw[cohort]["raw_sha256"][name]
        byte_count += path.stat().st_size
        states = {
            c["case_id"]: Metadata()
            for c in members.values()
            if flag(c["selected_timeout"])
        }
        last = {}
        with path.open(newline="") as f:
            rows = csv.reader(f)
            header = next(rows)
            assert len(set(header)) == len(header)
            pos = {
                k: header.index(k)
                for k in (
                    "scenario",
                    "planner",
                    "time_s",
                    "measured_speed",
                    "observation_age",
                    "correction_received",
                    "precision_activation",
                    "epoch_fusion_available",
                    "epoch_fusion_group_count",
                    "stationary_epoch_fusion",
                )
            }
            for row in rows:
                identity = row[pos["scenario"]], row[pos["planner"]]
                c = members[identity]
                n = step(row[pos["time_s"]])
                assert (
                    n == last.get(identity, -1) + 1
                ), "duplicate, missing or unordered raw checkpoint"
                last[identity] = n
                cid = c["case_id"]
                if cid not in states:
                    continue
                gridrows += 1
                if n == 0:
                    continue
                assert flag(row[pos["stationary_epoch_fusion"]], raw=True)
                snapshot = states[cid].advance(
                    n,
                    float(row[pos["measured_speed"]]),
                    float(row[pos["observation_age"]]),
                    flag(row[pos["correction_received"]], raw=True),
                    flag(row[pos["precision_activation"]], raw=True),
                )
                assert snapshot["fusion_available"] == flag(
                    row[pos["epoch_fusion_available"]], raw=True
                ), (cid, n, "availability")
                assert snapshot["retained_groups"] == integer(
                    row[pos["epoch_fusion_group_count"]]
                ), (cid, n, "group count")
                decisions += 1
                for key in wanted[cid].get(n, ()):
                    assert key not in snapshots
                    snapshots[key] = snapshot.copy()
        assert set(last) == set(members)
        for identity, c in members.items():
            assert last[identity] == (
                9000 if c["case_id"] in states else step(c["time_s"])
            )
        if ordinal % 12 == 0:
            print(f"Reconstructed original histories: {ordinal}/108 files", flush=True)
    assert len(snapshots) == sum(
        len(keys) for times in wanted.values() for keys in times.values()
    )
    return snapshots, dict(
        raw_files=len(files),
        raw_bytes=byte_count,
        timeout_grid_rows=gridrows,
        verified_decisions=decisions,
    )


def group_rows(common, s):
    epochs, counts = numbers(s["epoch_steps"]), numbers(s["sample_counts"])
    assert len(epochs) == len(counts) == s["retained_groups"]
    n = step(s["time_s"])
    return [
        dict(
            **common,
            rank=i,
            epoch_step=e,
            epoch_time_s=e / 50,
            sample_count=c,
            age_ms=(n - e) * 20,
            ttl_remaining_ms=15000 - (n - e) * 20,
        )
        for i, (e, c) in enumerate(zip(epochs, counts), 1)
    ]


def operation(after):
    if after["moving"]:
        return (
            "motion_and_precision_clear"
            if after["precision_activation"]
            else "motion_clear"
        )
    if after["precision_activation"]:
        return "precision_clear"
    if after["expired_epoch_steps"]:
        return "ttl_expiry"
    if after["capacity_evicted_epoch_steps"]:
        return "capacity_eviction"
    if after["observation_age"] > 1.5 + 1e-9:
        return "age_gate"
    epochs, counts = numbers(after["epoch_steps"]), numbers(after["sample_counts"])
    if len(epochs) == 3 and epochs[-1] != step(after["current_epoch_time_s"]):
        return "latest_epoch_gate"
    if len(epochs) == 3 and any(c < 10 for c in counts[:2]):
        return "older_samples_gate"
    return "other_gate"


def build(events, snapshots):
    ne = Counter(e["case_id"] for e in events)
    nr = Counter(e["case_id"] for e in events if flag(e["received"]))
    output, ss, gg = [], [], []
    for e in events:
        eid, cid = e["event_id"], e["case_id"]
        meta = {k: e[k] for k in (*META, "event_id", "source_event_id")}
        immediate = snapshots[(eid, "immediate")]
        assert immediate["fusion_available"] == (
            e["immediate_preview"] == "three_epoch"
        )
        assert immediate["received"] == flag(e["received"]) and immediate[
            "precision_activation"
        ] == flag(e["precision_activation"])
        epochs = numbers(immediate["epoch_steps"])
        ttl = 15000 - (step(e["event_time_s"]) - epochs[0]) * 20 if epochs else None
        limit = immediate["fusion_available"] and ttl == 0
        loss = bool(integer(e["preview_changes"]))
        after = snapshots[(eid, "loss_after")] if loss else None
        if loss:
            before = snapshots[(eid, "loss_before")]
            assert before["fusion_available"] and not after["fusion_available"]
            assert step(after["time_s"]) == step(before["time_s"]) + 1
            assert (step(after["time_s"]) - step(e["event_time_s"])) * 20 == integer(
                e["first_preview_change_offset_ms"]
            )
            assert math.isclose(
                after["measured_speed"],
                float(e["first_preview_change_speed"]),
                rel_tol=1e-10,
                abs_tol=1e-100,
            )
            assert after["observation_age"] == float(
                e["first_preview_change_observation_age"]
            )
            assert after["retained_groups"] == integer(
                e["first_preview_change_retained_groups"]
            )
        for role in (
            ("immediate", "loss_before", "loss_after") if loss else ("immediate",)
        ):
            s = snapshots[(eid, role)]
            common = dict(
                **meta,
                role=role,
                time_s=s["time_s"],
                offset_ms=(step(s["time_s"]) - step(e["event_time_s"])) * 20,
            )
            ss.append({**common, **s})
            gg.extend(group_rows(common, s))
        parent = integer(e["parent_rp_records"])
        weight, received_weight = Fraction(1, ne[cid]), (
            Fraction(1, nr[cid]) if flag(e["received"]) else Fraction()
        )
        assert math.isclose(
            float(weight), float(e["census_case_weight"]), abs_tol=1e-12
        )
        assert math.isclose(
            float(received_weight), float(e["received_case_weight"]), abs_tol=1e-12
        )
        forward = integer(e["forward_steps"]) > 0
        out = dict(
            **meta,
            event_kind=e["event_kind"],
            event_time_s=float(e["event_time_s"]),
            scheduled=flag(e["scheduled"]),
            received=flag(e["received"]),
            precision_activation=flag(e["precision_activation"]),
            horizon=e["horizon"],
            has_forward_sample=forward,
            immediate_actual=e["immediate_actual"],
            immediate_preview=e["immediate_preview"],
            immediate_groups=immediate["retained_groups"],
            immediate_oldest_epoch_step=epochs[0] if epochs else None,
            immediate_oldest_epoch_time_s=epochs[0] / 50 if epochs else None,
            immediate_oldest_age_ms=15000 - ttl if ttl is not None else None,
            immediate_oldest_ttl_ms=ttl,
            immediate_at_ttl_limit=limit,
            preview_loss=loss,
            loss_offset_ms=(
                integer(e["first_preview_change_offset_ms"]) if loss else None
            ),
            loss_operation=operation(after) if loss else "no_observed_loss",
            next_step_loss=(
                (loss and integer(e["first_preview_change_offset_ms"]) == 20)
                if forward
                else None
            ),
            loss_motion_clear=after["moving"] if loss else None,
            loss_precision_clear=after["precision_activation"] if loss else None,
            loss_expired_groups=(
                len(numbers(after["expired_epoch_steps"])) if loss else None
            ),
            loss_capacity_evicted_groups=(
                len(numbers(after["capacity_evicted_epoch_steps"])) if loss else None
            ),
            loss_age_gate=after["observation_age"] > 1.5 + 1e-9 if loss else None,
            loss_group_count_gate=after["retained_groups"] != 3 if loss else None,
            loss_latest_epoch_gate=(
                (
                    not numbers(after["epoch_steps"])
                    or numbers(after["epoch_steps"])[-1]
                    != step(after["current_epoch_time_s"])
                )
                if loss
                else None
            ),
            loss_older_sample_gate=(
                any(c < 10 for c in numbers(after["sample_counts"])[:2])
                if loss
                else None
            ),
            census_case_weight=weight,
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
        output.append(out)
    return output, ss, gg


def summarize(cases, events, meta):
    fusion = [e for e in events if e["immediate_preview"] == "three_epoch"]
    limit = [e for e in fusion if e["immediate_at_ttl_limit"]]
    eligible = [e for e in fusion if e["has_forward_sample"]]
    risk = [e for e in eligible if e["immediate_at_ttl_limit"]]
    other = [e for e in eligible if not e["immediate_at_ttl_limit"]]
    lostrisk, lostother = (
        [e for e in rr if e["next_step_loss"]] for rr in (risk, other)
    )
    mass = lambda rows, key="census_case_weight": sum(
        (e[key] for e in rows), Fraction()
    )
    out = dict(
        **meta,
        planned_cases=len(cases),
        timeout_cases=sum(flag(c["selected_timeout"]) for c in cases),
        held_cases=sum(flag(c["held"]) for c in cases),
        census_cases=len({e["case_id"] for e in events}),
        census_events=len(events),
        received_events=sum(e["received"] for e in events),
        fusion_events=len(fusion),
        ttl_limit_events=len(limit),
        observed_loss_events=sum(e["preview_loss"] for e in events),
        eligible_fusion_events=len(eligible),
        eligible_ttl_limit_events=len(risk),
        eligible_other_fusion_events=len(other),
        one_step_ttl_limit_loss_events=len(lostrisk),
        one_step_other_fusion_loss_events=len(lostother),
        census_case_mass=mass(events),
        received_case_mass=mass(events, "received_case_weight"),
        fusion_case_mass=mass(fusion),
        ttl_limit_case_mass=mass(limit),
        eligible_ttl_limit_case_mass=mass(risk),
        eligible_other_fusion_case_mass=mass(other),
        one_step_ttl_limit_loss_case_mass=mass(lostrisk),
        one_step_other_fusion_loss_case_mass=mass(lostother),
        rp_records=sum(e["rp_records"] for e in events),
        rp_case_mass=mass(events, "rp_case_mass"),
        rp_different_preview_records=sum(
            e["rp_different_preview_records"] for e in events
        ),
        rp_different_preview_case_mass=mass(events, "rp_different_preview_case_mass"),
    )
    for op in ALL_OPS:
        selected = [e for e in events if e["loss_operation"] == op]
        out[op + "_events"] = len(selected)
        out[op + "_case_mass"] = mass(selected)
        out["rp_" + op + "_records"] = sum(e["rp_records"] for e in selected)
        out["rp_" + op + "_case_mass"] = mass(selected, "rp_case_mass")
    for label, den, num in (
        ("ttl_limit", risk, lostrisk),
        ("other_fusion", other, lostother),
    ):
        out[label + "_one_step_loss_fraction"] = (
            Fraction(len(num), len(den)) if den else None
        )
        out[label + "_one_step_loss_case_mass_fraction"] = (
            mass(num) / mass(den) if den else None
        )
    return out


def aggregate(cases, events):
    rows = []
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
        cc, ee = [c for c in cases if accepts(c)], [e for e in events if accepts(e)]
        for kind, horizon in product(
            ("all", "received_any", *KINDS), ("all", "full", "clipped", "no_forward")
        ):
            selected = [
                e
                for e in ee
                if (
                    kind == "all"
                    or (
                        e["received"]
                        if kind == "received_any"
                        else e["event_kind"] == kind
                    )
                )
                and (horizon == "all" or e["horizon"] == horizon)
            ]
            rows.append(
                summarize(
                    cc,
                    selected,
                    dict(zip(GROUP, (part, noise, arm, mode, kind, horizon))),
                )
            )
    return rows


def closure(rows):
    lookup = indexed(rows, GROUP)
    checks = 0
    for r in rows:
        for field, whole, pieces in (
            ("mode", "all_modes", ("none", "random", "blackout")),
            ("event_group", "all", KINDS),
            ("horizon", "all", ("full", "clipped", "no_forward")),
        ):
            if r[field] != whole:
                continue
            children = [
                lookup[tuple(v if k == field else r[k] for k in GROUP)] for v in pieces
            ]
            for name in (*COUNTS, *MASSES):
                assert sum(c[name] for c in children) == r[name], (field, name)
                checks += 1
        assert sum(r[op + "_events"] for op in ALL_OPS) == r["census_events"]
        assert sum(r[op + "_case_mass"] for op in ALL_OPS) == r["census_case_mass"]
        assert sum(r["rp_" + op + "_records"] for op in ALL_OPS) == r["rp_records"]
        assert sum(r["rp_" + op + "_case_mass"] for op in ALL_OPS) == r["rp_case_mass"]
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
        "logged-preview-transitions/sources.json": BASE_SHA,
        **{
            "logged-preview-transitions/" + k: h
            for k, h in prior["output_sha256"].items()
        },
    }
    helpers = {
        **prior["analysis"]["helpers_sha256"],
        "analyze_logged_preview_transitions.py": prior["analysis"]["script_sha256"],
    }
    for name, h in pins.items():
        assert digest(RESULTS / name) == h, name
    for name, h in helpers.items():
        assert digest(ROOT / name) == h, name
    assert digest(REPO / SIMULATOR) == SIMULATOR_SHA
    assert digest(REPO / VEHICLE) == VEHICLE_SHA
    code = {name: digest(REPO / name) for name in (SIMULATOR, VEHICLE)}
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(prior["raw"])
    cases = read_rows(RESULTS / "census-rp-selection/case-selection.csv")
    events = read_rows(BASE / "event-trace.csv")
    snapshots, audit = collect(cases, events, prior["raw"], roots)
    ee, ss, gg = build(events, snapshots)
    summary = aggregate(cases, ee)
    checks = closure(summary)
    assert len(cases) == 972 and len({c["family_id"] for c in cases}) == 324
    assert len(ee) == 3997 and len(ss) == 4301 and len(summary) == 1344
    assert (
        audit["verified_decisions"] == 5130000 and audit["timeout_grid_rows"] == 5130570
    )
    assert (
        sum(e["census_case_weight"] for e in ee)
        == sum(e["received_case_weight"] for e in ee)
        == 570
    )
    assert (
        sum(e["rp_records"] for e in ee) == 10995
        and sum(e["rp_case_mass"] for e in ee) == 157
    )
    audit.update(
        planned_cases=972,
        planned_families=324,
        timeout_cases=570,
        census_events=len(ee),
        snapshot_rows=len(ss),
        group_rows=len(gg),
        summary_rows=len(summary),
        additive_closure_comparisons=checks,
    )
    args.output.mkdir(parents=True)
    for name, rows in (
        ("event-mechanisms", ee),
        ("snapshots", ss),
        ("group-history", gg),
        ("retention-summary", summary),
    ):
        save_rows(
            args.output / (name + ".csv"),
            [
                {k: float(v) if isinstance(v, Fraction) else v for k, v in r.items()}
                for r in rows
            ],
        )
    meta = dict(
        input_sha256=pins,
        raw=prior["raw"],
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
            code_sha256=code,
        ),
        schema=dict(
            loss_operations=ALL_OPS,
            snapshot_roles=("immediate", "loss_before", "loss_after"),
            epoch_ttl_ms=15000,
            step_ms=20,
            metadata_only=True,
            weighting="original all-census, received-census and RP parent measures before filtering; exact sums before division",
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
