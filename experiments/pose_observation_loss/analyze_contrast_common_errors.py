#!/usr/bin/env python3
"""Decompose pair midpoints into a contrast projection and common residual."""

import argparse
from collections import defaultdict
from itertools import product
import json
import math
from pathlib import Path
import statistics

from analyze_commanded_recovery_arc import flag
from analyze_epoch_position_components import PARTS
from analyze_epoch_preview import digest, read_rows
from analyze_joint_admission_geometry import KEY
from analyze_speed_probe_shared_information import BINS, PAIRINGS, category
from analyze_terminal_arc_live import COHORTS, SIMULATOR
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PROTOCOL = ROOT / "results/contrast-common-errors/protocol.md"
EPS = 1e-12
PHASES = ["immediate", "after_0p18", "after_0p5", "after_1p5"]
INPUTS = ["actual", "preview"]
KINDS = [
    "received",
    "missed",
    "precision_only",
    "received_and_precision",
    "missed_and_precision",
]
POPULATIONS = ["all_states", "raw_zero_loss", "command_only_zero_loss"]
ACCURACY = [
    "both_inside",
    "inside_to_outside",
    "outside_to_inside",
    "both_outside",
    "unavailable",
]
ESTIMATORS = [
    "raw_observation",
    "recovery_mean",
    "raw_fallback",
    "three_epoch",
    "motion_filter",
    "unavailable",
]
PINNED = {
    "speed-probe-shared-information/sources.json": "56444ffd8625ce3eabff48ecf6439470234d00e591beb8e33f7f195e55f4a6a0",
    "speed-probe-shared-information/pairs.csv": "f48da3e2e64fe6ad264890137944f5776034408fa7bf359baecae0d0c2c9744b",
    "speed-probe-shared-information/loss-links.csv": "b25ba5f8fe9f6cafbe1bc20a32b318455804adf3dc1639f10d335607fea7222b",
    "speed-probe-shared-information/events.csv": "bc7a40a8da19e573d443c09fec79d4cb5fa1023de74b0bfbcfad2d4d40b07274",
    "speed-probe-error-components/components.csv": "60efb9edbba466a857c25043e60c0d9b851f6d3ff418d5e0f8e999281df991e8",
    "correction-information-renewal/sources.json": "30cfcbf3a7987e2c64da1b0ce0b16b36f1e92c319b7f51bc3c7048e1e3f535e6",
    "correction-information-renewal/transitions.csv": "bb985e1cd4f66915338512ff08921ea7454f3f1988ded37726be28fb801ad185",
    "correction-information-renewal/accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "correction-information-renewal/events.csv": "819b026c74884681e8df9fb282998a19ade0d12f82e8d1ffa3ad1a3b5ce1df6f",
    "correction-centre-transitions/snapshots.csv": "9d25ee780dcb7695efee4c586509f8fe9c65edac9bb00b0cc5e4d16f2b8b5fcf",
}


def projection_model(va, vb, c):
    assert min(va, vb) >= 0 and c * c <= va * vb + EPS
    vd, vm, cmd = va + vb - 2 * c, (va + vb + 2 * c) / 4, (va - vb) / 2
    assert min(vd, vm) >= -EPS
    vd, vm = max(0.0, vd), max(0.0, vm)
    degenerate = vd <= EPS
    if degenerate:
        assert abs(cmd) <= EPS
        k, explained = 0.0, 0.0
    else:
        k = cmd / vd
        explained = cmd * k
    vr = vm - explained
    assert vr >= -EPS and explained >= 0
    vr = max(0.0, vr)
    wa, wb = 0.5 - k, 0.5 + k
    assert abs(wa + wb - 1) < EPS
    weighted = math.fsum([wa * wa * va, wb * wb * vb, 2 * wa * wb * c])
    assert abs(vr - weighted) < EPS
    orthogonal = cmd - k * vd
    assert abs(orthogonal) <= EPS
    assert abs(vm - vr - explained) < EPS
    return dict(
        a_variance=va,
        b_variance=vb,
        covariance=c,
        contrast_variance=vd,
        midpoint_variance=vm,
        midpoint_contrast_covariance=cmd,
        projection_coefficient=k,
        a_weight=wa,
        b_weight=wb,
        explained_midpoint_variance=explained,
        residual_variance=vr,
        zero_or_roundoff_contrast=degenerate,
        negative_weight=wa < 0 or wb < 0,
        residual_contrast_covariance=orthogonal,
    )


def project(va, vb, c, a, b):
    result = projection_model(va, vb, c)
    wa, wb = result["a_weight"], result["b_weight"]
    for side, p in [("a", a), ("b", b)]:
        error = [float(p["error_" + axis]) for axis in ["x", "y"]]
        assert abs(math.hypot(*error) - float(p["error_norm"])) < 1e-8
        result[side + "_error_norm"] = float(p["error_norm"])
        for j, axis in enumerate(["x", "y"]):
            assert (
                abs(sum(float(p[part + "_" + axis]) for part in PARTS) - error[j])
                < 1e-8
            )
    for axis in ["x", "y"]:
        x, y = float(a["error_" + axis]), float(b["error_" + axis])
        d, m = x - y, (x + y) / 2
        r = wa * x + wb * y
        assert abs(m - result["projection_coefficient"] * d - r) < 1e-8
        (
            result["contrast_" + axis],
            result["midpoint_" + axis],
            result["residual_" + axis],
        ) = (d, m, r)
        for part in PARTS:
            result["residual_" + part + "_" + axis] = wa * float(
                a[part + "_" + axis]
            ) + wb * float(b[part + "_" + axis])
        assert abs(sum(result["residual_" + p + "_" + axis] for p in PARTS) - r) < 1e-8
    for name in ["contrast", "midpoint", "residual"]:
        result[name + "_norm"] = math.hypot(result[name + "_x"], result[name + "_y"])
    return result


def statistics_for(rows, ids, categories):
    matched = [r for r in rows if r["available"]]
    item = dict(
        records=len(rows),
        matched=len(matched),
        unavailable=len(rows) - len(matched),
        unique_source_pairs=len({r["source_id"] for r in rows}),
        cases=len({tuple(r[k] for k in KEY) for r in rows}),
        opportunities=len(ids),
        zero_or_roundoff_pairs=sum(r["zero_or_roundoff_contrast"] for r in matched),
        negative_weight_pairs=sum(r["negative_weight"] for r in matched),
    )
    for label in categories:
        item["category_" + label] = sum(r["category"] == label for r in rows)
    assert sum(item["category_" + label] for label in categories) == len(rows)
    for side, estimator in product(["a", "b"], ESTIMATORS):
        item[side + "_estimator_" + estimator] = sum(
            (r[side + "_estimator"] or "unavailable") == estimator for r in rows
        )
    for mode in ["shared_single", "preview_three"]:
        item["anchor_mode_" + mode] = sum(r.get("anchor_mode") == mode for r in rows)
    for field in [
        "contrast_variance",
        "midpoint_variance",
        "residual_variance",
        "explained_midpoint_variance",
    ]:
        item["pooled_sigma_" + field] = (
            math.sqrt(statistics.fmean(r[field] for r in matched)) if matched else ""
        )
    for field in ["a_weight", "b_weight"]:
        item["mean_" + field] = (
            statistics.fmean(r[field] for r in matched) if matched else ""
        )
    for name in ["contrast", "midpoint", "residual"]:
        item["observed_" + name + "_rms"] = (
            math.sqrt(statistics.fmean(r[name + "_norm"] ** 2 for r in matched))
            if matched
            else ""
        )
    for part in PARTS:
        item["observed_residual_" + part + "_rms"] = (
            math.sqrt(
                statistics.fmean(
                    r["residual_" + part + "_x"] ** 2
                    + r["residual_" + part + "_y"] ** 2
                    for r in matched
                )
            )
            if matched
            else ""
        )
    return item


def simultaneous_summary(rows, links, events):
    groups = defaultdict(list)
    opportunities = defaultdict(set)
    lookup = {r["source_id"]: r for r in rows}

    def add(r, pop, ids):
        kinds = {"all", *[events[e]["event_kind"] for e in ids]}
        for co, kind in product(["all", r["cohort"]], kinds):
            key = co, r["variant"], kind, pop, r["pairing"]
            groups[key].append(r)
            opportunities[key].update(
                e for e in ids if kind == "all" or events[e]["event_kind"] == kind
            )

    for r in rows:
        add(r, "all_states", r["source_event_ids"].split("+"))
    for l, (a, b) in product(links, PAIRINGS):
        r = lookup[l[a + "_" + b + "_pair_id"]]
        assert all(r[k] == l[k] for k in KEY)
        add(r, "raw_zero_loss", [l["event_id"]])
        if flag(l["command_only_zero_loss"]):
            add(r, "command_only_zero_loss", [l["event_id"]])
    result = []
    for co, arm, kind, pop, (a, b) in product(
        ["all", *COHORTS], ["epoch", "curved"], ["all", *KINDS], POPULATIONS, PAIRINGS
    ):
        key = co, arm, kind, pop, a + "_" + b
        result.append(
            dict(
                cohort=co,
                variant=arm,
                event_kind=kind,
                population=pop,
                pairing=a + "_" + b,
                **statistics_for(groups[key], opportunities[key], BINS)
            )
        )
    return result


def lagged_summary(rows, events):
    groups = defaultdict(list)
    for r in rows:
        for co, kind in product(["all", r["cohort"]], ["all", r["event_kind"]]):
            groups[co, r["variant"], kind, r["phase"], r["input_kind"]].append(r)
    result = []
    for co, arm, kind, phase, inp in product(
        ["all", *COHORTS], ["epoch", "curved"], ["all", *KINDS], PHASES, INPUTS
    ):
        rr = groups[co, arm, kind, phase, inp]
        result.append(
            dict(
                cohort=co,
                variant=arm,
                event_kind=kind,
                phase=phase,
                input_kind=inp,
                **statistics_for(rr, {r["event_id"] for r in rr}, ACCURACY)
            )
        )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    base = ROOT / "results"
    for n, h in PINNED.items():
        assert digest(base / n) == h, n
    prior = [
        json.loads((base / n / "sources.json").read_text())
        for n in ["speed-probe-shared-information", "correction-information-renewal"]
    ]
    helpers = {}
    for meta, name in zip(
        prior,
        [
            "analyze_speed_probe_shared_information.py",
            "analyze_correction_information_renewal.py",
        ],
    ):
        for n, h in {
            **meta["analysis"]["helpers_sha256"],
            name: meta["analysis"]["script_sha256"],
        }.items():
            assert n not in helpers or helpers[n] == h
            helpers[n] = h
    for n, h in helpers.items():
        assert digest(ROOT / n) == h, n
    assert (
        prior[0]["map_sha256"]
        == prior[1]["map_sha256"]
        == digest(ROOT / "inputs/obstacles.csv")
    )
    assert all(digest(SIMULATOR) == p["analysis"]["simulator_sha256"] for p in prior)
    accounting = read_rows(base / "correction-information-renewal/accounting.csv")
    event_rows = read_rows(base / "correction-information-renewal/events.csv")
    events = {e["event_id"]: e for e in event_rows}
    for e in read_rows(base / "speed-probe-shared-information/events.csv"):
        for k in ["event_id", *KEY, "event_kind", "event_time_s"]:
            assert e[k] == events[e["event_id"]][k]
    components = {
        p["pose_id"]: p
        for p in read_rows(base / "speed-probe-error-components/components.csv")
    }
    snapshots = {
        s["snapshot_id"]: s
        for s in read_rows(base / "correction-centre-transitions/snapshots.csv")
    }
    pairs = read_rows(base / "speed-probe-shared-information/pairs.csv")
    links = read_rows(base / "speed-probe-shared-information/loss-links.csv")
    transitions = read_rows(base / "correction-information-renewal/transitions.csv")
    assert (
        len(accounting),
        len(events),
        len(components),
        len(snapshots),
        len(pairs),
        len(links),
        len(transitions),
    ) == (648, 2583, 19830, 25830, 19830, 550, 20664)
    simultaneous = []
    for p in pairs:
        a, b = components[p["a_pose_id"]], components[p["b_pose_id"]]
        result = project(
            float(p["a_variance_joint"]),
            float(p["b_variance_joint"]),
            float(p["covariance_joint"]),
            a,
            b,
        )
        assert (
            abs(result["contrast_variance"] - float(p["difference_variance_joint"]))
            < EPS
        )
        for axis in ["x", "y"]:
            assert abs(result["contrast_" + axis] - float(p["delta_" + axis])) < 1e-8
        assert all(
            a[k] == b[k] == p[k]
            for k in [*KEY, "state_id", "time_s", "truth_x", "truth_y"]
            if k in p
        )
        assert all(a[k] == b[k] for k in ["truth_x", "truth_y"])
        typed = dict(
            p,
            position_separation=float(p["position_separation"]),
            a_inside=flag(p["a_inside"]),
            b_inside=flag(p["b_inside"]),
        )
        simultaneous.append(
            dict(
                source_id=p["pair_id"],
                **{
                    k: p[k]
                    for k in [
                        *KEY,
                        "state_id",
                        "time_s",
                        "chain_id",
                        "chain_index",
                        "source_event_ids",
                        "pairing",
                        "anchor_mode",
                    ]
                },
                a_id=p["a_pose_id"],
                b_id=p["b_pose_id"],
                a_estimator=p["a_estimator"],
                b_estimator=p["b_estimator"],
                available=True,
                category=category(typed, 0.05),
                **result
            )
        )
    # Preserve the projection field order for unavailable references too.
    blank = {k: "" for k in result}
    lagged = []
    for p in transitions:
        a, b = snapshots[p["pre_snapshot_id"]], snapshots[p["post_snapshot_id"]]
        available = flag(p["available"])
        assert available == flag(b["available"]) and flag(a["available"])
        assert all(a[k] == b[k] == p[k] for k in [*KEY, "event_id", "input_kind"])
        assert a["phase"] == "pre" and b["phase"] == p["phase"]
        if available:
            result = project(
                float(p["pre_variance"]),
                float(p["post_variance"]),
                float(p["covariance_joint"]),
                a,
                b,
            )
            assert (
                abs(result["contrast_variance"] - float(p["difference_variance_joint"]))
                < EPS
            )
            for axis in ["x", "y"]:
                assert (
                    abs(result["contrast_" + axis] + float(p["error_delta_" + axis]))
                    < 1e-8
                )
        else:
            result = blank
        lagged.append(
            dict(
                source_id=p["transition_id"],
                **{
                    k: p[k]
                    for k in [
                        *KEY,
                        "event_id",
                        "event_kind",
                        "event_time_s",
                        "phase",
                        "input_kind",
                        "event_anchor_replaced",
                        "post_shared_anchor_weight",
                        "intervening_anchor_changes",
                    ]
                },
                a_id=p["pre_snapshot_id"],
                b_id=p["post_snapshot_id"],
                a_estimator=a["estimator"],
                b_estimator=b["estimator"],
                available=available,
                category=p["accuracy_transition"],
                **result
            )
        )
    assert sum(r["available"] for r in lagged) == 18456
    assert sum(flag(l["command_only_zero_loss"]) for l in links) == 509
    ss, ls = simultaneous_summary(simultaneous, links, events), lagged_summary(
        lagged, events
    )
    audit = dict(
        accounted_cases=648,
        timeout_cases=368,
        opportunities=2583,
        simultaneous_states=6610,
        simultaneous_pairs=len(simultaneous),
        requested_lagged_pairs=len(lagged),
        matched_lagged_pairs=18456,
        unavailable_lagged_pairs=2208,
        raw_zero_loss_links=550,
        command_only_links=509,
        simultaneous_summary_rows=len(ss),
        lagged_summary_rows=len(ls),
        simultaneous_zero_or_roundoff_pairs=sum(
            r["zero_or_roundoff_contrast"] for r in simultaneous
        ),
        lagged_zero_or_roundoff_pairs=sum(
            r["zero_or_roundoff_contrast"] for r in lagged if r["available"]
        ),
    )
    source = dict(
        input_sha256=PINNED,
        map_sha256=prior[0]["map_sha256"],
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
            simulator_sha256=digest(SIMULATOR),
        ),
        schema=dict(
            variance_guard_m2=EPS,
            simultaneous_agreement_m=0.05,
            point_accuracy_m=0.1,
            inputs=INPUTS,
            phases=PHASES,
            lagged_contrast="oracle error difference after known truth displacement subtraction",
        ),
        inherited_validation=dict(observation_and_physics=True, new_rollout=False),
        audit=audit,
    )
    args.output.mkdir(parents=True)
    for n, rr in [
        ("accounting", accounting),
        ("events", event_rows),
        ("loss-links", links),
        ("simultaneous", simultaneous),
        ("lagged", lagged),
        ("simultaneous-summary", ss),
        ("lagged-summary", ls),
    ]:
        save_rows(args.output / (n + ".csv"), rr)
    for n, prefix in [
        ("accounting", "correction-information-renewal"),
        ("events", "correction-information-renewal"),
        ("loss-links", "speed-probe-shared-information"),
    ]:
        assert digest(args.output / (n + ".csv")) == PINNED[prefix + "/" + n + ".csv"]
    (args.output / "sources.json").write_text(json.dumps(source, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
