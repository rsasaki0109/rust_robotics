#!/usr/bin/env python3
"""Audit shared sample/anchor covariance and pairwise position agreement."""

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
from analyze_speed_probe_starts import ESTIMATES, KINDS
from analyze_terminal_arc_live import COHORTS, SIMULATOR
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
PRIOR = ROOT / "results/speed-probe-error-components"
PROTOCOL = ROOT / "results/speed-probe-shared-information/protocol.md"
LOSSES = ROOT / "results/speed-probe-coverage-spans/loss-memberships.csv"
PINNED = {
    "accounting.csv": "b101dcc2ece4dcacfc1ea8c792afbf27b041b35eccb44be1c245483bd719fe13",
    "events.csv": "bc7a40a8da19e573d443c09fec79d4cb5fa1023de74b0bfbcfad2d4d40b07274",
    "components.csv": "60efb9edbba466a857c25043e60c0d9b851f6d3ff418d5e0f8e999281df991e8",
    "groups.csv": "c2cf508a23447808a711f7d760c6f32874e58db24acce097e93941088fc01af8",
    "sources.json": "95a343b123b2f49ff8e519b3df0c3bce28cd253f4f0bc0aadb26b92e0f7a923e",
}
LOSS_SHA = "3c28f5a36e41279c3dceae50983868b54fb8ed5652617aa4ec527008e98ead07"
PAIRINGS = [("raw", "recovery"), ("raw", "preview"), ("recovery", "preview")]
THRESHOLDS = [0.02, 0.05, 0.10]
MODES = ["all", "shared_single", "preview_three"]
POPULATIONS = ["all_states", "raw_zero_loss", "command_only_zero_loss"]
BINS = [
    "agree_both_inside",
    "agree_a_inside_b_outside",
    "agree_a_outside_b_inside",
    "agree_both_outside",
    "disagree",
]
EPS = 1e-12


def ar_covariance(a, b, rho):
    """Linear scan of the sparse AR kernel; values include sample sigmas."""
    assert 0 <= rho <= 1
    previous, past_a, past_b, covariance = None, 0.0, 0.0, 0.0
    for step in sorted(set(a) | set(b)):
        if previous is not None:
            assert step > previous
            decay = rho ** (step - previous)
            past_a *= decay
            past_b *= decay
        x, y = a.get(step, 0.0), b.get(step, 0.0)
        covariance += x * y + x * past_b + y * past_a
        past_a += x
        past_b += y
        previous = step
    return covariance


def nonnegative(value):
    assert value >= -EPS, value
    return max(0.0, value)


def model(na, aa, nb, ab, cn, ca):
    assert min(na, aa, nb, ab) >= 0
    assert cn * cn <= na * nb + EPS and ca * ca <= aa * ab + EPS
    va, vb, cov = na + aa, nb + ab, cn + ca
    assert cov * cov <= va * vb + EPS
    dn, da = na + nb - 2 * cn, aa + ab - 2 * ca
    dn, da = nonnegative(dn), nonnegative(da)
    return dict(
        covariance_noise=cn,
        covariance_anchor=ca,
        covariance_joint=cov,
        difference_variance_noise=dn,
        difference_variance_anchor=da,
        difference_variance_joint=dn + da,
        independent_variance_joint=va + vb,
        difference_sigma_per_axis=math.sqrt(dn + da),
        independent_sigma_per_axis=math.sqrt(va + vb),
        correlation_joint=cov / math.sqrt(va * vb) if va * vb else "",
    )


def audit_matrix(diagonal, cross):
    scale = max(diagonal)
    if scale == 0:
        assert all(c == 0 for c in cross)
        return 0.0
    a, b, c = [v / scale for v in diagonal]
    x, y, z = [v / scale for v in cross]
    assert min(a, b, c) >= 0
    assert min(a * b - x * x, a * c - y * y, b * c - z * z) >= -1e-10
    determinant = a * b * c + 2 * x * y * z - a * z * z - b * y * y - c * x * x
    assert determinant >= -1e-10, determinant
    return determinant


def information(p, groups, current_anchor, current_variance, rho):
    sigma = float(p["sample_sigma_min"])
    assert sigma == float(p["sample_sigma_max"]) and sigma >= 0
    samples, anchors, vectors = {}, defaultdict(float), {}
    for g in groups:
        n, w = int(g["sample_count"]), float(g["group_weight"])
        first, last = int(g["first_sample_step"]), int(g["last_sample_step"])
        assert n == last - first + 1 and n > 0 and w >= 0
        anchor = g["anchor_ids"]
        assert "+" not in anchor
        vector = float(g["anchor_x"]), float(g["anchor_y"])
        if anchor in vectors:
            assert math.dist(vectors[anchor], vector) < 1e-10
        vectors[anchor] = vector
        anchors[anchor] += w
        for step in range(first, last + 1):
            assert step not in samples
            samples[step] = w / n * sigma
    assert abs(sum(float(g["group_weight"]) for g in groups) - 1) < EPS
    assert len(samples) == int(p["used_sample_count"])
    assert len(groups) == int(p["source_group_count"])
    assert current_anchor in anchors
    if p["pose_kind"] in ["raw", "recovery"]:
        assert set(anchors) == {current_anchor}
    else:
        assert len(anchors) in [1, 3]
    for i, a in enumerate(["x", "y"]):
        assert (
            abs(
                sum(w * vectors[k][i] for k, w in anchors.items())
                - float(p["anchor_" + a])
            )
            < 1e-10
        )
    vn = ar_covariance(samples, samples, rho)
    va = float(p["model_anchor_sigma_per_axis"]) ** 2
    if len(anchors) == 1:
        assert abs(va - current_variance) < EPS
        va = current_variance
    assert abs(vn - float(p["model_noise_sigma_per_axis"]) ** 2) < EPS
    assert abs(vn + va - float(p["model_noise_anchor_sigma_per_axis"]) ** 2) < EPS
    return dict(
        samples=samples,
        anchors=dict(anchors),
        vectors=vectors,
        noise=vn,
        anchor=va,
        rho=rho,
        current_anchor=current_anchor,
    )


def pair_record(pair_id, a, b, ia, ib, current_variance):
    assert all(
        a[k] == b[k]
        for k in [
            *KEY,
            "state_id",
            "time_s",
            "chain_id",
            "chain_index",
            "truth_x",
            "truth_y",
            "source_event_ids",
        ]
    )
    assert ia["current_anchor"] == ib["current_anchor"]
    shared = set(ia["anchors"]) & set(ib["anchors"])
    assert shared == {ia["current_anchor"]}
    ca = sum(ia["anchors"][k] * ib["anchors"][k] * current_variance for k in shared)
    cn = ar_covariance(ia["samples"], ib["samples"], ia["rho"])
    assert ia["rho"] == ib["rho"]
    same_steps = set(ia["samples"]) & set(ib["samples"])
    same_step_cov = sum(ia["samples"][s] * ib["samples"][s] for s in same_steps)
    delta = [float(a["pose_" + axis]) - float(b["pose_" + axis]) for axis in ["x", "y"]]
    pieces = {
        f"delta_{p}_{axis}": float(a[p + "_" + axis]) - float(b[p + "_" + axis])
        for p, axis in product(PARTS, ["x", "y"])
    }
    for i, axis in enumerate(["x", "y"]):
        assert abs(sum(pieces[f"delta_{p}_{axis}"] for p in PARTS) - delta[i]) < 1e-10
    separation = math.hypot(*delta)
    dyaw = float(a["pose_yaw"]) - float(b["pose_yaw"])
    return dict(
        pair_id=pair_id,
        state_id=a["state_id"],
        chain_id=a["chain_id"],
        chain_index=a["chain_index"],
        **{k: a[k] for k in KEY},
        time_s=a["time_s"],
        source_event_ids=a["source_event_ids"],
        pairing=a["pose_kind"] + "_" + b["pose_kind"],
        a_pose_id=a["pose_id"],
        b_pose_id=b["pose_id"],
        a_estimator=a["estimator"],
        b_estimator=b["estimator"],
        anchor_mode="shared_single" if len(ib["anchors"]) == 1 else "preview_three",
        current_anchor_id=ia["current_anchor"],
        current_anchor_variance=current_variance,
        a_current_anchor_weight=ia["anchors"][ia["current_anchor"]],
        b_current_anchor_weight=ib["anchors"][ib["current_anchor"]],
        a_sample_count=len(ia["samples"]),
        b_sample_count=len(ib["samples"]),
        shared_sample_count=len(same_steps),
        rho=ia["rho"],
        noise_same_step_covariance=same_step_cov,
        noise_temporal_covariance=cn - same_step_cov,
        a_variance_joint=ia["noise"] + ia["anchor"],
        b_variance_joint=ib["noise"] + ib["anchor"],
        **model(ia["noise"], ia["anchor"], ib["noise"], ib["anchor"], cn, ca),
        delta_x=delta[0],
        delta_y=delta[1],
        position_separation=separation,
        heading_separation_deg=abs(math.degrees(math.remainder(dyaw, 2 * math.pi))),
        a_position_error=float(a["error_norm"]),
        b_position_error=float(b["error_norm"]),
        a_inside=float(a["error_norm"]) <= 0.1,
        b_inside=float(b["error_norm"]) <= 0.1,
        within_reconstruction_tolerance=separation <= 1e-8,
        accuracy_near_boundary=min(
            abs(float(a["error_norm"]) - 0.1), abs(float(b["error_norm"]) - 0.1)
        )
        <= 1e-8,
        **pieces,
        **{
            f"delta_{p}_norm": math.hypot(
                pieces[f"delta_{p}_x"], pieces[f"delta_{p}_y"]
            )
            for p in PARTS
        },
    )


def category(pair, threshold):
    if pair["position_separation"] > threshold:
        return "disagree"
    return BINS[
        (
            0
            if pair["a_inside"] and pair["b_inside"]
            else 1 if pair["a_inside"] else 2 if pair["b_inside"] else 3
        )
    ]


def summarize(pairs, links, events):
    lookup = {r["pair_id"]: r for r in pairs}
    groups = defaultdict(list)

    def add(p, pop, ids):
        for co, kind, mode in product(
            ["all", p["cohort"]],
            {"all", *[events[e]["event_kind"] for e in ids]},
            ["all", p["anchor_mode"]],
        ):
            groups[co, p["variant"], kind, p["pairing"], pop, mode].append((p, ids))

    for p in pairs:
        add(p, "all_states", p["source_event_ids"].split("+"))
    for link in links:
        for a, b in PAIRINGS:
            p = lookup[link[a + "_" + b + "_pair_id"]]
            add(p, "raw_zero_loss", [link["event_id"]])
            if link["command_only_zero_loss"]:
                add(p, "command_only_zero_loss", [link["event_id"]])
    output = []
    for co, arm, kind, pairing, pop, mode, threshold in product(
        ["all", *COHORTS],
        ["epoch", "curved"],
        ["all", *KINDS],
        [a + "_" + b for a, b in PAIRINGS],
        POPULATIONS,
        MODES,
        THRESHOLDS,
    ):
        selected = groups[co, arm, kind, pairing, pop, mode]
        rr = [p for p, _ in selected]
        ids = {e for _, ee in selected for e in ee}
        counts = {c: sum(category(p, threshold) == c for p in rr) for c in BINS}
        assert sum(counts.values()) == len(rr)
        output.append(
            dict(
                cohort=co,
                variant=arm,
                event_kind=kind,
                pairing=pairing,
                population=pop,
                anchor_mode=mode,
                agreement_threshold_m=threshold,
                records=len(rr),
                states=len({p["state_id"] for p in rr}),
                cases=len({tuple(p[k] for k in KEY) for p in rr}),
                opportunities=sum(
                    kind == "all" or events[e]["event_kind"] == kind for e in ids
                ),
                **counts,
                within_reconstruction_tolerance=sum(
                    p["within_reconstruction_tolerance"] for p in rr
                ),
                mean_position_separation=(
                    statistics.fmean(p["position_separation"] for p in rr) if rr else ""
                ),
                mean_correlation_joint=(
                    statistics.fmean(p["correlation_joint"] for p in rr) if rr else ""
                ),
                pooled_difference_sigma=(
                    math.sqrt(
                        statistics.fmean(p["difference_variance_joint"] for p in rr)
                    )
                    if rr
                    else ""
                ),
                pooled_independent_sigma=(
                    math.sqrt(
                        statistics.fmean(p["independent_variance_joint"] for p in rr)
                    )
                    if rr
                    else ""
                ),
                mean_covariance_noise=(
                    statistics.fmean(p["covariance_noise"] for p in rr) if rr else ""
                ),
                mean_covariance_anchor=(
                    statistics.fmean(p["covariance_anchor"] for p in rr) if rr else ""
                ),
            )
        )
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for n, h in PINNED.items():
        assert digest(PRIOR / n) == h, n
    assert digest(LOSSES) == LOSS_SHA
    prior = json.loads((PRIOR / "sources.json").read_text())
    raw_source = ROOT / "results/joint-admission-geometry/sources.json"
    assert (
        digest(raw_source)
        == prior["input_sha256"]["joint-admission-geometry/sources.json"]
    )
    raw_metadata = json.loads(raw_source.read_text())
    rhos = {}
    for cohort in COHORTS:
        tau = raw_metadata["cohorts"][cohort]["provenance"]["noise_correlation_time_s"]
        rhos[cohort] = math.exp(-0.02 / tau) if tau else 0.0
    helpers = dict(prior["analysis"]["helpers_sha256"])
    helpers["analyze_speed_probe_error_components.py"] = prior["analysis"][
        "script_sha256"
    ]
    for n, h in helpers.items():
        assert digest(ROOT / n) == h, n
    assert digest(SIMULATOR) == prior["analysis"]["simulator_sha256"]
    assert digest(ROOT / "inputs/obstacles.csv") == prior["map_sha256"]
    accounting, event_rows, components, group_rows = (
        read_rows(PRIOR / n)
        for n in ["accounting.csv", "events.csv", "components.csv", "groups.csv"]
    )
    losses = read_rows(LOSSES)
    assert (
        len(accounting),
        len(event_rows),
        len(components),
        len(group_rows),
        len(losses),
    ) == (648, 2583, 19830, 26990, 19800)
    events = {e["event_id"]: e for e in event_rows}
    groups = defaultdict(list)
    for g in group_rows:
        groups[g["pose_id"]].append(g)
    states = defaultdict(dict)
    for p in components:
        assert p["pose_kind"] not in states[p["state_id"]]
        states[p["state_id"]][p["pose_kind"]] = p
    assert len(states) == 6610 and all(
        set(s) == set(ESTIMATES) for s in states.values()
    )
    inputs, weights, pairs, registry, determinants = [], [], [], {}, []
    infos = {}
    for sid in sorted(states, key=int):
        state = states[sid]
        raw = state["raw"]
        current = groups[raw["pose_id"]][0]["anchor_ids"]
        variance = float(raw["model_anchor_sigma_per_axis"]) ** 2
        for kind in ESTIMATES:
            p = state[kind]
            info = information(
                p, groups[p["pose_id"]], current, variance, rhos[p["cohort"]]
            )
            infos[p["pose_id"]] = info
            inputs.append(
                dict(
                    pose_id=p["pose_id"],
                    state_id=sid,
                    **{k: p[k] for k in KEY},
                    time_s=p["time_s"],
                    pose_kind=kind,
                    estimator=p["estimator"],
                    current_anchor_id=current,
                    current_anchor_weight=info["anchors"][current],
                    sample_count=len(info["samples"]),
                    anchor_count=len(info["anchors"]),
                    rho=info["rho"],
                    variance_noise=info["noise"],
                    variance_anchor=info["anchor"],
                    variance_joint=info["noise"] + info["anchor"],
                )
            )
            for anchor, w in sorted(info["anchors"].items(), key=lambda kv: int(kv[0])):
                vector = info["vectors"][anchor]
                key = tuple(p[k] for k in KEY) + (anchor,)
                if key in registry:
                    assert math.dist(registry[key], vector) < 1e-10
                registry[key] = vector
                weights.append(
                    dict(
                        pose_id=p["pose_id"],
                        anchor_id=anchor,
                        weight=w,
                        anchor_x=vector[0],
                        anchor_y=vector[1],
                        current_anchor=anchor == current,
                        known_current_variance=variance if anchor == current else "",
                    )
                )
        records = []
        for a, b in PAIRINGS:
            pa, pb = state[a], state[b]
            r = pair_record(
                len(pairs) + 1,
                pa,
                pb,
                infos[pa["pose_id"]],
                infos[pb["pose_id"]],
                variance,
            )
            pairs.append(r)
            records.append(r)
        diagonal = [
            infos[state[k]["pose_id"]]["noise"] + infos[state[k]["pose_id"]]["anchor"]
            for k in ESTIMATES
        ]
        determinants.append(
            audit_matrix(diagonal, [r["covariance_joint"] for r in records])
        )
    lookup = {(r["state_id"], r["pairing"]): r for r in pairs}
    links = []
    seen = set()
    for loss in losses:
        if (
            loss["pose_kind"] != "raw"
            or float(loss["radius_m"]) != 0
            or int(loss["yaw_bound_deg"]) != 0
        ):
            continue
        assert loss["transition_id"] not in seen
        seen.add(loss["transition_id"])
        p = states[loss["state_id"]]["raw"]
        links.append(
            dict(
                transition_id=loss["transition_id"],
                event_id=loss["event_id"],
                state_id=loss["state_id"],
                **{k: p[k] for k in KEY},
                time_s=p["time_s"],
                command_only_zero_loss=flag(loss["command_only_zero_loss"]),
                **{
                    a
                    + "_"
                    + b
                    + "_pair_id": lookup[loss["state_id"], a + "_" + b]["pair_id"]
                    for a, b in PAIRINGS
                },
            )
        )
    assert len(links) == 550 and sum(r["command_only_zero_loss"] for r in links) == 509
    assert len({l["state_id"] for l in links}) == 550
    summary = summarize(pairs, links, events)
    audit = dict(
        accounted_cases=648,
        source_timeouts=sum(flag(r["selected"]) for r in accounting),
        opportunities=2583,
        audited_states=len(states),
        inputs=len(inputs),
        anchor_weights=len(weights),
        pairs=len(pairs),
        raw_zero_loss_links=len(links),
        command_only_links=509,
        summary_rows=len(summary),
        covariance_matrices=len(determinants),
        min_normalized_determinant=min(determinants),
        accuracy_near_boundary_pairs=sum(p["accuracy_near_boundary"] for p in pairs),
    )
    sources = dict(
        input_sha256=PINNED,
        loss_input_sha256=LOSS_SHA,
        map_sha256=prior["map_sha256"],
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            helpers_sha256=dict(sorted(helpers.items())),
            simulator_sha256=digest(SIMULATOR),
        ),
        schema=dict(
            pairings=PAIRINGS,
            agreement_thresholds_m=THRESHOLDS,
            accuracy_radius_m=0.1,
            bins=BINS,
            anchor_modes=MODES,
            populations=POPULATIONS,
            variance_roundoff_guard_m2=EPS,
            cohort_rho=rhos,
            partial_model="isotropic observation noise plus shared anchor; excludes drift and nonlinear transport",
        ),
        inherited_validation=dict(
            source_sha256=PINNED["sources.json"],
            observation_and_physics=True,
            new_rollout=False,
        ),
        audit=audit,
    )
    args.output.mkdir(parents=True)
    for name, rr in [
        ("accounting", accounting),
        ("events", event_rows),
        ("inputs", inputs),
        ("anchor-weights", weights),
        ("pairs", pairs),
        ("loss-links", links),
        ("summary", summary),
    ]:
        save_rows(args.output / f"{name}.csv", rr)
    for n in ["accounting.csv", "events.csv"]:
        assert digest(args.output / n) == PINNED[n]
    (args.output / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
