#!/usr/bin/env python3
"""Reproduce the original audit selection and transfer immutable known cutoffs."""

import argparse
from collections import Counter, defaultdict, deque
from copy import copy
from fractions import Fraction
from itertools import product
import json
import math
from pathlib import Path
import shutil

from analyze_commanded_recovery_arc import flag
from analyze_contrast_common_errors import project
from analyze_contrast_risk_transfer import (
    ARMS,
    BUDGETS,
    KINDS,
    NOISES,
    SCORES,
    SIM_VIEWS,
    TARGETS,
    case_weights,
    score_tick,
)
from analyze_correction_centre_transitions import checked_poses, gate_context
from analyze_correction_gate_boundaries import label_pair, nominal_command
from analyze_correction_gate_sequences import failure_mask
from analyze_epoch_position_components import Replay, decompose
from analyze_epoch_preview import digest, read_rows
from analyze_estimator_mode_transfer import STRATA
from analyze_fresh_seed_modes import FRESH, accounting, identity
from analyze_joint_admission_geometry import KEY
from analyze_mode_conditional_cutoffs import (
    decision,
    difference,
    exported,
    method_metrics,
    overlap_metrics,
)
from analyze_speed_probe_error_components import input_groups
from analyze_speed_probe_shared_information import PAIRINGS, ar_covariance
from analyze_terminal_arc_live import COHORTS, SIMULATOR, load_tracks
from summarize_stopping_preview import save_rows

ROOT = Path(__file__).resolve().parent
BASE = ROOT / "results"
PROTOCOL = BASE / "fresh-seed-cutoff-transfer/protocol.md"
STRATUM_NAMES = [*STRATA, "recovery_fallback_single"]
PINNED = {
    "fresh-seed-mode-census/sources.json": "1fcb5cde097933e76dceb736266609725f0d64336cd4331f20bdf953fba4adb3",
    "fresh-seed-mode-census/accounting.csv": "00e055290eb36175bd7951510ada1a31957eff41e853049816a7d6bc86a90462",
    "fresh-seed-mode-census/events.csv": "d9eedf0c559abb81f39c9ae68a859e15fcec3471e2ed1274383eb8e0e1b39dfd",
    "mode-conditional-cutoffs/membership.csv": "e2924bd538b435b26ffab25e4bdeba7d93119e12355f6fed2f1d5371eac4f79c",
    "mode-conditional-cutoffs/thresholds.csv": "e877f83cb96aaf14c2a273c06416eb41998237fed8ffdfac0529b729a5c70207",
    "mode-conditional-cutoffs/conditional-thresholds.csv": "4d20ddb96399552a932aa4c0db904b7e6c085291863b9273e451c5692410c040",
    "mode-conditional-cutoffs/comparison.csv": "4dd9971f43171627831f07be71b25cd24bd52b24ee4d41c14a314af35a84ac01",
    "contrast-risk-transfer/scores.csv": "96e963f962a138f066f01d582d34e1a33a5d315de693f160fa37b3a3beca52e6",
    "contrast-common-errors/simultaneous.csv": "25e405945fd3e18ea93738866cc9b265bda55017ec0d04422ea03f52c6b31031",
}


def frozen(state):
    result = copy(state)
    result.motion = deque(state.motion, maxlen=25)
    result.recovery = deque(state.recovery, maxlen=25)
    result.groups = deque(
        (dict(g, samples=deque(g["samples"], maxlen=25)) for g in state.groups),
        maxlen=3,
    )
    return result


def edge_windows(events):
    edges = defaultdict(list)
    for e in events:
        step = round(float(e["event_time_s"]) / 0.02)
        assert 7500 <= step <= 9000
        for offset in range(0, min(75, 9000 - step) + 1):
            edges[step + offset].append((str(e["event_id"]), offset))
    return edges


def focused(a, b):
    return any(c["route"] == "joint" and flag(c["active_ready"]) for c in [a, b])


def stratum(view, estimator, anchor_mode):
    if view == "sim_raw_recovery" and estimator == "raw_fallback":
        assert anchor_mode == "shared_single"
        return "recovery_fallback_single"
    choices = [
        name for name, pair in STRATA.items() if pair == (estimator, anchor_mode)
    ]
    assert len(choices) == 1, (view, estimator, anchor_mode)
    return choices[0]


def pose_inputs(snapshot, kind):
    state, before, row = snapshot["state"], snapshot["before"], snapshot["row"]
    estimator, groups = input_groups(state, kind)
    comp, _ = decompose(
        groups,
        tuple(float(before[a]) for a in ["x", "y"]),
        state.odom,
        snapshot["rho"],
    )
    samples, anchors = {}, defaultdict(float)
    for g in groups:
        ids = {s.anchor_id for s in g["samples"]}
        assert len(ids) == 1
        anchors[next(iter(ids))] += g["weight"]
        for sample in g["samples"]:
            assert sample.step not in samples
            samples[sample.step] = g["weight"] / len(g["samples"]) * sample.sigma
    assert len(samples) == comp["used_sample_count"]
    assert comp["sample_sigma_min"] == comp["sample_sigma_max"]
    va = (
        state.anchor_sigma**2
        if len(anchors) == 1
        else comp["model_anchor_sigma_per_axis"] ** 2
    )
    vn = ar_covariance(samples, samples, snapshot["rho"])
    assert abs(vn - comp["model_noise_sigma_per_axis"] ** 2) < 1e-12
    assert abs(va - comp["model_anchor_sigma_per_axis"] ** 2) < 1e-12
    return comp, dict(samples=samples, anchors=anchors, variance=vn + va), estimator


def state_pairs(snapshot):
    state, row = snapshot["state"], snapshot["row"]
    _, error = checked_poses(state, row)
    pieces = {
        kind: pose_inputs(snapshot, kind) for kind in ["raw", "recovery", "preview"]
    }
    pairs = []
    rho = snapshot["rho"]
    for a, b in PAIRINGS:
        ac, ai, ae = pieces[a]
        bc, bi, be = pieces[b]
        shared = set(ai["anchors"]) & set(bi["anchors"])
        assert shared == {state.anchor_id}
        cov = ar_covariance(ai["samples"], bi["samples"], rho) + sum(
            ai["anchors"][k] * bi["anchors"][k] * state.anchor_sigma**2 for k in shared
        )
        p = project(ai["variance"], bi["variance"], cov, ac, bc)
        view = "sim_" + a + "_" + b
        anchor_mode = "preview_three" if len(bi["anchors"]) == 3 else "shared_single"
        pairs.append(
            dict(
                view=view,
                a_estimator=ae,
                b_estimator=be,
                anchor_mode=anchor_mode,
                stratum=stratum(view, be, anchor_mode),
                **p,
            )
        )
    return pieces, pairs, error


def collect_case(case, track, events, old_ids, next_ids, rho):
    edges = edge_windows(events)
    needs = set(edges) | {s - 1 for s in edges}
    state, previous, selected, links = Replay(), None, {}, []
    selected_edges, max_error = 0, 0.0

    def capture(snapshot, step):
        nonlocal max_error
        if step not in selected:
            k = identity(case), step
            if case["partition"] != "fresh":
                assert k in old_ids, (k, "unexpected historical audit state")
                sid = old_ids[k]
            else:
                sid = next_ids["state"]
                next_ids["state"] += 1
            pieces, pairs, error = state_pairs(snapshot)
            max_error = max(max_error, error)
            selected[step] = dict(
                state_id=sid,
                time_s=float(snapshot["row"]["time_s"]),
                truth_time_s=float(snapshot["before"]["time_s"]),
                pieces=pieces,
                pairs=pairs,
                events=set(),
                raw_links=0,
                command_links=0,
            )
        return selected[step]

    for step, (before, row) in enumerate(zip(track, track[1:]), 1):
        assert round(float(row["time_s"]) / 0.02) == step
        assert abs(float(row["noise_rho"]) - rho) < 1e-12
        state.advance(before, row)
        state.check_counts(row)
        if step not in needs:
            continue
        current = dict(
            state=frozen(state),
            rho=rho,
            before=before,
            row=row,
            context=gate_context(before, row),
        )
        if step in edges:
            assert previous is not None and float(previous["row"]["time_s"]) == float(
                before["time_s"]
            )
            if focused(previous["context"], current["context"]):
                a, b = capture(previous, step - 1), capture(current, step)
                for eid, offset in edges[step]:
                    selected_edges += 1
                    a["events"].add(eid)
                    b["events"].add(eid)
                    if (
                        offset
                        and label_pair(previous["context"], current["context"])
                        == "joint_lost"
                    ):
                        reason, _ = nominal_command(
                            float(row["speed_free_distance"]),
                            float(row["speed_cap"]),
                            float(row["uncapped_target_speed"]),
                            float(row["nominal_target_speed"]),
                        )
                        if reason == "speed_cap_zero":
                            mask, _ = failure_mask(current["context"])
                            b["raw_links"] += 1
                            b["command_links"] += mask == 16
                            links.append(
                                dict(
                                    **{k: case[k] for k in KEY},
                                    event_id=eid,
                                    pre_state_id=a["state_id"],
                                    post_state_id=b["state_id"],
                                    end_offset_steps=offset,
                                    failed_mask=mask,
                                    command_only_zero_loss=mask == 16,
                                )
                            )
        previous = current
    assert len(track) - 1 == 9000
    return (
        selected,
        links,
        dict(
            selected_states=len(selected),
            selected_transitions=selected_edges,
            max_pose_error=max_error,
            replayed_intervals=9000,
        ),
    )


def evaluate(rows, pooled, conditional, events, partitions):
    groups = defaultdict(list)
    for r in rows:
        for kind in {
            "all",
            *[events[e]["event_kind"] for e in r["source_event_ids"].split("+")],
        }:
            groups[r["partition"], r["noise"], r["variant"], kind, r["view"]].append(r)
    pc = {(t["noise"], t["view"], t["score"], t["budget"]): t for t in pooled}
    cc = {
        (t["noise"], t["view"], t["stratum"], t["score"], t["budget"]): t
        for t in conditional
    }
    result = []
    for key in product(partitions, NOISES, ARMS, ["all", *KINDS], SIM_VIEWS):
        parent = groups[key]
        weights = case_weights(parent)
        group_mass = sum(weights, Fraction())
        for s, score, budget in product(["all", *STRATUM_NAMES], SCORES, BUDGETS):
            sub = [
                (r, w)
                for r, w in zip(parent, weights)
                if s == "all" or r["stratum"] == s
            ]
            rr, ww = [r for r, w in sub], [w for r, w in sub]
            mass = sum(ww, Fraction())
            pt = pc[key[1], key[4], score, budget]
            pf = [decision(r, score, pt) for r in rr]
            cuts = [
                cc.get(
                    (key[1], key[4], r["stratum"], score, budget),
                    dict(
                        rule="all_available" if budget == "all" else "no_training_data"
                    ),
                )
                for r in rr
            ]
            cf = [decision(r, score, t) for r, t in zip(rr, cuts)]
            unsupported = [(r, w) for r, w, f in zip(rr, ww, cf) if f is None]
            record = dict(
                zip(["partition", "noise", "variant", "event_kind", "view"], key)
            )
            record.update(
                stratum=s,
                records=len(rr),
                cases=len({identity(r) for r in rr}),
                case_mass=mass,
                group_case_mass=group_mass,
                case_mass_share=mass / group_mass if group_mass else None,
                score=score,
                budget=budget,
                pooled_threshold_id=pt["threshold_id"],
                pooled_rule=pt["rule"],
                conditional_support=(
                    "unsupported" if unsupported else "complete" if rr else "empty"
                ),
                conditional_unsupported_rows=len(unsupported),
                conditional_unsupported_case_mass=sum(
                    (w for r, w in unsupported), Fraction()
                ),
                conditional_unsupported_strata="+".join(
                    s
                    for s in STRATUM_NAMES
                    if any(r["stratum"] == s for r, w in unsupported)
                ),
            )
            stats = {
                "pooled": method_metrics(rr, ww, pf),
                "conditional": method_metrics(rr, ww, cf),
            }
            for method, metrics in stats.items():
                record.update({method + "_" + k: v for k, v in metrics.items()})
            record.update(overlap_metrics(rr, ww, pf, cf))
            for field in [
                "retained_rows",
                "retained_case_mass",
                "case_weighted_coverage",
                *[
                    t + s
                    for t in TARGETS
                    for s in ["_error_case_mass", "_case_weighted_risk"]
                ],
            ]:
                record[field + "_delta"] = difference(
                    stats["conditional"][field], stats["pooled"][field]
                )
            result.append(exported(record))
    return result


def support(cases, scores):
    groups = defaultdict(list)
    for r in scores:
        if r["view"] == "sim_recovery_preview":
            groups[identity(r)].append(r)
    result = []
    for c in cases:
        rr = groups[identity(c)]
        strata = Counter(r["stratum"] for r in rr)
        result.append(
            dict(
                **c,
                audited_states=len(rr),
                filter_states=strata["filter_single"],
                fusion_states=strata["fusion_three"],
                actual_fallback_states=sum(
                    r["a_estimator"] == "raw_fallback" for r in rr
                ),
                support=(
                    "mixed"
                    if len(strata) == 2
                    else (
                        "filter_only"
                        if strata["filter_single"]
                        else "fusion_only" if strata["fusion_three"] else "no_audit"
                    )
                ),
            )
        )
    return result


def regression(rows, old_scores, old_members, projections, pooled, conditional, events):
    actual = {str(r["source_id"]): r for r in rows}
    expected = {
        r["source_id"]: r for r in old_scores if r["source_family"] == "simultaneous"
    }
    assert set(actual) == set(expected) and len(actual) == 19830
    members = {r["source_id"]: r for r in old_members}
    fields = 0
    maximum = 0.0
    for sid, r in actual.items():
        e = expected[sid]
        m = members[sid]
        for k in [*KEY, "source_event_ids", "partition", "noise", "view"]:
            assert str(r[k]) == e[k], (sid, k, r[k], e[k])
            fields += 1
        for k in ["a_estimator", "b_estimator", "anchor_mode", "stratum"]:
            assert r[k] == m[k]
            fields += 1
        for score in SCORES:
            assert r[score + "_tick"] == int(e[score + "_tick"]), (
                sid,
                score,
                r[score + "_tick"],
                e[score + "_tick"],
            )
            fields += 1
        for target in TARGETS:
            assert r[target + "_outside"] == flag(e[target + "_outside"])
            fields += 1
            delta = abs(r[target + "_error_m"] - float(e[target + "_error_m"]))
            assert delta < 1e-8
            maximum = max(maximum, delta)
            fields += 1
        for k in ["raw_zero_loss_references", "command_only_loss_references"]:
            assert r[k] == int(e[k]), (sid, k)
            fields += 1
        for k, v in projections[sid].items():
            if k in ["source_id", "view"]:
                continue
            if isinstance(v, bool):
                assert v == flag(projections["expected"][sid][k])
            else:
                d = abs(v - float(projections["expected"][sid][k]))
                assert d < 1e-8, (sid, k, d)
                maximum = max(maximum, d)
            fields += 1
    comparisons = evaluate(rows, pooled, conditional, events, ["known", "new"])
    index = {
        tuple(
            r[k]
            for k in [
                "partition",
                "noise",
                "variant",
                "event_kind",
                "view",
                "stratum",
                "score",
                "budget",
            ]
        ): r
        for r in comparisons
    }
    for e in read_rows(BASE / "mode-conditional-cutoffs/comparison.csv"):
        r = index[
            tuple(
                e[k]
                for k in [
                    "partition",
                    "noise",
                    "variant",
                    "event_kind",
                    "view",
                    "stratum",
                    "score",
                    "budget",
                ]
            )
        ]
        for k, v in r.items():
            assert str(v) == e[k], (k, v, e[k])
            fields += 1
    assert sum(r["raw_zero_loss_references"] for r in rows) == 550 * 3
    assert sum(r["command_only_loss_references"] for r in rows) == 509 * 3
    return dict(
        selected_states=len({r["state_id"] for r in rows}),
        simultaneous_pairs=len(rows),
        compared_fields=fields,
        max_float_discrepancy=maximum,
        historical_comparisons=6912,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new output directory")
    for n, sha in PINNED.items():
        assert digest(BASE / n) == sha, n
    census = json.loads((BASE / "fresh-seed-mode-census/sources.json").read_text())
    historic = json.loads(
        (BASE / "correction-centre-transitions/sources.json").read_text()
    )
    for meta in [census, historic]:
        assert digest(SIMULATOR) == meta["analysis"]["simulator_sha256"]
        for n, sha in meta["analysis"]["helpers_sha256"].items():
            assert digest(ROOT / n) == sha, n
    cases = read_rows(BASE / "fresh-seed-mode-census/accounting.csv")
    for c in cases:
        c["selected_timeout"] = flag(c["selected_timeout"])
    ee = read_rows(BASE / "fresh-seed-mode-census/events.csv")
    for e in ee:
        old = e["event_id"]
        e["source_event_id"] = old
        e["event_id"] = str(
            int(old.split("_")[1]) + (2583 if old.startswith("fresh_") else 0)
        )
    events = {e["event_id"]: e for e in ee}
    bycase = defaultdict(list)
    for e in ee:
        bycase[identity(e)].append(e)
    old_sim = read_rows(BASE / "contrast-common-errors/simultaneous.csv")
    old_members = read_rows(BASE / "mode-conditional-cutoffs/membership.csv")
    old_scores = read_rows(BASE / "contrast-risk-transfer/scores.csv")
    old_ids = {
        (identity(r), round(float(r["time_s"]) / 0.02)): int(r["state_id"])
        for r in old_sim
    }
    pair_ids = {
        (int(r["state_id"]), "sim_" + r["pairing"]): int(r["source_id"])
        for r in old_sim
    }
    assert len(old_ids) == 6610
    next_ids = dict(state=max(old_ids.values()) + 1, pair=19831)
    pooled = read_rows(BASE / "mode-conditional-cutoffs/thresholds.csv")
    conditional = read_rows(
        BASE / "mode-conditional-cutoffs/conditional-thresholds.csv"
    )
    all_scores, poses, pair_output, links, verification = [], [], [], [], []
    projections = {"expected": {r["source_id"]: r for r in old_sim}}
    sources = {}
    reg = None
    for cohort in [*COHORTS, *FRESH]:
        meta = (census["fresh"] if cohort in FRESH else historic["cohorts"])[cohort]
        run = Path(meta["path"])
        assert json.loads((run / "provenance.json").read_text()) == meta["provenance"]
        cc = [c for c in cases if c["cohort"] == cohort and c["selected_timeout"]]
        trial_cases = defaultdict(list)
        for c in cc:
            trial_cases[c["variant"], c["mode"], c["seed"]].append(c)
        counts = Counter()
        hashes = {}
        maxerr = 0.0
        for (variant, mode, seed), current in sorted(trial_cases.items()):
            n = f"{variant}-{mode}-{seed}/trajectories.csv"
            hashes[n] = digest(run / n)
            assert hashes[n] == meta["raw_sha256"][n]
            tracks = load_tracks(run / n)
            for c in sorted(current, key=identity):
                states, ll, v = collect_case(
                    c,
                    tracks[c["scenario"], c["planner"]],
                    bycase[identity(c)],
                    old_ids,
                    next_ids,
                    (
                        math.exp(-0.02 / meta["provenance"]["noise_correlation_time_s"])
                        if meta["provenance"]["noise_correlation_time_s"]
                        else 0.0
                    ),
                )
                links.extend(ll)
                for k, value in v.items():
                    if k == "max_pose_error":
                        maxerr = max(maxerr, value)
                    else:
                        counts[k] += value
                for step, st in sorted(states.items()):
                    common = dict(
                        **{k: c[k] for k in KEY},
                        partition=(
                            c["partition"]
                            if c["partition"] != "existing_new"
                            else "new"
                        ),
                        noise=c["noise"],
                        state_id=st["state_id"],
                        time_s=st["time_s"],
                        truth_time_s=st["truth_time_s"],
                        source_event_ids="+".join(sorted(st["events"], key=int)),
                    )
                    if cohort in FRESH:
                        for kind, (comp, info, estimator) in st["pieces"].items():
                            poses.append(
                                dict(
                                    **common,
                                    pose_kind=kind,
                                    estimator=estimator,
                                    **comp,
                                )
                            )
                    for p in st["pairs"]:
                        if cohort in FRESH:
                            sid = next_ids["pair"]
                            next_ids["pair"] += 1
                        else:
                            sid = pair_ids[st["state_id"], p["view"]]
                        r = dict(
                            source_family="simultaneous",
                            source_id=sid,
                            **common,
                            **{
                                k: p[k]
                                for k in [
                                    "view",
                                    "a_estimator",
                                    "b_estimator",
                                    "anchor_mode",
                                    "stratum",
                                ]
                            },
                            available=True,
                            raw_zero_loss_references=st["raw_links"],
                            command_only_loss_references=st["command_links"],
                        )
                        for score, field in [
                            ("contrast", "contrast_norm"),
                            ("midpoint_sigma", "midpoint_variance"),
                            ("residual_sigma", "residual_variance"),
                        ]:
                            value = (
                                p[field] if score == "contrast" else math.sqrt(p[field])
                            )
                            r[score + "_score_m"] = value
                            r[score + "_tick"] = score_tick(value)
                        for target in TARGETS:
                            r[target + "_error_m"] = p[target + "_norm"]
                            r[target + "_outside"] = p[target + "_norm"] > 0.1
                        all_scores.append(r)
                        model = {
                            k: v
                            for k, v in p.items()
                            if isinstance(v, (int, float, bool))
                        }
                        if cohort in FRESH:
                            pair_output.append(dict(source_id=sid, **common, **p))
                        else:
                            projections[str(sid)] = model
            del tracks
        sources[cohort] = dict(
            path=str(run),
            provenance_sha256=digest(run / "provenance.json"),
            raw_sha256=hashes,
        )
        verification.append(
            dict(cohort=cohort, cases=len(cc), **counts, max_pose_error=maxerr)
        )
        print(cohort, dict(counts), flush=True)
        if cohort == COHORTS[-1]:
            reg = regression(
                all_scores,
                old_scores,
                old_members,
                projections,
                pooled,
                conditional,
                events,
            )
            assert (
                reg["selected_states"] == 6610
                and sum(v["selected_transitions"] for v in verification) == 6187
            )
            print("Historical regression passed", reg, flush=True)
    fresh = [r for r in all_scores if r["partition"] == "fresh"]
    comparison = evaluate(fresh, pooled, conditional, events, ["fresh"])
    args.output.mkdir(parents=True)
    for n, rr in [
        ("scores", fresh),
        ("poses", poses),
        ("projections", pair_output),
        ("loss-links", [r for r in links if r["cohort"] in FRESH]),
        ("case-support", support(cases, all_scores)),
        ("comparison", comparison),
        ("verification", verification),
        ("events", ee),
    ]:
        save_rows(args.output / (n + ".csv"), rr)
    for n in ["accounting.csv"]:
        shutil.copyfile(BASE / "fresh-seed-mode-census" / n, args.output / n)
    for n in ["thresholds.csv", "conditional-thresholds.csv"]:
        shutil.copyfile(BASE / "mode-conditional-cutoffs" / n, args.output / n)
    helpers = {
        p.name: digest(p)
        for p in ROOT.glob("*.py")
        if p.name not in [Path(__file__).name] and not p.name.startswith("test_")
    }
    provenance = dict(
        input_sha256=PINNED,
        raw=sources,
        analysis=dict(
            script_sha256=digest(Path(__file__)),
            protocol_sha256=digest(PROTOCOL),
            simulator_sha256=digest(SIMULATOR),
            helpers_sha256=helpers,
        ),
        historical_regression=reg,
        audit=dict(
            accounting_cases=len(cases),
            fresh_cases=324,
            fresh_selected_states=len(fresh) // 3,
            fresh_pairs=len(fresh),
            fresh_audited_cases=len({identity(r) for r in fresh}),
            fresh_loss_links=sum(r["cohort"] in FRESH for r in links),
            fresh_command_only_links=sum(
                r["cohort"] in FRESH and r["command_only_zero_loss"] for r in links
            ),
            comparison_rows=len(comparison),
        ),
        schema=dict(
            population="original joint-ready adjacent-state audit within all late timeout event windows",
            fresh_seeds=[277, 278, 279],
            training_seeds=[269, 270, 271],
            cutoffs_refit=False,
            strata=STRATUM_NAMES,
            reporting_parent_case_weights="1/n_case before stratum and retention",
            point_truth="preceding physical row",
        ),
    )
    (args.output / "sources.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(json.dumps(provenance["audit"]), flush=True)


if __name__ == "__main__":
    main()
