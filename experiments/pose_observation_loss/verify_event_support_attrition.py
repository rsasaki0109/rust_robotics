#!/usr/bin/env python3
"""Independent raw-vector labels and NumPy pair masks; no analyzer imports."""

import argparse
from collections import Counter, defaultdict
import csv
from fractions import Fraction
import hashlib
from itertools import product
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
KEY = ("cohort", "variant", "mode", "seed", "scenario", "planner")
META = ("case_id", "family_id", *KEY, "partition", "accounting_partition", "noise")
GROUP = ("partition", "noise", "variant", "stratum", "population", "target", "label")
STRATA = (
    "all",
    "recovery_single",
    "filter_single",
    "fusion_three",
    "recovery_fallback_single",
)
TARGETS = ("midpoint", "residual")
LABELS = ("observed", "modeled", "unmodeled")
POPULATIONS = ("source", "common_family")
SCOPES = ("same_event", "le_500ms", "le_100ms", "le_20ms")
REASONS = (
    "no_parent_rp",
    "no_stratum_rp",
    "family_ineligible",
    "no_outside",
    "no_inside",
    "event_separated",
    "gap_excluded",
    "supported",
)
MODES = ("none", "random", "blackout")
SEEDS = {
    "known": ("269", "270", "271"),
    "new": ("274", "275", "276"),
    "fresh": ("277", "278", "279"),
}
SCENARIOS = ("dense_50x50", "maze_50x50", "open_50x50")
PLANNERS = ("A*", "JPS", "Theta*")
checks = 0


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def indexed(rows, keys):
    result = {tuple(r[k] for k in keys): r for r in rows}
    assert len(result) == len(rows)
    return result


def check(actual, expected):
    global checks
    checks += 1
    if expected is None:
        assert actual == "", (actual, expected)
    elif isinstance(expected, bool):
        assert actual == str(expected), (actual, expected)
    elif isinstance(expected, str):
        assert actual == expected, (actual, expected)
    elif isinstance(expected, int):
        assert actual == str(expected), (actual, expected)
    else:
        assert actual != "" and math.isclose(
            float(actual), float(expected), rel_tol=1e-10, abs_tol=2e-12
        ), (actual, expected)


def fields(actual, expected):
    assert set(actual) == set(expected), (
        set(actual) - set(expected),
        set(expected) - set(actual),
    )
    for k, value in expected.items():
        check(actual[k], value)


def microseconds(value):
    value = float(value)
    assert math.isfinite(value)
    rounded = round(value * 1_000_000)
    assert abs(rounded / 1_000_000 - value) <= 1e-7
    return rounded


def outside(vector):
    return math.hypot(*vector) > 0.1


def direct(rows, target, label):
    """All masks are for one full original case, never a pooled class product."""
    assert len({r["case_id"] for r in rows}) <= 1
    positive = np.asarray([r["labels"][target][label] for r in rows], dtype=bool)
    event = np.asarray([r["event_id"] for r in rows], dtype=object)
    time = np.asarray([r["time_us"] for r in rows], dtype=np.int64)
    same = event[positive, None] == event[None, ~positive]
    distance = np.abs(time[positive, None] - time[None, ~positive])
    positive_events, negative_events = set(event[positive]), set(event[~positive])
    answer = dict(
        records=len(rows),
        events=len(set(event)),
        outside_rows=int(positive.sum()),
        inside_rows=int((~positive).sum()),
        positive_only_events=len(positive_events - negative_events),
        negative_only_events=len(negative_events - positive_events),
        case_pair_count=int(positive.sum()) * int((~positive).sum()),
    )
    for scope, maximum in zip(SCOPES, (None, 500_000, 100_000, 20_000)):
        mask = same if maximum is None else same & (distance <= maximum)
        answer[scope + "_pair_count"] = int(mask.sum())
        answer[scope + "_supported_events"] = len(
            set(event[positive][np.any(mask, axis=1)])
        )
    answer["other_event_pair_count"] = (
        answer["case_pair_count"] - answer["same_event_pair_count"]
    )
    return answer


def classify(parent, before, eligible, population, values, scope):
    conditions = (
        parent == 0,
        before == 0,
        population == "common_family" and not eligible,
        values["outside_rows"] == 0,
        values["inside_rows"] == 0,
        values["same_event_pair_count"] == 0,
        values[scope + "_pair_count"] == 0,
        True,
    )
    return next(name for name, condition in zip(REASONS, conditions) if condition)


def load_states(cases, census):
    """Reconstruct E/P/U from original signed pose parts, not saved support labels."""
    by_key = indexed(cases, KEY)
    events = indexed(census, ("event_id",))
    old_p = indexed(
        read(RESULTS / "contrast-common-errors/simultaneous.csv"), ("source_id",)
    )
    old_pose = indexed(
        read(RESULTS / "speed-probe-error-components/components.csv"), ("pose_id",)
    )
    old_s = indexed(
        read(RESULTS / "contrast-risk-transfer/scores.csv"),
        ("source_family", "source_id"),
    )
    new_p = indexed(
        read(RESULTS / "fresh-seed-cutoff-transfer/projections.csv"), ("source_id",)
    )
    new_pose = indexed(
        read(RESULTS / "fresh-seed-cutoff-transfer/poses.csv"),
        (*KEY, "state_id", "pose_kind"),
    )
    focus = []
    for m in read(RESULTS / "estimator-mode-transfer/membership.csv"):
        if m["view"] == "sim_recovery_preview":
            p, score = old_p[(m["source_id"],)], old_s[("simultaneous", m["source_id"])]
            poses = [old_pose[(m[k],)] for k in ("a_id", "b_id")]
            focus.append((m, p, score, poses))
    for m in read(RESULTS / "fresh-seed-cutoff-transfer/scores.csv"):
        if m["view"] == "sim_recovery_preview":
            p = new_p[(m["source_id"],)]
            poses = [
                new_pose[tuple(m[k] for k in KEY) + (m["state_id"], kind)]
                for kind in ("recovery", "preview")
            ]
            focus.append((m, p, m, poses))
    assert len(focus) == 10995
    result = []
    for m, projection, score, poses in focus:
        info = by_key[tuple(m[k] for k in KEY)]
        assert m["partition"] == info["partition"] and m["noise"] == info["noise"]
        assert m["stratum"] in STRATA[1:] and m["seed"] in SEEDS[m["partition"]]
        tags = m["source_event_ids"].split("+")
        assert len(tags) == 1 and tags[0]
        event = events[(tags[0],)]
        for k in KEY:
            check(event[k], m[k])
            check(projection[k], m[k])
            check(score[k], m[k])
        for pose, estimator in zip(poses, (m["a_estimator"], m["b_estimator"])):
            check(pose["estimator"], estimator)
            for k in (*KEY, "state_id", "time_s", "source_event_ids"):
                check(pose[k], projection[k])
            for axis in ("x", "y"):
                assert (
                    abs(
                        math.fsum(
                            float(pose[part + "_" + axis])
                            for part in ("noise", "anchor", "drift", "transport")
                        )
                        - float(pose["error_" + axis])
                    )
                    < 1e-8
                )
        assert (
            projection["source_event_ids"]
            == score["source_event_ids"]
            == m["source_event_ids"]
        )
        labels = {}
        for target in TARGETS:
            weights = (
                (0.5, 0.5)
                if target == "midpoint"
                else (float(projection["a_weight"]), float(projection["b_weight"]))
            )
            vectors = {
                part: [
                    math.fsum(
                        weights[i] * float(poses[i][part + "_" + axis])
                        for i in range(2)
                    )
                    for axis in ("x", "y")
                ]
                for part in ("noise", "anchor", "drift", "transport")
            }
            observed = [float(projection[target + "_" + axis]) for axis in ("x", "y")]
            modeled = [vectors["noise"][j] + vectors["anchor"][j] for j in range(2)]
            unmodeled = [
                vectors["drift"][j] + vectors["transport"][j] for j in range(2)
            ]
            for j in range(2):
                assert (
                    abs(math.fsum(vectors[part][j] for part in vectors) - observed[j])
                    < 1e-8
                )
            labels[target] = dict(
                zip(LABELS, (outside(observed), outside(modeled), outside(unmodeled)))
            )
            check(score[target + "_outside"], labels[target]["observed"])
        result.append(
            dict(
                case_id=info["case_id"],
                family_id=info["family_id"],
                planner=info["planner"],
                stratum=m["stratum"],
                source_id=m["source_id"],
                event_id=tags[0],
                time_us=microseconds(projection["time_s"]),
                labels=labels,
            )
        )
    assert len(indexed(result, ("case_id", "source_id"))) == 10995
    counts = Counter(r["case_id"] for r in result)
    assert len(counts) == 157
    for case in cases:
        check(case["parent_records"], counts[case["case_id"]])
        check(
            case["case_row_weight"],
            Fraction(1, counts[case["case_id"]]) if counts[case["case_id"]] else None,
        )
    return result


def reconstruct(cases, states, census):
    by_case, by_event, eligible = defaultdict(list), defaultdict(list), defaultdict(set)
    case_info = indexed(cases, ("case_id",))
    for r in states:
        by_case[r["case_id"]].append(r)
        by_event[r["event_id"]].append(r)
        for s in ("all", r["stratum"]):
            eligible[r["family_id"], s].add(r["planner"])
    common = {key for key, planners in eligible.items() if planners == set(PLANNERS)}
    case_rows, event_rows, cover = [], [], []
    for info in cases:
        parent = len(by_case[info["case_id"]])
        for stratum, target, label, population in product(
            STRATA, TARGETS, LABELS, POPULATIONS
        ):
            before = [
                r
                for r in by_case[info["case_id"]]
                if stratum == "all" or r["stratum"] == stratum
            ]
            is_common = (info["family_id"], stratum) in common
            selected = before if population == "source" or is_common else []
            values = direct(selected, target, label)
            answer = {k: info[k] for k in META}
            answer.update(
                stratum=stratum,
                target=target,
                label=label,
                population=population,
                parent_records=parent,
                stratum_records=len(before),
                common_family_eligible=is_common,
            )
            answer.update(values)
            answer["case_mass"] = (
                Fraction(len(selected), parent) if parent else Fraction()
            )
            for unit in ("case", "other_event", *SCOPES):
                answer[unit + "_pair_mass"] = (
                    Fraction(values[unit + "_pair_count"], parent * parent)
                    if parent
                    else Fraction()
                )
            for scope in SCOPES:
                answer[scope + "_reason"] = classify(
                    parent, len(before), is_common, population, values, scope
                )
            case_rows.append(answer)
    for event in census:
        rr = by_event[event["event_id"]]
        info = next((r for r in cases if all(r[k] == event[k] for k in KEY)), None)
        assert info is not None
        covered = {k: info[k] for k in META}
        covered.update({k: event[k] for k in ("event_id", "event_kind")})
        for flag in ("scheduled", "received", "precision_activation"):
            assert event[flag] in ("True", "False")
            covered[flag] = event[flag] == "True"
        for stratum in STRATA:
            covered[stratum + "_records"] = sum(
                stratum == "all" or r["stratum"] == stratum for r in rr
            )
        covered["rp_referenced"] = bool(rr)
        cover.append(covered)
        if not rr:
            continue
        assert all(r["case_id"] == info["case_id"] for r in rr)
        parent = len(by_case[info["case_id"]])
        for stratum, target, label, population in product(
            STRATA, TARGETS, LABELS, POPULATIONS
        ):
            before = [r for r in rr if stratum == "all" or r["stratum"] == stratum]
            is_common = (info["family_id"], stratum) in common
            selected = before if population == "source" or is_common else []
            values = direct(selected, target, label)
            answer = {k: info[k] for k in META}
            answer.update(
                event_id=event["event_id"],
                stratum=stratum,
                target=target,
                label=label,
                population=population,
                parent_records=parent,
                stratum_records=len(before),
                common_family_eligible=is_common,
            )
            answer.update(
                {k: values[k] for k in ("records", "outside_rows", "inside_rows")}
            )
            answer["case_mass"] = Fraction(len(selected), parent)
            for scope in SCOPES:
                answer[scope + "_pair_count"] = values[scope + "_pair_count"]
                answer[scope + "_pair_mass"] = Fraction(
                    values[scope + "_pair_count"], parent * parent
                )
                answer[scope + "_reason"] = classify(
                    parent, len(before), is_common, population, values, scope
                )
            event_rows.append(answer)
    return case_rows, event_rows, cover


def summaries(case_rows):
    groups = defaultdict(list)
    for r in case_rows:
        groups[tuple(r[k] for k in GROUP)].append(r)
    result = []
    for key, rows in sorted(groups.items()):
        for mode in (*MODES, "all_modes"):
            rr = rows if mode == "all_modes" else [r for r in rows if r["mode"] == mode]
            r = dict(zip(GROUP, key))
            r.update(
                mode=mode,
                planned_cases=len(rr),
                parent_represented_cases=sum(c["parent_records"] > 0 for c in rr),
                stratum_represented_cases=sum(c["stratum_records"] > 0 for c in rr),
                family_eligible_cases=sum(c["common_family_eligible"] for c in rr),
                selected_cases=sum(c["records"] > 0 for c in rr),
            )
            assert len(rr) == (81 if mode == "all_modes" else 27)
            for field in (
                "records",
                "events",
                "outside_rows",
                "inside_rows",
                "case_mass",
                "case_pair_count",
                "case_pair_mass",
                "other_event_pair_count",
                "other_event_pair_mass",
                "positive_only_events",
                "negative_only_events",
            ):
                r[field] = sum(
                    (c[field] for c in rr), Fraction() if field.endswith("mass") else 0
                )
            for scope in SCOPES:
                for suffix in ("pair_count", "pair_mass", "supported_events"):
                    field = scope + "_" + suffix
                    r[field] = sum(
                        (c[field] for c in rr),
                        Fraction() if suffix == "pair_mass" else 0,
                    )
                r[scope + "_supported_cases"] = sum(
                    c[scope + "_pair_count"] > 0 for c in rr
                )
                for why in REASONS:
                    r[scope + "_cases_" + why] = sum(
                        c[scope + "_reason"] == why for c in rr
                    )
                assert sum(r[scope + "_cases_" + why] for why in REASONS) == len(rr)
                r[scope + "_over_case"] = (
                    r[scope + "_pair_mass"] / r["case_pair_mass"]
                    if r["case_pair_mass"]
                    else None
                )
                if scope != "same_event":
                    r[scope + "_over_event"] = (
                        r[scope + "_pair_mass"] / r["same_event_pair_mass"]
                        if r["same_event_pair_mass"]
                        else None
                    )
            result.append(r)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", required=True, type=Path)
    parser.add_argument("--figures", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not __debug__ or args.output.exists():
        parser.error("Run without -O and use a new evidence file")
    sources = json.loads((args.results / "sources.json").read_text())
    base_sha = "9ade09e0728e4fd3f6a7a0bca821c05b90917d2ea3adf7963ee8ea8f15d98849"
    check(sha(RESULTS / "event-loss-mode-influence/sources.json"), base_sha)
    check(sources["input_sha256"]["event-loss-mode-influence/sources.json"], base_sha)
    for name, h in sources["input_sha256"].items():
        check(sha(RESULTS / name), h)
    for name, h in sources["analysis"]["helpers_sha256"].items():
        check(sha(ROOT / name), h)
    check(
        sha(ROOT / "analyze_event_support_attrition.py"),
        sources["analysis"]["script_sha256"],
    )
    check(
        sha(RESULTS / "event-support-attrition/protocol.md"),
        sources["analysis"]["protocol_sha256"],
    )
    for name, h in sources["output_sha256"].items():
        check(sha(args.results / name), h)
    data = RESULTS / "event-temporal-rankings"
    cases, census = read(data / "cases.csv"), read(data / "events.csv")
    assert len(cases) == 972 and len(census) == 3997
    case_index = indexed(cases, KEY)
    for part, noise, arm, mode, scenario, planner in product(
        SEEDS, ("white", "correlated"), ("epoch", "curved"), MODES, SCENARIOS, PLANNERS
    ):
        for seed in SEEDS[part]:
            assert (
                part + "-" + noise,
                arm,
                mode,
                seed,
                scenario,
                planner,
            ) in case_index
    states = load_states(cases, census)
    cc, ee, coverage = reconstruct(cases, states, census)
    ss = summaries(cc)
    expected_sizes = ((cc, 58320), (ee, 16500), (coverage, 3997), (ss, 2880))
    assert all(len(rows) == size for rows, size in expected_sizes)
    for name, expected, keys in (
        (
            "case-support.csv",
            cc,
            ("case_id", "stratum", "target", "label", "population"),
        ),
        (
            "event-support.csv",
            ee,
            ("event_id", "stratum", "target", "label", "population"),
        ),
        ("event-coverage.csv", coverage, ("event_id",)),
        ("support-summary.csv", ss, (*GROUP, "mode")),
    ):
        actual = indexed(read(args.results / name), keys)
        assert len(actual) == len(expected)
        for r in expected:
            fields(actual[tuple(r[k] for k in keys)], r)
    summary_index = indexed(ss, (*GROUP, "mode"))
    prior_count = 0
    for r in read(RESULTS / "event-loss-mode-influence/mode-effects.csv"):
        s = summary_index[tuple(r[k] for k in GROUP) + (r["unit_id"],)]
        scope = r["scope"]
        for field, value in (
            ("removed_records", s["records"]),
            ("removed_cases", s["selected_cases"]),
            ("removed_events", s["events"]),
            ("mode_pair_count", s[scope + "_pair_count"]),
            ("mode_events_with_pairs", s[scope + "_supported_events"]),
            ("mode_cases_with_pairs", s[scope + "_supported_cases"]),
            ("mode_denominator", s[scope + "_pair_mass"]),
        ):
            check(r[field], value)
            prior_count += 1
    for r in read(RESULTS / "event-score-ties/precision-summary.csv"):
        s = summary_index[tuple(r[k] for k in GROUP) + ("all_modes",)]
        for field, suffix in (
            ("pair_count", "pair_count"),
            ("events_with_pairs", "supported_events"),
            ("cases_with_pairs", "supported_cases"),
            ("denominator", "pair_mass"),
        ):
            check(r[field], s[r["scope"] + "_" + suffix])
            prior_count += 1
    assert prior_count == 216000
    focus = [
        r
        for r in ss
        if r["target"] == "residual"
        and r["stratum"] in ("filter_single", "fusion_three")
        and r["label"] in ("observed", "modeled")
        and r["mode"] in MODES
    ]
    plotted = indexed(read(args.figures / "plot-values.csv"), (*GROUP, "mode"))
    assert len(plotted) == len(focus) == 288
    for r in focus:
        fields(plotted[tuple(r[k] for k in (*GROUP, "mode"))], r)
    plot = json.loads((args.figures / "plot-sources.json").read_text())
    check(sha(ROOT / "plot_event_support_attrition.py"), plot["script_sha256"])
    check(sha(args.results / "sources.json"), plot["analysis_sources_sha256"])
    for name, h in plot["inputs_sha256"].items():
        check(sha(args.results / name), h)
    for name, h in plot["outputs_sha256"].items():
        check(sha(args.figures / name), h)
    audit = dict(
        planned_cases=972,
        families=324,
        source_states=10995,
        source_view_references=32985,
        represented_cases=157,
        census_events=3997,
        referenced_events=275,
        case_support_rows=len(cc),
        event_support_rows=len(ee),
        summary_rows=len(ss),
        preceding_support_comparisons=prior_count,
    )
    for k, value in audit.items():
        check(str(sources["audit"][k]), value)
    proof = dict(
        comparisons=checks,
        reconstructed_states=len(states),
        reconstructed_targets=2 * len(states),
        **audit,
        plotted_rows=len(plotted),
        pair_method="direct within-full-case NumPy event/time masks, Fraction original parent weights",
        script_sha256=sha(Path(__file__)),
        sources_sha256=sha(args.results / "sources.json"),
        plot_sources_sha256=sha(args.figures / "plot-sources.json"),
    )
    args.output.write_text(json.dumps(proof, indent=2) + "\n")
    print(json.dumps(proof, indent=2), flush=True)


if __name__ == "__main__":
    main()
