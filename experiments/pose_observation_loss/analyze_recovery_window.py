#!/usr/bin/env python3
"""Reconstruct the recorded recovery sample windows without a new rollout."""

import argparse
from collections import Counter, defaultdict, deque
import csv
from fractions import Fraction
import hashlib
from itertools import product
import json
import math
from pathlib import Path

import analyze_recovery_geometry as prior

ROOT, REPO, RESULTS = prior.ROOT, prior.REPO, prior.RESULTS
BASE = RESULTS / "fusion-return-recovery-geometry"
STUDY = RESULTS / "fusion-return-recovery-window"
BASE_SHA = "2427ad221774c58a9f74183efb0f7986e33d1fe91935296f9ae858e01a281177"
PROOF_SHA = "9ef57c15aaed541cab0006cf99c87ba792f2f3d98458bbc65e10b79d87acf7b3"
META, KEY, GROUP = prior.META, prior.KEY, prior.GROUP
PHASES, BRANCHES, CATEGORIES = prior.PHASES, prior.BRANCHES, prior.CATEGORIES
H, P = 5e-10, 1e-12
SENSE_RADIUS = 3 * H + 2 * P
KINDS = (
    "audited",
    "mean",
    "fallback",
    "growing",
    "full",
    "evicted",
    "prior_unknown_mean",
    "nominal_only_reset_mean",
)
REFS = tuple(
    f"{p}_{b}_window_{k}_refs" for p, b in product(PHASES, BRANCHES) for k in KINDS
)
HAS = tuple("has_available_window_" + k for k in ("mean", "fallback", "evicted"))
RAW = tuple("prior_truth_" + a for a in ("x", "y", "yaw")) + tuple(
    p + a for p in ("noise_", "bias_") for a in ("x", "y", "yaw")
)
FIELDS = (
    "window_kind",
    "window_member_steps",
    "window_replayed_count",
    "window_last_clear_step",
    "window_last_clear_reasons",
    "window_pushes_since_clear",
    "window_evictions_since_clear",
    *(f"mean_{a}_{s}" for a in ("x", "y") for s in ("center", "lo", "hi")),
    "mean_yaw_center",
    "mean_yaw_radius",
    "mean_yaw_enclosure",
    "mean_log_x_residual",
    "mean_log_y_residual",
    "mean_log_yaw_error",
    *("rounded_mean_minus_current_" + a for a in ("x", "y")),
    *(f"{c}_delta_{a}" for c in ("truth", "noise", "bias") for a in ("x", "y")),
    *(c + "_delta_norm" for c in ("truth", "noise", "bias")),
    "decomposition_x_residual",
    "decomposition_y_residual",
)
sha, read, write, flag, step = (
    prior.sha,
    prior.read,
    prior.write,
    prior.flag,
    prior.step,
)


def contract(source):
    fragment = source[
        source.index(
            "            if averaged_recovery {",
            source.index("let sensed = if position_sigma"),
        ) : source.index("            let preview_end = Point2D::new(")
    ]
    for text in (
        "correction_received",
        "precision_activation",
        "(correction && !correction_received)",
        "sensed.v.abs() > 0.02",
        "observation_age > 1.5",
        "safety_paused && sensed.v.abs() <= 0.02 && observation_age <= 1.5",
        "recovery_samples.len() > 25",
        "recovery_samples.pop_front()",
        "recovery_samples.push_back((sensed.x, sensed.y, sensed.yaw))",
        "if averaged_recovery && recovery_sample_count > 0",
        "sin_yaw.atan2(cos_yaw)",
        "(sensed.x, sensed.y, sensed.yaw)",
    ):
        assert text in fragment
    assert (
        fragment.index("recovery_samples.clear()")
        < fragment.index("recovery_samples.push_back")
        < fragment.index("recovery_samples.pop_front()")
        < fragment.index("let recovery_sample_count")
    )
    assert "const DT: f64 = 0.02;" in source
    assert (
        source.index("if correction_received || precision_activation {")
        < source.index(
            "let observation_age = observation_time - last_observation_time;"
        )
        < source.index("// After a fresh correction")
    )
    return dict(
        window_fragment_sha256=hashlib.sha256(fragment.encode()).hexdigest(),
        time_arithmetic="binary64 n*0.02-last_observation_step*0.02",
    )


def fold(values):
    total = 0.0
    for value in values:
        total += value
    return total


def circular_error(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def sample(n, before, current, epoch):
    raw = {"prior_truth_" + a: before[a] for a in ("x", "y", "yaw")}
    raw.update({k: current[k] for k in RAW if not k.startswith("prior_truth_")})
    assert all(math.isfinite(float(v)) and abs(float(v)) <= 100 for v in raw.values())
    centers = {
        a: (float(raw["prior_truth_" + a]) + float(raw["noise_" + a]))
        + float(raw["bias_" + a])
        for a in ("x", "y", "yaw")
    }
    assert all(abs(v) <= 100 for v in centers.values())
    return dict(
        source_step=n,
        source_epoch_step=epoch,
        **raw,
        **{"sensed_" + a + "_center": v for a, v in centers.items()},
    )


class Window:
    def __init__(self, initial):
        self.members = deque()
        self.last_observation = 0
        self.prior_paused = flag(initial["safety_paused"], raw=True)
        self.last_clear = 0
        self.clear_reasons = ""
        self.pushes = 0
        self.evictions = 0

    def update(self, n, before, current):
        received, precision, correction = (
            flag(current[k], raw=True)
            for k in ("correction_received", "precision_activation", "correction")
        )
        assert not received or correction
        assert flag(current["stationary_epoch_fusion"], raw=True)
        if received or precision:
            self.last_observation = n
        age = n * 0.02 - self.last_observation * 0.02
        assert abs(age - float(current["observation_age"])) <= 5e-7 + P
        speed = float(current["measured_speed"])
        assert math.isfinite(speed)
        paused = self.prior_paused or (correction and not received)
        reasons = [
            name
            for name, yes in (
                ("received", received),
                ("precision", precision),
                ("missed", correction and not received),
                ("motion", abs(speed) > 0.02),
                ("age", age > 1.5),
            )
            if yes
        ]
        if reasons:
            self.members.clear()
            self.last_clear = n
            self.clear_reasons = ";".join(reasons)
            self.pushes = self.evictions = 0
        current_sample = sample(n, before, current, self.last_observation)
        if paused and abs(speed) <= 0.02 and age <= 1.5:
            self.members.append(current_sample)
            self.pushes += 1
            if len(self.members) > 25:
                self.members.popleft()
                self.evictions += 1
        assert len(self.members) == int(current["recovery_sample_count"]), (
            n,
            len(self.members),
            current["recovery_sample_count"],
        )
        self.prior_paused = flag(current["safety_paused"], raw=True)
        return current_sample


def reconstruct(window, current_sample, current):
    members = list(window.members)
    effective = members or [current_sample]
    n = len(effective)
    out = dict(
        window_kind="mean" if members else "fallback",
        window_member_steps=";".join(str(s["source_step"]) for s in members),
        window_replayed_count=len(members),
        window_last_clear_step=window.last_clear,
        window_last_clear_reasons=window.clear_reasons,
        window_pushes_since_clear=window.pushes,
        window_evictions_since_clear=window.evictions,
    )
    for axis in ("x", "y"):
        center = fold(s["sensed_" + axis + "_center"] for s in effective) / n
        radius = SENSE_RADIUS + (n * P) / n + P if members else SENSE_RADIUS
        lo, hi = center - radius, center + radius
        logged = float(current["recovery_" + axis])
        assert lo <= logged + H and hi >= logged - H, (axis, lo, hi, logged)
        out.update(
            {
                f"mean_{axis}_center": center,
                f"mean_{axis}_lo": lo,
                f"mean_{axis}_hi": hi,
                f"mean_log_{axis}_residual": center - logged,
            }
        )
        difference = center - current_sample["sensed_" + axis + "_center"]
        out["rounded_mean_minus_current_" + axis] = difference
        for component, prefix in (
            ("truth", "prior_truth_"),
            ("noise", "noise_"),
            ("bias", "bias_"),
        ):
            out[f"{component}_delta_{axis}"] = fold(
                float(s[prefix + axis]) for s in effective
            ) / n - float(current_sample[prefix + axis])
        residual = difference - fold(
            out[f"{c}_delta_{axis}"] for c in ("truth", "noise", "bias")
        )
        assert abs(residual) <= P
        out[f"decomposition_{axis}_residual"] = residual
    for c in ("truth", "noise", "bias"):
        out[c + "_delta_norm"] = math.hypot(out[c + "_delta_x"], out[c + "_delta_y"])
    if members:
        sine = fold(math.sin(s["sensed_yaw_center"]) for s in members) / n
        cosine = fold(math.cos(s["sensed_yaw_center"]) for s in members) / n
        vector_radius = math.sqrt(2) * (SENSE_RADIUS + P + P + P)
        norm = math.hypot(sine, cosine)
        bounded = norm > 0 and vector_radius <= norm / 2
        yaw = math.atan2(sine, cosine) if bounded else 0.0
        radius = 2 * vector_radius / norm + P if bounded else math.pi
    else:
        yaw = current_sample["sensed_yaw_center"]
        radius = SENSE_RADIUS
        bounded = True
    error = circular_error(yaw, float(current["recovery_yaw"]))
    assert error <= radius + H + P
    out.update(
        mean_yaw_center=yaw,
        mean_yaw_radius=radius,
        mean_yaw_enclosure="bounded_circle" if bounded else "full_circle",
        mean_log_yaw_error=error,
    )
    assert set(out) == set(FIELDS)
    return out, effective


def lineage(d, fields, effective):
    mean = fields["window_kind"] == "mean"
    return [
        dict(
            **{k: d[k] for k in (*META, "event_id", "time_s", "phase", "branch")},
            sample_role="retained" if mean else "raw_fallback",
            sample_order=i if mean else 0,
            source_time_s=s["source_step"] / 50,
            **s,
        )
        for i, s in enumerate(effective, 1)
    ]


def collect(cases, selected, raw, roots):
    files = defaultdict(dict)
    wanted = defaultdict(list)
    for c in cases:
        files[c["cohort"], f"{c['variant']}-{c['mode']}-{c['seed']}/trajectories.csv"][
            c["scenario"], c["planner"]
        ] = c
    for d in selected:
        wanted[d["case_id"], step(d["time_s"])].append(d)
    selected_cases = {d["case_id"] for d in selected}
    assert set(files) == {(p, n) for p, s in raw.items() for n in s["raw_sha256"]}
    snapshots = {}
    audit = Counter()
    for i, ((cohort, name), members) in enumerate(sorted(files.items()), 1):
        path = Path(roots.get(cohort, raw[cohort]["path"])) / name
        assert sha(path) == raw[cohort]["raw_sha256"][name]
        audit["raw_bytes"] += path.stat().st_size
        last = {}
        previous = {}
        windows = {}
        with path.open(newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
            assert len(header) == len(set(header))
            pos = {
                k: header.index(k)
                for k in (
                    "scenario",
                    "planner",
                    "time_s",
                    "effective_position_sigma",
                    "effective_yaw_sigma_rad",
                )
            }
            for cells in reader:
                assert len(cells) == len(header)
                identity = cells[pos["scenario"]], cells[pos["planner"]]
                assert identity in members
                c = members[identity]
                n = step(cells[pos["time_s"]])
                assert n == last.get(identity, -1) + 1
                last[identity] = n
                if not flag(c["selected_timeout"]):
                    continue
                audit["timeout_grid_rows"] += 1
                if n == 0:
                    prior.prior.previous.noisy_branch(
                        {k: cells[j] for k, j in pos.items()}
                    )
                    audit["noisy_branch_histories"] += 1
                if c["case_id"] not in selected_cases:
                    continue
                row = dict(zip(header, cells))
                if n == 0:
                    windows[identity] = Window(row)
                    audit["replayed_histories"] += 1
                else:
                    window = windows[identity]
                    point = window.update(n, previous[identity], row)
                    audit["replayed_steps"] += 1
                    if (c["case_id"], n) in wanted:
                        fields, effective = reconstruct(window, point, row)
                        for d in wanted[c["case_id"], n]:
                            for k in (*prior.INPUTS, "speed_margin"):
                                assert float(d[k]) == float(row[k]), k
                            for axis in ("x", "y"):
                                assert math.isclose(
                                    float(d["sensed_" + axis + "_center"]),
                                    point["sensed_" + axis + "_center"],
                                    abs_tol=2e-12,
                                )
                                for prefix in ("noise_", "bias_"):
                                    assert d[prefix + axis] == row[prefix + axis]
                                assert (
                                    d["prior_truth_" + axis] == previous[identity][axis]
                                )
                            snapshots[d["event_id"], d["time_s"]] = (
                                fields,
                                lineage(d, fields, effective),
                            )
                previous[identity] = row
        assert set(last) == set(members)
        for identity, c in members.items():
            assert last[identity] == (
                9000 if flag(c["selected_timeout"]) else step(c["time_s"])
            )
        audit["raw_files"] += 1
        if i % 18 == 0:
            print(f"Recovery windows read: {i}/108 files", flush=True)
    assert snapshots.keys() == {(d["event_id"], d["time_s"]) for d in selected}
    assert audit["replayed_histories"] == len(selected_cases)
    return snapshots, dict(audit)


def anchor(e, dd, ne, nr):
    out = dict(e, **prior.prior.weights(e, ne, nr), **dict.fromkeys((*REFS, *HAS)))
    if not flag(e["returned"] or "False"):
        assert not dd
        return out
    assert all(all(d[k] == e[k] for k in META) for d in dd)
    counts = Counter()
    for d in dd:
        if not d["window_audited"]:
            continue
        kinds = ["audited", d["window_kind"]]
        if d["window_kind"] == "mean":
            kinds.append("full" if d["window_replayed_count"] == 25 else "growing")
            if d["window_evictions_since_clear"]:
                kinds.append("evicted")
            if d["prior_certainty"] == "unknown":
                kinds.append("prior_unknown_mean")
            if d["timer_action"] == "reset" and d["refined_failed_gates"] == "nominal":
                kinds.append("nominal_only_reset_mean")
        for k in kinds:
            counts[f"{d['phase']}_{d['branch']}_window_{k}_refs"] += 1
    out.update({k: counts[k] for k in REFS})
    for kind in ("mean", "fallback", "evicted"):
        out["has_available_window_" + kind] = any(
            d["phase"] == "available"
            and d["window_audited"]
            and (
                d["window_evictions_since_clear"] > 0
                if kind == "evicted"
                else d["window_kind"] == kind
            )
            for d in dd
        )
    return out


def closure(rows):
    db = {tuple(r[k] for k in GROUP): r for r in rows}
    assert len(db) == len(rows)
    checks = 0
    for key, r in db.items():
        for i, whole, parts in (
            (3, "all_modes", ("none", "random", "blackout")),
            (4, "all", CATEGORIES),
        ):
            if key[i] == whole:
                for field in (
                    *REFS,
                    *(h + s for h in HAS for s in ("_events", "_case_mass")),
                ):
                    assert r[field] == sum(
                        db[key[:i] + (p,) + key[i + 1 :]][field] for p in parts
                    )
                    checks += 1
        for p, b in product(PHASES, BRANCHES):
            prefix = f"{p}_{b}_window_"
            assert (
                r[prefix + "audited_refs"]
                == r[prefix + "mean_refs"] + r[prefix + "fallback_refs"]
            )
            assert (
                r[prefix + "mean_refs"]
                == r[prefix + "growing_refs"] + r[prefix + "full_refs"]
            )
            assert r[prefix + "evicted_refs"] <= r[prefix + "full_refs"]
    return checks


def summaries(old, anchors):
    rows = []
    for fixed in old:
        row = dict(fixed)
        p, n, arm, mode, category = (row[k] for k in GROUP)
        aa = [
            a
            for a in anchors
            if (a["partition"], a["noise"], a["variant"]) == (p, n, arm)
            and (mode == "all_modes" or a["mode"] == mode)
            and (category == "all" or a["anchor_category"] == category)
        ]
        returned = [a for a in aa if flag(a["returned"] or "False")]
        mass = lambda rr: sum((a["census_case_weight"] for a in rr), Fraction())
        row.update({k: sum(a[k] or 0 for a in aa) for k in REFS})
        for name in HAS:
            selected = [a for a in aa if a[name]]
            row.update(
                {
                    name + "_events": len(selected),
                    name + "_case_mass": mass(selected),
                    name
                    + "_fraction": (
                        Fraction(len(selected), len(returned)) if returned else None
                    ),
                    name
                    + "_case_mass_fraction": (
                        mass(selected) / mass(returned) if returned else None
                    ),
                }
            )
        assert math.isclose(
            float(fixed["census_case_mass"]), float(mass(aa)), abs_tol=2e-12
        )
        rows.append(row)
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--raw-roots", type=Path)
    args = p.parse_args()
    if not __debug__ or args.output.exists():
        p.error("Use a new output directory without -O")
    freeze = json.loads((STUDY / "freeze-pins.json").read_text())
    assert freeze["sources_sha256"] == sha(BASE / "sources.json") == BASE_SHA
    assert (
        freeze["proof_sha256"]
        == sha(BASE / "independent-verification.json")
        == PROOF_SHA
    )
    source = json.loads((BASE / "sources.json").read_text())
    proof = json.loads((BASE / "independent-verification.json").read_text())
    pins = {
        **source["input_sha256"],
        **{
            "fusion-return-recovery-geometry/" + n: h
            for n, h in source["output_sha256"].items()
        },
        "fusion-return-recovery-geometry/sources.json": BASE_SHA,
        "fusion-return-recovery-geometry/independent-verification.json": PROOF_SHA,
        "fusion-return-recovery-geometry/protocol.md": source["analysis"][
            "protocol_sha256"
        ],
        "fusion-return-recovery-geometry/freeze-pins.json": source["analysis"][
            "freeze_sha256"
        ],
    }
    helpers = {
        **source["analysis"]["helpers_sha256"],
        "analyze_recovery_geometry.py": source["analysis"]["script_sha256"],
    }
    vhelpers = {
        **source["analysis"]["verifier_helpers_sha256"],
        "verify_recovery_geometry.py": freeze["verifier_helper_sha256"],
    }
    for name, h in pins.items():
        assert sha(RESULTS / name) == h, name
    for name, h in {**helpers, **vhelpers}.items():
        assert sha(ROOT / name) == h, name
    for name, h in source["analysis"]["code_sha256"].items():
        assert sha(REPO / name) == h, name
    source_contract = contract(
        (REPO / next(iter(source["analysis"]["code_sha256"]))).read_text()
    )
    roots = json.loads(args.raw_roots.read_text()) if args.raw_roots else {}
    assert isinstance(roots, dict) and set(roots) <= set(source["raw"])
    prior.prior.previous.provenance(source["raw"], roots, proof["native"]["rustc"])
    cases = read(RESULTS / "census-rp-selection/case-selection.csv")
    old = read(BASE / "recovery-geometry-decisions.csv")
    selected = [d for d in old if float(d["speed_free_distance"]) == 0]
    assert len(selected) == 1096
    snapshots, audit = collect(cases, selected, source["raw"], roots)
    decisions = []
    lineages = []
    for d in old:
        audited = float(d["speed_free_distance"]) == 0
        row = dict(d, window_audited=audited, **dict.fromkeys(FIELDS))
        if audited:
            fields, lines = snapshots[d["event_id"], d["time_s"]]
            row.update(fields)
            lineages.extend(lines)
        decisions.append(row)
    events = read(BASE / "anchor-recovery-geometry.csv")
    ne = Counter(e["case_id"] for e in events)
    nr = Counter(e["case_id"] for e in events if flag(e["received"]))
    db = defaultdict(list)
    for d in decisions:
        db[d["event_id"]].append(d)
    anchors = [
        anchor(e, db[e["event_id"]], ne[e["case_id"]], nr[e["case_id"]]) for e in events
    ]
    summary = summaries(read(BASE / "recovery-geometry-summary.csv"), anchors)
    audit.update(
        planned_cases=len(cases),
        planned_families=len({c["family_id"] for c in cases}),
        timeout_cases=len(ne),
        census_events=len(anchors),
        returned_anchors=sum(flag(e["returned"] or "False") for e in events),
        decision_references=len(decisions),
        audited_zero_probes=len(selected),
        mean_references=sum(d.get("window_kind") == "mean" for d in decisions),
        fallback_references=sum(d.get("window_kind") == "fallback" for d in decisions),
        lineage_rows=len(lineages),
        summary_rows=len(summary),
        new_additive_closure_comparisons=closure(summary),
    )
    assert (
        len(cases),
        len(ne),
        len(anchors),
        len(decisions),
        len(summary),
        audit["mean_references"],
        audit["fallback_references"],
    ) == (972, 570, 3997, 3751, 336, 921, 175)
    assert (
        sum(a["census_case_weight"] for a in anchors)
        == sum(a["received_case_weight"] for a in anchors)
        == 570
    )
    assert (
        sum(a["rp_case_mass"] for a in anchors) == 157
        and sum(int(a["rp_records"]) for a in anchors) == 10995
    )
    args.output.mkdir(parents=True)
    for name, rows in (
        ("window-decisions", decisions),
        ("window-lineage", lineages),
        ("anchor-window", anchors),
        ("window-summary", summary),
    ):
        write(args.output / (name + ".csv"), rows)
    out = dict(
        input_sha256=pins,
        raw=source["raw"],
        map_sha256=source["map_sha256"],
        analysis=dict(
            script_sha256=sha(Path(__file__)),
            protocol_sha256=sha(STUDY / "protocol.md"),
            freeze_sha256=sha(STUDY / "freeze-pins.json"),
            helpers_sha256=helpers,
            verifier_helpers_sha256=vhelpers,
            code_sha256=source["analysis"]["code_sha256"],
            source_contract=source_contract,
        ),
        schema=dict(
            half_unit=H,
            elemental_allowance=P,
            sensed_radius=SENSE_RADIUS,
            closure_allowance=P,
            phases=PHASES,
            branches=BRANCHES,
            retrospective=True,
            inherited="all prior columns and outcomes; new replay only for selected histories; rounded-input decomposition, no causal intervention",
        ),
        inherited_recovery_geometry_proof_sha256=PROOF_SHA,
        inherited_validation=source["inherited_validation"],
        audit=audit,
        output_sha256={p.name: sha(p) for p in sorted(args.output.iterdir())},
    )
    (args.output / "sources.json").write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps(audit, indent=2), flush=True)


if __name__ == "__main__":
    main()
