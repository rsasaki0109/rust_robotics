"""Check paired trajectories, physical invariants, and recorded outcomes."""

from pathlib import Path
from collections import deque
import csv
import hashlib
import json
import math
import sys

from verify_restart_consistency import check_restart_track

if not __debug__:
    raise RuntimeError("Run the verifier without Python -O; assertions are required")

run = Path(sys.argv[1]).resolve()
root = Path(__file__).resolve().parent
provenance = json.loads((run / "provenance.json").read_text())
noise_tau = provenance.get("noise_correlation_time_s", 0.0)
assert math.isfinite(noise_tau) and noise_tau >= 0.0
noise_rho = math.exp(-0.02 / noise_tau) if noise_tau > 0 else 0.0
innovation_scale = math.sqrt(1.0 - noise_rho * noise_rho)
rows = list(csv.DictReader((run / "all-results.csv").open()))
seeds = {int(r["seed"]) for r in rows}
variants = [
    v
    for v in [
        "baseline",
        "aware",
        "averaged",
        "guarded",
        "consistent",
        "clearance",
        "paced",
    ]
    if any(r["variant"] == v for r in rows)
]
assert variants in [
    ["baseline", "aware"],
    ["aware", "averaged"],
    ["averaged", "guarded"],
    ["guarded", "consistent"],
    ["guarded", "clearance"],
    ["clearance", "paced"],
]
if "clearance" in variants:
    from verify_clearance_paths import audit

    audit(run / "planning-inputs")
    for variant in variants:
        name = "fine-nominal" if variant == "guarded" else "fine-padding"
        directory = run / "planning-inputs" / name
        assert provenance["inputs_by_variant_sha256"][variant] == {
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in directory.glob("*.csv")
        }
if "paced" in variants:
    assert (
        provenance["inputs_by_variant_sha256"]["clearance"]
        == provenance["inputs_by_variant_sha256"]["paced"]
    )
reference_variant = variants[0]
assert len(rows) == 9 * 3 * len(variants) * len(seeds)
goals = {"open_50x50": (48, 48), "dense_50x50": (48, 48), "maze_50x50": (35, 45)}
events = []
for item in json.loads((run / "manifest.json").read_text()):
    if item["variant"] == "baseline":
        continue
    groups = {}
    pairs = {}
    for r in csv.DictReader((run / item["directory"] / "trajectories.csv").open()):
        groups.setdefault((r["scenario"], r["planner"]), []).append(r)
    for r in csv.DictReader(
        (
            run
            / f'{reference_variant}-{item["mode"]}-{item["seed"]}'
            / "trajectories.csv"
        ).open()
    ):
        pairs.setdefault((r["scenario"], r["planner"]), []).append(r)
    for key, track in groups.items():
        m = next(
            r
            for r in rows
            if r["variant"] == item["variant"]
            and r["mode"] == item["mode"]
            and int(r["seed"]) == item["seed"]
            and (r["scenario"], r["planner"]) == key
        )
        if "consistency_ready" in track[0]:
            check_restart_track(track, item["variant"] == "consistent")
        goal = goals[key[0]]
        hold = 0.0
        pauses = 0
        resumes = 0
        paused_time = 0.0
        last_observation = 0.0
        rms_integral = 0.0
        clearance = float(track[0]["step_min_obstacle_distance"])
        recovery_samples = deque(maxlen=25)
        previous_normalized_noise = [0.0, 0.0, 0.0]
        guard_seconds = 0.0
        guard_entries = 0
        guard_exits = 0
        frozen_steering = 0.0
        speed_limited_seconds = 0.0
        speed_braking_seconds = 0.0
        for i, r in enumerate(track[1:], 1):
            before = track[i - 1]
            t = float(r["time_s"])
            v = float(before["v"])
            speed = float(r["v"])
            assert abs(t - float(before["time_s"]) - 0.02) < 1e-8
            if "noise_rho" in r:
                assert abs(float(r["noise_rho"]) - noise_rho) < 1e-12
                precise = r["precision_active"] == "true"
                for axis, (field, sigma) in enumerate(
                    [
                        ("x", 0.03 if precise else 0.1),
                        ("y", 0.03 if precise else 0.1),
                        ("yaw", math.radians(0.6 if precise else 2)),
                    ]
                ):
                    latent = float(r[f"noise_{field}"]) / sigma
                    innovation = float(r[f"noise_innovation_{field}"])
                    expected = (
                        innovation
                        if i == 1 or noise_rho == 0
                        else (
                            noise_rho * previous_normalized_noise[axis]
                            + innovation_scale * innovation
                        )
                    )
                    assert abs(latent - expected) < 2e-7
                    previous_normalized_noise[axis] = latent
            else:
                assert noise_tau == 0.0
            available = (
                float(r["loss_uniform"]) >= 0.3
                if item["mode"] == "random"
                else not 45 <= t < 75 if item["mode"] == "blackout" else True
            )
            assert (r["observation_available"] == "true") == available
            assert (r["correction"] == "true") == (i % 250 == 0)
            assert (r["correction_received"] == "true") == (i % 250 == 0 and available)
            assert all(
                math.isfinite(float(r[k]))
                for k in [
                    "x",
                    "y",
                    "yaw",
                    "v",
                    "tracking_error",
                    "step_min_obstacle_distance",
                    "steering",
                    "acceleration",
                    "recovery_clearance",
                ]
            )
            if (
                r["correction_received"] == "true"
                or r["precision_activation"] == "true"
            ):
                last_observation = t
            assert abs(float(r["observation_age"]) - (t - last_observation)) < 1e-8
            paused = r["safety_paused"] == "true"
            was_paused = before["safety_paused"] == "true"
            if (
                r["correction"] == "true"
                and r["correction_received"] == "false"
                and not was_paused
            ):
                pauses += 1
            if item["variant"] in [
                "averaged",
                "guarded",
                "consistent",
                "clearance",
                "paced",
            ]:
                if (
                    r["correction_received"] == "true"
                    or r["precision_activation"] == "true"
                    or (
                        r["correction"] == "true"
                        and r["correction_received"] == "false"
                    )
                    or abs(v) > 0.02
                    or float(r["observation_age"]) > 1.5
                ):
                    recovery_samples.clear()
                # The row contains the control decision before its physical update.
                entering_pause = (
                    r["correction"] == "true"
                    and r["correction_received"] == "false"
                    and not was_paused
                )
                if (
                    (was_paused or entering_pause)
                    and abs(v) <= 0.02
                    and float(r["observation_age"]) <= 1.5
                ):
                    recovery_samples.append(
                        (
                            float(before["x"])
                            + float(r["noise_x"])
                            + float(r["bias_x"]),
                            float(before["y"])
                            + float(r["noise_y"])
                            + float(r["bias_y"]),
                            float(before["yaw"])
                            + float(r["noise_yaw"])
                            + float(r["bias_yaw"]),
                        )
                    )
                assert len(recovery_samples) == int(r["recovery_sample_count"])
                if recovery_samples:
                    mean_x = sum(p[0] for p in recovery_samples) / len(recovery_samples)
                    mean_y = sum(p[1] for p in recovery_samples) / len(recovery_samples)
                    yaw_mean = math.atan2(
                        sum(math.sin(p[2]) for p in recovery_samples),
                        sum(math.cos(p[2]) for p in recovery_samples),
                    )
                    assert abs(mean_x - float(r["recovery_x"])) < 2e-9
                    assert abs(mean_y - float(r["recovery_y"])) < 2e-9
                    assert (
                        abs(
                            math.atan2(
                                math.sin(yaw_mean - float(r["recovery_yaw"])),
                                math.cos(yaw_mean - float(r["recovery_yaw"])),
                            )
                        )
                        < 2e-9
                    )
                if r["resumed"] == "true":
                    assert len(recovery_samples) >= 10
                    assert float(track[i - 49]["recovery_evidence"]) >= 0.02 - 1e-8
            if r["resumed"] == "true":
                resumes += 1
                assert (
                    was_paused
                    and not paused
                    and float(r["observation_age"]) <= 1.5
                    and abs(v) <= 0.02
                )
                margin = (
                    0.6
                    + 2 * math.sqrt(2) * float(r["effective_position_sigma"])
                    + 0.02 * float(r["observation_age"])
                )
                assert float(r["recovery_clearance"]) > margin
                assert all(
                    float(s["recovery_evidence"]) >= 0.02 - 1e-8
                    for s in track[i - 49 : i]
                )
            if paused:
                paused_time += 0.02
                assert float(r["target_speed"]) == 0 and float(r["steering"]) == 0
            guarded = r.get("guard_active", "false") == "true"
            if item["variant"] in ["guarded", "consistent", "clearance", "paced"]:
                margin = (
                    0.5
                    + 2 * math.sqrt(2) * float(r["effective_position_sigma"])
                    + 0.02 * float(r["observation_age"])
                )
                assert abs(margin - float(r["preview_margin"])) < 3e-9
                previously_guarded = before.get("guard_active", "false") == "true"
                if r["guard_triggered"] == "true":
                    guard_entries += 1
                    assert not previously_guarded and guarded
                    assert float(r["preview_clearance_lower_bound"]) <= margin + 2e-9
                    frozen_steering = float(r["nominal_steering"])
                if r["guard_released"] == "true":
                    guard_exits += 1
                    assert previously_guarded and not guarded and abs(v) <= 0.02
                    assert (
                        float(r["preview_clearance_lower_bound"]) > margin + 0.05 - 2e-9
                    )
                    assert all(
                        float(s["guard_release_evidence"]) >= 0.02 - 1e-8
                        for s in track[i - 24 : i]
                    )
                if guarded:
                    guard_seconds += 0.02
                    assert float(r["target_speed"]) == 0
                    assert (
                        abs(float(r["steering"]) - (0 if paused else frozen_steering))
                        < 2e-9
                    )
                else:
                    assert r["target_speed"] == r["nominal_target_speed"]
                    assert r["steering"] == r["nominal_steering"]
            else:
                assert not guarded
            speed_braking = r.get("speed_braking", "false") == "true"
            if "speed_cap" in r:
                free = float(r["speed_free_distance"])
                cap = float(r["speed_cap"])
                enabled = item["variant"] == "paced"
                assert 0 <= free <= 2 and abs(free / 0.05 - round(free / 0.05)) < 2e-8
                expected_cap = (
                    min(1, max(0, math.sqrt(0.0225 + free) - 0.15)) if enabled else 1
                )
                assert abs(cap - expected_cap) < 2e-9
                margin = (
                    0.6
                    + 2 * math.sqrt(2) * float(r["effective_position_sigma"])
                    + 0.02 * float(r["observation_age"])
                    if enabled
                    else 0
                )
                assert abs(float(r["speed_margin"]) - margin) < 3e-9
                if abs(v - cap - 1e-9) > 2e-9:
                    assert speed_braking == (enabled and v > cap + 1e-9)
                assert (
                    abs(
                        float(r["nominal_target_speed"])
                        - min(float(r["uncapped_target_speed"]), cap)
                    )
                    < 2e-9
                )
                if not enabled:
                    assert free == 2 and not speed_braking
                if not (guarded or paused or r["stop_latched"] == "true"):
                    speed_limited_seconds += (
                        0.02 if cap < float(r["uncapped_target_speed"]) - 1e-8 else 0.0
                    )
                    speed_braking_seconds += 0.02 if speed_braking else 0.0
            brake = paused or r["stop_latched"] == "true" or guarded or speed_braking
            expected = (
                max(-1, min(0, -4 * v)) if brake else float(r["target_speed"]) - v
            )
            assert abs(float(r["acceleration"]) - expected) < 4e-9
            assert abs(v + expected * 0.02 - speed) < 2e-9
            assert (
                abs(
                    float(r["x"])
                    - (float(before["x"]) + v * math.cos(float(before["yaw"])) * 0.02)
                )
                < 2e-9
            )
            assert (
                abs(
                    float(r["y"])
                    - (float(before["y"]) + v * math.sin(float(before["yaw"])) * 0.02)
                )
                < 2e-9
            )
            dist = math.hypot(float(r["x"]) - goal[0], float(r["y"]) - goal[1])
            hold = (
                hold + 0.02
                if r["stop_latched"] == "true" and dist < 0.25 and abs(speed) <= 0.02
                else 0.0
            )
            assert abs(hold - float(r["hold_time_s"])) < 1e-7
            clearance = min(clearance, float(r["step_min_obstacle_distance"]))
            rms_integral += (
                float(before["tracking_error"]) ** 2 + float(r["tracking_error"]) ** 2
            ) * 0.01
            if i < len(pairs[key]):
                ref = pairs[key][i]
                assert (
                    ref["observation_available"] == r["observation_available"]
                    and ref["loss_uniform"] == r["loss_uniform"]
                )
                for field, yaw in [
                    ("noise_x", False),
                    ("noise_y", False),
                    ("noise_yaw", True),
                ]:

                    def sigma(a):
                        return (
                            math.radians(0.6 if a["precision_active"] == "true" else 2)
                            if yaw
                            else (0.03 if a["precision_active"] == "true" else 0.1)
                        )

                    assert (
                        abs(float(r[field]) / sigma(r) - float(ref[field]) / sigma(ref))
                        < 2e-7
                    )
                    if "noise_rho" in r and "noise_rho" in ref:
                        assert (
                            r[f"noise_innovation_{field.removeprefix('noise_')}"]
                            == ref[f"noise_innovation_{field.removeprefix('noise_')}"]
                        )
                    if r["correction_received"] == "true":
                        bias_field = field.replace("noise_", "bias_")
                        assert (
                            abs(
                                float(r[bias_field]) / sigma(r)
                                - float(ref[bias_field]) / sigma(ref)
                            )
                            < 2e-7
                        )
        assert pauses == int(m["pause_count"]) and resumes == int(m["resume_count"])
        assert abs(paused_time - float(m["paused_time_s"])) < 1e-7
        assert (
            abs(
                math.sqrt(rms_integral / float(m["time_s"]))
                - float(m["rms_tracking_error"])
            )
            < 1e-7
        )
        assert abs(clearance - float(m["min_obstacle_distance"])) < 1e-7
        if m["held"] == "true":
            assert hold >= 10 - 1e-8 and clearance > 0.5
        if m["collision"] == "true":
            assert clearance <= 0.5
        events.append(
            dict(
                variant=item["variant"],
                guard_entries=guard_entries,
                guard_exits=guard_exits,
                guard_time_s=round(guard_seconds, 6),
                mode=item["mode"],
                seed=item["seed"],
                scenario=key[0],
                planner=key[1],
                pauses=pauses,
                resumes=resumes,
                paused_time_s=round(paused_time, 6),
                status=m["status"],
                contact_while_paused=(
                    track[-1]["safety_paused"] if m["collision"] == "true" else ""
                ),
                last_observation_age=track[-1]["observation_age"],
            )
        )
        if "paced" in variants:
            events[-1].update(
                speed_limited_time_s=round(speed_limited_seconds, 6),
                speed_braking_time_s=round(speed_braking_seconds, 6),
            )
with (run / "control-events.csv").open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(events[0]), lineterminator="\n")
    w.writeheader()
    w.writerows(events)
golden_paths = [root / "results" / "all-results.csv"] + list(
    (root / "results" / "recovery-averaging").glob("*-results.csv")
)
if noise_tau != 0.0:
    golden_paths = [
        root / "results" / "correlated-noise" / f"tau-{noise_tau:g}-results.csv"
    ]
if noise_tau in [0.0, 1.0]:
    golden_paths.append(root / "results" / "stopping-preview" / "all-results.csv")
if "clearance" in variants:
    # Snapshot keys include the path fixtures: these fine-grid paths differ.
    golden_paths = []
if "paced" in variants and noise_tau in [0.0, 1.0]:
    pinned_inputs = (
        root / "results" / "planning-clearance" / "fixtures" / "fine-padding"
    )
    expected_hashes = {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in pinned_inputs.glob("*.csv")
    }
    if provenance["inputs_by_variant_sha256"]["clearance"] == expected_hashes:
        golden_paths = [root / "results" / "planning-clearance" / "all-results.csv"]
for golden_path in golden_paths:
    if not golden_path.exists():
        continue
    golden = {
        (r["variant"], r["mode"], r["seed"], r["scenario"], r["planner"]): {
            k: v for k, v in r.items() if k != "cohort"
        }
        for r in csv.DictReader(golden_path.open())
        if "cohort" not in r
        or r["cohort"]
        in (
            {"white-validation"}
            if noise_tau == 0
            else (
                {"reference", "correlated-validation"}
                if "paced" in variants
                else {"correlated-validation"}
            )
        )
    }
    for r in rows:
        key = tuple(r[k] for k in ["variant", "mode", "seed", "scenario", "planner"])
        if key in golden:
            assert all(r[k] == v for k, v in golden[key].items() if v != ""), (
                "regression mismatch",
                key,
            )
for r in rows:
    if (
        not any(v in variants for v in ["guarded", "clearance", "paced"])
        and r["variant"] != reference_variant
        and r["mode"] == "none"
    ):
        ref = next(
            a
            for a in rows
            if a["variant"] == reference_variant
            and a["mode"] == "none"
            and all(a[k] == r[k] for k in ["seed", "scenario", "planner"])
        )
        assert all(r[k] == v for k, v in ref.items() if k != "variant"), (
            "no-loss regression",
            r,
        )

summary = []
for mode in ["none", "random", "blackout"]:
    for variant in variants:
        selected = [r for r in rows if r["mode"] == mode and r["variant"] == variant]
        assert len(selected) == 9 * len(seeds)
        summary.append(
            dict(
                mode=mode,
                variant=variant,
                trials=len(selected),
                held=sum(r["held"] == "true" for r in selected),
                collision=sum(r["collision"] == "true" for r in selected),
                timeout=sum(r["status"] == "timeout" for r in selected),
                out_of_bounds=sum(r["status"] == "out_of_bounds" for r in selected),
            )
        )
with (run / "summary.csv").open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(summary[0]), lineterminator="\n")
    w.writeheader()
    w.writerows(summary)
# A portable static plot; the runner needs only the Python standard library.
svg = [
    '<svg xmlns="http://www.w3.org/2000/svg" width="980" height="420" viewBox="0 0 980 420">',
    '<rect width="980" height="420" fill="white"/>',
    "<style>text {font-family: sans-serif; font-size: 14px}</style>",
]
colors = [("held", "#16a34a"), ("collision", "#dc2626"), ("timeout", "#94a3b8")]
for panel, mode in enumerate(["none", "random", "blackout"]):
    x0 = panel * 320 + 30
    svg.append(f'<text x="{x0}" y="30">{mode}: {9*len(seeds)} paired trials</text>')
    for col, variant in enumerate(variants):
        result = next(
            r for r in summary if r["mode"] == mode and r["variant"] == variant
        )
        x = x0 + col * 140
        y = 340
        for key, color in colors:
            height = 280 * result[key] / result["trials"]
            y -= height
            svg.append(
                f'<rect x="{x}" y="{y}" width="100" height="{height}" fill="{color}"/>'
            )
            if result[key]:
                svg.append(
                    f'<text x="{x+50}" y="{y+height/2+5}" text-anchor="middle">{result[key]}</text>'
                )
        svg.append(f'<text x="{x+50}" y="365" text-anchor="middle">{variant}</text>')
for col, (key, color) in enumerate(colors):
    svg.append(
        f'<rect x="{20+col*230}" y="390" width="15" height="15" fill="{color}"/>'
    )
    svg.append(f'<text x="{45+col*230}" y="403">{key}</text>')
svg.append("</svg>")
(run / "control-comparison.svg").write_text("\n".join(svg) + "\n")
lines = [
    "# 外部観測欠落時の停止と復帰",
    "",
    "| 欠落条件 | 制御 | 10秒保持 | 接触 | 時間切れ | 地図外 |",
    "|---|---|---:|---:|---:|---:|",
]
for r in summary:
    lines.append(
        f'| {r["mode"]} | {r["variant"]} | {r["held"]}/{r["trials"]} | {r["collision"]} | {r["timeout"]} | {r["out_of_bounds"]} |'
    )
lines += [
    "",
    "予定された5秒周期の外部姿勢観測を受信できない時点で停止待機を開始します。受信可否の将来情報や真の位置・障害物距離は制御に使用しません。減速度は最大1、速度比例ゲイン4で、位置や速度を強制変更していません。停止中は操舵ゼロです。",
    "新しい観測を受信後、受信から1.5秒以内・計測速度0.02以下・推定姿勢から前方0.6の線分が既知の障害物と地図境界から離れている条件を1秒連続で満たせば再開します。距離の余裕は半径0.5に0.1＋2√2σ＋位置ドリフト速度×観測経過時間を加算します。この余裕は簡易的な仮定であり、安全保証ではありません。受信時に経路探索位置をリセットし、再開から5秒は速度0.4以下に制限します。",
    "成功条件は従来と同じ、停止指令中の真の目標距離0.25未満・真の速度0.02以下を10秒連続です。半径0.5の車体が移動区間で障害物に接触したら即終了します。全条件の上限180秒も同じです。待機による時間切れを成功扱いしません。",
    "同じ3地図×3経路で、指定したシードの対比較を行いました。保存済みの初回実験はシード201〜210です。今回のシードは末尾に記録しています。欠落可否と正規化した観測ノイズの共通区間一致を確認しました。同じノイズ条件の保存済み結果があるシードも照合します。",
    "CSVから全停止復帰試行の時間間隔、有限値、観測経過時間、停止・復帰条件、ブレーキ応答と移動、接触・保持判定、RMSと最小距離を検証しました。外部観測欠落中も局所姿勢推定は利用可能で、終端の精密観測は理想化しています。実機・EKF・全センサー停止への検証ではありません。",
    "",
    (
        "guardedは走行中の停止軌道予測による追加ブレーキです。瞬間の推定姿勢・既知の地図・計測速度に基づき、0.2秒の通常指令の後に制限付きブレーキをかける軌道を確認します。到達の真値条件は同じです。"
        if "guarded" in variants
        else ""
    ),
    (
        "consistentは停止中の予定補正前25点と補正後10〜25点を比較し、位置差0.2以下・向きの差5度以下・補正後1.5秒以内の場合に平均姿勢で停止予測を確認します。既存の0.5秒連続確認も必要です。共通の偏りは検出できないため、安全保証ではありません。"
        if "consistent" in variants
        else ""
    ),
    (
        "clearanceは刻み幅0.125で計画半径0.5と0.99の経路を比較します。両方とも同じguarded制御です。全経路の線分距離を独立に監査しています。"
        if "clearance" in variants and "paced" not in variants
        else ""
    ),
    (
        "pacedは同じ広い経路で、推定姿勢から前方2の固定操舵の弧を調べ、余裕を満たす距離から速度を制限します。真の姿勢は制御に使いません。"
        if "paced" in variants
        else ""
    ),
    "![比較](control-comparison.svg)",
    "",
    "再実行: `python3 experiments/pose_observation_loss/run.py"
    + (
        " --anticipatory-speed-comparison FIXTURES"
        if "paced" in variants
        else (
            " --path-clearance-comparison FIXTURES"
            if "clearance" in variants
            else (
                " --restart-consistency-comparison"
                if "consistent" in variants
                else (
                    " --stopping-preview-comparison"
                    if "guarded" in variants
                    else " --recovery-comparison" if "averaged" in variants else ""
                )
            )
        )
    )
    + f" --noise-correlation-time {noise_tau}`（同じシードを指定）。",
    (
        "averagedは停止中の推定姿勢を最大25点、位置は平均・向きは円周平均にして再開判定に使用します。補正受信・欠落・移動・観測期限切れで窓を消去し、10点以上集まってから1秒の確認を始めます。距離余裕・有効時間・走行と保持の条件は変更しません。"
        if "averaged" in variants
        else ""
    ),
    "",
    "生データ: all-results.csv / control-events.csv / 各条件のtrajectories.csv。",
    f"局所姿勢ノイズの相関時間: {noise_tau}秒、隣接相関ρ={noise_rho:.6f}。σと制御条件は同じです。",
    f"実行情報: provenance.json。検証したシード: {sorted(seeds)}。",
]
(run / "report.md").write_text("\n".join(lines) + "\n")
for r in summary:
    print(r)
print("Verification passed; report:", run / "report.md")
