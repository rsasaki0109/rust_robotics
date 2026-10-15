"""Check paired trajectories, physical invariants, and recorded outcomes."""

from pathlib import Path
import csv
import json
import math
import sys

if not __debug__:
    raise RuntimeError("Run the verifier without Python -O; assertions are required")

run = Path(sys.argv[1]).resolve()
root = Path(__file__).resolve().parent
rows = list(csv.DictReader((run / "all-results.csv").open()))
seeds = {int(r["seed"]) for r in rows}
assert len(rows) == 54 * len(seeds)
goals = {"open_50x50": (48, 48), "dense_50x50": (48, 48), "maze_50x50": (35, 45)}
events = []
for item in json.loads((run / "manifest.json").read_text()):
    if item["variant"] != "aware":
        continue
    groups = {}
    pairs = {}
    for r in csv.DictReader((run / item["directory"] / "trajectories.csv").open()):
        groups.setdefault((r["scenario"], r["planner"]), []).append(r)
    for r in csv.DictReader(
        (run / f'baseline-{item["mode"]}-{item["seed"]}' / "trajectories.csv").open()
    ):
        pairs.setdefault((r["scenario"], r["planner"]), []).append(r)
    for key, track in groups.items():
        m = next(
            r
            for r in rows
            if r["variant"] == "aware"
            and r["mode"] == item["mode"]
            and int(r["seed"]) == item["seed"]
            and (r["scenario"], r["planner"]) == key
        )
        goal = goals[key[0]]
        hold = 0.0
        pauses = 0
        resumes = 0
        paused_time = 0.0
        last_observation = 0.0
        rms_integral = 0.0
        clearance = float(track[0]["step_min_obstacle_distance"])
        for i, r in enumerate(track[1:], 1):
            before = track[i - 1]
            t = float(r["time_s"])
            v = float(before["v"])
            speed = float(r["v"])
            assert abs(t - float(before["time_s"]) - 0.02) < 1e-8
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
            brake = paused or r["stop_latched"] == "true"
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
with (run / "control-events.csv").open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(events[0]), lineterminator="\n")
    w.writeheader()
    w.writerows(events)
golden_path = root / "results" / "all-results.csv"
if golden_path.exists():
    golden = {
        (r["variant"], r["mode"], r["seed"], r["scenario"], r["planner"]): r
        for r in csv.DictReader(golden_path.open())
    }
    for r in rows:
        key = tuple(r[k] for k in ["variant", "mode", "seed", "scenario", "planner"])
        if key in golden:
            assert all(r[k] == v for k, v in golden[key].items() if v != ""), (
                "regression mismatch",
                key,
            )
for r in rows:
    if r["variant"] == "aware" and r["mode"] == "none":
        ref = next(
            a
            for a in rows
            if a["variant"] == "baseline"
            and a["mode"] == "none"
            and all(a[k] == r[k] for k in ["seed", "scenario", "planner"])
        )
        assert all(r[k] == v for k, v in ref.items() if k != "variant"), (
            "no-loss regression",
            r,
        )

summary = []
for mode in ["none", "random", "blackout"]:
    for variant in ["baseline", "aware"]:
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
    for col, variant in enumerate(["baseline", "aware"]):
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
    "同じ3地図×3経路で、指定したシードの対比較を行いました。保存済みの実験はシード201〜210です。これらのシードは以前の実験にも使用したもので、未知条件の検証ではありません。欠落可否と正規化した観測ノイズの共通区間一致を確認しました。保存済み結果の該当シードも照合します。",
    "CSVから全停止復帰試行の時間間隔、有限値、観測経過時間、停止・復帰条件、ブレーキ応答と移動、接触・保持判定、RMSと最小距離を検証しました。外部観測欠落中も局所姿勢推定は利用可能で、終端の精密観測は理想化しています。実機・EKF・全センサー停止への検証ではありません。",
    "",
    "![比較](control-comparison.svg)",
    "",
    "再実行: `python3 experiments/pose_observation_loss/run.py`",
    "",
    "生データ: all-results.csv / control-events.csv / 各条件のtrajectories.csv。",
    f"実行情報: provenance.json。検証したシード: {sorted(seeds)}。",
]
(run / "report.md").write_text("\n".join(lines) + "\n")
for r in summary:
    print(r)
print("Verification passed; report:", run / "report.md")
