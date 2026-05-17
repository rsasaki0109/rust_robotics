# Corrected SLAM Evaluation Plan

Last updated: 2026-05-17

## Decision Summary

Corrected SLAM remains experimental. The current pipeline has working ROS2 wiring,
TF, diagnostics, Gazebo ground-truth comparison, synthetic ICP acceptance, and a
multi-scenario revaluation harness. It has not yet shown stable superiority over
raw / EKF odometry, so it should not become the default navigation frame.

The next decision point is not another single ICP threshold tweak. The evaluation
must separate:

- cases where ICP should be accepted and improve odometry
- cases where ICP should be rejected or attenuated
- cases where raw odometry is already too accurate for short missions to reveal
  a meaningful improvement

## Test Tiers

### Smoke

Purpose: verify launch, topics, TF, mission completion, diagnostics availability,
and absence of obvious runtime breakage.

Runs: every PR through `ros2-smoke.yml`.

Do not require corrected SLAM to beat raw odometry here yet. Accuracy thresholds
in smoke should be limited to safety guards, such as no NaN, no huge correction
jump, and no catastrophic ground-truth error.

Current corrected-frame smoke safety guards are intentionally loose and
configurable:

- `SMOKE_MAX_SLAM_XY_ERROR` (default `0.50`)
- `SMOKE_MAX_SLAM_YAW_ERROR` (default `0.524`, about 30 degrees)
- `SMOKE_MAX_SLAM_XY_MAX` (default `0.75`)
- `SMOKE_MAX_APPLIED_TRANSLATION_DELTA` (default `0.20`, XY magnitude)
- `SMOKE_MAX_APPLIED_YAW_DELTA` (default `0.35`)

### Acceptance

Purpose: positive-control ICP behavior.

Runs: every PR.

Current command:

```bash
./ros2_nodes/launch/run_slam_icp_acceptance_test.sh
```

Expected behavior: clean synthetic scan matching reaches `status=icp_ok`,
`gate_reason=accepted`, and `blend_applied=true`.

### Revaluation

Purpose: corrected-frame accuracy and gate-policy decisions.

Runs: manually, nightly, or before release. Store artifacts.

Current command:

```bash
./ros2_nodes/launch/run_navigation_revaluation_matrix.sh
python3 scripts/summarize_slam_revaluation.py reports/slam_revaluation/*.csv
```

The matrix now writes CSV, JSONL, a Markdown summary, and per-run logs under
`reports/slam_revaluation/`.

GitHub Actions route: run the `ROS2 Smoke` workflow manually with
`run_revaluation=true`. The workflow keeps normal PR smoke behavior unchanged,
then uploads `reports/slam_revaluation/` as an artifact for the manual run.

## Scenario Roadmap

Existing scenarios:

- `short_default`
- `three_hops`
- `long_two_legs`

Existing odom mechanism profiles:

- `raw_realistic`
- `odom_xy_scale_1pct`
- `odom_xy_scale_3pct`
- `odom_yaw_drift_1deg_per_m`
- `odom_yaw_drift_3deg_per_m`
- `odom_turn_yaw_scale_3pct`

Add scenarios in this order:

1. `rich_geometry_turns`: 3-5 m, asymmetric obstacles, several 90 degree turns.
   This should be a positive-control Gazebo scenario for accepted ICP.
2. `long_loop_return`: 6-10 m, returns near the start. This reveals accumulated
   drift and map-frame consistency problems.
3. `corridor_degenerate`: parallel walls or weak longitudinal constraints. This
   should reject or attenuate ambiguous ICP.
4. `stop_and_go_low_motion`: low speed, stops, and restarts. This validates
   low-motion rejection and static drift behavior.
5. `wheel_slip_segment`: localized odom degradation. This is more realistic than
   constant scale or yaw drift.

## ICP Gate Metrics

The current gate uses ICP mean error, iteration count, motion size, and correction
size. Keep that policy, but treat gate decisions as a classification problem.

Metrics now emitted in `/slam_diagnostics` and revaluation artifacts:

- `icp_inlier_ratio_5cm`
- `icp_error_median`
- `icp_error_p90`
- `icp_relative_error_reduction`

Recommended next metrics:

- trimmed mean residual
- odom-prior innovation in XY and yaw
- geometry conditioning or observability score

Recommended gate classes:

- `accepted`
- `attenuated`
- `rejected_bad_match`
- `rejected_degenerate`
- `rejected_low_motion`
- `rejected_large_innovation`

Low-motion rejection is appropriate for scan-to-scan ICP. During near-zero motion,
sensor noise and nearest-neighbor instability can be larger than true motion, so
integrating corrections can create static drift. Revisit this only after adding
scan-to-map or local-submap matching.

## Regression Promotion

Promote accuracy checks gradually:

1. Smoke keeps plumbing checks only.
2. Add safety guards: bounded max XY/yaw error and bounded per-frame correction.
3. Add loose non-regression: corrected RMSE may not exceed raw RMSE by more than
   a small epsilon.
4. Require improvement only for positive-control scenarios with injected odom
   drift and rich geometry.
5. Consider default mode only after aggregate matrix results show non-regression
   in baseline scenarios, clear improvement in drift scenarios, and safe rejects
   in negative-control scenarios.

## Default-Mode Gate

Corrected SLAM can become a default candidate only when all of these are true:

- baseline realistic odom has completion rate at least equal to raw odom
- baseline corrected RMSE is no worse than raw RMSE plus a small epsilon
- catastrophic correction count is zero
- biased scale / yaw / slip scenarios show stable median improvement
- negative-control scenarios do not produce false accepted corrections
- gate metrics and reasons are preserved in machine-readable artifacts

Until then, keep `ENABLE_SLAM_CORRECTED_FRAME=true` as an experimental opt-in.
