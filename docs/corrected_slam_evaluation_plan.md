# Corrected SLAM Evaluation Plan

Last updated: 2026-05-18

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

## 2026-05-17/18 Tuning Sweep — Findings and Course Correction

This section records the tuning revaluation sprint on branch
`dev/corrected-slam-eval`. The headline result is that the existing biased-odom
revaluation framework cannot reward ICP for correcting odometry bias, so further
single-knob ICP threshold tuning against it is not useful. The tuning track is
paused; the next step is to restructure the evaluation (see "Path A" below).

### Instrumentation gap (fixed)

`ros2_nodes/launch/run_navigation_smoke_test.sh` originally only captured
`/slam_diagnostics` at startup and after the mission finished. Both of those
windows have the robot stationary at the goal pose, so every gate histogram in
prior reports was dominated by `gate_reason=low_motion` with `icp_rejected`. The
mid-mission ICP behavior was simply not observed.

Fix shipped on `dev/corrected-slam-eval`:

- `5af6305` `Capture /slam_diagnostics across the mission window`
- `474578e` `Move mission-window diagnostics subscriber before further startup waits`

The smoke script now starts a background `ros2 topic echo
/slam_diagnostics` right after the startup topic check, keeps it running for
the whole mission, stops it after `/mission_status` capture, and dumps the
mid-mission stream under a `Mission-window slam diagnostics:` heading in the
log. The mission diagnostics file is also dumped on cleanup failure so a
timed-out run is still inspectable. Sample counts went from about 22 (stationary
only) to several hundred (mid-motion) per run.

Any conclusion drawn from gate histograms in reports older than `5af6305`
should be re-checked with this instrumentation; "low_motion dominant" was an
instrumentation artifact, not a property of the gate.

### Attenuated gate states are real and frequent

With mid-mission capture in place, `gate_reason` now shows three states that
were essentially invisible before:

- `attenuated_error`
- `attenuated_iterations`
- `attenuated_low_motion`

These come from `compute_icp_blend_decision()` in
`ros2_nodes/slam_node/src/main.rs` (around lines 395-502). After the hard
rejects (`translation_outlier`, `yaw_outlier`, `high_error`,
`slow_convergence`, `low_motion`), the function attenuates the blend via
`ramp_weight` of error, iterations, translation, yaw, and motion. When the
combined scale drops below 1.0 the diagnostics emit `attenuated_<smallest
weight>` and `status=icp_attenuated`; otherwise the run is `accepted` and
`status=icp_ok`. blend_alpha is then `base_alpha * scale`.

This means ICP is contributing to the corrected frame in attenuated form much
more often than the older "almost always low_motion" picture suggested.

### Loose ICP reject-error sweep made SLAM strictly worse

`837cbb9` added two loose profiles to the tuning sweep in
`ros2_nodes/launch/run_navigation_revaluation_matrix.sh`:

- `loose_error` (`SLAM_ICP_REJECT_ERROR=0.018`)
- `loose_error_low_alpha` (`SLAM_ICP_BLEND_ALPHA=0.10 SLAM_ICP_REJECT_ERROR=0.018`)

(plus an unfinished `very_loose_error` at `0.025`).

The motivation was that the default reject threshold `0.011` sits below the
observed LIDAR noise floor near `0.013`, so almost every ICP match was being
rejected on residual alone. Loosening to `0.018` did open the gate: the
attenuated states started firing during motion and corrections actually blended
in.

Against the biased odom profile `odom_xy_scale_3pct`, however, `improvement_xy`
moved the wrong way:

- `default`              about `-0.005`
- `loose_error`          about `-0.021`
- `loose_error_low_alpha` about `-0.013`
- `loose_error` on `long_two_legs` about `-0.023`

That is, letting more ICP through made corrected SLAM 3-4x worse on the same
biased odom run. Lowering blend alpha helped (smaller correction per scan),
but the direction of the correction was still wrong.

### Structural root cause: biased map cannot bias-correct

In the current evaluation, the SLAM input odom is biased
(`SLAM_INPUT_ODOM_XY_SCALE=1.03`) and the SLAM map is built from that same
biased odom. ICP fits each new scan against a map already deformed by the bias,
so a "better" match reinforces the bias instead of correcting it. The gate is
not at fault; the experiment cannot reward bias correction in principle. This
also explains why earlier sweeps could not produce a profile that was both
gate-active and `slam_better_xy=True` on biased odom.

A separate but real bug in the tuning matrix at the time of this sprint:
`strict_error` was set to `SLAM_ICP_REJECT_ERROR=0.011`, which is the same as
`DEFAULT_ICP_REJECT_ERROR` (see line 33 of
`ros2_nodes/slam_node/src/main.rs`). `strict_error` and `strict_low_alpha`
were therefore functionally duplicates of `default` and `low_alpha`. This is
worth fixing the next time the tuning matrix is touched, but it does not
change the structural conclusion above.

### Decision: pause tuning, take Path A next

The tuning sweep is paused until the evaluation framework is changed so that
ICP can in principle improve corrected SLAM under odom bias.

Path A — restructure the evaluation so that the SLAM map is built from
**unbiased** odom and the bias is injected later, only on the input odom path
used for the corrected-frame estimate. This makes the map an honest reference
and gives ICP something to correct against.

Sketch:

- Keep `SLAM_INPUT_ODOM_XY_SCALE` / `SLAM_INPUT_ODOM_YAW_SCALE` style knobs,
  but route them only through the corrected-frame input, not through the
  mapping step.
- Either (a) build the map in a pre-pass with unbiased odom and freeze it for
  the corrected-frame run, or (b) split the SLAM node so the mapping update
  and the ICP correction update consume two different odom sources, with bias
  applied only to the ICP-correction input.
- Reuse the existing biased odom profile names (`odom_xy_scale_3pct`,
  `odom_yaw_drift_3deg_per_m`, etc.) so the matrix harness does not need to
  change shape.
- Re-run the tuning sweep (`default`, `low_alpha`, plus a corrected
  `strict_error` / `loose_error` / `very_loose_error`) against
  `odom_xy_scale_3pct` and `odom_yaw_drift_3deg_per_m` on the existing
  scenarios, with the mid-mission diagnostics already in place.

Alternatives considered:

- Path B: inject bias as a `/scan` or entity-pose offset instead of odom
  scale. This also gives ICP something to correct, but requires building a
  new bias injection path and matching ground truth.
- Path C: drop bias evaluation from the corrected-SLAM tuning loop and treat
  ICP gating as purely a noise-robustness mechanism on clean odom.
- Path D: stop tuning and document findings (this section). Done.

Path D is done; Path A is the next active task. Path B is a fallback if Path A
runs into a structural blocker in the SLAM node. Path C is intentionally not
pursued because the biased-odom matrix is the main lever we have for
demonstrating value of corrected SLAM.

### Followups not addressed yet

- `very_loose_error` (`SLAM_ICP_REJECT_ERROR=0.025`) matrix rows 60-62 were
  never executed; the workflow run hit a step timeout at 6/9 results. Re-run
  if/when revaluation is re-enabled, but only after Path A is in place,
  otherwise the result is just "more loose, more wrong".
- The `strict_error` / `strict_low_alpha` duplicate-of-default bug above.
- The biased-odom revaluation still has no positive-control scenario where
  ICP is expected to clearly improve raw odom. `rich_geometry_turns` from the
  Scenario Roadmap above is the obvious candidate to add alongside Path A.
