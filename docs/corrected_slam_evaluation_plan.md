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

### Why loose_error made things worse — ICP is noise-dominated during in-place rotation

Two earlier drafts of this section were wrong. The first claimed the SLAM
map is built from biased odom and ICP therefore reinforces the bias; that is
false because the SLAM node uses **scan-to-scan** ICP (`icp_matching` at
`ros2_nodes/slam_node/src/main.rs:830` takes `previous_scan` and
`current_scan`; the persistent `st.map` is for publishing / navigation only,
not an ICP input). The second draft suggested a sign bug in
`icp_motion_delta`; investigation rules that out as well.

The investigation:

- `icp_matching` (`crates/rust_robotics_slam/src/icp_matching.rs:60`)
  takes only point matrices; there is no pose-prior argument. There is no
  biased initial guess. Hypothesis "biased ICP init" is rejected.
- `svd_motion_estimation` returns `(R, t)` satisfying
  `previous ≈ R * current + t`, i.e. the transform that maps `current_scan`
  back into the `previous_scan` frame. Equivalently, this transform is the
  robot motion expressed in the previous body frame:
  `T_prev←curr = robot_motion_in_prev_body_frame`. So
  `result.translation` directly **is** the robot motion delta, with the
  intended sign. `icp_motion_delta` is correct.
- The `icp_matching` library test (`icp_matching.rs:407`) checks
  `(result.translation[0] + translation.x).abs() < 0.1`, which can be read
  as a sign flip in isolation, but in that test `apply_2d_transformation`
  shifts `previous_points` by `translation`, so `T_prev←curr = -translation`
  exactly because the synthetic robot motion is `-translation`. The
  assertion is consistent with the body-frame motion interpretation.
- Mid-mission logs from the loose-profile run confirm `icp_dx`, `icp_dy`,
  and `icp_dyaw` are in the same body frame and the same sign convention
  as `odom_dx`, `odom_dy`, `odom_dyaw`. Example, from
  `navigation_revaluation_20260518T003221Z_logs/55_odom_xy_scale_3pct_loose_error_three_hops.log`:
  `odom_dx=0.013, odom_dy=-0.001, odom_dyaw=-0.126`,
  `icp_dx=0.011,  icp_dy= 0.014,  icp_dyaw=-0.137`.

So why did loose_error make corrected SLAM 3-4x worse? Looking at those
same log lines, `raw_yaw` is sweeping from `-2.181` to `-3.137` over ~8 scans
while `raw_x` / `raw_y` are nearly constant. The robot is doing an in-place
or near-in-place rotation. In that regime:

- True forward motion per scan is roughly zero.
- Biased odom (`SLAM_INPUT_ODOM_XY_SCALE=1.03`) and TurtleBot3 wheel-odom
  noise both fabricate a small nonzero `odom_dx`.
- ICP scan-to-scan also fabricates a nonzero translation because the LIDAR
  noise floor (~1.3 cm mean residual) is comparable to the true per-scan
  translation, so `icp_dx` / `icp_dy` are dominated by noise rather than
  signal. The `icp_dy=0.014` vs `odom_dy=-0.001` mismatch in the example
  above is exactly this noise: even the sign of the small ICP translation
  is essentially random during in-place rotation.

The default reject threshold (`0.011`) was below the LIDAR noise floor and
rejected almost every match — including the noise corrections — which is
why default sat near `improvement_xy ≈ -0.005` (a small safety bias rather
than a real correction). Loosening to `0.018` admitted those noise
corrections through the `attenuated_error` path with `blend_alpha ≈ 0.08`
to `0.15`. Even at that small alpha, integrating a stream of noisy
corrections over a multi-hop mission pulled the corrected estimate further
from ground truth than just trusting biased odom.

So the loose-profile regression is **not** a bug in the node and **not**
fixed by another threshold tweak. It is structural to scan-to-scan ICP on
this robot+scenario combination:

- short missions with significant in-place rotation,
- low forward speed,
- a LIDAR with residual noise comparable to per-scan true motion.

### Separate bug: strict_error == default

In `ros2_nodes/launch/run_navigation_revaluation_matrix.sh`, the tuning
profile `strict_error` was set to `SLAM_ICP_REJECT_ERROR=0.011`, which is the
same as `DEFAULT_ICP_REJECT_ERROR` at line 33 of
`ros2_nodes/slam_node/src/main.rs`. `strict_error` and `strict_low_alpha` are
therefore functional duplicates of `default` and `low_alpha`. Fix the next
time the tuning matrix is touched (e.g., drop to `0.008`).

### Decision: pause tuning, fix the scenario set before any more sweeps

Single-knob ICP gate tuning on the current scenarios cannot show ICP doing
useful work — there is barely a forward-motion regime in which ICP signal
exceeds noise. The next active task is scenario-side, not gate-side:

- Add a positive-control scenario with sustained forward motion and rich
  geometry (`rich_geometry_turns` from the Scenario Roadmap is the obvious
  candidate: 3-5 m, asymmetric obstacles, several discrete 90 degree
  turns). Validate it on default and unbiased odom first; only then revisit
  loose-profile sweeps.
- Optionally add `long_loop_return` (6-10 m, returns near start) to expose
  accumulated drift that ICP should be able to correct.
- Until such a scenario exists, do not draw more conclusions from
  short_default / three_hops / long_two_legs about ICP gate tuning. Use
  them only as smoke/safety scenarios.

Paths considered earlier:

- Path A (restructure evaluation so the SLAM map is built from unbiased
  odom): **not justified** for the current node; the map is not an ICP
  input.
- Path B (inject bias as `/scan` or entity-pose offset): still a fallback,
  but not the next move.
- Path C (treat ICP gating purely as noise-robustness on clean odom): in
  effect this is what the current default already is. The matrix should
  acknowledge that.
- Path D (document findings, pause tuning): done in this section.
- Path E (new, the actual next step): scenario-side fix described above.

### First rich_geometry_turns attempt — scenario design works, path was too tight

The first shape committed for `rich_geometry_turns` was
`0.55,0.0;1.10,0.0;1.10,0.55;0.55,0.55` (4 waypoints, 2.2 m, two 90-degree
turns). The mission did **not** complete on the baseline odom + default
profile validation run (Actions run `26015254947`,
`navigation_revaluation_20260518T053249Z`). The robot reached waypoint 1
`(0.55, 0)` and then republished waypoint 2 `(1.10, 0)` until the 240 s
timeout: world `(-0.9, -0.5)` requires squeezing between the
turtlebot3_world cylinder columns at `x=-1.1` and `x=0`, where the path
clearance to the `(-1.1, 0)` cylinder is only about 0.5 m. After DWA
costmap inflation the navigable corridor is roughly 0.245 m on each side,
which the burger could not fit through reliably. The existing successful
scenarios (`short_default`, `three_hops`, `long_two_legs`) all stay at
relative `x <= 0.55`, i.e. world `x <= -1.45`, and never cross any
cylinder column — that is why they complete.

However the 1314 mid-mission diagnostic samples captured during that
failed run are an unexpectedly clean positive control for the scenario
design itself:

- `icp_initial_error_mean = 0.110` (large per-scan nearest-neighbor
  distance before ICP transform; consistent with the robot actually
  rotating and translating between scans)
- `icp_error_mean = 0.014` after ICP convergence
- `icp_relative_error_reduction = 0.873` (about 87 percent)
- `icp_inlier_ratio_5cm = 0.991`
- `icp_iterations = 28` (median; converges with effort, well below the
  100-iteration cap)
- All 1314 samples gated `high_error` because `0.014 > DEFAULT_ICP_REJECT_ERROR (0.011)`

This is the regime the previous scenarios never reached: ICP is finding
a meaningful correction signal, but the default reject threshold is just
below the converged residual floor and rejects every match. With
`loose_error (0.018)` the same samples would mostly be admitted through
the attenuated path with nonzero blend alpha. So once a navigable
rich_geometry_turns variant exists, the next tuning question becomes:
"on this scenario, with this much actual ICP signal, does `loose_error`
help, hurt, or stay neutral?"

Reshaped path (current): `0.55,0.0;0.55,0.5;0.0,0.5` — three waypoints,
two 90-degree turns, ~1.6 m total, all inside the working-scenario
corridor at relative `x <= 0.55`. Pending fresh validation.

### Baseline odom × loose_error on all four scenarios — XY worse, YAW better

After `rich_geometry_turns` was reshaped to the all-diagonal three-hops
pattern `(0.55,0.05); (0.20,0.55); (0,0)` and validated as completing
(run `26017689809`: `mission_completed=true`, 134 mid-mission scans,
49 high_error + 85 low_motion), a focused 8-run slice was triggered to
compare `loose_error` and `loose_error_low_alpha` across all four
scenarios on the unbiased baseline odom (Actions run `26018186549`,
`navigation_revaluation_20260518T065743Z`). The step hit the 75-minute
timeout after the loose_error half completed, so the four loose_error
runs and the rich_geometry_turns log are usable; loose_error_low_alpha
rows were not produced.

Results for **baseline (unbiased) odom × loose_error**:

| scenario              | improvement_xy | improvement_yaw | slam_better_yaw |
| --------------------- | --------------:| ---------------:| ---------------:|
| short_default         | -0.023         | +0.005          | True            |
| three_hops            | -0.011         | +0.043          | True            |
| long_two_legs         | -0.045         | -0.001          | False           |
| rich_geometry_turns   | -0.022         | +0.027          | True            |

(rich_geometry_turns row taken from the mid-mission log; the CSV row
was not written because the step timed out during summary generation.)

The pattern is consistent on clean odom: scan-to-scan ICP corrections
**hurt XY by 1–4 cm but help or stay neutral on yaw**, often making
`slam_better_yaw=True`. The earlier biased-odom regression
(`loose_error` made XY 3-4x worse) is therefore not a biased-odom
specific failure mode — it is the same XY noise injection that already
happens on clean odom, amplified by the 3 percent scale bias.

Practical reading:

- The default `SLAM_ICP_REJECT_ERROR=0.011` is well tuned for XY
  accuracy on this robot+world. Loosening it makes XY worse on every
  scenario tested so far, including the new positive-control
  `rich_geometry_turns`.
- Scan-to-scan ICP on a TurtleBot3 burger is more reliable as a
  **yaw** correction than as a **translation** correction. If the
  goal is yaw accuracy, the gate can be loosened; if the goal is XY
  accuracy, default is the right setting.
- The simplest next improvement is to **split the gate into XY and
  yaw branches** so the yaw blend can be admitted while XY is still
  rejected. That avoids the current "all-or-nothing" XY/yaw coupling.

### Followups not addressed yet

- Re-run the missed loose_error_low_alpha slice (matrix indices
  20-23, 4 runs) to see if reducing blend alpha eliminates the XY
  regression while keeping the yaw improvement. With ~19 min per run
  including the rich_geometry_turns step, the next slice should fit
  within the 75-minute step timeout.
- `very_loose_error` (`SLAM_ICP_REJECT_ERROR=0.025`) rows are not a
  priority; if loose_error already hurts XY at 0.018, very_loose will
  hurt more.
- Investigate the yaw-only blend split above. This is a code change
  in `compute_icp_blend_decision()` and `blend_motion_delta()` in
  `ros2_nodes/slam_node/src/main.rs`, not a tuning sweep.
- Biased odom × tuning sweep is still pending. Run it only after the
  yaw-only blend question is resolved, otherwise the result is
  predictable from the unbiased data above.
- The revaluation matrix script does not write the CSV row until the
  whole step succeeds; on step timeout the last completed run's row
  is lost even though the log is uploaded. Consider flushing the CSV
  per-run so partial sweeps are self-describing.
