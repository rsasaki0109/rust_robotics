# Experiments

## Problem

Preset: `crossover-windows`

Compare multiple runtime-aggregation strategies on crossover-focused MovingAI bucket windows before promoting any evaluation method into the stable workflow.

## Shared Input

- `arena2` buckets: `86, 87, 88, 89, 90`
- `8room_000` buckets: `80, 85, 90`
- `random512-10-0` buckets: `130, 132, 135`
- `Berlin_0_512` buckets: `120, 140, 160, 186`
- per-scenario timing iterations: `3`
- near-tie reporting band: `|A*/JPS ratio - 1.0| < 0.020`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 1896.14 | 0.100 | 0.533 | 0.533 | 0.199 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 5514.01 | 0.300 | 0.733 | 0.667 | 0.185 | 31 | 0 | 1 | yes |
| `percentile-bucket` | functional-percentile | 9686.57 | 0.500 | 0.800 | 0.733 | 0.167 | 43 | 2 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 16356.51 | 0.907 | 0.933 | 0.933 | 0.036 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 18582.57 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw faster side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `8room_000/80` | outcome=`JPS`, ratio=`1.120`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.962`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.962`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.944`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.910`, coverage=`10/10`, escalated=`no` |
| `8room_000/85` | outcome=`A*`, ratio=`0.936`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.940`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.941`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.929`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.941`, coverage=`10/10`, escalated=`no` |
| `8room_000/90` | outcome=`near-tie`, raw=`JPS`, ratio=`1.018`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`A*`, ratio=`0.994`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.979`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.957`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.959`, coverage=`10/10`, escalated=`no` |
| `Berlin_0_512/120` | outcome=`A*`, ratio=`0.932`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.962`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.443`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.173`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.162`, coverage=`10/10`, escalated=`no` |
| `Berlin_0_512/140` | outcome=`A*`, ratio=`0.905`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.970`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.972`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.613`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.679`, coverage=`10/10`, escalated=`no` |
| `Berlin_0_512/160` | outcome=`JPS`, ratio=`1.726`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.153`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`JPS`, ratio=`1.009`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.213`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.224`, coverage=`10/10`, escalated=`no` |
| `Berlin_0_512/186` | outcome=`A*`, ratio=`0.956`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.632`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.329`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.292`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.282`, coverage=`10/10`, escalated=`no` |
| `arena2/86` | outcome=`A*`, ratio=`0.721`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.714`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.705`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.766`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.756`, coverage=`10/10`, escalated=`no` |
| `arena2/87` | outcome=`A*`, ratio=`0.701`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.039`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.699`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.701`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.686`, coverage=`10/10`, escalated=`no` |
| `arena2/88` | outcome=`A*`, ratio=`0.669`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.759`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.748`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.715`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.718`, coverage=`10/10`, escalated=`no` |
| `arena2/89` | outcome=`JPS`, ratio=`1.110`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.109`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.066`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.734`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.747`, coverage=`10/10`, escalated=`no` |
| `arena2/90` | outcome=`near-tie`, raw=`JPS`, ratio=`1.016`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.703`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.690`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.027`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.978`, coverage=`10/10`, escalated=`no` |
| `random512-10-0/130` | outcome=`JPS`, ratio=`1.577`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.673`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.614`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.703`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.664`, coverage=`10/10`, escalated=`no` |
| `random512-10-0/132` | outcome=`JPS`, ratio=`1.858`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.819`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.873`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.866`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.848`, coverage=`10/10`, escalated=`no` |
| `random512-10-0/135` | outcome=`A*`, ratio=`0.661`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.700`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.346`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.693`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.944`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Stable core: `RuntimeAggregationVariant`, `RuntimeSamplingPlan`, `RuntimeExperimentCase`, `RuntimeObservation`, `RuntimeVariantReport`, and `run_variant_suite`.
- Experimental region: concrete selection strategies under `crates/rust_robotics_planning/src/experiments/moving_ai_runtime/`.

## Related Planning Preset

Preset: `grid-threshold-planners`

- Shared contract: `PathPlanner::plan(Point2D, Point2D) -> Result<Path2D, RoboticsError>`
- Compared variants: `AStarPlanner`, `FringeSearchPlanner`, `IDAStarPlanner`
- Current summary: quality tied on the measured local fixtures; median-timed mean runtime was `A* 161.71 us`, `Fringe Search 230.13 us`, `IDA* 313.10 us`, mean expanded nodes were `19.33 / 20.00 / 18.67`, and mean peak live search state was `70.67 / 34.67 / 10.67`
- Larger-window sweep summary: on the current `arena2` bucket-`5` windows (`margin = 8 / 16 / 24`), exact `A*`, `Fringe Search`, and all `9/9` bounded `IDA*` runs matched after the threshold tolerance fix
- Cheap-budget floor summary: on the same bucket-`5` slice, `look2-iter1` still stops at budgets `18 / 20 / 21`, while `22` is the current minimum exact budget
- Mid-slice floor summary: on the `arena2` bucket-`11` windows (`margin = 8 / 16 / 24`), `look2-iter1` stops at `42` and first reaches exact at `43`
- Harder-slice summary: on the `arena2` bucket-`12` windows (`margin = 8 / 16 / 24`), the same `look2-iter1` policy still stops at `131`, while `132` is the current minimum exact budget
- One-iteration reachability summary: on `arena2` buckets `10` and `15`, `look2-iter1-exp10000` still stops on `max_iterations`
- Threshold-round summary: on `arena2` bucket `10`, exactness returns at `iter32` with `iter24` still failing; on bucket `15`, `iter96` still fails on `max_iterations` and `iter128` first hits `max_expanded_nodes`
- Bucket-`15` expansion-band summary: `look2-iter128` still has no exact run through `exp800000`, and `look2-iter256` still has no exact run through `exp1200000`
- Bucket-`15` high-cost recovery summary: representative `window16` still fails through `iter256-exp3000000`, and on the full slice `look2-iter512-exp1750775` stays mixed (`window08` exact, `window16/24` expansion-stop) while `look2-iter512-exp1750776` reaches exact
- Bucket-`15` diagnostics summary: the new failure-preserving `IDA*` reports show the hard slice is re-expansion-dominated (about `1.75M` total expansions vs roughly `1.0k` unique states near the floor); at fixed `look2-iter512-exp1750776`, the full-slice iteration floor is `275`, with `274` still failing on `max_iterations` and leaving a mean next-threshold gap of `0.058875`
- Detailed report: [`docs/experiments_planning_grid_threshold.md`](experiments_planning_grid_threshold.md)

## Related Control Preset

Preset: `control-drone-trajectory-variants`

- Shared contract: sampled `DesiredState` sequences fed into the same quadrotor PD tracker
- Compared variants: `quintic-uniform`, `quintic-distance-scaled`, `minimum-snap-distance-scaled`
- Current summary: `quintic-distance-scaled` is the practical default for lower peak speed and acceleration on mixed-length loops, while `minimum-snap-distance-scaled` remains experimental on the current stop-go preset
- Latest local mean metrics: generation `523.55 / 517.66 / 2387.46 us`, RMSE `3.5357 / 3.6117 / 3.9138 m`, mean peak speed `4.7557 / 3.8577 / 4.5006 m/s`, mean peak acceleration `3.8625 / 3.4018 / 4.4268 m/s^2`
- Detailed report: [`docs/experiments_control_drone_trajectory.md`](experiments_control_drone_trajectory.md)

Preset: `control-drone-trajectory-pass-through`

- Shared contract: the same sampled `DesiredState` surface and quadrotor PD tracker, but with non-zero pass-through waypoint velocities synthesized inside the experiment
- Compared variants: `quintic-uniform`, `quintic-distance-scaled`, `minimum-snap-distance-scaled`
- Current summary: the pass-through preset still favors `quintic-distance-scaled`; the current velocity-only boundary policy was not enough to make minimum-snap preferable
- Latest local mean metrics: generation `591.93 / 600.82 / 2598.43 us`, RMSE `10.1726 / 9.8653 / 10.5532 m`, mean jerk RMS `4.1334 / 4.0965 / 4.9339`, mean snap RMS `10.7007 / 10.6551 / 16.8112`
- Detailed report: [`docs/experiments_control_drone_trajectory_pass_through.md`](experiments_control_drone_trajectory_pass_through.md)

Preset: `control-drone-trajectory-pass-through-accel`

- Shared contract: the same sampled `DesiredState` surface and quadrotor PD tracker, with pass-through waypoint velocities plus finite-difference waypoint accelerations synthesized inside the experiment
- Compared variants: `quintic-uniform`, `quintic-distance-scaled`, `minimum-snap-distance-scaled`
- Current summary: the richer acceleration boundary still favors `quintic-distance-scaled`; minimum-snap did not become preferable on the current controller/preset pair
- Latest local mean metrics: generation `596.10 / 600.26 / 2672.31 us`, RMSE `10.0424 / 9.7819 / 10.3241 m`, mean jerk RMS `3.8517 / 3.8070 / 4.5766`, mean snap RMS `10.0410 / 9.9684 / 15.4774`
- Detailed report: [`docs/experiments_control_drone_trajectory_pass_through_accel.md`](experiments_control_drone_trajectory_pass_through_accel.md)

Preset: `control-drone-trajectory-pass-through-accel-jerk`

- Shared contract: the same sampled `DesiredState` surface and quadrotor PD tracker, with pass-through waypoint velocities, finite-difference waypoint accelerations, and finite-difference waypoint jerk synthesized inside the experiment
- Compared variants: `quintic-uniform`, `quintic-distance-scaled`, `minimum-snap-distance-scaled`
- Current summary: even the jerk-enriched boundary still favors `quintic-distance-scaled`; the added jerk term only changes the minimum-snap branch and did not make it preferable on the current controller/preset pair
- Latest local mean metrics: generation `542.63 / 542.30 / 2211.26 us`, RMSE `10.0424 / 9.7819 / 10.3186 m`, mean jerk RMS `3.8517 / 3.8070 / 4.5775`, mean snap RMS `10.0410 / 9.9684 / 15.4840`
- Detailed report: [`docs/experiments_control_drone_trajectory_pass_through_accel_jerk.md`](experiments_control_drone_trajectory_pass_through_accel_jerk.md)

Preset: `control-drone-trajectory-coupled-continuity`

- Shared contract: the same sampled `DesiredState` surface and quadrotor PD tracker, with globally solved closed-loop waypoint velocities plus coupled waypoint acceleration continuity and averaged segment jerk synthesized inside the experiment
- Compared variants: `quintic-uniform`, `quintic-distance-scaled`, `minimum-snap-distance-scaled`
- Current summary: coupled continuity dramatically smooths the generated trajectories, but the current PD tracker follows them worse on the harder loop families, so trajectory-side smoothing alone still does not settle the generator decision
- Latest local mean metrics: generation `486.49 / 489.79 / 1905.09 us`, RMSE `15.3070 / 15.4363 / 15.3624 m`, mean jerk RMS `0.9460 / 0.9441 / 0.9784`, mean snap RMS `0.0000 / 0.0000 / 1.8045`
- Detailed report: [`docs/experiments_control_drone_trajectory_coupled_continuity.md`](experiments_control_drone_trajectory_coupled_continuity.md)

Preset: `control-drone-trajectory-coupled-continuity-controllers`

- Shared contract: the same coupled-continuity desired-state trajectories, but compared under package-local controller variants instead of only the baseline PD tracker
- Compared controllers: `baseline-pd`, `attitude-rate-damped-pd`, `attitude-rate-damped-lateral-pd`
- Current summary: pure attitude-rate damping is still mixed, but adding bounded lateral feedback on top of it sharply improves the coupled-continuity runs and recovers the figure-eight regressions without giving back the low-effort behavior
- Latest local mean metrics: RMSE `15.3686 / 15.1201 / 1.7724 m`, mean max error `26.2037 / 26.4761 / 3.0775 m`, mean max thrust `2.0871 / 2.0538 / 2.0592 N`, mean torque norm `3.4787 / 0.1408 / 0.1676 N m`
- Detailed report: [`docs/experiments_control_drone_trajectory_coupled_continuity_controllers.md`](experiments_control_drone_trajectory_coupled_continuity_controllers.md)

Preset: `control-drone-trajectory-noncoupled-controllers`

- Shared contract: the existing non-coupled desired-state presets, compared under the same package-local controller variants used for the coupled-continuity rerun
- Compared controllers: `baseline-pd`, `attitude-rate-damped-pd`, `attitude-rate-damped-lateral-pd`
- Current summary: the bounded lateral-feedback controller generalizes cleanly beyond the coupled preset and beats both the baseline and pure damping controllers on all four tested non-coupled presets
- Latest local mean RMSE by preset: `stop-go 3.6871 / 3.8431 / 0.9226 m`, `pass-through 10.1970 / 9.2314 / 0.9434 m`, `pass-through-accel 10.0495 / 9.2911 / 0.9313 m`, `pass-through-accel-jerk 10.0476 / 9.2909 / 0.9317 m`
- Detailed report: [`docs/experiments_control_drone_trajectory_noncoupled_controllers.md`](experiments_control_drone_trajectory_noncoupled_controllers.md)

Preset: `control-drone-trajectory-controller-generator-pairings`

- Shared contract: the same package-local trajectory generators re-evaluated under explicit controller + generator pairings
- Compared pairings: `baseline-pd` and `attitude-rate-damped-lateral-pd` crossed with `quintic-uniform`, `quintic-distance-scaled`, and `minimum-snap-distance-scaled`
- Current summary: the stronger controller changes some local winner labels, but it still does not overturn the practical generator choice: `quintic-distance-scaled` stays best across all four non-coupled presets, while coupled-continuity only gives `minimum-snap` a tiny RMSE/max-error edge at much higher generation cost
- Latest local bounded-feedback mean RMSE by preset: `stop-go 0.9144 / 0.7631 / 1.0903 m`, `pass-through 0.9194 / 0.9188 / 0.9919 m`, `pass-through-accel 0.9267 / 0.9199 / 0.9473 m`, `pass-through-accel-jerk 0.9267 / 0.9199 / 0.9486 m`, `coupled-continuity 1.7825 / 1.7694 / 1.7652 m`
- Detailed report: [`docs/experiments_control_drone_trajectory_controller_generator_pairings.md`](experiments_control_drone_trajectory_controller_generator_pairings.md)
