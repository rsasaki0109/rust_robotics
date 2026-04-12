# Interfaces

## Current Minimal Interface

The stable surface is intentionally smaller than the implementation set.

```rust
pub use rust_robotics_core::experiments::{
    ExperimentSamplingPlan as RuntimeSamplingPlan,
    ExperimentVariantReport,
    VariantDescriptor,
};

pub trait RuntimeAggregationVariant {
    fn descriptor(&self) -> VariantDescriptor;
    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize>;
    fn sampling_plan(&self, total_scenarios: usize) -> RuntimeSamplingPlan;
}

pub struct RuntimeExperimentCase { /* family, map, scen, buckets */ }
pub struct RuntimeObservation { /* same metrics for every variant */ }
pub type RuntimeVariantReport = ExperimentVariantReport<RuntimeObservation>;

pub fn run_variant_suite(/* variants, cases, config */) -> Vec<RuntimeVariantReport>;
```

## Stability Boundary

- Shared core: `rust_robotics_core::experiments::{VariantDescriptor, ExperimentSamplingPlan, ExperimentVariantReport, annotate_against_reference, read_source_metrics}`.
- Package-local stable surface: `RuntimeAggregationVariant`, `RuntimeExperimentCase`, `RuntimeObservation`, and `run_variant_suite`.
- Experiments: concrete selection strategies under `src/experiments/moving_ai_runtime/`; sibling validation currently lives in `crates/rust_robotics_localization/src/experiments/ukf_ckf_accuracy/`, `crates/rust_robotics_control/src/experiments/path_tracking_accuracy/`, and `crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/`.

## Validated Problem Sets

- `crossover-windows`: [`docs/experiments.md`] + [`docs/decisions.md`]
- `long-tail-windows`: [`docs/experiments_long_tail.md`] + [`docs/decisions_long_tail.md`]
- `synthetic-local-grids`: [`docs/experiments_synthetic.md`] + [`docs/decisions_synthetic.md`]
- `localization-noise-windows`: [`docs/experiments_localization.md`] + [`docs/decisions_localization.md`]
- `localization-long-horizon-windows`: [`docs/experiments_localization_long_horizon.md`] + [`docs/decisions_localization_long_horizon.md`]
- `localization-dropout-bias-windows`: [`docs/experiments_localization_dropout_bias.md`] + [`docs/decisions_localization_dropout_bias.md`]
- `localization-outlier-burst-windows`: [`docs/experiments_localization_outlier_burst.md`] + [`docs/decisions_localization_outlier_burst.md`]
- `localization-process-mismatch-windows`: [`docs/experiments_localization_process_mismatch.md`] + [`docs/decisions_localization_process_mismatch.md`]
- `localization-sensor-rate-mismatch-windows`: [`docs/experiments_localization_sensor_rate_mismatch.md`] + [`docs/decisions_localization_sensor_rate_mismatch.md`]
- `localization-control-latency-windows`: [`docs/experiments_localization_control_latency.md`] + [`docs/decisions_localization_control_latency.md`]
- `localization-process-noise-anisotropy-windows`: [`docs/experiments_localization_process_noise_anisotropy.md`] + [`docs/decisions_localization_process_noise_anisotropy.md`]
- `localization-sensor-bias-burst-windows`: [`docs/experiments_localization_sensor_bias_burst.md`] + [`docs/decisions_localization_sensor_bias_burst.md`]
- `localization-actuator-saturation-windows`: [`docs/experiments_localization_actuator_saturation.md`] + [`docs/decisions_localization_actuator_saturation.md`]
- `control-tracking-windows`: [`docs/experiments_control_tracking.md`] + [`docs/decisions_control_tracking.md`]
- `control-actuation-mismatch-windows`: [`docs/experiments_control_actuation_mismatch.md`] + [`docs/decisions_control_actuation_mismatch.md`]
- `control-drone-trajectory-variants`: [`docs/experiments_control_drone_trajectory.md`] + [`docs/decisions_control_drone_trajectory.md`]
- `control-drone-trajectory-pass-through`: [`docs/experiments_control_drone_trajectory_pass_through.md`] + [`docs/decisions_control_drone_trajectory_pass_through.md`]
- `control-drone-trajectory-pass-through-accel`: [`docs/experiments_control_drone_trajectory_pass_through_accel.md`] + [`docs/decisions_control_drone_trajectory_pass_through_accel.md`]
- `control-drone-trajectory-pass-through-accel-jerk`: [`docs/experiments_control_drone_trajectory_pass_through_accel_jerk.md`] + [`docs/decisions_control_drone_trajectory_pass_through_accel_jerk.md`]
- `control-drone-trajectory-coupled-continuity`: [`docs/experiments_control_drone_trajectory_coupled_continuity.md`] + [`docs/decisions_control_drone_trajectory_coupled_continuity.md`]
- `control-drone-trajectory-coupled-continuity-controllers`: [`docs/experiments_control_drone_trajectory_coupled_continuity_controllers.md`] + [`docs/decisions_control_drone_trajectory_coupled_continuity_controllers.md`]
- `control-drone-trajectory-noncoupled-controllers`: [`docs/experiments_control_drone_trajectory_noncoupled_controllers.md`] + [`docs/decisions_control_drone_trajectory_noncoupled_controllers.md`]
- `control-drone-trajectory-controller-generator-pairings`: [`docs/experiments_control_drone_trajectory_controller_generator_pairings.md`] + [`docs/decisions_control_drone_trajectory_controller_generator_pairings.md`]
- `mapping-point-cloud-sampling-windows`: [`docs/experiments_mapping_sampling.md`] + [`docs/decisions_mapping_sampling.md`]
- `mapping-occlusion-corruption-windows`: [`docs/experiments_mapping_occlusion.md`] + [`docs/decisions_mapping_occlusion.md`]
- `mapping-density-shift-windows`: [`docs/experiments_mapping_density_shift.md`] + [`docs/decisions_mapping_density_shift.md`]
- `mapping-anisotropic-noise-windows`: [`docs/experiments_mapping_anisotropic_noise.md`] + [`docs/decisions_mapping_anisotropic_noise.md`]
- `mapping-sparse-outlier-burst-windows`: [`docs/experiments_mapping_sparse_outlier_burst.md`] + [`docs/decisions_mapping_sparse_outlier_burst.md`]
- `mapping-resolution-ladder-windows`: [`docs/experiments_mapping_resolution_ladder.md`] + [`docs/decisions_mapping_resolution_ladder.md`]
- `planning-grid-threshold-planners`: [`docs/experiments_planning_grid_threshold.md`] + [`docs/decisions_planning_grid_threshold.md`]

## Cross-Package Summaries

- planning summary: [`docs/experiments_planning_summary.md`] + [`docs/decisions_planning_summary.md`]
- localization summary: [`docs/experiments_localization_summary.md`] + [`docs/decisions_localization_summary.md`]
- control summary: [`docs/experiments_control_summary.md`] + [`docs/decisions_control_summary.md`]
- mapping summary: [`docs/experiments_mapping_summary.md`] + [`docs/decisions_mapping_summary.md`]
- workspace summary: [`docs/experiments_workspace_summary.md`] + [`docs/decisions_workspace_summary.md`]

## Active Variants

- `first-scenario`: style=`direct-imperative`, source=`crates/rust_robotics_planning/src/experiments/moving_ai_runtime/first_scenario.rs`
- `sampled-bucket`: style=`configurable-pipeline`, source=`crates/rust_robotics_planning/src/experiments/moving_ai_runtime/sampled_bucket.rs`
- `percentile-bucket`: style=`functional-percentile`, source=`crates/rust_robotics_planning/src/experiments/moving_ai_runtime/percentile_bucket.rs`
- `variance-triggered`: style=`adaptive-two-stage`, source=`crates/rust_robotics_planning/src/experiments/moving_ai_runtime/variance_triggered.rs`
- `full-bucket`: style=`collector-aggregate`, source=`crates/rust_robotics_planning/src/experiments/moving_ai_runtime/full_bucket.rs`

## Planner Comparison Boundary

- Stable caller contract for `planning-grid-threshold-planners`: `PathPlanner::plan(Point2D, Point2D) -> Result<Path2D, RoboticsError>`
- Compared concrete variants remain package-local:
  - `AStarPlanner` in `crates/rust_robotics_planning/src/a_star.rs`
  - `FringeSearchPlanner` in `crates/rust_robotics_planning/src/fringe_search.rs`
  - `IDAStarPlanner` in `crates/rust_robotics_planning/src/ida_star.rs`
- Package-local diagnostic surface for this preset: `IDAStarPlanner::plan_with_report(...) -> IDAStarPlanReport`, with `IDAStarSearchStats::{expanded_nodes, unique_expanded_nodes, reexpanded_nodes, ...}` plus `last_searched_threshold` / `next_threshold` for failure-side contour reporting.
- Current evidence for this preset now spans finite cheap floors on `arena2` bucket-`5` (`22`), bucket-`11` (`43`), bucket-`12` (`132`), a threshold-round floor on bucket `10` (`iter32`), and a staged threshold-round/expansion boundary on bucket `15` (`iter96` fail, `iter128-exp200000` expansion stop, `iter128-exp250000` iteration stop, representative `iter256-exp3000000` still iteration stop, representative `iter512-exp1750775` expansion stop, representative `iter512-exp1750776` exact, full-slice `iter512-exp1750775` mixed, full-slice `iter512-exp1750776` exact, fixed-budget iteration floor `iter275` with `iter274` still failing and a mean next-threshold gap of `0.058875`).
- This preset promoted no new core helper beyond the existing `PathPlanner` boundary.

## Drone Trajectory Boundary

- Stable caller contract for `control-drone-trajectory-variants`: sampled `DesiredState` sequences consumed by the existing quadrotor PD tracker.
- Stable helpers promoted from the preset:
  - `generate_waypoint_trajectory_with_durations`
  - `sample_trajectory_segments`
  - `simulate_desired_states`
- Compared concrete generators remain package-local or experimental:
  - `quintic-uniform` and `quintic-distance-scaled` in [`crates/rust_robotics_control/src/drone_3d_trajectory.rs`](../crates/rust_robotics_control/src/drone_3d_trajectory.rs)
  - `minimum-snap-distance-scaled` in [`crates/rust_robotics_control/src/minimum_snap_trajectory.rs`](../crates/rust_robotics_control/src/minimum_snap_trajectory.rs)
  - evaluation harness in [`crates/rust_robotics_control/src/experiments/drone_trajectory_quality/mod.rs`](../crates/rust_robotics_control/src/experiments/drone_trajectory_quality/mod.rs)
- Current evidence: `quintic-distance-scaled` lowers mean peak speed and acceleration versus `quintic-uniform` at similar generation cost, while `minimum-snap-distance-scaled` remains slower and more aggressive on the current stop-go waypoint preset.
- Additional pass-through evidence: under heuristic non-zero waypoint velocities, `quintic-distance-scaled` still beats `minimum-snap-distance-scaled` on generation cost, RMSE, jerk RMS, and snap RMS, so the stable boundary does not widen yet.
- Additional acceleration-boundary evidence: under heuristic pass-through velocities plus finite-difference waypoint accelerations, `quintic-distance-scaled` still beats `minimum-snap-distance-scaled` on generation cost and the current quality metrics, so no new stable helper is promoted.
- Additional jerk-boundary evidence: under heuristic pass-through velocities plus finite-difference waypoint accelerations and jerk, `quintic-distance-scaled` still beats `minimum-snap-distance-scaled` on generation cost and the current quality metrics, and the added jerk term only affects the minimum-snap branch, so no new stable helper is promoted.
- Additional coupled-continuity evidence: globally solved closed-loop waypoint boundaries dramatically reduce jerk and snap for both generators, but tracking RMSE grows on the current PD tracker, so the stable helper surface still should not widen and controller/feedforward comparison is now the clearer next step.
- Additional controller-variant evidence: under the coupled-continuity preset, pure attitude-rate damping stays mixed, but a bounded lateral-feedback variant on top of the same damping recovers the figure-eight regressions and sharply lowers mean tracking error while keeping control effort low; controller variants still remain experimental and package-local because that result is only validated on this preset so far.
- Additional non-coupled controller evidence: the same bounded lateral-feedback variant now also beats the baseline and pure damping controllers across the tested stop-go and pass-through-family presets, but those controller variants still remain package-local because the gain is only characterized through experiment reruns, not through a stable controller abstraction.
- Additional pairing evidence: explicit controller + generator reruns show the bounded lateral-feedback controller does change some local generator winner labels, but it still does not justify widening the stable boundary; `quintic-distance-scaled` remains the practical non-coupled choice, while the tiny coupled-only minimum-snap edge stays experiment-local.
