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
- `mapping-point-cloud-sampling-windows`: [`docs/experiments_mapping_sampling.md`] + [`docs/decisions_mapping_sampling.md`]
- `mapping-occlusion-corruption-windows`: [`docs/experiments_mapping_occlusion.md`] + [`docs/decisions_mapping_occlusion.md`]
- `mapping-density-shift-windows`: [`docs/experiments_mapping_density_shift.md`] + [`docs/decisions_mapping_density_shift.md`]
- `mapping-anisotropic-noise-windows`: [`docs/experiments_mapping_anisotropic_noise.md`] + [`docs/decisions_mapping_anisotropic_noise.md`]
- `mapping-sparse-outlier-burst-windows`: [`docs/experiments_mapping_sparse_outlier_burst.md`] + [`docs/decisions_mapping_sparse_outlier_burst.md`]
- `mapping-resolution-ladder-windows`: [`docs/experiments_mapping_resolution_ladder.md`] + [`docs/decisions_mapping_resolution_ladder.md`]

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
