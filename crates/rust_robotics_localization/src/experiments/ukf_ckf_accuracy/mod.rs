mod first_scenario;
mod full_bucket;
mod percentile_bucket;
mod sampled_bucket;
mod variance_triggered;

use std::time::Instant;

use nalgebra::{Matrix2, Matrix4, Vector2, Vector4};
use rand::{Rng, SeedableRng};
use rand_distr::{Distribution, Normal};
use rust_robotics_core::{
    annotate_against_reference as annotate_reference_reports, average_coverage_ratio,
    read_source_metrics, ExperimentObservation, ExperimentSamplingPlan, ExperimentVariantReport,
    ExtensibilityMetrics, VariantDescriptor,
};

use crate::{
    cubature_kalman_filter::{CKFConfig, CKFLocalizer},
    unscented_kalman_filter::{UKFConfig, UKFLocalizer, UKFParams},
};

pub use first_scenario::FirstScenarioAccuracyAggregation;
pub use full_bucket::FullBucketAccuracyAggregation;
pub use percentile_bucket::PercentileBucketAccuracyAggregation;
pub use sampled_bucket::SampledBucketAccuracyAggregation;
pub use variance_triggered::VarianceTriggeredAccuracyAggregation;

const DT: f64 = 0.1;

pub type AccuracySamplingPlan = ExperimentSamplingPlan;

pub trait AccuracyAggregationVariant {
    fn descriptor(&self) -> VariantDescriptor;
    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize>;

    fn sampling_plan(&self, total_scenarios: usize) -> AccuracySamplingPlan {
        AccuracySamplingPlan::static_slots(self.selected_slots(total_scenarios))
    }
}

#[derive(Debug, Clone, Copy)]
struct MotionProfile {
    velocity: f64,
    yaw_rate: f64,
    true_velocity_scale: f64,
    true_velocity_wave: f64,
    true_yaw_rate_scale: f64,
    true_yaw_wave_deg: f64,
    command_velocity_wave: f64,
    command_yaw_wave_deg: f64,
    control_latency_steps: usize,
    actuator_velocity_limit: f64,
    actuator_yaw_limit_deg: f64,
    process_noise_longitudinal: f64,
    process_noise_lateral: f64,
    process_noise_yaw_deg: f64,
    control_noise_v: f64,
    control_noise_yaw_deg: f64,
    control_bias_v: f64,
    control_bias_yaw_deg: f64,
    obs_noise_x: f64,
    obs_noise_y: f64,
    observation_refresh_interval: usize,
    observation_hold_probability: f64,
    observation_outlier_probability: f64,
    observation_outlier_scale: f64,
    observation_outlier_burst_len: usize,
    observation_bias_burst_interval: usize,
}

#[derive(Debug, Clone, Copy)]
pub struct AccuracyExperimentCase {
    pub family_name: &'static str,
    seed_offset: u64,
    profile: MotionProfile,
    pub buckets: &'static [u32],
}

#[derive(Debug, Clone)]
pub struct AccuracyObservation {
    pub family_name: &'static str,
    pub bucket: u32,
    pub total_scenarios: usize,
    pub initial_slots: Vec<usize>,
    pub selected_slots: Vec<usize>,
    pub escalated: bool,
    pub ukf_bucket_median_rmse: f64,
    pub ckf_bucket_median_rmse: f64,
    pub ukf_min_rmse: f64,
    pub ukf_max_rmse: f64,
    pub ckf_min_rmse: f64,
    pub ckf_max_rmse: f64,
    pub ckf_wins: usize,
}

impl AccuracyObservation {
    pub fn ukf_over_ckf(&self) -> f64 {
        self.ukf_bucket_median_rmse / self.ckf_bucket_median_rmse.max(1e-9)
    }

    pub fn winner(&self) -> &'static str {
        if self.ukf_over_ckf() > 1.0 {
            "CKF"
        } else {
            "UKF"
        }
    }

    pub fn coverage_ratio(&self) -> f64 {
        self.selected_slots.len() as f64 / self.total_scenarios as f64
    }
}

impl ExperimentObservation for AccuracyObservation {
    type Key = (&'static str, u32);

    fn comparison_key(&self) -> Self::Key {
        (self.family_name, self.bucket)
    }

    fn winner_label(&self) -> &'static str {
        AccuracyObservation::winner(self)
    }

    fn ratio_value(&self) -> f64 {
        AccuracyObservation::ukf_over_ckf(self)
    }

    fn coverage_ratio(&self) -> f64 {
        AccuracyObservation::coverage_ratio(self)
    }
}

pub type AccuracyVariantReport = ExperimentVariantReport<AccuracyObservation>;

#[derive(Debug, Clone, Copy)]
pub struct AccuracyEvaluationConfig {
    pub scenarios_per_bucket: usize,
}

impl Default for AccuracyEvaluationConfig {
    fn default() -> Self {
        Self {
            scenarios_per_bucket: 10,
        }
    }
}

pub fn localization_noise_process_problem() -> Vec<AccuracyExperimentCase> {
    vec![
        AccuracyExperimentCase {
            family_name: "balanced-circle",
            seed_offset: 10_000,
            profile: MotionProfile {
                velocity: 1.0,
                yaw_rate: 0.10,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.25,
                control_noise_yaw_deg: 4.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.35,
                obs_noise_y: 0.35,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[60, 100, 140],
        },
        AccuracyExperimentCase {
            family_name: "fast-turn",
            seed_offset: 20_000,
            profile: MotionProfile {
                velocity: 1.3,
                yaw_rate: 0.22,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.30,
                control_noise_yaw_deg: 6.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.40,
                obs_noise_y: 0.40,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[80, 120, 160],
        },
        AccuracyExperimentCase {
            family_name: "gps-stress",
            seed_offset: 30_000,
            profile: MotionProfile {
                velocity: 0.9,
                yaw_rate: 0.14,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.20,
                control_noise_yaw_deg: 5.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.30,
                obs_noise_y: 0.75,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[80, 120, 160],
        },
    ]
}

pub fn localization_long_horizon_process_problem() -> Vec<AccuracyExperimentCase> {
    vec![
        AccuracyExperimentCase {
            family_name: "long-horizon-turn",
            seed_offset: 40_000,
            profile: MotionProfile {
                velocity: 1.1,
                yaw_rate: 0.18,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.35,
                control_noise_yaw_deg: 8.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.45,
                obs_noise_y: 0.45,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[180, 240, 320],
        },
        AccuracyExperimentCase {
            family_name: "high-curvature",
            seed_offset: 50_000,
            profile: MotionProfile {
                velocity: 0.8,
                yaw_rate: 0.32,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.28,
                control_noise_yaw_deg: 9.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.40,
                obs_noise_y: 0.40,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[140, 220, 300],
        },
        AccuracyExperimentCase {
            family_name: "anisotropic-gps",
            seed_offset: 60_000,
            profile: MotionProfile {
                velocity: 1.0,
                yaw_rate: 0.12,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.22,
                control_noise_yaw_deg: 5.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.25,
                obs_noise_y: 1.10,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[160, 240, 320],
        },
    ]
}

pub fn localization_dropout_bias_process_problem() -> Vec<AccuracyExperimentCase> {
    vec![
        AccuracyExperimentCase {
            family_name: "dropout-lag",
            seed_offset: 70_000,
            profile: MotionProfile {
                velocity: 1.0,
                yaw_rate: 0.16,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.24,
                control_noise_yaw_deg: 5.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.35,
                obs_noise_y: 0.35,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.18,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[100, 180, 260],
        },
        AccuracyExperimentCase {
            family_name: "biased-odometry",
            seed_offset: 80_000,
            profile: MotionProfile {
                velocity: 1.1,
                yaw_rate: 0.14,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.20,
                control_noise_yaw_deg: 4.5,
                control_bias_v: 0.08,
                control_bias_yaw_deg: 1.8,
                obs_noise_x: 0.32,
                obs_noise_y: 0.32,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[120, 200, 280],
        },
        AccuracyExperimentCase {
            family_name: "dropout-bias-mix",
            seed_offset: 90_000,
            profile: MotionProfile {
                velocity: 0.95,
                yaw_rate: 0.24,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.26,
                control_noise_yaw_deg: 6.5,
                control_bias_v: -0.05,
                control_bias_yaw_deg: 2.4,
                obs_noise_x: 0.30,
                obs_noise_y: 0.85,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.22,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[120, 200, 280],
        },
    ]
}

pub fn localization_outlier_burst_process_problem() -> Vec<AccuracyExperimentCase> {
    vec![
        AccuracyExperimentCase {
            family_name: "sparse-outlier-burst",
            seed_offset: 100_000,
            profile: MotionProfile {
                velocity: 1.0,
                yaw_rate: 0.15,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.20,
                control_noise_yaw_deg: 4.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.30,
                obs_noise_y: 0.30,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.05,
                observation_outlier_scale: 6.0,
                observation_outlier_burst_len: 3,
                observation_bias_burst_interval: 0,
            },
            buckets: &[120, 200, 280],
        },
        AccuracyExperimentCase {
            family_name: "anisotropic-outlier-burst",
            seed_offset: 110_000,
            profile: MotionProfile {
                velocity: 0.95,
                yaw_rate: 0.20,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.24,
                control_noise_yaw_deg: 5.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.25,
                obs_noise_y: 0.95,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.06,
                observation_outlier_scale: 7.5,
                observation_outlier_burst_len: 4,
                observation_bias_burst_interval: 0,
            },
            buckets: &[120, 220, 320],
        },
        AccuracyExperimentCase {
            family_name: "burst-plus-bias",
            seed_offset: 120_000,
            profile: MotionProfile {
                velocity: 1.1,
                yaw_rate: 0.11,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.22,
                control_noise_yaw_deg: 4.0,
                control_bias_v: 0.06,
                control_bias_yaw_deg: 1.4,
                obs_noise_x: 0.28,
                obs_noise_y: 0.28,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.04,
                observation_outlier_scale: 5.5,
                observation_outlier_burst_len: 3,
                observation_bias_burst_interval: 0,
            },
            buckets: &[140, 220, 300],
        },
    ]
}

pub fn localization_process_mismatch_process_problem() -> Vec<AccuracyExperimentCase> {
    vec![
        AccuracyExperimentCase {
            family_name: "slip-understeer",
            seed_offset: 130_000,
            profile: MotionProfile {
                velocity: 1.1,
                yaw_rate: 0.15,
                true_velocity_scale: 0.88,
                true_velocity_wave: 0.10,
                true_yaw_rate_scale: 0.78,
                true_yaw_wave_deg: 4.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.22,
                control_noise_yaw_deg: 4.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.30,
                obs_noise_y: 0.30,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[120, 200, 280],
        },
        AccuracyExperimentCase {
            family_name: "actuator-scale-drift",
            seed_offset: 140_000,
            profile: MotionProfile {
                velocity: 0.95,
                yaw_rate: 0.22,
                true_velocity_scale: 1.12,
                true_velocity_wave: 0.14,
                true_yaw_rate_scale: 1.22,
                true_yaw_wave_deg: 6.5,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.24,
                control_noise_yaw_deg: 5.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.28,
                obs_noise_y: 0.55,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[140, 220, 320],
        },
        AccuracyExperimentCase {
            family_name: "oscillatory-wheel-slip",
            seed_offset: 150_000,
            profile: MotionProfile {
                velocity: 1.05,
                yaw_rate: 0.18,
                true_velocity_scale: 0.94,
                true_velocity_wave: 0.20,
                true_yaw_rate_scale: 1.08,
                true_yaw_wave_deg: 8.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.26,
                control_noise_yaw_deg: 6.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.32,
                obs_noise_y: 0.32,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[140, 240, 340],
        },
    ]
}

pub fn localization_sensor_rate_mismatch_process_problem() -> Vec<AccuracyExperimentCase> {
    vec![
        AccuracyExperimentCase {
            family_name: "slow-gps-3x",
            seed_offset: 160_000,
            profile: MotionProfile {
                velocity: 1.0,
                yaw_rate: 0.16,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.14,
                command_yaw_wave_deg: 4.0,
                control_latency_steps: 2,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.22,
                control_noise_yaw_deg: 4.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.28,
                obs_noise_y: 0.28,
                observation_refresh_interval: 3,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[120, 200, 280],
        },
        AccuracyExperimentCase {
            family_name: "anisotropic-gps-5x",
            seed_offset: 170_000,
            profile: MotionProfile {
                velocity: 0.95,
                yaw_rate: 0.20,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.10,
                command_yaw_wave_deg: 5.5,
                control_latency_steps: 4,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.24,
                control_noise_yaw_deg: 5.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.24,
                obs_noise_y: 0.95,
                observation_refresh_interval: 5,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[140, 220, 320],
        },
        AccuracyExperimentCase {
            family_name: "turning-camera-4x",
            seed_offset: 180_000,
            profile: MotionProfile {
                velocity: 1.1,
                yaw_rate: 0.26,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.16,
                command_yaw_wave_deg: 7.0,
                control_latency_steps: 3,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.26,
                control_noise_yaw_deg: 6.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.35,
                obs_noise_y: 0.35,
                observation_refresh_interval: 4,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[140, 240, 340],
        },
    ]
}

pub fn localization_control_latency_process_problem() -> Vec<AccuracyExperimentCase> {
    vec![
        AccuracyExperimentCase {
            family_name: "steering-lag-2",
            seed_offset: 190_000,
            profile: MotionProfile {
                velocity: 1.0,
                yaw_rate: 0.18,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.12,
                command_yaw_wave_deg: 5.0,
                control_latency_steps: 2,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.22,
                control_noise_yaw_deg: 4.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.30,
                obs_noise_y: 0.30,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[120, 200, 280],
        },
        AccuracyExperimentCase {
            family_name: "drive-lag-4",
            seed_offset: 200_000,
            profile: MotionProfile {
                velocity: 0.95,
                yaw_rate: 0.22,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.18,
                command_yaw_wave_deg: 6.0,
                control_latency_steps: 4,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.24,
                control_noise_yaw_deg: 5.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.28,
                obs_noise_y: 0.55,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[140, 220, 320],
        },
        AccuracyExperimentCase {
            family_name: "oscillatory-command-lag-3",
            seed_offset: 210_000,
            profile: MotionProfile {
                velocity: 1.1,
                yaw_rate: 0.26,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.14,
                command_yaw_wave_deg: 8.0,
                control_latency_steps: 3,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.26,
                control_noise_yaw_deg: 6.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.32,
                obs_noise_y: 0.32,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[140, 240, 340],
        },
    ]
}

pub fn localization_process_noise_anisotropy_process_problem() -> Vec<AccuracyExperimentCase> {
    vec![
        AccuracyExperimentCase {
            family_name: "crosswind-corridor",
            seed_offset: 220_000,
            profile: MotionProfile {
                velocity: 1.0,
                yaw_rate: 0.16,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.01,
                process_noise_lateral: 0.18,
                process_noise_yaw_deg: 1.2,
                control_noise_v: 0.22,
                control_noise_yaw_deg: 4.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.28,
                obs_noise_y: 0.28,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[120, 200, 280],
        },
        AccuracyExperimentCase {
            family_name: "traction-slip-turn",
            seed_offset: 230_000,
            profile: MotionProfile {
                velocity: 0.95,
                yaw_rate: 0.24,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.04,
                process_noise_lateral: 0.12,
                process_noise_yaw_deg: 3.5,
                control_noise_v: 0.24,
                control_noise_yaw_deg: 5.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.30,
                obs_noise_y: 0.55,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[140, 220, 320],
        },
        AccuracyExperimentCase {
            family_name: "yaw-dominant-drift",
            seed_offset: 240_000,
            profile: MotionProfile {
                velocity: 1.1,
                yaw_rate: 0.28,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.02,
                process_noise_lateral: 0.10,
                process_noise_yaw_deg: 5.0,
                control_noise_v: 0.26,
                control_noise_yaw_deg: 6.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.32,
                obs_noise_y: 0.32,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[140, 240, 340],
        },
    ]
}

pub fn localization_sensor_bias_burst_process_problem() -> Vec<AccuracyExperimentCase> {
    vec![
        AccuracyExperimentCase {
            family_name: "gps-bias-burst",
            seed_offset: 250_000,
            profile: MotionProfile {
                velocity: 1.0,
                yaw_rate: 0.16,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.22,
                control_noise_yaw_deg: 4.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.28,
                obs_noise_y: 0.28,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.35,
                observation_outlier_burst_len: 6,
                observation_bias_burst_interval: 45,
            },
            buckets: &[120, 200, 280],
        },
        AccuracyExperimentCase {
            family_name: "camera-bias-burst",
            seed_offset: 260_000,
            profile: MotionProfile {
                velocity: 0.95,
                yaw_rate: 0.24,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.24,
                control_noise_yaw_deg: 5.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.30,
                obs_noise_y: 0.55,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.45,
                observation_outlier_burst_len: 5,
                observation_bias_burst_interval: 36,
            },
            buckets: &[140, 220, 320],
        },
        AccuracyExperimentCase {
            family_name: "anisotropic-bias-burst",
            seed_offset: 270_000,
            profile: MotionProfile {
                velocity: 1.1,
                yaw_rate: 0.28,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.26,
                control_noise_yaw_deg: 6.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.32,
                obs_noise_y: 0.32,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.55,
                observation_outlier_burst_len: 4,
                observation_bias_burst_interval: 30,
            },
            buckets: &[140, 240, 340],
        },
    ]
}

pub fn localization_actuator_saturation_process_problem() -> Vec<AccuracyExperimentCase> {
    vec![
        AccuracyExperimentCase {
            family_name: "velocity-clipping",
            seed_offset: 280_000,
            profile: MotionProfile {
                velocity: 1.0,
                yaw_rate: 0.16,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.42,
                command_yaw_wave_deg: 4.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 1.05,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.22,
                control_noise_yaw_deg: 4.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.28,
                obs_noise_y: 0.28,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[120, 200, 280],
        },
        AccuracyExperimentCase {
            family_name: "steering-clipping",
            seed_offset: 290_000,
            profile: MotionProfile {
                velocity: 0.95,
                yaw_rate: 0.26,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.18,
                command_yaw_wave_deg: 15.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 10.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.24,
                control_noise_yaw_deg: 5.5,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.30,
                obs_noise_y: 0.55,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[140, 220, 320],
        },
        AccuracyExperimentCase {
            family_name: "coupled-saturation",
            seed_offset: 300_000,
            profile: MotionProfile {
                velocity: 1.1,
                yaw_rate: 0.22,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.30,
                command_yaw_wave_deg: 12.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 1.18,
                actuator_yaw_limit_deg: 11.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.26,
                control_noise_yaw_deg: 6.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.32,
                obs_noise_y: 0.32,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[140, 240, 340],
        },
    ]
}

pub fn default_accuracy_variants() -> Vec<Box<dyn AccuracyAggregationVariant>> {
    vec![
        Box::new(FirstScenarioAccuracyAggregation::new()),
        Box::new(SampledBucketAccuracyAggregation::new(vec![0, 4, 9])),
        Box::new(PercentileBucketAccuracyAggregation::new(vec![
            0.0, 0.25, 0.5, 0.75, 1.0,
        ])),
        Box::new(VarianceTriggeredAccuracyAggregation::new(
            vec![0, 4, 9],
            0.10,
        )),
        Box::new(FullBucketAccuracyAggregation::new()),
    ]
}

pub fn run_variant_suite(
    variants: &[Box<dyn AccuracyAggregationVariant>],
    cases: &[AccuracyExperimentCase],
    config: AccuracyEvaluationConfig,
) -> Vec<AccuracyVariantReport> {
    let mut reports = Vec::with_capacity(variants.len());

    for variant in variants {
        let started = Instant::now();
        let mut observations = Vec::new();
        for case in cases {
            for &bucket in case.buckets {
                observations.push(measure_bucket_observation(
                    &**variant,
                    case,
                    bucket,
                    config.scenarios_per_bucket,
                ));
            }
        }
        let descriptor = variant.descriptor();
        let source_metrics = read_source_metrics(std::path::Path::new(descriptor.source_path))
            .expect("experiment source metrics should be readable");
        let average_coverage_ratio = average_coverage_ratio(&observations);
        let extensibility_metrics = ExtensibilityMetrics {
            average_coverage_ratio,
            knob_count: descriptor.knob_count,
            reports_dispersion: descriptor.reports_dispersion,
        };
        reports.push(AccuracyVariantReport {
            descriptor,
            evaluation_runtime_ms: started.elapsed().as_secs_f64() * 1000.0,
            observations,
            source_metrics,
            extensibility_metrics,
            agreement_vs_reference: None,
            mean_ratio_error_vs_reference: None,
        });
    }

    annotate_reference_reports(&mut reports, "full-bucket");
    reports
}

fn measure_bucket_observation(
    variant: &dyn AccuracyAggregationVariant,
    case: &AccuracyExperimentCase,
    bucket: u32,
    total_scenarios: usize,
) -> AccuracyObservation {
    let plan = variant.sampling_plan(total_scenarios);
    let initial_slots = normalize_slots(total_scenarios, &plan.initial_slots);
    assert!(
        !initial_slots.is_empty(),
        "{} bucket {} should select at least one scenario",
        case.family_name,
        bucket
    );

    let mut slot_samples = measure_slot_samples(&initial_slots, case, bucket);
    let mut selected_slots = initial_slots.clone();
    let mut escalated = false;
    let escalation_slots = normalize_slots(total_scenarios, &plan.escalation_slots);

    if should_escalate(&slot_samples, &plan) {
        let additional_slots: Vec<usize> = escalation_slots
            .into_iter()
            .filter(|slot| !selected_slots.contains(slot))
            .collect();
        if !additional_slots.is_empty() {
            slot_samples.extend(measure_slot_samples(&additional_slots, case, bucket));
            selected_slots.extend(additional_slots);
            selected_slots.sort_unstable();
            escalated = true;
        }
    }

    let ukf_samples: Vec<f64> = slot_samples.iter().map(|sample| sample.ukf_rmse).collect();
    let ckf_samples: Vec<f64> = slot_samples.iter().map(|sample| sample.ckf_rmse).collect();
    let ckf_wins = slot_samples
        .iter()
        .filter(|sample| sample.ckf_rmse < sample.ukf_rmse)
        .count();

    AccuracyObservation {
        family_name: case.family_name,
        bucket,
        total_scenarios,
        initial_slots,
        selected_slots,
        escalated,
        ukf_bucket_median_rmse: median_value(&ukf_samples),
        ckf_bucket_median_rmse: median_value(&ckf_samples),
        ukf_min_rmse: min_value(&ukf_samples),
        ukf_max_rmse: max_value(&ukf_samples),
        ckf_min_rmse: min_value(&ckf_samples),
        ckf_max_rmse: max_value(&ckf_samples),
        ckf_wins,
    }
}

#[derive(Debug, Clone, Copy)]
struct SlotAccuracySample {
    ukf_rmse: f64,
    ckf_rmse: f64,
}

fn measure_slot_samples(
    slots: &[usize],
    case: &AccuracyExperimentCase,
    bucket: u32,
) -> Vec<SlotAccuracySample> {
    let mut samples = Vec::with_capacity(slots.len());
    for slot in slots {
        let data = generate_sim_data(case, bucket, *slot);
        let ukf_rmse = run_ukf_rmse(&data);
        let ckf_rmse = run_ckf_rmse(&data);
        assert!(ukf_rmse.is_finite(), "UKF RMSE should stay finite");
        assert!(ckf_rmse.is_finite(), "CKF RMSE should stay finite");
        samples.push(SlotAccuracySample { ukf_rmse, ckf_rmse });
    }
    samples
}

fn should_escalate(slot_samples: &[SlotAccuracySample], plan: &AccuracySamplingPlan) -> bool {
    if slot_samples.is_empty() || plan.escalation_slots.is_empty() {
        return false;
    }

    let vote_split = {
        let ckf_wins = slot_samples
            .iter()
            .filter(|sample| sample.ckf_rmse < sample.ukf_rmse)
            .count();
        ckf_wins > 0 && ckf_wins < slot_samples.len()
    };
    let ratio_close = plan
        .escalate_if_ratio_margin_below
        .map(|threshold| {
            let ukf_samples: Vec<f64> = slot_samples.iter().map(|sample| sample.ukf_rmse).collect();
            let ckf_samples: Vec<f64> = slot_samples.iter().map(|sample| sample.ckf_rmse).collect();
            (median_value(&ukf_samples) / median_value(&ckf_samples).max(1e-9) - 1.0).abs()
                < threshold
        })
        .unwrap_or(false);

    (plan.escalate_if_vote_split && vote_split) || ratio_close
}

fn normalize_slots(total_scenarios: usize, slots: &[usize]) -> Vec<usize> {
    let mut normalized = slots
        .iter()
        .copied()
        .filter(|slot| *slot < total_scenarios)
        .collect::<Vec<_>>();
    normalized.sort_unstable();
    normalized.dedup();
    normalized
}

#[derive(Debug)]
struct SimData {
    ground_truth: Vec<Vector4<f64>>,
    noisy_controls: Vec<Vector2<f64>>,
    noisy_observations: Vec<Vector2<f64>>,
}

fn generate_sim_data(case: &AccuracyExperimentCase, bucket: u32, slot: usize) -> SimData {
    let steps = bucket as usize;
    let scale = bucket as f64 / 100.0;
    let profile = case.profile;
    let mut rng =
        rand::rngs::StdRng::seed_from_u64(case.seed_offset + bucket as u64 * 100 + slot as u64);

    let noise_v = Normal::new(0.0, profile.control_noise_v * scale).unwrap();
    let noise_yaw = Normal::new(0.0, profile.control_noise_yaw_deg.to_radians() * scale).unwrap();
    let noise_obs_x = Normal::new(0.0, profile.obs_noise_x * scale).unwrap();
    let noise_obs_y = Normal::new(0.0, profile.obs_noise_y * scale).unwrap();

    let initial_true_control = true_control_for_step(profile, scale, 0, slot);
    let mut state = Vector4::new(0.0, 0.0, 0.0, initial_true_control[0]);
    let mut last_observation = None;
    let mut outlier_burst_steps_remaining = 0usize;
    let mut outlier_offset = Vector2::zeros();

    let mut ground_truth = Vec::with_capacity(steps);
    let mut noisy_controls = Vec::with_capacity(steps);
    let mut noisy_observations = Vec::with_capacity(steps);
    let observation_refresh_interval = profile.observation_refresh_interval.max(1);
    let refresh_phase = slot % observation_refresh_interval;

    for step in 0..steps {
        let commanded_control = commanded_control_for_step(profile, scale, step, slot);
        let true_control = true_control_for_step(profile, scale, step, slot);
        state = motion_model(&state, &true_control, DT);
        state = apply_process_disturbance(state, profile, scale, &mut rng);

        let noisy_control = Vector2::new(
            commanded_control[0] + profile.control_bias_v + noise_v.sample(&mut rng),
            commanded_control[1]
                + profile.control_bias_yaw_deg.to_radians()
                + noise_yaw.sample(&mut rng),
        );
        let fresh_observation = Vector2::new(
            state[0] + noise_obs_x.sample(&mut rng),
            state[1] + noise_obs_y.sample(&mut rng),
        );
        if outlier_burst_steps_remaining == 0
            && profile.observation_outlier_probability > 0.0
            && rng.random::<f64>() < profile.observation_outlier_probability
        {
            outlier_burst_steps_remaining = profile.observation_outlier_burst_len.max(1);
            outlier_offset = Vector2::new(
                Normal::new(0.0, profile.observation_outlier_scale * scale)
                    .unwrap()
                    .sample(&mut rng),
                Normal::new(0.0, profile.observation_outlier_scale * scale)
                    .unwrap()
                    .sample(&mut rng),
            );
        }
        let burst_adjusted_observation = if outlier_burst_steps_remaining > 0 {
            outlier_burst_steps_remaining -= 1;
            fresh_observation + outlier_offset
        } else {
            fresh_observation
        };
        let bias_adjusted_observation =
            burst_adjusted_observation + sensor_bias_offset_for_step(profile, scale, step, slot);
        let reused_by_cadence =
            last_observation.is_some() && step % observation_refresh_interval != refresh_phase;
        let cadence_observation = if reused_by_cadence {
            last_observation.expect("stale observation should exist")
        } else {
            bias_adjusted_observation
        };
        let noisy_observation = match (reused_by_cadence, last_observation) {
            (true, _) | (_, None) => cadence_observation,
            (false, Some(prev)) if rng.random::<f64>() < profile.observation_hold_probability => {
                prev
            }
            _ => bias_adjusted_observation,
        };

        ground_truth.push(state);
        noisy_controls.push(noisy_control);
        noisy_observations.push(noisy_observation);
        last_observation = Some(noisy_observation);
    }

    SimData {
        ground_truth,
        noisy_controls,
        noisy_observations,
    }
}

fn true_control_for_step(
    profile: MotionProfile,
    scale: f64,
    step: usize,
    slot: usize,
) -> Vector2<f64> {
    let delayed_step = step.saturating_sub(profile.control_latency_steps);
    let commanded_control = apply_actuator_saturation(
        profile,
        commanded_control_for_step(profile, scale, delayed_step, slot),
    );
    let velocity_phase = step as f64 * 0.07 + slot as f64 * 0.31;
    let yaw_phase = step as f64 * 0.05 + slot as f64 * 0.17 + 0.6;
    let true_velocity = (commanded_control[0] * profile.true_velocity_scale
        + profile.true_velocity_wave * scale * velocity_phase.sin())
    .max(0.05);
    let true_yaw_rate = commanded_control[1] * profile.true_yaw_rate_scale
        + profile.true_yaw_wave_deg.to_radians() * scale * yaw_phase.cos();

    Vector2::new(true_velocity, true_yaw_rate)
}

fn commanded_control_for_step(
    profile: MotionProfile,
    scale: f64,
    step: usize,
    slot: usize,
) -> Vector2<f64> {
    let velocity_phase = step as f64 * 0.09 + slot as f64 * 0.21 + 0.3;
    let yaw_phase = step as f64 * 0.06 + slot as f64 * 0.27 + 0.8;
    let commanded_velocity =
        (profile.velocity + profile.command_velocity_wave * scale * velocity_phase.sin()).max(0.05);
    let commanded_yaw_rate =
        profile.yaw_rate + profile.command_yaw_wave_deg.to_radians() * scale * yaw_phase.cos();

    Vector2::new(commanded_velocity, commanded_yaw_rate)
}

fn apply_actuator_saturation(profile: MotionProfile, control: Vector2<f64>) -> Vector2<f64> {
    let velocity_limit = profile.actuator_velocity_limit;
    let yaw_limit = profile.actuator_yaw_limit_deg.to_radians();
    let saturated_velocity = if velocity_limit > 0.0 {
        control[0].clamp(-velocity_limit, velocity_limit)
    } else {
        control[0]
    };
    let saturated_yaw = if yaw_limit > 0.0 {
        control[1].clamp(-yaw_limit, yaw_limit)
    } else {
        control[1]
    };

    Vector2::new(saturated_velocity, saturated_yaw)
}

fn apply_process_disturbance(
    mut state: Vector4<f64>,
    profile: MotionProfile,
    scale: f64,
    rng: &mut rand::rngs::StdRng,
) -> Vector4<f64> {
    let longitudinal = sample_zero_mean(rng, profile.process_noise_longitudinal * scale);
    let lateral = sample_zero_mean(rng, profile.process_noise_lateral * scale);
    let yaw_noise = sample_zero_mean(rng, profile.process_noise_yaw_deg.to_radians() * scale);

    if longitudinal == 0.0 && lateral == 0.0 && yaw_noise == 0.0 {
        return state;
    }

    let yaw = state[2];
    state[0] += longitudinal * yaw.cos() - lateral * yaw.sin();
    state[1] += longitudinal * yaw.sin() + lateral * yaw.cos();
    state[2] += yaw_noise;
    state
}

fn sensor_bias_offset_for_step(
    profile: MotionProfile,
    scale: f64,
    step: usize,
    slot: usize,
) -> Vector2<f64> {
    let interval = profile.observation_bias_burst_interval;
    if interval == 0 {
        return Vector2::zeros();
    }

    let burst_len = profile.observation_outlier_burst_len.max(1);
    let phase = slot % interval;
    if (step + phase) % interval >= burst_len {
        return Vector2::zeros();
    }

    let magnitude = profile.observation_outlier_scale * scale;
    let direction = if slot.is_multiple_of(2) { 1.0 } else { -1.0 };
    Vector2::new(magnitude, direction * 0.6 * magnitude)
}

fn sample_zero_mean(rng: &mut rand::rngs::StdRng, sigma: f64) -> f64 {
    if sigma <= 0.0 {
        0.0
    } else {
        Normal::new(0.0, sigma).unwrap().sample(rng)
    }
}

fn motion_model(x: &Vector4<f64>, u: &Vector2<f64>, dt: f64) -> Vector4<f64> {
    let yaw = x[2];
    Vector4::new(
        x[0] + dt * u[0] * yaw.cos(),
        x[1] + dt * u[0] * yaw.sin(),
        x[2] + dt * u[1],
        u[0],
    )
}

fn run_ukf_rmse(data: &SimData) -> f64 {
    let first_observation = data
        .noisy_observations
        .first()
        .copied()
        .unwrap_or_else(Vector2::zeros);
    let observation_var_x = variance(data.noisy_observations.iter().map(|z| z[0]));
    let observation_var_y = variance(data.noisy_observations.iter().map(|z| z[1]));
    let control_var_v = variance(data.noisy_controls.iter().map(|u| u[0]));
    let control_var_yaw = variance(data.noisy_controls.iter().map(|u| u[1]));

    let config = UKFConfig {
        params: UKFParams::default(),
        process_noise: Vector4::new(
            control_var_v.max(1e-4),
            control_var_v.max(1e-4),
            control_var_yaw.max(1e-5),
            control_var_v.max(1e-4),
        ),
        observation_noise: Vector2::new(observation_var_x.max(1e-4), observation_var_y.max(1e-4)),
        dt: DT,
    };
    let mut ukf = UKFLocalizer::with_initial_state(
        Vector4::new(first_observation[0], first_observation[1], 0.0, 0.0),
        config,
    );

    let mut positions = Vec::with_capacity(data.ground_truth.len());
    for (control, observation) in data
        .noisy_controls
        .iter()
        .zip(data.noisy_observations.iter())
    {
        ukf.step(control, observation);
        let state = ukf.estimate();
        positions.push((state[0], state[1]));
    }
    compute_rmse(&positions, &data.ground_truth)
}

fn run_ckf_rmse(data: &SimData) -> f64 {
    let first_observation = data
        .noisy_observations
        .first()
        .copied()
        .unwrap_or_else(Vector2::zeros);
    let observation_var_x = variance(data.noisy_observations.iter().map(|z| z[0]));
    let observation_var_y = variance(data.noisy_observations.iter().map(|z| z[1]));
    let control_var_v = variance(data.noisy_controls.iter().map(|u| u[0]));
    let control_var_yaw = variance(data.noisy_controls.iter().map(|u| u[1]));

    let mut r = Matrix2::<f64>::identity();
    r[(0, 0)] = observation_var_x.max(1e-4);
    r[(1, 1)] = observation_var_y.max(1e-4);
    let q = Matrix4::from_diagonal(&Vector4::new(
        control_var_v.max(1e-4),
        control_var_v.max(1e-4),
        control_var_yaw.max(1e-5),
        control_var_v.max(1e-4),
    ));

    let mut ckf = CKFLocalizer::with_initial_state(
        Vector4::new(first_observation[0], first_observation[1], 0.0, 0.0),
        CKFConfig { q, r },
    );

    let mut positions = Vec::with_capacity(data.ground_truth.len());
    for (control, observation) in data
        .noisy_controls
        .iter()
        .zip(data.noisy_observations.iter())
    {
        ckf.step(control, observation, DT);
        let state = ckf.estimate();
        positions.push((state[0], state[1]));
    }
    compute_rmse(&positions, &data.ground_truth)
}

fn compute_rmse(positions: &[(f64, f64)], ground_truth: &[Vector4<f64>]) -> f64 {
    let n = positions.len().min(ground_truth.len());
    if n == 0 {
        return f64::NAN;
    }
    let sum_sq: f64 = positions
        .iter()
        .zip(ground_truth.iter())
        .map(|((x, y), gt)| (x - gt[0]).powi(2) + (y - gt[1]).powi(2))
        .sum();
    (sum_sq / n as f64).sqrt()
}

fn variance<I>(values: I) -> f64
where
    I: Iterator<Item = f64>,
{
    let samples: Vec<f64> = values.collect();
    if samples.len() <= 1 {
        return 0.0;
    }
    let mean = samples.iter().sum::<f64>() / samples.len() as f64;
    samples
        .iter()
        .map(|value| (value - mean).powi(2))
        .sum::<f64>()
        / samples.len() as f64
}

fn median_value(samples: &[f64]) -> f64 {
    assert!(
        !samples.is_empty(),
        "median_value requires at least one sample"
    );
    let mut sorted = samples.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    sorted[sorted.len() / 2]
}

fn min_value(samples: &[f64]) -> f64 {
    samples.iter().copied().fold(f64::INFINITY, f64::min)
}

fn max_value(samples: &[f64]) -> f64 {
    samples.iter().copied().fold(f64::NEG_INFINITY, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_variant_selects_one_slot() {
        let variant = FirstScenarioAccuracyAggregation::new();
        assert_eq!(variant.selected_slots(10), vec![0]);
    }

    #[test]
    fn percentile_variant_maps_percentiles_to_unique_slots() {
        let variant = PercentileBucketAccuracyAggregation::new(vec![0.0, 0.25, 0.5, 0.75, 1.0]);
        assert_eq!(variant.selected_slots(10), vec![0, 2, 5, 7, 9]);
        assert_eq!(variant.selected_slots(1), vec![0]);
    }

    #[test]
    fn variance_variant_builds_adaptive_sampling_plan() {
        let variant = VarianceTriggeredAccuracyAggregation::new(vec![0, 4, 9], 0.10);
        let plan = variant.sampling_plan(10);
        assert_eq!(plan.initial_slots, vec![0, 4, 9]);
        assert_eq!(plan.escalation_slots, (0..10).collect::<Vec<_>>());
        assert!(plan.escalate_if_vote_split);
        assert_eq!(plan.escalate_if_ratio_margin_below, Some(0.10));
    }

    #[test]
    fn suite_runs_on_small_noise_problem() {
        let variants = default_accuracy_variants();
        let cases = vec![AccuracyExperimentCase {
            family_name: "test-case",
            seed_offset: 99_000,
            profile: MotionProfile {
                velocity: 1.0,
                yaw_rate: 0.10,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.20,
                control_noise_yaw_deg: 4.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.30,
                obs_noise_y: 0.30,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[60, 100],
        }];
        let reports = run_variant_suite(
            &variants,
            &cases,
            AccuracyEvaluationConfig {
                scenarios_per_bucket: 3,
            },
        );
        assert_eq!(reports.len(), 5);
        assert!(reports.iter().all(|report| report.observations.len() == 2));
    }

    #[test]
    fn sensor_rate_mismatch_reuses_observations_between_refreshes() {
        let case = AccuracyExperimentCase {
            family_name: "sensor-rate-test",
            seed_offset: 123_000,
            profile: MotionProfile {
                velocity: 1.0,
                yaw_rate: 0.12,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.0,
                control_noise_yaw_deg: 0.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.0,
                obs_noise_y: 0.0,
                observation_refresh_interval: 3,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[6],
        };

        let data = generate_sim_data(&case, 6, 0);
        assert_eq!(data.noisy_observations[0], data.noisy_observations[1]);
        assert_eq!(data.noisy_observations[1], data.noisy_observations[2]);
        assert_ne!(data.noisy_observations[2], data.noisy_observations[3]);
        assert_eq!(data.noisy_observations[3], data.noisy_observations[4]);
        assert_eq!(data.noisy_observations[4], data.noisy_observations[5]);
    }

    #[test]
    fn control_latency_uses_delayed_commanded_control_for_truth() {
        let profile = MotionProfile {
            velocity: 1.0,
            yaw_rate: 0.20,
            true_velocity_scale: 1.0,
            true_velocity_wave: 0.0,
            true_yaw_rate_scale: 1.0,
            true_yaw_wave_deg: 0.0,
            command_velocity_wave: 0.20,
            command_yaw_wave_deg: 6.0,
            control_latency_steps: 2,
            actuator_velocity_limit: 0.0,
            actuator_yaw_limit_deg: 0.0,
            process_noise_longitudinal: 0.0,
            process_noise_lateral: 0.0,
            process_noise_yaw_deg: 0.0,
            control_noise_v: 0.0,
            control_noise_yaw_deg: 0.0,
            control_bias_v: 0.0,
            control_bias_yaw_deg: 0.0,
            obs_noise_x: 0.0,
            obs_noise_y: 0.0,
            observation_refresh_interval: 1,
            observation_hold_probability: 0.0,
            observation_outlier_probability: 0.0,
            observation_outlier_scale: 0.0,
            observation_outlier_burst_len: 0,
            observation_bias_burst_interval: 0,
        };

        let delayed_command = commanded_control_for_step(profile, 1.0, 2, 0);
        let truth_control = true_control_for_step(profile, 1.0, 4, 0);

        assert!((truth_control[0] - delayed_command[0]).abs() < 1e-9);
        assert!((truth_control[1] - delayed_command[1]).abs() < 1e-9);
    }

    #[test]
    fn process_noise_anisotropy_changes_ground_truth_trajectory() {
        let base_case = AccuracyExperimentCase {
            family_name: "process-noise-test",
            seed_offset: 321_000,
            profile: MotionProfile {
                velocity: 1.0,
                yaw_rate: 0.18,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.0,
                control_noise_yaw_deg: 0.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.0,
                obs_noise_y: 0.0,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 0.0,
                observation_outlier_burst_len: 0,
                observation_bias_burst_interval: 0,
            },
            buckets: &[8],
        };
        let noisy_case = AccuracyExperimentCase {
            profile: MotionProfile {
                process_noise_longitudinal: 0.02,
                process_noise_lateral: 0.18,
                process_noise_yaw_deg: 4.0,
                ..base_case.profile
            },
            ..base_case
        };

        let clean = generate_sim_data(&base_case, 8, 0);
        let noisy = generate_sim_data(&noisy_case, 8, 0);

        assert_ne!(clean.ground_truth, noisy.ground_truth);
    }

    #[test]
    fn sensor_bias_burst_adds_deterministic_observation_offset() {
        let case = AccuracyExperimentCase {
            family_name: "sensor-bias-test",
            seed_offset: 456_000,
            profile: MotionProfile {
                velocity: 1.0,
                yaw_rate: 0.14,
                true_velocity_scale: 1.0,
                true_velocity_wave: 0.0,
                true_yaw_rate_scale: 1.0,
                true_yaw_wave_deg: 0.0,
                command_velocity_wave: 0.0,
                command_yaw_wave_deg: 0.0,
                control_latency_steps: 0,
                actuator_velocity_limit: 0.0,
                actuator_yaw_limit_deg: 0.0,
                process_noise_longitudinal: 0.0,
                process_noise_lateral: 0.0,
                process_noise_yaw_deg: 0.0,
                control_noise_v: 0.0,
                control_noise_yaw_deg: 0.0,
                control_bias_v: 0.0,
                control_bias_yaw_deg: 0.0,
                obs_noise_x: 0.0,
                obs_noise_y: 0.0,
                observation_refresh_interval: 1,
                observation_hold_probability: 0.0,
                observation_outlier_probability: 0.0,
                observation_outlier_scale: 1.0,
                observation_outlier_burst_len: 2,
                observation_bias_burst_interval: 4,
            },
            buckets: &[4],
        };

        let data = generate_sim_data(&case, 4, 0);
        let offset0 = data.noisy_observations[0]
            - Vector2::new(data.ground_truth[0][0], data.ground_truth[0][1]);
        let offset1 = data.noisy_observations[1]
            - Vector2::new(data.ground_truth[1][0], data.ground_truth[1][1]);
        let offset2 = data.noisy_observations[2]
            - Vector2::new(data.ground_truth[2][0], data.ground_truth[2][1]);
        assert!((offset0[0] - 0.04).abs() < 1e-9 && (offset0[1] - 0.024).abs() < 1e-9);
        assert!((offset1[0] - 0.04).abs() < 1e-9 && (offset1[1] - 0.024).abs() < 1e-9);
        assert!(offset2.norm() < 1e-12);
    }

    #[test]
    fn actuator_saturation_caps_truth_control() {
        let profile = MotionProfile {
            velocity: 1.0,
            yaw_rate: 0.24,
            true_velocity_scale: 1.0,
            true_velocity_wave: 0.0,
            true_yaw_rate_scale: 1.0,
            true_yaw_wave_deg: 0.0,
            command_velocity_wave: 0.50,
            command_yaw_wave_deg: 14.0,
            control_latency_steps: 0,
            actuator_velocity_limit: 1.05,
            actuator_yaw_limit_deg: 9.0,
            process_noise_longitudinal: 0.0,
            process_noise_lateral: 0.0,
            process_noise_yaw_deg: 0.0,
            control_noise_v: 0.0,
            control_noise_yaw_deg: 0.0,
            control_bias_v: 0.0,
            control_bias_yaw_deg: 0.0,
            obs_noise_x: 0.0,
            obs_noise_y: 0.0,
            observation_refresh_interval: 1,
            observation_hold_probability: 0.0,
            observation_outlier_probability: 0.0,
            observation_outlier_scale: 0.0,
            observation_outlier_burst_len: 0,
            observation_bias_burst_interval: 0,
        };

        let unsaturated = commanded_control_for_step(profile, 1.0, 7, 0);
        let truth = true_control_for_step(profile, 1.0, 7, 0);

        assert!(unsaturated[0] > profile.actuator_velocity_limit);
        assert!(unsaturated[1].abs() > profile.actuator_yaw_limit_deg.to_radians());
        assert!((truth[0] - profile.actuator_velocity_limit).abs() < 1e-9);
        assert!((truth[1].abs() - profile.actuator_yaw_limit_deg.to_radians()).abs() < 1e-9);
    }
}
