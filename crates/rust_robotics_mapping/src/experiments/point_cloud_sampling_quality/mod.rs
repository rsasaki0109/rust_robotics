mod first_scenario;
mod full_bucket;
mod percentile_bucket;
mod sampled_bucket;
mod variance_triggered;

use std::collections::HashMap;
use std::f64::consts::PI;
use std::time::Instant;

use nalgebra::DVector;
use rand::prelude::IndexedRandom;
use rand::rngs::StdRng;
use rand::seq::SliceRandom;
use rand::{Rng, SeedableRng};
use rust_robotics_core::{
    annotate_against_reference as annotate_reference_reports, average_coverage_ratio,
    read_source_metrics, ExperimentObservation, ExperimentSamplingPlan, ExperimentVariantReport,
    ExtensibilityMetrics, VariantDescriptor,
};

use crate::point_cloud_sampling::{
    farthest_point_sampling, poisson_disk_sampling, voxel_point_sampling,
};

pub use first_scenario::FirstScenarioPointSamplingAggregation;
pub use full_bucket::FullBucketPointSamplingAggregation;
pub use percentile_bucket::PercentileBucketPointSamplingAggregation;
pub use sampled_bucket::SampledBucketPointSamplingAggregation;
pub use variance_triggered::VarianceTriggeredPointSamplingAggregation;

pub type PointSamplingSamplingPlan = ExperimentSamplingPlan;

pub trait PointSamplingAggregationVariant {
    fn descriptor(&self) -> VariantDescriptor;
    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize>;

    fn sampling_plan(&self, total_scenarios: usize) -> PointSamplingSamplingPlan {
        PointSamplingSamplingPlan::static_slots(self.selected_slots(total_scenarios))
    }
}

#[derive(Debug, Clone, Copy)]
enum CloudFamilyKind {
    LayeredPlane,
    TwinClusterBridge,
    RingWithOutliers,
}

#[derive(Debug, Clone, Copy)]
enum CloudCorruptionMode {
    None,
    SlabDrop {
        axis: usize,
        center: f64,
        half_width: f64,
        drop_probability: f64,
    },
    DensityShift {
        axis: usize,
        center: f64,
        half_width: f64,
        dense_duplicate_probability: f64,
        sparse_keep_probability: f64,
    },
    SectorDrop {
        start_angle: f64,
        end_angle: f64,
        drop_probability: f64,
    },
}

#[derive(Debug, Clone)]
pub struct PointSamplingExperimentCase {
    pub family_name: &'static str,
    pub buckets: Vec<u32>,
    base_point_count: usize,
    cloud_kind: CloudFamilyKind,
    base_jitter_scale: f64,
    base_outlier_fraction: f64,
    extra_noise_scales: [f64; 3],
    corruption_mode: CloudCorruptionMode,
    burst_slots: &'static [usize],
    burst_noise_scales: [f64; 3],
    burst_outlier_multiplier: f64,
}

#[derive(Debug, Clone)]
pub struct PointSamplingObservation {
    pub family_name: &'static str,
    pub bucket: u32,
    pub total_scenarios: usize,
    pub initial_slots: Vec<usize>,
    pub selected_slots: Vec<usize>,
    pub escalated: bool,
    pub voxel_bucket_median_score: f64,
    pub farthest_bucket_median_score: f64,
    pub poisson_bucket_median_score: f64,
    pub voxel_min_score: f64,
    pub voxel_max_score: f64,
    pub farthest_min_score: f64,
    pub farthest_max_score: f64,
    pub poisson_min_score: f64,
    pub poisson_max_score: f64,
    pub voxel_wins: usize,
    pub farthest_wins: usize,
    pub poisson_wins: usize,
}

impl PointSamplingObservation {
    pub fn winner(&self) -> &'static str {
        let candidates = [
            ("Voxel", self.voxel_bucket_median_score),
            ("FarthestPoint", self.farthest_bucket_median_score),
            ("PoissonDisk", self.poisson_bucket_median_score),
        ];
        candidates
            .into_iter()
            .min_by(|lhs, rhs| lhs.1.total_cmp(&rhs.1))
            .map(|entry| entry.0)
            .unwrap_or("Voxel")
    }

    pub fn runner_up_over_best(&self) -> f64 {
        let mut scores = [
            self.voxel_bucket_median_score,
            self.farthest_bucket_median_score,
            self.poisson_bucket_median_score,
        ];
        scores.sort_by(f64::total_cmp);
        scores[1] / scores[0].max(1e-9)
    }

    pub fn coverage_ratio(&self) -> f64 {
        self.selected_slots.len() as f64 / self.total_scenarios as f64
    }
}

impl ExperimentObservation for PointSamplingObservation {
    type Key = (&'static str, u32);

    fn comparison_key(&self) -> Self::Key {
        (self.family_name, self.bucket)
    }

    fn winner_label(&self) -> &'static str {
        PointSamplingObservation::winner(self)
    }

    fn ratio_value(&self) -> f64 {
        PointSamplingObservation::runner_up_over_best(self)
    }

    fn coverage_ratio(&self) -> f64 {
        PointSamplingObservation::coverage_ratio(self)
    }
}

pub type PointSamplingVariantReport = ExperimentVariantReport<PointSamplingObservation>;

#[derive(Debug, Clone, Copy)]
pub struct PointSamplingEvaluationConfig {
    pub scenarios_per_bucket: usize,
}

impl Default for PointSamplingEvaluationConfig {
    fn default() -> Self {
        Self {
            scenarios_per_bucket: 10,
        }
    }
}

pub fn mapping_point_cloud_sampling_process_problem() -> Vec<PointSamplingExperimentCase> {
    vec![
        PointSamplingExperimentCase {
            family_name: "layered-plane",
            buckets: vec![48, 96, 144],
            base_point_count: 288,
            cloud_kind: CloudFamilyKind::LayeredPlane,
            base_jitter_scale: 0.05,
            base_outlier_fraction: 0.02,
            extra_noise_scales: [0.0, 0.0, 0.0],
            corruption_mode: CloudCorruptionMode::None,
            burst_slots: &[],
            burst_noise_scales: [0.0, 0.0, 0.0],
            burst_outlier_multiplier: 1.0,
        },
        PointSamplingExperimentCase {
            family_name: "twin-cluster-bridge",
            buckets: vec![60, 120, 180],
            base_point_count: 360,
            cloud_kind: CloudFamilyKind::TwinClusterBridge,
            base_jitter_scale: 0.07,
            base_outlier_fraction: 0.04,
            extra_noise_scales: [0.0, 0.0, 0.0],
            corruption_mode: CloudCorruptionMode::None,
            burst_slots: &[],
            burst_noise_scales: [0.0, 0.0, 0.0],
            burst_outlier_multiplier: 1.0,
        },
        PointSamplingExperimentCase {
            family_name: "ring-with-outliers",
            buckets: vec![72, 132, 192],
            base_point_count: 420,
            cloud_kind: CloudFamilyKind::RingWithOutliers,
            base_jitter_scale: 0.09,
            base_outlier_fraction: 0.07,
            extra_noise_scales: [0.0, 0.0, 0.0],
            corruption_mode: CloudCorruptionMode::None,
            burst_slots: &[],
            burst_noise_scales: [0.0, 0.0, 0.0],
            burst_outlier_multiplier: 1.0,
        },
    ]
}

pub fn mapping_occlusion_corruption_process_problem() -> Vec<PointSamplingExperimentCase> {
    vec![
        PointSamplingExperimentCase {
            family_name: "slab-occluded-plane",
            buckets: vec![48, 96, 144],
            base_point_count: 288,
            cloud_kind: CloudFamilyKind::LayeredPlane,
            base_jitter_scale: 0.05,
            base_outlier_fraction: 0.02,
            extra_noise_scales: [0.2, 1.5, 0.4],
            corruption_mode: CloudCorruptionMode::SlabDrop {
                axis: 0,
                center: 0.0,
                half_width: 2.4,
                drop_probability: 0.82,
            },
            burst_slots: &[],
            burst_noise_scales: [0.0, 0.0, 0.0],
            burst_outlier_multiplier: 1.0,
        },
        PointSamplingExperimentCase {
            family_name: "bridge-shadow-clusters",
            buckets: vec![60, 120, 180],
            base_point_count: 360,
            cloud_kind: CloudFamilyKind::TwinClusterBridge,
            base_jitter_scale: 0.07,
            base_outlier_fraction: 0.04,
            extra_noise_scales: [0.4, 1.0, 1.8],
            corruption_mode: CloudCorruptionMode::SlabDrop {
                axis: 1,
                center: 0.0,
                half_width: 1.8,
                drop_probability: 0.76,
            },
            burst_slots: &[],
            burst_noise_scales: [0.0, 0.0, 0.0],
            burst_outlier_multiplier: 1.0,
        },
        PointSamplingExperimentCase {
            family_name: "sector-dropout-ring",
            buckets: vec![72, 132, 192],
            base_point_count: 420,
            cloud_kind: CloudFamilyKind::RingWithOutliers,
            base_jitter_scale: 0.09,
            base_outlier_fraction: 0.05,
            extra_noise_scales: [1.1, 0.6, 0.9],
            corruption_mode: CloudCorruptionMode::SectorDrop {
                start_angle: 0.20 * PI,
                end_angle: 0.62 * PI,
                drop_probability: 0.88,
            },
            burst_slots: &[],
            burst_noise_scales: [0.0, 0.0, 0.0],
            burst_outlier_multiplier: 1.0,
        },
    ]
}

pub fn mapping_density_shift_process_problem() -> Vec<PointSamplingExperimentCase> {
    vec![
        PointSamplingExperimentCase {
            family_name: "density-striped-plane",
            buckets: vec![48, 96, 144],
            base_point_count: 288,
            cloud_kind: CloudFamilyKind::LayeredPlane,
            base_jitter_scale: 0.05,
            base_outlier_fraction: 0.02,
            extra_noise_scales: [1.6, 0.3, 0.6],
            corruption_mode: CloudCorruptionMode::DensityShift {
                axis: 1,
                center: 0.0,
                half_width: 3.2,
                dense_duplicate_probability: 0.85,
                sparse_keep_probability: 0.58,
            },
            burst_slots: &[],
            burst_noise_scales: [0.0, 0.0, 0.0],
            burst_outlier_multiplier: 1.0,
        },
        PointSamplingExperimentCase {
            family_name: "left-heavy-bridge",
            buckets: vec![60, 120, 180],
            base_point_count: 360,
            cloud_kind: CloudFamilyKind::TwinClusterBridge,
            base_jitter_scale: 0.07,
            base_outlier_fraction: 0.03,
            extra_noise_scales: [0.5, 1.4, 0.7],
            corruption_mode: CloudCorruptionMode::DensityShift {
                axis: 0,
                center: -4.5,
                half_width: 4.0,
                dense_duplicate_probability: 0.90,
                sparse_keep_probability: 0.62,
            },
            burst_slots: &[],
            burst_noise_scales: [0.0, 0.0, 0.0],
            burst_outlier_multiplier: 1.0,
        },
        PointSamplingExperimentCase {
            family_name: "core-heavy-ring",
            buckets: vec![72, 132, 192],
            base_point_count: 420,
            cloud_kind: CloudFamilyKind::RingWithOutliers,
            base_jitter_scale: 0.09,
            base_outlier_fraction: 0.04,
            extra_noise_scales: [0.8, 0.8, 1.2],
            corruption_mode: CloudCorruptionMode::DensityShift {
                axis: 0,
                center: 0.0,
                half_width: 5.5,
                dense_duplicate_probability: 0.95,
                sparse_keep_probability: 0.54,
            },
            burst_slots: &[],
            burst_noise_scales: [0.0, 0.0, 0.0],
            burst_outlier_multiplier: 1.0,
        },
    ]
}

pub fn mapping_anisotropic_noise_process_problem() -> Vec<PointSamplingExperimentCase> {
    vec![
        PointSamplingExperimentCase {
            family_name: "sheared-plane-noise",
            buckets: vec![48, 96, 144],
            base_point_count: 288,
            cloud_kind: CloudFamilyKind::LayeredPlane,
            base_jitter_scale: 0.05,
            base_outlier_fraction: 0.02,
            extra_noise_scales: [0.3, 2.2, 0.8],
            corruption_mode: CloudCorruptionMode::None,
            burst_slots: &[],
            burst_noise_scales: [0.0, 0.0, 0.0],
            burst_outlier_multiplier: 1.0,
        },
        PointSamplingExperimentCase {
            family_name: "bridge-z-noise",
            buckets: vec![60, 120, 180],
            base_point_count: 360,
            cloud_kind: CloudFamilyKind::TwinClusterBridge,
            base_jitter_scale: 0.07,
            base_outlier_fraction: 0.04,
            extra_noise_scales: [0.4, 0.7, 2.4],
            corruption_mode: CloudCorruptionMode::None,
            burst_slots: &[],
            burst_noise_scales: [0.0, 0.0, 0.0],
            burst_outlier_multiplier: 1.0,
        },
        PointSamplingExperimentCase {
            family_name: "radial-ring-noise",
            buckets: vec![72, 132, 192],
            base_point_count: 420,
            cloud_kind: CloudFamilyKind::RingWithOutliers,
            base_jitter_scale: 0.09,
            base_outlier_fraction: 0.06,
            extra_noise_scales: [1.8, 1.8, 0.5],
            corruption_mode: CloudCorruptionMode::None,
            burst_slots: &[],
            burst_noise_scales: [0.0, 0.0, 0.0],
            burst_outlier_multiplier: 1.0,
        },
    ]
}

pub fn mapping_sparse_outlier_burst_process_problem() -> Vec<PointSamplingExperimentCase> {
    vec![
        PointSamplingExperimentCase {
            family_name: "plane-burst-outliers",
            buckets: vec![48, 96, 144],
            base_point_count: 288,
            cloud_kind: CloudFamilyKind::LayeredPlane,
            base_jitter_scale: 0.05,
            base_outlier_fraction: 0.015,
            extra_noise_scales: [0.2, 0.2, 0.1],
            corruption_mode: CloudCorruptionMode::None,
            burst_slots: &[4, 5, 7],
            burst_noise_scales: [0.6, 1.0, 2.0],
            burst_outlier_multiplier: 6.0,
        },
        PointSamplingExperimentCase {
            family_name: "bridge-burst-outliers",
            buckets: vec![60, 120, 180],
            base_point_count: 360,
            cloud_kind: CloudFamilyKind::TwinClusterBridge,
            base_jitter_scale: 0.07,
            base_outlier_fraction: 0.02,
            extra_noise_scales: [0.2, 0.3, 0.2],
            corruption_mode: CloudCorruptionMode::None,
            burst_slots: &[4, 5, 7],
            burst_noise_scales: [0.4, 1.8, 1.2],
            burst_outlier_multiplier: 6.5,
        },
        PointSamplingExperimentCase {
            family_name: "ring-burst-outliers",
            buckets: vec![72, 132, 192],
            base_point_count: 420,
            cloud_kind: CloudFamilyKind::RingWithOutliers,
            base_jitter_scale: 0.09,
            base_outlier_fraction: 0.025,
            extra_noise_scales: [0.2, 0.2, 0.2],
            corruption_mode: CloudCorruptionMode::None,
            burst_slots: &[4, 5, 7],
            burst_noise_scales: [1.6, 0.7, 0.9],
            burst_outlier_multiplier: 7.0,
        },
    ]
}

pub fn mapping_resolution_ladder_process_problem() -> Vec<PointSamplingExperimentCase> {
    vec![
        PointSamplingExperimentCase {
            family_name: "plane-resolution-ladder",
            buckets: vec![48, 96, 144],
            base_point_count: 288,
            cloud_kind: CloudFamilyKind::LayeredPlane,
            base_jitter_scale: 0.045,
            base_outlier_fraction: 0.01,
            extra_noise_scales: [0.18, 0.25, 0.15],
            corruption_mode: CloudCorruptionMode::DensityShift {
                axis: 1,
                center: 0.0,
                half_width: 4.2,
                dense_duplicate_probability: 0.52,
                sparse_keep_probability: 0.84,
            },
            burst_slots: &[1, 2, 3, 5, 7, 8],
            burst_noise_scales: [0.35, 0.55, 0.95],
            burst_outlier_multiplier: 2.4,
        },
        PointSamplingExperimentCase {
            family_name: "bridge-resolution-ladder",
            buckets: vec![60, 120, 180],
            base_point_count: 360,
            cloud_kind: CloudFamilyKind::TwinClusterBridge,
            base_jitter_scale: 0.06,
            base_outlier_fraction: 0.015,
            extra_noise_scales: [0.22, 0.35, 0.30],
            corruption_mode: CloudCorruptionMode::DensityShift {
                axis: 0,
                center: 0.0,
                half_width: 3.8,
                dense_duplicate_probability: 0.58,
                sparse_keep_probability: 0.80,
            },
            burst_slots: &[1, 2, 3, 5, 7, 8],
            burst_noise_scales: [0.28, 0.90, 0.75],
            burst_outlier_multiplier: 2.8,
        },
        PointSamplingExperimentCase {
            family_name: "ring-resolution-ladder",
            buckets: vec![72, 132, 192],
            base_point_count: 420,
            cloud_kind: CloudFamilyKind::RingWithOutliers,
            base_jitter_scale: 0.08,
            base_outlier_fraction: 0.02,
            extra_noise_scales: [0.30, 0.30, 0.20],
            corruption_mode: CloudCorruptionMode::DensityShift {
                axis: 0,
                center: 0.0,
                half_width: 5.8,
                dense_duplicate_probability: 0.60,
                sparse_keep_probability: 0.76,
            },
            burst_slots: &[1, 2, 3, 5, 7, 8],
            burst_noise_scales: [0.95, 0.48, 0.42],
            burst_outlier_multiplier: 3.0,
        },
    ]
}

pub fn default_point_sampling_variants() -> Vec<Box<dyn PointSamplingAggregationVariant>> {
    vec![
        Box::new(FirstScenarioPointSamplingAggregation::new()),
        Box::new(SampledBucketPointSamplingAggregation::new(vec![0, 4, 9])),
        Box::new(PercentileBucketPointSamplingAggregation::new(vec![
            0.0, 0.25, 0.5, 0.75, 1.0,
        ])),
        Box::new(VarianceTriggeredPointSamplingAggregation::new(
            vec![0, 4, 9],
            0.08,
        )),
        Box::new(FullBucketPointSamplingAggregation::new()),
    ]
}

pub fn run_variant_suite(
    variants: &[Box<dyn PointSamplingAggregationVariant>],
    cases: &[PointSamplingExperimentCase],
    config: PointSamplingEvaluationConfig,
) -> Vec<PointSamplingVariantReport> {
    let slot_cache = build_slot_sample_cache(cases, config.scenarios_per_bucket);
    let mut reports = Vec::with_capacity(variants.len());

    for variant in variants {
        let started = Instant::now();
        let mut observations = Vec::new();
        for case in cases {
            for &bucket in &case.buckets {
                observations.push(measure_bucket_observation(
                    &**variant,
                    case,
                    bucket,
                    config.scenarios_per_bucket,
                    &slot_cache,
                ));
            }
        }

        let descriptor = variant.descriptor();
        let source_metrics = read_source_metrics(std::path::Path::new(descriptor.source_path))
            .expect("experiment source metrics should be readable");
        let extensibility_metrics = ExtensibilityMetrics {
            average_coverage_ratio: average_coverage_ratio(&observations),
            knob_count: descriptor.knob_count,
            reports_dispersion: descriptor.reports_dispersion,
        };

        reports.push(PointSamplingVariantReport {
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
    variant: &dyn PointSamplingAggregationVariant,
    case: &PointSamplingExperimentCase,
    bucket: u32,
    total_scenarios: usize,
    slot_cache: &HashMap<(&'static str, u32, usize), SlotPointSamplingSample>,
) -> PointSamplingObservation {
    let plan = variant.sampling_plan(total_scenarios);
    let initial_slots = normalize_slots(total_scenarios, &plan.initial_slots);
    assert!(
        !initial_slots.is_empty(),
        "{} bucket {} should select at least one scenario",
        case.family_name,
        bucket
    );

    let mut slot_samples = measure_slot_samples(&initial_slots, case, bucket, slot_cache);
    let mut selected_slots = initial_slots.clone();
    let mut escalated = false;
    let escalation_slots = normalize_slots(total_scenarios, &plan.escalation_slots);

    if should_escalate(&slot_samples, &plan) {
        let additional_slots = escalation_slots
            .into_iter()
            .filter(|slot| !selected_slots.contains(slot))
            .collect::<Vec<_>>();
        if !additional_slots.is_empty() {
            slot_samples.extend(measure_slot_samples(
                &additional_slots,
                case,
                bucket,
                slot_cache,
            ));
            selected_slots.extend(additional_slots);
            selected_slots.sort_unstable();
            escalated = true;
        }
    }

    let voxel_scores = slot_samples
        .iter()
        .map(|sample| sample.voxel_score)
        .collect::<Vec<_>>();
    let farthest_scores = slot_samples
        .iter()
        .map(|sample| sample.farthest_score)
        .collect::<Vec<_>>();
    let poisson_scores = slot_samples
        .iter()
        .map(|sample| sample.poisson_score)
        .collect::<Vec<_>>();

    PointSamplingObservation {
        family_name: case.family_name,
        bucket,
        total_scenarios,
        initial_slots,
        selected_slots,
        escalated,
        voxel_bucket_median_score: median_value(&voxel_scores),
        farthest_bucket_median_score: median_value(&farthest_scores),
        poisson_bucket_median_score: median_value(&poisson_scores),
        voxel_min_score: min_value(&voxel_scores),
        voxel_max_score: max_value(&voxel_scores),
        farthest_min_score: min_value(&farthest_scores),
        farthest_max_score: max_value(&farthest_scores),
        poisson_min_score: min_value(&poisson_scores),
        poisson_max_score: max_value(&poisson_scores),
        voxel_wins: slot_samples
            .iter()
            .filter(|sample| sample.winner() == "Voxel")
            .count(),
        farthest_wins: slot_samples
            .iter()
            .filter(|sample| sample.winner() == "FarthestPoint")
            .count(),
        poisson_wins: slot_samples
            .iter()
            .filter(|sample| sample.winner() == "PoissonDisk")
            .count(),
    }
}

#[derive(Debug, Clone, Copy)]
struct SlotPointSamplingSample {
    voxel_score: f64,
    farthest_score: f64,
    poisson_score: f64,
}

impl SlotPointSamplingSample {
    fn winner(&self) -> &'static str {
        [
            ("Voxel", self.voxel_score),
            ("FarthestPoint", self.farthest_score),
            ("PoissonDisk", self.poisson_score),
        ]
        .into_iter()
        .min_by(|lhs, rhs| lhs.1.total_cmp(&rhs.1))
        .map(|entry| entry.0)
        .unwrap_or("Voxel")
    }

    fn runner_up_over_best(&self) -> f64 {
        let mut scores = [self.voxel_score, self.farthest_score, self.poisson_score];
        scores.sort_by(f64::total_cmp);
        scores[1] / scores[0].max(1e-9)
    }
}

fn measure_slot_samples(
    slots: &[usize],
    case: &PointSamplingExperimentCase,
    bucket: u32,
    slot_cache: &HashMap<(&'static str, u32, usize), SlotPointSamplingSample>,
) -> Vec<SlotPointSamplingSample> {
    slots
        .iter()
        .map(|slot| {
            *slot_cache
                .get(&(case.family_name, bucket, *slot))
                .expect("slot sample should exist")
        })
        .collect()
}

fn should_escalate(
    slot_samples: &[SlotPointSamplingSample],
    plan: &PointSamplingSamplingPlan,
) -> bool {
    if slot_samples.is_empty() || plan.escalation_slots.is_empty() {
        return false;
    }

    let vote_split = {
        let first_winner = slot_samples[0].winner();
        slot_samples
            .iter()
            .skip(1)
            .any(|sample| sample.winner() != first_winner)
    };
    let ratio_close = plan
        .escalate_if_ratio_margin_below
        .map(|threshold| {
            let ratios = slot_samples
                .iter()
                .map(SlotPointSamplingSample::runner_up_over_best)
                .collect::<Vec<_>>();
            (median_value(&ratios) - 1.0).abs() < threshold
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

fn generate_point_cloud(
    case: &PointSamplingExperimentCase,
    bucket: u32,
    slot: usize,
) -> Vec<DVector<f64>> {
    let seed = family_seed(case.family_name) ^ ((bucket as u64) << 16) ^ slot as u64;
    let mut rng = StdRng::seed_from_u64(seed);
    let burst_active = is_burst_slot(case, slot);
    let jitter_scale = case.base_jitter_scale
        * (1.0 + bucket as f64 / 260.0)
        * (1.0 + slot as f64 / 40.0)
        * if burst_active {
            1.10 + bucket as f64 / 900.0
        } else {
            1.0
        };
    let outlier_count = ((case.base_point_count as f64)
        * case.base_outlier_fraction
        * (0.6 + bucket as f64 / 300.0))
        .round() as usize;

    let mut points = match case.cloud_kind {
        CloudFamilyKind::LayeredPlane => {
            generate_layered_plane_points(case.base_point_count, jitter_scale, slot, &mut rng)
        }
        CloudFamilyKind::TwinClusterBridge => {
            generate_twin_cluster_points(case.base_point_count, jitter_scale, slot, &mut rng)
        }
        CloudFamilyKind::RingWithOutliers => {
            generate_ring_points(case.base_point_count, jitter_scale, slot, &mut rng)
        }
    };

    apply_extra_noise(&mut points, case, bucket, slot, &mut rng);
    apply_structured_corruption(&mut points, case, bucket, &mut rng);
    append_outliers(&mut points, outlier_count, &mut rng);
    append_burst_outliers(&mut points, case, bucket, slot, outlier_count, &mut rng);
    points
}

fn build_slot_sample_cache(
    cases: &[PointSamplingExperimentCase],
    total_scenarios: usize,
) -> HashMap<(&'static str, u32, usize), SlotPointSamplingSample> {
    let mut cache = HashMap::new();
    for case in cases {
        for &bucket in &case.buckets {
            for slot in 0..total_scenarios {
                let cloud = generate_point_cloud(case, bucket, slot);
                let target_count = bucket as usize;
                let voxel_score = evaluate_voxel_quality(&cloud, target_count);
                let farthest_score = evaluate_farthest_quality(&cloud, target_count, slot as u64);
                let poisson_score = evaluate_poisson_quality(&cloud, target_count, slot as u64);
                assert!(voxel_score.is_finite(), "Voxel score must stay finite");
                assert!(
                    farthest_score.is_finite(),
                    "Farthest-point score must stay finite"
                );
                assert!(poisson_score.is_finite(), "Poisson score must stay finite");
                cache.insert(
                    (case.family_name, bucket, slot),
                    SlotPointSamplingSample {
                        voxel_score,
                        farthest_score,
                        poisson_score,
                    },
                );
            }
        }
    }
    cache
}

fn generate_layered_plane_points(
    base_point_count: usize,
    jitter_scale: f64,
    slot: usize,
    rng: &mut StdRng,
) -> Vec<DVector<f64>> {
    let side = ((base_point_count / 2) as f64).sqrt().round() as usize;
    let mut points = Vec::with_capacity(side * side * 2);
    let phase = slot as f64 * 0.17;
    for layer in 0..2 {
        for ix in 0..side {
            for iy in 0..side {
                let x = -12.0 + 24.0 * ix as f64 / (side.saturating_sub(1).max(1) as f64);
                let y = -12.0 + 24.0 * iy as f64 / (side.saturating_sub(1).max(1) as f64);
                let z = layer as f64 * 1.8 + 0.25 * (0.22 * x + phase).sin();
                points.push(jittered_point([x, y, z], jitter_scale, rng));
            }
        }
    }
    points
}

fn generate_twin_cluster_points(
    base_point_count: usize,
    jitter_scale: f64,
    slot: usize,
    rng: &mut StdRng,
) -> Vec<DVector<f64>> {
    let bridge_count = base_point_count / 6;
    let cluster_count = (base_point_count - bridge_count) / 2;
    let mut points = Vec::with_capacity(base_point_count);
    let phase = slot as f64 * 0.11;

    for _ in 0..cluster_count {
        points.push(jittered_point(
            [
                -7.0 + rng.gen_range(-2.4..2.4),
                -2.0 + rng.gen_range(-2.6..2.6),
                1.0 + rng.gen_range(-1.5..1.5),
            ],
            jitter_scale,
            rng,
        ));
    }
    for _ in 0..cluster_count {
        points.push(jittered_point(
            [
                7.0 + rng.gen_range(-2.2..2.2),
                2.0 + rng.gen_range(-2.3..2.3),
                -0.8 + rng.gen_range(-1.3..1.3),
            ],
            jitter_scale,
            rng,
        ));
    }
    for index in 0..bridge_count {
        let t = if bridge_count <= 1 {
            0.0
        } else {
            index as f64 / (bridge_count - 1) as f64
        };
        let x = -4.5 + 9.0 * t;
        let y = 1.4 * (2.0 * PI * t + phase).sin();
        let z = 0.9 * (PI * t + phase * 0.5).cos();
        points.push(jittered_point([x, y, z], jitter_scale, rng));
    }
    points
}

fn generate_ring_points(
    base_point_count: usize,
    jitter_scale: f64,
    slot: usize,
    rng: &mut StdRng,
) -> Vec<DVector<f64>> {
    let ring_count = base_point_count * 3 / 4;
    let core_count = base_point_count - ring_count;
    let mut points = Vec::with_capacity(base_point_count);
    let phase = slot as f64 * 0.09;

    for index in 0..ring_count {
        let t = index as f64 / ring_count.max(1) as f64;
        let theta = 2.0 * PI * t;
        let radius = 8.0 + 1.2 * (5.0 * theta + phase).sin();
        let x = radius * theta.cos();
        let y = radius * theta.sin();
        let z = 1.5 * (theta * 0.5 + phase).cos();
        points.push(jittered_point([x, y, z], jitter_scale, rng));
    }
    for _ in 0..core_count {
        let theta = rng.gen_range(0.0..2.0 * PI);
        let radius = rng.gen_range(0.0..3.0);
        let x = radius * theta.cos();
        let y = radius * theta.sin();
        let z = rng.gen_range(-1.5..1.5);
        points.push(jittered_point([x, y, z], jitter_scale, rng));
    }
    points
}

fn append_outliers(points: &mut Vec<DVector<f64>>, outlier_count: usize, rng: &mut StdRng) {
    for _ in 0..outlier_count {
        points.push(DVector::from_vec(vec![
            rng.gen_range(-18.0..18.0),
            rng.gen_range(-18.0..18.0),
            rng.gen_range(-6.0..6.0),
        ]));
    }
}

fn apply_extra_noise(
    points: &mut [DVector<f64>],
    case: &PointSamplingExperimentCase,
    bucket: u32,
    slot: usize,
    rng: &mut StdRng,
) {
    let noise_scales = effective_noise_scales(case, slot);
    if noise_scales == [0.0, 0.0, 0.0] {
        return;
    }

    let severity = 0.35 + bucket as f64 / 220.0;
    for point in points {
        for dim in 0..3 {
            let amplitude = case.base_jitter_scale * noise_scales[dim] * severity;
            if amplitude > 0.0 {
                point[dim] += rng.gen_range(-amplitude..amplitude);
            }
        }
    }
}

fn effective_noise_scales(case: &PointSamplingExperimentCase, slot: usize) -> [f64; 3] {
    if !is_burst_slot(case, slot) {
        return case.extra_noise_scales;
    }

    [
        case.extra_noise_scales[0] + case.burst_noise_scales[0],
        case.extra_noise_scales[1] + case.burst_noise_scales[1],
        case.extra_noise_scales[2] + case.burst_noise_scales[2],
    ]
}

fn is_burst_slot(case: &PointSamplingExperimentCase, slot: usize) -> bool {
    case.burst_slots.contains(&slot)
}

fn append_burst_outliers(
    points: &mut Vec<DVector<f64>>,
    case: &PointSamplingExperimentCase,
    bucket: u32,
    slot: usize,
    base_outlier_count: usize,
    rng: &mut StdRng,
) {
    if !is_burst_slot(case, slot) || case.burst_outlier_multiplier <= 1.0 {
        return;
    }

    let extra_count = ((base_outlier_count as f64) * (case.burst_outlier_multiplier - 1.0))
        .round()
        .max(6.0) as usize;
    let severity = 0.9 + bucket as f64 / 180.0;
    let centers = burst_cluster_centers(case.cloud_kind);
    for index in 0..extra_count {
        let center = centers[index % centers.len()];
        points.push(DVector::from_vec(vec![
            center[0] + rng.gen_range(-1.4 * severity..1.4 * severity),
            center[1] + rng.gen_range(-1.2 * severity..1.2 * severity),
            center[2] + rng.gen_range(-severity..severity),
        ]));
    }
}

fn burst_cluster_centers(cloud_kind: CloudFamilyKind) -> [[f64; 3]; 3] {
    match cloud_kind {
        CloudFamilyKind::LayeredPlane => {
            [[16.0, -13.0, 4.5], [-15.0, 14.0, -3.8], [13.5, 15.5, 3.2]]
        }
        CloudFamilyKind::TwinClusterBridge => {
            [[0.0, 13.5, 7.0], [14.0, -13.5, -6.0], [-13.0, 12.5, 6.2]]
        }
        CloudFamilyKind::RingWithOutliers => {
            [[0.0, 0.0, 12.5], [15.5, 0.0, -10.5], [-15.5, 1.5, 10.5]]
        }
    }
}

fn apply_structured_corruption(
    points: &mut Vec<DVector<f64>>,
    case: &PointSamplingExperimentCase,
    bucket: u32,
    rng: &mut StdRng,
) {
    let severity_scale = (0.75 + bucket as f64 / 260.0).clamp(0.0, 1.0);
    match case.corruption_mode {
        CloudCorruptionMode::None => {}
        CloudCorruptionMode::SlabDrop {
            axis,
            center,
            half_width,
            drop_probability,
        } => {
            points.retain(|point| {
                let in_slab = (point[axis] - center).abs() <= half_width;
                !(in_slab && rng.gen_bool((drop_probability * severity_scale).clamp(0.0, 0.98)))
            });
        }
        CloudCorruptionMode::DensityShift {
            axis,
            center,
            half_width,
            dense_duplicate_probability,
            sparse_keep_probability,
        } => {
            let target_len = points.len();
            let mut shifted = Vec::with_capacity(target_len * 2);
            for point in points.iter() {
                let favored = (point[axis] - center).abs() <= half_width;
                if favored {
                    shifted.push(point.clone());
                    if rng.gen_bool((dense_duplicate_probability * severity_scale).clamp(0.0, 0.98))
                    {
                        let mut duplicate = point.clone();
                        let amplitude = case.base_jitter_scale * (0.35 + bucket as f64 / 480.0);
                        for dim in 0..duplicate.len() {
                            let axis_scale = if dim == axis { 1.0 } else { 0.35 };
                            duplicate[dim] +=
                                rng.gen_range(-amplitude * axis_scale..amplitude * axis_scale);
                        }
                        shifted.push(duplicate);
                    }
                } else if rng.gen_bool(
                    (sparse_keep_probability + 0.15 * (1.0 - severity_scale)).clamp(0.0, 1.0),
                ) {
                    shifted.push(point.clone());
                }
            }
            if shifted.is_empty() {
                shifted.extend(points.iter().cloned());
            }
            while shifted.len() < target_len {
                let source = shifted
                    .choose(rng)
                    .cloned()
                    .or_else(|| points.choose(rng).cloned())
                    .expect("density shift should have a source point");
                shifted.push(source);
            }
            if shifted.len() > target_len {
                shifted.shuffle(rng);
                shifted.truncate(target_len);
            }
            *points = shifted;
        }
        CloudCorruptionMode::SectorDrop {
            start_angle,
            end_angle,
            drop_probability,
        } => {
            points.retain(|point| {
                let theta = point[1].atan2(point[0]);
                let normalized = if theta < 0.0 { theta + 2.0 * PI } else { theta };
                let in_sector = normalized >= start_angle && normalized <= end_angle;
                !(in_sector && rng.gen_bool((drop_probability * severity_scale).clamp(0.0, 0.98)))
            });
        }
    }
}

fn jittered_point(base: [f64; 3], jitter_scale: f64, rng: &mut StdRng) -> DVector<f64> {
    DVector::from_vec(vec![
        base[0] + rng.gen_range(-jitter_scale..jitter_scale),
        base[1] + rng.gen_range(-jitter_scale..jitter_scale),
        base[2] + rng.gen_range(-0.8 * jitter_scale..0.8 * jitter_scale),
    ])
}

fn evaluate_voxel_quality(points: &[DVector<f64>], target_count: usize) -> f64 {
    let target_count = target_count.clamp(8, points.len());
    let spacing = characteristic_spacing(points, target_count);
    let mut best_score = f64::INFINITY;
    for factor in [0.35, 0.55, 0.85, 1.30, 2.0] {
        let sample = voxel_point_sampling(points, (spacing * factor).max(1e-4));
        let quality = score_sample_quality(points, &sample, target_count);
        best_score = best_score.min(quality);
    }
    best_score
}

fn evaluate_farthest_quality(points: &[DVector<f64>], target_count: usize, slot_seed: u64) -> f64 {
    let target_count = target_count.clamp(8, points.len());
    let sample = farthest_point_sampling(points, target_count, family_seed("fps") ^ slot_seed);
    score_sample_quality(points, &sample, target_count)
}

fn evaluate_poisson_quality(points: &[DVector<f64>], target_count: usize, slot_seed: u64) -> f64 {
    let target_count = target_count.clamp(8, points.len());
    let spacing = characteristic_spacing(points, target_count);
    let max_iter = points.len() * 40;
    let mut best_score = f64::INFINITY;
    for factor in [0.10, 0.20, 0.35, 0.60, 1.0, 1.45] {
        let sample = poisson_disk_sampling(
            points,
            target_count,
            (spacing * factor).max(1e-6),
            family_seed("poisson") ^ slot_seed,
            max_iter,
        );
        let quality = score_sample_quality(points, &sample, target_count);
        best_score = best_score.min(quality);
    }
    best_score
}

fn score_sample_quality(
    original: &[DVector<f64>],
    sample: &[DVector<f64>],
    target_count: usize,
) -> f64 {
    if sample.is_empty() {
        return f64::INFINITY;
    }

    let coverage_error = average_nearest_distance(original, sample);
    let support_error = average_nearest_distance(sample, original);
    let spacing = mean_sample_spacing(sample);
    let count_error_ratio =
        (sample.len() as f64 - target_count as f64).abs() / target_count.max(1) as f64;
    let centroid_error = centroid_distance(original, sample);

    coverage_error
        + 0.25 * support_error
        + 0.45 * count_error_ratio
        + 0.12 / spacing.max(1e-6)
        + 0.08 * centroid_error
}

fn average_nearest_distance(points: &[DVector<f64>], reference: &[DVector<f64>]) -> f64 {
    if points.is_empty() || reference.is_empty() {
        return f64::INFINITY;
    }

    points
        .iter()
        .map(|point| {
            reference
                .iter()
                .map(|candidate| (point - candidate).norm())
                .fold(f64::INFINITY, f64::min)
        })
        .sum::<f64>()
        / points.len() as f64
}

fn mean_sample_spacing(sample: &[DVector<f64>]) -> f64 {
    if sample.len() <= 1 {
        return 1e-6;
    }

    sample
        .iter()
        .enumerate()
        .map(|(index, point)| {
            sample
                .iter()
                .enumerate()
                .filter(|(candidate_index, _)| *candidate_index != index)
                .map(|(_, candidate)| (point - candidate).norm())
                .fold(f64::INFINITY, f64::min)
        })
        .sum::<f64>()
        / sample.len() as f64
}

fn centroid_distance(lhs: &[DVector<f64>], rhs: &[DVector<f64>]) -> f64 {
    if lhs.is_empty() || rhs.is_empty() {
        return f64::INFINITY;
    }

    let lhs_centroid = mean_point(lhs);
    let rhs_centroid = mean_point(rhs);
    (lhs_centroid - rhs_centroid).norm()
}

fn mean_point(points: &[DVector<f64>]) -> DVector<f64> {
    let dim = points.first().map(DVector::len).unwrap_or(0);
    let mut mean = DVector::zeros(dim);
    for point in points {
        mean += point;
    }
    mean / points.len().max(1) as f64
}

fn characteristic_spacing(points: &[DVector<f64>], target_count: usize) -> f64 {
    let (min_point, max_point) = bounds(points);
    let extent = (&max_point - &min_point).norm().max(1e-3);
    let dim = points.first().map(DVector::len).unwrap_or(3) as f64;
    extent / (target_count.max(1) as f64).powf(1.0 / dim).max(1.0)
}

fn bounds(points: &[DVector<f64>]) -> (DVector<f64>, DVector<f64>) {
    let mut min_point = points
        .first()
        .cloned()
        .unwrap_or_else(|| DVector::from_vec(vec![0.0, 0.0, 0.0]));
    let mut max_point = min_point.clone();
    for point in points.iter().skip(1) {
        for dim in 0..point.len() {
            min_point[dim] = min_point[dim].min(point[dim]);
            max_point[dim] = max_point[dim].max(point[dim]);
        }
    }
    (min_point, max_point)
}

fn family_seed(name: &str) -> u64 {
    name.bytes().fold(0u64, |hash, byte| {
        hash.wrapping_mul(131).wrapping_add(byte as u64)
    })
}

fn median_value(values: &[f64]) -> f64 {
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    let mid = sorted.len() / 2;
    if sorted.len().is_multiple_of(2) {
        (sorted[mid - 1] + sorted[mid]) / 2.0
    } else {
        sorted[mid]
    }
}

fn min_value(values: &[f64]) -> f64 {
    values.iter().copied().fold(f64::INFINITY, f64::min)
}

fn max_value(values: &[f64]) -> f64 {
    values.iter().copied().fold(f64::NEG_INFINITY, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reduced_problem() -> Vec<PointSamplingExperimentCase> {
        mapping_point_cloud_sampling_process_problem()
            .into_iter()
            .take(1)
            .map(|mut case| {
                case.buckets.truncate(1);
                case
            })
            .collect()
    }

    #[test]
    fn point_sampling_problem_runs_and_stays_finite() {
        let reports = run_variant_suite(
            &default_point_sampling_variants(),
            &reduced_problem(),
            PointSamplingEvaluationConfig {
                scenarios_per_bucket: 2,
            },
        );
        assert_eq!(reports.len(), 5);
        assert!(reports
            .iter()
            .all(|report| report.evaluation_runtime_ms.is_finite()));
        assert!(reports.iter().all(|report| {
            report.observations.iter().all(|observation| {
                observation.voxel_bucket_median_score.is_finite()
                    && observation.farthest_bucket_median_score.is_finite()
                    && observation.poisson_bucket_median_score.is_finite()
            })
        }));
    }

    #[test]
    fn point_sampling_problem_reference_covers_all_scenarios() {
        let reports = run_variant_suite(
            &default_point_sampling_variants(),
            &reduced_problem(),
            PointSamplingEvaluationConfig {
                scenarios_per_bucket: 3,
            },
        );
        let reference = reports
            .iter()
            .find(|report| report.descriptor.id == "full-bucket")
            .expect("full-bucket report should exist");
        assert!(reference.observations.iter().all(|observation| {
            observation.selected_slots.len() == observation.total_scenarios
        }));
    }

    #[test]
    fn occlusion_problem_runs_and_stays_finite() {
        let reports = run_variant_suite(
            &default_point_sampling_variants(),
            &mapping_occlusion_corruption_process_problem()
                .into_iter()
                .take(1)
                .map(|mut case| {
                    case.buckets.truncate(1);
                    case
                })
                .collect::<Vec<_>>(),
            PointSamplingEvaluationConfig {
                scenarios_per_bucket: 2,
            },
        );
        assert_eq!(reports.len(), 5);
        assert!(reports.iter().all(|report| {
            report.observations.iter().all(|observation| {
                observation.voxel_bucket_median_score.is_finite()
                    && observation.farthest_bucket_median_score.is_finite()
                    && observation.poisson_bucket_median_score.is_finite()
            })
        }));
    }

    #[test]
    fn density_shift_problem_runs_and_stays_finite() {
        let reports = run_variant_suite(
            &default_point_sampling_variants(),
            &mapping_density_shift_process_problem()
                .into_iter()
                .take(1)
                .map(|mut case| {
                    case.buckets.truncate(1);
                    case
                })
                .collect::<Vec<_>>(),
            PointSamplingEvaluationConfig {
                scenarios_per_bucket: 2,
            },
        );
        assert_eq!(reports.len(), 5);
        assert!(reports.iter().all(|report| {
            report.observations.iter().all(|observation| {
                observation.voxel_bucket_median_score.is_finite()
                    && observation.farthest_bucket_median_score.is_finite()
                    && observation.poisson_bucket_median_score.is_finite()
            })
        }));
    }

    #[test]
    fn anisotropic_noise_problem_runs_and_stays_finite() {
        let reports = run_variant_suite(
            &default_point_sampling_variants(),
            &mapping_anisotropic_noise_process_problem()
                .into_iter()
                .take(1)
                .map(|mut case| {
                    case.buckets.truncate(1);
                    case
                })
                .collect::<Vec<_>>(),
            PointSamplingEvaluationConfig {
                scenarios_per_bucket: 2,
            },
        );
        assert_eq!(reports.len(), 5);
        assert!(reports.iter().all(|report| {
            report.observations.iter().all(|observation| {
                observation.voxel_bucket_median_score.is_finite()
                    && observation.farthest_bucket_median_score.is_finite()
                    && observation.poisson_bucket_median_score.is_finite()
            })
        }));
    }

    #[test]
    fn sparse_outlier_burst_problem_runs_and_stays_finite() {
        let reports = run_variant_suite(
            &default_point_sampling_variants(),
            &mapping_sparse_outlier_burst_process_problem()
                .into_iter()
                .take(1)
                .map(|mut case| {
                    case.buckets.truncate(1);
                    case
                })
                .collect::<Vec<_>>(),
            PointSamplingEvaluationConfig {
                scenarios_per_bucket: 2,
            },
        );
        assert_eq!(reports.len(), 5);
        assert!(reports.iter().all(|report| {
            report.observations.iter().all(|observation| {
                observation.voxel_bucket_median_score.is_finite()
                    && observation.farthest_bucket_median_score.is_finite()
                    && observation.poisson_bucket_median_score.is_finite()
            })
        }));
    }

    #[test]
    fn resolution_ladder_problem_runs_and_stays_finite() {
        let reports = run_variant_suite(
            &default_point_sampling_variants(),
            &mapping_resolution_ladder_process_problem()
                .into_iter()
                .take(1)
                .map(|mut case| {
                    case.buckets.truncate(1);
                    case
                })
                .collect::<Vec<_>>(),
            PointSamplingEvaluationConfig {
                scenarios_per_bucket: 2,
            },
        );
        assert_eq!(reports.len(), 5);
        assert!(reports.iter().all(|report| {
            report.observations.iter().all(|observation| {
                observation.voxel_bucket_median_score.is_finite()
                    && observation.farthest_bucket_median_score.is_finite()
                    && observation.poisson_bucket_median_score.is_finite()
            })
        }));
    }
}
