mod first_scenario;
mod full_bucket;
mod percentile_bucket;
mod sampled_bucket;
mod variance_triggered;

use std::collections::VecDeque;
use std::time::Instant;

use rust_robotics_core::{
    annotate_against_reference as annotate_reference_reports, average_coverage_ratio,
    read_source_metrics, ControlInput, ExperimentObservation, ExperimentSamplingPlan,
    ExperimentVariantReport, ExtensibilityMetrics, Path2D, PathTracker, Point2D, State2D,
    VariantDescriptor,
};

use crate::{PurePursuitConfig, PurePursuitController, StanleyConfig, StanleyController};

pub use first_scenario::FirstScenarioTrackingAggregation;
pub use full_bucket::FullBucketTrackingAggregation;
pub use percentile_bucket::PercentileBucketTrackingAggregation;
pub use sampled_bucket::SampledBucketTrackingAggregation;
pub use variance_triggered::VarianceTriggeredTrackingAggregation;

const DT: f64 = 0.1;
const MAX_LINEAR_SPEED: f64 = 8.0;
const MAX_ANGULAR_SPEED: f64 = 1.4;

pub type TrackingSamplingPlan = ExperimentSamplingPlan;

pub trait TrackingAggregationVariant {
    fn descriptor(&self) -> VariantDescriptor;
    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize>;

    fn sampling_plan(&self, total_scenarios: usize) -> TrackingSamplingPlan {
        TrackingSamplingPlan::static_slots(self.selected_slots(total_scenarios))
    }
}

#[derive(Debug, Clone)]
pub struct TrackingExperimentCase {
    pub family_name: &'static str,
    pub path: Path2D,
    pub buckets: Vec<u32>,
    base_lateral_offset: f64,
    base_yaw_offset_deg: f64,
    initial_speed: f64,
    control_latency_steps: usize,
    velocity_response_gain: f64,
    omega_response_gain: f64,
    goal_penalty_weight: f64,
}

#[derive(Debug, Clone)]
pub struct TrackingObservation {
    pub family_name: &'static str,
    pub bucket: u32,
    pub total_scenarios: usize,
    pub initial_slots: Vec<usize>,
    pub selected_slots: Vec<usize>,
    pub escalated: bool,
    pub pure_pursuit_bucket_median_score: f64,
    pub stanley_bucket_median_score: f64,
    pub pure_pursuit_min_score: f64,
    pub pure_pursuit_max_score: f64,
    pub stanley_min_score: f64,
    pub stanley_max_score: f64,
    pub stanley_wins: usize,
}

impl TrackingObservation {
    pub fn pure_pursuit_over_stanley(&self) -> f64 {
        self.pure_pursuit_bucket_median_score / self.stanley_bucket_median_score.max(1e-9)
    }

    pub fn winner(&self) -> &'static str {
        if self.pure_pursuit_over_stanley() > 1.0 {
            "Stanley"
        } else {
            "PurePursuit"
        }
    }

    pub fn coverage_ratio(&self) -> f64 {
        self.selected_slots.len() as f64 / self.total_scenarios as f64
    }
}

impl ExperimentObservation for TrackingObservation {
    type Key = (&'static str, u32);

    fn comparison_key(&self) -> Self::Key {
        (self.family_name, self.bucket)
    }

    fn winner_label(&self) -> &'static str {
        TrackingObservation::winner(self)
    }

    fn ratio_value(&self) -> f64 {
        TrackingObservation::pure_pursuit_over_stanley(self)
    }

    fn coverage_ratio(&self) -> f64 {
        TrackingObservation::coverage_ratio(self)
    }
}

pub type TrackingVariantReport = ExperimentVariantReport<TrackingObservation>;

#[derive(Debug, Clone, Copy)]
pub struct TrackingEvaluationConfig {
    pub scenarios_per_bucket: usize,
}

impl Default for TrackingEvaluationConfig {
    fn default() -> Self {
        Self {
            scenarios_per_bucket: 10,
        }
    }
}

pub fn control_tracking_process_problem() -> Vec<TrackingExperimentCase> {
    vec![
        TrackingExperimentCase {
            family_name: "straight-recovery",
            path: build_straight_path(120.0, 1.0),
            buckets: vec![60, 120, 180],
            base_lateral_offset: 1.2,
            base_yaw_offset_deg: 8.0,
            initial_speed: 0.4,
            control_latency_steps: 0,
            velocity_response_gain: 1.0,
            omega_response_gain: 1.0,
            goal_penalty_weight: 0.25,
        },
        TrackingExperimentCase {
            family_name: "slalom-recovery",
            path: build_slalom_path(110.0, 1.0),
            buckets: vec![80, 140, 220],
            base_lateral_offset: 1.5,
            base_yaw_offset_deg: 11.0,
            initial_speed: 0.5,
            control_latency_steps: 0,
            velocity_response_gain: 1.0,
            omega_response_gain: 1.0,
            goal_penalty_weight: 0.25,
        },
        TrackingExperimentCase {
            family_name: "tight-turn-recovery",
            path: build_tight_turn_path(),
            buckets: vec![100, 180, 260],
            base_lateral_offset: 1.8,
            base_yaw_offset_deg: 14.0,
            initial_speed: 0.3,
            control_latency_steps: 0,
            velocity_response_gain: 1.0,
            omega_response_gain: 1.0,
            goal_penalty_weight: 0.25,
        },
    ]
}

pub fn control_actuation_mismatch_process_problem() -> Vec<TrackingExperimentCase> {
    vec![
        TrackingExperimentCase {
            family_name: "velocity-sag-straight",
            path: build_straight_path(150.0, 1.0),
            buckets: vec![80, 140, 220],
            base_lateral_offset: 1.0,
            base_yaw_offset_deg: 9.0,
            initial_speed: 0.8,
            control_latency_steps: 1,
            velocity_response_gain: 0.78,
            omega_response_gain: 0.92,
            goal_penalty_weight: 0.30,
        },
        TrackingExperimentCase {
            family_name: "steering-lag-slalom",
            path: build_slalom_path(130.0, 0.8),
            buckets: vec![100, 180, 260],
            base_lateral_offset: 1.7,
            base_yaw_offset_deg: 13.0,
            initial_speed: 1.0,
            control_latency_steps: 3,
            velocity_response_gain: 0.94,
            omega_response_gain: 0.68,
            goal_penalty_weight: 0.35,
        },
        TrackingExperimentCase {
            family_name: "understeer-hairpin",
            path: build_offset_hairpin_path(),
            buckets: vec![120, 200, 300],
            base_lateral_offset: 2.0,
            base_yaw_offset_deg: 16.0,
            initial_speed: 0.9,
            control_latency_steps: 2,
            velocity_response_gain: 0.88,
            omega_response_gain: 0.58,
            goal_penalty_weight: 0.40,
        },
    ]
}

pub fn default_tracking_variants() -> Vec<Box<dyn TrackingAggregationVariant>> {
    vec![
        Box::new(FirstScenarioTrackingAggregation::new()),
        Box::new(SampledBucketTrackingAggregation::new(vec![0, 4, 9])),
        Box::new(PercentileBucketTrackingAggregation::new(vec![
            0.0, 0.25, 0.5, 0.75, 1.0,
        ])),
        Box::new(VarianceTriggeredTrackingAggregation::new(
            vec![0, 4, 9],
            0.10,
        )),
        Box::new(FullBucketTrackingAggregation::new()),
    ]
}

pub fn run_variant_suite(
    variants: &[Box<dyn TrackingAggregationVariant>],
    cases: &[TrackingExperimentCase],
    config: TrackingEvaluationConfig,
) -> Vec<TrackingVariantReport> {
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

        reports.push(TrackingVariantReport {
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
    variant: &dyn TrackingAggregationVariant,
    case: &TrackingExperimentCase,
    bucket: u32,
    total_scenarios: usize,
) -> TrackingObservation {
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

    let pure_pursuit_samples: Vec<f64> = slot_samples
        .iter()
        .map(|sample| sample.pure_pursuit_score)
        .collect();
    let stanley_samples: Vec<f64> = slot_samples
        .iter()
        .map(|sample| sample.stanley_score)
        .collect();
    let stanley_wins = slot_samples
        .iter()
        .filter(|sample| sample.stanley_score < sample.pure_pursuit_score)
        .count();

    TrackingObservation {
        family_name: case.family_name,
        bucket,
        total_scenarios,
        initial_slots,
        selected_slots,
        escalated,
        pure_pursuit_bucket_median_score: median_value(&pure_pursuit_samples),
        stanley_bucket_median_score: median_value(&stanley_samples),
        pure_pursuit_min_score: min_value(&pure_pursuit_samples),
        pure_pursuit_max_score: max_value(&pure_pursuit_samples),
        stanley_min_score: min_value(&stanley_samples),
        stanley_max_score: max_value(&stanley_samples),
        stanley_wins,
    }
}

#[derive(Debug, Clone, Copy)]
struct SlotTrackingSample {
    pure_pursuit_score: f64,
    stanley_score: f64,
}

fn measure_slot_samples(
    slots: &[usize],
    case: &TrackingExperimentCase,
    bucket: u32,
) -> Vec<SlotTrackingSample> {
    let mut samples = Vec::with_capacity(slots.len());
    for &slot in slots {
        let initial_state = build_initial_state(case, bucket, slot);
        let pure_pursuit_score = simulate_pure_pursuit(case, bucket, initial_state);
        let stanley_score = simulate_stanley(case, bucket, initial_state);
        assert!(
            pure_pursuit_score.is_finite(),
            "PurePursuit score must stay finite"
        );
        assert!(stanley_score.is_finite(), "Stanley score must stay finite");
        samples.push(SlotTrackingSample {
            pure_pursuit_score,
            stanley_score,
        });
    }
    samples
}

fn should_escalate(slot_samples: &[SlotTrackingSample], plan: &TrackingSamplingPlan) -> bool {
    if slot_samples.is_empty() || plan.escalation_slots.is_empty() {
        return false;
    }

    let vote_split = {
        let stanley_wins = slot_samples
            .iter()
            .filter(|sample| sample.stanley_score < sample.pure_pursuit_score)
            .count();
        stanley_wins > 0 && stanley_wins < slot_samples.len()
    };
    let ratio_close = plan
        .escalate_if_ratio_margin_below
        .map(|threshold| {
            let pure_pursuit_samples: Vec<f64> = slot_samples
                .iter()
                .map(|sample| sample.pure_pursuit_score)
                .collect();
            let stanley_samples: Vec<f64> = slot_samples
                .iter()
                .map(|sample| sample.stanley_score)
                .collect();
            (median_value(&pure_pursuit_samples) / median_value(&stanley_samples).max(1e-9) - 1.0)
                .abs()
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

fn simulate_pure_pursuit(
    case: &TrackingExperimentCase,
    bucket: u32,
    initial_state: State2D,
) -> f64 {
    let mut tracker = PurePursuitController::new(PurePursuitConfig {
        look_ahead_gain: 0.12,
        look_ahead_distance: 2.4,
        wheelbase: 2.9,
        kp: 1.0,
        goal_threshold: 2.0,
    });
    simulate_tracker(&mut tracker, case, initial_state, bucket)
}

fn simulate_stanley(case: &TrackingExperimentCase, bucket: u32, initial_state: State2D) -> f64 {
    let mut tracker = StanleyController::new(StanleyConfig {
        k: 0.55,
        wheelbase: 2.9,
        kp: 1.0,
        goal_threshold: 2.5,
    });
    simulate_tracker(&mut tracker, case, initial_state, bucket)
}

fn simulate_tracker<T: PathTracker>(
    tracker: &mut T,
    case: &TrackingExperimentCase,
    mut state: State2D,
    bucket: u32,
) -> f64 {
    let steps = bucket as usize;
    let path = &case.path;
    let goal = *path
        .points
        .last()
        .expect("path should have at least one point");
    let mut squared_error_sum = 0.0;
    let mut control_buffer =
        VecDeque::from(vec![ControlInput::zero(); case.control_latency_steps + 1]);

    for _ in 0..steps {
        let control = tracker.compute_control(&state, path);
        control_buffer.push_back(control);
        let delayed_control = control_buffer
            .pop_front()
            .expect("control buffer should always contain one element");
        let actual_control = ControlInput::new(
            delayed_control.v * case.velocity_response_gain,
            delayed_control.omega * case.omega_response_gain,
        );
        state = propagate_state(state, actual_control);
        let query = Point2D::new(state.x, state.y);
        let nearest_idx = path.nearest_point_index(query).unwrap_or(0);
        let nearest_point = path.points[nearest_idx];
        let error = query.distance(&nearest_point);
        squared_error_sum += error * error;
    }

    let rmse = (squared_error_sum / steps.max(1) as f64).sqrt();
    let goal_distance = state.position().distance(&goal);
    rmse + case.goal_penalty_weight * goal_distance
}

fn propagate_state(state: State2D, control: ControlInput) -> State2D {
    let v = control.v.clamp(0.0, MAX_LINEAR_SPEED);
    let omega = control.omega.clamp(-MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
    let mut yaw = state.yaw + omega * DT;
    while yaw > std::f64::consts::PI {
        yaw -= 2.0 * std::f64::consts::PI;
    }
    while yaw < -std::f64::consts::PI {
        yaw += 2.0 * std::f64::consts::PI;
    }

    State2D::new(
        state.x + v * state.yaw.cos() * DT,
        state.y + v * state.yaw.sin() * DT,
        yaw,
        v,
    )
}

fn build_initial_state(case: &TrackingExperimentCase, bucket: u32, slot: usize) -> State2D {
    let start = case.path.points[0];
    let yaw = case.path.yaw_profile().first().copied().unwrap_or(0.0);
    let scale = bucket as f64 / 100.0;
    let slot_phase = slot as f64 * 0.73 + 0.4;
    let lateral_offset = case.base_lateral_offset * scale * slot_phase.sin();
    let backward_offset = (1.5 + slot as f64 * 0.15) * scale;
    let yaw_offset = case.base_yaw_offset_deg.to_radians() * scale * (slot_phase * 0.7).cos();
    let speed = (case.initial_speed + 0.08 * slot as f64).min(2.2);

    let x = start.x - backward_offset * yaw.cos() - lateral_offset * yaw.sin();
    let y = start.y - backward_offset * yaw.sin() + lateral_offset * yaw.cos();

    State2D::new(x, y, yaw + yaw_offset, speed)
}

fn build_straight_path(length: f64, ds: f64) -> Path2D {
    let steps = (length / ds).round() as usize;
    Path2D::from_points(
        (0..=steps)
            .map(|i| Point2D::new(i as f64 * ds, 0.0))
            .collect(),
    )
}

fn build_slalom_path(length: f64, ds: f64) -> Path2D {
    let steps = (length / ds).round() as usize;
    Path2D::from_points(
        (0..=steps)
            .map(|i| {
                let x = i as f64 * ds;
                let y = 2.4 * (x / 8.0).sin() + 0.6 * (x / 3.5).sin();
                Point2D::new(x, y)
            })
            .collect(),
    )
}

fn build_tight_turn_path() -> Path2D {
    let mut points = Vec::new();
    for i in 0..=30 {
        points.push(Point2D::new(i as f64, 0.0));
    }
    for i in 1..=18 {
        let theta = -std::f64::consts::FRAC_PI_2 + i as f64 / 18.0 * std::f64::consts::FRAC_PI_2;
        points.push(Point2D::new(
            30.0 + 10.0 * theta.cos(),
            10.0 + 10.0 * theta.sin(),
        ));
    }
    for i in 1..=40 {
        points.push(Point2D::new(40.0, 10.0 + i as f64));
    }
    Path2D::from_points(points)
}

fn build_offset_hairpin_path() -> Path2D {
    let mut points = Vec::new();
    for i in 0..=40 {
        points.push(Point2D::new(i as f64, 0.4 * (i as f64 / 6.0).sin()));
    }
    for i in 1..=24 {
        let theta = -std::f64::consts::FRAC_PI_2 + i as f64 / 24.0 * (std::f64::consts::PI * 0.92);
        points.push(Point2D::new(
            40.0 + 12.0 * theta.cos(),
            12.0 + 12.0 * theta.sin(),
        ));
    }
    for i in 1..=46 {
        points.push(Point2D::new(
            28.0 - i as f64 * 0.85,
            24.0 + 0.8 * (i as f64 / 5.0).sin(),
        ));
    }
    Path2D::from_points(points)
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
        let variant = FirstScenarioTrackingAggregation::new();
        assert_eq!(variant.selected_slots(10), vec![0]);
    }

    #[test]
    fn percentile_variant_maps_percentiles_to_unique_slots() {
        let variant = PercentileBucketTrackingAggregation::new(vec![0.0, 0.25, 0.5, 0.75, 1.0]);
        assert_eq!(variant.selected_slots(10), vec![0, 2, 5, 7, 9]);
        assert_eq!(variant.selected_slots(1), vec![0]);
    }

    #[test]
    fn suite_runs_on_control_tracking_problem() {
        let variants = default_tracking_variants();
        let mut problem = control_tracking_process_problem();
        problem.truncate(1);
        problem[0].buckets = vec![60];
        let reports = run_variant_suite(&variants, &problem, TrackingEvaluationConfig::default());
        assert_eq!(reports.len(), 5);
        assert!(reports
            .iter()
            .all(|report| !report.observations.is_empty() && report.evaluation_runtime_ms >= 0.0));
    }

    #[test]
    fn representative_scores_are_finite() {
        let problem = control_tracking_process_problem();
        let initial_state = build_initial_state(&problem[1], 80, 3);
        let pp = simulate_pure_pursuit(&problem[1], 80, initial_state);
        let stanley = simulate_stanley(&problem[1], 80, initial_state);
        assert!(pp.is_finite());
        assert!(stanley.is_finite());
    }

    #[test]
    fn actuation_mismatch_problem_runs_and_stays_finite() {
        let variants = default_tracking_variants();
        let mut problem = control_actuation_mismatch_process_problem();
        problem.truncate(1);
        problem[0].buckets = vec![80];
        let reports = run_variant_suite(&variants, &problem, TrackingEvaluationConfig::default());
        assert_eq!(reports.len(), 5);
        assert!(reports.iter().all(|report| {
            !report.observations.is_empty()
                && report.observations.iter().all(|observation| {
                    observation.pure_pursuit_bucket_median_score.is_finite()
                        && observation.stanley_bucket_median_score.is_finite()
                })
        }));
    }
}
