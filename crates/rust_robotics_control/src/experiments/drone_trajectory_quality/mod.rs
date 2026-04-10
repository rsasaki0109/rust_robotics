use std::hint::black_box;
use std::time::Instant;

use nalgebra::{DMatrix, Vector3};

use crate::drone_3d_trajectory::{
    compute_control, generate_trajectory_segment_full, generate_waypoint_trajectory_with_durations,
    sample_trajectory_segments, simulate_desired_states, step_dynamics, ControlOutput,
    ControllerGains, DesiredState, QuadrotorParams, QuadrotorState, SimulationConfig,
    SimulationRecord, TrajectorySegment,
};
use crate::minimum_snap_trajectory::{
    generate_minimum_snap_segment_full, generate_waypoint_minimum_snap_trajectory_with_durations,
    sample_minimum_snap_trajectory, MinimumSnapBoundary, MinimumSnapSegment,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DroneTrajectoryVariant {
    QuinticUniform,
    QuinticDistanceScaled,
    MinimumSnapDistanceScaled,
}

impl DroneTrajectoryVariant {
    pub fn id(self) -> &'static str {
        match self {
            Self::QuinticUniform => "quintic-uniform",
            Self::QuinticDistanceScaled => "quintic-distance-scaled",
            Self::MinimumSnapDistanceScaled => "minimum-snap-distance-scaled",
        }
    }

    pub fn design_style(self) -> &'static str {
        match self {
            Self::QuinticUniform => "fixed-duration-quintic",
            Self::QuinticDistanceScaled => "distance-scaled-quintic",
            Self::MinimumSnapDistanceScaled => "distance-scaled-minimum-snap",
        }
    }
}

#[derive(Debug, Clone)]
pub struct DroneTrajectoryCase {
    pub family_name: &'static str,
    pub waypoints: Vec<Vector3<f64>>,
    pub baseline_segment_duration: f64,
    pub cruise_speed: f64,
    pub min_segment_duration: f64,
    pub dt: f64,
    pub boundary_mode: DroneBoundaryMode,
    pub tangent_scale: f64,
    pub acceleration_scale: f64,
    pub jerk_scale: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DroneBoundaryMode {
    StopGo,
    PassThrough,
    PassThroughAccel,
    PassThroughAccelJerk,
    CoupledContinuity,
}

#[derive(Debug, Clone)]
pub struct DroneTrajectoryMetrics {
    pub family_name: &'static str,
    pub variant: DroneTrajectoryVariant,
    pub generation_us: f64,
    pub total_duration_s: f64,
    pub tracking_rmse_m: f64,
    pub max_position_error_m: f64,
    pub jerk_rms: f64,
    pub snap_rms: f64,
    pub max_speed_mps: f64,
    pub max_acceleration_mps2: f64,
}

#[derive(Debug, Clone)]
pub struct DroneTrajectorySummary {
    pub variant: DroneTrajectoryVariant,
    pub mean_generation_us: f64,
    pub mean_total_duration_s: f64,
    pub mean_tracking_rmse_m: f64,
    pub mean_max_position_error_m: f64,
    pub mean_jerk_rms: f64,
    pub mean_snap_rms: f64,
    pub mean_max_speed_mps: f64,
    pub mean_max_acceleration_mps2: f64,
}

/// Tunable gains for the bounded lateral feedback controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LateralGainSet {
    pub lateral_position_gain: f64,
    pub lateral_velocity_gain: f64,
    pub max_lateral_correction: f64,
    pub max_attitude_command: f64,
    pub rate_damping: Vector3<f64>,
}

impl Default for LateralGainSet {
    fn default() -> Self {
        Self {
            lateral_position_gain: 0.15,
            lateral_velocity_gain: 0.45,
            max_lateral_correction: 0.75,
            max_attitude_command: 0.30,
            rate_damping: Vector3::new(6.0, 6.0, 4.0),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DroneControllerVariant {
    BaselinePd,
    AttitudeRateDampedPd,
    AttitudeRateDampedLateralPd,
}

impl DroneControllerVariant {
    pub fn id(self) -> &'static str {
        match self {
            Self::BaselinePd => "baseline-pd",
            Self::AttitudeRateDampedPd => "attitude-rate-damped-pd",
            Self::AttitudeRateDampedLateralPd => "attitude-rate-damped-lateral-pd",
        }
    }

    pub fn design_style(self) -> &'static str {
        match self {
            Self::BaselinePd => "z-pd-plus-attitude-feedforward",
            Self::AttitudeRateDampedPd => "baseline-pd-plus-attitude-rate-damping",
            Self::AttitudeRateDampedLateralPd => {
                "baseline-pd-plus-bounded-lateral-feedback-and-rate-damping"
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct DroneControllerMetrics {
    pub family_name: &'static str,
    pub trajectory_variant: DroneTrajectoryVariant,
    pub controller_variant: DroneControllerVariant,
    pub generation_us: f64,
    pub total_duration_s: f64,
    pub tracking_rmse_m: f64,
    pub max_position_error_m: f64,
    pub mean_thrust_n: f64,
    pub max_thrust_n: f64,
    pub mean_torque_norm_nm: f64,
}

#[derive(Debug, Clone)]
pub struct DroneControllerSummary {
    pub controller_variant: DroneControllerVariant,
    pub mean_generation_us: f64,
    pub mean_total_duration_s: f64,
    pub mean_tracking_rmse_m: f64,
    pub mean_max_position_error_m: f64,
    pub mean_thrust_n: f64,
    pub mean_max_thrust_n: f64,
    pub mean_torque_norm_nm: f64,
}

#[derive(Debug, Clone)]
struct BuiltTrajectory {
    desired_states: Vec<DesiredState>,
    jerk_norms: Vec<f64>,
    snap_norms: Vec<f64>,
    total_duration_s: f64,
}

pub fn default_drone_trajectory_cases() -> Vec<DroneTrajectoryCase> {
    vec![
        DroneTrajectoryCase {
            family_name: "box-climb",
            waypoints: vec![
                Vector3::new(-4.0, -4.0, 1.5),
                Vector3::new(4.0, -4.0, 2.5),
                Vector3::new(4.0, 4.0, 3.5),
                Vector3::new(-4.0, 4.0, 2.0),
            ],
            baseline_segment_duration: 4.0,
            cruise_speed: 2.2,
            min_segment_duration: 2.5,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::StopGo,
            tangent_scale: 0.0,
            acceleration_scale: 0.0,
            jerk_scale: 0.0,
        },
        DroneTrajectoryCase {
            family_name: "diamond-ascent",
            waypoints: vec![
                Vector3::new(0.0, -5.5, 1.0),
                Vector3::new(5.0, 0.0, 2.8),
                Vector3::new(0.0, 5.5, 4.2),
                Vector3::new(-5.0, 0.0, 2.0),
            ],
            baseline_segment_duration: 3.6,
            cruise_speed: 2.4,
            min_segment_duration: 2.2,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::StopGo,
            tangent_scale: 0.0,
            acceleration_scale: 0.0,
            jerk_scale: 0.0,
        },
        DroneTrajectoryCase {
            family_name: "stair-zigzag",
            waypoints: vec![
                Vector3::new(-6.0, -2.0, 1.0),
                Vector3::new(-2.0, 2.0, 2.0),
                Vector3::new(2.0, -2.0, 3.0),
                Vector3::new(6.0, 2.0, 4.0),
            ],
            baseline_segment_duration: 3.8,
            cruise_speed: 2.5,
            min_segment_duration: 2.4,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::StopGo,
            tangent_scale: 0.0,
            acceleration_scale: 0.0,
            jerk_scale: 0.0,
        },
    ]
}

pub fn pass_through_drone_trajectory_cases() -> Vec<DroneTrajectoryCase> {
    vec![
        DroneTrajectoryCase {
            family_name: "oval-cruise",
            waypoints: vec![
                Vector3::new(-6.0, 0.0, 1.5),
                Vector3::new(-2.0, 5.0, 2.2),
                Vector3::new(4.0, 5.5, 3.0),
                Vector3::new(7.0, 0.5, 3.4),
                Vector3::new(3.0, -5.0, 2.7),
                Vector3::new(-3.0, -5.5, 1.8),
            ],
            baseline_segment_duration: 2.9,
            cruise_speed: 3.0,
            min_segment_duration: 1.8,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::PassThrough,
            tangent_scale: 0.85,
            acceleration_scale: 0.0,
            jerk_scale: 0.0,
        },
        DroneTrajectoryCase {
            family_name: "figure-eight-climb",
            waypoints: vec![
                Vector3::new(-5.5, 0.0, 1.0),
                Vector3::new(-1.5, 4.5, 2.0),
                Vector3::new(2.0, 0.0, 3.0),
                Vector3::new(5.5, -4.5, 4.0),
                Vector3::new(1.5, 0.0, 3.0),
                Vector3::new(-2.0, 4.5, 2.0),
            ],
            baseline_segment_duration: 2.7,
            cruise_speed: 3.1,
            min_segment_duration: 1.7,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::PassThrough,
            tangent_scale: 0.9,
            acceleration_scale: 0.0,
            jerk_scale: 0.0,
        },
        DroneTrajectoryCase {
            family_name: "banked-diamond",
            waypoints: vec![
                Vector3::new(0.0, -6.0, 1.2),
                Vector3::new(6.0, 0.0, 2.4),
                Vector3::new(0.0, 6.0, 3.6),
                Vector3::new(-6.0, 0.0, 2.4),
            ],
            baseline_segment_duration: 3.2,
            cruise_speed: 3.0,
            min_segment_duration: 2.0,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::PassThrough,
            tangent_scale: 0.82,
            acceleration_scale: 0.0,
            jerk_scale: 0.0,
        },
    ]
}

pub fn pass_through_accel_drone_trajectory_cases() -> Vec<DroneTrajectoryCase> {
    vec![
        DroneTrajectoryCase {
            family_name: "oval-cruise-accel",
            waypoints: vec![
                Vector3::new(-6.0, 0.0, 1.5),
                Vector3::new(-2.0, 5.0, 2.2),
                Vector3::new(4.0, 5.5, 3.0),
                Vector3::new(7.0, 0.5, 3.4),
                Vector3::new(3.0, -5.0, 2.7),
                Vector3::new(-3.0, -5.5, 1.8),
            ],
            baseline_segment_duration: 2.9,
            cruise_speed: 3.0,
            min_segment_duration: 1.8,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::PassThroughAccel,
            tangent_scale: 0.85,
            acceleration_scale: 0.65,
            jerk_scale: 0.0,
        },
        DroneTrajectoryCase {
            family_name: "figure-eight-climb-accel",
            waypoints: vec![
                Vector3::new(-5.5, 0.0, 1.0),
                Vector3::new(-1.5, 4.5, 2.0),
                Vector3::new(2.0, 0.0, 3.0),
                Vector3::new(5.5, -4.5, 4.0),
                Vector3::new(1.5, 0.0, 3.0),
                Vector3::new(-2.0, 4.5, 2.0),
            ],
            baseline_segment_duration: 2.7,
            cruise_speed: 3.1,
            min_segment_duration: 1.7,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::PassThroughAccel,
            tangent_scale: 0.9,
            acceleration_scale: 0.70,
            jerk_scale: 0.0,
        },
        DroneTrajectoryCase {
            family_name: "banked-diamond-accel",
            waypoints: vec![
                Vector3::new(0.0, -6.0, 1.2),
                Vector3::new(6.0, 0.0, 2.4),
                Vector3::new(0.0, 6.0, 3.6),
                Vector3::new(-6.0, 0.0, 2.4),
            ],
            baseline_segment_duration: 3.2,
            cruise_speed: 3.0,
            min_segment_duration: 2.0,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::PassThroughAccel,
            tangent_scale: 0.82,
            acceleration_scale: 0.60,
            jerk_scale: 0.0,
        },
    ]
}

pub fn pass_through_accel_jerk_drone_trajectory_cases() -> Vec<DroneTrajectoryCase> {
    vec![
        DroneTrajectoryCase {
            family_name: "oval-cruise-accel-jerk",
            waypoints: vec![
                Vector3::new(-6.0, 0.0, 1.5),
                Vector3::new(-2.0, 5.0, 2.2),
                Vector3::new(4.0, 5.5, 3.0),
                Vector3::new(7.0, 0.5, 3.4),
                Vector3::new(3.0, -5.0, 2.7),
                Vector3::new(-3.0, -5.5, 1.8),
            ],
            baseline_segment_duration: 2.9,
            cruise_speed: 3.0,
            min_segment_duration: 1.8,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::PassThroughAccelJerk,
            tangent_scale: 0.85,
            acceleration_scale: 0.65,
            jerk_scale: 0.45,
        },
        DroneTrajectoryCase {
            family_name: "figure-eight-climb-accel-jerk",
            waypoints: vec![
                Vector3::new(-5.5, 0.0, 1.0),
                Vector3::new(-1.5, 4.5, 2.0),
                Vector3::new(2.0, 0.0, 3.0),
                Vector3::new(5.5, -4.5, 4.0),
                Vector3::new(1.5, 0.0, 3.0),
                Vector3::new(-2.0, 4.5, 2.0),
            ],
            baseline_segment_duration: 2.7,
            cruise_speed: 3.1,
            min_segment_duration: 1.7,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::PassThroughAccelJerk,
            tangent_scale: 0.9,
            acceleration_scale: 0.70,
            jerk_scale: 0.50,
        },
        DroneTrajectoryCase {
            family_name: "banked-diamond-accel-jerk",
            waypoints: vec![
                Vector3::new(0.0, -6.0, 1.2),
                Vector3::new(6.0, 0.0, 2.4),
                Vector3::new(0.0, 6.0, 3.6),
                Vector3::new(-6.0, 0.0, 2.4),
            ],
            baseline_segment_duration: 3.2,
            cruise_speed: 3.0,
            min_segment_duration: 2.0,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::PassThroughAccelJerk,
            tangent_scale: 0.82,
            acceleration_scale: 0.60,
            jerk_scale: 0.40,
        },
    ]
}

pub fn coupled_continuity_drone_trajectory_cases() -> Vec<DroneTrajectoryCase> {
    vec![
        DroneTrajectoryCase {
            family_name: "oval-cruise-coupled",
            waypoints: vec![
                Vector3::new(-6.0, 0.0, 1.5),
                Vector3::new(-2.0, 5.0, 2.2),
                Vector3::new(4.0, 5.5, 3.0),
                Vector3::new(7.0, 0.5, 3.4),
                Vector3::new(3.0, -5.0, 2.7),
                Vector3::new(-3.0, -5.5, 1.8),
            ],
            baseline_segment_duration: 2.9,
            cruise_speed: 3.0,
            min_segment_duration: 1.8,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::CoupledContinuity,
            tangent_scale: 1.0,
            acceleration_scale: 1.0,
            jerk_scale: 1.0,
        },
        DroneTrajectoryCase {
            family_name: "figure-eight-climb-coupled",
            waypoints: vec![
                Vector3::new(-5.5, 0.0, 1.0),
                Vector3::new(-1.5, 4.5, 2.0),
                Vector3::new(2.0, 0.0, 3.0),
                Vector3::new(5.5, -4.5, 4.0),
                Vector3::new(1.5, 0.0, 3.0),
                Vector3::new(-2.0, 4.5, 2.0),
            ],
            baseline_segment_duration: 2.7,
            cruise_speed: 3.1,
            min_segment_duration: 1.7,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::CoupledContinuity,
            tangent_scale: 1.0,
            acceleration_scale: 1.0,
            jerk_scale: 1.0,
        },
        DroneTrajectoryCase {
            family_name: "banked-diamond-coupled",
            waypoints: vec![
                Vector3::new(0.0, -6.0, 1.2),
                Vector3::new(6.0, 0.0, 2.4),
                Vector3::new(0.0, 6.0, 3.6),
                Vector3::new(-6.0, 0.0, 2.4),
            ],
            baseline_segment_duration: 3.2,
            cruise_speed: 3.0,
            min_segment_duration: 2.0,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::CoupledContinuity,
            tangent_scale: 1.0,
            acceleration_scale: 1.0,
            jerk_scale: 1.0,
        },
        DroneTrajectoryCase {
            family_name: "spiral-climb-coupled",
            waypoints: vec![
                Vector3::new(4.0, 0.0, 1.0),
                Vector3::new(0.0, 4.0, 2.0),
                Vector3::new(-4.0, 0.0, 3.0),
                Vector3::new(0.0, -4.0, 4.0),
                Vector3::new(4.0, 0.0, 5.0),
                Vector3::new(0.0, 4.0, 6.0),
            ],
            baseline_segment_duration: 2.5,
            cruise_speed: 3.2,
            min_segment_duration: 1.6,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::CoupledContinuity,
            tangent_scale: 1.0,
            acceleration_scale: 1.0,
            jerk_scale: 1.0,
        },
        DroneTrajectoryCase {
            family_name: "reverse-s-coupled",
            waypoints: vec![
                Vector3::new(-6.0, -3.0, 1.5),
                Vector3::new(-2.0, 3.0, 2.5),
                Vector3::new(2.0, -3.0, 3.5),
                Vector3::new(6.0, 3.0, 2.5),
            ],
            baseline_segment_duration: 3.0,
            cruise_speed: 2.8,
            min_segment_duration: 1.9,
            dt: 0.05,
            boundary_mode: DroneBoundaryMode::CoupledContinuity,
            tangent_scale: 1.0,
            acceleration_scale: 1.0,
            jerk_scale: 1.0,
        },
    ]
}

pub fn default_drone_trajectory_variants() -> [DroneTrajectoryVariant; 3] {
    [
        DroneTrajectoryVariant::QuinticUniform,
        DroneTrajectoryVariant::QuinticDistanceScaled,
        DroneTrajectoryVariant::MinimumSnapDistanceScaled,
    ]
}

pub fn default_drone_controller_variants() -> [DroneControllerVariant; 3] {
    [
        DroneControllerVariant::BaselinePd,
        DroneControllerVariant::AttitudeRateDampedPd,
        DroneControllerVariant::AttitudeRateDampedLateralPd,
    ]
}

pub fn evaluate_drone_trajectory_variants(
    cases: &[DroneTrajectoryCase],
    variants: &[DroneTrajectoryVariant],
) -> (Vec<DroneTrajectoryMetrics>, Vec<DroneTrajectorySummary>) {
    let metrics: Vec<_> = cases
        .iter()
        .flat_map(|case| {
            variants
                .iter()
                .copied()
                .map(move |variant| evaluate_case_variant(case, variant))
        })
        .collect();

    let summaries = variants
        .iter()
        .copied()
        .map(|variant| summarize_variant(&metrics, variant))
        .collect();

    (metrics, summaries)
}

pub fn evaluate_drone_controller_variants(
    cases: &[DroneTrajectoryCase],
    trajectory_variants: &[DroneTrajectoryVariant],
    controller_variants: &[DroneControllerVariant],
) -> (Vec<DroneControllerMetrics>, Vec<DroneControllerSummary>) {
    let metrics: Vec<_> = cases
        .iter()
        .flat_map(|case| {
            trajectory_variants
                .iter()
                .copied()
                .flat_map(move |trajectory_variant| {
                    controller_variants
                        .iter()
                        .copied()
                        .map(move |controller_variant| {
                            evaluate_controller_case_variant(
                                case,
                                trajectory_variant,
                                controller_variant,
                            )
                        })
                })
        })
        .collect();

    let summaries = controller_variants
        .iter()
        .copied()
        .map(|controller_variant| summarize_controller_variant(&metrics, controller_variant))
        .collect();

    (metrics, summaries)
}

fn evaluate_case_variant(
    case: &DroneTrajectoryCase,
    variant: DroneTrajectoryVariant,
) -> DroneTrajectoryMetrics {
    let built = build_variant_trajectory(case, variant);
    let simulation_config = SimulationConfig {
        dt: case.dt,
        segment_duration: case.baseline_segment_duration,
        n_loops: 1,
        ..Default::default()
    };
    let record =
        simulate_desired_states(case.waypoints[0], &built.desired_states, &simulation_config);
    let actual_positions = &record.positions[1..];

    let mut squared_error_sum = 0.0;
    let mut max_position_error_m = 0.0_f64;
    for (actual, desired) in actual_positions.iter().zip(&built.desired_states) {
        let error = (actual - desired.position).norm();
        squared_error_sum += error * error;
        max_position_error_m = max_position_error_m.max(error);
    }

    let tracking_rmse_m = (squared_error_sum / built.desired_states.len() as f64).sqrt();
    let jerk_rms = rms(&built.jerk_norms);
    let snap_rms = rms(&built.snap_norms);
    let max_speed_mps = built
        .desired_states
        .iter()
        .map(|state| state.velocity.norm())
        .fold(0.0, f64::max);
    let max_acceleration_mps2 = built
        .desired_states
        .iter()
        .map(|state| state.acceleration.norm())
        .fold(0.0, f64::max);

    DroneTrajectoryMetrics {
        family_name: case.family_name,
        variant,
        generation_us: benchmark_generation(case, variant, 128),
        total_duration_s: built.total_duration_s,
        tracking_rmse_m,
        max_position_error_m,
        jerk_rms,
        snap_rms,
        max_speed_mps,
        max_acceleration_mps2,
    }
}

fn summarize_variant(
    metrics: &[DroneTrajectoryMetrics],
    variant: DroneTrajectoryVariant,
) -> DroneTrajectorySummary {
    let rows: Vec<_> = metrics
        .iter()
        .filter(|row| row.variant == variant)
        .collect();
    let denom = rows.len() as f64;

    DroneTrajectorySummary {
        variant,
        mean_generation_us: rows.iter().map(|row| row.generation_us).sum::<f64>() / denom,
        mean_total_duration_s: rows.iter().map(|row| row.total_duration_s).sum::<f64>() / denom,
        mean_tracking_rmse_m: rows.iter().map(|row| row.tracking_rmse_m).sum::<f64>() / denom,
        mean_max_position_error_m: rows.iter().map(|row| row.max_position_error_m).sum::<f64>()
            / denom,
        mean_jerk_rms: rows.iter().map(|row| row.jerk_rms).sum::<f64>() / denom,
        mean_snap_rms: rows.iter().map(|row| row.snap_rms).sum::<f64>() / denom,
        mean_max_speed_mps: rows.iter().map(|row| row.max_speed_mps).sum::<f64>() / denom,
        mean_max_acceleration_mps2: rows
            .iter()
            .map(|row| row.max_acceleration_mps2)
            .sum::<f64>()
            / denom,
    }
}

fn evaluate_controller_case_variant(
    case: &DroneTrajectoryCase,
    trajectory_variant: DroneTrajectoryVariant,
    controller_variant: DroneControllerVariant,
) -> DroneControllerMetrics {
    let built = build_variant_trajectory(case, trajectory_variant);
    let simulation_config = SimulationConfig {
        dt: case.dt,
        segment_duration: case.baseline_segment_duration,
        n_loops: 1,
        ..Default::default()
    };
    let record = simulate_desired_states_with_controller_variant(
        case.waypoints[0],
        &built.desired_states,
        &simulation_config,
        controller_variant,
    );
    let actual_positions = &record.positions[1..];

    let mut squared_error_sum = 0.0;
    let mut max_position_error_m = 0.0_f64;
    for (actual, desired) in actual_positions.iter().zip(&built.desired_states) {
        let error = (actual - desired.position).norm();
        squared_error_sum += error * error;
        max_position_error_m = max_position_error_m.max(error);
    }

    let tracking_rmse_m = (squared_error_sum / built.desired_states.len() as f64).sqrt();
    let mean_thrust_n = record
        .controls
        .iter()
        .map(|control| control.thrust)
        .sum::<f64>()
        / record.controls.len() as f64;
    let max_thrust_n = record
        .controls
        .iter()
        .map(|control| control.thrust)
        .fold(f64::NEG_INFINITY, f64::max);
    let mean_torque_norm_nm = record
        .controls
        .iter()
        .map(|control| {
            Vector3::new(
                control.roll_torque,
                control.pitch_torque,
                control.yaw_torque,
            )
            .norm()
        })
        .sum::<f64>()
        / record.controls.len() as f64;

    DroneControllerMetrics {
        family_name: case.family_name,
        trajectory_variant,
        controller_variant,
        generation_us: benchmark_generation(case, trajectory_variant, 128),
        total_duration_s: built.total_duration_s,
        tracking_rmse_m,
        max_position_error_m,
        mean_thrust_n,
        max_thrust_n,
        mean_torque_norm_nm,
    }
}

fn summarize_controller_variant(
    metrics: &[DroneControllerMetrics],
    controller_variant: DroneControllerVariant,
) -> DroneControllerSummary {
    let rows: Vec<_> = metrics
        .iter()
        .filter(|row| row.controller_variant == controller_variant)
        .collect();
    let denom = rows.len() as f64;

    DroneControllerSummary {
        controller_variant,
        mean_generation_us: rows.iter().map(|row| row.generation_us).sum::<f64>() / denom,
        mean_total_duration_s: rows.iter().map(|row| row.total_duration_s).sum::<f64>() / denom,
        mean_tracking_rmse_m: rows.iter().map(|row| row.tracking_rmse_m).sum::<f64>() / denom,
        mean_max_position_error_m: rows.iter().map(|row| row.max_position_error_m).sum::<f64>()
            / denom,
        mean_thrust_n: rows.iter().map(|row| row.mean_thrust_n).sum::<f64>() / denom,
        mean_max_thrust_n: rows.iter().map(|row| row.max_thrust_n).sum::<f64>() / denom,
        mean_torque_norm_nm: rows.iter().map(|row| row.mean_torque_norm_nm).sum::<f64>() / denom,
    }
}

fn benchmark_generation(
    case: &DroneTrajectoryCase,
    variant: DroneTrajectoryVariant,
    iterations: usize,
) -> f64 {
    let start = Instant::now();
    for _ in 0..iterations {
        black_box(build_variant_trajectory(case, variant));
    }
    start.elapsed().as_secs_f64() * 1_000_000.0 / iterations as f64
}

fn simulate_desired_states_with_controller_variant(
    start_position: Vector3<f64>,
    desired_states: &[DesiredState],
    config: &SimulationConfig,
    controller_variant: DroneControllerVariant,
) -> SimulationRecord {
    let mut state = QuadrotorState::new(start_position);
    let mut record = SimulationRecord {
        positions: vec![state.position],
        orientations: vec![state.orientation],
        controls: Vec::with_capacity(desired_states.len()),
    };

    for desired in desired_states {
        let control = compute_controller_variant(
            &state,
            desired,
            &config.params,
            &config.gains,
            controller_variant,
        );
        step_dynamics(&mut state, &control, &config.params, config.dt);
        record.positions.push(state.position);
        record.orientations.push(state.orientation);
        record.controls.push(control);
    }

    record
}

fn compute_controller_variant(
    state: &QuadrotorState,
    desired: &DesiredState,
    params: &QuadrotorParams,
    gains: &ControllerGains,
    controller_variant: DroneControllerVariant,
) -> ControlOutput {
    match controller_variant {
        DroneControllerVariant::BaselinePd => compute_control(state, desired, params, gains),
        DroneControllerVariant::AttitudeRateDampedPd => {
            compute_attitude_rate_damped_control(state, desired, params, gains)
        }
        DroneControllerVariant::AttitudeRateDampedLateralPd => {
            compute_attitude_rate_damped_lateral_control(state, desired, params, gains)
        }
    }
}

fn compute_attitude_rate_damped_control(
    state: &QuadrotorState,
    desired: &DesiredState,
    params: &QuadrotorParams,
    gains: &ControllerGains,
) -> ControlOutput {
    let mut control = compute_control(state, desired, params, gains);
    let rate_damping = Vector3::new(6.5, 6.5, 4.0);
    control.roll_torque -= rate_damping.x * state.angular_velocity.x;
    control.pitch_torque -= rate_damping.y * state.angular_velocity.y;
    control.yaw_torque -= rate_damping.z * state.angular_velocity.z;
    control
}

fn compute_attitude_rate_damped_lateral_control(
    state: &QuadrotorState,
    desired: &DesiredState,
    params: &QuadrotorParams,
    gains: &ControllerGains,
) -> ControlOutput {
    compute_attitude_rate_damped_lateral_control_with_gains(
        state,
        desired,
        params,
        gains,
        &LateralGainSet::default(),
    )
}

pub fn compute_attitude_rate_damped_lateral_control_with_gains(
    state: &QuadrotorState,
    desired: &DesiredState,
    params: &QuadrotorParams,
    gains: &ControllerGains,
    lateral: &LateralGainSet,
) -> ControlOutput {
    let g = params.gravity;
    let m = params.mass;
    let des_yaw = desired.yaw;
    let (sy, cy) = des_yaw.sin_cos();

    let position_error = desired.position - state.position;
    let velocity_error = desired.velocity - state.velocity;
    let lateral_feedback = Vector3::new(
        (lateral.lateral_position_gain * position_error.x
            + lateral.lateral_velocity_gain * velocity_error.x)
            .clamp(
                -lateral.max_lateral_correction,
                lateral.max_lateral_correction,
            ),
        (lateral.lateral_position_gain * position_error.y
            + lateral.lateral_velocity_gain * velocity_error.y)
            .clamp(
                -lateral.max_lateral_correction,
                lateral.max_lateral_correction,
            ),
        0.0,
    );
    let lateral_acceleration = desired.acceleration + lateral_feedback;

    let thrust = m
        * (g + desired.acceleration.z
            + gains.kp_pos.z * position_error.z
            + gains.kd_pos.z * velocity_error.z);
    let des_roll = ((lateral_acceleration.x * sy - lateral_acceleration.y * cy) / g)
        .clamp(-lateral.max_attitude_command, lateral.max_attitude_command);
    let des_pitch = ((lateral_acceleration.x * cy - lateral_acceleration.y * sy) / g)
        .clamp(-lateral.max_attitude_command, lateral.max_attitude_command);

    ControlOutput {
        thrust,
        roll_torque: gains.kp_att.x * (des_roll - state.orientation.x)
            - lateral.rate_damping.x * state.angular_velocity.x,
        pitch_torque: gains.kp_att.y * (des_pitch - state.orientation.y)
            - lateral.rate_damping.y * state.angular_velocity.y,
        yaw_torque: gains.kp_att.z * (des_yaw - state.orientation.z)
            - lateral.rate_damping.z * state.angular_velocity.z,
    }
}

fn build_variant_trajectory(
    case: &DroneTrajectoryCase,
    variant: DroneTrajectoryVariant,
) -> BuiltTrajectory {
    let segment_durations = match variant {
        DroneTrajectoryVariant::QuinticUniform => {
            vec![case.baseline_segment_duration; case.waypoints.len()]
        }
        DroneTrajectoryVariant::QuinticDistanceScaled
        | DroneTrajectoryVariant::MinimumSnapDistanceScaled => distance_scaled_durations(case),
    };

    match variant {
        DroneTrajectoryVariant::QuinticUniform | DroneTrajectoryVariant::QuinticDistanceScaled => {
            build_quintic_variant(case, &segment_durations)
        }
        DroneTrajectoryVariant::MinimumSnapDistanceScaled => {
            build_minimum_snap_variant(case, &segment_durations)
        }
    }
}

fn build_quintic_variant(case: &DroneTrajectoryCase, segment_durations: &[f64]) -> BuiltTrajectory {
    match case.boundary_mode {
        DroneBoundaryMode::StopGo => {
            let segments =
                generate_waypoint_trajectory_with_durations(&case.waypoints, segment_durations);
            build_quintic_bundle(&segments, segment_durations, case.dt)
        }
        DroneBoundaryMode::PassThrough
        | DroneBoundaryMode::PassThroughAccel
        | DroneBoundaryMode::PassThroughAccelJerk
        | DroneBoundaryMode::CoupledContinuity => {
            let boundary_profile = derive_boundary_profile(case, segment_durations);
            let segments =
                build_quintic_pass_through_segments(case, segment_durations, &boundary_profile);
            build_quintic_bundle(&segments, segment_durations, case.dt)
        }
    }
}

fn build_minimum_snap_variant(
    case: &DroneTrajectoryCase,
    segment_durations: &[f64],
) -> BuiltTrajectory {
    match case.boundary_mode {
        DroneBoundaryMode::StopGo => {
            let segments = generate_waypoint_minimum_snap_trajectory_with_durations(
                &case.waypoints,
                segment_durations,
            );
            build_minimum_snap_bundle(&segments, case.dt)
        }
        DroneBoundaryMode::PassThrough
        | DroneBoundaryMode::PassThroughAccel
        | DroneBoundaryMode::PassThroughAccelJerk
        | DroneBoundaryMode::CoupledContinuity => {
            let boundary_profile = derive_boundary_profile(case, segment_durations);
            let segments = build_minimum_snap_pass_through_segments(
                case,
                segment_durations,
                &boundary_profile,
            );
            build_minimum_snap_bundle(&segments, case.dt)
        }
    }
}

#[derive(Debug, Clone)]
struct WaypointBoundaryProfile {
    velocities: Vec<Vector3<f64>>,
    accelerations: Vec<Vector3<f64>>,
    jerks: Vec<Vector3<f64>>,
}

fn derive_boundary_profile(
    case: &DroneTrajectoryCase,
    segment_durations: &[f64],
) -> WaypointBoundaryProfile {
    match case.boundary_mode {
        DroneBoundaryMode::PassThrough
        | DroneBoundaryMode::PassThroughAccel
        | DroneBoundaryMode::PassThroughAccelJerk => {
            derive_local_pass_through_profile(case, segment_durations)
        }
        DroneBoundaryMode::CoupledContinuity => {
            derive_coupled_continuity_profile(case, segment_durations)
        }
        DroneBoundaryMode::StopGo => WaypointBoundaryProfile {
            velocities: vec![Vector3::zeros(); case.waypoints.len()],
            accelerations: vec![Vector3::zeros(); case.waypoints.len()],
            jerks: vec![Vector3::zeros(); case.waypoints.len()],
        },
    }
}

fn derive_local_pass_through_profile(
    case: &DroneTrajectoryCase,
    segment_durations: &[f64],
) -> WaypointBoundaryProfile {
    let n = case.waypoints.len();
    let mut velocities = vec![Vector3::zeros(); n];

    for index in 0..n {
        let prev = (index + n - 1) % n;
        let next = (index + 1) % n;
        let incoming = (case.waypoints[index] - case.waypoints[prev]) / segment_durations[prev];
        let outgoing = (case.waypoints[next] - case.waypoints[index]) / segment_durations[index];
        velocities[index] = 0.5 * case.tangent_scale * (incoming + outgoing);
    }

    let mut accelerations = vec![Vector3::zeros(); n];
    if matches!(
        case.boundary_mode,
        DroneBoundaryMode::PassThroughAccel | DroneBoundaryMode::PassThroughAccelJerk
    ) {
        for index in 0..n {
            let prev = (index + n - 1) % n;
            let avg_dt = 0.5 * (segment_durations[prev] + segment_durations[index]);
            accelerations[index] =
                case.acceleration_scale * (velocities[index] - velocities[prev]) / avg_dt;
        }
    }

    let mut jerks = vec![Vector3::zeros(); n];
    if matches!(case.boundary_mode, DroneBoundaryMode::PassThroughAccelJerk) {
        for index in 0..n {
            let prev = (index + n - 1) % n;
            let avg_dt = 0.5 * (segment_durations[prev] + segment_durations[index]);
            jerks[index] = case.jerk_scale * (accelerations[index] - accelerations[prev]) / avg_dt;
        }
    }

    WaypointBoundaryProfile {
        velocities,
        accelerations,
        jerks,
    }
}

fn derive_coupled_continuity_profile(
    case: &DroneTrajectoryCase,
    segment_durations: &[f64],
) -> WaypointBoundaryProfile {
    let mut velocities = solve_coupled_waypoint_velocities(&case.waypoints, segment_durations);
    for velocity in &mut velocities {
        *velocity *= case.tangent_scale;
    }

    let mut accelerations =
        derive_coupled_waypoint_accelerations(&case.waypoints, segment_durations, &velocities);
    for acceleration in &mut accelerations {
        *acceleration *= case.acceleration_scale;
    }

    let mut jerks = derive_coupled_waypoint_jerks(&case.waypoints, segment_durations, &velocities);
    for jerk in &mut jerks {
        *jerk *= case.jerk_scale;
    }

    WaypointBoundaryProfile {
        velocities,
        accelerations,
        jerks,
    }
}

fn solve_coupled_waypoint_velocities(
    waypoints: &[Vector3<f64>],
    segment_durations: &[f64],
) -> Vec<Vector3<f64>> {
    let n = waypoints.len();
    let mut system = DMatrix::zeros(n, n);
    let mut rhs = DMatrix::zeros(n, 3);

    for index in 0..n {
        let prev = (index + n - 1) % n;
        let next = (index + 1) % n;
        let h_prev = segment_durations[prev];
        let h_next = segment_durations[index];
        let delta_prev = waypoints[index] - waypoints[prev];
        let delta_next = waypoints[next] - waypoints[index];

        system[(index, prev)] = 2.0 / h_prev;
        system[(index, index)] = 4.0 / h_prev + 4.0 / h_next;
        system[(index, next)] = 2.0 / h_next;

        let boundary_rhs =
            6.0 * delta_prev / (h_prev * h_prev) + 6.0 * delta_next / (h_next * h_next);
        rhs[(index, 0)] = boundary_rhs.x;
        rhs[(index, 1)] = boundary_rhs.y;
        rhs[(index, 2)] = boundary_rhs.z;
    }

    let solution = system
        .lu()
        .solve(&rhs)
        .expect("Coupled continuity system should be invertible");

    (0..n)
        .map(|index| {
            Vector3::new(
                solution[(index, 0)],
                solution[(index, 1)],
                solution[(index, 2)],
            )
        })
        .collect()
}

fn derive_coupled_waypoint_accelerations(
    waypoints: &[Vector3<f64>],
    segment_durations: &[f64],
    velocities: &[Vector3<f64>],
) -> Vec<Vector3<f64>> {
    let n = waypoints.len();
    (0..n)
        .map(|index| {
            let prev = (index + n - 1) % n;
            let next = (index + 1) % n;
            let prev_delta = waypoints[index] - waypoints[prev];
            let next_delta = waypoints[next] - waypoints[index];
            let prev_end = cubic_segment_end_acceleration(
                prev_delta,
                segment_durations[prev],
                velocities[prev],
                velocities[index],
            );
            let next_start = cubic_segment_start_acceleration(
                next_delta,
                segment_durations[index],
                velocities[index],
                velocities[next],
            );
            0.5 * (prev_end + next_start)
        })
        .collect()
}

fn derive_coupled_waypoint_jerks(
    waypoints: &[Vector3<f64>],
    segment_durations: &[f64],
    velocities: &[Vector3<f64>],
) -> Vec<Vector3<f64>> {
    let n = waypoints.len();
    (0..n)
        .map(|index| {
            let prev = (index + n - 1) % n;
            let next = (index + 1) % n;
            let prev_jerk = cubic_segment_jerk(
                waypoints[index] - waypoints[prev],
                segment_durations[prev],
                velocities[prev],
                velocities[index],
            );
            let next_jerk = cubic_segment_jerk(
                waypoints[next] - waypoints[index],
                segment_durations[index],
                velocities[index],
                velocities[next],
            );
            0.5 * (prev_jerk + next_jerk)
        })
        .collect()
}

fn cubic_segment_start_acceleration(
    delta: Vector3<f64>,
    duration: f64,
    start_velocity: Vector3<f64>,
    end_velocity: Vector3<f64>,
) -> Vector3<f64> {
    6.0 * delta / (duration * duration) - (4.0 * start_velocity + 2.0 * end_velocity) / duration
}

fn cubic_segment_end_acceleration(
    delta: Vector3<f64>,
    duration: f64,
    start_velocity: Vector3<f64>,
    end_velocity: Vector3<f64>,
) -> Vector3<f64> {
    -6.0 * delta / (duration * duration) + (2.0 * start_velocity + 4.0 * end_velocity) / duration
}

fn cubic_segment_jerk(
    delta: Vector3<f64>,
    duration: f64,
    start_velocity: Vector3<f64>,
    end_velocity: Vector3<f64>,
) -> Vector3<f64> {
    -12.0 * delta / duration.powi(3) + 6.0 * (start_velocity + end_velocity) / duration.powi(2)
}

fn build_quintic_pass_through_segments(
    case: &DroneTrajectoryCase,
    segment_durations: &[f64],
    profile: &WaypointBoundaryProfile,
) -> Vec<TrajectorySegment> {
    let n = case.waypoints.len();
    (0..n)
        .map(|index| {
            let next = (index + 1) % n;
            generate_trajectory_segment_full(
                &case.waypoints[index],
                &case.waypoints[next],
                &profile.velocities[index],
                &profile.velocities[next],
                &profile.accelerations[index],
                &profile.accelerations[next],
                segment_durations[index],
            )
        })
        .collect()
}

fn build_minimum_snap_pass_through_segments(
    case: &DroneTrajectoryCase,
    segment_durations: &[f64],
    profile: &WaypointBoundaryProfile,
) -> Vec<MinimumSnapSegment> {
    let n = case.waypoints.len();
    (0..n)
        .map(|index| {
            let next = (index + 1) % n;
            let start = MinimumSnapBoundary {
                position: case.waypoints[index],
                velocity: profile.velocities[index],
                acceleration: profile.accelerations[index],
                jerk: profile.jerks[index],
            };
            let end = MinimumSnapBoundary {
                position: case.waypoints[next],
                velocity: profile.velocities[next],
                acceleration: profile.accelerations[next],
                jerk: profile.jerks[next],
            };
            generate_minimum_snap_segment_full(&start, &end, segment_durations[index])
        })
        .collect()
}

fn build_quintic_bundle(
    segments: &[TrajectorySegment],
    segment_durations: &[f64],
    dt: f64,
) -> BuiltTrajectory {
    let desired_states = sample_trajectory_segments(segments, segment_durations, dt);
    let mut jerk_norms = Vec::new();
    let mut snap_norms = Vec::new();

    for (segment, duration) in segments.iter().zip(segment_durations.iter().copied()) {
        let mut t = 0.0;
        while t <= duration {
            jerk_norms.push(segment.jerk(t).norm());
            snap_norms.push(segment.snap(t).norm());
            t += dt;
        }
    }

    BuiltTrajectory {
        desired_states,
        jerk_norms,
        snap_norms,
        total_duration_s: segment_durations.iter().sum(),
    }
}

fn build_minimum_snap_bundle(segments: &[MinimumSnapSegment], dt: f64) -> BuiltTrajectory {
    let desired_states = sample_minimum_snap_trajectory(segments, dt);
    let mut jerk_norms = Vec::new();
    let mut snap_norms = Vec::new();

    for segment in segments {
        let mut t = 0.0;
        while t <= segment.duration {
            jerk_norms.push(segment.jerk(t).norm());
            snap_norms.push(segment.snap(t).norm());
            t += dt;
        }
    }

    BuiltTrajectory {
        desired_states,
        jerk_norms,
        snap_norms,
        total_duration_s: segments.iter().map(|segment| segment.duration).sum(),
    }
}

fn distance_scaled_durations(case: &DroneTrajectoryCase) -> Vec<f64> {
    let mut raw: Vec<f64> = (0..case.waypoints.len())
        .map(|index| {
            let next = (index + 1) % case.waypoints.len();
            let distance = (case.waypoints[next] - case.waypoints[index]).norm();
            (distance / case.cruise_speed).max(case.min_segment_duration)
        })
        .collect();

    let target_total = case.baseline_segment_duration * case.waypoints.len() as f64;
    let raw_total: f64 = raw.iter().sum();
    let scale = target_total / raw_total;
    for duration in &mut raw {
        *duration *= scale;
    }
    raw
}

fn rms(values: &[f64]) -> f64 {
    let sum = values.iter().map(|value| value * value).sum::<f64>();
    (sum / values.len() as f64).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coupled_continuity_profile_preserves_c2_waypoint_acceleration() {
        let case = coupled_continuity_drone_trajectory_cases()
            .into_iter()
            .next()
            .expect("coupled continuity cases should exist");
        let segment_durations = distance_scaled_durations(&case);
        let profile = derive_coupled_continuity_profile(&case, &segment_durations);
        let n = case.waypoints.len();

        for index in 0..n {
            let prev = (index + n - 1) % n;
            let next = (index + 1) % n;
            let prev_end = cubic_segment_end_acceleration(
                case.waypoints[index] - case.waypoints[prev],
                segment_durations[prev],
                profile.velocities[prev],
                profile.velocities[index],
            );
            let next_start = cubic_segment_start_acceleration(
                case.waypoints[next] - case.waypoints[index],
                segment_durations[index],
                profile.velocities[index],
                profile.velocities[next],
            );

            assert!(
                (prev_end - next_start).norm() < 1e-8,
                "waypoint {index} lost coupled acceleration continuity: prev_end={prev_end:?}, next_start={next_start:?}"
            );
            assert!(
                (profile.accelerations[index] - prev_end).norm() < 1e-8,
                "stored acceleration should match the coupled continuity solution at waypoint {index}"
            );
        }
    }
}

/// Metrics for one case/variant/gain-set triple in a gain sensitivity sweep.
#[derive(Debug, Clone)]
pub struct GainSweepMetrics {
    pub family_name: &'static str,
    pub trajectory_variant: DroneTrajectoryVariant,
    pub gain_set: LateralGainSet,
    pub gain_label: String,
    pub tracking_rmse_m: f64,
    pub max_position_error_m: f64,
    pub mean_thrust_n: f64,
    pub max_thrust_n: f64,
    pub mean_torque_norm_nm: f64,
}

/// Run one case/variant with a specific lateral gain set.
pub fn evaluate_lateral_gain_case(
    case: &DroneTrajectoryCase,
    trajectory_variant: DroneTrajectoryVariant,
    lateral: &LateralGainSet,
    gain_label: &str,
) -> GainSweepMetrics {
    let built = build_variant_trajectory(case, trajectory_variant);
    let simulation_config = SimulationConfig {
        dt: case.dt,
        segment_duration: case.baseline_segment_duration,
        n_loops: 1,
        ..Default::default()
    };

    let mut state = QuadrotorState::new(case.waypoints[0]);
    let mut controls = Vec::with_capacity(built.desired_states.len());
    let mut squared_error_sum = 0.0;
    let mut max_position_error_m = 0.0_f64;

    for desired in &built.desired_states {
        let control = compute_attitude_rate_damped_lateral_control_with_gains(
            &state,
            desired,
            &simulation_config.params,
            &simulation_config.gains,
            lateral,
        );
        step_dynamics(
            &mut state,
            &control,
            &simulation_config.params,
            simulation_config.dt,
        );
        let error = (state.position - desired.position).norm();
        squared_error_sum += error * error;
        max_position_error_m = max_position_error_m.max(error);
        controls.push(control);
    }

    let tracking_rmse_m = (squared_error_sum / built.desired_states.len() as f64).sqrt();
    let mean_thrust_n = controls.iter().map(|c| c.thrust).sum::<f64>() / controls.len() as f64;
    let max_thrust_n = controls
        .iter()
        .map(|c| c.thrust)
        .fold(f64::NEG_INFINITY, f64::max);
    let mean_torque_norm_nm = controls
        .iter()
        .map(|c| Vector3::new(c.roll_torque, c.pitch_torque, c.yaw_torque).norm())
        .sum::<f64>()
        / controls.len() as f64;

    GainSweepMetrics {
        family_name: case.family_name,
        trajectory_variant,
        gain_set: *lateral,
        gain_label: gain_label.to_string(),
        tracking_rmse_m,
        max_position_error_m,
        mean_thrust_n,
        max_thrust_n,
        mean_torque_norm_nm,
    }
}
