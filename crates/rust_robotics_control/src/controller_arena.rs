//! Deterministic, headless comparison of classical path-tracking controllers.
//!
//! The arena deliberately separates simulation from visualization. Every
//! controller receives the same path, initial state, clock, speed response, and
//! turn-rate response. The returned samples are suitable for native tests,
//! browser replay, and future benchmark inputs.

use rust_robotics_core::{
    ControlInput, Path2D, PathTracker, Point2D, RoboticsError, RoboticsResult, State2D,
};

use crate::{
    LQRSteerConfig, LQRSteerController, PurePursuitConfig, PurePursuitController, StanleyConfig,
    StanleyController,
};

const DEFAULT_DT: f64 = 0.1;
const DEFAULT_MAX_LINEAR_SPEED: f64 = 8.0;
const DEFAULT_MAX_ANGULAR_SPEED: f64 = 1.4;
const DEFAULT_GOAL_TOLERANCE: f64 = 1.0;
const SPEED_RESPONSE_PER_SECOND: f64 = 1.8;
const MIN_CURVATURE_SPEED: f64 = 1e-6;

/// Controller implementations supported by Controller Arena version 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArenaControllerKind {
    PurePursuit,
    Stanley,
    LqrSteer,
}

impl ArenaControllerKind {
    pub const ALL: [Self; 3] = [Self::PurePursuit, Self::Stanley, Self::LqrSteer];

    pub fn label(self) -> &'static str {
        match self {
            Self::PurePursuit => "Pure Pursuit",
            Self::Stanley => "Stanley",
            Self::LqrSteer => "LQR Steer",
        }
    }
}

/// Built-in deterministic paths.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArenaPreset {
    StraightRecovery,
    SlalomRecovery,
    HairpinRecovery,
}

impl ArenaPreset {
    pub const ALL: [Self; 3] = [
        Self::StraightRecovery,
        Self::SlalomRecovery,
        Self::HairpinRecovery,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::StraightRecovery => "Straight recovery",
            Self::SlalomRecovery => "Slalom recovery",
            Self::HairpinRecovery => "Hairpin recovery",
        }
    }

    pub fn slug(self) -> &'static str {
        match self {
            Self::StraightRecovery => "straight",
            Self::SlalomRecovery => "slalom",
            Self::HairpinRecovery => "hairpin",
        }
    }

    pub fn from_slug(value: &str) -> Option<Self> {
        match value {
            "straight" => Some(Self::StraightRecovery),
            "slalom" => Some(Self::SlalomRecovery),
            "hairpin" => Some(Self::HairpinRecovery),
            _ => None,
        }
    }

    /// Creates this preset with user-adjustable common dynamics.
    pub fn scenario(self, target_speed: f64, turn_rate_response_gain: f64) -> ArenaScenario {
        let (path, max_steps) = match self {
            Self::StraightRecovery => (straight_path(), 220),
            Self::SlalomRecovery => (slalom_path(), 300),
            Self::HairpinRecovery => (hairpin_path(), 420),
        };
        let start = path.points[0];
        let start_yaw = path.yaw_profile().first().copied().unwrap_or(0.0);
        ArenaScenario {
            preset: self,
            path,
            initial_state: State2D::new(
                start.x - 2.0 * start_yaw.cos() - 1.5 * start_yaw.sin(),
                start.y - 2.0 * start_yaw.sin() + 1.5 * start_yaw.cos(),
                start_yaw + 0.12,
                0.5,
            ),
            dt: DEFAULT_DT,
            target_speed,
            turn_rate_response_gain,
            max_steps,
            max_linear_speed: DEFAULT_MAX_LINEAR_SPEED,
            max_angular_speed: DEFAULT_MAX_ANGULAR_SPEED,
            goal_tolerance: DEFAULT_GOAL_TOLERANCE,
        }
    }
}

/// Shared input to every arena controller.
#[derive(Debug, Clone)]
pub struct ArenaScenario {
    pub preset: ArenaPreset,
    pub path: Path2D,
    pub initial_state: State2D,
    pub dt: f64,
    pub target_speed: f64,
    /// Multiplier applied to the requested angular velocity; 1.0 is ideal.
    pub turn_rate_response_gain: f64,
    pub max_steps: usize,
    pub max_linear_speed: f64,
    pub max_angular_speed: f64,
    pub goal_tolerance: f64,
}

impl ArenaScenario {
    fn validate(&self) -> RoboticsResult<()> {
        if self.path.len() < 2 {
            return Err(invalid("arena path must contain at least two points"));
        }
        if !self
            .path
            .points
            .iter()
            .all(|point| point.x.is_finite() && point.y.is_finite())
        {
            return Err(invalid("arena path coordinates must be finite"));
        }
        for (name, value) in [
            ("dt", self.dt),
            ("target_speed", self.target_speed),
            ("turn_rate_response_gain", self.turn_rate_response_gain),
            ("max_linear_speed", self.max_linear_speed),
            ("max_angular_speed", self.max_angular_speed),
            ("goal_tolerance", self.goal_tolerance),
        ] {
            if !value.is_finite() {
                return Err(invalid(&format!("{name} must be finite")));
            }
        }
        if self.dt <= 0.0
            || self.target_speed <= 0.0
            || !(0.0..=1.0).contains(&self.turn_rate_response_gain)
            || self.max_steps == 0
            || self.max_linear_speed <= 0.0
            || self.max_angular_speed <= 0.0
            || self.goal_tolerance <= 0.0
        {
            return Err(invalid(
                "arena dynamics require positive limits and a response gain in 0..=1",
            ));
        }
        if !state_is_finite(self.initial_state) {
            return Err(invalid("arena initial state must be finite"));
        }
        Ok(())
    }
}

/// One simulation sample. The initial sample uses a zero command at `time = 0`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArenaSample {
    pub time: f64,
    pub state: State2D,
    pub command: ControlInput,
    pub cross_track_error: f64,
}

/// Metrics derived exclusively from an [`ArenaRun`]'s samples.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArenaMetrics {
    pub cross_track_rmse: f64,
    pub final_goal_distance: f64,
    pub max_cross_track_error: f64,
    pub angular_command_smoothness: f64,
}

/// Complete deterministic trace for one controller.
#[derive(Debug, Clone, PartialEq)]
pub struct ArenaRun {
    pub controller: ArenaControllerKind,
    pub samples: Vec<ArenaSample>,
    pub metrics: ArenaMetrics,
}

/// Runs every requested controller under one shared scenario.
pub fn run_controller_arena(
    scenario: &ArenaScenario,
    controllers: &[ArenaControllerKind],
) -> RoboticsResult<Vec<ArenaRun>> {
    scenario.validate()?;
    if controllers.is_empty() {
        return Err(invalid("at least one arena controller is required"));
    }
    controllers
        .iter()
        .copied()
        .map(|kind| run_one(scenario, kind))
        .collect()
}

/// Recomputes metrics from samples, providing one source of truth for tests and
/// alternate front ends.
pub fn metrics_from_samples(
    samples: &[ArenaSample],
    goal: Point2D,
) -> RoboticsResult<ArenaMetrics> {
    if samples.is_empty() {
        return Err(invalid("arena metrics require at least one sample"));
    }
    if !samples.iter().all(sample_is_finite) {
        return Err(RoboticsError::NumericalError(
            "arena samples contain a non-finite value".to_string(),
        ));
    }

    let squared_error_sum = samples
        .iter()
        .map(|sample| sample.cross_track_error * sample.cross_track_error)
        .sum::<f64>();
    let max_cross_track_error = samples
        .iter()
        .map(|sample| sample.cross_track_error.abs())
        .fold(0.0_f64, f64::max);
    let squared_command_delta_sum = samples
        .windows(2)
        .map(|pair| {
            let delta = pair[1].command.omega - pair[0].command.omega;
            delta * delta
        })
        .sum::<f64>();
    let command_delta_count = samples.len().saturating_sub(1).max(1);
    let final_state = samples
        .last()
        .expect("non-empty samples were checked")
        .state;

    Ok(ArenaMetrics {
        cross_track_rmse: (squared_error_sum / samples.len() as f64).sqrt(),
        final_goal_distance: final_state.position().distance(&goal),
        max_cross_track_error,
        angular_command_smoothness: (squared_command_delta_sum / command_delta_count as f64).sqrt(),
    })
}

fn run_one(scenario: &ArenaScenario, kind: ArenaControllerKind) -> RoboticsResult<ArenaRun> {
    let mut tracker = build_tracker(kind, scenario);
    let goal = *scenario
        .path
        .points
        .last()
        .expect("validated path has at least two points");
    let mut state = scenario.initial_state;
    let mut samples = Vec::with_capacity(scenario.max_steps + 1);
    samples.push(ArenaSample {
        time: 0.0,
        state,
        command: ControlInput::zero(),
        cross_track_error: cross_track_error(&scenario.path, state),
    });

    for step in 1..=scenario.max_steps {
        let requested = tracker.compute_control(&state, &scenario.path);
        if !requested.v.is_finite() || !requested.omega.is_finite() {
            return Err(RoboticsError::NumericalError(format!(
                "{} returned a non-finite command",
                kind.label()
            )));
        }

        // Controllers contribute steering curvature. Longitudinal response is
        // owned by the arena so all controllers receive the same speed model.
        let curvature = if requested.v.abs() > MIN_CURVATURE_SPEED {
            requested.omega / requested.v
        } else {
            0.0
        };
        let speed_alpha = (SPEED_RESPONSE_PER_SECOND * scenario.dt).clamp(0.0, 1.0);
        let v = (state.v + (scenario.target_speed - state.v) * speed_alpha)
            .clamp(0.0, scenario.max_linear_speed);
        let omega = (curvature * v * scenario.turn_rate_response_gain)
            .clamp(-scenario.max_angular_speed, scenario.max_angular_speed);
        let command = ControlInput::new(v, omega);
        state = propagate(state, command, scenario.dt);
        let sample = ArenaSample {
            time: step as f64 * scenario.dt,
            state,
            command,
            cross_track_error: cross_track_error(&scenario.path, state),
        };
        if !sample_is_finite(&sample) {
            return Err(RoboticsError::NumericalError(format!(
                "{} produced a non-finite state",
                kind.label()
            )));
        }
        samples.push(sample);

        if state.position().distance(&goal) <= scenario.goal_tolerance {
            break;
        }
    }

    let metrics = metrics_from_samples(&samples, goal)?;
    Ok(ArenaRun {
        controller: kind,
        samples,
        metrics,
    })
}

fn build_tracker(kind: ArenaControllerKind, scenario: &ArenaScenario) -> Box<dyn PathTracker> {
    match kind {
        ArenaControllerKind::PurePursuit => {
            Box::new(PurePursuitController::new(PurePursuitConfig {
                look_ahead_gain: 0.12,
                look_ahead_distance: 2.4,
                wheelbase: 2.9,
                kp: 1.0,
                goal_threshold: scenario.goal_tolerance,
            }))
        }
        ArenaControllerKind::Stanley => Box::new(StanleyController::new(StanleyConfig {
            k: 0.55,
            wheelbase: 2.9,
            kp: 1.0,
            goal_threshold: scenario.goal_tolerance,
        })),
        ArenaControllerKind::LqrSteer => {
            let mut tracker = LQRSteerController::new(LQRSteerConfig {
                wheelbase: 2.9,
                max_steer: 45.0_f64.to_radians(),
                kp: 1.0,
                q_diag: [1.0, 1.0, 1.0, 1.0],
                r: 1.0,
                dt: scenario.dt,
                goal_threshold: scenario.goal_tolerance,
            });
            tracker.set_path_with_speed(scenario.path.clone(), scenario.target_speed);
            Box::new(tracker)
        }
    }
}

fn propagate(state: State2D, command: ControlInput, dt: f64) -> State2D {
    let x = state.x + command.v * state.yaw.cos() * dt;
    let y = state.y + command.v * state.yaw.sin() * dt;
    let yaw = normalize_angle(state.yaw + command.omega * dt);
    State2D::new(x, y, yaw, command.v)
}

fn normalize_angle(mut angle: f64) -> f64 {
    while angle > std::f64::consts::PI {
        angle -= 2.0 * std::f64::consts::PI;
    }
    while angle < -std::f64::consts::PI {
        angle += 2.0 * std::f64::consts::PI;
    }
    angle
}

fn cross_track_error(path: &Path2D, state: State2D) -> f64 {
    let position = state.position();
    path.nearest_point_index(position)
        .map(|index| position.distance(&path.points[index]))
        .unwrap_or(f64::INFINITY)
}

fn straight_path() -> Path2D {
    Path2D::from_points(
        (0..=100)
            .map(|index| Point2D::new(index as f64 * 0.5, 0.0))
            .collect(),
    )
}

fn slalom_path() -> Path2D {
    Path2D::from_points(
        (0..=120)
            .map(|index| {
                let x = index as f64 * 0.5;
                Point2D::new(x, 3.0 * (x / 7.0).sin())
            })
            .collect(),
    )
}

fn hairpin_path() -> Path2D {
    let mut points = Vec::new();
    for index in 0..=40 {
        points.push(Point2D::new(index as f64 * 0.5, 0.0));
    }
    let radius = 6.0;
    for index in 1..=38 {
        let theta = -std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * index as f64 / 38.0;
        points.push(Point2D::new(
            20.0 + radius * theta.cos(),
            6.0 + radius * theta.sin(),
        ));
    }
    for index in 1..=40 {
        points.push(Point2D::new(20.0 - index as f64 * 0.5, 12.0));
    }
    Path2D::from_points(points)
}

fn state_is_finite(state: State2D) -> bool {
    state.x.is_finite() && state.y.is_finite() && state.yaw.is_finite() && state.v.is_finite()
}

fn sample_is_finite(sample: &ArenaSample) -> bool {
    sample.time.is_finite()
        && state_is_finite(sample.state)
        && sample.command.v.is_finite()
        && sample.command.omega.is_finite()
        && sample.cross_track_error.is_finite()
}

fn invalid(message: &str) -> RoboticsError {
    RoboticsError::InvalidParameter(message.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-12;

    #[test]
    fn presets_are_finite_and_continuous() {
        for preset in ArenaPreset::ALL {
            let scenario = preset.scenario(3.0, 0.85);
            scenario.validate().unwrap();
            assert!(scenario.path.len() >= 100);
            assert!(scenario.path.total_length() > 45.0);
            assert!(scenario.path.points.windows(2).all(|pair| {
                let spacing = pair[0].distance(&pair[1]);
                spacing > 0.0 && spacing < 0.7
            }));
        }
    }

    #[test]
    fn every_controller_runs_every_preset_with_shared_initial_state() {
        for preset in ArenaPreset::ALL {
            let scenario = preset.scenario(3.0, 0.85);
            let runs = run_controller_arena(&scenario, &ArenaControllerKind::ALL).unwrap();
            assert_eq!(runs.len(), ArenaControllerKind::ALL.len());
            for run in runs {
                assert!(
                    run.samples.len() > 1,
                    "{} has no trace",
                    run.controller.label()
                );
                assert_eq!(run.samples[0].state, scenario.initial_state);
                assert!(run.samples.iter().all(sample_is_finite));
                assert!(run.samples.iter().all(|sample| {
                    sample.command.v.abs() <= scenario.max_linear_speed + TOLERANCE
                        && sample.command.omega.abs() <= scenario.max_angular_speed + TOLERANCE
                }));
                assert!(metrics_are_finite(run.metrics));
            }
        }
    }

    #[test]
    fn repeated_runs_are_deterministic() {
        for preset in ArenaPreset::ALL {
            let scenario = preset.scenario(2.75, 0.72);
            let first = run_controller_arena(&scenario, &ArenaControllerKind::ALL).unwrap();
            let second = run_controller_arena(&scenario, &ArenaControllerKind::ALL).unwrap();
            assert_eq!(first, second);
        }
    }

    #[test]
    fn stored_metrics_match_recomputation() {
        for preset in ArenaPreset::ALL {
            let scenario = preset.scenario(3.5, 1.0);
            let goal = *scenario.path.points.last().unwrap();
            for run in run_controller_arena(&scenario, &ArenaControllerKind::ALL).unwrap() {
                let recomputed = metrics_from_samples(&run.samples, goal).unwrap();
                assert_metrics_close(run.metrics, recomputed);
            }
        }
    }

    #[test]
    fn invalid_scenarios_and_empty_controller_lists_are_rejected() {
        let mut scenario = ArenaPreset::StraightRecovery.scenario(3.0, 1.0);
        assert!(run_controller_arena(&scenario, &[]).is_err());
        scenario.turn_rate_response_gain = 1.1;
        assert!(run_controller_arena(&scenario, &ArenaControllerKind::ALL).is_err());
        scenario = ArenaPreset::StraightRecovery.scenario(3.0, 1.0);
        scenario.path = Path2D::new();
        assert!(run_controller_arena(&scenario, &ArenaControllerKind::ALL).is_err());
    }

    fn metrics_are_finite(metrics: ArenaMetrics) -> bool {
        metrics.cross_track_rmse.is_finite()
            && metrics.final_goal_distance.is_finite()
            && metrics.max_cross_track_error.is_finite()
            && metrics.angular_command_smoothness.is_finite()
    }

    fn assert_metrics_close(left: ArenaMetrics, right: ArenaMetrics) {
        assert!((left.cross_track_rmse - right.cross_track_rmse).abs() <= TOLERANCE);
        assert!((left.final_goal_distance - right.final_goal_distance).abs() <= TOLERANCE);
        assert!((left.max_cross_track_error - right.max_cross_track_error).abs() <= TOLERANCE);
        assert!(
            (left.angular_command_smoothness - right.angular_command_smoothness).abs() <= TOLERANCE
        );
    }
}
