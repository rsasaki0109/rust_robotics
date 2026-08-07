//! Meta-Control: deterministic controller mode selection.
//!
//! A mode selector drives the shared Controller Arena engine through a
//! [`PathTracker`] facade and switches between Pure Pursuit, Stanley, and LQR
//! Steer mid-run according to a deterministic policy. Each sub-controller
//! keeps its own internal state, so a switch resumes the selected controller
//! exactly where its own history left off — the selector is evaluated against
//! the same shared simulation clock, speed model, and turn-rate response as the
//! arena's fixed controllers.
//!
//! This is the differentiator the repository can show that few others can: all
//! controllers live under one `PathTracker` contract, so a *meta* controller
//! that composes them is just another implementor of the same trait.

use crate::controller_arena::{
    apply_shared_response, build_tracker, cross_track_error, invalid, metrics_from_samples,
    normalize_angle, propagate, state_is_finite, ArenaControllerKind, ArenaMetrics, ArenaSample,
    ArenaScenario,
};
use rust_robotics_core::{
    ControlInput, Path2D, PathTracker, Point2D, RoboticsError, RoboticsResult, State2D,
};

/// Deterministic rule that picks which controller drives the current step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MetaControlPolicy {
    /// Never switch: reproduces a single fixed controller (baseline).
    Fixed(ArenaControllerKind),
    /// Use `error_mode` while the cross-track error exceeds the threshold,
    /// otherwise `cruise_mode`.
    SwitchOnError {
        error_threshold: f64,
        error_mode: ArenaControllerKind,
        cruise_mode: ArenaControllerKind,
    },
    /// Use `tight_mode` while the local path curvature exceeds the threshold,
    /// otherwise `smooth_mode`.
    SwitchOnCurvature {
        curvature_threshold: f64,
        tight_mode: ArenaControllerKind,
        smooth_mode: ArenaControllerKind,
    },
}

impl MetaControlPolicy {
    pub fn label(self) -> &'static str {
        match self {
            MetaControlPolicy::Fixed(kind) => match kind {
                ArenaControllerKind::PurePursuit => "fixed: pure pursuit",
                ArenaControllerKind::Stanley => "fixed: stanley",
                ArenaControllerKind::LqrSteer => "fixed: lqr steer",
            },
            MetaControlPolicy::SwitchOnError { .. } => "meta: switch on error",
            MetaControlPolicy::SwitchOnCurvature { .. } => "meta: switch on curvature",
        }
    }

    pub fn validate(self) -> RoboticsResult<()> {
        match self {
            MetaControlPolicy::Fixed(_) => Ok(()),
            MetaControlPolicy::SwitchOnError {
                error_threshold, ..
            } => {
                if !error_threshold.is_finite() || error_threshold <= 0.0 {
                    Err(invalid(
                        "switch-on-error threshold must be finite and positive",
                    ))
                } else {
                    Ok(())
                }
            }
            MetaControlPolicy::SwitchOnCurvature {
                curvature_threshold,
                ..
            } => {
                if !curvature_threshold.is_finite() || curvature_threshold <= 0.0 {
                    Err(invalid(
                        "switch-on-curvature threshold must be finite and positive",
                    ))
                } else {
                    Ok(())
                }
            }
        }
    }
}

/// A [`PathTracker`] that composes the three arena controllers and selects one
/// per step. Holds one tracker instance per controller kind so internal state
/// survives mode switches.
pub struct MetaController {
    policy: MetaControlPolicy,
    goal_tolerance: f64,
    trackers: [Box<dyn PathTracker>; 3],
    active: Option<ArenaControllerKind>,
}

impl MetaController {
    pub fn new(scenario: &ArenaScenario, policy: MetaControlPolicy) -> RoboticsResult<Self> {
        policy.validate()?;
        scenario.validate()?;
        Ok(Self {
            policy,
            goal_tolerance: scenario.goal_tolerance,
            trackers: ArenaControllerKind::ALL.map(|kind| build_tracker(kind, scenario)),
            active: None,
        })
    }

    pub fn policy(&self) -> MetaControlPolicy {
        self.policy
    }

    /// The mode chosen for the most recent [`PathTracker::compute_control`]
    /// call, if any.
    pub fn active_mode(&self) -> Option<ArenaControllerKind> {
        self.active
    }

    /// Pure selection logic, exposed so callers can record or test the policy
    /// without advancing any controller state.
    pub fn select_mode(&self, state: &State2D, path: &Path2D) -> ArenaControllerKind {
        match self.policy {
            MetaControlPolicy::Fixed(kind) => kind,
            MetaControlPolicy::SwitchOnError {
                error_threshold,
                error_mode,
                cruise_mode,
            } => {
                let error = cross_track_error(path, *state);
                if error > error_threshold {
                    error_mode
                } else {
                    cruise_mode
                }
            }
            MetaControlPolicy::SwitchOnCurvature {
                curvature_threshold,
                tight_mode,
                smooth_mode,
            } => {
                let curvature = path_curvature_at(path, state);
                if curvature > curvature_threshold {
                    tight_mode
                } else {
                    smooth_mode
                }
            }
        }
    }

    fn tracker_for(&mut self, kind: ArenaControllerKind) -> &mut Box<dyn PathTracker> {
        &mut self.trackers[kind_index(kind)]
    }
}

impl PathTracker for MetaController {
    fn compute_control(&mut self, current_state: &State2D, path: &Path2D) -> ControlInput {
        let kind = self.select_mode(current_state, path);
        self.active = Some(kind);
        self.tracker_for(kind).compute_control(current_state, path)
    }

    fn is_goal_reached(&self, current_state: &State2D, goal: Point2D) -> bool {
        current_state.position().distance(&goal) < self.goal_tolerance
    }
}

/// One meta-control simulation sample. Identical to [`ArenaSample`] plus the
/// controller mode that produced the command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MetaControlSample {
    pub time: f64,
    pub state: State2D,
    pub command: ControlInput,
    pub cross_track_error: f64,
    pub active_mode: ArenaControllerKind,
}

/// Complete deterministic trace for one meta-control policy.
#[derive(Debug, Clone, PartialEq)]
pub struct MetaControlRun {
    pub policy: MetaControlPolicy,
    pub samples: Vec<MetaControlSample>,
    pub metrics: ArenaMetrics,
    pub switch_count: usize,
}

impl MetaControlRun {
    /// Number of samples spent in each controller mode, indexed by
    /// [`ArenaControllerKind`] order.
    pub fn mode_counts(&self) -> [usize; 3] {
        let mut counts = [0usize; 3];
        for sample in &self.samples {
            counts[kind_index(sample.active_mode)] += 1;
        }
        counts
    }

    /// Fraction of samples spent in `mode`.
    pub fn mode_share(&self, mode: ArenaControllerKind) -> f64 {
        let total = self.samples.len().max(1) as f64;
        let used = self
            .samples
            .iter()
            .filter(|sample| sample.active_mode == mode)
            .count() as f64;
        used / total
    }
}

/// Runs one meta-control policy under a shared arena scenario.
pub fn run_meta_control(
    scenario: &ArenaScenario,
    policy: MetaControlPolicy,
) -> RoboticsResult<MetaControlRun> {
    policy.validate()?;
    scenario.validate()?;
    let goal = *scenario
        .path
        .points
        .last()
        .ok_or_else(|| invalid("meta-control path must contain at least two points"))?;

    let mut controller = MetaController::new(scenario, policy)?;
    let mut state = scenario.initial_state;
    let initial_mode = controller.select_mode(&state, &scenario.path);
    let mut samples = Vec::with_capacity(scenario.max_steps + 1);
    samples.push(MetaControlSample {
        time: 0.0,
        state,
        command: ControlInput::zero(),
        cross_track_error: cross_track_error(&scenario.path, state),
        active_mode: initial_mode,
    });

    let mut switches = 0usize;
    let mut previous_mode: Option<ArenaControllerKind> = None;
    for step in 1..=scenario.max_steps {
        let requested = controller.compute_control(&state, &scenario.path);
        let mode = controller
            .active_mode()
            .expect("compute_control always selects a mode");
        if !requested.v.is_finite() || !requested.omega.is_finite() {
            return Err(RoboticsError::NumericalError(format!(
                "{} returned a non-finite command",
                mode.label()
            )));
        }

        if let Some(previous) = previous_mode {
            if previous != mode {
                switches += 1;
            }
        }
        previous_mode = Some(mode);

        let command = apply_shared_response(scenario, state, requested);
        state = propagate(state, command, scenario.dt);
        let sample = MetaControlSample {
            time: step as f64 * scenario.dt,
            state,
            command,
            cross_track_error: cross_track_error(&scenario.path, state),
            active_mode: mode,
        };
        if !meta_sample_is_finite(&sample) {
            return Err(RoboticsError::NumericalError(format!(
                "{} produced a non-finite state",
                mode.label()
            )));
        }
        samples.push(sample);

        if state.position().distance(&goal) <= scenario.goal_tolerance {
            break;
        }
    }

    let metrics = metrics_from_samples(&to_arena_samples(&samples), goal)?;
    Ok(MetaControlRun {
        policy,
        samples,
        metrics,
        switch_count: switches,
    })
}

/// Recomputes metrics from meta-control samples (same source of truth as the
/// arena).
pub fn meta_metrics_from_samples(
    samples: &[MetaControlSample],
    goal: Point2D,
) -> RoboticsResult<ArenaMetrics> {
    metrics_from_samples(&to_arena_samples(samples), goal)
}

fn to_arena_samples(samples: &[MetaControlSample]) -> Vec<ArenaSample> {
    samples
        .iter()
        .map(|sample| ArenaSample {
            time: sample.time,
            state: sample.state,
            command: sample.command,
            cross_track_error: sample.cross_track_error,
        })
        .collect()
}

fn kind_index(kind: ArenaControllerKind) -> usize {
    match kind {
        ArenaControllerKind::PurePursuit => 0,
        ArenaControllerKind::Stanley => 1,
        ArenaControllerKind::LqrSteer => 2,
    }
}

fn meta_sample_is_finite(sample: &MetaControlSample) -> bool {
    sample.time.is_finite()
        && state_is_finite(sample.state)
        && sample.command.v.is_finite()
        && sample.command.omega.is_finite()
        && sample.cross_track_error.is_finite()
}

/// Magnitude of local path curvature (rad/m) at the robot's nearest path point.
fn path_curvature_at(path: &Path2D, state: &State2D) -> f64 {
    let Some(index) = path.nearest_point_index(state.position()) else {
        return 0.0;
    };
    let yaw = path.yaw_profile();
    let n = yaw.len();
    if n < 3 {
        return 0.0;
    }
    let i = index.clamp(1, n - 2);
    let d1 = normalize_angle(yaw[i + 1] - yaw[i]);
    let d2 = normalize_angle(yaw[i] - yaw[i - 1]);
    let ds = path.points[i].distance(&path.points[i - 1]).max(1e-3);
    (d1.abs() + d2.abs()).min(core::f64::consts::PI) / (2.0 * ds)
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-12;

    fn assert_metrics_close(left: ArenaMetrics, right: ArenaMetrics) {
        assert!((left.cross_track_rmse - right.cross_track_rmse).abs() <= TOLERANCE);
        assert!((left.final_goal_distance - right.final_goal_distance).abs() <= TOLERANCE);
        assert!((left.max_cross_track_error - right.max_cross_track_error).abs() <= TOLERANCE);
        assert!(
            (left.angular_command_smoothness - right.angular_command_smoothness).abs() <= TOLERANCE
        );
    }

    #[test]
    fn fixed_policy_reproduces_arena_run() {
        for preset in crate::controller_arena::ArenaPreset::ALL {
            let scenario = preset.scenario(3.0, 0.85);
            for kind in ArenaControllerKind::ALL {
                let arena_run = crate::run_controller_arena(&scenario, &[kind])
                    .unwrap()
                    .pop()
                    .unwrap();
                let meta_run = run_meta_control(&scenario, MetaControlPolicy::Fixed(kind)).unwrap();
                assert_eq!(meta_run.samples.len(), arena_run.samples.len());
                for (meta, arena) in meta_run.samples.iter().zip(arena_run.samples.iter()) {
                    assert_eq!(meta.state, arena.state);
                    assert_eq!(meta.command, arena.command);
                    assert!((meta.cross_track_error - arena.cross_track_error).abs() <= TOLERANCE);
                }
                assert_metrics_close(meta_run.metrics, arena_run.metrics);
                assert_eq!(meta_run.switch_count, 0);
            }
        }
    }

    #[test]
    fn error_policy_switches_to_correction_mode_then_cruises() {
        let scenario = crate::controller_arena::ArenaPreset::StraightRecovery.scenario(3.0, 0.85);
        let policy = MetaControlPolicy::SwitchOnError {
            error_threshold: 0.5,
            error_mode: ArenaControllerKind::Stanley,
            cruise_mode: ArenaControllerKind::PurePursuit,
        };
        let run = run_meta_control(&scenario, policy).unwrap();
        // The preset starts ~2.5 m off the path, so Stanley (error_mode) must be
        // active first, then the selector switches to Pure Pursuit (cruise_mode)
        // once the cross-track error is under the threshold.
        assert_eq!(run.samples[0].active_mode, ArenaControllerKind::Stanley);
        assert!(run.mode_counts()[1] > 0, "Stanley never used");
        assert!(run.mode_counts()[0] > 0, "Pure Pursuit never used");
        assert!(run.switch_count >= 1);
        assert!(run.samples.last().unwrap().cross_track_error.abs() < 0.5);
    }

    #[test]
    fn curvature_policy_uses_tight_mode_on_hairpin_arc() {
        let scenario = crate::controller_arena::ArenaPreset::HairpinRecovery.scenario(3.0, 0.85);
        let policy = MetaControlPolicy::SwitchOnCurvature {
            curvature_threshold: 0.10,
            tight_mode: ArenaControllerKind::LqrSteer,
            smooth_mode: ArenaControllerKind::PurePursuit,
        };
        let run = run_meta_control(&scenario, policy).unwrap();
        // Hairpin arc radius is 6 m -> ~0.17 rad/m, above the 0.10 threshold;
        // straights are ~0, so both modes must appear.
        assert!(run.mode_counts()[2] > 0, "LQR Steer never used on the arc");
        assert!(
            run.mode_counts()[0] > 0,
            "Pure Pursuit never used on straights"
        );
        assert!(run.switch_count >= 1);
    }

    #[test]
    fn repeated_runs_are_deterministic() {
        let scenario = crate::controller_arena::ArenaPreset::SlalomRecovery.scenario(2.75, 0.72);
        let policy = MetaControlPolicy::SwitchOnError {
            error_threshold: 0.4,
            error_mode: ArenaControllerKind::Stanley,
            cruise_mode: ArenaControllerKind::PurePursuit,
        };
        let first = run_meta_control(&scenario, policy).unwrap();
        let second = run_meta_control(&scenario, policy).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn invalid_policies_are_rejected() {
        let scenario = crate::controller_arena::ArenaPreset::StraightRecovery.scenario(3.0, 1.0);
        assert!(run_meta_control(
            &scenario,
            MetaControlPolicy::SwitchOnError {
                error_threshold: 0.0,
                error_mode: ArenaControllerKind::Stanley,
                cruise_mode: ArenaControllerKind::PurePursuit,
            }
        )
        .is_err());
        assert!(run_meta_control(
            &scenario,
            MetaControlPolicy::SwitchOnCurvature {
                curvature_threshold: f64::NAN,
                tight_mode: ArenaControllerKind::LqrSteer,
                smooth_mode: ArenaControllerKind::PurePursuit,
            }
        )
        .is_err());
    }

    #[test]
    fn switch_count_matches_adjacent_mode_changes() {
        let scenario = crate::controller_arena::ArenaPreset::SlalomRecovery.scenario(3.0, 0.85);
        let policy = MetaControlPolicy::SwitchOnCurvature {
            curvature_threshold: 0.05,
            tight_mode: ArenaControllerKind::Stanley,
            smooth_mode: ArenaControllerKind::LqrSteer,
        };
        let run = run_meta_control(&scenario, policy).unwrap();
        let adjacent = run
            .samples
            .windows(2)
            .filter(|pair| pair[0].active_mode != pair[1].active_mode)
            .count();
        assert_eq!(run.switch_count, adjacent);
    }

    #[test]
    fn stored_metrics_match_recomputation() {
        let scenario = crate::controller_arena::ArenaPreset::StraightRecovery.scenario(3.5, 1.0);
        let goal = *scenario.path.points.last().unwrap();
        let policy = MetaControlPolicy::SwitchOnError {
            error_threshold: 0.5,
            error_mode: ArenaControllerKind::Stanley,
            cruise_mode: ArenaControllerKind::PurePursuit,
        };
        let run = run_meta_control(&scenario, policy).unwrap();
        let recomputed = meta_metrics_from_samples(&run.samples, goal).unwrap();
        assert_metrics_close(run.metrics, recomputed);
    }
}
