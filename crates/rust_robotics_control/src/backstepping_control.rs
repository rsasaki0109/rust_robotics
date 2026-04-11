//! Backstepping-style controller for a unicycle robot.
//!
//! Uses a recursive Lyapunov-based control law in the robot body frame to
//! drive the robot to a target pose.
//!
//! References:
//! - Claude Samson, "Control of chained systems application to path following
//!   and time-varying point-stabilization of mobile robots":
//!   <https://ieeexplore.ieee.org/document/362899>
//! - R. W. Brockett, "Asymptotic Stability and Feedback Stabilization":
//!   <https://hrl.harvard.edu/publications/brockett83asymptotic.pdf>
//!
//! Notes:
//! A stationary unicycle posture cannot in general be asymptotically
//! stabilized by smooth time-invariant pure-state feedback alone. To keep the
//! API posture-regulation oriented while preserving robust convergence in
//! practice, this module uses the backstepping tracking-form control law away
//! from the goal and a terminal pose regulator close to the target.

use rust_robotics_core::{ControlInput, Path2D, Point2D, Pose2D};
use std::f64::consts::PI;

/// Configuration for the backstepping controller.
#[derive(Debug, Clone, Copy)]
pub struct BacksteppingConfig {
    /// Position gain.
    pub k1: f64,
    /// Lateral error gain.
    pub k2: f64,
    /// Heading error gain.
    pub k3: f64,
    /// Integration time-step \[s\].
    pub dt: f64,
    /// Convergence tolerance for position \[m\] and yaw \[rad\].
    pub goal_tolerance: f64,
    /// Maximum number of simulation steps.
    pub max_steps: usize,
}

impl Default for BacksteppingConfig {
    fn default() -> Self {
        Self {
            k1: 3.0,
            k2: 8.0,
            k3: 3.0,
            dt: 0.01,
            goal_tolerance: 0.05,
            max_steps: 10_000,
        }
    }
}

/// Configuration for the time-varying pose stabilizer.
#[derive(Debug, Clone, Copy)]
pub struct TimeVaryingBacksteppingConfig {
    /// Base gains and integration settings shared with the hybrid controller.
    pub base: BacksteppingConfig,
    /// Initial orbit radius as a multiple of the start-goal distance.
    pub orbit_radius_scale: f64,
    /// Angular speed of the virtual goal orbit \[rad/s\].
    pub orbit_frequency: f64,
    /// Exponential decay rate for the virtual goal orbit \[1/s\].
    pub orbit_decay_rate: f64,
}

impl Default for TimeVaryingBacksteppingConfig {
    fn default() -> Self {
        Self {
            base: BacksteppingConfig::default(),
            orbit_radius_scale: 0.75,
            orbit_frequency: 2.0,
            orbit_decay_rate: 0.8,
        }
    }
}

/// One recorded simulation step.
#[derive(Debug, Clone, Copy)]
pub struct BacksteppingStep {
    pub pose: Pose2D,
    pub control: ControlInput,
    pub distance_to_goal: f64,
}

/// Full backstepping simulation result.
#[derive(Debug, Clone)]
pub struct BacksteppingResult {
    pub steps: Vec<BacksteppingStep>,
    pub converged: bool,
}

impl BacksteppingResult {
    /// Returns the final pose in the recorded rollout.
    pub fn final_pose(&self) -> Pose2D {
        self.steps
            .last()
            .map(|step| step.pose)
            .unwrap_or_else(Pose2D::origin)
    }

    /// Returns the number of applied control steps.
    pub fn iterations(&self) -> usize {
        self.steps.len().saturating_sub(1)
    }

    /// Returns the traced path of the robot center.
    pub fn path(&self) -> Path2D {
        Path2D::from_points(
            self.steps
                .iter()
                .map(|step| Point2D::new(step.pose.x, step.pose.y))
                .collect(),
        )
    }
}

/// Backstepping controller.
pub struct BacksteppingController {
    config: BacksteppingConfig,
}

impl BacksteppingController {
    /// Creates a new controller from the provided configuration.
    pub fn new(config: BacksteppingConfig) -> Self {
        Self { config }
    }

    /// Returns the controller configuration.
    pub fn config(&self) -> BacksteppingConfig {
        self.config
    }

    /// Simulates the controller from `start` to `goal`.
    pub fn simulate(&self, start: Pose2D, goal: Pose2D) -> BacksteppingResult {
        let mut pose = start;
        let mut steps = vec![BacksteppingStep {
            pose,
            control: ControlInput::zero(),
            distance_to_goal: distance_to_goal(pose, goal),
        }];

        for _ in 0..self.config.max_steps {
            if self.is_converged(pose, goal) {
                return BacksteppingResult {
                    steps,
                    converged: true,
                };
            }

            let control = self.compute_control(pose, goal);
            pose = integrate_pose(pose, control, self.config.dt);
            steps.push(BacksteppingStep {
                pose,
                control,
                distance_to_goal: distance_to_goal(pose, goal),
            });
        }

        BacksteppingResult {
            steps,
            converged: false,
        }
    }

    /// Convenience wrapper returning only the traced `x`/`y` points.
    pub fn planning(&self, start: (f64, f64, f64), goal: (f64, f64, f64)) -> Vec<(f64, f64)> {
        self.simulate(
            Pose2D::new(start.0, start.1, start.2),
            Pose2D::new(goal.0, goal.1, goal.2),
        )
        .path()
        .points
        .into_iter()
        .map(|point| (point.x, point.y))
        .collect()
    }

    fn is_converged(&self, pose: Pose2D, goal: Pose2D) -> bool {
        let position_ok = distance_to_goal(pose, goal) <= self.config.goal_tolerance;
        let yaw_ok = normalize_angle(goal.yaw - pose.yaw).abs() <= self.config.goal_tolerance;
        position_ok && yaw_ok
    }

    fn compute_control(&self, pose: Pose2D, goal: Pose2D) -> ControlInput {
        let dx = goal.x - pose.x;
        let dy = goal.y - pose.y;
        let distance = (dx.powi(2) + dy.powi(2)).sqrt();

        if distance <= self.terminal_region_radius() {
            return self.compute_terminal_control(pose, goal, distance);
        }

        self.compute_tracking_control(pose, goal)
    }

    fn compute_tracking_control(&self, pose: Pose2D, goal: Pose2D) -> ControlInput {
        let dx = goal.x - pose.x;
        let dy = goal.y - pose.y;

        let cos_theta = pose.yaw.cos();
        let sin_theta = pose.yaw.sin();

        let e1 = cos_theta * dx + sin_theta * dy;
        let e2 = -sin_theta * dx + cos_theta * dy;
        let e3 = normalize_angle(goal.yaw - pose.yaw);

        let vr = self.config.k1 * (e1.powi(2) + e2.powi(2)).sqrt();
        let v = vr * e3.cos() + self.config.k1 * e1;
        let omega = vr * (self.config.k2 * e2 + self.config.k3 * e3.sin());

        ControlInput::new(v, omega)
    }

    fn compute_terminal_control(&self, pose: Pose2D, goal: Pose2D, distance: f64) -> ControlInput {
        let alpha = normalize_angle((goal.y - pose.y).atan2(goal.x - pose.x) - pose.yaw);
        let beta = normalize_angle(goal.yaw - pose.yaw - alpha);

        let mut v = self.config.k1 * distance;
        if !(-PI / 2.0..=PI / 2.0).contains(&alpha) {
            v = -v;
        }

        let omega = self.config.k2 * alpha - self.config.k3 * beta;
        ControlInput::new(v, omega)
    }

    fn terminal_region_radius(&self) -> f64 {
        10.0 * self.config.goal_tolerance
    }
}

/// Time-varying pose stabilizer based on a shrinking virtual-goal orbit.
///
/// This controller keeps the target explicitly time-varying by asking the
/// backstepping tracking law to follow a virtual goal that spirals into the
/// desired pose. The construction is Samson-inspired, but remains an
/// engineering approximation rather than a direct transcription of the chained
/// coordinates feedback law.
pub struct TimeVaryingBacksteppingController {
    config: TimeVaryingBacksteppingConfig,
    tracker: BacksteppingController,
}

impl TimeVaryingBacksteppingController {
    /// Creates a new time-varying stabilizer.
    pub fn new(config: TimeVaryingBacksteppingConfig) -> Self {
        Self {
            tracker: BacksteppingController::new(config.base),
            config,
        }
    }

    /// Simulates the controller from `start` to `goal`.
    pub fn simulate(&self, start: Pose2D, goal: Pose2D) -> BacksteppingResult {
        let mut pose = start;
        let mut steps = vec![BacksteppingStep {
            pose,
            control: ControlInput::zero(),
            distance_to_goal: distance_to_goal(pose, goal),
        }];
        let reference_radius = distance_to_goal(start, goal).max(self.config.base.goal_tolerance)
            * self.config.orbit_radius_scale;
        let initial_phase = (start.y - goal.y).atan2(start.x - goal.x);

        for step_index in 0..self.config.base.max_steps {
            if self.is_converged(pose, goal) {
                return BacksteppingResult {
                    steps,
                    converged: true,
                };
            }

            let time = step_index as f64 * self.config.base.dt;
            let distance = distance_to_goal(pose, goal);
            let control = if distance <= self.tracker.terminal_region_radius() {
                self.tracker.compute_terminal_control(pose, goal, distance)
            } else {
                let virtual_goal = self.virtual_goal(goal, reference_radius, initial_phase, time);
                self.tracker.compute_tracking_control(pose, virtual_goal)
            };
            pose = integrate_pose(pose, control, self.config.base.dt);
            steps.push(BacksteppingStep {
                pose,
                control,
                distance_to_goal: distance_to_goal(pose, goal),
            });
        }

        BacksteppingResult {
            steps,
            converged: false,
        }
    }

    fn is_converged(&self, pose: Pose2D, goal: Pose2D) -> bool {
        let position_ok = distance_to_goal(pose, goal) <= self.config.base.goal_tolerance;
        let yaw_ok = normalize_angle(goal.yaw - pose.yaw).abs() <= self.config.base.goal_tolerance;
        position_ok && yaw_ok
    }

    fn virtual_goal(
        &self,
        goal: Pose2D,
        reference_radius: f64,
        initial_phase: f64,
        time: f64,
    ) -> Pose2D {
        let amplitude = reference_radius * (-self.config.orbit_decay_rate * time).exp();
        let phase = initial_phase + self.config.orbit_frequency * time;
        Pose2D::new(
            goal.x + amplitude * phase.cos(),
            goal.y + amplitude * phase.sin(),
            goal.yaw,
        )
    }
}

/// Runs the backstepping controller from `start` to `goal`.
pub fn backstepping_control(
    start: Pose2D,
    goal: Pose2D,
    config: BacksteppingConfig,
) -> BacksteppingResult {
    BacksteppingController::new(config).simulate(start, goal)
}

/// Runs the Samson-inspired time-varying backstepping controller.
pub fn time_varying_backstepping_control(
    start: Pose2D,
    goal: Pose2D,
    config: TimeVaryingBacksteppingConfig,
) -> BacksteppingResult {
    TimeVaryingBacksteppingController::new(config).simulate(start, goal)
}

/// Normalizes an angle into `[-pi, pi]`.
pub fn normalize_angle(mut angle: f64) -> f64 {
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

fn integrate_pose(pose: Pose2D, control: ControlInput, dt: f64) -> Pose2D {
    let x = pose.x + control.v * pose.yaw.cos() * dt;
    let y = pose.y + control.v * pose.yaw.sin() * dt;
    let yaw = normalize_angle(pose.yaw + control.omega * dt);
    Pose2D::new(x, y, yaw)
}

fn distance_to_goal(pose: Pose2D, goal: Pose2D) -> f64 {
    ((goal.x - pose.x).powi(2) + (goal.y - pose.y).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_backstepping_config_defaults() {
        let config = BacksteppingConfig::default();
        assert_eq!(config.k1, 3.0);
        assert_eq!(config.k2, 8.0);
        assert_eq!(config.k3, 3.0);
        assert_eq!(config.dt, 0.01);
        assert_eq!(config.goal_tolerance, 0.05);
        assert_eq!(config.max_steps, 10_000);
    }

    #[test]
    fn test_backstepping_converges_to_goal() {
        let start = Pose2D::origin();
        let goal = Pose2D::new(2.0, 1.5, PI / 4.0);

        let result = backstepping_control(start, goal, BacksteppingConfig::default());
        let final_pose = result.final_pose();

        assert!(result.converged);
        assert!(distance_to_goal(final_pose, goal) < 0.05);
        assert!(normalize_angle(final_pose.yaw - goal.yaw).abs() < 0.05);
    }

    #[test]
    fn test_backstepping_returns_non_empty_path() {
        let controller = BacksteppingController::new(BacksteppingConfig::default());
        let result = controller.simulate(Pose2D::origin(), Pose2D::new(1.5, 0.5, 0.0));
        let path = result.path();

        assert!(path.points.len() > 1);
        assert_eq!(path.points[0].x, 0.0);
        assert_eq!(path.points[0].y, 0.0);
    }

    #[test]
    fn test_time_varying_backstepping_config_defaults() {
        let config = TimeVaryingBacksteppingConfig::default();
        assert_eq!(config.base.goal_tolerance, 0.05);
        assert_eq!(config.orbit_radius_scale, 0.75);
        assert_eq!(config.orbit_frequency, 2.0);
        assert_eq!(config.orbit_decay_rate, 0.8);
    }

    #[test]
    fn test_time_varying_backstepping_converges_to_goal() {
        let start = Pose2D::origin();
        let goal = Pose2D::new(1.5, 1.0, PI / 2.0);
        let controller = TimeVaryingBacksteppingController::new(TimeVaryingBacksteppingConfig {
            base: BacksteppingConfig {
                k1: 2.5,
                k2: 6.0,
                k3: 3.0,
                dt: 0.01,
                goal_tolerance: 0.05,
                max_steps: 12_000,
            },
            orbit_radius_scale: 0.75,
            orbit_frequency: 2.0,
            orbit_decay_rate: 0.8,
        });

        let result = controller.simulate(start, goal);
        let final_pose = result.final_pose();

        assert!(result.converged);
        assert!(distance_to_goal(final_pose, goal) < 0.05);
        assert!(normalize_angle(final_pose.yaw - goal.yaw).abs() < 0.05);
    }
}
