//! Feedback Linearization controller for a unicycle robot.
//!
//! Places a virtual point P at distance `L` ahead of the robot and applies
//! proportional control to drive P to the goal, then transforms the
//! Cartesian velocity commands back into (v, ω) for the unicycle.
use rust_robotics_core::{ControlInput, Path2D, Point2D, Pose2D};

/// Configuration for the Feedback Linearization controller.
#[derive(Debug, Clone, Copy)]
pub struct FeedbackLinearizationConfig {
    /// Proportional gain on x-error of the virtual point \[1/s\].
    pub k1: f64,
    /// Proportional gain on y-error of the virtual point \[1/s\].
    pub k2: f64,
    /// Distance of the virtual point ahead of the robot \[m\].
    pub offset_distance: f64,
    /// Integration time-step \[s\].
    pub dt: f64,
    /// Distance at which convergence is declared \[m\].
    pub goal_tolerance: f64,
    /// Maximum simulation steps before giving up.
    pub max_steps: usize,
}

impl Default for FeedbackLinearizationConfig {
    fn default() -> Self {
        Self {
            k1: 1.0,
            k2: 1.0,
            offset_distance: 0.1,
            dt: 0.01,
            goal_tolerance: 0.05,
            max_steps: 10_000,
        }
    }
}

/// One recorded step of the simulation.
#[derive(Debug, Clone, Copy)]
pub struct FeedbackLinearizationStep {
    pub pose: Pose2D,
    pub control: ControlInput,
    pub distance_to_goal: f64,
}

/// Full simulation result.
#[derive(Debug, Clone)]
pub struct FeedbackLinearizationResult {
    pub steps: Vec<FeedbackLinearizationStep>,
    pub converged: bool,
}

impl FeedbackLinearizationResult {
    pub fn final_pose(&self) -> Pose2D {
        self.steps
            .last()
            .map(|s| s.pose)
            .unwrap_or_else(Pose2D::origin)
    }

    pub fn iterations(&self) -> usize {
        self.steps.len().saturating_sub(1)
    }

    pub fn path(&self) -> Path2D {
        Path2D::from_points(
            self.steps
                .iter()
                .map(|s| Point2D::new(s.pose.x, s.pose.y))
                .collect(),
        )
    }
}

/// Feedback Linearization controller.
pub struct FeedbackLinearizationController {
    config: FeedbackLinearizationConfig,
}

impl FeedbackLinearizationController {
    pub fn new(config: FeedbackLinearizationConfig) -> Self {
        Self { config }
    }

    pub fn config(&self) -> FeedbackLinearizationConfig {
        self.config
    }

    /// Simulate the controller from `start` to `goal`.
    pub fn simulate(&self, start: Pose2D, goal: Pose2D) -> FeedbackLinearizationResult {
        let mut pose = start;
        let mut steps = vec![FeedbackLinearizationStep {
            pose,
            control: ControlInput::zero(),
            distance_to_goal: dist(pose, goal),
        }];

        for _ in 0..self.config.max_steps {
            // Convergence is declared when the virtual point P (L ahead of the
            // robot) is within goal_tolerance of the goal.  This avoids the
            // inherent steady-state offset of L between the robot centre and P.
            let l = self.config.offset_distance;
            let px = pose.x + l * pose.yaw.cos();
            let py = pose.y + l * pose.yaw.sin();
            let p_dist = ((goal.x - px).powi(2) + (goal.y - py).powi(2)).sqrt();
            if p_dist <= self.config.goal_tolerance {
                return FeedbackLinearizationResult {
                    steps,
                    converged: true,
                };
            }

            let control = self.compute_control(pose, goal);
            pose = integrate_pose(pose, control, self.config.dt);
            steps.push(FeedbackLinearizationStep {
                pose,
                control,
                distance_to_goal: dist(pose, goal),
            });
        }

        FeedbackLinearizationResult {
            steps,
            converged: false,
        }
    }

    fn compute_control(&self, pose: Pose2D, goal: Pose2D) -> ControlInput {
        let l = self.config.offset_distance;
        let (cos_t, sin_t) = (pose.yaw.cos(), pose.yaw.sin());

        // Virtual point P ahead of robot
        let px = pose.x + l * cos_t;
        let py = pose.y + l * sin_t;

        // Desired Cartesian velocity for P
        let ux = -self.config.k1 * (px - goal.x);
        let uy = -self.config.k2 * (py - goal.y);

        // Transform back to unicycle (v, ω)
        let v = ux * cos_t + uy * sin_t;
        let omega = (-ux * sin_t + uy * cos_t) / l;

        ControlInput::new(v, omega)
    }
}

/// Convenience function.
pub fn feedback_linearization(
    start: Pose2D,
    goal: Pose2D,
    config: FeedbackLinearizationConfig,
) -> FeedbackLinearizationResult {
    FeedbackLinearizationController::new(config).simulate(start, goal)
}

fn integrate_pose(pose: Pose2D, control: ControlInput, dt: f64) -> Pose2D {
    let yaw = pose.yaw + control.omega * dt;
    Pose2D::new(
        pose.x + control.v * yaw.cos() * dt,
        pose.y + control.v * yaw.sin() * dt,
        yaw,
    )
}

fn dist(pose: Pose2D, goal: Pose2D) -> f64 {
    ((goal.x - pose.x).powi(2) + (goal.y - pose.y).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_defaults() {
        let cfg = FeedbackLinearizationConfig::default();
        assert_eq!(cfg.k1, 1.0);
        assert_eq!(cfg.k2, 1.0);
        assert_eq!(cfg.offset_distance, 0.1);
        assert_eq!(cfg.dt, 0.01);
        assert_eq!(cfg.goal_tolerance, 0.05);
        assert_eq!(cfg.max_steps, 10_000);
    }

    #[test]
    fn test_feedback_linearization_converges() {
        let start = Pose2D::origin();
        let goal = Pose2D::new(5.0, 5.0, 0.0);
        let result = feedback_linearization(start, goal, FeedbackLinearizationConfig::default());
        let fp = result.final_pose();

        let l = FeedbackLinearizationConfig::default().offset_distance;
        let px = fp.x + l * fp.yaw.cos();
        let py = fp.y + l * fp.yaw.sin();
        assert!(
            result.converged,
            "controller did not converge (virtual point distance: {:.4})",
            ((goal.x - px).powi(2) + (goal.y - py).powi(2)).sqrt()
        );
        // Virtual point P is at goal; robot body is at most L away.
        assert!((px - goal.x).abs() < 0.1);
        assert!((py - goal.y).abs() < 0.1);
    }

    #[test]
    fn test_feedback_linearization_non_empty_path() {
        let controller =
            FeedbackLinearizationController::new(FeedbackLinearizationConfig::default());
        let result = controller.simulate(Pose2D::origin(), Pose2D::new(2.0, 1.0, 0.0));
        let path = result.path();

        assert!(
            path.points.len() > 1,
            "path should have more than one point"
        );
        assert_eq!(path.points[0].x, 0.0);
        assert_eq!(path.points[0].y, 0.0);
    }
}
