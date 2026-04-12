//
// Sliding Mode Control for unicycle mobile robot
// Forces the system state onto a sliding surface s=0,
// providing robust control against disturbances.
use rust_robotics_core::{ControlInput, Path2D, Point2D, Pose2D};
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy)]
pub struct SlidingModeConfig {
    /// Sliding surface slope weight on heading error.
    pub lambda: f64,
    /// Reaching gain (control aggressiveness).
    pub eta: f64,
    /// Boundary layer thickness for chattering reduction.
    pub phi: f64,
    pub dt: f64,
    pub goal_tolerance: f64,
    pub max_steps: usize,
}

impl Default for SlidingModeConfig {
    fn default() -> Self {
        Self {
            lambda: 1.0,
            eta: 0.5,
            phi: 0.1,
            dt: 0.01,
            goal_tolerance: 0.05,
            max_steps: 10_000,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SlidingModeStep {
    pub pose: Pose2D,
    pub control: ControlInput,
    pub distance_to_goal: f64,
}

#[derive(Debug, Clone)]
pub struct SlidingModeResult {
    pub steps: Vec<SlidingModeStep>,
    pub converged: bool,
}

impl SlidingModeResult {
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

pub struct SlidingModeController {
    config: SlidingModeConfig,
}

impl SlidingModeController {
    pub fn new(config: SlidingModeConfig) -> Self {
        Self { config }
    }

    pub fn config(&self) -> SlidingModeConfig {
        self.config
    }

    pub fn simulate(&self, start: Pose2D, goal: Pose2D) -> SlidingModeResult {
        let mut pose = start;
        let mut steps = vec![SlidingModeStep {
            pose,
            control: ControlInput::zero(),
            distance_to_goal: dist(pose, goal),
        }];

        for _ in 0..self.config.max_steps {
            if dist(pose, goal) <= self.config.goal_tolerance {
                return SlidingModeResult {
                    steps,
                    converged: true,
                };
            }

            let control = self.compute_control(pose, goal);
            pose = integrate_pose(pose, control, self.config.dt);
            steps.push(SlidingModeStep {
                pose,
                control,
                distance_to_goal: dist(pose, goal),
            });
        }

        SlidingModeResult {
            steps,
            converged: false,
        }
    }

    fn compute_control(&self, pose: Pose2D, goal: Pose2D) -> ControlInput {
        let x_diff = goal.x - pose.x;
        let y_diff = goal.y - pose.y;
        let rho = (x_diff.powi(2) + y_diff.powi(2)).sqrt();
        let alpha = normalize_angle(y_diff.atan2(x_diff) - pose.yaw);

        // Sliding surface: s = alpha + lambda * rho_dot_desired
        // Use alpha as the primary surface; lambda couples the distance error.
        // s = alpha  (heading error); we want s -> 0
        let s = alpha;

        // Forward velocity: proportional to distance, sign corrected so robot
        // always drives towards the goal (back up if facing away by > 90 deg).
        let v = if alpha.abs() <= PI / 2.0 {
            self.config.eta * rho
        } else {
            -self.config.eta * rho
        };

        // Angular velocity: sliding mode reaching law u = -eta * sat(s / phi)
        // plus a proportional term lambda*rho to pull rho down while on surface.
        let omega =
            -self.config.eta * sat(s / self.config.phi) - self.config.lambda * rho * alpha.signum();

        ControlInput::new(v, omega)
    }
}

pub fn sliding_mode_control(
    start: Pose2D,
    goal: Pose2D,
    config: SlidingModeConfig,
) -> SlidingModeResult {
    SlidingModeController::new(config).simulate(start, goal)
}

/// Smooth saturation function (boundary layer approximation of sign).
fn sat(x: f64) -> f64 {
    x.clamp(-1.0, 1.0)
}

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
    let yaw = normalize_angle(pose.yaw + control.omega * dt);
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
    fn test_sliding_mode_config_defaults() {
        let cfg = SlidingModeConfig::default();
        assert_eq!(cfg.lambda, 1.0);
        assert_eq!(cfg.eta, 0.5);
        assert_eq!(cfg.phi, 0.1);
        assert_eq!(cfg.dt, 0.01);
        assert_eq!(cfg.goal_tolerance, 0.05);
        assert_eq!(cfg.max_steps, 10_000);
    }

    #[test]
    fn test_sliding_mode_converges() {
        let start = Pose2D::origin();
        let goal = Pose2D::new(3.0, 3.0, 0.0);

        let result = sliding_mode_control(start, goal, SlidingModeConfig::default());

        println!(
            "converged: {}, steps: {}, final: ({:.3}, {:.3})",
            result.converged,
            result.iterations(),
            result.final_pose().x,
            result.final_pose().y,
        );

        assert!(result.converged);
        assert!(dist(result.final_pose(), goal) <= SlidingModeConfig::default().goal_tolerance);
    }

    #[test]
    fn test_sliding_mode_returns_non_empty_path() {
        let controller = SlidingModeController::new(SlidingModeConfig::default());
        let result = controller.simulate(Pose2D::origin(), Pose2D::new(2.0, 1.0, 0.0));
        let path = result.path();

        assert!(path.points.len() > 1);
        assert_eq!(path.points[0].x, 0.0);
        assert_eq!(path.points[0].y, 0.0);
    }
}
