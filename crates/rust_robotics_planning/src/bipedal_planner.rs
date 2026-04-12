//! Bipedal walking planner with modified designated footsteps.
//!
//! Ports the linear inverted-pendulum footstep adjustment demo from
//! PythonRobotics.
//!
//! Reference:
//! - PythonRobotics `Bipedal/bipedal_planner/bipedal_planner.py`

use rust_robotics_core::{Point2D, RoboticsError, RoboticsResult};

/// One foot placement in world or body-relative coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Footstep {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Footstep {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }
}

/// Configuration for the bipedal planner.
#[derive(Debug, Clone, Copy)]
pub struct BipedalPlannerConfig {
    /// Support phase duration \[s\].
    pub t_sup: f64,
    /// Center-of-mass height \[m\].
    pub z_c: f64,
    /// Footstep modification weight on COM position error.
    pub a: f64,
    /// Footstep modification weight on COM velocity error.
    pub b: f64,
    /// Number of Euler integration steps per support phase.
    pub time_split: usize,
    /// Keep one COM sample every `trajectory_stride` integration steps.
    pub trajectory_stride: usize,
    /// Gravitational acceleration [m/s^2].
    pub gravity: f64,
}

impl Default for BipedalPlannerConfig {
    fn default() -> Self {
        Self {
            t_sup: 0.8,
            z_c: 0.8,
            a: 10.0,
            b: 1.0,
            time_split: 100,
            trajectory_stride: 10,
            gravity: 9.8,
        }
    }
}

impl BipedalPlannerConfig {
    fn validate(&self) -> RoboticsResult<()> {
        if self.t_sup <= 0.0 || !self.t_sup.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "BipedalPlanner: t_sup must be positive".to_string(),
            ));
        }
        if self.z_c <= 0.0 || !self.z_c.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "BipedalPlanner: z_c must be positive".to_string(),
            ));
        }
        if self.a <= 0.0 || self.b <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "BipedalPlanner: a and b must be positive".to_string(),
            ));
        }
        if self.time_split == 0 || self.trajectory_stride == 0 {
            return Err(RoboticsError::InvalidParameter(
                "BipedalPlanner: time_split and trajectory_stride must be greater than zero"
                    .to_string(),
            ));
        }
        if self.gravity <= 0.0 || !self.gravity.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "BipedalPlanner: gravity must be positive".to_string(),
            ));
        }
        Ok(())
    }
}

/// Walking plan output.
#[derive(Debug, Clone)]
pub struct BipedalPlan {
    pub reference_footsteps: Vec<Footstep>,
    pub modified_footsteps: Vec<Footstep>,
    pub com_trajectory: Vec<Point2D>,
}

#[derive(Debug, Clone, Copy)]
struct PendulumState {
    x: f64,
    x_dot: f64,
    y: f64,
    y_dot: f64,
}

/// Planner for the PythonRobotics-style bipedal footstep adjustment demo.
pub struct BipedalPlanner {
    config: BipedalPlannerConfig,
}

impl BipedalPlanner {
    pub fn new(config: BipedalPlannerConfig) -> Self {
        Self { config }
    }

    pub fn config(&self) -> BipedalPlannerConfig {
        self.config
    }

    /// Compute modified footsteps and a coarse COM trajectory.
    pub fn plan(&self, reference_footsteps: &[Footstep]) -> RoboticsResult<BipedalPlan> {
        self.config.validate()?;
        if reference_footsteps.is_empty() {
            return Err(RoboticsError::InvalidParameter(
                "BipedalPlanner: reference footsteps must not be empty".to_string(),
            ));
        }

        let mut plan = BipedalPlan {
            reference_footsteps: vec![Footstep::new(0.0, 0.0, 0.0)],
            modified_footsteps: vec![Footstep::new(0.0, 0.0, 0.0)],
            com_trajectory: Vec::new(),
        };

        let mut px = 0.0;
        let mut py = 0.0;
        let mut px_star = 0.0;
        let mut py_star = 0.0;
        let mut state = PendulumState {
            x: 0.0,
            x_dot: 0.0,
            y: 0.01,
            y_dot: 0.0,
        };

        for (index, current_step) in reference_footsteps.iter().copied().enumerate() {
            state =
                self.integrate_inverted_pendulum(state, px_star, py_star, &mut plan.com_trajectory);

            let step_number = index + 1;
            let sign = alternating_sign(step_number);
            let (dx, dy) =
                rotate_vector(current_step.theta, current_step.x, -sign * current_step.y);
            px += dx;
            py += dy;

            let next_step = reference_footsteps
                .get(index + 1)
                .copied()
                .unwrap_or_else(|| Footstep::new(0.0, 0.0, 0.0));
            let tc = (self.config.z_c / self.config.gravity).sqrt();
            let c = (self.config.t_sup / tc).cosh();
            let s = (self.config.t_sup / tc).sinh();

            let (x_ref, y_ref) =
                rotate_vector(next_step.theta, next_step.x / 2.0, sign * next_step.y / 2.0);
            let (vx_ref, vy_ref) = rotate_vector(
                next_step.theta,
                (1.0 + c) / (tc * s) * x_ref,
                (c - 1.0) / (tc * s) * y_ref,
            );

            plan.reference_footsteps
                .push(Footstep::new(px, py, current_step.theta));

            let xd = px + x_ref;
            let yd = py + y_ref;
            let xd_dot = vx_ref;
            let yd_dot = vy_ref;

            let d = self.config.a * (c - 1.0).powi(2) + self.config.b * (s / tc).powi(2);
            px_star = -self.config.a * (c - 1.0) / d * (xd - c * state.x - tc * s * state.x_dot)
                - self.config.b * s / (tc * d) * (xd_dot - s / tc * state.x - c * state.x_dot);
            py_star = -self.config.a * (c - 1.0) / d * (yd - c * state.y - tc * s * state.y_dot)
                - self.config.b * s / (tc * d) * (yd_dot - s / tc * state.y - c * state.y_dot);

            plan.modified_footsteps
                .push(Footstep::new(px_star, py_star, current_step.theta));
        }

        Ok(plan)
    }

    fn integrate_inverted_pendulum(
        &self,
        mut state: PendulumState,
        px_star: f64,
        py_star: f64,
        com_trajectory: &mut Vec<Point2D>,
    ) -> PendulumState {
        let delta_time = self.config.t_sup / self.config.time_split as f64;

        for step in 0..self.config.time_split {
            let x_ddot = self.config.gravity / self.config.z_c * (state.x - px_star);
            state.x += state.x_dot * delta_time;
            state.x_dot += x_ddot * delta_time;

            let y_ddot = self.config.gravity / self.config.z_c * (state.y - py_star);
            state.y += state.y_dot * delta_time;
            state.y_dot += y_ddot * delta_time;

            if step % self.config.trajectory_stride == 0 {
                com_trajectory.push(Point2D::new(state.x, state.y));
            }
        }

        state
    }
}

fn alternating_sign(n: usize) -> f64 {
    if n.is_multiple_of(2) {
        1.0
    } else {
        -1.0
    }
}

fn rotate_vector(theta: f64, x: f64, y: f64) -> (f64, f64) {
    let cos_theta = theta.cos();
    let sin_theta = theta.sin();
    (cos_theta * x - sin_theta * y, sin_theta * x + cos_theta * y)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {expected:.12}, got {actual:.12}, tolerance {tolerance:.3e}"
        );
    }

    fn default_pythonrobotics_footsteps() -> Vec<Footstep> {
        vec![
            Footstep::new(0.0, 0.2, 0.0),
            Footstep::new(0.3, 0.2, 0.0),
            Footstep::new(0.3, 0.2, 0.2),
            Footstep::new(0.3, 0.2, 0.2),
            Footstep::new(0.0, 0.2, 0.2),
        ]
    }

    #[test]
    fn test_bipedal_planner_config_defaults() {
        let config = BipedalPlannerConfig::default();
        assert_eq!(config.t_sup, 0.8);
        assert_eq!(config.z_c, 0.8);
        assert_eq!(config.a, 10.0);
        assert_eq!(config.b, 1.0);
        assert_eq!(config.time_split, 100);
        assert_eq!(config.trajectory_stride, 10);
        assert_eq!(config.gravity, 9.8);
    }

    #[test]
    fn test_bipedal_planner_matches_pythonrobotics_default_walk() {
        let planner = BipedalPlanner::new(BipedalPlannerConfig::default());
        let plan = planner
            .plan(&default_pythonrobotics_footsteps())
            .expect("planner should succeed");

        assert_eq!(plan.reference_footsteps.len(), 6);
        assert_eq!(plan.modified_footsteps.len(), 6);
        assert_eq!(plan.com_trajectory.len(), 50);

        let expected_reference = [
            Footstep::new(0.0, 0.0, 0.0),
            Footstep::new(0.0, 0.2, 0.0),
            Footstep::new(0.3, 0.0, 0.0),
            Footstep::new(0.5542861071933602, 0.2556141148067667, 0.2),
            Footstep::new(0.888039946704745, 0.11920159847703674, 0.2),
            Footstep::new(0.8483060805457328, 0.31521491404528507, 0.2),
        ];
        let expected_modified = [
            Footstep::new(0.0, 0.0, 0.0),
            Footstep::new(-0.02068187189406957, 0.16806092102834097, 0.0),
            Footstep::new(0.2914994382902407, -0.004038373768573439, 0.0),
            Footstep::new(0.5054452070274589, 0.2653647289918646, 0.2),
            Footstep::new(0.9019274635077843, 0.16741541638077787, 0.2),
            Footstep::new(0.8258088100265661, 0.29759216443331915, 0.2),
        ];

        for (actual, expected) in plan
            .reference_footsteps
            .iter()
            .zip(expected_reference.iter())
        {
            assert_close(actual.x, expected.x, 1.0e-12);
            assert_close(actual.y, expected.y, 1.0e-12);
            assert_close(actual.theta, expected.theta, 1.0e-12);
        }
        for (actual, expected) in plan.modified_footsteps.iter().zip(expected_modified.iter()) {
            assert_close(actual.x, expected.x, 1.0e-12);
            assert_close(actual.y, expected.y, 1.0e-12);
            assert_close(actual.theta, expected.theta, 1.0e-12);
        }

        let first = plan.com_trajectory.first().expect("trajectory");
        let middle = &plan.com_trajectory[25];
        let last = plan.com_trajectory.last().expect("trajectory");
        assert_close(first.x, 0.0, 1.0e-12);
        assert_close(first.y, 0.01, 1.0e-12);
        assert_close(middle.x, 0.2858111071163949, 1.0e-12);
        assert_close(middle.y, 0.04910787389588683, 1.0e-12);
        assert_close(last.x, 0.8572352745281623, 1.0e-12);
        assert_close(last.y, 0.22162409933209326, 1.0e-12);
    }

    #[test]
    fn test_bipedal_planner_rejects_empty_footsteps() {
        let planner = BipedalPlanner::new(BipedalPlannerConfig::default());
        let error = planner.plan(&[]).expect_err("empty footsteps must fail");
        let message = error.to_string();
        assert!(message.contains("reference footsteps"));
    }
}
