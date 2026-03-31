//! LQR-based path planner
//!
//! Uses a discrete-time double-integrator model with LQR control
//! to generate a smooth path from start to goal.
//! The state is expressed relative to the goal so the LQR drives
//! the error to zero.
//!
//! Reference: PythonRobotics LQRPlanner by Atsushi Sakai

use nalgebra::{Matrix1, Matrix2, Matrix2x1, Vector2};
use rust_robotics_core::{Path2D, Point2D, RoboticsError, RoboticsResult};

/// Configuration for the LQR path planner.
#[derive(Debug, Clone)]
pub struct LqrPlannerConfig {
    /// Maximum simulation time \[s\]
    pub max_time: f64,
    /// Time step \[s\]
    pub dt: f64,
    /// Goal tolerance distance \[m\]
    pub goal_dist: f64,
    /// Maximum DARE iterations
    pub max_iter: usize,
    /// DARE convergence threshold
    pub eps: f64,
}

impl Default for LqrPlannerConfig {
    fn default() -> Self {
        Self {
            max_time: 100.0,
            dt: 0.1,
            goal_dist: 0.1,
            max_iter: 150,
            eps: 0.01,
        }
    }
}

/// LQR-based path planner.
///
/// Plans a path by formulating each axis (x, y) as an independent
/// double-integrator and using discrete-time LQR to compute the
/// control input that drives the state towards the goal.
pub struct LqrPlanner {
    pub config: LqrPlannerConfig,
}

impl LqrPlanner {
    pub fn new(config: LqrPlannerConfig) -> Self {
        Self { config }
    }

    /// Plan a path from `start` to `goal`, returning a [`Path2D`].
    pub fn plan(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        let (rx, ry) = self.planning(start.x, start.y, goal.x, goal.y)?;
        Path2D::try_from_xy(&rx, &ry)
    }

    /// Plan a path returning raw coordinate vectors.
    pub fn planning(
        &self,
        sx: f64,
        sy: f64,
        gx: f64,
        gy: f64,
    ) -> RoboticsResult<(Vec<f64>, Vec<f64>)> {
        let mut rx = vec![sx];
        let mut ry = vec![sy];

        // State: error relative to goal for each axis
        let mut x = Vector2::new(sx - gx, sy - gy);

        let (a, b) = self.get_system_model();

        let mut time = 0.0;
        while time <= self.config.max_time {
            time += self.config.dt;

            let u = self.lqr_control(&a, &b, &x);
            x = a * x + b * u;

            rx.push(x[0] + gx);
            ry.push(x[1] + gy);

            let d =
                ((gx - *rx.last().unwrap()).powi(2) + (gy - *ry.last().unwrap()).powi(2)).sqrt();
            if d <= self.config.goal_dist {
                return Ok((rx, ry));
            }
        }

        Err(RoboticsError::PlanningError(
            "LqrPlanner: could not reach goal within max_time".to_string(),
        ))
    }

    /// Build the discrete-time double-integrator model.
    ///
    /// State: \[position, velocity\]
    /// The A matrix encodes `position += dt * position + velocity`
    /// and `velocity += dt * velocity`.
    fn get_system_model(&self) -> (Matrix2<f64>, Matrix2x1<f64>) {
        let a = Matrix2::new(self.config.dt, 1.0, 0.0, self.config.dt);
        let b = Matrix2x1::new(0.0, 1.0);
        (a, b)
    }

    /// Compute the LQR control input `u = -K * x`.
    fn lqr_control(&self, a: &Matrix2<f64>, b: &Matrix2x1<f64>, x: &Vector2<f64>) -> f64 {
        let q = Matrix2::<f64>::identity();
        let r = Matrix1::<f64>::identity();
        let (k, _, _) = self.dlqr(a, b, &q, &r);
        let u = -(k * x);
        u[0]
    }

    /// Solve a discrete-time Algebraic Riccati Equation (DARE).
    fn solve_dare(
        &self,
        a: &Matrix2<f64>,
        b: &Matrix2x1<f64>,
        q: &Matrix2<f64>,
        r: &Matrix1<f64>,
    ) -> Matrix2<f64> {
        let mut p = *q;

        for _ in 0..self.config.max_iter {
            let bt_p = b.transpose() * p;
            let denom = r + bt_p * b;
            if denom[0].abs() < 1e-10 {
                break;
            }

            let pn =
                a.transpose() * p * a - a.transpose() * p * b * (1.0 / denom[0]) * bt_p * a + q;

            if (pn - p).abs().max() < self.config.eps {
                break;
            }
            p = pn;
        }
        p
    }

    /// Solve the discrete-time LQR controller.
    ///
    /// Returns `(K, P, eigenvalues)` where K is the gain matrix.
    fn dlqr(
        &self,
        a: &Matrix2<f64>,
        b: &Matrix2x1<f64>,
        q: &Matrix2<f64>,
        r: &Matrix1<f64>,
    ) -> (nalgebra::Matrix1x2<f64>, Matrix2<f64>, Vector2<f64>) {
        let p = self.solve_dare(a, b, q, r);

        let bt_p = b.transpose() * p;
        let denom = r + bt_p * b;
        let k = (1.0 / denom[0]) * bt_p * a;

        let closed_loop = a - b * k;
        let eigenvalues = closed_loop
            .eigenvalues()
            .unwrap_or_else(|| Vector2::zeros());

        (k, p, eigenvalues)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lqr_planner_reaches_goal() {
        let planner = LqrPlanner::new(LqrPlannerConfig::default());
        let (rx, ry) = planner.planning(0.0, 0.0, 10.0, 10.0).unwrap();

        assert!(rx.len() > 2);
        assert_eq!(rx.len(), ry.len());

        // Final point should be close to goal
        let last_x = *rx.last().unwrap();
        let last_y = *ry.last().unwrap();
        let d = ((last_x - 10.0).powi(2) + (last_y - 10.0).powi(2)).sqrt();
        assert!(d <= 0.1, "Final distance to goal: {d}");
    }

    #[test]
    fn test_lqr_planner_negative_goal() {
        let planner = LqrPlanner::new(LqrPlannerConfig::default());
        let (rx, ry) = planner.planning(5.0, 5.0, -10.0, -20.0).unwrap();

        let last_x = *rx.last().unwrap();
        let last_y = *ry.last().unwrap();
        let d = ((last_x + 10.0).powi(2) + (last_y + 20.0).powi(2)).sqrt();
        assert!(d <= 0.1, "Final distance to goal: {d}");
    }

    #[test]
    fn test_lqr_planner_same_start_goal() {
        let planner = LqrPlanner::new(LqrPlannerConfig::default());
        let (rx, ry) = planner.planning(5.0, 5.0, 5.0, 5.0).unwrap();

        // Should reach goal immediately (start == goal)
        assert!(!rx.is_empty());
    }

    #[test]
    fn test_lqr_planner_plan_returns_path2d() {
        let planner = LqrPlanner::new(LqrPlannerConfig::default());
        let path = planner
            .plan(Point2D::new(0.0, 0.0), Point2D::new(10.0, 10.0))
            .unwrap();

        assert!(path.points.len() > 2);
    }

    #[test]
    fn test_lqr_planner_starts_at_start() {
        let planner = LqrPlanner::new(LqrPlannerConfig::default());
        let (rx, ry) = planner.planning(3.0, 7.0, 20.0, 15.0).unwrap();

        assert!((rx[0] - 3.0).abs() < 1e-10);
        assert!((ry[0] - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_lqr_planner_config_custom() {
        let config = LqrPlannerConfig {
            max_time: 50.0,
            dt: 0.05,
            goal_dist: 0.5,
            max_iter: 200,
            eps: 0.001,
        };
        let planner = LqrPlanner::new(config);
        let result = planner.planning(0.0, 0.0, 5.0, 5.0);
        assert!(result.is_ok());
    }

    #[test]
    fn test_lqr_planner_unreachable_returns_error() {
        let config = LqrPlannerConfig {
            max_time: 0.01, // Very short time
            goal_dist: 0.001,
            ..LqrPlannerConfig::default()
        };
        let planner = LqrPlanner::new(config);
        let result = planner.planning(0.0, 0.0, 1000.0, 1000.0);
        assert!(result.is_err());
    }
}
