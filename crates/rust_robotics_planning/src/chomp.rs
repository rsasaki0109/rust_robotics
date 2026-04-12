//! CHOMP (Covariant Hamiltonian Optimization for Motion Planning).
//!
//! This module provides a practical 2D CHOMP-style optimizer that balances
//! smoothness and obstacle avoidance costs over a fixed waypoint trajectory.

use nalgebra::Vector2;
use std::f64::consts::PI;

use rust_robotics_core::{Path2D, PathPlanner, Point2D, RoboticsError};

use crate::rrt::CircleObstacle;

const DEFAULT_INFLUENCE_DISTANCE: f64 = 2.0;
const DEFAULT_ROBOT_RADIUS: f64 = 0.8;
const EPS: f64 = 1.0e-9;

/// Configuration for CHOMP trajectory optimization.
#[derive(Debug, Clone, Copy)]
pub struct ChompConfig {
    /// Number of waypoints including start and goal.
    pub n_waypoints: usize,
    /// Time step \[s\] used for smoothness scaling.
    pub dt: f64,
    /// Maximum optimization iterations.
    pub max_iterations: usize,
    /// Gradient descent step size.
    pub learning_rate: f64,
    /// Weight for obstacle potential.
    pub obstacle_cost_weight: f64,
    /// Weight for smoothness penalty.
    pub smoothness_weight: f64,
}

impl Default for ChompConfig {
    fn default() -> Self {
        Self {
            n_waypoints: 50,
            dt: 0.1,
            max_iterations: 100,
            learning_rate: 0.01,
            obstacle_cost_weight: 1.0,
            smoothness_weight: 1.0,
        }
    }
}

/// Optimization result for CHOMP.
#[derive(Debug, Clone)]
pub struct ChompResult {
    pub path: Path2D,
    pub cost: f64,
    pub iterations: usize,
}

/// CHOMP planner.
pub struct ChompPlanner {
    config: ChompConfig,
    obstacles: Vec<CircleObstacle>,
}

impl ChompPlanner {
    pub fn new(obstacles: Vec<CircleObstacle>, config: ChompConfig) -> Self {
        Self { config, obstacles }
    }

    pub fn optimize(&self, start: Point2D, goal: Point2D) -> Result<ChompResult, RoboticsError> {
        if self.config.n_waypoints < 2 {
            return Err(RoboticsError::InvalidParameter(
                "CHOMP requires at least 2 waypoints".to_string(),
            ));
        }
        if self.config.dt <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "CHOMP dt must be positive".to_string(),
            ));
        }
        if self.config.learning_rate <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "CHOMP learning_rate must be positive".to_string(),
            ));
        }

        let mut waypoints = self.initialize_trajectory(start, goal);
        let mut cost = self.trajectory_cost(&waypoints);
        let mut iterations = 0usize;

        for iter in 0..self.config.max_iterations {
            iterations = iter + 1;
            let mut step_size = self.config.learning_rate;
            let mut accepted = None;

            for _ in 0..8 {
                let mut next = waypoints.clone();
                for i in 1..(waypoints.len() - 1) {
                    let smooth = self.smoothness_gradient(&waypoints, i);
                    let obstacle = self.obstacle_gradient(&waypoints[i]);
                    let grad = self.config.smoothness_weight * smooth
                        + self.config.obstacle_cost_weight * obstacle;
                    next[i] -= step_size * grad;
                }

                // Keep boundary conditions fixed.
                next[0] = Vector2::new(start.x, start.y);
                *next.last_mut().expect("non-empty trajectory") = Vector2::new(goal.x, goal.y);

                let next_cost = self.trajectory_cost(&next);
                if next_cost <= cost {
                    accepted = Some((next, next_cost));
                    break;
                }
                step_size *= 0.5;
            }

            let Some((next, next_cost)) = accepted else {
                break;
            };
            if (cost - next_cost).abs() < 1.0e-9 {
                waypoints = next;
                cost = next_cost;
                break;
            }
            waypoints = next;
            cost = next_cost;
        }

        let points = waypoints
            .into_iter()
            .map(|p| Point2D::new(p.x, p.y))
            .collect();

        Ok(ChompResult {
            path: Path2D::from_points(points),
            cost,
            iterations,
        })
    }

    fn initialize_trajectory(&self, start: Point2D, goal: Point2D) -> Vec<Vector2<f64>> {
        let mut trajectory = Vec::with_capacity(self.config.n_waypoints);
        for i in 0..self.config.n_waypoints {
            let t = if self.config.n_waypoints == 1 {
                0.0
            } else {
                i as f64 / (self.config.n_waypoints - 1) as f64
            };
            trajectory.push(Vector2::new(
                start.x + t * (goal.x - start.x),
                start.y + t * (goal.y - start.y) + 1.0e-3 * (PI * t).sin(),
            ));
        }
        trajectory
    }

    fn smoothness_gradient(&self, trajectory: &[Vector2<f64>], i: usize) -> Vector2<f64> {
        let dt2 = self.config.dt * self.config.dt;
        let second_diff = trajectory[i - 1] - 2.0 * trajectory[i] + trajectory[i + 1];
        -2.0 * second_diff / dt2
    }

    fn obstacle_gradient(&self, point: &Vector2<f64>) -> Vector2<f64> {
        let mut grad = Vector2::zeros();
        for obs in &self.obstacles {
            let center = Vector2::new(obs.x, obs.y);
            let delta = *point - center;
            let norm = delta.norm().max(EPS);
            let signed_distance = norm - (obs.radius + DEFAULT_ROBOT_RADIUS);
            if signed_distance < DEFAULT_INFLUENCE_DISTANCE {
                let direction = delta / norm;
                // Potential increases near/inside obstacles; gradient points toward
                // obstacle so gradient descent moves away.
                grad -= (DEFAULT_INFLUENCE_DISTANCE - signed_distance) * direction;
            }
        }
        grad
    }

    fn smoothness_cost(&self, trajectory: &[Vector2<f64>]) -> f64 {
        trajectory
            .windows(3)
            .map(|window| {
                let second_diff = window[0] - 2.0 * window[1] + window[2];
                second_diff.norm_squared() / (self.config.dt * self.config.dt)
            })
            .sum()
    }

    fn obstacle_cost(&self, trajectory: &[Vector2<f64>]) -> f64 {
        let mut cost = 0.0;
        for point in trajectory {
            for obs in &self.obstacles {
                let dx = point.x - obs.x;
                let dy = point.y - obs.y;
                let distance = (dx * dx + dy * dy).sqrt();
                let signed_distance = distance - (obs.radius + DEFAULT_ROBOT_RADIUS);
                if signed_distance < DEFAULT_INFLUENCE_DISTANCE {
                    let penetration = DEFAULT_INFLUENCE_DISTANCE - signed_distance;
                    cost += 0.5 * penetration * penetration;
                }
            }
        }
        cost
    }

    fn trajectory_cost(&self, trajectory: &[Vector2<f64>]) -> f64 {
        self.config.smoothness_weight * self.smoothness_cost(trajectory)
            + self.config.obstacle_cost_weight * self.obstacle_cost(trajectory)
    }
}

impl PathPlanner for ChompPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        self.optimize(start, goal).map(|result| result.path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rrt::{AreaBounds, RRTConfig, RRTNode, RRTPlanner};

    fn smoothness_metric(path: &Path2D) -> f64 {
        if path.points.len() < 3 {
            return 0.0;
        }
        path.points
            .windows(3)
            .map(|w| {
                let ax = w[0].x - 2.0 * w[1].x + w[2].x;
                let ay = w[0].y - 2.0 * w[1].y + w[2].y;
                ax * ax + ay * ay
            })
            .sum()
    }

    #[test]
    fn test_chomp_converges_and_keeps_endpoints() {
        let planner = ChompPlanner::new(
            vec![CircleObstacle::new(5.0, 2.0, 1.0)],
            ChompConfig {
                obstacle_cost_weight: 3.0,
                ..Default::default()
            },
        );
        let start = Point2D::new(0.0, 0.0);
        let goal = Point2D::new(10.0, 4.0);

        let initial = planner.initialize_trajectory(start, goal);
        let initial_cost = planner.trajectory_cost(&initial);
        let result = planner
            .optimize(start, goal)
            .expect("optimization should succeed");

        assert!(result.cost <= initial_cost);
        assert_eq!(result.path.points.first().unwrap().x, start.x);
        assert_eq!(result.path.points.first().unwrap().y, start.y);
        assert_eq!(result.path.points.last().unwrap().x, goal.x);
        assert_eq!(result.path.points.last().unwrap().y, goal.y);
    }

    #[test]
    fn test_chomp_avoids_central_obstacle() {
        let obstacles = vec![CircleObstacle::new(5.0, 0.0, 1.5)];
        let planner = ChompPlanner::new(
            obstacles.clone(),
            ChompConfig {
                max_iterations: 200,
                obstacle_cost_weight: 5.0,
                ..Default::default()
            },
        );
        let path = planner
            .plan(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0))
            .expect("planning should succeed");

        let min_dist = path
            .points
            .iter()
            .map(|p| ((p.x - obstacles[0].x).powi(2) + (p.y - obstacles[0].y).powi(2)).sqrt())
            .fold(f64::INFINITY, f64::min);

        assert!(
            min_dist > obstacles[0].radius,
            "optimized trajectory should bend around obstacle"
        );
    }

    #[test]
    fn test_chomp_path_is_smoother_than_rrt_path() {
        let mut rrt = RRTPlanner::new(
            vec![],
            AreaBounds::new(-2.0, 15.0, -5.0, 5.0),
            None,
            RRTConfig::default(),
        );
        let samples = [
            [2.0, 2.0],
            [4.0, -2.0],
            [6.0, 2.0],
            [8.0, -2.0],
            [10.0, 0.0],
            [10.0, 0.0],
        ];
        let mut idx = 0usize;
        let rrt_path = rrt
            .plan_with_sampler(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0), |_| {
                let sample = samples[idx.min(samples.len() - 1)];
                idx += 1;
                RRTNode::new(sample[0], sample[1])
            })
            .expect("rrt should find a path");

        let chomp = ChompPlanner::new(vec![], ChompConfig::default());
        let chomp_path = chomp
            .plan(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0))
            .expect("chomp should optimize a path");

        assert!(smoothness_metric(&chomp_path) < smoothness_metric(&rrt_path));
    }
}
