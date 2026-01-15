//! Pure Pursuit path tracking algorithm
//!
//! A geometric path tracking controller that computes the steering angle
//! to follow a reference path by looking ahead to a target point.

use crate::common::{Point2D, Path2D, State2D, ControlInput, PathTracker};

/// Vehicle state for Pure Pursuit
#[derive(Debug, Clone, Copy)]
pub struct VehicleState {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
    pub rear_x: f64,
    pub rear_y: f64,
    pub wheelbase: f64,
}

impl VehicleState {
    pub fn new(x: f64, y: f64, yaw: f64, v: f64, wheelbase: f64) -> Self {
        let rear_x = x - (wheelbase / 2.0) * yaw.cos();
        let rear_y = y - (wheelbase / 2.0) * yaw.sin();
        VehicleState { x, y, yaw, v, rear_x, rear_y, wheelbase }
    }

    pub fn update(&mut self, a: f64, delta: f64, dt: f64) {
        self.x += self.v * self.yaw.cos() * dt;
        self.y += self.v * self.yaw.sin() * dt;
        self.yaw += self.v / self.wheelbase * delta.tan() * dt;
        self.v += a * dt;
        self.rear_x = self.x - (self.wheelbase / 2.0) * self.yaw.cos();
        self.rear_y = self.y - (self.wheelbase / 2.0) * self.yaw.sin();
    }

    pub fn calc_distance(&self, px: f64, py: f64) -> f64 {
        let dx = self.rear_x - px;
        let dy = self.rear_y - py;
        (dx * dx + dy * dy).sqrt()
    }

    pub fn to_state2d(&self) -> State2D {
        State2D::new(self.x, self.y, self.yaw, self.v)
    }
}

impl From<State2D> for VehicleState {
    fn from(s: State2D) -> Self {
        VehicleState::new(s.x, s.y, s.yaw, s.v, 2.9) // default wheelbase
    }
}

/// Configuration for Pure Pursuit controller
#[derive(Debug, Clone)]
pub struct PurePursuitConfig {
    /// Look-ahead gain (k)
    pub look_ahead_gain: f64,
    /// Look-ahead distance offset (Lfc)
    pub look_ahead_distance: f64,
    /// Vehicle wheelbase
    pub wheelbase: f64,
    /// Speed proportional gain
    pub kp: f64,
    /// Goal distance threshold
    pub goal_threshold: f64,
}

impl Default for PurePursuitConfig {
    fn default() -> Self {
        Self {
            look_ahead_gain: 0.1,
            look_ahead_distance: 2.0,
            wheelbase: 2.9,
            kp: 1.0,
            goal_threshold: 2.0,
        }
    }
}

/// Pure Pursuit path tracking controller
pub struct PurePursuitController {
    config: PurePursuitConfig,
    path: Path2D,
    old_nearest_index: Option<usize>,
}

impl PurePursuitController {
    /// Create a new Pure Pursuit controller
    pub fn new(config: PurePursuitConfig) -> Self {
        PurePursuitController {
            config,
            path: Path2D::new(),
            old_nearest_index: None,
        }
    }

    /// Create with simplified parameters (legacy interface)
    pub fn with_params(k: f64, lfc: f64, wb: f64) -> Self {
        let config = PurePursuitConfig {
            look_ahead_gain: k,
            look_ahead_distance: lfc,
            wheelbase: wb,
            ..Default::default()
        };
        Self::new(config)
    }

    /// Set the reference path
    pub fn set_path(&mut self, path: Path2D) {
        self.path = path;
        self.old_nearest_index = None;
    }

    /// Get the current reference path
    pub fn get_path(&self) -> &Path2D {
        &self.path
    }

    /// Compute steering angle for current state
    pub fn compute_steering(&mut self, state: &VehicleState) -> f64 {
        let (target_idx, lf) = self.search_target_index(state);

        let (tx, ty) = if target_idx < self.path.len() {
            let p = &self.path.points[target_idx];
            (p.x, p.y)
        } else {
            let p = self.path.points.last().unwrap();
            (p.x, p.y)
        };

        let alpha = (ty - state.rear_y).atan2(tx - state.rear_x) - state.yaw;
        (2.0 * state.wheelbase * alpha.sin() / lf).atan2(1.0)
    }

    /// Search for target point index on path
    fn search_target_index(&mut self, state: &VehicleState) -> (usize, f64) {
        let mut ind = match self.old_nearest_index {
            None => {
                // Find nearest point
                let mut min_dist = f64::MAX;
                let mut min_idx = 0;
                for (i, p) in self.path.points.iter().enumerate() {
                    let d = state.calc_distance(p.x, p.y);
                    if d < min_dist {
                        min_dist = d;
                        min_idx = i;
                    }
                }
                min_idx
            }
            Some(prev_idx) => {
                // Search from previous index
                let mut idx = prev_idx;
                let mut dist = state.calc_distance(
                    self.path.points[idx].x,
                    self.path.points[idx].y
                );

                while idx + 1 < self.path.len() {
                    let next_dist = state.calc_distance(
                        self.path.points[idx + 1].x,
                        self.path.points[idx + 1].y
                    );
                    if dist < next_dist {
                        break;
                    }
                    idx += 1;
                    dist = next_dist;
                }
                idx
            }
        };

        self.old_nearest_index = Some(ind);

        // Calculate look-ahead distance
        let lf = self.config.look_ahead_gain * state.v + self.config.look_ahead_distance;

        // Find target point at look-ahead distance
        while ind < self.path.len() {
            let p = &self.path.points[ind];
            if state.calc_distance(p.x, p.y) >= lf {
                break;
            }
            if ind + 1 >= self.path.len() {
                break;
            }
            ind += 1;
        }

        (ind, lf)
    }

    /// Proportional speed control
    pub fn compute_acceleration(&self, target_speed: f64, current_speed: f64) -> f64 {
        self.config.kp * (target_speed - current_speed)
    }

    /// Check if goal is reached
    pub fn is_goal_reached(&self, state: &VehicleState) -> bool {
        if let Some(goal) = self.path.points.last() {
            let dx = state.x - goal.x;
            let dy = state.y - goal.y;
            (dx * dx + dy * dy).sqrt() < self.config.goal_threshold
        } else {
            true
        }
    }

    /// Simulate path tracking (legacy interface)
    pub fn planning(&mut self, waypoints: Vec<(f64, f64)>, target_speed: f64) -> Option<Vec<(f64, f64)>> {
        if waypoints.len() < 2 {
            return None;
        }

        // Set path
        let path = Path2D::from_points(
            waypoints.iter().map(|&(x, y)| Point2D::new(x, y)).collect()
        );
        self.set_path(path);

        // Initial state
        let init_pos = &self.path.points[0];
        let mut state = VehicleState::new(
            init_pos.x,
            init_pos.y - 3.0,
            0.0,
            0.0,
            self.config.wheelbase
        );

        let mut trajectory = vec![(state.x, state.y)];
        let dt = 0.1;
        let t_max = 100.0;
        let mut time = 0.0;

        while time < t_max {
            let ai = self.compute_acceleration(target_speed, state.v);
            let di = self.compute_steering(&state);
            state.update(ai, di, dt);
            time += dt;

            trajectory.push((state.x, state.y));

            if self.is_goal_reached(&state) {
                break;
            }
        }

        Some(trajectory)
    }
}

impl PathTracker for PurePursuitController {
    fn compute_control(&mut self, current_state: &State2D, path: &Path2D) -> ControlInput {
        // Set path if different
        if self.path.len() != path.len() {
            self.set_path(path.clone());
        }

        let vehicle_state = VehicleState::new(
            current_state.x,
            current_state.y,
            current_state.yaw,
            current_state.v,
            self.config.wheelbase
        );

        let delta = self.compute_steering(&vehicle_state);

        // Compute speed control (assume constant target speed for now)
        let target_speed = 5.0; // m/s
        let v = current_state.v + self.compute_acceleration(target_speed, current_state.v) * 0.1;

        // Convert steering angle to angular velocity
        let omega = v * delta.tan() / self.config.wheelbase;

        ControlInput::new(v, omega)
    }

    fn is_goal_reached(&self, current_state: &State2D, goal: Point2D) -> bool {
        let dx = current_state.x - goal.x;
        let dy = current_state.y - goal.y;
        (dx * dx + dy * dy).sqrt() < self.config.goal_threshold
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pure_pursuit_creation() {
        let config = PurePursuitConfig::default();
        let controller = PurePursuitController::new(config);
        assert!(controller.path.is_empty());
    }

    #[test]
    fn test_pure_pursuit_set_path() {
        let mut controller = PurePursuitController::with_params(0.1, 2.0, 2.9);
        let path = Path2D::from_points(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(2.0, 0.0),
        ]);
        controller.set_path(path);
        assert_eq!(controller.get_path().len(), 3);
    }

    #[test]
    fn test_pure_pursuit_planning() {
        let mut controller = PurePursuitController::with_params(0.1, 2.0, 2.9);
        let waypoints: Vec<(f64, f64)> = (0..10)
            .map(|i| (i as f64 * 2.0, 0.0))
            .collect();

        let result = controller.planning(waypoints, 5.0);
        assert!(result.is_some());
        assert!(result.unwrap().len() > 0);
    }
}
