//! Dynamic Window Approach (DWA) local planner
//!
//! Implements velocity-space local planning for mobile robots.
//! DWA searches the velocity space for admissible velocities and selects
//! the best trajectory based on goal direction, speed, and obstacle clearance.

use nalgebra::{Vector2, Vector4, Vector5};

use rust_robotics_core::{
    ControlInput, Obstacles, Path2D, Point2D, RoboticsError, RoboticsResult, State2D,
};

const ROBOT_STUCK_FLAG_CONS: f64 = 0.001;

fn snap_to_resolution(value: f64, resolution: f64) -> f64 {
    if !value.is_finite() || !resolution.is_finite() || resolution <= 0.0 {
        return value;
    }
    (value / resolution).round() * resolution
}

fn arange_samples(start: f64, stop: f64, step: f64) -> Vec<f64> {
    if !(start.is_finite() && stop.is_finite() && step.is_finite()) || step <= 0.0 || start >= stop
    {
        return Vec::new();
    }
    let count = ((stop - start) / step).ceil().max(0.0) as usize;
    (0..count)
        .map(|index| snap_to_resolution(start + step * index as f64, step))
        .collect()
}

/// Robot state for DWA: [x, y, yaw, v, omega]
pub type DWAState = Vector5<f64>;

/// Control input: [v, omega]
pub type DWAControl = Vector2<f64>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DWARobotType {
    #[default]
    Circle,
    Rectangle,
}

fn rectangle_collision(
    robot_x: f64,
    robot_y: f64,
    robot_yaw: f64,
    obstacle_x: f64,
    obstacle_y: f64,
    robot_width: f64,
    robot_length: f64,
) -> bool {
    let relative_x = obstacle_x - robot_x;
    let relative_y = obstacle_y - robot_y;
    let local_x = relative_x * robot_yaw.cos() + relative_y * robot_yaw.sin();
    let local_y = -relative_x * robot_yaw.sin() + relative_y * robot_yaw.cos();
    local_x <= robot_length / 2.0
        && local_x >= -robot_length / 2.0
        && local_y <= robot_width / 2.0
        && local_y >= -robot_width / 2.0
}

/// Configuration for DWA planner
#[derive(Debug, Clone)]
pub struct DWAConfig {
    pub max_speed: f64,
    pub min_speed: f64,
    pub max_yaw_rate: f64,
    pub max_accel: f64,
    pub max_delta_yaw_rate: f64,
    pub v_resolution: f64,
    pub yaw_rate_resolution: f64,
    pub dt: f64,
    pub predict_time: f64,
    pub to_goal_cost_gain: f64,
    pub speed_cost_gain: f64,
    pub obstacle_cost_gain: f64,
    pub robot_type: DWARobotType,
    pub robot_radius: f64,
    pub robot_width: f64,
    pub robot_length: f64,
    pub goal_threshold: f64,
}

impl Default for DWAConfig {
    fn default() -> Self {
        Self {
            max_speed: 1.0,
            min_speed: -0.5,
            max_yaw_rate: 40.0_f64.to_radians(),
            max_accel: 0.2,
            max_delta_yaw_rate: 40.0_f64.to_radians(),
            v_resolution: 0.01,
            yaw_rate_resolution: 0.1_f64.to_radians(),
            dt: 0.1,
            predict_time: 3.0,
            to_goal_cost_gain: 0.15,
            speed_cost_gain: 1.0,
            obstacle_cost_gain: 1.0,
            robot_type: DWARobotType::Circle,
            robot_radius: 1.0,
            robot_width: 0.5,
            robot_length: 1.2,
            goal_threshold: 1.0,
        }
    }
}

impl DWAConfig {
    pub fn validate(&self) -> RoboticsResult<()> {
        if !self.max_speed.is_finite()
            || !self.min_speed.is_finite()
            || self.min_speed > self.max_speed
        {
            return Err(RoboticsError::InvalidParameter(
                "DWA speed range must be finite and min_speed must be <= max_speed".to_string(),
            ));
        }
        if !self.max_yaw_rate.is_finite() || self.max_yaw_rate <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "DWA max_yaw_rate must be positive and finite".to_string(),
            ));
        }
        if !self.max_accel.is_finite() || self.max_accel <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "DWA max_accel must be positive and finite".to_string(),
            ));
        }
        if !self.max_delta_yaw_rate.is_finite() || self.max_delta_yaw_rate <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "DWA max_delta_yaw_rate must be positive and finite".to_string(),
            ));
        }
        if !self.v_resolution.is_finite() || self.v_resolution <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "DWA v_resolution must be positive and finite".to_string(),
            ));
        }
        if !self.yaw_rate_resolution.is_finite() || self.yaw_rate_resolution <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "DWA yaw_rate_resolution must be positive and finite".to_string(),
            ));
        }
        if !self.dt.is_finite() || self.dt <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "DWA dt must be positive and finite".to_string(),
            ));
        }
        if !self.predict_time.is_finite() || self.predict_time <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "DWA predict_time must be positive and finite".to_string(),
            ));
        }
        if !self.to_goal_cost_gain.is_finite() || self.to_goal_cost_gain < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "DWA to_goal_cost_gain must be non-negative and finite".to_string(),
            ));
        }
        if !self.speed_cost_gain.is_finite() || self.speed_cost_gain < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "DWA speed_cost_gain must be non-negative and finite".to_string(),
            ));
        }
        if !self.obstacle_cost_gain.is_finite() || self.obstacle_cost_gain < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "DWA obstacle_cost_gain must be non-negative and finite".to_string(),
            ));
        }
        if !self.robot_width.is_finite() || self.robot_width < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "DWA robot_width must be non-negative and finite".to_string(),
            ));
        }
        if !self.robot_length.is_finite() || self.robot_length < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "DWA robot_length must be non-negative and finite".to_string(),
            ));
        }
        if self.robot_type == DWARobotType::Rectangle
            && (self.robot_width <= 0.0 || self.robot_length <= 0.0)
        {
            return Err(RoboticsError::InvalidParameter(
                "DWA rectangle robot dimensions must be positive".to_string(),
            ));
        }
        if !self.robot_radius.is_finite() || self.robot_radius < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "DWA robot_radius must be non-negative and finite".to_string(),
            ));
        }
        if !self.goal_threshold.is_finite() || self.goal_threshold < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "DWA goal_threshold must be non-negative and finite".to_string(),
            ));
        }
        Ok(())
    }
}

/// Dynamic window bounds: [v_min, v_max, yaw_rate_min, yaw_rate_max]
pub type DynamicWindow = Vector4<f64>;

/// Predicted trajectory
#[derive(Debug, Clone)]
pub struct Trajectory {
    pub states: Vec<DWAState>,
    pub control: DWAControl,
    pub cost: f64,
}

impl Trajectory {
    pub fn new() -> Self {
        Self {
            states: Vec::new(),
            control: DWAControl::zeros(),
            cost: f64::MAX,
        }
    }

    pub fn final_state(&self) -> Option<&DWAState> {
        self.states.last()
    }

    pub fn to_path(&self) -> Path2D {
        let points: Vec<Point2D> = self
            .states
            .iter()
            .map(|s| Point2D::new(s[0], s[1]))
            .collect();
        Path2D::from_points(points)
    }
}

impl Default for Trajectory {
    fn default() -> Self {
        Self::new()
    }
}

/// Dynamic Window Approach local planner
pub struct DWAPlanner {
    config: DWAConfig,
    state: DWAState,
    goal: Point2D,
    obstacles: Vec<Point2D>,
    best_trajectory: Trajectory,
}

impl DWAPlanner {
    pub fn new(config: DWAConfig) -> Self {
        Self::try_new(config).expect("invalid DWA configuration")
    }

    pub fn try_new(config: DWAConfig) -> RoboticsResult<Self> {
        config.validate()?;
        Ok(Self {
            config,
            state: DWAState::zeros(),
            goal: Point2D::origin(),
            obstacles: Vec::new(),
            best_trajectory: Trajectory::new(),
        })
    }

    pub fn with_defaults() -> Self {
        Self::new(DWAConfig::default())
    }

    pub fn set_state(&mut self, state: DWAState) {
        self.try_set_state(state)
            .expect("DWA state must contain only finite values")
    }

    pub fn try_set_state(&mut self, state: DWAState) -> RoboticsResult<()> {
        Self::validate_state(&state)?;
        self.state = state;
        Ok(())
    }

    pub fn set_state_from_2d(&mut self, state: &State2D, omega: f64) {
        self.try_set_state_from_2d(state, omega)
            .expect("DWA state must contain only finite values")
    }

    pub fn try_set_state_from_2d(&mut self, state: &State2D, omega: f64) -> RoboticsResult<()> {
        self.try_set_state(DWAState::new(state.x, state.y, state.yaw, state.v, omega))
    }

    pub fn get_state(&self) -> &DWAState {
        &self.state
    }

    pub fn state_2d(&self) -> State2D {
        State2D::new(self.state[0], self.state[1], self.state[2], self.state[3])
    }

    pub fn set_goal(&mut self, goal: Point2D) {
        self.try_set_goal(goal)
            .expect("DWA goal must contain only finite values")
    }

    pub fn try_set_goal(&mut self, goal: Point2D) -> RoboticsResult<()> {
        Self::validate_goal(&goal)?;
        self.goal = goal;
        Ok(())
    }

    pub fn set_obstacles(&mut self, obstacles: Vec<Point2D>) {
        self.try_set_obstacles(obstacles)
            .expect("DWA obstacles must contain only finite values")
    }

    pub fn try_set_obstacles(&mut self, obstacles: Vec<Point2D>) -> RoboticsResult<()> {
        Self::validate_obstacles(&obstacles)?;
        self.obstacles = obstacles;
        Ok(())
    }

    pub fn set_obstacles_from_tuples(&mut self, obstacles: &[(f64, f64)]) {
        self.try_set_obstacles(
            obstacles
                .iter()
                .map(|(x, y)| Point2D::new(*x, *y))
                .collect(),
        )
        .expect("DWA obstacles must contain only finite values")
    }

    pub fn set_obstacles_from_obstacles(&mut self, obstacles: &Obstacles) -> RoboticsResult<()> {
        self.try_set_obstacles(obstacles.points.clone())
    }

    pub fn get_best_trajectory(&self) -> &Trajectory {
        &self.best_trajectory
    }
    pub fn best_path(&self) -> Path2D {
        self.best_trajectory.to_path()
    }
    pub fn config(&self) -> &DWAConfig {
        &self.config
    }

    fn motion(&self, state: &DWAState, control: &DWAControl) -> DWAState {
        let dt = self.config.dt;
        let mut next = *state;
        next[2] += control[1] * dt;
        next[0] += control[0] * next[2].cos() * dt;
        next[1] += control[0] * next[2].sin() * dt;
        next[3] = control[0];
        next[4] = control[1];
        next
    }

    fn calc_dynamic_window(&self) -> DynamicWindow {
        let config = &self.config;
        let state = &self.state;
        let vs = Vector4::new(
            config.min_speed,
            config.max_speed,
            -config.max_yaw_rate,
            config.max_yaw_rate,
        );
        let vd = Vector4::new(
            state[3] - config.max_accel * config.dt,
            state[3] + config.max_accel * config.dt,
            state[4] - config.max_delta_yaw_rate * config.dt,
            state[4] + config.max_delta_yaw_rate * config.dt,
        );
        DynamicWindow::new(
            vs[0].max(vd[0]),
            vs[1].min(vd[1]),
            vs[2].max(vd[2]),
            vs[3].min(vd[3]),
        )
    }

    fn predict_trajectory(&self, v: f64, omega: f64) -> Trajectory {
        let config = &self.config;
        let mut state = self.state;
        let control = DWAControl::new(v, omega);
        let mut states = Vec::new();
        states.push(state);
        let mut time = 0.0;
        while time <= config.predict_time {
            state = self.motion(&state, &control);
            states.push(state);
            time += config.dt;
        }
        Trajectory {
            states,
            control,
            cost: 0.0,
        }
    }

    fn calc_to_goal_cost(&self, trajectory: &Trajectory) -> f64 {
        if let Some(final_state) = trajectory.final_state() {
            let dx = self.goal.x - final_state[0];
            let dy = self.goal.y - final_state[1];
            let target_angle = dy.atan2(dx);
            let heading_error = target_angle - final_state[2];
            heading_error.sin().atan2(heading_error.cos()).abs()
        } else {
            f64::MAX
        }
    }

    fn calc_speed_cost(&self, trajectory: &Trajectory) -> f64 {
        if let Some(final_state) = trajectory.final_state() {
            self.config.max_speed - final_state[3]
        } else {
            f64::MAX
        }
    }

    fn calc_obstacle_cost(&self, trajectory: &Trajectory) -> f64 {
        if self.obstacles.is_empty() {
            return 0.0;
        }
        let mut min_dist = f64::MAX;
        for state in &trajectory.states {
            for obs in &self.obstacles {
                let dx = state[0] - obs.x;
                let dy = state[1] - obs.y;
                let dist = (dx * dx + dy * dy).sqrt();
                let collision = match self.config.robot_type {
                    DWARobotType::Circle => dist <= self.config.robot_radius,
                    DWARobotType::Rectangle => rectangle_collision(
                        state[0],
                        state[1],
                        state[2],
                        obs.x,
                        obs.y,
                        self.config.robot_width,
                        self.config.robot_length,
                    ),
                };
                if collision {
                    return f64::MAX;
                }
                if dist < min_dist {
                    min_dist = dist;
                }
            }
        }
        if min_dist < f64::MAX {
            1.0 / min_dist
        } else {
            0.0
        }
    }

    pub fn plan_step(&mut self) -> DWAControl {
        self.try_plan_step().expect("invalid DWA planning state")
    }

    pub fn try_plan_step(&mut self) -> RoboticsResult<DWAControl> {
        self.config.validate()?;
        Self::validate_state(&self.state)?;
        Self::validate_goal(&self.goal)?;
        Self::validate_obstacles(&self.obstacles)?;

        let config = &self.config;
        let dw = self.calc_dynamic_window();
        let mut best_trajectory = Trajectory {
            states: vec![self.state],
            control: DWAControl::zeros(),
            cost: f64::MAX,
        };
        let mut min_cost = f64::MAX;

        for v in arange_samples(dw[0], dw[1], config.v_resolution) {
            for omega in arange_samples(dw[2], dw[3], config.yaw_rate_resolution) {
                let trajectory = self.predict_trajectory(v, omega);
                let goal_cost = config.to_goal_cost_gain * self.calc_to_goal_cost(&trajectory);
                let speed_cost = config.speed_cost_gain * self.calc_speed_cost(&trajectory);
                let obstacle_cost =
                    config.obstacle_cost_gain * self.calc_obstacle_cost(&trajectory);
                let total_cost = goal_cost + speed_cost + obstacle_cost;
                if total_cost <= min_cost {
                    min_cost = total_cost;
                    let mut control = DWAControl::new(v, omega);
                    if control[0].abs() < ROBOT_STUCK_FLAG_CONS
                        && self.state[3].abs() < ROBOT_STUCK_FLAG_CONS
                    {
                        control[1] = -config.max_delta_yaw_rate;
                    }
                    best_trajectory = Trajectory {
                        states: trajectory.states,
                        control,
                        cost: total_cost,
                    };
                }
            }
        }

        self.best_trajectory = best_trajectory;
        Ok(self.best_trajectory.control)
    }

    pub fn step(&mut self) -> DWAControl {
        self.try_step().expect("invalid DWA step")
    }

    pub fn try_step(&mut self) -> RoboticsResult<DWAControl> {
        let control = self.try_plan_step()?;
        self.state = self.motion(&self.state, &control);
        Ok(control)
    }

    pub fn is_goal_reached(&self) -> bool {
        let dx = self.state[0] - self.goal.x;
        let dy = self.state[1] - self.goal.y;
        (dx * dx + dy * dy).sqrt() <= self.config.goal_threshold
    }

    pub fn distance_to_goal(&self) -> f64 {
        let dx = self.state[0] - self.goal.x;
        let dy = self.state[1] - self.goal.y;
        (dx * dx + dy * dy).sqrt()
    }

    pub fn navigate_to_goal(&mut self, max_steps: usize) -> Vec<DWAState> {
        self.try_navigate_to_goal(max_steps)
            .expect("invalid DWA navigation")
    }

    pub fn try_navigate_to_goal(&mut self, max_steps: usize) -> RoboticsResult<Vec<DWAState>> {
        let mut path = vec![self.state];
        for _ in 0..max_steps {
            if self.is_goal_reached() {
                break;
            }
            self.try_step()?;
            path.push(self.state);
        }
        Ok(path)
    }

    pub fn try_plan_input(&mut self) -> RoboticsResult<ControlInput> {
        let control = self.try_plan_step()?;
        Ok(Self::control_to_input(&control))
    }

    pub fn control_to_input(control: &DWAControl) -> ControlInput {
        ControlInput::new(control[0], control[1])
    }

    fn validate_state(state: &DWAState) -> RoboticsResult<()> {
        if state.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "DWA state must contain only finite values".to_string(),
            ));
        }
        Ok(())
    }

    fn validate_goal(goal: &Point2D) -> RoboticsResult<()> {
        if !goal.x.is_finite() || !goal.y.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "DWA goal must contain only finite values".to_string(),
            ));
        }
        Ok(())
    }

    fn validate_obstacles(obstacles: &[Point2D]) -> RoboticsResult<()> {
        if obstacles
            .iter()
            .any(|o| !o.x.is_finite() || !o.y.is_finite())
        {
            return Err(RoboticsError::InvalidParameter(
                "DWA obstacles must contain only finite values".to_string(),
            ));
        }
        Ok(())
    }
}

// Legacy interface
#[derive(Debug, Clone)]
#[deprecated(note = "Use DWAConfig instead")]
pub struct Config {
    pub max_speed: f64,
    pub min_speed: f64,
    pub max_yaw_rate: f64,
    pub max_accel: f64,
    pub max_delta_yaw_rate: f64,
    pub v_resolution: f64,
    pub yaw_rate_resolution: f64,
    pub dt: f64,
    pub predict_time: f64,
    pub to_goal_cost_gain: f64,
    pub speed_cost_gain: f64,
    pub obstacle_cost_gain: f64,
    pub robot_type: DWARobotType,
    pub robot_radius: f64,
    pub robot_width: f64,
    pub robot_length: f64,
}

pub fn motion(mut x: Vector5<f64>, u: Vector2<f64>, dt: f64) -> Vector5<f64> {
    x[2] += u[1] * dt;
    x[0] += u[0] * x[2].cos() * dt;
    x[1] += u[0] * x[2].sin() * dt;
    x[3] = u[0];
    x[4] = u[1];
    x
}

#[allow(deprecated)]
pub fn calc_dynamic_window(x: Vector5<f64>, config: &Config) -> Vector4<f64> {
    let vs = Vector4::new(
        config.min_speed,
        config.max_speed,
        -config.max_yaw_rate,
        config.max_yaw_rate,
    );
    let vd = Vector4::new(
        x[3] - config.max_accel * config.dt,
        x[3] + config.max_accel * config.dt,
        x[4] - config.max_delta_yaw_rate * config.dt,
        x[4] + config.max_delta_yaw_rate * config.dt,
    );
    Vector4::new(
        vs[0].max(vd[0]),
        vs[1].min(vd[1]),
        vs[2].max(vd[2]),
        vs[3].min(vd[3]),
    )
}

#[allow(deprecated)]
pub fn predict_trajectory(
    x_init: Vector5<f64>,
    v: f64,
    y: f64,
    config: &Config,
) -> Vec<Vector5<f64>> {
    let mut x = x_init;
    let mut trajectory = vec![x_init];
    let mut time = 0.0;
    while time <= config.predict_time {
        x = motion(x, Vector2::new(v, y), config.dt);
        trajectory.push(x);
        time += config.dt;
    }
    trajectory
}

#[allow(deprecated)]
pub fn calc_obstacle_cost(trajectory: &[Vector5<f64>], ob: &[(f64, f64)], config: &Config) -> f64 {
    if trajectory.is_empty() || ob.is_empty() {
        return 0.0;
    }
    let mut min_r = f64::MAX;
    for state in trajectory {
        for obstacle in ob {
            let r = ((state[0] - obstacle.0).powi(2) + (state[1] - obstacle.1).powi(2)).sqrt();
            let collision = match config.robot_type {
                DWARobotType::Circle => r <= config.robot_radius,
                DWARobotType::Rectangle => rectangle_collision(
                    state[0],
                    state[1],
                    state[2],
                    obstacle.0,
                    obstacle.1,
                    config.robot_width,
                    config.robot_length,
                ),
            };
            if collision {
                return f64::MAX;
            }
            if r < min_r {
                min_r = r;
            }
        }
    }
    if min_r < f64::MAX {
        1.0 / min_r
    } else {
        0.0
    }
}

pub fn calc_to_goal_cost(trajectory: &[Vector5<f64>], goal: (f64, f64)) -> f64 {
    if trajectory.is_empty() {
        return f64::MAX;
    }
    let last = &trajectory[trajectory.len() - 1];
    let dx = goal.0 - last[0];
    let dy = goal.1 - last[1];
    let error_angle = dy.atan2(dx);
    let cost_angle = error_angle - last[2];
    cost_angle.sin().atan2(cost_angle.cos()).abs()
}

#[allow(deprecated)]
pub fn dwa_control(
    x: Vector5<f64>,
    config: &Config,
    goal: (f64, f64),
    ob: &[(f64, f64)],
) -> ((f64, f64), Vec<Vector5<f64>>) {
    let dw = calc_dynamic_window(x, config);
    calc_control_and_trajectory(x, dw, config, goal, ob)
}

#[allow(deprecated)]
pub fn calc_control_and_trajectory(
    x: Vector5<f64>,
    dw: Vector4<f64>,
    config: &Config,
    goal: (f64, f64),
    ob: &[(f64, f64)],
) -> ((f64, f64), Vec<Vector5<f64>>) {
    let mut min_cost = f64::MAX;
    let mut best_u = (0.0, 0.0);
    let mut best_trajectory = vec![x];
    for v in arange_samples(dw[0], dw[1], config.v_resolution) {
        for y in arange_samples(dw[2], dw[3], config.yaw_rate_resolution) {
            let trajectory = predict_trajectory(x, v, y, config);
            let to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(&trajectory, goal);
            let speed_cost =
                config.speed_cost_gain * (config.max_speed - trajectory.last().unwrap()[3]);
            let ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(&trajectory, ob, config);
            let final_cost = to_goal_cost + speed_cost + ob_cost;
            if final_cost <= min_cost {
                min_cost = final_cost;
                best_u = (v, y);
                best_trajectory = trajectory;
                if best_u.0.abs() < ROBOT_STUCK_FLAG_CONS && x[3].abs() < ROBOT_STUCK_FLAG_CONS {
                    best_u.1 = -config.max_delta_yaw_rate;
                }
            }
        }
    }
    (best_u, best_trajectory)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Copy)]
    struct ClosedLoopSample {
        control: [f64; 2],
        state: [f64; 5],
        predicted_len: usize,
        predicted_last: [f64; 5],
        dist: f64,
    }

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1.0e-12,
            "expected {expected}, got {actual}"
        );
    }

    fn assert_close_tol(actual: f64, expected: f64, tol: f64) {
        assert!(
            (actual - expected).abs() < tol,
            "expected {expected}, got {actual} (tol={tol})"
        );
    }

    fn assert_state_close(actual: &DWAState, expected: &[f64; 5]) {
        for (value, expected) in actual.iter().zip(expected.iter()) {
            assert_close(*value, *expected);
        }
    }

    fn assert_state_close_tol(actual: &[f64; 5], expected: &[f64; 5], tol: f64) {
        for (value, expected) in actual.iter().zip(expected.iter()) {
            assert_close_tol(*value, *expected, tol);
        }
    }

    fn assert_sample_close(actual: &ClosedLoopSample, expected: &ClosedLoopSample) {
        assert_close_tol(actual.control[0], expected.control[0], 0.05);
        assert_close_tol(actual.control[1], expected.control[1], 0.05);
        assert_state_close_tol(&actual.state, &expected.state, 0.25);
        assert_eq!(actual.predicted_len, expected.predicted_len);
        assert_state_close_tol(&actual.predicted_last, &expected.predicted_last, 0.25);
        assert_close_tol(actual.dist, expected.dist, 0.2);
    }

    fn to_state_array(state: &DWAState) -> [f64; 5] {
        [state[0], state[1], state[2], state[3], state[4]]
    }

    fn run_closed_loop_trace(
        mut dwa: DWAPlanner,
        max_steps: usize,
    ) -> RoboticsResult<Vec<ClosedLoopSample>> {
        let mut trace = Vec::new();
        for _step in 0..max_steps {
            let control = dwa.try_plan_step()?;
            let predicted = dwa.get_best_trajectory();
            let predicted_len = predicted.states.len();
            let predicted_last = to_state_array(predicted.final_state().unwrap());
            let next_state = dwa.motion(&dwa.state, &control);
            dwa.state = next_state;
            trace.push(ClosedLoopSample {
                control: [control[0], control[1]],
                state: to_state_array(&dwa.state),
                predicted_len,
                predicted_last,
                dist: dwa.distance_to_goal(),
            });
            if dwa.is_goal_reached() {
                break;
            }
        }
        Ok(trace)
    }

    fn pythonrobotics_default_obstacles() -> Vec<Point2D> {
        vec![
            Point2D::new(-1.0, -1.0),
            Point2D::new(0.0, 2.0),
            Point2D::new(4.0, 2.0),
            Point2D::new(5.0, 4.0),
            Point2D::new(5.0, 5.0),
            Point2D::new(5.0, 6.0),
            Point2D::new(5.0, 9.0),
            Point2D::new(8.0, 9.0),
            Point2D::new(7.0, 9.0),
            Point2D::new(8.0, 10.0),
            Point2D::new(9.0, 11.0),
            Point2D::new(12.0, 13.0),
            Point2D::new(12.0, 12.0),
            Point2D::new(15.0, 15.0),
            Point2D::new(13.0, 13.0),
        ]
    }

    fn pythonrobotics_stuck_obstacles() -> Vec<Point2D> {
        vec![
            Point2D::new(1.0, 1.0),
            Point2D::new(-0.0, -2.0),
            Point2D::new(-2.0, -6.0),
            Point2D::new(-2.0, -8.0),
            Point2D::new(-3.0, -9.27),
            Point2D::new(-3.79, -9.39),
            Point2D::new(-7.25, -8.97),
            Point2D::new(-7.0, -2.0),
            Point2D::new(-3.0, -4.0),
            Point2D::new(-6.0, -5.0),
            Point2D::new(-3.5, -5.8),
            Point2D::new(-6.0, -9.0),
            Point2D::new(-8.8, -9.0),
            Point2D::new(-5.0, -9.0),
            Point2D::new(-7.5, -3.0),
            Point2D::new(-9.0, -8.0),
            Point2D::new(-5.8, -4.4),
            Point2D::new(-12.0, -12.0),
            Point2D::new(-3.0, -2.0),
            Point2D::new(-13.0, -13.0),
        ]
    }

    #[test]
    fn test_dwa_creation() {
        let dwa = DWAPlanner::with_defaults();
        assert_eq!(dwa.state, DWAState::zeros());
    }

    #[test]
    fn test_dwa_config_default() {
        let config = DWAConfig::default();
        assert_eq!(config.max_speed, 1.0);
        assert_eq!(config.robot_type, DWARobotType::Circle);
        assert_eq!(config.robot_radius, 1.0);
        assert_eq!(config.robot_width, 0.5);
        assert_eq!(config.robot_length, 1.2);
    }

    #[test]
    fn test_dwa_set_goal() {
        let mut dwa = DWAPlanner::with_defaults();
        dwa.set_goal(Point2D::new(10.0, 10.0));
        assert_eq!(dwa.goal.x, 10.0);
        assert_eq!(dwa.goal.y, 10.0);
    }

    #[test]
    fn test_dwa_motion() {
        let dwa = DWAPlanner::with_defaults();
        let state = DWAState::new(0.0, 0.0, 0.0, 0.0, 0.0);
        let control = DWAControl::new(1.0, 0.0);
        let next = dwa.motion(&state, &control);
        assert!(next[0] > 0.0);
        assert_eq!(next[3], 1.0);
    }

    #[test]
    fn test_dwa_dynamic_window() {
        let mut dwa = DWAPlanner::with_defaults();
        dwa.set_state(DWAState::new(0.0, 0.0, 0.0, 0.5, 0.0));
        let dw = dwa.calc_dynamic_window();
        assert!(dw[0] >= dwa.config.min_speed);
        assert!(dw[1] <= dwa.config.max_speed);
    }

    #[test]
    fn test_dwa_predict_trajectory() {
        let dwa = DWAPlanner::with_defaults();
        let trajectory = dwa.predict_trajectory(1.0, 0.0);
        assert!(!trajectory.states.is_empty());
        let final_state = trajectory.final_state().unwrap();
        assert!(final_state[0] > 0.0);
    }

    #[test]
    fn test_dwa_obstacle_cost() {
        let mut dwa = DWAPlanner::with_defaults();
        dwa.set_obstacles(vec![Point2D::new(100.0, 100.0)]);
        let trajectory = dwa.predict_trajectory(1.0, 0.0);
        let cost = dwa.calc_obstacle_cost(&trajectory);
        assert!(cost < 1.0);
    }

    #[test]
    fn test_dwa_collision_detection() {
        let mut dwa = DWAPlanner::with_defaults();
        dwa.set_obstacles(vec![Point2D::new(0.5, 0.0)]);
        let trajectory = dwa.predict_trajectory(1.0, 0.0);
        let cost = dwa.calc_obstacle_cost(&trajectory);
        assert_eq!(cost, f64::MAX);
    }

    #[test]
    fn test_dwa_plan_step() {
        let mut dwa = DWAPlanner::with_defaults();
        dwa.set_state(DWAState::new(0.0, 0.0, 0.0, 0.0, 0.0));
        dwa.set_goal(Point2D::new(10.0, 0.0));
        let control = dwa.plan_step();
        assert!(control[0] > 0.0);
    }

    #[test]
    fn test_dwa_goal_reached() {
        let mut dwa = DWAPlanner::with_defaults();
        dwa.set_state(DWAState::new(10.0, 10.0, 0.0, 0.0, 0.0));
        dwa.set_goal(Point2D::new(10.0, 10.0));
        assert!(dwa.is_goal_reached());
    }

    #[test]
    fn test_dwa_navigate() {
        let mut dwa = DWAPlanner::with_defaults();
        dwa.set_state(DWAState::new(0.0, 0.0, 0.0, 0.0, 0.0));
        dwa.set_goal(Point2D::new(3.0, 0.0));
        let path = dwa.navigate_to_goal(100);
        assert!(path.len() > 1);
        let final_state = path.last().unwrap();
        let dist = ((final_state[0] - 3.0).powi(2) + final_state[1].powi(2)).sqrt();
        assert!(dist <= dwa.config.goal_threshold);
    }

    #[test]
    fn test_trajectory_to_path() {
        let dwa = DWAPlanner::with_defaults();
        let trajectory = dwa.predict_trajectory(1.0, 0.1);
        let path = trajectory.to_path();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_legacy_motion() {
        let x = Vector5::new(0.0, 0.0, 0.0, 0.0, 0.0);
        let u = Vector2::new(1.0, 0.0);
        let next = motion(x, u, 0.1);
        assert!(next[0] > 0.0);
    }

    #[test]
    fn test_dwa_try_new_rejects_invalid_config() {
        let config = DWAConfig {
            dt: 0.0,
            ..Default::default()
        };
        let err = match DWAPlanner::try_new(config) {
            Ok(_) => panic!("expected invalid config to be rejected"),
            Err(err) => err,
        };
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_dwa_state_2d_matches_internal_state() {
        let mut dwa = DWAPlanner::with_defaults();
        dwa.set_state(DWAState::new(1.0, 2.0, 0.3, 0.4, 0.1));
        let state = dwa.state_2d();
        assert_eq!(state.x, 1.0);
        assert_eq!(state.y, 2.0);
        assert_eq!(state.yaw, 0.3);
        assert_eq!(state.v, 0.4);
    }

    #[test]
    fn test_dwa_set_obstacles_from_obstacles() {
        let mut dwa = DWAPlanner::with_defaults();
        let obstacles =
            Obstacles::from_points(vec![Point2D::new(1.0, 1.0), Point2D::new(2.0, 2.0)]);
        dwa.set_obstacles_from_obstacles(&obstacles).unwrap();
        assert_eq!(dwa.obstacles.len(), 2);
    }

    #[test]
    fn test_dwa_try_plan_input_returns_common_control() {
        let mut dwa = DWAPlanner::with_defaults();
        dwa.set_state(DWAState::new(0.0, 0.0, 0.0, 0.0, 0.0));
        dwa.set_goal(Point2D::new(10.0, 0.0));
        let control = dwa.try_plan_input().unwrap();
        assert!(control.v > 0.0);
    }

    #[test]
    fn test_dwa_best_path_after_planning() {
        let mut dwa = DWAPlanner::with_defaults();
        dwa.set_state(DWAState::new(0.0, 0.0, 0.0, 0.0, 0.0));
        dwa.set_goal(Point2D::new(3.0, 0.0));
        dwa.try_plan_step().unwrap();
        let best_path = dwa.best_path();
        assert!(!best_path.is_empty());
    }

    #[test]
    fn test_legacy_obstacle_cost_checks_all_states_and_obstacles() {
        #[allow(deprecated)]
        let config = Config {
            max_speed: 1.0,
            min_speed: -0.5,
            max_yaw_rate: 40.0_f64.to_radians(),
            max_accel: 0.2,
            max_delta_yaw_rate: 40.0_f64.to_radians(),
            v_resolution: 0.01,
            yaw_rate_resolution: 0.1_f64.to_radians(),
            dt: 0.1,
            predict_time: 3.0,
            to_goal_cost_gain: 0.15,
            speed_cost_gain: 1.0,
            obstacle_cost_gain: 1.0,
            robot_type: DWARobotType::Circle,
            robot_radius: 0.5,
            robot_width: 0.5,
            robot_length: 1.2,
        };
        let trajectory = vec![
            Vector5::new(0.0, 0.0, 0.0, 0.0, 0.0),
            Vector5::new(2.0, 2.0, 0.0, 0.0, 0.0),
        ];
        let obstacles = [(10.0, 10.0), (2.0, 2.0)];
        #[allow(deprecated)]
        let cost = calc_obstacle_cost(&trajectory, &obstacles, &config);
        assert_eq!(cost, f64::MAX);
    }

    #[test]
    fn test_dwa_rectangle_collision_detection_respects_local_frame() {
        let mut dwa = DWAPlanner::new(DWAConfig {
            robot_type: DWARobotType::Rectangle,
            robot_radius: 0.2,
            robot_width: 0.5,
            robot_length: 1.2,
            ..DWAConfig::default()
        });
        dwa.set_state(DWAState::new(
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_2,
            0.0,
            0.0,
        ));
        dwa.set_obstacles(vec![Point2D::new(0.0, 0.55)]);

        let trajectory = Trajectory {
            states: vec![*dwa.get_state()],
            control: DWAControl::zeros(),
            cost: 0.0,
        };
        let cost = dwa.calc_obstacle_cost(&trajectory);
        assert_eq!(cost, f64::MAX);

        #[allow(deprecated)]
        let legacy_config = Config {
            max_speed: 1.0,
            min_speed: -0.5,
            max_yaw_rate: 40.0_f64.to_radians(),
            max_accel: 0.2,
            max_delta_yaw_rate: 40.0_f64.to_radians(),
            v_resolution: 0.01,
            yaw_rate_resolution: 0.1_f64.to_radians(),
            dt: 0.1,
            predict_time: 3.0,
            to_goal_cost_gain: 0.15,
            speed_cost_gain: 1.0,
            obstacle_cost_gain: 1.0,
            robot_type: DWARobotType::Rectangle,
            robot_radius: 0.2,
            robot_width: 0.5,
            robot_length: 1.2,
        };
        #[allow(deprecated)]
        let legacy_cost = calc_obstacle_cost(&trajectory.states, &[(0.0, 0.55)], &legacy_config);
        assert_eq!(legacy_cost, f64::MAX);
    }

    #[test]
    fn test_dwa_default_main_initial_parity_matches_pythonrobotics() {
        let mut dwa = DWAPlanner::with_defaults();
        dwa.set_state(DWAState::new(
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_8,
            0.0,
            0.0,
        ));
        dwa.set_goal(Point2D::new(10.0, 10.0));
        dwa.set_obstacles(pythonrobotics_default_obstacles());

        let control = dwa.try_plan_step().unwrap();
        assert_close(control[0], 0.02);
        assert_close(control[1], 0.068_067_840_827_779_04);

        let trajectory = &dwa.get_best_trajectory().states;
        assert_eq!(trajectory.len(), 31);
        assert_state_close(
            &trajectory[0],
            &[0.0, 0.0, std::f64::consts::FRAC_PI_8, 0.0, 0.0],
        );
        assert_state_close(
            &trajectory[1],
            &[
                0.001_842_506_612_952_407,
                0.000_777_926_334_061_682_8,
                0.399_505_865_781_502_05,
                0.02,
                0.068_067_840_827_779_04,
            ],
        );
        assert_state_close(
            &trajectory[2],
            &[
                0.003_679_675_406_577_515,
                0.001_568_376_094_470_316,
                0.406_312_649_864_279_95,
                0.02,
                0.068_067_840_827_779_04,
            ],
        );
        assert_state_close(
            &trajectory[5],
            &[
                0.009_158_304_932_256_946,
                0.004_014_496_810_584_791,
                0.426_733_002_112_613_67,
                0.02,
                0.068_067_840_827_779_04,
            ],
        );
        assert_state_close(
            &trajectory[10],
            &[
                0.018_174_703_618_670_7,
                0.008_338_301_686_158_353,
                0.460_766_922_526_503_2,
                0.02,
                0.068_067_840_827_779_04,
            ],
        );
        assert_state_close(
            &trajectory[20],
            &[
                0.035_740_187_678_676_45,
                0.017_893_451_940_005_592,
                0.528_834_763_354_282_3,
                0.02,
                0.068_067_840_827_779_04,
            ],
        );
        assert_state_close(
            &trajectory[30],
            &[
                0.052_615_098_653_616_07,
                0.028_621_196_634_198_105,
                0.596_902_604_182_061_3,
                0.02,
                0.068_067_840_827_779_04,
            ],
        );

        #[allow(deprecated)]
        let legacy_config = Config {
            max_speed: 1.0,
            min_speed: -0.5,
            max_yaw_rate: 40.0_f64.to_radians(),
            max_accel: 0.2,
            max_delta_yaw_rate: 40.0_f64.to_radians(),
            v_resolution: 0.01,
            yaw_rate_resolution: 0.1_f64.to_radians(),
            dt: 0.1,
            predict_time: 3.0,
            to_goal_cost_gain: 0.15,
            speed_cost_gain: 1.0,
            obstacle_cost_gain: 1.0,
            robot_type: DWARobotType::Circle,
            robot_radius: 1.0,
            robot_width: 0.5,
            robot_length: 1.2,
        };
        let obstacle_tuples: Vec<_> = pythonrobotics_default_obstacles()
            .into_iter()
            .map(|point| (point.x, point.y))
            .collect();
        #[allow(deprecated)]
        let (legacy_u, legacy_trajectory) = dwa_control(
            DWAState::new(0.0, 0.0, std::f64::consts::FRAC_PI_8, 0.0, 0.0),
            &legacy_config,
            (10.0, 10.0),
            &obstacle_tuples,
        );
        assert_close(legacy_u.0, control[0]);
        assert_close(legacy_u.1, control[1]);
        assert_eq!(legacy_trajectory.len(), trajectory.len());
        assert_state_close(
            legacy_trajectory.last().unwrap(),
            &[
                0.052_615_098_653_616_07,
                0.028_621_196_634_198_105,
                0.596_902_604_182_061_3,
                0.02,
                0.068_067_840_827_779_04,
            ],
        );
    }

    #[test]
    fn test_dwa_stuck_initial_control_matches_pythonrobotics() {
        let mut dwa = DWAPlanner::new(DWAConfig {
            to_goal_cost_gain: 0.2,
            obstacle_cost_gain: 2.0,
            ..DWAConfig::default()
        });
        dwa.set_state(DWAState::new(
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_8,
            0.0,
            0.0,
        ));
        dwa.set_goal(Point2D::new(-5.0, -7.0));
        dwa.set_obstacles(pythonrobotics_stuck_obstacles());

        let control = dwa.try_plan_step().unwrap();
        assert!(control[0].abs() < 1.0e-15);
        assert_close(control[1], -0.698_131_700_797_731_8);

        let trajectory = &dwa.get_best_trajectory().states;
        assert_eq!(trajectory.len(), 31);
        assert_state_close(
            &trajectory[1],
            &[
                -3.214_541_934_276_051_4e-19,
                -1.305_290_122_999_866_4e-19,
                0.385_717_764_690_746_8,
                -3.469_446_951_953_614e-18,
                -0.069_813_170_079_773_18,
            ],
        );
        assert_state_close(
            &trajectory[30],
            &[
                -9.971_752_160_697_17e-18,
                -2.915_944_394_677_270_7e-18,
                0.183_259_571_459_404_3,
                -3.469_446_951_953_614e-18,
                -0.069_813_170_079_773_18,
            ],
        );

        #[allow(deprecated)]
        let legacy_config = Config {
            max_speed: 1.0,
            min_speed: -0.5,
            max_yaw_rate: 40.0_f64.to_radians(),
            max_accel: 0.2,
            max_delta_yaw_rate: 40.0_f64.to_radians(),
            v_resolution: 0.01,
            yaw_rate_resolution: 0.1_f64.to_radians(),
            dt: 0.1,
            predict_time: 3.0,
            to_goal_cost_gain: 0.2,
            speed_cost_gain: 1.0,
            obstacle_cost_gain: 2.0,
            robot_type: DWARobotType::Circle,
            robot_radius: 1.0,
            robot_width: 0.5,
            robot_length: 1.2,
        };
        let obstacle_tuples: Vec<_> = pythonrobotics_stuck_obstacles()
            .into_iter()
            .map(|point| (point.x, point.y))
            .collect();
        #[allow(deprecated)]
        let (legacy_u, legacy_trajectory) = dwa_control(
            DWAState::new(0.0, 0.0, std::f64::consts::FRAC_PI_8, 0.0, 0.0),
            &legacy_config,
            (-5.0, -7.0),
            &obstacle_tuples,
        );
        assert!(legacy_u.0.abs() < 1.0e-15);
        assert_close(legacy_u.1, control[1]);
        assert_eq!(legacy_trajectory.len(), trajectory.len());
        assert_state_close(
            legacy_trajectory.last().unwrap(),
            &[
                -9.971_752_160_697_17e-18,
                -2.915_944_394_677_270_7e-18,
                0.183_259_571_459_404_3,
                -3.469_446_951_953_614e-18,
                -0.069_813_170_079_773_18,
            ],
        );
    }

    #[test]
    fn test_dwa_rectangle_main_initial_parity_matches_pythonrobotics() {
        let mut dwa = DWAPlanner::new(DWAConfig {
            robot_type: DWARobotType::Rectangle,
            ..DWAConfig::default()
        });
        dwa.set_state(DWAState::new(
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_8,
            0.0,
            0.0,
        ));
        dwa.set_goal(Point2D::new(1.0, 1.0));
        dwa.set_obstacles(pythonrobotics_default_obstacles());

        let control = dwa.try_plan_step().unwrap();
        assert_close(control[0], 0.02);
        assert_close(control[1], 0.068_067_840_827_779_04);

        let trajectory = &dwa.get_best_trajectory().states;
        assert_eq!(trajectory.len(), 31);
        assert_state_close(
            &trajectory[1],
            &[
                0.001_842_506_612_952_407,
                0.000_777_926_334_061_682_8,
                0.399_505_865_781_502_05,
                0.02,
                0.068_067_840_827_779_04,
            ],
        );
        assert_state_close(
            &trajectory[30],
            &[
                0.052_615_098_653_616_07,
                0.028_621_196_634_198_105,
                0.596_902_604_182_061_3,
                0.02,
                0.068_067_840_827_779_04,
            ],
        );

        #[allow(deprecated)]
        let legacy_config = Config {
            max_speed: 1.0,
            min_speed: -0.5,
            max_yaw_rate: 40.0_f64.to_radians(),
            max_accel: 0.2,
            max_delta_yaw_rate: 40.0_f64.to_radians(),
            v_resolution: 0.01,
            yaw_rate_resolution: 0.1_f64.to_radians(),
            dt: 0.1,
            predict_time: 3.0,
            to_goal_cost_gain: 0.15,
            speed_cost_gain: 1.0,
            obstacle_cost_gain: 1.0,
            robot_type: DWARobotType::Rectangle,
            robot_radius: 1.0,
            robot_width: 0.5,
            robot_length: 1.2,
        };
        let obstacle_tuples: Vec<_> = pythonrobotics_default_obstacles()
            .into_iter()
            .map(|point| (point.x, point.y))
            .collect();
        #[allow(deprecated)]
        let (legacy_u, legacy_trajectory) = dwa_control(
            DWAState::new(0.0, 0.0, std::f64::consts::FRAC_PI_8, 0.0, 0.0),
            &legacy_config,
            (1.0, 1.0),
            &obstacle_tuples,
        );
        assert_close(legacy_u.0, control[0]);
        assert_close(legacy_u.1, control[1]);
        assert_eq!(legacy_trajectory.len(), trajectory.len());
        assert_state_close(
            legacy_trajectory.last().unwrap(),
            &[
                0.052_615_098_653_616_07,
                0.028_621_196_634_198_105,
                0.596_902_604_182_061_3,
                0.02,
                0.068_067_840_827_779_04,
            ],
        );
    }

    #[test]
    fn test_dwa_default_closed_loop_matches_pythonrobotics_main() {
        let mut dwa = DWAPlanner::with_defaults();
        dwa.set_state(DWAState::new(
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_8,
            0.0,
            0.0,
        ));
        dwa.set_goal(Point2D::new(10.0, 10.0));
        dwa.set_obstacles(pythonrobotics_default_obstacles());

        let trace = run_closed_loop_trace(dwa, 1_000).unwrap();
        assert_eq!(trace.len(), 221);
        let expected = [
            (
                0,
                ClosedLoopSample {
                    control: [0.02, 0.068_067_840_827_779_04],
                    state: [
                        0.001_842_506_612_952_407,
                        0.000_777_926_334_061_682_8,
                        0.399_505_865_781_502_05,
                        0.02,
                        0.068_067_840_827_779_04,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        0.052_615_098_653_616_07,
                        0.028_621_196_634_198_105,
                        0.596_902_604_182_061_3,
                        0.02,
                        0.068_067_840_827_779_04,
                    ],
                    dist: 14.140_282_717_861_751,
                },
            ),
            (
                1,
                ClosedLoopSample {
                    control: [0.04, 0.129_154_364_647_580_6],
                    state: [
                        0.005_507_118_539_734_564,
                        0.002_381_241_470_298_463_5,
                        0.412_421_302_246_260_1,
                        0.04,
                        0.129_154_364_647_580_6,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        0.100_285_621_194_050_45,
                        0.068_082_434_446_487_08,
                        0.786_968_959_724_242_9,
                        0.04,
                        0.129_154_364_647_580_6,
                    ],
                    dist: 14.136_557_883_673_978,
                },
            ),
            (
                2,
                ClosedLoopSample {
                    control: [0.06, 0.125_663_706_143_592],
                    state: [
                        0.010_973_381_433_457_923,
                        0.004_855_098_777_562_393,
                        0.424_987_672_860_619_34,
                        0.06,
                        0.125_663_706_143_592,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        0.152_459_040_871_135_35,
                        0.104_477_455_550_390_23,
                        0.789_412_420_677_035_5,
                        0.06,
                        0.125_663_706_143_592,
                    ],
                    dist: 14.130_943_860_296_941,
                },
            ),
            (
                10,
                ClosedLoopSample {
                    control: [0.209_999_999_999_999_96, 0.097_738_438_111_683_48],
                    state: [
                        0.113_698_261_449_284_92,
                        0.058_672_484_225_696_07,
                        0.511_905_069_609_937_5,
                        0.209_999_999_999_999_96,
                        0.097_738_438_111_683_48,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        0.593_749_519_151_196_6,
                        0.430_099_924_235_568_86,
                        0.795_346_540_133_818_3,
                        0.209_999_999_999_999_96,
                        0.097_738_438_111_683_48,
                    ],
                    dist: 14.020_305_090_887_366,
                },
            ),
            (
                50,
                ClosedLoopSample {
                    control: [0.990_000_000_000_000_7, 0.054_105_206_811_826_91],
                    state: [
                        1.950_167_034_188_733_6,
                        1.694_578_822_612_822_2,
                        0.921_882_910_903_411_1,
                        0.990_000_000_000_000_7,
                        0.054_105_206_811_826_91,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        3.492_440_509_739_950_5,
                        4.112_668_327_825_561,
                        1.078_788_010_657_709_2,
                        0.990_000_000_000_000_7,
                        0.054_105_206_811_826_91,
                    ],
                    dist: 11.566_323_171_658_883,
                },
            ),
            (
                100,
                ClosedLoopSample {
                    control: [0.990_000_000_000_000_7, 0.078_539_816_339_751_13],
                    state: [
                        3.530_364_817_697_800_6,
                        6.300_822_976_885_286,
                        1.472_883_355_758_044_3,
                        0.990_000_000_000_000_7,
                        0.078_539_816_339_751_13,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        3.473_368_247_999_231,
                        9.165_061_521_908_662,
                        1.700_648_823_143_322_3,
                        0.990_000_000_000_000_7,
                        0.078_539_816_339_751_13,
                    ],
                    dist: 7.452_522_394_493_172,
                },
            ),
            (
                150,
                ClosedLoopSample {
                    control: [0.990_000_000_000_000_7, -0.064_577_182_323_779_74],
                    state: [
                        5.167_079_306_155_943,
                        10.598_045_278_751_194,
                        0.561_297_887_441_443_9,
                        0.990_000_000_000_000_7,
                        -0.064_577_182_323_779_74,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        7.730_226_783_306_823,
                        11.882_133_506_527_275,
                        0.374_024_058_702_483_34,
                        0.990_000_000_000_000_7,
                        -0.064_577_182_323_779_74,
                    ],
                    dist: 4.869_782_396_413_901_5,
                },
            ),
            (
                200,
                ClosedLoopSample {
                    control: [0.990_000_000_000_000_7, -0.383_972_435_438_737_94],
                    state: [
                        9.503_213_445_849_001,
                        12.241_991_336_274_216,
                        -0.511_730_536_684_608_7,
                        0.990_000_000_000_000_7,
                        -0.383_972_435_438_737_94,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        10.769_122_970_663_686,
                        9.828_742_617_084_059,
                        -1.625_250_599_456_947_5,
                        0.990_000_000_000_000_7,
                        -0.383_972_435_438_737_94,
                    ],
                    dist: 2.296_371_492_662_689_5,
                },
            ),
            (
                220,
                ClosedLoopSample {
                    control: [0.990_000_000_000_000_7, -0.452_040_276_266_514_66],
                    state: [
                        10.682_812_904_798_725,
                        10.723_560_741_508_503,
                        -1.339_365_667_980_288_6,
                        0.990_000_000_000_000_7,
                        -0.452_040_276_266_514_66,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        9.529_554_489_209_27,
                        8.315_446_240_490_422,
                        -2.650_282_469_153_183,
                        0.990_000_000_000_000_7,
                        -0.452_040_276_266_514_66,
                    ],
                    dist: 0.994_873_665_151_514_7,
                },
            ),
        ];
        for (index, expected) in expected {
            assert_sample_close(&trace[index], &expected);
        }
    }

    #[test]
    fn test_dwa_stuck_closed_loop_matches_pythonrobotics_reference() {
        let mut dwa = DWAPlanner::new(DWAConfig {
            to_goal_cost_gain: 0.2,
            obstacle_cost_gain: 2.0,
            ..DWAConfig::default()
        });
        dwa.set_state(DWAState::new(
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_8,
            0.0,
            0.0,
        ));
        dwa.set_goal(Point2D::new(-5.0, -7.0));
        dwa.set_obstacles(pythonrobotics_stuck_obstacles());

        let trace = run_closed_loop_trace(dwa, 1_000).unwrap();
        assert_eq!(trace.len(), 461);
        let expected = [
            (
                0,
                ClosedLoopSample {
                    control: [-3.469_446_951_953_614e-18, -0.698_131_700_797_731_8],
                    state: [
                        -3.290_158_615_020_658_6e-19,
                        -1.100_871_673_005_335_8e-19,
                        0.322_885_911_618_950_97,
                        -3.469_446_951_953_614e-18,
                        -0.698_131_700_797_731_8,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        -9.971_752_160_697_17e-18,
                        -2.915_944_394_677_270_7e-18,
                        0.183_259_571_459_404_3,
                        -3.469_446_951_953_614e-18,
                        -0.069_813_170_079_773_18,
                    ],
                    dist: 8.602_325_267_042_627,
                },
            ),
            (
                1,
                ClosedLoopSample {
                    control: [0.019_999_999_999_999_993, -0.698_131_700_797_731_8],
                    state: [
                        0.001_936_295_280_756_214_5,
                        0.000_500_760_008_108_882_6,
                        0.253_072_741_539_177_8,
                        0.019_999_999_999_999_993,
                        -0.698_131_700_797_731_8,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        0.036_000_096_958_749_65,
                        -0.034_162_816_412_153_525,
                        -1.771_509_190_774_245_5,
                        0.019_999_999_999_999_993,
                        -0.698_131_700_797_731_8,
                    ],
                    dist: 8.603_858_296_887_57,
                },
            ),
            (
                2,
                ClosedLoopSample {
                    control: [0.039_999_999_999_999_994, -0.698_131_700_797_731_8],
                    state: [
                        0.005_869_314_911_012_033,
                        0.001_229_702_110_077_472_6,
                        0.183_259_571_459_404_64,
                        0.039_999_999_999_999_994,
                        -0.698_131_700_797_731_8,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        0.068_994_945_132_430_06,
                        -0.072_680_914_645_250_44,
                        -1.841_322_360_854_018_8,
                        0.039_999_999_999_999_994,
                        -0.698_131_700_797_731_8,
                    ],
                    dist: 8.606_738_345_022_23,
                },
            ),
            (
                10,
                ClosedLoopSample {
                    control: [0.189_999_999_999_999_95, -0.647_517_152_489_896_2],
                    state: [
                        0.104_132_970_400_981_02,
                        -0.016_809_261_291_929_67,
                        -0.366_344_609_993_609_74,
                        0.189_999_999_999_999_95,
                        -0.647_517_152_489_896_2,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        0.213_572_392_076_469_75,
                        -0.477_614_640_782_877_6,
                        -2.244_144_352_214_308_3,
                        0.189_999_999_999_999_95,
                        -0.647_517_152_489_896_2,
                    ],
                    dist: 8.649_689_374_348_22,
                },
            ),
            (
                50,
                ClosedLoopSample {
                    control: [0.029_999_999_999_999_79, -0.167_551_608_191_450_54],
                    state: [
                        0.343_298_669_611_588_45,
                        -0.402_879_787_203_639_7,
                        -1.769_938_394_447_439,
                        0.029_999_999_999_999_79,
                        -0.167_551_608_191_450_54,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        0.305_790_986_927_416_24,
                        -0.480_433_609_151_910_36,
                        -2.255_838_058_202_643,
                        0.029_999_999_999_999_79,
                        -0.167_551_608_191_450_54,
                    ],
                    dist: 8.489_572_178_547,
                },
            ),
            (
                100,
                ClosedLoopSample {
                    control: [0.269_999_999_999_999_7, -0.005_235_987_755_980_706],
                    state: [
                        0.106_092_820_485_199_51,
                        -0.538_702_831_822_151_7,
                        -2.837_905_363_742_752,
                        0.269_999_999_999_999_7,
                        -0.005_235_987_755_980_706,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        -0.642_886_107_988_420_7,
                        -0.766_974_159_554_404_4,
                        -2.853_089_728_235_101_8,
                        0.269_999_999_999_999_7,
                        -0.005_235_987_755_980_706,
                    ],
                    dist: 8.235_323_004_406_906,
                },
            ),
            (
                200,
                ClosedLoopSample {
                    control: [0.990_000_000_000_000_3, 0.308_923_277_603_009_64],
                    state: [
                        1.852_193_706_436_120_7,
                        -4.220_215_843_560_918,
                        0.757_647_428_290_844_2,
                        0.990_000_000_000_000_3,
                        0.308_923_277_603_009_64,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        2.803_482_546_405_410_5,
                        -1.612_237_727_831_649_8,
                        1.653_524_933_339_573_6,
                        0.990_000_000_000_000_3,
                        0.308_923_277_603_009_64,
                    ],
                    dist: 7.394_576_292_588_543,
                },
            ),
            (
                300,
                ClosedLoopSample {
                    control: [0.990_000_000_000_000_3, 0.371_755_130_674_814_8],
                    state: [
                        0.769_215_774_844_203_3,
                        3.386_587_574_598_417_3,
                        3.243_519_881_906_541,
                        0.990_000_000_000_000_3,
                        0.371_755_130_674_814_8,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        -1.391_460_065_500_792_3,
                        1.711_199_521_041_678_8,
                        4.321_609_760_863_501,
                        0.990_000_000_000_000_3,
                        0.371_755_130_674_814_8,
                    ],
                    dist: 11.881_290_001_574_468,
                },
            ),
            (
                400,
                ClosedLoopSample {
                    control: [0.299_999_999_999_999_7, 0.118_682_389_135_646_52],
                    state: [
                        -4.821_582_173_842_302,
                        -2.257_266_302_545_886_6,
                        4.841_543_345_032_829,
                        0.299_999_999_999_999_7,
                        0.118_682_389_135_646_52,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        -4.559_815_331_068_457,
                        -3.082_459_945_503_760_2,
                        5.185_722_273_526_192,
                        0.299_999_999_999_999_7,
                        0.118_682_389_135_646_52,
                    ],
                    dist: 4.746_088_478_490_219,
                },
            ),
            (
                460,
                ClosedLoopSample {
                    control: [0.990_000_000_000_000_3, -0.506_145_483_078_318_9],
                    state: [
                        -4.985_352_164_947_927,
                        -6.059_407_170_506_825,
                        4.171_685_978_117_614,
                        0.990_000_000_000_000_3,
                        -0.506_145_483_078_318_9,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        -7.510_205_006_513_761,
                        -6.760_560_048_330_578_6,
                        2.703_864_077_190_488,
                        0.990_000_000_000_000_3,
                        -0.506_145_483_078_318_9,
                    ],
                    dist: 0.940_706_877_813_535_5,
                },
            ),
        ];
        for (index, expected) in expected {
            assert_sample_close(&trace[index], &expected);
        }
    }

    #[test]
    fn test_dwa_rectangle_closed_loop_matches_pythonrobotics_main2() {
        let mut dwa = DWAPlanner::new(DWAConfig {
            robot_type: DWARobotType::Rectangle,
            ..DWAConfig::default()
        });
        dwa.set_state(DWAState::new(
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_8,
            0.0,
            0.0,
        ));
        dwa.set_goal(Point2D::new(1.0, 1.0));
        dwa.set_obstacles(pythonrobotics_default_obstacles());

        let trace = run_closed_loop_trace(dwa, 1_000).unwrap();
        assert!((21..=22).contains(&trace.len()));
        assert!(trace.last().unwrap().dist <= 1.05);
        let expected = [
            (
                0,
                ClosedLoopSample {
                    control: [0.02, 0.068_067_840_827_779_04],
                    state: [
                        0.001_842_506_612_952_407,
                        0.000_777_926_334_061_682_8,
                        0.399_505_865_781_502_05,
                        0.02,
                        0.068_067_840_827_779_04,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        0.052_615_098_653_616_07,
                        0.028_621_196_634_198_105,
                        0.596_902_604_182_061_3,
                        0.02,
                        0.068_067_840_827_779_04,
                    ],
                    dist: 1.412_360_837_075_983_3,
                },
            ),
            (
                1,
                ClosedLoopSample {
                    control: [0.04, 0.134_390_352_403_563_56],
                    state: [
                        0.005_506_278_543_593_619,
                        0.002_383_160_036_749_725_7,
                        0.412_944_901_021_858_4,
                        0.04,
                        0.134_390_352_403_563_56,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        0.099_685_436_565_874_95,
                        0.068_843_867_784_433_95,
                        0.802_676_922_992_193_3,
                        0.04,
                        0.134_390_352_403_563_56,
                    ],
                    dist: 1.408_636_617_937_526_7,
                },
            ),
            (
                2,
                ClosedLoopSample {
                    control: [0.06, 0.132_645_023_151_569_34],
                    state: [
                        0.010_969_514_971_661_525,
                        0.004_863_693_796_050_7,
                        0.426_209_403_337_015_3,
                        0.06,
                        0.132_645_023_151_569_34,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        0.151_191_833_116_684_02,
                        0.106_069_645_038_751_56,
                        0.810_879_970_476_567,
                        0.06,
                        0.132_645_023_151_569_34,
                    ],
                    dist: 1.403_024_436_081_079_3,
                },
            ),
            (
                5,
                ClosedLoopSample {
                    control: [0.119_999_999_999_999_95, 0.129_154_364_647_581],
                    state: [
                        0.037_928_184_668_002_49,
                        0.018_021_637_577_041_828,
                        0.465_304_778_581_688_45,
                        0.119_999_999_999_999_95,
                        0.129_154_364_647_581,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        0.311_447_525_499_293_9,
                        0.229_879_307_030_410_07,
                        0.839_852_436_059_674,
                        0.119_999_999_999_999_95,
                        0.129_154_364_647_581,
                    ],
                    dist: 1.374_723_129_260_244_9,
                },
            ),
            (
                10,
                ClosedLoopSample {
                    control: [0.209_999_999_999_999_96, 0.127_409_035_395_587_23],
                    state: [
                        0.113_124_079_420_109_7,
                        0.059_725_368_755_827_526,
                        0.529_183_829_204_681_3,
                        0.209_999_999_999_999_96,
                        0.127_409_035_395_587_23,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        0.568_258_124_617_134_2,
                        0.459_148_358_042_203_83,
                        0.898_670_031_851_883_7,
                        0.209_999_999_999_999_96,
                        0.127_409_035_395_587_23,
                    ],
                    dist: 1.292_542_177_519_090_6,
                },
            ),
            (
                20,
                ClosedLoopSample {
                    control: [0.340_000_000_000_000_1, 0.139_626_340_159_548_67],
                    state: [
                        0.365_710_870_049_402_1,
                        0.235_568_742_831_355_18,
                        0.666_192_175_486_237_9,
                        0.340_000_000_000_000_1,
                        0.139_626_340_159_548_67,
                    ],
                    predicted_len: 31,
                    predicted_last: [
                        0.992_955_085_115_716,
                        0.987_609_593_409_779_3,
                        1.071_108_561_948_929_3,
                        0.340_000_000_000_000_1,
                        0.139_626_340_159_548_67,
                    ],
                    dist: 0.993_316_589_668_128_5,
                },
            ),
        ];
        for (index, expected) in expected {
            assert_sample_close(&trace[index], &expected);
        }
    }
}
