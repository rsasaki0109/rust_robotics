//! Dynamic Window Approach (DWA) local planner
//!
//! Implements velocity-space local planning for mobile robots.
//! DWA searches the velocity space for admissible velocities and selects
//! the best trajectory based on goal direction, speed, and obstacle clearance.

use nalgebra::{Vector2, Vector4, Vector5};

use rust_robotics_core::{
    ControlInput, Obstacles, Path2D, Point2D, RoboticsError, RoboticsResult, State2D,
};

/// Robot state for DWA: [x, y, yaw, v, omega]
pub type DWAState = Vector5<f64>;

/// Control input: [v, omega]
pub type DWAControl = Vector2<f64>;

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
    pub robot_radius: f64,
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
            robot_radius: 1.0,
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
        next[0] += control[0] * state[2].cos() * dt;
        next[1] += control[0] * state[2].sin() * dt;
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
                if dist <= self.config.robot_radius {
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
        let mut best_trajectory = Trajectory::new();
        let mut min_cost = f64::MAX;

        let mut v = dw[0];
        while v <= dw[1] {
            let mut omega = dw[2];
            while omega <= dw[3] {
                let trajectory = self.predict_trajectory(v, omega);
                let goal_cost = config.to_goal_cost_gain * self.calc_to_goal_cost(&trajectory);
                let speed_cost = config.speed_cost_gain * self.calc_speed_cost(&trajectory);
                let obstacle_cost =
                    config.obstacle_cost_gain * self.calc_obstacle_cost(&trajectory);
                let total_cost = goal_cost + speed_cost + obstacle_cost;
                if total_cost < min_cost {
                    min_cost = total_cost;
                    best_trajectory = Trajectory {
                        states: trajectory.states,
                        control: DWAControl::new(v, omega),
                        cost: total_cost,
                    };
                }
                omega += config.yaw_rate_resolution;
            }
            v += config.v_resolution;
        }

        if best_trajectory.states.is_empty() {
            return Err(RoboticsError::PlanningError(
                "DWA failed to find an admissible trajectory".to_string(),
            ));
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
    pub robot_radius: f64,
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
    for state in trajectory.iter().take(trajectory.len().saturating_sub(1)) {
        for obstacle in ob.iter().take(ob.len().saturating_sub(1)) {
            let r = ((state[0] - obstacle.0).powi(2) + (state[1] - obstacle.1).powi(2)).sqrt();
            if r <= config.robot_radius {
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
    let mut v = dw[0];
    while v <= dw[1] {
        let mut y = dw[2];
        while y <= dw[3] {
            let trajectory = predict_trajectory(x, v, y, config);
            let to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(&trajectory, goal);
            let speed_cost =
                config.speed_cost_gain * (config.max_speed - trajectory.last().unwrap()[3]);
            let ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(&trajectory, ob, config);
            let final_cost = to_goal_cost + speed_cost + ob_cost;
            if final_cost < min_cost {
                min_cost = final_cost;
                best_u = (v, y);
                best_trajectory = trajectory;
            }
            y += config.yaw_rate_resolution;
        }
        v += config.v_resolution;
    }
    (best_u, best_trajectory)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dwa_creation() {
        let dwa = DWAPlanner::with_defaults();
        assert_eq!(dwa.state, DWAState::zeros());
    }

    #[test]
    fn test_dwa_config_default() {
        let config = DWAConfig::default();
        assert_eq!(config.max_speed, 1.0);
        assert_eq!(config.robot_radius, 1.0);
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
}
