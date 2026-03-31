//! State Lattice Planner
//!
//! Implements state lattice planning for path planning.
//! Uses a model predictive trajectory generator to create smooth paths.
//!
//! Based on:
//! - PythonRobotics State Lattice Planner by Atsushi Sakai
//! - "State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation"

use rust_robotics_core::{Obstacles, Path2D, Point2D, RoboticsError};

use super::motion_model::{normalize_angle, MotionModel, MotionModelConfig};
use super::trajectory_generator::{
    LookupTable, TargetState, TrajectoryGenerator, TrajectoryGeneratorConfig, TrajectoryParams,
};
use nalgebra::Vector3;

/// Configuration for State Lattice Planner
#[derive(Debug, Clone)]
pub struct StateLatticeConfig {
    /// Motion model configuration
    pub motion_config: MotionModelConfig,
    /// Trajectory generator configuration
    pub trajectory_config: TrajectoryGeneratorConfig,

    // Uniform polar sampling parameters
    /// Number of xy samples for polar sampling
    pub nxy: usize,
    /// Number of heading samples
    pub nh: usize,
    /// Distance for sampling \[m\]
    pub d: f64,
    /// Minimum angle for sampling \[rad\]
    pub a_min: f64,
    /// Maximum angle for sampling \[rad\]
    pub a_max: f64,
    /// Minimum heading offset angle [rad]
    pub p_min: f64,
    /// Maximum heading offset angle [rad]
    pub p_max: f64,

    // Lane sampling parameters
    /// Lane center offset
    pub lane_center: f64,
    /// Lane heading
    pub lane_heading: f64,
    /// Lane width \[m\]
    pub lane_width: f64,
    /// Vehicle width \[m\]
    pub vehicle_width: f64,

    // Biased sampling parameters
    /// Number of samples for biased sampling
    pub ns: usize,
}

impl Default for StateLatticeConfig {
    fn default() -> Self {
        Self {
            motion_config: MotionModelConfig::default(),
            trajectory_config: TrajectoryGeneratorConfig::default(),
            nxy: 5,
            nh: 3,
            d: 20.0,
            a_min: -45.0_f64.to_radians(),
            a_max: 45.0_f64.to_radians(),
            p_min: -45.0_f64.to_radians(),
            p_max: 45.0_f64.to_radians(),
            lane_center: 0.0,
            lane_heading: 0.0,
            lane_width: 3.0,
            vehicle_width: 1.0,
            ns: 10,
        }
    }
}

/// Target state for planning
#[derive(Debug, Clone, Copy)]
pub struct TargetPose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl TargetPose {
    pub fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    pub fn to_target_state(&self) -> TargetState {
        Vector3::new(self.x, self.y, self.yaw)
    }
}

/// Generated trajectory
#[derive(Debug, Clone)]
pub struct Trajectory {
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub yaw: Vec<f64>,
    pub params: TrajectoryParams,
}

impl Trajectory {
    pub fn to_path(&self) -> Path2D {
        let points: Vec<Point2D> = self
            .x
            .iter()
            .zip(self.y.iter())
            .map(|(&x, &y)| Point2D::new(x, y))
            .collect();
        Path2D::from_points(points)
    }

    pub fn len(&self) -> usize {
        self.x.len()
    }

    pub fn is_empty(&self) -> bool {
        self.x.is_empty()
    }
}

/// State Lattice Planner
pub struct StateLattice {
    config: StateLatticeConfig,
    trajectory_generator: TrajectoryGenerator,
    lookup_table: LookupTable,
}

impl StateLattice {
    pub fn new(config: StateLatticeConfig) -> Self {
        let motion_model = MotionModel::new(config.motion_config.clone());
        let trajectory_generator =
            TrajectoryGenerator::new(motion_model, config.trajectory_config.clone());
        let lookup_table = LookupTable::generate_default();

        Self {
            config,
            trajectory_generator,
            lookup_table,
        }
    }

    pub fn with_defaults() -> Self {
        Self::new(StateLatticeConfig::default())
    }

    /// Set lookup table from CSV data
    pub fn set_lookup_table_from_csv(&mut self, csv_data: &str) {
        self.lookup_table = LookupTable::from_csv(csv_data);
    }

    /// Set initial curvature
    pub fn set_initial_curvature(&mut self, k0: f64) {
        self.trajectory_generator.set_k0(k0);
    }

    /// Get configuration
    pub fn config(&self) -> &StateLatticeConfig {
        &self.config
    }

    // ========================================================================
    // State Sampling Methods
    // ========================================================================

    /// Calculate uniform polar states
    pub fn calc_uniform_polar_states(&self) -> Vec<TargetPose> {
        let config = &self.config;

        let angle_samples: Vec<f64> = (0..config.nxy)
            .map(|i| {
                if config.nxy > 1 {
                    i as f64 / (config.nxy - 1) as f64
                } else {
                    0.5
                }
            })
            .collect();

        self.sample_states(&angle_samples)
    }

    /// Calculate biased polar states toward a goal
    pub fn calc_biased_polar_states(&self, goal_angle: f64) -> Vec<TargetPose> {
        let config = &self.config;

        if config.nxy == 0 {
            return Vec::new();
        }

        if config.ns <= 1 || config.nxy == 1 {
            return self.sample_states(&[0.5]);
        }

        let asi: Vec<f64> = (0..config.ns - 1)
            .map(|i| {
                config.a_min + (config.a_max - config.a_min) * i as f64 / (config.ns - 1) as f64
            })
            .collect();

        let cnav: Vec<f64> = asi
            .iter()
            .map(|&angle| std::f64::consts::PI - (angle - goal_angle).abs())
            .collect();
        let cnav_sum: f64 = cnav.iter().sum();
        let cnav_max = cnav.iter().copied().fold(f64::NEG_INFINITY, f64::max);

        let normalized: Vec<f64> = cnav
            .iter()
            .map(|&value| (cnav_max - value) / (cnav_max * config.ns as f64 - cnav_sum))
            .collect();

        let mut cumulative = Vec::with_capacity(normalized.len());
        let mut running_sum = 0.0;
        for value in normalized {
            running_sum += value;
            cumulative.push(running_sum);
        }

        let mut angle_samples = Vec::with_capacity(config.nxy);
        let mut li = 0usize;
        for i in 0..config.nxy {
            let threshold = i as f64 / (config.nxy - 1) as f64;
            for (ii, &sample) in cumulative.iter().enumerate().take(config.ns - 1).skip(li) {
                if ii as f64 / config.ns as f64 >= threshold {
                    angle_samples.push(sample);
                    li = ii.saturating_sub(1);
                    break;
                }
            }
        }

        self.sample_states(&angle_samples)
    }

    /// Calculate lane states for structured driving
    pub fn calc_lane_states(&self) -> Vec<TargetPose> {
        let config = &self.config;

        let nxy = config.nxy;
        let d = config.d;
        let l_center = config.lane_center;
        let l_heading = config.lane_heading;
        let l_width = config.lane_width;
        let v_width = config.vehicle_width;

        let mut states = Vec::new();

        for i in 0..nxy {
            let delta = if nxy > 1 {
                -0.5 * (l_width - v_width) + (l_width - v_width) * i as f64 / (nxy - 1) as f64
            } else {
                0.0
            };
            let x = d - delta * l_heading.sin();
            let y = l_center + delta * l_heading.cos();
            states.push(TargetPose::new(x, y, l_heading));
        }

        states
    }

    /// Sample states from angle samples
    fn sample_states(&self, angle_samples: &[f64]) -> Vec<TargetPose> {
        let config = &self.config;
        let mut states = Vec::new();

        for &sample in angle_samples {
            let angle = config.a_min + (config.a_max - config.a_min) * sample;
            let x = config.d * angle.cos();
            let y = config.d * angle.sin();

            for j in 0..config.nh {
                let yaw = if config.nh == 1 {
                    (config.p_max - config.p_min) / 2.0 + angle
                } else {
                    config.p_min
                        + (config.p_max - config.p_min) * j as f64 / (config.nh - 1) as f64
                        + angle
                };

                states.push(TargetPose::new(x, y, normalize_angle(yaw)));
            }
        }

        states
    }

    // ========================================================================
    // Path Generation Methods
    // ========================================================================

    /// Generate paths to target states
    pub fn generate_paths(&self, targets: &[TargetPose]) -> Vec<Trajectory> {
        let mut paths = Vec::new();

        for target in targets {
            if let Some(trajectory) = self.generate_path_to_target(target) {
                paths.push(trajectory);
            }
        }

        paths
    }

    /// Generate a single path to a target
    fn generate_path_to_target(&self, target: &TargetPose) -> Option<Trajectory> {
        let target_state = target.to_target_state();
        let dist = target.x.hypot(target.y);

        let init_params = if let Some(nearest) = self.lookup_table.find_nearest(&target_state) {
            Vector3::new(dist, nearest.km, nearest.kf)
        } else {
            Vector3::new(dist.max(1.0), 0.0, 0.0)
        };

        let result = self
            .trajectory_generator
            .generate_optimized(&target_state, &init_params)?;

        Some(Trajectory {
            x: result.0,
            y: result.1,
            yaw: result.2,
            params: result.3,
        })
    }

    /// Generate paths using uniform polar sampling
    pub fn plan_uniform_polar(&self) -> Vec<Trajectory> {
        let targets = self.calc_uniform_polar_states();
        self.generate_paths(&targets)
    }

    /// Generate paths using biased polar sampling
    pub fn plan_biased_polar(&self, goal_angle: f64) -> Vec<Trajectory> {
        let targets = self.calc_biased_polar_states(goal_angle);
        self.generate_paths(&targets)
    }

    /// Generate paths using lane sampling
    pub fn plan_lane_states(&self) -> Vec<Trajectory> {
        let targets = self.calc_lane_states();
        self.generate_paths(&targets)
    }

    // ========================================================================
    // Full Planning Interface
    // ========================================================================

    /// Plan a path from start to goal using state lattice
    pub fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        let dx = goal.x - start.x;
        let dy = goal.y - start.y;
        let distance = (dx * dx + dy * dy).sqrt();
        let goal_angle = dy.atan2(dx);

        let paths = self.plan_biased_polar(goal_angle);

        if paths.is_empty() {
            return Err(RoboticsError::PlanningError(
                "No valid paths found".to_string(),
            ));
        }

        let best_path = paths
            .iter()
            .min_by(|a, b| {
                let a_final_x = a.x.last().unwrap_or(&0.0);
                let a_final_y = a.y.last().unwrap_or(&0.0);
                let b_final_x = b.x.last().unwrap_or(&0.0);
                let b_final_y = b.y.last().unwrap_or(&0.0);

                let scale = distance / self.config.d;
                let goal_x = dx;
                let goal_y = dy;

                let a_cost =
                    (a_final_x * scale - goal_x).powi(2) + (a_final_y * scale - goal_y).powi(2);
                let b_cost =
                    (b_final_x * scale - goal_x).powi(2) + (b_final_y * scale - goal_y).powi(2);

                a_cost
                    .partial_cmp(&b_cost)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap();

        let scale = distance / self.config.d;
        let cos_angle = goal_angle.cos();
        let sin_angle = goal_angle.sin();

        let points: Vec<Point2D> = best_path
            .x
            .iter()
            .zip(best_path.y.iter())
            .map(|(&lx, &ly)| {
                let sx = lx * scale;
                let sy = ly * scale;
                let wx = start.x + sx * cos_angle - sy * sin_angle;
                let wy = start.y + sx * sin_angle + sy * cos_angle;
                Point2D::new(wx, wy)
            })
            .collect();

        Ok(Path2D::from_points(points))
    }

    // ========================================================================
    // Obstacle-Aware Planning
    // ========================================================================

    /// Plan the best collision-free trajectory considering obstacles.
    ///
    /// Generates candidate trajectories using biased polar sampling toward the
    /// goal, checks each against obstacles, scores them, and returns the best
    /// collision-free trajectory.
    ///
    /// All coordinates are in the ego frame (vehicle at origin heading along +x).
    /// Transform obstacles into the ego frame before calling.
    pub fn plan_with_obstacles(
        &self,
        goal: Point2D,
        obstacles: &Obstacles,
        robot_radius: f64,
    ) -> Result<ObstacleAwarePlanResult, RoboticsError> {
        let goal_angle = goal.y.atan2(goal.x);

        let targets = self.calc_biased_polar_states(goal_angle);
        if targets.is_empty() {
            return Err(RoboticsError::PlanningError(
                "no target states sampled".to_string(),
            ));
        }

        let paths = self.generate_paths(&targets);
        if paths.is_empty() {
            return Err(RoboticsError::PlanningError(
                "no valid trajectories generated".to_string(),
            ));
        }

        let mut valid_indices: Vec<usize> = Vec::new();
        for (idx, traj) in paths.iter().enumerate() {
            if !check_trajectory_collision(traj, obstacles, robot_radius) {
                valid_indices.push(idx);
            }
        }

        if valid_indices.is_empty() {
            return Err(RoboticsError::PlanningError(
                "all trajectories collide with obstacles".to_string(),
            ));
        }

        let goal_vec = Vector3::new(goal.x, goal.y, goal_angle);
        let mut best_idx = valid_indices[0];
        let mut best_cost = f64::MAX;
        for &idx in &valid_indices {
            let cost = trajectory_cost(&paths[idx], &goal_vec, obstacles);
            if cost < best_cost {
                best_cost = cost;
                best_idx = idx;
            }
        }

        Ok(ObstacleAwarePlanResult {
            best: paths[best_idx].clone(),
            candidates: paths,
            valid_indices,
        })
    }

    /// Plan from a world-frame pose with obstacles, returning a world-frame result.
    ///
    /// Transforms obstacles into the ego frame, plans, then transforms the
    /// result back to world frame.
    pub fn plan_from_pose_with_obstacles(
        &self,
        pose_x: f64,
        pose_y: f64,
        pose_yaw: f64,
        goal: Point2D,
        obstacles: &Obstacles,
        robot_radius: f64,
    ) -> Result<ObstacleAwarePlanResult, RoboticsError> {
        let cos_yaw = pose_yaw.cos();
        let sin_yaw = pose_yaw.sin();

        // Transform goal to ego frame.
        let dx = goal.x - pose_x;
        let dy = goal.y - pose_y;
        let ego_goal = Point2D::new(cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy);

        // Transform obstacles to ego frame.
        let ego_obs = Obstacles::from_points(
            obstacles
                .points
                .iter()
                .map(|o| {
                    let odx = o.x - pose_x;
                    let ody = o.y - pose_y;
                    Point2D::new(
                        cos_yaw * odx + sin_yaw * ody,
                        -sin_yaw * odx + cos_yaw * ody,
                    )
                })
                .collect(),
        );

        let mut result = self.plan_with_obstacles(ego_goal, &ego_obs, robot_radius)?;

        // Transform results back to world frame.
        for traj in &mut result.candidates {
            transform_trajectory_to_world(traj, pose_x, pose_y, pose_yaw);
        }
        transform_trajectory_to_world(&mut result.best, pose_x, pose_y, pose_yaw);

        Ok(result)
    }
}

/// Result of obstacle-aware planning.
#[derive(Debug, Clone)]
pub struct ObstacleAwarePlanResult {
    /// The lowest-cost collision-free trajectory.
    pub best: Trajectory,
    /// All candidate trajectories (including those that collide).
    pub candidates: Vec<Trajectory>,
    /// Indices into `candidates` that are collision-free.
    pub valid_indices: Vec<usize>,
}

/// Check if any point on a trajectory is within `radius` of any obstacle.
fn check_trajectory_collision(traj: &Trajectory, obstacles: &Obstacles, radius: f64) -> bool {
    let r_sq = radius * radius;
    for (&tx, &ty) in traj.x.iter().zip(traj.y.iter()) {
        for obs in &obstacles.points {
            let dx = tx - obs.x;
            let dy = ty - obs.y;
            if dx * dx + dy * dy <= r_sq {
                return true;
            }
        }
    }
    false
}

/// Score a trajectory: lower is better.
///
/// Combines goal-deviation cost and obstacle-proximity cost.
fn trajectory_cost(traj: &Trajectory, goal: &Vector3<f64>, obstacles: &Obstacles) -> f64 {
    if traj.is_empty() {
        return f64::MAX;
    }

    let last_x = *traj.x.last().unwrap();
    let last_y = *traj.y.last().unwrap();
    let last_yaw = *traj.yaw.last().unwrap();

    // Goal deviation.
    let dx = last_x - goal[0];
    let dy = last_y - goal[1];
    let dyaw = normalize_angle(last_yaw - goal[2]);
    let goal_cost = (dx * dx + dy * dy).sqrt() + dyaw.abs();

    // Inverse minimum distance to nearest obstacle (clearance incentive).
    let mut min_dist_sq = f64::MAX;
    if !obstacles.is_empty() {
        for (&tx, &ty) in traj.x.iter().zip(traj.y.iter()) {
            for obs in &obstacles.points {
                let d2 = (tx - obs.x).powi(2) + (ty - obs.y).powi(2);
                if d2 < min_dist_sq {
                    min_dist_sq = d2;
                }
            }
        }
    }
    let obstacle_cost = if min_dist_sq < f64::MAX && min_dist_sq > 1e-12 {
        1.0 / min_dist_sq.sqrt()
    } else {
        0.0
    };

    // Lateral offset cost.
    let lateral_cost = last_y.abs();

    goal_cost + 5.0 * obstacle_cost + 0.5 * lateral_cost
}

/// Transform a trajectory from ego frame to world frame.
fn transform_trajectory_to_world(traj: &mut Trajectory, ox: f64, oy: f64, oyaw: f64) {
    let cos_yaw = oyaw.cos();
    let sin_yaw = oyaw.sin();
    for ((&mut ref mut x, &mut ref mut y), yaw) in traj
        .x
        .iter_mut()
        .zip(traj.y.iter_mut())
        .zip(traj.yaw.iter_mut())
    {
        let lx = *x;
        let ly = *y;
        *x = cos_yaw * lx - sin_yaw * ly + ox;
        *y = sin_yaw * lx + cos_yaw * ly + oy;
        *yaw = normalize_angle(*yaw + oyaw);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_8};

    type TrajectoryTerminal = (f64, f64, f64, f64, f64, f64);

    fn assert_pose_close(actual: &TargetPose, expected: (f64, f64, f64)) {
        assert!((actual.x - expected.0).abs() < 1e-9);
        assert!((actual.y - expected.1).abs() < 1e-9);
        assert!((actual.yaw - expected.2).abs() < 1e-9);
    }

    fn assert_trajectory_close_with_tolerance(
        actual: &Trajectory,
        expected: TrajectoryTerminal,
        tolerance: TrajectoryTerminal,
    ) {
        let observed = (
            *actual.x.last().unwrap(),
            *actual.y.last().unwrap(),
            *actual.yaw.last().unwrap(),
            actual.params[0],
            actual.params[1],
            actual.params[2],
        );
        assert!(
            (observed.0 - expected.0).abs() < tolerance.0,
            "x mismatch: observed={observed:?}, expected={expected:?}"
        );
        assert!(
            (observed.1 - expected.1).abs() < tolerance.1,
            "y mismatch: observed={observed:?}, expected={expected:?}"
        );
        assert!(
            (observed.2 - expected.2).abs() < tolerance.2,
            "yaw mismatch: observed={observed:?}, expected={expected:?}"
        );
        assert!(
            (observed.3 - expected.3).abs() < tolerance.3,
            "s mismatch: observed={observed:?}, expected={expected:?}"
        );
        assert!(
            (observed.4 - expected.4).abs() < tolerance.4,
            "km mismatch: observed={observed:?}, expected={expected:?}"
        );
        assert!(
            (observed.5 - expected.5).abs() < tolerance.5,
            "kf mismatch: observed={observed:?}, expected={expected:?}"
        );
    }

    fn parse_reference_trajectories(csv: &str) -> Vec<TrajectoryTerminal> {
        csv.lines()
            .skip(1)
            .filter(|line| !line.trim().is_empty())
            .map(|line| {
                let values: Vec<f64> = line
                    .split(',')
                    .map(|value| value.parse::<f64>().unwrap())
                    .collect();
                assert_eq!(values.len(), 6);
                (
                    values[0], values[1], values[2], values[3], values[4], values[5],
                )
            })
            .collect()
    }

    fn assert_trajectory_table_close(
        actual: &[Trajectory],
        expected: &[TrajectoryTerminal],
        tolerance: TrajectoryTerminal,
    ) {
        assert_eq!(actual.len(), expected.len());
        for (actual, expected) in actual.iter().zip(expected.iter().copied()) {
            assert_trajectory_close_with_tolerance(actual, expected, tolerance);
        }
    }

    #[test]
    fn test_state_lattice_creation() {
        let planner = StateLattice::with_defaults();
        assert!(planner.config.nxy > 0);
    }

    #[test]
    fn test_uniform_polar_states() {
        let planner = StateLattice::with_defaults();
        let states = planner.calc_uniform_polar_states();
        assert!(!states.is_empty());
    }

    #[test]
    fn test_uniform_polar_states_match_upstream_reference() {
        let planner = StateLattice::new(StateLatticeConfig {
            nxy: 5,
            nh: 3,
            d: 20.0,
            a_min: -45.0_f64.to_radians(),
            a_max: 45.0_f64.to_radians(),
            p_min: -45.0_f64.to_radians(),
            p_max: 45.0_f64.to_radians(),
            ..Default::default()
        });
        let states = planner.calc_uniform_polar_states();

        assert_eq!(states.len(), 15);
        assert_pose_close(
            &states[0],
            (14.142135623730951, -14.14213562373095, -FRAC_PI_2),
        );
        assert_pose_close(
            &states[5],
            (18.477590650225736, -7.653668647301796, FRAC_PI_8),
        );
        assert_pose_close(
            states.last().unwrap(),
            (14.142135623730951, 14.14213562373095, FRAC_PI_2),
        );
    }

    #[test]
    fn test_biased_polar_states() {
        let planner = StateLattice::with_defaults();
        let states = planner.calc_biased_polar_states(0.0);
        assert!(!states.is_empty());
    }

    #[test]
    fn test_biased_polar_states_match_upstream_reference_window() {
        let planner = StateLattice::new(StateLatticeConfig {
            nxy: 30,
            nh: 2,
            d: 20.0,
            a_min: -45.0_f64.to_radians(),
            a_max: 45.0_f64.to_radians(),
            p_min: -20.0_f64.to_radians(),
            p_max: 20.0_f64.to_radians(),
            ns: 100,
            ..Default::default()
        });
        let states = planner.calc_biased_polar_states(0.0);

        assert_eq!(states.len(), 58);
        assert_pose_close(
            &states[0],
            (14.554768777999886, -13.717095385647784, -1.1048434557770916),
        );
        assert_pose_close(
            &states[5],
            (
                16.88791878728749,
                -10.714392144867987,
                -0.216_293_881_998_737_4,
            ),
        );
        assert_pose_close(
            states.last().unwrap(),
            (16.077756451864964, 11.89561883529035, 0.9860589731081659),
        );
    }

    #[test]
    fn test_lane_states() {
        let planner = StateLattice::with_defaults();
        let states = planner.calc_lane_states();
        assert!(!states.is_empty());
    }

    #[test]
    fn test_lane_states_match_upstream_reference() {
        let planner = StateLattice::new(StateLatticeConfig {
            lane_center: 10.0,
            lane_heading: 0.0,
            lane_width: 3.0,
            vehicle_width: 1.0,
            d: 10.0,
            nxy: 5,
            ..Default::default()
        });
        let states = planner.calc_lane_states();

        assert_eq!(states.len(), 5);
        assert_pose_close(&states[0], (10.0, 9.0, 0.0));
        assert_pose_close(&states[2], (10.0, 10.0, 0.0));
        assert_pose_close(states.last().unwrap(), (10.0, 11.0, 0.0));
    }

    #[test]
    fn test_generate_paths() {
        let planner = StateLattice::with_defaults();
        let targets = vec![
            TargetPose::new(10.0, 0.0, 0.0),
            TargetPose::new(10.0, 2.0, 0.2),
        ];
        let paths = planner.generate_paths(&targets);
        assert!(!paths.is_empty() || targets.is_empty());
    }

    #[test]
    fn test_generate_lane_paths_match_upstream_full_example() {
        let planner = StateLattice::new(StateLatticeConfig {
            lane_center: 10.0,
            lane_heading: 0.0,
            lane_width: 3.0,
            vehicle_width: 1.0,
            d: 10.0,
            nxy: 5,
            ..Default::default()
        });
        let targets = planner.calc_lane_states();
        let paths = planner.generate_paths(&targets);
        let expected =
            parse_reference_trajectories(include_str!("testdata/lane_state_sampling_test1.csv"));

        assert_eq!(targets.len(), expected.len());
        assert_trajectory_table_close(&paths, &expected, (5e-3, 5e-3, 5e-4, 5e-3, 5e-4, 5e-4));
    }

    #[test]
    fn test_generate_uniform_paths_match_upstream_full_example() {
        let planner = StateLattice::new(StateLatticeConfig {
            nxy: 5,
            nh: 3,
            d: 20.0,
            a_min: -45.0_f64.to_radians(),
            a_max: 45.0_f64.to_radians(),
            p_min: -45.0_f64.to_radians(),
            p_max: 45.0_f64.to_radians(),
            ..Default::default()
        });
        let targets = planner.calc_uniform_polar_states();
        let paths = planner.generate_paths(&targets);
        let expected = parse_reference_trajectories(include_str!(
            "testdata/uniform_terminal_state_sampling_test1.csv"
        ));

        assert_eq!(targets.len(), expected.len());
        assert_trajectory_table_close(
            &paths,
            &expected,
            (1.5e-1, 1.5e-1, 5e-2, 2e-1, 1e-2, 1.5e-2),
        );
    }

    #[test]
    fn test_generate_uniform2_paths_match_upstream_full_example() {
        let mut planner = StateLattice::new(StateLatticeConfig {
            nxy: 6,
            nh: 3,
            d: 20.0,
            a_min: 10.0_f64.to_radians(),
            a_max: 45.0_f64.to_radians(),
            p_min: -20.0_f64.to_radians(),
            p_max: 20.0_f64.to_radians(),
            ..Default::default()
        });
        planner.set_initial_curvature(0.1);
        let targets = planner.calc_uniform_polar_states();
        let paths = planner.generate_paths(&targets);
        let expected = parse_reference_trajectories(include_str!(
            "testdata/uniform_terminal_state_sampling_test2.csv"
        ));

        assert_eq!(targets.len(), expected.len());
        assert_trajectory_table_close(&paths, &expected, (1e-1, 1e-1, 2e-2, 2e-1, 1e-2, 1e-2));
    }

    #[test]
    fn test_generate_biased_paths_match_upstream_full_example() {
        let planner = StateLattice::new(StateLatticeConfig {
            nxy: 30,
            nh: 2,
            d: 20.0,
            a_min: -45.0_f64.to_radians(),
            a_max: 45.0_f64.to_radians(),
            p_min: -20.0_f64.to_radians(),
            p_max: 20.0_f64.to_radians(),
            ns: 100,
            ..Default::default()
        });
        let targets = planner.calc_biased_polar_states(0.0);
        let paths = planner.generate_paths(&targets);
        let expected = parse_reference_trajectories(include_str!(
            "testdata/biased_terminal_state_sampling_test1.csv"
        ));

        assert_eq!(targets.len(), expected.len());
        assert_trajectory_table_close(&paths, &expected, (1.5e-1, 1.5e-1, 3e-2, 2e-1, 1e-2, 1e-2));
    }

    #[test]
    fn test_generate_biased2_paths_match_upstream_full_example() {
        let planner = StateLattice::new(StateLatticeConfig {
            nxy: 30,
            nh: 1,
            d: 20.0,
            a_min: 0.0,
            a_max: 45.0_f64.to_radians(),
            p_min: -20.0_f64.to_radians(),
            p_max: 20.0_f64.to_radians(),
            ns: 100,
            ..Default::default()
        });
        let targets = planner.calc_biased_polar_states(30.0_f64.to_radians());
        let paths = planner.generate_paths(&targets);
        let expected = parse_reference_trajectories(include_str!(
            "testdata/biased_terminal_state_sampling_test2.csv"
        ));

        assert_eq!(targets.len(), expected.len());
        assert_trajectory_table_close(&paths, &expected, (1e-1, 1e-1, 2e-2, 2e-1, 1e-2, 1e-2));
    }

    #[test]
    fn test_plan_uniform_polar() {
        let planner = StateLattice::with_defaults();
        let _paths = planner.plan_uniform_polar();
    }

    #[test]
    fn test_target_pose() {
        let target = TargetPose::new(10.0, 5.0, 0.5);
        let state = target.to_target_state();
        assert_eq!(state[0], 10.0);
        assert_eq!(state[1], 5.0);
        assert_eq!(state[2], 0.5);
    }

    #[test]
    fn test_trajectory_to_path() {
        let traj = Trajectory {
            x: vec![0.0, 1.0, 2.0],
            y: vec![0.0, 0.5, 1.0],
            yaw: vec![0.0, 0.1, 0.2],
            params: Vector3::new(2.0, 0.0, 0.0),
        };

        let path = traj.to_path();
        assert_eq!(path.len(), 3);
    }

    #[test]
    fn test_plan_straight() {
        let planner = StateLattice::with_defaults();
        let start = Point2D::new(0.0, 0.0);
        let goal = Point2D::new(20.0, 0.0);

        let result = planner.plan(start, goal);
        if let Ok(path) = result {
            assert!(!path.is_empty());
            assert!((path.points[0].x - start.x).abs() < 1.0);
        }
    }

    // ====================================================================
    // Obstacle-aware planning tests
    // ====================================================================

    #[test]
    fn test_plan_with_obstacles_no_obstacles() {
        let planner = StateLattice::with_defaults();
        let goal = Point2D::new(20.0, 0.0);
        let obstacles = Obstacles::new();

        let result = planner.plan_with_obstacles(goal, &obstacles, 1.0);
        assert!(result.is_ok(), "planning should succeed with no obstacles");
        let pr = result.unwrap();
        assert!(!pr.best.is_empty());
        assert!(!pr.valid_indices.is_empty());
        assert_eq!(pr.valid_indices.len(), pr.candidates.len());
    }

    #[test]
    fn test_plan_with_obstacles_avoids_blocked_path() {
        let planner = StateLattice::with_defaults();
        let goal = Point2D::new(20.0, 0.0);
        // Place obstacle on the straight-ahead path.
        let obstacles = Obstacles::from_points(vec![Point2D::new(10.0, 0.0)]);

        let result = planner.plan_with_obstacles(goal, &obstacles, 1.0);
        assert!(result.is_ok());
        let pr = result.unwrap();
        // The best trajectory should not go through (10, 0).
        let _last_y = pr.best.y.last().unwrap();
        // It should have been deflected laterally or picked a different angle.
        assert!(
            pr.valid_indices.len() < pr.candidates.len(),
            "some trajectories should have been filtered out"
        );
    }

    #[test]
    fn test_plan_with_obstacles_all_blocked() {
        let planner = StateLattice::new(StateLatticeConfig {
            nxy: 5,
            nh: 3,
            d: 20.0,
            ..Default::default()
        });
        // Dense wall of obstacles across all angles.
        let mut obs_pts = Vec::new();
        for i in -30..=30 {
            obs_pts.push(Point2D::new(10.0, i as f64 * 1.0));
        }
        let obstacles = Obstacles::from_points(obs_pts);

        let result = planner.plan_with_obstacles(Point2D::new(20.0, 0.0), &obstacles, 3.0);
        assert!(result.is_err(), "all trajectories should be blocked");
    }

    #[test]
    fn test_collision_check() {
        let traj = Trajectory {
            x: vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
            y: vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            yaw: vec![0.0; 6],
            params: Vector3::new(5.0, 0.0, 0.0),
        };

        // Obstacle on the path.
        let obs = Obstacles::from_points(vec![Point2D::new(2.5, 0.0)]);
        assert!(check_trajectory_collision(&traj, &obs, 0.6));

        // Obstacle far away.
        let obs_far = Obstacles::from_points(vec![Point2D::new(2.5, 10.0)]);
        assert!(!check_trajectory_collision(&traj, &obs_far, 0.6));
    }

    #[test]
    fn test_plan_from_pose_with_obstacles() {
        let planner = StateLattice::with_defaults();
        let obstacles = Obstacles::new();
        let goal = Point2D::new(30.0, 10.0);

        let result = planner.plan_from_pose_with_obstacles(10.0, 5.0, 0.3, goal, &obstacles, 1.0);
        assert!(result.is_ok());
        let pr = result.unwrap();
        let ep_x = pr.best.x.last().unwrap();
        let ep_y = pr.best.y.last().unwrap();
        // Trajectory should advance from the start pose toward the goal.
        assert!(*ep_x > 10.0 || *ep_y > 5.0, "ep = ({}, {})", ep_x, ep_y);
    }

    #[test]
    fn test_transform_trajectory_to_world() {
        let mut traj = Trajectory {
            x: vec![1.0, 2.0],
            y: vec![0.0, 0.0],
            yaw: vec![0.0, 0.0],
            params: Vector3::new(2.0, 0.0, 0.0),
        };

        let yaw = std::f64::consts::FRAC_PI_2;
        transform_trajectory_to_world(&mut traj, 5.0, 3.0, yaw);

        // After 90-degree rotation: ego (1,0) -> world (5 + 0, 3 + 1) = (5, 4)
        assert!((traj.x[0] - 5.0).abs() < 1e-9);
        assert!((traj.y[0] - 4.0).abs() < 1e-9);
    }
}
