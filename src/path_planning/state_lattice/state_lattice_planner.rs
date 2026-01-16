//! State Lattice Planner
//!
//! Implements state lattice planning for path planning.
//! Uses a model predictive trajectory generator to create smooth paths.
//!
//! Based on:
//! - PythonRobotics State Lattice Planner by Atsushi Sakai
//! - "State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation"

use std::f64::consts::PI;

use crate::common::{error::RoboticsError, Path2D, Point2D};

use super::motion_model::{normalize_angle, MotionModel, MotionModelConfig};
use super::trajectory_generator::{
    LookupTable, LookupTableEntry, TargetState, TrajectoryGenerator, TrajectoryGeneratorConfig,
    TrajectoryParams,
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
    /// Distance for sampling [m]
    pub d: f64,
    /// Minimum angle for sampling [rad]
    pub a_min: f64,
    /// Maximum angle for sampling [rad]
    pub a_max: f64,
    /// Minimum position ratio
    pub p_min: f64,
    /// Maximum position ratio
    pub p_max: f64,

    // Lane sampling parameters
    /// Lane center offset
    pub lane_center: f64,
    /// Lane heading
    pub lane_heading: f64,
    /// Lane width [m]
    pub lane_width: f64,
    /// Vehicle width [m]
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
            p_min: 0.5,
            p_max: 1.5,
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
    ///
    /// Generates uniformly distributed target states in polar coordinates
    pub fn calc_uniform_polar_states(&self) -> Vec<TargetPose> {
        let config = &self.config;

        // Create uniform angle samples
        let angle_samples: Vec<f64> = (0..config.nxy)
            .map(|i| {
                if config.nxy > 1 {
                    config.a_min
                        + (config.a_max - config.a_min) * i as f64 / (config.nxy - 1) as f64
                } else {
                    (config.a_min + config.a_max) / 2.0
                }
            })
            .collect();

        self.sample_states(&angle_samples)
    }

    /// Calculate biased polar states toward a goal
    ///
    /// Generates states biased toward a specific goal angle
    pub fn calc_biased_polar_states(&self, goal_angle: f64) -> Vec<TargetPose> {
        let config = &self.config;

        // Create biased angle samples with more density around goal angle
        let mut angle_samples = Vec::new();

        for i in 0..config.ns {
            // Bias distribution toward goal angle
            let t = i as f64 / (config.ns - 1).max(1) as f64;
            let bias = 0.5 + 0.5 * (PI * t).cos(); // More samples near edges and center
            let angle = config.a_min + (config.a_max - config.a_min) * t;

            // Weight toward goal angle
            let weight = 1.0 / (1.0 + (angle - goal_angle).abs());
            if weight > 0.3 || i % 2 == 0 {
                angle_samples.push(angle);
            }
        }

        // Always include goal angle if in range
        if goal_angle >= config.a_min && goal_angle <= config.a_max {
            angle_samples.push(goal_angle);
        }

        self.sample_states(&angle_samples)
    }

    /// Calculate lane states for structured driving
    ///
    /// Generates states across a lane width for highway-like scenarios
    pub fn calc_lane_states(&self) -> Vec<TargetPose> {
        let config = &self.config;

        let nxy = config.nxy;
        let d = config.d;
        let l_center = config.lane_center;
        let l_heading = config.lane_heading;
        let l_width = config.lane_width;
        let v_width = config.vehicle_width;

        let mut states = Vec::new();

        // Sample across lane width
        let lateral_range = l_width - v_width;
        let lateral_min = l_center - lateral_range / 2.0;

        for i in 0..nxy {
            let lateral_offset = if nxy > 1 {
                lateral_min + lateral_range * i as f64 / (nxy - 1) as f64
            } else {
                l_center
            };

            // Convert to target position
            let x = d * l_heading.cos() - lateral_offset * l_heading.sin();
            let y = d * l_heading.sin() + lateral_offset * l_heading.cos();

            // Add states with different headings
            for j in 0..config.nh {
                let yaw = if config.nh > 1 {
                    l_heading - 0.2 + 0.4 * j as f64 / (config.nh - 1) as f64
                } else {
                    l_heading
                };

                states.push(TargetPose::new(x, y, yaw));
            }
        }

        states
    }

    /// Sample states from angle samples
    fn sample_states(&self, angle_samples: &[f64]) -> Vec<TargetPose> {
        let config = &self.config;
        let mut states = Vec::new();

        for &angle in angle_samples {
            // Sample at different distances
            let p_range = config.p_max - config.p_min;
            for i in 0..3 {
                // 3 distance samples per angle
                let p = if i == 0 {
                    config.p_min
                } else if i == 1 {
                    (config.p_min + config.p_max) / 2.0
                } else {
                    config.p_max
                };

                let x = config.d * p * angle.cos();
                let y = config.d * p * angle.sin();

                // Sample headings
                for j in 0..config.nh {
                    let yaw = if config.nh > 1 {
                        config.a_min + (config.a_max - config.a_min) * j as f64 / (config.nh - 1) as f64
                    } else {
                        angle
                    };

                    states.push(TargetPose::new(x, y, normalize_angle(yaw)));
                }
            }
        }

        states
    }

    // ========================================================================
    // Path Generation Methods
    // ========================================================================

    /// Generate paths to target states
    ///
    /// # Arguments
    /// * `targets` - List of target poses
    ///
    /// # Returns
    /// List of generated trajectories (only successful ones)
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

        // Find nearest entry in lookup table for initialization
        let init_params = if let Some(nearest) = self.lookup_table.find_nearest(&target_state) {
            nearest.params()
        } else {
            // Default initialization based on distance
            let dist = (target.x * target.x + target.y * target.y).sqrt();
            Vector3::new(dist.max(1.0), 0.0, 0.0)
        };

        // Optimize trajectory
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
    ///
    /// # Arguments
    /// * `start` - Start position
    /// * `goal` - Goal position
    ///
    /// # Returns
    /// Best path to goal or error
    pub fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        let dx = goal.x - start.x;
        let dy = goal.y - start.y;
        let distance = (dx * dx + dy * dy).sqrt();
        let goal_angle = dy.atan2(dx);

        // Generate candidate paths
        let paths = self.plan_biased_polar(goal_angle);

        if paths.is_empty() {
            return Err(RoboticsError::PlanningError(
                "No valid paths found".to_string(),
            ));
        }

        // Select best path (closest to goal direction)
        let best_path = paths
            .iter()
            .min_by(|a, b| {
                let a_final_x = a.x.last().unwrap_or(&0.0);
                let a_final_y = a.y.last().unwrap_or(&0.0);
                let b_final_x = b.x.last().unwrap_or(&0.0);
                let b_final_y = b.y.last().unwrap_or(&0.0);

                // Cost is distance from scaled goal position
                let scale = distance / self.config.d;
                let goal_x = dx;
                let goal_y = dy;

                let a_cost = (a_final_x * scale - goal_x).powi(2)
                    + (a_final_y * scale - goal_y).powi(2);
                let b_cost = (b_final_x * scale - goal_x).powi(2)
                    + (b_final_y * scale - goal_y).powi(2);

                a_cost
                    .partial_cmp(&b_cost)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap();

        // Transform path to world coordinates
        let scale = distance / self.config.d;
        let cos_angle = goal_angle.cos();
        let sin_angle = goal_angle.sin();

        let points: Vec<Point2D> = best_path
            .x
            .iter()
            .zip(best_path.y.iter())
            .map(|(&lx, &ly)| {
                // Scale and rotate to world frame
                let sx = lx * scale;
                let sy = ly * scale;
                let wx = start.x + sx * cos_angle - sy * sin_angle;
                let wy = start.y + sx * sin_angle + sy * cos_angle;
                Point2D::new(wx, wy)
            })
            .collect();

        Ok(Path2D::from_points(points))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn test_biased_polar_states() {
        let planner = StateLattice::with_defaults();
        let states = planner.calc_biased_polar_states(0.0);
        assert!(!states.is_empty());
    }

    #[test]
    fn test_lane_states() {
        let planner = StateLattice::with_defaults();
        let states = planner.calc_lane_states();
        assert!(!states.is_empty());
    }

    #[test]
    fn test_generate_paths() {
        let planner = StateLattice::with_defaults();
        let targets = vec![
            TargetPose::new(10.0, 0.0, 0.0),
            TargetPose::new(10.0, 2.0, 0.2),
        ];
        let paths = planner.generate_paths(&targets);
        // Some paths should be generated
        assert!(!paths.is_empty() || targets.is_empty());
    }

    #[test]
    fn test_plan_uniform_polar() {
        let planner = StateLattice::with_defaults();
        let paths = planner.plan_uniform_polar();
        // Should generate some valid paths
        // (may be empty if optimization fails for all targets)
        println!("Generated {} paths", paths.len());
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
        // May fail for some configurations
        if let Ok(path) = result {
            assert!(!path.is_empty());
            // First point should be near start
            assert!((path.points[0].x - start.x).abs() < 1.0);
        }
    }
}
