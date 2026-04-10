//! Fringe Search path planning algorithm.
//!
//! Fringe Search replaces the priority queue of A* with iteratively
//! increasing `f`-cost thresholds over two fringe lists. It keeps the same
//! admissible Euclidean heuristic as A* while avoiding heap maintenance.

use std::collections::{HashMap, VecDeque};

use crate::grid::{get_motion_model_8, GridMap, Node};
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Configuration for Fringe Search.
#[derive(Debug, Clone)]
pub struct FringeSearchConfig {
    /// Grid resolution in meters.
    pub resolution: f64,
    /// Robot radius for obstacle inflation.
    pub robot_radius: f64,
    /// Heuristic weight (1.0 = admissible, >1.0 = weighted search).
    pub heuristic_weight: f64,
}

impl Default for FringeSearchConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        }
    }
}

impl FringeSearchConfig {
    pub fn validate(&self) -> RoboticsResult<()> {
        if !self.resolution.is_finite() || self.resolution <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "resolution must be positive and finite, got {}",
                self.resolution
            )));
        }
        if !self.robot_radius.is_finite() || self.robot_radius < 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "robot_radius must be non-negative and finite, got {}",
                self.robot_radius
            )));
        }
        if !self.heuristic_weight.is_finite() || self.heuristic_weight <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "heuristic_weight must be positive and finite, got {}",
                self.heuristic_weight
            )));
        }

        Ok(())
    }
}

/// Fringe Search planner on an 8-connected occupancy grid.
#[derive(Debug)]
pub struct FringeSearchPlanner {
    grid_map: GridMap,
    config: FringeSearchConfig,
    motion: Vec<(i32, i32, f64)>,
}

/// Search-effort statistics collected during one Fringe Search query.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FringeSearchStats {
    pub threshold_iterations: usize,
    pub expanded_nodes: usize,
    pub generated_nodes: usize,
    pub deferred_nodes: usize,
    pub stale_nodes: usize,
    pub max_active_nodes: usize,
}

impl FringeSearchPlanner {
    /// Create a new planner from obstacle x/y vectors.
    pub fn new(ox: &[f64], oy: &[f64], config: FringeSearchConfig) -> Self {
        Self::try_new(ox, oy, config).expect(
            "invalid Fringe Search planner input: obstacle list must be non-empty and valid, and config values must be positive/finite",
        )
    }

    /// Create a validated planner from obstacle x/y vectors.
    pub fn try_new(ox: &[f64], oy: &[f64], config: FringeSearchConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.resolution, config.robot_radius)?;

        Ok(Self {
            grid_map,
            config,
            motion: get_motion_model_8(),
        })
    }

    /// Create from obstacle x/y vectors with explicit scalar parameters.
    pub fn from_obstacles(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let config = FringeSearchConfig {
            resolution,
            robot_radius,
            ..Default::default()
        };
        Self::new(ox, oy, config)
    }

    /// Create from typed obstacle points.
    pub fn from_obstacle_points(
        obstacles: &Obstacles,
        config: FringeSearchConfig,
    ) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::from_obstacles(obstacles, config.resolution, config.robot_radius)?;

        Ok(Self {
            grid_map,
            config,
            motion: get_motion_model_8(),
        })
    }

    /// Plan without importing the trait explicitly.
    pub fn plan(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        self.plan_impl(start, goal).map(|(path, _stats)| path)
    }

    /// Plan from raw coordinates without importing the trait explicitly.
    pub fn plan_xy(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> RoboticsResult<Path2D> {
        self.plan_impl(Point2D::new(sx, sy), Point2D::new(gx, gy))
            .map(|(path, _stats)| path)
    }

    /// Plan a path and return per-query search-effort statistics.
    pub fn plan_with_stats(
        &self,
        start: Point2D,
        goal: Point2D,
    ) -> RoboticsResult<(Path2D, FringeSearchStats)> {
        self.plan_impl(start, goal)
    }

    pub fn grid_map(&self) -> &GridMap {
        &self.grid_map
    }

    fn calc_heuristic(&self, x: i32, y: i32, goal_x: i32, goal_y: i32) -> f64 {
        self.config.heuristic_weight * (((x - goal_x).pow(2) + (y - goal_y).pow(2)) as f64).sqrt()
    }

    fn build_path(&self, goal_index: usize, node_storage: &[Node]) -> Path2D {
        let mut points = Vec::new();
        let mut current_index = Some(goal_index);

        while let Some(index) = current_index {
            let node = &node_storage[index];
            points.push(Point2D::new(
                self.grid_map.calc_x_position(node.x),
                self.grid_map.calc_y_position(node.y),
            ));
            current_index = node.parent_index;
        }

        points.reverse();
        Path2D::from_points(points)
    }

    fn ensure_query_is_valid(&self, x: i32, y: i32, label: &str) -> RoboticsResult<()> {
        if self.grid_map.is_valid(x, y) {
            return Ok(());
        }

        Err(RoboticsError::PlanningError(format!(
            "{} position is invalid",
            label
        )))
    }

    fn plan_impl(
        &self,
        start: Point2D,
        goal: Point2D,
    ) -> RoboticsResult<(Path2D, FringeSearchStats)> {
        let start_x = self.grid_map.calc_x_index(start.x);
        let start_y = self.grid_map.calc_y_index(start.y);
        let goal_x = self.grid_map.calc_x_index(goal.x);
        let goal_y = self.grid_map.calc_y_index(goal.y);

        self.ensure_query_is_valid(start_x, start_y, "Start")?;
        self.ensure_query_is_valid(goal_x, goal_y, "Goal")?;

        let mut node_storage = vec![Node::new(start_x, start_y, 0.0, None)];
        let start_grid_index = self.grid_map.calc_index(start_x, start_y);

        let mut best_cost_by_grid = HashMap::new();
        best_cost_by_grid.insert(start_grid_index, 0.0);

        let mut current_fringe = VecDeque::new();
        current_fringe.push_back(0usize);

        let mut f_limit = self.calc_heuristic(start_x, start_y, goal_x, goal_y);
        let mut stats = FringeSearchStats {
            max_active_nodes: current_fringe.len(),
            ..Default::default()
        };

        while !current_fringe.is_empty() {
            stats.threshold_iterations += 1;
            let mut next_fringe = VecDeque::new();
            let mut min_exceeded = f64::INFINITY;

            while let Some(current_index) = current_fringe.pop_front() {
                let current_node = node_storage[current_index].clone();
                let current_grid_index = self.grid_map.calc_index(current_node.x, current_node.y);

                let Some(&best_known_cost) = best_cost_by_grid.get(&current_grid_index) else {
                    continue;
                };
                if current_node.cost > best_known_cost + 1e-9 {
                    stats.stale_nodes += 1;
                    continue;
                }

                let f_cost = current_node.cost
                    + self.calc_heuristic(current_node.x, current_node.y, goal_x, goal_y);
                if f_cost > f_limit {
                    stats.deferred_nodes += 1;
                    min_exceeded = min_exceeded.min(f_cost);
                    next_fringe.push_back(current_index);
                    stats.max_active_nodes = stats
                        .max_active_nodes
                        .max(current_fringe.len() + next_fringe.len());
                    continue;
                }

                stats.expanded_nodes += 1;

                if current_node.x == goal_x && current_node.y == goal_y {
                    return Ok((self.build_path(current_index, &node_storage), stats));
                }

                for &(dx, dy, move_cost) in &self.motion {
                    if !self
                        .grid_map
                        .is_valid_offset(current_node.x, current_node.y, dx, dy)
                    {
                        continue;
                    }

                    let next_x = current_node.x + dx;
                    let next_y = current_node.y + dy;
                    let next_grid_index = self.grid_map.calc_index(next_x, next_y);
                    let next_cost = current_node.cost + move_cost;

                    if best_cost_by_grid
                        .get(&next_grid_index)
                        .is_some_and(|&known_cost| next_cost >= known_cost - 1e-9)
                    {
                        continue;
                    }

                    node_storage.push(Node::new(next_x, next_y, next_cost, Some(current_index)));
                    let next_index = node_storage.len() - 1;
                    best_cost_by_grid.insert(next_grid_index, next_cost);
                    stats.generated_nodes += 1;

                    let next_f_cost =
                        next_cost + self.calc_heuristic(next_x, next_y, goal_x, goal_y);
                    if next_f_cost <= f_limit {
                        current_fringe.push_back(next_index);
                    } else {
                        min_exceeded = min_exceeded.min(next_f_cost);
                        next_fringe.push_back(next_index);
                    }
                    stats.max_active_nodes = stats
                        .max_active_nodes
                        .max(current_fringe.len() + next_fringe.len());
                }
            }

            if next_fringe.is_empty() || !min_exceeded.is_finite() {
                break;
            }

            current_fringe = next_fringe;
            f_limit = min_exceeded;
        }

        Err(RoboticsError::PlanningError("No path found".to_string()))
    }
}

impl PathPlanner for FringeSearchPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        self.plan_impl(start, goal).map(|(path, _stats)| path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::a_star::{AStarConfig, AStarPlanner};
    use crate::moving_ai::{MovingAiMap, MovingAiScenario};

    fn create_simple_obstacles() -> (Vec<f64>, Vec<f64>) {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        for i in 0..11 {
            ox.push(i as f64);
            oy.push(0.0);
            ox.push(i as f64);
            oy.push(10.0);
            ox.push(0.0);
            oy.push(i as f64);
            ox.push(10.0);
            oy.push(i as f64);
        }

        for i in 4..7 {
            ox.push(5.0);
            oy.push(i as f64);
        }

        (ox, oy)
    }

    #[test]
    fn test_fringe_search_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = FringeSearchPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let path = planner
            .plan(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0))
            .expect("Fringe Search should find a path on the simple map");

        assert!(!path.is_empty());
    }

    #[test]
    fn test_fringe_search_try_new_rejects_invalid_config() {
        let (ox, oy) = create_simple_obstacles();
        let config = FringeSearchConfig {
            heuristic_weight: 0.0,
            ..Default::default()
        };

        let err = FringeSearchPlanner::try_new(&ox, &oy, config)
            .expect_err("invalid heuristic weight should be rejected");
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_fringe_search_matches_simple_map_length() {
        let (ox, oy) = create_simple_obstacles();
        let planner = FringeSearchPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);
        let a_star = AStarPlanner::new(
            &ox,
            &oy,
            AStarConfig {
                resolution: 1.0,
                robot_radius: 0.5,
                heuristic_weight: 1.0,
            },
        );

        let path = planner
            .plan(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0))
            .expect("Fringe Search should find a path on the simple map");
        let reference = a_star
            .plan(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0))
            .expect("A* should find the same reference path on the simple map");

        assert!(
            (path.total_length() - reference.total_length()).abs() < 1e-6,
            "simple-map path length {} should match A* reference length {}",
            path.total_length(),
            reference.total_length()
        );
    }

    #[test]
    fn test_fringe_search_matches_moving_ai_arena2_bucket_80_optimal_length() {
        let map = MovingAiMap::parse_str(include_str!("../benchdata/moving_ai/dao/arena2.map"))
            .expect("arena2 MovingAI map should parse");
        let scenario =
            MovingAiScenario::parse_str(include_str!("../benchdata/moving_ai/dao/arena2.map.scen"))
                .expect("arena2 MovingAI scenarios should parse")
                .into_iter()
                .find(|row| row.bucket == 80)
                .expect("arena2 MovingAI bucket 80 scenario should exist");

        let planner = FringeSearchPlanner::from_obstacle_points(
            &map.to_obstacles(),
            FringeSearchConfig {
                resolution: 1.0,
                robot_radius: 0.5,
                heuristic_weight: 1.0,
            },
        )
        .expect("Fringe Search planner should build from arena2 obstacles");

        let start = map
            .planning_point(scenario.start_x, scenario.start_y)
            .expect("arena2 start should be valid");
        let goal = map
            .planning_point(scenario.goal_x, scenario.goal_y)
            .expect("arena2 goal should be valid");

        let path = planner
            .plan(start, goal)
            .expect("Fringe Search should solve the arena2 bucket 80 scenario");

        assert!(
            (path.total_length() - scenario.optimal_length).abs() < 1e-6,
            "Fringe Search path length {} should match MovingAI optimal {} when corner cutting is disabled",
            path.total_length(),
            scenario.optimal_length
        );
    }
}
