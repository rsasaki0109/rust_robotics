//! Depth-First Search path planning algorithm
//!
//! Grid-based path planning using depth-first search with a LIFO stack.
//! DFS does **not** guarantee the shortest path, but will find a valid
//! path if one exists.

use std::collections::HashSet;

use crate::grid::{GridMap, Node};
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Configuration for DFS planner
#[derive(Debug, Clone)]
pub struct DFSConfig {
    /// Grid resolution in meters
    pub resolution: f64,
    /// Robot radius for obstacle inflation
    pub robot_radius: f64,
}

impl Default for DFSConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
        }
    }
}

impl DFSConfig {
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

        Ok(())
    }
}

/// Depth-First Search path planner
pub struct DFSPlanner {
    grid_map: GridMap,
    #[allow(dead_code)]
    config: DFSConfig,
    motion: Vec<(i32, i32, f64)>,
}

impl DFSPlanner {
    /// Create a new DFS planner with obstacle positions
    pub fn new(ox: &[f64], oy: &[f64], config: DFSConfig) -> Self {
        Self::try_new(ox, oy, config).expect(
            "invalid DFS planner input: obstacle list must be non-empty and valid, and config values must be positive/finite",
        )
    }

    /// Create a validated DFS planner with obstacle positions
    pub fn try_new(ox: &[f64], oy: &[f64], config: DFSConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.resolution, config.robot_radius)?;
        let motion = Self::get_motion_model();

        Ok(DFSPlanner {
            grid_map,
            config,
            motion,
        })
    }

    /// Create from obstacle x/y vectors with default config
    pub fn from_obstacles(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let config = DFSConfig {
            resolution,
            robot_radius,
        };
        Self::new(ox, oy, config)
    }

    /// Create a validated DFS planner from typed obstacle points
    pub fn from_obstacle_points(obstacles: &Obstacles, config: DFSConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::from_obstacles(obstacles, config.resolution, config.robot_radius)?;
        let motion = Self::get_motion_model();

        Ok(DFSPlanner {
            grid_map,
            config,
            motion,
        })
    }

    /// Plan a path returning (rx, ry) vectors (legacy interface)
    pub fn planning(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> Option<(Vec<f64>, Vec<f64>)> {
        match self.plan_xy(sx, sy, gx, gy) {
            Ok(path) => Some((path.x_coords(), path.y_coords())),
            Err(_) => None,
        }
    }

    /// Plan a path without requiring the PathPlanner trait in scope
    pub fn plan(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        self.plan_impl(start, goal)
    }

    /// Plan a path from raw coordinates without requiring the PathPlanner trait in scope
    pub fn plan_xy(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> RoboticsResult<Path2D> {
        self.plan_impl(Point2D::new(sx, sy), Point2D::new(gx, gy))
    }

    /// Get reference to the grid map
    pub fn grid_map(&self) -> &GridMap {
        &self.grid_map
    }

    fn get_motion_model() -> Vec<(i32, i32, f64)> {
        // dx, dy, cost (8-connected grid)
        vec![
            (1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (-1, -1, std::f64::consts::SQRT_2),
            (-1, 1, std::f64::consts::SQRT_2),
            (1, -1, std::f64::consts::SQRT_2),
            (1, 1, std::f64::consts::SQRT_2),
        ]
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

    fn plan_impl(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        let start_x = self.grid_map.calc_x_index(start.x);
        let start_y = self.grid_map.calc_y_index(start.y);
        let goal_x = self.grid_map.calc_x_index(goal.x);
        let goal_y = self.grid_map.calc_y_index(goal.y);

        self.ensure_query_is_valid(start_x, start_y, "Start")?;
        self.ensure_query_is_valid(goal_x, goal_y, "Goal")?;

        // LIFO stack for DFS (Vec with push/pop)
        let mut stack: Vec<usize> = Vec::new();
        let mut closed_set: HashSet<i32> = HashSet::new();
        let mut node_storage: Vec<Node> = Vec::new();

        // Add start node
        node_storage.push(Node::new(start_x, start_y, 0.0, None));
        let start_storage_index = 0;
        stack.push(start_storage_index);

        while let Some(current_storage_index) = stack.pop() {
            let current_x = node_storage[current_storage_index].x;
            let current_y = node_storage[current_storage_index].y;
            let current_cost = node_storage[current_storage_index].cost;
            let current_grid_index = self.grid_map.calc_index(current_x, current_y);

            // Check if we reached the goal
            if current_x == goal_x && current_y == goal_y {
                return Ok(self.build_path(current_storage_index, &node_storage));
            }

            // Skip if already visited
            if closed_set.contains(&current_grid_index) {
                continue;
            }

            // Mark as visited
            closed_set.insert(current_grid_index);

            // Expand neighbors (push to stack for LIFO ordering)
            for &(dx, dy, move_cost) in &self.motion {
                let new_x = current_x + dx;
                let new_y = current_y + dy;
                let new_cost = current_cost + move_cost;
                let new_grid_index = self.grid_map.calc_index(new_x, new_y);

                // Skip if not valid or already visited
                if !self.grid_map.is_valid(new_x, new_y) {
                    continue;
                }
                if closed_set.contains(&new_grid_index) {
                    continue;
                }

                // Add to storage and stack
                node_storage.push(Node::new(
                    new_x,
                    new_y,
                    new_cost,
                    Some(current_storage_index),
                ));
                let new_storage_index = node_storage.len() - 1;
                stack.push(new_storage_index);
            }
        }

        Err(RoboticsError::PlanningError("No path found".to_string()))
    }
}

impl PathPlanner for DFSPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        self.plan_impl(start, goal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use rust_robotics_core::Obstacles;

    fn create_simple_obstacles() -> (Vec<f64>, Vec<f64>) {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        // Boundary
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

        // Internal obstacle
        for i in 4..7 {
            ox.push(5.0);
            oy.push(i as f64);
        }

        (ox, oy)
    }

    #[test]
    fn test_dfs_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = DFSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(8.0, 8.0);

        let result = planner.plan(start, goal);
        assert!(result.is_ok());

        let path = result.unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_dfs_legacy_interface() {
        let (ox, oy) = create_simple_obstacles();
        let planner = DFSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let result = planner.planning(2.0, 2.0, 8.0, 8.0);
        assert!(result.is_some());

        let (rx, ry) = result.unwrap();
        assert!(!rx.is_empty());
        assert_eq!(rx.len(), ry.len());
    }

    #[test]
    fn test_dfs_from_obstacle_points() {
        let (ox, oy) = create_simple_obstacles();
        let obstacles = Obstacles::try_from_xy(&ox, &oy).unwrap();
        let planner = DFSPlanner::from_obstacle_points(&obstacles, DFSConfig::default()).unwrap();

        let path = planner.plan_xy(2.0, 2.0, 8.0, 8.0).unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_dfs_try_new_rejects_invalid_config() {
        let (ox, oy) = create_simple_obstacles();
        let config = DFSConfig {
            resolution: 0.0,
            ..Default::default()
        };

        let err = match DFSPlanner::try_new(&ox, &oy, config) {
            Ok(_) => panic!("expected invalid config to be rejected"),
            Err(err) => err,
        };
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_dfs_preserves_asymmetric_world_coordinates() {
        let mut obstacles = Obstacles::new();

        for x in 10..=20 {
            obstacles.push(Point2D::new(x as f64, -4.0));
            obstacles.push(Point2D::new(x as f64, 6.0));
        }
        for y in -4..=6 {
            obstacles.push(Point2D::new(10.0, y as f64));
            obstacles.push(Point2D::new(20.0, y as f64));
        }

        let planner = DFSPlanner::from_obstacle_points(&obstacles, DFSConfig::default()).unwrap();
        let path = planner.plan_xy(12.0, -2.0, 18.0, 4.0).unwrap();

        let first = path.points.first().unwrap();
        let last = path.points.last().unwrap();
        assert!((first.x - 12.0).abs() < 1e-6);
        assert!((first.y + 2.0).abs() < 1e-6);
        assert!((last.x - 18.0).abs() < 1e-6);
        assert!((last.y - 4.0).abs() < 1e-6);
    }

    #[test]
    fn test_dfs_path_starts_at_start_and_ends_at_goal() {
        let (ox, oy) = create_simple_obstacles();
        let planner = DFSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let path = planner.plan_xy(2.0, 2.0, 8.0, 8.0).unwrap();
        let first = path.points.first().unwrap();
        let last = path.points.last().unwrap();

        assert!((first.x - 2.0).abs() < 1e-6);
        assert!((first.y - 2.0).abs() < 1e-6);
        assert!((last.x - 8.0).abs() < 1e-6);
        assert!((last.y - 8.0).abs() < 1e-6);
    }
}
