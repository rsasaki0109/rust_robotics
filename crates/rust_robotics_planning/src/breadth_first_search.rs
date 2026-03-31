//! Breadth-First Search path planning algorithm
//!
//! Grid-based path planning using BFS. Unlike A*, BFS uses a FIFO queue
//! instead of a priority queue and does not use a heuristic for node ordering.
//! It guarantees the shortest path in terms of number of grid steps (unweighted).

use std::collections::{HashMap, VecDeque};

use crate::grid::{GridMap, Node};
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Configuration for BFS planner
#[derive(Debug, Clone)]
pub struct BFSConfig {
    /// Grid resolution in meters
    pub resolution: f64,
    /// Robot radius for obstacle inflation
    pub robot_radius: f64,
}

impl Default for BFSConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
        }
    }
}

impl BFSConfig {
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

/// Node entry for BFS FIFO queue
#[derive(Debug)]
struct QueueNode {
    x: i32,
    y: i32,
    cost: f64,
    index: usize,
}

/// Breadth-First Search path planner
pub struct BFSPlanner {
    grid_map: GridMap,
    #[allow(dead_code)]
    config: BFSConfig,
    motion: Vec<(i32, i32, f64)>,
}

impl BFSPlanner {
    /// Create a new BFS planner with obstacle positions
    pub fn new(ox: &[f64], oy: &[f64], config: BFSConfig) -> Self {
        Self::try_new(ox, oy, config).expect(
            "invalid BFS planner input: obstacle list must be non-empty and valid, and config values must be positive/finite",
        )
    }

    /// Create a validated BFS planner with obstacle positions
    pub fn try_new(ox: &[f64], oy: &[f64], config: BFSConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.resolution, config.robot_radius)?;
        let motion = Self::get_motion_model();

        Ok(BFSPlanner {
            grid_map,
            config,
            motion,
        })
    }

    /// Create from obstacle x/y vectors with default config
    pub fn from_obstacles(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let config = BFSConfig {
            resolution,
            robot_radius,
        };
        Self::new(ox, oy, config)
    }

    /// Create a validated BFS planner from typed obstacle points
    pub fn from_obstacle_points(obstacles: &Obstacles, config: BFSConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::from_obstacles(obstacles, config.resolution, config.robot_radius)?;
        let motion = Self::get_motion_model();

        Ok(BFSPlanner {
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

    /// 8-connected motion model matching PythonRobotics BFS default
    fn get_motion_model() -> Vec<(i32, i32, f64)> {
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

        let mut queue = VecDeque::new();
        let mut closed_set = HashMap::new();
        let mut node_storage: Vec<Node> = Vec::new();

        // Add start node
        node_storage.push(Node::new(start_x, start_y, 0.0, None));
        let start_index = 0;

        queue.push_back(QueueNode {
            x: start_x,
            y: start_y,
            cost: 0.0,
            index: start_index,
        });

        let start_grid_index = self.grid_map.calc_index(start_x, start_y);
        closed_set.insert(start_grid_index, start_index);

        while let Some(current) = queue.pop_front() {
            // Check if we reached the goal
            if current.x == goal_x && current.y == goal_y {
                return Ok(self.build_path(current.index, &node_storage));
            }

            // Expand neighbors
            for &(dx, dy, move_cost) in &self.motion {
                let new_x = current.x + dx;
                let new_y = current.y + dy;
                let new_cost = current.cost + move_cost;
                let new_grid_index = self.grid_map.calc_index(new_x, new_y);

                // Skip if not valid or already visited
                if !self.grid_map.is_valid(new_x, new_y) {
                    continue;
                }
                if closed_set.contains_key(&new_grid_index) {
                    continue;
                }

                // Add to storage and queue
                node_storage.push(Node::new(new_x, new_y, new_cost, Some(current.index)));
                let new_index = node_storage.len() - 1;

                closed_set.insert(new_grid_index, new_index);
                queue.push_back(QueueNode {
                    x: new_x,
                    y: new_y,
                    cost: new_cost,
                    index: new_index,
                });
            }
        }

        Err(RoboticsError::PlanningError("No path found".to_string()))
    }
}

impl PathPlanner for BFSPlanner {
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
    fn test_bfs_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = BFSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(8.0, 8.0);

        let result = planner.plan(start, goal);
        assert!(result.is_ok());

        let path = result.unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_bfs_legacy_interface() {
        let (ox, oy) = create_simple_obstacles();
        let planner = BFSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let result = planner.planning(2.0, 2.0, 8.0, 8.0);
        assert!(result.is_some());

        let (rx, ry) = result.unwrap();
        assert!(!rx.is_empty());
        assert_eq!(rx.len(), ry.len());
    }

    #[test]
    fn test_bfs_from_obstacle_points() {
        let (ox, oy) = create_simple_obstacles();
        let obstacles = Obstacles::try_from_xy(&ox, &oy).unwrap();
        let planner = BFSPlanner::from_obstacle_points(&obstacles, BFSConfig::default()).unwrap();

        let path = planner.plan_xy(2.0, 2.0, 8.0, 8.0).unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_bfs_try_new_rejects_invalid_config() {
        let (ox, oy) = create_simple_obstacles();
        let config = BFSConfig {
            resolution: 0.0,
            ..Default::default()
        };

        let err = match BFSPlanner::try_new(&ox, &oy, config) {
            Ok(_) => panic!("expected invalid config to be rejected"),
            Err(err) => err,
        };
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_bfs_preserves_asymmetric_world_coordinates() {
        let mut obstacles = Obstacles::new();

        for x in 10..=20 {
            obstacles.push(Point2D::new(x as f64, -4.0));
            obstacles.push(Point2D::new(x as f64, 6.0));
        }
        for y in -4..=6 {
            obstacles.push(Point2D::new(10.0, y as f64));
            obstacles.push(Point2D::new(20.0, y as f64));
        }

        let planner = BFSPlanner::from_obstacle_points(&obstacles, BFSConfig::default()).unwrap();
        let path = planner.plan_xy(12.0, -2.0, 18.0, 4.0).unwrap();

        let first = path.points.first().unwrap();
        let last = path.points.last().unwrap();
        assert!((first.x - 12.0).abs() < 1e-6);
        assert!((first.y + 2.0).abs() < 1e-6);
        assert!((last.x - 18.0).abs() < 1e-6);
        assert!((last.y - 4.0).abs() < 1e-6);
    }
}
