//! Bidirectional Breadth-First Search path planning
//!
//! Grid-based path planning that searches from both start and goal
//! simultaneously, meeting in the middle.

use std::collections::{HashMap, VecDeque};

use crate::grid::{GridMap, Node};
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Configuration for Bidirectional BFS planner
#[derive(Debug, Clone)]
pub struct BidirectionalBFSConfig {
    /// Grid resolution in meters
    pub resolution: f64,
    /// Robot radius for obstacle inflation
    pub robot_radius: f64,
}

impl Default for BidirectionalBFSConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
        }
    }
}

impl BidirectionalBFSConfig {
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

/// Bidirectional Breadth-First Search planner
pub struct BidirectionalBFSPlanner {
    grid_map: GridMap,
    #[allow(dead_code)]
    config: BidirectionalBFSConfig,
    motion: Vec<(i32, i32, f64)>,
}

impl BidirectionalBFSPlanner {
    pub fn new(ox: &[f64], oy: &[f64], config: BidirectionalBFSConfig) -> Self {
        Self::try_new(ox, oy, config).expect("invalid BidirectionalBFS planner input")
    }

    pub fn try_new(ox: &[f64], oy: &[f64], config: BidirectionalBFSConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.resolution, config.robot_radius)?;
        let motion = Self::get_motion_model();
        Ok(Self {
            grid_map,
            config,
            motion,
        })
    }

    pub fn from_obstacles(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let config = BidirectionalBFSConfig {
            resolution,
            robot_radius,
        };
        Self::new(ox, oy, config)
    }

    pub fn from_obstacle_points(
        obstacles: &Obstacles,
        config: BidirectionalBFSConfig,
    ) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::from_obstacles(obstacles, config.resolution, config.robot_radius)?;
        let motion = Self::get_motion_model();
        Ok(Self {
            grid_map,
            config,
            motion,
        })
    }

    pub fn planning(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> Option<(Vec<f64>, Vec<f64>)> {
        match self.plan_xy(sx, sy, gx, gy) {
            Ok(path) => Some((path.x_coords(), path.y_coords())),
            Err(_) => None,
        }
    }

    pub fn plan(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        self.plan_impl(start, goal)
    }

    pub fn plan_xy(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> RoboticsResult<Path2D> {
        self.plan_impl(Point2D::new(sx, sy), Point2D::new(gx, gy))
    }

    pub fn grid_map(&self) -> &GridMap {
        &self.grid_map
    }

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

    fn build_half_path(&self, goal_index: usize, node_storage: &[Node]) -> Vec<Point2D> {
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
        points
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

        // Forward search (from start)
        let mut fwd_queue = VecDeque::new();
        let mut fwd_closed: HashMap<i32, usize> = HashMap::new();
        let mut fwd_storage: Vec<Node> = Vec::new();

        fwd_storage.push(Node::new(start_x, start_y, 0.0, None));
        fwd_queue.push_back(0usize);
        fwd_closed.insert(self.grid_map.calc_index(start_x, start_y), 0);

        // Backward search (from goal)
        let mut bwd_queue = VecDeque::new();
        let mut bwd_closed: HashMap<i32, usize> = HashMap::new();
        let mut bwd_storage: Vec<Node> = Vec::new();

        bwd_storage.push(Node::new(goal_x, goal_y, 0.0, None));
        bwd_queue.push_back(0usize);
        bwd_closed.insert(self.grid_map.calc_index(goal_x, goal_y), 0);

        loop {
            // Expand forward
            if let Some(fwd_idx) = fwd_queue.pop_front() {
                let fwd_node = &fwd_storage[fwd_idx];
                let fx = fwd_node.x;
                let fy = fwd_node.y;
                let fcost = fwd_node.cost;
                let fwd_grid_index = self.grid_map.calc_index(fx, fy);

                // Check meeting condition
                if let Some(&bwd_idx) = bwd_closed.get(&fwd_grid_index) {
                    let mut path = self.build_half_path(fwd_idx, &fwd_storage);
                    let mut bwd_path = self.build_half_path(bwd_idx, &bwd_storage);
                    bwd_path.reverse();
                    if !bwd_path.is_empty() {
                        path.extend_from_slice(&bwd_path[1..]);
                    }
                    return Ok(Path2D::from_points(path));
                }

                for &(dx, dy, move_cost) in &self.motion {
                    let nx = fx + dx;
                    let ny = fy + dy;
                    let new_grid_index = self.grid_map.calc_index(nx, ny);

                    if !self.grid_map.is_valid(nx, ny) || fwd_closed.contains_key(&new_grid_index) {
                        continue;
                    }

                    fwd_storage.push(Node::new(nx, ny, fcost + move_cost, Some(fwd_idx)));
                    let new_idx = fwd_storage.len() - 1;
                    fwd_closed.insert(new_grid_index, new_idx);
                    fwd_queue.push_back(new_idx);
                }
            } else {
                return Err(RoboticsError::PlanningError(
                    "No path found (forward queue empty)".to_string(),
                ));
            }

            // Expand backward
            if let Some(bwd_idx) = bwd_queue.pop_front() {
                let bwd_node = &bwd_storage[bwd_idx];
                let bx = bwd_node.x;
                let by = bwd_node.y;
                let bcost = bwd_node.cost;
                let bwd_grid_index = self.grid_map.calc_index(bx, by);

                // Check meeting condition
                if let Some(&fwd_idx) = fwd_closed.get(&bwd_grid_index) {
                    let mut path = self.build_half_path(fwd_idx, &fwd_storage);
                    let mut bwd_path = self.build_half_path(bwd_idx, &bwd_storage);
                    bwd_path.reverse();
                    if !bwd_path.is_empty() {
                        path.extend_from_slice(&bwd_path[1..]);
                    }
                    return Ok(Path2D::from_points(path));
                }

                for &(dx, dy, move_cost) in &self.motion {
                    let nx = bx + dx;
                    let ny = by + dy;
                    let new_grid_index = self.grid_map.calc_index(nx, ny);

                    if !self.grid_map.is_valid(nx, ny) || bwd_closed.contains_key(&new_grid_index) {
                        continue;
                    }

                    bwd_storage.push(Node::new(nx, ny, bcost + move_cost, Some(bwd_idx)));
                    let new_idx = bwd_storage.len() - 1;
                    bwd_closed.insert(new_grid_index, new_idx);
                    bwd_queue.push_back(new_idx);
                }
            } else {
                return Err(RoboticsError::PlanningError(
                    "No path found (backward queue empty)".to_string(),
                ));
            }
        }
    }
}

impl PathPlanner for BidirectionalBFSPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        self.plan_impl(start, goal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn test_bidirectional_bfs_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = BidirectionalBFSPlanner::from_obstacles(&ox, &oy, 0.5, 0.1);

        let result = planner.planning(2.0, 2.0, 8.0, 8.0);
        assert!(result.is_some());
        let (rx, ry) = result.unwrap();
        assert!(rx.len() > 2);
        assert_eq!(rx.len(), ry.len());
    }

    #[test]
    fn test_bidirectional_bfs_plan_returns_valid_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = BidirectionalBFSPlanner::from_obstacles(&ox, &oy, 0.5, 0.1);

        let path = planner.plan(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0));
        assert!(path.is_ok());
        let path = path.unwrap();
        assert!(path.len() > 2);
    }

    #[test]
    fn test_bidirectional_bfs_no_path() {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        // Boundary
        for i in -1..=61 {
            let v = i as f64;
            ox.push(v);
            oy.push(-1.0);
            ox.push(v);
            oy.push(61.0);
            ox.push(-1.0);
            oy.push(v);
            ox.push(61.0);
            oy.push(v);
        }

        // Complete dense wall at x=30
        for i in -1..=61 {
            for dx in -1..=1 {
                ox.push(30.0 + dx as f64);
                oy.push(i as f64);
            }
        }

        let planner = BidirectionalBFSPlanner::from_obstacles(&ox, &oy, 1.0, 0.0);
        let result = planner.plan(Point2D::new(10.0, 30.0), Point2D::new(50.0, 30.0));
        assert!(result.is_err());
    }

    #[test]
    fn test_bidirectional_bfs_from_obstacle_points() {
        let (ox, oy) = create_simple_obstacles();
        let obstacles = Obstacles::try_from_xy(&ox, &oy).expect("obstacle creation should succeed");
        let config = BidirectionalBFSConfig {
            resolution: 0.5,
            robot_radius: 0.1,
        };
        let planner = BidirectionalBFSPlanner::from_obstacle_points(&obstacles, config);
        assert!(planner.is_ok());
    }

    #[test]
    fn test_bidirectional_bfs_legacy_interface() {
        let (ox, oy) = create_simple_obstacles();
        let planner = BidirectionalBFSPlanner::from_obstacles(&ox, &oy, 0.5, 0.1);

        let result = planner.planning(2.0, 2.0, 8.0, 8.0);
        assert!(result.is_some());
    }
}
