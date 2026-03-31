//! Bidirectional A* path planning algorithm
//!
//! Grid-based path planning using bidirectional A* search.
//! Searches simultaneously from start and goal, terminating when
//! the two search frontiers meet.

use std::collections::HashMap;

use crate::grid::{GridMap, Node};
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Configuration for Bidirectional A* planner
#[derive(Debug, Clone)]
pub struct BidirectionalAStarConfig {
    /// Grid resolution in meters
    pub resolution: f64,
    /// Robot radius for obstacle inflation
    pub robot_radius: f64,
    /// Heuristic weight (1.0 = optimal, >1.0 = faster but suboptimal)
    pub heuristic_weight: f64,
}

impl Default for BidirectionalAStarConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        }
    }
}

impl BidirectionalAStarConfig {
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

/// Bidirectional A* path planner
pub struct BidirectionalAStarPlanner {
    grid_map: GridMap,
    config: BidirectionalAStarConfig,
    motion: Vec<(i32, i32, f64)>,
}

impl BidirectionalAStarPlanner {
    /// Create a new Bidirectional A* planner with obstacle positions
    pub fn new(ox: &[f64], oy: &[f64], config: BidirectionalAStarConfig) -> Self {
        Self::try_new(ox, oy, config).expect(
            "invalid Bidirectional A* planner input: obstacle list must be non-empty and valid, and config values must be positive/finite",
        )
    }

    /// Create a validated Bidirectional A* planner with obstacle positions
    pub fn try_new(
        ox: &[f64],
        oy: &[f64],
        config: BidirectionalAStarConfig,
    ) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.resolution, config.robot_radius)?;
        let motion = Self::get_motion_model();

        Ok(BidirectionalAStarPlanner {
            grid_map,
            config,
            motion,
        })
    }

    /// Create from obstacle x/y vectors with default config
    pub fn from_obstacles(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let config = BidirectionalAStarConfig {
            resolution,
            robot_radius,
            ..Default::default()
        };
        Self::new(ox, oy, config)
    }

    /// Create a validated Bidirectional A* planner from typed obstacle points
    pub fn from_obstacle_points(
        obstacles: &Obstacles,
        config: BidirectionalAStarConfig,
    ) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::from_obstacles(obstacles, config.resolution, config.robot_radius)?;
        let motion = Self::get_motion_model();

        Ok(BidirectionalAStarPlanner {
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

    fn calc_heuristic(&self, n1_x: i32, n1_y: i32, n2_x: i32, n2_y: i32) -> f64 {
        self.config.heuristic_weight * (((n1_x - n2_x).pow(2) + (n1_y - n2_y).pow(2)) as f64).sqrt()
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

    /// Find the key in `open_set` with the lowest f-cost = g + h toward `target`.
    fn best_open_key(
        &self,
        open_set: &HashMap<i32, usize>,
        node_storage: &[Node],
        target_x: i32,
        target_y: i32,
    ) -> i32 {
        *open_set
            .iter()
            .min_by(|a, b| {
                let na = &node_storage[*a.1];
                let nb = &node_storage[*b.1];
                let fa = na.cost + self.calc_heuristic(na.x, na.y, target_x, target_y);
                let fb = nb.cost + self.calc_heuristic(nb.x, nb.y, target_x, target_y);
                fa.partial_cmp(&fb).unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(k, _)| k)
            .unwrap()
    }

    /// Build a path segment by tracing parent pointers from `start_index` back to the root.
    /// Returns points in order from `start_index` back to root.
    fn trace_path(&self, start_index: usize, node_storage: &[Node]) -> Vec<Point2D> {
        let mut points = Vec::new();
        let mut current = Some(start_index);

        while let Some(idx) = current {
            let node = &node_storage[idx];
            points.push(Point2D::new(
                self.grid_map.calc_x_position(node.x),
                self.grid_map.calc_y_position(node.y),
            ));
            current = node.parent_index;
        }

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
        let mut node_storage_a: Vec<Node> = Vec::new();
        let mut open_set_a: HashMap<i32, usize> = HashMap::new();
        let mut closed_set_a: HashMap<i32, usize> = HashMap::new();

        // Backward search (from goal)
        let mut node_storage_b: Vec<Node> = Vec::new();
        let mut open_set_b: HashMap<i32, usize> = HashMap::new();
        let mut closed_set_b: HashMap<i32, usize> = HashMap::new();

        // Initialize forward start node
        node_storage_a.push(Node::new(start_x, start_y, 0.0, None));
        let start_grid_index = self.grid_map.calc_index(start_x, start_y);
        open_set_a.insert(start_grid_index, 0);

        // Initialize backward start node (goal)
        node_storage_b.push(Node::new(goal_x, goal_y, 0.0, None));
        let goal_grid_index = self.grid_map.calc_index(goal_x, goal_y);
        open_set_b.insert(goal_grid_index, 0);

        loop {
            if open_set_a.is_empty() || open_set_b.is_empty() {
                return Err(RoboticsError::PlanningError("No path found".to_string()));
            }

            // --- Expand forward (A) ---
            // Use the current best of B as heuristic target for A
            let current_b_idx = {
                let key = self.best_open_key(&open_set_b, &node_storage_b, start_x, start_y);
                node_storage_b[open_set_b[&key]].x // just need position for heuristic
            };
            let current_b_y = {
                let key = self.best_open_key(&open_set_b, &node_storage_b, start_x, start_y);
                node_storage_b[open_set_b[&key]].y
            };

            let c_id_a =
                self.best_open_key(&open_set_a, &node_storage_a, current_b_idx, current_b_y);
            let current_a_storage_idx = open_set_a[&c_id_a];
            let current_a_x = node_storage_a[current_a_storage_idx].x;
            let current_a_y = node_storage_a[current_a_storage_idx].y;
            let current_a_cost = node_storage_a[current_a_storage_idx].cost;

            // --- Expand backward (B) ---
            let c_id_b = self.best_open_key(&open_set_b, &node_storage_b, current_a_x, current_a_y);
            let current_b_storage_idx = open_set_b[&c_id_b];
            let current_b_x = node_storage_b[current_b_storage_idx].x;
            let current_b_y2 = node_storage_b[current_b_storage_idx].y;
            let current_b_cost = node_storage_b[current_b_storage_idx].cost;

            // Check meeting condition: both searches picked the same cell
            if current_a_x == current_b_x && current_a_y == current_b_y2 {
                // Build forward path (from meeting point back to start), then reverse
                let mut forward_points = self.trace_path(current_a_storage_idx, &node_storage_a);
                forward_points.reverse();

                // Build backward path (from meeting point back to goal)
                let backward_points = self.trace_path(current_b_storage_idx, &node_storage_b);

                // Join: forward (start..meeting) + backward (meeting..goal)
                // The meeting point is the last element of forward and first of backward,
                // so skip the first element of backward to avoid duplication.
                let mut all_points = forward_points;
                all_points.extend(backward_points.into_iter().skip(1));

                return Ok(Path2D::from_points(all_points));
            }

            // Move A and B to closed sets
            open_set_a.remove(&c_id_a);
            open_set_b.remove(&c_id_b);
            closed_set_a.insert(c_id_a, current_a_storage_idx);
            closed_set_b.insert(c_id_b, current_b_storage_idx);

            // Expand neighbors for both directions
            for &(dx, dy, move_cost) in &self.motion {
                // --- Forward neighbor ---
                let new_a_x = current_a_x + dx;
                let new_a_y = current_a_y + dy;
                let new_a_grid_index = self.grid_map.calc_index(new_a_x, new_a_y);

                if self.grid_map.is_valid(new_a_x, new_a_y)
                    && !closed_set_a.contains_key(&new_a_grid_index)
                {
                    let new_cost = current_a_cost + move_cost;
                    if let Some(&existing_idx) = open_set_a.get(&new_a_grid_index) {
                        if node_storage_a[existing_idx].cost > new_cost {
                            // Found a better path; add new node entry
                            node_storage_a.push(Node::new(
                                new_a_x,
                                new_a_y,
                                new_cost,
                                Some(current_a_storage_idx),
                            ));
                            let new_idx = node_storage_a.len() - 1;
                            open_set_a.insert(new_a_grid_index, new_idx);
                        }
                    } else {
                        node_storage_a.push(Node::new(
                            new_a_x,
                            new_a_y,
                            new_cost,
                            Some(current_a_storage_idx),
                        ));
                        let new_idx = node_storage_a.len() - 1;
                        open_set_a.insert(new_a_grid_index, new_idx);
                    }
                }

                // --- Backward neighbor ---
                let new_b_x = current_b_x + dx;
                let new_b_y = current_b_y2 + dy;
                let new_b_grid_index = self.grid_map.calc_index(new_b_x, new_b_y);

                if self.grid_map.is_valid(new_b_x, new_b_y)
                    && !closed_set_b.contains_key(&new_b_grid_index)
                {
                    let new_cost = current_b_cost + move_cost;
                    if let Some(&existing_idx) = open_set_b.get(&new_b_grid_index) {
                        if node_storage_b[existing_idx].cost > new_cost {
                            node_storage_b.push(Node::new(
                                new_b_x,
                                new_b_y,
                                new_cost,
                                Some(current_b_storage_idx),
                            ));
                            let new_idx = node_storage_b.len() - 1;
                            open_set_b.insert(new_b_grid_index, new_idx);
                        }
                    } else {
                        node_storage_b.push(Node::new(
                            new_b_x,
                            new_b_y,
                            new_cost,
                            Some(current_b_storage_idx),
                        ));
                        let new_idx = node_storage_b.len() - 1;
                        open_set_b.insert(new_b_grid_index, new_idx);
                    }
                }
            }
        }
    }
}

impl PathPlanner for BidirectionalAStarPlanner {
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
    fn test_bidirectional_a_star_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = BidirectionalAStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(8.0, 8.0);

        let result = planner.plan(start, goal);
        assert!(result.is_ok());

        let path = result.unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_bidirectional_a_star_legacy_interface() {
        let (ox, oy) = create_simple_obstacles();
        let planner = BidirectionalAStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let result = planner.planning(2.0, 2.0, 8.0, 8.0);
        assert!(result.is_some());

        let (rx, ry) = result.unwrap();
        assert!(!rx.is_empty());
        assert_eq!(rx.len(), ry.len());
    }

    #[test]
    fn test_bidirectional_a_star_from_obstacle_points() {
        let (ox, oy) = create_simple_obstacles();
        let obstacles = Obstacles::try_from_xy(&ox, &oy).unwrap();
        let planner = BidirectionalAStarPlanner::from_obstacle_points(
            &obstacles,
            BidirectionalAStarConfig::default(),
        )
        .unwrap();

        let path = planner.plan_xy(2.0, 2.0, 8.0, 8.0).unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_bidirectional_a_star_try_new_rejects_invalid_config() {
        let (ox, oy) = create_simple_obstacles();
        let config = BidirectionalAStarConfig {
            heuristic_weight: 0.0,
            ..Default::default()
        };

        let err = match BidirectionalAStarPlanner::try_new(&ox, &oy, config) {
            Ok(_) => panic!("expected invalid config to be rejected"),
            Err(err) => err,
        };
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_bidirectional_a_star_path_is_collision_free() {
        let (ox, oy) = create_simple_obstacles();
        let planner = BidirectionalAStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let path = planner.plan_xy(2.0, 2.0, 8.0, 8.0).unwrap();

        // Every point on the path should map to a valid (non-obstacle) grid cell
        let grid = planner.grid_map();
        for pt in &path.points {
            let gx = grid.calc_x_index(pt.x);
            let gy = grid.calc_y_index(pt.y);
            assert!(
                grid.is_valid(gx, gy),
                "Path point ({}, {}) maps to invalid grid cell ({}, {})",
                pt.x,
                pt.y,
                gx,
                gy
            );
        }
    }

    #[test]
    fn test_bidirectional_a_star_start_and_goal_correct() {
        let (ox, oy) = create_simple_obstacles();
        let planner = BidirectionalAStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let path = planner.plan_xy(2.0, 2.0, 8.0, 8.0).unwrap();

        let first = path.points.first().unwrap();
        let last = path.points.last().unwrap();
        assert!((first.x - 2.0).abs() < 1e-6);
        assert!((first.y - 2.0).abs() < 1e-6);
        assert!((last.x - 8.0).abs() < 1e-6);
        assert!((last.y - 8.0).abs() < 1e-6);
    }

    #[test]
    fn test_bidirectional_a_star_preserves_asymmetric_world_coordinates() {
        let mut obstacles = Obstacles::new();

        for x in 10..=20 {
            obstacles.push(Point2D::new(x as f64, -4.0));
            obstacles.push(Point2D::new(x as f64, 6.0));
        }
        for y in -4..=6 {
            obstacles.push(Point2D::new(10.0, y as f64));
            obstacles.push(Point2D::new(20.0, y as f64));
        }

        let planner = BidirectionalAStarPlanner::from_obstacle_points(
            &obstacles,
            BidirectionalAStarConfig::default(),
        )
        .unwrap();
        let path = planner.plan_xy(12.0, -2.0, 18.0, 4.0).unwrap();

        let first = path.points.first().unwrap();
        let last = path.points.last().unwrap();
        assert!((first.x - 12.0).abs() < 1e-6);
        assert!((first.y + 2.0).abs() < 1e-6);
        assert!((last.x - 18.0).abs() < 1e-6);
        assert!((last.y - 4.0).abs() < 1e-6);
    }
}
