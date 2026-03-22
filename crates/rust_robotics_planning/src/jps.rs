#![allow(clippy::collapsible_if)]

//! Jump Point Search (JPS) path planning algorithm
//!
//! JPS is an optimization of A* for uniform-cost grids that reduces the
//! number of nodes expanded by identifying "jump points" - nodes that
//! have forced neighbors or are the goal.
//!
//! Reference: Harabor, D., & Grastien, A. (2011). Online Graph Pruning for
//! Pathfinding on Grid Maps.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use crate::grid::{GridMap, Node};
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Configuration for JPS planner
#[derive(Debug, Clone)]
pub struct JPSConfig {
    /// Grid resolution in meters
    pub resolution: f64,
    /// Robot radius for obstacle inflation
    pub robot_radius: f64,
    /// Heuristic weight (1.0 = optimal, >1.0 = faster but suboptimal)
    pub heuristic_weight: f64,
}

impl Default for JPSConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        }
    }
}

impl JPSConfig {
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

/// Direction for movement in JPS
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct Direction {
    dx: i32,
    dy: i32,
}

impl Direction {
    fn new(dx: i32, dy: i32) -> Self {
        Self { dx, dy }
    }

    fn is_diagonal(&self) -> bool {
        self.dx != 0 && self.dy != 0
    }

    fn is_cardinal(&self) -> bool {
        !self.is_diagonal()
    }
}

/// Node with priority for JPS open set (min-heap)
#[derive(Debug)]
struct PriorityNode {
    x: i32,
    y: i32,
    cost: f64,
    priority: f64,
    index: usize,
}

impl Eq for PriorityNode {}

impl PartialEq for PriorityNode {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority
    }
}

impl Ord for PriorityNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap behavior
        other
            .priority
            .partial_cmp(&self.priority)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for PriorityNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Jump Point Search path planner
pub struct JPSPlanner {
    grid_map: GridMap,
    config: JPSConfig,
}

impl JPSPlanner {
    /// Create a new JPS planner with obstacle positions
    pub fn new(ox: &[f64], oy: &[f64], config: JPSConfig) -> Self {
        Self::try_new(ox, oy, config).expect(
            "invalid JPS planner input: obstacle list must be non-empty and valid, and config values must be positive/finite",
        )
    }

    /// Create a validated JPS planner with obstacle positions
    pub fn try_new(ox: &[f64], oy: &[f64], config: JPSConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.resolution, config.robot_radius)?;
        Ok(JPSPlanner { grid_map, config })
    }

    /// Create from obstacle x/y vectors with default config
    pub fn from_obstacles(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let config = JPSConfig {
            resolution,
            robot_radius,
            ..Default::default()
        };
        Self::new(ox, oy, config)
    }

    /// Create a validated JPS planner from typed obstacle points
    pub fn from_obstacle_points(obstacles: &Obstacles, config: JPSConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::from_obstacles(obstacles, config.resolution, config.robot_radius)?;
        Ok(JPSPlanner { grid_map, config })
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
        // Octile distance heuristic for 8-connected grid
        let dx = (n1_x - n2_x).abs() as f64;
        let dy = (n1_y - n2_y).abs() as f64;
        let d_min = dx.min(dy);
        let d_max = dx.max(dy);
        self.config.heuristic_weight * (d_min * std::f64::consts::SQRT_2 + (d_max - d_min))
    }

    /// Check if a position has forced neighbors in the given direction
    fn has_forced_neighbor(&self, x: i32, y: i32, dir: Direction) -> bool {
        if dir.is_cardinal() {
            // For cardinal directions, check perpendicular neighbors
            if dir.dx != 0 {
                // Moving horizontally
                let blocked_up = !self.grid_map.is_valid(x - dir.dx, y + 1);
                let open_up = self.grid_map.is_valid(x, y + 1);
                let blocked_down = !self.grid_map.is_valid(x - dir.dx, y - 1);
                let open_down = self.grid_map.is_valid(x, y - 1);
                (blocked_up && open_up) || (blocked_down && open_down)
            } else {
                // Moving vertically
                let blocked_right = !self.grid_map.is_valid(x + 1, y - dir.dy);
                let open_right = self.grid_map.is_valid(x + 1, y);
                let blocked_left = !self.grid_map.is_valid(x - 1, y - dir.dy);
                let open_left = self.grid_map.is_valid(x - 1, y);
                (blocked_right && open_right) || (blocked_left && open_left)
            }
        } else {
            // For diagonal directions, check for forced neighbors
            let blocked_h = !self.grid_map.is_valid(x - dir.dx, y);
            let open_h_diag = self.grid_map.is_valid(x - dir.dx, y + dir.dy);
            let blocked_v = !self.grid_map.is_valid(x, y - dir.dy);
            let open_v_diag = self.grid_map.is_valid(x + dir.dx, y - dir.dy);
            (blocked_h && open_h_diag) || (blocked_v && open_v_diag)
        }
    }

    /// Jump in a given direction from (x, y) until we find a jump point or hit an obstacle
    fn jump(&self, x: i32, y: i32, dir: Direction, goal_x: i32, goal_y: i32) -> Option<(i32, i32)> {
        let nx = x + dir.dx;
        let ny = y + dir.dy;

        if !self.grid_map.is_valid(nx, ny) {
            return None;
        }

        if dir.is_diagonal() {
            if !self.grid_map.is_valid(x + dir.dx, y) && !self.grid_map.is_valid(x, y + dir.dy) {
                return None;
            }
        }

        if nx == goal_x && ny == goal_y {
            return Some((nx, ny));
        }

        if self.has_forced_neighbor(nx, ny, dir) {
            return Some((nx, ny));
        }

        if dir.is_diagonal() {
            if self
                .jump(nx, ny, Direction::new(dir.dx, 0), goal_x, goal_y)
                .is_some()
            {
                return Some((nx, ny));
            }
            if self
                .jump(nx, ny, Direction::new(0, dir.dy), goal_x, goal_y)
                .is_some()
            {
                return Some((nx, ny));
            }
        }

        self.jump(nx, ny, dir, goal_x, goal_y)
    }

    fn get_successors(
        &self,
        x: i32,
        y: i32,
        parent_dir: Option<Direction>,
        goal_x: i32,
        goal_y: i32,
    ) -> Vec<(i32, i32, Direction)> {
        let mut successors = Vec::new();
        let directions = self.get_neighbors(x, y, parent_dir);

        for dir in directions {
            if let Some((jx, jy)) = self.jump(x, y, dir, goal_x, goal_y) {
                successors.push((jx, jy, dir));
            }
        }

        successors
    }

    fn get_neighbors(&self, x: i32, y: i32, parent_dir: Option<Direction>) -> Vec<Direction> {
        match parent_dir {
            None => {
                let mut dirs = Vec::new();
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        let dir = Direction::new(dx, dy);
                        if self.grid_map.is_valid(x + dx, y + dy) {
                            if dir.is_diagonal() {
                                if self.grid_map.is_valid(x + dx, y)
                                    || self.grid_map.is_valid(x, y + dy)
                                {
                                    dirs.push(dir);
                                }
                            } else {
                                dirs.push(dir);
                            }
                        }
                    }
                }
                dirs
            }
            Some(dir) => {
                let mut dirs = Vec::new();

                if dir.is_diagonal() {
                    if self.grid_map.is_valid(x + dir.dx, y + dir.dy) {
                        if self.grid_map.is_valid(x + dir.dx, y)
                            || self.grid_map.is_valid(x, y + dir.dy)
                        {
                            dirs.push(dir);
                        }
                    }
                    if self.grid_map.is_valid(x + dir.dx, y) {
                        dirs.push(Direction::new(dir.dx, 0));
                    }
                    if self.grid_map.is_valid(x, y + dir.dy) {
                        dirs.push(Direction::new(0, dir.dy));
                    }

                    if !self.grid_map.is_valid(x - dir.dx, y) {
                        if self.grid_map.is_valid(x - dir.dx, y + dir.dy) {
                            dirs.push(Direction::new(-dir.dx, dir.dy));
                        }
                    }
                    if !self.grid_map.is_valid(x, y - dir.dy) {
                        if self.grid_map.is_valid(x + dir.dx, y - dir.dy) {
                            dirs.push(Direction::new(dir.dx, -dir.dy));
                        }
                    }
                } else if dir.dx != 0 {
                    if self.grid_map.is_valid(x + dir.dx, y) {
                        dirs.push(dir);
                    }
                    if !self.grid_map.is_valid(x - dir.dx, y + 1) {
                        if self.grid_map.is_valid(x, y + 1)
                            && self.grid_map.is_valid(x + dir.dx, y + 1)
                        {
                            dirs.push(Direction::new(dir.dx, 1));
                        }
                    }
                    if !self.grid_map.is_valid(x - dir.dx, y - 1) {
                        if self.grid_map.is_valid(x, y - 1)
                            && self.grid_map.is_valid(x + dir.dx, y - 1)
                        {
                            dirs.push(Direction::new(dir.dx, -1));
                        }
                    }
                } else {
                    if self.grid_map.is_valid(x, y + dir.dy) {
                        dirs.push(dir);
                    }
                    if !self.grid_map.is_valid(x + 1, y - dir.dy) {
                        if self.grid_map.is_valid(x + 1, y)
                            && self.grid_map.is_valid(x + 1, y + dir.dy)
                        {
                            dirs.push(Direction::new(1, dir.dy));
                        }
                    }
                    if !self.grid_map.is_valid(x - 1, y - dir.dy) {
                        if self.grid_map.is_valid(x - 1, y)
                            && self.grid_map.is_valid(x - 1, y + dir.dy)
                        {
                            dirs.push(Direction::new(-1, dir.dy));
                        }
                    }
                }

                dirs
            }
        }
    }

    fn calc_distance(&self, x1: i32, y1: i32, x2: i32, y2: i32) -> f64 {
        let dx = (x2 - x1).abs() as f64;
        let dy = (y2 - y1).abs() as f64;
        let d_min = dx.min(dy);
        let d_max = dx.max(dy);
        d_min * std::f64::consts::SQRT_2 + (d_max - d_min)
    }

    fn build_path(&self, goal_index: usize, node_storage: &[Node]) -> Path2D {
        let mut jump_points = Vec::new();
        let mut current_index = Some(goal_index);

        while let Some(index) = current_index {
            let node = &node_storage[index];
            jump_points.push((node.x, node.y));
            current_index = node.parent_index;
        }

        jump_points.reverse();

        let mut points = Vec::new();

        for i in 0..jump_points.len() {
            let (x, y) = jump_points[i];

            if i == 0 {
                points.push(Point2D::new(
                    self.grid_map.calc_x_position(x),
                    self.grid_map.calc_y_position(y),
                ));
            } else {
                let (px, py) = jump_points[i - 1];
                let dx = (x - px).signum();
                let dy = (y - py).signum();

                let mut cx = px;
                let mut cy = py;

                while cx != x || cy != y {
                    if cx != x && cy != y {
                        cx += dx;
                        cy += dy;
                    } else if cx != x {
                        cx += dx;
                    } else {
                        cy += dy;
                    }

                    points.push(Point2D::new(
                        self.grid_map.calc_x_position(cx),
                        self.grid_map.calc_y_position(cy),
                    ));
                }
            }
        }

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

        let mut open_set = BinaryHeap::new();
        let mut closed_set: HashMap<i32, usize> = HashMap::new();
        let mut node_storage: Vec<Node> = Vec::new();
        let mut direction_map: HashMap<usize, Option<Direction>> = HashMap::new();

        node_storage.push(Node::new(start_x, start_y, 0.0, None));
        let start_index = 0;
        direction_map.insert(start_index, None);

        open_set.push(PriorityNode {
            x: start_x,
            y: start_y,
            cost: 0.0,
            priority: self.calc_heuristic(start_x, start_y, goal_x, goal_y),
            index: start_index,
        });

        while let Some(current) = open_set.pop() {
            let current_grid_index = self.grid_map.calc_index(current.x, current.y);

            if current.x == goal_x && current.y == goal_y {
                return Ok(self.build_path(current.index, &node_storage));
            }

            if let Some(&existing_index) = closed_set.get(&current_grid_index) {
                if node_storage[existing_index].cost <= current.cost {
                    continue;
                }
            }

            closed_set.insert(current_grid_index, current.index);

            let parent_dir = direction_map.get(&current.index).copied().flatten();
            let successors = self.get_successors(current.x, current.y, parent_dir, goal_x, goal_y);

            for (jx, jy, dir) in successors {
                let new_cost = current.cost + self.calc_distance(current.x, current.y, jx, jy);
                let new_grid_index = self.grid_map.calc_index(jx, jy);

                if let Some(&existing_index) = closed_set.get(&new_grid_index) {
                    if node_storage[existing_index].cost <= new_cost {
                        continue;
                    }
                }

                node_storage.push(Node::new(jx, jy, new_cost, Some(current.index)));
                let new_index = node_storage.len() - 1;
                direction_map.insert(new_index, Some(dir));

                let priority = new_cost + self.calc_heuristic(jx, jy, goal_x, goal_y);
                open_set.push(PriorityNode {
                    x: jx,
                    y: jy,
                    cost: new_cost,
                    priority,
                    index: new_index,
                });
            }
        }

        Err(RoboticsError::PlanningError("No path found".to_string()))
    }
}

impl PathPlanner for JPSPlanner {
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

        for i in 0..61 {
            ox.push(i as f64);
            oy.push(0.0);
            ox.push(i as f64);
            oy.push(60.0);
            ox.push(0.0);
            oy.push(i as f64);
            ox.push(60.0);
            oy.push(i as f64);
        }

        for i in 20..40 {
            ox.push(30.0);
            oy.push(i as f64);
        }

        (ox, oy)
    }

    fn create_small_obstacles() -> (Vec<f64>, Vec<f64>) {
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
    fn test_jps_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = JPSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(10.0, 10.0);
        let goal = Point2D::new(50.0, 50.0);

        let result = planner.plan(start, goal);
        assert!(result.is_ok());

        let path = result.unwrap();
        assert!(!path.is_empty());

        let first = &path.points[0];
        let last = &path.points[path.len() - 1];
        assert!((first.x - 10.0).abs() < 2.0);
        assert!((first.y - 10.0).abs() < 2.0);
        assert!((last.x - 50.0).abs() < 2.0);
        assert!((last.y - 50.0).abs() < 2.0);
    }

    #[test]
    fn test_jps_small_map() {
        let (ox, oy) = create_small_obstacles();
        let planner = JPSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(8.0, 8.0);

        let result = planner.plan(start, goal);
        assert!(result.is_ok());

        let path = result.unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_jps_legacy_interface() {
        let (ox, oy) = create_small_obstacles();
        let planner = JPSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let result = planner.planning(2.0, 2.0, 8.0, 8.0);
        assert!(result.is_some());

        let (rx, ry) = result.unwrap();
        assert!(!rx.is_empty());
        assert_eq!(rx.len(), ry.len());
    }

    #[test]
    fn test_jps_no_path() {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        for i in 0..10 {
            ox.push(i as f64);
            oy.push(0.0);
            ox.push(i as f64);
            oy.push(9.0);
            ox.push(0.0);
            oy.push(i as f64);
            ox.push(9.0);
            oy.push(i as f64);
        }

        for i in 1..9 {
            ox.push(5.0);
            oy.push(i as f64);
        }

        let planner = JPSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(2.0, 5.0);
        let goal = Point2D::new(7.0, 5.0);

        let result = planner.plan(start, goal);
        assert!(result.is_err());
    }

    #[test]
    fn test_jps_diagonal_path() {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        for i in 0..21 {
            ox.push(i as f64);
            oy.push(0.0);
            ox.push(i as f64);
            oy.push(20.0);
            ox.push(0.0);
            oy.push(i as f64);
            ox.push(20.0);
            oy.push(i as f64);
        }

        let planner = JPSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(18.0, 18.0);

        let result = planner.plan(start, goal);
        assert!(result.is_ok());

        let path = result.unwrap();
        assert!(path.len() >= 2, "Path should have at least start and goal");
        let total_len = path.total_length();
        assert!(
            total_len < 30.0,
            "Path should be efficient, got length {}",
            total_len
        );
    }

    #[test]
    fn test_jps_from_obstacle_points() {
        let (ox, oy) = create_small_obstacles();
        let obstacles = Obstacles::try_from_xy(&ox, &oy).unwrap();
        let planner = JPSPlanner::from_obstacle_points(&obstacles, JPSConfig::default()).unwrap();

        let path = planner.plan_xy(2.0, 2.0, 8.0, 8.0).unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_jps_try_new_rejects_invalid_config() {
        let (ox, oy) = create_small_obstacles();
        let config = JPSConfig {
            heuristic_weight: 0.0,
            ..Default::default()
        };

        let err = match JPSPlanner::try_new(&ox, &oy, config) {
            Ok(_) => panic!("expected invalid config to be rejected"),
            Err(err) => err,
        };
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_jps_preserves_asymmetric_world_coordinates() {
        let mut obstacles = Obstacles::new();

        for x in 10..=20 {
            obstacles.push(Point2D::new(x as f64, -4.0));
            obstacles.push(Point2D::new(x as f64, 6.0));
        }
        for y in -4..=6 {
            obstacles.push(Point2D::new(10.0, y as f64));
            obstacles.push(Point2D::new(20.0, y as f64));
        }

        let planner = JPSPlanner::from_obstacle_points(&obstacles, JPSConfig::default()).unwrap();
        let path = planner.plan_xy(12.0, -2.0, 18.0, 4.0).unwrap();

        let first = path.points.first().unwrap();
        let last = path.points.last().unwrap();
        assert!((first.x - 12.0).abs() < 1e-6);
        assert!((first.y + 2.0).abs() < 1e-6);
        assert!((last.x - 18.0).abs() < 1e-6);
        assert!((last.y - 4.0).abs() < 1e-6);
    }
}
