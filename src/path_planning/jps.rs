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

use crate::common::{Path2D, PathPlanner, Point2D, RoboticsError};
use crate::utils::{GridMap, Node};

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
    direction: Option<Direction>,
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
        let grid_map = GridMap::new(ox, oy, config.resolution, config.robot_radius);
        JPSPlanner { grid_map, config }
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

    /// Plan a path returning (rx, ry) vectors (legacy interface)
    pub fn planning(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> Option<(Vec<f64>, Vec<f64>)> {
        let start = Point2D::new(sx, sy);
        let goal = Point2D::new(gx, gy);

        match self.plan(start, goal) {
            Ok(path) => Some((path.x_coords(), path.y_coords())),
            Err(_) => None,
        }
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
            // Forced neighbors occur when there's an obstacle adjacent
            let blocked_h = !self.grid_map.is_valid(x - dir.dx, y);
            let open_h_diag = self.grid_map.is_valid(x - dir.dx, y + dir.dy);
            let blocked_v = !self.grid_map.is_valid(x, y - dir.dy);
            let open_v_diag = self.grid_map.is_valid(x + dir.dx, y - dir.dy);
            (blocked_h && open_h_diag) || (blocked_v && open_v_diag)
        }
    }

    /// Jump in a given direction from (x, y) until we find a jump point or hit an obstacle
    fn jump(
        &self,
        x: i32,
        y: i32,
        dir: Direction,
        goal_x: i32,
        goal_y: i32,
    ) -> Option<(i32, i32)> {
        let nx = x + dir.dx;
        let ny = y + dir.dy;

        // Check if next position is valid
        if !self.grid_map.is_valid(nx, ny) {
            return None;
        }

        // For diagonal moves, ensure we can actually make the move
        // (check that we're not cutting corners)
        if dir.is_diagonal() {
            // Check that both cardinal directions are passable
            if !self.grid_map.is_valid(x + dir.dx, y) && !self.grid_map.is_valid(x, y + dir.dy) {
                return None;
            }
        }

        // Check if we reached the goal
        if nx == goal_x && ny == goal_y {
            return Some((nx, ny));
        }

        // Check for forced neighbors
        if self.has_forced_neighbor(nx, ny, dir) {
            return Some((nx, ny));
        }

        // For diagonal movement, recursively check horizontal and vertical
        if dir.is_diagonal() {
            // Check horizontal direction
            if self
                .jump(nx, ny, Direction::new(dir.dx, 0), goal_x, goal_y)
                .is_some()
            {
                return Some((nx, ny));
            }
            // Check vertical direction
            if self
                .jump(nx, ny, Direction::new(0, dir.dy), goal_x, goal_y)
                .is_some()
            {
                return Some((nx, ny));
            }
        }

        // Continue jumping in the same direction
        self.jump(nx, ny, dir, goal_x, goal_y)
    }

    /// Get the successors (jump points) from a given position
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

    /// Get natural neighbors based on direction of travel
    fn get_neighbors(&self, x: i32, y: i32, parent_dir: Option<Direction>) -> Vec<Direction> {
        match parent_dir {
            None => {
                // Start node: check all 8 directions
                let mut dirs = Vec::new();
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        let dir = Direction::new(dx, dy);
                        if self.grid_map.is_valid(x + dx, y + dy) {
                            // For diagonal moves, check corner cutting
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
                    // Diagonal movement: natural neighbors are diagonal, horizontal, and vertical
                    // Natural: continue diagonal
                    if self.grid_map.is_valid(x + dir.dx, y + dir.dy) {
                        if self.grid_map.is_valid(x + dir.dx, y)
                            || self.grid_map.is_valid(x, y + dir.dy)
                        {
                            dirs.push(dir);
                        }
                    }
                    // Natural: horizontal
                    if self.grid_map.is_valid(x + dir.dx, y) {
                        dirs.push(Direction::new(dir.dx, 0));
                    }
                    // Natural: vertical
                    if self.grid_map.is_valid(x, y + dir.dy) {
                        dirs.push(Direction::new(0, dir.dy));
                    }

                    // Forced neighbors for diagonal
                    // Check if there's a blocked cell adjacent causing forced neighbor
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
                } else {
                    // Cardinal movement
                    if dir.dx != 0 {
                        // Horizontal movement
                        // Natural: continue forward
                        if self.grid_map.is_valid(x + dir.dx, y) {
                            dirs.push(dir);
                        }
                        // Forced: diagonal up if blocked above behind us
                        if !self.grid_map.is_valid(x - dir.dx, y + 1) {
                            if self.grid_map.is_valid(x, y + 1)
                                && self.grid_map.is_valid(x + dir.dx, y + 1)
                            {
                                dirs.push(Direction::new(dir.dx, 1));
                            }
                        }
                        // Forced: diagonal down if blocked below behind us
                        if !self.grid_map.is_valid(x - dir.dx, y - 1) {
                            if self.grid_map.is_valid(x, y - 1)
                                && self.grid_map.is_valid(x + dir.dx, y - 1)
                            {
                                dirs.push(Direction::new(dir.dx, -1));
                            }
                        }
                    } else {
                        // Vertical movement
                        // Natural: continue forward
                        if self.grid_map.is_valid(x, y + dir.dy) {
                            dirs.push(dir);
                        }
                        // Forced: diagonal right if blocked right behind us
                        if !self.grid_map.is_valid(x + 1, y - dir.dy) {
                            if self.grid_map.is_valid(x + 1, y)
                                && self.grid_map.is_valid(x + 1, y + dir.dy)
                            {
                                dirs.push(Direction::new(1, dir.dy));
                            }
                        }
                        // Forced: diagonal left if blocked left behind us
                        if !self.grid_map.is_valid(x - 1, y - dir.dy) {
                            if self.grid_map.is_valid(x - 1, y)
                                && self.grid_map.is_valid(x - 1, y + dir.dy)
                            {
                                dirs.push(Direction::new(-1, dir.dy));
                            }
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
        // Octile distance for actual cost
        let d_min = dx.min(dy);
        let d_max = dx.max(dy);
        d_min * std::f64::consts::SQRT_2 + (d_max - d_min)
    }

    fn build_path(&self, goal_index: usize, node_storage: &[Node]) -> Path2D {
        // First, collect jump points in reverse order
        let mut jump_points = Vec::new();
        let mut current_index = Some(goal_index);

        while let Some(index) = current_index {
            let node = &node_storage[index];
            jump_points.push((node.x, node.y));
            current_index = node.parent_index;
        }

        jump_points.reverse();

        // Now interpolate between consecutive jump points
        let mut points = Vec::new();

        for i in 0..jump_points.len() {
            let (x, y) = jump_points[i];

            if i == 0 {
                // Add start point
                points.push(Point2D::new(
                    self.grid_map.calc_grid_position(x),
                    self.grid_map.calc_grid_position(y),
                ));
            } else {
                // Interpolate from previous jump point to current
                let (px, py) = jump_points[i - 1];
                let dx = (x - px).signum();
                let dy = (y - py).signum();

                let mut cx = px;
                let mut cy = py;

                // Walk from previous to current, adding intermediate points
                while cx != x || cy != y {
                    // Move one step
                    if cx != x && cy != y {
                        // Diagonal movement
                        cx += dx;
                        cy += dy;
                    } else if cx != x {
                        // Horizontal movement
                        cx += dx;
                    } else {
                        // Vertical movement
                        cy += dy;
                    }

                    points.push(Point2D::new(
                        self.grid_map.calc_grid_position(cx),
                        self.grid_map.calc_grid_position(cy),
                    ));
                }
            }
        }

        Path2D::from_points(points)
    }
}

impl PathPlanner for JPSPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        let start_x = self.grid_map.calc_xy_index(start.x);
        let start_y = self.grid_map.calc_xy_index(start.y);
        let goal_x = self.grid_map.calc_xy_index(goal.x);
        let goal_y = self.grid_map.calc_xy_index(goal.y);

        // Validate start and goal
        if !self.grid_map.is_valid(start_x, start_y) {
            return Err(RoboticsError::PlanningError(
                "Start position is invalid".to_string(),
            ));
        }
        if !self.grid_map.is_valid(goal_x, goal_y) {
            return Err(RoboticsError::PlanningError(
                "Goal position is invalid".to_string(),
            ));
        }

        let mut open_set = BinaryHeap::new();
        let mut closed_set: HashMap<i32, usize> = HashMap::new();
        let mut node_storage: Vec<Node> = Vec::new();
        let mut direction_map: HashMap<usize, Option<Direction>> = HashMap::new();

        // Add start node
        node_storage.push(Node::new(start_x, start_y, 0.0, None));
        let start_index = 0;
        direction_map.insert(start_index, None);

        open_set.push(PriorityNode {
            x: start_x,
            y: start_y,
            cost: 0.0,
            priority: self.calc_heuristic(start_x, start_y, goal_x, goal_y),
            index: start_index,
            direction: None,
        });

        while let Some(current) = open_set.pop() {
            let current_grid_index = self.grid_map.calc_index(current.x, current.y);

            // Check if we reached the goal
            if current.x == goal_x && current.y == goal_y {
                return Ok(self.build_path(current.index, &node_storage));
            }

            // Skip if already in closed set with lower cost
            if let Some(&existing_index) = closed_set.get(&current_grid_index) {
                if node_storage[existing_index].cost <= current.cost {
                    continue;
                }
            }

            // Move to closed set
            closed_set.insert(current_grid_index, current.index);

            // Get successors using JPS
            let parent_dir = direction_map.get(&current.index).copied().flatten();
            let successors = self.get_successors(current.x, current.y, parent_dir, goal_x, goal_y);

            for (jx, jy, dir) in successors {
                let new_cost = current.cost + self.calc_distance(current.x, current.y, jx, jy);
                let new_grid_index = self.grid_map.calc_index(jx, jy);

                // Skip if already visited with lower cost
                if let Some(&existing_index) = closed_set.get(&new_grid_index) {
                    if node_storage[existing_index].cost <= new_cost {
                        continue;
                    }
                }

                // Add to storage and open set
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
                    direction: Some(dir),
                });
            }
        }

        Err(RoboticsError::PlanningError("No path found".to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_simple_obstacles() -> (Vec<f64>, Vec<f64>) {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        // Boundary
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

        // Internal obstacle (vertical wall)
        for i in 20..40 {
            ox.push(30.0);
            oy.push(i as f64);
        }

        (ox, oy)
    }

    fn create_small_obstacles() -> (Vec<f64>, Vec<f64>) {
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
    fn test_jps_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = JPSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(10.0, 10.0);
        let goal = Point2D::new(50.0, 50.0);

        let result = planner.plan(start, goal);
        assert!(result.is_ok());

        let path = result.unwrap();
        assert!(path.len() > 0);

        // Check that path starts near start and ends near goal
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
        assert!(path.len() > 0);
    }

    #[test]
    fn test_jps_legacy_interface() {
        let (ox, oy) = create_small_obstacles();
        let planner = JPSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let result = planner.planning(2.0, 2.0, 8.0, 8.0);
        assert!(result.is_some());

        let (rx, ry) = result.unwrap();
        assert!(rx.len() > 0);
        assert_eq!(rx.len(), ry.len());
    }

    #[test]
    fn test_jps_no_path() {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        // Create a box that completely surrounds the start
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

        // Wall separating start from goal
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

        // Just boundary, no internal obstacles
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
        // For a diagonal path with no obstacles, JPS should find an efficient path
        // The path is interpolated, so it will have all intermediate grid points
        // From (2,2) to (18,18) is 16 diagonal steps, so 17 points total
        assert!(path.len() >= 2, "Path should have at least start and goal");
        // Verify path is roughly diagonal (total length should be close to sqrt(2) * 16 ≈ 22.6)
        let total_len = path.total_length();
        assert!(total_len < 30.0, "Path should be efficient, got length {}", total_len);
    }
}
