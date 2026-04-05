//! Anya-inspired optimal any-angle pathfinding
//!
//! This module implements optimal any-angle pathfinding using a visibility
//! graph approach on grid corner points. Instead of expanding intervals
//! (as in the original Anya algorithm), we build a visibility graph over
//! obstacle corner points and run Dijkstra's algorithm to find the
//! shortest euclidean path.
//!
//! The result is the same optimal any-angle path that Anya would produce,
//! since both methods find the shortest path through free space whose
//! waypoints lie on obstacle corners.
//!
//! Reference: Harabor, Grastien, Oz, Aksakalli (2016)
//! "Optimal Any-Angle Pathfinding In Practice", Journal of AI Research.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::grid::GridMap;
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Configuration for Anya planner
#[derive(Debug, Clone)]
pub struct AnyaConfig {
    pub resolution: f64,
    pub robot_radius: f64,
    pub heuristic_weight: f64,
}

impl Default for AnyaConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        }
    }
}

impl AnyaConfig {
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

#[derive(Debug)]
struct DijkstraNode {
    node_id: usize,
    cost: f64,
}

impl Eq for DijkstraNode {}
impl PartialEq for DijkstraNode {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}
impl Ord for DijkstraNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for DijkstraNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub struct AnyaPlanner {
    grid_map: GridMap,
    #[allow(dead_code)]
    config: AnyaConfig,
}

impl AnyaPlanner {
    pub fn new(ox: &[f64], oy: &[f64], config: AnyaConfig) -> Self {
        Self::try_new(ox, oy, config).expect(
            "invalid Anya planner input: obstacle list must be non-empty and valid, and config values must be positive/finite",
        )
    }

    pub fn try_new(ox: &[f64], oy: &[f64], config: AnyaConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.resolution, config.robot_radius)?;
        Ok(AnyaPlanner { grid_map, config })
    }

    pub fn from_obstacles(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let config = AnyaConfig {
            resolution,
            robot_radius,
            ..Default::default()
        };
        Self::new(ox, oy, config)
    }

    pub fn from_obstacle_points(
        obstacles: &Obstacles,
        config: AnyaConfig,
    ) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::from_obstacles(obstacles, config.resolution, config.robot_radius)?;
        Ok(AnyaPlanner { grid_map, config })
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

    /// Line-of-sight check using DDA ray marching (same algorithm as theta_star).
    fn line_of_sight(&self, x0: i32, y0: i32, x1: i32, y1: i32) -> bool {
        if !self.grid_map.is_valid(x0, y0) || !self.grid_map.is_valid(x1, y1) {
            return false;
        }

        if x0 == x1 && y0 == y1 {
            return true;
        }

        let dx = x1 - x0;
        let dy = y1 - y0;
        let step_x = dx.signum();
        let step_y = dy.signum();
        let abs_dx = dx.abs() as f64;
        let abs_dy = dy.abs() as f64;

        let mut x = x0;
        let mut y = y0;
        let mut t_max_x = if step_x != 0 {
            0.5 / abs_dx
        } else {
            f64::INFINITY
        };
        let mut t_max_y = if step_y != 0 {
            0.5 / abs_dy
        } else {
            f64::INFINITY
        };
        let t_delta_x = if step_x != 0 {
            1.0 / abs_dx
        } else {
            f64::INFINITY
        };
        let t_delta_y = if step_y != 0 {
            1.0 / abs_dy
        } else {
            f64::INFINITY
        };

        while x != x1 || y != y1 {
            let advance_x = t_max_x <= t_max_y;
            let advance_y = t_max_y <= t_max_x;
            let next_x = if advance_x { x + step_x } else { x };
            let next_y = if advance_y { y + step_y } else { y };

            if !self.grid_map.is_valid_step(x, y, next_x, next_y) {
                return false;
            }

            x = next_x;
            y = next_y;

            if advance_x {
                t_max_x += t_delta_x;
            }
            if advance_y {
                t_max_y += t_delta_y;
            }
        }

        true
    }

    /// Find all grid corner points adjacent to at least one obstacle and
    /// at least one free cell. These are the candidate waypoints for the
    /// optimal any-angle path.
    /// Collect ALL free cells as visibility-graph nodes.
    ///
    /// For a grid with no-corner-cutting semantics, the optimal any-angle
    /// path may turn at any free cell, not just obstacle-adjacent corners.
    /// Using all free cells guarantees the visibility graph produces the
    /// true optimal any-angle path (at the cost of O(V^2) LOS checks).
    fn find_all_free_cells(&self) -> Vec<(i32, i32)> {
        let mut cells = Vec::new();
        for ix in 0..self.grid_map.x_width {
            for iy in 0..self.grid_map.y_width {
                if self.grid_map.is_valid(ix, iy) {
                    cells.push((ix, iy));
                }
            }
        }
        cells
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

        // Direct line of sight => straight line path
        if self.line_of_sight(start_x, start_y, goal_x, goal_y) {
            return Ok(Path2D::from_points(vec![
                Point2D::new(
                    self.grid_map.calc_x_position(start_x),
                    self.grid_map.calc_y_position(start_y),
                ),
                Point2D::new(
                    self.grid_map.calc_x_position(goal_x),
                    self.grid_map.calc_y_position(goal_y),
                ),
            ]));
        }

        // Build visibility graph nodes: ALL free cells (guarantees optimality)
        let mut nodes: Vec<(i32, i32)> = self.find_all_free_cells();

        // Add start and goal if not already present
        let start_pos = (start_x, start_y);
        let goal_pos = (goal_x, goal_y);
        if !nodes.contains(&start_pos) {
            nodes.push(start_pos);
        }
        if !nodes.contains(&goal_pos) {
            nodes.push(goal_pos);
        }

        let n = nodes.len();
        let start_id = nodes.iter().position(|&p| p == start_pos).unwrap();
        let goal_id = nodes.iter().position(|&p| p == goal_pos).unwrap();

        // Build adjacency list: for each pair, check LOS and store distance
        let mut adj: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n];
        for i in 0..n {
            for j in (i + 1)..n {
                let (x0, y0) = nodes[i];
                let (x1, y1) = nodes[j];
                if self.line_of_sight(x0, y0, x1, y1) {
                    let dist =
                        (((x0 - x1).pow(2) + (y0 - y1).pow(2)) as f64).sqrt();
                    adj[i].push((j, dist));
                    adj[j].push((i, dist));
                }
            }
        }

        // Dijkstra
        let mut dist = vec![f64::INFINITY; n];
        let mut prev: Vec<Option<usize>> = vec![None; n];
        let mut visited = vec![false; n];
        dist[start_id] = 0.0;

        let mut heap = BinaryHeap::new();
        heap.push(DijkstraNode {
            node_id: start_id,
            cost: 0.0,
        });

        while let Some(current) = heap.pop() {
            if current.node_id == goal_id {
                break;
            }
            if visited[current.node_id] {
                continue;
            }
            visited[current.node_id] = true;

            for &(neighbor, weight) in &adj[current.node_id] {
                let new_dist = dist[current.node_id] + weight;
                if new_dist < dist[neighbor] {
                    dist[neighbor] = new_dist;
                    prev[neighbor] = Some(current.node_id);
                    heap.push(DijkstraNode {
                        node_id: neighbor,
                        cost: new_dist,
                    });
                }
            }
        }

        if dist[goal_id].is_infinite() {
            return Err(RoboticsError::PlanningError("No path found".to_string()));
        }

        // Reconstruct path
        let mut path_indices = Vec::new();
        let mut current = goal_id;
        loop {
            path_indices.push(current);
            match prev[current] {
                Some(p) => current = p,
                None => break,
            }
        }
        path_indices.reverse();

        let points: Vec<Point2D> = path_indices
            .iter()
            .map(|&id| {
                let (ix, iy) = nodes[id];
                Point2D::new(
                    self.grid_map.calc_x_position(ix),
                    self.grid_map.calc_y_position(iy),
                )
            })
            .collect();

        Ok(Path2D::from_points(points))
    }
}

impl PathPlanner for AnyaPlanner {
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
        for i in 5..15 {
            ox.push(10.0);
            oy.push(i as f64);
        }
        (ox, oy)
    }

    #[test]
    fn test_anya_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = AnyaPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);
        let result = planner.plan(Point2D::new(2.0, 10.0), Point2D::new(18.0, 10.0));
        assert!(result.is_ok());
        let path = result.unwrap();
        assert!(!path.is_empty());
        // Path should start near start and end near goal
        let xs = path.x_coords();
        let ys = path.y_coords();
        assert!((xs[0] - 2.0).abs() < 1.5);
        assert!((ys[0] - 10.0).abs() < 1.5);
        assert!((*xs.last().unwrap() - 18.0).abs() < 1.5);
        assert!((*ys.last().unwrap() - 10.0).abs() < 1.5);
    }

    #[test]
    fn test_anya_shorter_than_a_star() {
        let (ox, oy) = create_simple_obstacles();
        let anya_planner = AnyaPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);
        let a_star_planner = crate::a_star::AStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);
        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(18.0, 18.0);
        let anya_path = anya_planner.plan(start, goal).unwrap();
        let a_star_path = a_star_planner.plan(start, goal).unwrap();
        let anya_length = anya_path.total_length();
        let a_star_length = a_star_path.total_length();
        assert!(
            anya_length <= a_star_length + 0.1,
            "Anya path ({}) should not be longer than A* path ({})",
            anya_length,
            a_star_length
        );
    }

    #[test]
    fn test_anya_optimal_straight_line() {
        // Open grid with only boundary obstacles, diagonal path should be
        // close to euclidean distance.
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
        let planner = AnyaPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);
        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(18.0, 18.0);
        let path = planner.plan(start, goal).unwrap();
        let path_length = path.total_length();
        let euclidean = ((18.0 - 2.0_f64).powi(2) + (18.0 - 2.0_f64).powi(2)).sqrt();
        assert!(
            (path_length - euclidean).abs() < 1.0,
            "Straight-line path length ({}) should be close to euclidean distance ({})",
            path_length,
            euclidean
        );
    }

    #[test]
    fn test_anya_from_obstacle_points() {
        let (ox, oy) = create_simple_obstacles();
        let obstacles = Obstacles::try_from_xy(&ox, &oy).unwrap();
        let planner =
            AnyaPlanner::from_obstacle_points(&obstacles, AnyaConfig::default()).unwrap();
        let path = planner.plan_xy(2.0, 10.0, 18.0, 10.0).unwrap();
        assert!(!path.is_empty());
    }
}
