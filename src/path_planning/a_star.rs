//! A* path planning algorithm
//!
//! Grid-based path planning using A* search algorithm with
//! configurable heuristic weight.

use std::collections::{HashMap, BinaryHeap};
use std::cmp::Ordering;

use crate::common::{Point2D, Path2D, PathPlanner, RoboticsError};
use crate::utils::{GridMap, Node};

/// Configuration for A* planner
#[derive(Debug, Clone)]
pub struct AStarConfig {
    /// Grid resolution in meters
    pub resolution: f64,
    /// Robot radius for obstacle inflation
    pub robot_radius: f64,
    /// Heuristic weight (1.0 = optimal, >1.0 = faster but suboptimal)
    pub heuristic_weight: f64,
}

impl Default for AStarConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        }
    }
}

/// Node with priority for A* open set (min-heap)
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
        other.priority.partial_cmp(&self.priority).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for PriorityNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// A* path planner
pub struct AStarPlanner {
    grid_map: GridMap,
    config: AStarConfig,
    motion: Vec<(i32, i32, f64)>,
}

impl AStarPlanner {
    /// Create a new A* planner with obstacle positions
    pub fn new(ox: &[f64], oy: &[f64], config: AStarConfig) -> Self {
        let grid_map = GridMap::new(ox, oy, config.resolution, config.robot_radius);
        let motion = Self::get_motion_model();

        AStarPlanner { grid_map, config, motion }
    }

    /// Create from obstacle x/y vectors with default config
    pub fn from_obstacles(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let config = AStarConfig {
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
        self.config.heuristic_weight
            * (((n1_x - n2_x).pow(2) + (n1_y - n2_y).pow(2)) as f64).sqrt()
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
                self.grid_map.calc_grid_position(node.x),
                self.grid_map.calc_grid_position(node.y),
            ));
            current_index = node.parent_index;
        }

        points.reverse();
        Path2D::from_points(points)
    }
}

impl PathPlanner for AStarPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        let start_x = self.grid_map.calc_xy_index(start.x);
        let start_y = self.grid_map.calc_xy_index(start.y);
        let goal_x = self.grid_map.calc_xy_index(goal.x);
        let goal_y = self.grid_map.calc_xy_index(goal.y);

        let mut open_set = BinaryHeap::new();
        let mut closed_set = HashMap::new();
        let mut node_storage: Vec<Node> = Vec::new();

        // Add start node
        node_storage.push(Node::new(start_x, start_y, 0.0, None));
        let start_index = 0;

        open_set.push(PriorityNode {
            x: start_x,
            y: start_y,
            cost: 0.0,
            priority: self.calc_heuristic(start_x, start_y, goal_x, goal_y),
            index: start_index,
        });

        while let Some(current) = open_set.pop() {
            let current_grid_index = self.grid_map.calc_index(current.x, current.y);

            // Check if we reached the goal
            if current.x == goal_x && current.y == goal_y {
                return Ok(self.build_path(current.index, &node_storage));
            }

            // Skip if already in closed set
            if closed_set.contains_key(&current_grid_index) {
                continue;
            }

            // Move to closed set
            closed_set.insert(current_grid_index, current.index);

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

                // Add to storage and open set
                node_storage.push(Node::new(new_x, new_y, new_cost, Some(current.index)));
                let new_index = node_storage.len() - 1;

                let priority = new_cost + self.calc_heuristic(new_x, new_y, goal_x, goal_y);
                open_set.push(PriorityNode {
                    x: new_x,
                    y: new_y,
                    cost: new_cost,
                    priority,
                    index: new_index,
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
        for i in 0..11 {
            ox.push(i as f64); oy.push(0.0);
            ox.push(i as f64); oy.push(10.0);
            ox.push(0.0); oy.push(i as f64);
            ox.push(10.0); oy.push(i as f64);
        }

        // Internal obstacle
        for i in 4..7 {
            ox.push(5.0);
            oy.push(i as f64);
        }

        (ox, oy)
    }

    #[test]
    fn test_a_star_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = AStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(8.0, 8.0);

        let result = planner.plan(start, goal);
        assert!(result.is_ok());

        let path = result.unwrap();
        assert!(path.len() > 0);
    }

    #[test]
    fn test_a_star_legacy_interface() {
        let (ox, oy) = create_simple_obstacles();
        let planner = AStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let result = planner.planning(2.0, 2.0, 8.0, 8.0);
        assert!(result.is_some());

        let (rx, ry) = result.unwrap();
        assert!(rx.len() > 0);
        assert_eq!(rx.len(), ry.len());
    }
}
