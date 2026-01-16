//! Theta* path planning algorithm
//!
//! Theta* is an any-angle path planning algorithm that extends A* by
//! allowing paths to connect any two visible nodes, not just grid neighbors.
//! This produces shorter, more natural paths compared to standard A*.
//!
//! Key features:
//! - Line-of-sight checks to skip intermediate nodes
//! - Produces any-angle paths (not restricted to grid directions)
//! - Optimal or near-optimal path lengths
//!
//! Reference: Nash, A., Daniel, K., Koenig, S., & Felner, A. (2007).
//! "Theta*: Any-Angle Path Planning on Grids"

use std::collections::{HashMap, BinaryHeap};
use std::cmp::Ordering;

use crate::common::{Point2D, Path2D, PathPlanner, RoboticsError};
use crate::utils::{GridMap, Node};

/// Configuration for Theta* planner
#[derive(Debug, Clone)]
pub struct ThetaStarConfig {
    /// Grid resolution in meters
    pub resolution: f64,
    /// Robot radius for obstacle inflation
    pub robot_radius: f64,
    /// Heuristic weight (1.0 = optimal, >1.0 = faster but suboptimal)
    pub heuristic_weight: f64,
}

impl Default for ThetaStarConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        }
    }
}

/// Node with priority for Theta* open set (min-heap)
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

/// Theta* path planner
///
/// Theta* extends A* by checking line-of-sight between a node and its
/// grandparent. If there's a clear line-of-sight, the path skips the
/// parent node, resulting in shorter, more direct paths.
pub struct ThetaStarPlanner {
    grid_map: GridMap,
    config: ThetaStarConfig,
    motion: Vec<(i32, i32, f64)>,
}

impl ThetaStarPlanner {
    /// Create a new Theta* planner with obstacle positions
    pub fn new(ox: &[f64], oy: &[f64], config: ThetaStarConfig) -> Self {
        let grid_map = GridMap::new(ox, oy, config.resolution, config.robot_radius);
        let motion = Self::get_motion_model();

        ThetaStarPlanner { grid_map, config, motion }
    }

    /// Create from obstacle x/y vectors with default config
    pub fn from_obstacles(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let config = ThetaStarConfig {
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

    /// Check if there is line-of-sight between two grid cells
    ///
    /// Uses Bresenham-like line algorithm to check if all cells
    /// along the line are obstacle-free.
    fn line_of_sight(&self, x0: i32, y0: i32, x1: i32, y1: i32) -> bool {
        let mut x = x0;
        let mut y = y0;
        let dx = (x1 - x0).abs();
        let dy = (y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };

        let mut err = dx - dy;

        loop {
            // Check if current cell is valid (not obstacle)
            if !self.grid_map.is_valid(x, y) {
                return false;
            }

            if x == x1 && y == y1 {
                break;
            }

            let e2 = 2 * err;
            if e2 > -dy {
                err -= dy;
                x += sx;
            }
            if e2 < dx {
                err += dx;
                y += sy;
            }
        }

        true
    }

    /// Calculate Euclidean distance between two grid cells
    fn euclidean_distance(&self, x1: i32, y1: i32, x2: i32, y2: i32) -> f64 {
        (((x1 - x2).pow(2) + (y1 - y2).pow(2)) as f64).sqrt()
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
}

impl PathPlanner for ThetaStarPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        let start_x = self.grid_map.calc_x_index(start.x);
        let start_y = self.grid_map.calc_y_index(start.y);
        let goal_x = self.grid_map.calc_x_index(goal.x);
        let goal_y = self.grid_map.calc_y_index(goal.y);

        let mut open_set = BinaryHeap::new();
        let mut closed_set = HashMap::new();
        let mut node_storage: Vec<Node> = Vec::new();
        // Track g-values for each grid cell
        let mut g_values: HashMap<i32, f64> = HashMap::new();
        // Track the best node index for each grid cell
        let mut best_index: HashMap<i32, usize> = HashMap::new();

        // Add start node
        node_storage.push(Node::new(start_x, start_y, 0.0, None));
        let start_index = 0;
        let start_grid_index = self.grid_map.calc_index(start_x, start_y);
        g_values.insert(start_grid_index, 0.0);
        best_index.insert(start_grid_index, start_index);

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

            let current_node = &node_storage[current.index];
            let parent_index = current_node.parent_index;

            // Expand neighbors
            for &(dx, dy, _) in &self.motion {
                let new_x = current.x + dx;
                let new_y = current.y + dy;
                let new_grid_index = self.grid_map.calc_index(new_x, new_y);

                // Skip if not valid or already visited
                if !self.grid_map.is_valid(new_x, new_y) {
                    continue;
                }
                if closed_set.contains_key(&new_grid_index) {
                    continue;
                }

                // Theta* main logic: check line-of-sight to grandparent
                let (new_cost, new_parent_index) = if let Some(p_idx) = parent_index {
                    let parent_node = &node_storage[p_idx];

                    // Check line-of-sight from parent to neighbor
                    if self.line_of_sight(parent_node.x, parent_node.y, new_x, new_y) {
                        // Path 2: Connect directly to grandparent
                        let dist = self.euclidean_distance(parent_node.x, parent_node.y, new_x, new_y);
                        let cost = parent_node.cost + dist;
                        (cost, Some(p_idx))
                    } else {
                        // Path 1: Standard A* path through current node
                        let dist = self.euclidean_distance(current.x, current.y, new_x, new_y);
                        let cost = current.cost + dist;
                        (cost, Some(current.index))
                    }
                } else {
                    // Start node: standard A* path
                    let dist = self.euclidean_distance(current.x, current.y, new_x, new_y);
                    let cost = current.cost + dist;
                    (cost, Some(current.index))
                };

                // Check if this path is better than existing path
                let existing_g = g_values.get(&new_grid_index).copied().unwrap_or(f64::INFINITY);
                if new_cost < existing_g {
                    g_values.insert(new_grid_index, new_cost);

                    // Add to storage and open set
                    node_storage.push(Node::new(new_x, new_y, new_cost, new_parent_index));
                    let new_index = node_storage.len() - 1;
                    best_index.insert(new_grid_index, new_index);

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
        for i in 0..21 {
            ox.push(i as f64); oy.push(0.0);
            ox.push(i as f64); oy.push(20.0);
            ox.push(0.0); oy.push(i as f64);
            ox.push(20.0); oy.push(i as f64);
        }

        // Internal obstacle (vertical wall)
        for i in 5..15 {
            ox.push(10.0);
            oy.push(i as f64);
        }

        (ox, oy)
    }

    #[test]
    fn test_theta_star_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = ThetaStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(2.0, 10.0);
        let goal = Point2D::new(18.0, 10.0);

        let result = planner.plan(start, goal);
        assert!(result.is_ok());

        let path = result.unwrap();
        assert!(path.len() > 0);
    }

    #[test]
    fn test_theta_star_legacy_interface() {
        let (ox, oy) = create_simple_obstacles();
        let planner = ThetaStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let result = planner.planning(2.0, 10.0, 18.0, 10.0);
        assert!(result.is_some());

        let (rx, ry) = result.unwrap();
        assert!(rx.len() > 0);
        assert_eq!(rx.len(), ry.len());
    }

    #[test]
    fn test_theta_star_shorter_than_a_star() {
        // Theta* should produce shorter paths than A* due to any-angle paths
        let (ox, oy) = create_simple_obstacles();

        let theta_planner = ThetaStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);
        let a_star_planner = crate::path_planning::a_star::AStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(18.0, 18.0);

        let theta_path = theta_planner.plan(start, goal).unwrap();
        let a_star_path = a_star_planner.plan(start, goal).unwrap();

        // Theta* path should have fewer waypoints (due to path smoothing)
        // or shorter total length
        let theta_length = theta_path.total_length();
        let a_star_length = a_star_path.total_length();

        // Theta* should produce path length <= A* path length
        assert!(theta_length <= a_star_length + 0.1,
            "Theta* path ({}) should not be significantly longer than A* path ({})",
            theta_length, a_star_length);
    }

    #[test]
    fn test_line_of_sight() {
        let (ox, oy) = create_simple_obstacles();
        let planner = ThetaStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        // Clear line of sight
        assert!(planner.line_of_sight(2, 2, 5, 5));

        // Blocked by obstacle (wall at x=10)
        assert!(!planner.line_of_sight(5, 10, 15, 10));
    }
}
