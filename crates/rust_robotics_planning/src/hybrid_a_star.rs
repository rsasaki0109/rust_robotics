//! Hybrid A* path planning algorithm
//!
//! Plans paths for car-like robots with non-holonomic constraints
//! by combining A* grid search with continuous-space vehicle motion
//! primitives and Reeds-Shepp analytic expansion.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::f64::consts::PI;

use crate::grid::GridMap;
use crate::reeds_shepp_path;
use rust_robotics_core::{RoboticsError, RoboticsResult};

/// Configuration for Hybrid A* planner
#[derive(Debug, Clone)]
pub struct HybridAStarConfig {
    /// Grid resolution in meters
    pub xy_resolution: f64,
    /// Yaw angle resolution in radians
    pub yaw_resolution: f64,
    /// Robot wheelbase (distance between axles)
    pub wheelbase: f64,
    /// Maximum steering angle in radians
    pub max_steer: f64,
    /// Number of discrete steering inputs
    pub n_steer: usize,
    /// Distance traveled per expansion step
    pub step_size: f64,
    /// Robot radius for collision checking
    pub robot_radius: f64,
    /// Maximum curvature (1 / minimum turning radius)
    pub max_curvature: f64,
    /// Cost multiplier for switching direction
    pub switch_back_cost: f64,
    /// Cost multiplier for steering angle
    pub steer_cost: f64,
    /// Cost multiplier for changing steering
    pub steer_change_cost: f64,
    /// Heuristic weight for holonomic-with-obstacles heuristic
    pub h_cost: f64,
    /// Interval for attempting Reeds-Shepp analytic expansion
    pub analytic_expansion_interval: usize,
}

impl Default for HybridAStarConfig {
    fn default() -> Self {
        let wheelbase = 3.0;
        let max_steer: f64 = 0.6;
        let max_curvature = (max_steer).tan() / wheelbase;
        Self {
            xy_resolution: 2.0,
            yaw_resolution: PI / 36.0, // 5 degrees
            wheelbase,
            max_steer,
            n_steer: 20,
            step_size: 1.5,
            robot_radius: 1.5,
            max_curvature,
            switch_back_cost: 100.0,
            steer_cost: 1.0,
            steer_change_cost: 5.0,
            h_cost: 1.0,
            analytic_expansion_interval: 5,
        }
    }
}

impl HybridAStarConfig {
    pub fn validate(&self) -> RoboticsResult<()> {
        if !self.xy_resolution.is_finite() || self.xy_resolution <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "xy_resolution must be positive and finite, got {}",
                self.xy_resolution
            )));
        }
        if !self.yaw_resolution.is_finite() || self.yaw_resolution <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "yaw_resolution must be positive and finite, got {}",
                self.yaw_resolution
            )));
        }
        if !self.wheelbase.is_finite() || self.wheelbase <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "wheelbase must be positive and finite, got {}",
                self.wheelbase
            )));
        }
        if self.n_steer == 0 {
            return Err(RoboticsError::InvalidParameter(
                "n_steer must be at least 1".to_string(),
            ));
        }
        Ok(())
    }
}

/// A node in the Hybrid A* search tree
#[derive(Debug, Clone)]
struct HybridNode {
    /// Grid x index
    x_index: i32,
    /// Grid y index
    y_index: i32,
    /// Yaw index (discretized)
    yaw_index: i32,
    /// Direction: true = forward, false = backward
    direction: bool,
    /// Continuous x positions along this node's arc
    x_list: Vec<f64>,
    /// Continuous y positions along this node's arc
    y_list: Vec<f64>,
    /// Continuous yaw angles along this node's arc
    yaw_list: Vec<f64>,
    /// Direction list (1 = forward, -1 = backward) for each point
    directions: Vec<i32>,
    /// Steering angle used
    steer: f64,
    /// Accumulated cost
    cost: f64,
    /// Parent node index in storage
    parent_index: Option<usize>,
}

/// Priority wrapper for BinaryHeap (min-heap)
#[derive(Debug)]
struct PriorityNode {
    index: usize,
    cost: f64,
}

impl Eq for PriorityNode {}

impl PartialEq for PriorityNode {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl Ord for PriorityNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for PriorityNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Result of Hybrid A* planning
#[derive(Debug, Clone)]
pub struct HybridAStarPath {
    /// X coordinates along the path
    pub x: Vec<f64>,
    /// Y coordinates along the path
    pub y: Vec<f64>,
    /// Yaw angles along the path
    pub yaw: Vec<f64>,
    /// Direction at each point (1 = forward, -1 = backward)
    pub directions: Vec<i32>,
}

impl HybridAStarPath {
    pub fn is_empty(&self) -> bool {
        self.x.is_empty()
    }

    pub fn len(&self) -> usize {
        self.x.len()
    }
}

/// Hybrid A* path planner
pub struct HybridAStarPlanner {
    config: HybridAStarConfig,
    grid_map: GridMap,
    /// Precomputed holonomic heuristic (cost from each grid cell to goal)
    h_map: Vec<Vec<f64>>,
    goal_x: i32,
    goal_y: i32,
}

impl HybridAStarPlanner {
    /// Create a new planner from obstacle coordinates
    pub fn new(ox: &[f64], oy: &[f64], config: HybridAStarConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.xy_resolution, config.robot_radius)?;

        Ok(Self {
            config,
            grid_map,
            h_map: Vec::new(),
            goal_x: 0,
            goal_y: 0,
        })
    }

    /// Plan a path from start pose to goal pose
    pub fn plan(
        &mut self,
        sx: f64,
        sy: f64,
        syaw: f64,
        gx: f64,
        gy: f64,
        gyaw: f64,
    ) -> RoboticsResult<HybridAStarPath> {
        // Compute grid indices for goal
        self.goal_x = self.grid_map.calc_x_index(gx);
        self.goal_y = self.grid_map.calc_y_index(gy);

        // Compute holonomic heuristic map (Dijkstra from goal)
        self.h_map = self.compute_holonomic_heuristic();

        let start_yaw_index = self.calc_yaw_index(syaw);
        let start_node = HybridNode {
            x_index: self.grid_map.calc_x_index(sx),
            y_index: self.grid_map.calc_y_index(sy),
            yaw_index: start_yaw_index,
            direction: true,
            x_list: vec![sx],
            y_list: vec![sy],
            yaw_list: vec![syaw],
            directions: vec![1],
            steer: 0.0,
            cost: 0.0,
            parent_index: None,
        };

        let goal_yaw_index = self.calc_yaw_index(gyaw);
        let goal_node = HybridNode {
            x_index: self.goal_x,
            y_index: self.goal_y,
            yaw_index: goal_yaw_index,
            direction: true,
            x_list: vec![gx],
            y_list: vec![gy],
            yaw_list: vec![gyaw],
            directions: vec![1],
            steer: 0.0,
            cost: 0.0,
            parent_index: None,
        };

        let mut open_set = BinaryHeap::new();
        let mut closed_set: HashMap<i64, usize> = HashMap::new();
        let mut node_storage: Vec<HybridNode> = Vec::new();

        node_storage.push(start_node.clone());
        let start_idx = 0;
        let start_key = self.calc_hybrid_index(&node_storage[start_idx]);

        let h = self.calc_heuristic(&node_storage[start_idx]);
        open_set.push(PriorityNode {
            index: start_idx,
            cost: node_storage[start_idx].cost + h,
        });

        let mut open_map: HashMap<i64, usize> = HashMap::new();
        open_map.insert(start_key, start_idx);

        let mut iteration = 0;

        while let Some(current_priority) = open_set.pop() {
            let current_idx = current_priority.index;
            let current_key = self.calc_hybrid_index(&node_storage[current_idx]);

            if closed_set.contains_key(&current_key) {
                continue;
            }

            closed_set.insert(current_key, current_idx);
            open_map.remove(&current_key);

            // Check goal reached
            if self.is_goal(&node_storage[current_idx], &goal_node) {
                return Ok(self.build_path(current_idx, &node_storage, None));
            }

            // Try analytic expansion (Reeds-Shepp)
            iteration += 1;
            if iteration % self.config.analytic_expansion_interval == 0 {
                if let Some(rs) = self.analytic_expansion(&node_storage[current_idx], &goal_node) {
                    return Ok(self.build_path(current_idx, &node_storage, Some(&rs)));
                }
            }

            // Expand neighbors with motion primitives
            let neighbors = self.get_neighbors(&node_storage[current_idx], current_idx);
            for neighbor in neighbors {
                let neighbor_key = self.calc_hybrid_index(&neighbor);

                if closed_set.contains_key(&neighbor_key) {
                    continue;
                }

                if let Some(&existing_idx) = open_map.get(&neighbor_key) {
                    if neighbor.cost < node_storage[existing_idx].cost {
                        node_storage.push(neighbor);
                        let new_idx = node_storage.len() - 1;
                        open_map.insert(neighbor_key, new_idx);
                        let h = self.calc_heuristic(&node_storage[new_idx]);
                        open_set.push(PriorityNode {
                            index: new_idx,
                            cost: node_storage[new_idx].cost + h,
                        });
                    }
                } else {
                    node_storage.push(neighbor);
                    let new_idx = node_storage.len() - 1;
                    open_map.insert(neighbor_key, new_idx);
                    let h = self.calc_heuristic(&node_storage[new_idx]);
                    open_set.push(PriorityNode {
                        index: new_idx,
                        cost: node_storage[new_idx].cost + h,
                    });
                }
            }
        }

        Err(RoboticsError::PlanningError(
            "Hybrid A*: no path found".to_string(),
        ))
    }

    /// Discretize yaw to grid index
    fn calc_yaw_index(&self, yaw: f64) -> i32 {
        let normalized = pi_2_pi(yaw);
        ((normalized + PI) / self.config.yaw_resolution).round() as i32
    }

    /// Compute a unique index for the hybrid state (x, y, yaw)
    fn calc_hybrid_index(&self, node: &HybridNode) -> i64 {
        let n_yaw = ((2.0 * PI) / self.config.yaw_resolution).round() as i64 + 1;
        (node.yaw_index as i64)
            + n_yaw * (node.y_index as i64)
            + n_yaw * (self.grid_map.y_width as i64) * (node.x_index as i64)
    }

    /// Check if node is at goal within grid tolerance
    fn is_goal(&self, node: &HybridNode, goal: &HybridNode) -> bool {
        node.x_index == goal.x_index
            && node.y_index == goal.y_index
            && node.yaw_index == goal.yaw_index
    }

    /// Get heuristic cost from holonomic A* map
    fn calc_heuristic(&self, node: &HybridNode) -> f64 {
        let ix = node.x_index;
        let iy = node.y_index;

        if ix >= 0 && ix < self.grid_map.x_width && iy >= 0 && iy < self.grid_map.y_width {
            self.h_map[ix as usize][iy as usize] * self.config.h_cost
        } else {
            // Fallback: Euclidean distance
            let dx = (node.x_index - self.goal_x) as f64;
            let dy = (node.y_index - self.goal_y) as f64;
            (dx * dx + dy * dy).sqrt() * self.config.h_cost
        }
    }

    /// Generate neighbor nodes using bicycle model motion primitives
    fn get_neighbors(&self, current: &HybridNode, parent_idx: usize) -> Vec<HybridNode> {
        let mut neighbors = Vec::new();

        let n_steer = self.config.n_steer as f64;
        let steer_step = 2.0 * self.config.max_steer / n_steer;

        // For each steering angle
        for i in 0..=self.config.n_steer {
            let steer = -self.config.max_steer + i as f64 * steer_step;

            // Forward and backward
            for &direction in &[1.0_f64, -1.0] {
                let current_x = *current.x_list.last().unwrap();
                let current_y = *current.y_list.last().unwrap();
                let current_yaw = *current.yaw_list.last().unwrap();

                if let Some(node) = self.simulate_motion(
                    current_x,
                    current_y,
                    current_yaw,
                    steer,
                    direction,
                    current,
                    parent_idx,
                ) {
                    neighbors.push(node);
                }
            }
        }

        neighbors
    }

    /// Simulate bicycle model forward by one step
    fn simulate_motion(
        &self,
        x: f64,
        y: f64,
        yaw: f64,
        steer: f64,
        direction: f64,
        parent: &HybridNode,
        parent_idx: usize,
    ) -> Option<HybridNode> {
        let arc_length = self.config.xy_resolution * 1.5;
        let n_steps = (arc_length / self.config.step_size).ceil() as usize;
        let d = direction * self.config.step_size;

        let mut cx = x;
        let mut cy = y;
        let mut cyaw = yaw;

        let mut x_list = vec![cx];
        let mut y_list = vec![cy];
        let mut yaw_list = vec![cyaw];

        for _ in 0..n_steps {
            cx += d * cyaw.cos();
            cy += d * cyaw.sin();
            cyaw += d * steer.tan() / self.config.wheelbase;
            cyaw = pi_2_pi(cyaw);

            x_list.push(cx);
            y_list.push(cy);
            yaw_list.push(cyaw);
        }

        // Collision check
        if !self.verify_path(&x_list, &y_list) {
            return None;
        }

        let dir_val = if direction > 0.0 { 1 } else { -1 };
        let directions = vec![dir_val; x_list.len()];

        // Compute cost
        let mut cost = parent.cost;

        // Distance cost
        cost += arc_length;

        // Steering cost
        cost += self.config.steer_cost * steer.abs();

        // Steering change cost
        cost += self.config.steer_change_cost * (steer - parent.steer).abs();

        // Direction switch cost
        let parent_dir = if parent.direction { 1.0 } else { -1.0 };
        if direction != parent_dir {
            cost += self.config.switch_back_cost;
        }

        let x_idx = self.grid_map.calc_x_index(cx);
        let y_idx = self.grid_map.calc_y_index(cy);
        let yaw_idx = self.calc_yaw_index(cyaw);

        Some(HybridNode {
            x_index: x_idx,
            y_index: y_idx,
            yaw_index: yaw_idx,
            direction: direction > 0.0,
            x_list,
            y_list,
            yaw_list,
            directions,
            steer,
            cost,
            parent_index: Some(parent_idx),
        })
    }

    /// Verify that all points in a path are collision-free
    fn verify_path(&self, x_list: &[f64], y_list: &[f64]) -> bool {
        for (&x, &y) in x_list.iter().zip(y_list.iter()) {
            let ix = self.grid_map.calc_x_index(x);
            let iy = self.grid_map.calc_y_index(y);
            if !self.grid_map.is_valid(ix, iy) {
                return false;
            }
        }
        true
    }

    /// Attempt analytic expansion using Reeds-Shepp path
    fn analytic_expansion(&self, current: &HybridNode, goal: &HybridNode) -> Option<HybridNode> {
        let sx = *current.x_list.last().unwrap();
        let sy = *current.y_list.last().unwrap();
        let syaw = *current.yaw_list.last().unwrap();

        let gx = *goal.x_list.last().unwrap();
        let gy = *goal.y_list.last().unwrap();
        let gyaw = *goal.yaw_list.last().unwrap();

        let result = reeds_shepp_path::reeds_shepp_path_planning(
            sx,
            sy,
            syaw,
            gx,
            gy,
            gyaw,
            self.config.max_curvature,
            self.config.step_size,
        );

        let (path_x, path_y, path_yaw, _ctypes, _lengths) = result?;

        if path_x.is_empty() {
            return None;
        }

        // Collision check along the Reeds-Shepp path
        if !self.verify_path(&path_x, &path_y) {
            return None;
        }

        // Build directions from consecutive positions
        let directions: Vec<i32> = path_yaw
            .windows(2)
            .map(|w| {
                let diff = pi_2_pi(w[1] - w[0]);
                if diff.abs() < 1e-6 {
                    1
                } else if diff > 0.0 {
                    1
                } else {
                    -1
                }
            })
            .chain(std::iter::once(1))
            .collect();

        let last_x = *path_x.last().unwrap();
        let last_y = *path_y.last().unwrap();
        let last_yaw = *path_yaw.last().unwrap();

        Some(HybridNode {
            x_index: self.grid_map.calc_x_index(last_x),
            y_index: self.grid_map.calc_y_index(last_y),
            yaw_index: self.calc_yaw_index(last_yaw),
            direction: true,
            x_list: path_x,
            y_list: path_y,
            yaw_list: path_yaw,
            directions,
            steer: 0.0,
            cost: 0.0,
            parent_index: None,
        })
    }

    /// Build the final path by tracing back from goal to start
    fn build_path(
        &self,
        final_idx: usize,
        storage: &[HybridNode],
        rs_segment: Option<&HybridNode>,
    ) -> HybridAStarPath {
        let mut segments: Vec<(Vec<f64>, Vec<f64>, Vec<f64>, Vec<i32>)> = Vec::new();

        let mut current_idx = Some(final_idx);
        while let Some(idx) = current_idx {
            let node = &storage[idx];
            segments.push((
                node.x_list.clone(),
                node.y_list.clone(),
                node.yaw_list.clone(),
                node.directions.clone(),
            ));
            current_idx = node.parent_index;
        }

        segments.reverse();

        let mut x = Vec::new();
        let mut y = Vec::new();
        let mut yaw = Vec::new();
        let mut directions = Vec::new();

        for (sx, sy, syaw, sd) in &segments {
            x.extend_from_slice(sx);
            y.extend_from_slice(sy);
            yaw.extend_from_slice(syaw);
            directions.extend_from_slice(sd);
        }

        // Append Reeds-Shepp segment if present
        if let Some(rs) = rs_segment {
            x.extend_from_slice(&rs.x_list);
            y.extend_from_slice(&rs.y_list);
            yaw.extend_from_slice(&rs.yaw_list);
            directions.extend_from_slice(&rs.directions);
        }

        HybridAStarPath {
            x,
            y,
            yaw,
            directions,
        }
    }

    /// Compute holonomic heuristic using Dijkstra from goal on the grid
    fn compute_holonomic_heuristic(&self) -> Vec<Vec<f64>> {
        let xw = self.grid_map.x_width as usize;
        let yw = self.grid_map.y_width as usize;

        let mut cost_map = vec![vec![f64::INFINITY; yw]; xw];

        let gx = self.goal_x;
        let gy = self.goal_y;

        if gx < 0 || gx >= self.grid_map.x_width || gy < 0 || gy >= self.grid_map.y_width {
            return cost_map;
        }

        cost_map[gx as usize][gy as usize] = 0.0;

        let mut open_set = BinaryHeap::new();
        open_set.push(DijkstraNode {
            x: gx,
            y: gy,
            cost: 0.0,
        });

        let motions: [(i32, i32, f64); 8] = [
            (1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (-1, -1, std::f64::consts::SQRT_2),
            (-1, 1, std::f64::consts::SQRT_2),
            (1, -1, std::f64::consts::SQRT_2),
            (1, 1, std::f64::consts::SQRT_2),
        ];

        while let Some(current) = open_set.pop() {
            if current.cost > cost_map[current.x as usize][current.y as usize] {
                continue;
            }

            for &(dx, dy, mc) in &motions {
                let nx = current.x + dx;
                let ny = current.y + dy;

                if nx < 0 || nx >= self.grid_map.x_width || ny < 0 || ny >= self.grid_map.y_width {
                    continue;
                }

                if self.grid_map.obstacle_map[nx as usize][ny as usize] {
                    continue;
                }

                let new_cost = current.cost + mc;
                if new_cost < cost_map[nx as usize][ny as usize] {
                    cost_map[nx as usize][ny as usize] = new_cost;
                    open_set.push(DijkstraNode {
                        x: nx,
                        y: ny,
                        cost: new_cost,
                    });
                }
            }
        }

        cost_map
    }
}

/// Dijkstra node for holonomic heuristic computation
#[derive(Debug)]
struct DijkstraNode {
    x: i32,
    y: i32,
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

/// Normalize angle to [-pi, pi]
fn pi_2_pi(angle: f64) -> f64 {
    let mut a = angle;
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Create a rectangular boundary with obstacles
    fn create_boundary_obstacles(
        x_min: f64,
        x_max: f64,
        y_min: f64,
        y_max: f64,
        step: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        let mut x = x_min;
        while x <= x_max {
            ox.push(x);
            oy.push(y_min);
            ox.push(x);
            oy.push(y_max);
            x += step;
        }

        let mut y = y_min;
        while y <= y_max {
            ox.push(x_min);
            oy.push(y);
            ox.push(x_max);
            oy.push(y);
            y += step;
        }

        (ox, oy)
    }

    #[test]
    fn test_open_space_path() {
        let (ox, oy) = create_boundary_obstacles(0.0, 50.0, 0.0, 50.0, 1.0);

        let config = HybridAStarConfig {
            xy_resolution: 2.0,
            yaw_resolution: PI / 18.0,
            robot_radius: 1.0,
            ..Default::default()
        };

        let mut planner = HybridAStarPlanner::new(&ox, &oy, config).unwrap();

        let result = planner.plan(10.0, 10.0, 0.0, 40.0, 40.0, 0.0);
        assert!(result.is_ok(), "Planning failed: {:?}", result.err());

        let path = result.unwrap();
        assert!(!path.is_empty());
        assert_eq!(path.x.len(), path.y.len());
        assert_eq!(path.x.len(), path.yaw.len());

        // Check path starts near start
        let start_dist = ((path.x[0] - 10.0).powi(2) + (path.y[0] - 10.0).powi(2)).sqrt();
        assert!(
            start_dist < 5.0,
            "Path start too far from start: {}",
            start_dist
        );

        let last = path.x.len() - 1;
        let goal_dist = ((path.x[last] - 40.0).powi(2) + (path.y[last] - 40.0).powi(2)).sqrt();
        assert!(
            goal_dist < 10.0,
            "Path end too far from goal: {}",
            goal_dist
        );
    }

    #[test]
    fn test_path_around_obstacle() {
        let (mut ox, mut oy) = create_boundary_obstacles(0.0, 50.0, 0.0, 50.0, 1.0);

        // Add a wall in the middle
        let mut y = 10.0;
        while y <= 40.0 {
            ox.push(25.0);
            oy.push(y);
            y += 1.0;
        }

        let config = HybridAStarConfig {
            xy_resolution: 2.0,
            yaw_resolution: PI / 18.0,
            robot_radius: 1.0,
            ..Default::default()
        };

        let mut planner = HybridAStarPlanner::new(&ox, &oy, config).unwrap();

        let result = planner.plan(10.0, 25.0, 0.0, 40.0, 25.0, 0.0);
        assert!(
            result.is_ok(),
            "Planning with obstacle failed: {:?}",
            result.err()
        );

        let path = result.unwrap();
        assert!(!path.is_empty());
        assert!(path.len() > 5, "Path should have multiple waypoints");
    }

    #[test]
    fn test_goal_tolerance() {
        let (ox, oy) = create_boundary_obstacles(0.0, 50.0, 0.0, 50.0, 1.0);

        let config = HybridAStarConfig {
            xy_resolution: 2.0,
            yaw_resolution: PI / 18.0,
            robot_radius: 1.0,
            ..Default::default()
        };

        let mut planner = HybridAStarPlanner::new(&ox, &oy, config).unwrap();

        let result = planner.plan(10.0, 10.0, 0.0, 35.0, 35.0, PI / 2.0);
        assert!(result.is_ok(), "Planning failed: {:?}", result.err());

        let path = result.unwrap();
        let last = path.x.len() - 1;
        let goal_dist = ((path.x[last] - 35.0).powi(2) + (path.y[last] - 35.0).powi(2)).sqrt();
        assert!(
            goal_dist < 15.0,
            "Path end too far from goal: dist={}",
            goal_dist
        );
    }

    #[test]
    fn test_invalid_config() {
        let (ox, oy) = create_boundary_obstacles(0.0, 50.0, 0.0, 50.0, 1.0);

        let config = HybridAStarConfig {
            xy_resolution: 0.0,
            ..Default::default()
        };

        let result = HybridAStarPlanner::new(&ox, &oy, config);
        assert!(result.is_err());
    }

    #[test]
    fn test_path_result_consistency() {
        let (ox, oy) = create_boundary_obstacles(0.0, 50.0, 0.0, 50.0, 1.0);

        let config = HybridAStarConfig {
            xy_resolution: 2.0,
            yaw_resolution: PI / 18.0,
            robot_radius: 1.0,
            ..Default::default()
        };

        let mut planner = HybridAStarPlanner::new(&ox, &oy, config).unwrap();

        let result = planner.plan(10.0, 10.0, 0.0, 30.0, 30.0, 0.0);
        assert!(result.is_ok());

        let path = result.unwrap();
        // All vectors should have same length
        assert_eq!(path.x.len(), path.y.len());
        assert_eq!(path.x.len(), path.yaw.len());
        assert_eq!(path.x.len(), path.directions.len());

        // All yaw values should be in [-pi, pi]
        for &yaw in &path.yaw {
            assert!(
                yaw >= -PI - 0.01 && yaw <= PI + 0.01,
                "Yaw out of range: {}",
                yaw
            );
        }

        // All directions should be 1 or -1
        for &d in &path.directions {
            assert!(d == 1 || d == -1, "Invalid direction: {}", d);
        }
    }
}
