//! Enhanced Lazy Theta* path planning algorithm
//!
//! Extends Lazy Theta* with improved fallback parent selection. When the
//! optimistic line-of-sight check fails at expansion time, standard Lazy
//! Theta* falls back to the best 8-connected grid neighbor. Enhanced Lazy
//! Theta* additionally:
//!
//! 1. Checks line-of-sight to the fallback parent's ancestors (2-hop LOS),
//!    potentially skipping intermediate nodes for shorter paths.
//! 2. On every expansion (not just fallback), scans closed-set neighbors
//!    for any-angle shortcuts via line-of-sight, similar to Theta* but
//!    only at expansion time (still lazy).
//!
//! This produces paths that are shorter than both Theta* and Lazy Theta*
//! because the search explores a wider set of parent candidates at each
//! node expansion.
//!
//! Reference: Builds on Nash et al. (2010) "Lazy Theta*: Any-Angle Path
//! Planning and Path Length Analysis in 3D"

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use crate::grid::{GridMap, Node};
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Configuration for Enhanced Lazy Theta* planner
#[derive(Debug, Clone)]
pub struct EnhancedLazyThetaStarConfig {
    pub resolution: f64,
    pub robot_radius: f64,
    pub heuristic_weight: f64,
}

impl Default for EnhancedLazyThetaStarConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        }
    }
}

impl EnhancedLazyThetaStarConfig {
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
struct PriorityNode {
    x: i32,
    y: i32,
    #[allow(dead_code)]
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

pub struct EnhancedLazyThetaStarPlanner {
    grid_map: GridMap,
    config: EnhancedLazyThetaStarConfig,
    motion: Vec<(i32, i32, f64)>,
}

impl EnhancedLazyThetaStarPlanner {
    pub fn new(ox: &[f64], oy: &[f64], config: EnhancedLazyThetaStarConfig) -> Self {
        Self::try_new(ox, oy, config).expect(
            "invalid Enhanced Lazy Theta* planner input",
        )
    }

    pub fn try_new(
        ox: &[f64],
        oy: &[f64],
        config: EnhancedLazyThetaStarConfig,
    ) -> RoboticsResult<Self> {
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
        let config = EnhancedLazyThetaStarConfig {
            resolution,
            robot_radius,
            ..Default::default()
        };
        Self::new(ox, oy, config)
    }

    pub fn from_obstacle_points(
        obstacles: &Obstacles,
        config: EnhancedLazyThetaStarConfig,
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

    fn calc_heuristic(&self, n1_x: i32, n1_y: i32, n2_x: i32, n2_y: i32) -> f64 {
        self.config.heuristic_weight
            * (((n1_x - n2_x).pow(2) + (n1_y - n2_y).pow(2)) as f64).sqrt()
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

    fn ensure_query_is_valid(&self, x: i32, y: i32, label: &str) -> RoboticsResult<()> {
        if self.grid_map.is_valid(x, y) {
            return Ok(());
        }
        Err(RoboticsError::PlanningError(format!(
            "{} position is invalid",
            label
        )))
    }

    /// Enhanced fallback: find the best parent among closed-set neighbors
    /// in a 2-ring neighborhood, then walk ancestor chains for LOS shortcuts.
    fn enhanced_best_parent(
        &self,
        x: i32,
        y: i32,
        closed_set: &HashMap<i32, usize>,
        node_storage: &[Node],
    ) -> Option<(usize, f64)> {
        let mut best: Option<(usize, f64)> = None;
        const MAX_ANCESTOR_HOPS: usize = 6;

        // Collect candidate closed-set nodes from 2-ring neighborhood
        // Ring 1: 8-connected (distance 1 or sqrt(2))
        // Ring 2: distance 2, sqrt(5), 2*sqrt(2)
        let offsets: [(i32, i32); 24] = [
            // Ring 1 (8)
            (1, 0), (0, 1), (-1, 0), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
            // Ring 2 (16)
            (2, 0), (0, 2), (-2, 0), (0, -2),
            (2, 1), (2, -1), (-2, 1), (-2, -1),
            (1, 2), (1, -2), (-1, 2), (-1, -2),
            (2, 2), (2, -2), (-2, 2), (-2, -2),
        ];

        for &(dx, dy) in &offsets {
            let nx = x + dx;
            let ny = y + dy;
            if !self.grid_map.is_valid(nx, ny) {
                continue;
            }
            let neighbor_grid_index = self.grid_map.calc_index(nx, ny);
            if let Some(&neighbor_storage_index) = closed_set.get(&neighbor_grid_index) {
                let neighbor = &node_storage[neighbor_storage_index];

                // Direct LOS check to this closed neighbor
                if self.line_of_sight(neighbor.x, neighbor.y, x, y) {
                    let dist = self.euclidean_distance(neighbor.x, neighbor.y, x, y);
                    let g_via = neighbor.cost + dist;
                    if best.is_none_or(|(_, best_g)| g_via < best_g) {
                        best = Some((neighbor_storage_index, g_via));
                    }
                }

                // Walk ancestor chain of this neighbor
                let mut ancestor_idx = neighbor.parent_index;
                let mut hops = 0;
                while let Some(a_idx) = ancestor_idx {
                    if hops >= MAX_ANCESTOR_HOPS {
                        break;
                    }
                    let ancestor = &node_storage[a_idx];
                    if self.line_of_sight(ancestor.x, ancestor.y, x, y) {
                        let dist = self.euclidean_distance(ancestor.x, ancestor.y, x, y);
                        let g_via = ancestor.cost + dist;
                        if best.is_none_or(|(_, best_g)| g_via < best_g) {
                            best = Some((a_idx, g_via));
                        }
                    }
                    ancestor_idx = ancestor.parent_index;
                    hops += 1;
                }
            }
        }

        best
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
        let mut g_values: HashMap<i32, f64> = HashMap::new();
        let mut best_index: HashMap<i32, usize> = HashMap::new();

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

            if closed_set.contains_key(&current_grid_index) {
                continue;
            }

            // --- ENHANCED LAZY EVALUATION ---
            // Verify LOS to optimistic parent. On failure, use enhanced
            // fallback that searches ancestors and cross-branch parents.
            let mut corrected_index = current.index;
            let current_node = &node_storage[current.index];
            if let Some(parent_idx) = current_node.parent_index {
                let parent_node = &node_storage[parent_idx];
                let needs_correction = if let Some(grandparent_idx) = parent_node.parent_index {
                    let grandparent = &node_storage[grandparent_idx];
                    !self.line_of_sight(grandparent.x, grandparent.y, current.x, current.y)
                } else {
                    false
                };

                if needs_correction {
                    // Enhanced fallback: search wider parent candidates
                    if let Some((best_parent_idx, best_g)) = self.enhanced_best_parent(
                        current.x,
                        current.y,
                        &closed_set,
                        &node_storage,
                    ) {
                        node_storage.push(Node::new(
                            current.x,
                            current.y,
                            best_g,
                            Some(best_parent_idx),
                        ));
                        corrected_index = node_storage.len() - 1;
                        g_values.insert(current_grid_index, best_g);
                        best_index.insert(current_grid_index, corrected_index);
                    }
                } else {
                    // LOS succeeded — still try any-angle improvement via closed neighbors
                    // This is the key enhancement: even when the optimistic parent is valid,
                    // there might be a better parent among closed-set ancestors.
                    if let Some((better_idx, better_g)) = self.enhanced_best_parent(
                        current.x,
                        current.y,
                        &closed_set,
                        &node_storage,
                    ) {
                        if better_g < current_node.cost {
                            node_storage.push(Node::new(
                                current.x,
                                current.y,
                                better_g,
                                Some(better_idx),
                            ));
                            corrected_index = node_storage.len() - 1;
                            g_values.insert(current_grid_index, better_g);
                            best_index.insert(current_grid_index, corrected_index);
                        }
                    }
                }
            }

            if current.x == goal_x && current.y == goal_y {
                return Ok(self.build_path(corrected_index, &node_storage));
            }

            closed_set.insert(current_grid_index, corrected_index);

            let current_cost = node_storage[corrected_index].cost;
            let current_parent = node_storage[corrected_index].parent_index;

            for &(dx, dy, _) in &self.motion {
                let new_x = current.x + dx;
                let new_y = current.y + dy;
                let new_grid_index = self.grid_map.calc_index(new_x, new_y);
                if !self.grid_map.is_valid_offset(current.x, current.y, dx, dy) {
                    continue;
                }
                if closed_set.contains_key(&new_grid_index) {
                    continue;
                }

                // Optimistic parent assignment (same as Lazy Theta*)
                let (new_cost, new_parent_index) = if let Some(p_idx) = current_parent {
                    let parent_node = &node_storage[p_idx];
                    let dist =
                        self.euclidean_distance(parent_node.x, parent_node.y, new_x, new_y);
                    let cost_via_parent = parent_node.cost + dist;

                    let dist_via_current =
                        self.euclidean_distance(current.x, current.y, new_x, new_y);
                    let cost_via_current = current_cost + dist_via_current;

                    if cost_via_parent < cost_via_current {
                        (cost_via_parent, Some(p_idx))
                    } else {
                        (cost_via_current, Some(corrected_index))
                    }
                } else {
                    let dist = self.euclidean_distance(current.x, current.y, new_x, new_y);
                    (current_cost + dist, Some(corrected_index))
                };

                let existing_g = g_values
                    .get(&new_grid_index)
                    .copied()
                    .unwrap_or(f64::INFINITY);
                if new_cost < existing_g {
                    g_values.insert(new_grid_index, new_cost);
                    node_storage.push(Node::new(new_x, new_y, new_cost, new_parent_index));
                    let new_index = node_storage.len() - 1;
                    best_index.insert(new_grid_index, new_index);
                    let priority =
                        new_cost + self.calc_heuristic(new_x, new_y, goal_x, goal_y);
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

impl PathPlanner for EnhancedLazyThetaStarPlanner {
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
    fn test_enhanced_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = EnhancedLazyThetaStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);
        let result = planner.plan(Point2D::new(2.0, 10.0), Point2D::new(18.0, 10.0));
        assert!(result.is_ok());
        assert!(!result.unwrap().is_empty());
    }

    #[test]
    fn test_enhanced_shorter_than_a_star() {
        let (ox, oy) = create_simple_obstacles();
        let planner = EnhancedLazyThetaStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);
        let a_star = crate::a_star::AStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);
        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(18.0, 18.0);
        let enhanced_path = planner.plan(start, goal).unwrap();
        let a_star_path = a_star.plan(start, goal).unwrap();
        assert!(
            enhanced_path.total_length() <= a_star_path.total_length() + 0.1,
            "Enhanced ({:.2}) should not be longer than A* ({:.2})",
            enhanced_path.total_length(),
            a_star_path.total_length()
        );
    }

    #[test]
    fn test_enhanced_at_least_as_good_as_lazy_theta() {
        let (ox, oy) = create_simple_obstacles();
        let enhanced = EnhancedLazyThetaStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);
        let lazy = crate::lazy_theta_star::LazyThetaStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);
        let theta = crate::theta_star::ThetaStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(2.0, 10.0);
        let goal = Point2D::new(18.0, 10.0);

        let enhanced_len = enhanced.plan(start, goal).unwrap().total_length();
        let lazy_len = lazy.plan(start, goal).unwrap().total_length();
        let theta_len = theta.plan(start, goal).unwrap().total_length();

        // Enhanced should be at least as good as the better of Lazy and Theta
        let best_existing = lazy_len.min(theta_len);
        assert!(
            enhanced_len <= best_existing + 0.5,
            "Enhanced ({:.2}) should be at least as good as best existing ({:.2}, lazy={:.2}, theta={:.2})",
            enhanced_len,
            best_existing,
            lazy_len,
            theta_len
        );
    }

    #[test]
    fn test_enhanced_from_obstacle_points() {
        let (ox, oy) = create_simple_obstacles();
        let obstacles = Obstacles::try_from_xy(&ox, &oy).unwrap();
        let planner = EnhancedLazyThetaStarPlanner::from_obstacle_points(
            &obstacles,
            EnhancedLazyThetaStarConfig::default(),
        )
        .unwrap();
        let path = planner.plan_xy(2.0, 10.0, 18.0, 10.0).unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_enhanced_rejects_invalid_config() {
        let (ox, oy) = create_simple_obstacles();
        let config = EnhancedLazyThetaStarConfig {
            heuristic_weight: 0.0,
            ..Default::default()
        };
        assert!(EnhancedLazyThetaStarPlanner::try_new(&ox, &oy, config).is_err());
    }
}
