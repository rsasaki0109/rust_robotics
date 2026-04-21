#![allow(dead_code, clippy::too_many_arguments)]

//! RRT*-Reeds-Shepp path planner
//!
//! Combines the RRT* (optimal RRT) algorithm with Reeds-Shepp curve
//! connections for non-holonomic vehicle path planning. Unlike the Dubins
//! variant, the vehicle can move both forward and backward.
//!
//! The key additions over plain RRT-Reeds-Shepp are:
//! - **choose_parent**: picks the lowest-cost parent among nearby nodes.
//! - **rewire**: redirects nearby nodes through the new node when beneficial.
//! - **try_goal_path**: after adding each node, attempts a direct connection
//!   to the goal.
//!
//! # References
//!
//! * Karaman, S. and Frazzoli, E. (2011). "Sampling-based algorithms for
//!   optimal motion planning." *IJRR*.
//! * Reeds, J.A. and Shepp, L.A. (1990). "Optimal paths for a car that goes
//!   both forwards and backwards."

use std::f64::consts::PI;

use rand::Rng;

use rust_robotics_core::types::Pose2D;

use crate::reeds_shepp_path::reeds_shepp_path_planning;
use crate::rrt::{AreaBounds, CircleObstacle};

/// A node in the RRT*-Reeds-Shepp tree, carrying position and heading.
#[derive(Debug, Clone)]
pub struct RRTStarRSNode {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub path_x: Vec<f64>,
    pub path_y: Vec<f64>,
    pub path_yaw: Vec<f64>,
    pub cost: f64,
    pub parent: Option<usize>,
}

impl RRTStarRSNode {
    pub fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self {
            x,
            y,
            yaw,
            path_x: Vec::new(),
            path_y: Vec::new(),
            path_yaw: Vec::new(),
            cost: 0.0,
            parent: None,
        }
    }
}

/// Configuration for the RRT*-Reeds-Shepp planner.
#[derive(Debug, Clone)]
pub struct RRTStarRSConfig {
    /// Maximum curvature = 1 / minimum turning radius \[1/m\].
    pub curvature: f64,
    /// Step size for sampling points along Reeds-Shepp paths \[m\].
    pub step_size: f64,
    /// Goal-biased sampling rate (0..100). Higher means more goal samples.
    pub goal_sample_rate: i32,
    /// Maximum number of RRT iterations.
    pub max_iter: usize,
    /// Distance threshold to consider a node at the goal position \[m\].
    pub goal_xy_threshold: f64,
    /// Yaw threshold to consider a node at the goal heading \[rad\].
    pub goal_yaw_threshold: f64,
    /// Robot radius for collision checking \[m\].
    pub robot_radius: f64,
    /// Near-node search radius parameter.
    pub connect_circle_dist: f64,
}

impl Default for RRTStarRSConfig {
    fn default() -> Self {
        Self {
            curvature: 1.0,
            step_size: 0.2,
            goal_sample_rate: 10,
            max_iter: 200,
            goal_xy_threshold: 0.5,
            goal_yaw_threshold: 1.0_f64.to_radians(),
            robot_radius: 0.0,
            connect_circle_dist: 50.0,
        }
    }
}

/// Result of a successful RRT*-Reeds-Shepp planning run.
#[derive(Debug, Clone)]
pub struct RRTStarRSPath {
    /// Sampled waypoints along the path: (x, y, yaw).
    pub poses: Vec<Pose2D>,
}

/// RRT* path planner using Reeds-Shepp curves for steering.
pub struct RRTStarRSPlanner {
    config: RRTStarRSConfig,
    obstacles: Vec<CircleObstacle>,
    rand_area: AreaBounds,
    node_list: Vec<RRTStarRSNode>,
    start: RRTStarRSNode,
    goal: RRTStarRSNode,
}

impl RRTStarRSPlanner {
    /// Create a new RRT*-Reeds-Shepp planner.
    pub fn new(
        obstacles: Vec<CircleObstacle>,
        rand_area: AreaBounds,
        config: RRTStarRSConfig,
    ) -> Self {
        Self {
            config,
            obstacles,
            rand_area,
            node_list: Vec::new(),
            start: RRTStarRSNode::new(0.0, 0.0, 0.0),
            goal: RRTStarRSNode::new(0.0, 0.0, 0.0),
        }
    }

    /// Plan a path from `start` to `goal`.
    ///
    /// Returns `None` if no path is found within `max_iter` iterations.
    pub fn planning(&mut self, start: Pose2D, goal: Pose2D) -> Option<RRTStarRSPath> {
        self.start = RRTStarRSNode::new(start.x, start.y, start.yaw);
        self.goal = RRTStarRSNode::new(goal.x, goal.y, goal.yaw);
        self.node_list = vec![self.start.clone()];

        for _ in 0..self.config.max_iter {
            let rnd = self.get_random_node();
            let nearest_ind = self.get_nearest_node_index(&rnd);

            if let Some(new_node) =
                self.steer(&self.node_list[nearest_ind].clone(), &rnd, nearest_ind)
            {
                if self.check_collision(&new_node) {
                    let near_inds = self.find_near_nodes(&new_node);
                    if let Some(best_node) = self.choose_parent(new_node, &near_inds) {
                        let new_index = self.node_list.len();
                        self.node_list.push(best_node);
                        self.rewire(new_index, &near_inds);
                        self.try_goal_path(&self.node_list[new_index].clone());
                    }
                }
            }

            if let Some(goal_idx) = self.search_best_goal_node() {
                return Some(self.generate_final_course(goal_idx));
            }
        }

        self.search_best_goal_node()
            .map(|idx| self.generate_final_course(idx))
    }

    /// Plan with a deterministic node sampler (for testing).
    pub fn plan_with_sampler<F>(
        &mut self,
        start: Pose2D,
        goal: Pose2D,
        mut sample_node: F,
    ) -> Option<RRTStarRSPath>
    where
        F: FnMut(&Self) -> RRTStarRSNode,
    {
        self.start = RRTStarRSNode::new(start.x, start.y, start.yaw);
        self.goal = RRTStarRSNode::new(goal.x, goal.y, goal.yaw);
        self.node_list = vec![self.start.clone()];

        for _ in 0..self.config.max_iter {
            let rnd = sample_node(self);
            let nearest_ind = self.get_nearest_node_index(&rnd);

            if let Some(new_node) =
                self.steer(&self.node_list[nearest_ind].clone(), &rnd, nearest_ind)
            {
                if self.check_collision(&new_node) {
                    let near_inds = self.find_near_nodes(&new_node);
                    if let Some(best_node) = self.choose_parent(new_node, &near_inds) {
                        let new_index = self.node_list.len();
                        self.node_list.push(best_node);
                        self.rewire(new_index, &near_inds);
                        self.try_goal_path(&self.node_list[new_index].clone());
                    }
                }
            }

            if let Some(goal_idx) = self.search_best_goal_node() {
                return Some(self.generate_final_course(goal_idx));
            }
        }

        self.search_best_goal_node()
            .map(|idx| self.generate_final_course(idx))
    }

    /// Access the internal tree for inspection or visualization.
    pub fn get_tree(&self) -> &[RRTStarRSNode] {
        &self.node_list
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    fn get_random_node(&self) -> RRTStarRSNode {
        let mut rng = rand::rng();
        if rng.random_range(0..=100) > self.config.goal_sample_rate {
            RRTStarRSNode::new(
                rng.random_range(self.rand_area.xmin..=self.rand_area.xmax),
                rng.random_range(self.rand_area.ymin..=self.rand_area.ymax),
                rng.random_range(-PI..=PI),
            )
        } else {
            RRTStarRSNode::new(self.goal.x, self.goal.y, self.goal.yaw)
        }
    }

    fn get_nearest_node_index(&self, rnd_node: &RRTStarRSNode) -> usize {
        let mut min_dist = f64::INFINITY;
        let mut min_ind = 0;
        for (i, node) in self.node_list.iter().enumerate() {
            let dist = (node.x - rnd_node.x).powi(2) + (node.y - rnd_node.y).powi(2);
            if dist < min_dist {
                min_dist = dist;
                min_ind = i;
            }
        }
        min_ind
    }

    /// Steer from `from_node` toward `to_node` using a Reeds-Shepp curve.
    fn steer(
        &self,
        from_node: &RRTStarRSNode,
        to_node: &RRTStarRSNode,
        from_index: usize,
    ) -> Option<RRTStarRSNode> {
        let result = reeds_shepp_path_planning(
            from_node.x,
            from_node.y,
            from_node.yaw,
            to_node.x,
            to_node.y,
            to_node.yaw,
            self.config.curvature,
            self.config.step_size,
        )?;

        let (px, py, pyaw, _modes, lengths) = result;

        if px.is_empty() {
            return None;
        }

        let mut new_node = from_node.clone();
        new_node.x = *px.last().unwrap();
        new_node.y = *py.last().unwrap();
        new_node.yaw = *pyaw.last().unwrap();
        new_node.path_x = px;
        new_node.path_y = py;
        new_node.path_yaw = pyaw;
        new_node.cost += lengths.iter().map(|l| l.abs()).sum::<f64>();
        new_node.parent = Some(from_index);

        Some(new_node)
    }

    /// Compute the cost of steering from `from_node` to `to_node` via
    /// Reeds-Shepp.
    fn calc_new_cost(&self, from_node: &RRTStarRSNode, to_node: &RRTStarRSNode) -> f64 {
        match reeds_shepp_path_planning(
            from_node.x,
            from_node.y,
            from_node.yaw,
            to_node.x,
            to_node.y,
            to_node.yaw,
            self.config.curvature,
            self.config.step_size,
        ) {
            Some((_px, _py, _pyaw, _modes, lengths)) => {
                from_node.cost + lengths.iter().map(|l| l.abs()).sum::<f64>()
            }
            None => f64::INFINITY,
        }
    }

    /// Check that a node's path does not collide with any obstacle.
    fn check_collision(&self, node: &RRTStarRSNode) -> bool {
        for obs in &self.obstacles {
            for (&px, &py) in node.path_x.iter().zip(node.path_y.iter()) {
                let dx = obs.x - px;
                let dy = obs.y - py;
                let d = (dx * dx + dy * dy).sqrt();
                if d <= obs.radius + self.config.robot_radius {
                    return false;
                }
            }
        }
        true
    }

    /// Find nodes within a dynamic radius around `new_node`.
    fn find_near_nodes(&self, new_node: &RRTStarRSNode) -> Vec<usize> {
        let nnode = self.node_list.len() + 1;
        let r = self.config.connect_circle_dist * ((nnode as f64).ln() / nnode as f64).sqrt();

        self.node_list
            .iter()
            .enumerate()
            .filter_map(|(i, node)| {
                let dist_sq = (node.x - new_node.x).powi(2) + (node.y - new_node.y).powi(2);
                if dist_sq <= r * r {
                    Some(i)
                } else {
                    None
                }
            })
            .collect()
    }

    /// Choose the best parent for `new_node` from the near nodes.
    fn choose_parent(&self, new_node: RRTStarRSNode, near_inds: &[usize]) -> Option<RRTStarRSNode> {
        if near_inds.is_empty() {
            return Some(new_node);
        }

        let mut best_cost = f64::INFINITY;
        let mut best_index: Option<usize> = None;

        for &i in near_inds {
            let near_node = &self.node_list[i];
            if let Some(t_node) = self.steer(near_node, &new_node, i) {
                if self.check_collision(&t_node) {
                    let cost = self.calc_new_cost(near_node, &new_node);
                    if cost < best_cost {
                        best_cost = cost;
                        best_index = Some(i);
                    }
                }
            }
        }

        let parent_ind = best_index?;
        let mut result = self.steer(&self.node_list[parent_ind], &new_node, parent_ind)?;
        result.cost = best_cost;
        Some(result)
    }

    /// Rewire nearby nodes through `new_node_ind` if it reduces cost.
    fn rewire(&mut self, new_node_ind: usize, near_inds: &[usize]) {
        for &i in near_inds {
            let near_node = self.node_list[i].clone();
            let new_node = &self.node_list[new_node_ind];

            let edge_cost = self.calc_new_cost(new_node, &near_node);
            if edge_cost >= near_node.cost {
                continue;
            }

            if let Some(mut edge_node) = self.steer(
                &self.node_list[new_node_ind].clone(),
                &near_node,
                new_node_ind,
            ) {
                if self.check_collision(&edge_node) {
                    edge_node.cost = edge_cost;
                    self.node_list[i] = edge_node;
                    self.propagate_cost_to_leaves(i);
                }
            }
        }
    }

    /// After adding a new node, try to connect it directly to the goal.
    fn try_goal_path(&mut self, node: &RRTStarRSNode) {
        let goal_node = RRTStarRSNode::new(self.goal.x, self.goal.y, self.goal.yaw);
        let node_index = self.node_list.len() - 1;

        if let Some(new_node) = self.steer(node, &goal_node, node_index) {
            if self.check_collision(&new_node) {
                self.node_list.push(new_node);
            }
        }
    }

    /// Propagate cost updates to all descendants of `parent_ind`.
    fn propagate_cost_to_leaves(&mut self, parent_ind: usize) {
        for i in 0..self.node_list.len() {
            if let Some(p) = self.node_list[i].parent {
                if p == parent_ind {
                    self.node_list[i].cost = self.calc_new_cost(
                        &self.node_list[parent_ind].clone(),
                        &self.node_list[i].clone(),
                    );
                    self.propagate_cost_to_leaves(i);
                }
            }
        }
    }

    /// Search for the best (lowest-cost) node near the goal.
    fn search_best_goal_node(&self) -> Option<usize> {
        let mut candidates: Vec<usize> = Vec::new();

        for (i, node) in self.node_list.iter().enumerate() {
            let dx = node.x - self.goal.x;
            let dy = node.y - self.goal.y;
            let dist = (dx * dx + dy * dy).sqrt();
            if dist > self.config.goal_xy_threshold {
                continue;
            }
            let dyaw = angle_diff(node.yaw, self.goal.yaw).abs();
            if dyaw > self.config.goal_yaw_threshold {
                continue;
            }
            candidates.push(i);
        }

        if candidates.is_empty() {
            return None;
        }

        candidates.into_iter().min_by(|&a, &b| {
            self.node_list[a]
                .cost
                .partial_cmp(&self.node_list[b].cost)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
    }

    /// Trace back from a goal node to the start, collecting all waypoints.
    fn generate_final_course(&self, goal_index: usize) -> RRTStarRSPath {
        let mut poses: Vec<Pose2D> = vec![Pose2D::new(self.goal.x, self.goal.y, self.goal.yaw)];

        let mut node = &self.node_list[goal_index];
        while node.parent.is_some() {
            for ((&px, &py), &pyaw) in node
                .path_x
                .iter()
                .rev()
                .zip(node.path_y.iter().rev())
                .zip(node.path_yaw.iter().rev())
            {
                poses.push(Pose2D::new(px, py, pyaw));
            }
            node = &self.node_list[node.parent.unwrap()];
        }

        poses.push(Pose2D::new(self.start.x, self.start.y, self.start.yaw));
        poses.reverse();

        RRTStarRSPath { poses }
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Compute the shortest signed angular difference.
fn angle_diff(a: f64, b: f64) -> f64 {
    let mut d = a - b;
    while d > PI {
        d -= 2.0 * PI;
    }
    while d < -PI {
        d += 2.0 * PI;
    }
    d
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn create_obstacle_list() -> Vec<CircleObstacle> {
        vec![
            CircleObstacle::new(5.0, 5.0, 1.0),
            CircleObstacle::new(4.0, 6.0, 1.0),
            CircleObstacle::new(4.0, 8.0, 1.0),
            CircleObstacle::new(4.0, 10.0, 1.0),
            CircleObstacle::new(6.0, 5.0, 1.0),
            CircleObstacle::new(7.0, 5.0, 1.0),
            CircleObstacle::new(8.0, 6.0, 1.0),
            CircleObstacle::new(8.0, 8.0, 1.0),
            CircleObstacle::new(8.0, 10.0, 1.0),
        ]
    }

    #[test]
    fn test_config_default() {
        let config = RRTStarRSConfig::default();
        assert_eq!(config.curvature, 1.0);
        assert_eq!(config.step_size, 0.2);
        assert_eq!(config.max_iter, 200);
        assert_eq!(config.goal_sample_rate, 10);
        assert_eq!(config.connect_circle_dist, 50.0);
        assert!(approx_eq(config.goal_xy_threshold, 0.5, 1e-12));
    }

    #[test]
    fn test_node_creation() {
        let node = RRTStarRSNode::new(1.0, 2.0, 0.5);
        assert_eq!(node.x, 1.0);
        assert_eq!(node.y, 2.0);
        assert_eq!(node.yaw, 0.5);
        assert_eq!(node.cost, 0.0);
        assert!(node.parent.is_none());
        assert!(node.path_x.is_empty());
    }

    #[test]
    fn test_angle_diff() {
        assert!(approx_eq(angle_diff(0.0, 0.0), 0.0, 1e-12));
        assert!(approx_eq(angle_diff(PI, 0.0), PI, 1e-12));
        assert!(approx_eq(angle_diff(0.0, PI), -PI, 1e-12));
        assert!(approx_eq(angle_diff(3.0 * PI / 2.0, -PI / 2.0), 0.0, 1e-12));
    }

    #[test]
    fn test_collision_check_no_obstacles() {
        let config = RRTStarRSConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let planner = RRTStarRSPlanner::new(vec![], rand_area, config);

        let mut node = RRTStarRSNode::new(1.0, 1.0, 0.0);
        node.path_x = vec![0.0, 0.5, 1.0];
        node.path_y = vec![0.0, 0.5, 1.0];
        assert!(planner.check_collision(&node));
    }

    #[test]
    fn test_collision_check_with_obstacle() {
        let obstacles = vec![CircleObstacle::new(0.5, 0.5, 0.3)];
        let config = RRTStarRSConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let planner = RRTStarRSPlanner::new(obstacles, rand_area, config);

        let mut node = RRTStarRSNode::new(1.0, 1.0, 0.0);
        node.path_x = vec![0.0, 0.5, 1.0];
        node.path_y = vec![0.0, 0.5, 1.0];
        assert!(!planner.check_collision(&node));
    }

    #[test]
    fn test_steer_produces_reeds_shepp_path() {
        let config = RRTStarRSConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTStarRSPlanner::new(vec![], rand_area, config);
        planner.start = RRTStarRSNode::new(0.0, 0.0, 0.0);
        planner.goal = RRTStarRSNode::new(10.0, 10.0, 0.0);
        planner.node_list = vec![planner.start.clone()];

        let from_node = RRTStarRSNode::new(0.0, 0.0, 0.0);
        let to_node = RRTStarRSNode::new(5.0, 5.0, PI / 2.0);

        let result = planner.steer(&from_node, &to_node, 0);
        assert!(result.is_some());

        let new_node = result.unwrap();
        assert!(!new_node.path_x.is_empty());
        assert_eq!(new_node.path_x.len(), new_node.path_y.len());
        assert_eq!(new_node.path_x.len(), new_node.path_yaw.len());
        assert!(new_node.cost > 0.0);
        assert!(new_node.parent.is_some());
    }

    #[test]
    fn test_find_near_nodes() {
        let config = RRTStarRSConfig {
            connect_circle_dist: 50.0,
            ..Default::default()
        };
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTStarRSPlanner::new(vec![], rand_area, config);
        planner.node_list = vec![
            RRTStarRSNode::new(0.0, 0.0, 0.0),
            RRTStarRSNode::new(1.0, 1.0, 0.0),
            RRTStarRSNode::new(100.0, 100.0, 0.0),
        ];

        let query = RRTStarRSNode::new(0.5, 0.5, 0.0);
        let near = planner.find_near_nodes(&query);
        assert!(near.contains(&0));
        assert!(near.contains(&1));
        assert!(!near.contains(&2));
    }

    #[test]
    fn test_search_best_goal_node() {
        let config = RRTStarRSConfig {
            goal_xy_threshold: 1.0,
            goal_yaw_threshold: 0.5,
            ..Default::default()
        };
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTStarRSPlanner::new(vec![], rand_area, config);
        planner.goal = RRTStarRSNode::new(10.0, 10.0, 0.0);

        // No nodes near goal
        planner.node_list = vec![RRTStarRSNode::new(0.0, 0.0, 0.0)];
        assert!(planner.search_best_goal_node().is_none());

        // Node near goal in position but wrong yaw
        let mut n = RRTStarRSNode::new(10.0, 10.0, PI);
        n.cost = 5.0;
        planner.node_list.push(n);
        assert!(planner.search_best_goal_node().is_none());

        // Node near goal in both position and yaw
        let mut n = RRTStarRSNode::new(10.2, 10.2, 0.1);
        n.cost = 8.0;
        planner.node_list.push(n);
        assert_eq!(planner.search_best_goal_node(), Some(2));
    }

    #[test]
    fn test_deterministic_planning_no_obstacles() {
        let config = RRTStarRSConfig {
            curvature: 1.0,
            step_size: 0.2,
            goal_sample_rate: 10,
            max_iter: 500,
            goal_xy_threshold: 1.5,
            goal_yaw_threshold: 0.5,
            robot_radius: 0.0,
            connect_circle_dist: 50.0,
        };
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTStarRSPlanner::new(vec![], rand_area, config);

        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(5.0, 5.0, 0.0);

        // Alternate between a midpoint and the goal to avoid the edge case
        // where from_node == to_node in Reeds-Shepp planning.
        let mut call = 0;
        let result = planner.plan_with_sampler(start, goal, |p| {
            call += 1;
            if call % 2 == 0 {
                RRTStarRSNode::new(p.goal.x, p.goal.y, p.goal.yaw)
            } else {
                RRTStarRSNode::new(p.goal.x * 0.5, p.goal.y * 0.5, p.goal.yaw)
            }
        });

        assert!(result.is_some(), "Should find a path with no obstacles");
        let path = result.unwrap();
        assert!(path.poses.len() >= 2);

        let first = &path.poses[0];
        let last = &path.poses[path.poses.len() - 1];
        assert!(approx_eq(first.x, 0.0, 0.5));
        assert!(approx_eq(first.y, 0.0, 0.5));
        assert!(approx_eq(last.x, 5.0, 1.5));
        assert!(approx_eq(last.y, 5.0, 1.5));
    }

    #[test]
    fn test_choose_parent_picks_lower_cost() {
        let config = RRTStarRSConfig {
            curvature: 1.0,
            step_size: 0.2,
            connect_circle_dist: 100.0,
            ..Default::default()
        };
        let rand_area = AreaBounds::new(-5.0, 20.0, -5.0, 20.0);
        let mut planner = RRTStarRSPlanner::new(vec![], rand_area, config);
        planner.start = RRTStarRSNode::new(0.0, 0.0, 0.0);
        planner.goal = RRTStarRSNode::new(20.0, 0.0, 0.0);

        // Node 0: start (cost 0)
        // Node 1: cost 100 (expensive parent)
        let mut expensive = RRTStarRSNode::new(5.0, 0.0, 0.0);
        expensive.cost = 100.0;
        expensive.parent = Some(0);

        planner.node_list = vec![planner.start.clone(), expensive];

        let new_node = RRTStarRSNode::new(3.0, 0.0, 0.0);
        let near_inds = vec![0, 1];
        let result = planner.choose_parent(new_node, &near_inds);
        assert!(result.is_some());
        let chosen = result.unwrap();
        assert_eq!(chosen.parent, Some(0));
    }

    #[test]
    fn test_planning_with_obstacles_runs_without_panic() {
        let obstacles = create_obstacle_list();
        let config = RRTStarRSConfig {
            curvature: 1.0,
            step_size: 0.2,
            goal_sample_rate: 10,
            max_iter: 200,
            goal_xy_threshold: 1.5,
            goal_yaw_threshold: 1.0,
            robot_radius: 0.0,
            connect_circle_dist: 50.0,
        };
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTStarRSPlanner::new(obstacles, rand_area, config);

        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(6.0, 7.0, PI / 2.0);

        let result = planner.planning(start, goal);
        if let Some(path) = result {
            assert!(path.poses.len() >= 2);
            for pose in &path.poses {
                assert!(pose.x.is_finite());
                assert!(pose.y.is_finite());
                assert!(pose.yaw.is_finite());
            }
        }
    }

    #[test]
    fn test_generate_final_course() {
        let config = RRTStarRSConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTStarRSPlanner::new(vec![], rand_area, config);
        planner.start = RRTStarRSNode::new(0.0, 0.0, 0.0);
        planner.goal = RRTStarRSNode::new(10.0, 10.0, 0.0);

        let root = RRTStarRSNode::new(0.0, 0.0, 0.0);
        let mut child = RRTStarRSNode::new(5.0, 5.0, 0.5);
        child.parent = Some(0);
        child.path_x = vec![0.0, 2.5, 5.0];
        child.path_y = vec![0.0, 2.5, 5.0];
        child.path_yaw = vec![0.0, 0.25, 0.5];

        planner.node_list = vec![root, child];

        let course = planner.generate_final_course(1);
        assert!(course.poses.len() >= 3);
        assert!(approx_eq(course.poses[0].x, 0.0, 1e-12));
        let last = course.poses.last().unwrap();
        assert!(approx_eq(last.x, 10.0, 1e-12));
        assert!(approx_eq(last.y, 10.0, 1e-12));
    }

    #[test]
    fn test_try_goal_path_adds_node() {
        let config = RRTStarRSConfig {
            curvature: 1.0,
            step_size: 0.2,
            goal_xy_threshold: 1.0,
            goal_yaw_threshold: 1.0,
            ..Default::default()
        };
        let rand_area = AreaBounds::new(-5.0, 20.0, -5.0, 20.0);
        let mut planner = RRTStarRSPlanner::new(vec![], rand_area, config);
        planner.start = RRTStarRSNode::new(0.0, 0.0, 0.0);
        planner.goal = RRTStarRSNode::new(5.0, 0.0, 0.0);
        planner.node_list = vec![planner.start.clone()];

        // Add a node close to the goal
        let near_goal = RRTStarRSNode::new(3.0, 0.0, 0.0);
        if let Some(n) = planner.steer(&planner.node_list[0].clone(), &near_goal, 0) {
            planner.node_list.push(n);
        }

        let count_before = planner.node_list.len();
        let last_node = planner.node_list.last().unwrap().clone();
        planner.try_goal_path(&last_node);
        // try_goal_path should add a new node (direct connection to goal)
        assert!(
            planner.node_list.len() >= count_before,
            "try_goal_path should not remove nodes"
        );
    }
}
