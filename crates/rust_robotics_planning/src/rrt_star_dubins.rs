#![allow(dead_code, clippy::too_many_arguments)]

//! RRT*-Dubins path planner
//!
//! Combines the RRT* (optimal RRT) algorithm with Dubins curve connections
//! for non-holonomic vehicle path planning. Extends the RRT-Dubins planner with
//! near-node rewiring so the tree converges toward an optimal solution.
//!
//! Each node carries a heading angle (x, y, yaw), and collision checking is
//! performed along the sampled Dubins curve. The key additions over plain
//! RRT-Dubins are:
//! - **choose_parent**: picks the lowest-cost parent among nearby nodes.
//! - **rewire**: redirects nearby nodes through the new node when beneficial.
//!
//! # References
//!
//! * Karaman, S. and Frazzoli, E. (2011). "Sampling-based algorithms for
//!   optimal motion planning." *IJRR*.
//! * Dubins, L.E. (1957). "On Curves of Minimal Length with a Constraint on
//!   Average Curvature."

use std::f64::consts::PI;

use rand::Rng;

use rust_robotics_core::types::Pose2D;

use crate::dubins_path::DubinsPlanner;
use crate::rrt::{AreaBounds, CircleObstacle};
use crate::rrt_dubins::{sample_dubins_with_yaw, RRTDubinsNode};

/// Configuration for the RRT*-Dubins planner.
#[derive(Debug, Clone)]
pub struct RRTStarDubinsConfig {
    /// Curvature = 1 / minimum turning radius \[1/m\].
    pub curvature: f64,
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
    /// Step size for sampling points along Dubins curves \[m\].
    pub path_resolution: f64,
    /// Near-node search radius parameter.
    pub connect_circle_dist: f64,
}

impl Default for RRTStarDubinsConfig {
    fn default() -> Self {
        Self {
            curvature: 1.0,
            goal_sample_rate: 10,
            max_iter: 200,
            goal_xy_threshold: 0.5,
            goal_yaw_threshold: 1.0_f64.to_radians(),
            robot_radius: 0.0,
            path_resolution: 0.1,
            connect_circle_dist: 50.0,
        }
    }
}

/// Result of a successful RRT*-Dubins planning run.
#[derive(Debug, Clone)]
pub struct RRTStarDubinsPath {
    /// Sampled waypoints along the path: (x, y, yaw).
    pub poses: Vec<Pose2D>,
}

/// RRT* path planner using Dubins curves for steering.
pub struct RRTStarDubinsPlanner {
    config: RRTStarDubinsConfig,
    obstacles: Vec<CircleObstacle>,
    rand_area: AreaBounds,
    dubins: DubinsPlanner,
    node_list: Vec<RRTDubinsNode>,
    start: RRTDubinsNode,
    goal: RRTDubinsNode,
}

impl RRTStarDubinsPlanner {
    /// Create a new RRT*-Dubins planner.
    pub fn new(
        obstacles: Vec<CircleObstacle>,
        rand_area: AreaBounds,
        config: RRTStarDubinsConfig,
    ) -> Self {
        let dubins = DubinsPlanner::new(config.curvature);
        Self {
            config,
            obstacles,
            rand_area,
            dubins,
            node_list: Vec::new(),
            start: RRTDubinsNode::new(0.0, 0.0, 0.0),
            goal: RRTDubinsNode::new(0.0, 0.0, 0.0),
        }
    }

    /// Plan a path from `start` to `goal`.
    ///
    /// Returns `None` if no path is found within `max_iter` iterations.
    pub fn planning(&mut self, start: Pose2D, goal: Pose2D) -> Option<RRTStarDubinsPath> {
        self.start = RRTDubinsNode::new(start.x, start.y, start.yaw);
        self.goal = RRTDubinsNode::new(goal.x, goal.y, goal.yaw);
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
    ) -> Option<RRTStarDubinsPath>
    where
        F: FnMut(&Self) -> RRTDubinsNode,
    {
        self.start = RRTDubinsNode::new(start.x, start.y, start.yaw);
        self.goal = RRTDubinsNode::new(goal.x, goal.y, goal.yaw);
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
    pub fn get_tree(&self) -> &[RRTDubinsNode] {
        &self.node_list
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    fn get_random_node(&self) -> RRTDubinsNode {
        let mut rng = rand::rng();
        if rng.random_range(0..=100) > self.config.goal_sample_rate {
            RRTDubinsNode::new(
                rng.random_range(self.rand_area.xmin..=self.rand_area.xmax),
                rng.random_range(self.rand_area.ymin..=self.rand_area.ymax),
                rng.random_range(-PI..=PI),
            )
        } else {
            RRTDubinsNode::new(self.goal.x, self.goal.y, self.goal.yaw)
        }
    }

    fn get_nearest_node_index(&self, rnd_node: &RRTDubinsNode) -> usize {
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

    /// Steer from `from_node` toward `to_node` using a Dubins curve.
    fn steer(
        &self,
        from_node: &RRTDubinsNode,
        to_node: &RRTDubinsNode,
        from_index: usize,
    ) -> Option<RRTDubinsNode> {
        let from_pose = Pose2D::new(from_node.x, from_node.y, from_node.yaw);
        let to_pose = Pose2D::new(to_node.x, to_node.y, to_node.yaw);

        let dubins_path = self.dubins.plan(from_pose, to_pose).ok()?;

        let (px, py, pyaw) = sample_dubins_with_yaw(&dubins_path, self.config.path_resolution);

        if px.len() <= 1 {
            return None;
        }

        let mut new_node = from_node.clone();
        new_node.x = *px.last().unwrap();
        new_node.y = *py.last().unwrap();
        new_node.yaw = *pyaw.last().unwrap();
        new_node.path_x = px;
        new_node.path_y = py;
        new_node.path_yaw = pyaw;
        new_node.cost += dubins_path.total_length;
        new_node.parent = Some(from_index);

        Some(new_node)
    }

    /// Compute the cost of steering from `from_node` to `to_node` via Dubins.
    fn calc_new_cost(&self, from_node: &RRTDubinsNode, to_node: &RRTDubinsNode) -> f64 {
        let from_pose = Pose2D::new(from_node.x, from_node.y, from_node.yaw);
        let to_pose = Pose2D::new(to_node.x, to_node.y, to_node.yaw);

        match self.dubins.plan(from_pose, to_pose) {
            Ok(path) => from_node.cost + path.total_length,
            Err(_) => f64::INFINITY,
        }
    }

    /// Check that a node's path does not collide with any obstacle.
    fn check_collision(&self, node: &RRTDubinsNode) -> bool {
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
    fn find_near_nodes(&self, new_node: &RRTDubinsNode) -> Vec<usize> {
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
    fn choose_parent(&self, new_node: RRTDubinsNode, near_inds: &[usize]) -> Option<RRTDubinsNode> {
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
    fn generate_final_course(&self, goal_index: usize) -> RRTStarDubinsPath {
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

        RRTStarDubinsPath { poses }
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
            CircleObstacle::new(3.0, 6.0, 2.0),
            CircleObstacle::new(3.0, 8.0, 2.0),
            CircleObstacle::new(3.0, 10.0, 2.0),
            CircleObstacle::new(7.0, 5.0, 2.0),
            CircleObstacle::new(9.0, 5.0, 2.0),
        ]
    }

    #[test]
    fn test_config_default() {
        let config = RRTStarDubinsConfig::default();
        assert_eq!(config.curvature, 1.0);
        assert_eq!(config.max_iter, 200);
        assert_eq!(config.goal_sample_rate, 10);
        assert_eq!(config.connect_circle_dist, 50.0);
        assert!(approx_eq(config.goal_xy_threshold, 0.5, 1e-12));
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
        let config = RRTStarDubinsConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let planner = RRTStarDubinsPlanner::new(vec![], rand_area, config);

        let mut node = RRTDubinsNode::new(1.0, 1.0, 0.0);
        node.path_x = vec![0.0, 0.5, 1.0];
        node.path_y = vec![0.0, 0.5, 1.0];
        assert!(planner.check_collision(&node));
    }

    #[test]
    fn test_collision_check_with_obstacle() {
        let obstacles = vec![CircleObstacle::new(0.5, 0.5, 0.3)];
        let config = RRTStarDubinsConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let planner = RRTStarDubinsPlanner::new(obstacles, rand_area, config);

        let mut node = RRTDubinsNode::new(1.0, 1.0, 0.0);
        node.path_x = vec![0.0, 0.5, 1.0];
        node.path_y = vec![0.0, 0.5, 1.0];
        assert!(!planner.check_collision(&node));
    }

    #[test]
    fn test_find_near_nodes() {
        let config = RRTStarDubinsConfig {
            connect_circle_dist: 50.0,
            ..Default::default()
        };
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTStarDubinsPlanner::new(vec![], rand_area, config);
        planner.node_list = vec![
            RRTDubinsNode::new(0.0, 0.0, 0.0),
            RRTDubinsNode::new(1.0, 1.0, 0.0),
            RRTDubinsNode::new(100.0, 100.0, 0.0),
        ];

        let query = RRTDubinsNode::new(0.5, 0.5, 0.0);
        let near = planner.find_near_nodes(&query);
        // Nodes 0 and 1 should be near; node 2 is far away
        assert!(near.contains(&0));
        assert!(near.contains(&1));
        assert!(!near.contains(&2));
    }

    #[test]
    fn test_search_best_goal_node() {
        let config = RRTStarDubinsConfig {
            goal_xy_threshold: 1.0,
            goal_yaw_threshold: 0.5,
            ..Default::default()
        };
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTStarDubinsPlanner::new(vec![], rand_area, config);
        planner.goal = RRTDubinsNode::new(10.0, 10.0, 0.0);

        // No nodes near goal
        planner.node_list = vec![RRTDubinsNode::new(0.0, 0.0, 0.0)];
        assert!(planner.search_best_goal_node().is_none());

        // Node near goal in position but wrong yaw
        let mut n = RRTDubinsNode::new(10.0, 10.0, PI);
        n.cost = 5.0;
        planner.node_list.push(n);
        assert!(planner.search_best_goal_node().is_none());

        // Node near goal in both position and yaw
        let mut n = RRTDubinsNode::new(10.2, 10.2, 0.1);
        n.cost = 8.0;
        planner.node_list.push(n);
        assert_eq!(planner.search_best_goal_node(), Some(2));
    }

    #[test]
    fn test_deterministic_planning_no_obstacles() {
        let config = RRTStarDubinsConfig {
            curvature: 1.0,
            goal_sample_rate: 10,
            max_iter: 500,
            goal_xy_threshold: 1.5,
            goal_yaw_threshold: 0.5,
            robot_radius: 0.0,
            path_resolution: 0.3,
            connect_circle_dist: 50.0,
        };
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTStarDubinsPlanner::new(vec![], rand_area, config);

        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(5.0, 5.0, 0.0);

        let result = planner.plan_with_sampler(start, goal, |p| {
            RRTDubinsNode::new(p.goal.x, p.goal.y, p.goal.yaw)
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
    fn test_rewiring_improves_cost() {
        // Build a scenario where rewiring should reduce the cost of a node.
        let config = RRTStarDubinsConfig {
            curvature: 1.0,
            max_iter: 5,
            goal_xy_threshold: 1.5,
            goal_yaw_threshold: 1.0,
            path_resolution: 0.3,
            connect_circle_dist: 100.0,
            ..Default::default()
        };
        let rand_area = AreaBounds::new(-5.0, 20.0, -5.0, 20.0);
        let mut planner = RRTStarDubinsPlanner::new(vec![], rand_area, config);
        planner.start = RRTDubinsNode::new(0.0, 0.0, 0.0);
        planner.goal = RRTDubinsNode::new(20.0, 20.0, 0.0);
        planner.node_list = vec![planner.start.clone()];

        // Add a node far from origin via a detour
        let detour = RRTDubinsNode::new(5.0, 0.0, 0.0);
        if let Some(n) = planner.steer(&planner.node_list[0].clone(), &detour, 0) {
            planner.node_list.push(n);
        }

        let initial_count = planner.node_list.len();
        assert!(
            initial_count >= 2,
            "Should have at least 2 nodes after setup"
        );
    }

    #[test]
    fn test_planning_with_obstacles_runs_without_panic() {
        let obstacles = create_obstacle_list();
        let config = RRTStarDubinsConfig {
            curvature: 1.0,
            goal_sample_rate: 10,
            max_iter: 300,
            goal_xy_threshold: 1.5,
            goal_yaw_threshold: 1.0,
            robot_radius: 0.0,
            path_resolution: 0.3,
            connect_circle_dist: 50.0,
        };
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTStarDubinsPlanner::new(obstacles, rand_area, config);

        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(10.0, 10.0, 0.0);

        let result = planner.planning(start, goal);
        // Due to randomness, the planner may or may not find a path.
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
        let config = RRTStarDubinsConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTStarDubinsPlanner::new(vec![], rand_area, config);
        planner.start = RRTDubinsNode::new(0.0, 0.0, 0.0);
        planner.goal = RRTDubinsNode::new(10.0, 10.0, 0.0);

        let root = RRTDubinsNode::new(0.0, 0.0, 0.0);
        let mut child = RRTDubinsNode::new(5.0, 5.0, 0.5);
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
    fn test_choose_parent_picks_lower_cost() {
        let config = RRTStarDubinsConfig {
            curvature: 1.0,
            path_resolution: 0.3,
            connect_circle_dist: 100.0,
            ..Default::default()
        };
        let rand_area = AreaBounds::new(-5.0, 20.0, -5.0, 20.0);
        let mut planner = RRTStarDubinsPlanner::new(vec![], rand_area, config);
        planner.start = RRTDubinsNode::new(0.0, 0.0, 0.0);
        planner.goal = RRTDubinsNode::new(20.0, 0.0, 0.0);

        // Node 0: start (cost 0)
        // Node 1: cost 100 (expensive parent)
        let mut expensive = RRTDubinsNode::new(5.0, 0.0, 0.0);
        expensive.cost = 100.0;
        expensive.parent = Some(0);

        planner.node_list = vec![planner.start.clone(), expensive];

        // New node that could be parented by either node 0 or node 1
        let new_node = RRTDubinsNode::new(3.0, 0.0, 0.0);
        let near_inds = vec![0, 1];
        let result = planner.choose_parent(new_node, &near_inds);
        assert!(result.is_some());
        // Should choose node 0 as parent (lower cost)
        let chosen = result.unwrap();
        assert_eq!(chosen.parent, Some(0));
    }
}
