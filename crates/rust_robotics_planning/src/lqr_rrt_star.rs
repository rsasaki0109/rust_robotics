#![allow(dead_code, clippy::too_many_arguments)]

//! LQR-RRT* path planning algorithm
//!
//! Combines RRT* with LQR-based steering and cost computation.
//! Instead of straight-line extension, the tree is grown using
//! LQR trajectories, and the cost metric is the sum of LQR
//! trajectory segment lengths.
//!
//! Reference: PythonRobotics LQRRRTStar by Atsushi Sakai

use rand::Rng;

use crate::lqr_planner::{LqrPlanner, LqrPlannerConfig};
use rust_robotics_core::{Path2D, Point2D, RoboticsError, RoboticsResult};

/// A node in the LQR-RRT* tree.
#[derive(Debug, Clone)]
pub struct Node {
    pub x: f64,
    pub y: f64,
    pub path_x: Vec<f64>,
    pub path_y: Vec<f64>,
    pub cost: f64,
    pub parent: Option<usize>,
}

impl Node {
    pub fn new(x: f64, y: f64) -> Self {
        Node {
            x,
            y,
            path_x: Vec::new(),
            path_y: Vec::new(),
            cost: 0.0,
            parent: None,
        }
    }
}

/// Configuration for the LQR-RRT* planner.
#[derive(Debug, Clone)]
pub struct LqrRrtStarConfig {
    /// Random sampling area minimum bound
    pub min_rand: f64,
    /// Random sampling area maximum bound
    pub max_rand: f64,
    /// Goal sampling rate (0-100). Higher means more goal-biased sampling.
    pub goal_sample_rate: i32,
    /// Maximum number of iterations
    pub max_iter: i32,
    /// Connection circle distance for finding near nodes
    pub connect_circle_dist: f64,
    /// Step size for path resampling
    pub step_size: f64,
    /// Goal proximity threshold
    pub goal_xy_th: f64,
    /// Robot radius for collision checking
    pub robot_radius: f64,
    /// Whether to continue searching until max iterations
    pub search_until_max_iter: bool,
    /// Configuration for the internal LQR planner
    pub lqr_config: LqrPlannerConfig,
}

impl Default for LqrRrtStarConfig {
    fn default() -> Self {
        Self {
            min_rand: -2.0,
            max_rand: 15.0,
            goal_sample_rate: 10,
            max_iter: 200,
            connect_circle_dist: 50.0,
            step_size: 0.2,
            goal_xy_th: 0.5,
            robot_radius: 0.0,
            search_until_max_iter: true,
            lqr_config: LqrPlannerConfig::default(),
        }
    }
}

/// LQR-RRT* planner.
///
/// Uses LQR trajectories for tree steering and LQR path length
/// as the cost metric, combined with the RRT* rewiring strategy.
pub struct LqrRrtStar {
    pub start: Node,
    pub end: Node,
    pub config: LqrRrtStarConfig,
    pub obstacle_list: Vec<(f64, f64, f64)>,
    pub node_list: Vec<Node>,
    lqr_planner: LqrPlanner,
}

impl LqrRrtStar {
    /// Create a new LQR-RRT* planner.
    pub fn new(
        start: (f64, f64),
        goal: (f64, f64),
        obstacle_list: Vec<(f64, f64, f64)>,
        config: LqrRrtStarConfig,
    ) -> Self {
        let lqr_planner = LqrPlanner::new(config.lqr_config.clone());
        LqrRrtStar {
            start: Node::new(start.0, start.1),
            end: Node::new(goal.0, goal.1),
            config,
            obstacle_list,
            node_list: Vec::new(),
            lqr_planner,
        }
    }

    /// Run the LQR-RRT* planning algorithm.
    ///
    /// Returns a path as a vector of \[x, y\] pairs (goal to start order),
    /// or `None` if no path is found.
    pub fn planning(&mut self) -> Option<Vec<[f64; 2]>> {
        self.planning_with_sampler(|planner| planner.get_random_node())
    }

    /// Run planning with a custom sampling function (useful for testing).
    fn planning_with_sampler<F>(&mut self, mut sample_node: F) -> Option<Vec<[f64; 2]>>
    where
        F: FnMut(&LqrRrtStar) -> Node,
    {
        self.node_list = vec![self.start.clone()];

        for _i in 0..self.config.max_iter {
            let rnd = sample_node(self);
            let nearest_ind = self.get_nearest_node_index(&rnd);
            let new_node = self.steer(&self.node_list[nearest_ind].clone(), &rnd);

            if let Some(node) = new_node {
                if self.check_collision_free(&node) {
                    let near_inds = self.find_near_nodes(&node);
                    let chosen = self.choose_parent(node, &near_inds);

                    if let Some(new_node) = chosen {
                        let new_index = self.node_list.len();
                        self.node_list.push(new_node);
                        self.rewire(new_index, &near_inds);
                    }
                }
            }

            if !self.config.search_until_max_iter {
                if let Some(last_index) = self.search_best_goal_node() {
                    return Some(self.generate_final_course(last_index));
                }
            }
        }

        if let Some(last_index) = self.search_best_goal_node() {
            return Some(self.generate_final_course(last_index));
        }

        None
    }

    /// Sample a random node, with goal bias.
    fn get_random_node(&self) -> Node {
        let mut rng = rand::thread_rng();
        if rng.gen_range(0..=100) > self.config.goal_sample_rate {
            Node::new(
                rng.gen_range(self.config.min_rand..=self.config.max_rand),
                rng.gen_range(self.config.min_rand..=self.config.max_rand),
            )
        } else {
            Node::new(self.end.x, self.end.y)
        }
    }

    /// Find the index of the nearest node to `rnd_node` using Euclidean distance.
    fn get_nearest_node_index(&self, rnd_node: &Node) -> usize {
        let mut min_dist = f64::INFINITY;
        let mut nearest_ind = 0;
        for (i, node) in self.node_list.iter().enumerate() {
            let dist = self.euclidean_distance(node, rnd_node);
            if dist < min_dist {
                min_dist = dist;
                nearest_ind = i;
            }
        }
        nearest_ind
    }

    /// Steer from `from_node` towards `to_node` using LQR trajectory.
    ///
    /// Returns a new node at the end of the LQR path, or `None` if
    /// the LQR planner fails.
    fn steer(&self, from_node: &Node, to_node: &Node) -> Option<Node> {
        let (wx, wy) = self
            .lqr_planner
            .planning(from_node.x, from_node.y, to_node.x, to_node.y)
            .ok()?;

        let (px, py, course_lens) = self.sample_path(&wx, &wy, self.config.step_size);

        if px.is_empty() {
            return None;
        }

        let mut new_node = from_node.clone();
        new_node.x = *px.last().unwrap();
        new_node.y = *py.last().unwrap();
        new_node.path_x = px;
        new_node.path_y = py;
        new_node.cost += course_lens.iter().map(|c| c.abs()).sum::<f64>();
        // parent will be set by choose_parent or the caller
        new_node.parent = None;

        Some(new_node)
    }

    /// Resample an LQR trajectory at the given step size.
    ///
    /// Returns resampled (px, py) and segment lengths.
    fn sample_path(&self, wx: &[f64], wy: &[f64], step: f64) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let mut px = Vec::new();
        let mut py = Vec::new();

        for i in 0..wx.len().saturating_sub(1) {
            let mut t = 0.0;
            while t < 1.0 {
                px.push(t * wx[i + 1] + (1.0 - t) * wx[i]);
                py.push(t * wy[i + 1] + (1.0 - t) * wy[i]);
                t += step;
            }
        }

        if px.len() < 2 {
            return (px, py, Vec::new());
        }

        let clen: Vec<f64> = px
            .windows(2)
            .zip(py.windows(2))
            .map(|(xw, yw)| {
                let dx = xw[1] - xw[0];
                let dy = yw[1] - yw[0];
                (dx * dx + dy * dy).sqrt()
            })
            .collect();

        (px, py, clen)
    }

    /// Compute the cost of an LQR path from `from_node` to `to_node`.
    fn calc_new_cost(&self, from_node: &Node, to_node: &Node) -> f64 {
        let result = self
            .lqr_planner
            .planning(from_node.x, from_node.y, to_node.x, to_node.y);

        let (wx, wy) = match result {
            Ok(v) => v,
            Err(_) => return f64::INFINITY,
        };

        let (_px, _py, course_lens) = self.sample_path(&wx, &wy, self.config.step_size);

        if course_lens.is_empty() {
            return f64::INFINITY;
        }

        from_node.cost + course_lens.iter().sum::<f64>()
    }

    /// Check if a node's path is collision-free against all obstacles.
    fn check_collision_free(&self, node: &Node) -> bool {
        if node.path_x.is_empty() || node.path_y.is_empty() {
            return true;
        }
        for &(ox, oy, size) in &self.obstacle_list {
            for (&px, &py) in node.path_x.iter().zip(node.path_y.iter()) {
                let d = (px - ox).powi(2) + (py - oy).powi(2);
                if d <= (size + self.config.robot_radius).powi(2) {
                    return false;
                }
            }
        }
        true
    }

    /// Find all nodes within the connection radius.
    fn find_near_nodes(&self, new_node: &Node) -> Vec<usize> {
        let nnode = self.node_list.len() + 1;
        let r = self.config.connect_circle_dist * ((nnode as f64).ln() / nnode as f64).sqrt();

        self.node_list
            .iter()
            .enumerate()
            .filter_map(|(i, node)| {
                let dist_sq = (node.x - new_node.x).powi(2) + (node.y - new_node.y).powi(2);
                if dist_sq <= r.powi(2) {
                    Some(i)
                } else {
                    None
                }
            })
            .collect()
    }

    /// Choose the best parent for `new_node` from the near nodes,
    /// using LQR cost as the metric.
    fn choose_parent(&self, mut new_node: Node, near_inds: &[usize]) -> Option<Node> {
        if near_inds.is_empty() {
            return None;
        }

        let mut best_cost = f64::INFINITY;
        let mut best_ind: Option<usize> = None;

        for &i in near_inds {
            let near_node = &self.node_list[i];
            let cost = self.calc_new_cost(near_node, &new_node);

            // Build a temporary node with the LQR path to check collision
            if cost < best_cost {
                let t_node = self.steer(near_node, &new_node);
                if let Some(ref tn) = t_node {
                    if self.check_collision_free(tn) {
                        best_cost = cost;
                        best_ind = Some(i);
                    }
                }
            }
        }

        let parent_ind = best_ind?;

        // Re-steer from the best parent to get the actual path
        let near_node = &self.node_list[parent_ind].clone();
        if let Some(mut result_node) = self.steer(near_node, &new_node) {
            result_node.cost = best_cost;
            result_node.parent = Some(parent_ind);
            Some(result_node)
        } else {
            // Fallback: use the new_node as-is with the best parent
            new_node.cost = best_cost;
            new_node.parent = Some(parent_ind);
            Some(new_node)
        }
    }

    /// Rewire the tree: check if routing through `new_node_ind` is cheaper
    /// for any of the `near_inds` nodes.
    fn rewire(&mut self, new_node_ind: usize, near_inds: &[usize]) {
        for &i in near_inds {
            let near_node = self.node_list[i].clone();
            let new_node = self.node_list[new_node_ind].clone();

            let edge_cost = self.calc_new_cost(&new_node, &near_node);

            if edge_cost < near_node.cost {
                if let Some(mut edge_node) = self.steer(&new_node, &near_node) {
                    edge_node.cost = edge_cost;
                    edge_node.parent = Some(new_node_ind);

                    if self.check_collision_free(&edge_node) {
                        self.node_list[i] = edge_node;
                        self.propagate_cost_to_leaves(i);
                    }
                }
            }
        }
    }

    /// Propagate cost updates from a parent to all descendants.
    fn propagate_cost_to_leaves(&mut self, parent_ind: usize) {
        for i in 0..self.node_list.len() {
            if let Some(node_parent) = self.node_list[i].parent {
                if node_parent == parent_ind {
                    let parent = self.node_list[parent_ind].clone();
                    self.node_list[i].cost =
                        self.calc_new_cost(&parent, &self.node_list[i].clone());
                    self.propagate_cost_to_leaves(i);
                }
            }
        }
    }

    /// Search for the best goal node within `goal_xy_th` distance.
    fn search_best_goal_node(&self) -> Option<usize> {
        let dist_to_goal_list: Vec<f64> = self
            .node_list
            .iter()
            .map(|n| self.calc_dist_to_goal(n.x, n.y))
            .collect();

        let goal_inds: Vec<usize> = dist_to_goal_list
            .iter()
            .enumerate()
            .filter_map(|(i, &dist)| {
                if dist <= self.config.goal_xy_th {
                    Some(i)
                } else {
                    None
                }
            })
            .collect();

        if goal_inds.is_empty() {
            return None;
        }

        let mut min_cost = f64::INFINITY;
        let mut best_ind = None;
        for &i in &goal_inds {
            if self.node_list[i].cost < min_cost {
                min_cost = self.node_list[i].cost;
                best_ind = Some(i);
            }
        }

        best_ind
    }

    /// Generate the final path by tracing from goal node back to start.
    ///
    /// Returns path segments in reverse (goal to start), including
    /// the full LQR sub-paths stored at each node.
    fn generate_final_course(&self, goal_index: usize) -> Vec<[f64; 2]> {
        let mut path = vec![[self.end.x, self.end.y]];
        let mut node = &self.node_list[goal_index];

        while let Some(parent_ind) = node.parent {
            for (&ix, &iy) in node.path_x.iter().rev().zip(node.path_y.iter().rev()) {
                path.push([ix, iy]);
            }
            node = &self.node_list[parent_ind];
        }
        path.push([self.start.x, self.start.y]);

        path
    }

    fn calc_dist_to_goal(&self, x: f64, y: f64) -> f64 {
        let dx = x - self.end.x;
        let dy = y - self.end.y;
        (dx * dx + dy * dy).sqrt()
    }

    fn euclidean_distance(&self, a: &Node, b: &Node) -> f64 {
        let dx = a.x - b.x;
        let dy = a.y - b.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Plan a path from `start` to `goal`, returning a [`Path2D`].
    pub fn plan_from(&mut self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        self.start = Node::new(start.x, start.y);
        self.end = Node::new(goal.x, goal.y);

        self.planning()
            .map(|raw_path| {
                Path2D::from_points(
                    raw_path
                        .into_iter()
                        .rev()
                        .map(|p| Point2D::new(p[0], p[1]))
                        .collect(),
                )
            })
            .ok_or_else(|| {
                RoboticsError::PlanningError(
                    "LQR-RRT*: Cannot find path within max iterations".to_string(),
                )
            })
    }

    /// Get the tree nodes for external inspection.
    pub fn get_tree(&self) -> &[Node] {
        &self.node_list
    }

    /// Get the obstacle list.
    pub fn get_obstacles(&self) -> &[(f64, f64, f64)] {
        &self.obstacle_list
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_obstacle_list() -> Vec<(f64, f64, f64)> {
        vec![
            (5.0, 5.0, 1.0),
            (4.0, 6.0, 1.0),
            (4.0, 7.5, 1.0),
            (4.0, 9.0, 1.0),
            (6.0, 5.0, 1.0),
            (7.0, 5.0, 1.0),
        ]
    }

    fn default_config() -> LqrRrtStarConfig {
        LqrRrtStarConfig {
            min_rand: -2.0,
            max_rand: 15.0,
            goal_sample_rate: 10,
            max_iter: 200,
            connect_circle_dist: 50.0,
            step_size: 0.2,
            goal_xy_th: 0.5,
            robot_radius: 0.0,
            search_until_max_iter: true,
            lqr_config: LqrPlannerConfig::default(),
        }
    }

    #[test]
    fn test_node_creation() {
        let node = Node::new(1.0, 2.0);
        assert_eq!(node.x, 1.0);
        assert_eq!(node.y, 2.0);
        assert!(node.path_x.is_empty());
        assert!(node.path_y.is_empty());
        assert_eq!(node.cost, 0.0);
        assert!(node.parent.is_none());
    }

    #[test]
    fn test_config_default() {
        let config = LqrRrtStarConfig::default();
        assert_eq!(config.min_rand, -2.0);
        assert_eq!(config.max_rand, 15.0);
        assert_eq!(config.goal_sample_rate, 10);
        assert_eq!(config.max_iter, 200);
        assert_eq!(config.step_size, 0.2);
        assert_eq!(config.goal_xy_th, 0.5);
    }

    #[test]
    fn test_sample_path_basic() {
        let config = default_config();
        let planner = LqrRrtStar::new((0.0, 0.0), (10.0, 10.0), vec![], config);

        let wx = vec![0.0, 1.0, 2.0];
        let wy = vec![0.0, 1.0, 2.0];
        let (px, py, clen) = planner.sample_path(&wx, &wy, 0.5);

        assert!(!px.is_empty());
        assert_eq!(px.len(), py.len());
        assert!(!clen.is_empty());
        // All segment lengths should be positive
        for &c in &clen {
            assert!(c >= 0.0);
        }
    }

    #[test]
    fn test_sample_path_empty() {
        let config = default_config();
        let planner = LqrRrtStar::new((0.0, 0.0), (10.0, 10.0), vec![], config);

        let (px, py, clen) = planner.sample_path(&[], &[], 0.5);
        assert!(px.is_empty());
        assert!(py.is_empty());
        assert!(clen.is_empty());
    }

    #[test]
    fn test_sample_path_single_point() {
        let config = default_config();
        let planner = LqrRrtStar::new((0.0, 0.0), (10.0, 10.0), vec![], config);

        let (px, py, clen) = planner.sample_path(&[1.0], &[2.0], 0.5);
        assert!(px.is_empty());
        assert!(py.is_empty());
        assert!(clen.is_empty());
    }

    #[test]
    fn test_lqr_steer_produces_path() {
        let config = default_config();
        let planner = LqrRrtStar::new((0.0, 0.0), (10.0, 10.0), vec![], config);

        let from = Node::new(0.0, 0.0);
        let to = Node::new(3.0, 3.0);
        let result = planner.steer(&from, &to);

        assert!(result.is_some());
        let node = result.unwrap();
        assert!(!node.path_x.is_empty());
        assert!(!node.path_y.is_empty());
        assert!(node.cost > 0.0);
    }

    #[test]
    fn test_collision_free_no_obstacles() {
        let config = default_config();
        let planner = LqrRrtStar::new((0.0, 0.0), (10.0, 10.0), vec![], config);

        let mut node = Node::new(1.0, 1.0);
        node.path_x = vec![0.0, 0.5, 1.0];
        node.path_y = vec![0.0, 0.5, 1.0];
        assert!(planner.check_collision_free(&node));
    }

    #[test]
    fn test_collision_detected() {
        let config = default_config();
        let planner = LqrRrtStar::new((0.0, 0.0), (10.0, 10.0), vec![(0.5, 0.5, 1.0)], config);

        let mut node = Node::new(1.0, 1.0);
        node.path_x = vec![0.0, 0.5, 1.0];
        node.path_y = vec![0.0, 0.5, 1.0];
        assert!(!planner.check_collision_free(&node));
    }

    #[test]
    fn test_find_near_nodes() {
        let mut config = default_config();
        config.connect_circle_dist = 100.0;
        let mut planner = LqrRrtStar::new((0.0, 0.0), (10.0, 10.0), vec![], config);

        planner.node_list = vec![
            Node::new(0.0, 0.0),
            Node::new(1.0, 0.0),
            Node::new(100.0, 100.0),
        ];

        let query = Node::new(0.5, 0.0);
        let near = planner.find_near_nodes(&query);
        // Nodes 0 and 1 should be near; node 2 is far away
        assert!(near.contains(&0));
        assert!(near.contains(&1));
    }

    #[test]
    fn test_euclidean_distance() {
        let config = default_config();
        let planner = LqrRrtStar::new((0.0, 0.0), (10.0, 10.0), vec![], config);

        let a = Node::new(0.0, 0.0);
        let b = Node::new(3.0, 4.0);
        let d = planner.euclidean_distance(&a, &b);
        assert!((d - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_calc_dist_to_goal() {
        let config = default_config();
        let planner = LqrRrtStar::new((0.0, 0.0), (10.0, 10.0), vec![], config);

        let d = planner.calc_dist_to_goal(7.0, 6.0);
        let expected = ((7.0 - 10.0_f64).powi(2) + (6.0 - 10.0_f64).powi(2)).sqrt();
        assert!((d - expected).abs() < 1e-10);
    }

    #[test]
    fn test_planning_no_obstacles() {
        let mut config = default_config();
        config.max_iter = 300;
        config.search_until_max_iter = true;
        let mut planner = LqrRrtStar::new((0.0, 0.0), (5.0, 5.0), vec![], config);

        let path = planner.planning();
        assert!(path.is_some(), "Should find a path with no obstacles");
        let path = path.unwrap();
        assert!(path.len() >= 2);
        // First point should be near goal, last near start
        let first = path.first().unwrap();
        assert!(
            (first[0] - 5.0).abs() < 1.0 && (first[1] - 5.0).abs() < 1.0,
            "Path should start near goal"
        );
    }

    #[test]
    fn test_planning_with_obstacles() {
        let mut config = default_config();
        config.max_iter = 300;
        config.search_until_max_iter = true;
        let mut planner = LqrRrtStar::new((0.0, 0.0), (6.0, 7.0), default_obstacle_list(), config);

        let path = planner.planning();
        // With randomness we cannot guarantee a path every time,
        // but we verify the structure if found.
        if let Some(path) = path {
            assert!(path.len() >= 2);
            let first = path.first().unwrap();
            assert!(
                (first[0] - 6.0).abs() < 1.0 && (first[1] - 7.0).abs() < 1.0,
                "Path should start near goal"
            );
        }
    }

    #[test]
    fn test_planning_early_termination() {
        let mut config = default_config();
        config.max_iter = 500;
        config.search_until_max_iter = false;
        let mut planner = LqrRrtStar::new((0.0, 0.0), (3.0, 3.0), vec![], config);

        let path = planner.planning();
        assert!(path.is_some(), "Should find a path with early termination");
    }

    #[test]
    fn test_plan_from_returns_path2d() {
        let mut config = default_config();
        config.max_iter = 300;
        let mut planner = LqrRrtStar::new((0.0, 0.0), (5.0, 5.0), vec![], config);

        let result = planner.plan_from(Point2D::new(0.0, 0.0), Point2D::new(5.0, 5.0));
        assert!(result.is_ok(), "plan_from should succeed with no obstacles");
        let path = result.unwrap();
        assert!(path.points.len() >= 2);
    }

    #[test]
    fn test_get_tree_and_obstacles() {
        let config = default_config();
        let obstacles = default_obstacle_list();
        let planner = LqrRrtStar::new((0.0, 0.0), (10.0, 10.0), obstacles.clone(), config);

        assert!(planner.get_tree().is_empty());
        assert_eq!(planner.get_obstacles().len(), obstacles.len());
    }

    #[test]
    fn test_search_best_goal_node_none_when_far() {
        let config = default_config();
        let mut planner = LqrRrtStar::new((0.0, 0.0), (100.0, 100.0), vec![], config);
        planner.node_list = vec![Node::new(0.0, 0.0)];

        assert!(planner.search_best_goal_node().is_none());
    }

    #[test]
    fn test_search_best_goal_node_finds_closest() {
        let mut config = default_config();
        config.goal_xy_th = 1.0;
        let mut planner = LqrRrtStar::new((0.0, 0.0), (5.0, 5.0), vec![], config);

        let mut n1 = Node::new(5.0, 5.0);
        n1.cost = 10.0;
        let mut n2 = Node::new(4.5, 4.5);
        n2.cost = 8.0;
        let n3 = Node::new(0.0, 0.0); // far from goal

        planner.node_list = vec![n3, n1, n2];

        let best = planner.search_best_goal_node();
        assert!(best.is_some());
        // Node at index 2 has cost 8.0 < 10.0
        assert_eq!(best.unwrap(), 2);
    }

    #[test]
    fn test_generate_final_course_simple() {
        let config = default_config();
        let mut planner = LqrRrtStar::new((0.0, 0.0), (5.0, 5.0), vec![], config);

        let mut start_node = Node::new(0.0, 0.0);
        start_node.parent = None;

        let mut mid_node = Node::new(2.5, 2.5);
        mid_node.parent = Some(0);
        mid_node.path_x = vec![0.5, 1.0, 1.5, 2.0, 2.5];
        mid_node.path_y = vec![0.5, 1.0, 1.5, 2.0, 2.5];

        let mut goal_node = Node::new(4.8, 4.8);
        goal_node.parent = Some(1);
        goal_node.path_x = vec![3.0, 3.5, 4.0, 4.5, 4.8];
        goal_node.path_y = vec![3.0, 3.5, 4.0, 4.5, 4.8];

        planner.node_list = vec![start_node, mid_node, goal_node];

        let path = planner.generate_final_course(2);
        // Should start with goal [5.0, 5.0], then reverse of goal_node path,
        // then reverse of mid_node path, then start [0.0, 0.0]
        assert_eq!(path.first().unwrap(), &[5.0, 5.0]);
        assert_eq!(path.last().unwrap(), &[0.0, 0.0]);
        assert!(path.len() > 2);
    }

    #[test]
    fn test_steer_same_point() {
        let config = default_config();
        let planner = LqrRrtStar::new((0.0, 0.0), (10.0, 10.0), vec![], config);

        let from = Node::new(5.0, 5.0);
        let to = Node::new(5.0, 5.0);
        // LQR from same point to same point should succeed quickly
        let result = planner.steer(&from, &to);
        // The LQR planner should handle start == goal
        if let Some(node) = result {
            assert!((node.x - 5.0).abs() < 1.0);
            assert!((node.y - 5.0).abs() < 1.0);
        }
    }

    #[test]
    fn test_seeded_deterministic_no_obstacles() {
        // Use a fixed sample sequence to verify deterministic behavior
        let config = LqrRrtStarConfig {
            max_iter: 50,
            search_until_max_iter: false,
            goal_xy_th: 0.5,
            ..default_config()
        };
        let mut planner = LqrRrtStar::new((0.0, 0.0), (3.0, 3.0), vec![], config);

        // Always sample the goal point
        let path = planner.planning_with_sampler(|p| Node::new(p.end.x, p.end.y));

        assert!(
            path.is_some(),
            "Goal-biased sampling with no obstacles should find a path"
        );
        let path = path.unwrap();
        assert!(path.len() >= 2);
        assert_eq!(path.first().unwrap(), &[3.0, 3.0]);
        assert_eq!(path.last().unwrap(), &[0.0, 0.0]);
    }
}
