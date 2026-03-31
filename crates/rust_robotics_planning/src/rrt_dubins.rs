//! RRT-Dubins path planner
//!
//! Combines the RRT (Rapidly-exploring Random Tree) algorithm with Dubins
//! curve connections. Instead of straight-line steering between nodes, this
//! planner uses Dubins paths that respect a minimum turning radius constraint.
//!
//! Each node carries a heading angle (x, y, yaw), and collision checking is
//! performed along the sampled Dubins curve.
//!
//! # References
//!
//! * LaValle, S.M. (1998). "Rapidly-exploring random trees: A new tool for
//!   path planning."
//! * Dubins, L.E. (1957). "On Curves of Minimal Length with a Constraint on
//!   Average Curvature."

use std::f64::consts::PI;

use rand::Rng;

use rust_robotics_core::types::Pose2D;

use crate::dubins_path::DubinsPlanner;
use crate::rrt::{AreaBounds, CircleObstacle};

/// A node in the RRT-Dubins tree, carrying position and heading.
#[derive(Debug, Clone)]
pub struct RRTDubinsNode {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub path_x: Vec<f64>,
    pub path_y: Vec<f64>,
    pub path_yaw: Vec<f64>,
    pub cost: f64,
    pub parent: Option<usize>,
}

impl RRTDubinsNode {
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

/// Configuration for the RRT-Dubins planner.
#[derive(Debug, Clone)]
pub struct RRTDubinsConfig {
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
}

impl Default for RRTDubinsConfig {
    fn default() -> Self {
        Self {
            curvature: 1.0,
            goal_sample_rate: 10,
            max_iter: 200,
            goal_xy_threshold: 0.5,
            goal_yaw_threshold: 1.0_f64.to_radians(),
            robot_radius: 0.0,
            path_resolution: 0.1,
        }
    }
}

/// Result of a successful RRT-Dubins planning run.
#[derive(Debug, Clone)]
pub struct RRTDubinsPath {
    /// Sampled waypoints along the path: (x, y, yaw).
    pub poses: Vec<Pose2D>,
}

/// RRT path planner using Dubins curves for steering.
pub struct RRTDubinsPlanner {
    config: RRTDubinsConfig,
    obstacles: Vec<CircleObstacle>,
    rand_area: AreaBounds,
    dubins: DubinsPlanner,
    node_list: Vec<RRTDubinsNode>,
    start: RRTDubinsNode,
    goal: RRTDubinsNode,
}

impl RRTDubinsPlanner {
    /// Create a new RRT-Dubins planner.
    pub fn new(
        obstacles: Vec<CircleObstacle>,
        rand_area: AreaBounds,
        config: RRTDubinsConfig,
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
    pub fn planning(&mut self, start: Pose2D, goal: Pose2D) -> Option<RRTDubinsPath> {
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
                    self.node_list.push(new_node);

                    // Check if we reached the goal
                    if let Some(goal_idx) = self.search_best_goal_node() {
                        return Some(self.generate_final_course(goal_idx));
                    }
                }
            }
        }

        // Final attempt to find a goal node
        self.search_best_goal_node()
            .map(|idx| self.generate_final_course(idx))
    }

    /// Plan with a deterministic node sampler (for testing).
    pub fn plan_with_sampler<F>(
        &mut self,
        start: Pose2D,
        goal: Pose2D,
        mut sample_node: F,
    ) -> Option<RRTDubinsPath>
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
                    self.node_list.push(new_node);
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

    fn get_random_node(&self) -> RRTDubinsNode {
        let mut rng = rand::thread_rng();
        if rng.gen_range(0..=100) > self.config.goal_sample_rate {
            RRTDubinsNode::new(
                rng.gen_range(self.rand_area.xmin..=self.rand_area.xmax),
                rng.gen_range(self.rand_area.ymin..=self.rand_area.ymax),
                rng.gen_range(-PI..=PI),
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
    ///
    /// `from_index` is the index of `from_node` in `self.node_list`, used to
    /// set the parent pointer on the resulting node.
    ///
    /// Returns `None` if no valid Dubins path exists.
    fn steer(
        &self,
        from_node: &RRTDubinsNode,
        to_node: &RRTDubinsNode,
        from_index: usize,
    ) -> Option<RRTDubinsNode> {
        let from_pose = Pose2D::new(from_node.x, from_node.y, from_node.yaw);
        let to_pose = Pose2D::new(to_node.x, to_node.y, to_node.yaw);

        let dubins_path = self.dubins.plan(from_pose, to_pose).ok()?;

        // Sample points along the Dubins path, including yaw
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

    /// Check that a node's path does not collide with any obstacle.
    ///
    /// Returns `true` if collision-free.
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

    /// Search for the best (lowest-cost) node that is close enough to the goal
    /// in both position and heading.
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

        // Return the candidate with minimum cost
        candidates.into_iter().min_by(|&a, &b| {
            self.node_list[a]
                .cost
                .partial_cmp(&self.node_list[b].cost)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
    }

    /// Trace back from a goal node to the start, collecting all waypoints.
    fn generate_final_course(&self, goal_index: usize) -> RRTDubinsPath {
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

        RRTDubinsPath { poses }
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

/// Sample a Dubins path returning separate x, y, yaw vectors.
///
/// This mirrors `DubinsPlanner::sample_path` but also tracks the yaw at each
/// sample point.
pub fn sample_dubins_with_yaw(
    path: &crate::dubins_path::DubinsPath,
    step: f64,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    use crate::dubins_path::SegmentType;
    use rust_robotics_core::types::Point2D;

    let step = if step <= 0.0 { 0.1 } else { step };
    let segments = path.path_type.segments();

    let mut xs = vec![path.start.x];
    let mut ys = vec![path.start.y];
    let mut yaws = vec![path.start.yaw];

    // Track the cumulative yaw at the start of each segment
    let mut current_yaw = path.start.yaw;

    for (i, &seg) in segments.iter().enumerate() {
        let seg_len = path.lengths[i] * path.curvature; // normalised length
        if seg_len.abs() < 1e-12 {
            continue;
        }

        let origin = Point2D::new(*xs.last().unwrap(), *ys.last().unwrap());
        let origin_yaw = current_yaw;

        let mut cl = step;
        while (cl + step).abs() <= seg_len.abs() {
            let (x, y, yaw) = interpolate_segment(cl, seg, path.curvature, origin, origin_yaw);
            xs.push(x);
            ys.push(y);
            yaws.push(yaw);
            cl += step;
        }

        // End of segment
        let (x, y, yaw) = interpolate_segment(seg_len, seg, path.curvature, origin, origin_yaw);
        xs.push(x);
        ys.push(y);
        yaws.push(yaw);

        // Update current_yaw for the next segment
        current_yaw = match seg {
            SegmentType::Straight => current_yaw,
            SegmentType::Left => current_yaw + seg_len,
            SegmentType::Right => current_yaw - seg_len,
        };
    }

    (xs, ys, yaws)
}

/// Interpolate a point at a given normalised arc-length within one segment.
///
/// Duplicated from `dubins_path` module internals since those are private.
fn interpolate_segment(
    length: f64,
    seg: crate::dubins_path::SegmentType,
    curvature: f64,
    origin: rust_robotics_core::types::Point2D,
    origin_yaw: f64,
) -> (f64, f64, f64) {
    use crate::dubins_path::SegmentType;

    match seg {
        SegmentType::Straight => {
            let nx = origin.x + length / curvature * origin_yaw.cos();
            let ny = origin.y + length / curvature * origin_yaw.sin();
            (nx, ny, origin_yaw)
        }
        SegmentType::Left => {
            let r = 1.0 / curvature;
            let ldx = length.sin() * r;
            let ldy = (1.0 - length.cos()) * r;
            let gdx = origin_yaw.cos() * ldx - origin_yaw.sin() * ldy;
            let gdy = origin_yaw.sin() * ldx + origin_yaw.cos() * ldy;
            (origin.x + gdx, origin.y + gdy, origin_yaw + length)
        }
        SegmentType::Right => {
            let r = 1.0 / curvature;
            let ldx = length.sin() * r;
            let ldy = -(1.0 - length.cos()) * r;
            let gdx = origin_yaw.cos() * ldx - origin_yaw.sin() * ldy;
            let gdy = origin_yaw.sin() * ldx + origin_yaw.cos() * ldy;
            (origin.x + gdx, origin.y + gdy, origin_yaw - length)
        }
    }
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
    fn test_rrt_dubins_node_creation() {
        let node = RRTDubinsNode::new(1.0, 2.0, 0.5);
        assert_eq!(node.x, 1.0);
        assert_eq!(node.y, 2.0);
        assert_eq!(node.yaw, 0.5);
        assert_eq!(node.cost, 0.0);
        assert!(node.parent.is_none());
        assert!(node.path_x.is_empty());
    }

    #[test]
    fn test_rrt_dubins_config_default() {
        let config = RRTDubinsConfig::default();
        assert_eq!(config.curvature, 1.0);
        assert_eq!(config.max_iter, 200);
        assert_eq!(config.goal_sample_rate, 10);
        assert!(approx_eq(config.goal_xy_threshold, 0.5, 1e-12));
    }

    #[test]
    fn test_angle_diff() {
        assert!(approx_eq(angle_diff(0.0, 0.0), 0.0, 1e-12));
        assert!(approx_eq(angle_diff(PI, 0.0), PI, 1e-12));
        assert!(approx_eq(angle_diff(0.0, PI), -PI, 1e-12));
        // Wrap-around
        assert!(approx_eq(angle_diff(3.0 * PI / 2.0, -PI / 2.0), 0.0, 1e-12));
    }

    #[test]
    fn test_collision_check_no_obstacles() {
        let obstacles = vec![];
        let config = RRTDubinsConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let planner = RRTDubinsPlanner::new(obstacles, rand_area, config);

        let mut node = RRTDubinsNode::new(1.0, 1.0, 0.0);
        node.path_x = vec![0.0, 0.5, 1.0];
        node.path_y = vec![0.0, 0.5, 1.0];
        assert!(planner.check_collision(&node));
    }

    #[test]
    fn test_collision_check_with_obstacle() {
        let obstacles = vec![CircleObstacle::new(0.5, 0.5, 0.3)];
        let config = RRTDubinsConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let planner = RRTDubinsPlanner::new(obstacles, rand_area, config);

        let mut node = RRTDubinsNode::new(1.0, 1.0, 0.0);
        node.path_x = vec![0.0, 0.5, 1.0];
        node.path_y = vec![0.0, 0.5, 1.0];
        // The path passes through (0.5, 0.5), right on the obstacle center
        assert!(!planner.check_collision(&node));
    }

    #[test]
    fn test_steer_produces_dubins_path() {
        let obstacles = vec![];
        let config = RRTDubinsConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTDubinsPlanner::new(obstacles, rand_area, config);
        planner.start = RRTDubinsNode::new(0.0, 0.0, 0.0);
        planner.goal = RRTDubinsNode::new(10.0, 10.0, 0.0);
        planner.node_list = vec![planner.start.clone()];

        let from_node = RRTDubinsNode::new(0.0, 0.0, 0.0);
        let to_node = RRTDubinsNode::new(5.0, 5.0, PI / 2.0);

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
    fn test_steer_endpoint_matches_dubins_goal() {
        let obstacles = vec![];
        let config = RRTDubinsConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTDubinsPlanner::new(obstacles, rand_area, config);
        planner.start = RRTDubinsNode::new(0.0, 0.0, 0.0);
        planner.goal = RRTDubinsNode::new(10.0, 10.0, 0.0);
        planner.node_list = vec![planner.start.clone()];

        let from = RRTDubinsNode::new(0.0, 0.0, 0.0);
        let to = RRTDubinsNode::new(5.0, 0.0, 0.0);
        let result = planner.steer(&from, &to, 0).unwrap();

        // Endpoint should be close to the target
        assert!(approx_eq(result.x, 5.0, 0.3));
        assert!(approx_eq(result.y, 0.0, 0.3));
    }

    #[test]
    fn test_nearest_node_index() {
        let obstacles = vec![];
        let config = RRTDubinsConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTDubinsPlanner::new(obstacles, rand_area, config);
        planner.node_list = vec![
            RRTDubinsNode::new(0.0, 0.0, 0.0),
            RRTDubinsNode::new(5.0, 5.0, 0.0),
            RRTDubinsNode::new(10.0, 10.0, 0.0),
        ];

        let query = RRTDubinsNode::new(4.0, 4.0, 0.0);
        assert_eq!(planner.get_nearest_node_index(&query), 1);

        let query = RRTDubinsNode::new(9.0, 9.0, 0.0);
        assert_eq!(planner.get_nearest_node_index(&query), 2);
    }

    #[test]
    fn test_search_best_goal_node() {
        let obstacles = vec![];
        let config = RRTDubinsConfig {
            goal_xy_threshold: 1.0,
            goal_yaw_threshold: 0.5,
            ..Default::default()
        };
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTDubinsPlanner::new(obstacles, rand_area, config);
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
        let obstacles = vec![];
        let config = RRTDubinsConfig {
            curvature: 1.0,
            goal_sample_rate: 10,
            max_iter: 500,
            goal_xy_threshold: 1.5,
            goal_yaw_threshold: 0.5,
            robot_radius: 0.0,
            path_resolution: 0.3,
        };
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTDubinsPlanner::new(obstacles, rand_area, config);

        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(5.0, 5.0, 0.0);

        // Feed the goal as every sample to guarantee convergence
        let result = planner.plan_with_sampler(start, goal, |p| {
            RRTDubinsNode::new(p.goal.x, p.goal.y, p.goal.yaw)
        });

        assert!(result.is_some(), "Should find a path with no obstacles");
        let path = result.unwrap();
        assert!(path.poses.len() >= 2);

        // First and last poses should be near start and goal
        let first = &path.poses[0];
        let last = &path.poses[path.poses.len() - 1];
        assert!(approx_eq(first.x, 0.0, 0.5));
        assert!(approx_eq(first.y, 0.0, 0.5));
        assert!(approx_eq(last.x, 5.0, 1.5));
        assert!(approx_eq(last.y, 5.0, 1.5));
    }

    #[test]
    fn test_planning_with_obstacles() {
        let obstacles = create_obstacle_list();
        let config = RRTDubinsConfig {
            curvature: 1.0,
            goal_sample_rate: 10,
            max_iter: 1000,
            goal_xy_threshold: 1.5,
            goal_yaw_threshold: 1.0,
            robot_radius: 0.0,
            path_resolution: 0.3,
        };
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTDubinsPlanner::new(obstacles, rand_area, config);

        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(10.0, 10.0, 0.0);

        let result = planner.planning(start, goal);
        // Due to randomness, the planner may or may not find a path.
        // We just verify it runs without panicking and returns a valid
        // structure if it succeeds.
        if let Some(path) = result {
            assert!(path.poses.len() >= 2);
            // Verify all path points are collision-free (no NaN)
            for pose in &path.poses {
                assert!(pose.x.is_finite());
                assert!(pose.y.is_finite());
                assert!(pose.yaw.is_finite());
            }
        }
    }

    #[test]
    fn test_sample_dubins_with_yaw_straight() {
        let planner = DubinsPlanner::new(1.0);
        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(5.0, 0.0, 0.0);
        let path = planner.plan(start, goal).unwrap();

        let (xs, ys, yaws) = sample_dubins_with_yaw(&path, 0.5);

        assert!(xs.len() > 2);
        assert_eq!(xs.len(), ys.len());
        assert_eq!(xs.len(), yaws.len());

        // Start point
        assert!(approx_eq(xs[0], 0.0, 1e-12));
        assert!(approx_eq(ys[0], 0.0, 1e-12));

        // End point should be near goal
        assert!(approx_eq(*xs.last().unwrap(), 5.0, 0.3));
        assert!(approx_eq(*ys.last().unwrap(), 0.0, 0.3));

        // Yaw should stay near 0 for a straight path
        for &yaw in &yaws {
            assert!(approx_eq(yaw, 0.0, 0.1));
        }
    }

    #[test]
    fn test_sample_dubins_with_yaw_turn() {
        let planner = DubinsPlanner::new(1.0);
        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(3.0, 3.0, PI / 2.0);
        let path = planner.plan(start, goal).unwrap();

        let (xs, ys, yaws) = sample_dubins_with_yaw(&path, 0.1);

        assert!(xs.len() > 2);
        // End yaw should be near PI/2
        let end_yaw = *yaws.last().unwrap();
        assert!(
            approx_eq(angle_diff(end_yaw, PI / 2.0).abs(), 0.0, 0.3),
            "end yaw {} should be near PI/2",
            end_yaw
        );
    }

    #[test]
    fn test_generate_final_course() {
        let obstacles = vec![];
        let config = RRTDubinsConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let mut planner = RRTDubinsPlanner::new(obstacles, rand_area, config);
        planner.start = RRTDubinsNode::new(0.0, 0.0, 0.0);
        planner.goal = RRTDubinsNode::new(10.0, 10.0, 0.0);

        // Build a simple two-node tree
        let root = RRTDubinsNode::new(0.0, 0.0, 0.0);
        let mut child = RRTDubinsNode::new(5.0, 5.0, 0.5);
        child.parent = Some(0);
        child.path_x = vec![0.0, 2.5, 5.0];
        child.path_y = vec![0.0, 2.5, 5.0];
        child.path_yaw = vec![0.0, 0.25, 0.5];

        planner.node_list = vec![root, child];

        let course = planner.generate_final_course(1);
        // Should include start + path points (reversed) + goal
        assert!(course.poses.len() >= 3);
        // First pose should be start
        assert!(approx_eq(course.poses[0].x, 0.0, 1e-12));
        // Last pose should be goal
        let last = course.poses.last().unwrap();
        assert!(approx_eq(last.x, 10.0, 1e-12));
        assert!(approx_eq(last.y, 10.0, 1e-12));
    }
}
