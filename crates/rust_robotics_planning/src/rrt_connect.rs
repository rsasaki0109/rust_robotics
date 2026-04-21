//! RRT-Connect path planning algorithm.
//!
//! RRT-Connect aggressively grows two trees and uses a CONNECT step that keeps
//! extending the opposite tree toward a newly added node until trapped or
//! reached.

use rand::Rng;

use rust_robotics_core::{Path2D, PathPlanner, Point2D, RoboticsError};

/// Circular obstacle \(x, y, radius\).
#[derive(Debug, Clone)]
pub struct CircleObstacle {
    pub x: f64,
    pub y: f64,
    pub radius: f64,
}

impl CircleObstacle {
    pub fn new(x: f64, y: f64, radius: f64) -> Self {
        Self { x, y, radius }
    }
}

/// Axis-aligned random sampling bounds.
#[derive(Debug, Clone)]
pub struct AreaBounds {
    pub xmin: f64,
    pub xmax: f64,
    pub ymin: f64,
    pub ymax: f64,
}

impl AreaBounds {
    pub fn new(xmin: f64, xmax: f64, ymin: f64, ymax: f64) -> Self {
        Self {
            xmin,
            xmax,
            ymin,
            ymax,
        }
    }
}

/// Configuration for RRT-Connect planner.
#[derive(Debug, Clone)]
pub struct RRTConnectConfig {
    /// Maximum extension distance per tree growth step \[m\].
    pub expand_dis: f64,
    /// Interpolation distance used for collision checks \[m\].
    pub path_resolution: f64,
    /// Maximum number of planning iterations.
    pub max_iter: usize,
    /// Robot radius used during obstacle collision checks \[m\].
    pub robot_radius: f64,
}

impl Default for RRTConnectConfig {
    fn default() -> Self {
        Self {
            expand_dis: 3.0,
            path_resolution: 0.5,
            max_iter: 500,
            robot_radius: 0.8,
        }
    }
}

#[derive(Debug, Clone)]
struct RRTNode {
    x: f64,
    y: f64,
    parent: Option<usize>,
}

impl RRTNode {
    fn new(x: f64, y: f64) -> Self {
        Self { x, y, parent: None }
    }

    fn to_point(&self) -> Point2D {
        Point2D::new(self.x, self.y)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ExtendStatus {
    Advanced,
    Reached,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ConnectPolicy {
    AggressiveConnect,
    ExtendOnce,
}

/// RRT-Connect planner.
pub struct RRTConnectPlanner {
    config: RRTConnectConfig,
    obstacles: Vec<CircleObstacle>,
    rand_area: AreaBounds,
}

impl RRTConnectPlanner {
    pub fn new(
        obstacles: Vec<CircleObstacle>,
        rand_area: AreaBounds,
        config: RRTConnectConfig,
    ) -> Self {
        Self {
            config,
            obstacles,
            rand_area,
        }
    }

    fn get_random_node(&self) -> RRTNode {
        let mut rng = rand::rng();
        RRTNode::new(
            rng.random_range(self.rand_area.xmin..=self.rand_area.xmax),
            rng.random_range(self.rand_area.ymin..=self.rand_area.ymax),
        )
    }

    fn dist(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
        let dx = ax - bx;
        let dy = ay - by;
        (dx * dx + dy * dy).sqrt()
    }

    fn get_nearest_node_index(tree: &[RRTNode], target: &RRTNode) -> usize {
        tree.iter()
            .enumerate()
            .map(|(i, node)| {
                let dx = node.x - target.x;
                let dy = node.y - target.y;
                (i, dx * dx + dy * dy)
            })
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(0)
    }

    fn point_in_collision(&self, x: f64, y: f64) -> bool {
        self.obstacles
            .iter()
            .any(|obs| Self::dist(x, y, obs.x, obs.y) <= obs.radius + self.config.robot_radius)
    }

    fn steer(
        &self,
        from: &RRTNode,
        to: &RRTNode,
        parent_idx: usize,
    ) -> Option<(RRTNode, ExtendStatus)> {
        let dx = to.x - from.x;
        let dy = to.y - from.y;
        let distance = (dx * dx + dy * dy).sqrt();
        if distance < f64::EPSILON {
            return None;
        }

        let theta = dy.atan2(dx);
        let step = distance.min(self.config.expand_dis);
        let n_steps = (step / self.config.path_resolution).floor() as usize;

        let mut cx = from.x;
        let mut cy = from.y;
        for _ in 0..n_steps {
            cx += self.config.path_resolution * theta.cos();
            cy += self.config.path_resolution * theta.sin();
            if self.point_in_collision(cx, cy) {
                return None;
            }
        }

        let mut status = ExtendStatus::Advanced;
        if Self::dist(cx, cy, to.x, to.y) <= self.config.path_resolution {
            cx = to.x;
            cy = to.y;
            status = ExtendStatus::Reached;
        }
        if self.point_in_collision(cx, cy) {
            return None;
        }

        Some((
            RRTNode {
                x: cx,
                y: cy,
                parent: Some(parent_idx),
            },
            status,
        ))
    }

    fn extend_tree(
        &self,
        tree: &mut Vec<RRTNode>,
        target: &RRTNode,
    ) -> Option<(usize, ExtendStatus)> {
        let nearest_idx = Self::get_nearest_node_index(tree, target);
        let nearest = tree[nearest_idx].clone();
        let (node, status) = self.steer(&nearest, target, nearest_idx)?;
        tree.push(node);
        Some((tree.len() - 1, status))
    }

    fn connect_tree(
        &self,
        tree: &mut Vec<RRTNode>,
        target: &RRTNode,
        policy: ConnectPolicy,
    ) -> Option<(usize, ExtendStatus)> {
        match policy {
            ConnectPolicy::ExtendOnce => self.extend_tree(tree, target),
            ConnectPolicy::AggressiveConnect => {
                let mut latest = None;
                while let Some((new_idx, status)) = self.extend_tree(tree, target) {
                    latest = Some((new_idx, status));
                    if status == ExtendStatus::Reached {
                        break;
                    }
                }
                latest
            }
        }
    }

    fn trace_path(tree: &[RRTNode], idx: usize) -> Vec<Point2D> {
        let mut path = Vec::new();
        let mut current = Some(idx);
        while let Some(i) = current {
            path.push(tree[i].to_point());
            current = tree[i].parent;
        }
        path.reverse();
        path
    }

    fn reconstruct_path(
        &self,
        tree_a: &[RRTNode],
        idx_a: usize,
        tree_b: &[RRTNode],
        idx_b: usize,
        a_is_start: bool,
    ) -> Path2D {
        let mut path_a = Self::trace_path(tree_a, idx_a);
        let mut path_b = Self::trace_path(tree_b, idx_b);

        if a_is_start {
            path_b.reverse();
            path_a.extend(path_b.into_iter().skip(1));
            Path2D::from_points(path_a)
        } else {
            path_a.reverse();
            path_b.extend(path_a.into_iter().skip(1));
            Path2D::from_points(path_b)
        }
    }

    fn run_with_sampler<F>(
        &self,
        start: Point2D,
        goal: Point2D,
        policy: ConnectPolicy,
        mut sample: F,
    ) -> Result<(Path2D, usize), RoboticsError>
    where
        F: FnMut() -> RRTNode,
    {
        let mut tree_a = vec![RRTNode::new(start.x, start.y)];
        let mut tree_b = vec![RRTNode::new(goal.x, goal.y)];
        let mut a_is_start = true;

        for iter in 0..self.config.max_iter {
            let rnd = sample();
            if let Some((new_idx_a, _)) = self.extend_tree(&mut tree_a, &rnd) {
                let target = tree_a[new_idx_a].clone();
                if let Some((new_idx_b, status)) = self.connect_tree(&mut tree_b, &target, policy) {
                    if status == ExtendStatus::Reached {
                        let path = self
                            .reconstruct_path(&tree_a, new_idx_a, &tree_b, new_idx_b, a_is_start);
                        return Ok((path, iter + 1));
                    }
                }
            }

            std::mem::swap(&mut tree_a, &mut tree_b);
            a_is_start = !a_is_start;
        }

        Err(RoboticsError::PlanningError(
            "RRTConnect: Cannot find path within max iterations".to_string(),
        ))
    }

    fn run(
        &self,
        start: Point2D,
        goal: Point2D,
        policy: ConnectPolicy,
    ) -> Result<(Path2D, usize), RoboticsError> {
        self.run_with_sampler(start, goal, policy, || self.get_random_node())
    }
}

impl PathPlanner for RRTConnectPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        self.run(start, goal, ConnectPolicy::AggressiveConnect)
            .map(|(path, _)| path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_collision_free(path: &Path2D, obstacles: &[CircleObstacle], robot_radius: f64) {
        for point in &path.points {
            for obs in obstacles {
                let d = ((point.x - obs.x).powi(2) + (point.y - obs.y).powi(2)).sqrt();
                assert!(
                    d > obs.radius + robot_radius,
                    "path collides with obstacle at ({}, {})",
                    obs.x,
                    obs.y
                );
            }
        }
    }

    #[test]
    fn test_rrt_connect_finds_path_no_obstacles() {
        let planner = RRTConnectPlanner::new(
            vec![],
            AreaBounds::new(-2.0, 15.0, -2.0, 15.0),
            RRTConnectConfig::default(),
        );
        let result = planner.plan(Point2D::new(0.0, 0.0), Point2D::new(10.0, 10.0));
        assert!(
            result.is_ok(),
            "expected a path but got: {:?}",
            result.err()
        );
        let path = result.unwrap();
        assert!(path.points.len() >= 2);
    }

    #[test]
    fn test_rrt_connect_finds_path_with_obstacles() {
        let obstacles = vec![
            CircleObstacle::new(5.0, 5.0, 1.0),
            CircleObstacle::new(3.0, 6.0, 2.0),
            CircleObstacle::new(3.0, 8.0, 2.0),
            CircleObstacle::new(3.0, 10.0, 2.0),
            CircleObstacle::new(7.0, 5.0, 2.0),
            CircleObstacle::new(9.0, 5.0, 2.0),
            CircleObstacle::new(8.0, 10.0, 1.0),
        ];
        let planner = RRTConnectPlanner::new(
            obstacles.clone(),
            AreaBounds::new(-2.0, 15.0, -2.0, 15.0),
            RRTConnectConfig {
                max_iter: 2000,
                ..Default::default()
            },
        );
        let result = planner.plan(Point2D::new(0.0, 0.0), Point2D::new(6.0, 10.0));
        assert!(
            result.is_ok(),
            "expected a path but got: {:?}",
            result.err()
        );
        let path = result.unwrap();
        assert_collision_free(&path, &obstacles, RRTConnectConfig::default().robot_radius);
    }

    #[test]
    fn test_rrt_connect_requires_fewer_iterations_than_extend_only() {
        let planner = RRTConnectPlanner::new(
            vec![],
            AreaBounds::new(-5.0, 20.0, -5.0, 20.0),
            RRTConnectConfig::default(),
        );
        let start = Point2D::new(0.0, 0.0);
        let goal = Point2D::new(12.0, 0.0);
        let samples = [[6.0, 0.0], [6.0, 0.0], [6.0, 0.0]];

        let mut idx_connect = 0usize;
        let (_, connect_iters) = planner
            .run_with_sampler(start, goal, ConnectPolicy::AggressiveConnect, || {
                let sample = samples[idx_connect.min(samples.len() - 1)];
                idx_connect += 1;
                RRTNode::new(sample[0], sample[1])
            })
            .expect("connect should find a path");

        let mut idx_extend = 0usize;
        let (_, extend_iters) = planner
            .run_with_sampler(start, goal, ConnectPolicy::ExtendOnce, || {
                let sample = samples[idx_extend.min(samples.len() - 1)];
                idx_extend += 1;
                RRTNode::new(sample[0], sample[1])
            })
            .expect("extend-only should find a path");

        assert!(connect_iters < extend_iters);
    }
}
