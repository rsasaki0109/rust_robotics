//! Bidirectional RRT (Rapidly-exploring Random Tree) path planning algorithm
//!
//! Grows two trees simultaneously — one from start, one from goal — and
//! attempts to connect them each iteration for faster convergence.

use rand::Rng;

use rust_robotics_core::{Path2D, PathPlanner, Point2D, RoboticsError};

/// Internal node for a Bidirectional RRT tree
#[derive(Debug, Clone)]
pub struct RRTNode {
    pub x: f64,
    pub y: f64,
    pub parent: Option<usize>,
}

impl RRTNode {
    pub fn new(x: f64, y: f64) -> Self {
        RRTNode { x, y, parent: None }
    }

    fn to_point(&self) -> Point2D {
        Point2D::new(self.x, self.y)
    }
}

/// Circular obstacle \(x, y, radius\)
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

/// Axis-aligned bounding box for the random-sampling area
#[derive(Debug, Clone)]
pub struct AreaBounds {
    pub xmin: f64,
    pub xmax: f64,
    pub ymin: f64,
    pub ymax: f64,
}

impl AreaBounds {
    pub fn new(xmin: f64, xmax: f64, ymin: f64, ymax: f64) -> Self {
        AreaBounds {
            xmin,
            xmax,
            ymin,
            ymax,
        }
    }
}

/// Configuration for the Bidirectional RRT planner
#[derive(Debug, Clone)]
pub struct BidirectionalRRTConfig {
    /// Maximum step length when extending a tree \[m\]
    pub expand_dis: f64,
    /// Interpolation resolution along a segment \[m\]
    pub path_resolution: f64,
    /// Maximum number of iterations before failure
    pub max_iter: usize,
    /// Robot radius used for collision checking \[m\]
    pub robot_radius: f64,
}

impl Default for BidirectionalRRTConfig {
    fn default() -> Self {
        Self {
            expand_dis: 3.0,
            path_resolution: 0.5,
            max_iter: 500,
            robot_radius: 0.8,
        }
    }
}

/// Bidirectional RRT path planner
pub struct BidirectionalRRTPlanner {
    config: BidirectionalRRTConfig,
    obstacles: Vec<CircleObstacle>,
    rand_area: AreaBounds,
}

impl BidirectionalRRTPlanner {
    pub fn new(
        obstacles: Vec<CircleObstacle>,
        rand_area: AreaBounds,
        config: BidirectionalRRTConfig,
    ) -> Self {
        BidirectionalRRTPlanner {
            config,
            obstacles,
            rand_area,
        }
    }

    // ── helpers ──────────────────────────────────────────────────────────────

    fn get_random_node(&self) -> RRTNode {
        let mut rng = rand::rng();
        RRTNode::new(
            rng.random_range(self.rand_area.xmin..=self.rand_area.xmax),
            rng.random_range(self.rand_area.ymin..=self.rand_area.ymax),
        )
    }

    fn get_nearest_node_index(tree: &[RRTNode], target: &RRTNode) -> usize {
        tree.iter()
            .enumerate()
            .map(|(i, n)| {
                let dx = n.x - target.x;
                let dy = n.y - target.y;
                (i, dx * dx + dy * dy)
            })
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(0)
    }

    fn dist(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
        let dx = ax - bx;
        let dy = ay - by;
        (dx * dx + dy * dy).sqrt()
    }

    /// Steer from `from` toward `to` by at most `expand_dis`, interpolating
    /// with `path_resolution`. Returns `None` if collision detected.
    fn steer(&self, from: &RRTNode, to: &RRTNode, parent_idx: usize) -> Option<RRTNode> {
        let dx = to.x - from.x;
        let dy = to.y - from.y;
        let d = (dx * dx + dy * dy).sqrt();
        let theta = dy.atan2(dx);
        let step = d.min(self.config.expand_dis);
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
        // snap to destination if close enough
        if Self::dist(cx, cy, to.x, to.y) <= self.config.path_resolution {
            cx = to.x;
            cy = to.y;
        }
        if self.point_in_collision(cx, cy) {
            return None;
        }

        Some(RRTNode {
            x: cx,
            y: cy,
            parent: Some(parent_idx),
        })
    }

    fn point_in_collision(&self, x: f64, y: f64) -> bool {
        for obs in &self.obstacles {
            if Self::dist(x, y, obs.x, obs.y) <= obs.radius + self.config.robot_radius {
                return true;
            }
        }
        false
    }

    /// Check whether a straight segment between two nodes is collision-free.
    fn check_collision(&self, ax: f64, ay: f64, bx: f64, by: f64) -> bool {
        let d = Self::dist(ax, ay, bx, by);
        let n = (d / self.config.path_resolution).ceil() as usize;
        for i in 0..=n {
            let t = if n == 0 { 0.0 } else { i as f64 / n as f64 };
            let x = ax + t * (bx - ax);
            let y = ay + t * (by - ay);
            if self.point_in_collision(x, y) {
                return false;
            }
        }
        true
    }

    // ── path extraction ───────────────────────────────────────────────────────

    /// Trace a tree from leaf `idx` back to its root, returning nodes in
    /// root→leaf order.
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

    // ── core algorithm ────────────────────────────────────────────────────────

    fn run(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        // tree_a grows from start, tree_b grows from goal; they are swapped
        // each iteration for balanced growth.
        let mut tree_a: Vec<RRTNode> = vec![RRTNode::new(start.x, start.y)];
        let mut tree_b: Vec<RRTNode> = vec![RRTNode::new(goal.x, goal.y)];

        // Track which tree started from start (needed for path direction).
        let mut a_is_start = true;

        for _ in 0..self.config.max_iter {
            // Extend tree_a toward a random sample.
            let rnd = self.get_random_node();
            let nearest_a = Self::get_nearest_node_index(&tree_a, &rnd);
            if let Some(new_node) = self.steer(&tree_a[nearest_a].clone(), &rnd, nearest_a) {
                tree_a.push(new_node);
                let new_a_idx = tree_a.len() - 1;
                let (na_x, na_y) = (tree_a[new_a_idx].x, tree_a[new_a_idx].y);

                // Try to connect tree_b to the new node in tree_a.
                let nearest_b = Self::get_nearest_node_index(&tree_b, &tree_a[new_a_idx]);
                let (nb_x, nb_y) = (tree_b[nearest_b].x, tree_b[nearest_b].y);

                if Self::dist(na_x, na_y, nb_x, nb_y) <= self.config.expand_dis
                    && self.check_collision(na_x, na_y, nb_x, nb_y)
                {
                    // Trees connected — reconstruct path.
                    let mut path_a = Self::trace_path(&tree_a, new_a_idx);
                    let mut path_b = Self::trace_path(&tree_b, nearest_b);

                    // path_a is root_a → junction_a
                    // path_b is root_b → junction_b
                    // Ensure the full path runs: start → goal.
                    if a_is_start {
                        // path_a: start→…, path_b: goal→… → reverse path_b
                        path_b.reverse();
                        path_a.extend(path_b);
                    } else {
                        // tree_a grew from goal, tree_b from start
                        path_b.reverse();
                        path_b.extend(path_a);
                        path_a = path_b;
                    }
                    return Ok(Path2D::from_points(path_a));
                }
            }

            // Swap trees for balanced growth.
            std::mem::swap(&mut tree_a, &mut tree_b);
            a_is_start = !a_is_start;
        }

        Err(RoboticsError::PlanningError(
            "BidirectionalRRT: Cannot find path within max iterations".to_string(),
        ))
    }
}

impl PathPlanner for BidirectionalRRTPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        self.run(start, goal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bidir_rrt_config_defaults() {
        let cfg = BidirectionalRRTConfig::default();
        assert_eq!(cfg.expand_dis, 3.0);
        assert_eq!(cfg.path_resolution, 0.5);
        assert_eq!(cfg.max_iter, 500);
        assert_eq!(cfg.robot_radius, 0.8);
    }

    #[test]
    fn test_bidir_rrt_finds_path_no_obstacles() {
        let planner = BidirectionalRRTPlanner::new(
            vec![],
            AreaBounds::new(-2.0, 15.0, -2.0, 15.0),
            BidirectionalRRTConfig::default(),
        );
        let result = planner.plan(Point2D::new(0.0, 0.0), Point2D::new(10.0, 10.0));
        assert!(
            result.is_ok(),
            "expected a path but got: {:?}",
            result.err()
        );
        let path = result.unwrap();
        assert!(
            path.points.len() >= 2,
            "path should have at least start and goal"
        );
    }

    #[test]
    fn test_bidir_rrt_finds_path_with_obstacles() {
        let obstacles = vec![
            CircleObstacle::new(5.0, 5.0, 1.0),
            CircleObstacle::new(3.0, 6.0, 2.0),
            CircleObstacle::new(3.0, 8.0, 2.0),
            CircleObstacle::new(3.0, 10.0, 2.0),
            CircleObstacle::new(7.0, 5.0, 2.0),
            CircleObstacle::new(9.0, 5.0, 2.0),
            CircleObstacle::new(8.0, 10.0, 1.0),
        ];
        let planner = BidirectionalRRTPlanner::new(
            obstacles,
            AreaBounds::new(-2.0, 15.0, -2.0, 15.0),
            BidirectionalRRTConfig {
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
    }
}
