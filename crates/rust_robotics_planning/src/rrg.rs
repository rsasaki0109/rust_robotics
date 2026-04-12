#![allow(dead_code)]

//! RRG (Rapidly-exploring Random Graph) path planning algorithm
//!
//! Like RRT\* but keeps ALL collision-free edges within the rewiring radius,
//! maintaining a graph structure instead of a tree. This gives asymptotic
//! optimality and stronger graph connectivity for multi-query planning.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use rand::Rng;

use rust_robotics_core::{Path2D, PathPlanner, Point2D, RoboticsError};

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// A node in the RRG graph.
#[derive(Debug, Clone)]
pub struct RRGNode {
    pub x: f64,
    pub y: f64,
    /// Indices of all neighbours connected by a collision-free edge.
    pub neighbors: Vec<usize>,
}

impl RRGNode {
    fn new(x: f64, y: f64) -> Self {
        RRGNode {
            x,
            y,
            neighbors: Vec::new(),
        }
    }
}

/// Circular obstacle described by centre and radius.
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

/// Axis-aligned bounding box for the sampling area.
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

/// Configuration for [`RRGPlanner`].
#[derive(Debug, Clone)]
pub struct RRGConfig {
    /// Maximum extension distance per step \[m\].
    pub expand_dis: f64,
    /// Resolution used when checking a path segment for collisions \[m\].
    pub path_resolution: f64,
    /// Maximum number of iterations.
    pub max_iter: usize,
    /// Robot body radius added to each obstacle \[m\].
    pub robot_radius: f64,
    /// Radius within which all collision-free neighbours receive a new edge \[m\].
    pub rewire_radius: f64,
}

impl Default for RRGConfig {
    fn default() -> Self {
        Self {
            expand_dis: 3.0,
            path_resolution: 0.5,
            max_iter: 500,
            robot_radius: 0.8,
            rewire_radius: 5.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Planner
// ---------------------------------------------------------------------------

/// RRG planner.
///
/// Builds a graph incrementally by connecting every new node to **all**
/// collision-free neighbours within `rewire_radius`, rather than only the
/// best-cost parent (as RRT\* does).  Dijkstra is then run on the resulting
/// graph to retrieve the shortest path.
pub struct RRGPlanner {
    config: RRGConfig,
    obstacles: Vec<CircleObstacle>,
    rand_area: AreaBounds,
    /// All nodes accumulated during the last `plan` call.
    nodes: Vec<RRGNode>,
}

impl RRGPlanner {
    /// Create a new planner.
    pub fn new(obstacles: Vec<CircleObstacle>, rand_area: AreaBounds, config: RRGConfig) -> Self {
        Self {
            config,
            obstacles,
            rand_area,
            nodes: Vec::new(),
        }
    }

    // ------------------------------------------------------------------
    // Core algorithm
    // ------------------------------------------------------------------

    fn plan_internal(&mut self, start: Point2D, goal: Point2D) -> Option<Path2D> {
        self.nodes.clear();
        self.nodes.push(RRGNode::new(start.x, start.y)); // index 0

        let mut rng = rand::thread_rng();

        for _ in 0..self.config.max_iter {
            // Sample a random point inside the area bounds.
            let rx = rng.gen_range(self.rand_area.xmin..=self.rand_area.xmax);
            let ry = rng.gen_range(self.rand_area.ymin..=self.rand_area.ymax);

            // Find nearest existing node.
            let nearest_idx = self.nearest_index(rx, ry);
            let (nx, ny) = self.steer(nearest_idx, rx, ry);

            // Collision check for the segment from nearest node to the new point.
            if !self.segment_collision_free(
                self.nodes[nearest_idx].x,
                self.nodes[nearest_idx].y,
                nx,
                ny,
            ) {
                continue;
            }

            let new_idx = self.nodes.len();
            self.nodes.push(RRGNode::new(nx, ny));

            // Connect to ALL collision-free neighbours within rewire_radius.
            let near = self.nodes_within_radius(new_idx);
            for ni in near {
                let (ax, ay) = (self.nodes[new_idx].x, self.nodes[new_idx].y);
                let (bx, by) = (self.nodes[ni].x, self.nodes[ni].y);
                if self.segment_collision_free(ax, ay, bx, by) {
                    // Bidirectional edge.
                    self.nodes[new_idx].neighbors.push(ni);
                    self.nodes[ni].neighbors.push(new_idx);
                }
            }
        }

        // Try to attach the goal to the graph.
        let goal_idx = self.attach_goal(goal.x, goal.y)?;

        // Run Dijkstra from node 0 (start) to goal_idx.
        self.dijkstra(0, goal_idx, start, goal)
    }

    /// Steer from node `from_idx` towards `(tx, ty)` by at most `expand_dis`.
    fn steer(&self, from_idx: usize, tx: f64, ty: f64) -> (f64, f64) {
        let fx = self.nodes[from_idx].x;
        let fy = self.nodes[from_idx].y;
        let dx = tx - fx;
        let dy = ty - fy;
        let dist = (dx * dx + dy * dy).sqrt();
        if dist <= self.config.expand_dis {
            (tx, ty)
        } else {
            let scale = self.config.expand_dis / dist;
            (fx + dx * scale, fy + dy * scale)
        }
    }

    fn nearest_index(&self, x: f64, y: f64) -> usize {
        self.nodes
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                let da = (a.x - x).powi(2) + (a.y - y).powi(2);
                let db = (b.x - x).powi(2) + (b.y - y).powi(2);
                da.partial_cmp(&db).unwrap_or(Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap_or(0)
    }

    /// Returns indices of existing nodes (excluding `exclude`) within `rewire_radius`.
    fn nodes_within_radius(&self, exclude: usize) -> Vec<usize> {
        let r2 = self.config.rewire_radius.powi(2);
        let nx = self.nodes[exclude].x;
        let ny = self.nodes[exclude].y;
        self.nodes
            .iter()
            .enumerate()
            .filter(|(i, node)| {
                if *i == exclude {
                    return false;
                }
                let dx = node.x - nx;
                let dy = node.y - ny;
                dx * dx + dy * dy <= r2
            })
            .map(|(i, _)| i)
            .collect()
    }

    /// Check that a straight-line segment is free of obstacles.
    fn segment_collision_free(&self, x1: f64, y1: f64, x2: f64, y2: f64) -> bool {
        let dx = x2 - x1;
        let dy = y2 - y1;
        let dist = (dx * dx + dy * dy).sqrt();
        let steps = ((dist / self.config.path_resolution).ceil() as usize).max(1);

        for i in 0..=steps {
            let t = i as f64 / steps as f64;
            let px = x1 + t * dx;
            let py = y1 + t * dy;
            for obs in &self.obstacles {
                let ex = px - obs.x;
                let ey = py - obs.y;
                let r = obs.radius + self.config.robot_radius;
                if ex * ex + ey * ey <= r * r {
                    return false;
                }
            }
        }
        true
    }

    /// Try to connect the goal to the graph by finding the nearest node that
    /// has a collision-free line-of-sight.  Returns the goal node index on
    /// success.
    fn attach_goal(&mut self, gx: f64, gy: f64) -> Option<usize> {
        // Collect candidates sorted by distance.
        let mut candidates: Vec<(usize, f64)> = self
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| {
                let dx = n.x - gx;
                let dy = n.y - gy;
                (i, dx * dx + dy * dy)
            })
            .collect();
        candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal));

        for (idx, _) in candidates {
            if self.segment_collision_free(self.nodes[idx].x, self.nodes[idx].y, gx, gy) {
                let goal_idx = self.nodes.len();
                self.nodes.push(RRGNode::new(gx, gy));
                // Bidirectional edge.
                self.nodes[goal_idx].neighbors.push(idx);
                self.nodes[idx].neighbors.push(goal_idx);
                return Some(goal_idx);
            }
        }
        None
    }

    /// Dijkstra shortest path on the graph from `src` to `dst`.
    fn dijkstra(
        &self,
        src: usize,
        dst: usize,
        start_pt: Point2D,
        goal_pt: Point2D,
    ) -> Option<Path2D> {
        let n = self.nodes.len();
        let mut dist = vec![f64::INFINITY; n];
        let mut prev: HashMap<usize, usize> = HashMap::new();
        dist[src] = 0.0;

        // Min-heap: (neg_cost, index) — we negate because BinaryHeap is a max-heap.
        let mut heap: BinaryHeap<DijkstraItem> = BinaryHeap::new();
        heap.push(DijkstraItem {
            cost: 0.0,
            idx: src,
        });

        while let Some(DijkstraItem { cost, idx }) = heap.pop() {
            if idx == dst {
                break;
            }
            if cost > dist[idx] {
                continue;
            }
            for &nb in &self.nodes[idx].neighbors {
                let dx = self.nodes[nb].x - self.nodes[idx].x;
                let dy = self.nodes[nb].y - self.nodes[idx].y;
                let edge_cost = (dx * dx + dy * dy).sqrt();
                let new_cost = dist[idx] + edge_cost;
                if new_cost < dist[nb] {
                    dist[nb] = new_cost;
                    prev.insert(nb, idx);
                    heap.push(DijkstraItem {
                        cost: new_cost,
                        idx: nb,
                    });
                }
            }
        }

        if dist[dst] == f64::INFINITY {
            return None;
        }

        // Reconstruct path.
        let mut path_pts: Vec<Point2D> = Vec::new();
        let mut cur = dst;
        loop {
            path_pts.push(Point2D::new(self.nodes[cur].x, self.nodes[cur].y));
            match prev.get(&cur) {
                Some(&p) => cur = p,
                None => break,
            }
        }
        path_pts.reverse();

        // Replace first/last with exact start/goal coordinates.
        if let Some(first) = path_pts.first_mut() {
            *first = start_pt;
        }
        if let Some(last) = path_pts.last_mut() {
            *last = goal_pt;
        }

        Some(Path2D::from_points(path_pts))
    }

    /// Return the graph built during the last planning call (for inspection).
    pub fn get_graph(&self) -> &[RRGNode] {
        &self.nodes
    }
}

// ---------------------------------------------------------------------------
// PathPlanner trait
// ---------------------------------------------------------------------------

impl PathPlanner for RRGPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        // `plan_internal` mutates `self.nodes`, so we need `&mut self`.
        // We work around the `&self` signature by cloning the planner state.
        let mut planner = RRGPlanner {
            config: self.config.clone(),
            obstacles: self.obstacles.clone(),
            rand_area: self.rand_area.clone(),
            nodes: Vec::new(),
        };
        planner.plan_internal(start, goal).ok_or_else(|| {
            RoboticsError::PlanningError("RRG: no path found within max_iter".to_string())
        })
    }
}

// ---------------------------------------------------------------------------
// Internal priority-queue item for Dijkstra
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct DijkstraItem {
    cost: f64,
    idx: usize,
}

impl PartialEq for DijkstraItem {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost && self.idx == other.idx
    }
}

impl Eq for DijkstraItem {}

impl Ord for DijkstraItem {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse order so BinaryHeap becomes a min-heap.
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for DijkstraItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_bounds() -> AreaBounds {
        AreaBounds::new(-2.0, 15.0, -2.0, 15.0)
    }

    #[test]
    fn test_rrg_config_defaults() {
        let cfg = RRGConfig::default();
        assert_eq!(cfg.expand_dis, 3.0);
        assert_eq!(cfg.path_resolution, 0.5);
        assert_eq!(cfg.max_iter, 500);
        assert_eq!(cfg.robot_radius, 0.8);
        assert_eq!(cfg.rewire_radius, 5.0);
    }

    #[test]
    fn test_rrg_finds_path_no_obstacles() {
        let cfg = RRGConfig {
            max_iter: 500,
            ..Default::default()
        };
        let mut planner = RRGPlanner::new(vec![], default_bounds(), cfg);

        let start = Point2D::new(0.0, 0.0);
        let goal = Point2D::new(6.0, 10.0);

        let path = planner.plan_internal(start, goal);
        assert!(path.is_some(), "should find a path with no obstacles");

        let path = path.unwrap();
        let pts = &path.points;
        assert!(!pts.is_empty());

        let first = pts.first().unwrap();
        let last = pts.last().unwrap();
        assert!((first.x - 0.0).abs() < 1e-9 && (first.y - 0.0).abs() < 1e-9);
        assert!((last.x - 6.0).abs() < 1e-9 && (last.y - 10.0).abs() < 1e-9);
        println!(
            "[no_obstacles] path length {} nodes, {} segments",
            pts.len(),
            pts.len().saturating_sub(1)
        );
    }

    #[test]
    fn test_rrg_finds_path_with_obstacles() {
        let obstacles = vec![
            CircleObstacle::new(5.0, 5.0, 1.0),
            CircleObstacle::new(3.0, 6.0, 2.0),
            CircleObstacle::new(3.0, 8.0, 2.0),
            CircleObstacle::new(3.0, 10.0, 2.0),
            CircleObstacle::new(7.0, 5.0, 2.0),
            CircleObstacle::new(9.0, 5.0, 2.0),
        ];
        let cfg = RRGConfig {
            max_iter: 2000,
            ..Default::default()
        };
        let mut planner = RRGPlanner::new(obstacles, default_bounds(), cfg);

        let start = Point2D::new(0.0, 0.0);
        let goal = Point2D::new(6.0, 10.0);

        let path = planner.plan_internal(start, goal);
        assert!(path.is_some(), "should find a path even with obstacles");

        let path = path.unwrap();
        let pts = &path.points;
        let first = pts.first().unwrap();
        let last = pts.last().unwrap();
        assert!((first.x - 0.0).abs() < 1e-9 && (first.y - 0.0).abs() < 1e-9);
        assert!((last.x - 6.0).abs() < 1e-9 && (last.y - 10.0).abs() < 1e-9);
        println!("[with_obstacles] path length {} nodes", pts.len());
    }
}
