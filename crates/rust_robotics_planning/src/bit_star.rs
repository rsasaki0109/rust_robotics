#![allow(dead_code, clippy::too_many_arguments)]

//! Batch Informed Trees (BIT*) path planning algorithm
//!
//! An asymptotically optimal sampling-based planner that combines RRT* rewiring
//! with informed (ellipsoidal) sampling. Vertices are processed in batches
//! ordered by potential solution cost, using a lazy edge evaluation strategy.

use std::collections::BinaryHeap;
use std::f64::consts::PI;

use nalgebra::{Matrix2, Vector2};
use rand::Rng;

use rust_robotics_core::{Path2D, Point2D, RoboticsError, RoboticsResult};

/// A vertex in the BIT* search tree.
#[derive(Clone, Debug)]
struct Vertex {
    pos: Vector2<f64>,
    /// Cost-to-come from the start vertex through the tree.
    cost: f64,
    /// Index of the parent vertex in the vertex list, if connected to the tree.
    parent: Option<usize>,
}

impl Vertex {
    fn new(x: f64, y: f64) -> Self {
        Self {
            pos: Vector2::new(x, y),
            cost: f64::INFINITY,
            parent: None,
        }
    }
}

/// An edge in the priority queue, ordered by estimated total cost (low cost = high priority).
#[derive(Clone, Debug, PartialEq)]
struct QueueEdge {
    /// Estimated total cost of a solution through this edge.
    estimated_cost: f64,
    from: usize,
    to: usize,
}

impl Eq for QueueEdge {}

impl PartialOrd for QueueEdge {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for QueueEdge {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Reverse ordering for min-heap behaviour with BinaryHeap (max-heap).
        other
            .estimated_cost
            .partial_cmp(&self.estimated_cost)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

/// Configuration for the BIT* planner.
#[derive(Clone, Debug)]
pub struct BITStarConfig {
    /// Number of new samples to draw per batch.
    pub batch_size: usize,
    /// Maximum number of batches to run.
    pub max_batches: usize,
    /// Connection radius factor. The actual radius is `eta * (log(n)/n)^(1/d)` for d=2.
    pub eta: f64,
    /// Goal proximity threshold – a vertex closer than this to the goal is considered a goal vertex.
    pub goal_threshold: f64,
}

impl Default for BITStarConfig {
    fn default() -> Self {
        Self {
            batch_size: 100,
            max_batches: 200,
            eta: 40.0,
            goal_threshold: 0.5,
        }
    }
}

/// Batch Informed Trees (BIT*) planner.
pub struct BITStar {
    config: BITStarConfig,
    start: Vector2<f64>,
    goal: Vector2<f64>,
    obstacles: Vec<(f64, f64, f64)>, // (x, y, radius)
    area_min: f64,
    area_max: f64,
    /// All vertices (both in tree and unconnected samples).
    vertices: Vec<Vertex>,
    /// Set of vertex indices that are part of the tree (have finite cost).
    tree_set: Vec<bool>,
}

impl BITStar {
    /// Create a new BIT* planner.
    ///
    /// * `start` – (x, y) start position
    /// * `goal` – (x, y) goal position
    /// * `obstacles` – list of circular obstacles (x, y, radius)
    /// * `rand_area` – (min, max) bounds for uniform sampling
    /// * `config` – planner configuration
    pub fn new(
        start: (f64, f64),
        goal: (f64, f64),
        obstacles: Vec<(f64, f64, f64)>,
        rand_area: (f64, f64),
        config: BITStarConfig,
    ) -> Self {
        Self {
            config,
            start: Vector2::new(start.0, start.1),
            goal: Vector2::new(goal.0, goal.1),
            obstacles,
            area_min: rand_area.0,
            area_max: rand_area.1,
            vertices: Vec::new(),
            tree_set: Vec::new(),
        }
    }

    /// Run the planner and return the best path found (start to goal), or `None`.
    pub fn planning(&mut self) -> Option<Vec<[f64; 2]>> {
        self.reset();

        let mut best_cost = f64::INFINITY;

        for _batch in 0..self.config.max_batches {
            // --- Sample a new batch ---
            self.add_samples(best_cost);

            // --- Build edge queue ---
            let mut edge_queue = self.build_edge_queue(best_cost);

            // --- Process edges ---
            while let Some(edge) = edge_queue.pop() {
                // Prune: skip if the optimistic estimate already exceeds best_cost.
                if edge.estimated_cost >= best_cost {
                    break;
                }

                let from_idx = edge.from;
                let to_idx = edge.to;

                // True cost of traversing this edge.
                let edge_cost = self.dist(from_idx, to_idx);
                let new_cost = self.vertices[from_idx].cost + edge_cost;

                // Only useful if it would improve the target vertex.
                if new_cost >= self.vertices[to_idx].cost {
                    continue;
                }

                // Lazy collision check.
                if !self.collision_free(from_idx, to_idx) {
                    continue;
                }

                // Accept this edge – wire / rewire.
                self.vertices[to_idx].cost = new_cost;
                self.vertices[to_idx].parent = Some(from_idx);
                self.tree_set[to_idx] = true;

                // Check whether this vertex can reach the goal cheaply.
                let dist_to_goal = (self.vertices[to_idx].pos - self.goal).norm();
                if dist_to_goal < self.config.goal_threshold {
                    let total = new_cost + dist_to_goal;
                    if total < best_cost {
                        best_cost = total;
                    }
                }
            }

            // Prune vertices whose heuristic cost exceeds best_cost.
            self.prune(best_cost);
        }

        self.extract_path(best_cost)
    }

    /// Convenience wrapper returning [`Path2D`].
    pub fn plan_from(&mut self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        self.start = Vector2::new(start.x, start.y);
        self.goal = Vector2::new(goal.x, goal.y);

        self.planning()
            .map(|raw| {
                Path2D::from_points(raw.into_iter().map(|p| Point2D::new(p[0], p[1])).collect())
            })
            .ok_or_else(|| {
                RoboticsError::PlanningError(
                    "BIT*: Cannot find path within max batches".to_string(),
                )
            })
    }

    // ---- Internal helpers ----

    fn reset(&mut self) {
        self.vertices.clear();
        self.tree_set.clear();
        // Add start vertex with zero cost.
        let mut start_v = Vertex::new(self.start.x, self.start.y);
        start_v.cost = 0.0;
        self.vertices.push(start_v);
        self.tree_set.push(true);
    }

    /// Sample new vertices using informed (ellipsoidal) sampling when a solution exists,
    /// or uniform sampling otherwise.
    fn add_samples(&mut self, best_cost: f64) {
        let mut rng = rand::thread_rng();
        let c_min = (self.goal - self.start).norm();

        for _ in 0..self.config.batch_size {
            let pos = if best_cost < f64::INFINITY {
                self.sample_ellipse(best_cost, c_min, &mut rng)
            } else {
                Vector2::new(
                    rng.gen_range(self.area_min..=self.area_max),
                    rng.gen_range(self.area_min..=self.area_max),
                )
            };

            let mut v = Vertex::new(pos.x, pos.y);
            v.cost = f64::INFINITY;
            self.vertices.push(v);
            self.tree_set.push(false);
        }
    }

    /// Sample a point uniformly inside the prolate hyperspheroid focused on start/goal.
    fn sample_ellipse(&self, c_best: f64, c_min: f64, rng: &mut impl Rng) -> Vector2<f64> {
        let center = (self.start + self.goal) / 2.0;
        let diff = self.goal - self.start;
        let angle = diff.y.atan2(diff.x);
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        let rotation = Matrix2::new(cos_a, -sin_a, sin_a, cos_a);

        let r1 = c_best / 2.0;
        let r2 = (c_best * c_best - c_min * c_min).max(0.0).sqrt() / 2.0;

        // Uniform sampling inside an ellipse via unit-disk transform.
        let theta = rng.gen_range(0.0..2.0 * PI);
        let r = rng.gen::<f64>().sqrt();
        let unit = Vector2::new(r * theta.cos(), r * theta.sin());
        let scaled = Vector2::new(r1 * unit.x, r2 * unit.y);

        center + rotation * scaled
    }

    /// Compute the connection radius for the current number of vertices.
    fn connection_radius(&self) -> f64 {
        let n = self.vertices.len().max(2) as f64;
        self.config.eta * (n.ln() / n).sqrt()
    }

    /// Build the edge queue: for every tree vertex, add edges to nearby non-tree or
    /// improvable vertices.
    fn build_edge_queue(&self, best_cost: f64) -> BinaryHeap<QueueEdge> {
        let r = self.connection_radius();
        let r_sq = r * r;
        let mut queue = BinaryHeap::new();

        for (i, vi) in self.vertices.iter().enumerate() {
            if !self.tree_set[i] {
                continue;
            }
            for (j, vj) in self.vertices.iter().enumerate() {
                if i == j {
                    continue;
                }
                let d_sq = (vi.pos - vj.pos).norm_squared();
                if d_sq > r_sq {
                    continue;
                }
                let edge_cost = d_sq.sqrt();
                let new_cost = vi.cost + edge_cost;

                // Skip if this cannot improve the target vertex.
                if new_cost >= vj.cost {
                    continue;
                }

                // Optimistic estimate of total solution cost through this edge.
                let estimated = new_cost + (vj.pos - self.goal).norm();
                if estimated >= best_cost {
                    continue;
                }

                queue.push(QueueEdge {
                    estimated_cost: estimated,
                    from: i,
                    to: j,
                });
            }
        }

        queue
    }

    /// Euclidean distance between two vertices.
    fn dist(&self, a: usize, b: usize) -> f64 {
        (self.vertices[a].pos - self.vertices[b].pos).norm()
    }

    /// Check that the straight-line segment between two vertices is collision-free.
    fn collision_free(&self, a: usize, b: usize) -> bool {
        let pa = self.vertices[a].pos;
        let pb = self.vertices[b].pos;
        self.segment_collision_free(pa.x, pa.y, pb.x, pb.y)
    }

    fn segment_collision_free(&self, x1: f64, y1: f64, x2: f64, y2: f64) -> bool {
        for &(ox, oy, radius) in &self.obstacles {
            let dd = Self::point_to_segment_dist_sq([x1, y1], [x2, y2], [ox, oy]);
            if dd <= radius * radius {
                return false;
            }
        }
        true
    }

    fn point_to_segment_dist_sq(v: [f64; 2], w: [f64; 2], p: [f64; 2]) -> f64 {
        let l2 = (w[0] - v[0]).powi(2) + (w[1] - v[1]).powi(2);
        if l2 == 0.0 {
            return (p[0] - v[0]).powi(2) + (p[1] - v[1]).powi(2);
        }
        let t =
            (((p[0] - v[0]) * (w[0] - v[0]) + (p[1] - v[1]) * (w[1] - v[1])) / l2).clamp(0.0, 1.0);
        let proj = [v[0] + t * (w[0] - v[0]), v[1] + t * (w[1] - v[1])];
        (p[0] - proj[0]).powi(2) + (p[1] - proj[1]).powi(2)
    }

    /// Remove samples whose optimistic cost exceeds `best_cost`.
    fn prune(&mut self, best_cost: f64) {
        if best_cost >= f64::INFINITY {
            return;
        }

        for i in 0..self.vertices.len() {
            // Never prune the start vertex.
            if i == 0 {
                continue;
            }
            let heuristic = (self.vertices[i].pos - self.start).norm()
                + (self.vertices[i].pos - self.goal).norm();
            if heuristic > best_cost {
                // Disconnect this vertex.
                self.vertices[i].cost = f64::INFINITY;
                self.vertices[i].parent = None;
                self.tree_set[i] = false;
            }
        }
    }

    /// Trace back from the best goal-connected vertex to produce the path.
    fn extract_path(&self, best_cost: f64) -> Option<Vec<[f64; 2]>> {
        if best_cost >= f64::INFINITY {
            return None;
        }

        // Find the best vertex that is near the goal.
        let mut best_idx = None;
        let mut best_total = f64::INFINITY;
        for (i, v) in self.vertices.iter().enumerate() {
            if !self.tree_set[i] {
                continue;
            }
            let dist_to_goal = (v.pos - self.goal).norm();
            if dist_to_goal < self.config.goal_threshold {
                let total = v.cost + dist_to_goal;
                if total < best_total {
                    best_total = total;
                    best_idx = Some(i);
                }
            }
        }

        let best_idx = best_idx?;

        // Trace back to start.
        let mut path = vec![[self.goal.x, self.goal.y]];
        let mut current = best_idx;
        loop {
            let v = &self.vertices[current];
            path.push([v.pos.x, v.pos.y]);
            match v.parent {
                Some(p) => current = p,
                None => break,
            }
        }
        path.reverse();
        Some(path)
    }

    /// Get the current vertices for inspection.
    pub fn get_vertices(&self) -> Vec<(f64, f64, f64)> {
        self.vertices
            .iter()
            .map(|v| (v.pos.x, v.pos.y, v.cost))
            .collect()
    }

    /// Get the obstacle list.
    pub fn get_obstacles(&self) -> &[(f64, f64, f64)] {
        &self.obstacles
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: compute path length from a list of waypoints.
    fn path_length(path: &[[f64; 2]]) -> f64 {
        path.windows(2)
            .map(|w| ((w[1][0] - w[0][0]).powi(2) + (w[1][1] - w[0][1]).powi(2)).sqrt())
            .sum()
    }

    #[test]
    fn test_bit_star_finds_path_open_space() {
        let config = BITStarConfig {
            batch_size: 200,
            max_batches: 10,
            eta: 40.0,
            goal_threshold: 1.0,
        };
        let mut planner = BITStar::new(
            (0.0, 0.0),
            (10.0, 10.0),
            vec![], // no obstacles
            (-2.0, 15.0),
            config,
        );

        let path = planner.planning();
        assert!(path.is_some(), "BIT* should find a path in open space");

        let path = path.unwrap();
        // Path must start near start and end at goal.
        assert!(path.len() >= 2);
        let first = path.first().unwrap();
        let last = path.last().unwrap();
        assert!(
            (first[0]).abs() < 1.5 && (first[1]).abs() < 1.5,
            "Path should start near (0,0)"
        );
        assert!(
            (last[0] - 10.0).abs() < 1.5 && (last[1] - 10.0).abs() < 1.5,
            "Path should end near (10,10)"
        );

        // Path cost should be reasonable (straight-line is ~14.14).
        let cost = path_length(&path);
        assert!(
            cost < 25.0,
            "Path cost {} is unreasonably large for open space",
            cost
        );
    }

    #[test]
    fn test_bit_star_finds_path_around_obstacles() {
        let obstacles = vec![(5.0, 5.0, 1.0)];
        let config = BITStarConfig {
            batch_size: 200,
            max_batches: 10,
            eta: 30.0,
            goal_threshold: 2.0,
        };
        let mut planner = BITStar::new(
            (0.0, 0.0),
            (10.0, 10.0),
            obstacles.clone(),
            (-5.0, 20.0),
            config,
        );

        let path = planner.planning();
        assert!(path.is_some(), "BIT* should find a path around obstacles");

        let path = path.unwrap();
        // Verify no segment of the returned path collides with obstacles.
        for window in path.windows(2) {
            let (x1, y1) = (window[0][0], window[0][1]);
            let (x2, y2) = (window[1][0], window[1][1]);
            for &(ox, oy, r) in &obstacles {
                let dd = BITStar::point_to_segment_dist_sq([x1, y1], [x2, y2], [ox, oy]);
                assert!(
                    dd > r * r * 0.9, // slight tolerance for numerical issues
                    "Path segment ({},{})--({},{}) collides with obstacle ({},{},{})",
                    x1,
                    y1,
                    x2,
                    y2,
                    ox,
                    oy,
                    r
                );
            }
        }
    }

    #[test]
    fn test_bit_star_cost_improves_with_more_iterations() {
        let obstacles = vec![(5.0, 5.0, 1.5)];

        let mut costs = Vec::new();
        for &max_batches in &[3, 10, 20] {
            let config = BITStarConfig {
                batch_size: 100,
                max_batches,
                eta: 30.0,
                goal_threshold: 2.0,
            };
            let mut trial_costs = Vec::new();
            for _ in 0..5 {
                let mut planner = BITStar::new(
                    (0.0, 0.0),
                    (10.0, 10.0),
                    obstacles.clone(),
                    (-5.0, 20.0),
                    config.clone(),
                );
                if let Some(path) = planner.planning() {
                    trial_costs.push(path_length(&path));
                }
            }
            assert!(
                !trial_costs.is_empty(),
                "At least one trial with max_batches={} should find a path",
                max_batches
            );
            trial_costs.sort_by(|a, b| a.partial_cmp(b).unwrap());
            costs.push(trial_costs[trial_costs.len() / 2]);
        }

        // With more batches the median cost should not increase (it should decrease or stay).
        assert!(
            costs[2] <= costs[0] + 1.0,
            "Cost should improve (or stay similar) with more batches: {:?}",
            costs
        );
    }

    #[test]
    fn test_bit_star_plan_from_returns_path2d() {
        let config = BITStarConfig {
            batch_size: 200,
            max_batches: 20,
            eta: 40.0,
            goal_threshold: 1.0,
        };
        let mut planner = BITStar::new((0.0, 0.0), (10.0, 10.0), vec![], (-2.0, 15.0), config);

        let result = planner.plan_from(Point2D::new(0.0, 0.0), Point2D::new(10.0, 10.0));
        assert!(result.is_ok(), "plan_from should succeed in open space");
        let path = result.unwrap();
        assert!(path.points.len() >= 2);
    }
}
