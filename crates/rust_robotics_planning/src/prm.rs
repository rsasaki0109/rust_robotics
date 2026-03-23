#![allow(dead_code, clippy::too_many_arguments)]

//! Probabilistic Road-Map (PRM) path planning
//!
//! Sampling-based planner that builds a roadmap of collision-free
//! configurations and searches for a path using Dijkstra's algorithm.

use rand::Rng;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

// Parameters
const N_SAMPLE: usize = 500;
const N_KNN: usize = 10;
const MAX_EDGE_LEN: f64 = 30.0;

/// Node for Dijkstra search
#[derive(Clone)]
struct Node {
    x: f64,
    y: f64,
    cost: f64,
    parent: Option<usize>,
}

impl Node {
    fn new(x: f64, y: f64) -> Self {
        Node {
            x,
            y,
            cost: f64::INFINITY,
            parent: None,
        }
    }
}

/// Priority queue item for Dijkstra
#[derive(Clone)]
struct QueueItem {
    cost: f64,
    index: usize,
}

impl PartialEq for QueueItem {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl Eq for QueueItem {}

impl Ord for QueueItem {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for QueueItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Simple KDTree for nearest neighbor search
struct KDTree {
    points: Vec<(f64, f64)>,
}

impl KDTree {
    fn new(points: Vec<(f64, f64)>) -> Self {
        KDTree { points }
    }

    fn query_knn(&self, x: f64, y: f64, k: usize) -> Vec<(usize, f64)> {
        let mut distances: Vec<(usize, f64)> = self
            .points
            .iter()
            .enumerate()
            .map(|(i, (px, py))| {
                let d = ((x - px).powi(2) + (y - py).powi(2)).sqrt();
                (i, d)
            })
            .collect();

        distances.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        distances.truncate(k + 1);
        distances
    }

    fn min_distance(&self, x: f64, y: f64) -> f64 {
        self.points
            .iter()
            .map(|(px, py)| ((x - px).powi(2) + (y - py).powi(2)).sqrt())
            .fold(f64::INFINITY, f64::min)
    }
}

/// PRM Planner
pub struct PRMPlanner {
    sample_x: Vec<f64>,
    sample_y: Vec<f64>,
    road_map: Vec<Vec<usize>>,
}

impl PRMPlanner {
    /// Create a new PRM planner
    pub fn new(
        ox: &[f64],
        oy: &[f64],
        start: (f64, f64),
        goal: (f64, f64),
        robot_radius: f64,
    ) -> Self {
        let obstacle_tree = KDTree::new(ox.iter().zip(oy.iter()).map(|(&x, &y)| (x, y)).collect());

        let min_x = ox.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_x = ox.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_y = oy.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_y = oy.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        let (sample_x, sample_y) = Self::sample_points(
            start,
            goal,
            min_x,
            max_x,
            min_y,
            max_y,
            robot_radius,
            &obstacle_tree,
        );

        let road_map = Self::generate_road_map(&sample_x, &sample_y, robot_radius, &obstacle_tree);

        PRMPlanner {
            sample_x,
            sample_y,
            road_map,
        }
    }

    fn sample_points(
        start: (f64, f64),
        goal: (f64, f64),
        min_x: f64,
        max_x: f64,
        min_y: f64,
        max_y: f64,
        robot_radius: f64,
        obstacle_tree: &KDTree,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut rng = rand::thread_rng();
        let mut sample_x = Vec::new();
        let mut sample_y = Vec::new();

        while sample_x.len() < N_SAMPLE {
            let x = rng.gen_range(min_x..max_x);
            let y = rng.gen_range(min_y..max_y);

            let min_dist = obstacle_tree.min_distance(x, y);
            if min_dist > robot_radius {
                sample_x.push(x);
                sample_y.push(y);
            }
        }

        sample_x.push(start.0);
        sample_y.push(start.1);
        sample_x.push(goal.0);
        sample_y.push(goal.1);

        (sample_x, sample_y)
    }

    fn generate_road_map(
        sample_x: &[f64],
        sample_y: &[f64],
        robot_radius: f64,
        obstacle_tree: &KDTree,
    ) -> Vec<Vec<usize>> {
        let sample_tree = KDTree::new(
            sample_x
                .iter()
                .zip(sample_y.iter())
                .map(|(&x, &y)| (x, y))
                .collect(),
        );

        let mut road_map: Vec<Vec<usize>> = vec![Vec::new(); sample_x.len()];

        for (i, (&x, &y)) in sample_x.iter().zip(sample_y.iter()).enumerate() {
            let neighbors = sample_tree.query_knn(x, y, N_KNN);

            for (j, dist) in neighbors {
                if i == j {
                    continue;
                }

                if dist > MAX_EDGE_LEN {
                    continue;
                }

                if !Self::is_collision(x, y, sample_x[j], sample_y[j], robot_radius, obstacle_tree)
                {
                    road_map[i].push(j);
                }
            }
        }

        road_map
    }

    fn is_collision(
        x1: f64,
        y1: f64,
        x2: f64,
        y2: f64,
        robot_radius: f64,
        obstacle_tree: &KDTree,
    ) -> bool {
        let dx = x2 - x1;
        let dy = y2 - y1;
        let d = (dx * dx + dy * dy).sqrt();

        if d == 0.0 {
            return false;
        }

        let step = robot_radius;
        let n_steps = (d / step).ceil() as usize;

        for i in 0..=n_steps {
            let t = i as f64 / n_steps as f64;
            let x = x1 + t * dx;
            let y = y1 + t * dy;

            let min_dist = obstacle_tree.min_distance(x, y);
            if min_dist <= robot_radius {
                return true;
            }
        }

        false
    }

    /// Plan path using Dijkstra's algorithm
    pub fn plan(&self) -> Option<(Vec<f64>, Vec<f64>)> {
        let n = self.sample_x.len();
        let start_idx = n - 2;
        let goal_idx = n - 1;

        let mut nodes: Vec<Node> = self
            .sample_x
            .iter()
            .zip(self.sample_y.iter())
            .map(|(&x, &y)| Node::new(x, y))
            .collect();

        nodes[start_idx].cost = 0.0;

        let mut open_set = BinaryHeap::new();
        open_set.push(QueueItem {
            cost: 0.0,
            index: start_idx,
        });

        let mut closed_set: HashMap<usize, bool> = HashMap::new();

        while let Some(current) = open_set.pop() {
            if current.index == goal_idx {
                return Some(self.reconstruct_path(&nodes, goal_idx));
            }

            if closed_set.contains_key(&current.index) {
                continue;
            }
            closed_set.insert(current.index, true);

            for &neighbor_idx in &self.road_map[current.index] {
                if closed_set.contains_key(&neighbor_idx) {
                    continue;
                }

                let dx = nodes[neighbor_idx].x - nodes[current.index].x;
                let dy = nodes[neighbor_idx].y - nodes[current.index].y;
                let edge_cost = (dx * dx + dy * dy).sqrt();
                let new_cost = nodes[current.index].cost + edge_cost;

                if new_cost < nodes[neighbor_idx].cost {
                    nodes[neighbor_idx].cost = new_cost;
                    nodes[neighbor_idx].parent = Some(current.index);
                    open_set.push(QueueItem {
                        cost: new_cost,
                        index: neighbor_idx,
                    });
                }
            }
        }

        None
    }

    fn reconstruct_path(&self, nodes: &[Node], goal_idx: usize) -> (Vec<f64>, Vec<f64>) {
        let mut path_x = Vec::new();
        let mut path_y = Vec::new();

        let mut current = goal_idx;
        while let Some(parent) = nodes[current].parent {
            path_x.push(nodes[current].x);
            path_y.push(nodes[current].y);
            current = parent;
        }
        path_x.push(nodes[current].x);
        path_y.push(nodes[current].y);

        path_x.reverse();
        path_y.reverse();

        (path_x, path_y)
    }

    /// Get sample points for external inspection
    pub fn get_samples(&self) -> (&[f64], &[f64]) {
        (&self.sample_x, &self.sample_y)
    }

    /// Get road map edges for external inspection
    pub fn get_edges(&self) -> Vec<((f64, f64), (f64, f64))> {
        let mut edges = Vec::new();

        for (i, neighbors) in self.road_map.iter().enumerate() {
            for &j in neighbors {
                if i < j {
                    edges.push((
                        (self.sample_x[i], self.sample_y[i]),
                        (self.sample_x[j], self.sample_y[j]),
                    ));
                }
            }
        }

        edges
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_prm_creation() {
        let mut ox = Vec::new();
        let mut oy = Vec::new();
        for i in 0..20 {
            ox.push(i as f64);
            oy.push(0.0);
            ox.push(i as f64);
            oy.push(20.0);
            ox.push(0.0);
            oy.push(i as f64);
            ox.push(20.0);
            oy.push(i as f64);
        }

        let prm = PRMPlanner::new(&ox, &oy, (2.0, 2.0), (18.0, 18.0), 2.0);
        let (sx, sy) = prm.get_samples();
        assert!(!sx.is_empty());
        assert_eq!(sx.len(), sy.len());
    }
}
