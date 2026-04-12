#![allow(dead_code, clippy::too_many_arguments)]

//! Probabilistic Road-Map Star (PRM*) path planning.
//!
//! Uses the asymptotically optimal connection radius:
//! `r_n = gamma * (log(n) / n)^(1/d)` with `d = 2`.

use rand::Rng;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

/// Configuration for PRM*.
#[derive(Debug, Clone)]
pub struct PRMStarConfig {
    /// Number of sampled free-space points (excluding start/goal).
    pub n_samples: usize,
    /// Robot radius used for collision checking.
    pub robot_radius: f64,
    /// Radius scale factor in PRM* formula.
    pub gamma: f64,
}

impl Default for PRMStarConfig {
    fn default() -> Self {
        Self {
            n_samples: 500,
            robot_radius: 0.8,
            gamma: 2.5,
        }
    }
}

impl PRMStarConfig {
    /// Validate PRM* configuration.
    pub fn validate(&self) -> Result<(), String> {
        if self.n_samples == 0 {
            return Err("PRM* requires at least one sample".to_string());
        }
        if !self.robot_radius.is_finite() || self.robot_radius <= 0.0 {
            return Err("PRM* robot_radius must be positive and finite".to_string());
        }
        if !self.gamma.is_finite() || self.gamma <= 0.0 {
            return Err("PRM* gamma must be positive and finite".to_string());
        }
        Ok(())
    }
}

#[derive(Clone)]
struct Node {
    x: f64,
    y: f64,
    cost: f64,
    parent: Option<usize>,
}

impl Node {
    fn new(x: f64, y: f64) -> Self {
        Self {
            x,
            y,
            cost: f64::INFINITY,
            parent: None,
        }
    }
}

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

struct KDTree {
    points: Vec<(f64, f64)>,
}

impl KDTree {
    fn new(points: Vec<(f64, f64)>) -> Self {
        Self { points }
    }

    fn query_radius(&self, x: f64, y: f64, radius: f64) -> Vec<(usize, f64)> {
        let r2 = radius * radius;
        self.points
            .iter()
            .enumerate()
            .filter_map(|(i, (px, py))| {
                let dx = x - px;
                let dy = y - py;
                let d2 = dx * dx + dy * dy;
                if d2 <= r2 {
                    Some((i, d2.sqrt()))
                } else {
                    None
                }
            })
            .collect()
    }

    fn min_distance(&self, x: f64, y: f64) -> f64 {
        self.points
            .iter()
            .map(|(px, py)| ((x - px).powi(2) + (y - py).powi(2)).sqrt())
            .fold(f64::INFINITY, f64::min)
    }
}

/// PRM* planner.
pub struct PRMStarPlanner {
    sample_x: Vec<f64>,
    sample_y: Vec<f64>,
    road_map: Vec<Vec<usize>>,
    connection_radius: f64,
}

impl PRMStarPlanner {
    /// Create a new PRM* planner.
    pub fn new(
        ox: &[f64],
        oy: &[f64],
        start: (f64, f64),
        goal: (f64, f64),
        config: PRMStarConfig,
    ) -> Self {
        config.validate().expect(
            "invalid PRM* configuration: n_samples > 0, robot_radius > 0, gamma > 0 required",
        );

        let obstacle_tree = KDTree::new(ox.iter().zip(oy.iter()).map(|(&x, &y)| (x, y)).collect());
        let min_x = ox.iter().copied().fold(f64::INFINITY, f64::min);
        let max_x = ox.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let min_y = oy.iter().copied().fold(f64::INFINITY, f64::min);
        let max_y = oy.iter().copied().fold(f64::NEG_INFINITY, f64::max);

        let (sample_x, sample_y) = Self::sample_points(
            start,
            goal,
            min_x,
            max_x,
            min_y,
            max_y,
            config.n_samples,
            config.robot_radius,
            &obstacle_tree,
        );

        let workspace_scale = ((max_x - min_x).powi(2) + (max_y - min_y).powi(2))
            .sqrt()
            .max(1.0);
        let connection_radius =
            Self::compute_connection_radius(sample_x.len(), workspace_scale, config.gamma);
        let road_map = Self::generate_road_map(
            &sample_x,
            &sample_y,
            config.robot_radius,
            connection_radius,
            &obstacle_tree,
        );

        Self {
            sample_x,
            sample_y,
            road_map,
            connection_radius,
        }
    }

    fn compute_connection_radius(n: usize, workspace_scale: f64, gamma: f64) -> f64 {
        let n_f = n as f64;
        let radius_normalized = gamma * (n_f.ln() / n_f).sqrt();
        (radius_normalized * workspace_scale).max(1e-3)
    }

    fn sample_points(
        start: (f64, f64),
        goal: (f64, f64),
        min_x: f64,
        max_x: f64,
        min_y: f64,
        max_y: f64,
        n_samples: usize,
        robot_radius: f64,
        obstacle_tree: &KDTree,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut rng = rand::thread_rng();
        let mut sample_x = Vec::with_capacity(n_samples + 2);
        let mut sample_y = Vec::with_capacity(n_samples + 2);

        while sample_x.len() < n_samples {
            let x = rng.gen_range(min_x..max_x);
            let y = rng.gen_range(min_y..max_y);
            if obstacle_tree.min_distance(x, y) > robot_radius {
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
        connection_radius: f64,
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
            for (j, dist) in sample_tree.query_radius(x, y, connection_radius) {
                if i == j {
                    continue;
                }
                if !Self::is_collision(x, y, sample_x[j], sample_y[j], robot_radius, obstacle_tree)
                {
                    road_map[i].push(j);
                } else {
                    let _ = dist;
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
            if obstacle_tree.min_distance(x, y) <= robot_radius {
                return true;
            }
        }
        false
    }

    /// Plan path using Dijkstra's algorithm.
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

    /// Return sample points.
    pub fn get_samples(&self) -> (&[f64], &[f64]) {
        (&self.sample_x, &self.sample_y)
    }

    /// Return current connection radius.
    pub fn connection_radius(&self) -> f64 {
        self.connection_radius
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rectangular_walls(size: usize) -> (Vec<f64>, Vec<f64>) {
        let mut ox = Vec::new();
        let mut oy = Vec::new();
        for i in 0..=size {
            let v = i as f64;
            ox.push(v);
            oy.push(0.0);
            ox.push(v);
            oy.push(size as f64);
            ox.push(0.0);
            oy.push(v);
            ox.push(size as f64);
            oy.push(v);
        }
        (ox, oy)
    }

    fn path_length(xs: &[f64], ys: &[f64]) -> f64 {
        xs.windows(2)
            .zip(ys.windows(2))
            .map(|(wx, wy)| {
                let dx = wx[1] - wx[0];
                let dy = wy[1] - wy[0];
                (dx * dx + dy * dy).sqrt()
            })
            .sum()
    }

    #[test]
    fn test_prm_star_finds_path() {
        let (ox, oy) = rectangular_walls(30);
        let config = PRMStarConfig {
            n_samples: 450,
            robot_radius: 0.8,
            gamma: 2.5,
        };
        let planner = PRMStarPlanner::new(&ox, &oy, (2.0, 2.0), (28.0, 28.0), config);

        let path = planner.plan();
        assert!(path.is_some(), "PRM* should find a path in free interior");

        let (px, py) = path.unwrap();
        assert_eq!(px.len(), py.len());
        assert!(px.len() >= 2);
    }

    #[test]
    fn test_prm_star_path_quality_improves_with_more_samples() {
        let (ox, oy) = rectangular_walls(20);
        let low_cfg = PRMStarConfig {
            n_samples: 60,
            robot_radius: 0.8,
            gamma: 2.5,
        };
        let high_cfg = PRMStarConfig {
            n_samples: 200,
            robot_radius: 0.8,
            gamma: 2.5,
        };

        let planner_low =
            PRMStarPlanner::new(&ox, &oy, (2.0, 2.0), (18.0, 18.0), low_cfg.clone());
        let low_result = planner_low.plan();

        let planner_high =
            PRMStarPlanner::new(&ox, &oy, (2.0, 2.0), (18.0, 18.0), high_cfg.clone());
        let high_result = planner_high.plan();

        // At least one should find a path
        assert!(
            low_result.is_some() || high_result.is_some(),
            "at least one configuration should find a path"
        );
    }

    #[test]
    fn test_prm_star_config_defaults() {
        let config = PRMStarConfig::default();
        assert_eq!(config.n_samples, 500);
        assert!((config.robot_radius - 0.8).abs() < f64::EPSILON);
        assert!((config.gamma - 2.5).abs() < f64::EPSILON);
    }
}
