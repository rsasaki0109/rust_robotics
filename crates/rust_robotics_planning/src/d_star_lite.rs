#![allow(
    dead_code,
    clippy::legacy_numeric_constants
)]

//! D* Lite path planning algorithm
//!
//! Incremental heuristic search algorithm that efficiently replans
//! when the graph changes (e.g., new obstacles detected).

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashSet};
use std::f64::INFINITY;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct Node {
    x: i32,
    y: i32,
}

impl Node {
    pub fn new(x: i32, y: i32) -> Self {
        Node { x, y }
    }

    fn add(&self, other: &Node) -> Node {
        Node {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }

    fn distance_to(&self, other: &Node) -> f64 {
        let dx = (self.x - other.x) as f64;
        let dy = (self.y - other.y) as f64;
        (dx * dx + dy * dy).sqrt()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Key {
    k1: f64,
    k2: f64,
}

impl Eq for Key {}

impl PartialOrd for Key {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Key {
    fn cmp(&self, other: &Self) -> Ordering {
        match self.k1.partial_cmp(&other.k1) {
            Some(Ordering::Equal) => self.k2.partial_cmp(&other.k2).unwrap_or(Ordering::Equal),
            Some(ord) => ord,
            None => Ordering::Equal,
        }
    }
}

#[derive(Debug)]
pub struct DStarLite {
    x_min_world: i32,
    y_min_world: i32,
    x_max: i32,
    y_max: i32,
    start: Node,
    goal: Node,
    km: f64,
    rhs: nalgebra::DMatrix<f64>,
    g: nalgebra::DMatrix<f64>,
    u: BinaryHeap<(Key, Node)>,
    obstacles: HashSet<(i32, i32)>,
    detected_obstacles: HashSet<(i32, i32)>,
    motions: [Node; 8],
    detected_obstacles_for_plotting: Vec<(f64, f64)>,
    initialized: bool,
}

impl DStarLite {
    pub fn new(ox: &[i32], oy: &[i32]) -> Self {
        let x_min_world = *ox.iter().min().unwrap_or(&0);
        let y_min_world = *oy.iter().min().unwrap_or(&0);
        let x_max = ox.iter().max().unwrap_or(&0) - x_min_world + 1;
        let y_max = oy.iter().max().unwrap_or(&0) - y_min_world + 1;

        let obstacles: HashSet<_> = ox
            .iter()
            .zip(oy.iter())
            .map(|(&x, &y)| (x - x_min_world, y - y_min_world))
            .collect();

        let g = nalgebra::DMatrix::from_element(x_max as usize, y_max as usize, INFINITY);
        let rhs = nalgebra::DMatrix::from_element(x_max as usize, y_max as usize, INFINITY);

        let motions = [
            Node::new(1, 0),
            Node::new(0, 1),
            Node::new(-1, 0),
            Node::new(0, -1),
            Node::new(1, 1),
            Node::new(1, -1),
            Node::new(-1, 1),
            Node::new(-1, -1),
        ];

        DStarLite {
            x_min_world,
            y_min_world,
            x_max,
            y_max,
            start: Node::new(0, 0),
            goal: Node::new(0, 0),
            km: 0.0,
            rhs,
            g,
            u: BinaryHeap::new(),
            obstacles,
            detected_obstacles: HashSet::new(),
            motions,
            detected_obstacles_for_plotting: Vec::new(),
            initialized: false,
        }
    }

    fn is_valid(&self, node: &Node) -> bool {
        node.x >= 0 && node.x < self.x_max && node.y >= 0 && node.y < self.y_max
    }

    fn is_obstacle(&self, node: &Node) -> bool {
        self.obstacles.contains(&(node.x, node.y))
            || self.detected_obstacles.contains(&(node.x, node.y))
    }

    fn get_neighbors(&self, node: &Node) -> Vec<Node> {
        self.motions
            .iter()
            .map(|m| node.add(m))
            .filter(|n| self.is_valid(n))
            .collect()
    }

    fn calculate_key(&self, node: &Node) -> Key {
        let min_rhs_g = self.g[(node.x as usize, node.y as usize)]
            .min(self.rhs[(node.x as usize, node.y as usize)]);
        let h = self.heuristic(&self.start, node);
        Key {
            k1: min_rhs_g + h + self.km,
            k2: min_rhs_g,
        }
    }

    fn heuristic(&self, a: &Node, b: &Node) -> f64 {
        let dx = (a.x - b.x).abs() as f64;
        let dy = (a.y - b.y).abs() as f64;
        (dx * dx + dy * dy).sqrt()
    }

    fn c(&self, _u: &Node, v: &Node) -> f64 {
        if self.is_obstacle(v) {
            return INFINITY;
        }
        self.heuristic(_u, v)
    }

    fn initialize(&mut self, start: Node, goal: Node) {
        self.start = Node::new(start.x - self.x_min_world, start.y - self.y_min_world);
        self.goal = Node::new(goal.x - self.x_min_world, goal.y - self.y_min_world);

        if !self.initialized {
            self.initialized = true;
            self.u.clear();
            self.km = 0.0;
            self.rhs =
                nalgebra::DMatrix::from_element(self.x_max as usize, self.y_max as usize, INFINITY);
            self.g = nalgebra::DMatrix::from_element(self.x_max as usize, self.y_max as usize, INFINITY);
            self.rhs[(self.goal.x as usize, self.goal.y as usize)] = 0.0;
            self.u.push((self.calculate_key(&self.goal), self.goal));
            self.detected_obstacles.clear();
        }
    }

    fn update_vertex(&mut self, u: Node) {
        if u.x != self.goal.x || u.y != self.goal.y {
            let mut min_rhs = INFINITY;
            for s in self.get_neighbors(&u) {
                let cost = self.c(&u, &s) + self.g[(s.x as usize, s.y as usize)];
                if cost < min_rhs {
                    min_rhs = cost;
                }
            }
            self.rhs[(u.x as usize, u.y as usize)] = min_rhs;
        }

        self.u = std::mem::take(&mut self.u)
            .into_iter()
            .filter(|(_, node)| node.x != u.x || node.y != u.y)
            .collect();

        if (self.g[(u.x as usize, u.y as usize)] - self.rhs[(u.x as usize, u.y as usize)]).abs()
            > 1e-6
        {
            self.u.push((self.calculate_key(&u), u));
        }
    }

    fn compute_shortest_path(&mut self) {
        while !self.u.is_empty() {
            let (k_old, u) = self.u.pop().unwrap();

            if k_old < self.calculate_key(&u) {
                self.u.push((self.calculate_key(&u), u));
            } else if self.g[(u.x as usize, u.y as usize)] > self.rhs[(u.x as usize, u.y as usize)]
            {
                self.g[(u.x as usize, u.y as usize)] = self.rhs[(u.x as usize, u.y as usize)];

                for s in self.get_neighbors(&u) {
                    self.update_vertex(s);
                }
            } else {
                self.g[(u.x as usize, u.y as usize)] = INFINITY;
                self.update_vertex(u);
                for s in self.get_neighbors(&u) {
                    self.update_vertex(s);
                }
            }
        }
    }

    fn plan(&mut self, start: Node, goal: Node) -> Option<Vec<(i32, i32)>> {
        self.initialize(
            Node::new(start.x, start.y),
            Node::new(goal.x, goal.y),
        );
        self.compute_shortest_path();
        self.reconstruct_path()
    }

    /// Plan a path from integer coordinates
    pub fn plan_xy(&mut self, sx: i32, sy: i32, gx: i32, gy: i32) -> Option<Vec<(i32, i32)>> {
        self.plan(Node::new(sx, sy), Node::new(gx, gy))
    }

    fn reconstruct_path(&self) -> Option<Vec<(i32, i32)>> {
        let mut path = Vec::new();
        let mut current = self.start;

        while current.x != self.goal.x || current.y != self.goal.y {
            path.push((
                current.x + self.x_min_world,
                current.y + self.y_min_world,
            ));

            let mut min_cost = INFINITY;
            let mut next_node = current;

            for neighbor in self.get_neighbors(&current) {
                let cost = self.c(&current, &neighbor)
                    + self.g[(neighbor.x as usize, neighbor.y as usize)];
                if cost < min_cost {
                    min_cost = cost;
                    next_node = neighbor;
                }
            }

            if next_node.x == current.x && next_node.y == current.y {
                return None;
            }

            current = next_node;
        }

        path.push((
            self.goal.x + self.x_min_world,
            self.goal.y + self.y_min_world,
        ));

        Some(path)
    }

    pub fn add_obstacle(&mut self, x: i32, y: i32) {
        let node = Node::new(x - self.x_min_world, y - self.y_min_world);
        if self.is_valid(&node) {
            self.detected_obstacles.insert((node.x, node.y));
            self.detected_obstacles_for_plotting
                .push((x as f64, y as f64));
            self.update_vertex(node);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_d_star_lite_creation() {
        let ox: Vec<i32> = (0..10).collect();
        let oy: Vec<i32> = vec![0; 10];
        let dstar = DStarLite::new(&ox, &oy);
        assert!(dstar.x_max > 0);
        assert!(dstar.y_max > 0);
    }
}
