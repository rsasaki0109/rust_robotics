// Probabilistic Road-Map (PRM) path planning
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109)
//         Rust port

use rand::Rng;
use std::collections::{BinaryHeap, HashMap};
use std::cmp::Ordering;
use gnuplot::{Figure, Caption, Color, AxesCommon, PointSymbol, PointSize};

// Parameters
const N_SAMPLE: usize = 500; // number of sample points
const N_KNN: usize = 10; // number of edges per node
const MAX_EDGE_LEN: f64 = 30.0; // maximum edge length [m]
const ROBOT_RADIUS: f64 = 5.0; // robot radius [m]

const SHOW_ANIMATION: bool = true;

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
        // Reverse for min-heap
        other.cost.partial_cmp(&self.cost).unwrap_or(Ordering::Equal)
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

    /// Find k nearest neighbors
    fn query_knn(&self, x: f64, y: f64, k: usize) -> Vec<(usize, f64)> {
        let mut distances: Vec<(usize, f64)> = self.points
            .iter()
            .enumerate()
            .map(|(i, (px, py))| {
                let d = ((x - px).powi(2) + (y - py).powi(2)).sqrt();
                (i, d)
            })
            .collect();

        distances.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        distances.truncate(k + 1); // +1 because point itself is included
        distances
    }

    /// Find all points within radius
    fn query_radius(&self, x: f64, y: f64, radius: f64) -> Vec<(usize, f64)> {
        self.points
            .iter()
            .enumerate()
            .filter_map(|(i, (px, py))| {
                let d = ((x - px).powi(2) + (y - py).powi(2)).sqrt();
                if d <= radius {
                    Some((i, d))
                } else {
                    None
                }
            })
            .collect()
    }

    /// Get minimum distance to any point
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
    obstacle_tree: KDTree,
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

        // Calculate bounds
        let min_x = ox.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_x = ox.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_y = oy.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_y = oy.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        // Sample points
        let (sample_x, sample_y) = Self::sample_points(
            start, goal,
            min_x, max_x, min_y, max_y,
            robot_radius,
            &obstacle_tree,
        );

        // Generate road map
        let road_map = Self::generate_road_map(
            &sample_x, &sample_y,
            robot_radius,
            &obstacle_tree,
        );

        PRMPlanner {
            sample_x,
            sample_y,
            road_map,
            obstacle_tree,
        }
    }

    /// Sample collision-free points
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

            // Check collision
            let min_dist = obstacle_tree.min_distance(x, y);
            if min_dist > robot_radius {
                sample_x.push(x);
                sample_y.push(y);
            }
        }

        // Add start and goal
        sample_x.push(start.0);
        sample_y.push(start.1);
        sample_x.push(goal.0);
        sample_y.push(goal.1);

        (sample_x, sample_y)
    }

    /// Generate road map by connecting nearby samples
    fn generate_road_map(
        sample_x: &[f64],
        sample_y: &[f64],
        robot_radius: f64,
        obstacle_tree: &KDTree,
    ) -> Vec<Vec<usize>> {
        let sample_tree = KDTree::new(
            sample_x.iter().zip(sample_y.iter()).map(|(&x, &y)| (x, y)).collect()
        );

        let mut road_map: Vec<Vec<usize>> = vec![Vec::new(); sample_x.len()];

        for (i, (&x, &y)) in sample_x.iter().zip(sample_y.iter()).enumerate() {
            // Find k nearest neighbors
            let neighbors = sample_tree.query_knn(x, y, N_KNN);

            for (j, dist) in neighbors {
                if i == j {
                    continue;
                }

                // Check edge length
                if dist > MAX_EDGE_LEN {
                    continue;
                }

                // Check collision
                if !Self::is_collision(
                    x, y,
                    sample_x[j], sample_y[j],
                    robot_radius,
                    obstacle_tree,
                ) {
                    road_map[i].push(j);
                }
            }
        }

        road_map
    }

    /// Check if path between two points collides with obstacles
    fn is_collision(
        x1: f64, y1: f64,
        x2: f64, y2: f64,
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

        let mut nodes: Vec<Node> = self.sample_x
            .iter()
            .zip(self.sample_y.iter())
            .map(|(&x, &y)| Node::new(x, y))
            .collect();

        nodes[start_idx].cost = 0.0;

        let mut open_set = BinaryHeap::new();
        open_set.push(QueueItem { cost: 0.0, index: start_idx });

        let mut closed_set: HashMap<usize, bool> = HashMap::new();

        while let Some(current) = open_set.pop() {
            if current.index == goal_idx {
                // Found path, reconstruct
                return Some(self.reconstruct_path(&nodes, goal_idx));
            }

            if closed_set.contains_key(&current.index) {
                continue;
            }
            closed_set.insert(current.index, true);

            // Expand neighbors
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

        None // No path found
    }

    /// Reconstruct path from goal to start
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

    /// Get sample points for visualization
    pub fn get_samples(&self) -> (&[f64], &[f64]) {
        (&self.sample_x, &self.sample_y)
    }

    /// Get road map edges for visualization
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

fn main() {
    println!("PRM path planning start!");

    // Define obstacles (rectangle boundary + internal obstacles)
    let mut ox = Vec::new();
    let mut oy = Vec::new();

    // Boundary
    for i in -10..=60 {
        ox.push(i as f64);
        oy.push(-10.0);
    }
    for i in -10..=60 {
        ox.push(60.0);
        oy.push(i as f64);
    }
    for i in -10..=60 {
        ox.push(i as f64);
        oy.push(60.0);
    }
    for i in -10..=60 {
        ox.push(-10.0);
        oy.push(i as f64);
    }

    // Internal obstacles
    for i in -10..=40 {
        ox.push(20.0);
        oy.push(i as f64);
    }
    for i in 0..=40 {
        ox.push(40.0);
        oy.push(60.0 - i as f64);
    }

    let start = (0.0, 0.0);
    let goal = (50.0, 50.0);

    // Create PRM planner
    println!("Building road map...");
    let prm = PRMPlanner::new(&ox, &oy, start, goal, ROBOT_RADIUS);

    // Plan path
    println!("Planning path...");
    let path = prm.plan();

    // Visualization
    let mut fig = Figure::new();

    let (sample_x, sample_y) = prm.get_samples();

    fig.axes2d()
        .set_title("PRM Path Planning", &[])
        .set_x_label("x [m]", &[])
        .set_y_label("y [m]", &[])
        .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0))
        .points(&ox, &oy, &[Caption("Obstacles"), Color("black"), PointSymbol('.'), PointSize(0.5)])
        .points(sample_x, sample_y, &[Caption("Samples"), Color("gray"), PointSymbol('.'), PointSize(1.0)])
        .points(&[start.0], &[start.1], &[Caption("Start"), Color("blue"), PointSymbol('O'), PointSize(2.0)])
        .points(&[goal.0], &[goal.1], &[Caption("Goal"), Color("red"), PointSymbol('O'), PointSize(2.0)]);

    // Draw road map edges
    let edges = prm.get_edges();
    for ((x1, y1), (x2, y2)) in &edges {
        fig.axes2d().lines(&[*x1, *x2], &[*y1, *y2], &[Color("lightgray")]);
    }

    // Draw path
    if let Some((path_x, path_y)) = &path {
        fig.axes2d().lines(path_x, path_y, &[Caption("Path"), Color("green")]);
        println!("Path found!");
    } else {
        println!("No path found!");
    }

    if SHOW_ANIMATION {
        fig.show_and_keep_running().unwrap();
    }

    fig.save_to_svg("./img/path_planning/prm.svg", 640, 480).unwrap();
    println!("Plot saved to ./img/path_planning/prm.svg");

    println!("Done!");
}
