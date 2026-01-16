// Voronoi Road-Map path planning
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109)
//         Rust port

use rand::Rng;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::cmp::Ordering;
use gnuplot::{Figure, Caption, Color, AxesCommon, PointSymbol, PointSize};

// Parameters
const N_KNN: usize = 10; // number of edges per node
const MAX_EDGE_LEN: f64 = 30.0; // maximum edge length [m]
const ROBOT_RADIUS: f64 = 5.0; // robot radius [m]

const SHOW_ANIMATION: bool = false;

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

/// Simple Voronoi diagram computation using Fortune's algorithm approximation
/// This is a simplified version that extracts vertices from obstacle positions
fn compute_voronoi_vertices(ox: &[f64], oy: &[f64], min_x: f64, max_x: f64, min_y: f64, max_y: f64, robot_radius: f64) -> Vec<(f64, f64)> {
    let mut vertices = Vec::new();
    let n = ox.len();

    if n < 3 {
        return vertices;
    }

    // For each pair of obstacles, compute perpendicular bisector intersections
    // This is a simplified Voronoi approximation
    for i in 0..n {
        for j in (i + 1)..n {
            // Midpoint of two obstacles
            let mx = (ox[i] + ox[j]) / 2.0;
            let my = (oy[i] + oy[j]) / 2.0;

            // Check if midpoint is far enough from obstacles
            let dist_i = ((mx - ox[i]).powi(2) + (my - oy[i]).powi(2)).sqrt();
            let dist_j = ((mx - ox[j]).powi(2) + (my - oy[j]).powi(2)).sqrt();

            if dist_i > robot_radius * 1.5 && dist_j > robot_radius * 1.5 {
                if mx > min_x && mx < max_x && my > min_y && my < max_y {
                    vertices.push((mx, my));
                }
            }
        }
    }

    // Add circumcenters of obstacle triangles (Delaunay dual)
    for i in 0..n {
        for j in (i + 1)..n {
            for k in (j + 1)..n {
                if let Some((cx, cy)) = circumcenter(ox[i], oy[i], ox[j], oy[j], ox[k], oy[k]) {
                    // Check if circumcenter is inside bounds and far from obstacles
                    if cx > min_x && cx < max_x && cy > min_y && cy < max_y {
                        let min_dist = ox.iter().zip(oy.iter())
                            .map(|(&px, &py)| ((cx - px).powi(2) + (cy - py).powi(2)).sqrt())
                            .fold(f64::INFINITY, f64::min);

                        if min_dist > robot_radius * 1.5 {
                            vertices.push((cx, cy));
                        }
                    }
                }
            }
        }
    }

    // Remove duplicates
    let mut unique_vertices = Vec::new();
    for (x, y) in vertices {
        let is_dup = unique_vertices.iter().any(|(vx, vy): &(f64, f64)| {
            ((x - vx).powi(2) + (y - vy).powi(2)).sqrt() < 1.0
        });
        if !is_dup {
            unique_vertices.push((x, y));
        }
    }

    unique_vertices
}

/// Compute circumcenter of three points
fn circumcenter(x1: f64, y1: f64, x2: f64, y2: f64, x3: f64, y3: f64) -> Option<(f64, f64)> {
    let d = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

    if d.abs() < 1e-10 {
        return None; // Collinear points
    }

    let ux = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2)) / d;
    let uy = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1)) / d;

    Some((ux, uy))
}

/// Voronoi Road-Map Planner
pub struct VoronoiPlanner {
    sample_x: Vec<f64>,
    sample_y: Vec<f64>,
    road_map: Vec<Vec<usize>>,
    obstacle_tree: KDTree,
}

impl VoronoiPlanner {
    /// Create a new Voronoi Road-Map planner
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

        // Get Voronoi vertices as sample points
        let voronoi_vertices = compute_voronoi_vertices(ox, oy, min_x, max_x, min_y, max_y, robot_radius);

        let mut sample_x: Vec<f64> = voronoi_vertices.iter().map(|(x, _)| *x).collect();
        let mut sample_y: Vec<f64> = voronoi_vertices.iter().map(|(_, y)| *y).collect();

        // Add start and goal
        sample_x.push(start.0);
        sample_y.push(start.1);
        sample_x.push(goal.0);
        sample_y.push(goal.1);

        // Generate road map
        let road_map = Self::generate_road_map(
            &sample_x, &sample_y,
            robot_radius,
            &obstacle_tree,
        );

        VoronoiPlanner {
            sample_x,
            sample_y,
            road_map,
            obstacle_tree,
        }
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
            let neighbors = sample_tree.query_knn(x, y, N_KNN);

            for (j, dist) in neighbors {
                if i == j {
                    continue;
                }

                if dist > MAX_EDGE_LEN {
                    continue;
                }

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
        if n < 2 {
            return None;
        }

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
    println!("Voronoi Road-Map path planning start!");

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

    // Create Voronoi planner
    println!("Building Voronoi road map...");
    let planner = VoronoiPlanner::new(&ox, &oy, start, goal, ROBOT_RADIUS);

    // Plan path
    println!("Planning path...");
    let path = planner.plan();

    // Visualization
    let mut fig = Figure::new();

    let (sample_x, sample_y) = planner.get_samples();
    let edges = planner.get_edges();


    // Prepare edge data as disconnected line segments using NaN separator
    let mut edge_x = Vec::new();
    let mut edge_y = Vec::new();
    for ((x1, y1), (x2, y2)) in &edges {
        edge_x.push(*x1);
        edge_x.push(*x2);
        edge_x.push(f64::NAN); // NaN separates line segments
        edge_y.push(*y1);
        edge_y.push(*y2);
        edge_y.push(f64::NAN);
    }

    {
        let axes = fig.axes2d();
        axes.set_title("Voronoi Road-Map Path Planning", &[])
            .set_x_label("x [m]", &[])
            .set_y_label("y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));

        // Draw road map edges as single dataset with NaN separators
        axes.lines(&edge_x, &edge_y, &[Color("gray")]);

        // Draw obstacles and samples
        axes.points(&ox, &oy, &[Caption("Obstacles"), Color("black"), PointSymbol('.'), PointSize(0.5)])
            .points(sample_x, sample_y, &[Caption("Voronoi Vertices"), Color("cyan"), PointSymbol('o'), PointSize(1.0)])
            .points(&[start.0], &[start.1], &[Caption("Start"), Color("blue"), PointSymbol('O'), PointSize(2.0)])
            .points(&[goal.0], &[goal.1], &[Caption("Goal"), Color("red"), PointSymbol('O'), PointSize(2.0)]);

        // Draw path
        if let Some((path_x, path_y)) = &path {
            axes.lines(path_x, path_y, &[Caption("Path"), Color("green")]);
            println!("Path found with {} waypoints!", path_x.len());
        } else {
            println!("No path found!");
        }
    }

    if SHOW_ANIMATION {
        fig.show_and_keep_running().unwrap();
    }

    fig.save_to_svg("./img/path_planning/voronoi_road_map.svg", 640, 480).unwrap();
    fig.close();
    println!("Plot saved to ./img/path_planning/voronoi_road_map.svg");

    println!("Done!");
}
