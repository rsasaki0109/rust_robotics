#![allow(dead_code)]

//! Visibility Road-Map path planning
//!
//! Builds a graph by connecting start, goal, and expanded obstacle vertices
//! that have line-of-sight to each other (no obstacle edge intersections),
//! then searches the graph with Dijkstra's algorithm.
//!
//! Reference: PythonRobotics VisibilityRoadMap

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

/// A 2-D point.
#[derive(Clone, Copy, Debug)]
struct Point {
    x: f64,
    y: f64,
}

/// Orientation of an ordered triplet (p, q, r).
/// Returns 0 if collinear, 1 if clockwise, 2 if counter-clockwise.
fn orientation(p: Point, q: Point, r: Point) -> i32 {
    let val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if val > 0.0 {
        1
    } else if val < 0.0 {
        2
    } else {
        0
    }
}

/// Check whether point `q` lies on segment `pr`.
fn on_segment(p: Point, q: Point, r: Point) -> bool {
    q.x <= p.x.max(r.x) && q.x >= p.x.min(r.x) && q.y <= p.y.max(r.y) && q.y >= p.y.min(r.y)
}

/// Return `true` when segments (p1, q1) and (p2, q2) intersect (proper or
/// collinear-touching).
fn segments_intersect(p1: Point, q1: Point, p2: Point, q2: Point) -> bool {
    let o1 = orientation(p1, q1, p2);
    let o2 = orientation(p1, q1, q2);
    let o3 = orientation(p2, q2, p1);
    let o4 = orientation(p2, q2, q1);

    if o1 != o2 && o3 != o4 {
        return true;
    }
    if o1 == 0 && on_segment(p1, p2, q1) {
        return true;
    }
    if o2 == 0 && on_segment(p1, q2, q1) {
        return true;
    }
    if o3 == 0 && on_segment(p2, p1, q2) {
        return true;
    }
    if o4 == 0 && on_segment(p2, q1, q2) {
        return true;
    }
    false
}

// ---------------------------------------------------------------------------
// Obstacle polygon
// ---------------------------------------------------------------------------

/// A closed polygon obstacle.
///
/// Vertices are stored in clockwise order with the last vertex equal to the
/// first (closed).
#[derive(Clone, Debug)]
pub struct ObstaclePolygon {
    /// x-coordinates (closed: first == last).
    pub x_list: Vec<f64>,
    /// y-coordinates (closed: first == last).
    pub y_list: Vec<f64>,
}

impl ObstaclePolygon {
    /// Create a new obstacle polygon from vertex lists.
    ///
    /// The polygon is automatically closed and reordered to clockwise.
    pub fn new(mut x_list: Vec<f64>, mut y_list: Vec<f64>) -> Self {
        assert!(
            x_list.len() >= 3 && x_list.len() == y_list.len(),
            "polygon must have at least 3 vertices"
        );

        // Close the polygon if not already closed.
        let is_closed = (x_list[0] - *x_list.last().unwrap()).abs() < 1e-12
            && (y_list[0] - *y_list.last().unwrap()).abs() < 1e-12;
        if !is_closed {
            x_list.push(x_list[0]);
            y_list.push(y_list[0]);
        }

        // Ensure clockwise winding.
        if !Self::is_clockwise(&x_list, &y_list) {
            x_list.reverse();
            y_list.reverse();
        }

        ObstaclePolygon { x_list, y_list }
    }

    fn is_clockwise(x: &[f64], y: &[f64]) -> bool {
        let n = x.len();
        let mut sum = 0.0;
        for i in 0..n - 1 {
            sum += (x[i + 1] - x[i]) * (y[i + 1] + y[i]);
        }
        sum += (x[0] - x[n - 1]) * (y[0] + y[n - 1]);
        sum >= 0.0
    }

    /// Number of unique vertices (excluding the closing duplicate).
    fn vertex_count(&self) -> usize {
        self.x_list.len() - 1
    }
}

// ---------------------------------------------------------------------------
// Dijkstra helpers (self-contained, same pattern as voronoi_road_map)
// ---------------------------------------------------------------------------

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

/// Run Dijkstra on an adjacency-list graph, returning the path as a sequence
/// of node indices from `start_idx` to `goal_idx`, or `None`.
fn dijkstra_search(
    nodes: &mut [Node],
    road_map: &[Vec<usize>],
    start_idx: usize,
    goal_idx: usize,
) -> Option<Vec<usize>> {
    nodes[start_idx].cost = 0.0;

    let mut open = BinaryHeap::new();
    open.push(QueueItem {
        cost: 0.0,
        index: start_idx,
    });

    let mut closed: HashMap<usize, bool> = HashMap::new();

    while let Some(cur) = open.pop() {
        if cur.index == goal_idx {
            // Reconstruct.
            let mut path = Vec::new();
            let mut idx = goal_idx;
            loop {
                path.push(idx);
                match nodes[idx].parent {
                    Some(p) => idx = p,
                    None => break,
                }
            }
            path.reverse();
            return Some(path);
        }

        if closed.contains_key(&cur.index) {
            continue;
        }
        closed.insert(cur.index, true);

        for &nb in &road_map[cur.index] {
            if closed.contains_key(&nb) {
                continue;
            }
            let dx = nodes[nb].x - nodes[cur.index].x;
            let dy = nodes[nb].y - nodes[cur.index].y;
            let new_cost = nodes[cur.index].cost + (dx * dx + dy * dy).sqrt();
            if new_cost < nodes[nb].cost {
                nodes[nb].cost = new_cost;
                nodes[nb].parent = Some(cur.index);
                open.push(QueueItem {
                    cost: new_cost,
                    index: nb,
                });
            }
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Visibility Road-Map planner
// ---------------------------------------------------------------------------

/// Visibility Road-Map planner.
///
/// Given polygon obstacles, the planner expands each obstacle vertex outward
/// by `expand_distance`, connects all mutually visible vertices (including
/// start and goal), and finds the shortest path via Dijkstra.
pub struct VisibilityRoadMap {
    /// Distance to expand obstacle vertices outward in configuration space.
    expand_distance: f64,
}

impl VisibilityRoadMap {
    /// Create a new planner.
    pub fn new(expand_distance: f64) -> Self {
        VisibilityRoadMap { expand_distance }
    }

    /// Plan a path from `(sx, sy)` to `(gx, gy)` avoiding `obstacles`.
    ///
    /// Returns the path as `(xs, ys)` vectors, or `None` if unreachable.
    pub fn plan(
        &self,
        sx: f64,
        sy: f64,
        gx: f64,
        gy: f64,
        obstacles: &[ObstaclePolygon],
    ) -> Option<(Vec<f64>, Vec<f64>)> {
        let mut nodes = self.generate_visibility_nodes(sx, sy, gx, gy, obstacles);
        let road_map = self.generate_road_map(&nodes, obstacles);

        let path_indices = dijkstra_search(&mut nodes, &road_map, 0, 1)?;

        let rx: Vec<f64> = path_indices.iter().map(|&i| nodes[i].x).collect();
        let ry: Vec<f64> = path_indices.iter().map(|&i| nodes[i].y).collect();
        Some((rx, ry))
    }

    // -- node generation ----------------------------------------------------

    fn generate_visibility_nodes(
        &self,
        sx: f64,
        sy: f64,
        gx: f64,
        gy: f64,
        obstacles: &[ObstaclePolygon],
    ) -> Vec<Node> {
        let mut nodes = vec![Node::new(sx, sy), Node::new(gx, gy)];

        for obs in obstacles {
            let (cvx, cvy) = self.vertices_in_configuration_space(obs);
            for (vx, vy) in cvx.into_iter().zip(cvy) {
                nodes.push(Node::new(vx, vy));
            }
        }

        nodes
    }

    /// Expand each obstacle vertex outward by `expand_distance`.
    fn vertices_in_configuration_space(&self, obs: &ObstaclePolygon) -> (Vec<f64>, Vec<f64>) {
        let n = obs.vertex_count();
        // Work with the unique vertices (excluding closing duplicate).
        let x = &obs.x_list[..n];
        let y = &obs.y_list[..n];

        let mut cvx = Vec::with_capacity(n);
        let mut cvy = Vec::with_capacity(n);

        for i in 0..n {
            let prev = if i == 0 { n - 1 } else { i - 1 };
            let next = (i + 1) % n;
            let (ox, oy) = self.calc_offset(x[prev], y[prev], x[i], y[i], x[next], y[next]);
            cvx.push(ox);
            cvy.push(oy);
        }

        (cvx, cvy)
    }

    /// Compute the offset position for a vertex given its predecessor and successor.
    fn calc_offset(&self, px: f64, py: f64, x: f64, y: f64, nx: f64, ny: f64) -> (f64, f64) {
        let p_vec = (y - py).atan2(x - px);
        let n_vec = (ny - y).atan2(nx - x);
        let offset_vec = (p_vec.sin() + n_vec.sin()).atan2(p_vec.cos() + n_vec.cos())
            + std::f64::consts::FRAC_PI_2;
        let offset_x = x + self.expand_distance * offset_vec.cos();
        let offset_y = y + self.expand_distance * offset_vec.sin();
        (offset_x, offset_y)
    }

    // -- road-map generation ------------------------------------------------

    fn generate_road_map(&self, nodes: &[Node], obstacles: &[ObstaclePolygon]) -> Vec<Vec<usize>> {
        let n = nodes.len();
        let mut road_map: Vec<Vec<usize>> = Vec::with_capacity(n);

        for i in 0..n {
            let mut neighbors = Vec::new();
            for j in 0..n {
                if i == j {
                    continue;
                }
                let dx = nodes[i].x - nodes[j].x;
                let dy = nodes[i].y - nodes[j].y;
                if (dx * dx + dy * dy).sqrt() <= 0.1 {
                    continue; // nearly coincident
                }

                if obstacles
                    .iter()
                    .all(|obs| Self::is_edge_valid(&nodes[i], &nodes[j], obs))
                {
                    neighbors.push(j);
                }
            }
            road_map.push(neighbors);
        }

        road_map
    }

    /// Return `true` when the segment between two nodes does not cross any
    /// edge of the given obstacle polygon.
    fn is_edge_valid(a: &Node, b: &Node, obs: &ObstaclePolygon) -> bool {
        let p1 = Point { x: a.x, y: a.y };
        let p2 = Point { x: b.x, y: b.y };

        for i in 0..obs.x_list.len() - 1 {
            let p3 = Point {
                x: obs.x_list[i],
                y: obs.y_list[i],
            };
            let p4 = Point {
                x: obs.x_list[i + 1],
                y: obs.y_list[i + 1],
            };
            if segments_intersect(p1, p2, p3, p4) {
                return false;
            }
        }
        true
    }

    /// Return the road-map edges for visualisation purposes.
    ///
    /// Each entry is `((x1, y1), (x2, y2))` with `i < j` to avoid duplicates.
    pub fn get_edges(
        &self,
        sx: f64,
        sy: f64,
        gx: f64,
        gy: f64,
        obstacles: &[ObstaclePolygon],
    ) -> Vec<((f64, f64), (f64, f64))> {
        let nodes = self.generate_visibility_nodes(sx, sy, gx, gy, obstacles);
        let road_map = self.generate_road_map(&nodes, obstacles);

        let mut edges = Vec::new();
        for (i, neighbors) in road_map.iter().enumerate() {
            for &j in neighbors {
                if i < j {
                    edges.push(((nodes[i].x, nodes[i].y), (nodes[j].x, nodes[j].y)));
                }
            }
        }
        edges
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Build the same obstacle set used in the Python reference `main()`.
    fn make_obstacles() -> Vec<ObstaclePolygon> {
        vec![
            ObstaclePolygon::new(vec![20.0, 30.0, 15.0], vec![20.0, 20.0, 30.0]),
            ObstaclePolygon::new(vec![40.0, 45.0, 50.0, 40.0], vec![50.0, 40.0, 20.0, 40.0]),
            ObstaclePolygon::new(vec![20.0, 30.0, 30.0, 20.0], vec![40.0, 45.0, 60.0, 50.0]),
        ]
    }

    // -- geometry -----------------------------------------------------------

    #[test]
    fn test_segments_intersect_crossing() {
        let p1 = Point { x: 0.0, y: 0.0 };
        let q1 = Point { x: 10.0, y: 10.0 };
        let p2 = Point { x: 10.0, y: 0.0 };
        let q2 = Point { x: 0.0, y: 10.0 };
        assert!(segments_intersect(p1, q1, p2, q2));
    }

    #[test]
    fn test_segments_no_intersect() {
        let p1 = Point { x: 0.0, y: 0.0 };
        let q1 = Point { x: 5.0, y: 0.0 };
        let p2 = Point { x: 0.0, y: 1.0 };
        let q2 = Point { x: 5.0, y: 1.0 };
        assert!(!segments_intersect(p1, q1, p2, q2));
    }

    #[test]
    fn test_segments_collinear_overlap() {
        let p1 = Point { x: 0.0, y: 0.0 };
        let q1 = Point { x: 5.0, y: 0.0 };
        let p2 = Point { x: 3.0, y: 0.0 };
        let q2 = Point { x: 7.0, y: 0.0 };
        assert!(segments_intersect(p1, q1, p2, q2));
    }

    // -- obstacle polygon ---------------------------------------------------

    #[test]
    fn test_polygon_closes_and_clockwise() {
        let obs = ObstaclePolygon::new(vec![0.0, 1.0, 0.5], vec![0.0, 0.0, 1.0]);
        // Closed: first == last.
        assert_eq!(obs.x_list.first(), obs.x_list.last());
        assert_eq!(obs.y_list.first(), obs.y_list.last());
        // Clockwise check.
        assert!(ObstaclePolygon::is_clockwise(&obs.x_list, &obs.y_list));
    }

    #[test]
    fn test_polygon_vertex_count() {
        let obs = ObstaclePolygon::new(vec![0.0, 4.0, 4.0, 0.0], vec![0.0, 0.0, 4.0, 4.0]);
        assert_eq!(obs.vertex_count(), 4);
    }

    // -- configuration-space expansion --------------------------------------

    #[test]
    fn test_configuration_space_vertices_count() {
        let planner = VisibilityRoadMap::new(5.0);
        let obs = ObstaclePolygon::new(vec![20.0, 30.0, 15.0], vec![20.0, 20.0, 30.0]);
        let (cvx, cvy) = planner.vertices_in_configuration_space(&obs);
        // One expanded vertex per original vertex.
        assert_eq!(cvx.len(), obs.vertex_count());
        assert_eq!(cvy.len(), obs.vertex_count());
    }

    #[test]
    fn test_expanded_vertices_are_further_from_centroid() {
        let planner = VisibilityRoadMap::new(5.0);
        let obs = ObstaclePolygon::new(vec![0.0, 10.0, 5.0], vec![0.0, 0.0, 8.66]);
        let (cvx, cvy) = planner.vertices_in_configuration_space(&obs);

        let cx: f64 =
            obs.x_list[..obs.vertex_count()].iter().sum::<f64>() / obs.vertex_count() as f64;
        let cy: f64 =
            obs.y_list[..obs.vertex_count()].iter().sum::<f64>() / obs.vertex_count() as f64;

        for i in 0..obs.vertex_count() {
            let orig_dist = ((obs.x_list[i] - cx).powi(2) + (obs.y_list[i] - cy).powi(2)).sqrt();
            let exp_dist = ((cvx[i] - cx).powi(2) + (cvy[i] - cy).powi(2)).sqrt();
            assert!(
                exp_dist > orig_dist,
                "expanded vertex {i} should be further from centroid"
            );
        }
    }

    // -- edge validity ------------------------------------------------------

    #[test]
    fn test_edge_valid_no_obstacle_crossing() {
        let a = Node::new(0.0, 0.0);
        let b = Node::new(10.0, 0.0);
        let obs = ObstaclePolygon::new(vec![0.0, 5.0, 5.0, 0.0], vec![5.0, 5.0, 10.0, 10.0]);
        assert!(VisibilityRoadMap::is_edge_valid(&a, &b, &obs));
    }

    #[test]
    fn test_edge_invalid_crosses_obstacle() {
        let a = Node::new(0.0, 0.0);
        let b = Node::new(10.0, 10.0);
        let obs = ObstaclePolygon::new(vec![3.0, 7.0, 5.0], vec![0.0, 0.0, 10.0]);
        assert!(!VisibilityRoadMap::is_edge_valid(&a, &b, &obs));
    }

    // -- full planning ------------------------------------------------------

    #[test]
    fn test_plan_finds_path() {
        let planner = VisibilityRoadMap::new(5.0);
        let obstacles = make_obstacles();
        let result = planner.plan(10.0, 10.0, 50.0, 50.0, &obstacles);
        assert!(result.is_some(), "planner should find a path");

        let (rx, ry) = result.unwrap();
        assert!(rx.len() >= 2);
        assert_eq!(rx.len(), ry.len());

        // Path starts at start.
        assert!((rx[0] - 10.0).abs() < 1e-9);
        assert!((ry[0] - 10.0).abs() < 1e-9);

        // Path ends at goal.
        assert!((*rx.last().unwrap() - 50.0).abs() < 1e-9);
        assert!((*ry.last().unwrap() - 50.0).abs() < 1e-9);
    }

    #[test]
    fn test_plan_with_no_obstacles() {
        let planner = VisibilityRoadMap::new(5.0);
        let result = planner.plan(0.0, 0.0, 10.0, 10.0, &[]);
        assert!(result.is_some());

        let (rx, ry) = result.unwrap();
        // Direct line: start -> goal.
        assert_eq!(rx.len(), 2);
        assert!((rx[0] - 0.0).abs() < 1e-9);
        assert!((*rx.last().unwrap() - 10.0).abs() < 1e-9);
        assert!((ry[0] - 0.0).abs() < 1e-9);
        assert!((*ry.last().unwrap() - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_get_edges_nonempty() {
        let planner = VisibilityRoadMap::new(5.0);
        let obstacles = make_obstacles();
        let edges = planner.get_edges(10.0, 10.0, 50.0, 50.0, &obstacles);
        assert!(!edges.is_empty(), "road-map should have visibility edges");
    }

    #[test]
    fn test_plan_path_cost_reasonable() {
        let planner = VisibilityRoadMap::new(5.0);
        let obstacles = make_obstacles();
        let (rx, ry) = planner.plan(10.0, 10.0, 50.0, 50.0, &obstacles).unwrap();

        // Compute path length.
        let mut length = 0.0;
        for i in 1..rx.len() {
            length += ((rx[i] - rx[i - 1]).powi(2) + (ry[i] - ry[i - 1]).powi(2)).sqrt();
        }

        let straight = ((50.0 - 10.0_f64).powi(2) + (50.0 - 10.0_f64).powi(2)).sqrt();
        // Path should be longer than straight-line (obstacles in the way)
        // but not absurdly long.
        assert!(
            length >= straight,
            "path cannot be shorter than straight line"
        );
        assert!(
            length < straight * 3.0,
            "path length {length:.1} seems excessively long (straight={straight:.1})"
        );
    }
}
