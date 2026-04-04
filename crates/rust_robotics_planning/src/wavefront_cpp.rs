//! Wavefront Coverage Path Planner
//!
//! Uses BFS wavefront expansion from a goal cell to build a distance/path
//! transform matrix, then greedily follows highest-valued unvisited neighbours
//! to produce a coverage path that visits every reachable free cell.
//!
//! Reference: "Planning paths of complete coverage of an unstructured
//! environment by a mobile robot" — Zelinsky et al.

use std::collections::{HashSet, VecDeque};

/// Distance metric used during wavefront expansion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DistanceType {
    /// All 8-connected moves cost 1.
    Chessboard,
    /// Cardinal moves cost 1, diagonal moves cost sqrt(2).
    Euclidean,
}

/// Transform type used to build the wavefront matrix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TransformType {
    /// Pure distance transform (no obstacle weighting).
    Distance,
    /// Path transform — adds an obstacle-proximity penalty weighted by `alpha`.
    Path,
}

/// Configuration for the wavefront coverage path planner.
#[derive(Debug, Clone)]
pub struct WavefrontCppConfig {
    /// Distance metric for BFS expansion.
    pub distance_type: DistanceType,
    /// Transform type (distance or path).
    pub transform_type: TransformType,
    /// Weight of obstacle proximity penalty (only used with `TransformType::Path`).
    pub alpha: f64,
}

impl Default for WavefrontCppConfig {
    fn default() -> Self {
        Self {
            distance_type: DistanceType::Chessboard,
            transform_type: TransformType::Distance,
            alpha: 0.01,
        }
    }
}

/// 2D grid for wavefront planning. `true` = obstacle, `false` = free.
pub struct WavefrontGrid {
    /// Number of rows.
    pub rows: usize,
    /// Number of columns.
    pub cols: usize,
    /// Row-major obstacle data: `cells[row * cols + col]`.
    cells: Vec<bool>,
}

impl WavefrontGrid {
    /// Creates a new grid filled with free cells.
    pub fn new(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            cells: vec![false; rows * cols],
        }
    }

    /// Creates a grid from a row-major boolean vector.
    ///
    /// `true` = obstacle, `false` = free.
    pub fn from_vec(rows: usize, cols: usize, cells: Vec<bool>) -> Self {
        assert_eq!(cells.len(), rows * cols);
        Self { rows, cols, cells }
    }

    /// Returns whether the cell is an obstacle.
    pub fn is_obstacle(&self, row: usize, col: usize) -> bool {
        self.cells[row * self.cols + col]
    }

    /// Sets a cell as obstacle or free.
    pub fn set_obstacle(&mut self, row: usize, col: usize, val: bool) {
        self.cells[row * self.cols + col] = val;
    }

    fn in_bounds(&self, r: i32, c: i32) -> bool {
        r >= 0 && (r as usize) < self.rows && c >= 0 && (c as usize) < self.cols
    }

    fn is_free_signed(&self, r: i32, c: i32) -> bool {
        self.in_bounds(r, c) && !self.is_obstacle(r as usize, c as usize)
    }
}

/// 8-connected neighbour increments: E, SE, S, SW, W, NW, N, NE.
const INC_ORDER: [(i32, i32); 8] = [
    (0, 1),
    (1, 1),
    (1, 0),
    (1, -1),
    (0, -1),
    (-1, -1),
    (-1, 0),
    (-1, 1),
];

/// Compute the obstacle distance transform using BFS (chessboard distance).
///
/// Returns a matrix where each free cell holds its minimum chessboard distance
/// to the nearest obstacle, and obstacle cells hold 0.
fn obstacle_distance_transform(grid: &WavefrontGrid) -> Vec<f64> {
    let n = grid.rows * grid.cols;
    let mut dist = vec![f64::INFINITY; n];
    let mut queue: VecDeque<(usize, usize)> = VecDeque::new();

    // Seed: all obstacle cells have distance 0.
    for r in 0..grid.rows {
        for c in 0..grid.cols {
            if grid.is_obstacle(r, c) {
                dist[r * grid.cols + c] = 0.0;
                queue.push_back((r, c));
            }
        }
    }

    while let Some((r, c)) = queue.pop_front() {
        let cur = dist[r * grid.cols + c];
        for &(dr, dc) in &INC_ORDER {
            let nr = r as i32 + dr;
            let nc = c as i32 + dc;
            if grid.in_bounds(nr, nc) {
                let nr = nr as usize;
                let nc = nc as usize;
                let nd = cur + 1.0;
                if nd < dist[nr * grid.cols + nc] {
                    dist[nr * grid.cols + nc] = nd;
                    queue.push_back((nr, nc));
                }
            }
        }
    }

    dist
}

/// Build the wavefront transform matrix via BFS from `src`.
///
/// Each free cell receives a cost indicating the wavefront expansion distance
/// from `src`. Obstacle cells remain at `f64::INFINITY`.
fn build_transform(
    grid: &WavefrontGrid,
    src: (usize, usize),
    config: &WavefrontCppConfig,
) -> Vec<f64> {
    let n = grid.rows * grid.cols;
    let mut mat = vec![f64::INFINITY; n];
    mat[src.0 * grid.cols + src.1] = 0.0;

    let costs: [f64; 8] = match config.distance_type {
        DistanceType::Chessboard => [1.0; 8],
        DistanceType::Euclidean => {
            let s = std::f64::consts::SQRT_2;
            [1.0, s, 1.0, s, 1.0, s, 1.0, s]
        }
    };

    let obstacle_dist = match config.transform_type {
        TransformType::Distance => vec![0.0; n],
        TransformType::Path => obstacle_distance_transform(grid),
    };

    let mut visited = vec![false; n];
    visited[src.0 * grid.cols + src.1] = true;
    let mut queue: VecDeque<(usize, usize)> = VecDeque::new();
    queue.push_back(src);
    let mut enqueued = HashSet::new();
    enqueued.insert(src);

    while let Some((r, c)) = queue.pop_front() {
        for (k, &(dr, dc)) in INC_ORDER.iter().enumerate() {
            let nr = r as i32 + dr;
            let nc = c as i32 + dc;
            if grid.is_free_signed(nr, nc) {
                let nr_u = nr as usize;
                let nc_u = nc as usize;
                let idx = nr_u * grid.cols + nc_u;
                let cur_idx = r * grid.cols + c;

                visited[cur_idx] = true;

                let new_cost = mat[idx] + costs[k] + config.alpha * obstacle_dist[idx];
                if new_cost < mat[cur_idx] {
                    mat[cur_idx] = new_cost;
                }

                if !visited[idx] && !enqueued.contains(&(nr_u, nc_u)) {
                    queue.push_back((nr_u, nc_u));
                    enqueued.insert((nr_u, nc_u));
                }
            }
        }
    }

    mat
}

/// Determine the 8-connected neighbour search order based on start→goal direction.
///
/// Prioritises directions that move away from the goal so that the greedy
/// traversal covers remote cells first before heading back.
fn search_order(start: (usize, usize), goal: (usize, usize)) -> [(i32, i32); 8] {
    let sr = start.0 as i32;
    let sc = start.1 as i32;
    let gr = goal.0 as i32;
    let gc = goal.1 as i32;

    if sr >= gr && sc >= gc {
        [
            (1, 0),
            (0, 1),
            (-1, 0),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ]
    } else if sr <= gr && sc >= gc {
        [
            (-1, 0),
            (0, 1),
            (1, 0),
            (0, -1),
            (-1, 1),
            (-1, -1),
            (1, 1),
            (1, -1),
        ]
    } else if sr >= gr && sc <= gc {
        [
            (1, 0),
            (0, -1),
            (-1, 0),
            (0, 1),
            (1, -1),
            (-1, -1),
            (1, 1),
            (-1, 1),
        ]
    } else {
        [
            (-1, 0),
            (0, -1),
            (0, 1),
            (1, 0),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]
    }
}

/// Run the wavefront coverage path planner.
///
/// Returns a path (list of `(row, col)` cells) that attempts to visit every
/// reachable free cell, starting from `start` and ending at `goal`.
///
/// The algorithm:
/// 1. Build a wavefront transform matrix via BFS from `goal`.
/// 2. From `start`, greedily move to the unvisited neighbour with the highest
///    transform value (farthest from goal).
/// 3. When stuck, backtrace along the existing path to find a cell with an
///    unvisited neighbour and resume from there.
pub fn wavefront_cpp(
    grid: &WavefrontGrid,
    start: (usize, usize),
    goal: (usize, usize),
    config: &WavefrontCppConfig,
) -> Vec<(usize, usize)> {
    let transform = build_transform(grid, goal, config);
    let order = search_order(start, goal);

    let mut path: Vec<(usize, usize)> = Vec::new();
    let mut visited = vec![false; grid.rows * grid.cols];
    let mut current = start;

    loop {
        if current == goal {
            path.push(current);
            break;
        }

        let (r, c) = current;
        path.push((r, c));
        visited[r * grid.cols + c] = true;

        // Search back through the path for a cell with an unvisited neighbour.
        let mut best = None;
        let mut best_val = f64::NEG_INFINITY;

        for &(pr, pc) in path.iter().rev() {
            for &(dr, dc) in &order {
                let nr = pr as i32 + dr;
                let nc = pc as i32 + dc;
                if grid.is_free_signed(nr, nc) {
                    let nr_u = nr as usize;
                    let nc_u = nc as usize;
                    let idx = nr_u * grid.cols + nc_u;
                    if !visited[idx] && transform[idx] != f64::INFINITY && transform[idx] > best_val
                    {
                        best_val = transform[idx];
                        best = Some((nr_u, nc_u));
                    }
                }
            }
            // If we found a candidate from the current backtrack cell, use it.
            if best.is_some() {
                break;
            }
        }

        match best {
            Some(next) => current = next,
            None => {
                // No reachable unvisited cell found — coverage complete.
                break;
            }
        }
    }

    path
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: create a small open grid with no obstacles.
    fn open_grid(rows: usize, cols: usize) -> WavefrontGrid {
        WavefrontGrid::new(rows, cols)
    }

    #[test]
    fn test_simple_open_grid_visits_all_cells() {
        let grid = open_grid(5, 5);
        let config = WavefrontCppConfig::default();
        let path = wavefront_cpp(&grid, (4, 0), (0, 0), &config);

        // Path should start at start and end at goal.
        assert_eq!(*path.first().unwrap(), (4, 0));
        assert_eq!(*path.last().unwrap(), (0, 0));

        // All 25 free cells should be visited.
        let unique: HashSet<_> = path.iter().copied().collect();
        assert_eq!(unique.len(), 25);
    }

    #[test]
    fn test_start_equals_goal() {
        let grid = open_grid(3, 3);
        let config = WavefrontCppConfig::default();
        let path = wavefront_cpp(&grid, (0, 0), (0, 0), &config);

        assert_eq!(*path.first().unwrap(), (0, 0));
        assert_eq!(*path.last().unwrap(), (0, 0));
    }

    #[test]
    fn test_grid_with_obstacles() {
        // 5x5 grid with a wall in the middle row (row 2), leaving a gap at col 0.
        let mut grid = WavefrontGrid::new(5, 5);
        for c in 1..5 {
            grid.set_obstacle(2, c, true);
        }

        let config = WavefrontCppConfig::default();
        let path = wavefront_cpp(&grid, (4, 4), (0, 0), &config);

        // No path cell should be on an obstacle.
        for &(r, c) in &path {
            assert!(!grid.is_obstacle(r, c), "Path on obstacle at ({r}, {c})");
        }

        // The 4 obstacle cells are not visited; 21 free cells should be covered.
        let unique: HashSet<_> = path.iter().copied().collect();
        assert_eq!(unique.len(), 21);
    }

    #[test]
    fn test_euclidean_distance_type() {
        let grid = open_grid(4, 4);
        let config = WavefrontCppConfig {
            distance_type: DistanceType::Euclidean,
            transform_type: TransformType::Distance,
            alpha: 0.0,
        };
        let path = wavefront_cpp(&grid, (3, 0), (0, 0), &config);

        let unique: HashSet<_> = path.iter().copied().collect();
        assert_eq!(unique.len(), 16);
    }

    #[test]
    fn test_path_transform_type() {
        let mut grid = WavefrontGrid::new(6, 6);
        // Place a couple of obstacles.
        grid.set_obstacle(2, 2, true);
        grid.set_obstacle(2, 3, true);

        let config = WavefrontCppConfig {
            distance_type: DistanceType::Chessboard,
            transform_type: TransformType::Path,
            alpha: 0.01,
        };
        let path = wavefront_cpp(&grid, (5, 0), (0, 0), &config);

        for &(r, c) in &path {
            assert!(!grid.is_obstacle(r, c));
        }

        let unique: HashSet<_> = path.iter().copied().collect();
        // 36 total - 2 obstacles = 34 free cells.
        assert_eq!(unique.len(), 34);
    }

    #[test]
    fn test_from_vec_constructor() {
        #[rustfmt::skip]
        let cells = vec![
            false, false, false,
            false, true,  false,
            false, false, false,
        ];
        let grid = WavefrontGrid::from_vec(3, 3, cells);
        assert!(grid.is_obstacle(1, 1));
        assert!(!grid.is_obstacle(0, 0));
    }

    #[test]
    fn test_obstacle_distance_transform_basic() {
        let mut grid = WavefrontGrid::new(5, 5);
        grid.set_obstacle(2, 2, true);
        let dist = obstacle_distance_transform(&grid);

        // Obstacle cell itself should be 0.
        assert_eq!(dist[2 * 5 + 2], 0.0);
        // Immediate neighbours should be 1.
        assert_eq!(dist[5 + 2], 1.0);
        assert_eq!(dist[2 * 5 + 1], 1.0);
        // Corner cell (0,0) chessboard distance = 2.
        assert_eq!(dist[0], 2.0);
    }

    #[test]
    fn test_single_cell_grid() {
        let grid = open_grid(1, 1);
        let config = WavefrontCppConfig::default();
        let path = wavefront_cpp(&grid, (0, 0), (0, 0), &config);
        assert_eq!(path, vec![(0, 0)]);
    }

    #[test]
    fn test_no_path_cells_are_duplicated() {
        let grid = open_grid(4, 4);
        let config = WavefrontCppConfig::default();
        let path = wavefront_cpp(&grid, (3, 3), (0, 0), &config);

        let unique: HashSet<_> = path.iter().copied().collect();
        assert_eq!(
            unique.len(),
            path.len(),
            "Path should not contain duplicates"
        );
    }

    #[test]
    fn test_search_order_all_quadrants() {
        // Just verify they return 8 unique directions.
        let cases = [
            ((3, 3), (0, 0)),
            ((0, 0), (3, 3)),
            ((3, 0), (0, 3)),
            ((0, 3), (3, 0)),
        ];
        for (s, g) in cases {
            let order = search_order(s, g);
            let set: HashSet<_> = order.iter().copied().collect();
            assert_eq!(set.len(), 8);
        }
    }
}
