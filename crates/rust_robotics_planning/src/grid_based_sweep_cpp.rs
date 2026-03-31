//! Grid-based sweep coverage path planner (boustrophedon / lawn-mower pattern).
//!
//! Ported from PythonRobotics `GridBasedSweepCPP`.
//!
//! The algorithm:
//! 1. Find the longest edge of the input polygon to determine the sweep direction.
//! 2. Rotate the polygon so that the longest edge is axis-aligned.
//! 3. Build a grid map from the rotated polygon boundary, marking cells outside
//!    the polygon as occupied.
//! 4. Expand obstacles by one cell to avoid boundary collisions.
//! 5. Perform a boustrophedon (back-and-forth) sweep search on the grid.
//! 6. Convert the resulting grid path back to global coordinates.

// ---------------------------------------------------------------------------
// Grid map (self-contained, float-valued)
// ---------------------------------------------------------------------------

/// A 2D grid map with floating-point cell values.
///
/// Coordinate system: the grid is centred at (`center_x`, `center_y`) in world
/// coordinates. Cell (0, 0) sits at the lower-left corner.
struct FloatGridMap {
    width: usize,
    height: usize,
    resolution: f64,
    _center_x: f64,
    _center_y: f64,
    left_lower_x: f64,
    left_lower_y: f64,
    data: Vec<f64>,
}

impl FloatGridMap {
    fn new(width: usize, height: usize, resolution: f64, center_x: f64, center_y: f64) -> Self {
        let left_lower_x = center_x - (width as f64) / 2.0 * resolution;
        let left_lower_y = center_y - (height as f64) / 2.0 * resolution;
        let n = width * height;
        Self {
            width,
            height,
            resolution,
            _center_x: center_x,
            _center_y: center_y,
            left_lower_x,
            left_lower_y,
            data: vec![0.0; n],
        }
    }

    fn grid_index(&self, xi: i32, yi: i32) -> Option<usize> {
        if xi < 0 || yi < 0 || (xi as usize) >= self.width || (yi as usize) >= self.height {
            return None;
        }
        Some(yi as usize * self.width + xi as usize)
    }

    fn get_value(&self, xi: i32, yi: i32) -> Option<f64> {
        self.grid_index(xi, yi).map(|i| self.data[i])
    }

    fn set_value(&mut self, xi: i32, yi: i32, val: f64) -> bool {
        if let Some(i) = self.grid_index(xi, yi) {
            self.data[i] = val;
            true
        } else {
            false
        }
    }

    /// Return the world position of the centre of cell (`xi`, `yi`).
    fn cell_center(&self, xi: i32, yi: i32) -> (f64, f64) {
        let x = self.left_lower_x + (xi as f64) * self.resolution + self.resolution / 2.0;
        let y = self.left_lower_y + (yi as f64) * self.resolution + self.resolution / 2.0;
        (x, y)
    }

    /// True when the cell is occupied (value >= `threshold`) or out of bounds.
    fn is_occupied(&self, xi: i32, yi: i32, threshold: f64) -> bool {
        match self.get_value(xi, yi) {
            Some(v) => v >= threshold,
            None => true,
        }
    }

    // -- polygon helpers ----------------------------------------------------

    /// Mark cells that are *outside* (or *inside*) the given polygon with `val`.
    fn set_value_from_polygon(&mut self, pol_x: &[f64], pol_y: &[f64], val: f64, inside: bool) {
        for yi in 0..self.height as i32 {
            for xi in 0..self.width as i32 {
                let (px, py) = self.cell_center(xi, yi);
                let is_inside = point_in_polygon(px, py, pol_x, pol_y);
                if is_inside == inside {
                    self.set_value(xi, yi, val);
                }
            }
        }
    }

    /// Expand occupied cells by one grid step in 8-connected neighbourhood.
    fn expand_grid(&mut self) {
        let mut marks: Vec<(i32, i32, f64)> = Vec::new();
        for yi in 0..self.height as i32 {
            for xi in 0..self.width as i32 {
                if self.is_occupied(xi, yi, 1.0) {
                    if let Some(v) = self.get_value(xi, yi) {
                        for &(dx, dy) in &[
                            (1, 0),
                            (0, 1),
                            (1, 1),
                            (-1, 0),
                            (0, -1),
                            (-1, -1),
                            (1, -1),
                            (-1, 1),
                        ] {
                            marks.push((xi + dx, yi + dy, v));
                        }
                    }
                }
            }
        }
        for (x, y, v) in marks {
            self.set_value(x, y, v);
        }
    }
}

/// Ray-casting point-in-polygon test.
fn point_in_polygon(px: f64, py: f64, poly_x: &[f64], poly_y: &[f64]) -> bool {
    let n = poly_x.len() - 1; // last vertex == first vertex (closed ring)
    let mut inside = false;
    for i1 in 0..n {
        let i2 = (i1 + 1) % (n + 1);
        let (min_x, max_x) = if poly_x[i1] >= poly_x[i2] {
            (poly_x[i2], poly_x[i1])
        } else {
            (poly_x[i1], poly_x[i2])
        };
        if !(min_x <= px && px < max_x) {
            continue;
        }
        let slope = (poly_y[i2] - poly_y[i1]) / (poly_x[i2] - poly_x[i1]);
        if (poly_y[i1] + slope * (px - poly_x[i1]) - py) > 0.0 {
            inside = !inside;
        }
    }
    inside
}

// ---------------------------------------------------------------------------
// 2D rotation helper
// ---------------------------------------------------------------------------

/// Rotate points by angle `theta` (radians). Returns (rx, ry).
fn rotate_points(x: &[f64], y: &[f64], theta: f64) -> (Vec<f64>, Vec<f64>) {
    let c = theta.cos();
    let s = theta.sin();
    let rx: Vec<f64> = x
        .iter()
        .zip(y.iter())
        .map(|(&xi, &yi)| c * xi + s * yi)
        .collect();
    let ry: Vec<f64> = x
        .iter()
        .zip(y.iter())
        .map(|(&xi, &yi)| -s * xi + c * yi)
        .collect();
    (rx, ry)
}

// ---------------------------------------------------------------------------
// Sweep searcher
// ---------------------------------------------------------------------------

/// Sweep (vertical) direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SweepDirection {
    Up = 1,
    Down = -1,
}

/// Horizontal moving direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MovingDirection {
    Right = 1,
    Left = -1,
}

struct SweepSearcher {
    moving_dir: i32,
    sweep_dir: i32,
    turning_window: Vec<(i32, i32)>,
    x_indexes_goal_y: Vec<i32>,
    goal_y: i32,
}

impl SweepSearcher {
    fn new(
        moving: MovingDirection,
        sweep: SweepDirection,
        x_inds_goal_y: Vec<i32>,
        goal_y: i32,
    ) -> Self {
        let moving_dir = moving as i32;
        let sweep_dir = sweep as i32;
        let turning_window = Self::build_turning_window(moving_dir, sweep_dir);
        Self {
            moving_dir,
            sweep_dir,
            turning_window,
            x_indexes_goal_y: x_inds_goal_y,
            goal_y,
        }
    }

    fn build_turning_window(md: i32, sd: i32) -> Vec<(i32, i32)> {
        vec![(md, 0), (md, sd), (0, sd), (-md, sd)]
    }

    fn swap_moving_direction(&mut self) {
        self.moving_dir = -self.moving_dir;
        self.turning_window = Self::build_turning_window(self.moving_dir, self.sweep_dir);
    }

    /// Advance one step. Returns `Some((nx, ny))` or `None` if stuck.
    fn move_target_grid(&mut self, cx: i32, cy: i32, grid: &FloatGridMap) -> Option<(i32, i32)> {
        let nx = cx + self.moving_dir;
        let ny = cy;

        // Try to move in the current moving direction.
        if !grid.is_occupied(nx, ny, 0.5) {
            return Some((nx, ny));
        }

        // Current direction blocked -- try a turning manoeuvre.
        if let Some((mut tnx, tny)) = self.find_safe_turning_grid(cx, cy, grid) {
            // Keep moving in the current direction until blocked.
            while !grid.is_occupied(tnx + self.moving_dir, tny, 0.5) {
                tnx += self.moving_dir;
            }
            self.swap_moving_direction();
            Some((tnx, tny))
        } else {
            // Try moving backward one step.
            let bx = cx - self.moving_dir;
            let by = cy;
            if grid.is_occupied(bx, by, 1.0) {
                None // completely stuck
            } else {
                Some((bx, by))
            }
        }
    }

    fn find_safe_turning_grid(&self, cx: i32, cy: i32, grid: &FloatGridMap) -> Option<(i32, i32)> {
        for &(dx, dy) in &self.turning_window {
            let nx = cx + dx;
            let ny = cy + dy;
            if !grid.is_occupied(nx, ny, 0.5) {
                return Some((nx, ny));
            }
        }
        None
    }

    fn is_search_done(&self, grid: &FloatGridMap) -> bool {
        for &ix in &self.x_indexes_goal_y {
            if !grid.is_occupied(ix, self.goal_y, 0.5) {
                return false;
            }
        }
        true
    }

    fn search_start_grid(&self, grid: &FloatGridMap) -> Option<(i32, i32)> {
        let (x_inds, y_ind) = match self.sweep_dir.cmp(&0) {
            std::cmp::Ordering::Less => search_free_grid_index_at_edge_y(grid, true),
            _ => search_free_grid_index_at_edge_y(grid, false),
        };
        let y_ind = y_ind?;
        if x_inds.is_empty() {
            return None;
        }
        let x_ind = if self.moving_dir > 0 {
            *x_inds.iter().min().unwrap()
        } else {
            *x_inds.iter().max().unwrap()
        };
        Some((x_ind, y_ind))
    }
}

/// Search for free cells along the topmost (or bottommost) row that contains
/// any free cells.
fn search_free_grid_index_at_edge_y(
    grid: &FloatGridMap,
    from_upper: bool,
) -> (Vec<i32>, Option<i32>) {
    let y_range: Box<dyn Iterator<Item = i32>> = if from_upper {
        Box::new((0..grid.height as i32).rev())
    } else {
        Box::new(0..grid.height as i32)
    };

    for iy in y_range {
        let mut x_inds = Vec::new();
        let x_iter: Box<dyn Iterator<Item = i32>> = if from_upper {
            Box::new((0..grid.width as i32).rev())
        } else {
            Box::new(0..grid.width as i32)
        };
        for ix in x_iter {
            if !grid.is_occupied(ix, iy, 0.5) {
                x_inds.push(ix);
            }
        }
        if !x_inds.is_empty() {
            return (x_inds, Some(iy));
        }
    }
    (Vec::new(), None)
}

// ---------------------------------------------------------------------------
// Grid map construction
// ---------------------------------------------------------------------------

fn setup_grid_map(
    ox: &[f64],
    oy: &[f64],
    resolution: f64,
    sweep_dir: SweepDirection,
) -> (FloatGridMap, Vec<i32>, i32) {
    let offset_grid: usize = 10;
    let (min_x, max_x) = min_max(ox);
    let (min_y, max_y) = min_max(oy);
    let width = ((max_x - min_x) / resolution).ceil() as usize + offset_grid;
    let height = ((max_y - min_y) / resolution).ceil() as usize + offset_grid;
    let center_x = (max_x + min_x) / 2.0;
    let center_y = (max_y + min_y) / 2.0;

    let mut grid = FloatGridMap::new(width, height, resolution, center_x, center_y);
    grid.set_value_from_polygon(ox, oy, 1.0, false); // mark outside as occupied
    grid.expand_grid();

    let (x_inds_goal_y, goal_y) = match sweep_dir {
        SweepDirection::Up => search_free_grid_index_at_edge_y(&grid, true),
        SweepDirection::Down => search_free_grid_index_at_edge_y(&grid, false),
    };
    let goal_y = goal_y.unwrap_or(0);

    (grid, x_inds_goal_y, goal_y)
}

fn min_max(v: &[f64]) -> (f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for &x in v {
        if x < lo {
            lo = x;
        }
        if x > hi {
            hi = x;
        }
    }
    (lo, hi)
}

// ---------------------------------------------------------------------------
// Sweep path search
// ---------------------------------------------------------------------------

fn sweep_path_search(
    searcher: &mut SweepSearcher,
    grid: &mut FloatGridMap,
) -> (Vec<f64>, Vec<f64>) {
    let (mut cx, mut cy) = match searcher.search_start_grid(grid) {
        Some(p) => p,
        None => return (Vec::new(), Vec::new()),
    };

    if !grid.set_value(cx, cy, 0.5) {
        return (Vec::new(), Vec::new());
    }

    let (x, y) = grid.cell_center(cx, cy);
    let mut px = vec![x];
    let mut py = vec![y];

    loop {
        match searcher.move_target_grid(cx, cy, grid) {
            Some((nx, ny)) => {
                cx = nx;
                cy = ny;
            }
            None => break,
        }

        if searcher.is_search_done(grid) {
            break;
        }

        let (x, y) = grid.cell_center(cx, cy);
        px.push(x);
        py.push(y);

        grid.set_value(cx, cy, 0.5);
    }

    (px, py)
}

// ---------------------------------------------------------------------------
// Coordinate conversion
// ---------------------------------------------------------------------------

/// Find the longest polygon edge and return its direction vector and start
/// vertex.
fn find_sweep_direction_and_start(ox: &[f64], oy: &[f64]) -> ([f64; 2], [f64; 2]) {
    let mut max_dist = 0.0_f64;
    let mut vec = [0.0, 0.0];
    let mut start = [0.0, 0.0];
    for i in 0..ox.len() - 1 {
        let dx = ox[i + 1] - ox[i];
        let dy = oy[i + 1] - oy[i];
        let d = (dx * dx + dy * dy).sqrt();
        if d > max_dist {
            max_dist = d;
            vec = [dx, dy];
            start = [ox[i], oy[i]];
        }
    }
    (vec, start)
}

fn convert_grid_coordinate(
    ox: &[f64],
    oy: &[f64],
    sweep_vec: [f64; 2],
    sweep_start: [f64; 2],
) -> (Vec<f64>, Vec<f64>) {
    let tx: Vec<f64> = ox.iter().map(|x| x - sweep_start[0]).collect();
    let ty: Vec<f64> = oy.iter().map(|y| y - sweep_start[1]).collect();
    let theta = sweep_vec[1].atan2(sweep_vec[0]);
    rotate_points(&tx, &ty, theta)
}

fn convert_global_coordinate(
    px: &[f64],
    py: &[f64],
    sweep_vec: [f64; 2],
    sweep_start: [f64; 2],
) -> (Vec<f64>, Vec<f64>) {
    let theta = sweep_vec[1].atan2(sweep_vec[0]);
    let (rx, ry) = rotate_points(px, py, -theta);
    let gx: Vec<f64> = rx.iter().map(|x| x + sweep_start[0]).collect();
    let gy: Vec<f64> = ry.iter().map(|y| y + sweep_start[1]).collect();
    (gx, gy)
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Configuration for the grid-based sweep coverage path planner.
#[derive(Debug, Clone)]
pub struct GridBasedSweepConfig {
    /// Grid resolution \[m\].
    pub resolution: f64,
    /// Horizontal moving direction at the start of the sweep.
    pub moving_direction: MovingDirection,
    /// Vertical sweep direction.
    pub sweep_direction: SweepDirection,
}

impl Default for GridBasedSweepConfig {
    fn default() -> Self {
        Self {
            resolution: 5.0,
            moving_direction: MovingDirection::Right,
            sweep_direction: SweepDirection::Up,
        }
    }
}

/// Result of the grid-based sweep coverage path planner.
#[derive(Debug, Clone)]
pub struct SweepCoveragePath {
    /// X coordinates of the planned path in the global frame.
    pub x: Vec<f64>,
    /// Y coordinates of the planned path in the global frame.
    pub y: Vec<f64>,
}

/// Plan a boustrophedon (lawn-mower) coverage path over the area enclosed by
/// the given polygon.
///
/// The polygon is specified as a sequence of vertices (`ox`, `oy`) forming a
/// closed ring (first vertex == last vertex).
///
/// # Arguments
/// * `ox` - x-coordinates of the polygon boundary.
/// * `oy` - y-coordinates of the polygon boundary.
/// * `config` - planner configuration.
///
/// # Returns
/// A [`SweepCoveragePath`] containing the planned waypoints in global
/// coordinates.
///
/// # Example
/// ```
/// use rust_robotics_planning::grid_based_sweep_cpp::{
///     plan_sweep_coverage, GridBasedSweepConfig,
/// };
///
/// let ox = vec![0.0, 50.0, 50.0, 0.0, 0.0];
/// let oy = vec![0.0, 0.0, 30.0, 30.0, 0.0];
/// let config = GridBasedSweepConfig {
///     resolution: 5.0,
///     ..Default::default()
/// };
/// let path = plan_sweep_coverage(&ox, &oy, &config);
/// assert!(!path.x.is_empty());
/// ```
pub fn plan_sweep_coverage(
    ox: &[f64],
    oy: &[f64],
    config: &GridBasedSweepConfig,
) -> SweepCoveragePath {
    assert!(
        ox.len() >= 4 && ox.len() == oy.len(),
        "polygon must have at least 3 distinct vertices (4 points with closing vertex)"
    );

    let (sweep_vec, sweep_start) = find_sweep_direction_and_start(ox, oy);
    let (rox, roy) = convert_grid_coordinate(ox, oy, sweep_vec, sweep_start);

    let (mut grid, x_inds_goal_y, goal_y) =
        setup_grid_map(&rox, &roy, config.resolution, config.sweep_direction);

    let mut searcher = SweepSearcher::new(
        config.moving_direction,
        config.sweep_direction,
        x_inds_goal_y,
        goal_y,
    );

    let (px, py) = sweep_path_search(&mut searcher, &mut grid);
    let (gx, gy) = convert_global_coordinate(&px, &py, sweep_vec, sweep_start);

    SweepCoveragePath { x: gx, y: gy }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: Euclidean distance between two points.
    fn dist(x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
        ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt()
    }

    // -- unit tests for internal helpers ------------------------------------

    #[test]
    fn test_point_in_polygon_square() {
        // Square from (0,0) to (10,10), closed.
        let px = vec![0.0, 10.0, 10.0, 0.0, 0.0];
        let py = vec![0.0, 0.0, 10.0, 10.0, 0.0];
        assert!(point_in_polygon(5.0, 5.0, &px, &py));
        assert!(!point_in_polygon(15.0, 5.0, &px, &py));
        assert!(!point_in_polygon(-1.0, 5.0, &px, &py));
    }

    #[test]
    fn test_rotate_points_90_degrees() {
        let x = vec![1.0, 0.0];
        let y = vec![0.0, 1.0];
        let (rx, ry) = rotate_points(&x, &y, PI / 2.0);
        // After rotating by 90 degrees: (1,0) -> (0, -1), (0,1) -> (1, 0)
        assert!((rx[0] - 0.0).abs() < 1e-10);
        assert!((ry[0] - (-1.0)).abs() < 1e-10);
        assert!((rx[1] - 1.0).abs() < 1e-10);
        assert!((ry[1] - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_find_sweep_direction_longest_edge() {
        let ox = vec![0.0, 100.0, 100.0, 0.0, 0.0];
        let oy = vec![0.0, 0.0, 10.0, 10.0, 0.0];
        let (vec, start) = find_sweep_direction_and_start(&ox, &oy);
        // Longest edge is the bottom: (0,0) -> (100,0)
        assert!((vec[0] - 100.0).abs() < 1e-10);
        assert!((vec[1] - 0.0).abs() < 1e-10);
        assert!((start[0] - 0.0).abs() < 1e-10);
        assert!((start[1] - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_grid_map_set_get() {
        let mut gm = FloatGridMap::new(10, 10, 1.0, 5.0, 5.0);
        assert!(!gm.is_occupied(5, 5, 0.5));
        gm.set_value(5, 5, 1.0);
        assert!(gm.is_occupied(5, 5, 0.5));
    }

    #[test]
    fn test_grid_map_out_of_bounds() {
        let gm = FloatGridMap::new(5, 5, 1.0, 2.5, 2.5);
        assert!(gm.is_occupied(-1, 0, 0.5));
        assert!(gm.is_occupied(0, -1, 0.5));
        assert!(gm.is_occupied(5, 0, 0.5));
        assert!(gm.is_occupied(0, 5, 0.5));
    }

    // -- integration tests --------------------------------------------------

    #[test]
    fn test_rectangle_coverage() {
        let ox = vec![0.0, 50.0, 50.0, 0.0, 0.0];
        let oy = vec![0.0, 0.0, 30.0, 30.0, 0.0];
        let config = GridBasedSweepConfig {
            resolution: 5.0,
            ..Default::default()
        };
        let path = plan_sweep_coverage(&ox, &oy, &config);

        assert!(
            !path.x.is_empty(),
            "coverage path should not be empty for a rectangle"
        );
        // Path should have many waypoints.
        assert!(
            path.x.len() > 5,
            "expected more than 5 waypoints, got {}",
            path.x.len()
        );
    }

    #[test]
    fn test_polygon_coverage() {
        let ox = vec![0.0, 20.0, 50.0, 100.0, 130.0, 40.0, 0.0];
        let oy = vec![0.0, -20.0, 0.0, 30.0, 60.0, 80.0, 0.0];
        let config = GridBasedSweepConfig {
            resolution: 5.0,
            ..Default::default()
        };
        let path = plan_sweep_coverage(&ox, &oy, &config);

        assert!(!path.x.is_empty());
        assert!(path.x.len() > 10);
    }

    #[test]
    fn test_path_lengths_match() {
        let ox = vec![0.0, 50.0, 50.0, 0.0, 0.0];
        let oy = vec![0.0, 0.0, 30.0, 30.0, 0.0];
        let config = GridBasedSweepConfig::default();
        let path = plan_sweep_coverage(&ox, &oy, &config);

        assert_eq!(
            path.x.len(),
            path.y.len(),
            "x and y paths must have the same length"
        );
    }

    #[test]
    fn test_waypoints_inside_polygon_neighbourhood() {
        // All waypoints should be reasonably close to the polygon interior.
        let ox = vec![0.0, 50.0, 50.0, 0.0, 0.0];
        let oy = vec![0.0, 0.0, 30.0, 30.0, 0.0];
        let config = GridBasedSweepConfig {
            resolution: 1.3,
            ..Default::default()
        };
        let path = plan_sweep_coverage(&ox, &oy, &config);

        let margin = config.resolution * 3.0;
        for (&x, &y) in path.x.iter().zip(path.y.iter()) {
            assert!(
                x > -margin && x < 50.0 + margin && y > -margin && y < 30.0 + margin,
                "waypoint ({}, {}) is too far from the polygon",
                x,
                y
            );
        }
    }

    #[test]
    fn test_consecutive_waypoints_are_close() {
        let ox = vec![0.0, 50.0, 50.0, 0.0, 0.0];
        let oy = vec![0.0, 0.0, 30.0, 30.0, 0.0];
        let config = GridBasedSweepConfig {
            resolution: 2.0,
            ..Default::default()
        };
        let path = plan_sweep_coverage(&ox, &oy, &config);

        let max_step = config.resolution * 3.0; // allow some slack for turns
        for i in 1..path.x.len() {
            let d = dist(path.x[i - 1], path.y[i - 1], path.x[i], path.y[i]);
            assert!(
                d < max_step,
                "step {} -> {} is too large: {:.2} (max {:.2})",
                i - 1,
                i,
                d,
                max_step,
            );
        }
    }

    #[test]
    fn test_different_sweep_directions() {
        let ox = vec![0.0, 50.0, 50.0, 0.0, 0.0];
        let oy = vec![0.0, 0.0, 30.0, 30.0, 0.0];

        let cfg_up_right = GridBasedSweepConfig {
            resolution: 5.0,
            moving_direction: MovingDirection::Right,
            sweep_direction: SweepDirection::Up,
        };
        let cfg_down_left = GridBasedSweepConfig {
            resolution: 5.0,
            moving_direction: MovingDirection::Left,
            sweep_direction: SweepDirection::Down,
        };

        let path1 = plan_sweep_coverage(&ox, &oy, &cfg_up_right);
        let path2 = plan_sweep_coverage(&ox, &oy, &cfg_down_left);

        assert!(!path1.x.is_empty());
        assert!(!path2.x.is_empty());
        // Paths should differ in at least the first waypoint.
        let same_start =
            (path1.x[0] - path2.x[0]).abs() < 1e-6 && (path1.y[0] - path2.y[0]).abs() < 1e-6;
        assert!(
            !same_start,
            "different sweep configs should produce different starting points"
        );
    }

    #[test]
    fn test_fine_resolution_rectangle() {
        let ox = vec![0.0, 50.0, 50.0, 0.0, 0.0];
        let oy = vec![0.0, 0.0, 30.0, 30.0, 0.0];
        let config = GridBasedSweepConfig {
            resolution: 1.3,
            ..Default::default()
        };
        let path = plan_sweep_coverage(&ox, &oy, &config);
        // Finer resolution should produce more waypoints.
        assert!(
            path.x.len() > 50,
            "fine resolution should yield many waypoints, got {}",
            path.x.len()
        );
    }

    #[test]
    #[should_panic(expected = "polygon must have at least 3 distinct vertices")]
    fn test_too_few_vertices_panics() {
        let ox = vec![0.0, 1.0];
        let oy = vec![0.0, 1.0];
        plan_sweep_coverage(&ox, &oy, &GridBasedSweepConfig::default());
    }

    #[test]
    fn test_large_irregular_polygon() {
        let ox = vec![0.0, 20.0, 50.0, 200.0, 130.0, 40.0, 0.0];
        let oy = vec![0.0, -80.0, 0.0, 30.0, 60.0, 80.0, 0.0];
        let config = GridBasedSweepConfig {
            resolution: 5.0,
            ..Default::default()
        };
        let path = plan_sweep_coverage(&ox, &oy, &config);
        assert!(!path.x.is_empty());
        assert!(path.x.len() > 20);
    }
}
