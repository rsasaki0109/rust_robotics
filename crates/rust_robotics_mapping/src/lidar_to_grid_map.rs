//! LIDAR to 2D Grid Map conversion.
//!
//! Converts lidar scan data (angle, distance pairs) into an occupancy grid map
//! using either Bresenham ray tracing or flood fill.
//!
//! Based on the work by Erno Horvath, Csaba Hajdu, and Atsushi Sakai (@Atsushi_twi).

use nalgebra::DMatrix;
use std::collections::VecDeque;
use std::fs::File;
use std::io::Write;

/// Grid map extension area \[m\].
pub const EXTEND_AREA: f64 = 1.0;

/// Occupancy value for free cells.
pub const FREE: f64 = 0.0;
/// Occupancy value for unknown cells.
pub const UNKNOWN: f64 = 0.5;
/// Occupancy value for occupied cells.
pub const OCCUPIED: f64 = 1.0;

/// Method used to generate the occupancy grid map.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GridMapMethod {
    /// Bresenham ray tracing from sensor origin to each obstacle point.
    Bresenham,
    /// Flood fill from sensor origin after initial ray-based boundary marking.
    FloodFill,
}

/// Grid map configuration computed from obstacle positions.
#[derive(Debug, Clone)]
struct GridMapConfig {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
    x_width: usize,
    y_width: usize,
}

/// LIDAR-based occupancy grid map.
///
/// Cell values:
/// - `0.0` = free space
/// - `0.5` = unknown
/// - `1.0` = occupied (obstacle)
pub struct LidarGridMap {
    pub data: DMatrix<f64>,
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
    pub resolution: f64,
    pub x_width: usize,
    pub y_width: usize,
    pub center_ix: usize,
    pub center_iy: usize,
}

impl LidarGridMap {
    /// Create a new LIDAR grid map from obstacle positions in Cartesian coordinates.
    ///
    /// # Arguments
    /// * `ox` - obstacle x coordinates \[m\]
    /// * `oy` - obstacle y coordinates \[m\]
    /// * `resolution` - grid cell size \[m\]
    /// * `method` - algorithm to use for free-space detection
    pub fn new(ox: &[f64], oy: &[f64], resolution: f64, method: GridMapMethod) -> Self {
        assert_eq!(ox.len(), oy.len(), "ox and oy must have the same length");
        assert!(resolution > 0.0, "resolution must be positive");

        let config = calc_grid_map_config(ox, oy, resolution);

        // Sensor is at the origin (0, 0) in world coordinates.
        let center_ix = ((-config.min_x) / resolution).round() as usize;
        let center_iy = ((-config.min_y) / resolution).round() as usize;

        let mut data = DMatrix::from_element(config.x_width, config.y_width, UNKNOWN);

        match method {
            GridMapMethod::Bresenham => {
                generate_bresenham(&mut data, ox, oy, &config, resolution, center_ix, center_iy);
            }
            GridMapMethod::FloodFill => {
                generate_flood_fill(&mut data, ox, oy, &config, resolution, center_ix, center_iy);
            }
        }

        LidarGridMap {
            data,
            min_x: config.min_x,
            min_y: config.min_y,
            max_x: config.max_x,
            max_y: config.max_y,
            resolution,
            x_width: config.x_width,
            y_width: config.y_width,
            center_ix,
            center_iy,
        }
    }

    /// Create a LIDAR grid map from angle and distance pairs.
    ///
    /// # Arguments
    /// * `angles` - lidar beam angles \[rad\]
    /// * `distances` - corresponding distances \[m\]
    /// * `resolution` - grid cell size \[m\]
    /// * `method` - algorithm to use
    pub fn from_scan(
        angles: &[f64],
        distances: &[f64],
        resolution: f64,
        method: GridMapMethod,
    ) -> Self {
        assert_eq!(
            angles.len(),
            distances.len(),
            "angles and distances must have the same length"
        );
        let ox: Vec<f64> = angles
            .iter()
            .zip(distances.iter())
            .map(|(a, d)| a.sin() * d)
            .collect();
        let oy: Vec<f64> = angles
            .iter()
            .zip(distances.iter())
            .map(|(a, d)| a.cos() * d)
            .collect();
        Self::new(&ox, &oy, resolution, method)
    }

    /// Get grid index from world coordinates.
    pub fn get_index(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let ix = ((x - self.min_x) / self.resolution).floor() as i32;
        let iy = ((y - self.min_y) / self.resolution).floor() as i32;

        if ix >= 0 && ix < self.x_width as i32 && iy >= 0 && iy < self.y_width as i32 {
            Some((ix as usize, iy as usize))
        } else {
            None
        }
    }

    /// Get occupancy value at world coordinates.
    pub fn get_occupancy(&self, x: f64, y: f64) -> Option<f64> {
        self.get_index(x, y).map(|(ix, iy)| self.data[(ix, iy)])
    }
}

/// Compute grid map extents from obstacle positions.
fn calc_grid_map_config(ox: &[f64], oy: &[f64], resolution: f64) -> GridMapConfig {
    let min_x = ox.iter().cloned().fold(f64::INFINITY, f64::min).floor() - EXTEND_AREA / 2.0;
    let min_y = oy.iter().cloned().fold(f64::INFINITY, f64::min).floor() - EXTEND_AREA / 2.0;
    let max_x = ox.iter().cloned().fold(f64::NEG_INFINITY, f64::max).ceil() + EXTEND_AREA / 2.0;
    let max_y = oy.iter().cloned().fold(f64::NEG_INFINITY, f64::max).ceil() + EXTEND_AREA / 2.0;

    let x_width = ((max_x - min_x) / resolution).round() as usize;
    let y_width = ((max_y - min_y) / resolution).round() as usize;

    GridMapConfig {
        min_x,
        min_y,
        max_x,
        max_y,
        x_width,
        y_width,
    }
}

/// Bresenham's line algorithm returning a list of (x, y) grid points.
fn bresenham(start: (i32, i32), end: (i32, i32)) -> Vec<(i32, i32)> {
    let (mut x1, mut y1) = start;
    let (mut x2, mut y2) = end;
    let dx = x2 - x1;
    let dy = y2 - y1;
    let is_steep = dy.abs() > dx.abs();

    if is_steep {
        std::mem::swap(&mut x1, &mut y1);
        std::mem::swap(&mut x2, &mut y2);
    }

    let mut swapped = false;
    if x1 > x2 {
        std::mem::swap(&mut x1, &mut x2);
        std::mem::swap(&mut y1, &mut y2);
        swapped = true;
    }

    let dx = x2 - x1;
    let dy = y2 - y1;
    let mut error = dx / 2;
    let y_step = if y1 < y2 { 1 } else { -1 };

    let mut y = y1;
    let mut points = Vec::new();
    for x in x1..=x2 {
        let coord = if is_steep { (y, x) } else { (x, y) };
        points.push(coord);
        error -= dy.abs();
        if error < 0 {
            y += y_step;
            error += dx;
        }
    }

    if swapped {
        points.reverse();
    }
    points
}

/// Generate occupancy grid using Bresenham ray tracing.
fn generate_bresenham(
    data: &mut DMatrix<f64>,
    ox: &[f64],
    oy: &[f64],
    config: &GridMapConfig,
    resolution: f64,
    center_ix: usize,
    center_iy: usize,
) {
    let xw = config.x_width;
    let yw = config.y_width;

    for (&x, &y) in ox.iter().zip(oy.iter()) {
        let ix = ((x - config.min_x) / resolution).round() as i32;
        let iy = ((y - config.min_y) / resolution).round() as i32;

        let laser_beams = bresenham((center_ix as i32, center_iy as i32), (ix, iy));

        // Mark cells along the ray as free.
        for &(bx, by) in &laser_beams {
            if bx >= 0 && bx < xw as i32 && by >= 0 && by < yw as i32 {
                data[(bx as usize, by as usize)] = FREE;
            }
        }

        // Mark the obstacle cell and its immediate neighbors as occupied.
        for di in 0..=1_i32 {
            for dj in 0..=1_i32 {
                let oi = ix + di;
                let oj = iy + dj;
                if oi >= 0 && oi < xw as i32 && oj >= 0 && oj < yw as i32 {
                    data[(oi as usize, oj as usize)] = OCCUPIED;
                }
            }
        }
    }
}

/// Generate occupancy grid using flood fill from the sensor origin.
fn generate_flood_fill(
    data: &mut DMatrix<f64>,
    ox: &[f64],
    oy: &[f64],
    config: &GridMapConfig,
    resolution: f64,
    center_ix: usize,
    center_iy: usize,
) {
    let xw = config.x_width;
    let yw = config.y_width;

    // Phase 1: Trace rays between consecutive obstacle points to mark boundary free cells.
    let mut prev_ix = center_ix as i32 - 1;
    let mut prev_iy = center_iy as i32;
    for (&x, &y) in ox.iter().zip(oy.iter()) {
        let ix = ((x - config.min_x) / resolution).round() as i32;
        let iy = ((y - config.min_y) / resolution).round() as i32;

        let line = bresenham((prev_ix, prev_iy), (ix, iy));
        for &(bx, by) in &line {
            if bx >= 0 && bx < xw as i32 && by >= 0 && by < yw as i32 {
                data[(bx as usize, by as usize)] = FREE;
            }
        }

        prev_ix = ix;
        prev_iy = iy;
    }

    // Phase 2: Flood fill from the sensor center to mark all reachable unknown cells as free.
    flood_fill(center_ix, center_iy, data);

    // Phase 3: Mark obstacle cells and their neighbors as occupied.
    for (&x, &y) in ox.iter().zip(oy.iter()) {
        let ix = ((x - config.min_x) / resolution).round() as i32;
        let iy = ((y - config.min_y) / resolution).round() as i32;
        for di in 0..=1_i32 {
            for dj in 0..=1_i32 {
                let oi = ix + di;
                let oj = iy + dj;
                if oi >= 0 && oi < xw as i32 && oj >= 0 && oj < yw as i32 {
                    data[(oi as usize, oj as usize)] = OCCUPIED;
                }
            }
        }
    }
}

/// BFS flood fill from the given starting point. Cells with value `UNKNOWN` (0.5)
/// that are reachable from the start are set to `FREE` (0.0).
fn flood_fill(start_x: usize, start_y: usize, data: &mut DMatrix<f64>) {
    let (sx, sy) = data.shape();
    let mut queue = VecDeque::new();
    queue.push_back((start_x, start_y));

    while let Some((nx, ny)) = queue.pop_back() {
        // West
        if nx > 0 && (data[(nx - 1, ny)] - UNKNOWN).abs() < f64::EPSILON {
            data[(nx - 1, ny)] = FREE;
            queue.push_front((nx - 1, ny));
        }
        // East
        if nx < sx - 1 && (data[(nx + 1, ny)] - UNKNOWN).abs() < f64::EPSILON {
            data[(nx + 1, ny)] = FREE;
            queue.push_front((nx + 1, ny));
        }
        // North
        if ny > 0 && (data[(nx, ny - 1)] - UNKNOWN).abs() < f64::EPSILON {
            data[(nx, ny - 1)] = FREE;
            queue.push_front((nx, ny - 1));
        }
        // South
        if ny < sy - 1 && (data[(nx, ny + 1)] - UNKNOWN).abs() < f64::EPSILON {
            data[(nx, ny + 1)] = FREE;
            queue.push_front((nx, ny + 1));
        }
    }
}

fn occupancy_style(value: f64) -> (&'static str, f64) {
    if value >= 0.99 {
        ("#b91c1c", 0.95) // occupied - red
    } else if value >= 0.49 {
        ("#94a3b8", 0.60) // unknown - grey
    } else {
        ("#f0fdf4", 0.85) // free - light green
    }
}

/// Build an SVG visualization of the LIDAR grid map.
pub fn build_lidar_grid_map_svg(grid_map: &LidarGridMap, ox: &[f64], oy: &[f64]) -> String {
    let width = 920.0;
    let height = 700.0;
    let margin = 72.0;
    let side_panel_width = 220.0;
    let available_plot_width = width - 2.0 * margin - side_panel_width;
    let available_plot_height = height - 2.0 * margin;
    let x_range = grid_map.max_x - grid_map.min_x;
    let y_range = grid_map.max_y - grid_map.min_y;
    let scale = (available_plot_width / x_range)
        .min(available_plot_height / y_range)
        .max(1.0);
    let plot_width = x_range * scale;
    let plot_height = y_range * scale;
    let plot_left = margin;
    let plot_top = margin + (available_plot_height - plot_height) / 2.0;
    let legend_x = plot_left + plot_width + 36.0;
    let legend_y = plot_top + 24.0;

    let to_svg_x = |x: f64| plot_left + (x - grid_map.min_x) * scale;
    let to_svg_y = |y: f64| plot_top + plot_height - (y - grid_map.min_y) * scale;
    let cell_size = grid_map.resolution * scale;
    let origin_x = grid_map.min_x + grid_map.center_ix as f64 * grid_map.resolution;
    let origin_y = grid_map.min_y + grid_map.center_iy as f64 * grid_map.resolution;

    let mut svg = String::new();
    svg.push_str(&format!(
        "<?xml version='1.0' encoding='UTF-8'?>\n<svg xmlns='http://www.w3.org/2000/svg' width='{width}' height='{height}' viewBox='0 0 {width} {height}'>\n"
    ));
    svg.push_str("<rect width='100%' height='100%' fill='white'/>\n");
    svg.push_str(&format!(
        "<text x='{:.1}' y='38' text-anchor='middle' font-size='24' font-family='sans-serif' font-weight='700' fill='#111827'>LIDAR to Grid Map</text>\n",
        width / 2.0
    ));
    svg.push_str(&format!(
        "<rect x='{plot_left:.1}' y='{plot_top:.1}' width='{plot_width:.1}' height='{plot_height:.1}' fill='#fffdf7' stroke='#d6d3d1' stroke-width='2'/>\n"
    ));

    // X axis ticks
    let tick_step = pick_tick_step(x_range);
    let x_tick_start = (grid_map.min_x / tick_step).ceil() as i32;
    let x_tick_end = (grid_map.max_x / tick_step).floor() as i32;
    for tick in x_tick_start..=x_tick_end {
        let x = tick as f64 * tick_step;
        let sx = to_svg_x(x);
        svg.push_str(&format!(
            "<line x1='{sx:.2}' y1='{plot_top:.2}' x2='{sx:.2}' y2='{:.2}' stroke='#ece7dc' stroke-width='1'/>\n",
            plot_top + plot_height
        ));
        svg.push_str(&format!(
            "<text x='{sx:.2}' y='{:.2}' text-anchor='middle' font-size='14' font-family='sans-serif' fill='#374151'>{x:.1}</text>\n",
            plot_top + plot_height + 24.0
        ));
    }

    // Y axis ticks
    let y_tick_step = pick_tick_step(y_range);
    let y_tick_start = (grid_map.min_y / y_tick_step).ceil() as i32;
    let y_tick_end = (grid_map.max_y / y_tick_step).floor() as i32;
    for tick in y_tick_start..=y_tick_end {
        let y = tick as f64 * y_tick_step;
        let sy = to_svg_y(y);
        svg.push_str(&format!(
            "<line x1='{plot_left:.2}' y1='{sy:.2}' x2='{:.2}' y2='{sy:.2}' stroke='#ece7dc' stroke-width='1'/>\n",
            plot_left + plot_width
        ));
        svg.push_str(&format!(
            "<text x='{:.2}' y='{:.2}' text-anchor='end' font-size='14' font-family='sans-serif' fill='#374151'>{y:.1}</text>\n",
            plot_left - 10.0,
            sy + 5.0
        ));
    }

    // Grid cells
    for iy in 0..grid_map.y_width {
        for ix in 0..grid_map.x_width {
            let value = grid_map.data[(ix, iy)];
            let x = grid_map.min_x + ix as f64 * grid_map.resolution;
            let y = grid_map.min_y + iy as f64 * grid_map.resolution;
            let sx = to_svg_x(x);
            let sy = to_svg_y(y + grid_map.resolution);
            let (fill, opacity) = occupancy_style(value);
            svg.push_str(&format!(
                "<rect x='{sx:.2}' y='{sy:.2}' width='{cell_size:.2}' height='{cell_size:.2}' fill='{fill}' fill-opacity='{opacity:.3}' stroke='none'/>\n"
            ));
        }
    }

    // Obstacle points
    for (x, y) in ox.iter().zip(oy.iter()) {
        svg.push_str(&format!(
            "<circle cx='{:.2}' cy='{:.2}' r='3.8' fill='#111827' stroke='white' stroke-width='1.2'/>\n",
            to_svg_x(*x),
            to_svg_y(*y)
        ));
    }

    // Sensor origin
    svg.push_str(&format!(
        "<circle cx='{:.2}' cy='{:.2}' r='6' fill='#2563eb' stroke='white' stroke-width='2'/>\n",
        to_svg_x(origin_x),
        to_svg_y(origin_y)
    ));

    // Legend
    svg.push_str(&format!(
        "<rect x='{legend_x:.1}' y='{legend_y:.1}' width='182' height='180' rx='18' fill='white' stroke='#d6d3d1'/>\n"
    ));
    let legend_rows = [
        ("#b91c1c", "Occupied cell"),
        ("#94a3b8", "Unknown cell"),
        ("#f0fdf4", "Free space"),
    ];
    for (index, (color, label)) in legend_rows.iter().enumerate() {
        let row_y = legend_y + 28.0 + index as f64 * 26.0;
        svg.push_str(&format!(
            "<rect x='{:.1}' y='{:.1}' width='22' height='14' rx='4' fill='{color}' stroke='#cbd5e1'/>\n",
            legend_x + 14.0,
            row_y - 10.0
        ));
        svg.push_str(&format!(
            "<text x='{:.1}' y='{:.1}' font-size='16' font-family='sans-serif' fill='#111827'>{label}</text>\n",
            legend_x + 48.0,
            row_y + 1.0
        ));
    }
    let obstacle_row_y = legend_y + 112.0;
    svg.push_str(&format!(
        "<circle cx='{:.1}' cy='{obstacle_row_y:.1}' r='4.0' fill='#111827' stroke='white' stroke-width='1.2'/>\n",
        legend_x + 25.0
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='16' font-family='sans-serif' fill='#111827'>Obstacle points</text>\n",
        legend_x + 48.0,
        obstacle_row_y + 1.0
    ));
    let sensor_row_y = legend_y + 138.0;
    svg.push_str(&format!(
        "<circle cx='{:.1}' cy='{sensor_row_y:.1}' r='5.5' fill='#2563eb' stroke='white' stroke-width='1.5'/>\n",
        legend_x + 25.0
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='16' font-family='sans-serif' fill='#111827'>Sensor origin</text>\n",
        legend_x + 48.0,
        sensor_row_y + 1.0
    ));

    // Axis labels
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' text-anchor='middle' font-size='16' font-family='sans-serif' fill='#111827'>x [m]</text>\n",
        plot_left + plot_width / 2.0,
        plot_top + plot_height + 52.0
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' text-anchor='middle' font-size='16' font-family='sans-serif' fill='#111827' transform='rotate(-90, {:.1}, {:.1})'>y [m]</text>\n",
        plot_left - 48.0,
        plot_top + plot_height / 2.0,
        plot_left - 48.0,
        plot_top + plot_height / 2.0
    ));
    svg.push_str("</svg>\n");

    svg
}

/// Choose a reasonable tick step for the given axis range.
fn pick_tick_step(range: f64) -> f64 {
    if range > 20.0 {
        10.0
    } else if range > 4.0 {
        2.0
    } else if range > 1.0 {
        0.5
    } else {
        0.1
    }
}

/// Save a LIDAR grid map SVG to a file.
pub fn save_lidar_grid_map_svg(
    filename: &str,
    grid_map: &LidarGridMap,
    ox: &[f64],
    oy: &[f64],
) -> std::io::Result<()> {
    let svg = build_lidar_grid_map_svg(grid_map, ox, oy);
    let mut file = File::create(filename)?;
    file.write_all(svg.as_bytes())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Generate a simple circular scan for testing.
    fn circular_scan(n: usize, radius: f64) -> (Vec<f64>, Vec<f64>) {
        let angles: Vec<f64> = (0..n).map(|i| 2.0 * PI * i as f64 / n as f64).collect();
        let distances: Vec<f64> = vec![radius; n];
        (angles, distances)
    }

    #[test]
    fn test_bresenham_horizontal() {
        let pts = bresenham((0, 0), (5, 0));
        assert_eq!(pts.len(), 6);
        assert_eq!(pts[0], (0, 0));
        assert_eq!(pts[5], (5, 0));
    }

    #[test]
    fn test_bresenham_vertical() {
        let pts = bresenham((0, 0), (0, 5));
        assert_eq!(pts.len(), 6);
        assert_eq!(pts[0], (0, 0));
        assert_eq!(pts[5], (0, 5));
    }

    #[test]
    fn test_bresenham_diagonal() {
        let pts = bresenham((0, 0), (3, 3));
        assert_eq!(pts.len(), 4);
        for (i, &(x, y)) in pts.iter().enumerate() {
            assert_eq!(x, i as i32);
            assert_eq!(y, i as i32);
        }
    }

    #[test]
    fn test_bresenham_reversed() {
        let fwd = bresenham((1, 1), (5, 3));
        let rev = bresenham((5, 3), (1, 1));
        // Reversed line should produce same set of points in opposite order.
        assert_eq!(fwd.len(), rev.len());
        for (a, b) in fwd.iter().zip(rev.iter().rev()) {
            assert_eq!(a, b);
        }
    }

    #[test]
    fn test_grid_map_bresenham_basic() {
        let (angles, distances) = circular_scan(36, 5.0);
        let grid = LidarGridMap::from_scan(&angles, &distances, 0.5, GridMapMethod::Bresenham);

        assert!(grid.x_width > 0);
        assert!(grid.y_width > 0);

        // Sensor origin should be free.
        let origin_val = grid.data[(grid.center_ix, grid.center_iy)];
        assert!(
            origin_val < 0.01,
            "origin should be free, got {}",
            origin_val
        );

        // At least some cells should be occupied.
        let occupied_count = grid.data.iter().filter(|&&v| v >= 0.99).count();
        assert!(occupied_count > 0, "expected some occupied cells");
    }

    #[test]
    fn test_grid_map_flood_fill_basic() {
        let (angles, distances) = circular_scan(36, 5.0);
        let grid = LidarGridMap::from_scan(&angles, &distances, 0.5, GridMapMethod::FloodFill);

        // Sensor origin should be free.
        let origin_val = grid.data[(grid.center_ix, grid.center_iy)];
        assert!(
            origin_val < 0.01,
            "origin should be free, got {}",
            origin_val
        );

        // At least some cells should be occupied.
        let occupied_count = grid.data.iter().filter(|&&v| v >= 0.99).count();
        assert!(occupied_count > 0, "expected some occupied cells");
    }

    #[test]
    fn test_grid_map_from_cartesian() {
        let ox = vec![1.0, 0.0, -1.0, 0.0];
        let oy = vec![0.0, 1.0, 0.0, -1.0];
        let grid = LidarGridMap::new(&ox, &oy, 0.5, GridMapMethod::Bresenham);

        // All obstacle positions should be within bounds.
        for (&x, &y) in ox.iter().zip(oy.iter()) {
            assert!(grid.get_index(x, y).is_some());
        }
    }

    #[test]
    fn test_get_occupancy_out_of_bounds() {
        let grid = LidarGridMap::new(&[1.0], &[1.0], 0.5, GridMapMethod::Bresenham);
        assert!(grid.get_occupancy(1000.0, 1000.0).is_none());
    }

    #[test]
    fn test_svg_output() {
        let ox = vec![1.0, 0.0, -1.0, 0.0];
        let oy = vec![0.0, 1.0, 0.0, -1.0];
        let grid = LidarGridMap::new(&ox, &oy, 0.5, GridMapMethod::Bresenham);
        let svg = build_lidar_grid_map_svg(&grid, &ox, &oy);

        assert!(svg.contains("<rect width='100%' height='100%' fill='white'/>"));
        assert!(svg.contains("LIDAR to Grid Map"));
        assert!(svg.contains("Occupied cell"));
        assert!(svg.contains("Free space"));
        assert!(svg.contains("Sensor origin"));
    }

    #[test]
    fn test_both_methods_produce_occupied_and_free() {
        let (angles, distances) = circular_scan(72, 3.0);

        for method in [GridMapMethod::Bresenham, GridMapMethod::FloodFill] {
            let grid = LidarGridMap::from_scan(&angles, &distances, 0.25, method);
            let free_count = grid.data.iter().filter(|&&v| v < 0.01).count();
            let occupied_count = grid.data.iter().filter(|&&v| v >= 0.99).count();
            assert!(free_count > 0, "{:?}: expected free cells", method);
            assert!(occupied_count > 0, "{:?}: expected occupied cells", method);
        }
    }
}
