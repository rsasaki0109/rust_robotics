//! Occupancy Grid Mapping with log-odds update.
//!
//! Implements a 2-D occupancy grid that fuses range sensor scans using
//! Bayesian log-odds updates and Bresenham ray casting.

/// Configuration parameters for [`OccupancyGridMap`].
#[derive(Clone, Debug)]
pub struct OccupancyGridConfig {
    /// Cell size \[m\].
    pub resolution: f64,
    /// Number of cells along the x-axis.
    pub width: usize,
    /// Number of cells along the y-axis.
    pub height: usize,
    /// Log-odds value assigned to every cell at initialisation.
    pub prior_log_odds: f64,
    /// Log-odds increment added when a cell is observed as occupied.
    pub occupied_log_odds: f64,
    /// Log-odds decrement added when a cell is observed as free.
    pub free_log_odds: f64,
    /// Upper saturation limit for log-odds values.
    pub max_log_odds: f64,
    /// Lower saturation limit for log-odds values.
    pub min_log_odds: f64,
}

impl Default for OccupancyGridConfig {
    fn default() -> Self {
        Self {
            resolution: 0.5,
            width: 100,
            height: 100,
            prior_log_odds: 0.0,
            occupied_log_odds: 0.85,
            free_log_odds: -0.4,
            max_log_odds: 5.0,
            min_log_odds: -5.0,
        }
    }
}

/// Occupancy grid map stored as log-odds values.
///
/// The grid is indexed as `grid[ix][iy]` where `ix` is the column (x-axis)
/// and `iy` is the row (y-axis).  World coordinate `(0, 0)` maps to the
/// centre of the grid.
pub struct OccupancyGridMap {
    /// Log-odds grid: `grid[ix][iy]`.
    pub grid: Vec<Vec<f64>>,
    /// Configuration used to build this map.
    pub config: OccupancyGridConfig,
}

impl OccupancyGridMap {
    /// Create a new occupancy grid initialised to `config.prior_log_odds`.
    pub fn new(config: OccupancyGridConfig) -> Self {
        let grid = vec![vec![config.prior_log_odds; config.height]; config.width];
        Self { grid, config }
    }

    /// Update the map from a single 2-D laser scan.
    ///
    /// # Parameters
    /// - `robot_x`, `robot_y` – robot position in world coordinates \[m\].
    /// - `robot_yaw` – robot heading \[rad\].
    /// - `scan_ranges` – measured ranges for each beam \[m\].
    /// - `angle_min` – angle of the first beam relative to the robot heading \[rad\].
    /// - `angle_increment` – angular step between consecutive beams \[rad\].
    pub fn update_with_scan(
        &mut self,
        robot_x: f64,
        robot_y: f64,
        robot_yaw: f64,
        scan_ranges: &[f64],
        angle_min: f64,
        angle_increment: f64,
    ) {
        let origin = match self.world_to_grid(robot_x, robot_y) {
            Some(o) => o,
            None => return,
        };

        for (i, &range) in scan_ranges.iter().enumerate() {
            if range <= 0.0 || !range.is_finite() {
                continue;
            }

            let angle = robot_yaw + angle_min + i as f64 * angle_increment;
            let end_x = robot_x + range * angle.cos();
            let end_y = robot_y + range * angle.sin();

            let end_cell = self.world_to_grid(end_x, end_y);

            let end_ix = end_cell.map(|(ix, _)| ix as i32).unwrap_or({
                // Clamp endpoint to grid boundary along the ray direction.
                let ix = (end_x / self.config.resolution + self.config.width as f64 / 2.0).round()
                    as i32;
                ix.clamp(0, self.config.width as i32 - 1)
            });
            let end_iy = end_cell.map(|(_, iy)| iy as i32).unwrap_or({
                let iy = (end_y / self.config.resolution + self.config.height as f64 / 2.0).round()
                    as i32;
                iy.clamp(0, self.config.height as i32 - 1)
            });

            // Collect cells along the ray (excluding endpoint).
            let ray_cells = bresenham_line(origin.0 as i32, origin.1 as i32, end_ix, end_iy);

            let free_delta = self.config.free_log_odds;
            let occ_delta = self.config.occupied_log_odds;
            let min_l = self.config.min_log_odds;
            let max_l = self.config.max_log_odds;
            let w = self.config.width;
            let h = self.config.height;

            // Mark free cells along the ray (all cells except the last).
            let ray_len = ray_cells.len();
            for &(cx, cy) in ray_cells.iter().take(ray_len.saturating_sub(1)) {
                if cx >= 0 && cx < w as i32 && cy >= 0 && cy < h as i32 {
                    let l = &mut self.grid[cx as usize][cy as usize];
                    *l = (*l + free_delta).clamp(min_l, max_l);
                }
            }

            // Mark endpoint as occupied (only if it falls inside the grid).
            if let Some((ex, ey)) = end_cell {
                let l = &mut self.grid[ex][ey];
                *l = (*l + occ_delta).clamp(min_l, max_l);
            }
        }
    }

    /// Convert a log-odds value at `(ix, iy)` to an occupancy probability.
    ///
    /// Returns a value in `[0, 1]` via `1 - 1 / (1 + exp(l))`.
    pub fn get_probability(&self, ix: usize, iy: usize) -> f64 {
        let l = self.grid[ix][iy];
        1.0 - 1.0 / (1.0 + l.exp())
    }

    /// Convert world coordinates to grid indices.
    ///
    /// Returns `None` when the point lies outside the grid.
    pub fn world_to_grid(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let ix = (x / self.config.resolution + self.config.width as f64 / 2.0).floor() as i32;
        let iy = (y / self.config.resolution + self.config.height as f64 / 2.0).floor() as i32;

        if ix >= 0 && ix < self.config.width as i32 && iy >= 0 && iy < self.config.height as i32 {
            Some((ix as usize, iy as usize))
        } else {
            None
        }
    }

    /// Return `true` when the occupancy probability at `(ix, iy)` exceeds
    /// `threshold`.
    pub fn is_occupied(&self, ix: usize, iy: usize, threshold: f64) -> bool {
        self.get_probability(ix, iy) > threshold
    }
}

/// Bresenham's line algorithm returning all grid cells from `(x0, y0)` to
/// `(x1, y1)`, inclusive of both endpoints.
fn bresenham_line(x0: i32, y0: i32, x1: i32, y1: i32) -> Vec<(i32, i32)> {
    let mut cells = Vec::new();

    let dx = (x1 - x0).abs();
    let dy = (y1 - y0).abs();
    let sx = if x0 < x1 { 1_i32 } else { -1 };
    let sy = if y0 < y1 { 1_i32 } else { -1 };

    let mut x = x0;
    let mut y = y0;
    let mut err = dx - dy;

    loop {
        cells.push((x, y));
        if x == x1 && y == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 > -dy {
            err -= dy;
            x += sx;
        }
        if e2 < dx {
            err += dx;
            y += sy;
        }
    }

    cells
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Prior probability from log-odds 0 should be 0.5.
    #[test]
    fn test_initial_grid_is_prior_probability() {
        let map = OccupancyGridMap::new(OccupancyGridConfig::default());
        let p = map.get_probability(50, 50);
        println!("Prior probability at (50,50): {p}");
        assert!((p - 0.5).abs() < 1e-10, "Expected 0.5 but got {p}");
    }

    /// After a scan that places an obstacle directly in front of the robot,
    /// the endpoint cell should have probability > 0.5.
    #[test]
    fn test_update_marks_occupied_cell() {
        let mut map = OccupancyGridMap::new(OccupancyGridConfig::default());
        // Robot at origin, single beam pointing along +x at 5 m.
        map.update_with_scan(0.0, 0.0, 0.0, &[5.0], 0.0, 0.0);

        let (ex, ey) = map.world_to_grid(5.0, 0.0).unwrap();
        let p = map.get_probability(ex, ey);
        println!("Occupied cell probability at ({ex},{ey}): {p}");
        assert!(p > 0.5, "Expected occupied (p > 0.5) but got {p}");
    }

    /// Cells between the robot and the obstacle should be updated as free
    /// (probability < 0.5) after several identical scans.
    #[test]
    fn test_update_marks_free_cells_along_ray() {
        let mut map = OccupancyGridMap::new(OccupancyGridConfig::default());
        // Apply the same scan multiple times to overcome the prior.
        for _ in 0..5 {
            map.update_with_scan(0.0, 0.0, 0.0, &[5.0], 0.0, 0.0);
        }

        // A cell at 2 m along the ray should be free.
        let (fx, fy) = map.world_to_grid(2.0, 0.0).unwrap();
        let p = map.get_probability(fx, fy);
        println!("Free cell probability at ({fx},{fy}): {p}");
        assert!(p < 0.5, "Expected free (p < 0.5) but got {p}");
    }

    /// `world_to_grid` should round-trip correctly for a simple coordinate.
    #[test]
    fn test_world_to_grid_conversion() {
        let cfg = OccupancyGridConfig::default();
        let map = OccupancyGridMap::new(cfg.clone());

        // Origin should map to the centre of the grid.
        let (cx, cy) = map.world_to_grid(0.0, 0.0).unwrap();
        println!("Grid centre: ({cx}, {cy})");
        assert_eq!(cx, cfg.width / 2);
        assert_eq!(cy, cfg.height / 2);

        // A point outside the grid should return None.
        let outside = map.world_to_grid(1_000.0, 1_000.0);
        assert!(outside.is_none());
    }
}
