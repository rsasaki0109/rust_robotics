//! Ray Casting Grid Map for occupancy mapping.
//!
//! Based on the work by Atsushi Sakai (@Atsushi_twi) and Ryohei Sasaki (@rsasaki0109).

use nalgebra::DMatrix;
use std::collections::HashMap;
use std::f64::consts::PI;
use std::fs::File;
use std::io::Write;

/// Grid map extension area \[m\].
pub const EXTEND_AREA: f64 = 10.0;
/// Default grid resolution \[m\].
pub const XY_RESOLUTION: f64 = 0.25;
/// Default yaw resolution \[rad\] (10 degrees).
pub const YAW_RESOLUTION: f64 = PI / 18.0;

/// Cell information for ray casting.
#[derive(Clone)]
struct CellInfo {
    ix: usize,
    iy: usize,
    distance: f64,
}

/// Precomputed data for ray casting.
struct PrecastDB {
    /// Maps angle_id -> Vec<CellInfo> sorted by distance.
    cells_by_angle: HashMap<i32, Vec<CellInfo>>,
}

/// Ray Casting Grid Map for occupancy mapping.
pub struct RayCastingGridMap {
    pub data: DMatrix<f64>,
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
    pub resolution: f64,
    pub x_width: usize,
    pub y_width: usize,
    precast_db: PrecastDB,
    origin_ix: usize,
    origin_iy: usize,
}

impl RayCastingGridMap {
    /// Create a new Ray Casting Grid Map.
    pub fn new(ox: &[f64], oy: &[f64], resolution: f64, yaw_resolution: f64) -> Self {
        let min_x = ox.iter().cloned().fold(f64::INFINITY, f64::min) - EXTEND_AREA;
        let min_y = oy.iter().cloned().fold(f64::INFINITY, f64::min) - EXTEND_AREA;
        let max_x = ox.iter().cloned().fold(f64::NEG_INFINITY, f64::max) + EXTEND_AREA;
        let max_y = oy.iter().cloned().fold(f64::NEG_INFINITY, f64::max) + EXTEND_AREA;

        let x_width = ((max_x - min_x) / resolution).round() as usize;
        let y_width = ((max_y - min_y) / resolution).round() as usize;

        let origin_x = (min_x + max_x) / 2.0;
        let origin_y = (min_y + max_y) / 2.0;
        let origin_ix = ((origin_x - min_x) / resolution).round() as usize;
        let origin_iy = ((origin_y - min_y) / resolution).round() as usize;

        let data = DMatrix::from_element(x_width, y_width, 0.0);

        let precast_db = Self::precompute_db(
            x_width,
            y_width,
            origin_ix,
            origin_iy,
            resolution,
            yaw_resolution,
        );

        let mut grid_map = RayCastingGridMap {
            data,
            min_x,
            min_y,
            max_x,
            max_y,
            resolution,
            x_width,
            y_width,
            precast_db,
            origin_ix,
            origin_iy,
        };

        for (obs_x, obs_y) in ox.iter().zip(oy.iter()) {
            grid_map.ray_casting(*obs_x, *obs_y, yaw_resolution);
        }

        grid_map
    }

    /// Precompute database of cells organized by angle.
    fn precompute_db(
        x_width: usize,
        y_width: usize,
        origin_ix: usize,
        origin_iy: usize,
        resolution: f64,
        yaw_resolution: f64,
    ) -> PrecastDB {
        let mut cells_by_angle: HashMap<i32, Vec<CellInfo>> = HashMap::new();

        for ix in 0..x_width {
            for iy in 0..y_width {
                let dx = ix as f64 - origin_ix as f64;
                let dy = iy as f64 - origin_iy as f64;

                let distance = (dx * dx + dy * dy).sqrt() * resolution;
                let angle = atan_zero_to_twopi(dy, dx);
                let angle_id = (angle / yaw_resolution).floor() as i32;

                let cell_info = CellInfo { ix, iy, distance };

                cells_by_angle.entry(angle_id).or_default().push(cell_info);
            }
        }

        for cells in cells_by_angle.values_mut() {
            cells.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
        }

        PrecastDB { cells_by_angle }
    }

    /// Perform ray casting for a single obstacle.
    fn ray_casting(&mut self, obs_x: f64, obs_y: f64, yaw_resolution: f64) {
        let obs_ix = ((obs_x - self.min_x) / self.resolution).round() as i32;
        let obs_iy = ((obs_y - self.min_y) / self.resolution).round() as i32;

        if obs_ix < 0
            || obs_ix >= self.x_width as i32
            || obs_iy < 0
            || obs_iy >= self.y_width as i32
        {
            return;
        }

        let dx = obs_ix as f64 - self.origin_ix as f64;
        let dy = obs_iy as f64 - self.origin_iy as f64;
        let obs_angle = atan_zero_to_twopi(dy, dx);
        let obs_distance = (dx * dx + dy * dy).sqrt() * self.resolution;
        let angle_id = (obs_angle / yaw_resolution).floor() as i32;

        if let Some(cells) = self.precast_db.cells_by_angle.get(&angle_id) {
            for cell in cells {
                if cell.distance < obs_distance {
                    self.data[(cell.ix, cell.iy)] = 0.5;
                } else if (cell.distance - obs_distance).abs() < self.resolution {
                    self.data[(cell.ix, cell.iy)] = 1.0;
                    break;
                }
            }
        }
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

    /// Get occupancy at world coordinates.
    pub fn get_occupancy(&self, x: f64, y: f64) -> Option<f64> {
        self.get_index(x, y).map(|(ix, iy)| self.data[(ix, iy)])
    }
}

/// Convert atan2 result to 0-2pi range.
fn atan_zero_to_twopi(y: f64, x: f64) -> f64 {
    let angle = y.atan2(x);
    if angle < 0.0 {
        angle + 2.0 * PI
    } else {
        angle
    }
}

fn occupancy_style(value: f64) -> (&'static str, f64) {
    if value >= 0.99 {
        ("#b91c1c", 0.95)
    } else if value >= 0.49 {
        ("#bfdbfe", 0.92)
    } else {
        ("#f8fafc", 0.75)
    }
}

/// Build an SVG visualization of the ray casting grid map.
pub fn build_ray_casting_grid_map_svg(
    grid_map: &RayCastingGridMap,
    ox: &[f64],
    oy: &[f64],
) -> String {
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
    let origin_x = grid_map.min_x + grid_map.origin_ix as f64 * grid_map.resolution;
    let origin_y = grid_map.min_y + grid_map.origin_iy as f64 * grid_map.resolution;

    let mut svg = String::new();
    svg.push_str(&format!(
        "<?xml version='1.0' encoding='UTF-8'?>\n<svg xmlns='http://www.w3.org/2000/svg' width='{width}' height='{height}' viewBox='0 0 {width} {height}'>\n"
    ));
    svg.push_str("<rect width='100%' height='100%' fill='white'/>\n");
    svg.push_str(&format!(
        "<text x='{:.1}' y='38' text-anchor='middle' font-size='24' font-family='sans-serif' font-weight='700' fill='#111827'>Ray Casting Grid Map</text>\n",
        width / 2.0
    ));
    svg.push_str(&format!(
        "<rect x='{plot_left:.1}' y='{plot_top:.1}' width='{plot_width:.1}' height='{plot_height:.1}' fill='#fffdf7' stroke='#d6d3d1' stroke-width='2'/>\n"
    ));

    let x_tick_start = (grid_map.min_x / 10.0).ceil() as i32;
    let x_tick_end = (grid_map.max_x / 10.0).floor() as i32;
    for tick in x_tick_start..=x_tick_end {
        let x = tick as f64 * 10.0;
        let sx = to_svg_x(x);
        svg.push_str(&format!(
            "<line x1='{sx:.2}' y1='{plot_top:.2}' x2='{sx:.2}' y2='{:.2}' stroke='#ece7dc' stroke-width='1'/>\n",
            plot_top + plot_height
        ));
        svg.push_str(&format!(
            "<text x='{sx:.2}' y='{:.2}' text-anchor='middle' font-size='14' font-family='sans-serif' fill='#374151'>{x:.0}</text>\n",
            plot_top + plot_height + 24.0
        ));
    }

    let y_tick_start = (grid_map.min_y / 10.0).ceil() as i32;
    let y_tick_end = (grid_map.max_y / 10.0).floor() as i32;
    for tick in y_tick_start..=y_tick_end {
        let y = tick as f64 * 10.0;
        let sy = to_svg_y(y);
        svg.push_str(&format!(
            "<line x1='{plot_left:.2}' y1='{sy:.2}' x2='{:.2}' y2='{sy:.2}' stroke='#ece7dc' stroke-width='1'/>\n",
            plot_left + plot_width
        ));
        svg.push_str(&format!(
            "<text x='{:.2}' y='{:.2}' text-anchor='end' font-size='14' font-family='sans-serif' fill='#374151'>{y:.0}</text>\n",
            plot_left - 10.0,
            sy + 5.0
        ));
    }

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

    for (x, y) in ox.iter().zip(oy.iter()) {
        svg.push_str(&format!(
            "<circle cx='{:.2}' cy='{:.2}' r='3.8' fill='#111827' stroke='white' stroke-width='1.2'/>\n",
            to_svg_x(*x),
            to_svg_y(*y)
        ));
    }
    svg.push_str(&format!(
        "<circle cx='{:.2}' cy='{:.2}' r='6' fill='#2563eb' stroke='white' stroke-width='2'/>\n",
        to_svg_x(origin_x),
        to_svg_y(origin_y)
    ));

    svg.push_str(&format!(
        "<rect x='{legend_x:.1}' y='{legend_y:.1}' width='182' height='156' rx='18' fill='white' stroke='#d6d3d1'/>\n"
    ));
    let legend_rows = [
        ("#b91c1c", "Occupied cell"),
        ("#bfdbfe", "Free space"),
        ("#f8fafc", "Unknown cell"),
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

    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='15' font-family='sans-serif' font-weight='700' fill='#111827'>Occupancy bands</text>\n",
        legend_x + 18.0,
        legend_y + 190.0
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='13' font-family='monospace' fill='#374151'>1.0 obstacle</text>\n",
        legend_x + 18.0,
        legend_y + 216.0
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='13' font-family='monospace' fill='#374151'>0.5 free</text>\n",
        legend_x + 18.0,
        legend_y + 238.0
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='13' font-family='monospace' fill='#374151'>0.0 unknown</text>\n",
        legend_x + 18.0,
        legend_y + 260.0
    ));

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

/// Save a ray casting grid map SVG to a file.
pub fn save_ray_casting_grid_map_svg(
    filename: &str,
    grid_map: &RayCastingGridMap,
    ox: &[f64],
    oy: &[f64],
) -> std::io::Result<()> {
    let svg = build_ray_casting_grid_map_svg(grid_map, ox, oy);
    let mut file = File::create(filename)?;
    file.write_all(svg.as_bytes())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ray_casting_svg_has_white_background() {
        let grid_map = RayCastingGridMap::new(&[-1.0, 1.0], &[0.0, 0.0], 1.0, PI / 4.0);
        let svg = build_ray_casting_grid_map_svg(&grid_map, &[-1.0, 1.0], &[0.0, 0.0]);

        assert!(svg.contains("<rect width='100%' height='100%' fill='white'/>"));
        assert!(svg.contains("Ray Casting Grid Map"));
        assert!(svg.contains("Occupied cell"));
        assert!(svg.contains("Sensor origin"));
    }
}
