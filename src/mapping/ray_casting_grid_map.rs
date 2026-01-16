// Ray Casting Grid Map
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109)
//         Rust port

use nalgebra::DMatrix;
use gnuplot::{Figure, AxesCommon, AutoOption, PlotOption};
use std::collections::HashMap;
use std::f64::consts::PI;

// Parameters
const EXTEND_AREA: f64 = 10.0; // [m] grid map extension area
const XY_RESOLUTION: f64 = 0.25; // [m] grid resolution
const YAW_RESOLUTION: f64 = PI / 18.0; // [rad] angle resolution (10 degrees)

/// Cell information for ray casting
#[derive(Clone)]
struct CellInfo {
    ix: usize,
    iy: usize,
    distance: f64,
}

/// Precomputed data for ray casting
struct PrecastDB {
    /// Maps angle_id -> Vec<CellInfo> sorted by distance
    cells_by_angle: HashMap<i32, Vec<CellInfo>>,
}

/// Ray Casting Grid Map for occupancy mapping
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
    /// Create a new Ray Casting Grid Map
    pub fn new(ox: &[f64], oy: &[f64], resolution: f64, yaw_resolution: f64) -> Self {
        // Calculate grid bounds
        let min_x = ox.iter().cloned().fold(f64::INFINITY, f64::min) - EXTEND_AREA;
        let min_y = oy.iter().cloned().fold(f64::INFINITY, f64::min) - EXTEND_AREA;
        let max_x = ox.iter().cloned().fold(f64::NEG_INFINITY, f64::max) + EXTEND_AREA;
        let max_y = oy.iter().cloned().fold(f64::NEG_INFINITY, f64::max) + EXTEND_AREA;

        let x_width = ((max_x - min_x) / resolution).round() as usize;
        let y_width = ((max_y - min_y) / resolution).round() as usize;

        // Origin (sensor position) at center
        let origin_x = (min_x + max_x) / 2.0;
        let origin_y = (min_y + max_y) / 2.0;
        let origin_ix = ((origin_x - min_x) / resolution).round() as usize;
        let origin_iy = ((origin_y - min_y) / resolution).round() as usize;

        // Initialize grid with unknown (0.0)
        let data = DMatrix::from_element(x_width, y_width, 0.0);

        // Precompute cell database
        let precast_db = Self::precompute_db(
            x_width, y_width, origin_ix, origin_iy, resolution, yaw_resolution
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

        // Apply ray casting for each obstacle
        for (obs_x, obs_y) in ox.iter().zip(oy.iter()) {
            grid_map.ray_casting(*obs_x, *obs_y, yaw_resolution);
        }

        grid_map
    }

    /// Precompute database of cells organized by angle
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

                cells_by_angle
                    .entry(angle_id)
                    .or_insert_with(Vec::new)
                    .push(cell_info);
            }
        }

        // Sort cells by distance for each angle
        for cells in cells_by_angle.values_mut() {
            cells.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
        }

        PrecastDB { cells_by_angle }
    }

    /// Perform ray casting for a single obstacle
    fn ray_casting(&mut self, obs_x: f64, obs_y: f64, yaw_resolution: f64) {
        // Convert obstacle position to grid coordinates
        let obs_ix = ((obs_x - self.min_x) / self.resolution).round() as i32;
        let obs_iy = ((obs_y - self.min_y) / self.resolution).round() as i32;

        if obs_ix < 0 || obs_ix >= self.x_width as i32 ||
           obs_iy < 0 || obs_iy >= self.y_width as i32 {
            return;
        }

        // Calculate angle to obstacle
        let dx = obs_ix as f64 - self.origin_ix as f64;
        let dy = obs_iy as f64 - self.origin_iy as f64;
        let obs_angle = atan_zero_to_twopi(dy, dx);
        let obs_distance = (dx * dx + dy * dy).sqrt() * self.resolution;
        let angle_id = (obs_angle / yaw_resolution).floor() as i32;

        // Get all cells at this angle
        if let Some(cells) = self.precast_db.cells_by_angle.get(&angle_id) {
            for cell in cells {
                if cell.distance < obs_distance {
                    // Free space (before obstacle)
                    self.data[(cell.ix, cell.iy)] = 0.5;
                } else if (cell.distance - obs_distance).abs() < self.resolution {
                    // Obstacle
                    self.data[(cell.ix, cell.iy)] = 1.0;
                    break;
                }
            }
        }
    }

    /// Get grid index from world coordinates
    pub fn get_index(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let ix = ((x - self.min_x) / self.resolution).floor() as i32;
        let iy = ((y - self.min_y) / self.resolution).floor() as i32;

        if ix >= 0 && ix < self.x_width as i32 && iy >= 0 && iy < self.y_width as i32 {
            Some((ix as usize, iy as usize))
        } else {
            None
        }
    }

    /// Get occupancy at world coordinates
    pub fn get_occupancy(&self, x: f64, y: f64) -> Option<f64> {
        self.get_index(x, y).map(|(ix, iy)| self.data[(ix, iy)])
    }
}

/// Convert atan2 result to 0-2π range
fn atan_zero_to_twopi(y: f64, x: f64) -> f64 {
    let angle = y.atan2(x);
    if angle < 0.0 {
        angle + 2.0 * PI
    } else {
        angle
    }
}

/// Draw heatmap of the grid map
fn draw_heatmap(grid_map: &RayCastingGridMap, ox: &[f64], oy: &[f64]) {
    let mut fig = Figure::new();

    let x_width = grid_map.x_width;
    let y_width = grid_map.y_width;

    // Convert matrix to Vec for gnuplot
    let mut z_data: Vec<f64> = Vec::with_capacity(x_width * y_width);
    for iy in 0..y_width {
        for ix in 0..x_width {
            z_data.push(grid_map.data[(ix, iy)]);
        }
    }

    fig.axes2d()
        .set_title("Ray Casting Grid Map", &[])
        .set_x_label("x [m]", &[])
        .set_y_label("y [m]", &[])
        .set_aspect_ratio(AutoOption::Fix(1.0))
        .image(
            z_data.iter().cloned(),
            x_width,
            y_width,
            Some((grid_map.min_x, grid_map.min_y, grid_map.max_x, grid_map.max_y)),
            &[PlotOption::Caption("Occupancy")]
        )
        .points(
            ox,
            oy,
            &[PlotOption::Caption("Obstacles"), PlotOption::Color("red"), gnuplot::PointSymbol('O'), gnuplot::PointSize(1.0)]
        );

    fig.show_and_keep_running().unwrap();
    fig.save_to_svg("./img/mapping/ray_casting_grid_map.svg", 640, 480).unwrap();
    println!("Plot saved to ./img/mapping/ray_casting_grid_map.svg");
}

fn main() {
    println!("Ray Casting Grid Map start!");

    // Define obstacles (simulating a square room with walls)
    let mut ox: Vec<f64> = Vec::new();
    let mut oy: Vec<f64> = Vec::new();

    // Bottom wall
    for i in -20..=20 {
        ox.push(i as f64);
        oy.push(-20.0);
    }
    // Top wall
    for i in -20..=20 {
        ox.push(i as f64);
        oy.push(20.0);
    }
    // Left wall
    for i in -20..=20 {
        ox.push(-20.0);
        oy.push(i as f64);
    }
    // Right wall
    for i in -20..=20 {
        ox.push(20.0);
        oy.push(i as f64);
    }

    // Add some internal obstacles
    for i in -10..=10 {
        ox.push(i as f64);
        oy.push(i as f64);
    }

    // Create Ray Casting Grid Map
    let grid_map = RayCastingGridMap::new(&ox, &oy, XY_RESOLUTION, YAW_RESOLUTION);

    println!("Grid map created:");
    println!("  Size: {} x {}", grid_map.x_width, grid_map.y_width);
    println!("  Bounds: ({:.1}, {:.1}) to ({:.1}, {:.1})",
             grid_map.min_x, grid_map.min_y, grid_map.max_x, grid_map.max_y);

    // Draw heatmap
    draw_heatmap(&grid_map, &ox, &oy);

    println!("Done!");
}
