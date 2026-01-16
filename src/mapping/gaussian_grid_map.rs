// Gaussian Grid Map
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109)
//         Rust port

use nalgebra::DMatrix;
use gnuplot::{Figure, AxesCommon, AutoOption, PlotOption};
use std::f64::consts::PI;

// Parameters
const EXTEND_AREA: f64 = 10.0; // [m] grid map extension area
const XY_RESOLUTION: f64 = 0.5; // [m] grid resolution
const STD: f64 = 5.0; // standard deviation for Gaussian

/// Gaussian Grid Map for occupancy mapping
pub struct GaussianGridMap {
    pub data: DMatrix<f64>,
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
    pub resolution: f64,
    pub x_width: usize,
    pub y_width: usize,
}

impl GaussianGridMap {
    /// Create a new Gaussian Grid Map from obstacle positions
    pub fn new(ox: &[f64], oy: &[f64], resolution: f64, std_dev: f64) -> Self {
        // Calculate grid bounds
        let min_x = ox.iter().cloned().fold(f64::INFINITY, f64::min) - EXTEND_AREA;
        let min_y = oy.iter().cloned().fold(f64::INFINITY, f64::min) - EXTEND_AREA;
        let max_x = ox.iter().cloned().fold(f64::NEG_INFINITY, f64::max) + EXTEND_AREA;
        let max_y = oy.iter().cloned().fold(f64::NEG_INFINITY, f64::max) + EXTEND_AREA;

        let x_width = ((max_x - min_x) / resolution).round() as usize;
        let y_width = ((max_y - min_y) / resolution).round() as usize;

        let mut data = DMatrix::from_element(x_width, y_width, 0.0);

        // Calculate probability for each cell
        for ix in 0..x_width {
            for iy in 0..y_width {
                let x = min_x + ix as f64 * resolution;
                let y = min_y + iy as f64 * resolution;

                // Find minimum distance to obstacles
                let min_dist = ox.iter().zip(oy.iter())
                    .map(|(ox_i, oy_i)| ((x - ox_i).powi(2) + (y - oy_i).powi(2)).sqrt())
                    .fold(f64::INFINITY, f64::min);

                // Apply Gaussian: P = 1 - CDF(distance)
                // This gives higher probability near obstacles
                let pdf = 1.0 - normal_cdf(min_dist, 0.0, std_dev);
                data[(ix, iy)] = pdf;
            }
        }

        GaussianGridMap {
            data,
            min_x,
            min_y,
            max_x,
            max_y,
            resolution,
            x_width,
            y_width,
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

    /// Get probability at world coordinates
    pub fn get_probability(&self, x: f64, y: f64) -> Option<f64> {
        self.get_index(x, y).map(|(ix, iy)| self.data[(ix, iy)])
    }

    /// Get world coordinates from grid index
    pub fn get_xy(&self, ix: usize, iy: usize) -> (f64, f64) {
        let x = self.min_x + ix as f64 * self.resolution;
        let y = self.min_y + iy as f64 * self.resolution;
        (x, y)
    }
}

/// Standard normal CDF using error function approximation
fn normal_cdf(x: f64, mean: f64, std_dev: f64) -> f64 {
    let z = (x - mean) / (std_dev * (2.0_f64).sqrt());
    0.5 * (1.0 + erf(z))
}

/// Error function approximation (Horner's method)
fn erf(x: f64) -> f64 {
    // Approximation constants
    let a1 =  0.254829592;
    let a2 = -0.284496736;
    let a3 =  1.421413741;
    let a4 = -1.453152027;
    let a5 =  1.061405429;
    let p  =  0.3275911;

    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let x = x.abs();

    let t = 1.0 / (1.0 + p * x);
    let y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * (-x * x).exp();

    sign * y
}

/// Draw heatmap of the grid map
fn draw_heatmap(grid_map: &GaussianGridMap, ox: &[f64], oy: &[f64]) {
    let mut fig = Figure::new();

    // Prepare data for heatmap
    let x_width = grid_map.x_width;
    let y_width = grid_map.y_width;

    // Convert matrix to Vec<Vec<f64>> for gnuplot
    let mut z_data: Vec<f64> = Vec::with_capacity(x_width * y_width);
    for iy in 0..y_width {
        for ix in 0..x_width {
            z_data.push(grid_map.data[(ix, iy)]);
        }
    }

    // Create coordinate vectors
    let x_coords: Vec<f64> = (0..x_width)
        .map(|ix| grid_map.min_x + ix as f64 * grid_map.resolution)
        .collect();
    let y_coords: Vec<f64> = (0..y_width)
        .map(|iy| grid_map.min_y + iy as f64 * grid_map.resolution)
        .collect();

    fig.axes2d()
        .set_title("Gaussian Grid Map", &[])
        .set_x_label("x [m]", &[])
        .set_y_label("y [m]", &[])
        .set_aspect_ratio(AutoOption::Fix(1.0))
        .image(
            z_data.iter().cloned(),
            x_width,
            y_width,
            Some((grid_map.min_x, grid_map.min_y, grid_map.max_x, grid_map.max_y)),
            &[PlotOption::Caption("Probability")]
        )
        .points(
            ox,
            oy,
            &[PlotOption::Caption("Obstacles"), PlotOption::Color("white"), gnuplot::PointSymbol('O'), gnuplot::PointSize(1.5)]
        );

    fig.show_and_keep_running().unwrap();
    fig.save_to_svg("./img/mapping/gaussian_grid_map.svg", 640, 480).unwrap();
    println!("Plot saved to ./img/mapping/gaussian_grid_map.svg");
}

fn main() {
    println!("Gaussian Grid Map start!");

    // Define obstacles (sample points)
    let ox: Vec<f64> = vec![
        0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
        0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
    ];
    let oy: Vec<f64> = vec![
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
        0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
        0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
    ];

    // Create Gaussian Grid Map
    let grid_map = GaussianGridMap::new(&ox, &oy, XY_RESOLUTION, STD);

    println!("Grid map created:");
    println!("  Size: {} x {}", grid_map.x_width, grid_map.y_width);
    println!("  Bounds: ({:.1}, {:.1}) to ({:.1}, {:.1})",
             grid_map.min_x, grid_map.min_y, grid_map.max_x, grid_map.max_y);

    // Draw heatmap
    draw_heatmap(&grid_map, &ox, &oy);

    println!("Done!");
}
