use std::collections::HashMap;

use nalgebra as na; // For matrices, vectors, stats, etc.
use na::{DMatrix, Matrix2, Vector2, SymmetricEigen};
use rand::prelude::*;
use rand_distr::{Distribution, Uniform};
use gnuplot::{Figure, Axes2D, Caption, Color, PointSymbol};
use gnuplot::AxesCommon;

#[derive(Debug, Clone)]
struct NDTGrid {
    n_points: usize,
    mean_x: f64,
    mean_y: f64,
    center_grid_x: f64,
    center_grid_y: f64,
    covariance: Matrix2<f64>,
    eig_values: Vector2<f64>,
    eig_vec: Matrix2<f64>,
}

impl Default for NDTGrid {
    fn default() -> Self {
        Self {
            n_points: 0,
            mean_x: 0.0,
            mean_y: 0.0,
            center_grid_x: 0.0,
            center_grid_y: 0.0,
            covariance: Matrix2::zeros(),
            eig_values: Vector2::zeros(),
            eig_vec: Matrix2::zeros(),
        }
    }
}

/// A 2D grid map that stores an `NDTGrid` in each cell.
struct GridMap {
    width: usize,
    height: usize,
    resolution: f64,
    center_x: f64,
    center_y: f64,
    data: Vec<NDTGrid>,
}

impl GridMap {
    fn new(width: usize,
           height: usize,
           resolution: f64,
           center_x: f64,
           center_y: f64) -> Self
    {
        let data = vec![NDTGrid::default(); width * height];
        GridMap {
            width,
            height,
            resolution,
            center_x,
            center_y,
            data
        }
    }

    /// Convert (x, y) in world coordinates to a single index in the grid.
    /// Returns `None` if out of bounds.
    fn calc_grid_index_from_xy_pos(&self, x: f64, y: f64) -> Option<usize> {
        // Shift so center is 0,0 in grid coordinates:
        let gx = (x - self.center_x) / self.resolution + (self.width as f64) / 2.0;
        let gy = (y - self.center_y) / self.resolution + (self.height as f64) / 2.0;

        let ix = gx.floor() as isize;
        let iy = gy.floor() as isize;

        if ix < 0 || iy < 0 || ix >= self.width as isize || iy >= self.height as isize {
            return None;
        }
        // row-major index, for example: row = iy, col = ix
        let idx = (iy as usize) * self.width + (ix as usize);
        Some(idx)
    }

    /// Convert a single index to the center-world-coordinates of that grid cell.
    /// (Assumes row-major indexing: index = row * width + col).
    fn calc_grid_central_xy_position_from_grid_index(&self, idx: usize) -> (f64, f64) {
        let row = idx / self.width;
        let col = idx % self.width;

        // center in grid coords:
        let cx = (col as f64 + 0.5) - (self.width as f64)/2.0;
        let cy = (row as f64 + 0.5) - (self.height as f64)/2.0;
        // scale by resolution and add the map center:
        let x = self.center_x + cx * self.resolution;
        let y = self.center_y + cy * self.resolution;
        (x, y)
    }
}

/// Normal Distribution Transform (NDT) Map
struct NDTMap {
    min_n_points: usize,
    resolution: f64,
    grid_map: GridMap,

    /// For convenience, store the cluster -> indices map. (grid index -> list of point indices)
    grid_index_map: HashMap<usize, Vec<usize>>,
    ox: Vec<f64>,
    oy: Vec<f64>,
}

impl NDTMap {
    fn new(ox: &[f64], oy: &[f64], resolution: f64) -> Self {
        let min_x = ox.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_x = ox.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_y = oy.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_y = oy.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        let width = ((max_x - min_x) / resolution).ceil() as usize + 3;
        let height = ((max_y - min_y) / resolution).ceil() as usize + 3;
        let center_x = ox.iter().sum::<f64>() / (ox.len() as f64);
        let center_y = oy.iter().sum::<f64>() / (oy.len() as f64);

        let grid_map = GridMap::new(width, height, resolution, center_x, center_y);

        let mut ndt_map = NDTMap {
            min_n_points: 3,
            resolution,
            grid_map,
            grid_index_map: HashMap::new(),
            ox: ox.to_vec(),
            oy: oy.to_vec(),
        };
        ndt_map.grid_index_map = ndt_map.create_grid_index_map(ox, oy);
        ndt_map.construct_grid_map();
        ndt_map
    }

    /// Build each cell's NDTGrid from the points that fall in it.
    fn construct_grid_map(&mut self) {
        for (grid_index, inds) in &self.grid_index_map {
            let mut ndt = NDTGrid::default();
            ndt.n_points = inds.len();
            if ndt.n_points >= self.min_n_points {
                // Compute means:
                let mut sum_x = 0.0;
                let mut sum_y = 0.0;
                for &i in inds {
                    sum_x += self.ox[i];
                    sum_y += self.oy[i];
                }
                let mean_x = sum_x / (inds.len() as f64);
                let mean_y = sum_y / (inds.len() as f64);

                ndt.mean_x = mean_x;
                ndt.mean_y = mean_y;

                // Center of the cell in world coords:
                let (cx, cy) = self.grid_map.calc_grid_central_xy_position_from_grid_index(*grid_index);
                ndt.center_grid_x = cx;
                ndt.center_grid_y = cy;

                // Build the 2xN matrix for covariance calc:
                let mut xs = Vec::with_capacity(inds.len());
                let mut ys = Vec::with_capacity(inds.len());
                for &i in inds {
                    xs.push(self.ox[i]);
                    ys.push(self.oy[i]);
                }
                // Covariance:
                let cov = compute_covariance_2d(&xs, &ys, mean_x, mean_y);
                ndt.covariance = cov;

                // Eigen decomposition:
                let sym_eig = SymmetricEigen::new(cov);
                ndt.eig_values = sym_eig.eigenvalues;
                ndt.eig_vec = sym_eig.eigenvectors;

                // Store
                self.grid_map.data[*grid_index] = ndt;
            }
        }
    }

    /// Return (grid_index -> Vec<point_index>) mapping.
    fn create_grid_index_map(&self, ox: &[f64], oy: &[f64]) -> HashMap<usize, Vec<usize>> {
        let mut map: HashMap<usize, Vec<usize>> = HashMap::new();
        for i in 0..ox.len() {
            if let Some(idx) = self.grid_map.calc_grid_index_from_xy_pos(ox[i], oy[i]) {
                map.entry(idx).or_default().push(i);
            }
        }
        map
    }
}

/// Compute a 2x2 covariance matrix from a set of points, given precomputed means.
fn compute_covariance_2d(xs: &[f64], ys: &[f64], mean_x: f64, mean_y: f64) -> Matrix2<f64> {
    // naive: sum( (x - mean_x)*(x - mean_x) ), etc. then divide by n-1
    let n = xs.len() as f64;
    if n <= 1.0 {
        return Matrix2::zeros();
    }

    let mut sxx = 0.0;
    let mut syy = 0.0;
    let mut sxy = 0.0;
    for (&x, &y) in xs.iter().zip(ys.iter()) {
        let dx = x - mean_x;
        let dy = y - mean_y;
        sxx += dx * dx;
        syy += dy * dy;
        sxy += dx * dy;
    }
    // sample covariance uses n-1
    let denom = n - 1.0;
    Matrix2::new(sxx/denom, sxy/denom,
                 sxy/denom, syy/denom)
}

/// Generates dummy corridor-like data with random noise.
fn create_dummy_observation_data() -> (Vec<f64>, Vec<f64>) {
    let mut ox = Vec::new();
    let mut oy = Vec::new();

    // left corridor
    for y in -50..50 {
        ox.push(-20.0);
        oy.push(y as f64);
    }
    // right corridor 1
    for y in -50..0 {
        ox.push(20.0);
        oy.push(y as f64);
    }
    // right corridor 2
    for x in 20..50 {
        ox.push(x as f64);
        oy.push(0.0);
    }
    // right corridor 3
    // for x in 20..50 => y = x/2 + 10
    for x in 20..50 {
        ox.push(x as f64);
        oy.push(x as f64 / 2.0 + 10.0);
    }
    // right corridor 4
    for y in 20..50 {
        ox.push(20.0);
        oy.push(y as f64);
    }

    // Add random noise
    let mut rng = thread_rng();
    let uni = Uniform::new(0.0, 1.0);
    for i in 0..ox.len() {
        ox[i] += uni.sample(&mut rng);
        oy[i] += uni.sample(&mut rng);
    }
    (ox, oy)
}

/// Generate (x_vals, y_vals) describing an ellipse for the given mean and covariance.
/// This is analogous to Python's plot_covariance_ellipse.
fn covariance_ellipse_points(
    mean_x: f64,
    mean_y: f64,
    cov: &Matrix2<f64>,
    n_points: usize,
) -> (Vec<f64>, Vec<f64>) {
    // Eigen-decomposition
    let sym = SymmetricEigen::new(*cov);
    let vals = sym.eigenvalues;
    let vecs = sym.eigenvectors;

    // If eigenvalues are tiny or negative, clamp them to a small positive number to avoid NaNs.
    let lambda1 = vals[0].max(1e-6);
    let lambda2 = vals[1].max(1e-6);

    // 2D ellipse with radius ~ sqrt(eigenvalues)
    // Typically, a 2-sigma ellipse would multiply sqrt by 2 or so.
    // Here we just do 1-sigma for demonstration.
    let r1 = lambda1.sqrt();
    let r2 = lambda2.sqrt();

    // We’ll parametric-sample around the ellipse in principal-axis space,
    // then rotate by eigenvectors, and finally shift by the mean.
    let mut xs = Vec::with_capacity(n_points);
    let mut ys = Vec::with_capacity(n_points);

    for i in 0..n_points {
        let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n_points as f64);
        // ellipse in local coords
        let px = r1 * theta.cos();
        let py = r2 * theta.sin();

        // rotate by eigenvectors
        // The columns of `vecs` are eigenvectors in nalgebra’s convention.
        let rotated = vecs * Vector2::new(px, py);

        xs.push(mean_x + rotated[0]);
        ys.push(mean_y + rotated[1]);
    }

    (xs, ys)
}

fn main() {
    println!("NDT example in Rust!");

    let (ox, oy) = create_dummy_observation_data();
    let grid_resolution = 10.0;
    let ndt_map = NDTMap::new(&ox, &oy, grid_resolution);

    // Prepare data for plotting.
    let mut fg = Figure::new();
    {
        // 1) plot raw data in red (dots)
        let mut axes = fg.axes2d();
        // axes.set_aspect_ratio(1.0); // similar to plt.axis("equal")
        axes.points(
            &ox,
            &oy,
            &[
                Caption("Raw observation"),
                Color("red"),
                PointSymbol('O'),
            ],
        );

        // 2) For each grid cell cluster, plot them in blue "x"
        //    We stored ndt_map.grid_index_map => index -> Vec<usize>
        //    We'll gather them in a separate pass so they can be plotted group by group.
        for (_grid_idx, point_inds) in &ndt_map.grid_index_map {
            if point_inds.is_empty() {
                continue;
            }
            let cx: Vec<f64> = point_inds.iter().map(|&i| ox[i]).collect();
            let cy: Vec<f64> = point_inds.iter().map(|&i| oy[i]).collect();
            axes.points(
                &cx,
                &cy,
                &[
                    Color("blue"),
                    PointSymbol('x'),
                ],
            );
        }

        // 3) plot the covariance ellipses for each valid NDT cell
        for ndt in &ndt_map.grid_map.data {
            if ndt.n_points > 0 {
                let (ellipse_x, ellipse_y) = covariance_ellipse_points(
                    ndt.mean_x,
                    ndt.mean_y,
                    &ndt.covariance,
                    50, // resolution of the ellipse
                );
                axes.lines(&ellipse_x, &ellipse_y, &[Color("black")]);
            }
        }
    }

    fg.show().unwrap();
}
