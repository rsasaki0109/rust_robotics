//! Normal Distribution Transform (NDT) mapping algorithm.

use std::collections::HashMap;

use nalgebra::{Matrix2, SymmetricEigen, Vector2};
use rand::prelude::*;
use rand_distr::{Distribution, Uniform};

/// A single cell in the NDT grid, storing distribution parameters.
#[derive(Debug, Clone)]
pub struct NDTGrid {
    pub n_points: usize,
    pub mean_x: f64,
    pub mean_y: f64,
    pub center_grid_x: f64,
    pub center_grid_y: f64,
    pub covariance: Matrix2<f64>,
    pub eig_values: Vector2<f64>,
    pub eig_vec: Matrix2<f64>,
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
pub struct GridMap {
    pub width: usize,
    pub height: usize,
    pub resolution: f64,
    pub center_x: f64,
    pub center_y: f64,
    pub data: Vec<NDTGrid>,
}

impl GridMap {
    pub fn new(width: usize, height: usize, resolution: f64, center_x: f64, center_y: f64) -> Self {
        let data = vec![NDTGrid::default(); width * height];
        GridMap {
            width,
            height,
            resolution,
            center_x,
            center_y,
            data,
        }
    }

    /// Convert (x, y) in world coordinates to a single index in the grid.
    /// Returns `None` if out of bounds.
    pub fn calc_grid_index_from_xy_pos(&self, x: f64, y: f64) -> Option<usize> {
        let gx = (x - self.center_x) / self.resolution + (self.width as f64) / 2.0;
        let gy = (y - self.center_y) / self.resolution + (self.height as f64) / 2.0;

        let ix = gx.floor() as isize;
        let iy = gy.floor() as isize;

        if ix < 0 || iy < 0 || ix >= self.width as isize || iy >= self.height as isize {
            return None;
        }
        let idx = (iy as usize) * self.width + (ix as usize);
        Some(idx)
    }

    /// Convert a single index to the center-world-coordinates of that grid cell.
    pub fn calc_grid_central_xy_position_from_grid_index(&self, idx: usize) -> (f64, f64) {
        let row = idx / self.width;
        let col = idx % self.width;

        let cx = (col as f64 + 0.5) - (self.width as f64) / 2.0;
        let cy = (row as f64 + 0.5) - (self.height as f64) / 2.0;
        let x = self.center_x + cx * self.resolution;
        let y = self.center_y + cy * self.resolution;
        (x, y)
    }
}

/// Normal Distribution Transform (NDT) Map
pub struct NDTMap {
    pub min_n_points: usize,
    pub resolution: f64,
    pub grid_map: GridMap,
    pub grid_index_map: HashMap<usize, Vec<usize>>,
    pub ox: Vec<f64>,
    pub oy: Vec<f64>,
}

impl NDTMap {
    pub fn new(ox: &[f64], oy: &[f64], resolution: f64) -> Self {
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
            let mut ndt = NDTGrid {
                n_points: inds.len(),
                ..NDTGrid::default()
            };
            if ndt.n_points >= self.min_n_points {
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

                let (cx, cy) = self
                    .grid_map
                    .calc_grid_central_xy_position_from_grid_index(*grid_index);
                ndt.center_grid_x = cx;
                ndt.center_grid_y = cy;

                let mut xs = Vec::with_capacity(inds.len());
                let mut ys = Vec::with_capacity(inds.len());
                for &i in inds {
                    xs.push(self.ox[i]);
                    ys.push(self.oy[i]);
                }
                let cov = compute_covariance_2d(&xs, &ys, mean_x, mean_y);
                ndt.covariance = cov;

                let sym_eig = SymmetricEigen::new(cov);
                ndt.eig_values = sym_eig.eigenvalues;
                ndt.eig_vec = sym_eig.eigenvectors;

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
pub fn compute_covariance_2d(xs: &[f64], ys: &[f64], mean_x: f64, mean_y: f64) -> Matrix2<f64> {
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
    let denom = n - 1.0;
    Matrix2::new(sxx / denom, sxy / denom, sxy / denom, syy / denom)
}

/// Generates dummy corridor-like data with random noise.
pub fn create_dummy_observation_data() -> (Vec<f64>, Vec<f64>) {
    let mut ox = Vec::new();
    let mut oy = Vec::new();

    for y in -50..50 {
        ox.push(-20.0);
        oy.push(y as f64);
    }
    for y in -50..0 {
        ox.push(20.0);
        oy.push(y as f64);
    }
    for x in 20..50 {
        ox.push(x as f64);
        oy.push(0.0);
    }
    for x in 20..50 {
        ox.push(x as f64);
        oy.push(x as f64 / 2.0 + 10.0);
    }
    for y in 20..50 {
        ox.push(20.0);
        oy.push(y as f64);
    }

    let mut rng = thread_rng();
    let uni = Uniform::new(0.0, 1.0);
    for i in 0..ox.len() {
        ox[i] += uni.sample(&mut rng);
        oy[i] += uni.sample(&mut rng);
    }
    (ox, oy)
}

/// Generate (x_vals, y_vals) describing an ellipse for the given mean and covariance.
pub fn covariance_ellipse_points(
    mean_x: f64,
    mean_y: f64,
    cov: &Matrix2<f64>,
    n_points: usize,
) -> (Vec<f64>, Vec<f64>) {
    let sym = SymmetricEigen::new(*cov);
    let vals = sym.eigenvalues;
    let vecs = sym.eigenvectors;

    let lambda1 = vals[0].max(1e-6);
    let lambda2 = vals[1].max(1e-6);

    let r1 = lambda1.sqrt();
    let r2 = lambda2.sqrt();

    let mut xs = Vec::with_capacity(n_points);
    let mut ys = Vec::with_capacity(n_points);

    for i in 0..n_points {
        let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n_points as f64);
        let px = r1 * theta.cos();
        let py = r2 * theta.sin();

        let rotated = vecs * Vector2::new(px, py);

        xs.push(mean_x + rotated[0]);
        ys.push(mean_y + rotated[1]);
    }

    (xs, ys)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ndt_map_creation() {
        let (ox, oy) = create_dummy_observation_data();
        let ndt_map = NDTMap::new(&ox, &oy, 10.0);
        assert!(!ndt_map.grid_index_map.is_empty());
        assert!(ndt_map.grid_map.data.len() > 0);
    }

    #[test]
    fn test_covariance_2d() {
        let xs = vec![1.0, 2.0, 3.0];
        let ys = vec![4.0, 5.0, 6.0];
        let cov = compute_covariance_2d(&xs, &ys, 2.0, 5.0);
        assert!((cov[(0, 0)] - 1.0).abs() < 1e-10);
        assert!((cov[(1, 1)] - 1.0).abs() < 1e-10);
        assert!((cov[(0, 1)] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_covariance_ellipse_points() {
        let cov = Matrix2::new(1.0, 0.0, 0.0, 1.0);
        let (xs, ys) = covariance_ellipse_points(0.0, 0.0, &cov, 100);
        assert_eq!(xs.len(), 100);
        assert_eq!(ys.len(), 100);
    }
}
