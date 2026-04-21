#![allow(dead_code, clippy::too_many_arguments)]

//! Histogram Filter 2D Localization
//!
//! Implements grid-based Bayesian filtering for 2D robot localization
//! using a probability distribution over a discretized grid.

use nalgebra::{DMatrix, Vector2, Vector4};
use rand_distr::{Distribution, Normal};

// Simulation parameters
const DT: f64 = 0.1; // time step [s]
const MAX_RANGE: f64 = 10.0; // maximum observation range [m]
const MOTION_STD: f64 = 1.0; // standard deviation of motion model
const RANGE_STD: f64 = 3.0; // standard deviation of observation model
const XY_RESOLUTION: f64 = 0.5; // grid resolution [m]

/// Grid map for histogram filter
pub struct GridMap {
    data: DMatrix<f64>,
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
    resolution: f64,
    x_width: usize,
    y_width: usize,
}

impl GridMap {
    pub fn new(min_x: f64, min_y: f64, max_x: f64, max_y: f64, resolution: f64) -> Self {
        let x_width = ((max_x - min_x) / resolution).round() as usize;
        let y_width = ((max_y - min_y) / resolution).round() as usize;
        let data = DMatrix::from_element(x_width, y_width, 1.0 / (x_width * y_width) as f64);

        GridMap {
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

    pub fn get_index(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let ix = ((x - self.min_x) / self.resolution).floor() as i32;
        let iy = ((y - self.min_y) / self.resolution).floor() as i32;

        if ix >= 0 && ix < self.x_width as i32 && iy >= 0 && iy < self.y_width as i32 {
            Some((ix as usize, iy as usize))
        } else {
            None
        }
    }

    pub fn get_xy(&self, ix: usize, iy: usize) -> (f64, f64) {
        let x = self.min_x + ix as f64 * self.resolution + self.resolution / 2.0;
        let y = self.min_y + iy as f64 * self.resolution + self.resolution / 2.0;
        (x, y)
    }

    pub fn normalize(&mut self) {
        let sum: f64 = self.data.iter().sum();
        if sum > 0.0 {
            self.data /= sum;
        }
    }

    pub fn get_max_prob_position(&self) -> (f64, f64) {
        let mut max_prob = 0.0;
        let mut max_ix = 0;
        let mut max_iy = 0;

        for ix in 0..self.x_width {
            for iy in 0..self.y_width {
                if self.data[(ix, iy)] > max_prob {
                    max_prob = self.data[(ix, iy)];
                    max_ix = ix;
                    max_iy = iy;
                }
            }
        }

        self.get_xy(max_ix, max_iy)
    }

    /// Get weighted mean position (more stable than max probability)
    pub fn get_mean_position(&self) -> (f64, f64) {
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_w = 0.0;

        for ix in 0..self.x_width {
            for iy in 0..self.y_width {
                let w = self.data[(ix, iy)];
                let (x, y) = self.get_xy(ix, iy);
                sum_x += x * w;
                sum_y += y * w;
                sum_w += w;
            }
        }

        if sum_w > 0.0 {
            (sum_x / sum_w, sum_y / sum_w)
        } else {
            (0.0, 0.0)
        }
    }
}

/// Histogram filter for 2D localization
pub struct HistogramFilter {
    grid_map: GridMap,
    dx: f64, // accumulated x displacement
    dy: f64, // accumulated y displacement
}

impl HistogramFilter {
    pub fn new(min_x: f64, min_y: f64, max_x: f64, max_y: f64, resolution: f64) -> Self {
        HistogramFilter {
            grid_map: GridMap::new(min_x, min_y, max_x, max_y, resolution),
            dx: 0.0,
            dy: 0.0,
        }
    }

    /// Create histogram filter with initial position
    pub fn new_with_initial_pos(
        min_x: f64,
        min_y: f64,
        max_x: f64,
        max_y: f64,
        resolution: f64,
        init_x: f64,
        init_y: f64,
        init_std: f64,
    ) -> Self {
        let mut hf = HistogramFilter::new(min_x, min_y, max_x, max_y, resolution);

        // Set Gaussian distribution around initial position
        for ix in 0..hf.grid_map.x_width {
            for iy in 0..hf.grid_map.y_width {
                let (gx, gy) = hf.grid_map.get_xy(ix, iy);
                let dx = gx - init_x;
                let dy = gy - init_y;
                let prob = (-(dx * dx + dy * dy) / (2.0 * init_std * init_std)).exp();
                hf.grid_map.data[(ix, iy)] = prob;
            }
        }
        hf.grid_map.normalize();

        hf
    }

    /// Motion update: shift probability grid based on movement
    pub fn motion_update(&mut self, u: Vector2<f64>, yaw: f64, dt: f64) {
        // Accumulate displacement
        self.dx += u[0] * dt * yaw.cos();
        self.dy += u[0] * dt * yaw.sin();

        // Shift grid when displacement exceeds resolution
        let shift_x = (self.dx / self.grid_map.resolution).floor() as i32;
        let shift_y = (self.dy / self.grid_map.resolution).floor() as i32;

        if shift_x != 0 || shift_y != 0 {
            self.dx -= shift_x as f64 * self.grid_map.resolution;
            self.dy -= shift_y as f64 * self.grid_map.resolution;

            self.shift_grid(shift_x, shift_y);
        }

        // Apply Gaussian blur for motion uncertainty
        self.gaussian_blur();
    }

    /// Shift the probability grid
    /// Following Python's map_shift: grid_map.data[ix + x_shift][iy + y_shift] = tmp_grid_map[ix][iy]
    fn shift_grid(&mut self, shift_x: i32, shift_y: i32) {
        let x_width = self.grid_map.x_width;
        let y_width = self.grid_map.y_width;
        let old_data = self.grid_map.data.clone();
        let mut new_data = DMatrix::from_element(x_width, y_width, 0.0);

        for ix in 0..x_width {
            for iy in 0..y_width {
                let nix = ix as i32 + shift_x;
                let niy = iy as i32 + shift_y;

                if nix >= 0 && nix < x_width as i32 && niy >= 0 && niy < y_width as i32 {
                    new_data[(nix as usize, niy as usize)] = old_data[(ix, iy)];
                }
            }
        }

        self.grid_map.data = new_data;
        self.grid_map.normalize();
    }

    /// Apply Gaussian blur to represent motion uncertainty
    fn gaussian_blur(&mut self) {
        let kernel_size = 3;
        let sigma = MOTION_STD / self.grid_map.resolution;

        // Create Gaussian kernel
        let mut kernel = DMatrix::from_element(kernel_size, kernel_size, 0.0);
        let center = kernel_size as i32 / 2;
        let mut kernel_sum = 0.0;

        for i in 0..kernel_size {
            for j in 0..kernel_size {
                let x = i as i32 - center;
                let y = j as i32 - center;
                let value = (-(x * x + y * y) as f64 / (2.0 * sigma * sigma)).exp();
                kernel[(i, j)] = value;
                kernel_sum += value;
            }
        }
        kernel /= kernel_sum;

        // Apply convolution
        let x_width = self.grid_map.x_width;
        let y_width = self.grid_map.y_width;
        let mut new_data = DMatrix::from_element(x_width, y_width, 0.0);

        for ix in 0..x_width {
            for iy in 0..y_width {
                let mut sum = 0.0;
                for ki in 0..kernel_size {
                    for kj in 0..kernel_size {
                        let ni = ix as i32 + ki as i32 - center;
                        let nj = iy as i32 + kj as i32 - center;

                        if ni >= 0 && ni < x_width as i32 && nj >= 0 && nj < y_width as i32 {
                            sum +=
                                self.grid_map.data[(ni as usize, nj as usize)] * kernel[(ki, kj)];
                        }
                    }
                }
                new_data[(ix, iy)] = sum;
            }
        }

        self.grid_map.data = new_data;
        self.grid_map.normalize();
    }

    /// Observation update: multiply grid probabilities by observation likelihoods
    pub fn observation_update(&mut self, z: &[(f64, f64, f64)], _rfid: &[(f64, f64)]) {
        for (z_d, z_id_x, z_id_y) in z {
            for ix in 0..self.grid_map.x_width {
                for iy in 0..self.grid_map.y_width {
                    let (gx, gy) = self.grid_map.get_xy(ix, iy);

                    // Calculate predicted distance to landmark
                    let pred_d = ((gx - z_id_x).powi(2) + (gy - z_id_y).powi(2)).sqrt();

                    // Calculate likelihood using Gaussian
                    let diff = pred_d - z_d;
                    let likelihood = (-diff.powi(2) / (2.0 * RANGE_STD.powi(2))).exp();

                    self.grid_map.data[(ix, iy)] *= likelihood;
                }
            }
        }

        self.grid_map.normalize();
    }

    pub fn get_estimated_position(&self) -> (f64, f64) {
        self.grid_map.get_mean_position()
    }

    /// Get probability distribution data for heatmap visualization
    /// Returns (x_coords, y_coords, probability_values) for each grid cell
    pub fn get_heatmap_data(&self) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let mut xs = Vec::new();
        let mut ys = Vec::new();
        let mut probs = Vec::new();

        for ix in 0..self.grid_map.x_width {
            for iy in 0..self.grid_map.y_width {
                let (x, y) = self.grid_map.get_xy(ix, iy);
                xs.push(x);
                ys.push(y);
                probs.push(self.grid_map.data[(ix, iy)]);
            }
        }

        (xs, ys, probs)
    }

    /// Get grid dimensions for heatmap
    pub fn get_grid_dims(&self) -> (usize, usize) {
        (self.grid_map.x_width, self.grid_map.y_width)
    }

    /// Get min coordinates
    pub fn get_min_coords(&self) -> (f64, f64) {
        (self.grid_map.min_x, self.grid_map.min_y)
    }

    /// Get resolution
    pub fn get_resolution(&self) -> f64 {
        self.grid_map.resolution
    }

    /// Get probability data as 2D matrix for heatmap (row-major, y then x)
    pub fn get_prob_matrix(&self) -> Vec<f64> {
        let mut data = Vec::new();
        for iy in 0..self.grid_map.y_width {
            for ix in 0..self.grid_map.x_width {
                data.push(self.grid_map.data[(ix, iy)]);
            }
        }
        data
    }
}

/// Motion model
pub fn motion_model(x: Vector4<f64>, u: Vector2<f64>, dt: f64) -> Vector4<f64> {
    let yaw = x[2];
    Vector4::new(
        x[0] + u[0] * dt * yaw.cos(),
        x[1] + u[0] * dt * yaw.sin(),
        x[2] + u[1] * dt,
        u[0],
    )
}

/// Get observations from RFID landmarks
pub fn get_observations(
    x_true: &Vector4<f64>,
    rfid: &[(f64, f64)],
    normal: &Normal<f64>,
) -> Vec<(f64, f64, f64)> {
    let mut z = Vec::new();

    for (lx, ly) in rfid {
        let d = ((x_true[0] - lx).powi(2) + (x_true[1] - ly).powi(2)).sqrt();

        if d <= MAX_RANGE {
            // Add noise to measurement
            let d_noisy = d + normal.sample(&mut rand::rng()) * RANGE_STD * 0.1;
            z.push((d_noisy, *lx, *ly));
        }
    }

    z
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_grid_map_creation() {
        let gm = GridMap::new(-5.0, -5.0, 5.0, 5.0, 1.0);
        assert_eq!(gm.x_width, 10);
        assert_eq!(gm.y_width, 10);
        // Uniform distribution should sum to 1
        let sum: f64 = gm.data.iter().sum();
        assert!((sum - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_grid_map_index() {
        let gm = GridMap::new(-5.0, -5.0, 5.0, 5.0, 1.0);
        assert_eq!(gm.get_index(0.0, 0.0), Some((5, 5)));
        assert_eq!(gm.get_index(-6.0, 0.0), None);
    }

    #[test]
    fn test_histogram_filter_creation() {
        let hf = HistogramFilter::new(-5.0, -5.0, 5.0, 5.0, 1.0);
        let (x, y) = hf.get_estimated_position();
        assert!(x.is_finite());
        assert!(y.is_finite());
    }

    #[test]
    fn test_histogram_filter_with_initial_pos() {
        let hf = HistogramFilter::new_with_initial_pos(-5.0, -5.0, 5.0, 5.0, 1.0, 0.0, 0.0, 1.0);
        let (x, y) = hf.get_estimated_position();
        // Mean position should be near the initial position
        assert!((x - 0.0).abs() < 1.0);
        assert!((y - 0.0).abs() < 1.0);
    }

    #[test]
    fn test_histogram_filter_observation_update() {
        let mut hf =
            HistogramFilter::new_with_initial_pos(-10.0, -10.0, 10.0, 10.0, 0.5, 0.0, 0.0, 2.0);
        // Observe distance 5.0 to landmark at (5, 0)
        let observations = vec![(5.0, 5.0, 0.0)];
        hf.observation_update(&observations, &[(5.0, 0.0)]);
        let (x, _y) = hf.get_estimated_position();
        // After observation, estimate should still be finite
        assert!(x.is_finite());
    }

    #[test]
    fn test_histogram_filter_motion_update() {
        let mut hf =
            HistogramFilter::new_with_initial_pos(-10.0, -10.0, 10.0, 10.0, 0.5, 0.0, 0.0, 1.0);
        let u = Vector2::new(1.0, 0.0);
        hf.motion_update(u, 0.0, 0.1);
        let (x, y) = hf.get_estimated_position();
        assert!(x.is_finite());
        assert!(y.is_finite());
    }

    #[test]
    fn test_motion_model() {
        let x = Vector4::new(0.0, 0.0, 0.0, 0.0);
        let u = Vector2::new(1.0, 0.1);
        let x_next = motion_model(x, u, 0.1);
        // Moving forward with yaw=0, x should increase
        assert!(x_next[0] > 0.0);
        assert_eq!(x_next[3], 1.0);
    }
}
