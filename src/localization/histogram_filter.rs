// Histogram Filter 2D Localization
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109)
//         Rust port

use nalgebra::{DMatrix, Vector2, Vector4};
use rand_distr::{Distribution, Normal};
use gnuplot::{Figure, Caption, Color, AxesCommon};

// Simulation parameters
const DT: f64 = 0.1; // time step [s]
const SIM_TIME: f64 = 50.0; // simulation time [s]
const MAX_RANGE: f64 = 10.0; // maximum observation range [m]
const MOTION_STD: f64 = 1.0; // standard deviation of motion model
const RANGE_STD: f64 = 3.0; // standard deviation of observation model
const XY_RESOLUTION: f64 = 0.5; // grid resolution [m]

const SHOW_ANIMATION: bool = false;

/// Grid map for histogram filter
struct GridMap {
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
    fn new(min_x: f64, min_y: f64, max_x: f64, max_y: f64, resolution: f64) -> Self {
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

    fn get_index(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let ix = ((x - self.min_x) / self.resolution).floor() as i32;
        let iy = ((y - self.min_y) / self.resolution).floor() as i32;

        if ix >= 0 && ix < self.x_width as i32 && iy >= 0 && iy < self.y_width as i32 {
            Some((ix as usize, iy as usize))
        } else {
            None
        }
    }

    fn get_xy(&self, ix: usize, iy: usize) -> (f64, f64) {
        let x = self.min_x + ix as f64 * self.resolution + self.resolution / 2.0;
        let y = self.min_y + iy as f64 * self.resolution + self.resolution / 2.0;
        (x, y)
    }

    fn normalize(&mut self) {
        let sum: f64 = self.data.iter().sum();
        if sum > 0.0 {
            self.data /= sum;
        }
    }

    fn get_max_prob_position(&self) -> (f64, f64) {
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
    fn get_mean_position(&self) -> (f64, f64) {
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
struct HistogramFilter {
    grid_map: GridMap,
    dx: f64, // accumulated x displacement
    dy: f64, // accumulated y displacement
}

impl HistogramFilter {
    fn new(min_x: f64, min_y: f64, max_x: f64, max_y: f64, resolution: f64) -> Self {
        HistogramFilter {
            grid_map: GridMap::new(min_x, min_y, max_x, max_y, resolution),
            dx: 0.0,
            dy: 0.0,
        }
    }

    /// Create histogram filter with initial position
    fn new_with_initial_pos(min_x: f64, min_y: f64, max_x: f64, max_y: f64, resolution: f64,
                             init_x: f64, init_y: f64, init_std: f64) -> Self {
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
    fn motion_update(&mut self, u: Vector2<f64>, yaw: f64, dt: f64) {
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

                if nix >= 0 && nix < x_width as i32 &&
                   niy >= 0 && niy < y_width as i32 {
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
                            sum += self.grid_map.data[(ni as usize, nj as usize)] * kernel[(ki, kj)];
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
    fn observation_update(&mut self, z: &[(f64, f64, f64)], rfid: &[(f64, f64)]) {
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

    fn get_estimated_position(&self) -> (f64, f64) {
        self.grid_map.get_mean_position()
    }

    /// Get probability distribution data for heatmap visualization
    /// Returns (x_coords, y_coords, probability_values) for each grid cell
    fn get_heatmap_data(&self) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
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
    fn get_grid_dims(&self) -> (usize, usize) {
        (self.grid_map.x_width, self.grid_map.y_width)
    }

    /// Get min coordinates
    fn get_min_coords(&self) -> (f64, f64) {
        (self.grid_map.min_x, self.grid_map.min_y)
    }

    /// Get resolution
    fn get_resolution(&self) -> f64 {
        self.grid_map.resolution
    }

    /// Get probability data as 2D matrix for heatmap (row-major, y then x)
    fn get_prob_matrix(&self) -> Vec<f64> {
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
fn motion_model(x: Vector4<f64>, u: Vector2<f64>, dt: f64) -> Vector4<f64> {
    let yaw = x[2];
    Vector4::new(
        x[0] + u[0] * dt * yaw.cos(),
        x[1] + u[0] * dt * yaw.sin(),
        x[2] + u[1] * dt,
        u[0],
    )
}

/// Get observations from RFID landmarks
fn get_observations(x_true: &Vector4<f64>, rfid: &[(f64, f64)], normal: &Normal<f64>) -> Vec<(f64, f64, f64)> {
    let mut z = Vec::new();

    for (lx, ly) in rfid {
        let d = ((x_true[0] - lx).powi(2) + (x_true[1] - ly).powi(2)).sqrt();

        if d <= MAX_RANGE {
            // Add noise to measurement
            let d_noisy = d + normal.sample(&mut rand::thread_rng()) * RANGE_STD * 0.1;
            z.push((d_noisy, *lx, *ly));
        }
    }

    z
}

fn main() {
    println!("Histogram Filter 2D Localization start!");

    // RFID landmark positions [x, y]
    let rfid: Vec<(f64, f64)> = vec![
        (10.0, 0.0),
        (10.0, 10.0),
        (0.0, 15.0),
        (-5.0, 20.0),
    ];

    // Grid area
    let area = (-5.0, -5.0, 15.0, 25.0); // (min_x, min_y, max_x, max_y)

    // Initialize histogram filter with initial position at origin
    let mut hf = HistogramFilter::new_with_initial_pos(
        area.0, area.1, area.2, area.3, XY_RESOLUTION,
        0.0, 0.0, 1.0  // initial position (x, y) and initial standard deviation
    );

    // State: [x, y, yaw, v]
    let mut x_true = Vector4::new(0.0, 0.0, 0.0, 0.0);
    let mut x_dr = Vector4::new(0.0, 0.0, 0.0, 0.0); // Dead reckoning

    // Control input: [v, yaw_rate]
    let u = Vector2::new(1.0, 0.1);

    // Noise
    let normal = Normal::new(0.0, 1.0).unwrap();
    let input_noise_std = 0.5;

    // History for plotting
    let mut h_true: Vec<(f64, f64)> = vec![(0.0, 0.0)];
    let mut h_dr: Vec<(f64, f64)> = vec![(0.0, 0.0)];

    let mut time = 0.0;
    let mut fig = Figure::new();

    while time <= SIM_TIME {
        time += DT;

        // True state update
        x_true = motion_model(x_true, u, DT);

        // Dead reckoning with noise
        let u_noisy = Vector2::new(
            u[0] + normal.sample(&mut rand::thread_rng()) * input_noise_std,
            u[1] + normal.sample(&mut rand::thread_rng()) * input_noise_std * 0.1,
        );
        x_dr = motion_model(x_dr, u_noisy, DT);

        // Get observations
        let z = get_observations(&x_true, &rfid, &normal);

        // Histogram filter update
        hf.motion_update(u_noisy, x_dr[2], DT);

        if !z.is_empty() {
            hf.observation_update(&z, &rfid);
        }

        // Store history
        h_true.push((x_true[0], x_true[1]));
        h_dr.push((x_dr[0], x_dr[1]));

        // Animation
        if SHOW_ANIMATION {
            fig.clear_axes();

            let true_x: Vec<f64> = h_true.iter().map(|p| p.0).collect();
            let true_y: Vec<f64> = h_true.iter().map(|p| p.1).collect();
            let dr_x: Vec<f64> = h_dr.iter().map(|p| p.0).collect();
            let dr_y: Vec<f64> = h_dr.iter().map(|p| p.1).collect();
            let rfid_x: Vec<f64> = rfid.iter().map(|p| p.0).collect();
            let rfid_y: Vec<f64> = rfid.iter().map(|p| p.1).collect();

            fig.axes2d()
                .set_title("Histogram Filter Localization", &[])
                .set_x_label("x [m]", &[])
                .set_y_label("y [m]", &[])
                .set_x_range(gnuplot::Fix(area.0 - 2.0), gnuplot::Fix(area.2 + 2.0))
                .set_y_range(gnuplot::Fix(area.1 - 2.0), gnuplot::Fix(area.3 + 2.0))
                .points(&rfid_x, &rfid_y, &[Caption("RFID"), Color("black"), gnuplot::PointSymbol('O'), gnuplot::PointSize(2.0)])
                .lines(&true_x, &true_y, &[Caption("True"), Color("blue")])
                .lines(&dr_x, &dr_y, &[Caption("Dead Reckoning"), Color("yellow")]);

            fig.show_and_keep_running().unwrap();
        }
    }

    println!("Done!");

    // Save final plot with heatmap
    fig.clear_axes();

    let true_x: Vec<f64> = h_true.iter().map(|p| p.0).collect();
    let true_y: Vec<f64> = h_true.iter().map(|p| p.1).collect();
    let dr_x: Vec<f64> = h_dr.iter().map(|p| p.0).collect();
    let dr_y: Vec<f64> = h_dr.iter().map(|p| p.1).collect();
    let rfid_x: Vec<f64> = rfid.iter().map(|p| p.0).collect();
    let rfid_y: Vec<f64> = rfid.iter().map(|p| p.1).collect();

    // Get heatmap data from histogram filter
    let (x_width, y_width) = hf.get_grid_dims();
    let (min_x, min_y) = hf.get_min_coords();
    let resolution = hf.get_resolution();
    let heatmap_data = hf.get_prob_matrix();

    fig.axes2d()
        .set_title("Histogram Filter Localization", &[])
        .set_x_label("x [m]", &[])
        .set_y_label("y [m]", &[])
        .set_x_range(gnuplot::Fix(area.0 - 2.0), gnuplot::Fix(area.2 + 2.0))
        .set_y_range(gnuplot::Fix(area.1 - 2.0), gnuplot::Fix(area.3 + 2.0))
        .image(
            heatmap_data.iter(),
            x_width,
            y_width,
            Some((min_x, min_y, min_x + x_width as f64 * resolution, min_y + y_width as f64 * resolution)),
            &[],
        )
        .points(&rfid_x, &rfid_y, &[Caption("RFID"), Color("black"), gnuplot::PointSymbol('O'), gnuplot::PointSize(2.0)])
        .lines(&true_x, &true_y, &[Caption("True"), Color("blue")])
        .lines(&dr_x, &dr_y, &[Caption("Dead Reckoning"), Color("orange")]);

    fig.save_to_svg("./img/localization/histogram_filter.svg", 640, 480).unwrap();
    println!("Plot saved to ./img/localization/histogram_filter.svg");
}
