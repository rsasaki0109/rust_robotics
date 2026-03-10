#![allow(dead_code, clippy::too_many_arguments)]

// Histogram Filter 2D Localization
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109)
//         Rust port

use gnuplot::{AxesCommon, Caption, Color, Figure};
use nalgebra::{DMatrix, Vector2, Vector4};
use rand_distr::{Distribution, Normal};
use std::fs::File;
use std::io::Write;

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
    fn new_with_initial_pos(
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
    fn observation_update(&mut self, z: &[(f64, f64, f64)], _rfid: &[(f64, f64)]) {
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
fn get_observations(
    x_true: &Vector4<f64>,
    rfid: &[(f64, f64)],
    normal: &Normal<f64>,
) -> Vec<(f64, f64, f64)> {
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

fn lerp_rgb(start: (u8, u8, u8), end: (u8, u8, u8), t: f64) -> (u8, u8, u8) {
    let t = t.clamp(0.0, 1.0);
    let blend = |a: u8, b: u8| -> u8 { (a as f64 + (b as f64 - a as f64) * t).round() as u8 };
    (
        blend(start.0, end.0),
        blend(start.1, end.1),
        blend(start.2, end.2),
    )
}

fn heatmap_style(probability: f64, max_probability: f64) -> (String, f64) {
    if max_probability <= 0.0 || probability <= 0.0 {
        return ("#fff7ed".to_string(), 0.0);
    }

    let t = (probability / max_probability).sqrt().clamp(0.0, 1.0);
    let rgb = if t < 0.5 {
        lerp_rgb((255, 247, 237), (254, 240, 138), t * 2.0)
    } else {
        lerp_rgb((254, 240, 138), (185, 28, 28), (t - 0.5) * 2.0)
    };

    (
        format!("#{:02x}{:02x}{:02x}", rgb.0, rgb.1, rgb.2),
        0.15 + 0.8 * t,
    )
}

fn path_data(
    points: &[(f64, f64)],
    to_svg_x: &impl Fn(f64) -> f64,
    to_svg_y: &impl Fn(f64) -> f64,
) -> String {
    let mut path = String::new();

    for (i, (x, y)) in points.iter().enumerate() {
        let sx = to_svg_x(*x);
        let sy = to_svg_y(*y);
        if i == 0 {
            path.push_str(&format!("M {:.2},{:.2}", sx, sy));
        } else {
            path.push_str(&format!(" L {:.2},{:.2}", sx, sy));
        }
    }

    path
}

fn build_histogram_filter_svg(
    area: (f64, f64, f64, f64),
    rfid: &[(f64, f64)],
    history_true: &[(f64, f64)],
    history_dr: &[(f64, f64)],
    history_est: &[(f64, f64)],
    x_width: usize,
    y_width: usize,
    min_x: f64,
    min_y: f64,
    resolution: f64,
    heatmap_data: &[f64],
) -> String {
    let width = 920.0;
    let height = 700.0;
    let margin = 72.0;
    let side_panel_width = 220.0;
    let available_plot_width = width - 2.0 * margin - side_panel_width;
    let available_plot_height = height - 2.0 * margin;
    let x_range = area.2 - area.0;
    let y_range = area.3 - area.1;
    let scale = (available_plot_width / x_range)
        .min(available_plot_height / y_range)
        .max(1.0);
    let plot_width = x_range * scale;
    let plot_height = y_range * scale;
    let plot_left = margin;
    let plot_top = margin + (available_plot_height - plot_height) / 2.0;

    let to_svg_x = |x: f64| plot_left + (x - area.0) * scale;
    let to_svg_y = |y: f64| plot_top + plot_height - (y - area.1) * scale;

    let max_probability = heatmap_data.iter().copied().fold(0.0, f64::max);
    let legend_x = plot_left + plot_width + 36.0;
    let legend_y = plot_top + 24.0;
    let colorbar_x = legend_x + 18.0;
    let colorbar_y = legend_y + 168.0;
    let colorbar_height = 240.0;
    let colorbar_width = 26.0;

    let mut svg = String::new();
    svg.push_str(&format!(
        "<?xml version='1.0' encoding='UTF-8'?>\n<svg xmlns='http://www.w3.org/2000/svg' width='{width}' height='{height}' viewBox='0 0 {width} {height}'>\n"
    ));
    svg.push_str("<rect width='100%' height='100%' fill='white'/>\n");
    svg.push_str(&format!(
        "<text x='{:.1}' y='38' text-anchor='middle' font-size='24' font-family='sans-serif' font-weight='700' fill='#111827'>Histogram Filter Localization</text>\n",
        width / 2.0
    ));
    svg.push_str(&format!(
        "<rect x='{plot_left:.1}' y='{plot_top:.1}' width='{plot_width:.1}' height='{plot_height:.1}' fill='#fffdf7' stroke='#d6d3d1' stroke-width='2'/>\n"
    ));

    let x_tick_start = (area.0 / 5.0).ceil() as i32;
    let x_tick_end = (area.2 / 5.0).floor() as i32;
    for tick in x_tick_start..=x_tick_end {
        let x = tick as f64 * 5.0;
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

    let y_tick_start = (area.1 / 5.0).ceil() as i32;
    let y_tick_end = (area.3 / 5.0).floor() as i32;
    for tick in y_tick_start..=y_tick_end {
        let y = tick as f64 * 5.0;
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

    let cell_width = resolution * scale;
    let cell_height = resolution * scale;
    for iy in 0..y_width {
        for ix in 0..x_width {
            let probability = heatmap_data[iy * x_width + ix];
            if probability <= max_probability * 0.01 {
                continue;
            }

            let cell_x = min_x + ix as f64 * resolution;
            let cell_y = min_y + iy as f64 * resolution;
            let sx = to_svg_x(cell_x);
            let sy = to_svg_y(cell_y + resolution);
            let (fill, opacity) = heatmap_style(probability, max_probability);
            svg.push_str(&format!(
                "<rect x='{sx:.2}' y='{sy:.2}' width='{cell_width:.2}' height='{cell_height:.2}' fill='{fill}' fill-opacity='{opacity:.3}' stroke='none'/>\n"
            ));
        }
    }

    let true_path = path_data(history_true, &to_svg_x, &to_svg_y);
    let dr_path = path_data(history_dr, &to_svg_x, &to_svg_y);
    let est_path = path_data(history_est, &to_svg_x, &to_svg_y);

    if !true_path.is_empty() {
        svg.push_str(&format!(
            "<path d='{true_path}' fill='none' stroke='#2563eb' stroke-width='5' stroke-linecap='round' stroke-linejoin='round'/>\n"
        ));
    }
    if !dr_path.is_empty() {
        svg.push_str(&format!(
            "<path d='{dr_path}' fill='none' stroke='#d97706' stroke-width='4' stroke-linecap='round' stroke-linejoin='round'/>\n"
        ));
    }
    if !est_path.is_empty() {
        svg.push_str(&format!(
            "<path d='{est_path}' fill='none' stroke='#16a34a' stroke-width='4' stroke-linecap='round' stroke-linejoin='round'/>\n"
        ));
    }

    for (x, y) in rfid {
        let sx = to_svg_x(*x);
        let sy = to_svg_y(*y);
        svg.push_str(&format!(
            "<circle cx='{sx:.2}' cy='{sy:.2}' r='6.5' fill='#111827' stroke='white' stroke-width='2'/>\n"
        ));
    }

    if let Some((start_x, start_y)) = history_true.first() {
        svg.push_str(&format!(
            "<circle cx='{:.2}' cy='{:.2}' r='7' fill='#1d4ed8' stroke='white' stroke-width='2.5'/>\n",
            to_svg_x(*start_x),
            to_svg_y(*start_y)
        ));
    }
    if let Some((goal_x, goal_y)) = history_true.last() {
        svg.push_str(&format!(
            "<circle cx='{:.2}' cy='{:.2}' r='7' fill='#7c3aed' stroke='white' stroke-width='2.5'/>\n",
            to_svg_x(*goal_x),
            to_svg_y(*goal_y)
        ));
    }
    if let Some((est_x, est_y)) = history_est.last() {
        svg.push_str(&format!(
            "<circle cx='{:.2}' cy='{:.2}' r='6' fill='#16a34a' stroke='white' stroke-width='2'/>\n",
            to_svg_x(*est_x),
            to_svg_y(*est_y)
        ));
    }

    svg.push_str(&format!(
        "<rect x='{legend_x:.1}' y='{legend_y:.1}' width='182' height='132' rx='18' fill='white' stroke='#d6d3d1'/>\n"
    ));
    let legend_items = [
        ("#2563eb", "True Path", false),
        ("#d97706", "Dead Reckoning", false),
        ("#16a34a", "Histogram Estimate", false),
        ("#111827", "RFID Landmark", true),
    ];
    for (index, (color, label, point)) in legend_items.iter().enumerate() {
        let row_y = legend_y + 28.0 + index as f64 * 26.0;
        if *point {
            svg.push_str(&format!(
                "<circle cx='{:.1}' cy='{row_y:.1}' r='5.5' fill='{color}' stroke='white' stroke-width='1.5'/>\n",
                legend_x + 24.0
            ));
        } else {
            svg.push_str(&format!(
                "<line x1='{:.1}' y1='{row_y:.1}' x2='{:.1}' y2='{row_y:.1}' stroke='{color}' stroke-width='5' stroke-linecap='round'/>\n",
                legend_x + 12.0,
                legend_x + 38.0
            ));
        }
        svg.push_str(&format!(
            "<text x='{:.1}' y='{:.1}' font-size='16' font-family='sans-serif' fill='#111827'>{label}</text>\n",
            legend_x + 50.0,
            row_y + 5.0
        ));
    }

    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='16' font-family='sans-serif' font-weight='700' fill='#111827'>Probability</text>\n",
        legend_x + 18.0,
        colorbar_y - 14.0
    ));
    for step in 0..32 {
        let t0 = step as f64 / 31.0;
        let (fill, _) = heatmap_style(t0.max(1e-6), 1.0);
        let y = colorbar_y + colorbar_height * (1.0 - t0) - colorbar_height / 32.0;
        svg.push_str(&format!(
            "<rect x='{colorbar_x:.1}' y='{y:.2}' width='{colorbar_width:.1}' height='{:.2}' fill='{fill}'/>\n",
            colorbar_height / 32.0 + 0.8
        ));
    }
    svg.push_str(&format!(
        "<rect x='{colorbar_x:.1}' y='{colorbar_y:.1}' width='{colorbar_width:.1}' height='{colorbar_height:.1}' fill='none' stroke='#9ca3af'/>\n"
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='13' font-family='monospace' fill='#374151'>max {:.4}</text>\n",
        colorbar_x + colorbar_width + 14.0,
        colorbar_y + 12.0,
        max_probability
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='13' font-family='monospace' fill='#374151'>min 0.0000</text>\n",
        colorbar_x + colorbar_width + 14.0,
        colorbar_y + colorbar_height - 2.0
    ));

    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' text-anchor='middle' font-size='16' font-family='sans-serif' fill='#111827'>x [m]</text>\n",
        plot_left + plot_width / 2.0,
        plot_top + plot_height + 52.0
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' text-anchor='middle' font-size='16' font-family='sans-serif' fill='#111827' transform='rotate(-90, {:.1}, {:.1})'>y [m]</text>\n",
        plot_left - 50.0,
        plot_top + plot_height / 2.0,
        plot_left - 50.0,
        plot_top + plot_height / 2.0
    ));
    svg.push_str("</svg>\n");

    svg
}

fn save_histogram_filter_svg(
    filename: &str,
    area: (f64, f64, f64, f64),
    rfid: &[(f64, f64)],
    history_true: &[(f64, f64)],
    history_dr: &[(f64, f64)],
    history_est: &[(f64, f64)],
    x_width: usize,
    y_width: usize,
    min_x: f64,
    min_y: f64,
    resolution: f64,
    heatmap_data: &[f64],
) -> std::io::Result<()> {
    let svg = build_histogram_filter_svg(
        area,
        rfid,
        history_true,
        history_dr,
        history_est,
        x_width,
        y_width,
        min_x,
        min_y,
        resolution,
        heatmap_data,
    );
    let mut file = File::create(filename)?;
    file.write_all(svg.as_bytes())?;
    Ok(())
}

fn main() {
    println!("Histogram Filter 2D Localization start!");

    // RFID landmark positions [x, y]
    let rfid: Vec<(f64, f64)> = vec![(10.0, 0.0), (10.0, 10.0), (0.0, 15.0), (-5.0, 20.0)];

    // Grid area (matching Python version: -15 to 15 for x, -5 to 25 for y)
    let area = (-15.0, -5.0, 15.0, 25.0); // (min_x, min_y, max_x, max_y)

    // Initialize histogram filter with initial position at origin
    let mut hf = HistogramFilter::new_with_initial_pos(
        area.0,
        area.1,
        area.2,
        area.3,
        XY_RESOLUTION,
        0.0,
        0.0,
        1.0, // initial position (x, y) and initial standard deviation
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
    let mut h_est: Vec<(f64, f64)> = vec![(0.0, 0.0)];

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
        h_est.push(hf.get_estimated_position());

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
                .points(
                    &rfid_x,
                    &rfid_y,
                    &[
                        Caption("RFID"),
                        Color("black"),
                        gnuplot::PointSymbol('O'),
                        gnuplot::PointSize(2.0),
                    ],
                )
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
            Some((
                min_x,
                min_y,
                min_x + x_width as f64 * resolution,
                min_y + y_width as f64 * resolution,
            )),
            &[],
        )
        .points(
            &rfid_x,
            &rfid_y,
            &[
                Caption("RFID"),
                Color("black"),
                gnuplot::PointSymbol('O'),
                gnuplot::PointSize(2.0),
            ],
        )
        .lines(&true_x, &true_y, &[Caption("True"), Color("blue")])
        .lines(&dr_x, &dr_y, &[Caption("Dead Reckoning"), Color("orange")]);

    save_histogram_filter_svg(
        "./img/localization/histogram_filter.svg",
        area,
        &rfid,
        &h_true,
        &h_dr,
        &h_est,
        x_width,
        y_width,
        min_x,
        min_y,
        resolution,
        &heatmap_data,
    )
    .unwrap();
    println!("Plot saved to ./img/localization/histogram_filter.svg");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_histogram_svg_uses_white_background_and_legend_box() {
        let svg = build_histogram_filter_svg(
            (-2.0, -2.0, 2.0, 2.0),
            &[(0.0, 0.0)],
            &[(0.0, 0.0), (1.0, 0.5)],
            &[(0.0, 0.0), (0.8, 0.2)],
            &[(0.0, 0.0), (0.9, 0.4)],
            2,
            2,
            -2.0,
            -2.0,
            2.0,
            &[0.0, 0.1, 0.2, 0.3],
        );

        assert!(svg.contains("<rect width='100%' height='100%' fill='white'/>"));
        assert!(svg.contains("Histogram Estimate"));
        assert!(svg.contains("Probability"));
    }
}
