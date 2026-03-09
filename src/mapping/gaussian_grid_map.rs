#![allow(dead_code)]

// Gaussian Grid Map
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109)
//         Rust port

use nalgebra::DMatrix;
use std::fs::File;
use std::io::Write;

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
                let min_dist = ox
                    .iter()
                    .zip(oy.iter())
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
    let a1 = 0.254829592;
    let a2 = -0.284496736;
    let a3 = 1.421413741;
    let a4 = -1.453152027;
    let a5 = 1.061405429;
    let p = 0.3275911;

    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let x = x.abs();

    let t = 1.0 / (1.0 + p * x);
    let y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * (-x * x).exp();

    sign * y
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

fn gaussian_heat_style(probability: f64, max_probability: f64) -> (String, f64) {
    if max_probability <= 0.0 || probability <= 0.0 {
        return ("#fffaf0".to_string(), 0.0);
    }

    let t = (probability / max_probability).sqrt().clamp(0.0, 1.0);
    let rgb = if t < 0.5 {
        lerp_rgb((255, 250, 240), (191, 219, 254), t * 2.0)
    } else {
        lerp_rgb((191, 219, 254), (30, 64, 175), (t - 0.5) * 2.0)
    };

    (
        format!("#{:02x}{:02x}{:02x}", rgb.0, rgb.1, rgb.2),
        0.14 + 0.82 * t,
    )
}

fn build_gaussian_grid_map_svg(grid_map: &GaussianGridMap, ox: &[f64], oy: &[f64]) -> String {
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

    let max_probability = grid_map.data.iter().copied().fold(0.0, f64::max);
    let cell_size = grid_map.resolution * scale;

    let mut svg = String::new();
    svg.push_str(&format!(
        "<?xml version='1.0' encoding='UTF-8'?>\n<svg xmlns='http://www.w3.org/2000/svg' width='{width}' height='{height}' viewBox='0 0 {width} {height}'>\n"
    ));
    svg.push_str("<rect width='100%' height='100%' fill='white'/>\n");
    svg.push_str(&format!(
        "<text x='{:.1}' y='38' text-anchor='middle' font-size='24' font-family='sans-serif' font-weight='700' fill='#111827'>Gaussian Grid Map</text>\n",
        width / 2.0
    ));
    svg.push_str(&format!(
        "<rect x='{plot_left:.1}' y='{plot_top:.1}' width='{plot_width:.1}' height='{plot_height:.1}' fill='#fffdf7' stroke='#d6d3d1' stroke-width='2'/>\n"
    ));

    let x_tick_start = (grid_map.min_x / 5.0).ceil() as i32;
    let x_tick_end = (grid_map.max_x / 5.0).floor() as i32;
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

    let y_tick_start = (grid_map.min_y / 5.0).ceil() as i32;
    let y_tick_end = (grid_map.max_y / 5.0).floor() as i32;
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

    for iy in 0..grid_map.y_width {
        for ix in 0..grid_map.x_width {
            let probability = grid_map.data[(ix, iy)];
            if probability <= max_probability * 0.01 {
                continue;
            }
            let x = grid_map.min_x + ix as f64 * grid_map.resolution;
            let y = grid_map.min_y + iy as f64 * grid_map.resolution;
            let sx = to_svg_x(x);
            let sy = to_svg_y(y + grid_map.resolution);
            let (fill, opacity) = gaussian_heat_style(probability, max_probability);
            svg.push_str(&format!(
                "<rect x='{sx:.2}' y='{sy:.2}' width='{cell_size:.2}' height='{cell_size:.2}' fill='{fill}' fill-opacity='{opacity:.3}' stroke='none'/>\n"
            ));
        }
    }

    for (x, y) in ox.iter().zip(oy.iter()) {
        svg.push_str(&format!(
            "<circle cx='{:.2}' cy='{:.2}' r='5.5' fill='#111827' stroke='white' stroke-width='1.8'/>\n",
            to_svg_x(*x),
            to_svg_y(*y)
        ));
    }

    svg.push_str(&format!(
        "<rect x='{legend_x:.1}' y='{legend_y:.1}' width='182' height='118' rx='18' fill='white' stroke='#d6d3d1'/>\n"
    ));
    let swatches = [
        ("#1e40af", "High probability"),
        ("#93c5fd", "Medium probability"),
        ("#fffaf0", "Low probability"),
    ];
    for (index, (color, label)) in swatches.iter().enumerate() {
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
    let obstacle_row_y = legend_y + 28.0 + swatches.len() as f64 * 26.0;
    svg.push_str(&format!(
        "<circle cx='{:.1}' cy='{obstacle_row_y:.1}' r='5.5' fill='#111827' stroke='white' stroke-width='1.5'/>\n",
        legend_x + 25.0
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='16' font-family='sans-serif' fill='#111827'>Obstacles</text>\n",
        legend_x + 48.0,
        obstacle_row_y + 1.0
    ));

    let colorbar_x = legend_x + 18.0;
    let colorbar_y = legend_y + 160.0;
    let colorbar_height = 250.0;
    let colorbar_width = 26.0;
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='16' font-family='sans-serif' font-weight='700' fill='#111827'>Probability</text>\n",
        legend_x + 18.0,
        colorbar_y - 14.0
    ));
    for step in 0..32 {
        let t = step as f64 / 31.0;
        let (fill, _) = gaussian_heat_style(t.max(1e-6), 1.0);
        let y = colorbar_y + colorbar_height * (1.0 - t) - colorbar_height / 32.0;
        svg.push_str(&format!(
            "<rect x='{colorbar_x:.1}' y='{y:.2}' width='{colorbar_width:.1}' height='{:.2}' fill='{fill}'/>\n",
            colorbar_height / 32.0 + 0.8
        ));
    }
    svg.push_str(&format!(
        "<rect x='{colorbar_x:.1}' y='{colorbar_y:.1}' width='{colorbar_width:.1}' height='{colorbar_height:.1}' fill='none' stroke='#9ca3af'/>\n"
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='13' font-family='monospace' fill='#374151'>max {:.3}</text>\n",
        colorbar_x + colorbar_width + 14.0,
        colorbar_y + 12.0,
        max_probability
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='13' font-family='monospace' fill='#374151'>min 0.000</text>\n",
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
        plot_left - 48.0,
        plot_top + plot_height / 2.0,
        plot_left - 48.0,
        plot_top + plot_height / 2.0
    ));
    svg.push_str("</svg>\n");

    svg
}

fn save_gaussian_grid_map_svg(
    filename: &str,
    grid_map: &GaussianGridMap,
    ox: &[f64],
    oy: &[f64],
) -> std::io::Result<()> {
    let svg = build_gaussian_grid_map_svg(grid_map, ox, oy);
    let mut file = File::create(filename)?;
    file.write_all(svg.as_bytes())?;
    Ok(())
}

/// Draw heatmap of the grid map
fn draw_heatmap(grid_map: &GaussianGridMap, ox: &[f64], oy: &[f64]) {
    save_gaussian_grid_map_svg("./img/mapping/gaussian_grid_map.svg", grid_map, ox, oy).unwrap();
    println!("Plot saved to ./img/mapping/gaussian_grid_map.svg");
}

fn main() {
    println!("Gaussian Grid Map start!");

    // Define obstacles (sample points)
    let ox: Vec<f64> = vec![
        0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
    ];
    let oy: Vec<f64> = vec![
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
        0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
    ];

    // Create Gaussian Grid Map
    let grid_map = GaussianGridMap::new(&ox, &oy, XY_RESOLUTION, STD);

    println!("Grid map created:");
    println!("  Size: {} x {}", grid_map.x_width, grid_map.y_width);
    println!(
        "  Bounds: ({:.1}, {:.1}) to ({:.1}, {:.1})",
        grid_map.min_x, grid_map.min_y, grid_map.max_x, grid_map.max_y
    );

    // Draw heatmap
    draw_heatmap(&grid_map, &ox, &oy);

    println!("Done!");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gaussian_grid_map_svg_has_white_background() {
        let grid_map = GaussianGridMap::new(&[0.0, 1.0], &[0.0, 1.0], 1.0, 2.0);
        let svg = build_gaussian_grid_map_svg(&grid_map, &[0.0, 1.0], &[0.0, 1.0]);

        assert!(svg.contains("<rect width='100%' height='100%' fill='white'/>"));
        assert!(svg.contains("Gaussian Grid Map"));
        assert!(svg.contains("Probability"));
        assert!(svg.contains("Obstacles"));
    }
}
