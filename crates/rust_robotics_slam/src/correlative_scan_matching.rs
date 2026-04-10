//! Brute-force correlative scan matching.
//!
//! The matcher scores candidate poses by transforming a query scan into the
//! reference frame and counting occupancy hits in a discretized grid.
//!
//! Reference:
//! - Edwin Olson, "Real-Time Correlative Scan Matching":
//!   <https://april.eecs.umich.edu/pdfs/olson2009icra.pdf>
//!
//! The lookup table follows the rasterization idea in Olson's paper: each
//! reference point contributes a radially symmetric score field and the grid
//! stores the maximum value contributed at each cell.

use std::collections::HashMap;
use std::f64::consts::PI;

/// Configuration for correlative scan matching.
#[derive(Debug, Clone, Copy)]
pub struct CorrelativeScanMatcherConfig {
    /// Search window for x and y offsets \[m\].
    pub linear_search_range: f64,
    /// Search window for yaw offsets \[rad\].
    pub angular_search_range: f64,
    /// Search step for x and y offsets \[m\].
    pub linear_step: f64,
    /// Search step for yaw offsets \[rad\].
    pub angular_step: f64,
    /// Occupancy lookup grid resolution \[m\].
    pub grid_resolution: f64,
}

impl Default for CorrelativeScanMatcherConfig {
    fn default() -> Self {
        Self {
            linear_search_range: 1.0,
            angular_search_range: 0.2,
            linear_step: 0.1,
            angular_step: 0.02,
            grid_resolution: 0.05,
        }
    }
}

/// Best scan matching result found in the search window.
#[derive(Debug, Clone, Copy)]
pub struct ScanMatchResult {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub score: f64,
    pub converged: bool,
}

/// Performs brute-force correlative scan matching.
pub fn correlative_scan_match(
    reference_x: &[f64],
    reference_y: &[f64],
    query_x: &[f64],
    query_y: &[f64],
    initial_pose: (f64, f64, f64),
    config: &CorrelativeScanMatcherConfig,
) -> ScanMatchResult {
    if reference_x.len() != reference_y.len()
        || query_x.len() != query_y.len()
        || reference_x.is_empty()
        || query_x.is_empty()
        || config.linear_step <= 0.0
        || config.angular_step <= 0.0
        || config.grid_resolution <= 0.0
    {
        return ScanMatchResult {
            x: initial_pose.0,
            y: initial_pose.1,
            yaw: initial_pose.2,
            score: 0.0,
            converged: false,
        };
    }

    let grid = build_lookup_table(reference_x, reference_y, config.grid_resolution);
    let linear_offsets = enumerate_offsets(config.linear_search_range, config.linear_step);
    let angular_offsets = enumerate_offsets(config.angular_search_range, config.angular_step);

    let mut best = ScanMatchResult {
        x: initial_pose.0,
        y: initial_pose.1,
        yaw: normalize_angle(initial_pose.2),
        score: -1.0,
        converged: false,
    };
    let mut best_penalty = f64::INFINITY;

    for dx in &linear_offsets {
        for dy in &linear_offsets {
            for dyaw in &angular_offsets {
                let candidate = (
                    initial_pose.0 + dx,
                    initial_pose.1 + dy,
                    normalize_angle(initial_pose.2 + dyaw),
                );
                let score =
                    score_candidate(&grid, query_x, query_y, candidate, config.grid_resolution);
                let penalty = dx * dx + dy * dy + dyaw * dyaw;

                if score > best.score || (score == best.score && penalty < best_penalty) {
                    best = ScanMatchResult {
                        x: candidate.0,
                        y: candidate.1,
                        yaw: candidate.2,
                        score,
                        converged: score > 0.0,
                    };
                    best_penalty = penalty;
                }
            }
        }
    }

    best
}

fn enumerate_offsets(range: f64, step: f64) -> Vec<f64> {
    let n_steps = (range / step).round() as i32;
    (-n_steps..=n_steps)
        .map(|index| index as f64 * step)
        .collect()
}

fn build_lookup_table(
    reference_x: &[f64],
    reference_y: &[f64],
    resolution: f64,
) -> HashMap<(i32, i32), f64> {
    let mut grid: HashMap<(i32, i32), f64> = HashMap::new();
    let sigma = resolution;
    let cutoff_radius = (3.0 * sigma / resolution).ceil() as i32;
    let inv_two_sigma_sq = 0.5 / (sigma * sigma);

    for (&x, &y) in reference_x.iter().zip(reference_y.iter()) {
        let (cx, cy) = cell_index(x, y, resolution);
        for ix in (cx - cutoff_radius)..=(cx + cutoff_radius) {
            for iy in (cy - cutoff_radius)..=(cy + cutoff_radius) {
                let gx = ix as f64 * resolution;
                let gy = iy as f64 * resolution;
                let squared_distance = (gx - x).powi(2) + (gy - y).powi(2);
                let weight = (-squared_distance * inv_two_sigma_sq).exp();
                if weight < 1.0e-6 {
                    continue;
                }

                grid.entry((ix, iy))
                    .and_modify(|score| *score = (*score).max(weight))
                    .or_insert(weight);
            }
        }
    }

    grid
}

fn score_candidate(
    grid: &HashMap<(i32, i32), f64>,
    query_x: &[f64],
    query_y: &[f64],
    pose: (f64, f64, f64),
    resolution: f64,
) -> f64 {
    let cos_yaw = pose.2.cos();
    let sin_yaw = pose.2.sin();
    let mut score = 0.0;

    for (&x, &y) in query_x.iter().zip(query_y.iter()) {
        let world_x = cos_yaw * x - sin_yaw * y + pose.0;
        let world_y = sin_yaw * x + cos_yaw * y + pose.1;
        let cell = cell_index(world_x, world_y, resolution);
        score += grid.get(&cell).copied().unwrap_or(0.0);
    }

    score
}

fn cell_index(x: f64, y: f64, resolution: f64) -> (i32, i32) {
    (
        (x / resolution).round() as i32,
        (y / resolution).round() as i32,
    )
}

fn normalize_angle(mut angle: f64) -> f64 {
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inverse_transform_points(
        points: &[(f64, f64)],
        pose: (f64, f64, f64),
    ) -> (Vec<f64>, Vec<f64>) {
        let cos_yaw = pose.2.cos();
        let sin_yaw = pose.2.sin();
        let mut xs = Vec::with_capacity(points.len());
        let mut ys = Vec::with_capacity(points.len());

        for &(x, y) in points {
            let dx = x - pose.0;
            let dy = y - pose.1;
            xs.push(cos_yaw * dx + sin_yaw * dy);
            ys.push(-sin_yaw * dx + cos_yaw * dy);
        }

        (xs, ys)
    }

    fn fixture_points() -> Vec<(f64, f64)> {
        vec![
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (0.0, 1.0),
            (0.0, 2.0),
            (1.0, 1.0),
            (1.5, 2.0),
        ]
    }

    #[test]
    fn test_identity_match() {
        let points = fixture_points();
        let (reference_x, reference_y): (Vec<_>, Vec<_>) = points.iter().copied().unzip();
        let config = CorrelativeScanMatcherConfig::default();

        let result = correlative_scan_match(
            &reference_x,
            &reference_y,
            &reference_x,
            &reference_y,
            (0.0, 0.0, 0.0),
            &config,
        );

        assert!(result.converged);
        assert_eq!(result.x, 0.0);
        assert_eq!(result.y, 0.0);
        assert_eq!(result.yaw, 0.0);
        assert_eq!(result.score, points.len() as f64);
    }

    #[test]
    fn test_known_translation_recovery() {
        let points = fixture_points();
        let (reference_x, reference_y): (Vec<_>, Vec<_>) = points.iter().copied().unzip();
        let true_pose = (0.4, -0.3, 0.0);
        let (query_x, query_y) = inverse_transform_points(&points, true_pose);
        let config = CorrelativeScanMatcherConfig {
            linear_search_range: 0.6,
            angular_search_range: 0.1,
            linear_step: 0.1,
            angular_step: 0.05,
            grid_resolution: 0.05,
        };

        let result = correlative_scan_match(
            &reference_x,
            &reference_y,
            &query_x,
            &query_y,
            (0.0, 0.0, 0.0),
            &config,
        );

        assert!(result.converged);
        assert!((result.x - true_pose.0).abs() < 1.0e-9);
        assert!((result.y - true_pose.1).abs() < 1.0e-9);
        assert!(result.yaw.abs() < 1.0e-9);
    }

    #[test]
    fn test_known_rotation_recovery() {
        let points = fixture_points();
        let (reference_x, reference_y): (Vec<_>, Vec<_>) = points.iter().copied().unzip();
        let true_pose = (0.0, 0.0, 0.16);
        let (query_x, query_y) = inverse_transform_points(&points, true_pose);
        let config = CorrelativeScanMatcherConfig {
            linear_search_range: 0.2,
            angular_search_range: 0.3,
            linear_step: 0.1,
            angular_step: 0.02,
            grid_resolution: 0.05,
        };

        let result = correlative_scan_match(
            &reference_x,
            &reference_y,
            &query_x,
            &query_y,
            (0.0, 0.0, 0.0),
            &config,
        );

        assert!(result.converged);
        assert!(result.x.abs() < 1.0e-9);
        assert!(result.y.abs() < 1.0e-9);
        assert!((result.yaw - true_pose.2).abs() < 1.0e-9);
    }

    #[test]
    fn test_lookup_table_prefers_exact_alignment() {
        let points = fixture_points();
        let (reference_x, reference_y): (Vec<_>, Vec<_>) = points.iter().copied().unzip();
        let resolution = 0.05;
        let grid = build_lookup_table(&reference_x, &reference_y, resolution);

        let aligned = score_candidate(
            &grid,
            &reference_x,
            &reference_y,
            (0.0, 0.0, 0.0),
            resolution,
        );
        let translated = score_candidate(
            &grid,
            &reference_x,
            &reference_y,
            (0.15, 0.0, 0.0),
            resolution,
        );
        let rotated = score_candidate(
            &grid,
            &reference_x,
            &reference_y,
            (0.0, 0.0, 0.15),
            resolution,
        );

        assert!(aligned > translated);
        assert!(aligned > rotated);
    }
}
