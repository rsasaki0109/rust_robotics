//! Rectangle fitting via L-shape search.
//!
//! Fits a minimum bounding rectangle to a set of 2D points by searching
//! over orientations and selecting the one that best matches according to
//! a chosen criterion (area, closeness, or variance).
//!
//! Reference: PythonRobotics rectangle_fitting by Atsushi Sakai.
//! Based on "Efficient L-Shape Fitting for Vehicle Detection Using Laser
//! Scanners" (CMU Robotics Institute).

use nalgebra::{Matrix2, Vector2};
use std::f64::consts::FRAC_PI_2;

// ---------------------------------------------------------------------------
// Rectangle data
// ---------------------------------------------------------------------------

/// Four-line representation of a fitted rectangle.
///
/// Each side `i` is represented by the line `a[i]*x + b[i]*y = c[i]`.
#[derive(Debug, Clone, PartialEq)]
pub struct RectangleData {
    /// Coefficients a for the four sides.
    pub a: [f64; 4],
    /// Coefficients b for the four sides.
    pub b: [f64; 4],
    /// Coefficients c for the four sides.
    pub c: [f64; 4],
}

impl RectangleData {
    fn new() -> Self {
        Self {
            a: [0.0; 4],
            b: [0.0; 4],
            c: [0.0; 4],
        }
    }

    /// Compute the four corner vertices of the rectangle.
    ///
    /// Returns `[(x0,y0), (x1,y1), (x2,y2), (x3,y3)]`.
    pub fn corners(&self) -> [(f64, f64); 4] {
        let cross = |i: usize, j: usize| -> (f64, f64) {
            let det = self.a[i] * self.b[j] - self.a[j] * self.b[i];
            let x = (self.b[i] * -self.c[j] - self.b[j] * -self.c[i]) / det;
            let y = (self.a[j] * -self.c[i] - self.a[i] * -self.c[j]) / det;
            (x, y)
        };
        [cross(0, 1), cross(1, 2), cross(2, 3), cross(3, 0)]
    }
}

// ---------------------------------------------------------------------------
// Fitting criteria
// ---------------------------------------------------------------------------

/// Criterion used to score rectangle orientations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Criteria {
    /// Minimise bounding area (negative area is maximised).
    Area,
    /// Maximise closeness of points to the nearest edge.
    Closeness,
    /// Minimise variance of point-to-edge distances.
    Variance,
}

// ---------------------------------------------------------------------------
// L-shape fitting
// ---------------------------------------------------------------------------

/// Configuration and entry point for the L-shape rectangle fitting algorithm.
#[derive(Debug, Clone)]
pub struct LShapeFitting {
    /// Which criterion to optimise.
    pub criteria: Criteria,
    /// Minimum distance clamp for the closeness criterion \[m\].
    pub min_dist_of_closeness_criteria: f64,
    /// Angular search step \[rad\].
    pub d_theta: f64,
    /// Base range for adaptive segmentation \[m\].
    pub r0: f64,
    /// Range-dependent factor for adaptive segmentation \[m\].
    pub rd: f64,
}

impl Default for LShapeFitting {
    fn default() -> Self {
        Self {
            criteria: Criteria::Variance,
            min_dist_of_closeness_criteria: 0.01,
            d_theta: 1.0_f64.to_radians(),
            r0: 3.0,
            rd: 0.001,
        }
    }
}

impl LShapeFitting {
    /// Create a new `LShapeFitting` with default parameters.
    pub fn new() -> Self {
        Self::default()
    }

    /// Run the full fitting pipeline: segment the points, then fit a
    /// rectangle to each cluster.
    ///
    /// Returns a vector of `(RectangleData, Vec<usize>)` pairs where
    /// the second element contains the point indices belonging to that
    /// cluster.
    pub fn fitting(&self, ox: &[f64], oy: &[f64]) -> Vec<(RectangleData, Vec<usize>)> {
        let segments = self.adaptive_range_segmentation(ox, oy);
        segments
            .into_iter()
            .map(|ids| {
                let cx: Vec<f64> = ids.iter().map(|&i| ox[i]).collect();
                let cy: Vec<f64> = ids.iter().map(|&i| oy[i]).collect();
                let rect = self.rectangle_search(&cx, &cy);
                (rect, ids)
            })
            .collect()
    }

    /// Fit a single rectangle to the given points (without segmentation).
    pub fn rectangle_search(&self, x: &[f64], y: &[f64]) -> RectangleData {
        assert_eq!(x.len(), y.len());
        assert!(!x.is_empty(), "need at least one point");

        let n = x.len();

        let mut best_cost = f64::NEG_INFINITY;
        let mut best_theta: f64 = 0.0;

        let mut theta = 0.0_f64;
        while theta < FRAC_PI_2 - self.d_theta {
            let rot = rot_mat_2d(theta);

            let mut c1 = Vec::with_capacity(n);
            let mut c2 = Vec::with_capacity(n);
            for i in 0..n {
                let p = rot * Vector2::new(x[i], y[i]);
                c1.push(p[0]);
                c2.push(p[1]);
            }

            let cost = match self.criteria {
                Criteria::Area => calc_area_criterion(&c1, &c2),
                Criteria::Closeness => self.calc_closeness_criterion(&c1, &c2),
                Criteria::Variance => calc_variance_criterion(&c1, &c2),
            };

            if cost > best_cost {
                best_cost = cost;
                best_theta = theta;
            }

            theta += self.d_theta;
        }

        let sin_s = best_theta.sin();
        let cos_s = best_theta.cos();

        let c1_s: Vec<f64> = x
            .iter()
            .zip(y.iter())
            .map(|(xi, yi)| cos_s * xi + sin_s * yi)
            .collect();
        let c2_s: Vec<f64> = x
            .iter()
            .zip(y.iter())
            .map(|(xi, yi)| -sin_s * xi + cos_s * yi)
            .collect();

        let c1_min = c1_s.iter().cloned().fold(f64::INFINITY, f64::min);
        let c1_max = c1_s.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let c2_min = c2_s.iter().cloned().fold(f64::INFINITY, f64::min);
        let c2_max = c2_s.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        let mut rect = RectangleData::new();
        rect.a = [cos_s, -sin_s, cos_s, -sin_s];
        rect.b = [sin_s, cos_s, sin_s, cos_s];
        rect.c = [c1_min, c2_min, c1_max, c2_max];

        rect
    }

    /// Adaptive range segmentation – groups nearby points into clusters.
    fn adaptive_range_segmentation(&self, ox: &[f64], oy: &[f64]) -> Vec<Vec<usize>> {
        let n = ox.len();
        let mut segments: Vec<Vec<usize>> = Vec::new();

        // Build initial per-point neighbour sets.
        let mut sets: Vec<Vec<usize>> = Vec::with_capacity(n);
        for i in 0..n {
            let r = self.r0 + self.rd * (ox[i].powi(2) + oy[i].powi(2)).sqrt();
            let mut cluster = Vec::new();
            for j in 0..n {
                let d = ((ox[i] - ox[j]).powi(2) + (oy[i] - oy[j]).powi(2)).sqrt();
                if d <= r {
                    cluster.push(j);
                }
            }
            sets.push(cluster);
        }

        // Merge overlapping clusters.
        let mut merged: Vec<Vec<bool>> = sets
            .iter()
            .map(|s| {
                let mut bv = vec![false; n];
                for &idx in s {
                    bv[idx] = true;
                }
                bv
            })
            .collect();

        loop {
            let mut changed = false;
            let len = merged.len();
            let mut i = 0;
            while i < len {
                let mut j = i + 1;
                while j < merged.len() {
                    let overlap = merged[i]
                        .iter()
                        .zip(merged[j].iter())
                        .any(|(a, b)| *a && *b);
                    if overlap {
                        let other = merged.remove(j);
                        for k in 0..n {
                            merged[i][k] = merged[i][k] || other[k];
                        }
                        changed = true;
                    } else {
                        j += 1;
                    }
                }
                i += 1;
            }
            if !changed {
                break;
            }
        }

        for bv in merged {
            let ids: Vec<usize> = bv
                .iter()
                .enumerate()
                .filter_map(|(i, &v)| if v { Some(i) } else { None })
                .collect();
            if !ids.is_empty() {
                segments.push(ids);
            }
        }

        segments
    }

    fn calc_closeness_criterion(&self, c1: &[f64], c2: &[f64]) -> f64 {
        let (c1_max, c1_min, c2_max, c2_min) = find_min_max(c1, c2);
        c1.iter()
            .zip(c2.iter())
            .map(|(v1, v2)| {
                let d1 = (c1_max - v1).min(v1 - c1_min);
                let d2 = (c2_max - v2).min(v2 - c2_min);
                1.0 / d1.min(d2).max(self.min_dist_of_closeness_criteria)
            })
            .sum()
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

fn rot_mat_2d(angle: f64) -> Matrix2<f64> {
    let (s, c) = angle.sin_cos();
    Matrix2::new(c, s, -s, c)
}

fn find_min_max(c1: &[f64], c2: &[f64]) -> (f64, f64, f64, f64) {
    let c1_max = c1.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let c1_min = c1.iter().cloned().fold(f64::INFINITY, f64::min);
    let c2_max = c2.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let c2_min = c2.iter().cloned().fold(f64::INFINITY, f64::min);
    (c1_max, c1_min, c2_max, c2_min)
}

fn calc_area_criterion(c1: &[f64], c2: &[f64]) -> f64 {
    let (c1_max, c1_min, c2_max, c2_min) = find_min_max(c1, c2);
    -((c1_max - c1_min) * (c2_max - c2_min))
}

fn calc_variance_criterion(c1: &[f64], c2: &[f64]) -> f64 {
    let (c1_max, c1_min, c2_max, c2_min) = find_min_max(c1, c2);

    let mut e1: Vec<f64> = Vec::new();
    let mut e2: Vec<f64> = Vec::new();

    for (v1, v2) in c1.iter().zip(c2.iter()) {
        let d1 = (c1_max - v1).min(v1 - c1_min);
        let d2 = (c2_max - v2).min(v2 - c2_min);
        if d1 < d2 {
            e1.push(d1);
        } else {
            e2.push(d2);
        }
    }

    let var = |vals: &[f64]| -> f64 {
        if vals.is_empty() {
            return 0.0;
        }
        let mean = vals.iter().sum::<f64>() / vals.len() as f64;
        vals.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / vals.len() as f64
    };

    let v1 = if e1.is_empty() { 0.0 } else { -var(&e1) };
    let v2 = if e2.is_empty() { 0.0 } else { -var(&e2) };
    v1 + v2
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Generate points on the perimeter of an axis-aligned rectangle.
    fn rect_points(cx: f64, cy: f64, w: f64, h: f64, n_per_side: usize) -> (Vec<f64>, Vec<f64>) {
        let mut xs = Vec::new();
        let mut ys = Vec::new();
        let half_w = w / 2.0;
        let half_h = h / 2.0;
        for i in 0..n_per_side {
            let t = i as f64 / n_per_side as f64;
            // bottom
            xs.push(cx - half_w + w * t);
            ys.push(cy - half_h);
            // top
            xs.push(cx - half_w + w * t);
            ys.push(cy + half_h);
            // left
            xs.push(cx - half_w);
            ys.push(cy - half_h + h * t);
            // right
            xs.push(cx + half_w);
            ys.push(cy - half_h + h * t);
        }
        (xs, ys)
    }

    #[test]
    fn test_rectangle_search_axis_aligned() {
        let (x, y) = rect_points(0.0, 0.0, 4.0, 2.0, 20);
        let fitter = LShapeFitting::new();
        let rect = fitter.rectangle_search(&x, &y);

        let corners = rect.corners();
        let mut xs: Vec<f64> = corners.iter().map(|c| c.0).collect();
        let mut ys: Vec<f64> = corners.iter().map(|c| c.1).collect();
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        ys.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let width = xs[3] - xs[0];
        let height = ys[3] - ys[0];
        let (w, h) = if width > height {
            (width, height)
        } else {
            (height, width)
        };

        assert!((w - 4.0).abs() < 0.2, "width mismatch: {}", w);
        assert!((h - 2.0).abs() < 0.2, "height mismatch: {}", h);
    }

    #[test]
    fn test_rectangle_search_rotated() {
        let angle = PI / 6.0; // 30 degrees
        let (s, c) = angle.sin_cos();
        let (base_x, base_y) = rect_points(0.0, 0.0, 6.0, 3.0, 20);
        let x: Vec<f64> = base_x
            .iter()
            .zip(base_y.iter())
            .map(|(bx, by)| c * bx - s * by)
            .collect();
        let y: Vec<f64> = base_x
            .iter()
            .zip(base_y.iter())
            .map(|(bx, by)| s * bx + c * by)
            .collect();

        let fitter = LShapeFitting::new();
        let rect = fitter.rectangle_search(&x, &y);

        let corners = rect.corners();
        // Check that all 4 edges have roughly the right lengths.
        let dist =
            |a: (f64, f64), b: (f64, f64)| ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt();
        let mut sides = [
            dist(corners[0], corners[1]),
            dist(corners[1], corners[2]),
            dist(corners[2], corners[3]),
            dist(corners[3], corners[0]),
        ];
        sides.sort_by(|a, b| a.partial_cmp(b).unwrap());

        assert!((sides[0] - 3.0).abs() < 0.3, "short side: {}", sides[0]);
        assert!((sides[1] - 3.0).abs() < 0.3, "short side: {}", sides[1]);
        assert!((sides[2] - 6.0).abs() < 0.3, "long side: {}", sides[2]);
        assert!((sides[3] - 6.0).abs() < 0.3, "long side: {}", sides[3]);
    }

    #[test]
    fn test_fitting_with_segmentation() {
        // Two separated clusters
        let (mut x, mut y) = rect_points(0.0, 0.0, 2.0, 1.0, 10);
        let (x2, y2) = rect_points(20.0, 20.0, 3.0, 1.5, 10);
        x.extend(x2);
        y.extend(y2);

        let fitter = LShapeFitting {
            r0: 3.0,
            rd: 0.001,
            ..LShapeFitting::default()
        };
        let results = fitter.fitting(&x, &y);
        assert_eq!(results.len(), 2, "should detect two clusters");
    }

    #[test]
    fn test_criteria_area() {
        let (x, y) = rect_points(0.0, 0.0, 4.0, 2.0, 15);
        let fitter = LShapeFitting {
            criteria: Criteria::Area,
            ..LShapeFitting::default()
        };
        let rect = fitter.rectangle_search(&x, &y);
        let corners = rect.corners();
        // Just verify we get 4 finite corners.
        for (cx, cy) in &corners {
            assert!(cx.is_finite() && cy.is_finite());
        }
    }

    #[test]
    fn test_criteria_closeness() {
        let (x, y) = rect_points(0.0, 0.0, 4.0, 2.0, 15);
        let fitter = LShapeFitting {
            criteria: Criteria::Closeness,
            ..LShapeFitting::default()
        };
        let rect = fitter.rectangle_search(&x, &y);
        let corners = rect.corners();
        for (cx, cy) in &corners {
            assert!(cx.is_finite() && cy.is_finite());
        }
    }
}
