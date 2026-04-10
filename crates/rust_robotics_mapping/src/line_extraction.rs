//! Line extraction from 2D point clouds using the Split-and-Merge algorithm.
//!
//! The Split-and-Merge (Iterative End Point Fit) algorithm recursively splits
//! a point sequence wherever a point deviates too far from the line connecting
//! the two endpoints, then merges adjacent collinear segments.
//!
//! Reference: PythonRobotics line_extraction by Atsushi Sakai.

/// Configuration parameters for line extraction.
#[derive(Debug, Clone, PartialEq)]
pub struct LineExtractionConfig {
    /// Maximum perpendicular distance \[m\] from a point to its segment line
    /// before the segment is split.
    pub distance_threshold: f64,
    /// Minimum number of points required to form a valid segment.
    pub min_segment_length: usize,
    /// Maximum angle difference \[rad\] between two adjacent segments for them
    /// to be merged into one.
    pub merge_angle_threshold: f64,
}

impl Default for LineExtractionConfig {
    fn default() -> Self {
        Self {
            distance_threshold: 0.1,
            min_segment_length: 3,
            merge_angle_threshold: 0.1,
        }
    }
}

/// A line segment defined by its two endpoints.
#[derive(Debug, Clone, PartialEq)]
pub struct LineSegment {
    /// X coordinate of the start point.
    pub start_x: f64,
    /// Y coordinate of the start point.
    pub start_y: f64,
    /// X coordinate of the end point.
    pub end_x: f64,
    /// Y coordinate of the end point.
    pub end_y: f64,
}

impl LineSegment {
    /// Euclidean length of the segment.
    pub fn length(&self) -> f64 {
        let dx = self.end_x - self.start_x;
        let dy = self.end_y - self.start_y;
        dx.hypot(dy)
    }

    /// Angle of the segment in radians (atan2 of direction vector).
    fn angle(&self) -> f64 {
        (self.end_y - self.start_y).atan2(self.end_x - self.start_x)
    }
}

/// Result of a line extraction run.
pub struct LineExtractionResult {
    /// Extracted line segments.
    pub segments: Vec<LineSegment>,
    /// X coordinates of the input points (echoed for convenience).
    pub x: Vec<f64>,
    /// Y coordinates of the input points (echoed for convenience).
    pub y: Vec<f64>,
}

/// Perpendicular distance from point `(px, py)` to the line through
/// `(x1, y1)` and `(x2, y2)`.
///
/// Returns `0.0` when the two line endpoints coincide.
pub fn point_to_line_distance(px: f64, py: f64, x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    let dx = x2 - x1;
    let dy = y2 - y1;
    let len_sq = dx * dx + dy * dy;
    if len_sq == 0.0 {
        return ((px - x1).powi(2) + (py - y1).powi(2)).sqrt();
    }
    // |cross product| / |line vector|
    ((dy * px - dx * py + x2 * y1 - y2 * x1) / len_sq.sqrt()).abs()
}

/// Extract line segments from a 2D point cloud using Split-and-Merge.
///
/// # Panics
///
/// Panics if `x` and `y` have different lengths or contain fewer than 2 points.
pub fn line_extraction(
    x: &[f64],
    y: &[f64],
    config: &LineExtractionConfig,
) -> LineExtractionResult {
    assert_eq!(x.len(), y.len(), "x and y must have the same length");
    assert!(
        x.len() >= 2,
        "at least 2 points are required for line extraction"
    );

    let indices: Vec<usize> = (0..x.len()).collect();
    let mut segments = split_and_merge(x, y, &indices, config);
    merge_collinear(&mut segments, config.merge_angle_threshold);

    LineExtractionResult {
        segments,
        x: x.to_vec(),
        y: y.to_vec(),
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Recursively split the point subsequence `indices` into line segments.
fn split_and_merge(
    x: &[f64],
    y: &[f64],
    indices: &[usize],
    config: &LineExtractionConfig,
) -> Vec<LineSegment> {
    if indices.len() < config.min_segment_length {
        // Not enough points — emit a degenerate segment if at least 2 points.
        if indices.len() >= 2 {
            let first = indices[0];
            let last = *indices.last().unwrap();
            return vec![LineSegment {
                start_x: x[first],
                start_y: y[first],
                end_x: x[last],
                end_y: y[last],
            }];
        }
        return vec![];
    }

    let first = indices[0];
    let last = *indices.last().unwrap();
    let (x1, y1) = (x[first], y[first]);
    let (x2, y2) = (x[last], y[last]);

    // Find the point with the maximum perpendicular distance.
    let (max_dist, split_pos) = indices[1..indices.len() - 1]
        .iter()
        .enumerate()
        .map(|(i, &idx)| {
            let d = point_to_line_distance(x[idx], y[idx], x1, y1, x2, y2);
            (d, i + 1) // offset by 1 because we sliced from index 1
        })
        .fold((0.0_f64, 0usize), |(best_d, best_i), (d, i)| {
            if d > best_d {
                (d, i)
            } else {
                (best_d, best_i)
            }
        });

    if max_dist > config.distance_threshold && split_pos > 0 {
        // Split at the farthest point and recurse on both halves.
        let left = split_and_merge(x, y, &indices[..=split_pos], config);
        let right = split_and_merge(x, y, &indices[split_pos..], config);
        [left, right].concat()
    } else {
        vec![LineSegment {
            start_x: x1,
            start_y: y1,
            end_x: x2,
            end_y: y2,
        }]
    }
}

/// Merge adjacent segments whose angular difference is below `threshold`.
fn merge_collinear(segments: &mut Vec<LineSegment>, threshold: f64) {
    let mut i = 0;
    while i + 1 < segments.len() {
        let angle_diff = (segments[i].angle() - segments[i + 1].angle())
            .abs()
            .rem_euclid(std::f64::consts::PI);
        let angle_diff = angle_diff.min(std::f64::consts::PI - angle_diff);
        if angle_diff < threshold {
            let merged = LineSegment {
                start_x: segments[i].start_x,
                start_y: segments[i].start_y,
                end_x: segments[i + 1].end_x,
                end_y: segments[i + 1].end_y,
            };
            segments[i] = merged;
            segments.remove(i + 1);
        } else {
            i += 1;
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_4;

    #[test]
    fn test_config_defaults() {
        let cfg = LineExtractionConfig::default();
        assert_eq!(cfg.distance_threshold, 0.1);
        assert_eq!(cfg.min_segment_length, 3);
        assert_eq!(cfg.merge_angle_threshold, 0.1);
    }

    #[test]
    fn test_point_to_line_distance_known_values() {
        // Point (0, 1) to the x-axis should be 1.0.
        let d = point_to_line_distance(0.0, 1.0, -1.0, 0.0, 1.0, 0.0);
        assert!((d - 1.0).abs() < 1e-12, "expected 1.0, got {d}");

        // Point on the line should give 0.
        let d2 = point_to_line_distance(0.5, 0.0, 0.0, 0.0, 1.0, 0.0);
        assert!(d2.abs() < 1e-12, "expected 0.0, got {d2}");

        // Point (1, 1) to the diagonal y=x (through origin at 45°).
        // Distance = |1*1 - 1*1| / sqrt(2) = 0
        let d3 = point_to_line_distance(1.0, 1.0, 0.0, 0.0, 2.0, 2.0);
        assert!(d3.abs() < 1e-12, "expected 0.0, got {d3}");

        // Point (0, 1) to the diagonal y=x.
        // Line: y - x = 0  =>  distance = |0 - 1| / sqrt(2) = 1/sqrt(2)
        let d4 = point_to_line_distance(0.0, 1.0, 0.0, 0.0, 1.0, 1.0);
        let expected = 1.0 / std::f64::consts::SQRT_2;
        assert!(
            (d4 - expected).abs() < 1e-12,
            "expected {expected}, got {d4}"
        );
    }

    #[test]
    fn test_collinear_points_produce_one_segment() {
        // Ten points along y = 2x; should yield a single segment.
        let x: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|xi| 2.0 * xi).collect();

        let cfg = LineExtractionConfig::default();
        let result = line_extraction(&x, &y, &cfg);

        assert_eq!(
            result.segments.len(),
            1,
            "collinear points should yield exactly 1 segment, got {:?}",
            result.segments
        );
        let seg = &result.segments[0];
        assert!((seg.start_x - 0.0).abs() < 1e-10);
        assert!((seg.start_y - 0.0).abs() < 1e-10);
        assert!((seg.end_x - 9.0).abs() < 1e-10);
        assert!((seg.end_y - 18.0).abs() < 1e-10);
        println!("collinear segment: {seg:?}, length={}", seg.length());
    }

    #[test]
    fn test_l_shape_produces_two_segments() {
        // Horizontal arm: (0,0)..(4,0), then vertical arm: (4,0)..(4,4).
        let mut x = Vec::new();
        let mut y = Vec::new();
        for i in 0..=4 {
            x.push(i as f64);
            y.push(0.0);
        }
        // Exclude the shared corner to avoid duplicate.
        for j in 1..=4 {
            x.push(4.0);
            y.push(j as f64);
        }

        let cfg = LineExtractionConfig {
            distance_threshold: 0.05,
            min_segment_length: 3,
            merge_angle_threshold: 0.1,
        };
        let result = line_extraction(&x, &y, &cfg);

        assert_eq!(
            result.segments.len(),
            2,
            "L-shape should yield 2 segments, got {:?}",
            result.segments
        );

        // The first segment should be nearly horizontal (|angle| ≈ 0).
        let h_seg = &result.segments[0];
        assert!(
            h_seg.angle().abs() < 0.01,
            "first segment should be horizontal, angle={}",
            h_seg.angle()
        );

        // The second segment should be nearly vertical (|angle| ≈ π/2).
        let v_seg = &result.segments[1];
        assert!(
            (v_seg.angle().abs() - FRAC_PI_4 * 2.0).abs() < 0.01,
            "second segment should be vertical, angle={}",
            v_seg.angle()
        );

        println!("L-shape segments: {h_seg:?}, {v_seg:?}");
    }
}
