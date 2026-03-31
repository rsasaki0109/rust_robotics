//! Dubins Path Planner
//!
//! Computes the shortest path between two poses (x, y, yaw) for a forward-only
//! vehicle with a bounded minimum turning radius. Unlike Reeds-Shepp paths,
//! Dubins paths only allow forward motion.
//!
//! There are exactly 6 candidate path types, each composed of at most 3 segments
//! of Left (L), Right (R), or Straight (S) arcs:
//! **LSL, RSR, LSR, RSL, RLR, LRL**.
//!
//! # References
//!
//! * Dubins, L.E. (1957). "On Curves of Minimal Length with a Constraint on
//!   Average Curvature, and with Prescribed Initial and Terminal Positions and
//!   Tangents". *American Journal of Mathematics*, 79(3), 497–516.

use std::f64::consts::PI;

use rust_robotics_core::error::{RoboticsError, RoboticsResult};
use rust_robotics_core::types::{Path2D, Point2D, Pose2D};

/// Segment type in a Dubins path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentType {
    /// Left turn (counter-clockwise arc)
    Left,
    /// Straight segment
    Straight,
    /// Right turn (clockwise arc)
    Right,
}

/// One of the six Dubins path type families.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DubinsPathType {
    /// Left – Straight – Left
    LSL,
    /// Right – Straight – Right
    RSR,
    /// Left – Straight – Right
    LSR,
    /// Right – Straight – Left
    RSL,
    /// Right – Left – Right
    RLR,
    /// Left – Right – Left
    LRL,
}

impl DubinsPathType {
    /// Return the three segment types for this path family.
    pub fn segments(self) -> [SegmentType; 3] {
        match self {
            Self::LSL => [SegmentType::Left, SegmentType::Straight, SegmentType::Left],
            Self::RSR => [
                SegmentType::Right,
                SegmentType::Straight,
                SegmentType::Right,
            ],
            Self::LSR => [SegmentType::Left, SegmentType::Straight, SegmentType::Right],
            Self::RSL => [SegmentType::Right, SegmentType::Straight, SegmentType::Left],
            Self::RLR => [SegmentType::Right, SegmentType::Left, SegmentType::Right],
            Self::LRL => [SegmentType::Left, SegmentType::Right, SegmentType::Left],
        }
    }
}

const ALL_PATH_TYPES: [DubinsPathType; 6] = [
    DubinsPathType::LSL,
    DubinsPathType::RSR,
    DubinsPathType::LSR,
    DubinsPathType::RSL,
    DubinsPathType::RLR,
    DubinsPathType::LRL,
];

/// A computed Dubins path consisting of up to three segments.
///
/// The `lengths` array stores the arc-length of each segment in world units
/// \[m\], and `total_length` is their sum.
#[must_use]
#[derive(Debug, Clone)]
pub struct DubinsPath {
    /// The family of this path (e.g. LSL, RSR, …).
    pub path_type: DubinsPathType,
    /// Arc-lengths of the three segments \[m\].
    pub lengths: [f64; 3],
    /// Total path length \[m\].
    pub total_length: f64,
    /// Curvature used when computing this path (1 / min turning radius).
    pub curvature: f64,
    /// Start pose used for interpolation.
    pub start: Pose2D,
}

/// Dubins path planner.
///
/// Computes the shortest forward-only path between two `Pose2D` waypoints for a
/// vehicle whose minimum turning radius is `1.0 / curvature`.
///
/// # Example
///
/// ```
/// use rust_robotics_planning::dubins_path::DubinsPlanner;
/// use rust_robotics_core::types::Pose2D;
///
/// let planner = DubinsPlanner::new(1.0);
/// let start = Pose2D::new(0.0, 0.0, 0.0);
/// let goal  = Pose2D::new(5.0, 5.0, std::f64::consts::FRAC_PI_2);
/// let path  = planner.plan(start, goal).unwrap();
/// assert!(path.total_length > 0.0);
/// ```
pub struct DubinsPlanner {
    /// Curvature = 1 / minimum turning radius \[1/m\].
    pub curvature: f64,
}

impl DubinsPlanner {
    /// Create a new planner with the given curvature (1 / min turning radius).
    ///
    /// # Errors
    ///
    /// Returns `InvalidParameter` if `curvature` is not positive and finite.
    pub fn new(curvature: f64) -> Self {
        Self { curvature }
    }

    /// Plan the shortest Dubins path from `start` to `goal`.
    ///
    /// # Errors
    ///
    /// Returns `PlanningError` if no valid path can be found (should not happen
    /// for finite inputs—included for robustness).
    pub fn plan(&self, start: Pose2D, goal: Pose2D) -> RoboticsResult<DubinsPath> {
        if !self.curvature.is_finite() || self.curvature <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "curvature must be positive and finite".to_string(),
            ));
        }

        // Transform goal into the start-centred, normalised frame.
        let dx = goal.x - start.x;
        let dy = goal.y - start.y;
        let d = (dx * dx + dy * dy).sqrt() * self.curvature;

        let theta = dy.atan2(dx);
        let alpha = mod2pi(start.yaw - theta);
        let beta = mod2pi(goal.yaw - theta);

        let mut best: Option<DubinsPath> = None;

        for &pt in &ALL_PATH_TYPES {
            if let Some(lengths) = compute_path_params(alpha, beta, d, pt) {
                // All segment lengths must be non-negative for a valid Dubins path.
                if lengths.iter().any(|&l| l < 0.0) {
                    continue;
                }
                // Convert normalised lengths to world lengths.
                let scale = 1.0 / self.curvature;
                let world_lengths = [lengths[0] * scale, lengths[1] * scale, lengths[2] * scale];
                let total = world_lengths[0] + world_lengths[1] + world_lengths[2];

                if best.as_ref().is_none_or(|b| total < b.total_length) {
                    best = Some(DubinsPath {
                        path_type: pt,
                        lengths: world_lengths,
                        total_length: total,
                        curvature: self.curvature,
                        start,
                    });
                }
            }
        }

        best.ok_or_else(|| RoboticsError::PlanningError("no valid Dubins path found".to_string()))
    }

    /// Sample evenly-spaced points along a previously computed `DubinsPath`.
    ///
    /// `step` is the distance \[m\] between consecutive sample points.
    pub fn sample_path(&self, path: &DubinsPath, step: f64) -> Path2D {
        let mut points = vec![Point2D::new(path.start.x, path.start.y)];
        let segments = path.path_type.segments();
        let step = if step <= 0.0 { 0.1 } else { step };

        for (i, &seg) in segments.iter().enumerate() {
            let seg_len = path.lengths[i] * path.curvature;
            if seg_len.abs() < 1e-12 {
                continue;
            }

            let origin = *points.last().unwrap();
            let origin_yaw = segment_end_yaw(&points, path, i);
            let mut current_length = step;

            while (current_length + step).abs() <= seg_len.abs() {
                let (x, y, _) =
                    interpolate_segment(current_length, seg, path.curvature, origin, origin_yaw);
                points.push(Point2D::new(x, y));
                current_length += step;
            }

            let (x, y, _) = interpolate_segment(seg_len, seg, path.curvature, origin, origin_yaw);
            points.push(Point2D::new(x, y));
        }

        Path2D::from_points(points)
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Normalise an angle to \[0, 2π).
fn mod2pi(angle: f64) -> f64 {
    let mut v = angle % (2.0 * PI);
    if v < 0.0 {
        v += 2.0 * PI;
    }
    v
}

fn segment_end_yaw(points: &[Point2D], path: &DubinsPath, segment_index: usize) -> f64 {
    let segments = path.path_type.segments();
    let mut yaw = path.start.yaw;
    for (i, seg) in segments.iter().enumerate().take(segment_index) {
        let normalized_len = path.lengths[i] * path.curvature;
        yaw = match seg {
            SegmentType::Straight => yaw,
            SegmentType::Left => yaw + normalized_len,
            SegmentType::Right => yaw - normalized_len,
        };
    }
    if points.is_empty() {
        path.start.yaw
    } else {
        yaw
    }
}

/// Interpolate a point at a normalized segment length.
fn interpolate_segment(
    length: f64,
    seg: SegmentType,
    curvature: f64,
    origin: Point2D,
    origin_yaw: f64,
) -> (f64, f64, f64) {
    match seg {
        SegmentType::Straight => {
            let nx = origin.x + length / curvature * origin_yaw.cos();
            let ny = origin.y + length / curvature * origin_yaw.sin();
            (nx, ny, origin_yaw)
        }
        SegmentType::Left => {
            let r = 1.0 / curvature;
            let ldx = length.sin() * r;
            let ldy = (1.0 - length.cos()) * r;
            let gdx = origin_yaw.cos() * ldx - origin_yaw.sin() * ldy;
            let gdy = origin_yaw.sin() * ldx + origin_yaw.cos() * ldy;
            (origin.x + gdx, origin.y + gdy, origin_yaw + length)
        }
        SegmentType::Right => {
            let r = 1.0 / curvature;
            let ldx = length.sin() * r;
            let ldy = -(1.0 - length.cos()) * r;
            let gdx = origin_yaw.cos() * ldx - origin_yaw.sin() * ldy;
            let gdy = origin_yaw.sin() * ldx + origin_yaw.cos() * ldy;
            (origin.x + gdx, origin.y + gdy, origin_yaw - length)
        }
    }
}

/// Compute the three normalised segment lengths for a given Dubins path type.
///
/// `alpha` and `beta` are the start and goal headings relative to the
/// start-to-goal line, normalised to \[0, 2π). `d` is the normalised distance
/// (Euclidean distance × curvature).
///
/// Returns `None` when the geometry has no real solution.
fn compute_path_params(
    alpha: f64,
    beta: f64,
    d: f64,
    path_type: DubinsPathType,
) -> Option<[f64; 3]> {
    match path_type {
        DubinsPathType::LSL => lsl(alpha, beta, d),
        DubinsPathType::RSR => rsr(alpha, beta, d),
        DubinsPathType::LSR => lsr(alpha, beta, d),
        DubinsPathType::RSL => rsl(alpha, beta, d),
        DubinsPathType::RLR => rlr(alpha, beta, d),
        DubinsPathType::LRL => lrl(alpha, beta, d),
    }
}

fn lsl(alpha: f64, beta: f64, d: f64) -> Option<[f64; 3]> {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let cos_ab = (alpha - beta).cos();
    let p_sq = 2.0 + d * d - 2.0 * cos_ab + 2.0 * d * (sa - sb);
    if p_sq < 0.0 {
        return None;
    }
    let tmp = (cb - ca).atan2(d + sa - sb);
    let t = mod2pi(-alpha + tmp);
    let p = p_sq.sqrt();
    let q = mod2pi(beta - tmp);
    Some([t, p, q])
}

fn rsr(alpha: f64, beta: f64, d: f64) -> Option<[f64; 3]> {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let cos_ab = (alpha - beta).cos();
    let p_sq = 2.0 + d * d - 2.0 * cos_ab + 2.0 * d * (sb - sa);
    if p_sq < 0.0 {
        return None;
    }
    let tmp = (ca - cb).atan2(d - sa + sb);
    let t = mod2pi(alpha - tmp);
    let p = p_sq.sqrt();
    let q = mod2pi(-beta + tmp);
    Some([t, p, q])
}

fn lsr(alpha: f64, beta: f64, d: f64) -> Option<[f64; 3]> {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let cos_ab = (alpha - beta).cos();
    let p_sq = -2.0 + d * d + 2.0 * cos_ab + 2.0 * d * (sa + sb);
    if p_sq < 0.0 {
        return None;
    }
    let p = p_sq.sqrt();
    let tmp = (-ca - cb).atan2(d + sa + sb) - (-2.0_f64).atan2(p);
    let t = mod2pi(-alpha + tmp);
    let q = mod2pi(-beta + tmp);
    Some([t, p, q])
}

fn rsl(alpha: f64, beta: f64, d: f64) -> Option<[f64; 3]> {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let cos_ab = (alpha - beta).cos();
    let p_sq = -2.0 + d * d + 2.0 * cos_ab - 2.0 * d * (sa + sb);
    if p_sq < 0.0 {
        return None;
    }
    let p = p_sq.sqrt();
    let tmp = (ca + cb).atan2(d - sa - sb) - (2.0_f64).atan2(p);
    let t = mod2pi(alpha - tmp);
    let q = mod2pi(beta - tmp);
    Some([t, p, q])
}

fn rlr(alpha: f64, beta: f64, d: f64) -> Option<[f64; 3]> {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let cos_ab = (alpha - beta).cos();
    let val = (6.0 - d * d + 2.0 * cos_ab + 2.0 * d * (sa - sb)) / 8.0;
    if val.abs() > 1.0 {
        return None;
    }
    let p = mod2pi(2.0 * PI - val.acos());
    let t = mod2pi(alpha - (ca - cb).atan2(d - sa + sb) + mod2pi(p / 2.0));
    let q = mod2pi(alpha - beta - t + mod2pi(p));
    Some([t, p, q])
}

fn lrl(alpha: f64, beta: f64, d: f64) -> Option<[f64; 3]> {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let cos_ab = (alpha - beta).cos();
    let val = (6.0 - d * d + 2.0 * cos_ab - 2.0 * d * (sa - sb)) / 8.0;
    if val.abs() > 1.0 {
        return None;
    }
    let p = mod2pi(2.0 * PI - val.acos());
    let t = mod2pi(-alpha - (ca - cb).atan2(d + sa - sb) + p / 2.0);
    let q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p));
    Some([t, p, q])
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
#[allow(clippy::excessive_precision)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const CURVATURE: f64 = 1.0;
    const EPS: f64 = 0.3;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn assert_point_close(actual: &Point2D, expected: (f64, f64), tolerance: f64) {
        assert!(
            (actual.x - expected.0).abs() < tolerance,
            "x mismatch: actual=({}, {}), expected={expected:?}",
            actual.x,
            actual.y
        );
        assert!(
            (actual.y - expected.1).abs() < tolerance,
            "y mismatch: actual=({}, {}), expected={expected:?}",
            actual.x,
            actual.y
        );
    }

    #[test]
    fn test_dubins_straight_line() {
        // Same heading, goal straight ahead → should be close to pure straight.
        let planner = DubinsPlanner::new(CURVATURE);
        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(10.0, 0.0, 0.0);
        let path = planner.plan(start, goal).unwrap();

        // Total length should be approximately the Euclidean distance.
        assert!(
            approx_eq(path.total_length, 10.0, EPS),
            "expected ~10.0, got {}",
            path.total_length
        );

        // Sample and verify endpoints.
        let sampled = planner.sample_path(&path, 0.1);
        assert!(!sampled.is_empty());
        let first = &sampled.points[0];
        let last = &sampled.points[sampled.len() - 1];
        assert!(approx_eq(first.x, 0.0, EPS) && approx_eq(first.y, 0.0, EPS));
        assert!(
            approx_eq(last.x, 10.0, EPS) && approx_eq(last.y, 0.0, EPS),
            "last point = ({}, {})",
            last.x,
            last.y
        );
    }

    #[test]
    fn test_dubins_u_turn() {
        // Goal at same position but opposite heading → requires a turn-around.
        let planner = DubinsPlanner::new(CURVATURE);
        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(0.0, 0.0, PI);
        let path = planner.plan(start, goal).unwrap();

        // A U-turn for curvature=1 (radius=1) should have length = π.
        // (semicircle of radius 1)
        assert!(
            path.total_length >= PI - EPS,
            "U-turn too short: {}",
            path.total_length
        );
        assert!(path.total_length.is_finite());
    }

    #[test]
    fn test_dubins_90_degree_turn() {
        let planner = DubinsPlanner::new(1.0);
        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(3.0, 3.0, FRAC_PI_2);
        let path = planner.plan(start, goal).unwrap();

        assert!(path.total_length > 0.0);
        assert!(path.total_length.is_finite());

        let sampled = planner.sample_path(&path, 0.05);
        assert!(sampled.len() > 2);
        let last = &sampled.points[sampled.len() - 1];
        assert!(
            approx_eq(last.x, 3.0, EPS) && approx_eq(last.y, 3.0, EPS),
            "endpoint ({}, {}) far from goal (3, 3)",
            last.x,
            last.y
        );
    }

    #[test]
    fn test_dubins_all_types_finite() {
        // Verify that the planner returns a finite path for a variety of goals.
        let planner = DubinsPlanner::new(0.5);
        let start = Pose2D::new(0.0, 0.0, 0.0);

        let goals = [
            Pose2D::new(4.0, 4.0, 1.0),
            Pose2D::new(-3.0, 5.0, -1.5),
            Pose2D::new(6.0, -2.0, PI),
            Pose2D::new(0.0, 8.0, -FRAC_PI_2),
        ];

        for goal in &goals {
            let path = planner.plan(start, *goal).unwrap();
            assert!(
                path.total_length.is_finite(),
                "non-finite length for goal {:?}",
                goal
            );
            assert!(
                path.lengths.iter().all(|l| l.is_finite() && *l >= 0.0),
                "invalid segment lengths for goal {:?}: {:?}",
                goal,
                path.lengths
            );
        }
    }

    #[test]
    fn test_dubins_path_length_positive() {
        let planner = DubinsPlanner::new(2.0);
        let start = Pose2D::new(1.0, 2.0, 0.5);
        let goal = Pose2D::new(5.0, 6.0, -0.5);
        let path = planner.plan(start, goal).unwrap();

        assert!(path.total_length > 0.0);
        // Total length must be at least the straight-line distance.
        let euclidean = ((goal.x - start.x).powi(2) + (goal.y - start.y).powi(2)).sqrt();
        assert!(
            path.total_length >= euclidean - 1e-9,
            "Dubins path ({}) shorter than Euclidean ({})",
            path.total_length,
            euclidean
        );
    }

    #[test]
    fn test_dubins_invalid_curvature() {
        let planner = DubinsPlanner::new(0.0);
        let result = planner.plan(Pose2D::origin(), Pose2D::new(1.0, 0.0, 0.0));
        assert!(result.is_err());

        let planner = DubinsPlanner::new(-1.0);
        let result = planner.plan(Pose2D::origin(), Pose2D::new(1.0, 0.0, 0.0));
        assert!(result.is_err());
    }

    #[test]
    fn test_dubins_sample_path_consistent_length() {
        let planner = DubinsPlanner::new(1.0);
        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(5.0, 0.0, 0.0);
        let path = planner.plan(start, goal).unwrap();
        let sampled = planner.sample_path(&path, 0.1);

        // The polyline length should approximate the analytical total length.
        let polyline_len = sampled.total_length();
        assert!(
            approx_eq(polyline_len, path.total_length, 0.5),
            "polyline {} vs analytical {}",
            polyline_len,
            path.total_length
        );
    }

    #[test]
    fn test_compute_path_params_match_upstream_representative_window() {
        let alpha = PI;
        let beta = FRAC_PI_2;
        let d = 5.656_854_249_492_38;

        assert_eq!(
            compute_path_params(alpha, beta, d, DubinsPathType::LSL),
            Some([
                3.353_117_643_613_227_3,
                4.763_012_859_631_52,
                1.359_271_336_771_462_2
            ])
        );
        assert_eq!(
            compute_path_params(alpha, beta, d, DubinsPathType::RSR),
            Some([
                3.290_698_833_617_270_2,
                6.731_545_773_370_686,
                4.563_282_800_357_213
            ])
        );
        assert_eq!(
            compute_path_params(alpha, beta, d, DubinsPathType::LSR),
            Some([
                3.592_361_893_573_783,
                6.427_574_075_729_097,
                5.163_158_220_368_679
            ])
        );
        assert_eq!(
            compute_path_params(alpha, beta, d, DubinsPathType::RSL),
            Some([
                3.786_455_298_170_522_6,
                4.322_764_335_586_111,
                2.215_658_971_375_626
            ])
        );
        assert_eq!(
            compute_path_params(alpha, beta, d, DubinsPathType::RLR),
            None
        );
        assert_eq!(
            compute_path_params(alpha, beta, d, DubinsPathType::LRL),
            None
        );
    }

    #[test]
    fn test_compute_path_params_match_upstream_rlr_lrl_case() {
        let alpha = 0.0;
        let beta = 0.0;
        let d = 0.5;

        assert_eq!(
            compute_path_params(alpha, beta, d, DubinsPathType::RLR),
            Some([
                3.016_264_822_421_727_7,
                6.032_529_644_843_455,
                3.016_264_822_421_727_7
            ])
        );
        assert_eq!(
            compute_path_params(alpha, beta, d, DubinsPathType::LRL),
            Some([
                3.016_264_822_421_727_7,
                6.032_529_644_843_455,
                3.016_264_822_421_727_7
            ])
        );
    }

    #[test]
    fn test_dubins_plan_matches_upstream_main_example() {
        let planner = DubinsPlanner::new(1.0);
        let start = Pose2D::new(1.0, 1.0, 45.0_f64.to_radians());
        let goal = Pose2D::new(-3.0, -3.0, -45.0_f64.to_radians());
        let path = planner.plan(start, goal).unwrap();

        assert_eq!(path.path_type, DubinsPathType::LSL);
        assert!(approx_eq(path.lengths[0], 3.353_117_643_613_227, 1e-12));
        assert!(approx_eq(path.lengths[1], 4.763_012_859_631_52, 1e-12));
        assert!(approx_eq(path.lengths[2], 1.359_271_336_771_462, 1e-12));
        assert!(approx_eq(path.total_length, 9.475_401_840_016_21, 1e-12));
    }

    #[test]
    fn test_dubins_sample_path_matches_upstream_main_example() {
        let planner = DubinsPlanner::new(1.0);
        let start = Pose2D::new(1.0, 1.0, 45.0_f64.to_radians());
        let goal = Pose2D::new(-3.0, -3.0, -45.0_f64.to_radians());
        let path = planner.plan(start, goal).unwrap();
        let sampled = planner.sample_path(&path, 0.1);

        let expected_samples = [
            (0usize, 1.0, 1.0),
            (10, 1.269_954_482_712_928, 1.920_065_196_345_844),
            (20, 0.641_603_345_345_556, 2.644_337_407_902_28),
            (30, -0.307_350_274_196_291, 2.506_924_103_516_754),
            (93, -3.0, -3.0),
        ];

        assert_eq!(sampled.len(), 94);
        for (index, x, y) in expected_samples {
            assert_point_close(&sampled.points[index], (x, y), 1e-12);
        }

        let sum_x: f64 = sampled.points.iter().map(|point| point.x).sum();
        let sum_y: f64 = sampled.points.iter().map(|point| point.y).sum();
        let weighted_sum_x: f64 = sampled
            .points
            .iter()
            .enumerate()
            .map(|(index, point)| (index + 1) as f64 * point.x)
            .sum();
        let weighted_sum_y: f64 = sampled
            .points
            .iter()
            .enumerate()
            .map(|(index, point)| (index + 1) as f64 * point.y)
            .sum();

        assert!(approx_eq(sum_x, -105.952_049_447_379_94, 1e-9));
        assert!(approx_eq(sum_y, 52.686_985_733_395_81, 1e-9));
        assert!(approx_eq(weighted_sum_x, -8_974.696_943_796_953, 1e-6));
        assert!(approx_eq(weighted_sum_y, -1_434.362_763_693_722_6, 1e-6));
    }
}
