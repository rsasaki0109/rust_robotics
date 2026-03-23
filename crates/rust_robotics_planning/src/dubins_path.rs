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
    fn segments(self) -> [SegmentType; 3] {
        match self {
            Self::LSL => [SegmentType::Left, SegmentType::Straight, SegmentType::Left],
            Self::RSR => [SegmentType::Right, SegmentType::Straight, SegmentType::Right],
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
                let world_lengths = [
                    lengths[0] * scale,
                    lengths[1] * scale,
                    lengths[2] * scale,
                ];
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

        best.ok_or_else(|| {
            RoboticsError::PlanningError("no valid Dubins path found".to_string())
        })
    }

    /// Sample evenly-spaced points along a previously computed `DubinsPath`.
    ///
    /// `step` is the distance \[m\] between consecutive sample points.
    pub fn sample_path(&self, path: &DubinsPath, step: f64) -> Path2D {
        let mut points = Vec::new();
        let segments = path.path_type.segments();

        let mut x = path.start.x;
        let mut y = path.start.y;
        let mut yaw = path.start.yaw;

        let step = if step <= 0.0 { 0.1 } else { step };

        // For each of the three segments, step through and interpolate.
        for (i, &seg) in segments.iter().enumerate() {
            let seg_len = path.lengths[i];
            if seg_len < 1e-12 {
                continue;
            }

            let mut traveled = 0.0;
            while traveled < seg_len {
                points.push(Point2D::new(x, y));
                let ds = step.min(seg_len - traveled);
                let (nx, ny, nyaw) = step_segment(x, y, yaw, ds, seg, path.curvature);
                x = nx;
                y = ny;
                yaw = nyaw;
                traveled += ds;
            }
        }

        // Push the final point.
        points.push(Point2D::new(x, y));

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

/// Step a single infinitesimal motion along a segment.
fn step_segment(
    x: f64,
    y: f64,
    yaw: f64,
    ds: f64,
    seg: SegmentType,
    curvature: f64,
) -> (f64, f64, f64) {
    match seg {
        SegmentType::Straight => {
            let nx = x + ds * yaw.cos();
            let ny = y + ds * yaw.sin();
            (nx, ny, yaw)
        }
        SegmentType::Left => {
            // Arc with positive angular change.
            let dphi = ds * curvature;
            let r = 1.0 / curvature;
            let cx = x - r * yaw.sin();
            let cy = y + r * yaw.cos();
            let new_yaw = yaw + dphi;
            let nx = cx + r * new_yaw.sin();
            let ny = cy - r * new_yaw.cos();
            (nx, ny, new_yaw)
        }
        SegmentType::Right => {
            // Arc with negative angular change.
            let dphi = ds * curvature;
            let r = 1.0 / curvature;
            let cx = x + r * yaw.sin();
            let cy = y - r * yaw.cos();
            let new_yaw = yaw - dphi;
            let nx = cx - r * new_yaw.sin();
            let ny = cy + r * new_yaw.cos();
            (nx, ny, new_yaw)
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
    let c_ab = (2.0 + d * d - 2.0 * (ca - cb) + 2.0 * d * (sa - sb)).max(0.0);
    let p_sq = c_ab;
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
    let c_ab = (2.0 + d * d - 2.0 * (ca - cb) - 2.0 * d * (sa - sb)).max(0.0);
    let p_sq = c_ab;
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
    let p_sq = -2.0 + d * d + 2.0 * (ca + cb) + 2.0 * d * (sa + sb);
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
    let p_sq = -2.0 + d * d + 2.0 * (ca + cb) - 2.0 * d * (sa + sb);
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
    let val = (6.0 - d * d + 2.0 * (ca - cb) + 2.0 * d * (sa - sb)) / 8.0;
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
    let val = (6.0 - d * d + 2.0 * (ca - cb) - 2.0 * d * (sa - sb)) / 8.0;
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
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const CURVATURE: f64 = 1.0;
    const EPS: f64 = 0.3;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
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
}
