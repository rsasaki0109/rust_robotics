//! Scan-to-map 2D LiDAR odometry with a bounded local submap.
//!
//! Scan-to-scan matching registers each new scan against the previous one, so
//! per-scan matching noise accumulates as a random walk. This module instead
//! keeps a **local submap** of recent keyframe scans stored in the corrected
//! world frame and registers every new scan against it:
//!
//! ```text
//! predicted_pose = corrected_pose ⊕ odom_delta
//! corrected_pose = ICP(submap_world, scan_body, seed = predicted_pose)
//! submap        ← submap ∪ (corrected_pose * scan_body)   (keyframes only)
//! ```
//!
//! Because the submap is anchored to the running corrected pose, each accepted
//! correction "sticks": the next scan is matched against geometry that already
//! absorbed previous corrections, instead of against a single neighbor.
//!
//! Registration is point-to-line Gauss-Newton: submap points carry a normal
//! estimated from their neighbors in beam order, and correspondences farther
//! than [`ScanToMapConfig::max_correspondence_distance`] are rejected so newly
//! observed geometry does not drag the estimate. Corrections larger than the
//! configured gates are rejected and the odometry prediction is kept.
//!
//! Setting `max_scans = 1` and a zero keyframe threshold reduces the matcher to
//! seeded scan-to-scan matching ([`ScanToMapConfig::scan_to_scan`]), so both
//! modes can be compared under identical inputs.
//!
//! Design notes: `docs/scan_to_map_icp_design.md`.

use std::collections::{HashMap, VecDeque};
use std::f64::consts::PI;

use nalgebra::{Matrix2, Matrix3, Vector2, Vector3};
use rust_robotics_core::Pose2D;

/// Tuning parameters for [`ScanToMapMatcher`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScanToMapConfig {
    /// Maximum number of keyframe scans kept in the submap.
    pub max_scans: usize,
    /// Submap points farther than this from the predicted pose are ignored \[m\].
    pub max_radius: f64,
    /// Minimum translation since the last keyframe before a scan is inserted \[m\].
    pub keyframe_translation: f64,
    /// Minimum rotation since the last keyframe before a scan is inserted \[rad\].
    pub keyframe_yaw: f64,
    /// Voxel edge used to decimate scans before matching and insertion \[m\].
    /// `0.0` disables decimation.
    pub voxel_size: f64,
    /// Gauss-Newton iterations (each one re-associates correspondences).
    pub max_iterations: usize,
    /// Correspondences farther than this are rejected \[m\].
    pub max_correspondence_distance: f64,
    /// Huber threshold on the point-to-line residual \[m\].
    pub huber_delta: f64,
    /// Minimum number of accepted correspondences for a valid match.
    pub min_correspondences: usize,
    /// Reject matches whose correction exceeds this translation \[m\].
    pub max_correction_translation: f64,
    /// Reject matches whose correction exceeds this rotation \[rad\].
    pub max_correction_yaw: f64,
    /// Reject matches whose mean absolute point-to-line residual exceeds this \[m\].
    pub max_mean_residual: f64,
    /// Neighbor search radius used for normal estimation in beam order \[m\].
    pub normal_neighbor_distance: f64,
    /// When `λ_min / λ_max` of the translational Hessian falls below this
    /// ratio (e.g. in a featureless corridor), the Gauss-Newton step along the
    /// weak direction is discarded and the odometry prediction is kept there.
    /// `0.0` disables the projection.
    pub degeneracy_ratio: f64,
}

impl Default for ScanToMapConfig {
    fn default() -> Self {
        Self {
            max_scans: 20,
            max_radius: 12.0,
            keyframe_translation: 0.25,
            keyframe_yaw: 0.1,
            voxel_size: 0.05,
            max_iterations: 20,
            max_correspondence_distance: 0.3,
            huber_delta: 0.05,
            min_correspondences: 20,
            max_correction_translation: 0.5,
            max_correction_yaw: 0.2,
            max_mean_residual: 0.1,
            normal_neighbor_distance: 0.3,
            degeneracy_ratio: 0.03,
        }
    }
}

impl ScanToMapConfig {
    /// Seeded scan-to-scan preset: the submap holds only the previous scan.
    pub fn scan_to_scan() -> Self {
        Self {
            max_scans: 1,
            keyframe_translation: 0.0,
            keyframe_yaw: 0.0,
            ..Self::default()
        }
    }
}

/// Outcome of a single [`ScanToMapMatcher::update`] call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MatchStatus {
    /// The submap was too sparse to match against; the prediction was kept.
    Bootstrap,
    /// The match passed all gates and its correction was applied.
    Accepted,
    /// Too few correspondences survived the distance gate.
    RejectedTooFewCorrespondences,
    /// The correction exceeded the translation or rotation gate.
    RejectedLargeCorrection,
    /// The final mean residual exceeded [`ScanToMapConfig::max_mean_residual`].
    RejectedHighResidual,
}

impl MatchStatus {
    /// Whether the match correction was applied.
    pub fn is_accepted(self) -> bool {
        self == Self::Accepted
    }
}

/// Diagnostics returned by [`ScanToMapMatcher::update`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScanToMapUpdate {
    /// Pose predicted from the previous corrected pose and the odometry delta.
    pub predicted_pose: Pose2D,
    /// Pose after registration (equal to `predicted_pose` unless accepted).
    pub corrected_pose: Pose2D,
    /// Match status and gate decision.
    pub status: MatchStatus,
    /// Gauss-Newton iterations run.
    pub iterations: usize,
    /// Correspondences used in the final iteration.
    pub correspondences: usize,
    /// Mean absolute point-to-line residual after registration \[m\].
    pub mean_residual: f64,
    /// Registration normal matrix (see [`ScanRegistration::hessian`]);
    /// zero when no registration ran.
    pub hessian: Matrix3<f64>,
    /// Submap points inside the matching radius.
    pub submap_points: usize,
    /// Whether this scan was inserted into the submap as a keyframe.
    pub inserted_keyframe: bool,
}

/// A line segment used by [`ray_cast_ranges`] to synthesize range scans.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LineSegment {
    pub start: Vector2<f64>,
    pub end: Vector2<f64>,
}

impl LineSegment {
    pub fn new(start: Vector2<f64>, end: Vector2<f64>) -> Self {
        Self { start, end }
    }
}

#[derive(Debug, Clone)]
struct Keyframe {
    points: Vec<Vector2<f64>>,
    normals: Vec<Vector2<f64>>,
}

/// Incremental scan-to-map LiDAR odometry.
#[derive(Debug, Clone)]
pub struct ScanToMapMatcher {
    config: ScanToMapConfig,
    pose: Pose2D,
    keyframes: VecDeque<Keyframe>,
    last_keyframe_pose: Option<Pose2D>,
}

impl ScanToMapMatcher {
    /// Creates a matcher whose corrected trajectory starts at `initial_pose`.
    pub fn new(config: ScanToMapConfig, initial_pose: Pose2D) -> Self {
        Self {
            config,
            pose: initial_pose,
            keyframes: VecDeque::new(),
            last_keyframe_pose: None,
        }
    }

    /// Current corrected pose.
    pub fn pose(&self) -> Pose2D {
        self.pose
    }

    /// Matcher configuration.
    pub fn config(&self) -> &ScanToMapConfig {
        &self.config
    }

    /// Number of keyframe scans currently held in the submap.
    pub fn keyframe_count(&self) -> usize {
        self.keyframes.len()
    }

    /// All submap points in the corrected world frame.
    pub fn submap_points(&self) -> Vec<Vector2<f64>> {
        self.keyframes
            .iter()
            .flat_map(|keyframe| keyframe.points.iter().copied())
            .collect()
    }

    /// Drops the submap and restarts the trajectory at `pose`.
    pub fn reset(&mut self, pose: Pose2D) {
        self.pose = pose;
        self.keyframes.clear();
        self.last_keyframe_pose = None;
    }

    /// Integrates one odometry step and registers the scan taken at its end.
    ///
    /// * `odom_delta` - body-frame motion since the previous call
    ///   (`x` forward, `y` left, `yaw` counter-clockwise).
    /// * `scan_body` - scan points in the body frame, **ordered by beam angle**
    ///   (the order is used to estimate normals).
    pub fn update(&mut self, odom_delta: Pose2D, scan_body: &[Vector2<f64>]) -> ScanToMapUpdate {
        let predicted_pose = compose_pose(self.pose, odom_delta);
        let (target_points, target_normals) = self.local_target(predicted_pose);
        let source = voxel_downsample(scan_body, self.config.voxel_size);

        let mut update = ScanToMapUpdate {
            predicted_pose,
            corrected_pose: predicted_pose,
            status: MatchStatus::Bootstrap,
            iterations: 0,
            correspondences: 0,
            mean_residual: 0.0,
            hessian: Matrix3::zeros(),
            submap_points: target_points.len(),
            inserted_keyframe: false,
        };

        if target_points.len() >= self.config.min_correspondences {
            let registration = register_point_to_line(
                &source,
                &target_points,
                &target_normals,
                predicted_pose,
                &self.config,
            );
            update.iterations = registration.iterations;
            update.correspondences = registration.correspondences;
            update.mean_residual = registration.mean_residual;
            update.hessian = registration.hessian;
            let correction = relative_pose(predicted_pose, registration.pose);
            update.status = if registration.correspondences < self.config.min_correspondences {
                MatchStatus::RejectedTooFewCorrespondences
            } else if correction.x.hypot(correction.y) > self.config.max_correction_translation
                || correction.yaw.abs() > self.config.max_correction_yaw
            {
                MatchStatus::RejectedLargeCorrection
            } else if registration.mean_residual > self.config.max_mean_residual {
                MatchStatus::RejectedHighResidual
            } else {
                update.corrected_pose = registration.pose;
                MatchStatus::Accepted
            };
        }

        self.pose = update.corrected_pose;
        let may_insert = matches!(
            update.status,
            MatchStatus::Bootstrap | MatchStatus::Accepted
        );
        if may_insert && self.is_keyframe(self.pose) {
            self.insert_keyframe(scan_body, self.pose);
            update.inserted_keyframe = true;
        }
        update
    }

    fn is_keyframe(&self, pose: Pose2D) -> bool {
        let Some(last) = self.last_keyframe_pose else {
            return true;
        };
        let delta = relative_pose(last, pose);
        delta.x.hypot(delta.y) >= self.config.keyframe_translation
            || delta.yaw.abs() >= self.config.keyframe_yaw
    }

    fn insert_keyframe(&mut self, scan_body: &[Vector2<f64>], pose: Pose2D) {
        let (points, world_normals) = scan_points_with_normals(
            scan_body,
            pose,
            self.config.normal_neighbor_distance,
            self.config.voxel_size,
        );
        self.keyframes.push_back(Keyframe {
            points,
            normals: world_normals,
        });
        while self.keyframes.len() > self.config.max_scans.max(1) {
            self.keyframes.pop_front();
        }
        self.last_keyframe_pose = Some(pose);
    }

    fn local_target(&self, center: Pose2D) -> (Vec<Vector2<f64>>, Vec<Vector2<f64>>) {
        let center = Vector2::new(center.x, center.y);
        let radius_sq = self.config.max_radius * self.config.max_radius;
        let mut points = Vec::new();
        let mut normals = Vec::new();
        let mut occupied = HashMap::new();
        // Newest keyframes first so overlapping voxels keep the freshest point.
        for keyframe in self.keyframes.iter().rev() {
            for (point, normal) in keyframe.points.iter().zip(&keyframe.normals) {
                if (point - center).norm_squared() > radius_sq {
                    continue;
                }
                if self.config.voxel_size > 0.0
                    && occupied
                        .insert(voxel_key(point, self.config.voxel_size), ())
                        .is_some()
                {
                    continue;
                }
                points.push(*point);
                normals.push(*normal);
            }
        }
        (points, normals)
    }
}

/// Result of [`register_point_to_line`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScanRegistration {
    /// Estimated pose mapping source (body) points onto the target frame.
    pub pose: Pose2D,
    /// Gauss-Newton iterations run.
    pub iterations: usize,
    /// Correspondences inside `max_correspondence_distance` in the last iteration.
    pub correspondences: usize,
    /// Mean absolute point-to-line residual of those correspondences \[m\].
    pub mean_residual: f64,
    /// Gauss-Newton normal matrix `Σ w J Jᵀ` of the last iteration over the
    /// world-frame `(x, y, yaw)`. Its small eigenvalues reveal directions the
    /// geometry cannot observe (e.g. along a featureless corridor).
    pub hessian: Matrix3<f64>,
}

/// Transforms a beam-ordered body-frame scan to `pose` and attaches normals.
///
/// Points without a reliable normal are dropped; `voxel_size > 0` keeps one
/// point per voxel. Returns `(points, normals)` in the frame of `pose`.
pub fn scan_points_with_normals(
    scan_body: &[Vector2<f64>],
    pose: Pose2D,
    normal_neighbor_distance: f64,
    voxel_size: f64,
) -> (Vec<Vector2<f64>>, Vec<Vector2<f64>>) {
    let normals = estimate_scan_normals(scan_body, normal_neighbor_distance);
    let rotation = rotation(pose.yaw);
    let translation = Vector2::new(pose.x, pose.y);
    let mut points = Vec::new();
    let mut world_normals = Vec::new();
    let mut occupied = HashMap::new();
    for (point, normal) in scan_body.iter().zip(normals) {
        let Some(normal) = normal else {
            continue;
        };
        let world = rotation * point + translation;
        if voxel_size > 0.0 && occupied.insert(voxel_key(&world, voxel_size), ()).is_some() {
            continue;
        }
        points.push(world);
        world_normals.push(rotation * normal);
    }
    (points, world_normals)
}

/// Registers body-frame `source` points against `target` points with normals,
/// starting from `seed`, by point-to-line Gauss-Newton on `(x, y, yaw)`.
///
/// Uses `max_iterations`, `max_correspondence_distance`, and `huber_delta`
/// from `config`; the acceptance gates are left to the caller.
pub fn register_point_to_line(
    source: &[Vector2<f64>],
    target: &[Vector2<f64>],
    target_normals: &[Vector2<f64>],
    seed: Pose2D,
    config: &ScanToMapConfig,
) -> ScanRegistration {
    let grid = NeighborGrid::new(target, config.max_correspondence_distance);

    let mut pose = seed;
    let mut iterations = 0;
    let mut correspondences = 0;
    let mut mean_residual = f64::INFINITY;
    let mut final_hessian = Matrix3::zeros();

    for _ in 0..config.max_iterations.max(1) {
        iterations += 1;
        let (sin, cos) = pose.yaw.sin_cos();
        let rot = Matrix2::new(cos, -sin, sin, cos);
        let rot_derivative = Matrix2::new(-sin, -cos, cos, -sin);
        let translation = Vector2::new(pose.x, pose.y);

        let mut hessian = Matrix3::zeros();
        let mut gradient = Vector3::zeros();
        let mut residual_sum = 0.0;
        correspondences = 0;
        for point in source {
            let world = rot * point + translation;
            let Some(index) = grid.nearest_within(&world) else {
                continue;
            };
            let normal = target_normals[index];
            let residual = normal.dot(&(world - target[index]));
            let jacobian = Vector3::new(normal.x, normal.y, normal.dot(&(rot_derivative * point)));
            let weight = huber_weight(residual, config.huber_delta);
            hessian += weight * jacobian * jacobian.transpose();
            gradient += weight * jacobian * residual;
            residual_sum += residual.abs();
            correspondences += 1;
        }
        if correspondences == 0 {
            break;
        }
        mean_residual = residual_sum / correspondences as f64;
        final_hessian = hessian;
        if correspondences < 3 {
            break;
        }
        // Light Levenberg damping keeps degenerate geometry (corridors) bounded.
        let damping = 1.0e-6 * hessian.trace().max(1.0e-9);
        let Some(inverse) = (hessian + Matrix3::identity() * damping).try_inverse() else {
            break;
        };
        let mut step = -inverse * gradient;
        if let Some(weak) = degenerate_direction(&hessian, config.degeneracy_ratio) {
            let along = weak.dot(&step.xy());
            step.x -= along * weak.x;
            step.y -= along * weak.y;
        }
        pose = Pose2D::new(
            pose.x + step.x,
            pose.y + step.y,
            wrap_angle(pose.yaw + step.z),
        );
        if step.x.hypot(step.y) < 1.0e-5 && step.z.abs() < 1.0e-5 {
            break;
        }
    }

    ScanRegistration {
        pose,
        iterations,
        correspondences,
        mean_residual,
        hessian: final_hessian,
    }
}

/// Weakest translation direction of a registration Hessian, if its
/// `λ_min / λ_max` ratio is below `ratio_threshold`.
pub fn degenerate_direction(hessian: &Matrix3<f64>, ratio_threshold: f64) -> Option<Vector2<f64>> {
    let (ratio, direction) = translational_observability(hessian);
    (ratio < ratio_threshold).then_some(direction)
}

/// `(λ_min / λ_max, weakest unit direction)` of the translational 2×2 block of
/// a registration Hessian. A ratio near 0 means translation along the
/// direction is unobservable; an all-zero Hessian returns ratio 0.
pub fn translational_observability(hessian: &Matrix3<f64>) -> (f64, Vector2<f64>) {
    let (a, b, c) = (hessian[(0, 0)], hessian[(0, 1)], hessian[(1, 1)]);
    let half_trace = 0.5 * (a + c);
    let spread = (0.25 * (a - c) * (a - c) + b * b).sqrt();
    let (large, small) = (half_trace + spread, half_trace - spread);
    // Eigenvector of the larger eigenvalue has angle 0.5·atan2(2b, a − c).
    let angle = 0.5 * (2.0 * b).atan2(a - c);
    let weak = Vector2::new(-angle.sin(), angle.cos());
    if large <= 0.0 {
        return (0.0, weak);
    }
    ((small / large).max(0.0), weak)
}

/// Dense uniform grid answering "nearest point within `radius`" queries.
///
/// The cell edge equals the radius, so the 3×3 block around the query cell
/// contains every candidate. Cells are stored CSR-style (offsets + indices).
struct NeighborGrid<'a> {
    points: &'a [Vector2<f64>],
    radius: f64,
    min_x: f64,
    min_y: f64,
    columns: i64,
    rows: i64,
    offsets: Vec<usize>,
    indices: Vec<usize>,
}

impl<'a> NeighborGrid<'a> {
    fn new(points: &'a [Vector2<f64>], radius: f64) -> Self {
        let radius = radius.max(1.0e-6);
        let (mut min_x, mut min_y) = (f64::INFINITY, f64::INFINITY);
        let (mut max_x, mut max_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
        for point in points {
            min_x = min_x.min(point.x);
            min_y = min_y.min(point.y);
            max_x = max_x.max(point.x);
            max_y = max_y.max(point.y);
        }
        if points.is_empty() {
            (min_x, min_y, max_x, max_y) = (0.0, 0.0, 0.0, 0.0);
        }
        let columns = ((max_x - min_x) / radius).floor() as i64 + 1;
        let rows = ((max_y - min_y) / radius).floor() as i64 + 1;
        let cell_of = |point: &Vector2<f64>| -> usize {
            let column = ((point.x - min_x) / radius).floor() as i64;
            let row = ((point.y - min_y) / radius).floor() as i64;
            (row * columns + column) as usize
        };
        let mut offsets = vec![0usize; (columns * rows) as usize + 1];
        for point in points {
            offsets[cell_of(point) + 1] += 1;
        }
        for cell in 1..offsets.len() {
            offsets[cell] += offsets[cell - 1];
        }
        let mut cursor = offsets.clone();
        let mut indices = vec![0usize; points.len()];
        for (index, point) in points.iter().enumerate() {
            let cell = cell_of(point);
            indices[cursor[cell]] = index;
            cursor[cell] += 1;
        }
        Self {
            points,
            radius,
            min_x,
            min_y,
            columns,
            rows,
            offsets,
            indices,
        }
    }

    fn nearest_within(&self, query: &Vector2<f64>) -> Option<usize> {
        let (qx, qy) = (query.x, query.y);
        let column = ((qx - self.min_x) / self.radius).floor() as i64;
        let row = ((qy - self.min_y) / self.radius).floor() as i64;
        let mut best = None;
        let mut best_distance_sq = self.radius * self.radius;
        for r in (row - 1).max(0)..=(row + 1).min(self.rows - 1) {
            for c in (column - 1).max(0)..=(column + 1).min(self.columns - 1) {
                let cell = (r * self.columns + c) as usize;
                for &index in &self.indices[self.offsets[cell]..self.offsets[cell + 1]] {
                    let point = &self.points[index];
                    let (dx, dy) = (point.x - qx, point.y - qy);
                    let distance_sq = dx * dx + dy * dy;
                    if distance_sq <= best_distance_sq {
                        best_distance_sq = distance_sq;
                        best = Some(index);
                    }
                }
            }
        }
        best
    }
}

fn huber_weight(residual: f64, delta: f64) -> f64 {
    let magnitude = residual.abs();
    if delta <= 0.0 || magnitude <= delta {
        1.0
    } else {
        delta / magnitude
    }
}

/// Estimates unit normals for a beam-ordered scan by local PCA.
///
/// A point gets `None` when fewer than two neighbors within `max_distance`
/// exist in its ±2 beam window, or when the neighborhood is not line-like.
pub fn estimate_scan_normals(
    points: &[Vector2<f64>],
    max_distance: f64,
) -> Vec<Option<Vector2<f64>>> {
    const WINDOW: usize = 2;
    let max_distance_sq = max_distance * max_distance;
    (0..points.len())
        .map(|i| {
            let start = i.saturating_sub(WINDOW);
            let end = (i + WINDOW + 1).min(points.len());
            let neighborhood: Vec<Vector2<f64>> = points[start..end]
                .iter()
                .filter(|p| (*p - points[i]).norm_squared() <= max_distance_sq)
                .copied()
                .collect();
            if neighborhood.len() < 3 {
                return None;
            }
            let count = neighborhood.len() as f64;
            let mean_x = neighborhood.iter().map(|p| p.x).sum::<f64>() / count;
            let mean_y = neighborhood.iter().map(|p| p.y).sum::<f64>() / count;
            let (mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0);
            for p in &neighborhood {
                let (dx, dy) = (p.x - mean_x, p.y - mean_y);
                sxx += dx * dx;
                sxy += dx * dy;
                syy += dy * dy;
            }
            // Closed-form eigen decomposition of the 2×2 scatter matrix.
            let half_trace = 0.5 * (sxx + syy);
            let spread = (0.25 * (sxx - syy) * (sxx - syy) + sxy * sxy).sqrt();
            let (large, small) = (half_trace + spread, half_trace - spread);
            if large <= 0.0 || small > 0.1 * large {
                return None;
            }
            // Principal (line) direction angle; the normal is perpendicular.
            let angle = 0.5 * (2.0 * sxy).atan2(sxx - syy);
            Some(Vector2::new(-angle.sin(), angle.cos()))
        })
        .collect()
}

/// Composes a body-frame delta onto `pose`: `pose ⊕ delta`.
pub fn compose_pose(pose: Pose2D, delta: Pose2D) -> Pose2D {
    let (sin, cos) = pose.yaw.sin_cos();
    Pose2D::new(
        pose.x + cos * delta.x - sin * delta.y,
        pose.y + sin * delta.x + cos * delta.y,
        wrap_angle(pose.yaw + delta.yaw),
    )
}

/// Body-frame delta that takes `from` to `to`: `from⁻¹ ⊕ to`.
pub fn relative_pose(from: Pose2D, to: Pose2D) -> Pose2D {
    let (sin, cos) = from.yaw.sin_cos();
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    Pose2D::new(
        cos * dx + sin * dy,
        -sin * dx + cos * dy,
        wrap_angle(to.yaw - from.yaw),
    )
}

/// Transforms body-frame scan points into the world frame at `pose`.
pub fn transform_scan_to_world(scan_body: &[Vector2<f64>], pose: Pose2D) -> Vec<Vector2<f64>> {
    let rotation = rotation(pose.yaw);
    let translation = Vector2::new(pose.x, pose.y);
    scan_body
        .iter()
        .map(|point| rotation * point + translation)
        .collect()
}

/// Ranges of `beam_count` beams spread uniformly over `[-π, π)` from `pose`.
///
/// Beams that hit nothing within `max_range` return `f64::INFINITY`.
pub fn ray_cast_ranges(
    pose: Pose2D,
    segments: &[LineSegment],
    beam_count: usize,
    max_range: f64,
) -> Vec<f64> {
    let origin = Vector2::new(pose.x, pose.y);
    (0..beam_count)
        .map(|beam| {
            let angle = pose.yaw + beam_angle(beam, beam_count);
            let direction = Vector2::new(angle.cos(), angle.sin());
            segments
                .iter()
                .filter_map(|segment| ray_segment_distance(origin, direction, segment))
                .filter(|range| *range <= max_range)
                .fold(f64::INFINITY, f64::min)
        })
        .collect()
}

/// Converts [`ray_cast_ranges`] output to body-frame points, skipping misses.
pub fn ranges_to_points(ranges: &[f64]) -> Vec<Vector2<f64>> {
    ranges
        .iter()
        .enumerate()
        .filter(|(_, range)| range.is_finite())
        .map(|(beam, range)| {
            let angle = beam_angle(beam, ranges.len());
            Vector2::new(range * angle.cos(), range * angle.sin())
        })
        .collect()
}

fn beam_angle(beam: usize, beam_count: usize) -> f64 {
    -PI + 2.0 * PI * beam as f64 / beam_count.max(1) as f64
}

fn ray_segment_distance(
    origin: Vector2<f64>,
    direction: Vector2<f64>,
    segment: &LineSegment,
) -> Option<f64> {
    let edge = segment.end - segment.start;
    let denominator = direction.perp(&edge);
    if denominator.abs() < 1.0e-12 {
        return None;
    }
    let offset = segment.start - origin;
    let range = offset.perp(&edge) / denominator;
    let along = offset.perp(&direction) / denominator;
    (range > 0.0 && (0.0..=1.0).contains(&along)).then_some(range)
}

fn voxel_downsample(points: &[Vector2<f64>], voxel_size: f64) -> Vec<Vector2<f64>> {
    if voxel_size <= 0.0 {
        return points.to_vec();
    }
    let mut occupied = HashMap::new();
    points
        .iter()
        .filter(|point| occupied.insert(voxel_key(point, voxel_size), ()).is_none())
        .copied()
        .collect()
}

fn voxel_key(point: &Vector2<f64>, voxel_size: f64) -> (i64, i64) {
    (
        (point.x / voxel_size).floor() as i64,
        (point.y / voxel_size).floor() as i64,
    )
}

fn rotation(yaw: f64) -> Matrix2<f64> {
    let (sin, cos) = yaw.sin_cos();
    Matrix2::new(cos, -sin, sin, cos)
}

fn wrap_angle(angle: f64) -> f64 {
    let wrapped = (angle + PI).rem_euclid(2.0 * PI) - PI;
    if wrapped <= -PI {
        wrapped + 2.0 * PI
    } else {
        wrapped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn room() -> Vec<LineSegment> {
        let corners = [
            Vector2::new(-6.0, -4.0),
            Vector2::new(6.0, -4.0),
            Vector2::new(6.0, 4.0),
            Vector2::new(-6.0, 4.0),
        ];
        let mut walls: Vec<LineSegment> = (0..4)
            .map(|i| LineSegment::new(corners[i], corners[(i + 1) % 4]))
            .collect();
        // An off-axis box and a stub wall break the room's symmetry.
        let box_corners = [
            Vector2::new(1.0, 1.0),
            Vector2::new(2.0, 1.4),
            Vector2::new(1.6, 2.4),
            Vector2::new(0.6, 2.0),
        ];
        walls.extend((0..4).map(|i| LineSegment::new(box_corners[i], box_corners[(i + 1) % 4])));
        walls.push(LineSegment::new(
            Vector2::new(-3.0, -4.0),
            Vector2::new(-3.0, -1.5),
        ));
        walls
    }

    fn scan_at(pose: Pose2D) -> Vec<Vector2<f64>> {
        ranges_to_points(&ray_cast_ranges(pose, &room(), 360, 15.0))
    }

    fn pose_error(a: Pose2D, b: Pose2D) -> (f64, f64) {
        let delta = relative_pose(a, b);
        (delta.x.hypot(delta.y), delta.yaw.abs())
    }

    #[test]
    fn compose_and_relative_are_inverse() {
        let a = Pose2D::new(1.0, -2.0, 0.7);
        let delta = Pose2D::new(0.3, 0.1, -0.2);
        let b = compose_pose(a, delta);
        let recovered = relative_pose(a, b);
        assert!((recovered.x - delta.x).abs() < 1e-12);
        assert!((recovered.y - delta.y).abs() < 1e-12);
        assert!((recovered.yaw - delta.yaw).abs() < 1e-12);
    }

    #[test]
    fn transform_scan_to_world_rotates_then_translates() {
        let world =
            transform_scan_to_world(&[Vector2::new(1.0, 0.0)], Pose2D::new(2.0, 3.0, PI / 2.0));
        assert!((world[0] - Vector2::new(2.0, 4.0)).norm() < 1e-12);
    }

    #[test]
    fn ray_cast_hits_room_walls() {
        let ranges = ray_cast_ranges(Pose2D::origin(), &room(), 4, 15.0);
        // Beams at -pi, -pi/2, 0, pi/2.
        assert!((ranges[0] - 6.0).abs() < 1e-9);
        assert!((ranges[1] - 4.0).abs() < 1e-9);
        assert!((ranges[2] - 6.0).abs() < 1e-9);
        assert!((ranges[3] - 4.0).abs() < 1e-9);
        // A beam toward the box is occluded before the +y wall.
        let toward_box = ray_cast_ranges(Pose2D::new(1.3, 0.0, PI / 2.0), &room(), 4, 15.0);
        assert!(toward_box[2] < 1.5);
    }

    #[test]
    fn normals_are_perpendicular_to_a_straight_wall() {
        let points: Vec<_> = (0..10).map(|i| Vector2::new(i as f64 * 0.1, 2.0)).collect();
        let normals = estimate_scan_normals(&points, 0.3);
        for normal in normals.iter().flatten() {
            assert!(normal.x.abs() < 1e-9 && (normal.y.abs() - 1.0).abs() < 1e-9);
        }
        assert!(normals.iter().all(Option::is_some));
    }

    #[test]
    fn first_update_bootstraps_submap() {
        let mut matcher = ScanToMapMatcher::new(ScanToMapConfig::default(), Pose2D::origin());
        let update = matcher.update(Pose2D::origin(), &scan_at(Pose2D::origin()));
        assert_eq!(update.status, MatchStatus::Bootstrap);
        assert!(update.inserted_keyframe);
        assert_eq!(matcher.keyframe_count(), 1);
        assert!(!matcher.submap_points().is_empty());
    }

    #[test]
    fn corrects_a_biased_odometry_step() {
        let mut matcher = ScanToMapMatcher::new(ScanToMapConfig::default(), Pose2D::origin());
        matcher.update(Pose2D::origin(), &scan_at(Pose2D::origin()));

        let truth = Pose2D::new(0.3, 0.05, 0.04);
        // Odometry overestimates the motion and misses part of the turn.
        let biased_delta = Pose2D::new(0.42, -0.03, 0.0);
        let update = matcher.update(biased_delta, &scan_at(truth));
        assert_eq!(update.status, MatchStatus::Accepted);
        let (predicted_xy, predicted_yaw) = pose_error(update.predicted_pose, truth);
        let (xy, yaw) = pose_error(update.corrected_pose, truth);
        assert!(predicted_xy > 0.1 && predicted_yaw > 0.03);
        assert!(xy < 5e-3, "translation error {xy}");
        assert!(yaw < 1e-3, "yaw error {yaw}");
    }

    #[test]
    fn rejects_corrections_beyond_the_gate() {
        let config = ScanToMapConfig {
            max_correction_translation: 0.05,
            ..ScanToMapConfig::default()
        };
        let mut matcher = ScanToMapMatcher::new(config, Pose2D::origin());
        matcher.update(Pose2D::origin(), &scan_at(Pose2D::origin()));
        let update = matcher.update(
            Pose2D::new(0.5, 0.0, 0.0),
            &scan_at(Pose2D::new(0.3, 0.0, 0.0)),
        );
        assert_eq!(update.status, MatchStatus::RejectedLargeCorrection);
        assert_eq!(update.corrected_pose, update.predicted_pose);
        assert!(!update.inserted_keyframe);
    }

    #[test]
    fn stationary_scans_do_not_grow_the_submap() {
        let mut matcher = ScanToMapMatcher::new(ScanToMapConfig::default(), Pose2D::origin());
        let scan = scan_at(Pose2D::origin());
        for _ in 0..5 {
            matcher.update(Pose2D::origin(), &scan);
        }
        assert_eq!(matcher.keyframe_count(), 1);
    }

    #[test]
    fn scan_to_scan_preset_keeps_one_scan() {
        let mut matcher = ScanToMapMatcher::new(ScanToMapConfig::scan_to_scan(), Pose2D::origin());
        let mut truth = Pose2D::origin();
        for _ in 0..4 {
            let delta = Pose2D::new(0.2, 0.0, 0.02);
            truth = compose_pose(truth, delta);
            matcher.update(delta, &scan_at(truth));
        }
        assert_eq!(matcher.keyframe_count(), 1);
        let (xy, yaw) = pose_error(matcher.pose(), truth);
        assert!(xy < 1e-2 && yaw < 1e-3);
    }

    #[test]
    fn submap_is_bounded_by_max_scans() {
        let config = ScanToMapConfig {
            max_scans: 3,
            ..ScanToMapConfig::default()
        };
        let mut matcher = ScanToMapMatcher::new(config, Pose2D::origin());
        let mut truth = Pose2D::origin();
        for _ in 0..8 {
            let delta = Pose2D::new(0.3, 0.0, 0.0);
            truth = compose_pose(truth, delta);
            matcher.update(delta, &scan_at(truth));
        }
        assert_eq!(matcher.keyframe_count(), 3);
    }
}
