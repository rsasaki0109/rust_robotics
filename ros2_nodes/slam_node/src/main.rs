use nalgebra::DMatrix;
use rust_robotics_mapping::occupancy_grid_map::{OccupancyGridConfig, OccupancyGridMap};
use rust_robotics_slam::{icp_matching::icp_matching, ICPResult};
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::{
        builtin_interfaces,
        common_interfaces::{geometry_msgs, nav_msgs, sensor_msgs, std_msgs},
    },
    pr_info, pr_warn,
};
use std::time::{Duration, Instant};
use std::{
    env,
    sync::{Arc, Mutex},
};

const ICP_INFO_LOG_INTERVAL: Duration = Duration::from_secs(5);
const DEFAULT_INPUT_ODOM_TOPIC: &str = "/odom";
const DEFAULT_MAP_FRAME_ID: &str = "map";
const DEFAULT_ODOM_FRAME_ID: &str = "odom";
const DEFAULT_BASE_FRAME_ID: &str = "base_footprint";
const DEFAULT_OUTPUT_POSE_TOPIC: &str = "/slam_pose";
const DEFAULT_OUTPUT_ODOM_TOPIC: &str = "/slam_odom";
const DEFAULT_DIAGNOSTICS_TOPIC: &str = "/slam_diagnostics";

/// Default ICP blend / gate tuning (override with `SLAM_ICP_*` environment variables).
/// Error gates use **mean** nearest-neighbor distance \[m/point\] from ICP (not sum over points).
const DEFAULT_ICP_BLEND_ALPHA: f64 = 0.35;
const DEFAULT_ICP_FULL_WEIGHT_ERROR: f64 = 0.007;
const DEFAULT_ICP_REJECT_ERROR: f64 = 0.011;
const DEFAULT_ICP_FULL_WEIGHT_ITERATIONS: f64 = 12.0;
const DEFAULT_ICP_REJECT_ITERATIONS: f64 = 40.0;
const DEFAULT_ICP_FULL_WEIGHT_TRANSLATION_CORRECTION: f64 = 0.05;
const DEFAULT_ICP_MAX_TRANSLATION_CORRECTION: f64 = 0.25;
const DEFAULT_ICP_FULL_WEIGHT_YAW_CORRECTION: f64 = 0.08;
const DEFAULT_ICP_MAX_YAW_CORRECTION: f64 = 0.35;
const DEFAULT_ICP_FULL_WEIGHT_TRANSLATION_MOTION: f64 = 0.05;
const DEFAULT_ICP_FULL_WEIGHT_YAW_MOTION: f64 = 0.08;

#[derive(Clone, Copy, Debug, PartialEq)]
struct IcpGatingParams {
    blend_alpha: f64,
    blend_alpha_yaw: f64,
    full_weight_error: f64,
    reject_error: f64,
    full_weight_error_yaw: f64,
    reject_error_yaw: f64,
    full_weight_iterations: f64,
    reject_iterations: f64,
    full_weight_translation_correction: f64,
    max_translation_correction: f64,
    full_weight_yaw_correction: f64,
    max_yaw_correction: f64,
    full_weight_translation_motion: f64,
    full_weight_yaw_motion: f64,
}

impl Default for IcpGatingParams {
    fn default() -> Self {
        Self {
            blend_alpha: DEFAULT_ICP_BLEND_ALPHA,
            blend_alpha_yaw: DEFAULT_ICP_BLEND_ALPHA,
            full_weight_error: DEFAULT_ICP_FULL_WEIGHT_ERROR,
            reject_error: DEFAULT_ICP_REJECT_ERROR,
            full_weight_error_yaw: DEFAULT_ICP_FULL_WEIGHT_ERROR,
            reject_error_yaw: DEFAULT_ICP_REJECT_ERROR,
            full_weight_iterations: DEFAULT_ICP_FULL_WEIGHT_ITERATIONS,
            reject_iterations: DEFAULT_ICP_REJECT_ITERATIONS,
            full_weight_translation_correction: DEFAULT_ICP_FULL_WEIGHT_TRANSLATION_CORRECTION,
            max_translation_correction: DEFAULT_ICP_MAX_TRANSLATION_CORRECTION,
            full_weight_yaw_correction: DEFAULT_ICP_FULL_WEIGHT_YAW_CORRECTION,
            max_yaw_correction: DEFAULT_ICP_MAX_YAW_CORRECTION,
            full_weight_translation_motion: DEFAULT_ICP_FULL_WEIGHT_TRANSLATION_MOTION,
            full_weight_yaw_motion: DEFAULT_ICP_FULL_WEIGHT_YAW_MOTION,
        }
    }
}

#[derive(Clone, Copy)]
struct Pose2D {
    x: f64,
    y: f64,
    yaw: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct MotionDelta {
    x: f64,
    y: f64,
    yaw: f64,
}

#[derive(Clone)]
struct OdomMeasurement {
    pose: Pose2D,
    linear_x: f64,
    angular_z: f64,
    frame_id: String,
    child_frame_id: String,
    received: bool,
}

struct SlamState {
    raw_odom: OdomMeasurement,
    corrected_pose: Pose2D,
    corrected_initialized: bool,
    map: OccupancyGridMap,
    previous_scan: Option<DMatrix<f64>>,
    previous_raw_odom_at_scan: Option<Pose2D>,
    last_icp_log_at: Option<Instant>,
    /// Accumulated local point cloud in the corrected world frame. Only
    /// populated when `SLAM_ICP_MODE=scan_to_map`.
    local_submap_world: Option<DMatrix<f64>>,
    /// Number of scans appended into `local_submap_world` since startup;
    /// gates the bootstrap before scan-to-map ICP is allowed to run.
    local_submap_scan_count: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum IcpMode {
    ScanToScan,
    ScanToMap,
}

impl IcpMode {
    fn as_str(self) -> &'static str {
        match self {
            IcpMode::ScanToScan => "scan_to_scan",
            IcpMode::ScanToMap => "scan_to_map",
        }
    }
}

/// Submap tuning loaded from env at startup. Holds capacity bounds plus the
/// bootstrap-scan count that gates scan-to-map ICP off until the submap has
/// enough material to match against.
#[derive(Clone, Copy, Debug)]
struct SubmapParams {
    budget: SubmapBudget,
    bootstrap_scans: usize,
}

#[derive(Clone, Copy)]
struct ScanDiagnostics {
    scan_points: usize,
    had_previous_scan: bool,
    icp_converged: bool,
    icp_iterations: usize,
    icp_final_error: f64,
    icp_initial_error: f64,
    icp_error_median: f64,
    icp_error_p90: f64,
    icp_inlier_ratio_5cm: f64,
    icp_relative_error_reduction: f64,
    odom_delta: Option<MotionDelta>,
    icp_delta: Option<MotionDelta>,
    applied_delta: Option<MotionDelta>,
    blend_applied: bool,
    blend_alpha: f64,
    blend_alpha_yaw: f64,
    gate_reason: &'static str,
    gate_reason_yaw: &'static str,
    /// Which ICP target was matched against on this scan. Always reported so
    /// downstream summarizers can split rows by mode without parsing env.
    icp_mode: &'static str,
    /// Submap point count after this scan was processed. 0 under scan_to_scan.
    submap_points: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct IcpBlendDecision {
    /// XY-axis blend weight (0..1). Backward-compatible field name `alpha` keeps existing call sites
    /// and tests working — the XY axis is the primary blend.
    alpha: f64,
    /// Gate reason for the XY axis. Same string set as before.
    reason: &'static str,
    /// Yaw-axis blend weight (0..1). When yaw env overrides are unset this equals `alpha`.
    alpha_yaw: f64,
    /// Gate reason for the yaw axis. When yaw env overrides are unset this equals `reason`.
    reason_yaw: &'static str,
}

impl IcpBlendDecision {
    fn all_rejected(reason: &'static str) -> Self {
        Self {
            alpha: 0.0,
            reason,
            alpha_yaw: 0.0,
            reason_yaw: reason,
        }
    }
}

fn yaw_from_quaternion(x: f64, y: f64, z: f64, w: f64) -> f64 {
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    siny_cosp.atan2(cosy_cosp)
}

fn scan_to_point_matrix(scan: &sensor_msgs::msg::LaserScan) -> Option<DMatrix<f64>> {
    let mut points = Vec::<(f64, f64)>::new();
    let mut angle = scan.angle_min as f64;

    for range in scan.ranges.iter() {
        let r = *range as f64;
        if r.is_finite() && r > scan.range_min as f64 && r < scan.range_max as f64 {
            points.push((r * angle.cos(), r * angle.sin()));
        }
        angle += scan.angle_increment as f64;
    }

    if points.is_empty() {
        return None;
    }

    let mut m = DMatrix::zeros(2, points.len());
    for (i, (x, y)) in points.iter().enumerate() {
        m[(0, i)] = *x;
        m[(1, i)] = *y;
    }
    Some(m)
}

/// Keep every `stride`-th column for ICP (1 = full resolution). Reduces noise and cost when `stride > 1`.
fn subsample_points_for_icp(m: &DMatrix<f64>, stride: usize) -> DMatrix<f64> {
    if stride <= 1 || m.ncols() == 0 {
        return m.clone();
    }
    let idx: Vec<usize> = (0..m.ncols()).step_by(stride).collect();
    const MIN_ICP_POINTS: usize = 4;
    if idx.len() < MIN_ICP_POINTS {
        return m.clone();
    }
    let mut out = DMatrix::zeros(2, idx.len());
    for (j, &i) in idx.iter().enumerate() {
        out[(0, j)] = m[(0, i)];
        out[(1, j)] = m[(1, i)];
    }
    out
}

fn icp_point_stride_from_env() -> usize {
    let s = env::var("SLAM_ICP_POINT_STRIDE")
        .ok()
        .and_then(|raw| raw.trim().parse::<usize>().ok())
        .unwrap_or(1);
    s.clamp(1, 16)
}

/// Read `SLAM_ICP_MODE`. Default is scan-to-scan so the existing smoke and
/// matrix paths behave identically when the flag is unset.
fn icp_mode_from_env() -> IcpMode {
    match env::var("SLAM_ICP_MODE").ok().as_deref() {
        Some(raw) => match raw.trim().to_ascii_lowercase().as_str() {
            "scan_to_map" | "scan-to-map" | "scantomap" => IcpMode::ScanToMap,
            _ => IcpMode::ScanToScan,
        },
        None => IcpMode::ScanToScan,
    }
}

const DEFAULT_SUBMAP_MAX_POINTS: usize = 6_400;
const DEFAULT_SUBMAP_RADIUS_M: f64 = 3.0;
const DEFAULT_SUBMAP_BOOTSTRAP_SCANS: usize = 3;

fn submap_params_from_env() -> SubmapParams {
    let max_points = env::var("SLAM_SUBMAP_MAX_POINTS")
        .ok()
        .and_then(|raw| raw.trim().parse::<usize>().ok())
        .unwrap_or(DEFAULT_SUBMAP_MAX_POINTS);
    let max_radius_m = env::var("SLAM_SUBMAP_RADIUS_M")
        .ok()
        .and_then(|raw| raw.trim().parse::<f64>().ok())
        .filter(|v| v.is_finite() && *v > 0.0)
        .unwrap_or(DEFAULT_SUBMAP_RADIUS_M);
    let bootstrap_scans = env::var("SLAM_SUBMAP_BOOTSTRAP_SCANS")
        .ok()
        .and_then(|raw| raw.trim().parse::<usize>().ok())
        .unwrap_or(DEFAULT_SUBMAP_BOOTSTRAP_SCANS);
    SubmapParams {
        budget: SubmapBudget {
            max_points,
            max_radius_m,
        },
        bootstrap_scans,
    }
}

fn copy_stamp(
    target: &mut builtin_interfaces::UnsafeTime,
    source: &builtin_interfaces::UnsafeTime,
) {
    target.sec = source.sec;
    target.nanosec = source.nanosec;
}

fn topic_from_env(key: &str, default: &str) -> String {
    env::var(key)
        .ok()
        .filter(|value| !value.trim().is_empty())
        .unwrap_or_else(|| default.to_string())
}

fn bool_from_env(key: &str, default: bool) -> bool {
    match env::var(key) {
        Ok(raw) if !raw.trim().is_empty() => {
            let value = raw.trim().to_ascii_lowercase();
            matches!(value.as_str(), "1" | "true" | "yes" | "on")
        }
        _ => default,
    }
}

fn f64_from_env(key: &str, default: f64) -> f64 {
    env::var(key)
        .ok()
        .and_then(|raw| raw.trim().parse::<f64>().ok())
        .filter(|v| v.is_finite())
        .unwrap_or(default)
}

fn clamp_f64(v: f64, min: f64, max: f64) -> f64 {
    v.clamp(min, max)
}

/// Loads [`IcpGatingParams`] from `SLAM_ICP_*` env vars (unset → defaults from [`IcpGatingParams::default`]).
///
/// The yaw-axis error and blend-alpha settings default to the XY values when their
/// `SLAM_ICP_*_YAW` overrides are unset, preserving prior single-gate behavior.
fn icp_gating_params_from_env() -> IcpGatingParams {
    let mut p = IcpGatingParams::default();
    p.blend_alpha = clamp_f64(
        f64_from_env("SLAM_ICP_BLEND_ALPHA", p.blend_alpha),
        0.0,
        1.0,
    );
    p.blend_alpha_yaw = clamp_f64(
        f64_from_env("SLAM_ICP_BLEND_ALPHA_YAW", p.blend_alpha),
        0.0,
        1.0,
    );
    p.full_weight_error = f64_from_env("SLAM_ICP_FULL_WEIGHT_ERROR", p.full_weight_error).max(1e-6);
    p.reject_error = f64_from_env("SLAM_ICP_REJECT_ERROR", p.reject_error).max(p.full_weight_error + 1e-6);
    p.full_weight_error_yaw =
        f64_from_env("SLAM_ICP_FULL_WEIGHT_ERROR_YAW", p.full_weight_error).max(1e-6);
    p.reject_error_yaw = f64_from_env("SLAM_ICP_REJECT_ERROR_YAW", p.reject_error)
        .max(p.full_weight_error_yaw + 1e-6);
    p.full_weight_iterations = f64_from_env("SLAM_ICP_FULL_WEIGHT_ITERATIONS", p.full_weight_iterations).max(0.0);
    p.reject_iterations = f64_from_env("SLAM_ICP_REJECT_ITERATIONS", p.reject_iterations).max(p.full_weight_iterations + 1e-6);
    p.full_weight_translation_correction = f64_from_env(
        "SLAM_ICP_FULL_WEIGHT_TRANSLATION_CORRECTION",
        p.full_weight_translation_correction,
    )
    .max(0.0);
    p.max_translation_correction = f64_from_env(
        "SLAM_ICP_MAX_TRANSLATION_CORRECTION",
        p.max_translation_correction,
    )
    .max(p.full_weight_translation_correction + 1e-6);
    p.full_weight_yaw_correction = f64_from_env(
        "SLAM_ICP_FULL_WEIGHT_YAW_CORRECTION",
        p.full_weight_yaw_correction,
    )
    .max(0.0);
    p.max_yaw_correction =
        f64_from_env("SLAM_ICP_MAX_YAW_CORRECTION", p.max_yaw_correction).max(p.full_weight_yaw_correction + 1e-6);
    p.full_weight_translation_motion = f64_from_env(
        "SLAM_ICP_FULL_WEIGHT_TRANSLATION_MOTION",
        p.full_weight_translation_motion,
    )
    .max(0.0);
    p.full_weight_yaw_motion =
        f64_from_env("SLAM_ICP_FULL_WEIGHT_YAW_MOTION", p.full_weight_yaw_motion).max(0.0);
    p
}

fn frame_or_default(frame_id: String, default: &str) -> String {
    if frame_id.is_empty() {
        default.to_string()
    } else {
        frame_id
    }
}

fn odom_measurement_from_msg(msg: &nav_msgs::msg::Odometry) -> OdomMeasurement {
    let q = &msg.pose.pose.orientation;
    OdomMeasurement {
        pose: Pose2D {
            x: msg.pose.pose.position.x,
            y: msg.pose.pose.position.y,
            yaw: yaw_from_quaternion(q.x, q.y, q.z, q.w),
        },
        linear_x: msg.twist.twist.linear.x,
        angular_z: msg.twist.twist.angular.z,
        frame_id: frame_or_default(msg.header.frame_id.get_string(), DEFAULT_ODOM_FRAME_ID),
        child_frame_id: frame_or_default(msg.child_frame_id.get_string(), DEFAULT_BASE_FRAME_ID),
        received: true,
    }
}

fn normalize_angle(mut angle: f64) -> f64 {
    while angle > std::f64::consts::PI {
        angle -= std::f64::consts::TAU;
    }
    while angle < -std::f64::consts::PI {
        angle += std::f64::consts::TAU;
    }
    angle
}

fn relative_motion_local(previous: Pose2D, current: Pose2D) -> MotionDelta {
    let dx_world = current.x - previous.x;
    let dy_world = current.y - previous.y;
    let cos_yaw = previous.yaw.cos();
    let sin_yaw = previous.yaw.sin();

    MotionDelta {
        x: cos_yaw * dx_world + sin_yaw * dy_world,
        y: -sin_yaw * dx_world + cos_yaw * dy_world,
        yaw: normalize_angle(current.yaw - previous.yaw),
    }
}

fn compose_pose_local(previous: Pose2D, delta: MotionDelta) -> Pose2D {
    let cos_yaw = previous.yaw.cos();
    let sin_yaw = previous.yaw.sin();
    Pose2D {
        x: previous.x + cos_yaw * delta.x - sin_yaw * delta.y,
        y: previous.y + sin_yaw * delta.x + cos_yaw * delta.y,
        yaw: normalize_angle(previous.yaw + delta.yaw),
    }
}

fn icp_motion_delta(result: &ICPResult) -> Option<MotionDelta> {
    if result.rotation.nrows() < 2 || result.rotation.ncols() < 2 || result.translation.nrows() < 2
    {
        return None;
    }

    let x = result.translation[0];
    let y = result.translation[1];
    let yaw = result.rotation[(1, 0)].atan2(result.rotation[(0, 0)]);
    if !x.is_finite() || !y.is_finite() || !yaw.is_finite() {
        return None;
    }

    Some(MotionDelta {
        x,
        y,
        yaw: normalize_angle(yaw),
    })
}

fn translation_magnitude(delta: MotionDelta) -> f64 {
    delta.x.hypot(delta.y)
}

/// Rotate-and-translate a 2×N point cloud from body frame at `pose` into the world frame.
///
/// Each column is a 2D point `(x, y)` in `pose`'s body frame. Returns a fresh
/// `2×N` matrix of the same points expressed in the world frame.
fn transform_scan_to_world(scan_body: &DMatrix<f64>, pose: Pose2D) -> DMatrix<f64> {
    if scan_body.nrows() < 2 || scan_body.ncols() == 0 {
        return DMatrix::zeros(scan_body.nrows().max(2), scan_body.ncols());
    }
    let cos_yaw = pose.yaw.cos();
    let sin_yaw = pose.yaw.sin();
    let mut out = DMatrix::zeros(2, scan_body.ncols());
    for c in 0..scan_body.ncols() {
        let bx = scan_body[(0, c)];
        let by = scan_body[(1, c)];
        out[(0, c)] = cos_yaw * bx - sin_yaw * by + pose.x;
        out[(1, c)] = sin_yaw * bx + cos_yaw * by + pose.y;
    }
    out
}

/// Apply a world-frame rigid residual (2D rotation + translation) on top of a predicted pose.
///
/// Interpreted as: rotate the predicted position about the world origin by
/// `residual.yaw`, then translate by `(residual.x, residual.y)`, and add
/// `residual.yaw` to the predicted yaw. This is the inverse mapping of how
/// scan-to-map ICP returns its residual (it rotates *points* in world frame).
fn apply_world_residual(predicted: Pose2D, residual_world: MotionDelta) -> Pose2D {
    let cos_yaw = residual_world.yaw.cos();
    let sin_yaw = residual_world.yaw.sin();
    Pose2D {
        x: cos_yaw * predicted.x - sin_yaw * predicted.y + residual_world.x,
        y: sin_yaw * predicted.x + cos_yaw * predicted.y + residual_world.y,
        yaw: normalize_angle(predicted.yaw + residual_world.yaw),
    }
}

/// Bound on the local submap used by scan-to-map ICP.
///
/// `max_points` caps point count so ICP cost stays predictable; `max_radius_m`
/// is the L2 radius around `anchor` outside of which points are pruned, so the
/// submap stays a *local* reference as the robot drives.
#[derive(Clone, Copy, Debug)]
struct SubmapBudget {
    max_points: usize,
    max_radius_m: f64,
}

/// Append a new world-frame scan into the submap and prune to the budget.
///
/// Returns the merged + pruned submap. Pruning rule: drop any point whose L2
/// distance from `(anchor.x, anchor.y)` exceeds `budget.max_radius_m`; then,
/// if the result still exceeds `budget.max_points`, keep only the most
/// recently appended `budget.max_points` columns (newest-first).
#[allow(dead_code)] // submap maintenance is hooked up in Phase 2.
fn append_and_prune(
    existing: Option<DMatrix<f64>>,
    new_world: &DMatrix<f64>,
    anchor: Pose2D,
    budget: SubmapBudget,
) -> DMatrix<f64> {
    let new_cols = if new_world.nrows() >= 2 {
        new_world.ncols()
    } else {
        0
    };
    let existing_cols = match existing.as_ref() {
        Some(m) if m.nrows() >= 2 => m.ncols(),
        _ => 0,
    };
    let total = existing_cols + new_cols;
    if total == 0 {
        return DMatrix::zeros(2, 0);
    }

    let mut combined = DMatrix::zeros(2, total);
    if let Some(m) = existing.as_ref() {
        for c in 0..existing_cols {
            combined[(0, c)] = m[(0, c)];
            combined[(1, c)] = m[(1, c)];
        }
    }
    for c in 0..new_cols {
        combined[(0, existing_cols + c)] = new_world[(0, c)];
        combined[(1, existing_cols + c)] = new_world[(1, c)];
    }

    // Radius prune around anchor.
    let r2 = budget.max_radius_m * budget.max_radius_m;
    let mut keep_idx: Vec<usize> = Vec::with_capacity(combined.ncols());
    for c in 0..combined.ncols() {
        let dx = combined[(0, c)] - anchor.x;
        let dy = combined[(1, c)] - anchor.y;
        if dx * dx + dy * dy <= r2 {
            keep_idx.push(c);
        }
    }

    // Cap to max_points by keeping the newest tail of survivors.
    if budget.max_points > 0 && keep_idx.len() > budget.max_points {
        let drop = keep_idx.len() - budget.max_points;
        keep_idx.drain(0..drop);
    }

    let mut out = DMatrix::zeros(2, keep_idx.len());
    for (j, &i) in keep_idx.iter().enumerate() {
        out[(0, j)] = combined[(0, i)];
        out[(1, j)] = combined[(1, i)];
    }
    out
}

fn ramp_weight(value: f64, full_weight_limit: f64, reject_limit: f64) -> f64 {
    if value <= full_weight_limit {
        1.0
    } else if value >= reject_limit {
        0.0
    } else {
        1.0 - (value - full_weight_limit) / (reject_limit - full_weight_limit)
    }
}

fn ramp_up_weight(value: f64, reject_limit: f64, full_weight_limit: f64) -> f64 {
    if value <= reject_limit {
        0.0
    } else if value >= full_weight_limit {
        1.0
    } else {
        (value - reject_limit) / (full_weight_limit - reject_limit)
    }
}

fn compute_icp_blend_decision(
    odom: MotionDelta,
    icp: MotionDelta,
    converged: bool,
    iterations: usize,
    final_error: f64,
    p: &IcpGatingParams,
) -> IcpBlendDecision {
    if !converged {
        return IcpBlendDecision::all_rejected("not_converged");
    }
    if !final_error.is_finite() {
        return IcpBlendDecision::all_rejected("invalid_error");
    }

    let correction = MotionDelta {
        x: icp.x - odom.x,
        y: icp.y - odom.y,
        yaw: normalize_angle(icp.yaw - odom.yaw),
    };
    let correction_translation = translation_magnitude(correction);
    let correction_yaw = correction.yaw.abs();

    // Iterations gate is shared (it reflects ICP solver health, not per-axis quality).
    let iteration_weight = ramp_weight(
        iterations as f64,
        p.full_weight_iterations,
        p.reject_iterations,
    );

    // Motion gate is split: XY axis only allows blend when robot actually translates;
    // yaw axis allows blend when either yaw or translation motion is present.
    let translation_motion_weight = ramp_up_weight(
        translation_magnitude(odom),
        p.full_weight_translation_motion * 0.25,
        p.full_weight_translation_motion,
    );
    let yaw_motion_weight = ramp_up_weight(
        odom.yaw.abs(),
        p.full_weight_yaw_motion * 0.25,
        p.full_weight_yaw_motion,
    );
    let yaw_axis_motion_weight = translation_motion_weight.max(yaw_motion_weight);

    let correction_translation_weight = ramp_weight(
        correction_translation,
        p.full_weight_translation_correction,
        p.max_translation_correction,
    );
    let correction_yaw_weight = ramp_weight(
        correction_yaw,
        p.full_weight_yaw_correction,
        p.max_yaw_correction,
    );

    let (alpha_xy, reason_xy) = compute_axis_decision(AxisInputs {
        base_alpha: p.blend_alpha,
        final_error,
        full_weight_error: p.full_weight_error,
        reject_error: p.reject_error,
        iteration_weight,
        correction_size: correction_translation,
        max_correction: p.max_translation_correction,
        correction_weight: correction_translation_weight,
        motion_weight: translation_motion_weight,
        outlier_reason: "translation_outlier",
        attenuated_correction_reason: "attenuated_translation",
    });

    let (alpha_yaw, reason_yaw) = compute_axis_decision(AxisInputs {
        base_alpha: p.blend_alpha_yaw,
        final_error,
        full_weight_error: p.full_weight_error_yaw,
        reject_error: p.reject_error_yaw,
        iteration_weight,
        correction_size: correction_yaw,
        max_correction: p.max_yaw_correction,
        correction_weight: correction_yaw_weight,
        motion_weight: yaw_axis_motion_weight,
        outlier_reason: "yaw_outlier",
        attenuated_correction_reason: "attenuated_yaw",
    });

    IcpBlendDecision {
        alpha: alpha_xy,
        reason: reason_xy,
        alpha_yaw,
        reason_yaw,
    }
}

struct AxisInputs {
    base_alpha: f64,
    final_error: f64,
    full_weight_error: f64,
    reject_error: f64,
    iteration_weight: f64,
    correction_size: f64,
    max_correction: f64,
    correction_weight: f64,
    motion_weight: f64,
    outlier_reason: &'static str,
    attenuated_correction_reason: &'static str,
}

fn compute_axis_decision(a: AxisInputs) -> (f64, &'static str) {
    if a.correction_size >= a.max_correction {
        return (0.0, a.outlier_reason);
    }
    let error_weight = ramp_weight(a.final_error, a.full_weight_error, a.reject_error);
    if error_weight <= 0.0 {
        return (0.0, "high_error");
    }
    if a.iteration_weight <= 0.0 {
        return (0.0, "slow_convergence");
    }
    if a.correction_weight <= 0.0 {
        return (0.0, a.outlier_reason);
    }
    if a.motion_weight <= 0.0 {
        return (0.0, "low_motion");
    }
    let scale = error_weight
        .min(a.iteration_weight)
        .min(a.correction_weight)
        .min(a.motion_weight);
    let alpha = a.base_alpha * scale;
    if alpha <= 0.0 {
        return (0.0, "rejected");
    }
    let reason = if scale < 0.999 {
        if scale == a.motion_weight {
            "attenuated_low_motion"
        } else if scale == error_weight {
            "attenuated_error"
        } else if scale == a.iteration_weight {
            "attenuated_iterations"
        } else {
            a.attenuated_correction_reason
        }
    } else {
        "accepted"
    };
    (alpha, reason)
}

fn blend_motion_delta(
    odom: MotionDelta,
    icp: MotionDelta,
    alpha_xy: f64,
    alpha_yaw: f64,
    p: &IcpGatingParams,
) -> MotionDelta {
    let correction_x = (icp.x - odom.x).clamp(
        -p.max_translation_correction,
        p.max_translation_correction,
    );
    let correction_y = (icp.y - odom.y).clamp(
        -p.max_translation_correction,
        p.max_translation_correction,
    );
    let correction_yaw =
        normalize_angle(icp.yaw - odom.yaw).clamp(-p.max_yaw_correction, p.max_yaw_correction);

    MotionDelta {
        x: odom.x + alpha_xy * correction_x,
        y: odom.y + alpha_xy * correction_y,
        yaw: normalize_angle(odom.yaw + alpha_yaw * correction_yaw),
    }
}

fn apply_yaw_to_pose(pose: &mut geometry_msgs::msg::Pose, yaw: f64) {
    let half_yaw = 0.5 * yaw;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = half_yaw.sin();
    pose.orientation.w = half_yaw.cos();
}

fn push_delta_fields(fields: &mut Vec<String>, prefix: &str, delta: Option<MotionDelta>) {
    match delta {
        Some(delta) => {
            fields.push(format!("{}_dx={:.3}", prefix, delta.x));
            fields.push(format!("{}_dy={:.3}", prefix, delta.y));
            fields.push(format!("{}_dyaw={:.3}", prefix, delta.yaw));
        }
        None => {
            fields.push(format!("{}_dx=na", prefix));
            fields.push(format!("{}_dy=na", prefix));
            fields.push(format!("{}_dyaw=na", prefix));
        }
    }
}

fn diagnostics_status(diagnostics: ScanDiagnostics, base_blend_alpha: f64) -> &'static str {
    if !diagnostics.had_previous_scan {
        "bootstrap"
    } else if diagnostics.blend_applied {
        let xy_at_full = diagnostics.blend_alpha + 1e-6 >= base_blend_alpha;
        let yaw_at_full = diagnostics.blend_alpha_yaw + 1e-6 >= base_blend_alpha;
        if xy_at_full && yaw_at_full {
            "icp_ok"
        } else {
            "icp_attenuated"
        }
    } else if diagnostics.gate_reason == "missing_icp_delta"
        || diagnostics.gate_reason == "syncing_to_odom"
    {
        "odom_only"
    } else {
        "icp_rejected"
    }
}

fn format_slam_diagnostics(
    frame_id: &str,
    raw_frame_id: &str,
    raw_pose: Pose2D,
    estimate_pose: Pose2D,
    corrected_mode: bool,
    diagnostics: ScanDiagnostics,
    base_blend_alpha: f64,
) -> String {
    let mut fields = vec![
        format!("status={}", diagnostics_status(diagnostics, base_blend_alpha)),
        format!("frame={}", frame_id),
        format!("raw_frame={}", raw_frame_id),
        format!("corrected_mode={}", corrected_mode),
        format!("scan_points={}", diagnostics.scan_points),
        format!("icp_mode={}", diagnostics.icp_mode),
        format!("submap_points={}", diagnostics.submap_points),
        format!("icp_converged={}", diagnostics.icp_converged),
        format!("icp_iterations={}", diagnostics.icp_iterations),
        format!("icp_error_mean={:.4}", diagnostics.icp_final_error),
        format!("icp_initial_error_mean={:.4}", diagnostics.icp_initial_error),
        format!("icp_error_median={:.4}", diagnostics.icp_error_median),
        format!("icp_error_p90={:.4}", diagnostics.icp_error_p90),
        format!("icp_inlier_ratio_5cm={:.3}", diagnostics.icp_inlier_ratio_5cm),
        format!(
            "icp_relative_error_reduction={:.3}",
            diagnostics.icp_relative_error_reduction
        ),
        format!("blend_applied={}", diagnostics.blend_applied),
        format!("blend_alpha={:.3}", diagnostics.blend_alpha),
        format!("blend_alpha_yaw={:.3}", diagnostics.blend_alpha_yaw),
        format!("gate_reason={}", diagnostics.gate_reason),
        format!("gate_reason_yaw={}", diagnostics.gate_reason_yaw),
        format!("raw_x={:.3}", raw_pose.x),
        format!("raw_y={:.3}", raw_pose.y),
        format!("raw_yaw={:.3}", raw_pose.yaw),
        format!("estimate_x={:.3}", estimate_pose.x),
        format!("estimate_y={:.3}", estimate_pose.y),
        format!("estimate_yaw={:.3}", estimate_pose.yaw),
    ];
    push_delta_fields(&mut fields, "odom", diagnostics.odom_delta);
    push_delta_fields(&mut fields, "icp", diagnostics.icp_delta);
    push_delta_fields(&mut fields, "applied", diagnostics.applied_delta);
    fields.join(" ")
}

fn build_occupancy_grid_msg(
    map: &OccupancyGridMap,
    frame_id: &str,
    stamp: &builtin_interfaces::UnsafeTime,
) -> Result<nav_msgs::msg::OccupancyGrid, DynError> {
    let mut msg = nav_msgs::msg::OccupancyGrid::new().ok_or("failed to allocate OccupancyGrid")?;
    let _ = msg.header.frame_id.assign(frame_id);
    copy_stamp(&mut msg.header.stamp, stamp);

    msg.info.resolution = map.config.resolution as f32;
    msg.info.width = map.config.width as u32;
    msg.info.height = map.config.height as u32;
    msg.info.origin.position.x = -(map.config.width as f64) * map.config.resolution * 0.5;
    msg.info.origin.position.y = -(map.config.height as f64) * map.config.resolution * 0.5;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    let total_cells = map.config.width * map.config.height;
    let mut data =
        safe_drive::msg::I8Seq::<0>::new(total_cells).ok_or("failed to allocate map data")?;
    for ix in 0..map.config.width {
        for iy in 0..map.config.height {
            let p = map.get_probability(ix, iy);
            let value = (p * 100.0).clamp(0.0, 100.0) as i8;
            data.as_slice_mut()[iy * map.config.width + ix] = value;
        }
    }

    msg.data = data;
    Ok(msg)
}

fn publish_corrected_pose(
    publisher: &safe_drive::topic::publisher::Publisher<geometry_msgs::msg::PoseStamped>,
    pose: Pose2D,
    frame_id: &str,
    stamp: &builtin_interfaces::UnsafeTime,
) -> Result<(), DynError> {
    let mut msg = geometry_msgs::msg::PoseStamped::new().ok_or("failed to allocate PoseStamped")?;
    let _ = msg.header.frame_id.assign(frame_id);
    copy_stamp(&mut msg.header.stamp, stamp);
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;
    msg.pose.position.z = 0.0;
    apply_yaw_to_pose(&mut msg.pose, pose.yaw);
    publisher.send(&msg)?;
    Ok(())
}

fn publish_corrected_odom(
    publisher: &safe_drive::topic::publisher::Publisher<nav_msgs::msg::Odometry>,
    pose: Pose2D,
    odom: &OdomMeasurement,
    frame_id: &str,
    stamp: &builtin_interfaces::UnsafeTime,
) -> Result<(), DynError> {
    let mut msg = nav_msgs::msg::Odometry::new().ok_or("failed to allocate Odometry")?;
    let _ = msg.header.frame_id.assign(frame_id);
    let _ = msg.child_frame_id.assign(&odom.child_frame_id);
    copy_stamp(&mut msg.header.stamp, stamp);
    msg.pose.pose.position.x = pose.x;
    msg.pose.pose.position.y = pose.y;
    msg.pose.pose.position.z = 0.0;
    apply_yaw_to_pose(&mut msg.pose.pose, pose.yaw);
    msg.twist.twist.linear.x = odom.linear_x;
    msg.twist.twist.angular.z = odom.angular_z;
    publisher.send(&msg)?;
    Ok(())
}

fn publish_diagnostics(
    publisher: &safe_drive::topic::publisher::Publisher<std_msgs::msg::String>,
    status: &str,
) -> Result<(), DynError> {
    let mut msg = std_msgs::msg::String::new().ok_or("failed to allocate String")?;
    let _ = msg.data.assign(status);
    publisher.send(&msg)?;
    Ok(())
}

fn main() -> Result<(), DynError> {
    let input_odom_topic = topic_from_env("SLAM_INPUT_ODOM_TOPIC", DEFAULT_INPUT_ODOM_TOPIC);
    let output_pose_topic = topic_from_env("SLAM_OUTPUT_POSE_TOPIC", DEFAULT_OUTPUT_POSE_TOPIC);
    let output_odom_topic = topic_from_env("SLAM_OUTPUT_ODOM_TOPIC", DEFAULT_OUTPUT_ODOM_TOPIC);
    let diagnostics_topic = topic_from_env("SLAM_DIAGNOSTICS_TOPIC", DEFAULT_DIAGNOSTICS_TOPIC);
    let corrected_frame_id = topic_from_env("SLAM_CORRECTED_FRAME_ID", DEFAULT_MAP_FRAME_ID);
    let use_corrected_frame = bool_from_env("SLAM_USE_CORRECTED_FRAME", false);
    let icp_params = icp_gating_params_from_env();

    let ctx = Context::new()?;
    let node = ctx.create_node("slam_node", None, Default::default())?;

    let scan_sub = node.create_subscriber::<sensor_msgs::msg::LaserScan>("/scan", None)?;
    let odom_sub = node.create_subscriber::<nav_msgs::msg::Odometry>(&input_odom_topic, None)?;
    let map_pub = node.create_publisher::<nav_msgs::msg::OccupancyGrid>("/map", None)?;
    let corrected_pose_pub =
        node.create_publisher::<geometry_msgs::msg::PoseStamped>(&output_pose_topic, None)?;
    let corrected_odom_pub =
        node.create_publisher::<nav_msgs::msg::Odometry>(&output_odom_topic, None)?;
    let diagnostics_pub =
        node.create_publisher::<std_msgs::msg::String>(&diagnostics_topic, None)?;

    let map_config = OccupancyGridConfig {
        resolution: 0.2,
        width: 300,
        height: 300,
        ..Default::default()
    };

    let state = Arc::new(Mutex::new(SlamState {
        raw_odom: OdomMeasurement {
            pose: Pose2D {
                x: 0.0,
                y: 0.0,
                yaw: 0.0,
            },
            linear_x: 0.0,
            angular_z: 0.0,
            frame_id: DEFAULT_ODOM_FRAME_ID.to_string(),
            child_frame_id: DEFAULT_BASE_FRAME_ID.to_string(),
            received: false,
        },
        corrected_pose: Pose2D {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
        },
        corrected_initialized: false,
        map: OccupancyGridMap::new(map_config),
        previous_scan: None,
        previous_raw_odom_at_scan: None,
        last_icp_log_at: None,
        local_submap_world: None,
        local_submap_scan_count: 0,
    }));

    let logger = Logger::new("slam_node");
    pr_info!(
        logger,
        "slam node started (odom topic: {}, corrected_frame: {}, corrected_mode: {})",
        input_odom_topic,
        corrected_frame_id,
        use_corrected_frame
    );
    if use_corrected_frame {
        pr_info!(
            logger,
            "ICP gating params: blend_alpha={:.3} err=[{:.3},{:.3}] iter=[{:.1},{:.1}] max_corr_trans={:.3} max_corr_yaw={:.3} motion_trans={:.3} motion_yaw={:.3}",
            icp_params.blend_alpha,
            icp_params.full_weight_error,
            icp_params.reject_error,
            icp_params.full_weight_iterations,
            icp_params.reject_iterations,
            icp_params.max_translation_correction,
            icp_params.max_yaw_correction,
            icp_params.full_weight_translation_motion,
            icp_params.full_weight_yaw_motion
        );
    }
    let icp_mode = icp_mode_from_env();
    let submap_params = submap_params_from_env();
    if use_corrected_frame {
        pr_info!(
            logger,
            "ICP mode={} (submap max_points={} radius_m={:.2} bootstrap_scans={})",
            icp_mode.as_str(),
            submap_params.budget.max_points,
            submap_params.budget.max_radius_m,
            submap_params.bootstrap_scans
        );
    }
    let icp_point_stride = icp_point_stride_from_env();
    if use_corrected_frame {
        pr_info!(
            logger,
            "ICP scan subsample stride={} (SLAM_ICP_POINT_STRIDE, 1=full resolution)",
            icp_point_stride
        );
    }

    let mut selector = ctx.create_selector()?;

    let state_odom = state.clone();
    selector.add_subscriber(
        odom_sub,
        Box::new(move |msg| {
            if let Ok(mut st) = state_odom.lock() {
                st.raw_odom = odom_measurement_from_msg(&msg);
            }
        }),
    );

    let state_scan = state.clone();
    let icp_params_scan = icp_params;
    let icp_stride_scan = icp_point_stride;
    let icp_mode_scan = icp_mode;
    let submap_params_scan = submap_params;
    let pub_scan = map_pub;
    let pub_pose = corrected_pose_pub;
    let pub_odom = corrected_odom_pub;
    let pub_diag = diagnostics_pub;
    let log_scan = Logger::new("slam_node");
    selector.add_subscriber(
        scan_sub,
        Box::new(move |msg| {
            let full_scan = match scan_to_point_matrix(&msg) {
                Some(m) => m,
                None => return,
            };
            let current_scan = subsample_points_for_icp(&full_scan, icp_stride_scan);

            let mut st = match state_scan.lock() {
                Ok(guard) => guard,
                Err(_) => {
                    pr_warn!(log_scan, "failed to lock SLAM state");
                    return;
                }
            };

            if !st.raw_odom.received {
                pr_warn!(
                    log_scan,
                    "skipping scan because no odom has been received yet"
                );
                return;
            }

            let raw_odom = st.raw_odom.clone();
            let raw_pose = raw_odom.pose;
            let had_previous_scan = st.previous_scan.is_some();
            let odom_delta = st
                .previous_raw_odom_at_scan
                .map(|previous_raw_odom| relative_motion_local(previous_raw_odom, raw_pose));
            let mut icp_converged = false;
            let mut icp_iterations = 0;
            let mut icp_final_error = f64::NAN;
            let mut icp_initial_error = f64::NAN;
            let mut icp_error_median = f64::NAN;
            let mut icp_error_p90 = f64::NAN;
            let mut icp_inlier_ratio_5cm = f64::NAN;
            let mut icp_relative_error_reduction = f64::NAN;
            let mut icp_delta: Option<MotionDelta> = None;
            let prev_corrected = st.corrected_pose;
            let icp_result_opt = match icp_mode_scan {
                IcpMode::ScanToScan => st
                    .previous_scan
                    .as_ref()
                    .map(|previous| icp_matching(previous, &current_scan)),
                IcpMode::ScanToMap => {
                    // Need the submap warmed up plus an odom delta to seed the prediction.
                    let bootstrap_done = st.corrected_initialized
                        && st.local_submap_scan_count >= submap_params_scan.bootstrap_scans;
                    match (st.local_submap_world.as_ref(), odom_delta, bootstrap_done) {
                        (Some(submap), Some(od), true) => {
                            let predicted = compose_pose_local(prev_corrected, od);
                            let current_world = transform_scan_to_world(&current_scan, predicted);
                            let result = icp_matching(submap, &current_world);
                            if let Some(residual_world) = icp_motion_delta(&result) {
                                let corrected_candidate =
                                    apply_world_residual(predicted, residual_world);
                                icp_delta = Some(relative_motion_local(
                                    prev_corrected,
                                    corrected_candidate,
                                ));
                            }
                            Some(result)
                        }
                        _ => None,
                    }
                }
            };
            if let Some(icp_result) = icp_result_opt {
                icp_converged = icp_result.converged;
                icp_iterations = icp_result.iterations;
                icp_final_error = icp_result.final_error_mean;
                icp_initial_error = icp_result.initial_error_mean;
                icp_error_median = icp_result.final_error_median;
                icp_error_p90 = icp_result.final_error_p90;
                icp_inlier_ratio_5cm = icp_result.inlier_ratio_5cm;
                icp_relative_error_reduction = icp_result.relative_error_reduction;
                // For scan-to-scan, derive the per-scan body delta directly. For
                // scan-to-map, icp_delta is already set above from the world residual.
                if matches!(icp_mode_scan, IcpMode::ScanToScan) {
                    icp_delta = icp_motion_delta(&icp_result);
                }
                let now = Instant::now();
                if !icp_result.converged {
                    pr_warn!(
                        log_scan,
                        "ICP did not converge: iter={}, error_mean={:.4}",
                        icp_result.iterations,
                        icp_result.final_error_mean
                    );
                    st.last_icp_log_at = Some(now);
                } else if st.last_icp_log_at.map_or(true, |last_log_at| {
                    now.duration_since(last_log_at) >= ICP_INFO_LOG_INTERVAL
                }) {
                    pr_info!(
                        log_scan,
                        "ICP converged: iter={}, error_mean={:.4}",
                        icp_result.iterations,
                        icp_result.final_error_mean
                    );
                    st.last_icp_log_at = Some(now);
                }
            }
            let mut applied_delta = None;
            let mut blend_applied = false;
            let mut blend_alpha = 0.0;
            let mut blend_alpha_yaw = 0.0;
            let mut gate_reason = if had_previous_scan {
                "odom_only"
            } else {
                "bootstrap"
            };
            let mut gate_reason_yaw = gate_reason;
            let pose = if use_corrected_frame {
                if !st.corrected_initialized {
                    st.corrected_pose = raw_pose;
                    st.corrected_initialized = true;
                } else if let Some(odom_delta) = odom_delta {
                    let blended_delta = if let Some(icp_delta) = icp_delta {
                        let decision = compute_icp_blend_decision(
                            odom_delta,
                            icp_delta,
                            icp_converged,
                            icp_iterations,
                            icp_final_error,
                            &icp_params_scan,
                        );
                        blend_alpha = decision.alpha;
                        gate_reason = decision.reason;
                        blend_alpha_yaw = decision.alpha_yaw;
                        gate_reason_yaw = decision.reason_yaw;
                        if decision.alpha > 0.0 || decision.alpha_yaw > 0.0 {
                            blend_applied = true;
                            blend_motion_delta(
                                odom_delta,
                                icp_delta,
                                decision.alpha,
                                decision.alpha_yaw,
                                &icp_params_scan,
                            )
                        } else {
                            odom_delta
                        }
                    } else {
                        gate_reason = "missing_icp_delta";
                        gate_reason_yaw = "missing_icp_delta";
                        odom_delta
                    };
                    applied_delta = Some(blended_delta);
                    st.corrected_pose = compose_pose_local(st.corrected_pose, blended_delta);
                } else {
                    gate_reason = "syncing_to_odom";
                    gate_reason_yaw = "syncing_to_odom";
                    st.corrected_pose = raw_pose;
                }
                st.corrected_pose
            } else {
                raw_pose
            };

            let scan_ranges: Vec<f64> = msg.ranges.iter().map(|v| *v as f64).collect();
            st.map.update_with_scan(
                pose.x,
                pose.y,
                pose.yaw,
                &scan_ranges,
                msg.angle_min as f64,
                msg.angle_increment as f64,
            );

            // Maintain the local submap *after* the corrected pose has been updated for
            // this scan, so the appended points reflect the just-corrected pose.
            if matches!(icp_mode_scan, IcpMode::ScanToMap) && st.corrected_initialized {
                let anchor = st.corrected_pose;
                let scan_world_corrected = transform_scan_to_world(&current_scan, anchor);
                let new_submap = append_and_prune(
                    st.local_submap_world.take(),
                    &scan_world_corrected,
                    anchor,
                    submap_params_scan.budget,
                );
                st.local_submap_world = Some(new_submap);
                st.local_submap_scan_count = st.local_submap_scan_count.saturating_add(1);
            }
            let submap_points = st
                .local_submap_world
                .as_ref()
                .map_or(0, |m| m.ncols());

            let frame_id = if use_corrected_frame {
                corrected_frame_id.clone()
            } else {
                raw_odom.frame_id.clone()
            };
            match build_occupancy_grid_msg(&st.map, &frame_id, &msg.header.stamp) {
                Ok(map_msg) => {
                    if let Err(err) = pub_scan.send(&map_msg) {
                        pr_warn!(log_scan, "failed to publish map: {}", err);
                    }
                }
                Err(err) => {
                    pr_warn!(log_scan, "failed to build OccupancyGrid: {}", err);
                }
            }

            if use_corrected_frame {
                if let Err(err) =
                    publish_corrected_pose(&pub_pose, pose, &frame_id, &msg.header.stamp)
                {
                    pr_warn!(log_scan, "failed to publish corrected pose: {}", err);
                }
                if let Err(err) =
                    publish_corrected_odom(&pub_odom, pose, &raw_odom, &frame_id, &msg.header.stamp)
                {
                    pr_warn!(log_scan, "failed to publish corrected odom: {}", err);
                }
            }

            let diagnostics = format_slam_diagnostics(
                &frame_id,
                &raw_odom.frame_id,
                raw_pose,
                pose,
                use_corrected_frame,
                ScanDiagnostics {
                    scan_points: current_scan.ncols(),
                    had_previous_scan,
                    icp_converged,
                    icp_iterations,
                    icp_final_error,
                    icp_initial_error,
                    icp_error_median,
                    icp_error_p90,
                    icp_inlier_ratio_5cm,
                    icp_relative_error_reduction,
                    odom_delta,
                    icp_delta,
                    applied_delta,
                    blend_applied,
                    blend_alpha,
                    blend_alpha_yaw,
                    gate_reason,
                    gate_reason_yaw,
                    icp_mode: icp_mode_scan.as_str(),
                    submap_points,
                },
                icp_params_scan.blend_alpha,
            );
            if let Err(err) = publish_diagnostics(&pub_diag, &diagnostics) {
                pr_warn!(log_scan, "failed to publish slam diagnostics: {}", err);
            }

            st.previous_scan = Some(current_scan);
            st.previous_raw_odom_at_scan = Some(raw_pose);
        }),
    );

    loop {
        selector.wait()?;
    }
}

#[cfg(test)]
mod tests {
    use super::{
        append_and_prune, apply_world_residual, blend_motion_delta, compose_pose_local,
        compute_icp_blend_decision, format_slam_diagnostics, normalize_angle,
        odom_measurement_from_msg, relative_motion_local, subsample_points_for_icp,
        transform_scan_to_world, IcpGatingParams, MotionDelta, Pose2D, ScanDiagnostics,
        SubmapBudget, DEFAULT_BASE_FRAME_ID, DEFAULT_ICP_BLEND_ALPHA, DEFAULT_ODOM_FRAME_ID,
    };
    use nalgebra::DMatrix;
    use safe_drive::msg::common_interfaces::nav_msgs;

    fn default_icp() -> IcpGatingParams {
        IcpGatingParams::default()
    }

    #[test]
    fn subsample_points_for_icp_stride_one_is_identity() {
        let m = DMatrix::from_row_slice(2, 4, &[1.0, 2.0, 3.0, 4.0, 0.0, 0.0, 0.0, 0.0]);
        let out = subsample_points_for_icp(&m, 1);
        assert_eq!(out.ncols(), 4);
        assert_eq!(out[(0, 0)], 1.0);
    }

    #[test]
    fn subsample_points_for_icp_stride_two_halves_columns() {
        let m = DMatrix::from_row_slice(2, 8, &[
            0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, //
            10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0,
        ]);
        let out = subsample_points_for_icp(&m, 2);
        assert_eq!(out.ncols(), 4);
        assert_eq!(out[(0, 0)], 0.0);
        assert_eq!(out[(0, 1)], 2.0);
    }

    #[test]
    fn odom_measurement_defaults_frames_when_headers_are_empty() {
        let msg = nav_msgs::msg::Odometry::new().expect("odom");
        let odom = odom_measurement_from_msg(&msg);
        assert_eq!(odom.frame_id, DEFAULT_ODOM_FRAME_ID);
        assert_eq!(odom.child_frame_id, DEFAULT_BASE_FRAME_ID);
    }

    #[test]
    fn odom_measurement_uses_source_headers() {
        let mut msg = nav_msgs::msg::Odometry::new().expect("odom");
        let _ = msg.header.frame_id.assign("odom_filtered");
        let _ = msg.child_frame_id.assign("base_link");
        let odom = odom_measurement_from_msg(&msg);
        assert_eq!(odom.frame_id, "odom_filtered");
        assert_eq!(odom.child_frame_id, "base_link");
    }

    #[test]
    fn relative_motion_local_projects_into_previous_heading_frame() {
        let previous = Pose2D {
            x: 1.0,
            y: 2.0,
            yaw: std::f64::consts::FRAC_PI_2,
        };
        let current = Pose2D {
            x: 1.0,
            y: 3.0,
            yaw: std::f64::consts::PI,
        };
        let delta = relative_motion_local(previous, current);
        assert!((delta.x - 1.0).abs() < 1e-9);
        assert!(delta.y.abs() < 1e-9);
        assert!((delta.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn compose_pose_local_applies_local_motion_in_world_frame() {
        let previous = Pose2D {
            x: 1.0,
            y: 2.0,
            yaw: std::f64::consts::FRAC_PI_2,
        };
        let delta = MotionDelta {
            x: 1.0,
            y: 0.0,
            yaw: std::f64::consts::FRAC_PI_2,
        };
        let pose = compose_pose_local(previous, delta);
        assert!((pose.x - 1.0).abs() < 1e-9);
        assert!((pose.y - 3.0).abs() < 1e-9);
        assert!((pose.yaw - std::f64::consts::PI).abs() < 1e-9);
    }

    #[test]
    fn blend_motion_delta_limits_large_icp_corrections() {
        let odom = MotionDelta {
            x: 0.1,
            y: 0.0,
            yaw: 0.0,
        };
        let icp = MotionDelta {
            x: 1.0,
            y: -1.0,
            yaw: 1.0,
        };
        let blended = blend_motion_delta(
            odom,
            icp,
            DEFAULT_ICP_BLEND_ALPHA,
            DEFAULT_ICP_BLEND_ALPHA,
            &default_icp(),
        );
        assert!(blended.x > odom.x);
        assert!(blended.x < 0.2);
        assert!(blended.y.abs() < 0.1);
        assert!(blended.yaw.abs() < 0.2);
    }

    #[test]
    fn compute_icp_blend_decision_rejects_high_error() {
        let decision = compute_icp_blend_decision(
            MotionDelta {
                x: 0.1,
                y: 0.0,
                yaw: 0.0,
            },
            MotionDelta {
                x: 0.11,
                y: 0.0,
                yaw: 0.01,
            },
            true,
            4,
            10.0,
            &default_icp(),
        );
        assert_eq!(decision.alpha, 0.0);
        assert_eq!(decision.reason, "high_error");
    }

    #[test]
    fn compute_icp_blend_decision_attenuates_low_motion() {
        let decision = compute_icp_blend_decision(
            MotionDelta {
                x: 0.02,
                y: 0.0,
                yaw: 0.02,
            },
            MotionDelta {
                x: 0.03,
                y: 0.0,
                yaw: 0.03,
            },
            true,
            5,
            0.005,
            &default_icp(),
        );
        assert!(decision.alpha > 0.0);
        assert!(decision.alpha < DEFAULT_ICP_BLEND_ALPHA);
        assert_eq!(decision.reason, "attenuated_low_motion");
    }

    #[test]
    fn compute_icp_blend_decision_accepts_clean_match() {
        let decision = compute_icp_blend_decision(
            MotionDelta {
                x: 0.12,
                y: 0.01,
                yaw: 0.09,
            },
            MotionDelta {
                x: 0.13,
                y: 0.015,
                yaw: 0.1,
            },
            true,
            4,
            0.006,
            &default_icp(),
        );
        assert!((decision.alpha - DEFAULT_ICP_BLEND_ALPHA).abs() < 1e-9);
        assert_eq!(decision.reason, "accepted");
    }

    #[test]
    fn normalize_angle_wraps_into_pi_range() {
        let wrapped = normalize_angle(3.5);
        assert!(wrapped < std::f64::consts::PI);
    }

    #[test]
    fn format_slam_diagnostics_includes_status_and_deltas() {
        let text = format_slam_diagnostics(
            "map",
            "odom",
            Pose2D {
                x: 1.0,
                y: 2.0,
                yaw: 0.3,
            },
            Pose2D {
                x: 1.1,
                y: 2.1,
                yaw: 0.4,
            },
            true,
            ScanDiagnostics {
                scan_points: 42,
                had_previous_scan: true,
                icp_converged: true,
                icp_iterations: 7,
                icp_final_error: 0.006,
                icp_initial_error: 0.040,
                icp_error_median: 0.005,
                icp_error_p90: 0.010,
                icp_inlier_ratio_5cm: 0.950,
                icp_relative_error_reduction: 0.850,
                odom_delta: Some(MotionDelta {
                    x: 0.1,
                    y: 0.0,
                    yaw: 0.01,
                }),
                icp_delta: Some(MotionDelta {
                    x: 0.2,
                    y: 0.1,
                    yaw: 0.02,
                }),
                applied_delta: Some(MotionDelta {
                    x: 0.15,
                    y: 0.05,
                    yaw: 0.015,
                }),
                blend_applied: true,
                blend_alpha: DEFAULT_ICP_BLEND_ALPHA,
                blend_alpha_yaw: DEFAULT_ICP_BLEND_ALPHA,
                gate_reason: "accepted",
                gate_reason_yaw: "accepted",
                icp_mode: "scan_to_scan",
                submap_points: 0,
            },
            DEFAULT_ICP_BLEND_ALPHA,
        );
        assert!(text.contains("status=icp_ok"));
        assert!(text.contains("icp_mode=scan_to_scan"));
        assert!(text.contains("submap_points=0"));
        assert!(text.contains("icp_error_mean=0.006"));
        assert!(text.contains("icp_error_p90=0.010"));
        assert!(text.contains("icp_inlier_ratio_5cm=0.950"));
        assert!(text.contains("icp_relative_error_reduction=0.850"));
        assert!(text.contains("scan_points=42"));
        assert!(text.contains("odom_dx=0.100"));
        assert!(text.contains("applied_dyaw=0.015"));
        assert!(text.contains("blend_alpha=0.350"));
        assert!(text.contains("blend_alpha_yaw=0.350"));
        assert!(text.contains("gate_reason=accepted"));
        assert!(text.contains("gate_reason_yaw=accepted"));
    }

    #[test]
    fn compute_icp_blend_decision_yaw_only_during_in_place_rotation() {
        // Robot is rotating in place: large yaw, near-zero translation.
        // Both odom and ICP report this motion. With a looser yaw error threshold,
        // alpha_yaw should be > 0 while alpha_xy stays 0 (low_motion on translation).
        let mut params = default_icp();
        params.full_weight_error_yaw = 0.020;
        params.reject_error_yaw = 0.030;
        // Translation motion is well below full_weight_translation_motion (0.05).
        let odom = MotionDelta {
            x: 0.001,
            y: 0.0,
            yaw: 0.12,
        };
        let icp = MotionDelta {
            x: 0.0,
            y: 0.0,
            yaw: 0.12,
        };
        // ICP error 0.018 is above default reject (0.011) but below yaw reject (0.030).
        let decision = compute_icp_blend_decision(odom, icp, true, 8, 0.018, &params);
        assert_eq!(decision.alpha, 0.0, "XY must be rejected by high_error");
        assert_eq!(decision.reason, "high_error");
        assert!(
            decision.alpha_yaw > 0.0,
            "yaw must be admitted under looser reject_error_yaw"
        );
    }

    #[test]
    fn transform_scan_to_world_at_origin_is_identity() {
        let scan = DMatrix::from_row_slice(2, 3, &[1.0, 0.0, -1.0, 0.0, 1.0, 0.0]);
        let world = transform_scan_to_world(
            &scan,
            Pose2D {
                x: 0.0,
                y: 0.0,
                yaw: 0.0,
            },
        );
        assert_eq!(world.ncols(), 3);
        for c in 0..3 {
            assert!((world[(0, c)] - scan[(0, c)]).abs() < 1e-12);
            assert!((world[(1, c)] - scan[(1, c)]).abs() < 1e-12);
        }
    }

    #[test]
    fn transform_scan_to_world_applies_translation_and_rotation() {
        // Single body point at (+1, 0). Pose at (10, 20) facing +y (yaw=pi/2).
        // After rotation, body +x becomes world +y; then translate to (10, 21).
        let scan = DMatrix::from_row_slice(2, 1, &[1.0, 0.0]);
        let world = transform_scan_to_world(
            &scan,
            Pose2D {
                x: 10.0,
                y: 20.0,
                yaw: std::f64::consts::FRAC_PI_2,
            },
        );
        assert!((world[(0, 0)] - 10.0).abs() < 1e-9);
        assert!((world[(1, 0)] - 21.0).abs() < 1e-9);
    }

    #[test]
    fn transform_scan_to_world_handles_empty_scan() {
        let empty = DMatrix::zeros(2, 0);
        let world = transform_scan_to_world(
            &empty,
            Pose2D {
                x: 0.0,
                y: 0.0,
                yaw: 1.2,
            },
        );
        assert_eq!(world.ncols(), 0);
    }

    #[test]
    fn apply_world_residual_zero_is_identity() {
        let predicted = Pose2D {
            x: 1.0,
            y: 2.0,
            yaw: 0.3,
        };
        let out = apply_world_residual(
            predicted,
            MotionDelta {
                x: 0.0,
                y: 0.0,
                yaw: 0.0,
            },
        );
        assert!((out.x - 1.0).abs() < 1e-12);
        assert!((out.y - 2.0).abs() < 1e-12);
        assert!((out.yaw - 0.3).abs() < 1e-12);
    }

    #[test]
    fn apply_world_residual_pure_translation_shifts_position_only() {
        let predicted = Pose2D {
            x: 1.0,
            y: 2.0,
            yaw: 0.3,
        };
        let out = apply_world_residual(
            predicted,
            MotionDelta {
                x: 0.1,
                y: -0.2,
                yaw: 0.0,
            },
        );
        assert!((out.x - 1.1).abs() < 1e-9);
        assert!((out.y - 1.8).abs() < 1e-9);
        assert!((out.yaw - 0.3).abs() < 1e-9);
    }

    #[test]
    fn apply_world_residual_pure_rotation_rotates_position_about_origin() {
        // Predicted pose at (1, 0). Residual is rotate by pi/2 about world origin.
        // Expected new position: (0, 1). Yaw shifts from 0 to pi/2.
        let predicted = Pose2D {
            x: 1.0,
            y: 0.0,
            yaw: 0.0,
        };
        let out = apply_world_residual(
            predicted,
            MotionDelta {
                x: 0.0,
                y: 0.0,
                yaw: std::f64::consts::FRAC_PI_2,
            },
        );
        assert!(out.x.abs() < 1e-9);
        assert!((out.y - 1.0).abs() < 1e-9);
        assert!((out.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn append_and_prune_into_empty_keeps_points_within_radius() {
        let new_world = DMatrix::from_row_slice(2, 3, &[
            0.0, 0.5, 5.0, //
            0.0, 0.0, 0.0,
        ]);
        let anchor = Pose2D {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
        };
        let out = append_and_prune(
            None,
            &new_world,
            anchor,
            SubmapBudget {
                max_points: 0,
                max_radius_m: 1.0,
            },
        );
        // (0,0) and (0.5,0) survive, (5,0) is pruned.
        assert_eq!(out.ncols(), 2);
        assert!((out[(0, 0)] - 0.0).abs() < 1e-12);
        assert!((out[(0, 1)] - 0.5).abs() < 1e-12);
    }

    #[test]
    fn append_and_prune_concatenates_existing_and_new() {
        let existing = DMatrix::from_row_slice(2, 2, &[0.1, 0.2, 0.0, 0.0]);
        let new_world = DMatrix::from_row_slice(2, 2, &[0.3, 0.4, 0.0, 0.0]);
        let out = append_and_prune(
            Some(existing),
            &new_world,
            Pose2D {
                x: 0.0,
                y: 0.0,
                yaw: 0.0,
            },
            SubmapBudget {
                max_points: 0,
                max_radius_m: 10.0,
            },
        );
        assert_eq!(out.ncols(), 4);
        // Insertion order is preserved: existing first, then new.
        assert!((out[(0, 0)] - 0.1).abs() < 1e-12);
        assert!((out[(0, 1)] - 0.2).abs() < 1e-12);
        assert!((out[(0, 2)] - 0.3).abs() < 1e-12);
        assert!((out[(0, 3)] - 0.4).abs() < 1e-12);
    }

    #[test]
    fn append_and_prune_caps_to_max_points_keeping_newest_tail() {
        let existing = DMatrix::from_row_slice(2, 3, &[0.10, 0.11, 0.12, 0.0, 0.0, 0.0]);
        let new_world = DMatrix::from_row_slice(2, 2, &[0.20, 0.21, 0.0, 0.0]);
        let out = append_and_prune(
            Some(existing),
            &new_world,
            Pose2D {
                x: 0.0,
                y: 0.0,
                yaw: 0.0,
            },
            SubmapBudget {
                max_points: 3,
                max_radius_m: 10.0,
            },
        );
        // 5 total points, all within radius, cap to 3 keeping the 3 newest:
        // drop existing[0]=0.10 and existing[1]=0.11; keep 0.12, 0.20, 0.21.
        assert_eq!(out.ncols(), 3);
        assert!((out[(0, 0)] - 0.12).abs() < 1e-12);
        assert!((out[(0, 1)] - 0.20).abs() < 1e-12);
        assert!((out[(0, 2)] - 0.21).abs() < 1e-12);
    }

    #[test]
    fn append_and_prune_uses_anchor_for_radius_check() {
        let new_world = DMatrix::from_row_slice(2, 3, &[
            9.0, 10.0, 11.0, //
            10.0, 10.0, 10.0,
        ]);
        let anchor = Pose2D {
            x: 10.0,
            y: 10.0,
            yaw: 0.0,
        };
        let out = append_and_prune(
            None,
            &new_world,
            anchor,
            SubmapBudget {
                max_points: 0,
                max_radius_m: 1.5,
            },
        );
        // All three are within 1.5 m of anchor (10,10): (9,10), (10,10), (11,10).
        assert_eq!(out.ncols(), 3);
    }

    #[test]
    fn append_and_prune_empty_inputs_return_empty_matrix() {
        let empty = DMatrix::zeros(2, 0);
        let out = append_and_prune(
            None,
            &empty,
            Pose2D {
                x: 0.0,
                y: 0.0,
                yaw: 0.0,
            },
            SubmapBudget {
                max_points: 10,
                max_radius_m: 1.0,
            },
        );
        assert_eq!(out.ncols(), 0);
    }

    #[test]
    fn compute_icp_blend_decision_xy_low_motion_during_in_place_rotation() {
        // Same in-place rotation regime but with default (equal) yaw thresholds.
        // Even with default thresholds, the split motion gate should reject XY for
        // low_motion (no translation) while yaw is still attenuated by motion ramp-up.
        let params = default_icp();
        let odom = MotionDelta {
            x: 0.0,
            y: 0.0,
            yaw: 0.12,
        };
        let icp = MotionDelta {
            x: 0.0,
            y: 0.0,
            yaw: 0.12,
        };
        let decision = compute_icp_blend_decision(odom, icp, true, 6, 0.006, &params);
        assert_eq!(decision.alpha, 0.0);
        assert_eq!(decision.reason, "low_motion");
        assert!(decision.alpha_yaw > 0.0);
    }
}
