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
    full_weight_error: f64,
    reject_error: f64,
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
            full_weight_error: DEFAULT_ICP_FULL_WEIGHT_ERROR,
            reject_error: DEFAULT_ICP_REJECT_ERROR,
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
}

#[derive(Clone, Copy)]
struct ScanDiagnostics {
    scan_points: usize,
    had_previous_scan: bool,
    icp_converged: bool,
    icp_iterations: usize,
    icp_final_error: f64,
    odom_delta: Option<MotionDelta>,
    icp_delta: Option<MotionDelta>,
    applied_delta: Option<MotionDelta>,
    blend_applied: bool,
    blend_alpha: f64,
    gate_reason: &'static str,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct IcpBlendDecision {
    alpha: f64,
    reason: &'static str,
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
fn icp_gating_params_from_env() -> IcpGatingParams {
    let mut p = IcpGatingParams::default();
    p.blend_alpha = clamp_f64(
        f64_from_env("SLAM_ICP_BLEND_ALPHA", p.blend_alpha),
        0.0,
        1.0,
    );
    p.full_weight_error = f64_from_env("SLAM_ICP_FULL_WEIGHT_ERROR", p.full_weight_error).max(1e-6);
    p.reject_error = f64_from_env("SLAM_ICP_REJECT_ERROR", p.reject_error).max(p.full_weight_error + 1e-6);
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
        return IcpBlendDecision {
            alpha: 0.0,
            reason: "not_converged",
        };
    }
    if !final_error.is_finite() {
        return IcpBlendDecision {
            alpha: 0.0,
            reason: "invalid_error",
        };
    }

    let correction = MotionDelta {
        x: icp.x - odom.x,
        y: icp.y - odom.y,
        yaw: normalize_angle(icp.yaw - odom.yaw),
    };
    let correction_translation = translation_magnitude(correction);
    let correction_yaw = correction.yaw.abs();
    if correction_translation >= p.max_translation_correction {
        return IcpBlendDecision {
            alpha: 0.0,
            reason: "translation_outlier",
        };
    }
    if correction_yaw >= p.max_yaw_correction {
        return IcpBlendDecision {
            alpha: 0.0,
            reason: "yaw_outlier",
        };
    }

    let error_weight = ramp_weight(final_error, p.full_weight_error, p.reject_error);
    if error_weight <= 0.0 {
        return IcpBlendDecision {
            alpha: 0.0,
            reason: "high_error",
        };
    }

    let iteration_weight = ramp_weight(
        iterations as f64,
        p.full_weight_iterations,
        p.reject_iterations,
    );
    if iteration_weight <= 0.0 {
        return IcpBlendDecision {
            alpha: 0.0,
            reason: "slow_convergence",
        };
    }

    let correction_translation_weight = ramp_weight(
        correction_translation,
        p.full_weight_translation_correction,
        p.max_translation_correction,
    );
    if correction_translation_weight <= 0.0 {
        return IcpBlendDecision {
            alpha: 0.0,
            reason: "translation_outlier",
        };
    }

    let correction_yaw_weight = ramp_weight(
        correction_yaw,
        p.full_weight_yaw_correction,
        p.max_yaw_correction,
    );
    if correction_yaw_weight <= 0.0 {
        return IcpBlendDecision {
            alpha: 0.0,
            reason: "yaw_outlier",
        };
    }

    let odom_motion = translation_magnitude(odom);
    let odom_motion_weight = ramp_up_weight(
        odom_motion,
        p.full_weight_translation_motion * 0.25,
        p.full_weight_translation_motion,
    )
    .max(ramp_up_weight(
        odom.yaw.abs(),
        p.full_weight_yaw_motion * 0.25,
        p.full_weight_yaw_motion,
    ));
    if odom_motion_weight <= 0.0 {
        return IcpBlendDecision {
            alpha: 0.0,
            reason: "low_motion",
        };
    }

    let scale = error_weight
        .min(iteration_weight)
        .min(correction_translation_weight)
        .min(correction_yaw_weight)
        .min(odom_motion_weight);
    let alpha = p.blend_alpha * scale;
    if alpha <= 0.0 {
        return IcpBlendDecision {
            alpha: 0.0,
            reason: "rejected",
        };
    }

    let reason = if scale < 0.999 {
        if scale == odom_motion_weight {
            "attenuated_low_motion"
        } else if scale == error_weight {
            "attenuated_error"
        } else if scale == iteration_weight {
            "attenuated_iterations"
        } else if scale == correction_translation_weight {
            "attenuated_translation"
        } else {
            "attenuated_yaw"
        }
    } else {
        "accepted"
    };

    IcpBlendDecision { alpha, reason }
}

fn blend_motion_delta(odom: MotionDelta, icp: MotionDelta, alpha: f64, p: &IcpGatingParams) -> MotionDelta {
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
        x: odom.x + alpha * correction_x,
        y: odom.y + alpha * correction_y,
        yaw: normalize_angle(odom.yaw + alpha * correction_yaw),
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
        if diagnostics.blend_alpha + 1e-6 >= base_blend_alpha {
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
        format!("icp_converged={}", diagnostics.icp_converged),
        format!("icp_iterations={}", diagnostics.icp_iterations),
        format!("icp_error_mean={:.4}", diagnostics.icp_final_error),
        format!("blend_applied={}", diagnostics.blend_applied),
        format!("blend_alpha={:.3}", diagnostics.blend_alpha),
        format!("gate_reason={}", diagnostics.gate_reason),
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
            let mut icp_converged = false;
            let mut icp_iterations = 0;
            let mut icp_final_error = f64::NAN;
            let mut icp_delta = None;
            if let Some(previous) = st.previous_scan.as_ref() {
                let icp_result = icp_matching(previous, &current_scan);
                icp_converged = icp_result.converged;
                icp_iterations = icp_result.iterations;
                icp_final_error = icp_result.final_error_mean;
                icp_delta = icp_motion_delta(&icp_result);
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

            let odom_delta = st
                .previous_raw_odom_at_scan
                .map(|previous_raw_odom| relative_motion_local(previous_raw_odom, raw_pose));
            let mut applied_delta = None;
            let mut blend_applied = false;
            let mut blend_alpha = 0.0;
            let mut gate_reason = if had_previous_scan {
                "odom_only"
            } else {
                "bootstrap"
            };
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
                        if decision.alpha > 0.0 {
                            blend_applied = true;
                            blend_motion_delta(odom_delta, icp_delta, decision.alpha, &icp_params_scan)
                        } else {
                            odom_delta
                        }
                    } else {
                        gate_reason = "missing_icp_delta";
                        odom_delta
                    };
                    applied_delta = Some(blended_delta);
                    st.corrected_pose = compose_pose_local(st.corrected_pose, blended_delta);
                } else {
                    gate_reason = "syncing_to_odom";
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
                    odom_delta,
                    icp_delta,
                    applied_delta,
                    blend_applied,
                    blend_alpha,
                    gate_reason,
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
        blend_motion_delta, compose_pose_local, compute_icp_blend_decision,
        format_slam_diagnostics, normalize_angle, odom_measurement_from_msg,
        relative_motion_local, subsample_points_for_icp, IcpGatingParams, MotionDelta, Pose2D,
        ScanDiagnostics, DEFAULT_BASE_FRAME_ID, DEFAULT_ICP_BLEND_ALPHA, DEFAULT_ODOM_FRAME_ID,
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
        let blended = blend_motion_delta(odom, icp, DEFAULT_ICP_BLEND_ALPHA, &default_icp());
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
                gate_reason: "accepted",
            },
            DEFAULT_ICP_BLEND_ALPHA,
        );
        assert!(text.contains("status=icp_ok"));
        assert!(text.contains("icp_error_mean=0.006"));
        assert!(text.contains("scan_points=42"));
        assert!(text.contains("odom_dx=0.100"));
        assert!(text.contains("applied_dyaw=0.015"));
        assert!(text.contains("blend_alpha=0.350"));
        assert!(text.contains("gate_reason=accepted"));
    }
}
