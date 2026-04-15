use nalgebra::DMatrix;
use rust_robotics_mapping::occupancy_grid_map::{OccupancyGridConfig, OccupancyGridMap};
use rust_robotics_slam::{icp_matching::icp_matching, ICPResult};
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::{
        builtin_interfaces,
        common_interfaces::{geometry_msgs, nav_msgs, sensor_msgs},
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
const ICP_BLEND_ALPHA: f64 = 0.35;
const MAX_ICP_TRANSLATION_CORRECTION: f64 = 0.25;
const MAX_ICP_YAW_CORRECTION: f64 = 0.35;

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

fn blend_motion_delta(odom: MotionDelta, icp: MotionDelta) -> MotionDelta {
    let correction_x = (icp.x - odom.x).clamp(
        -MAX_ICP_TRANSLATION_CORRECTION,
        MAX_ICP_TRANSLATION_CORRECTION,
    );
    let correction_y = (icp.y - odom.y).clamp(
        -MAX_ICP_TRANSLATION_CORRECTION,
        MAX_ICP_TRANSLATION_CORRECTION,
    );
    let correction_yaw =
        normalize_angle(icp.yaw - odom.yaw).clamp(-MAX_ICP_YAW_CORRECTION, MAX_ICP_YAW_CORRECTION);

    MotionDelta {
        x: odom.x + ICP_BLEND_ALPHA * correction_x,
        y: odom.y + ICP_BLEND_ALPHA * correction_y,
        yaw: normalize_angle(odom.yaw + ICP_BLEND_ALPHA * correction_yaw),
    }
}

fn apply_yaw_to_pose(pose: &mut geometry_msgs::msg::Pose, yaw: f64) {
    let half_yaw = 0.5 * yaw;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = half_yaw.sin();
    pose.orientation.w = half_yaw.cos();
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

fn main() -> Result<(), DynError> {
    let input_odom_topic = topic_from_env("SLAM_INPUT_ODOM_TOPIC", DEFAULT_INPUT_ODOM_TOPIC);
    let output_pose_topic = topic_from_env("SLAM_OUTPUT_POSE_TOPIC", DEFAULT_OUTPUT_POSE_TOPIC);
    let output_odom_topic = topic_from_env("SLAM_OUTPUT_ODOM_TOPIC", DEFAULT_OUTPUT_ODOM_TOPIC);
    let corrected_frame_id = topic_from_env("SLAM_CORRECTED_FRAME_ID", DEFAULT_MAP_FRAME_ID);
    let use_corrected_frame = bool_from_env("SLAM_USE_CORRECTED_FRAME", false);

    let ctx = Context::new()?;
    let node = ctx.create_node("slam_node", None, Default::default())?;

    let scan_sub = node.create_subscriber::<sensor_msgs::msg::LaserScan>("/scan", None)?;
    let odom_sub = node.create_subscriber::<nav_msgs::msg::Odometry>(&input_odom_topic, None)?;
    let map_pub = node.create_publisher::<nav_msgs::msg::OccupancyGrid>("/map", None)?;
    let corrected_pose_pub =
        node.create_publisher::<geometry_msgs::msg::PoseStamped>(&output_pose_topic, None)?;
    let corrected_odom_pub =
        node.create_publisher::<nav_msgs::msg::Odometry>(&output_odom_topic, None)?;

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
    let pub_scan = map_pub;
    let pub_pose = corrected_pose_pub;
    let pub_odom = corrected_odom_pub;
    let log_scan = Logger::new("slam_node");
    selector.add_subscriber(
        scan_sub,
        Box::new(move |msg| {
            let current_scan = match scan_to_point_matrix(&msg) {
                Some(m) => m,
                None => return,
            };

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
            let mut icp_delta = None;
            if let Some(previous) = st.previous_scan.as_ref() {
                let icp_result = icp_matching(previous, &current_scan);
                icp_delta = icp_motion_delta(&icp_result);
                let now = Instant::now();
                if !icp_result.converged {
                    pr_warn!(
                        log_scan,
                        "ICP did not converge: iter={}, error={:.4}",
                        icp_result.iterations,
                        icp_result.final_error
                    );
                    st.last_icp_log_at = Some(now);
                } else if st.last_icp_log_at.map_or(true, |last_log_at| {
                    now.duration_since(last_log_at) >= ICP_INFO_LOG_INTERVAL
                }) {
                    pr_info!(
                        log_scan,
                        "ICP converged: iter={}, error={:.4}",
                        icp_result.iterations,
                        icp_result.final_error
                    );
                    st.last_icp_log_at = Some(now);
                }
            }

            let pose = if use_corrected_frame {
                if !st.corrected_initialized {
                    st.corrected_pose = raw_pose;
                    st.corrected_initialized = true;
                } else if let Some(previous_raw_odom) = st.previous_raw_odom_at_scan {
                    let odom_delta = relative_motion_local(previous_raw_odom, raw_pose);
                    let blended_delta = if let Some(icp_delta) = icp_delta {
                        blend_motion_delta(odom_delta, icp_delta)
                    } else {
                        odom_delta
                    };
                    st.corrected_pose = compose_pose_local(st.corrected_pose, blended_delta);
                } else {
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
        blend_motion_delta, compose_pose_local, normalize_angle, odom_measurement_from_msg,
        relative_motion_local, MotionDelta, Pose2D, DEFAULT_BASE_FRAME_ID, DEFAULT_ODOM_FRAME_ID,
    };
    use safe_drive::msg::common_interfaces::nav_msgs;

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
        let blended = blend_motion_delta(odom, icp);
        assert!(blended.x > odom.x);
        assert!(blended.x < 0.2);
        assert!(blended.y.abs() < 0.1);
        assert!(blended.yaw.abs() < 0.2);
    }

    #[test]
    fn normalize_angle_wraps_into_pi_range() {
        let wrapped = normalize_angle(3.5);
        assert!(wrapped < std::f64::consts::PI);
    }
}
