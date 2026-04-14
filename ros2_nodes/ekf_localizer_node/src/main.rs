use rust_robotics_core::{ControlInput, Point2D, State2D};
use rust_robotics_localization::{EKFConfig, EKFLocalizer};
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::{
        builtin_interfaces,
        common_interfaces::{geometry_msgs, nav_msgs},
    },
    pr_info, pr_warn,
};
use std::{
    env, io,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

const DEFAULT_INPUT_ODOM_TOPIC: &str = "/odom";
const DEFAULT_OUTPUT_ODOM_TOPIC: &str = "/ekf_odom";
const DEFAULT_OUTPUT_POSE_TOPIC: &str = "/ekf_pose";
const DEFAULT_FRAME_ID: &str = "odom";
const LOG_INTERVAL: Duration = Duration::from_secs(5);
const FALLBACK_DT: f64 = 0.1;
const MIN_DT: f64 = 1e-3;
const MAX_DT: f64 = 0.5;

struct EkfState {
    localizer: EKFLocalizer,
    initialized: bool,
    last_update_at: Option<Instant>,
    last_log_at: Option<Instant>,
}

fn topic_from_env(key: &str, default: &str) -> String {
    env::var(key)
        .ok()
        .filter(|value| !value.trim().is_empty())
        .unwrap_or_else(|| default.to_string())
}

fn yaw_from_quaternion(x: f64, y: f64, z: f64, w: f64) -> f64 {
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    siny_cosp.atan2(cosy_cosp)
}

fn apply_yaw_to_pose(pose: &mut geometry_msgs::msg::Pose, yaw: f64) {
    let half_yaw = 0.5 * yaw;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = half_yaw.sin();
    pose.orientation.w = half_yaw.cos();
}

fn sanitize_dt(last_update_at: Option<Instant>, now: Instant) -> f64 {
    match last_update_at {
        Some(previous) => now
            .saturating_duration_since(previous)
            .as_secs_f64()
            .clamp(MIN_DT, MAX_DT),
        None => FALLBACK_DT,
    }
}

fn initial_state_from_odom(msg: &nav_msgs::msg::Odometry) -> State2D {
    let orientation = &msg.pose.pose.orientation;
    State2D::new(
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
        msg.twist.twist.linear.x,
    )
}

fn copy_stamp(
    target: &mut builtin_interfaces::UnsafeTime,
    source: &builtin_interfaces::UnsafeTime,
) {
    target.sec = source.sec;
    target.nanosec = source.nanosec;
}

fn output_frame_id(source: &nav_msgs::msg::Odometry) -> String {
    let source_frame_id = source.header.frame_id.get_string();
    if source_frame_id.is_empty() {
        DEFAULT_FRAME_ID.to_string()
    } else {
        source_frame_id
    }
}

fn publish_pose(
    publisher: &safe_drive::topic::publisher::Publisher<geometry_msgs::msg::PoseStamped>,
    state: State2D,
    source: &nav_msgs::msg::Odometry,
) -> Result<(), DynError> {
    let mut msg = geometry_msgs::msg::PoseStamped::new().ok_or("failed to allocate PoseStamped")?;
    copy_stamp(&mut msg.header.stamp, &source.header.stamp);
    let frame_id = output_frame_id(source);
    let _ = msg.header.frame_id.assign(&frame_id);
    msg.pose.position.x = state.x;
    msg.pose.position.y = state.y;
    msg.pose.position.z = 0.0;
    apply_yaw_to_pose(&mut msg.pose, state.yaw);
    publisher.send(&msg)?;
    Ok(())
}

fn publish_odom(
    publisher: &safe_drive::topic::publisher::Publisher<nav_msgs::msg::Odometry>,
    state: State2D,
    source: &nav_msgs::msg::Odometry,
) -> Result<(), DynError> {
    let mut msg = nav_msgs::msg::Odometry::new().ok_or("failed to allocate Odometry")?;
    copy_stamp(&mut msg.header.stamp, &source.header.stamp);
    let frame_id = output_frame_id(source);
    let _ = msg.header.frame_id.assign(&frame_id);
    let child_frame_id = source.child_frame_id.get_string();
    if !child_frame_id.is_empty() {
        let _ = msg.child_frame_id.assign(&child_frame_id);
    }

    msg.pose.pose.position.x = state.x;
    msg.pose.pose.position.y = state.y;
    msg.pose.pose.position.z = 0.0;
    apply_yaw_to_pose(&mut msg.pose.pose, state.yaw);
    msg.pose.covariance = source.pose.covariance;

    msg.twist.twist.linear.x = source.twist.twist.linear.x;
    msg.twist.twist.linear.y = source.twist.twist.linear.y;
    msg.twist.twist.linear.z = source.twist.twist.linear.z;
    msg.twist.twist.angular.x = source.twist.twist.angular.x;
    msg.twist.twist.angular.y = source.twist.twist.angular.y;
    msg.twist.twist.angular.z = source.twist.twist.angular.z;
    msg.twist.covariance = source.twist.covariance;

    publisher.send(&msg)?;
    Ok(())
}

fn should_log(last_log_at: &mut Option<Instant>, now: Instant) -> bool {
    let do_log = last_log_at
        .map(|previous| now.duration_since(previous) >= LOG_INTERVAL)
        .unwrap_or(true);
    if do_log {
        *last_log_at = Some(now);
    }
    do_log
}

fn main() -> Result<(), DynError> {
    let input_odom_topic = topic_from_env("EKF_INPUT_ODOM_TOPIC", DEFAULT_INPUT_ODOM_TOPIC);
    let output_odom_topic = topic_from_env("EKF_OUTPUT_ODOM_TOPIC", DEFAULT_OUTPUT_ODOM_TOPIC);
    let output_pose_topic = topic_from_env("EKF_OUTPUT_POSE_TOPIC", DEFAULT_OUTPUT_POSE_TOPIC);

    let initial_localizer =
        EKFLocalizer::with_initial_state_2d(State2D::origin(), EKFConfig::default())
            .map_err(|err| io::Error::new(io::ErrorKind::InvalidInput, err.to_string()))?;
    let shared_state = Arc::new(Mutex::new(EkfState {
        localizer: initial_localizer,
        initialized: false,
        last_update_at: None,
        last_log_at: None,
    }));

    let ctx = Context::new()?;
    let node = ctx.create_node("ekf_localizer_node", None, Default::default())?;
    let odom_sub = node.create_subscriber::<nav_msgs::msg::Odometry>(&input_odom_topic, None)?;
    let pose_pub =
        node.create_publisher::<geometry_msgs::msg::PoseStamped>(&output_pose_topic, None)?;
    let odom_pub = node.create_publisher::<nav_msgs::msg::Odometry>(&output_odom_topic, None)?;

    let logger = Logger::new("ekf_localizer_node");
    pr_info!(
        logger,
        "ekf localizer started (input: {}, pose: {}, odom: {})",
        input_odom_topic,
        output_pose_topic,
        output_odom_topic
    );

    let mut selector = ctx.create_selector()?;
    let state_cb = shared_state.clone();
    let logger_cb = Logger::new("ekf_localizer_node");
    selector.add_subscriber(
        odom_sub,
        Box::new(move |msg| {
            let now = Instant::now();
            let mut state = match state_cb.lock() {
                Ok(guard) => guard,
                Err(_) => {
                    pr_warn!(logger_cb, "failed to lock EKF state on odom callback");
                    return;
                }
            };

            if !state.initialized {
                let initialized = initial_state_from_odom(&msg);
                state.localizer =
                    match EKFLocalizer::with_initial_state_2d(initialized, EKFConfig::default()) {
                        Ok(localizer) => localizer,
                        Err(err) => {
                            pr_warn!(logger_cb, "failed to initialize EKF state: {}", err);
                            return;
                        }
                    };
                state.initialized = true;
                state.last_update_at = Some(now);

                if let Err(err) = publish_pose(&pose_pub, initialized, &msg) {
                    pr_warn!(
                        logger_cb,
                        "failed to publish initial filtered pose: {}",
                        err
                    );
                    return;
                }
                if let Err(err) = publish_odom(&odom_pub, initialized, &msg) {
                    pr_warn!(
                        logger_cb,
                        "failed to publish initial filtered odom: {}",
                        err
                    );
                    return;
                }

                if should_log(&mut state.last_log_at, now) {
                    pr_info!(
                        logger_cb,
                        "initialized filtered pose x={:.2} y={:.2} yaw={:.2} v={:.2}",
                        initialized.x,
                        initialized.y,
                        initialized.yaw,
                        initialized.v
                    );
                }
                return;
            }

            let dt = sanitize_dt(state.last_update_at, now);
            state.last_update_at = Some(now);

            let measurement = Point2D::new(msg.pose.pose.position.x, msg.pose.pose.position.y);
            let control = ControlInput::new(msg.twist.twist.linear.x, msg.twist.twist.angular.z);

            let estimate = match state.localizer.estimate_state(measurement, control, dt) {
                Ok(estimate) => estimate,
                Err(err) => {
                    pr_warn!(logger_cb, "EKF update failed: {}", err);
                    return;
                }
            };

            if let Err(err) = publish_pose(&pose_pub, estimate, &msg) {
                pr_warn!(logger_cb, "failed to publish filtered pose: {}", err);
                return;
            }
            if let Err(err) = publish_odom(&odom_pub, estimate, &msg) {
                pr_warn!(logger_cb, "failed to publish filtered odom: {}", err);
                return;
            }

            if should_log(&mut state.last_log_at, now) {
                pr_info!(
                    logger_cb,
                    "filtered pose x={:.2} y={:.2} yaw={:.2} v={:.2}",
                    estimate.x,
                    estimate.y,
                    estimate.yaw,
                    estimate.v
                );
            }
        }),
    );

    loop {
        selector.wait()?;
    }
}

#[cfg(test)]
mod tests {
    use super::{
        copy_stamp, output_frame_id, sanitize_dt, yaw_from_quaternion, DEFAULT_FRAME_ID,
        FALLBACK_DT, MAX_DT, MIN_DT,
    };
    use safe_drive::msg::common_interfaces::nav_msgs;
    use std::time::{Duration, Instant};

    #[test]
    fn sanitize_dt_uses_default_without_history() {
        let now = Instant::now();
        assert!((sanitize_dt(None, now) - FALLBACK_DT).abs() < 1e-9);
    }

    #[test]
    fn sanitize_dt_clamps_bounds() {
        let now = Instant::now();
        let short = now - Duration::from_micros(100);
        let long = now - Duration::from_secs(2);
        assert!((sanitize_dt(Some(short), now) - MIN_DT).abs() < 1e-9);
        assert!((sanitize_dt(Some(long), now) - MAX_DT).abs() < 1e-9);
    }

    #[test]
    fn yaw_from_quaternion_handles_planar_rotation() {
        let half = std::f64::consts::FRAC_PI_4;
        let yaw = yaw_from_quaternion(0.0, 0.0, half.sin(), half.cos());
        assert!((yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn output_frame_id_falls_back_when_source_is_empty() {
        let msg = nav_msgs::msg::Odometry::new().unwrap();
        assert_eq!(output_frame_id(&msg), DEFAULT_FRAME_ID);
    }

    #[test]
    fn copy_stamp_preserves_source_timestamp() {
        let mut source = nav_msgs::msg::Odometry::new().unwrap();
        let mut target = nav_msgs::msg::Odometry::new().unwrap();
        source.header.stamp.sec = 12;
        source.header.stamp.nanosec = 34;

        copy_stamp(&mut target.header.stamp, &source.header.stamp);

        assert_eq!(target.header.stamp.sec, 12);
        assert_eq!(target.header.stamp.nanosec, 34);
    }
}
