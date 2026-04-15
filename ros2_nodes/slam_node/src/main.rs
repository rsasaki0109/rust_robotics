use nalgebra::DMatrix;
use rust_robotics_mapping::occupancy_grid_map::{OccupancyGridConfig, OccupancyGridMap};
use rust_robotics_slam::icp_matching::icp_matching;
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::{builtin_interfaces, common_interfaces::{nav_msgs, sensor_msgs}},
    pr_info, pr_warn,
};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const ICP_INFO_LOG_INTERVAL: Duration = Duration::from_secs(5);
const DEFAULT_MAP_FRAME_ID: &str = "odom";

#[derive(Clone, Copy)]
struct Pose2D {
    x: f64,
    y: f64,
    yaw: f64,
}

struct SlamState {
    pose: Pose2D,
    map: OccupancyGridMap,
    odom_frame_id: String,
    previous_scan: Option<DMatrix<f64>>,
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

fn map_frame_id_from_odom(msg: &nav_msgs::msg::Odometry) -> String {
    let frame_id = msg.header.frame_id.get_string();
    if frame_id.is_empty() {
        DEFAULT_MAP_FRAME_ID.to_string()
    } else {
        frame_id
    }
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
    let mut data = safe_drive::msg::I8Seq::<0>::new(total_cells).ok_or("failed to allocate map data")?;
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

fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("slam_node", None, Default::default())?;

    let scan_sub = node.create_subscriber::<sensor_msgs::msg::LaserScan>("/scan", None)?;
    let odom_sub = node.create_subscriber::<nav_msgs::msg::Odometry>("/odom", None)?;
    let map_pub = node.create_publisher::<nav_msgs::msg::OccupancyGrid>("/map", None)?;

    let map_config = OccupancyGridConfig {
        resolution: 0.2,
        width: 300,
        height: 300,
        ..Default::default()
    };

    let state = Arc::new(Mutex::new(SlamState {
        pose: Pose2D {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
        },
        map: OccupancyGridMap::new(map_config),
        odom_frame_id: DEFAULT_MAP_FRAME_ID.to_string(),
        previous_scan: None,
        last_icp_log_at: None,
    }));

    let logger = Logger::new("slam_node");
    pr_info!(logger, "slam node started");

    let mut selector = ctx.create_selector()?;

    let state_odom = state.clone();
    selector.add_subscriber(
        odom_sub,
        Box::new(move |msg| {
            let q = &msg.pose.pose.orientation;
            let yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w);
            if let Ok(mut st) = state_odom.lock() {
                st.pose = Pose2D {
                    x: msg.pose.pose.position.x,
                    y: msg.pose.pose.position.y,
                    yaw,
                };
                st.odom_frame_id = map_frame_id_from_odom(&msg);
            }
        }),
    );

    let state_scan = state.clone();
    let pub_scan = map_pub;
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

            if let Some(previous) = st.previous_scan.as_ref() {
                let icp_result = icp_matching(previous, &current_scan);
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

            let pose = st.pose;
            let scan_ranges: Vec<f64> = msg.ranges.iter().map(|v| *v as f64).collect();
            st.map.update_with_scan(
                pose.x,
                pose.y,
                pose.yaw,
                &scan_ranges,
                msg.angle_min as f64,
                msg.angle_increment as f64,
            );

            let frame_id = st.odom_frame_id.clone();
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

            st.previous_scan = Some(current_scan);
        }),
    );

    loop {
        selector.wait()?;
    }
}

#[cfg(test)]
mod tests {
    use super::{map_frame_id_from_odom, DEFAULT_MAP_FRAME_ID};
    use safe_drive::msg::common_interfaces::nav_msgs;

    #[test]
    fn map_frame_defaults_to_odom_when_header_is_empty() {
        let msg = nav_msgs::msg::Odometry::new().expect("odom");
        assert_eq!(map_frame_id_from_odom(&msg), DEFAULT_MAP_FRAME_ID);
    }

    #[test]
    fn map_frame_uses_source_odom_header() {
        let mut msg = nav_msgs::msg::Odometry::new().expect("odom");
        let _ = msg.header.frame_id.assign("odom_filtered");
        assert_eq!(map_frame_id_from_odom(&msg), "odom_filtered");
    }
}
