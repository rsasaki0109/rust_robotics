use rust_robotics_core::Point2D;
use rust_robotics_planning::dwa::DWAState;
use rust_robotics_planning::{DWAConfig, DWAPlanner};
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::{geometry_msgs, nav_msgs, sensor_msgs},
    pr_info, pr_warn,
};
use std::sync::{Arc, Mutex};

#[derive(Clone, Copy)]
struct OdomState {
    x: f64,
    y: f64,
    yaw: f64,
    v: f64,
    omega: f64,
}

#[derive(Default)]
struct PlannerState {
    odom: Option<OdomState>,
    global_path: Vec<Point2D>,
    obstacles: Vec<Point2D>,
}

fn yaw_from_quaternion(x: f64, y: f64, z: f64, w: f64) -> f64 {
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    siny_cosp.atan2(cosy_cosp)
}

fn nearest_goal(path: &[Point2D], x: f64, y: f64) -> Option<Point2D> {
    let (nearest_idx, _) = path.iter().enumerate().min_by(|(_, a), (_, b)| {
        let da = (a.x - x).powi(2) + (a.y - y).powi(2);
        let db = (b.x - x).powi(2) + (b.y - y).powi(2);
        da.total_cmp(&db)
    })?;
    let look_ahead = (nearest_idx + 10).min(path.len().saturating_sub(1));
    path.get(look_ahead).copied()
}

fn build_obstacles_from_scan(scan: &sensor_msgs::msg::LaserScan, odom: OdomState) -> Vec<Point2D> {
    let mut obstacles = Vec::new();
    let mut angle = scan.angle_min as f64;

    for range in scan.ranges.iter() {
        let r = *range as f64;
        if r.is_finite() && r > scan.range_min as f64 && r < scan.range_max as f64 {
            let beam_angle = odom.yaw + angle;
            obstacles.push(Point2D::new(
                odom.x + r * beam_angle.cos(),
                odom.y + r * beam_angle.sin(),
            ));
        }
        angle += scan.angle_increment as f64;
    }
    obstacles
}

fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("dwa_planner_node", None, Default::default())?;

    let scan_sub = node.create_subscriber::<sensor_msgs::msg::LaserScan>("/scan", None)?;
    let odom_sub = node.create_subscriber::<nav_msgs::msg::Odometry>("/odom", None)?;
    let path_sub = node.create_subscriber::<nav_msgs::msg::Path>("/planned_path", None)?;
    let cmd_pub = node.create_publisher::<geometry_msgs::msg::Twist>("/cmd_vel", None)?;

    let mut config = DWAConfig::default();
    config.goal_threshold = 0.3;
    config.robot_radius = 0.35;

    let planner = Arc::new(Mutex::new(DWAPlanner::new(config)));
    let state = Arc::new(Mutex::new(PlannerState::default()));
    let logger = Logger::new("dwa_planner_node");
    pr_info!(logger, "dwa planner started");

    let mut selector = ctx.create_selector()?;

    let state_path = state.clone();
    selector.add_subscriber(
        path_sub,
        Box::new(move |msg| {
            let mut points = Vec::with_capacity(msg.poses.len());
            for pose in msg.poses.iter() {
                points.push(Point2D::new(pose.pose.position.x, pose.pose.position.y));
            }
            if let Ok(mut st) = state_path.lock() {
                st.global_path = points;
            }
        }),
    );

    let state_scan = state.clone();
    let log_scan = Logger::new("dwa_planner_node");
    selector.add_subscriber(
        scan_sub,
        Box::new(move |msg| {
            let mut st = match state_scan.lock() {
                Ok(guard) => guard,
                Err(_) => {
                    pr_warn!(log_scan, "failed to lock DWA state on scan");
                    return;
                }
            };
            if let Some(odom) = st.odom {
                st.obstacles = build_obstacles_from_scan(&msg, odom);
            }
        }),
    );

    let state_odom = state.clone();
    let planner_odom = planner.clone();
    let log_odom = Logger::new("dwa_planner_node");
    let pub_odom = cmd_pub;
    selector.add_subscriber(
        odom_sub,
        Box::new(move |msg| {
            let q = &msg.pose.pose.orientation;
            let yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w);
            let odom = OdomState {
                x: msg.pose.pose.position.x,
                y: msg.pose.pose.position.y,
                yaw,
                v: msg.twist.twist.linear.x,
                omega: msg.twist.twist.angular.z,
            };

            let (goal, obstacles) = {
                let mut st = match state_odom.lock() {
                    Ok(guard) => guard,
                    Err(_) => {
                        pr_warn!(log_odom, "failed to lock DWA state on odom");
                        return;
                    }
                };
                st.odom = Some(odom);
                let goal = nearest_goal(&st.global_path, odom.x, odom.y);
                (goal, st.obstacles.clone())
            };

            let goal = match goal {
                Some(goal) => goal,
                None => {
                    pr_warn!(log_odom, "planned path is empty; skipping DWA step");
                    return;
                }
            };

            let mut planner = match planner_odom.lock() {
                Ok(guard) => guard,
                Err(_) => {
                    pr_warn!(log_odom, "failed to lock DWA planner");
                    return;
                }
            };

            if planner
                .try_set_state(DWAState::new(odom.x, odom.y, odom.yaw, odom.v, odom.omega))
                .is_err()
            {
                pr_warn!(log_odom, "invalid odometry state for DWA");
                return;
            }
            let _ = planner.try_set_goal(goal);
            let _ = planner.try_set_obstacles(obstacles);

            let control = match planner.try_plan_step() {
                Ok(control) => control,
                Err(err) => {
                    pr_warn!(log_odom, "DWA planning failed: {}", err);
                    return;
                }
            };

            let mut cmd = match geometry_msgs::msg::Twist::new() {
                Some(msg) => msg,
                None => {
                    pr_warn!(log_odom, "failed to allocate Twist message");
                    return;
                }
            };
            cmd.linear.x = control[0];
            cmd.angular.z = control[1];
            let _ = pub_odom.send(&cmd);
        }),
    );

    loop {
        selector.wait()?;
    }
}
