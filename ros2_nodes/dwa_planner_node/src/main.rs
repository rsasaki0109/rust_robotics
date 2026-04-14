use rust_robotics_core::Point2D;
use rust_robotics_planning::dwa::DWAState;
use rust_robotics_planning::{DWAConfig, DWAPlanner};
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::{geometry_msgs, nav_msgs, sensor_msgs},
    qos::{policy::DurabilityPolicy, Profile},
    pr_info, pr_warn,
};
use std::{env, sync::{Arc, Mutex}};

const DEFAULT_ODOM_TOPIC: &str = "/odom";
const DEFAULT_GOAL_THRESHOLD: f64 = 0.3;
const DEFAULT_ROBOT_RADIUS: f64 = 0.35;

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
    warned_missing_path: bool,
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

fn load_f64_env(name: &str, default: f64) -> Result<f64, String> {
    match env::var(name) {
        Ok(raw) if !raw.trim().is_empty() => {
            let value = raw
                .trim()
                .parse::<f64>()
                .map_err(|_| format!("invalid {} '{}'", name, raw))?;
            if !value.is_finite() || value <= 0.0 {
                return Err(format!(
                    "{} must be positive and finite, got {}",
                    name, value
                ));
            }
            Ok(value)
        }
        _ => Ok(default),
    }
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
    let odom_topic = env::var("RUST_NAV_ODOM_TOPIC")
        .ok()
        .filter(|value| !value.trim().is_empty())
        .unwrap_or_else(|| DEFAULT_ODOM_TOPIC.to_string());
    let goal_threshold = load_f64_env("DWA_GOAL_THRESHOLD", DEFAULT_GOAL_THRESHOLD)
        .map_err(|err| std::io::Error::new(std::io::ErrorKind::InvalidInput, err))?;
    let robot_radius = load_f64_env("DWA_ROBOT_RADIUS", DEFAULT_ROBOT_RADIUS)
        .map_err(|err| std::io::Error::new(std::io::ErrorKind::InvalidInput, err))?;

    let mut path_qos = Profile::default();
    path_qos.durability = DurabilityPolicy::TransientLocal;

    let scan_sub = node.create_subscriber::<sensor_msgs::msg::LaserScan>("/scan", None)?;
    let odom_sub = node.create_subscriber::<nav_msgs::msg::Odometry>(&odom_topic, None)?;
    let path_sub = node.create_subscriber::<nav_msgs::msg::Path>("/planned_path", Some(path_qos))?;
    let cmd_pub = node.create_publisher::<geometry_msgs::msg::Twist>("/cmd_vel", None)?;

    let mut config = DWAConfig::default();
    config.goal_threshold = goal_threshold;
    config.robot_radius = robot_radius;

    let planner = Arc::new(Mutex::new(DWAPlanner::new(config)));
    let state = Arc::new(Mutex::new(PlannerState::default()));
    let logger = Logger::new("dwa_planner_node");
    pr_info!(
        logger,
        "dwa planner started (odom topic: {}, goal_threshold={:.2}, robot_radius={:.2})",
        odom_topic,
        goal_threshold,
        robot_radius
    );

    let mut selector = ctx.create_selector()?;

    let state_path = state.clone();
    let log_path = Logger::new("dwa_planner_node");
    selector.add_subscriber(
        path_sub,
        Box::new(move |msg| {
            let mut points = Vec::with_capacity(msg.poses.len());
            for pose in msg.poses.iter() {
                points.push(Point2D::new(pose.pose.position.x, pose.pose.position.y));
            }
            if let Ok(mut st) = state_path.lock() {
                let had_path = !st.global_path.is_empty();
                st.global_path = points;
                if st.global_path.is_empty() {
                    if had_path {
                        st.warned_missing_path = true;
                        pr_warn!(log_path, "planned path cleared");
                    }
                } else {
                    st.warned_missing_path = false;
                    pr_info!(
                        log_path,
                        "received planned path with {} points",
                        st.global_path.len()
                    );
                }
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

            let (goal, obstacles, should_warn_missing_path) = {
                let mut st = match state_odom.lock() {
                    Ok(guard) => guard,
                    Err(_) => {
                        pr_warn!(log_odom, "failed to lock DWA state on odom");
                        return;
                    }
                };
                st.odom = Some(odom);
                let goal = nearest_goal(&st.global_path, odom.x, odom.y);
                let should_warn_missing_path = goal.is_none() && !st.warned_missing_path;
                if goal.is_none() {
                    st.warned_missing_path = true;
                }
                (goal, st.obstacles.clone(), should_warn_missing_path)
            };

            let goal = match goal {
                Some(goal) => goal,
                None => {
                    if should_warn_missing_path {
                        pr_warn!(log_odom, "planned path is empty; waiting for planner output");
                    }
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
