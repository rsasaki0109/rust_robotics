use rust_robotics_core::Point2D;
use rust_robotics_planning::{AStarConfig, AStarPlanner};
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::{geometry_msgs, nav_msgs},
    qos::{policy::DurabilityPolicy, Profile},
    pr_info, pr_warn,
};
use std::env;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const DEFAULT_ODOM_TOPIC: &str = "/odom";
const ROBOT_RADIUS: f64 = 0.4;
const MAP_OCCUPANCY_THRESHOLD: i8 = 60;
const MAP_REBUILD_LOG_INTERVAL: Duration = Duration::from_secs(5);

#[derive(Default)]
struct PlannerState {
    robot_pose: Option<Point2D>,
    goal_pose: Option<Point2D>,
    planner: Option<Arc<AStarPlanner>>,
    last_map_log_at: Option<Instant>,
}

struct MapPlannerUpdate {
    planner: Arc<AStarPlanner>,
    resolution: f64,
    width: usize,
    height: usize,
    occupied_cells: usize,
}

fn map_cell_center(origin: f64, index: usize, resolution: f64) -> f64 {
    origin + (index as f64 + 0.5) * resolution
}

fn rebuild_planner_from_map(
    map: &nav_msgs::msg::OccupancyGrid,
    threshold: i8,
) -> Result<MapPlannerUpdate, String> {
    let info = &map.info;
    let resolution = info.resolution as f64;
    if !resolution.is_finite() || resolution <= 0.0 {
        return Err(format!("invalid map resolution: {}", info.resolution));
    }

    let width = info.width as usize;
    let height = info.height as usize;
    if width == 0 || height == 0 {
        return Err("map dimensions must be non-zero".to_string());
    }

    let expected_cells = width
        .checked_mul(height)
        .ok_or_else(|| "map dimensions overflowed usize".to_string())?;
    if map.data.len() != expected_cells {
        return Err(format!(
            "map payload length mismatch: expected {}, got {}",
            expected_cells,
            map.data.len()
        ));
    }

    let origin_x = info.origin.position.x;
    let origin_y = info.origin.position.y;
    let max_x = origin_x + width as f64 * resolution;
    let max_y = origin_y + height as f64 * resolution;

    let mut ox = Vec::with_capacity((width + height) * 4);
    let mut oy = Vec::with_capacity((width + height) * 4);

    // Add a perimeter so A* sees the full OccupancyGrid bounds as its search area.
    for ix in 0..=width {
        let x = origin_x + ix as f64 * resolution;
        ox.push(x);
        oy.push(origin_y);
        ox.push(x);
        oy.push(max_y);
    }
    for iy in 0..=height {
        let y = origin_y + iy as f64 * resolution;
        ox.push(origin_x);
        oy.push(y);
        ox.push(max_x);
        oy.push(y);
    }

    let mut occupied_cells = 0;
    for (index, cell) in map.data.iter().enumerate() {
        if *cell < threshold {
            continue;
        }

        let grid_x = index % width;
        let grid_y = index / width;
        ox.push(map_cell_center(origin_x, grid_x, resolution));
        oy.push(map_cell_center(origin_y, grid_y, resolution));
        occupied_cells += 1;
    }

    let planner = AStarPlanner::try_new(
        &ox,
        &oy,
        AStarConfig {
            resolution,
            robot_radius: ROBOT_RADIUS,
            ..Default::default()
        },
    )
    .map_err(|err| err.to_string())?;

    Ok(MapPlannerUpdate {
        planner: Arc::new(planner),
        resolution,
        width,
        height,
        occupied_cells,
    })
}

fn publish_path(
    publisher: &safe_drive::topic::publisher::Publisher<nav_msgs::msg::Path>,
    path_points: &[Point2D],
) -> Result<(), DynError> {
    let mut path_msg = nav_msgs::msg::Path::new().ok_or("failed to allocate nav_msgs/Path")?;
    let _ = path_msg.header.frame_id.assign("map");

    let mut pose_seq = geometry_msgs::msg::PoseStampedSeq::<0>::new(path_points.len())
        .ok_or("failed to allocate pose sequence")?;

    for (idx, point) in path_points.iter().enumerate() {
        let pose = &mut pose_seq.as_slice_mut()[idx];
        let _ = pose.header.frame_id.assign("map");
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
    }

    path_msg.poses = pose_seq;
    publisher.send(&path_msg)?;
    Ok(())
}

fn attempt_plan(
    planner: Arc<AStarPlanner>,
    start: Point2D,
    goal: Point2D,
    publisher: &safe_drive::topic::publisher::Publisher<nav_msgs::msg::Path>,
    logger: &Logger,
    log_success: bool,
) {
    match planner.plan(start, goal) {
        Ok(path) => {
            if let Err(err) = publish_path(publisher, &path.points) {
                pr_warn!(logger, "failed to publish path: {}", err);
                return;
            }
            if log_success {
                pr_info!(
                    logger,
                    "planned path: {} points, start=({:.2},{:.2}) goal=({:.2},{:.2})",
                    path.points.len(),
                    start.x,
                    start.y,
                    goal.x,
                    goal.y
                );
            }
        }
        Err(err) => {
            pr_warn!(
                logger,
                "A* failed for start=({:.2},{:.2}) goal=({:.2},{:.2}): {}",
                start.x,
                start.y,
                goal.x,
                goal.y,
                err
            );
        }
    }
}

fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("path_planner_node", None, Default::default())?;
    let odom_topic = env::var("RUST_NAV_ODOM_TOPIC")
        .ok()
        .filter(|value| !value.trim().is_empty())
        .unwrap_or_else(|| DEFAULT_ODOM_TOPIC.to_string());

    let goal_sub = node.create_subscriber::<geometry_msgs::msg::PoseStamped>("/goal_pose", None)?;
    let odom_sub = node.create_subscriber::<nav_msgs::msg::Odometry>(&odom_topic, None)?;
    let map_sub = node.create_subscriber::<nav_msgs::msg::OccupancyGrid>("/map", None)?;
    let mut path_qos = Profile::default();
    path_qos.durability = DurabilityPolicy::TransientLocal;
    let path_pub = Arc::new(node.create_publisher::<nav_msgs::msg::Path>(
        "/planned_path",
        Some(path_qos),
    )?);

    let state = Arc::new(Mutex::new(PlannerState::default()));
    let logger = Logger::new("path_planner_node");

    pr_info!(logger, "path planner started (odom topic: {})", odom_topic);

    let mut selector = ctx.create_selector()?;

    let state_odom = state.clone();
    selector.add_subscriber(
        odom_sub,
        Box::new(move |msg| {
            if let Ok(mut st) = state_odom.lock() {
                st.robot_pose = Some(Point2D::new(
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                ));
            }
        }),
    );

    let state_map = state.clone();
    let pub_map = path_pub.clone();
    let log_map = Logger::new("path_planner_node");
    selector.add_subscriber(
        map_sub,
        Box::new(move |msg| {
            let planner_update = match rebuild_planner_from_map(&msg, MAP_OCCUPANCY_THRESHOLD) {
                Ok(update) => update,
                Err(err) => {
                    pr_warn!(log_map, "failed to rebuild planner from /map: {}", err);
                    return;
                }
            };

            let (replan_request, should_log_rebuild) = {
                let mut st = match state_map.lock() {
                    Ok(guard) => guard,
                    Err(_) => {
                        pr_warn!(log_map, "failed to lock planner state on /map");
                        return;
                    }
                };
                st.planner = Some(planner_update.planner.clone());
                let now = Instant::now();
                let should_log_rebuild = st.last_map_log_at.map_or(true, |last_log_at| {
                    now.duration_since(last_log_at) >= MAP_REBUILD_LOG_INTERVAL
                });
                if should_log_rebuild {
                    st.last_map_log_at = Some(now);
                }
                (
                    match (st.robot_pose, st.goal_pose) {
                        (Some(start), Some(goal)) => {
                            Some((planner_update.planner.clone(), start, goal))
                        }
                        _ => None,
                    },
                    should_log_rebuild,
                )
            };

            if should_log_rebuild {
                pr_info!(
                    log_map,
                    "planner rebuilt from /map: {}x{} cells @ {:.2} m, {} occupied cells",
                    planner_update.width,
                    planner_update.height,
                    planner_update.resolution,
                    planner_update.occupied_cells
                );
            }

            if let Some((planner, start, goal)) = replan_request {
                attempt_plan(planner, start, goal, pub_map.as_ref(), &log_map, false);
            }
        }),
    );

    let state_goal = state.clone();
    let pub_goal = path_pub;
    let log_goal = Logger::new("path_planner_node");
    let odom_topic_goal = odom_topic.clone();
    selector.add_subscriber(
        goal_sub,
        Box::new(move |msg| {
            let goal_world = Point2D::new(msg.pose.position.x, msg.pose.position.y);
            let plan_request = {
                let mut st = match state_goal.lock() {
                    Ok(guard) => guard,
                    Err(_) => {
                        pr_warn!(log_goal, "failed to lock planner state on /goal_pose");
                        return;
                    }
                };
                st.goal_pose = Some(goal_world);

                let start = match st.robot_pose {
                    Some(pose) => pose,
                    None => {
                        pr_warn!(
                            log_goal,
                            "robot pose from {} is not available yet",
                            odom_topic_goal
                        );
                        return;
                    }
                };
                let planner = match st.planner.as_ref() {
                    Some(planner) => planner.clone(),
                    None => {
                        pr_warn!(log_goal, "planner is not ready yet; waiting for /map");
                        return;
                    }
                };
                (planner, start, goal_world)
            };

            attempt_plan(
                plan_request.0,
                plan_request.1,
                plan_request.2,
                pub_goal.as_ref(),
                &log_goal,
                true,
            );
        }),
    );

    loop {
        selector.wait()?;
    }
}
