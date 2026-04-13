use rust_robotics_core::Point2D;
use rust_robotics_planning::{AStarConfig, AStarPlanner};
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::{geometry_msgs, nav_msgs},
    pr_info, pr_warn,
};
use std::sync::{Arc, Mutex};

const GRID_RESOLUTION: f64 = 0.5;
const ROBOT_RADIUS: f64 = 0.4;

#[derive(Default)]
struct PlannerState {
    robot_pose: Option<Point2D>,
}

fn create_static_obstacles() -> (Vec<f64>, Vec<f64>) {
    let mut ox = Vec::new();
    let mut oy = Vec::new();

    for i in -20..=20 {
        ox.push(i as f64);
        oy.push(-20.0);
        ox.push(i as f64);
        oy.push(20.0);
        ox.push(-20.0);
        oy.push(i as f64);
        ox.push(20.0);
        oy.push(i as f64);
    }

    for i in -8..=8 {
        ox.push(0.0);
        oy.push(i as f64);
    }

    (ox, oy)
}

fn world_to_grid(point: Point2D, resolution: f64) -> (i32, i32) {
    (
        (point.x / resolution).round() as i32,
        (point.y / resolution).round() as i32,
    )
}

fn grid_to_world(ix: i32, iy: i32, resolution: f64) -> Point2D {
    Point2D::new(ix as f64 * resolution, iy as f64 * resolution)
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

fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("path_planner_node", None, Default::default())?;

    let goal_sub = node.create_subscriber::<geometry_msgs::msg::PoseStamped>("/goal_pose", None)?;
    let robot_sub =
        node.create_subscriber::<geometry_msgs::msg::PoseStamped>("/robot_pose", None)?;
    let path_pub = node.create_publisher::<nav_msgs::msg::Path>("/planned_path", None)?;

    let (ox, oy) = create_static_obstacles();
    let planner = Arc::new(AStarPlanner::new(
        &ox,
        &oy,
        AStarConfig {
            resolution: GRID_RESOLUTION,
            robot_radius: ROBOT_RADIUS,
            ..Default::default()
        },
    ));
    let state = Arc::new(Mutex::new(PlannerState::default()));
    let logger = Logger::new("path_planner_node");

    pr_info!(logger, "path planner started");

    let mut selector = ctx.create_selector()?;

    let state_robot = state.clone();
    selector.add_subscriber(
        robot_sub,
        Box::new(move |msg| {
            if let Ok(mut st) = state_robot.lock() {
                st.robot_pose = Some(Point2D::new(msg.pose.position.x, msg.pose.position.y));
            }
        }),
    );

    let planner_goal = planner.clone();
    let state_goal = state.clone();
    let pub_goal = path_pub;
    let log_goal = Logger::new("path_planner_node");
    selector.add_subscriber(
        goal_sub,
        Box::new(move |msg| {
            let goal_world = Point2D::new(msg.pose.position.x, msg.pose.position.y);
            let start_world = {
                let guard = match state_goal.lock() {
                    Ok(guard) => guard,
                    Err(_) => {
                        pr_warn!(log_goal, "failed to lock planner state");
                        return;
                    }
                };
                match guard.robot_pose {
                    Some(p) => p,
                    None => {
                        pr_warn!(log_goal, "robot pose is not available yet");
                        return;
                    }
                }
            };

            let (sx, sy) = world_to_grid(start_world, GRID_RESOLUTION);
            let (gx, gy) = world_to_grid(goal_world, GRID_RESOLUTION);
            let start = grid_to_world(sx, sy, GRID_RESOLUTION);
            let goal = grid_to_world(gx, gy, GRID_RESOLUTION);

            match planner_goal.plan(start, goal) {
                Ok(path) => {
                    if let Err(err) = publish_path(&pub_goal, &path.points) {
                        pr_warn!(log_goal, "failed to publish path: {}", err);
                        return;
                    }
                    pr_info!(
                        log_goal,
                        "planned path: {} points, start=({:.2},{:.2}) goal=({:.2},{:.2})",
                        path.points.len(),
                        start.x,
                        start.y,
                        goal.x,
                        goal.y
                    );
                }
                Err(err) => {
                    pr_warn!(log_goal, "A* failed: {}", err);
                }
            }
        }),
    );

    loop {
        selector.wait()?;
    }
}
