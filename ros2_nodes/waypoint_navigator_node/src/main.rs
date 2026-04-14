use rust_robotics_core::Point2D;
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::{geometry_msgs, nav_msgs},
    pr_info, pr_warn,
};
use std::{
    env, io,
    sync::{Arc, Mutex},
};

const DEFAULT_WAYPOINTS: &str = "0.5,0.0;0.5,0.5;0.0,0.5";
const DEFAULT_GOAL_TOLERANCE: f64 = 0.35;
const GOAL_FRAME_ID: &str = "map";

#[derive(Clone)]
struct MissionConfig {
    waypoints: Vec<Point2D>,
    goal_tolerance: f64,
    loop_mission: bool,
}

#[derive(Default)]
struct NavigatorState {
    current_waypoint_idx: Option<usize>,
    mission_completed: bool,
}

enum MissionEvent {
    Start { next_idx: usize },
    Advance { reached_idx: usize, next_idx: usize },
    Loop { reached_idx: usize, next_idx: usize },
    Complete { reached_idx: usize },
}

fn parse_waypoints(raw: &str) -> Result<Vec<Point2D>, String> {
    let mut waypoints = Vec::new();

    for (index, entry) in raw.split(';').enumerate() {
        let trimmed = entry.trim();
        if trimmed.is_empty() {
            continue;
        }

        let mut parts = trimmed.split(',').map(str::trim);
        let x = parts
            .next()
            .ok_or_else(|| format!("waypoint {} is missing x", index))?;
        let y = parts
            .next()
            .ok_or_else(|| format!("waypoint {} is missing y", index))?;
        if parts.next().is_some() {
            return Err(format!(
                "waypoint {} must be 'x,y' but got '{}'",
                index, trimmed
            ));
        }

        let x = x
            .parse::<f64>()
            .map_err(|_| format!("waypoint {} has invalid x '{}'", index, x))?;
        let y = y
            .parse::<f64>()
            .map_err(|_| format!("waypoint {} has invalid y '{}'", index, y))?;
        if !x.is_finite() || !y.is_finite() {
            return Err(format!(
                "waypoint {} must contain finite coordinates, got '{}'",
                index, trimmed
            ));
        }
        waypoints.push(Point2D::new(x, y));
    }

    if waypoints.is_empty() {
        return Err("mission must contain at least one waypoint".to_string());
    }

    Ok(waypoints)
}

fn parse_bool(raw: &str) -> Option<bool> {
    match raw.trim().to_ascii_lowercase().as_str() {
        "1" | "true" | "yes" | "on" => Some(true),
        "0" | "false" | "no" | "off" => Some(false),
        _ => None,
    }
}

fn load_mission_config() -> Result<MissionConfig, String> {
    let waypoint_spec = env::var("WAYPOINT_NAV_WAYPOINTS")
        .ok()
        .filter(|value| !value.trim().is_empty())
        .unwrap_or_else(|| DEFAULT_WAYPOINTS.to_string());
    let waypoints = parse_waypoints(&waypoint_spec)?;

    let goal_tolerance = match env::var("WAYPOINT_NAV_GOAL_TOLERANCE") {
        Ok(raw) if !raw.trim().is_empty() => raw
            .trim()
            .parse::<f64>()
            .map_err(|_| format!("invalid WAYPOINT_NAV_GOAL_TOLERANCE '{}'", raw))?,
        _ => DEFAULT_GOAL_TOLERANCE,
    };
    if !goal_tolerance.is_finite() || goal_tolerance <= 0.0 {
        return Err(format!(
            "WAYPOINT_NAV_GOAL_TOLERANCE must be positive and finite, got {}",
            goal_tolerance
        ));
    }

    let loop_mission = match env::var("WAYPOINT_NAV_LOOP") {
        Ok(raw) if !raw.trim().is_empty() => {
            parse_bool(&raw).ok_or_else(|| format!("invalid WAYPOINT_NAV_LOOP '{}'", raw))?
        }
        _ => false,
    };

    Ok(MissionConfig {
        waypoints,
        goal_tolerance,
        loop_mission,
    })
}

fn format_waypoints(waypoints: &[Point2D]) -> String {
    waypoints
        .iter()
        .enumerate()
        .map(|(idx, point)| format!("{}:({:.2},{:.2})", idx + 1, point.x, point.y))
        .collect::<Vec<_>>()
        .join(" -> ")
}

fn distance(a: Point2D, b: Point2D) -> f64 {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()
}

fn publish_goal(
    publisher: &safe_drive::topic::publisher::Publisher<geometry_msgs::msg::PoseStamped>,
    waypoint: Point2D,
) -> Result<(), DynError> {
    let mut msg = geometry_msgs::msg::PoseStamped::new().ok_or("failed to allocate PoseStamped")?;
    let _ = msg.header.frame_id.assign(GOAL_FRAME_ID);
    msg.pose.position.x = waypoint.x;
    msg.pose.position.y = waypoint.y;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.w = 1.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    publisher.send(&msg)?;
    Ok(())
}

fn main() -> Result<(), DynError> {
    let mission =
        load_mission_config().map_err(|err| io::Error::new(io::ErrorKind::InvalidInput, err))?;

    let ctx = Context::new()?;
    let node = ctx.create_node("waypoint_navigator_node", None, Default::default())?;

    let odom_sub = node.create_subscriber::<nav_msgs::msg::Odometry>("/odom", None)?;
    let goal_pub = node.create_publisher::<geometry_msgs::msg::PoseStamped>("/goal_pose", None)?;

    let state = Arc::new(Mutex::new(NavigatorState::default()));
    let logger = Logger::new("waypoint_navigator_node");
    pr_info!(
        logger,
        "waypoint navigator started: {} waypoint(s), tolerance={:.2} m, loop={}",
        mission.waypoints.len(),
        mission.goal_tolerance,
        mission.loop_mission
    );
    pr_info!(logger, "mission: {}", format_waypoints(&mission.waypoints));

    let mut selector = ctx.create_selector()?;
    let mission_cb = mission.clone();
    let state_odom = state.clone();
    let log_odom = Logger::new("waypoint_navigator_node");
    selector.add_subscriber(
        odom_sub,
        Box::new(move |msg| {
            let robot_pose = Point2D::new(msg.pose.pose.position.x, msg.pose.pose.position.y);

            let event = {
                let mut st = match state_odom.lock() {
                    Ok(guard) => guard,
                    Err(_) => {
                        pr_warn!(log_odom, "failed to lock mission state on /odom");
                        return;
                    }
                };

                if st.mission_completed {
                    None
                } else {
                    match st.current_waypoint_idx {
                        None => {
                            st.current_waypoint_idx = Some(0);
                            Some(MissionEvent::Start { next_idx: 0 })
                        }
                        Some(current_idx) => {
                            let current_goal = mission_cb.waypoints[current_idx];
                            if distance(robot_pose, current_goal) > mission_cb.goal_tolerance {
                                None
                            } else if current_idx + 1 < mission_cb.waypoints.len() {
                                let next_idx = current_idx + 1;
                                st.current_waypoint_idx = Some(next_idx);
                                Some(MissionEvent::Advance {
                                    reached_idx: current_idx,
                                    next_idx,
                                })
                            } else if mission_cb.loop_mission {
                                st.current_waypoint_idx = Some(0);
                                Some(MissionEvent::Loop {
                                    reached_idx: current_idx,
                                    next_idx: 0,
                                })
                            } else {
                                st.current_waypoint_idx = None;
                                st.mission_completed = true;
                                Some(MissionEvent::Complete {
                                    reached_idx: current_idx,
                                })
                            }
                        }
                    }
                }
            };

            match event {
                Some(MissionEvent::Start { next_idx }) => {
                    let goal = mission_cb.waypoints[next_idx];
                    if let Err(err) = publish_goal(&goal_pub, goal) {
                        pr_warn!(log_odom, "failed to publish first goal: {}", err);
                        return;
                    }
                    pr_info!(
                        log_odom,
                        "starting mission at waypoint {}/{} -> ({:.2}, {:.2})",
                        next_idx + 1,
                        mission_cb.waypoints.len(),
                        goal.x,
                        goal.y
                    );
                }
                Some(MissionEvent::Advance {
                    reached_idx,
                    next_idx,
                }) => {
                    let reached = mission_cb.waypoints[reached_idx];
                    let next = mission_cb.waypoints[next_idx];
                    if let Err(err) = publish_goal(&goal_pub, next) {
                        pr_warn!(log_odom, "failed to publish next goal: {}", err);
                        return;
                    }
                    pr_info!(
                        log_odom,
                        "reached waypoint {}/{} at ({:.2}, {:.2}); next -> ({:.2}, {:.2})",
                        reached_idx + 1,
                        mission_cb.waypoints.len(),
                        reached.x,
                        reached.y,
                        next.x,
                        next.y
                    );
                }
                Some(MissionEvent::Loop {
                    reached_idx,
                    next_idx,
                }) => {
                    let reached = mission_cb.waypoints[reached_idx];
                    let next = mission_cb.waypoints[next_idx];
                    if let Err(err) = publish_goal(&goal_pub, next) {
                        pr_warn!(log_odom, "failed to publish looped goal: {}", err);
                        return;
                    }
                    pr_info!(
                        log_odom,
                        "completed mission lap at waypoint {}/{} ({:.2}, {:.2}); restarting -> ({:.2}, {:.2})",
                        reached_idx + 1,
                        mission_cb.waypoints.len(),
                        reached.x,
                        reached.y,
                        next.x,
                        next.y
                    );
                }
                Some(MissionEvent::Complete { reached_idx }) => {
                    let reached = mission_cb.waypoints[reached_idx];
                    pr_info!(
                        log_odom,
                        "mission complete at waypoint {}/{} ({:.2}, {:.2})",
                        reached_idx + 1,
                        mission_cb.waypoints.len(),
                        reached.x,
                        reached.y
                    );
                }
                None => {}
            }
        }),
    );

    loop {
        selector.wait()?;
    }
}

#[cfg(test)]
mod tests {
    use super::{parse_bool, parse_waypoints};

    #[test]
    fn parse_waypoints_accepts_semicolon_delimited_points() {
        let mission = parse_waypoints("0.5,0.0; 1.0, -0.5; -2.5,3.0").unwrap();
        assert_eq!(mission.len(), 3);
        assert!((mission[0].x - 0.5).abs() < 1e-9);
        assert!((mission[1].y + 0.5).abs() < 1e-9);
        assert!((mission[2].x + 2.5).abs() < 1e-9);
    }

    #[test]
    fn parse_waypoints_rejects_bad_pairs() {
        let err = parse_waypoints("0.5;1.0,2.0").unwrap_err();
        assert!(err.contains("missing y"));
    }

    #[test]
    fn parse_bool_accepts_common_values() {
        assert_eq!(parse_bool("true"), Some(true));
        assert_eq!(parse_bool("ON"), Some(true));
        assert_eq!(parse_bool("0"), Some(false));
        assert_eq!(parse_bool("no"), Some(false));
        assert_eq!(parse_bool("maybe"), None);
    }
}
