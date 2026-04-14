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
    time::{Duration, Instant},
};

const DEFAULT_ODOM_TOPIC: &str = "/odom";
const DEFAULT_WAYPOINTS: &str = "0.5,0.0;0.5,0.5;0.0,0.5";
const DEFAULT_GOAL_TOLERANCE: f64 = 0.35;
const DEFAULT_WAYPOINT_FRAME: &str = "map";
const GOAL_FRAME_ID: &str = "map";
const GOAL_REPUBLISH_INTERVAL: Duration = Duration::from_secs(2);
const GOAL_PROGRESS_ARM_DISTANCE: f64 = 0.05;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MissionFrame {
    Map,
    RelativeStart,
}

impl MissionFrame {
    fn as_str(self) -> &'static str {
        match self {
            MissionFrame::Map => "map",
            MissionFrame::RelativeStart => "relative_start",
        }
    }
}

#[derive(Clone)]
struct MissionConfig {
    waypoints: Vec<Point2D>,
    goal_tolerance: f64,
    loop_mission: bool,
    frame: MissionFrame,
}

#[derive(Default)]
struct NavigatorState {
    current_waypoint_idx: Option<usize>,
    mission_completed: bool,
    last_goal_publish_at: Option<Instant>,
    anchor_pose: Option<Point2D>,
    active_goal: Option<ActiveGoalState>,
}

#[derive(Clone, Copy, Debug)]
struct ActiveGoalState {
    published_goal: Point2D,
    published_robot_pose: Point2D,
    completion_armed: bool,
}

enum MissionEvent {
    Start {
        next_idx: usize,
        goal: Point2D,
    },
    Advance {
        reached_idx: usize,
        reached_goal: Point2D,
        next_idx: usize,
        next_goal: Point2D,
    },
    Loop {
        reached_idx: usize,
        reached_goal: Point2D,
        next_idx: usize,
        next_goal: Point2D,
    },
    Complete {
        reached_idx: usize,
        reached_goal: Point2D,
    },
    Republish {
        current_idx: usize,
        goal: Point2D,
    },
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

fn parse_mission_frame(raw: &str) -> Option<MissionFrame> {
    let normalized = raw.trim().to_ascii_lowercase().replace(['-', ' '], "_");
    match normalized.as_str() {
        "map" | "absolute" => Some(MissionFrame::Map),
        "relative" | "relative_start" | "start_relative" => Some(MissionFrame::RelativeStart),
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

    let frame = match env::var("WAYPOINT_NAV_FRAME") {
        Ok(raw) if !raw.trim().is_empty() => parse_mission_frame(&raw)
            .ok_or_else(|| format!("invalid WAYPOINT_NAV_FRAME '{}'", raw))?,
        _ => parse_mission_frame(DEFAULT_WAYPOINT_FRAME)
            .expect("DEFAULT_WAYPOINT_FRAME must be valid"),
    };

    Ok(MissionConfig {
        waypoints,
        goal_tolerance,
        loop_mission,
        frame,
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

fn should_republish_goal(last_goal_publish_at: Option<Instant>, now: Instant) -> bool {
    match last_goal_publish_at {
        Some(last_sent_at) => now.duration_since(last_sent_at) >= GOAL_REPUBLISH_INTERVAL,
        None => true,
    }
}

fn resolve_waypoint(
    waypoint: Point2D,
    frame: MissionFrame,
    anchor_pose: Option<Point2D>,
) -> Point2D {
    match frame {
        MissionFrame::Map => waypoint,
        MissionFrame::RelativeStart => {
            let anchor = anchor_pose.unwrap_or(Point2D::origin());
            Point2D::new(anchor.x + waypoint.x, anchor.y + waypoint.y)
        }
    }
}

fn new_active_goal(goal: Point2D, robot_pose: Point2D, goal_tolerance: f64) -> ActiveGoalState {
    ActiveGoalState {
        published_goal: goal,
        published_robot_pose: robot_pose,
        completion_armed: distance(robot_pose, goal) > goal_tolerance,
    }
}

fn arm_goal_completion_if_progressed(
    active_goal: &mut ActiveGoalState,
    robot_pose: Point2D,
    goal_tolerance: f64,
) {
    if active_goal.completion_armed {
        return;
    }

    let moved_since_publish = distance(robot_pose, active_goal.published_robot_pose);
    let remaining_distance = distance(robot_pose, active_goal.published_goal);
    if moved_since_publish >= GOAL_PROGRESS_ARM_DISTANCE || remaining_distance > goal_tolerance {
        active_goal.completion_armed = true;
    }
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
    let odom_topic = env::var("RUST_NAV_ODOM_TOPIC")
        .ok()
        .filter(|value| !value.trim().is_empty())
        .unwrap_or_else(|| DEFAULT_ODOM_TOPIC.to_string());

    let ctx = Context::new()?;
    let node = ctx.create_node("waypoint_navigator_node", None, Default::default())?;

    let odom_sub = node.create_subscriber::<nav_msgs::msg::Odometry>(&odom_topic, None)?;
    let goal_pub = node.create_publisher::<geometry_msgs::msg::PoseStamped>("/goal_pose", None)?;

    let state = Arc::new(Mutex::new(NavigatorState::default()));
    let logger = Logger::new("waypoint_navigator_node");
    pr_info!(
        logger,
        "waypoint navigator started: {} waypoint(s), tolerance={:.2} m, loop={}, frame={}",
        mission.waypoints.len(),
        mission.goal_tolerance,
        mission.loop_mission,
        mission.frame.as_str()
    );
    pr_info!(logger, "waypoint odom topic: {}", odom_topic);
    pr_info!(logger, "mission: {}", format_waypoints(&mission.waypoints));

    let mut selector = ctx.create_selector()?;
    let mission_cb = mission.clone();
    let state_odom = state.clone();
    let log_odom = Logger::new("waypoint_navigator_node");
    selector.add_subscriber(
        odom_sub,
        Box::new(move |msg| {
            let robot_pose = Point2D::new(msg.pose.pose.position.x, msg.pose.pose.position.y);
            let now = Instant::now();

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
                    if mission_cb.frame == MissionFrame::RelativeStart && st.anchor_pose.is_none() {
                        st.anchor_pose = Some(robot_pose);
                    }

                    match st.current_waypoint_idx {
                        None => {
                            let goal = resolve_waypoint(
                                mission_cb.waypoints[0],
                                mission_cb.frame,
                                st.anchor_pose,
                            );
                            st.current_waypoint_idx = Some(0);
                            st.last_goal_publish_at = Some(now);
                            st.active_goal = Some(new_active_goal(
                                goal,
                                robot_pose,
                                mission_cb.goal_tolerance,
                            ));
                            Some(MissionEvent::Start { next_idx: 0, goal })
                        }
                        Some(current_idx) => {
                            let current_goal = resolve_waypoint(
                                mission_cb.waypoints[current_idx],
                                mission_cb.frame,
                                st.anchor_pose,
                            );
                            let current_distance = distance(robot_pose, current_goal);
                            if let Some(active_goal) = st.active_goal.as_mut() {
                                arm_goal_completion_if_progressed(
                                    active_goal,
                                    robot_pose,
                                    mission_cb.goal_tolerance,
                                );
                            }
                            let completion_armed = st
                                .active_goal
                                .map(|active_goal| active_goal.completion_armed)
                                .unwrap_or(current_distance > mission_cb.goal_tolerance);

                            if current_distance > mission_cb.goal_tolerance || !completion_armed {
                                if should_republish_goal(st.last_goal_publish_at, now) {
                                    st.last_goal_publish_at = Some(now);
                                    Some(MissionEvent::Republish {
                                        current_idx,
                                        goal: current_goal,
                                    })
                                } else {
                                    None
                                }
                            } else if current_idx + 1 < mission_cb.waypoints.len() {
                                let next_idx = current_idx + 1;
                                let next_goal = resolve_waypoint(
                                    mission_cb.waypoints[next_idx],
                                    mission_cb.frame,
                                    st.anchor_pose,
                                );
                                st.current_waypoint_idx = Some(next_idx);
                                st.last_goal_publish_at = Some(now);
                                st.active_goal = Some(new_active_goal(
                                    next_goal,
                                    robot_pose,
                                    mission_cb.goal_tolerance,
                                ));
                                Some(MissionEvent::Advance {
                                    reached_idx: current_idx,
                                    reached_goal: current_goal,
                                    next_idx,
                                    next_goal,
                                })
                            } else if mission_cb.loop_mission {
                                let next_goal = resolve_waypoint(
                                    mission_cb.waypoints[0],
                                    mission_cb.frame,
                                    st.anchor_pose,
                                );
                                st.current_waypoint_idx = Some(0);
                                st.last_goal_publish_at = Some(now);
                                st.active_goal = Some(new_active_goal(
                                    next_goal,
                                    robot_pose,
                                    mission_cb.goal_tolerance,
                                ));
                                Some(MissionEvent::Loop {
                                    reached_idx: current_idx,
                                    reached_goal: current_goal,
                                    next_idx: 0,
                                    next_goal,
                                })
                            } else {
                                st.current_waypoint_idx = None;
                                st.mission_completed = true;
                                st.last_goal_publish_at = None;
                                st.active_goal = None;
                                Some(MissionEvent::Complete {
                                    reached_idx: current_idx,
                                    reached_goal: current_goal,
                                })
                            }
                        }
                    }
                }
            };

            match event {
                Some(MissionEvent::Start { next_idx, goal }) => {
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
                    reached_goal,
                    next_idx,
                    next_goal,
                }) => {
                    if let Err(err) = publish_goal(&goal_pub, next_goal) {
                        pr_warn!(log_odom, "failed to publish next goal: {}", err);
                        return;
                    }
                    pr_info!(
                        log_odom,
                        "reached waypoint {}/{} at ({:.2}, {:.2}); next waypoint {}/{} -> ({:.2}, {:.2})",
                        reached_idx + 1,
                        mission_cb.waypoints.len(),
                        reached_goal.x,
                        reached_goal.y,
                        next_idx + 1,
                        mission_cb.waypoints.len(),
                        next_goal.x,
                        next_goal.y
                    );
                }
                Some(MissionEvent::Loop {
                    reached_idx,
                    reached_goal,
                    next_idx,
                    next_goal,
                }) => {
                    if let Err(err) = publish_goal(&goal_pub, next_goal) {
                        pr_warn!(log_odom, "failed to publish looped goal: {}", err);
                        return;
                    }
                    pr_info!(
                        log_odom,
                        "completed mission lap at waypoint {}/{} ({:.2}, {:.2}); restarting at waypoint {}/{} -> ({:.2}, {:.2})",
                        reached_idx + 1,
                        mission_cb.waypoints.len(),
                        reached_goal.x,
                        reached_goal.y,
                        next_idx + 1,
                        mission_cb.waypoints.len(),
                        next_goal.x,
                        next_goal.y
                    );
                }
                Some(MissionEvent::Complete {
                    reached_idx,
                    reached_goal,
                }) => {
                    pr_info!(
                        log_odom,
                        "mission complete at waypoint {}/{} ({:.2}, {:.2})",
                        reached_idx + 1,
                        mission_cb.waypoints.len(),
                        reached_goal.x,
                        reached_goal.y
                    );
                }
                Some(MissionEvent::Republish { current_idx, goal }) => {
                    if let Err(err) = publish_goal(&goal_pub, goal) {
                        pr_warn!(log_odom, "failed to republish active goal: {}", err);
                        return;
                    }
                    pr_info!(
                        log_odom,
                        "republishing waypoint {}/{} -> ({:.2}, {:.2})",
                        current_idx + 1,
                        mission_cb.waypoints.len(),
                        goal.x,
                        goal.y
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
    use super::{
        arm_goal_completion_if_progressed, new_active_goal, parse_bool, parse_mission_frame,
        parse_waypoints, resolve_waypoint, should_republish_goal, MissionFrame,
        GOAL_PROGRESS_ARM_DISTANCE, GOAL_REPUBLISH_INTERVAL,
    };
    use rust_robotics_core::Point2D;
    use std::time::{Duration, Instant};

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

    #[test]
    fn parse_mission_frame_accepts_relative_aliases() {
        assert_eq!(parse_mission_frame("map"), Some(MissionFrame::Map));
        assert_eq!(
            parse_mission_frame("relative_start"),
            Some(MissionFrame::RelativeStart)
        );
        assert_eq!(
            parse_mission_frame("relative"),
            Some(MissionFrame::RelativeStart)
        );
        assert_eq!(parse_mission_frame("bad"), None);
    }

    #[test]
    fn resolve_waypoint_offsets_relative_start_frame() {
        let resolved = resolve_waypoint(
            Point2D::new(0.5, -0.2),
            MissionFrame::RelativeStart,
            Some(Point2D::new(-2.0, -0.5)),
        );
        assert!((resolved.x + 1.5).abs() < 1e-9);
        assert!((resolved.y + 0.7).abs() < 1e-9);
    }

    #[test]
    fn should_republish_goal_triggers_without_history() {
        assert!(should_republish_goal(None, Instant::now()));
    }

    #[test]
    fn should_republish_goal_waits_for_interval() {
        let now = Instant::now();
        assert!(!should_republish_goal(Some(now), now));
        assert!(!should_republish_goal(
            Some(now),
            now + GOAL_REPUBLISH_INTERVAL - Duration::from_millis(1)
        ));
        assert!(should_republish_goal(
            Some(now),
            now + GOAL_REPUBLISH_INTERVAL
        ));
    }

    #[test]
    fn new_active_goal_requires_progress_when_goal_is_already_close() {
        let active_goal = new_active_goal(Point2D::new(0.1, 0.1), Point2D::new(0.0, 0.0), 0.2);
        assert!(!active_goal.completion_armed);
    }

    #[test]
    fn arm_goal_completion_when_robot_moves_after_publish() {
        let mut active_goal = new_active_goal(Point2D::new(0.1, 0.1), Point2D::new(0.0, 0.0), 0.2);

        arm_goal_completion_if_progressed(
            &mut active_goal,
            Point2D::new(GOAL_PROGRESS_ARM_DISTANCE + 0.01, 0.0),
            0.2,
        );

        assert!(active_goal.completion_armed);
    }

    #[test]
    fn arm_goal_completion_when_robot_is_seen_outside_tolerance() {
        let mut active_goal = new_active_goal(Point2D::new(0.1, 0.1), Point2D::new(0.0, 0.0), 0.2);

        arm_goal_completion_if_progressed(&mut active_goal, Point2D::new(0.5, 0.5), 0.2);

        assert!(active_goal.completion_armed);
    }
}
