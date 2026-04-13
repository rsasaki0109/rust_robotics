use rust_robotics_core::Point2D;
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::{geometry_msgs, nav_msgs, std_msgs},
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
const GOAL_COMPLETION_ARM_DELAY: Duration = Duration::from_millis(500);
const NAVIGATION_CANCEL_TOPIC: &str = "/navigation_cancel";
const DEFAULT_STUCK_TIMEOUT: Duration = Duration::from_secs(6);
const DEFAULT_STUCK_PROGRESS_DISTANCE: f64 = 0.05;
const DEFAULT_MAX_RECOVERY_ATTEMPTS: u32 = 2;
const DEFAULT_RECOVERY_SETTLE_DURATION: Duration = Duration::from_millis(500);
const DEFAULT_RECOVERY_ROTATE_DURATION: Duration = Duration::from_millis(1400);
const DEFAULT_RECOVERY_BACKOFF_DURATION: Duration = Duration::from_millis(900);
const DEFAULT_RECOVERY_ROTATE_SPEED: f64 = 0.7;
const DEFAULT_RECOVERY_BACKOFF_SPEED: f64 = 0.08;

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
    recovery: RecoveryConfig,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MissionStatus {
    Idle,
    Running,
    Recovering,
    Failed,
    Completed,
}

#[derive(Clone)]
struct RecoveryConfig {
    stuck_timeout: Duration,
    progress_distance: f64,
    max_attempts_per_waypoint: u32,
    settle_duration: Duration,
    rotate_duration: Duration,
    backoff_duration: Duration,
    rotate_speed: f64,
    backoff_speed: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RecoveryPhase {
    Settle,
    Rotate,
    Backoff,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RecoveryPhaseTransition {
    None,
    ToRotate,
    ToBackoff,
    Complete,
}

#[derive(Clone, Copy, Debug)]
struct RecoveryState {
    waypoint_idx: usize,
    goal: Point2D,
    attempt: u32,
    phase: RecoveryPhase,
    phase_started_at: Instant,
}

struct NavigatorState {
    mission_status: MissionStatus,
    current_waypoint_idx: Option<usize>,
    current_waypoint_recovery_attempts: u32,
    last_goal_publish_at: Option<Instant>,
    anchor_pose: Option<Point2D>,
    active_goal: Option<ActiveGoalState>,
    recovery: Option<RecoveryState>,
}

impl Default for NavigatorState {
    fn default() -> Self {
        Self {
            mission_status: MissionStatus::Idle,
            current_waypoint_idx: None,
            current_waypoint_recovery_attempts: 0,
            last_goal_publish_at: None,
            anchor_pose: None,
            active_goal: None,
            recovery: None,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct ActiveGoalState {
    published_goal: Point2D,
    published_robot_pose: Point2D,
    last_progress_pose: Point2D,
    published_at: Instant,
    last_progress_at: Instant,
    completion_armed: bool,
}

#[derive(Clone, Copy, Debug)]
struct VelocityCommand {
    linear_x: f64,
    angular_z: f64,
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
    RecoveryStarted {
        current_idx: usize,
        goal: Point2D,
        attempt: u32,
    },
    RecoveryRotating {
        current_idx: usize,
        goal: Point2D,
        attempt: u32,
    },
    RecoveryBackingOff {
        current_idx: usize,
        goal: Point2D,
        attempt: u32,
    },
    RecoveryRetry {
        current_idx: usize,
        goal: Point2D,
        attempt: u32,
    },
    Failed {
        current_idx: usize,
        goal: Point2D,
        attempts: u32,
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
    let recovery = load_recovery_config()?;

    Ok(MissionConfig {
        waypoints,
        goal_tolerance,
        loop_mission,
        frame,
        recovery,
    })
}

fn load_positive_f64_env(name: &str, default: f64) -> Result<f64, String> {
    match env::var(name) {
        Ok(raw) if !raw.trim().is_empty() => {
            let value = raw
                .trim()
                .parse::<f64>()
                .map_err(|_| format!("invalid {} '{}'", name, raw))?;
            if !value.is_finite() || value <= 0.0 {
                return Err(format!("{} must be positive and finite, got {}", name, value));
            }
            Ok(value)
        }
        _ => Ok(default),
    }
}

fn load_duration_env(name: &str, default: Duration) -> Result<Duration, String> {
    let seconds = load_positive_f64_env(name, default.as_secs_f64())?;
    Ok(Duration::from_secs_f64(seconds))
}

fn load_nonnegative_u32_env(name: &str, default: u32) -> Result<u32, String> {
    match env::var(name) {
        Ok(raw) if !raw.trim().is_empty() => raw
            .trim()
            .parse::<u32>()
            .map_err(|_| format!("invalid {} '{}'", name, raw)),
        _ => Ok(default),
    }
}

fn load_recovery_config() -> Result<RecoveryConfig, String> {
    Ok(RecoveryConfig {
        stuck_timeout: load_duration_env("WAYPOINT_NAV_STUCK_TIMEOUT", DEFAULT_STUCK_TIMEOUT)?,
        progress_distance: load_positive_f64_env(
            "WAYPOINT_NAV_STUCK_PROGRESS_DISTANCE",
            DEFAULT_STUCK_PROGRESS_DISTANCE,
        )?,
        max_attempts_per_waypoint: load_nonnegative_u32_env(
            "WAYPOINT_NAV_MAX_RECOVERY_ATTEMPTS",
            DEFAULT_MAX_RECOVERY_ATTEMPTS,
        )?,
        settle_duration: load_duration_env(
            "WAYPOINT_NAV_RECOVERY_SETTLE_SECONDS",
            DEFAULT_RECOVERY_SETTLE_DURATION,
        )?,
        rotate_duration: load_duration_env(
            "WAYPOINT_NAV_RECOVERY_ROTATE_SECONDS",
            DEFAULT_RECOVERY_ROTATE_DURATION,
        )?,
        backoff_duration: load_duration_env(
            "WAYPOINT_NAV_RECOVERY_BACKOFF_SECONDS",
            DEFAULT_RECOVERY_BACKOFF_DURATION,
        )?,
        rotate_speed: load_positive_f64_env(
            "WAYPOINT_NAV_RECOVERY_ROTATE_SPEED",
            DEFAULT_RECOVERY_ROTATE_SPEED,
        )?,
        backoff_speed: load_positive_f64_env(
            "WAYPOINT_NAV_RECOVERY_BACKOFF_SPEED",
            DEFAULT_RECOVERY_BACKOFF_SPEED,
        )?,
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

fn new_active_goal(
    goal: Point2D,
    robot_pose: Point2D,
    goal_tolerance: f64,
    published_at: Instant,
) -> ActiveGoalState {
    ActiveGoalState {
        published_goal: goal,
        published_robot_pose: robot_pose,
        last_progress_pose: robot_pose,
        published_at,
        last_progress_at: published_at,
        completion_armed: distance(robot_pose, goal) > goal_tolerance,
    }
}

fn update_goal_progress(
    active_goal: &mut ActiveGoalState,
    robot_pose: Point2D,
    min_progress_distance: f64,
    now: Instant,
) {
    if distance(robot_pose, active_goal.last_progress_pose) >= min_progress_distance {
        active_goal.last_progress_pose = robot_pose;
        active_goal.last_progress_at = now;
    }
}

fn goal_is_stuck(active_goal: &ActiveGoalState, now: Instant, stuck_timeout: Duration) -> bool {
    now.saturating_duration_since(active_goal.last_progress_at) >= stuck_timeout
}

fn arm_goal_completion_if_progressed(
    active_goal: &mut ActiveGoalState,
    robot_pose: Point2D,
    goal_tolerance: f64,
    now: Instant,
) {
    if active_goal.completion_armed {
        return;
    }

    let moved_since_publish = distance(robot_pose, active_goal.published_robot_pose);
    let remaining_distance = distance(robot_pose, active_goal.published_goal);
    let elapsed_since_publish = now.saturating_duration_since(active_goal.published_at);
    if moved_since_publish >= GOAL_PROGRESS_ARM_DISTANCE
        || remaining_distance > goal_tolerance
        || elapsed_since_publish >= GOAL_COMPLETION_ARM_DELAY
    {
        active_goal.completion_armed = true;
    }
}

fn publish_cancel(
    publisher: &safe_drive::topic::publisher::Publisher<std_msgs::msg::Empty>,
) -> Result<(), DynError> {
    let msg = std_msgs::msg::Empty::new().ok_or("failed to allocate Empty")?;
    publisher.send(&msg)?;
    Ok(())
}

fn publish_velocity_command(
    publisher: &safe_drive::topic::publisher::Publisher<geometry_msgs::msg::Twist>,
    command: VelocityCommand,
) -> Result<(), DynError> {
    let mut msg = geometry_msgs::msg::Twist::new().ok_or("failed to allocate Twist")?;
    msg.linear.x = command.linear_x;
    msg.angular.z = command.angular_z;
    publisher.send(&msg)?;
    Ok(())
}

fn stop_command() -> VelocityCommand {
    VelocityCommand {
        linear_x: 0.0,
        angular_z: 0.0,
    }
}

fn recovery_command_for_phase(
    recovery: &RecoveryState,
    config: &RecoveryConfig,
) -> VelocityCommand {
    match recovery.phase {
        RecoveryPhase::Settle => stop_command(),
        RecoveryPhase::Rotate => {
            let direction = if recovery.attempt % 2 == 0 { -1.0 } else { 1.0 };
            VelocityCommand {
                linear_x: 0.0,
                angular_z: direction * config.rotate_speed,
            }
        }
        RecoveryPhase::Backoff => VelocityCommand {
            linear_x: -config.backoff_speed,
            angular_z: 0.0,
        },
    }
}

fn advance_recovery_phase(
    recovery: &mut RecoveryState,
    now: Instant,
    config: &RecoveryConfig,
) -> RecoveryPhaseTransition {
    let elapsed = now.saturating_duration_since(recovery.phase_started_at);
    match recovery.phase {
        RecoveryPhase::Settle if elapsed >= config.settle_duration => {
            recovery.phase = RecoveryPhase::Rotate;
            recovery.phase_started_at = now;
            RecoveryPhaseTransition::ToRotate
        }
        RecoveryPhase::Rotate if elapsed >= config.rotate_duration => {
            recovery.phase = RecoveryPhase::Backoff;
            recovery.phase_started_at = now;
            RecoveryPhaseTransition::ToBackoff
        }
        RecoveryPhase::Backoff if elapsed >= config.backoff_duration => {
            RecoveryPhaseTransition::Complete
        }
        _ => RecoveryPhaseTransition::None,
    }
}

fn begin_recovery_attempt(
    state: &mut NavigatorState,
    current_idx: usize,
    goal: Point2D,
    now: Instant,
    config: &RecoveryConfig,
) -> MissionEvent {
    let attempt = state.current_waypoint_recovery_attempts + 1;
    if attempt > config.max_attempts_per_waypoint {
        state.mission_status = MissionStatus::Failed;
        state.current_waypoint_idx = None;
        state.last_goal_publish_at = None;
        state.active_goal = None;
        state.recovery = None;
        MissionEvent::Failed {
            current_idx,
            goal,
            attempts: state.current_waypoint_recovery_attempts,
        }
    } else {
        state.mission_status = MissionStatus::Recovering;
        state.current_waypoint_recovery_attempts = attempt;
        state.last_goal_publish_at = None;
        state.active_goal = None;
        state.recovery = Some(RecoveryState {
            waypoint_idx: current_idx,
            goal,
            attempt,
            phase: RecoveryPhase::Settle,
            phase_started_at: now,
        });
        MissionEvent::RecoveryStarted {
            current_idx,
            goal,
            attempt,
        }
    }
}

fn resume_waypoint_after_recovery(
    state: &mut NavigatorState,
    current_idx: usize,
    goal: Point2D,
    attempt: u32,
    robot_pose: Point2D,
    goal_tolerance: f64,
    now: Instant,
) -> MissionEvent {
    state.mission_status = MissionStatus::Running;
    state.current_waypoint_idx = Some(current_idx);
    state.last_goal_publish_at = Some(now);
    state.active_goal = Some(new_active_goal(goal, robot_pose, goal_tolerance, now));
    state.recovery = None;
    MissionEvent::RecoveryRetry {
        current_idx,
        goal,
        attempt,
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
    let cancel_pub =
        node.create_publisher::<std_msgs::msg::Empty>(NAVIGATION_CANCEL_TOPIC, None)?;
    let cmd_pub = node.create_publisher::<geometry_msgs::msg::Twist>("/cmd_vel", None)?;

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
    pr_info!(
        logger,
        "recovery: timeout={:.1}s, max_attempts={}, rotate={:.1}s@{:.2}rad/s, backoff={:.1}s@{:.2}m/s",
        mission.recovery.stuck_timeout.as_secs_f64(),
        mission.recovery.max_attempts_per_waypoint,
        mission.recovery.rotate_duration.as_secs_f64(),
        mission.recovery.rotate_speed,
        mission.recovery.backoff_duration.as_secs_f64(),
        mission.recovery.backoff_speed
    );

    let mut selector = ctx.create_selector()?;
    let mission_cb = mission.clone();
    let state_odom = state.clone();
    let log_odom = Logger::new("waypoint_navigator_node");
    selector.add_subscriber(
        odom_sub,
        Box::new(move |msg| {
            let robot_pose = Point2D::new(msg.pose.pose.position.x, msg.pose.pose.position.y);
            let now = Instant::now();

            let (event, recovery_cmd) = {
                let mut st = match state_odom.lock() {
                    Ok(guard) => guard,
                    Err(_) => {
                        pr_warn!(log_odom, "failed to lock mission state on /odom");
                        return;
                    }
                };

                if mission_cb.frame == MissionFrame::RelativeStart && st.anchor_pose.is_none() {
                    st.anchor_pose = Some(robot_pose);
                }

                match st.mission_status {
                    MissionStatus::Completed | MissionStatus::Failed => (None, None),
                    MissionStatus::Idle => {
                        let goal = resolve_waypoint(
                            mission_cb.waypoints[0],
                            mission_cb.frame,
                            st.anchor_pose,
                        );
                        st.mission_status = MissionStatus::Running;
                        st.current_waypoint_idx = Some(0);
                        st.current_waypoint_recovery_attempts = 0;
                        st.last_goal_publish_at = Some(now);
                        st.active_goal = Some(new_active_goal(
                            goal,
                            robot_pose,
                            mission_cb.goal_tolerance,
                            now,
                        ));
                        (
                            Some(MissionEvent::Start { next_idx: 0, goal }),
                            Some(stop_command()),
                        )
                    }
                    MissionStatus::Recovering => {
                        match st.recovery.as_mut() {
                            Some(recovery) => {
                                let transition =
                                    advance_recovery_phase(recovery, now, &mission_cb.recovery);
                                match transition {
                                    RecoveryPhaseTransition::Complete => {
                                        let current_idx = recovery.waypoint_idx;
                                        let goal = recovery.goal;
                                        let attempt = recovery.attempt;
                                        (
                                            Some(resume_waypoint_after_recovery(
                                                &mut st,
                                                current_idx,
                                                goal,
                                                attempt,
                                                robot_pose,
                                                mission_cb.goal_tolerance,
                                                now,
                                            )),
                                            Some(stop_command()),
                                        )
                                    }
                                    RecoveryPhaseTransition::ToRotate => (
                                        Some(MissionEvent::RecoveryRotating {
                                            current_idx: recovery.waypoint_idx,
                                            goal: recovery.goal,
                                            attempt: recovery.attempt,
                                        }),
                                        Some(recovery_command_for_phase(
                                            recovery,
                                            &mission_cb.recovery,
                                        )),
                                    ),
                                    RecoveryPhaseTransition::ToBackoff => (
                                        Some(MissionEvent::RecoveryBackingOff {
                                            current_idx: recovery.waypoint_idx,
                                            goal: recovery.goal,
                                            attempt: recovery.attempt,
                                        }),
                                        Some(recovery_command_for_phase(
                                            recovery,
                                            &mission_cb.recovery,
                                        )),
                                    ),
                                    RecoveryPhaseTransition::None => (
                                        None,
                                        Some(recovery_command_for_phase(
                                            recovery,
                                            &mission_cb.recovery,
                                        )),
                                    ),
                                }
                            }
                            None => {
                                pr_warn!(
                                    log_odom,
                                    "recovery state missing while navigator status is recovering"
                                );
                                st.mission_status = MissionStatus::Failed;
                                st.current_waypoint_idx = None;
                                st.last_goal_publish_at = None;
                                st.active_goal = None;
                                (
                                    Some(MissionEvent::Failed {
                                        current_idx: 0,
                                        goal: robot_pose,
                                        attempts: st.current_waypoint_recovery_attempts,
                                    }),
                                    Some(stop_command()),
                                )
                            }
                        }
                    }
                    MissionStatus::Running => match st.current_waypoint_idx {
                        None => {
                            st.mission_status = MissionStatus::Completed;
                            (None, Some(stop_command()))
                        }
                        Some(current_idx) => {
                            let current_goal = resolve_waypoint(
                                mission_cb.waypoints[current_idx],
                                mission_cb.frame,
                                st.anchor_pose,
                            );
                            let current_distance = distance(robot_pose, current_goal);
                            if let Some(active_goal) = st.active_goal.as_mut() {
                                update_goal_progress(
                                    active_goal,
                                    robot_pose,
                                    mission_cb.recovery.progress_distance,
                                    now,
                                );
                                arm_goal_completion_if_progressed(
                                    active_goal,
                                    robot_pose,
                                    mission_cb.goal_tolerance,
                                    now,
                                );
                            }
                            let completion_armed = st
                                .active_goal
                                .map(|active_goal| active_goal.completion_armed)
                                .unwrap_or(current_distance > mission_cb.goal_tolerance);

                            let should_recover = current_distance > mission_cb.goal_tolerance
                                && st
                                    .active_goal
                                    .map(|active_goal| {
                                        goal_is_stuck(
                                            &active_goal,
                                            now,
                                            mission_cb.recovery.stuck_timeout,
                                        )
                                    })
                                    .unwrap_or(false);

                            if should_recover {
                                (
                                    Some(begin_recovery_attempt(
                                        &mut st,
                                        current_idx,
                                        current_goal,
                                        now,
                                        &mission_cb.recovery,
                                    )),
                                    Some(stop_command()),
                                )
                            } else if current_distance > mission_cb.goal_tolerance || !completion_armed {
                                if should_republish_goal(st.last_goal_publish_at, now) {
                                    st.last_goal_publish_at = Some(now);
                                    (
                                        Some(MissionEvent::Republish {
                                            current_idx,
                                            goal: current_goal,
                                        }),
                                        None,
                                    )
                                } else {
                                    (None, None)
                                }
                            } else if current_idx + 1 < mission_cb.waypoints.len() {
                                let next_idx = current_idx + 1;
                                let next_goal = resolve_waypoint(
                                    mission_cb.waypoints[next_idx],
                                    mission_cb.frame,
                                    st.anchor_pose,
                                );
                                st.current_waypoint_idx = Some(next_idx);
                                st.current_waypoint_recovery_attempts = 0;
                                st.last_goal_publish_at = Some(now);
                                st.active_goal = Some(new_active_goal(
                                    next_goal,
                                    robot_pose,
                                    mission_cb.goal_tolerance,
                                    now,
                                ));
                                (
                                    Some(MissionEvent::Advance {
                                        reached_idx: current_idx,
                                        reached_goal: current_goal,
                                        next_idx,
                                        next_goal,
                                    }),
                                    None,
                                )
                            } else if mission_cb.loop_mission {
                                let next_goal = resolve_waypoint(
                                    mission_cb.waypoints[0],
                                    mission_cb.frame,
                                    st.anchor_pose,
                                );
                                st.current_waypoint_idx = Some(0);
                                st.current_waypoint_recovery_attempts = 0;
                                st.last_goal_publish_at = Some(now);
                                st.active_goal = Some(new_active_goal(
                                    next_goal,
                                    robot_pose,
                                    mission_cb.goal_tolerance,
                                    now,
                                ));
                                (
                                    Some(MissionEvent::Loop {
                                        reached_idx: current_idx,
                                        reached_goal: current_goal,
                                        next_idx: 0,
                                        next_goal,
                                    }),
                                    None,
                                )
                            } else {
                                st.current_waypoint_idx = None;
                                st.mission_status = MissionStatus::Completed;
                                st.last_goal_publish_at = None;
                                st.active_goal = None;
                                st.recovery = None;
                                (
                                    Some(MissionEvent::Complete {
                                        reached_idx: current_idx,
                                        reached_goal: current_goal,
                                    }),
                                    Some(stop_command()),
                                )
                            }
                        }
                    },
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
                    if let Err(err) = publish_cancel(&cancel_pub) {
                        pr_warn!(log_odom, "failed to publish navigation cancel: {}", err);
                    }
                    pr_info!(
                        log_odom,
                        "mission complete at waypoint {}/{} ({:.2}, {:.2})",
                        reached_idx + 1,
                        mission_cb.waypoints.len(),
                        reached_goal.x,
                        reached_goal.y
                    );
                }
                Some(MissionEvent::RecoveryStarted {
                    current_idx,
                    goal,
                    attempt,
                }) => {
                    if let Err(err) = publish_cancel(&cancel_pub) {
                        pr_warn!(log_odom, "failed to publish navigation cancel: {}", err);
                    }
                    pr_warn!(
                        log_odom,
                        "stuck near waypoint {}/{} -> ({:.2}, {:.2}); starting recovery attempt {}",
                        current_idx + 1,
                        mission_cb.waypoints.len(),
                        goal.x,
                        goal.y,
                        attempt
                    );
                }
                Some(MissionEvent::RecoveryRotating {
                    current_idx,
                    goal,
                    attempt,
                }) => {
                    pr_info!(
                        log_odom,
                        "recovery attempt {} for waypoint {}/{} -> ({:.2}, {:.2}): rotating in place",
                        attempt,
                        current_idx + 1,
                        mission_cb.waypoints.len(),
                        goal.x,
                        goal.y
                    );
                }
                Some(MissionEvent::RecoveryBackingOff {
                    current_idx,
                    goal,
                    attempt,
                }) => {
                    pr_info!(
                        log_odom,
                        "recovery attempt {} for waypoint {}/{} -> ({:.2}, {:.2}): backing off",
                        attempt,
                        current_idx + 1,
                        mission_cb.waypoints.len(),
                        goal.x,
                        goal.y
                    );
                }
                Some(MissionEvent::RecoveryRetry {
                    current_idx,
                    goal,
                    attempt,
                }) => {
                    if let Err(err) = publish_goal(&goal_pub, goal) {
                        pr_warn!(log_odom, "failed to republish recovered goal: {}", err);
                    }
                    pr_info!(
                        log_odom,
                        "reissuing waypoint {}/{} -> ({:.2}, {:.2}) after recovery attempt {}",
                        current_idx + 1,
                        mission_cb.waypoints.len(),
                        goal.x,
                        goal.y,
                        attempt
                    );
                }
                Some(MissionEvent::Failed {
                    current_idx,
                    goal,
                    attempts,
                }) => {
                    if let Err(err) = publish_cancel(&cancel_pub) {
                        pr_warn!(log_odom, "failed to publish navigation cancel: {}", err);
                    }
                    pr_warn!(
                        log_odom,
                        "mission failed at waypoint {}/{} -> ({:.2}, {:.2}) after {} recovery attempt(s)",
                        current_idx + 1,
                        mission_cb.waypoints.len(),
                        goal.x,
                        goal.y,
                        attempts
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

            if let Some(command) = recovery_cmd {
                if let Err(err) = publish_velocity_command(&cmd_pub, command) {
                    pr_warn!(log_odom, "failed to publish recovery velocity command: {}", err);
                }
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
        advance_recovery_phase, arm_goal_completion_if_progressed, begin_recovery_attempt,
        goal_is_stuck, new_active_goal, parse_bool, parse_mission_frame, parse_waypoints,
        recovery_command_for_phase, resolve_waypoint, resume_waypoint_after_recovery,
        should_republish_goal, update_goal_progress, MissionEvent, MissionFrame, MissionStatus,
        NavigatorState, RecoveryConfig, RecoveryPhase, RecoveryPhaseTransition, RecoveryState,
        GOAL_COMPLETION_ARM_DELAY, GOAL_PROGRESS_ARM_DISTANCE, GOAL_REPUBLISH_INTERVAL,
    };
    use rust_robotics_core::Point2D;
    use std::time::{Duration, Instant};

    fn recovery_config() -> RecoveryConfig {
        RecoveryConfig {
            stuck_timeout: Duration::from_secs(2),
            progress_distance: 0.05,
            max_attempts_per_waypoint: 2,
            settle_duration: Duration::from_millis(500),
            rotate_duration: Duration::from_secs(1),
            backoff_duration: Duration::from_millis(800),
            rotate_speed: 0.7,
            backoff_speed: 0.08,
        }
    }

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
        let now = Instant::now();
        let active_goal = new_active_goal(Point2D::new(0.1, 0.1), Point2D::new(0.0, 0.0), 0.2, now);
        assert!(!active_goal.completion_armed);
    }

    #[test]
    fn arm_goal_completion_when_robot_moves_after_publish() {
        let now = Instant::now();
        let mut active_goal =
            new_active_goal(Point2D::new(0.1, 0.1), Point2D::new(0.0, 0.0), 0.2, now);

        arm_goal_completion_if_progressed(
            &mut active_goal,
            Point2D::new(GOAL_PROGRESS_ARM_DISTANCE + 0.01, 0.0),
            0.2,
            now,
        );

        assert!(active_goal.completion_armed);
    }

    #[test]
    fn arm_goal_completion_when_robot_is_seen_outside_tolerance() {
        let now = Instant::now();
        let mut active_goal =
            new_active_goal(Point2D::new(0.1, 0.1), Point2D::new(0.0, 0.0), 0.2, now);

        arm_goal_completion_if_progressed(&mut active_goal, Point2D::new(0.5, 0.5), 0.2, now);

        assert!(active_goal.completion_armed);
    }

    #[test]
    fn arm_goal_completion_after_short_settle_delay() {
        let now = Instant::now();
        let mut active_goal =
            new_active_goal(Point2D::new(0.1, 0.1), Point2D::new(0.0, 0.0), 0.2, now);

        arm_goal_completion_if_progressed(
            &mut active_goal,
            Point2D::new(0.0, 0.0),
            0.2,
            now + GOAL_COMPLETION_ARM_DELAY,
        );

        assert!(active_goal.completion_armed);
    }

    #[test]
    fn update_goal_progress_tracks_last_progress_pose_and_time() {
        let now = Instant::now();
        let mut active_goal =
            new_active_goal(Point2D::new(1.0, 0.0), Point2D::new(0.0, 0.0), 0.2, now);
        let next_pose = Point2D::new(0.08, 0.0);
        let next_time = now + Duration::from_secs(1);

        update_goal_progress(&mut active_goal, next_pose, 0.05, next_time);

        assert!((active_goal.last_progress_pose.x - next_pose.x).abs() < 1e-9);
        assert_eq!(active_goal.last_progress_at, next_time);
    }

    #[test]
    fn goal_is_stuck_after_progress_timeout() {
        let now = Instant::now();
        let active_goal =
            new_active_goal(Point2D::new(1.0, 0.0), Point2D::new(0.0, 0.0), 0.2, now);

        assert!(!goal_is_stuck(&active_goal, now + Duration::from_secs(1), Duration::from_secs(2)));
        assert!(goal_is_stuck(&active_goal, now + Duration::from_secs(2), Duration::from_secs(2)));
    }

    #[test]
    fn advance_recovery_phase_steps_through_sequence() {
        let config = recovery_config();
        let now = Instant::now();
        let mut recovery = RecoveryState {
            waypoint_idx: 0,
            goal: Point2D::new(0.4, 0.0),
            attempt: 1,
            phase: RecoveryPhase::Settle,
            phase_started_at: now,
        };

        assert_eq!(
            advance_recovery_phase(&mut recovery, now + config.settle_duration, &config),
            RecoveryPhaseTransition::ToRotate
        );
        assert_eq!(recovery.phase, RecoveryPhase::Rotate);

        assert_eq!(
            advance_recovery_phase(&mut recovery, now + config.settle_duration + config.rotate_duration, &config),
            RecoveryPhaseTransition::ToBackoff
        );
        assert_eq!(recovery.phase, RecoveryPhase::Backoff);

        assert_eq!(
            advance_recovery_phase(
                &mut recovery,
                now + config.settle_duration + config.rotate_duration + config.backoff_duration,
                &config
            ),
            RecoveryPhaseTransition::Complete
        );
    }

    #[test]
    fn recovery_rotate_direction_alternates_by_attempt() {
        let config = recovery_config();
        let now = Instant::now();
        let rotate_attempt_one = RecoveryState {
            waypoint_idx: 0,
            goal: Point2D::new(0.4, 0.0),
            attempt: 1,
            phase: RecoveryPhase::Rotate,
            phase_started_at: now,
        };
        let rotate_attempt_two = RecoveryState {
            attempt: 2,
            ..rotate_attempt_one
        };

        let cmd_one = recovery_command_for_phase(&rotate_attempt_one, &config);
        let cmd_two = recovery_command_for_phase(&rotate_attempt_two, &config);

        assert!(cmd_one.angular_z > 0.0);
        assert!(cmd_two.angular_z < 0.0);
        assert_eq!(cmd_one.linear_x, 0.0);
        assert_eq!(cmd_two.linear_x, 0.0);
    }

    #[test]
    fn begin_recovery_attempt_enters_recovering_state() {
        let config = recovery_config();
        let now = Instant::now();
        let goal = Point2D::new(1.0, 0.0);
        let mut state = NavigatorState {
            mission_status: MissionStatus::Running,
            current_waypoint_idx: Some(0),
            active_goal: Some(new_active_goal(goal, Point2D::origin(), 0.2, now)),
            ..Default::default()
        };

        let event = begin_recovery_attempt(&mut state, 0, goal, now, &config);

        match event {
            MissionEvent::RecoveryStarted {
                current_idx,
                goal: event_goal,
                attempt,
            } => {
                assert_eq!(current_idx, 0);
                assert_eq!(attempt, 1);
                assert!((event_goal.x - goal.x).abs() < 1e-9);
            }
            _ => panic!("expected RecoveryStarted"),
        }
        assert_eq!(state.mission_status, MissionStatus::Recovering);
        assert_eq!(state.current_waypoint_recovery_attempts, 1);
        assert!(state.active_goal.is_none());
        assert!(state.recovery.is_some());
    }

    #[test]
    fn begin_recovery_attempt_fails_after_attempt_budget_is_spent() {
        let config = recovery_config();
        let now = Instant::now();
        let goal = Point2D::new(1.0, 0.0);
        let mut state = NavigatorState {
            mission_status: MissionStatus::Running,
            current_waypoint_idx: Some(0),
            current_waypoint_recovery_attempts: config.max_attempts_per_waypoint,
            active_goal: Some(new_active_goal(goal, Point2D::origin(), 0.2, now)),
            ..Default::default()
        };

        let event = begin_recovery_attempt(&mut state, 0, goal, now, &config);

        match event {
            MissionEvent::Failed {
                current_idx,
                goal: event_goal,
                attempts,
            } => {
                assert_eq!(current_idx, 0);
                assert_eq!(attempts, config.max_attempts_per_waypoint);
                assert!((event_goal.x - goal.x).abs() < 1e-9);
            }
            _ => panic!("expected Failed"),
        }
        assert_eq!(state.mission_status, MissionStatus::Failed);
        assert!(state.current_waypoint_idx.is_none());
        assert!(state.active_goal.is_none());
        assert!(state.recovery.is_none());
    }

    #[test]
    fn resume_waypoint_after_recovery_restores_running_goal() {
        let now = Instant::now();
        let goal = Point2D::new(0.8, 0.2);
        let robot_pose = Point2D::new(0.1, -0.1);
        let mut state = NavigatorState {
            mission_status: MissionStatus::Recovering,
            current_waypoint_idx: Some(0),
            current_waypoint_recovery_attempts: 1,
            recovery: Some(RecoveryState {
                waypoint_idx: 0,
                goal,
                attempt: 1,
                phase: RecoveryPhase::Backoff,
                phase_started_at: now,
            }),
            ..Default::default()
        };

        let event =
            resume_waypoint_after_recovery(&mut state, 0, goal, 1, robot_pose, 0.2, now);

        match event {
            MissionEvent::RecoveryRetry {
                current_idx,
                goal: event_goal,
                attempt,
            } => {
                assert_eq!(current_idx, 0);
                assert_eq!(attempt, 1);
                assert!((event_goal.y - goal.y).abs() < 1e-9);
            }
            _ => panic!("expected RecoveryRetry"),
        }
        assert_eq!(state.mission_status, MissionStatus::Running);
        assert_eq!(state.current_waypoint_idx, Some(0));
        assert!(state.recovery.is_none());
        assert_eq!(state.last_goal_publish_at, Some(now));
        let active_goal = state.active_goal.expect("active goal should be recreated");
        assert!((active_goal.published_goal.x - goal.x).abs() < 1e-9);
        assert!((active_goal.published_robot_pose.x - robot_pose.x).abs() < 1e-9);
    }
}
