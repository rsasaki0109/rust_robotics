//! Navigation on a LiDAR-built occupancy grid for the Drive mode: A* plans on
//! the grid, Pure Pursuit follows the plan from the estimated pose, plus the
//! on-screen joystick and the grid renderer.

use egui::{Color32, Pos2, Rect, Stroke, Vec2};
use nalgebra::Vector2;
use rust_robotics_control::pure_pursuit::VehicleState;
use rust_robotics_control::{PurePursuitConfig, PurePursuitController};
use rust_robotics_core::{Obstacles, Path2D, Point2D, Pose2D};
use rust_robotics_planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics_slam::lidar_occupancy::{CellState, OccupancyGrid};

/// Planner grid cell \[m\].
const PLAN_RESOLUTION: f64 = 0.2;
/// Obstacle inflation \[m\]: robot radius plus a margin for estimate error.
const PLAN_CLEARANCE: f64 = 0.5;
/// Replan this often while following, as the map keeps growing \[s\].
const REPLAN_INTERVAL: f64 = 1.0;
const GOAL_TOLERANCE: f64 = 0.3;
const LOOKAHEAD: f64 = 0.6;
/// Turn in place while the lookahead point is more than this off the nose.
const TURN_IN_PLACE: f64 = 1.0;
/// Pure Pursuit's virtual wheelbase \[m\]; with ω = v tan δ / L the
/// curvature is the classic 2 sin α / lookahead regardless of L.
const WHEELBASE: f64 = 0.4;

/// Plans a collision-free path on `grid` with A*, treating unknown cells as
/// free so the robot explores toward goals it has not mapped yet.
pub(crate) fn plan_on_grid(
    grid: &OccupancyGrid,
    start: Vector2<f64>,
    goal: Vector2<f64>,
) -> Result<Path2D, String> {
    let occupied = grid.occupied_points();
    if occupied.is_empty() {
        return Err("the map is empty".into());
    }
    // The planner grid spans its obstacles' bounding box; the grid's own
    // corners stretch it over the whole map so goals in unexplored space
    // can still be planned to.
    let (width, height) = grid.size();
    let min = grid.origin();
    let max = min + Vector2::new(width as f64, height as f64) * grid.resolution();
    let obstacles = Obstacles::from_points(
        occupied
            .iter()
            .chain(&[min, max])
            .map(|point| Point2D::new(point.x, point.y))
            .collect(),
    );
    let planner = AStarPlanner::from_obstacle_points(
        &obstacles,
        AStarConfig {
            resolution: PLAN_RESOLUTION,
            robot_radius: PLAN_CLEARANCE,
            heuristic_weight: 1.0,
        },
    )
    .map_err(|error| error.to_string())?;
    // A robot hugging a wall stands inside the inflated obstacles, and a
    // tapped goal often lands on or next to a wall: use the nearest free
    // planner cells instead.
    let map = planner.grid_map();
    let nearest_free = |point: Vector2<f64>, max_rings: i32| {
        let (x, y) = (map.calc_x_index(point.x), map.calc_y_index(point.y));
        (0..=max_rings)
            .flat_map(|ring| {
                (-ring..=ring).flat_map(move |dx| (-ring..=ring).map(move |dy| (dx, dy)))
            })
            .find(|&(dx, dy)| map.is_valid(x + dx, y + dy))
            .map(|(dx, dy)| Point2D::new(map.calc_x_position(x + dx), map.calc_y_position(y + dy)))
    };
    let free_start = nearest_free(start, 3).ok_or("the robot is boxed in")?;
    let free_goal = nearest_free(goal, 5).ok_or("the goal is inside an obstacle")?;
    let mut path = planner
        .plan(free_start, free_goal)
        .map_err(|_| "no path to the goal".to_string())?;
    path.points.insert(0, Point2D::new(start.x, start.y));
    Ok(path)
}

/// What the navigator is doing.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum NavStatus {
    Idle,
    Following,
    Reached,
    Failed(String),
    /// The pose estimate is too uncertain to act on.
    WaitingForLocalization,
}

/// Goal-directed driving: plan on the grid, follow with Pure Pursuit.
pub(crate) struct Navigator {
    goal: Option<Vector2<f64>>,
    path: Option<Path2D>,
    tracker: PurePursuitController,
    since_plan: f64,
    pub(crate) status: NavStatus,
}

impl Default for Navigator {
    fn default() -> Self {
        Self {
            goal: None,
            path: None,
            tracker: PurePursuitController::new(PurePursuitConfig {
                look_ahead_gain: 0.0,
                look_ahead_distance: LOOKAHEAD,
                wheelbase: WHEELBASE,
                kp: 1.0,
                goal_threshold: GOAL_TOLERANCE,
            }),
            since_plan: 0.0,
            status: NavStatus::Idle,
        }
    }
}

impl Navigator {
    pub(crate) fn goal(&self) -> Option<Vector2<f64>> {
        self.goal
    }

    pub(crate) fn path(&self) -> Option<&Path2D> {
        self.path.as_ref()
    }

    pub(crate) fn is_active(&self) -> bool {
        self.goal.is_some()
    }

    pub(crate) fn set_goal(&mut self, goal: Vector2<f64>) {
        self.goal = Some(goal);
        self.path = None;
        // Plan on the next control call.
        self.since_plan = REPLAN_INTERVAL;
        self.status = NavStatus::Following;
    }

    pub(crate) fn cancel(&mut self) {
        *self = Self::default();
    }

    /// Unicycle command `(v, ω)` toward the goal from the estimated `pose`,
    /// or `None` when there is nothing to do. `confident` is false while the
    /// estimate is too uncertain to plan from.
    pub(crate) fn control(
        &mut self,
        grid: &OccupancyGrid,
        pose: Pose2D,
        confident: bool,
        speed: f64,
        turn_rate: f64,
        dt: f64,
    ) -> Option<(f64, f64)> {
        let goal = self.goal?;
        let position = Vector2::new(pose.x, pose.y);
        if (goal - position).norm() < GOAL_TOLERANCE {
            self.goal = None;
            self.path = None;
            self.status = NavStatus::Reached;
            return None;
        }
        if !confident {
            self.status = NavStatus::WaitingForLocalization;
            self.path = None;
            return None;
        }
        self.since_plan += dt;
        if self.path.is_none() || self.since_plan >= REPLAN_INTERVAL {
            self.since_plan = 0.0;
            match plan_on_grid(grid, position, goal) {
                Ok(path) => {
                    // The planner may have snapped the goal off a wall.
                    if let Some(end) = path.points.last() {
                        self.goal = Some(Vector2::new(end.x, end.y));
                    }
                    self.tracker.set_path(path.clone());
                    self.path = Some(path);
                    self.status = NavStatus::Following;
                }
                Err(reason) => {
                    self.goal = None;
                    self.path = None;
                    self.status = NavStatus::Failed(reason);
                    return None;
                }
            }
        }

        // Rotate toward the path first when it leaves behind the robot.
        let path = self.path.as_ref()?;
        let here = Point2D::new(pose.x, pose.y);
        let nearest = path.nearest_point_index(here).unwrap_or(0);
        let lookahead = path.points[nearest..]
            .iter()
            .find(|point| point.distance(&here) >= LOOKAHEAD)
            .or(path.points.last())?;
        let bearing = (lookahead.y - pose.y).atan2(lookahead.x - pose.x) - pose.yaw;
        let bearing = bearing.sin().atan2(bearing.cos());
        if bearing.abs() > TURN_IN_PLACE {
            return Some((0.0, turn_rate.copysign(bearing)));
        }
        let remaining = (goal - position).norm();
        let cruise = speed * (0.4 + 0.6 * (remaining / 1.5).min(1.0));
        // VehicleState's reference point is the rear axle, half a wheelbase
        // behind the given center.
        let state = VehicleState::new(pose.x, pose.y, pose.yaw, cruise, WHEELBASE);
        let curvature = self.tracker.compute_steering(&state).tan() / WHEELBASE;
        // Slow down in tight turns instead of saturating the turn rate,
        // which would cut the corner.
        let v = cruise.min(turn_rate / curvature.abs().max(1.0e-9));
        let omega = (v * curvature).clamp(-turn_rate, turn_rate);
        Some((v, omega))
    }
}

/// An on-screen joystick for touch screens, anchored to the lower right of
/// `area`. Returns `(forward, turn)` in `[-1, 1]` while it is held.
pub(crate) fn joystick(ui: &mut egui::Ui, area: Rect) -> Option<(f64, f64)> {
    let radius = (area.width().min(area.height()) * 0.12).clamp(36.0, 64.0);
    let center = area.right_bottom() - Vec2::splat(radius + 12.0);
    let rect = Rect::from_center_size(center, Vec2::splat(radius * 2.0));
    let response = ui.interact(rect, ui.id().with("drive_joystick"), egui::Sense::drag());
    let painter = ui.painter_at(area);
    painter.circle(
        center,
        radius,
        Color32::from_rgba_unmultiplied(255, 255, 255, 18),
        Stroke::new(1.5_f32, Color32::from_rgba_unmultiplied(255, 255, 255, 80)),
    );
    let held = response
        .is_pointer_button_down_on()
        .then(|| response.interact_pointer_pos())
        .flatten();
    let offset = held.map_or(Vec2::ZERO, |pos| {
        let offset = pos - center;
        offset * (radius / offset.length().max(radius))
    });
    painter.circle_filled(
        center + offset,
        radius * 0.38,
        Color32::from_rgba_unmultiplied(255, 255, 255, if held.is_some() { 150 } else { 70 }),
    );
    held.map(|_| {
        let (x, y) = (offset.x / radius, offset.y / radius);
        // Small dead zone so a tap does not creep.
        let dead = |value: f32| {
            if value.abs() < 0.15 {
                0.0
            } else {
                f64::from(value)
            }
        };
        (dead(-y), dead(-x))
    })
}

/// Paints `grid` as an image (free: dark blue, occupied: light gray,
/// unknown: transparent) into the world rectangle `world` of the scene.
pub(crate) fn grid_image(grid: &OccupancyGrid) -> egui::ColorImage {
    let (width, height) = grid.size();
    let mut pixels = Vec::with_capacity(width * height);
    // Image rows run top-down, grid rows bottom-up.
    for y in (0..height).rev() {
        for x in 0..width {
            pixels.push(match grid.state(x, y) {
                CellState::Unknown => Color32::TRANSPARENT,
                CellState::Free => Color32::from_rgba_unmultiplied(40, 62, 92, 120),
                CellState::Occupied => Color32::from_rgba_unmultiplied(200, 205, 215, 200),
            });
        }
    }
    egui::ColorImage {
        size: [width, height],
        pixels,
    }
}

/// Draws a planned path and its goal.
pub(crate) fn draw_path(
    painter: &egui::Painter,
    to_screen: impl Fn(f64, f64) -> Pos2,
    path: Option<&Path2D>,
    goal: Option<Vector2<f64>>,
) {
    const PATH: Color32 = Color32::from_rgb(80, 220, 240);
    if let Some(path) = path {
        let points: Vec<Pos2> = path.points.iter().map(|p| to_screen(p.x, p.y)).collect();
        painter.add(egui::Shape::line(points, Stroke::new(2.0_f32, PATH)));
    }
    if let Some(goal) = goal {
        let center = to_screen(goal.x, goal.y);
        painter.circle_stroke(center, 7.0, Stroke::new(2.0_f32, PATH));
        painter.circle_filled(center, 2.5, PATH);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rust_robotics_slam::lidar_occupancy::OccupancyConfig;
    use rust_robotics_slam::scan_to_map::{ranges_to_points, ray_cast_ranges, LineSegment};

    /// A 10 × 6 m room split by a wall with a gap at the top.
    fn walls() -> Vec<LineSegment> {
        let v = Vector2::new;
        vec![
            LineSegment::new(v(-5.0, -3.0), v(5.0, -3.0)),
            LineSegment::new(v(5.0, -3.0), v(5.0, 3.0)),
            LineSegment::new(v(5.0, 3.0), v(-5.0, 3.0)),
            LineSegment::new(v(-5.0, 3.0), v(-5.0, -3.0)),
            LineSegment::new(v(0.0, -3.0), v(0.0, 1.5)),
        ]
    }

    fn grid() -> OccupancyGrid {
        let poses = [
            Pose2D::new(-2.5, 0.0, 0.0),
            Pose2D::new(0.0, 2.2, 0.0),
            Pose2D::new(2.5, 0.0, 0.0),
        ];
        let scans: Vec<_> = poses
            .iter()
            .map(|pose| ranges_to_points(&ray_cast_ranges(*pose, &walls(), 360, 12.0)))
            .collect();
        OccupancyGrid::from_scans(
            poses.iter().copied().zip(scans.iter().map(Vec::as_slice)),
            0.5,
            OccupancyConfig::default(),
        )
    }

    #[test]
    fn plans_through_the_gap() {
        let path =
            plan_on_grid(&grid(), Vector2::new(-2.5, -2.0), Vector2::new(2.5, -2.0)).expect("path");
        let top = path.points.iter().map(|p| p.y).fold(f64::MIN, f64::max);
        assert!(top > 1.5, "path does not use the gap: max y {top}");
    }

    #[test]
    fn navigator_drives_to_the_goal_without_touching_walls() {
        let grid = grid();
        let mut navigator = Navigator::default();
        navigator.set_goal(Vector2::new(2.5, -2.0));
        let mut pose = Pose2D::new(-2.5, -2.0, std::f64::consts::PI);
        let dt = 1.0 / 30.0;
        for _ in 0..1_500 {
            let Some((v, omega)) = navigator.control(&grid, pose, true, 1.6, 1.4, dt) else {
                break;
            };
            pose = Pose2D::new(
                pose.x + v * pose.yaw.cos() * dt,
                pose.y + v * pose.yaw.sin() * dt,
                pose.yaw + omega * dt,
            );
            let clearance = walls()
                .iter()
                .map(|wall| {
                    let edge = wall.end - wall.start;
                    let point = Vector2::new(pose.x, pose.y);
                    let t = ((point - wall.start).dot(&edge) / edge.norm_squared()).clamp(0.0, 1.0);
                    (wall.start + edge * t - point).norm()
                })
                .fold(f64::INFINITY, f64::min);
            assert!(clearance > 0.3, "touched a wall at {pose:?}");
        }
        assert_eq!(navigator.status, NavStatus::Reached);
    }

    #[test]
    fn plans_into_unexplored_space() {
        // A corridor open to the east; everything past x = 4 is unknown.
        let v = Vector2::new;
        let corridor = [
            LineSegment::new(v(-4.0, -1.5), v(4.0, -1.5)),
            LineSegment::new(v(-4.0, 1.5), v(4.0, 1.5)),
            LineSegment::new(v(-4.0, -1.5), v(-4.0, 1.5)),
        ];
        let pose = Pose2D::new(-2.0, 0.0, 0.0);
        let mut grid = OccupancyGrid::new(v(-6.0, -6.0), v(12.0, 6.0), OccupancyConfig::default());
        grid.insert_ranges(pose, &ray_cast_ranges(pose, &corridor, 360, 6.0), 6.0);
        let goal = v(10.0, 3.0);
        assert_eq!(grid.state_at(goal), CellState::Unknown);
        let path = plan_on_grid(&grid, v(pose.x, pose.y), goal).expect("path into the unknown");
        let end = path.points.last().expect("points");
        assert!((v(end.x, end.y) - goal).norm() < 0.5, "ends at {end:?}");
    }

    #[test]
    fn goals_on_a_wall_snap_to_free_space() {
        // The divider wall at x = 0: a tap right on it still plans.
        let path = plan_on_grid(&grid(), Vector2::new(-2.5, -2.0), Vector2::new(0.0, -1.0))
            .expect("snapped goal");
        let end = path.points.last().expect("points");
        assert!(end.x.abs() > 0.4 && end.x.abs() < 1.3, "ends at {end:?}");
    }

    #[test]
    fn unreachable_goals_fail_cleanly() {
        let mut navigator = Navigator::default();
        navigator.set_goal(Vector2::new(50.0, 0.0));
        let command = navigator.control(&grid(), Pose2D::new(-2.5, 0.0, 0.0), true, 1.6, 1.4, 0.1);
        assert!(command.is_none());
        assert!(matches!(navigator.status, NavStatus::Failed(_)));
        assert!(!navigator.is_active());
    }
}
