//! Interactive LiDAR graph SLAM: drive a robot through the corridor loop with
//! the arrow keys, the on-screen joystick or auto-drive while scan-to-map
//! odometry and loop closure run live. The node scans feed an occupancy grid
//! the robot can navigate on (click a goal: A* + Pure Pursuit), and the
//! finished map can be frozen to localize a kidnapped robot with MCL. Also
//! hosts the shared renderer for LiDAR SLAM scenes.

use egui::{Color32, Pos2, Rect, Stroke, Vec2};
use nalgebra::Vector2;
use rand::{rngs::StdRng, SeedableRng};
use rand_distr::{Distribution, Normal};
use rust_robotics_core::Pose2D;
use rust_robotics_optimization::LinearSolver;
use rust_robotics_slam::{
    frontier_exploration::{is_frontier_near, next_frontier_goal, FrontierConfig},
    lidar_graph_slam::{LidarGraphSlam, LidarGraphSlamConfig},
    lidar_loop_scenario::{
        corridor_loop_deltas, corridor_loop_start, corridor_loop_start_for, corridor_loop_walls,
        corridor_loop_walls_for, CorridorLayout, CorridorLoopConfig,
    },
    lidar_mcl::{LidarMcl, LidarMclConfig},
    lidar_occupancy::{CellState, OccupancyConfig, OccupancyGrid},
    pose_graph_optimization::PoseGraphConfig,
    scan_to_map::{
        compose_pose, ranges_to_points, ray_cast_ranges, relative_pose, transform_scan_to_world,
        LineSegment,
    },
};

use crate::slam_nav::{draw_path, grid_image, joystick, NavStatus, Navigator};

pub(crate) const WORLD_X: (f64, f64) = (-16.0, 16.0);
pub(crate) const WORLD_Y: (f64, f64) = (-11.0, 11.0);
/// Upper bound on map points drawn per frame (the stride adapts to it).
const MAX_MAP_POINTS: usize = 9_000;

const DT: f64 = 1.0 / 30.0;
const DRIVE_SPEED: f64 = 1.6;
const TURN_RATE: f64 = 1.4;
const ROBOT_RADIUS: f64 = 0.3;
const BEAMS: usize = 180;
const MAX_RANGE: f64 = 8.0;
const ODOMETRY_NOISE: f64 = 0.0015;
/// Accumulate odometry until the robot moved this far before a SLAM update.
const UPDATE_DISTANCE: f64 = 0.1;
const UPDATE_YAW: f64 = 0.05;
const AUTO_LOOKAHEAD: usize = 10;
/// Drawn walls snap to this grid \[m\].
const WALL_SNAP: f64 = 0.1;
const MIN_WALL_LENGTH: f64 = 0.3;
const MAX_CUSTOM_WALLS: usize = 64;
/// Pose-graph iterations per frame while a re-optimization is pending, so a
/// loop closure never freezes the frame.
const OPTIMIZE_ITERATIONS_PER_FRAME: usize = 2;
const MCL_PARTICLES: usize = 2_000;
/// Navigate on the MCL estimate only once the particles agree this well \[m\].
const MCL_CONFIDENT_SPREAD: f64 = 0.4;
/// Re-check that the exploration goal is still a frontier this often \[s\].
const EXPLORE_RECHECK: f64 = 0.5;
const FRONTIER: Color32 = Color32::from_rgb(255, 160, 60);
/// DWA's trajectory while it overrides Pure Pursuit.
const LOCAL_PLAN: Color32 = Color32::from_rgb(250, 240, 120);
/// People are squares of this half-size \[m\] walking at about 0.7 m/s.
const PERSON_HALF_SIZE: f64 = 0.22;
const PERSON_SPEED: f64 = 0.7;
const MAX_PEOPLE: usize = 8;
const PERSON: Color32 = Color32::from_rgb(200, 120, 255);
/// A kidnapped robot lands at least this far from any wall \[m\].
const KIDNAP_CLEARANCE: f64 = 0.8;

const WALL: Color32 = Color32::from_rgb(90, 95, 105);
const TRUTH: Color32 = Color32::from_rgba_premultiplied(60, 60, 60, 110);
const ODOMETRY: Color32 = Color32::from_rgb(170, 120, 230);
const FRONT_END: Color32 = Color32::from_rgb(240, 150, 70);
const GRAPH: Color32 = Color32::from_rgb(90, 210, 140);
const LOOP_EDGE: Color32 = Color32::from_rgb(230, 90, 220);
const SCAN: Color32 = Color32::from_rgb(255, 90, 100);
const WRONG_LOOP_EDGE: Color32 = Color32::from_rgb(250, 220, 90);
/// A loop edge more than this far from ground truth is a false closure \[m\].
const WRONG_LOOP_TOLERANCE: f64 = 0.5;
/// Pillar spacing of the aliased corridor \[m\].
const ALIASED_SPACING: f64 = 2.0;
const GRAPH_MAP: Color32 = Color32::from_rgba_premultiplied(48, 77, 122, 120);
const FRONT_END_MAP: Color32 = Color32::from_rgba_premultiplied(113, 71, 33, 120);
const PARTICLE: Color32 = Color32::from_rgb(250, 200, 80);
const TRUTH_ROBOT: Color32 = Color32::from_rgb(200, 200, 200);

/// Everything needed to draw one LiDAR SLAM scene.
pub(crate) struct LidarSceneView<'a> {
    pub walls: &'a [LineSegment],
    /// Node scans and the poses to render them at.
    pub map: Vec<(Pose2D, &'a [Vector2<f64>])>,
    pub map_at_front_end: bool,
    pub truth: &'a [Pose2D],
    pub odometry: &'a [Pose2D],
    pub front_end: &'a [Pose2D],
    pub nodes: &'a [Pose2D],
    pub loop_edges: Vec<(Pose2D, Pose2D)>,
    /// Loop edges known (from ground truth) to be false closures.
    pub wrong_loop_edges: Vec<(Pose2D, Pose2D)>,
    pub scan: &'a [Vector2<f64>],
    pub estimate: Pose2D,
    /// Scan-to-map pose, drawn when the front end is running.
    pub front_end_pose: Option<Pose2D>,
    /// Occupancy grid texture and the world rectangle `(min, max)` it covers.
    pub grid: Option<(egui::TextureId, Vector2<f64>, Vector2<f64>)>,
}

fn world_rect(ui: &egui::Ui, reserved_height: f32) -> Rect {
    let aspect = ((WORLD_Y.1 - WORLD_Y.0) / (WORLD_X.1 - WORLD_X.0)) as f32;
    let width = ui
        .available_width()
        .min((ui.available_height() - reserved_height).max(120.0) / aspect);
    Rect::from_min_size(ui.cursor().min, Vec2::new(width, width * aspect))
}

fn to_screen(rect: Rect, x: f64, y: f64) -> Pos2 {
    let u = ((x - WORLD_X.0) / (WORLD_X.1 - WORLD_X.0)) as f32;
    let v = 1.0 - ((y - WORLD_Y.0) / (WORLD_Y.1 - WORLD_Y.0)) as f32;
    rect.min + Vec2::new(u * rect.width(), v * rect.height())
}

fn to_world(rect: Rect, pos: Pos2) -> Vector2<f64> {
    let u = f64::from((pos.x - rect.min.x) / rect.width());
    let v = f64::from((pos.y - rect.min.y) / rect.height());
    Vector2::new(
        WORLD_X.0 + u * (WORLD_X.1 - WORLD_X.0),
        WORLD_Y.1 - v * (WORLD_Y.1 - WORLD_Y.0),
    )
}

fn snap(point: Vector2<f64>) -> Vector2<f64> {
    let grid = |value: f64| (value / WALL_SNAP).round() * WALL_SNAP;
    Vector2::new(
        grid(point.x).clamp(WORLD_X.0, WORLD_X.1),
        grid(point.y).clamp(WORLD_Y.0, WORLD_Y.1),
    )
}

fn draw_trail(painter: &egui::Painter, rect: Rect, poses: &[Pose2D], stroke: Stroke) {
    if poses.len() < 2 {
        return;
    }
    let points = poses
        .iter()
        .map(|pose| to_screen(rect, pose.x, pose.y))
        .collect();
    painter.add(egui::Shape::line(points, stroke));
}

fn draw_robot(painter: &egui::Painter, rect: Rect, pose: Pose2D, color: Color32) {
    let center = to_screen(rect, pose.x, pose.y);
    painter.circle_filled(center, 5.0, color);
    let tip = center + Vec2::new(pose.yaw.cos() as f32 * 9.0, -pose.yaw.sin() as f32 * 9.0);
    painter.line_segment([center, tip], Stroke::new(2.0_f32, color));
}

/// Draws a LiDAR SLAM scene, leaving `reserved_height` px below it free.
pub(crate) fn draw_lidar_scene(
    ui: &mut egui::Ui,
    view: &LidarSceneView<'_>,
    reserved_height: f32,
) -> egui::Response {
    let rect = world_rect(ui, reserved_height);
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, Color32::from_rgb(18, 22, 28));
    if let Some((texture, min, max)) = view.grid {
        painter.image(
            texture,
            Rect::from_two_pos(to_screen(rect, min.x, max.y), to_screen(rect, max.x, min.y)),
            Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
            Color32::WHITE,
        );
    }

    for wall in view.walls {
        painter.line_segment(
            [
                to_screen(rect, wall.start.x, wall.start.y),
                to_screen(rect, wall.end.x, wall.end.y),
            ],
            Stroke::new(1.0_f32, WALL),
        );
    }

    let total_points: usize = view.map.iter().map(|(_, scan)| scan.len()).sum();
    let stride = total_points.div_ceil(MAX_MAP_POINTS).max(2);
    let map_color = if view.map_at_front_end {
        FRONT_END_MAP
    } else {
        GRAPH_MAP
    };
    for (pose, scan) in &view.map {
        for point in transform_scan_to_world(scan, *pose).iter().step_by(stride) {
            painter.circle_filled(to_screen(rect, point.x, point.y), 1.0, map_color);
        }
    }

    draw_trail(&painter, rect, view.truth, Stroke::new(1.0_f32, TRUTH));
    draw_trail(
        &painter,
        rect,
        view.odometry,
        Stroke::new(1.0_f32, ODOMETRY),
    );
    draw_trail(
        &painter,
        rect,
        view.front_end,
        Stroke::new(1.5_f32, FRONT_END),
    );
    draw_trail(&painter, rect, view.nodes, Stroke::new(2.0_f32, GRAPH));
    for (a, b) in &view.loop_edges {
        painter.line_segment(
            [to_screen(rect, a.x, a.y), to_screen(rect, b.x, b.y)],
            Stroke::new(1.5_f32, LOOP_EDGE),
        );
    }
    for (a, b) in &view.wrong_loop_edges {
        painter.line_segment(
            [to_screen(rect, a.x, a.y), to_screen(rect, b.x, b.y)],
            Stroke::new(3.0_f32, WRONG_LOOP_EDGE),
        );
    }
    for point in transform_scan_to_world(view.scan, view.estimate) {
        painter.circle_filled(to_screen(rect, point.x, point.y), 1.4, SCAN);
    }
    if let Some(pose) = view.front_end_pose {
        draw_robot(&painter, rect, pose, FRONT_END);
    }
    draw_robot(&painter, rect, view.estimate, GRAPH);
    ui.allocate_rect(rect, egui::Sense::click_and_drag())
}

/// Distance from `point` to the closest wall segment.
fn wall_clearance(walls: &[LineSegment], point: Vector2<f64>) -> f64 {
    walls
        .iter()
        .map(|wall| {
            let edge = wall.end - wall.start;
            let t = ((point - wall.start).dot(&edge) / edge.norm_squared()).clamp(0.0, 1.0);
            (wall.start + edge * t - point).norm()
        })
        .fold(f64::INFINITY, f64::min)
}

fn wrap_angle(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

fn rectangle(min: (f64, f64), max: (f64, f64)) -> Vec<LineSegment> {
    let corners = [
        Vector2::new(min.0, min.1),
        Vector2::new(max.0, min.1),
        Vector2::new(max.0, max.1),
        Vector2::new(min.0, max.1),
    ];
    (0..4)
        .map(|i| LineSegment::new(corners[i], corners[(i + 1) % 4]))
        .collect()
}

/// Built-in worlds; drawn walls are added on top of the preset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum WorldPreset {
    CorridorLoop,
    AliasedCorridor,
    PillarHall,
    EmptyBox,
}

impl WorldPreset {
    const ALL: [Self; 4] = [
        Self::CorridorLoop,
        Self::AliasedCorridor,
        Self::PillarHall,
        Self::EmptyBox,
    ];

    /// Corridor worlds share the centerline used by auto-drive.
    fn has_centerline(self) -> bool {
        matches!(self, Self::CorridorLoop | Self::AliasedCorridor)
    }

    fn start_pose(self) -> Pose2D {
        match self {
            // Start mid-corridor, where no corner is in LiDAR range.
            Self::AliasedCorridor => corridor_loop_start_for(&CorridorLoopConfig {
                start_offset: 12.0,
                ..CorridorLoopConfig::default()
            }),
            _ => corridor_loop_start(),
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::CorridorLoop => "Corridor loop",
            Self::AliasedCorridor => "Aliased corridor",
            Self::PillarHall => "Pillar hall",
            Self::EmptyBox => "Empty box",
        }
    }

    fn slug(self) -> &'static str {
        match self {
            Self::CorridorLoop => "corridor",
            Self::AliasedCorridor => "aliased",
            Self::PillarHall => "hall",
            Self::EmptyBox => "box",
        }
    }

    fn from_slug(value: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|preset| preset.slug() == value)
    }

    fn walls(self) -> Vec<LineSegment> {
        match self {
            Self::CorridorLoop => corridor_loop_walls(),
            Self::AliasedCorridor => corridor_loop_walls_for(CorridorLayout::Periodic {
                spacing: ALIASED_SPACING,
            }),
            Self::PillarHall => {
                let mut walls = rectangle((-15.0, -10.0), (15.0, 10.0));
                for (x, y) in [
                    (-8.6, -4.8),
                    (-3.9, -5.3),
                    (1.2, -4.6),
                    (6.4, -5.1),
                    (10.8, -4.4),
                    (-9.2, 0.4),
                    (-4.4, -0.3),
                    (0.7, 0.6),
                    (5.6, -0.2),
                    (11.1, 0.5),
                    (-8.1, 5.2),
                    (-3.2, 4.7),
                    (1.9, 5.4),
                    (6.9, 4.8),
                    (10.4, 5.6),
                ] {
                    walls.extend(rectangle((x - 0.3, y - 0.3), (x + 0.3, y + 0.3)));
                }
                walls
            }
            Self::EmptyBox => rectangle((-15.0, -10.0), (15.0, 10.0)),
        }
    }
}

fn encode_walls(walls: &[LineSegment]) -> String {
    walls
        .iter()
        .map(|wall| {
            format!(
                "{:.1},{:.1},{:.1},{:.1}",
                wall.start.x, wall.start.y, wall.end.x, wall.end.y
            )
        })
        .collect::<Vec<_>>()
        .join(";")
}

fn decode_walls(value: &str) -> Vec<LineSegment> {
    value
        .split(';')
        .filter_map(|wall| {
            let numbers: Vec<f64> = wall
                .split(',')
                .map(|number| number.parse::<f64>().ok().filter(|n| n.is_finite()))
                .collect::<Option<_>>()?;
            let [x1, y1, x2, y2] = numbers.as_slice() else {
                return None;
            };
            let start = snap(Vector2::new(*x1, *y1));
            let end = snap(Vector2::new(*x2, *y2));
            ((end - start).norm() >= MIN_WALL_LENGTH).then(|| LineSegment::new(start, end))
        })
        .take(MAX_CUSTOM_WALLS)
        .collect()
}

/// Live, keyboard-driven LiDAR graph SLAM on an editable world.
pub struct SlamDriveDemo {
    /// Preset walls plus custom walls, as seen by the LiDAR.
    walls: Vec<LineSegment>,
    pub(crate) preset: WorldPreset,
    pub(crate) custom_walls: Vec<LineSegment>,
    pub(crate) edit_walls: bool,
    drag_start: Option<Vector2<f64>>,
    drag_end: Option<Vector2<f64>>,
    /// Screen rect of the map in the last frame (for tests and hit-testing).
    last_map_rect: Option<Rect>,
    centerline: Vec<Pose2D>,
    rng: StdRng,
    slam: LidarGraphSlam,
    truth: Pose2D,
    odometry: Pose2D,
    pending_odometry: Pose2D,
    pending_distance: f64,
    driven: f64,
    truth_trail: Vec<Pose2D>,
    odometry_trail: Vec<Pose2D>,
    front_end_trail: Vec<Pose2D>,
    /// Ground truth at each pose-graph node.
    node_truth: Vec<Pose2D>,
    last_scan: Vec<Vector2<f64>>,
    /// Reject ambiguous loop matches (perceptual aliasing).
    pub(crate) ambiguity_check: bool,
    pub(crate) odometry_scale_error_pct: f32,
    pub(crate) yaw_drift_deg_per_m: f32,
    pub(crate) range_noise_cm: f32,
    pub(crate) auto_drive: bool,
    pub(crate) show_front_end_map: bool,
    /// Occupancy grid of the node scans at their graph poses.
    grid: OccupancyGrid,
    /// Node scans already folded into `grid`.
    grid_nodes: usize,
    /// Bumped whenever `grid` (or the frozen MCL map) changes.
    grid_version: u64,
    grid_texture: Option<(u64, egui::TextureHandle)>,
    pub(crate) show_grid: bool,
    navigator: Navigator,
    /// Localization on the frozen map after a kidnapping; SLAM is paused.
    mcl: Option<LidarMcl>,
    kidnappings: usize,
    /// Joystick deflection `(forward, turn)` from the last frame.
    joystick: Option<(f64, f64)>,
    /// Raw ranges of each node's scan, so the grid also learns the free
    /// space along beams without a return.
    node_ranges: Vec<Vec<f64>>,
    /// Drive to frontiers until the map is complete.
    pub(crate) explore: bool,
    exploration: ExploreState,
    /// People walking around: seen by the LiDAR, not in the map.
    pub(crate) people_count: usize,
    people: Vec<Pose2D>,
    /// Ticks the robot was blocked by a person.
    person_bumps: usize,
}

/// Progress of frontier exploration.
#[derive(Debug, Default)]
struct ExploreState {
    /// The frontier being driven to and its cluster.
    target: Option<Vector2<f64>>,
    cells: Vec<Vector2<f64>>,
    /// Frontiers the planner could not reach.
    failed: Vec<Vector2<f64>>,
    since_check: f64,
    complete: bool,
    goals: usize,
}

fn empty_grid() -> OccupancyGrid {
    OccupancyGrid::new(
        Vector2::new(WORLD_X.0, WORLD_Y.0),
        Vector2::new(WORLD_X.1, WORLD_Y.1),
        OccupancyConfig::default(),
    )
}

impl Default for SlamDriveDemo {
    fn default() -> Self {
        let mut demo = Self::blank();
        demo.start();
        demo
    }
}

impl SlamDriveDemo {
    /// All state at its initial value, before the first scan is taken.
    fn blank() -> Self {
        let centerline_config = CorridorLoopConfig {
            step: 0.1,
            extra_distance: 0.0,
            ..CorridorLoopConfig::default()
        };
        let mut pose = corridor_loop_start();
        let mut centerline = vec![pose];
        for delta in corridor_loop_deltas(&centerline_config) {
            pose = compose_pose(pose, delta);
            centerline.push(pose);
        }
        Self {
            walls: corridor_loop_walls(),
            preset: WorldPreset::CorridorLoop,
            custom_walls: Vec::new(),
            edit_walls: false,
            drag_start: None,
            drag_end: None,
            last_map_rect: None,
            centerline,
            rng: StdRng::seed_from_u64(7),
            slam: LidarGraphSlam::new(Self::slam_config(true), corridor_loop_start()),
            truth: corridor_loop_start(),
            odometry: corridor_loop_start(),
            pending_odometry: Pose2D::origin(),
            pending_distance: 0.0,
            driven: 0.0,
            truth_trail: Vec::new(),
            odometry_trail: Vec::new(),
            front_end_trail: Vec::new(),
            node_truth: Vec::new(),
            last_scan: Vec::new(),
            ambiguity_check: true,
            odometry_scale_error_pct: 3.0,
            yaw_drift_deg_per_m: 1.0,
            range_noise_cm: 2.0,
            auto_drive: false,
            show_front_end_map: false,
            grid: empty_grid(),
            grid_nodes: 0,
            grid_version: 0,
            grid_texture: None,
            show_grid: true,
            navigator: Navigator::default(),
            mcl: None,
            kidnappings: 0,
            joystick: None,
            node_ranges: Vec::new(),
            explore: false,
            exploration: ExploreState::default(),
            people_count: 0,
            people: Vec::new(),
            person_bumps: 0,
        }
    }

    /// Builds the walls for the current world and takes the first scan.
    fn start(&mut self) {
        let pose = self.preset.start_pose();
        self.truth = pose;
        self.odometry = pose;
        self.slam = LidarGraphSlam::new(Self::slam_config(self.ambiguity_check), pose);
        self.rebuild_walls();
        self.slam_update(Pose2D::origin());
    }

    /// Screen rect of the scene in the last frame.
    #[cfg(test)]
    pub(crate) fn last_map_rect(&self) -> Option<Rect> {
        self.last_map_rect
    }

    /// Whether SLAM is paused for localization on the frozen map.
    fn localizing(&self) -> bool {
        self.mcl.is_some()
    }

    /// Folds node scans added since the last call into the grid.
    fn extend_grid(&mut self) {
        let poses = self.slam.node_poses();
        if poses.len() == self.grid_nodes {
            return;
        }
        for (index, pose) in poses.iter().enumerate().skip(self.grid_nodes) {
            if let Some(ranges) = self.node_ranges.get(index) {
                self.grid.insert_ranges(*pose, ranges, MAX_RANGE);
            } else if let Some(scan) = self.slam.node_scan(index) {
                self.grid.insert_scan(*pose, scan);
            }
        }
        self.grid_nodes = poses.len();
        self.grid_version += 1;
    }

    /// Rebuilds the grid after the graph poses moved (loop closure).
    fn rebuild_grid(&mut self) {
        self.grid = empty_grid();
        self.grid_nodes = 0;
        self.extend_grid();
    }

    /// Spends this frame's optimizer budget on a pending re-optimization.
    fn step_optimizer(&mut self) {
        if self.slam.optimization_pending()
            && !self.slam.optimize_step(OPTIMIZE_ITERATIONS_PER_FRAME)
        {
            self.rebuild_grid();
        }
    }

    /// Freezes the map, teleports the robot to a random free spot and starts
    /// MCL. The first kidnapping spreads the particles over the whole map
    /// (global localization); later ones leave them believing the old pose.
    fn kidnap(&mut self) -> bool {
        if self.mcl.is_none() {
            if self.slam.optimization_pending() {
                self.slam.optimize();
            }
            self.rebuild_grid();
        }
        let grid = self
            .mcl
            .as_ref()
            .map_or(&self.grid, |mcl| mcl.grid())
            .clone();
        let Some(pose) = (0..5_000).find_map(|_| {
            let x = rand::Rng::random_range(&mut self.rng, WORLD_X.0..WORLD_X.1);
            let y = rand::Rng::random_range(&mut self.rng, WORLD_Y.0..WORLD_Y.1);
            let point = Vector2::new(x, y);
            (grid.state_at(point) == CellState::Free
                && wall_clearance(&self.walls, point) > KIDNAP_CLEARANCE)
                .then(|| {
                    let yaw = rand::Rng::random_range(
                        &mut self.rng,
                        -std::f64::consts::PI..std::f64::consts::PI,
                    );
                    Pose2D::new(x, y, yaw)
                })
        }) else {
            return false;
        };
        if self.mcl.is_none() {
            self.mcl = Some(LidarMcl::new(
                grid,
                LidarMclConfig {
                    particles: MCL_PARTICLES,
                    ..LidarMclConfig::default()
                },
            ));
        }
        self.truth = pose;
        self.odometry = pose;
        self.truth_trail.clear();
        self.odometry_trail.clear();
        self.front_end_trail.clear();
        self.set_explore(false);
        self.auto_drive = false;
        self.kidnappings += 1;
        self.grid_version += 1;
        // Localize from the first scan at the new spot.
        self.slam_update(Pose2D::origin());
        true
    }

    fn slam_config(ambiguity_check: bool) -> LidarGraphSlamConfig {
        LidarGraphSlamConfig {
            loop_ambiguity_check: ambiguity_check,
            // Re-optimize over several frames instead of stalling one.
            deferred_optimization: true,
            // Long drives grow the graph; block-sparse PCG keeps each
            // re-optimization interactive.
            pose_graph: PoseGraphConfig {
                max_iterations: 30,
                linear_solver: LinearSolver::BlockSparsePcg {
                    max_iterations: 1_000,
                    tolerance: 1.0e-8,
                },
                ..PoseGraphConfig::default()
            },
            ..LidarGraphSlamConfig::default()
        }
    }

    fn rebuild_walls(&mut self) {
        self.walls = self.preset.walls();
        self.walls.extend(self.custom_walls.iter().copied());
    }

    /// Adds a drawn wall unless it is degenerate or would trap the robot.
    fn add_wall(&mut self, start: Vector2<f64>, end: Vector2<f64>) -> bool {
        let wall = LineSegment::new(snap(start), snap(end));
        let robot = Vector2::new(self.truth.x, self.truth.y);
        if (wall.end - wall.start).norm() < MIN_WALL_LENGTH
            || self.custom_walls.len() >= MAX_CUSTOM_WALLS
            || wall_clearance(&[wall], robot) < ROBOT_RADIUS
        {
            return false;
        }
        self.custom_walls.push(wall);
        self.rebuild_walls();
        true
    }

    fn set_preset(&mut self, preset: WorldPreset) {
        self.preset = preset;
        if !preset.has_centerline() {
            self.auto_drive = false;
        }
        self.reset();
    }

    pub fn reset(&mut self) {
        let mut fresh = Self {
            preset: self.preset,
            custom_walls: std::mem::take(&mut self.custom_walls),
            edit_walls: self.edit_walls,
            ambiguity_check: self.ambiguity_check,
            odometry_scale_error_pct: self.odometry_scale_error_pct,
            yaw_drift_deg_per_m: self.yaw_drift_deg_per_m,
            range_noise_cm: self.range_noise_cm,
            auto_drive: self.auto_drive,
            show_front_end_map: self.show_front_end_map,
            show_grid: self.show_grid,
            people_count: self.people_count,
            ..Self::blank()
        };
        fresh.start();
        *self = fresh;
    }

    pub fn apply_share_query(&mut self, query: &str) {
        if let Some(value) = crate::share::bounded_f32(query, "odom_scale", 0.0, 10.0) {
            self.odometry_scale_error_pct = value;
        }
        if let Some(value) = crate::share::bounded_f32(query, "yaw_drift", 0.0, 3.0) {
            self.yaw_drift_deg_per_m = value;
        }
        if let Some(value) = crate::share::bounded_f32(query, "noise", 0.0, 5.0) {
            self.range_noise_cm = value;
        }
        if let Some(value) = crate::share::boolean(query, "auto") {
            self.auto_drive = value;
        }
        if let Some(value) = crate::share::boolean(query, "frontend_map") {
            self.show_front_end_map = value;
        }
        if let Some(value) = crate::share::boolean(query, "grid") {
            self.show_grid = value;
        }
        if let Some(value) = crate::share::bounded_f32(query, "people", 0.0, MAX_PEOPLE as f32) {
            self.people_count = value as usize;
        }
        let preset = crate::share::value(query, "world").and_then(WorldPreset::from_slug);
        let custom = crate::share::value(query, "walls").map(decode_walls);
        let ambiguity_check = crate::share::boolean(query, "alias_check");
        if preset.is_some() || custom.is_some() || ambiguity_check.is_some() {
            self.preset = preset.unwrap_or(self.preset);
            self.custom_walls = custom.unwrap_or_default();
            self.ambiguity_check = ambiguity_check.unwrap_or(self.ambiguity_check);
            if !self.preset.has_centerline() {
                self.auto_drive = false;
            }
            self.reset();
        }
    }

    pub fn share_query_suffix(&self) -> String {
        let mut query = format!(
            "odom_scale={}&yaw_drift={}&noise={}&auto={}&frontend_map={}&grid={}&people={}&world={}&alias_check={}",
            self.odometry_scale_error_pct,
            self.yaw_drift_deg_per_m,
            self.range_noise_cm,
            u8::from(self.auto_drive),
            u8::from(self.show_front_end_map),
            u8::from(self.show_grid),
            self.people_count,
            self.preset.slug(),
            u8::from(self.ambiguity_check),
        );
        if !self.custom_walls.is_empty() {
            query.push_str("&walls=");
            query.push_str(&encode_walls(&self.custom_walls));
        }
        query
    }

    /// Noisy ranges (infinite for no return) of a LiDAR scan at the truth.
    fn scan(&mut self) -> Vec<f64> {
        let sigma = f64::from(self.range_noise_cm) / 100.0;
        let noise = Normal::new(0.0, sigma.max(1.0e-9)).expect("finite noise");
        ray_cast_ranges(self.truth, &self.scene_walls(), BEAMS, MAX_RANGE)
            .into_iter()
            .map(|range| range + noise.sample(&mut self.rng))
            .collect()
    }

    fn slam_update(&mut self, odom_delta: Pose2D) {
        let ranges = self.scan();
        let scan = ranges_to_points(&ranges);
        if let Some(mcl) = &mut self.mcl {
            mcl.predict(odom_delta);
            mcl.update(&scan);
            self.last_scan = scan;
            self.truth_trail.push(self.truth);
            return;
        }
        let update = self.slam.update(odom_delta, &scan);
        if update.new_node.is_some() {
            self.node_truth.push(self.truth);
            self.node_ranges.push(ranges);
        }
        if update.optimized {
            self.rebuild_grid();
        } else {
            self.extend_grid();
        }
        self.last_scan = scan;
        self.truth_trail.push(self.truth);
        self.odometry_trail.push(self.odometry);
        self.front_end_trail.push(self.slam.front_end_pose());
    }

    /// The pose the robot believes it has and whether it is sure enough to
    /// navigate on it.
    fn believed_pose(&self) -> (Pose2D, bool) {
        match &self.mcl {
            Some(mcl) => {
                let (pose, spread) = mcl.estimate();
                (pose, spread < MCL_CONFIDENT_SPREAD)
            }
            None => (self.slam.pose(), true),
        }
    }

    fn navigation_control(&mut self) -> Option<(f64, f64)> {
        let (pose, confident) = self.believed_pose();
        let grid = self.mcl.as_ref().map_or(&self.grid, |mcl| mcl.grid());
        // The live scan, placed at the estimate, guards against what the
        // map does not show.
        let obstacles = transform_scan_to_world(&self.last_scan, pose);
        self.navigator.control(
            grid,
            pose,
            confident,
            &obstacles,
            DRIVE_SPEED,
            TURN_RATE,
            DT,
        )
    }

    fn auto_control(&self) -> (f64, f64) {
        let nearest = self
            .centerline
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                let da = (a.x - self.truth.x).powi(2) + (a.y - self.truth.y).powi(2);
                let db = (b.x - self.truth.x).powi(2) + (b.y - self.truth.y).powi(2);
                da.total_cmp(&db)
            })
            .map_or(0, |(index, _)| index);
        // The last centerline pose repeats the first, so wrap one short.
        let target = self.centerline[(nearest + AUTO_LOOKAHEAD) % (self.centerline.len() - 1)];
        let heading = (target.y - self.truth.y).atan2(target.x - self.truth.x);
        let error = wrap_angle(heading - self.truth.yaw);
        let omega = (2.5 * error).clamp(-TURN_RATE, TURN_RATE);
        let speed = DRIVE_SPEED * (1.0 - 0.5 * error.abs().min(1.0));
        (speed, omega)
    }

    fn keyboard_control(ctx: &egui::Context) -> (f64, f64) {
        ctx.input(|input| {
            let mut speed = 0.0;
            let mut omega = 0.0;
            if input.key_down(egui::Key::ArrowUp) {
                speed = DRIVE_SPEED;
            }
            if input.key_down(egui::Key::ArrowDown) {
                speed = -0.5 * DRIVE_SPEED;
            }
            if input.key_down(egui::Key::ArrowLeft) {
                omega = TURN_RATE;
            }
            if input.key_down(egui::Key::ArrowRight) {
                omega = -TURN_RATE;
            }
            (speed, omega)
        })
    }

    /// Walls plus the outline of every person, as the LiDAR sees them.
    fn scene_walls(&self) -> Vec<LineSegment> {
        let mut segments = self.walls.clone();
        for person in &self.people {
            segments.extend(rectangle(
                (person.x - PERSON_HALF_SIZE, person.y - PERSON_HALF_SIZE),
                (person.x + PERSON_HALF_SIZE, person.y + PERSON_HALF_SIZE),
            ));
        }
        segments
    }

    /// Distance from the robot center at `point` to the nearest person edge.
    fn person_clearance(&self, point: Vector2<f64>) -> f64 {
        self.people
            .iter()
            .map(|person| {
                let dx = ((point.x - person.x).abs() - PERSON_HALF_SIZE).max(0.0);
                let dy = ((point.y - person.y).abs() - PERSON_HALF_SIZE).max(0.0);
                dx.hypot(dy)
            })
            .fold(f64::INFINITY, f64::min)
    }

    /// Spawns or removes people to match `people_count` and walks them:
    /// straight ahead, turning to a random heading at walls, other people
    /// or on touching the robot.
    fn step_people(&mut self) {
        self.people.truncate(self.people_count);
        let robot = Vector2::new(self.truth.x, self.truth.y);
        while self.people.len() < self.people_count {
            let spawned = (0..500).find_map(|_| {
                let point = Vector2::new(
                    rand::Rng::random_range(&mut self.rng, WORLD_X.0..WORLD_X.1),
                    rand::Rng::random_range(&mut self.rng, WORLD_Y.0..WORLD_Y.1),
                );
                let inside = self.is_inside_world(point);
                (inside && wall_clearance(&self.walls, point) > 1.0 && (point - robot).norm() > 3.0)
                    .then(|| {
                        let heading = rand::Rng::random_range(
                            &mut self.rng,
                            -std::f64::consts::PI..std::f64::consts::PI,
                        );
                        Pose2D::new(point.x, point.y, heading)
                    })
            });
            match spawned {
                Some(person) => self.people.push(person),
                None => break,
            }
        }
        for index in 0..self.people.len() {
            let person = self.people[index];
            let next = compose_pose(person, Pose2D::new(PERSON_SPEED * DT, 0.0, 0.0));
            let point = Vector2::new(next.x, next.y);
            let crowded =
                self.people.iter().enumerate().any(|(other, p)| {
                    other != index && (Vector2::new(p.x, p.y) - point).norm() < 0.8
                });
            // People do not yield to the robot (only a touch stops them):
            // avoiding them is the robot's job.
            if wall_clearance(&self.walls, point) < PERSON_HALF_SIZE + 0.25
                || (point - robot).norm() < ROBOT_RADIUS + PERSON_HALF_SIZE
                || crowded
            {
                let turn = rand::Rng::random_range(&mut self.rng, 1.5..4.7);
                self.people[index].yaw = wrap_angle(person.yaw + turn);
            } else {
                self.people[index] = next;
            }
        }
    }

    /// Whether `point` lies in the walkable area: a ray to the right from
    /// there crosses the closed wall outlines an odd number of times (inside
    /// the outer walls, outside the corridor loop's inner block and pillars).
    fn is_inside_world(&self, point: Vector2<f64>) -> bool {
        let crossings = self
            .walls
            .iter()
            .filter(|wall| {
                let (a, b) = (wall.start, wall.end);
                (a.y > point.y) != (b.y > point.y)
                    && point.x < a.x + (point.y - a.y) / (b.y - a.y) * (b.x - a.x)
            })
            .count();
        crossings % 2 == 1
    }

    /// Advances the simulation by one tick; returns whether the robot moved.
    fn tick(&mut self, speed: f64, omega: f64) -> bool {
        self.step_optimizer();
        self.step_people();
        if speed == 0.0 && omega == 0.0 {
            return false;
        }
        let mut next = compose_pose(self.truth, Pose2D::new(speed * DT, 0.0, omega * DT));
        let point = Vector2::new(next.x, next.y);
        if wall_clearance(&self.walls, point) < ROBOT_RADIUS {
            // Bumped into a wall: turn in place only.
            next = Pose2D::new(self.truth.x, self.truth.y, next.yaw);
        } else if self.person_clearance(point) < ROBOT_RADIUS {
            next = Pose2D::new(self.truth.x, self.truth.y, next.yaw);
            self.person_bumps += 1;
        }
        let true_delta = relative_pose(self.truth, next);
        self.truth = next;

        let noise = Normal::new(0.0, ODOMETRY_NOISE).expect("finite noise");
        let travelled = true_delta.x.hypot(true_delta.y);
        let scale = 1.0 + f64::from(self.odometry_scale_error_pct) / 100.0;
        let drift = f64::from(self.yaw_drift_deg_per_m).to_radians() * travelled;
        let jitter = if travelled > 0.0 { 1.0 } else { 0.0 };
        let odom_delta = Pose2D::new(
            true_delta.x * scale + jitter * noise.sample(&mut self.rng),
            true_delta.y * scale + jitter * noise.sample(&mut self.rng),
            true_delta.yaw + drift + jitter * noise.sample(&mut self.rng),
        );
        self.odometry = compose_pose(self.odometry, odom_delta);
        self.pending_odometry = compose_pose(self.pending_odometry, odom_delta);
        self.pending_distance += travelled;
        self.driven += travelled;

        if self.pending_distance >= UPDATE_DISTANCE || self.pending_odometry.yaw.abs() >= UPDATE_YAW
        {
            let odom = self.pending_odometry;
            self.pending_odometry = Pose2D::origin();
            self.pending_distance = 0.0;
            self.slam_update(odom);
        }
        true
    }

    /// Whether each loop closure is false, judged against ground truth.
    fn loop_is_wrong(&self) -> Vec<bool> {
        self.slam
            .loop_closures()
            .iter()
            .map(|closure| {
                let truth =
                    relative_pose(self.node_truth[closure.from], self.node_truth[closure.to]);
                let error = relative_pose(truth, closure.relative);
                error.x.hypot(error.y) > WRONG_LOOP_TOLERANCE
            })
            .collect()
    }

    fn view(&self) -> (Vec<Pose2D>, Vec<Pose2D>) {
        (self.slam.node_poses(), self.slam.node_front_end_poses())
    }

    fn status(&self, ui: &mut egui::Ui) {
        let error = |pose: Pose2D| {
            let delta = relative_pose(self.truth, pose);
            delta.x.hypot(delta.y)
        };
        if let Some(mcl) = &self.mcl {
            let (estimate, spread) = mcl.estimate();
            ui.label(format!(
                "Localizing on the frozen map (kidnapped {}×) · {} particles · spread {:.2} m · \
                 scan fit {:.2} · {} · true error {:.2} m",
                self.kidnappings,
                mcl.particles().len(),
                spread,
                mcl.fit(),
                if spread < MCL_CONFIDENT_SPREAD {
                    "converged"
                } else {
                    "drive around to disambiguate"
                },
                error(estimate),
            ));
        } else {
            let wrong = self
                .loop_is_wrong()
                .into_iter()
                .filter(|wrong| *wrong)
                .count();
            ui.label(format!(
                "Driven {:.1} m · nodes {} · loop closures {} ({} wrong, {} ambiguous rejected){} · \
                 position error: odometry {:.2} m, scan-to-map {:.3} m, graph SLAM {:.3} m",
                self.driven,
                self.slam.node_poses().len(),
                self.slam.loop_closures().len(),
                wrong,
                self.slam.ambiguous_loop_rejections(),
                if self.slam.optimization_pending() {
                    " · optimizing…"
                } else {
                    ""
                },
                error(self.odometry),
                error(self.slam.front_end_pose()),
                error(self.slam.pose()),
            ));
        }
        if self.explore {
            ui.label(format!(
                "Exploring: frontier {} (orange), {} unreachable skipped.",
                self.exploration.goals,
                self.exploration.failed.len()
            ));
        } else if self.exploration.complete {
            ui.label(format!(
                "Exploration complete: no reachable frontier left after {} goals and {:.0} m.",
                self.exploration.goals, self.driven
            ));
        }
        match &self.navigator.status {
            NavStatus::Idle => {}
            NavStatus::Following => {
                ui.label("Navigating: A* on the occupancy grid, Pure Pursuit on the estimate.");
            }
            NavStatus::Reached => {
                ui.label("Goal reached.");
            }
            NavStatus::Failed(reason) => {
                ui.label(format!("Navigation stopped: {reason}."));
            }
            NavStatus::WaitingForLocalization => {
                ui.label("Waiting for MCL to converge before navigating — drive a little.");
            }
        }
        ui.label(
            "Arrow keys or the joystick drive (click the map first for keys). Click the map to \
             send the robot to a goal: A* plans on the occupancy grid built from the SLAM map and \
             Pure Pursuit follows it; tick Explore and it maps the world by itself, frontier by \
             frontier. Drive a full lap — or tick Auto-drive on the corridor loop — \
             to close the loop. Kidnap robot freezes the map, teleports the robot and localizes it \
             with MCL (yellow particles, gray robot = truth); in long, similar-looking corridors \
             it can converge on a look-alike spot until a distinctive feature comes into view. \
             Purple: wheel odometry · orange: \
             scan-to-map · green: pose graph · magenta: loop edges · red: current scan.",
        );
        match self.preset {
            WorldPreset::CorridorLoop => {
                ui.label("The top corridor has no pillars, so scan matching drifts there.");
            }
            WorldPreset::AliasedCorridor => {
                ui.label(
                    "Pillars repeat every 2 m, so neighboring places look identical. Untick \
                     Reject ambiguous loops and watch loop closures lock onto the wrong pillar \
                     (yellow) and bend the map. Kidnap the robot here and MCL may settle on a \
                     look-alike spot, too.",
                );
            }
            _ => {}
        }
    }

    fn world_controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.label("World:");
            for preset in WorldPreset::ALL {
                if ui
                    .selectable_label(self.preset == preset, preset.label())
                    .clicked()
                    && self.preset != preset
                {
                    self.set_preset(preset);
                }
            }
            ui.separator();
            ui.checkbox(&mut self.edit_walls, "Edit walls (drag on the map)");
            if ui
                .add_enabled(
                    !self.custom_walls.is_empty(),
                    egui::Button::new("Undo wall"),
                )
                .clicked()
            {
                self.custom_walls.pop();
                self.rebuild_walls();
            }
            if ui
                .add_enabled(
                    !self.custom_walls.is_empty(),
                    egui::Button::new("Clear walls"),
                )
                .clicked()
            {
                self.custom_walls.clear();
                self.rebuild_walls();
            }
        });
    }

    /// Handles wall drawing on the map; returns the in-progress wall, if any.
    fn edit_walls(&mut self, response: &egui::Response) -> Option<(Vector2<f64>, Vector2<f64>)> {
        if !self.edit_walls {
            self.drag_start = None;
            self.drag_end = None;
            return None;
        }
        let pointer = response
            .interact_pointer_pos()
            .map(|pos| snap(to_world(response.rect, pos)));
        if response.drag_started() {
            // egui reports a drag only after the pointer moved past a
            // threshold, so anchor the wall where the button went down.
            let origin = response.ctx.input(|input| input.pointer.press_origin());
            self.drag_start = origin
                .map(|pos| snap(to_world(response.rect, pos)))
                .or(pointer);
        }
        if pointer.is_some() {
            self.drag_end = pointer;
        }
        let start = self.drag_start?;
        let end = self.drag_end?;
        if response.drag_stopped() {
            self.drag_start = None;
            self.drag_end = None;
            self.add_wall(start, end);
            return None;
        }
        Some((start, end))
    }

    /// Starts or stops frontier exploration.
    fn set_explore(&mut self, on: bool) {
        self.explore = on && !self.localizing();
        self.exploration = ExploreState::default();
        self.navigator.cancel();
        if self.explore {
            self.auto_drive = false;
        }
    }

    /// Picks the next frontier when the navigator is idle or the current
    /// one has been seen; stops when no reachable frontier is left.
    fn explore_step(&mut self) {
        let config = FrontierConfig::default();
        if let NavStatus::Failed(_) = self.navigator.status {
            if let Some(target) = self.exploration.target.take() {
                self.exploration.failed.push(target);
            }
            self.navigator.status = NavStatus::Idle;
        }
        self.exploration.since_check += DT;
        let seen = self.navigator.is_active()
            && self.exploration.since_check >= EXPLORE_RECHECK
            && self
                .exploration
                .target
                .is_some_and(|target| !is_frontier_near(&self.grid, target, 0.5));
        if self.exploration.since_check >= EXPLORE_RECHECK {
            self.exploration.since_check = 0.0;
        }
        if self.navigator.is_active() && !seen {
            return;
        }
        let pose = self.slam.pose();
        match next_frontier_goal(
            &self.grid,
            Vector2::new(pose.x, pose.y),
            &self.exploration.failed,
            &config,
        ) {
            Some(goal) => {
                self.navigator.set_goal(goal.target);
                self.exploration.target = Some(goal.target);
                self.exploration.cells = goal
                    .frontier
                    .cells
                    .iter()
                    .map(|&(x, y)| self.grid.cell_center(x, y))
                    .collect();
                self.exploration.goals += 1;
            }
            None => {
                self.navigator.cancel();
                self.explore = false;
                self.exploration.target = None;
                self.exploration.cells.clear();
                self.exploration.complete = true;
            }
        }
    }

    /// Driver input this frame: keyboard or joystick, then the navigator,
    /// then auto-drive.
    fn control(&mut self, ctx: &egui::Context) -> (f64, f64) {
        let keyboard = Self::keyboard_control(ctx);
        let manual = if keyboard != (0.0, 0.0) {
            Some(keyboard)
        } else {
            self.joystick.map(|(forward, turn)| {
                let speed = if forward >= 0.0 { 1.0 } else { 0.5 } * forward * DRIVE_SPEED;
                (speed, turn * TURN_RATE)
            })
        };
        if let Some(command) = manual.filter(|command| *command != (0.0, 0.0)) {
            if self.explore {
                self.set_explore(false);
            }
            if self.navigator.is_active() {
                self.navigator.cancel();
            }
            return command;
        }
        if self.explore && !self.localizing() {
            self.explore_step();
        }
        if self.navigator.is_active() {
            return self.navigation_control().unwrap_or((0.0, 0.0));
        }
        if self.auto_drive && !self.localizing() {
            return self.auto_control();
        }
        (0.0, 0.0)
    }

    /// Uploads the occupancy grid texture if the grid changed.
    fn grid_view(
        &mut self,
        ctx: &egui::Context,
    ) -> Option<(egui::TextureId, Vector2<f64>, Vector2<f64>)> {
        if !self.show_grid {
            return None;
        }
        let grid = self.mcl.as_ref().map_or(&self.grid, |mcl| mcl.grid());
        let options = egui::TextureOptions::NEAREST;
        match &mut self.grid_texture {
            Some((version, handle)) => {
                if *version != self.grid_version {
                    handle.set(grid_image(grid), options);
                    *version = self.grid_version;
                }
            }
            None => {
                let handle = ctx.load_texture("slam_drive_grid", grid_image(grid), options);
                self.grid_texture = Some((self.grid_version, handle));
            }
        }
        let (width, height) = grid.size();
        let min = grid.origin();
        let max = min + Vector2::new(width as f64, height as f64) * grid.resolution();
        self.grid_texture
            .as_ref()
            .map(|(_, handle)| (handle.id(), min, max))
    }

    pub fn ui(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        self.world_controls(ui);
        ui.horizontal_wrapped(|ui| {
            let auto_available = self.preset.has_centerline() && !self.localizing();
            if !auto_available {
                self.auto_drive = false;
            }
            if ui
                .add_enabled(
                    auto_available,
                    egui::Checkbox::new(&mut self.auto_drive, "Auto-drive"),
                )
                .changed()
                && self.auto_drive
            {
                self.set_explore(false);
                self.auto_drive = true;
            }
            if ui
                .button("Reset")
                .on_hover_text("Start a new SLAM run")
                .clicked()
            {
                self.reset();
            }
            if ui
                .checkbox(&mut self.ambiguity_check, "Reject ambiguous loops")
                .on_hover_text(
                    "Re-register each loop match from shifted seeds and reject it when another \
                     alignment fits nearly as well (perceptual aliasing). Restarts the run.",
                )
                .changed()
            {
                self.reset();
            }
            ui.checkbox(
                &mut self.show_front_end_map,
                "Map at front-end poses (no loop closure)",
            );
            ui.checkbox(&mut self.show_grid, "Occupancy grid");
        });
        ui.horizontal_wrapped(|ui| {
            let kidnap_label = if self.localizing() {
                "Kidnap again"
            } else {
                "Kidnap robot (freeze map, localize with MCL)"
            };
            if ui
                .add_enabled(
                    self.slam.node_poses().len() >= 5,
                    egui::Button::new(kidnap_label),
                )
                .on_hover_text(
                    "Teleport the robot to a random free spot of the map. SLAM stops; Monte \
                     Carlo localization has to find the robot again on the frozen map.",
                )
                .clicked()
            {
                self.kidnap();
            }
            if let Some(mcl) = &mut self.mcl {
                if ui.button("Spread particles").clicked() {
                    mcl.initialize_global();
                }
            }
            let mut explore = self.explore;
            if ui
                .add_enabled(
                    !self.localizing(),
                    egui::Checkbox::new(&mut explore, "Explore (frontiers)"),
                )
                .on_hover_text(
                    "Drive autonomously to the nearest frontier between known free and unknown \
                     space (A* + Pure Pursuit) until no reachable frontier is left.",
                )
                .changed()
            {
                self.set_explore(explore);
            }
            if self.navigator.is_active() && ui.button("Cancel goal").clicked() {
                self.navigator.cancel();
            }
        });
        // Sliders are composite widgets that a wrapping row cannot break,
        // so stack them on narrow (phone) screens instead of overflowing.
        let sliders = |ui: &mut egui::Ui, demo: &mut Self| {
            ui.add(
                egui::Slider::new(&mut demo.odometry_scale_error_pct, 0.0..=10.0)
                    .text("odometry scale error %"),
            );
            ui.add(
                egui::Slider::new(&mut demo.yaw_drift_deg_per_m, 0.0..=3.0).text("yaw drift °/m"),
            );
            ui.add(egui::Slider::new(&mut demo.range_noise_cm, 0.0..=5.0).text("range noise cm"));
            ui.add(egui::Slider::new(&mut demo.people_count, 0..=MAX_PEOPLE).text("moving people"))
                .on_hover_text(
                    "People walk around: the LiDAR sees them but the map does not. While \
                 navigating, DWA steers around them (yellow arc) when the Pure Pursuit arc \
                 would hit one.",
                );
        };
        if ui.available_width() < 800.0 {
            ui.vertical(|ui| sliders(ui, self));
        } else {
            ui.horizontal(|ui| sliders(ui, self));
        }

        let (speed, omega) = self.control(ctx);
        let moving = self.tick(speed, omega);

        let (nodes, front_end_nodes) = self.view();
        let map_poses = if self.show_front_end_map {
            &front_end_nodes
        } else {
            &nodes
        };
        let grid = self.grid_view(ctx);
        let map = map_poses
            .iter()
            .enumerate()
            .filter_map(|(index, pose)| self.slam.node_scan(index).map(|scan| (*pose, scan)))
            .collect();
        let mut loop_edges = Vec::new();
        let mut wrong_loop_edges = Vec::new();
        for (closure, wrong) in self.slam.loop_closures().iter().zip(self.loop_is_wrong()) {
            let edge = (nodes[closure.from], nodes[closure.to]);
            if wrong {
                wrong_loop_edges.push(edge);
            } else {
                loop_edges.push(edge);
            }
        }
        let (estimate, _) = self.believed_pose();
        let view = LidarSceneView {
            walls: &self.walls,
            map,
            map_at_front_end: self.show_front_end_map,
            truth: &self.truth_trail,
            odometry: &self.odometry_trail,
            front_end: &self.front_end_trail,
            nodes: &nodes,
            loop_edges,
            wrong_loop_edges,
            scan: &self.last_scan,
            estimate,
            front_end_pose: (!self.localizing()).then(|| self.slam.front_end_pose()),
            grid,
        };
        let response = draw_lidar_scene(ui, &view, 96.0);
        let rect = response.rect;
        self.last_map_rect = Some(rect);
        {
            let painter = ui.painter_at(rect);
            if let Some(mcl) = &self.mcl {
                for particle in mcl.particles().iter().step_by(2) {
                    painter.circle_filled(
                        to_screen(rect, particle.pose.x, particle.pose.y),
                        1.2,
                        PARTICLE,
                    );
                }
                draw_robot(&painter, rect, self.truth, TRUTH_ROBOT);
                draw_robot(&painter, rect, estimate, GRAPH);
            }
            for person in &self.people {
                painter.rect_filled(
                    Rect::from_two_pos(
                        to_screen(
                            rect,
                            person.x - PERSON_HALF_SIZE,
                            person.y + PERSON_HALF_SIZE,
                        ),
                        to_screen(
                            rect,
                            person.x + PERSON_HALF_SIZE,
                            person.y - PERSON_HALF_SIZE,
                        ),
                    ),
                    1.0,
                    PERSON,
                );
            }
            if self.explore {
                for cell in &self.exploration.cells {
                    painter.circle_filled(to_screen(rect, cell.x, cell.y), 1.5, FRONTIER);
                }
            }
            if let Some(local) = self.navigator.local_plan() {
                let points = local
                    .points
                    .iter()
                    .map(|p| to_screen(rect, p.x, p.y))
                    .collect();
                painter.add(egui::Shape::line(points, Stroke::new(2.5_f32, LOCAL_PLAN)));
            }
            draw_path(
                &painter,
                |x, y| to_screen(rect, x, y),
                self.navigator.path(),
                self.navigator.goal(),
            );
        }
        if response.clicked() || response.drag_started() {
            // Give the arrow keys back to driving if a slider had focus.
            ctx.memory_mut(|memory| {
                if let Some(id) = memory.focused() {
                    memory.surrender_focus(id);
                }
            });
        }
        if !self.edit_walls && response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                self.navigator.set_goal(to_world(rect, pos));
                self.auto_drive = false;
            }
        }
        if let Some((start, end)) = self.edit_walls(&response) {
            ui.painter_at(rect).line_segment(
                [
                    to_screen(rect, start.x, start.y),
                    to_screen(rect, end.x, end.y),
                ],
                Stroke::new(3.0_f32, Color32::from_rgb(250, 220, 90)),
            );
        }
        self.joystick = joystick(ui, rect);
        ui.separator();
        self.status(ui);

        let keys_held = ctx.input(|input| {
            [
                egui::Key::ArrowUp,
                egui::Key::ArrowDown,
                egui::Key::ArrowLeft,
                egui::Key::ArrowRight,
            ]
            .iter()
            .any(|key| input.key_down(*key))
        });
        if moving
            || self.auto_drive
            || keys_held
            || self.drag_start.is_some()
            || self.joystick.is_some()
            || self.navigator.is_active()
            || self.slam.optimization_pending()
            || !self.people.is_empty()
        {
            ctx.request_repaint_after(std::time::Duration::from_secs_f64(DT));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn auto_drive_follows_the_corridor_without_bumping() {
        let mut demo = SlamDriveDemo::default();
        for _ in 0..600 {
            let (speed, omega) = demo.auto_control();
            demo.tick(speed, omega);
            assert!(
                wall_clearance(&demo.walls, Vector2::new(demo.truth.x, demo.truth.y))
                    >= ROBOT_RADIUS
            );
        }
        // 600 ticks at up to 1.6 m/s for 1/30 s.
        assert!(demo.driven > 20.0, "drove {:.1} m", demo.driven);
        assert!(demo.slam.node_poses().len() > 15);
    }

    #[test]
    fn walls_block_the_robot() {
        // Face the outer wall below the start and push into it.
        let mut demo = SlamDriveDemo {
            truth: Pose2D::new(-12.0, -8.5, -std::f64::consts::FRAC_PI_2),
            ..SlamDriveDemo::default()
        };
        for _ in 0..200 {
            demo.tick(DRIVE_SPEED, 0.0);
        }
        assert!(demo.truth.y > -10.0 + ROBOT_RADIUS - 1e-9);
    }

    #[test]
    fn share_query_round_trips_settings() {
        let demo = SlamDriveDemo {
            odometry_scale_error_pct: 6.5,
            yaw_drift_deg_per_m: 2.0,
            range_noise_cm: 1.0,
            auto_drive: true,
            show_front_end_map: true,
            ..SlamDriveDemo::default()
        };
        let query = demo.share_query_suffix();
        let mut restored = SlamDriveDemo::default();
        restored.apply_share_query(&query);
        assert_eq!(restored.odometry_scale_error_pct, 6.5);
        assert_eq!(restored.yaw_drift_deg_per_m, 2.0);
        assert_eq!(restored.range_noise_cm, 1.0);
        assert!(restored.auto_drive);
        assert!(restored.show_front_end_map);
    }

    #[test]
    fn drawn_walls_snap_block_the_lidar_and_round_trip() {
        let mut demo = SlamDriveDemo {
            preset: WorldPreset::EmptyBox,
            ..SlamDriveDemo::default()
        };
        demo.reset();
        // A wall right in front of the robot (start pose faces +x).
        assert!(demo.add_wall(Vector2::new(-10.04, -9.5), Vector2::new(-9.97, -7.5)));
        assert_eq!(demo.custom_walls[0].start, Vector2::new(-10.0, -9.5));
        let ranges = ray_cast_ranges(demo.truth, &demo.walls, 4, MAX_RANGE);
        assert!(
            (ranges[2] - 2.0).abs() < 1e-9,
            "forward range {}",
            ranges[2]
        );

        let query = demo.share_query_suffix();
        assert!(query.contains("world=box"));
        let mut restored = SlamDriveDemo::default();
        restored.apply_share_query(&query);
        assert_eq!(restored.preset, WorldPreset::EmptyBox);
        assert_eq!(restored.custom_walls, demo.custom_walls);
        assert_eq!(restored.walls.len(), demo.walls.len());
    }

    #[test]
    fn walls_on_the_robot_or_too_short_are_rejected() {
        let mut demo = SlamDriveDemo::default();
        let robot = Vector2::new(demo.truth.x, demo.truth.y);
        let before = demo.walls.len();
        assert!(!demo.add_wall(
            robot - Vector2::new(1.0, 0.0),
            robot + Vector2::new(1.0, 0.0)
        ));
        assert!(!demo.add_wall(Vector2::new(0.0, -8.5), Vector2::new(0.1, -8.5)));
        assert_eq!(demo.walls.len(), before);
        assert!(decode_walls("1,2,3;x,1,2,3;0,0,0,0").is_empty());
    }

    #[test]
    fn pillar_hall_start_is_clear() {
        let walls = WorldPreset::PillarHall.walls();
        let start = corridor_loop_start();
        assert!(wall_clearance(&walls, Vector2::new(start.x, start.y)) > 1.0);
    }

    #[test]
    fn dragging_on_the_map_draws_a_wall() {
        let ctx = egui::Context::default();
        let mut demo = SlamDriveDemo {
            edit_walls: true,
            ..SlamDriveDemo::default()
        };
        let screen = Rect::from_min_size(Pos2::ZERO, Vec2::new(1200.0, 900.0));
        let run = |events: Vec<egui::Event>, demo: &mut SlamDriveDemo| {
            let input = egui::RawInput {
                screen_rect: Some(screen),
                events,
                ..Default::default()
            };
            let _ = ctx.run(input, |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| demo.ui(ctx, ui));
            });
        };
        run(Vec::new(), &mut demo);
        let rect = demo.last_map_rect.expect("map drawn");
        // Inside the corridor loop's central block, away from the robot.
        let (a, b) = (to_screen(rect, -5.0, 0.0), to_screen(rect, 5.0, 0.0));
        let button = |pos, pressed| egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: egui::Modifiers::NONE,
        };
        run(vec![egui::Event::PointerMoved(a)], &mut demo);
        run(vec![button(a, true)], &mut demo);
        for step in 1..=5 {
            let t = step as f32 / 5.0;
            run(vec![egui::Event::PointerMoved(a + (b - a) * t)], &mut demo);
        }
        run(vec![button(b, false)], &mut demo);
        run(Vec::new(), &mut demo);

        assert_eq!(demo.custom_walls.len(), 1, "no wall was drawn");
        let wall = demo.custom_walls[0];
        assert!(
            (wall.start - Vector2::new(-5.0, 0.0)).norm() < 0.15,
            "{:?}",
            wall.start
        );
        assert!(
            (wall.end - Vector2::new(5.0, 0.0)).norm() < 0.15,
            "{:?}",
            wall.end
        );
    }

    #[test]
    fn first_scan_sees_the_selected_world() {
        let mut demo = SlamDriveDemo::default();
        demo.set_preset(WorldPreset::EmptyBox);
        let box_walls = WorldPreset::EmptyBox.walls();
        let scan = demo.slam.node_scan(0).expect("first node");
        assert!(!scan.is_empty());
        for point in transform_scan_to_world(scan, demo.truth) {
            assert!(
                wall_clearance(&box_walls, point) < 0.15,
                "scan point {point:?} is not on the box walls"
            );
        }
    }

    /// Auto-drives `ticks` ticks of the corridor loop to build a map.
    fn mapped_demo(ticks: usize) -> SlamDriveDemo {
        let mut demo = SlamDriveDemo::default();
        for _ in 0..ticks {
            let (speed, omega) = demo.auto_control();
            demo.tick(speed, omega);
        }
        demo
    }

    fn clearance(demo: &SlamDriveDemo) -> f64 {
        wall_clearance(&demo.walls, Vector2::new(demo.truth.x, demo.truth.y))
    }

    /// Drives straight, turning left whenever a wall is close ahead.
    fn wander(demo: &SlamDriveDemo) -> (f64, f64) {
        let pose = demo.truth;
        let ahead = Vector2::new(pose.x + 0.8 * pose.yaw.cos(), pose.y + 0.8 * pose.yaw.sin());
        if wall_clearance(&demo.walls, ahead) < 0.6 {
            (0.0, TURN_RATE)
        } else {
            (DRIVE_SPEED * 0.6, 0.0)
        }
    }

    #[test]
    fn the_occupancy_grid_follows_the_slam_map() {
        let demo = mapped_demo(300);
        assert_eq!(demo.grid_nodes, demo.slam.node_poses().len());
        // Walls the robot has seen are occupied, the corridor it drove is free.
        let start = corridor_loop_start();
        assert_eq!(
            demo.grid.state_at(Vector2::new(start.x, start.y)),
            CellState::Free
        );
        let occupied = demo.grid.occupied_points();
        assert!(occupied.len() > 200, "{} occupied cells", occupied.len());
        let near_wall = occupied
            .iter()
            .filter(|point| wall_clearance(&demo.walls, **point) < 0.2)
            .count();
        assert!(
            near_wall * 10 > occupied.len() * 9,
            "occupied cells off the walls"
        );
    }

    #[test]
    fn navigates_to_a_clicked_goal_on_the_built_map() {
        let mut demo = mapped_demo(450);
        // Back to a spot the robot already mapped: near the start.
        let start = corridor_loop_start();
        let goal = Vector2::new(start.x + 3.0, start.y);
        demo.navigator.set_goal(goal);
        for _ in 0..900 {
            let Some((speed, omega)) = demo.navigation_control() else {
                break;
            };
            demo.tick(speed, omega);
            assert!(clearance(&demo) >= ROBOT_RADIUS);
        }
        assert_eq!(demo.navigator.status, NavStatus::Reached);
        let reached = Vector2::new(demo.truth.x, demo.truth.y);
        assert!((reached - goal).norm() < 0.5, "stopped at {reached:?}");
    }

    #[test]
    fn mcl_finds_the_kidnapped_robot_on_the_frozen_map() {
        // Explore the pillar hall, then kidnap the robot six times in a row:
        // the first is a global localization, the others start from a
        // confident, wrong belief. Require five of six to end within 0.3 m.
        let mut demo = SlamDriveDemo {
            preset: WorldPreset::PillarHall,
            ..SlamDriveDemo::blank()
        };
        demo.start();
        explore(&mut demo, 8_000);
        let nodes = demo.slam.node_poses().len();
        let mut localized = 0;
        for seed in 1..=6 {
            demo.rng = StdRng::seed_from_u64(seed);
            assert!(demo.kidnap());
            assert!(demo.localizing());
            for _ in 0..900 {
                let (speed, omega) = wander(&demo);
                demo.tick(speed, omega);
            }
            let (estimate, _) = demo.believed_pose();
            let error = relative_pose(demo.truth, estimate);
            localized += usize::from(error.x.hypot(error.y) < 0.3);
        }
        assert!(
            localized >= 5,
            "only {localized} of 6 kidnappings localized"
        );
        // SLAM is paused while localizing.
        assert_eq!(demo.slam.node_poses().len(), nodes);
    }

    #[test]
    fn joystick_drives_and_map_clicks_set_goals() {
        let ctx = egui::Context::default();
        let mut demo = SlamDriveDemo::default();
        let screen = Rect::from_min_size(Pos2::ZERO, Vec2::new(1200.0, 900.0));
        let run = |events: Vec<egui::Event>, demo: &mut SlamDriveDemo| {
            let input = egui::RawInput {
                screen_rect: Some(screen),
                events,
                ..Default::default()
            };
            let _ = ctx.run(input, |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| demo.ui(ctx, ui));
            });
        };
        let button = |pos, pressed| egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: egui::Modifiers::NONE,
        };
        run(Vec::new(), &mut demo);
        let rect = demo.last_map_rect.expect("map drawn");
        let radius = (rect.width().min(rect.height()) * 0.12).clamp(36.0, 64.0);
        let stick = rect.right_bottom() - Vec2::splat(radius + 12.0);

        // Push the stick fully forward and hold it.
        let start = demo.truth;
        run(vec![egui::Event::PointerMoved(stick)], &mut demo);
        run(vec![button(stick, true)], &mut demo);
        let up = stick - Vec2::new(0.0, radius);
        for _ in 0..20 {
            run(vec![egui::Event::PointerMoved(up)], &mut demo);
        }
        run(vec![button(up, false)], &mut demo);
        let moved = relative_pose(start, demo.truth);
        assert!(moved.x > 0.5, "joystick did not drive forward: {moved:?}");
        assert!(moved.y.abs() < 0.1 && moved.yaw.abs() < 0.05);
        assert!(demo.navigator.goal().is_none(), "joystick press set a goal");

        // A click elsewhere on the map sets a navigation goal there.
        let target = to_screen(rect, -12.0, -8.5);
        run(vec![egui::Event::PointerMoved(target)], &mut demo);
        run(vec![button(target, true)], &mut demo);
        run(vec![button(target, false)], &mut demo);
        let goal = demo.navigator.goal().expect("goal set");
        assert!((goal - Vector2::new(-12.0, -8.5)).norm() < 0.1, "{goal:?}");
    }

    /// Runs exploration for at most `max_ticks`; returns the ticks used.
    fn explore(demo: &mut SlamDriveDemo, max_ticks: usize) -> usize {
        demo.set_explore(true);
        for tick in 0..max_ticks {
            if !demo.explore {
                return tick;
            }
            demo.explore_step();
            let (speed, omega) = if demo.navigator.is_active() {
                demo.navigation_control().unwrap_or((0.0, 0.0))
            } else {
                (0.0, 0.0)
            };
            demo.tick(speed, omega);
            assert!(clearance(demo) >= ROBOT_RADIUS - 1e-9);
        }
        max_ticks
    }

    #[test]
    fn exploration_completes_on_the_corridor_loop() {
        let mut demo = SlamDriveDemo::default();
        let ticks = explore(&mut demo, 6_000);
        assert!(
            demo.exploration.complete,
            "still exploring after {ticks} ticks"
        );
        // A full lap is ~85 m; exploring sees most of it from a distance.
        assert!(demo.driven > 40.0, "explored only {:.0} m", demo.driven);
    }

    #[test]
    fn navigation_steers_around_walking_people() {
        let mut demo = SlamDriveDemo {
            preset: WorldPreset::PillarHall,
            rng: StdRng::seed_from_u64(4),
            people_count: 6,
            ..SlamDriveDemo::blank()
        };
        demo.start();
        // Across the unexplored hall and back: the map grows on the way and
        // the people are only in the live scan.
        let mut reached = 0;
        for goal in [Vector2::new(10.0, 7.5), Vector2::new(-12.0, -7.5)] {
            demo.navigator.set_goal(goal);
            for _ in 0..1_500 {
                let Some((speed, omega)) = demo.navigation_control() else {
                    break;
                };
                demo.tick(speed, omega);
                assert!(clearance(&demo) >= ROBOT_RADIUS - 1e-9);
            }
            reached += usize::from(demo.navigator.status == NavStatus::Reached);
        }
        assert_eq!(reached, 2, "status {:?}", demo.navigator.status);
        assert_eq!(demo.people.len(), 6);
        assert!(
            demo.person_bumps <= 5,
            "{} ticks blocked by a person",
            demo.person_bumps
        );
    }

    #[test]
    fn exploration_maps_the_pillar_hall() {
        let mut demo = SlamDriveDemo {
            preset: WorldPreset::PillarHall,
            ..SlamDriveDemo::blank()
        };
        demo.start();
        let ticks = explore(&mut demo, 12_000);
        assert!(
            demo.exploration.complete,
            "still exploring after {ticks} ticks"
        );
        // Sample the hall floor away from walls and pillars.
        let walls = WorldPreset::PillarHall.walls();
        let (mut floor, mut known) = (0, 0);
        for i in 0..60 {
            for j in 0..40 {
                let point = Vector2::new(
                    -14.5 + 29.0 * i as f64 / 59.0,
                    -9.5 + 19.0 * j as f64 / 39.0,
                );
                if wall_clearance(&walls, point) < 0.6 {
                    continue;
                }
                floor += 1;
                known += usize::from(demo.grid.state_at(point) == CellState::Free);
            }
        }
        let coverage = known as f64 / floor as f64;
        eprintln!(
            "explored in {ticks} ticks, {:.0} m, {} goals, coverage {coverage:.3}",
            demo.driven, demo.exploration.goals
        );
        assert!(
            coverage > 0.9,
            "only {:.0} % of the floor is known",
            100.0 * coverage
        );
    }

    fn auto_drive_wrong_loops(ambiguity_check: bool, seed: u64) -> (usize, usize) {
        let mut demo = SlamDriveDemo {
            preset: WorldPreset::AliasedCorridor,
            ambiguity_check,
            rng: StdRng::seed_from_u64(seed),
            ..SlamDriveDemo::blank()
        };
        demo.start();
        let lap = 85.4;
        while demo.driven < lap + 14.0 {
            let (speed, omega) = demo.auto_control();
            demo.tick(speed, omega);
        }
        let wrong = demo.loop_is_wrong().into_iter().filter(|w| *w).count();
        (wrong, demo.slam.loop_closures().len())
    }

    #[test]
    fn aliased_corridor_needs_the_ambiguity_check() {
        // Aliasing is stochastic: without the check some seeds lock onto the
        // wrong pillar; with it, no seed may produce a false loop.
        let seeds = 1..=6;
        let failures = seeds
            .clone()
            .filter(|seed| auto_drive_wrong_loops(false, *seed).0 > 0)
            .count();
        assert!(failures > 0, "aliasing never produced a false loop");
        for seed in seeds {
            let (wrong, closures) = auto_drive_wrong_loops(true, seed);
            assert_eq!(
                wrong, 0,
                "seed {seed}: ambiguity check let a false loop through"
            );
            assert!(
                closures > 0,
                "seed {seed}: ambiguity check rejected every loop"
            );
        }
    }
}
