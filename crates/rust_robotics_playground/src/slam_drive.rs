//! Interactive LiDAR graph SLAM: drive a robot through the corridor loop with
//! the arrow keys (or auto-drive) while scan-to-map odometry and loop closure
//! run live, and a shared renderer for LiDAR SLAM scenes.

use egui::{Color32, Pos2, Rect, Stroke, Vec2};
use nalgebra::Vector2;
use rand::{rngs::StdRng, SeedableRng};
use rand_distr::{Distribution, Normal};
use rust_robotics_core::Pose2D;
use rust_robotics_optimization::LinearSolver;
use rust_robotics_slam::{
    lidar_graph_slam::{LidarGraphSlam, LidarGraphSlamConfig},
    lidar_loop_scenario::{
        corridor_loop_deltas, corridor_loop_start, corridor_loop_walls, CorridorLoopConfig,
    },
    pose_graph_optimization::PoseGraphConfig,
    scan_to_map::{
        compose_pose, ranges_to_points, ray_cast_ranges, relative_pose, transform_scan_to_world,
        LineSegment,
    },
};

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

const WALL: Color32 = Color32::from_rgb(90, 95, 105);
const TRUTH: Color32 = Color32::from_rgba_premultiplied(60, 60, 60, 110);
const ODOMETRY: Color32 = Color32::from_rgb(170, 120, 230);
const FRONT_END: Color32 = Color32::from_rgb(240, 150, 70);
const GRAPH: Color32 = Color32::from_rgb(90, 210, 140);
const LOOP_EDGE: Color32 = Color32::from_rgb(230, 90, 220);
const SCAN: Color32 = Color32::from_rgb(255, 90, 100);
const GRAPH_MAP: Color32 = Color32::from_rgba_premultiplied(48, 77, 122, 120);
const FRONT_END_MAP: Color32 = Color32::from_rgba_premultiplied(113, 71, 33, 120);

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
    pub scan: &'a [Vector2<f64>],
    pub estimate: Pose2D,
    pub front_end_pose: Pose2D,
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
    for point in transform_scan_to_world(view.scan, view.estimate) {
        painter.circle_filled(to_screen(rect, point.x, point.y), 1.4, SCAN);
    }
    draw_robot(&painter, rect, view.front_end_pose, FRONT_END);
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
    PillarHall,
    EmptyBox,
}

impl WorldPreset {
    const ALL: [Self; 3] = [Self::CorridorLoop, Self::PillarHall, Self::EmptyBox];

    fn label(self) -> &'static str {
        match self {
            Self::CorridorLoop => "Corridor loop",
            Self::PillarHall => "Pillar hall",
            Self::EmptyBox => "Empty box",
        }
    }

    fn slug(self) -> &'static str {
        match self {
            Self::CorridorLoop => "corridor",
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
    last_scan: Vec<Vector2<f64>>,
    pub(crate) odometry_scale_error_pct: f32,
    pub(crate) yaw_drift_deg_per_m: f32,
    pub(crate) range_noise_cm: f32,
    pub(crate) auto_drive: bool,
    pub(crate) show_front_end_map: bool,
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
            slam: LidarGraphSlam::new(Self::slam_config(), corridor_loop_start()),
            truth: corridor_loop_start(),
            odometry: corridor_loop_start(),
            pending_odometry: Pose2D::origin(),
            pending_distance: 0.0,
            driven: 0.0,
            truth_trail: Vec::new(),
            odometry_trail: Vec::new(),
            front_end_trail: Vec::new(),
            last_scan: Vec::new(),
            odometry_scale_error_pct: 3.0,
            yaw_drift_deg_per_m: 1.0,
            range_noise_cm: 2.0,
            auto_drive: false,
            show_front_end_map: false,
        }
    }

    /// Builds the walls for the current world and takes the first scan.
    fn start(&mut self) {
        self.rebuild_walls();
        self.slam_update(Pose2D::origin());
    }

    fn slam_config() -> LidarGraphSlamConfig {
        LidarGraphSlamConfig {
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
        if preset != WorldPreset::CorridorLoop {
            self.auto_drive = false;
        }
        self.reset();
    }

    pub fn reset(&mut self) {
        let mut fresh = Self {
            preset: self.preset,
            custom_walls: std::mem::take(&mut self.custom_walls),
            edit_walls: self.edit_walls,
            odometry_scale_error_pct: self.odometry_scale_error_pct,
            yaw_drift_deg_per_m: self.yaw_drift_deg_per_m,
            range_noise_cm: self.range_noise_cm,
            auto_drive: self.auto_drive,
            show_front_end_map: self.show_front_end_map,
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
        let preset = crate::share::value(query, "world").and_then(WorldPreset::from_slug);
        let custom = crate::share::value(query, "walls").map(decode_walls);
        if preset.is_some() || custom.is_some() {
            self.preset = preset.unwrap_or(self.preset);
            self.custom_walls = custom.unwrap_or_default();
            if self.preset != WorldPreset::CorridorLoop {
                self.auto_drive = false;
            }
            self.reset();
        }
    }

    pub fn share_query_suffix(&self) -> String {
        let mut query = format!(
            "odom_scale={}&yaw_drift={}&noise={}&auto={}&frontend_map={}&world={}",
            self.odometry_scale_error_pct,
            self.yaw_drift_deg_per_m,
            self.range_noise_cm,
            u8::from(self.auto_drive),
            u8::from(self.show_front_end_map),
            self.preset.slug(),
        );
        if !self.custom_walls.is_empty() {
            query.push_str("&walls=");
            query.push_str(&encode_walls(&self.custom_walls));
        }
        query
    }

    fn scan(&mut self) -> Vec<Vector2<f64>> {
        let sigma = f64::from(self.range_noise_cm) / 100.0;
        let noise = Normal::new(0.0, sigma.max(1.0e-9)).expect("finite noise");
        let ranges: Vec<f64> = ray_cast_ranges(self.truth, &self.walls, BEAMS, MAX_RANGE)
            .into_iter()
            .map(|range| range + noise.sample(&mut self.rng))
            .collect();
        ranges_to_points(&ranges)
    }

    fn slam_update(&mut self, odom_delta: Pose2D) {
        let scan = self.scan();
        self.slam.update(odom_delta, &scan);
        self.last_scan = scan;
        self.truth_trail.push(self.truth);
        self.odometry_trail.push(self.odometry);
        self.front_end_trail.push(self.slam.front_end_pose());
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

    /// Advances the simulation by one tick; returns whether the robot moved.
    fn tick(&mut self, speed: f64, omega: f64) -> bool {
        if speed == 0.0 && omega == 0.0 {
            return false;
        }
        let mut next = compose_pose(self.truth, Pose2D::new(speed * DT, 0.0, omega * DT));
        if wall_clearance(&self.walls, Vector2::new(next.x, next.y)) < ROBOT_RADIUS {
            // Bumped into a wall: turn in place only.
            next = Pose2D::new(self.truth.x, self.truth.y, next.yaw);
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

    fn view(&self) -> (Vec<Pose2D>, Vec<Pose2D>) {
        (self.slam.node_poses(), self.slam.node_front_end_poses())
    }

    fn status(&self, ui: &mut egui::Ui) {
        let error = |pose: Pose2D| {
            let delta = relative_pose(self.truth, pose);
            delta.x.hypot(delta.y)
        };
        ui.label(format!(
            "Driven {:.1} m · nodes {} · loop closures {} · position error: odometry {:.2} m, \
             scan-to-map {:.3} m, graph SLAM {:.3} m",
            self.driven,
            self.slam.node_poses().len(),
            self.slam.loop_closures().len(),
            error(self.odometry),
            error(self.slam.front_end_pose()),
            error(self.slam.pose()),
        ));
        ui.label(
            "Arrow keys drive (click the map first). Drive a full lap — or tick Auto-drive on \
             the corridor loop — and return to the start to close the loop. Tick Edit walls and \
             drag on the map to build your own course. Purple: wheel odometry · orange: \
             scan-to-map · green: pose graph · magenta: loop edges · red: current scan.",
        );
        if self.preset == WorldPreset::CorridorLoop {
            ui.label("The top corridor has no pillars, so scan matching drifts there.");
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

    pub fn ui(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        self.world_controls(ui);
        ui.horizontal(|ui| {
            let auto_available = self.preset == WorldPreset::CorridorLoop;
            if !auto_available {
                self.auto_drive = false;
            }
            ui.add_enabled(
                auto_available,
                egui::Checkbox::new(&mut self.auto_drive, "Auto-drive"),
            );
            if ui.button("Reset").clicked() {
                self.reset();
            }
            ui.checkbox(
                &mut self.show_front_end_map,
                "Map at front-end poses (no loop closure)",
            );
        });
        ui.horizontal_wrapped(|ui| {
            ui.add(
                egui::Slider::new(&mut self.odometry_scale_error_pct, 0.0..=10.0)
                    .text("odometry scale error %"),
            );
            ui.add(
                egui::Slider::new(&mut self.yaw_drift_deg_per_m, 0.0..=3.0).text("yaw drift °/m"),
            );
            ui.add(egui::Slider::new(&mut self.range_noise_cm, 0.0..=5.0).text("range noise cm"));
        });

        let (speed, omega) = if self.auto_drive {
            self.auto_control()
        } else {
            Self::keyboard_control(ctx)
        };
        let moving = self.tick(speed, omega);

        let (nodes, front_end_nodes) = self.view();
        let map_poses = if self.show_front_end_map {
            &front_end_nodes
        } else {
            &nodes
        };
        let map = map_poses
            .iter()
            .enumerate()
            .filter_map(|(index, pose)| self.slam.node_scan(index).map(|scan| (*pose, scan)))
            .collect();
        let loop_edges = self
            .slam
            .loop_closures()
            .iter()
            .map(|closure| (nodes[closure.from], nodes[closure.to]))
            .collect();
        let view = LidarSceneView {
            walls: &self.walls,
            map,
            map_at_front_end: self.show_front_end_map,
            truth: &self.truth_trail,
            odometry: &self.odometry_trail,
            front_end: &self.front_end_trail,
            nodes: &nodes,
            loop_edges,
            scan: &self.last_scan,
            estimate: self.slam.pose(),
            front_end_pose: self.slam.front_end_pose(),
        };
        let response = draw_lidar_scene(ui, &view, 96.0);
        self.last_map_rect = Some(response.rect);
        if response.clicked() || response.drag_started() {
            // Give the arrow keys back to driving if a slider had focus.
            ctx.memory_mut(|memory| {
                if let Some(id) = memory.focused() {
                    memory.surrender_focus(id);
                }
            });
        }
        if let Some((start, end)) = self.edit_walls(&response) {
            let rect = response.rect;
            ui.painter_at(rect).line_segment(
                [
                    to_screen(rect, start.x, start.y),
                    to_screen(rect, end.x, end.y),
                ],
                Stroke::new(3.0_f32, Color32::from_rgb(250, 220, 90)),
            );
        }
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
        if moving || self.auto_drive || keys_held || self.drag_start.is_some() {
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
}
