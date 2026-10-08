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
    ui.allocate_rect(rect, egui::Sense::click())
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

/// Live, keyboard-driven LiDAR graph SLAM on the corridor loop.
pub struct SlamDriveDemo {
    walls: Vec<LineSegment>,
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
        let mut demo = Self {
            walls: corridor_loop_walls(),
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
        };
        demo.slam_update(Pose2D::origin());
        demo
    }
}

impl SlamDriveDemo {
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

    pub fn reset(&mut self) {
        let fresh = Self {
            odometry_scale_error_pct: self.odometry_scale_error_pct,
            yaw_drift_deg_per_m: self.yaw_drift_deg_per_m,
            range_noise_cm: self.range_noise_cm,
            auto_drive: self.auto_drive,
            show_front_end_map: self.show_front_end_map,
            ..Self::default()
        };
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
    }

    pub fn share_query_suffix(&self) -> String {
        format!(
            "odom_scale={}&yaw_drift={}&noise={}&auto={}&frontend_map={}",
            self.odometry_scale_error_pct,
            self.yaw_drift_deg_per_m,
            self.range_noise_cm,
            u8::from(self.auto_drive),
            u8::from(self.show_front_end_map)
        )
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
            "Arrow keys drive (click the map first). Drive a full lap — or tick Auto-drive — \
             and return to the start to close the loop. Purple: wheel odometry · orange: \
             scan-to-map · green: pose graph · magenta: loop edges · red: current scan. \
             The top corridor has no pillars, so scan matching drifts there.",
        );
    }

    pub fn ui(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.auto_drive, "Auto-drive");
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
        if draw_lidar_scene(ui, &view, 72.0).clicked() {
            // Give the arrow keys back to driving if a slider had focus.
            ctx.memory_mut(|memory| {
                if let Some(id) = memory.focused() {
                    memory.surrender_focus(id);
                }
            });
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
        if moving || self.auto_drive || keys_held {
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
}
