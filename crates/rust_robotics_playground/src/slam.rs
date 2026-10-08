//! Interactive SLAM demos: EKF-SLAM, FastSLAM, ICP scan matching, a LiDAR
//! graph SLAM loop-closure replay, and live LiDAR SLAM driving.

use std::f64::consts::PI;

use egui::{Color32, Pos2, Rect, Stroke, Vec2};
use nalgebra::{DMatrix, Vector2, Vector3};
use rand::{Rng, SeedableRng};
use rust_robotics_core::Pose2D;
use rust_robotics_slam::{
    ekf_slam::{ekf_slam_known_correspondences, EKFSLAMState},
    fastslam1::{create_particles, fastslam_update, get_best_particle},
    icp_matching::icp_matching,
    lidar_graph_slam::LidarGraphSlamConfig,
    lidar_loop_scenario::{
        aliased_corridor_config, run_corridor_loop, CorridorLoopConfig, CorridorLoopRun,
    },
    scan_to_map::relative_pose,
};

use crate::slam_drive::{draw_lidar_scene, LidarSceneView, MapView, SlamDriveDemo};

const DT: f64 = 0.1;
const MAX_RANGE: f64 = 18.0;
const WORLD_MIN: f64 = -1.0;
const WORLD_MAX: f64 = 14.0;
const STEPS: usize = 72;

const LOOP_FRAMES_PER_TICK: usize = 2;
/// A loop edge more than this far from ground truth is a false closure \[m\].
const WRONG_LOOP_TOLERANCE: f64 = 0.5;

/// Recorded runs available in the loop-closure replay.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LoopScenario {
    Corridor,
    AliasedNoCheck,
    AliasedWithCheck,
}

impl LoopScenario {
    const ALL: [Self; 3] = [Self::Corridor, Self::AliasedNoCheck, Self::AliasedWithCheck];

    fn index(self) -> usize {
        match self {
            Self::Corridor => 0,
            Self::AliasedNoCheck => 1,
            Self::AliasedWithCheck => 2,
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::Corridor => "Corridor loop",
            Self::AliasedNoCheck => "Aliased corridor, no ambiguity check",
            Self::AliasedWithCheck => "Aliased corridor, ambiguity check",
        }
    }

    fn slug(self) -> &'static str {
        match self {
            Self::Corridor => "corridor",
            Self::AliasedNoCheck => "aliased_off",
            Self::AliasedWithCheck => "aliased_on",
        }
    }

    fn from_slug(value: &str) -> Option<Self> {
        Self::ALL
            .into_iter()
            .find(|scenario| scenario.slug() == value)
    }

    fn run(self) -> CorridorLoopRun {
        let (scenario, ambiguity_check) = match self {
            Self::Corridor => (CorridorLoopConfig::default(), true),
            Self::AliasedNoCheck => (aliased_corridor_config(), false),
            Self::AliasedWithCheck => (aliased_corridor_config(), true),
        };
        run_corridor_loop(
            &scenario,
            LidarGraphSlamConfig {
                loop_ambiguity_check: ambiguity_check,
                ..LidarGraphSlamConfig::default()
            },
        )
    }
}

const LANDMARKS: [[f64; 2]; 6] = [
    [2.5, 1.5],
    [6.0, 1.5],
    [9.5, 1.5],
    [2.5, 6.5],
    [6.0, 6.5],
    [9.5, 6.5],
];

const R_DIST: f64 = 0.3;
const R_ANGLE: f64 = 5.0 * PI / 180.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SlamKind {
    EkfSlam,
    FastSlam,
    Icp,
    LoopClosure,
    Drive,
}

impl SlamKind {
    fn label(self) -> &'static str {
        match self {
            Self::EkfSlam => "EKF-SLAM",
            Self::FastSlam => "FastSLAM 1.0",
            Self::Icp => "ICP Scan Matching",
            Self::LoopClosure => "LiDAR Loop Closure",
            Self::Drive => "Drive LiDAR SLAM",
        }
    }

    fn slug(self) -> &'static str {
        match self {
            Self::EkfSlam => "ekf",
            Self::FastSlam => "fastslam",
            Self::Icp => "icp",
            Self::LoopClosure => "loop",
            Self::Drive => "drive",
        }
    }

    fn from_slug(value: &str) -> Option<Self> {
        match value {
            "ekf" => Some(Self::EkfSlam),
            "fastslam" => Some(Self::FastSlam),
            "icp" => Some(Self::Icp),
            "loop" => Some(Self::LoopClosure),
            "drive" => Some(Self::Drive),
            _ => None,
        }
    }
}

#[derive(Clone)]
struct SlamFrame {
    true_pose: [f64; 3],
    est_pose: [f64; 3],
    true_landmarks: Vec<[f64; 2]>,
    est_landmarks: Vec<[f64; 2]>,
    particles: Vec<[f64; 2]>,
    scan_prev: Vec<[f64; 2]>,
    scan_curr: Vec<[f64; 2]>,
    scan_aligned: Vec<[f64; 2]>,
    icp_error: f64,
}

pub struct SlamDemo {
    kind: SlamKind,
    frame_idx: usize,
    playing: bool,
    ekf_frames: Vec<SlamFrame>,
    fastslam_frames: Vec<SlamFrame>,
    icp_frames: Vec<SlamFrame>,
    /// Selected loop-closure replay scenario.
    loop_scenario: LoopScenario,
    /// Recorded runs per scenario, computed the first time each is shown.
    loop_runs: [Option<CorridorLoopRun>; 3],
    /// Render the loop-closure map at front-end poses (before correction).
    show_front_end_map: bool,
    /// Live, keyboard-driven LiDAR graph SLAM.
    drive: SlamDriveDemo,
    /// Input time of the last replay step \[s\].
    last_advance: f64,
}

fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle;
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

fn motion_model(x: Vector3<f64>, u: Vector2<f64>) -> Vector3<f64> {
    Vector3::new(
        x[0] + u[0] * DT * x[2].cos(),
        x[1] + u[0] * DT * x[2].sin(),
        normalize_angle(x[2] + u[1] * DT),
    )
}

fn control(step: usize) -> Vector2<f64> {
    let phase = step % 36;
    if phase < 14 {
        Vector2::new(1.0, 0.0)
    } else if phase < 20 {
        Vector2::new(0.35, 0.55)
    } else if phase < 28 {
        Vector2::new(0.9, 0.0)
    } else {
        Vector2::new(0.35, 0.55)
    }
}

fn noisy_observations(rng: &mut rand::rngs::StdRng, pose: Vector3<f64>) -> Vec<(f64, f64, usize)> {
    let mut z = Vec::new();
    for (id, lm) in LANDMARKS.iter().enumerate() {
        let dx = lm[0] - pose[0];
        let dy = lm[1] - pose[1];
        let d = (dx * dx + dy * dy).sqrt();
        if d <= MAX_RANGE {
            let angle = normalize_angle(dy.atan2(dx) - pose[2]);
            let d_noisy = d + rng.sample(rand_distr::Normal::new(0.0, R_DIST).unwrap());
            let angle_noisy = angle + rng.sample(rand_distr::Normal::new(0.0, R_ANGLE).unwrap());
            z.push((d_noisy.max(0.05), angle_noisy, id));
        }
    }
    z
}

fn simulate_scan(rng: &mut rand::rngs::StdRng, pose: Vector3<f64>) -> Vec<[f64; 2]> {
    let mut pts = Vec::new();
    for lm in LANDMARKS {
        let dx = lm[0] - pose[0];
        let dy = lm[1] - pose[1];
        let d = (dx * dx + dy * dy).sqrt();
        if d <= MAX_RANGE && d > 0.4 {
            let nx = lm[0] + rng.sample(rand_distr::Normal::new(0.0, 0.04).unwrap());
            let ny = lm[1] + rng.sample(rand_distr::Normal::new(0.0, 0.04).unwrap());
            pts.push([nx, ny]);
        }
    }
    for deg in (0..360).step_by(15) {
        let ray = deg as f64 * PI / 180.0;
        let dir = Vector2::new(ray.cos(), ray.sin());
        let mut best = MAX_RANGE;
        for lm in LANDMARKS {
            let rel = Vector2::new(lm[0] - pose[0], lm[1] - pose[1]);
            let along = rel.dot(&dir);
            if along > 0.2 {
                let perp = (rel[0] * dir[1] - rel[1] * dir[0]).abs();
                if perp < 0.35 && along < best {
                    best = along;
                }
            }
        }
        if best < MAX_RANGE {
            let hit = Vector2::new(pose[0], pose[1]) + dir * best;
            let nx = hit[0] + rng.sample(rand_distr::Normal::new(0.0, 0.03).unwrap());
            let ny = hit[1] + rng.sample(rand_distr::Normal::new(0.0, 0.03).unwrap());
            pts.push([nx, ny]);
        }
    }
    pts
}

fn points_to_matrix(points: &[[f64; 2]]) -> DMatrix<f64> {
    let mut m = DMatrix::zeros(2, points.len());
    for (j, p) in points.iter().enumerate() {
        m[(0, j)] = p[0];
        m[(1, j)] = p[1];
    }
    m
}

fn apply_transform(
    points: &[[f64; 2]],
    rot: &DMatrix<f64>,
    trans: &nalgebra::DVector<f64>,
) -> Vec<[f64; 2]> {
    let mat = points_to_matrix(points);
    let out = rot * mat;
    points
        .iter()
        .enumerate()
        .map(|(j, _)| [out[(0, j)] + trans[0], out[(1, j)] + trans[1]])
        .collect()
}

fn run_ekf_timeline(rng: &mut rand::rngs::StdRng) -> Vec<SlamFrame> {
    let mut true_pose = Vector3::zeros();
    let mut ekf = EKFSLAMState::new();
    let mut frames = Vec::with_capacity(STEPS);

    for step in 0..STEPS {
        let u = control(step);
        let obs = noisy_observations(rng, true_pose);
        ekf_slam_known_correspondences(&mut ekf, &u, &obs, LANDMARKS.len());
        true_pose = motion_model(true_pose, u);

        let est = ekf.get_robot_pose();
        let est_landmarks: Vec<[f64; 2]> = (0..ekf.n_landmarks())
            .filter_map(|i| ekf.get_landmark(i).map(|lm| [lm[0], lm[1]]))
            .collect();

        frames.push(SlamFrame {
            true_pose: [true_pose[0], true_pose[1], true_pose[2]],
            est_pose: [est[0], est[1], est[2]],
            true_landmarks: LANDMARKS.to_vec(),
            est_landmarks,
            particles: Vec::new(),
            scan_prev: Vec::new(),
            scan_curr: Vec::new(),
            scan_aligned: Vec::new(),
            icp_error: 0.0,
        });
    }
    frames
}

fn run_fastslam_timeline(rng: &mut rand::rngs::StdRng) -> Vec<SlamFrame> {
    let mut true_pose = Vector3::zeros();
    let mut particles = create_particles(60, LANDMARKS.len());
    let mut frames = Vec::with_capacity(STEPS);

    for step in 0..STEPS {
        let u = control(step);
        let obs = noisy_observations(rng, true_pose);
        fastslam_update(&mut particles, u, &obs);
        true_pose = motion_model(true_pose, u);

        let best = get_best_particle(&particles);
        let est_landmarks: Vec<[f64; 2]> = best
            .landmarks
            .iter()
            .filter(|lm| lm.cov[(0, 0)] < 100.0)
            .map(|lm| [lm.x, lm.y])
            .collect();
        let particles: Vec<[f64; 2]> = particles.iter().step_by(3).map(|p| [p.x, p.y]).collect();

        frames.push(SlamFrame {
            true_pose: [true_pose[0], true_pose[1], true_pose[2]],
            est_pose: [best.x, best.y, best.yaw],
            true_landmarks: LANDMARKS.to_vec(),
            est_landmarks,
            particles,
            scan_prev: Vec::new(),
            scan_curr: Vec::new(),
            scan_aligned: Vec::new(),
            icp_error: 0.0,
        });
    }
    frames
}

fn run_icp_timeline(rng: &mut rand::rngs::StdRng) -> Vec<SlamFrame> {
    let mut true_pose = Vector3::zeros();
    let mut prev_scan = simulate_scan(rng, true_pose);
    let mut frames = Vec::with_capacity(STEPS);

    for step in 0..STEPS {
        let u = control(step);
        true_pose = motion_model(true_pose, u);
        let curr_scan = simulate_scan(rng, true_pose);

        let (scan_aligned, icp_error) = if prev_scan.len() >= 3 && curr_scan.len() >= 3 {
            let prev_mat = points_to_matrix(&prev_scan);
            let curr_mat = points_to_matrix(&curr_scan);
            let result = icp_matching(&prev_mat, &curr_mat);
            (
                apply_transform(&curr_scan, &result.rotation, &result.translation),
                result.final_error_mean,
            )
        } else {
            (curr_scan.clone(), 0.0)
        };

        frames.push(SlamFrame {
            true_pose: [true_pose[0], true_pose[1], true_pose[2]],
            est_pose: [true_pose[0], true_pose[1], true_pose[2]],
            true_landmarks: LANDMARKS.to_vec(),
            est_landmarks: Vec::new(),
            particles: Vec::new(),
            scan_prev: prev_scan.clone(),
            scan_curr: curr_scan.clone(),
            scan_aligned,
            icp_error,
        });
        prev_scan = curr_scan;
        let _ = step;
    }
    frames
}

fn build_timelines() -> (Vec<SlamFrame>, Vec<SlamFrame>, Vec<SlamFrame>) {
    let mut rng = rand::rngs::StdRng::seed_from_u64(42);
    (
        run_ekf_timeline(&mut rng),
        run_fastslam_timeline(&mut rng),
        run_icp_timeline(&mut rng),
    )
}

impl Default for SlamDemo {
    fn default() -> Self {
        let (ekf_frames, fastslam_frames, icp_frames) = build_timelines();
        Self {
            kind: SlamKind::Drive,
            frame_idx: 0,
            playing: true,
            ekf_frames,
            fastslam_frames,
            icp_frames,
            loop_scenario: LoopScenario::Corridor,
            loop_runs: [None, None, None],
            show_front_end_map: false,
            drive: SlamDriveDemo::default(),
            last_advance: 0.0,
        }
    }
}

impl SlamDemo {
    pub fn apply_share_query(&mut self, query: &str) {
        if let Some(kind) = crate::share::value(query, "algorithm").and_then(SlamKind::from_slug) {
            self.kind = kind;
        }
        // The loop-closure run is computed lazily, so its frame index is
        // clamped when the mode is first drawn.
        let max_idx = match self.kind {
            SlamKind::LoopClosure | SlamKind::Drive => usize::MAX,
            _ => self.active_frames().len().saturating_sub(1),
        };
        if let Some(frame) = crate::share::bounded_usize(query, "frame", max_idx) {
            self.frame_idx = frame;
        }
        if let Some(playing) = crate::share::boolean(query, "playing") {
            self.playing = playing;
        }
        if let Some(front_end) = crate::share::boolean(query, "frontend_map") {
            self.show_front_end_map = front_end;
        }
        if let Some(scenario) =
            crate::share::value(query, "scenario").and_then(LoopScenario::from_slug)
        {
            self.loop_scenario = scenario;
        }
        if self.kind == SlamKind::Drive {
            self.drive.apply_share_query(query);
        }
    }

    pub fn share_query(&self) -> String {
        if self.kind == SlamKind::Drive {
            return format!(
                "tab=slam&algorithm=drive&{}",
                self.drive.share_query_suffix()
            );
        }
        let mut query = format!(
            "tab=slam&algorithm={}&frame={}&playing={}",
            self.kind.slug(),
            self.frame_idx,
            u8::from(self.playing)
        );
        if self.kind == SlamKind::LoopClosure {
            query.push_str(&format!(
                "&frontend_map={}&scenario={}",
                u8::from(self.show_front_end_map),
                self.loop_scenario.slug()
            ));
        }
        query
    }

    fn active_frames(&self) -> &[SlamFrame] {
        match self.kind {
            SlamKind::EkfSlam => &self.ekf_frames,
            SlamKind::FastSlam => &self.fastslam_frames,
            SlamKind::Icp | SlamKind::LoopClosure | SlamKind::Drive => &self.icp_frames,
        }
    }

    fn loop_run(&mut self) -> &CorridorLoopRun {
        let scenario = self.loop_scenario;
        self.loop_runs[scenario.index()].get_or_insert_with(|| scenario.run())
    }

    fn frame_count(&mut self) -> usize {
        match self.kind {
            SlamKind::LoopClosure => self.loop_run().frames.len(),
            _ => self.active_frames().len(),
        }
    }

    fn reset(&mut self) {
        let kind = self.kind;
        let loop_runs = std::mem::take(&mut self.loop_runs);
        let loop_scenario = self.loop_scenario;
        let drive = std::mem::take(&mut self.drive);
        *self = Self::default();
        self.kind = kind;
        self.loop_runs = loop_runs;
        self.loop_scenario = loop_scenario;
        self.drive = drive;
    }

    fn world_rect(ui: &egui::Ui) -> (Rect, f32) {
        let rect = crate::ui_kit::fit_rect(ui, 1.0, 72.0);
        (rect, rect.width())
    }

    fn world_to_screen(&self, rect: Rect, side: f32, x: f64, y: f64) -> Pos2 {
        let u = ((x - WORLD_MIN) / (WORLD_MAX - WORLD_MIN)) as f32;
        let v = 1.0 - ((y - WORLD_MIN) / (WORLD_MAX - WORLD_MIN)) as f32;
        rect.min + Vec2::new(u * side, v * side)
    }

    fn draw_robot(painter: &egui::Painter, center: Pos2, yaw: f64, color: Color32, radius: f32) {
        painter.circle_filled(center, radius, color);
        let tip = center
            + Vec2::new(
                (yaw.cos() as f32) * radius * 1.8,
                -(yaw.sin() as f32) * radius * 1.8,
            );
        painter.line_segment([center, tip], Stroke::new(2.0_f32, color));
    }

    fn draw_scene(&self, ui: &mut egui::Ui, rect: Rect, side: f32, frame: &SlamFrame) {
        let frames = self.active_frames();
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_rgb(18, 22, 28));

        for lm in &frame.true_landmarks {
            let c = self.world_to_screen(rect, side, lm[0], lm[1]);
            painter.circle_stroke(
                c,
                6.0,
                Stroke::new(1.5_f32, Color32::from_rgb(200, 180, 80)),
            );
            painter.circle_filled(c, 2.5, Color32::from_rgb(220, 200, 90));
        }

        for lm in &frame.est_landmarks {
            let c = self.world_to_screen(rect, side, lm[0], lm[1]);
            painter.circle_stroke(
                c,
                5.0,
                Stroke::new(1.2_f32, Color32::from_rgb(255, 140, 80)),
            );
            painter.circle_filled(c, 2.0, Color32::from_rgb(255, 120, 60));
        }

        if matches!(self.kind, SlamKind::Icp) {
            for p in &frame.scan_prev {
                let c = self.world_to_screen(rect, side, p[0], p[1]);
                painter.circle_filled(c, 2.0, Color32::from_rgba_unmultiplied(100, 180, 255, 140));
            }
            for p in &frame.scan_curr {
                let c = self.world_to_screen(rect, side, p[0], p[1]);
                painter.circle_filled(c, 2.0, Color32::from_rgba_unmultiplied(255, 100, 100, 140));
            }
            for p in &frame.scan_aligned {
                let c = self.world_to_screen(rect, side, p[0], p[1]);
                painter.circle_filled(c, 2.5, Color32::from_rgba_unmultiplied(120, 255, 160, 180));
            }
        }

        if matches!(self.kind, SlamKind::FastSlam) {
            for p in &frame.particles {
                let c = self.world_to_screen(rect, side, p[0], p[1]);
                painter.circle_filled(c, 1.5, Color32::from_rgba_unmultiplied(100, 180, 255, 60));
            }
        }

        if self.frame_idx >= 1 {
            let trail: Vec<Pos2> = frames[..=self.frame_idx]
                .iter()
                .map(|f| self.world_to_screen(rect, side, f.true_pose[0], f.true_pose[1]))
                .collect();
            painter.add(egui::Shape::line(
                trail,
                Stroke::new(1.2_f32, Color32::from_rgba_unmultiplied(120, 220, 140, 80)),
            ));
        }

        let true_c = self.world_to_screen(rect, side, frame.true_pose[0], frame.true_pose[1]);
        Self::draw_robot(
            &painter,
            true_c,
            frame.true_pose[2],
            Color32::from_rgb(120, 220, 140),
            7.0,
        );

        if !matches!(self.kind, SlamKind::Icp) {
            let est_c = self.world_to_screen(rect, side, frame.est_pose[0], frame.est_pose[1]);
            Self::draw_robot(
                &painter,
                est_c,
                frame.est_pose[2],
                Color32::from_rgb(255, 160, 90),
                6.0,
            );
        }
    }

    fn draw_loop_scene(&mut self, ui: &mut egui::Ui) {
        let frame_idx = self.frame_idx;
        let show_front_end_map = self.show_front_end_map;
        let run = self.loop_run();
        let frame = &run.frames[frame_idx];
        let node_count = frame.node_poses.len();
        let map_poses = if show_front_end_map {
            &run.node_front_end[..node_count]
        } else {
            &frame.node_poses[..]
        };
        let history = &run.frames[..=frame_idx];
        let truth: Vec<Pose2D> = history.iter().map(|f| f.truth).collect();
        let odometry: Vec<Pose2D> = history.iter().map(|f| f.odometry).collect();
        let front_end: Vec<Pose2D> = history.iter().map(|f| f.front_end).collect();
        let view = LidarSceneView {
            walls: &run.walls,
            map: map_poses
                .iter()
                .zip(&run.node_scans)
                .map(|(pose, scan)| (*pose, scan.as_slice()))
                .collect(),
            map_at_front_end: show_front_end_map,
            truth: &truth,
            odometry: &odometry,
            front_end: &front_end,
            nodes: &frame.node_poses,
            loop_edges: Vec::new(),
            wrong_loop_edges: Vec::new(),
            scan: &frame.scan,
            estimate: frame.estimate,
            front_end_pose: Some(frame.front_end),
            grid: None,
        };
        let mut view = view;
        for closure in run
            .loop_closures
            .iter()
            .filter(|closure| closure.to < node_count)
        {
            let edge = (frame.node_poses[closure.from], frame.node_poses[closure.to]);
            if run.is_wrong_loop(closure, WRONG_LOOP_TOLERANCE) {
                view.wrong_loop_edges.push(edge);
            } else {
                view.loop_edges.push(edge);
            }
        }
        draw_lidar_scene(ui, &view, 72.0, MapView::default());
    }

    fn loop_status(&mut self, ui: &mut egui::Ui) {
        let frame_idx = self.frame_idx;
        let run = self.loop_run();
        let frame = &run.frames[frame_idx];
        let error = |pose: Pose2D| {
            let delta = relative_pose(frame.truth, pose);
            delta.x.hypot(delta.y)
        };
        let closures: Vec<_> = run
            .loop_closures
            .iter()
            .filter(|closure| closure.to < frame.node_poses.len())
            .collect();
        let wrong = closures
            .iter()
            .filter(|closure| run.is_wrong_loop(closure, WRONG_LOOP_TOLERANCE))
            .count();
        ui.label(format!(
            "{} loop closures ({} wrong)  ·  error: scan-to-map {:.2} m, graph SLAM {:.2} m",
            closures.len(),
            wrong,
            error(frame.front_end),
            error(frame.estimate),
        ));
    }

    pub fn controls(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Mode");
        ui.horizontal_wrapped(|ui| {
            if ui
                .selectable_label(self.kind == SlamKind::Drive, "Drive (live)")
                .clicked()
            {
                self.kind = SlamKind::Drive;
            }
        });
        crate::ui_kit::section(ui, "Replays");
        ui.horizontal_wrapped(|ui| {
            for kind in [
                SlamKind::EkfSlam,
                SlamKind::FastSlam,
                SlamKind::Icp,
                SlamKind::LoopClosure,
            ] {
                if ui
                    .selectable_label(self.kind == kind, kind.label())
                    .clicked()
                {
                    self.kind = kind;
                    self.frame_idx = 0;
                    self.playing = true;
                }
            }
        });

        if self.kind == SlamKind::Drive {
            self.drive.controls(ctx, ui);
            return;
        }

        if self.kind == SlamKind::LoopClosure {
            crate::ui_kit::section(ui, "Scenario");
            for scenario in LoopScenario::ALL {
                if ui
                    .selectable_label(self.loop_scenario == scenario, scenario.label())
                    .clicked()
                {
                    self.loop_scenario = scenario;
                    self.playing = false;
                }
            }
            let max_idx = self.frame_count().saturating_sub(1);
            self.frame_idx = self.frame_idx.min(max_idx);
            crate::ui_kit::section(ui, "Jump");
            ui.horizontal_wrapped(|ui| {
                if let Some(first_loop) = self.loop_run().first_loop_frame() {
                    if ui.button("Just before the loop").clicked() {
                        self.frame_idx = first_loop.saturating_sub(1);
                        self.playing = false;
                    }
                    if ui.button("First loop closure").clicked() {
                        self.frame_idx = first_loop;
                        self.playing = false;
                    }
                }
            });
            ui.checkbox(&mut self.show_front_end_map, "Map without loop closure");
            crate::ui_kit::legend(
                ui,
                &[
                    (Color32::from_rgb(170, 120, 230), "odometry"),
                    (Color32::from_rgb(240, 150, 70), "scan-to-map"),
                    (Color32::from_rgb(90, 210, 140), "pose graph"),
                    (Color32::from_rgb(230, 90, 220), "loop edge"),
                    (Color32::from_rgb(250, 220, 90), "false loop"),
                ],
            );
            let note = if self.loop_scenario == LoopScenario::Corridor {
                "The top corridor has no pillars, so the scan-to-map front end drifts there \
                 until the loop closes and the pose graph pulls everything straight."
            } else {
                "Aliased corridor: identical pillars every 2.5 m and a start mid-corridor. \
                 Without the ambiguity check, loop matches lock onto the pillar next door \
                 and bend the map; with it, those matches are rejected until a unique view \
                 (a corner) closes the loop."
            };
            crate::ui_kit::how_it_works(ui, "loop_help", note);
        } else {
            let note = match self.kind {
                SlamKind::EkfSlam => {
                    "EKF-SLAM keeps the robot pose and every landmark in one Gaussian; each \
                     range-bearing observation corrects them all together."
                }
                SlamKind::FastSlam => {
                    "FastSLAM 1.0: each particle is a pose hypothesis with its own small \
                     landmark EKFs; particles that explain the observations survive."
                }
                _ => {
                    "ICP aligns the current scan (red) to the previous one (blue) by \
                     alternating nearest-neighbor matching and a rigid fit (green = aligned)."
                }
            };
            crate::ui_kit::how_it_works(ui, "replay_help", note);
        }
        if ui.button("Reset").clicked() {
            self.reset();
        }
    }

    pub fn scene(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        if self.kind == SlamKind::Drive {
            self.drive.scene(ctx, ui);
            return;
        }

        let max_idx = self.frame_count().saturating_sub(1);
        self.frame_idx = self.frame_idx.min(max_idx);
        if self.kind == SlamKind::LoopClosure {
            self.draw_loop_scene(ui);
            crate::ui_kit::playback(ui, &mut self.playing, &mut self.frame_idx, max_idx);
            self.loop_status(ui);
        } else {
            let frame = self.active_frames()[self.frame_idx].clone();
            let (rect, side) = Self::world_rect(ui);
            self.draw_scene(ui, rect, side, &frame);
            let _ = ui.allocate_rect(rect, egui::Sense::hover());
            crate::ui_kit::playback(ui, &mut self.playing, &mut self.frame_idx, max_idx);
            match self.kind {
                SlamKind::EkfSlam | SlamKind::FastSlam => {
                    ui.label(format!(
                        "{} landmarks, {} estimated",
                        frame.true_landmarks.len(),
                        frame.est_landmarks.len()
                    ));
                }
                _ => {
                    ui.label(format!("ICP mean error {:.4} m per point", frame.icp_error));
                }
            }
        }

        let (advance, period) = match self.kind {
            SlamKind::LoopClosure => (LOOP_FRAMES_PER_TICK, 0.05),
            _ => (1, 0.12),
        };
        if self.playing && self.frame_idx < max_idx {
            if crate::ui_kit::every(ctx, &mut self.last_advance, period) {
                self.frame_idx = (self.frame_idx + advance).min(max_idx);
            }
        } else if self.frame_idx >= max_idx {
            self.playing = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{LoopScenario, SlamDemo, SlamKind};

    #[test]
    fn share_query_round_trips_timeline() {
        let demo = SlamDemo {
            kind: SlamKind::Icp,
            frame_idx: 17,
            playing: true,
            ..SlamDemo::default()
        };
        let query = demo.share_query();
        let mut restored = SlamDemo::default();
        restored.apply_share_query(&query);
        assert_eq!(restored.kind, SlamKind::Icp);
        assert_eq!(restored.frame_idx, 17);
        assert!(restored.playing);
    }

    #[test]
    fn loop_closure_share_query_restores_without_running_the_scenario() {
        let demo = SlamDemo {
            kind: SlamKind::LoopClosure,
            frame_idx: 321,
            show_front_end_map: true,
            loop_scenario: LoopScenario::AliasedNoCheck,
            ..SlamDemo::default()
        };
        let query = demo.share_query();
        assert!(query.contains("algorithm=loop"));
        assert!(query.contains("scenario=aliased_off"));
        assert!(query.contains("frontend_map=1"));
        let mut restored = SlamDemo::default();
        restored.apply_share_query(&query);
        assert_eq!(restored.kind, SlamKind::LoopClosure);
        assert_eq!(restored.frame_idx, 321);
        assert!(restored.show_front_end_map);
        assert_eq!(restored.loop_scenario, LoopScenario::AliasedNoCheck);
        assert!(restored.loop_runs.iter().all(Option::is_none));
    }

    #[test]
    fn drive_scene_fits_a_phone_screen() {
        // Inside the narrow-layout scroll area, the scene (and the joystick
        // in its corner) must not overflow a 390 px wide screen.
        let ctx = egui::Context::default();
        let mut demo = SlamDemo::default();
        demo.apply_share_query("algorithm=drive");
        for _ in 0..3 {
            let input = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(390.0, 844.0),
                )),
                ..Default::default()
            };
            let _ = ctx.run(input, |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    egui::ScrollArea::vertical()
                        .auto_shrink([false, false])
                        .show(ui, |ui| demo.scene(ctx, ui));
                });
            });
        }
        let map = demo.drive.last_map_rect().expect("scene drawn");
        assert!(map.right() <= 390.0, "scene overflows: {map:?}");
        assert!(map.width() > 300.0, "scene too small: {map:?}");
    }
}
