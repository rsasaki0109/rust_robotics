//! Interactive SLAM timeline demo: EKF-SLAM, FastSLAM, and ICP scan matching.

use std::f64::consts::PI;

use egui::{Color32, Pos2, Rect, Stroke, Vec2};
use nalgebra::{DMatrix, Vector2, Vector3};
use rand::{Rng, SeedableRng};
use rust_robotics_slam::{
    ekf_slam::{ekf_slam_known_correspondences, EKFSLAMState},
    fastslam1::{create_particles, fastslam_update, get_best_particle},
    icp_matching::icp_matching,
};

const DT: f64 = 0.1;
const MAX_RANGE: f64 = 18.0;
const WORLD_MIN: f64 = -1.0;
const WORLD_MAX: f64 = 14.0;
const STEPS: usize = 72;

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
}

impl SlamKind {
    fn label(self) -> &'static str {
        match self {
            Self::EkfSlam => "EKF-SLAM",
            Self::FastSlam => "FastSLAM 1.0",
            Self::Icp => "ICP Scan Matching",
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
            kind: SlamKind::EkfSlam,
            frame_idx: 0,
            playing: false,
            ekf_frames,
            fastslam_frames,
            icp_frames,
        }
    }
}

impl SlamDemo {
    fn active_frames(&self) -> &[SlamFrame] {
        match self.kind {
            SlamKind::EkfSlam => &self.ekf_frames,
            SlamKind::FastSlam => &self.fastslam_frames,
            SlamKind::Icp => &self.icp_frames,
        }
    }

    fn reset(&mut self) {
        let kind = self.kind;
        let fresh = Self::default();
        *self = fresh;
        self.kind = kind;
    }

    fn world_rect(&self, ui: &egui::Ui) -> (Rect, f32) {
        let side = ui.available_width().min(ui.available_height() - 8.0);
        let origin = ui.cursor().min;
        (Rect::from_min_size(origin, Vec2::splat(side)), side)
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
        painter.line_segment([center, tip], Stroke::new(2.0, color));
    }

    fn draw_scene(&self, ui: &mut egui::Ui, rect: Rect, side: f32, frame: &SlamFrame) {
        let frames = self.active_frames();
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_rgb(18, 22, 28));

        for lm in &frame.true_landmarks {
            let c = self.world_to_screen(rect, side, lm[0], lm[1]);
            painter.circle_stroke(c, 6.0, Stroke::new(1.5, Color32::from_rgb(200, 180, 80)));
            painter.circle_filled(c, 2.5, Color32::from_rgb(220, 200, 90));
        }

        for lm in &frame.est_landmarks {
            let c = self.world_to_screen(rect, side, lm[0], lm[1]);
            painter.circle_stroke(c, 5.0, Stroke::new(1.2, Color32::from_rgb(255, 140, 80)));
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
                Stroke::new(1.2, Color32::from_rgba_unmultiplied(120, 220, 140, 80)),
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

    pub fn ui(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Algorithm:");
            for kind in [SlamKind::EkfSlam, SlamKind::FastSlam, SlamKind::Icp] {
                if ui
                    .selectable_label(self.kind == kind, kind.label())
                    .clicked()
                {
                    self.kind = kind;
                    self.frame_idx = self
                        .frame_idx
                        .min(self.active_frames().len().saturating_sub(1));
                }
            }
            ui.separator();
            if ui.button("Reset").clicked() {
                self.reset();
            }
            if ui.checkbox(&mut self.playing, "Play").changed() && self.playing {
                ctx.request_repaint();
            }
        });

        let max_idx = self.active_frames().len().saturating_sub(1);
        ui.horizontal(|ui| {
            ui.label(format!("Step {}/{}", self.frame_idx, max_idx));
            ui.add(egui::Slider::new(&mut self.frame_idx, 0..=max_idx).text("timeline"));
        });

        let frame = self.active_frames()[self.frame_idx].clone();
        let (rect, side) = self.world_rect(ui);
        self.draw_scene(ui, rect, side, &frame);

        ui.separator();
        ui.horizontal(|ui| match self.kind {
            SlamKind::EkfSlam | SlamKind::FastSlam => {
                ui.label(format!(
                    "Landmarks: {} true, {} estimated",
                    frame.true_landmarks.len(),
                    frame.est_landmarks.len()
                ));
            }
            SlamKind::Icp => {
                ui.label(format!(
                    "ICP mean error: {:.4} m/point  (prev=blue, curr=red, aligned=green)",
                    frame.icp_error
                ));
            }
        });

        if self.playing && self.frame_idx < max_idx {
            self.frame_idx += 1;
            ctx.request_repaint_after(std::time::Duration::from_millis(120));
        } else if self.frame_idx >= max_idx {
            self.playing = false;
        }
    }
}
