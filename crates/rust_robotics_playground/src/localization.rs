//! Interactive localization demo: Particle Filter and EKF. Drive with the
//! arrow keys, by holding the pointer on the map, or let the robot tour the
//! landmarks on its own.

mod belief;

use std::{collections::BTreeMap, f64::consts::PI};

use egui::{Color32, Pos2, Rect, Stroke, Vec2};
use nalgebra::{Matrix2, Matrix4, Vector2};
use rand::Rng;
use rust_robotics_core::{ControlInput, Obstacles, Point2D, State2D};
use rust_robotics_localization::{
    EKFConfig, EKFLocalizer, PFMeasurement, ParticleFilterConfig, ParticleFilterLocalizer,
};

const DT: f64 = 0.1;
const LINEAR_SPEED: f64 = 1.2;
const TURN_RATE: f64 = 1.0;
const WORLD_MIN: f64 = -1.0;
const WORLD_MAX: f64 = 12.0;
const SENSOR: Color32 = Color32::from_rgb(91, 215, 231);
const TRUTH: Color32 = Color32::from_rgb(90, 220, 120);
const HISTORY: usize = 120;

/// A fixed sensor clock. Ignore background-tab gaps instead of catching up.
#[derive(Default)]
struct SensorClock {
    remainder: f64,
}

impl SensorClock {
    fn ticks(&mut self, elapsed: f64, playing: bool) -> usize {
        if !playing || !elapsed.is_finite() || !(0.0..1.0).contains(&elapsed) {
            self.remainder = 0.0;
            return 0;
        }
        self.remainder += elapsed;
        let ticks = ((self.remainder + 1e-9) / DT).floor() as usize;
        self.remainder = (self.remainder - ticks as f64 * DT).max(0.0);
        ticks
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FilterKind {
    ParticleFilter,
    Ekf,
}

impl FilterKind {
    fn label(self) -> &'static str {
        match self {
            Self::ParticleFilter => "Particle Filter",
            Self::Ekf => "EKF",
        }
    }

    fn slug(self) -> &'static str {
        match self {
            Self::ParticleFilter => "pf",
            Self::Ekf => "ekf",
        }
    }

    fn from_slug(value: &str) -> Option<Self> {
        match value {
            "pf" => Some(Self::ParticleFilter),
            "ekf" => Some(Self::Ekf),
            _ => None,
        }
    }
}

pub struct LocalizationDemo {
    belief: belief::BeliefLab,
    filter: FilterKind,
    true_state: State2D,
    pf: ParticleFilterLocalizer,
    ekf: EKFLocalizer,
    landmarks: Obstacles,
    noise_scale: f32,
    step_count: u64,
    trail_true: Vec<Point2D>,
    trail_est: Vec<Point2D>,
    /// Tour the landmarks on its own.
    auto_drive: bool,
    /// Next corner of the automatic tour.
    tour_index: usize,
    playing: bool,
    single_step: bool,
    clock: SensorClock,
    last_time: Option<f64>,
    ranges: PFMeasurement,
    position_fix: Option<Point2D>,
    errors: Vec<f64>,
    show_observations: bool,
    anchor: usize,
    actions: BTreeMap<&'static str, [f32; 2]>,
    action_hits: BTreeMap<&'static str, bool>,
    #[cfg(target_arch = "wasm32")]
    action_pointer: Option<egui::Pos2>,
    #[cfg(target_arch = "wasm32")]
    scroll_revision: u64,
    #[cfg(target_arch = "wasm32")]
    scrolling: bool,
    scene_rect: Option<Rect>,
    #[cfg(target_arch = "wasm32")]
    action_count: u64,
    #[cfg(target_arch = "wasm32")]
    last_action: &'static str,
    #[cfg(target_arch = "wasm32")]
    ui_revision: u64,
}

/// Corners of the automatic tour around the landmarks.
const TOUR: [(f64, f64); 4] = [(8.5, 3.5), (8.5, 6.5), (3.5, 6.5), (3.5, 3.5)];

impl Default for LocalizationDemo {
    fn default() -> Self {
        let landmarks = Obstacles::from_points(vec![
            Point2D::new(2.0, 2.0),
            Point2D::new(10.0, 2.0),
            Point2D::new(2.0, 8.0),
            Point2D::new(10.0, 8.0),
            Point2D::new(6.0, 5.0),
        ]);

        let mut pf = ParticleFilterLocalizer::with_initial_state_2d(
            State2D::new(5.0, 5.0, 0.0, 0.0),
            ParticleFilterConfig {
                n_particles: 120,
                dt: DT,
                range_noise: 0.25,
                ..ParticleFilterConfig::default()
            },
        )
        .expect("valid PF config");
        pf.set_landmarks_from_obstacles(&landmarks)
            .expect("valid landmarks");

        let ekf = EKFLocalizer::with_initial_state_2d(
            State2D::new(5.0, 5.0, 0.0, 0.0),
            EKFConfig::default(),
        )
        .expect("valid EKF config");

        Self {
            belief: belief::BeliefLab::default(),
            filter: FilterKind::ParticleFilter,
            true_state: State2D::new(5.0, 5.0, 0.0, 0.0),
            pf,
            ekf,
            landmarks,
            noise_scale: 1.0,
            step_count: 0,
            trail_true: Vec::new(),
            trail_est: Vec::new(),
            auto_drive: true,
            tour_index: 0,
            playing: true,
            single_step: false,
            clock: SensorClock::default(),
            last_time: None,
            ranges: Vec::new(),
            position_fix: None,
            errors: Vec::new(),
            show_observations: true,
            anchor: 0,
            actions: BTreeMap::new(),
            action_hits: BTreeMap::new(),
            #[cfg(target_arch = "wasm32")]
            action_pointer: None,
            #[cfg(target_arch = "wasm32")]
            scroll_revision: 0,
            #[cfg(target_arch = "wasm32")]
            scrolling: false,
            scene_rect: None,
            #[cfg(target_arch = "wasm32")]
            action_count: 0,
            #[cfg(target_arch = "wasm32")]
            last_action: "",
            #[cfg(target_arch = "wasm32")]
            ui_revision: 0,
        }
    }
}

impl LocalizationDemo {
    pub fn apply_share_query(&mut self, query: &str) {
        if crate::share::value(query, "tab") != Some("localization") {
            return;
        }
        self.belief.apply_query(query);
        if let Some(filter) = crate::share::value(query, "filter").and_then(FilterKind::from_slug) {
            self.filter = filter;
        }
        if let Some(noise) = crate::share::bounded_f32(query, "noise", 0.2, 3.0) {
            self.noise_scale = noise;
        }
        if let Some(playing) = crate::share::boolean(query, "loc_playing") {
            self.playing = playing;
        }
        if let Some(tour) = crate::share::boolean(query, "loc_tour") {
            self.auto_drive = tour;
        }
        if let Some(show) = crate::share::boolean(query, "loc_observations") {
            self.show_observations = show;
        }
        if let Some(anchor) = crate::share::bounded_usize(query, "loc_anchor", 4) {
            self.anchor = anchor;
        }
        self.reset();
    }

    pub fn share_query(&self) -> String {
        if self.belief.active {
            return self.belief.share_query();
        }
        format!(
            "tab=localization&filter={}&noise={:.2}&loc_playing={}&loc_tour={}&loc_observations={}&loc_anchor={}",
            self.filter.slug(),
            self.noise_scale,
            u8::from(self.playing),
            u8::from(self.auto_drive),
            u8::from(self.show_observations),
            self.anchor,
        )
    }

    fn reset(&mut self) {
        let filter = self.filter;
        let noise = self.noise_scale;
        let fresh = Self::default();
        self.true_state = fresh.true_state;
        self.pf = fresh.pf;
        self.ekf = fresh.ekf;
        self.landmarks = fresh.landmarks;
        self.step_count = 0;
        self.tour_index = 0;
        self.trail_true.clear();
        self.trail_est.clear();
        self.filter = filter;
        self.noise_scale = noise;
        self.ranges.clear();
        self.position_fix = None;
        self.errors.clear();
        self.single_step = false;
        self.clock.remainder = 0.0;
        self.last_time = None;
        self.apply_noise_scale();
    }

    fn read_control(&self, ctx: &egui::Context) -> ControlInput {
        let mut v = 0.0;
        let mut omega = 0.0;
        ctx.input(|input| {
            if input.key_down(egui::Key::ArrowUp) {
                v = LINEAR_SPEED;
            }
            if input.key_down(egui::Key::ArrowDown) {
                v = -LINEAR_SPEED * 0.5;
            }
            if input.key_down(egui::Key::ArrowLeft) {
                omega = TURN_RATE;
            }
            if input.key_down(egui::Key::ArrowRight) {
                omega = -TURN_RATE;
            }
        });
        ControlInput::new(v, omega)
    }

    /// Drives toward a world point: turn first, then go.
    fn steer_toward(&self, target: Point2D) -> ControlInput {
        let dx = target.x - self.true_state.x;
        let dy = target.y - self.true_state.y;
        if dx.hypot(dy) < 0.3 {
            return ControlInput::new(0.0, 0.0);
        }
        let error = (dy.atan2(dx) - self.true_state.yaw + PI).rem_euclid(2.0 * PI) - PI;
        let omega = (2.0 * error).clamp(-TURN_RATE, TURN_RATE);
        let v = if error.abs() < 1.0 {
            LINEAR_SPEED
        } else {
            0.2 * LINEAR_SPEED
        };
        ControlInput::new(v, omega)
    }

    fn tour_control(&mut self) -> ControlInput {
        let (x, y) = TOUR[self.tour_index];
        let corner = Point2D::new(x, y);
        if (corner.x - self.true_state.x).hypot(corner.y - self.true_state.y) < 0.6 {
            self.tour_index = (self.tour_index + 1) % TOUR.len();
        }
        let (x, y) = TOUR[self.tour_index];
        self.steer_toward(Point2D::new(x, y))
    }

    fn screen_to_world(rect: Rect, pos: Pos2) -> Point2D {
        let u = f64::from((pos.x - rect.min.x) / rect.width());
        let v = f64::from((pos.y - rect.min.y) / rect.height());
        Point2D::new(
            WORLD_MIN + u * (WORLD_MAX - WORLD_MIN),
            WORLD_MIN + (1.0 - v) * (WORLD_MAX - WORLD_MIN),
        )
    }

    fn propagate_true(state: &mut State2D, control: ControlInput) {
        state.x += control.v * state.yaw.cos() * DT;
        state.y += control.v * state.yaw.sin() * DT;
        state.yaw += control.omega * DT;
        state.v = control.v;
    }

    fn pf_measurements(&self, noise: f64) -> PFMeasurement {
        let mut rng = rand::rng();
        self.landmarks
            .points
            .iter()
            .map(|landmark| {
                let dx = self.true_state.x - landmark.x;
                let dy = self.true_state.y - landmark.y;
                let range = (dx * dx + dy * dy).sqrt();
                let noisy = range + rng.sample(rand_distr::Normal::new(0.0, noise).unwrap());
                (noisy.max(0.0), landmark.x, landmark.y)
            })
            .collect()
    }

    fn noisy_position(&self, noise: f64) -> Point2D {
        let mut rng = rand::rng();
        let nx = rng.sample(rand_distr::Normal::new(0.0, noise).unwrap());
        let ny = rng.sample(rand_distr::Normal::new(0.0, noise).unwrap());
        Point2D::new(self.true_state.x + nx, self.true_state.y + ny)
    }

    fn apply_noise_scale(&mut self) {
        let sigma = self.sensor_sigma();
        let _ = self.pf.set_range_noise(sigma);

        let r = sigma.powi(2).max(1e-4);
        let _ = self
            .ekf
            .try_set_measurement_noise(Matrix2::new(r, 0.0, 0.0, r));
    }

    fn step_simulation(&mut self, control: ControlInput) {
        self.apply_noise_scale();
        let meas_sigma = self.sensor_sigma();

        Self::propagate_true(&mut self.true_state, control);

        let estimate = match self.filter {
            FilterKind::ParticleFilter => {
                self.ranges = self.pf_measurements(meas_sigma);
                self.pf
                    .try_step_state(control, &self.ranges)
                    .unwrap_or_else(|_| self.pf.state_2d())
            }
            FilterKind::Ekf => {
                let z = self.noisy_position(meas_sigma);
                self.position_fix = Some(z);
                self.ekf
                    .estimate_state(z, control, DT)
                    .unwrap_or_else(|_| self.ekf.state_2d())
            }
        };

        let true_point = Point2D::new(self.true_state.x, self.true_state.y);
        let est_point = Point2D::new(estimate.x, estimate.y);
        self.push_trail(true_point, est_point);
        self.errors
            .push((true_point.x - est_point.x).hypot(true_point.y - est_point.y));
        if self.errors.len() > HISTORY {
            self.errors.remove(0);
        }

        self.step_count += 1;
    }

    fn sensor_sigma(&self) -> f64 {
        0.12 * f64::from(self.noise_scale)
    }

    fn estimate(&self) -> State2D {
        match self.filter {
            FilterKind::ParticleFilter => self.pf.state_2d(),
            FilterKind::Ekf => self.ekf.state_2d(),
        }
    }

    fn estimate_color(&self) -> Color32 {
        match self.filter {
            FilterKind::ParticleFilter => Color32::from_rgb(80, 170, 255),
            FilterKind::Ekf => Color32::from_rgb(255, 140, 70),
        }
    }

    fn action(&mut self, name: &'static str, response: &egui::Response) {
        self.actions.insert(name, response.rect.center().into());
        self.action_hits.insert(name, response.hovered());
        if response.clicked() {
            #[cfg(target_arch = "wasm32")]
            {
                self.action_count += 1;
                self.last_action = name;
            }
            response.ctx.request_repaint();
        }
    }

    fn push_trail(&mut self, truth: Point2D, est: Point2D) {
        const MAX: usize = 200;
        self.trail_true.push(truth);
        self.trail_est.push(est);
        if self.trail_true.len() > MAX {
            let excess = self.trail_true.len() - MAX;
            self.trail_true.drain(0..excess);
            self.trail_est.drain(0..excess);
        }
    }

    fn world_rect(ui: &egui::Ui) -> (Rect, f32) {
        let rect = crate::ui_kit::fit_rect(ui, 1.0, 120.0);
        (rect, rect.width())
    }

    fn world_to_screen(&self, rect: Rect, side: f32, p: Point2D) -> Pos2 {
        let u = ((p.x - WORLD_MIN) / (WORLD_MAX - WORLD_MIN)) as f32;
        let v = 1.0 - ((p.y - WORLD_MIN) / (WORLD_MAX - WORLD_MIN)) as f32;
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

    fn draw_covariance_ellipse(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        side: f32,
        center: Point2D,
        cov: Matrix4<f64>,
        color: Color32,
    ) {
        let pos_cov = Matrix2::new(cov[(0, 0)], cov[(0, 1)], cov[(1, 0)], cov[(1, 1)]);
        let eigen = pos_cov.symmetric_eigen();
        // 95% probability contour of the model's 2D Gaussian, chi²(2).
        let k = 5.991_f64.sqrt();

        let mut points = Vec::with_capacity(33);
        for i in 0..=32 {
            let t = i as f64 / 32.0 * 2.0 * PI;
            let dir = Vector2::new(t.cos(), t.sin());
            let lambda1 = eigen.eigenvalues[0].max(0.0);
            let lambda2 = eigen.eigenvalues[1].max(0.0);
            let v1 = eigen.eigenvectors.column(0);
            let v2 = eigen.eigenvectors.column(1);
            let offset = v1 * (k * lambda1.sqrt() * dir.x) + v2 * (k * lambda2.sqrt() * dir.y);
            let p = Point2D::new(center.x + offset.x, center.y + offset.y);
            points.push(self.world_to_screen(rect, side, p));
        }
        painter.add(egui::Shape::closed_line(
            points,
            Stroke::new(1.5_f32, color),
        ));
    }

    fn draw_scene(&self, ui: &mut egui::Ui, rect: Rect, side: f32) {
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_rgb(18, 22, 28));

        for i in 0..=11 {
            let a = self.world_to_screen(rect, side, Point2D::new(f64::from(i), WORLD_MIN));
            let b = self.world_to_screen(rect, side, Point2D::new(f64::from(i), WORLD_MAX));
            painter.line_segment([a, b], Stroke::new(0.5_f32, Color32::from_gray(35)));
            let a = self.world_to_screen(rect, side, Point2D::new(WORLD_MIN, f64::from(i)));
            let b = self.world_to_screen(rect, side, Point2D::new(WORLD_MAX, f64::from(i)));
            painter.line_segment([a, b], Stroke::new(0.5_f32, Color32::from_gray(35)));
        }

        if self.show_observations {
            for (i, &(range, x, y)) in self.ranges.iter().enumerate() {
                let center = self.world_to_screen(rect, side, Point2D::new(x, y));
                let radius = range as f32 * side / (WORLD_MAX - WORLD_MIN) as f32;
                painter.circle_stroke(
                    center,
                    radius,
                    Stroke::new(
                        if i == self.anchor { 2.0_f32 } else { 0.75_f32 },
                        SENSOR.gamma_multiply(if i == self.anchor { 0.85 } else { 0.2 }),
                    ),
                );
            }
            if let Some(fix) = self.position_fix {
                let c = self.world_to_screen(rect, side, fix);
                for dir in [Vec2::new(5.0, 5.0), Vec2::new(5.0, -5.0)] {
                    painter.line_segment([c - dir, c + dir], Stroke::new(2.0_f32, SENSOR));
                }
            }
        }

        for (i, landmark) in self.landmarks.points.iter().enumerate() {
            let c = self.world_to_screen(rect, side, *landmark);
            painter.circle_stroke(
                c,
                6.0,
                Stroke::new(1.5_f32, Color32::from_rgb(200, 180, 80)),
            );
            painter.circle_filled(c, 2.5, Color32::from_rgb(220, 200, 90));
            painter.text(
                c + Vec2::new(10.0, -10.0),
                egui::Align2::LEFT_BOTTOM,
                (b'A' + i as u8) as char,
                egui::FontId::proportional(12.0),
                Color32::from_rgb(220, 200, 90),
            );
        }

        if self.trail_true.len() >= 2 {
            let true_pts: Vec<Pos2> = self
                .trail_true
                .iter()
                .map(|p| self.world_to_screen(rect, side, *p))
                .collect();
            painter.add(egui::Shape::line(
                true_pts,
                Stroke::new(1.5_f32, Color32::from_rgba_unmultiplied(120, 220, 140, 90)),
            ));
        }
        if self.trail_est.len() >= 2 {
            let est_pts: Vec<Pos2> = self
                .trail_est
                .iter()
                .map(|p| self.world_to_screen(rect, side, *p))
                .collect();
            let est_color = match self.filter {
                FilterKind::ParticleFilter => Color32::from_rgba_unmultiplied(100, 180, 255, 120),
                FilterKind::Ekf => Color32::from_rgba_unmultiplied(255, 160, 90, 120),
            };
            painter.add(egui::Shape::line(est_pts, Stroke::new(1.5_f32, est_color)));
        }

        if matches!(self.filter, FilterKind::ParticleFilter) {
            for particle in self.pf.get_particles() {
                let alpha =
                    (particle.w * self.pf.get_particles().len() as f64 * 8.0).clamp(0.05, 0.85);
                let c = self.world_to_screen(rect, side, Point2D::new(particle.x, particle.y));
                painter.circle_filled(
                    c,
                    1.8,
                    Color32::from_rgba_unmultiplied(100, 180, 255, (alpha * 255.0) as u8),
                );
            }
        }

        let est = self.estimate();

        if matches!(self.filter, FilterKind::Ekf) {
            self.draw_covariance_ellipse(
                &painter,
                rect,
                side,
                Point2D::new(est.x, est.y),
                *self.ekf.get_covariance_matrix(),
                Color32::from_rgb(255, 160, 90),
            );
        }

        let true_c = self.world_to_screen(
            rect,
            side,
            Point2D::new(self.true_state.x, self.true_state.y),
        );
        let est_c = self.world_to_screen(rect, side, Point2D::new(est.x, est.y));
        painter.line_segment(
            [true_c, est_c],
            Stroke::new(2.0_f32, Color32::WHITE.gamma_multiply(0.7)),
        );

        Self::draw_robot(&painter, true_c, self.true_state.yaw, TRUTH, 7.0);
        Self::draw_robot(
            &painter,
            est_c,
            est.yaw,
            match self.filter {
                FilterKind::ParticleFilter => Color32::from_rgb(80, 170, 255),
                FilterKind::Ekf => Color32::from_rgb(255, 140, 70),
            },
            6.0,
        );
        crate::ui_kit::overlay_text(
            ui,
            rect,
            match self.filter {
                FilterKind::ParticleFilter => "RANGE SENSOR / particle hypotheses",
                FilterKind::Ekf => "POSITION SENSOR / 95% model ellipse",
            },
        );
    }

    fn error_plot(&self, ui: &mut egui::Ui) {
        let (rect, _) =
            ui.allocate_exact_size(Vec2::new(ui.available_width(), 64.0), egui::Sense::hover());
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 4.0, Color32::from_rgb(10, 14, 20));
        let max = self.errors.iter().copied().fold(0.5, f64::max);
        let points = self
            .errors
            .iter()
            .enumerate()
            .map(|(i, e)| {
                egui::pos2(
                    rect.left() + i as f32 / (HISTORY - 1) as f32 * rect.width(),
                    rect.bottom() - 6.0 - (e / max) as f32 * (rect.height() - 22.0),
                )
            })
            .collect::<Vec<_>>();
        if points.len() > 1 {
            painter.add(egui::Shape::line(
                points.clone(),
                Stroke::new(1.8_f32, self.estimate_color()),
            ));
        }
        if let Some(last) = points.last() {
            painter.circle_filled(*last, 2.5, self.estimate_color());
        }
        painter.text(
            rect.left_top() + Vec2::splat(5.0),
            egui::Align2::LEFT_TOP,
            format!("{max:.2} m / last 12 s"),
            egui::FontId::proportional(11.0),
            Color32::GRAY,
        );
    }

    fn observation_readout(&mut self, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Sensor readout");
        if self.step_count == 0 {
            crate::ui_kit::hint(ui, "Play or step to take the first observation.");
            return;
        }
        match self.filter {
            FilterKind::ParticleFilter => {
                ui.horizontal_wrapped(|ui| {
                    for i in 0..5 {
                        let response = ui.selectable_label(
                            self.anchor == i,
                            ((b'A' + i as u8) as char).to_string(),
                        );
                        self.action(
                            ["anchor-a", "anchor-b", "anchor-c", "anchor-d", "anchor-e"][i],
                            &response,
                        );
                        if response.clicked() {
                            self.anchor = i;
                        }
                    }
                });
                if let Some(&(range, x, y)) = self.ranges.get(self.anchor) {
                    let actual = (self.true_state.x - x).hypot(self.true_state.y - y);
                    ui.label(format!(
                        "Range {}: {range:.2} m",
                        (b'A' + self.anchor as u8) as char
                    ));
                    ui.small(format!(
                        "True {actual:.2} m / residual {:+.2} m",
                        range - actual
                    ));
                }
                crate::ui_kit::hint(ui, "Each cyan circle is a measured distance from a landmark. Particles must explain all five ranges.");
            }
            FilterKind::Ekf => {
                if let Some(fix) = self.position_fix {
                    ui.label(format!("Position fix: {:.2}, {:.2} m", fix.x, fix.y));
                    ui.small(format!(
                        "Raw fix error {:.2} m",
                        (fix.x - self.true_state.x).hypot(fix.y - self.true_state.y)
                    ));
                }
                crate::ui_kit::hint(ui, "The cyan cross is a noisy position fix. The ellipse is the EKF model's 95% position contour.");
            }
        }
    }

    pub fn controls(&mut self, _ctx: &egui::Context, ui: &mut egui::Ui) {
        if self.belief.active {
            self.belief.controls(ui);
            return;
        }
        self.actions.clear();
        self.action_hits.clear();
        let response = ui.button("Open belief update lab");
        self.belief.widget("inspect", &response);
        if self.belief.button("inspect", &response) {
            self.belief.enter();
            return;
        }
        crate::ui_kit::section(ui, "Observation clock / 10 Hz");
        ui.horizontal_wrapped(|ui| {
            let response = ui.button(if self.playing { "Pause" } else { "Play" });
            self.action("play", &response);
            if response.clicked() {
                self.playing = !self.playing;
                self.clock.remainder = 0.0;
                self.last_time = None;
            }
            let response = ui.button("Step +0.1 s");
            self.action("step", &response);
            if response.clicked() {
                self.playing = false;
                self.single_step = true;
            }
            let response = ui.button("Reset");
            self.action("reset", &response);
            if response.clicked() {
                self.reset();
            }
        });
        crate::ui_kit::section(ui, "Filter + sensor");
        ui.horizontal_wrapped(|ui| {
            for kind in [FilterKind::ParticleFilter, FilterKind::Ekf] {
                let response = ui.selectable_label(self.filter == kind, kind.label());
                self.action(kind.slug(), &response);
                if response.clicked() && self.filter != kind {
                    self.filter = kind;
                    self.reset();
                }
            }
        });
        crate::ui_kit::section(ui, "Sensor noise");
        ui.add(egui::Slider::new(&mut self.noise_scale, 0.2..=3.0).logarithmic(true));
        ui.horizontal_wrapped(|ui| {
            for (name, label, noise) in [("quiet", "Quiet", 0.3), ("noisy", "Noisy", 3.0)] {
                let response = ui.button(label);
                self.action(name, &response);
                if response.clicked() {
                    self.noise_scale = noise;
                }
            }
        });
        ui.small(format!(
            "Next observation: sigma {:.3} m",
            self.sensor_sigma()
        ));
        let response = ui.checkbox(&mut self.show_observations, "Show observations");
        self.action("observations", &response);

        crate::ui_kit::section(ui, "Driving");
        let response = ui.checkbox(&mut self.auto_drive, "Auto tour");
        self.action("tour", &response);
        crate::ui_kit::hint(
            ui,
            "Hold the map or use arrow keys to drive. Pause freezes motion and sensing; Step takes one observation, even while stationary.",
        );
        crate::ui_kit::legend(
            ui,
            &[
                (Color32::from_rgb(90, 220, 120), "true pose"),
                (
                    match self.filter {
                        FilterKind::ParticleFilter => Color32::from_rgb(80, 170, 255),
                        FilterKind::Ekf => Color32::from_rgb(255, 140, 70),
                    },
                    "estimate",
                ),
                (Color32::from_rgb(220, 200, 90), "landmark"),
                (SENSOR, "observation"),
            ],
        );
        self.observation_readout(ui);
        crate::ui_kit::hint(
            ui,
            "Sharing copies these settings into a fresh run. Observations are newly randomized.",
        );
        crate::ui_kit::how_it_works(
            ui,
            "localization_help",
            "PF uses five landmark ranges; EKF uses a position fix. These are different \
             sensor models, so this is not an accuracy ranking. Change noise within a \
             filter and inspect the actual observations and estimate error. Switching \
             filters starts a fresh run. The EKF ellipse describes model uncertainty, \
             not a guarantee that the true robot stays inside it.",
        );
        #[cfg(target_arch = "wasm32")]
        {
            self.ui_revision += 1;
            self.action_pointer = ui.ctx().input(|i| i.pointer.hover_pos());
        }
        if let Some(rect) = self.scene_rect {
            self.publish_state(rect);
        }
    }

    pub fn scene(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        if self.belief.active {
            self.belief.scene(ui);
            return;
        }
        let (rect, side) = Self::world_rect(ui);
        self.scene_rect = Some(rect);
        let response = ui.allocate_rect(rect, egui::Sense::click_and_drag());

        let keyboard = self.read_control(ctx);
        let keys = keyboard.v.abs() > 1e-6 || keyboard.omega.abs() > 1e-6;
        let target = response
            .is_pointer_button_down_on()
            .then(|| response.interact_pointer_pos())
            .flatten()
            .map(|pos| Self::screen_to_world(rect, pos));
        if keys || target.is_some() {
            self.auto_drive = false;
        }
        let now = ctx.input(|input| input.time);
        let elapsed = self.last_time.replace(now).map_or(0.0, |last| now - last);
        let ticks = if std::mem::take(&mut self.single_step) {
            self.clock.remainder = 0.0;
            1
        } else {
            self.clock.ticks(elapsed, self.playing)
        };
        for _ in 0..ticks {
            let control = if keys {
                keyboard
            } else if let Some(target) = target {
                self.steer_toward(target)
            } else if self.auto_drive {
                self.tour_control()
            } else {
                ControlInput::new(0.0, 0.0)
            };
            self.step_simulation(control);
        }
        self.draw_scene(ui, rect, side);

        let est = self.estimate();
        let error = (self.true_state.x - est.x).hypot(self.true_state.y - est.y);
        ui.label(format!(
            "{} · {:.1} s · error {:.2} m · {}",
            self.filter.label(),
            self.step_count as f64 * DT,
            error,
            if self.playing { "live" } else { "paused" },
        ));
        ui.small("Position error / estimate to true robot");
        self.error_plot(ui);
        self.publish_state(rect);
        if self.playing {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));
        }
    }

    pub fn begin_frame(&mut self, ctx: &egui::Context, active: bool) {
        self.belief.begin_frame(ctx, active);
        #[cfg(target_arch = "wasm32")]
        if active && !self.belief.active {
            // Read before a ScrollArea consumes smooth_scroll_delta. These
            // diagnostics acknowledge input without changing scrolling.
            let (received, scrolling) = ctx.input(|i| {
                (
                    i.raw_scroll_delta != Vec2::ZERO,
                    i.raw_scroll_delta != Vec2::ZERO || i.smooth_scroll_delta != Vec2::ZERO,
                )
            });
            self.scroll_revision += u64::from(received);
            self.scrolling = scrolling;
        }
    }

    pub fn finish(&mut self, ctx: &egui::Context, active: bool) {
        self.belief.finish(ctx, active);
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish_state(&self, _rect: Rect) {}

    /// Derived read-only browser diagnostics; the controls above own all mutations.
    #[cfg(target_arch = "wasm32")]
    fn publish_state(&self, rect: Rect) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let actions = self
            .actions
            .iter()
            .map(|(key, p)| format!("\"{key}\":[{:.2},{:.2}]", p[0], p[1]))
            .collect::<Vec<_>>()
            .join(",");
        let hits = self
            .action_hits
            .iter()
            .map(|(key, hit)| format!("\"{key}\":{hit}"))
            .collect::<Vec<_>>()
            .join(",");
        let pointer = self
            .action_pointer
            .map_or_else(|| "null".to_owned(), |p| format!("[{:.2},{:.2}]", p.x, p.y));
        let est = self.estimate();
        let ranges = self
            .ranges
            .iter()
            .map(|(r, x, y)| format!("[{r:.8},{x:.8},{y:.8}]"))
            .collect::<Vec<_>>()
            .join(",");
        let errors = self
            .errors
            .iter()
            .map(|e| format!("{e:.8}"))
            .collect::<Vec<_>>()
            .join(",");
        let fix = self
            .position_fix
            .map_or_else(|| "null".to_owned(), |p| format!("[{:.8},{:.8}]", p.x, p.y));
        for (key, value) in [
            ("data-loc-filter", self.filter.slug().to_owned()),
            ("data-loc-steps", self.step_count.to_string()),
            ("data-loc-playing", u8::from(self.playing).to_string()),
            ("data-loc-tour", u8::from(self.auto_drive).to_string()),
            (
                "data-loc-observations",
                u8::from(self.show_observations).to_string(),
            ),
            ("data-loc-noise", self.noise_scale.to_string()),
            (
                "data-loc-truth",
                format!(
                    "[{:.8},{:.8},{:.8}]",
                    self.true_state.x, self.true_state.y, self.true_state.yaw
                ),
            ),
            (
                "data-loc-estimate",
                format!("[{:.8},{:.8},{:.8}]", est.x, est.y, est.yaw),
            ),
            ("data-loc-ranges", format!("[{ranges}]")),
            ("data-loc-fix", fix),
            ("data-loc-errors", format!("[{errors}]")),
            ("data-loc-share", self.share_query()),
            (
                "data-loc-rect",
                format!(
                    "[{:.2},{:.2},{:.2},{:.2}]",
                    rect.left(),
                    rect.top(),
                    rect.width(),
                    rect.height()
                ),
            ),
            ("data-loc-actions", format!("{{{actions}}}")),
            ("data-loc-hits", format!("{{{hits}}}")),
            ("data-loc-pointer", pointer),
            ("data-loc-scroll-revision", self.scroll_revision.to_string()),
            ("data-loc-scrolling", u8::from(self.scrolling).to_string()),
            ("data-loc-action-count", self.action_count.to_string()),
            ("data-loc-last-action", self.last_action.to_owned()),
            ("data-loc-ui-revision", self.ui_revision.to_string()),
            (
                "aria-label",
                format!(
                    "Localization, {} with {}, {} observations, {}",
                    self.filter.label(),
                    if self.filter == FilterKind::ParticleFilter {
                        "landmark ranges"
                    } else {
                        "position fixes"
                    },
                    self.step_count,
                    if self.playing { "playing" } else { "paused" }
                ),
            ),
        ] {
            let _ = canvas.set_attribute(key, &value);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{FilterKind, LocalizationDemo, SensorClock};

    #[test]
    fn share_query_round_trips_configuration() {
        let demo = LocalizationDemo {
            filter: FilterKind::Ekf,
            noise_scale: 2.4,
            playing: false,
            auto_drive: false,
            show_observations: false,
            anchor: 3,
            ..LocalizationDemo::default()
        };
        let query = demo.share_query();
        let mut restored = LocalizationDemo::default();
        restored.apply_share_query(&query);
        assert_eq!(restored.filter, FilterKind::Ekf);
        assert!((restored.noise_scale - 2.4).abs() < f32::EPSILON);
        assert!(!restored.playing);
        assert!(!restored.auto_drive);
        assert!(!restored.show_observations);
        assert_eq!(restored.anchor, 3);
    }

    #[test]
    fn auto_tour_drives_around_the_landmarks_and_stays_tracked() {
        let mut demo = LocalizationDemo::default();
        let mut seen = [false; 4];
        for _ in 0..1500 {
            let control = demo.tour_control();
            demo.step_simulation(control);
            seen[demo.tour_index] = true;
            assert!(
                (super::WORLD_MIN..=super::WORLD_MAX).contains(&demo.true_state.x)
                    && (super::WORLD_MIN..=super::WORLD_MAX).contains(&demo.true_state.y),
                "left the world"
            );
        }
        assert!(seen.iter().all(|s| *s), "tour corners visited: {seen:?}");
        let est = demo.pf.state_2d();
        let error = (demo.true_state.x - est.x).hypot(demo.true_state.y - est.y);
        assert!(error < 1.0, "particle filter lost the robot: {error:.2} m");
    }

    #[test]
    fn invalid_shared_configuration_is_ignored() {
        let mut demo = LocalizationDemo::default();
        demo.apply_share_query("tab=localization&filter=bad&noise=99&loc_anchor=5&loc_playing=bad");
        assert_eq!(demo.filter, FilterKind::ParticleFilter);
        assert_eq!(demo.noise_scale, 1.0);
        assert_eq!(demo.anchor, 0);
        assert!(demo.playing);
    }

    #[test]
    fn observation_clock_is_independent_of_paint_rate() {
        for (paints, elapsed) in [(1000, 0.01), (250, 0.04), (25, 0.4)] {
            let mut clock = SensorClock::default();
            assert_eq!(
                (0..paints)
                    .map(|_| clock.ticks(elapsed, true))
                    .sum::<usize>(),
                100
            );
        }
    }

    #[test]
    fn shared_settings_restart_an_existing_trial_only_for_localization() {
        let mut demo = LocalizationDemo::default();
        demo.step_simulation(rust_robotics_core::ControlInput::new(1.0, 0.0));
        demo.apply_share_query("tab=grid&noise=3");
        assert_eq!(demo.step_count, 1);
        assert_eq!(demo.noise_scale, 1.0);
        demo.apply_share_query("tab=localization&filter=ekf&noise=3&loc_playing=0");
        assert_eq!(demo.step_count, 0);
        assert_eq!(demo.filter, FilterKind::Ekf);
        assert_eq!(demo.noise_scale, 3.0);
        assert!(!demo.playing);
        assert!(demo.ranges.is_empty() && demo.errors.is_empty());
        assert_eq!((demo.true_state.x, demo.true_state.y), (5.0, 5.0));
    }

    #[test]
    fn pause_and_background_gaps_do_not_accumulate_work() {
        let mut clock = SensorClock::default();
        assert_eq!(clock.ticks(0.09, true), 0);
        assert_eq!(clock.ticks(0.09, false), 0);
        assert_eq!(clock.ticks(0.02, true), 0);
        for gap in [1.0, 30.0, f64::NAN, f64::INFINITY, -0.1] {
            assert_eq!(clock.ticks(gap, true), 0);
            assert_eq!(clock.ticks(0.1, true), 1);
        }
    }

    #[test]
    fn stationary_robot_still_observes_and_retains_bounded_error_history() {
        for filter in [FilterKind::ParticleFilter, FilterKind::Ekf] {
            let mut demo = LocalizationDemo {
                filter,
                ..LocalizationDemo::default()
            };
            for _ in 0..135 {
                demo.step_simulation(rust_robotics_core::ControlInput::new(0.0, 0.0));
                let est = demo.estimate();
                let actual_error = (demo.true_state.x - est.x).hypot(demo.true_state.y - est.y);
                assert_eq!(demo.errors.last().copied(), Some(actual_error));
            }
            assert_eq!(demo.step_count, 135);
            assert_eq!(demo.errors.len(), super::HISTORY);
            assert_eq!((demo.true_state.x, demo.true_state.y), (5.0, 5.0));
            if filter == FilterKind::ParticleFilter {
                assert_eq!(demo.ranges.len(), 5);
                assert!(demo.position_fix.is_none());
                for ((range, x, y), landmark) in demo.ranges.iter().zip(&demo.landmarks.points) {
                    assert!(range.is_finite() && *range >= 0.0);
                    assert_eq!((*x, *y), (landmark.x, landmark.y));
                }
            } else {
                assert!(demo.ranges.is_empty());
                assert!(demo.position_fix.is_some());
            }
        }
    }

    #[test]
    fn filter_switch_reset_clears_observations_without_losing_settings() {
        let mut demo = LocalizationDemo {
            noise_scale: 3.0,
            playing: false,
            auto_drive: false,
            ..LocalizationDemo::default()
        };
        demo.step_simulation(rust_robotics_core::ControlInput::new(1.0, 0.1));
        assert!(!demo.ranges.is_empty());
        demo.filter = FilterKind::Ekf;
        demo.reset();
        assert_eq!(demo.filter, FilterKind::Ekf);
        assert_eq!(demo.noise_scale, 3.0);
        assert!(!demo.playing && !demo.auto_drive);
        assert_eq!(demo.step_count, 0);
        assert!(demo.ranges.is_empty() && demo.position_fix.is_none() && demo.errors.is_empty());
        assert!(demo.trail_est.is_empty() && demo.trail_true.is_empty());
        assert_eq!((demo.true_state.x, demo.true_state.y), (5.0, 5.0));
        demo.step_simulation(rust_robotics_core::ControlInput::new(0.0, 0.0));
        assert!(demo.position_fix.is_some());
        assert!(demo.ranges.is_empty());
    }
}
