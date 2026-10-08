//! Interactive localization demo: Particle Filter and EKF. Drive with the
//! arrow keys, by holding the pointer on the map, or let the robot tour the
//! landmarks on its own.

use std::f64::consts::PI;

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
        }
    }
}

impl LocalizationDemo {
    pub fn apply_share_query(&mut self, query: &str) {
        if let Some(filter) = crate::share::value(query, "filter").and_then(FilterKind::from_slug) {
            self.filter = filter;
        }
        if let Some(noise) = crate::share::bounded_f32(query, "noise", 0.2, 3.0) {
            self.noise_scale = noise;
            self.apply_noise_scale();
        }
    }

    pub fn share_query(&self) -> String {
        format!(
            "tab=localization&filter={}&noise={:.2}",
            self.filter.slug(),
            self.noise_scale
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
        let sigma = 0.15 * f64::from(self.noise_scale);
        let range = (0.08 + sigma).max(0.01);
        let _ = self.pf.set_range_noise(range);

        let r = sigma.powi(2).max(1e-4);
        let _ = self
            .ekf
            .try_set_measurement_noise(Matrix2::new(r, 0.0, 0.0, r));
    }

    fn step_simulation(&mut self, control: ControlInput) {
        if control.v.abs() < 1e-6 && control.omega.abs() < 1e-6 {
            return;
        }

        self.apply_noise_scale();
        let meas_sigma = 0.12 * f64::from(self.noise_scale);

        Self::propagate_true(&mut self.true_state, control);

        let estimate = match self.filter {
            FilterKind::ParticleFilter => {
                let obs = self.pf_measurements(meas_sigma);
                self.pf
                    .try_step_state(control, &obs)
                    .unwrap_or_else(|_| self.pf.state_2d())
            }
            FilterKind::Ekf => {
                let z = self.noisy_position(meas_sigma);
                self.ekf
                    .estimate_state(z, control, DT)
                    .unwrap_or_else(|_| self.ekf.state_2d())
            }
        };

        let true_point = Point2D::new(self.true_state.x, self.true_state.y);
        let est_point = Point2D::new(estimate.x, estimate.y);
        self.push_trail(true_point, est_point);

        self.step_count += 1;
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
        let rect = crate::ui_kit::fit_rect(ui, 1.0, 34.0);
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
        let k = 2.0_f64.sqrt();

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

        for landmark in &self.landmarks.points {
            let c = self.world_to_screen(rect, side, *landmark);
            painter.circle_stroke(
                c,
                6.0,
                Stroke::new(1.5_f32, Color32::from_rgb(200, 180, 80)),
            );
            painter.circle_filled(c, 2.5, Color32::from_rgb(220, 200, 90));
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

        let est = match self.filter {
            FilterKind::ParticleFilter => self.pf.state_2d(),
            FilterKind::Ekf => self.ekf.state_2d(),
        };

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

        Self::draw_robot(
            &painter,
            true_c,
            self.true_state.yaw,
            Color32::from_rgb(90, 220, 120),
            7.0,
        );
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
    }

    pub fn controls(&mut self, _ctx: &egui::Context, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Filter");
        ui.horizontal_wrapped(|ui| {
            for kind in [FilterKind::ParticleFilter, FilterKind::Ekf] {
                if ui
                    .selectable_label(self.filter == kind, kind.label())
                    .clicked()
                {
                    self.filter = kind;
                    self.trail_true.clear();
                    self.trail_est.clear();
                }
            }
        });
        crate::ui_kit::section(ui, "Sensor noise");
        ui.add(egui::Slider::new(&mut self.noise_scale, 0.2..=3.0).logarithmic(true));

        crate::ui_kit::section(ui, "Driving");
        ui.horizontal_wrapped(|ui| {
            ui.checkbox(&mut self.auto_drive, "Auto tour");
            if ui.button("Reset").clicked() {
                self.reset();
            }
        });
        crate::ui_kit::hint(
            ui,
            "Hold the pointer on the map to drive there, or use the arrow keys.",
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
            ],
        );
        crate::ui_kit::how_it_works(
            ui,
            "localization_help",
            "The robot measures noisy ranges to the five landmarks. The particle filter \
             keeps a cloud of pose hypotheses and resamples them by how well they explain \
             the ranges; the EKF keeps one Gaussian (the ellipse) and corrects it with a \
             noisy position fix. Raise the noise to see which one copes better.",
        );
    }

    pub fn scene(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        let (rect, side) = Self::world_rect(ui);
        let response = ui.allocate_rect(rect, egui::Sense::click_and_drag());

        let mut control = self.read_control(ctx);
        let keys = control.v.abs() > 1e-6 || control.omega.abs() > 1e-6;
        if !keys {
            if let Some(pos) = response
                .is_pointer_button_down_on()
                .then(|| response.interact_pointer_pos())
                .flatten()
            {
                control = self.steer_toward(Self::screen_to_world(rect, pos));
            } else if self.auto_drive {
                control = self.tour_control();
            }
        } else {
            self.auto_drive = false;
        }
        if response.is_pointer_button_down_on() {
            self.auto_drive = false;
        }
        self.step_simulation(control);
        self.draw_scene(ui, rect, side);

        let est = match self.filter {
            FilterKind::ParticleFilter => self.pf.state_2d(),
            FilterKind::Ekf => self.ekf.state_2d(),
        };
        let error = (self.true_state.x - est.x).hypot(self.true_state.y - est.y);
        ui.label(format!(
            "{}  ·  error {:.2} m  ·  {} steps",
            self.filter.label(),
            error,
            self.step_count,
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::{FilterKind, LocalizationDemo};

    #[test]
    fn share_query_round_trips_configuration() {
        let demo = LocalizationDemo {
            filter: FilterKind::Ekf,
            noise_scale: 2.4,
            ..LocalizationDemo::default()
        };
        let query = demo.share_query();
        let mut restored = LocalizationDemo::default();
        restored.apply_share_query(&query);
        assert_eq!(restored.filter, FilterKind::Ekf);
        assert!((restored.noise_scale - 2.4).abs() < f32::EPSILON);
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
        demo.apply_share_query("filter=bad&noise=99");
        assert_eq!(demo.filter, FilterKind::ParticleFilter);
        assert_eq!(demo.noise_scale, 1.0);
    }
}
