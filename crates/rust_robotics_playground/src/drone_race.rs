//! Interactive 3-D quadrotor race, with bounded incremental replay generation.

mod flight;
mod renderer;

use egui::{Color32, FontId, Rect, RichText, Stroke, Vec2};
use flight::{Course, Flight, Status, DT, MAX_STEPS};
use renderer::{CameraMode, ViewOptions, CYAN, GOLD};
use web_time::Instant;

pub struct DroneRaceDemo {
    flight: Flight,
    cursor: usize,
    seek: Option<usize>,
    playing: bool,
    phase: f64,
    last_time: Option<f64>,
    camera: CameraMode,
    trail: bool,
    prediction: bool,
    slow: bool,
    cinema: bool,
    last_plan_ms: f64,
    error: Option<String>,
}

impl Default for DroneRaceDemo {
    fn default() -> Self {
        Self {
            flight: Flight::new(Course::Neon),
            cursor: 0,
            seek: None,
            playing: true,
            phase: 0.0,
            last_time: None,
            camera: CameraMode::Chase,
            trail: true,
            prediction: true,
            slow: false,
            cinema: false,
            last_plan_ms: 0.0,
            error: None,
        }
    }
}

impl DroneRaceDemo {
    pub fn apply_share_query(&mut self, query: &str) {
        if crate::share::value(query, "tab") != Some("drone") {
            return;
        }
        if let Some(course) = crate::share::value(query, "course").and_then(Course::from_slug) {
            if course != self.flight.course {
                self.flight = Flight::new(course);
                self.error = None;
            }
        }
        if let Some(camera) = crate::share::value(query, "camera").and_then(CameraMode::from_slug) {
            self.camera = camera;
        }
        self.trail = crate::share::boolean(query, "trail").unwrap_or(true);
        self.prediction = crate::share::boolean(query, "prediction").unwrap_or(true);
        self.slow = crate::share::boolean(query, "slow").unwrap_or(false);
        self.cinema = crate::share::boolean(query, "cinema").unwrap_or(false);
        self.playing = crate::share::boolean(query, "playing").unwrap_or(true);
        let frame = crate::share::bounded_usize(query, "frame", MAX_STEPS).unwrap_or(0);
        self.seek_to(frame);
    }

    pub fn share_query(&self) -> String {
        format!(
            "tab=drone&course={}&camera={}&frame={}&playing={}&trail={}&prediction={}&slow={}&cinema={}",
            self.flight.course.slug(),
            self.camera.slug(),
            self.seek.unwrap_or(self.cursor),
            u8::from(self.playing),
            u8::from(self.trail),
            u8::from(self.prediction),
            u8::from(self.slow),
            u8::from(self.cinema)
        )
    }

    pub fn cinema(&self) -> bool {
        self.cinema
    }

    fn seek_to(&mut self, target: usize) {
        let target = target.min(MAX_STEPS);
        if target <= self.flight.last_index() || self.flight.done() {
            self.cursor = target.min(self.flight.last_index());
            self.seek = None;
        } else {
            self.seek = Some(target);
        }
        self.phase = 0.0;
        self.last_time = None;
    }

    fn rewind(&mut self) {
        self.seek_to(0);
        self.playing = true;
    }

    fn step_flight(&mut self) -> bool {
        let begin = Instant::now();
        let result = self.flight.step();
        self.last_plan_ms = begin.elapsed().as_secs_f64() * 1000.0;
        match result {
            Ok(stepped) => stepped,
            Err(error) => {
                self.error = Some(error.to_string());
                self.playing = false;
                self.seek = None;
                false
            }
        }
    }

    fn advance(&mut self, now: f64) {
        let elapsed = self.last_time.map_or(0.0, |last| {
            let delta = now - last;
            // Changing tabs or restoring a background browser never creates
            // an unbounded catch-up loop.
            if (0.0..0.25).contains(&delta) {
                delta.min(0.1)
            } else {
                0.0
            }
        });
        self.last_time = Some(now);
        if self.error.is_some() {
            return;
        }
        if let Some(target) = self.seek {
            let begin = Instant::now();
            // At most two controller calls per paint. A long share-link seek
            // yields to the browser between batches, even on slow devices.
            for _ in 0..2 {
                if self.flight.last_index() >= target || self.flight.done() {
                    break;
                }
                if !self.step_flight() || begin.elapsed().as_secs_f64() >= 0.008 {
                    break;
                }
            }
            if self.flight.last_index() >= target || self.flight.done() {
                self.cursor = target.min(self.flight.last_index());
                self.seek = None;
            }
            self.phase = 0.0;
            return;
        }
        if !self.playing {
            return;
        }
        if self.flight.frames[self.cursor].status != Status::Flying {
            self.playing = false;
            return;
        }
        self.phase = (self.phase + elapsed * if self.slow { 0.5 } else { 1.0 }).min(2.0 * DT);
        for _ in 0..2 {
            if self.phase < DT {
                break;
            }
            if self.cursor == self.flight.last_index() && !self.step_flight() {
                break;
            }
            if self.cursor < self.flight.last_index() {
                self.cursor += 1;
                self.phase -= DT;
            }
            if self.flight.frames[self.cursor].status != Status::Flying {
                self.playing = false;
                self.phase = 0.0;
                crate::engagement::track("drone_race_completed");
                break;
            }
        }
    }

    pub fn controls(&mut self, _ctx: &egui::Context, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "THE CIRCUIT");
        let mut course = self.flight.course;
        for candidate in Course::ALL {
            ui.selectable_value(&mut course, candidate, candidate.title());
        }
        if course != self.flight.course {
            self.flight = Flight::new(course);
            self.error = None;
            self.rewind();
        }
        ui.add_space(8.0);
        ui.label(RichText::new("Five gates. One autonomous pilot.").color(CYAN));
        crate::ui_kit::hint(
            ui,
            "Watch the quadrotor lean into turns and climb through the course.",
        );

        crate::ui_kit::section(ui, "CAMERA");
        ui.horizontal(|ui| {
            for camera in CameraMode::ALL {
                ui.selectable_value(&mut self.camera, camera, camera.title());
            }
        });
        crate::ui_kit::hint(
            ui,
            match self.camera {
                CameraMode::Chase => "Follow the drone from behind.",
                CameraMode::Pilot => "See through the drone's eyes, including its pitch and roll.",
                CameraMode::Course => "Watch the entire circuit from the grandstand.",
            },
        );

        if ui.button("Cinema view").clicked() {
            self.cinema = true;
        }

        crate::ui_kit::section(ui, "PLAYBACK");
        ui.horizontal(|ui| {
            if ui
                .button(if self.playing { "Pause" } else { "Play" })
                .clicked()
            {
                if self.flight.frames[self.cursor].status != Status::Flying {
                    self.rewind();
                } else {
                    self.playing = !self.playing;
                    self.last_time = None;
                    self.phase = 0.0;
                }
            }
            if ui.button("Replay").clicked() {
                self.rewind();
            }
            if ui.button("Step").clicked() {
                self.playing = false;
                self.seek_to(self.cursor + 1);
            }
        });
        ui.checkbox(&mut self.slow, "Slow motion");
        let mut frame = self.seek.unwrap_or(self.cursor);
        // Full-range seeking builds only the needed prefix, incrementally.
        let end = if self.flight.done() {
            self.flight.last_index()
        } else {
            MAX_STEPS
        };
        if ui
            .add(egui::Slider::new(&mut frame, 0..=end).text("Frame"))
            .changed()
        {
            self.playing = false;
            self.seek_to(frame);
        }
        if let Some(target) = self.seek {
            ui.label(format!(
                "Preparing replay · {} / {target}",
                self.flight.last_index()
            ));
        }
        crate::ui_kit::hint(ui, "Space: pause · R: replay · 1 / 2 / 3: camera");

        crate::ui_kit::section(ui, "FLIGHT LINES");
        ui.checkbox(&mut self.trail, "Flight trail");
        ui.checkbox(&mut self.prediction, "Predicted flight");
        ui.horizontal(|ui| {
            for (color, label) in [(CYAN, "Flown"), (GOLD, "Predicted")] {
                let (line, _) = ui.allocate_exact_size(Vec2::new(18.0, 10.0), egui::Sense::hover());
                ui.painter().line_segment(
                    [line.left_center(), line.right_center()],
                    Stroke::new(2.0_f32, color),
                );
                ui.colored_label(color, label);
            }
        });
        ui.add_space(12.0);
        ui.collapsing("Inside the pilot", |ui| {
            ui.label("MPPI samples 900 possible command sequences, each 1.4 seconds long, before every flight update.");
            ui.label("The quadrotor accelerates along its own thrust axis: it must tilt to turn. The dotted line is the controller's remaining predicted horizon.");
            ui.label(format!("Latest planning call: {:.1} ms", self.last_plan_ms));
            ui.label("Fixed seed: 7 · flight step: 0.05 s");
            ui.label("A shared link rebuilds the same commands and flight history; replay speed and camera do not affect physics.");
        });
        if let Some(error) = &self.error {
            ui.colored_label(Color32::LIGHT_RED, error);
        }
    }

    pub fn scene(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        if !ctx.wants_keyboard_input() {
            ctx.input(|input| {
                if input.key_pressed(egui::Key::Space) {
                    self.playing = !self.playing;
                    self.last_time = None;
                    self.phase = 0.0;
                }
                if input.key_pressed(egui::Key::R) {
                    self.rewind();
                }
                if input.key_pressed(egui::Key::Escape) {
                    self.cinema = false;
                }
                for (key, camera) in [
                    (egui::Key::Num1, CameraMode::Chase),
                    (egui::Key::Num2, CameraMode::Pilot),
                    (egui::Key::Num3, CameraMode::Course),
                ] {
                    if input.key_pressed(key) {
                        self.camera = camera;
                    }
                }
            });
        }
        self.advance(ctx.input(|input| input.time));
        let available = ui.available_size();
        let height = if !self.cinema && ctx.screen_rect().width() < 700.0 {
            (available.x * 0.95).clamp(300.0, 520.0)
        } else {
            available.y.max(300.0)
        };
        let (rect, _) = ui.allocate_exact_size(
            Vec2::new(available.x.max(120.0), height),
            egui::Sense::click(),
        );
        let painter = ui.painter_at(rect);
        let current = &self.flight.frames[self.cursor];
        let (draw_frame, state, visual_time) =
            if self.playing && self.cursor > 0 && self.seek.is_none() {
                let previous = self.cursor - 1;
                let alpha = self.phase / DT;
                (
                    previous,
                    flight::interpolate(self.flight.frames[previous].state, current.state, alpha),
                    (previous as f64 + alpha) * DT,
                )
            } else {
                (self.cursor, current.state, self.cursor as f64 * DT)
            };
        renderer::render(
            &painter,
            rect,
            &self.flight,
            draw_frame,
            state,
            visual_time,
            &ViewOptions {
                camera: self.camera,
                trail: self.trail,
                prediction: self.prediction,
            },
        );
        self.hud(&painter, rect, state.speed(), visual_time);
        if self.cinema {
            let top = if rect.width() >= 600.0 {
                rect.right_top() + Vec2::new(-258.0, 50.0)
            } else {
                rect.left_top() + Vec2::new(16.0, 108.0)
            };
            for (i, camera) in CameraMode::ALL.into_iter().enumerate() {
                let button = Rect::from_min_size(
                    top + Vec2::new(i as f32 * 74.0, 0.0),
                    Vec2::new(68.0, 28.0),
                );
                if ui
                    .put(
                        button,
                        egui::Button::new(camera.title()).selected(self.camera == camera),
                    )
                    .clicked()
                {
                    self.camera = camera;
                }
            }
            let exit = Rect::from_min_size(top + Vec2::new(0.0, 36.0), Vec2::new(110.0, 24.0));
            if ui.put(exit, egui::Button::new("Show controls")).clicked() {
                self.cinema = false;
            }
        }
        if self.seek.is_some() {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "PREPARING REPLAY",
                FontId::monospace(16.0),
                CYAN,
            );
        } else if current.status != Status::Flying {
            let center = rect.center();
            let card =
                Rect::from_center_size(center, Vec2::new(rect.width().min(350.0) - 20.0, 144.0));
            painter.rect_filled(card, 12.0, Color32::from_rgba_unmultiplied(10, 18, 34, 240));
            painter.rect_stroke(
                card,
                12.0,
                Stroke::new(1.0_f32, CYAN.gamma_multiply(0.4)),
                egui::StrokeKind::Inside,
            );
            let title = match current.status {
                Status::Finished => "COURSE COMPLETE",
                Status::GroundContact => "GROUND CONTACT",
                Status::TimeLimit => "TIME LIMIT",
                Status::Flying => "",
            };
            painter.text(
                center + Vec2::new(0.0, -43.0),
                egui::Align2::CENTER_CENTER,
                title,
                FontId::proportional(22.0),
                CYAN,
            );
            painter.text(
                center + Vec2::new(0.0, -8.0),
                egui::Align2::CENTER_CENTER,
                format!(
                    "{} / {} gates  ·  {:.2} s",
                    current.passed,
                    self.flight.lap.gate_count(),
                    self.cursor as f64 * DT
                ),
                FontId::monospace(14.0),
                Color32::WHITE,
            );
            if ui
                .put(
                    Rect::from_center_size(center + Vec2::new(0.0, 36.0), Vec2::new(128.0, 30.0)),
                    egui::Button::new("Fly again"),
                )
                .clicked()
            {
                self.rewind();
            }
        }
        self.publish_status();
        if self.playing || self.seek.is_some() {
            ctx.request_repaint();
        }
    }

    fn hud(&self, painter: &egui::Painter, rect: Rect, speed: f64, time: f64) {
        let pad = if rect.width() < 500.0 { 16.0 } else { 26.0 };
        let left = rect.left() + pad;
        let top = rect.top() + pad;
        let small = Color32::from_rgb(158, 180, 201);
        painter.text(
            egui::pos2(left, top),
            egui::Align2::LEFT_TOP,
            "RUST / FLIGHT LAB",
            FontId::monospace(10.0),
            CYAN,
        );
        painter.text(
            egui::pos2(left, top + 20.0),
            egui::Align2::LEFT_TOP,
            self.flight.course.title().to_uppercase(),
            FontId::proportional(if rect.width() < 500.0 { 22.0 } else { 30.0 }),
            Color32::WHITE,
        );
        let mode = if self.seek.is_some() {
            "BUILDING REPLAY"
        } else if !self.playing {
            "PAUSED"
        } else {
            "AUTONOMOUS FLIGHT"
        };
        painter.text(
            egui::pos2(left, top + 59.0),
            egui::Align2::LEFT_TOP,
            mode,
            FontId::monospace(10.0),
            small,
        );
        if rect.width() >= 500.0 {
            painter.text(
                rect.right_top() + Vec2::new(-pad, pad),
                egui::Align2::RIGHT_TOP,
                format!("{} CAM", self.camera.title().to_uppercase()),
                FontId::monospace(11.0),
                small,
            );
        }
        let bottom = rect.bottom() - pad;
        let width = (rect.width() - pad * 2.0) / 3.0;
        let current = &self.flight.frames[self.cursor];
        for (i, (label, value, suffix)) in [
            (
                "GATES",
                format!(
                    "{:02} / {:02}",
                    current.passed,
                    self.flight.lap.gate_count()
                ),
                "",
            ),
            ("SPEED", format!("{speed:.1}"), "m/s"),
            ("FLIGHT TIME", format!("{time:05.2}"), "s"),
        ]
        .into_iter()
        .enumerate()
        {
            let x = left + width * i as f32;
            painter.text(
                egui::pos2(x, bottom - 47.0),
                egui::Align2::LEFT_TOP,
                label,
                FontId::monospace(10.0),
                small,
            );
            painter.text(
                egui::pos2(x, bottom - 28.0),
                egui::Align2::LEFT_TOP,
                value,
                FontId::monospace(if rect.width() < 500.0 { 21.0 } else { 29.0 }),
                Color32::WHITE,
            );
            if !suffix.is_empty() {
                painter.text(
                    egui::pos2(x + width - 20.0, bottom - 7.0),
                    egui::Align2::RIGHT_BOTTOM,
                    suffix,
                    FontId::monospace(11.0),
                    small,
                );
            }
        }
        let track_y = bottom - 68.0;
        let count = self.flight.lap.gate_count();
        for i in 0..count {
            let gap = 6.0;
            let segment = (rect.width() - pad * 2.0 - gap * (count - 1) as f32) / count as f32;
            let x = left + i as f32 * (segment + gap);
            let color = if i < current.passed {
                CYAN
            } else if i == current.passed {
                GOLD
            } else {
                Color32::from_rgb(50, 67, 84)
            };
            painter.rect_filled(
                Rect::from_min_size(egui::pos2(x, track_y), Vec2::new(segment, 3.0)),
                2.0,
                color,
            );
        }
    }

    #[cfg(target_arch = "wasm32")]
    fn publish_status(&self) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let frame = &self.flight.frames[self.cursor];
        // Accessible scene summary also permits functional browser checks of
        // real flight progress, paused replays and camera changes.
        for (key, value) in [
            ("data-drone-frame", self.cursor.to_string()),
            ("data-drone-generated", self.flight.last_index().to_string()),
            ("data-drone-gates", frame.passed.to_string()),
            (
                "data-drone-status",
                self.error
                    .as_ref()
                    .map_or(frame.status.slug(), |_| "error")
                    .to_string(),
            ),
            ("data-drone-camera", self.camera.slug().to_string()),
            ("data-drone-course", self.flight.course.slug().to_string()),
            (
                "data-drone-position",
                frame
                    .state
                    .position
                    .iter()
                    .map(|v| format!("{v:.6}"))
                    .collect::<Vec<_>>()
                    .join(","),
            ),
            (
                "data-drone-planning-ms",
                format!("{:.2}", self.last_plan_ms),
            ),
            ("data-drone-playing", u8::from(self.playing).to_string()),
            ("data-drone-cinema", u8::from(self.cinema).to_string()),
            (
                "data-drone-seeking",
                u8::from(self.seek.is_some()).to_string(),
            ),
            (
                "aria-label",
                format!(
                    "Drone race: {} of {} gates, {:.2} seconds, {} camera",
                    frame.passed,
                    self.flight.lap.gate_count(),
                    self.cursor as f64 * DT,
                    self.camera.title()
                ),
            ),
        ] {
            let _ = canvas.set_attribute(key, &value);
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish_status(&self) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn share_links_restore_paused_flight_and_all_view_settings() {
        let mut original = DroneRaceDemo::default();
        for _ in 0..12 {
            original.flight.step().unwrap();
        }
        original.cursor = 12;
        original.playing = false;
        original.camera = CameraMode::Pilot;
        original.slow = true;
        original.cinema = true;
        original.trail = false;
        let query = original.share_query();
        let mut restored = DroneRaceDemo::default();
        restored.apply_share_query(&query);
        for i in 0..20 {
            restored.advance(i as f64 * 0.01);
        }
        assert!(restored.seek.is_none());
        assert_eq!(restored.share_query(), query);
        assert_eq!(restored.flight.frames, original.flight.frames);
        let count = restored.flight.frames.len();
        restored.advance(1000.0);
        assert_eq!(restored.cursor, 12);
        assert_eq!(restored.flight.frames.len(), count);
    }

    #[test]
    fn bounded_seek_yields_and_replay_uses_cached_history() {
        let mut demo = DroneRaceDemo::default();
        demo.apply_share_query("tab=drone&frame=600&playing=0");
        demo.advance(1.0);
        assert!(demo.flight.last_index() <= 2);
        assert!(demo.seek.is_some());
        demo.seek_to(0);
        demo.playing = true;
        let cached = demo.flight.frames.clone();
        demo.advance(2.0);
        demo.advance(200.0);
        assert_eq!(demo.cursor, 0, "background time must not advance the race");
        assert_eq!(demo.flight.frames, cached);
    }

    #[test]
    fn unrelated_and_invalid_links_do_not_change_or_allocate_flight_history() {
        let mut demo = DroneRaceDemo::default();
        let before = demo.share_query();
        demo.apply_share_query("tab=arena&course=climb&frame=600");
        assert_eq!(demo.share_query(), before);
        demo.apply_share_query("tab=drone&frame=999999&camera=invalid&course=unknown");
        assert_eq!(demo.cursor, 0);
        assert!(demo.seek.is_none());
        assert_eq!(demo.flight.frames.len(), 1);
        assert_eq!(demo.camera, CameraMode::Chase);
        assert_eq!(demo.flight.course, Course::Neon);
    }
}
