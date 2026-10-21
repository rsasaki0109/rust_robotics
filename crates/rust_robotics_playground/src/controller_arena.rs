//! Interactive replay front end for the deterministic Controller Arena engine.

mod dwa;
mod safety;

use std::collections::BTreeMap;

use egui::{Color32, Pos2, Rect, Stroke, Vec2};
use rust_robotics_control::{
    metrics_from_samples, run_controller_arena, ArenaControllerKind, ArenaMetrics, ArenaPreset,
    ArenaRun, ArenaSample, ArenaScenario,
};
use rust_robotics_core::{Path2D, Point2D, State2D};

const DEFAULT_SPEED: f64 = 3.0;
const DEFAULT_RESPONSE: f64 = 0.85;
const MIN_SPEED: f64 = 1.0;
const MAX_SPEED: f64 = 6.0;
const MIN_RESPONSE: f64 = 0.4;
const MAX_RESPONSE: f64 = 1.0;
/// Drawing canvas `[0, DRAW_W] × [0, DRAW_H]` \[m\].
const DRAW_W: f64 = 60.0;
const DRAW_H: f64 = 30.0;
/// Spacing of a drawn course's points \[m\] (like the presets').
const COURSE_SPACING: f64 = 0.5;
/// Spacing of the points stored in share links \[m\].
const LINK_SPACING: f64 = 1.5;
/// Shorter strokes are ignored \[m\].
const MIN_COURSE_LENGTH: f64 = 8.0;
const MAX_COURSE_LENGTH: f64 = 500.0;

pub struct ControllerArenaDemo {
    safety: safety::SafetyLab,
    dwa: dwa::DwaLab,
    preset: ArenaPreset,
    target_speed: f64,
    turn_rate_response_gain: f64,
    scenario: rust_robotics_control::ArenaScenario,
    runs: Vec<ArenaRun>,
    frame_idx: usize,
    playing: bool,
    dirty: bool,
    error: Option<String>,
    last_advance: Option<f64>,
    replay_phase: f64,
    replay_rate: f64,
    focus: ArenaControllerKind,
    actions: BTreeMap<&'static str, [f32; 2]>,
    action_hits: BTreeMap<&'static str, bool>,
    scene_rect: Option<Rect>,
    plot_rects: [Option<Rect>; 2],
    #[cfg(target_arch = "wasm32")]
    action_count: u64,
    #[cfg(target_arch = "wasm32")]
    last_action: &'static str,
    #[cfg(target_arch = "wasm32")]
    ui_revision: u64,
    /// Key points of the course the user drew, every `LINK_SPACING` m and
    /// rounded to 0.1 m: exactly what a share link stores (kept while a
    /// preset is shown).
    drawn: Option<Vec<[f64; 2]>>,
    /// Race on the drawn course instead of the preset.
    use_drawn: bool,
    /// The scene is a canvas to draw a new course on.
    drawing: bool,
    /// The stroke being drawn \[m\].
    stroke: Vec<[f64; 2]>,
    /// Why the last stroke was not used.
    stroke_note: Option<&'static str>,
}

impl Default for ControllerArenaDemo {
    fn default() -> Self {
        Self::new(ArenaPreset::SlalomRecovery, DEFAULT_SPEED, DEFAULT_RESPONSE)
    }
}

impl ControllerArenaDemo {
    fn new(preset: ArenaPreset, target_speed: f64, turn_rate_response_gain: f64) -> Self {
        let scenario = preset.scenario(target_speed, turn_rate_response_gain);
        let result = run_controller_arena(&scenario, &ArenaControllerKind::ALL);
        let (runs, error) = match result {
            Ok(runs) => (runs, None),
            Err(error) => (Vec::new(), Some(error.to_string())),
        };
        Self {
            safety: safety::SafetyLab::default(),
            dwa: dwa::DwaLab::default(),
            preset,
            target_speed,
            turn_rate_response_gain,
            scenario,
            runs,
            frame_idx: 0,
            playing: true,
            dirty: false,
            error,
            last_advance: None,
            replay_phase: 0.0,
            replay_rate: 1.0,
            focus: ArenaControllerKind::PurePursuit,
            actions: BTreeMap::new(),
            action_hits: BTreeMap::new(),
            scene_rect: None,
            plot_rects: [None; 2],
            #[cfg(target_arch = "wasm32")]
            action_count: 0,
            #[cfg(target_arch = "wasm32")]
            last_action: "",
            #[cfg(target_arch = "wasm32")]
            ui_revision: 0,
            drawn: None,
            use_drawn: false,
            drawing: false,
            stroke: Vec::new(),
            stroke_note: None,
        }
    }

    /// The scenario for the current settings: the preset, or the preset's
    /// dynamics on the drawn course.
    fn build_scenario(&self) -> ArenaScenario {
        let mut scenario = self
            .preset
            .scenario(self.target_speed, self.turn_rate_response_gain);
        if let (true, Some(keys)) = (self.use_drawn, &self.drawn) {
            let course = resample(keys, COURSE_SPACING);
            let path =
                Path2D::from_points(course.iter().map(|p| Point2D::new(p[0], p[1])).collect());
            let length = course_length(&course);
            let start = path.points[0];
            let yaw = path.yaw_profile().first().copied().unwrap_or(0.0);
            // The same start offset as the presets: behind and beside the
            // first point, slightly misaligned.
            scenario.initial_state = State2D::new(
                start.x - 2.0 * yaw.cos() - 1.5 * yaw.sin(),
                start.y - 2.0 * yaw.sin() + 1.5 * yaw.cos(),
                yaw + 0.12,
                0.5,
            );
            scenario.max_steps = ((length / self.target_speed / scenario.dt) * 3.0) as usize + 150;
            scenario.path = path;
        }
        scenario
    }

    pub fn apply_share_query(&mut self, query: &str) {
        self.dwa.apply_query(query);
        self.safety.apply_query(query);
        if self.safety.active || self.dwa.active {
            return;
        }
        if crate::share::value(query, "tab") != Some("arena") {
            return;
        }
        self.use_drawn = false;
        if let Some(course) = crate::share::value(query, "course").and_then(decode_course) {
            self.drawn = Some(course);
            self.use_drawn = true;
            self.dirty = true;
        }
        self.preset = crate::share::value(query, "preset")
            .and_then(ArenaPreset::from_slug)
            .unwrap_or(self.preset);
        self.target_speed =
            bounded_query_f64(query, "speed", MIN_SPEED, MAX_SPEED).unwrap_or(self.target_speed);
        self.turn_rate_response_gain =
            bounded_query_f64(query, "response", MIN_RESPONSE, MAX_RESPONSE)
                .unwrap_or(self.turn_rate_response_gain);
        self.rebuild();
        self.seek(crate::share::bounded_usize(query, "arena_frame", 20_000).unwrap_or(0));
        self.playing = crate::share::boolean(query, "arena_playing").unwrap_or(true)
            && self.frame_idx < self.max_frame();
        self.replay_rate = bounded_query_f64(query, "arena_rate", 0.25, 2.0).unwrap_or(1.0);
        self.focus = crate::share::value(query, "arena_focus")
            .and_then(controller_from_slug)
            .unwrap_or(ArenaControllerKind::PurePursuit);
    }

    pub fn share_query(&self) -> String {
        if self.dwa.active {
            return self.dwa.share_query();
        }
        if self.safety.active {
            return self.safety.share_query();
        }
        let mut query = format!(
            "tab=arena&preset={}&speed={}&response={}&arena_frame={}&arena_playing={}&arena_rate={}&arena_focus={}",
            self.preset.slug(),
            self.target_speed,
            self.turn_rate_response_gain,
            self.frame_idx,
            u8::from(self.playing),
            self.replay_rate,
            controller_slug(self.focus),
        );
        if let (true, Some(course)) = (self.use_drawn, &self.drawn) {
            query.push_str("&course=");
            query.push_str(&encode_course(course));
        }
        query
    }

    /// Turns the finished stroke into the course (or says why not).
    fn finish_stroke(&mut self) {
        let stroke = std::mem::take(&mut self.stroke);
        if course_length(&stroke) < MIN_COURSE_LENGTH {
            self.stroke_note = Some("Too short: draw a longer course.");
            return;
        }
        if course_length(&stroke) > MAX_COURSE_LENGTH {
            self.stroke_note = Some("Too long: keep the course under 500 m.");
            return;
        }
        let smooth = smooth(&resample(&stroke, COURSE_SPACING), 4);
        // Keep exactly what a share link stores, so links replay exactly.
        let keys = resample(&smooth, LINK_SPACING)
            .iter()
            .map(|p| [(p[0] * 10.0).round() / 10.0, (p[1] * 10.0).round() / 10.0])
            .collect();
        self.drawn = Some(keys);
        self.use_drawn = true;
        self.drawing = false;
        self.stroke_note = None;
        self.dirty = true;
    }

    /// The drawing canvas: drag to draw, release to race.
    fn draw_canvas(&mut self, ui: &mut egui::Ui) {
        let rect = crate::ui_kit::fit_rect(ui, (DRAW_H / DRAW_W) as f32, 72.0);
        self.scene_rect = Some(rect);
        let response = ui.allocate_rect(rect, egui::Sense::click_and_drag());
        let to_world = |pos: Pos2| {
            [
                f64::from((pos.x - rect.left()) / rect.width()) * DRAW_W,
                f64::from((rect.bottom() - pos.y) / rect.height()) * DRAW_H,
            ]
        };
        let to_screen = |p: [f64; 2]| {
            Pos2::new(
                rect.left() + (p[0] / DRAW_W) as f32 * rect.width(),
                rect.bottom() - (p[1] / DRAW_H) as f32 * rect.height(),
            )
        };
        if response.drag_started() {
            self.stroke.clear();
            if let Some(origin) = response.ctx.input(|i| i.pointer.press_origin()) {
                self.stroke.push(to_world(origin));
            }
        }
        if response.dragged() {
            if let Some(pos) = response.interact_pointer_pos() {
                let p = to_world(pos);
                let far = self
                    .stroke
                    .last()
                    .map_or(true, |q| (p[0] - q[0]).hypot(p[1] - q[1]) > 0.3);
                if far && rect.contains(pos) {
                    self.stroke.push(p);
                }
            }
        }
        if response.drag_stopped() {
            self.finish_stroke();
        }

        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 5.0, Color32::from_rgb(18, 22, 28));
        for k in 1..(DRAW_W / 5.0) as usize {
            let x = k as f64 * 5.0;
            painter.line_segment(
                [to_screen([x, 0.0]), to_screen([x, DRAW_H])],
                Stroke::new(1.0_f32, Color32::from_rgb(30, 35, 44)),
            );
        }
        for k in 1..(DRAW_H / 5.0) as usize {
            let y = k as f64 * 5.0;
            painter.line_segment(
                [to_screen([0.0, y]), to_screen([DRAW_W, y])],
                Stroke::new(1.0_f32, Color32::from_rgb(30, 35, 44)),
            );
        }
        if self.stroke.len() >= 2 {
            let points = self.stroke.iter().map(|p| to_screen(*p)).collect();
            painter.add(egui::Shape::line(
                points,
                Stroke::new(3.0_f32, crate::ui_kit::ACCENT),
            ));
        }
        crate::ui_kit::overlay_text(
            ui,
            rect,
            self.stroke_note.unwrap_or(
                "Drag to draw a course (grid: 5 m). Release and the three controllers race it.",
            ),
        );
    }

    fn rebuild(&mut self) {
        self.scenario = self.build_scenario();
        match run_controller_arena(&self.scenario, &ArenaControllerKind::ALL) {
            Ok(runs) => {
                self.runs = runs;
                self.error = None;
            }
            Err(error) => {
                self.runs.clear();
                self.error = Some(error.to_string());
            }
        }
        self.seek(0);
        self.dirty = false;
    }

    fn seek(&mut self, frame: usize) {
        self.frame_idx = frame.min(self.max_frame());
        self.playing = false;
        self.last_advance = None;
        self.replay_phase = 0.0;
    }

    fn advance(&mut self, now: f64) {
        let elapsed = self
            .last_advance
            .replace(now)
            .map_or(0.0, |last| now - last);
        if !self.playing || !elapsed.is_finite() || !(0.0..1.0).contains(&elapsed) {
            self.replay_phase = 0.0;
            return;
        }
        self.replay_phase += elapsed * self.replay_rate / self.scenario.dt;
        let steps = (self.replay_phase + 1e-9).floor() as usize;
        self.replay_phase = (self.replay_phase - steps as f64).max(0.0);
        self.frame_idx = (self.frame_idx + steps).min(self.max_frame());
        if self.frame_idx == self.max_frame() {
            self.playing = false;
        }
    }

    fn prefix<'a>(&self, run: &'a ArenaRun) -> &'a [ArenaSample] {
        &run.samples[..=self.frame_idx.min(run.samples.len().saturating_sub(1))]
    }

    fn prefix_metrics(&self, run: &ArenaRun) -> ArenaMetrics {
        metrics_from_samples(
            self.prefix(run),
            *self.scenario.path.points.last().expect("valid path"),
        )
        .expect("validated arena samples")
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

    fn max_frame(&self) -> usize {
        self.runs
            .iter()
            .map(|run| run.samples.len().saturating_sub(1))
            .max()
            .unwrap_or(0)
    }

    pub fn begin_frame(&mut self, ctx: &egui::Context, active: bool) {
        self.dwa.begin_frame(ctx, active);
        self.safety.begin_frame(ctx, active);
    }
    pub fn finish(&mut self, ctx: &egui::Context, active: bool) {
        self.dwa.finish(ctx, active);
        self.safety.finish(ctx, active);
    }
    pub fn controls(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        if self.dwa.active {
            self.dwa.controls(ui);
            return;
        }
        if self.safety.active {
            self.safety.controls(ui);
            return;
        }
        let r = ui.button("Open DWA velocity lab");
        if self.dwa.button("inspect", &r) {
            self.dwa.enter();
            return;
        }
        let r = ui.button("Open safety filter lab");
        if self.safety.button("inspect", &r) {
            self.safety.enter();
            return;
        }
        crate::ui_kit::section(ui, "Course");
        ui.horizontal_wrapped(|ui| {
            for (preset, label) in ArenaPreset::ALL
                .into_iter()
                .zip(["Straight", "Slalom", "Hairpin"])
            {
                let response = ui.selectable_label(!self.use_drawn && self.preset == preset, label);
                self.action(preset.slug(), &response);
                if response.clicked() {
                    self.preset = preset;
                    self.use_drawn = false;
                    self.dirty = true;
                }
            }
            if self.drawn.is_some() {
                let response = ui.selectable_label(self.use_drawn, "Your course");
                self.action("custom", &response);
                if response.clicked() {
                    self.use_drawn = true;
                    self.dirty = true;
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            let response = ui.button(if self.drawing {
                "Cancel drawing"
            } else {
                "Draw a course"
            });
            self.action("draw", &response);
            if response.clicked() {
                if self.drawing {
                    self.drawing = false;
                    self.stroke.clear();
                } else {
                    self.drawing = true;
                    self.stroke.clear();
                    self.stroke_note = None;
                    self.seek(self.frame_idx);
                }
            }
        });
        crate::ui_kit::section(ui, "Target speed");
        if ui
            .add(
                egui::Slider::new(&mut self.target_speed, MIN_SPEED..=MAX_SPEED)
                    .clamping(egui::SliderClamping::Edits)
                    .suffix(" m/s")
                    .step_by(0.25),
            )
            .changed()
        {
            self.dirty = true;
        }
        crate::ui_kit::section(ui, "Steering response");
        if ui
            .add(
                egui::Slider::new(
                    &mut self.turn_rate_response_gain,
                    MIN_RESPONSE..=MAX_RESPONSE,
                )
                .clamping(egui::SliderClamping::Edits)
                .step_by(0.05),
            )
            .changed()
        {
            self.dirty = true;
        }
        ui.horizontal_wrapped(|ui| {
            for (key, label, gain) in [("ideal", "Ideal", 1.0), ("weak", "Weaker", 0.5)] {
                let response = ui.selectable_label(self.turn_rate_response_gain == gain, label);
                self.action(key, &response);
                if response.clicked() {
                    self.turn_rate_response_gain = gain;
                    self.dirty = true;
                }
            }
            let response = ui.button("Reset");
            self.action("reset", &response);
            if response.clicked() {
                self.preset = ArenaPreset::SlalomRecovery;
                self.target_speed = DEFAULT_SPEED;
                self.turn_rate_response_gain = DEFAULT_RESPONSE;
                self.use_drawn = false;
                self.drawing = false;
                self.focus = ArenaControllerKind::PurePursuit;
                self.replay_rate = 1.0;
                self.dirty = true;
            }
        });
        if self.dirty {
            self.rebuild();
            self.playing = true;
        }

        crate::ui_kit::section(ui, "Inspect a controller");
        self.draw_metrics(ui);
        crate::ui_kit::how_it_works(
            ui,
            "arena_help",
            "All three trackers share a path, initial state, speed response, turn limits, \
             and 0.1 s simulation clock. Turn rate is the applied command after the shared \
             response, not a steering-wheel angle. Positive turns left. Error uses the \
             nearest sampled path point. Graphs and metrics include only samples up to \
             the selected time. Shorter traces stop at their last sample; their final \
             command is labelled as such. These traces describe one model, not a universal ranking.",
        );
        #[cfg(target_arch = "wasm32")]
        {
            self.ui_revision += 1;
        }
        if let Some(rect) = self.scene_rect {
            self.publish_state(ctx, rect);
        }
    }

    pub fn scene(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        if self.dwa.active {
            self.dwa.scene(ui);
            return;
        }
        if self.safety.active {
            self.safety.scene(ui);
            return;
        }
        if self.drawing {
            self.draw_canvas(ui);
            #[cfg(target_arch = "wasm32")]
            {
                self.ui_revision += 1;
            }
            if let Some(rect) = self.scene_rect {
                self.publish_state(ctx, rect);
            }
            return;
        }
        if self.dirty {
            // Settings changed: rerun all controllers and replay from the start.
            self.rebuild();
            self.playing = true;
        }
        if let Some(error) = &self.error {
            ui.colored_label(Color32::LIGHT_RED, format!("Arena error: {error}"));
        }
        self.playback(ui);
        self.advance(ctx.input(|i| i.time));
        let aspect = self
            .world_bounds()
            .map(|b| ((b.max_y - b.min_y) / (b.max_x - b.min_x)) as f32)
            .unwrap_or(0.6)
            .clamp(0.35, 1.2);
        let rect = crate::ui_kit::fit_rect(ui, aspect, 240.0);
        self.scene_rect = Some(rect);
        let _ = ui.allocate_rect(rect, egui::Sense::hover());
        self.draw_scene(ui, rect);

        let legend: Vec<_> = self
            .runs
            .iter()
            .map(|run| (controller_color(run.controller), run.controller.label()))
            .collect();
        crate::ui_kit::legend(ui, &legend);
        self.draw_plot(ui, false);
        self.draw_plot(ui, true);
        #[cfg(target_arch = "wasm32")]
        {
            self.ui_revision += 1;
        }
        self.publish_state(ctx, rect);
        if self.playing {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));
        }
    }

    fn playback(&mut self, ui: &mut egui::Ui) {
        let last = self.max_frame();
        ui.horizontal_wrapped(|ui| {
            for (key, label) in [
                ("play", if self.playing { "Pause" } else { "Play" }),
                ("start", "Start"),
                ("back", "Back"),
                ("next", "Step"),
                ("end", "End"),
            ] {
                let response = ui.button(label);
                self.action(key, &response);
                if response.clicked() {
                    match key {
                        "play" => {
                            let playing = !self.playing;
                            self.seek(if playing && self.frame_idx == last {
                                0
                            } else {
                                self.frame_idx
                            });
                            self.playing = playing;
                        }
                        "start" => self.seek(0),
                        "back" => self.seek(self.frame_idx.saturating_sub(1)),
                        "next" => self.seek(self.frame_idx.saturating_add(1)),
                        "end" => self.seek(last),
                        _ => unreachable!(),
                    }
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            let mut frame = self.frame_idx;
            let response = ui.add(egui::Slider::new(&mut frame, 0..=last.max(1)).show_value(false));
            self.action("timeline", &response);
            if response.changed() {
                self.seek(frame);
            }
            ui.monospace(format!(
                "{:.1} / {:.1} s",
                self.frame_idx as f64 * self.scenario.dt,
                last as f64 * self.scenario.dt
            ));
            for (key, rate) in [("slow", 0.25), ("normal", 1.0), ("fast", 2.0)] {
                let response = ui.selectable_label(self.replay_rate == rate, format!("{rate}x"));
                self.action(key, &response);
                if response.clicked() {
                    self.replay_rate = rate;
                    self.last_advance = None;
                    self.replay_phase = 0.0;
                }
            }
        });
    }

    fn draw_scene(&self, ui: &egui::Ui, rect: Rect) {
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 5.0, Color32::from_rgb(18, 22, 28));
        let Some(bounds) = self.world_bounds() else {
            return;
        };

        let reference: Vec<Pos2> = self
            .scenario
            .path
            .points
            .iter()
            .map(|point| world_to_screen(rect, bounds, point.x, point.y))
            .collect();
        painter.add(egui::Shape::line(
            reference,
            Stroke::new(3.0_f32, Color32::from_rgb(175, 180, 190)),
        ));

        for (point, label) in [
            (self.scenario.path.points.first(), "START"),
            (self.scenario.path.points.last(), "GOAL"),
        ] {
            if let Some(point) = point {
                let p = world_to_screen(rect, bounds, point.x, point.y);
                painter.circle_stroke(p, 7.0, Stroke::new(1.5_f32, Color32::LIGHT_GRAY));
                painter.text(
                    p + Vec2::new(0.0, -12.0),
                    egui::Align2::CENTER_BOTTOM,
                    label,
                    egui::FontId::proportional(10.0),
                    Color32::LIGHT_GRAY,
                );
            }
        }

        for run in &self.runs {
            let color =
                controller_color(run.controller).gamma_multiply(if run.controller == self.focus {
                    1.0
                } else {
                    0.55
                });
            let upto = self.frame_idx.min(run.samples.len().saturating_sub(1));
            let trail: Vec<Pos2> = run.samples[..=upto]
                .iter()
                .map(|sample| world_to_screen(rect, bounds, sample.state.x, sample.state.y))
                .collect();
            if trail.len() >= 2 {
                painter.add(egui::Shape::line(
                    trail,
                    Stroke::new(
                        2.0_f32,
                        Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), 190),
                    ),
                ));
            }
            if let Some(sample) = run.samples.get(upto) {
                if run.controller == self.focus {
                    let p = world_to_screen(rect, bounds, sample.state.x, sample.state.y);
                    painter.circle_stroke(p, 11.0, Stroke::new(2.0_f32, color));
                    if let Some(target) = self.scenario.path.points.iter().min_by(|a, b| {
                        a.distance(&sample.state.position())
                            .total_cmp(&b.distance(&sample.state.position()))
                    }) {
                        painter.line_segment(
                            [p, world_to_screen(rect, bounds, target.x, target.y)],
                            Stroke::new(1.5_f32, Color32::WHITE.gamma_multiply(0.65)),
                        );
                    }
                }
                draw_robot(
                    &painter,
                    world_to_screen(rect, bounds, sample.state.x, sample.state.y),
                    sample.state.yaw,
                    color,
                );
            }
        }
    }

    fn draw_metrics(&mut self, ui: &mut egui::Ui) {
        for index in 0..self.runs.len() {
            let run = &self.runs[index];
            let kind = run.controller;
            let sample = *self.prefix(run).last().expect("initial sample");
            let ended = self.frame_idx >= run.samples.len() - 1;
            let color = controller_color(kind);
            egui::Frame::new()
                .fill(Color32::from_rgb(22, 28, 36))
                .inner_margin(8)
                .show(ui, |ui| {
                    let response = ui.selectable_label(
                        self.focus == kind,
                        egui::RichText::new(kind.label()).color(if self.focus == kind {
                            Color32::from_rgb(10, 15, 22)
                        } else {
                            color
                        }),
                    );
                    self.action(controller_slug(kind), &response);
                    if response.clicked() {
                        self.focus = kind;
                    }
                    ui.small(format!(
                        "Error {:.2} m / speed {:.2} m/s",
                        sample.cross_track_error.abs(),
                        sample.state.v
                    ));
                    ui.small(format!(
                        "{}turn {:+.3} rad/s",
                        if ended { "Last " } else { "Applied " },
                        sample.command.omega
                    ));
                    let (rect, _) = ui.allocate_exact_size(
                        Vec2::new(ui.available_width(), 8.0),
                        egui::Sense::hover(),
                    );
                    ui.painter()
                        .rect_filled(rect, 2.0, Color32::from_rgb(9, 14, 20));
                    let center = rect.center().x;
                    let end = center
                        + (sample.command.omega / self.scenario.max_angular_speed) as f32
                            * rect.width()
                            * 0.5;
                    ui.painter().rect_filled(
                        Rect::from_min_max(
                            egui::pos2(center.min(end), rect.top()),
                            egui::pos2(center.max(end), rect.bottom()),
                        ),
                        1.0,
                        color,
                    );
                    ui.painter().line_segment(
                        [
                            egui::pos2(center, rect.top()),
                            egui::pos2(center, rect.bottom()),
                        ],
                        Stroke::new(1.0_f32, Color32::WHITE),
                    );
                    if ended {
                        ui.small(format!("Trace ended at {:.1} s", sample.time));
                    }
                });
        }
        if let Some(run) = self.runs.iter().find(|run| run.controller == self.focus) {
            let metrics = self.prefix_metrics(run);
            crate::ui_kit::section(ui, "Selected trace / observed so far");
            ui.small(format!(
                "RMSE {:.3} m / max {:.3} m",
                metrics.cross_track_rmse, metrics.max_cross_track_error
            ));
            ui.small(format!(
                "Goal now {:.2} m / turn change {:.3} rad/s",
                metrics.final_goal_distance, metrics.angular_command_smoothness
            ));
        }
    }

    fn draw_plot(&mut self, ui: &mut egui::Ui, angular: bool) {
        ui.small(if angular {
            "Applied turn rate [rad/s], + left"
        } else {
            "Path error [m] / sampled path points"
        });
        let (rect, response) = ui.allocate_exact_size(
            Vec2::new(ui.available_width(), 72.0),
            egui::Sense::click_and_drag(),
        );
        self.plot_rects[usize::from(angular)] = Some(rect);
        self.action(if angular { "turn-plot" } else { "error-plot" }, &response);
        if (response.clicked() || response.dragged()) && response.interact_pointer_pos().is_some() {
            let x = response.interact_pointer_pos().expect("checked pointer").x;
            self.seek(
                (((x - rect.left()) / rect.width()).clamp(0.0, 1.0) * self.max_frame() as f32)
                    .round() as usize,
            );
        }
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 4.0, Color32::from_rgb(10, 14, 20));
        let peak = if angular {
            self.scenario.max_angular_speed
        } else {
            self.runs
                .iter()
                .flat_map(|run| self.prefix(run))
                .map(|s| s.cross_track_error.abs())
                .fold(0.5_f64, f64::max)
        };
        let y = |value: f64| {
            if angular {
                rect.center().y - (value / peak) as f32 * (rect.height() * 0.38)
            } else {
                rect.bottom() - 4.0 - (value / peak) as f32 * (rect.height() - 22.0)
            }
        };
        painter.line_segment(
            [
                egui::pos2(rect.left(), y(0.0)),
                egui::pos2(rect.right(), y(0.0)),
            ],
            Stroke::new(0.75_f32, Color32::from_gray(65)),
        );
        for run in &self.runs {
            let color =
                controller_color(run.controller).gamma_multiply(if run.controller == self.focus {
                    1.0
                } else {
                    0.5
                });
            let points: Vec<_> = self
                .prefix(run)
                .iter()
                .map(|s| {
                    egui::pos2(
                        rect.left()
                            + s.time as f32
                                / (self.max_frame().max(1) as f32 * self.scenario.dt as f32)
                                * rect.width(),
                        y(if angular {
                            s.command.omega
                        } else {
                            s.cross_track_error.abs()
                        }),
                    )
                })
                .collect();
            let end = points.last().copied();
            if points.len() > 1 {
                painter.add(egui::Shape::line(points, Stroke::new(1.8_f32, color)));
            }
            if let Some(p) = end {
                painter.circle_filled(p, 2.5, color);
            }
        }
        let x = rect.left() + self.frame_idx as f32 / self.max_frame().max(1) as f32 * rect.width();
        painter.line_segment(
            [egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())],
            Stroke::new(1.0_f32, Color32::WHITE.gamma_multiply(0.5)),
        );
        painter.text(
            rect.left_top() + Vec2::splat(4.0),
            egui::Align2::LEFT_TOP,
            format!("{peak:.2} / click to inspect"),
            egui::FontId::proportional(10.0),
            Color32::GRAY,
        );
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish_state(&self, _ctx: &egui::Context, _rect: Rect) {}

    #[cfg(target_arch = "wasm32")]
    fn publish_state(&self, ctx: &egui::Context, rect: Rect) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let encode_point = |p: &Point2D| format!("[{:.8},{:.8}]", p.x, p.y);
        let current = self
            .runs
            .iter()
            .map(|run| {
                let prefix = self.prefix(run);
                let sample = prefix.last().expect("initial sample");
                let metrics = self.prefix_metrics(run);
                format!(
                    concat!(
                        "{{\"id\":\"{id}\",\"frame\":{frame},\"time\":{time:.8},",
                        "\"x\":{x:.8},\"y\":{y:.8},\"yaw\":{yaw:.8},\"v\":{v:.8},",
                        "\"omega\":{omega:.8},\"error\":{error:.8},\"rmse\":{rmse:.8},",
                        "\"max\":{max:.8},\"goal\":{goal:.8},\"smooth\":{smooth:.8},\"ended\":{ended}}}"
                    ),
                    id = controller_slug(run.controller),
                    frame = prefix.len() - 1,
                    time = sample.time,
                    x = sample.state.x,
                    y = sample.state.y,
                    yaw = sample.state.yaw,
                    v = sample.state.v,
                    omega = sample.command.omega,
                    error = sample.cross_track_error,
                    rmse = metrics.cross_track_rmse,
                    max = metrics.max_cross_track_error,
                    goal = metrics.final_goal_distance,
                    smooth = metrics.angular_command_smoothness,
                    ended = self.frame_idx >= run.samples.len() - 1,
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        let series = if self.playing {
            String::new()
        } else {
            self.runs
                .iter()
                .map(|run| {
                    format!(
                        "[{}]",
                        self.prefix(run)
                            .iter()
                            .map(|s| format!(
                                "[{:.8},{:.8},{:.8}]",
                                s.time, s.command.omega, s.cross_track_error
                            ))
                            .collect::<Vec<_>>()
                            .join(",")
                    )
                })
                .collect::<Vec<_>>()
                .join(",")
        };
        let actions = self
            .actions
            .iter()
            .map(|(key, p)| format!("\"{key}\":[{:.2},{:.2}]", p[0], p[1]))
            .collect::<Vec<_>>()
            .join(",");
        let plots = self.plot_rects.map(|rect| {
            rect.map_or_else(
                || "null".to_owned(),
                |rect| {
                    format!(
                        "[{:.2},{:.2},{:.2},{:.2}]",
                        rect.left(),
                        rect.top(),
                        rect.width(),
                        rect.height()
                    )
                },
            )
        });
        let hits = self
            .action_hits
            .iter()
            .map(|(key, hit)| format!("\"{key}\":{hit}"))
            .collect::<Vec<_>>()
            .join(",");
        let pointer = ctx
            .input(|i| i.pointer.hover_pos())
            .map_or_else(|| "null".to_owned(), |p| format!("[{:.2},{:.2}]", p.x, p.y));
        for (key, value) in [
            ("data-arena-frame", self.frame_idx.to_string()),
            ("data-arena-total", self.max_frame().to_string()),
            ("data-arena-playing", u8::from(self.playing).to_string()),
            ("data-arena-focus", controller_slug(self.focus).to_owned()),
            ("data-arena-preset", self.preset.slug().to_owned()),
            (
                "data-arena-response",
                self.turn_rate_response_gain.to_string(),
            ),
            ("data-arena-custom", u8::from(self.use_drawn).to_string()),
            ("data-arena-drawing", u8::from(self.drawing).to_string()),
            ("data-arena-ready", u8::from(!self.dirty).to_string()),
            ("data-arena-current", format!("[{current}]")),
            ("data-arena-series", format!("[{series}]")),
            (
                "data-arena-goal",
                self.scenario
                    .path
                    .points
                    .last()
                    .map_or_else(|| "null".to_owned(), encode_point),
            ),
            ("data-arena-actions", format!("{{{actions}}}")),
            ("data-arena-hits", format!("{{{hits}}}")),
            ("data-arena-pointer", pointer),
            ("data-arena-plots", format!("[{},{}]", plots[0], plots[1])),
            ("data-arena-action-count", self.action_count.to_string()),
            ("data-arena-last-action", self.last_action.to_owned()),
            ("data-arena-ui-revision", self.ui_revision.to_string()),
            ("data-arena-share", self.share_query()),
            (
                "data-arena-rect",
                format!(
                    "[{:.2},{:.2},{:.2},{:.2}]",
                    rect.left(),
                    rect.top(),
                    rect.width(),
                    rect.height()
                ),
            ),
            (
                "aria-label",
                format!(
                    "Controller comparison at {:.1} seconds, {}, {} selected",
                    self.frame_idx as f64 * self.scenario.dt,
                    if self.playing { "playing" } else { "paused" },
                    self.focus.label()
                ),
            ),
        ] {
            let _ = canvas.set_attribute(key, &value);
        }
    }

    fn world_bounds(&self) -> Option<WorldBounds> {
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for (x, y) in self
            .scenario
            .path
            .points
            .iter()
            .map(|point| (point.x, point.y))
            .chain(
                self.runs
                    .iter()
                    .flat_map(|run| run.samples.iter())
                    .map(|sample| (sample.state.x, sample.state.y)),
            )
        {
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
        }
        if !min_x.is_finite() {
            return None;
        }
        let margin = 2.0;
        Some(WorldBounds {
            min_x: min_x - margin,
            max_x: max_x + margin,
            min_y: min_y - margin,
            max_y: max_y + margin,
        })
    }
}

#[derive(Clone, Copy)]
struct WorldBounds {
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
}

fn world_to_screen(rect: Rect, bounds: WorldBounds, x: f64, y: f64) -> Pos2 {
    let world_width = (bounds.max_x - bounds.min_x).max(1e-6);
    let world_height = (bounds.max_y - bounds.min_y).max(1e-6);
    let scale = (rect.width() / world_width as f32)
        .min(rect.height() / world_height as f32)
        .max(1e-6);
    let drawn_width = world_width as f32 * scale;
    let drawn_height = world_height as f32 * scale;
    let left = rect.left() + (rect.width() - drawn_width) * 0.5;
    let top = rect.top() + (rect.height() - drawn_height) * 0.5;
    Pos2::new(
        left + (x - bounds.min_x) as f32 * scale,
        top + drawn_height - (y - bounds.min_y) as f32 * scale,
    )
}

fn course_length(points: &[[f64; 2]]) -> f64 {
    points
        .windows(2)
        .map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]))
        .sum()
}

/// Points every `spacing` meters along the polyline (both ends kept).
fn resample(points: &[[f64; 2]], spacing: f64) -> Vec<[f64; 2]> {
    let Some(&first) = points.first() else {
        return Vec::new();
    };
    let mut out = vec![first];
    let mut carried = 0.0;
    for w in points.windows(2) {
        let (a, b) = (w[0], w[1]);
        let length = (b[0] - a[0]).hypot(b[1] - a[1]);
        let mut at = spacing - carried;
        while at <= length {
            let t = at / length;
            out.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
            at += spacing;
        }
        carried = length - (at - spacing);
    }
    let last = *points.last().expect("non-empty");
    if out
        .last()
        .is_some_and(|p| (p[0] - last[0]).hypot(p[1] - last[1]) > spacing * 0.25)
    {
        out.push(last);
    }
    out
}

/// Moving average over `half` points on each side; the ends stay put.
fn smooth(points: &[[f64; 2]], half: usize) -> Vec<[f64; 2]> {
    let n = points.len();
    (0..n)
        .map(|i| {
            let reach = half.min(i).min(n - 1 - i);
            let window = &points[i - reach..=i + reach];
            let count = window.len() as f64;
            [
                window.iter().map(|p| p[0]).sum::<f64>() / count,
                window.iter().map(|p| p[1]).sum::<f64>() / count,
            ]
        })
        .collect()
}

/// Preserve all input precision, including courses supplied in an incoming URL.
fn encode_course(keys: &[[f64; 2]]) -> String {
    keys.iter()
        .map(|p| format!("{},{}", p[0], p[1]))
        .collect::<Vec<_>>()
        .join(";")
}

/// The course key points a link describes.
fn decode_course(value: &str) -> Option<Vec<[f64; 2]>> {
    let points: Vec<[f64; 2]> = value
        .split(';')
        .take(1000)
        .map(|pair| {
            let (x, y) = pair.split_once(',')?;
            let p = [x.parse::<f64>().ok()?, y.parse::<f64>().ok()?];
            p.iter()
                .all(|v| v.is_finite() && v.abs() <= 1000.0)
                .then_some(p)
        })
        .collect::<Option<_>>()?;
    (points.len() >= 2 && (MIN_COURSE_LENGTH..=MAX_COURSE_LENGTH).contains(&course_length(&points)))
        .then_some(points)
}

fn draw_robot(painter: &egui::Painter, center: Pos2, yaw: f64, color: Color32) {
    let heading = Vec2::angled(-(yaw as f32));
    painter.circle_filled(center, 5.5, color);
    painter.line_segment(
        [center, center + heading * 12.0],
        Stroke::new(2.5_f32, color),
    );
}

fn controller_color(kind: ArenaControllerKind) -> Color32 {
    match kind {
        ArenaControllerKind::PurePursuit => Color32::from_rgb(80, 180, 255),
        ArenaControllerKind::Stanley => Color32::from_rgb(255, 155, 70),
        ArenaControllerKind::LqrSteer => Color32::from_rgb(100, 220, 130),
    }
}

fn controller_slug(kind: ArenaControllerKind) -> &'static str {
    match kind {
        ArenaControllerKind::PurePursuit => "pp",
        ArenaControllerKind::Stanley => "stanley",
        ArenaControllerKind::LqrSteer => "lqr",
    }
}

fn controller_from_slug(value: &str) -> Option<ArenaControllerKind> {
    ArenaControllerKind::ALL
        .into_iter()
        .find(|kind| controller_slug(*kind) == value)
}

fn bounded_query_f64(query: &str, key: &str, min: f64, max: f64) -> Option<f64> {
    crate::share::value(query, key)
        .and_then(|value| value.parse::<f64>().ok())
        .filter(|value| value.is_finite() && (min..=max).contains(value))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn share_query_round_trips_arena_settings() {
        let source = ControllerArenaDemo::new(ArenaPreset::HairpinRecovery, 4.25, 0.65);
        let query = source.share_query();
        let mut restored = ControllerArenaDemo::default();
        restored.apply_share_query(&query);
        assert_eq!(restored.preset, ArenaPreset::HairpinRecovery);
        assert_eq!(restored.target_speed, 4.25);
        assert_eq!(restored.turn_rate_response_gain, 0.65);
        assert_eq!(restored.share_query(), query);
    }

    /// A wavy stroke drawn across the canvas, as pointer samples.
    fn wavy_stroke() -> Vec<[f64; 2]> {
        (0..=160)
            .map(|i| {
                let x = 5.0 + i as f64 * 0.3;
                [x, 15.0 + 6.0 * (x / 6.0).sin()]
            })
            .collect()
    }

    #[test]
    fn a_drawn_course_is_raced_by_all_three_controllers() {
        let mut demo = ControllerArenaDemo {
            drawing: true,
            stroke: wavy_stroke(),
            ..ControllerArenaDemo::default()
        };
        demo.finish_stroke();
        assert!(!demo.drawing && demo.use_drawn);
        demo.rebuild();
        assert_eq!(demo.runs.len(), 3, "{:?}", demo.error);
        let course = resample(demo.drawn.as_ref().unwrap(), COURSE_SPACING);
        // Evenly spaced points.
        for w in course.windows(2) {
            let d = (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]);
            assert!(d < COURSE_SPACING * 1.3, "gap {d}");
        }
        let end = course.last().unwrap();
        for run in &demo.runs {
            assert!(
                run.metrics.final_goal_distance < 1.5,
                "{} ends {:.2} m from the goal {end:?}",
                run.controller.label(),
                run.metrics.final_goal_distance
            );
            assert!(run.metrics.max_cross_track_error < 3.0);
        }
    }

    #[test]
    fn short_strokes_are_ignored() {
        let mut demo = ControllerArenaDemo {
            drawing: true,
            stroke: vec![[1.0, 1.0], [3.0, 1.0]],
            ..ControllerArenaDemo::default()
        };
        demo.finish_stroke();
        assert!(demo.drawing && demo.drawn.is_none());
        assert!(demo.stroke_note.is_some());
    }

    #[test]
    fn a_shared_course_replays_the_same_race() {
        let mut demo = ControllerArenaDemo {
            stroke: wavy_stroke(),
            ..ControllerArenaDemo::default()
        };
        demo.finish_stroke();
        demo.rebuild();
        let query = demo.share_query();
        assert!(query.contains("course="));
        let mut restored = ControllerArenaDemo::default();
        restored.apply_share_query(&query);
        restored.rebuild();
        assert_eq!(restored.drawn, demo.drawn);
        assert_eq!(restored.runs, demo.runs);
        // Junk courses are ignored.
        let mut junk = ControllerArenaDemo::default();
        junk.apply_share_query("tab=arena&course=1,2;nan,3");
        junk.apply_share_query("tab=arena&course=1,1;2,1");
        assert!(junk.drawn.is_none());
    }

    #[test]
    fn invalid_share_values_fall_back_without_panicking() {
        let mut demo = ControllerArenaDemo::default();
        demo.apply_share_query("tab=arena&preset=unknown&speed=nan&response=4.0");
        assert_eq!(demo.preset, ArenaPreset::SlalomRecovery);
        assert_eq!(demo.target_speed, DEFAULT_SPEED);
        assert_eq!(demo.turn_rate_response_gain, DEFAULT_RESPONSE);
    }

    #[test]
    fn sharing_a_paused_moment_restores_the_exact_controller_samples() {
        let mut source = ControllerArenaDemo {
            stroke: wavy_stroke(),
            ..ControllerArenaDemo::new(ArenaPreset::HairpinRecovery, 4.255, 0.654321)
        };
        source.finish_stroke();
        // Incoming links can contain more precision than the drawing UI emits.
        source.drawn.as_mut().unwrap()[1][0] += 0.0123456789;
        source.rebuild();
        source.seek(70);
        source.focus = ArenaControllerKind::Stanley;
        source.replay_rate = 0.25;
        let mut restored = ControllerArenaDemo::default();
        restored.apply_share_query(&source.share_query());
        assert_eq!(restored.share_query(), source.share_query());
        assert!(!restored.playing && restored.use_drawn);
        assert_eq!(restored.frame_idx, 70);
        assert_eq!(restored.runs, source.runs);
        for (actual, expected) in restored.runs.iter().zip(&source.runs) {
            assert_eq!(restored.prefix(actual), source.prefix(expected));
            assert_eq!(
                restored.prefix_metrics(actual),
                source.prefix_metrics(expected)
            );
        }
    }

    #[test]
    fn rendering_shared_settings_does_not_round_or_restart_a_paused_trace() {
        for (speed, response) in [(3.0, 0.85), (3.133, 0.666666), (4.25, 0.65)] {
            let mut demo = ControllerArenaDemo::default();
            demo.apply_share_query(&format!(
                "tab=arena&preset=hairpin&speed={speed}&response={response}&arena_frame=100&arena_playing=0"
            ));
            let shared = demo.share_query();
            let context = egui::Context::default();
            for _ in 0..3 {
                let _ = context.run(egui::RawInput::default(), |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| demo.controls(ctx, ui));
                });
                assert_eq!(
                    demo.share_query(),
                    shared,
                    "displaying sliders edited the scenario"
                );
                assert_eq!(demo.frame_idx, 100);
                assert!(!demo.playing && !demo.dirty);
            }
        }
    }

    #[test]
    fn inspection_metrics_stop_at_the_selected_time_and_the_end_of_each_trace() {
        let mut demo = ControllerArenaDemo::default();
        demo.seek(0);
        for run in &demo.runs {
            let initial = run.samples[0];
            let metrics = demo.prefix_metrics(run);
            assert_eq!(demo.prefix(run).len(), 1);
            assert_eq!(metrics.cross_track_rmse, initial.cross_track_error.abs());
            assert_eq!(
                metrics.max_cross_track_error,
                initial.cross_track_error.abs()
            );
            assert_eq!(metrics.angular_command_smoothness, 0.0);
            assert_ne!(
                metrics, run.metrics,
                "future samples leaked into initial metrics"
            );
        }
        demo.seek(50);
        for run in &demo.runs {
            assert_eq!(demo.prefix(run).len(), 51);
            let goal = *demo.scenario.path.points.last().unwrap();
            let current = demo.prefix(run).last().unwrap();
            assert_eq!(current.time, 5.0);
            assert_eq!(
                demo.prefix_metrics(run).final_goal_distance,
                current.state.position().distance(&goal)
            );
        }
        demo.seek(usize::MAX);
        assert_eq!(demo.frame_idx, demo.max_frame());
        assert!(!demo.playing);
        assert!(demo
            .runs
            .iter()
            .any(|run| run.samples.len() - 1 < demo.frame_idx));
        for run in &demo.runs {
            assert_eq!(demo.prefix(run), run.samples);
            assert_eq!(demo.prefix_metrics(run), run.metrics);
        }
    }

    #[test]
    fn playback_uses_elapsed_time_across_fast_and_slow_paints() {
        let mut demo = ControllerArenaDemo::default();
        for (paints, period) in [(100, 0.01), (25, 0.04), (5, 0.2)] {
            for rate in [0.25, 1.0, 2.0] {
                demo.seek(0);
                demo.playing = true;
                demo.replay_rate = rate;
                demo.advance(0.0);
                for i in 1..=paints {
                    demo.advance(i as f64 * period);
                }
                assert_eq!(demo.frame_idx, (10.0_f64 * rate).floor() as usize);
            }
        }
        demo.seek(0);
        demo.playing = true;
        demo.replay_rate = 1.0;
        demo.advance(0.0);
        for i in 1..=5 {
            demo.advance(i as f64 * 0.4);
        }
        assert_eq!(demo.frame_idx, 20, "slow paints lost simulation ticks");
    }

    #[test]
    fn pause_seeking_and_background_gaps_do_not_accumulate_replay_time() {
        let mut demo = ControllerArenaDemo::default();
        demo.advance(0.0);
        demo.advance(0.15);
        assert_eq!(demo.frame_idx, 1);
        demo.seek(40);
        demo.advance(0.2);
        demo.advance(20.0);
        assert_eq!(demo.frame_idx, 40);
        demo.playing = true;
        demo.advance(20.05);
        assert_eq!(demo.frame_idx, 40, "old fractional tick survived seeking");
        demo.advance(30.0);
        assert_eq!(
            demo.frame_idx, 40,
            "a background gap fast-forwarded the replay"
        );
        demo.advance(30.1);
        assert_eq!(demo.frame_idx, 41);
        demo.seek(demo.max_frame() - 1);
        demo.playing = true;
        demo.advance(31.0);
        demo.advance(31.4);
        assert_eq!(demo.frame_idx, demo.max_frame());
        assert!(!demo.playing);
    }

    #[test]
    fn inspection_shares_validate_fields_and_ignore_other_tabs() {
        let mut demo = ControllerArenaDemo::default();
        let before = demo.share_query();
        demo.apply_share_query("tab=drone&preset=hairpin&speed=6&arena_frame=100&arena_playing=0");
        assert_eq!(demo.share_query(), before);
        demo.apply_share_query(
            "tab=arena&arena_frame=20000&arena_playing=1&arena_rate=NaN&arena_focus=unknown",
        );
        assert_eq!(demo.frame_idx, demo.max_frame());
        assert!(!demo.playing);
        assert_eq!(demo.replay_rate, 1.0);
        assert_eq!(demo.focus, ArenaControllerKind::PurePursuit);
        demo.apply_share_query("tab=arena&arena_frame=-2&arena_playing=0&arena_rate=99");
        assert_eq!(demo.frame_idx, 0);
        assert!(!demo.playing);
        assert_eq!(demo.replay_rate, 1.0);
    }

    #[test]
    fn oversized_courses_are_rejected_before_building_a_long_replay() {
        assert!(decode_course("0,0;501,0").is_none());
        let mut demo = ControllerArenaDemo {
            drawing: true,
            stroke: vec![[0.0, 0.0], [501.0, 0.0]],
            ..ControllerArenaDemo::default()
        };
        demo.finish_stroke();
        assert!(demo.drawing && demo.drawn.is_none());
        assert_eq!(
            demo.stroke_note,
            Some("Too long: keep the course under 500 m.")
        );
    }
}
