//! Interactive replay front end for the deterministic Controller Arena engine.

use egui::{Color32, Pos2, Rect, Stroke, Vec2};
use rust_robotics_control::{
    run_controller_arena, ArenaControllerKind, ArenaPreset, ArenaRun, ArenaScenario,
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

pub struct ControllerArenaDemo {
    preset: ArenaPreset,
    target_speed: f64,
    turn_rate_response_gain: f64,
    scenario: rust_robotics_control::ArenaScenario,
    runs: Vec<ArenaRun>,
    frame_idx: usize,
    playing: bool,
    dirty: bool,
    error: Option<String>,
    /// Input time of the last replay step \[s\].
    last_advance: f64,
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
            preset,
            target_speed,
            turn_rate_response_gain,
            scenario,
            runs,
            frame_idx: 0,
            playing: true,
            dirty: false,
            error,
            last_advance: 0.0,
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
        if let Some(course) = crate::share::value(query, "course").and_then(decode_course) {
            self.drawn = Some(course);
            self.use_drawn = true;
            self.dirty = true;
        }
        let preset = crate::share::value(query, "preset")
            .and_then(ArenaPreset::from_slug)
            .unwrap_or(self.preset);
        let target_speed =
            bounded_query_f64(query, "speed", MIN_SPEED, MAX_SPEED).unwrap_or(self.target_speed);
        let response = bounded_query_f64(query, "response", MIN_RESPONSE, MAX_RESPONSE)
            .unwrap_or(self.turn_rate_response_gain);
        if preset != self.preset
            || target_speed != self.target_speed
            || response != self.turn_rate_response_gain
        {
            let (drawn, use_drawn) = (self.drawn.take(), self.use_drawn);
            *self = Self::new(preset, target_speed, response);
            self.drawn = drawn;
            self.use_drawn = use_drawn;
            self.dirty |= use_drawn;
        }
    }

    pub fn share_query(&self) -> String {
        let mut query = format!(
            "tab=arena&preset={}&speed={:.2}&response={:.2}",
            self.preset.slug(),
            self.target_speed,
            self.turn_rate_response_gain
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
        self.frame_idx = 0;
        self.playing = false;
        self.dirty = false;
    }

    fn max_frame(&self) -> usize {
        self.runs
            .iter()
            .map(|run| run.samples.len().saturating_sub(1))
            .max()
            .unwrap_or(0)
    }

    pub fn controls(&mut self, _ctx: &egui::Context, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Course");
        let selected = if self.use_drawn && self.drawn.is_some() {
            "Your course"
        } else {
            self.preset.label()
        };
        egui::ComboBox::from_id_salt("arena_preset")
            .selected_text(selected)
            .width(ui.available_width().min(240.0))
            .show_ui(ui, |ui| {
                for preset in ArenaPreset::ALL {
                    if ui
                        .selectable_label(!self.use_drawn && self.preset == preset, preset.label())
                        .clicked()
                    {
                        self.preset = preset;
                        self.use_drawn = false;
                        self.dirty = true;
                    }
                }
                if self.drawn.is_some()
                    && ui.selectable_label(self.use_drawn, "Your course").clicked()
                {
                    self.use_drawn = true;
                    self.dirty = true;
                }
            });
        ui.horizontal_wrapped(|ui| {
            if self.drawing {
                if ui.button("Cancel drawing").clicked() {
                    self.drawing = false;
                    self.stroke.clear();
                }
            } else if ui
                .button("Draw a course")
                .on_hover_text("Draw your own path and race the controllers on it")
                .clicked()
            {
                self.drawing = true;
                self.stroke.clear();
                self.stroke_note = None;
                self.playing = false;
            }
        });
        crate::ui_kit::section(ui, "Target speed");
        if ui
            .add(
                egui::Slider::new(&mut self.target_speed, MIN_SPEED..=MAX_SPEED)
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
                .step_by(0.05),
            )
            .changed()
        {
            self.dirty = true;
        }
        if ui.button("Reset").clicked() {
            let drawn = self.drawn.take();
            *self = Self::default();
            // Keep the drawing available in the course list.
            self.drawn = drawn;
        }

        crate::ui_kit::section(ui, "Result");
        self.draw_metrics(ui);
        crate::ui_kit::how_it_works(
            ui,
            "arena_help",
            "All three controllers follow the same path with the same vehicle model. \
             RMSE summarizes path error, final is the distance left to the goal, max is \
             the worst excursion, and Δω is the RMS change of the turn command (lower is \
             smoother). These compare traces under one model; they are not a universal \
             ranking.",
        );
    }

    pub fn scene(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        if self.drawing {
            self.draw_canvas(ui);
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
        let aspect = self
            .world_bounds()
            .map(|b| ((b.max_y - b.min_y) / (b.max_x - b.min_x)) as f32)
            .unwrap_or(0.6)
            .clamp(0.35, 1.2);
        let rect = crate::ui_kit::fit_rect(ui, aspect, 72.0);
        let _ = ui.allocate_rect(rect, egui::Sense::hover());
        self.draw_scene(ui, rect);

        let max_frame = self.max_frame();
        crate::ui_kit::playback(ui, &mut self.playing, &mut self.frame_idx, max_frame);
        let legend: Vec<_> = self
            .runs
            .iter()
            .map(|run| (controller_color(run.controller), run.controller.label()))
            .collect();
        crate::ui_kit::legend(ui, &legend);

        if self.playing && self.frame_idx < max_frame {
            if crate::ui_kit::every(ctx, &mut self.last_advance, 0.05) {
                self.frame_idx += 1;
            }
        } else if self.frame_idx >= max_frame {
            self.playing = false;
        }
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

        for run in &self.runs {
            let color = controller_color(run.controller);
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
                draw_robot(
                    &painter,
                    world_to_screen(rect, bounds, sample.state.x, sample.state.y),
                    sample.state.yaw,
                    color,
                );
            }
        }
    }

    fn draw_metrics(&self, ui: &mut egui::Ui) {
        egui::Grid::new("controller_arena_metrics")
            .striped(true)
            .spacing([10.0, 3.0])
            .show(ui, |ui| {
                ui.label("");
                ui.small("RMSE");
                ui.small("final");
                ui.small("max");
                ui.small("Δω");
                ui.end_row();
                for run in &self.runs {
                    ui.colored_label(controller_color(run.controller), run.controller.label());
                    ui.monospace(format!("{:.2}", run.metrics.cross_track_rmse));
                    ui.monospace(format!("{:.2}", run.metrics.final_goal_distance));
                    ui.monospace(format!("{:.2}", run.metrics.max_cross_track_error));
                    ui.monospace(format!("{:.2}", run.metrics.angular_command_smoothness));
                    ui.end_row();
                }
            });
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

/// Course key points as `x,y;x,y;...` (they are already rounded to 0.1 m).
fn encode_course(keys: &[[f64; 2]]) -> String {
    keys.iter()
        .map(|p| format!("{:.1},{:.1}", p[0], p[1]))
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
    (points.len() >= 2 && course_length(&points) >= MIN_COURSE_LENGTH).then_some(points)
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
}
