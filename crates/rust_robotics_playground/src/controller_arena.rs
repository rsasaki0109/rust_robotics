//! Interactive replay front end for the deterministic Controller Arena engine.

use egui::{Color32, Pos2, Rect, Stroke, Vec2};
use rust_robotics_control::{run_controller_arena, ArenaControllerKind, ArenaPreset, ArenaRun};

const DEFAULT_SPEED: f64 = 3.0;
const DEFAULT_RESPONSE: f64 = 0.85;
const MIN_SPEED: f64 = 1.0;
const MAX_SPEED: f64 = 6.0;
const MIN_RESPONSE: f64 = 0.4;
const MAX_RESPONSE: f64 = 1.0;

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
            playing: false,
            dirty: false,
            error,
        }
    }

    pub fn apply_share_query(&mut self, query: &str) {
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
            *self = Self::new(preset, target_speed, response);
        }
    }

    pub fn share_query(&self) -> String {
        format!(
            "tab=arena&preset={}&speed={:.2}&response={:.2}",
            self.preset.slug(),
            self.target_speed,
            self.turn_rate_response_gain
        )
    }

    fn rebuild(&mut self) {
        self.scenario = self
            .preset
            .scenario(self.target_speed, self.turn_rate_response_gain);
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

    pub fn ui(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.label("Path:");
            egui::ComboBox::from_id_salt("arena_preset")
                .selected_text(self.preset.label())
                .show_ui(ui, |ui| {
                    for preset in ArenaPreset::ALL {
                        if ui
                            .selectable_value(&mut self.preset, preset, preset.label())
                            .changed()
                        {
                            self.dirty = true;
                        }
                    }
                });
            ui.separator();
            if ui
                .add(
                    egui::Slider::new(&mut self.target_speed, MIN_SPEED..=MAX_SPEED)
                        .text("Target speed (m/s)")
                        .step_by(0.25),
                )
                .changed()
            {
                self.dirty = true;
            }
            if ui
                .add(
                    egui::Slider::new(
                        &mut self.turn_rate_response_gain,
                        MIN_RESPONSE..=MAX_RESPONSE,
                    )
                    .text("Turn response")
                    .step_by(0.05),
                )
                .changed()
            {
                self.dirty = true;
            }
            ui.separator();
            if ui.button("Run").clicked() {
                self.rebuild();
                self.playing = true;
            }
            if ui
                .add_enabled(
                    !self.dirty,
                    egui::Button::new(if self.playing { "Pause" } else { "Play" }),
                )
                .clicked()
            {
                self.playing = !self.playing;
            }
            if ui
                .add_enabled(!self.dirty, egui::Button::new("Step"))
                .clicked()
            {
                self.playing = false;
                self.frame_idx = (self.frame_idx + 1).min(self.max_frame());
            }
            if ui.button("Reset").clicked() {
                *self = Self::default();
            }
            if self.dirty {
                ui.colored_label(Color32::YELLOW, "settings changed — press Run");
            }
        });

        let max_frame = self.max_frame();
        ui.horizontal(|ui| {
            ui.label(format!("Frame {}/{}", self.frame_idx, max_frame));
            ui.add(egui::Slider::new(&mut self.frame_idx, 0..=max_frame).text("timeline"));
        });

        if let Some(error) = &self.error {
            ui.colored_label(Color32::LIGHT_RED, format!("Arena error: {error}"));
        }

        let width = ui.available_width().max(320.0);
        let height = (ui.available_height() - 155.0).clamp(260.0, 520.0);
        let (rect, _) = ui.allocate_exact_size(Vec2::new(width, height), egui::Sense::hover());
        self.draw_scene(ui, rect);

        ui.add_space(6.0);
        self.draw_metrics(ui);
        ui.label(
            "RMSE summarizes path error; final error measures goal accuracy; max error exposes \
             the worst excursion; Δω RMS measures command smoothness. These are comparative \
             traces under one model—not a universal controller ranking.",
        );

        if self.playing && self.frame_idx < max_frame {
            self.frame_idx += 1;
            ctx.request_repaint_after(std::time::Duration::from_millis(50));
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
            .spacing([16.0, 3.0])
            .show(ui, |ui| {
                ui.strong("Controller");
                ui.strong("RMSE (m)");
                ui.strong("Final (m)");
                ui.strong("Max (m)");
                ui.strong("Δω RMS (rad/s)");
                ui.end_row();
                for run in &self.runs {
                    ui.colored_label(controller_color(run.controller), run.controller.label());
                    ui.monospace(format!("{:.3}", run.metrics.cross_track_rmse));
                    ui.monospace(format!("{:.3}", run.metrics.final_goal_distance));
                    ui.monospace(format!("{:.3}", run.metrics.max_cross_track_error));
                    ui.monospace(format!("{:.3}", run.metrics.angular_command_smoothness));
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

    #[test]
    fn invalid_share_values_fall_back_without_panicking() {
        let mut demo = ControllerArenaDemo::default();
        demo.apply_share_query("tab=arena&preset=unknown&speed=nan&response=4.0");
        assert_eq!(demo.preset, ArenaPreset::SlalomRecovery);
        assert_eq!(demo.target_speed, DEFAULT_SPEED);
        assert_eq!(demo.turn_rate_response_gain, DEFAULT_RESPONSE);
    }
}
