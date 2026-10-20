use super::{distance, metrics, AdmmFormationDemo, AdmmRun, CORNER, CYCLES};
use crate::admm_formation::{goal, HORIZON};
use egui::{Color32, Pos2, Rect, Sense, Shape, Stroke};

const MIN: f64 = -1.0;
const MAX: f64 = 5.5;
const OBS: Color32 = Color32::from_rgb(190, 162, 246);
const GOLD: Color32 = Color32::from_rgb(255, 213, 108);

fn screen(rect: Rect, p: [f64; 2]) -> Pos2 {
    egui::pos2(
        rect.left() + ((p[0] - MIN) / (MAX - MIN)) as f32 * rect.width(),
        rect.bottom() - ((p[1] - MIN) / (MAX - MIN)) as f32 * rect.height(),
    )
}

fn layout(area: Rect) -> Vec<Rect> {
    let wide = area.width() >= 680.0;
    let width = if wide {
        (area.width() - 12.0) / 2.0
    } else {
        area.width()
    };
    (0..2)
        .map(|i| {
            let offset = if wide {
                egui::vec2(i as f32 * (width + 12.0), 25.0)
            } else {
                egui::vec2(0.0, i as f32 * (width + 35.0) + 25.0)
            };
            Rect::from_min_size(area.min + offset, egui::vec2(width, width))
        })
        .collect()
}

/// Witnesses come from coordinates actually passed to the painter.
fn map(
    ui: &egui::Ui,
    rect: Rect,
    run: &AdmmRun,
    frame: usize,
    horizon: bool,
    observations: bool,
) -> String {
    let painter = ui.painter_at(rect);
    let f = &run.frames[frame];
    let mut witnesses = Vec::new();
    let mut point =
        |kind: &str, index: usize, p: [f64; 2], color: Color32, radius: f32, filled: bool| {
            let at = screen(rect, p);
            if filled {
                painter.circle_filled(at, radius, color);
            } else {
                painter.circle_stroke(at, radius, Stroke::new(1.3_f32, color));
            }
            witnesses.push(format!(
                "{{\"kind\":\"{kind}\",\"index\":{index},\"world\":{p:?},\"pixel\":[{},{}]}}",
                at.x, at.y
            ));
        };
    painter.rect_filled(rect, 7.0, Color32::from_rgb(17, 25, 35));
    for k in 0..=5 {
        for (a, b) in [
            ([k as f64, MIN], [k as f64, MAX]),
            ([MIN, k as f64], [MAX, k as f64]),
        ] {
            painter.line_segment(
                [screen(rect, a), screen(rect, b)],
                Stroke::new(0.5_f32, Color32::from_gray(45)),
            );
        }
    }
    let path = |points: Vec<[f64; 2]>, color: Color32, width: f32| {
        if points.len() > 1 {
            painter.add(Shape::line(
                points.into_iter().map(|p| screen(rect, p)).collect(),
                Stroke::new(width, color),
            ));
        }
    };
    path(
        (0..=CYCLES + HORIZON - 1).map(goal).collect(),
        Color32::from_gray(95),
        1.5,
    );
    path(
        run.frames[..=frame].iter().map(|f| f.center).collect(),
        run.color.gamma_multiply(0.65),
        2.0,
    );
    if let Some(plan) = &f.plan {
        if horizon {
            path(plan.report.center.clone(), run.color, 2.5);
            for (t, p) in plan.report.center.iter().enumerate() {
                point("horizon", t, *p, run.color, 2.5, true);
            }
            for (i, tr) in plan.report.trajectories.iter().enumerate() {
                let from = screen(rect, f.agents[i]);
                let to = screen(rect, tr[1]);
                painter.arrow(from, to - from, Stroke::new(1.5_f32, GOLD));
                point("command", i, tr[1], GOLD, 5.5, false);
            }
        }
        if observations {
            for (i, tr) in plan.references.iter().enumerate() {
                path(tr.clone(), OBS.gamma_multiply(0.22), 0.8);
                let to = screen(rect, tr[1]);
                let from = screen(rect, plan.report.trajectories[i][1]);
                painter.line_segment([from, to], Stroke::new(0.8_f32, OBS.gamma_multiply(0.6)));
                point("observation", i, tr[1], OBS, 4.0, false);
            }
        }
    }
    let center = screen(rect, f.center);
    for (i, p) in f.agents.iter().enumerate() {
        painter.line_segment(
            [center, screen(rect, *p)],
            Stroke::new(1.0_f32, run.color.gamma_multiply(0.45)),
        );
        point("agent", i, *p, run.color, 5.0, true);
        painter.text(
            screen(rect, *p) + egui::vec2(7.0, -10.0),
            egui::Align2::LEFT_TOP,
            format!("{}", i + 1),
            egui::FontId::proportional(11.0),
            run.color,
        );
    }
    point("center", 0, f.center, run.color, 8.0, false);
    point("goal", 0, goal(frame), Color32::WHITE, 4.0, true);
    format!("[{}]", witnesses.join(","))
}

impl AdmmFormationDemo {
    pub(in crate::admm_formation) fn inspector_controls(
        &mut self,
        _ctx: &egui::Context,
        ui: &mut egui::Ui,
    ) {
        crate::ui_kit::section(ui, "Formation lab");
        crate::ui_kit::hint(
            ui,
            "Four robots · identical observations · two smoothing costs",
        );
        crate::ui_kit::section(ui, "Observation noise / m");
        ui.horizontal_wrapped(|ui| {
            for (name, label, noise) in [
                ("clean", "0", 0.0),
                ("noisy", "0.25", 0.25),
                ("heavy", "0.50", 0.5),
            ] {
                let r = ui.selectable_label(self.noise_amp == noise, label);
                if self.inspector.button(name, &r) {
                    let frame = self.frame_idx;
                    self.noise_amp = noise;
                    self.dirty = true;
                    self.rebuild_if_needed();
                    self.seek(frame);
                }
            }
        });
        ui.small(format!("Amplitude {:.5} m per coordinate", self.noise_amp));
        crate::ui_kit::section(ui, "Replay the applied positions");
        ui.horizontal_wrapped(|ui| {
            let r = ui.button(if self.playing { "Pause" } else { "Play" });
            if self.inspector.button("play", &r) {
                if self.frame_idx == CYCLES {
                    self.frame_idx = 0;
                }
                self.playing = !self.playing;
                self.inspector.reset_clock();
            }
            for (name, label, frame) in [
                ("back", "−1", self.frame_idx.saturating_sub(1)),
                ("step", "+1", self.frame_idx + 1),
            ] {
                let r = ui.button(label);
                if self.inspector.button(name, &r) {
                    self.seek(frame);
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            for (name, label, frame) in [
                ("start", "Start", 0),
                ("before", "Before turn", CORNER - 1),
                ("corner", "At turn", CORNER),
                ("result", "Result", CYCLES),
            ] {
                let r = ui.button(label);
                if self.inspector.button(name, &r) {
                    self.seek(frame);
                }
            }
        });
        let r = ui.checkbox(&mut self.inspector.horizon, "Plan + next command");
        if self.inspector.button("horizon", &r) {
            self.playing = false;
            self.inspector.reset_clock();
        }
        let r = ui.checkbox(
            &mut self.inspector.observations,
            "Each robot's noisy reference",
        );
        if self.inspector.button("observations", &r) {
            self.playing = false;
            self.inspector.reset_clock();
        }
        crate::ui_kit::section(ui, "Applied prefix only");
        for (label, run) in [
            ("Stiff · λ = 0", &self.stiff_run),
            ("Smooth · λ = 8", &self.smooth_run),
        ] {
            let (mean, rms) = metrics(run, self.frame_idx);
            ui.colored_label(run.color, label);
            ui.label(format!("Mean tracking error: {mean:.4} m"));
            ui.label(format!("RMS 2nd difference: {rms:.4} m/step²"));
            if let Some(p) = &run.frames[self.frame_idx].plan {
                ui.small(format!(
                    "Next solve: {} iterations · primal {:.1e} · dual {:.1e}",
                    p.report.iterations,
                    p.report.primal_residuals.last().unwrap_or(&0.0),
                    p.report.dual_residuals.last().unwrap_or(&0.0)
                ));
            }
        }
        crate::ui_kit::legend(
            ui,
            &[
                (GOLD, "next command"),
                (OBS, "noisy reference"),
                (Color32::WHITE, "current goal"),
            ],
        );
        crate::ui_kit::how_it_works(ui,"formation_lab_help","Each robot observes its own noisy reference trajectory. ADMM agrees on a shared center and fixed formation offsets across a 10-point horizon. Both panels use the same observations and solver settings; only the smoothness cost changes. At step k, the plan is anchored at the current center. +1 applies the solver's point 1 directly, then re-plans. Outlined gold circles are these next commanded robot positions; purple circles are their individual references at point 1. The residual graph shows actual inner ADMM iterations of the NEXT solve, not robot motion. The top graph shows the full recorded tracking errors; applied trails and statistics stop at the selected step. The 2nd difference measures changes between successive spatial steps, not physical acceleration. This kinematic demonstration has no dynamics, obstacles or collision avoidance. Playback is 5 recorded steps/s; background gaps of at least 1 s are discarded. Share restores this exact step, noise and layers paused.");
        let r = ui.button("Overlay view");
        if self.inspector.button("overlay", &r) {
            self.inspector.active = false;
            self.playing = false;
        }
    }

    pub(in crate::admm_formation) fn inspector_scene(&mut self, ui: &mut egui::Ui) {
        ui.heading("Formation lab");
        ui.label(format!(
            "Step {} / {CYCLES} · same noise, different smoothness",
            self.frame_idx
        ));
        self.timeline(ui);
        let width = ui.available_width();
        let height = if width >= 680.0 {
            (width - 12.0) / 2.0 + 25.0
        } else {
            width * 2.0 + 60.0
        };
        let (area, _) = ui.allocate_exact_size(egui::vec2(width, height), Sense::hover());
        self.inspector.maps = layout(area);
        for (i, (label, run)) in [
            ("Stiff · λ = 0", &self.stiff_run),
            ("Smooth · λ = 8", &self.smooth_run),
        ]
        .into_iter()
        .enumerate()
        {
            let rect = self.inspector.maps[i];
            ui.painter().text(
                rect.left_top() - egui::vec2(0.0, 22.0),
                egui::Align2::LEFT_TOP,
                label,
                egui::FontId::proportional(15.0),
                run.color,
            );
            self.inspector.witnesses[i] = map(
                ui,
                rect,
                run,
                self.frame_idx,
                self.inspector.horizon,
                self.inspector.observations,
            );
        }
        self.inspector.painted = Some((
            self.frame_idx,
            self.noise_amp,
            self.inspector.horizon,
            self.inspector.observations,
        ));
        self.residuals(ui);
        crate::ui_kit::hint(
            ui,
            if self.frame_idx == CYCLES {
                "Final applied positions · no next solve"
            } else {
                "Solid centerline: next 10-point plan · gold rings: positions applied by +1"
            },
        );
    }

    fn timeline(&mut self, ui: &mut egui::Ui) {
        let (rect, r) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), 106.0),
            Sense::click_and_drag(),
        );
        let plot = Rect::from_min_max(
            rect.min + egui::vec2(32.0, 19.0),
            rect.max - egui::vec2(10.0, 22.0),
        );
        self.inspector.widget("graph", &r);
        self.inspector.graph = Some(plot);
        if r.clicked() || r.dragged() {
            if let Some(p) = r.interact_pointer_pos() {
                self.seek(
                    (((p.x - plot.left()) / plot.width()).clamp(0.0, 1.0) * CYCLES as f32).round()
                        as usize,
                );
            }
        }
        if r.clicked() || r.drag_stopped() {
            self.inspector.acknowledge("graph", &r.ctx);
        }
        let curves: [Vec<f64>; 2] = [&self.stiff_run, &self.smooth_run].map(|run| {
            run.frames
                .iter()
                .enumerate()
                .map(|(k, f)| distance(f.center, goal(k)))
                .collect()
        });
        let max = curves.iter().flatten().copied().fold(0.01_f64, f64::max) * 1.1;
        let p = |k: usize, v: f64| {
            egui::pos2(
                plot.left() + k as f32 / CYCLES as f32 * plot.width(),
                plot.bottom() - (v / max) as f32 * plot.height(),
            )
        };
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 7.0, Color32::from_rgb(17, 25, 35));
        for (curve, color) in curves
            .iter()
            .zip([self.stiff_run.color, self.smooth_run.color])
        {
            painter.add(Shape::line(
                curve.iter().enumerate().map(|(i, v)| p(i, *v)).collect(),
                Stroke::new(1.8_f32, color),
            ));
            painter.circle_filled(p(self.frame_idx, curve[self.frame_idx]), 3.0, color);
        }
        for (frame, color) in [
            (CORNER, Color32::from_gray(100)),
            (self.frame_idx, Color32::WHITE),
        ] {
            let x = p(frame, 0.0).x;
            painter.line_segment(
                [egui::pos2(x, plot.top()), egui::pos2(x, plot.bottom())],
                Stroke::new(1.0_f32, color),
            );
        }
        painter.text(
            rect.min + egui::vec2(5.0, 2.0),
            egui::Align2::LEFT_TOP,
            "Tracking error / m · full run · tap to seek",
            egui::FontId::proportional(11.0),
            Color32::LIGHT_GRAY,
        );
        for k in [0, CORNER, CYCLES] {
            painter.text(
                p(k, 0.0) + egui::vec2(0.0, 4.0),
                egui::Align2::CENTER_TOP,
                k.to_string(),
                egui::FontId::proportional(11.0),
                Color32::GRAY,
            );
        }
    }

    fn residuals(&self, ui: &mut egui::Ui) {
        let (rect, _) =
            ui.allocate_exact_size(egui::vec2(ui.available_width(), 112.0), Sense::hover());
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 7.0, Color32::from_rgb(17, 25, 35));
        painter.text(
            rect.min + egui::vec2(5.0, 2.0),
            egui::Align2::LEFT_TOP,
            "Next solve · ADMM residual (log₁₀) · bright: primal · faint: dual",
            egui::FontId::proportional(11.0),
            Color32::LIGHT_GRAY,
        );
        let plans =
            [&self.stiff_run, &self.smooth_run].map(|run| run.frames[self.frame_idx].plan.as_ref());
        let Some(a) = plans[0] else {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "Run complete · no next horizon",
                egui::FontId::proportional(13.0),
                Color32::GRAY,
            );
            return;
        };
        let b = plans[1].expect("paired plan");
        let last = a
            .report
            .iterations
            .max(b.report.iterations)
            .saturating_sub(1)
            .max(1);
        let plot = Rect::from_min_max(
            rect.min + egui::vec2(32.0, 24.0),
            rect.max - egui::vec2(12.0, 21.0),
        );
        let top = [a, b]
            .iter()
            .flat_map(|p| {
                p.report
                    .primal_residuals
                    .iter()
                    .chain(&p.report.dual_residuals)
            })
            .copied()
            .fold(1.0_f64, f64::max)
            .log10()
            .ceil();
        let point = |i: usize, v: f64| {
            egui::pos2(
                plot.left() + i as f32 / last as f32 * plot.width(),
                plot.bottom()
                    - ((v.max(1e-10).log10() + 10.0) / (top + 10.0)) as f32 * plot.height(),
            )
        };
        let y = point(0, 1e-7).y;
        painter.line_segment(
            [egui::pos2(plot.left(), y), egui::pos2(plot.right(), y)],
            Stroke::new(0.7_f32, Color32::GRAY),
        );
        painter.text(
            egui::pos2(rect.left() + 2.0, y),
            egui::Align2::LEFT_CENTER,
            "−7",
            egui::FontId::proportional(10.0),
            Color32::GRAY,
        );
        for (plan, color) in [a, b]
            .into_iter()
            .zip([self.stiff_run.color, self.smooth_run.color])
        {
            for (values, alpha) in [
                (&plan.report.primal_residuals, 1.0),
                (&plan.report.dual_residuals, 0.4),
            ] {
                painter.add(Shape::line(
                    values
                        .iter()
                        .enumerate()
                        .map(|(i, v)| point(i, *v))
                        .collect(),
                    Stroke::new(1.5_f32, color.gamma_multiply(alpha)),
                ));
            }
        }
        painter.text(
            egui::pos2(plot.left(), plot.bottom() + 4.0),
            egui::Align2::LEFT_TOP,
            "1",
            egui::FontId::proportional(10.0),
            Color32::GRAY,
        );
        painter.text(
            egui::pos2(plot.right(), plot.bottom() + 4.0),
            egui::Align2::RIGHT_TOP,
            format!("{} iterations", last + 1),
            egui::FontId::proportional(10.0),
            Color32::GRAY,
        );
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(super) fn publish_formation(&self, _ctx: &egui::Context, _active: bool) {
        let _ = &self.inspector.witnesses;
    }

    #[cfg(target_arch = "wasm32")]
    pub(super) fn publish_formation(&self, ctx: &egui::Context, active: bool) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let set = |key: &str, value: String| {
            let _ = canvas.set_attribute(&format!("data-formation-{key}"), &value);
        };
        let s = &self.inspector;
        set("active", u8::from(active).to_string());
        set("actions", format!("{:?}", s.actions));
        set("hits", format!("{:?}", s.hits));
        set("action-count", s.action_count.to_string());
        set("last-action", s.last_action.into());
        set("ui-revision", s.revision.to_string());
        set(
            "pointer",
            ctx.input(|i| i.pointer.hover_pos())
                .map_or_else(|| "null".into(), |p| format!("[{},{}]", p.x, p.y)),
        );
        if !active {
            return;
        }
        set("frame", self.frame_idx.to_string());
        set("noise", self.noise_amp.to_string());
        set("playing", u8::from(self.playing).to_string());
        set("horizon", u8::from(s.horizon).to_string());
        set("observations", u8::from(s.observations).to_string());
        set(
            "painted",
            s.painted.map_or_else(
                || "null".into(),
                |p| format!("[{}, {}, {}, {}]", p.0, p.1, p.2, p.3),
            ),
        );
        set(
            "witnesses",
            format!("[{},{}]", s.witnesses[0], s.witnesses[1]),
        );
        let rect = |r: Rect| format!("[{},{},{},{}]", r.left(), r.top(), r.width(), r.height());
        set(
            "maps",
            format!(
                "[{}]",
                s.maps
                    .iter()
                    .map(|r| rect(*r))
                    .collect::<Vec<_>>()
                    .join(",")
            ),
        );
        set("graph", s.graph.map_or_else(|| "null".into(), rect));
        set(
            "runs",
            format!(
                "[{},{}]",
                run_json(&self.stiff_run, self.frame_idx),
                run_json(&self.smooth_run, self.frame_idx)
            ),
        );
        set("share", self.share_query());
    }
}

#[cfg(target_arch = "wasm32")]
fn run_json(run: &AdmmRun, frame: usize) -> String {
    let f = &run.frames[frame];
    let (mean, rms) = metrics(run, frame);
    let history: Vec<_> = run.frames[..=frame].iter().map(|f| f.center).collect();
    let errors: Vec<_> = run
        .frames
        .iter()
        .enumerate()
        .map(|(i, f)| distance(f.center, goal(i)))
        .collect();
    let plan=f.plan.as_ref().map_or_else(||"null".into(),|p|format!("{{\"references\":{:?},\"center\":{:?},\"trajectories\":{:?},\"primal\":{:?},\"dual\":{:?},\"iterations\":{}}}",p.references,p.report.center,p.report.trajectories,p.report.primal_residuals,p.report.dual_residuals,p.report.iterations));
    format!("{{\"center\":{:?},\"agents\":{:?},\"history\":{history:?},\"errors\":{errors:?},\"mean\":{mean},\"rms\":{rms},\"plan\":{plan}}}",f.center,f.agents)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn phone_and_desktop_maps_use_equal_square_world_scales_and_actual_points() {
        for width in [296.0, 900.0] {
            let maps = layout(Rect::from_min_size(Pos2::ZERO, egui::vec2(width, 2000.0)));
            assert_eq!(maps[0].size(), maps[1].size());
            assert_eq!(maps[0].width(), maps[0].height());
            for r in maps {
                assert_eq!(screen(r, [MIN, MIN]), r.left_bottom());
                assert_eq!(screen(r, [MAX, MAX]), r.right_top());
            }
        }
    }
}
