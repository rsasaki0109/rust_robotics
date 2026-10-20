use super::{nominal_states, state_values, MppiDemo};
use crate::mppi::{
    sample_color, to_screen, MppiState2D, DT, GOAL, MOVING, OBSTACLE, PLAN, ROBOT, ROBOT_RADIUS,
    WORLD,
};
use egui::{Color32, Rect, Sense, Shape, Stroke};

const SELECTED: Color32 = Color32::from_rgb(222, 143, 255);
const CURSOR: Color32 = Color32::from_rgb(255, 220, 115);

impl MppiDemo {
    pub(in crate::mppi) fn inspector_controls(&mut self, _ctx: &egui::Context, ui: &mut egui::Ui) {
        self.ensure_inspector();
        crate::ui_kit::section(ui, "MPPI decision lab");
        crate::ui_kit::hint(
            ui,
            "One frozen solve. Select a candidate and inspect its future without moving the robot.",
        );
        ui.horizontal_wrapped(|ui| {
            let r = ui.button("Circle challenge");
            if self.inspector.button("circle", &r) {
                self.inspector_circle();
            }
            let r = ui.button("Seed 7 / 19");
            if self.inspector.button("seed", &r) {
                self.inspector.seed = if self.inspector.seed == 7 { 19 } else { 7 };
                self.rebuild();
            }
        });
        crate::ui_kit::section(ui, "Temperature λ · same sampled controls");
        ui.horizontal_wrapped(|ui| {
            for (name, lambda) in [("cold", 1.0), ("balanced", 30.0), ("warm", 100.0)] {
                let r = ui.selectable_label(self.knobs.lambda == lambda, format!("{lambda}"));
                if self.inspector.button(name, &r) {
                    self.knobs.lambda = lambda;
                    self.rebuild();
                }
            }
        });
        self.ensure_inspector();
        ui.small(format!(
            "Seed {} · {} candidates · λ {}",
            self.inspector.seed,
            self.knobs.samples + 1,
            self.knobs.lambda
        ));
        ui.horizontal_wrapped(|ui| {
            for (name, label, position) in [
                ("best", "Best", 0),
                ("worst", "Worst", self.inspector.order.len() - 1),
            ] {
                let r = ui.button(label);
                if self.inspector.button(name, &r) {
                    self.inspector.selected = self.inspector.order[position];
                }
            }
            let r = ui.button("Nominal #0");
            if self.inspector.button("nominal", &r) {
                self.inspector.selected = 0;
            }
        });
        crate::ui_kit::section(ui, "Forecast time · no robot motion");
        ui.horizontal_wrapped(|ui| {
            for (name, label, cursor) in [
                ("start", "Start", 0),
                ("back", "−1", self.inspector.cursor.saturating_sub(1)),
                (
                    "step",
                    "+1",
                    (self.inspector.cursor + 1).min(self.knobs.horizon),
                ),
                ("end", "End", self.knobs.horizon),
            ] {
                let r = ui.button(label);
                if self.inspector.button(name, &r) {
                    self.inspector.cursor = cursor;
                }
            }
        });
        let r = ui.checkbox(&mut self.show_samples, "Show all weighted candidates");
        self.inspector.button("cloud", &r);
        let r = ui.checkbox(&mut self.moving, "Predict moving obstacles");
        if self.inspector.button("moving", &r) {
            self.rebuild();
        }
        self.ensure_inspector();
        let cloud = self.cloud.as_ref().expect("inspector plan");
        let sample = &cloud.rollouts[self.inspector.selected];
        let d = cloud.plan.sampling_diagnostics;
        let at = self.inspector.cursor;
        ui.colored_label(
            SELECTED,
            format!(
                "Candidate #{} · weight {:.3}%",
                self.inspector.selected,
                100.0 * cloud.weights[self.inspector.selected]
            ),
        );
        ui.label(format!("Total cost {:.2}", sample.cost));
        ui.small(format!(
            "Goal / speed / effort: {:.2}",
            sample.model_cost - sample.constraint_cost
        ));
        ui.small(format!("Obstacle penalty: {:.2}", sample.constraint_cost));
        ui.small(format!(
            "Effective samples: {:.1} / {}",
            d.effective_sample_size, d.sample_count
        ));
        ui.label(format!(
            "Forecast +{:.1} s · point {at}/{}",
            at as f64 * DT,
            self.knobs.horizon
        ));
        if at > 0 {
            let c = sample.controls[at - 1];
            ui.small(format!(
                "Candidate acceleration: ({:.2}, {:.2}) m/s²",
                c.ax, c.ay
            ));
        }
        let c = cloud.plan.first_control;
        ui.colored_label(
            PLAN,
            format!("Next MPPI command: ({:.2}, {:.2}) m/s²", c.ax, c.ay),
        );
        crate::ui_kit::legend(
            ui,
            &[
                (SELECTED, "selected candidate"),
                (PLAN, "weighted control plan"),
                (CURSOR, "forecast position"),
                (GOAL, "goal"),
            ],
        );
        crate::ui_kit::how_it_works(ui, "mppi_inspector_help", "This view starts from the configured start with zero velocity and a fresh seeded controller. Candidate 0 is its zero-control nominal; other candidates are bounded random accelerations. Costs include goal tracking, speed, effort and obstacle penalties. Weights are exp(-(cost-min_cost)/λ), normalized to sum to one. MPPI averages CONTROL SEQUENCES, then simulates that new sequence for the green path. It does not average paths or simply choose the best candidate. Low λ concentrates weight; high λ spreads it. The temperature buttons use exactly the same seeded candidates. Candidate rank and forecast time are separate axes. Gold rings compare the selected candidate and weighted plan at the same future time. The robot and movers stay frozen. Moving-obstacle outlines use the planner's linear predictions. Soft costs and the averaged plan do not guarantee collision avoidance. Share reproduces this frozen decision, candidate and forecast time. Live control returns to a fresh paused simulation; Play starts it.");
        let r = ui.button("Live control");
        if self.inspector.button("live", &r) {
            self.inspector.active = false;
            self.restart();
            self.playing = false;
        }
    }

    pub(in crate::mppi) fn inspector_scene(&mut self, ui: &mut egui::Ui) {
        if ui.ctx().screen_rect().width() >= 700.0 {
            egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .show(ui, |ui| self.inspector_content(ui));
        } else {
            self.inspector_content(ui);
        }
    }

    fn inspector_content(&mut self, ui: &mut egui::Ui) {
        self.ensure_inspector();
        ui.heading("MPPI decision lab");
        ui.label("Which candidates shape the next command?");
        // ScrollArea has unlimited content height. Reserve room for both
        // charts using the viewport, so desktop maps do not fill the page.
        let side = ui
            .available_width()
            .min((ui.ctx().screen_rect().height() - 380.0).max(160.0));
        let rect = Rect::from_min_size(
            ui.next_widget_position() + egui::vec2((ui.available_width() - side) / 2.0, 0.0),
            egui::vec2(side, side),
        );
        ui.allocate_rect(rect, Sense::hover());
        self.inspector.map = Some(rect);
        // Both charts handle input before the corresponding paths are painted.
        self.inspector_chart(ui, false);
        self.inspector_chart(ui, true);
        let cloud = self.cloud.as_ref().expect("plan");
        let origin = self.plan_start.expect("origin");
        let selected = &cloud.rollouts[self.inspector.selected];
        let nominal = nominal_states(origin, cloud);
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 7.0, Color32::from_rgb(17, 25, 35));
        let scale = rect.width() / WORLD as f32;
        for k in (0..=20).step_by(2) {
            for (a, b) in [
                ([k as f64, 0.0], [k as f64, WORLD]),
                ([0.0, k as f64], [WORLD, k as f64]),
            ] {
                painter.line_segment(
                    [to_screen(rect, a), to_screen(rect, b)],
                    Stroke::new(0.5_f32, Color32::from_gray(45)),
                );
            }
        }
        for &(x, y, r) in &self.obstacles {
            painter.circle_filled(to_screen(rect, [x, y]), r as f32 * scale, OBSTACLE);
        }
        if self.moving {
            let t = self.inspector.cursor as f64 * DT;
            for m in &self.movers {
                let from = to_screen(rect, [m.x, m.y]);
                let to = to_screen(rect, [m.x + m.vx * t, m.y + m.vy * t]);
                painter.circle_filled(from, m.radius as f32 * scale, MOVING.gamma_multiply(0.45));
                painter.line_segment([from, to], Stroke::new(1.0_f32, MOVING));
                painter.circle_stroke(to, m.radius as f32 * scale, Stroke::new(1.5_f32, MOVING));
            }
        }
        let path = |states: &[MppiState2D], color: Color32, width: f32| {
            painter.add(Shape::line(
                states.iter().map(|s| to_screen(rect, [s.x, s.y])).collect(),
                Stroke::new(width, color),
            ));
        };
        self.inspector.cloud_drawn = 0;
        if self.show_samples {
            let max = cloud.weights.iter().copied().fold(0.0_f64, f64::max);
            for &i in self.inspector.order.iter().rev() {
                let t = (cloud.weights[i] / max).sqrt() as f32;
                path(&cloud.rollouts[i].states, sample_color(t), 0.8 + 1.2 * t);
                self.inspector.cloud_drawn += 1;
            }
        }
        let mut witnesses = Vec::new();
        for (kind, states, color) in [
            ("selected", &selected.states, SELECTED),
            ("nominal", &nominal, PLAN),
        ] {
            let points: Vec<_> = states.iter().map(|s| to_screen(rect, [s.x, s.y])).collect();
            for (index, (s, p)) in states.iter().zip(&points).enumerate() {
                witnesses.push(format!(
                    "{{\"kind\":\"{kind}\",\"index\":{index},\"state\":{:?},\"pixel\":[{},{}]}}",
                    state_values(*s),
                    p.x,
                    p.y
                ));
            }
            painter.add(Shape::line(points, Stroke::new(2.5_f32, color)));
            let s = states[self.inspector.cursor];
            let p = to_screen(rect, [s.x, s.y]);
            painter.circle_stroke(p, 7.0, Stroke::new(2.0_f32, CURSOR));
            witnesses.push(format!(
                "{{\"kind\":\"{kind}_cursor\",\"index\":{},\"state\":{:?},\"pixel\":[{},{}]}}",
                self.inspector.cursor,
                state_values(s),
                p.x,
                p.y
            ));
        }
        painter.circle_filled(to_screen(rect, self.goal), 6.0, GOAL);
        painter.circle_filled(
            to_screen(rect, [origin.x, origin.y]),
            (ROBOT_RADIUS as f32 * scale).max(4.0),
            ROBOT,
        );
        self.inspector.witnesses = format!("[{}]", witnesses.join(","));
        self.inspector.painted = Some((
            self.inspector.generation,
            self.inspector.selected,
            self.inspector.cursor,
            self.show_samples,
        ));
        crate::ui_kit::hint(ui,"Frozen robot · purple: one candidate · green: weighted controls · gold: same forecast time");
    }

    fn inspector_chart(&mut self, ui: &mut egui::Ui, time: bool) {
        let (rect, r) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), 100.0),
            Sense::click_and_drag(),
        );
        let plot = Rect::from_min_max(
            rect.min + egui::vec2(24.0, 20.0),
            rect.max - egui::vec2(10.0, 22.0),
        );
        let name = if time { "time" } else { "weights" };
        self.inspector.widget(name, &r);
        self.inspector.charts[usize::from(time)] = Some(plot);
        if r.clicked() || r.dragged() {
            if let Some(p) = r.interact_pointer_pos() {
                let f = ((p.x - plot.left()) / plot.width()).clamp(0.0, 1.0);
                if time {
                    self.inspector.cursor = (f * self.knobs.horizon as f32).round() as usize;
                } else {
                    self.inspector.selected = self.inspector.order
                        [(f * (self.inspector.order.len() - 1) as f32).round() as usize];
                }
                r.ctx.request_repaint();
            }
        }
        if r.clicked() || r.drag_stopped() {
            self.inspector.ack(name, &r.ctx);
        }
        let cloud = self.cloud.as_ref().expect("cloud");
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 7.0, Color32::from_rgb(17, 25, 35));
        let mut selected_x = plot.left();
        let title = if time {
            "Forecast time / s · tap to inspect"
        } else {
            "Weight by cost rank · tap a candidate"
        };
        painter.text(
            rect.min + egui::vec2(5.0, 2.0),
            egui::Align2::LEFT_TOP,
            title,
            egui::FontId::proportional(11.0),
            Color32::LIGHT_GRAY,
        );
        if time {
            let sample = &cloud.rollouts[self.inspector.selected];
            let nominal = nominal_states(self.plan_start.expect("origin"), cloud);
            let max = sample
                .states
                .iter()
                .chain(&nominal)
                .map(|s| s.vx.hypot(s.vy))
                .fold(0.1_f64, f64::max)
                * 1.05;
            for (states, color) in [(&sample.states, SELECTED), (&nominal, PLAN)] {
                painter.add(Shape::line(
                    states
                        .iter()
                        .enumerate()
                        .map(|(i, s)| {
                            egui::pos2(
                                plot.left() + i as f32 / self.knobs.horizon as f32 * plot.width(),
                                plot.bottom() - (s.vx.hypot(s.vy) / max) as f32 * plot.height(),
                            )
                        })
                        .collect(),
                    Stroke::new(1.5_f32, color),
                ));
            }
            selected_x += self.inspector.cursor as f32 / self.knobs.horizon as f32 * plot.width();
            painter.text(
                plot.left_bottom() + egui::vec2(0.0, 4.0),
                egui::Align2::LEFT_TOP,
                "0",
                egui::FontId::proportional(10.0),
                Color32::GRAY,
            );
            painter.text(
                plot.right_bottom() + egui::vec2(0.0, 4.0),
                egui::Align2::RIGHT_TOP,
                format!("{:.1} s · curves: speed", self.knobs.horizon as f64 * DT),
                egui::FontId::proportional(10.0),
                Color32::GRAY,
            );
        } else {
            let max = cloud.weights.iter().copied().fold(0.0_f64, f64::max);
            for (rank, &index) in self.inspector.order.iter().enumerate() {
                let x = plot.left()
                    + rank as f32 / (self.inspector.order.len() - 1) as f32 * plot.width();
                let y = plot.bottom() - (cloud.weights[index] / max) as f32 * plot.height();
                painter.line_segment(
                    [egui::pos2(x, plot.bottom()), egui::pos2(x, y)],
                    Stroke::new(
                        1.1_f32,
                        if index == self.inspector.selected {
                            SELECTED
                        } else {
                            PLAN.gamma_multiply(0.6)
                        },
                    ),
                );
                if index == self.inspector.selected {
                    selected_x = x;
                }
            }
            painter.text(
                plot.left_bottom() + egui::vec2(0.0, 4.0),
                egui::Align2::LEFT_TOP,
                "lowest cost",
                egui::FontId::proportional(10.0),
                Color32::GRAY,
            );
            painter.text(
                plot.right_bottom() + egui::vec2(0.0, 4.0),
                egui::Align2::RIGHT_TOP,
                format!("max {:.1}% · relative height", 100.0 * max),
                egui::FontId::proportional(10.0),
                Color32::GRAY,
            );
        }
        painter.line_segment(
            [
                egui::pos2(selected_x, plot.top()),
                egui::pos2(selected_x, plot.bottom()),
            ],
            Stroke::new(1.0_f32, CURSOR),
        );
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(super) fn publish_inspector(&self, _ctx: &egui::Context, _active: bool) {
        let _ = (&self.inspector.cloud_json, &self.inspector.witnesses);
    }

    #[cfg(target_arch = "wasm32")]
    pub(super) fn publish_inspector(&self, ctx: &egui::Context, active: bool) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let set = |key: &str, value: String| {
            let _ = canvas.set_attribute(&format!("data-mppi-{key}"), &value);
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
        let Some(cloud) = &self.cloud else {
            return;
        };
        let origin = self.plan_start.expect("origin");
        let r = &cloud.rollouts[s.selected];
        set("generation", s.generation.to_string());
        set("sample", s.selected.to_string());
        set("cursor", s.cursor.to_string());
        set("horizon", self.knobs.horizon.to_string());
        set("seed", s.seed.to_string());
        set("show", u8::from(self.show_samples).to_string());
        set("moving", u8::from(self.moving).to_string());
        set("cloud", s.cloud_json.clone());
        set("origin", format!("{:?}", state_values(origin)));
        set("robot", format!("{:?}", state_values(self.state)));
        set("selected",format!("{{\"cost\":{},\"model\":{},\"constraint\":{},\"terminal\":{},\"stage\":{:?},\"controls\":{:?},\"states\":{:?}}}",r.cost,r.model_cost,r.constraint_cost,r.terminal_value_cost,r.stage_costs,r.controls.iter().map(|c|[c.ax,c.ay]).collect::<Vec<_>>(),r.states.iter().map(|s|state_values(*s)).collect::<Vec<_>>()));
        set(
            "nominal",
            format!(
                "{{\"controls\":{:?},\"states\":{:?}}}",
                cloud
                    .plan
                    .nominal_controls
                    .iter()
                    .map(|c| [c.ax, c.ay])
                    .collect::<Vec<_>>(),
                nominal_states(origin, cloud)
                    .iter()
                    .map(|s| state_values(*s))
                    .collect::<Vec<_>>()
            ),
        );
        set(
            "painted",
            s.painted.map_or_else(
                || "null".into(),
                |p| format!("[{},{},{},{}]", p.0, p.1, p.2, p.3),
            ),
        );
        set("witnesses", s.witnesses.clone());
        set("drawn", s.cloud_drawn.to_string());
        let rect = |r: Rect| format!("[{},{},{},{}]", r.left(), r.top(), r.width(), r.height());
        set("map", s.map.map_or_else(|| "null".into(), rect));
        set(
            "charts",
            format!(
                "[{},{}]",
                s.charts[0].map_or_else(|| "null".into(), rect),
                s.charts[1].map_or_else(|| "null".into(), rect)
            ),
        );
        set("share", self.share_query());
    }
}
