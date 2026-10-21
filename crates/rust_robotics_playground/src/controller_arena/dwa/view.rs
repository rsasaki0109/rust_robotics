use super::{array, distance, motion, point, Case, DwaLab, Edit, Point, Policy, DT, WORLD};
use egui::{Color32, Pos2, Rect, Sense, Stroke};
const CYAN: Color32 = Color32::from_rgb(86, 215, 235);
const GOLD: Color32 = Color32::from_rgb(255, 210, 106);
const RED: Color32 = Color32::from_rgb(238, 101, 123);
const GREEN: Color32 = Color32::from_rgb(115, 211, 161);
fn pixel(p: Pos2) -> [f32; 2] {
    [p.x, p.y]
}
fn screen(r: Rect, p: Point) -> Pos2 {
    egui::pos2(
        r.left() + p[0] as f32 / WORLD[0] as f32 * r.width(),
        r.bottom() - p[1] as f32 / WORLD[1] as f32 * r.height(),
    )
}
fn world(r: Rect, p: Pos2) -> Point {
    [
        f64::from((p.x - r.left()) / r.width()) * WORLD[0],
        f64::from((r.bottom() - p.y) / r.height()) * WORLD[1],
    ]
}
fn velocity(r: Rect, p: Point) -> Pos2 {
    egui::pos2(
        r.left() + (p[0] as f32 + 0.1) / 1.4 * r.width(),
        r.center().y - p[1] as f32 / 4.4 * r.height(),
    )
}
fn line(ui: &egui::Ui, points: &[Pos2], stroke: Stroke) {
    if points.len() > 1 {
        ui.painter().add(egui::Shape::line(points.to_vec(), stroke));
    }
}
impl DwaLab {
    pub(in crate::controller_arena) fn controls(&mut self, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "DWA velocity lab");
        crate::ui_kit::hint(ui,"Try a speed and turn. See why an arc is chosen or rejected, then watch the next real decision.");
        ui.horizontal_wrapped(|ui| {
            for c in Case::ALL {
                let r = ui.selectable_label(self.case == c, c.label());
                if self.button(c.slug(), &r) {
                    self.preset(c);
                }
            }
        });
        crate::ui_kit::section(ui, "Recorded decisions");
        ui.horizontal_wrapped(|ui| {
            let r = ui.button(if self.playing { "Pause" } else { "Play" });
            if self.button("play", &r) {
                if self.cursor == self.last() {
                    self.seek(0);
                }
                self.playing = !self.playing;
                self.clock = None;
                self.pending = 0.;
            }
            for (name, label, at) in [
                ("start", "Start", 0),
                ("back", "-1", self.cursor.saturating_sub(1)),
                ("step", "+1", self.cursor + 1),
                ("end", "End", self.last()),
            ] {
                let r = ui.button(label);
                if self.button(name, &r) {
                    self.seek(at);
                }
            }
        });
        ui.small(format!(
            "{} / {} decisions · {:.1} s · 5 decisions/s",
            self.cursor,
            self.last(),
            self.cursor as f64 * DT
        ));
        crate::ui_kit::section(ui, "Change the decision");
        ui.horizontal_wrapped(|ui| {
            for p in Policy::ALL {
                let r = ui.selectable_label(self.policy == p, p.label());
                if self.button(p.slug(), &r) {
                    self.policy = p;
                    self.record();
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("Look ahead");
            for (name, label, v) in [
                ("short", "1.6 s", 1.6),
                ("medium", "2.4 s", 2.4),
                ("long", "3.2 s", 3.2),
            ] {
                let r = ui.selectable_label(self.horizon == v, label);
                if self.button(name, &r) {
                    self.horizon = v;
                    self.record();
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("Robot radius");
            for (name, label, v) in [
                ("small", "0.25 m", 0.25),
                ("normal", "0.45 m", 0.45),
                ("large", "0.70 m", 0.7),
            ] {
                let r = ui.selectable_label(self.radius == v, label);
                if self.button(name, &r) {
                    self.radius = v;
                    self.record();
                }
            }
        });
        crate::ui_kit::section(ui, "Inspect a sampled arc");
        ui.horizontal_wrapped(|ui| {
            let r = ui.button("Previous");
            if self.button("prev", &r) {
                self.selected = self.selected.saturating_sub(1);
            }
            let r = ui.button("Next");
            if self.button("next", &r) {
                self.selected =
                    (self.selected + 1).min(self.trace().candidates.len().saturating_sub(1));
            }
            let r = ui.button("Chosen");
            if self.button("chosen", &r) {
                self.selected = self.trace().selected.unwrap_or(0);
            }
            let r = ui.button("Collision");
            if self.button("collision", &r) {
                if let Some(i) = self
                    .trace()
                    .candidates
                    .iter()
                    .position(|c| c.obstacle_cost.is_none())
                {
                    self.selected = i;
                }
            }
        });
        if let Some(c) = self.trace().candidates.get(self.selected) {
            ui.label(format!(
                "Arc {} · v {:.2} m/s · turn {:+.2} rad/s",
                self.selected, c.trajectory.control[0], c.trajectory.control[1]
            ));
            if c.obstacle_cost.is_some() {
                ui.small(format!(
                    "Cost {:.3} · {}",
                    c.trajectory.cost,
                    if self.trace().selected == Some(self.selected) {
                        "chosen"
                    } else {
                        "alternative"
                    }
                ));
            } else {
                ui.colored_label(RED, "Rejected: sampled footprint collision");
            }
        }
        let r = ui.checkbox(&mut self.cloud, "Show all sampled arcs");
        self.button("cloud", &r);
        crate::ui_kit::section(ui, "Edit the experiment");
        ui.horizontal_wrapped(|ui| {
            for e in Edit::ALL {
                let r = ui.selectable_label(self.edit == e, e.label());
                if self.button(e.slug(), &r) {
                    self.edit = e;
                    self.playing = false;
                    self.clock = None;
                    self.pending = 0.;
                }
            }
        });
        ui.small("Tap the map to place the goal or gold obstacle point. An edit records a fresh run. Inspect an arc on the map or velocity plot.");
        crate::ui_kit::legend(
            ui,
            &[
                (CYAN, "chosen arc"),
                (GOLD, "inspected arc / editable point"),
                (RED, "rejected collision"),
                (GREEN, "executed path"),
            ],
        );
        crate::ui_kit::how_it_works(ui,"dwa_help","DWA samples constant speed and turn rate near the current velocity, bounded by speed, yaw-rate and acceleration limits. The library rounds samples to the configured resolution and excludes each upper endpoint; rounding can place a sample slightly outside the window. A sampled arc starts at the current pose, turns first, then translates every 0.2 seconds. The library predicts while time is <= the horizon, so the last sample can extend beyond that horizon. Collision checks include the initial pose and use a circular robot against point obstacles at those discrete samples, not swept collision checking or braking-distance guarantees. Border and wall points are discrete too. For clear arcs the cost is weighted final heading error, lost speed, and reciprocal minimum center distance to any point. A smaller sum wins; the last sampled equal-cost candidate wins. The heading term measures final orientation, not distance travelled toward the goal. Rejected arcs remain rejected even with a small obstacle weight. If every arc collides, the library returns zero command; this is a fallback, not proof of a safe stop. When both sampled and current speed are nearly zero, a stuck-robot rule overrides the chosen turn to -2 rad/s, outside the usual window. The cyan one-step mark and command label show this override separately from the sampled arc. Crossing point is observed at each decision; DWA treats that position as static throughout its prediction. It has no knowledge of future motion. A replan can be late or fail. Replay shows actual applied poses and only the executed history; the candidate fan is one local prediction. Goal arrival, a blocked decision and the 120-step limit are distinct. Share restores this exact deterministic experiment paused.");
        let r = ui.button("Controller Arena");
        if self.button("legacy", &r) {
            self.active = false;
            self.playing = false;
        }
    }
    pub(in crate::controller_arena) fn scene(&mut self, ui: &mut egui::Ui) {
        if ui.ctx().screen_rect().width() >= 700. {
            egui::ScrollArea::vertical()
                .id_salt("dwa_scene")
                .show(ui, |ui| self.draw(ui));
        } else {
            self.draw(ui);
        }
        if self.playing {
            ui.ctx()
                .request_repaint_after(std::time::Duration::from_millis(16));
        }
    }
    fn draw(&mut self, ui: &mut egui::Ui) {
        ui.heading("Which way should the robot turn?");
        let title = match self.status() {
            "arrived" => "Arrived at the goal",
            "blocked" => "No collision-free sampled arc: zero-command fallback",
            "limit" => "Stopped at the 120-decision limit",
            _ => "Choose, move for 0.2 s, then observe and plan again",
        };
        ui.label(
            egui::RichText::new(title).color(if self.status() == "blocked" {
                RED
            } else {
                CYAN
            }),
        );
        let t = self.trace();
        let rejected = t
            .candidates
            .iter()
            .filter(|c| c.obstacle_cost.is_none())
            .count();
        ui.small(format!(
            "{} samples · {} rejected · decision {}",
            t.candidates.len(),
            rejected,
            self.cursor
        ));
        let width = ui.available_width().max(160.);
        let (map, r) = ui.allocate_exact_size(egui::vec2(width, width * 2. / 3.), Sense::click());
        self.map = Some(map);
        self.widget("map", &r);
        if r.clicked() {
            if let Some(p) = r.interact_pointer_pos() {
                let p = world(map, p);
                match self.edit {
                    Edit::Goal | Edit::Obstacle => {
                        let p = [
                            (p[0].clamp(0.5, 11.5) * 10.).round() / 10.,
                            (p[1].clamp(0.5, 7.5) * 10.).round() / 10.,
                        ];
                        if self.edit == Edit::Goal {
                            self.goal = p;
                        } else {
                            self.probe = p;
                        }
                        self.record();
                    }
                    Edit::Inspect => {
                        if let Some((i, d)) = self
                            .trace()
                            .candidates
                            .iter()
                            .enumerate()
                            .map(|(i, c)| {
                                (
                                    i,
                                    c.trajectory
                                        .states
                                        .iter()
                                        .map(|s| distance(point(s), p))
                                        .fold(f64::MAX, f64::min),
                                )
                            })
                            .min_by(|a, b| a.1.total_cmp(&b.1))
                        {
                            if d < 1. {
                                self.selected = i;
                            }
                        }
                    }
                }
                self.ack("map", ui.ctx());
            }
        }
        let mut ws = Vec::new();
        let painter = ui.painter_at(map);
        painter.rect_filled(map, 8., Color32::from_rgb(17, 27, 39));
        for x in 1..12 {
            painter.line_segment(
                [screen(map, [x as f64, 0.]), screen(map, [x as f64, 8.])],
                Stroke::new(1.0_f32, Color32::from_gray(40)),
            );
        }
        for y in 1..8 {
            painter.line_segment(
                [screen(map, [0., y as f64]), screen(map, [12., y as f64])],
                Stroke::new(1.0_f32, Color32::from_gray(40)),
            );
        }
        let obstacles = self.obstacles(self.cursor);
        for (i, p) in obstacles.iter().enumerate() {
            let q = screen(map, *p);
            let c = if i + 1 == obstacles.len() {
                GOLD
            } else if self.case == Case::Crossing && i + 2 == obstacles.len() {
                RED
            } else {
                Color32::from_gray(140)
            };
            painter.circle_filled(q, 3., c);
            if i >= 134 {
                painter.circle_stroke(
                    q,
                    self.radius as f32 * map.width() / 12.,
                    Stroke::new(1.0_f32, c.gamma_multiply(0.45)),
                );
            }
            ws.push(format!(
                "{{\"kind\":\"obstacle\",\"index\":{i},\"world\":{p:?},\"pixel\":{:?}}}",
                pixel(q)
            ));
        }
        let trace = self.trace();
        for (i, c) in trace.candidates.iter().enumerate() {
            let chosen = trace.selected == Some(i);
            let selected = self.selected == i;
            if !self.cloud && !chosen && !selected {
                continue;
            }
            let color = if chosen {
                CYAN
            } else if selected {
                GOLD
            } else if c.obstacle_cost.is_none() {
                RED.gamma_multiply(0.23)
            } else {
                Color32::from_rgba_unmultiplied(99, 158, 199, 48)
            };
            let pixels = c
                .trajectory
                .states
                .iter()
                .map(|s| screen(map, point(s)))
                .collect::<Vec<_>>();
            painter.add(egui::Shape::line(
                pixels.clone(),
                Stroke::new(if selected || chosen { 2.2_f32 } else { 0.8_f32 }, color),
            ));
            ws.push(format!("{{\"kind\":\"arc\",\"index\":{i},\"chosen\":{chosen},\"selected\":{selected},\"pixels\":{:?}}}",pixels.into_iter().map(pixel).collect::<Vec<_>>()));
        }
        // These are executed poses, never the unexecuted suffix of the run.
        let pixels = self.states[..=self.cursor]
            .iter()
            .map(|s| screen(map, point(s)))
            .collect::<Vec<_>>();
        line(ui, &pixels, Stroke::new(2.5_f32, GREEN));
        ws.push(format!(
            "{{\"kind\":\"history\",\"pixels\":{:?}}}",
            pixels.into_iter().map(pixel).collect::<Vec<_>>()
        ));
        let s = self.state();
        let robot = screen(map, point(s));
        let radius = self.radius as f32 * map.width() / 12.;
        painter.circle_filled(robot, radius, Color32::from_rgb(32, 82, 99));
        painter.circle_stroke(robot, radius, Stroke::new(2.0_f32, CYAN));
        let heading = screen(
            map,
            [
                s[0] + self.radius * s[2].cos(),
                s[1] + self.radius * s[2].sin(),
            ],
        );
        painter.line_segment([robot, heading], Stroke::new(2.0_f32, Color32::WHITE));
        ws.push(format!("{{\"kind\":\"robot\",\"state\":{:?},\"pixel\":{:?},\"radius\":{radius},\"heading\":{:?}}}",array(s),pixel(robot),pixel(heading)));
        let goal = screen(map, self.goal);
        painter.circle_stroke(goal, 7., Stroke::new(2.0_f32, GREEN));
        painter.line_segment(
            [goal - egui::vec2(10., 0.), goal + egui::vec2(10., 0.)],
            Stroke::new(1.0_f32, GREEN),
        );
        painter.line_segment(
            [goal - egui::vec2(0., 10.), goal + egui::vec2(0., 10.)],
            Stroke::new(1.0_f32, GREEN),
        );
        ws.push(format!("{{\"kind\":\"goal\",\"pixel\":{:?}}}", pixel(goal)));
        if self.cursor < self.last() {
            let next = motion(*self.state(), trace.applied_control, DT);
            let q = screen(map, point(&next));
            painter.line_segment([robot, q], Stroke::new(3.0_f32, CYAN));
            painter.circle_filled(q, 3., CYAN);
            ws.push(format!(
                "{{\"kind\":\"applied\",\"state\":{:?},\"pixel\":{:?}}}",
                array(&next),
                pixel(q)
            ));
        }
        let u = trace.applied_control;
        ui.small(format!(
            "{}v {:.2} m/s · turn {:+.2} rad/s{}",
            if self.cursor == self.last() {
                "Preview only: "
            } else {
                "Applied: "
            },
            u[0],
            u[1],
            if trace.stuck_override {
                " · stuck-rule override"
            } else {
                ""
            }
        ));
        let (timeline, r) = ui.allocate_exact_size(egui::vec2(width, 26.), Sense::click());
        self.timeline = Some(timeline);
        self.widget("timeline", &r);
        if r.clicked() {
            if let Some(p) = r.interact_pointer_pos() {
                let at = ((p.x - timeline.left()) / timeline.width() * self.last() as f32)
                    .round()
                    .max(0.) as usize;
                self.seek(at);
                self.ack("timeline", ui.ctx());
                ui.ctx().request_repaint();
            }
        }
        // A timeline click changes the decision after the map was painted. The
        // next frame supplies one coherent map/plot witness, never stale proof.
        if r.clicked() {
            return;
        }
        ui.painter()
            .rect_filled(timeline, 5., Color32::from_gray(34));
        let x = timeline.left() + self.cursor as f32 / self.last().max(1) as f32 * timeline.width();
        ui.painter().line_segment(
            [
                egui::pos2(x, timeline.top()),
                egui::pos2(x, timeline.bottom()),
            ],
            Stroke::new(3.0_f32, CYAN),
        );
        ui.small("Timeline: tap to seek · executed history stays green");
        ui.add_space(6.);
        ui.strong("Velocity window and actual samples");
        ui.small("Horizontal: speed [m/s] · vertical: turn [rad/s]");
        let (plot, r) = ui.allocate_exact_size(egui::vec2(width, 150.), Sense::click());
        self.velocity = Some(plot);
        self.widget("velocity", &r);
        if r.clicked() {
            if let Some(p) = r.interact_pointer_pos() {
                let i = self
                    .trace()
                    .candidates
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| {
                        velocity(plot, [a.trajectory.control[0], a.trajectory.control[1]])
                            .distance_sq(p)
                            .total_cmp(
                                &velocity(plot, [b.trajectory.control[0], b.trajectory.control[1]])
                                    .distance_sq(p),
                            )
                    })
                    .map(|(i, _)| i);
                if let Some(i) = i {
                    self.selected = i;
                }
                self.ack("velocity", ui.ctx());
                ui.ctx().request_repaint();
                return;
            }
        }
        ui.painter()
            .rect_filled(plot, 6., Color32::from_rgb(20, 28, 39));
        let t = self.trace();
        let dw = t.dynamic_window;
        let rect = Rect::from_two_pos(
            velocity(plot, [dw[0], dw[2]]),
            velocity(plot, [dw[1], dw[3]]),
        );
        let valid = dw[0] < dw[1] && dw[2] < dw[3];
        if valid {
            ui.painter()
                .rect_filled(rect, 0., CYAN.gamma_multiply(0.15));
            ui.painter().rect_stroke(
                rect,
                0.,
                Stroke::new(1.0_f32, CYAN),
                egui::StrokeKind::Inside,
            );
        }
        ws.push(format!(
            "{{\"kind\":\"window\",\"valid\":{valid},\"rect\":{}}}",
            if valid {
                format!(
                    "{:?}",
                    [rect.left(), rect.top(), rect.width(), rect.height()]
                )
            } else {
                "null".into()
            }
        ));
        for yaw in [-1.2, 0., 1.2] {
            let a = velocity(plot, [0., yaw]);
            let b = velocity(plot, [1.2, yaw]);
            ui.painter()
                .line_segment([a, b], Stroke::new(1.0_f32, Color32::from_gray(58)));
        }
        for (i, c) in t.candidates.iter().enumerate() {
            let p = velocity(plot, [c.trajectory.control[0], c.trajectory.control[1]]);
            let chosen = t.selected == Some(i);
            let selected = self.selected == i;
            let color = if chosen {
                CYAN
            } else if selected {
                GOLD
            } else if c.obstacle_cost.is_none() {
                RED
            } else {
                GREEN
            };
            ui.painter()
                .circle_filled(p, if chosen || selected { 4.5 } else { 2.5 }, color);
            ws.push(format!("{{\"kind\":\"sample\",\"index\":{i},\"pixel\":{:?},\"chosen\":{chosen},\"selected\":{selected}}}",pixel(p)));
        }
        ui.small(if valid {
            "Cyan rectangle: acceleration window · red points: rejected"
        } else {
            "Empty acceleration window: no samples; zero-command fallback"
        });
        ui.add_space(6.);
        ui.strong("Inspected cost · smaller is better");
        let (bar, _) = ui.allocate_exact_size(egui::vec2(width, 72.), Sense::hover());
        self.bars = Some(bar);
        if let Some(c) = self.trace().candidates.get(self.selected) {
            let values = [Some(c.goal_cost), Some(c.speed_cost), c.obstacle_cost];
            let scale = values.iter().flatten().copied().fold(1., f64::max);
            let colors = [CYAN, GREEN, GOLD];
            for (i, v) in values.iter().enumerate() {
                let y = bar.top() + i as f32 * 24.;
                let r = Rect::from_min_size(
                    egui::pos2(bar.left(), y + 16.),
                    egui::vec2((v.unwrap_or(0.) / scale) as f32 * bar.width(), 5.),
                );
                ui.painter().rect_filled(r, 2., colors[i]);
                let label = ["heading", "lost speed", "1 / closest center distance"][i];
                ui.painter().text(
                    egui::pos2(bar.left(), y),
                    egui::Align2::LEFT_TOP,
                    format!(
                        "{label}: {}",
                        v.map_or("COLLISION".into(), |v| format!("{v:.3}"))
                    ),
                    egui::FontId::proportional(12.),
                    Color32::from_gray(210),
                );
                ws.push(format!("{{\"kind\":\"cost\",\"term\":{i},\"value\":{},\"scale\":{scale},\"rect\":{:?}}}",v.map_or("null".into(),|v|v.to_string()),[r.left(),r.top(),r.width(),r.height()]));
            }
        }
        self.witnesses = format!("[{}]", ws.join(","));
        self.painted = Some([
            self.generation,
            self.cursor,
            self.selected,
            usize::from(self.cloud),
        ]);
    }
}
