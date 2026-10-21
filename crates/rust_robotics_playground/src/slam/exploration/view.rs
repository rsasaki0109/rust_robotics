use super::*;
use egui::{Color32, Pos2, Sense, Stroke, StrokeKind};
const BLUE: Color32 = Color32::from_rgb(91, 174, 245);
const GOLD: Color32 = Color32::from_rgb(255, 209, 105);
const GREEN: Color32 = Color32::from_rgb(90, 220, 150);
const ORANGE: Color32 = Color32::from_rgb(245, 151, 93);
const CYAN: Color32 = Color32::from_rgb(91, 215, 231);
fn screen(r: Rect, c: Cell) -> Pos2 {
    egui::pos2(
        r.left() + (c.0 as f32 + 0.5) / WIDTH as f32 * r.width(),
        r.top() + (c.1 as f32 + 0.5) / HEIGHT as f32 * r.height(),
    )
}
fn cell_at(r: Rect, p: Pos2) -> Cell {
    (
        ((p.x - r.left()) / r.width() * WIDTH as f32)
            .floor()
            .clamp(0., WIDTH as f32 - 1.) as i32,
        ((p.y - r.top()) / r.height() * HEIGHT as f32)
            .floor()
            .clamp(0., HEIGHT as f32 - 1.) as i32,
    )
}
fn color(known: u8, occupied: bool, truth: bool) -> Color32 {
    match known {
        1 => Color32::from_rgb(53, 77, 94),
        2 => Color32::from_rgb(161, 172, 184),
        _ => {
            if truth && occupied {
                Color32::from_rgb(76, 53, 63)
            } else {
                Color32::from_rgb(21, 30, 43)
            }
        }
    }
}
impl ExploreLab {
    pub(in crate::slam) fn controls(&mut self, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Frontier exploration lab");
        crate::ui_kit::hint(ui,"Explore beyond the visible map. Inspect the actual alternatives, then watch the robot uncover what is behind the wall.");
        ui.horizontal_wrapped(|ui| {
            for case in Case::ALL {
                let r = ui.selectable_label(self.case == case, case.label());
                if self.button(case.slug(), &r) {
                    self.preset(case);
                }
            }
        });
        crate::ui_kit::section(ui, "Recorded navigation");
        ui.horizontal_wrapped(|ui| {
            let r = ui.button(if self.playing {
                "Pause"
            } else {
                "Play exploration"
            });
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
        ui.horizontal_wrapped(|ui| {
            for (name, label, at) in [
                ("next_decision", "Next observation", self.next_decision()),
                ("first_wall", "First wall", self.first_wall()),
            ] {
                let r = ui.button(label);
                if self.button(name, &r) {
                    self.seek(at);
                }
            }
        });
        ui.small(format!(
            "Move {} / {} · replay 4 cells/s",
            self.cursor,
            self.last()
        ));
        let r = ui.checkbox(&mut self.truth, "Reveal hidden world");
        self.button("truth", &r);
        let r = ui.checkbox(&mut self.route, "Show remaining current plan");
        self.button("route", &r);
        crate::ui_kit::section(ui, "Choose a frontier");
        ui.horizontal_wrapped(|ui| {
            for s in Strategy::ALL {
                let r = ui.selectable_label(self.strategy == s, s.label());
                if self.button(s.slug(), &r) {
                    self.strategy = s;
                    self.record();
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("Sensor");
            for (name, label, v) in [
                ("range_short", "3 cells", 3.),
                ("range_normal", "5 cells", 5.),
                ("range_long", "7 cells", 7.),
            ] {
                let r = ui.selectable_label(self.range == v, label);
                if self.button(name, &r) {
                    self.range = v;
                    self.record();
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("Observe every");
            for (name, label, v) in [("budget_one", "1 move", 1), ("budget_three", "3 moves", 3)] {
                let r = ui.selectable_label(self.budget == v, label);
                if self.button(name, &r) {
                    self.budget = v;
                    self.record();
                }
            }
        });
        crate::ui_kit::section(ui, "Inspect an alternative");
        ui.horizontal_wrapped(|ui| {
            let r = ui.button("Previous");
            if self.button("prev_candidate", &r) {
                self.selected = self.selected.saturating_sub(1);
            }
            let r = ui.button("Next");
            if self.button("next_candidate", &r) {
                self.selected =
                    (self.selected + 1).min(self.step().candidates.len().saturating_sub(1));
            }
            let r = ui.button("Actual choice");
            if self.button("best_candidate", &r) {
                self.selected = self.step().selected.unwrap_or(0);
            }
        });
        if let Some(c) = self.candidate() {
            ui.label(format!(
                "Candidate {} · {} at {:?}",
                self.selected,
                if self.step().selected == Some(self.selected) {
                    "actual choice"
                } else {
                    "alternative"
                },
                point(c.representative)
            ));
            ui.small(c.score.map_or("Unreachable on the known map".into(), |v| {
                format!("Score {v:+.3} · {} frontier cells", c.members.len())
            }));
        } else {
            ui.small("No frontier alternatives in this phase.");
        }
        crate::ui_kit::section(ui, "Edit the experiment");
        ui.horizontal_wrapped(|ui| {
            for edit in Edit::ALL {
                let r = ui.selectable_label(self.edit == edit, edit.label());
                if self.button(edit.slug(), &r) {
                    self.edit = edit;
                    self.playing = false;
                    self.clock = None;
                    self.pending = 0.;
                }
            }
        });
        ui.small("Inspect by tapping a frontier. Draw or erase walls by dragging; this reveals the world while editing. Move start/goal to free cells. Release records a fresh run.");
        if !self.note.is_empty() {
            ui.colored_label(GOLD, self.note);
        }
        crate::ui_kit::legend(
            ui,
            &[
                (BLUE, "travelled path"),
                (GREEN, "actual choice"),
                (GOLD, "inspected frontier"),
                (ORANGE, "other frontier"),
                (CYAN, "last observation"),
            ],
        );
        crate::ui_kit::how_it_works(ui,"explore_help","The library reveals cells within sensor range along sampled lines of sight. A wall itself can be seen; it hides cells behind it. Frontiers are known-free cells next to unknown space, clustered with eight-neighbor connectivity. A cluster representative is the member closest to its centroid, with a coordinate tie break. The actual score combines goal gain minus known-free Dijkstra travel cost, plus a known-wall line-of-sight bonus and normalized unknown-neighbor count. An unknown neighbor shared by two frontier cells is counted twice; this score is a heuristic, not a prediction of newly discovered area. Unknown cells do not block the line-of-sight bonus, but motion uses known-free destination cells only. The navigator follows a decreasing distance-field gradient for the chosen move budget, then observes again. The range circle stays at the last observation position between observations. Once a known-free route to the goal exists, the library follows it without further sensing. This is an eight-neighbor point-grid model; diagonal corner crossing is allowed. The remaining-plan overlay shows only the current executed segment, not future discoveries. Replay advances actual recorded cells; no unobserved map updates are interpolated. The coverage graph shows the applied prefix. Sealed room can stop with no frontiers or no progress, and every run has a 64-decision limit. A stopped run is not presented as arrival. Revealing or editing the hidden world does not give it to the robot. Edits restart from unknown knowledge; seeking and inspecting do not replan. Shared links restore the exact world and paused cursor.");
        let r = ui.button("SLAM demos");
        if self.button("legacy", &r) {
            self.active = false;
            self.seek(self.cursor);
        }
    }
    pub(in crate::slam) fn scene(&mut self, ui: &mut egui::Ui) {
        if ui.ctx().screen_rect().width() >= 700. {
            egui::ScrollArea::vertical()
                .id_salt("explore_scene")
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
        ui.heading("What should the robot explore next?");
        let step = self.step();
        let status = match step.phase {
            FrontierNavPhase::Frontier => "Following a frontier plan",
            FrontierNavPhase::Goal => "Known route to the goal",
            FrontierNavPhase::Arrived => "Arrived at the goal",
            FrontierNavPhase::NoFrontiers => "Stopped: no frontier remains",
            FrontierNavPhase::NoReachableFrontier => "Stopped: no reachable frontier",
            FrontierNavPhase::NoProgress => "Stopped: frontier choice made no progress",
            FrontierNavPhase::IterationLimit => "Stopped: decision limit reached",
        };
        ui.label(status);
        ui.small(format!(
            "Robot {:?} · {} / {} cells observed · {:.1} cells travelled",
            point(self.robot()),
            self.coverage[self.cursor],
            WIDTH * HEIGHT,
            self.lengths[self.cursor]
        ));
        ui.small(format!(
            "Last observation {:?} · sensor {:.0} cells · decision {} / {}",
            point(step.robot),
            self.range,
            self.decision() + 1,
            self.steps.len()
        ));
        let side = (ui.available_width().min(744.) / WIDTH as f32)
            .floor()
            .max(1.);
        let (r, response) = ui.allocate_exact_size(
            egui::vec2(side * WIDTH as f32, side * HEIGHT as f32),
            Sense::click_and_drag(),
        );
        self.map = Some(r);
        self.widget("map", &response);
        if response.drag_started() {
            if let Some(p) = response
                .ctx
                .input(|i| i.pointer.press_origin())
                .filter(|p| r.contains(*p))
            {
                let c = cell_at(r, p);
                self.drag = Some((c, c));
            }
        }
        if response.dragged() {
            if let (Some((_, end)), Some(p)) = (&mut self.drag, response.interact_pointer_pos()) {
                *end = cell_at(r, p);
            }
        }
        if response.drag_stopped() {
            if let Some((a, b)) = self.drag.take() {
                if self.edit == Edit::Walls {
                    self.paint(a, b);
                } else {
                    self.set_cell(b);
                }
                self.ack("map", &response.ctx);
            }
        } else if response.clicked() {
            if let Some(p) = response.interact_pointer_pos() {
                self.set_cell(cell_at(r, p));
                self.ack("map", &response.ctx);
            }
        }
        ui.small("Dark: unknown · light wall: observed · range circle: last observation");
        let mut witnesses = self.draw_charts(ui);
        let painter = ui.painter_at(r);
        let known = flat(&self.step().known);
        let truth = self.truth_visible();
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                let c = (x as i32, y as i32);
                let i = y * WIDTH + x;
                let center = screen(r, c);
                let min = center - egui::vec2(side / 2., side / 2.);
                let rgb = color(known[i], self.world.occupied[x][y], truth);
                painter.rect_filled(
                    Rect::from_min_size(min, egui::vec2(side, side)).shrink(0.5),
                    0.,
                    rgb,
                );
                witnesses.push(format!("{{\"kind\":\"cell\",\"cell\":[{x},{y}],\"known\":{},\"occupied\":{},\"truth\":{truth},\"pixel\":{:?},\"size\":{side},\"rgb\":{:?}}}",known[i],self.world.occupied[x][y],[min.x,min.y],[rgb.r(),rgb.g(),rgb.b()]));
            }
        }
        let step = self.step();
        for (index, candidate) in step.candidates.iter().enumerate() {
            let chosen = step.selected == Some(index);
            let selected = index == self.selected;
            let color = if selected {
                GOLD
            } else if chosen {
                GREEN
            } else {
                ORANGE
            };
            for &c in &candidate.members {
                let center = screen(r, c);
                let rect = Rect::from_center_size(center, egui::vec2(side - 2., side - 2.));
                painter.rect_stroke(rect, 0., Stroke::new(1.5_f32, color), StrokeKind::Inside);
                witnesses.push(format!("{{\"kind\":\"frontier\",\"candidate\":{index},\"cell\":{:?},\"chosen\":{chosen},\"selected\":{selected},\"pixel\":{:?},\"size\":{}}}",point(c),[rect.left(),rect.top()],rect.width()));
            }
            let p = screen(r, candidate.representative);
            painter.circle_stroke(p, side * 0.24, Stroke::new(1.5_f32, color));
            painter.text(
                p,
                egui::Align2::CENTER_CENTER,
                index.to_string(),
                egui::FontId::monospace((side * 0.6).clamp(8., 13.)),
                color,
            );
            witnesses.push(format!("{{\"kind\":\"representative\",\"candidate\":{index},\"cell\":{:?},\"pixel\":{:?},\"radius\":{}}}",point(candidate.representative),[p.x,p.y],side*0.24));
        }
        let path = &self.report.as_ref().unwrap().path;
        for (at, &c) in path.iter().enumerate().take(self.cursor + 1) {
            let p = screen(r, c);
            if at > 0 {
                painter.line_segment([screen(r, path[at - 1]), p], Stroke::new(2.0_f32, BLUE));
            }
            painter.circle_filled(p, 1.7, BLUE);
            witnesses.push(format!(
                "{{\"kind\":\"history\",\"at\":{at},\"cell\":{:?},\"pixel\":{:?}}}",
                point(c),
                [p.x, p.y]
            ));
        }
        if self.route && self.plan_end() > self.cursor {
            for at in self.cursor..=self.plan_end() {
                let c = path[at];
                let p = screen(r, c);
                if at > self.cursor {
                    painter.line_segment(
                        [screen(r, path[at - 1]), p],
                        Stroke::new(2.0_f32, BLUE.gamma_multiply(0.4)),
                    );
                }
                painter.circle_stroke(p, 2.5, Stroke::new(1.0_f32, BLUE));
                witnesses.push(format!(
                    "{{\"kind\":\"plan\",\"at\":{at},\"cell\":{:?},\"pixel\":{:?}}}",
                    point(c),
                    [p.x, p.y]
                ));
            }
        }
        let anchor = screen(r, step.robot);
        let radius = self.range as f32 * side;
        painter.circle_stroke(
            anchor,
            radius,
            Stroke::new(0.8_f32, CYAN.gamma_multiply(0.5)),
        );
        painter.circle_stroke(anchor, 4., Stroke::new(1.0_f32, CYAN));
        witnesses.push(format!(
            "{{\"kind\":\"sensor\",\"cell\":{:?},\"pixel\":{:?},\"radius\":{radius}}}",
            point(step.robot),
            [anchor.x, anchor.y]
        ));
        let goal = screen(r, self.world.goal);
        painter.rect_stroke(
            Rect::from_center_size(goal, egui::vec2(side * 0.55, side * 0.55)),
            0.,
            Stroke::new(2.0_f32, GREEN),
            StrokeKind::Inside,
        );
        witnesses.push(format!(
            "{{\"kind\":\"goal\",\"cell\":{:?},\"pixel\":{:?},\"size\":{}}}",
            point(self.world.goal),
            [goal.x, goal.y],
            side * 0.55
        ));
        let robot = screen(r, self.robot());
        painter.circle_filled(robot, side * 0.31, BLUE);
        painter.circle_stroke(robot, side * 0.31, Stroke::new(1.5_f32, Color32::WHITE));
        witnesses.push(format!(
            "{{\"kind\":\"robot\",\"cell\":{:?},\"pixel\":{:?},\"radius\":{}}}",
            point(self.robot()),
            [robot.x, robot.y],
            side * 0.31
        ));
        if let Some((a, b)) = self.drag {
            for c in line(a, b) {
                painter.rect_stroke(
                    Rect::from_center_size(screen(r, c), egui::vec2(side, side)),
                    0.,
                    Stroke::new(2.0_f32, GOLD),
                    StrokeKind::Inside,
                );
            }
        }
        self.witnesses = format!("[{}]", witnesses.join(","));
        self.painted = Some((
            self.generation,
            self.cursor,
            self.selected,
            self.truth,
            self.route,
            self.edit,
        ));
    }
    fn draw_charts(&mut self, ui: &mut egui::Ui) -> Vec<String> {
        ui.label("Observed map / tap or drag to seek");
        let (g, response) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), 72.),
            Sense::click_and_drag(),
        );
        self.graph = Some(g);
        self.widget("graph", &response);
        if response.clicked() || response.dragged() || response.drag_stopped() {
            if let Some(p) = response.interact_pointer_pos() {
                self.seek(
                    (((p.x - g.left()) / g.width()).clamp(0., 1.) * self.last() as f32).round()
                        as usize,
                );
                if response.clicked() || response.drag_stopped() {
                    self.ack("graph", &response.ctx);
                }
            }
        }
        ui.label("Frontier scores / tap an alternative");
        let n = self.step().candidates.len();
        let (c, response) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), (n as f32 * 22.).clamp(44., 176.)),
            Sense::click(),
        );
        self.candidates = Some(c);
        self.widget("candidates", &response);
        if response.clicked() && n > 0 {
            if let Some(p) = response.interact_pointer_pos() {
                self.selected =
                    (((p.y - c.top()) / c.height()).clamp(0., 0.999999) * n as f32) as usize;
                self.playing = false;
                self.clock = None;
                self.pending = 0.;
                self.ack("candidates", &response.ctx);
            }
        }
        ui.label("Inspected score: goal / travel / sight / discovery");
        let (terms, _) =
            ui.allocate_exact_size(egui::vec2(ui.available_width(), 80.), Sense::hover());
        self.components = Some(terms);
        let painter = ui.painter_at(g);
        painter.rect_filled(g, 4., Color32::from_rgb(13, 21, 31));
        let project = |at: usize, count: usize| {
            egui::pos2(
                g.left() + at as f32 / self.last().max(1) as f32 * g.width(),
                g.bottom() - count as f32 / (WIDTH * HEIGHT) as f32 * g.height(),
            )
        };
        let mut witnesses = Vec::new();
        for at in 0..=self.cursor {
            let p = project(at, self.coverage[at]);
            if at > 0 {
                painter.line_segment(
                    [project(at - 1, self.coverage[at - 1]), p],
                    Stroke::new(1.5_f32, BLUE),
                );
            }
            witnesses.push(format!(
                "{{\"kind\":\"coverage\",\"at\":{at},\"count\":{},\"pixel\":{:?}}}",
                self.coverage[at],
                [p.x, p.y]
            ));
        }
        let p = project(self.cursor, self.coverage[self.cursor]);
        painter.circle_filled(p, 4., GOLD);
        painter.line_segment(
            [egui::pos2(p.x, g.top()), egui::pos2(p.x, g.bottom())],
            Stroke::new(1.0_f32, GOLD),
        );
        witnesses.push(format!(
            "{{\"kind\":\"cursor\",\"at\":{},\"pixel\":{:?}}}",
            self.cursor,
            [p.x, p.y]
        ));
        let painter = ui.painter_at(c);
        painter.rect_filled(c, 4., Color32::from_rgb(13, 21, 31));
        let scale = self
            .step()
            .candidates
            .iter()
            .filter_map(|a| a.score)
            .map(f64::abs)
            .fold(1., f64::max);
        let zero = c.center().x;
        painter.line_segment(
            [egui::pos2(zero, c.top()), egui::pos2(zero, c.bottom())],
            Stroke::new(0.75_f32, Color32::GRAY),
        );
        for (index, a) in self.step().candidates.iter().enumerate() {
            let y = c.top() + (index as f32 + 0.5) / n as f32 * c.height();
            let width = (a.score.unwrap_or(0.) / scale) as f32 * c.width() * 0.45;
            let rect = Rect::from_min_max(
                egui::pos2(zero.min(zero + width), y - 5.),
                egui::pos2(zero.max(zero + width), y + 5.),
            );
            painter.rect_filled(
                rect,
                1.,
                if index == self.selected {
                    GOLD
                } else if self.step().selected == Some(index) {
                    GREEN
                } else {
                    ORANGE
                },
            );
            painter.text(
                egui::pos2(c.left() + 3., y),
                egui::Align2::LEFT_CENTER,
                index.to_string(),
                egui::FontId::monospace(10.),
                Color32::WHITE,
            );
            witnesses.push(format!("{{\"kind\":\"score\",\"candidate\":{index},\"value\":{},\"pixel\":{:?},\"scale\":{scale}}}",a.score.map_or("null".into(),|v|v.to_string()),[rect.left(),rect.top(),rect.width(),rect.height()]));
        }
        let painter = ui.painter_at(terms);
        painter.rect_filled(terms, 4., Color32::from_rgb(13, 21, 31));
        if let Some(values) = self.terms() {
            let scale = values.iter().copied().map(f64::abs).fold(1., f64::max);
            let zero = terms.center().x;
            painter.line_segment(
                [
                    egui::pos2(zero, terms.top()),
                    egui::pos2(zero, terms.bottom()),
                ],
                Stroke::new(0.75_f32, Color32::GRAY),
            );
            for (index, value) in values.into_iter().enumerate() {
                let y = terms.top() + (index as f32 + 0.5) * 20.;
                let width = (value / scale) as f32 * terms.width() * 0.45;
                let rect = Rect::from_min_max(
                    egui::pos2(zero.min(zero + width), y - 5.),
                    egui::pos2(zero.max(zero + width), y + 5.),
                );
                painter.rect_filled(rect, 1., [GREEN, ORANGE, CYAN, GOLD][index]);
                painter.text(
                    egui::pos2(terms.left() + 3., y),
                    egui::Align2::LEFT_CENTER,
                    ["goal", "travel", "sight", "discovery"][index],
                    egui::FontId::monospace(10.),
                    Color32::WHITE,
                );
                witnesses.push(format!("{{\"kind\":\"component\",\"component\":{index},\"value\":{value},\"pixel\":{:?},\"scale\":{scale}}}",[rect.left(),rect.top(),rect.width(),rect.height()]));
            }
        } else {
            painter.text(
                terms.center(),
                egui::Align2::CENTER_CENTER,
                "No reachable frontier selected",
                egui::FontId::proportional(12.),
                Color32::GRAY,
            );
        }
        witnesses
    }
}
