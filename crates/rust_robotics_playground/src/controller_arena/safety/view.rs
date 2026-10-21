use super::{distance, dot, Case, Constraint, Decision, Point, SafetyLab, DT, RADIUS};
use egui::{Color32, Pos2, Rect, Sense, Stroke};
const RAW: Color32 = Color32::from_rgb(255, 163, 100);
const FILTERED: Color32 = Color32::from_rgb(86, 210, 236);
const GOLD: Color32 = Color32::from_rgb(255, 211, 106);
const GREEN: Color32 = Color32::from_rgb(100, 215, 153);
const RED: Color32 = Color32::from_rgb(255, 112, 122);
fn screen(r: Rect, p: Point) -> Pos2 {
    egui::pos2(
        r.left() + ((p[0] + 6.) / 12.) as f32 * r.width(),
        r.bottom() - ((p[1] + 4.) / 8.) as f32 * r.height(),
    )
}
fn world(r: Rect, p: Pos2) -> Point {
    [
        -6. + f64::from((p.x - r.left()) / r.width()) * 12.,
        -4. + f64::from((r.bottom() - p.y) / r.height()) * 8.,
    ]
}
fn velocity_screen(r: Rect, p: Point, span: f64) -> Pos2 {
    egui::pos2(
        r.center().x + (p[0] / span) as f32 * r.width() / 2.,
        r.center().y - (p[1] / span) as f32 * r.height() / 2.,
    )
}
fn velocity_world(r: Rect, p: Pos2, span: f64) -> Point {
    [
        f64::from((p.x - r.center().x) / r.width()) * 2. * span,
        f64::from((r.center().y - p.y) / r.height()) * 2. * span,
    ]
}
fn pixel(p: Pos2) -> [f32; 2] {
    [p.x, p.y]
}
fn boundary(c: &Constraint, span: f64) -> Vec<Point> {
    let mut result = Vec::new();
    for x in [-span, span] {
        if c.gradient[1].abs() > 1e-10 {
            let y = (c.bound - c.gradient[0] * x) / c.gradient[1];
            if y.abs() <= span + 1e-9 {
                result.push([x, y]);
            }
        }
    }
    for y in [-span, span] {
        if c.gradient[0].abs() > 1e-10 {
            let x = (c.bound - c.gradient[1] * y) / c.gradient[0];
            if x.abs() <= span + 1e-9 && !result.iter().any(|&p| distance(p, [x, y]) < 1e-8) {
                result.push([x, y]);
            }
        }
    }
    result
}
impl SafetyLab {
    pub(in crate::controller_arena) fn controls(&mut self, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Safety filter lab");
        crate::ui_kit::hint(ui,"Compare a goal-seeking command with the library's CBF correction. Inspect clearance and the actual velocity constraints.");
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
            let r = ui.button(if self.playing { "Pause" } else { "Play" });
            if self.button("play", &r) {
                if self.cursor == self.last() {
                    self.cursor = 0;
                }
                self.playing = !self.playing;
                self.clock = None;
                self.pending = 0.;
            }
            for (key, label, at) in [
                ("start", "Start", 0),
                ("back", "-1", self.cursor.saturating_sub(1)),
                ("step", "+1", self.cursor + 1),
                ("end", "End", self.last()),
            ] {
                let r = ui.button(label);
                if self.button(key, &r) {
                    self.seek(at);
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            for (key, label, at) in [
                (
                    "before",
                    "Before correction",
                    self.first().saturating_sub(1),
                ),
                ("first", "First correction", self.first()),
            ] {
                let r = ui.button(label);
                if self.button(key, &r) {
                    self.seek(at);
                }
            }
        });
        ui.small(format!(
            "{:.2} / {:.2} s / dt {:.2} s",
            self.cursor as f64 * DT,
            self.last() as f64 * DT,
            DT
        ));
        let r = ui.checkbox(&mut self.future, "Show recorded future paths");
        self.button("future", &r);
        crate::ui_kit::section(ui, "Barrier gain");
        ui.horizontal_wrapped(|ui| {
            for (key, label, value) in [
                ("gentle", "Gentle", 0.8),
                ("balanced", "Balanced", 2.),
                ("firm", "Approach faster", 5.),
            ] {
                let r = ui.selectable_label(self.alpha == value, label);
                if self.button(key, &r) {
                    self.alpha = value;
                    self.cursor = 0;
                    self.record();
                }
            }
        });
        let r = ui.add(
            egui::Slider::new(&mut self.alpha, 0.5..=5.)
                .clamping(egui::SliderClamping::Edits)
                .text("alpha"),
        );
        self.widget("alpha", &r);
        if r.changed() {
            self.cursor = 0;
            self.record();
        }
        crate::ui_kit::section(ui, "Extra clearance");
        ui.horizontal_wrapped(|ui| {
            for (key, label, value) in [
                ("no_margin", "0 m", 0.),
                ("normal_margin", "0.1 m", 0.1),
                ("wide_margin", "0.4 m", 0.4),
            ] {
                let r = ui.selectable_label(self.margin == value, label);
                if self.button(key, &r) {
                    self.margin = value;
                    self.cursor = 0;
                    self.record();
                }
            }
        });
        let r = ui.add(
            egui::Slider::new(&mut self.margin, 0.0..=0.45)
                .clamping(egui::SliderClamping::Edits)
                .suffix(" m"),
        );
        self.widget("margin", &r);
        if r.changed() {
            self.cursor = 0;
            self.record();
        }
        crate::ui_kit::section(ui, "Selected obstacle");
        ui.horizontal_wrapped(|ui| {
            let r = ui.button("Previous obstacle");
            if self.button("previous_obstacle", &r) {
                self.selected = self.selected.saturating_sub(1);
            }
            let r = ui.button("Next obstacle");
            if self.button("next_obstacle", &r) {
                self.selected = (self.selected + 1).min(self.boxes.len() - 1);
            }
        });
        let d = self.decision();
        let c = d.constraints[self.selected];
        ui.small(format!(
            "Obstacle {} / centre distance {:.3} m",
            self.selected, c.distance
        ));
        ui.small(format!(
            "Nominal slack {:+.4} / applied {:+.4}",
            c.nominal_slack, c.applied_slack
        ));
        crate::ui_kit::section(ui, "Edit the map");
        ui.horizontal_wrapped(|ui| {
            for (key, label, on) in [
                ("edit_goal", "Goal", false),
                ("edit_obstacle", "Selected obstacle", true),
            ] {
                let r = ui.selectable_label(self.edit_obstacle == on, label);
                if self.button(key, &r) {
                    self.edit_obstacle = on;
                }
            }
        });
        crate::ui_kit::hint(ui,"Tap or drag on the map, then release to rebuild both recorded runs. Obstacle edits preserve its shape.");
        let r = ui.button("Reset experiment");
        if self.button("reset", &r) {
            self.alpha = 2.;
            self.margin = 0.1;
            self.future = true;
            self.edit_obstacle = false;
            self.preset(self.case);
        }
        crate::ui_kit::legend(
            ui,
            &[
                (RAW, "unfiltered"),
                (FILTERED, "CBF filtered"),
                (GOLD, "goal / correction"),
                (GREEN, "feasible velocities"),
            ],
        );
        crate::ui_kit::how_it_works(ui,"safety_help","Both recorded runs start at the same position, use the same goal-seeking controller and advance a point robot with a 0.2 m disc radius at dt=0.05 s. The library records every control actually applied. The CBF minimizes change to nominal velocity subject to outward-gradient dot velocity >= -alpha * (signed distance - radius - margin). Green in the velocity plot satisfies every displayed half-plane, clipped only to the diagram's viewing window; the box is not a speed constraint. A larger alpha allows faster approach, rather than adding clearance. Faint future paths are complete recorded runs, not online predictions. The chart and metrics use physical time, and finished runs hold their last position without inventing another command. Tap the clearance chart to seek and pause. Tap a velocity boundary to select its obstacle. The filter is reactive: a head-on approach can stop without reaching the goal. Trapped start begins with an overlapping robot disc and opposing constraints; the library's infeasible fallback can violate a barrier. The status checks every constraint independently and never labels that fallback as feasible. Editing an obstacle can also invalidate the starting clearance. This sampled single-integrator model does not represent braking dynamics. Share restores exact inputs, time, selected obstacle and overlays, paused.");
        let r = ui.button("Path-tracking comparison");
        if self.button("legacy", &r) {
            self.active = false;
            self.playing = false;
        }
    }
    pub(in crate::controller_arena) fn scene(&mut self, ui: &mut egui::Ui) {
        if ui.ctx().screen_rect().width() >= 700. {
            egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .show(ui, |ui| self.content(ui));
        } else {
            self.content(ui);
        }
        if self.playing {
            ui.ctx()
                .request_repaint_after(std::time::Duration::from_millis(16));
        }
    }
    fn content(&mut self, ui: &mut egui::Ui) {
        ui.heading("Safety filter lab");
        ui.label("How much must a command change to clear an obstacle?");
        let wide = ui.available_width() >= 650.;
        let available = ui.available_width();
        let map_width = if wide {
            (available * 0.58).min(560.)
        } else {
            available.min(((ui.ctx().screen_rect().height() - 500.).max(160.)) * 1.5)
        };
        // Integer 3:2 dimensions keep the map isotropic after egui rounds
        // widget rectangles; fractional sizes otherwise distort the disc.
        let map_width = (map_width / 3.).floor() * 3.;
        let plane_width = if wide {
            (available - map_width - 18.).min(320.)
        } else {
            available.min((ui.ctx().screen_rect().height() - 480.).clamp(160., 240.))
        };
        let plane_width = plane_width.floor();
        let mut maps = None;
        let mut planes = None;
        if wide {
            ui.horizontal(|ui| {
                let (r, response) = ui.allocate_exact_size(
                    egui::vec2(map_width, map_width * 2. / 3.),
                    Sense::click_and_drag(),
                );
                maps = Some((r, response));
                let (r, response) = ui.allocate_exact_size(
                    egui::vec2(plane_width, plane_width),
                    Sense::click_and_drag(),
                );
                planes = Some((r, response));
            });
        } else {
            ui.horizontal(|ui| {
                ui.add_space((available - map_width) / 2.);
                let (r, response) = ui.allocate_exact_size(
                    egui::vec2(map_width, map_width * 2. / 3.),
                    Sense::click_and_drag(),
                );
                maps = Some((r, response));
            });
            ui.small("Velocity constraints / tap a boundary to select its obstacle");
            ui.horizontal(|ui| {
                ui.add_space((available - plane_width) / 2.);
                let (r, response) = ui.allocate_exact_size(
                    egui::vec2(plane_width, plane_width),
                    Sense::click_and_drag(),
                );
                planes = Some((r, response));
            });
        }
        let (map, response) = maps.unwrap();
        let (plane, vresponse) = planes.unwrap();
        self.map = Some(map);
        self.velocity = Some(plane);
        self.widget("map", &response);
        self.widget("velocity", &vresponse);
        if response.drag_started() {
            if let Some(p) = response
                .ctx
                .input(|i| i.pointer.press_origin())
                .filter(|p| map.contains(*p))
            {
                self.drag = Some(world(map, p));
            }
        }
        if response.dragged() {
            if let Some(p) = response.interact_pointer_pos() {
                self.drag = Some(world(map, p));
            }
        }
        if response.drag_stopped() {
            if let Some(p) = self.drag.take() {
                self.edit([p[0].clamp(-6., 6.), p[1].clamp(-4., 4.)]);
                self.ack("map", &response.ctx);
            }
        } else if response.clicked() {
            if let Some(p) = response.interact_pointer_pos() {
                self.edit(world(map, p));
                self.ack("map", &response.ctx);
            }
        }
        if vresponse.clicked() {
            if let Some(p) = vresponse.interact_pointer_pos() {
                let d = self.decision();
                let u = velocity_world(plane, p, d.span);
                self.selected = (0..d.constraints.len())
                    .min_by(|&a, &b| {
                        (dot(d.constraints[a].gradient, u) - d.constraints[a].bound)
                            .abs()
                            .total_cmp(
                                &(dot(d.constraints[b].gradient, u) - d.constraints[b].bound).abs(),
                            )
                    })
                    .unwrap();
                self.seek(self.cursor);
                self.ack("velocity", &vresponse.ctx);
            }
        }
        let mut witnesses = self.draw_timeline(ui);
        let d = self.decision();
        self.paint_map(ui, map, &d, &mut witnesses);
        self.paint_velocity(ui, plane, &d, &mut witnesses);
        self.witnesses = format!("[{}]", witnesses.join(","));
        self.painted = Some((self.generation, self.cursor, self.selected, self.future));
        let feasible = d.constraints.iter().all(|c| c.applied_slack >= -1e-7);
        ui.colored_label(
            if feasible { GREEN } else { RED },
            if !d.has_step {
                "Run finished / no next command"
            } else if feasible {
                "Applied velocity satisfies every displayed barrier"
            } else {
                "Applied velocity violates at least one displayed barrier"
            },
        );
        ui.small(format!(
            "t {:.2} s / selected obstacle {} / alpha {:.3} / margin {:.3} m",
            self.cursor as f64 * DT,
            self.selected,
            self.alpha,
            self.margin
        ));
        for (run, color, label) in [
            (&self.runs[0], RAW, "Raw"),
            (&self.runs[1], FILTERED, "CBF"),
        ] {
            let at = self.cursor.min(run.report.steps);
            let minimum = run.clearances[..=at]
                .iter()
                .copied()
                .fold(f64::INFINITY, f64::min);
            ui.colored_label(
                color,
                format!(
                    "{label}: clearance now {:+.3} m / minimum so far {minimum:+.3} m / {}",
                    run.clearances[at],
                    if self.cursor >= run.report.steps && run.report.reached_goal {
                        "goal reached, holding"
                    } else if self.cursor >= run.report.steps {
                        "time limit, holding"
                    } else {
                        "recorded control"
                    }
                ),
            );
        }
        if self.case == Case::HeadOn {
            ui.small("Head-on: a reactive filter can stop at a face without finding a route.");
        }
        let starting_clearance = self.runs[1].clearances[0];
        if starting_clearance < 0. {
            ui.colored_label(RED, "The robot disc overlaps an obstacle at the start.");
        } else if starting_clearance < self.margin {
            ui.colored_label(
                GOLD,
                "The start is inside the requested extra clearance margin.",
            );
        }
        if self.case == Case::Trapped {
            ui.small("Opposing barriers can require incompatible velocity directions.");
        }
    }
    fn draw_timeline(&mut self, ui: &mut egui::Ui) -> Vec<String> {
        ui.small("Disc clearance / tap or drag to select physical time");
        let (r, response) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), 64.),
            Sense::click_and_drag(),
        );
        self.graph = Some(r);
        self.widget("graph", &response);
        if response.clicked() || response.dragged() || response.drag_stopped() {
            if let Some(p) = response.interact_pointer_pos() {
                self.seek(
                    (((p.x - r.left()) / r.width()).clamp(0., 1.) * self.last() as f32).round()
                        as usize,
                );
                if response.clicked() || response.drag_stopped() {
                    self.ack("graph", &response.ctx);
                }
            }
        }
        let curves = self.curves();
        let low = curves.iter().flatten().copied().fold(-0.25, f64::min);
        let high = curves.iter().flatten().copied().fold(0.3, f64::max);
        let to_screen = |i: usize, v: f64| {
            egui::pos2(
                r.left() + i as f32 / self.last().max(1) as f32 * r.width(),
                r.bottom() - 3. - ((v - low) / (high - low)) as f32 * (r.height() - 6.),
            )
        };
        let painter = ui.painter_at(r);
        painter.rect_filled(r, 4., Color32::from_rgb(10, 16, 24));
        painter.line_segment(
            [to_screen(0, 0.), to_screen(self.last(), 0.)],
            Stroke::new(1.0_f32, RED.gamma_multiply(0.5)),
        );
        let mut witnesses = vec![];
        for (run, curve) in curves.iter().enumerate() {
            let points: Vec<_> = curve
                .iter()
                .enumerate()
                .map(|(i, &v)| to_screen(i, v))
                .collect();
            if points.len() > 1 {
                painter.add(egui::Shape::line(
                    points.clone(),
                    Stroke::new(1.5_f32, if run == 0 { RAW } else { FILTERED }),
                ));
            }
            for (i, p) in points.iter().enumerate() {
                witnesses.push(format!("{{\"kind\":\"clearance\",\"run\":{run},\"index\":{i},\"value\":{},\"pixel\":{:?}}}",curve[i],pixel(*p)));
            }
        }
        let x = to_screen(self.cursor, 0.).x;
        painter.line_segment(
            [egui::pos2(x, r.top()), egui::pos2(x, r.bottom())],
            Stroke::new(1.5_f32, GOLD),
        );
        witnesses.push(format!(
            "{{\"kind\":\"cursor\",\"index\":{},\"pixel\":[{x},{}]}}",
            self.cursor,
            r.center().y
        ));
        witnesses
    }
    fn paint_map(&self, ui: &egui::Ui, r: Rect, d: &Decision, witnesses: &mut Vec<String>) {
        let p = ui.painter_at(r);
        p.rect_filled(r, 5., Color32::from_rgb(14, 22, 32));
        for x in -5..=5 {
            p.line_segment(
                [screen(r, [x as f64, -4.]), screen(r, [x as f64, 4.])],
                Stroke::new(0.5_f32, Color32::from_gray(37)),
            );
        }
        for y in -3..=3 {
            p.line_segment(
                [screen(r, [-6., y as f64]), screen(r, [6., y as f64])],
                Stroke::new(0.5_f32, Color32::from_gray(37)),
            );
        }
        for (i, o) in self.filter.obstacles.iter().enumerate() {
            let vertices = o.vertices();
            let pixels: Vec<_> = vertices.iter().map(|&q| screen(r, q)).collect();
            p.add(egui::Shape::convex_polygon(
                pixels.clone(),
                Color32::from_rgb(73, 78, 94),
                Stroke::new(
                    if i == self.selected { 2.0_f32 } else { 1.0_f32 },
                    if i == self.selected {
                        GOLD
                    } else {
                        Color32::GRAY
                    },
                ),
            ));
            for (j, &q) in vertices.iter().enumerate() {
                witnesses.push(format!("{{\"kind\":\"obstacle\",\"index\":{i},\"vertex\":{j},\"world\":{q:?},\"pixel\":{:?}}}",pixel(pixels[j])));
            }
        }
        for (i, run) in self.runs.iter().enumerate() {
            let color = if i == 0 { RAW } else { FILTERED };
            let at = self.cursor.min(run.report.steps);
            if self.future && run.report.path.len() > 1 {
                let pixels = run.report.path.iter().map(|&q| screen(r, q)).collect();
                p.add(egui::Shape::line(
                    pixels,
                    Stroke::new(1.2_f32, color.gamma_multiply(0.25)),
                ));
            }
            let points: Vec<_> = run.report.path[..=at]
                .iter()
                .map(|&q| screen(r, q))
                .collect();
            if points.len() > 1 {
                p.add(egui::Shape::line(
                    points.clone(),
                    Stroke::new(2.0_f32, color),
                ));
            }
            for (j, &q) in run.report.path.iter().enumerate() {
                if j <= at || self.future {
                    witnesses.push(format!("{{\"kind\":\"path\",\"run\":{i},\"index\":{j},\"future\":{},\"world\":{q:?},\"pixel\":{:?}}}",j>at,pixel(screen(r,q))));
                }
            }
            let q = run.position(self.cursor);
            let c = screen(r, q);
            let radius = (RADIUS / 12.) as f32 * r.width();
            p.circle_filled(c, radius, color.gamma_multiply(0.25));
            p.circle_stroke(c, radius, Stroke::new(2.0_f32, color));
            witnesses.push(format!("{{\"kind\":\"robot\",\"run\":{i},\"world\":{q:?},\"pixel\":{:?},\"radius\":{radius}}}",pixel(c)));
        }
        let pos = screen(r, d.position);
        let halo = ((RADIUS + self.margin) / 12.) as f32 * r.width();
        p.circle_stroke(
            pos,
            halo,
            Stroke::new(1.0_f32, FILTERED.gamma_multiply(0.4)),
        );
        witnesses.push(format!(
            "{{\"kind\":\"margin\",\"world\":{:?},\"pixel\":{:?},\"radius\":{halo}}}",
            d.position,
            pixel(pos)
        ));
        let goal = screen(r, self.goal);
        p.circle_stroke(goal, 7., Stroke::new(2.0_f32, GOLD));
        p.circle_filled(goal, 2., GOLD);
        witnesses.push(format!(
            "{{\"kind\":\"goal\",\"world\":{:?},\"pixel\":{:?}}}",
            self.goal,
            pixel(goal)
        ));
        if d.has_step {
            for (role, u, color) in [
                ("nominal", d.nominal, RAW),
                ("applied", d.applied, FILTERED),
            ] {
                let end = [d.position[0] + u[0] * 0.65, d.position[1] + u[1] * 0.65];
                let e = screen(r, end);
                p.arrow(pos, e - pos, Stroke::new(2.0_f32, color));
                witnesses.push(format!("{{\"kind\":\"command\",\"role\":\"{role}\",\"world\":[{:?},{end:?}],\"pixel\":[{:?},{:?}]}}",d.position,pixel(pos),pixel(e)));
            }
        }
        let c = d.constraints[self.selected];
        let closest = [
            d.position[0] - c.gradient[0] * c.distance,
            d.position[1] - c.gradient[1] * c.distance,
        ];
        p.line_segment(
            [pos, screen(r, closest)],
            Stroke::new(1.0_f32, GOLD.gamma_multiply(0.7)),
        );
        witnesses.push(format!("{{\"kind\":\"distance\",\"index\":{},\"world\":[{:?},{closest:?}],\"pixel\":[{:?},{:?}]}}",self.selected,d.position,pixel(pos),pixel(screen(r,closest))));
        if let Some(q) = self.drag {
            p.circle_stroke(screen(r, q), 9., Stroke::new(2.0_f32, GOLD));
        }
        p.text(
            r.left_top() + egui::vec2(6., 5.),
            egui::Align2::LEFT_TOP,
            "Recorded paths / velocity arrows x 0.65 s",
            egui::FontId::proportional(11.),
            Color32::LIGHT_GRAY,
        );
    }
    fn paint_velocity(&self, ui: &egui::Ui, r: Rect, d: &Decision, witnesses: &mut Vec<String>) {
        let p = ui.painter_at(r);
        p.rect_filled(r, 5., Color32::from_rgb(10, 18, 25));
        let screen = |q| velocity_screen(r, q, d.span);
        if d.feasible.len() >= 3 {
            p.add(egui::Shape::convex_polygon(
                d.feasible.iter().map(|&q| screen(q)).collect(),
                GREEN.gamma_multiply(0.14),
                Stroke::NONE,
            ));
        }
        for (i, &q) in d.feasible.iter().enumerate() {
            witnesses.push(format!(
                "{{\"kind\":\"feasible\",\"index\":{i},\"velocity\":{q:?},\"pixel\":{:?}}}",
                pixel(screen(q))
            ));
        }
        for c in [[[-d.span, 0.], [d.span, 0.]], [[0., -d.span], [0., d.span]]] {
            p.line_segment(
                [screen(c[0]), screen(c[1])],
                Stroke::new(0.8_f32, Color32::from_gray(70)),
            );
        }
        for (i, c) in d.constraints.iter().enumerate() {
            let line = boundary(c, d.span);
            if line.len() == 2 {
                p.line_segment(
                    [screen(line[0]), screen(line[1])],
                    Stroke::new(
                        if i == self.selected { 2.0_f32 } else { 1.0_f32 },
                        if i == self.selected { GOLD } else { GREEN },
                    ),
                );
                witnesses.push(format!("{{\"kind\":\"boundary\",\"index\":{i},\"velocity\":{line:?},\"pixel\":[{:?},{:?}]}}",pixel(screen(line[0])),pixel(screen(line[1]))));
            }
        }
        if d.has_step {
            for (role, u, color) in [
                ("nominal", d.nominal, RAW),
                ("applied", d.applied, FILTERED),
            ] {
                p.arrow(
                    screen([0., 0.]),
                    screen(u) - screen([0., 0.]),
                    Stroke::new(2.0_f32, color),
                );
                p.circle_filled(screen(u), 4., color);
                witnesses.push(format!("{{\"kind\":\"velocity\",\"role\":\"{role}\",\"velocity\":{u:?},\"pixel\":{:?}}}",pixel(screen(u))));
            }
            p.line_segment(
                [screen(d.nominal), screen(d.applied)],
                Stroke::new(2.0_f32, GOLD),
            );
            witnesses.push(format!(
                "{{\"kind\":\"correction\",\"velocity\":[{:?},{:?}],\"pixel\":[{:?},{:?}]}}",
                d.nominal,
                d.applied,
                pixel(screen(d.nominal)),
                pixel(screen(d.applied))
            ));
        }
        p.text(
            r.left_top() + egui::vec2(6., 5.),
            egui::Align2::LEFT_TOP,
            "Velocity / green satisfies all constraints",
            egui::FontId::proportional(10.5),
            Color32::LIGHT_GRAY,
        );
        p.text(
            r.right_bottom() - egui::vec2(5., 5.),
            egui::Align2::RIGHT_BOTTOM,
            format!("+/- {:.2} m/s / display window", d.span),
            egui::FontId::proportional(10.),
            Color32::GRAY,
        );
        if d.feasible.is_empty() {
            p.text(
                r.center(),
                egui::Align2::CENTER_CENTER,
                "No feasible velocity in this window",
                egui::FontId::proportional(12.),
                RED,
            );
        }
    }
}
