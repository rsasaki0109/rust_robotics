use super::{Case, Cell, TimeLab, GATE, GOAL, HEIGHT, HORIZON, START, TICK_SECONDS, WIDTH};
use egui::{Color32, Pos2, Rect, Sense, Stroke};
const RAW: Color32 = Color32::from_rgb(255, 163, 100);
const AWARE: Color32 = Color32::from_rgb(86, 210, 236);
const GOLD: Color32 = Color32::from_rgb(255, 211, 106);
const GREEN: Color32 = Color32::from_rgb(100, 215, 153);
const RED: Color32 = Color32::from_rgb(255, 112, 122);
const MUTED: Color32 = Color32::from_rgb(77, 88, 106);

fn screen(r: Rect, p: Cell) -> Pos2 {
    r.min
        + egui::vec2(
            (p[0] as f32 + 0.5) * r.width() / WIDTH as f32,
            (p[1] as f32 + 0.5) * r.height() / HEIGHT as f32,
        )
}
fn cell_at(r: Rect, p: Pos2) -> Option<Cell> {
    if !r.contains(p) {
        return None;
    }
    let cell = [
        ((p.x - r.left()) / r.width() * WIDTH as f32).floor() as usize,
        ((p.y - r.top()) / r.height() * HEIGHT as f32).floor() as usize,
    ];
    (cell[0] < WIDTH && cell[1] < HEIGHT).then_some(cell)
}
fn phase(s: &TimeLab, run: usize, t: usize) -> &'static str {
    let r = &s.runs[run];
    let Some(arrival) = r.arrival else {
        return "no plan";
    };
    if r.conflicts.contains(&t) {
        "conflict"
    } else if t >= arrival {
        "arrived"
    } else if r.waiting(t) {
        "wait"
    } else {
        "move"
    }
}
fn phase_color(p: &str, run: usize) -> Color32 {
    match p {
        "no plan" => MUTED,
        "conflict" => RED,
        "arrived" => GREEN,
        "wait" => GOLD,
        _ => {
            if run == 0 {
                RAW
            } else {
                AWARE
            }
        }
    }
}
impl TimeLab {
    pub(in crate::grid_planners) fn controls(&mut self, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Time-aware routes");
        crate::ui_kit::hint(
            ui,
            "A short closure can make waiting faster. A longer one can make a detour win.",
        );
        ui.horizontal_wrapped(|ui| {
            for c in Case::ALL {
                let r = ui.selectable_label(self.case == c, c.label());
                if self.button(c.slug(), &r) {
                    self.preset(c);
                }
            }
        });
        crate::ui_kit::section(ui, "One clock, two routes");
        ui.horizontal_wrapped(|ui| {
            let r = ui.button(if self.playing { "Pause" } else { "Play" });
            if self.button("play", &r) {
                if self.cursor == HORIZON {
                    self.cursor = 0;
                }
                self.playing = !self.playing;
                self.clock = None;
                self.pending = 0.;
            }
            for (key, label, t) in [
                ("start", "Start", 0),
                ("back", "-1", self.cursor.saturating_sub(1)),
                ("step", "+1", self.cursor + 1),
                ("end", "End", HORIZON),
            ] {
                let r = ui.button(label);
                if self.button(key, &r) {
                    self.seek(t);
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            for (key, label, t) in [
                ("before", "Before change", self.first().saturating_sub(1)),
                ("first", "First change", self.first()),
            ] {
                let r = ui.button(label);
                if self.button(key, &r) {
                    self.seek(t);
                }
            }
        });
        ui.small(format!(
            "Tick {} / {HORIZON} · {:.1} s per tick",
            self.cursor, TICK_SECONDS
        ));
        let r = ui.checkbox(&mut self.future, "Show planned future");
        self.button("future", &r);
        crate::ui_kit::section(ui, "Gate reservation");
        ui.label(format!(
            "Closed [{}, {}) ticks",
            self.close,
            self.close + self.duration
        ));
        ui.horizontal_wrapped(|ui| {
            for (key, label, close) in [
                ("earlier", "Close earlier", self.close.saturating_sub(1)),
                ("later", "Close later", (self.close + 1).min(12)),
            ] {
                let r = ui.button(label);
                if self.button(key, &r) {
                    self.close = close;
                    self.cursor = 0;
                    self.record();
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            for (key, label, duration) in [
                ("shorter", "Shorter", self.duration.saturating_sub(1).max(1)),
                ("longer", "Longer", (self.duration + 1).min(25)),
            ] {
                let r = ui.button(label);
                if self.button(key, &r) {
                    self.duration = duration;
                    self.cursor = 0;
                    self.record();
                }
            }
            let r = ui.button("Preset timing");
            if self.button("normal_schedule", &r) {
                self.close = 5;
                self.duration = self.case.duration();
                self.cursor = 0;
                self.record();
            }
        });
        crate::ui_kit::section(ui, "Whole planned routes");
        for i in 0..2 {
            let r = &self.runs[i];
            let color = if i == 0 { RAW } else { AWARE };
            ui.colored_label(
                color,
                if i == 0 {
                    "Time-unaware"
                } else {
                    "SIPP · time-aware"
                },
            );
            if let Some(t) = r.arrival {
                ui.small(format!(
                    "Arrival {t} · {} moves · {} waits",
                    r.moves, r.waits
                ));
                ui.small(format!("{} occupied-cell conflicts", r.conflicts.len()));
                let (moves, waits) = r.observed(self.cursor);
                ui.small(format!(
                    "Completed by tick {}: {moves} moves, {waits} waits",
                    self.cursor
                ));
            } else {
                ui.colored_label(RED, "No route within 24 ticks")
                    .on_hover_text(r.error.as_deref().unwrap_or("No plan returned"));
            }
        }
        crate::ui_kit::section(ui, "Inspect or edit");
        ui.horizontal_wrapped(|ui| {
            for (key, label, edit) in [
                ("select", "Inspect cell", false),
                ("walls", "Draw walls", true),
            ] {
                let r = ui.selectable_label(self.edit_walls == edit, label);
                if self.button(key, &r) {
                    self.edit_walls = edit;
                }
            }
        });
        crate::ui_kit::hint(ui,"Tap a cell for its safe intervals. In Draw walls, tap or drag to paint; release to replan both routes.");
        let r = ui.button("Reset experiment");
        if self.button("reset", &r) {
            self.future = true;
            self.edit_walls = false;
            self.preset(self.case);
        }
        crate::ui_kit::legend(
            ui,
            &[
                (RAW, "time-unaware"),
                (AWARE, "SIPP"),
                (GOLD, "waiting"),
                (RED, "conflict"),
                (GREEN, "arrived / free interval"),
            ],
        );
        crate::ui_kit::how_it_works(ui,"time_help","Both routes use the library SIPP planner on the same four-neighbor grid: each move takes one tick. Time-unaware ignores the gate's schedule; time-aware uses its exact known reservation. A reservation [a,b) includes a and excludes b. Compressed timed waypoints can contain waits: the robot stays in the previous cell until one tick before the next arrival. The map shows integer-tick cell occupancy, not continuous swept-body motion or uncertain predictions. A route must arrive within 24 ticks. Finished routes hold the goal, which is never the reserved gate. No route displays no invented robot trajectory. The selected cell's green intervals come directly from SIPP; the two lower rows show moving, waiting, conflicts and arrival on the same clock. Pale marks are a complete recorded plan, rather than a forecast. Metrics separate the whole route from transitions completed by the cursor. Share reopens the same schedule, map, cursor, selected cell and overlays, paused.");
        let r = ui.button("Static grid comparison");
        if self.button("legacy", &r) {
            self.active = false;
            self.playing = false;
        }
    }
    pub(in crate::grid_planners) fn scene(&mut self, ui: &mut egui::Ui) {
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
        ui.heading("Wait or take the detour?");
        ui.label("One map. One schedule. Two decisions.");
        ui.horizontal_wrapped(|ui| {
            for i in 0..2 {
                ui.colored_label(
                    phase_color(phase(self, i, self.cursor), i),
                    format!(
                        "{}: {}",
                        if i == 0 { "Time-unaware" } else { "SIPP" },
                        phase(self, i, self.cursor)
                    ),
                );
            }
        });
        // Integer cell dimensions preserve a square grid after egui pixel rounding.
        let cell = (ui.available_width().min(650.) / WIDTH as f32).floor();
        let size = egui::vec2(cell * WIDTH as f32, cell * HEIGHT as f32);
        let mut region = None;
        ui.horizontal(|ui| {
            ui.add_space(((ui.available_width() - size.x) / 2.).max(0.));
            region = Some(ui.allocate_exact_size(size, Sense::click_and_drag()));
        });
        let (map, response) = region.unwrap();
        self.map_rect = Some(map);
        self.widget("map", &response);
        self.map_input(map, &response);
        ui.small(format!(
            "Selected cell ({}, {}) · gate ({}, {}) closed [{}, {})",
            self.selected[0],
            self.selected[1],
            GATE[0],
            GATE[1],
            self.close,
            self.close + self.duration
        ));
        ui.label("Selected cell · green safe intervals · dots are route visits");
        let (band, response) = ui.allocate_exact_size(
            egui::vec2(ui.available_width().floor(), 48.),
            Sense::click_and_drag(),
        );
        self.band = Some(band);
        self.widget("band", &response);
        self.time_input(band, &response, "band");
        ui.label("Time-unaware (top) / SIPP (bottom) · status at each tick");
        let (graph, response) = ui.allocate_exact_size(
            egui::vec2(ui.available_width().floor(), 72.),
            Sense::click_and_drag(),
        );
        self.graph = Some(graph);
        self.widget("graph", &response);
        self.time_input(graph, &response, "graph");
        ui.small(
            "Tap or drag either time strip to seek and pause. Pale colors show the planned future.",
        );
        self.paint(ui, map, band, graph);
        self.painted = Some((self.generation, self.cursor, self.selected, self.future));
    }
    fn map_input(&mut self, map: Rect, r: &egui::Response) {
        if r.drag_started() {
            self.drag = r
                .ctx
                .input(|i| i.pointer.press_origin())
                .and_then(|p| cell_at(map, p));
        }
        if r.clicked() {
            if let Some(p) = r.interact_pointer_pos().and_then(|p| cell_at(map, p)) {
                self.edit(p, p);
                self.ack("map", &r.ctx);
            }
        }
        if r.drag_stopped() {
            if let (Some(from), Some(to)) = (
                self.drag.take(),
                r.interact_pointer_pos().and_then(|p| cell_at(map, p)),
            ) {
                self.edit(from, to);
                self.ack("map", &r.ctx);
            }
        }
    }
    fn time_input(&mut self, rect: Rect, r: &egui::Response, key: &'static str) {
        if (r.clicked() || r.dragged() || r.drag_stopped()) && r.interact_pointer_pos().is_some() {
            let p = r.interact_pointer_pos().unwrap();
            self.seek(
                (((p.x - rect.left()) / rect.width()).clamp(0., 1.) * (HORIZON + 1) as f32).floor()
                    as usize,
            );
        }
        if r.clicked() || r.drag_stopped() {
            self.ack(key, &r.ctx);
        }
    }
    fn paint(&mut self, ui: &egui::Ui, map: Rect, band: Rect, graph: Rect) {
        let painter = ui.painter();
        let mut witnesses = vec![];
        let cell = map.width() / WIDTH as f32;
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                let p = [x, y];
                let center = screen(map, p);
                let r = Rect::from_center_size(center, egui::Vec2::splat(cell));
                let color = if self.map[x][y] {
                    Color32::from_rgb(57, 67, 83)
                } else if p == GATE {
                    if self.reserved(p, self.cursor) {
                        RED.gamma_multiply(0.65)
                    } else {
                        GREEN.gamma_multiply(0.4)
                    }
                } else {
                    Color32::from_rgb(20, 29, 40)
                };
                painter.rect_filled(r, 0., color);
                painter.rect_stroke(
                    r,
                    0.,
                    Stroke::new(0.6_f32, Color32::from_rgb(43, 54, 70)),
                    egui::StrokeKind::Inside,
                );
                witnesses.push(format!("{{\"kind\":\"cell\",\"cell\":{p:?},\"wall\":{},\"reserved\":{},\"pixel\":{:?},\"size\":{cell}}}",self.map[x][y],self.reserved(p,self.cursor),[r.left(),r.top()]));
            }
        }
        for (run, r) in self.runs.iter().enumerate() {
            let color = if run == 0 { RAW } else { AWARE };
            let end = if self.future { HORIZON } else { self.cursor };
            for (t, &p) in r.positions.iter().enumerate().take(end + 1) {
                let point = screen(map, p);
                let future = t > self.cursor;
                if t > 0 {
                    painter.line_segment(
                        [screen(map, r.positions[t - 1]), point],
                        Stroke::new(
                            if run == 0 { 5.0_f32 } else { 2.5_f32 },
                            color.gamma_multiply(if future { 0.25 } else { 0.95 }),
                        ),
                    );
                }
                witnesses.push(format!("{{\"kind\":\"path\",\"run\":{run},\"t\":{t},\"cell\":{p:?},\"future\":{future},\"pixel\":{:?}}}",[point.x,point.y]));
            }
            if let Some(p) = r.position(self.cursor) {
                let point = screen(map, p);
                let conflict = r.conflicts.contains(&self.cursor);
                let color = if conflict { RED } else { color };
                let radius = cell * if run == 0 { 0.24 } else { 0.36 };
                if run == 0 {
                    painter.circle_filled(point, radius, color);
                } else {
                    painter.circle_stroke(point, radius, Stroke::new(2.5_f32, color));
                }
                witnesses.push(format!("{{\"kind\":\"robot\",\"run\":{run},\"cell\":{p:?},\"conflict\":{conflict},\"pixel\":{:?},\"radius\":{radius}}}",[point.x,point.y]));
            }
        }
        for (p, label, color) in [(START, "S", GREEN), (GOAL, "G", GREEN), (GATE, "X", GOLD)] {
            painter.text(
                screen(map, p),
                egui::Align2::CENTER_CENTER,
                label,
                egui::FontId::proportional((cell * 0.45).min(18.)),
                color,
            );
        }
        let selected = Rect::from_center_size(screen(map, self.selected), egui::Vec2::splat(cell));
        painter.rect_stroke(
            selected.shrink(1.),
            0.,
            Stroke::new(2.0_f32, GOLD),
            egui::StrokeKind::Inside,
        );
        if let Some(from) = self.drag {
            if let Some(to) = ui
                .ctx()
                .input(|i| i.pointer.latest_pos())
                .and_then(|p| cell_at(map, p))
            {
                for (x, y) in super::super::cells_between((from[0], from[1]), (to[0], to[1])) {
                    painter.rect_stroke(
                        Rect::from_center_size(screen(map, [x, y]), egui::Vec2::splat(cell))
                            .shrink(2.),
                        0.,
                        Stroke::new(2.0_f32, GOLD),
                        egui::StrokeKind::Inside,
                    );
                }
            }
        }
        painter.rect_filled(band, 2., RED.gamma_multiply(0.25));
        let scale = band.width() / (HORIZON + 1) as f32;
        for interval in &self.intervals[self.selected[1] * WIDTH + self.selected[0]] {
            let r = Rect::from_min_max(
                egui::pos2(band.left() + interval[0] as f32 * scale, band.top()),
                egui::pos2(band.left() + interval[1] as f32 * scale, band.bottom()),
            );
            painter.rect_filled(r, 0., GREEN.gamma_multiply(0.35));
            witnesses.push(format!(
                "{{\"kind\":\"interval\",\"interval\":{interval:?},\"pixel\":{:?}}}",
                [r.left(), r.top(), r.width(), r.height()]
            ));
        }
        for (run, r) in self.runs.iter().enumerate() {
            let end = if self.future { HORIZON } else { self.cursor };
            for (t, &p) in r.positions.iter().enumerate().take(end + 1) {
                if p != self.selected {
                    continue;
                }
                let point = egui::pos2(
                    band.left() + (t as f32 + 0.5) * scale,
                    band.top() + band.height() * (if run == 0 { 0.3 } else { 0.7 }),
                );
                let color = if self.reserved(p, t) {
                    RED
                } else if run == 0 {
                    RAW
                } else {
                    AWARE
                };
                painter.circle_filled(
                    point,
                    3.5,
                    color.gamma_multiply(if t > self.cursor { 0.4 } else { 1. }),
                );
                witnesses.push(format!(
                    "{{\"kind\":\"visit\",\"run\":{run},\"t\":{t},\"cell\":{p:?},\"pixel\":{:?}}}",
                    [point.x, point.y]
                ));
            }
        }
        painter.rect_filled(graph, 2., Color32::from_rgb(20, 29, 40));
        let graph_scale = graph.width() / (HORIZON + 1) as f32;
        for run in 0..2 {
            for t in 0..=HORIZON {
                if !self.future && t > self.cursor {
                    continue;
                }
                let status = phase(self, run, t);
                let r = Rect::from_min_size(
                    egui::pos2(
                        graph.left() + t as f32 * graph_scale,
                        graph.top() + run as f32 * 36.,
                    ),
                    egui::vec2(graph_scale, 24.),
                );
                painter.rect_filled(
                    r.shrink2(egui::vec2(0.5, 1.)),
                    1.,
                    phase_color(status, run).gamma_multiply(if t > self.cursor {
                        0.25
                    } else {
                        0.9
                    }),
                );
                witnesses.push(format!("{{\"kind\":\"phase\",\"run\":{run},\"t\":{t},\"status\":\"{status}\",\"pixel\":{:?}}}",[r.left(),r.top(),r.width(),r.height()]));
            }
        }
        for (name, r) in [("band", band), ("graph", graph)] {
            let x = r.left() + (self.cursor as f32 + 0.5) / (HORIZON + 1) as f32 * r.width();
            painter.line_segment(
                [egui::pos2(x, r.top()), egui::pos2(x, r.bottom())],
                Stroke::new(2.0_f32, GOLD),
            );
            witnesses.push(format!(
                "{{\"kind\":\"cursor\",\"strip\":\"{name}\",\"t\":{},\"pixel\":{:?}}}",
                self.cursor,
                [x, r.top(), x, r.bottom()]
            ));
        }
        for t in [0, 5, 10, 15, 20, 24] {
            let x = graph.left() + (t as f32 + 0.5) * graph_scale;
            painter.text(
                egui::pos2(x, graph.bottom() - 2.),
                egui::Align2::CENTER_BOTTOM,
                t.to_string(),
                egui::FontId::proportional(10.),
                Color32::LIGHT_GRAY,
            );
        }
        self.witnesses = witnesses.join(",");
    }
}
