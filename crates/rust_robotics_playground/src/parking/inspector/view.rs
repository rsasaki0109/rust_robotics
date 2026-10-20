use super::{ParkingDemo, REPLAY_RATE};
use crate::parking::{
    car_corners, draw_car, to_screen, to_world, ASPHALT, CAR, EGO, FORWARD, GOAL, LINE, LOT_H,
    LOT_W, PARKED, REVERSE,
};
use egui::{Color32, Rect, Sense, Shape, Stroke};

const SWITCH: Color32 = Color32::from_rgb(255, 220, 115);
fn gear(d: i32) -> Color32 {
    if d < 0 {
        REVERSE
    } else {
        FORWARD
    }
}

impl ParkingDemo {
    pub(in crate::parking) fn inspector_controls(&mut self, ui: &mut egui::Ui) {
        self.ensure_planned();
        crate::ui_kit::section(ui, "Parking maneuver lab");
        crate::ui_kit::hint(ui, "Follow the car's full body through a real Hybrid A* maneuver. Blue: forward. Orange: reverse.");
        ui.horizontal_wrapped(|ui| {
            for (name, label) in [
                ("parallel", "Parallel"),
                ("back_in", "Back in"),
                ("turn", "Turn around"),
                ("blocked", "Blocked goal"),
            ] {
                let r = ui.button(label);
                if self.inspector.button(name, &r) {
                    self.inspector.cursor = 0;
                    match name {
                        "parallel" => self.preset_parallel(),
                        "back_in" => self.preset_back_in(),
                        "turn" => self.preset_turn(),
                        _ => {
                            self.plan = None;
                            self.car = super::PARALLEL_START;
                            self.inspector_goal([2.5, 1.4, 0.0]);
                        }
                    }
                }
            }
        });
        let last = self.inspector_last();
        crate::ui_kit::section(ui, "Recorded poses · no interpolation");
        ui.horizontal_wrapped(|ui| {
            let r = ui.button(if self.inspector.playing {
                "Pause"
            } else {
                "Play"
            });
            if self.inspector.button("play", &r) && self.plan.is_some() {
                if self.inspector.cursor == last {
                    self.inspector.cursor = 0;
                }
                self.inspector.playing = !self.inspector.playing;
                self.inspector.clock = None;
                self.inspector.pending = 0.0;
            }
            for (name, label, index) in [
                ("start", "Start", 0),
                ("back", "−1", self.inspector.cursor.saturating_sub(1)),
                ("step", "+1", self.inspector.cursor + 1),
                ("end", "End", last),
            ] {
                let r = ui.button(label);
                if self.inspector.button(name, &r) {
                    self.inspector.seek(index, last);
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            let before = self
                .inspector
                .changes
                .iter()
                .copied()
                .find(|&i| self.plan.as_ref().is_some_and(|p| p.distance[i] > 1e-9))
                .or_else(|| self.inspector.changes.first().copied())
                .unwrap_or(0)
                .saturating_sub(1);
            let previous = self
                .inspector
                .changes
                .iter()
                .copied()
                .rev()
                .find(|&i| i < self.inspector.cursor)
                .unwrap_or(0);
            let next = self
                .inspector
                .changes
                .iter()
                .copied()
                .find(|&i| i > self.inspector.cursor)
                .unwrap_or(last);
            for (name, label, index) in [
                ("before", "Before switch", before),
                ("previous", "Previous switch", previous),
                ("next", "Next switch", next),
            ] {
                let r = ui.button(label);
                if self.inspector.button(name, &r) {
                    self.inspector.seek(index, last);
                }
            }
        });
        let r = ui.checkbox(&mut self.inspector.ghosts, "Show sampled body footprints");
        self.inspector.button("sweep", &r);
        let r = ui.checkbox(&mut self.inspector.circles, "Show collision-check circles");
        self.inspector.button("circles", &r);
        if let Some(p) = &self.plan {
            let i = self.inspector.cursor;
            ui.colored_label(
                gear(p.directions[i]),
                format!(
                    "{} · pose {i}/{last}",
                    if p.directions[i] < 0 {
                        "REVERSE"
                    } else {
                        "FORWARD"
                    }
                ),
            );
            ui.label(format!(
                "Distance {:.2} / {:.2} m",
                p.distance[i],
                p.length()
            ));
            ui.label(format!(
                "Heading {:.1}° · {} recorded gear changes",
                p.poses[i][2].to_degrees(),
                self.inspector.changes.len()
            ));
            ui.small(format!(
                "Plan {:.0} ms · replay {:.0} poses/s",
                p.elapsed_ms, REPLAY_RATE
            ));
        } else if let Some(error) = &self.failure {
            ui.colored_label(REVERSE, format!("No path: {error}"));
        }
        ui.small("Car 4.6 × 1.9 m · wheelbase 2.7 m");
        ui.small(format!(
            "Minimum turn radius {:.2} m",
            super::super::WHEELBASE / super::super::MAX_STEER.tan()
        ));
        crate::ui_kit::legend(
            ui,
            &[
                (FORWARD, "forward"),
                (REVERSE, "reverse"),
                (SWITCH, "gear boundary"),
                (GOAL, "goal"),
            ],
        );
        crate::ui_kit::hint(ui,"Tap the lot for a goal position. Drag from that point toward the desired heading. Each edit replans from the configured start.");
        crate::ui_kit::how_it_works(ui,"parking_inspector_help","Hybrid A* uses short forward and reverse bicycle-model arcs and Reeds-Shepp connections. The rear axle's path is only a line: the body also needs room. Faint rectangles show a subset of recorded body poses, not a continuous collision guarantee. Gold markers show boundaries in the planner's recorded directions; a direction change can have consecutive samples at the same position. +1 selects the next actual sample. The circles are the same three covering circles used by the planner; its collision grid also discretizes the obstacle field. Playback advances ten recorded samples per second, not constant physical speed. Seeking or editing a goal pauses replay. All edits start from the configured start, not the inspected pose. Share restores the precise start, goal, sample and overlays as a paused experiment. Live driving returns to the existing animation from the path start.");
        let r = ui.button("Live driving");
        if self.inspector.button("live", &r) {
            self.inspector.active = false;
            self.driven = 0.0;
            self.last_time = None;
        }
    }

    pub(in crate::parking) fn inspector_scene(&mut self, ui: &mut egui::Ui) {
        if ui.ctx().screen_rect().width() >= 700.0 {
            egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .show(ui, |ui| self.inspector_content(ui));
        } else {
            self.inspector_content(ui);
        }
    }

    fn inspector_content(&mut self, ui: &mut egui::Ui) {
        self.ensure_planned();
        ui.heading("Parking maneuver lab");
        ui.label("Why does a car need to reverse here?");
        let width = ui
            .available_width()
            .min((ui.ctx().screen_rect().height() - 310.0).max(160.0) * 1.5);
        let rect = Rect::from_min_size(
            ui.next_widget_position() + egui::vec2((ui.available_width() - width) / 2.0, 0.0),
            egui::vec2(width, width * 2.0 / 3.0),
        );
        let response = ui.allocate_rect(rect, Sense::click_and_drag());
        self.inspector.map = Some(rect);
        self.inspector.widget("goal", &response);
        if response.drag_started() {
            if let Some(origin) = response
                .ctx
                .input(|i| i.pointer.press_origin())
                .filter(|p| rect.contains(*p))
            {
                let p = to_world(rect, origin);
                self.dragging = Some([p[0], p[1], self.goal[2]]);
            }
        }
        if let (Some(goal), Some(origin), Some(pos)) = (
            &mut self.dragging,
            response.ctx.input(|i| i.pointer.press_origin()),
            response.interact_pointer_pos(),
        ) {
            let delta = pos - origin;
            if delta.length() > super::super::MIN_HEADING_DRAG {
                goal[2] = f64::from(-delta.y).atan2(f64::from(delta.x));
            }
        }
        if response.drag_stopped() {
            if let Some(goal) = self.dragging.take() {
                self.inspector_goal(goal);
                self.inspector.ack("goal", &response.ctx);
            }
        } else if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                let p = to_world(rect, pos);
                self.inspector_goal([p[0], p[1], self.goal[2]]);
                self.inspector.ack("goal", &response.ctx);
            }
        }
        // Process the graph first, so the selected car and graph use one sample.
        self.inspector_graph(ui);
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 7.0, ASPHALT);
        for slot in 0..=10 {
            let x = 1.0 + slot as f64 * 2.9;
            painter.line_segment(
                [
                    to_screen(rect, [x, LOT_H]),
                    to_screen(rect, [x, LOT_H - 5.4]),
                ],
                Stroke::new(1.0_f32, LINE),
            );
        }
        painter.line_segment(
            [to_screen(rect, [0.0, 2.6]), to_screen(rect, [LOT_W, 2.6])],
            Stroke::new(1.0_f32, LINE),
        );
        for &[x, y, w, h] in &self.cars {
            painter.rect_filled(
                Rect::from_two_pos(to_screen(rect, [x, y]), to_screen(rect, [x + w, y + h])),
                3.0,
                PARKED,
            );
        }
        self.inspector.ghost_indices.clear();
        let mut witnesses = Vec::new();
        if let Some(p) = &self.plan {
            if self.inspector.ghosts {
                let stride = (p.poses.len() / 45).max(1);
                for i in (0..p.poses.len()).step_by(stride) {
                    let corners = car_corners(p.poses[i]);
                    let pixels: Vec<_> = corners.iter().map(|&c| to_screen(rect, c)).collect();
                    painter.add(Shape::closed_line(
                        pixels.clone(),
                        Stroke::new(0.65_f32, gear(p.directions[i]).gamma_multiply(0.22)),
                    ));
                    self.inspector.ghost_indices.push(i);
                    for (corner, (&world, pixel)) in corners.iter().zip(&pixels).enumerate() {
                        witnesses.push(witness("ghost", i, corner, world, *pixel));
                    }
                }
            }
            for (i, pair) in p.poses.windows(2).enumerate() {
                painter.line_segment(
                    [
                        to_screen(rect, [pair[0][0], pair[0][1]]),
                        to_screen(rect, [pair[1][0], pair[1][1]]),
                    ],
                    Stroke::new(2.5_f32, gear(p.directions[i + 1])),
                );
            }
            for (number, &i) in self.inspector.changes.iter().enumerate() {
                let world = [p.poses[i][0], p.poses[i][1]];
                let pixel = to_screen(rect, world);
                painter.circle_stroke(pixel, 6.0, Stroke::new(1.5_f32, SWITCH));
                painter.text(
                    pixel + egui::vec2(7.0, -7.0),
                    egui::Align2::LEFT_BOTTOM,
                    (number + 1).to_string(),
                    egui::FontId::proportional(11.0),
                    SWITCH,
                );
                witnesses.push(witness("switch", i, 0, world, pixel));
            }
        }
        draw_car(
            &painter,
            rect,
            self.dragging.unwrap_or(self.goal),
            None,
            GOAL,
        );
        let pose = self.inspected_pose();
        let color = self
            .plan
            .as_ref()
            .map_or(EGO, |p| gear(p.directions[self.inspector.cursor]));
        let body = car_corners(pose);
        let pixels: Vec<_> = body.iter().map(|&world| to_screen(rect, world)).collect();
        painter.add(Shape::convex_polygon(
            pixels.clone(),
            color.gamma_multiply(0.55),
            Stroke::new(2.0_f32, color),
        ));
        // Mark the front edge, making a reversed car's heading unambiguous.
        painter.line_segment([pixels[1], pixels[2]], Stroke::new(4.0_f32, Color32::WHITE));
        for (corner, (&world, &pixel)) in body.iter().zip(&pixels).enumerate() {
            witnesses.push(witness("body", self.inspector.cursor, corner, world, pixel));
        }
        // The rear axle is the planner's reference point, not the body center.
        painter.circle_filled(to_screen(rect, [pose[0], pose[1]]), 3.0, Color32::WHITE);
        if self.inspector.circles {
            let (sin, cos) = pose[2].sin_cos();
            for (i, (offset, radius)) in CAR.covering_circles().into_iter().enumerate() {
                let world = [pose[0] + offset * cos, pose[1] + offset * sin];
                let pixel = to_screen(rect, world);
                painter.circle_stroke(
                    pixel,
                    radius as f32 * rect.width() / LOT_W as f32,
                    Stroke::new(1.0_f32, SWITCH.gamma_multiply(0.75)),
                );
                witnesses.push(witness("circle", self.inspector.cursor, i, world, pixel));
            }
        }
        self.inspector.witnesses = format!("[{}]", witnesses.join(","));
        self.inspector.painted = Some((
            self.inspector.generation,
            self.inspector.cursor,
            self.inspector.ghosts,
            self.inspector.circles,
        ));
        if let Some(error) = &self.failure {
            ui.colored_label(REVERSE, format!("No path · {error}"));
        }
        crate::ui_kit::hint(
            ui,
            "White dot: rear axle · gold: gear boundaries · outlines: sampled body poses",
        );
    }

    fn inspector_graph(&mut self, ui: &mut egui::Ui) {
        let (rect, r) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), 100.0),
            Sense::click_and_drag(),
        );
        let plot = Rect::from_min_max(
            rect.min + egui::vec2(12.0, 26.0),
            rect.max - egui::vec2(12.0, 25.0),
        );
        self.inspector.graph = Some(plot);
        self.inspector.widget("graph", &r);
        if r.clicked() || r.dragged() {
            if let Some(pos) = r.interact_pointer_pos() {
                let last = self.inspector_last();
                let index = (((pos.x - plot.left()) / plot.width()).clamp(0.0, 1.0) * last as f32)
                    .round() as usize;
                self.inspector.seek(index, last);
                r.ctx.request_repaint();
            }
        }
        if r.clicked() || r.drag_stopped() {
            self.inspector.ack("graph", &r.ctx);
        }
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 7.0, Color32::from_rgb(17, 25, 35));
        painter.text(
            rect.min + egui::vec2(5.0, 3.0),
            egui::Align2::LEFT_TOP,
            "Maneuver samples · tap or drag to inspect",
            egui::FontId::proportional(11.0),
            Color32::LIGHT_GRAY,
        );
        if let Some(p) = &self.plan {
            let last = self.inspector_last().max(1);
            let x = |i: usize| plot.left() + i as f32 / last as f32 * plot.width();
            for i in 1..p.poses.len() {
                painter.rect_filled(
                    Rect::from_min_max(
                        egui::pos2(x(i - 1), plot.top()),
                        egui::pos2(x(i), plot.bottom()),
                    ),
                    0.0,
                    gear(p.directions[i]).gamma_multiply(0.65),
                );
            }
            for &i in &self.inspector.changes {
                painter.line_segment(
                    [
                        egui::pos2(x(i), plot.top()),
                        egui::pos2(x(i), plot.bottom()),
                    ],
                    Stroke::new(1.0_f32, SWITCH),
                );
            }
            painter.line_segment(
                [
                    egui::pos2(x(self.inspector.cursor), plot.top() - 3.0),
                    egui::pos2(x(self.inspector.cursor), plot.bottom() + 3.0),
                ],
                Stroke::new(2.0_f32, Color32::WHITE),
            );
            painter.text(
                plot.left_bottom() + egui::vec2(0.0, 5.0),
                egui::Align2::LEFT_TOP,
                "0",
                egui::FontId::proportional(10.0),
                Color32::GRAY,
            );
            painter.text(
                plot.right_bottom() + egui::vec2(0.0, 5.0),
                egui::Align2::RIGHT_TOP,
                format!("{} · stored samples", p.poses.len() - 1),
                egui::FontId::proportional(10.0),
                Color32::GRAY,
            );
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(super) fn publish_inspector(&self, _ctx: &egui::Context, _active: bool) {
        let _ = (&self.inspector.plan_json, &self.inspector.witnesses);
    }

    #[cfg(target_arch = "wasm32")]
    pub(super) fn publish_inspector(&self, ctx: &egui::Context, active: bool) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let set = |name: &str, value: String| {
            let _ = canvas.set_attribute(&format!("data-parking-{name}"), &value);
        };
        let s = &self.inspector;
        set("active", u8::from(active).to_string());
        set("actions", format!("{:?}", s.actions));
        set("hits", format!("{:?}", s.hits));
        set("action-count", s.count.to_string());
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
        set(
            "down",
            ctx.input(|i| u8::from(i.pointer.primary_down()).to_string()),
        );
        set(
            "press",
            ctx.input(|i| i.pointer.press_origin())
                .map_or_else(|| "null".into(), |p| format!("[{},{}]", p.x, p.y)),
        );
        set(
            "drag",
            self.dragging
                .map_or_else(|| "null".into(), |p| format!("{p:?}")),
        );
        set("generation", s.generation.to_string());
        set("cursor", s.cursor.to_string());
        set("last", self.inspector_last().to_string());
        set("playing", u8::from(s.playing).to_string());
        set("sweep", u8::from(s.ghosts).to_string());
        set("circles", u8::from(s.circles).to_string());
        set("plan", s.plan_json.clone());
        set("start", format!("{:?}", self.car));
        set("goal", format!("{:?}", self.goal));
        set("pose", format!("{:?}", self.inspected_pose()));
        set("cars", format!("{:?}", self.cars));
        set(
            "cover",
            format!(
                "{:?}",
                CAR.covering_circles()
                    .iter()
                    .map(|&(offset, radius)| [offset, radius])
                    .collect::<Vec<_>>()
            ),
        );
        set(
            "failure",
            self.failure
                .as_ref()
                .map_or_else(|| "null".into(), |e| format!("{e:?}")),
        );
        let rect = |r: Rect| format!("[{},{},{},{}]", r.left(), r.top(), r.width(), r.height());
        set("map", s.map.map_or_else(|| "null".into(), rect));
        set("graph", s.graph.map_or_else(|| "null".into(), rect));
        set(
            "painted",
            s.painted.map_or_else(
                || "null".into(),
                |p| format!("[{},{},{},{}]", p.0, p.1, p.2, p.3),
            ),
        );
        set("witnesses", s.witnesses.clone());
        set("ghost-indices", format!("{:?}", s.ghost_indices));
        set("share", self.inspector_query());
    }
}

fn witness(kind: &str, index: usize, corner: usize, world: [f64; 2], pixel: egui::Pos2) -> String {
    format!("{{\"kind\":\"{kind}\",\"index\":{index},\"corner\":{corner},\"world\":{world:?},\"pixel\":[{},{}]}}",pixel.x,pixel.y)
}
