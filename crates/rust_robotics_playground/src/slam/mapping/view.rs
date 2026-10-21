use super::*;
use egui::{Color32, Pos2, Sense, Stroke, StrokeKind};

const GOLD: Color32 = Color32::from_rgb(255, 209, 105);
const BLUE: Color32 = Color32::from_rgb(91, 174, 245);
const GREEN: Color32 = Color32::from_rgb(90, 220, 150);
const RED: Color32 = Color32::from_rgb(245, 112, 112);
fn screen(r: Rect, p: Point) -> Pos2 {
    egui::pos2(
        r.left() + (p[0] / 12.) as f32 * r.width(),
        r.bottom() - (p[1] / 8.) as f32 * r.height(),
    )
}
fn world(r: Rect, p: Pos2) -> Point {
    [
        f64::from((p.x - r.left()) / r.width()) * 12.,
        f64::from((r.bottom() - p.y) / r.height()) * 8.,
    ]
}
fn color(value: f32, seen: usize) -> Color32 {
    if seen == 0 {
        return Color32::from_rgb(34, 44, 57);
    }
    let p = probability(value);
    let a = [49., 115., 172.];
    let b = [207., 90., 89.];
    Color32::from_rgb(
        (a[0] * (1. - p) + b[0] * p).round() as u8,
        (a[1] * (1. - p) + b[1] * p).round() as u8,
        (a[2] * (1. - p) + b[2] * p).round() as u8,
    )
}
fn status(state: u8, seen: usize) -> &'static str {
    if seen == 0 {
        "Unseen"
    } else {
        match state {
            1 => "Free",
            2 => "Occupied",
            _ => "Uncertain",
        }
    }
}
impl MappingLab {
    pub(in crate::slam) fn controls(&mut self, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Occupancy evidence lab");
        crate::ui_kit::hint(ui,"Watch a map change its mind. A farther return clears space; a missing observation leaves the old belief intact.");
        ui.horizontal_wrapped(|ui| {
            for case in Case::ALL {
                let r = ui.selectable_label(self.case == case, case.label());
                if self.button(case.slug(), &r) {
                    self.case = case;
                    self.record();
                }
            }
        });
        crate::ui_kit::section(ui, "Recorded observations");
        ui.horizontal_wrapped(|ui| {
            let r = ui.button(if self.playing { "Pause" } else { "Play scans" });
            if self.button("play", &r) {
                if self.cursor == LAST {
                    self.cursor = 0;
                }
                self.playing = !self.playing;
                self.clock = None;
                self.pending = 0.;
            }
            for (name, label, at) in [
                ("start", "Start", 0),
                ("back", "−1", self.cursor.saturating_sub(1)),
                ("step", "+1", self.cursor + 1),
                ("end", "End", LAST),
            ] {
                let r = ui.button(label);
                if self.button(name, &r) {
                    self.seek(at);
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            for (name, label, at, enabled) in [
                ("first", "First return", 1, true),
                ("before", "Before switch", 4, self.case.switches()),
                ("change", "Input switch", 5, self.case.switches()),
                (
                    "cleared",
                    "Cleared cell",
                    self.cleared().unwrap_or(0),
                    self.cleared().is_some(),
                ),
            ] {
                let r = ui.add_enabled(enabled, egui::Button::new(label));
                if self.button(name, &r) {
                    self.seek(at);
                }
            }
        });
        ui.small(format!(
            "Batch {} / {LAST} · {:.1} s · 2 batches/s",
            self.cursor,
            self.cursor as f64 * 0.5
        ));
        let r = ui.checkbox(&mut self.future, "Show remaining recorded history");
        self.button("future", &r);
        let r = ui.checkbox(&mut self.rays, "Show current return rays");
        self.button("rays", &r);
        crate::ui_kit::section(ui, "Evidence per scan");
        ui.horizontal_wrapped(|ui| {
            ui.label("Hit");
            for (name, label, v) in [
                ("soft_hit", "+0.4", 0.4),
                ("normal_hit", "+0.9", 0.9),
                ("strong_hit", "+1.4", 1.4),
            ] {
                let r = ui.selectable_label(self.hit == v, label);
                if self.button(name, &r) {
                    self.hit = v;
                    self.record();
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("Pass");
            for (name, label, v) in [
                ("gentle_clear", "−0.2", -0.2),
                ("normal_clear", "−0.4", -0.4),
                ("fast_clear", "−0.8", -0.8),
            ] {
                let r = ui.selectable_label(self.miss == v, label);
                if self.button(name, &r) {
                    self.miss = v;
                    self.record();
                }
            }
        });
        let r = ui.checkbox(&mut self.duplicates, "Duplicate returns in each scan");
        if self.button("duplicates", &r) {
            self.record();
        }
        ui.small("One update per cell per scan. A hit wins over a pass.");
        crate::ui_kit::section(ui, "Map interaction");
        ui.horizontal_wrapped(|ui| {
            for edit in Edit::ALL {
                let r = ui.selectable_label(self.edit == edit, edit.label());
                if self.button(edit.slug(), &r) {
                    self.edit = edit;
                    self.seek(self.cursor);
                }
            }
        });
        ui.small("Tap to inspect. Move the known sensor or near returns by tapping or dragging; release records a fresh experiment.");
        crate::ui_kit::legend(
            ui,
            &[
                (BLUE, "free evidence"),
                (RED, "occupied evidence"),
                (GREEN, "cells passed this scan"),
                (GOLD, "selected cell"),
            ],
        );
        crate::ui_kit::how_it_works(ui,"mapping_help","The Rust occupancy grid traces each sparse return with Bresenham's algorithm. Cells before an endpoint receive the pass increment; the endpoint receives the hit increment. Each cell changes once per scan, even with duplicate returns, and a hit overrides a grazing beam. Log-odds are clamped to ±5. Probability is 1/(1+exp(−log-odds)); it describes this model's belief, not measured accuracy. Occupied means log-odds > 0.6; free means < −0.3. The band between is uncertain. Unseen means no scan has updated the cell; an uncertain cell can already have observations. Farther returns uses four near scans, then shifts returns 3 m to the right. Only cells crossed by those new rays can clear. No observations instead supplies empty scans after batch four: it supplies no free-space evidence and has no artificial decay. This lab uses sparse return points at known, fixed poses, not a full range sensor, a moving-object tracker, or pose estimation. Hit wins combines a near hit and a longer grazing beam in one scan. The graph shows actual recorded cell values; future segments are recorded evidence, not predictions. Seeking pauses without rebuilding the experiment. Edits and evidence changes rebuild it from an unknown map.");
        let r = ui.button("SLAM demos");
        if self.button("legacy", &r) {
            self.active = false;
            self.seek(self.cursor);
        }
    }
    pub(in crate::slam) fn scene(&mut self, ui: &mut egui::Ui) {
        if ui.ctx().screen_rect().width() >= 700. {
            egui::ScrollArea::vertical()
                .id_salt("mapping_scene")
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
        ui.heading("What did the sensor actually observe?");
        ui.label(self.case.label());
        let f = self.frame();
        let index = self.index();
        let value = f.values[index];
        ui.label(format!(
            "Cell {:?} · {} · P(occupied) {:.1}%",
            self.selected,
            status(f.states[index], f.seen[index]),
            probability(value) * 100.
        ));
        ui.small(format!(
            "log-odds {value:+.3} · {} observing scans · {}",
            f.seen[index],
            f.updates
                .iter()
                .find(|u| [u.x, u.y] == self.selected)
                .map_or("No update this batch".into(), |u| format!(
                    "{}: {:+.3} to {:+.3}",
                    if u.evidence == OccupancyEvidence::Hit {
                        "Hit"
                    } else {
                        "Pass"
                    },
                    u.before,
                    u.after
                ))
        ));
        let [unseen, uncertain, free, occupied] = self.stats();
        ui.small(format!(
            "{unseen} unseen · {uncertain} uncertain · {free} free · {occupied} occupied"
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
        if response.drag_started() || response.dragged() {
            if let Some(p) = response.interact_pointer_pos() {
                self.drag = Some(bounded(world(r, p), self.edit));
            }
        }
        if response.clicked() || response.drag_stopped() {
            if let Some(p) = response.interact_pointer_pos() {
                self.set_point(world(r, p));
                self.ack("map", &response.ctx);
            }
            self.drag = None;
        }
        ui.small("Known sensor pose · 0.5 m cells · current batch outlines only");
        let mut witnesses = self.draw_graph(ui);
        let painter = ui.painter_at(r);
        let f = self.frame();
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                let index = y * WIDTH + x;
                let min = screen(r, [x as f64 * RESOLUTION, (y + 1) as f64 * RESOLUTION]);
                let rect = Rect::from_min_size(min, egui::vec2(side, side));
                let c = color(f.values[index], f.seen[index]);
                painter.rect_filled(rect.shrink(0.5), 0., c);
                witnesses.push(format!("{{\"kind\":\"cell\",\"cell\":[{x},{y}],\"pixel\":{:?},\"size\":{side},\"value\":{},\"probability\":{},\"state\":{},\"seen\":{},\"rgb\":{:?}}}",
                [min.x,min.y],f.values[index],probability(f.values[index]),f.states[index],f.seen[index],[c.r(),c.g(),c.b()]));
            }
        }
        for u in &f.updates {
            let min = screen(r, [u.x as f64 * RESOLUTION, (u.y + 1) as f64 * RESOLUTION]);
            let rect = Rect::from_min_size(min, egui::vec2(side, side)).shrink(1.);
            let hit = u.evidence == OccupancyEvidence::Hit;
            painter.rect_stroke(
                rect,
                0.,
                Stroke::new(1.0_f32, if hit { RED } else { GREEN }),
                StrokeKind::Inside,
            );
            witnesses.push(format!(
                "{{\"kind\":\"update\",\"cell\":[{},{}],\"hit\":{hit},\"pixel\":{:?},\"size\":{}}}",
                u.x,
                u.y,
                [rect.left(), rect.top()],
                rect.width()
            ));
        }
        let origin = screen(r, self.sensor);
        if self.rays {
            for (index, &p) in f.points.iter().enumerate() {
                let end = screen(r, p);
                painter.line_segment(
                    [origin, end],
                    Stroke::new(1.0_f32, GOLD.gamma_multiply(0.65)),
                );
                painter.circle_filled(end, 3., RED);
                witnesses.push(format!("{{\"kind\":\"ray\",\"index\":{index},\"world\":[{:?},{p:?}],\"pixel\":[{:?},{:?}]}}",self.sensor,[origin.x,origin.y],[end.x,end.y]));
            }
        }
        painter.circle_filled(origin, 5., BLUE);
        let target = screen(r, self.target);
        painter.circle_stroke(target, 6., Stroke::new(2.0_f32, RED));
        for (kind, pixel, p) in [
            ("sensor", origin, self.sensor),
            ("target", target, self.target),
        ] {
            witnesses.push(format!(
                "{{\"kind\":\"{kind}\",\"world\":{p:?},\"pixel\":{:?}}}",
                [pixel.x, pixel.y]
            ));
        }
        let min = screen(
            r,
            [
                self.selected[0] as f64 * RESOLUTION,
                (self.selected[1] + 1) as f64 * RESOLUTION,
            ],
        );
        let selected = Rect::from_min_size(min, egui::vec2(side, side));
        painter.rect_stroke(selected, 0., Stroke::new(2.5f32, GOLD), StrokeKind::Inside);
        witnesses.push(format!(
            "{{\"kind\":\"selection\",\"cell\":{:?},\"pixel\":{:?},\"size\":{side}}}",
            self.selected,
            [min.x, min.y]
        ));
        if let Some(p) = self.drag {
            painter.circle_stroke(screen(r, p), 8., Stroke::new(2.0_f32, GOLD));
        }

        self.witnesses = format!("[{}]", witnesses.join(","));
        self.painted = Some((
            self.generation,
            self.cursor,
            self.selected,
            self.future,
            self.rays,
        ));
    }
    fn draw_graph(&mut self, ui: &mut egui::Ui) -> Vec<String> {
        ui.label("Selected cell / accumulated evidence");
        ui.small("Tap or drag to seek · gold cursor · faint segments are remaining recorded scans");
        let (r, response) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), 112.),
            Sense::click_and_drag(),
        );
        self.graph = Some(r);
        self.widget("graph", &response);
        if response.clicked() || response.dragged() || response.drag_stopped() {
            if let Some(p) = response.interact_pointer_pos() {
                self.seek(
                    (((p.x - r.left()) / r.width()).clamp(0., 1.) * LAST as f32).round() as usize,
                );
                if response.clicked() || response.drag_stopped() {
                    self.ack("graph", &response.ctx);
                }
            }
        }
        let painter = ui.painter_at(r);
        painter.rect_filled(r, 4., Color32::from_rgb(13, 21, 31));
        let project = |at: usize, value: f32| {
            egui::pos2(
                r.left() + at as f32 / LAST as f32 * r.width(),
                r.bottom() - (value + 5.) / 10. * r.height(),
            )
        };
        let mut witnesses = Vec::new();
        for value in [-0.3, 0.6] {
            let y = project(0, value).y;
            painter.line_segment(
                [egui::pos2(r.left(), y), egui::pos2(r.right(), y)],
                Stroke::new(0.75f32, Color32::from_gray(95)),
            );
            witnesses.push(format!(
                "{{\"kind\":\"threshold\",\"value\":{value},\"pixel\":[{},{y},{},{y}]}}",
                r.left(),
                r.right()
            ));
        }
        let limit = if self.future { LAST } else { self.cursor };
        let mut previous = None;
        for at in 0..=limit {
            let value = self.frames[at].values[self.index()];
            let p = project(at, value);
            if let Some(a) = previous {
                painter.line_segment(
                    [a, p],
                    Stroke::new(
                        2.0_f32,
                        if at > self.cursor {
                            BLUE.gamma_multiply(0.3)
                        } else {
                            BLUE
                        },
                    ),
                );
            }
            painter.circle_filled(
                p,
                2.,
                if at > self.cursor {
                    BLUE.gamma_multiply(0.3)
                } else {
                    BLUE
                },
            );
            previous = Some(p);
            witnesses.push(format!("{{\"kind\":\"history\",\"at\":{at},\"value\":{value},\"future\":{},\"pixel\":{:?}}}",at>self.cursor,[p.x,p.y]));
        }
        let p = project(self.cursor, self.frame().values[self.index()]);
        painter.line_segment(
            [egui::pos2(p.x, r.top()), egui::pos2(p.x, r.bottom())],
            Stroke::new(1.0_f32, GOLD),
        );
        painter.circle_filled(p, 4., GOLD);
        witnesses.push(format!(
            "{{\"kind\":\"cursor\",\"at\":{},\"pixel\":{:?}}}",
            self.cursor,
            [p.x, p.y]
        ));
        ui.small("log-odds −5 to +5 · free < −0.3 · occupied > +0.6");
        witnesses
    }
}
