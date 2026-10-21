use super::{apply, Case, Point, ScanLab, SPAN, TRUTH};
use egui::{Color32, Pos2, Rect, Sense, Stroke};

const BLUE: Color32 = Color32::from_rgb(96, 175, 255);
const CYAN: Color32 = Color32::from_rgb(91, 215, 231);
const GOLD: Color32 = Color32::from_rgb(255, 209, 105);
const ORANGE: Color32 = Color32::from_rgb(255, 160, 90);
const GREEN: Color32 = Color32::from_rgb(90, 220, 150);
fn screen(r: Rect, p: Point) -> Pos2 {
    egui::pos2(
        r.left() + ((p[0] + 7.) / SPAN) as f32 * r.width(),
        r.bottom() - ((p[1] + 7.) / SPAN) as f32 * r.height(),
    )
}
fn world(r: Rect, p: Pos2) -> Point {
    [
        -7. + f64::from((p.x - r.left()) / r.width()) * SPAN,
        -7. + f64::from((r.bottom() - p.y) / r.height()) * SPAN,
    ]
}
fn angle_error(a: f64, b: f64) -> f64 {
    (a - b + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

impl ScanLab {
    pub(in crate::slam) fn controls(&mut self, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Scan matching lab");
        crate::ui_kit::hint(ui,"Match a scan by alternating real nearest neighbors and rigid fits. A small residual can still describe the wrong place.");
        ui.horizontal_wrapped(|ui| {
            for case in Case::ALL {
                let r = ui.selectable_label(self.case == case, case.label());
                if self.button(case.slug(), &r) {
                    self.preset(case);
                }
            }
        });
        crate::ui_kit::section(ui, "Recorded ICP iterations");
        ui.horizontal_wrapped(|ui| {
            let r = ui.button(if self.playing {
                "Pause"
            } else {
                "Play iterations"
            });
            if self.button("play", &r) {
                if self.cursor == self.last() {
                    self.cursor = 0;
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
            "Iteration {} / {} / replay 2 iterations/s",
            self.cursor,
            self.last()
        ));
        let r = ui.checkbox(&mut self.pairs, "Show all correspondence lines");
        self.button("pairs", &r);
        let r = ui.checkbox(&mut self.ghost, "Show initial alignment");
        self.button("ghost", &r);
        crate::ui_kit::section(ui, "Selected source point");
        ui.horizontal_wrapped(|ui| {
            let r = ui.button("Previous point");
            if self.button("prev_point", &r) {
                self.selected = self.selected.saturating_sub(1);
            }
            let r = ui.button("Worst residual");
            if self.button("worst", &r) {
                let f = self.frame();
                self.selected = (0..f.points.len())
                    .max_by(|&a, &b| {
                        self.residual(a)
                            .total_cmp(&self.residual(b))
                            .then(b.cmp(&a))
                    })
                    .unwrap();
            }
        });
        ui.label(format!(
            "Source {} / target {}",
            self.selected,
            self.frame().pairs[self.selected]
        ));
        ui.small(format!(
            "Selected distance {:.4} m",
            self.residual(self.selected)
        ));
        crate::ui_kit::section(ui, "Initial pose");
        ui.small(format!(
            "x {:.3} / y {:.3} m / {:.1} deg",
            self.initial[0],
            self.initial[1],
            self.initial[2].to_degrees()
        ));
        let r = ui.button("Reset initial pose");
        if self.button("reset_pose", &r) {
            self.edit(self.case.initial());
        }
        crate::ui_kit::hint(ui,"Tap the map to set the source frame's initial position. Drag from that point to set its heading. Each release reruns ICP on the same raw scan.");
        crate::ui_kit::legend(
            ui,
            &[
                (CYAN, "target scan"),
                (BLUE, "aligned source"),
                (ORANGE, "initial source"),
                (GOLD, "selected match"),
                (GREEN, "known true pose"),
            ],
        );
        crate::ui_kit::how_it_works(ui,"icp_lab_help","This is the library's point-to-point ICP, using nearest-neighbor associations and an SVD rigid fit. The source scan stays fixed; each initial-pose edit starts a new solve. The target indices shown are the actual associations used by the next fit; several source points can share a target. The plotted residual is mean nearest-neighbor distance, while the SVD step minimizes squared distances for its fixed associations. A divergent step rejected by the solver is not replayed. Start and +1 show accepted recorded alignments, not an interpolated animation. Play advances two iterations per second; it is not sensor time. Seeking pauses playback without solving again. Corner starts from a nearby pose estimate. Poor guess uses the same scans but starts at the origin, where this local solver stops at an incorrect alignment. Partial scan removes a wall segment from the source while retaining it in the target. Repeated bays gives three identical target bays but only the central bay in the source. Its preset starts at the neighboring bay: matching residual can be almost zero even though position is wrong by three metres. The true pose is shown for this synthetic experiment, and is never supplied to the ICP solver. Convergence only describes the solver's stopping condition. The selected match remains visible when other lines are hidden. Share restores the exact initial pose, case, iteration, point and overlays, paused.");
        let r = ui.button("Legacy scan replay");
        if self.button("legacy", &r) {
            self.active = false;
            self.playing = false;
        }
    }

    fn residual(&self, i: usize) -> f64 {
        let f = self.frame();
        let p = f.points[i];
        let q = self.target[f.pairs[i]];
        (p[0] - q[0]).hypot(p[1] - q[1])
    }

    pub(in crate::slam) fn scene(&mut self, ui: &mut egui::Ui) {
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
        ui.heading("Scan matching lab");
        ui.label("Which points match, and where does that move the scan?");
        let width = ui
            .available_width()
            .min((ui.ctx().screen_rect().height() - 410.).max(160.));
        let r = Rect::from_min_size(
            ui.next_widget_position() + egui::vec2((ui.available_width() - width) / 2., 0.),
            egui::vec2(width, width),
        );
        let response = ui.allocate_rect(r, Sense::click_and_drag());
        self.map = Some(r);
        self.widget("pose", &response);
        if response.drag_started() {
            if let Some(p) = response
                .ctx
                .input(|i| i.pointer.press_origin())
                .filter(|p| r.contains(*p))
            {
                let [x, y] = world(r, p);
                self.drag = Some([x, y, self.initial[2]]);
            }
        }
        if let (Some(pose), Some(origin), Some(pos)) = (
            &mut self.drag,
            response.ctx.input(|i| i.pointer.press_origin()),
            response.interact_pointer_pos(),
        ) {
            let delta = pos - origin;
            if delta.length() > 10. {
                pose[2] = f64::from(-delta.y).atan2(f64::from(delta.x));
            }
        }
        if response.drag_stopped() {
            if let Some(pose) = self.drag.take() {
                self.edit(pose);
                self.ack("pose", &response.ctx);
            }
        } else if response.clicked() {
            if let Some(p) = response.interact_pointer_pos() {
                let [x, y] = world(r, p);
                self.edit([x, y, self.initial[2]]);
                self.ack("pose", &response.ctx);
            }
        }
        let mut witnesses = self.draw_charts(ui);
        let painter = ui.painter_at(r);
        painter.rect_filled(r, 7., Color32::from_rgb(14, 22, 32));
        for i in -6..=6 {
            for line in [
                [[i as f64, -7.], [i as f64, 7.]],
                [[-7., i as f64], [7., i as f64]],
            ] {
                painter.line_segment(
                    [screen(r, line[0]), screen(r, line[1])],
                    Stroke::new(0.5_f32, Color32::from_gray(39)),
                );
            }
        }
        for (i, &p) in self.target.iter().enumerate() {
            let c = screen(r, p);
            painter.rect_filled(Rect::from_center_size(c, egui::vec2(5., 5.)), 0., CYAN);
            witnesses.push(format!(
                "{{\"kind\":\"target\",\"index\":{i},\"world\":{p:?},\"pixel\":{:?}}}",
                [c.x, c.y]
            ));
        }
        if self.ghost {
            for (i, &p) in self.source.iter().enumerate() {
                let p = apply(self.initial, p);
                let c = screen(r, p);
                painter.circle_stroke(c, 3.5, Stroke::new(1.0_f32, ORANGE.gamma_multiply(0.5)));
                witnesses.push(format!(
                    "{{\"kind\":\"initial\",\"index\":{i},\"world\":{p:?},\"pixel\":{:?}}}",
                    [c.x, c.y]
                ));
            }
        }
        for (i, &p) in self.frame().points.iter().enumerate() {
            let q = self.target[self.frame().pairs[i]];
            let a = screen(r, p);
            let b = screen(r, q);
            if self.pairs || i == self.selected {
                painter.line_segment(
                    [a, b],
                    Stroke::new(
                        if i == self.selected { 2.0_f32 } else { 0.7_f32 },
                        if i == self.selected {
                            GOLD
                        } else {
                            BLUE.gamma_multiply(0.3)
                        },
                    ),
                );
                witnesses.push(format!("{{\"kind\":\"pair\",\"index\":{i},\"target\":{},\"world\":[{p:?},{q:?}],\"pixel\":[{:?},{:?}]}}",self.frame().pairs[i],[a.x,a.y],[b.x,b.y]));
            }
            painter.circle_filled(
                a,
                if i == self.selected { 4.5 } else { 2.7 },
                if i == self.selected { GOLD } else { BLUE },
            );
            witnesses.push(format!(
                "{{\"kind\":\"source\",\"index\":{i},\"world\":{p:?},\"pixel\":{:?}}}",
                [a.x, a.y]
            ));
        }
        let f = self.frame();
        let p = f.points[self.selected];
        let q = self.target[f.pairs[self.selected]];
        painter.circle_stroke(screen(r, q), 6., Stroke::new(1.5_f32, GOLD));
        painter.text(
            screen(r, p) + egui::vec2(8., -7.),
            egui::Align2::LEFT_BOTTOM,
            format!("{} / {}", self.selected, f.pairs[self.selected]),
            egui::FontId::proportional(11.),
            GOLD,
        );
        for (i, pose, color) in [
            (0, self.initial, ORANGE),
            (1, f.pose, BLUE),
            (2, TRUTH, GREEN),
        ] {
            if i == 0 && !self.ghost {
                continue;
            }
            let c = screen(r, [pose[0], pose[1]]);
            let end = apply(pose, [0.65, 0.]);
            let e = screen(r, end);
            painter.circle_stroke(c, 5., Stroke::new(1.5_f32, color));
            painter.line_segment([c, e], Stroke::new(2.0_f32, color));
            witnesses.push(format!("{{\"kind\":\"origin\",\"index\":{i},\"world\":[{},{}],\"pixel\":{:?},\"end\":{end:?},\"end_pixel\":{:?}}}",pose[0],pose[1],[c.x,c.y],[e.x,e.y]));
        }
        if let Some(pose) = self.drag {
            let c = screen(r, [pose[0], pose[1]]);
            let e = screen(r, apply(pose, [0.8, 0.]));
            painter.circle_stroke(c, 8., Stroke::new(2.0_f32, GOLD));
            painter.line_segment([c, e], Stroke::new(2.0_f32, GOLD));
        }
        self.witnesses = format!("[{}]", witnesses.join(","));
        self.painted = Some((
            self.generation,
            self.cursor,
            self.selected,
            self.pairs,
            self.ghost,
        ));
        let f = self.frame();
        let offset = (f.pose[0] - TRUTH[0]).hypot(f.pose[1] - TRUTH[1]);
        ui.label(format!(
            "Iteration {} / {} / mean residual {:.4} m",
            self.cursor,
            self.last(),
            f.mean
        ));
        ui.label(format!(
            "Known pose error {:.3} m / {:.2} deg",
            offset,
            angle_error(f.pose[2], TRUTH[2]).abs().to_degrees()
        ));
        ui.small(format!(
            "{} source / {} target points / {}",
            self.source.len(),
            self.target.len(),
            if self.cursor < self.last() {
                "recorded alignment"
            } else if self.converged {
                "solver converged"
            } else {
                "solver stopped"
            }
        ));
        if self.case == Case::Bays {
            ui.colored_label(
                GOLD,
                "Repeated bays: small residual can match a different place.",
            );
        } else {
            ui.small("Tap or drag the map to change the initial pose.");
        }
    }

    fn draw_charts(&mut self, ui: &mut egui::Ui) -> Vec<String> {
        ui.small("Mean residual / tap or drag to select an iteration");
        let (g, response) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), 58.),
            Sense::click_and_drag(),
        );
        self.charts[0] = Some(g);
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
        ui.small("Point residuals / tap or drag to select a source point");
        let (r, response) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), 48.),
            Sense::click_and_drag(),
        );
        self.charts[1] = Some(r);
        self.widget("residuals", &response);
        if response.clicked() || response.dragged() || response.drag_stopped() {
            if let Some(p) = response.interact_pointer_pos() {
                self.selected = (((p.x - r.left()) / r.width()).clamp(0., 0.999999)
                    * self.source.len() as f32) as usize;
                self.playing = false;
                self.clock = None;
                self.pending = 0.;
                if response.clicked() || response.drag_stopped() {
                    self.ack("residuals", &response.ctx);
                }
            }
        }
        let painter = ui.painter_at(g);
        painter.rect_filled(g, 4., Color32::from_rgb(10, 16, 24));
        let max = self.frames.iter().map(|f| f.mean).fold(0.05_f64, f64::max);
        let pixels: Vec<_> = self
            .frames
            .iter()
            .enumerate()
            .map(|(i, f)| {
                egui::pos2(
                    g.left() + i as f32 / self.last().max(1) as f32 * g.width(),
                    g.bottom() - 3. - (f.mean / max) as f32 * (g.height() - 6.),
                )
            })
            .collect();
        if pixels.len() > 1 {
            painter.add(egui::Shape::line(
                pixels.clone(),
                Stroke::new(1.5_f32, BLUE),
            ));
        }
        let c = pixels[self.cursor];
        painter.circle_filled(c, 4., GOLD);
        let mut witnesses = Vec::new();
        for (i, p) in pixels.iter().enumerate() {
            witnesses.push(format!(
                "{{\"kind\":\"error\",\"index\":{i},\"pixel\":{:?}}}",
                [p.x, p.y]
            ));
        }
        witnesses.push(format!(
            "{{\"kind\":\"cursor\",\"index\":{},\"pixel\":{:?}}}",
            self.cursor,
            [c.x, c.y]
        ));
        let painter = ui.painter_at(r);
        painter.rect_filled(r, 4., Color32::from_rgb(10, 16, 24));
        let residuals: Vec<_> = (0..self.source.len()).map(|i| self.residual(i)).collect();
        let max = residuals.iter().copied().fold(0.05_f64, f64::max);
        let mut bars = Vec::new();
        for (i, d) in residuals.into_iter().enumerate() {
            let x = r.left() + i as f32 / self.source.len() as f32 * r.width();
            let h = (d / max) as f32 * (r.height() - 4.);
            let b = Rect::from_min_max(
                egui::pos2(x, r.bottom() - h),
                egui::pos2(x + r.width() / self.source.len() as f32, r.bottom()),
            );
            painter.rect_filled(b, 0., if i == self.selected { GOLD } else { BLUE });
            bars.push(format!(
                "[{i},{d},{},{},{},{}]",
                b.left(),
                b.top(),
                b.width(),
                b.height()
            ));
        }
        self.bars = format!("[{}]", bars.join(","));
        witnesses
    }
}
