use super::{ess, values, BeliefLab, LANDMARKS, LAST, N, PHASES};
use egui::{Color32, Pos2, Rect, Sense, Stroke};

const BLUE: Color32 = Color32::from_rgb(96, 175, 255);
const GOLD: Color32 = Color32::from_rgb(255, 209, 105);
const CYAN: Color32 = Color32::from_rgb(91, 215, 231);
const GREEN: Color32 = Color32::from_rgb(90, 220, 150);

fn screen(r: Rect, p: [f64; 2]) -> Pos2 {
    egui::pos2(
        r.left() + ((p[0] + 1.) / 13.) as f32 * r.width(),
        r.bottom() - ((p[1] + 1.) / 13.) as f32 * r.height(),
    )
}

impl BeliefLab {
    pub(in crate::localization) fn controls(&mut self, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Belief update lab");
        crate::ui_kit::hint(ui,"How does one observation change where a robot thinks it is? Inspect the actual 96 hypotheses.");
        ui.horizontal_wrapped(|ui| {
            let r = ui.button(if self.playing { "Pause" } else { "Play phases" });
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
                ("back", "−1 phase", self.cursor.saturating_sub(1)),
                ("step", "+1 phase", self.cursor + 1),
            ] {
                let r = ui.button(label);
                if self.button(name, &r) {
                    self.seek(at);
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            for (i, label) in PHASES.iter().enumerate() {
                let r = ui.selectable_label(self.cursor % 4 == i, *label);
                if self.button(["prior", "predict", "weight", "resample"][i], &r) {
                    self.seek(self.cursor / 4 * 4 + i);
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            for (name, label, at) in [
                ("prev_scan", "Previous scan", self.cursor.saturating_sub(4)),
                ("next_scan", "Next scan", self.cursor + 4),
                (
                    "event",
                    "Before resampling",
                    self.scans
                        .iter()
                        .position(|s| s.parents.is_some())
                        .unwrap_or(0)
                        * 4
                        + 2,
                ),
            ] {
                let r = ui.button(label);
                if self.button(name, &r) {
                    self.seek(at);
                }
            }
        });
        crate::ui_kit::section(ui, "Landmark evidence");
        ui.horizontal_wrapped(|ui| {
            for (name, label, n) in [
                ("one", "A only", 1),
                ("two", "A + B", 2),
                ("all", "All five", 5),
            ] {
                let r = ui.selectable_label(self.landmarks == n, label);
                if self.button(name, &r) {
                    self.landmarks = n;
                    self.record();
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            for (name, label, sigma) in [
                ("quiet", "Quiet", 0.15),
                ("normal", "Normal", 0.25),
                ("noisy", "Noisy", 0.65),
            ] {
                let r = ui.selectable_label(self.sigma == sigma, label);
                if self.button(name, &r) {
                    self.sigma = sigma;
                    self.record();
                }
            }
        });
        ui.small(format!(
            "Range sigma {:.2} m · seed {}",
            self.sigma, self.seed
        ));
        let r = ui.button("New seed");
        if self.button("seed", &r) {
            self.seed = self.seed.wrapping_add(1);
            self.record();
        }
        let r = ui.checkbox(&mut self.ranges, "Show measured ranges");
        self.button("ranges", &r);
        crate::ui_kit::section(ui, "Selected hypothesis");
        ui.horizontal_wrapped(|ui| {
            let r = ui.button("Previous particle");
            if self.button("prev_particle", &r) {
                self.selected = self.selected.saturating_sub(1);
            }
            let r = ui.button("Highest weight");
            if self.button("best", &r) {
                self.selected = self
                    .cloud()
                    .iter()
                    .enumerate()
                    .max_by(|a, b| a.1.w.total_cmp(&b.1.w).then(b.0.cmp(&a.0)))
                    .unwrap()
                    .0;
            }
        });
        let p = &self.cloud()[self.selected];
        ui.label(format!("Particle {} · weight {:.5}", self.selected, p.w));
        ui.small(format!("Position {:.3}, {:.3} m", p.x, p.y));
        if self.cursor % 4 >= 2 {
            let parent = if self.cursor % 4 == 3 {
                self.scan()
                    .parents
                    .as_ref()
                    .map_or(self.selected, |a| a[self.selected])
            } else {
                self.selected
            };
            let p = &self.scan().clouds[1][parent];
            let (z, x, y) = self.scan().observations[0];
            let predicted = (p.x - x).hypot(p.y - y);
            ui.small(format!("A: measured {z:.3} / predicted {predicted:.3} m"));
            ui.small(format!("Residual {:+.3} m", z - predicted));
            if self.cursor % 4 == 3 {
                if let Some(parents) = &self.scan().parents {
                    ui.colored_label(GOLD, format!("Copied from weighted particle {parent}"));
                    ui.small(format!(
                        "{} output copies of that parent",
                        parents.iter().filter(|&&a| a == parent).count()
                    ));
                } else {
                    ui.small("No resampling: population kept its weights.");
                }
            }
        }
        crate::ui_kit::legend(
            ui,
            &[
                (BLUE, "hypotheses"),
                (GOLD, "selected particle"),
                (GREEN, "true pose"),
                (CYAN, "range observation"),
            ],
        );
        crate::ui_kit::how_it_works(ui,"belief_help","Prior carries the previous scan's posterior. Predict applies the library's noisy motion model. Weight multiplies those prior weights by the enabled landmark range likelihoods and normalizes. When effective sample size falls below 52.8 of 96, systematic resampling copies actual particles with one seeded offset and gives each output weight 1/96. Otherwise Resample keeps the same population. Copies do not move; overlapping points represent multiple copies. Particle numbers refer to the current population, so an output index can have a different parent. The gold label identifies that parent. Replay advances two recorded phases per second; sensing in the experiment is every 0.25 s. Seeking never reruns the filter. Landmark or noise edits rebuild a paused experiment; separate seeded sensor and filter generators preserve the initial cloud, truth and common measurements. One landmark may leave ambiguous positions. This is a finite particle approximation, not a guarantee of accuracy. Share restores the exact settings, phase and selected index, paused.");
        let r = ui.button("Live PF / EKF");
        if self.button("live", &r) {
            self.active = false;
            self.playing = false;
        }
    }

    pub(in crate::localization) fn scene(&mut self, ui: &mut egui::Ui) {
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
        ui.heading("Belief update lab");
        ui.label("Motion spreads hypotheses. Evidence changes their weight.");
        ui.horizontal_wrapped(|ui| {
            for (i, label) in PHASES.iter().enumerate() {
                ui.colored_label(
                    if self.cursor % 4 == i {
                        GOLD
                    } else {
                        Color32::GRAY
                    },
                    format!("{} {label}", i + 1),
                );
                if i < 3 {
                    ui.weak("/");
                }
            }
        });
        let width = ui
            .available_width()
            .min((ui.ctx().screen_rect().height() - 320.).max(160.));
        let rect = Rect::from_min_size(
            ui.next_widget_position() + egui::vec2((ui.available_width() - width) / 2., 0.),
            egui::vec2(width, width),
        );
        let r = ui.allocate_rect(rect, Sense::hover());
        self.widget("map", &r);
        self.map = Some(rect);
        // Process chart selection before painting the matching cloud.
        self.weight_graph(ui);
        let phase = self.cursor % 4;
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 7., Color32::from_rgb(14, 22, 32));
        for i in 0..=11 {
            for line in [
                [[i as f64, -1.], [i as f64, 12.]],
                [[-1., i as f64], [12., i as f64]],
            ] {
                painter.line_segment(
                    [screen(rect, line[0]), screen(rect, line[1])],
                    Stroke::new(0.5_f32, Color32::from_gray(39)),
                );
            }
        }
        let mut witnesses = Vec::new();
        if phase >= 2 && self.ranges {
            for &(range, x, y) in &self.scan().observations {
                let c = screen(rect, [x, y]);
                let radius = (range / 13.) as f32 * rect.width();
                painter.circle_stroke(c, radius, Stroke::new(1.0_f32, CYAN.gamma_multiply(0.32)));
                witnesses.push(format!("{{\"kind\":\"range\",\"world\":[{x},{y}],\"pixel\":{:?},\"radius\":{radius},\"range\":{range}}}",[c.x,c.y]));
            }
        }
        for (i, p) in LANDMARKS.iter().enumerate() {
            let c = screen(rect, *p);
            let color = if i < self.landmarks {
                GOLD
            } else {
                Color32::from_gray(80)
            };
            painter.circle_filled(c, 4., color);
            painter.text(
                c + egui::vec2(7., -7.),
                egui::Align2::LEFT_BOTTOM,
                (b'A' + i as u8) as char,
                egui::FontId::proportional(12.),
                color,
            );
        }
        let mut estimate = [0., 0.];
        for (i, p) in self.cloud().iter().enumerate() {
            let c = screen(rect, [p.x, p.y]);
            let radius = (2. + (p.w * N as f64).sqrt() as f32).min(7.);
            painter.circle_filled(
                c,
                radius,
                BLUE.gamma_multiply((0.25 + p.w as f32 * N as f32 * 0.18).min(1.)),
            );
            witnesses.push(format!("{{\"kind\":\"particle\",\"index\":{i},\"state\":{:?},\"pixel\":{:?},\"radius\":{radius}}}",values(p),[c.x,c.y]));
            estimate[0] += p.w * p.x;
            estimate[1] += p.w * p.y;
        }
        let selected = &self.cloud()[self.selected];
        let c = screen(rect, [selected.x, selected.y]);
        painter.circle_stroke(c, 10., Stroke::new(2.0_f32, GOLD));
        painter.text(
            c + egui::vec2(12., -10.),
            egui::Align2::LEFT_BOTTOM,
            format!("#{}", self.selected),
            egui::FontId::proportional(12.),
            GOLD,
        );
        let truth = self.truth();
        let c = screen(rect, [truth[0], truth[1]]);
        painter.circle_stroke(c, 7., Stroke::new(2.0_f32, GREEN));
        painter.line_segment(
            [
                c,
                c + egui::vec2(truth[2].cos() as f32, -truth[2].sin() as f32) * 14.,
            ],
            Stroke::new(2.0_f32, GREEN),
        );
        let e = screen(rect, estimate);
        painter.circle_stroke(e, 7., Stroke::new(2.0_f32, Color32::WHITE));
        witnesses.push(format!(
            "{{\"kind\":\"estimate\",\"world\":{estimate:?},\"pixel\":{:?}}}",
            [e.x, e.y]
        ));
        witnesses.push(format!(
            "{{\"kind\":\"truth\",\"world\":{:?},\"pixel\":{:?}}}",
            [truth[0], truth[1]],
            [c.x, c.y]
        ));
        self.witnesses = format!("[{}]", witnesses.join(","));
        self.painted = Some((self.generation, self.cursor, self.selected, self.ranges));
        ui.label(format!(
            "Scan {}/24 · {} · ESS {:.1} / 96",
            self.cursor / 4 + 1,
            PHASES[phase],
            ess(self.cloud())
        ));
        if phase >= 2 {
            let parents = &self.scan().parents;
            ui.colored_label(
                GOLD,
                if let Some(a) = parents {
                    let mut unique = a.clone();
                    unique.sort_unstable();
                    unique.dedup();
                    format!(
                        "Weighted ESS {:.1} < 52.8 / {} distinct parents",
                        self.scan().ess,
                        unique.len()
                    )
                } else {
                    format!(
                        "Weighted ESS {:.1} >= 52.8 / keep population",
                        self.scan().ess
                    )
                },
            );
        } else {
            ui.small("White ring: weighted mean · green: true robot");
        }
        ui.small("Tap or drag the weight bars to inspect a hypothesis.");
    }

    fn weight_graph(&mut self, ui: &mut egui::Ui) {
        ui.small("Normalized particle weights / current phase");
        let (r, response) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), 70.),
            Sense::click_and_drag(),
        );
        self.graph = Some(r);
        self.widget("graph", &response);
        if response.clicked() || response.dragged() || response.drag_stopped() {
            if let Some(p) = response.interact_pointer_pos() {
                self.selected =
                    (((p.x - r.left()) / r.width()).clamp(0., 0.999999) * N as f32) as usize;
                self.playing = false;
                self.clock = None;
                self.pending = 0.;
                if response.clicked() || response.drag_stopped() {
                    self.ack("graph", &response.ctx);
                }
            }
        }
        let painter = ui.painter_at(r);
        painter.rect_filled(r, 4., Color32::from_rgb(10, 16, 24));
        let max = self
            .cloud()
            .iter()
            .map(|p| p.w)
            .fold(1. / N as f64, f64::max);
        let mut bars = Vec::new();
        for (i, p) in self.cloud().iter().enumerate() {
            let x = r.left() + i as f32 / N as f32 * r.width();
            let h = (p.w / max) as f32 * (r.height() - 6.);
            let b = Rect::from_min_max(
                egui::pos2(x, r.bottom() - h),
                egui::pos2(x + r.width() / N as f32, r.bottom()),
            );
            painter.rect_filled(b, 0., if i == self.selected { GOLD } else { BLUE });
            bars.push(format!(
                "[{i},{},{},{},{},{}]",
                p.w,
                b.left(),
                b.top(),
                b.width(),
                b.height()
            ));
        }
        self.bars = format!("[{}]", bars.join(","));
    }
}
