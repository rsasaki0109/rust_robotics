use super::{ElbowBranch, TrackingComparison, TwoJointArm, DT, LAST, SPEED_LIMIT};
use crate::arm::ArmDemo;
use egui::{Color32, Rect, Sense, Shape, Stroke};

const JOINT: Color32 = Color32::from_rgb(76, 216, 203);
const CARTESIAN: Color32 = Color32::from_rgb(255, 151, 101);
const REQUEST: Color32 = Color32::from_rgb(255, 206, 106);

impl TrackingComparison {
    fn acknowledge(&mut self, name: &'static str, ctx: &egui::Context) {
        self.action_count += 1;
        self.last_action = name;
        ctx.request_repaint();
    }

    fn widget(&mut self, name: &'static str, r: &egui::Response) {
        self.actions.insert(name, r.rect.center().into());
        self.hits.insert(name, r.hovered());
    }

    fn button(&mut self, name: &'static str, r: &egui::Response) -> bool {
        self.widget(name, r);
        if r.clicked() {
            self.acknowledge(name, &r.ctx);
            true
        } else {
            false
        }
    }

    pub fn mode_button(&mut self, name: &'static str, r: &egui::Response) {
        self.button(name, r);
    }

    pub fn controls(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Compare tracking");
        crate::ui_kit::hint(
            ui,
            "Same start, target, gain and joint-speed budget. Two different ways to move.",
        );
        ui.label(format!("Both joint-rate limits: {SPEED_LIMIT:.1} rad/s"));
        ui.horizontal_wrapped(|ui| {
            for (key, title) in [
                ("reach", "Reach"),
                ("near", "Near straight"),
                ("straight", "Straight"),
                ("outside", "Outside"),
            ] {
                let r = ui.button(title);
                if self.button(key, &r) {
                    self.preset(key);
                }
            }
        });
        ui.label(format!(
            "Target x {:.2}, y {:.2} m",
            self.config.target[0], self.config.target[1]
        ));
        let reachable = TwoJointArm::inverse_kinematics(
            self.config.target[0],
            self.config.target[1],
            self.config.branch,
        )
        .is_some();
        if !reachable {
            ui.colored_label(CARTESIAN, "Outside workspace: both arms hold");
        }
        ui.horizontal_wrapped(|ui| {
            for (key, title, branch) in [
                ("positive", "Joint bend +", ElbowBranch::Positive),
                ("negative", "Joint bend -", ElbowBranch::Negative),
            ] {
                let r = ui.selectable_label(self.config.branch == branch, title);
                if self.button(key, &r) {
                    let mut c = self.config.clone();
                    c.branch = branch;
                    self.rebuild(c);
                }
            }
        });
        crate::ui_kit::hint(ui,"The branch selects the joint target. Cartesian DLS follows local tip error and chooses its own branch.");
        ui.label("Cartesian damping");
        let mut damping = self.config.damping;
        let r = ui.add(
            egui::Slider::new(&mut damping, 0.02..=1.0)
                .fixed_decimals(2)
                .clamping(egui::SliderClamping::Edits),
        );
        self.widget("damping", &r);
        if r.changed() {
            let mut c = self.config.clone();
            c.damping = damping;
            self.rebuild(c);
            ctx.request_repaint();
        }
        if r.clicked() || r.drag_stopped() {
            self.acknowledge("damping", ctx);
        }
        ui.horizontal_wrapped(|ui| {
            for (key, title, value) in [("low", "Low 0.05", 0.05), ("high", "High 0.60", 0.6)] {
                let r = ui.selectable_label(self.config.damping == value, title);
                if self.button(key, &r) {
                    let mut c = self.config.clone();
                    c.damping = value;
                    self.rebuild(c);
                }
            }
        });
        crate::ui_kit::section(ui, "Inspect the same time");
        ui.horizontal_wrapped(|ui| {
            let r = ui.button(if self.playing { "Pause" } else { "Play" });
            if self.button("play", &r) {
                if self.frame == LAST {
                    self.seek(0);
                }
                self.playing = !self.playing;
                self.last_time = None;
            }
            for (key, title, frame) in [
                ("start", "Start", 0),
                ("step", "+0.01 s", (self.frame + 1).min(LAST)),
                ("result", "Result", LAST),
            ] {
                let r = ui.button(title);
                if self.button(key, &r) {
                    self.seek(frame);
                }
            }
        });
        let mut frame = self.frame;
        let r = ui.add(egui::Slider::new(&mut frame, 0..=LAST).text("step"));
        self.widget("cursor", &r);
        if r.changed() {
            self.seek(frame);
            ctx.request_repaint();
        }
        if r.clicked() || r.drag_stopped() {
            self.acknowledge("cursor", ctx);
        }
        ui.label(format!(
            "t {:.2} / 6.00 s · {}",
            self.frame as f64 * DT,
            if self.playing { "Playing" } else { "Paused" }
        ));
        ui.label(if self.frame < LAST {
            "Command for the next 0.01 s:"
        } else {
            "Run ended: commands are zero."
        });
        for (i, color, title) in [(0, JOINT, "Joint target"), (1, CARTESIAN, "Cartesian DLS")] {
            let s = self.frames[self.frame][i];
            ui.colored_label(
                color,
                egui::RichText::new(format!("{title} · error {:.3} m", s.error)).strong(),
            );
            ui.label(format!(
                "Rates {:.2}, {:.2} rad/s{}",
                s.rates[0],
                s.rates[1],
                if s.limited { " · limited" } else { "" }
            ));
            ui.label(format!(
                "Tip {:.2}, {:.2} m/s · |det J| {:.3}",
                s.velocity[0],
                s.velocity[1],
                s.determinant.abs()
            ));
        }
        crate::ui_kit::legend(
            ui,
            &[
                (JOINT, "joint target"),
                (CARTESIAN, "Cartesian DLS"),
                (REQUEST, "DLS requested velocity"),
            ],
        );
        crate::ui_kit::how_it_works(ui,"arm_tracking_help","Both runs start at the same joint pose. Gain is 4, time step is 0.01 s, and the Euclidean joint-rate limit is 2 rad/s for both. Joint target follows the selected analytic IK angles. Cartesian DLS uses the damped Jacobian inverse on tip error. Damping changes only DLS; branch selection changes only joint tracking. At a straight pose, a purely radial error has no first-order Jacobian response: DLS can stall even for a reachable target. Both reject goals outside the workspace. These are complete recorded 6 s kinematic runs, without inertia or obstacles. The graph contains both complete runs; its cursor selects one shared time. Arrows show instantaneous tip velocities at that pose, with a 0.2 s display scale, rather than an integrated path. Share reconstructs both runs at the same paused step.");
    }

    pub fn scene(&mut self, _ctx: &egui::Context, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.label(
                egui::RichText::new("Tracking comparison")
                    .strong()
                    .size(20.0),
            );
        });
        let rect = crate::ui_kit::fit_rect(ui, 1.0, 210.0);
        let r = ui.allocate_rect(rect, Sense::click_and_drag());
        self.rect = Some(rect);
        self.widget("aim", &r);
        if r.clicked() || r.dragged() {
            if let Some(p) = r.interact_pointer_pos() {
                let scale = f64::from(rect.width()) / 5.2;
                let mut c = self.config.clone();
                c.target = [
                    (f64::from(p.x - rect.center().x) / scale).clamp(-2.5, 2.5),
                    (f64::from(rect.center().y - p.y) / scale).clamp(-2.5, 2.5),
                ];
                self.rebuild(c);
                r.ctx.request_repaint();
            }
        }
        if r.clicked() || r.drag_stopped() {
            self.acknowledge("aim", &r.ctx);
        }
        // Handle the graph before drawing either arm at its selected time.
        self.error_graph(ui);
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 10.0, Color32::from_rgb(17, 25, 35));
        // A rectangular camera uses the same world scale on both axes.
        let screen = |p: [f64; 2]| ArmDemo::screen(rect, p);
        painter.circle_stroke(
            screen([0.0, 0.0]),
            2.0 * rect.width() / 5.2,
            Stroke::new(1.0_f32, Color32::from_gray(65)),
        );
        for i in -5..=5 {
            let x = f64::from(i) * 0.5;
            painter.line_segment(
                [screen([x, -1.8]), screen([x, 1.8])],
                Stroke::new(0.6_f32, Color32::from_gray(36)),
            );
            painter.line_segment(
                [screen([-2.6, x]), screen([2.6, x])],
                Stroke::new(0.6_f32, Color32::from_gray(36)),
            );
        }
        for (controller, color) in [(0, JOINT), (1, CARTESIAN)] {
            let points = self.frames[..=self.frame]
                .iter()
                .map(|p| screen(p[controller].tip))
                .collect::<Vec<_>>();
            if points.len() > 1 {
                painter.add(Shape::line(
                    points,
                    Stroke::new(2.0_f32, color.gamma_multiply(0.6)),
                ));
            }
        }
        for (controller, color) in [(0, JOINT), (1, CARTESIAN)] {
            let s = self.frames[self.frame][controller];
            ArmDemo::draw_pose(&painter, rect, s.angles, color, true);
            let at = screen(s.tip);
            if controller == 1 {
                let end = screen([
                    s.tip[0] + 0.2 * s.requested[0],
                    s.tip[1] + 0.2 * s.requested[1],
                ]);
                painter.extend(Shape::dashed_line(
                    &[at, end],
                    Stroke::new(2.0_f32, REQUEST),
                    5.0,
                    4.0,
                ));
            }
            let velocity = screen([
                s.tip[0] + 0.2 * s.velocity[0],
                s.tip[1] + 0.2 * s.velocity[1],
            ]) - at;
            if velocity.length() > 1.0 {
                painter.arrow(at, velocity, Stroke::new(2.5_f32, color));
            }
        }
        let goal = screen(self.config.target);
        painter.circle_stroke(goal, 10.0, Stroke::new(2.0_f32, Color32::WHITE));
        painter.line_segment(
            [goal - egui::vec2(14.0, 0.0), goal + egui::vec2(14.0, 0.0)],
            Stroke::new(1.0_f32, Color32::WHITE),
        );
        painter.line_segment(
            [goal - egui::vec2(0.0, 14.0), goal + egui::vec2(0.0, 14.0)],
            Stroke::new(1.0_f32, Color32::WHITE),
        );
        crate::ui_kit::overlay_text(ui, rect, "Drag the target to restart both recorded runs");
        self.painted_frame = Some(self.frame);
        self.painted_generation = Some(self.generation);
        ui.horizontal_wrapped(|ui| {
            ui.label(format!("t {:.2} s", self.frame as f64 * DT));
            for (i, color, title) in [(0, JOINT, "Joint target"), (1, CARTESIAN, "Cartesian DLS")] {
                ui.colored_label(
                    color,
                    format!("{title}: {:.3} m", self.frames[self.frame][i].error),
                );
            }
        });
        crate::ui_kit::hint(
            ui,
            "Full 6 s runs · tap the error graph to inspect one shared time",
        );
    }

    fn error_graph(&mut self, ui: &mut egui::Ui) {
        let (rect, r) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), 118.0),
            Sense::click_and_drag(),
        );
        self.widget("graph", &r);
        let plot = Rect::from_min_max(
            rect.min + egui::vec2(34.0, 12.0),
            rect.max - egui::vec2(10.0, 22.0),
        );
        self.graph = Some(plot);
        if r.clicked() || r.dragged() {
            if let Some(p) = r.interact_pointer_pos() {
                self.seek(
                    (((p.x - plot.left()) / plot.width()).clamp(0.0, 1.0) * LAST as f32).round()
                        as usize,
                );
                r.ctx.request_repaint();
            }
        }
        if r.clicked() || r.drag_stopped() {
            self.acknowledge("graph", &r.ctx);
        }
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 6.0, Color32::from_rgb(17, 25, 35));
        let max = self
            .frames
            .iter()
            .flat_map(|p| p.iter().map(|s| s.error))
            .fold(0.05_f64, f64::max)
            * 1.08;
        let point = |frame: usize, error: f64| {
            egui::pos2(
                plot.left() + plot.width() * frame as f32 / LAST as f32,
                plot.bottom() - plot.height() * (error / max) as f32,
            )
        };
        for (controller, color) in [(0, JOINT), (1, CARTESIAN)] {
            painter.add(Shape::line(
                self.frames
                    .iter()
                    .enumerate()
                    .map(|(frame, p)| point(frame, p[controller].error))
                    .collect(),
                Stroke::new(1.8_f32, color),
            ));
            painter.circle_filled(
                point(self.frame, self.frames[self.frame][controller].error),
                3.5,
                color,
            );
        }
        let x = point(self.frame, 0.0).x;
        painter.line_segment(
            [egui::pos2(x, plot.top()), egui::pos2(x, plot.bottom())],
            Stroke::new(1.0_f32, Color32::WHITE),
        );
        painter.text(
            rect.left_top() + egui::vec2(5.0, 2.0),
            egui::Align2::LEFT_TOP,
            "Error / m",
            egui::FontId::proportional(11.0),
            Color32::from_gray(190),
        );
        for frame in [0, LAST / 2, LAST] {
            painter.text(
                egui::pos2(point(frame, 0.0).x, plot.bottom() + 5.0),
                egui::Align2::CENTER_TOP,
                format!("{:.0} s", frame as f64 * DT),
                egui::FontId::proportional(11.0),
                Color32::from_gray(170),
            );
        }
    }

    pub fn finish(&mut self, ctx: &egui::Context, active: bool) {
        self.revision += 1;
        self.publish(ctx, active);
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish(&mut self, _ctx: &egui::Context, _active: bool) {
        let _ = self.published_generation;
    }

    #[cfg(target_arch = "wasm32")]
    fn publish(&mut self, ctx: &egui::Context, active: bool) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let set = |key: &str, value: &str| {
            let _ = canvas.set_attribute(&format!("data-arm-cmp-{key}"), value);
        };
        let pair = |p: [f64; 2]| format!("[{},{}]", p[0], p[1]);
        let actions = self
            .actions
            .iter()
            .map(|(k, p)| format!("\"{k}\":[{},{}]", p[0], p[1]))
            .collect::<Vec<_>>()
            .join(",");
        let hits = self
            .hits
            .iter()
            .map(|(k, v)| format!("\"{k}\":{v}"))
            .collect::<Vec<_>>()
            .join(",");
        set("active", if active { "1" } else { "0" });
        set("actions", &format!("{{{actions}}}"));
        set("hits", &format!("{{{hits}}}"));
        set("action-count", &self.action_count.to_string());
        set("last-action", self.last_action);
        set("ui-revision", &self.revision.to_string());
        set(
            "pointer",
            &ctx.input(|i| i.pointer.hover_pos())
                .map_or_else(|| "null".into(), |p| format!("[{},{}]", p.x, p.y)),
        );
        if !active {
            return;
        }
        if self.published_generation != Some(self.generation) {
            set("errors", &self.errors_json);
            self.published_generation = Some(self.generation);
        }
        let samples=self.frames[self.frame].map(|s|format!("{{\"angles\":{},\"tip\":{},\"error\":{},\"rates\":{},\"velocity\":{},\"requested\":{},\"determinant\":{},\"limited\":{}}}",pair(s.angles),pair(s.tip),s.error,pair(s.rates),pair(s.velocity),pair(s.requested),s.determinant,s.limited));
        for (key, value) in [
            ("frame", self.frame.to_string()),
            ("last", LAST.to_string()),
            ("dt", DT.to_string()),
            ("limit", SPEED_LIMIT.to_string()),
            ("playing", u8::from(self.playing).to_string()),
            ("samples", format!("[{},{}]", samples[0], samples[1])),
            ("start", pair(self.config.start)),
            ("target", pair(self.config.target)),
            ("damping", self.config.damping.to_string()),
            (
                "branch",
                if self.config.branch == ElbowBranch::Positive {
                    "positive"
                } else {
                    "negative"
                }
                .into(),
            ),
            ("generation", self.generation.to_string()),
            (
                "painted-frame",
                self.painted_frame
                    .map_or_else(|| "null".into(), |v| v.to_string()),
            ),
            (
                "painted-generation",
                self.painted_generation
                    .map_or_else(|| "null".into(), |v| v.to_string()),
            ),
            ("share", self.share_query()),
            (
                "rect",
                self.rect.map_or_else(
                    || "null".into(),
                    |r| format!("[{},{},{},{}]", r.left(), r.top(), r.width(), r.height()),
                ),
            ),
            (
                "graph",
                self.graph.map_or_else(
                    || "null".into(),
                    |r| format!("[{},{},{},{}]", r.left(), r.top(), r.width(), r.height()),
                ),
            ),
        ] {
            set(key, &value);
        }
    }
}
