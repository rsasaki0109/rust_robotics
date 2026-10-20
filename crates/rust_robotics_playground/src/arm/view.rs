use super::{ArmDemo, ElbowBranch, TwoJointArm};
use egui::{Color32, Pos2, Rect, Sense, Shape, Stroke};

const ARM: Color32 = Color32::from_rgb(76, 216, 203);
const GOAL: Color32 = Color32::from_rgb(255, 196, 92);
const OTHER: Color32 = Color32::from_rgb(167, 147, 255);
const OUTSIDE: Color32 = Color32::from_rgb(255, 120, 126);

impl ArmDemo {
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

    pub fn controls(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            let r = ui.selectable_label(!self.comparison_mode, "Live arm");
            self.comparison.mode_button("live", &r);
            if self.button("live", &r) {
                self.comparison_mode = false;
            }
            let r = ui.selectable_label(self.comparison_mode, "Compare tracking");
            self.comparison.mode_button("compare", &r);
            if self.button("compare", &r) && !self.comparison_mode {
                self.comparison.capture_pose(
                    [self.arm.theta1, self.arm.theta2],
                    self.target(),
                    self.branch,
                );
                self.comparison_mode = true;
                self.playing = false;
                self.last_time = None;
            }
        });
        if self.comparison_mode {
            self.comparison.controls(ctx, ui);
            return;
        }
        crate::ui_kit::section(ui, "Try a target");
        ui.horizontal_wrapped(|ui| {
            for (key, title) in [
                ("reach", "Reach"),
                ("stretch", "Stretch"),
                ("outside", "Outside"),
                ("fold", "Fold"),
            ] {
                let r = ui.button(title);
                if self.button(key, &r) {
                    self.preset(key);
                }
            }
        });
        crate::ui_kit::hint(
            ui,
            "Drag a target in the scene. Play follows it; Snap solves it instantly.",
        );
        let reachable = self.solution(self.branch).is_some();
        ui.horizontal_wrapped(|ui| {
            let r = ui.add_enabled(
                reachable && !self.joints_mode,
                egui::Button::new(if self.playing { "Pause" } else { "Play" }),
            );
            if self.button("play", &r) {
                self.playing = !self.playing;
                self.last_time = None;
            }
            let r = ui.add_enabled(
                reachable && !self.joints_mode,
                egui::Button::new("Step 0.05 s"),
            );
            if self.button("step", &r) {
                self.playing = false;
                self.last_time = None;
                self.advance(0.05);
            }
            let r = ui.add_enabled(reachable, egui::Button::new("Snap"));
            if self.button("snap", &r) {
                self.snap();
            }
        });
        ui.label(format!(
            "Target: x {:.2}, y {:.2} m",
            self.arm.target_x, self.arm.target_y
        ));
        ui.colored_label(
            if reachable { ARM } else { OUTSIDE },
            if reachable {
                "Inside reach"
            } else {
                "Outside reach - arm holds"
            },
        );

        crate::ui_kit::section(ui, "Two ways to bend");
        ui.horizontal_wrapped(|ui| {
            for (key, title, branch) in [
                ("positive", "Bend +", ElbowBranch::Positive),
                ("negative", "Bend -", ElbowBranch::Negative),
            ] {
                let r = ui.selectable_label(self.branch == branch, title);
                if self.button(key, &r) {
                    self.branch = branch;
                }
            }
            let r = ui.checkbox(&mut self.show_other, "Other solution");
            self.button("other", &r);
        });
        crate::ui_kit::hint(
            ui,
            "Bend + / - chooses the sign of the elbow angle. Both solutions reach the same target.",
        );

        crate::ui_kit::section(ui, "Move joints yourself");
        ui.horizontal_wrapped(|ui| {
            let r = ui.selectable_label(!self.joints_mode, "Target (IK)");
            if self.button("ik", &r) {
                self.joints_mode = false;
            }
            let r = ui.selectable_label(self.joints_mode, "Joints (FK)");
            if self.button("fk", &r) {
                self.joints_mode = true;
                self.playing = false;
                self.last_time = None;
            }
        });
        for (name, title, value) in [
            ("shoulder", "Shoulder angle", self.arm.theta1),
            ("elbow", "Elbow angle", self.arm.theta2),
        ] {
            let mut degrees = value.to_degrees();
            ui.label(title);
            let r = ui.add_enabled(
                self.joints_mode,
                egui::Slider::new(&mut degrees, -180.0..=180.0)
                    .suffix("°")
                    .fixed_decimals(1)
                    .clamping(egui::SliderClamping::Edits),
            );
            self.widget(name, &r);
            // Center of the actual slider rail, excluding its numeric editor.
            self.actions.insert(
                name,
                [
                    r.rect.left() + ui.spacing().slider_width * 0.5,
                    r.rect.center().y,
                ],
            );
            if r.changed() {
                if name == "shoulder" {
                    self.arm.theta1 = degrees.to_radians();
                } else {
                    self.arm.theta2 = degrees.to_radians();
                }
                self.clear_motion();
                ctx.request_repaint();
            }
            if r.clicked() || r.drag_stopped() {
                self.acknowledge(name, ctx);
            }
        }
        ui.label(format!(
            "Tip: x {:.2}, y {:.2} m",
            self.tip()[0],
            self.tip()[1]
        ));
        ui.label(format!("Target error: {:.3} m", self.error()));
        ui.label(format!("|det J|: {:.3} m^2", self.determinant().abs()));
        if self.determinant().abs() < 0.03 {
            ui.colored_label(GOAL, "Near singular: straight or fully folded");
        }
        let r = ui.button("Reset experiment");
        if self.button("reset", &r) {
            self.preset("reach");
            self.show_other = true;
        }
        crate::ui_kit::legend(
            ui,
            &[
                (ARM, "current arm"),
                (GOAL, "selected IK pose"),
                (OTHER, "other IK pose"),
            ],
        );
        crate::ui_kit::how_it_works(ui, "arm_help", "Two unit-length links, with unrestricted joints. The shaded disk is the reachable workspace (radius 2 m). Bend + and - are the analytic IK branches. At full stretch the branches meet; at the origin the shoulder angle is undetermined. |det J| measures local Cartesian dexterity and vanishes in both singular poses. Play tracks the selected angles with a proportional joint controller, without inertia or collision checking. Joint errors wrap to the shortest rotation. Snap and manual joint edits start a new tip trail. Share preserves the pose and target, opens paused, and starts a new trail there.");
    }

    pub(super) fn screen(rect: Rect, point: [f64; 2]) -> Pos2 {
        let scale = rect.width() / 5.2;
        egui::pos2(
            rect.center().x + point[0] as f32 * scale,
            rect.center().y - point[1] as f32 * scale,
        )
    }

    fn elbow_point(angles: [f64; 2]) -> [f64; 2] {
        let l1 = TwoJointArm::LINK_LENGTHS[0];
        [l1 * angles[0].cos(), l1 * angles[0].sin()]
    }

    fn pose_points(angles: [f64; 2]) -> [[f64; 2]; 3] {
        let elbow = Self::elbow_point(angles);
        let l2 = TwoJointArm::LINK_LENGTHS[1];
        [
            [0.0, 0.0],
            elbow,
            [
                elbow[0] + l2 * (angles[0] + angles[1]).cos(),
                elbow[1] + l2 * (angles[0] + angles[1]).sin(),
            ],
        ]
    }

    pub(super) fn draw_pose(
        painter: &egui::Painter,
        rect: Rect,
        angles: [f64; 2],
        color: Color32,
        current: bool,
    ) {
        let points = Self::pose_points(angles).map(|p| Self::screen(rect, p));
        let width: f32 = if current { 8.0 } else { 2.0 };
        for segment in points.windows(2) {
            if current {
                painter.line_segment(
                    [segment[0], segment[1]],
                    Stroke::new(13.0_f32, Color32::from_rgb(14, 24, 32)),
                );
            }
            painter.line_segment([segment[0], segment[1]], Stroke::new(width, color));
        }
        for point in points {
            painter.circle_filled(
                point,
                if current { 7.0 } else { 3.0 },
                if current {
                    Color32::from_rgb(20, 30, 42)
                } else {
                    color
                },
            );
            painter.circle_stroke(
                point,
                if current { 7.0 } else { 4.0 },
                Stroke::new(2.0_f32, color),
            );
        }
    }

    pub fn scene(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        if self.comparison_mode {
            self.comparison.scene(ctx, ui);
            return;
        }
        ui.horizontal_wrapped(|ui| {
            ui.label(egui::RichText::new("Arm lab").strong().size(20.0));
            let reachable = self.solution(self.branch).is_some();
            ui.colored_label(
                if reachable { ARM } else { OUTSIDE },
                if reachable {
                    "Reachable target"
                } else {
                    "Outside reach"
                },
            );
        });
        let rect = crate::ui_kit::fit_rect(ui, 1.0, 96.0);
        let r = ui.allocate_rect(rect, Sense::click_and_drag());
        self.rect = Some(rect);
        self.widget("aim", &r);
        if r.clicked() || r.dragged() {
            if let Some(p) = r.interact_pointer_pos() {
                let scale = f64::from(rect.width()) / 5.2;
                self.set_target([
                    (f64::from(p.x - rect.center().x) / scale).clamp(-2.5, 2.5),
                    (f64::from(rect.center().y - p.y) / scale).clamp(-2.5, 2.5),
                ]);
                r.ctx.request_repaint();
            }
        }
        if r.clicked() || r.drag_stopped() {
            self.acknowledge("aim", &r.ctx);
        }
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 12.0, Color32::from_rgb(17, 25, 35));
        let center = Self::screen(rect, [0.0, 0.0]);
        let reach = TwoJointArm::LINK_LENGTHS.iter().sum::<f64>() as f32 * rect.width() / 5.2;
        painter.circle_filled(center, reach, ARM.gamma_multiply(0.065));
        painter.circle_stroke(
            center,
            reach,
            Stroke::new(1.5_f32, ARM.gamma_multiply(0.45)),
        );
        for i in -5..=5 {
            let x = f64::from(i) * 0.5;
            let color = if i == 0 {
                Color32::from_gray(65)
            } else {
                Color32::from_gray(36)
            };
            painter.line_segment(
                [Self::screen(rect, [x, -2.5]), Self::screen(rect, [x, 2.5])],
                Stroke::new(0.7_f32, color),
            );
            painter.line_segment(
                [Self::screen(rect, [-2.5, x]), Self::screen(rect, [2.5, x])],
                Stroke::new(0.7_f32, color),
            );
        }
        for (p, text) in [
            ([2.15, -0.16], "x / m"),
            ([0.3, 2.3], "y / m"),
            ([-1.8, -1.8], "reach 2 m"),
        ] {
            painter.text(
                Self::screen(rect, p),
                egui::Align2::CENTER_CENTER,
                text,
                egui::FontId::proportional(11.0),
                Color32::from_gray(160),
            );
        }
        if self.trail.len() > 1 {
            painter.add(Shape::line(
                self.trail.iter().map(|p| Self::screen(rect, *p)).collect(),
                Stroke::new(1.5_f32, ARM.gamma_multiply(0.55)),
            ));
        }
        if self.show_other {
            if let Some(angles) = self.solution(self.other_branch()) {
                Self::draw_pose(&painter, rect, angles, OTHER.gamma_multiply(0.65), false);
            }
        }
        if let Some(angles) = self.solution(self.branch) {
            Self::draw_pose(&painter, rect, angles, GOAL.gamma_multiply(0.8), false);
        }
        let goal = Self::screen(rect, self.target());
        let tip = Self::screen(rect, self.tip());
        painter.extend(Shape::dashed_line(
            &[tip, goal],
            Stroke::new(1.0_f32, GOAL.gamma_multiply(0.6)),
            4.0,
            5.0,
        ));
        Self::draw_pose(
            &painter,
            rect,
            [self.arm.theta1, self.arm.theta2],
            ARM,
            true,
        );
        painter.circle_filled(tip, 3.0, Color32::WHITE);
        let color = if self.solution(self.branch).is_some() {
            GOAL
        } else {
            OUTSIDE
        };
        painter.circle_stroke(goal, 12.0, Stroke::new(2.0_f32, color));
        painter.line_segment(
            [goal - egui::vec2(17.0, 0.0), goal + egui::vec2(17.0, 0.0)],
            Stroke::new(1.5_f32, color),
        );
        painter.line_segment(
            [goal - egui::vec2(0.0, 17.0), goal + egui::vec2(0.0, 17.0)],
            Stroke::new(1.5_f32, color),
        );
        crate::ui_kit::overlay_text(ui, rect, "Drag a target. Explore two ways to reach it.");
        ui.horizontal_wrapped(|ui| {
            ui.label(format!(
                "{} · error {:.3} m",
                if self.playing {
                    "Tracking"
                } else if self.joints_mode {
                    "Manual joints"
                } else {
                    "Paused"
                },
                self.error()
            ));
            ui.label(format!("|det J| {:.3}", self.determinant().abs()));
        });
        crate::ui_kit::hint(
            ui,
            "Gold = selected solution. Purple = other solution. Cyan = current arm.",
        );
    }

    pub fn finish(&mut self, ctx: &egui::Context, active: bool) {
        self.revision += 1;
        self.publish(ctx, active && !self.comparison_mode);
        self.comparison.finish(ctx, active && self.comparison_mode);
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish(&self, _ctx: &egui::Context, _active: bool) {}

    #[cfg(target_arch = "wasm32")]
    fn publish(&self, ctx: &egui::Context, active: bool) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let pair = |p: [f64; 2]| format!("[{},{}]", p[0], p[1]);
        let solution = |v: Option<[f64; 2]>| v.map_or_else(|| "null".into(), pair);
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
        for (key, value) in [
            ("active", u8::from(active).to_string()),
            ("angles", pair([self.arm.theta1, self.arm.theta2])),
            ("target", pair(self.target())),
            ("tip", pair(self.tip())),
            (
                "elbow",
                pair(Self::elbow_point([self.arm.theta1, self.arm.theta2])),
            ),
            ("solution", solution(self.solution(self.branch))),
            ("alternate", solution(self.solution(self.other_branch()))),
            (
                "reachable",
                u8::from(self.solution(self.branch).is_some()).to_string(),
            ),
            (
                "branch",
                if self.branch == ElbowBranch::Positive {
                    "positive"
                } else {
                    "negative"
                }
                .into(),
            ),
            ("mode", if self.joints_mode { "fk" } else { "ik" }.into()),
            ("other", u8::from(self.show_other).to_string()),
            ("playing", u8::from(self.playing).to_string()),
            ("error", self.error().to_string()),
            ("determinant", self.determinant().to_string()),
            ("elapsed", self.elapsed.to_string()),
            ("steps", self.steps.to_string()),
            (
                "trail",
                format!(
                    "[{}]",
                    self.trail
                        .iter()
                        .map(|p| pair(*p))
                        .collect::<Vec<_>>()
                        .join(",")
                ),
            ),
            ("actions", format!("{{{actions}}}")),
            ("hits", format!("{{{hits}}}")),
            (
                "pointer",
                ctx.input(|i| i.pointer.hover_pos())
                    .map_or_else(|| "null".into(), |p| format!("[{},{}]", p.x, p.y)),
            ),
            ("action-count", self.action_count.to_string()),
            ("last-action", self.last_action.into()),
            ("ui-revision", self.revision.to_string()),
            ("share", self.share_query()),
            (
                "rect",
                self.rect.map_or_else(
                    || "null".into(),
                    |r| format!("[{},{},{},{}]", r.left(), r.top(), r.width(), r.height()),
                ),
            ),
        ] {
            let _ = canvas.set_attribute(&format!("data-arm-{key}"), &value);
        }
    }
}
