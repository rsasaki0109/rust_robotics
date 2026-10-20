//! A single-contact experiment using the very same solve as MPPI pushing.
use std::collections::BTreeMap;

use egui::{Color32, Pos2, Rect, Sense, Shape, Stroke, Vec2};
use rust_robotics_control::pusher_slider::{
    ContactMode, PusherCommand, PusherSliderParams, SliderState,
};

const DT: f64 = 0.1;
const BOX: Color32 = Color32::from_rgb(110, 165, 240);
const COMMAND: Color32 = Color32::from_rgb(80, 225, 235);
const STICK: Color32 = Color32::from_rgb(250, 220, 90);
const SLIDE: Color32 = Color32::from_rgb(255, 140, 70);
const FACES: [&str; 4] = ["Back", "+y", "Front", "-y"];

pub(super) struct ContactLab {
    params: PusherSliderParams,
    command: PusherCommand,
    pose: SliderState,
    trail: Vec<[f64; 2]>,
    steps: usize,
    actions: BTreeMap<&'static str, [f32; 2]>,
    hits: BTreeMap<&'static str, bool>,
    action_count: u64,
    last_action: &'static str,
    revision: u64,
    rect: Option<Rect>,
}

impl Default for ContactLab {
    fn default() -> Self {
        Self {
            params: PusherSliderParams::default(),
            command: PusherCommand::new(0.0, 0.03, 0.0),
            pose: SliderState::new(0.0, 0.0, 0.0),
            trail: vec![[0.0, 0.0]],
            steps: 0,
            actions: BTreeMap::new(),
            hits: BTreeMap::new(),
            action_count: 0,
            last_action: "",
            revision: 0,
            rect: None,
        }
    }
}

fn mode_name(mode: ContactMode) -> &'static str {
    match mode {
        ContactMode::Stick => "Sticking",
        ContactMode::SlideUp => "Sliding +t",
        ContactMode::SlideDown => "Sliding -t",
        ContactMode::Separated => "Separated",
    }
}

fn mode_color(mode: ContactMode) -> Color32 {
    match mode {
        ContactMode::Stick => STICK,
        ContactMode::SlideUp | ContactMode::SlideDown => SLIDE,
        ContactMode::Separated => Color32::GRAY,
    }
}

impl ContactLab {
    fn reset_pose(&mut self) {
        self.pose = SliderState::new(0.0, 0.0, 0.0);
        self.steps = 0;
        self.trail = vec![[0.0, 0.0]];
    }

    fn preset(&mut self, name: &str) {
        self.command = match name {
            "edge" => PusherCommand::new(0.035, 0.03, 0.0),
            "glance" => PusherCommand::new(0.0, 0.03, 0.06),
            "release" => PusherCommand::new(0.0, 0.0, 0.06),
            _ => PusherCommand::new(0.0, 0.03, 0.0),
        };
        self.params.pusher_friction = 0.3;
        self.reset_pose();
    }

    fn push(&mut self) {
        self.pose = self.params.step(self.pose, self.command, DT).0;
        self.steps += 1;
        if self.trail.len() == 1000 {
            self.trail.remove(0);
        }
        self.trail.push([self.pose.x(), self.pose.y()]);
    }

    /// Actual body-frame velocity of the box material point at the contact.
    fn contact_velocity(&self) -> [f64; 2] {
        let ([vx, vy, omega], _) = self.params.twist(self.command);
        let ([x, y], _, _) = self
            .params
            .contact_frame(self.command.face, self.command.contact);
        [vx - omega * y, vy + omega * x]
    }

    fn requested_velocity(&self) -> [f64; 2] {
        let (_, n, t) = self
            .params
            .contact_frame(self.command.face, self.command.contact);
        [
            n[0] * self.command.push_speed + t[0] * self.command.tangent_speed,
            n[1] * self.command.push_speed + t[1] * self.command.tangent_speed,
        ]
    }

    pub(super) fn action(&mut self, name: &'static str, response: &egui::Response) {
        self.actions.insert(name, response.rect.center().into());
        self.hits.insert(name, response.hovered());
        if response.clicked() {
            self.acknowledge(name);
            response.ctx.request_repaint();
        }
    }

    fn acknowledge(&mut self, name: &'static str) {
        self.action_count += 1;
        self.last_action = name;
    }

    pub(super) fn controls(&mut self, ui: &mut egui::Ui) {
        crate::ui_kit::hint(
            ui,
            "One contact. One push. See what makes a box turn or slide.",
        );
        crate::ui_kit::section(ui, "Start an experiment");
        ui.horizontal_wrapped(|ui| {
            for (key, label) in [
                ("center", "Center"),
                ("edge", "Off-center"),
                ("glance", "Glancing"),
                ("release", "Release"),
            ] {
                let r = ui.button(label);
                self.action(key, &r);
                if r.clicked() {
                    self.preset(key);
                }
            }
        });
        crate::ui_kit::section(ui, "Contact face");
        ui.horizontal_wrapped(|ui| {
            for (face, label) in FACES.iter().enumerate() {
                let r = ui.selectable_label(self.command.face == face, *label);
                self.action(["back", "plus-y", "front", "minus-y"][face], &r);
                if r.clicked() {
                    self.command.face = face;
                }
            }
        });
        let mut offset = self.command.contact * 1000.0;
        ui.label("Contact offset (mm)");
        if ui
            .add(
                egui::Slider::new(&mut offset, -45.0..=45.0)
                    .fixed_decimals(1)
                    .clamping(egui::SliderClamping::Edits),
            )
            .changed()
        {
            self.command.contact = offset / 1000.0;
        }
        crate::ui_kit::section(ui, "Pusher velocity · or drag on the box");
        let mut normal = self.command.push_speed * 1000.0;
        ui.label("Inward velocity (mm/s)");
        if ui
            .add(
                egui::Slider::new(&mut normal, 0.0..=80.0)
                    .fixed_decimals(1)
                    .clamping(egui::SliderClamping::Edits),
            )
            .changed()
        {
            self.command.push_speed = normal / 1000.0;
        }
        let mut tangent = self.command.tangent_speed * 1000.0;
        ui.label("Along-face velocity (mm/s)");
        if ui
            .add(
                egui::Slider::new(&mut tangent, -120.0..=120.0)
                    .fixed_decimals(1)
                    .clamping(egui::SliderClamping::Edits),
            )
            .changed()
        {
            self.command.tangent_speed = tangent / 1000.0;
        }
        crate::ui_kit::section(ui, "Contact friction μ");
        ui.add(
            egui::Slider::new(&mut self.params.pusher_friction, 0.05..=1.0)
                .fixed_decimals(2)
                .clamping(egui::SliderClamping::Edits),
        );
        ui.horizontal_wrapped(|ui| {
            for (name, label, value) in [("low", "Low · 0.1", 0.1), ("high", "High · 0.8", 0.8)] {
                let r = ui.selectable_label(self.params.pusher_friction == value, label);
                self.action(name, &r);
                if r.clicked() {
                    self.params.pusher_friction = value;
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            let r = ui.button("Push 0.1 s");
            self.action("push", &r);
            if r.clicked() {
                self.push();
            }
            let r = ui.button("Reset pose");
            self.action("reset-pose", &r);
            if r.clicked() {
                self.reset_pose();
            }
        });
        let ([vx, vy, w], mode) = self.params.twist(self.command);
        ui.colored_label(
            mode_color(mode),
            egui::RichText::new(mode_name(mode)).strong().size(19.0),
        );
        ui.label(format!(
            "Box: {:.1}, {:.1} mm/s · {:.1}°/s",
            vx * 1000.0,
            vy * 1000.0,
            w.to_degrees()
        ));
        ui.label(format!(
            "{} pushes · x {:.1}, y {:.1} mm · {:.1}°",
            self.steps,
            self.pose.x() * 1000.0,
            self.pose.y() * 1000.0,
            self.pose.theta().to_degrees()
        ));
        crate::ui_kit::legend(
            ui,
            &[
                (COMMAND, "pusher velocity"),
                (mode_color(mode), "box contact velocity"),
                (Color32::WHITE, "next 0.1 s"),
            ],
        );
        crate::ui_kit::how_it_works(ui, "contact_lab_help", "The velocity fan samples the single-contact quasi-static model: yellow directions stick; orange directions slide. Arrows use the same velocity scale. The white outline is one 0.1 s Euler step. Offset contact couples translation and rotation. Each push recontacts the selected face; the point pusher has no path or inertia. This lab has no obstacles. Share keeps the current pose and settings; the trail starts there.");
    }

    /// World-to-screen camera follows the box, leaving room for velocity arrows.
    fn screen(&self, rect: Rect, p: [f64; 2]) -> Pos2 {
        let scale = rect.width() / 0.32;
        rect.center()
            + Vec2::new(
                (p[0] - self.pose.x()) as f32 * scale,
                -(p[1] - self.pose.y()) as f32 * scale,
            )
    }

    fn body_point(&self, p: [f64; 2]) -> [f64; 2] {
        let (s, c) = self.pose.theta().sin_cos();
        [
            self.pose.x() + c * p[0] - s * p[1],
            self.pose.y() + s * p[0] + c * p[1],
        ]
    }

    fn arrow(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        p: [f64; 2],
        v: [f64; 2],
        color: Color32,
        width: f32,
    ) {
        let from = self.screen(rect, self.body_point(p));
        let tip = self.screen(rect, self.body_point([p[0] + v[0], p[1] + v[1]]));
        if from.distance(tip) > 0.1 {
            painter.arrow(from, tip - from, Stroke::new(width, color));
        }
    }

    fn outline(&self, rect: Rect, state: SliderState) -> Vec<Pos2> {
        let b = self.params.half_extent;
        let (s, c) = state.theta().sin_cos();
        [(-b, -b), (b, -b), (b, b), (-b, b)]
            .iter()
            .map(|&(x, y)| {
                self.screen(rect, [state.x() + c * x - s * y, state.y() + s * x + c * y])
            })
            .collect()
    }

    fn aim(&mut self, rect: Rect, position: Pos2) {
        let scale = f64::from(rect.width()) / 0.32;
        let (s, c) = self.pose.theta().sin_cos();
        let world = [
            f64::from(position.x - rect.center().x) / scale,
            -f64::from(position.y - rect.center().y) / scale,
        ];
        let (p, n, t) = self
            .params
            .contact_frame(self.command.face, self.command.contact);
        let v = [
            c * world[0] + s * world[1] - p[0],
            -s * world[0] + c * world[1] - p[1],
        ];
        // One second of velocity corresponds to one metre of arrow length.
        self.command.push_speed = (v[0] * n[0] + v[1] * n[1]).clamp(0.0, 0.08);
        self.command.tangent_speed = (v[0] * t[0] + v[1] * t[1]).clamp(-0.12, 0.12);
    }

    pub(super) fn scene(&mut self, ui: &mut egui::Ui) {
        ui.label(egui::RichText::new("Contact lab").strong().size(20.0));
        let rect = crate::ui_kit::fit_rect(ui, 0.72, 78.0);
        let response = ui.allocate_rect(rect, Sense::click_and_drag());
        self.rect = Some(rect);
        self.actions.insert("aim", rect.center().into());
        self.hits.insert("aim", response.hovered());
        if response.clicked() || response.dragged() {
            if let Some(pos) = response.interact_pointer_pos() {
                self.aim(rect, pos);
            }
        }
        if response.clicked() || response.drag_stopped() {
            self.acknowledge("aim");
        }
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 10.0, Color32::from_rgb(20, 27, 37));
        for i in -8..=8 {
            let d = f64::from(i) * 0.025;
            painter.line_segment(
                [
                    self.screen(rect, [self.pose.x() + d, self.pose.y() - 0.15]),
                    self.screen(rect, [self.pose.x() + d, self.pose.y() + 0.15]),
                ],
                Stroke::new(0.5_f32, Color32::from_gray(40)),
            );
            painter.line_segment(
                [
                    self.screen(rect, [self.pose.x() - 0.2, self.pose.y() + d]),
                    self.screen(rect, [self.pose.x() + 0.2, self.pose.y() + d]),
                ],
                Stroke::new(0.5_f32, Color32::from_gray(40)),
            );
        }
        let (p, n, t) = self
            .params
            .contact_frame(self.command.face, self.command.contact);
        // This is a velocity-direction fan. Regimes come directly from twist().
        for i in 0..80 {
            let angle = (-80.0 + f64::from(i) * 2.0).to_radians();
            let angle2 = angle + 2.0_f64.to_radians();
            let command = PusherCommand::on_face(
                self.command.face,
                self.command.contact,
                0.03,
                0.03 * (angle + 1.0_f64.to_radians()).tan(),
            );
            let color = mode_color(self.params.twist(command).1).gamma_multiply(0.24);
            let point = |a: f64| {
                self.screen(
                    rect,
                    self.body_point([
                        p[0] + 0.115 * (n[0] * a.cos() + t[0] * a.sin()),
                        p[1] + 0.115 * (n[1] * a.cos() + t[1] * a.sin()),
                    ]),
                )
            };
            painter.add(Shape::convex_polygon(
                vec![
                    self.screen(rect, self.body_point(p)),
                    point(angle),
                    point(angle2),
                ],
                color,
                Stroke::NONE,
            ));
        }
        let trail = self.trail.iter().map(|p| self.screen(rect, *p)).collect();
        painter.add(Shape::line(
            trail,
            Stroke::new(1.5_f32, BOX.gamma_multiply(0.6)),
        ));
        painter.add(Shape::convex_polygon(
            self.outline(rect, self.pose),
            BOX.gamma_multiply(0.12),
            Stroke::new(2.0_f32, BOX),
        ));
        let next = self.params.step(self.pose, self.command, DT).0;
        painter.add(Shape::closed_line(
            self.outline(rect, next),
            Stroke::new(1.5_f32, Color32::WHITE),
        ));
        self.arrow(&painter, rect, [0.0, 0.0], [0.025, 0.0], BOX, 1.5);
        let mode = self.params.twist(self.command).1;
        // Thick pusher arrow stays visible under the thinner realized velocity.
        self.arrow(&painter, rect, p, self.requested_velocity(), COMMAND, 5.0);
        self.arrow(
            &painter,
            rect,
            p,
            self.contact_velocity(),
            mode_color(mode),
            2.0,
        );
        painter.circle_filled(self.screen(rect, self.body_point(p)), 5.0, mode_color(mode));
        crate::ui_kit::overlay_text(ui, rect, "Drag to aim the cyan velocity arrow");
        ui.label(format!(
            "{} · preview +0.1 s · camera follows box",
            mode_name(mode)
        ));
        crate::ui_kit::hint(ui,"Try Glancing, then Low / High friction. Watch the two velocity arrows separate and meet.");
    }

    pub(super) fn apply_query(&mut self, query: &str) {
        *self = Self::default();
        let number = |key, min, max| {
            crate::share::value(query, key)
                .and_then(|v| v.parse::<f64>().ok())
                .filter(|v| v.is_finite() && (min..=max).contains(v))
        };
        if let Some(preset) = crate::share::value(query, "lab_preset") {
            self.preset(preset);
        }
        if let Some(face) = crate::share::bounded_usize(query, "lab_face", 3) {
            self.command.face = face;
        }
        if let Some(v) = number("lab_contact", -0.045, 0.045) {
            self.command.contact = v;
        }
        if let Some(v) = number("lab_normal", 0.0, 0.08) {
            self.command.push_speed = v;
        }
        if let Some(v) = number("lab_tangent", -0.12, 0.12) {
            self.command.tangent_speed = v;
        }
        if let Some(v) = number("lab_mu", 0.05, 1.0) {
            self.params.pusher_friction = v;
        }
        if let (Some(x), Some(y), Some(theta)) = (
            number("lab_x", -1000.0, 1000.0),
            number("lab_y", -1000.0, 1000.0),
            number("lab_theta", -1e6, 1e6),
        ) {
            self.pose = SliderState::new(x, y, theta);
        }
        self.trail = vec![[self.pose.x(), self.pose.y()]];
    }

    pub(super) fn share_query(&self) -> String {
        format!("tab=pushing&push_mode=contact&lab_face={}&lab_contact={}&lab_normal={}&lab_tangent={}&lab_mu={}&lab_x={}&lab_y={}&lab_theta={}",self.command.face,self.command.contact,self.command.push_speed,self.command.tangent_speed,self.params.pusher_friction,self.pose.x(),self.pose.y(),self.pose.theta())
    }

    pub(super) fn finish(&mut self, ctx: &egui::Context, active: bool) {
        self.revision += 1;
        self.publish(ctx, active);
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
        let ([vx, vy, w], mode) = self.params.twist(self.command);
        let next = self.params.step(self.pose, self.command, DT).0;
        let (contact, normal, tangent) = self
            .params
            .contact_frame(self.command.face, self.command.contact);
        let json = |p: &[f64]| {
            format!(
                "[{}]",
                p.iter().map(f64::to_string).collect::<Vec<_>>().join(",")
            )
        };
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
        let pointer = ctx
            .input(|i| i.pointer.hover_pos())
            .map_or_else(|| "null".into(), |p| format!("[{},{}]", p.x, p.y));
        for (key, value) in [
            ("mode", if active { "contact" } else { "delivery" }.into()),
            ("pose", json(&self.pose.pose)),
            ("next", json(&next.pose)),
            ("twist", json(&[vx, vy, w])),
            ("contact", json(&contact)),
            ("normal", json(&normal)),
            ("tangent", json(&tangent)),
            ("requested", json(&self.requested_velocity())),
            ("realized", json(&self.contact_velocity())),
            ("regime", mode_name(mode).into()),
            ("steps", self.steps.to_string()),
            ("mu", self.params.pusher_friction.to_string()),
            (
                "command",
                json(&[
                    self.command.face as f64,
                    self.command.contact,
                    self.command.push_speed,
                    self.command.tangent_speed,
                ]),
            ),
            ("actions", format!("{{{actions}}}")),
            ("hits", format!("{{{hits}}}")),
            ("pointer", pointer),
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
            let _ = canvas.set_attribute(&format!("data-lab-{key}"), &value);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn central_push_translates_and_preview_is_the_executed_step() {
        let mut lab = ContactLab::default();
        let (next, mode) = lab.params.step(lab.pose, lab.command, DT);
        assert_eq!(mode, ContactMode::Stick);
        assert_eq!(lab.contact_velocity(), lab.requested_velocity());
        lab.push();
        assert_eq!(lab.pose, next);
        assert!((lab.pose.x() - 0.003).abs() < 1e-12);
        assert_eq!(lab.pose.y(), 0.0);
        assert_eq!(lab.pose.theta(), 0.0);
    }
    #[test]
    fn off_center_push_rotates_and_friction_changes_glancing_contact() {
        let mut lab = ContactLab::default();
        lab.preset("edge");
        assert!(lab.params.twist(lab.command).0[2].abs() > 0.1);
        lab.preset("glance");
        lab.params.pusher_friction = 0.1;
        assert_eq!(lab.params.twist(lab.command).1, ContactMode::SlideUp);
        let v = lab.contact_velocity();
        assert!((v[0] - 0.03).abs() < 1e-12);
        assert!(v[1] < 0.06);
        lab.params.pusher_friction = 0.8;
        assert_eq!(lab.params.twist(lab.command).1, ContactMode::Stick);
        assert!((lab.contact_velocity()[1] - 0.06).abs() < 1e-12);
        lab.preset("release");
        lab.push();
        assert_eq!(lab.pose.pose, [0.0; 3]);
        assert_eq!(lab.params.twist(lab.command).1, ContactMode::Separated);
    }
    #[test]
    fn contact_velocity_obeys_rigid_body_kinematics_on_every_face() {
        let mut lab = ContactLab::default();
        for face in 0..4 {
            lab.command = PusherCommand::on_face(face, 0.018, 0.03, 0.0);
            let v = lab.contact_velocity();
            let request = lab.requested_velocity();
            assert_eq!(lab.params.twist(lab.command).1, ContactMode::Stick);
            for axis in 0..2 {
                assert!((v[axis] - request[axis]).abs() < 1e-12);
            }
        }
    }
    #[test]
    fn share_preserves_exact_pose_and_command_but_starts_a_new_trail() {
        let mut lab = ContactLab::default();
        lab.preset("edge");
        lab.command.face = 3;
        lab.params.pusher_friction = 0.812345678901;
        for _ in 0..7 {
            lab.push();
        }
        let mut restored = ContactLab::default();
        restored.apply_query(&lab.share_query());
        assert_eq!(restored.pose, lab.pose);
        assert_eq!(restored.command, lab.command);
        assert_eq!(restored.params, lab.params);
        assert_eq!(restored.steps, 0);
        assert_eq!(restored.trail.len(), 1);
        assert_eq!(
            restored.params.step(restored.pose, restored.command, DT),
            lab.params.step(lab.pose, lab.command, DT)
        );
    }
    #[test]
    fn malformed_queries_cannot_install_nonfinite_or_out_of_range_values() {
        let mut lab = ContactLab::default();
        lab.apply_query("lab_face=4&lab_contact=NaN&lab_normal=-1&lab_tangent=inf&lab_mu=0&lab_x=0&lab_y=NaN&lab_theta=0");
        assert_eq!(lab.command, ContactLab::default().command);
        assert_eq!(lab.pose.pose, [0.0; 3]);
        assert_eq!(lab.params.pusher_friction, 0.3);
    }
    #[test]
    fn aiming_rotates_with_the_box_and_clamps_speeds() {
        let mut lab = ContactLab::default();
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(640.0, 460.8));
        for face in 0..4 {
            lab.pose.pose[2] = 0.75;
            lab.command.face = face;
            let (p, n, t) = lab.params.contact_frame(face, 0.0);
            let tip = lab.screen(
                rect,
                lab.body_point([
                    p[0] + 0.025 * n[0] - 0.04 * t[0],
                    p[1] + 0.025 * n[1] - 0.04 * t[1],
                ]),
            );
            lab.aim(rect, tip);
            assert!((lab.command.push_speed - 0.025).abs() < 1e-8);
            assert!((lab.command.tangent_speed + 0.04).abs() < 1e-8);
        }
        lab.aim(rect, Pos2::new(-1e5, 1e5));
        assert!((0.0..=0.08).contains(&lab.command.push_speed));
        assert!(lab.command.tangent_speed.abs() <= 0.12);
    }

    #[test]
    fn displaying_precise_velocities_preserves_values_and_scene_geometry() {
        let ctx = egui::Context::default();
        crate::ui_kit::apply_style(&ctx);
        let mut lab = ContactLab::default();
        let render = |lab: &mut ContactLab| {
            let _ = ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1280.0, 900.0))),
                    ..Default::default()
                },
                |ctx| {
                    egui::SidePanel::left("controls")
                        .default_width(320.0)
                        .show(ctx, |ui| {
                            lab.controls(ui);
                        });
                    egui::CentralPanel::default().show(ctx, |ui| lab.scene(ui));
                },
            );
        };
        for _ in 0..4 {
            render(&mut lab);
        }
        let rect = lab.rect;
        lab.command.push_speed = 0.021077943615257052;
        lab.command.tangent_speed = 0.03913588524391929;
        let query = lab.share_query();
        for _ in 0..4 {
            render(&mut lab);
            assert_eq!(
                lab.share_query(),
                query,
                "formatting sliders changed the experiment"
            );
            assert_eq!(
                lab.rect, rect,
                "velocity readouts moved the scene while aiming"
            );
        }
    }
}
