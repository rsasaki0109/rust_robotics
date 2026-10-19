//! A paused, scrollable flight guide that fits both phone orientations.

use egui::{Color32, Rect, RichText, Stroke, Vec2};

use super::flight::{Flight, DT};
use super::renderer::{self, CameraMode, GhostView, ViewOptions, CYAN, GHOST, GOLD};

#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub enum Action {
    #[default]
    Open,
    Fly,
    Watch,
    Build,
    Close,
}

pub fn show(
    ctx: &egui::Context,
    ui: &mut egui::Ui,
    flight: &Flight,
    frame: usize,
    ghost: Option<&Flight>,
    replay: bool,
) -> Action {
    let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    renderer::render(
        &painter,
        rect,
        flight,
        frame,
        flight.frames[frame].state,
        frame as f64 * DT,
        &ViewOptions {
            camera: CameraMode::Course,
            orbit: None,
            trail: true,
            prediction: false,
            edit_gate: None,
            ghost: ghost.map(|g| {
                let frame = frame.min(g.last_index());
                GhostView {
                    flight: g,
                    frame,
                    state: g.frames[frame].state,
                }
            }),
        },
    );
    painter.rect_filled(rect, 0.0, Color32::from_black_alpha(130));
    let card = Rect::from_center_size(
        rect.center(),
        Vec2::new(
            (rect.width() - 24.0).min(580.0),
            (rect.height() - 24.0).min(590.0),
        ),
    );
    painter.rect_filled(card, 16.0, Color32::from_rgba_unmultiplied(10, 18, 32, 248));
    painter.rect_stroke(
        card,
        16.0,
        Stroke::new(1.0_f32, CYAN.gamma_multiply(0.5)),
        egui::StrokeKind::Inside,
    );
    let mut action = Action::Open;
    let mut buttons = Vec::new();
    ui.scope_builder(egui::UiBuilder::new().max_rect(card.shrink(18.0)), |ui| {
        let footer = 120.0 + ui.spacing().item_spacing.y * 3.0;
        egui::ScrollArea::vertical().id_salt("flight-guide-scroll").max_height((ui.available_height() - footer).max(40.0)).show(ui, |ui| {
            ui.label(RichText::new("RUST / FLIGHT LAB").color(CYAN).size(12.0));
            ui.label(RichText::new("Five gates. Your flight.").size(if card.width() < 400.0 { 23.0 } else { 30.0 }));
            ui.label("Fly through the numbered openings in order. Aim for their centers and keep moving forward.");
            ui.add_space(8.0);
            for (title, text, color) in [
                ("01 / STEER", "W / S: forward / back\nA / D: left / right · E / Q: climb / descend", CYAN),
                ("02 / ON A PHONE", "Hold the flight buttons. Use two fingers to move forward and climb together. Release to brake.", CYAN),
                ("03 / FOLLOW THE GATES", if ghost.is_some_and(|g| g.is_manual()) {
                    "The gold gate is next. The gold ghost replays the best flight for this course. Negative gate splits mean you crossed sooner. You can pass through the ghost."
                } else {
                    "The gold gate is next. The purple drone is the autopilot at your flight time. Finish once to race your best as a gold ghost next time. You can pass through it."
                }, if ghost.is_some_and(|g| g.is_manual()) { GOLD } else { GHOST }),
            ] {
                ui.label(RichText::new(title).color(color).strong().size(11.0));
                ui.label(text);
                ui.add_space(5.0);
            }
            ui.label(RichText::new(if replay {
                "This is a shared replay. Retry starts your own attempt."
            } else if frame > 0 {
                "Your flight is paused. Space or Play resumes; Retry starts again."
            } else {
                "Your clock waits for the first key or flight button. Start with forward + climb."
            }).color(GOLD));
            ui.label(RichText::new("Space: pause · R: retry · P: photo · H: this guide").small());
            ui.label(RichText::new("After flying: replay the attempt, save a photo, or use Share to send the flight to a friend.").small());
        });
            ui.add_space(ui.spacing().item_spacing.y);
            let width = ui.available_width();
            for (name, label, choice) in [
                ("fly", if replay { "Back to replay" } else if flight.is_manual() && frame > 0 { "Back to flight" } else { "Ready to fly" }, Action::Fly),
                ("watch", "Watch the autopilot first", Action::Watch),
                ("build", "Build your own circuit", Action::Build),
            ] {
                let mut button = egui::Button::new(label);
                if choice == Action::Fly { button = button.fill(CYAN.gamma_multiply(0.25)); }
                let response = ui.add_sized([width, 40.0], button);
                if response.clicked() { action = choice; }
                buttons.push((name, response.rect));
            }
    });
    if !ctx.wants_keyboard_input() && ctx.input(|input| input.key_pressed(egui::Key::Escape)) {
        action = Action::Close;
    }
    publish(card, &buttons);
    action
}

#[cfg(target_arch = "wasm32")]
fn publish(card: Rect, buttons: &[(&str, Rect)]) {
    if let Some(canvas) = web_sys::window()
        .and_then(|w| w.document())
        .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
    {
        let fields = buttons
            .iter()
            .map(|(name, r)| format!("\"{name}\":[{:.1},{:.1}]", r.center().x, r.center().y))
            .collect::<Vec<_>>()
            .join(",");
        let _ = canvas.set_attribute("data-drone-guide-actions", &format!("{{{fields}}}"));
        let _ = canvas.set_attribute(
            "data-drone-guide-rect",
            &format!(
                "[{:.1},{:.1},{:.1},{:.1}]",
                card.left(),
                card.top(),
                card.width(),
                card.height()
            ),
        );
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn publish(_card: Rect, _buttons: &[(&str, Rect)]) {}
