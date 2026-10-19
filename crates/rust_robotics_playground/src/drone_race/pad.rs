//! Held keys and a multi-touch flight pad. Touch positions are tracked by ID,
//! so steering and climbing can be held simultaneously on a phone.

use std::collections::BTreeMap;

use egui::{Color32, FontId, Pos2, Rect, Stroke, TouchId, Vec2};

use super::{manual::PilotInput, renderer::CYAN};

#[derive(Default)]
pub struct FlightPad {
    touches: BTreeMap<TouchId, Pos2>,
}

pub fn buttons(rect: Rect) -> [(&'static str, Rect); 6] {
    let side = if rect.width() < 500.0 { 44.0 } else { 42.0 };
    let gap = side + 6.0;
    let origin = rect.left_bottom() + Vec2::new(18.0, -222.0);
    let button = |position| Rect::from_min_size(position, Vec2::splat(side));
    [
        ("W", button(origin + Vec2::new(gap, 0.0))),
        ("S", button(origin + Vec2::new(gap, gap))),
        ("A", button(origin + Vec2::new(0.0, gap))),
        ("D", button(origin + Vec2::new(gap * 2.0, gap))),
        (
            "E / UP",
            button(egui::pos2(rect.right() - 18.0 - side, origin.y)),
        ),
        (
            "Q / DOWN",
            button(egui::pos2(rect.right() - 18.0 - side, origin.y + gap)),
        ),
    ]
}

pub fn actions(rect: Rect, cinema: bool) -> [Rect; 3] {
    let top = if rect.width() < 600.0 {
        rect.left_top() + Vec2::new(16.0, if cinema { 180.0 } else { 106.0 })
    } else {
        rect.right_top() + Vec2::new(-258.0, if cinema { 122.0 } else { 50.0 })
    };
    std::array::from_fn(|i| {
        Rect::from_min_size(top + Vec2::new(i as f32 * 86.0, 0.0), Vec2::new(80.0, 28.0))
    })
}

impl FlightPad {
    pub fn clear(&mut self) {
        self.touches.clear();
    }

    pub fn read(&mut self, ctx: &egui::Context, rect: Rect, enabled: bool) -> PilotInput {
        let keyboard = !ctx.wants_keyboard_input();
        ctx.input(|input| {
            if !input.focused || !enabled {
                self.touches.clear();
                return PilotInput::default();
            }
            for event in &input.events {
                if let egui::Event::Touch { id, phase, pos, .. } = event {
                    match phase {
                        egui::TouchPhase::Start | egui::TouchPhase::Move => {
                            self.touches.insert(*id, *pos);
                        }
                        egui::TouchPhase::End | egui::TouchPhase::Cancel => {
                            self.touches.remove(id);
                        }
                    }
                }
            }
            let buttons = buttons(rect);
            let held = |index: usize, keys: &[egui::Key]| {
                let touch = self.touches.values().any(|p| buttons[index].1.contains(*p));
                let mouse = self.touches.is_empty()
                    && input.pointer.primary_down()
                    && input
                        .pointer
                        .interact_pos()
                        .is_some_and(|p| buttons[index].1.contains(p));
                i8::from(
                    touch || mouse || (keyboard && keys.iter().any(|key| input.key_down(*key))),
                )
            };
            PilotInput::new(
                held(0, &[egui::Key::W, egui::Key::ArrowUp])
                    - held(1, &[egui::Key::S, egui::Key::ArrowDown]),
                held(3, &[egui::Key::D, egui::Key::ArrowRight])
                    - held(2, &[egui::Key::A, egui::Key::ArrowLeft]),
                held(4, &[egui::Key::E]) - held(5, &[egui::Key::Q]),
            )
        })
    }

    pub fn paint(ui: &mut egui::Ui, rect: Rect, input: PilotInput, enabled: bool) {
        let active = [
            input.forward > 0,
            input.forward < 0,
            input.right < 0,
            input.right > 0,
            input.lift > 0,
            input.lift < 0,
        ];
        for (i, (label, button)) in buttons(rect).into_iter().enumerate() {
            // Capture drags here to keep the phone's surrounding ScrollArea
            // from scrolling while a finger holds a flight command.
            ui.interact(button, ui.id().with(("flight-pad", i)), egui::Sense::drag());
            let painter = ui.painter();
            painter.rect_filled(
                button,
                9.0,
                if active[i] {
                    CYAN.gamma_multiply(0.25)
                } else {
                    Color32::from_rgba_unmultiplied(12, 24, 43, 220)
                },
            );
            painter.rect_stroke(
                button,
                9.0,
                Stroke::new(
                    1.0_f32,
                    if active[i] {
                        CYAN
                    } else {
                        CYAN.gamma_multiply(if enabled { 0.4 } else { 0.12 })
                    },
                ),
                egui::StrokeKind::Inside,
            );
            painter.text(
                button.center(),
                egui::Align2::CENTER_CENTER,
                label,
                FontId::monospace(if i >= 4 { 8.0 } else { 15.0 }),
                if enabled { CYAN } else { Color32::GRAY },
            );
        }
        ui.painter().text(
            rect.center_bottom() + Vec2::new(0.0, -110.0),
            egui::Align2::CENTER_BOTTOM,
            if enabled {
                "HOLD TO FLY / RELEASE TO BRAKE"
            } else {
                "RECORDED FLIGHT / R TO TRY AGAIN"
            },
            FontId::monospace(9.0),
            Color32::from_rgb(158, 180, 201),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simultaneous_touch_commands_release_and_focus_loss_clears_the_pad() {
        let ctx = egui::Context::default();
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(390.0, 650.0));
        let positions = buttons(rect);
        let mut pad = FlightPad::default();
        let touch = |id, phase, index: usize| egui::Event::Touch {
            device_id: egui::TouchDeviceId(1),
            id: TouchId(id),
            phase,
            pos: positions[index].1.center(),
            force: None,
        };
        let mut run = |events: Vec<egui::Event>, focused| {
            ctx.begin_pass(egui::RawInput {
                events,
                focused,
                ..Default::default()
            });
            let command = pad.read(&ctx, rect, true);
            let _ = ctx.end_pass();
            command
        };
        assert_eq!(
            run(
                vec![
                    touch(1, egui::TouchPhase::Start, 0),
                    touch(2, egui::TouchPhase::Start, 4)
                ],
                true
            ),
            PilotInput::new(1, 0, 1)
        );
        assert_eq!(
            run(vec![touch(1, egui::TouchPhase::End, 0)], true),
            PilotInput::new(0, 0, 1)
        );
        assert_eq!(run(Vec::new(), false), PilotInput::default());
        assert_eq!(run(Vec::new(), true), PilotInput::default());
    }
}
