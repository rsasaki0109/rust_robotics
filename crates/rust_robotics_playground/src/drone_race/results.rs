//! End-of-flight card and touch-sized actions.

use egui::{Color32, FontId, Rect, Stroke, Vec2};

use super::flight::{Status, DT};
use super::renderer::{CYAN, GHOST, GOLD};
use super::{pad, DroneRaceDemo};

#[derive(PartialEq, Eq)]
pub enum Action {
    None,
    Again,
    Best,
}

fn layout(rect: Rect, manual: bool, best: bool) -> (Rect, Rect, Option<Rect>) {
    let compact = pad::compact(rect);
    let center = rect.center() + Vec2::new(0.0, if compact && manual { 30.0 } else { 0.0 });
    let height = if manual {
        if compact {
            200.0
        } else {
            240.0
        }
    } else {
        144.0
    };
    let card = Rect::from_center_size(center, Vec2::new((rect.width() - 20.0).min(380.0), height));
    let width = if best {
        (card.width() - 32.0) / 2.0
    } else {
        128.0
    };
    let y = if manual {
        if compact {
            65.0
        } else {
            73.0
        }
    } else {
        36.0
    };
    let again = Rect::from_center_size(
        center + Vec2::new(if best { -(width + 8.0) / 2.0 } else { 0.0 }, y),
        Vec2::new(width, 40.0),
    );
    let replay = best.then(|| again.translate(Vec2::new(width + 8.0, 0.0)));
    (card, again, replay)
}

pub fn show(demo: &DroneRaceDemo, ui: &mut egui::Ui, rect: Rect) -> Action {
    let current = &demo.flight.frames[demo.cursor];
    let manual = demo.flight.is_manual();
    let best = manual
        .then(|| demo.records.best(demo.flight.course))
        .flatten();
    let (card, again, replay) = layout(rect, manual, best.is_some());
    let center = card.center();
    let compact = pad::compact(rect);
    let painter = ui.painter_at(card);
    painter.rect_filled(card, 12.0, Color32::from_rgba_unmultiplied(10, 18, 34, 245));
    painter.rect_stroke(
        card,
        12.0,
        Stroke::new(1.0_f32, CYAN.gamma_multiply(0.4)),
        egui::StrokeKind::Inside,
    );
    let title = match current.status {
        Status::Finished => "COURSE COMPLETE",
        Status::GroundContact => "GROUND CONTACT",
        Status::TimeLimit => "TIME LIMIT",
        Status::Flying => "REPLAY COMPLETE",
    };
    let title_y = if manual {
        if compact {
            -70.0
        } else {
            -87.0
        }
    } else {
        -43.0
    };
    painter.text(
        center + Vec2::new(0.0, title_y),
        egui::Align2::CENTER_CENTER,
        title,
        FontId::proportional(22.0),
        CYAN,
    );
    painter.text(
        center
            + Vec2::new(
                0.0,
                if manual {
                    if compact {
                        -41.0
                    } else {
                        -52.0
                    }
                } else {
                    -8.0
                },
            ),
        egui::Align2::CENTER_CENTER,
        format!(
            "{} / {} gates  ·  {:.2} s",
            current.passed,
            demo.flight.lap.gate_count(),
            demo.cursor as f64 * DT
        ),
        FontId::monospace(14.0),
        Color32::WHITE,
    );
    if manual {
        let comparison = demo.ghost.as_ref().map_or_else(String::new, |ghost| {
            if !ghost.done() {
                "Preparing autopilot result...".to_string()
            } else if ghost.frames[ghost.last_index()].status == Status::Finished {
                let time = ghost.last_index() as f64 * DT;
                if current.status == Status::Finished {
                    format!(
                        "Autopilot {time:.2} s / You {:+.2} s",
                        demo.cursor as f64 * DT - time
                    )
                } else {
                    format!("Autopilot finished in {time:.2} s")
                }
            } else {
                "Autopilot did not finish".to_string()
            }
        });
        painter.text(
            center + Vec2::new(0.0, if compact { -18.0 } else { -25.0 }),
            egui::Align2::CENTER_CENTER,
            comparison,
            FontId::monospace(11.0),
            GHOST,
        );
        let label = if let Some(improvement) = &demo.new_best {
            improvement.previous_steps.map_or_else(
                || "NEW PERSONAL BEST".to_owned(),
                |previous| {
                    format!(
                        "NEW BEST  /  -{:.2} s",
                        (previous - demo.cursor) as f64 * DT
                    )
                },
            )
        } else if let Some(best) = best {
            if current.status == Status::Finished {
                format!(
                    "BEST {:.2} s  /  {:+.2} s",
                    best.steps() as f64 * DT,
                    (demo.cursor as f64 - best.steps() as f64) * DT
                )
            } else {
                format!("PERSONAL BEST {:.2} s", best.steps() as f64 * DT)
            }
        } else {
            "Finish all five gates to set a best".to_owned()
        };
        painter.text(
            center + Vec2::new(0.0, if compact { 6.0 } else { 3.0 }),
            egui::Align2::CENTER_CENTER,
            label,
            FontId::monospace(11.0),
            GOLD,
        );
        if best.is_some() {
            painter.text(
                center + Vec2::new(0.0, if compact { 26.0 } else { 24.0 }),
                egui::Align2::CENTER_CENTER,
                if demo.records.persistent {
                    "Saved in this browser"
                } else {
                    "Saved for this session"
                },
                FontId::monospace(10.0),
                Color32::from_rgb(158, 180, 201),
            );
        }
    }
    publish_buttons(again, replay);
    if ui
        .put(
            again,
            egui::Button::new(if manual { "New attempt" } else { "Fly again" }),
        )
        .clicked()
    {
        return Action::Again;
    }
    if let Some(replay) = replay {
        if ui.put(replay, egui::Button::new("Replay best")).clicked() {
            return Action::Best;
        }
    }
    Action::None
}

#[cfg(target_arch = "wasm32")]
fn publish_buttons(again: Rect, replay: Option<Rect>) {
    let Some(canvas) = web_sys::window()
        .and_then(|w| w.document())
        .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
    else {
        return;
    };
    for (name, rect) in [("again", Some(again)), ("best", replay)] {
        let value = rect.map_or_else(String::new, |r| {
            format!("[{:.1},{:.1}]", r.center().x, r.center().y)
        });
        let _ = canvas.set_attribute(&format!("data-drone-result-{name}"), &value);
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn publish_buttons(_again: Rect, _replay: Option<Rect>) {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn result_actions_fit_portrait_landscape_and_desktop() {
        for size in [
            Vec2::new(320.0, 698.0),
            Vec2::new(667.0, 333.0),
            Vec2::new(844.0, 348.0),
            Vec2::new(1280.0, 858.0),
        ] {
            let scene = Rect::from_min_size(egui::pos2(0.0, 42.0), size);
            for manual in [true, false] {
                for best in [false, manual] {
                    let (card, again, replay) = layout(scene, manual, best);
                    assert!(scene.contains_rect(card));
                    assert!(card.contains_rect(again));
                    assert!(again.height() >= 40.0);
                    if let Some(replay) = replay {
                        assert!(card.contains_rect(replay));
                        assert!(!again.intersects(replay));
                    }
                }
            }
        }
    }
}
