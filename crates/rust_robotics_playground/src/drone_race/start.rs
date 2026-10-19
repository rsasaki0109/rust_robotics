//! A foreground countdown. Its clock is separate from recorded flight time.

use egui::{Color32, FontId, Rect, Stroke, Vec2};

use super::renderer::{CYAN, GOLD};

pub struct StartSequence {
    pub(super) age: f64,
    #[cfg(target_arch = "wasm32")]
    focus: Option<FocusGuard>,
}

impl Default for StartSequence {
    fn default() -> Self {
        Self {
            age: 0.0,
            #[cfg(target_arch = "wasm32")]
            focus: FocusGuard::install(),
        }
    }
}

impl StartSequence {
    pub fn advance(&mut self, elapsed: f64) {
        let waiting = self.waiting();
        self.age += elapsed;
        if waiting && !self.waiting() {
            self.age = 3.0;
        }
    }

    pub fn waiting(&self) -> bool {
        self.age < 3.0
    }

    pub fn finished(&self) -> bool {
        self.age >= 3.7
    }

    pub fn interrupted(&self) -> bool {
        #[cfg(target_arch = "wasm32")]
        if let Some(focus) = &self.focus {
            return focus.interrupted.get()
                || focus.document.hidden()
                || !focus.document.has_focus().unwrap_or(true);
        }
        false
    }

    pub fn label(&self) -> &'static str {
        if self.age < 1.0 {
            "3"
        } else if self.age < 2.0 {
            "2"
        } else if self.waiting() {
            "1"
        } else {
            "go"
        }
    }

    pub fn paint(&self, painter: &egui::Painter, scene: Rect) {
        let center = scene.center() - Vec2::new(0.0, 10.0);
        let waiting = self.waiting();
        let color = if waiting { GOLD } else { CYAN };
        let fade = if waiting {
            1.0
        } else {
            (1.0 - (self.age - 3.0) / 0.7).clamp(0.0, 1.0) as f32
        };
        let pulse = if waiting {
            (1.0 - self.age.fract()) as f32
        } else {
            0.0
        };
        let radius = 62.0 + pulse * 8.0;
        painter.circle_filled(
            center,
            radius,
            Color32::from_black_alpha((210.0 * fade) as u8),
        );
        for (width, opacity) in [(12.0_f32, 0.035), (5.0, 0.12), (1.5, 0.8)] {
            painter.circle_stroke(
                center,
                radius,
                Stroke::new(width, color.gamma_multiply(opacity * fade)),
            );
        }
        for index in 0..3 {
            let lit = !waiting || self.age >= f64::from(index);
            let point = center + Vec2::new((index as f32 - 1.0) * 28.0, -90.0);
            painter.circle_filled(
                point,
                6.0,
                if lit {
                    color.gamma_multiply(fade)
                } else {
                    Color32::from_rgb(45, 50, 61)
                },
            );
        }
        painter.text(
            center,
            egui::Align2::CENTER_CENTER,
            if waiting { self.label() } else { "GO" },
            FontId::proportional(if waiting { 78.0 } else { 56.0 }),
            color.gamma_multiply(fade),
        );
        if waiting {
            painter.rect_filled(
                Rect::from_center_size(center + Vec2::new(0.0, 98.0), Vec2::new(270.0, 46.0)),
                6.0,
                Color32::from_black_alpha(200),
            );
            painter.text(
                center + Vec2::new(0.0, 88.0),
                egui::Align2::CENTER_CENTER,
                "HOLD YOUR FLIGHT CONTROLS",
                FontId::monospace(10.0),
                Color32::WHITE,
            );
            painter.text(
                center + Vec2::new(0.0, 108.0),
                egui::Align2::CENTER_CENTER,
                "Space / Pause to cancel",
                FontId::monospace(10.0),
                Color32::from_rgb(158, 180, 201),
            );
        }
    }
}

// Remember a loss of browser focus even when a hidden tab suspends painting
// and egui next sees an already-focused canvas. Remove both listeners when
// the sequence completes, is cancelled, or the demo is dropped.
#[cfg(target_arch = "wasm32")]
struct FocusGuard {
    window: web_sys::Window,
    document: web_sys::Document,
    interrupted: std::rc::Rc<std::cell::Cell<bool>>,
    listener: wasm_bindgen::closure::Closure<dyn FnMut(web_sys::Event)>,
}

#[cfg(target_arch = "wasm32")]
impl FocusGuard {
    fn install() -> Option<Self> {
        use wasm_bindgen::JsCast;
        let window = web_sys::window()?;
        let document = window.document()?;
        let interrupted = std::rc::Rc::new(std::cell::Cell::new(false));
        let signal = interrupted.clone();
        let observed = document.clone();
        let listener =
            wasm_bindgen::closure::Closure::wrap(Box::new(move |event: web_sys::Event| {
                if event.type_() == "blur" || observed.hidden() {
                    signal.set(true);
                }
            }) as Box<dyn FnMut(web_sys::Event)>);
        let callback = listener.as_ref().unchecked_ref();
        let _ = window.add_event_listener_with_callback("blur", callback);
        let _ = document.add_event_listener_with_callback("visibilitychange", callback);
        Some(Self {
            window,
            document,
            interrupted,
            listener,
        })
    }
}

#[cfg(target_arch = "wasm32")]
impl Drop for FocusGuard {
    fn drop(&mut self) {
        use wasm_bindgen::JsCast;
        let callback = self.listener.as_ref().unchecked_ref();
        let _ = self
            .window
            .remove_event_listener_with_callback("blur", callback);
        let _ = self
            .document
            .remove_event_listener_with_callback("visibilitychange", callback);
    }
}
