//! A paused view of real flight history. Export crops eframe's rendered pixels.

use egui::{Color32, FontId, Rect, RichText, Vec2};

use super::flight::{Flight, DT};
use super::renderer::{self, CameraMode, GhostView, Orbit, ViewOptions, CYAN};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum Format {
    #[default]
    Landscape,
    Square,
    Portrait,
}

impl Format {
    const ALL: [Self; 3] = [Self::Landscape, Self::Square, Self::Portrait];

    fn slug(self) -> &'static str {
        match self {
            Self::Landscape => "wide",
            Self::Square => "square",
            Self::Portrait => "portrait",
        }
    }

    fn title(self) -> &'static str {
        match self {
            Self::Landscape => "16:9",
            Self::Square => "1:1",
            Self::Portrait => "4:5",
        }
    }

    fn ratio(self) -> f32 {
        match self {
            Self::Landscape => 16.0 / 9.0,
            Self::Square => 1.0,
            Self::Portrait => 4.0 / 5.0,
        }
    }

    fn fit(self, available: Rect) -> Rect {
        let width = available.width().min(available.height() * self.ratio());
        Rect::from_center_size(available.center(), Vec2::new(width, width / self.ratio()))
    }
}

#[derive(Clone, Debug)]
struct Capture {
    rect: Rect,
    pixels_per_point: f32,
    filename: String,
}

impl Capture {
    fn crop(&self, image: &egui::ColorImage) -> Option<egui::ColorImage> {
        // A viewport can resize between request and delivery. Never index past
        // its actual image or silently save a truncated composition.
        let rect = self.rect * self.pixels_per_point;
        if !rect.is_finite()
            || rect.min.x < 0.0
            || rect.min.y < 0.0
            || rect.max.x > image.width() as f32
            || rect.max.y > image.height() as f32
            || rect.width() < 1.0
            || rect.height() < 1.0
        {
            return None;
        }
        Some(image.region(&rect, None))
    }
}

pub struct PhotoMode {
    pub frame: usize,
    pub orbit: Orbit,
    pub trail: bool,
    pub ghost: bool,
    format: Format,
    caption: bool,
    saving: bool,
    message: String,
}

impl PhotoMode {
    pub fn new(frame: usize, trail: bool, ghost: bool) -> Self {
        Self {
            frame,
            orbit: Orbit::default(),
            trail,
            ghost,
            format: Format::default(),
            caption: true,
            saving: false,
            message: String::new(),
        }
    }

    pub fn apply_query(&mut self, query: &str) {
        let bounded = crate::share::bounded_f32;
        self.orbit.yaw = bounded(query, "yaw", 0.0, 360.0).unwrap_or(self.orbit.yaw);
        self.orbit.pitch = bounded(query, "pitch", 10.0, 80.0).unwrap_or(self.orbit.pitch);
        self.orbit.distance = bounded(query, "distance", 2.0, 25.0).unwrap_or(self.orbit.distance);
        self.format = Format::ALL
            .into_iter()
            .find(|f| crate::share::value(query, "format") == Some(f.slug()))
            .unwrap_or_default();
        self.caption = crate::share::boolean(query, "caption").unwrap_or(true);
    }

    pub fn query(&self) -> String {
        format!(
            "&photo=1&yaw={:.3}&pitch={:.3}&distance={:.3}&format={}&caption={}",
            self.orbit.yaw,
            self.orbit.pitch,
            self.orbit.distance,
            self.format.slug(),
            u8::from(self.caption)
        )
    }

    fn receive_capture(&mut self, ctx: &egui::Context) {
        let capture = ctx.input(|input| {
            input.events.iter().find_map(|event| {
                if let egui::Event::Screenshot {
                    user_data, image, ..
                } = event
                {
                    let request = user_data.data.as_ref()?.downcast_ref::<Capture>()?;
                    Some((request.clone(), image.clone()))
                } else {
                    None
                }
            })
        });
        let Some((request, image)) = capture else {
            return;
        };
        self.saving = false;
        self.message = match request.crop(&image) {
            Some(image) => match download(&image, &request.filename) {
                Ok(()) => format!(
                    "PNG sent to downloads / {} × {}",
                    image.width(),
                    image.height()
                ),
                Err(()) => "Could not save. Try Save PNG again.".to_string(),
            },
            None => "The window changed size. Try Save PNG again.".to_string(),
        };
    }

    /// Returns true on Close. The caller's cursor and flight cache are untouched.
    pub fn show(
        &mut self,
        ctx: &egui::Context,
        ui: &mut egui::Ui,
        flight: &Flight,
        ghost: Option<&Flight>,
    ) -> bool {
        self.receive_capture(ctx);
        self.frame = self.frame.min(flight.last_index());
        let mut actions = Vec::new();
        let mut close = false;
        let mut save = false;
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new("PHOTO MODE").color(CYAN).size(18.0));
            let button = ui.button("Back to flight");
            close = button.clicked();
            actions.push(("close", button.rect));
            let button = ui.add_enabled(
                cfg!(target_arch = "wasm32") && !self.saving,
                egui::Button::new(if self.saving { "Saving…" } else { "Save PNG" }),
            );
            save = button.clicked();
            actions.push(("save", button.rect));
        });
        ui.horizontal_wrapped(|ui| {
            for format in Format::ALL {
                let button = ui.selectable_value(&mut self.format, format, format.title());
                actions.push((format.slug(), button.rect));
            }
            let trail = ui.checkbox(&mut self.trail, "Trail");
            actions.push(("trail", trail.rect));
            if ghost.is_some() {
                let button = ui.checkbox(&mut self.ghost, "Ghost");
                actions.push(("ghost", button.rect));
            }
            let caption = ui.checkbox(&mut self.caption, "Caption");
            actions.push(("caption", caption.rect));
        });
        ui.horizontal_wrapped(|ui| {
            for (name, label, orbit) in [
                ("hero", "Hero", Orbit::default()),
                (
                    "side",
                    "Side",
                    Orbit {
                        yaw: 270.0,
                        pitch: 15.0,
                        distance: 4.0,
                    },
                ),
                (
                    "above",
                    "Above",
                    Orbit {
                        yaw: 235.0,
                        pitch: 75.0,
                        distance: 8.0,
                    },
                ),
            ] {
                let button = ui.button(label);
                if button.clicked() {
                    self.orbit = orbit;
                }
                actions.push((name, button.rect));
            }
            for (name, label, factor) in [("in", "+", 0.8), ("out", "−", 1.25)] {
                let button = ui.add_sized([30.0, 22.0], egui::Button::new(label));
                if button.clicked() {
                    self.orbit.distance = (self.orbit.distance * factor).clamp(2.0, 25.0);
                }
                actions.push((name, button.rect));
            }
            ui.weak("Drag to orbit · scroll or + / − to zoom");
        });
        ui.horizontal(|ui| {
            ui.label("Moment");
            let slider = ui
                .add(egui::Slider::new(&mut self.frame, 0..=flight.last_index()).show_value(false));
            actions.push(("timeline", slider.rect));
            ui.monospace(format!("{:.2} s", self.frame as f64 * DT));
        });
        if !self.message.is_empty() {
            ui.label(RichText::new(&self.message).small().color(CYAN));
        }
        #[cfg(not(target_arch = "wasm32"))]
        ui.weak("Open the browser playground to download a PNG.");
        let available = ui.available_size();
        let (space, _) = ui.allocate_exact_size(
            Vec2::new(available.x, available.y.max(120.0)),
            egui::Sense::hover(),
        );
        let rect = self.format.fit(space.shrink(6.0));
        let response = ui.interact(rect, ui.id().with("photo-orbit"), egui::Sense::drag());
        if response.dragged() {
            let delta = response.drag_delta();
            self.orbit.yaw = (self.orbit.yaw - delta.x * 0.4).rem_euclid(360.0);
            self.orbit.pitch = (self.orbit.pitch + delta.y * 0.3).clamp(10.0, 80.0);
        }
        if response.hovered() {
            let scroll = ctx.input(|input| input.smooth_scroll_delta.y);
            self.orbit.distance = (self.orbit.distance * (-scroll * 0.003).exp()).clamp(2.0, 25.0);
        }
        let current = &flight.frames[self.frame];
        let painter = ui.painter_at(rect);
        renderer::render(
            &painter,
            rect,
            flight,
            self.frame,
            current.state,
            self.frame as f64 * DT,
            &ViewOptions {
                camera: CameraMode::Course,
                orbit: Some(self.orbit),
                trail: self.trail,
                prediction: false,
                edit_gate: None,
                ghost: ghost.filter(|_| self.ghost).map(|g| {
                    let frame = self.frame.min(g.last_index());
                    GhostView {
                        flight: g,
                        frame,
                        state: g.frames[frame].state,
                    }
                }),
            },
        );
        if self.caption {
            self.paint_caption(&painter, rect, flight);
        }
        ui.painter().rect_stroke(
            rect,
            0.0,
            egui::Stroke::new(1.0_f32, CYAN.gamma_multiply(0.3)),
            egui::StrokeKind::Outside,
        );
        if save {
            self.saving = true;
            self.message.clear();
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::new(
                Capture {
                    rect,
                    pixels_per_point: ctx.pixels_per_point(),
                    filename: format!(
                        "rust-flight-{}-{}-{}.png",
                        flight.course.slug(),
                        self.frame,
                        self.format.slug()
                    ),
                },
            )));
            ctx.request_repaint();
        }
        publish(rect, &actions, self);
        close
            || (!ctx.wants_keyboard_input()
                && ctx.input(|input| input.key_pressed(egui::Key::Escape)))
    }

    fn paint_caption(&self, painter: &egui::Painter, rect: Rect, flight: &Flight) {
        let small = rect.width() < 500.0;
        let pad = if small { 14.0 } else { 24.0 };
        let caption = Rect::from_min_max(
            egui::pos2(rect.left(), rect.bottom() - 76.0),
            rect.right_bottom(),
        );
        painter.rect_filled(caption, 0.0, Color32::from_black_alpha(170));
        painter.text(
            caption.left_top() + Vec2::new(pad, 12.0),
            egui::Align2::LEFT_TOP,
            "RUST / FLIGHT LAB",
            FontId::monospace(if small { 11.0 } else { 14.0 }),
            CYAN,
        );
        painter.text(
            caption.left_top() + Vec2::new(pad, 37.0),
            egui::Align2::LEFT_TOP,
            format!(
                "{} / {}",
                flight.course.title(),
                if flight.is_manual() {
                    "You vs autopilot"
                } else {
                    "Autonomous flight"
                }
            ),
            FontId::proportional(if small { 12.0 } else { 16.0 }),
            Color32::WHITE,
        );
        painter.text(
            rect.right_top() + Vec2::new(-pad, pad),
            egui::Align2::RIGHT_TOP,
            format!(
                "{:02}/05  /  {:.2} s",
                flight.frames[self.frame].passed,
                self.frame as f64 * DT
            ),
            FontId::monospace(if small { 10.0 } else { 13.0 }),
            CYAN,
        );
    }
}

#[cfg(target_arch = "wasm32")]
fn publish(rect: Rect, actions: &[(&str, Rect)], photo: &PhotoMode) {
    let Some(canvas) = web_sys::window()
        .and_then(|w| w.document())
        .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
    else {
        return;
    };
    let fields = actions
        .iter()
        .map(|(name, r)| format!("\"{name}\":[{:.1},{:.1}]", r.center().x, r.center().y))
        .collect::<Vec<_>>()
        .join(",");
    for (key, value) in [
        ("data-drone-photo-actions", format!("{{{fields}}}")),
        (
            "data-drone-photo-rect",
            format!(
                "[{:.2},{:.2},{:.2},{:.2}]",
                rect.left(),
                rect.top(),
                rect.width(),
                rect.height()
            ),
        ),
        (
            "data-drone-photo-orbit",
            format!(
                "{:.3},{:.3},{:.3}",
                photo.orbit.yaw, photo.orbit.pitch, photo.orbit.distance
            ),
        ),
        ("data-drone-photo-format", photo.format.slug().to_string()),
        (
            "data-drone-photo-saving",
            u8::from(photo.saving).to_string(),
        ),
    ] {
        let _ = canvas.set_attribute(key, &value);
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn publish(_rect: Rect, _actions: &[(&str, Rect)], _photo: &PhotoMode) {}

#[cfg(target_arch = "wasm32")]
fn download(image: &egui::ColorImage, filename: &str) -> Result<(), ()> {
    use wasm_bindgen::prelude::*;
    #[wasm_bindgen(
        inline_js = "export function downloadFlightPhoto(bytes, width, height, filename) {\n const canvas = document.createElement('canvas'); canvas.width = width; canvas.height = height;\n const context = canvas.getContext('2d'); if (!context) throw new Error('No image encoder');\n context.putImageData(new ImageData(new Uint8ClampedArray(bytes), width, height), 0, 0);\n const link = document.createElement('a'); link.href = canvas.toDataURL('image/png'); link.download = filename;\n document.body.append(link); link.click(); link.remove();\n}"
    )]
    extern "C" {
        #[wasm_bindgen(catch, js_name = downloadFlightPhoto)]
        fn download_photo(
            bytes: &[u8],
            width: u32,
            height: u32,
            filename: &str,
        ) -> Result<(), JsValue>;
    }
    let bytes = image
        .pixels
        .iter()
        .flat_map(Color32::to_array)
        .collect::<Vec<_>>();
    download_photo(
        &bytes,
        image.width() as u32,
        image.height() as u32,
        filename,
    )
    .map_err(|_| ())
}

#[cfg(not(target_arch = "wasm32"))]
fn download(_image: &egui::ColorImage, _filename: &str) -> Result<(), ()> {
    Err(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capture_crops_physical_pixels_and_rejects_a_resized_viewport() {
        let mut image = egui::ColorImage::new([12, 8], Color32::BLACK);
        image[(4, 2)] = CYAN;
        let capture = Capture {
            rect: Rect::from_min_size(egui::pos2(2.0, 1.0), Vec2::new(3.0, 2.0)),
            pixels_per_point: 2.0,
            filename: String::new(),
        };
        let cropped = capture.crop(&image).unwrap();
        assert_eq!(cropped.size, [6, 4]);
        assert_eq!(cropped[(0, 0)], CYAN);
        assert!(capture
            .crop(&egui::ColorImage::new([5, 5], Color32::BLACK))
            .is_none());
    }

    #[test]
    fn camera_query_ignores_nonfinite_and_out_of_range_values() {
        let mut photo = PhotoMode::new(0, true, true);
        photo.apply_query("yaw=NaN&pitch=90&distance=-2&format=unknown&caption=0");
        assert_eq!(photo.orbit, Orbit::default());
        assert_eq!(photo.format, Format::Landscape);
        assert!(!photo.caption);
    }
}
