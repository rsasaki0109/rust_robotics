//! Shared look and layout helpers: one dark theme for the whole playground,
//! scenes that fill (and center in) the space they get, and the small
//! section headings used by every demo's control panel.

use egui::{Color32, Rect, Vec2};

/// Accent color of the playground (buttons, selections, links).
pub const ACCENT: Color32 = Color32::from_rgb(255, 152, 56);

/// Dark visuals matching the canvases and the landing page, slightly larger
/// text, and roomier controls (comfortable on touch screens too).
pub fn apply_style(ctx: &egui::Context) {
    let mut visuals = egui::Visuals::dark();
    visuals.panel_fill = Color32::from_rgb(16, 20, 26);
    visuals.window_fill = Color32::from_rgb(22, 27, 34);
    visuals.extreme_bg_color = Color32::from_rgb(10, 13, 18);
    visuals.faint_bg_color = Color32::from_rgb(24, 29, 37);
    visuals.selection.bg_fill = ACCENT.gamma_multiply(0.85);
    visuals.selection.stroke.color = Color32::from_rgb(20, 16, 10);
    visuals.hyperlink_color = ACCENT;
    visuals.widgets.inactive.corner_radius = egui::CornerRadius::same(6);
    visuals.widgets.hovered.corner_radius = egui::CornerRadius::same(6);
    visuals.widgets.active.corner_radius = egui::CornerRadius::same(6);
    visuals.widgets.noninteractive.corner_radius = egui::CornerRadius::same(6);
    // Dark whatever the system prefers: the canvases are dark.
    ctx.options_mut(|options| options.theme_preference = egui::ThemePreference::Dark);
    ctx.set_visuals_of(egui::Theme::Dark, visuals);

    ctx.style_mut_of(egui::Theme::Dark, |style| {
        use egui::{FontId, TextStyle};
        style
            .text_styles
            .insert(TextStyle::Body, FontId::proportional(14.5));
        style
            .text_styles
            .insert(TextStyle::Button, FontId::proportional(14.5));
        style
            .text_styles
            .insert(TextStyle::Small, FontId::proportional(12.5));
        style
            .text_styles
            .insert(TextStyle::Heading, FontId::proportional(20.0));
        style
            .text_styles
            .insert(TextStyle::Monospace, FontId::monospace(13.0));
        style.spacing.item_spacing = Vec2::new(8.0, 7.0);
        style.spacing.button_padding = Vec2::new(10.0, 5.0);
        style.spacing.interact_size.y = 26.0;
        style.spacing.slider_width = 130.0;
    });
}

/// A rectangle of the given `aspect` (height / width) that fills the space
/// left in `ui`, keeping `reserve` points free below it, centered
/// horizontally.
pub fn fit_rect(ui: &egui::Ui, aspect: f32, reserve: f32) -> Rect {
    let available = ui.available_size();
    let height_room = (available.y - reserve).max(160.0);
    let width = available.x.min(height_room / aspect).max(120.0);
    let left = ui.cursor().min.x + ((available.x - width) / 2.0).max(0.0);
    Rect::from_min_size(
        egui::pos2(left, ui.cursor().min.y),
        Vec2::new(width, width * aspect),
    )
}

/// Text drawn over the top-left corner of a scene, wrapped to fit it.
pub fn overlay_text(ui: &egui::Ui, rect: Rect, text: &str) {
    let galley = ui.painter().layout(
        text.to_owned(),
        egui::FontId::proportional(13.0),
        Color32::from_rgb(210, 215, 225),
        rect.width() - 24.0,
    );
    let pos = rect.left_top() + Vec2::new(12.0, 10.0);
    let painter = ui.painter_at(rect);
    painter.rect_filled(
        Rect::from_min_size(pos, galley.size()).expand(5.0),
        4.0,
        Color32::from_black_alpha(150),
    );
    painter.galley(pos, galley, Color32::WHITE);
}

/// A section heading in a control panel.
pub fn section(ui: &mut egui::Ui, title: &str) {
    ui.add_space(6.0);
    ui.label(
        egui::RichText::new(title)
            .small()
            .strong()
            .color(Color32::from_rgb(150, 160, 175)),
    );
}

/// Secondary explanatory text.
pub fn hint(ui: &mut egui::Ui, text: &str) {
    ui.label(egui::RichText::new(text).weak());
}

/// A collapsed "How it works" block for longer explanations.
pub fn how_it_works(ui: &mut egui::Ui, id: &str, text: &str) {
    ui.add_space(4.0);
    egui::CollapsingHeader::new("How it works")
        .id_salt(id)
        .default_open(false)
        .show(ui, |ui| hint(ui, text));
}

/// A colored dot followed by a label, for legends.
pub fn legend(ui: &mut egui::Ui, entries: &[(Color32, &str)]) {
    ui.horizontal_wrapped(|ui| {
        for (color, label) in entries {
            // One widget per entry, so rows wrap between entries and never
            // split a dot from its label.
            let galley = ui.painter().layout_no_wrap(
                (*label).to_owned(),
                egui::TextStyle::Small.resolve(ui.style()),
                ui.visuals().text_color(),
            );
            let size = Vec2::new(14.0 + galley.size().x, galley.size().y.max(12.0));
            let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
            ui.painter()
                .circle_filled(egui::pos2(rect.left() + 5.0, rect.center().y), 4.5, *color);
            ui.painter().galley(
                egui::pos2(rect.left() + 14.0, rect.center().y - galley.size().y / 2.0),
                galley,
                ui.visuals().text_color(),
            );
        }
    });
}

/// Play/pause button, frame counter, and a timeline slider that takes the
/// rest of the row. Pressing play at the end starts over.
pub fn playback(ui: &mut egui::Ui, playing: &mut bool, frame: &mut usize, last: usize) {
    ui.horizontal(|ui| {
        let label = if *playing { "⏸ Pause" } else { "▶ Play" };
        if ui
            .add_sized([86.0, 26.0], egui::Button::new(label))
            .clicked()
        {
            if !*playing && *frame >= last {
                *frame = 0;
            }
            *playing = !*playing;
        }
        let counter = format!("{frame}/{last}");
        let counter_width = 64.0;
        ui.spacing_mut().slider_width = (ui.available_width() - counter_width - 16.0).max(60.0);
        let response = ui.add(egui::Slider::new(frame, 0..=last.max(1)).show_value(false));
        if response.dragged() {
            *playing = false;
        }
        ui.label(egui::RichText::new(counter).monospace().weak());
    });
}

/// Whether `period` seconds have passed since `*last` (then resets it), so
/// replays advance at a fixed rate whatever the frame rate. Schedules the
/// next repaint.
pub fn every(ctx: &egui::Context, last: &mut f64, period: f64) -> bool {
    let now = ctx.input(|i| i.time);
    ctx.request_repaint_after(std::time::Duration::from_secs_f64(period));
    if now - *last >= period || now < *last {
        *last = now;
        true
    } else {
        false
    }
}
