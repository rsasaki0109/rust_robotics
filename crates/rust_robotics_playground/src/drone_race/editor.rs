//! Direct manipulation of a bounded course draft. Editing never plans or
//! advances a flight; applying starts a fresh race and opponent on the draft.

use egui::{Color32, FontId, Pos2, Rect, RichText, Stroke, Vec2};

use super::{
    flight::{Course, Flight},
    renderer::{self, CameraMode, ViewOptions, CYAN, GOLD},
};

#[derive(Clone, Copy, PartialEq, Eq, Default)]
enum MapView {
    #[default]
    Top,
    Side,
}

pub enum EditorAction {
    Editing,
    Apply(Course),
    Cancel,
}

pub struct CourseEditor {
    original: Course,
    preview: Flight,
    selected: usize,
    view: MapView,
    undo: Vec<Course>,
    grab_offset: Vec2,
}

impl CourseEditor {
    pub fn new(course: Course) -> Self {
        Self {
            original: course,
            preview: Flight::new(course),
            selected: 0,
            view: MapView::Top,
            undo: Vec::new(),
            grab_offset: Vec2::ZERO,
        }
    }

    pub fn course(&self) -> Course {
        self.preview.course
    }
    pub fn preview(&self) -> &Flight {
        &self.preview
    }

    fn remember(&mut self) {
        let course = self.course();
        if self.undo.last() != Some(&course) {
            if self.undo.len() == 32 {
                self.undo.remove(0);
            }
            self.undo.push(course);
        }
    }

    fn set_course(&mut self, course: Course) {
        if course != self.course() {
            self.preview = Flight::new(course);
        }
    }

    fn undo(&mut self) {
        if let Some(course) = self.undo.pop() {
            self.set_course(course);
        }
    }

    fn project(&self, map: Rect, point: [f64; 3]) -> Pos2 {
        let vertical = match self.view {
            MapView::Top => (point[1] + 4.0) / 8.0,
            MapView::Side => point[2] / 7.0,
        };
        Pos2::new(
            map.left() + map.width() * (point[0] / 26.0) as f32,
            map.bottom() - map.height() * vertical as f32,
        )
    }

    fn unproject(&self, map: Rect, pos: Pos2, mut point: [f64; 3]) -> [f64; 3] {
        point[0] = f64::from((pos.x - map.left()) / map.width()) * 26.0;
        let vertical = f64::from((map.bottom() - pos.y) / map.height());
        match self.view {
            MapView::Top => point[1] = vertical * 8.0 - 4.0,
            MapView::Side => point[2] = vertical * 7.0,
        }
        point
    }

    pub fn show(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) -> EditorAction {
        let mut action = EditorAction::Editing;
        let mut actions = Vec::new();
        let before = self.course();
        let selected_before = self.selected;
        ui.add_space(10.0);
        ui.horizontal(|ui| {
            ui.label(RichText::new("BUILD YOUR CIRCUIT").size(20.0).color(CYAN));
        });
        ui.label(
            RichText::new("Drag a gate in Top or Side view. The 3D preview follows your edits.")
                .size(11.0),
        );
        ui.horizontal_wrapped(|ui| {
            let fly = ui.button("Fly course");
            actions.push(("fly", fly.rect));
            if fly.clicked() {
                action = EditorAction::Apply(self.course());
            }
            let cancel = ui.button("Cancel");
            actions.push(("cancel", cancel.rect));
            if cancel.clicked() {
                action = EditorAction::Cancel;
            }
            let undo = ui.add_enabled(!self.undo.is_empty(), egui::Button::new("Undo"));
            actions.push(("undo", undo.rect));
            if undo.clicked() {
                self.undo();
            }
            let reset = ui.button("Reset");
            actions.push(("reset", reset.rect));
            if reset.clicked() && self.course() != self.original {
                self.remember();
                self.set_course(self.original);
            }
        });
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label(RichText::new("GATE").size(10.0).color(GOLD));
            for i in 0..5 {
                ui.selectable_value(&mut self.selected, i, format!("{:02}", i + 1));
            }
        });
        ui.horizontal(|ui| {
            let (low, high) = self.course().x_range(self.selected);
            let mut point = self.course().centers()[self.selected];
            for (axis, prefix, range) in [
                (0, "X ", low..=high),
                (1, "Y ", -3.0..=3.0),
                (2, "Z ", 1.2..=6.0),
            ] {
                let response = ui.add(
                    egui::DragValue::new(&mut point[axis])
                        .speed(0.1)
                        .range(range)
                        .prefix(prefix)
                        .suffix(" m")
                        .fixed_decimals(1),
                );
                if response.drag_started() {
                    self.remember();
                }
                if response.changed() {
                    if !response.dragged() {
                        self.remember();
                    }
                    self.set_course(self.course().moved(self.selected, point));
                }
            }
        });
        let width = ui.available_width().max(120.0);
        let preview_height = (ui.available_height() * 0.45).clamp(140.0, 300.0);
        let (preview_rect, _) =
            ui.allocate_exact_size(Vec2::new(width, preview_height), egui::Sense::hover());
        renderer::render(
            &ui.painter_at(preview_rect),
            preview_rect,
            self.preview(),
            0,
            self.preview().frames[0].state,
            0.0,
            &ViewOptions {
                camera: CameraMode::Course,
                trail: false,
                prediction: false,
                ghost: None,
                edit_gate: Some(self.selected),
            },
        );
        ui.horizontal(|ui| {
            let top = ui.selectable_value(&mut self.view, MapView::Top, "Top / position");
            actions.push(("top", top.rect));
            let side = ui.selectable_value(&mut self.view, MapView::Side, "Side / height");
            actions.push(("side", side.rect));
        });
        let height = (ui.available_height() - 22.0).clamp(125.0, 240.0);
        let (outer, _) = ui.allocate_exact_size(Vec2::new(width, height), egui::Sense::hover());
        let map = outer.shrink2(Vec2::new(24.0, 22.0));
        let painter = ui.painter_at(outer);
        painter.rect_filled(outer, 10.0, Color32::from_rgb(11, 22, 38));
        for x in (0..=24).step_by(2) {
            let screen_x = self.project(map, [x as f64, 0.0, 0.0]).x;
            painter.line_segment(
                [
                    egui::pos2(screen_x, map.top()),
                    egui::pos2(screen_x, map.bottom()),
                ],
                Stroke::new(0.6_f32, Color32::from_rgb(36, 54, 72)),
            );
            if x % 4 == 0 {
                painter.text(
                    egui::pos2(screen_x, map.bottom() + 8.0),
                    egui::Align2::CENTER_TOP,
                    format!("{x}"),
                    FontId::monospace(9.0),
                    Color32::GRAY,
                );
            }
        }
        let levels: Vec<f64> = if self.view == MapView::Top {
            vec![-3.0, 0.0, 3.0]
        } else {
            vec![0.0, 2.0, 4.0, 6.0]
        };
        for level in levels {
            let y = self
                .project(
                    map,
                    if self.view == MapView::Top {
                        [0.0, level, 0.0]
                    } else {
                        [0.0, 0.0, level]
                    },
                )
                .y;
            painter.line_segment(
                [egui::pos2(map.left(), y), egui::pos2(map.right(), y)],
                Stroke::new(0.6_f32, Color32::from_rgb(36, 54, 72)),
            );
            painter.text(
                egui::pos2(map.left() - 5.0, y),
                egui::Align2::RIGHT_CENTER,
                format!("{level:.0}"),
                FontId::monospace(9.0),
                Color32::GRAY,
            );
        }
        let mut previous = self.project(map, [0.0, 0.0, 2.0]);
        painter.circle_filled(previous, 4.0, CYAN);
        for point in self.course().centers() {
            let pos = self.project(map, point);
            painter.line_segment(
                [previous, pos],
                Stroke::new(1.2_f32, CYAN.gamma_multiply(0.4)),
            );
            previous = pos;
        }
        for i in 0..5 {
            let pos = self.project(map, self.course().centers()[i]);
            let response = ui.interact(
                Rect::from_center_size(pos, Vec2::splat(40.0)),
                ui.id().with(("course-gate", i)),
                egui::Sense::click_and_drag(),
            );
            if response.clicked() || response.drag_started() {
                self.selected = i;
            }
            if response.drag_started() {
                self.remember();
                self.grab_offset = ctx
                    .input(|input| input.pointer.press_origin())
                    .map_or(Vec2::ZERO, |pointer| pos - pointer);
            }
            if response.dragged() {
                if let Some(pointer) = response.interact_pointer_pos() {
                    let point =
                        self.unproject(map, pointer + self.grab_offset, self.course().centers()[i]);
                    self.set_course(self.course().moved(i, point));
                }
            }
            let pos = self.project(map, self.course().centers()[i]);
            let color = if i == self.selected { GOLD } else { CYAN };
            painter.circle_filled(pos, 14.0, Color32::from_rgb(18, 37, 57));
            painter.circle_stroke(
                pos,
                14.0,
                Stroke::new(if i == self.selected { 2.0_f32 } else { 1.0_f32 }, color),
            );
            painter.text(
                pos,
                egui::Align2::CENTER_CENTER,
                format!("{:02}", i + 1),
                FontId::monospace(11.0),
                color,
            );
        }
        ui.label(
            RichText::new("Distances in meters / 2 m minimum gate spacing / 0.1 m grid")
                .size(9.0)
                .color(Color32::GRAY),
        );
        if !ctx.wants_keyboard_input() && ctx.input(|input| input.key_pressed(egui::Key::Escape)) {
            action = EditorAction::Cancel;
        }
        if before != self.course() || selected_before != self.selected {
            ctx.request_repaint();
        }
        self.publish(map, &actions);
        if matches!(action, EditorAction::Apply(_)) {
            EditorAction::Apply(self.course())
        } else {
            action
        }
    }

    #[cfg(target_arch = "wasm32")]
    fn publish(&self, map: Rect, actions: &[(&str, Rect)]) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let handles = self
            .course()
            .centers()
            .map(|point| {
                let p = self.project(map, point);
                format!("[{:.1},{:.1}]", p.x, p.y)
            })
            .join(",");
        let buttons = actions
            .iter()
            .map(|(name, rect)| {
                format!("\"{name}\":[{:.1},{:.1}]", rect.center().x, rect.center().y)
            })
            .collect::<Vec<_>>()
            .join(",");
        for (key, value) in [
            ("data-drone-editor-handles", format!("[{handles}]")),
            ("data-drone-editor-actions", format!("{{{buttons}}}")),
            ("data-drone-editor-selected", self.selected.to_string()),
            (
                "data-drone-editor-view",
                if self.view == MapView::Top {
                    "top"
                } else {
                    "side"
                }
                .to_string(),
            ),
            (
                "data-drone-editor-map",
                format!(
                    "[{:.1},{:.1},{:.1},{:.1}]",
                    map.left(),
                    map.top(),
                    map.width(),
                    map.height()
                ),
            ),
        ] {
            let _ = canvas.set_attribute(key, &value);
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish(&self, _map: Rect, _actions: &[(&str, Rect)]) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn undo_restores_the_draft_and_preview_never_generates_flight() {
        let mut editor = CourseEditor::new(Course::CLIMB);
        editor.remember();
        editor.set_course(editor.course().moved(2, [12.0, -0.4, 4.8]));
        assert!(editor.course().is_custom());
        assert_eq!(editor.preview.last_index(), 0);
        editor.undo();
        assert_eq!(editor.course(), Course::CLIMB);
        assert_eq!(editor.preview.last_index(), 0);
        let map = Rect::from_min_size(Pos2::ZERO, Vec2::new(520.0, 160.0));
        for view in [MapView::Top, MapView::Side] {
            editor.view = view;
            let point = [12.0, -0.4, 4.8];
            let restored = editor.unproject(map, editor.project(map, point), point);
            for i in 0..3 {
                assert!((restored[i] - point[i]).abs() < 1e-6);
            }
        }
    }
}
