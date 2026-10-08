//! Interactive planar pushing: drag the goal pose of a square slider and watch
//! a face-switching MPPI controller push it there under quasi-static
//! stick/slide contact (`rust_robotics_control::pusher_slider`).

use egui::{Color32, Pos2, Rect, Shape, Stroke, Vec2};
use rust_robotics_control::pusher_slider::{
    ContactMode, PusherCommand, PusherMppiConfig, PusherSliderMppiController, PusherSliderParams,
    SliderState,
};

/// Visible table area \[m\].
const TABLE_X: (f64, f64) = (-0.12, 0.48);
const TABLE_Y: (f64, f64) = (-0.2, 0.2);
/// Radius of a user-placed obstacle disc \[m\].
const OBSTACLE_RADIUS: f64 = 0.025;
const MAX_OBSTACLES: usize = 12;
/// Goal tolerances, matching `simulate_push`.
const POSITION_TOLERANCE_FACTOR: f64 = 0.2;
const HEADING_TOLERANCE: f64 = 0.05;
/// Real-time repaint interval; one 0.1 s control step per frame runs ~2× real time.
const FRAME_MS: u64 = 50;

const TABLE: Color32 = Color32::from_rgb(24, 28, 34);
const SLIDER: Color32 = Color32::from_rgb(110, 165, 240);
const GOAL: Color32 = Color32::from_rgb(90, 210, 140);
const OBSTACLE: Color32 = Color32::from_rgb(200, 90, 90);
const TRAIL: Color32 = Color32::from_rgba_premultiplied(70, 90, 120, 160);
const STICK: Color32 = Color32::from_rgb(250, 220, 90);
const SLIDE: Color32 = Color32::from_rgb(240, 120, 60);

fn wrap_angle(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// What a drag on the table edits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DragTool {
    /// Drag sets the goal position.
    MoveGoal,
    /// Drag points the goal heading toward the pointer.
    RotateGoal,
    /// Click adds an obstacle (click an obstacle to remove it).
    Obstacles,
}

pub struct PushingDemo {
    params: PusherSliderParams,
    controller: PusherSliderMppiController,
    slider: SliderState,
    goal: SliderState,
    obstacles: Vec<[f64; 2]>,
    trail: Vec<[f64; 2]>,
    last_command: Option<PusherCommand>,
    last_mode: ContactMode,
    steps: usize,
    stick_steps: usize,
    slide_steps: usize,
    face_switches: usize,
    pub(crate) running: bool,
    pub(crate) friction: f32,
    tool: DragTool,
}

impl Default for PushingDemo {
    fn default() -> Self {
        let params = PusherSliderParams::default();
        let mut demo = Self {
            params,
            controller: Self::controller(params, &[]),
            slider: SliderState::new(0.0, 0.0, 0.0),
            goal: SliderState::new(0.3, 0.08, std::f64::consts::FRAC_PI_2),
            obstacles: Vec::new(),
            trail: Vec::new(),
            last_command: None,
            last_mode: ContactMode::Separated,
            steps: 0,
            stick_steps: 0,
            slide_steps: 0,
            face_switches: 0,
            running: true,
            friction: params.pusher_friction as f32,
            tool: DragTool::MoveGoal,
        };
        demo.trail.push([demo.slider.x(), demo.slider.y()]);
        demo
    }
}

impl PushingDemo {
    fn controller(
        params: PusherSliderParams,
        obstacles: &[[f64; 2]],
    ) -> PusherSliderMppiController {
        let config = PusherMppiConfig {
            // Strong enough that the slider never overlaps an obstacle on the
            // straight-line path (weaker weights let it shove straight through).
            obstacle_weight: 100_000.0,
            obstacle_radius: OBSTACLE_RADIUS + 1.2 * params.half_extent,
            ..PusherMppiConfig::default()
        };
        PusherSliderMppiController::new(config, params)
            .expect("default pusher MPPI config is valid")
            .with_obstacles(obstacles.to_vec())
    }

    /// Rebuilds the controller after the friction or obstacles changed.
    fn rebuild_controller(&mut self) {
        self.params.pusher_friction = f64::from(self.friction);
        self.controller = Self::controller(self.params, &self.obstacles);
    }

    fn reset_slider(&mut self) {
        self.slider = SliderState::new(0.0, 0.0, 0.0);
        self.trail = vec![[0.0, 0.0]];
        self.last_command = None;
        self.last_mode = ContactMode::Separated;
        self.steps = 0;
        self.stick_steps = 0;
        self.slide_steps = 0;
        self.face_switches = 0;
        self.rebuild_controller();
    }

    fn set_goal(&mut self, x: f64, y: f64, theta: f64) {
        self.goal = SliderState::new(
            x.clamp(TABLE_X.0, TABLE_X.1),
            y.clamp(TABLE_Y.0, TABLE_Y.1),
            wrap_angle(theta),
        );
    }

    fn errors(&self) -> (f64, f64) {
        let position = (self.slider.x() - self.goal.x()).hypot(self.slider.y() - self.goal.y());
        let heading = wrap_angle(self.slider.theta() - self.goal.theta()).abs();
        (position, heading)
    }

    fn goal_reached(&self) -> bool {
        let (position, heading) = self.errors();
        position < POSITION_TOLERANCE_FACTOR * self.params.half_extent
            && heading < HEADING_TOLERANCE
    }

    /// Plans and executes one pusher command; returns whether it pushed.
    fn step(&mut self) -> bool {
        if self.goal_reached() {
            self.last_command = None;
            self.last_mode = ContactMode::Separated;
            return false;
        }
        let Ok(plan) = self.controller.plan(self.slider, self.goal) else {
            return false;
        };
        let dt = self.controller.config().dt;
        let (next, mode) = self.params.step(self.slider, plan.command, dt);
        if self
            .last_command
            .is_some_and(|last| last.face != plan.command.face)
        {
            self.face_switches += 1;
        }
        match mode {
            ContactMode::Stick => self.stick_steps += 1,
            ContactMode::SlideUp | ContactMode::SlideDown => self.slide_steps += 1,
            ContactMode::Separated => {}
        }
        self.slider = next;
        self.last_command = Some(plan.command);
        self.last_mode = mode;
        self.steps += 1;
        self.trail.push([next.x(), next.y()]);
        true
    }

    pub fn apply_share_query(&mut self, query: &str) {
        let number =
            |key: &str, min: f32, max: f32| crate::share::bounded_f32(query, key, min, max);
        if let (Some(x), Some(y)) = (
            number("goal_x", TABLE_X.0 as f32, TABLE_X.1 as f32),
            number("goal_y", TABLE_Y.0 as f32, TABLE_Y.1 as f32),
        ) {
            let theta = number("goal_deg", -180.0, 180.0).unwrap_or(0.0);
            self.set_goal(f64::from(x), f64::from(y), f64::from(theta).to_radians());
        }
        if let Some(friction) = number("mu", 0.05, 1.0) {
            self.friction = friction;
        }
        if let Some(value) = crate::share::value(query, "obstacles") {
            self.obstacles = value
                .split(';')
                .filter_map(|pair| {
                    let (x, y) = pair.split_once(',')?;
                    let (x, y) = (x.parse::<f64>().ok()?, y.parse::<f64>().ok()?);
                    (x.is_finite() && y.is_finite())
                        .then_some([x.clamp(TABLE_X.0, TABLE_X.1), y.clamp(TABLE_Y.0, TABLE_Y.1)])
                })
                .take(MAX_OBSTACLES)
                .collect();
        }
        if let Some(running) = crate::share::boolean(query, "running") {
            self.running = running;
        }
        self.reset_slider();
    }

    pub fn share_query(&self) -> String {
        let mut query = format!(
            "tab=pushing&goal_x={:.3}&goal_y={:.3}&goal_deg={:.0}&mu={:.2}&running={}",
            self.goal.x(),
            self.goal.y(),
            self.goal.theta().to_degrees(),
            self.friction,
            u8::from(self.running),
        );
        if !self.obstacles.is_empty() {
            let obstacles: Vec<String> = self
                .obstacles
                .iter()
                .map(|[x, y]| format!("{x:.3},{y:.3}"))
                .collect();
            query.push_str("&obstacles=");
            query.push_str(&obstacles.join(";"));
        }
        query
    }

    fn table_rect(ui: &egui::Ui) -> Rect {
        let aspect = ((TABLE_Y.1 - TABLE_Y.0) / (TABLE_X.1 - TABLE_X.0)) as f32;
        let width = ui
            .available_width()
            .min((ui.available_height() - 64.0).max(120.0) / aspect);
        Rect::from_min_size(ui.cursor().min, Vec2::new(width, width * aspect))
    }

    fn to_screen(rect: Rect, x: f64, y: f64) -> Pos2 {
        let u = ((x - TABLE_X.0) / (TABLE_X.1 - TABLE_X.0)) as f32;
        let v = 1.0 - ((y - TABLE_Y.0) / (TABLE_Y.1 - TABLE_Y.0)) as f32;
        rect.min + Vec2::new(u * rect.width(), v * rect.height())
    }

    fn to_table(rect: Rect, pos: Pos2) -> [f64; 2] {
        let u = f64::from((pos.x - rect.min.x) / rect.width());
        let v = f64::from((pos.y - rect.min.y) / rect.height());
        [
            TABLE_X.0 + u * (TABLE_X.1 - TABLE_X.0),
            TABLE_Y.1 - v * (TABLE_Y.1 - TABLE_Y.0),
        ]
    }

    fn square(&self, rect: Rect, state: SliderState) -> Vec<Pos2> {
        let b = self.params.half_extent;
        let (sin, cos) = state.theta().sin_cos();
        [(-b, -b), (b, -b), (b, b), (-b, b)]
            .into_iter()
            .map(|(px, py)| {
                Self::to_screen(
                    rect,
                    state.x() + cos * px - sin * py,
                    state.y() + sin * px + cos * py,
                )
            })
            .collect()
    }

    fn draw(&self, painter: &egui::Painter, rect: Rect) {
        painter.rect_filled(rect, 0.0, TABLE);
        let scale = rect.width() / (TABLE_X.1 - TABLE_X.0) as f32;

        for [x, y] in &self.obstacles {
            painter.circle_filled(
                Self::to_screen(rect, *x, *y),
                OBSTACLE_RADIUS as f32 * scale,
                OBSTACLE,
            );
        }
        let trail: Vec<Pos2> = self
            .trail
            .iter()
            .map(|[x, y]| Self::to_screen(rect, *x, *y))
            .collect();
        painter.add(Shape::line(trail, Stroke::new(1.5_f32, TRAIL)));

        // Goal outline with a heading tick.
        let goal = self.square(rect, self.goal);
        painter.add(Shape::closed_line(goal, Stroke::new(2.0_f32, GOAL)));
        let goal_center = Self::to_screen(rect, self.goal.x(), self.goal.y());
        let reach = self.params.half_extent * 1.4;
        let goal_tip = Self::to_screen(
            rect,
            self.goal.x() + reach * self.goal.theta().cos(),
            self.goal.y() + reach * self.goal.theta().sin(),
        );
        painter.line_segment([goal_center, goal_tip], Stroke::new(2.0_f32, GOAL));

        // Slider with a heading tick.
        let slider = self.square(rect, self.slider);
        painter.add(Shape::convex_polygon(
            slider,
            SLIDER.gamma_multiply(0.85),
            Stroke::new(1.5_f32, SLIDER),
        ));
        let center = Self::to_screen(rect, self.slider.x(), self.slider.y());
        let tip = Self::to_screen(
            rect,
            self.slider.x() + reach * self.slider.theta().cos(),
            self.slider.y() + reach * self.slider.theta().sin(),
        );
        painter.line_segment([center, tip], Stroke::new(2.0_f32, Color32::WHITE));

        // Pusher: contact point plus push direction, colored by contact mode.
        if let Some(command) = self.last_command {
            let color = match self.last_mode {
                ContactMode::Stick => STICK,
                ContactMode::SlideUp | ContactMode::SlideDown => SLIDE,
                ContactMode::Separated => Color32::GRAY,
            };
            let [cx, cy] = self.params.contact_point(self.slider, command);
            let contact = Self::to_screen(rect, cx, cy);
            let toward = Vec2::new(center.x - contact.x, center.y - contact.y).normalized();
            painter.line_segment(
                [contact - toward * 22.0, contact],
                Stroke::new(3.0_f32, color),
            );
            painter.circle_filled(contact, 5.0, color);
        }
    }

    fn handle_input(&mut self, response: &egui::Response) {
        let Some(pos) = response.interact_pointer_pos() else {
            return;
        };
        let [x, y] = Self::to_table(response.rect, pos);
        match self.tool {
            DragTool::MoveGoal if response.dragged() || response.clicked() => {
                self.set_goal(x, y, self.goal.theta());
            }
            DragTool::RotateGoal if response.dragged() || response.clicked() => {
                let heading = (y - self.goal.y()).atan2(x - self.goal.x());
                self.set_goal(self.goal.x(), self.goal.y(), heading);
            }
            DragTool::Obstacles if response.clicked() => {
                let hit = self
                    .obstacles
                    .iter()
                    .position(|[ox, oy]| (ox - x).hypot(oy - y) < OBSTACLE_RADIUS);
                match hit {
                    Some(index) => {
                        self.obstacles.remove(index);
                    }
                    None if self.obstacles.len() < MAX_OBSTACLES => self.obstacles.push([x, y]),
                    None => {}
                }
                self.rebuild_controller();
            }
            _ => {}
        }
    }

    pub fn ui(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.checkbox(&mut self.running, "Run");
            if ui.button("Reset slider").clicked() {
                self.reset_slider();
            }
            ui.separator();
            ui.label("Drag on the table to:");
            ui.selectable_value(&mut self.tool, DragTool::MoveGoal, "move goal");
            ui.selectable_value(&mut self.tool, DragTool::RotateGoal, "turn goal");
            ui.selectable_value(&mut self.tool, DragTool::Obstacles, "add/remove obstacles");
            ui.separator();
            ui.label("Presets:");
            if ui.button("Translate").clicked() {
                self.set_goal(0.3, 0.0, 0.0);
            }
            if ui.button("Turn 90° in place").clicked() {
                self.set_goal(0.0, 0.0, std::f64::consts::FRAC_PI_2);
                self.reset_slider();
            }
            if ui.button("Sideways").clicked() {
                self.set_goal(0.0, 0.15, 0.0);
                self.reset_slider();
            }
        });
        ui.horizontal(|ui| {
            let mut heading = self.goal.theta().to_degrees() as f32;
            if ui
                .add(egui::Slider::new(&mut heading, -180.0..=180.0).text("goal heading °"))
                .changed()
            {
                self.set_goal(
                    self.goal.x(),
                    self.goal.y(),
                    f64::from(heading).to_radians(),
                );
            }
            if ui
                .add(egui::Slider::new(&mut self.friction, 0.05..=1.0).text("pusher friction μ"))
                .changed()
            {
                self.rebuild_controller();
            }
        });

        let pushed = self.running && self.step();

        let rect = Self::table_rect(ui);
        self.draw(&ui.painter_at(rect), rect);
        let response = ui.allocate_rect(rect, egui::Sense::click_and_drag());
        self.handle_input(&response);

        ui.separator();
        let (position, heading) = self.errors();
        let contact = if self.goal_reached() {
            "Goal reached".to_string()
        } else {
            let mode = match self.last_mode {
                ContactMode::Stick => "stick",
                ContactMode::SlideUp => "slide ↑",
                ContactMode::SlideDown => "slide ↓",
                ContactMode::Separated => "no contact",
            };
            let face = self.last_command.map_or("-", |command| {
                ["back", "+y side", "front", "-y side"][command.face % 4]
            });
            format!("Contact: {mode} on the {face} face")
        };
        ui.label(format!(
            "{contact} · steps {} (stick {}, slide {}) · face switches {} · error {:.1} mm, \
             {:.1}°",
            self.steps,
            self.stick_steps,
            self.slide_steps,
            self.face_switches,
            position * 1000.0,
            heading.to_degrees(),
        ));
        ui.label(
            "Quasi-static pushing: the slider moves only while pushed, and the contact sticks \
             (yellow) or slides along the face (orange) when the push leaves the friction cone. \
             MPPI plans on all four faces and switches faces to turn the slider in place.",
        );

        if (self.running && pushed) || response.dragged() {
            ctx.request_repaint_after(std::time::Duration::from_millis(FRAME_MS));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run_until_reached(demo: &mut PushingDemo, max_steps: usize) -> bool {
        for _ in 0..max_steps {
            if !demo.step() {
                return demo.goal_reached();
            }
        }
        demo.goal_reached()
    }

    #[test]
    fn default_goal_is_reached_with_face_switching() {
        let mut demo = PushingDemo::default();
        assert!(
            run_until_reached(&mut demo, 400),
            "errors {:?}",
            demo.errors()
        );
        assert!(demo.face_switches > 0);
        assert!(demo.stick_steps > 0);
    }

    #[test]
    fn steps_stop_once_the_goal_is_reached() {
        let mut demo = PushingDemo::default();
        demo.set_goal(0.0, 0.0, 0.0);
        assert!(demo.goal_reached());
        assert!(!demo.step());
        assert_eq!(demo.steps, 0);
    }

    #[test]
    fn share_query_round_trips_goal_friction_and_obstacles() {
        let mut demo = PushingDemo::default();
        demo.set_goal(0.25, -0.1, -std::f64::consts::FRAC_PI_4);
        demo.friction = 0.6;
        demo.obstacles = vec![[0.15, 0.02], [0.2, -0.05]];
        demo.running = false;
        let query = demo.share_query();
        let mut restored = PushingDemo::default();
        restored.apply_share_query(&query);
        assert!((restored.goal.x() - 0.25).abs() < 1e-3);
        assert!((restored.goal.y() + 0.1).abs() < 1e-3);
        assert!((restored.goal.theta() + std::f64::consts::FRAC_PI_4).abs() < 0.01);
        assert!((restored.friction - 0.6).abs() < 1e-6);
        assert_eq!(restored.obstacles, demo.obstacles);
        assert!(!restored.running);
        assert!((restored.params.pusher_friction - 0.6).abs() < 1e-6);
    }

    #[test]
    fn screen_and_table_coordinates_round_trip() {
        let rect = Rect::from_min_size(Pos2::new(10.0, 20.0), Vec2::new(600.0, 400.0));
        let screen = PushingDemo::to_screen(rect, 0.1, -0.05);
        let [x, y] = PushingDemo::to_table(rect, screen);
        assert!((x - 0.1).abs() < 1e-4 && (y + 0.05).abs() < 1e-4);
    }

    #[test]
    fn obstacle_on_the_straight_path_is_avoided() {
        let mut demo = PushingDemo {
            obstacles: vec![[0.15, 0.0]],
            ..PushingDemo::default()
        };
        demo.rebuild_controller();
        demo.set_goal(0.3, 0.0, 0.0);
        assert!(
            run_until_reached(&mut demo, 400),
            "errors {:?}",
            demo.errors()
        );
        let clearance = demo
            .trail
            .iter()
            .map(|[x, y]| (x - 0.15).hypot(*y))
            .fold(f64::INFINITY, f64::min);
        let touching = OBSTACLE_RADIUS + demo.params.half_extent;
        assert!(clearance > touching, "clearance {clearance:.3} m");
    }
}
