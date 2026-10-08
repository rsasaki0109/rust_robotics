//! MPPI (Model Predictive Path Integral) control of a point-mass robot among
//! circular obstacles, drawn with its whole sample cloud: every rollout of
//! every control step, shaded by its path-integral weight.

use egui::{Color32, Pos2, Rect, Sense, Shape, Stroke};
use rust_robotics_control::{
    MppiCircularObstacle2D, MppiConfig, MppiControl2D, MppiController2D, MppiMovingObstacle2D,
    MppiSampledPlan2D, MppiState2D,
};

/// The world is the square `[0, WORLD]²` \[m\].
const WORLD: f64 = 20.0;
const ROBOT_RADIUS: f64 = 0.3;
/// Control period and MPPI step \[s\].
const DT: f64 = 0.1;
/// Acceleration limit per axis \[m/s²\].
const CONTROL_LIMIT: f64 = 2.0;
const MAX_OBSTACLES: usize = 24;
const MAX_TRAIL: usize = 800;
/// How close \[m\] a press must be to the robot or goal to drag it.
const GRAB_RADIUS: f64 = 0.8;
/// Close enough \[m\] and slow enough \[m/s\] to count as arrived.
const GOAL_TOLERANCE: f64 = 0.4;
const ARRIVED_SPEED: f64 = 0.3;

const OBSTACLE: Color32 = Color32::from_rgb(70, 76, 92);
const MOVING: Color32 = Color32::from_rgb(120, 92, 150);
/// Sample colors `[r, g, b, a]` at the lowest and the highest weight.
const SAMPLE_LOW: [u8; 4] = [60, 100, 160, 45];
const SAMPLE_HIGH: [u8; 4] = [255, 196, 90, 235];
const PLAN: Color32 = Color32::from_rgb(90, 220, 120);
const TRAIL: Color32 = Color32::from_rgb(255, 152, 56);
const ROBOT: Color32 = Color32::from_rgb(235, 240, 245);
const GOAL: Color32 = Color32::from_rgb(240, 90, 90);

/// What a drag on the field moves.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Grab {
    Robot,
    Goal,
}

/// A moving obstacle that bounces off the world's edges.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Mover {
    x: f64,
    y: f64,
    vx: f64,
    vy: f64,
    radius: f64,
}

impl Mover {
    fn step(&mut self, dt: f64) {
        self.x += self.vx * dt;
        self.y += self.vy * dt;
        if !(self.radius..=WORLD - self.radius).contains(&self.x) {
            self.vx = -self.vx;
            self.x = self.x.clamp(self.radius, WORLD - self.radius);
        }
        if !(self.radius..=WORLD - self.radius).contains(&self.y) {
            self.vy = -self.vy;
            self.y = self.y.clamp(self.radius, WORLD - self.radius);
        }
    }
}

fn default_movers() -> Vec<Mover> {
    vec![
        Mover {
            x: 4.0,
            y: 13.5,
            vx: 1.6,
            vy: 0.0,
            radius: 1.0,
        },
        Mover {
            x: 13.0,
            y: 3.0,
            vx: 0.0,
            vy: 1.3,
            radius: 0.9,
        },
    ]
}

fn default_obstacles() -> Vec<(f64, f64, f64)> {
    vec![
        (6.0, 6.0, 1.6),
        (10.0, 10.0, 2.0),
        (5.0, 15.0, 1.4),
        (15.0, 6.0, 1.5),
        (14.5, 14.0, 1.7),
        (9.5, 3.0, 1.0),
        (3.0, 10.0, 1.0),
        (17.0, 10.5, 1.0),
    ]
}

/// MPPI knobs exposed in the panel (and in share links).
#[derive(Debug, Clone, Copy, PartialEq)]
struct Knobs {
    samples: usize,
    horizon: usize,
    lambda: f64,
    sigma: f64,
}

impl Default for Knobs {
    fn default() -> Self {
        Self {
            samples: 256,
            horizon: 30,
            // Costs here run to the thousands; λ = 30 keeps about 10-20 %
            // of the samples effective, so the weighting is visible.
            lambda: 30.0,
            sigma: 1.2,
        }
    }
}

impl Knobs {
    fn config(self, obstacles: &[(f64, f64, f64)]) -> MppiConfig {
        MppiConfig {
            horizon: self.horizon,
            samples: self.samples,
            dt: DT,
            lambda: self.lambda,
            noise_sigma: self.sigma,
            control_limit: CONTROL_LIMIT,
            goal_weight: 1.0,
            velocity_weight: 3.0,
            control_weight: 0.05,
            terminal_weight: 6.0,
            // Steep enough that grazing an obstacle outweighs any detour.
            constraint_weight: 5000.0,
            safety_margin: ROBOT_RADIUS + 0.3,
            obstacles: obstacles
                .iter()
                .map(|&(x, y, r)| MppiCircularObstacle2D::new(x, y, r))
                .collect(),
            seed: 7,
            ..MppiConfig::default()
        }
    }
}

pub struct MppiDemo {
    knobs: Knobs,
    /// Circles `(x, y, radius)`.
    obstacles: Vec<(f64, f64, f64)>,
    moving: bool,
    movers: Vec<Mover>,
    start: [f64; 2],
    goal: [f64; 2],
    state: MppiState2D,
    controller: Option<MppiController2D>,
    /// The latest step's sample cloud.
    cloud: Option<MppiSampledPlan2D>,
    trail: Vec<[f64; 2]>,
    playing: bool,
    show_samples: bool,
    /// Simulated time not yet stepped \[s\].
    pending: f64,
    elapsed: f64,
    /// Steps spent overlapping an obstacle.
    contacts: usize,
    arrived_at: Option<f64>,
    /// Radius of the next obstacle placed \[m\].
    new_radius: f32,
    grab: Option<Grab>,
}

impl Default for MppiDemo {
    fn default() -> Self {
        let start = [2.0, 2.0];
        Self {
            knobs: Knobs::default(),
            obstacles: default_obstacles(),
            moving: true,
            movers: default_movers(),
            start,
            goal: [17.5, 17.5],
            state: MppiState2D::new(start[0], start[1], 0.0, 0.0),
            controller: None,
            cloud: None,
            trail: vec![start],
            playing: true,
            show_samples: true,
            pending: 0.0,
            elapsed: 0.0,
            contacts: 0,
            arrived_at: None,
            new_radius: 1.2,
            grab: None,
        }
    }
}

fn to_screen(rect: Rect, p: [f64; 2]) -> Pos2 {
    let scale = rect.width() / WORLD as f32;
    Pos2::new(
        rect.left() + p[0] as f32 * scale,
        rect.bottom() - p[1] as f32 * scale,
    )
}

fn to_world(rect: Rect, pos: Pos2) -> [f64; 2] {
    let scale = rect.width() / WORLD as f32;
    [
        f64::from((pos.x - rect.left()) / scale).clamp(0.0, WORLD),
        f64::from((rect.bottom() - pos.y) / scale).clamp(0.0, WORLD),
    ]
}

fn parse_point(value: &str) -> Option<[f64; 2]> {
    let (x, y) = value.split_once(',')?;
    let point = [x.parse::<f64>().ok()?, y.parse::<f64>().ok()?];
    point
        .iter()
        .all(|v| v.is_finite() && (0.0..=WORLD).contains(v))
        .then_some(point)
}

/// Mixes the low- and high-weight sample colors (`t` in `[0, 1]`).
fn sample_color(t: f32) -> Color32 {
    let mix = |i: usize| {
        let (a, b) = (f32::from(SAMPLE_LOW[i]), f32::from(SAMPLE_HIGH[i]));
        (a + (b - a) * t).round() as u8
    };
    Color32::from_rgba_unmultiplied(mix(0), mix(1), mix(2), mix(3))
}

impl MppiDemo {
    pub fn apply_share_query(&mut self, query: &str) {
        if crate::share::value(query, "tab") != Some("mppi") {
            return;
        }
        let number = |key| crate::share::value(query, key).and_then(|v| v.parse::<f64>().ok());
        if let Some(samples) = number("samples").filter(|v| v.is_finite()) {
            self.knobs.samples = (samples as usize).clamp(16, 1024);
        }
        if let Some(horizon) = number("horizon").filter(|v| v.is_finite()) {
            self.knobs.horizon = (horizon as usize).clamp(5, 60);
        }
        if let Some(lambda) = number("lambda").filter(|v| v.is_finite()) {
            self.knobs.lambda = lambda.clamp(0.01, 100.0);
        }
        if let Some(sigma) = number("sigma").filter(|v| v.is_finite()) {
            self.knobs.sigma = sigma.clamp(0.1, 4.0);
        }
        if let Some(moving) = crate::share::value(query, "moving") {
            self.moving = moving != "0";
        }
        if let Some(goal) = crate::share::value(query, "goal").and_then(parse_point) {
            self.goal = goal;
        }
        if let Some(start) = crate::share::value(query, "start").and_then(parse_point) {
            self.start = start;
        }
        if let Some(value) = crate::share::value(query, "obstacles") {
            self.obstacles = value
                .split(';')
                .filter_map(|circle| {
                    let mut parts = circle.split(',').map(|v| v.parse::<f64>().ok());
                    let (x, y, r) = (parts.next()??, parts.next()??, parts.next()??);
                    ([x, y, r].iter().all(|v| v.is_finite()) && r > 0.0).then_some((
                        x.clamp(0.0, WORLD),
                        y.clamp(0.0, WORLD),
                        r.min(4.0),
                    ))
                })
                .take(MAX_OBSTACLES)
                .collect();
        }
        self.restart();
    }

    pub fn share_query(&self) -> String {
        let obstacles: Vec<String> = self
            .obstacles
            .iter()
            .map(|(x, y, r)| format!("{x:.1},{y:.1},{r:.1}"))
            .collect();
        format!(
            "tab=mppi&samples={}&horizon={}&lambda={}&sigma={:.2}&moving={}&start={:.1},{:.1}&goal={:.1},{:.1}&obstacles={}",
            self.knobs.samples,
            self.knobs.horizon,
            self.knobs.lambda,
            self.knobs.sigma,
            u8::from(self.moving),
            self.start[0],
            self.start[1],
            self.goal[0],
            self.goal[1],
            obstacles.join(";")
        )
    }

    /// Back to the start, with fresh movers and a fresh controller.
    fn restart(&mut self) {
        self.state = MppiState2D::new(self.start[0], self.start[1], 0.0, 0.0);
        self.movers = default_movers();
        self.trail = vec![self.start];
        self.elapsed = 0.0;
        self.contacts = 0;
        self.arrived_at = None;
        self.pending = 0.0;
        self.cloud = None;
        self.controller = None;
    }

    /// New knobs or obstacles: keep the robot where it is, plan afresh.
    fn rebuild(&mut self) {
        self.controller = None;
        self.cloud = None;
    }

    fn controller(&mut self) -> &mut MppiController2D {
        if self.controller.is_none() {
            let config = self.knobs.config(&self.obstacles);
            self.controller = Some(
                MppiController2D::new(config).expect("the panel only offers valid MPPI settings"),
            );
        }
        self.controller.as_mut().expect("just created")
    }

    /// One control period: plan, apply the first control, move everything.
    fn step(&mut self) {
        let movers: Vec<MppiMovingObstacle2D> = if self.moving {
            self.movers
                .iter()
                .map(|m| MppiMovingObstacle2D::new(m.x, m.y, m.vx, m.vy, m.radius))
                .collect()
        } else {
            Vec::new()
        };
        let state = self.state;
        let goal = (self.goal[0], self.goal[1]);
        let controller = self.controller();
        controller
            .set_moving_obstacles(movers)
            .expect("movers are finite");
        let control = match controller.plan_with_samples(state, goal) {
            Ok(cloud) => {
                let control = cloud.plan.first_control;
                self.cloud = Some(cloud);
                control
            }
            Err(_) => MppiControl2D::new(0.0, 0.0),
        };
        let mut next = self.state.step(control, DT);
        // The world's edge is a wall.
        for (position, velocity) in [(&mut next.x, &mut next.vx), (&mut next.y, &mut next.vy)] {
            if !(ROBOT_RADIUS..=WORLD - ROBOT_RADIUS).contains(position) {
                *position = position.clamp(ROBOT_RADIUS, WORLD - ROBOT_RADIUS);
                *velocity = 0.0;
            }
        }
        self.state = next;
        if self.moving {
            for mover in &mut self.movers {
                mover.step(DT);
            }
        }
        self.elapsed += DT;
        if self.in_contact() {
            self.contacts += 1;
        }
        let position = [self.state.x, self.state.y];
        if self.trail.last().map_or(true, |last| {
            (last[0] - position[0]).hypot(last[1] - position[1]) > 0.05
        }) {
            self.trail.push(position);
            if self.trail.len() > MAX_TRAIL {
                self.trail.remove(0);
            }
        }
        let distance = (position[0] - self.goal[0]).hypot(position[1] - self.goal[1]);
        let speed = self.state.vx.hypot(self.state.vy);
        if self.arrived_at.is_none() && distance < GOAL_TOLERANCE && speed < ARRIVED_SPEED {
            self.arrived_at = Some(self.elapsed);
        }
    }

    fn in_contact(&self) -> bool {
        let (x, y) = (self.state.x, self.state.y);
        let statics = self
            .obstacles
            .iter()
            .any(|&(ox, oy, r)| (x - ox).hypot(y - oy) < r + ROBOT_RADIUS);
        let moving = self.moving
            && self
                .movers
                .iter()
                .any(|m| (x - m.x).hypot(y - m.y) < m.radius + ROBOT_RADIUS);
        statics || moving
    }

    fn near(a: [f64; 2], b: [f64; 2]) -> bool {
        (a[0] - b[0]).hypot(a[1] - b[1]) < GRAB_RADIUS
    }

    fn handle_pointer(&mut self, rect: Rect, response: &egui::Response) {
        let robot = [self.state.x, self.state.y];
        if response.drag_started() {
            let origin = response
                .ctx
                .input(|i| i.pointer.press_origin())
                .map(|pos| to_world(rect, pos));
            self.grab = origin.and_then(|p| {
                if Self::near(p, self.goal) {
                    Some(Grab::Goal)
                } else if Self::near(p, robot) {
                    Some(Grab::Robot)
                } else {
                    None
                }
            });
        }
        if response.dragged() {
            if let (Some(grab), Some(pos)) = (self.grab, response.interact_pointer_pos()) {
                let p = to_world(rect, pos);
                match grab {
                    Grab::Goal => {
                        self.goal = p;
                        self.arrived_at = None;
                    }
                    Grab::Robot => {
                        self.start = p;
                        self.state = MppiState2D::new(p[0], p[1], 0.0, 0.0);
                        self.trail = vec![p];
                        self.arrived_at = None;
                        self.cloud = None;
                    }
                }
            }
        }
        if response.drag_stopped() {
            self.grab = None;
        }
        if response.clicked() {
            let Some(p) = response
                .interact_pointer_pos()
                .map(|pos| to_world(rect, pos))
            else {
                return;
            };
            if Self::near(p, self.goal) || Self::near(p, robot) {
                return;
            }
            let hit = self
                .obstacles
                .iter()
                .position(|&(x, y, r)| (p[0] - x).hypot(p[1] - y) <= r);
            match hit {
                Some(index) => {
                    self.obstacles.remove(index);
                }
                None if self.obstacles.len() < MAX_OBSTACLES => {
                    let r = f64::from(self.new_radius);
                    let blocks = |q: [f64; 2]| (q[0] - p[0]).hypot(q[1] - p[1]) <= r + ROBOT_RADIUS;
                    if blocks(robot) || blocks(self.goal) {
                        return;
                    }
                    self.obstacles.push((p[0], p[1], r));
                }
                None => return,
            }
            self.rebuild();
        }
    }

    pub fn controls(&mut self, _ctx: &egui::Context, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Run");
        ui.horizontal_wrapped(|ui| {
            let label = if self.playing { "Pause" } else { "Play" };
            if ui.button(label).clicked() {
                self.playing = !self.playing;
            }
            if ui.button("Step").clicked() {
                self.playing = false;
                self.step();
            }
            if ui.button("Restart").clicked() {
                self.restart();
            }
        });
        ui.horizontal_wrapped(|ui| {
            if ui.checkbox(&mut self.moving, "Moving obstacles").changed() {
                self.movers = default_movers();
            }
            ui.checkbox(&mut self.show_samples, "Show samples");
        });

        crate::ui_kit::section(ui, "MPPI");
        let before = self.knobs;
        let mut samples = self.knobs.samples as f64;
        ui.add(
            egui::Slider::new(&mut samples, 16.0..=1024.0)
                .logarithmic(true)
                .integer()
                .text("samples"),
        );
        self.knobs.samples = samples.round() as usize;
        ui.add(
            egui::Slider::new(&mut self.knobs.horizon, 5..=60)
                .suffix(" steps")
                .text("horizon"),
        );
        ui.add(
            egui::Slider::new(&mut self.knobs.lambda, 0.01..=100.0)
                .logarithmic(true)
                .text("temperature λ"),
        );
        ui.add(
            egui::Slider::new(&mut self.knobs.sigma, 0.1..=4.0)
                .suffix(" m/s²")
                .text("noise σ"),
        );
        if self.knobs != before {
            self.rebuild();
        }
        if ui.button("Default settings").clicked() {
            self.knobs = Knobs::default();
            self.rebuild();
        }

        crate::ui_kit::section(ui, "Obstacles");
        ui.add(
            egui::Slider::new(&mut self.new_radius, 0.4..=3.0)
                .suffix(" m")
                .text("new radius"),
        );
        ui.horizontal_wrapped(|ui| {
            if ui.button("Clear").clicked() {
                self.obstacles.clear();
                self.rebuild();
            }
            if ui.button("Reset field").clicked() {
                self.obstacles = default_obstacles();
                self.rebuild();
            }
        });
        crate::ui_kit::hint(
            ui,
            "Drag the red goal or the robot. Click the field to add an obstacle, click one \
             to remove it.",
        );
        crate::ui_kit::legend(
            ui,
            &[
                (sample_color(0.4), "low-weight samples"),
                (sample_color(1.0), "high-weight samples"),
                (PLAN, "plan"),
                (TRAIL, "path driven"),
                (MOVING, "moving obstacle"),
            ],
        );
        crate::ui_kit::how_it_works(
            ui,
            "mppi_help",
            "Every 0.1 s MPPI perturbs its current plan with random accelerations (noise σ) \
             and simulates each perturbed plan over the horizon. Every rollout gets a cost \
             (distance to the goal, speed, effort, and a steep penalty inside an obstacle, \
             which for moving obstacles uses their predicted positions) and a weight \
             exp(-cost / λ). The new plan is the weighted average of all rollouts; the robot \
             applies its first acceleration and the rest warm-starts the next step.\n\n\
             A low temperature λ follows only the very best rollouts (few effective \
             samples, jittery); a high one averages many (smooth, but it can blur a gap \
             between obstacles). More samples find narrow gaps more reliably; a longer \
             horizon sees further but costs more.",
        );
    }

    pub fn scene(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        let rect = crate::ui_kit::fit_rect(ui, 1.0, 34.0);
        let response = ui.allocate_rect(rect, Sense::click_and_drag());
        self.handle_pointer(rect, &response);

        if self.playing && self.grab != Some(Grab::Robot) {
            self.pending += f64::from(ctx.input(|i| i.stable_dt)).min(0.1);
            // At most two control steps a frame, so a slow device slows the
            // clock instead of falling behind.
            let mut steps = 0;
            while self.pending >= DT && steps < 2 {
                self.pending -= DT;
                self.step();
                steps += 1;
            }
            self.pending = self.pending.min(DT);
            ctx.request_repaint();
        } else if self.cloud.is_none() {
            // Show what MPPI would do from here.
            self.step_preview();
        }

        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_rgb(18, 22, 28));
        let scale = rect.width() / WORLD as f32;
        for &(x, y, r) in &self.obstacles {
            painter.circle_filled(to_screen(rect, [x, y]), r as f32 * scale, OBSTACLE);
        }
        if self.moving {
            for mover in &self.movers {
                painter.circle_filled(
                    to_screen(rect, [mover.x, mover.y]),
                    mover.radius as f32 * scale,
                    MOVING,
                );
            }
        }

        if let Some(cloud) = &self.cloud {
            if self.show_samples {
                let max_weight = cloud.weights.iter().copied().fold(0.0_f64, f64::max);
                let mut order: Vec<usize> = (0..cloud.rollouts.len()).collect();
                order.sort_by(|&a, &b| cloud.weights[a].total_cmp(&cloud.weights[b]));
                for index in order {
                    let t = if max_weight > 0.0 {
                        (cloud.weights[index] / max_weight).sqrt() as f32
                    } else {
                        0.0
                    };
                    let points = cloud.rollouts[index]
                        .states
                        .iter()
                        .map(|s| to_screen(rect, [s.x, s.y]))
                        .collect();
                    painter.add(Shape::line(
                        points,
                        Stroke::new(1.0 + 1.5 * t, sample_color(t)),
                    ));
                }
            }
            let mut state = self.state;
            let mut plan = vec![to_screen(rect, [state.x, state.y])];
            for control in cloud.plan.nominal_controls.iter().skip(1) {
                state = state.step(*control, DT);
                plan.push(to_screen(rect, [state.x, state.y]));
            }
            painter.add(Shape::line(plan, Stroke::new(3.0_f32, PLAN)));
        }
        if self.trail.len() >= 2 {
            let points = self.trail.iter().map(|p| to_screen(rect, *p)).collect();
            painter.add(Shape::line(points, Stroke::new(2.0_f32, TRAIL)));
        }
        painter.circle_filled(to_screen(rect, self.goal), 7.0, GOAL);
        painter.circle_filled(
            to_screen(rect, [self.state.x, self.state.y]),
            (ROBOT_RADIUS as f32 * scale).max(5.0),
            ROBOT,
        );

        let speed = self.state.vx.hypot(self.state.vy);
        let mut status = match self.arrived_at {
            Some(t) => format!("Reached the goal in {t:.1} s"),
            None => format!("{:.1} s  ·  speed {speed:.1} m/s", self.elapsed),
        };
        if let Some(cloud) = &self.cloud {
            let d = cloud.plan.sampling_diagnostics;
            status.push_str(&format!(
                "  ·  effective samples {:.0} of {}",
                d.effective_sample_size, d.sample_count
            ));
        }
        if self.contacts > 0 {
            status.push_str(&format!(
                "  ·  in contact {:.1} s",
                self.contacts as f64 * DT
            ));
        }
        ui.label(status);
    }

    /// Plans once from the current state without moving (paused view).
    fn step_preview(&mut self) {
        let state = self.state;
        let goal = (self.goal[0], self.goal[1]);
        if let Ok(cloud) = self.controller().plan_with_samples(state, goal) {
            self.cloud = Some(cloud);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(demo: &mut MppiDemo, seconds: f64) {
        for _ in 0..(seconds / DT).round() as usize {
            demo.step();
            if demo.arrived_at.is_some() {
                break;
            }
        }
    }

    #[test]
    fn reaches_the_goal_through_the_default_field_without_contact() {
        for moving in [false, true] {
            let mut demo = MppiDemo {
                moving,
                ..MppiDemo::default()
            };
            run(&mut demo, 40.0);
            assert!(
                demo.arrived_at.is_some(),
                "moving={moving}: stuck at ({:.1}, {:.1})",
                demo.state.x,
                demo.state.y
            );
            assert_eq!(demo.contacts, 0, "moving={moving}");
        }
    }

    #[test]
    fn extreme_settings_keep_planning() {
        for (samples, horizon, lambda, sigma) in [
            (16, 5, 0.01, 0.1),
            (1024, 60, 100.0, 4.0),
            (16, 60, 0.01, 4.0),
        ] {
            let mut demo = MppiDemo {
                knobs: Knobs {
                    samples,
                    horizon,
                    lambda,
                    sigma,
                },
                ..MppiDemo::default()
            };
            for _ in 0..5 {
                demo.step();
                assert!(demo.cloud.is_some());
                assert!(demo.state.x.is_finite() && demo.state.y.is_finite());
            }
        }
    }

    #[test]
    fn the_sample_cloud_has_one_weight_per_rollout() {
        let mut demo = MppiDemo::default();
        demo.step();
        let cloud = demo.cloud.as_ref().unwrap();
        assert_eq!(cloud.rollouts.len(), demo.knobs.samples + 1);
        assert_eq!(cloud.weights.len(), cloud.rollouts.len());
        assert_eq!(cloud.rollouts[0].states.len(), demo.knobs.horizon + 1);
    }

    #[test]
    fn share_query_round_trips() {
        let mut demo = MppiDemo {
            knobs: Knobs {
                samples: 64,
                horizon: 20,
                lambda: 0.5,
                sigma: 2.0,
            },
            moving: false,
            goal: [12.0, 4.0],
            start: [1.0, 18.0],
            obstacles: vec![(5.0, 5.0, 1.0), (8.5, 9.0, 2.5)],
            ..MppiDemo::default()
        };
        let query = demo.share_query();
        let mut restored = MppiDemo::default();
        restored.apply_share_query(&query);
        assert_eq!(restored.knobs, demo.knobs);
        assert_eq!(restored.moving, demo.moving);
        assert_eq!(restored.goal, demo.goal);
        assert_eq!(restored.start, demo.start);
        assert_eq!(restored.obstacles, demo.obstacles);
        assert_eq!(restored.state.x, 1.0);
        demo.restart();
        assert_eq!(restored.share_query(), demo.share_query());
    }
}
