//! Parking with Hybrid A*: drag out a goal pose in a parking lot and watch a
//! car-like robot plan a path with forward and reverse segments (Reeds-Shepp
//! shortcuts included) and drive it.

use web_time::Instant;

use egui::{Color32, Pos2, Rect, Sense, Shape, Stroke};
use rust_robotics_planning::{HybridAStarConfig, HybridAStarPlanner, VehicleFootprint};

/// The lot is `[0, LOT_W] × [0, LOT_H]` \[m\].
const LOT_W: f64 = 30.0;
const LOT_H: f64 = 20.0;
const WHEELBASE: f64 = 2.7;
const MAX_STEER: f64 = 0.6;
const CAR: VehicleFootprint = VehicleFootprint {
    front: 3.6,
    rear: 1.0,
    width: 1.9,
};
/// Parked cars, `(min x, min y, width, height)`.
const PARKED_LONG: f64 = 4.6;
/// Driving speed of the animation \[m/s\].
const DRIVE_SPEED: f64 = 3.0;
/// Hybrid A* gives up after this many expansions (keeps a hopeless goal
/// from freezing the page).
const MAX_EXPANSIONS: usize = 40_000;
/// A drag shorter than this \[px\] keeps the goal's heading.
const MIN_HEADING_DRAG: f32 = 12.0;

const ASPHALT: Color32 = Color32::from_rgb(28, 32, 39);
const LINE: Color32 = Color32::from_rgb(70, 76, 88);
const PARKED: Color32 = Color32::from_rgb(78, 84, 98);
const EGO: Color32 = Color32::from_rgb(80, 170, 255);
const GOAL: Color32 = Color32::from_rgb(90, 220, 120);
const FORWARD: Color32 = Color32::from_rgb(80, 170, 255);
const REVERSE: Color32 = Color32::from_rgb(255, 152, 56);

/// A pose `[x, y, yaw]` of the rear axle.
type Pose = [f64; 3];

/// The parked cars: a curbside row with a gap (parallel parking) and a row
/// of perpendicular slots with one free.
fn parked_cars() -> Vec<[f64; 4]> {
    let mut cars = Vec::new();
    // The gap between the second and third car is 7.5 m.
    for x in [0.4, 5.3, 17.4, 22.3] {
        cars.push([x, 0.4, PARKED_LONG, 1.9]);
    }
    for slot in 0..10 {
        if slot == 6 {
            continue;
        }
        let x = 1.0 + slot as f64 * 2.9;
        cars.push([x + 0.5, LOT_H - 0.4 - PARKED_LONG, 1.9, PARKED_LONG]);
    }
    cars
}

/// Obstacle points for the planner: the lot's walls and the parked cars,
/// filled.
fn obstacle_points(cars: &[[f64; 4]]) -> (Vec<f64>, Vec<f64>) {
    let (mut ox, mut oy) = (Vec::new(), Vec::new());
    let step = 0.25;
    let mut t = 0.0;
    while t <= LOT_W + 1e-9 {
        for y in [0.0, LOT_H] {
            ox.push(t);
            oy.push(y);
        }
        t += step;
    }
    let mut t = 0.0;
    while t <= LOT_H + 1e-9 {
        for x in [0.0, LOT_W] {
            ox.push(x);
            oy.push(t);
        }
        t += step;
    }
    for &[x0, y0, w, h] in cars {
        let (nx, ny) = ((w / step).ceil() as usize, (h / step).ceil() as usize);
        for i in 0..=nx {
            for j in 0..=ny {
                ox.push(x0 + w * i as f64 / nx as f64);
                oy.push(y0 + h * j as f64 / ny as f64);
            }
        }
    }
    (ox, oy)
}

fn planner_config() -> HybridAStarConfig {
    HybridAStarConfig {
        xy_resolution: 0.5,
        yaw_resolution: std::f64::consts::PI / 36.0,
        wheelbase: WHEELBASE,
        max_steer: MAX_STEER,
        n_steer: 6,
        step_size: 0.25,
        max_curvature: MAX_STEER.tan() / WHEELBASE,
        switch_back_cost: 15.0,
        analytic_expansion_interval: 3,
        ..HybridAStarConfig::default()
    }
}

/// A planned path ready to drive.
#[derive(Debug, Clone)]
struct Plan {
    poses: Vec<Pose>,
    /// +1 forward, -1 reverse, per pose.
    directions: Vec<i32>,
    /// Arc length from the start to each pose \[m\].
    distance: Vec<f64>,
    switches: usize,
    elapsed_ms: f64,
}

impl Plan {
    fn length(&self) -> f64 {
        self.distance.last().copied().unwrap_or(0.0)
    }

    /// The pose `s` meters along the path.
    fn pose_at(&self, s: f64) -> Pose {
        let i = self.distance.partition_point(|&d| d <= s).max(1);
        if i >= self.poses.len() {
            return *self.poses.last().expect("non-empty plan");
        }
        let (a, b) = (self.poses[i - 1], self.poses[i]);
        let span = (self.distance[i] - self.distance[i - 1]).max(1e-9);
        let t = ((s - self.distance[i - 1]) / span).clamp(0.0, 1.0);
        let dyaw = (b[2] - a[2] + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
            - std::f64::consts::PI;
        [
            a[0] + t * (b[0] - a[0]),
            a[1] + t * (b[1] - a[1]),
            a[2] + t * dyaw,
        ]
    }
}

pub struct ParkingDemo {
    cars: Vec<[f64; 4]>,
    /// Where the car is (the start of the next plan).
    car: Pose,
    goal: Pose,
    plan: Option<Plan>,
    /// Why the last plan failed.
    failure: Option<String>,
    /// Meters driven along the current plan.
    driven: f64,
    /// Input time of the last animation frame \[s\].
    last_time: Option<f64>,
    /// Goal being dragged out: press position (world) and current heading.
    dragging: Option<Pose>,
    /// Built on first use and kept: the parked cars never move.
    planner: Option<HybridAStarPlanner>,
    /// Plan on the next frame (set at startup and by share links, so a
    /// visitor who never opens this tab pays nothing).
    needs_plan: bool,
}

/// Start and goal of the parallel-parking preset (the default scene).
const PARALLEL_START: Pose = [3.0, 8.0, 0.0];
/// Centered in the curbside gap, rear axle 1 m ahead of the rear bumper.
const PARALLEL_GOAL: Pose = [9.9 + (7.5 - PARKED_LONG) / 2.0 + CAR.rear, 1.35, 0.0];

impl Default for ParkingDemo {
    fn default() -> Self {
        Self {
            cars: parked_cars(),
            car: PARALLEL_START,
            goal: PARALLEL_GOAL,
            plan: None,
            failure: None,
            driven: 0.0,
            last_time: None,
            dragging: None,
            planner: None,
            needs_plan: true,
        }
    }
}

fn wrap(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Corners of the car at `pose`, counter-clockwise.
fn car_corners(pose: Pose) -> [[f64; 2]; 4] {
    let (s, c) = pose[2].sin_cos();
    let at = |along: f64, side: f64| {
        [
            pose[0] + along * c - side * s,
            pose[1] + along * s + side * c,
        ]
    };
    let half = CAR.width / 2.0;
    [
        at(-CAR.rear, -half),
        at(CAR.front, -half),
        at(CAR.front, half),
        at(-CAR.rear, half),
    ]
}

fn to_screen(rect: Rect, p: [f64; 2]) -> Pos2 {
    let scale = rect.width() / LOT_W as f32;
    Pos2::new(
        rect.left() + p[0] as f32 * scale,
        rect.bottom() - p[1] as f32 * scale,
    )
}

fn to_world(rect: Rect, pos: Pos2) -> [f64; 2] {
    let scale = rect.width() / LOT_W as f32;
    [
        f64::from((pos.x - rect.left()) / scale).clamp(0.0, LOT_W),
        f64::from((rect.bottom() - pos.y) / scale).clamp(0.0, LOT_H),
    ]
}

fn parse_pose(value: &str) -> Option<Pose> {
    let mut parts = value.split(',').map(|v| v.parse::<f64>().ok());
    let pose = [parts.next()??, parts.next()??, parts.next()??];
    (pose.iter().all(|v| v.is_finite())
        && (0.0..=LOT_W).contains(&pose[0])
        && (0.0..=LOT_H).contains(&pose[1]))
    .then(|| [pose[0], pose[1], pose[2].to_radians()])
}

impl ParkingDemo {
    pub fn apply_share_query(&mut self, query: &str) {
        if crate::share::value(query, "tab") != Some("parking") {
            return;
        }
        let car = crate::share::value(query, "car").and_then(parse_pose);
        let goal = crate::share::value(query, "goal").and_then(parse_pose);
        if let Some(car) = car {
            self.car = car;
        }
        if let Some(goal) = goal {
            self.goal = [goal[0], goal[1], wrap(goal[2])];
        }
        if car.is_some() || goal.is_some() {
            // The link's car pose is the start, not where a plan left it.
            self.plan = None;
            self.failure = None;
            self.needs_plan = true;
        }
    }

    pub fn share_query(&self) -> String {
        format!(
            "tab=parking&car={:.1},{:.1},{:.0}&goal={:.1},{:.1},{:.0}",
            self.car[0],
            self.car[1],
            self.car[2].to_degrees(),
            self.goal[0],
            self.goal[1],
            self.goal[2].to_degrees()
        )
    }

    /// Plans from where the car is now to `goal`.
    fn set_goal(&mut self, goal: Pose) {
        // Leave the car at the last path sample it reached: samples were
        // collision-checked, poses between them were not.
        if let Some(plan) = &self.plan {
            let reached = plan
                .distance
                .partition_point(|&d| d <= self.driven)
                .saturating_sub(1);
            self.car = plan.poses[reached];
        }
        self.goal = [goal[0], goal[1], wrap(goal[2])];
        self.plan_now();
    }

    /// Plans from `self.car` to `self.goal`.
    fn plan_now(&mut self) {
        self.needs_plan = false;
        self.driven = 0.0;
        if self.planner.is_none() {
            let (ox, oy) = obstacle_points(&self.cars);
            match HybridAStarPlanner::with_vehicle(&ox, &oy, planner_config(), CAR) {
                Ok(planner) => self.planner = Some(planner.with_max_expansions(MAX_EXPANSIONS)),
                Err(error) => {
                    self.plan = None;
                    self.failure = Some(error.to_string());
                    return;
                }
            }
        }
        let planner = self.planner.as_mut().expect("planner built above");
        let timer = Instant::now();
        let result = planner.plan(
            self.car[0],
            self.car[1],
            self.car[2],
            self.goal[0],
            self.goal[1],
            self.goal[2],
        );
        let elapsed_ms = timer.elapsed().as_secs_f64() * 1000.0;
        match result {
            Ok(path) if !path.is_empty() => {
                let poses: Vec<Pose> = (0..path.len())
                    .map(|i| [path.x[i], path.y[i], path.yaw[i]])
                    .collect();
                let mut distance = vec![0.0];
                for w in poses.windows(2) {
                    let last = *distance.last().unwrap();
                    distance.push(last + (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]));
                }
                let switches = path.directions.windows(2).filter(|w| w[0] != w[1]).count();
                self.plan = Some(Plan {
                    poses,
                    directions: path.directions,
                    distance,
                    switches,
                    elapsed_ms,
                });
                self.failure = None;
            }
            Ok(_) => {
                self.plan = None;
                self.failure = Some("empty path".to_string());
            }
            Err(error) => {
                self.plan = None;
                self.failure = Some(
                    error
                        .to_string()
                        .rsplit(": ")
                        .next()
                        .unwrap_or("no path")
                        .to_string(),
                );
            }
        }
        self.last_time = None;
    }

    fn preset_parallel(&mut self) {
        self.car = PARALLEL_START;
        self.plan = None;
        self.set_goal(PARALLEL_GOAL);
    }

    /// Runs the plan a share link or startup asked for.
    pub(crate) fn ensure_planned(&mut self) {
        if self.needs_plan {
            self.plan_now();
        }
    }

    fn preset_back_in(&mut self) {
        self.car = [4.0, 9.0, 0.0];
        self.plan = None;
        let slot_center = 1.0 + 6.0 * 2.9 + 0.5 + 0.95;
        self.set_goal([
            slot_center,
            LOT_H - 0.6 - CAR.rear,
            -std::f64::consts::FRAC_PI_2,
        ]);
    }

    fn preset_turn(&mut self) {
        self.car = [15.0, 7.0, 0.0];
        self.plan = None;
        self.set_goal([15.0, 7.0, std::f64::consts::PI]);
    }

    /// The pose the car is drawn at.
    fn current_pose(&self) -> Pose {
        self.plan
            .as_ref()
            .map_or(self.car, |plan| plan.pose_at(self.driven))
    }

    fn handle_pointer(&mut self, rect: Rect, response: &egui::Response) {
        let press = response
            .ctx
            .input(|i| i.pointer.press_origin())
            .filter(|pos| rect.contains(*pos));
        if response.drag_started() {
            if let Some(pos) = press {
                let [x, y] = to_world(rect, pos);
                self.dragging = Some([x, y, self.goal[2]]);
            }
        }
        if let (Some(goal), Some(origin), Some(pos)) =
            (&mut self.dragging, press, response.interact_pointer_pos())
        {
            let drag = pos - origin;
            if drag.length() > MIN_HEADING_DRAG {
                goal[2] = f64::from(-drag.y).atan2(f64::from(drag.x));
            }
        }
        if response.drag_stopped() {
            if let Some(goal) = self.dragging.take() {
                self.set_goal(goal);
            }
        }
        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                let [x, y] = to_world(rect, pos);
                self.set_goal([x, y, self.goal[2]]);
            }
        }
    }

    pub fn controls(&mut self, _ctx: &egui::Context, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Try");
        ui.horizontal_wrapped(|ui| {
            if ui.button("Parallel park").clicked() {
                self.preset_parallel();
            }
            if ui.button("Back into a slot").clicked() {
                self.preset_back_in();
            }
            if ui.button("Turn around").clicked() {
                self.preset_turn();
            }
        });
        if ui
            .button("Drive it again")
            .on_hover_text("Replay the current path from its start")
            .clicked()
        {
            self.driven = 0.0;
            self.last_time = None;
        }
        crate::ui_kit::hint(
            ui,
            "Press on the lot where the car should go and drag toward where it should \
             face. A click keeps the last heading. The next plan starts where the car is.",
        );

        if let Some(plan) = &self.plan {
            crate::ui_kit::section(ui, "Plan");
            egui::Grid::new("parking_plan")
                .num_columns(2)
                .spacing([12.0, 2.0])
                .show(ui, |ui| {
                    let row = |ui: &mut egui::Ui, name: &str, value: String| {
                        ui.label(egui::RichText::new(name).weak());
                        ui.monospace(value);
                        ui.end_row();
                    };
                    row(ui, "length", format!("{:.1} m", plan.length()));
                    row(ui, "gear changes", plan.switches.to_string());
                    row(ui, "planning", format!("{:.0} ms", plan.elapsed_ms));
                });
        }
        crate::ui_kit::legend(
            ui,
            &[
                (FORWARD, "forward"),
                (REVERSE, "reverse"),
                (GOAL, "goal"),
                (PARKED, "parked car"),
            ],
        );
        crate::ui_kit::how_it_works(
            ui,
            "parking_help",
            "Hybrid A* searches over (x, y, heading) with short forward and reverse arcs \
             of a car that cannot turn tighter than its steering allows, guided by a \
             grid distance to the goal. Every few expansions it tries to finish with a \
             Reeds-Shepp curve (the shortest forward/reverse path ignoring obstacles); \
             the car is checked as a rectangle, covered by three circles.",
        );
    }

    pub fn scene(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        self.ensure_planned();
        let rect = crate::ui_kit::fit_rect(ui, (LOT_H / LOT_W) as f32, 34.0);
        let response = ui.allocate_rect(rect, Sense::click_and_drag());
        self.handle_pointer(rect, &response);

        // Drive along the plan at a steady speed.
        if let Some(plan) = &self.plan {
            if self.driven < plan.length() {
                let now = ctx.input(|i| i.time);
                let dt = self
                    .last_time
                    .map_or(0.0, |last| (now - last).clamp(0.0, 0.1));
                self.last_time = Some(now);
                self.driven = (self.driven + DRIVE_SPEED * dt).min(plan.length());
                ctx.request_repaint();
            }
        }

        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, ASPHALT);
        // Slot lines of the perpendicular row and the curb.
        for slot in 0..=10 {
            let x = 1.0 + slot as f64 * 2.9;
            painter.line_segment(
                [
                    to_screen(rect, [x, LOT_H]),
                    to_screen(rect, [x, LOT_H - 5.4]),
                ],
                Stroke::new(1.0_f32, LINE),
            );
        }
        painter.line_segment(
            [to_screen(rect, [0.0, 2.6]), to_screen(rect, [LOT_W, 2.6])],
            Stroke::new(1.0_f32, LINE),
        );
        for &[x, y, w, h] in &self.cars {
            painter.rect_filled(
                Rect::from_two_pos(to_screen(rect, [x, y]), to_screen(rect, [x + w, y + h])),
                3.0,
                PARKED,
            );
        }

        // The path, colored by gear.
        if let Some(plan) = &self.plan {
            for (i, w) in plan.poses.windows(2).enumerate() {
                let color = if plan.directions[i] < 0 {
                    REVERSE
                } else {
                    FORWARD
                };
                painter.line_segment(
                    [
                        to_screen(rect, [w[0][0], w[0][1]]),
                        to_screen(rect, [w[1][0], w[1][1]]),
                    ],
                    Stroke::new(2.0_f32, color.gamma_multiply(0.8)),
                );
            }
        }

        let goal = self.dragging.unwrap_or(self.goal);
        draw_car(&painter, rect, goal, None, GOAL);
        draw_car(&painter, rect, self.current_pose(), Some(EGO), EGO);

        let status = if let Some(drag) = self.dragging {
            format!("Goal heading {:.0}°: release to plan", drag[2].to_degrees())
        } else if let Some(failure) = &self.failure {
            format!("No path: {failure}")
        } else if let Some(plan) = &self.plan {
            let gear = plan
                .directions
                .get(
                    plan.distance
                        .partition_point(|&d| d <= self.driven)
                        .saturating_sub(1),
                )
                .copied()
                .unwrap_or(1);
            if self.driven < plan.length() {
                format!(
                    "{}  ·  {:.1} / {:.1} m",
                    if gear < 0 { "Reversing" } else { "Driving" },
                    self.driven,
                    plan.length()
                )
            } else {
                format!(
                    "Parked  ·  {:.1} m, {} gear changes",
                    plan.length(),
                    plan.switches
                )
            }
        } else {
            String::new()
        };
        ui.label(status);
    }
}

/// A car outline (filled when `fill` is set) with a mark at its front.
fn draw_car(
    painter: &egui::Painter,
    rect: Rect,
    pose: Pose,
    fill: Option<Color32>,
    stroke: Color32,
) {
    let corners: Vec<Pos2> = car_corners(pose)
        .iter()
        .map(|c| to_screen(rect, *c))
        .collect();
    painter.add(Shape::convex_polygon(
        corners.clone(),
        fill.map_or(Color32::TRANSPARENT, |c| c.gamma_multiply(0.55)),
        Stroke::new(2.0_f32, stroke),
    ));
    // Windshield line across the front third.
    let (s, c) = pose[2].sin_cos();
    let along = CAR.front - 1.2;
    let half = CAR.width / 2.0 - 0.15;
    let a = [
        pose[0] + along * c + half * s,
        pose[1] + along * s - half * c,
    ];
    let b = [
        pose[0] + along * c - half * s,
        pose[1] + along * s + half * c,
    ];
    painter.line_segment(
        [to_screen(rect, a), to_screen(rect, b)],
        Stroke::new(2.0_f32, stroke),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Whether the car at `pose` overlaps a parked car or leaves the lot.
    fn collides(pose: Pose, cars: &[[f64; 4]]) -> bool {
        // Sample the car's outline and inside.
        let (s, c) = pose[2].sin_cos();
        (0..=12).any(|i| {
            (0..=4).any(|j| {
                let along = -CAR.rear + (CAR.front + CAR.rear) * i as f64 / 12.0;
                let side = -CAR.width / 2.0 + CAR.width * j as f64 / 4.0;
                let p = [
                    pose[0] + along * c - side * s,
                    pose[1] + along * s + side * c,
                ];
                !(0.0..=LOT_W).contains(&p[0])
                    || !(0.0..=LOT_H).contains(&p[1])
                    || cars
                        .iter()
                        .any(|&[x, y, w, h]| p[0] > x && p[0] < x + w && p[1] > y && p[1] < y + h)
            })
        })
    }

    fn check_plan(demo: &ParkingDemo, name: &str, needs_reverse: bool) {
        let plan = demo
            .plan
            .as_ref()
            .unwrap_or_else(|| panic!("{name}: {:?}", demo.failure));
        let end = *plan.poses.last().unwrap();
        assert!(
            (end[0] - demo.goal[0]).hypot(end[1] - demo.goal[1]) < 0.6,
            "{name}: ends at {end:?}, goal {:?}",
            demo.goal
        );
        assert!(
            wrap(end[2] - demo.goal[2]).abs() < 0.15,
            "{name}: heading {end:?}"
        );
        for pose in &plan.poses {
            assert!(
                !collides(*pose, &demo.cars),
                "{name}: collision at {pose:?}"
            );
        }
        if needs_reverse {
            assert!(
                plan.directions.iter().any(|&d| d < 0),
                "{name}: never reverses"
            );
        }
    }

    #[test]
    fn presets_park_without_touching_anything() {
        let mut demo = ParkingDemo::default();
        demo.ensure_planned();
        check_plan(&demo, "parallel", true);
        demo.preset_back_in();
        check_plan(&demo, "back in", true);
        demo.preset_turn();
        check_plan(&demo, "turn around", true);
    }

    #[test]
    fn plans_do_not_change_gear_more_than_needed() {
        let mut demo = ParkingDemo::default();
        demo.ensure_planned();
        let switches = |demo: &ParkingDemo| demo.plan.as_ref().expect("plan").switches;
        assert!(switches(&demo) <= 3, "parallel: {}", switches(&demo));
        demo.preset_back_in();
        assert!(switches(&demo) <= 2, "back in: {}", switches(&demo));
        demo.preset_turn();
        assert!(switches(&demo) <= 2, "turn around: {}", switches(&demo));
        // A straight drive down the street never reverses.
        demo.plan = None;
        demo.car = [3.0, 8.0, 0.0];
        demo.set_goal([25.0, 8.0, 0.0]);
        assert_eq!(switches(&demo), 0);
    }

    #[test]
    fn driving_ends_at_the_goal_and_the_next_plan_starts_there() {
        let mut demo = ParkingDemo::default();
        assert!(demo.plan.is_none(), "planned before the tab was shown");
        demo.ensure_planned();
        let length = demo.plan.as_ref().unwrap().length();
        demo.driven = length;
        let parked = demo.current_pose();
        demo.set_goal([20.0, 9.0, 0.0]);
        let start = demo.plan.as_ref().expect("plan").poses[0];
        assert!((start[0] - parked[0]).hypot(start[1] - parked[1]) < 1e-6);

        // A new goal mid-drive starts from a path sample, which is known to
        // be free, so planning from there never fails on the start pose.
        let plan = demo.plan.clone().expect("plan");
        for k in 1..20 {
            demo.plan = Some(plan.clone());
            demo.driven = plan.length() * k as f64 / 20.0 + 0.013;
            demo.set_goal([25.0, 9.0, 0.0]);
            assert!(
                !demo.failure.as_deref().unwrap_or("").contains("start"),
                "stuck at {k}: {:?}",
                demo.failure
            );
        }
    }

    #[test]
    fn impossible_goals_fail_quickly_with_a_reason() {
        let mut demo = ParkingDemo::default();
        let timer = std::time::Instant::now();
        // On top of a parked car.
        demo.set_goal([2.5, 1.4, 0.0]);
        assert!(demo.plan.is_none());
        assert!(demo.failure.as_deref().unwrap_or("").contains("collides"));
        assert!(timer.elapsed().as_secs_f64() < 5.0);
    }

    #[test]
    fn share_query_round_trips() {
        let demo = ParkingDemo {
            car: [6.0, 9.0, 0.5],
            goal: [20.0, 8.0, 3.0],
            ..ParkingDemo::default()
        };
        let query = demo.share_query();
        let mut restored = ParkingDemo::default();
        restored.apply_share_query(&query);
        restored.ensure_planned();
        assert!((restored.car[0] - 6.0).abs() < 1e-9);
        assert!((restored.goal[0] - 20.0).abs() < 1e-9);
        assert!((restored.goal[2] - 3.0).abs() < 0.01);
        // Other tabs and junk leave it alone.
        let mut untouched = ParkingDemo::default();
        untouched.apply_share_query("tab=grid&goal=1,1,0");
        untouched.apply_share_query("tab=parking&goal=99,1,0&car=a,b,c");
        assert_eq!(untouched.car, ParkingDemo::default().car);
    }
}
