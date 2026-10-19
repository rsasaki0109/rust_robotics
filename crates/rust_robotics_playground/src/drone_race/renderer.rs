//! Small perspective renderer using the playground's existing egui painter.
//! Near-plane clipping happens before projection; surfaces are depth sorted.

use egui::{Color32, FontId, Pos2, Rect, Stroke};
use rust_robotics_control::QuadrotorState;

use super::flight::{add, cross, dot, rotate, scale, sub, unit, Flight};

pub const CYAN: Color32 = Color32::from_rgb(82, 232, 226);
pub const GOLD: Color32 = Color32::from_rgb(255, 193, 100);
pub const GHOST: Color32 = Color32::from_rgb(199, 155, 255);
const BG: Color32 = Color32::from_rgb(10, 15, 32);
const NEAR: f64 = 0.15;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CameraMode {
    #[default]
    Chase,
    Pilot,
    Course,
}

impl CameraMode {
    pub const ALL: [Self; 3] = [Self::Chase, Self::Pilot, Self::Course];

    pub fn slug(self) -> &'static str {
        match self {
            Self::Chase => "chase",
            Self::Pilot => "pilot",
            Self::Course => "course",
        }
    }

    pub fn title(self) -> &'static str {
        match self {
            Self::Chase => "Chase",
            Self::Pilot => "Pilot",
            Self::Course => "Course",
        }
    }

    pub fn from_slug(value: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|mode| mode.slug() == value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Orbit {
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
}

impl Default for Orbit {
    fn default() -> Self {
        Self {
            yaw: 235.0,
            pitch: 20.0,
            distance: 5.0,
        }
    }
}

struct Camera {
    eye: [f64; 3],
    right: [f64; 3],
    up: [f64; 3],
    forward: [f64; 3],
    rect: Rect,
}

impl Camera {
    fn orbit(orbit: Orbit, target: [f64; 3], rect: Rect) -> Self {
        let yaw = f64::from(orbit.yaw).to_radians();
        let pitch = f64::from(orbit.pitch).to_radians();
        let direction = [
            yaw.cos() * pitch.cos(),
            yaw.sin() * pitch.cos(),
            pitch.sin(),
        ];
        Self::new(
            add(target, scale(direction, f64::from(orbit.distance))),
            target,
            [0.0, 0.0, 1.0],
            rect,
        )
    }

    fn course_preview(flight: &Flight, rect: Rect) -> Self {
        let mut points = vec![[0.0, 0.0, 2.0]];
        for gate in &flight.lap.gates {
            let normal = gate.normal();
            let up = unit(sub([0.0, 0.0, 1.0], scale(normal, normal[2])));
            let right = unit(cross(normal, up));
            for side in [-1.0, 1.0] {
                for height in [-1.0, 1.0] {
                    points.push(add(
                        gate.center(),
                        add(
                            scale(right, side * (gate.half_width() + 0.12)),
                            scale(up, height * (gate.half_height() + 0.12)),
                        ),
                    ));
                }
            }
            points.push([gate.center()[0], gate.center()[1], 0.0]);
        }
        let mut low = [f64::INFINITY; 3];
        let mut high = [f64::NEG_INFINITY; 3];
        for point in &points {
            for i in 0..3 {
                low[i] = low[i].min(point[i]);
                high[i] = high[i].max(point[i]);
            }
        }
        let target = scale(add(low, high), 0.5);
        let direction = unit([-0.65, -1.0, 0.6]);
        let inner = rect.shrink2(egui::vec2(rect.width() * 0.08, rect.height() * 0.1));
        let mut distance = 80.0_f64;
        let mut best = distance;
        for _ in 0..40 {
            let camera = Self::new(
                add(target, scale(direction, distance)),
                target,
                [0.0, 0.0, 1.0],
                rect,
            );
            if !points
                .iter()
                .all(|p| camera.project(*p).is_some_and(|p| inner.contains(p)))
            {
                break;
            }
            best = distance;
            distance *= 0.9;
        }
        Self::new(
            add(target, scale(direction, best)),
            target,
            [0.0, 0.0, 1.0],
            rect,
        )
    }

    fn new(eye: [f64; 3], target: [f64; 3], up: [f64; 3], rect: Rect) -> Self {
        let forward = unit(sub(target, eye));
        let right = unit(cross(forward, up));
        let up = unit(cross(right, forward));
        Self {
            eye,
            right,
            up,
            forward,
            rect,
        }
    }

    fn for_flight(mode: CameraMode, state: QuadrotorState, rect: Rect) -> Self {
        match mode {
            CameraMode::Course => {
                Self::new([-5.0, -22.0, 15.0], [10.0, 0.0, 2.8], [0.0, 0.0, 1.0], rect)
            }
            CameraMode::Chase => {
                // Both circuits run along +x. Following the smooth lateral
                // velocity avoids a camera snap when the active gate changes.
                let direction = unit([1.0, state.velocity[1] * 0.07, 0.0]);
                let eye = add(
                    sub(state.position, scale(direction, 4.3)),
                    [0.0, -0.35, 2.0],
                );
                Self::new(
                    eye,
                    add(state.position, add(scale(direction, 3.0), [0.0, 0.0, 0.15])),
                    [0.0, 0.0, 1.0],
                    rect,
                )
            }
            CameraMode::Pilot => {
                let eye = add(state.position, rotate(state.attitude, [0.22, 0.0, 0.05]));
                Self::new(
                    eye,
                    add(eye, rotate(state.attitude, [1.0, 0.0, 0.0])),
                    rotate(state.attitude, [0.0, 0.0, 1.0]),
                    rect,
                )
            }
        }
    }

    fn view(&self, p: [f64; 3]) -> [f64; 3] {
        let p = sub(p, self.eye);
        [dot(p, self.right), dot(p, self.up), dot(p, self.forward)]
    }

    fn project_view(&self, p: [f64; 3]) -> Pos2 {
        let focal = f64::from(self.rect.height().min(self.rect.width() * 1.4)) * 0.82;
        Pos2::new(
            self.rect.center().x + (p[0] / p[2] * focal) as f32,
            self.rect.center().y - (p[1] / p[2] * focal) as f32,
        )
    }

    fn project(&self, p: [f64; 3]) -> Option<Pos2> {
        let p = self.view(p);
        (p[2] >= NEAR).then(|| self.project_view(p))
    }
}

/// Clip a convex polygon in camera space to z>=NEAR.
fn clip_near(vertices: &[[f64; 3]]) -> Vec<[f64; 3]> {
    let mut clipped = Vec::new();
    let Some(&last) = vertices.last() else {
        return clipped;
    };
    let mut a = last;
    for &b in vertices {
        let a_in = a[2] >= NEAR;
        let b_in = b[2] >= NEAR;
        if a_in != b_in {
            let t = (NEAR - a[2]) / (b[2] - a[2]);
            let mut point = add(a, scale(sub(b, a), t));
            point[2] = NEAR;
            clipped.push(point);
        }
        if b_in {
            clipped.push(b);
        }
        a = b;
    }
    clipped
}

enum Primitive {
    Face(Vec<Pos2>, Color32),
    Line([Pos2; 2], Stroke),
}

struct Scene<'a> {
    camera: &'a Camera,
    shapes: Vec<(f64, Primitive)>,
}

impl Scene<'_> {
    fn face(&mut self, vertices: &[[f64; 3]], color: Color32) {
        let viewed: Vec<_> = vertices.iter().map(|p| self.camera.view(*p)).collect();
        let clipped = clip_near(&viewed);
        if clipped.len() >= 3 {
            let depth = clipped.iter().map(|p| p[2]).sum::<f64>() / clipped.len() as f64;
            let points = clipped
                .into_iter()
                .map(|p| self.camera.project_view(p))
                .collect();
            self.shapes
                .push((depth, Primitive::Face(points, fog(color, depth))));
        }
    }

    fn line(&mut self, a: [f64; 3], b: [f64; 3], stroke: Stroke) {
        let (mut a, mut b) = (self.camera.view(a), self.camera.view(b));
        if a[2] < NEAR && b[2] < NEAR {
            return;
        }
        if a[2] < NEAR {
            a = add(a, scale(sub(b, a), (NEAR - a[2]) / (b[2] - a[2])));
        } else if b[2] < NEAR {
            b = add(b, scale(sub(a, b), (NEAR - b[2]) / (a[2] - b[2])));
        }
        let depth = (a[2] + b[2]) * 0.5;
        self.shapes.push((
            depth,
            Primitive::Line(
                [self.camera.project_view(a), self.camera.project_view(b)],
                Stroke::new(stroke.width, fog(stroke.color, depth)),
            ),
        ));
    }

    fn glow(&mut self, a: [f64; 3], b: [f64; 3], color: Color32, width: f32) {
        self.line(a, b, Stroke::new(width + 7.0, color.gamma_multiply(0.06)));
        self.line(a, b, Stroke::new(width + 3.0, color.gamma_multiply(0.16)));
        self.line(a, b, Stroke::new(width, color));
    }

    fn cuboid(&mut self, center: [f64; 3], axes: [[f64; 3]; 3], color: Color32) {
        let p: [[f64; 3]; 8] = std::array::from_fn(|i| {
            let mut p = center;
            for (axis, v) in axes.iter().enumerate() {
                p = add(p, scale(*v, if i & (1 << axis) == 0 { -1.0 } else { 1.0 }));
            }
            p
        });
        for (indices, light) in [
            ([0, 1, 3, 2], 0.45),
            ([4, 5, 7, 6], 1.0),
            ([0, 1, 5, 4], 0.65),
            ([2, 3, 7, 6], 0.8),
            ([0, 2, 6, 4], 0.55),
            ([1, 3, 7, 5], 0.85),
        ] {
            self.face(&indices.map(|i| p[i]), color.gamma_multiply(light));
        }
    }

    fn disc(&mut self, center: [f64; 3], x: [f64; 3], y: [f64; 3], color: Color32) {
        let vertices: Vec<_> = (0..24)
            .map(|i| {
                let theta = i as f64 * std::f64::consts::TAU / 24.0;
                add(center, add(scale(x, theta.cos()), scale(y, theta.sin())))
            })
            .collect();
        self.face(&vertices, color);
    }

    fn paint(mut self, painter: &egui::Painter) {
        self.shapes.sort_by(|a, b| b.0.total_cmp(&a.0));
        for (_, primitive) in self.shapes {
            match primitive {
                Primitive::Face(vertices, color) => {
                    painter.add(egui::Shape::convex_polygon(vertices, color, Stroke::NONE));
                }
                Primitive::Line(points, stroke) => {
                    painter.line_segment(points, stroke);
                }
            }
        }
    }
}

fn fog(color: Color32, depth: f64) -> Color32 {
    let amount = ((depth - 5.0) / 45.0).clamp(0.0, 0.8) as f32;
    Color32::from_rgba_premultiplied(
        (f32::from(color.r()) * (1.0 - amount)
            + f32::from(BG.r()) * amount * f32::from(color.a()) / 255.0) as u8,
        (f32::from(color.g()) * (1.0 - amount)
            + f32::from(BG.g()) * amount * f32::from(color.a()) / 255.0) as u8,
        (f32::from(color.b()) * (1.0 - amount)
            + f32::from(BG.b()) * amount * f32::from(color.a()) / 255.0) as u8,
        color.a(),
    )
}

fn ground(scene: &mut Scene<'_>) {
    for x in (-10..=80).step_by(2) {
        scene.line(
            [x as f64, -40.0, 0.01],
            [x as f64, 40.0, 0.01],
            Stroke::new(0.65_f32, Color32::from_rgb(38, 54, 72)),
        );
    }
    for y in (-40..=40).step_by(2) {
        scene.line(
            [-10.0, y as f64, 0.01],
            [80.0, y as f64, 0.01],
            Stroke::new(0.65_f32, Color32::from_rgb(38, 54, 72)),
        );
    }
    for y in [-4.0, 4.0] {
        scene.glow(
            [-2.0, y, 0.02],
            [23.0, y, 0.02],
            CYAN.gamma_multiply(0.45),
            1.3,
        );
    }
}

fn gate(scene: &mut Scene<'_>, gate: &rust_robotics_control::RacingGatePlane3D, color: Color32) {
    let normal = gate.normal();
    let up = unit(sub([0.0, 0.0, 1.0], scale(normal, normal[2])));
    let right = unit(cross(normal, up));
    let w = gate.half_width();
    let h = gate.half_height();
    for sign in [-1.0, 1.0] {
        let center = add(gate.center(), scale(right, sign * (w + 0.045)));
        scene.cuboid(
            center,
            [
                scale(normal, 0.065),
                scale(right, 0.045),
                scale(up, h + 0.09),
            ],
            color.gamma_multiply(0.65),
        );
        scene.glow(
            add(center, scale(up, -h)),
            add(center, scale(up, h)),
            color,
            2.5,
        );
        let low = add(center, scale(up, -h - 0.09));
        scene.cuboid(
            [low[0], low[1], low[2] * 0.5],
            [
                [0.035, 0.0, 0.0],
                [0.0, 0.035, 0.0],
                [0.0, 0.0, low[2] * 0.5],
            ],
            Color32::from_rgb(54, 63, 83),
        );
        let center = add(gate.center(), scale(up, sign * (h + 0.045)));
        scene.cuboid(
            center,
            [scale(normal, 0.065), scale(right, w), scale(up, 0.045)],
            color.gamma_multiply(0.65),
        );
        scene.glow(
            add(center, scale(right, -w)),
            add(center, scale(right, w)),
            color,
            2.5,
        );
    }
}

fn drone(scene: &mut Scene<'_>, state: QuadrotorState, time: f64) {
    let point = |p| add(state.position, rotate(state.attitude, p));
    let axis = |p| rotate(state.attitude, p);
    scene.disc(
        [state.position[0], state.position[1], 0.025],
        [0.45, 0.0, 0.0],
        [0.0, 0.45, 0.0],
        Color32::from_black_alpha(70),
    );
    scene.cuboid(
        state.position,
        [
            axis([0.19, 0.0, 0.0]),
            axis([0.0, 0.10, 0.0]),
            axis([0.0, 0.0, 0.065]),
        ],
        Color32::from_rgb(209, 220, 230),
    );
    scene.cuboid(
        point([0.03, 0.0, 0.085]),
        [
            axis([0.12, 0.0, 0.0]),
            axis([0.0, 0.075, 0.0]),
            axis([0.0, 0.0, 0.035]),
        ],
        Color32::from_rgb(35, 50, 68),
    );
    for x in [-0.27, 0.27] {
        for y in [-0.27, 0.27] {
            let center = point([x, y, 0.02]);
            let color = if x > 0.0 {
                CYAN
            } else {
                Color32::from_rgb(225, 111, 181)
            };
            scene.line(
                point([0.0, 0.0, 0.0]),
                center,
                Stroke::new(5.0_f32, Color32::from_rgb(124, 139, 155)),
            );
            scene.disc(
                center,
                axis([0.145, 0.0, 0.0]),
                axis([0.0, 0.145, 0.0]),
                color.gamma_multiply(0.18),
            );
            scene.cuboid(
                center,
                [
                    axis([0.035, 0.0, 0.0]),
                    axis([0.0, 0.035, 0.0]),
                    axis([0.0, 0.0, 0.035]),
                ],
                color,
            );
            let theta = time * 115.0 * if x * y > 0.0 { 1.0 } else { -1.0 };
            let blade = axis([theta.cos() * 0.14, theta.sin() * 0.14, 0.0]);
            scene.line(
                sub(center, blade),
                add(center, blade),
                Stroke::new(2.0_f32, color),
            );
        }
    }
    scene.glow(
        point([0.20, -0.07, 0.0]),
        point([0.20, 0.07, 0.0]),
        GOLD,
        3.0,
    );
}

fn checkpoint_glow(scene: &mut Scene<'_>, flight: &Flight, event: super::checkpoint::Checkpoint) {
    let gate = &flight.lap.gates[event.gate - 1];
    // Keep a fading ring from sweeping across the chase camera's near plane.
    if scene.camera.view(gate.center())[2] < 1.0 {
        return;
    }
    let normal = gate.normal();
    let up = unit(sub([0.0, 0.0, 1.0], scale(normal, normal[2])));
    let right = unit(cross(normal, up));
    let progress = event.progress();
    let color = if event.finish { GOLD } else { CYAN }.gamma_multiply(event.fade());
    let point = |angle: f64, expansion: f64| {
        add(
            gate.center(),
            add(
                scale(right, angle.cos() * (gate.half_width() + expansion)),
                scale(up, angle.sin() * (gate.half_height() + expansion)),
            ),
        )
    };
    for i in 0..32 {
        let angle = |i| f64::from(i) * std::f64::consts::TAU / 32.0;
        scene.glow(
            point(angle(i), progress * 0.8),
            point(angle(i + 1), progress * 0.8),
            color,
            1.6,
        );
    }
    // A fixed, bounded set of decorative streaks; it never samples or changes
    // flight physics. A replay or a paused photograph has the same sparks.
    for i in 0..20 {
        let angle = f64::from(i) * std::f64::consts::TAU / 20.0 + event.gate as f64 * 0.137;
        let outward = add(scale(right, angle.cos()), scale(up, angle.sin()));
        let travel = progress * (0.5 + f64::from(i % 4) * 0.2);
        let head = add(point(angle, travel), scale(normal, progress * 0.45));
        let tail = sub(
            head,
            scale(add(outward, scale(normal, 0.3)), 0.10 + progress * 0.12),
        );
        scene.glow(tail, head, color, 1.3);
    }
}

pub struct GhostView<'a> {
    pub flight: &'a Flight,
    pub frame: usize,
    pub state: QuadrotorState,
}

pub struct ViewOptions<'a> {
    pub camera: CameraMode,
    pub orbit: Option<Orbit>,
    pub trail: bool,
    pub prediction: bool,
    pub ghost: Option<GhostView<'a>>,
    pub edit_gate: Option<usize>,
}

fn ghost_drone(scene: &mut Scene<'_>, state: QuadrotorState, time: f64, color: Color32) {
    let point = |p| add(state.position, rotate(state.attitude, p));
    let color = color.gamma_multiply(0.8);
    for x in [-0.27, 0.27] {
        for y in [-0.27, 0.27] {
            scene.glow(point([0.0, 0.0, 0.0]), point([x, y, 0.02]), color, 1.5);
            for i in 0..16 {
                let p = |j| {
                    let angle = j as f64 * std::f64::consts::TAU / 16.0 + time * 4.0;
                    point([x + angle.cos() * 0.15, y + angle.sin() * 0.15, 0.02])
                };
                scene.line(p(i), p(i + 1), Stroke::new(1.0_f32, color));
            }
        }
    }
    scene.glow(
        point([0.2, -0.08, 0.0]),
        point([0.2, 0.08, 0.0]),
        color,
        3.0,
    );
}

pub fn render(
    painter: &egui::Painter,
    rect: Rect,
    flight: &Flight,
    frame: usize,
    state: QuadrotorState,
    time: f64,
    options: &ViewOptions<'_>,
) {
    let mut background = egui::epaint::Mesh::default();
    for (pos, color) in [
        (rect.left_top(), Color32::from_rgb(7, 11, 26)),
        (rect.right_top(), Color32::from_rgb(15, 20, 43)),
        (rect.right_bottom(), Color32::from_rgb(25, 36, 55)),
        (rect.left_bottom(), Color32::from_rgb(11, 22, 38)),
    ] {
        background.colored_vertex(pos, color);
    }
    background.indices = vec![0, 1, 2, 0, 2, 3];
    painter.add(background);
    let current = &flight.frames[frame];
    let camera = if let Some(orbit) = options.orbit {
        Camera::orbit(orbit, state.position, rect)
    } else if options.edit_gate.is_some() {
        Camera::course_preview(flight, rect)
    } else {
        Camera::for_flight(options.camera, state, rect)
    };
    let active_gate = options.edit_gate.unwrap_or(current.passed);
    let mut floor = Scene {
        camera: &camera,
        shapes: Vec::new(),
    };
    floor.face(
        &[
            [-60.0, -70.0, 0.0],
            [200.0, -70.0, 0.0],
            [200.0, 70.0, 0.0],
            [-60.0, 70.0, 0.0],
        ],
        Color32::from_rgb(16, 26, 44),
    );
    floor.paint(painter);
    let mut markings = Scene {
        camera: &camera,
        shapes: Vec::new(),
    };
    ground(&mut markings);
    markings.paint(painter);
    let mut scene = Scene {
        camera: &camera,
        shapes: Vec::new(),
    };

    // Low trackside light pylons give scale without obscuring the flying line.
    for x in (0..=22).step_by(2) {
        for y in [-4.5, 4.5] {
            let height = 0.35 + (x % 3) as f64 * 0.15;
            scene.cuboid(
                [x as f64, y, height * 0.5],
                [[0.10, 0.0, 0.0], [0.0, 0.10, 0.0], [0.0, 0.0, height * 0.5]],
                Color32::from_rgb(47, 56, 78),
            );
            scene.glow(
                [x as f64, y, height],
                [x as f64, y, height + 0.09],
                CYAN,
                2.0,
            );
        }
    }
    for (i, g) in flight.lap.gates.iter().enumerate() {
        // The chase camera trails the physical drone. Hide gates already
        // crossed by the drone so they cannot sweep across that camera's
        // near plane and obscure the next target. The full course view keeps
        // every gate. This only changes rendering.
        if options.camera == CameraMode::Chase && i < current.passed {
            continue;
        }
        let color = if i < current.passed {
            Color32::from_rgb(100, 186, 143)
        } else if i == active_gate {
            GOLD
        } else {
            CYAN
        };
        gate(&mut scene, g, color);
        let c = g.center();
        scene.disc(
            [c[0], c[1], 0.025],
            [0.28, 0.0, 0.0],
            [0.0, 0.28, 0.0],
            color.gamma_multiply(0.18),
        );
    }
    if options.trail {
        let start = frame.saturating_sub(240);
        for (i, pair) in flight.frames[start..=frame].windows(2).enumerate() {
            let strength = 0.25 + 0.7 * (i + 1) as f32 / (frame - start).max(1) as f32;
            scene.glow(
                pair[0].state.position,
                pair[1].state.position,
                CYAN.gamma_multiply(strength),
                2.0,
            );
        }
        scene.glow(current.state.position, state.position, CYAN, 2.0);
    }
    if options.prediction {
        for (i, pair) in current.prediction.windows(2).enumerate() {
            if i % 2 == 0 {
                scene.line(
                    pair[0],
                    pair[1],
                    Stroke::new(1.6_f32, GOLD.gamma_multiply(0.65)),
                );
            }
        }
    }
    if let Some(ghost) = &options.ghost {
        let color = if ghost.flight.is_manual() {
            GOLD
        } else {
            GHOST
        };
        let start = ghost.frame.saturating_sub(240);
        if options.trail {
            for (i, pair) in ghost.flight.frames[start..=ghost.frame]
                .windows(2)
                .enumerate()
            {
                if i % 2 == 0 {
                    scene.line(
                        pair[0].state.position,
                        pair[1].state.position,
                        Stroke::new(1.5_f32, color.gamma_multiply(0.5)),
                    );
                }
            }
        }
        ghost_drone(&mut scene, ghost.state, time, color);
    }
    if options.camera != CameraMode::Pilot {
        drone(&mut scene, state, time);
    }
    if let Some(event) = super::checkpoint::Checkpoint::at(flight, frame, time) {
        checkpoint_glow(&mut scene, flight, event);
    }
    scene.paint(painter);

    if let Some(ghost) = &options.ghost {
        if let Some(pos) = camera.project(add(ghost.state.position, [0.0, 0.0, 0.6])) {
            if rect.shrink(16.0).contains(pos) && time > 0.3 {
                painter.text(
                    pos,
                    egui::Align2::CENTER_BOTTOM,
                    if ghost.flight.is_manual() {
                        "BEST FLIGHT"
                    } else {
                        "AUTOPILOT"
                    },
                    FontId::monospace(10.0),
                    if ghost.flight.is_manual() {
                        GOLD
                    } else {
                        GHOST
                    },
                );
            }
        }
    }

    for (i, gate) in flight.lap.gates.iter().enumerate() {
        if options.camera == CameraMode::Chase && i < current.passed {
            continue;
        }
        let c = gate.center();
        if let Some(pos) = camera.project(add(c, [0.0, 0.0, gate.half_height() + 0.27])) {
            if rect.shrink(16.0).contains(pos) {
                let color = if i == active_gate {
                    GOLD
                } else {
                    CYAN.gamma_multiply(0.75)
                };
                painter.text(
                    pos,
                    egui::Align2::CENTER_CENTER,
                    format!("{:02}", i + 1),
                    FontId::monospace(12.0),
                    color,
                );
            }
        }
    }
    if options.camera == CameraMode::Pilot {
        let c = rect.center();
        painter.line_segment(
            [c + egui::vec2(-12.0, 0.0), c + egui::vec2(-4.0, 0.0)],
            Stroke::new(1.0_f32, Color32::WHITE.gamma_multiply(0.65)),
        );
        painter.line_segment(
            [c + egui::vec2(4.0, 0.0), c + egui::vec2(12.0, 0.0)],
            Stroke::new(1.0_f32, Color32::WHITE.gamma_multiply(0.65)),
        );
        painter.circle_stroke(
            c,
            2.0,
            Stroke::new(1.0_f32, Color32::WHITE.gamma_multiply(0.65)),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn near_plane_clipping_keeps_crossing_faces_and_discards_hidden_ones() {
        let face = [
            [-1.0, -1.0, -1.0],
            [1.0, -1.0, 1.0],
            [1.0, 1.0, 1.0],
            [-1.0, 1.0, -1.0],
        ];
        let clipped = clip_near(&face);
        assert_eq!(clipped.len(), 4);
        assert!(clipped
            .iter()
            .all(|p| p[2] >= NEAR && p.iter().all(|v| v.is_finite())));
        assert_eq!(clipped.iter().filter(|p| p[2] == NEAR).count(), 2);
        assert!(clip_near(&[[-1.0, 0.0, -2.0], [1.0, 0.0, -1.0], [0.0, 1.0, 0.0]]).is_empty());
    }

    #[test]
    fn perspective_centers_the_look_target_and_hides_points_behind_camera() {
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(800.0, 600.0));
        let camera = Camera::new([0.0, -3.0, 2.0], [0.0, 0.0, 2.0], [0.0, 0.0, 1.0], rect);
        assert_eq!(camera.project([0.0, 0.0, 2.0]), Some(rect.center()));
        assert!(camera.project([0.0, -4.0, 2.0]).is_none());
        assert!(camera.project([1.0, 0.0, 2.0]).unwrap().x > rect.center().x);
        assert!(camera.project([0.0, 0.0, 3.0]).unwrap().y < rect.center().y);
    }

    #[test]
    fn editor_camera_keeps_high_and_offset_gates_in_both_preview_shapes() {
        use crate::drone_race::flight::Course;
        let flight = Flight::new(
            Course::CLIMB
                .moved(0, [2.0, -3.0, 6.0])
                .moved(4, [24.0, 3.0, 1.2]),
        );
        for size in [egui::vec2(1256.0, 280.0), egui::vec2(366.0, 220.0)] {
            let rect = Rect::from_min_size(Pos2::ZERO, size);
            let camera = Camera::course_preview(&flight, rect);
            for gate in &flight.lap.gates {
                assert!(rect
                    .shrink(8.0)
                    .contains(camera.project(gate.center()).unwrap()));
                assert!(rect.contains(
                    camera
                        .project([gate.center()[0], gate.center()[1], 0.0])
                        .unwrap()
                ));
            }
        }
    }
}
