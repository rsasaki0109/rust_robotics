#![allow(dead_code)]
use std::io::{BufRead, Write};
const RADIUS: f64 = 0.5;
#[derive(Clone, Copy)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}
impl Point2D {
pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
pub fn distance(&self, other: &Point2D) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
}
fn point_segment_distance(p: Point2D, a: Point2D, b: Point2D) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let d2 = dx * dx + dy * dy;
    if d2 < 1e-20 {
        return p.distance(&a);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / d2).clamp(0.0, 1.0);
    p.distance(&Point2D::new(a.x + t * dx, a.y + t * dy))
}
fn geometry(recovery_x: f64, recovery_y: f64, recovery_yaw: f64, margin: f64, obs: &[Point2D]) -> (f64, bool) {
            let preview_end = Point2D::new(
                recovery_x + 0.6 * recovery_yaw.cos(),
                recovery_y + 0.6 * recovery_yaw.sin(),
            );
            // This uses estimated pose and the known map, never the true clearance.
            let recovery_clearance = obs
                .iter()
                .map(|&p| {
                    point_segment_distance(p, Point2D::new(recovery_x, recovery_y), preview_end)
                })
                .fold(f64::INFINITY, f64::min);
            let recovery_margin = margin;
            let recovery_bounds = [recovery_x, recovery_y, preview_end.x, preview_end.y]
                .iter()
                .all(|p| (recovery_margin..=(50.0 - recovery_margin)).contains(p));
(recovery_clearance, recovery_bounds)
}
fn margins(position_sigma: f64, drift_speed: f64, observation_age: f64, precision_active: bool, recovery_guard_buffer: bool) -> (f64, f64, f64) {
            let effective_position_sigma = if precision_active {
                0.03
            } else {
                position_sigma
            };
            let recovery_margin = RADIUS
                + 0.1
                + 2.0 * 2.0_f64.sqrt() * effective_position_sigma
                + drift_speed * observation_age;
            let base_preview_margin = RADIUS
                // Match the recovery/speed-probe buffer in this optional arm.
                // Release keeps its existing additional hysteresis of 0.05.
                + if recovery_guard_buffer { 0.1 } else { 0.0 }
                + 2.0 * 2.0_f64.sqrt() * effective_position_sigma
                + drift_speed * observation_age;
let guard_release_margin = base_preview_margin + 0.05;
(recovery_margin, guard_release_margin, effective_position_sigma)
}


const WHEELBASE: f64 = 0.4;
#[derive(Clone, Copy)]
pub struct VehicleState {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
    pub rear_x: f64,
    pub rear_y: f64,
    pub wheelbase: f64,
}
impl VehicleState {
pub fn new(x: f64, y: f64, yaw: f64, v: f64, wheelbase: f64) -> Self {
        let rear_x = x - (wheelbase / 2.0) * yaw.cos();
        let rear_y = y - (wheelbase / 2.0) * yaw.sin();
        VehicleState {
            x,
            y,
            yaw,
            v,
            rear_x,
            rear_y,
            wheelbase,
        }
    }
}
fn speed_free_distance(
    sensed: VehicleState,
    steering: f64,
    margin: f64,
    obstacles: &[Point2D],
) -> f64 {
    let start = Point2D::new(sensed.x, sensed.y);
    let nearby: Vec<Point2D> = obstacles
        .iter()
        .copied()
        .filter(|p| p.distance(&start) <= 2.0 + margin)
        .collect();
    if [start.x, start.y, 50.0 - start.x, 50.0 - start.y]
        .iter()
        .any(|&d| d <= margin)
        || nearby.iter().any(|p| p.distance(&start) <= margin)
    {
        return 0.0;
    }
    let curvature = steering.tan() / WHEELBASE;
    let mut previous = start;
    for step in 1..=40 {
        let length = step as f64 * 0.05;
        let yaw = sensed.yaw + curvature * length;
        let point = if curvature.abs() < 1e-9 {
            Point2D::new(
                start.x + length * sensed.yaw.cos(),
                start.y + length * sensed.yaw.sin(),
            )
        } else {
            Point2D::new(
                start.x + (yaw.sin() - sensed.yaw.sin()) / curvature,
                start.y - (yaw.cos() - sensed.yaw.cos()) / curvature,
            )
        };
        if [point.x, point.y, 50.0 - point.x, 50.0 - point.y]
            .iter()
            .any(|&d| d <= margin)
            || nearby
                .iter()
                .any(|&p| point_segment_distance(p, previous, point) <= margin)
        {
            return (step - 1) as f64 * 0.05;
        }
        previous = point;
    }
    2.0
}
fn aligned_release_preview(stop: f64, terminal: f64) -> f64 {
    stop.min(terminal - 0.05)
}
fn at_step(sensed: VehicleState, steering: f64, step: usize) -> Point2D {
let start = Point2D::new(sensed.x, sensed.y);
if step == 0 { return start; }
let curvature = steering.tan() / WHEELBASE;
        let length = step as f64 * 0.05;
        let yaw = sensed.yaw + curvature * length;
        let point = if curvature.abs() < 1e-9 {
            Point2D::new(
                start.x + length * sensed.yaw.cos(),
                start.y + length * sensed.yaw.sin(),
            )
        } else {
            Point2D::new(
                start.x + (yaw.sin() - sensed.yaw.sin()) / curvature,
                start.y - (yaw.cos() - sensed.yaw.cos()) / curvature,
            )
        };
point
}
fn projection(p: Point2D, a: Point2D, b: Point2D) -> (f64, Point2D) {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let d2 = dx * dx + dy * dy;
    if d2 < 1e-20 {
        return (0.0, a);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / d2).clamp(0.0, 1.0);
    (t, Point2D::new(a.x + t * dx, a.y + t * dy))
}


use std::collections::VecDeque;
const DT: f64 = 0.02;
impl VehicleState {
pub fn update(&mut self, a: f64, delta: f64, dt: f64) {
        self.x += self.v * self.yaw.cos() * dt;
        self.y += self.v * self.yaw.sin() * dt;
        self.yaw += self.v / self.wheelbase * delta.tan() * dt;
        self.v += a * dt;
        self.rear_x = self.x - (self.wheelbase / 2.0) * self.yaw.cos();
        self.rear_y = self.y - (self.wheelbase / 2.0) * self.yaw.sin();
    }
}
fn mean_pose(samples: &VecDeque<(f64, f64, f64)>) -> (f64, f64, f64) {
    let n = samples.len() as f64;
    (
        samples.iter().map(|p| p.0).sum::<f64>() / n,
        samples.iter().map(|p| p.1).sum::<f64>() / n,
        samples
            .iter()
            .map(|p| p.2.sin())
            .sum::<f64>()
            .atan2(samples.iter().map(|p| p.2.cos()).sum::<f64>()),
    )
}
#[derive(Default)]
struct MotionPoseFilter {
    samples: VecDeque<(f64, f64, f64)>,
    previous_speed: f64,
}
impl MotionPoseFilter {
    fn observe(
        &mut self,
        sensed: VehicleState,
        previous_steering: f64,
        reset: bool,
    ) -> VehicleState {
        if reset {
            self.samples.clear();
        }
        // Transport each old observation through the last applied motion,
        // using measured speed and our steering command, never true pose.
        let distance = self.previous_speed * DT;
        let turn = distance / WHEELBASE * previous_steering.tan();
        for (x, y, yaw) in &mut self.samples {
            *x += distance * yaw.cos();
            *y += distance * yaw.sin();
            *yaw = (*yaw + turn).sin().atan2((*yaw + turn).cos());
        }
        self.samples.push_back((sensed.x, sensed.y, sensed.yaw));
        if self.samples.len() > 25 {
            self.samples.pop_front();
        }
        self.previous_speed = sensed.v;
        let (x, y, yaw) = mean_pose(&self.samples);
        VehicleState::new(x, y, yaw, sensed.v, WHEELBASE)
    }
}
struct ObservationEvent {
    time: f64,
    correction: bool,
    received: bool,
    precision_activation: bool,
}
struct StationaryEpoch {
    time: f64,
    samples: VecDeque<(f64, f64, f64)>,
}
#[derive(Default)]
struct StationaryEpochFusion {
    groups: VecDeque<StationaryEpoch>,
    epoch: Option<f64>,
    previous_speed: f64,
    previous_time: Option<f64>,
}
impl StationaryEpochFusion {
    fn observe(
        &mut self,
        sensed: VehicleState,
        previous_steering: f64,
        event: ObservationEvent,
        age: f64,
    ) -> Option<(f64, f64)> {
        assert!(self.previous_time.map_or(true, |time| event.time > time));
        self.previous_time = Some(event.time);
        if sensed.v.abs() > 0.02 || event.precision_activation {
            self.groups.clear();
        }
        if event.received || event.precision_activation {
            self.epoch = Some(event.time);
        }
        while self
            .groups
            .front()
            .is_some_and(|group| event.time - group.time > 15.0 + 1e-9)
        {
            self.groups.pop_front();
        }
        let distance = self.previous_speed * DT;
        let turn = distance / WHEELBASE * previous_steering.tan();
        for group in &mut self.groups {
            for (x, y, yaw) in &mut group.samples {
                *x += distance * yaw.cos();
                *y += distance * yaw.sin();
                *yaw = (*yaw + turn).sin().atan2((*yaw + turn).cos());
            }
        }
        if sensed.v.abs() <= 0.02 && (0.0..=0.5 + 1e-9).contains(&age) {
            if let Some(epoch) = self.epoch {
                if self.groups.back().map_or(true, |group| group.time != epoch) {
                    self.groups.push_back(StationaryEpoch {
                        time: epoch,
                        samples: VecDeque::new(),
                    });
                    if self.groups.len() > 3 {
                        self.groups.pop_front();
                    }
                }
                let samples = &mut self.groups.back_mut().unwrap().samples;
                samples.push_back((sensed.x, sensed.y, sensed.yaw));
                if samples.len() > 25 {
                    samples.pop_front();
                }
            }
        }
        self.previous_speed = sensed.v;
        if sensed.v.abs() > 0.02
            || age > 1.5 + 1e-9
            || self.groups.len() != 3
            || Some(self.groups.back().unwrap().time) != self.epoch
            || self.groups.iter().take(2).any(|g| g.samples.len() < 10)
        {
            return None;
        }
        let (x, y) = self.groups.iter().fold((0.0, 0.0), |(x, y), group| {
            let (mean_x, mean_y, _) = mean_pose(&group.samples);
            (x + mean_x, y + mean_y)
        });
        Some((x / 3.0, y / 3.0))
    }
}
fn terminal_arc_clearance(sensed: VehicleState, steering: f64, obstacles: &[Point2D]) -> f64 {
    const LENGTH: f64 = 0.6;
    let start = Point2D::new(sensed.x, sensed.y);
    let curvature = steering.tan() / WHEELBASE;
    let at = |length: f64| {
        let half = curvature * length / 2.0;
        let distance = length * if half == 0.0 { 1.0 } else { half.sin() / half };
        Point2D::new(
            start.x + distance * (sensed.yaw + half).cos(),
            start.y + distance * (sensed.yaw + half).sin(),
        )
    };
    let end = at(LENGTH);
    let boundary = |p: Point2D| p.x.min(p.y).min(50.0 - p.x).min(50.0 - p.y);
    let mut clearance = boundary(start).min(boundary(end));
    if curvature == 0.0 {
        return obstacles
            .iter()
            .map(|&p| point_segment_distance(p, start, end))
            .fold(clearance, f64::min);
    }
    // Wall minima may occur inside the arc, where a tangent is cardinal.
    let sign = curvature.signum();
    for heading in [
        0.0,
        std::f64::consts::FRAC_PI_2,
        std::f64::consts::PI,
        3.0 * std::f64::consts::FRAC_PI_2,
    ] {
        let progress = ((heading - sensed.yaw) * sign).rem_euclid(std::f64::consts::TAU);
        if progress <= curvature.abs() * LENGTH {
            clearance = clearance.min(boundary(at(progress / curvature.abs())));
        }
    }
    let (sine, cosine) = sensed.yaw.sin_cos();
    for &point in obstacles {
        // Triangle inequality: a farther point cannot lower the current min.
        if point.distance(&start) - LENGTH >= clearance {
            continue;
        }
        let (dx, dy) = (point.x - start.x, point.y - start.y);
        let (forward, left) = (dx * cosine + dy * sine, -dx * sine + dy * cosine);
        let progress = (curvature * forward).atan2(1.0 - curvature * left) / curvature;
        let distance = if (0.0..=LENGTH).contains(&progress) {
            let norm = (curvature * forward).hypot(1.0 - curvature * left);
            (curvature * (forward * forward + left * left) - 2.0 * left).abs() / (norm + 1.0)
        } else {
            point.distance(&start).min(point.distance(&end))
        };
        clearance = clearance.min(distance);
    }
    clearance
}
fn stopping_preview(
    sensed: VehicleState,
    target: f64,
    steering: f64,
    margin: f64,
    obstacles: &[Point2D],
    recovery_preview: bool,
    terminal_steering_preview: bool,
) -> (f64, f64) {
    let initial = Point2D::new(sensed.x, sensed.y);
    let mut predicted = sensed;
    let mut points = vec![initial];
    let mut length = 0.0;
    for step in 0..200 {
        let acceleration = if step < 10 {
            target - predicted.v
        } else {
            (-4.0 * predicted.v).clamp(-1.0, 0.0)
        };
        predicted.update(acceleration, steering, DT);
        predicted.yaw = predicted.yaw.sin().atan2(predicted.yaw.cos());
        let point = Point2D::new(predicted.x, predicted.y);
        length += points.last().unwrap().distance(&point);
        points.push(point);
        if step >= 10 && predicted.v.abs() <= 0.001 {
            break;
        }
    }
    assert!(predicted.v.abs() <= 0.001);
    let mut clearance = points
        .iter()
        .flat_map(|p| [p.x, p.y, 50.0 - p.x, 50.0 - p.y])
        .fold(f64::INFINITY, f64::min);
    for &obstacle in obstacles {
        let distance = obstacle.distance(&initial);
        if distance > length + margin + 0.05 {
            clearance = clearance.min(distance - length);
        } else {
            for pair in points.windows(2) {
                clearance = clearance.min(point_segment_distance(obstacle, pair[0], pair[1]));
            }
        }
    }
    let recovery_clearance = if recovery_preview && terminal_steering_preview {
        terminal_arc_clearance(predicted, steering, obstacles)
    } else if recovery_preview {
        let start = Point2D::new(predicted.x, predicted.y);
        let end = Point2D::new(
            start.x + 0.6 * predicted.yaw.cos(),
            start.y + 0.6 * predicted.yaw.sin(),
        );
        let boundary = [
            start.x,
            start.y,
            50.0 - start.x,
            50.0 - start.y,
            end.x,
            end.y,
            50.0 - end.x,
            50.0 - end.y,
        ]
        .into_iter()
        .fold(f64::INFINITY, f64::min);
        obstacles
            .iter()
            .map(|&p| point_segment_distance(p, start, end))
            .fold(boundary, f64::min)
    } else {
        0.0
    };
    (clearance, recovery_clearance)
}
fn stopping_motion(
    sensed: VehicleState,
    target: f64,
    steering: f64,
    margin: f64,
    obstacles: &[Point2D],
    recovery_preview: bool,
    terminal_steering_preview: bool,
) -> (VehicleState, Vec<Point2D>, f64) {
    let initial = Point2D::new(sensed.x, sensed.y);
    let mut predicted = sensed;
    let mut points = vec![initial];
    let mut length = 0.0;
    for step in 0..200 {
        let acceleration = if step < 10 {
            target - predicted.v
        } else {
            (-4.0 * predicted.v).clamp(-1.0, 0.0)
        };
        predicted.update(acceleration, steering, DT);
        predicted.yaw = predicted.yaw.sin().atan2(predicted.yaw.cos());
        let point = Point2D::new(predicted.x, predicted.y);
        length += points.last().unwrap().distance(&point);
        points.push(point);
        if step >= 10 && predicted.v.abs() <= 0.001 {
            break;
        }
    }
    assert!(predicted.v.abs() <= 0.001);
    (predicted, points, length)
}

fn preview_margins(position_sigma: f64, drift_speed: f64, observation_age: f64, precision_active: bool, recovery_guard_buffer: bool) -> (f64, f64, f64, f64) {
            let effective_position_sigma = if precision_active {
                0.03
            } else {
                position_sigma
            };
            let recovery_margin = RADIUS
                + 0.1
                + 2.0 * 2.0_f64.sqrt() * effective_position_sigma
                + drift_speed * observation_age;
            let base_preview_margin = RADIUS
                // Match the recovery/speed-probe buffer in this optional arm.
                // Release keeps its existing additional hysteresis of 0.05.
                + if recovery_guard_buffer { 0.1 } else { 0.0 }
                + 2.0 * 2.0_f64.sqrt() * effective_position_sigma
                + drift_speed * observation_age;
let guard_release_margin = base_preview_margin + 0.05;
(recovery_margin, guard_release_margin, effective_position_sigma, base_preview_margin)
}
const SEQUENCE: usize = 0;
const ID: usize = 1;
const CASE_ID: usize = 2;
const SCENARIO: usize = 3;
const STEP: usize = 4;
const TIME_S: usize = 5;
const RAW_X: usize = 6;
const RAW_Y: usize = 7;
const RAW_YAW: usize = 8;
const MEASURED_SPEED: usize = 9;
const OBSERVATION_AGE: usize = 10;
const CORRECTION: usize = 11;
const RECEIVED: usize = 12;
const PRECISION_ACTIVATION: usize = 13;
const APPLIED_STEERING: usize = 14;
const NOMINAL_TARGET: usize = 15;
const NOMINAL_STEERING: usize = 16;
const SPEED_MARGIN: usize = 17;
const MEAN_X: usize = 18;
const MEAN_Y: usize = 19;
const MEAN_YAW: usize = 20;
const PRECISION_ACTIVE: usize = 21;
const FILTER_COUNT: usize = 22;
const FUSION_AVAILABLE: usize = 23;
const FUSION_GROUPS: usize = 24;
const FILTER_X_9: usize = 25;
const FILTER_Y_9: usize = 26;
const PREVIEW_X_9: usize = 27;
const PREVIEW_Y_9: usize = 28;
const PREVIEW_YAW_9: usize = 29;
const POSITION_SIGMA: usize = 30;
const DRIFT_SPEED: usize = 31;
const GUARD_BUFFER: usize = 32;


const Q_ID: usize = 0;
const Q_TRANSITION_ID: usize = 1;
const Q_LAG: usize = 2;
const Q_SCENARIO: usize = 3;
const Q_MEAN_X: usize = 4;
const Q_MEAN_Y: usize = 5;
const Q_MEAN_YAW: usize = 6;
const Q_SPEED_MARGIN: usize = 7;
const Q_OBSERVATION_AGE: usize = 8;
const Q_PRECISION_ACTIVE: usize = 9;
const Q_POSITION_SIGMA: usize = 10;
const Q_DRIFT_SPEED: usize = 11;
const Q_GUARD_BUFFER: usize = 12;
const Q_PREVIEW_X: usize = 13;
const Q_PREVIEW_Y: usize = 14;
const Q_PREVIEW_YAW: usize = 15;
const Q_MEASURED_SPEED: usize = 16;
const Q_NOMINAL_TARGET: usize = 17;
const Q_NOMINAL_STEERING: usize = 18;
const Q_APPLIED_STEERING: usize = 19;
const Q_JOINT_PENDING: usize = 20;
const Q_CORRECTION: usize = 21;
const Q_RECEIVED: usize = 22;
const Q_PRECISION_ACTIVATION: usize = 23;
const Q_COUNT: usize = 24;
fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 3);
    let mut maps = std::collections::BTreeMap::<String, Vec<Point2D>>::new();
    for line in std::fs::read_to_string(&args[1]).unwrap().lines().skip(1) {
        let f: Vec<_> = line.split(',').collect();
        assert_eq!(f.len(), 3);
        maps.entry(f[0].to_owned()).or_default().push(Point2D::new(f[1].parse().unwrap(), f[2].parse().unwrap()));
    }
    let mut input = std::io::BufReader::new(std::fs::File::open(&args[2]).unwrap()).lines();
    assert_eq!(input.next().unwrap().unwrap(), "id	transition_id	lag	scenario	mean_x	mean_y	mean_yaw	speed_margin	observation_age	precision_active	position_sigma	drift_speed	guard_buffer	preview_x	preview_y	preview_yaw	measured_speed	nominal_target	nominal_steering	applied_steering	joint_pending	correction	received	precision_activation	count");
    let mut output = std::io::BufWriter::new(std::io::stdout().lock());
    writeln!(output, "id,transition_id,lag,variant,scenario,valid_now,valid_prefix,carry_x,carry_y,carry_yaw,x,y,yaw,speed,target,steering,recovery_clearance,recovery_clearance_9,recovery_bounds,recovery_margin,preview_margin,preview_margin_9,release_margin,release_margin_9,stop,stop_9,terminal,terminal_9,entry,entry_9,release,release_9,effective_position_sigma_9").unwrap();
    let (mut sequence, mut transition, mut previous_lag) = (0usize, 0usize, 0usize);
    let mut scenario = String::new();
    let mut carry = (0.0, 0.0, 0.0);
    let (mut previous_speed, mut previous_steering) = (0.0, 0.0);
    let mut valid_prefix = true;
    for line in input {
        let line = line.unwrap();
        let f: Vec<_> = line.split('\t').collect();
        assert_eq!(f.len(), 25);
        sequence += 1;
        assert_eq!(f[Q_ID].parse::<usize>().unwrap(), sequence);
        let tid = f[Q_TRANSITION_ID].parse::<usize>().unwrap();
        let lag = f[Q_LAG].parse::<usize>().unwrap();
        assert!(lag <= 50);
        let number = |i: usize| { let value: f64 = f[i].parse().unwrap(); assert!(value.is_finite()); value };
        for column in [0, 1, 2, 4, 5, 6, 7, 8, 10, 11, 13, 14, 15, 16, 17, 18, 19, 24] { number(column); }
        let flag = |i: usize| f[i].parse::<bool>().unwrap();
        let count = f[Q_COUNT].parse::<usize>().unwrap();
        let actual = VehicleState::new(number(Q_PREVIEW_X), number(Q_PREVIEW_Y), number(Q_PREVIEW_YAW), number(Q_MEASURED_SPEED), WHEELBASE);
        let valid_now = !flag(Q_JOINT_PENDING) && !flag(Q_CORRECTION) && !flag(Q_RECEIVED)
            && !flag(Q_PRECISION_ACTIVATION) && actual.v.abs() <= 0.02
            && number(Q_OBSERVATION_AGE) <= 1.5 && count >= 10;
        if lag == 0 {
            assert_eq!(tid, transition + 1);
            transition = tid;
            scenario = f[Q_SCENARIO].to_owned();
            carry = (number(Q_MEAN_X), number(Q_MEAN_Y), number(Q_MEAN_YAW));
            valid_prefix = true;
        } else {
            assert_eq!(tid, transition);
            assert_eq!(lag, previous_lag + 1);
            assert_eq!(scenario, f[Q_SCENARIO]);
            let mut transport = MotionPoseFilter {
                samples: VecDeque::from([carry]), previous_speed,
            };
            transport.observe(actual, previous_steering, false);
            carry = *transport.samples.front().unwrap();
            valid_prefix &= valid_now;
        }
        previous_lag = lag;
        previous_speed = actual.v;
        previous_steering = number(Q_APPLIED_STEERING);
        let (margin, release_margin, sigma, base) = preview_margins(number(Q_POSITION_SIGMA), number(Q_DRIFT_SPEED), number(Q_OBSERVATION_AGE), flag(Q_PRECISION_ACTIVE), flag(Q_GUARD_BUFFER));
        assert_eq!(margin, number(Q_SPEED_MARGIN));
        let obstacles = maps.get(f[Q_SCENARIO]).unwrap();
        let (line, bounds) = geometry(number(Q_MEAN_X), number(Q_MEAN_Y), number(Q_MEAN_YAW), margin, obstacles);
        for variant in ["recorded", "carried_diagnostic_xy", "gated_carried_xy"] {
            if lag == 0 && variant != "recorded" { continue; }
            let mut pose = actual;
            if variant == "carried_diagnostic_xy" || (variant == "gated_carried_xy" && valid_prefix) {
                pose.x = carry.0; pose.y = carry.1;
            }
            assert_eq!((pose.yaw, pose.v, pose.wheelbase, pose.rear_x, pose.rear_y),
                (actual.yaw, actual.v, actual.wheelbase, actual.rear_x, actual.rear_y));
            let target = number(Q_NOMINAL_TARGET);
            let steering = number(Q_NOMINAL_STEERING);
            let (stop, terminal) = stopping_preview(pose, target, steering, base, obstacles, true, true);
            let entry = stop.min(terminal - 0.1);
            let release = aligned_release_preview(stop, terminal);
            let mut values = vec![sequence.to_string(), tid.to_string(), lag.to_string(), variant.to_owned(), scenario.clone(), valid_now.to_string(), valid_prefix.to_string()];
            values.extend([carry.0, carry.1, carry.2, pose.x, pose.y, pose.yaw, pose.v, target, steering].iter().map(|v| format!("{:.17e}", v)));
            values.extend([format!("{:.17e}", line), format!("{:.9}", line), bounds.to_string(),
                format!("{:.17e}", margin), format!("{:.17e}", base), format!("{:.9}", base),
                format!("{:.17e}", release_margin), format!("{:.9}", release_margin),
                format!("{:.17e}", stop), format!("{:.9}", stop), format!("{:.17e}", terminal), format!("{:.9}", terminal),
                format!("{:.17e}", entry), format!("{:.9}", entry), format!("{:.17e}", release), format!("{:.9}", release), format!("{:.9}", sigma)]);
            writeln!(output, "{}", values.join(",")).unwrap();
        }
    }
    assert!(sequence > 0);
}
