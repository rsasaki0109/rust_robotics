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

fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 5);
    let mut maps = std::collections::BTreeMap::<String, Vec<(usize, Point2D)>>::new();
    for (i, line) in std::fs::read_to_string(&args[1]).unwrap().lines().skip(1).enumerate() {
        let f: Vec<_> = line.split(',').collect();
        maps.entry(f[0].to_owned()).or_default().push((i + 1, Point2D::new(f[1].parse().unwrap(), f[2].parse().unwrap())));
    }
    let mut trace = std::io::BufWriter::new(std::io::stdout().lock());
    let mut witnesses = std::io::BufWriter::new(std::fs::File::create(&args[3]).unwrap());
    let mut paths = std::io::BufWriter::new(std::fs::File::create(&args[4]).unwrap());
    writeln!(trace, "id,recovery_clearance,recovery_clearance_9,recovery_bounds,margin,release_margin,release_margin_9,actual_distance,mean_yaw_distance,straight_distance,start_clearance,start_wall_clearance,line_end_x,line_end_y,heading_delta,nearest_obstacle_ids,stop_lo,stop_hi,terminal_release_lo,terminal_release_hi,release_lo,release_hi").unwrap();
    writeln!(witnesses, "id,obstacle_id,obstacle_x,obstacle_y,t,closest_x,closest_y,distance").unwrap();
    writeln!(paths, "id,variant,step,x,y").unwrap();
    let input = std::io::BufReader::new(std::fs::File::open(&args[2]).unwrap());
    for line in input.lines().skip(1) {
        let line = line.unwrap();
        let f: Vec<_> = line.split('\t').collect();
        assert_eq!(f.len(), 18);
        let v: Vec<f64> = f[2..9].iter().map(|x| x.parse().unwrap()).collect();
        assert!(v.iter().all(|x| x.is_finite()));
        let (margin, release_margin, _) = margins(f[10].parse().unwrap(), f[11].parse().unwrap(), v[6], f[9].parse().unwrap(), f[12].parse().unwrap());
        assert_eq!(margin, v[5], "Recorded margin differs at {}", f[0]);
        let entries = maps.get(f[1]).unwrap();
        let obstacles: Vec<_> = entries.iter().map(|(_, p)| *p).collect();
        let (clearance, bounds) = geometry(v[0], v[1], v[2], margin, &obstacles);
        let start = Point2D::new(v[0], v[1]);
        let end = Point2D::new(v[0] + 0.6 * v[2].cos(), v[1] + 0.6 * v[2].sin());
        let mut ids = Vec::new();
        for (id, point) in entries {
            if point_segment_distance(*point, start, end) == clearance {
                ids.push(id.to_string());
                let (t, closest) = projection(*point, start, end);
                assert_eq!(point.distance(&closest), clearance);
                writeln!(witnesses, "{},{},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e}", f[0], id, point.x, point.y, t, closest.x, closest.y, clearance).unwrap();
            }
        }
        assert!(!ids.is_empty());
        let start_clearance = obstacles.iter().map(|p| p.distance(&start)).fold(f64::INFINITY, f64::min);
        let start_wall = start.x.min(start.y).min(50.0 - start.x).min(50.0 - start.y);
        let mut distances = Vec::new();
        for (name, yaw, steering) in [("raw_curve", v[3], v[4]), ("mean_curve", v[2], v[4]), ("mean_straight", v[2], 0.0)] {
            let state = || VehicleState::new(v[0], v[1], yaw, 0.0, WHEELBASE);
            distances.push(speed_free_distance(state(), steering, margin, &obstacles));
            if f[17].parse::<bool>().unwrap() {
                for step in 0..=40 {
                    let point = at_step(state(), steering, step);
                    writeln!(paths, "{},{},{},{:.17e},{:.17e}", f[0], name, step, point.x, point.y).unwrap();
                }
            }
        }
        let stop_lo: f64 = f[13].parse().unwrap();
        let stop_hi: f64 = f[14].parse().unwrap();
        let term_lo: f64 = f[15].parse().unwrap();
        let term_hi: f64 = f[16].parse().unwrap();
        assert!([stop_lo, stop_hi, term_lo, term_hi].iter().all(|x| x.is_finite()));
        assert!(stop_lo <= stop_hi && term_lo <= term_hi);
        let term_release_lo = aligned_release_preview(f64::INFINITY, term_lo);
        let term_release_hi = aligned_release_preview(f64::INFINITY, term_hi);
        let release_lo = aligned_release_preview(stop_lo, term_lo);
        let release_hi = aligned_release_preview(stop_hi, term_hi);
        let delta = v[3] - v[2];
        let fields = vec![f[0].to_owned(), format!("{:.17e}", clearance), format!("{:.9}", clearance), bounds.to_string(),
            format!("{:.17e}", margin), format!("{:.17e}", release_margin), format!("{:.9}", release_margin),
            format!("{:.17e}", distances[0]), format!("{:.17e}", distances[1]), format!("{:.17e}", distances[2]),
            format!("{:.17e}", start_clearance), format!("{:.17e}", start_wall), format!("{:.17e}", end.x), format!("{:.17e}", end.y),
            format!("{:.17e}", delta.sin().atan2(delta.cos())), ids.join(";"),
            format!("{:.17e}", aligned_release_preview(stop_lo, f64::INFINITY)), format!("{:.17e}", aligned_release_preview(stop_hi, f64::INFINITY)),
            format!("{:.17e}", term_release_lo), format!("{:.17e}", term_release_hi), format!("{:.17e}", release_lo), format!("{:.17e}", release_hi)];
        writeln!(trace, "{}", fields.join(",")).unwrap();
    }
}
