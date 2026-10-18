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

fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 3);
    let mut maps = std::collections::BTreeMap::<String, Vec<Point2D>>::new();
    for line in std::fs::read_to_string(&args[1]).unwrap().lines().skip(1) {
        let f: Vec<_> = line.split(',').collect();
        maps.entry(f[0].to_owned()).or_default().push(Point2D::new(f[1].parse().unwrap(), f[2].parse().unwrap()));
    }
    let mut output = std::io::BufWriter::new(std::io::stdout().lock());
    writeln!(output, "id,recovery_clearance,recovery_clearance_9,recovery_bounds,release_margin,release_margin_9,recovery_margin,precision_active,effective_position_sigma_9").unwrap();
    let input = std::io::BufReader::new(std::fs::File::open(&args[2]).unwrap());
    for line in input.lines().skip(1) {
        let line = line.unwrap();
        let f: Vec<_> = line.split('\t').collect();
        assert_eq!(f.len(), 11);
        let values: Vec<f64> = f[2..6].iter().map(|v| v.parse().unwrap()).collect();
        assert!(values.iter().all(|v| v.is_finite()));
        assert!(values[3] > 0.0);
        let precision: bool = f[7].parse().unwrap();
        let (recovery_margin, release_margin, sigma) = margins(f[8].parse().unwrap(), f[9].parse().unwrap(), f[6].parse().unwrap(), precision, f[10].parse().unwrap());
        assert_eq!(recovery_margin, values[3], "Logged recovery margin differs at query {}", f[0]);
        let (clearance, bounds) = geometry(values[0], values[1], values[2], recovery_margin, maps.get(f[1]).unwrap());
        writeln!(output, "{},{:.17e},{:.9},{},{:.17e},{:.9},{:.17e},{},{:.9}", f[0], clearance, clearance, bounds, release_margin, release_margin, recovery_margin, precision, sigma).unwrap();
    }
}
