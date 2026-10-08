//! Headless comparison of raw odometry, seeded scan-to-scan matching, and
//! scan-to-map matching on a deterministic biased-odometry scenario.
//!
//! The robot drives two laps of a circle inside a room. Wheel odometry
//! over-reports distance by 3 % and drifts 1 deg per meter in yaw; the 2D
//! LiDAR has 1 cm range noise. All three estimators consume identical inputs.

use nalgebra::Vector2;
use rand::{rngs::StdRng, SeedableRng};
use rand_distr::{Distribution, Normal};
use rust_robotics::core::Pose2D;
use rust_robotics::slam::scan_to_map::{
    compose_pose, ranges_to_points, ray_cast_ranges, relative_pose, LineSegment, ScanToMapConfig,
    ScanToMapMatcher,
};

const STEPS: usize = 280;
const STEP_LENGTH: f64 = 0.1;
const TURN_RADIUS: f64 = 2.2;
const ODOM_SCALE: f64 = 1.03;
const YAW_DRIFT_PER_METER: f64 = 1.0_f64 * std::f64::consts::PI / 180.0;
const RANGE_NOISE: f64 = 0.01;
const BEAMS: usize = 240;
const MAX_RANGE: f64 = 15.0;

type DemoResult<T> = Result<T, Box<dyn std::error::Error>>;

#[derive(Default)]
struct Stats {
    squared_position_error: f64,
    final_position_error: f64,
    final_yaw_error: f64,
    accepted: usize,
}

impl Stats {
    fn record(&mut self, estimate: Pose2D, truth: Pose2D) {
        let delta = relative_pose(truth, estimate);
        let position_error = delta.x.hypot(delta.y);
        self.squared_position_error += position_error * position_error;
        self.final_position_error = position_error;
        self.final_yaw_error = delta.yaw.abs();
    }

    fn rmse(&self) -> f64 {
        (self.squared_position_error / STEPS as f64).sqrt()
    }
}

fn room() -> Vec<LineSegment> {
    let mut walls = polygon(&[(-6.0, -4.0), (6.0, -4.0), (6.0, 4.0), (-6.0, 4.0)]);
    walls.extend(polygon(&[(1.0, 1.0), (2.0, 1.4), (1.6, 2.4), (0.6, 2.0)]));
    walls.push(LineSegment::new(
        Vector2::new(-3.0, -4.0),
        Vector2::new(-3.0, -1.5),
    ));
    walls.push(LineSegment::new(
        Vector2::new(3.5, -4.0),
        Vector2::new(4.5, -2.5),
    ));
    walls
}

fn polygon(corners: &[(f64, f64)]) -> Vec<LineSegment> {
    (0..corners.len())
        .map(|i| {
            let (ax, ay) = corners[i];
            let (bx, by) = corners[(i + 1) % corners.len()];
            LineSegment::new(Vector2::new(ax, ay), Vector2::new(bx, by))
        })
        .collect()
}

fn main() -> DemoResult<()> {
    let walls = room();
    let mut rng = StdRng::seed_from_u64(7);
    let range_noise = Normal::new(0.0, RANGE_NOISE)?;
    let odom_noise = Normal::new(0.0, 0.002)?;

    let start = Pose2D::new(-0.5, -3.0, 0.0);
    let true_delta = Pose2D::new(STEP_LENGTH, 0.0, STEP_LENGTH / TURN_RADIUS);

    let mut truth = start;
    let mut odometry = start;
    let mut scan_to_scan = ScanToMapMatcher::new(ScanToMapConfig::scan_to_scan(), start);
    let mut scan_to_map = ScanToMapMatcher::new(ScanToMapConfig::default(), start);
    let initial_scan = ranges_to_points(&ray_cast_ranges(start, &walls, BEAMS, MAX_RANGE));
    scan_to_scan.update(Pose2D::origin(), &initial_scan);
    scan_to_map.update(Pose2D::origin(), &initial_scan);

    let mut odometry_stats = Stats::default();
    let mut scan_to_scan_stats = Stats::default();
    let mut scan_to_map_stats = Stats::default();
    let mut peak_submap_points = 0;

    for _ in 0..STEPS {
        truth = compose_pose(truth, true_delta);
        let odom_delta = Pose2D::new(
            true_delta.x * ODOM_SCALE + odom_noise.sample(&mut rng),
            odom_noise.sample(&mut rng),
            true_delta.yaw + YAW_DRIFT_PER_METER * STEP_LENGTH + 0.5 * odom_noise.sample(&mut rng),
        );
        let ranges: Vec<f64> = ray_cast_ranges(truth, &walls, BEAMS, MAX_RANGE)
            .into_iter()
            .map(|range| range + range_noise.sample(&mut rng))
            .collect();
        let scan = ranges_to_points(&ranges);

        odometry = compose_pose(odometry, odom_delta);
        odometry_stats.record(odometry, truth);

        let update = scan_to_scan.update(odom_delta, &scan);
        scan_to_scan_stats.accepted += usize::from(update.status.is_accepted());
        scan_to_scan_stats.record(update.corrected_pose, truth);

        let update = scan_to_map.update(odom_delta, &scan);
        scan_to_map_stats.accepted += usize::from(update.status.is_accepted());
        scan_to_map_stats.record(update.corrected_pose, truth);
        peak_submap_points = peak_submap_points.max(update.submap_points);
    }

    println!(
        "Scan-to-map LiDAR odometry: {STEPS} steps, {:.1} m, odom scale {ODOM_SCALE}, \
         yaw drift 1 deg/m, range noise {RANGE_NOISE} m",
        STEPS as f64 * STEP_LENGTH
    );
    println!(
        "{:<14} {:>10} {:>12} {:>14} {:>9}",
        "estimator", "rmse_m", "final_xy_m", "final_yaw_deg", "accepted"
    );
    for (name, stats) in [
        ("raw_odometry", &odometry_stats),
        ("scan_to_scan", &scan_to_scan_stats),
        ("scan_to_map", &scan_to_map_stats),
    ] {
        println!(
            "{:<14} {:>10.4} {:>12.4} {:>14.3} {:>9}",
            name,
            stats.rmse(),
            stats.final_position_error,
            stats.final_yaw_error.to_degrees(),
            stats.accepted
        );
    }
    println!(
        "scan_to_map submap: {} keyframes, peak {} points in radius",
        scan_to_map.keyframe_count(),
        peak_submap_points
    );

    if scan_to_map_stats.rmse() >= odometry_stats.rmse() {
        return Err("scan-to-map did not improve on raw odometry".into());
    }
    if scan_to_map_stats.rmse() > scan_to_scan_stats.rmse() {
        return Err("scan-to-map was worse than scan-to-scan".into());
    }
    if scan_to_map_stats.accepted < STEPS * 9 / 10 {
        return Err("scan-to-map rejected more than 10% of scans".into());
    }
    Ok(())
}
