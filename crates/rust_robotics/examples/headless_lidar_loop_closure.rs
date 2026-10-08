//! Headless loop-closure demo: scan-to-map odometry with and without the
//! pose-graph back end on a deterministic corridor loop.
//!
//! The robot drives slightly more than one lap of a 3 m wide rectangular
//! corridor (about 85 m per lap) with pillars along the walls. Wheel odometry
//! over-reports distance by 3 % and drifts 1 deg per meter in yaw; the 2D
//! LiDAR has 8 m range and 2 cm range noise. The top corridor has no pillars,
//! so along it the scan-to-map front end cannot observe forward motion and
//! inherits the odometry scale error; revisiting the start closes the loop.

use nalgebra::Vector2;
use rand::{rngs::StdRng, SeedableRng};
use rand_distr::{Distribution, Normal};
use rust_robotics::core::Pose2D;
use rust_robotics::slam::lidar_graph_slam::{LidarGraphSlam, LidarGraphSlamConfig};
use rust_robotics::slam::scan_to_map::{
    compose_pose, ranges_to_points, ray_cast_ranges, relative_pose, LineSegment,
};

const STEP: f64 = 0.2;
const CORNER_RADIUS: f64 = 1.5;
const ODOM_SCALE: f64 = 1.03;
const YAW_DRIFT_PER_METER: f64 = 1.0_f64 * std::f64::consts::PI / 180.0;
const RANGE_NOISE: f64 = 0.02;
const BEAMS: usize = 180;
const MAX_RANGE: f64 = 8.0;
const EXTRA_DISTANCE: f64 = 12.0;

type DemoResult<T> = Result<T, Box<dyn std::error::Error>>;

fn rectangle(min: (f64, f64), max: (f64, f64)) -> Vec<LineSegment> {
    let corners = [
        Vector2::new(min.0, min.1),
        Vector2::new(max.0, min.1),
        Vector2::new(max.0, max.1),
        Vector2::new(min.0, max.1),
    ];
    (0..4)
        .map(|i| LineSegment::new(corners[i], corners[(i + 1) % 4]))
        .collect()
}

/// Outer walls 30 × 20 m, inner block 24 × 14 m, pillars at irregular spacing
/// except along the top corridor.
fn corridor_world() -> Vec<LineSegment> {
    let mut walls = rectangle((-15.0, -10.0), (15.0, 10.0));
    walls.extend(rectangle((-12.0, -7.0), (12.0, 7.0)));
    let pillar = |x: f64, y: f64| rectangle((x - 0.2, y - 0.2), (x + 0.2, y + 0.2));
    for x in [-9.3, -4.1, 1.2, 6.7, 10.4] {
        walls.extend(pillar(x, -9.8));
    }
    for x in [-7.6, -1.9, 3.8, 8.9] {
        walls.extend(pillar(x, -7.2));
    }
    // The top corridor is left featureless: in its middle the LiDAR sees two
    // parallel walls only, so along-corridor motion is unobservable and the
    // odometry scale error leaks into the scan-to-map front end.
    for y in [-4.4, 1.7, 6.2] {
        walls.extend(pillar(-14.8, y));
        walls.extend(pillar(14.8, -y + 0.9));
    }
    for y in [-2.8, 3.9] {
        walls.extend(pillar(-12.2, y));
        walls.extend(pillar(12.2, -y));
    }
    walls
}

/// Body-frame step deltas of the corridor centerline loop (rounded rectangle).
fn centerline_deltas(total_distance: f64) -> Vec<Pose2D> {
    let quarter = std::f64::consts::FRAC_PI_2 * CORNER_RADIUS;
    let half_lap = [
        (27.0 - 2.0 * CORNER_RADIUS, 0.0),
        (quarter, 1.0 / CORNER_RADIUS),
        (17.0 - 2.0 * CORNER_RADIUS, 0.0),
        (quarter, 1.0 / CORNER_RADIUS),
    ];
    let mut deltas = Vec::new();
    let mut travelled = 0.0;
    'outer: loop {
        // `half_lap` covers half the rectangle; the other half mirrors it.
        for (length, curvature) in half_lap.iter().chain(&half_lap) {
            let steps = (length / STEP).round() as usize;
            let step = length / steps as f64;
            for _ in 0..steps {
                deltas.push(Pose2D::new(step, 0.0, step * curvature));
                travelled += step;
                if travelled >= total_distance {
                    break 'outer;
                }
            }
        }
    }
    deltas
}

fn rmse(estimates: &[Pose2D], truth: &[Pose2D]) -> f64 {
    let sum: f64 = estimates
        .iter()
        .zip(truth)
        .map(|(estimate, truth)| (estimate.x - truth.x).powi(2) + (estimate.y - truth.y).powi(2))
        .sum();
    (sum / truth.len().max(1) as f64).sqrt()
}

fn main() -> DemoResult<()> {
    let walls = corridor_world();
    let lap_length =
        2.0 * (27.0 + 17.0) - 8.0 * CORNER_RADIUS + 2.0 * std::f64::consts::PI * CORNER_RADIUS;
    let deltas = centerline_deltas(lap_length + EXTRA_DISTANCE);

    let mut rng = StdRng::seed_from_u64(11);
    let range_noise = Normal::new(0.0, RANGE_NOISE)?;
    let odom_noise = Normal::new(0.0, 0.003)?;
    let scan = |pose: Pose2D, rng: &mut StdRng| {
        let ranges: Vec<f64> = ray_cast_ranges(pose, &walls, BEAMS, MAX_RANGE)
            .into_iter()
            .map(|range| range + range_noise.sample(rng))
            .collect();
        ranges_to_points(&ranges)
    };

    let start = Pose2D::new(-12.0, -8.5, 0.0);
    let mut truth = start;
    let mut odometry = start;
    let mut slam = LidarGraphSlam::new(LidarGraphSlamConfig::default(), start);
    slam.update(Pose2D::origin(), &scan(truth, &mut rng));

    let mut node_truth = vec![truth];
    let mut node_odometry = vec![odometry];
    let mut first_closure_node = None;
    for true_delta in &deltas {
        truth = compose_pose(truth, *true_delta);
        let odom_delta = Pose2D::new(
            true_delta.x * ODOM_SCALE + odom_noise.sample(&mut rng),
            odom_noise.sample(&mut rng),
            true_delta.yaw + YAW_DRIFT_PER_METER * true_delta.x + odom_noise.sample(&mut rng),
        );
        odometry = compose_pose(odometry, odom_delta);
        let update = slam.update(odom_delta, &scan(truth, &mut rng));
        if update.new_node.is_some() {
            node_truth.push(truth);
            node_odometry.push(odometry);
        }
        if let (None, Some(closure)) = (first_closure_node, update.loop_closure) {
            first_closure_node = Some(closure.to);
        }
    }

    // Fold in the loop edges that were consistent with the graph.
    slam.optimize();
    let front_end_nodes = slam.node_front_end_poses();
    let optimized_nodes = slam.node_poses();
    let final_error = |pose: Pose2D| {
        let delta = relative_pose(truth, pose);
        (delta.x.hypot(delta.y), delta.yaw.abs().to_degrees())
    };
    let (odom_final, odom_yaw) = final_error(odometry);
    let (front_final, front_yaw) = final_error(slam.front_end_pose());
    let (slam_final, slam_yaw) = final_error(slam.pose());

    println!(
        "LiDAR loop closure: {:.1} m driven ({:.1} m lap), {} nodes, {} loop closures \
         (first at node {})",
        deltas.len() as f64 * STEP,
        lap_length,
        optimized_nodes.len(),
        slam.loop_closures().len(),
        first_closure_node.map_or("-".to_string(), |node| node.to_string()),
    );
    println!(
        "{:<22} {:>14} {:>12} {:>14}",
        "estimator", "node_rmse_m", "final_xy_m", "final_yaw_deg"
    );
    let odom_rmse = rmse(&node_odometry, &node_truth);
    let front_rmse = rmse(&front_end_nodes, &node_truth);
    let slam_rmse = rmse(&optimized_nodes, &node_truth);
    for (name, node_rmse, final_xy, final_yaw) in [
        ("raw_odometry", odom_rmse, odom_final, odom_yaw),
        ("scan_to_map", front_rmse, front_final, front_yaw),
        ("scan_to_map+loop", slam_rmse, slam_final, slam_yaw),
    ] {
        println!("{name:<22} {node_rmse:>14.4} {final_xy:>12.4} {final_yaw:>14.3}");
    }

    if slam.loop_closures().is_empty() {
        return Err("no loop closure was accepted".into());
    }
    if slam_rmse >= front_rmse || front_rmse >= odom_rmse {
        return Err("expected raw odometry > scan-to-map > scan-to-map+loop node RMSE".into());
    }
    if slam_final >= front_final {
        return Err("loop closure did not reduce the final position error".into());
    }
    Ok(())
}
