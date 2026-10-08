//! Run LiDAR graph SLAM on a CARMEN laser log and score it with the
//! relative-pose metric of the Kümmerle et al. SLAM benchmark.
//!
//! ```bash
//! # Real data, e.g. the Intel Research Lab log and its benchmark relations:
//! cargo run --release -p rust_robotics --example carmen_lidar_slam --no-default-features \
//!     --features slam -- intel.clf intel.relations --map intel_map.png
//!
//! # No arguments: a synthetic 180° front-laser log of the corridor loop is
//! # generated, written as CARMEN text, parsed back, and scored (used in CI).
//! cargo run -p rust_robotics --example carmen_lidar_slam --no-default-features --features slam
//! ```
//!
//! Scans are fed to `LidarGraphSlam` whenever the laser moved 0.1 m or turned
//! 0.05 rad since the last processed scan; skipped scans are placed by
//! odometry relative to the last processed one. The scored SLAM trajectory
//! places every scan relative to its pose-graph node, using the node poses
//! after a final optimization.

use std::error::Error;

use nalgebra::Vector2;
use rand::{rngs::StdRng, SeedableRng};
use rand_distr::{Distribution, Normal};
use rust_robotics::core::Pose2D;
use rust_robotics::slam::carmen::{format_flaser, parse_carmen_log, CarmenScan};
use rust_robotics::slam::lidar_graph_slam::{LidarGraphSlam, LidarGraphSlamConfig};
use rust_robotics::slam::lidar_loop_scenario::{
    corridor_loop_deltas, corridor_loop_start, corridor_loop_walls, CorridorLoopConfig,
};
use rust_robotics::slam::scan_to_map::{
    compose_pose, ray_cast_ranges_fov, relative_pose, transform_scan_to_world,
};
use rust_robotics::slam::slam_benchmark::{
    evaluate_relations, format_relations, parse_relations, Relation, RelationErrors,
};

type DemoResult<T> = Result<T, Box<dyn Error>>;

const MIN_RANGE: f64 = 0.1;
const MAX_USABLE_RANGE: f64 = 25.0;
const PROCESS_DISTANCE: f64 = 0.1;
const PROCESS_YAW: f64 = 0.05;
const TIME_TOLERANCE: f64 = 0.05;
/// CARMEN logs mark "no return" with a reading at the sensor maximum.
const SYNTHETIC_NO_RETURN: f64 = 81.83;

struct Args {
    log: Option<String>,
    relations: Option<String>,
    map: Option<String>,
}

fn parse_args() -> DemoResult<Args> {
    let mut args = Args {
        log: None,
        relations: None,
        map: None,
    };
    let mut iter = std::env::args().skip(1);
    while let Some(arg) = iter.next() {
        if arg == "--map" {
            args.map = Some(iter.next().ok_or("--map needs a path")?);
        } else if args.log.is_none() {
            args.log = Some(arg);
        } else if args.relations.is_none() {
            args.relations = Some(arg);
        } else {
            return Err(format!("unexpected argument `{arg}`").into());
        }
    }
    Ok(args)
}

/// Synthetic corridor-loop log with a 180° front laser and biased odometry,
/// plus local and loop relations from ground truth.
fn synthetic_log() -> DemoResult<(String, String)> {
    let config = CorridorLoopConfig {
        step: 0.1,
        ..CorridorLoopConfig::default()
    };
    let walls = corridor_loop_walls();
    let mut rng = StdRng::seed_from_u64(5);
    let range_noise = Normal::new(0.0, 0.015)?;
    let odom_noise = Normal::new(0.0, 0.002)?;
    let beams = 180;
    let angle_min = -std::f64::consts::FRAC_PI_2;
    let angle_increment = std::f64::consts::PI / beams as f64;

    let mut truth = corridor_loop_start();
    let mut odometry = truth;
    let mut lines = Vec::new();
    let mut truth_poses = Vec::new();
    for (step, delta) in std::iter::once(Pose2D::origin())
        .chain(corridor_loop_deltas(&config))
        .enumerate()
    {
        truth = compose_pose(truth, delta);
        let odom_delta = Pose2D::new(
            delta.x * config.odometry_scale + odom_noise.sample(&mut rng),
            odom_noise.sample(&mut rng),
            delta.yaw + config.odometry_yaw_drift * delta.x + odom_noise.sample(&mut rng),
        );
        odometry = compose_pose(odometry, odom_delta);
        let ranges = ray_cast_ranges_fov(truth, &walls, angle_min, angle_increment, beams, 20.0)
            .into_iter()
            .map(|range| {
                if range.is_finite() {
                    range + range_noise.sample(&mut rng)
                } else {
                    SYNTHETIC_NO_RETURN
                }
            })
            .collect();
        let timestamp = step as f64 * 0.1;
        lines.push(format_flaser(&CarmenScan {
            timestamp,
            angle_min,
            angle_increment,
            max_range: f64::INFINITY,
            ranges,
            laser_odometry: odometry,
        }));
        truth_poses.push((timestamp, truth));
    }

    let relation = |i: usize, j: usize| Relation {
        timestamp_from: truth_poses[i].0,
        timestamp_to: truth_poses[j].0,
        relative: relative_pose(truth_poses[i].1, truth_poses[j].1),
    };
    let mut relations = Vec::new();
    // Local relations every 2 m over 2 m, like the benchmark's short relations…
    for i in (0..truth_poses.len().saturating_sub(20)).step_by(20) {
        relations.push(relation(i, i + 20));
    }
    // …and loop relations between revisits of the same place.
    for i in (0..truth_poses.len()).step_by(10) {
        if let Some(j) = (i + 300..truth_poses.len()).find(|&j| {
            let (a, b) = (truth_poses[i].1, truth_poses[j].1);
            (a.x - b.x).hypot(a.y - b.y) < 0.5
        }) {
            relations.push(relation(i, j));
        }
    }
    Ok((lines.join("\n"), format_relations(&relations)))
}

struct Trajectories {
    odometry: Vec<(f64, Pose2D)>,
    front_end: Vec<(f64, Pose2D)>,
    slam: Vec<(f64, Pose2D)>,
}

fn run_slam(scans: &[CarmenScan]) -> (Trajectories, LidarGraphSlam) {
    let first = scans
        .first()
        .map_or(Pose2D::origin(), |scan| scan.laser_odometry);
    let mut slam = LidarGraphSlam::new(LidarGraphSlamConfig::default(), first);
    let mut odometry = Vec::new();
    let mut front_end = Vec::new();
    // (timestamp, node index, offset from the node's front-end pose)
    let mut anchors: Vec<(f64, usize, Pose2D)> = Vec::new();
    let mut last_processed: Option<Pose2D> = None;

    for scan in scans {
        let delta = last_processed.map_or(Pose2D::origin(), |last| {
            relative_pose(last, scan.laser_odometry)
        });
        let moved = delta.x.hypot(delta.y) >= PROCESS_DISTANCE || delta.yaw.abs() >= PROCESS_YAW;
        if last_processed.is_none() || moved {
            let points = scan.points(MIN_RANGE, MAX_USABLE_RANGE);
            slam.update(delta, &points);
            last_processed = Some(scan.laser_odometry);
        }
        // Skipped scans are placed by odometry relative to the last processed
        // scan, so every scan (and every benchmark relation) gets a pose.
        let residual = last_processed.map_or(Pose2D::origin(), |last| {
            relative_pose(last, scan.laser_odometry)
        });
        let front_end_pose = compose_pose(slam.front_end_pose(), residual);
        let node = slam.node_poses().len() - 1;
        let node_front_end = slam.node_front_end_poses()[node];
        anchors.push((
            scan.timestamp,
            node,
            relative_pose(node_front_end, front_end_pose),
        ));
        odometry.push((scan.timestamp, scan.laser_odometry));
        front_end.push((scan.timestamp, front_end_pose));
    }

    slam.optimize();
    let nodes = slam.node_poses();
    let slam_trajectory = anchors
        .iter()
        .map(|(time, node, offset)| (*time, compose_pose(nodes[*node], *offset)))
        .collect();
    (
        Trajectories {
            odometry,
            front_end,
            slam: slam_trajectory,
        },
        slam,
    )
}

fn write_map_png(path: &str, slam: &LidarGraphSlam) -> DemoResult<()> {
    const RESOLUTION: f64 = 0.05;
    let points: Vec<Vector2<f64>> = slam
        .node_poses()
        .iter()
        .enumerate()
        .filter_map(|(index, pose)| {
            slam.node_scan(index)
                .map(|scan| transform_scan_to_world(scan, *pose))
        })
        .flatten()
        .collect();
    let (mut min, mut max) = (
        Vector2::new(f64::INFINITY, f64::INFINITY),
        Vector2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for point in &points {
        min = min.inf(point);
        max = max.sup(point);
    }
    let width = (((max.x - min.x) / RESOLUTION).ceil() as u32 + 1).clamp(1, 8000);
    let height = (((max.y - min.y) / RESOLUTION).ceil() as u32 + 1).clamp(1, 8000);
    let mut pixels = vec![255u8; (width * height) as usize];
    for point in &points {
        let u = ((point.x - min.x) / RESOLUTION) as u32;
        let v = ((max.y - point.y) / RESOLUTION) as u32;
        if u < width && v < height {
            pixels[(v * width + u) as usize] = 0;
        }
    }
    let file = std::io::BufWriter::new(std::fs::File::create(path)?);
    let mut encoder = png::Encoder::new(file, width, height);
    encoder.set_color(png::ColorType::Grayscale);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(&pixels)?;
    println!("Wrote map {path} ({width}×{height} px, {RESOLUTION} m/px)");
    Ok(())
}

fn print_errors(name: &str, errors: &RelationErrors) {
    println!(
        "{name:<18} {:>8} {:>10.4} ± {:<8.4} {:>10.5} {:>9.3} ± {:<7.3}",
        errors.matched,
        errors.translation_mean,
        errors.translation_std,
        errors.translation_mean_squared,
        errors.rotation_mean.to_degrees(),
        errors.rotation_std.to_degrees(),
    );
}

fn main() -> DemoResult<()> {
    let args = parse_args()?;
    let synthetic = args.log.is_none();
    let (log_text, relations_text) = match &args.log {
        Some(path) => (
            std::fs::read_to_string(path)?,
            match &args.relations {
                Some(path) => Some(std::fs::read_to_string(path)?),
                None => None,
            },
        ),
        None => {
            let (log, relations) = synthetic_log()?;
            (log, Some(relations))
        }
    };

    let scans = parse_carmen_log(&log_text)?;
    if scans.is_empty() {
        return Err("the log contains no FLASER / ROBOTLASER1 lines".into());
    }
    let (trajectories, slam) = run_slam(&scans);
    println!(
        "{}: {} scans, {} nodes, {} loop closures, {} ambiguous rejected",
        args.log.as_deref().unwrap_or("synthetic corridor-loop log"),
        scans.len(),
        slam.node_poses().len(),
        slam.loop_closures().len(),
        slam.ambiguous_loop_rejections(),
    );
    if let Some(path) = &args.map {
        write_map_png(path, &slam)?;
    }

    let Some(relations_text) = relations_text else {
        println!("No relations file given; skipping the benchmark metric.");
        return Ok(());
    };
    let relations = parse_relations(&relations_text)?;
    println!(
        "{:<18} {:>8} {:>21} {:>10} {:>19}",
        "estimator", "matched", "trans_abs_m", "trans_sq", "rot_abs_deg"
    );
    let odometry = evaluate_relations(&trajectories.odometry, &relations, TIME_TOLERANCE);
    let front_end = evaluate_relations(&trajectories.front_end, &relations, TIME_TOLERANCE);
    let graph = evaluate_relations(&trajectories.slam, &relations, TIME_TOLERANCE);
    print_errors("odometry", &odometry);
    print_errors("scan_to_map", &front_end);
    print_errors("graph_slam", &graph);
    if graph.unmatched > 0 {
        println!(
            "{} relations had no processed scan within {TIME_TOLERANCE} s",
            graph.unmatched
        );
    }

    if synthetic {
        if graph.matched == 0 || slam.loop_closures().is_empty() {
            return Err("synthetic run produced no loop closure or no matched relation".into());
        }
        if !(graph.translation_mean < front_end.translation_mean
            && front_end.translation_mean < odometry.translation_mean
            && graph.translation_mean < 0.05)
        {
            return Err("expected odometry > scan-to-map > graph SLAM relation error".into());
        }
    }
    Ok(())
}
