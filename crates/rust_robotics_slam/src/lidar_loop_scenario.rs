//! Deterministic corridor-loop scenario for [`LidarGraphSlam`].
//!
//! A robot drives slightly more than one lap of a 3 m wide rectangular
//! corridor (30 × 20 m outer walls, 24 × 14 m inner block) with pillars along
//! the walls — except the top corridor, which is left featureless so the
//! scan-to-map front end cannot observe forward motion there. Wheel odometry
//! over-reports distance and drifts in yaw; the simulated 2D LiDAR adds range
//! noise. [`run_corridor_loop`] records every step so examples, gallery
//! renderers, and the playground can replay the same run.

use nalgebra::Vector2;
use rand::{rngs::StdRng, SeedableRng};
use rand_distr::{Distribution, Normal};
use rust_robotics_core::Pose2D;

use crate::lidar_graph_slam::{LidarGraphSlam, LidarGraphSlamConfig, LoopClosure};
use crate::scan_to_map::{
    compose_pose, ranges_to_points, ray_cast_ranges, relative_pose, LineSegment,
};

/// Pillar arrangement along the bottom corridor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CorridorLayout {
    /// Irregularly spaced pillars: every place looks different.
    Irregular,
    /// Identical pillars every `spacing` meters on both walls of the bottom
    /// corridor: neighboring places look the same (perceptual aliasing).
    Periodic { spacing: f64 },
}

/// Scenario parameters for [`run_corridor_loop`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CorridorLoopConfig {
    /// Pillar arrangement along the bottom corridor.
    pub layout: CorridorLayout,
    /// Distance along the centerline (from the bottom-left start) at which
    /// the robot starts \[m\].
    pub start_offset: f64,
    /// Distance travelled per step \[m\].
    pub step: f64,
    /// Turning radius at the corridor corners \[m\].
    pub corner_radius: f64,
    /// Distance driven past one full lap \[m\].
    pub extra_distance: f64,
    /// Odometry distance scale factor (1.03 = over-reports by 3 %).
    pub odometry_scale: f64,
    /// Odometry yaw drift \[rad/m\].
    pub odometry_yaw_drift: f64,
    /// Standard deviation of per-step odometry noise \[m, rad\].
    pub odometry_noise: f64,
    /// Standard deviation of LiDAR range noise \[m\].
    pub range_noise: f64,
    /// LiDAR beams over 360°.
    pub beams: usize,
    /// LiDAR maximum range \[m\].
    pub max_range: f64,
    /// RNG seed for odometry and range noise.
    pub seed: u64,
}

impl Default for CorridorLoopConfig {
    fn default() -> Self {
        Self {
            layout: CorridorLayout::Irregular,
            start_offset: 0.0,
            step: 0.2,
            corner_radius: 1.5,
            extra_distance: 12.0,
            odometry_scale: 1.03,
            odometry_yaw_drift: 1.0_f64.to_radians(),
            odometry_noise: 0.003,
            range_noise: 0.02,
            beams: 180,
            max_range: 8.0,
            seed: 11,
        }
    }
}

/// State recorded after one [`LidarGraphSlam::update`].
#[derive(Debug, Clone, PartialEq)]
pub struct CorridorLoopFrame {
    /// Ground-truth pose.
    pub truth: Pose2D,
    /// Dead-reckoned wheel odometry.
    pub odometry: Pose2D,
    /// Scan-to-map front-end pose (no loop closure).
    pub front_end: Pose2D,
    /// Graph SLAM pose estimate.
    pub estimate: Pose2D,
    /// Optimized node poses at this step.
    pub node_poses: Vec<Pose2D>,
    /// Loop closure accepted at this step, if any.
    pub loop_closure: Option<LoopClosure>,
    /// Whether the graph was re-optimized at this step.
    pub optimized: bool,
    /// Noisy body-frame scan observed at this step.
    pub scan: Vec<Vector2<f64>>,
}

/// A full recorded run of the corridor-loop scenario.
#[derive(Debug, Clone)]
pub struct CorridorLoopRun {
    /// Environment walls.
    pub walls: Vec<LineSegment>,
    /// Length of one lap of the corridor centerline \[m\].
    pub lap_length: f64,
    /// Total distance driven \[m\].
    pub distance: f64,
    /// One frame per update, starting with the initial scan.
    pub frames: Vec<CorridorLoopFrame>,
    /// Body-frame scan of each node.
    pub node_scans: Vec<Vec<Vector2<f64>>>,
    /// Ground truth at each node.
    pub node_truth: Vec<Pose2D>,
    /// Wheel odometry at each node.
    pub node_odometry: Vec<Pose2D>,
    /// Front-end pose at each node.
    pub node_front_end: Vec<Pose2D>,
    /// Node poses after a final optimization over all edges.
    pub final_node_poses: Vec<Pose2D>,
    /// Pose estimate after the final optimization.
    pub final_estimate: Pose2D,
    /// All accepted loop closures.
    pub loop_closures: Vec<LoopClosure>,
    /// Loop matches rejected as ambiguous (perceptual aliasing).
    pub ambiguous_loop_rejections: usize,
}

impl CorridorLoopRun {
    /// Whether `closure`'s measured relative pose is off from ground truth by
    /// more than `tolerance` meters (a false loop closure).
    pub fn is_wrong_loop(&self, closure: &LoopClosure, tolerance: f64) -> bool {
        let truth = relative_pose(self.node_truth[closure.from], self.node_truth[closure.to]);
        let error = relative_pose(truth, closure.relative);
        error.x.hypot(error.y) > tolerance
    }

    /// Number of false loop closures (see [`Self::is_wrong_loop`]).
    pub fn wrong_loop_closures(&self, tolerance: f64) -> usize {
        self.loop_closures
            .iter()
            .filter(|closure| self.is_wrong_loop(closure, tolerance))
            .count()
    }

    /// Index of the first frame that accepted a loop closure.
    pub fn first_loop_frame(&self) -> Option<usize> {
        self.frames
            .iter()
            .position(|frame| frame.loop_closure.is_some())
    }
}

/// Perceptual-aliasing variant: identical pillars every 2.5 m along the
/// bottom corridor and a start in its middle, where no corner is in LiDAR
/// range, so loop matches can lock onto the wrong pillar.
pub fn aliased_corridor_config() -> CorridorLoopConfig {
    CorridorLoopConfig {
        layout: CorridorLayout::Periodic { spacing: 2.5 },
        start_offset: 12.0,
        extra_distance: 14.0,
        ..CorridorLoopConfig::default()
    }
}

/// Planar position RMSE between two equally indexed pose sequences \[m\].
pub fn position_rmse(estimates: &[Pose2D], truth: &[Pose2D]) -> f64 {
    let sum: f64 = estimates
        .iter()
        .zip(truth)
        .map(|(estimate, truth)| (estimate.x - truth.x).powi(2) + (estimate.y - truth.y).powi(2))
        .sum();
    (sum / truth.len().max(1) as f64).sqrt()
}

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

/// Walls of the corridor loop: outer 30 × 20 m, inner block 24 × 14 m, and
/// pillars at irregular spacing except along the top corridor.
pub fn corridor_loop_walls() -> Vec<LineSegment> {
    corridor_loop_walls_for(CorridorLayout::Irregular)
}

/// Walls of the corridor loop with the given bottom-corridor `layout`.
pub fn corridor_loop_walls_for(layout: CorridorLayout) -> Vec<LineSegment> {
    let mut walls = rectangle((-15.0, -10.0), (15.0, 10.0));
    walls.extend(rectangle((-12.0, -7.0), (12.0, 7.0)));
    let pillar = |x: f64, y: f64| rectangle((x - 0.2, y - 0.2), (x + 0.2, y + 0.2));
    match layout {
        CorridorLayout::Irregular => {
            for x in [-9.3, -4.1, 1.2, 6.7, 10.4] {
                walls.extend(pillar(x, -9.8));
            }
            for x in [-7.6, -1.9, 3.8, 8.9] {
                walls.extend(pillar(x, -7.2));
            }
        }
        CorridorLayout::Periodic { spacing } => {
            let spacing = spacing.max(0.5);
            let count = (22.0 / spacing).floor() as i32;
            for i in 0..=count {
                let x = -11.0 + f64::from(i) * spacing;
                walls.extend(pillar(x, -9.8));
                walls.extend(pillar(x, -7.2));
            }
        }
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

/// Start pose of the scenario (bottom corridor, heading +x).
pub fn corridor_loop_start() -> Pose2D {
    Pose2D::new(-12.0, -8.5, 0.0)
}

/// Length of one lap of the corridor centerline \[m\].
pub fn corridor_loop_lap_length(corner_radius: f64) -> f64 {
    2.0 * (27.0 + 17.0) - 8.0 * corner_radius + 2.0 * std::f64::consts::PI * corner_radius
}

/// Start pose of `config`: `start_offset` meters along the centerline.
pub fn corridor_loop_start_for(config: &CorridorLoopConfig) -> Pose2D {
    centerline_deltas(config, config.start_offset)
        .into_iter()
        .fold(corridor_loop_start(), compose_pose)
}

/// Body-frame step deltas along the corridor centerline (rounded rectangle),
/// starting `start_offset` meters along it and covering one lap plus
/// `extra_distance`.
pub fn corridor_loop_deltas(config: &CorridorLoopConfig) -> Vec<Pose2D> {
    let skipped = centerline_deltas(config, config.start_offset).len();
    let total = corridor_loop_lap_length(config.corner_radius) + config.extra_distance;
    centerline_deltas(config, config.start_offset + total)
        .into_iter()
        .skip(skipped)
        .collect()
}

/// Deltas from the bottom-left start covering at least `total` meters.
fn centerline_deltas(config: &CorridorLoopConfig, total: f64) -> Vec<Pose2D> {
    if total <= 0.0 {
        return Vec::new();
    }
    let radius = config.corner_radius;
    let quarter = std::f64::consts::FRAC_PI_2 * radius;
    let half_lap = [
        (27.0 - 2.0 * radius, 0.0),
        (quarter, 1.0 / radius),
        (17.0 - 2.0 * radius, 0.0),
        (quarter, 1.0 / radius),
    ];
    let mut deltas = Vec::new();
    let mut travelled = 0.0;
    'outer: loop {
        // `half_lap` covers half the rectangle; the other half mirrors it.
        for (length, curvature) in half_lap.iter().chain(&half_lap) {
            let steps = ((length / config.step).round() as usize).max(1);
            let step = length / steps as f64;
            for _ in 0..steps {
                deltas.push(Pose2D::new(step, 0.0, step * curvature));
                travelled += step;
                if travelled >= total {
                    break 'outer;
                }
            }
        }
    }
    deltas
}

/// Runs [`LidarGraphSlam`] through the corridor loop and records every step.
///
/// # Panics
///
/// Panics if `odometry_noise` or `range_noise` is negative or not finite.
pub fn run_corridor_loop(
    config: &CorridorLoopConfig,
    slam_config: LidarGraphSlamConfig,
) -> CorridorLoopRun {
    let walls = corridor_loop_walls_for(config.layout);
    let deltas = corridor_loop_deltas(config);
    let mut rng = StdRng::seed_from_u64(config.seed);
    let range_noise = Normal::new(0.0, config.range_noise).expect("valid range noise");
    let odom_noise = Normal::new(0.0, config.odometry_noise).expect("valid odometry noise");
    let scan = |pose: Pose2D, rng: &mut StdRng| {
        let ranges: Vec<f64> = ray_cast_ranges(pose, &walls, config.beams, config.max_range)
            .into_iter()
            .map(|range| range + range_noise.sample(rng))
            .collect();
        ranges_to_points(&ranges)
    };

    let start = corridor_loop_start_for(config);
    let mut truth = start;
    let mut odometry = start;
    let mut slam = LidarGraphSlam::new(slam_config, start);
    let mut frames = Vec::with_capacity(deltas.len() + 1);
    let mut node_truth = Vec::new();
    let mut node_odometry = Vec::new();

    let mut record = |slam: &mut LidarGraphSlam,
                      truth: Pose2D,
                      odometry: Pose2D,
                      odom_delta: Pose2D,
                      scan: Vec<Vector2<f64>>| {
        let update = slam.update(odom_delta, &scan);
        if update.new_node.is_some() {
            node_truth.push(truth);
            node_odometry.push(odometry);
        }
        frames.push(CorridorLoopFrame {
            truth,
            odometry,
            front_end: slam.front_end_pose(),
            estimate: update.pose,
            node_poses: slam.node_poses(),
            loop_closure: update.loop_closure,
            optimized: update.optimized,
            scan,
        });
    };

    let initial_scan = scan(truth, &mut rng);
    record(&mut slam, truth, odometry, Pose2D::origin(), initial_scan);
    for true_delta in &deltas {
        truth = compose_pose(truth, *true_delta);
        let odom_delta = Pose2D::new(
            true_delta.x * config.odometry_scale + odom_noise.sample(&mut rng),
            odom_noise.sample(&mut rng),
            true_delta.yaw + config.odometry_yaw_drift * true_delta.x + odom_noise.sample(&mut rng),
        );
        odometry = compose_pose(odometry, odom_delta);
        let current_scan = scan(truth, &mut rng);
        record(&mut slam, truth, odometry, odom_delta, current_scan);
    }

    // Fold in loop edges that were consistent with the graph when added.
    slam.optimize();
    let node_count = slam.node_poses().len();
    CorridorLoopRun {
        lap_length: corridor_loop_lap_length(config.corner_radius),
        distance: deltas.iter().map(|delta| delta.x).sum(),
        frames,
        node_scans: (0..node_count)
            .map(|index| slam.node_scan(index).unwrap_or_default().to_vec())
            .collect(),
        node_truth,
        node_odometry,
        node_front_end: slam.node_front_end_poses(),
        final_node_poses: slam.node_poses(),
        final_estimate: slam.pose(),
        loop_closures: slam.loop_closures().to_vec(),
        ambiguous_loop_rejections: slam.ambiguous_loop_rejections(),
        walls,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn short_run_records_frames_and_nodes_consistently() {
        let config = CorridorLoopConfig {
            extra_distance: -70.0,
            ..CorridorLoopConfig::default()
        };
        let run = run_corridor_loop(&config, LidarGraphSlamConfig::default());
        assert_eq!(run.frames.len(), corridor_loop_deltas(&config).len() + 1);
        let nodes = run.final_node_poses.len();
        assert!(nodes > 5);
        assert_eq!(run.node_truth.len(), nodes);
        assert_eq!(run.node_odometry.len(), nodes);
        assert_eq!(run.node_front_end.len(), nodes);
        assert_eq!(run.node_scans.len(), nodes);
        assert_eq!(run.frames.last().map(|f| f.node_poses.len()), Some(nodes));
        // No revisit yet, so nothing to close.
        assert!(run.loop_closures.is_empty());
        assert!(run.first_loop_frame().is_none());
    }

    #[test]
    fn lap_deltas_close_the_rectangle() {
        let config = CorridorLoopConfig {
            extra_distance: 0.0,
            ..CorridorLoopConfig::default()
        };
        let end = corridor_loop_deltas(&config)
            .into_iter()
            .fold(corridor_loop_start(), compose_pose);
        let start = corridor_loop_start();
        assert!((end.x - start.x).hypot(end.y - start.y) < 0.25);
    }
}
