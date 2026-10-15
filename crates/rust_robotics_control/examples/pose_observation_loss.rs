//! Deterministic pose-observation loss experiment for Pure Pursuit.
//!
//! See experiments/pose_observation_loss/README.md for the model, limitations,
//! paired experiment runner, and recorded outcomes. This is a simulation of a
//! candidate pause/recovery policy, not a production safety controller.

use rust_robotics_control::pure_pursuit::{PurePursuitConfig, PurePursuitController, VehicleState};
use rust_robotics_core::{Path2D, Point2D};
use std::collections::{BTreeMap, VecDeque};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

const DT: f64 = 0.02;
const WHEELBASE: f64 = 0.4;
const RADIUS: f64 = 0.5;
const TARGET_SPEED: f64 = 1.0;
const GOAL_THRESHOLD: f64 = 0.25;
const MAX_TIME: f64 = 180.0;
const MAX_STEERING: f64 = 35.0 * std::f64::consts::PI / 180.0;
const SPACING: f64 = 0.1;

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

fn distance_to_path(p: Point2D, path: &Path2D) -> f64 {
    path.points
        .windows(2)
        .map(|s| point_segment_distance(p, s[0], s[1]))
        .fold(f64::INFINITY, f64::min)
}

// Preserve every original corner; split each straight segment into intervals
// no longer than SPACING instead of smoothing or cutting corners.
fn resample(path: &Path2D) -> Path2D {
    let mut points = vec![path.points[0]];
    for s in path.points.windows(2) {
        let n = (s[0].distance(&s[1]) / SPACING).ceil() as usize;
        assert!(n > 0);
        for index in 1..=n {
            let t = index as f64 / n as f64;
            points.push(Point2D::new(
                s[0].x + t * (s[1].x - s[0].x),
                s[0].y + t * (s[1].y - s[0].y),
            ));
        }
    }
    let dense = Path2D::from_points(points);
    assert!((dense.total_length() - path.total_length()).abs() < 1e-7);
    dense
}

struct NoiseRng(u64);
impl NoiseRng {
    fn uniform(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9e3779b97f4a7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
        z ^= z >> 31;
        ((z >> 11) as f64 + 0.5) / 9007199254740992.0
    }
    fn normal(&mut self) -> f64 {
        let u1 = self.uniform();
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

// Predict the proposed command for 0.2 s, then the existing bounded brake,
// holding the proposed steering. Return a conservative lower bound on center
// clearance. Distant points use the triangle inequality instead of all segments.
fn stopping_preview(
    sensed: VehicleState,
    target: f64,
    steering: f64,
    margin: f64,
    obstacles: &[Point2D],
    recovery_preview: bool,
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
    let recovery_clearance = if recovery_preview {
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

// Find the first unsafe chord of a fixed-steering arc, in 0.05-unit intervals.
// This is an estimated geometric probe, not a bound on future closed-loop motion.
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

// This gate compares measurements across a scheduled correction while stopped.
// Agreement is only a heuristic: a common observation bias can pass it.
#[derive(Default)]
struct RestartConsistency {
    samples: VecDeque<(f64, f64, f64)>,
    previous: Option<(f64, f64, f64)>,
    epoch: Option<f64>,
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

impl RestartConsistency {
    fn observe(&mut self, sensed: VehicleState, active: bool, event: ObservationEvent) {
        if !active || sensed.v.abs() > 0.02 || event.precision_activation {
            *self = Self::default();
        } else if event.correction {
            self.previous =
                (event.received && self.samples.len() == 25).then(|| mean_pose(&self.samples));
            self.samples.clear();
            self.epoch = event.received.then_some(event.time);
        }
        if active && sensed.v.abs() <= 0.02 && !event.precision_activation {
            self.samples.push_back((sensed.x, sensed.y, sensed.yaw));
            if self.samples.len() > 25 {
                self.samples.pop_front();
            }
        }
    }

    fn deltas(&self) -> (f64, f64) {
        match (self.previous, self.samples.is_empty()) {
            (Some(before), false) => {
                let after = mean_pose(&self.samples);
                let yaw = after.2 - before.2;
                (
                    (after.0 - before.0).hypot(after.1 - before.1),
                    yaw.sin().atan2(yaw.cos()).abs(),
                )
            }
            _ => (-1.0, -1.0),
        }
    }

    fn ready(&self, time: f64) -> bool {
        let (position, yaw) = self.deltas();
        self.previous.is_some()
            && self.samples.len() >= 10
            && self.epoch.is_some_and(|epoch| time - epoch <= 1.5)
            && (0.0..=0.2).contains(&position)
            && (0.0..=5.0_f64.to_radians()).contains(&yaw)
    }
}

#[cfg(test)]
mod motion_filter_tests {
    use super::*;

    #[test]
    fn stationary_stop_can_be_clear_while_forward_recovery_is_blocked() {
        let state = VehicleState::new(25.0, 25.0, 0.0, 0.0, WHEELBASE);
        let obstacles = [Point2D::new(26.2, 25.0)];
        let (stop, recovery) = stopping_preview(state, 0.0, 0.0, 0.8, &obstacles, true);
        assert!((stop - 1.2).abs() < 1e-12);
        assert!((recovery - 0.6).abs() < 1e-12);
        assert!(stop > 0.8 && recovery - 0.1 < 0.8);
        let (original, sentinel) = stopping_preview(state, 0.0, 0.0, 0.8, &obstacles, false);
        assert_eq!(original, stop);
        assert_eq!(sentinel, 0.0);
    }

    #[test]
    fn recovery_segment_includes_map_boundary() {
        let state = VehicleState::new(49.0, 25.0, 0.0, 0.0, WHEELBASE);
        let (stop, recovery) = stopping_preview(state, 0.0, 0.0, 0.8, &[], true);
        assert_eq!(stop, 1.0);
        assert!((recovery - 0.4).abs() < 1e-12);
    }

    #[test]
    fn terminal_recovery_preview_respects_curved_motion_symmetry() {
        let left = VehicleState::new(25.0, 25.0, 0.0, 0.7, WHEELBASE);
        let a = stopping_preview(left, 0.8, 0.4, 0.8, &[Point2D::new(26.0, 25.6)], true);
        let b = stopping_preview(left, 0.8, -0.4, 0.8, &[Point2D::new(26.0, 24.4)], true);
        assert!((a.0 - b.0).abs() < 1e-12);
        assert!((a.1 - b.1).abs() < 1e-12);
    }

    #[test]
    fn compensates_straight_motion_and_cancels_alternating_errors() {
        let mut filter = MotionPoseFilter::default();
        let mut estimate = VehicleState::new(0.0, 0.0, 0.0, 1.0, WHEELBASE);
        for i in 0..24 {
            let error = if i % 2 == 0 { 0.1 } else { -0.1 };
            estimate = filter.observe(
                VehicleState::new(i as f64 * DT + error, 0.0, 0.0, 1.0, WHEELBASE),
                0.0,
                false,
            );
        }
        assert!((estimate.x - 23.0 * DT).abs() < 1e-12);
        assert!(estimate.y.abs() < 1e-12);
    }

    #[test]
    fn curved_motion_transports_old_heading_before_averaging() {
        let mut filter = MotionPoseFilter::default();
        filter.observe(VehicleState::new(0.0, 0.0, 0.0, 1.0, WHEELBASE), 0.0, false);
        filter.observe(
            VehicleState::new(0.02, 0.0, 0.02, 1.0, WHEELBASE),
            0.4_f64.atan(),
            false,
        );
        let x = 0.02 + 0.02 * 0.02_f64.cos();
        let y = 0.02 * 0.02_f64.sin();
        let estimate = filter.observe(
            VehicleState::new(x, y, 0.04, 1.0, WHEELBASE),
            0.4_f64.atan(),
            false,
        );
        assert!((estimate.x - x).abs() < 1e-12);
        assert!((estimate.y - y).abs() < 1e-12);
        assert!((estimate.yaw - 0.04).abs() < 1e-12);
    }

    #[test]
    fn circular_heading_mean_respects_wraparound() {
        let mut filter = MotionPoseFilter::default();
        let pi = std::f64::consts::PI;
        filter.observe(
            VehicleState::new(0.0, 0.0, pi - 0.1, 0.0, WHEELBASE),
            0.0,
            false,
        );
        let estimate = filter.observe(
            VehicleState::new(0.0, 0.0, -pi + 0.1, 0.0, WHEELBASE),
            0.0,
            false,
        );
        assert!((estimate.yaw.abs() - pi).abs() < 1e-12);
    }

    #[test]
    fn window_is_bounded_and_correction_discards_old_bias() {
        let mut filter = MotionPoseFilter::default();
        for _ in 0..40 {
            filter.observe(
                VehicleState::new(10.0, 0.0, 0.0, 0.0, WHEELBASE),
                0.0,
                false,
            );
        }
        assert_eq!(filter.samples.len(), 25);
        let estimate = filter.observe(VehicleState::new(2.0, 3.0, 0.4, 0.0, WHEELBASE), 0.0, true);
        assert_eq!(filter.samples.len(), 1);
        assert!((estimate.x - 2.0).abs() < 1e-12);
        assert!((estimate.y - 3.0).abs() < 1e-12);
        assert!((estimate.yaw - 0.4).abs() < 1e-12);
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if !(15..=25).contains(&args.len()) {
        return Err("usage: pose_observation_loss INPUT OUTPUT LOOKAHEAD POSITION_SIGMA YAW_SIGMA_DEG SEED DRIFT_SPEED DRIFT_YAW_DEG_S CORRECTION_PERIOD SLOWDOWN STOP_HOLD PRECISE_TERMINAL LOSS_MODE LOSS_AWARE [AVERAGED_RECOVERY [NOISE_CORRELATION_TIME_S [STOPPING_GUARD [CONSISTENT_RESTART [ANTICIPATORY_SPEED [JOINT_RECOVERY [RECOVERY_GUARD_BUFFER [MOTION_ENTRY_BUFFER [MOTION_FILTER [RECOVERY_PREVIEW]]]]]]]]]]; see experiments/pose_observation_loss/run.py".into());
    }
    let input = Path::new(&args[1]);
    let output = Path::new(&args[2]);
    let lookahead: f64 = args[3].parse()?;
    let position_sigma: f64 = args[4].parse()?;
    let yaw_sigma: f64 = args[5].parse::<f64>()?.to_radians();
    let seed: u64 = args[6].parse()?;
    let terminal_slowdown: bool = args[10].parse()?;
    let stop_hold: bool = args[11].parse()?;
    let precise_terminal: bool = args[12].parse()?;
    let loss_mode = args[13].as_str();
    let loss_aware: bool = args[14].parse()?;
    let averaged_recovery = args
        .get(15)
        .map(|s| s.parse::<bool>())
        .transpose()?
        .unwrap_or(false);
    let noise_correlation_time_s = args
        .get(16)
        .map(|s| s.parse::<f64>())
        .transpose()?
        .unwrap_or(0.0);
    assert!(noise_correlation_time_s.is_finite() && noise_correlation_time_s >= 0.0);
    let noise_rho = if noise_correlation_time_s == 0.0 {
        0.0
    } else {
        (-DT / noise_correlation_time_s).exp()
    };
    let innovation_scale = (1.0 - noise_rho * noise_rho).sqrt();
    let stopping_guard = args
        .get(17)
        .map(|s| s.parse::<bool>())
        .transpose()?
        .unwrap_or(false);
    let consistent_restart = args
        .get(18)
        .map(|s| s.parse::<bool>())
        .transpose()?
        .unwrap_or(false);
    let anticipatory_speed = args
        .get(19)
        .map(|s| s.parse::<bool>())
        .transpose()?
        .unwrap_or(false);
    let joint_recovery = args
        .get(20)
        .map(|s| s.parse::<bool>())
        .transpose()?
        .unwrap_or(false);
    let recovery_guard_buffer = args
        .get(21)
        .map(|s| s.parse::<bool>())
        .transpose()?
        .unwrap_or(false);
    assert!(!recovery_guard_buffer || joint_recovery);
    let motion_entry_buffer = args
        .get(22)
        .map(|s| s.parse::<bool>())
        .transpose()?
        .unwrap_or(false);
    assert!(!motion_entry_buffer || (joint_recovery && !recovery_guard_buffer));
    let motion_filter = args
        .get(23)
        .map(|s| s.parse::<bool>())
        .transpose()?
        .unwrap_or(false);
    assert!(!motion_filter || (joint_recovery && !motion_entry_buffer && !recovery_guard_buffer));
    let recovery_preview = args
        .get(24)
        .map(|s| s.parse::<bool>())
        .transpose()?
        .unwrap_or(false);
    assert!(!recovery_preview || motion_filter);
    assert!(!joint_recovery || anticipatory_speed);
    assert!(!anticipatory_speed || (stopping_guard && !consistent_restart));
    assert!(!consistent_restart || stopping_guard);
    assert!(!stopping_guard || averaged_recovery);
    assert!(!averaged_recovery || loss_aware);
    assert!(["none", "random", "blackout"].contains(&loss_mode));
    let drift_speed: f64 = args[7].parse()?;
    let drift_yaw_rate: f64 = args[8].parse::<f64>()?.to_radians();
    let correction_period: f64 = args[9].parse()?;
    assert!(drift_speed.is_finite() && drift_speed >= 0.0);
    assert!(drift_yaw_rate.is_finite());
    assert!(correction_period == 0.0 || correction_period >= DT);
    let correction_steps = (correction_period / DT).round() as usize;
    let mut direction_rng = NoiseRng(seed ^ 0xa0761d6478bd642f);
    let drift_angle = 2.0 * std::f64::consts::PI * direction_rng.uniform();
    let yaw_sign = if direction_rng.uniform() < 0.5 {
        -1.0
    } else {
        1.0
    };
    assert!(position_sigma.is_finite() && position_sigma >= 0.0);
    assert!(yaw_sigma.is_finite() && yaw_sigma >= 0.0);
    assert!(lookahead.is_finite() && lookahead > 0.0);
    let mut paths: BTreeMap<(String, String), Vec<Point2D>> = BTreeMap::new();
    for line in std::fs::read_to_string(input.join("paths.csv"))?
        .lines()
        .skip(1)
    {
        let fields: Vec<&str> = line.split(',').collect();
        let points = paths
            .entry((fields[0].to_owned(), fields[1].to_owned()))
            .or_default();
        assert_eq!(fields[2].parse::<usize>()?, points.len());
        points.push(Point2D::new(fields[3].parse()?, fields[4].parse()?));
    }
    assert_eq!(paths.len(), 9);
    let mut obstacles: BTreeMap<String, Vec<Point2D>> = BTreeMap::new();
    for line in std::fs::read_to_string(input.join("obstacles.csv"))?
        .lines()
        .skip(1)
    {
        let f: Vec<&str> = line.split(',').collect();
        obstacles
            .entry(f[0].to_owned())
            .or_default()
            .push(Point2D::new(f[1].parse()?, f[2].parse()?));
    }
    let mut tracks = BufWriter::new(File::create(output.join("trajectories.csv"))?);
    let mut summary = BufWriter::new(File::create(output.join("results.csv"))?);
    writeln!(
        tracks,
        "scenario,planner,time_s,x,y,yaw,v,tracking_error,step_min_obstacle_distance,noise_x,noise_y,noise_yaw,bias_x,bias_y,bias_yaw,correction,target_speed,sensed_goal_distance,filtered_goal_distance,stop_latched,hold_time_s,restart_limited,precision_active,effective_position_sigma,effective_yaw_sigma_rad,precision_activation,observation_available,correction_received,loss_uniform,observation_age,safety_paused,recovery_evidence,recovery_clearance,resumed,steering,acceleration,recovery_x,recovery_y,recovery_yaw,recovery_sample_count,noise_innovation_x,noise_innovation_y,noise_innovation_yaw,noise_rho,guard_active,guard_triggered,guard_released,guard_release_evidence,preview_clearance_lower_bound,preview_margin,nominal_target_speed,nominal_steering,consistency_ready,pre_sample_count,stationary_sample_count,consistency_age,position_delta,yaw_delta_rad,preview_pose_x,preview_pose_y,preview_pose_yaw,speed_free_distance,speed_cap,speed_margin,speed_braking,uncapped_target_speed,guard_release_margin,guard_entry_extra,motion_filter_sample_count,recovery_preview_active,stop_preview_clearance_lower_bound,terminal_recovery_clearance"
    )?;
    writeln!(summary, "scenario,planner,lookahead_offset,status,reached,collision,time_s,distance_travelled,goal_distance,rms_tracking_error,max_tracking_error,min_obstacle_distance,min_radius_margin,resampled_points,held,hold_time_s,first_goal_entry_s,final_speed,max_hold_distance,latch_count,pause_count,resume_count,paused_time_s")?;
    for ((scenario, planner), points) in paths {
        let path = Path2D::from_points(points);
        assert!(path.len() >= 2);
        let initial = path.points[0];
        let goal = *path.points.last().unwrap();
        let yaw = (path.points[1].y - initial.y).atan2(path.points[1].x - initial.x);
        let dense = resample(&path);
        let dense_count = dense.len();
        let mut controller = PurePursuitController::new(PurePursuitConfig {
            look_ahead_gain: 0.1,
            look_ahead_distance: lookahead,
            wheelbase: WHEELBASE,
            kp: 1.0,
            goal_threshold: GOAL_THRESHOLD,
        });
        controller.set_path(dense.clone());
        let mut state = VehicleState::new(initial.x, initial.y, yaw, 0.0, WHEELBASE);
        let mut rng = NoiseRng(seed);
        let mut correlated_normals = [0.0; 3];
        let mut anchor_rng = NoiseRng(seed ^ 0xe7037ed1a0b428db);
        let mut terminal_anchor_rng = NoiseRng(seed ^ 0x8ebc6af09c88c6e3);
        let mut precision_active = false;
        let mut loss_rng = NoiseRng(seed ^ 0x589965cc75374cc3);
        let mut loss_uniform = 0.0;
        let (mut bias_x, mut bias_y, mut bias_yaw) = (0.0, 0.0, 0.0);
        let obs = &obstacles[&scenario];
        let initial_clearance = obs
            .iter()
            .map(|p| p.distance(&initial))
            .fold(f64::INFINITY, f64::min);
        assert!(initial_clearance > RADIUS);
        let initial_goal_distance = initial.distance(&goal);
        writeln!(
            tracks,
            "{scenario},{planner},0,{},{},{yaw},0,0,{initial_clearance},0,0,0,0,0,0,false,1,{initial_goal_distance},{initial_goal_distance},false,0,false,false,{position_sigma},{yaw_sigma},false,true,false,0,0,false,0,0,false,0,0,{},{},{yaw},0,0,0,0,{noise_rho:.12},false,false,false,0,0,0,1,0,false,0,0,-1,-1,-1,{},{},{yaw},2,1,0,false,1,0,0,0,false,0,0",
            initial.x, initial.y, initial.x, initial.y, initial.x, initial.y
        )?;
        let mut rms_sum = 0.0;
        let mut previous_error = 0.0;
        let mut max_error: f64 = 0.0;
        let mut clearance = initial_clearance;
        let mut travelled = 0.0;
        let mut time = 0.0;
        let mut reached = false;
        let mut collision = false;
        let mut status = "timeout";
        let mut filtered_goal_distance = initial_goal_distance;
        let mut stop_latched = false;
        let mut stop_evidence = 0.0;
        let mut resume_evidence = 0.0;
        let mut hold_time_s = 0.0;
        let mut max_hold_distance: f64 = 0.0;
        let mut first_goal_entry_s = -1.0;
        let mut held = false;
        let mut latch_count = 0;
        let mut last_observation_time = 0.0;
        let mut safety_paused = false;
        let mut recovery_evidence = 0.0;
        let mut pause_count = 0;
        let mut resume_count = 0;
        let mut paused_time_s = 0.0;
        let mut recovery_limit_until = 0.0;
        let mut guard_active = false;
        let mut guard_steering = 0.0;
        let mut guard_release_evidence = 0.0;
        let mut consistency = RestartConsistency::default();
        let mut pose_filter = MotionPoseFilter::default();
        let mut previous_steering = 0.0;
        let mut recovery_samples: VecDeque<(f64, f64, f64)> = VecDeque::new();
        for step in 1..=(MAX_TIME / DT) as usize {
            let from = Point2D::new(state.x, state.y);
            if step == 1 || (correction_steps > 0 && step % correction_steps == 0) {
                loss_uniform = loss_rng.uniform();
            }
            let observation_time = step as f64 * DT;
            let observation_available = match loss_mode {
                "random" => loss_uniform >= 0.3,
                "blackout" => !(45.0..75.0).contains(&observation_time),
                _ => true,
            };
            let precision_activation = precise_terminal
                && !precision_active
                && filtered_goal_distance <= 2.0
                && observation_available;
            precision_active |= precision_activation;
            let effective_position_sigma = if precision_active {
                0.03
            } else {
                position_sigma
            };
            let effective_yaw_sigma = if precision_active {
                0.6_f64.to_radians()
            } else {
                yaw_sigma
            };
            // Unit stationary variance for all correlation times. The first
            // sample is stationary, rather than ramping up from a zero state.
            // Consume exactly the legacy three draws even in correlated runs.
            let innovations = [rng.normal(), rng.normal(), rng.normal()];
            for axis in 0..3 {
                correlated_normals[axis] = if step == 1 || noise_rho == 0.0 {
                    innovations[axis]
                } else {
                    noise_rho * correlated_normals[axis] + innovation_scale * innovations[axis]
                };
            }
            let noise_x = effective_position_sigma * correlated_normals[0];
            let noise_y = effective_position_sigma * correlated_normals[1];
            let noise_yaw = effective_yaw_sigma * correlated_normals[2];
            bias_x += drift_speed * drift_angle.cos() * DT;
            bias_y += drift_speed * drift_angle.sin() * DT;
            bias_yaw += yaw_sign * drift_yaw_rate * DT;
            let correction = correction_steps > 0 && step % correction_steps == 0;
            let correction_received = correction && observation_available;
            if correction {
                // Draw for every scheduled opportunity, even if lost, so
                // later received observations stay paired across conditions.
                let anchor_x = anchor_rng.normal();
                let anchor_y = anchor_rng.normal();
                let anchor_yaw = anchor_rng.normal();
                if correction_received {
                    bias_x = effective_position_sigma * anchor_x;
                    bias_y = effective_position_sigma * anchor_y;
                    bias_yaw = effective_yaw_sigma * anchor_yaw;
                }
            }
            if precision_activation && !correction_received {
                // A newly available external terminal pose observation.
                // Keep its random stream separate from scheduled corrections.
                bias_x = effective_position_sigma * terminal_anchor_rng.normal();
                bias_y = effective_position_sigma * terminal_anchor_rng.normal();
                bias_yaw = effective_yaw_sigma * terminal_anchor_rng.normal();
            }
            let sensed = if position_sigma == 0.0
                && yaw_sigma == 0.0
                && drift_speed == 0.0
                && drift_yaw_rate == 0.0
            {
                state
            } else {
                VehicleState::new(
                    state.x + noise_x + bias_x,
                    state.y + noise_y + bias_y,
                    state.yaw + noise_yaw + bias_yaw,
                    state.v,
                    WHEELBASE,
                )
            };
            if correction_received || precision_activation {
                last_observation_time = observation_time;
            }
            let observation_age = observation_time - last_observation_time;
            if loss_aware && correction && !correction_received && !safety_paused {
                safety_paused = true;
                recovery_evidence = 0.0;
                pause_count += 1;
            }
            // After a fresh correction, discard the stale forward-only path cursor.
            if loss_aware && (safety_paused || guard_active) && correction_received {
                controller.set_path(dense.clone());
            }
            if averaged_recovery {
                // Only measured stationary poses from the current correction epoch.
                // A correction, missed observation, motion, or an expired observation
                // invalidates this window. No samples span a bias reset.
                if correction_received
                    || precision_activation
                    || (correction && !correction_received)
                    || sensed.v.abs() > 0.02
                    || observation_age > 1.5
                {
                    recovery_samples.clear();
                }
                if safety_paused && sensed.v.abs() <= 0.02 && observation_age <= 1.5 {
                    recovery_samples.push_back((sensed.x, sensed.y, sensed.yaw));
                    if recovery_samples.len() > 25 {
                        recovery_samples.pop_front();
                    }
                }
            }
            let recovery_sample_count = recovery_samples.len();
            let (recovery_x, recovery_y, recovery_yaw) =
                if averaged_recovery && recovery_sample_count > 0 {
                    let n = recovery_sample_count as f64;
                    let x = recovery_samples.iter().map(|p| p.0).sum::<f64>() / n;
                    let y = recovery_samples.iter().map(|p| p.1).sum::<f64>() / n;
                    let sin_yaw = recovery_samples.iter().map(|p| p.2.sin()).sum::<f64>();
                    let cos_yaw = recovery_samples.iter().map(|p| p.2.cos()).sum::<f64>();
                    (x, y, sin_yaw.atan2(cos_yaw))
                } else {
                    (sensed.x, sensed.y, sensed.yaw)
                };
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
            let recovery_margin = RADIUS
                + 0.1
                + 2.0 * 2.0_f64.sqrt() * effective_position_sigma
                + drift_speed * observation_age;
            let recovery_bounds = [recovery_x, recovery_y, preview_end.x, preview_end.y]
                .iter()
                .all(|p| (recovery_margin..=(50.0 - recovery_margin)).contains(p));
            let mut resumed = false;
            let recovery_ready = observation_age <= 1.5
                && sensed.v.abs() <= 0.02
                && recovery_clearance > recovery_margin
                && recovery_bounds
                && (!averaged_recovery || recovery_sample_count >= 10);
            if loss_aware && safety_paused && !joint_recovery {
                recovery_evidence = if recovery_ready {
                    recovery_evidence + DT
                } else {
                    0.0
                };
                if recovery_evidence + 1e-9 >= 1.0 {
                    safety_paused = false;
                    recovery_evidence = 0.0;
                    resume_count += 1;
                    recovery_limit_until = observation_time + 5.0;
                    resumed = true;
                }
            }
            let sensed_goal_distance = Point2D::new(sensed.x, sensed.y).distance(&goal);
            let alpha = DT / (0.25 + DT);
            filtered_goal_distance += alpha * (sensed_goal_distance - filtered_goal_distance);
            if stop_hold {
                if !stop_latched {
                    stop_evidence = if filtered_goal_distance <= 0.2 {
                        stop_evidence + DT
                    } else {
                        0.0
                    };
                    if stop_evidence + 1e-9 >= 0.2 {
                        stop_latched = true;
                        latch_count += 1;
                        resume_evidence = 0.0;
                    }
                } else {
                    resume_evidence = if filtered_goal_distance > 0.5 {
                        resume_evidence + DT
                    } else {
                        0.0
                    };
                    if resume_evidence + 1e-9 >= 1.0 {
                        stop_latched = false;
                        stop_evidence = 0.0;
                    }
                }
            }
            let restart_limited = stop_hold && latch_count > 0;
            let mut target_speed = if stop_hold && stop_latched {
                0.0
            } else if terminal_slowdown {
                (0.5 * sensed_goal_distance)
                    .clamp(0.1, if restart_limited { 0.2 } else { TARGET_SPEED })
            } else {
                TARGET_SPEED
            };
            // In joint mode, probe the command that would follow recovery.
            // The actual paused command is still zero, applied below.
            let joint_pending = joint_recovery && safety_paused;
            if loss_aware && safety_paused && !joint_pending {
                target_speed = 0.0;
            }
            if loss_aware && (observation_time < recovery_limit_until || joint_pending) {
                target_speed = target_speed.min(0.4);
            }
            let mut steering = controller
                .compute_steering(&sensed)
                .clamp(-MAX_STEERING, MAX_STEERING);
            if loss_aware && safety_paused && !joint_pending {
                steering = 0.0;
            }
            let uncapped_target_speed = target_speed;
            let speed_margin = if anticipatory_speed {
                RADIUS
                    + 0.1
                    + 2.0 * 2.0_f64.sqrt() * effective_position_sigma
                    + drift_speed * observation_age
            } else {
                0.0
            };
            let speed_free_distance = if anticipatory_speed {
                speed_free_distance(sensed, steering, speed_margin, obs)
            } else {
                2.0
            };
            // Reserve 0.3 s of travel and use design deceleration 0.5. Actual
            // braking retains the existing bounded response (maximum 1.0).
            let speed_cap = if anticipatory_speed {
                (-0.15 + (0.15_f64.powi(2) + speed_free_distance).sqrt()).clamp(0.0, TARGET_SPEED)
            } else {
                TARGET_SPEED
            };
            let speed_braking = anticipatory_speed && sensed.v > speed_cap + 1e-9;
            target_speed = target_speed.min(speed_cap);
            let nominal_target_speed = target_speed;
            let nominal_steering = steering;
            let base_preview_margin = RADIUS
                // Match the recovery/speed-probe buffer in this optional arm.
                // Release keeps its existing additional hysteresis of 0.05.
                + if recovery_guard_buffer { 0.1 } else { 0.0 }
                + 2.0 * 2.0_f64.sqrt() * effective_position_sigma
                + drift_speed * observation_age;
            // A flat +0.1 entry buffer with the old release rule would invert
            // hysteresis at rest. Taper the entry reserve to zero with speed.
            let guard_entry_extra = if motion_entry_buffer {
                0.1 * (sensed.v.abs() / 0.4).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let preview_margin = base_preview_margin + guard_entry_extra;
            let guard_release_margin = base_preview_margin + 0.05;
            if consistent_restart {
                consistency.observe(
                    sensed,
                    guard_active,
                    ObservationEvent {
                        time: observation_time,
                        correction,
                        received: correction_received,
                        precision_activation,
                    },
                );
            }
            let consistency_ready = consistent_restart && consistency.ready(observation_time);
            let pre_sample_count = if consistency.previous.is_some() {
                25
            } else {
                0
            };
            let stationary_sample_count = consistency.samples.len();
            let consistency_age = consistency
                .epoch
                .map_or(-1.0, |epoch| observation_time - epoch);
            let (position_delta, yaw_delta_rad) = consistency.deltas();
            let preview_pose = if consistency_ready {
                let (x, y, yaw) = mean_pose(&consistency.samples);
                VehicleState::new(x, y, yaw, sensed.v, WHEELBASE)
            } else if motion_filter {
                pose_filter.observe(
                    sensed,
                    previous_steering,
                    correction || precision_activation,
                )
            } else {
                sensed
            };
            let motion_filter_sample_count = pose_filter.samples.len();
            let (preview_pose_x, preview_pose_y, preview_pose_yaw) =
                (preview_pose.x, preview_pose.y, preview_pose.yaw);
            let (stop_preview_clearance_lower_bound, terminal_recovery_clearance) =
                if stopping_guard {
                    stopping_preview(
                        preview_pose,
                        target_speed,
                        steering,
                        preview_margin,
                        obs,
                        recovery_preview,
                    )
                } else {
                    (0.0, 0.0)
                };
            // Express the terminal line's extra 0.1 recovery buffer in the
            // guard's clearance units, so the existing hysteresis and timers
            // require both stopping and recovery clearance simultaneously.
            let preview_clearance_lower_bound = if recovery_preview {
                stop_preview_clearance_lower_bound.min(terminal_recovery_clearance - 0.1)
            } else {
                stop_preview_clearance_lower_bound
            };
            let mut guard_triggered = false;
            let mut guard_released = false;
            if stopping_guard {
                if !guard_active && preview_clearance_lower_bound <= preview_margin {
                    guard_active = true;
                    guard_steering = steering;
                    guard_release_evidence = 0.0;
                    guard_triggered = true;
                } else if guard_active && !joint_pending {
                    let ready = sensed.v.abs() <= 0.02
                        && (!consistent_restart || consistency_ready)
                        && preview_clearance_lower_bound > guard_release_margin;
                    guard_release_evidence = if ready {
                        guard_release_evidence + DT
                    } else {
                        0.0
                    };
                    if guard_release_evidence + 1e-9 >= 0.5 {
                        guard_active = false;
                        guard_release_evidence = 0.0;
                        guard_released = true;
                    }
                }
                if joint_pending {
                    // The old recovery line, freshness and stationary sample
                    // requirements remain necessary. Confirm the nonzero
                    // post-release command against the guard at the same time.
                    let ready = recovery_ready
                        && nominal_target_speed > 0.0
                        && preview_clearance_lower_bound > guard_release_margin;
                    recovery_evidence = if ready { recovery_evidence + DT } else { 0.0 };
                    guard_release_evidence = 0.0;
                    if recovery_evidence + 1e-9 >= 1.0 {
                        safety_paused = false;
                        recovery_evidence = 0.0;
                        resume_count += 1;
                        recovery_limit_until = observation_time + 5.0;
                        resumed = true;
                        guard_released = guard_active;
                        guard_active = false;
                    }
                }
                if safety_paused {
                    target_speed = 0.0;
                    steering = 0.0;
                }
                if guard_active {
                    target_speed = 0.0;
                    if !safety_paused {
                        steering = guard_steering;
                    }
                }
            }
            if safety_paused {
                paused_time_s += DT;
            }
            let acceleration = if (stop_hold && stop_latched)
                || (loss_aware && safety_paused)
                || guard_active
                || speed_braking
            {
                (-4.0 * state.v).clamp(-1.0, 0.0)
            } else {
                controller.compute_acceleration(target_speed, state.v)
            };
            state.update(acceleration, steering, DT);
            previous_steering = steering;
            state.yaw = state.yaw.sin().atan2(state.yaw.cos());
            assert!([state.x, state.y, state.yaw, state.v]
                .iter()
                .all(|v| v.is_finite()));
            let to = Point2D::new(state.x, state.y);
            let step_clearance = obs
                .iter()
                .map(|&p| point_segment_distance(p, from, to))
                .fold(f64::INFINITY, f64::min);
            clearance = clearance.min(step_clearance);
            travelled += from.distance(&to);
            let error = distance_to_path(to, &path);
            max_error = max_error.max(error);
            rms_sum += (previous_error * previous_error + error * error) * 0.5 * DT;
            previous_error = error;
            time = step as f64 * DT;
            let actual_goal_distance = to.distance(&goal);
            if actual_goal_distance < GOAL_THRESHOLD && first_goal_entry_s < 0.0 {
                first_goal_entry_s = time;
            }
            if stop_hold
                && stop_latched
                && actual_goal_distance < GOAL_THRESHOLD
                && state.v.abs() <= 0.02
            {
                hold_time_s += DT;
                max_hold_distance = max_hold_distance.max(actual_goal_distance);
            } else {
                hold_time_s = 0.0;
                max_hold_distance = 0.0;
            }
            writeln!(tracks, "{scenario},{planner},{time:.6},{:.9},{:.9},{:.9},{:.9},{error:.9},{step_clearance:.9},{noise_x:.9},{noise_y:.9},{noise_yaw:.9},{bias_x:.9},{bias_y:.9},{bias_yaw:.9},{correction},{target_speed:.9},{sensed_goal_distance:.9},{filtered_goal_distance:.9},{stop_latched},{hold_time_s:.6},{restart_limited},{precision_active},{effective_position_sigma:.9},{effective_yaw_sigma:.9},{precision_activation},{observation_available},{correction_received},{loss_uniform:.12},{observation_age:.6},{safety_paused},{recovery_evidence:.6},{recovery_clearance:.9},{resumed},{steering:.9},{acceleration:.9},{recovery_x:.9},{recovery_y:.9},{recovery_yaw:.9},{recovery_sample_count},{:.12},{:.12},{:.12},{noise_rho:.12},{guard_active},{guard_triggered},{guard_released},{guard_release_evidence:.6},{preview_clearance_lower_bound:.9},{preview_margin:.9},{nominal_target_speed:.9},{nominal_steering:.9},{consistency_ready},{pre_sample_count},{stationary_sample_count},{consistency_age:.6},{position_delta:.9},{yaw_delta_rad:.9},{preview_pose_x:.9},{preview_pose_y:.9},{preview_pose_yaw:.9},{speed_free_distance:.9},{speed_cap:.9},{speed_margin:.9},{speed_braking},{uncapped_target_speed:.9},{guard_release_margin:.9},{guard_entry_extra:.9},{motion_filter_sample_count},{recovery_preview},{stop_preview_clearance_lower_bound:.9},{terminal_recovery_clearance:.9}", state.x, state.y, state.yaw, state.v, innovations[0], innovations[1], innovations[2])?;
            collision = step_clearance <= RADIUS;
            reached = controller.is_goal_reached(&state);
            if collision {
                status = "collision";
                break;
            }
            if !(0.0..=50.0).contains(&state.x) || !(0.0..=50.0).contains(&state.y) {
                status = "out_of_bounds";
                break;
            }
            if stop_hold {
                if hold_time_s + 1e-9 >= 10.0 {
                    held = true;
                    status = "held";
                    break;
                }
            } else if reached {
                status = "reached";
                break;
            }
        }
        // Contact takes priority over goal proximity for a valid arrival.
        reached = !collision
            && if stop_hold {
                first_goal_entry_s >= 0.0
            } else {
                reached && status == "reached"
            };
        let goal_distance = Point2D::new(state.x, state.y).distance(&goal);
        let rms = (rms_sum / time).sqrt();
        writeln!(summary, "{scenario},{planner},{lookahead},{status},{reached},{collision},{time:.6},{travelled:.9},{goal_distance:.9},{rms:.9},{max_error:.9},{clearance:.9},{:.9},{dense_count},{held},{hold_time_s:.6},{first_goal_entry_s:.6},{:.9},{max_hold_distance:.9},{latch_count},{pause_count},{resume_count},{paused_time_s:.6}", clearance-RADIUS, state.v)?;
        println!("{scenario}/{planner}: {status}, time={time:.2}s, goal_distance={goal_distance:.3}, RMS={rms:.3}, min_clearance={clearance:.3}");
    }
    tracks.flush()?;
    summary.flush()?;
    Ok(())
}
