//! BranchOut-lite multimodal driving planner.
//!
//! This is a deterministic 2-D reproduction slice of BranchOut's core
//! multimodal planning idea: emit multiple plausible driving trajectories with
//! mixture weights, then evaluate distributional coverage instead of only a
//! single ground-truth path.

use rust_robotics_core::{RoboticsError, RoboticsResult};

const EPS: f64 = 1e-9;

/// Ego state on a lane-aligned 2-D road.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BranchOutPose2D {
    pub x: f64,
    pub y: f64,
    pub speed: f64,
}

impl BranchOutPose2D {
    pub fn new(x: f64, y: f64, speed: f64) -> Self {
        Self { x, y, speed }
    }
}

/// Circular traffic or road obstacle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BranchOutObstacle2D {
    pub x: f64,
    pub y: f64,
    pub radius: f64,
}

impl BranchOutObstacle2D {
    pub fn new(x: f64, y: f64, radius: f64) -> Self {
        Self { x, y, radius }
    }
}

/// Coarse driving command/mode used by the compact GMM-like head.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BranchOutDecisionMode2D {
    KeepLane,
    Yield,
    LaneChangeLeft,
    LaneChangeRight,
}

impl BranchOutDecisionMode2D {
    pub fn label(self) -> &'static str {
        match self {
            Self::KeepLane => "keep-lane",
            Self::Yield => "yield",
            Self::LaneChangeLeft => "lane-change-left",
            Self::LaneChangeRight => "lane-change-right",
        }
    }
}

/// Lane-level driving scene for a compact multimodal planner.
#[derive(Debug, Clone, PartialEq)]
pub struct BranchOutDrivingScene2D {
    pub start: BranchOutPose2D,
    pub lane_width: f64,
    pub lane_count_each_side: i32,
    pub route_length: f64,
    pub desired_speed: f64,
    pub obstacles: Vec<BranchOutObstacle2D>,
}

impl BranchOutDrivingScene2D {
    pub fn simple_overtake() -> Self {
        Self {
            start: BranchOutPose2D::new(0.0, 0.0, 2.2),
            lane_width: 1.2,
            lane_count_each_side: 1,
            route_length: 9.0,
            desired_speed: 2.2,
            obstacles: vec![BranchOutObstacle2D::new(4.1, 0.0, 0.42)],
        }
    }

    /// Overtake scene with enough lateral room that a lane change clears the
    /// stalled obstacle (the closed-loop planner prefers to overtake here).
    pub fn wide_overtake() -> Self {
        Self {
            start: BranchOutPose2D::new(0.0, 0.0, 2.2),
            lane_width: 1.6,
            lane_count_each_side: 1,
            route_length: 9.0,
            desired_speed: 2.2,
            obstacles: vec![BranchOutObstacle2D::new(4.1, 0.0, 0.42)],
        }
    }

    /// Single-lane blocked scene with no room to pass, so the closed-loop
    /// planner must yield behind the obstacle rather than overtake.
    pub fn forced_yield() -> Self {
        Self {
            start: BranchOutPose2D::new(0.0, 0.0, 2.2),
            lane_width: 1.2,
            lane_count_each_side: 0,
            route_length: 9.0,
            desired_speed: 2.2,
            obstacles: vec![BranchOutObstacle2D::new(4.1, 0.0, 0.42)],
        }
    }

    pub fn lane_center(&self, lane_index: i32) -> f64 {
        lane_index as f64 * self.lane_width
    }

    pub fn nearest_lane_index(&self, y: f64) -> i32 {
        (y / self.lane_width).round().clamp(
            -(self.lane_count_each_side as f64),
            self.lane_count_each_side as f64,
        ) as i32
    }
}

/// Planner configuration for BranchOut-lite.
#[derive(Debug, Clone, PartialEq)]
pub struct BranchOutPlannerConfig2D {
    pub horizon_steps: usize,
    pub dt: f64,
    pub ego_radius: f64,
    pub probability_temperature: f64,
    pub progress_weight: f64,
    pub collision_weight: f64,
    pub lane_weight: f64,
    pub comfort_weight: f64,
    pub route_weight: f64,
    pub modes: Vec<BranchOutDecisionMode2D>,
}

impl Default for BranchOutPlannerConfig2D {
    fn default() -> Self {
        Self {
            horizon_steps: 28,
            dt: 0.12,
            ego_radius: 0.32,
            probability_temperature: 4.0,
            progress_weight: 1.4,
            collision_weight: 80.0,
            lane_weight: 12.0,
            comfort_weight: 0.35,
            route_weight: 0.12,
            modes: vec![
                BranchOutDecisionMode2D::KeepLane,
                BranchOutDecisionMode2D::Yield,
                BranchOutDecisionMode2D::LaneChangeLeft,
                BranchOutDecisionMode2D::LaneChangeRight,
            ],
        }
    }
}

/// One mode trajectory with a GMM-like mixture probability.
#[derive(Debug, Clone, PartialEq)]
pub struct BranchOutTrajectory2D {
    pub mode: BranchOutDecisionMode2D,
    pub probability: f64,
    pub cost: f64,
    pub poses: Vec<BranchOutPose2D>,
    pub collision_risk: f64,
    pub comfort_cost: f64,
    pub route_completion: f64,
}

impl BranchOutTrajectory2D {
    pub fn final_pose(&self) -> BranchOutPose2D {
        *self
            .poses
            .last()
            .expect("validated BranchOut trajectory is non-empty")
    }
}

/// Planner result with one trajectory per decision mode.
#[derive(Debug, Clone, PartialEq)]
pub struct BranchOutPlan2D {
    pub trajectories: Vec<BranchOutTrajectory2D>,
}

impl BranchOutPlan2D {
    pub fn best(&self) -> Option<&BranchOutTrajectory2D> {
        self.trajectories
            .iter()
            .max_by(|a, b| a.probability.total_cmp(&b.probability))
    }

    pub fn probability_sum(&self) -> f64 {
        self.trajectories
            .iter()
            .map(|trajectory| trajectory.probability)
            .sum()
    }
}

/// Multimodal evaluation metrics inspired by BranchOut's distributional focus.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BranchOutMultimodalMetrics2D {
    pub mode_count: usize,
    pub mean_pairwise_final_distance: f64,
    pub mean_pairwise_frechet: f64,
    pub min_ground_truth_frechet: f64,
    pub negative_log_likelihood: f64,
    pub speed_jsd: f64,
    pub expected_route_completion: f64,
}

/// Configuration for a receding-horizon closed-loop BranchOut rollout.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BranchOutClosedLoopConfig2D {
    /// Number of closed-loop control steps to execute.
    pub steps: usize,
    /// Time-to-collision threshold; steps below it count as risky.
    pub ttc_threshold: f64,
    /// Route fraction at or above which the goal counts as reached.
    pub goal_completion: f64,
    /// Maximum lateral speed used when tracking the selected mode's lane.
    pub max_lateral_speed: f64,
}

impl Default for BranchOutClosedLoopConfig2D {
    fn default() -> Self {
        Self {
            steps: 40,
            ttc_threshold: 1.5,
            goal_completion: 0.95,
            max_lateral_speed: 0.9,
        }
    }
}

/// Closed-loop driving metrics from a receding-horizon BranchOut rollout.
#[derive(Debug, Clone, PartialEq)]
pub struct BranchOutClosedLoopMetrics2D {
    pub steps: usize,
    pub route_completion: f64,
    pub reached_goal: bool,
    pub collision_steps: usize,
    pub no_collision_rate: f64,
    pub min_clearance: f64,
    pub mean_comfort_cost: f64,
    pub min_time_to_collision: f64,
    pub risky_ttc_steps: usize,
    /// Realized closed-loop ego path (start pose plus one pose per step).
    pub executed_path: Vec<BranchOutPose2D>,
    /// Decision mode selected at each control step.
    pub mode_sequence: Vec<BranchOutDecisionMode2D>,
}

/// Deterministic multimodal planner over coarse driving modes.
#[derive(Debug, Clone, PartialEq)]
pub struct BranchOutPlanner2D {
    config: BranchOutPlannerConfig2D,
}

impl BranchOutPlanner2D {
    pub fn new(config: BranchOutPlannerConfig2D) -> RoboticsResult<Self> {
        validate_config(&config)?;
        Ok(Self { config })
    }

    pub fn config(&self) -> &BranchOutPlannerConfig2D {
        &self.config
    }

    pub fn plan(&self, scene: &BranchOutDrivingScene2D) -> RoboticsResult<BranchOutPlan2D> {
        validate_scene(scene)?;
        let mut trajectories = self
            .config
            .modes
            .iter()
            .map(|&mode| self.rollout_mode(scene, mode))
            .collect::<RoboticsResult<Vec<_>>>()?;
        assign_mixture_probabilities(&mut trajectories, self.config.probability_temperature)?;
        Ok(BranchOutPlan2D { trajectories })
    }

    pub fn evaluate_multimodal(
        &self,
        plan: &BranchOutPlan2D,
        ground_truths: &[Vec<BranchOutPose2D>],
    ) -> RoboticsResult<BranchOutMultimodalMetrics2D> {
        validate_plan(plan)?;
        if ground_truths.is_empty() {
            return Err(RoboticsError::InvalidParameter(
                "BranchOut ground_truths must be non-empty".to_string(),
            ));
        }
        for ground_truth in ground_truths {
            validate_poses(ground_truth)?;
        }

        let mean_pairwise_final_distance = mean_pairwise_final_distance(&plan.trajectories);
        let mean_pairwise_frechet = mean_pairwise_frechet(&plan.trajectories);
        let min_ground_truth_frechet = ground_truths
            .iter()
            .map(|gt| {
                plan.trajectories
                    .iter()
                    .map(|trajectory| discrete_frechet(&trajectory.poses, gt))
                    .fold(f64::INFINITY, f64::min)
            })
            .sum::<f64>()
            / ground_truths.len() as f64;
        let negative_log_likelihood = trajectory_set_nll(&plan.trajectories, ground_truths, 0.75);
        let speed_jsd = speed_jsd(&plan.trajectories, ground_truths, 8, 4.0);
        let expected_route_completion = plan
            .trajectories
            .iter()
            .map(|trajectory| trajectory.probability * trajectory.route_completion)
            .sum();

        Ok(BranchOutMultimodalMetrics2D {
            mode_count: plan.trajectories.len(),
            mean_pairwise_final_distance,
            mean_pairwise_frechet,
            min_ground_truth_frechet,
            negative_log_likelihood,
            speed_jsd,
            expected_route_completion,
        })
    }

    /// Run a receding-horizon closed-loop rollout: at every control step
    /// re-plan from the current ego pose, commit the first step of the
    /// highest-probability mode, advance the (optionally moving) obstacles, and
    /// accumulate closed-loop driving metrics.
    ///
    /// `obstacle_velocities` must match `scene.obstacles` in length; pass zeros
    /// for static traffic.
    pub fn simulate_closed_loop(
        &self,
        scene: &BranchOutDrivingScene2D,
        obstacle_velocities: &[(f64, f64)],
        config: BranchOutClosedLoopConfig2D,
    ) -> RoboticsResult<BranchOutClosedLoopMetrics2D> {
        validate_scene(scene)?;
        if obstacle_velocities.len() != scene.obstacles.len() {
            return Err(RoboticsError::InvalidParameter(
                "BranchOut obstacle_velocities length must match scene.obstacles".to_string(),
            ));
        }
        for &(vx, vy) in obstacle_velocities {
            if !vx.is_finite() || !vy.is_finite() {
                return Err(RoboticsError::InvalidParameter(
                    "BranchOut obstacle velocity must be finite".to_string(),
                ));
            }
        }
        if config.steps == 0
            || !config.ttc_threshold.is_finite()
            || config.ttc_threshold <= 0.0
            || !config.goal_completion.is_finite()
            || !(0.0..=1.0).contains(&config.goal_completion)
            || !config.max_lateral_speed.is_finite()
            || config.max_lateral_speed <= 0.0
        {
            return Err(RoboticsError::InvalidParameter(
                "BranchOut closed-loop config steps/ttc_threshold/goal_completion/max_lateral_speed are invalid"
                    .to_string(),
            ));
        }

        let dt = self.config.dt;
        let mut ego = scene.start;
        let mut obstacles = scene.obstacles.clone();
        let mut executed_path = vec![ego];
        let mut mode_sequence = Vec::with_capacity(config.steps);

        let mut collision_steps = 0;
        let mut min_clearance = f64::INFINITY;
        let mut min_time_to_collision = f64::INFINITY;
        let mut risky_ttc_steps = 0;

        for _ in 0..config.steps {
            let current_scene = BranchOutDrivingScene2D {
                start: ego,
                obstacles: obstacles.clone(),
                ..scene.clone()
            };
            let plan = self.plan(&current_scene)?;
            let mode = plan
                .best()
                .expect("validated plan always has at least one trajectory")
                .mode;

            // Receding-horizon commit: BranchOut selects the mode; the closed
            // loop tracks that mode's target lane with a bounded lateral rate
            // and a first-order speed law (the per-mode rollout's lateral curve
            // is back-loaded, so replaying only its first step would barely
            // steer). This keeps lane changes physically realizable.
            let start_lane = current_scene.nearest_lane_index(ego.y);
            let target_y =
                current_scene.lane_center(mode_target_lane(&current_scene, start_lane, mode));
            let desired_speed = match mode {
                BranchOutDecisionMode2D::Yield => yield_speed(&current_scene, ego.x),
                _ => current_scene.desired_speed,
            };
            let max_lateral_step = config.max_lateral_speed * dt;
            let mut next = ego;
            next.speed += 0.35 * (desired_speed - ego.speed);
            next.x += next.speed * dt;
            next.y += (target_y - ego.y).clamp(-max_lateral_step, max_lateral_step);
            let ego_velocity = ((next.x - ego.x) / dt, (next.y - ego.y) / dt);

            // Advance obstacles, then evaluate clearance/TTC at the committed
            // state against the obstacles' new positions.
            for (obstacle, &(vx, vy)) in obstacles.iter_mut().zip(obstacle_velocities) {
                obstacle.x += vx * dt;
                obstacle.y += vy * dt;
            }

            let mut step_min_ttc = f64::INFINITY;
            let mut step_min_clearance = f64::INFINITY;
            for (obstacle, &velocity) in obstacles.iter().zip(obstacle_velocities) {
                let radius_sum = obstacle.radius + self.config.ego_radius;
                let clearance =
                    point_distance((next.x, next.y), (obstacle.x, obstacle.y)) - radius_sum;
                step_min_clearance = step_min_clearance.min(clearance);
                let ttc = time_to_collision(
                    (next.x, next.y),
                    ego_velocity,
                    (obstacle.x, obstacle.y),
                    velocity,
                    radius_sum,
                );
                step_min_ttc = step_min_ttc.min(ttc);
            }
            min_clearance = min_clearance.min(step_min_clearance);
            if step_min_clearance < 0.0 {
                collision_steps += 1;
            }
            min_time_to_collision = min_time_to_collision.min(step_min_ttc);
            if step_min_ttc < config.ttc_threshold {
                risky_ttc_steps += 1;
            }

            ego = next;
            executed_path.push(ego);
            mode_sequence.push(mode);
        }

        if obstacles.is_empty() {
            min_clearance = f64::INFINITY;
        }
        let steps = config.steps;
        let route_completion = (ego.x / scene.route_length).clamp(0.0, 1.0);
        let mean_comfort_cost = closed_loop_comfort(&executed_path, dt);

        Ok(BranchOutClosedLoopMetrics2D {
            steps,
            route_completion,
            reached_goal: route_completion >= config.goal_completion && collision_steps == 0,
            collision_steps,
            no_collision_rate: (steps - collision_steps) as f64 / steps as f64,
            min_clearance,
            mean_comfort_cost,
            min_time_to_collision,
            risky_ttc_steps,
            executed_path,
            mode_sequence,
        })
    }

    fn rollout_mode(
        &self,
        scene: &BranchOutDrivingScene2D,
        mode: BranchOutDecisionMode2D,
    ) -> RoboticsResult<BranchOutTrajectory2D> {
        let start_lane = scene.nearest_lane_index(scene.start.y);
        let target_lane = mode_target_lane(scene, start_lane, mode);
        let target_y = scene.lane_center(target_lane);
        let mut poses = Vec::with_capacity(self.config.horizon_steps + 1);
        let mut pose = scene.start;
        poses.push(pose);

        for step in 1..=self.config.horizon_steps {
            let phase = step as f64 / self.config.horizon_steps as f64;
            let smooth = smoothstep(phase);
            let desired_speed = match mode {
                BranchOutDecisionMode2D::Yield => yield_speed(scene, pose.x),
                _ => scene.desired_speed,
            };
            pose.speed += 0.35 * (desired_speed - pose.speed);
            pose.x += pose.speed * self.config.dt;
            pose.y = scene.start.y + (target_y - scene.start.y) * smooth;
            poses.push(pose);
        }

        let (collision_risk, lane_penalty, comfort_cost) = self.cost_terms(scene, &poses);
        let final_pose = *poses
            .last()
            .expect("BranchOut rollout always contains start and horizon poses");
        let progress_error = (scene.route_length - final_pose.x).max(0.0);
        let route_completion = (final_pose.x / scene.route_length).clamp(0.0, 1.0);
        let target_route_y = scene.lane_center(start_lane);
        let route_deviation = (final_pose.y - target_route_y).abs();
        let cost = self.config.progress_weight * progress_error
            + self.config.collision_weight * collision_risk
            + self.config.lane_weight * lane_penalty
            + self.config.comfort_weight * comfort_cost
            + self.config.route_weight * route_deviation;

        Ok(BranchOutTrajectory2D {
            mode,
            probability: 0.0,
            cost,
            poses,
            collision_risk,
            comfort_cost,
            route_completion,
        })
    }

    fn cost_terms(
        &self,
        scene: &BranchOutDrivingScene2D,
        poses: &[BranchOutPose2D],
    ) -> (f64, f64, f64) {
        let mut collision_risk = 0.0;
        let mut lane_penalty = 0.0;
        let mut comfort_cost = 0.0;
        let road_half_width = (scene.lane_count_each_side as f64 + 0.5) * scene.lane_width;

        for pose in poses {
            for obstacle in &scene.obstacles {
                let clearance = point_distance((pose.x, pose.y), (obstacle.x, obstacle.y))
                    - obstacle.radius
                    - self.config.ego_radius;
                if clearance < 0.0 {
                    collision_risk += (1.0 - clearance).powi(2);
                } else {
                    collision_risk += 0.03 / (clearance + 0.3);
                }
            }
            if pose.y.abs() > road_half_width {
                lane_penalty += (pose.y.abs() - road_half_width).powi(2);
            }
        }

        for window in poses.windows(3) {
            let ay0 = window[1].y - window[0].y;
            let ay1 = window[2].y - window[1].y;
            comfort_cost += (ay1 - ay0).powi(2) / (self.config.dt * self.config.dt);
            comfort_cost += (window[2].speed - window[1].speed).powi(2);
        }

        let norm = poses.len() as f64;
        (
            collision_risk / norm,
            lane_penalty / norm,
            comfort_cost / norm,
        )
    }
}

fn assign_mixture_probabilities(
    trajectories: &mut [BranchOutTrajectory2D],
    temperature: f64,
) -> RoboticsResult<()> {
    let min_cost = trajectories
        .iter()
        .map(|trajectory| trajectory.cost)
        .fold(f64::INFINITY, f64::min);
    let mut weight_sum = 0.0;
    for trajectory in trajectories.iter_mut() {
        trajectory.probability = (-(trajectory.cost - min_cost) / temperature).exp();
        weight_sum += trajectory.probability;
    }
    if weight_sum <= 0.0 || !weight_sum.is_finite() {
        return Err(RoboticsError::PlanningError(
            "BranchOut mixture weights collapsed".to_string(),
        ));
    }
    for trajectory in trajectories {
        trajectory.probability /= weight_sum;
    }
    Ok(())
}

fn mode_target_lane(
    scene: &BranchOutDrivingScene2D,
    start_lane: i32,
    mode: BranchOutDecisionMode2D,
) -> i32 {
    match mode {
        BranchOutDecisionMode2D::KeepLane | BranchOutDecisionMode2D::Yield => start_lane,
        BranchOutDecisionMode2D::LaneChangeLeft => (start_lane + 1).min(scene.lane_count_each_side),
        BranchOutDecisionMode2D::LaneChangeRight => {
            (start_lane - 1).max(-scene.lane_count_each_side)
        }
    }
}

fn nearest_obstacle_x(scene: &BranchOutDrivingScene2D) -> Option<f64> {
    scene
        .obstacles
        .iter()
        .filter(|obstacle| obstacle.x >= scene.start.x)
        .map(|obstacle| obstacle.x)
        .min_by(f64::total_cmp)
}

fn yield_speed(scene: &BranchOutDrivingScene2D, ego_x: f64) -> f64 {
    let Some(obstacle_x) = nearest_obstacle_x(scene) else {
        return scene.desired_speed;
    };
    let stop_x = obstacle_x - 1.25;
    if ego_x >= stop_x {
        0.0
    } else {
        let distance_to_stop = (stop_x - ego_x).max(0.0);
        (0.75 * scene.desired_speed).min(distance_to_stop)
    }
}

fn mean_pairwise_final_distance(trajectories: &[BranchOutTrajectory2D]) -> f64 {
    let mut sum = 0.0;
    let mut count = 0;
    for i in 0..trajectories.len() {
        for j in i + 1..trajectories.len() {
            let a = trajectories[i].final_pose();
            let b = trajectories[j].final_pose();
            sum += point_distance((a.x, a.y), (b.x, b.y));
            count += 1;
        }
    }
    if count == 0 {
        0.0
    } else {
        sum / count as f64
    }
}

fn mean_pairwise_frechet(trajectories: &[BranchOutTrajectory2D]) -> f64 {
    let mut sum = 0.0;
    let mut count = 0;
    for i in 0..trajectories.len() {
        for j in i + 1..trajectories.len() {
            sum += discrete_frechet(&trajectories[i].poses, &trajectories[j].poses);
            count += 1;
        }
    }
    if count == 0 {
        0.0
    } else {
        sum / count as f64
    }
}

fn trajectory_set_nll(
    trajectories: &[BranchOutTrajectory2D],
    ground_truths: &[Vec<BranchOutPose2D>],
    sigma: f64,
) -> f64 {
    let variance = sigma * sigma;
    let normalizer = 2.0 * std::f64::consts::PI * variance;
    let mut nll = 0.0;
    for ground_truth in ground_truths {
        let gt_final = *ground_truth
            .last()
            .expect("validated ground-truth trajectory is non-empty");
        let likelihood = trajectories
            .iter()
            .map(|trajectory| {
                let final_pose = trajectory.final_pose();
                let distance_sq =
                    squared_distance((final_pose.x, final_pose.y), (gt_final.x, gt_final.y));
                trajectory.probability * (-0.5 * distance_sq / variance).exp() / normalizer
            })
            .sum::<f64>()
            .max(EPS);
        nll -= likelihood.ln();
    }
    nll / ground_truths.len() as f64
}

fn speed_jsd(
    trajectories: &[BranchOutTrajectory2D],
    ground_truths: &[Vec<BranchOutPose2D>],
    bins: usize,
    max_speed: f64,
) -> f64 {
    let mut predicted = vec![0.0; bins];
    for trajectory in trajectories {
        for pose in &trajectory.poses {
            predicted[speed_bin(pose.speed, bins, max_speed)] += trajectory.probability;
        }
    }
    normalize_distribution(&mut predicted);

    let mut truth = vec![0.0; bins];
    for trajectory in ground_truths {
        for pose in trajectory {
            truth[speed_bin(pose.speed, bins, max_speed)] += 1.0;
        }
    }
    normalize_distribution(&mut truth);

    let mixture = predicted
        .iter()
        .zip(&truth)
        .map(|(p, q)| 0.5 * (p + q))
        .collect::<Vec<_>>();
    0.5 * kl_divergence(&predicted, &mixture) + 0.5 * kl_divergence(&truth, &mixture)
}

fn speed_bin(speed: f64, bins: usize, max_speed: f64) -> usize {
    ((speed.clamp(0.0, max_speed) / max_speed) * bins as f64)
        .floor()
        .min((bins - 1) as f64) as usize
}

fn normalize_distribution(values: &mut [f64]) {
    let sum = values.iter().sum::<f64>();
    if sum > 0.0 {
        for value in values {
            *value /= sum;
        }
    }
}

fn kl_divergence(p: &[f64], q: &[f64]) -> f64 {
    p.iter()
        .zip(q)
        .filter(|(p, q)| **p > 0.0 && **q > 0.0)
        .map(|(p, q)| p * (p / q).ln())
        .sum()
}

fn discrete_frechet(a: &[BranchOutPose2D], b: &[BranchOutPose2D]) -> f64 {
    let mut ca = vec![vec![0.0; b.len()]; a.len()];
    for i in 0..a.len() {
        for j in 0..b.len() {
            let distance = point_distance((a[i].x, a[i].y), (b[j].x, b[j].y));
            ca[i][j] = if i == 0 && j == 0 {
                distance
            } else if i == 0 {
                ca[i][j - 1].max(distance)
            } else if j == 0 {
                ca[i - 1][j].max(distance)
            } else {
                ca[i - 1][j]
                    .min(ca[i - 1][j - 1])
                    .min(ca[i][j - 1])
                    .max(distance)
            };
        }
    }
    ca[a.len() - 1][b.len() - 1]
}

/// Time until the ego and obstacle disks (combined radius `radius_sum`) first
/// touch, given current positions and constant velocities. Returns `INFINITY`
/// when they are separating or never intersect; `0.0` when already overlapping.
fn time_to_collision(
    ego: (f64, f64),
    ego_velocity: (f64, f64),
    obstacle: (f64, f64),
    obstacle_velocity: (f64, f64),
    radius_sum: f64,
) -> f64 {
    let px = obstacle.0 - ego.0;
    let py = obstacle.1 - ego.1;
    let vx = obstacle_velocity.0 - ego_velocity.0;
    let vy = obstacle_velocity.1 - ego_velocity.1;
    let distance_sq = px * px + py * py;
    let radius_sq = radius_sum * radius_sum;
    if distance_sq <= radius_sq {
        return 0.0;
    }
    let a = vx * vx + vy * vy;
    if a <= EPS {
        return f64::INFINITY;
    }
    let b = 2.0 * (px * vx + py * vy);
    let c = distance_sq - radius_sq;
    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return f64::INFINITY;
    }
    let root = (-b - discriminant.sqrt()) / (2.0 * a);
    if root >= 0.0 {
        root
    } else {
        f64::INFINITY
    }
}

/// Mean closed-loop comfort cost: lateral jerk plus longitudinal acceleration
/// over the realized path, matching the per-mode comfort term.
fn closed_loop_comfort(path: &[BranchOutPose2D], dt: f64) -> f64 {
    if path.len() < 3 {
        return 0.0;
    }
    let mut comfort = 0.0;
    for window in path.windows(3) {
        let ay0 = window[1].y - window[0].y;
        let ay1 = window[2].y - window[1].y;
        comfort += (ay1 - ay0).powi(2) / (dt * dt);
        comfort += (window[2].speed - window[1].speed).powi(2);
    }
    comfort / (path.len() - 2) as f64
}

fn smoothstep(t: f64) -> f64 {
    let clamped = t.clamp(0.0, 1.0);
    clamped * clamped * (3.0 - 2.0 * clamped)
}

fn squared_distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    dx * dx + dy * dy
}

fn point_distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    squared_distance(a, b).sqrt()
}

fn validate_config(config: &BranchOutPlannerConfig2D) -> RoboticsResult<()> {
    if config.horizon_steps == 0 || config.modes.is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "BranchOut horizon_steps and modes must be non-empty".to_string(),
        ));
    }
    for (label, value) in [
        ("BranchOut dt", config.dt),
        ("BranchOut ego_radius", config.ego_radius),
        (
            "BranchOut probability_temperature",
            config.probability_temperature,
        ),
        ("BranchOut progress_weight", config.progress_weight),
        ("BranchOut collision_weight", config.collision_weight),
        ("BranchOut lane_weight", config.lane_weight),
        ("BranchOut comfort_weight", config.comfort_weight),
        ("BranchOut route_weight", config.route_weight),
    ] {
        if value < 0.0 || !value.is_finite() {
            return Err(RoboticsError::InvalidParameter(format!(
                "{label} must be finite and non-negative"
            )));
        }
    }
    if config.dt <= 0.0 || config.ego_radius <= 0.0 || config.probability_temperature <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "BranchOut dt, ego_radius, and probability_temperature must be positive".to_string(),
        ));
    }
    Ok(())
}

fn validate_scene(scene: &BranchOutDrivingScene2D) -> RoboticsResult<()> {
    validate_pose(scene.start)?;
    if scene.lane_width <= 0.0
        || !scene.lane_width.is_finite()
        || scene.route_length <= 0.0
        || !scene.route_length.is_finite()
        || scene.desired_speed <= 0.0
        || !scene.desired_speed.is_finite()
    {
        return Err(RoboticsError::InvalidParameter(
            "BranchOut scene lane_width, route_length, and desired_speed must be positive"
                .to_string(),
        ));
    }
    if scene.lane_count_each_side < 0 {
        return Err(RoboticsError::InvalidParameter(
            "BranchOut lane_count_each_side must be non-negative".to_string(),
        ));
    }
    for obstacle in &scene.obstacles {
        if !obstacle.x.is_finite() || !obstacle.y.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "BranchOut obstacle coordinates must be finite".to_string(),
            ));
        }
        if obstacle.radius <= 0.0 || !obstacle.radius.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "BranchOut obstacle radius must be positive".to_string(),
            ));
        }
    }
    Ok(())
}

fn validate_plan(plan: &BranchOutPlan2D) -> RoboticsResult<()> {
    if plan.trajectories.is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "BranchOut plan must contain trajectories".to_string(),
        ));
    }
    for trajectory in &plan.trajectories {
        validate_poses(&trajectory.poses)?;
        if trajectory.probability < 0.0
            || !trajectory.probability.is_finite()
            || !trajectory.cost.is_finite()
        {
            return Err(RoboticsError::InvalidParameter(
                "BranchOut trajectory probability and cost must be finite".to_string(),
            ));
        }
    }
    Ok(())
}

fn validate_poses(poses: &[BranchOutPose2D]) -> RoboticsResult<()> {
    if poses.is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "BranchOut trajectory poses must be non-empty".to_string(),
        ));
    }
    for &pose in poses {
        validate_pose(pose)?;
    }
    Ok(())
}

fn validate_pose(pose: BranchOutPose2D) -> RoboticsResult<()> {
    if !pose.x.is_finite() || !pose.y.is_finite() || !pose.speed.is_finite() || pose.speed < 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "BranchOut pose must be finite with non-negative speed".to_string(),
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn branchout_emits_multiple_modes_with_normalized_probabilities() {
        let planner = BranchOutPlanner2D::new(BranchOutPlannerConfig2D::default()).unwrap();
        let scene = BranchOutDrivingScene2D::simple_overtake();
        let plan = planner.plan(&scene).unwrap();

        assert_eq!(plan.trajectories.len(), 4);
        assert!((plan.probability_sum() - 1.0).abs() < 1e-9);
        assert!(plan.best().unwrap().probability > 0.25);
    }

    #[test]
    fn lane_change_modes_end_in_distinct_lanes() {
        let planner = BranchOutPlanner2D::new(BranchOutPlannerConfig2D::default()).unwrap();
        let scene = BranchOutDrivingScene2D::simple_overtake();
        let plan = planner.plan(&scene).unwrap();

        let left = plan
            .trajectories
            .iter()
            .find(|trajectory| trajectory.mode == BranchOutDecisionMode2D::LaneChangeLeft)
            .unwrap();
        let right = plan
            .trajectories
            .iter()
            .find(|trajectory| trajectory.mode == BranchOutDecisionMode2D::LaneChangeRight)
            .unwrap();

        assert!(left.final_pose().y > 0.9);
        assert!(right.final_pose().y < -0.9);
    }

    #[test]
    fn multimodal_metrics_reward_coverage() {
        let planner = BranchOutPlanner2D::new(BranchOutPlannerConfig2D::default()).unwrap();
        let scene = BranchOutDrivingScene2D::simple_overtake();
        let plan = planner.plan(&scene).unwrap();
        let ground_truths = plan
            .trajectories
            .iter()
            .filter(|trajectory| trajectory.mode != BranchOutDecisionMode2D::KeepLane)
            .map(|trajectory| trajectory.poses.clone())
            .collect::<Vec<_>>();
        let metrics = planner.evaluate_multimodal(&plan, &ground_truths).unwrap();

        assert_eq!(metrics.mode_count, 4);
        assert!(metrics.mean_pairwise_final_distance > 0.5);
        assert!(metrics.min_ground_truth_frechet < 0.1);
        assert!(metrics.negative_log_likelihood.is_finite());
        assert!(metrics.speed_jsd >= 0.0);
    }

    #[test]
    fn closed_loop_overtake_completes_route_without_collision() {
        let planner = BranchOutPlanner2D::new(BranchOutPlannerConfig2D::default()).unwrap();
        let scene = BranchOutDrivingScene2D::wide_overtake();
        let velocities = vec![(0.0, 0.0); scene.obstacles.len()];
        let metrics = planner
            .simulate_closed_loop(&scene, &velocities, BranchOutClosedLoopConfig2D::default())
            .unwrap();

        assert_eq!(metrics.executed_path.len(), metrics.steps + 1);
        assert_eq!(metrics.mode_sequence.len(), metrics.steps);
        assert_eq!(metrics.collision_steps, 0);
        assert_eq!(metrics.no_collision_rate, 1.0);
        assert!(metrics.min_clearance > 0.0);
        assert!(metrics.reached_goal);
        assert!(metrics.route_completion >= 0.95);
        assert!(metrics.min_time_to_collision > 0.0);
        assert!(metrics.mean_comfort_cost.is_finite());
    }

    #[test]
    fn closed_loop_yields_safely_when_blocked() {
        let planner = BranchOutPlanner2D::new(BranchOutPlannerConfig2D::default()).unwrap();
        let scene = BranchOutDrivingScene2D::forced_yield();
        let metrics = planner
            .simulate_closed_loop(
                &scene,
                &[(0.0, 0.0)],
                BranchOutClosedLoopConfig2D::default(),
            )
            .unwrap();

        // No room to pass: the ego stops behind the obstacle, so it never
        // reaches the goal but stays collision-free with positive clearance.
        assert_eq!(metrics.collision_steps, 0);
        assert!(!metrics.reached_goal);
        assert!(metrics.route_completion < 0.6);
        assert!(metrics.min_clearance > 0.0);
        assert!(metrics
            .mode_sequence
            .iter()
            .all(|&mode| mode == BranchOutDecisionMode2D::Yield));
    }

    #[test]
    fn closed_loop_is_deterministic() {
        let planner = BranchOutPlanner2D::new(BranchOutPlannerConfig2D::default()).unwrap();
        let scene = BranchOutDrivingScene2D::simple_overtake();
        let velocities = vec![(0.0, 0.0); scene.obstacles.len()];
        let config = BranchOutClosedLoopConfig2D::default();
        let first = planner
            .simulate_closed_loop(&scene, &velocities, config)
            .unwrap();
        let second = planner
            .simulate_closed_loop(&scene, &velocities, config)
            .unwrap();

        assert_eq!(first.executed_path, second.executed_path);
        assert_eq!(first.mode_sequence, second.mode_sequence);
        assert_eq!(first.collision_steps, second.collision_steps);
    }

    #[test]
    fn oncoming_obstacle_lowers_time_to_collision() {
        let planner = BranchOutPlanner2D::new(BranchOutPlannerConfig2D::default()).unwrap();
        // A blocker sits far ahead in the ego lane and drives toward the ego.
        let mut scene = BranchOutDrivingScene2D::simple_overtake();
        scene.obstacles = vec![BranchOutObstacle2D::new(8.5, 0.0, 0.42)];
        let static_metrics = planner
            .simulate_closed_loop(
                &scene,
                &[(0.0, 0.0)],
                BranchOutClosedLoopConfig2D::default(),
            )
            .unwrap();
        let oncoming_metrics = planner
            .simulate_closed_loop(
                &scene,
                &[(-1.6, 0.0)],
                BranchOutClosedLoopConfig2D::default(),
            )
            .unwrap();

        // An approaching obstacle must not raise the minimum time-to-collision.
        assert!(oncoming_metrics.min_time_to_collision <= static_metrics.min_time_to_collision);
        assert!(oncoming_metrics.min_time_to_collision.is_finite());
    }

    #[test]
    fn closed_loop_rejects_mismatched_velocities() {
        let planner = BranchOutPlanner2D::new(BranchOutPlannerConfig2D::default()).unwrap();
        let scene = BranchOutDrivingScene2D::simple_overtake();
        assert!(planner
            .simulate_closed_loop(&scene, &[], BranchOutClosedLoopConfig2D::default())
            .is_err());
    }

    #[test]
    fn time_to_collision_detects_closing_and_separating() {
        // Closing head-on along x: surface gap 4, closing speed 2 -> 2.0 s.
        let closing = time_to_collision((0.0, 0.0), (1.0, 0.0), (5.0, 0.0), (-1.0, 0.0), 1.0);
        assert!((closing - 2.0).abs() < 1e-9);
        // Separating: never collide.
        let separating = time_to_collision((0.0, 0.0), (-1.0, 0.0), (5.0, 0.0), (1.0, 0.0), 1.0);
        assert!(separating.is_infinite());
        // Already overlapping -> 0.
        let overlapping = time_to_collision((0.0, 0.0), (0.0, 0.0), (0.5, 0.0), (0.0, 0.0), 1.0);
        assert_eq!(overlapping, 0.0);
    }
}
