//! Conformal Prediction Safe Interval Path Planning (CP-SIPP).
//!
//! This module reproduces the global-planning slice from Liang et al.,
//! "Time-aware Motion Planning in Dynamic Environments with Conformal
//! Prediction" (L4DC 2026).  It converts predicted obstacle trajectories plus
//! calibration nonconformity scores into confidence-filtered safe intervals, then
//! delegates the compressed interval search to the existing SIPP implementation.

use rust_robotics_core::{RoboticsError, RoboticsResult};

use crate::sipp::{DynamicObstacle, Interval, SippConfig, SippPath, SippPlanner};

/// A predicted obstacle position at a discrete planning time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PredictedObstaclePoint {
    pub t: u64,
    pub x: f64,
    pub y: f64,
}

impl PredictedObstaclePoint {
    pub fn new(t: u64, x: f64, y: f64) -> Self {
        Self { t, x, y }
    }
}

/// A discrete predicted trajectory for one dynamic obstacle.
#[derive(Debug, Clone, PartialEq)]
pub struct PredictedObstacleTrajectory {
    pub samples: Vec<PredictedObstaclePoint>,
}

impl PredictedObstacleTrajectory {
    pub fn new(samples: Vec<PredictedObstaclePoint>) -> Self {
        Self { samples }
    }

    fn position_at(&self, t: u64) -> Option<(f64, f64)> {
        let mut lower: Option<&PredictedObstaclePoint> = None;
        let mut upper: Option<&PredictedObstaclePoint> = None;

        for sample in &self.samples {
            if sample.t == t {
                return Some((sample.x, sample.y));
            }
            if sample.t < t && lower.map_or(true, |current| sample.t > current.t) {
                lower = Some(sample);
            }
            if sample.t > t && upper.map_or(true, |current| sample.t < current.t) {
                upper = Some(sample);
            }
        }

        let (lower, upper) = (lower?, upper?);
        let span = (upper.t - lower.t) as f64;
        let alpha = (t - lower.t) as f64 / span;
        Some((
            lower.x + alpha * (upper.x - lower.x),
            lower.y + alpha * (upper.y - lower.y),
        ))
    }
}

/// Configuration for CP-SIPP over a discrete grid and finite time horizon.
///
/// `calibration_errors_by_time[t]` contains historical nonconformity scores for
/// prediction horizon `t`. A grid cell is safe at time `t` when the empirical
/// confidence induced by its distance to the nearest predicted obstacle is at
/// least `required_confidence`.
#[derive(Debug, Clone)]
pub struct ConformalSippConfig {
    pub width: i32,
    pub height: i32,
    /// Static obstacle map: `true` means blocked.
    pub obstacle_map: Vec<Vec<bool>>,
    /// Predicted obstacle centers indexed by discrete time.
    pub predicted_obstacles: Vec<PredictedObstacleTrajectory>,
    /// Historical prediction errors for each discrete time.
    pub calibration_errors_by_time: Vec<Vec<f64>>,
    /// Last discrete time that may appear in a plan.
    pub time_horizon: u64,
    /// Required empirical confidence in `[0, 1]`.
    pub required_confidence: f64,
    /// Base footprint radius added to prediction error margins.
    pub obstacle_radius: f64,
    /// Whether to allow diagonal (8-connected) movement.
    pub allow_diagonal: bool,
}

/// A CP-SIPP path plus trajectory-level confidence summary.
#[derive(Debug, Clone, PartialEq)]
pub struct ConformalSippPlan {
    pub path: SippPath,
    /// Minimum empirical confidence over all returned waypoints.
    pub min_confidence: f64,
    /// Boole-union upper bound on violation probability, using waypoint
    /// confidences: `sum_t (1 - c_t)`, capped at 1.
    pub trajectory_violation_bound: f64,
}

/// CP-SIPP planner with a fixed confidence threshold.
pub struct ConformalSippPlanner {
    planner: SippPlanner,
    width: i32,
    height: i32,
    predicted_obstacles: Vec<PredictedObstacleTrajectory>,
    calibration_errors_by_time: Vec<Vec<f64>>,
    time_horizon: u64,
    required_confidence: f64,
    obstacle_radius: f64,
}

impl ConformalSippPlanner {
    pub fn new(config: ConformalSippConfig) -> RoboticsResult<Self> {
        Self::validate_config(&config)?;

        let dynamic_obstacles = Self::build_dynamic_obstacles(&config);
        let planner = SippPlanner::new(SippConfig {
            width: config.width,
            height: config.height,
            obstacle_map: config.obstacle_map,
            dynamic_obstacles,
            allow_diagonal: config.allow_diagonal,
        })?;

        Ok(Self {
            planner,
            width: config.width,
            height: config.height,
            predicted_obstacles: config.predicted_obstacles,
            calibration_errors_by_time: config.calibration_errors_by_time,
            time_horizon: config.time_horizon,
            required_confidence: config.required_confidence,
            obstacle_radius: config.obstacle_radius,
        })
    }

    /// Plan a path and report empirical confidence over the returned waypoints.
    pub fn plan(&self, sx: i32, sy: i32, gx: i32, gy: i32) -> RoboticsResult<ConformalSippPlan> {
        let path = self.planner.plan(sx, sy, gx, gy)?;
        let mut min_confidence = 1.0;
        let mut violation_bound = 0.0;

        for waypoint in &path {
            let confidence = self.confidence_at(waypoint.x, waypoint.y, waypoint.t)?;
            if confidence < min_confidence {
                min_confidence = confidence;
            }
            violation_bound += 1.0 - confidence;
        }

        Ok(ConformalSippPlan {
            path,
            min_confidence,
            trajectory_violation_bound: violation_bound.min(1.0),
        })
    }

    /// Empirical conformal confidence that cell `(x, y)` is safe at time `t`.
    pub fn confidence_at(&self, x: i32, y: i32, t: u64) -> RoboticsResult<f64> {
        if x < 0 || y < 0 || x >= self.width || y >= self.height {
            return Err(RoboticsError::InvalidParameter(format!(
                "cell ({}, {}) is out of bounds",
                x, y
            )));
        }
        if t > self.time_horizon {
            return Ok(0.0);
        }
        Ok(Self::confidence_from_inputs(
            &self.predicted_obstacles,
            &self.calibration_errors_by_time,
            self.obstacle_radius,
            x,
            y,
            t,
        ))
    }

    /// Conservative empirical quantile radius used at time `t`, including the
    /// configured footprint radius.
    pub fn conformal_radius_at(&self, t: u64) -> RoboticsResult<f64> {
        if t > self.time_horizon {
            return Err(RoboticsError::InvalidParameter(format!(
                "time {} exceeds horizon {}",
                t, self.time_horizon
            )));
        }
        let scores = &self.calibration_errors_by_time[t as usize];
        Ok(Self::empirical_quantile(scores, self.required_confidence) + self.obstacle_radius)
    }

    /// Get the confidence-filtered safe intervals for a particular cell.
    pub fn get_safe_intervals(&self, x: i32, y: i32) -> &[Interval] {
        self.planner.get_safe_intervals(x, y)
    }

    fn validate_config(config: &ConformalSippConfig) -> RoboticsResult<()> {
        if config.width <= 0 || config.height <= 0 {
            return Err(RoboticsError::InvalidParameter(
                "width and height must be positive".to_string(),
            ));
        }
        if config.time_horizon >= u64::MAX - 1 {
            return Err(RoboticsError::InvalidParameter(
                "time_horizon must leave room for half-open intervals".to_string(),
            ));
        }
        if !(0.0..=1.0).contains(&config.required_confidence)
            || !config.required_confidence.is_finite()
        {
            return Err(RoboticsError::InvalidParameter(
                "required_confidence must be finite and in [0, 1]".to_string(),
            ));
        }
        if config.obstacle_radius < 0.0 || !config.obstacle_radius.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "obstacle_radius must be finite and non-negative".to_string(),
            ));
        }
        if config.obstacle_map.len() != config.width as usize {
            return Err(RoboticsError::InvalidParameter(format!(
                "obstacle_map x-dimension ({}) must match width ({})",
                config.obstacle_map.len(),
                config.width
            )));
        }
        for col in &config.obstacle_map {
            if col.len() != config.height as usize {
                return Err(RoboticsError::InvalidParameter(
                    "obstacle_map y-dimension must match height".to_string(),
                ));
            }
        }

        let required_score_sets = config.time_horizon as usize + 1;
        if config.calibration_errors_by_time.len() < required_score_sets {
            return Err(RoboticsError::InvalidParameter(format!(
                "calibration_errors_by_time must contain at least {} entries",
                required_score_sets
            )));
        }
        for (t, scores) in config
            .calibration_errors_by_time
            .iter()
            .take(required_score_sets)
            .enumerate()
        {
            if scores.is_empty() {
                return Err(RoboticsError::InvalidParameter(format!(
                    "calibration score set at time {} must not be empty",
                    t
                )));
            }
            if scores
                .iter()
                .any(|score| *score < 0.0 || !score.is_finite())
            {
                return Err(RoboticsError::InvalidParameter(format!(
                    "calibration score set at time {} contains an invalid score",
                    t
                )));
            }
        }

        for trajectory in &config.predicted_obstacles {
            for sample in &trajectory.samples {
                if sample.t > config.time_horizon {
                    return Err(RoboticsError::InvalidParameter(format!(
                        "predicted sample time {} exceeds horizon {}",
                        sample.t, config.time_horizon
                    )));
                }
                if !sample.x.is_finite() || !sample.y.is_finite() {
                    return Err(RoboticsError::InvalidParameter(
                        "predicted obstacle samples must be finite".to_string(),
                    ));
                }
            }
        }

        Ok(())
    }

    fn build_dynamic_obstacles(config: &ConformalSippConfig) -> Vec<DynamicObstacle> {
        let mut dynamic_obstacles = Vec::new();

        for y in 0..config.height {
            for x in 0..config.width {
                if config.obstacle_map[x as usize][y as usize] {
                    continue;
                }

                for t in 0..=config.time_horizon {
                    let confidence = Self::confidence_from_inputs(
                        &config.predicted_obstacles,
                        &config.calibration_errors_by_time,
                        config.obstacle_radius,
                        x,
                        y,
                        t,
                    );
                    if confidence < config.required_confidence {
                        dynamic_obstacles.push(DynamicObstacle {
                            x,
                            y,
                            interval: Interval::new(t, t + 1),
                        });
                    }
                }

                dynamic_obstacles.push(DynamicObstacle {
                    x,
                    y,
                    interval: Interval::new(config.time_horizon + 1, u64::MAX),
                });
            }
        }

        dynamic_obstacles
    }

    fn confidence_from_inputs(
        predicted_obstacles: &[PredictedObstacleTrajectory],
        calibration_errors_by_time: &[Vec<f64>],
        obstacle_radius: f64,
        x: i32,
        y: i32,
        t: u64,
    ) -> f64 {
        let mut min_distance = f64::INFINITY;
        for trajectory in predicted_obstacles {
            if let Some((ox, oy)) = trajectory.position_at(t) {
                let dx = x as f64 - ox;
                let dy = y as f64 - oy;
                min_distance = min_distance.min((dx * dx + dy * dy).sqrt());
            }
        }

        if min_distance.is_infinite() {
            return 1.0;
        }

        let margin = min_distance - obstacle_radius;
        if margin < 0.0 {
            return 0.0;
        }

        let scores = &calibration_errors_by_time[t as usize];
        let covered = scores.iter().filter(|score| **score <= margin).count();
        covered as f64 / scores.len() as f64
    }

    fn empirical_quantile(scores: &[f64], confidence: f64) -> f64 {
        let mut sorted = scores.to_vec();
        sorted.sort_by(|a, b| a.total_cmp(b));
        let rank = (confidence * sorted.len() as f64).ceil() as usize;
        let idx = rank.saturating_sub(1).min(sorted.len() - 1);
        sorted[idx]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn empty_map(width: i32, height: i32) -> Vec<Vec<bool>> {
        vec![vec![false; height as usize]; width as usize]
    }

    fn repeated_scores(horizon: u64, scores: &[f64]) -> Vec<Vec<f64>> {
        (0..=horizon).map(|_| scores.to_vec()).collect()
    }

    fn predicted_line(points: &[(u64, f64, f64)]) -> PredictedObstacleTrajectory {
        PredictedObstacleTrajectory::new(
            points
                .iter()
                .map(|(t, x, y)| PredictedObstaclePoint::new(*t, *x, *y))
                .collect(),
        )
    }

    fn path_length(path: &[crate::sipp::TimedWaypoint]) -> f64 {
        path.windows(2)
            .map(|window| {
                let dx = window[1].x as f64 - window[0].x as f64;
                let dy = window[1].y as f64 - window[0].y as f64;
                (dx * dx + dy * dy).sqrt()
            })
            .sum()
    }

    fn off_center_steps(path: &[crate::sipp::TimedWaypoint], center_y: i32) -> usize {
        path.iter()
            .filter(|waypoint| waypoint.y != center_y)
            .count()
    }

    #[test]
    fn predicted_trajectory_interpolates_sparse_samples() {
        let trajectory = predicted_line(&[(0, 0.0, 0.0), (4, 4.0, 2.0)]);

        assert_eq!(trajectory.position_at(0), Some((0.0, 0.0)));
        assert_eq!(trajectory.position_at(2), Some((2.0, 1.0)));
        assert_eq!(trajectory.position_at(4), Some((4.0, 2.0)));
        assert_eq!(trajectory.position_at(5), None);
    }

    #[test]
    fn conformal_intervals_use_interpolated_prediction() {
        let planner = ConformalSippPlanner::new(ConformalSippConfig {
            width: 5,
            height: 1,
            obstacle_map: empty_map(5, 1),
            predicted_obstacles: vec![predicted_line(&[(0, 0.0, 0.0), (4, 4.0, 0.0)])],
            calibration_errors_by_time: repeated_scores(4, &[0.1]),
            time_horizon: 4,
            required_confidence: 1.0,
            obstacle_radius: 0.0,
            allow_diagonal: false,
        })
        .unwrap();

        assert_eq!(
            planner.get_safe_intervals(2, 0),
            &[Interval::new(0, 2), Interval::new(3, 5)]
        );
        assert_eq!(planner.confidence_at(2, 0, 2).unwrap(), 0.0);
        assert_eq!(planner.confidence_at(2, 0, 0).unwrap(), 1.0);
    }

    #[test]
    fn conformal_intervals_expand_with_higher_required_confidence() {
        let low_confidence = ConformalSippPlanner::new(ConformalSippConfig {
            width: 3,
            height: 1,
            obstacle_map: empty_map(3, 1),
            predicted_obstacles: vec![predicted_line(&[(2, 1.0, 0.0)])],
            calibration_errors_by_time: repeated_scores(5, &[0.1, 1.1]),
            time_horizon: 5,
            required_confidence: 0.5,
            obstacle_radius: 0.0,
            allow_diagonal: false,
        })
        .unwrap();
        let high_confidence = ConformalSippPlanner::new(ConformalSippConfig {
            width: 3,
            height: 1,
            obstacle_map: empty_map(3, 1),
            predicted_obstacles: vec![predicted_line(&[(2, 1.0, 0.0)])],
            calibration_errors_by_time: repeated_scores(5, &[0.1, 1.1]),
            time_horizon: 5,
            required_confidence: 1.0,
            obstacle_radius: 0.0,
            allow_diagonal: false,
        })
        .unwrap();

        assert_eq!(
            low_confidence.get_safe_intervals(0, 0),
            &[Interval::new(0, 6)]
        );
        assert_eq!(
            high_confidence.get_safe_intervals(0, 0),
            &[Interval::new(0, 2), Interval::new(3, 6)]
        );
    }

    #[test]
    fn conformal_sipp_avoids_predicted_obstacle_region() {
        let planner = ConformalSippPlanner::new(ConformalSippConfig {
            width: 5,
            height: 3,
            obstacle_map: empty_map(5, 3),
            predicted_obstacles: vec![predicted_line(&[
                (2, 2.0, 1.0),
                (3, 2.0, 1.0),
                (4, 2.0, 1.0),
            ])],
            calibration_errors_by_time: repeated_scores(10, &[0.25]),
            time_horizon: 10,
            required_confidence: 1.0,
            obstacle_radius: 0.1,
            allow_diagonal: false,
        })
        .unwrap();

        let plan = planner.plan(0, 1, 4, 1).unwrap();
        assert_eq!(plan.path.first().unwrap().x, 0);
        assert_eq!(plan.path.last().unwrap().x, 4);

        for waypoint in &plan.path {
            assert!(
                !(waypoint.x == 2 && waypoint.y == 1 && (2..5).contains(&waypoint.t)),
                "path entered conformal obstacle region at {:?}",
                waypoint
            );
        }
        assert!(plan.min_confidence >= 1.0);
        assert_eq!(plan.trajectory_violation_bound, 0.0);
    }

    #[test]
    fn confidence_sweep_regression_matches_expected_safety_tradeoff() {
        let width = 9;
        let height = 5;
        let start = (0, 2);
        let goal = (8, 2);
        let time_horizon = 16;
        let obstacle_map = empty_map(width, height);
        let calibration_scores = repeated_scores(time_horizon, &[0.10, 0.60, 1.20]);
        let expected = [
            (0.0, 8, 8.0, 0, 0.0, 1.0),
            (0.5, 10, 10.0, 6, 2.0 / 3.0, 1.0 / 3.0),
            (0.7, 11, 10.0, 5, 1.0, 0.0),
            (1.0, 11, 10.0, 5, 1.0, 0.0),
        ];

        for (required_confidence, arrival_t, length, off_center, min_confidence, violation_bound) in
            expected
        {
            let planner = ConformalSippPlanner::new(ConformalSippConfig {
                width,
                height,
                obstacle_map: obstacle_map.clone(),
                predicted_obstacles: vec![predicted_line(&[(4, 4.0, 2.0), (5, 4.0, 2.0)])],
                calibration_errors_by_time: calibration_scores.clone(),
                time_horizon,
                required_confidence,
                obstacle_radius: 0.0,
                allow_diagonal: false,
            })
            .unwrap();
            let plan = planner.plan(start.0, start.1, goal.0, goal.1).unwrap();

            assert_eq!(plan.path.last().unwrap().t, arrival_t);
            assert!(
                (path_length(&plan.path) - length).abs() < 1e-9,
                "unexpected length for confidence {required_confidence}"
            );
            assert_eq!(off_center_steps(&plan.path, start.1), off_center);
            assert!(
                (plan.min_confidence - min_confidence).abs() < 1e-9,
                "unexpected min confidence for confidence {required_confidence}"
            );
            assert!(
                (plan.trajectory_violation_bound - violation_bound).abs() < 1e-9,
                "unexpected violation bound for confidence {required_confidence}"
            );
        }
    }

    #[test]
    fn conformal_radius_uses_empirical_quantile_plus_footprint() {
        let planner = ConformalSippPlanner::new(ConformalSippConfig {
            width: 2,
            height: 2,
            obstacle_map: empty_map(2, 2),
            predicted_obstacles: vec![],
            calibration_errors_by_time: repeated_scores(3, &[0.1, 0.4, 0.8, 1.2]),
            time_horizon: 3,
            required_confidence: 0.75,
            obstacle_radius: 0.2,
            allow_diagonal: false,
        })
        .unwrap();

        assert!((planner.conformal_radius_at(0).unwrap() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn conformal_config_rejects_missing_calibration_horizon() {
        let result = ConformalSippPlanner::new(ConformalSippConfig {
            width: 2,
            height: 2,
            obstacle_map: empty_map(2, 2),
            predicted_obstacles: vec![],
            calibration_errors_by_time: repeated_scores(2, &[0.1]),
            time_horizon: 3,
            required_confidence: 0.9,
            obstacle_radius: 0.0,
            allow_diagonal: false,
        });

        assert!(matches!(result, Err(RoboticsError::InvalidParameter(_))));
    }
}
