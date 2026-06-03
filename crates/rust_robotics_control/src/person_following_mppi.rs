//! Adaptive person-following helpers for MPPI.
//!
//! This is a compact 2-D reproduction slice of Adap-RPF: target-centric
//! following-point sampling, multi-objective candidate scoring, and
//! prediction-aware MPPI goal generation.

use rust_robotics_core::{RoboticsError, RoboticsResult};

use crate::mppi::{MppiMovingObstacle2D, MppiState2D};

const EPS: f64 = 1e-9;

/// Candidate following point evaluated around the predicted target person.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MppiPersonFollowingCandidate2D {
    pub x: f64,
    pub y: f64,
    pub offset_x: f64,
    pub offset_y: f64,
    pub distance_to_target: f64,
    pub bearing: f64,
    pub total_cost: f64,
    pub visibility_cost: f64,
    pub proximity_cost: f64,
    pub distance_cost: f64,
    pub travel_cost: f64,
    pub stickiness_cost: f64,
    pub feasible: bool,
}

/// Configuration for target-centric adaptive following-point sampling.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MppiPersonFollowingConfig2D {
    pub candidate_count: usize,
    pub horizon: usize,
    pub dt: f64,
    pub personal_radius: f64,
    pub social_radius: f64,
    pub desired_distance: f64,
    pub proximity_margin: f64,
    pub occlusion_margin: f64,
    pub visibility_weight: f64,
    pub proximity_weight: f64,
    pub distance_weight: f64,
    pub travel_weight: f64,
    pub stickiness_weight: f64,
}

impl Default for MppiPersonFollowingConfig2D {
    fn default() -> Self {
        Self {
            candidate_count: 64,
            horizon: 16,
            dt: 0.1,
            personal_radius: 0.55,
            social_radius: 1.45,
            desired_distance: 0.95,
            proximity_margin: 0.28,
            occlusion_margin: 0.18,
            visibility_weight: 7.0,
            proximity_weight: 5.0,
            distance_weight: 1.8,
            travel_weight: 0.18,
            stickiness_weight: 0.65,
        }
    }
}

/// Deterministic adaptive person-following sampler.
#[derive(Debug, Clone, PartialEq)]
pub struct MppiPersonFollowingSampler2D {
    config: MppiPersonFollowingConfig2D,
    previous_offset: Option<(f64, f64)>,
}

impl MppiPersonFollowingSampler2D {
    pub fn new(config: MppiPersonFollowingConfig2D) -> RoboticsResult<Self> {
        validate_person_following_config(&config)?;
        Ok(Self {
            config,
            previous_offset: None,
        })
    }

    pub fn config(&self) -> MppiPersonFollowingConfig2D {
        self.config
    }

    pub fn previous_offset(&self) -> Option<(f64, f64)> {
        self.previous_offset
    }

    pub fn reset_stickiness(&mut self) {
        self.previous_offset = None;
    }

    pub fn sample_candidates(
        &self,
        robot: MppiState2D,
        target: MppiMovingObstacle2D,
        pedestrians: &[MppiMovingObstacle2D],
    ) -> RoboticsResult<Vec<MppiPersonFollowingCandidate2D>> {
        validate_state(robot)?;
        validate_moving_obstacle(target)?;
        validate_moving_obstacles(pedestrians)?;

        let heading = target_heading(robot, target);
        let future_target = target.predict(self.config.dt * self.config.horizon as f64);
        let mut candidates = Vec::with_capacity(self.config.candidate_count);

        for index in 0..self.config.candidate_count {
            let u = halton(index + 1, 2);
            let v = halton(index + 1, 3);
            let radius_sq = self.config.personal_radius * self.config.personal_radius
                + u * (self.config.social_radius * self.config.social_radius
                    - self.config.personal_radius * self.config.personal_radius);
            let radius = radius_sq.sqrt();
            let bearing = heading + std::f64::consts::PI + (v - 0.5) * std::f64::consts::PI;
            let offset_x = radius * bearing.cos();
            let offset_y = radius * bearing.sin();
            let x = future_target.x + offset_x;
            let y = future_target.y + offset_y;
            candidates.push(self.score_candidate(
                robot,
                target,
                pedestrians,
                x,
                y,
                offset_x,
                offset_y,
                bearing,
            ));
        }

        Ok(candidates)
    }

    pub fn select_following_point(
        &mut self,
        robot: MppiState2D,
        target: MppiMovingObstacle2D,
        pedestrians: &[MppiMovingObstacle2D],
    ) -> RoboticsResult<MppiPersonFollowingCandidate2D> {
        let candidates = self.sample_candidates(robot, target, pedestrians)?;
        let selected = candidates
            .iter()
            .filter(|candidate| candidate.feasible)
            .min_by(|a, b| a.total_cost.total_cmp(&b.total_cost))
            .or_else(|| {
                candidates
                    .iter()
                    .min_by(|a, b| a.total_cost.total_cmp(&b.total_cost))
            })
            .copied()
            .expect("validated sampler creates at least one candidate");
        self.previous_offset = Some((selected.offset_x, selected.offset_y));
        Ok(selected)
    }

    pub fn goal_trajectory_for_candidate(
        &self,
        target: MppiMovingObstacle2D,
        candidate: MppiPersonFollowingCandidate2D,
    ) -> Vec<(f64, f64)> {
        (0..=self.config.horizon)
            .map(|step| {
                let predicted = target.predict(step as f64 * self.config.dt);
                (
                    predicted.x + candidate.offset_x,
                    predicted.y + candidate.offset_y,
                )
            })
            .collect()
    }

    fn score_candidate(
        &self,
        robot: MppiState2D,
        target: MppiMovingObstacle2D,
        pedestrians: &[MppiMovingObstacle2D],
        x: f64,
        y: f64,
        offset_x: f64,
        offset_y: f64,
        bearing: f64,
    ) -> MppiPersonFollowingCandidate2D {
        let distance_to_target = (offset_x * offset_x + offset_y * offset_y).sqrt();
        let distance_cost = (distance_to_target - self.config.desired_distance).powi(2);
        let travel_cost = squared_distance((robot.x, robot.y), (x, y));
        let stickiness_cost = self.previous_offset.map_or(0.0, |previous| {
            squared_distance(previous, (offset_x, offset_y))
        });
        let mut proximity_cost = 0.0;
        let mut visibility_cost = 0.0;
        let mut feasible = distance_to_target >= self.config.personal_radius;

        for step in 0..=self.config.horizon {
            let time = step as f64 * self.config.dt;
            let target_predicted = target.predict(time);
            let candidate_predicted =
                (target_predicted.x + offset_x, target_predicted.y + offset_y);
            for pedestrian in pedestrians {
                let pedestrian_predicted = pedestrian.predict(time);
                let clearance = point_distance(
                    candidate_predicted,
                    (pedestrian_predicted.x, pedestrian_predicted.y),
                ) - pedestrian_predicted.radius
                    - self.config.proximity_margin;
                if clearance < 0.0 {
                    feasible = false;
                    proximity_cost += (1.0 - clearance).powi(2);
                } else if clearance < self.config.social_radius {
                    proximity_cost += 1.0 / (clearance + 0.2);
                }

                let occlusion_clearance = point_segment_distance(
                    (pedestrian_predicted.x, pedestrian_predicted.y),
                    candidate_predicted,
                    (target_predicted.x, target_predicted.y),
                ) - pedestrian_predicted.radius
                    - self.config.occlusion_margin;
                if occlusion_clearance < 0.0 {
                    visibility_cost += (1.0 - occlusion_clearance).powi(2);
                }
            }
        }

        let scale = (self.config.horizon + 1) as f64;
        proximity_cost /= scale;
        visibility_cost /= scale;
        let total_cost = self.config.distance_weight * distance_cost
            + self.config.travel_weight * travel_cost
            + self.config.stickiness_weight * stickiness_cost
            + self.config.proximity_weight * proximity_cost
            + self.config.visibility_weight * visibility_cost;

        MppiPersonFollowingCandidate2D {
            x,
            y,
            offset_x,
            offset_y,
            distance_to_target,
            bearing,
            total_cost,
            visibility_cost,
            proximity_cost,
            distance_cost,
            travel_cost,
            stickiness_cost,
            feasible,
        }
    }
}

fn validate_person_following_config(config: &MppiPersonFollowingConfig2D) -> RoboticsResult<()> {
    if config.candidate_count == 0 || config.horizon == 0 {
        return Err(RoboticsError::InvalidParameter(
            "person-following candidate_count and horizon must be positive".to_string(),
        ));
    }
    for (label, value) in [
        ("person-following dt", config.dt),
        ("person-following personal_radius", config.personal_radius),
        ("person-following social_radius", config.social_radius),
        ("person-following desired_distance", config.desired_distance),
        ("person-following proximity_margin", config.proximity_margin),
        ("person-following occlusion_margin", config.occlusion_margin),
        (
            "person-following visibility_weight",
            config.visibility_weight,
        ),
        ("person-following proximity_weight", config.proximity_weight),
        ("person-following distance_weight", config.distance_weight),
        ("person-following travel_weight", config.travel_weight),
        (
            "person-following stickiness_weight",
            config.stickiness_weight,
        ),
    ] {
        if value < 0.0 || !value.is_finite() {
            return Err(RoboticsError::InvalidParameter(format!(
                "{label} must be finite and non-negative"
            )));
        }
    }
    if config.dt <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "person-following dt must be positive".to_string(),
        ));
    }
    if config.social_radius <= config.personal_radius {
        return Err(RoboticsError::InvalidParameter(
            "person-following social_radius must exceed personal_radius".to_string(),
        ));
    }
    Ok(())
}

fn validate_state(state: MppiState2D) -> RoboticsResult<()> {
    if !state.x.is_finite()
        || !state.y.is_finite()
        || !state.vx.is_finite()
        || !state.vy.is_finite()
    {
        return Err(RoboticsError::InvalidParameter(
            "person-following robot state must be finite".to_string(),
        ));
    }
    Ok(())
}

fn validate_moving_obstacles(obstacles: &[MppiMovingObstacle2D]) -> RoboticsResult<()> {
    for &obstacle in obstacles {
        validate_moving_obstacle(obstacle)?;
    }
    Ok(())
}

fn validate_moving_obstacle(obstacle: MppiMovingObstacle2D) -> RoboticsResult<()> {
    if !obstacle.x.is_finite()
        || !obstacle.y.is_finite()
        || !obstacle.vx.is_finite()
        || !obstacle.vy.is_finite()
    {
        return Err(RoboticsError::InvalidParameter(
            "person-following moving obstacle state must be finite".to_string(),
        ));
    }
    if obstacle.radius <= 0.0 || !obstacle.radius.is_finite() {
        return Err(RoboticsError::InvalidParameter(
            "person-following moving obstacle radius must be finite and positive".to_string(),
        ));
    }
    Ok(())
}

fn target_heading(robot: MppiState2D, target: MppiMovingObstacle2D) -> f64 {
    let speed_sq = target.vx * target.vx + target.vy * target.vy;
    if speed_sq > EPS {
        target.vy.atan2(target.vx)
    } else {
        (target.y - robot.y).atan2(target.x - robot.x)
    }
}

fn halton(mut index: usize, base: usize) -> f64 {
    let mut factor = 1.0 / base as f64;
    let mut result = 0.0;
    while index > 0 {
        result += factor * (index % base) as f64;
        index /= base;
        factor /= base as f64;
    }
    result
}

fn squared_distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    dx * dx + dy * dy
}

fn point_distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    squared_distance(a, b).sqrt()
}

fn point_segment_distance(point: (f64, f64), start: (f64, f64), end: (f64, f64)) -> f64 {
    let dx = end.0 - start.0;
    let dy = end.1 - start.1;
    let length_sq = dx * dx + dy * dy;
    if length_sq <= EPS {
        return point_distance(point, start);
    }
    let t = (((point.0 - start.0) * dx + (point.1 - start.1) * dy) / length_sq).clamp(0.0, 1.0);
    point_distance(point, (start.0 + t * dx, start.1 + t * dy))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sampler_generates_backward_social_candidates() {
        let sampler =
            MppiPersonFollowingSampler2D::new(MppiPersonFollowingConfig2D::default()).unwrap();
        let robot = MppiState2D::new(-1.0, 0.0, 0.0, 0.0);
        let target = MppiMovingObstacle2D::new(0.0, 0.0, 1.0, 0.0, 0.3);
        let candidates = sampler.sample_candidates(robot, target, &[]).unwrap();

        assert_eq!(candidates.len(), sampler.config().candidate_count);
        assert!(candidates.iter().all(|candidate| candidate.x < 2.0));
        assert!(candidates
            .iter()
            .all(|candidate| candidate.distance_to_target >= sampler.config().personal_radius));
        assert!(candidates
            .iter()
            .all(|candidate| candidate.distance_to_target <= sampler.config().social_radius));
    }

    #[test]
    fn occlusion_changes_selected_candidate_side() {
        let mut sampler =
            MppiPersonFollowingSampler2D::new(MppiPersonFollowingConfig2D::default()).unwrap();
        let robot = MppiState2D::new(-1.0, 0.0, 0.0, 0.0);
        let target = MppiMovingObstacle2D::new(0.0, 0.0, 0.7, 0.0, 0.3);
        let blocking_pedestrian = MppiMovingObstacle2D::new(-0.55, 0.0, 0.7, 0.0, 0.32);

        let selected = sampler
            .select_following_point(robot, target, &[blocking_pedestrian])
            .unwrap();

        assert!(selected.feasible);
        assert!(selected.offset_y.abs() > 0.15);
        assert!(selected.visibility_cost < 2.0);
    }

    #[test]
    fn selected_candidate_produces_horizon_goal_trajectory() {
        let mut sampler =
            MppiPersonFollowingSampler2D::new(MppiPersonFollowingConfig2D::default()).unwrap();
        let robot = MppiState2D::new(-1.0, 0.0, 0.0, 0.0);
        let target = MppiMovingObstacle2D::new(0.0, 0.0, 0.5, 0.0, 0.3);
        let selected = sampler.select_following_point(robot, target, &[]).unwrap();
        let goals = sampler.goal_trajectory_for_candidate(target, selected);

        assert_eq!(goals.len(), sampler.config().horizon + 1);
        assert!((goals[0].0 - selected.offset_x).abs() < 1e-9);
        assert!(goals.last().unwrap().0 > goals[0].0);
    }
}
