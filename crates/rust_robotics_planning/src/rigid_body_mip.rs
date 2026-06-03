//! Rigid-body MIP-style path planning.
//!
//! This module implements a deterministic first reproduction slice for
//! mixed-integer rigid-body planning. It avoids an external MILP solver by
//! searching a discretized SE(2) lattice, while every accepted pose carries the
//! same kind of binary disjunctive obstacle-avoidance certificate used in
//! convex-obstacle MILP formulations: for each obstacle, one active half-space
//! separates the whole robot rectangle from that obstacle.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::f64::consts::TAU;

use rust_robotics_core::{RoboticsError, RoboticsResult};

const EPS: f64 = 1.0e-9;

/// 2-D point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidBodyPoint2D {
    pub x: f64,
    pub y: f64,
}

impl RigidBodyPoint2D {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

/// Rigid-body center pose.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidBodyPose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl RigidBodyPose2D {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }
}

/// Half-space `a*x + b*y <= c` containing the obstacle interior.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidBodyHalfspace2D {
    pub a: f64,
    pub b: f64,
    pub c: f64,
}

impl RigidBodyHalfspace2D {
    pub fn new(a: f64, b: f64, c: f64) -> RoboticsResult<Self> {
        if !a.is_finite() || !b.is_finite() || !c.is_finite() || (a.abs() + b.abs()) <= EPS {
            return Err(RoboticsError::InvalidParameter(
                "rigid-body MIP half-space coefficients must be finite and non-zero".to_string(),
            ));
        }
        Ok(Self { a, b, c })
    }

    pub fn signed_violation(self, point: RigidBodyPoint2D) -> f64 {
        self.a * point.x + self.b * point.y - self.c
    }
}

/// Convex obstacle represented by vertices and fixed half-spaces.
#[derive(Debug, Clone, PartialEq)]
pub struct RigidBodyConvexObstacle2D {
    pub vertices: Vec<RigidBodyPoint2D>,
    pub halfspaces: Vec<RigidBodyHalfspace2D>,
}

impl RigidBodyConvexObstacle2D {
    pub fn convex_polygon(vertices: Vec<RigidBodyPoint2D>) -> RoboticsResult<Self> {
        if vertices.len() < 3 {
            return Err(RoboticsError::InvalidParameter(
                "rigid-body MIP convex polygon must have at least 3 vertices".to_string(),
            ));
        }
        if !vertices
            .iter()
            .all(|vertex| vertex.x.is_finite() && vertex.y.is_finite())
        {
            return Err(RoboticsError::InvalidParameter(
                "rigid-body MIP convex polygon vertices must be finite".to_string(),
            ));
        }
        let area = signed_area(&vertices);
        if area.abs() <= EPS {
            return Err(RoboticsError::InvalidParameter(
                "rigid-body MIP convex polygon area must be non-zero".to_string(),
            ));
        }
        let mut vertices = vertices;
        if area < 0.0 {
            vertices.reverse();
        }
        validate_convex_ccw_polygon(&vertices)?;

        let mut halfspaces = Vec::with_capacity(vertices.len());
        for index in 0..vertices.len() {
            let from = vertices[index];
            let to = vertices[(index + 1) % vertices.len()];
            let dx = to.x - from.x;
            let dy = to.y - from.y;
            halfspaces.push(RigidBodyHalfspace2D::new(
                dy,
                -dx,
                dy * from.x - dx * from.y,
            )?);
        }
        Ok(Self {
            vertices,
            halfspaces,
        })
    }

    pub fn aabb(min_x: f64, max_x: f64, min_y: f64, max_y: f64) -> RoboticsResult<Self> {
        if !min_x.is_finite()
            || !max_x.is_finite()
            || !min_y.is_finite()
            || !max_y.is_finite()
            || min_x >= max_x
            || min_y >= max_y
        {
            return Err(RoboticsError::InvalidParameter(
                "rigid-body MIP AABB bounds must be finite and ordered".to_string(),
            ));
        }
        Ok(Self {
            vertices: vec![
                RigidBodyPoint2D::new(min_x, min_y),
                RigidBodyPoint2D::new(max_x, min_y),
                RigidBodyPoint2D::new(max_x, max_y),
                RigidBodyPoint2D::new(min_x, max_y),
            ],
            halfspaces: vec![
                RigidBodyHalfspace2D::new(-1.0, 0.0, -min_x)?,
                RigidBodyHalfspace2D::new(1.0, 0.0, max_x)?,
                RigidBodyHalfspace2D::new(0.0, -1.0, -min_y)?,
                RigidBodyHalfspace2D::new(0.0, 1.0, max_y)?,
            ],
        })
    }
}

/// Active binary-style obstacle-avoidance choice for one obstacle at one pose.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidBodyMipSeparationCertificate2D {
    pub obstacle_index: usize,
    pub halfspace_index: usize,
    pub margin: f64,
}

/// Planner configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct RigidBodyMipConfig2D {
    pub min_x: f64,
    pub max_x: f64,
    pub min_y: f64,
    pub max_y: f64,
    pub position_step: f64,
    pub heading_count: usize,
    pub robot_half_length: f64,
    pub robot_half_width: f64,
    pub clearance: f64,
    pub move_cost: u64,
    pub turn_cost: u64,
    pub max_expansions: usize,
    pub obstacles: Vec<RigidBodyConvexObstacle2D>,
}

impl RigidBodyMipConfig2D {
    pub fn new(min_x: f64, max_x: f64, min_y: f64, max_y: f64) -> Self {
        Self {
            min_x,
            max_x,
            min_y,
            max_y,
            position_step: 0.5,
            heading_count: 8,
            robot_half_length: 0.55,
            robot_half_width: 0.25,
            clearance: 0.02,
            move_cost: 10,
            turn_cost: 3,
            max_expansions: 20_000,
            obstacles: Vec::new(),
        }
    }
}

/// Planning result.
#[derive(Debug, Clone, PartialEq)]
pub struct RigidBodyMipPlan2D {
    pub poses: Vec<RigidBodyPose2D>,
    pub certificates: Vec<Vec<RigidBodyMipSeparationCertificate2D>>,
    pub segment_certificates: Vec<Vec<RigidBodyMipSeparationCertificate2D>>,
    pub total_cost: u64,
    pub expanded_states: usize,
    pub binary_separation_choices: usize,
    pub segment_binary_separation_choices: usize,
    pub min_separation_margin: f64,
    pub min_segment_separation_margin: f64,
}

/// Discretized rigid-body planner with MIP-style separation certificates.
#[derive(Debug, Clone, PartialEq)]
pub struct RigidBodyMipPlanner2D {
    config: RigidBodyMipConfig2D,
    nx: i32,
    ny: i32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct SearchState {
    ix: i32,
    iy: i32,
    heading: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct OpenEntry {
    state: SearchState,
    f: u64,
    g: u64,
}

impl Ord for OpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f
            .cmp(&self.f)
            .then_with(|| other.g.cmp(&self.g))
            .then_with(|| other.state.heading.cmp(&self.state.heading))
    }
}

impl PartialOrd for OpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl RigidBodyMipPlanner2D {
    pub fn new(config: RigidBodyMipConfig2D) -> RoboticsResult<Self> {
        validate_config(&config)?;
        let nx = ((config.max_x - config.min_x) / config.position_step).round() as i32;
        let ny = ((config.max_y - config.min_y) / config.position_step).round() as i32;
        Ok(Self { config, nx, ny })
    }

    pub fn config(&self) -> &RigidBodyMipConfig2D {
        &self.config
    }

    pub fn plan(
        &self,
        start: RigidBodyPose2D,
        goal: RigidBodyPose2D,
        require_goal_heading: bool,
    ) -> RoboticsResult<RigidBodyMipPlan2D> {
        let start_state = self.pose_to_state(start)?;
        let goal_state = self.pose_to_state(goal)?;
        if !self.is_state_feasible(start_state) || !self.is_state_feasible(goal_state) {
            return Err(RoboticsError::InvalidParameter(
                "rigid-body MIP start and goal poses must be feasible".to_string(),
            ));
        }

        let mut open = BinaryHeap::new();
        let mut best_g = HashMap::new();
        let mut parent: HashMap<SearchState, Option<SearchState>> = HashMap::new();
        best_g.insert(start_state, 0);
        parent.insert(start_state, None);
        open.push(OpenEntry {
            state: start_state,
            g: 0,
            f: self.heuristic(start_state, goal_state, require_goal_heading),
        });

        let mut expanded_states = 0;
        while let Some(entry) = open.pop() {
            if entry.g > *best_g.get(&entry.state).unwrap_or(&u64::MAX) {
                continue;
            }
            expanded_states += 1;
            if expanded_states > self.config.max_expansions {
                return Err(RoboticsError::PlanningError(
                    "rigid-body MIP planner exceeded expansion limit".to_string(),
                ));
            }
            if self.is_goal(entry.state, goal_state, require_goal_heading) {
                return self.build_plan(parent, entry.state, entry.g, expanded_states);
            }
            for (next, step_cost) in self.successors(entry.state) {
                if !self.is_state_feasible(next) || !self.is_transition_feasible(entry.state, next)
                {
                    continue;
                }
                let next_g = entry.g + step_cost;
                if next_g < *best_g.get(&next).unwrap_or(&u64::MAX) {
                    best_g.insert(next, next_g);
                    parent.insert(next, Some(entry.state));
                    open.push(OpenEntry {
                        state: next,
                        g: next_g,
                        f: next_g + self.heuristic(next, goal_state, require_goal_heading),
                    });
                }
            }
        }

        Err(RoboticsError::PlanningError(
            "rigid-body MIP planner could not find a feasible path".to_string(),
        ))
    }

    pub fn robot_vertices(&self, pose: RigidBodyPose2D) -> Vec<RigidBodyPoint2D> {
        let c = pose.theta.cos();
        let s = pose.theta.sin();
        [
            (self.config.robot_half_length, self.config.robot_half_width),
            (self.config.robot_half_length, -self.config.robot_half_width),
            (
                -self.config.robot_half_length,
                -self.config.robot_half_width,
            ),
            (-self.config.robot_half_length, self.config.robot_half_width),
        ]
        .into_iter()
        .map(|(lx, ly)| RigidBodyPoint2D::new(pose.x + c * lx - s * ly, pose.y + s * lx + c * ly))
        .collect()
    }

    pub fn separation_certificates(
        &self,
        pose: RigidBodyPose2D,
    ) -> Option<Vec<RigidBodyMipSeparationCertificate2D>> {
        let vertices = self.robot_vertices(pose);
        self.separation_certificates_for_vertices(&vertices)
    }

    pub fn segment_separation_certificates(
        &self,
        from: RigidBodyPose2D,
        to: RigidBodyPose2D,
    ) -> Option<Vec<RigidBodyMipSeparationCertificate2D>> {
        let mut vertices = Vec::new();
        for alpha in [0.0, 0.5, 1.0] {
            vertices.extend(self.robot_vertices(interpolate_pose(from, to, alpha)));
        }
        self.separation_certificates_for_vertices(&vertices)
    }

    fn separation_certificates_for_vertices(
        &self,
        vertices: &[RigidBodyPoint2D],
    ) -> Option<Vec<RigidBodyMipSeparationCertificate2D>> {
        if !vertices
            .iter()
            .all(|vertex| self.point_inside_bounds(*vertex))
        {
            return None;
        }

        let mut certificates = Vec::with_capacity(self.config.obstacles.len());
        for (obstacle_index, obstacle) in self.config.obstacles.iter().enumerate() {
            let mut best: Option<RigidBodyMipSeparationCertificate2D> = None;
            for (halfspace_index, halfspace) in obstacle.halfspaces.iter().copied().enumerate() {
                let margin = vertices
                    .iter()
                    .map(|vertex| halfspace.signed_violation(*vertex))
                    .fold(f64::INFINITY, f64::min);
                if margin > self.config.clearance {
                    let certificate = RigidBodyMipSeparationCertificate2D {
                        obstacle_index,
                        halfspace_index,
                        margin,
                    };
                    best = match best {
                        Some(previous) if previous.margin >= certificate.margin => Some(previous),
                        _ => Some(certificate),
                    };
                }
            }
            certificates.push(best?);
        }
        Some(certificates)
    }

    pub fn is_pose_feasible(&self, pose: RigidBodyPose2D) -> bool {
        self.separation_certificates(pose).is_some()
    }

    pub fn is_segment_feasible(&self, from: RigidBodyPose2D, to: RigidBodyPose2D) -> bool {
        self.segment_separation_certificates(from, to).is_some()
    }

    fn build_plan(
        &self,
        parent: HashMap<SearchState, Option<SearchState>>,
        mut current: SearchState,
        total_cost: u64,
        expanded_states: usize,
    ) -> RoboticsResult<RigidBodyMipPlan2D> {
        let mut states = Vec::new();
        loop {
            states.push(current);
            match parent.get(&current).copied().flatten() {
                Some(previous) => current = previous,
                None => break,
            }
        }
        states.reverse();

        let mut poses = Vec::with_capacity(states.len());
        let mut certificates = Vec::with_capacity(states.len());
        let mut min_separation_margin = f64::INFINITY;
        for &state in &states {
            let pose = self.state_to_pose(state);
            let pose_certificates = self.separation_certificates(pose).ok_or_else(|| {
                RoboticsError::PlanningError(
                    "rigid-body MIP reconstructed an infeasible pose".to_string(),
                )
            })?;
            for certificate in &pose_certificates {
                min_separation_margin = min_separation_margin.min(certificate.margin);
            }
            poses.push(pose);
            certificates.push(pose_certificates);
        }
        let mut segment_certificates = Vec::with_capacity(poses.len().saturating_sub(1));
        let mut min_segment_separation_margin = f64::INFINITY;
        for window in poses.windows(2) {
            let certificates = self
                .segment_separation_certificates(window[0], window[1])
                .ok_or_else(|| {
                    RoboticsError::PlanningError(
                        "rigid-body MIP reconstructed an infeasible segment".to_string(),
                    )
                })?;
            for certificate in &certificates {
                min_segment_separation_margin =
                    min_segment_separation_margin.min(certificate.margin);
            }
            segment_certificates.push(certificates);
        }
        if segment_certificates.is_empty() {
            min_segment_separation_margin = min_separation_margin;
        }
        Ok(RigidBodyMipPlan2D {
            binary_separation_choices: certificates.iter().map(Vec::len).sum(),
            segment_binary_separation_choices: segment_certificates.iter().map(Vec::len).sum(),
            poses,
            certificates,
            segment_certificates,
            total_cost,
            expanded_states,
            min_separation_margin,
            min_segment_separation_margin,
        })
    }

    fn successors(&self, state: SearchState) -> Vec<(SearchState, u64)> {
        let mut successors = vec![
            (
                SearchState {
                    ix: state.ix + 1,
                    iy: state.iy,
                    heading: state.heading,
                },
                self.config.move_cost,
            ),
            (
                SearchState {
                    ix: state.ix - 1,
                    iy: state.iy,
                    heading: state.heading,
                },
                self.config.move_cost,
            ),
            (
                SearchState {
                    ix: state.ix,
                    iy: state.iy + 1,
                    heading: state.heading,
                },
                self.config.move_cost,
            ),
            (
                SearchState {
                    ix: state.ix,
                    iy: state.iy - 1,
                    heading: state.heading,
                },
                self.config.move_cost,
            ),
            (
                SearchState {
                    ix: state.ix,
                    iy: state.iy,
                    heading: (state.heading + 1) % self.config.heading_count,
                },
                self.config.turn_cost,
            ),
            (
                SearchState {
                    ix: state.ix,
                    iy: state.iy,
                    heading: (state.heading + self.config.heading_count - 1)
                        % self.config.heading_count,
                },
                self.config.turn_cost,
            ),
        ];
        successors.retain(|(candidate, _)| self.state_in_bounds(*candidate));
        successors
    }

    fn is_state_feasible(&self, state: SearchState) -> bool {
        self.state_in_bounds(state) && self.is_pose_feasible(self.state_to_pose(state))
    }

    fn is_transition_feasible(&self, from: SearchState, to: SearchState) -> bool {
        self.is_segment_feasible(self.state_to_pose(from), self.state_to_pose(to))
    }

    fn state_in_bounds(&self, state: SearchState) -> bool {
        state.ix >= 0
            && state.iy >= 0
            && state.ix <= self.nx
            && state.iy <= self.ny
            && state.heading < self.config.heading_count
    }

    fn is_goal(&self, state: SearchState, goal: SearchState, require_goal_heading: bool) -> bool {
        state.ix == goal.ix
            && state.iy == goal.iy
            && (!require_goal_heading || state.heading == goal.heading)
    }

    fn heuristic(&self, state: SearchState, goal: SearchState, require_goal_heading: bool) -> u64 {
        let moves =
            (state.ix - goal.ix).unsigned_abs() as u64 + (state.iy - goal.iy).unsigned_abs() as u64;
        let turns = if require_goal_heading {
            circular_heading_distance(state.heading, goal.heading, self.config.heading_count) as u64
        } else {
            0
        };
        moves * self.config.move_cost + turns * self.config.turn_cost
    }

    fn pose_to_state(&self, pose: RigidBodyPose2D) -> RoboticsResult<SearchState> {
        if !pose.x.is_finite() || !pose.y.is_finite() || !pose.theta.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "rigid-body MIP pose must be finite".to_string(),
            ));
        }
        let ix = ((pose.x - self.config.min_x) / self.config.position_step).round() as i32;
        let iy = ((pose.y - self.config.min_y) / self.config.position_step).round() as i32;
        let heading = theta_to_index(pose.theta, self.config.heading_count);
        let state = SearchState { ix, iy, heading };
        if !self.state_in_bounds(state) {
            return Err(RoboticsError::InvalidParameter(
                "rigid-body MIP pose is outside planner bounds".to_string(),
            ));
        }
        Ok(state)
    }

    fn state_to_pose(&self, state: SearchState) -> RigidBodyPose2D {
        RigidBodyPose2D::new(
            self.config.min_x + state.ix as f64 * self.config.position_step,
            self.config.min_y + state.iy as f64 * self.config.position_step,
            state.heading as f64 * TAU / self.config.heading_count as f64,
        )
    }

    fn point_inside_bounds(&self, point: RigidBodyPoint2D) -> bool {
        point.x >= self.config.min_x + self.config.clearance
            && point.x <= self.config.max_x - self.config.clearance
            && point.y >= self.config.min_y + self.config.clearance
            && point.y <= self.config.max_y - self.config.clearance
    }
}

fn circular_heading_distance(a: usize, b: usize, count: usize) -> usize {
    let raw = a.abs_diff(b);
    raw.min(count - raw)
}

fn theta_to_index(theta: f64, heading_count: usize) -> usize {
    let wrapped = theta.rem_euclid(TAU);
    ((wrapped / TAU * heading_count as f64).round() as usize) % heading_count
}

fn interpolate_pose(from: RigidBodyPose2D, to: RigidBodyPose2D, alpha: f64) -> RigidBodyPose2D {
    let dtheta = shortest_angle_delta(from.theta, to.theta);
    RigidBodyPose2D::new(
        from.x + (to.x - from.x) * alpha,
        from.y + (to.y - from.y) * alpha,
        (from.theta + dtheta * alpha).rem_euclid(TAU),
    )
}

fn shortest_angle_delta(from: f64, to: f64) -> f64 {
    (to - from + std::f64::consts::PI).rem_euclid(TAU) - std::f64::consts::PI
}

fn signed_area(vertices: &[RigidBodyPoint2D]) -> f64 {
    let mut area = 0.0;
    for index in 0..vertices.len() {
        let current = vertices[index];
        let next = vertices[(index + 1) % vertices.len()];
        area += current.x * next.y - next.x * current.y;
    }
    area * 0.5
}

fn validate_convex_ccw_polygon(vertices: &[RigidBodyPoint2D]) -> RoboticsResult<()> {
    for index in 0..vertices.len() {
        let a = vertices[index];
        let b = vertices[(index + 1) % vertices.len()];
        let c = vertices[(index + 2) % vertices.len()];
        let cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
        if cross < -EPS {
            return Err(RoboticsError::InvalidParameter(
                "rigid-body MIP polygon vertices must form a convex polygon".to_string(),
            ));
        }
        if ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt() <= EPS {
            return Err(RoboticsError::InvalidParameter(
                "rigid-body MIP polygon edges must have non-zero length".to_string(),
            ));
        }
    }
    Ok(())
}

fn validate_config(config: &RigidBodyMipConfig2D) -> RoboticsResult<()> {
    if !config.min_x.is_finite()
        || !config.max_x.is_finite()
        || !config.min_y.is_finite()
        || !config.max_y.is_finite()
        || config.min_x >= config.max_x
        || config.min_y >= config.max_y
    {
        return Err(RoboticsError::InvalidParameter(
            "rigid-body MIP workspace bounds must be finite and ordered".to_string(),
        ));
    }
    if !config.position_step.is_finite()
        || !config.robot_half_length.is_finite()
        || !config.robot_half_width.is_finite()
        || !config.clearance.is_finite()
        || config.position_step <= 0.0
        || config.robot_half_length <= 0.0
        || config.robot_half_width <= 0.0
        || config.clearance < 0.0
    {
        return Err(RoboticsError::InvalidParameter(
            "rigid-body MIP step, robot dimensions, and clearance must be valid".to_string(),
        ));
    }
    if config.heading_count < 4 || config.move_cost == 0 || config.turn_cost == 0 {
        return Err(RoboticsError::InvalidParameter(
            "rigid-body MIP heading count and action costs must be positive".to_string(),
        ));
    }
    if config.max_expansions == 0 {
        return Err(RoboticsError::InvalidParameter(
            "rigid-body MIP max_expansions must be positive".to_string(),
        ));
    }
    let steps_x = (config.max_x - config.min_x) / config.position_step;
    let steps_y = (config.max_y - config.min_y) / config.position_step;
    if (steps_x.round() - steps_x).abs() > 1.0e-6 || (steps_y.round() - steps_y).abs() > 1.0e-6 {
        return Err(RoboticsError::InvalidParameter(
            "rigid-body MIP bounds must align with position_step".to_string(),
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn corridor_planner() -> RigidBodyMipPlanner2D {
        let obstacles = vec![
            RigidBodyConvexObstacle2D::aabb(3.0, 7.0, 0.0, 2.6).unwrap(),
            RigidBodyConvexObstacle2D::aabb(3.0, 7.0, 3.4, 6.0).unwrap(),
        ];
        RigidBodyMipPlanner2D::new(RigidBodyMipConfig2D {
            obstacles,
            max_expansions: 50_000,
            ..RigidBodyMipConfig2D::new(0.0, 10.0, 0.0, 6.0)
        })
        .unwrap()
    }

    #[test]
    fn halfspace_certificate_separates_pose_from_obstacle() {
        let planner = corridor_planner();
        let pose = RigidBodyPose2D::new(4.0, 3.0, 0.0);
        let certificates = planner.separation_certificates(pose).unwrap();

        assert_eq!(certificates.len(), 2);
        assert!(certificates[0].margin > planner.config.clearance);
        assert!(certificates[1].margin > planner.config.clearance);
    }

    #[test]
    fn colliding_pose_has_no_certificate() {
        let planner = corridor_planner();
        let pose = RigidBodyPose2D::new(4.0, 2.4, 0.0);

        assert!(planner.separation_certificates(pose).is_none());
    }

    #[test]
    fn convex_polygon_constructor_accepts_clockwise_vertices() {
        let obstacle = RigidBodyConvexObstacle2D::convex_polygon(vec![
            RigidBodyPoint2D::new(2.0, 2.0),
            RigidBodyPoint2D::new(2.0, 1.0),
            RigidBodyPoint2D::new(3.0, 1.5),
        ])
        .unwrap();
        let planner = RigidBodyMipPlanner2D::new(RigidBodyMipConfig2D {
            obstacles: vec![obstacle],
            robot_half_length: 0.15,
            robot_half_width: 0.15,
            ..RigidBodyMipConfig2D::new(0.0, 5.0, 0.0, 5.0)
        })
        .unwrap();

        assert!(planner
            .separation_certificates(RigidBodyPose2D::new(0.8, 1.5, 0.0))
            .is_some());
        assert!(planner
            .separation_certificates(RigidBodyPose2D::new(2.35, 1.5, 0.0))
            .is_none());
    }

    #[test]
    fn segment_certificate_catches_midpoint_collision() {
        let planner = RigidBodyMipPlanner2D::new(RigidBodyMipConfig2D {
            obstacles: vec![RigidBodyConvexObstacle2D::aabb(1.45, 1.55, 0.8, 1.2).unwrap()],
            robot_half_length: 0.10,
            robot_half_width: 0.10,
            clearance: 0.0,
            ..RigidBodyMipConfig2D::new(0.0, 3.0, 0.0, 2.0)
        })
        .unwrap();
        let from = RigidBodyPose2D::new(1.0, 1.0, 0.0);
        let to = RigidBodyPose2D::new(2.0, 1.0, 0.0);

        assert!(planner.separation_certificates(from).is_some());
        assert!(planner.separation_certificates(to).is_some());
        assert!(planner.segment_separation_certificates(from, to).is_none());
    }

    #[test]
    fn planner_rotates_through_narrow_gap() {
        let planner = corridor_planner();
        let start = RigidBodyPose2D::new(1.0, 3.0, TAU / 4.0);
        let goal = RigidBodyPose2D::new(9.0, 3.0, 0.0);
        let plan = planner.plan(start, goal, true).unwrap();

        assert!(plan.poses.len() > 2);
        assert!(plan.expanded_states > 0);
        assert!(plan.binary_separation_choices >= plan.poses.len() * 2);
        assert_eq!(plan.segment_certificates.len(), plan.poses.len() - 1);
        assert!(plan.segment_binary_separation_choices >= plan.segment_certificates.len() * 2);
        assert!(plan.min_separation_margin > planner.config.clearance);
        assert!(plan.min_segment_separation_margin > planner.config.clearance);
        assert!((plan.poses.last().unwrap().theta - 0.0).abs() < 1.0e-9);
        assert!(plan
            .poses
            .iter()
            .filter(|pose| pose.x >= 3.0 && pose.x <= 7.0)
            .all(|pose| pose.theta.abs() < 1.0e-9 || (pose.theta - TAU / 2.0).abs() < 1.0e-9));
    }
}
