//! Kinodynamic-oriented STL-CBS planning.
//!
//! This module extends the grid STL-CBS idea with heading states and
//! time-consuming motion primitives. It keeps the high-level CBS structure but
//! replans each constrained agent with an A* search over `(x, y, heading, t)`.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

use rust_robotics_core::{RoboticsError, RoboticsResult};

use crate::stl_cbs::{
    first_conflict, stl_pairwise_separation_robustness, StlCbsConflict, StlCbsConflictKind,
    StlCbsPath, StlTimeInterval, StlTimedCell,
};

/// Four-connected heading for the kinodynamic grid state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum KinodynamicHeading2D {
    East,
    North,
    West,
    South,
}

impl KinodynamicHeading2D {
    pub fn left(self) -> Self {
        match self {
            Self::East => Self::North,
            Self::North => Self::West,
            Self::West => Self::South,
            Self::South => Self::East,
        }
    }

    pub fn right(self) -> Self {
        match self {
            Self::East => Self::South,
            Self::South => Self::West,
            Self::West => Self::North,
            Self::North => Self::East,
        }
    }

    pub fn reverse(self) -> Self {
        self.left().left()
    }

    pub fn delta(self) -> (i32, i32) {
        match self {
            Self::East => (1, 0),
            Self::North => (0, 1),
            Self::West => (-1, 0),
            Self::South => (0, -1),
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::East => "E",
            Self::North => "N",
            Self::West => "W",
            Self::South => "S",
        }
    }

    pub fn angle_degrees(self) -> f64 {
        match self {
            Self::East => 0.0,
            Self::North => -90.0,
            Self::West => 180.0,
            Self::South => 90.0,
        }
    }
}

/// Timed oriented pose.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct KinodynamicTimedPose2D {
    pub x: i32,
    pub y: i32,
    pub heading: KinodynamicHeading2D,
    pub t: u64,
}

impl KinodynamicTimedPose2D {
    pub fn new(x: i32, y: i32, heading: KinodynamicHeading2D, t: u64) -> Self {
        Self { x, y, heading, t }
    }

    pub fn cell(self) -> StlTimedCell {
        StlTimedCell::new(self.x, self.y, self.t)
    }
}

/// Continuous pose sampled between timed grid poses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KinodynamicContinuousPose2D {
    pub x: f64,
    pub y: f64,
    pub t: f64,
}

impl KinodynamicContinuousPose2D {
    pub fn new(x: f64, y: f64, t: f64) -> Self {
        Self { x, y, t }
    }
}

/// Agent query with start heading and optional terminal heading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KinodynamicStlCbsAgent2D {
    pub id: usize,
    pub start: (i32, i32, KinodynamicHeading2D),
    pub goal: (i32, i32),
    pub goal_heading: Option<KinodynamicHeading2D>,
}

impl KinodynamicStlCbsAgent2D {
    pub fn new(
        id: usize,
        start: (i32, i32),
        heading: KinodynamicHeading2D,
        goal: (i32, i32),
    ) -> Self {
        Self {
            id,
            start: (start.0, start.1, heading),
            goal,
            goal_heading: None,
        }
    }

    pub fn with_goal_heading(mut self, heading: KinodynamicHeading2D) -> Self {
        self.goal_heading = Some(heading);
        self
    }
}

/// Planner configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct KinodynamicStlCbsConfig2D {
    pub width: i32,
    pub height: i32,
    pub obstacle_map: Vec<Vec<bool>>,
    pub max_time: u64,
    pub max_cbs_nodes: usize,
    pub move_duration: u64,
    pub turn_duration: u64,
    pub wait_duration: u64,
    pub allow_wait: bool,
    pub allow_reverse: bool,
}

impl KinodynamicStlCbsConfig2D {
    pub fn new(width: i32, height: i32) -> Self {
        Self {
            width,
            height,
            obstacle_map: vec![vec![false; height.max(0) as usize]; width.max(0) as usize],
            max_time: 96,
            max_cbs_nodes: 2_048,
            move_duration: 2,
            turn_duration: 1,
            wait_duration: 1,
            allow_wait: true,
            allow_reverse: false,
        }
    }
}

/// One kinodynamic agent path.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KinodynamicStlCbsPath2D {
    pub agent_id: usize,
    pub poses: Vec<KinodynamicTimedPose2D>,
}

impl KinodynamicStlCbsPath2D {
    pub fn arrival_time(&self) -> u64 {
        self.poses.last().map_or(0, |pose| pose.t)
    }

    pub fn pose_at(&self, t: u64) -> KinodynamicTimedPose2D {
        self.poses
            .iter()
            .find(|pose| pose.t == t)
            .copied()
            .or_else(|| self.poses.iter().rev().find(|pose| pose.t < t).copied())
            .unwrap_or_else(|| {
                *self
                    .poses
                    .first()
                    .expect("validated kinodynamic STL-CBS path is non-empty")
            })
    }

    pub fn continuous_pose_at(&self, t: f64) -> KinodynamicContinuousPose2D {
        let first = *self
            .poses
            .first()
            .expect("validated kinodynamic STL-CBS path is non-empty");
        if t <= first.t as f64 {
            return KinodynamicContinuousPose2D::new(first.x as f64, first.y as f64, t);
        }
        let last = *self
            .poses
            .last()
            .expect("validated kinodynamic STL-CBS path is non-empty");
        if t >= last.t as f64 {
            return KinodynamicContinuousPose2D::new(last.x as f64, last.y as f64, t);
        }
        for window in self.poses.windows(2) {
            let from = window[0];
            let to = window[1];
            if t >= from.t as f64 && t <= to.t as f64 {
                let dt = (to.t - from.t) as f64;
                let alpha = if dt <= f64::EPSILON {
                    0.0
                } else {
                    (t - from.t as f64) / dt
                };
                return KinodynamicContinuousPose2D::new(
                    from.x as f64 + (to.x - from.x) as f64 * alpha,
                    from.y as f64 + (to.y - from.y) as f64 * alpha,
                    t,
                );
            }
        }
        KinodynamicContinuousPose2D::new(last.x as f64, last.y as f64, t)
    }

    pub fn cell_path(&self) -> StlCbsPath {
        StlCbsPath {
            agent_id: self.agent_id,
            waypoints: self.poses.iter().map(|pose| pose.cell()).collect(),
        }
    }
}

/// Kinodynamic CBS result.
#[derive(Debug, Clone, PartialEq)]
pub struct KinodynamicStlCbsPlan2D {
    pub paths: Vec<KinodynamicStlCbsPath2D>,
    pub cell_paths: Vec<StlCbsPath>,
    pub total_cost: u64,
    pub conflicts_resolved: usize,
    pub continuous_conflicts_resolved: usize,
    pub high_level_nodes_expanded: usize,
    pub min_pairwise_separation_robustness: f64,
    pub min_continuous_pairwise_separation_robustness: f64,
    pub first_continuous_conflict: Option<KinodynamicContinuousConflict2D>,
}

/// Continuous-time pairwise conflict found between integer ticks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KinodynamicContinuousConflict2D {
    pub agent_a: usize,
    pub agent_b: usize,
    pub t: f64,
    pub distance: f64,
    pub min_distance: f64,
    pub ax: f64,
    pub ay: f64,
    pub bx: f64,
    pub by: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum KinodynamicConstraintKind {
    Vertex {
        x: i32,
        y: i32,
        t: u64,
    },
    Edge {
        from_x: i32,
        from_y: i32,
        to_x: i32,
        to_y: i32,
        t: u64,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct KinodynamicConstraint {
    agent_id: usize,
    kind: KinodynamicConstraintKind,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct CbsNode {
    constraints: Vec<KinodynamicConstraint>,
    paths: Vec<KinodynamicStlCbsPath2D>,
    total_cost: u64,
    conflicts_resolved: usize,
    continuous_conflicts_resolved: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct CbsOpenEntry {
    node_index: usize,
    total_cost: u64,
    conflicts_resolved: usize,
}

impl Ord for CbsOpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .total_cost
            .cmp(&self.total_cost)
            .then_with(|| other.conflicts_resolved.cmp(&self.conflicts_resolved))
            .then_with(|| other.node_index.cmp(&self.node_index))
    }
}

impl PartialOrd for CbsOpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct SearchState {
    x: i32,
    y: i32,
    heading: KinodynamicHeading2D,
    t: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SearchOpenEntry {
    state: SearchState,
    f: u64,
    g: u64,
}

impl Ord for SearchOpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f
            .cmp(&self.f)
            .then_with(|| other.g.cmp(&self.g))
            .then_with(|| other.state.t.cmp(&self.state.t))
            .then_with(|| other.state.heading.cmp(&self.state.heading))
    }
}

impl PartialOrd for SearchOpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ParentLink {
    previous: Option<SearchState>,
    samples: Vec<KinodynamicTimedPose2D>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct KinodynamicStlCbsPlanner2D {
    config: KinodynamicStlCbsConfig2D,
}

impl KinodynamicStlCbsPlanner2D {
    pub fn new(config: KinodynamicStlCbsConfig2D) -> RoboticsResult<Self> {
        validate_config(&config)?;
        Ok(Self { config })
    }

    pub fn config(&self) -> &KinodynamicStlCbsConfig2D {
        &self.config
    }

    pub fn plan_independent(
        &self,
        agents: &[KinodynamicStlCbsAgent2D],
    ) -> RoboticsResult<Vec<KinodynamicStlCbsPath2D>> {
        self.validate_agents(agents)?;
        agents
            .iter()
            .map(|agent| self.plan_agent(*agent, &[]))
            .collect()
    }

    pub fn plan(
        &self,
        agents: &[KinodynamicStlCbsAgent2D],
    ) -> RoboticsResult<KinodynamicStlCbsPlan2D> {
        self.validate_agents(agents)?;
        let root_paths = self.plan_independent(agents)?;
        let root = CbsNode {
            total_cost: total_path_cost(&root_paths),
            constraints: Vec::new(),
            paths: root_paths,
            conflicts_resolved: 0,
            continuous_conflicts_resolved: 0,
        };

        let mut nodes = vec![root];
        let mut open = BinaryHeap::new();
        open.push(CbsOpenEntry {
            node_index: 0,
            total_cost: nodes[0].total_cost,
            conflicts_resolved: 0,
        });
        let mut expanded = 0;

        while let Some(entry) = open.pop() {
            expanded += 1;
            if expanded > self.config.max_cbs_nodes {
                return Err(RoboticsError::PlanningError(
                    "kinodynamic STL-CBS exceeded high-level node limit".to_string(),
                ));
            }
            let node = nodes[entry.node_index].clone();
            let cell_paths = cell_paths_from(&node.paths);
            if let Some(conflict) = first_conflict(&cell_paths, self.config.max_time) {
                for constrained_agent in [conflict.agent_a, conflict.agent_b] {
                    let mut constraints = node.constraints.clone();
                    constraints.push(constraint_from_conflict(conflict, constrained_agent));
                    let agent = *agents
                        .iter()
                        .find(|agent| agent.id == constrained_agent)
                        .expect("validated conflict references a known agent");
                    let agent_constraints = constraints_for_agent(&constraints, constrained_agent);
                    let Ok(replanned_path) = self.plan_agent(agent, &agent_constraints) else {
                        continue;
                    };
                    let mut paths = node.paths.clone();
                    let path_index = paths
                        .iter()
                        .position(|path| path.agent_id == constrained_agent)
                        .expect("validated node contains all agent paths");
                    paths[path_index] = replanned_path;
                    let child = CbsNode {
                        total_cost: total_path_cost(&paths),
                        constraints,
                        paths,
                        conflicts_resolved: node.conflicts_resolved + 1,
                        continuous_conflicts_resolved: node.continuous_conflicts_resolved,
                    };
                    let node_index = nodes.len();
                    open.push(CbsOpenEntry {
                        node_index,
                        total_cost: child.total_cost,
                        conflicts_resolved: child.conflicts_resolved,
                    });
                    nodes.push(child);
                }
            } else if let Some(continuous_conflict) =
                first_kinodynamic_continuous_conflict(&node.paths, 1.0, self.config.max_time)?
            {
                // Integer-time CBS is clean, but the continuous-time occupancy
                // check between ticks still found a violation. Convert it into
                // a high-level CBS branch on each involved agent.
                for constrained_agent in [continuous_conflict.agent_a, continuous_conflict.agent_b]
                {
                    let Some(new_constraint) = continuous_constraint_from_conflict(
                        &continuous_conflict,
                        &node.paths,
                        constrained_agent,
                    ) else {
                        continue;
                    };
                    let mut constraints = node.constraints.clone();
                    if node.constraints.contains(&new_constraint) {
                        // The same continuous constraint is already active; adding
                        // it again would not change the replan, so skip this branch.
                        continue;
                    }
                    constraints.push(new_constraint);
                    let agent = *agents
                        .iter()
                        .find(|agent| agent.id == constrained_agent)
                        .expect("validated conflict references a known agent");
                    let agent_constraints = constraints_for_agent(&constraints, constrained_agent);
                    let Ok(replanned_path) = self.plan_agent(agent, &agent_constraints) else {
                        continue;
                    };
                    let mut paths = node.paths.clone();
                    let path_index = paths
                        .iter()
                        .position(|path| path.agent_id == constrained_agent)
                        .expect("validated node contains all agent paths");
                    paths[path_index] = replanned_path;
                    let child = CbsNode {
                        total_cost: total_path_cost(&paths),
                        constraints,
                        paths,
                        conflicts_resolved: node.conflicts_resolved,
                        continuous_conflicts_resolved: node.continuous_conflicts_resolved + 1,
                    };
                    let node_index = nodes.len();
                    open.push(CbsOpenEntry {
                        node_index,
                        total_cost: child.total_cost,
                        conflicts_resolved: child.conflicts_resolved,
                    });
                    nodes.push(child);
                }
            } else {
                let cell_paths = cell_paths_from(&node.paths);
                let min_continuous_pairwise_separation_robustness =
                    kinodynamic_continuous_pairwise_separation_robustness(
                        &node.paths,
                        1.0,
                        self.config.max_time,
                    )?;
                let first_continuous_conflict =
                    first_kinodynamic_continuous_conflict(&node.paths, 1.0, self.config.max_time)?;
                return Ok(KinodynamicStlCbsPlan2D {
                    total_cost: node.total_cost,
                    min_pairwise_separation_robustness: stl_pairwise_separation_robustness(
                        &cell_paths,
                        1.0,
                        StlTimeInterval {
                            start: 0,
                            end: self.config.max_time,
                        },
                    )?,
                    cell_paths,
                    paths: node.paths,
                    conflicts_resolved: node.conflicts_resolved,
                    continuous_conflicts_resolved: node.continuous_conflicts_resolved,
                    high_level_nodes_expanded: expanded,
                    min_continuous_pairwise_separation_robustness,
                    first_continuous_conflict,
                });
            }
        }

        Err(RoboticsError::PlanningError(
            "kinodynamic STL-CBS could not find a conflict-free plan".to_string(),
        ))
    }

    fn plan_agent(
        &self,
        agent: KinodynamicStlCbsAgent2D,
        constraints: &[KinodynamicConstraintKind],
    ) -> RoboticsResult<KinodynamicStlCbsPath2D> {
        if self.violates_vertex_constraint(agent.start.0, agent.start.1, 0, constraints) {
            return Err(RoboticsError::PlanningError(
                "agent start violates a kinodynamic CBS vertex constraint".to_string(),
            ));
        }
        let start = SearchState {
            x: agent.start.0,
            y: agent.start.1,
            heading: agent.start.2,
            t: 0,
        };
        let mut open = BinaryHeap::new();
        let mut best_g = HashMap::new();
        let mut parent = HashMap::new();

        best_g.insert(start, 0);
        parent.insert(
            start,
            ParentLink {
                previous: None,
                samples: vec![KinodynamicTimedPose2D::new(
                    start.x,
                    start.y,
                    start.heading,
                    start.t,
                )],
            },
        );
        open.push(SearchOpenEntry {
            state: start,
            g: 0,
            f: self.heuristic(start, agent),
        });

        while let Some(entry) = open.pop() {
            if entry.state.t > self.config.max_time {
                continue;
            }
            if entry.g > *best_g.get(&entry.state).unwrap_or(&u64::MAX) {
                continue;
            }
            if self.goal_reached(entry.state, agent)
                && !self.violates_future_hold(entry.state, constraints)
            {
                return Ok(KinodynamicStlCbsPath2D {
                    agent_id: agent.id,
                    poses: reconstruct_path(&parent, entry.state, agent.id),
                });
            }

            for (next, samples) in self.successors(entry.state) {
                if next.t > self.config.max_time {
                    continue;
                }
                if self.violates_transition(entry.state, &samples, constraints) {
                    continue;
                }
                let next_g = entry.g + (next.t - entry.state.t);
                if next_g < *best_g.get(&next).unwrap_or(&u64::MAX) {
                    best_g.insert(next, next_g);
                    parent.insert(
                        next,
                        ParentLink {
                            previous: Some(entry.state),
                            samples,
                        },
                    );
                    open.push(SearchOpenEntry {
                        state: next,
                        g: next_g,
                        f: next_g + self.heuristic(next, agent),
                    });
                }
            }
        }

        Err(RoboticsError::PlanningError(format!(
            "no kinodynamic path found for agent {}",
            agent.id
        )))
    }

    fn successors(&self, state: SearchState) -> Vec<(SearchState, Vec<KinodynamicTimedPose2D>)> {
        let mut successors = Vec::new();
        if self.config.allow_wait {
            successors.push(self.same_cell_successor(
                state,
                state.heading,
                self.config.wait_duration,
            ));
        }
        successors.push(self.same_cell_successor(
            state,
            state.heading.left(),
            self.config.turn_duration,
        ));
        successors.push(self.same_cell_successor(
            state,
            state.heading.right(),
            self.config.turn_duration,
        ));
        if let Some(forward) = self.move_successor(state, state.heading, self.config.move_duration)
        {
            successors.push(forward);
        }
        if self.config.allow_reverse {
            if let Some(reverse) =
                self.move_successor(state, state.heading.reverse(), self.config.move_duration)
            {
                successors.push(reverse);
            }
        }
        successors
    }

    fn same_cell_successor(
        &self,
        state: SearchState,
        heading: KinodynamicHeading2D,
        duration: u64,
    ) -> (SearchState, Vec<KinodynamicTimedPose2D>) {
        let next = SearchState {
            x: state.x,
            y: state.y,
            heading,
            t: state.t + duration,
        };
        let samples = (1..=duration)
            .map(|dt| KinodynamicTimedPose2D::new(state.x, state.y, heading, state.t + dt))
            .collect();
        (next, samples)
    }

    fn move_successor(
        &self,
        state: SearchState,
        movement_heading: KinodynamicHeading2D,
        duration: u64,
    ) -> Option<(SearchState, Vec<KinodynamicTimedPose2D>)> {
        let (dx, dy) = movement_heading.delta();
        let nx = state.x + dx;
        let ny = state.y + dy;
        if !self.is_free(nx, ny) {
            return None;
        }
        let next = SearchState {
            x: nx,
            y: ny,
            heading: state.heading,
            t: state.t + duration,
        };
        let samples = (1..=duration)
            .map(|dt| {
                if dt == duration {
                    KinodynamicTimedPose2D::new(nx, ny, state.heading, state.t + dt)
                } else {
                    KinodynamicTimedPose2D::new(state.x, state.y, state.heading, state.t + dt)
                }
            })
            .collect();
        Some((next, samples))
    }

    fn goal_reached(&self, state: SearchState, agent: KinodynamicStlCbsAgent2D) -> bool {
        (state.x, state.y) == agent.goal
            && agent
                .goal_heading
                .map_or(true, |goal_heading| goal_heading == state.heading)
    }

    fn heuristic(&self, state: SearchState, agent: KinodynamicStlCbsAgent2D) -> u64 {
        manhattan((state.x, state.y), agent.goal) * self.config.move_duration
    }

    fn violates_transition(
        &self,
        state: SearchState,
        samples: &[KinodynamicTimedPose2D],
        constraints: &[KinodynamicConstraintKind],
    ) -> bool {
        let mut previous = KinodynamicTimedPose2D::new(state.x, state.y, state.heading, state.t);
        for &sample in samples {
            if !self.is_free(sample.x, sample.y)
                || self.violates_vertex_constraint(sample.x, sample.y, sample.t, constraints)
            {
                return true;
            }
            if (previous.x, previous.y) != (sample.x, sample.y)
                && self.violates_edge_constraint(
                    previous.x,
                    previous.y,
                    sample.x,
                    sample.y,
                    sample.t,
                    constraints,
                )
            {
                return true;
            }
            previous = sample;
        }
        false
    }

    fn violates_future_hold(
        &self,
        state: SearchState,
        constraints: &[KinodynamicConstraintKind],
    ) -> bool {
        (state.t..=self.config.max_time)
            .any(|t| self.violates_vertex_constraint(state.x, state.y, t, constraints))
    }

    fn violates_vertex_constraint(
        &self,
        x: i32,
        y: i32,
        t: u64,
        constraints: &[KinodynamicConstraintKind],
    ) -> bool {
        constraints.iter().any(|constraint| {
            matches!(
                constraint,
                KinodynamicConstraintKind::Vertex { x: cx, y: cy, t: ct }
                    if *cx == x && *cy == y && *ct == t
            )
        })
    }

    fn violates_edge_constraint(
        &self,
        from_x: i32,
        from_y: i32,
        to_x: i32,
        to_y: i32,
        t: u64,
        constraints: &[KinodynamicConstraintKind],
    ) -> bool {
        constraints.iter().any(|constraint| {
            matches!(
                constraint,
                KinodynamicConstraintKind::Edge {
                    from_x: cx0,
                    from_y: cy0,
                    to_x: cx1,
                    to_y: cy1,
                    t: ct,
                } if *cx0 == from_x
                    && *cy0 == from_y
                    && *cx1 == to_x
                    && *cy1 == to_y
                    && *ct == t
            )
        })
    }

    fn validate_agents(&self, agents: &[KinodynamicStlCbsAgent2D]) -> RoboticsResult<()> {
        if agents.is_empty() {
            return Err(RoboticsError::InvalidParameter(
                "kinodynamic STL-CBS requires at least one agent".to_string(),
            ));
        }
        let mut ids = HashSet::new();
        for agent in agents {
            if !ids.insert(agent.id) {
                return Err(RoboticsError::InvalidParameter(
                    "kinodynamic STL-CBS agent ids must be unique".to_string(),
                ));
            }
            if !self.is_free(agent.start.0, agent.start.1)
                || !self.is_free(agent.goal.0, agent.goal.1)
            {
                return Err(RoboticsError::InvalidParameter(format!(
                    "agent {} start/goal must be free grid cells",
                    agent.id
                )));
            }
        }
        Ok(())
    }

    fn is_free(&self, x: i32, y: i32) -> bool {
        x >= 0
            && y >= 0
            && x < self.config.width
            && y < self.config.height
            && !self.config.obstacle_map[x as usize][y as usize]
    }
}

fn cell_paths_from(paths: &[KinodynamicStlCbsPath2D]) -> Vec<StlCbsPath> {
    paths
        .iter()
        .map(KinodynamicStlCbsPath2D::cell_path)
        .collect()
}

fn constraint_from_conflict(
    conflict: StlCbsConflict,
    constrained_agent: usize,
) -> KinodynamicConstraint {
    let kind = match conflict.kind {
        StlCbsConflictKind::Vertex { x, y, t } => KinodynamicConstraintKind::Vertex { x, y, t },
        StlCbsConflictKind::Edge {
            ax0,
            ay0,
            ax1,
            ay1,
            bx0,
            by0,
            bx1,
            by1,
            t,
        } => {
            if constrained_agent == conflict.agent_a {
                KinodynamicConstraintKind::Edge {
                    from_x: ax0,
                    from_y: ay0,
                    to_x: ax1,
                    to_y: ay1,
                    t,
                }
            } else {
                KinodynamicConstraintKind::Edge {
                    from_x: bx0,
                    from_y: by0,
                    to_x: bx1,
                    to_y: by1,
                    t,
                }
            }
        }
    };
    KinodynamicConstraint {
        agent_id: constrained_agent,
        kind,
    }
}

/// Turn a continuous-time conflict into a discrete CBS constraint for one agent.
///
/// The conflict happens at continuous time `t`. Within the integer interval
/// `[floor(t), floor(t) + 1]` the constrained agent either holds a cell or
/// traverses an edge. We forbid exactly that behavior at the end tick so the
/// replan must change its sub-tick timing (typically by waiting), which removes
/// the between-tick near-miss instead of merely shifting it.
fn continuous_constraint_from_conflict(
    conflict: &KinodynamicContinuousConflict2D,
    paths: &[KinodynamicStlCbsPath2D],
    constrained_agent: usize,
) -> Option<KinodynamicConstraint> {
    let tick = conflict.t.floor() as u64;
    let path = paths
        .iter()
        .find(|path| path.agent_id == constrained_agent)?;
    let from = path.pose_at(tick);
    let to = path.pose_at(tick + 1);
    let kind = if (from.x, from.y) == (to.x, to.y) {
        KinodynamicConstraintKind::Vertex {
            x: from.x,
            y: from.y,
            t: tick + 1,
        }
    } else {
        KinodynamicConstraintKind::Edge {
            from_x: from.x,
            from_y: from.y,
            to_x: to.x,
            to_y: to.y,
            t: tick + 1,
        }
    };
    Some(KinodynamicConstraint {
        agent_id: constrained_agent,
        kind,
    })
}

fn constraints_for_agent(
    constraints: &[KinodynamicConstraint],
    agent_id: usize,
) -> Vec<KinodynamicConstraintKind> {
    constraints
        .iter()
        .filter(|constraint| constraint.agent_id == agent_id)
        .map(|constraint| constraint.kind)
        .collect()
}

fn total_path_cost(paths: &[KinodynamicStlCbsPath2D]) -> u64 {
    paths
        .iter()
        .map(KinodynamicStlCbsPath2D::arrival_time)
        .sum()
}

pub fn kinodynamic_continuous_pairwise_separation_robustness(
    paths: &[KinodynamicStlCbsPath2D],
    min_distance: f64,
    max_time: u64,
) -> RoboticsResult<f64> {
    validate_continuous_query(paths, min_distance)?;
    if paths.len() < 2 {
        return Ok(f64::INFINITY);
    }
    let mut robustness = f64::INFINITY;
    for t in 0..max_time {
        let t0 = t as f64;
        let t1 = (t + 1) as f64;
        for i in 0..paths.len() {
            for j in i + 1..paths.len() {
                let a0 = paths[i].continuous_pose_at(t0);
                let a1 = paths[i].continuous_pose_at(t1);
                let b0 = paths[j].continuous_pose_at(t0);
                let b1 = paths[j].continuous_pose_at(t1);
                let (_, distance) = closest_relative_segment_distance(a0, a1, b0, b1);
                robustness = robustness.min(distance - min_distance);
            }
        }
    }
    Ok(robustness)
}

pub fn first_kinodynamic_continuous_conflict(
    paths: &[KinodynamicStlCbsPath2D],
    min_distance: f64,
    max_time: u64,
) -> RoboticsResult<Option<KinodynamicContinuousConflict2D>> {
    validate_continuous_query(paths, min_distance)?;
    if paths.len() < 2 {
        return Ok(None);
    }
    for t in 0..max_time {
        let t0 = t as f64;
        let t1 = (t + 1) as f64;
        for i in 0..paths.len() {
            for j in i + 1..paths.len() {
                let a0 = paths[i].continuous_pose_at(t0);
                let a1 = paths[i].continuous_pose_at(t1);
                let b0 = paths[j].continuous_pose_at(t0);
                let b1 = paths[j].continuous_pose_at(t1);
                if let Some((alpha, distance)) =
                    earliest_continuous_violation_alpha(a0, a1, b0, b1, min_distance)
                {
                    let time = t0 + alpha;
                    let a = interpolate_continuous_pose(a0, a1, alpha);
                    let b = interpolate_continuous_pose(b0, b1, alpha);
                    return Ok(Some(KinodynamicContinuousConflict2D {
                        agent_a: paths[i].agent_id,
                        agent_b: paths[j].agent_id,
                        t: time,
                        distance,
                        min_distance,
                        ax: a.x,
                        ay: a.y,
                        bx: b.x,
                        by: b.y,
                    }));
                }
            }
        }
    }
    Ok(None)
}

fn validate_continuous_query(
    paths: &[KinodynamicStlCbsPath2D],
    min_distance: f64,
) -> RoboticsResult<()> {
    if !min_distance.is_finite() || min_distance < 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "kinodynamic continuous min_distance must be finite and non-negative".to_string(),
        ));
    }
    for path in paths {
        if path.poses.is_empty() {
            return Err(RoboticsError::InvalidParameter(
                "kinodynamic continuous path must contain poses".to_string(),
            ));
        }
        for window in path.poses.windows(2) {
            if window[1].t <= window[0].t {
                return Err(RoboticsError::InvalidParameter(
                    "kinodynamic continuous path times must be strictly increasing".to_string(),
                ));
            }
        }
    }
    Ok(())
}

fn closest_relative_segment_distance(
    a0: KinodynamicContinuousPose2D,
    a1: KinodynamicContinuousPose2D,
    b0: KinodynamicContinuousPose2D,
    b1: KinodynamicContinuousPose2D,
) -> (f64, f64) {
    let r0x = a0.x - b0.x;
    let r0y = a0.y - b0.y;
    let vx = (a1.x - a0.x) - (b1.x - b0.x);
    let vy = (a1.y - a0.y) - (b1.y - b0.y);
    let vv = vx * vx + vy * vy;
    let alpha = if vv <= f64::EPSILON {
        0.0
    } else {
        (-(r0x * vx + r0y * vy) / vv).clamp(0.0, 1.0)
    };
    let dx = r0x + vx * alpha;
    let dy = r0y + vy * alpha;
    (alpha, (dx * dx + dy * dy).sqrt())
}

fn earliest_continuous_violation_alpha(
    a0: KinodynamicContinuousPose2D,
    a1: KinodynamicContinuousPose2D,
    b0: KinodynamicContinuousPose2D,
    b1: KinodynamicContinuousPose2D,
    min_distance: f64,
) -> Option<(f64, f64)> {
    let r0x = a0.x - b0.x;
    let r0y = a0.y - b0.y;
    let vx = (a1.x - a0.x) - (b1.x - b0.x);
    let vy = (a1.y - a0.y) - (b1.y - b0.y);
    let (_, closest_distance) = closest_relative_segment_distance(a0, a1, b0, b1);
    if closest_distance >= min_distance - 1.0e-9 {
        return None;
    }
    let radius2 = min_distance * min_distance;
    let start2 = r0x * r0x + r0y * r0y;
    if start2 < radius2 {
        return Some((0.0, start2.sqrt()));
    }

    let a = vx * vx + vy * vy;
    if a <= f64::EPSILON {
        return None;
    }
    let b = 2.0 * (r0x * vx + r0y * vy);
    let c = start2 - radius2;
    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return None;
    }
    let sqrt_discriminant = discriminant.sqrt();
    let mut roots = [
        (-b - sqrt_discriminant) / (2.0 * a),
        (-b + sqrt_discriminant) / (2.0 * a),
    ];
    roots.sort_by(|left, right| left.partial_cmp(right).unwrap_or(Ordering::Equal));
    for alpha in roots {
        if (-f64::EPSILON..=1.0 + f64::EPSILON).contains(&alpha) {
            let alpha = alpha.clamp(0.0, 1.0);
            let dx = r0x + vx * alpha;
            let dy = r0y + vy * alpha;
            return Some((alpha, (dx * dx + dy * dy).sqrt()));
        }
    }
    None
}

fn interpolate_continuous_pose(
    from: KinodynamicContinuousPose2D,
    to: KinodynamicContinuousPose2D,
    alpha: f64,
) -> KinodynamicContinuousPose2D {
    KinodynamicContinuousPose2D::new(
        from.x + (to.x - from.x) * alpha,
        from.y + (to.y - from.y) * alpha,
        from.t + (to.t - from.t) * alpha,
    )
}

fn reconstruct_path(
    parent: &HashMap<SearchState, ParentLink>,
    mut current: SearchState,
    agent_id: usize,
) -> Vec<KinodynamicTimedPose2D> {
    let mut chunks = Vec::new();
    loop {
        let link = parent
            .get(&current)
            .expect("search parent map contains every reached state");
        chunks.push(link.samples.clone());
        match link.previous {
            Some(previous) => current = previous,
            None => break,
        }
    }
    chunks.reverse();
    let _ = agent_id;
    chunks.into_iter().flatten().collect()
}

fn manhattan(a: (i32, i32), b: (i32, i32)) -> u64 {
    (a.0 - b.0).unsigned_abs() as u64 + (a.1 - b.1).unsigned_abs() as u64
}

fn validate_config(config: &KinodynamicStlCbsConfig2D) -> RoboticsResult<()> {
    if config.width <= 0 || config.height <= 0 {
        return Err(RoboticsError::InvalidParameter(
            "kinodynamic STL-CBS width and height must be positive".to_string(),
        ));
    }
    if config.max_time == 0
        || config.max_cbs_nodes == 0
        || config.move_duration == 0
        || config.turn_duration == 0
        || config.wait_duration == 0
    {
        return Err(RoboticsError::InvalidParameter(
            "kinodynamic STL-CBS time limits and primitive durations must be positive".to_string(),
        ));
    }
    if config.obstacle_map.len() != config.width as usize {
        return Err(RoboticsError::InvalidParameter(
            "kinodynamic STL-CBS obstacle_map x-dimension must match width".to_string(),
        ));
    }
    for column in &config.obstacle_map {
        if column.len() != config.height as usize {
            return Err(RoboticsError::InvalidParameter(
                "kinodynamic STL-CBS obstacle_map y-dimension must match height".to_string(),
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn planner() -> KinodynamicStlCbsPlanner2D {
        KinodynamicStlCbsPlanner2D::new(KinodynamicStlCbsConfig2D {
            max_time: 32,
            max_cbs_nodes: 4_096,
            ..KinodynamicStlCbsConfig2D::new(7, 5)
        })
        .unwrap()
    }

    #[test]
    fn heading_primitives_make_paths_time_aware() {
        let planner = planner();
        let agent = KinodynamicStlCbsAgent2D::new(0, (0, 1), KinodynamicHeading2D::East, (2, 2));
        let path = planner.plan_independent(&[agent]).unwrap().remove(0);

        assert!(path.arrival_time() > manhattan((0, 1), (2, 2)));
        assert!(path
            .poses
            .iter()
            .any(|pose| pose.heading == KinodynamicHeading2D::North));
    }

    #[test]
    fn goal_heading_is_respected() {
        let planner = planner();
        let agent = KinodynamicStlCbsAgent2D::new(0, (0, 0), KinodynamicHeading2D::East, (2, 0))
            .with_goal_heading(KinodynamicHeading2D::West);
        let path = planner.plan_independent(&[agent]).unwrap().remove(0);

        assert_eq!(
            path.poses.last().unwrap().heading,
            KinodynamicHeading2D::West
        );
    }

    #[test]
    fn continuous_conflict_detects_between_tick_crossing() {
        let paths = vec![
            KinodynamicStlCbsPath2D {
                agent_id: 0,
                poses: vec![
                    KinodynamicTimedPose2D::new(0, 0, KinodynamicHeading2D::East, 0),
                    KinodynamicTimedPose2D::new(1, 1, KinodynamicHeading2D::East, 1),
                ],
            },
            KinodynamicStlCbsPath2D {
                agent_id: 1,
                poses: vec![
                    KinodynamicTimedPose2D::new(0, 1, KinodynamicHeading2D::East, 0),
                    KinodynamicTimedPose2D::new(1, 0, KinodynamicHeading2D::East, 1),
                ],
            },
        ];

        assert!(first_conflict(&cell_paths_from(&paths), 1).is_none());
        let robustness =
            kinodynamic_continuous_pairwise_separation_robustness(&paths, 0.2, 1).unwrap();
        let conflict = first_kinodynamic_continuous_conflict(&paths, 0.2, 1)
            .unwrap()
            .unwrap();

        assert!(robustness < 0.0);
        assert!((conflict.t - 0.4).abs() < 0.11);
        assert_eq!(conflict.agent_a, 0);
        assert_eq!(conflict.agent_b, 1);
    }

    /// Two agents cross perpendicularly through a shared center cell. Integer-time
    /// CBS can separate them at every integer tick, yet a between-tick near-miss
    /// remains. This exercises the continuous-conflict CBS branch: the plan must
    /// resolve at least one continuous conflict and end continuously safe.
    #[test]
    fn cbs_resolves_continuous_only_perpendicular_conflict() {
        let planner = KinodynamicStlCbsPlanner2D::new(KinodynamicStlCbsConfig2D {
            max_time: 48,
            max_cbs_nodes: 8_192,
            move_duration: 1,
            turn_duration: 1,
            wait_duration: 1,
            allow_reverse: true,
            ..KinodynamicStlCbsConfig2D::new(5, 5)
        })
        .unwrap();
        let agents = [
            KinodynamicStlCbsAgent2D::new(0, (0, 2), KinodynamicHeading2D::East, (4, 2)),
            KinodynamicStlCbsAgent2D::new(1, (2, 0), KinodynamicHeading2D::North, (2, 4)),
        ];

        // Independent plans collide on the grid, so plain integer CBS is engaged.
        let independent = planner.plan_independent(&agents).unwrap();
        assert!(first_conflict(&cell_paths_from(&independent), 48).is_some());

        let plan = planner.plan(&agents).unwrap();

        // The between-tick crossing must have been handled by the continuous branch.
        assert!(
            plan.continuous_conflicts_resolved > 0,
            "expected at least one continuous-conflict CBS branch"
        );
        // The final plan is clean at every integer tick and in continuous time.
        assert!(first_conflict(&plan.cell_paths, 48).is_none());
        assert!(plan.first_continuous_conflict.is_none());
        assert!(plan.min_continuous_pairwise_separation_robustness >= 0.0);
        // Both agents still reach their goals.
        for (agent, path) in agents.iter().zip(plan.paths.iter()) {
            let last = path.poses.last().unwrap();
            assert_eq!((last.x, last.y), agent.goal);
        }
    }

    #[test]
    fn cbs_resolves_oriented_corridor_conflict() {
        let planner = planner();
        let agents = [
            KinodynamicStlCbsAgent2D::new(0, (0, 2), KinodynamicHeading2D::East, (6, 2)),
            KinodynamicStlCbsAgent2D::new(1, (6, 2), KinodynamicHeading2D::West, (0, 2)),
        ];
        let independent = planner.plan_independent(&agents).unwrap();
        assert!(first_conflict(&cell_paths_from(&independent), 32).is_some());

        let plan = planner.plan(&agents).unwrap();
        assert!(plan.conflicts_resolved > 0);
        assert!(first_conflict(&plan.cell_paths, 32).is_none());
        assert!(plan.min_pairwise_separation_robustness >= 0.0);
        assert!(plan.min_continuous_pairwise_separation_robustness >= 0.0);
        assert!(plan.first_continuous_conflict.is_none());
    }
}
