//! STL-CBS foundation for multi-agent grid planning.
//!
//! This module combines a compact conflict-based search (CBS) planner with
//! Signal Temporal Logic style robustness primitives over timed grid paths.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

use rust_robotics_core::{RoboticsError, RoboticsResult};

/// Timed grid cell used by STL-CBS paths.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct StlTimedCell {
    pub x: i32,
    pub y: i32,
    pub t: u64,
}

impl StlTimedCell {
    pub fn new(x: i32, y: i32, t: u64) -> Self {
        Self { x, y, t }
    }
}

/// Agent start/goal query for STL-CBS.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StlCbsAgent {
    pub id: usize,
    pub start: (i32, i32),
    pub goal: (i32, i32),
}

impl StlCbsAgent {
    pub fn new(id: usize, start: (i32, i32), goal: (i32, i32)) -> Self {
        Self { id, start, goal }
    }
}

/// Configuration for the grid CBS planner.
#[derive(Debug, Clone, PartialEq)]
pub struct StlCbsConfig {
    pub width: i32,
    pub height: i32,
    pub obstacle_map: Vec<Vec<bool>>,
    pub max_time: u64,
    pub max_cbs_nodes: usize,
    pub allow_wait: bool,
}

impl StlCbsConfig {
    pub fn new(width: i32, height: i32) -> Self {
        Self {
            width,
            height,
            obstacle_map: vec![vec![false; height.max(0) as usize]; width.max(0) as usize],
            max_time: 64,
            max_cbs_nodes: 512,
            allow_wait: true,
        }
    }
}

/// One agent's planned timed path.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StlCbsPath {
    pub agent_id: usize,
    pub waypoints: Vec<StlTimedCell>,
}

impl StlCbsPath {
    pub fn arrival_time(&self) -> u64 {
        self.waypoints.last().map_or(0, |waypoint| waypoint.t)
    }

    pub fn position_at(&self, t: u64) -> StlTimedCell {
        self.waypoints
            .iter()
            .find(|waypoint| waypoint.t == t)
            .copied()
            .or_else(|| {
                self.waypoints
                    .iter()
                    .rev()
                    .find(|waypoint| waypoint.t < t)
                    .copied()
            })
            .unwrap_or_else(|| {
                *self
                    .waypoints
                    .first()
                    .expect("validated STL-CBS path is non-empty")
            })
    }
}

/// CBS result with robustness summary.
#[derive(Debug, Clone, PartialEq)]
pub struct StlCbsPlan {
    pub paths: Vec<StlCbsPath>,
    pub total_cost: u64,
    pub conflicts_resolved: usize,
    pub high_level_nodes_expanded: usize,
    pub min_pairwise_separation_robustness: f64,
}

/// Axis-aligned rectangle used by STL predicates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StlRectangle2D {
    pub min_x: f64,
    pub max_x: f64,
    pub min_y: f64,
    pub max_y: f64,
}

impl StlRectangle2D {
    pub fn new(min_x: f64, max_x: f64, min_y: f64, max_y: f64) -> RoboticsResult<Self> {
        if !min_x.is_finite()
            || !max_x.is_finite()
            || !min_y.is_finite()
            || !max_y.is_finite()
            || min_x > max_x
            || min_y > max_y
        {
            return Err(RoboticsError::InvalidParameter(
                "STL rectangle bounds must be finite and ordered".to_string(),
            ));
        }
        Ok(Self {
            min_x,
            max_x,
            min_y,
            max_y,
        })
    }

    pub fn inside_robustness(&self, x: f64, y: f64) -> f64 {
        (x - self.min_x)
            .min(self.max_x - x)
            .min(y - self.min_y)
            .min(self.max_y - y)
    }

    pub fn avoid_robustness(&self, x: f64, y: f64) -> f64 {
        -self.inside_robustness(x, y)
    }
}

/// Closed integer time interval for STL temporal operators.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StlTimeInterval {
    pub start: u64,
    pub end: u64,
}

impl StlTimeInterval {
    pub fn new(start: u64, end: u64) -> RoboticsResult<Self> {
        if start > end {
            return Err(RoboticsError::InvalidParameter(
                "STL time interval start must be <= end".to_string(),
            ));
        }
        Ok(Self { start, end })
    }
}

/// A high-level CBS conflict.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StlCbsConflict {
    pub agent_a: usize,
    pub agent_b: usize,
    pub kind: StlCbsConflictKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StlCbsConflictKind {
    Vertex {
        x: i32,
        y: i32,
        t: u64,
    },
    Edge {
        ax0: i32,
        ay0: i32,
        ax1: i32,
        ay1: i32,
        bx0: i32,
        by0: i32,
        bx1: i32,
        by1: i32,
        t: u64,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum StlCbsConstraintKind {
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
struct StlCbsConstraint {
    agent_id: usize,
    kind: StlCbsConstraintKind,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct CbsNode {
    constraints: Vec<StlCbsConstraint>,
    paths: Vec<StlCbsPath>,
    total_cost: u64,
    conflicts_resolved: usize,
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
    }
}

impl PartialOrd for SearchOpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Conflict-based multi-agent grid planner.
#[derive(Debug, Clone, PartialEq)]
pub struct StlCbsPlanner {
    config: StlCbsConfig,
    motions: Vec<(i32, i32)>,
}

impl StlCbsPlanner {
    pub fn new(config: StlCbsConfig) -> RoboticsResult<Self> {
        validate_config(&config)?;
        let mut motions = vec![(1, 0), (0, 1), (-1, 0), (0, -1)];
        if config.allow_wait {
            motions.push((0, 0));
        }
        Ok(Self { config, motions })
    }

    pub fn plan_independent(&self, agents: &[StlCbsAgent]) -> RoboticsResult<Vec<StlCbsPath>> {
        self.validate_agents(agents)?;
        agents
            .iter()
            .map(|agent| self.plan_agent(*agent, &[]))
            .collect()
    }

    pub fn plan(&self, agents: &[StlCbsAgent]) -> RoboticsResult<StlCbsPlan> {
        self.validate_agents(agents)?;
        let root_paths = self.plan_independent(agents)?;
        let root = CbsNode {
            total_cost: total_path_cost(&root_paths),
            constraints: Vec::new(),
            paths: root_paths,
            conflicts_resolved: 0,
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
                    "STL-CBS exceeded high-level node limit".to_string(),
                ));
            }
            let node = nodes[entry.node_index].clone();
            if let Some(conflict) = first_conflict(&node.paths, self.config.max_time) {
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
                return Ok(StlCbsPlan {
                    total_cost: node.total_cost,
                    min_pairwise_separation_robustness: stl_pairwise_separation_robustness(
                        &node.paths,
                        1.0,
                        StlTimeInterval {
                            start: 0,
                            end: self.config.max_time,
                        },
                    )?,
                    paths: node.paths,
                    conflicts_resolved: node.conflicts_resolved,
                    high_level_nodes_expanded: expanded,
                });
            }
        }

        Err(RoboticsError::PlanningError(
            "STL-CBS could not find a conflict-free plan".to_string(),
        ))
    }

    fn plan_agent(
        &self,
        agent: StlCbsAgent,
        constraints: &[StlCbsConstraintKind],
    ) -> RoboticsResult<StlCbsPath> {
        if self.violates_vertex_constraint(agent.start.0, agent.start.1, 0, constraints) {
            return Err(RoboticsError::PlanningError(
                "agent start violates a CBS vertex constraint".to_string(),
            ));
        }
        let start = SearchState {
            x: agent.start.0,
            y: agent.start.1,
            t: 0,
        };
        let mut open = BinaryHeap::new();
        let mut best_g = HashMap::new();
        let mut parent: HashMap<SearchState, Option<SearchState>> = HashMap::new();

        best_g.insert(start, 0);
        parent.insert(start, None);
        open.push(SearchOpenEntry {
            state: start,
            g: 0,
            f: manhattan(agent.start, agent.goal),
        });

        while let Some(entry) = open.pop() {
            if entry.state.t > self.config.max_time {
                continue;
            }
            if entry.g > *best_g.get(&entry.state).unwrap_or(&u64::MAX) {
                continue;
            }
            if (entry.state.x, entry.state.y) == agent.goal {
                return Ok(StlCbsPath {
                    agent_id: agent.id,
                    waypoints: reconstruct_timed_path(&parent, entry.state, agent.id),
                });
            }
            let next_t = entry.state.t + 1;
            if next_t > self.config.max_time {
                continue;
            }
            for &(dx, dy) in &self.motions {
                let nx = entry.state.x + dx;
                let ny = entry.state.y + dy;
                if !self.is_free(nx, ny) {
                    continue;
                }
                if self.violates_vertex_constraint(nx, ny, next_t, constraints)
                    || self.violates_edge_constraint(
                        entry.state.x,
                        entry.state.y,
                        nx,
                        ny,
                        next_t,
                        constraints,
                    )
                {
                    continue;
                }
                let next = SearchState {
                    x: nx,
                    y: ny,
                    t: next_t,
                };
                let next_g = entry.g + 1;
                if next_g < *best_g.get(&next).unwrap_or(&u64::MAX) {
                    best_g.insert(next, next_g);
                    parent.insert(next, Some(entry.state));
                    open.push(SearchOpenEntry {
                        state: next,
                        g: next_g,
                        f: next_g + manhattan((nx, ny), agent.goal),
                    });
                }
            }
        }

        Err(RoboticsError::PlanningError(format!(
            "no path found for agent {}",
            agent.id
        )))
    }

    fn validate_agents(&self, agents: &[StlCbsAgent]) -> RoboticsResult<()> {
        if agents.is_empty() {
            return Err(RoboticsError::InvalidParameter(
                "STL-CBS requires at least one agent".to_string(),
            ));
        }
        let mut ids = HashSet::new();
        for agent in agents {
            if !ids.insert(agent.id) {
                return Err(RoboticsError::InvalidParameter(
                    "STL-CBS agent ids must be unique".to_string(),
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

    fn violates_vertex_constraint(
        &self,
        x: i32,
        y: i32,
        t: u64,
        constraints: &[StlCbsConstraintKind],
    ) -> bool {
        constraints.iter().any(|constraint| {
            matches!(
                constraint,
                StlCbsConstraintKind::Vertex { x: cx, y: cy, t: ct }
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
        constraints: &[StlCbsConstraintKind],
    ) -> bool {
        constraints.iter().any(|constraint| {
            matches!(
                constraint,
                StlCbsConstraintKind::Edge {
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
}

/// Robustness of `F_[interval] inside(region)` for one path.
pub fn stl_eventually_reach_robustness(
    path: &StlCbsPath,
    region: StlRectangle2D,
    interval: StlTimeInterval,
) -> RoboticsResult<f64> {
    validate_path(path)?;
    let mut robustness = f64::NEG_INFINITY;
    for t in interval.start..=interval.end {
        let position = path.position_at(t);
        robustness = robustness.max(region.inside_robustness(position.x as f64, position.y as f64));
    }
    Ok(robustness)
}

/// Robustness of `G_[interval] outside(region)` for one path.
pub fn stl_always_avoid_robustness(
    path: &StlCbsPath,
    region: StlRectangle2D,
    interval: StlTimeInterval,
) -> RoboticsResult<f64> {
    validate_path(path)?;
    let mut robustness = f64::INFINITY;
    for t in interval.start..=interval.end {
        let position = path.position_at(t);
        robustness = robustness.min(region.avoid_robustness(position.x as f64, position.y as f64));
    }
    Ok(robustness)
}

/// Robustness of pairwise Euclidean separation over all paths.
pub fn stl_pairwise_separation_robustness(
    paths: &[StlCbsPath],
    min_distance: f64,
    interval: StlTimeInterval,
) -> RoboticsResult<f64> {
    if paths.len() < 2 {
        return Ok(f64::INFINITY);
    }
    if min_distance < 0.0 || !min_distance.is_finite() {
        return Err(RoboticsError::InvalidParameter(
            "STL min_distance must be finite and non-negative".to_string(),
        ));
    }
    for path in paths {
        validate_path(path)?;
    }
    let mut robustness = f64::INFINITY;
    for t in interval.start..=interval.end {
        for i in 0..paths.len() {
            for j in i + 1..paths.len() {
                let a = paths[i].position_at(t);
                let b = paths[j].position_at(t);
                robustness = robustness.min(euclidean((a.x, a.y), (b.x, b.y)) - min_distance);
            }
        }
    }
    Ok(robustness)
}

pub fn first_conflict(paths: &[StlCbsPath], max_time: u64) -> Option<StlCbsConflict> {
    for t in 0..=max_time {
        for i in 0..paths.len() {
            for j in i + 1..paths.len() {
                let a = paths[i].position_at(t);
                let b = paths[j].position_at(t);
                if a.x == b.x && a.y == b.y {
                    return Some(StlCbsConflict {
                        agent_a: paths[i].agent_id,
                        agent_b: paths[j].agent_id,
                        kind: StlCbsConflictKind::Vertex { x: a.x, y: a.y, t },
                    });
                }
                if t > 0 {
                    let a_prev = paths[i].position_at(t - 1);
                    let b_prev = paths[j].position_at(t - 1);
                    if a_prev.x == b.x && a_prev.y == b.y && b_prev.x == a.x && b_prev.y == a.y {
                        return Some(StlCbsConflict {
                            agent_a: paths[i].agent_id,
                            agent_b: paths[j].agent_id,
                            kind: StlCbsConflictKind::Edge {
                                ax0: a_prev.x,
                                ay0: a_prev.y,
                                ax1: a.x,
                                ay1: a.y,
                                bx0: b_prev.x,
                                by0: b_prev.y,
                                bx1: b.x,
                                by1: b.y,
                                t,
                            },
                        });
                    }
                }
            }
        }
    }
    None
}

fn constraint_from_conflict(
    conflict: StlCbsConflict,
    constrained_agent: usize,
) -> StlCbsConstraint {
    let kind = match conflict.kind {
        StlCbsConflictKind::Vertex { x, y, t } => StlCbsConstraintKind::Vertex { x, y, t },
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
                StlCbsConstraintKind::Edge {
                    from_x: ax0,
                    from_y: ay0,
                    to_x: ax1,
                    to_y: ay1,
                    t,
                }
            } else {
                StlCbsConstraintKind::Edge {
                    from_x: bx0,
                    from_y: by0,
                    to_x: bx1,
                    to_y: by1,
                    t,
                }
            }
        }
    };
    StlCbsConstraint {
        agent_id: constrained_agent,
        kind,
    }
}

fn constraints_for_agent(
    constraints: &[StlCbsConstraint],
    agent_id: usize,
) -> Vec<StlCbsConstraintKind> {
    constraints
        .iter()
        .filter(|constraint| constraint.agent_id == agent_id)
        .map(|constraint| constraint.kind)
        .collect()
}

fn total_path_cost(paths: &[StlCbsPath]) -> u64 {
    paths.iter().map(StlCbsPath::arrival_time).sum()
}

fn reconstruct_timed_path(
    parent: &HashMap<SearchState, Option<SearchState>>,
    mut current: SearchState,
    agent_id: usize,
) -> Vec<StlTimedCell> {
    let mut reversed = Vec::new();
    loop {
        reversed.push(StlTimedCell::new(current.x, current.y, current.t));
        match parent.get(&current).copied().flatten() {
            Some(previous) => current = previous,
            None => break,
        }
    }
    reversed.reverse();
    let _ = agent_id;
    reversed
}

fn manhattan(a: (i32, i32), b: (i32, i32)) -> u64 {
    (a.0 - b.0).unsigned_abs() as u64 + (a.1 - b.1).unsigned_abs() as u64
}

fn euclidean(a: (i32, i32), b: (i32, i32)) -> f64 {
    let dx = (a.0 - b.0) as f64;
    let dy = (a.1 - b.1) as f64;
    (dx * dx + dy * dy).sqrt()
}

fn validate_config(config: &StlCbsConfig) -> RoboticsResult<()> {
    if config.width <= 0 || config.height <= 0 {
        return Err(RoboticsError::InvalidParameter(
            "STL-CBS width and height must be positive".to_string(),
        ));
    }
    if config.max_time == 0 || config.max_cbs_nodes == 0 {
        return Err(RoboticsError::InvalidParameter(
            "STL-CBS max_time and max_cbs_nodes must be positive".to_string(),
        ));
    }
    if config.obstacle_map.len() != config.width as usize {
        return Err(RoboticsError::InvalidParameter(
            "STL-CBS obstacle_map x-dimension must match width".to_string(),
        ));
    }
    for column in &config.obstacle_map {
        if column.len() != config.height as usize {
            return Err(RoboticsError::InvalidParameter(
                "STL-CBS obstacle_map y-dimension must match height".to_string(),
            ));
        }
    }
    Ok(())
}

fn validate_path(path: &StlCbsPath) -> RoboticsResult<()> {
    if path.waypoints.is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "STL-CBS path must contain at least one waypoint".to_string(),
        ));
    }
    for window in path.waypoints.windows(2) {
        if window[1].t <= window[0].t {
            return Err(RoboticsError::InvalidParameter(
                "STL-CBS path times must be strictly increasing".to_string(),
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn corridor_planner() -> StlCbsPlanner {
        StlCbsPlanner::new(StlCbsConfig {
            max_time: 12,
            ..StlCbsConfig::new(5, 3)
        })
        .unwrap()
    }

    #[test]
    fn independent_paths_have_a_conflict() {
        let planner = corridor_planner();
        let agents = [
            StlCbsAgent::new(0, (0, 1), (4, 1)),
            StlCbsAgent::new(1, (4, 1), (0, 1)),
        ];
        let paths = planner.plan_independent(&agents).unwrap();

        assert!(matches!(
            first_conflict(&paths, 12).unwrap().kind,
            StlCbsConflictKind::Vertex { .. } | StlCbsConflictKind::Edge { .. }
        ));
    }

    #[test]
    fn cbs_resolves_two_agent_swap() {
        let planner = corridor_planner();
        let agents = [
            StlCbsAgent::new(0, (0, 1), (4, 1)),
            StlCbsAgent::new(1, (4, 1), (0, 1)),
        ];
        let plan = planner.plan(&agents).unwrap();

        assert!(plan.conflicts_resolved > 0);
        assert!(first_conflict(&plan.paths, 12).is_none());
        assert!(plan.min_pairwise_separation_robustness >= 0.0);
    }

    #[test]
    fn stl_goal_and_avoid_robustness_have_expected_signs() {
        let path = StlCbsPath {
            agent_id: 0,
            waypoints: vec![
                StlTimedCell::new(0, 0, 0),
                StlTimedCell::new(1, 0, 1),
                StlTimedCell::new(2, 0, 2),
            ],
        };
        let goal = StlRectangle2D::new(1.5, 2.5, -0.5, 0.5).unwrap();
        let hazard = StlRectangle2D::new(0.8, 1.2, -0.2, 0.2).unwrap();
        let interval = StlTimeInterval::new(0, 3).unwrap();

        assert!(stl_eventually_reach_robustness(&path, goal, interval).unwrap() > 0.0);
        assert!(stl_always_avoid_robustness(&path, hazard, interval).unwrap() < 0.0);
    }
}
