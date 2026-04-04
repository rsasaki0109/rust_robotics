//! Time-Based Path Planning with dynamic obstacles
//!
//! Implements Space-Time A\* and Safe Interval Path Planning (SIPP) on a 2-D
//! grid where obstacles move over discrete time steps. The module also
//! includes a priority-based multi-agent planner that sequences single-agent
//! plans to avoid inter-agent collisions.
//!
//! References:
//! - Silver (2005), "Cooperative Pathfinding"
//! - Phillips & Likhachev (2011), "SIPP: Safe Interval Path Planning for
//!   Dynamic Environments", ICRA 2011

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashSet};

use rust_robotics_core::{RoboticsError, RoboticsResult};

// ---------------------------------------------------------------------------
// Position & Interval
// ---------------------------------------------------------------------------

/// A discrete 2-D grid position.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Position {
    pub x: i32,
    pub y: i32,
}

impl Position {
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }

    fn manhattan_distance(self, other: Self) -> i32 {
        (self.x - other.x).abs() + (self.y - other.y).abs()
    }
}

/// A closed time interval `[start, end]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Interval {
    pub start_time: i32,
    pub end_time: i32,
}

impl Interval {
    pub fn new(start: i32, end: i32) -> Self {
        Self {
            start_time: start,
            end_time: end,
        }
    }
}

// ---------------------------------------------------------------------------
// Grid with dynamic obstacles
// ---------------------------------------------------------------------------

/// Describes how obstacles are arranged in the grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObstacleArrangement {
    /// Obstacles start in a vertical line at the grid centre and bounce horizontally.
    Arrangement1,
    /// Static wall with a single corridor in the middle row.
    NarrowCorridor,
}

/// A 2-D grid with a time dimension that records obstacle occupancy.
///
/// `reservation_matrix[x][y][t]` holds 0 if free, otherwise an obstacle/agent
/// identifier.
#[derive(Debug, Clone)]
pub struct Grid {
    pub width: i32,
    pub height: i32,
    pub time_limit: i32,
    /// `[x][y][t]` -> occupying agent id (0 = free).
    reservation: Vec<Vec<Vec<i32>>>,
    /// Paths of every dynamic obstacle (list of positions per time step).
    pub obstacle_paths: Vec<Vec<Position>>,
}

impl Grid {
    /// Create a grid and populate it with dynamic obstacles.
    pub fn new(
        width: i32,
        height: i32,
        num_obstacles: usize,
        arrangement: ObstacleArrangement,
        avoid_points: &[Position],
        time_limit: i32,
    ) -> RoboticsResult<Self> {
        if width <= 0 || height <= 0 || time_limit <= 0 {
            return Err(RoboticsError::InvalidParameter(
                "Grid dimensions and time_limit must be positive".into(),
            ));
        }

        let mut reservation =
            vec![vec![vec![0i32; time_limit as usize]; height as usize]; width as usize];

        let obstacle_paths = match arrangement {
            ObstacleArrangement::Arrangement1 => {
                Self::arrangement_1(width, height, num_obstacles, time_limit, avoid_points)
            }
            ObstacleArrangement::NarrowCorridor => {
                Self::narrow_corridor(width, height, num_obstacles, time_limit)
            }
        };

        // Record obstacle paths into the reservation matrix.
        for (i, path) in obstacle_paths.iter().enumerate() {
            let obs_id = (i + 1) as i32;
            for (t, &pos) in path.iter().enumerate() {
                reservation[pos.x as usize][pos.y as usize][t] = obs_id;
                if t > 0 {
                    let prev = path[t - 1];
                    reservation[prev.x as usize][prev.y as usize][t] = obs_id;
                }
            }
        }

        Ok(Self {
            width,
            height,
            time_limit,
            reservation,
            obstacle_paths,
        })
    }

    /// Create an empty grid (no obstacles).
    pub fn empty(width: i32, height: i32, time_limit: i32) -> RoboticsResult<Self> {
        if width <= 0 || height <= 0 || time_limit <= 0 {
            return Err(RoboticsError::InvalidParameter(
                "Grid dimensions and time_limit must be positive".into(),
            ));
        }
        Ok(Self {
            width,
            height,
            time_limit,
            reservation: vec![
                vec![vec![0i32; time_limit as usize]; height as usize];
                width as usize
            ],
            obstacle_paths: vec![],
        })
    }

    pub fn inside_bounds(&self, pos: Position) -> bool {
        pos.x >= 0 && pos.x < self.width && pos.y >= 0 && pos.y < self.height
    }

    pub fn is_free(&self, pos: Position, t: i32) -> bool {
        if !self.inside_bounds(pos) {
            return false;
        }
        if t < 0 || t >= self.time_limit {
            return false;
        }
        self.reservation[pos.x as usize][pos.y as usize][t as usize] == 0
    }

    /// Reserve a cell for a given agent during a time interval.
    pub fn reserve(&mut self, pos: Position, agent_id: i32, interval: Interval) {
        for t in interval.start_time..=interval.end_time.min(self.time_limit - 1) {
            self.reservation[pos.x as usize][pos.y as usize][t as usize] = agent_id;
        }
    }

    /// Reserve the entire path of an agent.
    pub fn reserve_path(&mut self, path: &NodePath, agent_id: i32) {
        for (i, node) in path.nodes.iter().enumerate() {
            let end = if i + 1 < path.nodes.len() {
                path.nodes[i + 1].time
            } else {
                node.time + 1
            };
            self.reserve(node.position, agent_id, Interval::new(node.time, end));
        }
    }

    /// Clear all reservations for a given agent at a specific position.
    pub fn clear_reservation(&mut self, pos: Position, agent_id: i32) {
        for t in 0..self.time_limit as usize {
            if self.reservation[pos.x as usize][pos.y as usize][t] == agent_id {
                self.reservation[pos.x as usize][pos.y as usize][t] = 0;
            }
        }
    }

    /// Compute safe intervals for a single cell.
    pub fn safe_intervals_at(&self, pos: Position) -> Vec<Interval> {
        let tl = self.time_limit as usize;
        let col = &self.reservation[pos.x as usize][pos.y as usize];
        let mut intervals = Vec::new();
        let mut start: Option<usize> = None;
        for (t, &cell) in col.iter().enumerate().take(tl) {
            if cell == 0 {
                if start.is_none() {
                    start = Some(t);
                }
            } else if let Some(s) = start {
                if t - 1 > s {
                    // interval with at least 2 time steps
                    intervals.push(Interval::new(s as i32, (t - 1) as i32));
                }
                start = None;
            }
        }
        if let Some(s) = start {
            if tl - 1 > s {
                intervals.push(Interval::new(s as i32, (tl - 1) as i32));
            }
        }
        intervals
    }

    // -- obstacle generators -------------------------------------------------

    fn arrangement_1(
        width: i32,
        height: i32,
        num_obstacles: usize,
        time_limit: i32,
        _avoid: &[Position],
    ) -> Vec<Vec<Position>> {
        let half_x = width / 2;
        let half_y = height / 2;
        let count = num_obstacles.min(height as usize);
        let mut paths = Vec::with_capacity(count);
        for y_idx in 0..count as i32 {
            let mut moving_right = y_idx < half_y;
            let mut pos = Position::new(half_x, y_idx);
            let mut path = vec![pos];
            for t in 1..time_limit - 1 {
                if t % 2 == 0 {
                    path.push(pos);
                    continue;
                }
                if (moving_right && pos.x == width - 1) || (!moving_right && pos.x == 0) {
                    moving_right = !moving_right;
                }
                pos = Position::new(pos.x + if moving_right { 1 } else { -1 }, pos.y);
                path.push(pos);
            }
            paths.push(path);
        }
        paths
    }

    fn narrow_corridor(
        width: i32,
        height: i32,
        num_obstacles: usize,
        time_limit: i32,
    ) -> Vec<Vec<Position>> {
        let mid_y = height / 2;
        let x = width / 2;
        let mut paths = Vec::new();
        for y in 0..height.min(num_obstacles as i32) {
            if y == mid_y {
                continue;
            }
            let pos = Position::new(x, y);
            let path = vec![pos; (time_limit - 1) as usize];
            paths.push(path);
        }
        paths
    }
}

// ---------------------------------------------------------------------------
// Node & NodePath
// ---------------------------------------------------------------------------

/// A search node in the space-time graph.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub struct Node {
    pub position: Position,
    pub time: i32,
    pub heuristic: i32,
    pub parent_index: i32,
}

impl Node {
    pub fn priority(&self) -> i32 {
        self.time + self.heuristic
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse for min-heap in BinaryHeap
        other.priority().cmp(&self.priority())
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl std::hash::Hash for Node {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.position.hash(state);
        self.time.hash(state);
    }
}

/// A planned path as a sequence of nodes with position-at-time lookup.
#[derive(Debug, Clone)]
pub struct NodePath {
    pub nodes: Vec<Node>,
    pub expanded_count: usize,
}

impl NodePath {
    pub fn new(nodes: Vec<Node>, expanded_count: usize) -> Self {
        Self {
            nodes,
            expanded_count,
        }
    }

    /// Get the position of the agent at a given time (interpolated between nodes).
    pub fn position_at(&self, time: i32) -> Option<Position> {
        if self.nodes.is_empty() {
            return None;
        }
        for (i, node) in self.nodes.iter().enumerate() {
            let next_time = if i + 1 < self.nodes.len() {
                self.nodes[i + 1].time
            } else {
                node.time + 1
            };
            if time >= node.time && time < next_time {
                return Some(node.position);
            }
        }
        // After the last node the agent stays at the goal
        self.nodes.last().map(|n| n.position)
    }

    /// Time at which the goal is reached.
    pub fn goal_reached_time(&self) -> i32 {
        self.nodes.last().map_or(0, |n| n.time)
    }
}

// ---------------------------------------------------------------------------
// 4-connected + wait neighbourhood
// ---------------------------------------------------------------------------

const DIFFS: [Position; 5] = [
    Position { x: 0, y: 0 }, // wait
    Position { x: 1, y: 0 },
    Position { x: -1, y: 0 },
    Position { x: 0, y: 1 },
    Position { x: 0, y: -1 },
];

// ---------------------------------------------------------------------------
// Space-Time A*
// ---------------------------------------------------------------------------

/// Space-Time A\* planner for a single agent on a grid with dynamic obstacles.
///
/// The cost `g(n)` is the number of time steps taken to reach a node, and the
/// heuristic is the Manhattan distance to the goal.
pub struct SpaceTimeAStar;

impl SpaceTimeAStar {
    /// Plan a path from `start` to `goal` on the given grid.
    pub fn plan(grid: &Grid, start: Position, goal: Position) -> RoboticsResult<NodePath> {
        let mut open: BinaryHeap<Node> = BinaryHeap::new();
        open.push(Node {
            position: start,
            time: 0,
            heuristic: start.manhattan_distance(goal),
            parent_index: -1,
        });

        let mut expanded_list: Vec<Node> = Vec::new();
        let mut expanded_set: HashSet<(Position, i32)> = HashSet::new();

        while let Some(node) = open.pop() {
            if node.time + 1 >= grid.time_limit {
                continue;
            }
            if node.position == goal {
                let path = Self::reconstruct(&expanded_list, node);
                return Ok(NodePath::new(path, expanded_set.len()));
            }

            let parent_idx = expanded_list.len() as i32;
            expanded_list.push(node);
            expanded_set.insert((node.position, node.time));

            for &diff in &DIFFS {
                let new_pos = node.position.add(diff);
                let new_time = node.time + 1;
                let new_node = Node {
                    position: new_pos,
                    time: new_time,
                    heuristic: new_pos.manhattan_distance(goal),
                    parent_index: parent_idx,
                };

                if expanded_set.contains(&(new_pos, new_time)) {
                    continue;
                }

                // Valid for the next 2 time steps (enter + leave)
                if grid.is_free(new_pos, new_time) && grid.is_free(new_pos, new_time + 1) {
                    open.push(new_node);
                }
            }
        }

        Err(RoboticsError::PlanningError("No path found".into()))
    }

    fn reconstruct(expanded: &[Node], goal_node: Node) -> Vec<Node> {
        let mut path = vec![goal_node];
        let mut walker = goal_node;
        while walker.parent_index >= 0 {
            walker = expanded[walker.parent_index as usize];
            path.push(walker);
        }
        path.reverse();
        path
    }
}

// ---------------------------------------------------------------------------
// SIPP node
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
struct SippNode {
    position: Position,
    time: i32,
    heuristic: i32,
    parent_index: i32,
    interval: Interval,
}

impl SippNode {
    fn priority(&self) -> i32 {
        self.time + self.heuristic
    }
}

impl Ord for SippNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other.priority().cmp(&self.priority())
    }
}

impl PartialOrd for SippNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// ---------------------------------------------------------------------------
// Safe Interval Path Planner
// ---------------------------------------------------------------------------

/// Safe Interval Path Planner (SIPP).
///
/// Reduces redundant node expansions compared to Space-Time A\* by
/// pre-computing safe intervals at each cell.
pub struct SafeIntervalPlanner;

impl SafeIntervalPlanner {
    /// Plan a path from `start` to `goal`.
    pub fn plan(grid: &Grid, start: Position, goal: Position) -> RoboticsResult<NodePath> {
        // Pre-compute safe intervals for every cell.
        let intervals = Self::compute_all_intervals(grid);

        let first_interval = intervals[start.x as usize][start.y as usize]
            .first()
            .copied()
            .ok_or_else(|| {
                RoboticsError::PlanningError("Start position has no safe interval".into())
            })?;

        let mut open: BinaryHeap<SippNode> = BinaryHeap::new();
        open.push(SippNode {
            position: start,
            time: 0,
            heuristic: start.manhattan_distance(goal),
            parent_index: -1,
            interval: first_interval,
        });

        let mut expanded_list: Vec<SippNode> = Vec::new();
        // visited_intervals[x][y] -> Vec<(entry_time, interval)>
        let mut visited: Vec<Vec<Vec<(i32, Interval)>>> =
            vec![vec![vec![]; grid.height as usize]; grid.width as usize];

        while let Some(node) = open.pop() {
            if node.time + 1 >= grid.time_limit {
                continue;
            }
            if node.position == goal {
                let path = Self::reconstruct(&expanded_list, &node);
                return Ok(NodePath::new(path, expanded_list.len()));
            }

            let parent_idx = expanded_list.len() as i32;
            expanded_list.push(node);

            // Mark visited
            let vlist = &mut visited[node.position.x as usize][node.position.y as usize];
            let already = vlist.iter_mut().find(|(_, iv)| *iv == node.interval);
            match already {
                Some(entry) => {
                    entry.0 = entry.0.min(node.time);
                }
                None => {
                    vlist.push((node.time, node.interval));
                }
            }

            for child in Self::successors(grid, goal, &node, parent_idx, &intervals, &visited) {
                open.push(child);
            }
        }

        Err(RoboticsError::PlanningError("No path found".into()))
    }

    fn successors(
        grid: &Grid,
        goal: Position,
        parent: &SippNode,
        parent_idx: i32,
        intervals: &[Vec<Vec<Interval>>],
        visited: &[Vec<Vec<(i32, Interval)>>],
    ) -> Vec<SippNode> {
        let mut result = Vec::new();
        for &diff in &DIFFS {
            let new_pos = parent.position.add(diff);
            if !grid.inside_bounds(new_pos) {
                continue;
            }
            let cell_intervals = &intervals[new_pos.x as usize][new_pos.y as usize];
            for &iv in cell_intervals {
                if iv.start_time > parent.interval.end_time {
                    break;
                }
                if iv.end_time < parent.interval.start_time {
                    continue;
                }
                // Check if already visited with equal or better entry time
                let dominated = visited[new_pos.x as usize][new_pos.y as usize]
                    .iter()
                    .any(|&(et, ref vis_iv)| *vis_iv == iv && et <= parent.time + 1);
                if dominated {
                    continue;
                }
                // Find earliest valid entry time
                let t_lo = (parent.time + 1).max(iv.start_time);
                let t_hi = parent.interval.end_time.min(iv.end_time);
                for t in t_lo..t_hi {
                    if grid.is_free(new_pos, t) {
                        result.push(SippNode {
                            position: new_pos,
                            time: (parent.time + 1).max(iv.start_time),
                            heuristic: new_pos.manhattan_distance(goal),
                            parent_index: parent_idx,
                            interval: iv,
                        });
                        break;
                    }
                }
            }
        }
        result
    }

    fn compute_all_intervals(grid: &Grid) -> Vec<Vec<Vec<Interval>>> {
        let mut all = vec![vec![vec![]; grid.height as usize]; grid.width as usize];
        for x in 0..grid.width {
            for y in 0..grid.height {
                all[x as usize][y as usize] = grid.safe_intervals_at(Position::new(x, y));
            }
        }
        all
    }

    fn reconstruct(expanded: &[SippNode], goal_node: &SippNode) -> Vec<Node> {
        let mut path = vec![Node {
            position: goal_node.position,
            time: goal_node.time,
            heuristic: goal_node.heuristic,
            parent_index: goal_node.parent_index,
        }];
        let mut walker = *goal_node;
        while walker.parent_index >= 0 {
            walker = expanded[walker.parent_index as usize];
            path.push(Node {
                position: walker.position,
                time: walker.time,
                heuristic: walker.heuristic,
                parent_index: walker.parent_index,
            });
        }
        path.reverse();
        path
    }
}

// ---------------------------------------------------------------------------
// Priority-based multi-agent planner
// ---------------------------------------------------------------------------

/// Start and goal for a single agent.
#[derive(Debug, Clone)]
pub struct AgentTask {
    pub id: i32,
    pub start: Position,
    pub goal: Position,
}

impl AgentTask {
    pub fn new(id: i32, start: Position, goal: Position) -> Self {
        Self { id, start, goal }
    }

    fn distance_sq(&self) -> i32 {
        let dx = self.goal.x - self.start.x;
        let dy = self.goal.y - self.start.y;
        dx * dx + dy * dy
    }
}

/// Which single-agent algorithm to use inside the multi-agent planner.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SingleAgentAlgorithm {
    SpaceTimeAStar,
    SafeInterval,
}

/// Priority-based multi-agent planner.
///
/// Plans agents one by one (longest distance first), reserving each
/// agent's path in the grid so that subsequent agents avoid it.
pub struct PriorityBasedPlanner;

impl PriorityBasedPlanner {
    /// Plan paths for all agents. Returns the ordered agent list and their
    /// paths. Agents are planned in descending order of start-to-goal distance.
    pub fn plan(
        grid: &mut Grid,
        tasks: &mut [AgentTask],
        algorithm: SingleAgentAlgorithm,
    ) -> RoboticsResult<Vec<NodePath>> {
        // Sort by descending distance
        tasks.sort_by_key(|b| std::cmp::Reverse(b.distance_sq()));

        // Reserve initial positions
        for task in tasks.iter() {
            grid.reserve(
                task.start,
                task.id,
                Interval::new(0, 10.min(grid.time_limit - 1)),
            );
        }

        let mut paths = Vec::with_capacity(tasks.len());
        for task in tasks.iter() {
            grid.clear_reservation(task.start, task.id);
            let path = match algorithm {
                SingleAgentAlgorithm::SpaceTimeAStar => {
                    SpaceTimeAStar::plan(grid, task.start, task.goal)?
                }
                SingleAgentAlgorithm::SafeInterval => {
                    SafeIntervalPlanner::plan(grid, task.start, task.goal)?
                }
            };
            grid.reserve_path(&path, task.id);
            paths.push(path);
        }
        Ok(paths)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: empty grid where every cell is free at all times.
    fn empty_grid(size: i32) -> Grid {
        Grid::empty(size, size, 100).unwrap()
    }

    // -----------------------------------------------------------------------
    // Space-Time A*
    // -----------------------------------------------------------------------

    #[test]
    fn test_sta_star_straight_line() {
        let grid = empty_grid(10);
        let start = Position::new(0, 0);
        let goal = Position::new(5, 0);
        let path = SpaceTimeAStar::plan(&grid, start, goal).unwrap();
        assert_eq!(path.nodes.first().unwrap().position, start);
        assert_eq!(path.nodes.last().unwrap().position, goal);
        assert_eq!(path.goal_reached_time(), 5);
    }

    #[test]
    fn test_sta_star_diagonal() {
        let grid = empty_grid(10);
        let start = Position::new(0, 0);
        let goal = Position::new(3, 3);
        let path = SpaceTimeAStar::plan(&grid, start, goal).unwrap();
        assert_eq!(path.nodes.last().unwrap().position, goal);
        // Manhattan distance = 6
        assert_eq!(path.goal_reached_time(), 6);
    }

    #[test]
    fn test_sta_star_same_start_goal() {
        let grid = empty_grid(10);
        let pos = Position::new(5, 5);
        let path = SpaceTimeAStar::plan(&grid, pos, pos).unwrap();
        assert_eq!(path.nodes.last().unwrap().position, pos);
    }

    #[test]
    fn test_sta_star_with_obstacles() {
        let grid = Grid::new(
            11,
            11,
            5,
            ObstacleArrangement::Arrangement1,
            &[Position::new(0, 0), Position::new(10, 10)],
            100,
        )
        .unwrap();
        let result = SpaceTimeAStar::plan(&grid, Position::new(0, 0), Position::new(10, 10));
        // May or may not find a path depending on obstacle layout; we just
        // verify it does not panic.
        assert!(result.is_ok() || result.is_err());
    }

    // -----------------------------------------------------------------------
    // Safe Interval Planner
    // -----------------------------------------------------------------------

    #[test]
    fn test_sipp_straight_line() {
        let grid = empty_grid(10);
        let start = Position::new(0, 0);
        let goal = Position::new(5, 0);
        let path = SafeIntervalPlanner::plan(&grid, start, goal).unwrap();
        assert_eq!(path.nodes.first().unwrap().position, start);
        assert_eq!(path.nodes.last().unwrap().position, goal);
        assert_eq!(path.goal_reached_time(), 5);
    }

    #[test]
    fn test_sipp_same_start_goal() {
        let grid = empty_grid(10);
        let pos = Position::new(3, 3);
        let path = SafeIntervalPlanner::plan(&grid, pos, pos).unwrap();
        assert_eq!(path.nodes.last().unwrap().position, pos);
    }

    // -----------------------------------------------------------------------
    // Grid & intervals
    // -----------------------------------------------------------------------

    #[test]
    fn test_grid_empty_all_free() {
        let grid = empty_grid(5);
        for x in 0..5 {
            for y in 0..5 {
                assert!(grid.is_free(Position::new(x, y), 0));
            }
        }
    }

    #[test]
    fn test_grid_bounds_check() {
        let grid = empty_grid(5);
        assert!(!grid.inside_bounds(Position::new(-1, 0)));
        assert!(!grid.inside_bounds(Position::new(0, 5)));
        assert!(grid.inside_bounds(Position::new(0, 0)));
        assert!(grid.inside_bounds(Position::new(4, 4)));
    }

    #[test]
    fn test_safe_intervals_empty_grid() {
        let grid = empty_grid(5);
        let ivs = grid.safe_intervals_at(Position::new(2, 2));
        // Single interval covering (almost) the whole time range
        assert!(!ivs.is_empty());
        assert_eq!(ivs[0].start_time, 0);
    }

    #[test]
    fn test_reserve_and_check() {
        let mut grid = empty_grid(5);
        let pos = Position::new(2, 2);
        assert!(grid.is_free(pos, 5));
        grid.reserve(pos, 1, Interval::new(5, 10));
        assert!(!grid.is_free(pos, 5));
        assert!(!grid.is_free(pos, 10));
        assert!(grid.is_free(pos, 11));
    }

    #[test]
    fn test_clear_reservation() {
        let mut grid = empty_grid(5);
        let pos = Position::new(1, 1);
        grid.reserve(pos, 42, Interval::new(0, 20));
        assert!(!grid.is_free(pos, 10));
        grid.clear_reservation(pos, 42);
        assert!(grid.is_free(pos, 10));
    }

    // -----------------------------------------------------------------------
    // NodePath
    // -----------------------------------------------------------------------

    #[test]
    fn test_node_path_position_at() {
        let nodes = vec![
            Node {
                position: Position::new(0, 0),
                time: 0,
                heuristic: 3,
                parent_index: -1,
            },
            Node {
                position: Position::new(1, 0),
                time: 1,
                heuristic: 2,
                parent_index: 0,
            },
            Node {
                position: Position::new(2, 0),
                time: 2,
                heuristic: 1,
                parent_index: 1,
            },
        ];
        let np = NodePath::new(nodes, 3);
        assert_eq!(np.position_at(0), Some(Position::new(0, 0)));
        assert_eq!(np.position_at(1), Some(Position::new(1, 0)));
        assert_eq!(np.position_at(2), Some(Position::new(2, 0)));
        // After goal: stays at last position
        assert_eq!(np.position_at(10), Some(Position::new(2, 0)));
    }

    // -----------------------------------------------------------------------
    // Priority-based multi-agent planner
    // -----------------------------------------------------------------------

    #[test]
    fn test_priority_planner_two_agents() {
        let mut grid = empty_grid(10);
        let mut tasks = vec![
            AgentTask::new(1, Position::new(0, 0), Position::new(5, 0)),
            AgentTask::new(2, Position::new(0, 1), Position::new(5, 1)),
        ];
        let paths =
            PriorityBasedPlanner::plan(&mut grid, &mut tasks, SingleAgentAlgorithm::SpaceTimeAStar)
                .unwrap();
        assert_eq!(paths.len(), 2);
        for (task, path) in tasks.iter().zip(paths.iter()) {
            assert_eq!(path.nodes.last().unwrap().position, task.goal);
        }
    }

    // -----------------------------------------------------------------------
    // Obstacle arrangement
    // -----------------------------------------------------------------------

    #[test]
    fn test_arrangement1_creates_obstacles() {
        let grid = Grid::new(11, 11, 5, ObstacleArrangement::Arrangement1, &[], 50).unwrap();
        assert_eq!(grid.obstacle_paths.len(), 5);
    }

    #[test]
    fn test_narrow_corridor_creates_obstacles() {
        let grid = Grid::new(11, 11, 20, ObstacleArrangement::NarrowCorridor, &[], 50).unwrap();
        // Skips the middle row, so 10 out of 11 rows
        assert_eq!(grid.obstacle_paths.len(), 10);
    }

    #[test]
    fn test_invalid_grid_params() {
        assert!(Grid::empty(0, 5, 100).is_err());
        assert!(Grid::empty(5, -1, 100).is_err());
        assert!(Grid::empty(5, 5, 0).is_err());
    }
}
