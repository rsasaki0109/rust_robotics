#![allow(clippy::collapsible_if)]

//! Jump Point Search (JPS) path planning algorithm
//!
//! JPS is an optimization of A* for uniform-cost grids that reduces the
//! number of nodes expanded by identifying "jump points" - nodes that
//! have forced neighbors or are the goal.
//!
//! Reference: Harabor, D., & Grastien, A. (2011). Online Graph Pruning for
//! Pathfinding on Grid Maps.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use crate::grid::{GridMap, Node};
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Configuration for JPS planner
#[derive(Debug, Clone)]
pub struct JPSConfig {
    /// Grid resolution in meters
    pub resolution: f64,
    /// Robot radius for obstacle inflation
    pub robot_radius: f64,
    /// Heuristic weight (1.0 = optimal, >1.0 = faster but suboptimal)
    pub heuristic_weight: f64,
}

impl Default for JPSConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        }
    }
}

impl JPSConfig {
    pub fn validate(&self) -> RoboticsResult<()> {
        if !self.resolution.is_finite() || self.resolution <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "resolution must be positive and finite, got {}",
                self.resolution
            )));
        }
        if !self.robot_radius.is_finite() || self.robot_radius < 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "robot_radius must be non-negative and finite, got {}",
                self.robot_radius
            )));
        }
        if !self.heuristic_weight.is_finite() || self.heuristic_weight <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "heuristic_weight must be positive and finite, got {}",
                self.heuristic_weight
            )));
        }

        Ok(())
    }
}

/// Why the JPS planner had to fall back to grid A*.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JPSFallbackReason {
    /// The reconstructed JPS path was invalid under the shared grid semantics.
    InvalidJumpPath,
    /// The reconstructed JPS path was valid but longer than the shared grid-optimal reference.
    SuboptimalJumpPath,
    /// The jump search failed to reach the goal, so the planner retried with grid A*.
    SearchExhausted,
}

impl JPSFallbackReason {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::InvalidJumpPath => "invalid_jump_path",
            Self::SuboptimalJumpPath => "suboptimal_jump_path",
            Self::SearchExhausted => "search_exhausted",
        }
    }
}

/// More specific classification for `InvalidJumpPath`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum JPSInvalidJumpPathDetail {
    /// The reconstructed jump path contains at least one invalid grid step.
    InvalidStepSequence,
    /// The reconstructed jump path is geometrically valid but its length no longer matches the
    /// search cost accumulated on jump points.
    CostMismatch,
    /// The reconstructed jump path has both invalid steps and a cost mismatch.
    InvalidStepSequenceAndCostMismatch,
}

impl JPSInvalidJumpPathDetail {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::InvalidStepSequence => "invalid_step_sequence",
            Self::CostMismatch => "cost_mismatch",
            Self::InvalidStepSequenceAndCostMismatch => "invalid_step_and_cost_mismatch",
        }
    }
}

/// Additional metrics attached to `cost_mismatch` fallback cases.
#[derive(Debug, Clone, PartialEq)]
pub struct JPSDetourSegment {
    pub from_x: i32,
    pub from_y: i32,
    pub to_x: i32,
    pub to_y: i32,
    pub expected_length: f64,
    pub reconstructed_length: f64,
}

/// Additional metrics attached to `cost_mismatch` fallback cases.
#[derive(Debug, Clone, PartialEq)]
pub struct JPSCostMismatchMetrics {
    pub search_cost: f64,
    pub reconstructed_path_length: f64,
    pub delta: f64,
    pub detour_segment_count: usize,
    pub detour_segments: Vec<JPSDetourSegment>,
}

/// Extra execution diagnostics for one JPS planning query.
#[derive(Debug, Clone, PartialEq)]
pub struct JPSPlanDiagnostics {
    pub used_fallback: bool,
    pub fallback_reason: Option<JPSFallbackReason>,
    pub invalid_jump_path_detail: Option<JPSInvalidJumpPathDetail>,
    pub cost_mismatch_metrics: Option<JPSCostMismatchMetrics>,
}

/// Planning result with path plus JPS-specific diagnostics.
#[derive(Debug, Clone)]
pub struct JPSPlanResult {
    pub path: Path2D,
    pub diagnostics: JPSPlanDiagnostics,
}

/// Direction for movement in JPS
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct Direction {
    dx: i32,
    dy: i32,
}

impl Direction {
    fn new(dx: i32, dy: i32) -> Self {
        Self { dx, dy }
    }

    fn is_diagonal(&self) -> bool {
        self.dx != 0 && self.dy != 0
    }

    fn is_cardinal(&self) -> bool {
        !self.is_diagonal()
    }
}

/// Node with priority for JPS open set (min-heap)
#[derive(Debug)]
struct PriorityNode {
    x: i32,
    y: i32,
    cost: f64,
    priority: f64,
    index: usize,
}

impl Eq for PriorityNode {}

impl PartialEq for PriorityNode {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority
    }
}

impl Ord for PriorityNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap behavior
        other
            .priority
            .partial_cmp(&self.priority)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for PriorityNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Jump Point Search path planner
pub struct JPSPlanner {
    grid_map: GridMap,
    config: JPSConfig,
}

impl JPSPlanner {
    fn state_key(&self, x: i32, y: i32, dir: Option<Direction>) -> (i32, Option<Direction>) {
        (self.grid_map.calc_index(x, y), dir)
    }

    /// Create a new JPS planner with obstacle positions
    pub fn new(ox: &[f64], oy: &[f64], config: JPSConfig) -> Self {
        Self::try_new(ox, oy, config).expect(
            "invalid JPS planner input: obstacle list must be non-empty and valid, and config values must be positive/finite",
        )
    }

    /// Create a validated JPS planner with obstacle positions
    pub fn try_new(ox: &[f64], oy: &[f64], config: JPSConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.resolution, config.robot_radius)?;
        Ok(JPSPlanner { grid_map, config })
    }

    /// Create from obstacle x/y vectors with default config
    pub fn from_obstacles(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let config = JPSConfig {
            resolution,
            robot_radius,
            ..Default::default()
        };
        Self::new(ox, oy, config)
    }

    /// Create a validated JPS planner from typed obstacle points
    pub fn from_obstacle_points(obstacles: &Obstacles, config: JPSConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::from_obstacles(obstacles, config.resolution, config.robot_radius)?;
        Ok(JPSPlanner { grid_map, config })
    }

    /// Plan a path returning (rx, ry) vectors (legacy interface)
    #[deprecated(note = "use plan() or plan_xy() instead")]
    pub fn planning(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> Option<(Vec<f64>, Vec<f64>)> {
        match self.plan_xy(sx, sy, gx, gy) {
            Ok(path) => Some((path.x_coords(), path.y_coords())),
            Err(_) => None,
        }
    }

    /// Plan a path without requiring the PathPlanner trait in scope
    pub fn plan(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        self.plan_with_diagnostics(start, goal)
            .map(|result| result.path)
    }

    /// Plan a path from raw coordinates without requiring the PathPlanner trait in scope
    pub fn plan_xy(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> RoboticsResult<Path2D> {
        self.plan_with_diagnostics(Point2D::new(sx, sy), Point2D::new(gx, gy))
            .map(|result| result.path)
    }

    /// Plan a path and report whether the query used the grid A* fallback.
    pub fn plan_with_diagnostics(
        &self,
        start: Point2D,
        goal: Point2D,
    ) -> RoboticsResult<JPSPlanResult> {
        self.plan_impl(start, goal)
    }

    /// Get reference to the grid map
    pub fn grid_map(&self) -> &GridMap {
        &self.grid_map
    }

    fn calc_heuristic(&self, n1_x: i32, n1_y: i32, n2_x: i32, n2_y: i32) -> f64 {
        // Octile distance heuristic for 8-connected grid
        let dx = (n1_x - n2_x).abs() as f64;
        let dy = (n1_y - n2_y).abs() as f64;
        let d_min = dx.min(dy);
        let d_max = dx.max(dy);
        self.config.heuristic_weight * (d_min * std::f64::consts::SQRT_2 + (d_max - d_min))
    }

    /// Check if a position has forced neighbors in the given direction
    fn has_forced_neighbor(&self, x: i32, y: i32, dir: Direction) -> bool {
        if dir.is_cardinal() {
            // For cardinal directions, check perpendicular neighbors
            if dir.dx != 0 {
                // Moving horizontally
                let blocked_up = !self.grid_map.is_valid(x - dir.dx, y + 1);
                let open_up = self.grid_map.is_valid(x, y + 1);
                let blocked_down = !self.grid_map.is_valid(x - dir.dx, y - 1);
                let open_down = self.grid_map.is_valid(x, y - 1);
                (blocked_up && open_up) || (blocked_down && open_down)
            } else {
                // Moving vertically
                let blocked_right = !self.grid_map.is_valid(x + 1, y - dir.dy);
                let open_right = self.grid_map.is_valid(x + 1, y);
                let blocked_left = !self.grid_map.is_valid(x - 1, y - dir.dy);
                let open_left = self.grid_map.is_valid(x - 1, y);
                (blocked_right && open_right) || (blocked_left && open_left)
            }
        } else {
            false
        }
    }

    /// Jump in a given direction from (x, y) until we find a jump point or hit an obstacle
    fn jump(&self, x: i32, y: i32, dir: Direction, goal_x: i32, goal_y: i32) -> Option<(i32, i32)> {
        let nx = x + dir.dx;
        let ny = y + dir.dy;

        if !self.grid_map.is_valid_step(x, y, nx, ny) {
            return None;
        }

        if nx == goal_x && ny == goal_y {
            return Some((nx, ny));
        }

        if self.has_forced_neighbor(nx, ny, dir) {
            return Some((nx, ny));
        }

        if dir.is_diagonal() {
            if self
                .jump(nx, ny, Direction::new(dir.dx, 0), goal_x, goal_y)
                .is_some()
            {
                return Some((nx, ny));
            }
            if self
                .jump(nx, ny, Direction::new(0, dir.dy), goal_x, goal_y)
                .is_some()
            {
                return Some((nx, ny));
            }
        }

        self.jump(nx, ny, dir, goal_x, goal_y)
    }

    fn get_successors(
        &self,
        x: i32,
        y: i32,
        parent_dir: Option<Direction>,
        goal_x: i32,
        goal_y: i32,
    ) -> Vec<(i32, i32, Direction)> {
        let mut successors = Vec::new();
        let directions = self.get_neighbors(x, y, parent_dir);

        for dir in directions {
            if let Some((jx, jy)) = self.jump(x, y, dir, goal_x, goal_y) {
                successors.push((jx, jy, dir));
            }
        }

        successors
    }

    fn get_neighbors(&self, x: i32, y: i32, parent_dir: Option<Direction>) -> Vec<Direction> {
        match parent_dir {
            None => {
                let mut dirs = Vec::new();
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        let dir = Direction::new(dx, dy);
                        if self.grid_map.is_valid_step(x, y, x + dx, y + dy) {
                            dirs.push(dir);
                        }
                    }
                }
                dirs
            }
            Some(dir) => {
                let mut dirs = Vec::new();

                if dir.is_diagonal() {
                    if self.grid_map.is_valid_step(x, y, x + dir.dx, y + dir.dy) {
                        dirs.push(dir);
                    }
                    if self.grid_map.is_valid_step(x, y, x + dir.dx, y) {
                        dirs.push(Direction::new(dir.dx, 0));
                    }
                    if self.grid_map.is_valid_step(x, y, x, y + dir.dy) {
                        dirs.push(Direction::new(0, dir.dy));
                    }

                    if !self.grid_map.is_valid(x - dir.dx, y) {
                        if self.grid_map.is_valid_step(x, y, x - dir.dx, y + dir.dy) {
                            dirs.push(Direction::new(-dir.dx, dir.dy));
                        }
                    }
                    if !self.grid_map.is_valid(x, y - dir.dy) {
                        if self.grid_map.is_valid_step(x, y, x + dir.dx, y - dir.dy) {
                            dirs.push(Direction::new(dir.dx, -dir.dy));
                        }
                    }
                } else if dir.dx != 0 {
                    if self.grid_map.is_valid_step(x, y, x + dir.dx, y) {
                        dirs.push(dir);
                    }
                    let blocked_lower = !self.grid_map.is_valid(x - dir.dx, y + 1);
                    if blocked_lower {
                        if self.grid_map.is_valid_step(x, y, x + dir.dx, y + 1) {
                            dirs.push(Direction::new(dir.dx, 1));
                        }
                        if self.grid_map.is_valid_step(x, y, x, y + 1) {
                            dirs.push(Direction::new(0, 1));
                        }
                    }
                    let blocked_upper = !self.grid_map.is_valid(x - dir.dx, y - 1);
                    if blocked_upper {
                        if self.grid_map.is_valid_step(x, y, x + dir.dx, y - 1) {
                            dirs.push(Direction::new(dir.dx, -1));
                        }
                        if self.grid_map.is_valid_step(x, y, x, y - 1) {
                            dirs.push(Direction::new(0, -1));
                        }
                    }
                } else {
                    if self.grid_map.is_valid_step(x, y, x, y + dir.dy) {
                        dirs.push(dir);
                    }
                    let blocked_right = !self.grid_map.is_valid(x + 1, y - dir.dy);
                    if blocked_right {
                        if self.grid_map.is_valid_step(x, y, x + 1, y + dir.dy) {
                            dirs.push(Direction::new(1, dir.dy));
                        }
                        if self.grid_map.is_valid_step(x, y, x + 1, y) {
                            dirs.push(Direction::new(1, 0));
                        }
                    }
                    let blocked_left = !self.grid_map.is_valid(x - 1, y - dir.dy);
                    if blocked_left {
                        if self.grid_map.is_valid_step(x, y, x - 1, y + dir.dy) {
                            dirs.push(Direction::new(-1, dir.dy));
                        }
                        if self.grid_map.is_valid_step(x, y, x - 1, y) {
                            dirs.push(Direction::new(-1, 0));
                        }
                    }
                }

                dirs
            }
        }
    }

    fn calc_distance(&self, x1: i32, y1: i32, x2: i32, y2: i32) -> f64 {
        let dx = (x2 - x1).abs() as f64;
        let dy = (y2 - y1).abs() as f64;
        let d_min = dx.min(dy);
        let d_max = dx.max(dy);
        d_min * std::f64::consts::SQRT_2 + (d_max - d_min)
    }

    fn grid_point(&self, x: i32, y: i32) -> Point2D {
        Point2D::new(
            self.grid_map.calc_x_position(x),
            self.grid_map.calc_y_position(y),
        )
    }

    fn collect_jump_points(&self, goal_index: usize, node_storage: &[Node]) -> Vec<(i32, i32)> {
        let mut jump_points = Vec::new();
        let mut current_index = Some(goal_index);

        while let Some(index) = current_index {
            let node = &node_storage[index];
            jump_points.push((node.x, node.y));
            current_index = node.parent_index;
        }

        jump_points.reverse();
        jump_points
    }

    fn append_reconstructed_segment(
        &self,
        points: &mut Vec<Point2D>,
        from: (i32, i32),
        to: (i32, i32),
    ) -> f64 {
        let (px, py) = from;
        let (x, y) = to;
        let dx = (x - px).signum();
        let dy = (y - py).signum();
        let mut cx = px;
        let mut cy = py;
        let mut reconstructed_length = 0.0;

        while cx != x || cy != y {
            let previous_x = cx;
            let previous_y = cy;

            if cx != x && cy != y {
                if self.grid_map.is_valid_step(cx, cy, cx + dx, cy + dy) {
                    cx += dx;
                    cy += dy;
                } else if self.grid_map.is_valid_step(cx, cy, cx + dx, cy) {
                    cx += dx;
                } else if self.grid_map.is_valid_step(cx, cy, cx, cy + dy) {
                    cy += dy;
                } else {
                    cx += dx;
                    cy += dy;
                }
            } else {
                if cx != x {
                    cx += dx;
                } else {
                    cy += dy;
                }
            }

            reconstructed_length += self.calc_distance(previous_x, previous_y, cx, cy);
            points.push(self.grid_point(cx, cy));
        }

        reconstructed_length
    }

    fn build_path_with_metrics(
        &self,
        goal_index: usize,
        node_storage: &[Node],
    ) -> (Path2D, Vec<JPSDetourSegment>) {
        let jump_points = self.collect_jump_points(goal_index, node_storage);
        let mut points = Vec::new();
        let mut detour_segments = Vec::new();

        if let Some(&(start_x, start_y)) = jump_points.first() {
            points.push(self.grid_point(start_x, start_y));
        }

        for segment in jump_points.windows(2) {
            let from = segment[0];
            let to = segment[1];
            let reconstructed_length = self.append_reconstructed_segment(&mut points, from, to);
            let expected_length = self.calc_distance(from.0, from.1, to.0, to.1);
            if (reconstructed_length - expected_length).abs() > 1e-6 {
                detour_segments.push(JPSDetourSegment {
                    from_x: from.0,
                    from_y: from.1,
                    to_x: to.0,
                    to_y: to.1,
                    expected_length,
                    reconstructed_length,
                });
            }
        }

        (Path2D::from_points(points), detour_segments)
    }

    fn build_search_path(&self, goal_index: usize, node_storage: &[Node]) -> Path2D {
        let points: Vec<Point2D> = self
            .collect_jump_points(goal_index, node_storage)
            .into_iter()
            .map(|(x, y)| self.grid_point(x, y))
            .collect();
        Path2D::from_points(points)
    }

    fn cost_mismatch_metrics(
        &self,
        detail: Option<JPSInvalidJumpPathDetail>,
        path: &Path2D,
        search_cost: f64,
        detour_segments: Vec<JPSDetourSegment>,
    ) -> Option<JPSCostMismatchMetrics> {
        match detail {
            Some(JPSInvalidJumpPathDetail::CostMismatch)
            | Some(JPSInvalidJumpPathDetail::InvalidStepSequenceAndCostMismatch) => {
                let reconstructed_path_length = path.total_length();
                Some(JPSCostMismatchMetrics {
                    search_cost,
                    reconstructed_path_length,
                    delta: reconstructed_path_length - search_cost,
                    detour_segment_count: detour_segments.len(),
                    detour_segments,
                })
            }
            _ => None,
        }
    }

    fn path_has_only_valid_steps(&self, path: &Path2D) -> bool {
        path.points.windows(2).all(|segment| {
            let from_x = self.grid_map.calc_x_index(segment[0].x);
            let from_y = self.grid_map.calc_y_index(segment[0].y);
            let to_x = self.grid_map.calc_x_index(segment[1].x);
            let to_y = self.grid_map.calc_y_index(segment[1].y);
            self.grid_map.is_valid_step(from_x, from_y, to_x, to_y)
        })
    }

    fn classify_invalid_jump_path(
        &self,
        path: &Path2D,
        expected_cost: f64,
    ) -> Option<JPSInvalidJumpPathDetail> {
        let has_only_valid_steps = self.path_has_only_valid_steps(path);
        let has_cost_mismatch = (path.total_length() - expected_cost).abs() > 1e-6;

        match (has_only_valid_steps, has_cost_mismatch) {
            (true, false) => None,
            (false, false) => Some(JPSInvalidJumpPathDetail::InvalidStepSequence),
            (true, true) => Some(JPSInvalidJumpPathDetail::CostMismatch),
            (false, true) => Some(JPSInvalidJumpPathDetail::InvalidStepSequenceAndCostMismatch),
        }
    }

    fn plan_with_grid_a_star(
        &self,
        start_x: i32,
        start_y: i32,
        goal_x: i32,
        goal_y: i32,
    ) -> RoboticsResult<Path2D> {
        let mut open_set = BinaryHeap::new();
        let mut closed_set: HashMap<i32, usize> = HashMap::new();
        let mut node_storage: Vec<Node> = Vec::new();

        node_storage.push(Node::new(start_x, start_y, 0.0, None));
        open_set.push(PriorityNode {
            x: start_x,
            y: start_y,
            cost: 0.0,
            priority: self.calc_heuristic(start_x, start_y, goal_x, goal_y),
            index: 0,
        });

        while let Some(current) = open_set.pop() {
            let current_grid_index = self.grid_map.calc_index(current.x, current.y);

            if current.x == goal_x && current.y == goal_y {
                return Ok(self.build_search_path(current.index, &node_storage));
            }

            if let Some(&existing_index) = closed_set.get(&current_grid_index) {
                if node_storage[existing_index].cost <= current.cost {
                    continue;
                }
            }

            closed_set.insert(current_grid_index, current.index);

            for dx in -1..=1 {
                for dy in -1..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }

                    if !self.grid_map.is_valid_offset(current.x, current.y, dx, dy) {
                        continue;
                    }

                    let new_x = current.x + dx;
                    let new_y = current.y + dy;
                    let move_cost = if dx != 0 && dy != 0 {
                        std::f64::consts::SQRT_2
                    } else {
                        1.0
                    };
                    let new_cost = current.cost + move_cost;
                    let new_grid_index = self.grid_map.calc_index(new_x, new_y);

                    if let Some(&existing_index) = closed_set.get(&new_grid_index) {
                        if node_storage[existing_index].cost <= new_cost {
                            continue;
                        }
                    }

                    node_storage.push(Node::new(new_x, new_y, new_cost, Some(current.index)));
                    let new_index = node_storage.len() - 1;
                    open_set.push(PriorityNode {
                        x: new_x,
                        y: new_y,
                        cost: new_cost,
                        priority: new_cost + self.calc_heuristic(new_x, new_y, goal_x, goal_y),
                        index: new_index,
                    });
                }
            }
        }

        Err(RoboticsError::PlanningError("No path found".to_string()))
    }

    fn direct_result(&self, path: Path2D) -> JPSPlanResult {
        JPSPlanResult {
            path,
            diagnostics: JPSPlanDiagnostics {
                used_fallback: false,
                fallback_reason: None,
                invalid_jump_path_detail: None,
                cost_mismatch_metrics: None,
            },
        }
    }

    fn fallback_result_with_path(
        &self,
        path: Path2D,
        fallback_reason: JPSFallbackReason,
        invalid_jump_path_detail: Option<JPSInvalidJumpPathDetail>,
        cost_mismatch_metrics: Option<JPSCostMismatchMetrics>,
    ) -> JPSPlanResult {
        JPSPlanResult {
            path,
            diagnostics: JPSPlanDiagnostics {
                used_fallback: true,
                fallback_reason: Some(fallback_reason),
                invalid_jump_path_detail,
                cost_mismatch_metrics,
            },
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn fallback_result(
        &self,
        start_x: i32,
        start_y: i32,
        goal_x: i32,
        goal_y: i32,
        fallback_reason: JPSFallbackReason,
        invalid_jump_path_detail: Option<JPSInvalidJumpPathDetail>,
        cost_mismatch_metrics: Option<JPSCostMismatchMetrics>,
    ) -> RoboticsResult<JPSPlanResult> {
        self.plan_with_grid_a_star(start_x, start_y, goal_x, goal_y)
            .map(|path| {
                self.fallback_result_with_path(
                    path,
                    fallback_reason,
                    invalid_jump_path_detail,
                    cost_mismatch_metrics,
                )
            })
    }

    fn ensure_query_is_valid(&self, x: i32, y: i32, label: &str) -> RoboticsResult<()> {
        if self.grid_map.is_valid(x, y) {
            return Ok(());
        }

        Err(RoboticsError::PlanningError(format!(
            "{} position is invalid",
            label
        )))
    }

    fn plan_impl(&self, start: Point2D, goal: Point2D) -> RoboticsResult<JPSPlanResult> {
        let start_x = self.grid_map.calc_x_index(start.x);
        let start_y = self.grid_map.calc_y_index(start.y);
        let goal_x = self.grid_map.calc_x_index(goal.x);
        let goal_y = self.grid_map.calc_y_index(goal.y);

        self.ensure_query_is_valid(start_x, start_y, "Start")?;
        self.ensure_query_is_valid(goal_x, goal_y, "Goal")?;

        let mut open_set = BinaryHeap::new();
        let mut closed_set: HashMap<(i32, Option<Direction>), usize> = HashMap::new();
        let mut node_storage: Vec<Node> = Vec::new();
        let mut direction_map: HashMap<usize, Option<Direction>> = HashMap::new();

        node_storage.push(Node::new(start_x, start_y, 0.0, None));
        let start_index = 0;
        direction_map.insert(start_index, None);

        open_set.push(PriorityNode {
            x: start_x,
            y: start_y,
            cost: 0.0,
            priority: self.calc_heuristic(start_x, start_y, goal_x, goal_y),
            index: start_index,
        });

        while let Some(current) = open_set.pop() {
            let current_dir = direction_map.get(&current.index).copied().flatten();
            let current_state_key = self.state_key(current.x, current.y, current_dir);

            if current.x == goal_x && current.y == goal_y {
                let (path, detour_segments) =
                    self.build_path_with_metrics(current.index, &node_storage);
                let invalid_jump_path_detail = self.classify_invalid_jump_path(&path, current.cost);
                if invalid_jump_path_detail.is_none() {
                    let reference_path =
                        self.plan_with_grid_a_star(start_x, start_y, goal_x, goal_y)?;
                    if reference_path.total_length() + 1e-6 < path.total_length() {
                        return Ok(self.fallback_result_with_path(
                            reference_path,
                            JPSFallbackReason::SuboptimalJumpPath,
                            None,
                            None,
                        ));
                    }
                    return Ok(self.direct_result(path));
                }
                let cost_mismatch_metrics = self.cost_mismatch_metrics(
                    invalid_jump_path_detail,
                    &path,
                    current.cost,
                    detour_segments,
                );

                return self.fallback_result(
                    start_x,
                    start_y,
                    goal_x,
                    goal_y,
                    JPSFallbackReason::InvalidJumpPath,
                    invalid_jump_path_detail,
                    cost_mismatch_metrics,
                );
            }

            if let Some(&existing_index) = closed_set.get(&current_state_key) {
                if node_storage[existing_index].cost <= current.cost {
                    continue;
                }
            }

            closed_set.insert(current_state_key, current.index);

            let successors = self.get_successors(current.x, current.y, current_dir, goal_x, goal_y);

            for (jx, jy, dir) in successors {
                let new_cost = current.cost + self.calc_distance(current.x, current.y, jx, jy);
                let new_state_key = self.state_key(jx, jy, Some(dir));

                if let Some(&existing_index) = closed_set.get(&new_state_key) {
                    if node_storage[existing_index].cost <= new_cost {
                        continue;
                    }
                }

                node_storage.push(Node::new(jx, jy, new_cost, Some(current.index)));
                let new_index = node_storage.len() - 1;
                direction_map.insert(new_index, Some(dir));

                let priority = new_cost + self.calc_heuristic(jx, jy, goal_x, goal_y);
                open_set.push(PriorityNode {
                    x: jx,
                    y: jy,
                    cost: new_cost,
                    priority,
                    index: new_index,
                });
            }
        }

        self.fallback_result(
            start_x,
            start_y,
            goal_x,
            goal_y,
            JPSFallbackReason::SearchExhausted,
            None,
            None,
        )
    }
}

impl PathPlanner for JPSPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        self.plan_with_diagnostics(start, goal)
            .map(|result| result.path)
    }
}

#[cfg(test)]
mod tests {
    #![allow(dead_code)]

    use super::*;

    use crate::a_star::{AStarConfig, AStarPlanner};
    use crate::moving_ai::{MovingAiMap, MovingAiScenario};
    use rust_robotics_core::Obstacles;
    use std::collections::{BinaryHeap, HashMap, HashSet};
    use std::hint::black_box;
    use std::time::Instant;

    #[derive(Debug)]
    struct RawTraceSuccessor {
        x: i32,
        y: i32,
        dir: Direction,
    }

    #[derive(Debug)]
    struct RawTraceExpansion {
        node_index: usize,
        parent_index: Option<usize>,
        x: i32,
        y: i32,
        cost: f64,
        parent_dir: Option<Direction>,
        successors: Vec<RawTraceSuccessor>,
    }

    #[derive(Debug)]
    enum RawTraceOutcome {
        GoalReached {
            jump_points: Vec<(i32, i32)>,
            path_length: f64,
            search_cost: f64,
            invalid_jump_path_detail: Option<JPSInvalidJumpPathDetail>,
        },
        SearchExhausted,
    }

    #[derive(Debug)]
    struct RawTrace {
        expansions: Vec<RawTraceExpansion>,
        outcome: RawTraceOutcome,
    }

    #[derive(Debug)]
    struct RawJumpChainStats {
        segment_count: usize,
        short_segment_count: usize,
        direction_change_count: usize,
    }

    #[derive(Debug)]
    struct NodeProbeCandidate {
        dir: Direction,
        jump_target: Option<(i32, i32)>,
    }

    fn trace_raw_jps(
        planner: &JPSPlanner,
        start: Point2D,
        goal: Point2D,
        max_expansions: usize,
    ) -> RawTrace {
        let start_x = planner.grid_map.calc_x_index(start.x);
        let start_y = planner.grid_map.calc_y_index(start.y);
        let goal_x = planner.grid_map.calc_x_index(goal.x);
        let goal_y = planner.grid_map.calc_y_index(goal.y);

        planner
            .ensure_query_is_valid(start_x, start_y, "Start")
            .expect("trace start should be valid");
        planner
            .ensure_query_is_valid(goal_x, goal_y, "Goal")
            .expect("trace goal should be valid");

        let mut open_set = BinaryHeap::new();
        let mut closed_set: HashMap<(i32, Option<Direction>), usize> = HashMap::new();
        let mut node_storage: Vec<Node> = Vec::new();
        let mut direction_map: HashMap<usize, Option<Direction>> = HashMap::new();
        let mut expansions = Vec::new();

        node_storage.push(Node::new(start_x, start_y, 0.0, None));
        direction_map.insert(0, None);
        open_set.push(PriorityNode {
            x: start_x,
            y: start_y,
            cost: 0.0,
            priority: planner.calc_heuristic(start_x, start_y, goal_x, goal_y),
            index: 0,
        });

        while let Some(current) = open_set.pop() {
            let current_dir = direction_map.get(&current.index).copied().flatten();
            let current_state_key = planner.state_key(current.x, current.y, current_dir);

            if current.x == goal_x && current.y == goal_y {
                let (path, _) = planner.build_path_with_metrics(current.index, &node_storage);
                let invalid_jump_path_detail =
                    planner.classify_invalid_jump_path(&path, current.cost);
                return RawTrace {
                    expansions,
                    outcome: RawTraceOutcome::GoalReached {
                        jump_points: planner.collect_jump_points(current.index, &node_storage),
                        path_length: path.total_length(),
                        search_cost: current.cost,
                        invalid_jump_path_detail,
                    },
                };
            }

            if let Some(&existing_index) = closed_set.get(&current_state_key) {
                if node_storage[existing_index].cost <= current.cost {
                    continue;
                }
            }

            closed_set.insert(current_state_key, current.index);

            let successors =
                planner.get_successors(current.x, current.y, current_dir, goal_x, goal_y);
            let trace_successors = successors
                .iter()
                .map(|(x, y, dir)| RawTraceSuccessor {
                    x: *x,
                    y: *y,
                    dir: *dir,
                })
                .collect();
            expansions.push(RawTraceExpansion {
                node_index: current.index,
                parent_index: node_storage[current.index].parent_index,
                x: current.x,
                y: current.y,
                cost: current.cost,
                parent_dir: current_dir,
                successors: trace_successors,
            });
            if expansions.len() >= max_expansions {
                return RawTrace {
                    expansions,
                    outcome: RawTraceOutcome::SearchExhausted,
                };
            }

            for (jx, jy, dir) in successors {
                let new_cost = current.cost + planner.calc_distance(current.x, current.y, jx, jy);
                let new_state_key = planner.state_key(jx, jy, Some(dir));

                if let Some(&existing_index) = closed_set.get(&new_state_key) {
                    if node_storage[existing_index].cost <= new_cost {
                        continue;
                    }
                }

                node_storage.push(Node::new(jx, jy, new_cost, Some(current.index)));
                let new_index = node_storage.len() - 1;
                direction_map.insert(new_index, Some(dir));
                open_set.push(PriorityNode {
                    x: jx,
                    y: jy,
                    cost: new_cost,
                    priority: new_cost + planner.calc_heuristic(jx, jy, goal_x, goal_y),
                    index: new_index,
                });
            }
        }

        RawTrace {
            expansions,
            outcome: RawTraceOutcome::SearchExhausted,
        }
    }

    fn trace_chain_to_root(
        expansions: &[RawTraceExpansion],
        terminal_node_index: usize,
    ) -> Vec<&RawTraceExpansion> {
        let expansion_index_by_node: HashMap<usize, usize> = expansions
            .iter()
            .enumerate()
            .map(|(ix, expansion)| (expansion.node_index, ix))
            .collect();

        let mut chain = Vec::new();
        let mut current = Some(terminal_node_index);
        while let Some(node_index) = current {
            let Some(&expansion_ix) = expansion_index_by_node.get(&node_index) else {
                break;
            };
            let expansion = &expansions[expansion_ix];
            chain.push(expansion);
            current = expansion.parent_index;
        }
        chain.reverse();
        chain
    }

    fn summarize_jump_chain(jump_points: &[(i32, i32)]) -> RawJumpChainStats {
        if jump_points.len() < 2 {
            return RawJumpChainStats {
                segment_count: 0,
                short_segment_count: 0,
                direction_change_count: 0,
            };
        }

        let mut short_segment_count = 0;
        let mut directions = Vec::new();

        for segment in jump_points.windows(2) {
            let dx = segment[1].0 - segment[0].0;
            let dy = segment[1].1 - segment[0].1;
            let length = ((dx * dx + dy * dy) as f64).sqrt();
            if length <= std::f64::consts::SQRT_2 + 1e-6 {
                short_segment_count += 1;
            }
            directions.push((dx.signum(), dy.signum()));
        }

        let direction_change_count = directions
            .windows(2)
            .filter(|pair| pair[0] != pair[1])
            .count();

        RawJumpChainStats {
            segment_count: jump_points.len() - 1,
            short_segment_count,
            direction_change_count,
        }
    }

    fn probe_node(
        planner: &JPSPlanner,
        x: i32,
        y: i32,
        parent_dir: Option<Direction>,
        goal_x: i32,
        goal_y: i32,
    ) -> Vec<NodeProbeCandidate> {
        planner
            .get_neighbors(x, y, parent_dir)
            .into_iter()
            .map(|dir| NodeProbeCandidate {
                jump_target: planner.jump(x, y, dir, goal_x, goal_y),
                dir,
            })
            .collect()
    }

    fn densest_direction_change_window(
        jump_points: &[(i32, i32)],
        window_segments: usize,
    ) -> Option<(usize, usize, usize)> {
        if jump_points.len() < window_segments + 1 || window_segments < 2 {
            return None;
        }

        let directions: Vec<(i32, i32)> = jump_points
            .windows(2)
            .map(|segment| {
                (
                    (segment[1].0 - segment[0].0).signum(),
                    (segment[1].1 - segment[0].1).signum(),
                )
            })
            .collect();

        let mut best: Option<(usize, usize, usize)> = None;
        for start in 0..=directions.len() - window_segments {
            let end = start + window_segments;
            let change_count = directions[start..end]
                .windows(2)
                .filter(|pair| pair[0] != pair[1])
                .count();
            if best
                .map(|(_, _, best_count)| change_count > best_count)
                .unwrap_or(true)
            {
                best = Some((start, end, change_count));
            }
        }
        best
    }

    fn first_short_segment_run(
        jump_points: &[(i32, i32)],
        min_run_len: usize,
    ) -> Option<(usize, usize)> {
        if jump_points.len() < 2 {
            return None;
        }

        let mut run_start = None;
        let mut run_len = 0usize;
        for (ix, segment) in jump_points.windows(2).enumerate() {
            let dx = segment[1].0 - segment[0].0;
            let dy = segment[1].1 - segment[0].1;
            let length = ((dx * dx + dy * dy) as f64).sqrt();
            if length <= std::f64::consts::SQRT_2 + 1e-6 {
                if run_start.is_none() {
                    run_start = Some(ix);
                }
                run_len += 1;
                if run_len >= min_run_len {
                    return run_start.map(|start| (start, run_len));
                }
            } else {
                run_start = None;
                run_len = 0;
            }
        }
        None
    }

    fn grid_points_from_path(planner: &JPSPlanner, path: &Path2D) -> Vec<(i32, i32)> {
        path.points
            .iter()
            .map(|point| {
                (
                    planner.grid_map.calc_x_index(point.x),
                    planner.grid_map.calc_y_index(point.y),
                )
            })
            .collect()
    }

    fn turn_points_from_grid_path(points: &[(i32, i32)]) -> Vec<(i32, i32)> {
        if points.len() < 2 {
            return points.to_vec();
        }

        let mut turn_points = vec![points[0]];
        let mut last_dir = (
            (points[1].0 - points[0].0).signum(),
            (points[1].1 - points[0].1).signum(),
        );

        for window in points.windows(3) {
            let current_dir = (
                (window[2].0 - window[1].0).signum(),
                (window[2].1 - window[1].1).signum(),
            );
            if current_dir != last_dir {
                turn_points.push(window[1]);
                last_dir = current_dir;
            }
        }

        turn_points.push(*points.last().unwrap());
        turn_points
    }

    fn first_missing_reference_turn_point(
        reference_turn_points: &[(i32, i32)],
        raw_trace: &RawTrace,
    ) -> Option<(usize, (i32, i32), bool, bool)> {
        let expanded_positions: HashSet<(i32, i32)> = raw_trace
            .expansions
            .iter()
            .map(|expansion| (expansion.x, expansion.y))
            .collect();
        let generated_positions: HashSet<(i32, i32)> = raw_trace
            .expansions
            .iter()
            .flat_map(|expansion| expansion.successors.iter().map(|succ| (succ.x, succ.y)))
            .collect();

        reference_turn_points
            .iter()
            .enumerate()
            .skip(1)
            .find_map(|(ix, &point)| {
                let expanded = expanded_positions.contains(&point);
                let generated = generated_positions.contains(&point);
                if expanded || generated {
                    None
                } else {
                    Some((ix, point, expanded, generated))
                }
            })
    }

    #[allow(clippy::type_complexity)]
    fn first_divergent_turn_point(
        reference_turn_points: &[(i32, i32)],
        raw_jump_points: &[(i32, i32)],
    ) -> Option<(usize, Option<(i32, i32)>, Option<(i32, i32)>)> {
        let max_len = reference_turn_points.len().max(raw_jump_points.len());
        for ix in 0..max_len {
            let reference_point = reference_turn_points.get(ix).copied();
            let raw_point = raw_jump_points.get(ix).copied();
            if reference_point != raw_point {
                return Some((ix, reference_point, raw_point));
            }
        }
        None
    }

    fn create_simple_obstacles() -> (Vec<f64>, Vec<f64>) {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        for i in 0..61 {
            ox.push(i as f64);
            oy.push(0.0);
            ox.push(i as f64);
            oy.push(60.0);
            ox.push(0.0);
            oy.push(i as f64);
            ox.push(60.0);
            oy.push(i as f64);
        }

        for i in 20..40 {
            ox.push(30.0);
            oy.push(i as f64);
        }

        (ox, oy)
    }

    const MOVING_AI_CASES: [(&str, &str, &str); 3] = [
        (
            "arena2",
            include_str!("../benchdata/moving_ai/dao/arena2.map"),
            include_str!("../benchdata/moving_ai/dao/arena2.map.scen"),
        ),
        (
            "8room_000",
            include_str!("../benchdata/moving_ai/room/8room_000.map"),
            include_str!("../benchdata/moving_ai/room/8room_000.map.scen"),
        ),
        (
            "random512-10-0",
            include_str!("../benchdata/moving_ai/random/random512-10-0.map"),
            include_str!("../benchdata/moving_ai/random/random512-10-0.map.scen"),
        ),
    ];

    fn assert_pure_jps_matches_moving_ai_bucket(
        name: &str,
        map_str: &str,
        scen_str: &str,
        bucket: u32,
    ) {
        let map = MovingAiMap::parse_str(map_str)
            .unwrap_or_else(|_| panic!("{name} MovingAI map should parse"));
        let scenario = MovingAiScenario::parse_str(scen_str)
            .unwrap_or_else(|_| panic!("{name} MovingAI scenarios should parse"))
            .into_iter()
            .find(|row| row.bucket == bucket)
            .unwrap_or_else(|| panic!("{name} MovingAI bucket {bucket} scenario should exist"));

        let planner = JPSPlanner::from_obstacle_points(
            &map.to_obstacles(),
            JPSConfig {
                resolution: 1.0,
                robot_radius: 0.5,
                heuristic_weight: 1.0,
            },
        )
        .unwrap_or_else(|_| panic!("JPS planner should build from {name} obstacles"));

        let start = map
            .planning_point(scenario.start_x, scenario.start_y)
            .unwrap_or_else(|_| panic!("{name} start should be valid"));
        let goal = map
            .planning_point(scenario.goal_x, scenario.goal_y)
            .unwrap_or_else(|_| panic!("{name} goal should be valid"));

        let result = planner
            .plan_with_diagnostics(start, goal)
            .unwrap_or_else(|_| panic!("JPS should solve the {name} bucket {bucket} scenario"));

        assert!(
            !result.path.is_empty(),
            "{name} bucket {bucket} path should not be empty"
        );
        println!(
            "{name}/bucket{bucket}: used_fallback={}, reason={:?}, detail={:?}, path_length={:.6}",
            result.diagnostics.used_fallback,
            result.diagnostics.fallback_reason,
            result.diagnostics.invalid_jump_path_detail,
            result.path.total_length()
        );
        assert!(
            !result.diagnostics.used_fallback,
            "{name} bucket {bucket} should solve as pure JPS, got reason={:?}, detail={:?}, path_length={:.6}",
            result.diagnostics.fallback_reason,
            result.diagnostics.invalid_jump_path_detail,
            result.path.total_length()
        );
        assert_eq!(
            result.diagnostics.fallback_reason, None,
            "{name} bucket {bucket} pure-JPS regression should not report a fallback reason"
        );
        assert_eq!(
            result.diagnostics.invalid_jump_path_detail,
            None,
            "{name} bucket {bucket} pure-JPS regression should not report an invalid_jump_path detail"
        );
        assert!(
            (result.path.total_length() - scenario.optimal_length).abs() <= 1e-6,
            "{name} bucket {bucket} pure JPS path length {} should match MovingAI optimal {}",
            result.path.total_length(),
            scenario.optimal_length
        );

        let grid = planner.grid_map();
        for segment in result.path.points.windows(2) {
            let from_x = grid.calc_x_index(segment[0].x);
            let from_y = grid.calc_y_index(segment[0].y);
            let to_x = grid.calc_x_index(segment[1].x);
            let to_y = grid.calc_y_index(segment[1].y);
            assert!(
                grid.is_valid_step(from_x, from_y, to_x, to_y),
                "{name} bucket {bucket} path contains an invalid step from ({from_x}, {from_y}) to ({to_x}, {to_y})"
            );
        }
    }

    fn build_moving_ai_bucket_planners(
        name: &str,
        map_str: &str,
        scen_str: &str,
        bucket: u32,
    ) -> (AStarPlanner, JPSPlanner, Point2D, Point2D, f64) {
        let map = MovingAiMap::parse_str(map_str)
            .unwrap_or_else(|_| panic!("{name} MovingAI map should parse"));
        let scenario = MovingAiScenario::parse_str(scen_str)
            .unwrap_or_else(|_| panic!("{name} MovingAI scenarios should parse"))
            .into_iter()
            .find(|row| row.bucket == bucket)
            .unwrap_or_else(|| panic!("{name} MovingAI bucket {bucket} scenario should exist"));
        let obstacles = map.to_obstacles();
        let a_star = AStarPlanner::from_obstacle_points(
            &obstacles,
            AStarConfig {
                resolution: 1.0,
                robot_radius: 0.5,
                heuristic_weight: 1.0,
            },
        )
        .unwrap_or_else(|_| panic!("A* planner should build from {name} obstacles"));
        let jps = JPSPlanner::from_obstacle_points(
            &obstacles,
            JPSConfig {
                resolution: 1.0,
                robot_radius: 0.5,
                heuristic_weight: 1.0,
            },
        )
        .unwrap_or_else(|_| panic!("JPS planner should build from {name} obstacles"));
        let start = map
            .planning_point(scenario.start_x, scenario.start_y)
            .unwrap_or_else(|_| panic!("{name} start should be valid"));
        let goal = map
            .planning_point(scenario.goal_x, scenario.goal_y)
            .unwrap_or_else(|_| panic!("{name} goal should be valid"));

        (a_star, jps, start, goal, scenario.optimal_length)
    }

    fn build_moving_ai_family_planners(
        name: &str,
        map_str: &str,
        scen_str: &str,
    ) -> (MovingAiMap, Vec<MovingAiScenario>, AStarPlanner, JPSPlanner) {
        let map = MovingAiMap::parse_str(map_str)
            .unwrap_or_else(|_| panic!("{name} MovingAI map should parse"));
        let scenarios = MovingAiScenario::parse_str(scen_str)
            .unwrap_or_else(|_| panic!("{name} MovingAI scenarios should parse"));
        let obstacles = map.to_obstacles();
        let a_star = AStarPlanner::from_obstacle_points(
            &obstacles,
            AStarConfig {
                resolution: 1.0,
                robot_radius: 0.5,
                heuristic_weight: 1.0,
            },
        )
        .unwrap_or_else(|_| panic!("A* planner should build from {name} obstacles"));
        let jps = JPSPlanner::from_obstacle_points(
            &obstacles,
            JPSConfig {
                resolution: 1.0,
                robot_radius: 0.5,
                heuristic_weight: 1.0,
            },
        )
        .unwrap_or_else(|_| panic!("JPS planner should build from {name} obstacles"));

        (map, scenarios, a_star, jps)
    }

    fn build_sampled_moving_ai_bucket_cases(
        name: &str,
        map: &MovingAiMap,
        scenarios: &[MovingAiScenario],
        bucket: u32,
        sample_slots: &[usize],
    ) -> Vec<(usize, Point2D, Point2D, f64)> {
        let bucket_scenarios: Vec<&MovingAiScenario> = scenarios
            .iter()
            .filter(|row| row.bucket == bucket)
            .collect();
        assert!(
            !bucket_scenarios.is_empty(),
            "{name} MovingAI bucket {bucket} scenarios should exist"
        );
        let max_slot = sample_slots.iter().copied().max().unwrap_or(0);
        assert!(
            bucket_scenarios.len() > max_slot,
            "{name} bucket {bucket} should have at least {} scenarios, found {}",
            max_slot + 1,
            bucket_scenarios.len()
        );

        sample_slots
            .iter()
            .map(|&slot| {
                let scenario = bucket_scenarios[slot];
                let start = map
                    .planning_point(scenario.start_x, scenario.start_y)
                    .unwrap_or_else(|_| panic!("{name} start should be valid"));
                let goal = map
                    .planning_point(scenario.goal_x, scenario.goal_y)
                    .unwrap_or_else(|_| panic!("{name} goal should be valid"));
                (slot, start, goal, scenario.optimal_length)
            })
            .collect()
    }

    fn measure_planner_median_us<F>(iterations: usize, mut plan: F) -> f64
    where
        F: FnMut() -> Path2D,
    {
        let mut samples = Vec::with_capacity(iterations);
        for _ in 0..iterations {
            let started = Instant::now();
            let path = plan();
            black_box(path.points.len());
            black_box(path.total_length());
            samples.push(started.elapsed().as_secs_f64() * 1_000_000.0);
        }
        samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
        samples[iterations / 2]
    }

    fn median_value(mut samples: Vec<f64>) -> f64 {
        assert!(
            !samples.is_empty(),
            "median_value requires at least one sample"
        );
        samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
        samples[samples.len() / 2]
    }

    #[derive(Debug, Clone, Copy)]
    struct BucketRuntimeAggregate {
        a_star_median_us: f64,
        jps_median_us: f64,
        a_star_min_us: f64,
        a_star_max_us: f64,
        jps_min_us: f64,
        jps_max_us: f64,
        jps_wins: usize,
        sample_count: usize,
    }

    #[allow(clippy::too_many_arguments)]
    fn measure_moving_ai_bucket_runtime_aggregate(
        name: &str,
        map: &MovingAiMap,
        scenarios: &[MovingAiScenario],
        a_star: &AStarPlanner,
        jps: &JPSPlanner,
        bucket: u32,
        sample_slots: &[usize],
        iterations: usize,
    ) -> BucketRuntimeAggregate {
        let sampled_cases =
            build_sampled_moving_ai_bucket_cases(name, map, scenarios, bucket, sample_slots);
        let mut a_star_samples = Vec::with_capacity(sampled_cases.len());
        let mut jps_samples = Vec::with_capacity(sampled_cases.len());
        let mut jps_wins = 0usize;

        for (slot, start, goal, optimal_length) in sampled_cases {
            let warmup_a = a_star
                .plan(start, goal)
                .unwrap_or_else(|_| panic!("A* should solve {name} bucket {bucket} slot {slot}"));
            assert!(
                (warmup_a.total_length() - optimal_length).abs() <= 1e-6,
                "A* warmup path for {name} bucket {bucket} slot {slot} should match MovingAI optimal {}",
                optimal_length
            );
            let warmup_j = jps
                .plan(start, goal)
                .unwrap_or_else(|_| panic!("JPS should solve {name} bucket {bucket} slot {slot}"));
            assert!(
                (warmup_j.total_length() - optimal_length).abs() <= 1e-6,
                "JPS warmup path for {name} bucket {bucket} slot {slot} should match MovingAI optimal {}",
                optimal_length
            );

            let a_star_median_us = measure_planner_median_us(iterations, || {
                let path = a_star.plan(start, goal).unwrap_or_else(|_| {
                    panic!("A* should solve {name} bucket {bucket} slot {slot}")
                });
                assert!(
                    (path.total_length() - optimal_length).abs() <= 1e-6,
                    "A* path length {} should match MovingAI optimal {} on {name} bucket {bucket} slot {slot}",
                    path.total_length(),
                    optimal_length
                );
                path
            });
            let jps_median_us = measure_planner_median_us(iterations, || {
                let path = jps.plan(start, goal).unwrap_or_else(|_| {
                    panic!("JPS should solve {name} bucket {bucket} slot {slot}")
                });
                assert!(
                    (path.total_length() - optimal_length).abs() <= 1e-6,
                    "JPS path length {} should match MovingAI optimal {} on {name} bucket {bucket} slot {slot}",
                    path.total_length(),
                    optimal_length
                );
                path
            });
            if jps_median_us < a_star_median_us {
                jps_wins += 1;
            }
            a_star_samples.push(a_star_median_us);
            jps_samples.push(jps_median_us);
        }

        let a_star_median_us = median_value(a_star_samples.clone());
        let jps_median_us = median_value(jps_samples.clone());
        let a_star_min_us = a_star_samples.iter().copied().fold(f64::INFINITY, f64::min);
        let a_star_max_us = a_star_samples
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        let jps_min_us = jps_samples.iter().copied().fold(f64::INFINITY, f64::min);
        let jps_max_us = jps_samples
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);

        BucketRuntimeAggregate {
            a_star_median_us,
            jps_median_us,
            a_star_min_us,
            a_star_max_us,
            jps_min_us,
            jps_max_us,
            jps_wins,
            sample_count: sample_slots.len(),
        }
    }

    fn create_small_obstacles() -> (Vec<f64>, Vec<f64>) {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        for i in 0..11 {
            ox.push(i as f64);
            oy.push(0.0);
            ox.push(i as f64);
            oy.push(10.0);
            ox.push(0.0);
            oy.push(i as f64);
            ox.push(10.0);
            oy.push(i as f64);
        }

        for i in 4..7 {
            ox.push(5.0);
            oy.push(i as f64);
        }

        (ox, oy)
    }

    fn draw_horizontal_line(
        start_x: i32,
        start_y: i32,
        length: i32,
        ox: &mut Vec<f64>,
        oy: &mut Vec<f64>,
    ) {
        for i in start_x..start_x + length {
            for j in start_y..start_y + 2 {
                ox.push(i as f64);
                oy.push(j as f64);
            }
        }
    }

    fn draw_vertical_line(
        start_x: i32,
        start_y: i32,
        length: i32,
        ox: &mut Vec<f64>,
        oy: &mut Vec<f64>,
    ) {
        for i in start_x..start_x + 2 {
            for j in start_y..start_y + length {
                ox.push(i as f64);
                oy.push(j as f64);
            }
        }
    }

    fn create_upstream_jump_point_maze() -> (Vec<f64>, Vec<f64>) {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        draw_vertical_line(0, 0, 50, &mut ox, &mut oy);
        draw_vertical_line(48, 0, 50, &mut ox, &mut oy);
        draw_horizontal_line(0, 0, 50, &mut ox, &mut oy);
        draw_horizontal_line(0, 48, 50, &mut ox, &mut oy);

        let vertical_x = [10, 10, 10, 15, 20, 20, 30, 30, 35, 30, 40, 45];
        let vertical_y = [10, 30, 45, 20, 5, 40, 10, 40, 5, 40, 10, 25];
        let vertical_len = [10, 10, 5, 10, 10, 5, 20, 10, 25, 10, 35, 15];
        for ((x, y), len) in vertical_x
            .iter()
            .zip(vertical_y.iter())
            .zip(vertical_len.iter())
        {
            draw_vertical_line(*x, *y, *len, &mut ox, &mut oy);
        }

        let horizontal_x = [35, 40, 15, 10, 45, 20, 10, 15, 25, 45, 10, 30, 10, 40];
        let horizontal_y = [5, 10, 15, 20, 20, 25, 30, 35, 35, 35, 40, 40, 45, 45];
        let horizontal_len = [10, 5, 10, 10, 5, 5, 10, 5, 10, 5, 10, 5, 5, 5];
        for ((x, y), len) in horizontal_x
            .iter()
            .zip(horizontal_y.iter())
            .zip(horizontal_len.iter())
        {
            draw_horizontal_line(*x, *y, *len, &mut ox, &mut oy);
        }

        (ox, oy)
    }

    fn upstream_jump_point_reference_polyline() -> Vec<Point2D> {
        vec![
            Point2D::new(5.0, 5.0),
            Point2D::new(7.0, 6.0),
            Point2D::new(9.0, 10.0),
            Point2D::new(9.0, 15.0),
            Point2D::new(9.0, 20.0),
            Point2D::new(10.0, 22.0),
            Point2D::new(11.0, 26.0),
            Point2D::new(10.0, 29.0),
            Point2D::new(9.0, 33.0),
            Point2D::new(9.0, 38.0),
            Point2D::new(10.0, 42.0),
            Point2D::new(14.0, 44.0),
            Point2D::new(19.0, 44.0),
            Point2D::new(22.0, 47.0),
            Point2D::new(25.0, 45.0),
            Point2D::new(29.0, 43.0),
            Point2D::new(29.0, 40.0),
            Point2D::new(32.0, 39.0),
            Point2D::new(35.0, 40.0),
            Point2D::new(36.0, 44.0),
            Point2D::new(35.0, 45.0),
        ]
    }

    fn polyline_length(points: &[Point2D]) -> f64 {
        points
            .windows(2)
            .map(|window| window[0].distance(&window[1]))
            .sum()
    }

    #[test]
    fn test_jps_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = JPSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(10.0, 10.0);
        let goal = Point2D::new(50.0, 50.0);

        let result = planner.plan(start, goal);
        assert!(result.is_ok());

        let path = result.unwrap();
        assert!(!path.is_empty());

        let first = &path.points[0];
        let last = &path.points[path.len() - 1];
        assert!((first.x - 10.0).abs() < 2.0);
        assert!((first.y - 10.0).abs() < 2.0);
        assert!((last.x - 50.0).abs() < 2.0);
        assert!((last.y - 50.0).abs() < 2.0);
    }

    #[test]
    fn test_jps_small_map() {
        let (ox, oy) = create_small_obstacles();
        let planner = JPSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(8.0, 8.0);

        let result = planner.plan(start, goal);
        assert!(result.is_ok());

        let path = result.unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_jps_diagnostics_are_consistent_on_small_map() {
        let (ox, oy) = create_small_obstacles();
        let planner = JPSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let result = planner
            .plan_with_diagnostics(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0))
            .expect("small-map JPS query should succeed");

        assert!(!result.path.is_empty());
        assert_eq!(
            result.diagnostics.used_fallback,
            result.diagnostics.fallback_reason.is_some()
        );
        if result.diagnostics.fallback_reason != Some(JPSFallbackReason::InvalidJumpPath) {
            assert_eq!(result.diagnostics.invalid_jump_path_detail, None);
            assert_eq!(result.diagnostics.cost_mismatch_metrics, None);
        }
        if !matches!(
            result.diagnostics.invalid_jump_path_detail,
            Some(JPSInvalidJumpPathDetail::CostMismatch)
                | Some(JPSInvalidJumpPathDetail::InvalidStepSequenceAndCostMismatch)
        ) {
            assert_eq!(result.diagnostics.cost_mismatch_metrics, None);
        }
    }

    #[test]
    #[allow(deprecated)]
    fn test_jps_legacy_interface() {
        let (ox, oy) = create_small_obstacles();
        let planner = JPSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let result = planner.planning(2.0, 2.0, 8.0, 8.0);
        assert!(result.is_some());

        let (rx, ry) = result.unwrap();
        assert!(!rx.is_empty());
        assert_eq!(rx.len(), ry.len());
    }

    #[test]
    fn test_jps_no_path() {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        for i in 0..10 {
            ox.push(i as f64);
            oy.push(0.0);
            ox.push(i as f64);
            oy.push(9.0);
            ox.push(0.0);
            oy.push(i as f64);
            ox.push(9.0);
            oy.push(i as f64);
        }

        for i in 1..9 {
            ox.push(5.0);
            oy.push(i as f64);
        }

        let planner = JPSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(2.0, 5.0);
        let goal = Point2D::new(7.0, 5.0);

        let result = planner.plan(start, goal);
        assert!(result.is_err());
    }

    #[test]
    fn test_jps_diagonal_path() {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        for i in 0..21 {
            ox.push(i as f64);
            oy.push(0.0);
            ox.push(i as f64);
            oy.push(20.0);
            ox.push(0.0);
            oy.push(i as f64);
            ox.push(20.0);
            oy.push(i as f64);
        }

        let planner = JPSPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(18.0, 18.0);

        let result = planner.plan(start, goal);
        assert!(result.is_ok());

        let path = result.unwrap();
        assert!(path.len() >= 2, "Path should have at least start and goal");
        let total_len = path.total_length();
        assert!(
            total_len < 30.0,
            "Path should be efficient, got length {}",
            total_len
        );
    }

    #[test]
    fn test_jps_from_obstacle_points() {
        let (ox, oy) = create_small_obstacles();
        let obstacles = Obstacles::try_from_xy(&ox, &oy).unwrap();
        let planner = JPSPlanner::from_obstacle_points(&obstacles, JPSConfig::default()).unwrap();

        let path = planner.plan_xy(2.0, 2.0, 8.0, 8.0).unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_jps_try_new_rejects_invalid_config() {
        let (ox, oy) = create_small_obstacles();
        let config = JPSConfig {
            heuristic_weight: 0.0,
            ..Default::default()
        };

        let err = match JPSPlanner::try_new(&ox, &oy, config) {
            Ok(_) => panic!("expected invalid config to be rejected"),
            Err(err) => err,
        };
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_jps_preserves_asymmetric_world_coordinates() {
        let mut obstacles = Obstacles::new();

        for x in 10..=20 {
            obstacles.push(Point2D::new(x as f64, -4.0));
            obstacles.push(Point2D::new(x as f64, 6.0));
        }
        for y in -4..=6 {
            obstacles.push(Point2D::new(10.0, y as f64));
            obstacles.push(Point2D::new(20.0, y as f64));
        }

        let planner = JPSPlanner::from_obstacle_points(&obstacles, JPSConfig::default()).unwrap();
        let path = planner.plan_xy(12.0, -2.0, 18.0, 4.0).unwrap();

        let first = path.points.first().unwrap();
        let last = path.points.last().unwrap();
        assert!((first.x - 12.0).abs() < 1e-6);
        assert!((first.y + 2.0).abs() < 1e-6);
        assert!((last.x - 18.0).abs() < 1e-6);
        assert!((last.y - 4.0).abs() < 1e-6);
    }

    #[test]
    fn test_jps_solves_upstream_jump_point_maze() {
        let (ox, oy) = create_upstream_jump_point_maze();
        let planner = JPSPlanner::from_obstacles(&ox, &oy, 1.0, 0.0);

        let path = planner
            .plan(Point2D::new(5.0, 5.0), Point2D::new(35.0, 45.0))
            .unwrap();

        let first = path.points.first().unwrap();
        let last = path.points.last().unwrap();
        assert!((first.x - 5.0).abs() < 1e-6);
        assert!((first.y - 5.0).abs() < 1e-6);
        assert!((last.x - 35.0).abs() < 1e-6);
        assert!((last.y - 45.0).abs() < 1e-6);

        let upstream_reference_length = polyline_length(&upstream_jump_point_reference_polyline());
        assert!(
            path.total_length() <= upstream_reference_length + 1e-6,
            "Rust JPS path should stay no worse than PythonRobotics jump-point variant reference, got {} vs {}",
            path.total_length(),
            upstream_reference_length,
        );

        let grid = planner.grid_map();
        for point in &path.points {
            let ix = grid.calc_x_index(point.x);
            let iy = grid.calc_y_index(point.y);
            assert!(
                grid.is_valid(ix, iy),
                "path point ({}, {}) should stay collision-free",
                point.x,
                point.y
            );
        }
    }

    #[test]
    fn test_jps_solves_moving_ai_arena2_bucket_80() {
        assert_pure_jps_matches_moving_ai_bucket(
            "arena2",
            include_str!("../benchdata/moving_ai/dao/arena2.map"),
            include_str!("../benchdata/moving_ai/dao/arena2.map.scen"),
            80,
        );
    }

    #[test]
    fn test_jps_reports_cost_mismatch_metrics_on_moving_ai_bucket_80_subset() {
        for (name, map_str, scen_str) in MOVING_AI_CASES {
            assert_pure_jps_matches_moving_ai_bucket(name, map_str, scen_str, 80);
        }
    }

    #[test]
    fn test_jps_solves_sampled_moving_ai_buckets_without_fallback() {
        const BUCKETS: [u32; 5] = [0, 20, 40, 60, 80];

        for (name, map_str, scen_str) in MOVING_AI_CASES {
            for bucket in BUCKETS {
                assert_pure_jps_matches_moving_ai_bucket(name, map_str, scen_str, bucket);
            }
        }
    }

    #[test]
    fn test_jps_solves_long_tail_moving_ai_buckets_without_fallback() {
        let cases: [(&str, &str, &str, &[u32]); 3] = [
            (
                "arena2",
                include_str!("../benchdata/moving_ai/dao/arena2.map"),
                include_str!("../benchdata/moving_ai/dao/arena2.map.scen"),
                &[90],
            ),
            (
                "8room_000",
                include_str!("../benchdata/moving_ai/room/8room_000.map"),
                include_str!("../benchdata/moving_ai/room/8room_000.map.scen"),
                &[120, 160, 213],
            ),
            (
                "random512-10-0",
                include_str!("../benchdata/moving_ai/random/random512-10-0.map"),
                include_str!("../benchdata/moving_ai/random/random512-10-0.map.scen"),
                &[100, 140, 177],
            ),
        ];

        for (name, map_str, scen_str, buckets) in cases {
            for bucket in buckets {
                assert_pure_jps_matches_moving_ai_bucket(name, map_str, scen_str, *bucket);
            }
        }
    }

    #[test]
    fn test_jps_solves_sampled_moving_ai_maze_buckets_without_fallback() {
        const BUCKETS: [u32; 6] = [0, 20, 40, 60, 80, 120];

        for bucket in BUCKETS {
            assert_pure_jps_matches_moving_ai_bucket(
                "maze512-1-0",
                include_str!("../benchdata/moving_ai/maze/maze512-1-0.map"),
                include_str!("../benchdata/moving_ai/maze/maze512-1-0.map.scen"),
                bucket,
            );
        }
    }

    #[test]
    fn test_jps_solves_long_tail_moving_ai_maze_buckets_without_fallback() {
        const BUCKETS: [u32; 4] = [200, 400, 800, 1211];

        for bucket in BUCKETS {
            assert_pure_jps_matches_moving_ai_bucket(
                "maze512-1-0",
                include_str!("../benchdata/moving_ai/maze/maze512-1-0.map"),
                include_str!("../benchdata/moving_ai/maze/maze512-1-0.map.scen"),
                bucket,
            );
        }
    }

    #[test]
    fn test_jps_solves_sampled_moving_ai_street_buckets_without_fallback() {
        const BUCKETS: [u32; 8] = [0, 20, 40, 60, 80, 120, 160, 186];

        for bucket in BUCKETS {
            assert_pure_jps_matches_moving_ai_bucket(
                "Berlin_0_512",
                include_str!("../benchdata/moving_ai/street/Berlin_0_512.map"),
                include_str!("../benchdata/moving_ai/street/Berlin_0_512.map.scen"),
                bucket,
            );
        }
    }

    #[test]
    fn test_jps_vs_a_star_runtime_on_long_tail_moving_ai_subset() {
        const ITERATIONS: usize = 7;
        let cases: [(&str, &str, &str, u32); 5] = [
            (
                "arena2",
                include_str!("../benchdata/moving_ai/dao/arena2.map"),
                include_str!("../benchdata/moving_ai/dao/arena2.map.scen"),
                90,
            ),
            (
                "8room_000",
                include_str!("../benchdata/moving_ai/room/8room_000.map"),
                include_str!("../benchdata/moving_ai/room/8room_000.map.scen"),
                213,
            ),
            (
                "random512-10-0",
                include_str!("../benchdata/moving_ai/random/random512-10-0.map"),
                include_str!("../benchdata/moving_ai/random/random512-10-0.map.scen"),
                177,
            ),
            (
                "maze512-1-0",
                include_str!("../benchdata/moving_ai/maze/maze512-1-0.map"),
                include_str!("../benchdata/moving_ai/maze/maze512-1-0.map.scen"),
                1211,
            ),
            (
                "Berlin_0_512",
                include_str!("../benchdata/moving_ai/street/Berlin_0_512.map"),
                include_str!("../benchdata/moving_ai/street/Berlin_0_512.map.scen"),
                186,
            ),
        ];

        for (name, map_str, scen_str, bucket) in cases {
            let (a_star, jps, start, goal, optimal_length) =
                build_moving_ai_bucket_planners(name, map_str, scen_str, bucket);

            let warmup_a = a_star
                .plan(start, goal)
                .unwrap_or_else(|_| panic!("A* should solve {name} bucket {bucket}"));
            assert!(
                (warmup_a.total_length() - optimal_length).abs() <= 1e-6,
                "A* warmup path for {name} bucket {bucket} should match MovingAI optimal {}",
                optimal_length
            );
            let warmup_j = jps
                .plan(start, goal)
                .unwrap_or_else(|_| panic!("JPS should solve {name} bucket {bucket}"));
            assert!(
                (warmup_j.total_length() - optimal_length).abs() <= 1e-6,
                "JPS warmup path for {name} bucket {bucket} should match MovingAI optimal {}",
                optimal_length
            );

            let a_star_median_us = measure_planner_median_us(ITERATIONS, || {
                let path = a_star
                    .plan(start, goal)
                    .unwrap_or_else(|_| panic!("A* should solve {name} bucket {bucket}"));
                assert!(
                    (path.total_length() - optimal_length).abs() <= 1e-6,
                    "A* path length {} should match MovingAI optimal {} on {name} bucket {bucket}",
                    path.total_length(),
                    optimal_length
                );
                path
            });
            let jps_median_us = measure_planner_median_us(ITERATIONS, || {
                let path = jps
                    .plan(start, goal)
                    .unwrap_or_else(|_| panic!("JPS should solve {name} bucket {bucket}"));
                assert!(
                    (path.total_length() - optimal_length).abs() <= 1e-6,
                    "JPS path length {} should match MovingAI optimal {} on {name} bucket {bucket}",
                    path.total_length(),
                    optimal_length
                );
                path
            });
            println!(
                "{name}/bucket{bucket}: a_star_median_us={:.2}, jps_median_us={:.2}, a_over_j={:.3}",
                a_star_median_us,
                jps_median_us,
                a_star_median_us / jps_median_us
            );
        }
    }

    #[test]
    fn test_jps_vs_a_star_runtime_crossover_samples_on_moving_ai() {
        const ITERATIONS: usize = 5;
        let families: [(&str, &str, &str, &[u32]); 5] = [
            (
                "arena2",
                include_str!("../benchdata/moving_ai/dao/arena2.map"),
                include_str!("../benchdata/moving_ai/dao/arena2.map.scen"),
                &[70, 75, 80, 85, 90],
            ),
            (
                "8room_000",
                include_str!("../benchdata/moving_ai/room/8room_000.map"),
                include_str!("../benchdata/moving_ai/room/8room_000.map.scen"),
                &[80, 120, 160, 213],
            ),
            (
                "random512-10-0",
                include_str!("../benchdata/moving_ai/random/random512-10-0.map"),
                include_str!("../benchdata/moving_ai/random/random512-10-0.map.scen"),
                &[80, 90, 100, 120, 140, 160, 177],
            ),
            (
                "maze512-1-0",
                include_str!("../benchdata/moving_ai/maze/maze512-1-0.map"),
                include_str!("../benchdata/moving_ai/maze/maze512-1-0.map.scen"),
                &[80, 120, 200, 400, 800, 1211],
            ),
            (
                "Berlin_0_512",
                include_str!("../benchdata/moving_ai/street/Berlin_0_512.map"),
                include_str!("../benchdata/moving_ai/street/Berlin_0_512.map.scen"),
                &[80, 120, 160, 186],
            ),
        ];

        for (name, map_str, scen_str, buckets) in families {
            for bucket in buckets {
                let (a_star, jps, start, goal, optimal_length) =
                    build_moving_ai_bucket_planners(name, map_str, scen_str, *bucket);

                let warmup_a = a_star
                    .plan(start, goal)
                    .unwrap_or_else(|_| panic!("A* should solve {name} bucket {bucket}"));
                assert!(
                    (warmup_a.total_length() - optimal_length).abs() <= 1e-6,
                    "A* warmup path for {name} bucket {bucket} should match MovingAI optimal {}",
                    optimal_length
                );
                let warmup_j = jps
                    .plan(start, goal)
                    .unwrap_or_else(|_| panic!("JPS should solve {name} bucket {bucket}"));
                assert!(
                    (warmup_j.total_length() - optimal_length).abs() <= 1e-6,
                    "JPS warmup path for {name} bucket {bucket} should match MovingAI optimal {}",
                    optimal_length
                );

                let a_star_median_us = measure_planner_median_us(ITERATIONS, || {
                    let path = a_star
                        .plan(start, goal)
                        .unwrap_or_else(|_| panic!("A* should solve {name} bucket {bucket}"));
                    assert!(
                        (path.total_length() - optimal_length).abs() <= 1e-6,
                        "A* path length {} should match MovingAI optimal {} on {name} bucket {bucket}",
                        path.total_length(),
                        optimal_length
                    );
                    path
                });
                let jps_median_us = measure_planner_median_us(ITERATIONS, || {
                    let path = jps
                        .plan(start, goal)
                        .unwrap_or_else(|_| panic!("JPS should solve {name} bucket {bucket}"));
                    assert!(
                        (path.total_length() - optimal_length).abs() <= 1e-6,
                        "JPS path length {} should match MovingAI optimal {} on {name} bucket {bucket}",
                        path.total_length(),
                        optimal_length
                    );
                    path
                });
                println!(
                    "{name}/bucket{bucket}: a_star_median_us={:.2}, jps_median_us={:.2}, a_over_j={:.3}",
                    a_star_median_us,
                    jps_median_us,
                    a_star_median_us / jps_median_us
                );
            }
        }
    }

    #[test]
    fn test_jps_vs_a_star_runtime_bucket_aggregate_samples_on_moving_ai() {
        const ITERATIONS: usize = 1;
        const SAMPLE_SLOTS: [usize; 3] = [0, 4, 9];
        let families: [(&str, &str, &str, &[u32]); 5] = [
            (
                "arena2",
                include_str!("../benchdata/moving_ai/dao/arena2.map"),
                include_str!("../benchdata/moving_ai/dao/arena2.map.scen"),
                &[80, 81, 82, 83, 84, 85],
            ),
            (
                "8room_000",
                include_str!("../benchdata/moving_ai/room/8room_000.map"),
                include_str!("../benchdata/moving_ai/room/8room_000.map.scen"),
                &[80, 90, 100, 110, 120],
            ),
            (
                "random512-10-0",
                include_str!("../benchdata/moving_ai/random/random512-10-0.map"),
                include_str!("../benchdata/moving_ai/random/random512-10-0.map.scen"),
                &[120, 125, 130, 135, 140],
            ),
            (
                "maze512-1-0",
                include_str!("../benchdata/moving_ai/maze/maze512-1-0.map"),
                include_str!("../benchdata/moving_ai/maze/maze512-1-0.map.scen"),
                &[80, 200, 1211],
            ),
            (
                "Berlin_0_512",
                include_str!("../benchdata/moving_ai/street/Berlin_0_512.map"),
                include_str!("../benchdata/moving_ai/street/Berlin_0_512.map.scen"),
                &[80, 100, 120, 140, 160, 180, 186],
            ),
        ];

        for (name, map_str, scen_str, buckets) in families {
            let (map, scenarios, a_star, jps) =
                build_moving_ai_family_planners(name, map_str, scen_str);

            for bucket in buckets {
                let aggregate = measure_moving_ai_bucket_runtime_aggregate(
                    name,
                    &map,
                    &scenarios,
                    &a_star,
                    &jps,
                    *bucket,
                    &SAMPLE_SLOTS,
                    ITERATIONS,
                );
                println!(
                    "{name}/bucket{bucket}: sample_slots={:?}, a_star_bucket_median_us={:.2}, jps_bucket_median_us={:.2}, a_over_j={:.3}, jps_wins={}/{}",
                    SAMPLE_SLOTS,
                    aggregate.a_star_median_us,
                    aggregate.jps_median_us,
                    aggregate.a_star_median_us / aggregate.jps_median_us,
                    aggregate.jps_wins,
                    aggregate.sample_count
                );
            }
        }
    }

    #[test]
    fn test_jps_vs_a_star_runtime_bucket_full_aggregate_on_narrowed_windows() {
        const ITERATIONS: usize = 1;
        const FULL_BUCKET_SLOTS: [usize; 10] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9];
        let families: [(&str, &str, &str, &[u32]); 4] = [
            (
                "arena2",
                include_str!("../benchdata/moving_ai/dao/arena2.map"),
                include_str!("../benchdata/moving_ai/dao/arena2.map.scen"),
                &[86, 87, 88, 89, 90],
            ),
            (
                "8room_000",
                include_str!("../benchdata/moving_ai/room/8room_000.map"),
                include_str!("../benchdata/moving_ai/room/8room_000.map.scen"),
                &[80, 85, 90],
            ),
            (
                "random512-10-0",
                include_str!("../benchdata/moving_ai/random/random512-10-0.map"),
                include_str!("../benchdata/moving_ai/random/random512-10-0.map.scen"),
                &[130, 132, 135],
            ),
            (
                "Berlin_0_512",
                include_str!("../benchdata/moving_ai/street/Berlin_0_512.map"),
                include_str!("../benchdata/moving_ai/street/Berlin_0_512.map.scen"),
                &[120, 140, 160, 186],
            ),
        ];

        for (name, map_str, scen_str, buckets) in families {
            let (map, scenarios, a_star, jps) =
                build_moving_ai_family_planners(name, map_str, scen_str);

            for bucket in buckets {
                let aggregate = measure_moving_ai_bucket_runtime_aggregate(
                    name,
                    &map,
                    &scenarios,
                    &a_star,
                    &jps,
                    *bucket,
                    &FULL_BUCKET_SLOTS,
                    ITERATIONS,
                );
                println!(
                    "{name}/bucket{bucket}: full_bucket_slots=0..9, a_star_bucket_median_us={:.2}, jps_bucket_median_us={:.2}, a_over_j={:.3}, a_star_range_us=[{:.2}, {:.2}], jps_range_us=[{:.2}, {:.2}], jps_wins={}/{}",
                    aggregate.a_star_median_us,
                    aggregate.jps_median_us,
                    aggregate.a_star_median_us / aggregate.jps_median_us,
                    aggregate.a_star_min_us,
                    aggregate.a_star_max_us,
                    aggregate.jps_min_us,
                    aggregate.jps_max_us,
                    aggregate.jps_wins,
                    aggregate.sample_count
                );
            }
        }
    }
}
