//! Iterative Deepening A* (IDA*) path planning algorithm.
//!
//! IDA* combines the admissible heuristic of A* with repeated depth-first
//! contour searches over increasing `f = g + h` thresholds. It uses much less
//! memory than A* at the cost of re-expanding states across iterations.

use std::collections::{HashMap, HashSet};
use std::f64::consts::SQRT_2;

use crate::grid::{get_motion_model_8, GridMap};
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

const COST_EPSILON: f64 = 1e-9;

/// Configuration for IDA*.
#[derive(Debug, Clone)]
pub struct IDAStarConfig {
    /// Grid resolution in meters.
    pub resolution: f64,
    /// Robot radius for obstacle inflation.
    pub robot_radius: f64,
    /// Heuristic weight (1.0 = admissible, >1.0 = weighted search).
    pub heuristic_weight: f64,
    /// Depth of optimistic local backup used to raise the initial threshold.
    pub initial_lookahead_depth: usize,
    /// Maximum number of threshold iterations.
    pub max_iterations: usize,
    /// Optional hard cap on expanded nodes for one query.
    pub max_expanded_nodes: Option<usize>,
}

impl Default for IDAStarConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
            initial_lookahead_depth: 2,
            max_iterations: 10_000,
            max_expanded_nodes: None,
        }
    }
}

impl IDAStarConfig {
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
        if self.max_iterations == 0 {
            return Err(RoboticsError::InvalidParameter(
                "max_iterations must be greater than zero".to_string(),
            ));
        }
        if matches!(self.max_expanded_nodes, Some(0)) {
            return Err(RoboticsError::InvalidParameter(
                "max_expanded_nodes must be greater than zero when provided".to_string(),
            ));
        }

        Ok(())
    }
}

enum SearchOutcome {
    Found,
    NextThreshold(f64),
    ExpansionBudgetExceeded,
}

struct SearchContext<'a> {
    goal: (i32, i32),
    threshold: f64,
    path: &'a mut Vec<(i32, i32)>,
    visited: &'a mut HashSet<i32>,
    best_g_by_grid: &'a mut HashMap<i32, f64>,
    expanded_grid_ids: &'a mut HashSet<i32>,
    stats: &'a mut IDAStarSearchStats,
}

/// IDA* planner on an 8-connected occupancy grid.
#[derive(Debug)]
pub struct IDAStarPlanner {
    grid_map: GridMap,
    config: IDAStarConfig,
    motion: Vec<(i32, i32, f64)>,
}

/// Per-contour (per-threshold-iteration) statistics for IDA* diagnostics.
///
/// Each contour corresponds to one depth-first search pass at a fixed
/// `f`-cost threshold. Tracking per-contour effort reveals how re-expansion
/// grows across successive thresholds and where the budget is consumed.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ContourStats {
    /// The `f`-cost threshold used for this contour.
    pub threshold: f64,
    /// Nodes expanded during this contour (includes re-expansions).
    pub expanded: usize,
    /// Nodes expanded here that were never expanded in any prior contour.
    pub new_unique: usize,
    /// Nodes expanded here that were already expanded in a prior contour.
    pub reexpanded: usize,
    /// Nodes generated (pushed onto the DFS stack) during this contour.
    pub generated: usize,
    /// Nodes pruned by `f > threshold` during this contour.
    pub threshold_prunes: usize,
    /// Nodes pruned by the current-path cycle check during this contour.
    pub cycle_prunes: usize,
    /// Nodes pruned by transposition (cost) comparison during this contour.
    pub transposition_prunes: usize,
    /// Maximum recursion depth reached during this contour.
    pub max_depth: usize,
}

/// Search-effort statistics collected during one IDA* query.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct IDAStarSearchStats {
    pub threshold_iterations: usize,
    pub expanded_nodes: usize,
    pub unique_expanded_nodes: usize,
    pub reexpanded_nodes: usize,
    pub generated_nodes: usize,
    pub threshold_prunes: usize,
    pub cycle_prunes: usize,
    pub transposition_prunes: usize,
    pub max_depth: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IDAStarPlanOutcome {
    Exact,
    MaxIterations,
    MaxExpandedNodes,
    NoPath,
}

#[derive(Debug, Clone)]
pub struct IDAStarPlanReport {
    pub outcome: IDAStarPlanOutcome,
    pub path: Option<Path2D>,
    pub stats: IDAStarSearchStats,
    pub initial_threshold: f64,
    pub last_searched_threshold: f64,
    pub next_threshold: Option<f64>,
    /// Per-contour effort breakdown, one entry per threshold iteration.
    pub contour_history: Vec<ContourStats>,
}

impl IDAStarPlanner {
    /// Create a new planner from obstacle x/y vectors.
    pub fn new(ox: &[f64], oy: &[f64], config: IDAStarConfig) -> Self {
        Self::try_new(ox, oy, config).expect(
            "invalid IDA* planner input: obstacle list must be non-empty and valid, and config values must be positive/finite",
        )
    }

    /// Create a validated planner from obstacle x/y vectors.
    pub fn try_new(ox: &[f64], oy: &[f64], config: IDAStarConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.resolution, config.robot_radius)?;

        Ok(Self {
            grid_map,
            config,
            motion: get_motion_model_8(),
        })
    }

    /// Create from obstacle x/y vectors with explicit scalar parameters.
    pub fn from_obstacles(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let config = IDAStarConfig {
            resolution,
            robot_radius,
            ..Default::default()
        };
        Self::new(ox, oy, config)
    }

    /// Create from typed obstacle points.
    pub fn from_obstacle_points(
        obstacles: &Obstacles,
        config: IDAStarConfig,
    ) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::from_obstacles(obstacles, config.resolution, config.robot_radius)?;

        Ok(Self {
            grid_map,
            config,
            motion: get_motion_model_8(),
        })
    }

    /// Plan without importing the trait explicitly.
    pub fn plan(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        self.plan_with_stats(start, goal).map(|(path, _stats)| path)
    }

    /// Plan from raw coordinates without importing the trait explicitly.
    pub fn plan_xy(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> RoboticsResult<Path2D> {
        self.plan_with_stats(Point2D::new(sx, sy), Point2D::new(gx, gy))
            .map(|(path, _stats)| path)
    }

    /// Plan a path and return per-query search-effort statistics.
    pub fn plan_with_stats(
        &self,
        start: Point2D,
        goal: Point2D,
    ) -> RoboticsResult<(Path2D, IDAStarSearchStats)> {
        let report = self.plan_with_report(start, goal)?;
        match report.outcome {
            IDAStarPlanOutcome::Exact => Ok((
                report
                    .path
                    .expect("exact IDA* report should always include a path"),
                report.stats,
            )),
            IDAStarPlanOutcome::MaxExpandedNodes => Err(RoboticsError::PlanningError(
                "IDA* exceeded max_expanded_nodes before finding a path".to_string(),
            )),
            IDAStarPlanOutcome::MaxIterations => Err(RoboticsError::PlanningError(
                "IDA* exceeded max_iterations before finding a path".to_string(),
            )),
            IDAStarPlanOutcome::NoPath => {
                Err(RoboticsError::PlanningError("No path found".to_string()))
            }
        }
    }

    /// Plan a path and return the query outcome plus search-effort diagnostics,
    /// even when the bounded search stops before reaching an exact path.
    pub fn plan_with_report(
        &self,
        start: Point2D,
        goal: Point2D,
    ) -> RoboticsResult<IDAStarPlanReport> {
        self.plan_impl(start, goal)
    }

    /// Plan a path returning `(rx, ry)` vectors.
    #[deprecated(note = "use plan() or plan_xy() instead")]
    pub fn planning(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> Option<(Vec<f64>, Vec<f64>)> {
        match self.plan_xy(sx, sy, gx, gy) {
            Ok(path) => Some((path.x_coords(), path.y_coords())),
            Err(_) => None,
        }
    }

    pub fn grid_map(&self) -> &GridMap {
        &self.grid_map
    }

    fn calc_heuristic(&self, x: i32, y: i32, goal_x: i32, goal_y: i32) -> f64 {
        let dx = (x - goal_x).abs() as f64;
        let dy = (y - goal_y).abs() as f64;
        let diagonal = dx.min(dy);
        let straight = dx.max(dy) - diagonal;

        // Octile distance is a tighter admissible lower bound than Euclidean
        // for an 8-connected grid with unit cardinal and sqrt(2) diagonal cost.
        self.config.heuristic_weight * (diagonal * SQRT_2 + straight)
    }

    fn lookahead_lower_bound(
        &self,
        current: (i32, i32),
        goal: (i32, i32),
        depth: usize,
        cache: &mut HashMap<(i32, usize), f64>,
    ) -> f64 {
        let (current_x, current_y) = current;
        let (goal_x, goal_y) = goal;
        if current_x == goal_x && current_y == goal_y {
            return 0.0;
        }
        if depth == 0 {
            return self.calc_heuristic(current_x, current_y, goal_x, goal_y);
        }

        let cache_key = (self.grid_map.calc_index(current_x, current_y), depth);
        if let Some(&cached) = cache.get(&cache_key) {
            return cached;
        }

        let mut best = f64::INFINITY;
        for &(dx, dy, move_cost) in &self.motion {
            if !self.grid_map.is_valid_offset(current_x, current_y, dx, dy) {
                continue;
            }

            let next_x = current_x + dx;
            let next_y = current_y + dy;
            let candidate =
                move_cost + self.lookahead_lower_bound((next_x, next_y), goal, depth - 1, cache);
            best = best.min(candidate);
        }

        let value = if best.is_finite() {
            best
        } else {
            self.calc_heuristic(current_x, current_y, goal_x, goal_y)
        };
        cache.insert(cache_key, value);
        value
    }

    fn calc_initial_threshold(&self, start_x: i32, start_y: i32, goal_x: i32, goal_y: i32) -> f64 {
        let mut cache = HashMap::new();
        self.lookahead_lower_bound(
            (start_x, start_y),
            (goal_x, goal_y),
            self.config.initial_lookahead_depth,
            &mut cache,
        )
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

    fn build_path(&self, path_cells: &[(i32, i32)]) -> Path2D {
        let points = path_cells
            .iter()
            .map(|&(x, y)| {
                Point2D::new(
                    self.grid_map.calc_x_position(x),
                    self.grid_map.calc_y_position(y),
                )
            })
            .collect();
        Path2D::from_points(points)
    }

    fn search_contour(
        &self,
        current: (i32, i32),
        g_cost: f64,
        depth: usize,
        ctx: &mut SearchContext<'_>,
    ) -> SearchOutcome {
        let (current_x, current_y) = current;
        let (goal_x, goal_y) = ctx.goal;
        ctx.stats.max_depth = ctx.stats.max_depth.max(depth);
        let f_cost = g_cost + self.calc_heuristic(current_x, current_y, goal_x, goal_y);
        if f_cost > ctx.threshold + COST_EPSILON {
            ctx.stats.threshold_prunes += 1;
            return SearchOutcome::NextThreshold(f_cost);
        }

        if self
            .config
            .max_expanded_nodes
            .is_some_and(|limit| ctx.stats.expanded_nodes >= limit)
        {
            return SearchOutcome::ExpansionBudgetExceeded;
        }

        let current_grid_index = self.grid_map.calc_index(current_x, current_y);
        ctx.stats.expanded_nodes += 1;
        if ctx.expanded_grid_ids.insert(current_grid_index) {
            ctx.stats.unique_expanded_nodes += 1;
        } else {
            ctx.stats.reexpanded_nodes += 1;
        }

        if current_x == goal_x && current_y == goal_y {
            return SearchOutcome::Found;
        }

        let mut min_exceeded = f64::INFINITY;
        let mut ordered_neighbors = Vec::with_capacity(self.motion.len());

        for &(dx, dy, move_cost) in &self.motion {
            if !self.grid_map.is_valid_offset(current_x, current_y, dx, dy) {
                continue;
            }

            let next_x = current_x + dx;
            let next_y = current_y + dy;
            let next_cost = g_cost + move_cost;
            let next_f_cost = next_cost + self.calc_heuristic(next_x, next_y, goal_x, goal_y);
            ordered_neighbors.push((next_f_cost, next_x, next_y, next_cost));
        }

        ordered_neighbors.sort_by(|lhs, rhs| {
            lhs.0.total_cmp(&rhs.0).then_with(|| {
                self.calc_heuristic(lhs.1, lhs.2, goal_x, goal_y)
                    .total_cmp(&self.calc_heuristic(rhs.1, rhs.2, goal_x, goal_y))
            })
        });

        for (_next_f_cost, next_x, next_y, next_cost) in ordered_neighbors {
            let next_grid_index = self.grid_map.calc_index(next_x, next_y);
            if ctx.visited.contains(&next_grid_index) {
                ctx.stats.cycle_prunes += 1;
                continue;
            }

            if ctx
                .best_g_by_grid
                .get(&next_grid_index)
                .is_some_and(|&best_g| next_cost >= best_g - COST_EPSILON)
            {
                ctx.stats.transposition_prunes += 1;
                continue;
            }

            ctx.stats.generated_nodes += 1;
            ctx.best_g_by_grid.insert(next_grid_index, next_cost);
            ctx.visited.insert(next_grid_index);
            ctx.path.push((next_x, next_y));

            match self.search_contour((next_x, next_y), next_cost, depth + 1, ctx) {
                SearchOutcome::Found => return SearchOutcome::Found,
                SearchOutcome::ExpansionBudgetExceeded => {
                    return SearchOutcome::ExpansionBudgetExceeded
                }
                SearchOutcome::NextThreshold(limit) => {
                    min_exceeded = min_exceeded.min(limit);
                }
            }

            ctx.path.pop();
            ctx.visited.remove(&next_grid_index);
        }

        SearchOutcome::NextThreshold(min_exceeded)
    }

    fn plan_impl(&self, start: Point2D, goal: Point2D) -> RoboticsResult<IDAStarPlanReport> {
        let start_x = self.grid_map.calc_x_index(start.x);
        let start_y = self.grid_map.calc_y_index(start.y);
        let goal_x = self.grid_map.calc_x_index(goal.x);
        let goal_y = self.grid_map.calc_y_index(goal.y);

        self.ensure_query_is_valid(start_x, start_y, "Start")?;
        self.ensure_query_is_valid(goal_x, goal_y, "Goal")?;

        let initial_threshold = self.calc_initial_threshold(start_x, start_y, goal_x, goal_y);
        let mut threshold = initial_threshold;
        let mut last_searched_threshold = threshold;
        let mut next_threshold = None;
        let start_grid_index = self.grid_map.calc_index(start_x, start_y);
        let mut stats = IDAStarSearchStats::default();
        let mut expanded_grid_ids = HashSet::new();
        let mut contour_history = Vec::new();

        for _ in 0..self.config.max_iterations {
            last_searched_threshold = threshold;
            let mut path = vec![(start_x, start_y)];
            let mut visited = HashSet::new();
            let mut best_g_by_grid = HashMap::new();
            visited.insert(start_grid_index);
            best_g_by_grid.insert(start_grid_index, 0.0);
            stats.threshold_iterations += 1;

            let snap = stats;
            let outcome = {
                let mut ctx = SearchContext {
                    goal: (goal_x, goal_y),
                    threshold,
                    path: &mut path,
                    visited: &mut visited,
                    best_g_by_grid: &mut best_g_by_grid,
                    expanded_grid_ids: &mut expanded_grid_ids,
                    stats: &mut stats,
                };
                self.search_contour((start_x, start_y), 0.0, 1, &mut ctx)
            };

            let contour_expanded = stats.expanded_nodes - snap.expanded_nodes;
            let contour_reexpanded = stats.reexpanded_nodes - snap.reexpanded_nodes;
            contour_history.push(ContourStats {
                threshold,
                expanded: contour_expanded,
                new_unique: contour_expanded - contour_reexpanded,
                reexpanded: contour_reexpanded,
                generated: stats.generated_nodes - snap.generated_nodes,
                threshold_prunes: stats.threshold_prunes - snap.threshold_prunes,
                cycle_prunes: stats.cycle_prunes - snap.cycle_prunes,
                transposition_prunes: stats.transposition_prunes - snap.transposition_prunes,
                max_depth: stats.max_depth,
            });

            match outcome {
                SearchOutcome::Found => {
                    return Ok(IDAStarPlanReport {
                        outcome: IDAStarPlanOutcome::Exact,
                        path: Some(self.build_path(&path)),
                        stats,
                        initial_threshold,
                        last_searched_threshold,
                        next_threshold: None,
                        contour_history,
                    });
                }
                SearchOutcome::ExpansionBudgetExceeded => {
                    return Ok(IDAStarPlanReport {
                        outcome: IDAStarPlanOutcome::MaxExpandedNodes,
                        path: None,
                        stats,
                        initial_threshold,
                        last_searched_threshold,
                        next_threshold: None,
                        contour_history,
                    });
                }
                SearchOutcome::NextThreshold(candidate_threshold)
                    if candidate_threshold.is_finite() =>
                {
                    next_threshold = Some(candidate_threshold);
                    threshold = candidate_threshold;
                }
                SearchOutcome::NextThreshold(_) => {
                    return Ok(IDAStarPlanReport {
                        outcome: IDAStarPlanOutcome::NoPath,
                        path: None,
                        stats,
                        initial_threshold,
                        last_searched_threshold,
                        next_threshold: None,
                        contour_history,
                    });
                }
            }
        }

        Ok(IDAStarPlanReport {
            outcome: IDAStarPlanOutcome::MaxIterations,
            path: None,
            stats,
            initial_threshold,
            last_searched_threshold,
            next_threshold,
            contour_history,
        })
    }
}

impl PathPlanner for IDAStarPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        IDAStarPlanner::plan(self, start, goal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::a_star::{AStarConfig, AStarPlanner};
    use crate::moving_ai::{MovingAiMap, MovingAiScenario};

    fn create_simple_obstacles() -> (Vec<f64>, Vec<f64>) {
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

    #[test]
    fn test_ida_star_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = IDAStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let path = planner
            .plan(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0))
            .expect("IDA* should find a path on the simple map");

        assert!(!path.is_empty());
    }

    #[test]
    fn test_ida_star_try_new_rejects_invalid_config() {
        let (ox, oy) = create_simple_obstacles();
        let config = IDAStarConfig {
            max_iterations: 0,
            ..Default::default()
        };

        let err = IDAStarPlanner::try_new(&ox, &oy, config)
            .expect_err("invalid max_iterations should be rejected");
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_ida_star_try_new_rejects_zero_expansion_budget() {
        let (ox, oy) = create_simple_obstacles();
        let config = IDAStarConfig {
            max_expanded_nodes: Some(0),
            ..Default::default()
        };

        let err = IDAStarPlanner::try_new(&ox, &oy, config)
            .expect_err("zero expansion budget should be rejected");
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_ida_star_matches_simple_map_length() {
        let (ox, oy) = create_simple_obstacles();
        let planner = IDAStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);
        let a_star = AStarPlanner::new(
            &ox,
            &oy,
            AStarConfig {
                resolution: 1.0,
                robot_radius: 0.5,
                heuristic_weight: 1.0,
            },
        );

        let path = planner
            .plan(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0))
            .expect("IDA* should find a path on the simple map");
        let reference = a_star
            .plan(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0))
            .expect("A* should find the same reference path on the simple map");

        assert!(
            (path.total_length() - reference.total_length()).abs() < 1e-6,
            "simple-map path length {} should match A* reference length {}",
            path.total_length(),
            reference.total_length()
        );
    }

    #[test]
    #[ignore = "long-running MovingAI benchmark"]
    fn test_ida_star_matches_sample_moving_ai_optimal_length() {
        let map = MovingAiMap::parse_str(include_str!("testdata/moving_ai/sample.map"))
            .expect("sample MovingAI map should parse");
        let scenario =
            MovingAiScenario::parse_str(include_str!("testdata/moving_ai/sample.map.scen"))
                .expect("sample MovingAI scenarios should parse")
                .into_iter()
                .next()
                .expect("sample MovingAI scenario should exist");

        let planner = IDAStarPlanner::from_obstacle_points(
            &map.to_obstacles(),
            IDAStarConfig {
                resolution: 1.0,
                robot_radius: 0.5,
                heuristic_weight: 1.0,
                initial_lookahead_depth: 2,
                max_iterations: 10_000,
                max_expanded_nodes: None,
            },
        )
        .expect("IDA* planner should build from sample obstacles");

        let start = map
            .planning_point(scenario.start_x, scenario.start_y)
            .expect("sample start should be valid");
        let goal = map
            .planning_point(scenario.goal_x, scenario.goal_y)
            .expect("sample goal should be valid");

        let path = planner
            .plan(start, goal)
            .expect("IDA* should solve the sample scenario");

        assert!(
            (path.total_length() - scenario.optimal_length).abs() < 1e-6,
            "IDA* path length {} should match sample MovingAI optimal {} when corner cutting is disabled",
            path.total_length(),
            scenario.optimal_length
        );
    }

    #[test]
    fn test_ida_star_report_keeps_failure_effort_on_iteration_limit() {
        let (ox, oy) = create_simple_obstacles();
        let planner = IDAStarPlanner::try_new(
            &ox,
            &oy,
            IDAStarConfig {
                max_iterations: 1,
                max_expanded_nodes: Some(10_000),
                ..Default::default()
            },
        )
        .expect("bounded IDA* planner should build");

        let report = planner
            .plan_with_report(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0))
            .expect("valid query should return an IDA* report");

        assert_eq!(report.outcome, IDAStarPlanOutcome::MaxIterations);
        assert!(report.path.is_none());
        assert_eq!(report.stats.threshold_iterations, 1);
        assert!(report.stats.expanded_nodes > 0);
        assert!(report.stats.unique_expanded_nodes > 0);
        assert!(report.next_threshold.is_some());
        assert!(report.next_threshold.unwrap() > report.last_searched_threshold);
    }

    #[test]
    fn test_ida_star_report_keeps_failure_effort_on_expansion_limit() {
        let (ox, oy) = create_simple_obstacles();
        let planner = IDAStarPlanner::try_new(
            &ox,
            &oy,
            IDAStarConfig {
                max_iterations: 10_000,
                max_expanded_nodes: Some(1),
                ..Default::default()
            },
        )
        .expect("bounded IDA* planner should build");

        let report = planner
            .plan_with_report(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0))
            .expect("valid query should return an IDA* report");

        assert_eq!(report.outcome, IDAStarPlanOutcome::MaxExpandedNodes);
        assert!(report.path.is_none());
        assert_eq!(report.stats.expanded_nodes, 1);
        assert_eq!(report.stats.unique_expanded_nodes, 1);
        assert_eq!(report.stats.reexpanded_nodes, 0);
        assert!(report.next_threshold.is_none());
    }
}
