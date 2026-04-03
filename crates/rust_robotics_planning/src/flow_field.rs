//! Flow Field path planning algorithm
//!
//! Computes a vector field that points toward the goal from every
//! reachable cell using weighted shortest-path integration. An agent follows
//! the field from its start position to the goal.
//!
//! Reference: Leif Erkenbrach, "Flow Field Pathfinding" (2013)
//! <https://leifnode.com/2013/12/flow-field-pathfinding/>

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use crate::grid::GridMap;
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Configuration for the flow field planner.
#[derive(Debug, Clone)]
pub struct FlowFieldConfig {
    /// Grid resolution in metres
    pub resolution: f64,
    /// Robot radius for obstacle inflation
    pub robot_radius: f64,
}

impl Default for FlowFieldConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
        }
    }
}

impl FlowFieldConfig {
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
        Ok(())
    }
}

/// A direction vector stored per cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct FlowDir {
    dx: i32,
    dy: i32,
}

/// Result of computing a flow field toward a specific goal.
///
/// The integration field stores the shortest-path distance (using 10/14 costs for
/// cardinal/diagonal moves) from every reachable cell to the goal.  The
/// vector field stores, for each cell, the offset to its best neighbour.
#[derive(Debug, Clone)]
pub struct FlowFieldResult {
    /// Integration cost for each cell (row-major, `x_width * y_width`).
    /// `u32::MAX` means unreachable.
    pub integration: Vec<u32>,
    /// Flow direction per cell.  `(0, 0)` means goal or unreachable.
    directions: Vec<FlowDir>,
    /// Grid width along x-axis.
    pub x_width: i32,
    /// Grid width along y-axis.
    pub y_width: i32,
}

/// Flow field path planner.
pub struct FlowFieldPlanner {
    grid_map: GridMap,
    #[allow(dead_code)]
    config: FlowFieldConfig,
    /// 8-connected neighbour offsets with cost (10 = cardinal, 14 = diagonal).
    motion: [(i32, i32, u32); 8],
}

impl FlowFieldPlanner {
    /// Create a new planner (panics on invalid input).
    pub fn new(ox: &[f64], oy: &[f64], config: FlowFieldConfig) -> Self {
        Self::try_new(ox, oy, config).expect(
            "invalid flow field planner input: obstacle list must be non-empty and valid, \
             and config values must be positive/finite",
        )
    }

    /// Create a validated planner.
    pub fn try_new(ox: &[f64], oy: &[f64], config: FlowFieldConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.resolution, config.robot_radius)?;
        Ok(Self {
            grid_map,
            config,
            motion: Self::motion_model(),
        })
    }

    /// Create from obstacle x/y vectors with explicit resolution and radius.
    pub fn from_obstacles(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let config = FlowFieldConfig {
            resolution,
            robot_radius,
        };
        Self::new(ox, oy, config)
    }

    /// Create a validated planner from typed obstacle points.
    pub fn from_obstacle_points(
        obstacles: &Obstacles,
        config: FlowFieldConfig,
    ) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::from_obstacles(obstacles, config.resolution, config.robot_radius)?;
        Ok(Self {
            grid_map,
            config,
            motion: Self::motion_model(),
        })
    }

    /// Get reference to the underlying grid map.
    pub fn grid_map(&self) -> &GridMap {
        &self.grid_map
    }

    /// Compute the flow field toward the given goal and extract the path
    /// from start to goal.
    pub fn plan(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        self.plan_impl(start, goal)
    }

    /// Plan from raw coordinates.
    pub fn plan_xy(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> RoboticsResult<Path2D> {
        self.plan_impl(Point2D::new(sx, sy), Point2D::new(gx, gy))
    }

    /// Legacy interface returning `(rx, ry)` vectors.
    pub fn planning(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> Option<(Vec<f64>, Vec<f64>)> {
        match self.plan_xy(sx, sy, gx, gy) {
            Ok(path) => Some((path.x_coords(), path.y_coords())),
            Err(_) => None,
        }
    }

    /// Compute the full flow field toward the goal without extracting a
    /// single path.  Useful when many agents share the same goal.
    pub fn compute_field(&self, goal: Point2D) -> RoboticsResult<FlowFieldResult> {
        let gx = self.grid_map.calc_x_index(goal.x);
        let gy = self.grid_map.calc_y_index(goal.y);
        self.ensure_valid(gx, gy, "Goal")?;
        Ok(self.build_field(gx, gy))
    }

    // ------------------------------------------------------------------
    // Private helpers
    // ------------------------------------------------------------------

    fn motion_model() -> [(i32, i32, u32); 8] {
        [
            (1, 0, 10),
            (0, 1, 10),
            (-1, 0, 10),
            (0, -1, 10),
            (-1, -1, 14),
            (-1, 1, 14),
            (1, -1, 14),
            (1, 1, 14),
        ]
    }

    fn ensure_valid(&self, x: i32, y: i32, label: &str) -> RoboticsResult<()> {
        if self.grid_map.is_valid(x, y) {
            Ok(())
        } else {
            Err(RoboticsError::PlanningError(format!(
                "{} position is invalid",
                label
            )))
        }
    }

    fn flat_index(&self, x: i32, y: i32) -> usize {
        (x as usize) * (self.grid_map.y_width as usize) + (y as usize)
    }

    /// Build the integration field (Dijkstra from goal) and derive the vector
    /// field.
    fn build_field(&self, gx: i32, gy: i32) -> FlowFieldResult {
        let w = self.grid_map.x_width as usize;
        let h = self.grid_map.y_width as usize;
        let total = w * h;

        let mut integration = vec![u32::MAX; total];
        let mut directions = vec![FlowDir { dx: 0, dy: 0 }; total];

        // Dijkstra from goal so the integration field respects 10/14 move costs.
        let goal_idx = self.flat_index(gx, gy);
        integration[goal_idx] = 0;

        let mut open = BinaryHeap::new();
        open.push((Reverse(0_u32), gx, gy));

        while let Some((Reverse(curr_cost), cx, cy)) = open.pop() {
            if curr_cost != integration[self.flat_index(cx, cy)] {
                continue;
            }

            let curr_cost = integration[self.flat_index(cx, cy)];
            for &(dx, dy, move_cost) in &self.motion {
                let nx = cx + dx;
                let ny = cy + dy;
                if !self.grid_map.is_valid_offset(cx, cy, dx, dy) {
                    continue;
                }
                let ni = self.flat_index(nx, ny);
                let new_cost = curr_cost.saturating_add(move_cost);
                if new_cost < integration[ni] {
                    integration[ni] = new_cost;
                    open.push((Reverse(new_cost), nx, ny));
                }
            }
        }

        // Derive vector field: each cell points to the neighbour that minimizes
        // remaining path cost including the step into that neighbour.
        for ix in 0..self.grid_map.x_width {
            for iy in 0..self.grid_map.y_width {
                if !self.grid_map.is_valid(ix, iy) {
                    continue;
                }
                let idx = self.flat_index(ix, iy);
                if integration[idx] == 0 {
                    // Goal cell
                    continue;
                }
                if integration[idx] == u32::MAX {
                    // Unreachable
                    continue;
                }

                let current_path_cost = integration[idx];
                let mut best_neighbor_cost = u32::MAX;
                let mut best_dx = 0i32;
                let mut best_dy = 0i32;
                for &(dx, dy, move_cost) in &self.motion {
                    let nx = ix + dx;
                    let ny = iy + dy;
                    if nx < 0
                        || ny < 0
                        || nx >= self.grid_map.x_width
                        || ny >= self.grid_map.y_width
                        || !self.grid_map.is_valid_offset(ix, iy, dx, dy)
                    {
                        continue;
                    }
                    let ni = self.flat_index(nx, ny);
                    if integration[ni] == u32::MAX {
                        continue;
                    }
                    let candidate_path_cost = integration[ni].saturating_add(move_cost);
                    if candidate_path_cost == current_path_cost
                        && integration[ni] < best_neighbor_cost
                    {
                        best_neighbor_cost = integration[ni];
                        best_dx = dx;
                        best_dy = dy;
                    }
                }
                directions[idx] = FlowDir {
                    dx: best_dx,
                    dy: best_dy,
                };
            }
        }

        FlowFieldResult {
            integration,
            directions,
            x_width: self.grid_map.x_width,
            y_width: self.grid_map.y_width,
        }
    }

    /// Follow the vector field from start to goal.
    fn extract_path(
        &self,
        field: &FlowFieldResult,
        sx: i32,
        sy: i32,
        gx: i32,
        gy: i32,
    ) -> RoboticsResult<Path2D> {
        let mut points = Vec::new();
        let mut cx = sx;
        let mut cy = sy;

        // Safety limit to avoid infinite loops on malformed fields.
        let max_steps = (self.grid_map.x_width as usize) * (self.grid_map.y_width as usize);

        for _ in 0..max_steps {
            points.push(Point2D::new(
                self.grid_map.calc_x_position(cx),
                self.grid_map.calc_y_position(cy),
            ));

            if cx == gx && cy == gy {
                return Ok(Path2D::from_points(points));
            }

            let idx = self.flat_index(cx, cy);
            let dir = field.directions[idx];
            if dir.dx == 0 && dir.dy == 0 {
                // Stuck (unreachable or at goal already handled above).
                return Err(RoboticsError::PlanningError(
                    "No path found: flow field has no direction at current cell".to_string(),
                ));
            }
            cx += dir.dx;
            cy += dir.dy;
        }

        Err(RoboticsError::PlanningError(
            "No path found: exceeded maximum step count".to_string(),
        ))
    }

    fn plan_impl(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        let sx = self.grid_map.calc_x_index(start.x);
        let sy = self.grid_map.calc_y_index(start.y);
        let gx = self.grid_map.calc_x_index(goal.x);
        let gy = self.grid_map.calc_y_index(goal.y);

        self.ensure_valid(sx, sy, "Start")?;
        self.ensure_valid(gx, gy, "Goal")?;

        let field = self.build_field(gx, gy);
        self.extract_path(&field, sx, sy, gx, gy)
    }
}

impl PathPlanner for FlowFieldPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        self.plan_impl(start, goal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::moving_ai::{MovingAiMap, MovingAiScenario};
    use rust_robotics_core::Obstacles;

    /// 10x10 box with boundary obstacles and a vertical wall at x=5.
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

        // Internal wall
        for i in 4..7 {
            ox.push(5.0);
            oy.push(i as f64);
        }

        (ox, oy)
    }

    #[test]
    fn test_flow_field_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = FlowFieldPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let result = planner.plan(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0));
        assert!(result.is_ok(), "expected path, got {:?}", result);

        let path = result.unwrap();
        assert!(!path.is_empty());
        // Path must start near the start point and end near the goal.
        let first = path.points.first().unwrap();
        let last = path.points.last().unwrap();
        assert!((first.x - 2.0).abs() < 1e-6);
        assert!((first.y - 2.0).abs() < 1e-6);
        assert!((last.x - 8.0).abs() < 1e-6);
        assert!((last.y - 8.0).abs() < 1e-6);
    }

    #[test]
    fn test_flow_field_legacy_interface() {
        let (ox, oy) = create_simple_obstacles();
        let planner = FlowFieldPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let result = planner.planning(2.0, 2.0, 8.0, 8.0);
        assert!(result.is_some());

        let (rx, ry) = result.unwrap();
        assert!(!rx.is_empty());
        assert_eq!(rx.len(), ry.len());
    }

    #[test]
    fn test_flow_field_from_obstacle_points() {
        let (ox, oy) = create_simple_obstacles();
        let obstacles = Obstacles::try_from_xy(&ox, &oy).unwrap();
        let planner =
            FlowFieldPlanner::from_obstacle_points(&obstacles, FlowFieldConfig::default()).unwrap();

        let path = planner.plan_xy(2.0, 2.0, 8.0, 8.0).unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_flow_field_invalid_config() {
        let (ox, oy) = create_simple_obstacles();
        let config = FlowFieldConfig {
            resolution: -1.0,
            ..Default::default()
        };
        let err = FlowFieldPlanner::try_new(&ox, &oy, config);
        assert!(err.is_err());
    }

    #[test]
    fn test_flow_field_invalid_start() {
        let (ox, oy) = create_simple_obstacles();
        let planner = FlowFieldPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let result = planner.plan(Point2D::new(-5.0, -5.0), Point2D::new(8.0, 8.0));
        assert!(result.is_err());
    }

    #[test]
    fn test_flow_field_invalid_goal() {
        let (ox, oy) = create_simple_obstacles();
        let planner = FlowFieldPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let result = planner.plan(Point2D::new(2.0, 2.0), Point2D::new(-5.0, -5.0));
        assert!(result.is_err());
    }

    #[test]
    fn test_flow_field_start_equals_goal() {
        let (ox, oy) = create_simple_obstacles();
        let planner = FlowFieldPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let path = planner
            .plan(Point2D::new(3.0, 3.0), Point2D::new(3.0, 3.0))
            .unwrap();
        // Path should contain exactly one point.
        assert_eq!(path.points.len(), 1);
    }

    #[test]
    fn test_flow_field_preserves_asymmetric_coordinates() {
        let mut obstacles = Obstacles::new();

        for x in 10..=20 {
            obstacles.push(Point2D::new(x as f64, -4.0));
            obstacles.push(Point2D::new(x as f64, 6.0));
        }
        for y in -4..=6 {
            obstacles.push(Point2D::new(10.0, y as f64));
            obstacles.push(Point2D::new(20.0, y as f64));
        }

        let planner =
            FlowFieldPlanner::from_obstacle_points(&obstacles, FlowFieldConfig::default()).unwrap();
        let path = planner.plan_xy(12.0, -2.0, 18.0, 4.0).unwrap();

        let first = path.points.first().unwrap();
        let last = path.points.last().unwrap();
        assert!((first.x - 12.0).abs() < 1e-6);
        assert!((first.y + 2.0).abs() < 1e-6);
        assert!((last.x - 18.0).abs() < 1e-6);
        assert!((last.y - 4.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_field_returns_valid_integration() {
        let (ox, oy) = create_simple_obstacles();
        let planner = FlowFieldPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let field = planner.compute_field(Point2D::new(8.0, 8.0)).unwrap();

        // Goal cell should have zero integration cost.
        let gx = planner.grid_map().calc_x_index(8.0);
        let gy = planner.grid_map().calc_y_index(8.0);
        let goal_idx = (gx as usize) * (planner.grid_map().y_width as usize) + (gy as usize);
        assert_eq!(field.integration[goal_idx], 0);
    }

    #[test]
    fn test_flow_field_path_monotonically_decreases_cost() {
        let (ox, oy) = create_simple_obstacles();
        let planner = FlowFieldPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let goal = Point2D::new(8.0, 8.0);
        let field = planner.compute_field(goal).unwrap();

        let path = planner.plan(Point2D::new(2.0, 2.0), goal).unwrap();

        // Each successive point must have lower or equal integration cost.
        let costs: Vec<u32> = path
            .points
            .iter()
            .map(|p| {
                let ix = planner.grid_map().calc_x_index(p.x);
                let iy = planner.grid_map().calc_y_index(p.y);
                let idx = (ix as usize) * (planner.grid_map().y_width as usize) + (iy as usize);
                field.integration[idx]
            })
            .collect();

        for window in costs.windows(2) {
            assert!(
                window[0] >= window[1],
                "cost should decrease along path, got {} -> {}",
                window[0],
                window[1]
            );
        }
    }

    #[test]
    fn test_flow_field_matches_moving_ai_random512_bucket_80_optimal_length() {
        let map = MovingAiMap::parse_str(include_str!(
            "../benchdata/moving_ai/random/random512-10-0.map"
        ))
        .expect("random512 map should parse");
        let scenario = MovingAiScenario::parse_str(include_str!(
            "../benchdata/moving_ai/random/random512-10-0.map.scen"
        ))
        .expect("random512 scenario should parse")
        .into_iter()
        .find(|entry| entry.bucket == 80)
        .expect("random512 MovingAI bucket 80 scenario should exist");

        let obstacles = map.to_obstacles();
        let planner = FlowFieldPlanner::from_obstacle_points(
            &obstacles,
            FlowFieldConfig {
                resolution: 1.0,
                robot_radius: 0.0,
            },
        )
        .expect("flow field planner should build from random512 obstacles");

        let start = map
            .planning_point(scenario.start_x, scenario.start_y)
            .expect("random512 start should map to planner coordinates");
        let goal = map
            .planning_point(scenario.goal_x, scenario.goal_y)
            .expect("random512 goal should map to planner coordinates");

        let path = planner
            .plan(start, goal)
            .expect("FlowField should solve the random512 bucket 80 scenario");

        assert!(
            (path.total_length() - scenario.optimal_length).abs() < 1e-6,
            "FlowField path length {} should match MovingAI optimal {}",
            path.total_length(),
            scenario.optimal_length
        );
    }
}
