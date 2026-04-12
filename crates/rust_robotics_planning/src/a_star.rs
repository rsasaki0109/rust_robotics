//! A* path planning algorithm
//!
//! Grid-based path planning using A* search algorithm with
//! configurable heuristic weight.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use crate::grid::{GridMap, Node};
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Configuration for A* planner
#[derive(Debug, Clone)]
pub struct AStarConfig {
    /// Grid resolution in meters
    pub resolution: f64,
    /// Robot radius for obstacle inflation
    pub robot_radius: f64,
    /// Heuristic weight (1.0 = optimal, >1.0 = faster but suboptimal)
    pub heuristic_weight: f64,
}

impl Default for AStarConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        }
    }
}

impl AStarConfig {
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

/// Node with priority for A* open set (min-heap)
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

/// A* path planner
pub struct AStarPlanner {
    grid_map: GridMap,
    config: AStarConfig,
    motion: Vec<(i32, i32, f64)>,
}

/// Search-effort statistics collected during one A* query.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AStarSearchStats {
    pub expanded_nodes: usize,
    pub generated_nodes: usize,
    pub skipped_closed_nodes: usize,
    pub max_frontier_len: usize,
}

impl AStarPlanner {
    /// Create a new A* planner with obstacle positions
    pub fn new(ox: &[f64], oy: &[f64], config: AStarConfig) -> Self {
        Self::try_new(ox, oy, config).expect(
            "invalid A* planner input: obstacle list must be non-empty and valid, and config values must be positive/finite",
        )
    }

    /// Create a validated A* planner with obstacle positions
    pub fn try_new(ox: &[f64], oy: &[f64], config: AStarConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.resolution, config.robot_radius)?;
        let motion = Self::get_motion_model();

        Ok(AStarPlanner {
            grid_map,
            config,
            motion,
        })
    }

    /// Create from obstacle x/y vectors with default config
    pub fn from_obstacles(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let config = AStarConfig {
            resolution,
            robot_radius,
            ..Default::default()
        };
        Self::new(ox, oy, config)
    }

    /// Create a validated A* planner from typed obstacle points
    pub fn from_obstacle_points(
        obstacles: &Obstacles,
        config: AStarConfig,
    ) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::from_obstacles(obstacles, config.resolution, config.robot_radius)?;
        let motion = Self::get_motion_model();

        Ok(AStarPlanner {
            grid_map,
            config,
            motion,
        })
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
        self.plan_impl(start, goal).map(|(path, _stats)| path)
    }

    /// Plan a path from raw coordinates without requiring the PathPlanner trait in scope
    pub fn plan_xy(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> RoboticsResult<Path2D> {
        self.plan_impl(Point2D::new(sx, sy), Point2D::new(gx, gy))
            .map(|(path, _stats)| path)
    }

    /// Plan a path and return per-query search-effort statistics.
    pub fn plan_with_stats(
        &self,
        start: Point2D,
        goal: Point2D,
    ) -> RoboticsResult<(Path2D, AStarSearchStats)> {
        self.plan_impl(start, goal)
    }

    /// Get reference to the grid map
    pub fn grid_map(&self) -> &GridMap {
        &self.grid_map
    }

    fn calc_heuristic(&self, n1_x: i32, n1_y: i32, n2_x: i32, n2_y: i32) -> f64 {
        self.config.heuristic_weight * (((n1_x - n2_x).pow(2) + (n1_y - n2_y).pow(2)) as f64).sqrt()
    }

    fn get_motion_model() -> Vec<(i32, i32, f64)> {
        // dx, dy, cost (8-connected grid)
        vec![
            (1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (-1, -1, std::f64::consts::SQRT_2),
            (-1, 1, std::f64::consts::SQRT_2),
            (1, -1, std::f64::consts::SQRT_2),
            (1, 1, std::f64::consts::SQRT_2),
        ]
    }

    fn build_path(&self, goal_index: usize, node_storage: &[Node]) -> Path2D {
        let mut points = Vec::new();
        let mut current_index = Some(goal_index);

        while let Some(index) = current_index {
            let node = &node_storage[index];
            points.push(Point2D::new(
                self.grid_map.calc_x_position(node.x),
                self.grid_map.calc_y_position(node.y),
            ));
            current_index = node.parent_index;
        }

        points.reverse();
        Path2D::from_points(points)
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

    fn plan_impl(
        &self,
        start: Point2D,
        goal: Point2D,
    ) -> RoboticsResult<(Path2D, AStarSearchStats)> {
        let start_x = self.grid_map.calc_x_index(start.x);
        let start_y = self.grid_map.calc_y_index(start.y);
        let goal_x = self.grid_map.calc_x_index(goal.x);
        let goal_y = self.grid_map.calc_y_index(goal.y);

        self.ensure_query_is_valid(start_x, start_y, "Start")?;
        self.ensure_query_is_valid(goal_x, goal_y, "Goal")?;

        let mut open_set = BinaryHeap::new();
        let mut closed_set = HashMap::new();
        let mut node_storage: Vec<Node> = Vec::new();
        let mut stats = AStarSearchStats::default();

        // Add start node
        node_storage.push(Node::new(start_x, start_y, 0.0, None));
        let start_index = 0;

        open_set.push(PriorityNode {
            x: start_x,
            y: start_y,
            cost: 0.0,
            priority: self.calc_heuristic(start_x, start_y, goal_x, goal_y),
            index: start_index,
        });
        stats.max_frontier_len = open_set.len();

        while let Some(current) = open_set.pop() {
            let current_grid_index = self.grid_map.calc_index(current.x, current.y);

            // Skip if already in closed set
            if closed_set.contains_key(&current_grid_index) {
                stats.skipped_closed_nodes += 1;
                continue;
            }

            stats.expanded_nodes += 1;

            // Check if we reached the goal
            if current.x == goal_x && current.y == goal_y {
                return Ok((self.build_path(current.index, &node_storage), stats));
            }

            // Move to closed set
            closed_set.insert(current_grid_index, current.index);

            // Expand neighbors
            for &(dx, dy, move_cost) in &self.motion {
                let new_x = current.x + dx;
                let new_y = current.y + dy;
                let new_cost = current.cost + move_cost;
                let new_grid_index = self.grid_map.calc_index(new_x, new_y);

                // Skip if not valid or already visited
                if !self.grid_map.is_valid_offset(current.x, current.y, dx, dy) {
                    continue;
                }
                if closed_set.contains_key(&new_grid_index) {
                    continue;
                }

                // Add to storage and open set
                node_storage.push(Node::new(new_x, new_y, new_cost, Some(current.index)));
                let new_index = node_storage.len() - 1;
                stats.generated_nodes += 1;

                let priority = new_cost + self.calc_heuristic(new_x, new_y, goal_x, goal_y);
                open_set.push(PriorityNode {
                    x: new_x,
                    y: new_y,
                    cost: new_cost,
                    priority,
                    index: new_index,
                });
                stats.max_frontier_len = stats.max_frontier_len.max(open_set.len());
            }
        }

        Err(RoboticsError::PlanningError("No path found".to_string()))
    }
}

impl PathPlanner for AStarPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        self.plan_impl(start, goal).map(|(path, _stats)| path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::moving_ai::{MovingAiMap, MovingAiScenario};

    use rust_robotics_core::Obstacles;

    fn create_simple_obstacles() -> (Vec<f64>, Vec<f64>) {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        // Boundary
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

        // Internal obstacle
        for i in 4..7 {
            ox.push(5.0);
            oy.push(i as f64);
        }

        (ox, oy)
    }

    #[test]
    fn test_a_star_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = AStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(8.0, 8.0);

        let result = planner.plan(start, goal);
        assert!(result.is_ok());

        let path = result.unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    #[allow(deprecated)]
    fn test_a_star_legacy_interface() {
        let (ox, oy) = create_simple_obstacles();
        let planner = AStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let result = planner.planning(2.0, 2.0, 8.0, 8.0);
        assert!(result.is_some());

        let (rx, ry) = result.unwrap();
        assert!(!rx.is_empty());
        assert_eq!(rx.len(), ry.len());
    }

    #[test]
    fn test_a_star_from_obstacle_points() {
        let (ox, oy) = create_simple_obstacles();
        let obstacles = Obstacles::try_from_xy(&ox, &oy).unwrap();
        let planner =
            AStarPlanner::from_obstacle_points(&obstacles, AStarConfig::default()).unwrap();

        let path = planner.plan_xy(2.0, 2.0, 8.0, 8.0).unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_a_star_try_new_rejects_invalid_config() {
        let (ox, oy) = create_simple_obstacles();
        let config = AStarConfig {
            heuristic_weight: 0.0,
            ..Default::default()
        };

        let err = match AStarPlanner::try_new(&ox, &oy, config) {
            Ok(_) => panic!("expected invalid config to be rejected"),
            Err(err) => err,
        };
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_a_star_preserves_asymmetric_world_coordinates() {
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
            AStarPlanner::from_obstacle_points(&obstacles, AStarConfig::default()).unwrap();
        let path = planner.plan_xy(12.0, -2.0, 18.0, 4.0).unwrap();

        let first = path.points.first().unwrap();
        let last = path.points.last().unwrap();
        assert!((first.x - 12.0).abs() < 1e-6);
        assert!((first.y + 2.0).abs() < 1e-6);
        assert!((last.x - 18.0).abs() < 1e-6);
        assert!((last.y - 4.0).abs() < 1e-6);
    }

    #[test]
    #[ignore = "long-running MovingAI benchmark"]
    fn test_a_star_matches_moving_ai_arena2_bucket_80_optimal_length() {
        let map = MovingAiMap::parse_str(include_str!("../benchdata/moving_ai/dao/arena2.map"))
            .expect("arena2 MovingAI map should parse");
        let scenario =
            MovingAiScenario::parse_str(include_str!("../benchdata/moving_ai/dao/arena2.map.scen"))
                .expect("arena2 MovingAI scenarios should parse")
                .into_iter()
                .find(|row| row.bucket == 80)
                .expect("arena2 MovingAI bucket 80 scenario should exist");

        let planner = AStarPlanner::from_obstacle_points(
            &map.to_obstacles(),
            AStarConfig {
                resolution: 1.0,
                robot_radius: 0.5,
                heuristic_weight: 1.0,
            },
        )
        .expect("A* planner should build from arena2 obstacles");

        let start = map
            .planning_point(scenario.start_x, scenario.start_y)
            .expect("arena2 start should be valid");
        let goal = map
            .planning_point(scenario.goal_x, scenario.goal_y)
            .expect("arena2 goal should be valid");

        let path = planner
            .plan(start, goal)
            .expect("A* should solve the arena2 bucket 80 scenario");

        assert!(
            (path.total_length() - scenario.optimal_length).abs() < 1e-6,
            "A* path length {} should match MovingAI optimal {} when corner cutting is disabled",
            path.total_length(),
            scenario.optimal_length
        );
    }
}
