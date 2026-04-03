//! Bidirectional Breadth-First Search path planning
//!
//! Grid-based path planning that searches from both start and goal
//! simultaneously, meeting in the middle.

use std::collections::HashMap;

use crate::grid::{GridMap, Node};
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Configuration for Bidirectional BFS planner
#[derive(Debug, Clone)]
pub struct BidirectionalBFSConfig {
    /// Grid resolution in meters
    pub resolution: f64,
    /// Robot radius for obstacle inflation
    pub robot_radius: f64,
}

impl Default for BidirectionalBFSConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
        }
    }
}

impl BidirectionalBFSConfig {
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

/// Bidirectional Breadth-First Search planner
pub struct BidirectionalBFSPlanner {
    grid_map: GridMap,
    #[allow(dead_code)]
    config: BidirectionalBFSConfig,
    motion: Vec<(i32, i32, f64)>,
}

impl BidirectionalBFSPlanner {
    pub fn new(ox: &[f64], oy: &[f64], config: BidirectionalBFSConfig) -> Self {
        Self::try_new(ox, oy, config).expect("invalid BidirectionalBFS planner input")
    }

    pub fn try_new(ox: &[f64], oy: &[f64], config: BidirectionalBFSConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.resolution, config.robot_radius)?;
        let motion = Self::get_motion_model();
        Ok(Self {
            grid_map,
            config,
            motion,
        })
    }

    pub fn from_obstacles(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> Self {
        let config = BidirectionalBFSConfig {
            resolution,
            robot_radius,
        };
        Self::new(ox, oy, config)
    }

    pub fn from_obstacle_points(
        obstacles: &Obstacles,
        config: BidirectionalBFSConfig,
    ) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::from_obstacles(obstacles, config.resolution, config.robot_radius)?;
        let motion = Self::get_motion_model();
        Ok(Self {
            grid_map,
            config,
            motion,
        })
    }

    pub fn planning(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> Option<(Vec<f64>, Vec<f64>)> {
        match self.plan_xy(sx, sy, gx, gy) {
            Ok(path) => Some((path.x_coords(), path.y_coords())),
            Err(_) => None,
        }
    }

    pub fn plan(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        self.plan_impl(start, goal)
    }

    pub fn plan_xy(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> RoboticsResult<Path2D> {
        self.plan_impl(Point2D::new(sx, sy), Point2D::new(gx, gy))
    }

    pub fn grid_map(&self) -> &GridMap {
        &self.grid_map
    }

    fn get_motion_model() -> Vec<(i32, i32, f64)> {
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

    fn build_half_path(&self, goal_index: usize, node_storage: &[Node]) -> Vec<Point2D> {
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
        points
    }

    fn best_open_key(&self, open_set: &HashMap<i32, usize>, node_storage: &[Node]) -> (i32, f64) {
        open_set
            .iter()
            .min_by(|a, b| {
                node_storage[*a.1]
                    .cost
                    .partial_cmp(&node_storage[*b.1].cost)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(k, idx)| (*k, node_storage[*idx].cost))
            .unwrap()
    }

    fn best_known_index(
        open_set: &HashMap<i32, usize>,
        closed_set: &HashMap<i32, usize>,
        node_storage: &[Node],
        grid_index: i32,
    ) -> Option<usize> {
        match (open_set.get(&grid_index), closed_set.get(&grid_index)) {
            (Some(&open_idx), Some(&closed_idx)) => {
                if node_storage[open_idx].cost <= node_storage[closed_idx].cost {
                    Some(open_idx)
                } else {
                    Some(closed_idx)
                }
            }
            (Some(&open_idx), None) => Some(open_idx),
            (None, Some(&closed_idx)) => Some(closed_idx),
            (None, None) => None,
        }
    }

    fn update_best_meeting(
        best_meeting: &mut Option<(usize, usize, f64)>,
        forward_index: usize,
        forward_cost: f64,
        backward_index: usize,
        backward_cost: f64,
    ) {
        let total_cost = forward_cost + backward_cost;
        if best_meeting
            .as_ref()
            .is_none_or(|(_, _, best_cost)| total_cost < *best_cost)
        {
            *best_meeting = Some((forward_index, backward_index, total_cost));
        }
    }

    fn build_meeting_path(
        &self,
        forward_index: usize,
        backward_index: usize,
        fwd_storage: &[Node],
        bwd_storage: &[Node],
    ) -> Path2D {
        let mut path = self.build_half_path(forward_index, fwd_storage);
        let mut bwd_path = self.build_half_path(backward_index, bwd_storage);
        bwd_path.reverse();
        if !bwd_path.is_empty() {
            path.extend_from_slice(&bwd_path[1..]);
        }
        Path2D::from_points(path)
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

    fn plan_impl(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        let start_x = self.grid_map.calc_x_index(start.x);
        let start_y = self.grid_map.calc_y_index(start.y);
        let goal_x = self.grid_map.calc_x_index(goal.x);
        let goal_y = self.grid_map.calc_y_index(goal.y);

        self.ensure_query_is_valid(start_x, start_y, "Start")?;
        self.ensure_query_is_valid(goal_x, goal_y, "Goal")?;

        // Forward search (from start)
        let mut fwd_open: HashMap<i32, usize> = HashMap::new();
        let mut fwd_closed: HashMap<i32, usize> = HashMap::new();
        let mut fwd_storage: Vec<Node> = Vec::new();

        fwd_storage.push(Node::new(start_x, start_y, 0.0, None));
        fwd_open.insert(self.grid_map.calc_index(start_x, start_y), 0);

        // Backward search (from goal)
        let mut bwd_open: HashMap<i32, usize> = HashMap::new();
        let mut bwd_closed: HashMap<i32, usize> = HashMap::new();
        let mut bwd_storage: Vec<Node> = Vec::new();

        bwd_storage.push(Node::new(goal_x, goal_y, 0.0, None));
        bwd_open.insert(self.grid_map.calc_index(goal_x, goal_y), 0);

        let mut best_meeting: Option<(usize, usize, f64)> = None;

        loop {
            if fwd_open.is_empty() || bwd_open.is_empty() {
                return best_meeting
                    .map(|(fwd_idx, bwd_idx, _)| {
                        self.build_meeting_path(fwd_idx, bwd_idx, &fwd_storage, &bwd_storage)
                    })
                    .ok_or_else(|| RoboticsError::PlanningError("No path found".to_string()));
            }

            let (fwd_key, min_cost_fwd) = self.best_open_key(&fwd_open, &fwd_storage);
            let (bwd_key, min_cost_bwd) = self.best_open_key(&bwd_open, &bwd_storage);

            if let Some((fwd_idx, bwd_idx, best_cost)) = best_meeting {
                if min_cost_fwd >= best_cost && min_cost_bwd >= best_cost {
                    return Ok(self.build_meeting_path(
                        fwd_idx,
                        bwd_idx,
                        &fwd_storage,
                        &bwd_storage,
                    ));
                }
            }

            if min_cost_fwd <= min_cost_bwd {
                let fwd_idx = fwd_open.remove(&fwd_key).unwrap();
                let fwd_node = &fwd_storage[fwd_idx];
                let fx = fwd_node.x;
                let fy = fwd_node.y;
                let fcost = fwd_node.cost;
                let fwd_grid_index = self.grid_map.calc_index(fx, fy);

                if let Some(&existing_closed_idx) = fwd_closed.get(&fwd_grid_index) {
                    if fwd_storage[existing_closed_idx].cost <= fcost {
                        continue;
                    }
                }

                if let Some(bwd_idx) =
                    Self::best_known_index(&bwd_open, &bwd_closed, &bwd_storage, fwd_grid_index)
                {
                    Self::update_best_meeting(
                        &mut best_meeting,
                        fwd_idx,
                        fcost,
                        bwd_idx,
                        bwd_storage[bwd_idx].cost,
                    );
                }

                fwd_closed.insert(fwd_grid_index, fwd_idx);

                for &(dx, dy, move_cost) in &self.motion {
                    let nx = fx + dx;
                    let ny = fy + dy;
                    let new_grid_index = self.grid_map.calc_index(nx, ny);

                    if !self.grid_map.is_valid_offset(fx, fy, dx, dy) {
                        continue;
                    }

                    if let Some(&closed_idx) = fwd_closed.get(&new_grid_index) {
                        if fwd_storage[closed_idx].cost <= fcost + move_cost {
                            continue;
                        }
                    }

                    if let Some(&open_idx) = fwd_open.get(&new_grid_index) {
                        if fwd_storage[open_idx].cost <= fcost + move_cost {
                            continue;
                        }
                    }

                    fwd_storage.push(Node::new(nx, ny, fcost + move_cost, Some(fwd_idx)));
                    let new_idx = fwd_storage.len() - 1;
                    fwd_open.insert(new_grid_index, new_idx);

                    if let Some(bwd_idx) =
                        Self::best_known_index(&bwd_open, &bwd_closed, &bwd_storage, new_grid_index)
                    {
                        Self::update_best_meeting(
                            &mut best_meeting,
                            new_idx,
                            fwd_storage[new_idx].cost,
                            bwd_idx,
                            bwd_storage[bwd_idx].cost,
                        );
                    }
                }
            } else {
                let bwd_idx = bwd_open.remove(&bwd_key).unwrap();
                let bwd_node = &bwd_storage[bwd_idx];
                let bx = bwd_node.x;
                let by = bwd_node.y;
                let bcost = bwd_node.cost;
                let bwd_grid_index = self.grid_map.calc_index(bx, by);

                if let Some(&existing_closed_idx) = bwd_closed.get(&bwd_grid_index) {
                    if bwd_storage[existing_closed_idx].cost <= bcost {
                        continue;
                    }
                }

                if let Some(fwd_idx) =
                    Self::best_known_index(&fwd_open, &fwd_closed, &fwd_storage, bwd_grid_index)
                {
                    Self::update_best_meeting(
                        &mut best_meeting,
                        fwd_idx,
                        fwd_storage[fwd_idx].cost,
                        bwd_idx,
                        bcost,
                    );
                }

                bwd_closed.insert(bwd_grid_index, bwd_idx);

                for &(dx, dy, move_cost) in &self.motion {
                    let nx = bx + dx;
                    let ny = by + dy;
                    let new_grid_index = self.grid_map.calc_index(nx, ny);

                    if !self.grid_map.is_valid_offset(bx, by, dx, dy) {
                        continue;
                    }

                    if let Some(&closed_idx) = bwd_closed.get(&new_grid_index) {
                        if bwd_storage[closed_idx].cost <= bcost + move_cost {
                            continue;
                        }
                    }

                    if let Some(&open_idx) = bwd_open.get(&new_grid_index) {
                        if bwd_storage[open_idx].cost <= bcost + move_cost {
                            continue;
                        }
                    }

                    bwd_storage.push(Node::new(nx, ny, bcost + move_cost, Some(bwd_idx)));
                    let new_idx = bwd_storage.len() - 1;
                    bwd_open.insert(new_grid_index, new_idx);

                    if let Some(fwd_idx) =
                        Self::best_known_index(&fwd_open, &fwd_closed, &fwd_storage, new_grid_index)
                    {
                        Self::update_best_meeting(
                            &mut best_meeting,
                            fwd_idx,
                            fwd_storage[fwd_idx].cost,
                            new_idx,
                            bwd_storage[new_idx].cost,
                        );
                    }
                }
            }
        }
    }
}

impl PathPlanner for BidirectionalBFSPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        self.plan_impl(start, goal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
    fn test_bidirectional_bfs_finds_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = BidirectionalBFSPlanner::from_obstacles(&ox, &oy, 0.5, 0.1);

        let result = planner.planning(2.0, 2.0, 8.0, 8.0);
        assert!(result.is_some());
        let (rx, ry) = result.unwrap();
        assert!(rx.len() > 2);
        assert_eq!(rx.len(), ry.len());
    }

    #[test]
    fn test_bidirectional_bfs_plan_returns_valid_path() {
        let (ox, oy) = create_simple_obstacles();
        let planner = BidirectionalBFSPlanner::from_obstacles(&ox, &oy, 0.5, 0.1);

        let path = planner.plan(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0));
        assert!(path.is_ok());
        let path = path.unwrap();
        assert!(path.len() > 2);
    }

    #[test]
    fn test_bidirectional_bfs_no_path() {
        let mut ox = Vec::new();
        let mut oy = Vec::new();

        // Boundary
        for i in -1..=61 {
            let v = i as f64;
            ox.push(v);
            oy.push(-1.0);
            ox.push(v);
            oy.push(61.0);
            ox.push(-1.0);
            oy.push(v);
            ox.push(61.0);
            oy.push(v);
        }

        // Complete dense wall at x=30
        for i in -1..=61 {
            for dx in -1..=1 {
                ox.push(30.0 + dx as f64);
                oy.push(i as f64);
            }
        }

        let planner = BidirectionalBFSPlanner::from_obstacles(&ox, &oy, 1.0, 0.0);
        let result = planner.plan(Point2D::new(10.0, 30.0), Point2D::new(50.0, 30.0));
        assert!(result.is_err());
    }

    #[test]
    fn test_bidirectional_bfs_from_obstacle_points() {
        let (ox, oy) = create_simple_obstacles();
        let obstacles = Obstacles::try_from_xy(&ox, &oy).expect("obstacle creation should succeed");
        let config = BidirectionalBFSConfig {
            resolution: 0.5,
            robot_radius: 0.1,
        };
        let planner = BidirectionalBFSPlanner::from_obstacle_points(&obstacles, config);
        assert!(planner.is_ok());
    }

    #[test]
    fn test_bidirectional_bfs_legacy_interface() {
        let (ox, oy) = create_simple_obstacles();
        let planner = BidirectionalBFSPlanner::from_obstacles(&ox, &oy, 0.5, 0.1);

        let result = planner.planning(2.0, 2.0, 8.0, 8.0);
        assert!(result.is_some());
    }

    #[test]
    fn test_bidirectional_bfs_matches_moving_ai_arena2_bucket_80_optimal_length() {
        let map = MovingAiMap::parse_str(include_str!("../benchdata/moving_ai/dao/arena2.map"))
            .expect("arena2 MovingAI map should parse");
        let scenario =
            MovingAiScenario::parse_str(include_str!("../benchdata/moving_ai/dao/arena2.map.scen"))
                .expect("arena2 MovingAI scenarios should parse")
                .into_iter()
                .find(|row| row.bucket == 80)
                .expect("arena2 MovingAI bucket 80 scenario should exist");

        let planner = BidirectionalBFSPlanner::from_obstacle_points(
            &map.to_obstacles(),
            BidirectionalBFSConfig {
                resolution: 1.0,
                robot_radius: 0.5,
            },
        )
        .expect("Bidirectional BFS planner should build from arena2 obstacles");

        let start = map
            .planning_point(scenario.start_x, scenario.start_y)
            .expect("arena2 start should be valid");
        let goal = map
            .planning_point(scenario.goal_x, scenario.goal_y)
            .expect("arena2 goal should be valid");

        let path = planner
            .plan(start, goal)
            .expect("Bidirectional BFS should solve the arena2 bucket 80 scenario");

        assert!(
            (path.total_length() - scenario.optimal_length).abs() < 1e-6,
            "Bidirectional BFS path length {} should match MovingAI optimal {}",
            path.total_length(),
            scenario.optimal_length
        );
    }
}
