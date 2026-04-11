//! PythonRobotics-style A* variants.
//!
//! This module ports the missing variants from
//! `PathPlanning/AStar/a_star_variants.py`:
//! beam search, iterative deepening, dynamic weighting, the file-local
//! theta-star-like farthest-step search, and the corner-graph jump-point variant.

use std::collections::HashMap;

use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

type GridCoord = (i32, i32);

/// Variant selection for the A* search loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AStarVariantMode {
    Standard,
    BeamSearch,
    IterativeDeepening,
    DynamicWeighting,
    ThetaStarLike,
    JumpPointCorners,
}

/// Configuration for PythonRobotics-style A* variants.
#[derive(Debug, Clone)]
pub struct AStarVariantConfig {
    pub resolution: f64,
    pub robot_radius: f64,
    pub mode: AStarVariantMode,
    pub beam_capacity: usize,
    pub epsilon: f64,
    pub upper_bound_depth: usize,
    pub max_theta: usize,
    pub only_corners: bool,
    pub max_corner: f64,
}

impl Default for AStarVariantConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.0,
            mode: AStarVariantMode::Standard,
            beam_capacity: 30,
            epsilon: 4.0,
            upper_bound_depth: 500,
            max_theta: 5,
            only_corners: false,
            max_corner: 5.0,
        }
    }
}

impl AStarVariantConfig {
    fn validate(&self) -> RoboticsResult<()> {
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
        if self.beam_capacity == 0 {
            return Err(RoboticsError::InvalidParameter(
                "beam_capacity must be greater than zero".to_string(),
            ));
        }
        if !self.epsilon.is_finite() || self.epsilon < 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "epsilon must be non-negative and finite, got {}",
                self.epsilon
            )));
        }
        if self.upper_bound_depth == 0 {
            return Err(RoboticsError::InvalidParameter(
                "upper_bound_depth must be greater than zero".to_string(),
            ));
        }
        if self.max_theta == 0 {
            return Err(RoboticsError::InvalidParameter(
                "max_theta must be greater than zero".to_string(),
            ));
        }
        if !self.max_corner.is_finite() || self.max_corner <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "max_corner must be positive and finite, got {}",
                self.max_corner
            )));
        }

        Ok(())
    }
}

#[derive(Debug, Clone)]
struct VariantGrid {
    resolution: f64,
    min_x: f64,
    min_y: f64,
    x_width: i32,
    y_width: i32,
    obstacle_map: Vec<Vec<bool>>,
}

impl VariantGrid {
    fn try_new(ox: &[f64], oy: &[f64], resolution: f64, robot_radius: f64) -> RoboticsResult<Self> {
        if ox.len() != oy.len() {
            return Err(RoboticsError::InvalidParameter(format!(
                "obstacle x/y coordinates must have matching lengths, got {} and {}",
                ox.len(),
                oy.len()
            )));
        }
        if ox.is_empty() {
            return Err(RoboticsError::InvalidParameter(
                "A* variants require at least one obstacle point".to_string(),
            ));
        }
        if ox.iter().chain(oy.iter()).any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "obstacle coordinates must be finite".to_string(),
            ));
        }

        let min_x = ox
            .iter()
            .fold(f64::INFINITY, |acc, &value| acc.min(value))
            .round();
        let min_y = oy
            .iter()
            .fold(f64::INFINITY, |acc, &value| acc.min(value))
            .round();
        let max_x = ox
            .iter()
            .fold(f64::NEG_INFINITY, |acc, &value| acc.max(value))
            .round();
        let max_y = oy
            .iter()
            .fold(f64::NEG_INFINITY, |acc, &value| acc.max(value))
            .round();
        let x_width = ((max_x - min_x) / resolution).round() as i32 + 1;
        let y_width = ((max_y - min_y) / resolution).round() as i32 + 1;

        let mut obstacle_map = vec![vec![false; y_width as usize]; x_width as usize];
        for ix in 0..x_width {
            let x = min_x + ix as f64 * resolution;
            for iy in 0..y_width {
                let y = min_y + iy as f64 * resolution;
                if ox
                    .iter()
                    .zip(oy.iter())
                    .any(|(&obs_x, &obs_y)| (obs_x - x).hypot(obs_y - y) <= robot_radius)
                {
                    obstacle_map[ix as usize][iy as usize] = true;
                }
            }
        }

        Ok(Self {
            resolution,
            min_x,
            min_y,
            x_width,
            y_width,
            obstacle_map,
        })
    }

    fn calc_x_index(&self, x: f64) -> i32 {
        ((x - self.min_x) / self.resolution).round() as i32
    }

    fn calc_y_index(&self, y: f64) -> i32 {
        ((y - self.min_y) / self.resolution).round() as i32
    }

    fn calc_x_position(&self, ix: i32) -> f64 {
        self.min_x + ix as f64 * self.resolution
    }

    fn calc_y_position(&self, iy: i32) -> f64 {
        self.min_y + iy as f64 * self.resolution
    }

    fn is_valid(&self, x: i32, y: i32) -> bool {
        x >= 0
            && x < self.x_width
            && y >= 0
            && y < self.y_width
            && !self.obstacle_map[x as usize][y as usize]
    }

    fn contains(&self, x: i32, y: i32) -> bool {
        x >= 0 && x < self.x_width && y >= 0 && y < self.y_width
    }

    fn is_obstacle_or_outside(&self, x: i32, y: i32) -> bool {
        !self.contains(x, y) || self.obstacle_map[x as usize][y as usize]
    }
}

#[derive(Debug, Clone)]
struct SearchNode {
    pred: Option<GridCoord>,
    gcost: f64,
    hcost: f64,
    fcost: f64,
    open: bool,
    in_open_list: bool,
}

impl SearchNode {
    fn new(hcost: f64) -> Self {
        Self {
            pred: None,
            gcost: f64::INFINITY,
            hcost,
            fcost: f64::INFINITY,
            open: true,
            in_open_list: false,
        }
    }
}

struct CostUpdate<'a> {
    current_threshold: f64,
    current: GridCoord,
    offset: f64,
    weight: Option<f64>,
    f_cost_list: &'a mut Vec<f64>,
    all_nodes: &'a mut HashMap<GridCoord, SearchNode>,
    open_set: &'a mut Vec<GridCoord>,
}

/// Planner implementing the missing PythonRobotics A* variants.
#[derive(Debug)]
pub struct AStarVariantPlanner {
    grid: VariantGrid,
    config: AStarVariantConfig,
}

impl AStarVariantPlanner {
    pub fn new(ox: &[f64], oy: &[f64], config: AStarVariantConfig) -> Self {
        Self::try_new(ox, oy, config).expect(
            "invalid A* variant input: obstacle list must be non-empty and valid, and config values must be positive/finite",
        )
    }

    pub fn try_new(ox: &[f64], oy: &[f64], config: AStarVariantConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid = VariantGrid::try_new(ox, oy, config.resolution, config.robot_radius)?;
        Ok(Self { grid, config })
    }

    pub fn from_obstacles(
        ox: &[f64],
        oy: &[f64],
        resolution: f64,
        robot_radius: f64,
        mode: AStarVariantMode,
    ) -> Self {
        let config = AStarVariantConfig {
            resolution,
            robot_radius,
            mode,
            ..Default::default()
        };
        Self::new(ox, oy, config)
    }

    pub fn from_obstacle_points(
        obstacles: &Obstacles,
        config: AStarVariantConfig,
    ) -> RoboticsResult<Self> {
        Self::try_new(&obstacles.x_coords(), &obstacles.y_coords(), config)
    }

    pub fn plan(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        self.plan_impl(start, goal)
    }

    pub fn plan_xy(&self, sx: f64, sy: f64, gx: f64, gy: f64) -> RoboticsResult<Path2D> {
        self.plan_impl(Point2D::new(sx, sy), Point2D::new(gx, gy))
    }

    fn heuristic(start: GridCoord, goal: GridCoord) -> f64 {
        let dx = (start.0 - goal.0).abs() as f64;
        let dy = (start.1 - goal.1).abs() as f64;
        14.0 * dx.min(dy) + 10.0 * (dx.max(dy) - dx.min(dy))
    }

    fn motion_model() -> [(i32, i32, f64); 8] {
        [
            (-1, -1, 14.0),
            (-1, 0, 10.0),
            (-1, 1, 14.0),
            (0, -1, 10.0),
            (0, 1, 10.0),
            (1, -1, 14.0),
            (1, 0, 10.0),
            (1, 1, 14.0),
        ]
    }

    fn build_node_table(&self, goal: GridCoord) -> HashMap<GridCoord, SearchNode> {
        let mut nodes = HashMap::new();
        for x in 0..self.grid.x_width {
            for y in 0..self.grid.y_width {
                if self.grid.is_valid(x, y) {
                    nodes.insert((x, y), SearchNode::new(Self::heuristic((x, y), goal)));
                }
            }
        }
        nodes
    }

    fn line_of_sight(&self, from: GridCoord, to: GridCoord) -> Option<f64> {
        let mut t = 0.0_f64;
        while t <= 0.5 {
            let forward_x = ((1.0 - t) * from.0 as f64 + t * to.0 as f64) as i32;
            let forward_y = ((1.0 - t) * from.1 as f64 + t * to.1 as f64) as i32;
            if self.grid.is_obstacle_or_outside(forward_x, forward_y) {
                return None;
            }

            let reverse_x = ((1.0 - t) * to.0 as f64 + t * from.0 as f64) as i32;
            let reverse_y = ((1.0 - t) * to.1 as f64 + t * from.1 as f64) as i32;
            if self.grid.is_obstacle_or_outside(reverse_x, reverse_y) {
                return None;
            }

            t += 0.001;
        }

        Some(((from.0 - to.0) as f64).hypot((from.1 - to.1) as f64))
    }

    fn key_points(&self) -> Vec<GridCoord> {
        let offsets1 = [(1, 0), (0, 1), (-1, 0), (1, 0)];
        let offsets2 = [(1, 1), (-1, 1), (-1, -1), (1, -1)];
        let offsets3 = [(0, 1), (-1, 0), (0, -1), (0, -1)];

        let mut corners = Vec::new();
        for x in 0..self.grid.x_width {
            for y in 0..self.grid.y_width {
                if self.grid.is_obstacle_or_outside(x, y) {
                    continue;
                }

                let mut empty_space = true;
                'neighbors: for dx in -1..=1 {
                    for dy in -1..=1 {
                        let nx = x + dx;
                        let ny = y + dy;
                        if !self.grid.contains(nx, ny) {
                            continue;
                        }
                        if self.grid.is_obstacle_or_outside(nx, ny) {
                            empty_space = false;
                            break 'neighbors;
                        }
                    }
                }
                if empty_space {
                    continue;
                }

                for ((i1, j1), ((i2, j2), (i3, j3))) in offsets1
                    .iter()
                    .copied()
                    .zip(offsets2.iter().copied().zip(offsets3.iter().copied()))
                {
                    let n1 = (x + i1, y + j1);
                    let n2 = (x + i2, y + j2);
                    let n3 = (x + i3, y + j3);
                    if !self.grid.contains(n1.0, n1.1)
                        || !self.grid.contains(n2.0, n2.1)
                        || !self.grid.contains(n3.0, n3.1)
                    {
                        continue;
                    }

                    let mut obstacle_count = 0;
                    if self.grid.is_obstacle_or_outside(n1.0, n1.1) {
                        obstacle_count += 1;
                    }
                    if self.grid.is_obstacle_or_outside(n2.0, n2.1) {
                        obstacle_count += 1;
                    }
                    if self.grid.is_obstacle_or_outside(n3.0, n3.1) {
                        obstacle_count += 1;
                    }
                    if obstacle_count == 1 || obstacle_count == 3 {
                        corners.push((x, y));
                        break;
                    }
                }
            }
        }

        if self.config.only_corners {
            return corners;
        }

        let mut key_points = corners.clone();
        for &(x1, y1) in &corners {
            for &(x2, y2) in &corners {
                if x1 == x2 && y1 == y2 {
                    continue;
                }
                if self.line_of_sight((x1, y1), (x2, y2)).is_none() {
                    continue;
                }
                key_points.push(((x1 + x2) / 2, (y1 + y2) / 2));
            }
        }

        key_points
    }

    fn get_farthest_point(
        &self,
        x: i32,
        y: i32,
        dx: i32,
        dy: i32,
        goal: GridCoord,
    ) -> (GridCoord, usize, bool) {
        let mut step_x = dx;
        let mut step_y = dy;
        let mut counter = 1usize;
        let mut got_goal = false;

        while !self.grid.is_obstacle_or_outside(x + step_x, y + step_y)
            && counter < self.config.max_theta
        {
            step_x += dx;
            step_y += dy;
            counter += 1;

            if (x + step_x, y + step_y) == goal {
                got_goal = true;
                break;
            }
            if !self.grid.contains(x + step_x, y + step_y) {
                break;
            }
        }

        (
            (x + step_x - 2 * dx, y + step_y - 2 * dy),
            counter,
            got_goal,
        )
    }

    fn build_corner_node_table(
        &self,
        start: GridCoord,
        goal: GridCoord,
    ) -> HashMap<GridCoord, SearchNode> {
        let mut nodes = HashMap::new();
        for point in self.key_points() {
            if self.grid.is_valid(point.0, point.1) {
                nodes
                    .entry(point)
                    .or_insert_with(|| SearchNode::new(Self::heuristic(point, goal)));
            }
        }
        nodes.insert(goal, SearchNode::new(0.0));
        nodes.insert(start, SearchNode::new(Self::heuristic(start, goal)));
        nodes
    }

    fn build_path(
        &self,
        all_nodes: &HashMap<GridCoord, SearchNode>,
        goal: GridCoord,
    ) -> RoboticsResult<Path2D> {
        let mut points = Vec::new();
        let mut current = goal;
        loop {
            points.push(Point2D::new(
                self.grid.calc_x_position(current.0),
                self.grid.calc_y_position(current.1),
            ));
            let node = all_nodes.get(&current).ok_or_else(|| {
                RoboticsError::PlanningError(
                    "goal node missing during path reconstruction".to_string(),
                )
            })?;
            match node.pred {
                Some(pred) => current = pred,
                None => break,
            }
        }
        points.reverse();
        Ok(Path2D::from_points(points))
    }

    fn update_node_cost(
        &self,
        candidate: GridCoord,
        mut no_valid_f: bool,
        update: &mut CostUpdate<'_>,
    ) -> bool {
        let current_cost = update.all_nodes[&update.current].gcost;
        let candidate_node = update
            .all_nodes
            .get_mut(&candidate)
            .expect("candidate node should be present for valid cell");
        if !candidate_node.open {
            return no_valid_f;
        }

        let g_cost = update.offset + current_cost;
        let mut h_cost = candidate_node.hcost;
        if let Some(weight) = update.weight {
            h_cost *= weight;
        }
        let f_cost = g_cost + h_cost;

        if f_cost < candidate_node.fcost && f_cost <= update.current_threshold {
            update.f_cost_list.push(f_cost);
            candidate_node.pred = Some(update.current);
            candidate_node.gcost = g_cost;
            candidate_node.fcost = f_cost;
            if !candidate_node.in_open_list {
                update.open_set.push(candidate);
                candidate_node.in_open_list = true;
            }
        }

        if update.current_threshold < f_cost && f_cost < candidate_node.fcost {
            no_valid_f = true;
        }

        no_valid_f
    }

    fn plan_jump_point_corners(
        &self,
        start_coord: GridCoord,
        goal_coord: GridCoord,
    ) -> RoboticsResult<Path2D> {
        let mut all_nodes = self.build_corner_node_table(start_coord, goal_coord);
        let start_node = all_nodes
            .get_mut(&start_coord)
            .expect("start node should exist in jump-point corner graph");
        start_node.gcost = 0.0;
        start_node.fcost = start_node.hcost;
        start_node.in_open_list = true;

        let mut open_set = vec![start_coord];
        while !open_set.is_empty() {
            open_set
                .sort_by(|left, right| all_nodes[left].fcost.total_cmp(&all_nodes[right].fcost));

            let mut chosen_index = 0usize;
            let lowest_f = all_nodes[&open_set[0]].fcost;
            let mut lowest_h = all_nodes[&open_set[0]].hcost;
            let mut lowest_g = all_nodes[&open_set[0]].gcost;
            for candidate in open_set.iter().skip(1) {
                let node = &all_nodes[candidate];
                if node.fcost == lowest_f && node.gcost < lowest_g {
                    lowest_g = node.gcost;
                    chosen_index += 1;
                } else if node.fcost == lowest_f && node.gcost == lowest_g && node.hcost < lowest_h
                {
                    lowest_h = node.hcost;
                    chosen_index += 1;
                } else {
                    break;
                }
            }

            let current = open_set[chosen_index];
            let candidate_points: Vec<_> = all_nodes.keys().copied().collect();
            for candidate in candidate_points {
                if candidate == current {
                    continue;
                }
                if ((current.0 - candidate.0) as f64).hypot((current.1 - candidate.1) as f64)
                    > self.config.max_corner
                {
                    continue;
                }
                let Some(offset) = self.line_of_sight(current, candidate) else {
                    continue;
                };

                if candidate == goal_coord {
                    all_nodes.get_mut(&goal_coord).unwrap().pred = Some(current);
                    return self.build_path(&all_nodes, goal_coord);
                }

                let current_cost = all_nodes[&current].gcost;
                let candidate_node = all_nodes.get_mut(&candidate).unwrap();
                if !candidate_node.open {
                    continue;
                }

                let g_cost = current_cost + offset;
                let f_cost = g_cost + candidate_node.hcost;
                if f_cost < candidate_node.fcost {
                    candidate_node.pred = Some(current);
                    candidate_node.gcost = g_cost;
                    candidate_node.fcost = f_cost;
                    if !candidate_node.in_open_list {
                        open_set.push(candidate);
                        candidate_node.in_open_list = true;
                    }
                }
            }

            {
                let current_node = all_nodes.get_mut(&current).unwrap();
                current_node.open = false;
                current_node.in_open_list = false;
                current_node.fcost = f64::INFINITY;
                current_node.hcost = f64::INFINITY;
            }
            open_set.remove(chosen_index);
        }

        Err(RoboticsError::PlanningError("no path found".to_string()))
    }

    fn plan_grid_variant(
        &self,
        start_coord: GridCoord,
        goal_coord: GridCoord,
    ) -> RoboticsResult<Path2D> {
        let mut all_nodes = self.build_node_table(goal_coord);
        let start_node = all_nodes
            .get_mut(&start_coord)
            .expect("start node should exist in valid grid");
        start_node.gcost = 0.0;
        start_node.fcost = start_node.hcost;
        start_node.in_open_list = true;

        let mut open_set = vec![start_coord];
        let mut goal_found = false;
        let mut current_threshold = f64::INFINITY;
        let mut depth = 0usize;
        let mut no_valid_f = false;

        while !open_set.is_empty() {
            open_set
                .sort_by(|left, right| all_nodes[left].fcost.total_cmp(&all_nodes[right].fcost));

            let mut chosen_index = 0usize;
            let lowest_f = all_nodes[&open_set[0]].fcost;
            let mut lowest_h = all_nodes[&open_set[0]].hcost;
            let mut lowest_g = all_nodes[&open_set[0]].gcost;
            for candidate in open_set.iter().skip(1) {
                let node = &all_nodes[candidate];
                if node.fcost == lowest_f && node.gcost < lowest_g {
                    lowest_g = node.gcost;
                    chosen_index += 1;
                } else if node.fcost == lowest_f && node.gcost == lowest_g && node.hcost < lowest_h
                {
                    lowest_h = node.hcost;
                    chosen_index += 1;
                } else {
                    break;
                }
            }

            if matches!(self.config.mode, AStarVariantMode::BeamSearch) {
                while open_set.len() > self.config.beam_capacity {
                    open_set.pop();
                }
            }

            let current = open_set[chosen_index];
            let mut f_cost_list = Vec::new();
            let weight = if matches!(self.config.mode, AStarVariantMode::DynamicWeighting) {
                Some(
                    1.0 + self.config.epsilon
                        - self.config.epsilon * depth as f64 / self.config.upper_bound_depth as f64,
                )
            } else {
                None
            };

            for (dx, dy, offset) in Self::motion_model() {
                let (candidate, offset, reached_goal) =
                    if matches!(self.config.mode, AStarVariantMode::ThetaStarLike) {
                        let (candidate, multiplier, reached_goal) =
                            self.get_farthest_point(current.0, current.1, dx, dy, goal_coord);
                        (candidate, offset * multiplier as f64, reached_goal)
                    } else {
                        ((current.0 + dx, current.1 + dy), offset, false)
                    };

                if reached_goal {
                    all_nodes.get_mut(&goal_coord).unwrap().pred = Some(current);
                    goal_found = true;
                    break;
                }
                if !all_nodes.contains_key(&candidate) {
                    continue;
                }

                if candidate == goal_coord {
                    all_nodes.get_mut(&goal_coord).unwrap().pred = Some(current);
                    goal_found = true;
                    break;
                }

                let mut update = CostUpdate {
                    current_threshold,
                    current,
                    offset,
                    weight,
                    f_cost_list: &mut f_cost_list,
                    all_nodes: &mut all_nodes,
                    open_set: &mut open_set,
                };
                no_valid_f = self.update_node_cost(candidate, no_valid_f, &mut update);
            }

            if goal_found {
                return self.build_path(&all_nodes, goal_coord);
            }

            if matches!(self.config.mode, AStarVariantMode::IterativeDeepening) {
                if let Some(next_threshold) = f_cost_list
                    .iter()
                    .copied()
                    .min_by(|left, right| left.total_cmp(right))
                {
                    current_threshold = next_threshold;
                } else {
                    current_threshold = f64::INFINITY;
                }

                if f_cost_list.is_empty() && no_valid_f {
                    let current_node = all_nodes.get_mut(&current).unwrap();
                    current_node.fcost = f64::INFINITY;
                    current_node.hcost = f64::INFINITY;
                    continue;
                }
            }

            {
                let current_node = all_nodes.get_mut(&current).unwrap();
                current_node.open = false;
                current_node.in_open_list = false;
                current_node.fcost = f64::INFINITY;
                current_node.hcost = f64::INFINITY;
            }
            open_set.remove(chosen_index);
            depth += 1;
        }

        Err(RoboticsError::PlanningError("no path found".to_string()))
    }

    fn plan_impl(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        let start_coord = (
            self.grid.calc_x_index(start.x),
            self.grid.calc_y_index(start.y),
        );
        let goal_coord = (
            self.grid.calc_x_index(goal.x),
            self.grid.calc_y_index(goal.y),
        );
        if !self.grid.is_valid(start_coord.0, start_coord.1) {
            return Err(RoboticsError::PlanningError(
                "start position is invalid".to_string(),
            ));
        }
        if !self.grid.is_valid(goal_coord.0, goal_coord.1) {
            return Err(RoboticsError::PlanningError(
                "goal position is invalid".to_string(),
            ));
        }

        if matches!(self.config.mode, AStarVariantMode::JumpPointCorners) {
            self.plan_jump_point_corners(start_coord, goal_coord)
        } else {
            self.plan_grid_variant(start_coord, goal_coord)
        }
    }
}

impl PathPlanner for AStarVariantPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        self.plan_impl(start, goal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn draw_horizontal_line(
        start_x: i32,
        start_y: i32,
        length: i32,
        obstacle_x: &mut Vec<f64>,
        obstacle_y: &mut Vec<f64>,
    ) {
        for x in start_x..start_x + length {
            for y in start_y..start_y + 2 {
                obstacle_x.push(x as f64);
                obstacle_y.push(y as f64);
            }
        }
    }

    fn draw_vertical_line(
        start_x: i32,
        start_y: i32,
        length: i32,
        obstacle_x: &mut Vec<f64>,
        obstacle_y: &mut Vec<f64>,
    ) {
        for x in start_x..start_x + 2 {
            for y in start_y..start_y + length {
                obstacle_x.push(x as f64);
                obstacle_y.push(y as f64);
            }
        }
    }

    fn build_pythonrobotics_maze() -> (Vec<f64>, Vec<f64>) {
        let mut obstacle_x = Vec::new();
        let mut obstacle_y = Vec::new();

        draw_vertical_line(0, 0, 50, &mut obstacle_x, &mut obstacle_y);
        draw_vertical_line(48, 0, 50, &mut obstacle_x, &mut obstacle_y);
        draw_horizontal_line(0, 0, 50, &mut obstacle_x, &mut obstacle_y);
        draw_horizontal_line(0, 48, 50, &mut obstacle_x, &mut obstacle_y);

        let all_x = [10, 10, 10, 15, 20, 20, 30, 30, 35, 30, 40, 45];
        let all_y = [10, 30, 45, 20, 5, 40, 10, 40, 5, 40, 10, 25];
        let all_len = [10, 10, 5, 10, 10, 5, 20, 10, 25, 10, 35, 15];
        for ((x, y), length) in all_x.iter().zip(all_y.iter()).zip(all_len.iter()) {
            draw_vertical_line(*x, *y, *length, &mut obstacle_x, &mut obstacle_y);
        }

        let all_x = [35, 40, 15, 10, 45, 20, 10, 15, 25, 45, 10, 30, 10, 40];
        let all_y = [5, 10, 15, 20, 20, 25, 30, 35, 35, 35, 40, 40, 45, 45];
        let all_len = [10, 5, 10, 10, 5, 5, 10, 5, 10, 5, 10, 5, 5, 5];
        for ((x, y), length) in all_x.iter().zip(all_y.iter()).zip(all_len.iter()) {
            draw_horizontal_line(*x, *y, *length, &mut obstacle_x, &mut obstacle_y);
        }

        (obstacle_x, obstacle_y)
    }

    fn parse_fixture(csv: &str) -> Path2D {
        let points = csv
            .lines()
            .skip(1)
            .filter(|line| !line.trim().is_empty())
            .map(|line| {
                let (x, y) = line
                    .split_once(',')
                    .expect("fixture rows must contain a comma");
                Point2D::new(x.parse().unwrap(), y.parse().unwrap())
            })
            .collect();
        Path2D::from_points(points)
    }

    fn assert_paths_match(actual: &Path2D, expected: &Path2D) {
        assert_eq!(actual.len(), expected.len());
        for (actual_point, expected_point) in actual.points.iter().zip(expected.points.iter()) {
            assert!(
                (actual_point.x - expected_point.x).abs() < 1.0e-12
                    && (actual_point.y - expected_point.y).abs() < 1.0e-12,
                "expected ({}, {}), got ({}, {})",
                expected_point.x,
                expected_point.y,
                actual_point.x,
                actual_point.y
            );
        }
    }

    fn create_variant_planner(mode: AStarVariantMode) -> AStarVariantPlanner {
        let (ox, oy) = build_pythonrobotics_maze();
        AStarVariantPlanner::new(
            &ox,
            &oy,
            AStarVariantConfig {
                resolution: 1.0,
                robot_radius: 0.0,
                mode,
                ..Default::default()
            },
        )
    }

    #[test]
    fn test_beam_search_matches_pythonrobotics_reference() {
        let planner = create_variant_planner(AStarVariantMode::BeamSearch);
        let actual = planner.plan_xy(5.0, 5.0, 35.0, 45.0).unwrap();
        let expected = parse_fixture(include_str!("testdata/a_star_variants_beam_python.csv"));
        assert_paths_match(&actual, &expected);
    }

    #[test]
    fn test_iterative_deepening_matches_pythonrobotics_reference() {
        let planner = create_variant_planner(AStarVariantMode::IterativeDeepening);
        let actual = planner.plan_xy(5.0, 5.0, 35.0, 45.0).unwrap();
        let expected = parse_fixture(include_str!(
            "testdata/a_star_variants_iterative_python.csv"
        ));
        assert_paths_match(&actual, &expected);
    }

    #[test]
    fn test_dynamic_weighting_matches_pythonrobotics_reference() {
        let planner = create_variant_planner(AStarVariantMode::DynamicWeighting);
        let actual = planner.plan_xy(5.0, 5.0, 35.0, 45.0).unwrap();
        let expected = parse_fixture(include_str!("testdata/a_star_variants_dynamic_python.csv"));
        assert_paths_match(&actual, &expected);
    }

    #[test]
    fn test_theta_star_like_matches_pythonrobotics_reference() {
        let planner = create_variant_planner(AStarVariantMode::ThetaStarLike);
        let actual = planner.plan_xy(5.0, 5.0, 35.0, 45.0).unwrap();
        let expected = parse_fixture(include_str!("testdata/a_star_variants_theta_python.csv"));
        assert_paths_match(&actual, &expected);
    }

    #[test]
    fn test_jump_point_corners_matches_pythonrobotics_reference() {
        let planner = create_variant_planner(AStarVariantMode::JumpPointCorners);
        let actual = planner.plan_xy(5.0, 5.0, 35.0, 45.0).unwrap();
        let expected = parse_fixture(include_str!("testdata/a_star_variants_jump_python.csv"));
        assert_paths_match(&actual, &expected);
    }

    #[test]
    fn test_variant_config_validation_rejects_invalid_beam_capacity() {
        let (ox, oy) = build_pythonrobotics_maze();
        let error = AStarVariantPlanner::try_new(
            &ox,
            &oy,
            AStarVariantConfig {
                beam_capacity: 0,
                ..Default::default()
            },
        )
        .unwrap_err();

        assert!(matches!(error, RoboticsError::InvalidParameter(_)));
    }
}
