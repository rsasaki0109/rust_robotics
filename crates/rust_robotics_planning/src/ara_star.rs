//! Anytime Repairing A\* (ARA\*) path planning algorithm
//!
//! ARA\* finds a suboptimal solution quickly using an inflated heuristic
//! (epsilon > 1), then iteratively decreases epsilon and improves the
//! solution, reusing previous search effort.
//!
//! Reference: Likhachev et al., "ARA\*: Anytime A\* with Provable Bounds on
//! Sub-Optimality", NIPS 2003.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use crate::grid::GridMap;
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Configuration for ARA\* planner
#[derive(Debug, Clone)]
pub struct ARAStarConfig {
    /// Grid resolution in meters \[m\]
    pub resolution: f64,
    /// Robot radius for obstacle inflation \[m\]
    pub robot_radius: f64,
    /// Initial inflation factor (epsilon). Values > 1 yield suboptimal but
    /// faster solutions.
    pub initial_epsilon: f64,
    /// Amount by which epsilon is decreased each iteration
    pub epsilon_step: f64,
    /// Minimum (final) epsilon. At 1.0 the algorithm is optimal A\*.
    pub final_epsilon: f64,
}

impl Default for ARAStarConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
            initial_epsilon: 3.0,
            epsilon_step: 0.5,
            final_epsilon: 1.0,
        }
    }
}

impl ARAStarConfig {
    /// Validate all configuration parameters.
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
        if !self.initial_epsilon.is_finite() || self.initial_epsilon < 1.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "initial_epsilon must be >= 1.0 and finite, got {}",
                self.initial_epsilon
            )));
        }
        if !self.epsilon_step.is_finite() || self.epsilon_step <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "epsilon_step must be positive and finite, got {}",
                self.epsilon_step
            )));
        }
        if !self.final_epsilon.is_finite() || self.final_epsilon < 1.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "final_epsilon must be >= 1.0 and finite, got {}",
                self.final_epsilon
            )));
        }
        if self.final_epsilon > self.initial_epsilon {
            return Err(RoboticsError::InvalidParameter(format!(
                "final_epsilon ({}) must be <= initial_epsilon ({})",
                self.final_epsilon, self.initial_epsilon
            )));
        }
        Ok(())
    }
}

// ── Priority node for the min-heap open set ──────────────────────────────────

#[derive(Debug)]
struct PriorityNode {
    grid_idx: i32,
    g: f64,
    f: f64,
}

impl Eq for PriorityNode {}

impl PartialEq for PriorityNode {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}

impl Ord for PriorityNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse for min-heap
        other.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for PriorityNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// ── ARA\* planner ─────────────────────────────────────────────────────────────

/// ARA\* (Anytime Repairing A\*) path planner.
///
/// Produces a sequence of improving solutions by repeatedly running weighted
/// A\* with a decreasing inflation factor epsilon.
pub struct ARAStarPlanner {
    grid_map: GridMap,
    config: ARAStarConfig,
    motion: Vec<(i32, i32, f64)>,
}

impl ARAStarPlanner {
    /// Create a new ARA\* planner. Panics on invalid input.
    pub fn new(ox: &[f64], oy: &[f64], config: ARAStarConfig) -> Self {
        Self::try_new(ox, oy, config).expect(
            "invalid ARA* planner input: obstacles must be non-empty and valid, config values must be positive/finite",
        )
    }

    /// Create a validated ARA\* planner.
    pub fn try_new(ox: &[f64], oy: &[f64], config: ARAStarConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.resolution, config.robot_radius)?;
        Ok(Self {
            grid_map,
            motion: Self::motion_model(),
            config,
        })
    }

    /// Create a validated ARA\* planner from typed obstacle points.
    pub fn from_obstacle_points(
        obstacles: &Obstacles,
        config: ARAStarConfig,
    ) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::from_obstacles(obstacles, config.resolution, config.robot_radius)?;
        Ok(Self {
            grid_map,
            motion: Self::motion_model(),
            config,
        })
    }

    /// Plan a path from `start` to `goal`, returning the best (lowest epsilon)
    /// solution found.
    pub fn plan(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        let solutions = self.plan_anytime(start, goal);
        solutions
            .into_iter()
            .last()
            .map(|(_, path)| path)
            .ok_or_else(|| RoboticsError::PlanningError("No path found".to_string()))
    }

    /// Run the full ARA\* anytime loop, returning every improving solution as
    /// a `(epsilon, path)` pair ordered from highest (first, fast) to lowest
    /// (last, most optimal) epsilon.
    pub fn plan_anytime(&self, start: Point2D, goal: Point2D) -> Vec<(f64, Path2D)> {
        let start_x = self.grid_map.calc_x_index(start.x);
        let start_y = self.grid_map.calc_y_index(start.y);
        let goal_x = self.grid_map.calc_x_index(goal.x);
        let goal_y = self.grid_map.calc_y_index(goal.y);

        if !self.grid_map.is_valid(start_x, start_y) || !self.grid_map.is_valid(goal_x, goal_y) {
            return Vec::new();
        }

        let start_idx = self.grid_map.calc_index(start_x, start_y);
        let goal_idx = self.grid_map.calc_index(goal_x, goal_y);

        // g-value table shared across iterations: grid_idx -> g
        let mut g_table: HashMap<i32, f64> = HashMap::new();
        // parent table: grid_idx -> (parent_grid_idx, node x, node y)
        let mut parent: HashMap<i32, Option<i32>> = HashMap::new();
        // node coordinate lookup: grid_idx -> (x, y)
        let mut coords: HashMap<i32, (i32, i32)> = HashMap::new();

        g_table.insert(start_idx, 0.0);
        parent.insert(start_idx, None);
        coords.insert(start_idx, (start_x, start_y));
        coords.insert(goal_idx, (goal_x, goal_y));

        let mut solutions: Vec<(f64, Path2D)> = Vec::new();
        let mut epsilon = self.config.initial_epsilon;

        loop {
            if let Some(path) = self.weighted_a_star(
                start_idx,
                goal_idx,
                start_x,
                start_y,
                goal_x,
                goal_y,
                epsilon,
                &mut g_table,
                &mut parent,
                &mut coords,
            ) {
                solutions.push((epsilon, path));
            }

            if epsilon <= self.config.final_epsilon + f64::EPSILON {
                break;
            }
            epsilon = (epsilon - self.config.epsilon_step).max(self.config.final_epsilon);
        }

        solutions
    }

    /// Get a reference to the underlying grid map.
    pub fn grid_map(&self) -> &GridMap {
        &self.grid_map
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

    fn motion_model() -> Vec<(i32, i32, f64)> {
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

    fn heuristic(&self, x: i32, y: i32, gx: i32, gy: i32) -> f64 {
        (((x - gx).pow(2) + (y - gy).pow(2)) as f64).sqrt()
    }

    /// Run one pass of weighted A\* with the given epsilon, seeding the search
    /// from the current g-value table (ARA\* "warm start").
    ///
    /// Returns the reconstructed path if the goal is reachable.
    #[allow(clippy::too_many_arguments)]
    fn weighted_a_star(
        &self,
        start_idx: i32,
        goal_idx: i32,
        start_x: i32,
        start_y: i32,
        goal_x: i32,
        goal_y: i32,
        epsilon: f64,
        g_table: &mut HashMap<i32, f64>,
        parent: &mut HashMap<i32, Option<i32>>,
        coords: &mut HashMap<i32, (i32, i32)>,
    ) -> Option<Path2D> {
        let start_g = *g_table.get(&start_idx).unwrap_or(&0.0);
        let start_h = epsilon * self.heuristic(start_x, start_y, goal_x, goal_y);

        let mut open: BinaryHeap<PriorityNode> = BinaryHeap::new();
        // incons holds nodes that were improved while already in closed
        let mut incons: HashMap<i32, f64> = HashMap::new();
        let mut closed: HashMap<i32, f64> = HashMap::new();

        open.push(PriorityNode {
            grid_idx: start_idx,
            g: start_g,
            f: start_g + start_h,
        });

        // Seed open set with all previously discovered nodes that are not
        // in closed — their f-values may have changed with the new epsilon.
        for (&idx, &g) in g_table.iter() {
            if idx == start_idx {
                continue;
            }
            if let Some(&(x, y)) = coords.get(&idx) {
                if self.grid_map.is_valid(x, y) {
                    let f = g + epsilon * self.heuristic(x, y, goal_x, goal_y);
                    open.push(PriorityNode {
                        grid_idx: idx,
                        g,
                        f,
                    });
                }
            }
        }

        while let Some(current) = open.pop() {
            // Skip stale entries (lazy deletion)
            if let Some(&best_g) = g_table.get(&current.grid_idx) {
                if current.g > best_g + f64::EPSILON {
                    continue;
                }
            }
            if closed.contains_key(&current.grid_idx) {
                continue;
            }

            closed.insert(current.grid_idx, current.g);

            if current.grid_idx == goal_idx {
                return Some(self.reconstruct_path(goal_idx, start_idx, parent, coords));
            }

            let (cx, cy) = match coords.get(&current.grid_idx) {
                Some(&c) => c,
                None => continue,
            };

            for &(dx, dy, move_cost) in &self.motion {
                if !self.grid_map.is_valid_offset(cx, cy, dx, dy) {
                    continue;
                }

                let nx = cx + dx;
                let ny = cy + dy;
                let n_idx = self.grid_map.calc_index(nx, ny);
                let new_g = current.g + move_cost;

                let old_g = g_table.get(&n_idx).copied().unwrap_or(f64::INFINITY);

                if new_g < old_g - f64::EPSILON {
                    g_table.insert(n_idx, new_g);
                    coords.insert(n_idx, (nx, ny));
                    parent.insert(n_idx, Some(current.grid_idx));

                    if closed.contains_key(&n_idx) {
                        // Node already expanded — put in incons for next pass
                        incons.insert(n_idx, new_g);
                    } else {
                        let f = new_g + epsilon * self.heuristic(nx, ny, goal_x, goal_y);
                        open.push(PriorityNode {
                            grid_idx: n_idx,
                            g: new_g,
                            f,
                        });
                    }
                }
            }
        }

        // Move incons into g_table so next iteration can pick them up
        for (idx, g) in incons {
            g_table.insert(idx, g);
        }

        None
    }

    fn reconstruct_path(
        &self,
        goal_idx: i32,
        start_idx: i32,
        parent: &HashMap<i32, Option<i32>>,
        coords: &HashMap<i32, (i32, i32)>,
    ) -> Path2D {
        let mut points = Vec::new();
        let mut current = goal_idx;

        loop {
            if let Some(&(x, y)) = coords.get(&current) {
                points.push(Point2D::new(
                    self.grid_map.calc_x_position(x),
                    self.grid_map.calc_y_position(y),
                ));
            }
            if current == start_idx {
                break;
            }
            match parent.get(&current) {
                Some(Some(p)) => current = *p,
                _ => break,
            }
        }

        points.reverse();
        Path2D::from_points(points)
    }
}

impl PathPlanner for ARAStarPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        ARAStarPlanner::plan(self, start, goal)
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_obstacles() -> (Vec<f64>, Vec<f64>) {
        let mut ox = Vec::new();
        let mut oy = Vec::new();
        for i in 0..=10 {
            ox.push(i as f64);
            oy.push(0.0);
            ox.push(i as f64);
            oy.push(10.0);
            ox.push(0.0);
            oy.push(i as f64);
            ox.push(10.0);
            oy.push(i as f64);
        }
        // Internal wall with a gap
        for i in 3..8 {
            ox.push(5.0);
            oy.push(i as f64);
        }
        (ox, oy)
    }

    #[test]
    fn test_ara_star_finds_path() {
        let (ox, oy) = simple_obstacles();
        let planner = ARAStarPlanner::new(&ox, &oy, ARAStarConfig::default());

        let path = planner.plan(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0));
        assert!(path.is_ok(), "ARA* should find a path: {:?}", path.err());
        assert!(!path.unwrap().is_empty());
    }

    #[test]
    fn test_plan_anytime_returns_multiple_solutions() {
        let (ox, oy) = simple_obstacles();
        let config = ARAStarConfig {
            initial_epsilon: 3.0,
            epsilon_step: 0.5,
            final_epsilon: 1.0,
            ..Default::default()
        };
        let planner = ARAStarPlanner::new(&ox, &oy, config);

        let solutions = planner.plan_anytime(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0));
        // With epsilon going 3.0 -> 2.5 -> 2.0 -> 1.5 -> 1.0 we expect 5 iterations
        assert!(
            solutions.len() >= 2,
            "plan_anytime should return at least 2 solutions, got {}",
            solutions.len()
        );
    }

    #[test]
    fn test_first_solution_cost_not_less_than_final() {
        let (ox, oy) = simple_obstacles();
        let config = ARAStarConfig {
            initial_epsilon: 3.0,
            epsilon_step: 0.5,
            final_epsilon: 1.0,
            ..Default::default()
        };
        let planner = ARAStarPlanner::new(&ox, &oy, config);

        let solutions = planner.plan_anytime(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0));
        assert!(solutions.len() >= 2, "need at least 2 solutions to compare");

        let first_cost = solutions.first().unwrap().1.total_length();
        let last_cost = solutions.last().unwrap().1.total_length();
        println!("first_cost={first_cost:.4} last_cost={last_cost:.4}");

        // Inflated heuristic can only produce solutions >= optimal length
        assert!(
            first_cost >= last_cost - 1e-9,
            "first solution ({first_cost}) should be >= final solution ({last_cost})"
        );
    }

    #[test]
    fn test_config_defaults() {
        let cfg = ARAStarConfig::default();
        assert_eq!(cfg.resolution, 1.0);
        assert_eq!(cfg.robot_radius, 0.5);
        assert_eq!(cfg.initial_epsilon, 3.0);
        assert_eq!(cfg.epsilon_step, 0.5);
        assert_eq!(cfg.final_epsilon, 1.0);
        assert!(cfg.validate().is_ok());
    }
}
