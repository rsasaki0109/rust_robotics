//! Tangent Bug path planning algorithm.
//!
//! An improvement over the basic Bug algorithms that uses a finite-range
//! sensor to detect obstacle boundaries and selects the tangent point that
//! minimises the heuristic distance to the goal.
//!
//! # Algorithm summary
//!
//! 1. **Motion-to-Goal**: move one step toward the goal on the integer grid.
//!    If the next cell is an obstacle, enter Boundary-Following mode.
//! 2. **Boundary-Following**: scan all boundary cells within `sensor_range`,
//!    pick the one with the minimum Euclidean distance to goal as the *tangent
//!    point*.  Walk along the obstacle boundary (4-connected free cells
//!    adjacent to at least one obstacle cell) in the direction that makes
//!    progress toward the tangent point.  Switch back to Motion-to-Goal when
//!    the direct path to the goal is unobstructed **and** the current distance
//!    to the goal is less than `d_reach` (the distance recorded when
//!    boundary-following began).
//!
//! Reference: Kamon & Rivlin (1997), *Sensory-based motion planning with
//! global proofs*, IEEE Trans. Robotics & Automation.

use std::collections::HashSet;

// ── Configuration ────────────────────────────────────────────────────────────

/// Configuration for the [`TangentBugPlanner`].
#[derive(Debug, Clone)]
pub struct TangentBugConfig {
    /// Sensor range used to discover boundary cells \[grid units\].
    pub sensor_range: f64,
    /// Safety limit on the total number of iterations.
    pub max_iterations: usize,
}

impl Default for TangentBugConfig {
    fn default() -> Self {
        Self {
            sensor_range: 10.0,
            max_iterations: 1_000_000,
        }
    }
}

// ── Result ────────────────────────────────────────────────────────────────────

/// The planned path returned by [`TangentBugPlanner::plan`].
#[derive(Debug, Clone)]
pub struct TangentBugResult {
    /// X coordinates of the path, from start to goal.
    pub path_x: Vec<f64>,
    /// Y coordinates of the path, from start to goal.
    pub path_y: Vec<f64>,
}

// ── Planner ───────────────────────────────────────────────────────────────────

/// Tangent Bug planner operating on an integer grid.
///
/// Obstacles are specified as a set of occupied grid cells.  The planner
/// pre-computes *boundary* cells (free cells that are 8-connected neighbours
/// of at least one obstacle cell) and uses a finite sensor range to pick
/// tangent waypoints.
pub struct TangentBugPlanner {
    goal_x: f64,
    goal_y: f64,
    obs: HashSet<(i64, i64)>,
    boundary: HashSet<(i64, i64)>,
    path_x: Vec<f64>,
    path_y: Vec<f64>,
    sensor_range: f64,
}

impl TangentBugPlanner {
    /// Create a new planner instance.
    ///
    /// # Arguments
    ///
    /// * `start_x`, `start_y` – starting position.
    /// * `goal_x`, `goal_y`   – goal position.
    /// * `obs_x`, `obs_y`     – coordinates of every occupied grid cell.
    /// * `sensor_range`       – maximum sensing radius \[grid units\].
    pub fn new(
        start_x: f64,
        start_y: f64,
        goal_x: f64,
        goal_y: f64,
        obs_x: &[f64],
        obs_y: &[f64],
        sensor_range: f64,
    ) -> Self {
        let obs: HashSet<(i64, i64)> = obs_x
            .iter()
            .zip(obs_y.iter())
            .map(|(&x, &y)| (x as i64, y as i64))
            .collect();

        let dirs: [(i64, i64); 8] = [
            (1, 1),
            (0, 1),
            (-1, 1),
            (-1, 0),
            (-1, -1),
            (0, -1),
            (1, -1),
            (1, 0),
        ];
        let mut boundary = HashSet::new();
        for &(ox, oy) in &obs {
            for &(dx, dy) in &dirs {
                let cx = ox + dx;
                let cy = oy + dy;
                if !obs.contains(&(cx, cy)) {
                    boundary.insert((cx, cy));
                }
            }
        }

        Self {
            goal_x,
            goal_y,
            obs,
            boundary,
            path_x: vec![start_x],
            path_y: vec![start_y],
            sensor_range,
        }
    }

    /// Run the Tangent Bug algorithm.
    ///
    /// Returns `Some(TangentBugResult)` when the goal is reached, or `None`
    /// when the iteration limit is exhausted without reaching the goal.
    pub fn plan(mut self, config: &TangentBugConfig) -> Option<TangentBugResult> {
        // Take the larger of constructor-provided and config-provided range so
        // the config acts as an override rather than a mandatory reset.
        let sensor_range = config.sensor_range.max(self.sensor_range);
        if self.run(config.max_iterations, sensor_range) {
            Some(TangentBugResult {
                path_x: self.path_x,
                path_y: self.path_y,
            })
        } else {
            None
        }
    }

    // ── internal helpers ──────────────────────────────────────────────────────

    fn cur(&self) -> (f64, f64) {
        (*self.path_x.last().unwrap(), *self.path_y.last().unwrap())
    }

    fn push(&mut self, x: f64, y: f64) {
        self.path_x.push(x);
        self.path_y.push(y);
    }

    fn at_goal(&self) -> bool {
        let (cx, cy) = self.cur();
        cx == self.goal_x && cy == self.goal_y
    }

    /// One-cell greedy step toward the goal (diagonal allowed).
    fn step_toward_goal(&self) -> (f64, f64) {
        let (cx, cy) = self.cur();
        (cx + sign(self.goal_x - cx), cy + sign(self.goal_y - cy))
    }

    fn is_obs(&self, x: i64, y: i64) -> bool {
        self.obs.contains(&(x, y))
    }

    fn is_boundary_cell(&self, x: i64, y: i64) -> bool {
        self.boundary.contains(&(x, y))
    }

    fn dist_to_goal(&self, x: f64, y: f64) -> f64 {
        ((x - self.goal_x).powi(2) + (y - self.goal_y).powi(2)).sqrt()
    }

    /// Scan boundary cells within `sensor_range` of the current position and
    /// return the one that minimises the Euclidean distance to the goal.
    /// Returns `None` if no boundary cell is in range.
    fn best_tangent_point(&self, sensor_range: f64) -> Option<(f64, f64)> {
        let (cx, cy) = self.cur();
        let sr2 = sensor_range * sensor_range;
        let mut best_dist = f64::INFINITY;
        let mut best: Option<(f64, f64)> = None;

        for &(bx, by) in &self.boundary {
            let bxf = bx as f64;
            let byf = by as f64;
            let d2 = (bxf - cx).powi(2) + (byf - cy).powi(2);
            if d2 <= sr2 {
                let dg = self.dist_to_goal(bxf, byf);
                if dg < best_dist {
                    best_dist = dg;
                    best = Some((bxf, byf));
                }
            }
        }
        best
    }

    /// Check whether the direct path from current position to the goal is
    /// clear of obstacles (no obstacle cell along the sign-based walk).
    fn direct_path_clear(&self) -> bool {
        let (mut lx, mut ly) = self.cur();
        loop {
            if lx == self.goal_x && ly == self.goal_y {
                return true;
            }
            let nx = lx + sign(self.goal_x - lx);
            let ny = ly + sign(self.goal_y - ly);
            if self.is_obs(nx as i64, ny as i64) {
                return false;
            }
            lx = nx;
            ly = ny;
        }
    }

    /// Walk one step along the obstacle boundary toward the tangent point
    /// `(tx, ty)`.  Boundary cells (free cells adjacent to at least one
    /// obstacle) are preferred; any free neighbour is used as a fallback.
    /// The neighbour with the smallest Euclidean distance to `(tx, ty)` that
    /// has not been visited in the current boundary-following episode is
    /// chosen.  If all neighbours have been visited, the closest unvisited
    /// free neighbour is chosen regardless.
    ///
    /// Returns `true` if a step was made.
    fn boundary_follow_step(&mut self, tx: f64, ty: f64, visited: &HashSet<(i64, i64)>) -> bool {
        let (cx, cy) = self.cur();
        let cxi = cx as i64;
        let cyi = cy as i64;

        // 4-connected neighbours, tried in order: prefer boundary, unvisited,
        // closest to target.
        let mut best_dist = f64::INFINITY;
        let mut best: Option<(f64, f64)> = None;
        let mut best_dist_any = f64::INFINITY;
        let mut best_any: Option<(f64, f64)> = None;

        for &(dx, dy) in &[(1i64, 0i64), (0, 1), (-1, 0), (0, -1)] {
            let nx = cxi + dx;
            let ny = cyi + dy;
            if self.is_obs(nx, ny) {
                continue;
            }
            let nxf = nx as f64;
            let nyf = ny as f64;
            let d = ((nxf - tx).powi(2) + (nyf - ty).powi(2)).sqrt();

            // Track closest free cell overall (for fallback).
            if d < best_dist_any {
                best_dist_any = d;
                best_any = Some((nxf, nyf));
            }

            // Prefer boundary cells that haven't been visited yet.
            if self.is_boundary_cell(nx, ny) && !visited.contains(&(nx, ny)) && d < best_dist {
                best_dist = d;
                best = Some((nxf, nyf));
            }
        }

        // Fall back to any free unvisited neighbour, then any free neighbour.
        let chosen = best.or_else(|| {
            // Any unvisited free neighbour closest to target.
            let (mut bd, mut bst) = (f64::INFINITY, None);
            let cxi = cx as i64;
            let cyi = cy as i64;
            for &(dx, dy) in &[(1i64, 0i64), (0, 1), (-1, 0), (0, -1)] {
                let nx = cxi + dx;
                let ny = cyi + dy;
                if !self.is_obs(nx, ny) && !visited.contains(&(nx, ny)) {
                    let d = ((nx as f64 - tx).powi(2) + (ny as f64 - ty).powi(2)).sqrt();
                    if d < bd {
                        bd = d;
                        bst = Some((nx as f64, ny as f64));
                    }
                }
            }
            bst
        });

        let chosen = chosen.or(best_any);

        if let Some((bx, by)) = chosen {
            self.push(bx, by);
            true
        } else {
            false
        }
    }

    // ── main loop ─────────────────────────────────────────────────────────────

    fn run(&mut self, max_iter: usize, sensor_range: f64) -> bool {
        #[derive(Clone, Copy, PartialEq, Eq)]
        enum Mode {
            /// Greedy motion toward the goal.
            ToGoal,
            /// Following the obstacle boundary via tangent points.
            Boundary,
        }

        let mut mode = Mode::ToGoal;
        // Distance to goal recorded when boundary-following began.
        let mut d_reach = f64::INFINITY;
        // Cells visited during the current boundary-following episode.
        let mut visited: HashSet<(i64, i64)> = HashSet::new();

        for _ in 0..max_iter {
            if self.at_goal() {
                return true;
            }

            match mode {
                Mode::ToGoal => {
                    let (nx, ny) = self.step_toward_goal();
                    if self.is_obs(nx as i64, ny as i64) {
                        // Obstacle ahead – switch to boundary-following.
                        d_reach = self.dist_to_goal(self.cur().0, self.cur().1);
                        visited.clear();
                        let (cx, cy) = self.cur();
                        visited.insert((cx as i64, cy as i64));
                        mode = Mode::Boundary;
                        // Do not move this iteration; re-evaluate next cycle.
                    } else {
                        self.push(nx, ny);
                    }
                }

                Mode::Boundary => {
                    // Check exit condition: direct path clear AND closer to
                    // goal than when we started following the boundary.
                    let (cx, cy) = self.cur();
                    let d_now = self.dist_to_goal(cx, cy);
                    if self.direct_path_clear() && d_now < d_reach {
                        visited.clear();
                        mode = Mode::ToGoal;
                        continue;
                    }

                    // Pick the best tangent point visible within sensor range.
                    if let Some((tx, ty)) = self.best_tangent_point(sensor_range) {
                        let moved = self.boundary_follow_step(tx, ty, &visited);
                        if moved {
                            let (nx, ny) = self.cur();
                            visited.insert((nx as i64, ny as i64));
                        } else {
                            // Truly stuck – give up.
                            return false;
                        }
                    } else {
                        // No boundary in sensor range – resume greedy motion.
                        visited.clear();
                        mode = Mode::ToGoal;
                    }
                }
            }
        }
        false
    }
}

// ── helpers ───────────────────────────────────────────────────────────────────

/// Signum returning -1.0, 0.0, or 1.0.
fn sign(v: f64) -> f64 {
    if v > 0.0 {
        1.0
    } else if v < 0.0 {
        -1.0
    } else {
        0.0
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_straight_path_no_obstacles() {
        let planner = TangentBugPlanner::new(0.0, 0.0, 5.0, 5.0, &[], &[], 10.0);
        let config = TangentBugConfig::default();
        let result = planner.plan(&config).expect("should reach goal");
        assert_eq!(*result.path_x.last().unwrap(), 5.0);
        assert_eq!(*result.path_y.last().unwrap(), 5.0);
        // Diagonal steps: 5 steps + start = 6 points.
        assert_eq!(result.path_x.len(), 6);
    }

    #[test]
    fn test_path_around_simple_obstacle() {
        // Vertical wall at x=3, y=-2..=6.  Robot at (0,0), goal at (6,0).
        // The robot must navigate around the top or bottom of the wall.
        let mut obs_x = Vec::new();
        let mut obs_y = Vec::new();
        for j in -2i32..=6 {
            obs_x.push(3.0f64);
            obs_y.push(j as f64);
        }

        let planner = TangentBugPlanner::new(0.0, 0.0, 6.0, 0.0, &obs_x, &obs_y, 8.0);
        let config = TangentBugConfig::default();
        let result = planner
            .plan(&config)
            .expect("should reach goal despite obstacle");
        assert_eq!(*result.path_x.last().unwrap(), 6.0);
        assert_eq!(*result.path_y.last().unwrap(), 0.0);
        // Path must be longer than the 6-step straight-line route.
        assert!(result.path_x.len() > 7);
    }

    #[test]
    fn test_config_defaults() {
        let config = TangentBugConfig::default();
        assert_eq!(config.sensor_range, 10.0);
        assert_eq!(config.max_iterations, 1_000_000);
    }
}
