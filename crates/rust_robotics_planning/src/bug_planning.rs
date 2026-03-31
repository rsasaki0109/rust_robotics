//! Bug algorithm family for path planning.
//!
//! Simple reactive planners that move greedily toward the goal on an integer
//! grid and follow obstacle boundaries when blocked.
//!
//! Three variants are provided:
//!
//! - **Bug0** -- greedy move toward goal; on hitting an obstacle, follow the
//!   boundary until the direct path is clear again.
//! - **Bug1** -- circumnavigate the entire obstacle, record the closest point
//!   to the goal, backtrack to that point, then resume greedy motion.
//! - **Bug2** -- precompute the start-goal line; follow the obstacle boundary
//!   until re-intersecting that line closer to the goal.
//!
//! Reference: <https://web.archive.org/web/20201103052224/https://sites.google.com/site/ece452bugalgorithms/>

use std::collections::HashSet;

/// Which Bug algorithm variant to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BugVariant {
    Bug0,
    Bug1,
    Bug2,
}

/// Configuration for the Bug planner.
#[derive(Debug, Clone)]
pub struct BugConfig {
    /// Algorithm variant.
    pub variant: BugVariant,
    /// Maximum number of iterations (safety limit).
    pub max_iterations: usize,
}

impl Default for BugConfig {
    fn default() -> Self {
        Self {
            variant: BugVariant::Bug0,
            max_iterations: 1_000_000,
        }
    }
}

/// Result of a Bug planning run.
#[derive(Debug, Clone)]
pub struct BugResult {
    /// X coordinates of the path.
    pub path_x: Vec<f64>,
    /// Y coordinates of the path.
    pub path_y: Vec<f64>,
}

/// Bug family path planner operating on an integer grid.
///
/// Obstacles are specified as a set of occupied grid cells. The planner
/// precomputes the *boundary* cells (free cells that are 8-connected
/// neighbours of at least one obstacle cell).
pub struct BugPlanner {
    goal_x: f64,
    goal_y: f64,
    obs: HashSet<(i64, i64)>,
    boundary: HashSet<(i64, i64)>,
    path_x: Vec<f64>,
    path_y: Vec<f64>,
}

impl BugPlanner {
    /// Create a new planner instance.
    ///
    /// `obs_x` / `obs_y` are the coordinates of every occupied grid cell.
    pub fn new(
        start_x: f64,
        start_y: f64,
        goal_x: f64,
        goal_y: f64,
        obs_x: &[f64],
        obs_y: &[f64],
    ) -> Self {
        let obs: HashSet<(i64, i64)> = obs_x
            .iter()
            .zip(obs_y.iter())
            .map(|(&x, &y)| (x as i64, y as i64))
            .collect();

        // Boundary = free cells that are 8-connected to an obstacle cell.
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
        }
    }

    /// Run the planner with the given configuration.
    pub fn plan(mut self, config: &BugConfig) -> Option<BugResult> {
        let ok = match config.variant {
            BugVariant::Bug0 => self.bug0(config.max_iterations),
            BugVariant::Bug1 => self.bug1(config.max_iterations),
            BugVariant::Bug2 => self.bug2(config.max_iterations),
        };
        if ok {
            Some(BugResult {
                path_x: self.path_x,
                path_y: self.path_y,
            })
        } else {
            None
        }
    }

    // ---- helpers ----

    /// Current position (last entry in path).
    fn cur(&self) -> (f64, f64) {
        (*self.path_x.last().unwrap(), *self.path_y.last().unwrap())
    }

    /// Greedy step toward the goal (sign-based, up to 1 cell per axis).
    fn mov_normal(&self) -> (f64, f64) {
        let (cx, cy) = self.cur();
        (cx + sign(self.goal_x - cx), cy + sign(self.goal_y - cy))
    }

    /// Move to the next unvisited boundary cell (4-connected neighbours).
    /// Returns `(x, y, back_to_start)`. If no unvisited boundary neighbour
    /// is found, the robot stays in place and `back_to_start` is `true`.
    fn mov_to_next_obs(&self, visited: &HashSet<(i64, i64)>) -> (f64, f64, bool) {
        let (cx, cy) = self.cur();
        let cx = cx as i64;
        let cy = cy as i64;
        for &(dx, dy) in &[(1i64, 0i64), (0, 1), (-1, 0), (0, -1)] {
            let nx = cx + dx;
            let ny = cy + dy;
            if self.boundary.contains(&(nx, ny)) && !visited.contains(&(nx, ny)) {
                return (nx as f64, ny as f64, false);
            }
        }
        (cx as f64, cy as f64, true)
    }

    fn at_goal(&self) -> bool {
        let (cx, cy) = self.cur();
        cx == self.goal_x && cy == self.goal_y
    }

    fn push(&mut self, x: f64, y: f64) {
        self.path_x.push(x);
        self.path_y.push(y);
    }

    /// Is (x,y) on the obstacle boundary?
    fn is_boundary(&self, x: f64, y: f64) -> bool {
        self.boundary.contains(&(x as i64, y as i64))
    }

    /// Is (x,y) an obstacle cell?
    fn is_obs(&self, x: f64, y: f64) -> bool {
        self.obs.contains(&(x as i64, y as i64))
    }

    // ---- Bug0 ----

    fn bug0(&mut self, max_iter: usize) -> bool {
        let mut mode = if self.is_boundary(self.cur().0, self.cur().1) {
            Mode::Obs
        } else {
            Mode::Normal
        };
        let mut visited = HashSet::new();

        for _ in 0..max_iter {
            if self.at_goal() {
                return true;
            }

            let (cand_x, cand_y) = if mode == Mode::Normal {
                self.mov_normal()
            } else {
                let (x, y, _) = self.mov_to_next_obs(&visited);
                (x, y)
            };

            match mode {
                Mode::Normal => {
                    if self.is_boundary(cand_x, cand_y) {
                        self.push(cand_x, cand_y);
                        visited.clear();
                        visited.insert((cand_x as i64, cand_y as i64));
                        mode = Mode::Obs;
                    } else {
                        self.push(cand_x, cand_y);
                    }
                }
                Mode::Obs => {
                    let (nx, ny) = self.mov_normal();
                    if !self.is_obs(nx, ny) {
                        mode = Mode::Normal;
                    } else {
                        self.push(cand_x, cand_y);
                        visited.insert((cand_x as i64, cand_y as i64));
                    }
                }
            }
        }
        false
    }

    // ---- Bug1 ----

    fn bug1(&mut self, max_iter: usize) -> bool {
        let mut mode = if self.is_boundary(self.cur().0, self.cur().1) {
            Mode::Obs
        } else {
            Mode::Normal
        };
        let mut visited = HashSet::new();
        let mut dist = f64::INFINITY;
        let mut exit_x = f64::NEG_INFINITY;
        let mut exit_y = f64::NEG_INFINITY;
        let mut back_to_start;
        let mut second_round = false;

        for _ in 0..max_iter {
            if self.at_goal() {
                return true;
            }

            let (cand_x, cand_y) = if mode == Mode::Normal {
                let c = self.mov_normal();
                back_to_start = false;
                c
            } else {
                let (x, y, b) = self.mov_to_next_obs(&visited);
                back_to_start = b;
                (x, y)
            };

            match mode {
                Mode::Normal => {
                    if self.is_boundary(cand_x, cand_y) {
                        self.push(cand_x, cand_y);
                        visited.clear();
                        visited.insert((cand_x as i64, cand_y as i64));
                        mode = Mode::Obs;
                        dist = f64::INFINITY;
                        second_round = false;
                    } else {
                        self.push(cand_x, cand_y);
                    }
                }
                Mode::Obs => {
                    let d =
                        ((cand_x - self.goal_x).powi(2) + (cand_y - self.goal_y).powi(2)).sqrt();
                    if d < dist && !second_round {
                        exit_x = cand_x;
                        exit_y = cand_y;
                        dist = d;
                    }
                    if back_to_start && !second_round {
                        second_round = true;
                        // Remove the first-round boundary trace; we will
                        // re-walk to exit_x/exit_y.
                        let n = visited.len();
                        let len = self.path_x.len();
                        self.path_x.truncate(len.saturating_sub(n));
                        self.path_y.truncate(len.saturating_sub(n));
                        visited.clear();
                    }
                    self.push(cand_x, cand_y);
                    visited.insert((cand_x as i64, cand_y as i64));
                    if cand_x == exit_x && cand_y == exit_y && second_round {
                        mode = Mode::Normal;
                    }
                }
            }
        }
        false
    }

    // ---- Bug2 ----

    fn bug2(&mut self, max_iter: usize) -> bool {
        // Precompute the start-goal line and find boundary hit points.
        let (sx, sy) = self.cur();
        let mut hit_points: Vec<(f64, f64)> = Vec::new();
        {
            let mut lx = sx;
            let mut ly = sy;
            loop {
                if lx == self.goal_x && ly == self.goal_y {
                    break;
                }
                let cx = lx + sign(self.goal_x - lx);
                let cy = ly + sign(self.goal_y - ly);
                if self.is_boundary(cx, cy) {
                    hit_points.push((cx, cy));
                }
                lx = cx;
                ly = cy;
            }
        }

        let mut mode = if self.is_boundary(sx, sy) {
            Mode::Obs
        } else {
            Mode::Normal
        };
        let mut visited = HashSet::new();

        for _ in 0..max_iter {
            if self.at_goal() {
                return true;
            }

            let (cand_x, cand_y) = if mode == Mode::Normal {
                self.mov_normal()
            } else {
                let (x, y, _) = self.mov_to_next_obs(&visited);
                (x, y)
            };

            match mode {
                Mode::Normal => {
                    if self.is_boundary(cand_x, cand_y) {
                        self.push(cand_x, cand_y);
                        visited.clear();
                        visited.insert((cand_x as i64, cand_y as i64));
                        // Remove the first matching hit point.
                        if let Some(idx) = hit_points
                            .iter()
                            .position(|&(hx, hy)| hx == cand_x && hy == cand_y)
                        {
                            hit_points.remove(idx);
                        }
                        mode = Mode::Obs;
                    } else {
                        self.push(cand_x, cand_y);
                    }
                }
                Mode::Obs => {
                    self.push(cand_x, cand_y);
                    visited.insert((cand_x as i64, cand_y as i64));
                    if let Some(idx) = hit_points
                        .iter()
                        .position(|&(hx, hy)| hx == cand_x && hy == cand_y)
                    {
                        hit_points.remove(idx);
                        mode = Mode::Normal;
                    }
                }
            }
        }
        false
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    Normal,
    Obs,
}

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

#[cfg(test)]
mod tests {
    use super::*;

    /// Build the same obstacle field used in the PythonRobotics reference.
    fn build_obstacles() -> (Vec<f64>, Vec<f64>) {
        let mut ox = Vec::new();
        let mut oy = Vec::new();
        let ranges: &[(i64, i64, i64, i64)] = &[
            (20, 40, 20, 40),
            (60, 100, 40, 80),
            (120, 140, 80, 100),
            (80, 140, 0, 20),
            (0, 20, 60, 100),
            (20, 40, 80, 100),
            (120, 160, 40, 60),
        ];
        for &(x0, x1, y0, y1) in ranges {
            for i in x0..x1 {
                for j in y0..y1 {
                    ox.push(i as f64);
                    oy.push(j as f64);
                }
            }
        }
        (ox, oy)
    }

    #[test]
    fn test_bug0_reaches_goal() {
        let (ox, oy) = build_obstacles();
        let planner = BugPlanner::new(0.0, 0.0, 167.0, 50.0, &ox, &oy);
        let config = BugConfig {
            variant: BugVariant::Bug0,
            ..Default::default()
        };
        let result = planner.plan(&config).expect("Bug0 should find a path");
        assert_eq!(*result.path_x.last().unwrap(), 167.0);
        assert_eq!(*result.path_y.last().unwrap(), 50.0);
        assert!(result.path_x.len() > 2);
    }

    #[test]
    fn test_bug1_reaches_goal() {
        let (ox, oy) = build_obstacles();
        let planner = BugPlanner::new(0.0, 0.0, 167.0, 50.0, &ox, &oy);
        let config = BugConfig {
            variant: BugVariant::Bug1,
            ..Default::default()
        };
        let result = planner.plan(&config).expect("Bug1 should find a path");
        assert_eq!(*result.path_x.last().unwrap(), 167.0);
        assert_eq!(*result.path_y.last().unwrap(), 50.0);
    }

    #[test]
    fn test_bug2_reaches_goal() {
        let (ox, oy) = build_obstacles();
        let planner = BugPlanner::new(0.0, 0.0, 167.0, 50.0, &ox, &oy);
        let config = BugConfig {
            variant: BugVariant::Bug2,
            ..Default::default()
        };
        let result = planner.plan(&config).expect("Bug2 should find a path");
        assert_eq!(*result.path_x.last().unwrap(), 167.0);
        assert_eq!(*result.path_y.last().unwrap(), 50.0);
    }

    #[test]
    fn test_straight_path_no_obstacles() {
        let planner = BugPlanner::new(0.0, 0.0, 5.0, 5.0, &[], &[]);
        let config = BugConfig {
            variant: BugVariant::Bug0,
            ..Default::default()
        };
        let result = planner.plan(&config).expect("should reach goal");
        assert_eq!(*result.path_x.last().unwrap(), 5.0);
        assert_eq!(*result.path_y.last().unwrap(), 5.0);
        // Diagonal steps: 5 steps + start = 6 points
        assert_eq!(result.path_x.len(), 6);
    }

    #[test]
    fn test_config_default() {
        let config = BugConfig::default();
        assert_eq!(config.variant, BugVariant::Bug0);
        assert_eq!(config.max_iterations, 1_000_000);
    }
}
