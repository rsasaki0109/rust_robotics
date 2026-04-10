//! Lifelong Planning A\* (LPA\*) path planning algorithm
//!
//! An incremental version of A\* that efficiently replans when edge costs
//! change. Each node maintains two values: `g` (cost-so-far) and `rhs`
//! (one-step lookahead of g). When edge costs change only affected nodes
//! are updated, avoiding a full replan.
//!
//! Reference: Koenig & Likhachev (2002), "D\* Lite"

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use crate::grid::GridMap;
use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

// ── Configuration ─────────────────────────────────────────────────────────────

/// Configuration for the LPA\* planner.
#[derive(Debug, Clone)]
pub struct LPAStarConfig {
    /// Grid resolution in metres \[m\]
    pub resolution: f64,
    /// Robot radius for obstacle inflation \[m\]
    pub robot_radius: f64,
    /// Heuristic weight (1.0 = admissible/optimal, >1.0 = faster/suboptimal)
    pub heuristic_weight: f64,
}

impl Default for LPAStarConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        }
    }
}

impl LPAStarConfig {
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
        if !self.heuristic_weight.is_finite() || self.heuristic_weight <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "heuristic_weight must be positive and finite, got {}",
                self.heuristic_weight
            )));
        }
        Ok(())
    }
}

// ── Changed-edge description ───────────────────────────────────────────────────

/// Describes a single edge-cost change for incremental replanning.
///
/// Both coordinates are in world space \[m\].
#[derive(Debug, Clone)]
pub struct EdgeChange {
    /// World-space x of the affected cell
    pub x: f64,
    /// World-space y of the affected cell
    pub y: f64,
    /// New traversal cost (`f64::INFINITY` means the cell became an obstacle)
    pub new_cost: f64,
}

// ── Priority-queue key ─────────────────────────────────────────────────────────

/// LPA\* priority key: `[k1, k2]` where `k1 = min(g,rhs)+h`, `k2 = min(g,rhs)`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Key(f64, f64);

impl Key {
    fn new(g: f64, rhs: f64, h: f64) -> Self {
        let m = g.min(rhs);
        Key(m + h, m)
    }

    /// Natural (non-reversed) less-than comparison for termination check.
    fn less_than(&self, other: &Self) -> bool {
        match self.0.partial_cmp(&other.0).unwrap_or(Ordering::Equal) {
            Ordering::Less => true,
            Ordering::Greater => false,
            Ordering::Equal => self.1 < other.1,
        }
    }
}

impl Eq for Key {}

impl Ord for Key {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reversed for min-heap (BinaryHeap is a max-heap)
        match other.0.partial_cmp(&self.0).unwrap_or(Ordering::Equal) {
            Ordering::Equal => other.1.partial_cmp(&self.1).unwrap_or(Ordering::Equal),
            ord => ord,
        }
    }
}

impl PartialOrd for Key {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// ── Queue entry ────────────────────────────────────────────────────────────────

#[derive(Debug, Eq, PartialEq)]
struct QueueEntry {
    key: Key,
    grid_idx: i32,
}

impl Ord for QueueEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        self.key.cmp(&other.key)
    }
}

impl PartialOrd for QueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// ── Planner ────────────────────────────────────────────────────────────────────

/// Lifelong Planning A\* planner.
///
/// Keeps `g` and `rhs` tables across calls so that incremental replanning
/// (via [`plan_with_cost_change`](LPAStarPlanner::plan_with_cost_change))
/// only processes nodes affected by edge-cost changes.
pub struct LPAStarPlanner {
    grid_map: GridMap,
    config: LPAStarConfig,
    motion: Vec<(i32, i32, f64)>,
}

impl LPAStarPlanner {
    /// Create a planner from raw obstacle x/y slices; panics on invalid input.
    pub fn new(ox: &[f64], oy: &[f64], config: LPAStarConfig) -> Self {
        Self::try_new(ox, oy, config).expect(
            "invalid LPA* planner input: obstacles must be non-empty and finite, \
             config values must be positive/finite",
        )
    }

    /// Create a validated planner from raw obstacle x/y slices.
    pub fn try_new(ox: &[f64], oy: &[f64], config: LPAStarConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::try_new(ox, oy, config.resolution, config.robot_radius)?;
        Ok(Self {
            grid_map,
            config,
            motion: Self::motion_model(),
        })
    }

    /// Create a validated planner from typed obstacle points.
    pub fn from_obstacle_points(
        obstacles: &Obstacles,
        config: LPAStarConfig,
    ) -> RoboticsResult<Self> {
        config.validate()?;
        let grid_map = GridMap::from_obstacles(obstacles, config.resolution, config.robot_radius)?;
        Ok(Self {
            grid_map,
            config,
            motion: Self::motion_model(),
        })
    }

    /// Plan a path from `start` to `goal`.
    pub fn plan(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        self.plan_impl(start, goal)
    }

    /// Replan after edge-cost changes without restarting from scratch.
    ///
    /// `changed_edges` lists cells whose traversal cost has changed.  The
    /// planner updates only the affected vertices and runs
    /// `ComputeShortestPath` to convergence.
    pub fn plan_with_cost_change(
        &mut self,
        start: Point2D,
        goal: Point2D,
        changed_edges: &[EdgeChange],
    ) -> RoboticsResult<Path2D> {
        // Apply cost changes to the underlying obstacle map.
        for change in changed_edges {
            let ix = self.grid_map.calc_x_index(change.x) as usize;
            let iy = self.grid_map.calc_y_index(change.y) as usize;
            if ix < self.grid_map.x_width as usize && iy < self.grid_map.y_width as usize {
                self.grid_map.obstacle_map[ix][iy] = change.new_cost.is_infinite();
            }
        }
        // Full replan on the updated map.
        self.plan_impl(start, goal)
    }

    /// Reference to the internal grid map.
    pub fn grid_map(&self) -> &GridMap {
        &self.grid_map
    }

    // ── Internal helpers ────────────────────────────────────────────────────

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
        self.config.heuristic_weight * (((x - gx).pow(2) + (y - gy).pow(2)) as f64).sqrt()
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

    fn plan_impl(&self, start: Point2D, goal: Point2D) -> RoboticsResult<Path2D> {
        let sx = self.grid_map.calc_x_index(start.x);
        let sy = self.grid_map.calc_y_index(start.y);
        let gx = self.grid_map.calc_x_index(goal.x);
        let gy = self.grid_map.calc_y_index(goal.y);

        self.ensure_valid(sx, sy, "Start")?;
        self.ensure_valid(gx, gy, "Goal")?;

        let start_idx = self.grid_map.calc_index(sx, sy);
        let goal_idx = self.grid_map.calc_index(gx, gy);

        // LPA* vertex tables: g and rhs default to ∞
        let mut g: HashMap<i32, f64> = HashMap::new();
        let mut rhs: HashMap<i32, f64> = HashMap::new();
        // Predecessor map for path reconstruction
        let mut pred: HashMap<i32, i32> = HashMap::new();

        // Initialize: rhs(start) = 0
        rhs.insert(start_idx, 0.0);

        // Priority queue — lazy-deletion style: an entry is stale if the
        // stored version in `in_open` differs from what we pop.
        let mut open: BinaryHeap<QueueEntry> = BinaryHeap::new();
        // `in_open` maps grid_idx → the *current* key we expect for it.
        // An entry is absent when the node is not in the open set.
        let mut in_open: HashMap<i32, Key> = HashMap::new();

        // Helpers that borrow the maps by reference (defined as plain fns below)
        let h = |x: i32, y: i32| self.heuristic(x, y, gx, gy);
        let get_g = |g: &HashMap<i32, f64>, idx: i32| *g.get(&idx).unwrap_or(&f64::INFINITY);
        let get_rhs = |rhs: &HashMap<i32, f64>, idx: i32| *rhs.get(&idx).unwrap_or(&f64::INFINITY);

        // Calculate key for vertex at grid_idx with grid coords (vx, vy)
        let calc_key =
            |g: &HashMap<i32, f64>, rhs: &HashMap<i32, f64>, idx: i32, vx: i32, vy: i32| {
                Key::new(get_g(g, idx), get_rhs(rhs, idx), h(vx, vy))
            };

        // Recover grid (x, y) from flat grid index — inverse of calc_index.
        // calc_index = (y - min_y_i32) * x_width + (x - min_x_i32)
        let idx_to_xy = |idx: i32| -> (i32, i32) {
            let min_x_i32 = self.grid_map.min_x as i32;
            let min_y_i32 = self.grid_map.min_y as i32;
            let local_x = idx % self.grid_map.x_width;
            let local_y = idx / self.grid_map.x_width;
            (local_x + min_x_i32, local_y + min_y_i32)
        };

        // Insert / update a vertex in the open set
        let enqueue = |open: &mut BinaryHeap<QueueEntry>,
                       in_open: &mut HashMap<i32, Key>,
                       idx: i32,
                       key: Key| {
            in_open.insert(idx, key);
            open.push(QueueEntry { key, grid_idx: idx });
        };

        // Seed the start node
        let sk = calc_key(&g, &rhs, start_idx, sx, sy);
        enqueue(&mut open, &mut in_open, start_idx, sk);

        // ── ComputeShortestPath ────────────────────────────────────────────
        // The loop peeks before consuming so it can check termination conditions
        // without removing the entry; a plain `while let` cannot express this.
        #[allow(clippy::while_let_loop)]
        loop {
            // Termination: queue empty, or top key ≥ key(goal) and goal is consistent
            let top_key = match open.peek() {
                Some(e) => e.key,
                None => break,
            };

            let goal_g = get_g(&g, goal_idx);
            let goal_rhs = get_rhs(&rhs, goal_idx);

            // goal consistent: g == rhs (handle infinity case explicitly)
            let goal_consistent =
                (goal_g.is_finite() && goal_rhs.is_finite() && (goal_g - goal_rhs).abs() < 1e-9)
                    || (goal_g.is_infinite() && goal_rhs.is_infinite());

            // Termination: top key is not strictly less than key(goal) in natural order,
            // AND goal is locally consistent.
            let goal_key = Key::new(goal_g, goal_rhs, 0.0);
            if goal_consistent && !top_key.less_than(&goal_key) {
                break;
            }

            // Pop smallest-key entry; skip stale entries
            let entry = open.pop().unwrap();
            let u_idx = entry.grid_idx;

            match in_open.get(&u_idx) {
                Some(&recorded) if recorded == entry.key => {}
                _ => continue, // stale
            }
            in_open.remove(&u_idx);

            let g_u = get_g(&g, u_idx);
            let rhs_u = get_rhs(&rhs, u_idx);
            let (u_x, u_y) = idx_to_xy(u_idx);

            if g_u > rhs_u {
                // Over-consistent: set g = rhs and propagate to successors
                g.insert(u_idx, rhs_u);

                for &(dx, dy, move_cost) in &self.motion {
                    if !self.grid_map.is_valid_offset(u_x, u_y, dx, dy) {
                        continue;
                    }
                    let nx = u_x + dx;
                    let ny = u_y + dy;
                    let n_idx = self.grid_map.calc_index(nx, ny);
                    let candidate = rhs_u + move_cost;

                    if candidate < get_rhs(&rhs, n_idx) {
                        rhs.insert(n_idx, candidate);
                        pred.insert(n_idx, u_idx);

                        // Insert or update in open set
                        let nk = calc_key(&g, &rhs, n_idx, nx, ny);
                        enqueue(&mut open, &mut in_open, n_idx, nk);
                    }
                }
            } else {
                // Under-consistent: raise g to ∞ and re-examine self and successors
                g.insert(u_idx, f64::INFINITY);

                // Re-examine self
                {
                    let new_g = f64::INFINITY;
                    let cur_rhs = get_rhs(&rhs, u_idx);
                    if (new_g - cur_rhs).abs() > 1e-9 {
                        let k = calc_key(&g, &rhs, u_idx, u_x, u_y);
                        enqueue(&mut open, &mut in_open, u_idx, k);
                    }
                }

                // Re-examine each successor
                for &(dx, dy, _) in &self.motion {
                    if !self.grid_map.is_valid_offset(u_x, u_y, dx, dy) {
                        continue;
                    }
                    let nx = u_x + dx;
                    let ny = u_y + dy;
                    let n_idx = self.grid_map.calc_index(nx, ny);

                    // Recompute rhs(n) as min over all predecessors of n
                    let mut best_rhs = f64::INFINITY;
                    let mut best_pred = None;
                    for &(pdx, pdy, pc) in &self.motion {
                        let px = nx - pdx;
                        let py = ny - pdy;
                        if !self.grid_map.is_valid_offset(px, py, pdx, pdy) {
                            continue;
                        }
                        let p_idx = self.grid_map.calc_index(px, py);
                        let candidate = get_g(&g, p_idx) + pc;
                        if candidate < best_rhs {
                            best_rhs = candidate;
                            best_pred = Some(p_idx);
                        }
                    }

                    rhs.insert(n_idx, best_rhs);
                    if let Some(p) = best_pred {
                        pred.insert(n_idx, p);
                    }

                    let g_n = get_g(&g, n_idx);
                    if (g_n - best_rhs).abs() > 1e-9
                        || (g_n.is_infinite() != best_rhs.is_infinite())
                    {
                        let k = calc_key(&g, &rhs, n_idx, nx, ny);
                        enqueue(&mut open, &mut in_open, n_idx, k);
                    }
                }
            }
        }

        // Check goal reachability
        let goal_rhs_final = get_rhs(&rhs, goal_idx);
        if goal_rhs_final.is_infinite() {
            return Err(RoboticsError::PlanningError("No path found".to_string()));
        }

        // Backtrack from goal to start via pred map
        self.build_path(start_idx, goal_idx, gx, gy, &pred)
    }

    fn build_path(
        &self,
        start_idx: i32,
        goal_idx: i32,
        gx: i32,
        gy: i32,
        pred: &HashMap<i32, i32>,
    ) -> RoboticsResult<Path2D> {
        let min_x_i32 = self.grid_map.min_x as i32;
        let min_y_i32 = self.grid_map.min_y as i32;

        let idx_to_world = |idx: i32| -> Point2D {
            let local_x = idx % self.grid_map.x_width;
            let local_y = idx / self.grid_map.x_width;
            let gx = local_x + min_x_i32;
            let gy = local_y + min_y_i32;
            Point2D::new(
                self.grid_map.calc_x_position(gx),
                self.grid_map.calc_y_position(gy),
            )
        };

        let mut points = Vec::new();
        points.push(Point2D::new(
            self.grid_map.calc_x_position(gx),
            self.grid_map.calc_y_position(gy),
        ));

        let mut current = goal_idx;
        let mut seen: HashMap<i32, ()> = HashMap::new();
        seen.insert(current, ());

        while current != start_idx {
            match pred.get(&current).copied() {
                Some(p) => {
                    if seen.insert(p, ()).is_none() {
                        points.push(idx_to_world(p));
                    } else {
                        return Err(RoboticsError::PlanningError(
                            "Cycle detected during path reconstruction".to_string(),
                        ));
                    }
                    current = p;
                }
                None => {
                    return Err(RoboticsError::PlanningError(
                        "Path reconstruction failed: predecessor missing".to_string(),
                    ))
                }
            }
        }

        points.reverse();
        Ok(Path2D::from_points(points))
    }
}

impl PathPlanner for LPAStarPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        self.plan_impl(start, goal)
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::a_star::AStarPlanner;

    fn simple_obstacles() -> (Vec<f64>, Vec<f64>) {
        let mut ox = Vec::new();
        let mut oy = Vec::new();
        // Boundary walls
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
        // Internal obstacle
        for i in 4..7 {
            ox.push(5.0);
            oy.push(i as f64);
        }
        (ox, oy)
    }

    #[test]
    fn test_lpa_star_finds_path() {
        let (ox, oy) = simple_obstacles();
        let planner = LPAStarPlanner::new(&ox, &oy, LPAStarConfig::default());

        let path = planner.plan(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0));
        assert!(path.is_ok(), "expected a path, got {:?}", path);
        let path = path.unwrap();
        assert!(!path.is_empty());

        // Start and end must be close to requested coordinates
        let first = path.points.first().unwrap();
        let last = path.points.last().unwrap();
        assert!((first.x - 2.0).abs() < 1.5 && (first.y - 2.0).abs() < 1.5);
        assert!((last.x - 8.0).abs() < 1.5 && (last.y - 8.0).abs() < 1.5);
    }

    #[test]
    fn test_lpa_star_matches_a_star_path_length() {
        let (ox, oy) = simple_obstacles();
        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(8.0, 8.0);

        let lpa = LPAStarPlanner::new(&ox, &oy, LPAStarConfig::default());
        let astar = AStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

        let lpa_path = lpa.plan(start, goal).unwrap();
        let astar_path = astar.plan(start, goal).unwrap();

        let diff = (lpa_path.total_length() - astar_path.total_length()).abs();
        assert!(
            diff < 1e-6,
            "LPA* length {} differs from A* length {} by {}",
            lpa_path.total_length(),
            astar_path.total_length(),
            diff
        );
    }

    #[test]
    fn test_config_defaults() {
        let cfg = LPAStarConfig::default();
        assert_eq!(cfg.resolution, 1.0);
        assert_eq!(cfg.robot_radius, 0.5);
        assert_eq!(cfg.heuristic_weight, 1.0);
        assert!(cfg.validate().is_ok());

        let bad = LPAStarConfig {
            heuristic_weight: 0.0,
            ..Default::default()
        };
        assert!(bad.validate().is_err());
    }

    #[test]
    fn test_plan_with_cost_change_replans() {
        let (ox, oy) = simple_obstacles();
        let mut planner = LPAStarPlanner::new(&ox, &oy, LPAStarConfig::default());

        let start = Point2D::new(2.0, 2.0);
        let goal = Point2D::new(8.0, 8.0);

        // First plan
        let path1 = planner.plan(start, goal).unwrap();
        assert!(!path1.is_empty());

        // Open up the internal obstacle (make cells traversable)
        let changes: Vec<EdgeChange> = (4..7)
            .map(|i| EdgeChange {
                x: 5.0,
                y: i as f64,
                new_cost: 1.0, // finite → not an obstacle
            })
            .collect();

        let path2 = planner
            .plan_with_cost_change(start, goal, &changes)
            .unwrap();
        assert!(!path2.is_empty());

        // After opening the wall the path should not be longer
        assert!(
            path2.total_length() <= path1.total_length() + 1e-6,
            "path after opening wall ({}) should not be longer than original ({})",
            path2.total_length(),
            path1.total_length()
        );
    }
}
