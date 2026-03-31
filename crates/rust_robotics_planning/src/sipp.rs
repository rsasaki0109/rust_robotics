//! Safe Interval Path Planning (SIPP)
//!
//! Plans paths in space-time by computing "safe intervals" at each grid cell
//! and searching over (cell, interval) pairs. This avoids expanding every
//! discrete timestep and efficiently handles dynamic obstacles with known
//! trajectories.
//!
//! Reference: Phillips & Likhachev, "SIPP: Safe Interval Path Planning for
//! Dynamic Environments", ICRA 2011.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use rust_robotics_core::{RoboticsError, RoboticsResult};

/// A half-open time interval `[start, end)`. `end == u64::MAX` means unbounded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Interval {
    pub start: u64,
    pub end: u64,
}

impl Interval {
    pub fn new(start: u64, end: u64) -> Self {
        Self { start, end }
    }

    /// An interval covering all time.
    pub fn infinite() -> Self {
        Self {
            start: 0,
            end: u64::MAX,
        }
    }

    pub fn contains(&self, t: u64) -> bool {
        t >= self.start && t < self.end
    }

    pub fn is_empty(&self) -> bool {
        self.start >= self.end
    }
}

/// A dynamic obstacle occupying a cell during a time interval.
#[derive(Debug, Clone)]
pub struct DynamicObstacle {
    pub x: i32,
    pub y: i32,
    /// Half-open interval during which the cell is blocked.
    pub interval: Interval,
}

/// Configuration for the SIPP planner.
#[derive(Debug, Clone)]
pub struct SippConfig {
    pub width: i32,
    pub height: i32,
    /// Static obstacle map: `true` means blocked.
    pub obstacle_map: Vec<Vec<bool>>,
    /// Dynamic obstacles with known trajectories.
    pub dynamic_obstacles: Vec<DynamicObstacle>,
    /// Whether to allow diagonal (8-connected) movement.
    pub allow_diagonal: bool,
}

/// A waypoint in the planned path, including the arrival time.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TimedWaypoint {
    pub x: i32,
    pub y: i32,
    pub t: u64,
}

/// Result of SIPP planning.
pub type SippPath = Vec<TimedWaypoint>;

// ---------------------------------------------------------------------------
// Internal types
// ---------------------------------------------------------------------------

/// Key for identifying a search state: grid cell + interval index at that cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct StateKey {
    x: i32,
    y: i32,
    interval_idx: usize,
}

/// Entry in the open-list priority queue.
struct OpenEntry {
    key: StateKey,
    f: u64,
}

impl Eq for OpenEntry {}
impl PartialEq for OpenEntry {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}
impl Ord for OpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // min-heap: reverse ordering
        other.f.cmp(&self.f)
    }
}
impl PartialOrd for OpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// ---------------------------------------------------------------------------
// Planner
// ---------------------------------------------------------------------------

/// Safe Interval Path Planning (SIPP) planner.
pub struct SippPlanner {
    width: i32,
    height: i32,
    obstacle_map: Vec<Vec<bool>>,
    /// For each cell (flattened by y * width + x), the sorted list of safe intervals.
    safe_intervals: Vec<Vec<Interval>>,
    motions: Vec<(i32, i32, u64)>,
}

impl SippPlanner {
    /// Build a new SIPP planner from a configuration.
    pub fn new(config: SippConfig) -> RoboticsResult<Self> {
        if config.width <= 0 || config.height <= 0 {
            return Err(RoboticsError::InvalidParameter(
                "width and height must be positive".to_string(),
            ));
        }
        if config.obstacle_map.len() != config.width as usize {
            return Err(RoboticsError::InvalidParameter(format!(
                "obstacle_map x-dimension ({}) must match width ({})",
                config.obstacle_map.len(),
                config.width
            )));
        }
        for col in &config.obstacle_map {
            if col.len() != config.height as usize {
                return Err(RoboticsError::InvalidParameter(
                    "obstacle_map y-dimension must match height".to_string(),
                ));
            }
        }

        let motions = if config.allow_diagonal {
            vec![
                (1, 0, 1),
                (0, 1, 1),
                (-1, 0, 1),
                (0, -1, 1),
                (1, 1, 1),
                (1, -1, 1),
                (-1, 1, 1),
                (-1, -1, 1),
            ]
        } else {
            vec![(1, 0, 1), (0, 1, 1), (-1, 0, 1), (0, -1, 1)]
        };

        let safe_intervals = Self::compute_safe_intervals(
            &config.obstacle_map,
            &config.dynamic_obstacles,
            config.width,
            config.height,
        );

        Ok(Self {
            width: config.width,
            height: config.height,
            obstacle_map: config.obstacle_map,
            safe_intervals,
            motions,
        })
    }

    /// Plan a timed path from `(sx, sy)` at time 0 to `(gx, gy)`.
    pub fn plan(&self, sx: i32, sy: i32, gx: i32, gy: i32) -> RoboticsResult<SippPath> {
        self.validate_pos(sx, sy, "start")?;
        self.validate_pos(gx, gy, "goal")?;

        if self.obstacle_map[sx as usize][sy as usize] {
            return Err(RoboticsError::PlanningError(
                "start is inside a static obstacle".to_string(),
            ));
        }
        if self.obstacle_map[gx as usize][gy as usize] {
            return Err(RoboticsError::PlanningError(
                "goal is inside a static obstacle".to_string(),
            ));
        }

        let start_intervals = &self.safe_intervals[self.cell_index(sx, sy)];
        let start_interval_idx = match start_intervals.iter().position(|iv| iv.contains(0)) {
            Some(idx) => idx,
            None => {
                return Err(RoboticsError::PlanningError(
                    "start cell is not safe at time 0".to_string(),
                ));
            }
        };

        let start_key = StateKey {
            x: sx,
            y: sy,
            interval_idx: start_interval_idx,
        };

        let h0 = Self::heuristic(sx, sy, gx, gy);

        let mut open = BinaryHeap::new();
        let mut best_g: HashMap<StateKey, u64> = HashMap::new();
        let mut parent_map: HashMap<StateKey, Option<StateKey>> = HashMap::new();

        best_g.insert(start_key, 0);
        parent_map.insert(start_key, None);
        open.push(OpenEntry {
            key: start_key,
            f: h0,
        });

        while let Some(current) = open.pop() {
            let g = match best_g.get(&current.key) {
                Some(&g) => g,
                None => continue,
            };

            // Stale entry check
            if current.f > g + Self::heuristic(current.key.x, current.key.y, gx, gy) {
                continue;
            }

            // Goal reached: we need the goal cell, any safe interval that contains g.
            if current.key.x == gx && current.key.y == gy {
                return Ok(Self::reconstruct_path(&parent_map, &best_g, current.key));
            }

            let current_interval = self.safe_intervals
                [self.cell_index(current.key.x, current.key.y)][current.key.interval_idx];

            // Expand move actions to neighboring cells
            for &(dx, dy, cost) in &self.motions {
                let nx = current.key.x + dx;
                let ny = current.key.y + dy;

                if !self.in_bounds(nx, ny) || self.obstacle_map[nx as usize][ny as usize] {
                    continue;
                }

                let arrival = g + cost;
                let neighbor_intervals = &self.safe_intervals[self.cell_index(nx, ny)];

                for (ni, niv) in neighbor_intervals.iter().enumerate() {
                    // The arrival time must fall within this safe interval.
                    if arrival >= niv.end {
                        continue;
                    }
                    // If arrival is before the interval starts, we could wait at current
                    // cell until niv.start - cost, then move.  But we must still be in
                    // the current safe interval at departure time.
                    let effective_arrival = if arrival >= niv.start {
                        arrival
                    } else {
                        // We need to wait at current cell until (niv.start - cost),
                        // then depart.  Check the current interval can hold us that long.
                        let depart = niv.start - cost;
                        if depart < g {
                            // Cannot depart before we arrived at current cell.
                            continue;
                        }
                        if depart >= current_interval.end {
                            // Waiting that long leaves the current safe interval.
                            continue;
                        }
                        niv.start
                    };

                    let nkey = StateKey {
                        x: nx,
                        y: ny,
                        interval_idx: ni,
                    };

                    if let Some(&prev_g) = best_g.get(&nkey) {
                        if effective_arrival >= prev_g {
                            continue;
                        }
                    }

                    best_g.insert(nkey, effective_arrival);
                    parent_map.insert(nkey, Some(current.key));
                    let f = effective_arrival + Self::heuristic(nx, ny, gx, gy);
                    open.push(OpenEntry { key: nkey, f });
                }
            }

            // Expand wait action: wait in the current cell and transition to the
            // *next* safe interval at the same cell (after an unsafe gap).
            let cell_intervals =
                &self.safe_intervals[self.cell_index(current.key.x, current.key.y)];
            if current.key.interval_idx + 1 < cell_intervals.len() {
                let next_idx = current.key.interval_idx + 1;
                let next_iv = cell_intervals[next_idx];
                // We can reach the next interval only if the current interval's end
                // touches the next interval's start (i.e., the gap is caused by a
                // dynamic obstacle passing through). The robot would need to leave
                // and come back, which is handled by move actions. But if we want to
                // explicitly model "wait through unsafe gap at same cell", SIPP does
                // NOT allow that -- the robot must be in a safe interval. So we skip
                // same-cell wait-through-gap. The wait action within the CURRENT
                // interval is already handled by the arrival-time logic above (the
                // robot can arrive at any time within its current interval and simply
                // wait).
                //
                // However, for completeness we do NOT add a same-cell transition here
                // because the robot cannot stay at a cell during an unsafe gap.
                let _ = (next_idx, next_iv); // suppress unused warnings
            }
        }

        Err(RoboticsError::PlanningError("no path found".to_string()))
    }

    /// Get the safe intervals for a particular cell.
    pub fn get_safe_intervals(&self, x: i32, y: i32) -> &[Interval] {
        &self.safe_intervals[self.cell_index(x, y)]
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    fn cell_index(&self, x: i32, y: i32) -> usize {
        (y as usize) * (self.width as usize) + (x as usize)
    }

    fn in_bounds(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < self.width && y < self.height
    }

    fn validate_pos(&self, x: i32, y: i32, label: &str) -> RoboticsResult<()> {
        if self.in_bounds(x, y) {
            Ok(())
        } else {
            Err(RoboticsError::InvalidParameter(format!(
                "{} ({}, {}) is out of bounds ({}x{})",
                label, x, y, self.width, self.height
            )))
        }
    }

    fn heuristic(x: i32, y: i32, gx: i32, gy: i32) -> u64 {
        // Chebyshev distance (consistent with 4- and 8-connected grids with unit cost).
        let dx = (x - gx).unsigned_abs() as u64;
        let dy = (y - gy).unsigned_abs() as u64;
        dx.max(dy)
    }

    /// Compute safe intervals for every cell.
    ///
    /// For cells that are static obstacles the list is empty (never safe).
    /// For cells without any dynamic obstacle the list is `[0, MAX)`.
    /// Otherwise, unsafe intervals from dynamic obstacles are merged and the
    /// complement within `[0, MAX)` gives the safe intervals.
    fn compute_safe_intervals(
        obstacle_map: &[Vec<bool>],
        dynamic_obstacles: &[DynamicObstacle],
        width: i32,
        height: i32,
    ) -> Vec<Vec<Interval>> {
        let n = (width as usize) * (height as usize);
        let mut unsafe_map: HashMap<usize, Vec<Interval>> = HashMap::new();

        for dob in dynamic_obstacles {
            if dob.x < 0 || dob.y < 0 || dob.x >= width || dob.y >= height {
                continue;
            }
            let idx = (dob.y as usize) * (width as usize) + (dob.x as usize);
            unsafe_map.entry(idx).or_default().push(dob.interval);
        }

        let mut safe = Vec::with_capacity(n);
        for y in 0..height {
            for x in 0..width {
                let idx = (y as usize) * (width as usize) + (x as usize);
                if obstacle_map[x as usize][y as usize] {
                    safe.push(Vec::new());
                } else if let Some(unsafes) = unsafe_map.get(&idx) {
                    safe.push(Self::complement_intervals(unsafes));
                } else {
                    safe.push(vec![Interval::infinite()]);
                }
            }
        }
        safe
    }

    /// Given a list of (possibly overlapping) unsafe intervals, return the
    /// sorted list of safe intervals within `[0, u64::MAX)`.
    fn complement_intervals(unsafe_intervals: &[Interval]) -> Vec<Interval> {
        if unsafe_intervals.is_empty() {
            return vec![Interval::infinite()];
        }

        // Merge overlapping unsafe intervals.
        let mut sorted: Vec<(u64, u64)> = unsafe_intervals
            .iter()
            .filter(|iv| !iv.is_empty())
            .map(|iv| (iv.start, iv.end))
            .collect();
        sorted.sort_by_key(|&(s, _)| s);

        let mut merged: Vec<(u64, u64)> = Vec::new();
        for (s, e) in sorted {
            if let Some(last) = merged.last_mut() {
                if s <= last.1 {
                    last.1 = last.1.max(e);
                    continue;
                }
            }
            merged.push((s, e));
        }

        // Build complement.
        let mut result = Vec::new();
        let mut cursor: u64 = 0;
        for (s, e) in &merged {
            if cursor < *s {
                result.push(Interval::new(cursor, *s));
            }
            cursor = *e;
        }
        if cursor < u64::MAX {
            result.push(Interval::new(cursor, u64::MAX));
        }
        result
    }

    fn reconstruct_path(
        parent_map: &HashMap<StateKey, Option<StateKey>>,
        best_g: &HashMap<StateKey, u64>,
        goal_key: StateKey,
    ) -> SippPath {
        let mut path = Vec::new();
        let mut current = Some(goal_key);

        while let Some(key) = current {
            let t = best_g[&key];
            path.push(TimedWaypoint {
                x: key.x,
                y: key.y,
                t,
            });
            current = parent_map.get(&key).copied().flatten();
        }

        path.reverse();
        path
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper to build an empty obstacle map of given size.
    fn empty_map(width: i32, height: i32) -> Vec<Vec<bool>> {
        vec![vec![false; height as usize]; width as usize]
    }

    /// Helper to build a config with no dynamic obstacles and 4-connected grid.
    fn simple_config(width: i32, height: i32, obstacles: &[(i32, i32)]) -> SippConfig {
        let mut map = empty_map(width, height);
        for &(x, y) in obstacles {
            map[x as usize][y as usize] = true;
        }
        SippConfig {
            width,
            height,
            obstacle_map: map,
            dynamic_obstacles: vec![],
            allow_diagonal: false,
        }
    }

    // -----------------------------------------------------------------------
    // Basic tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_straight_line_no_obstacles() {
        let config = simple_config(5, 5, &[]);
        let planner = SippPlanner::new(config).unwrap();
        let path = planner.plan(0, 0, 4, 0).unwrap();

        assert_eq!(path.len(), 5);
        assert_eq!(path.first().unwrap(), &TimedWaypoint { x: 0, y: 0, t: 0 });
        assert_eq!(path.last().unwrap(), &TimedWaypoint { x: 4, y: 0, t: 4 });
    }

    #[test]
    fn test_static_obstacle_detour() {
        // 5x5 grid, wall at x=2 from y=0..3, forcing a detour.
        let obstacles: Vec<(i32, i32)> = (0..3).map(|y| (2, y)).collect();
        let config = simple_config(5, 5, &obstacles);
        let planner = SippPlanner::new(config).unwrap();
        let path = planner.plan(0, 0, 4, 0).unwrap();

        // Must go around the wall. Path should exist and reach the goal.
        assert!(path.len() > 5, "path must detour around the wall");
        assert_eq!(path.first().unwrap().x, 0);
        assert_eq!(path.first().unwrap().y, 0);
        assert_eq!(path.last().unwrap().x, 4);
        assert_eq!(path.last().unwrap().y, 0);

        // No waypoint should be on a static obstacle.
        for wp in &path {
            assert!(
                !obstacles.contains(&(wp.x, wp.y)),
                "path must not pass through static obstacles"
            );
        }
    }

    #[test]
    fn test_static_only_behaves_like_astar() {
        // Without dynamic obstacles, SIPP path length should equal Manhattan
        // distance on an empty 4-connected grid.
        let config = simple_config(10, 10, &[]);
        let planner = SippPlanner::new(config).unwrap();
        let path = planner.plan(0, 0, 9, 9).unwrap();

        // Manhattan distance = 18, so path has 19 waypoints (including start).
        assert_eq!(path.len(), 19);
        assert_eq!(path.last().unwrap().t, 18);
    }

    // -----------------------------------------------------------------------
    // Dynamic obstacle tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_dynamic_obstacle_blocks_corridor() {
        // 5x1 corridor: (0,0) -> (4,0).
        // A dynamic obstacle blocks (2,0) during [2, 5).
        // Without the obstacle the shortest path arrives at (2,0) at t=2,
        // which is blocked. The planner must wait at (1,0) until t=4 then
        // move through (2,0) at t=5.
        let config = SippConfig {
            width: 5,
            height: 1,
            obstacle_map: empty_map(5, 1),
            dynamic_obstacles: vec![DynamicObstacle {
                x: 2,
                y: 0,
                interval: Interval::new(2, 5),
            }],
            allow_diagonal: false,
        };

        let planner = SippPlanner::new(config).unwrap();
        let path = planner.plan(0, 0, 4, 0).unwrap();

        // The robot reaches (2,0) no earlier than t=5.
        let at_2 = path.iter().find(|wp| wp.x == 2 && wp.y == 0).unwrap();
        assert!(
            at_2.t >= 5,
            "robot should not be at (2,0) before t=5, got t={}",
            at_2.t
        );

        assert_eq!(path.last().unwrap().x, 4);
        assert_eq!(path.last().unwrap().y, 0);
    }

    #[test]
    fn test_wait_and_go() {
        // 3x1 corridor: (0,0) -> (2,0).
        // Dynamic obstacle at (1,0) during [0, 10).
        // Robot must wait at (0,0) until t=9 then move to (1,0) at t=10.
        let config = SippConfig {
            width: 3,
            height: 1,
            obstacle_map: empty_map(3, 1),
            dynamic_obstacles: vec![DynamicObstacle {
                x: 1,
                y: 0,
                interval: Interval::new(0, 10),
            }],
            allow_diagonal: false,
        };

        let planner = SippPlanner::new(config).unwrap();
        let path = planner.plan(0, 0, 2, 0).unwrap();

        assert_eq!(path.first().unwrap(), &TimedWaypoint { x: 0, y: 0, t: 0 });

        let at_1 = path.iter().find(|wp| wp.x == 1).unwrap();
        assert!(
            at_1.t >= 10,
            "robot should reach (1,0) at t>=10, got t={}",
            at_1.t
        );

        assert_eq!(path.last().unwrap().x, 2);
        assert_eq!(path.last().unwrap().y, 0);
        assert!(path.last().unwrap().t >= 11);
    }

    #[test]
    fn test_multiple_dynamic_obstacles() {
        // 7x1 corridor.
        // Obstacle at (2,0) during [2,5) and at (4,0) during [4,8).
        let config = SippConfig {
            width: 7,
            height: 1,
            obstacle_map: empty_map(7, 1),
            dynamic_obstacles: vec![
                DynamicObstacle {
                    x: 2,
                    y: 0,
                    interval: Interval::new(2, 5),
                },
                DynamicObstacle {
                    x: 4,
                    y: 0,
                    interval: Interval::new(4, 8),
                },
            ],
            allow_diagonal: false,
        };

        let planner = SippPlanner::new(config).unwrap();
        let path = planner.plan(0, 0, 6, 0).unwrap();

        // Verify no waypoint violates dynamic obstacle constraints.
        for wp in &path {
            if wp.x == 2 && wp.y == 0 {
                assert!(wp.t < 2 || wp.t >= 5, "(2,0) blocked during [2,5)");
            }
            if wp.x == 4 && wp.y == 0 {
                assert!(wp.t < 4 || wp.t >= 8, "(4,0) blocked during [4,8)");
            }
        }
        assert_eq!(path.last().unwrap().x, 6);
    }

    // -----------------------------------------------------------------------
    // Safe interval computation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_safe_intervals_no_dynamic() {
        let config = simple_config(3, 3, &[]);
        let planner = SippPlanner::new(config).unwrap();
        let ivs = planner.get_safe_intervals(1, 1);
        assert_eq!(ivs.len(), 1);
        assert_eq!(ivs[0], Interval::infinite());
    }

    #[test]
    fn test_safe_intervals_with_dynamic() {
        let config = SippConfig {
            width: 3,
            height: 3,
            obstacle_map: empty_map(3, 3),
            dynamic_obstacles: vec![DynamicObstacle {
                x: 1,
                y: 1,
                interval: Interval::new(5, 10),
            }],
            allow_diagonal: false,
        };
        let planner = SippPlanner::new(config).unwrap();
        let ivs = planner.get_safe_intervals(1, 1);
        assert_eq!(ivs.len(), 2);
        assert_eq!(ivs[0], Interval::new(0, 5));
        assert_eq!(ivs[1], Interval::new(10, u64::MAX));
    }

    #[test]
    fn test_safe_intervals_static_obstacle() {
        let config = simple_config(3, 3, &[(1, 1)]);
        let planner = SippPlanner::new(config).unwrap();
        let ivs = planner.get_safe_intervals(1, 1);
        assert!(ivs.is_empty(), "static obstacle has no safe intervals");
    }

    // -----------------------------------------------------------------------
    // Edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_start_equals_goal() {
        let config = simple_config(3, 3, &[]);
        let planner = SippPlanner::new(config).unwrap();
        let path = planner.plan(1, 1, 1, 1).unwrap();
        assert_eq!(path.len(), 1);
        assert_eq!(path[0], TimedWaypoint { x: 1, y: 1, t: 0 });
    }

    #[test]
    fn test_no_path_due_to_static_wall() {
        // Build a wall that completely separates start from goal.
        let obstacles: Vec<(i32, i32)> = (0..5).map(|y| (2, y)).collect();
        let config = simple_config(5, 5, &obstacles);
        let planner = SippPlanner::new(config).unwrap();
        let result = planner.plan(0, 0, 4, 0);
        assert!(result.is_err());
    }

    #[test]
    fn test_diagonal_movement() {
        let config = SippConfig {
            width: 5,
            height: 5,
            obstacle_map: empty_map(5, 5),
            dynamic_obstacles: vec![],
            allow_diagonal: true,
        };
        let planner = SippPlanner::new(config).unwrap();
        let path = planner.plan(0, 0, 4, 4).unwrap();

        // With diagonal movement, path length should be 5 (Chebyshev distance + 1).
        assert_eq!(path.len(), 5);
        assert_eq!(path.last().unwrap().t, 4);
    }

    #[test]
    fn test_invalid_start_goal() {
        let config = simple_config(5, 5, &[]);
        let planner = SippPlanner::new(config).unwrap();

        assert!(planner.plan(-1, 0, 4, 0).is_err());
        assert!(planner.plan(0, 0, 5, 0).is_err());
    }
}
