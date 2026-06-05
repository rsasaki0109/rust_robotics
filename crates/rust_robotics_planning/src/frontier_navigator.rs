//! Long Range Navigator-lite: affordance-scored frontier navigation.
//!
//! This reproduces the decision core of long-range navigation past the local
//! map: a robot with a limited, occlusion-aware sensor cannot see a distant
//! goal, so it repeatedly picks the *frontier* (a known-free cell bordering
//! unknown space) that best advances toward the goal, then hands off to a local
//! planner that drives there over the known-free map. As the robot moves it
//! reveals more of the world, re-evaluates frontiers, and eventually sees a path
//! to the goal.
//!
//! - Sensing is occlusion-aware: cells are revealed only along a clear line of
//!   sight within the sensor range, so obstacles cast unknown shadows whose
//!   edges become frontiers.
//! - Each frontier is scored by an affordance combining goal progress, the
//!   known-free travel cost to reach it, whether it is in direct line of sight,
//!   and how much unknown space it borders (information gain).
//! - The local-planner handoff is a Dijkstra distance field over the known-free
//!   map: the robot follows its gradient toward the chosen frontier for a
//!   bounded step budget before re-sensing.
//!
//! Everything is deterministic for a fixed world and configuration.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, VecDeque};

use rust_robotics_core::{RoboticsError, RoboticsResult};

const SQRT2: f64 = std::f64::consts::SQRT_2;

/// Known-map cell state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Knowledge {
    Unknown,
    Free,
    Occupied,
}

/// A static occupancy world the robot explores.
#[derive(Debug, Clone, PartialEq)]
pub struct FrontierNavWorld {
    pub width: i32,
    pub height: i32,
    /// Ground-truth occupancy indexed `[x][y]`.
    pub occupied: Vec<Vec<bool>>,
    pub start: (i32, i32),
    pub goal: (i32, i32),
}

impl FrontierNavWorld {
    /// Empty (all-free) world of the given size.
    pub fn new(
        width: i32,
        height: i32,
        start: (i32, i32),
        goal: (i32, i32),
    ) -> RoboticsResult<Self> {
        if width <= 0 || height <= 0 {
            return Err(RoboticsError::InvalidParameter(
                "frontier-nav world dimensions must be positive".to_string(),
            ));
        }
        Ok(Self {
            width,
            height,
            occupied: vec![vec![false; height as usize]; width as usize],
            start,
            goal,
        })
    }

    fn in_bounds(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < self.width && y < self.height
    }

    fn is_occupied(&self, x: i32, y: i32) -> bool {
        !self.in_bounds(x, y) || self.occupied[x as usize][y as usize]
    }

    /// Set a rectangular block of cells occupied (clamped to bounds).
    pub fn fill_rect(&mut self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) {
        for x in min_x.max(0)..=max_x.min(self.width - 1) {
            for y in min_y.max(0)..=max_y.min(self.height - 1) {
                self.occupied[x as usize][y as usize] = true;
            }
        }
    }
}

/// One frontier selection, retained for inspection and rendering.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrontierChoice {
    pub robot: (i32, i32),
    pub frontier: (i32, i32),
    pub score: f64,
    pub line_of_sight: bool,
}

/// Configuration for the frontier navigator.
#[derive(Debug, Clone, PartialEq)]
pub struct FrontierNavConfig {
    /// Sensor range in cells.
    pub sensor_range: f64,
    /// Cells the robot moves toward a frontier before re-sensing.
    pub step_budget: usize,
    /// Maximum frontier-selection iterations.
    pub max_iterations: usize,
    /// Weight on goal progress from the frontier.
    pub goal_weight: f64,
    /// Weight on the known-free travel cost to the frontier.
    pub cost_weight: f64,
    /// Bonus for a frontier in direct line of sight.
    pub line_of_sight_weight: f64,
    /// Weight on bordered unknown space (information gain).
    pub openness_weight: f64,
}

impl Default for FrontierNavConfig {
    fn default() -> Self {
        Self {
            sensor_range: 5.0,
            step_budget: 3,
            max_iterations: 200,
            goal_weight: 1.0,
            cost_weight: 0.35,
            line_of_sight_weight: 1.5,
            openness_weight: 0.15,
        }
    }
}

/// Result of a frontier-navigation rollout.
#[derive(Debug, Clone, PartialEq)]
pub struct FrontierNavReport {
    pub path: Vec<(i32, i32)>,
    pub reached_goal: bool,
    pub path_length: f64,
    pub frontier_selections: usize,
    pub iterations: usize,
    pub known_free_cells: usize,
    pub selected_frontiers: Vec<FrontierChoice>,
    /// Final known map indexed `[x][y]`.
    pub known: Vec<Vec<Knowledge>>,
}

/// Drive the affordance-scored frontier navigator across the world.
pub fn simulate_frontier_navigation(
    world: &FrontierNavWorld,
    config: &FrontierNavConfig,
) -> RoboticsResult<FrontierNavReport> {
    validate(world, config)?;

    let (w, h) = (world.width, world.height);
    let mut known = vec![vec![Knowledge::Unknown; h as usize]; w as usize];
    let mut robot = world.start;
    let mut path = vec![robot];
    let mut path_length = 0.0;
    let mut selected_frontiers: Vec<FrontierChoice> = Vec::new();
    let mut reached_goal = false;
    let mut iterations = 0;

    sense(world, &mut known, robot, config.sensor_range);

    for _ in 0..config.max_iterations {
        iterations += 1;

        // If the goal is known free and reachable, hand off to the local planner.
        if known[world.goal.0 as usize][world.goal.1 as usize] == Knowledge::Free {
            if let Some(dist) = dijkstra_from(&known, world.goal) {
                if dist[robot.0 as usize][robot.1 as usize].is_finite() {
                    follow_gradient(
                        &known,
                        &dist,
                        &mut robot,
                        world.goal,
                        usize::MAX,
                        &mut path,
                        &mut path_length,
                    );
                    if robot == world.goal {
                        reached_goal = true;
                        break;
                    }
                }
            }
        }

        let frontiers = compute_frontiers(&known);
        if frontiers.is_empty() {
            break;
        }

        // Travel-cost field from the robot over known-free cells.
        let reach = match dijkstra_from(&known, robot) {
            Some(field) => field,
            None => break,
        };

        let goal_distance = euclid(robot, world.goal);
        let max_openness = frontiers
            .iter()
            .map(|f| f.openness)
            .max()
            .unwrap_or(1)
            .max(1) as f64;

        let mut best: Option<(f64, &FrontierCluster)> = None;
        for frontier in &frontiers {
            let reach_cost = reach[frontier.rep.0 as usize][frontier.rep.1 as usize];
            if !reach_cost.is_finite() {
                continue;
            }
            let los = line_of_sight(&known, robot, frontier.rep);
            let goal_gain = goal_distance - euclid(frontier.rep, world.goal);
            let openness = frontier.openness as f64 / max_openness;
            let score = config.goal_weight * goal_gain - config.cost_weight * reach_cost
                + config.line_of_sight_weight * if los { 1.0 } else { 0.0 }
                + config.openness_weight * openness;
            let better = match best {
                None => true,
                Some((best_score, best_cluster)) => {
                    score > best_score || (score == best_score && frontier.rep < best_cluster.rep)
                }
            };
            if better {
                best = Some((score, frontier));
            }
        }

        let Some((score, frontier)) = best else { break };
        let target = frontier.rep;
        selected_frontiers.push(FrontierChoice {
            robot,
            frontier: target,
            score,
            line_of_sight: line_of_sight(&known, robot, target),
        });

        // Local-planner handoff: move toward the frontier over known-free cells.
        let field = match dijkstra_from(&known, target) {
            Some(field) => field,
            None => break,
        };
        let before = robot;
        follow_gradient(
            &known,
            &field,
            &mut robot,
            target,
            config.step_budget,
            &mut path,
            &mut path_length,
        );
        if robot == before {
            break; // wedged: no progress possible.
        }
        sense(world, &mut known, robot, config.sensor_range);
    }

    let known_free_cells = known
        .iter()
        .flatten()
        .filter(|c| **c == Knowledge::Free)
        .count();

    Ok(FrontierNavReport {
        path,
        reached_goal,
        path_length,
        frontier_selections: selected_frontiers.len(),
        iterations,
        known_free_cells,
        selected_frontiers,
        known,
    })
}

fn validate(world: &FrontierNavWorld, config: &FrontierNavConfig) -> RoboticsResult<()> {
    if !world.in_bounds(world.start.0, world.start.1)
        || world.is_occupied(world.start.0, world.start.1)
    {
        return Err(RoboticsError::InvalidParameter(
            "frontier-nav start must be a free in-bounds cell".to_string(),
        ));
    }
    if !world.in_bounds(world.goal.0, world.goal.1) || world.is_occupied(world.goal.0, world.goal.1)
    {
        return Err(RoboticsError::InvalidParameter(
            "frontier-nav goal must be a free in-bounds cell".to_string(),
        ));
    }
    if !config.sensor_range.is_finite() || config.sensor_range <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "frontier-nav sensor range must be finite and positive".to_string(),
        ));
    }
    if config.step_budget == 0 || config.max_iterations == 0 {
        return Err(RoboticsError::InvalidParameter(
            "frontier-nav step budget and iterations must be positive".to_string(),
        ));
    }
    Ok(())
}

/// A cluster of contiguous frontier cells.
#[derive(Debug, Clone, PartialEq)]
struct FrontierCluster {
    rep: (i32, i32),
    openness: usize,
}

/// Reveal cells within `range` of `from` along a clear line of sight.
fn sense(world: &FrontierNavWorld, known: &mut [Vec<Knowledge>], from: (i32, i32), range: f64) {
    let r = range.ceil() as i32;
    for dx in -r..=r {
        for dy in -r..=r {
            let x = from.0 + dx;
            let y = from.1 + dy;
            if !world.in_bounds(x, y) {
                continue;
            }
            if ((dx * dx + dy * dy) as f64).sqrt() > range {
                continue;
            }
            if visible(world, from, (x, y)) {
                known[x as usize][y as usize] = if world.is_occupied(x, y) {
                    Knowledge::Occupied
                } else {
                    Knowledge::Free
                };
            }
        }
    }
}

/// Whether `target` is visible from `from` (no occupied cell strictly between).
fn visible(world: &FrontierNavWorld, from: (i32, i32), target: (i32, i32)) -> bool {
    let steps = (((target.0 - from.0).abs()).max((target.1 - from.1).abs()) * 3).max(1);
    for i in 1..=steps {
        let t = i as f64 / steps as f64;
        let x = (from.0 as f64 + t * (target.0 - from.0) as f64).round() as i32;
        let y = (from.1 as f64 + t * (target.1 - from.1) as f64).round() as i32;
        if (x, y) == target {
            return true;
        }
        if world.is_occupied(x, y) {
            return false; // an earlier occupied cell blocks the ray.
        }
    }
    true
}

/// Line of sight over the *known* map (blocked by known-occupied cells).
fn line_of_sight(known: &[Vec<Knowledge>], from: (i32, i32), target: (i32, i32)) -> bool {
    let steps = (((target.0 - from.0).abs()).max((target.1 - from.1).abs()) * 3).max(1);
    for i in 1..=steps {
        let t = i as f64 / steps as f64;
        let x = (from.0 as f64 + t * (target.0 - from.0) as f64).round() as i32;
        let y = (from.1 as f64 + t * (target.1 - from.1) as f64).round() as i32;
        if (x, y) == target {
            return true;
        }
        if known[x as usize][y as usize] == Knowledge::Occupied {
            return false;
        }
    }
    true
}

/// Frontier cells: known-free cells bordering unknown space, clustered.
fn compute_frontiers(known: &[Vec<Knowledge>]) -> Vec<FrontierCluster> {
    let w = known.len() as i32;
    let h = known[0].len() as i32;
    let is_free = |x: i32, y: i32| known[x as usize][y as usize] == Knowledge::Free;
    let unknown_neighbors = |x: i32, y: i32| {
        let mut count = 0;
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let nx = x + dx;
            let ny = y + dy;
            if nx >= 0
                && ny >= 0
                && nx < w
                && ny < h
                && known[nx as usize][ny as usize] == Knowledge::Unknown
            {
                count += 1;
            }
        }
        count
    };

    let mut is_frontier = vec![vec![false; h as usize]; w as usize];
    for x in 0..w {
        for y in 0..h {
            if is_free(x, y) && unknown_neighbors(x, y) > 0 {
                is_frontier[x as usize][y as usize] = true;
            }
        }
    }

    // Cluster frontier cells with 8-connected BFS.
    let mut visited = vec![vec![false; h as usize]; w as usize];
    let mut clusters = Vec::new();
    for x in 0..w {
        for y in 0..h {
            if !is_frontier[x as usize][y as usize] || visited[x as usize][y as usize] {
                continue;
            }
            let mut queue = VecDeque::new();
            queue.push_back((x, y));
            visited[x as usize][y as usize] = true;
            let mut members = Vec::new();
            let mut openness = 0;
            while let Some((cx, cy)) = queue.pop_front() {
                members.push((cx, cy));
                openness += unknown_neighbors(cx, cy);
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        let nx = cx + dx;
                        let ny = cy + dy;
                        if nx >= 0
                            && ny >= 0
                            && nx < w
                            && ny < h
                            && is_frontier[nx as usize][ny as usize]
                            && !visited[nx as usize][ny as usize]
                        {
                            visited[nx as usize][ny as usize] = true;
                            queue.push_back((nx, ny));
                        }
                    }
                }
            }
            // Representative: member closest to the cluster centroid.
            let cx = members.iter().map(|m| m.0).sum::<i32>() as f64 / members.len() as f64;
            let cy = members.iter().map(|m| m.1).sum::<i32>() as f64 / members.len() as f64;
            let rep = *members
                .iter()
                .min_by(|a, b| {
                    let da = (a.0 as f64 - cx).powi(2) + (a.1 as f64 - cy).powi(2);
                    let db = (b.0 as f64 - cx).powi(2) + (b.1 as f64 - cy).powi(2);
                    da.partial_cmp(&db)
                        .unwrap_or(Ordering::Equal)
                        .then_with(|| a.cmp(b))
                })
                .unwrap();
            clusters.push(FrontierCluster { rep, openness });
        }
    }
    clusters
}

/// Dijkstra distance field from `source` over known-free cells (8-connected).
fn dijkstra_from(known: &[Vec<Knowledge>], source: (i32, i32)) -> Option<Vec<Vec<f64>>> {
    let w = known.len() as i32;
    let h = known[0].len() as i32;
    if known[source.0 as usize][source.1 as usize] != Knowledge::Free {
        return None;
    }
    let mut dist = vec![vec![f64::INFINITY; h as usize]; w as usize];
    dist[source.0 as usize][source.1 as usize] = 0.0;
    let mut heap = BinaryHeap::new();
    heap.push(DijkstraNode {
        cost: 0.0,
        cell: source,
    });
    while let Some(DijkstraNode { cost, cell }) = heap.pop() {
        if cost > dist[cell.0 as usize][cell.1 as usize] {
            continue;
        }
        for dx in -1..=1 {
            for dy in -1..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let nx = cell.0 + dx;
                let ny = cell.1 + dy;
                if nx < 0 || ny < 0 || nx >= w || ny >= h {
                    continue;
                }
                if known[nx as usize][ny as usize] != Knowledge::Free {
                    continue;
                }
                let step = if dx != 0 && dy != 0 { SQRT2 } else { 1.0 };
                let next = cost + step;
                if next < dist[nx as usize][ny as usize] {
                    dist[nx as usize][ny as usize] = next;
                    heap.push(DijkstraNode {
                        cost: next,
                        cell: (nx, ny),
                    });
                }
            }
        }
    }
    Some(dist)
}

/// Step the robot down a distance field toward `target` for up to `budget` cells.
fn follow_gradient(
    known: &[Vec<Knowledge>],
    field: &[Vec<f64>],
    robot: &mut (i32, i32),
    target: (i32, i32),
    budget: usize,
    path: &mut Vec<(i32, i32)>,
    path_length: &mut f64,
) {
    let w = known.len() as i32;
    let h = known[0].len() as i32;
    for _ in 0..budget {
        if *robot == target {
            break;
        }
        let here = field[robot.0 as usize][robot.1 as usize];
        let mut best: Option<(f64, (i32, i32), f64)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let nx = robot.0 + dx;
                let ny = robot.1 + dy;
                if nx < 0 || ny < 0 || nx >= w || ny >= h {
                    continue;
                }
                if known[nx as usize][ny as usize] != Knowledge::Free {
                    continue;
                }
                let d = field[nx as usize][ny as usize];
                if !d.is_finite() || d >= here {
                    continue;
                }
                let step = if dx != 0 && dy != 0 { SQRT2 } else { 1.0 };
                let better = match best {
                    None => true,
                    Some((best_d, best_cell, _)) => {
                        d < best_d || (d == best_d && (nx, ny) < best_cell)
                    }
                };
                if better {
                    best = Some((d, (nx, ny), step));
                }
            }
        }
        let Some((_, next, step)) = best else { break };
        *robot = next;
        path.push(next);
        *path_length += step;
    }
}

#[derive(Debug, Clone, Copy)]
struct DijkstraNode {
    cost: f64,
    cell: (i32, i32),
}

impl PartialEq for DijkstraNode {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}
impl Eq for DijkstraNode {}
impl PartialOrd for DijkstraNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for DijkstraNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap on cost; tie-break by cell for determinism.
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
            .then_with(|| other.cell.cmp(&self.cell))
    }
}

fn euclid(a: (i32, i32), b: (i32, i32)) -> f64 {
    let dx = (a.0 - b.0) as f64;
    let dy = (a.1 - b.1) as f64;
    (dx * dx + dy * dy).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A world with a wall between start and goal, leaving a gap at the top.
    fn walled_world() -> FrontierNavWorld {
        let mut world = FrontierNavWorld::new(24, 16, (1, 8), (22, 8)).unwrap();
        // Vertical wall at x = 12 from y = 0 up to y = 12 (gap above y = 12).
        world.fill_rect(12, 13, 0, 12);
        world
    }

    #[test]
    fn reaches_goal_around_occlusion() {
        let report =
            simulate_frontier_navigation(&walled_world(), &FrontierNavConfig::default()).unwrap();
        assert!(report.reached_goal, "did not reach goal");
        assert!(*report.path.last().unwrap() == (22, 8));
        // Had to choose at least one frontier to get around the wall.
        assert!(report.frontier_selections >= 1);
    }

    #[test]
    fn open_world_reaches_goal() {
        let world = FrontierNavWorld::new(20, 12, (1, 6), (18, 6)).unwrap();
        let report = simulate_frontier_navigation(&world, &FrontierNavConfig::default()).unwrap();
        assert!(report.reached_goal);
    }

    #[test]
    fn is_deterministic() {
        let world = walled_world();
        let config = FrontierNavConfig::default();
        let first = simulate_frontier_navigation(&world, &config).unwrap();
        let second = simulate_frontier_navigation(&world, &config).unwrap();
        assert_eq!(first.path, second.path);
        assert_eq!(first.frontier_selections, second.frontier_selections);
    }

    #[test]
    fn sensing_is_occlusion_aware() {
        // A short wall at x = 4 within sensor range of the robot at (1, 8).
        let mut world = FrontierNavWorld::new(24, 16, (1, 8), (22, 8)).unwrap();
        world.fill_rect(4, 4, 5, 11);
        let mut known = vec![vec![Knowledge::Unknown; 16]; 24];
        sense(&world, &mut known, (1, 8), 6.0);
        // A near free cell is revealed.
        assert_eq!(known[3][8], Knowledge::Free);
        // The near wall face is seen as occupied.
        assert_eq!(known[4][8], Knowledge::Occupied);
        // A cell just behind the wall (within range) stays unknown (occluded).
        assert_eq!(known[6][8], Knowledge::Unknown);
    }

    #[test]
    fn rejects_blocked_start() {
        let mut world = FrontierNavWorld::new(10, 10, (1, 1), (8, 8)).unwrap();
        world.fill_rect(1, 1, 1, 1);
        assert!(simulate_frontier_navigation(&world, &FrontierNavConfig::default()).is_err());
    }
}
