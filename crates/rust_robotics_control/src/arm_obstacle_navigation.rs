#![allow(dead_code, clippy::new_without_default)]

//! Arm path planning with obstacle avoidance in joint space.
//!
//! Builds an occupancy grid in joint space by checking arm-obstacle collisions,
//! then uses A\* search on a toroidal grid to find a collision-free path.
//!
//! Reference: PythonRobotics ArmNavigation/arm\_obstacle\_navigation

use std::f64::consts::PI;

/// Default grid resolution (discretization of each joint angle axis).
const DEFAULT_GRID_SIZE: usize = 100;

/// Minimal 2-joint arm for collision checking in the occupancy grid.
///
/// This is a lightweight version focused on forward kinematics for the
/// obstacle navigation use case. For the full N-link arm with inverse
/// kinematics, see `n_joint_arm_control::NLinkArm`.
#[derive(Debug, Clone)]
struct Arm2Link {
    link_lengths: [f64; 2],
    points: [[f64; 2]; 3],
}

impl Arm2Link {
    fn new(link_lengths: [f64; 2]) -> Self {
        let mut arm = Arm2Link {
            link_lengths,
            points: [[0.0; 2]; 3],
        };
        arm.update_joints(0.0, 0.0);
        arm
    }

    fn update_joints(&mut self, theta1: f64, theta2: f64) {
        self.points[0] = [0.0, 0.0];
        self.points[1] = [
            self.link_lengths[0] * theta1.cos(),
            self.link_lengths[0] * theta1.sin(),
        ];
        let cumulative = theta1 + theta2;
        self.points[2] = [
            self.points[1][0] + self.link_lengths[1] * cumulative.cos(),
            self.points[1][1] + self.link_lengths[1] * cumulative.sin(),
        ];
    }
}

/// A circular obstacle defined by center (x, y) and radius.
#[derive(Debug, Clone, Copy)]
pub struct CircleObstacle {
    pub x: f64,
    pub y: f64,
    pub radius: f64,
}

impl CircleObstacle {
    pub fn new(x: f64, y: f64, radius: f64) -> Self {
        CircleObstacle { x, y, radius }
    }
}

/// A grid cell coordinate in joint space.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct GridCell {
    pub i: usize,
    pub j: usize,
}

/// Result of an A\* path search in joint space.
#[derive(Debug, Clone)]
pub struct ArmPath {
    /// Sequence of grid cells from start to goal.
    pub route_cells: Vec<GridCell>,
    /// Corresponding joint angle pairs \[theta1, theta2\] for each cell.
    pub joint_angles: Vec<[f64; 2]>,
}

/// Occupancy grid for a 2-joint arm in joint space.
///
/// Each cell `(i, j)` represents a pair of joint angles and is marked
/// as occupied (true) if the arm configuration collides with any obstacle.
#[derive(Debug, Clone)]
pub struct JointSpaceGrid {
    /// Grid size along each axis.
    pub size: usize,
    /// Flattened occupancy data (row-major): `grid[i * size + j]`.
    pub data: Vec<bool>,
}

impl JointSpaceGrid {
    /// Creates a new empty grid.
    pub fn new(size: usize) -> Self {
        JointSpaceGrid {
            size,
            data: vec![false; size * size],
        }
    }

    /// Returns the occupancy value at cell (i, j).
    pub fn get(&self, i: usize, j: usize) -> bool {
        self.data[i * self.size + j]
    }

    /// Sets the occupancy value at cell (i, j).
    pub fn set(&mut self, i: usize, j: usize, val: bool) {
        self.data[i * self.size + j] = val;
    }

    /// Converts a grid index to a joint angle in \[-pi, pi\).
    pub fn index_to_angle(&self, idx: usize) -> f64 {
        2.0 * PI * idx as f64 / self.size as f64 - PI
    }
}

/// Determines whether a line segment intersects a circle.
///
/// The line segment is defined by endpoints `a` and `b`.
/// The circle is defined by center `c` and `radius`.
///
/// Uses projection of circle center onto line segment to find closest point.
pub fn detect_collision(a: &[f64; 2], b: &[f64; 2], obstacle: &CircleObstacle) -> bool {
    let line_vec = [b[0] - a[0], b[1] - a[1]];
    let line_mag = (line_vec[0] * line_vec[0] + line_vec[1] * line_vec[1]).sqrt();
    if line_mag < 1e-12 {
        // Degenerate segment: just check point distance
        let dx = a[0] - obstacle.x;
        let dy = a[1] - obstacle.y;
        return (dx * dx + dy * dy).sqrt() <= obstacle.radius;
    }

    let line_unit = [line_vec[0] / line_mag, line_vec[1] / line_mag];
    let circle_vec = [obstacle.x - a[0], obstacle.y - a[1]];
    let proj = circle_vec[0] * line_unit[0] + circle_vec[1] * line_unit[1];

    let closest = if proj <= 0.0 {
        *a
    } else if proj >= line_mag {
        *b
    } else {
        [a[0] + line_unit[0] * proj, a[1] + line_unit[1] * proj]
    };

    let dx = closest[0] - obstacle.x;
    let dy = closest[1] - obstacle.y;
    (dx * dx + dy * dy).sqrt() <= obstacle.radius
}

/// Builds the joint-space occupancy grid for a 2-joint arm.
///
/// Iterates over all discretized joint angle combinations, computes the arm
/// configuration, and checks for collision with each obstacle.
pub fn build_occupancy_grid(
    link_lengths: &[f64; 2],
    obstacles: &[CircleObstacle],
    grid_size: usize,
) -> JointSpaceGrid {
    let mut grid = JointSpaceGrid::new(grid_size);
    let mut arm = Arm2Link::new(*link_lengths);

    for i in 0..grid_size {
        let theta1 = grid.index_to_angle(i);
        for j in 0..grid_size {
            let theta2 = grid.index_to_angle(j);
            arm.update_joints(theta1, theta2);

            let mut collision = false;
            // Check each link segment against each obstacle
            for k in 0..2 {
                if collision {
                    break;
                }
                for obs in obstacles {
                    if detect_collision(&arm.points[k], &arm.points[k + 1], obs) {
                        collision = true;
                        break;
                    }
                }
            }
            grid.set(i, j, collision);
        }
    }
    grid
}

/// Builds occupancy grid with default grid size.
pub fn build_occupancy_grid_default(
    link_lengths: &[f64; 2],
    obstacles: &[CircleObstacle],
) -> JointSpaceGrid {
    build_occupancy_grid(link_lengths, obstacles, DEFAULT_GRID_SIZE)
}

/// Finds the 4-connected neighbors on a toroidal grid.
fn find_neighbors(i: usize, j: usize, m: usize) -> [(usize, usize); 4] {
    let up = if i == 0 { m - 1 } else { i - 1 };
    let down = if i + 1 >= m { 0 } else { i + 1 };
    let left = if j == 0 { m - 1 } else { j - 1 };
    let right = if j + 1 >= m { 0 } else { j + 1 };
    [(up, j), (down, j), (i, left), (i, right)]
}

/// Computes the toroidal Manhattan heuristic for A\* on a toroidal grid.
fn calc_heuristic_map(m: usize, goal: GridCell) -> Vec<Vec<f64>> {
    let mut hmap = vec![vec![0.0f64; m]; m];
    for (i, hmap_row) in hmap.iter_mut().enumerate() {
        for (j, hmap_val) in hmap_row.iter_mut().enumerate() {
            let di = {
                let d = (i as isize - goal.i as isize).unsigned_abs();
                d.min(m - d)
            };
            let dj = {
                let d = (j as isize - goal.j as isize).unsigned_abs();
                d.min(m - d)
            };
            *hmap_val = (di + dj) as f64;
        }
    }
    hmap
}

/// Performs A\* search on a toroidal joint-space grid.
///
/// The grid wraps around in both dimensions (since joint angles are periodic).
///
/// Returns `Some(ArmPath)` if a route is found, `None` otherwise.
pub fn astar_torus(grid: &JointSpaceGrid, start: GridCell, goal: GridCell) -> Option<ArmPath> {
    let m = grid.size;

    // Cell states: 0 = free, 1 = obstacle, 2 = explored, 3 = frontier, 4 = start, 5 = goal
    let mut cell_state = vec![vec![0u8; m]; m];
    for (i, state_row) in cell_state.iter_mut().enumerate() {
        for (j, state_val) in state_row.iter_mut().enumerate() {
            if grid.get(i, j) {
                *state_val = 1; // obstacle
            }
        }
    }

    // Check start and goal are not in obstacles
    if cell_state[start.i][start.j] == 1 || cell_state[goal.i][goal.j] == 1 {
        return None;
    }

    cell_state[start.i][start.j] = 4;
    cell_state[goal.i][goal.j] = 5;

    let heuristic_map = calc_heuristic_map(m, goal);

    // f-score map (heuristic + distance) for explored nodes
    let mut f_score = vec![vec![f64::INFINITY; m]; m];
    let mut g_score = vec![vec![f64::INFINITY; m]; m];
    let mut parent: Vec<Vec<Option<(usize, usize)>>> = vec![vec![None; m]; m];

    f_score[start.i][start.j] = heuristic_map[start.i][start.j];
    g_score[start.i][start.j] = 0.0;

    loop {
        // Find cell with minimum f-score
        let mut min_f = f64::INFINITY;
        let mut current = (0, 0);
        for (i, f_row) in f_score.iter().enumerate() {
            for (j, &f_val) in f_row.iter().enumerate() {
                if f_val < min_f {
                    min_f = f_val;
                    current = (i, j);
                }
            }
        }

        if min_f.is_infinite() {
            return None; // No route found
        }

        if current == (goal.i, goal.j) {
            break; // Found the goal
        }

        // Mark as explored
        cell_state[current.0][current.1] = 2;
        f_score[current.0][current.1] = f64::INFINITY; // Remove from open set

        let neighbors = find_neighbors(current.0, current.1, m);
        for &(ni, nj) in &neighbors {
            let state = cell_state[ni][nj];
            if state == 0 || state == 5 {
                let tentative_g = g_score[current.0][current.1] + 1.0;
                if tentative_g < g_score[ni][nj] {
                    g_score[ni][nj] = tentative_g;
                    f_score[ni][nj] = tentative_g + heuristic_map[ni][nj];
                    parent[ni][nj] = Some(current);
                    if state != 5 {
                        cell_state[ni][nj] = 3; // frontier
                    }
                }
            }
        }
    }

    // Reconstruct route
    let mut route = vec![GridCell {
        i: goal.i,
        j: goal.j,
    }];
    let mut cur = (goal.i, goal.j);
    while let Some(p) = parent[cur.0][cur.1] {
        route.push(GridCell { i: p.0, j: p.1 });
        cur = p;
    }
    route.reverse();

    // Convert grid cells to joint angles
    let joint_angles: Vec<[f64; 2]> = route
        .iter()
        .map(|cell| [grid.index_to_angle(cell.i), grid.index_to_angle(cell.j)])
        .collect();

    Some(ArmPath {
        route_cells: route,
        joint_angles,
    })
}

/// High-level function: plan a collision-free path for a 2-joint arm.
///
/// Given link lengths, start/goal joint configurations (as grid cell indices),
/// and obstacles, builds the occupancy grid and runs A\* to find a path.
pub fn plan_arm_path(
    link_lengths: &[f64; 2],
    obstacles: &[CircleObstacle],
    start: GridCell,
    goal: GridCell,
    grid_size: usize,
) -> Option<ArmPath> {
    let grid = build_occupancy_grid(link_lengths, obstacles, grid_size);
    astar_torus(&grid, start, goal)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_collision_hit() {
        let a = [0.0, 0.0];
        let b = [2.0, 0.0];
        let obs = CircleObstacle::new(1.0, 0.0, 0.5);
        assert!(detect_collision(&a, &b, &obs));
    }

    #[test]
    fn test_detect_collision_miss() {
        let a = [0.0, 0.0];
        let b = [2.0, 0.0];
        let obs = CircleObstacle::new(1.0, 2.0, 0.5);
        assert!(!detect_collision(&a, &b, &obs));
    }

    #[test]
    fn test_detect_collision_endpoint() {
        let a = [0.0, 0.0];
        let b = [1.0, 0.0];
        // Obstacle at endpoint b
        let obs = CircleObstacle::new(1.0, 0.0, 0.1);
        assert!(detect_collision(&a, &b, &obs));
    }

    #[test]
    fn test_detect_collision_before_segment() {
        let a = [1.0, 0.0];
        let b = [2.0, 0.0];
        let obs = CircleObstacle::new(0.0, 0.0, 0.5);
        assert!(!detect_collision(&a, &b, &obs));
    }

    #[test]
    fn test_detect_collision_degenerate_segment() {
        let a = [0.0, 0.0];
        let b = [0.0, 0.0];
        let obs = CircleObstacle::new(0.0, 0.0, 0.5);
        assert!(detect_collision(&a, &b, &obs));
    }

    #[test]
    fn test_joint_space_grid_basic() {
        let grid = JointSpaceGrid::new(10);
        assert!(!grid.get(0, 0));
        assert_eq!(grid.size, 10);
    }

    #[test]
    fn test_grid_set_get() {
        let mut grid = JointSpaceGrid::new(10);
        grid.set(3, 5, true);
        assert!(grid.get(3, 5));
        assert!(!grid.get(5, 3));
    }

    #[test]
    fn test_index_to_angle() {
        let grid = JointSpaceGrid::new(100);
        // Index 0 => -pi
        assert!((grid.index_to_angle(0) - (-PI)).abs() < 1e-10);
        // Index 50 => 0
        assert!(grid.index_to_angle(50).abs() < 1e-10);
    }

    #[test]
    fn test_find_neighbors_middle() {
        let neighbors = find_neighbors(5, 5, 10);
        assert_eq!(neighbors, [(4, 5), (6, 5), (5, 4), (5, 6)]);
    }

    #[test]
    fn test_find_neighbors_wrap() {
        let neighbors = find_neighbors(0, 0, 10);
        assert_eq!(neighbors, [(9, 0), (1, 0), (0, 9), (0, 1)]);
    }

    #[test]
    fn test_heuristic_map_at_goal() {
        let goal = GridCell { i: 5, j: 5 };
        let hmap = calc_heuristic_map(10, goal);
        assert!((hmap[5][5] - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_heuristic_map_toroidal() {
        let goal = GridCell { i: 0, j: 0 };
        let hmap = calc_heuristic_map(10, goal);
        // Cell (9, 9) should have distance 2 (wrap around in both dims)
        assert!((hmap[9][9] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_astar_torus_empty_grid() {
        let grid = JointSpaceGrid::new(20);
        let start = GridCell { i: 2, j: 2 };
        let goal = GridCell { i: 18, j: 18 };
        let result = astar_torus(&grid, start, goal);
        assert!(result.is_some());
        let path = result.unwrap();
        assert_eq!(path.route_cells.first().unwrap(), &start);
        assert_eq!(path.route_cells.last().unwrap(), &goal);
        assert_eq!(path.route_cells.len(), path.joint_angles.len());
    }

    #[test]
    fn test_astar_torus_wrapping() {
        // On a 10x10 grid, going from (0,0) to (9,9) should wrap
        let grid = JointSpaceGrid::new(10);
        let start = GridCell { i: 0, j: 0 };
        let goal = GridCell { i: 9, j: 9 };
        let result = astar_torus(&grid, start, goal);
        assert!(result.is_some());
        let path = result.unwrap();
        // Wrapping path should be short: 2 steps
        assert_eq!(path.route_cells.len(), 3); // start + 2 moves
    }

    #[test]
    fn test_astar_torus_blocked() {
        // On a torus, completely isolating a cell requires blocking ALL its neighbors.
        // Block all 8 neighbors of (3,3) on a 5x5 torus.
        let mut grid = JointSpaceGrid::new(5);
        for di in [-1i32, 0, 1] {
            for dj in [-1i32, 0, 1] {
                if di == 0 && dj == 0 {
                    continue;
                }
                let ni = ((3 + di).rem_euclid(5)) as usize;
                let nj = ((3 + dj).rem_euclid(5)) as usize;
                grid.set(ni, nj, true);
            }
        }
        let start = GridCell { i: 0, j: 0 };
        let goal = GridCell { i: 3, j: 3 };
        let result = astar_torus(&grid, start, goal);
        assert!(result.is_none());
    }

    #[test]
    fn test_build_occupancy_grid_no_obstacles() {
        let link_lengths = [1.0, 1.0];
        let grid = build_occupancy_grid(&link_lengths, &[], 20);
        // With no obstacles, no cell should be occupied
        for i in 0..20 {
            for j in 0..20 {
                assert!(!grid.get(i, j));
            }
        }
    }

    #[test]
    fn test_build_occupancy_grid_with_obstacle() {
        let link_lengths = [1.0, 1.0];
        // Obstacle right on the x-axis at distance 1.5
        let obstacles = vec![CircleObstacle::new(1.5, 0.0, 0.3)];
        let grid = build_occupancy_grid(&link_lengths, &obstacles, 20);
        // The configuration (0, 0) should collide (arm extends to x=2 along x-axis)
        // Index for angle 0 is at index 10 (midpoint of grid)
        assert!(grid.get(10, 10));
    }

    #[test]
    fn test_plan_arm_path_simple() {
        // No obstacles: path should always be found
        let link_lengths = [1.0, 1.0];
        let start = GridCell { i: 5, j: 5 };
        let goal = GridCell { i: 15, j: 15 };
        let result = plan_arm_path(&link_lengths, &[], start, goal, 20);
        assert!(result.is_some());
        let path = result.unwrap();
        assert!(!path.route_cells.is_empty());
        assert_eq!(path.route_cells[0], start);
        assert_eq!(*path.route_cells.last().unwrap(), goal);
    }

    #[test]
    fn test_plan_arm_path_with_obstacles() {
        let link_lengths = [1.0, 1.0];
        let obstacles = vec![
            CircleObstacle::new(1.75, 0.75, 0.6),
            CircleObstacle::new(0.55, 1.5, 0.5),
        ];
        let grid = build_occupancy_grid(&link_lengths, &obstacles, 50);

        // Find a start and goal that are free
        let start = GridCell { i: 5, j: 25 };
        let goal = GridCell { i: 30, j: 30 };

        // Only try if start and goal are free
        if !grid.get(start.i, start.j) && !grid.get(goal.i, goal.j) {
            let result = astar_torus(&grid, start, goal);
            if let Some(path) = result {
                // Verify no cell in route is an obstacle
                for cell in &path.route_cells {
                    assert!(!grid.get(cell.i, cell.j));
                }
            }
        }
    }

    #[test]
    fn test_arm_path_joint_angles_match_cells() {
        let grid = JointSpaceGrid::new(20);
        let start = GridCell { i: 3, j: 7 };
        let goal = GridCell { i: 10, j: 15 };
        let result = astar_torus(&grid, start, goal).unwrap();

        for (cell, angles) in result.route_cells.iter().zip(result.joint_angles.iter()) {
            let expected_theta1 = grid.index_to_angle(cell.i);
            let expected_theta2 = grid.index_to_angle(cell.j);
            assert!((angles[0] - expected_theta1).abs() < 1e-10);
            assert!((angles[1] - expected_theta2).abs() < 1e-10);
        }
    }

    #[test]
    fn test_start_equals_goal() {
        let grid = JointSpaceGrid::new(10);
        let cell = GridCell { i: 5, j: 5 };
        let result = astar_torus(&grid, cell, cell);
        assert!(result.is_some());
        let path = result.unwrap();
        assert_eq!(path.route_cells.len(), 1);
    }
}
