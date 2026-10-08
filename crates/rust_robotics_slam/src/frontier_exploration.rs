//! Frontier-based exploration (Yamauchi, 1997) on an [`OccupancyGrid`].
//!
//! A *frontier cell* is a free cell next to an unknown one. Frontier cells are
//! grouped into 8-connected clusters; a breadth-first wavefront from the robot
//! through free cells with enough clearance from obstacles finds which
//! clusters are reachable and how far away they are. The explorer drives to
//! the closest reachable cell of the best cluster, trading travel distance
//! against cluster size, and is done when no reachable frontier remains.

use std::collections::VecDeque;

use nalgebra::Vector2;

use crate::lidar_occupancy::{CellState, OccupancyGrid};

/// Tuning parameters for [`find_frontiers`] and [`next_frontier_goal`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrontierConfig {
    /// Clusters with fewer cells are ignored (raster noise, wall cracks).
    pub min_cluster_cells: usize,
    /// Only cells at least this far from an occupied cell are traversable
    /// \[m\]; use the robot radius plus a margin.
    pub clearance: f64,
    /// Score = path length − `size_weight` × √(cluster cells) \[m\]: larger
    /// frontiers promise more new map per meter driven.
    pub size_weight: f64,
}

impl Default for FrontierConfig {
    fn default() -> Self {
        Self {
            min_cluster_cells: 8,
            clearance: 0.5,
            size_weight: 0.5,
        }
    }
}

/// One connected group of frontier cells.
#[derive(Debug, Clone, PartialEq)]
pub struct Frontier {
    /// Grid cells `(x, y)` of the cluster.
    pub cells: Vec<(usize, usize)>,
    /// World centroid of the cluster.
    pub centroid: Vector2<f64>,
}

impl Frontier {
    /// Number of cells in the cluster.
    pub fn len(&self) -> usize {
        self.cells.len()
    }

    /// Whether the cluster has no cells.
    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }
}

/// A chosen exploration target.
#[derive(Debug, Clone, PartialEq)]
pub struct FrontierGoal {
    /// The reachable frontier cell to drive to (world).
    pub target: Vector2<f64>,
    /// Wavefront path length from the robot to `target` \[m\].
    pub distance: f64,
    /// The cluster `target` belongs to.
    pub frontier: Frontier,
}

const NEIGHBORS_4: [(i64, i64); 4] = [(1, 0), (-1, 0), (0, 1), (0, -1)];
const NEIGHBORS_8: [(i64, i64); 8] = [
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
];

fn offset(
    grid: &OccupancyGrid,
    (x, y): (usize, usize),
    (dx, dy): (i64, i64),
) -> Option<(usize, usize)> {
    let (width, height) = grid.size();
    let (nx, ny) = (x as i64 + dx, y as i64 + dy);
    (nx >= 0 && ny >= 0 && (nx as usize) < width && (ny as usize) < height)
        .then_some((nx as usize, ny as usize))
}

fn is_frontier_cell(grid: &OccupancyGrid, cell: (usize, usize)) -> bool {
    grid.state(cell.0, cell.1) == CellState::Free
        && NEIGHBORS_4.iter().any(|&delta| {
            offset(grid, cell, delta).is_some_and(|(x, y)| grid.state(x, y) == CellState::Unknown)
        })
}

/// Every frontier cluster with at least `config.min_cluster_cells` cells.
pub fn find_frontiers(grid: &OccupancyGrid, config: &FrontierConfig) -> Vec<Frontier> {
    let (width, height) = grid.size();
    let mut visited = vec![false; width * height];
    let mut frontiers = Vec::new();
    for y in 0..height {
        for x in 0..width {
            if visited[y * width + x] || !is_frontier_cell(grid, (x, y)) {
                continue;
            }
            visited[y * width + x] = true;
            let mut cells = Vec::new();
            let mut queue = VecDeque::from([(x, y)]);
            while let Some(cell) = queue.pop_front() {
                cells.push(cell);
                for delta in NEIGHBORS_8 {
                    if let Some(next) = offset(grid, cell, delta) {
                        let index = next.1 * width + next.0;
                        if !visited[index] && is_frontier_cell(grid, next) {
                            visited[index] = true;
                            queue.push_back(next);
                        }
                    }
                }
            }
            if cells.len() >= config.min_cluster_cells {
                let sum = cells.iter().fold(Vector2::zeros(), |sum, &(x, y)| {
                    sum + grid.cell_center(x, y)
                });
                frontiers.push(Frontier {
                    centroid: sum / cells.len() as f64,
                    cells,
                });
            }
        }
    }
    frontiers
}

/// Wavefront distances \[m\] from `start` through free cells with clearance
/// (`f64::INFINITY` where unreachable). The start cell is always seeded so a
/// robot hugging a wall can still leave it.
fn wavefront(grid: &OccupancyGrid, start: Vector2<f64>, clearance: f64) -> Vec<f64> {
    let (width, height) = grid.size();
    let mut distance = vec![f64::INFINITY; width * height];
    let Some(start) = grid.cell_of(start) else {
        return distance;
    };
    let field = grid.distance_field(clearance + grid.resolution());
    let open = |(x, y): (usize, usize)| {
        grid.state(x, y) == CellState::Free && f64::from(field[y * width + x]) >= clearance
    };
    // Dijkstra over the 8-connected grid; the weights are only 1 and √2, so
    // a binary heap over ordered integers (millimeters) is exact enough.
    let mut heap = std::collections::BinaryHeap::new();
    distance[start.1 * width + start.0] = 0.0;
    heap.push(std::cmp::Reverse((0_u64, start)));
    while let Some(std::cmp::Reverse((millimeters, cell))) = heap.pop() {
        let here = distance[cell.1 * width + cell.0];
        if (millimeters as f64) > here * 1000.0 + 0.5 {
            continue;
        }
        for (dx, dy) in NEIGHBORS_8 {
            let Some(next) = offset(grid, cell, (dx, dy)) else {
                continue;
            };
            if !open(next) {
                continue;
            }
            let step = if dx != 0 && dy != 0 {
                std::f64::consts::SQRT_2
            } else {
                1.0
            } * grid.resolution();
            let index = next.1 * width + next.0;
            if here + step < distance[index] {
                distance[index] = here + step;
                heap.push(std::cmp::Reverse((
                    ((here + step) * 1000.0).round() as u64,
                    next,
                )));
            }
        }
    }
    distance
}

/// The best reachable frontier from `robot`, skipping clusters whose
/// centroid lies within 1 m of a point in `excluded` (e.g. goals the
/// planner already failed to reach). `None` when exploration is complete.
pub fn next_frontier_goal(
    grid: &OccupancyGrid,
    robot: Vector2<f64>,
    excluded: &[Vector2<f64>],
    config: &FrontierConfig,
) -> Option<FrontierGoal> {
    let (width, _) = grid.size();
    let distance = wavefront(grid, robot, config.clearance);
    find_frontiers(grid, config)
        .into_iter()
        .filter(|frontier| {
            excluded
                .iter()
                .all(|point| (point - frontier.centroid).norm() > 1.0)
        })
        .filter_map(|frontier| {
            let &(x, y) = frontier.cells.iter().min_by(|a, b| {
                distance[a.1 * width + a.0].total_cmp(&distance[b.1 * width + b.0])
            })?;
            let reach = distance[y * width + x];
            reach.is_finite().then(|| FrontierGoal {
                target: grid.cell_center(x, y),
                distance: reach,
                frontier,
            })
        })
        .min_by(|a, b| {
            let score = |goal: &FrontierGoal| {
                goal.distance - config.size_weight * (goal.frontier.len() as f64).sqrt()
            };
            score(a).total_cmp(&score(b))
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lidar_occupancy::OccupancyConfig;
    use crate::scan_to_map::{ray_cast_ranges, LineSegment};
    use rust_robotics_core::Pose2D;

    /// Two 6 × 4 m rooms joined by a 1.6 m door at x = 0.
    fn walls() -> Vec<LineSegment> {
        let v = Vector2::new;
        vec![
            LineSegment::new(v(-6.0, -2.0), v(6.0, -2.0)),
            LineSegment::new(v(6.0, -2.0), v(6.0, 2.0)),
            LineSegment::new(v(6.0, 2.0), v(-6.0, 2.0)),
            LineSegment::new(v(-6.0, 2.0), v(-6.0, -2.0)),
            LineSegment::new(v(0.0, -2.0), v(0.0, -0.8)),
            LineSegment::new(v(0.0, 0.8), v(0.0, 2.0)),
        ]
    }

    fn grid_after(poses: &[Pose2D]) -> OccupancyGrid {
        let mut grid = OccupancyGrid::new(
            Vector2::new(-7.0, -3.0),
            Vector2::new(7.0, 3.0),
            OccupancyConfig::default(),
        );
        for pose in poses {
            // A short-range LiDAR, so one room is not seen through the door.
            grid.insert_ranges(*pose, &ray_cast_ranges(*pose, &walls(), 360, 4.0), 4.0);
        }
        grid
    }

    #[test]
    fn the_unseen_room_is_a_reachable_frontier() {
        // Every corner of the left room is within the 4 m range from here.
        let robot = Pose2D::new(-3.0, 0.0, 0.0);
        let grid = grid_after(&[robot]);
        let frontiers = find_frontiers(&grid, &FrontierConfig::default());
        assert!(!frontiers.is_empty());
        let goal = next_frontier_goal(
            &grid,
            Vector2::new(robot.x, robot.y),
            &[],
            &FrontierConfig::default(),
        )
        .expect("goal");
        // The 4 m LiDAR ends at x = 0 (the door), so the frontier is there.
        assert!(goal.target.x > -0.5, "goal {:?}", goal.target);
        assert!(goal.distance.is_finite() && goal.distance > 2.0);
    }

    #[test]
    fn excluded_goals_are_skipped() {
        let robot = Vector2::new(-4.0, 0.0);
        let grid = grid_after(&[Pose2D::new(robot.x, robot.y, 0.0)]);
        let config = FrontierConfig::default();
        let first = next_frontier_goal(&grid, robot, &[], &config).expect("goal");
        let second = next_frontier_goal(&grid, robot, &[first.frontier.centroid], &config);
        assert!(second.map_or(true, |goal| goal.frontier.centroid
            != first.frontier.centroid));
    }

    #[test]
    fn exploration_is_complete_when_everything_was_seen() {
        let poses: Vec<Pose2D> = [-5.0, -3.0, -1.0, 1.0, 3.0, 5.0]
            .iter()
            .flat_map(|&x| [Pose2D::new(x, -1.0, 0.0), Pose2D::new(x, 1.0, 0.0)])
            .collect();
        let grid = grid_after(&poses);
        let goal = next_frontier_goal(
            &grid,
            Vector2::new(-4.0, 0.0),
            &[],
            &FrontierConfig::default(),
        );
        assert!(goal.is_none(), "unexpected frontier {goal:?}");
    }
}
