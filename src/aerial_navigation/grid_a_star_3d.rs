//! 3D grid-based A* path planning for aerial robots.
//!
//! The planner operates on a voxel grid and returns a collision-free path
//! between two 3D points inside a bounded workspace.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

use crate::common::{Point3D, RoboticsError, RoboticsResult};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct GridPoint3D {
    x: i32,
    y: i32,
    z: i32,
}

impl GridPoint3D {
    fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }
}

#[derive(Debug, Clone)]
pub struct Path3D {
    pub points: Vec<Point3D>,
}

impl Path3D {
    pub fn new(points: Vec<Point3D>) -> Self {
        Self { points }
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

#[derive(Debug, Clone)]
pub struct GridAStar3DConfig {
    pub resolution: f64,
    pub bounds_min: Point3D,
    pub bounds_max: Point3D,
    pub allow_diagonal: bool,
}

impl Default for GridAStar3DConfig {
    fn default() -> Self {
        Self {
            resolution: 1.0,
            bounds_min: Point3D::new(0.0, 0.0, 0.0),
            bounds_max: Point3D::new(10.0, 10.0, 5.0),
            allow_diagonal: true,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct PriorityNode {
    point: GridPoint3D,
    cost: f64,
    priority: f64,
}

impl Eq for PriorityNode {}

impl PartialEq for PriorityNode {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority
    }
}

impl Ord for PriorityNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .priority
            .partial_cmp(&self.priority)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for PriorityNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub struct GridAStar3DPlanner {
    config: GridAStar3DConfig,
    max_index: GridPoint3D,
    obstacles: HashSet<GridPoint3D>,
    motions: Vec<(i32, i32, i32, f64)>,
}

impl GridAStar3DPlanner {
    pub fn new(config: GridAStar3DConfig, obstacles: &[Point3D]) -> RoboticsResult<Self> {
        validate_config(&config)?;

        let max_index = GridPoint3D::new(
            ((config.bounds_max.x - config.bounds_min.x) / config.resolution).round() as i32,
            ((config.bounds_max.y - config.bounds_min.y) / config.resolution).round() as i32,
            ((config.bounds_max.z - config.bounds_min.z) / config.resolution).round() as i32,
        );

        let planner = Self {
            max_index,
            obstacles: obstacles
                .iter()
                .map(|point| quantize(point, &config))
                .collect(),
            motions: build_motion_model(config.allow_diagonal),
            config,
        };

        Ok(planner)
    }

    pub fn plan(&self, start: Point3D, goal: Point3D) -> RoboticsResult<Path3D> {
        let start_grid = quantize(&start, &self.config);
        let goal_grid = quantize(&goal, &self.config);

        if !self.is_valid(start_grid) {
            return Err(RoboticsError::PlanningError(
                "Start point is out of bounds or occupied".to_string(),
            ));
        }

        if !self.is_valid(goal_grid) {
            return Err(RoboticsError::PlanningError(
                "Goal point is out of bounds or occupied".to_string(),
            ));
        }

        let mut open_set = BinaryHeap::new();
        let mut came_from = HashMap::new();
        let mut best_cost = HashMap::new();

        open_set.push(PriorityNode {
            point: start_grid,
            cost: 0.0,
            priority: heuristic(start_grid, goal_grid),
        });
        best_cost.insert(start_grid, 0.0);

        while let Some(current) = open_set.pop() {
            if current.point == goal_grid {
                return Ok(self.reconstruct_path(goal_grid, start_grid, &came_from));
            }

            let Some(known_cost) = best_cost.get(&current.point).copied() else {
                continue;
            };
            if current.cost > known_cost {
                continue;
            }

            for (dx, dy, dz, move_cost) in &self.motions {
                let next = GridPoint3D::new(
                    current.point.x + dx,
                    current.point.y + dy,
                    current.point.z + dz,
                );
                if !self.is_valid(next) {
                    continue;
                }

                let tentative_cost = current.cost + move_cost;
                let current_best = best_cost.get(&next).copied().unwrap_or(f64::INFINITY);
                if tentative_cost >= current_best {
                    continue;
                }

                came_from.insert(next, current.point);
                best_cost.insert(next, tentative_cost);
                open_set.push(PriorityNode {
                    point: next,
                    cost: tentative_cost,
                    priority: tentative_cost + heuristic(next, goal_grid),
                });
            }
        }

        Err(RoboticsError::PlanningError("No 3D path found".to_string()))
    }

    fn is_valid(&self, point: GridPoint3D) -> bool {
        point.x >= 0
            && point.y >= 0
            && point.z >= 0
            && point.x <= self.max_index.x
            && point.y <= self.max_index.y
            && point.z <= self.max_index.z
            && !self.obstacles.contains(&point)
    }

    fn reconstruct_path(
        &self,
        goal: GridPoint3D,
        start: GridPoint3D,
        came_from: &HashMap<GridPoint3D, GridPoint3D>,
    ) -> Path3D {
        let mut points = vec![goal];
        let mut current = goal;

        while current != start {
            current = came_from[&current];
            points.push(current);
        }

        points.reverse();
        Path3D::new(
            points
                .into_iter()
                .map(|point| dequantize(point, &self.config))
                .collect(),
        )
    }
}

fn validate_config(config: &GridAStar3DConfig) -> RoboticsResult<()> {
    if config.resolution <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "resolution must be positive".to_string(),
        ));
    }

    if config.bounds_min.x > config.bounds_max.x
        || config.bounds_min.y > config.bounds_max.y
        || config.bounds_min.z > config.bounds_max.z
    {
        return Err(RoboticsError::InvalidParameter(
            "bounds_min must not exceed bounds_max".to_string(),
        ));
    }

    Ok(())
}

fn quantize(point: &Point3D, config: &GridAStar3DConfig) -> GridPoint3D {
    GridPoint3D::new(
        ((point.x - config.bounds_min.x) / config.resolution).round() as i32,
        ((point.y - config.bounds_min.y) / config.resolution).round() as i32,
        ((point.z - config.bounds_min.z) / config.resolution).round() as i32,
    )
}

fn dequantize(point: GridPoint3D, config: &GridAStar3DConfig) -> Point3D {
    Point3D::new(
        config.bounds_min.x + point.x as f64 * config.resolution,
        config.bounds_min.y + point.y as f64 * config.resolution,
        config.bounds_min.z + point.z as f64 * config.resolution,
    )
}

fn heuristic(a: GridPoint3D, b: GridPoint3D) -> f64 {
    let dx = (a.x - b.x) as f64;
    let dy = (a.y - b.y) as f64;
    let dz = (a.z - b.z) as f64;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn build_motion_model(allow_diagonal: bool) -> Vec<(i32, i32, i32, f64)> {
    let mut motions = vec![
        (1, 0, 0, 1.0),
        (-1, 0, 0, 1.0),
        (0, 1, 0, 1.0),
        (0, -1, 0, 1.0),
        (0, 0, 1, 1.0),
        (0, 0, -1, 1.0),
    ];

    if allow_diagonal {
        for dx in -1_i32..=1 {
            for dy in -1_i32..=1 {
                for dz in -1_i32..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    if dx.abs() + dy.abs() + dz.abs() <= 1 {
                        continue;
                    }
                    let cost = ((dx * dx + dy * dy + dz * dz) as f64).sqrt();
                    motions.push((dx, dy, dz, cost));
                }
            }
        }
    }

    motions
}

pub fn demo_grid_a_star_3d() {
    let config = GridAStar3DConfig {
        bounds_min: Point3D::new(0.0, 0.0, 0.0),
        bounds_max: Point3D::new(6.0, 6.0, 4.0),
        ..Default::default()
    };
    let obstacles = vec![
        Point3D::new(2.0, 2.0, 0.0),
        Point3D::new(2.0, 2.0, 1.0),
        Point3D::new(2.0, 2.0, 2.0),
        Point3D::new(3.0, 2.0, 2.0),
    ];

    let planner = GridAStar3DPlanner::new(config, &obstacles).expect("planner should build");
    let start = Point3D::new(0.0, 0.0, 0.0);
    let goal = Point3D::new(5.0, 5.0, 3.0);

    match planner.plan(start, goal) {
        Ok(path) => {
            println!("3D path found with {} waypoints:", path.len());
            for point in path.points {
                println!("({:.1}, {:.1}, {:.1})", point.x, point.y, point.z);
            }
        }
        Err(error) => eprintln!("Planning failed: {}", error),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn planner_with_config(
        allow_diagonal: bool,
        obstacles: Vec<Point3D>,
    ) -> GridAStar3DPlanner {
        GridAStar3DPlanner::new(
            GridAStar3DConfig {
                resolution: 1.0,
                bounds_min: Point3D::new(0.0, 0.0, 0.0),
                bounds_max: Point3D::new(4.0, 4.0, 4.0),
                allow_diagonal,
            },
            &obstacles,
        )
        .expect("planner should be created")
    }

    #[test]
    fn test_invalid_config_is_rejected() {
        let result = GridAStar3DPlanner::new(
            GridAStar3DConfig {
                resolution: 0.0,
                ..Default::default()
            },
            &[],
        );

        assert!(matches!(result, Err(RoboticsError::InvalidParameter(_))));
    }

    #[test]
    fn test_grid_a_star_3d_finds_path() {
        let planner = planner_with_config(true, vec![]);
        let start = Point3D::new(0.0, 0.0, 0.0);
        let goal = Point3D::new(3.0, 2.0, 1.0);

        let path = planner.plan(start, goal).expect("path should exist");

        assert!(!path.is_empty());
        assert_eq!(path.points.first().copied(), Some(start));
        assert_eq!(path.points.last().copied(), Some(goal));
    }

    #[test]
    fn test_grid_a_star_3d_avoids_obstacles() {
        let obstacles = vec![
            Point3D::new(1.0, 0.0, 0.0),
            Point3D::new(1.0, 1.0, 0.0),
            Point3D::new(1.0, 2.0, 0.0),
        ];
        let planner = planner_with_config(
            false,
            obstacles.clone(),
        );

        let path = planner
            .plan(Point3D::new(0.0, 0.0, 0.0), Point3D::new(2.0, 2.0, 0.0))
            .expect("path should route around the wall");

        assert!(path.points.iter().all(|point| !obstacles.contains(point)));
        assert!(path.points.iter().any(|point| point.z > 0.0));
        assert!(path.len() > 4);
    }

    #[test]
    fn test_grid_a_star_3d_reports_no_path() {
        let planner = planner_with_config(
            false,
            vec![
                Point3D::new(0.0, 1.0, 1.0),
                Point3D::new(2.0, 1.0, 1.0),
                Point3D::new(1.0, 0.0, 1.0),
                Point3D::new(1.0, 2.0, 1.0),
                Point3D::new(1.0, 1.0, 0.0),
                Point3D::new(1.0, 1.0, 2.0),
            ],
        );

        let result = planner.plan(Point3D::new(0.0, 0.0, 0.0), Point3D::new(1.0, 1.0, 1.0));

        assert!(matches!(result, Err(RoboticsError::PlanningError(_))));
    }
}
