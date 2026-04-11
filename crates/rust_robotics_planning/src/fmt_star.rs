//! Fast Marching Tree (FMT*) path planning.
//!
//! FMT* is a batch sampling-based planner that expands a wavefront over a
//! pre-sampled neighborhood graph to build a low-cost collision-free path.
//!
//! Reference:
//! - Lucas Janson, Edward Schmerling, Ashley Clark, Marco Pavone,
//!   "Fast Marching Tree: a Fast Marching Sampling-Based Method for Optimal
//!   Motion Planning in Many Dimensions":
//!   <https://stanford.edu/~pavone/papers/Janson.Schmerling.ea.IJRR15.pdf>

use rand::Rng;
use rust_robotics_core::{Path2D, PathPlanner, Point2D, RoboticsError, RoboticsResult};

/// Circular obstacle represented by center position and radius.
#[derive(Debug, Clone)]
pub struct CircleObstacle {
    pub x: f64,
    pub y: f64,
    pub radius: f64,
}

impl CircleObstacle {
    /// Creates a circular obstacle.
    pub fn new(x: f64, y: f64, radius: f64) -> Self {
        Self { x, y, radius }
    }
}

/// Axis-aligned sampling bounds.
#[derive(Debug, Clone)]
pub struct AreaBounds {
    pub xmin: f64,
    pub xmax: f64,
    pub ymin: f64,
    pub ymax: f64,
}

impl AreaBounds {
    /// Creates sampling bounds.
    pub fn new(xmin: f64, xmax: f64, ymin: f64, ymax: f64) -> Self {
        Self {
            xmin,
            xmax,
            ymin,
            ymax,
        }
    }
}

/// Configuration for the FMT* planner.
#[derive(Debug, Clone, Copy)]
pub struct FMTStarConfig {
    /// Number of random samples drawn before adding start and goal.
    pub n_samples: usize,
    /// Maximum connection distance \[m\].
    pub connection_radius: f64,
    /// Robot radius used for obstacle inflation \[m\].
    pub robot_radius: f64,
}

impl Default for FMTStarConfig {
    fn default() -> Self {
        Self {
            n_samples: 500,
            connection_radius: 5.0,
            robot_radius: 0.8,
        }
    }
}

impl FMTStarConfig {
    /// Validates the configuration.
    pub fn validate(&self) -> RoboticsResult<()> {
        if self.n_samples == 0 {
            return Err(RoboticsError::InvalidParameter(
                "FMT*: n_samples must be greater than zero".to_string(),
            ));
        }
        if self.connection_radius <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "FMT*: connection_radius must be positive".to_string(),
            ));
        }
        if self.robot_radius < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "FMT*: robot_radius cannot be negative".to_string(),
            ));
        }
        Ok(())
    }
}

/// Fast Marching Tree planner.
pub struct FMTStarPlanner {
    obstacles: Vec<CircleObstacle>,
    rand_area: AreaBounds,
    config: FMTStarConfig,
}

impl FMTStarPlanner {
    /// Creates a new planner.
    pub fn new(
        obstacles: Vec<CircleObstacle>,
        rand_area: AreaBounds,
        config: FMTStarConfig,
    ) -> Self {
        Self {
            obstacles,
            rand_area,
            config,
        }
    }

    fn plan_with_rng<R: Rng + ?Sized>(
        &self,
        start: Point2D,
        goal: Point2D,
        rng: &mut R,
    ) -> RoboticsResult<Path2D> {
        self.config.validate()?;
        self.validate_query_point(start, "start")?;
        self.validate_query_point(goal, "goal")?;

        let points = self.sample_points(start, goal, rng);
        let neighbors = self.build_neighbors(&points);
        let goal_index = points.len() - 1;

        let mut parents = vec![None; points.len()];
        let mut costs = vec![f64::INFINITY; points.len()];
        let mut is_open = vec![false; points.len()];
        let mut is_unvisited = vec![true; points.len()];
        let mut open_vertices = vec![0usize];

        costs[0] = 0.0;
        is_open[0] = true;
        is_unvisited[0] = false;

        while !open_vertices.is_empty() {
            let z_pos = open_vertices
                .iter()
                .enumerate()
                .min_by(|(_, lhs), (_, rhs)| costs[**lhs].total_cmp(&costs[**rhs]))
                .map(|(idx, _)| idx)
                .expect("open set is non-empty");
            let z = open_vertices[z_pos];

            if z == goal_index {
                return Ok(self.extract_path(&points, &parents, goal_index));
            }

            let mut newly_open = Vec::new();
            for &x in &neighbors[z] {
                if !is_unvisited[x] {
                    continue;
                }

                let best_parent = neighbors[x]
                    .iter()
                    .copied()
                    .filter(|&candidate| is_open[candidate])
                    .map(|candidate| {
                        let candidate_cost =
                            costs[candidate] + points[candidate].distance(&points[x]);
                        (candidate, candidate_cost)
                    })
                    .min_by(|lhs, rhs| lhs.1.total_cmp(&rhs.1));

                let Some((parent, best_cost)) = best_parent else {
                    continue;
                };

                if self.is_segment_collision_free(points[parent], points[x]) {
                    parents[x] = Some(parent);
                    costs[x] = best_cost;
                    newly_open.push(x);
                }
            }

            open_vertices.swap_remove(z_pos);
            is_open[z] = false;

            for idx in newly_open {
                if is_unvisited[idx] {
                    is_unvisited[idx] = false;
                    is_open[idx] = true;
                    open_vertices.push(idx);
                }
            }
        }

        Err(RoboticsError::PlanningError(
            "FMT*: Cannot find path with current samples".to_string(),
        ))
    }

    fn validate_query_point(&self, point: Point2D, label: &str) -> RoboticsResult<()> {
        let inside_bounds = point.x >= self.rand_area.xmin
            && point.x <= self.rand_area.xmax
            && point.y >= self.rand_area.ymin
            && point.y <= self.rand_area.ymax;
        if !inside_bounds {
            return Err(RoboticsError::InvalidParameter(format!(
                "FMT*: {label} is outside sampling bounds"
            )));
        }
        if !self.is_state_collision_free(point) {
            return Err(RoboticsError::InvalidParameter(format!(
                "FMT*: {label} lies inside an obstacle"
            )));
        }
        Ok(())
    }

    fn sample_points<R: Rng + ?Sized>(
        &self,
        start: Point2D,
        goal: Point2D,
        rng: &mut R,
    ) -> Vec<Point2D> {
        let mut points = Vec::with_capacity(self.config.n_samples + 2);
        points.push(start);

        while points.len() < self.config.n_samples + 1 {
            let point = Point2D::new(
                rng.gen_range(self.rand_area.xmin..=self.rand_area.xmax),
                rng.gen_range(self.rand_area.ymin..=self.rand_area.ymax),
            );
            if self.is_state_collision_free(point) {
                points.push(point);
            }
        }

        points.push(goal);
        points
    }

    fn build_neighbors(&self, points: &[Point2D]) -> Vec<Vec<usize>> {
        let mut neighbors = vec![Vec::new(); points.len()];
        for i in 0..points.len() {
            for j in (i + 1)..points.len() {
                if points[i].distance(&points[j]) <= self.config.connection_radius {
                    neighbors[i].push(j);
                    neighbors[j].push(i);
                }
            }
        }
        neighbors
    }

    fn extract_path(
        &self,
        points: &[Point2D],
        parents: &[Option<usize>],
        goal_index: usize,
    ) -> Path2D {
        let mut path = vec![points[goal_index]];
        let mut current = goal_index;

        while let Some(parent) = parents[current] {
            path.push(points[parent]);
            current = parent;
        }

        path.reverse();
        Path2D::from_points(path)
    }

    fn is_state_collision_free(&self, point: Point2D) -> bool {
        self.obstacles.iter().all(|obstacle| {
            let dx = point.x - obstacle.x;
            let dy = point.y - obstacle.y;
            (dx * dx + dy * dy).sqrt() > obstacle.radius + self.config.robot_radius
        })
    }

    fn is_segment_collision_free(&self, start: Point2D, end: Point2D) -> bool {
        let distance = start.distance(&end);
        if distance <= f64::EPSILON {
            return self.is_state_collision_free(start);
        }

        let step = (self.config.robot_radius * 0.5).max(0.05);
        let n_steps = (distance / step).ceil() as usize;

        for i in 0..=n_steps {
            let t = i as f64 / n_steps as f64;
            let point = Point2D::new(
                start.x + (end.x - start.x) * t,
                start.y + (end.y - start.y) * t,
            );
            if !self.is_state_collision_free(point) {
                return false;
            }
        }

        true
    }
}

impl PathPlanner for FMTStarPlanner {
    fn plan(&self, start: Point2D, goal: Point2D) -> Result<Path2D, RoboticsError> {
        let mut rng = rand::thread_rng();
        self.plan_with_rng(start, goal, &mut rng)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::{rngs::StdRng, SeedableRng};

    fn path_length(path: &Path2D) -> f64 {
        path.points
            .windows(2)
            .map(|pair| pair[0].distance(&pair[1]))
            .sum()
    }

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {expected:.12}, got {actual:.12}, tolerance {tolerance:.3e}"
        );
    }

    #[test]
    fn test_fmt_star_config_defaults() {
        let config = FMTStarConfig::default();
        assert_eq!(config.n_samples, 500);
        assert_eq!(config.connection_radius, 5.0);
        assert_eq!(config.robot_radius, 0.8);
    }

    #[test]
    fn test_fmt_star_finds_path_without_obstacles() {
        let planner = FMTStarPlanner::new(
            Vec::new(),
            AreaBounds::new(-1.0, 5.0, -1.0, 5.0),
            FMTStarConfig {
                n_samples: 50,
                connection_radius: 10.0,
                robot_radius: 0.1,
            },
        );
        let mut rng = StdRng::seed_from_u64(7);
        let path = planner
            .plan_with_rng(Point2D::new(0.0, 0.0), Point2D::new(4.0, 4.0), &mut rng)
            .expect("planner should find a path");

        assert!(!path.is_empty());
        assert_eq!(path.points.first().expect("path").x, 0.0);
        assert_eq!(path.points.first().expect("path").y, 0.0);
        assert_eq!(path.points.last().expect("path").x, 4.0);
        assert_eq!(path.points.last().expect("path").y, 4.0);
    }

    #[test]
    fn test_fmt_star_finds_path_with_obstacles() {
        let planner = FMTStarPlanner::new(
            vec![CircleObstacle::new(2.5, 2.5, 0.9)],
            AreaBounds::new(-1.0, 6.0, -1.0, 6.0),
            FMTStarConfig {
                n_samples: 400,
                connection_radius: 2.5,
                robot_radius: 0.2,
            },
        );
        let mut rng = StdRng::seed_from_u64(19);
        let path = planner
            .plan_with_rng(Point2D::new(0.0, 0.0), Point2D::new(5.0, 5.0), &mut rng)
            .expect("planner should find a path around the obstacle");

        assert!(path.points.len() >= 2);
        assert_eq!(path.points.first().expect("path").x, 0.0);
        assert_eq!(path.points.first().expect("path").y, 0.0);
        assert_eq!(path.points.last().expect("path").x, 5.0);
        assert_eq!(path.points.last().expect("path").y, 5.0);
        assert!(path
            .points
            .iter()
            .all(|point| { point.distance(&Point2D::new(2.5, 2.5)) > 1.1 }));
    }

    #[test]
    fn test_fmt_star_obstacle_case_regression() {
        let planner = FMTStarPlanner::new(
            vec![CircleObstacle::new(2.5, 2.5, 0.9)],
            AreaBounds::new(-1.0, 6.0, -1.0, 6.0),
            FMTStarConfig {
                n_samples: 400,
                connection_radius: 2.5,
                robot_radius: 0.2,
            },
        );
        let mut rng = StdRng::seed_from_u64(19);
        let path = planner
            .plan_with_rng(Point2D::new(0.0, 0.0), Point2D::new(5.0, 5.0), &mut rng)
            .expect("planner should find the regression path");

        assert_eq!(path.points.len(), 5);
        assert_close(path_length(&path), 7.535322534477, 1.0e-9);
        assert_close(path.points[1].x, 0.726415085720, 1.0e-9);
        assert_close(path.points[1].y, 1.726691311279, 1.0e-9);
        assert_close(path.points[2].x, 1.269433984463, 1.0e-9);
        assert_close(path.points[2].y, 3.074339829122, 1.0e-9);
        assert_close(path.points[3].x, 3.149556108870, 1.0e-9);
        assert_close(path.points[3].y, 4.214733441962, 1.0e-9);
    }
}
