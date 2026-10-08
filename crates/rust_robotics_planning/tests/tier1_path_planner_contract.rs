//! Tier 1 contract: every high-traffic planner is usable as a
//! `Box<dyn PathPlanner>` and returns a collision-free path from start to goal
//! on the same map (see `docs/api_traits.md`).

use rust_robotics_core::{Obstacles, Path2D, PathPlanner, Point2D};
use rust_robotics_planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics_planning::dijkstra::{DijkstraConfig, DijkstraPlanner};
use rust_robotics_planning::jps::{JPSConfig, JPSPlanner};
use rust_robotics_planning::rrt::{AreaBounds, CircleObstacle, RRTConfig, RRTPlanner};
use rust_robotics_planning::rrt_star::RRTStar;
use rust_robotics_planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};

const START: Point2D = Point2D { x: 2.0, y: 10.0 };
const GOAL: Point2D = Point2D { x: 18.0, y: 10.0 };
const CLEARANCE: f64 = 0.3;

/// A 20 × 20 m box with a wall at x = 10 between y = 5 and y = 15.
fn obstacle_points() -> Vec<Point2D> {
    let mut points = Vec::new();
    for i in 0..=20 {
        let i = f64::from(i);
        points.extend([
            Point2D::new(i, 0.0),
            Point2D::new(i, 20.0),
            Point2D::new(0.0, i),
            Point2D::new(20.0, i),
        ]);
    }
    for i in 5..15 {
        points.push(Point2D::new(10.0, f64::from(i)));
    }
    points
}

fn planners() -> Vec<(&'static str, Box<dyn PathPlanner>)> {
    let points = obstacle_points();
    let obstacles = Obstacles::from_points(points.clone());
    let circles: Vec<(f64, f64, f64)> = points.iter().map(|p| (p.x, p.y, 0.5)).collect();
    let grid = |weight| AStarConfig {
        resolution: 1.0,
        robot_radius: 0.5,
        heuristic_weight: weight,
    };
    vec![
        (
            "A*",
            Box::new(AStarPlanner::from_obstacle_points(&obstacles, grid(1.0)).unwrap()),
        ),
        (
            "Dijkstra",
            Box::new(
                DijkstraPlanner::from_obstacle_points(
                    &obstacles,
                    DijkstraConfig {
                        resolution: 1.0,
                        robot_radius: 0.5,
                    },
                )
                .unwrap(),
            ),
        ),
        (
            "JPS",
            Box::new(
                JPSPlanner::from_obstacle_points(
                    &obstacles,
                    JPSConfig {
                        resolution: 1.0,
                        robot_radius: 0.5,
                        heuristic_weight: 1.0,
                    },
                )
                .unwrap(),
            ),
        ),
        (
            "Theta*",
            Box::new(
                ThetaStarPlanner::from_obstacle_points(
                    &obstacles,
                    ThetaStarConfig {
                        resolution: 1.0,
                        robot_radius: 0.5,
                        heuristic_weight: 1.0,
                    },
                )
                .unwrap(),
            ),
        ),
        (
            "RRT",
            Box::new(RRTPlanner::new(
                circles
                    .iter()
                    .map(|&(x, y, radius)| CircleObstacle::new(x, y, radius))
                    .collect(),
                AreaBounds::new(0.5, 19.5, 0.5, 19.5),
                Some(AreaBounds::new(0.5, 19.5, 0.5, 19.5)),
                RRTConfig {
                    expand_dis: 1.0,
                    path_resolution: 0.25,
                    goal_sample_rate: 10,
                    max_iter: 5000,
                    robot_radius: 0.2,
                },
            )),
        ),
        (
            "RRT*",
            Box::new(RRTStar::new(
                (START.x, START.y),
                (GOAL.x, GOAL.y),
                circles,
                (0.5, 19.5),
                1.0,
                0.25,
                10,
                3000,
                5.0,
                false,
                0.2,
            )),
        ),
    ]
}

fn length(path: &Path2D) -> f64 {
    path.points
        .windows(2)
        .map(|pair| pair[0].distance(&pair[1]))
        .sum()
}

fn min_clearance(path: &Path2D, obstacles: &[Point2D]) -> f64 {
    let mut clearance = f64::INFINITY;
    for pair in path.points.windows(2) {
        let steps = (pair[0].distance(&pair[1]) / 0.05).ceil().max(1.0) as usize;
        for step in 0..=steps {
            let t = step as f64 / steps as f64;
            let point = Point2D::new(
                pair[0].x + (pair[1].x - pair[0].x) * t,
                pair[0].y + (pair[1].y - pair[0].y) * t,
            );
            for obstacle in obstacles {
                clearance = clearance.min(point.distance(obstacle));
            }
        }
    }
    clearance
}

#[test]
fn every_tier1_planner_solves_the_same_map_through_the_trait() {
    let obstacles = obstacle_points();
    for (name, planner) in planners() {
        let path = planner
            .plan(START, GOAL)
            .unwrap_or_else(|error| panic!("{name} failed: {error}"));
        assert!(path.len() >= 2, "{name}: path has {} points", path.len());
        let first = path.points[0];
        let last = path.points[path.len() - 1];
        assert!(first.distance(&START) < 1.0, "{name}: starts at {first:?}");
        assert!(last.distance(&GOAL) < 1.0, "{name}: ends at {last:?}");
        let clearance = min_clearance(&path, &obstacles);
        assert!(clearance >= CLEARANCE, "{name}: clearance {clearance:.3} m");
        // The wall forces a detour: no planner may cut straight through.
        assert!(
            length(&path) > START.distance(&GOAL),
            "{name}: path too short"
        );
    }
}

#[test]
fn dijkstra_and_a_star_find_equally_short_paths() {
    let planners = planners();
    let length_of = |name: &str| {
        let (_, planner) = planners
            .iter()
            .find(|(candidate, _)| *candidate == name)
            .expect("planner exists");
        length(&planner.plan(START, GOAL).expect("path"))
    };
    let (a_star, dijkstra) = (length_of("A*"), length_of("Dijkstra"));
    assert!(
        (a_star - dijkstra).abs() < 1e-9,
        "A* {a_star:.6} m vs Dijkstra {dijkstra:.6} m"
    );
}
