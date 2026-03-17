//! Headless grid-based planning example.
//!
//! This example demonstrates how to use the library-first planning APIs
//! without enabling visualization dependencies.

use rust_robotics::planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics::planning::jps::{JPSConfig, JPSPlanner};
use rust_robotics::planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};
use rust_robotics::prelude::*;

fn create_obstacles() -> Obstacles {
    let mut obstacles = Obstacles::new();

    for i in 0..=20 {
        obstacles.push(Point2D::new(i as f64, 0.0));
        obstacles.push(Point2D::new(i as f64, 20.0));
        obstacles.push(Point2D::new(0.0, i as f64));
        obstacles.push(Point2D::new(20.0, i as f64));
    }

    for i in 5..15 {
        obstacles.push(Point2D::new(10.0, i as f64));
    }

    obstacles
}

fn main() -> RoboticsResult<()> {
    let obstacles = create_obstacles();
    let start = Point2D::new(2.0, 10.0);
    let goal = Point2D::new(18.0, 10.0);

    let a_star = AStarPlanner::from_obstacle_points(
        &obstacles,
        AStarConfig {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        },
    )?;
    let jps = JPSPlanner::from_obstacle_points(
        &obstacles,
        JPSConfig {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        },
    )?;
    let theta_star = ThetaStarPlanner::from_obstacle_points(
        &obstacles,
        ThetaStarConfig {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        },
    )?;

    let a_star_path = a_star.plan(start, goal)?;
    let jps_path = jps.plan(start, goal)?;
    let theta_star_path = theta_star.plan(start, goal)?;

    println!(
        "A*: {} waypoints, length {:.2}",
        a_star_path.len(),
        a_star_path.total_length()
    );
    println!(
        "JPS: {} waypoints, length {:.2}",
        jps_path.len(),
        jps_path.total_length()
    );
    println!(
        "Theta*: {} waypoints, length {:.2}",
        theta_star_path.len(),
        theta_star_path.total_length()
    );

    Ok(())
}
