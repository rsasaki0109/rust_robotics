//! Any-Angle Planner Comparison
//!
//! Compares A*, Theta*, Lazy Theta*, and Enhanced Lazy Theta* on the same
//! 50x50 grid with a wall obstacle.  Prints path lengths and waypoint counts,
//! and optionally plots all paths on a single figure when the "viz" feature
//! is enabled.

use rust_robotics::planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics::planning::enhanced_lazy_theta_star::{
    EnhancedLazyThetaStarConfig, EnhancedLazyThetaStarPlanner,
};
use rust_robotics::planning::lazy_theta_star::{LazyThetaStarConfig, LazyThetaStarPlanner};
use rust_robotics::planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};
use rust_robotics::prelude::*;

fn create_obstacles() -> Obstacles {
    let mut obstacles = Obstacles::new();

    // Boundary walls (50x50 grid, indices 0..=50)
    for i in 0..=50 {
        obstacles.push(Point2D::new(i as f64, 0.0));
        obstacles.push(Point2D::new(i as f64, 50.0));
        obstacles.push(Point2D::new(0.0, i as f64));
        obstacles.push(Point2D::new(50.0, i as f64));
    }

    // Vertical wall obstacle from (25, 10) to (25, 40)
    for i in 10..=40 {
        obstacles.push(Point2D::new(25.0, i as f64));
    }

    obstacles
}

fn main() -> RoboticsResult<()> {
    let obstacles = create_obstacles();
    let start = Point2D::new(2.0, 2.0);
    let goal = Point2D::new(48.0, 48.0);
    let resolution = 1.0;
    let robot_radius = 0.5;

    // --- A* ---
    let a_star = AStarPlanner::from_obstacle_points(
        &obstacles,
        AStarConfig {
            resolution,
            robot_radius,
            heuristic_weight: 1.0,
        },
    )?;
    let a_star_path = a_star.plan(start, goal)?;

    // --- Theta* ---
    let theta_star = ThetaStarPlanner::from_obstacle_points(
        &obstacles,
        ThetaStarConfig {
            resolution,
            robot_radius,
            heuristic_weight: 1.0,
        },
    )?;
    let theta_star_path = theta_star.plan(start, goal)?;

    // --- Lazy Theta* ---
    let lazy_theta_star = LazyThetaStarPlanner::from_obstacle_points(
        &obstacles,
        LazyThetaStarConfig {
            resolution,
            robot_radius,
            heuristic_weight: 1.0,
        },
    )?;
    let lazy_theta_star_path = lazy_theta_star.plan(start, goal)?;

    // --- Enhanced Lazy Theta* ---
    let enhanced = EnhancedLazyThetaStarPlanner::from_obstacle_points(
        &obstacles,
        EnhancedLazyThetaStarConfig {
            resolution,
            robot_radius,
            heuristic_weight: 1.0,
        },
    )?;
    let enhanced_path = enhanced.plan(start, goal)?;

    // Print results
    println!("=== Any-Angle Planner Comparison (50x50 grid) ===");
    println!(
        "A*:                    {} waypoints, length {:.2}",
        a_star_path.len(),
        a_star_path.total_length()
    );
    println!(
        "Theta*:                {} waypoints, length {:.2}",
        theta_star_path.len(),
        theta_star_path.total_length()
    );
    println!(
        "Lazy Theta*:           {} waypoints, length {:.2}",
        lazy_theta_star_path.len(),
        lazy_theta_star_path.total_length()
    );
    println!(
        "Enhanced Lazy Theta*:  {} waypoints, length {:.2}",
        enhanced_path.len(),
        enhanced_path.total_length()
    );

    // Visualization (only when the "viz" feature is enabled)
    #[cfg(feature = "viz")]
    {
        use rust_robotics::viz::{colors, PathStyle, Visualizer};

        let mut vis = Visualizer::new();
        vis.set_title("Any-Angle Planner Comparison");
        vis.plot_obstacles(&obstacles);
        vis.plot_start(start);
        vis.plot_goal(goal);

        vis.plot_path(
            &a_star_path,
            &PathStyle::new(colors::RED, "A*").with_line_width(2.0),
        );
        vis.plot_path(
            &theta_star_path,
            &PathStyle::new(colors::BLUE, "Theta*").with_line_width(2.0),
        );
        vis.plot_path(
            &lazy_theta_star_path,
            &PathStyle::new(colors::ORANGE, "Lazy Theta*").with_line_width(2.0),
        );
        vis.plot_path(
            &enhanced_path,
            &PathStyle::new(colors::PURPLE, "Enhanced Lazy Theta*").with_line_width(2.0),
        );

        let _ = vis.save_png(
            "img/path_planning/any_angle_comparison.png",
            800,
            600,
        );
        println!("\nPlot saved to: img/path_planning/any_angle_comparison.png");
        let _ = vis.show();
    }

    Ok(())
}
