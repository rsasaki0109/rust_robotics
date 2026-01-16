//! Theta* Path Planning Example
//!
//! Demonstrates the Theta* path planning algorithm which produces
//! any-angle paths by using line-of-sight checks.
//!
//! Theta* extends A* by allowing paths to connect any two visible nodes,
//! resulting in shorter, more natural paths compared to standard A*.

use rust_robotics::path_planning::theta_star::{ThetaStarPlanner, ThetaStarConfig};
use rust_robotics::path_planning::a_star::{AStarPlanner, AStarConfig};
use rust_robotics::common::{Point2D, PathPlanner};
use rust_robotics::utils::{Visualizer, PathStyle, colors};

fn main() {
    println!("Theta* path planning start!!");

    // Parameters
    let start = Point2D::new(2.0, 2.0);
    let goal = Point2D::new(18.0, 18.0);
    let grid_size = 1.0;
    let robot_radius = 0.5;

    println!("Setting up environment...");

    // Create boundary obstacles
    let mut ox = Vec::new();
    let mut oy = Vec::new();

    for i in 0..21 {
        ox.push(i as f64); oy.push(0.0);
        ox.push(i as f64); oy.push(20.0);
        ox.push(0.0); oy.push(i as f64);
        ox.push(20.0); oy.push(i as f64);
    }

    // Add internal obstacles (L-shaped wall)
    for i in 5..15 {
        ox.push(10.0);
        oy.push(i as f64);
    }
    for i in 5..12 {
        ox.push(i as f64);
        oy.push(15.0);
    }

    println!("Created {} obstacles", ox.len());

    // Create Theta* planner
    let theta_config = ThetaStarConfig {
        resolution: grid_size,
        robot_radius,
        heuristic_weight: 1.0,
    };
    let theta_planner = ThetaStarPlanner::new(&ox, &oy, theta_config);

    // Create A* planner for comparison
    let a_star_config = AStarConfig {
        resolution: grid_size,
        robot_radius,
        heuristic_weight: 1.0,
    };
    let a_star_planner = AStarPlanner::new(&ox, &oy, a_star_config);

    // Plan paths with both algorithms
    let theta_result = theta_planner.plan(start, goal);
    let a_star_result = a_star_planner.plan(start, goal);

    match (theta_result, a_star_result) {
        (Ok(theta_path), Ok(a_star_path)) => {
            println!("\n=== Results ===");
            println!("Theta* path: {} waypoints, length: {:.2}",
                theta_path.len(), theta_path.total_length());
            println!("A* path: {} waypoints, length: {:.2}",
                a_star_path.len(), a_star_path.total_length());
            println!("Theta* improvement: {:.1}% shorter path",
                (1.0 - theta_path.total_length() / a_star_path.total_length()) * 100.0);

            // Visualize
            let mut vis = Visualizer::new();
            vis.set_title("Theta* vs A* Path Planning");
            vis.plot_obstacles_xy(&ox, &oy);
            vis.plot_start(start);
            vis.plot_goal(goal);

            // Plot A* path in blue
            let a_star_style = PathStyle {
                color: colors::BLUE,
                line_width: 2.0,
                point_size: 3.0,
            };
            vis.plot_path(&a_star_path, &a_star_style);

            // Plot Theta* path in red (on top)
            let theta_style = PathStyle {
                color: colors::RED,
                line_width: 2.0,
                point_size: 5.0,
            };
            vis.plot_path(&theta_path, &theta_style);

            // Save and show
            let _ = vis.save_png("img/path_planning/theta_star_result.png", 800, 600);
            println!("\nPlot saved to: img/path_planning/theta_star_result.png");
            println!("(Red: Theta*, Blue: A*)");
            let _ = vis.show();
        }
        (Err(e), _) => {
            println!("Theta* planning failed: {}", e);
        }
        (_, Err(e)) => {
            println!("A* planning failed: {}", e);
        }
    }

    println!("\nTheta* path planning finish!!");
}
