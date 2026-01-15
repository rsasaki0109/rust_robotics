//! A* Path Planning Example
//!
//! Demonstrates the A* path planning algorithm on a simple grid with obstacles.

use rust_robotics::path_planning::a_star::{AStarPlanner, AStarConfig};
use rust_robotics::common::{Point2D, PathPlanner};
use rust_robotics::utils::{Visualizer, PathStyle, PointStyle, colors};

fn main() {
    println!("A* path planning start!!");

    // Parameters
    let start = Point2D::new(2.0, 2.0);
    let goal = Point2D::new(8.0, 8.0);
    let grid_size = 1.0;
    let robot_radius = 0.5;

    println!("Setting up environment...");

    // Create boundary obstacles
    let mut ox = Vec::new();
    let mut oy = Vec::new();

    for i in 0..11 {
        ox.push(i as f64); oy.push(0.0);
        ox.push(i as f64); oy.push(10.0);
        ox.push(0.0); oy.push(i as f64);
        ox.push(10.0); oy.push(i as f64);
    }

    // Add internal obstacle
    for i in 4..7 {
        ox.push(5.0);
        oy.push(i as f64);
    }

    println!("Created {} obstacles", ox.len());

    // Create planner
    let config = AStarConfig {
        resolution: grid_size,
        robot_radius,
        heuristic_weight: 1.0,
    };
    let planner = AStarPlanner::new(&ox, &oy, config);

    // Plan path
    match planner.plan(start, goal) {
        Ok(path) => {
            println!("Path found with {} points", path.len());

            // Visualize
            let mut vis = Visualizer::new();
            vis.set_title("A* Path Planning");
            vis.plot_obstacles_xy(&ox, &oy);
            vis.plot_start(start);
            vis.plot_goal(goal);
            vis.plot_path(&path, &PathStyle::default());

            // Save and show
            let _ = vis.save_png("img/path_planning/a_star_result.png", 800, 600);
            println!("Plot saved to: img/path_planning/a_star_result.png");
            let _ = vis.show();
        }
        Err(e) => {
            println!("Planning failed: {}", e);
        }
    }

    println!("A* path planning finish!!");
}
