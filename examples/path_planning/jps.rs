//! Jump Point Search (JPS) Path Planning Example
//!
//! Demonstrates the JPS path planning algorithm on a grid with obstacles.
//! JPS is an optimization of A* that significantly reduces the number of
//! nodes expanded by identifying "jump points".

use rust_robotics::common::{PathPlanner, Point2D};
use rust_robotics::path_planning::jps::{JPSConfig, JPSPlanner};
use rust_robotics::utils::{PathStyle, Visualizer};

fn main() {
    println!("Jump Point Search (JPS) path planning start!!");

    // Parameters
    let start = Point2D::new(10.0, 10.0);
    let goal = Point2D::new(50.0, 50.0);
    let grid_size = 1.0;
    let robot_radius = 0.5;

    println!("Setting up environment...");

    // Create boundary obstacles
    let mut ox = Vec::new();
    let mut oy = Vec::new();

    // Outer boundary
    for i in 0..61 {
        ox.push(i as f64);
        oy.push(0.0);
        ox.push(i as f64);
        oy.push(60.0);
        ox.push(0.0);
        oy.push(i as f64);
        ox.push(60.0);
        oy.push(i as f64);
    }

    // Internal obstacles - vertical walls with gaps
    for i in 10..50 {
        ox.push(20.0);
        oy.push(i as f64);
    }
    for i in 10..50 {
        ox.push(40.0);
        oy.push(i as f64);
    }

    // Horizontal wall
    for i in 20..41 {
        ox.push(i as f64);
        oy.push(30.0);
    }

    println!("Created {} obstacle points", ox.len());

    // Create planner with default config
    let config = JPSConfig {
        resolution: grid_size,
        robot_radius,
        heuristic_weight: 1.0,
    };
    let planner = JPSPlanner::new(&ox, &oy, config);

    println!("Planning path from {:?} to {:?}...", start, goal);

    // Plan path
    let start_time = std::time::Instant::now();
    match planner.plan(start, goal) {
        Ok(path) => {
            let elapsed = start_time.elapsed();
            println!("Path found with {} waypoints", path.len());
            println!("Planning time: {:?}", elapsed);
            println!("Path total length: {:.2}", path.total_length());

            // Print waypoints
            println!("\nWaypoints:");
            for (i, point) in path.points.iter().enumerate() {
                println!("  {}: ({:.1}, {:.1})", i, point.x, point.y);
            }

            // Visualize
            let mut vis = Visualizer::new();
            vis.set_title("Jump Point Search (JPS) Path Planning");
            vis.plot_obstacles_xy(&ox, &oy);
            vis.plot_start(start);
            vis.plot_goal(goal);
            vis.plot_path(&path, &PathStyle::default());

            // Save and show
            match vis.save_png("img/path_planning/jps_result.png", 800, 600) {
                Ok(_) => println!("\nPNG saved to: img/path_planning/jps_result.png"),
                Err(e) => println!("\nFailed to save PNG: {}", e),
            }
            match vis.save_svg("img/path_planning/jps_result.svg", 800, 600) {
                Ok(_) => println!("SVG saved to: img/path_planning/jps_result.svg"),
                Err(e) => println!("Failed to save SVG: {}", e),
            }
            let _ = vis.show();
        }
        Err(e) => {
            println!("Planning failed: {}", e);
        }
    }

    println!("\nJump Point Search (JPS) path planning finish!!");
}
