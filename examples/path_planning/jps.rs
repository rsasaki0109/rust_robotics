//! Jump Point Search (JPS) Path Planning Example
//!
//! Demonstrates the JPS path planning algorithm on a grid with obstacles.
//! JPS is an optimization of A* that significantly reduces the number of
//! nodes expanded by identifying "jump points".

use rust_robotics::common::{Path2D, PathPlanner, Point2D};
use rust_robotics::path_planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics::path_planning::jps::{JPSConfig, JPSPlanner};
use std::fs::File;
use std::io::Write;

/// Generate SVG visualization of JPS path
fn generate_svg(
    ox: &[f64],
    oy: &[f64],
    jps_path: &Path2D,
    a_star_path: &Path2D,
    start: Point2D,
    goal: Point2D,
) -> String {
    let width = 700;
    let height = 700;
    let margin = 50.0;
    let plot_size = (width as f64 - 2.0 * margin, height as f64 - 2.0 * margin);

    // Find data bounds
    let min_x = ox.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_x = ox.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let min_y = oy.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_y = oy.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    let scale_x = plot_size.0 / (max_x - min_x);
    let scale_y = plot_size.1 / (max_y - min_y);
    let scale = scale_x.min(scale_y);

    let transform_x = |x: f64| margin + (x - min_x) * scale;
    let transform_y = |y: f64| height as f64 - margin - (y - min_y) * scale;

    let mut svg = String::new();

    // SVG header
    svg.push_str(&format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {} {}" width="{}" height="{}">
<style>
  .title {{ font: bold 16px sans-serif; }}
  .legend {{ font: 12px sans-serif; }}
</style>
<rect width="100%" height="100%" fill="white"/>
"#,
        width, height, width, height
    ));

    // Title
    svg.push_str(&format!(
        "<text x=\"{}\" y=\"30\" class=\"title\" text-anchor=\"middle\">Jump Point Search (JPS) Path Planning</text>\n",
        width / 2
    ));

    // Draw grid
    svg.push_str("<g stroke=\"#eee\" stroke-width=\"0.5\">\n");
    let grid_step = 10.0;
    let mut gx = min_x;
    while gx <= max_x {
        let x = transform_x(gx);
        svg.push_str(&format!(
            "<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\"/>\n",
            x,
            margin,
            x,
            height as f64 - margin
        ));
        gx += grid_step;
    }
    let mut gy = min_y;
    while gy <= max_y {
        let y = transform_y(gy);
        svg.push_str(&format!(
            "<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\"/>\n",
            margin,
            y,
            width as f64 - margin,
            y
        ));
        gy += grid_step;
    }
    svg.push_str("</g>\n");

    // Draw obstacles
    svg.push_str("<g fill=\"#333\">\n");
    for (&x, &y) in ox.iter().zip(oy.iter()) {
        let cx = transform_x(x);
        let cy = transform_y(y);
        svg.push_str(&format!(
            "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"8\" height=\"8\" rx=\"1\"/>\n",
            cx - 4.0,
            cy - 4.0
        ));
    }
    svg.push_str("</g>\n");

    // Draw A* path (blue, behind)
    if a_star_path.len() > 1 {
        svg.push_str("<polyline fill=\"none\" stroke=\"#0066CC\" stroke-width=\"2\" stroke-linecap=\"round\" stroke-linejoin=\"round\" stroke-opacity=\"0.5\" points=\"");
        for p in &a_star_path.points {
            svg.push_str(&format!("{:.1},{:.1} ", transform_x(p.x), transform_y(p.y)));
        }
        svg.push_str("\"/>\n");
    }

    // Draw JPS path (red, on top)
    if jps_path.len() > 1 {
        svg.push_str("<polyline fill=\"none\" stroke=\"#CC0000\" stroke-width=\"3\" stroke-linecap=\"round\" stroke-linejoin=\"round\" points=\"");
        for p in &jps_path.points {
            svg.push_str(&format!("{:.1},{:.1} ", transform_x(p.x), transform_y(p.y)));
        }
        svg.push_str("\"/>\n");
    }

    // Draw start point (green)
    svg.push_str(&format!(
        "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"8\" fill=\"#00CC00\" stroke=\"white\" stroke-width=\"2\"/>\n",
        transform_x(start.x),
        transform_y(start.y)
    ));
    svg.push_str(&format!(
        "<text x=\"{:.1}\" y=\"{:.1}\" class=\"legend\" text-anchor=\"middle\" fill=\"white\" font-weight=\"bold\">S</text>\n",
        transform_x(start.x),
        transform_y(start.y) + 4.0
    ));

    // Draw goal point (blue)
    svg.push_str(&format!(
        "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"8\" fill=\"#0000CC\" stroke=\"white\" stroke-width=\"2\"/>\n",
        transform_x(goal.x),
        transform_y(goal.y)
    ));
    svg.push_str(&format!(
        "<text x=\"{:.1}\" y=\"{:.1}\" class=\"legend\" text-anchor=\"middle\" fill=\"white\" font-weight=\"bold\">G</text>\n",
        transform_x(goal.x),
        transform_y(goal.y) + 4.0
    ));

    // Legend
    let legend_x = width as f64 - 180.0;
    let legend_y = height as f64 - 60.0;
    svg.push_str(&format!(
        "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"170\" height=\"50\" fill=\"white\" stroke=\"#ccc\" rx=\"5\"/>\n",
        legend_x, legend_y
    ));
    svg.push_str(&format!(
        "<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" stroke=\"#CC0000\" stroke-width=\"3\"/>\n",
        legend_x + 10.0, legend_y + 18.0, legend_x + 40.0, legend_y + 18.0
    ));
    svg.push_str(&format!(
        "<text x=\"{:.1}\" y=\"{:.1}\" class=\"legend\">JPS Path</text>\n",
        legend_x + 50.0, legend_y + 22.0
    ));
    svg.push_str(&format!(
        "<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" stroke=\"#0066CC\" stroke-width=\"2\" stroke-opacity=\"0.5\"/>\n",
        legend_x + 10.0, legend_y + 38.0, legend_x + 40.0, legend_y + 38.0
    ));
    svg.push_str(&format!(
        "<text x=\"{:.1}\" y=\"{:.1}\" class=\"legend\">A* Path</text>\n",
        legend_x + 50.0, legend_y + 42.0
    ));

    svg.push_str("</svg>\n");
    svg
}

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

    // Create JPS planner
    let jps_config = JPSConfig {
        resolution: grid_size,
        robot_radius,
        heuristic_weight: 1.0,
    };
    let jps_planner = JPSPlanner::new(&ox, &oy, jps_config);

    // Create A* planner for comparison
    let a_star_config = AStarConfig {
        resolution: grid_size,
        robot_radius,
        heuristic_weight: 1.0,
    };
    let a_star_planner = AStarPlanner::new(&ox, &oy, a_star_config);

    println!("Planning path from {:?} to {:?}...", start, goal);

    // Plan with JPS
    let jps_start = std::time::Instant::now();
    let jps_result = jps_planner.plan(start, goal);
    let jps_time = jps_start.elapsed();

    // Plan with A*
    let a_star_start = std::time::Instant::now();
    let a_star_result = a_star_planner.plan(start, goal);
    let a_star_time = a_star_start.elapsed();

    match (&jps_result, &a_star_result) {
        (Ok(jps_path), Ok(a_star_path)) => {
            println!("\n=== Results ===");
            println!(
                "JPS: {} waypoints, length: {:.2}, time: {:?}",
                jps_path.len(),
                jps_path.total_length(),
                jps_time
            );
            println!(
                "A*:  {} waypoints, length: {:.2}, time: {:?}",
                a_star_path.len(),
                a_star_path.total_length(),
                a_star_time
            );

            // Generate and save SVG
            let svg = generate_svg(&ox, &oy, jps_path, a_star_path, start, goal);

            // Use absolute path from CARGO_MANIFEST_DIR
            let abs_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("img/path_planning/jps_result.svg");

            match File::create(&abs_path) {
                Ok(mut file) => {
                    file.write_all(svg.as_bytes()).unwrap();
                    println!("\nSVG saved to: {}", abs_path.display());
                }
                Err(e) => {
                    println!("Failed to save SVG: {}", e);
                }
            }

            println!("(Red: JPS, Blue: A*)");
        }
        _ => {
            println!("Planning failed!");
            if let Err(e) = &jps_result {
                println!("JPS error: {}", e);
            }
            if let Err(e) = &a_star_result {
                println!("A* error: {}", e);
            }
        }
    }

    println!("\nJump Point Search (JPS) path planning finish!!");
}
