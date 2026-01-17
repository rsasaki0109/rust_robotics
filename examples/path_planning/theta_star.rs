//! Theta* Path Planning Example
//!
//! Demonstrates the Theta* path planning algorithm which produces
//! any-angle paths by using line-of-sight checks.
//!
//! Theta* extends A* by allowing paths to connect any two visible nodes,
//! resulting in shorter, more natural paths compared to standard A*.

use rust_robotics::path_planning::theta_star::{ThetaStarPlanner, ThetaStarConfig};
use rust_robotics::path_planning::a_star::{AStarPlanner, AStarConfig};
use rust_robotics::common::{Point2D, Path2D, PathPlanner};
use std::fs::File;
use std::io::Write;

/// Generate SVG visualization of paths
fn generate_svg(
    ox: &[f64],
    oy: &[f64],
    theta_path: &Path2D,
    a_star_path: &Path2D,
    start: Point2D,
    goal: Point2D,
) -> String {
    let width = 600;
    let height = 600;
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
  .axis-label {{ font: 12px sans-serif; }}
  .legend {{ font: 12px sans-serif; }}
</style>
<rect width="100%" height="100%" fill="white"/>
"#,
        width, height, width, height
    ));

    // Title
    svg.push_str(&format!(
        "<text x=\"{}\" y=\"30\" class=\"title\" text-anchor=\"middle\">Theta* vs A* Path Planning</text>\n",
        width / 2
    ));

    // Draw grid
    svg.push_str("<g stroke=\"#eee\" stroke-width=\"0.5\">\n");
    for i in 0..=20 {
        let x = transform_x(i as f64);
        let y = transform_y(i as f64);
        svg.push_str(&format!(
            "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\"/>\n",
            x, margin, x, height as f64 - margin
        ));
        svg.push_str(&format!(
            "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\"/>\n",
            margin, y, width as f64 - margin, y
        ));
    }
    svg.push_str("</g>\n");

    // Draw obstacles
    svg.push_str("<g fill=\"#333\">\n");
    for (&x, &y) in ox.iter().zip(oy.iter()) {
        let cx = transform_x(x);
        let cy = transform_y(y);
        svg.push_str(&format!(
            "<rect x=\"{}\" y=\"{}\" width=\"8\" height=\"8\" rx=\"1\"/>\n",
            cx - 4.0, cy - 4.0
        ));
    }
    svg.push_str("</g>\n");

    // Draw A* path (blue, behind)
    if a_star_path.len() > 1 {
        svg.push_str("<polyline fill=\"none\" stroke=\"#0066CC\" stroke-width=\"2\" stroke-linecap=\"round\" stroke-linejoin=\"round\" points=\"");
        for p in &a_star_path.points {
            svg.push_str(&format!("{:.1},{:.1} ", transform_x(p.x), transform_y(p.y)));
        }
        svg.push_str("\"/>\n");

        // Draw A* waypoints
        svg.push_str("<g fill=\"#0066CC\">\n");
        for p in &a_star_path.points {
            svg.push_str(&format!(
                "<circle cx=\"{}\" cy=\"{}\" r=\"3\"/>\n",
                transform_x(p.x), transform_y(p.y)
            ));
        }
        svg.push_str("</g>\n");
    }

    // Draw Theta* path (red, on top)
    if theta_path.len() > 1 {
        svg.push_str("<polyline fill=\"none\" stroke=\"#CC0000\" stroke-width=\"3\" stroke-linecap=\"round\" stroke-linejoin=\"round\" points=\"");
        for p in &theta_path.points {
            svg.push_str(&format!("{:.1},{:.1} ", transform_x(p.x), transform_y(p.y)));
        }
        svg.push_str("\"/>\n");

        // Draw Theta* waypoints
        svg.push_str("<g fill=\"#CC0000\">\n");
        for p in &theta_path.points {
            svg.push_str(&format!(
                "<circle cx=\"{}\" cy=\"{}\" r=\"5\"/>\n",
                transform_x(p.x), transform_y(p.y)
            ));
        }
        svg.push_str("</g>\n");
    }

    // Draw start point (green)
    svg.push_str(&format!(
        "<circle cx=\"{}\" cy=\"{}\" r=\"8\" fill=\"#00AA00\" stroke=\"white\" stroke-width=\"2\"/>\n",
        transform_x(start.x), transform_y(start.y)
    ));
    svg.push_str(&format!(
        "<text x=\"{}\" y=\"{}\" class=\"axis-label\" fill=\"#00AA00\">S</text>\n",
        transform_x(start.x) + 12.0, transform_y(start.y) + 4.0
    ));

    // Draw goal point (blue)
    svg.push_str(&format!(
        "<circle cx=\"{}\" cy=\"{}\" r=\"8\" fill=\"#0000AA\" stroke=\"white\" stroke-width=\"2\"/>\n",
        transform_x(goal.x), transform_y(goal.y)
    ));
    svg.push_str(&format!(
        "<text x=\"{}\" y=\"{}\" class=\"axis-label\" fill=\"#0000AA\">G</text>\n",
        transform_x(goal.x) + 12.0, transform_y(goal.y) + 4.0
    ));

    // Legend
    let legend_x = width as f64 - 140.0;
    let legend_y = 50.0;
    svg.push_str(&format!(
        "<rect x=\"{}\" y=\"{}\" width=\"130\" height=\"70\" fill=\"white\" stroke=\"#ccc\" rx=\"4\"/>\n",
        legend_x, legend_y
    ));
    // A* legend
    svg.push_str(&format!(
        "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"#0066CC\" stroke-width=\"2\"/>\n",
        legend_x + 10.0, legend_y + 25.0, legend_x + 40.0, legend_y + 25.0
    ));
    svg.push_str(&format!(
        "<text x=\"{}\" y=\"{}\" class=\"legend\">A* ({} pts)</text>\n",
        legend_x + 50.0, legend_y + 29.0, a_star_path.len()
    ));
    // Theta* legend
    svg.push_str(&format!(
        "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"#CC0000\" stroke-width=\"3\"/>\n",
        legend_x + 10.0, legend_y + 50.0, legend_x + 40.0, legend_y + 50.0
    ));
    svg.push_str(&format!(
        "<text x=\"{}\" y=\"{}\" class=\"legend\">Theta* ({} pts)</text>\n",
        legend_x + 50.0, legend_y + 54.0, theta_path.len()
    ));

    svg.push_str("</svg>\n");
    svg
}

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

            // Generate and save SVG
            let base_path = std::env::current_dir()
                .map(|p| p.join("img/path_planning"))
                .unwrap_or_else(|_| std::path::PathBuf::from("img/path_planning"));
            std::fs::create_dir_all(&base_path).ok();

            let svg_path = base_path.join("theta_star_result.svg");
            let svg_content = generate_svg(&ox, &oy, &theta_path, &a_star_path, start, goal);

            match File::create(&svg_path) {
                Ok(mut file) => {
                    if let Err(e) = file.write_all(svg_content.as_bytes()) {
                        println!("Warning: Failed to write SVG: {}", e);
                    } else {
                        println!("\nSVG saved to: {}", svg_path.display());
                    }
                }
                Err(e) => {
                    println!("Warning: Failed to create SVG file: {}", e);
                }
            }

            println!("(Red: Theta*, Blue: A*)");
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
