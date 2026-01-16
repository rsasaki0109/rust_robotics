//! State Lattice Path Planning Example
//!
//! Demonstrates the State Lattice path planning algorithm.
//! Shows three different sampling strategies:
//! 1. Uniform polar sampling
//! 2. Biased polar sampling (toward a goal direction)
//! 3. Lane-based sampling (for structured environments)

use rust_robotics::path_planning::state_lattice::{
    StateLattice, StateLatticeConfig,
};
use rust_robotics::common::Point2D;
use rust_robotics::utils::{Visualizer, PathStyle, PointStyle, colors};

fn main() {
    println!("State Lattice Path Planning Example");
    println!("====================================\n");

    // Test 1: Uniform Polar Sampling
    test_uniform_polar_sampling();

    // Test 2: Biased Polar Sampling
    test_biased_polar_sampling();

    // Test 3: Lane Sampling
    test_lane_sampling();

    // Test 4: Full Planning Example
    test_full_planning();

    println!("\nState Lattice examples finished!");
}

fn test_uniform_polar_sampling() {
    println!("Test 1: Uniform Polar Sampling");
    println!("-------------------------------");

    let planner = StateLattice::with_defaults();
    let states = planner.calc_uniform_polar_states();
    println!("Generated {} target states", states.len());

    let paths = planner.plan_uniform_polar();
    println!("Generated {} valid paths", paths.len());

    // Visualize
    let mut vis = Visualizer::new();
    vis.set_title("State Lattice - Uniform Polar Sampling");
    vis.set_x_label("X [m]");
    vis.set_y_label("Y [m]");

    // Plot origin
    vis.plot_point(
        Point2D::origin(),
        &PointStyle::new(colors::GREEN, "Start").with_size(2.0),
    );

    // Plot target states
    for state in &states {
        vis.plot_point(
            Point2D::new(state.x, state.y),
            &PointStyle::new(colors::BLUE, "").with_size(1.0).with_symbol('x'),
        );
    }

    // Plot generated paths
    for (i, path) in paths.iter().enumerate() {
        let color = if i % 3 == 0 {
            colors::RED
        } else if i % 3 == 1 {
            colors::ORANGE
        } else {
            colors::PURPLE
        };
        let path2d = path.to_path();
        vis.plot_path(&path2d, &PathStyle::new(color, "").with_line_width(1.0));
    }

    if let Err(e) = vis.save_png("img/path_planning/state_lattice_uniform.png", 800, 600) {
        eprintln!("Failed to save PNG: {}", e);
    }
    if let Err(e) = vis.save_svg("img/path_planning/state_lattice_uniform.svg", 800, 600) {
        eprintln!("Failed to save SVG: {}", e);
    }
    println!("Plot saved to: img/path_planning/state_lattice_uniform.png/.svg");
    println!();
}

fn test_biased_polar_sampling() {
    println!("Test 2: Biased Polar Sampling (toward 30 degrees)");
    println!("--------------------------------------------------");

    let planner = StateLattice::with_defaults();
    let goal_angle = 30.0_f64.to_radians();

    let states = planner.calc_biased_polar_states(goal_angle);
    println!("Generated {} target states", states.len());

    let paths = planner.plan_biased_polar(goal_angle);
    println!("Generated {} valid paths", paths.len());

    // Visualize
    let mut vis = Visualizer::new();
    vis.set_title("State Lattice - Biased Polar Sampling");
    vis.set_x_label("X [m]");
    vis.set_y_label("Y [m]");

    // Plot origin
    vis.plot_point(
        Point2D::origin(),
        &PointStyle::new(colors::GREEN, "Start").with_size(2.0),
    );

    // Plot goal direction
    let goal_dist = 25.0;
    let goal_x = goal_dist * goal_angle.cos();
    let goal_y = goal_dist * goal_angle.sin();
    vis.plot_point(
        Point2D::new(goal_x, goal_y),
        &PointStyle::new(colors::BLUE, "Goal Dir").with_size(2.0).with_symbol('*'),
    );

    // Plot generated paths
    for path in &paths {
        let path2d = path.to_path();
        vis.plot_path(&path2d, &PathStyle::new(colors::RED, "").with_line_width(1.0));
    }

    let _ = vis.save_png("img/path_planning/state_lattice_biased.png", 800, 600);
    let _ = vis.save_svg("img/path_planning/state_lattice_biased.svg", 800, 600);
    println!("Plot saved to: img/path_planning/state_lattice_biased.png/.svg");
    println!();
}

fn test_lane_sampling() {
    println!("Test 3: Lane-based Sampling");
    println!("----------------------------");

    let mut config = StateLatticeConfig::default();
    config.lane_width = 3.5;
    config.vehicle_width = 1.8;
    config.lane_heading = 0.0;
    config.d = 30.0;

    let planner = StateLattice::new(config);
    let states = planner.calc_lane_states();
    println!("Generated {} target states", states.len());

    let paths = planner.plan_lane_states();
    println!("Generated {} valid paths", paths.len());

    // Visualize
    let mut vis = Visualizer::new();
    vis.set_title("State Lattice - Lane Sampling");
    vis.set_x_label("X [m]");
    vis.set_y_label("Y [m]");

    // Plot lane boundaries (simple representation)
    let lane_width = 3.5;
    let lane_length = 35.0;
    for y_offset in [-lane_width / 2.0, lane_width / 2.0] {
        let lane_points = vec![
            Point2D::new(0.0, y_offset),
            Point2D::new(lane_length, y_offset),
        ];
        let lane_path = rust_robotics::common::Path2D::from_points(lane_points);
        vis.plot_path(&lane_path, &PathStyle::new(colors::GRAY, "Lane").with_line_width(2.0));
    }

    // Plot origin
    vis.plot_point(
        Point2D::origin(),
        &PointStyle::new(colors::GREEN, "Start").with_size(2.0),
    );

    // Plot target states
    for state in &states {
        vis.plot_point(
            Point2D::new(state.x, state.y),
            &PointStyle::new(colors::BLUE, "").with_size(1.0).with_symbol('x'),
        );
    }

    // Plot generated paths
    for path in &paths {
        let path2d = path.to_path();
        vis.plot_path(&path2d, &PathStyle::new(colors::RED, "").with_line_width(1.0));
    }

    let _ = vis.save_png("img/path_planning/state_lattice_lane.png", 800, 600);
    let _ = vis.save_svg("img/path_planning/state_lattice_lane.svg", 800, 600);
    println!("Plot saved to: img/path_planning/state_lattice_lane.png/.svg");
    println!();
}

fn test_full_planning() {
    println!("Test 4: Full Path Planning");
    println!("---------------------------");

    let planner = StateLattice::with_defaults();

    let start = Point2D::new(0.0, 0.0);
    let goal = Point2D::new(30.0, 10.0);

    println!("Planning from ({}, {}) to ({}, {})", start.x, start.y, goal.x, goal.y);

    match planner.plan(start, goal) {
        Ok(path) => {
            println!("Path found with {} points", path.len());
            println!("Path length: {:.2} m", path.total_length());

            // Visualize
            let mut vis = Visualizer::new();
            vis.set_title("State Lattice - Path Planning");
            vis.set_x_label("X [m]");
            vis.set_y_label("Y [m]");

            vis.plot_start(start);
            vis.plot_goal(goal);
            vis.plot_path(&path, &PathStyle::new(colors::RED, "Path").with_line_width(2.0));

            let _ = vis.save_png("img/path_planning/state_lattice_plan.png", 800, 600);
            let _ = vis.save_svg("img/path_planning/state_lattice_plan.svg", 800, 600);
            println!("Plot saved to: img/path_planning/state_lattice_plan.png/.svg");
        }
        Err(e) => {
            println!("Planning failed: {}", e);
        }
    }
    println!();
}
