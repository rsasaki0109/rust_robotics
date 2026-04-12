//! iLQR trajectory optimization visualization example.
//!
//! Runs iLQR from (0,0,0) to (10,10,0) and plots initial guess trajectory
//! vs optimized trajectory. Output:
//! `img/control/ilqr_trajectory.png`.

use gnuplot::{AxesCommon, Caption, Color, Figure, PointSymbol};
use nalgebra::Vector3;
use rust_robotics::control::ilqr::{ILQRConfig, ILQRPlanner};

fn linear_interpolation_trajectory(
    start: Vector3<f64>,
    goal: Vector3<f64>,
    horizon: usize,
) -> (Vec<f64>, Vec<f64>) {
    let mut x = Vec::with_capacity(horizon + 1);
    let mut y = Vec::with_capacity(horizon + 1);
    for i in 0..=horizon {
        let s = i as f64 / horizon as f64;
        x.push(start[0] + s * (goal[0] - start[0]));
        y.push(start[1] + s * (goal[1] - start[1]));
    }
    (x, y)
}

fn main() {
    std::fs::create_dir_all("img/control").expect("failed to create img/control directory");

    let start = Vector3::new(0.0, 0.0, 0.0);
    let goal = Vector3::new(10.0, 10.0, 0.0);
    let config = ILQRConfig {
        horizon: 150,
        dt: 0.1,
        max_iterations: 80,
        ..ILQRConfig::default()
    };
    let planner = ILQRPlanner::new(config);
    let result = planner.plan(start, goal);

    let (initial_x, initial_y) = linear_interpolation_trajectory(start, goal, config.horizon);
    let optimized_x: Vec<f64> = result.states.iter().map(|s| s[0]).collect();
    let optimized_y: Vec<f64> = result.states.iter().map(|s| s[1]).collect();

    let mut fg = Figure::new();
    {
        fg.axes2d()
            .set_title("iLQR Trajectory Optimization", &[])
            .set_x_label("x [m]", &[])
            .set_y_label("y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0))
            .lines(
                &initial_x,
                &initial_y,
                &[Caption("Initial Trajectory"), Color("gray")],
            )
            .lines(
                &optimized_x,
                &optimized_y,
                &[Caption("Optimized Trajectory"), Color("blue")],
            )
            .points(
                [start[0]],
                [start[1]],
                &[Caption("Start"), Color("green"), PointSymbol('O')],
            )
            .points(
                [goal[0]],
                [goal[1]],
                &[Caption("Goal"), Color("red"), PointSymbol('X')],
            );
    }

    let output_path = "img/control/ilqr_trajectory.png";
    fg.set_terminal("pngcairo", output_path);
    fg.show()
        .expect("failed to render gnuplot figure for ilqr trajectory");
    println!(
        "Saved {} (converged={}, iterations={}, cost={:.3})",
        output_path, result.converged, result.iterations, result.cost
    );
}
