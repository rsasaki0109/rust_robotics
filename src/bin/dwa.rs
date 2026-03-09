use std::fs;

use gnuplot::{AxesCommon, Caption, Color, Figure, PointSize, PointSymbol};
use rust_robotics::path_planning::{DWAPlanner, DWAState};
use rust_robotics::Point2D;

fn main() {
    println!("DWA demo start");

    fs::create_dir_all("img/path_planning").unwrap();

    let mut planner = DWAPlanner::with_defaults();
    planner.set_state(DWAState::new(0.0, 0.0, 0.0, 0.0, 0.0));
    planner.set_goal(Point2D::new(10.0, 10.0));

    let obstacles = vec![
        Point2D::new(2.5, 2.0),
        Point2D::new(3.0, 4.0),
        Point2D::new(4.5, 4.0),
        Point2D::new(5.0, 6.0),
        Point2D::new(6.5, 6.0),
        Point2D::new(7.0, 8.0),
        Point2D::new(8.5, 8.0),
    ];
    planner.set_obstacles(obstacles.clone());

    let states = planner.navigate_to_goal(250);
    let path_x: Vec<f64> = states.iter().map(|s| s[0]).collect();
    let path_y: Vec<f64> = states.iter().map(|s| s[1]).collect();
    let obs_x: Vec<f64> = obstacles.iter().map(|p| p.x).collect();
    let obs_y: Vec<f64> = obstacles.iter().map(|p| p.y).collect();

    let mut fig = Figure::new();
    {
        let axes = fig.axes2d();
        axes.points(
            &obs_x,
            &obs_y,
            &[
                Caption("Obstacles"),
                Color("black"),
                PointSymbol('O'),
                PointSize(1.5),
            ],
        );
        axes.lines(&path_x, &path_y, &[Caption("Trajectory"), Color("green")]);
        axes.points(
            [0.0],
            [0.0],
            &[
                Caption("Start"),
                Color("blue"),
                PointSymbol('O'),
                PointSize(2.0),
            ],
        );
        axes.points(
            [10.0],
            [10.0],
            &[
                Caption("Goal"),
                Color("red"),
                PointSymbol('O'),
                PointSize(2.0),
            ],
        );
        axes.set_title("Dynamic Window Approach", &[])
            .set_x_label("X [m]", &[])
            .set_y_label("Y [m]", &[])
            .set_aspect_ratio(gnuplot::Fix(1.0));
    }

    fig.save_to_svg("./img/path_planning/dwa.svg", 640, 480)
        .unwrap();
    println!("Saved plot to ./img/path_planning/dwa.svg");
}
