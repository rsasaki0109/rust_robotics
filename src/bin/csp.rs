// https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/CubicSpline
// Cubic spline planner
//
// Author: Atsushi Sakai(@Atsushi_twi)
//         Ryohei Sasaki(@rsasaki0109)
use rust_robotics::cubic_spline_planner;

use plotlib::page::Page;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::style::PointStyle;
use plotlib::style::LineStyle;

fn main() {
    let points = vec![
        (-2.5, 0.7),
        (0., -6.),
        (2.5, 5.),
        (5.0, 6.5),
        (7.5, 0.),
        (3.0, 5.),
        (-1.0, -2.),
    ];
    let ds = 0.01;

    let nx = points.len();
    let mut x: Vec<f64> = Vec::with_capacity(nx);
    let mut y: Vec<f64> = Vec::with_capacity(nx);

    for p in &points {
        x.push(p.0);
        y.push(p.1);
    }


    let sp = cubic_spline_planner::Spline2D::new(x, y);
    let s_end = sp.s[sp.s.len()-1]; 
    let n = (s_end / ds) as usize;
    let mut pos: Vec<(f64,f64)> = Vec::with_capacity(nx);
    for i in 0..n-1 {
        let pair = sp.clone().calc_position((i as f64/ (n -1)as f64)  * s_end);
        pos.push(pair);
    }


    let s0: Plot = Plot::new(points).point_style(
        PointStyle::new()
            .colour("#000000"),
    );

    let s1: Plot = Plot::new(pos).line_style(
        LineStyle::new() 
            .colour("#35C788")
            .width(2.),
    ); 


    let v = ContinuousView::new()
        .add(s0)
        .add(s1)
        .x_range(-5., 10.)
        .y_range(-7.5, 7.5)
        .x_label("x [m]")
        .y_label("y [m]");

    Page::single(&v).save("./img/csp.svg").unwrap();
}