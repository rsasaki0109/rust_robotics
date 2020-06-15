//
// Path planning with Bezier curve.
//
// author: Atsushi Sakai(@Atsushi_twi)
//         Ryohei Sasaki(@rsasaki0109)

use plotlib::page::Page;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::style::PointStyle;
use plotlib::style::LineStyle;

fn calc_4points_bezier_path(
    x_start: (f64, f64, f64), x_goal: (f64, f64, f64), offset: f64)
-> (Vec<(f64, f64)>, [(f64, f64) ;4])
{
    let x_diff = (x_start.0 - x_goal.0, x_start.1 - x_goal.1);
    let dist = ((x_diff.0).powi(2) + (x_diff.1).powi(2)).sqrt() / offset;
    let control_points = [
        (x_start.0, x_start.1),
        (x_start.0 + dist * (x_start.2).cos(), x_start.1 + dist * (x_start.2).sin()),
        (x_goal.0 - dist * (x_goal.2).cos(), x_goal.1 - dist * (x_goal.2).sin()),
        (x_goal.0, x_goal.1)
        ]; 
    let path = calc_bezier_path(control_points, 100);
    (path, control_points)
}

fn calc_bezier_path(
    control_points: [(f64, f64) ;4], n_points: usize)
-> Vec<(f64, f64)>
{
    let mut traj : Vec<(f64, f64)> = Vec::with_capacity(n_points);
    for i in 0..n_points-1 {
        let t = (i as f64) / ((n_points as f64) - 1.);
        traj.push(bezier(t, control_points));
    }
    traj
}

fn bernstein_poly(n: i32, i: i32, t: f64) -> f64
{
    (binom(n, i) as f64) * t.powi(i) * (1. - t).powi(n - i)
}

fn bezier(
    t: f64, control_points: [(f64, f64) ;4]
) -> (f64, f64)
{
    let n = control_points.len()-1;
    let mut point = (0., 0.);
    for i in 0..n+1 {
        let ber_poly = bernstein_poly(n as i32, i as i32, t);
        point.0 += ber_poly * control_points[i].0;
        point.1 += ber_poly * control_points[i].1;
    }
    point
}

// Ref:Donald E.Knuth, "The Art of Computer Programming (2)"
fn binom(n: i32, k: i32) -> i32 {
    (0..n + 1)
        .rev()
        .zip(1..k + 1)
        .fold(1, |mut r, (n, d)| { r *= n; r /= d; r })
}

#[warn(dead_code)]
fn bezier_derivatives_control_points(
    control_points: [(f64, f64) ;4], n_derivatives: usize) -> Vec<[(f64, f64) ;4]> {
    let mut w = vec![control_points];
    let mut tmp :[(f64, f64) ;4] = Default::default(); 
    let n = w.len(); 
    for i in 0..n_derivatives-1 {
        for j in 0..n-1 {
            tmp[j].0 = ((n - 1) as f64) * (w[i][j + 1].0 - w[i][j].0);
            tmp[j].1 = ((n - 1) as f64) * (w[i][j + 1].1 - w[i][j].1);
        }
        w.push(tmp);
    }
    w
}

#[warn(dead_code)]
fn curvature(dx: f64, dy: f64, ddx: f64, ddy: f64) -> f64 {
    (dx * ddy - dy * ddx) / (dx.powi(2) + dy.powi(2)).powf(3. / 2.)
}

fn main() {
    // [x, y, yaw]
    let start_x = (10., 1., 180.0/180.0 * std::f64::consts::PI);
    let end_x = (0., -3., - 45.0/180.0 * std::f64::consts::PI);

    let offset = 3.;

    let pair = calc_4points_bezier_path(start_x, end_x, offset);
    let path = pair.0;
    let control_points = pair.1;

    let mut arrow_start = vec![(start_x.0, start_x.1)];
    arrow_start.push((start_x.0 + (start_x.2).cos(), start_x.1 + (start_x.2).sin()));
    let mut arrow_goal = vec![(end_x.0, end_x.1)];
    arrow_goal.push((end_x.0 + (end_x.2).cos(), end_x.1 + (end_x.2).sin()));

    let s0: Plot = Plot::new(control_points.to_vec()).point_style(
        PointStyle::new()
            .colour("#000000"),
    );

    let s1: Plot = Plot::new(path).line_style(
        LineStyle::new() 
            .colour("#35C788"),
    );

    let s2: Plot = Plot::new(arrow_start).line_style(
        LineStyle::new() 
            .colour("#DD3355")
            .width(2.),
    );

    let s3: Plot = Plot::new(arrow_goal).line_style(
        LineStyle::new() 
            .colour("#DD3355")
            .width(2.),
    ); 

    let v = ContinuousView::new()
        .add(s0)
        .add(s1)
        .add(s2)
        .add(s3)
        .x_range(-5., 15.)
        .y_range(-4., 2.)
        .x_label("x [m]")
        .y_label("y [m]");

    Page::single(&v).save("./img/bezier_path.svg").unwrap();
}