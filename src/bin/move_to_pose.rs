// 
// Move to specified pose
// Author: Daniel Ingram (daniel-s-ingram)
//         Atsushi Sakai(@Atsushi_twi)
//         Ryohei Sasaki(@rsasaki0109) 
// P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7
use plotlib::page::Page;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::style::LineStyle;

fn move_to_pose(
    x_start: (f64, f64, f64), x_goal: (f64, f64, f64), kp: (f64, f64, f64), dt: f64)
-> Vec<(f64, f64)>
{
    let mut x = x_start;
    let mut x_diff = (x_goal.0 - x.0, x_goal.1 - x.1);
    let mut rho = ((x_diff.0).powi(2) + (x_diff.1).powi(2)).sqrt();
    let mut x_traj = vec![(x.0, x.1)];
    while rho > 0.001 {
        x_diff = (x_goal.0 - x.0, x_goal.1 - x.1);
        rho = ((x_diff.0).powi(2) + (x_diff.1).powi(2)).sqrt();
        let alpha = ((x_diff.1).atan2(x_diff.0) - x.2 + std::f64::consts::PI) %
            (2. * std::f64::consts::PI) - std::f64::consts::PI;
        let beta = (x_goal.2 - x.2 - alpha + std::f64::consts::PI) %
            (2. * std::f64::consts::PI) - std::f64::consts::PI;
        let mut v = kp.0 * rho;
        let w = kp.1 * alpha + kp.2 * beta;
        if alpha > std::f64::consts::PI /2. || alpha < -std::f64::consts::PI /2. {
            v = -v;
        }
        x.2 = x.2 + w * dt;
        x.0 = x.0 + v * (x.2).cos() * dt;
        x.1 = x.1 + v * (x.2).sin() * dt;
        x_traj.push((x.0, x.1));
    }
    x_traj
}
fn main() {
    let x_start = (0., 0., 0.0 * std::f64::consts::PI);
    let x_goal = (10., 10., - 1.5 * std::f64::consts::PI);
    // [kp_rho, kp_alpha, kp_beta]
    let kp = (9. , 15., -3.);
    let dt = 0.01;
    let x_trag = move_to_pose(x_start, x_goal, kp, dt);

    let mut arrow_start = vec![(x_start.0, x_start.1)];
    arrow_start.push((x_start.0 + (x_start.2).cos(), x_start.1 + (x_start.2).sin()));
    let mut arrow_goal = vec![(x_goal.0, x_goal.1)];
    arrow_goal.push((x_goal.0 + (x_goal.2).cos(), x_goal.1 + (x_goal.2).sin()));

    let s1: Plot = Plot::new(x_trag).line_style(
        LineStyle::new() 
            .colour("#35C788")
            .width(2.),
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
    .add(s1)
    .add(s2)
    .add(s3)
    .x_range(-0., 15.)
    .y_range(0., 15.)
    .x_label("x [m]")
    .y_label("y [m]");

    Page::single(&v).save("./img/move_to_pose.svg").unwrap();

}