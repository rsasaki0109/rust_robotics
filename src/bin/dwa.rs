// https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DynamicWindowApproach/dynamic_window_approach.py
// 
// Mobile robot motion planning sample with Dynamic Window Approach
// author: Atsushi Sakai (@Atsushi_twi)
//         Göktuğ Karakaşlı
//         Ryohei Sasaki (@rsasaki0109)
extern crate nalgebra as na;

use plotlib::page::Page;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::style::PointStyle;
use plotlib::style::LineStyle;

fn dwa_control(x: nalgebra::Vector5<f64>, config: &Config, goal: (f64, f64), ob: &Vec<(f64, f64)>) 
-> ((f64, f64), Vec<nalgebra::Vector5<f64>>)
{
    let dw = calc_dynamic_window(x, &config);
    let pair = calc_control_and_trajectory(x, dw, &config, goal, &ob);
    pair
}

struct Config{
    max_speed: f64,
    min_speed: f64,
    max_yaw_rate: f64,
    max_accel: f64,
    max_delta_yaw_rate: f64,
    v_resolution: f64,
    yaw_rate_resolution: f64,
    dt: f64,
    predict_time: f64,
    to_goal_cost_gain: f64,
    speed_cost_gain: f64,
    obstacle_cost_gain: f64,
    robot_radius: f64,
}

fn motion(mut x: nalgebra::Vector5<f64>, u: nalgebra::Vector2<f64>, dt: f64)
-> nalgebra::Vector5<f64>
{
    x[2] += u[1] * dt;
    x[0] += u[0] * x[2].cos() * dt;
    x[1] += u[0] * x[2].sin() * dt;
    x[3] = u[0];
    x[4] = u[1];
    x
}

fn calc_dynamic_window(x: nalgebra::Vector5<f64>, config: &Config) -> nalgebra::Vector4<f64>
{
    let vs = nalgebra::Vector4::new(config.min_speed, config.max_speed,
        -config.max_yaw_rate, config.max_yaw_rate);
    let vd = nalgebra::Vector4::new(
        x[3] - config.max_accel * config.dt,
        x[3] + config.max_accel * config.dt,
        x[4] - config.max_delta_yaw_rate * config.dt,
        x[4] + config.max_delta_yaw_rate * config.dt);    
    let dw = nalgebra::Vector4::new(
        if vs[0] > vd[0] {vs[0]} else { vd[0] },
        if vs[1] < vd[1] {vs[1]} else { vd[1] },
        if vs[2] > vd[2] {vs[2]} else { vd[2] },
        if vs[3] < vd[3] {vs[3]} else { vd[3] });        
    dw
}

fn predict_trajectory(x_init: nalgebra::Vector5<f64>, v: f64, y:f64, config: &Config)
-> Vec<nalgebra::Vector5<f64>>
{
    let mut x = x_init.clone();
    let n = (config.predict_time / config.dt) as usize;
    let mut trajectry: Vec<nalgebra::Vector5<f64>> = Vec::with_capacity(n);
    trajectry.push(x_init);
    let mut time = 0.;
    while time <= config.predict_time {
        x = motion(x, nalgebra::Vector2::new(v, y), config.dt);
        trajectry.push(x);
        time += config.dt;
    }
    trajectry
}

fn calc_control_and_trajectory(
    x: nalgebra::Vector5<f64>, dw: nalgebra::Vector4<f64>,
    config: &Config, goal: (f64, f64), ob: &Vec<(f64, f64)>) 
-> ((f64, f64), Vec<nalgebra::Vector5<f64>>)
{
    let x_init = x;
    let mut min_cost = std::f64::MAX;
    let mut best_u = (0., 0.);
    let n = (config.predict_time / config.dt) as usize;
    let mut best_trajectory: Vec<nalgebra::Vector5<f64>> = Vec::with_capacity(n);
    best_trajectory.push(x);
    let mut v = dw[0];
    while !(v >= dw[1]) {
        let mut y = dw[2];
        while !(y >= dw[3]) {
            let trajectory = predict_trajectory(x_init, v, y, & config);
            let to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(&trajectory, goal);
            let speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[trajectory.len()-1][3]);
            let ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(&trajectory, & ob, & config);
            let final_cost = to_goal_cost + speed_cost + ob_cost;
            if min_cost >= final_cost {
                min_cost = final_cost;
                best_u = (v, y);
                best_trajectory = trajectory;
            }
            y += config.yaw_rate_resolution;
        }
        v += config.v_resolution;
    } 
    (best_u, best_trajectory)
}

fn calc_obstacle_cost(
    trajectry: & Vec<nalgebra::Vector5<f64>>, ob: &Vec<(f64, f64)>, config: &Config)
-> f64
{
    let mut minr = std::f64::MAX;
    for i in 0..trajectry.len()-1 {
        for j in 0..ob.len()-1 {
            let r = ((trajectry[i][0] - ob[j].0).powi(2) + (trajectry[i][1] - ob[j].1).powi(2) ).sqrt();
            if r <= config.robot_radius {
                return std::f64::MAX;
            }
            if r < minr {
                minr = r;
            }
        }
    }
    1. / minr
}

fn calc_to_goal_cost(
    trajectry: &Vec<nalgebra::Vector5<f64>>, goal: (f64, f64))
-> f64
{
    let dx = goal.0 - trajectry[trajectry.len()-1][0];
    let dy = goal.1 - trajectry[trajectry.len()-1][1];
    let error_angle = dy.atan2(dx);
    let cost_angle = error_angle - trajectry[trajectry.len()-1][2];
    let cost = (((cost_angle).sin()).atan2((cost_angle).cos())).abs();
    cost
}

fn main() {

    //  initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    let mut x = nalgebra::Vector5::new(0., 0., std::f64::consts::PI / 8., 0., 0.);
    let goal = (10., 10.);

    let ob = vec![
        (0.0, 2.0),
        (4.0, 2.0),
        (5.0, 4.0),
        (5.0, 5.0),
        (5.0, 6.0),
        (5.0, 9.0),
        (8.0, 9.0),
        (7.0, 9.0),
    ];

    let config = Config {
        max_speed: 1.,
        min_speed: -0.5,
        max_yaw_rate: 40. * std::f64::consts::PI / 180.,
        max_accel: 0.2,
        max_delta_yaw_rate: 40. * std::f64::consts::PI / 180.,
        v_resolution: 0.01,
        yaw_rate_resolution: 0.1 * std::f64::consts::PI / 180.,
        dt: 0.1,
        predict_time: 3.,
        to_goal_cost_gain: 0.15,
        speed_cost_gain: 1.,
        obstacle_cost_gain: 1.,
        robot_radius: 1.
    };

    let n = (config.predict_time / config.dt) as usize;
    let mut trajectory: Vec<nalgebra::Vector5<f64>> = Vec::with_capacity(n);
    trajectory.push(x);

    let mut hpath  = vec![(x[0], x[1])];
    let mut hprepath  = vec![(x[0], x[1])];
    let mut predicted_trajectory: Vec<nalgebra::Vector5<f64>>;
    loop {
        let pair = dwa_control(x, & config, goal, &ob);
        let u = pair.0;
        predicted_trajectory = pair.1;
        x = motion(x, nalgebra::Vector2::new(u.0, u.1), config.dt);
        trajectory.push(x);
        hpath.push((x[0], x[1]));
        hprepath.clear();
        for t in predicted_trajectory {
            hprepath.push((t[0], t[1]));
        }
        let dist_to_goal = ((x[0] - goal.0).powi(2) + (x[1] - goal.1).powi(2)).sqrt();
        if dist_to_goal <= config.robot_radius {
            println!("Goal!!");
            break
        }

    }

    let s0: Plot = Plot::new(ob).point_style(
        PointStyle::new()
            .colour("#000000")
            .size(5.),
    );

    let s1: Plot = Plot::new(hpath).line_style(
        LineStyle::new() 
            .colour("#35C788")
            .width(2.),
    ); 

    let s2: Plot = Plot::new(hprepath).line_style(
        LineStyle::new() 
            .colour("#FFFF00")
            .width(2.),
    ); 

    let v = ContinuousView::new()
        .add(s0)
        .add(s1)
        .add(s2)
        .x_range(-0., 10.)
        .y_range(0., 10.)
        .x_label("x [m]")
        .y_label("y [m]");

    Page::single(&v).save("./img/dwa.svg").unwrap();
}