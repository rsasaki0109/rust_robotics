// 
// Path tracking simulation with Stanley steering control and PID speed control.
//
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109)
// Ref:
//     - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
//    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

use RustRobotics::cubic_spline_planner;

use plotlib::page::Page;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::style::LineStyle;

#[derive(Debug, Copy, Clone)] 
struct State {
    x: f64,
    y: f64,
    yaw: f64,
    v: f64,
    l: f64,
}

impl State {
    fn new(x:(f64,f64,f64,f64), l: f64)-> State{
        State {
          x: x.0, y: x.1,
          yaw: x.2, v: x.3,
          l: l,
        }
      }
      fn update(&mut self, a: f64, delta: f64, dt: f64) {
        self.x += self.v * (self.yaw).cos() * dt;
        self.y += self.v * (self.yaw).sin() * dt;
        self.yaw += self.v / self.l * (delta).tan() * dt;
        self.v += a * dt;
      }
}

fn pid_control(target: f64, current: f64, kp: f64) -> f64
{
    kp * (target - current)
}

fn stanley_control(state: State, c: &Vec<(f64, f64)>, cyaw: &Vec<f64>, last_target_idx: usize, k: f64)
-> (f64, usize)
{
    let (mut current_target_idx, error_front_axle) = calc_target_index(&c, &state);
    if last_target_idx >= current_target_idx {
        current_target_idx = last_target_idx;
    }
    let theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw);
    let theta_d = (k * error_front_axle).atan2(state.v);
    let delta = theta_e + theta_d;
    (delta, current_target_idx)
}

fn normalize_angle(mut angle: f64) -> f64
{
    while angle > std::f64::consts::PI {
        angle -= 2. * std::f64::consts::PI
    } 

    while angle < -std::f64::consts::PI {
        angle += 2. * std::f64::consts::PI
    } 
    angle
}

fn calc_target_index(c: &Vec<(f64, f64)>, state: &State) -> (usize, f64){
    let fx = state.x + state.l * (state.yaw).cos();
    let fy = state.y + state.l * (state.yaw).sin();
    let mut ind_min = 0;
    let mut d_min = std::f64::MAX;
    let nc = c.len();
    for i in 0..nc-1 {
        let diff_x = (fx - c[i].0, fy -c[i].1);
        let d = ((diff_x.0).powi(2) + (diff_x.1).powi(2)).sqrt();

        if d < d_min {
            d_min = d;
            ind_min = i;
          }
    }
    let diff_x = (fx - c[ind_min].0, fy -c[ind_min].1);
    let error_front_axle = - (state.yaw + 0.5 * std::f64::consts::PI).cos() * diff_x.0 - 
        (state.yaw + 0.5 * std::f64::consts::PI).sin() * diff_x.1;
    (ind_min, error_front_axle)
}

fn main() {
    let k = 0.5; // control gain
    let kp = 1.0; // speed proportional gain
    let dt = 0.1; // [s] time difference
    let l = 2.9; // [m] Wheel base of vehicle
    let _max_steer = 30. * std::f64::consts::PI / 180.;

    // targrt_cource
    let a = vec![
        (0., 0.),
        (100., 0.),
        (100., -30.),
        (50.0, -20.),
        (60.0, 0.),
    ];

    let nx = a.len();
    let mut ax: Vec<f64> = Vec::with_capacity(nx);
    let mut ay: Vec<f64> = Vec::with_capacity(nx);

    for p in &a {
        ax.push(p.0);
        ay.push(p.1);
    }

    let (c, cyaw, _ck, _s) = cubic_spline_planner::calc_spline_course(ax, ay, 0.1);

    let target_speed = 30.0 / 3.6;
    let max_simulation_time = 100.0;


    let init_x = (0., 5., 20./180. * std::f64::consts::PI, 0.); // [x, y, yaw, v] 
    let mut state: State = State::new(init_x, l);
    let mut states = vec![(state.x, state.y)];
    let last_idx = c.len() - 1;
    let mut time = 0.;
    let pair = calc_target_index(&c, &state);
    let target_ind = pair.0;

    while max_simulation_time > time && last_idx > target_ind {
        let ai = pid_control(target_speed, state.v, kp);
        let tmp = stanley_control(state, &c, &cyaw, target_ind, k);
        let di = tmp.0;
        state.update(ai, di, dt);
        time += dt;
  
        states.push((state.x, state.y));
      }

    let s0: Plot = Plot::new(c).line_style(
        LineStyle::new()
            .colour("#000000"),
    );

    let s1: Plot = Plot::new(states).line_style(
        LineStyle::new() 
            .colour("#35C788"),
    );

    let v = ContinuousView::new()
        .add(s0)
        .add(s1)
        .x_range(0., 100.)
        .y_range(-40., 40.)
        .x_label("x [m]")
        .y_label("y [m]");

    Page::single(&v).save("./img/stanley_control.svg").unwrap();
}
