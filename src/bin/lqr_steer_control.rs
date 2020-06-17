//
// Path tracking simulation with LQR steering control and PID speed control.
//
// author Atsushi Sakai (@Atsushi_twi)
//        Ryohei Sasaki (@rsasaki0109)
//
extern crate nalgebra;

use RustRobotics::cubic_spline_planner;

use plotlib::page::Page;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::style::LineStyle;


struct State {
    x: f64,
    y: f64,
    yaw: f64,
    v: f64,
    l: f64,
    max_steer: f64,
}

impl State {
    fn new(x:(f64,f64,f64,f64), l: f64, max_steer: f64)-> State{
        State {
          x: x.0, y: x.1,
          yaw: x.2, v: x.3,
          l: l, max_steer: max_steer
        }
      }
      fn update(&mut self, a: f64, mut delta: f64, dt: f64) {
        if delta >= self.max_steer {
            delta = self.max_steer;
        }
        if delta <= -self.max_steer {
            delta = self.max_steer;
        }
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

fn pi2pi(mut angle: f64) -> f64
{
    while angle > std::f64::consts::PI {
        angle -= 2. * std::f64::consts::PI
    } 

    while angle < -std::f64::consts::PI {
        angle += 2. * std::f64::consts::PI
    } 
    angle
}

fn solve_dare(
    a: nalgebra::Matrix4<f64>,
    b: nalgebra::Vector4<f64>,
    q: nalgebra::Matrix4<f64>,
    r: nalgebra::Matrix1<f64>,
) -> nalgebra::Matrix4<f64>
{
    let mut x = q; 
    let max_iter = 150;
    let eps = 0.01;
    let mut iter = 0;
    while iter < max_iter {
        let xn = a.transpose() * x * a - a.transpose() * x * b *
            (r + b.transpose() * x * b).try_inverse().unwrap() * b.transpose() * x * a + q;

        if (xn -x).abs().max() < eps {
            break;
        }
        x = xn;
        iter += 1;
    }
    x
}

fn dlqr(
    a: nalgebra::Matrix4<f64>,
    b: nalgebra::Vector4<f64>,
    q: nalgebra::Matrix4<f64>,
    r: nalgebra::Matrix1<f64>,
) -> (nalgebra::Matrix1x4<f64>, nalgebra::Matrix4<f64>, nalgebra::Vector4<f64>)
{
    let x = solve_dare(a, b, q, r);
    let k = (b.transpose() * x * b + r).try_inverse().unwrap() * (b.transpose() * x * a);
    let mat = a - b * k;
    let eigens = mat.try_symmetric_eigen(1e-9, 100).unwrap();
    let eigenvalues = eigens.eigenvalues;

    (k, x, eigenvalues)
}

fn lqr_steering_control(
    state: &State, c: &Vec<(f64, f64)>, cyaw: &Vec<f64>, ck: &Vec<f64>,
    pe: f64, pth_e: f64, dt: f64,
    q: nalgebra::Matrix4<f64>, r: nalgebra::Matrix1<f64>
)
-> (f64, usize, f64, f64)
{
    let (ind, e) = calc_target_index(&c, &cyaw, &state);
    let k = ck[ind];
    let v = state.v;
    let th_e = pi2pi(state.yaw - cyaw[ind]);

    let mut a = nalgebra::Matrix4::zeros();
    a[(0, 0)] = 1.0;
    a[(0, 1)] = dt;
    a[(1, 2)] = v;
    a[(2, 2)] = 1.0;
    a[(2, 3)] = dt;

    let mut b = nalgebra::Vector4::zeros();
    b[(3, 0)] = v / state.l;

    let (gain, _, _) = dlqr(a, b, q, r);

    let x = nalgebra::Vector4::new(
        e,
        (e - pe) / dt,
        th_e,
        (th_e - pth_e) / dt);
    
    let ff = (state.l * k).atan2(1.);
    let fb = pi2pi((- gain * x)[0]);

    let delta = ff + fb;

    (delta, ind, e, th_e)
}

fn calc_target_index(c: &Vec<(f64, f64)>, cyaw: &Vec<f64>, state: &State) -> (usize, f64){
    let mut ind_min = 0;
    let mut d_min = std::f64::MAX;
    let nc = c.len();
    for i in 0..nc-1 {
        let diff_x = (state.x - c[i].0, state.y -c[i].1);
        let d = ((diff_x.0).powi(2) + (diff_x.1).powi(2)).sqrt();

        if d < d_min {
            d_min = d;
            ind_min = i;
          }
    }
    let diff_x = (c[ind_min].0 - state.x, c[ind_min].1 - state.y);
    let arcang = cyaw[ind_min] - (diff_x.1).atan2(diff_x.0);
    let angle = pi2pi(arcang);
    if angle < 0. {
        d_min *= -1.;
    }
    (ind_min, d_min)
}

fn closed_loop_prediction(
    c: &Vec<(f64, f64)>, cyaw: &Vec<f64>, ck: &Vec<f64>,
    speed_profile: &Vec<f64>,
    goal: &(f64, f64),
) -> Vec<(f64, f64)>
{
    let kp = 1.; // speed proportional gain

    // LQR parameter
    let q = nalgebra::Matrix4::<f64>::identity();
    let r = nalgebra::Matrix1::<f64>::identity();

    // parameters
    let dt = 0.1; // time tick[s]
    let l = 0.5; // Wheel base of the vehicle [m]
    let max_steer = 45. * std::f64::consts::PI / 180.;

    let t_max = 500.0;
    let goal_dis = 0.3;
    let _stop_speed = 0.05;

    let init_x = (0., 0., 0./180. * std::f64::consts::PI, 0.); // [x, y, yaw, v] 
    let mut state: State = State::new(init_x, l, max_steer);
    let mut states = vec![(state.x, state.y)];

    let mut time = 0.;
    let mut e = 0.;
    let mut e_th = 0.;

    while time <= t_max {
        let tmp = lqr_steering_control(
            &state, c, cyaw, ck, e, e_th, dt, q, r);
        let dl = tmp.0;
        let target_ind = tmp.1;
        e = tmp.2;
        e_th = tmp.3;

        let ai = pid_control(speed_profile[target_ind], state.v, kp);
        state.update(ai, dl, dt);
        states.push((state.x, state.y));

        time += dt;

        let dx = state.x - goal.0;
        let dy = state.y - goal.1;
        let d = (dx.powi(2) + dy.powi(2)).sqrt();
        if d <= goal_dis {
            println!("Goal");
            break;
        }
    }
    states
}

fn calc_speed_profile(
    c: &Vec<(f64, f64)>, cyaw: &Vec<f64>, 
    target_speed: f64,
) -> Vec<f64>
{
    let n = c.len();
    let mut speed_profile : Vec<f64> = Vec::with_capacity(n);
    let mut direction = 1.;
    for i in 0..n-1 {
        let dyaw = (cyaw[i + 1] - cyaw[i]).abs();
        let switch = std::f64::consts::PI / 4. <= dyaw && dyaw < std::f64::consts::PI / 2.;

        if switch {
            direction *= -1.;
        }

        if direction != 1.0 {
            speed_profile.push(- target_speed);
        } else {
            speed_profile.push(target_speed);
        }

        if switch {
            speed_profile[i] = 0.;
        }
    }
    speed_profile.push(0.);
    speed_profile
}


fn main() {
    
    let a = vec![
        (0., 0.),
        (6., -3.),
        (12.5, -5.),
        (10.0, 6.5),
        (7.5, 3.),
        (3.0, 5.),
        (-1.0, -2.),
    ];

    let nx = a.len();
    let goal = a[nx-1];
    let mut ax: Vec<f64> = Vec::with_capacity(nx);
    let mut ay: Vec<f64> = Vec::with_capacity(nx);
    for p in &a {
        ax.push(p.0);
        ay.push(p.1);
    }
    let (c, cyaw, ck, _s) = cubic_spline_planner::calc_spline_course(ax, ay, 0.1);

    let target_speed = 10.0 / 3.6; // simulation parameter km/h -> m/s

    let sp = calc_speed_profile(&c, &cyaw, target_speed);
    let t = closed_loop_prediction(&c, &cyaw, &ck, &sp, &goal);

    let s0: Plot = Plot::new(c).line_style(
        LineStyle::new()
            .colour("#000000"),
    );

    let s1: Plot = Plot::new(t).line_style(
        LineStyle::new() 
            .colour("#35C788"),
    );


    let v = ContinuousView::new()
        .add(s0)
        .add(s1)
        .x_range(-2.5, 15.)
        .y_range(-7.5, 10.)
        .x_label("x [m]")
        .y_label("y [m]");

    Page::single(&v).save("./img/lqr_steer_control.svg").unwrap();

    
    
    
}
