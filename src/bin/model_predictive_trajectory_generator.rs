// https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/ModelPredictiveTrajectoryGenerator
// 
// Model trajectory generator
//
// author: Atsushi Sakai(@Atsushi_twi)
//         Ryohei Sasaki(@rsasaki0109)
extern crate nalgebra;

use plotlib::page::Page;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::style::LineStyle;


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
          l: l
        }
      }
    
    fn update(&mut self, v: f64, delta: f64, dt: f64) {
        self.v = v;
        self.x += self.v * (self.yaw).cos() * dt;
        self.y += self.v * (self.yaw).sin() * dt;
        self.yaw += self.v / self.l * (delta).tan() * dt;
        self.yaw = pi2pi(self.yaw);
        
      }
}

fn generate_trajectory(p: (f64, f64, f64), k0: f64, ds: f64, v: f64, l: f64) -> (Vec<(f64, f64)>, Vec<f64>)
{
    let s = p.0;
    let km = p.1;
    let kf = p.2;

    let n = (s / ds) as usize;
    let time = s / v; 

    let tk = (0., time / 2., time);
    let kk = (k0, km, kf);
    let coef = quad_interp(tk, kk);

    let init_x = (0., 0., 0., 0.); // [x, y, yaw, v] 
    let mut state: State = State::new(init_x, l);
    let mut c = vec![(state.x, state.y)];
    let mut cyaw = vec![(state.yaw)];

    let dt = time / (n as f64);
    for i in 0..n-1 {
        let x = (i as f64) * dt;
        let ikp = coef.0 * x * x + coef.1 * x + coef.2;
        state.update(v, ikp, dt);
        c.push((state.x, state.y));
        cyaw.push(state.yaw);
    }
    (c, cyaw)
}

fn generate_last_state(p: (f64, f64, f64), k0: f64, ds: f64, v: f64, l: f64) -> ((f64, f64), f64)
{   
    let s = p.0;
    let km = p.1;
    let kf = p.2;

    let n = (s / ds) as usize;
    let time = s / v; 

    let tk = (0., time / 2., time);
    let kk = (k0, km, kf);
    let coef = quad_interp(tk, kk);

    let init_x = (0., 0., 0., 0.); // [x, y, yaw, v] 
    let mut state: State = State::new(init_x, l);

    let dt = time / (n as f64);
    for i in 0..n-1 {
        let t = (i as f64) * dt;
        let ikp = coef.0 * t * t + coef.1 * t + coef.2;
        state.update(v, ikp, dt);
    }
    ((state.x, state.y), state.yaw)
}

fn quad_interp(x: (f64, f64, f64), y: (f64, f64, f64)) -> (f64, f64, f64)
{
    let mat = nalgebra::Matrix3::new(
        x.0 * x.0, x.0, 1.,
        x.1 * x.1, x.1, 1.,
        x.2 * x.2, x.2, 1.);
    let vec = nalgebra::Vector3::new(
        y.0 , y.1, y.2);

    let coef = mat.try_inverse().unwrap() * vec;

    (coef[0], coef[1], coef[2])
}

fn calc_diff(target: &State, x:(f64, f64, f64))-> nalgebra::Vector3<f64>{
    nalgebra::Vector3::new(
        target.x - x.0,
        target.y - x.1,
        pi2pi(target.yaw - x.2))
}

fn calc_j(target: &State, p: nalgebra::Vector3<f64>, h: nalgebra::Vector3<f64>, k0: f64, ds: f64, v: f64, l: f64)
-> nalgebra::Matrix3<f64>
{
    let (mut xp, mut yawp) = generate_last_state((p[0] + h[0], p[1], p[2]), k0, ds, v, l);
    let mut dp = calc_diff(&target, (xp.0, xp.1, yawp));
    let (mut xn, mut yawn) = generate_last_state((p[0] - h[0], p[1], p[2]), k0, ds, v, l);
    let mut dn = calc_diff(&target, (xn.0, xn.1, yawn));
    let d1 = (dp - dn) / (2. * h[0]);

    let mut pair = generate_last_state((p[0], p[1] + h[1], p[2]), k0, ds, v, l);
    xp = pair.0;
    yawp = pair.1;
    dp = calc_diff(&target, (xp.0, xp.1, yawp));
    pair = generate_last_state((p[0], p[1] - h[1], p[2]), k0, ds, v, l);
    xn = pair.0;
    yawn = pair.1;
    dn = calc_diff(&target, (xn.0, xn.1, yawn));
    let d2 = (dp - dn) / (2. * h[1]);

    pair = generate_last_state((p[0], p[1], p[2] + h[2]), k0, ds, v, l);
    xp = pair.0;
    yawp = pair.1;
    dp = calc_diff(&target, (xp.0, xp.1, yawp));
    pair = generate_last_state((p[0], p[1], p[2] - h[2]), k0, ds, v, l);
    xn = pair.0;
    yawn = pair.1;
    dn = calc_diff(&target, (xn.0, xn.1, yawn));
    let d3 = (dp - dn) / (2. * h[2]);

    let j = nalgebra::Matrix3::new(
        d1[0], d2[0], d3[0],
        d1[1], d2[1], d3[1],
        d1[2], d2[2], d3[2]);
    j
}

fn selection_learning_param(p: nalgebra::Vector3<f64>, k0: f64, ds: f64, v: f64, l: f64, target: &State)
-> f64
{
    let mut mincost = std::f64::MAX;
    let mut mina = 1.0;
    let maxa = 2.0;
    let da = 0.5;
    let na = ((maxa - mina) / da) as usize;
    for i in 0..na-1 {
        let a = mina + (na as f64) * (i as f64);
        let tp = p + a * p;
        let pair = generate_last_state((tp[0], tp[1], tp[2]), k0, ds, v, l);
        let xc = pair.0;
        let yawc = pair.1;
        let dc = calc_diff(&target, (xc.0, xc.1, yawc));
        let cost = dc.norm();
        if cost <= mincost && a != 0. {
            mina = a;
            mincost = cost;
        }
    }
    mina
}

fn optimize_trajectory(
    target: State, mut p: nalgebra::Vector3<f64>, h: nalgebra::Vector3<f64>,
    k0: f64, ds: f64, v: f64, l: f64, max_iter: usize, cost_th: f64)
-> (Vec<(f64, f64)>, Vec<f64>, nalgebra::Vector3<f64>)
{
    let mut xc: Vec<(f64, f64)> = vec![(0., 0.)];
    let mut yawc: Vec<f64> = vec![0.];
    for _i in 0..max_iter-1 {
        let pair = generate_trajectory((p[0], p[1], p[2]), k0, ds, v, l);
        xc = pair.0;
        yawc = pair.1;
        let dc = calc_diff(&target, (xc[xc.len()-1].0, xc[xc.len()-1].1, yawc[xc.len()-1]));
        let cost = dc.norm(); 
        if cost <= cost_th {
            println!("path is ok");
            break
        }

        let j = calc_j(&target, p, h, k0, ds, v, l);
        let dp = - j.try_inverse().unwrap() * dc;
        let alpha = selection_learning_param(p, k0, ds, v, l, &target);

        p += alpha * dp;
        
    }
    (xc, yawc, p)
}

fn test_optimize_trajectory() {

    let max_iter = 100;
    let h = nalgebra::Vector3::new(
        0.5,
        0.02,
        0.02);
    let cost_th = 0.1;

    let k0 = 0.;
    let l = 1.0;  // wheel base
    let ds = 0.1; // course distanse
    let v = 10.0 / 3.6; // velocity [m/s]

    let init_x = (5., 2., 90./180. * std::f64::consts::PI, 0.); // [x, y, yaw, v] 
    let target = State::new(init_x, l);
    
    let init_p = nalgebra::Vector3::new(
        6.,
        0.,
        0.);
    
    let tmp = optimize_trajectory(target, init_p, h, k0, ds, v, l, max_iter, cost_th);
    let xc = tmp.0;

    let s1: Plot = Plot::new(xc).line_style(
        LineStyle::new() 
            .colour("#35C788"),
      );

    let v = ContinuousView::new()
      .add(s1)
      .x_range( 0., 6.)
      .y_range(-1., 3.)
      .x_label("x [m]")
      .y_label("y [m]");

      Page::single(&v).save("./img/model_predictive_trajectory_generator.svg").unwrap();
}





fn main() {
    test_optimize_trajectory();
}