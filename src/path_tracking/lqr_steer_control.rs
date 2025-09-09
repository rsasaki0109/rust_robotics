//
// Path tracking simulation with LQR steering control and PID speed control.
//
// author Atsushi Sakai (@Atsushi_twi)
//        Ryohei Sasaki (@rsasaki0109)
//
extern crate nalgebra;

use gnuplot::{Figure, Caption, Color, AxesCommon};
use std::f64::consts::PI;
use nalgebra::{Matrix1, Matrix4, Vector4};


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
    while angle > PI {
        angle -= 2.0 * PI
    } 

    while angle < -PI {
        angle += 2.0 * PI
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
    let max_steer = 45.0 * PI / 180.0;

    let t_max = 500.0;
    let goal_dis = 0.3;
    let _stop_speed = 0.05;

    let init_x = (0.0, 0.0, 0.0 / 180.0 * PI, 0.0); // [x, y, yaw, v] 
    let mut state: State = State::new(init_x, l, max_steer);
    let mut states = vec![(state.x, state.y)];

    let mut time = 0.0;
    let mut e = 0.0;
    let mut e_th = 0.0;

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
            println!("Goal reached!");
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
        let switch = PI / 4.0 <= dyaw && dyaw < PI / 2.0;

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


pub struct LQRSteerController {
    pub reference_path: Vec<(f64, f64)>,
    pub reference_yaw: Vec<f64>,
    pub reference_curvature: Vec<f64>,
    pub speed_profile: Vec<f64>,
    pub trajectory: Vec<(f64, f64)>,
    pub goal: (f64, f64),
}

impl LQRSteerController {
    pub fn new() -> Self {
        LQRSteerController {
            reference_path: Vec::new(),
            reference_yaw: Vec::new(),
            reference_curvature: Vec::new(),
            speed_profile: Vec::new(),
            trajectory: Vec::new(),
            goal: (0.0, 0.0),
        }
    }

    pub fn planning(&mut self, waypoints: Vec<(f64, f64)>, target_speed: f64, ds: f64) -> bool {
        if waypoints.len() < 2 {
            println!("Need at least 2 waypoints");
            return false;
        }

        let ax: Vec<f64> = waypoints.iter().map(|p| p.0).collect();
        let ay: Vec<f64> = waypoints.iter().map(|p| p.1).collect();
        
        let (cx, cy, cyaw, ck, _s) = calc_spline_course(&ax, &ay, ds);
        
        self.reference_path = cx.iter().zip(cy.iter()).map(|(&x, &y)| (x, y)).collect();
        self.reference_yaw = cyaw;
        self.reference_curvature = ck;
        self.speed_profile = calc_speed_profile(&self.reference_path, &self.reference_yaw, target_speed);
        self.goal = *waypoints.last().unwrap();
        
        // Generate trajectory using closed loop prediction
        self.trajectory = closed_loop_prediction(
            &self.reference_path,
            &self.reference_yaw,
            &self.reference_curvature,
            &self.speed_profile,
            &self.goal,
        );
        
        true
    }

    pub fn visualize(&self, filename: &str) {
        let mut fg = Figure::new();
        {
            let axes = fg.axes2d()
                .set_title("LQR Steer Control Path Tracking", &[])
                .set_x_label("x [m]", &[])
                .set_y_label("y [m]", &[])
                .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));

            // Plot reference path
            let ref_x: Vec<f64> = self.reference_path.iter().map(|p| p.0).collect();
            let ref_y: Vec<f64> = self.reference_path.iter().map(|p| p.1).collect();
            axes.lines(&ref_x, &ref_y, &[Caption("Reference Path"), Color("blue")]);

            // Plot trajectory
            let traj_x: Vec<f64> = self.trajectory.iter().map(|p| p.0).collect();
            let traj_y: Vec<f64> = self.trajectory.iter().map(|p| p.1).collect();
            axes.lines(&traj_x, &traj_y, &[Caption("LQR Trajectory"), Color("red")]);

            // Plot start and goal
            if !self.reference_path.is_empty() {
                let start = &self.reference_path[0];
                axes.points(&[start.0], &[start.1], &[Caption("Start"), Color("green")]);
            }
            axes.points(&[self.goal.0], &[self.goal.1], &[Caption("Goal"), Color("red")]);
        }

        let output_path = format!("img/path_tracking/{}", filename);
        fg.set_terminal("pngcairo", &output_path);
        fg.show().unwrap();
        println!("LQR path tracking visualization saved to: {}", output_path);
    }
}

fn main() {
    // Create output directory
    std::fs::create_dir_all("img/path_tracking").unwrap();

    let mut controller = LQRSteerController::new();
    
    // Define waypoints
    let waypoints = vec![
        (0.0, 0.0),
        (6.0, -3.0),
        (12.5, -5.0),
        (10.0, 6.5),
        (7.5, 3.0),
        (3.0, 5.0),
        (-1.0, -2.0),
    ];
    
    let target_speed = 10.0 / 3.6; // 10 km/h to m/s
    let ds = 0.1; // course tick [m]
    
    println!("Starting LQR steer control path tracking...");
    
    if controller.planning(waypoints, target_speed, ds) {
        controller.visualize("lqr_steer_control.png");
        println!("LQR Steer Control Path Tracking complete!");
    } else {
        println!("Planning failed!");
    }
}

// Cubic spline functions (simplified version)
fn calc_spline_course(x: &[f64], y: &[f64], ds: f64) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
    let sp = CubicSpline2D::new(x, y);
    let mut s = 0.0;
    let mut course_x = Vec::new();
    let mut course_y = Vec::new();
    let mut course_yaw = Vec::new();
    let mut course_k = Vec::new();
    let mut course_s = Vec::new();
    
    while s < sp.s.last().unwrap() - ds {
        let (ix, iy) = sp.calc_position(s);
        let iyaw = sp.calc_yaw(s);
        let ik = sp.calc_curvature(s);
        course_x.push(ix);
        course_y.push(iy);
        course_yaw.push(iyaw);
        course_k.push(ik);
        course_s.push(s);
        s += ds;
    }
    
    (course_x, course_y, course_yaw, course_k, course_s)
}

struct CubicSpline {
    a: Vec<f64>,
    b: Vec<f64>,
    c: Vec<f64>,
    d: Vec<f64>,
    x: Vec<f64>,
}

impl CubicSpline {
    fn new(x: &[f64], y: &[f64]) -> Self {
        let n = x.len();
        let mut h = vec![0.0; n - 1];
        for i in 0..n - 1 {
            h[i] = x[i + 1] - x[i];
        }
        
        let mut a = vec![0.0; n];
        let mut b = vec![0.0; n];
        let mut c = vec![0.0; n];
        let mut d = vec![0.0; n];
        
        for i in 0..n {
            a[i] = y[i];
        }
        
        let mut alpha = vec![0.0; n - 1];
        for i in 1..n - 1 {
            alpha[i] = 3.0 * (a[i + 1] - a[i]) / h[i] - 3.0 * (a[i] - a[i - 1]) / h[i - 1];
        }
        
        let mut l = vec![1.0; n];
        let mut mu = vec![0.0; n];
        let mut z = vec![0.0; n];
        
        for i in 1..n - 1 {
            l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }
        
        for j in (0..n - 1).rev() {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        }
        
        CubicSpline {
            a,
            b,
            c,
            d,
            x: x.to_vec(),
        }
    }
    
    fn calc(&self, t: f64) -> f64 {
        if t < self.x[0] {
            return self.a[0];
        } else if t > self.x[self.x.len() - 1] {
            return self.a[self.a.len() - 1];
        }
        
        let mut i = self.search_index(t);
        if i >= self.x.len() - 1 {
            i = self.x.len() - 2;
        }
        
        let dx = t - self.x[i];
        self.a[i] + self.b[i] * dx + self.c[i] * dx * dx + self.d[i] * dx * dx * dx
    }
    
    fn calc_d(&self, t: f64) -> f64 {
        if t < self.x[0] {
            return self.b[0];
        } else if t > self.x[self.x.len() - 1] {
            return self.b[self.b.len() - 1];
        }
        
        let mut i = self.search_index(t);
        if i >= self.x.len() - 1 {
            i = self.x.len() - 2;
        }
        
        let dx = t - self.x[i];
        self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx * dx
    }
    
    fn calc_dd(&self, t: f64) -> f64 {
        if t < self.x[0] {
            return 2.0 * self.c[0];
        } else if t > self.x[self.x.len() - 1] {
            return 2.0 * self.c[self.c.len() - 1];
        }
        
        let mut i = self.search_index(t);
        if i >= self.x.len() - 1 {
            i = self.x.len() - 2;
        }
        
        let dx = t - self.x[i];
        2.0 * self.c[i] + 6.0 * self.d[i] * dx
    }
    
    fn search_index(&self, x: f64) -> usize {
        for i in 0..self.x.len() - 1 {
            if self.x[i] <= x && x <= self.x[i + 1] {
                return i;
            }
        }
        self.x.len() - 2
    }
}

struct CubicSpline2D {
    s: Vec<f64>,
    sx: CubicSpline,
    sy: CubicSpline,
}

impl CubicSpline2D {
    fn new(x: &[f64], y: &[f64]) -> Self {
        let mut s = vec![0.0];
        for i in 1..x.len() {
            let dx = x[i] - x[i - 1];
            let dy = y[i] - y[i - 1];
            s.push(s[i - 1] + (dx * dx + dy * dy).sqrt());
        }
        
        let sx = CubicSpline::new(&s, x);
        let sy = CubicSpline::new(&s, y);
        
        CubicSpline2D { s, sx, sy }
    }
    
    fn calc_position(&self, s: f64) -> (f64, f64) {
        let x = self.sx.calc(s);
        let y = self.sy.calc(s);
        (x, y)
    }
    
    fn calc_curvature(&self, s: f64) -> f64 {
        let dx = self.sx.calc_d(s);
        let ddx = self.sx.calc_dd(s);
        let dy = self.sy.calc_d(s);
        let ddy = self.sy.calc_dd(s);
        (ddy * dx - ddx * dy) / ((dx * dx + dy * dy).powf(1.5))
    }
    
    fn calc_yaw(&self, s: f64) -> f64 {
        let dx = self.sx.calc_d(s);
        let dy = self.sy.calc_d(s);
        dy.atan2(dx)
    }
}
