// 
// Path tracking simulation with Stanley steering control and PID speed control.
//
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109)
// Ref:
//     - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
//    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

use gnuplot::{Figure, Caption, Color, AxesCommon};
use std::f64::consts::PI;

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
    while angle > PI {
        angle -= 2.0 * PI
    } 

    while angle < -PI {
        angle += 2.0 * PI
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
    let error_front_axle = - (state.yaw + 0.5 * PI).cos() * diff_x.0 - 
        (state.yaw + 0.5 * PI).sin() * diff_x.1;
    (ind_min, error_front_axle)
}

pub struct StanleyController {
    pub reference_path: Vec<(f64, f64)>,
    pub reference_yaw: Vec<f64>,
    pub trajectory: Vec<(f64, f64)>,
    pub k: f64,
    pub l: f64,
}

impl StanleyController {
    pub fn new(k: f64, l: f64) -> Self {
        StanleyController {
            reference_path: Vec::new(),
            reference_yaw: Vec::new(),
            trajectory: Vec::new(),
            k,
            l,
        }
    }

    pub fn planning(&mut self, waypoints: Vec<(f64, f64)>, target_speed: f64, ds: f64) -> bool {
        if waypoints.len() < 2 {
            println!("Need at least 2 waypoints");
            return false;
        }

        let ax: Vec<f64> = waypoints.iter().map(|p| p.0).collect();
        let ay: Vec<f64> = waypoints.iter().map(|p| p.1).collect();
        
        let (cx, cy, cyaw, _ck, _s) = calc_spline_course(&ax, &ay, ds);
        
        self.reference_path = cx.iter().zip(cy.iter()).map(|(&x, &y)| (x, y)).collect();
        self.reference_yaw = cyaw;
        
        // Simulate Stanley tracking
        let kp = 1.0; // speed proportional gain
        let dt = 0.1; // [s] time difference
        let max_simulation_time = 100.0;

        let init_x = (waypoints[0].0, waypoints[0].1 + 5.0, 20.0 / 180.0 * PI, 0.0);
        let mut state = State::new(init_x, self.l);
        self.trajectory = vec![(state.x, state.y)];
        let last_idx = self.reference_path.len() - 1;
        let mut time = 0.0;
        let pair = calc_target_index(&self.reference_path, &state);
        let mut target_ind = pair.0;

        while max_simulation_time > time && last_idx > target_ind {
            let ai = pid_control(target_speed, state.v, kp);
            let tmp = stanley_control(state, &self.reference_path, &self.reference_yaw, target_ind, self.k);
            let di = tmp.0;
            target_ind = tmp.1;
            state.update(ai, di, dt);
            time += dt;
      
            self.trajectory.push((state.x, state.y));
            
            // Check if reached goal
            let goal = waypoints.last().unwrap();
            let dx = state.x - goal.0;
            let dy = state.y - goal.1;
            let distance_to_goal = (dx * dx + dy * dy).sqrt();
            if distance_to_goal < 3.0 {
                println!("Goal reached!");
                break;
            }
        }
        
        true
    }

    pub fn visualize(&self, filename: &str) {
        let mut fg = Figure::new();
        {
            let axes = fg.axes2d()
                .set_title("Stanley Controller Path Tracking", &[])
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
            axes.lines(&traj_x, &traj_y, &[Caption("Stanley Trajectory"), Color("red")]);

            // Plot start and goal
            if !self.reference_path.is_empty() {
                let start = &self.reference_path[0];
                axes.points(&[start.0], &[start.1], &[Caption("Start"), Color("green")]);
                let goal = self.reference_path.last().unwrap();
                axes.points(&[goal.0], &[goal.1], &[Caption("Goal"), Color("red")]);
            }
        }

        let output_path = format!("img/path_tracking/{}", filename);
        fg.set_terminal("pngcairo", &output_path);
        fg.show().unwrap();
        println!("Stanley controller path tracking visualization saved to: {}", output_path);
    }
}

fn main() {
    // Create output directory
    std::fs::create_dir_all("img/path_tracking").unwrap();

    let mut controller = StanleyController::new(0.5, 2.9);
    
    // Define waypoints
    let waypoints = vec![
        (0.0, 0.0),
        (100.0, 0.0),
        (100.0, -30.0),
        (50.0, -20.0),
        (60.0, 0.0),
    ];
    
    let target_speed = 30.0 / 3.6; // 30 km/h to m/s
    let ds = 0.1; // course tick [m]
    
    println!("Starting Stanley controller path tracking...");
    
    if controller.planning(waypoints, target_speed, ds) {
        controller.visualize("stanley_controller.png");
        println!("Stanley Controller Path Tracking complete!");
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
