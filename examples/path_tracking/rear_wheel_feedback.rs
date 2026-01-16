//! Rear Wheel Feedback Control Example
//!
//! Demonstrates path tracking using rear wheel feedback steering control.
//!
//! Run with: cargo run --example rear_wheel_feedback

use gnuplot::{Figure, Caption, Color, AxesCommon, PointSymbol};
use rust_robotics::path_tracking::rear_wheel_feedback::{
    RearWheelFeedbackController, RearWheelFeedbackConfig, VehicleState,
};
use rust_robotics::common::{Point2D, Path2D};
use std::f64::consts::PI;

// Cubic spline helper for path generation
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

        CubicSpline { a, b, c, d, x: x.to_vec() }
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
        let denom = (dx * dx + dy * dy).powf(1.5);
        if denom.abs() < 1e-10 {
            0.0
        } else {
            (ddy * dx - ddx * dy) / denom
        }
    }

    fn calc_yaw(&self, s: f64) -> f64 {
        let dx = self.sx.calc_d(s);
        let dy = self.sy.calc_d(s);
        dy.atan2(dx)
    }
}

fn calc_spline_course(x: &[f64], y: &[f64], ds: f64) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
    let sp = CubicSpline2D::new(x, y);
    let mut s = 0.0;
    let mut course_x = Vec::new();
    let mut course_y = Vec::new();
    let mut course_yaw = Vec::new();
    let mut course_k = Vec::new();

    let s_max = *sp.s.last().unwrap() - ds;
    while s < s_max {
        let (ix, iy) = sp.calc_position(s);
        let iyaw = sp.calc_yaw(s);
        let ik = sp.calc_curvature(s);
        course_x.push(ix);
        course_y.push(iy);
        course_yaw.push(iyaw);
        course_k.push(ik);
        s += ds;
    }

    (course_x, course_y, course_yaw, course_k)
}

fn main() {
    // Create output directory
    std::fs::create_dir_all("img/path_tracking").unwrap();

    println!("Starting Rear Wheel Feedback Control simulation...");

    // Define waypoints for a curved path
    let waypoints = vec![
        (0.0, 0.0),
        (10.0, -6.0),
        (20.5, 5.0),
        (35.0, 6.5),
        (70.5, 0.0),
    ];

    // Create controller
    let config = RearWheelFeedbackConfig {
        kth: 1.0,
        ke: 0.5,
        wheelbase: 2.9,
        kp: 1.0,
        goal_threshold: 0.5,
        max_steer: 45.0_f64.to_radians(),
    };
    let mut controller = RearWheelFeedbackController::new(config.clone());

    // Run simulation
    let target_speed = 10.0; // m/s
    let ds = 0.1; // path resolution

    // Generate spline path
    let ax: Vec<f64> = waypoints.iter().map(|p| p.0).collect();
    let ay: Vec<f64> = waypoints.iter().map(|p| p.1).collect();
    let (cx, cy, cyaw, ck) = calc_spline_course(&ax, &ay, ds);

    // Set path
    let path = Path2D::from_points(
        cx.iter().zip(cy.iter()).map(|(&x, &y)| Point2D::new(x, y)).collect()
    );
    controller.set_path_with_info(path, cyaw.clone(), ck);

    // Initialize state (start with offset from path)
    let init_yaw = cyaw.first().copied().unwrap_or(0.0);
    let mut state = VehicleState::new(
        waypoints[0].0,
        waypoints[0].1 - 1.0, // Start slightly off the path
        init_yaw + 0.2,       // Start with heading error
        0.0,
        config.wheelbase,
    );

    // Simulation
    let mut trajectory_x = vec![state.x];
    let mut trajectory_y = vec![state.y];
    let dt = 0.1;
    let t_max = 100.0;
    let mut time = 0.0;

    while time < t_max {
        let ai = controller.compute_acceleration(target_speed, state.v);
        let di = controller.compute_steering(&state);
        state.update(ai, di, dt);
        time += dt;

        trajectory_x.push(state.x);
        trajectory_y.push(state.y);

        if controller.is_goal_reached_vehicle(&state) {
            println!("Goal reached at time: {:.2} s", time);
            break;
        }
    }

    println!("Simulation finished. Trajectory points: {}", trajectory_x.len());

    // Plot results
    let mut fg = Figure::new();
    {
        let axes = fg.axes2d()
            .set_title("Rear Wheel Feedback Control", &[])
            .set_x_label("x [m]", &[])
            .set_y_label("y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));

        // Plot reference path
        axes.lines(&cx, &cy, &[Caption("Reference Path"), Color("blue")]);

        // Plot vehicle trajectory
        axes.lines(&trajectory_x, &trajectory_y, &[Caption("Vehicle Trajectory"), Color("red")]);

        // Plot waypoints
        let wp_x: Vec<f64> = waypoints.iter().map(|p| p.0).collect();
        let wp_y: Vec<f64> = waypoints.iter().map(|p| p.1).collect();
        axes.points(&wp_x, &wp_y, &[Caption("Waypoints"), Color("green"), PointSymbol('O')]);

        // Plot start and goal
        axes.points(&[trajectory_x[0]], &[trajectory_y[0]], &[Caption("Start"), Color("cyan"), PointSymbol('o')]);
        axes.points(&[*trajectory_x.last().unwrap()], &[*trajectory_y.last().unwrap()],
                   &[Caption("End"), Color("magenta"), PointSymbol('x')]);
    }

    // Save as SVG
    let svg_path = "img/path_tracking/rear_wheel_feedback.svg";
    fg.set_terminal("svg", svg_path);
    match fg.show() {
        Ok(_) => println!("SVG saved to: {}", svg_path),
        Err(e) => eprintln!("Failed to save SVG: {:?}", e),
    }

    // Also save as PNG
    let png_path = "img/path_tracking/rear_wheel_feedback.png";
    fg.set_terminal("pngcairo", png_path);
    match fg.show() {
        Ok(_) => println!("PNG saved to: {}", png_path),
        Err(e) => eprintln!("Failed to save PNG: {:?}", e),
    }

    println!("Rear Wheel Feedback Control visualization complete!");
}
