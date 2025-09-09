use gnuplot::{Figure, Caption, Color, AxesCommon};
use std::f64::consts::PI;

// Parameters
const MAX_T: f64 = 100.0; // maximum time to the goal [s]
const MIN_T: f64 = 5.0;   // minimum time to the goal [s]
const SHOW_ANIMATION: bool = true;

#[derive(Debug, Clone)]
pub struct QuinticPolynomial {
    pub a0: f64,
    pub a1: f64,
    pub a2: f64,
    pub a3: f64,
    pub a4: f64,
    pub a5: f64,
}

impl QuinticPolynomial {
    pub fn new(xs: f64, vxs: f64, axs: f64, xe: f64, vxe: f64, axe: f64, time: f64) -> Self {
        // Calculate coefficient of quintic polynomial
        let a0 = xs;
        let a1 = vxs;
        let a2 = axs / 2.0;

        // Solve the linear system for a3, a4, a5
        // A * [a3, a4, a5]^T = b
        let t2 = time * time;
        let t3 = t2 * time;
        let t4 = t3 * time;
        let t5 = t4 * time;

        // Matrix A
        let a_matrix = [
            [t3, t4, t5],
            [3.0 * t2, 4.0 * t3, 5.0 * t4],
            [6.0 * time, 12.0 * t2, 20.0 * t3],
        ];

        // Vector b
        let b = [
            xe - a0 - a1 * time - a2 * t2,
            vxe - a1 - 2.0 * a2 * time,
            axe - 2.0 * a2,
        ];

        // Solve using Cramer's rule
        let det_a = Self::determinant_3x3(&a_matrix);
        
        let mut a3_matrix = a_matrix;
        a3_matrix[0][0] = b[0];
        a3_matrix[1][0] = b[1];
        a3_matrix[2][0] = b[2];
        let a3 = Self::determinant_3x3(&a3_matrix) / det_a;

        let mut a4_matrix = a_matrix;
        a4_matrix[0][1] = b[0];
        a4_matrix[1][1] = b[1];
        a4_matrix[2][1] = b[2];
        let a4 = Self::determinant_3x3(&a4_matrix) / det_a;

        let mut a5_matrix = a_matrix;
        a5_matrix[0][2] = b[0];
        a5_matrix[1][2] = b[1];
        a5_matrix[2][2] = b[2];
        let a5 = Self::determinant_3x3(&a5_matrix) / det_a;

        QuinticPolynomial { a0, a1, a2, a3, a4, a5 }
    }

    fn determinant_3x3(matrix: &[[f64; 3]; 3]) -> f64 {
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
            - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    }

    pub fn calc_point(&self, t: f64) -> f64 {
        self.a0 + self.a1 * t + self.a2 * t.powi(2) + 
        self.a3 * t.powi(3) + self.a4 * t.powi(4) + self.a5 * t.powi(5)
    }

    pub fn calc_first_derivative(&self, t: f64) -> f64 {
        self.a1 + 2.0 * self.a2 * t + 
        3.0 * self.a3 * t.powi(2) + 4.0 * self.a4 * t.powi(3) + 5.0 * self.a5 * t.powi(4)
    }

    pub fn calc_second_derivative(&self, t: f64) -> f64 {
        2.0 * self.a2 + 6.0 * self.a3 * t + 
        12.0 * self.a4 * t.powi(2) + 20.0 * self.a5 * t.powi(3)
    }

    pub fn calc_third_derivative(&self, t: f64) -> f64 {
        6.0 * self.a3 + 24.0 * self.a4 * t + 60.0 * self.a5 * t.powi(2)
    }
}

#[derive(Debug)]
pub struct QuinticPolynomialsPlanner {
    pub time: Vec<f64>,
    pub rx: Vec<f64>,
    pub ry: Vec<f64>,
    pub ryaw: Vec<f64>,
    pub rv: Vec<f64>,
    pub ra: Vec<f64>,
    pub rj: Vec<f64>,
}

impl QuinticPolynomialsPlanner {
    pub fn new() -> Self {
        QuinticPolynomialsPlanner {
            time: Vec::new(),
            rx: Vec::new(),
            ry: Vec::new(),
            ryaw: Vec::new(),
            rv: Vec::new(),
            ra: Vec::new(),
            rj: Vec::new(),
        }
    }

    pub fn planning(
        &mut self,
        sx: f64, sy: f64, syaw: f64, sv: f64, sa: f64,
        gx: f64, gy: f64, gyaw: f64, gv: f64, ga: f64,
        max_accel: f64, max_jerk: f64, dt: f64,
    ) -> bool {
        let vxs = sv * syaw.cos();
        let vys = sv * syaw.sin();
        let vxg = gv * gyaw.cos();
        let vyg = gv * gyaw.sin();

        let axs = sa * syaw.cos();
        let ays = sa * syaw.sin();
        let axg = ga * gyaw.cos();
        let ayg = ga * gyaw.sin();

        let mut t = MIN_T;
        while t < MAX_T {
            let xqp = QuinticPolynomial::new(sx, vxs, axs, gx, vxg, axg, t);
            let yqp = QuinticPolynomial::new(sy, vys, ays, gy, vyg, ayg, t);

            self.time.clear();
            self.rx.clear();
            self.ry.clear();
            self.ryaw.clear();
            self.rv.clear();
            self.ra.clear();
            self.rj.clear();

            let mut current_t = 0.0;
            while current_t <= t + dt {
                self.time.push(current_t);
                self.rx.push(xqp.calc_point(current_t));
                self.ry.push(yqp.calc_point(current_t));

                let vx = xqp.calc_first_derivative(current_t);
                let vy = yqp.calc_first_derivative(current_t);
                let v = (vx * vx + vy * vy).sqrt();
                let yaw = vy.atan2(vx);
                self.rv.push(v);
                self.ryaw.push(yaw);

                let ax = xqp.calc_second_derivative(current_t);
                let ay = yqp.calc_second_derivative(current_t);
                let mut a = (ax * ax + ay * ay).sqrt();
                if self.rv.len() >= 2 && self.rv[self.rv.len() - 1] - self.rv[self.rv.len() - 2] < 0.0 {
                    a *= -1.0;
                }
                self.ra.push(a);

                let jx = xqp.calc_third_derivative(current_t);
                let jy = yqp.calc_third_derivative(current_t);
                let mut j = (jx * jx + jy * jy).sqrt();
                if self.ra.len() >= 2 && self.ra[self.ra.len() - 1] - self.ra[self.ra.len() - 2] < 0.0 {
                    j *= -1.0;
                }
                self.rj.push(j);

                current_t += dt;
            }

            let max_a = self.ra.iter().map(|x| x.abs()).fold(0.0, f64::max);
            let max_j = self.rj.iter().map(|x| x.abs()).fold(0.0, f64::max);

            if max_a <= max_accel && max_j <= max_jerk {
                println!("Found path!! T = {:.2}s", t);
                return true;
            }

            t += MIN_T;
        }

        false
    }

    pub fn visualize_path(&self, sx: f64, sy: f64, syaw: f64, gx: f64, gy: f64, gyaw: f64) {
        if !SHOW_ANIMATION {
            return;
        }

        let mut fg = Figure::new();
        let axes = fg.axes2d();

        // Plot path
        axes.lines(&self.rx, &self.ry, &[Caption("Quintic Polynomial Path"), Color("red")]);

        // Plot start and goal positions
        axes.points(&[sx], &[sy], &[Caption("Start"), Color("green")]);
        axes.points(&[gx], &[gy], &[Caption("Goal"), Color("blue")]);

        // Plot arrows for start and goal orientations
        let arrow_length = 2.0;
        let start_arrow_x = [sx, sx + arrow_length * syaw.cos()];
        let start_arrow_y = [sy, sy + arrow_length * syaw.sin()];
        let goal_arrow_x = [gx, gx + arrow_length * gyaw.cos()];
        let goal_arrow_y = [gy, gy + arrow_length * gyaw.sin()];

        axes.lines(&start_arrow_x, &start_arrow_y, &[Color("green")]);
        axes.lines(&goal_arrow_x, &goal_arrow_y, &[Color("blue")]);

        axes.set_title("Quintic Polynomials Path Planning", &[])
            .set_x_label("X [m]", &[])
            .set_y_label("Y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));

        // Save to file
        let output_path = "img/path_planning/quintic_polynomials_result.png";
        fg.save_to_png(output_path, 800, 600).unwrap();
        println!("Plot saved to: {}", output_path);

        fg.show().unwrap();
    }

    pub fn visualize_profiles(&self) {
        if !SHOW_ANIMATION || self.time.is_empty() {
            return;
        }

        // Yaw profile
        let mut fg = Figure::new();
        let axes = fg.axes2d();
        let yaw_deg: Vec<f64> = self.ryaw.iter().map(|y| y * 180.0 / PI).collect();
        axes.lines(&self.time, &yaw_deg, &[Caption("Yaw"), Color("red")]);
        axes.set_title("Yaw Profile", &[])
            .set_x_label("Time [s]", &[])
            .set_y_label("Yaw [deg]", &[]);
        fg.save_to_png("img/path_planning/quintic_yaw_profile.png", 800, 600).unwrap();

        // Velocity profile
        let mut fg = Figure::new();
        let axes = fg.axes2d();
        axes.lines(&self.time, &self.rv, &[Caption("Velocity"), Color("red")]);
        axes.set_title("Velocity Profile", &[])
            .set_x_label("Time [s]", &[])
            .set_y_label("Speed [m/s]", &[]);
        fg.save_to_png("img/path_planning/quintic_velocity_profile.png", 800, 600).unwrap();

        // Acceleration profile
        let mut fg = Figure::new();
        let axes = fg.axes2d();
        axes.lines(&self.time, &self.ra, &[Caption("Acceleration"), Color("red")]);
        axes.set_title("Acceleration Profile", &[])
            .set_x_label("Time [s]", &[])
            .set_y_label("Accel [m/s²]", &[]);
        fg.save_to_png("img/path_planning/quintic_acceleration_profile.png", 800, 600).unwrap();

        // Jerk profile
        let mut fg = Figure::new();
        let axes = fg.axes2d();
        axes.lines(&self.time, &self.rj, &[Caption("Jerk"), Color("red")]);
        axes.set_title("Jerk Profile", &[])
            .set_x_label("Time [s]", &[])
            .set_y_label("Jerk [m/s³]", &[]);
        fg.save_to_png("img/path_planning/quintic_jerk_profile.png", 800, 600).unwrap();

        println!("Profile plots saved to img/path_planning/");
    }
}

fn main() {
    println!("Quintic Polynomials Planner start!!");

    let sx = 10.0;  // start x position [m]
    let sy = 10.0;  // start y position [m]
    let syaw = 10.0_f64.to_radians(); // start yaw angle [rad]
    let sv = 1.0;   // start speed [m/s]
    let sa = 0.1;   // start accel [m/s²]
    let gx = 30.0;  // goal x position [m]
    let gy = -10.0; // goal y position [m]
    let gyaw = 20.0_f64.to_radians(); // goal yaw angle [rad]
    let gv = 1.0;   // goal speed [m/s]
    let ga = 0.1;   // goal accel [m/s²]
    let max_accel = 1.0; // max accel [m/s²]
    let max_jerk = 0.5;  // max jerk [m/s³]
    let dt = 0.1;   // time tick [s]

    let mut planner = QuinticPolynomialsPlanner::new();

    if planner.planning(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt) {
        println!("Path found with {} points!", planner.rx.len());
        planner.visualize_path(sx, sy, syaw, gx, gy, gyaw);
        planner.visualize_profiles();
    } else {
        println!("Cannot find feasible path");
    }

    println!("Quintic Polynomials Planner finish!!");
}
