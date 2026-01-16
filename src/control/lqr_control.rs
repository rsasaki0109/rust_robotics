//
// Inverted Pendulum LQR control
// author: Trung Kien - letrungkien.k53.hut@gmail.com
// Ported to Rust by: rust_robotics team
//
// Visualization style based on PythonRobotics by AtsushiSakai
//

use gnuplot::{Figure, AxesCommon, PlotOption, Coordinate};
use nalgebra::{Matrix4, Matrix1, Vector4, Matrix1x4};

// Model parameters
const L_BAR: f64 = 2.0;  // length of bar
const M: f64 = 1.0;      // [kg] cart mass
const MASS: f64 = 0.3;   // [kg] pendulum mass
const G: f64 = 9.8;      // [m/s^2] gravity

const DELTA_T: f64 = 0.1; // time tick [s]
const SIM_TIME: f64 = 5.0; // simulation time [s]

// Cart dimensions (matching PythonRobotics)
const CART_WIDTH: f64 = 1.0;
const CART_HEIGHT: f64 = 0.5;
const WHEEL_RADIUS: f64 = 0.1;

pub struct InvertedPendulumLQR {
    pub q: Matrix4<f64>,     // state cost matrix
    pub r: Matrix1<f64>,     // input cost matrix
    pub trajectory: Vec<(f64, Vector4<f64>)>, // time, state history
}

impl InvertedPendulumLQR {
    pub fn new() -> Self {
        let mut q = Matrix4::<f64>::zeros();
        q[(1, 1)] = 1.0;  // velocity cost
        q[(2, 2)] = 1.0;  // angle cost

        let r = Matrix1::<f64>::from_element(0.01); // input cost

        InvertedPendulumLQR {
            q,
            r,
            trajectory: Vec::new(),
        }
    }

    pub fn simulate(&mut self, x0: Vector4<f64>) -> bool {
        let mut x = x0;
        let mut time = 0.0;

        self.trajectory.clear();
        self.trajectory.push((time, x));

        while time < SIM_TIME {
            time += DELTA_T;

            // Calculate control input
            let u = self.lqr_control(&x);

            // Simulate inverted pendulum cart
            x = self.simulation_step(&x, u);

            self.trajectory.push((time, x));
        }

        println!("Simulation finished");
        println!("Final state: x={:.2} [m], theta={:.2} [deg]",
                x[0], x[2].to_degrees());

        true
    }

    fn simulation_step(&self, x: &Vector4<f64>, u: Matrix1<f64>) -> Vector4<f64> {
        let (a, b) = self.get_model_matrix();
        a * x + b * u[0]
    }

    fn lqr_control(&self, x: &Vector4<f64>) -> Matrix1<f64> {
        let (a, b) = self.get_model_matrix();
        let (k, _, _) = self.dlqr(&a, &b, &self.q, &self.r);
        Matrix1::from_element(-(k * x)[0])
    }

    fn get_model_matrix(&self) -> (Matrix4<f64>, Vector4<f64>) {
        let mut a = Matrix4::<f64>::new(
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, MASS * G / M, 0.0,
            0.0, 0.0, 0.0, 1.0,
            0.0, 0.0, G * (M + MASS) / (L_BAR * M), 0.0
        );
        a = Matrix4::<f64>::identity() + DELTA_T * a;

        let b = Vector4::new(
            0.0,
            1.0 / M,
            0.0,
            1.0 / (L_BAR * M)
        ) * DELTA_T;

        (a, b)
    }

    fn solve_dare(&self, a: &Matrix4<f64>, b: &Vector4<f64>, q: &Matrix4<f64>, r: &Matrix1<f64>) -> Matrix4<f64> {
        let mut p = *q;
        let max_iter = 150;
        let eps = 0.01;

        for _ in 0..max_iter {
            let bt_p = b.transpose() * p;
            let denominator = r + bt_p * b;
            if denominator[0].abs() < 1e-10 {
                break;
            }

            let pn = a.transpose() * p * a -
                     a.transpose() * p * b * (1.0 / denominator[0]) * bt_p * a + q;

            if (pn - p).abs().max() < eps {
                break;
            }
            p = pn;
        }
        p
    }

    fn dlqr(&self, a: &Matrix4<f64>, b: &Vector4<f64>, q: &Matrix4<f64>, r: &Matrix1<f64>) -> (Matrix1x4<f64>, Matrix4<f64>, Vector4<f64>) {
        let p = self.solve_dare(a, b, q, r);

        let bt_p = b.transpose() * p;
        let denominator = r + bt_p * b;
        let k = (1.0 / denominator[0]) * bt_p * a;

        let closed_loop = a - b * k;
        let eigenvalues = closed_loop.eigenvalues().unwrap();

        (k, p, eigenvalues)
    }

    /// Generate cart polygon points for drawing
    fn get_cart_polygon(&self, x: f64) -> (Vec<f64>, Vec<f64>) {
        let y_offset = WHEEL_RADIUS * 2.0;
        let half_w = CART_WIDTH / 2.0;

        // Rectangle corners (closed polygon)
        let xs = vec![
            x - half_w, x + half_w, x + half_w, x - half_w, x - half_w
        ];
        let ys = vec![
            y_offset, y_offset, y_offset + CART_HEIGHT, y_offset + CART_HEIGHT, y_offset
        ];
        (xs, ys)
    }

    /// Generate wheel circle points
    fn get_wheel_points(&self, cx: f64, cy: f64) -> (Vec<f64>, Vec<f64>) {
        let n = 20;
        let mut xs = Vec::with_capacity(n + 1);
        let mut ys = Vec::with_capacity(n + 1);
        for i in 0..=n {
            let angle = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
            xs.push(cx + WHEEL_RADIUS * angle.cos());
            ys.push(cy + WHEEL_RADIUS * angle.sin());
        }
        (xs, ys)
    }

    /// Generate pendulum bar endpoints
    fn get_pendulum_points(&self, x: f64, theta: f64) -> (Vec<f64>, Vec<f64>) {
        let y_offset = WHEEL_RADIUS * 2.0 + CART_HEIGHT;
        let bar_x = x + L_BAR * theta.sin();
        let bar_y = y_offset + L_BAR * theta.cos();
        (vec![x, bar_x], vec![y_offset, bar_y])
    }

    /// Visualize cart-pendulum animation (PythonRobotics style)
    pub fn visualize(&self, filename: &str) {
        // Create summary image with multiple frames
        let num_frames = 6;
        let total_steps = self.trajectory.len();
        let step_interval = total_steps / num_frames;

        let mut fg = Figure::new();
        {
            let axes = fg.axes2d()
                .set_title("Inverted Pendulum LQR Control", &[])
                .set_x_label("x [m]", &[])
                .set_y_label("y [m]", &[])
                .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0))
                .set_x_range(gnuplot::AutoOption::Fix(-3.0), gnuplot::AutoOption::Fix(3.0))
                .set_y_range(gnuplot::AutoOption::Fix(-1.0), gnuplot::AutoOption::Fix(4.0));

            // Draw ground line
            axes.lines(&[-4.0, 4.0], &[0.0, 0.0], &[PlotOption::Color("gray"), PlotOption::LineWidth(2.0)]);

            // Draw multiple frames with transparency effect (lighter colors for older frames)
            let colors = ["#CCCCFF", "#AAAAFF", "#8888FF", "#6666FF", "#4444FF", "#0000FF"];
            let pendulum_colors = ["#CCCCCC", "#AAAAAA", "#888888", "#666666", "#444444", "#000000"];

            for (frame_idx, color_idx) in (0..num_frames).zip(0..num_frames) {
                let step = if frame_idx == num_frames - 1 {
                    total_steps - 1
                } else {
                    frame_idx * step_interval
                };

                let (time, state) = &self.trajectory[step];
                let x_pos = state[0];
                let theta = state[2];

                // Draw cart
                let (cart_x, cart_y) = self.get_cart_polygon(x_pos);
                axes.lines(&cart_x, &cart_y, &[PlotOption::Color(colors[color_idx]), PlotOption::LineWidth(2.0)]);

                // Draw wheels
                let y_offset = WHEEL_RADIUS;
                let (w1x, w1y) = self.get_wheel_points(x_pos - CART_WIDTH / 4.0, y_offset);
                let (w2x, w2y) = self.get_wheel_points(x_pos + CART_WIDTH / 4.0, y_offset);
                axes.lines(&w1x, &w1y, &[PlotOption::Color(pendulum_colors[color_idx]), PlotOption::LineWidth(1.5)]);
                axes.lines(&w2x, &w2y, &[PlotOption::Color(pendulum_colors[color_idx]), PlotOption::LineWidth(1.5)]);

                // Draw pendulum
                let (pend_x, pend_y) = self.get_pendulum_points(x_pos, theta);
                axes.lines(&pend_x, &pend_y, &[PlotOption::Color(pendulum_colors[color_idx]), PlotOption::LineWidth(3.0)]);

                // Draw pendulum mass (circle at the end)
                let (mass_x, mass_y) = self.get_wheel_points(pend_x[1], pend_y[1]);
                axes.lines(&mass_x, &mass_y, &[PlotOption::Color(pendulum_colors[color_idx]), PlotOption::LineWidth(2.0)]);

                // Add time label for the last frame
                if frame_idx == num_frames - 1 {
                    axes.label(
                        &format!("t={:.1}s", time),
                        Coordinate::Graph(0.02),
                        Coordinate::Graph(0.95),
                        &[]
                    );
                }
            }

            // Add initial condition label
            let (_, initial_state) = &self.trajectory[0];
            axes.label(
                &format!("Initial angle: {:.1} deg", initial_state[2].to_degrees()),
                Coordinate::Graph(0.02),
                Coordinate::Graph(0.88),
                &[]
            );
        }

        let output_path = format!("img/inverted_pendulum/{}", filename);
        std::fs::create_dir_all("img/inverted_pendulum").unwrap();
        fg.set_terminal("pngcairo size 800,600", &output_path);
        fg.show().unwrap();
        println!("Inverted pendulum visualization saved to: {}", output_path);
    }

    /// Visualize single frame of cart-pendulum (for animation frames)
    pub fn visualize_frame(&self, step: usize, filename: &str) {
        if step >= self.trajectory.len() {
            return;
        }

        let (time, state) = &self.trajectory[step];
        let x_pos = state[0];
        let theta = state[2];

        let mut fg = Figure::new();
        {
            let axes = fg.axes2d()
                .set_title(&format!("Inverted Pendulum LQR Control  t={:.2}s", time), &[])
                .set_x_label("x [m]", &[])
                .set_y_label("y [m]", &[])
                .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0))
                .set_x_range(gnuplot::AutoOption::Fix(-3.0), gnuplot::AutoOption::Fix(3.0))
                .set_y_range(gnuplot::AutoOption::Fix(-1.0), gnuplot::AutoOption::Fix(4.0));

            // Draw ground line
            axes.lines(&[-4.0, 4.0], &[0.0, 0.0], &[PlotOption::Color("gray"), PlotOption::LineWidth(2.0)]);

            // Draw cart (blue, matching PythonRobotics)
            let (cart_x, cart_y) = self.get_cart_polygon(x_pos);
            axes.lines(&cart_x, &cart_y, &[PlotOption::Color("blue"), PlotOption::LineWidth(2.0)]);

            // Draw wheels (black)
            let y_offset = WHEEL_RADIUS;
            let (w1x, w1y) = self.get_wheel_points(x_pos - CART_WIDTH / 4.0, y_offset);
            let (w2x, w2y) = self.get_wheel_points(x_pos + CART_WIDTH / 4.0, y_offset);
            axes.lines(&w1x, &w1y, &[PlotOption::Color("black"), PlotOption::LineWidth(1.5)]);
            axes.lines(&w2x, &w2y, &[PlotOption::Color("black"), PlotOption::LineWidth(1.5)]);

            // Draw pendulum (black, matching PythonRobotics)
            let (pend_x, pend_y) = self.get_pendulum_points(x_pos, theta);
            axes.lines(&pend_x, &pend_y, &[PlotOption::Color("black"), PlotOption::LineWidth(3.0)]);

            // Draw pendulum mass
            let (mass_x, mass_y) = self.get_wheel_points(pend_x[1], pend_y[1]);
            axes.lines(&mass_x, &mass_y, &[PlotOption::Color("black"), PlotOption::LineWidth(2.0)]);

            // Add state info
            axes.label(
                &format!("x={:.2}m, θ={:.1}°", x_pos, theta.to_degrees()),
                Coordinate::Graph(0.02),
                Coordinate::Graph(0.95),
                &[]
            );
        }

        let output_path = format!("img/inverted_pendulum/{}", filename);
        fg.set_terminal("pngcairo size 640,480", &output_path);
        fg.show().unwrap();
    }
}

fn main() {
    // Create output directory
    std::fs::create_dir_all("img/inverted_pendulum").unwrap();

    let mut controller = InvertedPendulumLQR::new();

    // Initial state: [position, velocity, angle, angular_velocity]
    let x0 = Vector4::new(0.0, 0.0, 0.3, 0.0); // 0.3 rad initial angle (~17 deg)

    println!("Starting Inverted Pendulum LQR Control simulation...");

    if controller.simulate(x0) {
        // Generate summary image (multiple frames overlaid)
        controller.visualize("inverted_pendulum_lqr.png");

        // Generate individual frames for animation (optional)
        println!("Generating animation frames...");
        for (i, _) in controller.trajectory.iter().enumerate() {
            controller.visualize_frame(i, &format!("frame_{:03}.png", i));
        }

        println!("Inverted Pendulum LQR Control simulation complete!");
        println!("To create GIF animation, run:");
        println!("  convert -delay 10 -loop 0 img/inverted_pendulum/frame_*.png img/inverted_pendulum/animation_lqr.gif");
    } else {
        println!("Simulation failed!");
    }
}
