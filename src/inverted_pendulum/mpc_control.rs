//
// Inverted Pendulum MPC control
// author: Atsushi Sakai
// Ported to Rust by: rust_robotics team
//

use gnuplot::{Figure, Caption, Color, AxesCommon, PointSymbol};
use nalgebra::{Matrix4, Matrix1, Vector4, DMatrix, DVector};
use std::f64::consts::PI;

// Model parameters
const L_BAR: f64 = 2.0;  // length of bar
const M: f64 = 1.0;      // [kg] cart mass
const MASS: f64 = 0.3;   // [kg] pendulum mass
const G: f64 = 9.8;      // [m/s^2] gravity

const T: usize = 30;     // Horizon length
const DELTA_T: f64 = 0.1; // time tick [s]
const SIM_TIME: f64 = 5.0; // simulation time [s]

pub struct InvertedPendulumMPC {
    pub q: Matrix4<f64>,     // state cost matrix
    pub r: Matrix1<f64>,     // input cost matrix
    pub trajectory: Vec<(f64, Vector4<f64>)>, // time, state history
    pub prediction_history: Vec<Vec<Vector4<f64>>>, // predicted trajectories for animation
}

impl InvertedPendulumMPC {
    pub fn new() -> Self {
        let mut q = Matrix4::<f64>::zeros();
        q[(1, 1)] = 1.0;  // velocity cost
        q[(2, 2)] = 1.0;  // angle cost
        
        let r = Matrix1::<f64>::from_element(0.01); // input cost
        
        InvertedPendulumMPC {
            q,
            r,
            trajectory: Vec::new(),
            prediction_history: Vec::new(),
        }
    }

    pub fn simulate(&mut self, x0: Vector4<f64>) -> bool {
        let mut x = x0;
        let mut time = 0.0;
        
        self.trajectory.clear();
        self.prediction_history.clear();
        self.trajectory.push((time, x));

        let mut frame_count = 0;
        while time < SIM_TIME {
            time += DELTA_T;

            // Calculate MPC control input and predicted trajectory
            let (predicted_states, u) = self.mpc_control(&x);
            
            // Store prediction for animation
            self.prediction_history.push(predicted_states);

            // Simulate inverted pendulum cart
            x = self.simulation_step(&x, u);
            
            self.trajectory.push((time, x));

            // Generate frame for animation
            if frame_count % 5 == 0 { // Every 5th frame to reduce file count
                self.save_animation_frame(frame_count / 5, time, &x, &self.prediction_history.last().unwrap());
            }
            frame_count += 1;
        }

        println!("MPC simulation finished");
        println!("Final state: x={:.2} [m], theta={:.2} [deg]", 
                x[0], x[2].to_degrees());
        
        true
    }

    fn simulation_step(&self, x: &Vector4<f64>, u: Matrix1<f64>) -> Vector4<f64> {
        let (a, b) = self.get_model_matrix();
        a * x + b * u[0]
    }

    fn mpc_control(&self, x0: &Vector4<f64>) -> (Vec<Vector4<f64>>, Matrix1<f64>) {
        // Simplified MPC using iterative LQR approach (since we don't have cvxpy)
        let (a, b) = self.get_model_matrix();
        
        // Initialize control sequence
        let mut u_seq = vec![0.0; T];
        let mut best_cost = f64::INFINITY;
        let mut best_u_seq = u_seq.clone();
        
        // Simple gradient descent optimization
        for _iter in 0..50 {
            let (states, cost) = self.simulate_prediction(x0, &u_seq, &a, &b);
            
            if cost < best_cost {
                best_cost = cost;
                best_u_seq = u_seq.clone();
            }
            
            // Update control sequence using simple gradient
            for t in 0..T {
                let mut u_plus = u_seq.clone();
                let mut u_minus = u_seq.clone();
                let delta = 0.01;
                
                u_plus[t] += delta;
                u_minus[t] -= delta;
                
                let (_, cost_plus) = self.simulate_prediction(x0, &u_plus, &a, &b);
                let (_, cost_minus) = self.simulate_prediction(x0, &u_minus, &a, &b);
                
                let gradient = (cost_plus - cost_minus) / (2.0 * delta);
                u_seq[t] -= 0.1 * gradient; // Learning rate
                
                // Clamp control input
                u_seq[t] = u_seq[t].clamp(-10.0, 10.0);
            }
        }
        
        let (predicted_states, _) = self.simulate_prediction(x0, &best_u_seq, &a, &b);
        (predicted_states, Matrix1::from_element(best_u_seq[0]))
    }

    fn simulate_prediction(&self, x0: &Vector4<f64>, u_seq: &[f64], a: &Matrix4<f64>, b: &Vector4<f64>) -> (Vec<Vector4<f64>>, f64) {
        let mut x = *x0;
        let mut states = vec![x];
        let mut cost = 0.0;
        
        for t in 0..T {
            // State cost
            cost += (x.transpose() * self.q * x)[0];
            
            // Control cost
            cost += u_seq[t] * self.r[0] * u_seq[t];
            
            // Update state
            x = a * x + b * u_seq[t];
            states.push(x);
        }
        
        (states, cost)
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

    fn save_animation_frame(&self, frame: usize, time: f64, current_state: &Vector4<f64>, predicted_states: &[Vector4<f64>]) {
        let mut fg = Figure::new();
        {
            let mut axes = fg.axes2d()
                .set_title(&format!("Inverted Pendulum MPC Control - Frame {} (t={:.1}s)", frame, time), &[])
                .set_x_label("Position [m]", &[])
                .set_y_label("Height [m]", &[])
                .set_x_range(gnuplot::Fix(-6.0), gnuplot::Fix(3.0))
                .set_y_range(gnuplot::Fix(-0.5), gnuplot::Fix(3.0))
                .set_aspect_ratio(gnuplot::Fix(1.0));

            // Draw cart and pendulum
            self.draw_cart_pendulum(&mut axes, current_state[0], current_state[2]);
            
            // Draw predicted trajectory
            if predicted_states.len() > 1 {
                let pred_x: Vec<f64> = predicted_states.iter().map(|s| s[0]).collect();
                let pred_angles: Vec<f64> = predicted_states.iter().map(|s| s[2]).collect();
                
                // Draw predicted cart positions
                axes.points(&pred_x, &vec![0.2; pred_x.len()], &[
                    Caption("Predicted Path"), 
                    Color("red"), 
                    PointSymbol('.')
                ]);
                
                // Draw predicted pendulum tips
                let pred_tip_x: Vec<f64> = predicted_states.iter()
                    .map(|s| s[0] + L_BAR * s[2].sin()).collect();
                let pred_tip_y: Vec<f64> = predicted_states.iter()
                    .map(|s| 0.5 + L_BAR * s[2].cos()).collect();
                    
                axes.points(&pred_tip_x, &pred_tip_y, &[
                    Caption("Predicted Pendulum"), 
                    Color("orange"), 
                    PointSymbol('x')
                ]);
            }
        }

        let output_path = format!("img/inverted_pendulum/mpc/mpc_frame_{:04}.png", frame);
        std::fs::create_dir_all("img/inverted_pendulum/mpc").unwrap();
        fg.set_terminal("pngcairo", &output_path);
        fg.show().unwrap();
    }

    fn draw_cart_pendulum(&self, axes: &mut gnuplot::Axes2D, x: f64, theta: f64) {
        let cart_w = 1.0;
        let cart_h = 0.5;
        let radius = 0.1;

        // Cart body
        let cx = vec![x - cart_w/2.0, x + cart_w/2.0, x + cart_w/2.0, x - cart_w/2.0, x - cart_w/2.0];
        let cy = vec![radius*2.0, radius*2.0, cart_h + radius*2.0, cart_h + radius*2.0, radius*2.0];
        axes.lines(&cx, &cy, &[Caption("Cart"), Color("blue")]);

        // Pendulum rod
        let bx = vec![x, x + L_BAR * theta.sin()];
        let by = vec![cart_h + radius*2.0, cart_h + radius*2.0 + L_BAR * theta.cos()];
        axes.lines(&bx, &by, &[Caption("Pendulum"), Color("black")]);

        // Wheels
        let angles: Vec<f64> = (0..120).map(|i| i as f64 * PI / 60.0).collect();
        let wheel_x: Vec<f64> = angles.iter().map(|a| x - cart_w/4.0 + radius * a.cos()).collect();
        let wheel_y: Vec<f64> = angles.iter().map(|a| radius + radius * a.sin()).collect();
        axes.lines(&wheel_x, &wheel_y, &[Color("black")]);

        let wheel_x2: Vec<f64> = angles.iter().map(|a| x + cart_w/4.0 + radius * a.cos()).collect();
        axes.lines(&wheel_x2, &wheel_y, &[Color("black")]);

        // Pendulum mass
        let mass_x: Vec<f64> = angles.iter().map(|a| bx[1] + radius * a.cos()).collect();
        let mass_y: Vec<f64> = angles.iter().map(|a| by[1] + radius * a.sin()).collect();
        axes.lines(&mass_x, &mass_y, &[Color("red")]);
    }

    pub fn create_summary_plot(&self) {
        let mut fg = Figure::new();
        {
            let axes = fg.axes2d()
                .set_title("Inverted Pendulum MPC Control - Summary", &[])
                .set_x_label("Time [s]", &[])
                .set_y_label("State", &[]);

            let time: Vec<f64> = self.trajectory.iter().map(|(t, _)| *t).collect();
            let position: Vec<f64> = self.trajectory.iter().map(|(_, x)| x[0]).collect();
            let angle: Vec<f64> = self.trajectory.iter().map(|(_, x)| x[2].to_degrees()).collect();
            
            axes.lines(&time, &position, &[Caption("Position [m]"), Color("blue")]);
            axes.lines(&time, &angle, &[Caption("Angle [deg]"), Color("red")]);
        }

        let output_path = "img/inverted_pendulum/mpc/mpc_summary.png";
        fg.set_terminal("pngcairo", output_path);
        fg.show().unwrap();
        println!("MPC summary plot saved to: {}", output_path);
    }
}

fn main() {
    // Create output directory
    std::fs::create_dir_all("img/inverted_pendulum").unwrap();

    let mut controller = InvertedPendulumMPC::new();
    
    // Initial state: [position, velocity, angle, angular_velocity]
    let x0 = Vector4::new(0.0, 0.0, 0.3, 0.0); // 0.3 rad initial angle
    
    println!("Starting Inverted Pendulum MPC Control simulation...");
    
    if controller.simulate(x0) {
        controller.create_summary_plot();
        println!("MPC Control simulation complete!");
        println!("Animation frames saved in img/inverted_pendulum/mpc/mpc_frame_*.png");
        println!("You can create a video with: ffmpeg -r 10 -i img/inverted_pendulum/mpc/mpc_frame_%04d.png -c:v libx264 -pix_fmt yuv420p mpc_animation.mp4");
    } else {
        println!("Simulation failed!");
    }
}
