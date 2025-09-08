//
// Inverted Pendulum LQR control
// author: Trung Kien - letrungkien.k53.hut@gmail.com
// Ported to Rust by: rust_robotics team
//

use gnuplot::{Figure, Caption, Color, AxesCommon};
use nalgebra::{Matrix4, Matrix1, Vector4, Matrix1x4};

// Model parameters
const L_BAR: f64 = 2.0;  // length of bar
const M: f64 = 1.0;      // [kg] cart mass
const MASS: f64 = 0.3;   // [kg] pendulum mass
const G: f64 = 9.8;      // [m/s^2] gravity

const DELTA_T: f64 = 0.1; // time tick [s]
const SIM_TIME: f64 = 5.0; // simulation time [s]

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

    pub fn visualize(&self, filename: &str) {
        let mut fg = Figure::new();
        {
            let axes = fg.axes2d()
                .set_title("Inverted Pendulum LQR Control", &[])
                .set_x_label("Time [s]", &[])
                .set_y_label("State", &[]);

            // Plot position
            let time: Vec<f64> = self.trajectory.iter().map(|(t, _)| *t).collect();
            let position: Vec<f64> = self.trajectory.iter().map(|(_, x)| x[0]).collect();
            let angle: Vec<f64> = self.trajectory.iter().map(|(_, x)| x[2].to_degrees()).collect();
            
            axes.lines(&time, &position, &[Caption("Position [m]"), Color("blue")]);
            axes.lines(&time, &angle, &[Caption("Angle [deg]"), Color("red")]);
        }

        let output_path = format!("img/inverted_pendulum/{}", filename);
        std::fs::create_dir_all("img/inverted_pendulum").unwrap();
        fg.set_terminal("pngcairo", &output_path);
        fg.show().unwrap();
        println!("Inverted pendulum visualization saved to: {}", output_path);
    }
}

fn main() {
    // Create output directory
    std::fs::create_dir_all("img/inverted_pendulum").unwrap();

    let mut controller = InvertedPendulumLQR::new();
    
    // Initial state: [position, velocity, angle, angular_velocity]
    let x0 = Vector4::new(0.0, 0.0, 0.3, 0.0); // 0.3 rad initial angle
    
    println!("Starting Inverted Pendulum LQR Control simulation...");
    
    if controller.simulate(x0) {
        controller.visualize("inverted_pendulum_lqr.png");
        println!("Inverted Pendulum LQR Control simulation complete!");
    } else {
        println!("Simulation failed!");
    }
}
