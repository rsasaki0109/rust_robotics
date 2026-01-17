//
// Inverse kinematics of a two-joint arm
// Author: Daniel Ingram (daniel-s-ingram)
//         Atsushi Sakai (@Atsushi_twi)
// Ported to Rust by: rust_robotics team
//

use gnuplot::{Figure, Caption, Color, AxesCommon, PointSymbol};
use std::f64::consts::PI;

// Simulation parameters
const KP: f64 = 15.0;
const DT: f64 = 0.01;

// Link lengths
const L1: f64 = 1.0;
const L2: f64 = 1.0;

pub struct TwoJointArm {
    pub theta1: f64,
    pub theta2: f64,
    pub target_x: f64,
    pub target_y: f64,
    pub trajectory: Vec<(f64, f64, f64, f64)>, // (theta1, theta2, end_x, end_y)
}

impl TwoJointArm {
    pub fn new() -> Self {
        TwoJointArm {
            theta1: 0.0,
            theta2: 0.0,
            target_x: 2.0,
            target_y: 0.0,
            trajectory: Vec::new(),
        }
    }

    pub fn set_target(&mut self, x: f64, y: f64) {
        self.target_x = x;
        self.target_y = y;
    }

    pub fn solve_inverse_kinematics(&mut self, goal_threshold: f64) -> bool {
        let mut iterations = 0;
        const MAX_ITERATIONS: usize = 1000;
        
        self.trajectory.clear();
        
        while iterations < MAX_ITERATIONS {
            let (end_x, end_y) = self.forward_kinematics();
            self.trajectory.push((self.theta1, self.theta2, end_x, end_y));
            
            // Check if goal is reached
            let distance_to_goal = ((end_x - self.target_x).powi(2) + (end_y - self.target_y).powi(2)).sqrt();
            if distance_to_goal < goal_threshold {
                return true;
            }
            
            // Calculate target joint angles using inverse kinematics
            let (theta1_goal, theta2_goal) = match self.calculate_target_angles() {
                Some(angles) => angles,
                None => continue, // Target unreachable, skip this iteration
            };
            
            // Update joint angles with proportional control
            self.theta1 += KP * self.angle_diff(theta1_goal, self.theta1) * DT;
            self.theta2 += KP * self.angle_diff(theta2_goal, self.theta2) * DT;
            
            iterations += 1;
        }
        
        false // Failed to reach target within max iterations
    }

    fn calculate_target_angles(&self) -> Option<(f64, f64)> {
        let x = self.target_x;
        let y = self.target_y;
        
        // Check if target is reachable
        let distance = (x.powi(2) + y.powi(2)).sqrt();
        if distance > (L1 + L2) {
            return None; // Target too far
        }
        
        // Calculate theta2 using law of cosines
        let cos_theta2 = (x.powi(2) + y.powi(2) - L1.powi(2) - L2.powi(2)) / (2.0 * L1 * L2);
        
        // Clamp to valid range to avoid numerical errors
        let cos_theta2 = cos_theta2.clamp(-1.0, 1.0);
        let mut theta2_goal = cos_theta2.acos();
        
        // Calculate theta1
        let tmp = (L2 * theta2_goal.sin()).atan2(L1 + L2 * theta2_goal.cos());
        let mut theta1_goal = y.atan2(x) - tmp;
        
        // Try the other solution if theta1 is negative
        if theta1_goal < 0.0 {
            theta2_goal = -theta2_goal;
            let tmp = (L2 * theta2_goal.sin()).atan2(L1 + L2 * theta2_goal.cos());
            theta1_goal = y.atan2(x) - tmp;
        }
        
        Some((theta1_goal, theta2_goal))
    }

    fn forward_kinematics(&self) -> (f64, f64) {
        let elbow_x = L1 * self.theta1.cos();
        let elbow_y = L1 * self.theta1.sin();
        
        let end_x = elbow_x + L2 * (self.theta1 + self.theta2).cos();
        let end_y = elbow_y + L2 * (self.theta1 + self.theta2).sin();
        
        (end_x, end_y)
    }

    fn angle_diff(&self, theta1: f64, theta2: f64) -> f64 {
        let mut diff = theta1 - theta2;
        while diff > PI {
            diff -= 2.0 * PI;
        }
        while diff < -PI {
            diff += 2.0 * PI;
        }
        diff
    }

    pub fn save_animation_frames(&self, target_name: &str) {
        let output_dir = format!("img/arm_navigation/{}", target_name);
        std::fs::create_dir_all(&output_dir).unwrap();
        
        for (frame, &(theta1, theta2, end_x, end_y)) in self.trajectory.iter().enumerate() {
            if frame % 5 == 0 { // Save every 5th frame
                self.save_frame(frame / 5, theta1, theta2, end_x, end_y, &output_dir);
            }
        }
        
        println!("Animation frames saved in {}/frame_*.png", output_dir);
        println!("Create video with: ffmpeg -r 10 -i {}/frame_%04d.png -c:v libx264 -pix_fmt yuv420p {}_animation.mp4", output_dir, target_name);
    }

    fn save_frame(&self, frame: usize, theta1: f64, theta2: f64, end_x: f64, end_y: f64, output_dir: &str) {
        let mut fg = Figure::new();
        {
            let axes = fg.axes2d()
                .set_title(&format!("Two Joint Arm Control - Frame {} (Target: {:.2}, {:.2})", frame, self.target_x, self.target_y), &[])
                .set_x_label("X [m]", &[])
                .set_y_label("Y [m]", &[])
                .set_x_range(gnuplot::Fix(-2.5), gnuplot::Fix(2.5))
                .set_y_range(gnuplot::Fix(-2.5), gnuplot::Fix(2.5))
                .set_aspect_ratio(gnuplot::Fix(1.0));

            // Calculate joint positions
            let shoulder = (0.0, 0.0);
            let elbow = (L1 * theta1.cos(), L1 * theta1.sin());
            let wrist = (end_x, end_y);

            // Draw arm links (black lines like Python version)
            axes.lines(&[shoulder.0, elbow.0], &[shoulder.1, elbow.1], &[
                Color("black")
            ]);
            axes.lines(&[elbow.0, wrist.0], &[elbow.1, wrist.1], &[
                Color("black")
            ]);

            // Draw joints (red circles like Python version)
            axes.points(&[shoulder.0, elbow.0, wrist.0], &[shoulder.1, elbow.1, wrist.1], &[
                Color("red"), PointSymbol('O')
            ]);

            // Draw target (green star like Python version)
            axes.points(&[self.target_x], &[self.target_y], &[
                Color("green"), PointSymbol('*')
            ]);

            // Draw line to target (green dashed line like Python version)
            axes.lines(&[wrist.0, self.target_x], &[wrist.1, self.target_y], &[
                Color("green")
            ]);
        }

        let output_path = format!("{}/frame_{:04}.png", output_dir, frame);
        fg.set_terminal("pngcairo", &output_path);
        fg.show().unwrap();
    }

    /// Save a single arm visualization image (like PythonRobotics)
    pub fn save_arm_plot(&self, output_path: &str) {
        let (end_x, end_y) = self.forward_kinematics();

        let mut fg = Figure::new();
        {
            let axes = fg.axes2d()
                .set_title("Two Joint Arm to Point Control", &[])
                .set_x_label("X [m]", &[])
                .set_y_label("Y [m]", &[])
                .set_x_range(gnuplot::Fix(-2.5), gnuplot::Fix(2.5))
                .set_y_range(gnuplot::Fix(-2.5), gnuplot::Fix(2.5))
                .set_aspect_ratio(gnuplot::Fix(1.0));

            // Calculate joint positions
            let shoulder = (0.0, 0.0);
            let elbow = (L1 * self.theta1.cos(), L1 * self.theta1.sin());
            let wrist = (end_x, end_y);

            // Draw arm links (black lines like Python version)
            axes.lines(&[shoulder.0, elbow.0], &[shoulder.1, elbow.1], &[
                Color("black")
            ]);
            axes.lines(&[elbow.0, wrist.0], &[elbow.1, wrist.1], &[
                Color("black")
            ]);

            // Draw joints (red circles like Python version)
            axes.points(&[shoulder.0, elbow.0, wrist.0], &[shoulder.1, elbow.1, wrist.1], &[
                Color("red"), PointSymbol('O')
            ]);

            // Draw target (green star like Python version)
            axes.points(&[self.target_x], &[self.target_y], &[
                Color("green"), PointSymbol('*')
            ]);

            // Draw line to target (green dashed line like Python version)
            axes.lines(&[wrist.0, self.target_x], &[wrist.1, self.target_y], &[
                Color("green")
            ]);
        }

        fg.set_terminal("pngcairo", output_path);
        fg.show().unwrap();
        println!("Arm plot saved to: {}", output_path);
    }

    pub fn create_summary_plot(&self, target_name: &str) {
        let mut fg = Figure::new();
        {
            let axes = fg.axes2d()
                .set_title("Two Joint Arm - Joint Angles vs Time", &[])
                .set_x_label("Iteration", &[])
                .set_y_label("Angle [rad]", &[]);

            let iterations: Vec<f64> = (0..self.trajectory.len()).map(|i| i as f64).collect();
            let theta1_data: Vec<f64> = self.trajectory.iter().map(|(t1, _, _, _)| *t1).collect();
            let theta2_data: Vec<f64> = self.trajectory.iter().map(|(_, t2, _, _)| *t2).collect();
            
            axes.lines(&iterations, &theta1_data, &[Caption("Theta1 [rad]"), Color("blue")]);
            axes.lines(&iterations, &theta2_data, &[Caption("Theta2 [rad]"), Color("red")]);
        }

        let output_path = format!("img/arm_navigation/{}_summary.png", target_name);
        std::fs::create_dir_all("img/arm_navigation").unwrap();
        fg.set_terminal("pngcairo", &output_path);
        fg.show().unwrap();
        println!("Summary plot saved to: {}", output_path);
    }

    pub fn run_random_targets_demo(&mut self, num_targets: usize) {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        
        println!("Starting Two Joint Arm random targets demo...");
        
        for i in 0..num_targets {
            // Generate random target within workspace
            let angle = rng.gen::<f64>() * 2.0 * PI;
            let radius = rng.gen::<f64>() * (L1 + L2 - 0.1); // Slightly inside workspace
            let target_x = radius * angle.cos();
            let target_y = radius * angle.sin();
            
            self.set_target(target_x, target_y);
            
            println!("Target {}: ({:.2}, {:.2})", i + 1, target_x, target_y);
            
            if self.solve_inverse_kinematics(0.01) {
                println!("  ✓ Reached target in {} iterations", self.trajectory.len());
                let target_name = format!("target_{:02}", i + 1);
                self.save_animation_frames(&target_name);
            } else {
                println!("  ✗ Failed to reach target");
            }
        }
        
        self.create_summary_plot("random_demo");
        println!("Two Joint Arm demo complete!");
    }
}

fn main() {
    std::fs::create_dir_all("img/arm_navigation").unwrap();

    let mut arm = TwoJointArm::new();

    // Set a target and solve inverse kinematics (like PythonRobotics demo)
    arm.set_target(1.5, 0.5);

    if arm.solve_inverse_kinematics(0.01) {
        println!("Reached target in {} iterations", arm.trajectory.len());

        // Save the final arm position as a single image (like PythonRobotics)
        arm.save_arm_plot("img/arm_navigation/two_joint_arm_control.png");
    } else {
        println!("Failed to reach target");
    }
}
