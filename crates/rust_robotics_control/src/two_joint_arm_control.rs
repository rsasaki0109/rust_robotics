#![allow(
    dead_code,
    clippy::needless_borrows_for_generic_args,
    clippy::new_without_default
)]

//
// Inverse kinematics of a two-joint arm
// Author: Daniel Ingram (daniel-s-ingram)
//         Atsushi Sakai (@Atsushi_twi)
// Ported to Rust by: rust_robotics team
//

#[cfg(feature = "viz")]
use gnuplot::{AxesCommon, Caption, Color, Figure, PointSymbol};

use std::f64::consts::PI;

// Simulation parameters
const KP: f64 = 15.0;
const DT: f64 = 0.01;

// Link lengths
const L1: f64 = 1.0;
const L2: f64 = 1.0;

/// The sign of the elbow angle in the two analytic IK solutions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ElbowBranch {
    Positive,
    Negative,
}

pub struct TwoJointArm {
    pub theta1: f64,
    pub theta2: f64,
    pub target_x: f64,
    pub target_y: f64,
    pub trajectory: Vec<(f64, f64, f64, f64)>, // (theta1, theta2, end_x, end_y)
}

impl TwoJointArm {
    /// Link lengths in metres; this planar arm has unrestricted revolute joints.
    pub const LINK_LENGTHS: [f64; 2] = [L1, L2];

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

        if !goal_threshold.is_finite()
            || goal_threshold <= 0.0
            || !self.theta1.is_finite()
            || !self.theta2.is_finite()
        {
            return false;
        }

        while iterations < MAX_ITERATIONS {
            let (end_x, end_y) = self.forward_kinematics();
            self.trajectory
                .push((self.theta1, self.theta2, end_x, end_y));

            // Validate reachability before applying the numerical goal tolerance.
            let (theta1_goal, theta2_goal) = match self.calculate_target_angles() {
                Some(angles) => angles,
                None => return false,
            };

            // Check if goal is reached
            let distance_to_goal =
                ((end_x - self.target_x).powi(2) + (end_y - self.target_y).powi(2)).sqrt();
            if distance_to_goal < goal_threshold {
                return true;
            }

            // Update joint angles with proportional control
            self.theta1 += KP * self.angle_diff(theta1_goal, self.theta1) * DT;
            self.theta2 += KP * self.angle_diff(theta2_goal, self.theta2) * DT;

            iterations += 1;
        }

        false // Failed to reach target within max iterations
    }

    /// Solve one analytic IK branch without moving the arm.
    ///
    /// Returns `None` for nonfinite targets or targets outside the reachable
    /// disk. At the origin both unit links fold; the shoulder angle is
    /// underdetermined, so zero is returned as a deterministic representative.
    pub fn inverse_kinematics(x: f64, y: f64, branch: ElbowBranch) -> Option<(f64, f64)> {
        if !x.is_finite() || !y.is_finite() || x.hypot(y) > L1 + L2 {
            return None;
        }
        let cos_theta2 = (x.powi(2) + y.powi(2) - L1.powi(2) - L2.powi(2)) / (2.0 * L1 * L2);
        let sign = if branch == ElbowBranch::Positive {
            1.0
        } else {
            -1.0
        };
        let theta2 = sign * cos_theta2.clamp(-1.0, 1.0).acos();
        let theta1 = if x == 0.0 && y == 0.0 {
            0.0
        } else {
            y.atan2(x) - (L2 * theta2.sin()).atan2(L1 + L2 * theta2.cos())
        };
        Some((theta1, theta2))
    }

    fn calculate_target_angles(&self) -> Option<(f64, f64)> {
        let positive =
            Self::inverse_kinematics(self.target_x, self.target_y, ElbowBranch::Positive)?;
        if positive.0 < 0.0 {
            Self::inverse_kinematics(self.target_x, self.target_y, ElbowBranch::Negative)
        } else {
            Some(positive)
        }
    }

    pub fn forward_kinematics(&self) -> (f64, f64) {
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
}

// Visualization methods gated behind the "viz" feature
#[cfg(feature = "viz")]
impl TwoJointArm {
    pub fn save_animation_frames(&self, target_name: &str) {
        let output_dir = format!("img/arm_navigation/{}", target_name);
        std::fs::create_dir_all(&output_dir).unwrap();

        for (frame, &(theta1, theta2, end_x, end_y)) in self.trajectory.iter().enumerate() {
            if frame % 5 == 0 {
                // Save every 5th frame
                self.save_frame(frame / 5, theta1, theta2, end_x, end_y, &output_dir);
            }
        }

        println!("Animation frames saved in {}/frame_*.png", output_dir);
        println!("Create video with: ffmpeg -r 10 -i {}/frame_%04d.png -c:v libx264 -pix_fmt yuv420p {}_animation.mp4", output_dir, target_name);
    }

    fn save_frame(
        &self,
        frame: usize,
        theta1: f64,
        _theta2: f64,
        end_x: f64,
        end_y: f64,
        output_dir: &str,
    ) {
        let mut fg = Figure::new();
        {
            let axes = fg
                .axes2d()
                .set_title(
                    &format!(
                        "Two Joint Arm Control - Frame {} (Target: {:.2}, {:.2})",
                        frame, self.target_x, self.target_y
                    ),
                    &[],
                )
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
            axes.lines(
                &[shoulder.0, elbow.0],
                &[shoulder.1, elbow.1],
                &[Color("black")],
            );
            axes.lines(&[elbow.0, wrist.0], &[elbow.1, wrist.1], &[Color("black")]);

            // Draw joints (red circles like Python version)
            axes.points(
                &[shoulder.0, elbow.0, wrist.0],
                &[shoulder.1, elbow.1, wrist.1],
                &[Color("red"), PointSymbol('O')],
            );

            // Draw target (green star like Python version)
            axes.points(
                &[self.target_x],
                &[self.target_y],
                &[Color("green"), PointSymbol('*')],
            );

            // Draw line to target (green dashed line like Python version)
            axes.lines(
                &[wrist.0, self.target_x],
                &[wrist.1, self.target_y],
                &[Color("green")],
            );
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
            let axes = fg
                .axes2d()
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
            axes.lines(
                &[shoulder.0, elbow.0],
                &[shoulder.1, elbow.1],
                &[Color("black")],
            );
            axes.lines(&[elbow.0, wrist.0], &[elbow.1, wrist.1], &[Color("black")]);

            // Draw joints (red circles like Python version)
            axes.points(
                &[shoulder.0, elbow.0, wrist.0],
                &[shoulder.1, elbow.1, wrist.1],
                &[Color("red"), PointSymbol('O')],
            );

            // Draw target (green star like Python version)
            axes.points(
                &[self.target_x],
                &[self.target_y],
                &[Color("green"), PointSymbol('*')],
            );

            // Draw line to target (green dashed line like Python version)
            axes.lines(
                &[wrist.0, self.target_x],
                &[wrist.1, self.target_y],
                &[Color("green")],
            );
        }

        fg.set_terminal("pngcairo", output_path);
        fg.show().unwrap();
        println!("Arm plot saved to: {}", output_path);
    }

    pub fn create_summary_plot(&self, target_name: &str) {
        let mut fg = Figure::new();
        {
            let axes = fg
                .axes2d()
                .set_title("Two Joint Arm - Joint Angles vs Time", &[])
                .set_x_label("Iteration", &[])
                .set_y_label("Angle [rad]", &[]);

            let iterations: Vec<f64> = (0..self.trajectory.len()).map(|i| i as f64).collect();
            let theta1_data: Vec<f64> = self.trajectory.iter().map(|(t1, _, _, _)| *t1).collect();
            let theta2_data: Vec<f64> = self.trajectory.iter().map(|(_, t2, _, _)| *t2).collect();

            axes.lines(
                &iterations,
                &theta1_data,
                &[Caption("Theta1 [rad]"), Color("blue")],
            );
            axes.lines(
                &iterations,
                &theta2_data,
                &[Caption("Theta2 [rad]"), Color("red")],
            );
        }

        let output_path = format!("img/arm_navigation/{}_summary.png", target_name);
        std::fs::create_dir_all("img/arm_navigation").unwrap();
        fg.set_terminal("pngcairo", &output_path);
        fg.show().unwrap();
        println!("Summary plot saved to: {}", output_path);
    }

    pub fn run_random_targets_demo(&mut self, num_targets: usize) {
        use rand::Rng;
        let mut rng = rand::rng();

        println!("Starting Two Joint Arm random targets demo...");

        for i in 0..num_targets {
            // Generate random target within workspace
            let angle = rng.random::<f64>() * 2.0 * PI;
            let radius = rng.random::<f64>() * (L1 + L2 - 0.1); // Slightly inside workspace
            let target_x = radius * angle.cos();
            let target_y = radius * angle.sin();

            self.set_target(target_x, target_y);

            println!("Target {}: ({:.2}, {:.2})", i + 1, target_x, target_y);

            if self.solve_inverse_kinematics(0.01) {
                println!("  Reached target in {} iterations", self.trajectory.len());
                let target_name = format!("target_{:02}", i + 1);
                self.save_animation_frames(&target_name);
            } else {
                println!("  Failed to reach target");
            }
        }

        self.create_summary_plot("random_demo");
        println!("Two Joint Arm demo complete!");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_forward_kinematics_at_origin() {
        let arm = TwoJointArm::new();
        let (x, y) = arm.forward_kinematics();
        assert!((x - 2.0).abs() < 1e-10); // L1 + L2
        assert!(y.abs() < 1e-10);
    }

    #[test]
    fn test_solve_ik_reaches_target() {
        let mut arm = TwoJointArm::new();
        arm.set_target(1.5, 0.5);
        assert!(arm.solve_inverse_kinematics(0.01));

        let (end_x, end_y) = arm.forward_kinematics();
        let distance = ((end_x - 1.5).powi(2) + (end_y - 0.5).powi(2)).sqrt();
        assert!(distance < 0.01);
    }

    #[test]
    fn both_analytic_branches_reach_targets_in_every_quadrant() {
        for radius in [0.0, 0.001, 0.5, 1.3, 1.999, 2.0] {
            for direction in 0..32 {
                let angle = f64::from(direction) * 2.0 * PI / 32.0;
                let (x, y) = (radius * angle.cos(), radius * angle.sin());
                // Keep exact boundary targets within the disk despite roundoff.
                let scale = if x.hypot(y) > 2.0 {
                    2.0 / x.hypot(y)
                } else {
                    1.0
                };
                for branch in [ElbowBranch::Positive, ElbowBranch::Negative] {
                    let (theta1, theta2) =
                        TwoJointArm::inverse_kinematics(x * scale, y * scale, branch).unwrap();
                    let mut arm = TwoJointArm::new();
                    arm.theta1 = theta1;
                    arm.theta2 = theta2;
                    let (px, py) = arm.forward_kinematics();
                    assert!((px - x * scale).hypot(py - y * scale) < 1e-9);
                    assert_eq!(theta2.is_sign_positive(), branch == ElbowBranch::Positive);
                }
            }
        }
    }

    #[test]
    fn unreachable_or_invalid_targets_terminate_without_moving() {
        for (x, y) in [
            (2.01, 0.0),
            (0.0, -3.0),
            (f64::NAN, 1.0),
            (f64::INFINITY, 0.0),
        ] {
            assert!(TwoJointArm::inverse_kinematics(x, y, ElbowBranch::Positive).is_none());
            assert!(TwoJointArm::inverse_kinematics(x, y, ElbowBranch::Negative).is_none());
            let mut arm = TwoJointArm::new();
            arm.set_target(x, y);
            assert!(!arm.solve_inverse_kinematics(0.01));
            assert_eq!((arm.theta1, arm.theta2), (0.0, 0.0));
            assert_eq!(arm.trajectory.len(), 1);
        }
        for threshold in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(!TwoJointArm::new().solve_inverse_kinematics(threshold));
        }
        for (a, b) in [
            (f64::INFINITY, 0.0),
            (0.0, f64::NEG_INFINITY),
            (f64::NAN, 1.0),
        ] {
            let mut arm = TwoJointArm::new();
            arm.theta1 = a;
            arm.theta2 = b;
            assert!(!arm.solve_inverse_kinematics(0.01));
            assert!(arm.trajectory.is_empty());
        }
    }
}
