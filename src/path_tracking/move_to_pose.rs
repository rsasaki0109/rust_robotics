//
// Move to specified pose
// Author: Daniel Ingram (daniel-s-ingram)
//         Atsushi Sakai(@Atsushi_twi)
//         Ryohei Sasaki(@rsasaki0109)
// P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7
use crate::common::{ControlInput, Path2D, Point2D, Pose2D};
use gnuplot::{AxesCommon, Caption, Color, Figure};
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy)]
pub struct MoveToPoseConfig {
    pub kp_rho: f64,
    pub kp_alpha: f64,
    pub kp_beta: f64,
    pub dt: f64,
    pub goal_tolerance: f64,
    pub yaw_tolerance: f64,
    pub max_steps: usize,
}

impl Default for MoveToPoseConfig {
    fn default() -> Self {
        Self {
            kp_rho: 9.0,
            kp_alpha: 15.0,
            kp_beta: -3.0,
            dt: 0.01,
            goal_tolerance: 0.001,
            yaw_tolerance: 0.05,
            max_steps: 10_000,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct MoveToPoseStep {
    pub pose: Pose2D,
    pub control: ControlInput,
    pub distance_to_goal: f64,
}

#[derive(Debug, Clone)]
pub struct MoveToPoseResult {
    pub steps: Vec<MoveToPoseStep>,
    pub converged: bool,
}

impl MoveToPoseResult {
    pub fn final_pose(&self) -> Pose2D {
        self.steps
            .last()
            .map(|step| step.pose)
            .unwrap_or_else(Pose2D::origin)
    }

    pub fn iterations(&self) -> usize {
        self.steps.len().saturating_sub(1)
    }

    pub fn path(&self) -> Path2D {
        Path2D::from_points(
            self.steps
                .iter()
                .map(|step| Point2D::new(step.pose.x, step.pose.y))
                .collect(),
        )
    }
}

pub struct MoveToPoseController {
    config: MoveToPoseConfig,
}

impl MoveToPoseController {
    pub fn new(config: MoveToPoseConfig) -> Self {
        Self { config }
    }

    pub fn config(&self) -> MoveToPoseConfig {
        self.config
    }

    pub fn simulate(&self, start: Pose2D, goal: Pose2D) -> MoveToPoseResult {
        let mut pose = start;
        let mut steps = vec![MoveToPoseStep {
            pose,
            control: ControlInput::zero(),
            distance_to_goal: distance_to_goal(pose, goal),
        }];

        for _ in 0..self.config.max_steps {
            let distance = distance_to_goal(pose, goal);
            let yaw_error = normalize_angle(goal.yaw - pose.yaw).abs();
            if distance <= self.config.goal_tolerance && yaw_error <= self.config.yaw_tolerance {
                return MoveToPoseResult {
                    steps,
                    converged: true,
                };
            }

            let control = self.compute_control(pose, goal);
            pose = integrate_pose(pose, control, self.config.dt);
            steps.push(MoveToPoseStep {
                pose,
                control,
                distance_to_goal: distance_to_goal(pose, goal),
            });
        }

        MoveToPoseResult {
            steps,
            converged: false,
        }
    }

    pub fn planning(
        &self,
        start: (f64, f64, f64),
        goal: (f64, f64, f64),
    ) -> Vec<(f64, f64)> {
        self.simulate(
            Pose2D::new(start.0, start.1, start.2),
            Pose2D::new(goal.0, goal.1, goal.2),
        )
        .path()
        .points
        .into_iter()
        .map(|point| (point.x, point.y))
        .collect()
    }

    fn compute_control(&self, pose: Pose2D, goal: Pose2D) -> ControlInput {
        let x_diff = goal.x - pose.x;
        let y_diff = goal.y - pose.y;
        let rho = (x_diff.powi(2) + y_diff.powi(2)).sqrt();
        let alpha = normalize_angle(y_diff.atan2(x_diff) - pose.yaw);
        let beta = normalize_angle(goal.yaw - pose.yaw - alpha);

        let mut v = self.config.kp_rho * rho;
        if !(-PI / 2.0..=PI / 2.0).contains(&alpha) {
            v = -v;
        }

        let omega = self.config.kp_alpha * alpha + self.config.kp_beta * beta;
        ControlInput::new(v, omega)
    }
}

pub fn move_to_pose(start: Pose2D, goal: Pose2D, config: MoveToPoseConfig) -> MoveToPoseResult {
    MoveToPoseController::new(config).simulate(start, goal)
}

pub fn normalize_angle(mut angle: f64) -> f64 {
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

fn integrate_pose(pose: Pose2D, control: ControlInput, dt: f64) -> Pose2D {
    let yaw = normalize_angle(pose.yaw + control.omega * dt);
    Pose2D::new(
        pose.x + control.v * yaw.cos() * dt,
        pose.y + control.v * yaw.sin() * dt,
        yaw,
    )
}

fn distance_to_goal(pose: Pose2D, goal: Pose2D) -> f64 {
    ((goal.x - pose.x).powi(2) + (goal.y - pose.y).powi(2)).sqrt()
}

pub fn demo_move_to_pose() {
    std::fs::create_dir_all("img/path_tracking").unwrap_or_default();

    let start = Pose2D::origin();
    let goal = Pose2D::new(10.0, 10.0, -1.5 * PI);
    let result = move_to_pose(start, goal, MoveToPoseConfig::default());
    let path = result.path();

    println!("Starting Move to Pose...");
    println!(
        "Converged: {}, iterations: {}, final pose: ({:.3}, {:.3}, {:.3})",
        result.converged,
        result.iterations(),
        result.final_pose().x,
        result.final_pose().y,
        result.final_pose().yaw
    );

    let mut fg = Figure::new();
    {
        let axes = fg
            .axes2d()
            .set_title("Move to Pose", &[])
            .set_x_label("x [m]", &[])
            .set_y_label("y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));

        let traj_x = path.x_coords();
        let traj_y = path.y_coords();
        axes.lines(&traj_x, &traj_y, &[Caption("Trajectory"), Color("green")]);
        axes.points(&[start.x], &[start.y], &[Caption("Start"), Color("blue")]);
        axes.points(&[goal.x], &[goal.y], &[Caption("Goal"), Color("red")]);

        let arrow_start_x = vec![start.x, start.x + start.yaw.cos()];
        let arrow_start_y = vec![start.y, start.y + start.yaw.sin()];
        axes.lines(
            &arrow_start_x,
            &arrow_start_y,
            &[Caption("Start Orientation"), Color("blue")],
        );

        let arrow_goal_x = vec![goal.x, goal.x + goal.yaw.cos()];
        let arrow_goal_y = vec![goal.y, goal.y + goal.yaw.sin()];
        axes.lines(
            &arrow_goal_x,
            &arrow_goal_y,
            &[Caption("Goal Orientation"), Color("red")],
        );
    }

    let output_path = "img/path_tracking/move_to_pose.png";
    fg.set_terminal("pngcairo", output_path);
    fg.show().unwrap();
    println!("Move to pose visualization saved to: {}", output_path);
    println!("Move to Pose complete!");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_angle_wraps_to_pi_range() {
        let wrapped = normalize_angle(3.5 * PI);
        assert!((-PI..=PI).contains(&wrapped));
    }

    #[test]
    fn test_move_to_pose_converges() {
        let start = Pose2D::origin();
        let goal = Pose2D::new(5.0, 5.0, -PI / 2.0);

        let result = move_to_pose(start, goal, MoveToPoseConfig::default());
        let final_pose = result.final_pose();

        assert!(result.converged);
        assert!((final_pose.x - goal.x).abs() < 0.05);
        assert!((final_pose.y - goal.y).abs() < 0.05);
        assert!(normalize_angle(final_pose.yaw - goal.yaw).abs() < 0.05);
    }

    #[test]
    fn test_move_to_pose_returns_non_empty_path() {
        let controller = MoveToPoseController::new(MoveToPoseConfig::default());
        let path = controller.planning((0.0, 0.0, 0.0), (2.0, 1.0, 0.0));

        assert!(path.len() > 1);
        assert_eq!(path.first().copied(), Some((0.0, 0.0)));
    }
}
