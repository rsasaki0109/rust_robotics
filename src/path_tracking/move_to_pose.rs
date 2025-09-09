// 
// Move to specified pose
// Author: Daniel Ingram (daniel-s-ingram)
//         Atsushi Sakai(@Atsushi_twi)
//         Ryohei Sasaki(@rsasaki0109) 
// P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7
use gnuplot::{Figure, Caption, Color, AxesCommon};
use std::f64::consts::PI;

fn move_to_pose(
    x_start: (f64, f64, f64), x_goal: (f64, f64, f64), kp: (f64, f64, f64), dt: f64)
-> Vec<(f64, f64)>
{
    let mut x = x_start;
    let mut x_diff = (x_goal.0 - x.0, x_goal.1 - x.1);
    let mut rho = ((x_diff.0).powi(2) + (x_diff.1).powi(2)).sqrt();
    let mut x_traj = vec![(x.0, x.1)];
    while rho > 0.001 {
        x_diff = (x_goal.0 - x.0, x_goal.1 - x.1);
        rho = ((x_diff.0).powi(2) + (x_diff.1).powi(2)).sqrt();
        let alpha = normalize_angle((x_diff.1).atan2(x_diff.0) - x.2);
        let beta = normalize_angle(x_goal.2 - x.2 - alpha);
        let mut v = kp.0 * rho;
        let w = kp.1 * alpha + kp.2 * beta;
        if alpha > PI / 2.0 || alpha < -PI / 2.0 {
            v = -v;
        }
        x.2 = x.2 + w * dt;
        x.0 = x.0 + v * (x.2).cos() * dt;
        x.1 = x.1 + v * (x.2).sin() * dt;
        x_traj.push((x.0, x.1));
    }
    x_traj
}

fn normalize_angle(mut angle: f64) -> f64 {
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

fn main() {
    // Create output directory
    std::fs::create_dir_all("img/path_tracking").unwrap();

    let x_start = (0.0, 0.0, 0.0);
    let x_goal = (10.0, 10.0, -1.5 * PI);
    // [kp_rho, kp_alpha, kp_beta]
    let kp = (9. , 15., -3.);
    let dt = 0.01;
    let x_traj = move_to_pose(x_start, x_goal, kp, dt);

    println!("Starting Move to Pose...");
    
    let mut fg = Figure::new();
    {
        let axes = fg.axes2d()
            .set_title("Move to Pose", &[])
            .set_x_label("x [m]", &[])
            .set_y_label("y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0));

        // Plot trajectory
        let traj_x: Vec<f64> = x_traj.iter().map(|p| p.0).collect();
        let traj_y: Vec<f64> = x_traj.iter().map(|p| p.1).collect();
        axes.lines(&traj_x, &traj_y, &[Caption("Trajectory"), Color("green")]);

        // Plot start and goal positions
        axes.points(&[x_start.0], &[x_start.1], &[Caption("Start"), Color("blue")]);
        axes.points(&[x_goal.0], &[x_goal.1], &[Caption("Goal"), Color("red")]);
        
        // Plot orientation arrows
        let arrow_start_x = vec![x_start.0, x_start.0 + x_start.2.cos()];
        let arrow_start_y = vec![x_start.1, x_start.1 + x_start.2.sin()];
        axes.lines(&arrow_start_x, &arrow_start_y, &[Caption("Start Orientation"), Color("blue")]);
        
        let arrow_goal_x = vec![x_goal.0, x_goal.0 + x_goal.2.cos()];
        let arrow_goal_y = vec![x_goal.1, x_goal.1 + x_goal.2.sin()];
        axes.lines(&arrow_goal_x, &arrow_goal_y, &[Caption("Goal Orientation"), Color("red")]);
    }

    let output_path = "img/path_tracking/move_to_pose.png";
    fg.set_terminal("pngcairo", output_path);
    fg.show().unwrap();
    println!("Move to pose visualization saved to: {}", output_path);
    println!("Move to Pose complete!");

}