use rust_robotics::path_tracking::LQRSteerController;

fn main() {
    let waypoints = vec![
        (0.0, 0.0),
        (5.0, 0.0),
        (10.0, 2.0),
        (15.0, 0.0),
        (20.0, -2.0),
        (25.0, 0.0),
    ];

    let mut controller = LQRSteerController::with_defaults();
    match controller.planning(waypoints, 2.0, 0.5) {
        Some(trajectory) => {
            let end = trajectory.last().copied().unwrap_or((0.0, 0.0));
            println!(
                "LQR steer trajectory points={}, end=({:.3}, {:.3})",
                trajectory.len(),
                end.0,
                end.1
            );
        }
        None => {
            eprintln!("LQR steer control failed");
            std::process::exit(1);
        }
    }
}
