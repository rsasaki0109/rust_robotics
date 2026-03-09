use rust_robotics::path_tracking::PurePursuitController;

fn main() {
    let waypoints: Vec<(f64, f64)> = (0..20)
        .map(|i| (i as f64, (i as f64 / 3.0).sin() * 2.0))
        .collect();

    let mut controller = PurePursuitController::with_params(0.1, 2.0, 2.9);
    match controller.planning(waypoints, 5.0) {
        Some(trajectory) => {
            let end = trajectory.last().copied().unwrap_or((0.0, 0.0));
            println!(
                "Pure pursuit trajectory points={}, end=({:.3}, {:.3})",
                trajectory.len(),
                end.0,
                end.1
            );
        }
        None => {
            eprintln!("Pure pursuit failed");
            std::process::exit(1);
        }
    }
}
