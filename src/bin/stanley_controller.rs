use rust_robotics::path_tracking::{StanleyConfig, StanleyController};

fn main() {
    let waypoints: Vec<(f64, f64)> = (0..20)
        .map(|i| (i as f64, (i as f64 / 4.0).cos() * 2.0))
        .collect();

    let mut controller = StanleyController::new(StanleyConfig::default());
    match controller.planning(waypoints, 5.0, 0.5) {
        Some(trajectory) => {
            let end = trajectory.last().copied().unwrap_or((0.0, 0.0));
            println!(
                "Stanley trajectory points={}, end=({:.3}, {:.3})",
                trajectory.len(),
                end.0,
                end.1
            );
        }
        None => {
            eprintln!("Stanley controller failed");
            std::process::exit(1);
        }
    }
}
