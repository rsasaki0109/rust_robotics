use rust_robotics::localization::{PFControl, PFMeasurement, PFState, ParticleFilterConfig, ParticleFilterLocalizer};

fn main() {
    let initial = PFState::new(0.0, 0.0, 0.0, 0.0);
    let mut pf = ParticleFilterLocalizer::with_initial_state(initial, ParticleFilterConfig::default());
    let control = PFControl::new(1.0, 0.05);
    let observations: PFMeasurement = vec![
        (7.0, 5.0, 0.0),
        (6.0, 0.0, 5.0),
    ];

    for _ in 0..20 {
        let _ = pf.step(&control, &observations);
    }

    let estimate = pf.estimate();
    println!(
        "Particle filter estimate: x={:.3}, y={:.3}, yaw={:.3}, v={:.3}",
        estimate[0], estimate[1], estimate[2], estimate[3]
    );
}
