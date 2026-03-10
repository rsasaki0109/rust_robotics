//! Headless localization example using EKF, UKF, and Particle Filter.

use rust_robotics::prelude::*;

fn propagate_state(state: &mut State2D, control: ControlInput, dt: f64) {
    state.x += control.v * state.yaw.cos() * dt;
    state.y += control.v * state.yaw.sin() * dt;
    state.yaw += control.omega * dt;
    state.v = control.v;
}

fn build_pf_measurements(state: &State2D, landmarks: &Obstacles) -> PFMeasurement {
    landmarks
        .points
        .iter()
        .map(|landmark| {
            let dx = state.x - landmark.x;
            let dy = state.y - landmark.y;
            (((dx * dx + dy * dy).sqrt()), landmark.x, landmark.y)
        })
        .collect()
}

fn main() -> RoboticsResult<()> {
    let landmarks = Obstacles::from_points(vec![
        Point2D::new(5.0, 0.0),
        Point2D::new(0.0, 5.0),
        Point2D::new(5.0, 5.0),
    ]);
    let mut true_state = State2D::origin();
    let control = ControlInput::new(1.0, 0.1);

    let mut ekf = EKFLocalizer::with_initial_state_2d(State2D::origin(), EKFConfig::default())?;
    let mut ukf = UKFLocalizer::with_initial_state_2d(State2D::origin(), UKFConfig::default())?;
    let mut pf = ParticleFilterLocalizer::with_initial_state_2d(
        State2D::origin(),
        ParticleFilterConfig::default(),
    )?;
    pf.set_landmarks_from_obstacles(&landmarks)?;

    for step in 0..40 {
        propagate_state(&mut true_state, control, 0.1);

        let position_measurement = Point2D::new(
            true_state.x + ((step % 5) as f64 - 2.0) * 0.03,
            true_state.y + (((step * 2) % 5) as f64 - 2.0) * 0.03,
        );
        let pf_measurement = build_pf_measurements(&true_state, &landmarks);

        let ekf_state = ekf.estimate_state(position_measurement, control, 0.1)?;
        let ukf_state = ukf.estimate_state(control, position_measurement)?;
        let pf_state = pf.try_step_state(control, &pf_measurement)?;

        if step % 10 == 0 {
            println!(
                "step={step:02} true=({:.2}, {:.2}) ekf=({:.2}, {:.2}) ukf=({:.2}, {:.2}) pf=({:.2}, {:.2})",
                true_state.x,
                true_state.y,
                ekf_state.x,
                ekf_state.y,
                ukf_state.x,
                ukf_state.y,
                pf_state.x,
                pf_state.y
            );
        }
    }

    println!(
        "final true=({:.2}, {:.2}) ekf=({:.2}, {:.2}) ukf=({:.2}, {:.2}) pf=({:.2}, {:.2})",
        true_state.x,
        true_state.y,
        ekf.state_2d().x,
        ekf.state_2d().y,
        ukf.state_2d().x,
        ukf.state_2d().y,
        pf.state_2d().x,
        pf.state_2d().y
    );

    Ok(())
}
