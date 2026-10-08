//! Tier 1 contract: every high-traffic path tracker is usable as a
//! `Box<dyn PathTracker>`, returns a unicycle command `(v, ω)`, converges onto
//! the reference path, and follows a replaced path (see `docs/api_traits.md`).

use rust_robotics_control::{
    LQRSteerConfig, LQRSteerController, PurePursuitConfig, PurePursuitController,
    RearWheelFeedbackConfig, RearWheelFeedbackController, StanleyConfig, StanleyController,
};
use rust_robotics_core::{ControlInput, Path2D, PathTracker, Point2D, State2D};

const DT: f64 = 0.1;

fn trackers() -> Vec<(&'static str, Box<dyn PathTracker>)> {
    vec![
        (
            "Pure Pursuit",
            Box::new(PurePursuitController::new(PurePursuitConfig::default())),
        ),
        (
            "Stanley",
            Box::new(StanleyController::new(StanleyConfig::default())),
        ),
        (
            "LQR Steer",
            Box::new(LQRSteerController::new(LQRSteerConfig::default())),
        ),
        (
            "Rear Wheel Feedback",
            Box::new(RearWheelFeedbackController::new(
                RearWheelFeedbackConfig::default(),
            )),
        ),
    ]
}

/// A straight line along +x at height `y`, 0.5 m between points.
fn line(y: f64) -> Path2D {
    Path2D::from_points(
        (0..=240)
            .map(|i| Point2D::new(f64::from(i) * 0.5, y))
            .collect(),
    )
}

/// Integrates the unicycle command every tracker returns.
fn step(state: &mut State2D, control: ControlInput) {
    state.v = control.v;
    state.yaw += control.omega * DT;
    state.x += control.v * state.yaw.cos() * DT;
    state.y += control.v * state.yaw.sin() * DT;
}

fn drive(tracker: &mut dyn PathTracker, state: &mut State2D, path: &Path2D, steps: usize) {
    for _ in 0..steps {
        let control = tracker.compute_control(state, path);
        assert!(control.v.is_finite() && control.omega.is_finite());
        step(state, control);
    }
}

#[test]
fn every_tier1_tracker_converges_onto_the_path() {
    for (name, mut tracker) in trackers() {
        let mut state = State2D::new(0.0, 1.0, 0.0, 0.0);
        drive(tracker.as_mut(), &mut state, &line(0.0), 80);
        assert!(state.x > 10.0, "{name}: only reached x = {:.2}", state.x);
        assert!(
            state.y.abs() < 0.2,
            "{name}: cross-track error {:.3} m",
            state.y
        );
    }
}

#[test]
fn every_tier1_tracker_follows_a_replaced_path_of_equal_length() {
    let (first, second) = (line(0.0), line(2.0));
    assert_eq!(first.len(), second.len());
    for (name, mut tracker) in trackers() {
        let mut state = State2D::new(0.0, 0.0, 0.0, 0.0);
        drive(tracker.as_mut(), &mut state, &first, 30);
        drive(tracker.as_mut(), &mut state, &second, 80);
        assert!(
            (state.y - 2.0).abs() < 0.2,
            "{name}: still at y = {:.3} after the path moved to y = 2",
            state.y
        );
    }
}

#[test]
fn goal_check_uses_the_tracker_threshold() {
    for (name, tracker) in trackers() {
        let goal = Point2D::new(10.0, 0.0);
        assert!(
            tracker.is_goal_reached(&State2D::new(10.0, 0.0, 0.0, 0.0), goal),
            "{name}"
        );
        assert!(
            !tracker.is_goal_reached(&State2D::new(0.0, 0.0, 0.0, 0.0), goal),
            "{name}"
        );
    }
}
