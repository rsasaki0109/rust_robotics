//! Integration tests for LQR-RRT* planner.
//!
//! NOTE: The full unit tests are inside `src/lqr_rrt_star.rs`.
//! To run them, add `pub mod lqr_rrt_star;` to `lib.rs`.
//!
//! This integration test exercises the LQR planner dependency to
//! verify the core building block works correctly.

use rust_robotics_planning::lqr_planner::{LqrPlanner, LqrPlannerConfig};

/// Verify LQR planner produces a valid trajectory (used by LQR-RRT* for steering).
#[test]
fn test_lqr_steering_trajectory() {
    let planner = LqrPlanner::new(LqrPlannerConfig::default());
    let (rx, ry) = planner.planning(0.0, 0.0, 5.0, 5.0).unwrap();

    assert!(rx.len() > 2);
    assert_eq!(rx.len(), ry.len());

    let last_x = *rx.last().unwrap();
    let last_y = *ry.last().unwrap();
    let d = ((last_x - 5.0).powi(2) + (last_y - 5.0).powi(2)).sqrt();
    assert!(d <= 0.1, "LQR planner should reach the goal, d={d}");
}

/// Verify LQR path resampling logic (mirrors sample_path in lqr_rrt_star).
#[test]
fn test_lqr_path_resampling() {
    let planner = LqrPlanner::new(LqrPlannerConfig::default());
    let (wx, wy) = planner.planning(0.0, 0.0, 3.0, 3.0).unwrap();

    // Resample at step_size = 0.2 (same as LQR-RRT* default)
    let step = 0.2;
    let mut px = Vec::new();
    let mut py = Vec::new();

    for i in 0..wx.len().saturating_sub(1) {
        let mut t = 0.0;
        while t < 1.0 {
            px.push(t * wx[i + 1] + (1.0 - t) * wx[i]);
            py.push(t * wy[i + 1] + (1.0 - t) * wy[i]);
            t += step;
        }
    }

    assert!(!px.is_empty());
    assert_eq!(px.len(), py.len());

    // Compute segment lengths
    let clen: Vec<f64> = px
        .windows(2)
        .zip(py.windows(2))
        .map(|(xw, yw)| {
            let dx = xw[1] - xw[0];
            let dy = yw[1] - yw[0];
            (dx * dx + dy * dy).sqrt()
        })
        .collect();

    assert!(!clen.is_empty());
    let total_len: f64 = clen.iter().sum();
    assert!(total_len > 0.0, "Total path length should be positive");
}
