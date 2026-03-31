//! Integration tests for Model Predictive Trajectory Generator.

#[path = "../src/model_predictive_trajectory_generator.rs"]
mod model_predictive_trajectory_generator;

use model_predictive_trajectory_generator::*;
use std::f64::consts::PI;

#[test]
fn test_optimize_straight_ahead() {
    let cfg = MptgConfig::default();
    let target = TargetState::new(10.0, 0.0, 0.0);
    let init_p = nalgebra::Vector3::new(10.0, 0.0, 0.0);

    let result = optimize_trajectory(&target, 0.0, init_p, &cfg);
    assert!(result.is_some(), "Straight-ahead should converge");
    let res = result.unwrap();
    assert!(res.x.len() > 2);
}

#[test]
fn test_optimize_turn_right() {
    let cfg = MptgConfig::default();
    let target = TargetState::new(5.0, -3.0, -PI / 4.0);
    let init_p = nalgebra::Vector3::new(7.0, 0.0, 0.0);

    let result = optimize_trajectory(&target, 0.0, init_p, &cfg);
    assert!(result.is_some(), "Right turn should converge");
}

#[test]
fn test_optimize_turn_left_90deg() {
    let cfg = MptgConfig::default();
    let target = TargetState::new(5.0, 2.0, PI / 2.0);
    let init_p = nalgebra::Vector3::new(6.0, 0.0, 0.0);

    let result = optimize_trajectory(&target, 0.0, init_p, &cfg);
    assert!(result.is_some(), "Left 90-degree turn should converge");

    let res = result.unwrap();
    let err_x = (res.x.last().unwrap() - target.x).abs();
    let err_y = (res.y.last().unwrap() - target.y).abs();
    assert!(err_x < cfg.cost_th, "x error too large: {err_x}");
    assert!(err_y < cfg.cost_th, "y error too large: {err_y}");
}

#[test]
fn test_lookup_table_basic() {
    let cfg = MptgConfig {
        max_iter: 100,
        cost_th: 0.3,
        ..Default::default()
    };

    let table = generate_lookup_table(&[10.0, 15.0], &[0.0, 5.0], &[0.0], 0.0, &cfg);

    assert!(table.len() > 1, "Table should have entries beyond the seed");
}
