//! Block-sparse pose-graph benchmark at 1k, 5k, and 10k poses.

use std::time::Instant;

use nalgebra::Matrix3;
use rust_robotics::optimization::LinearSolver;
use rust_robotics::slam::pose_graph_optimization::{
    optimize_pose_graph, Edge2D, Pose2DNode, PoseGraphConfig,
};

fn relative(from: &Pose2DNode, to: &Pose2DNode) -> Pose2DNode {
    let (sin, cos) = from.yaw.sin_cos();
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    Pose2DNode::new(cos * dx + sin * dy, -sin * dx + cos * dy, to.yaw - from.yaw)
}

fn benchmark(size: usize) {
    let truth = (0..size)
        .map(|index| {
            let x = index as f64 * 0.05;
            Pose2DNode::new(x, 2.0 * (x * 0.015).sin(), 0.03 * (x * 0.015).cos())
        })
        .collect::<Vec<_>>();
    let initial = truth
        .iter()
        .enumerate()
        .map(|(index, pose)| {
            if index == 0 {
                return *pose;
            }
            let phase = index as f64;
            Pose2DNode::new(
                pose.x + 0.02 * (phase * 0.013).sin(),
                pose.y + 0.03 * (phase * 0.021).cos(),
                pose.yaw + 0.005 * (phase * 0.017).sin(),
            )
        })
        .collect::<Vec<_>>();
    let mut edges = (0..size - 1)
        .map(|from| Edge2D {
            from,
            to: from + 1,
            measurement: relative(&truth[from], &truth[from + 1]),
            information: Matrix3::identity() * 100.0,
        })
        .collect::<Vec<_>>();
    for from in (0..size.saturating_sub(100)).step_by(100) {
        edges.push(Edge2D {
            from,
            to: from + 100,
            measurement: relative(&truth[from], &truth[from + 100]),
            information: Matrix3::identity() * 20.0,
        });
    }

    let parameters = (size - 1) * 3;
    let dense_bytes = parameters * parameters * size_of::<f64>();
    let active_cross_blocks = edges
        .iter()
        .filter(|edge| edge.from != 0 && edge.to != 0)
        .count();
    let sparse_bytes = ((size - 1) + active_cross_blocks) * 9 * size_of::<f64>();
    let started = Instant::now();
    let result = optimize_pose_graph(
        &initial,
        &edges,
        &PoseGraphConfig {
            max_iterations: 25,
            tolerance: 1.0e-8,
            linear_solver: LinearSolver::BlockSparsePcg {
                max_iterations: 3_000,
                tolerance: 1.0e-5,
            },
        },
    );
    let elapsed = started.elapsed().as_secs_f64();
    let rmse = (result
        .poses
        .iter()
        .zip(&truth)
        .map(|(actual, expected)| {
            (actual.x - expected.x).powi(2)
                + (actual.y - expected.y).powi(2)
                + (actual.yaw - expected.yaw).powi(2)
        })
        .sum::<f64>()
        / size as f64)
        .sqrt();
    println!(
        "{size},{parameters},{},{dense_bytes},{sparse_bytes},{},{},{elapsed:.3},{rmse:.6e}",
        edges.len(),
        result.iterations,
        result.converged
    );
    assert!(result.converged, "{size}-pose graph did not converge");
    assert!(rmse < 5.0e-3, "{size}-pose graph RMSE is {rmse}");
}

fn main() {
    println!(
        "poses,parameters,edges,dense_hessian_bytes,stored_block_bytes,iterations,converged,elapsed_seconds,rmse"
    );
    for size in [1_000, 5_000, 10_000] {
        benchmark(size);
    }
}
