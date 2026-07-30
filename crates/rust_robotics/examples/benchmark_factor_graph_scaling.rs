//! Deterministic scaling benchmark for pose graphs and bundle adjustment.

use std::time::Instant;

use nalgebra::{Matrix2, Matrix3, Matrix4, Vector3};
use rust_robotics::core::{se3_exp, Vector6};
use rust_robotics::optimization::{LinearSolver, SolverConfig};
use rust_robotics::slam::bundle_adjustment::{
    bundle_adjust, reproject_world_point, BundleAdjustmentConfig, BundleAdjustmentProblem,
    CameraIntrinsics, ReprojectionObservation,
};
use rust_robotics::slam::pose_graph_optimization::{
    optimize_pose_graph, Edge2D, Pose2DNode, PoseGraphConfig,
};

type BenchmarkResult<T> = Result<T, Box<dyn std::error::Error>>;

fn relative_pose(from: &Pose2DNode, to: &Pose2DNode) -> Pose2DNode {
    let (sin_yaw, cos_yaw) = from.yaw.sin_cos();
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    Pose2DNode::new(
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
        to.yaw - from.yaw,
    )
}

fn benchmark_pose_graph(size: usize, linear_solver: LinearSolver, solver_name: &str) {
    let truth = (0..size)
        .map(|index| {
            let progress = index as f64 * 0.25;
            Pose2DNode::new(
                progress,
                (progress * 0.2).sin(),
                0.2 * (progress * 0.2).cos(),
            )
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
                pose.x + 0.04 * (phase * 0.7).sin(),
                pose.y + 0.05 * (phase * 0.3).cos(),
                pose.yaw + 0.02 * (phase * 0.5).sin(),
            )
        })
        .collect::<Vec<_>>();
    let mut edges = (0..size.saturating_sub(1))
        .map(|from| Edge2D {
            from,
            to: from + 1,
            measurement: relative_pose(&truth[from], &truth[from + 1]),
            information: Matrix3::identity() * 100.0,
        })
        .collect::<Vec<_>>();
    for from in (0..size.saturating_sub(10)).step_by(10) {
        edges.push(Edge2D {
            from,
            to: from + 10,
            measurement: relative_pose(&truth[from], &truth[from + 10]),
            information: Matrix3::identity() * 50.0,
        });
    }
    let parameters = size.saturating_sub(1) * 3;
    let residuals = edges.len() * 3;
    let dense_hessian_bytes = parameters * parameters * size_of::<f64>();
    let active_off_diagonal_blocks = edges
        .iter()
        .filter(|edge| edge.from != 0 && edge.to != 0)
        .count();
    let stored_hessian_bytes =
        (size.saturating_sub(1) + active_off_diagonal_blocks) * 9 * size_of::<f64>();
    let started = Instant::now();
    let result = optimize_pose_graph(
        &initial,
        &edges,
        &PoseGraphConfig {
            max_iterations: 50,
            tolerance: 1.0e-9,
            linear_solver,
        },
    );
    let elapsed = started.elapsed();
    let root_mean_square_error = (result
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
        "pose_graph_{solver_name},{size},{parameters},{residuals},{dense_hessian_bytes},{stored_hessian_bytes},{},{:.3},{root_mean_square_error:.6e}",
        result.iterations,
        elapsed.as_secs_f64() * 1_000.0
    );
}

fn benchmark_bundle_adjustment(
    point_count: usize,
    linear_solver: LinearSolver,
    solver_name: &str,
    use_schur_complement: bool,
) -> BenchmarkResult<()> {
    let intrinsics = CameraIntrinsics {
        fx: 400.0,
        fy: 400.0,
        cx: 320.0,
        cy: 240.0,
    };
    let cameras = vec![
        Matrix4::identity(),
        se3_exp(&Vector6::new(0.8, 0.0, 0.0, 0.0, 0.03, 0.0)),
        se3_exp(&Vector6::new(1.6, 0.1, 0.0, 0.0, -0.02, 0.01)),
    ];
    let truth = (0..point_count)
        .map(|index| {
            let index = index as f64;
            Vector3::new(
                -0.8 + 1.6 * ((index * 0.618_033_988_75).fract()),
                -0.5 + (index * 0.414_213_562_37).fract(),
                4.0 + (index * 0.271_828_182_84).fract() * 2.0,
            )
        })
        .collect::<Vec<_>>();
    let observations = cameras
        .iter()
        .enumerate()
        .flat_map(|(camera, pose)| {
            truth
                .iter()
                .enumerate()
                .map(move |(point, landmark)| -> BenchmarkResult<_> {
                    Ok(ReprojectionObservation {
                        camera,
                        point,
                        pixel: reproject_world_point(pose, landmark, &intrinsics)?,
                        information: Matrix2::identity(),
                    })
                })
        })
        .collect::<BenchmarkResult<Vec<_>>>()?;
    let initial_points = truth
        .iter()
        .enumerate()
        .map(|(index, point)| {
            let phase = index as f64;
            point + Vector3::new(0.08 * (phase * 0.7).sin(), 0.08 * (phase * 0.4).cos(), 0.12)
        })
        .collect();
    let parameters = point_count * 3;
    let residuals = observations.len() * 2;
    let dense_hessian_bytes = parameters * parameters * size_of::<f64>();
    let problem = BundleAdjustmentProblem {
        cameras,
        points: initial_points,
        observations,
        intrinsics,
    };
    let started = Instant::now();
    let result = bundle_adjust(
        &problem,
        &BundleAdjustmentConfig {
            fixed_cameras: 3,
            use_schur_complement,
            solver: SolverConfig {
                linear_solver,
                ..SolverConfig::default()
            },
            ..BundleAdjustmentConfig::default()
        },
    )?;
    let elapsed = started.elapsed();
    let stored_hessian_bytes = result.summary.hessian_scalar_entries * size_of::<f64>();
    let root_mean_square_error = (result
        .points
        .iter()
        .zip(&truth)
        .map(|(actual, expected)| (actual - expected).norm_squared())
        .sum::<f64>()
        / point_count as f64)
        .sqrt();
    println!(
        "bundle_adjustment_{solver_name},{point_count},{parameters},{residuals},{dense_hessian_bytes},{stored_hessian_bytes},{},{:.3},{root_mean_square_error:.6e}",
        result.summary.iterations,
        elapsed.as_secs_f64() * 1_000.0
    );
    Ok(())
}

fn main() -> BenchmarkResult<()> {
    println!(
        "algorithm,size,parameters,residuals,dense_hessian_bytes,stored_hessian_bytes,iterations,elapsed_ms,rmse"
    );
    for size in [10, 50, 100, 200] {
        benchmark_pose_graph(size, LinearSolver::Dense, "dense");
        benchmark_pose_graph(
            size,
            LinearSolver::BlockSparsePcg {
                max_iterations: 1_000,
                tolerance: 1.0e-10,
            },
            "block_sparse_pcg",
        );
    }
    for point_count in [10, 50, 100, 200] {
        benchmark_bundle_adjustment(point_count, LinearSolver::Dense, "dense", false)?;
        benchmark_bundle_adjustment(
            point_count,
            LinearSolver::BlockSparsePcg {
                max_iterations: 1_000,
                tolerance: 1.0e-10,
            },
            "block_sparse_pcg",
            false,
        )?;
        benchmark_bundle_adjustment(point_count, LinearSolver::Dense, "schur_complement", true)?;
    }
    Ok(())
}
