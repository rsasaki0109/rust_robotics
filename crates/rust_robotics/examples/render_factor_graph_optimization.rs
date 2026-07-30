//! Render factor-graph optimization as pure-Rust SVG and animated GIF assets.

use std::fmt::Write;

use nalgebra::Matrix3;
use rust_robotics::core::Pose2D;
use rust_robotics::optimization::LinearSolver;
use rust_robotics::slam::pose_graph_optimization::{
    optimize_pose_graph, Edge2D, Pose2DNode, PoseGraphConfig,
};
use rust_robotics::viz::{GifCanvasConfig, GifFrame, GifRecorder};

const SVG_OUTPUT: &str = "docs/assets/factor-graph-optimization.svg";
const GIF_OUTPUT: &str = "media/gallery/factor_graph_optimization.gif";

fn relative(from: &Pose2DNode, to: &Pose2DNode) -> Pose2DNode {
    let (sin, cos) = from.yaw.sin_cos();
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    Pose2DNode::new(cos * dx + sin * dy, -sin * dx + cos * dy, to.yaw - from.yaw)
}

fn graph() -> (Vec<Pose2DNode>, Vec<Pose2DNode>, Vec<Edge2D>) {
    let truth = (0..24)
        .map(|index| {
            let angle = index as f64 * std::f64::consts::TAU / 24.0;
            Pose2DNode::new(
                5.0 * angle.cos(),
                3.2 * angle.sin(),
                angle + std::f64::consts::FRAC_PI_2,
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
            let drift = index as f64 / 23.0;
            Pose2DNode::new(
                pose.x + 1.4 * drift + 0.18 * (index as f64 * 0.8).sin(),
                pose.y - 0.9 * drift + 0.14 * (index as f64 * 0.4).cos(),
                pose.yaw + 0.3 * drift,
            )
        })
        .collect::<Vec<_>>();
    let mut edges = (0..truth.len() - 1)
        .map(|from| Edge2D {
            from,
            to: from + 1,
            measurement: relative(&truth[from], &truth[from + 1]),
            information: Matrix3::identity() * 100.0,
        })
        .collect::<Vec<_>>();
    edges.push(Edge2D {
        from: truth.len() - 1,
        to: 0,
        measurement: relative(&truth[truth.len() - 1], &truth[0]),
        information: Matrix3::identity() * 100.0,
    });
    for from in [0, 6, 12] {
        edges.push(Edge2D {
            from,
            to: from + 6,
            measurement: relative(&truth[from], &truth[from + 6]),
            information: Matrix3::identity() * 40.0,
        });
    }
    (truth, initial, edges)
}

fn polyline(poses: &[Pose2DNode], x_offset: f64) -> String {
    poses
        .iter()
        .map(|pose| {
            let x = x_offset + 48.0 * (pose.x + 6.5);
            let y = 255.0 - 48.0 * pose.y;
            format!("{x:.1},{y:.1}")
        })
        .collect::<Vec<_>>()
        .join(" ")
}

fn render_svg(
    initial: &[Pose2DNode],
    optimized: &[Pose2DNode],
    edges: &[Edge2D],
    iterations: usize,
) -> String {
    let initial_points = polyline(initial, 10.0);
    let optimized_points = polyline(optimized, 670.0);
    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="1320" height="560" viewBox="0 0 1320 560" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<title id="title">Block-sparse pose graph optimization</title>"##
    )
    .unwrap();
    writeln!(svg, r##"<desc id="desc">A drifted pose graph before and after loop-closure optimization.</desc><rect width="1320" height="560" fill="#f8fafc"/>"##).unwrap();
    writeln!(svg, r##"<text x="56" y="48" font-family="sans-serif" font-size="28" font-weight="700" fill="#0f172a">Factor graph: drift → globally consistent trajectory</text>"##).unwrap();
    writeln!(svg, r##"<text x="56" y="77" font-family="sans-serif" font-size="15" fill="#475569">SE(2) loop closures · block-sparse Hessian · PCG · {iterations} nonlinear iterations</text>"##).unwrap();
    for (x, label, points, color) in [
        (50, "Noisy odometry", initial_points.as_str(), "#ef4444"),
        (710, "Optimized graph", optimized_points.as_str(), "#0f9f75"),
    ] {
        writeln!(svg, r##"<rect x="{x}" y="105" width="560" height="350" rx="18" fill="#ffffff" stroke="#cbd5e1"/><text x="{}" y="140" font-family="sans-serif" font-size="19" font-weight="700" fill="#0f172a">{label}</text><polyline points="{points}" fill="none" stroke="{color}" stroke-width="4" stroke-linejoin="round"/>"##, x + 24).unwrap();
    }
    for (poses, x_offset, color) in [(initial, 10.0, "#ef4444"), (optimized, 670.0, "#0f9f75")] {
        for edge in edges {
            if edge.to != edge.from + 1 && !(edge.from + 1 == poses.len() && edge.to == 0) {
                let a = &poses[edge.from];
                let b = &poses[edge.to];
                writeln!(svg, r##"<line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="#8b5cf6" stroke-width="2" stroke-dasharray="7 6" opacity=".75"/>"##, x_offset + 48.0 * (a.x + 6.5), 255.0 - 48.0 * a.y, x_offset + 48.0 * (b.x + 6.5), 255.0 - 48.0 * b.y).unwrap();
            }
        }
        for pose in poses {
            writeln!(svg, r##"<circle cx="{:.1}" cy="{:.1}" r="5" fill="{color}" stroke="#fff" stroke-width="2"/>"##, x_offset + 48.0 * (pose.x + 6.5), 255.0 - 48.0 * pose.y).unwrap();
        }
    }
    writeln!(svg, r##"<circle cx="101" cy="255" r="9" fill="#2563eb"/><circle cx="761" cy="255" r="9" fill="#2563eb"/><text x="56" y="505" font-family="sans-serif" font-size="14" fill="#475569">Blue: fixed gauge anchor · Purple dashed: loop closures · Generated entirely by RustRobotics</text></svg>"##).unwrap();
    svg
}

fn render_gif(initial: &[Pose2DNode], optimized: &[Pose2DNode], edges: &[Edge2D]) {
    let config = GifCanvasConfig::new(560, 390, (-7.0, 8.0), (-5.0, 5.0))
        .with_delay_cs(7)
        .with_grid_step(Some(2.0));
    let mut recorder = GifRecorder::new(GIF_OUTPUT, config.clone()).expect("create GIF");
    for frame_index in 0..36 {
        let progress = frame_index as f64 / 35.0;
        let smooth = progress * progress * (3.0 - 2.0 * progress);
        let poses = initial
            .iter()
            .zip(optimized)
            .map(|(from, to)| {
                Pose2DNode::new(
                    from.x + smooth * (to.x - from.x),
                    from.y + smooth * (to.y - from.y),
                    from.yaw + smooth * (to.yaw - from.yaw),
                )
            })
            .collect::<Vec<_>>();
        let mut frame = GifFrame::new(&config);
        for edge in edges {
            let a = &poses[edge.from];
            let b = &poses[edge.to];
            let loop_closure =
                edge.to != edge.from + 1 && !(edge.from + 1 == poses.len() && edge.to == 0);
            frame.draw_segment(
                (a.x, a.y),
                (b.x, b.y),
                if loop_closure {
                    (139, 92, 246)
                } else {
                    (148, 163, 184)
                },
                if loop_closure { 1.8 } else { 1.0 },
            );
        }
        for pose in &poses {
            frame.draw_robot(&Pose2D::new(pose.x, pose.y, pose.yaw), 0.12, (15, 159, 117));
        }
        frame.fill_circle(poses[0].x, poses[0].y, 0.14, (37, 99, 235));
        recorder
            .add_frame_with_delay(frame, if frame_index == 35 { 150 } else { 7 })
            .expect("write GIF frame");
    }
    recorder.finish().expect("finish GIF");
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (_truth, initial, edges) = graph();
    let result = optimize_pose_graph(
        &initial,
        &edges,
        &PoseGraphConfig {
            max_iterations: 50,
            tolerance: 1.0e-10,
            linear_solver: LinearSolver::BlockSparsePcg {
                max_iterations: 500,
                tolerance: 1.0e-11,
            },
        },
    );
    std::fs::create_dir_all("docs/assets")?;
    std::fs::create_dir_all("media/gallery")?;
    std::fs::write(
        SVG_OUTPUT,
        render_svg(&initial, &result.poses, &edges, result.iterations),
    )?;
    render_gif(&initial, &result.poses, &edges);
    println!("wrote {SVG_OUTPUT} and {GIF_OUTPUT}");
    Ok(())
}
