//! Render BranchOut-lite multimodal driving as a static SVG.

use std::fmt::Write;
use std::path::Path;

use rust_robotics::planning::{
    BranchOutDecisionMode2D, BranchOutDrivingScene2D, BranchOutPlanner2D, BranchOutPlannerConfig2D,
    BranchOutPose2D, BranchOutTrajectory2D,
};
use rust_robotics::prelude::*;

const OUTPUT: &str = "docs/assets/branchout-multimodal-driving.svg";
const LEFT: f64 = 58.0;
const TOP: f64 = 92.0;
const PLOT_WIDTH: f64 = 690.0;
const PLOT_HEIGHT: f64 = 330.0;
const X_MIN: f64 = -0.4;
const X_MAX: f64 = 7.8;
const Y_MIN: f64 = -2.15;
const Y_MAX: f64 = 2.15;

#[derive(Debug, Clone)]
struct BranchOutScene {
    driving_scene: BranchOutDrivingScene2D,
    branchout: Vec<BranchOutTrajectory2D>,
    unimodal: Vec<BranchOutTrajectory2D>,
    branchout_metrics: (f64, f64, f64),
    unimodal_metrics: (f64, f64, f64),
}

fn planner_with_modes(modes: Vec<BranchOutDecisionMode2D>) -> RoboticsResult<BranchOutPlanner2D> {
    BranchOutPlanner2D::new(BranchOutPlannerConfig2D {
        modes,
        ..BranchOutPlannerConfig2D::default()
    })
}

fn multimodal_ground_truths(plan: &[BranchOutTrajectory2D]) -> Vec<Vec<BranchOutPose2D>> {
    plan.iter()
        .filter(|trajectory| trajectory.mode != BranchOutDecisionMode2D::KeepLane)
        .map(|trajectory| trajectory.poses.clone())
        .collect()
}

fn build_scene() -> RoboticsResult<BranchOutScene> {
    let driving_scene = BranchOutDrivingScene2D::simple_overtake();
    let branchout = planner_with_modes(vec![
        BranchOutDecisionMode2D::KeepLane,
        BranchOutDecisionMode2D::Yield,
        BranchOutDecisionMode2D::LaneChangeLeft,
        BranchOutDecisionMode2D::LaneChangeRight,
    ])?;
    let unimodal = planner_with_modes(vec![BranchOutDecisionMode2D::KeepLane])?;
    let branchout_plan = branchout.plan(&driving_scene)?;
    let unimodal_plan = unimodal.plan(&driving_scene)?;
    let ground_truths = multimodal_ground_truths(&branchout_plan.trajectories);
    let branchout_metrics = branchout.evaluate_multimodal(&branchout_plan, &ground_truths)?;
    let unimodal_metrics = unimodal.evaluate_multimodal(&unimodal_plan, &ground_truths)?;

    Ok(BranchOutScene {
        driving_scene,
        branchout: branchout_plan.trajectories,
        unimodal: unimodal_plan.trajectories,
        branchout_metrics: (
            branchout_metrics.mean_pairwise_final_distance,
            branchout_metrics.min_ground_truth_frechet,
            branchout_metrics.negative_log_likelihood,
        ),
        unimodal_metrics: (
            unimodal_metrics.mean_pairwise_final_distance,
            unimodal_metrics.min_ground_truth_frechet,
            unimodal_metrics.negative_log_likelihood,
        ),
    })
}

fn sx(x: f64) -> f64 {
    LEFT + (x - X_MIN) / (X_MAX - X_MIN) * PLOT_WIDTH
}

fn sy(y: f64) -> f64 {
    TOP + (Y_MAX - y) / (Y_MAX - Y_MIN) * PLOT_HEIGHT
}

fn sdx(distance: f64) -> f64 {
    distance / (X_MAX - X_MIN) * PLOT_WIDTH
}

fn sdy(distance: f64) -> f64 {
    distance / (Y_MAX - Y_MIN) * PLOT_HEIGHT
}

fn color_for_mode(mode: BranchOutDecisionMode2D) -> &'static str {
    match mode {
        BranchOutDecisionMode2D::KeepLane => "#2563eb",
        BranchOutDecisionMode2D::Yield => "#f59e0b",
        BranchOutDecisionMode2D::LaneChangeLeft => "#059669",
        BranchOutDecisionMode2D::LaneChangeRight => "#dc2626",
    }
}

fn trajectory_points(poses: &[BranchOutPose2D]) -> String {
    poses
        .iter()
        .map(|pose| format!("{:.1},{:.1}", sx(pose.x), sy(pose.y)))
        .collect::<Vec<_>>()
        .join(" ")
}

fn render_road(svg: &mut String, scene: &BranchOutDrivingScene2D) {
    let road_top = sy(scene.lane_center(scene.lane_count_each_side) + scene.lane_width / 2.0);
    let road_bottom = sy(scene.lane_center(-scene.lane_count_each_side) - scene.lane_width / 2.0);
    writeln!(
        svg,
        r##"<rect x="{LEFT:.1}" y="{road_top:.1}" width="{PLOT_WIDTH:.1}" height="{:.1}" fill="#e5e7eb"/>"##,
        road_bottom - road_top
    )
    .unwrap();
    for lane in -scene.lane_count_each_side..=scene.lane_count_each_side {
        let y = sy(scene.lane_center(lane));
        writeln!(
            svg,
            r##"<line x1="{LEFT:.1}" y1="{y:.1}" x2="{:.1}" y2="{y:.1}" stroke="#ffffff" stroke-width="2.4" stroke-dasharray="18 14" opacity="0.95"/>"##,
            LEFT + PLOT_WIDTH
        )
        .unwrap();
    }
}

fn render_trajectory(svg: &mut String, trajectory: &BranchOutTrajectory2D, primary: bool) {
    let points = trajectory_points(&trajectory.poses);
    let color = color_for_mode(trajectory.mode);
    let width = if primary { 5.2 } else { 3.8 };
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="#ffffff" stroke-width="{:.1}" stroke-linecap="round" stroke-linejoin="round" opacity="0.82"/>"##,
        width + 5.0
    )
    .unwrap();
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="{color}" stroke-width="{width:.1}" stroke-linecap="round" stroke-linejoin="round" opacity="{:.2}"/>"##,
        if primary { 0.96 } else { 0.50 }
    )
    .unwrap();
    let final_pose = trajectory.final_pose();
    writeln!(
        svg,
        r##"<circle cx="{:.1}" cy="{:.1}" r="{:.1}" fill="{color}" stroke="#ffffff" stroke-width="1.6" opacity="{:.2}"/>"##,
        sx(final_pose.x),
        sy(final_pose.y),
        5.5 + 8.0 * trajectory.probability,
        if primary { 0.96 } else { 0.55 }
    )
    .unwrap();
}

fn render_obstacles(svg: &mut String, scene: &BranchOutDrivingScene2D) {
    for obstacle in &scene.obstacles {
        writeln!(
            svg,
            r##"<ellipse cx="{:.1}" cy="{:.1}" rx="{:.1}" ry="{:.1}" fill="#111827" stroke="#ffffff" stroke-width="2"/>"##,
            sx(obstacle.x),
            sy(obstacle.y),
            sdx(obstacle.radius),
            sdy(obstacle.radius)
        )
        .unwrap();
        writeln!(
            svg,
            r##"<ellipse cx="{:.1}" cy="{:.1}" rx="{:.1}" ry="{:.1}" fill="none" stroke="#ef4444" stroke-width="2" stroke-dasharray="6 5" opacity="0.70"/>"##,
            sx(obstacle.x),
            sy(obstacle.y),
            sdx(obstacle.radius + 0.32),
            sdy(obstacle.radius + 0.32)
        )
        .unwrap();
    }
}

fn render_legend(svg: &mut String, trajectories: &[BranchOutTrajectory2D], x: f64, y: f64) {
    for (index, trajectory) in trajectories.iter().enumerate() {
        let yy = y + index as f64 * 27.0;
        let color = color_for_mode(trajectory.mode);
        writeln!(
            svg,
            r##"<line x1="{x:.1}" y1="{yy:.1}" x2="{:.1}" y2="{yy:.1}" stroke="{color}" stroke-width="6" stroke-linecap="round"/>"##,
            x + 34.0
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{:.1}" y="{:.1}" fill="#334155" font-family="Consolas, Menlo, monospace" font-size="13">p={:.2} {}</text>"##,
            x + 48.0,
            yy + 4.0,
            trajectory.probability,
            trajectory.mode.label()
        )
        .unwrap();
    }
}

fn render_metrics(svg: &mut String, label: &str, metrics: (f64, f64, f64), x: f64, y: f64) {
    writeln!(
        svg,
        r##"<text x="{x:.1}" y="{y:.1}" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="15" font-weight="700">{label}</text>"##
    )
    .unwrap();
    for (index, line) in [
        format!("pairwise final {:.2}", metrics.0),
        format!("min GT Frechet {:.2}", metrics.1),
        format!("NLL {:.2}", metrics.2),
    ]
    .iter()
    .enumerate()
    {
        writeln!(
            svg,
            r##"<text x="{x:.1}" y="{:.1}" fill="#334155" font-family="Consolas, Menlo, monospace" font-size="13">{line}</text>"##,
            y + 27.0 + index as f64 * 22.0
        )
        .unwrap();
    }
}

fn render_svg(scene: &BranchOutScene) -> RoboticsResult<String> {
    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="1020" height="560" viewBox="0 0 1020 560" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<title id="title">BranchOut-lite multimodal driving</title>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<desc id="desc">A 2-D driving scene with multiple plausible decisions and mixture probabilities.</desc>"##
    )
    .unwrap();
    writeln!(svg, r##"<rect width="1020" height="560" fill="#f8fafc"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="42" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="26" font-weight="700">BranchOut-lite multimodal driving</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="68" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">A compact GMM-style head keeps several plausible futures alive instead of collapsing to one route.</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<rect x="{LEFT:.1}" y="{TOP:.1}" width="{PLOT_WIDTH:.1}" height="{PLOT_HEIGHT:.1}" rx="8" fill="#ffffff" stroke="#dbe3ee" stroke-width="1.5"/>"##
    )
    .unwrap();
    render_road(&mut svg, &scene.driving_scene);
    for trajectory in &scene.unimodal {
        render_trajectory(&mut svg, trajectory, false);
    }
    for trajectory in &scene.branchout {
        render_trajectory(&mut svg, trajectory, true);
    }
    render_obstacles(&mut svg, &scene.driving_scene);
    writeln!(
        svg,
        r##"<rect x="{LEFT:.1}" y="{TOP:.1}" width="{PLOT_WIDTH:.1}" height="{PLOT_HEIGHT:.1}" rx="8" fill="none" stroke="#dbe3ee" stroke-width="1.5"/>"##
    )
    .unwrap();

    render_legend(&mut svg, &scene.branchout, 784.0, 116.0);
    render_metrics(
        &mut svg,
        "branchout-lite",
        scene.branchout_metrics,
        784.0,
        256.0,
    );
    render_metrics(
        &mut svg,
        "unimodal baseline",
        scene.unimodal_metrics,
        784.0,
        372.0,
    );
    writeln!(
        svg,
        r##"<text x="58" y="490" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">Circle size at each trajectory endpoint represents mixture probability. The faded blue path is the single-mode keep-lane baseline.</text>"##
    )
    .unwrap();
    writeln!(svg, "</svg>").unwrap();
    Ok(svg)
}

fn main() -> RoboticsResult<()> {
    let scene = build_scene()?;
    let svg = render_svg(&scene)?;
    let output = Path::new(OUTPUT);
    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(output, svg)?;
    println!(
        "wrote {OUTPUT} branchout_nll={:.2} unimodal_nll={:.2}",
        scene.branchout_metrics.2, scene.unimodal_metrics.2
    );
    Ok(())
}
