//! BranchOut-lite multimodal driving example.
//!
//! A stopped vehicle creates several plausible decisions: yield, pass left, or
//! pass right. The BranchOut-lite planner emits all modes with mixture weights
//! and reports distributional metrics against a multimodal ground-truth set.

use rust_robotics::planning::{
    BranchOutDecisionMode2D, BranchOutDrivingScene2D, BranchOutPlanner2D, BranchOutPlannerConfig2D,
    BranchOutPose2D, BranchOutTrajectory2D,
};
use rust_robotics::prelude::*;

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

fn print_plan(label: &str, plan: &[BranchOutTrajectory2D]) {
    println!("{label}: modes={}", plan.len());
    for trajectory in plan {
        let final_pose = trajectory.final_pose();
        println!(
            "  {:>17}: p={:.2} cost={:.2} final=({:.2},{:.2}) route={:.2} collision={:.2} comfort={:.2}",
            trajectory.mode.label(),
            trajectory.probability,
            trajectory.cost,
            final_pose.x,
            final_pose.y,
            trajectory.route_completion,
            trajectory.collision_risk,
            trajectory.comfort_cost
        );
    }
}

fn main() -> RoboticsResult<()> {
    let scene = BranchOutDrivingScene2D::simple_overtake();
    let branchout = planner_with_modes(vec![
        BranchOutDecisionMode2D::KeepLane,
        BranchOutDecisionMode2D::Yield,
        BranchOutDecisionMode2D::LaneChangeLeft,
        BranchOutDecisionMode2D::LaneChangeRight,
    ])?;
    let unimodal = planner_with_modes(vec![BranchOutDecisionMode2D::KeepLane])?;

    let branchout_plan = branchout.plan(&scene)?;
    let unimodal_plan = unimodal.plan(&scene)?;
    let ground_truths = multimodal_ground_truths(&branchout_plan.trajectories);
    let branchout_metrics = branchout.evaluate_multimodal(&branchout_plan, &ground_truths)?;
    let unimodal_metrics = unimodal.evaluate_multimodal(&unimodal_plan, &ground_truths)?;

    print_plan("branchout-lite", &branchout_plan.trajectories);
    println!(
        "branchout-lite metrics: pairwise_final={:.2} pairwise_frechet={:.2} min_gt_frechet={:.2} nll={:.2} speed_jsd={:.2} expected_route={:.2}",
        branchout_metrics.mean_pairwise_final_distance,
        branchout_metrics.mean_pairwise_frechet,
        branchout_metrics.min_ground_truth_frechet,
        branchout_metrics.negative_log_likelihood,
        branchout_metrics.speed_jsd,
        branchout_metrics.expected_route_completion
    );

    print_plan("unimodal-baseline", &unimodal_plan.trajectories);
    println!(
        "unimodal-baseline metrics: pairwise_final={:.2} pairwise_frechet={:.2} min_gt_frechet={:.2} nll={:.2} speed_jsd={:.2} expected_route={:.2}",
        unimodal_metrics.mean_pairwise_final_distance,
        unimodal_metrics.mean_pairwise_frechet,
        unimodal_metrics.min_ground_truth_frechet,
        unimodal_metrics.negative_log_likelihood,
        unimodal_metrics.speed_jsd,
        unimodal_metrics.expected_route_completion
    );

    Ok(())
}
