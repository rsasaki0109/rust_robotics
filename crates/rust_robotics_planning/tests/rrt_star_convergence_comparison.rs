//! RRT* variant convergence comparison
//!
//! Compares RRT*, InformedRRT*, and BatchInformedRRT* on the same scenario,
//! measuring final path cost at fixed iteration counts.
//!
//! RRT*-Dubins, RRT*-ReedsShepp, and ClosedLoopRRT* use Pose2D (x,y,yaw)
//! and non-holonomic constraints, so they're not directly comparable with
//! the standard (x,y) planners and are excluded.

use rust_robotics_planning::batch_informed_rrt_star::{
    BatchInformedRRTStar, BatchInformedRRTStarConfig,
};
use rust_robotics_planning::informed_rrt_star::InformedRRTStar;
use rust_robotics_planning::rrt_star::RRTStar;

fn circle_obstacles() -> Vec<(f64, f64, f64)> {
    vec![
        (5.0, 5.0, 1.0),
        (3.0, 6.0, 2.0),
        (3.0, 8.0, 2.0),
        (3.0, 10.0, 2.0),
        (7.0, 5.0, 2.0),
        (9.0, 5.0, 2.0),
        (8.0, 10.0, 1.0),
    ]
}

fn path_cost(path: &[[f64; 2]]) -> f64 {
    if path.len() < 2 {
        return 0.0;
    }
    path.windows(2)
        .map(|w| ((w[1][0] - w[0][0]).powi(2) + (w[1][1] - w[0][1]).powi(2)).sqrt())
        .sum()
}

struct ConvergenceResult {
    name: &'static str,
    max_iter: i32,
    cost: Option<f64>,
    waypoints: usize,
}

fn run_rrt_star(max_iter: i32) -> ConvergenceResult {
    let obstacles = circle_obstacles();
    let mut planner = RRTStar::new(
        (0.0, 0.0),  // start
        (6.0, 10.0), // goal
        obstacles,
        (-2.0, 15.0), // rand_area
        3.0,          // expand_dis
        0.5,          // path_resolution
        20,           // goal_sample_rate
        max_iter,     // max_iter
        50.0,         // connect_circle_dist
        true,         // search_until_max_iter
        0.0,          // robot_radius
    );
    let path = planner.planning();
    ConvergenceResult {
        name: "RRT*",
        max_iter,
        cost: path.as_ref().map(|p| path_cost(p)),
        waypoints: path.map_or(0, |p| p.len()),
    }
}

fn run_informed_rrt_star(max_iter: i32) -> ConvergenceResult {
    let obstacles = circle_obstacles();
    let mut planner = InformedRRTStar::new(
        (0.0, 0.0),
        (6.0, 10.0),
        obstacles,
        (-2.0, 15.0),
        3.0,
        20,
        max_iter as usize,
    );
    let path = planner.planning();
    ConvergenceResult {
        name: "InformedRRT*",
        max_iter,
        cost: path.as_ref().map(|p| path_cost(p)),
        waypoints: path.map_or(0, |p| p.len()),
    }
}

fn run_batch_informed_rrt_star(max_iter: i32) -> ConvergenceResult {
    let obstacles = circle_obstacles();
    let batch_size = 50;
    let max_batches = (max_iter as usize) / batch_size;
    let config = BatchInformedRRTStarConfig {
        batch_size,
        max_batches: max_batches.max(1),
        expand_dis: 3.0,
        goal_sample_rate: 20,
        obstacle_list: obstacles,
        rand_area: (-2.0, 15.0),
    };
    let mut planner = BatchInformedRRTStar::new((0.0, 0.0), (6.0, 10.0), config);
    let path = planner.planning();
    ConvergenceResult {
        name: "BatchInformedRRT*",
        max_iter,
        cost: path.as_ref().map(|p| path_cost(p)),
        waypoints: path.map_or(0, |p| p.len()),
    }
}

#[test]
fn rrt_star_convergence_comparison() {
    let iter_counts = [100, 200, 500, 1000, 2000];

    println!();
    println!(
        "{:<22} {:>8} {:>12} {:>10}",
        "Planner", "MaxIter", "Cost", "Waypoints"
    );
    println!("{}", "-".repeat(55));

    for &max_iter in &iter_counts {
        let results = vec![
            run_rrt_star(max_iter),
            run_informed_rrt_star(max_iter),
            run_batch_informed_rrt_star(max_iter),
        ];
        for r in &results {
            let cost_str = r.cost.map_or("-".to_string(), |c| format!("{c:.2}"));
            println!(
                "{:<22} {:>8} {:>12} {:>10}",
                r.name, r.max_iter, cost_str, r.waypoints
            );
        }
        println!();
    }

    // At 2000 iterations, all three should find a path
    let results_2000 = vec![
        run_rrt_star(2000),
        run_informed_rrt_star(2000),
        run_batch_informed_rrt_star(2000),
    ];
    for r in &results_2000 {
        assert!(
            r.cost.is_some(),
            "{} should find a path at {} iterations",
            r.name,
            r.max_iter
        );
    }

    // InformedRRT* should have lower or equal cost to basic RRT* at 2000 iters
    // (informed sampling focuses on the optimal region)
    let rrt_cost = results_2000[0].cost.unwrap();
    let informed_cost = results_2000[1].cost.unwrap();
    println!(
        "At 2000 iter: RRT* cost={:.2}, InformedRRT* cost={:.2}, ratio={:.2}",
        rrt_cost,
        informed_cost,
        informed_cost / rrt_cost
    );
}
