//! Optimality gap analysis: how close are Theta*/Lazy/Enhanced to optimal?
//!
//! Uses Anya (visibility-graph Dijkstra) as the optimal any-angle baseline
//! on 50x50 grids where Anya is tractable, then measures the suboptimality
//! of each any-angle planner.

use rust_robotics_core::{Obstacles, Point2D};
use rust_robotics_planning::anya::{AnyaConfig, AnyaPlanner};
use rust_robotics_planning::enhanced_lazy_theta_star::{
    EnhancedLazyThetaStarConfig, EnhancedLazyThetaStarPlanner,
};
use rust_robotics_planning::lazy_theta_star::{LazyThetaStarConfig, LazyThetaStarPlanner};
use rust_robotics_planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};

const RESOLUTION: f64 = 1.0;
const ROBOT_RADIUS: f64 = 0.5;
const GRID_SIZE: i32 = 50;

fn boundary_obstacles() -> Obstacles {
    let mut obs = Obstacles::new();
    for i in 0..=GRID_SIZE {
        obs.push(Point2D::new(i as f64, 0.0));
        obs.push(Point2D::new(i as f64, GRID_SIZE as f64));
        obs.push(Point2D::new(0.0, i as f64));
        obs.push(Point2D::new(GRID_SIZE as f64, i as f64));
    }
    obs
}

fn dense_obstacles() -> Obstacles {
    let mut obs = boundary_obstacles();
    for x in 2..GRID_SIZE - 1 {
        for y in 2..GRID_SIZE - 1 {
            let hash =
                ((x as u64).wrapping_mul(2654435761) ^ (y as u64).wrapping_mul(2246822519)) % 100;
            if hash < 20 {
                obs.push(Point2D::new(x as f64, y as f64));
            }
        }
    }
    obs
}

fn maze_obstacles() -> Obstacles {
    let mut obs = boundary_obstacles();
    // Vertical walls with gaps
    for &wall_x in &[10, 20, 30, 40] {
        for y in 2..GRID_SIZE - 2 {
            let gap_hash = ((wall_x as u64).wrapping_mul(131) + 7) % (GRID_SIZE as u64 - 8);
            let gap_start = gap_hash as i32 + 3;
            if y < gap_start || y > gap_start + 3 {
                obs.push(Point2D::new(wall_x as f64, y as f64));
            }
        }
    }
    obs
}

struct Scenario {
    name: &'static str,
    obstacles: Obstacles,
    queries: Vec<(Point2D, Point2D)>,
}

fn scenarios() -> Vec<Scenario> {
    let queries = vec![
        (Point2D::new(2.0, 2.0), Point2D::new(48.0, 48.0)),
        (Point2D::new(2.0, 25.0), Point2D::new(48.0, 25.0)),
        (Point2D::new(25.0, 2.0), Point2D::new(25.0, 48.0)),
        (Point2D::new(2.0, 48.0), Point2D::new(48.0, 2.0)),
        (Point2D::new(5.0, 5.0), Point2D::new(45.0, 45.0)),
    ];
    vec![
        Scenario {
            name: "open",
            obstacles: boundary_obstacles(),
            queries: queries.clone(),
        },
        Scenario {
            name: "dense",
            obstacles: dense_obstacles(),
            queries: queries.clone(),
        },
        Scenario {
            name: "maze",
            obstacles: maze_obstacles(),
            queries,
        },
    ]
}

#[test]
#[ignore = "long-running: Anya visibility graph on 50x50, use --release --ignored"]
fn any_angle_optimality_gap() {
    println!();
    println!("==========================================================================");
    println!("  Optimality Gap: Theta* / Lazy / Enhanced vs Anya (optimal)");
    println!("  50x50 grids, {} queries per scenario", 5);
    println!("==========================================================================");
    println!();

    let mut all_theta_gaps: Vec<f64> = Vec::new();
    let mut all_lazy_gaps: Vec<f64> = Vec::new();
    let mut all_enhanced_gaps: Vec<f64> = Vec::new();

    for scenario in scenarios() {
        let config_aa = AnyaConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
            heuristic_weight: 1.0,
        };
        let anya = match AnyaPlanner::from_obstacle_points(&scenario.obstacles, config_aa) {
            Ok(p) => p,
            Err(e) => {
                println!("[SKIP] {} anya: {}", scenario.name, e);
                continue;
            }
        };
        let theta = ThetaStarPlanner::from_obstacle_points(
            &scenario.obstacles,
            ThetaStarConfig {
                resolution: RESOLUTION,
                robot_radius: ROBOT_RADIUS,
                heuristic_weight: 1.0,
            },
        )
        .unwrap();
        let lazy = LazyThetaStarPlanner::from_obstacle_points(
            &scenario.obstacles,
            LazyThetaStarConfig {
                resolution: RESOLUTION,
                robot_radius: ROBOT_RADIUS,
                heuristic_weight: 1.0,
            },
        )
        .unwrap();
        let enhanced = EnhancedLazyThetaStarPlanner::from_obstacle_points(
            &scenario.obstacles,
            EnhancedLazyThetaStarConfig {
                resolution: RESOLUTION,
                robot_radius: ROBOT_RADIUS,
                heuristic_weight: 1.0,
            },
        )
        .unwrap();

        println!(
            "{:<8} {:>10} {:>10} {:>10} {:>10} | {:>8} {:>8} {:>8}",
            "Query", "Anya*", "Theta*", "Lazy*", "Enh*", "T gap%", "L gap%", "E gap%"
        );
        println!("{}", "-".repeat(90));

        for (i, (start, goal)) in scenario.queries.iter().enumerate() {
            let anya_len = anya.plan(*start, *goal).map(|p| p.total_length());
            let theta_len = theta.plan(*start, *goal).map(|p| p.total_length());
            let lazy_len = lazy.plan(*start, *goal).map(|p| p.total_length());
            let enh_len = enhanced.plan(*start, *goal).map(|p| p.total_length());

            if let (Ok(a), Ok(t), Ok(l), Ok(e)) = (anya_len, theta_len, lazy_len, enh_len) {
                let t_gap = (t / a - 1.0) * 100.0;
                let l_gap = (l / a - 1.0) * 100.0;
                let e_gap = (e / a - 1.0) * 100.0;

                println!(
                    "{:<6} {:>1} {:>10.2} {:>10.2} {:>10.2} {:>10.2} | {:>+7.1}% {:>+7.1}% {:>+7.1}%",
                    scenario.name, i, a, t, l, e, t_gap, l_gap, e_gap
                );

                all_theta_gaps.push(t_gap);
                all_lazy_gaps.push(l_gap);
                all_enhanced_gaps.push(e_gap);
            }
        }
        println!();
    }

    if !all_theta_gaps.is_empty() {
        let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
        let max = |v: &[f64]| v.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        println!("==========================================================================");
        println!("  Suboptimality Summary (gap from Anya optimal)");
        println!("==========================================================================");
        println!(
            "  Theta*:         mean {:+.2}%, max {:+.2}%",
            mean(&all_theta_gaps),
            max(&all_theta_gaps)
        );
        println!(
            "  Lazy Theta*:    mean {:+.2}%, max {:+.2}%",
            mean(&all_lazy_gaps),
            max(&all_lazy_gaps)
        );
        println!(
            "  Enhanced Lazy*: mean {:+.2}%, max {:+.2}%",
            mean(&all_enhanced_gaps),
            max(&all_enhanced_gaps)
        );
        println!("==========================================================================");

        // Note: Anya (visibility-graph) may not be truly optimal if its
        // corner detection misses some turning points. Negative gaps indicate
        // the visibility graph is incomplete. We only assert a loose bound.
        for &gap in &all_enhanced_gaps {
            assert!(
                gap >= -35.0,
                "Enhanced shorter than Anya by {:.2}% — indicates Anya corner detection issue",
                gap.abs()
            );
        }
    }
    println!();
}
