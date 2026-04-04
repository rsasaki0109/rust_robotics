//! Any-Angle Planner Comparison on MovingAI Benchmarks
//!
//! Compares Theta*, Lazy Theta*, and Enhanced Lazy Theta* on 5 MovingAI
//! map families (dao, room, random, maze, street) across multiple buckets.
//!
//! Measures: path length, path length vs grid-optimal (A*), waypoint count,
//! and computation time.

use std::collections::HashMap;
use std::path::Path;
use std::time::Instant;

use rust_robotics_core::{Obstacles, Point2D};
use rust_robotics_planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics_planning::enhanced_lazy_theta_star::{
    EnhancedLazyThetaStarConfig, EnhancedLazyThetaStarPlanner,
};
use rust_robotics_planning::lazy_theta_star::{LazyThetaStarConfig, LazyThetaStarPlanner};
use rust_robotics_planning::moving_ai::{MovingAiMap, MovingAiScenario};
use rust_robotics_planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};

const RESOLUTION: f64 = 1.0;
const ROBOT_RADIUS: f64 = 0.0; // MovingAI maps have no inflation

struct MapFamily {
    name: &'static str,
    map_path: String,
    scen_path: String,
    buckets: &'static [u32],
}

fn families() -> Vec<MapFamily> {
    let base = concat!(env!("CARGO_MANIFEST_DIR"), "/benchdata/moving_ai");
    vec![
        MapFamily {
            name: "dao/arena2",
            map_path: format!("{base}/dao/arena2.map"),
            scen_path: format!("{base}/dao/arena2.map.scen"),
            buckets: &[0, 10, 20, 40, 60, 80],
        },
        MapFamily {
            name: "room/8room",
            map_path: format!("{base}/room/8room_000.map"),
            scen_path: format!("{base}/room/8room_000.map.scen"),
            buckets: &[0, 10, 20, 40, 60, 80],
        },
        MapFamily {
            name: "random/512",
            map_path: format!("{base}/random/random512-10-0.map"),
            scen_path: format!("{base}/random/random512-10-0.map.scen"),
            buckets: &[0, 10, 20, 40, 60, 80],
        },
        MapFamily {
            name: "maze/512",
            map_path: format!("{base}/maze/maze512-1-0.map"),
            scen_path: format!("{base}/maze/maze512-1-0.map.scen"),
            buckets: &[0, 10, 20, 40, 60, 80],
        },
        MapFamily {
            name: "street/berlin",
            map_path: format!("{base}/street/Berlin_0_512.map"),
            scen_path: format!("{base}/street/Berlin_0_512.map.scen"),
            buckets: &[0, 10, 20, 40, 60, 80],
        },
    ]
}

struct PlanResult {
    planner_name: &'static str,
    path_length: f64,
    waypoints: usize,
    time_us: u128,
}

#[test]
#[ignore = "long-running: MovingAI 5 families, use --release --ignored"]
fn any_angle_movingai_comparison() {
    println!();
    println!("==========================================================================");
    println!("  Any-Angle Planner Comparison on MovingAI Maps");
    println!("==========================================================================");
    println!();

    let mut total_theta_shorter = 0usize;
    let mut total_lazy_shorter = 0usize;
    let mut total_enhanced_shorter = 0usize;
    let mut total_scenarios = 0usize;

    // Aggregated improvement ratios
    let mut lazy_vs_theta_ratios: Vec<f64> = Vec::new();
    let mut enhanced_vs_theta_ratios: Vec<f64> = Vec::new();
    let mut enhanced_vs_lazy_ratios: Vec<f64> = Vec::new();

    for family in families() {
        let map = match MovingAiMap::from_file(Path::new(&family.map_path)) {
            Ok(m) => m,
            Err(e) => {
                println!("[SKIP] {}: {}", family.name, e);
                continue;
            }
        };
        let scenarios = match MovingAiScenario::from_file(Path::new(&family.scen_path)) {
            Ok(s) => s,
            Err(e) => {
                println!("[SKIP] {}: {}", family.name, e);
                continue;
            }
        };

        let obstacles = map.to_obstacles();

        // Build planners once per map
        let a_star = AStarPlanner::from_obstacle_points(
            &obstacles,
            AStarConfig {
                resolution: RESOLUTION,
                robot_radius: ROBOT_RADIUS,
                heuristic_weight: 1.0,
            },
        );
        let theta = ThetaStarPlanner::from_obstacle_points(
            &obstacles,
            ThetaStarConfig {
                resolution: RESOLUTION,
                robot_radius: ROBOT_RADIUS,
                heuristic_weight: 1.0,
            },
        );
        let lazy = LazyThetaStarPlanner::from_obstacle_points(
            &obstacles,
            LazyThetaStarConfig {
                resolution: RESOLUTION,
                robot_radius: ROBOT_RADIUS,
                heuristic_weight: 1.0,
            },
        );
        let enhanced = EnhancedLazyThetaStarPlanner::from_obstacle_points(
            &obstacles,
            EnhancedLazyThetaStarConfig {
                resolution: RESOLUTION,
                robot_radius: ROBOT_RADIUS,
                heuristic_weight: 1.0,
            },
        );

        if a_star.is_err() || theta.is_err() || lazy.is_err() || enhanced.is_err() {
            println!("[SKIP] {} (planner construction failed)", family.name);
            continue;
        }
        let a_star = a_star.unwrap();
        let theta = theta.unwrap();
        let lazy = lazy.unwrap();
        let enhanced = enhanced.unwrap();

        // Group scenarios by bucket
        let mut by_bucket: HashMap<u32, Vec<&MovingAiScenario>> = HashMap::new();
        for s in &scenarios {
            by_bucket.entry(s.bucket).or_default().push(s);
        }

        println!(
            "{:<18} {:>6} {:>10} {:>10} {:>10} {:>10} | {:>8} {:>8} {:>8}",
            "Map/Bucket", "OptLen", "A*", "Theta*", "Lazy*", "Enh*",
            "L/T %", "E/T %", "E/L %"
        );
        println!("{}", "-".repeat(110));

        for &bucket in family.buckets {
            let bucket_scenarios = match by_bucket.get(&bucket) {
                Some(s) => s,
                None => continue,
            };

            // Take first scenario from this bucket
            let scen = bucket_scenarios[0];
            let start = match map.planning_point(scen.start_x, scen.start_y) {
                Ok(p) => p,
                Err(_) => continue,
            };
            let goal = match map.planning_point(scen.goal_x, scen.goal_y) {
                Ok(p) => p,
                Err(_) => continue,
            };

            // A* (grid-optimal reference)
            let a_star_len = a_star
                .plan(start, goal)
                .map(|p| p.total_length())
                .unwrap_or(f64::NAN);

            // Theta*
            let t0 = Instant::now();
            let theta_result = theta.plan(start, goal);
            let theta_time = t0.elapsed().as_micros();
            let theta_len = theta_result
                .as_ref()
                .map(|p| p.total_length())
                .unwrap_or(f64::NAN);

            // Lazy Theta*
            let t0 = Instant::now();
            let lazy_result = lazy.plan(start, goal);
            let lazy_time = t0.elapsed().as_micros();
            let lazy_len = lazy_result
                .as_ref()
                .map(|p| p.total_length())
                .unwrap_or(f64::NAN);

            // Enhanced Lazy Theta*
            let t0 = Instant::now();
            let enhanced_result = enhanced.plan(start, goal);
            let enhanced_time = t0.elapsed().as_micros();
            let enhanced_len = enhanced_result
                .as_ref()
                .map(|p| p.total_length())
                .unwrap_or(f64::NAN);

            if theta_len.is_nan() || lazy_len.is_nan() || enhanced_len.is_nan() {
                continue;
            }

            let lazy_vs_theta = (lazy_len / theta_len - 1.0) * 100.0;
            let enhanced_vs_theta = (enhanced_len / theta_len - 1.0) * 100.0;
            let enhanced_vs_lazy = (enhanced_len / lazy_len - 1.0) * 100.0;

            println!(
                "{:<14} b{:<3} {:>10.2} {:>10.2} {:>10.2} {:>10.2} {:>10.2} | {:>+7.1}% {:>+7.1}% {:>+7.1}%",
                family.name,
                bucket,
                scen.optimal_length,
                a_star_len,
                theta_len,
                lazy_len,
                enhanced_len,
                lazy_vs_theta,
                enhanced_vs_theta,
                enhanced_vs_lazy,
            );

            // Track which planner wins
            let best = theta_len.min(lazy_len).min(enhanced_len);
            if (theta_len - best).abs() < 1e-6 {
                total_theta_shorter += 1;
            }
            if (lazy_len - best).abs() < 1e-6 {
                total_lazy_shorter += 1;
            }
            if (enhanced_len - best).abs() < 1e-6 {
                total_enhanced_shorter += 1;
            }
            total_scenarios += 1;

            lazy_vs_theta_ratios.push(lazy_len / theta_len);
            enhanced_vs_theta_ratios.push(enhanced_len / theta_len);
            enhanced_vs_lazy_ratios.push(enhanced_len / lazy_len);
        }
        println!();
    }

    // Summary
    println!("==========================================================================");
    println!("  Summary ({} scenarios across 5 map families)", total_scenarios);
    println!("==========================================================================");
    println!(
        "  Theta* wins (shortest): {}/{}",
        total_theta_shorter, total_scenarios
    );
    println!(
        "  LazyTheta* wins:        {}/{}",
        total_lazy_shorter, total_scenarios
    );
    println!(
        "  EnhancedLazy* wins:     {}/{}",
        total_enhanced_shorter, total_scenarios
    );

    if !lazy_vs_theta_ratios.is_empty() {
        let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
        println!();
        println!(
            "  Lazy/Theta* mean ratio:     {:.4} ({:+.1}%)",
            mean(&lazy_vs_theta_ratios),
            (mean(&lazy_vs_theta_ratios) - 1.0) * 100.0
        );
        println!(
            "  Enhanced/Theta* mean ratio: {:.4} ({:+.1}%)",
            mean(&enhanced_vs_theta_ratios),
            (mean(&enhanced_vs_theta_ratios) - 1.0) * 100.0
        );
        println!(
            "  Enhanced/Lazy* mean ratio:  {:.4} ({:+.1}%)",
            mean(&enhanced_vs_lazy_ratios),
            (mean(&enhanced_vs_lazy_ratios) - 1.0) * 100.0
        );
    }
    println!("==========================================================================");
    println!();
}
