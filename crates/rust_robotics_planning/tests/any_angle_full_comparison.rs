//! Full any-angle planner comparison with timing and post-processing
//!
//! Compares on MovingAI maps:
//! - Theta* (baseline)
//! - Lazy Theta* (deferred LOS, faster)
//! - Enhanced Lazy Theta* + optimize_path (best quality)
//! - A* + optimize_path (fast-then-smooth pipeline)
//!
//! Reports path quality AND computation time with statistical tests.

use std::collections::HashMap;
use std::path::Path;
use std::time::Instant;

use rust_robotics_planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics_planning::enhanced_lazy_theta_star::{
    EnhancedLazyThetaStarConfig, EnhancedLazyThetaStarPlanner,
};
use rust_robotics_planning::lazy_theta_star::{LazyThetaStarConfig, LazyThetaStarPlanner};
use rust_robotics_planning::moving_ai::{MovingAiMap, MovingAiScenario};
use rust_robotics_planning::path_smoothing::optimize_path;
use rust_robotics_planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};

const RESOLUTION: f64 = 1.0;
const ROBOT_RADIUS: f64 = 0.0;
const SCENARIOS_PER_BUCKET: usize = 10;
const RELAX_ITERATIONS: usize = 5;

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
            name: "dao",
            map_path: format!("{base}/dao/arena2.map"),
            scen_path: format!("{base}/dao/arena2.map.scen"),
            buckets: &[20, 40, 60, 80],
        },
        MapFamily {
            name: "room",
            map_path: format!("{base}/room/8room_000.map"),
            scen_path: format!("{base}/room/8room_000.map.scen"),
            buckets: &[20, 40, 60, 80],
        },
        MapFamily {
            name: "random",
            map_path: format!("{base}/random/random512-10-0.map"),
            scen_path: format!("{base}/random/random512-10-0.map.scen"),
            buckets: &[20, 40, 60, 80],
        },
        MapFamily {
            name: "street",
            map_path: format!("{base}/street/Berlin_0_512.map"),
            scen_path: format!("{base}/street/Berlin_0_512.map.scen"),
            buckets: &[20, 40, 60, 80],
        },
    ]
}

fn mann_whitney_u(a: &[f64], b: &[f64]) -> (f64, f64) {
    let n1 = a.len() as f64;
    let n2 = b.len() as f64;
    let mut u: f64 = 0.0;
    for &ai in a {
        for &bj in b {
            if ai < bj {
                u += 1.0;
            } else if (ai - bj).abs() < 1e-10 {
                u += 0.5;
            }
        }
    }
    let mean_u = n1 * n2 / 2.0;
    let std_u = (n1 * n2 * (n1 + n2 + 1.0) / 12.0).sqrt();
    if std_u < 1e-10 {
        return (0.0, 1.0);
    }
    let z = (u - mean_u) / std_u;
    let p = 2.0 * normal_cdf(-z.abs());
    (z, p)
}

fn normal_cdf(x: f64) -> f64 {
    let t = 1.0 / (1.0 + 0.2316419 * x.abs());
    let d = 0.3989422804014327;
    let p = d * (-x * x / 2.0).exp()
        * (t * (0.319381530
            + t * (-0.356563782
                + t * (1.781477937 + t * (-1.821255978 + t * 1.330274429)))));
    if x >= 0.0 { 1.0 - p } else { p }
}

#[test]
#[ignore = "long-running: 160 MovingAI scenarios, use --release --ignored"]
fn any_angle_full_comparison() {
    println!();
    println!("==========================================================================");
    println!("  Full Any-Angle Comparison: Quality + Speed");
    println!("  Planners: Theta*, LazyTheta*, Enhanced+Opt, A*+Opt");
    println!("  {} scenarios/bucket, {} families",
        SCENARIOS_PER_BUCKET, 4
    );
    println!("==========================================================================");

    let mut all_theta_len: Vec<f64> = Vec::new();
    let mut all_lazy_len: Vec<f64> = Vec::new();
    let mut all_enh_opt_len: Vec<f64> = Vec::new();
    let mut all_astar_opt_len: Vec<f64> = Vec::new();

    let mut all_theta_time: Vec<f64> = Vec::new();
    let mut all_lazy_time: Vec<f64> = Vec::new();
    let mut all_enh_opt_time: Vec<f64> = Vec::new();
    let mut all_astar_opt_time: Vec<f64> = Vec::new();

    for family in families() {
        let map = match MovingAiMap::from_file(Path::new(&family.map_path)) {
            Ok(m) => m,
            Err(e) => { println!("[SKIP] {}: {}", family.name, e); continue; }
        };
        let scenarios = match MovingAiScenario::from_file(Path::new(&family.scen_path)) {
            Ok(s) => s,
            Err(e) => { println!("[SKIP] {}: {}", family.name, e); continue; }
        };

        let obstacles = map.to_obstacles();

        let a_star = AStarPlanner::from_obstacle_points(&obstacles, AStarConfig {
            resolution: RESOLUTION, robot_radius: ROBOT_RADIUS, heuristic_weight: 1.0,
        }).unwrap();
        let theta = ThetaStarPlanner::from_obstacle_points(&obstacles, ThetaStarConfig {
            resolution: RESOLUTION, robot_radius: ROBOT_RADIUS, heuristic_weight: 1.0,
        }).unwrap();
        let lazy = LazyThetaStarPlanner::from_obstacle_points(&obstacles, LazyThetaStarConfig {
            resolution: RESOLUTION, robot_radius: ROBOT_RADIUS, heuristic_weight: 1.0,
        }).unwrap();
        let enhanced = EnhancedLazyThetaStarPlanner::from_obstacle_points(&obstacles, EnhancedLazyThetaStarConfig {
            resolution: RESOLUTION, robot_radius: ROBOT_RADIUS, heuristic_weight: 1.0,
        }).unwrap();

        let grid_map = a_star.grid_map().clone();

        let mut by_bucket: HashMap<u32, Vec<&MovingAiScenario>> = HashMap::new();
        for s in &scenarios {
            by_bucket.entry(s.bucket).or_default().push(s);
        }

        println!();
        println!("--- {} ---", family.name);
        println!(
            "{:>4} {:>4} | {:>8} {:>8} {:>8} {:>8} | {:>8} {:>8} {:>8} {:>8}",
            "Bkt", "N", "Θ* len", "LΘ* len", "E+O len", "A+O len",
            "Θ* ms", "LΘ* ms", "E+O ms", "A+O ms"
        );
        println!("{}", "-".repeat(100));

        for &bucket in family.buckets {
            let bucket_scenarios = match by_bucket.get(&bucket) {
                Some(s) => s,
                None => continue,
            };

            let n = SCENARIOS_PER_BUCKET.min(bucket_scenarios.len());
            let mut t_lens = Vec::new();
            let mut l_lens = Vec::new();
            let mut eo_lens = Vec::new();
            let mut ao_lens = Vec::new();
            let mut t_times = Vec::new();
            let mut l_times = Vec::new();
            let mut eo_times = Vec::new();
            let mut ao_times = Vec::new();

            for scen in bucket_scenarios.iter().take(n) {
                let start = match map.planning_point(scen.start_x, scen.start_y) {
                    Ok(p) => p, Err(_) => continue,
                };
                let goal = match map.planning_point(scen.goal_x, scen.goal_y) {
                    Ok(p) => p, Err(_) => continue,
                };

                // Theta*
                let t0 = Instant::now();
                let t_path = theta.plan(start, goal);
                let t_time = t0.elapsed().as_secs_f64() * 1000.0;

                // Lazy Theta*
                let t0 = Instant::now();
                let l_path = lazy.plan(start, goal);
                let l_time = t0.elapsed().as_secs_f64() * 1000.0;

                // Enhanced + optimize_path
                let t0 = Instant::now();
                let eo_path = enhanced.plan(start, goal)
                    .map(|p| optimize_path(&p, &grid_map, RELAX_ITERATIONS));
                let eo_time = t0.elapsed().as_secs_f64() * 1000.0;

                // A* + optimize_path
                let t0 = Instant::now();
                let ao_path = a_star.plan(start, goal)
                    .map(|p| optimize_path(&p, &grid_map, RELAX_ITERATIONS));
                let ao_time = t0.elapsed().as_secs_f64() * 1000.0;

                if let (Ok(t), Ok(l), Ok(eo), Ok(ao)) =
                    (&t_path, &l_path, &eo_path, &ao_path)
                {
                    t_lens.push(t.total_length());
                    l_lens.push(l.total_length());
                    eo_lens.push(eo.total_length());
                    ao_lens.push(ao.total_length());
                    t_times.push(t_time);
                    l_times.push(l_time);
                    eo_times.push(eo_time);
                    ao_times.push(ao_time);
                }
            }

            if t_lens.is_empty() { continue; }
            let nn = t_lens.len();
            let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;

            println!(
                "{:>4} {:>4} | {:>8.2} {:>8.2} {:>8.2} {:>8.2} | {:>8.2} {:>8.2} {:>8.2} {:>8.2}",
                bucket, nn,
                mean(&t_lens), mean(&l_lens), mean(&eo_lens), mean(&ao_lens),
                mean(&t_times), mean(&l_times), mean(&eo_times), mean(&ao_times),
            );

            all_theta_len.extend(&t_lens);
            all_lazy_len.extend(&l_lens);
            all_enh_opt_len.extend(&eo_lens);
            all_astar_opt_len.extend(&ao_lens);
            all_theta_time.extend(&t_times);
            all_lazy_time.extend(&l_times);
            all_enh_opt_time.extend(&eo_times);
            all_astar_opt_time.extend(&ao_times);
        }
    }

    let n = all_theta_len.len();
    if n == 0 { return; }
    let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;

    println!();
    println!("==========================================================================");
    println!("  Aggregate ({} scenarios)", n);
    println!("==========================================================================");
    println!("  PATH QUALITY (lower = better):");
    println!("    Theta*:       {:.2}", mean(&all_theta_len));
    println!("    LazyTheta*:   {:.2} ({:+.2}%)", mean(&all_lazy_len),
        (mean(&all_lazy_len) / mean(&all_theta_len) - 1.0) * 100.0);
    println!("    Enhanced+Opt: {:.2} ({:+.2}%)", mean(&all_enh_opt_len),
        (mean(&all_enh_opt_len) / mean(&all_theta_len) - 1.0) * 100.0);
    println!("    A*+Opt:       {:.2} ({:+.2}%)", mean(&all_astar_opt_len),
        (mean(&all_astar_opt_len) / mean(&all_theta_len) - 1.0) * 100.0);

    println!();
    println!("  SPEED (ms, lower = better):");
    println!("    Theta*:       {:.2}ms", mean(&all_theta_time));
    println!("    LazyTheta*:   {:.2}ms ({:.1}x)", mean(&all_lazy_time),
        mean(&all_theta_time) / mean(&all_lazy_time));
    println!("    Enhanced+Opt: {:.2}ms ({:.1}x)", mean(&all_enh_opt_time),
        mean(&all_theta_time) / mean(&all_enh_opt_time));
    println!("    A*+Opt:       {:.2}ms ({:.1}x)", mean(&all_astar_opt_time),
        mean(&all_theta_time) / mean(&all_astar_opt_time));

    // Statistical tests: Enhanced+Opt vs Theta* (quality)
    let (z_qual, p_qual) = mann_whitney_u(&all_enh_opt_len, &all_theta_len);
    // Lazy vs Theta* (speed)
    let (z_speed, p_speed) = mann_whitney_u(&all_theta_time, &all_lazy_time);

    println!();
    println!("  STATISTICAL TESTS:");
    println!("    Enhanced+Opt vs Theta* (quality): z={:.3}, p={:.6} {}",
        z_qual, p_qual,
        if p_qual < 0.001 { "***" } else if p_qual < 0.01 { "**" } else if p_qual < 0.05 { "*" } else { "n.s." });
    println!("    Theta* vs Lazy (speed, Theta slower): z={:.3}, p={:.6} {}",
        z_speed, p_speed,
        if p_speed < 0.001 { "***" } else if p_speed < 0.01 { "**" } else if p_speed < 0.05 { "*" } else { "n.s." });
    println!("==========================================================================");
    println!();
}
