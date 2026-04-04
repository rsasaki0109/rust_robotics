//! Statistical comparison: Lazy Theta* vs Theta* path quality
//!
//! Runs 10 scenarios per bucket across 5 MovingAI families and applies
//! Mann-Whitney U test to determine if Lazy Theta* produces statistically
//! significantly shorter paths than Theta*.

use std::collections::HashMap;
use std::path::Path;

use rust_robotics_planning::enhanced_lazy_theta_star::{
    EnhancedLazyThetaStarConfig, EnhancedLazyThetaStarPlanner,
};
use rust_robotics_planning::lazy_theta_star::{LazyThetaStarConfig, LazyThetaStarPlanner};
use rust_robotics_planning::moving_ai::{MovingAiMap, MovingAiScenario};
use rust_robotics_planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};

const RESOLUTION: f64 = 1.0;
const ROBOT_RADIUS: f64 = 0.0;
const SCENARIOS_PER_BUCKET: usize = 10;

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
            buckets: &[10, 20, 40, 60, 80],
        },
        MapFamily {
            name: "room/8room",
            map_path: format!("{base}/room/8room_000.map"),
            scen_path: format!("{base}/room/8room_000.map.scen"),
            buckets: &[10, 20, 40, 60, 80],
        },
        MapFamily {
            name: "random/512",
            map_path: format!("{base}/random/random512-10-0.map"),
            scen_path: format!("{base}/random/random512-10-0.map.scen"),
            buckets: &[10, 20, 40, 60, 80],
        },
        MapFamily {
            name: "maze/512",
            map_path: format!("{base}/maze/maze512-1-0.map"),
            scen_path: format!("{base}/maze/maze512-1-0.map.scen"),
            buckets: &[10, 20, 40, 60, 80],
        },
        MapFamily {
            name: "street/berlin",
            map_path: format!("{base}/street/Berlin_0_512.map"),
            scen_path: format!("{base}/street/Berlin_0_512.map.scen"),
            buckets: &[10, 20, 40, 60, 80],
        },
    ]
}

/// Mann-Whitney U test (two-sided).
/// Returns (U statistic, z-score, p-value approximation).
/// Positive z means `a` tends to be smaller (better) than `b`.
fn mann_whitney_u(a: &[f64], b: &[f64]) -> (f64, f64, f64) {
    let n1 = a.len() as f64;
    let n2 = b.len() as f64;

    // Count how many times a[i] < b[j]
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
        return (u, 0.0, 1.0);
    }

    let z = (u - mean_u) / std_u;

    // Normal approximation for p-value (two-sided)
    let p = 2.0 * normal_cdf(-z.abs());

    (u, z, p)
}

/// Standard normal CDF approximation (Abramowitz and Stegun)
fn normal_cdf(x: f64) -> f64 {
    let t = 1.0 / (1.0 + 0.2316419 * x.abs());
    let d = 0.3989422804014327; // 1/sqrt(2*pi)
    let p = d * (-x * x / 2.0).exp()
        * (t * (0.319381530
            + t * (-0.356563782
                + t * (1.781477937 + t * (-1.821255978 + t * 1.330274429)))));
    if x >= 0.0 {
        1.0 - p
    } else {
        p
    }
}

struct BucketResult {
    family: &'static str,
    bucket: u32,
    n_scenarios: usize,
    theta_lengths: Vec<f64>,
    lazy_lengths: Vec<f64>,
    enhanced_lengths: Vec<f64>,
}

#[test]
#[ignore = "long-running: 250 MovingAI scenarios, use --release --ignored"]
fn any_angle_statistical_test() {
    println!();
    println!("==========================================================================");
    println!("  Statistical Test: Lazy Theta* vs Theta* Path Quality");
    println!("  {} scenarios per bucket, Mann-Whitney U test",
        SCENARIOS_PER_BUCKET
    );
    println!("==========================================================================");
    println!();

    let mut all_results: Vec<BucketResult> = Vec::new();

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

        let theta = match ThetaStarPlanner::from_obstacle_points(
            &obstacles,
            ThetaStarConfig {
                resolution: RESOLUTION,
                robot_radius: ROBOT_RADIUS,
                heuristic_weight: 1.0,
            },
        ) {
            Ok(p) => p,
            Err(_) => continue,
        };
        let lazy = match LazyThetaStarPlanner::from_obstacle_points(
            &obstacles,
            LazyThetaStarConfig {
                resolution: RESOLUTION,
                robot_radius: ROBOT_RADIUS,
                heuristic_weight: 1.0,
            },
        ) {
            Ok(p) => p,
            Err(_) => continue,
        };
        let enhanced = match EnhancedLazyThetaStarPlanner::from_obstacle_points(
            &obstacles,
            EnhancedLazyThetaStarConfig {
                resolution: RESOLUTION,
                robot_radius: ROBOT_RADIUS,
                heuristic_weight: 1.0,
            },
        ) {
            Ok(p) => p,
            Err(_) => continue,
        };

        // Group scenarios by bucket
        let mut by_bucket: HashMap<u32, Vec<&MovingAiScenario>> = HashMap::new();
        for s in &scenarios {
            by_bucket.entry(s.bucket).or_default().push(s);
        }

        for &bucket in family.buckets {
            let bucket_scenarios = match by_bucket.get(&bucket) {
                Some(s) => s,
                None => continue,
            };

            let n = SCENARIOS_PER_BUCKET.min(bucket_scenarios.len());
            let mut theta_lengths = Vec::with_capacity(n);
            let mut lazy_lengths = Vec::with_capacity(n);
            let mut enhanced_lengths = Vec::with_capacity(n);

            for scen in bucket_scenarios.iter().take(n) {
                let start = match map.planning_point(scen.start_x, scen.start_y) {
                    Ok(p) => p,
                    Err(_) => continue,
                };
                let goal = match map.planning_point(scen.goal_x, scen.goal_y) {
                    Ok(p) => p,
                    Err(_) => continue,
                };

                let t_len = theta.plan(start, goal).map(|p| p.total_length());
                let l_len = lazy.plan(start, goal).map(|p| p.total_length());
                let e_len = enhanced.plan(start, goal).map(|p| p.total_length());

                if let (Ok(t), Ok(l), Ok(e)) = (t_len, l_len, e_len) {
                    theta_lengths.push(t);
                    lazy_lengths.push(l);
                    enhanced_lengths.push(e);
                }
            }

            if theta_lengths.len() >= 3 {
                all_results.push(BucketResult {
                    family: family.name,
                    bucket,
                    n_scenarios: theta_lengths.len(),
                    theta_lengths,
                    lazy_lengths,
                    enhanced_lengths,
                });
            }
        }
    }

    // Print results with statistical tests
    println!(
        "{:<18} {:>4} {:>4} {:>10} {:>10} {:>10} {:>8} {:>8} {:>10}",
        "Family", "Bkt", "N", "Theta* μ", "Lazy* μ", "Enh* μ", "Δ L/T%", "Δ E/T%", "p-value"
    );
    println!("{}", "-".repeat(100));

    let mut total_significant = 0usize;
    let mut total_tests = 0usize;
    let mut all_theta: Vec<f64> = Vec::new();
    let mut all_lazy: Vec<f64> = Vec::new();
    let mut all_enhanced: Vec<f64> = Vec::new();

    for r in &all_results {
        let theta_mean: f64 = r.theta_lengths.iter().sum::<f64>() / r.n_scenarios as f64;
        let lazy_mean: f64 = r.lazy_lengths.iter().sum::<f64>() / r.n_scenarios as f64;
        let enhanced_mean: f64 = r.enhanced_lengths.iter().sum::<f64>() / r.n_scenarios as f64;

        // Compute ratios per scenario for paired comparison
        let lazy_ratios: Vec<f64> = r.lazy_lengths.iter()
            .zip(r.theta_lengths.iter())
            .map(|(l, t)| l / t)
            .collect();
        let _ones: Vec<f64> = vec![1.0; lazy_ratios.len()];

        let (_, z, p) = mann_whitney_u(&r.lazy_lengths, &r.theta_lengths);
        let sig = if p < 0.05 && z > 0.0 { "*" } else { "" }; // z > 0 means lazy < theta

        let delta_lt = (lazy_mean / theta_mean - 1.0) * 100.0;
        let delta_et = (enhanced_mean / theta_mean - 1.0) * 100.0;

        println!(
            "{:<18} {:>4} {:>4} {:>10.2} {:>10.2} {:>10.2} {:>+7.1}% {:>+7.1}% {:>9.4} {}",
            r.family, r.bucket, r.n_scenarios,
            theta_mean, lazy_mean, enhanced_mean,
            delta_lt, delta_et, p, sig
        );

        if p < 0.05 && z > 0.0 {
            total_significant += 1;
        }
        total_tests += 1;

        all_theta.extend(&r.theta_lengths);
        all_lazy.extend(&r.lazy_lengths);
        all_enhanced.extend(&r.enhanced_lengths);
    }

    println!();
    println!("==========================================================================");
    println!("  Aggregate Results");
    println!("==========================================================================");
    println!(
        "  Total bucket-level tests: {}, significant (p<0.05, Lazy < Theta*): {}",
        total_tests, total_significant
    );

    if !all_theta.is_empty() {
        let n = all_theta.len();
        let theta_mean: f64 = all_theta.iter().sum::<f64>() / n as f64;
        let lazy_mean: f64 = all_lazy.iter().sum::<f64>() / n as f64;
        let enhanced_mean: f64 = all_enhanced.iter().sum::<f64>() / n as f64;

        let (_, z_all, p_all) = mann_whitney_u(&all_lazy, &all_theta);
        let (_, z_enh, p_enh) = mann_whitney_u(&all_enhanced, &all_theta);

        println!(
            "  Total scenarios: {}", n
        );
        println!(
            "  Overall Theta* mean: {:.2}, Lazy* mean: {:.2} ({:+.1}%), Enhanced* mean: {:.2} ({:+.1}%)",
            theta_mean,
            lazy_mean,
            (lazy_mean / theta_mean - 1.0) * 100.0,
            enhanced_mean,
            (enhanced_mean / theta_mean - 1.0) * 100.0,
        );
        println!(
            "  Lazy vs Theta* (all): z={:.3}, p={:.6} {}",
            z_all, p_all,
            if p_all < 0.001 { "***" } else if p_all < 0.01 { "**" } else if p_all < 0.05 { "*" } else { "n.s." }
        );
        println!(
            "  Enhanced vs Theta* (all): z={:.3}, p={:.6} {}",
            z_enh, p_enh,
            if p_enh < 0.001 { "***" } else if p_enh < 0.01 { "**" } else if p_enh < 0.05 { "*" } else { "n.s." }
        );
    }
    println!("==========================================================================");
    println!();
}
