//! FlowField vs N x A* multi-agent comparison
//!
//! Compares the cost of planning N paths to the same goal:
//! - FlowField: N calls to plan() (each recomputes the field internally)
//! - A*: N independent A* plans
//!
//! The key insight is that FlowField's field computation is O(V) regardless
//! of start position, while A* is O(V log V) per agent. For large N,
//! FlowField should win if we can amortize the field computation.
//!
//! Since FlowField.extract_path is private, we use plan() directly and
//! measure total wall-clock time for N agents.

use std::time::Instant;

use rust_robotics_core::{Obstacles, Point2D};
use rust_robotics_planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics_planning::flow_field::{FlowFieldConfig, FlowFieldPlanner};

const RESOLUTION: f64 = 1.0;
const ROBOT_RADIUS: f64 = 0.5;

fn dense_obstacles(size: i32) -> Obstacles {
    let mut obs = Obstacles::new();
    for i in 0..=size {
        obs.push(Point2D::new(i as f64, 0.0));
        obs.push(Point2D::new(i as f64, size as f64));
        obs.push(Point2D::new(0.0, i as f64));
        obs.push(Point2D::new(size as f64, i as f64));
    }
    for x in 2..size - 1 {
        for y in 2..size - 1 {
            let hash =
                ((x as u64).wrapping_mul(2654435761) ^ (y as u64).wrapping_mul(2246822519)) % 100;
            if hash < 15 {
                obs.push(Point2D::new(x as f64, y as f64));
            }
        }
    }
    obs
}

fn generate_starts(size: i32, n: usize) -> Vec<Point2D> {
    let mut starts = Vec::with_capacity(n);
    let mut count = 0;
    'outer: for x in 2..size - 2 {
        for y in 2..size - 2 {
            let hash =
                ((x as u64).wrapping_mul(2654435761) ^ (y as u64).wrapping_mul(2246822519)) % 100;
            if hash >= 15 {
                starts.push(Point2D::new(x as f64, y as f64));
                count += 1;
                if count >= n {
                    break 'outer;
                }
            }
        }
    }
    starts
}

#[test]
fn flowfield_vs_astar_multiagent() {
    let grid_size = 50;
    let obstacles = dense_obstacles(grid_size);
    let goal = Point2D::new(47.0, 47.0);
    let agent_counts = [1, 2, 5, 10, 20, 50];

    let a_star = AStarPlanner::from_obstacle_points(
        &obstacles,
        AStarConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
            heuristic_weight: 1.0,
        },
    )
    .unwrap();

    let flow_field = FlowFieldPlanner::from_obstacle_points(
        &obstacles,
        FlowFieldConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
        },
    )
    .unwrap();

    let all_starts = generate_starts(grid_size, 50);

    println!();
    println!(
        "{:>8} | {:>12} {:>12} | {:>10} {:>12}",
        "N agents", "A* total", "FF total", "FF/A*", "FF wins?"
    );
    println!("{}", "-".repeat(65));

    for &n in &agent_counts {
        let starts = &all_starts[..n];

        // A*: N independent plans
        let t0 = Instant::now();
        let mut a_star_ok = 0;
        for s in starts {
            if a_star.plan(*s, goal).is_ok() {
                a_star_ok += 1;
            }
        }
        let a_star_time = t0.elapsed();

        // FlowField: N plans (each recomputes field)
        let t0 = Instant::now();
        let mut ff_ok = 0;
        for s in starts {
            if flow_field.plan(*s, goal).is_ok() {
                ff_ok += 1;
            }
        }
        let ff_time = t0.elapsed();

        let ratio = ff_time.as_secs_f64() / a_star_time.as_secs_f64();
        let ff_wins = if ratio < 1.0 { "YES" } else { "no" };

        println!(
            "{:>8} | {:>10.1}µs {:>10.1}µs | {:>10.2}x {:>12}",
            n,
            a_star_time.as_micros() as f64,
            ff_time.as_micros() as f64,
            ratio,
            ff_wins,
        );

        assert_eq!(
            a_star_ok, ff_ok,
            "both planners should find the same number of paths"
        );
    }

    println!();
    println!("Note: FlowField currently recomputes the field for each plan() call.");
    println!("With field caching (compute once, extract N paths), FlowField would");
    println!("dominate for large N since field computation is O(V) and path");
    println!("extraction is O(path_length).");
    println!();
}
