//! Benchmark grid-based planners on a shared 100x100 map.
//!
//! Run with:
//!   cargo run -p rust_robotics --example benchmark_grid_planners --features planning --release

use std::time::Instant;

use nalgebra::DMatrix;
use rust_robotics::planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics::planning::ara_star::{ARAStarConfig, ARAStarPlanner};
use rust_robotics::planning::dijkstra::dijkstra_plan;
use rust_robotics::planning::fringe_search::{FringeSearchConfig, FringeSearchPlanner};
use rust_robotics::planning::grid_nalgebra::Map as DijkstraMap;
use rust_robotics::planning::ida_star::{IDAStarConfig, IDAStarPlanner};
use rust_robotics::planning::jps::{JPSConfig, JPSPlanner};
use rust_robotics::planning::lpa_star::{LPAStarConfig, LPAStarPlanner};
use rust_robotics::planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};
use rust_robotics::prelude::*;

#[derive(Debug)]
struct BenchRow {
    planner: &'static str,
    time_us: u128,
    path_length: f64,
}

fn build_grid() -> (Vec<f64>, Vec<f64>, DijkstraMap) {
    let width = 100usize;
    let height = 100usize;
    let mut occ = vec![0_i32; width * height];

    let mut set_obstacle = |x: usize, y: usize| {
        occ[y * width + x] = 1;
    };

    for x in 0..width {
        set_obstacle(x, 0);
        set_obstacle(x, height - 1);
    }
    for y in 0..height {
        set_obstacle(0, y);
        set_obstacle(width - 1, y);
    }

    for y in 10..90 {
        if !(45..=55).contains(&y) {
            set_obstacle(40, y);
        }
        if !(20..=30).contains(&y) {
            set_obstacle(70, y);
        }
    }
    for x in 20..85 {
        if !(58..=65).contains(&x) {
            set_obstacle(x, 60);
        }
    }

    let mut ox = Vec::new();
    let mut oy = Vec::new();
    for y in 0..height {
        for x in 0..width {
            if occ[y * width + x] == 1 {
                ox.push(x as f64);
                oy.push(y as f64);
            }
        }
    }

    let map_matrix = DMatrix::from_row_slice(height, width, &occ);
    let map = DijkstraMap::new(map_matrix, 1).expect("failed to build Dijkstra map");
    (ox, oy, map)
}

fn path_length_from_cells(path: &[(usize, usize)]) -> f64 {
    path.windows(2)
        .map(|w| {
            let dx = w[1].0 as f64 - w[0].0 as f64;
            let dy = w[1].1 as f64 - w[0].1 as f64;
            (dx * dx + dy * dy).sqrt()
        })
        .sum()
}

fn main() -> RoboticsResult<()> {
    let (ox, oy, dijkstra_map) = build_grid();
    let start = Point2D::new(10.0, 10.0);
    let goal = Point2D::new(90.0, 90.0);

    let a_star = AStarPlanner::new(&ox, &oy, AStarConfig::default());
    let jps = JPSPlanner::new(&ox, &oy, JPSConfig::default());
    let theta_star = ThetaStarPlanner::new(&ox, &oy, ThetaStarConfig::default());
    let lpa_star = LPAStarPlanner::new(&ox, &oy, LPAStarConfig::default());
    let ara_star = ARAStarPlanner::new(&ox, &oy, ARAStarConfig::default());
    let fringe = FringeSearchPlanner::new(&ox, &oy, FringeSearchConfig::default());
    let ida_star = IDAStarPlanner::new(&ox, &oy, IDAStarConfig::default());

    let mut rows = Vec::new();

    let t0 = Instant::now();
    let a_star_path = a_star.plan(start, goal)?;
    rows.push(BenchRow {
        planner: "A*",
        time_us: t0.elapsed().as_micros(),
        path_length: a_star_path.total_length(),
    });

    let t0 = Instant::now();
    let dijkstra_path = dijkstra_plan(&dijkstra_map, (10, 10), (90, 90)).ok_or_else(|| {
        RoboticsError::PlanningError("Dijkstra failed to find a path".to_string())
    })?;
    rows.push(BenchRow {
        planner: "Dijkstra",
        time_us: t0.elapsed().as_micros(),
        path_length: path_length_from_cells(&dijkstra_path),
    });

    let t0 = Instant::now();
    let jps_path = jps.plan(start, goal)?;
    rows.push(BenchRow {
        planner: "JPS",
        time_us: t0.elapsed().as_micros(),
        path_length: jps_path.total_length(),
    });

    let t0 = Instant::now();
    let theta_path = theta_star.plan(start, goal)?;
    rows.push(BenchRow {
        planner: "Theta*",
        time_us: t0.elapsed().as_micros(),
        path_length: theta_path.total_length(),
    });

    let t0 = Instant::now();
    let lpa_path = lpa_star.plan(start, goal)?;
    rows.push(BenchRow {
        planner: "LPA*",
        time_us: t0.elapsed().as_micros(),
        path_length: lpa_path.total_length(),
    });

    let t0 = Instant::now();
    let ara_path = ara_star.plan(start, goal)?;
    rows.push(BenchRow {
        planner: "ARA*",
        time_us: t0.elapsed().as_micros(),
        path_length: ara_path.total_length(),
    });

    let t0 = Instant::now();
    let fringe_path = fringe.plan(start, goal)?;
    rows.push(BenchRow {
        planner: "Fringe Search",
        time_us: t0.elapsed().as_micros(),
        path_length: fringe_path.total_length(),
    });

    let t0 = Instant::now();
    let ida_path = ida_star.plan(start, goal)?;
    rows.push(BenchRow {
        planner: "IDA*",
        time_us: t0.elapsed().as_micros(),
        path_length: ida_path.total_length(),
    });

    println!("| Planner | Time (us) | Path Length |");
    println!("|---------|-----------:|------------:|");
    for row in rows {
        println!(
            "| {} | {} | {:.3} |",
            row.planner, row.time_us, row.path_length
        );
    }

    Ok(())
}
