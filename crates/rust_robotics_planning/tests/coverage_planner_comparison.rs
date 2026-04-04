//! Coverage planner comparison: GridBasedSweep vs Wavefront vs SpiralSpanningTree
//!
//! Compares coverage percentage, path length, and computation time on
//! identical environments.

use std::collections::HashSet;
use std::time::Instant;

use rust_robotics_planning::grid_based_sweep_cpp::{
    GridBasedSweepConfig, MovingDirection, SweepDirection, plan_sweep_coverage,
};
use rust_robotics_planning::spiral_spanning_tree_cpp::{
    OccupancyGrid, SpiralSpanningTreePlanner,
};
use rust_robotics_planning::wavefront_cpp::{
    WavefrontCppConfig, WavefrontGrid, wavefront_cpp,
};

/// Create a rectangular polygon (closed) for GridBasedSweep.
fn rect_polygon(width: f64, height: f64) -> (Vec<f64>, Vec<f64>) {
    let ox = vec![0.0, width, width, 0.0, 0.0];
    let oy = vec![0.0, 0.0, height, height, 0.0];
    (ox, oy)
}

fn path_length_from_coords(x: &[f64], y: &[f64]) -> f64 {
    x.windows(2)
        .zip(y.windows(2))
        .map(|(wx, wy)| ((wx[1] - wx[0]).powi(2) + (wy[1] - wy[0]).powi(2)).sqrt())
        .sum()
}

fn path_length_from_cells(cells: &[(usize, usize)]) -> f64 {
    cells
        .windows(2)
        .map(|w| {
            let dr = w[1].0 as f64 - w[0].0 as f64;
            let dc = w[1].1 as f64 - w[0].1 as f64;
            (dr * dr + dc * dc).sqrt()
        })
        .sum()
}

struct CoverageResult {
    name: &'static str,
    cells_visited: usize,
    #[allow(dead_code)]
    total_free_cells: usize,
    coverage_pct: f64,
    path_length: f64,
    elapsed_us: u128,
}

#[test]
fn coverage_planner_comparison() {
    // Test on multiple grid sizes
    let sizes: Vec<(usize, f64)> = vec![(10, 10.0), (20, 20.0), (30, 30.0)];

    println!();
    println!(
        "{:<25} {:>6} {:>10} {:>12} {:>12} {:>10}",
        "Planner", "Size", "Visited", "Coverage%", "PathLen", "Time(µs)"
    );
    println!("{}", "-".repeat(80));

    for &(grid_size, world_size) in &sizes {
        let mut results: Vec<CoverageResult> = Vec::new();

        // Total free cells (interior of the grid)
        let total_free = grid_size * grid_size;

        // 1. GridBasedSweep
        {
            let (ox, oy) = rect_polygon(world_size, world_size);
            let config = GridBasedSweepConfig {
                resolution: 1.0,
                moving_direction: MovingDirection::Right,
                sweep_direction: SweepDirection::Up,
            };
            let t0 = Instant::now();
            let path = plan_sweep_coverage(&ox, &oy, &config);
            let elapsed = t0.elapsed().as_micros();

            let visited: HashSet<(i64, i64)> = path
                .x
                .iter()
                .zip(path.y.iter())
                .map(|(&x, &y)| (x.round() as i64, y.round() as i64))
                .collect();
            let cells_visited = visited.len();
            let coverage = cells_visited as f64 / total_free as f64 * 100.0;
            let length = path_length_from_coords(&path.x, &path.y);

            results.push(CoverageResult {
                name: "GridBasedSweep",
                cells_visited,
                total_free_cells: total_free,
                coverage_pct: coverage,
                path_length: length,
                elapsed_us: elapsed,
            });
        }

        // 2. Wavefront CPP
        {
            let grid = WavefrontGrid::new(grid_size, grid_size);
            // All cells are free by default in WavefrontGrid::new
            let start = (0, 0);
            let goal = (grid_size - 1, grid_size - 1);
            let config = WavefrontCppConfig::default();

            let t0 = Instant::now();
            let path = wavefront_cpp(&grid, start, goal, &config);
            let elapsed = t0.elapsed().as_micros();

            let visited: HashSet<(usize, usize)> = path.iter().copied().collect();
            let cells_visited = visited.len();
            let coverage = cells_visited as f64 / total_free as f64 * 100.0;
            let length = path_length_from_cells(&path);

            results.push(CoverageResult {
                name: "Wavefront",
                cells_visited,
                total_free_cells: total_free,
                coverage_pct: coverage,
                path_length: length,
                elapsed_us: elapsed,
            });
        }

        // 3. SpiralSpanningTree
        // Requires even dimensions
        {
            let even_size = if grid_size % 2 == 0 {
                grid_size
            } else {
                grid_size + 1
            };
            let occ = OccupancyGrid::all_free(even_size, even_size);
            let planner = SpiralSpanningTreePlanner::new(occ);
            let start = (0, 0); // merged node

            let t0 = Instant::now();
            let result = planner.plan(start);
            let elapsed = t0.elapsed().as_micros();

            // Count unique cells from path segments
            let mut visited_cells: HashSet<(i32, i32)> = HashSet::new();
            for seg in &result.path {
                visited_cells.insert((seg[0][0], seg[0][1]));
                visited_cells.insert((seg[1][0], seg[1][1]));
            }
            let cells_visited = visited_cells.len();
            let total = even_size * even_size;
            let coverage = cells_visited as f64 / total as f64 * 100.0;

            // Path length from route
            let length = result.route.windows(2).map(|w| {
                let dr = (w[1].0 - w[0].0) as f64;
                let dc = (w[1].1 - w[0].1) as f64;
                (dr * dr + dc * dc).sqrt()
            }).sum::<f64>();

            results.push(CoverageResult {
                name: "SpiralSTC",
                cells_visited,
                total_free_cells: total,
                coverage_pct: coverage,
                path_length: length,
                elapsed_us: elapsed,
            });
        }

        for r in &results {
            println!(
                "{:<25} {:>4}x{:<1} {:>10} {:>11.1}% {:>12.1} {:>10}",
                r.name,
                grid_size,
                grid_size,
                r.cells_visited,
                r.coverage_pct,
                r.path_length,
                r.elapsed_us,
            );
        }
        println!();

        // Basic sanity: all planners should visit at least some cells
        for r in &results {
            assert!(
                r.cells_visited > 0,
                "{} visited 0 cells on {}x{} grid",
                r.name,
                grid_size,
                grid_size
            );
        }
    }
}
